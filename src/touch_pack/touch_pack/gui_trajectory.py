"""gui_trajectory.py — gravação do arrasto como trajetória reproduzível.

O DobotStudio Pro grava o caminho inteiro do arrasto manual e o reproduz com
razão de velocidade. Aqui a gravação vira **waypoints densos dentro de um
Movimento** da própria aba Poses & Movimentos: quem reproduz é o executor que
já existe (interpola no Gazebo de uma vez e cadencia `MovJ` no braço real), com
a velocidade e a duração por trecho que ele já expõe.

O que se perde em relação ao fabricante: isto são waypoints, não um traço
contínuo — entre dois pontos vale a interpolação do controlador, não a curva
exata da sua mão. `TRAJ_TOL_DEG` é o botão que governa essa fidelidade. O que
se ganha: nenhum motor de reprodução paralelo, nenhum caminho novo até o braço
real, e a trajetória gravada é editável pose a pose como qualquer outra.

A amostragem roda no laço do Tk, que é onde `_latest_joint_rad` já chega tanto
do Gazebo quanto do feedback do braço real em MIRROR — de onde vem o arrasto
que se quer gravar.
"""
from __future__ import annotations

import math
import tkinter as tk

from .ui_helpers import (
    DANGER, FONT_LBL, FONT_MONO_S, FONT_SMALL, OK, PANEL, PRIMARY, TEXT_DIM,
    TEXT_MUTED, WARN, _shade,
)

# Período de amostragem (ms) e o quanto uma junta precisa andar para a pose
# virar waypoint. 2° a 200 ms segue um arrasto de mão sem encher o JSON de
# poses idênticas paradas.
TRAJ_SAMPLE_MS = 200
TRAJ_TOL_DEG = 2.0

# Teto de waypoints por gravação. Numa cadência de MovJ por pose, 300 poses já
# são vários minutos de reprodução — acima disso o que se gravou não foi um
# caminho, foi um esquecimento de apertar Stop.
TRAJ_MAX_POINTS = 300


def should_keep(q_prev_deg, q_new_deg, tol_deg: float = TRAJ_TOL_DEG) -> bool:
    """True se a pose nova merece virar waypoint.

    Primeira pose (`q_prev_deg` None) sempre entra: é o começo do caminho.
    """
    if q_prev_deg is None:
        return True
    return any(abs(float(a) - float(b)) >= tol_deg
               for a, b in zip(q_new_deg, q_prev_deg))


class TrajRecMixin:
    """Grava o arrasto num Movimento; a reprodução é a que já existia."""

    def _build_traj_card(self, parent: tk.Frame) -> None:
        card = self._card(parent, 'Record drag as a Motion', expand=False)
        tk.Label(card,
                 text=('Release the arm with Drag Teach, press Record and move '
                       'it by hand. Each waypoint becomes a pose; play it back '
                       'with Run on the Motion.'),
                 bg=PANEL, fg=TEXT_MUTED, font=FONT_SMALL, anchor='w',
                 wraplength=420, justify='left').pack(fill='x')

        self._traj_recording = False
        self._traj_samples: list[list[float]] = []
        self._traj_after_id = None

        btns = tk.Frame(card, bg=PANEL)
        btns.pack(fill='x', pady=(8, 0))
        self._traj_btn = tk.Button(
            btns, text='⏺  Record', command=self._traj_toggle,
            bg=PRIMARY, fg='white', activebackground=_shade(PRIMARY, -0.08),
            activeforeground='white', font=FONT_LBL, relief='flat', bd=0,
            padx=14, pady=7, cursor='hand2')
        self._traj_btn.pack(side='left', fill='x', expand=True)

        self._traj_lbl = tk.Label(card, text='', bg=PANEL, fg=TEXT_DIM,
                                  font=FONT_MONO_S, anchor='w')
        self._traj_lbl.pack(fill='x', pady=(8, 0))

    def _traj_toggle(self) -> None:
        if self._traj_recording:
            self._traj_stop()
        else:
            self._traj_start()

    def _traj_start(self) -> None:
        if self._latest_joint_rad is None:
            self._set_status(
                'No joint feedback yet — start the sim or connect the arm.',
                WARN)
            return
        self._traj_samples = []
        self._traj_recording = True
        self._traj_btn.config(text='⏹  Stop', bg=DANGER,
                              activebackground=_shade(DANGER, -0.08))
        self._set_status('Recording — drag the arm.', OK)
        self._traj_tick()

    def _traj_tick(self) -> None:
        if not self._traj_recording:
            return
        q = self._latest_joint_rad
        if q is not None and len(q) >= 6:
            q_deg = [math.degrees(float(v)) for v in q[:6]]
            anterior = self._traj_samples[-1] if self._traj_samples else None
            if should_keep(anterior, q_deg):
                self._traj_samples.append(q_deg)
                self._traj_lbl.config(
                    text=f'{len(self._traj_samples)} waypoint(s)')
        if len(self._traj_samples) >= TRAJ_MAX_POINTS:
            self._set_status(
                f'Recording stopped at the {TRAJ_MAX_POINTS}-waypoint ceiling.',
                WARN)
            self._traj_stop()
            return
        self._traj_after_id = self.root.after(TRAJ_SAMPLE_MS, self._traj_tick)

    def _traj_stop(self) -> None:
        self._traj_recording = False
        if self._traj_after_id is not None:
            try:
                self.root.after_cancel(self._traj_after_id)
            except (tk.TclError, ValueError):
                pass
            self._traj_after_id = None
        self._traj_btn.config(text='⏺  Record', bg=PRIMARY,
                              activebackground=_shade(PRIMARY, -0.08))

        if len(self._traj_samples) < 2:
            self._set_status(
                'Nothing recorded — the arm did not move enough.', WARN)
            return

        name = self._ask_name_dialog(
            'Save trajectory', f'Trajetória {self._next_movement_id}')
        if name is None:
            self._set_status('Recording discarded.', WARN)
            return

        # Poses gravadas em lote: `_add_pose` salva o JSON e escreve na barra
        # de status a cada chamada, e isso são dezenas de gravações de arquivo
        # para um único caminho.
        pose_ids = []
        for q_deg in self._traj_samples:
            pid = self._next_pose_id
            self._next_pose_id += 1
            self._poses.append({'id': pid, 'name': f'{name} {pid}',
                                'q_deg': [round(v, 2) for v in q_deg]})
            pose_ids.append(pid)

        mid = self._next_movement_id
        self._next_movement_id += 1
        # dur_s curto: os waypoints estão a ~2° um do outro, não a meia
        # estação de distância como numa sequência ensinada à mão.
        mov = {'id': mid, 'name': name, 'pose_ids': pose_ids,
               'speed_pct': 10, 'dur_s': 0.5}
        self._movements.append(mov)
        self._save_poses_data()
        self._refresh_poses_list()
        self._refresh_movements_list(select_id=mid)
        self._refresh_movement_detail(mov)
        self._traj_lbl.config(text=f'saved: {len(pose_ids)} waypoints')
        self._set_status(
            f'Motion "{name}" recorded with {len(pose_ids)} waypoints — '
            f'adjust the speed and press Run to play it back.', OK)
