"""gui_manual_extras.py — os dois cards que o Controle Manual não tinha.

1. **Jog cartesiano com passo fixo** (0,1 / 1 / 10 mm) no frame da FERRAMENTA
   ou da BASE. Os sliders movem juntas; para encostar uma ponteira num fantoma
   o gesto natural é "desce 0,1 mm no z da ferramenta", e isso não existia.

2. **Freios por junta** (`BrakeControl`). Quando o braço para em alarme com a
   ponta encostada, soltar o freio de uma junta é o que tira a ferramenta de lá
   sem energizar nada.

O jog reaproveita o caminho que já existe: escreve nos sliders e chama
`_publish_arm_from_sliders()`. Nada de rota nova para o braço real — quem
espelha é a mesma subscrição do JTC que já atende o arrasto 3D e a palpação.
Consequência assumida: no MIRROR o braço real anda por `MovJ` (junta), não por
`RelMovLTool` (linha reta do controlador). Para 0,1 mm a diferença não existe;
para 10 mm é um arco de alguns micrometros. Em troca, o jog funciona igual em
SIM_ONLY e o número de caminhos de comando continua sendo um.
"""
from __future__ import annotations

import logging
import math as _math
import tkinter as tk
from typing import Any

import numpy as np

from .constants import ARM_JOINTS
from .gui_constants import ARM_LIMITS_DEG
from .ui_helpers import (
    BG, DANGER, FONT_LBL, FONT_MONO_S, FONT_SMALL, OK, PANEL, PRIMARY,
    PRIMARY_HV, TEXT, TEXT_DIM, TEXT_MUTED, WARN, _shade,
)

try:
    from .manip3d import ik_step as _ik_step
except Exception:  # pragma: no cover — sem numpy/manip3d a GUI ainda abre
    _ik_step: Any = None

try:
    from .kinematics import forward_kinematics as _fk
except Exception:  # pragma: no cover
    _fk: Any = None

log = logging.getLogger('touch_pack.gui_manual_extras')

# Passos oferecidos (mm). 0,1 mm é a resolução que a palpação pede perto do
# contato; 10 mm é o reposicionamento grosso. Entre os dois não falta nada que
# justifique mais um botão.
JOG_STEPS_MM = (0.1, 1.0, 10.0)

# Iterações da IK diferencial por clique. O passo máximo por iteração do
# `ik_step` é 15 mm, então 8 fecham qualquer passo desta lista com folga.
JOG_IK_ITERS = 8

# Tolerância de parada da IK. A padrão do `ik_step` é 0,2 mm — feita para o
# arrasto contínuo, onde o quadro seguinte corrige o resto. Aqui ela arruinaria
# os dois extremos da lista de passos: o de 0,1 mm nasceria "já convergido" e
# não moveria NADA, e o de 10 mm pararia 0,25 mm curto. 5 µm fecha em 4
# iterações e fica uma ordem de grandeza abaixo da repetibilidade do CR10
# (±0,03 mm), que é o piso físico do que adianta perseguir.
JOG_IK_TOL_M = 5e-6


class ManualExtrasMixin:
    """Jog cartesiano e freios por junta."""

    def _build_manual_extras(self, parent: tk.Frame) -> None:
        self._build_cart_jog_card(parent)
        self._build_brake_card(parent)

    # Jog cartesiano
    def _build_cart_jog_card(self, parent: tk.Frame) -> None:
        card = self._card(parent, 'Cartesian jog — TCP', expand=False)

        self.jog_step_mm = tk.DoubleVar(value=1.0)
        self.jog_frame_var = tk.StringVar(value='base')

        row = tk.Frame(card, bg=PANEL)
        row.pack(fill='x', pady=(0, 8))
        tk.Label(row, text='Step', bg=PANEL, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(side='left', padx=(0, 6))
        for mm in JOG_STEPS_MM:
            tk.Radiobutton(row, text=f'{mm:g} mm', value=mm,
                           variable=self.jog_step_mm, bg=PANEL, fg=TEXT,
                           selectcolor=PANEL, activebackground=PANEL,
                           activeforeground=TEXT, font=FONT_SMALL,
                           highlightthickness=0, bd=0
                           ).pack(side='left', padx=(0, 4))
        tk.Label(row, text='  Frame', bg=PANEL, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(side='left', padx=(10, 6))
        for val, txt in (('tool', 'Tool'), ('base', 'Base')):
            tk.Radiobutton(row, text=txt, value=val,
                           variable=self.jog_frame_var, bg=PANEL, fg=TEXT,
                           selectcolor=PANEL, activebackground=PANEL,
                           activeforeground=TEXT, font=FONT_SMALL,
                           highlightthickness=0, bd=0
                           ).pack(side='left', padx=(0, 4))

        grid = tk.Frame(card, bg=PANEL)
        grid.pack(fill='x')
        for col, axis in enumerate('XYZ'):
            tk.Label(grid, text=axis, bg=PANEL, fg=TEXT_MUTED,
                     font=FONT_SMALL).grid(row=0, column=col, pady=(0, 2))
            for r, sign in ((1, +1.0), (2, -1.0)):
                tk.Button(grid, text=f'{axis}{"+" if sign > 0 else "−"}',
                          command=lambda a=col, s=sign: self._cart_jog(a, s),
                          bg=PRIMARY, fg='white', activebackground=PRIMARY_HV,
                          activeforeground='white', font=FONT_LBL,
                          relief='flat', bd=0, padx=18, pady=6, cursor='hand2'
                          ).grid(row=r, column=col, padx=3, pady=2, sticky='ew')
            grid.columnconfigure(col, weight=1)

        self._jog_lbl = tk.Label(
            card, text='TCP —', bg=PANEL, fg=TEXT_DIM, font=FONT_MONO_S,
            anchor='w', justify='left')
        self._jog_lbl.pack(fill='x', pady=(8, 0))

    def _cart_jog(self, axis: int, sign: float) -> None:
        """Anda `jog_step_mm` no eixo pedido, mantendo a atitude do TCP."""
        if _ik_step is None or _fk is None:
            self._set_status('kinematics unavailable — Cartesian jog off.',
                             DANGER)
            return
        reason = self._manip_gate_reason()
        if reason:
            self._set_status(f'Cartesian jog blocked: {reason}.', WARN)
            return
        try:
            q = np.array([_math.radians(float(self.arm_sliders[j].get()))
                          for j in ARM_JOINTS])
            step_m = float(self.jog_step_mm.get()) * 1e-3
        except (ValueError, tk.TclError):
            self._set_status('Invalid sliders.', DANGER)
            return

        T_end = self._manip_T_end()
        T_now = _fk(q, T_end=T_end)
        e = np.zeros(3)
        e[axis] = sign
        # No frame da ferramenta o eixo pedido é o eixo do TCP levado ao mundo;
        # na base é o eixo do mundo mesmo.
        d = (T_now[:3, :3] @ e) if self.jog_frame_var.get() == 'tool' else e
        p_target = T_now[:3, 3] + d * step_m

        # R_lock = atitude ATUAL: o jog move o ponto, o pulso segura a pose.
        # Sem isso, um passo em z da ferramenta viria acompanhado de rotação
        # e a ponta entraria torta no fantoma.
        res = None
        for _ in range(JOG_IK_ITERS):
            res = _ik_step(q, p_target, T_end=T_end, R_lock=T_now[:3, :3],
                           tol_pos_m=JOG_IK_TOL_M)
            q = res.q
        if res is None:
            return

        lo_hi = [ARM_LIMITS_DEG[j] for j in ARM_JOINTS]
        graus = [_math.degrees(v) for v in q[:6]]
        if any(not (lo <= g <= hi) for g, (lo, hi) in zip(graus, lo_hi)):
            self._set_status(
                'Cartesian jog refused: the step would leave a joint limit.',
                WARN)
            return

        with self._suppress():
            for j, g in zip(ARM_JOINTS, graus):
                self.arm_sliders[j].set(round(g, 5))
        self._publish_arm_from_sliders()

        p_mm = _fk(q, T_end=T_end)[:3, 3] * 1e3
        erro_mm = float(res.pos_err_m) * 1e3
        frame_txt = self.jog_frame_var.get()
        self._jog_lbl.config(
            text=f'TCP  x={p_mm[0]:+8.2f}  y={p_mm[1]:+8.2f}  z={p_mm[2]:+8.2f} mm'
                 f'   [{frame_txt}]  residual {erro_mm:.3f} mm',
            fg=WARN if (res.singular or erro_mm > 0.05) else TEXT_DIM)
        if res.singular:
            self._set_status(
                'Cartesian jog: singular Jacobian — the step was truncated.',
                WARN)

    # Freios por junta
    def _build_brake_card(self, parent: tk.Frame) -> None:
        card = self._card(parent, 'Joint brakes (arm DISABLED only)',
                          expand=False)
        tk.Label(card,
                 text=('Hold the joint before releasing its brake — it drops '
                       'under its own weight. The controller refuses this '
                       'while the arm is enabled.'),
                 bg=PANEL, fg=WARN, font=FONT_SMALL, anchor='w',
                 wraplength=430, justify='left').pack(fill='x')
        grid = tk.Frame(card, bg=PANEL)
        grid.pack(fill='x', pady=(8, 0))
        for i in range(6):
            axis = i + 1
            tk.Label(grid, text=f'J{axis}', bg=PANEL, fg=TEXT_MUTED,
                     font=FONT_SMALL).grid(row=0, column=i, pady=(0, 2))
            tk.Button(grid, text='release',
                      command=lambda a=axis: self._brake_set(a, True),
                      bg=DANGER, fg='white',
                      activebackground=_shade(DANGER, -0.08),
                      activeforeground='white', font=FONT_SMALL,
                      relief='flat', bd=0, padx=4, pady=5, cursor='hand2'
                      ).grid(row=1, column=i, padx=2, sticky='ew')
            tk.Button(grid, text='lock',
                      command=lambda a=axis: self._brake_set(a, False),
                      bg=BG, fg=TEXT, activebackground=_shade(BG, -0.08),
                      activeforeground=TEXT, font=FONT_SMALL,
                      relief='flat', bd=0, padx=4, pady=5, cursor='hand2'
                      ).grid(row=2, column=i, padx=2, pady=(2, 0), sticky='ew')
            grid.columnconfigure(i, weight=1)

    def _brake_set(self, axis: int, released: bool) -> None:
        if not self._robot_connected or self._real_driver is None:
            self._set_status('Connect the CR10 to control the brakes.', WARN)
            return
        try:
            self._real_driver.brake(axis, released)
        except Exception as exc:
            self._set_status(f'J{axis} brake: {exc}', DANGER)
            return
        self._set_status(
            f'J{axis} brake {"RELEASED — hold the joint" if released else "locked"}.',
            DANGER if released else OK)
