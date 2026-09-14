"""gui_poses.py — poses guardadas e as sequências que as encadeiam.

Uma pose é braço + mão capturados num instante; um movimento é uma lista
ordenada delas que o braço executa em sequência, opcionalmente em laço.

O que carrega risco aqui não é o CRUD, é a execução: ela roda numa thread
própria, move o braço REAL, e pode ser interrompida a qualquer momento. Duas
decisões sustentam isso — o worker é o único dono do `_exec_movement_id` (e o
`finally` dele é quem limpa), e parar manda `Halt()` ao braço antes de dizer
ao operador que parou.

Recortado de `palpation_gui.py` — os métodos operam sobre `self` como antes.
"""
from __future__ import annotations

import logging

# `urdf_to_dobot` vem do mesmo import opcional do driver: converter a pose só
# faz sentido com o braço real no ar, e a GUI abre sem ele.
try:
    from .kinematics import urdf_to_dobot as _urdf_to_dobot
except Exception:  # pragma: no cover
    _urdf_to_dobot = None

import json
import math
import numpy as np
import os
import threading
import time
import tkinter as tk
from .constants import (
    ARM_JOINTS,
    HAND_JOINTS,
    POSES_FILE,
    tool_stamp,
    tool_stamp_mismatch,
)
from .ui_helpers import (
    BG,
    BORDER,
    BTN_NEUTRAL,
    DANGER,
    DANGER_HV,
    FONT_HEAD,
    FONT_LBL,
    FONT_MONO_S,
    FONT_SMALL,
    OK,
    PANEL,
    PRIMARY,
    PRIMARY_HV,
    TEXT,
    TEXT_MUTED,
    WARN,
    _shade,
)
from builtin_interfaces.msg import Duration
from tkinter import ttk
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


log = logging.getLogger('touch_pack.palpation_gui')   # mesmo canal do host


class PosesMixin:
    """gui_poses.py — poses guardadas e as sequências que as encadeiam."""

    def _build_poses_tab(self, root: tk.Frame) -> None:
        """Layout dois-colunas: esquerda=Poses (fixa 310px), direita=Movimentos."""
        left = tk.Frame(root, bg=BG, width=310)
        left.pack(side='left', fill='y', padx=(12, 6), pady=12)
        left.pack_propagate(False)

        right = tk.Frame(root, bg=BG)
        right.pack(side='left', fill='both', expand=True, padx=(6, 12), pady=12)

        tk.Label(left, text='Poses', bg=BG, fg=TEXT, font=FONT_HEAD).pack(anchor='w')
        tk.Frame(left, bg=BORDER, height=1).pack(fill='x', pady=(4, 8))

        btn_row = tk.Frame(left, bg=BG)
        btn_row.pack(fill='x', pady=(0, 8))

        self._drag_btn = tk.Button(
            btn_row, text='✋ Drag OFF',
            command=self._toggle_drag,
            bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=_shade(BTN_NEUTRAL, -0.08),
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2')
        self._drag_btn.pack(side='left', padx=(0, 4))

        tk.Button(
            btn_row, text='◉ Robot',
            command=self._capture_pose_robot,
            bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=_shade(BTN_NEUTRAL, -0.08),
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2').pack(side='left', padx=(0, 4))

        tk.Button(
            btn_row, text='⌨ Sim',
            command=self._capture_pose_sim,
            bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=_shade(BTN_NEUTRAL, -0.08),
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2').pack(side='left')

        lbx_frame = tk.Frame(left, bg=BG)
        lbx_frame.pack(fill='both', expand=True)

        p_scroll = ttk.Scrollbar(lbx_frame, orient='vertical')
        p_scroll.pack(side='right', fill='y')

        self._poses_lbx = tk.Listbox(
            lbx_frame, yscrollcommand=p_scroll.set,
            bg=PANEL, fg=TEXT, font=FONT_MONO_S,
            selectbackground=PRIMARY, selectforeground='white',
            relief='flat', bd=0, highlightthickness=1,
            highlightbackground=BORDER, activestyle='none')
        self._poses_lbx.pack(side='left', fill='both', expand=True)
        p_scroll.config(command=self._poses_lbx.yview)

        pose_act = tk.Frame(left, bg=BG)
        pose_act.pack(fill='x', pady=(8, 0))

        tk.Button(
            pose_act, text='✏ Rename',
            command=self._rename_selected_pose,
            bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=_shade(BTN_NEUTRAL, -0.08),
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2').pack(side='left', padx=(0, 4))

        tk.Button(
            pose_act, text='✖ Delete',
            command=self._delete_selected_pose,
            bg=DANGER, fg='white',
            activebackground=DANGER_HV,
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2').pack(side='left')

        # RIGHT: Movimentos
        mov_hdr = tk.Frame(right, bg=BG)
        mov_hdr.pack(fill='x')

        tk.Label(mov_hdr, text='Motions', bg=BG, fg=TEXT,
                 font=FONT_HEAD).pack(side='left', anchor='w')

        tk.Button(
            mov_hdr, text='+ New',
            command=self._new_movement,
            bg=PRIMARY, fg='white',
            activebackground=PRIMARY_HV,
            font=FONT_SMALL, relief='flat', bd=0, padx=10, pady=4,
            cursor='hand2').pack(side='right')

        tk.Frame(right, bg=BORDER, height=1).pack(fill='x', pady=(4, 8))

        mov_lbx_frame = tk.Frame(right, bg=BG, height=120)
        mov_lbx_frame.pack(fill='x')
        mov_lbx_frame.pack_propagate(False)

        m_scroll = ttk.Scrollbar(mov_lbx_frame, orient='vertical')
        m_scroll.pack(side='right', fill='y')

        self._movs_lbx = tk.Listbox(
            mov_lbx_frame, yscrollcommand=m_scroll.set,
            bg=PANEL, fg=TEXT, font=FONT_MONO_S,
            selectbackground=PRIMARY, selectforeground='white',
            relief='flat', bd=0, highlightthickness=1,
            highlightbackground=BORDER, activestyle='none')
        self._movs_lbx.pack(side='left', fill='both', expand=True)
        m_scroll.config(command=self._movs_lbx.yview)
        self._movs_lbx.bind('<<ListboxSelect>>', self._on_movement_select)

        self._mov_detail_outer = tk.Frame(right, bg=BG)
        self._mov_detail_outer.pack(fill='both', expand=True, pady=(8, 0))

        self._refresh_poses_list()
        self._refresh_movements_list()
    # Dados: load / save
    def _load_poses_data(self) -> None:
        try:
            with open(POSES_FILE) as f:
                data = json.load(f)
            self._poses = data.get('poses', [])
            self._movements = data.get('movements', [])
            self._next_pose_id = max((p['id'] for p in self._poses), default=0) + 1
            self._next_movement_id = max(
                (m['id'] for m in self._movements), default=0) + 1
            if self._poses or self._movements:
                aviso = tool_stamp_mismatch(data, what='o poses.json')
                if aviso:
                    self.get_logger().warn(f'[FERRAMENTA] {aviso}')
        except (FileNotFoundError, json.JSONDecodeError, KeyError):
            self._poses = []
            self._movements = []
            self._next_pose_id = 1
            self._next_movement_id = 1
    def _save_poses_data(self) -> None:
        os.makedirs(os.path.dirname(POSES_FILE), exist_ok=True)
        with open(POSES_FILE, 'w') as f:
            json.dump({'poses': self._poses, 'movements': self._movements,
                       **tool_stamp()}, f, indent=2)
    # Lookup helpers
    def _pose_by_id(self, pid: int) -> dict | None:
        for p in self._poses:
            if p['id'] == pid:
                return p
        return None
    def _movement_by_id(self, mid: int) -> dict | None:
        for m in self._movements:
            if m['id'] == mid:
                return m
        return None
    def _pose_label(self, p: dict) -> str:
        q = p['q_deg']
        parts = ' '.join(f'J{i + 1}={v:+.0f}°' for i, v in enumerate(q))
        hand_marker = '  [Hand]' if p.get('hand_deg') else ''
        return f"{p['name']}{hand_marker}  [{parts}]"
    # Refresh widgets
    def _refresh_poses_list(self) -> None:
        lbx = self._poses_lbx
        if lbx is None:
            return
        lbx.delete(0, 'end')
        for p in self._poses:
            lbx.insert('end', self._pose_label(p))
    def _refresh_movements_list(self, select_id: int | None = None) -> None:
        lbx = self._movs_lbx
        if lbx is None:
            return
        lbx.delete(0, 'end')
        for m in self._movements:
            lbx.insert('end', m['name'])
        if select_id is not None:
            for i, m in enumerate(self._movements):
                if m['id'] == select_id:
                    lbx.selection_set(i)
                    lbx.see(i)
                    break
    def _on_movement_select(self, _event=None) -> None:
        lbx = self._movs_lbx
        if lbx is None:
            return
        sel = lbx.curselection()
        if not sel:
            return
        self._refresh_movement_detail(self._movements[sel[0]])
    def _refresh_movement_detail(self, mov: dict) -> None:
        outer = self._mov_detail_outer
        if outer is None:
            return
        if self._mov_detail_inner is not None:
            self._mov_detail_inner.destroy()

        inner = tk.Frame(outer, bg=PANEL,
                         highlightthickness=1, highlightbackground=BORDER)
        inner.pack(fill='both', expand=True)
        self._mov_detail_inner = inner

        # Header
        hdr = tk.Frame(inner, bg=PANEL)
        hdr.pack(fill='x', padx=12, pady=(10, 4))
        tk.Label(hdr, text=mov['name'], bg=PANEL, fg=TEXT,
                 font=FONT_HEAD).pack(side='left')
        tk.Button(hdr, text='✏',
                  command=lambda: self._rename_movement(mov['id']),
                  bg=BTN_NEUTRAL, fg=TEXT,
                  font=FONT_SMALL, relief='flat', bd=0,
                  padx=6, pady=2, cursor='hand2').pack(side='left', padx=(8, 0))
        tk.Button(hdr, text='✖ Delete',
                  command=lambda: self._delete_movement(mov['id']),
                  bg=DANGER, fg='white', activebackground=DANGER_HV,
                  font=FONT_SMALL, relief='flat', bd=0,
                  padx=8, pady=2, cursor='hand2').pack(side='right')
        tk.Frame(inner, bg=BORDER, height=1).pack(fill='x')

        body = tk.Frame(inner, bg=PANEL)
        body.pack(fill='both', expand=True, padx=12, pady=8)

        # Sequência
        seq_col = tk.Frame(body, bg=PANEL)
        seq_col.pack(side='left', fill='both', expand=True, padx=(0, 12))

        tk.Label(seq_col, text='Pose Sequence', bg=PANEL, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(anchor='w')

        seq_frame = tk.Frame(seq_col, bg=PANEL)
        seq_frame.pack(fill='both', expand=True, pady=(4, 0))

        seq_sb = ttk.Scrollbar(seq_frame, orient='vertical')
        seq_sb.pack(side='right', fill='y')

        seq_lbx = tk.Listbox(
            seq_frame, yscrollcommand=seq_sb.set,
            bg=BG, fg=TEXT, font=FONT_MONO_S,
            selectbackground=PRIMARY, selectforeground='white',
            relief='flat', bd=0, highlightthickness=0,
            activestyle='none', height=6)
        seq_lbx.pack(side='left', fill='both', expand=True)
        seq_sb.config(command=seq_lbx.yview)

        def _refresh_seq():
            seq_lbx.delete(0, 'end')
            for pid in mov['pose_ids']:
                p = self._pose_by_id(pid)
                if p is None:
                    seq_lbx.insert('end', f'[deletada:{pid}]')
                else:
                    hand_tag = ' [Hand]' if p.get('hand_deg') else ''
                    seq_lbx.insert('end', f"{p['name']}{hand_tag}")

        _refresh_seq()

        def _add_pose_to_seq():
            lbx = self._poses_lbx
            if lbx is None:
                return
            sel = lbx.curselection()
            if not sel:
                self._set_status('Select a pose in the list on the left.', WARN)
                return
            mov['pose_ids'].append(self._poses[sel[0]]['id'])
            _refresh_seq()
            self._save_poses_data()

        def _remove_pose_from_seq():
            sel = seq_lbx.curselection()
            if not sel:
                return
            idx = sel[0]
            if 0 <= idx < len(mov['pose_ids']):
                del mov['pose_ids'][idx]
                _refresh_seq()
                self._save_poses_data()

        def _move_up():
            sel = seq_lbx.curselection()
            if not sel:
                return
            i = sel[0]
            if i > 0:
                mov['pose_ids'][i - 1], mov['pose_ids'][i] = \
                    mov['pose_ids'][i], mov['pose_ids'][i - 1]
                _refresh_seq()
                seq_lbx.selection_set(i - 1)
                self._save_poses_data()

        def _move_down():
            sel = seq_lbx.curselection()
            if not sel:
                return
            i = sel[0]
            if i < len(mov['pose_ids']) - 1:
                mov['pose_ids'][i], mov['pose_ids'][i + 1] = \
                    mov['pose_ids'][i + 1], mov['pose_ids'][i]
                _refresh_seq()
                seq_lbx.selection_set(i + 1)
                self._save_poses_data()

        seq_btns = tk.Frame(seq_col, bg=PANEL)
        seq_btns.pack(fill='x', pady=(6, 0))

        for txt, cmd in [('+ Adicionar', _add_pose_to_seq),
                          ('−', _remove_pose_from_seq),
                          ('↑', _move_up),
                          ('↓', _move_down)]:
            tk.Button(seq_btns, text=txt, command=cmd,
                      bg=BTN_NEUTRAL, fg=TEXT,
                      activebackground=_shade(BTN_NEUTRAL, -0.08),
                      font=FONT_SMALL, relief='flat', bd=0,
                      padx=8, pady=3, cursor='hand2').pack(side='left', padx=(0, 4))

        # Controles + Execução
        ctrl_col = tk.Frame(body, bg=PANEL, width=190)
        ctrl_col.pack(side='left', fill='y')
        ctrl_col.pack_propagate(False)

        tk.Label(ctrl_col, text='Speed (%)', bg=PANEL, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(anchor='w')
        spd_var = tk.IntVar(value=mov.get('speed_pct', 10))

        def _on_spd(*_):
            try:
                v = max(1, min(100, int(spd_var.get())))
                mov['speed_pct'] = v
                self._save_poses_data()
            except (ValueError, tk.TclError):
                pass

        tk.Spinbox(ctrl_col, from_=1, to=100, textvariable=spd_var,
                   width=7, font=FONT_MONO_S, relief='flat', bd=1,
                   command=_on_spd).pack(anchor='w', pady=(0, 10))
        spd_var.trace_add('write', _on_spd)

        tk.Label(ctrl_col, text='Duration/step (s)', bg=PANEL, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(anchor='w')
        dur_var = tk.DoubleVar(value=mov.get('dur_s', 2.0))

        def _on_dur(*_):
            try:
                v = max(0.1, float(dur_var.get()))
                mov['dur_s'] = round(v, 2)
                self._save_poses_data()
            except (ValueError, tk.TclError):
                pass

        tk.Spinbox(ctrl_col, from_=0.1, to=60.0, increment=0.5,
                   textvariable=dur_var, width=7, format='%.1f',
                   font=FONT_MONO_S, relief='flat', bd=1,
                   command=_on_dur).pack(anchor='w', pady=(0, 16))
        dur_var.trace_add('write', _on_dur)

        _mid = mov['id']
        tk.Button(ctrl_col, text='▶ Run',
                  command=lambda: self._start_movement(_mid, loop=False),
                  bg=OK, fg='white', activebackground='#15803d',
                  font=FONT_SMALL, relief='flat', bd=0,
                  padx=8, pady=4, cursor='hand2').pack(fill='x', pady=(0, 4))
        tk.Button(ctrl_col, text='↻ Loop',
                  command=lambda: self._start_movement(_mid, loop=True),
                  bg=WARN, fg='white', activebackground='#b45309',
                  font=FONT_SMALL, relief='flat', bd=0,
                  padx=8, pady=4, cursor='hand2').pack(fill='x', pady=(0, 4))

        tk.Button(ctrl_col, text='■ Stop',
                  command=self._stop_execution,
                  bg=DANGER, fg='white', activebackground=DANGER_HV,
                  font=FONT_SMALL, relief='flat', bd=0,
                  padx=8, pady=4, cursor='hand2').pack(fill='x')
    # Captura de poses
    def _capture_hand_from_sliders(self) -> dict | None:
        """Retorna {junta: graus} dos sliders da mão se disponíveis, else None."""
        sliders = getattr(self, 'hand_sliders', None)
        if not sliders:
            return None
        return {j: float(sliders[j].get()) for j in HAND_JOINTS}
    def _capture_pose_robot(self) -> None:
        drv = self._real_driver
        if drv is None or not self._robot_connected:
            self._set_status('Real robot not connected — use ⌨ Sim.', WARN)
            return
        try:
            q_urdf = drv.read_joints_urdf()
            q_deg = [math.degrees(float(v)) for v in q_urdf]
            hand_deg = self._capture_hand_from_sliders()
            self._add_pose(q_deg, prefix='Robot', hand_deg=hand_deg)
        except Exception as exc:
            self._set_status(f'Error capturing real pose: {exc}', DANGER)
    def _capture_pose_sim(self) -> None:
        positions = self._latest_joint_rad
        if positions is None:
            self._set_status('No /joint_states reading — start the simulation.', WARN)
            return
        q_deg = [math.degrees(float(v)) for v in positions]
        hand_deg = self._capture_hand_from_sliders()
        self._add_pose(q_deg, prefix='Sim', hand_deg=hand_deg)
    def _add_pose(self, q_deg: list, prefix: str = 'Pose',
                  hand_deg: dict | None = None,
                  hand_eci_id: int | None = None) -> None:
        pid = self._next_pose_id
        self._next_pose_id += 1
        name = f'{prefix} {pid}'
        pose: dict = {'id': pid, 'name': name,
                      'q_deg': [round(float(v), 2) for v in q_deg[:6]]}
        if hand_deg is not None:
            pose['hand_deg'] = {j: round(float(hand_deg.get(j, 0)), 2)
                                for j in HAND_JOINTS}
            pose['hand_eci_id'] = hand_eci_id
        self._poses.append(pose)
        self._save_poses_data()
        self._refresh_poses_list()
        hand_info = '  + COVVI Hand' if hand_deg else ''
        self._set_status(f'Pose "{name}" captured{hand_info}.', OK)
    # Ações nas poses
    def _rename_selected_pose(self) -> None:
        lbx = self._poses_lbx
        if lbx is None:
            return
        sel = lbx.curselection()
        if not sel:
            self._set_status('Select a pose to rename.', WARN)
            return
        pose = self._poses[sel[0]]
        new_name = self._ask_name_dialog('Rename Pose', pose['name'])
        if new_name:
            pose['name'] = new_name
            self._save_poses_data()
            self._refresh_poses_list()
    def _delete_selected_pose(self) -> None:
        lbx = self._poses_lbx
        if lbx is None:
            return
        sel = lbx.curselection()
        if not sel:
            self._set_status('Select a pose to delete.', WARN)
            return
        pose = self._poses[sel[0]]
        pid = pose['id']
        for m in self._movements:
            m['pose_ids'] = [x for x in m['pose_ids'] if x != pid]
        self._poses.pop(sel[0])
        self._save_poses_data()
        self._refresh_poses_list()
        self._set_status(f'Pose "{pose["name"]}" deleted.', OK)
    # Ações nos movimentos
    def _new_movement(self) -> None:
        name = self._ask_name_dialog(
            'New Motion', f'Movimento {self._next_movement_id}')
        if name is None:
            return
        mid = self._next_movement_id
        self._next_movement_id += 1
        mov = {'id': mid, 'name': name, 'pose_ids': [],
               'speed_pct': 10, 'dur_s': 2.0}
        self._movements.append(mov)
        self._save_poses_data()
        self._refresh_movements_list(select_id=mid)
        self._refresh_movement_detail(mov)
    def _rename_movement(self, mov_id: int) -> None:
        mov = self._movement_by_id(mov_id)
        if mov is None:
            return
        new_name = self._ask_name_dialog('Rename Motion', mov['name'])
        if new_name:
            mov['name'] = new_name
            self._save_poses_data()
            self._refresh_movements_list(select_id=mov_id)
            self._refresh_movement_detail(mov)
    def _delete_movement(self, mov_id: int) -> None:
        mov = self._movement_by_id(mov_id)
        if mov is None:
            return
        name = mov['name']
        self._movements = [m for m in self._movements if m['id'] != mov_id]
        self._save_poses_data()
        self._refresh_movements_list()
        if self._mov_detail_inner is not None:
            self._mov_detail_inner.destroy()
            self._mov_detail_inner = None
        self._set_status(f'Motion "{name}" deleted.', OK)
    # Execução de movimentos
    def _start_movement(self, mov_id: int, loop: bool = False) -> None:
        if self._exec_thread is not None and self._exec_thread.is_alive():
            self._set_status('Execution in progress — stop first.', WARN)
            return
        mov = self._movement_by_id(mov_id)
        if mov is None:
            return
        if not mov['pose_ids']:
            self._set_status('Add poses to the sequence before running.', WARN)
            return
        self._exec_stop.clear()
        self._exec_movement_id = mov_id
        self._exec_thread = threading.Thread(
            target=self._execute_movement_worker,
            args=(dict(mov), loop),
            daemon=True, name='exec-movement')
        self._exec_thread.start()
        suffix = '  (loop)' if loop else ''
        self._set_status(f'Running "{mov["name"]}"{suffix}...', OK)
    def _stop_execution(self) -> None:
        self._exec_stop.set()
        # Não limpa _exec_movement_id aqui — o finally do worker faz isso.
        if (self._robot_mode == 'MIRROR' and self._robot_connected
                and self._real_driver is not None):
            try:
                self._real_driver.halt()
            except Exception as exc:
                # Engolir aqui dizia "Execution stopped." com o braço real
                # possivelmente ainda em movimento. Mesmo tratamento de
                # _on_stop_palpation: o operador precisa saber.
                self.get_logger().warning(f'Halt após stop falhou: {exc}')
                self._set_status(
                    f'Stop sent, but Halt() failed: {exc}', DANGER)
                return
        self._set_status('Execution stopped.', WARN)
    def _execute_movement_worker(self, mov: dict, loop: bool) -> None:
        try:
            self._run_movement_once(mov)
            while loop and not self._exec_stop.is_set():
                self._run_movement_once(mov)
        except Exception as exc:
            log.warning('Execução de movimento falhou: %s', exc)
            # `exc` é APAGADO ao sair do handler (PEP 3110), e este callback
            # roda depois — capturar por default do lambda é o que salva a
            # mensagem.
            self.root.after(
                0, lambda e=str(exc): self._set_status(
                    f'Execution failed: {e}', DANGER))
        finally:
            self._exec_movement_id = None
    def _run_movement_once(self, mov: dict) -> None:
        """Executa uma passagem completa pelo movimento."""
        dur_s = max(0.1, mov.get('dur_s', 2.0))
        speed_pct = max(1, min(100, mov.get('speed_pct', 10)))
        poses = [self._pose_by_id(pid) for pid in mov['pose_ids']]
        poses = [p for p in poses if p is not None]
        if not poses:
            return

        mode = self._robot_mode

        if mode in ('SIM_ONLY', 'MIRROR'):
            # Publica trajetória completa no Gazebo de uma vez.
            msg = JointTrajectory()
            msg.joint_names = ARM_JOINTS
            pontos = []
            for i, pose in enumerate(poses):
                pt = JointTrajectoryPoint()
                pt.positions = [math.radians(float(v)) for v in pose['q_deg']]
                pt.velocities = [0.0] * 6
                total_s = (i + 1) * dur_s
                pt.time_from_start = Duration(
                    sec=int(total_s),
                    nanosec=int((total_s % 1.0) * 1_000_000_000))
                pontos.append(pt)
            msg.points = pontos
            self._arm_pub.publish(msg)

        if mode == 'MIRROR':
            # Robô real: MovJ + mão por pose, cadenciado por dur_s — paralelo ao Gazebo.
            drv = self._real_driver
            if drv is not None and self._robot_connected:
                try:
                    drv._send_dash(f'SpeedFactor({speed_pct})')
                except Exception:
                    pass
                for pose in poses:
                    if self._exec_stop.is_set():
                        break
                    t_step_start = time.monotonic()
                    try:
                        if _urdf_to_dobot is not None:
                            q_urdf = np.array(
                                [math.radians(float(v)) for v in pose['q_deg']])
                            q_dobot_deg = np.degrees(_urdf_to_dobot(q_urdf)).tolist()
                        else:
                            q_dobot_deg = list(pose['q_deg'])
                        drv.mov_j_joint_deg(q_dobot_deg)
                    except Exception as exc:
                        log.warning('MovJ falhou: %s', exc)
                        break
                    # Aplica pose da mão COVVI se armazenada (Tk-safe via after).
                    self._apply_hand_pose_from_movement(pose)
                    # Aguarda o restante de dur_s para este passo,
                    # verificando _exec_stop a cada 100 ms.
                    deadline = t_step_start + dur_s
                    while not self._exec_stop.is_set():
                        remaining = deadline - time.monotonic()
                        if remaining <= 0.0:
                            break
                        self._exec_stop.wait(min(0.1, remaining))
        elif mode == 'SIM_ONLY':
            # Itera por pose aplicando mão a cada passo; aguarda dur_s por pose.
            for pose in poses:
                if self._exec_stop.is_set():
                    break
                self._apply_hand_pose_from_movement(pose)
                self._exec_stop.wait(dur_s)
    def _apply_hand_pose_from_movement(self, pose: dict) -> None:
        """Aplica a pose da mão COVVI armazenada na pose (thread-safe via after).

        No-op se a pose não tiver 'hand_deg' ou se estiver no modo touch_tool.
        """
        hand_deg = pose.get('hand_deg')
        if not hand_deg:
            return
        hand_eci_id = pose.get('hand_eci_id')
        self.root.after(
            0, lambda hd=dict(hand_deg), eid=hand_eci_id:
            self._apply_hand_preset(hd, eci_grip_id=eid))
    # Diálogo de nome
    def _ask_name_dialog(self, title: str, initial: str = '') -> str | None:
        result: list[str | None] = [None]
        dlg = tk.Toplevel(self.root)
        dlg.title(title)
        dlg.configure(bg=BG)
        dlg.resizable(False, False)
        dlg.grab_set()

        tk.Label(dlg, text=title, bg=BG, fg=TEXT, font=FONT_HEAD
                 ).pack(padx=24, pady=(16, 8))
        var = tk.StringVar(value=initial)
        entry = tk.Entry(dlg, textvariable=var, font=FONT_LBL, width=32)
        entry.pack(padx=24, pady=(0, 8))
        entry.select_range(0, 'end')
        entry.focus_set()

        def _ok(_=None):
            val = var.get().strip()
            if val:
                result[0] = val
            dlg.destroy()

        def _cancel(_=None):
            dlg.destroy()

        row = tk.Frame(dlg, bg=BG)
        row.pack(pady=(0, 16))
        tk.Button(row, text='OK', command=_ok,
                  bg=PRIMARY, fg='white', font=FONT_LBL,
                  relief='flat', bd=0, padx=16, pady=4,
                  cursor='hand2').pack(side='left', padx=4)
        tk.Button(row, text='Cancel', command=_cancel,
                  bg=BTN_NEUTRAL, fg=TEXT, font=FONT_LBL,
                  relief='flat', bd=0, padx=16, pady=4,
                  cursor='hand2').pack(side='left', padx=4)
        entry.bind('<Return>', _ok)
        entry.bind('<Escape>', _cancel)
        dlg.wait_window()
        return result[0]
