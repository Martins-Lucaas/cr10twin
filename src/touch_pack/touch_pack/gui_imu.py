"""gui_imu.py — rastreio do CR10 pelos IMUs do WBAN-Inercial (aba IMU).

Só existe com end_effector:=hand: a aba Palpação (inútil sem o touch_tool)
vira a aba IMU. Os IMUs chegam pelo `imu_node` (sensor_msgs/Imu em
/imu/sensor_<id>), que o launch sobe no modo hand. Dois modos:

  JOINT      cada junta ligada segue a rotação de um IMU em torno de um eixo
             dele (X/Y/Z, com inversão), relativa ao Start, somada à pose da
             junta no Start. Juntas desligadas seguram a pose do Start.
  CARTESIAN  o TCP segue a mão: orientação = IMU de orientação (ex.: ID 6);
             posição = ponta da cadeia de segmentos (braço, antebraço, mão),
             cada um com um IMU e um comprimento. Os deslocamentos desde o
             Start viram uma pose alvo do TCP e a IK diferencial do manip3d
             (a mesma da aba 3D Manipulation) resolve as 6 juntas.

Regras que não são óbvias olhando o código:

  * só publica quando o alvo muda. O espelhamento MIRROR reinicia um debounce
    de 80 ms a cada trajetória recebida; publicar o mesmo alvo a 30 Hz faria
    o braço real nunca receber o MovJ;
  * o braço real só recebe alguma coisa com "→ real CR10" marcado E o robô
    em MIRROR (gate em `_cb_arm_trajectory`). Sem isso, só o sim;
  * dado velho congela: se um IMU usado ficar IMU_STALE_S sem mensagem, o
    alvo para onde está (não extrapola).
"""
from __future__ import annotations

import json
import math as _math
import os
import time
import tkinter as tk
from tkinter import ttk

import numpy as np
from sensor_msgs.msg import Imu

from .constants import ARM_JOINTS, CONFIG_DIR
from .gui_constants import ARM_LIMITS_DEG, MANIP_TRAJ_DURATION_S
from .imu_math import (
    LIMB_AXES, cartesian_target, chain_position, quat_to_R, step_toward_pose,
    twist_deg, wrap180,
)
from .imu_node import imu_topic
from .lc_filter import QOS_SENSOR
from .ui_helpers import (
    BG, BTN_NEUTRAL, DANGER, FONT_LBL, FONT_MONO_S, FONT_SMALL, OK, PANEL,
    PRIMARY, TEXT, TEXT_DIM, TEXT_MUTED, WARN, _Tooltip,
)

try:
    from .manip3d import ik_step, tcp_pose
except Exception:  # pragma: no cover — mesmo guard da aba 3D
    ik_step = tcp_pose = None

IMU_CONFIG_FILE = os.path.join(CONFIG_DIR, 'imu_config.json')

IMU_IDS = ('—', '1', '2', '3', '4', '5', '6')
IMU_AXES = {'X': (1.0, 0.0, 0.0), 'Y': (0.0, 1.0, 0.0), 'Z': (0.0, 0.0, 1.0)}
# Segmentos da cadeia de posição, do ombro para a ponta (nome, compr. default)
IMU_SEGMENTS = (('upper_arm', 'Upper arm', 0.30),
                ('forearm', 'Forearm', 0.25),
                ('hand', 'Hand', 0.0))
IMU_TICK_MS = 33
IMU_STALE_S = 0.5          # sem mensagem nova há mais que isso → congela
IMU_MAX_DEG_S = 60.0       # teto de velocidade de cada junta (os dois modos)
IMU_MAX_TCP_M_S = 0.15     # teto de velocidade linear do TCP (cartesiano)
IMU_MAX_TCP_DEG_S = 60.0   # teto de velocidade angular do TCP (cartesiano)
IMU_BOX_M = (0.25, 0.25, 0.25)   # caixa em torno do TCP do Start
IMU_PUBLISH_DEADBAND_DEG = 0.1


class IMUMixin:
    """Aba IMU: rastreio articular e cartesiano do CR10 por IMUs (modo hand)."""

    # Defaults de classe: `_on_close` e `_cb_arm_trajectory` consultam estes
    # nomes também no modo touch_tool, em que a aba nunca é construída.
    _imu_tracking = False
    _imu_mirror_on = False

    # ── Construção ───────────────────────────────────────────────────
    def _build_imu_tab(self, root: tk.Frame) -> None:
        self._imu_latest: dict[int, tuple[float, np.ndarray]] = {}
        for sid in range(1, len(IMU_IDS)):
            self.create_subscription(
                Imu, imu_topic(sid),
                lambda msg, s=sid: self._imu_on_msg(s, msg), QOS_SENSOR)
        self._imu_after: str | None = None
        self._imu_last_tick = time.monotonic()
        # Estado do modo JOINT
        self._imu_ref_q: dict[str, np.ndarray] = {}
        self._imu_ref_deg: dict[str, float] = {}
        self._imu_unwrap: dict[str, float] = {}
        self._imu_target_deg: dict[str, float] = {}
        self._imu_published_deg: dict[str, float] | None = None
        # Estado do modo CARTESIAN
        self._imu_cart: dict | None = None

        saved = self._imu_load_config()
        joints_cfg = saved.get('joints', {})
        cart_cfg = saved.get('cartesian', {})
        self._imu_rows: dict[str, dict] = {}

        body = tk.Frame(root, bg=BG)
        body.pack(fill='both', expand=True)

        # ── Sensores ──────────────────────────────────────────────────
        card_conn = self._card(body, 'IMU sensors (WBAN-Inercial)', expand=False)
        self._imu_live_lbl = tk.Label(card_conn, text='', font=FONT_LBL,
                                      bg=PANEL, fg=TEXT_DIM, anchor='w')
        self._imu_live_lbl.pack(fill='x')
        tk.Label(card_conn, font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
                 justify='left', wraplength=640,
                 text='Published by imu_node (/imu/sensor_<id>). Keep each sensor '
                      'still for ~2 s after it connects (initial orientation).'
                 ).pack(fill='x', pady=(4, 0))

        # ── Modo ──────────────────────────────────────────────────────
        card_mode = self._card(body, 'Mode', expand=False)
        card_mode.master.pack_configure(pady=(12, 0))
        self._imu_mode_var = tk.StringVar(value=saved.get('mode', 'JOINT'))
        mode_row = tk.Frame(card_mode, bg=PANEL)
        mode_row.pack(fill='x')
        for code, text in (('JOINT', 'Joint — each joint follows one IMU'),
                           ('CARTESIAN', 'Cartesian — TCP follows the hand (IK)')):
            tk.Radiobutton(mode_row, text=text, value=code,
                           variable=self._imu_mode_var,
                           command=self._imu_on_mode_change, bg=PANEL, fg=TEXT,
                           activebackground=PANEL, selectcolor=PANEL,
                           font=FONT_LBL, highlightthickness=0, bd=0,
                           cursor='hand2').pack(side='left', padx=(0, 18))

        # ── Junta ↔ IMU (modo JOINT) ─────────────────────────────────
        card_map = self._card(body, 'Joint mode — joint ↔ IMU', expand=False)
        card_map.master.pack_configure(pady=(12, 0))
        grid = tk.Frame(card_map, bg=PANEL)
        grid.pack(fill='x')
        heads = ('Joint', 'Move', 'IMU', 'Axis', 'Invert', 'IMU angle', 'Target')
        for c, h in enumerate(heads):
            tk.Label(grid, text=h, font=FONT_SMALL, bg=PANEL, fg=TEXT_MUTED
                     ).grid(row=0, column=c, sticky='w', padx=6, pady=(0, 4))
        for r, j in enumerate(ARM_JOINTS, start=1):
            cfg = joints_cfg.get(j, {})
            v_on = tk.BooleanVar(value=bool(cfg.get('enabled', False)))
            v_id = tk.StringVar(value=str(cfg.get('imu', '—')))
            v_ax = tk.StringVar(value=str(cfg.get('axis', 'Z')))
            v_inv = tk.BooleanVar(value=bool(cfg.get('invert', False)))
            tk.Label(grid, text=j, font=FONT_LBL, bg=PANEL, fg=TEXT
                     ).grid(row=r, column=0, sticky='w', padx=6, pady=3)
            self._imu_check(grid, v_on).grid(row=r, column=1, padx=6)
            self._imu_combo(grid, v_id, IMU_IDS).grid(row=r, column=2, padx=6)
            self._imu_combo(grid, v_ax, tuple(IMU_AXES)).grid(row=r, column=3, padx=6)
            self._imu_check(grid, v_inv).grid(row=r, column=4, padx=6)
            lbl_ang = tk.Label(grid, text='—', font=FONT_MONO_S, bg=PANEL,
                               fg=TEXT_DIM, width=9, anchor='e')
            lbl_ang.grid(row=r, column=5, padx=6)
            lbl_tgt = tk.Label(grid, text='—', font=FONT_MONO_S, bg=PANEL,
                               fg=TEXT_DIM, width=9, anchor='e')
            lbl_tgt.grid(row=r, column=6, padx=6)
            self._imu_rows[j] = {'on': v_on, 'id': v_id, 'axis': v_ax,
                                 'inv': v_inv, 'ang': lbl_ang, 'tgt': lbl_tgt}
            for v in (v_on, v_id, v_ax, v_inv):
                v.trace_add('write', lambda *_a, jj=j: self._imu_on_row_change(jj))

        # ── Cadeia → TCP (modo CARTESIAN) ────────────────────────────
        card_cart = self._card(body, 'Cartesian mode — hand → TCP (inverse kinematics)',
                               expand=False)
        card_cart.master.pack_configure(pady=(12, 0))
        cg = tk.Frame(card_cart, bg=PANEL)
        cg.pack(fill='x')
        self._imu_cart_vars = {
            'ori_imu': tk.StringVar(value=str(cart_cfg.get('ori_imu', '6'))),
            'follow_ori': tk.BooleanVar(value=bool(cart_cfg.get('follow_ori', True))),
            'follow_pos': tk.BooleanVar(value=bool(cart_cfg.get('follow_pos', True))),
            'limb_axis': tk.StringVar(value=str(cart_cfg.get('limb_axis', '+X'))),
            'scale': tk.DoubleVar(value=float(cart_cfg.get('scale', 1.0))),
            'heading': tk.DoubleVar(value=float(cart_cfg.get('heading', 0.0))),
        }
        segs_cfg = cart_cfg.get('segments', {})
        self._imu_seg_vars: dict[str, tuple[tk.StringVar, tk.DoubleVar]] = {}
        cv = self._imu_cart_vars

        tk.Label(cg, text='Orientation IMU', font=FONT_LBL, bg=PANEL, fg=TEXT
                 ).grid(row=0, column=0, sticky='w', padx=6, pady=3)
        self._imu_combo(cg, cv['ori_imu'], IMU_IDS).grid(row=0, column=1, sticky='w', padx=6)
        self._imu_check(cg, cv['follow_ori'], 'follow orientation'
                        ).grid(row=0, column=2, columnspan=2, sticky='w', padx=6)
        tk.Label(cg, text='Segment', font=FONT_SMALL, bg=PANEL, fg=TEXT_MUTED
                 ).grid(row=1, column=0, sticky='w', padx=6, pady=(8, 2))
        tk.Label(cg, text='IMU', font=FONT_SMALL, bg=PANEL, fg=TEXT_MUTED
                 ).grid(row=1, column=1, sticky='w', padx=6, pady=(8, 2))
        tk.Label(cg, text='Length (m)', font=FONT_SMALL, bg=PANEL, fg=TEXT_MUTED
                 ).grid(row=1, column=2, sticky='w', padx=6, pady=(8, 2))
        for r, (key, label, length) in enumerate(IMU_SEGMENTS, start=2):
            sc = segs_cfg.get(key, {})
            v_id = tk.StringVar(value=str(sc.get('imu', '—')))
            v_len = tk.DoubleVar(value=float(sc.get('length_m', length)))
            tk.Label(cg, text=label, font=FONT_LBL, bg=PANEL, fg=TEXT
                     ).grid(row=r, column=0, sticky='w', padx=6, pady=3)
            self._imu_combo(cg, v_id, IMU_IDS).grid(row=r, column=1, sticky='w', padx=6)
            tk.Spinbox(cg, textvariable=v_len, from_=0.0, to=1.0, increment=0.01,
                       width=6, font=FONT_MONO_S).grid(row=r, column=2, sticky='w', padx=6)
            self._imu_seg_vars[key] = (v_id, v_len)
            for v in (v_id, v_len):
                v.trace_add('write', lambda *_a: self._imu_on_cart_change())
        r0 = 2 + len(IMU_SEGMENTS)
        opts = tk.Frame(cg, bg=PANEL)
        opts.grid(row=r0, column=0, columnspan=4, sticky='w', pady=(8, 0))
        tk.Label(opts, text='Limb axis', font=FONT_LBL, bg=PANEL, fg=TEXT
                 ).pack(side='left', padx=(6, 4))
        ax_cb = self._imu_combo(opts, cv['limb_axis'], tuple(LIMB_AXES))
        ax_cb.pack(side='left')
        _Tooltip(ax_cb, 'Sensor axis that points along the limb, toward the hand '
                 '(depends on how the sensors are worn).')
        tk.Label(opts, text='Scale', font=FONT_LBL, bg=PANEL, fg=TEXT
                 ).pack(side='left', padx=(14, 4))
        tk.Spinbox(opts, textvariable=cv['scale'], from_=0.1, to=3.0, increment=0.1,
                   width=5, font=FONT_MONO_S).pack(side='left')
        tk.Label(opts, text='Heading (°)', font=FONT_LBL, bg=PANEL, fg=TEXT
                 ).pack(side='left', padx=(14, 4))
        hd = tk.Spinbox(opts, textvariable=cv['heading'], from_=-180, to=180,
                        increment=5, width=6, font=FONT_MONO_S)
        hd.pack(side='left')
        _Tooltip(hd, 'Yaw from the human frame (magnetic north) to the robot '
                 'base frame. Adjust until moving your hand forward moves the '
                 'TCP forward.')
        self._imu_check(opts, cv['follow_pos'], 'follow position'
                        ).pack(side='left', padx=(14, 0))
        for v in cv.values():
            v.trace_add('write', lambda *_a: self._imu_on_cart_change())
        self._imu_cart_lbl = tk.Label(card_cart, text='', font=FONT_MONO_S,
                                      bg=PANEL, fg=TEXT_DIM, anchor='w', justify='left')
        self._imu_cart_lbl.pack(fill='x', pady=(8, 0))
        if ik_step is None:
            self._imu_cart_lbl.config(text='IK unavailable (manip3d failed to import).',
                                      fg=DANGER)

        # ── Rastreio ──────────────────────────────────────────────────
        card_run = self._card(body, 'Tracking', expand=False)
        card_run.master.pack_configure(pady=(12, 0))
        btns = tk.Frame(card_run, bg=PANEL)
        btns.pack(fill='x')
        self._imu_track_btn = tk.Button(
            btns, text='▶  Start tracking', command=self._imu_toggle_tracking,
            bg=OK, fg='white', activebackground=OK, activeforeground='white',
            font=FONT_LBL, relief='flat', bd=0, padx=12, pady=6, cursor='hand2')
        self._imu_track_btn.pack(side='left')
        zero_btn = tk.Button(
            btns, text='⟲  Zero', command=self._imu_zero,
            bg=BTN_NEUTRAL, fg=TEXT, font=FONT_LBL, relief='flat', bd=0,
            padx=12, pady=6, cursor='hand2')
        zero_btn.pack(side='left', padx=(8, 0))
        _Tooltip(zero_btn, 'Makes the current IMU orientation the reference. '
                 'While tracking, the arm stays where it is and follows from here.')
        self._imu_mirror_var = tk.BooleanVar(value=False)
        mirror_chk = tk.Checkbutton(
            btns, text='→ real CR10', variable=self._imu_mirror_var,
            command=self._imu_on_mirror_change, bg=PANEL, fg=TEXT,
            activebackground=PANEL, selectcolor=PANEL, font=FONT_LBL,
            highlightthickness=0, bd=0, cursor='hand2')
        mirror_chk.pack(side='left', padx=(16, 0))
        _Tooltip(mirror_chk, 'Unchecked: only the simulation follows the IMUs. '
                 'Checked AND robot in MIRROR: the real CR10 receives a MovJ '
                 'each time the motion settles (80 ms debounce).')
        self._imu_gate_lbl = tk.Label(
            card_run, text='', font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
            anchor='w', justify='left', wraplength=640)
        self._imu_gate_lbl.pack(fill='x', pady=(8, 0))
        self._imu_refresh_gate_lbl()

        self._imu_after = self.root.after(IMU_TICK_MS, self._imu_tick)

    def _imu_check(self, parent, var, text: str = '') -> tk.Checkbutton:
        return tk.Checkbutton(parent, text=text, variable=var, bg=PANEL, fg=TEXT,
                              activebackground=PANEL, selectcolor=PANEL,
                              font=FONT_LBL, highlightthickness=0, bd=0,
                              cursor='hand2')

    @staticmethod
    def _imu_combo(parent, var, values) -> ttk.Combobox:
        return ttk.Combobox(parent, textvariable=var, values=values, width=4,
                            state='readonly')

    # ── Entrada ROS ──────────────────────────────────────────────────
    def _imu_on_msg(self, sensor_id: int, msg: Imu) -> None:
        o = msg.orientation
        q = np.array([o.w, o.x, o.y, o.z], dtype=float)
        if np.all(np.isfinite(q)) and np.linalg.norm(q) > 0.5:
            # Rebind atômico do item — o tick do Tk lê este dict a 30 Hz.
            self._imu_latest[sensor_id] = (time.monotonic(), q)

    def _imu_fresh_q(self, sid, now: float):
        item = self._imu_latest.get(sid) if sid is not None else None
        if item is None or now - item[0] > IMU_STALE_S:
            return None
        return item[1]

    # ── Persistência ─────────────────────────────────────────────────
    def _imu_load_config(self) -> dict:
        try:
            with open(IMU_CONFIG_FILE) as fh:
                data = json.load(fh)
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def _imu_save_config(self) -> None:
        if len(self._imu_rows) != len(ARM_JOINTS) or not hasattr(self, '_imu_seg_vars'):
            return   # ainda construindo
        try:
            cart = {k: v.get() for k, v in self._imu_cart_vars.items()}
            cart['segments'] = {k: {'imu': i.get(), 'length_m': L.get()}
                                for k, (i, L) in self._imu_seg_vars.items()}
        except (tk.TclError, ValueError):
            return   # spinbox no meio da digitação
        data = {
            'mode': self._imu_mode_var.get(),
            'joints': {j: {'enabled': r['on'].get(), 'imu': r['id'].get(),
                           'axis': r['axis'].get(), 'invert': r['inv'].get()}
                       for j, r in self._imu_rows.items()},
            'cartesian': cart,
        }
        try:
            os.makedirs(CONFIG_DIR, exist_ok=True)
            with open(IMU_CONFIG_FILE, 'w') as fh:
                json.dump(data, fh, indent=2)
        except OSError as exc:
            self.get_logger().warn(f'imu config not saved: {exc}')

    def _imu_on_row_change(self, joint: str) -> None:
        self._imu_save_config()
        # Mudou IMU/eixo/liga-desliga: a junta recomeça do ponto onde está,
        # com nova referência capturada no próximo tick (sem salto).
        self._imu_ref_q.pop(joint, None)
        if joint in self._imu_target_deg:
            self._imu_ref_deg[joint] = self._imu_target_deg[joint]
        self._imu_unwrap.pop(joint, None)

    def _imu_on_cart_change(self) -> None:
        self._imu_save_config()
        if self._imu_cart is not None:
            self._imu_cart_rezero()

    def _imu_on_mode_change(self) -> None:
        if self._imu_tracking:
            self._imu_stop_tracking('IMU tracking stopped: mode changed.', WARN)
        self._imu_save_config()

    def _imu_row_cfg(self, joint: str):
        """(sensor_id, eixo, sinal) da junta, ou None se não estiver mapeada."""
        r = self._imu_rows[joint]
        try:
            sid = int(r['id'].get())
        except ValueError:
            return None
        return sid, IMU_AXES[r['axis'].get()], (-1.0 if r['inv'].get() else 1.0)

    # ── Cadeia cartesiana ────────────────────────────────────────────
    def _imu_cart_spec(self):
        """(ori_id|None, [(id, L)], eixo, escala, heading, follow_ori, follow_pos)."""
        cv = self._imu_cart_vars
        try:
            ori = int(cv['ori_imu'].get())
        except ValueError:
            ori = None
        segs = []
        for v_id, v_len in self._imu_seg_vars.values():
            try:
                sid, length = int(v_id.get()), float(v_len.get())
            except (ValueError, tk.TclError):
                continue
            if length > 0.0:
                segs.append((sid, length))
        try:
            scale, heading = float(cv['scale'].get()), float(cv['heading'].get())
        except (ValueError, tk.TclError):
            scale, heading = 1.0, 0.0
        return (ori, segs, LIMB_AXES[cv['limb_axis'].get()], scale, heading,
                bool(cv['follow_ori'].get()) and ori is not None,
                bool(cv['follow_pos'].get()) and bool(segs))

    def _imu_cart_sample(self, now: float):
        """(R_ori|None, p_mão|None) agora, ou None se algum IMU usado está velho."""
        ori, segs, axis, _s, _h, f_ori, f_pos = self._imu_cart_spec()
        R_ori = p_hand = None
        if f_ori:
            q = self._imu_fresh_q(ori, now)
            if q is None:
                return None
            R_ori = quat_to_R(q)
        if f_pos:
            rots = []
            for sid, _L in segs:
                q = self._imu_fresh_q(sid, now)
                if q is None:
                    return None
                rots.append(quat_to_R(q))
            p_hand = chain_position(rots, [L for _s, L in segs], axis)
        return R_ori, p_hand

    def _imu_cart_rezero(self) -> None:
        """Nova referência: mão de agora ↔ TCP comandado de agora (sem salto)."""
        st = self._imu_cart
        sample = self._imu_cart_sample(time.monotonic())
        if st is None or sample is None:
            if st is not None:
                st['ref'] = None   # captura no próximo tick com dado fresco
            return
        T = tcp_pose(st['q'], T_end=st['T_end'])
        st['p_tcp0'], st['R_tcp0'] = T[:3, 3].copy(), T[:3, :3].copy()
        st['p_cmd'], st['R_cmd'] = T[:3, 3].copy(), T[:3, :3].copy()
        st['ref'] = sample

    # ── Rastreio ─────────────────────────────────────────────────────
    def _imu_block_reason(self) -> str:
        with self._lock:
            phase = self._latest_phase
        if phase not in ('IDLE', 'DONE', 'ABORTED'):
            return f'robot busy ({phase})'
        if self._estop_latched:
            return 'E-STOP engaged'
        if self._drag_enabled:
            return 'drag teach active'
        if self._exec_movement_id is not None:
            return 'motion running'
        return ''

    def _imu_slider_q_deg(self) -> dict[str, float]:
        out = {}
        for j in ARM_JOINTS:
            lo, hi = ARM_LIMITS_DEG[j]
            out[j] = self._clamp_var(self.arm_sliders[j], lo, hi, 0.0)
        return out

    def _imu_toggle_tracking(self) -> None:
        if self._imu_tracking:
            self._imu_stop_tracking('IMU tracking stopped.')
            return
        reason = self._imu_block_reason()
        if reason:
            self._set_status(f'IMU tracking blocked: {reason}.', WARN)
            return
        now = time.monotonic()
        q_deg = self._imu_slider_q_deg()
        if self._imu_mode_var.get() == 'CARTESIAN':
            if ik_step is None:
                self._set_status('IMU: cartesian mode needs manip3d (IK).', DANGER)
                return
            ori, segs, *_r, f_ori, f_pos = self._imu_cart_spec()
            if not (f_ori or f_pos):
                self._set_status('IMU: pick an orientation IMU and/or segment IMUs '
                                 'with length > 0.', WARN)
                return
            used = ([ori] if f_ori else []) + ([s for s, _L in segs] if f_pos else [])
            missing = sorted({s for s in used if self._imu_fresh_q(s, now) is None})
            if missing:
                self._set_status(f'IMU: no data from sensor(s) {missing}.', WARN)
                return
            self._imu_cart = {
                'q': np.radians([q_deg[j] for j in ARM_JOINTS]),
                'T_end': self._manip_T_end(),
                'v_tcp': 0.0, 'ik_err': 0.0,
            }
            self._imu_cart_rezero()
            self._imu_published_deg = dict(q_deg)
            what = ' + '.join((['orientation'] if f_ori else [])
                              + (['position'] if f_pos else []))
        else:
            mapped = [j for j in ARM_JOINTS
                      if self._imu_rows[j]['on'].get() and self._imu_row_cfg(j)]
            if not mapped:
                self._set_status('IMU: enable at least one joint with an IMU ID.', WARN)
                return
            missing = sorted({self._imu_row_cfg(j)[0] for j in mapped
                              if self._imu_fresh_q(self._imu_row_cfg(j)[0], now) is None})
            if missing:
                self._set_status(f'IMU: no data from sensor(s) {missing}.', WARN)
                return
            self._imu_ref_deg = dict(q_deg)
            self._imu_target_deg = dict(q_deg)
            self._imu_published_deg = dict(q_deg)
            # Referência no instante do Start (e não no próximo tick, quando o
            # IMU pode já ter girado e esse giro sumiria do rastreio).
            self._imu_ref_q = {j: self._imu_fresh_q(self._imu_row_cfg(j)[0], now)
                               for j in mapped}
            self._imu_unwrap = {}
            what = ', '.join(mapped)
        self._imu_tracking = True
        self._imu_track_btn.config(text='■  Stop tracking', bg=DANGER,
                                   activebackground=DANGER)
        self._imu_refresh_gate_lbl()
        self._set_status(f'IMU tracking ({self._imu_mode_var.get().lower()}): {what}.', OK)

    def _imu_stop_tracking(self, msg: str | None, color: str = TEXT_DIM) -> None:
        self._imu_tracking = False
        self._imu_cart = None
        if getattr(self, '_imu_track_btn', None) is not None:
            self._imu_track_btn.config(text='▶  Start tracking', bg=OK,
                                       activebackground=OK)
            self._imu_refresh_gate_lbl()
        if msg:
            self._set_status(msg, color)

    def _imu_zero(self) -> None:
        for j in ARM_JOINTS:
            self._imu_ref_q.pop(j, None)
            self._imu_unwrap.pop(j, None)
            if j in self._imu_target_deg:
                self._imu_ref_deg[j] = self._imu_target_deg[j]
        if self._imu_cart is not None:
            self._imu_cart_rezero()
        self._set_status('IMU reference zeroed.', TEXT_MUTED)

    def _imu_on_mirror_change(self) -> None:
        self._imu_mirror_on = bool(self._imu_mirror_var.get())
        self._imu_refresh_gate_lbl()

    def _imu_refresh_gate_lbl(self) -> None:
        if self._imu_mirror_on and self._robot_mode == 'MIRROR':
            txt, color = ('→ real ON — the real CR10 gets a MovJ every time the '
                          'tracked motion settles.'), DANGER
        elif self._imu_mirror_on:
            txt, color = ('→ real is checked but the robot is in SIM_ONLY — only '
                          'Gazebo moves.'), TEXT_MUTED
        else:
            txt, color = 'Simulation only — the real arm is not commanded.', TEXT_DIM
        self._imu_gate_lbl.config(
            text=f'{txt}  Limits: {IMU_MAX_DEG_S:.0f}°/s per joint, TCP '
                 f'{IMU_MAX_TCP_M_S * 1000:.0f} mm/s and {IMU_MAX_TCP_DEG_S:.0f}°/s, '
                 f'±{IMU_BOX_M[0] * 1000:.0f} mm around the start pose; a sensor '
                 f'silent for {IMU_STALE_S:.1f} s freezes the motion.',
            fg=color)

    def _imu_shutdown(self) -> None:
        if getattr(self, '_imu_after', None) is not None:
            try:
                self.root.after_cancel(self._imu_after)
            except Exception:
                pass
            self._imu_after = None
        self._imu_tracking = False

    # ── Laço (Tk, 30 Hz) ─────────────────────────────────────────────
    def _imu_tick(self) -> None:
        self._imu_after = None
        now = time.monotonic()
        dt = min(0.1, now - self._imu_last_tick)
        self._imu_last_tick = now

        live = sorted(s for s in self._imu_latest if self._imu_fresh_q(s, now) is not None)
        self._imu_live_lbl.config(
            text=f'Live IMUs: {", ".join(map(str, live))}' if live
            else 'No IMU data — is imu_node running and are the ESPs on?',
            fg=OK if live else WARN)

        if self._imu_tracking:
            reason = self._imu_block_reason()
            if reason:
                self._imu_stop_tracking(f'IMU tracking stopped: {reason}.', WARN)

        q_new_deg = self._imu_tick_joint(now, dt)
        if self._imu_tracking and self._imu_cart is not None:
            q_new_deg = self._imu_tick_cartesian(now, dt)
        if self._imu_tracking and q_new_deg is not None:
            self._imu_publish_if_changed(q_new_deg)

        self._imu_after = self.root.after(IMU_TICK_MS, self._imu_tick)

    def _imu_tick_joint(self, now: float, dt: float):
        """Leituras da tabela; no modo JOINT em rastreio, devolve o alvo."""
        tracking = self._imu_tracking and self._imu_cart is None
        stale = []
        for j in ARM_JOINTS:
            row = self._imu_rows[j]
            cfg = self._imu_row_cfg(j)
            q = self._imu_fresh_q(cfg[0], now) if cfg else None
            ang = None
            if q is not None:
                if j not in self._imu_ref_q:
                    self._imu_ref_q[j] = q
                raw = twist_deg(self._imu_ref_q[j], q, cfg[1])
                # desembrulha para a junta passar de ±180° sem saltar
                prev = self._imu_unwrap.get(j, 0.0)
                ang = prev + wrap180(raw - prev)
                self._imu_unwrap[j] = ang
            row['ang'].config(text='—' if ang is None else f'{ang * cfg[2]:+7.1f}°',
                              fg=TEXT if ang is not None else TEXT_DIM)
            if not tracking:
                row['tgt'].config(text='—', fg=TEXT_DIM)
                continue
            if row['on'].get() and cfg:
                if ang is None:
                    stale.append(j)
                else:
                    lo, hi = ARM_LIMITS_DEG[j]
                    desired = max(lo, min(hi, self._imu_ref_deg[j] + cfg[2] * ang))
                    cur = self._imu_target_deg[j]
                    step = IMU_MAX_DEG_S * dt
                    self._imu_target_deg[j] = cur + max(-step, min(step, desired - cur))
            row['tgt'].config(text=f'{self._imu_target_deg[j]:+7.1f}°',
                              fg=PRIMARY if row['on'].get() else TEXT_DIM)
        if not tracking:
            return None
        if stale:
            self._set_status(f'IMU: no fresh data for {", ".join(stale)} — frozen.', WARN)
        return dict(self._imu_target_deg)

    def _imu_tick_cartesian(self, now: float, dt: float):
        st = self._imu_cart
        sample = self._imu_cart_sample(now)
        if sample is None:
            self._imu_cart_lbl.config(text='A sensor in use went silent — frozen.', fg=WARN)
            return None
        if st['ref'] is None:
            self._imu_cart_rezero()
            return None
        _o, _s, _a, scale, heading, f_ori, f_pos = self._imu_cart_spec()
        (R_ori0, p_h0), (R_ori, p_h) = st['ref'], sample
        dp = (p_h - p_h0) if f_pos else np.zeros(3)
        dR = (R_ori @ R_ori0.T) if f_ori else np.eye(3)
        p_des, R_des = cartesian_target(st['p_tcp0'], st['R_tcp0'], dp, dR,
                                        heading_deg=heading, scale=scale,
                                        box_m=IMU_BOX_M)
        p_prev = st['p_cmd']
        st['p_cmd'], st['R_cmd'] = step_toward_pose(
            p_prev, st['R_cmd'], p_des, R_des,
            IMU_MAX_TCP_M_S * dt, _math.radians(IMU_MAX_TCP_DEG_S) * dt)
        res = ik_step(st['q'], st['p_cmd'], T_end=st['T_end'], R_lock=st['R_cmd'])
        # Teto por junta também no cartesiano: perto de singularidade a IK
        # pode pedir um salto grande de punho por um passo pequeno do TCP.
        max_dq = _math.radians(IMU_MAX_DEG_S) * dt
        st['q'] = st['q'] + np.clip(res.q - st['q'], -max_dq, max_dq)
        v = float(np.linalg.norm(st['p_cmd'] - p_prev)) / max(dt, 1e-3)
        st['v_tcp'] = 0.8 * st['v_tcp'] + 0.2 * v
        d = (st['p_cmd'] - st['p_tcp0']) * 1000.0
        T = tcp_pose(st['q'], T_end=st['T_end'])
        err = float(np.linalg.norm(T[:3, 3] - st['p_cmd'])) * 1000.0
        flags = ('  SINGULAR' if res.singular else '') + ('  AT LIMIT' if res.at_limit else '')
        self._imu_cart_lbl.config(
            text=f'ΔTCP  x {d[0]:+6.0f}  y {d[1]:+6.0f}  z {d[2]:+6.0f} mm   '
                 f'v {st["v_tcp"] * 1000:5.0f} mm/s   IK err {err:4.1f} mm{flags}',
            fg=WARN if (flags or err > 5.0) else TEXT)
        return {j: _math.degrees(float(v)) for j, v in zip(ARM_JOINTS, st['q'])}

    def _imu_publish_if_changed(self, q_deg: dict[str, float]) -> None:
        pub = self._imu_published_deg or {}
        if not any(abs(q_deg[j] - pub.get(j, 1e9)) > IMU_PUBLISH_DEADBAND_DEG
                   for j in ARM_JOINTS):
            return
        with self._suppress():
            for j in ARM_JOINTS:
                self.arm_sliders[j].set(round(q_deg[j], 1))
        self._publish_arm_q([_math.radians(q_deg[j]) for j in ARM_JOINTS],
                            MANIP_TRAJ_DURATION_S)
        self._imu_published_deg = dict(q_deg)
