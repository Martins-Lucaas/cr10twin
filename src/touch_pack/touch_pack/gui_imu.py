"""gui_imu.py — rastreio do CR10 pelos IMUs do WBAN-Inercial (aba IMU).

Só existe com end_effector:=hand: a aba Palpação (inútil sem o touch_tool)
vira a aba IMU. Os IMUs chegam pelo `imu_node` (sensor_msgs/Imu em
/imu/sensor_<id>), que o launch sobe no modo hand. Modo único, cartesiano:
o TCP segue a mão — orientação = IMU de orientação (ex.: ID 6); posição =
ponta da cadeia de segmentos (braço, antebraço, mão), cada um com um IMU e um
comprimento. Os deslocamentos desde o Start viram uma pose alvo do TCP e a IK
diferencial do manip3d (a mesma da aba 3D Manipulation) resolve as juntas.
Juntas marcadas em "Lock joints" ficam fixas: a IK usa só as outras.

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

import faulthandler
import json
import math as _math
import os
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

import numpy as np
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from .constants import ARM_JOINTS, CONFIG_DIR
from .gui_constants import ARM_LIMITS_DEG, MANIP_TRAJ_DURATION_S
from .imu_math import (
    LIMB_AXES, cartesian_target, chain_position, quat_to_R, step_toward_pose,
)
from .imu_node import CALIB_REQ_TOPIC, CALIB_STATUS_TOPIC, imu_topic
from .gui_imu_calib import ImuCalibWizard
from .imu_recorder import TrackingRecorder
from .lc_filter import QOS_SENSOR
from .ui_helpers import (
    BG, BORDER, BTN_NEUTRAL, DANGER, FONT_HEAD, FONT_LBL, FONT_MONO_S,
    FONT_SMALL, OK, PANEL, PRIMARY, PRIMARY_HV, TEXT, TEXT_DIM, TEXT_MUTED,
    WARN, _shade, _Tooltip,
)

try:
    from .manip3d import ik_step, tcp_pose
except Exception:  # pragma: no cover — mesmo guard da aba 3D
    ik_step = tcp_pose = None

IMU_CONFIG_FILE = os.path.join(CONFIG_DIR, 'imu_config.json')

IMU_IDS = ('—', '1', '2', '3', '4', '5', '6')
# Segmentos da cadeia de posição, do ombro para a ponta (nome, compr. default)
IMU_SEGMENTS = (('upper_arm', 'Upper arm', 0.30),
                ('forearm', 'Forearm', 0.25),
                ('hand', 'Hand', 0.0))
IMU_TICK_MS = 33
IMU_STALE_S = 0.5          # sem mensagem nova há mais que isso → congela
IMU_MAX_DEG_S = 60.0       # teto de velocidade de cada junta
IMU_MAX_TCP_M_S = 0.15     # teto de velocidade linear do TCP (cartesiano)
IMU_MAX_TCP_DEG_S = 60.0   # teto de velocidade angular do TCP (cartesiano)
IMU_BOX_M = (0.25, 0.25, 0.25)   # caixa em torno do TCP do Start
IMU_PUBLISH_DEADBAND_DEG = 0.1
TK_STALL_S = 5.0   # laço do Tk parado por mais que isto → despeja as pilhas (0 = sem vigia)
# O ESP manda as amostras em rajadas (6 a cada 60 ms a 100 Hz): com a
# profundidade 1 do QOS_SENSOR a GUI só via a última de cada rajada e a
# gravação perdia ~metade. O rastreio usa só a mais nova de qualquer jeito.
QOS_IMU = QoSProfile(reliability=QOS_SENSOR.reliability,
                     durability=QOS_SENSOR.durability,
                     history=QOS_SENSOR.history, depth=50)
LED_OFF, LED_RIM = '#cbd5e1', '#94a3b8'
CALIB_NAMES = {'AG': 'Acc + Gyro', 'MAG': 'Magnetometer'}
CALIB_HINT = {'AG': 'keep it still and flat', 'MAG': 'rotate it slowly in all directions'}
CALIB_SHOW_S = 8.0   # quanto tempo o resultado fica no LED/label


class IMUMixin:
    """Aba IMU: rastreio cartesiano do CR10 por IMUs (modo hand)."""

    # Defaults de classe: `_on_close` e `_cb_arm_trajectory` consultam estes
    # nomes também no modo touch_tool, em que a aba nunca é construída.
    _imu_tracking = False
    _imu_mirror_on = False
    _imu_rec: TrackingRecorder | None = None
    _imu_wizard: ImuCalibWizard | None = None
    _imu_live: set = set()

    # ── Construção ───────────────────────────────────────────────────
    def _build_imu_tab(self, root: tk.Frame) -> None:
        self._imu_latest: dict[int, tuple[float, np.ndarray]] = {}
        for sid in range(1, len(IMU_IDS)):
            self.create_subscription(
                Imu, imu_topic(sid),
                lambda msg, s=sid: self._imu_on_msg(s, msg), QOS_IMU)
        self._imu_after: str | None = None
        self._imu_last_tick = time.monotonic()
        self._imu_published_deg: dict[str, float] | None = None
        self._imu_cart: dict | None = None

        saved = self._imu_load_config()
        cart_cfg = saved.get('cartesian', {})
        locked_cfg = set(saved.get('locked', ()))

        body = tk.Frame(root, bg=BG)
        body.pack(fill='both', expand=True)
        # Duas colunas, como a aba 3D: configuração à esquerda, operação
        # (o que se usa com o braço andando) fixa à direita.
        # Largura fixa por um espaçador, e NÃO por pack_propagate(False): dentro
        # do _scrollable a altura da aba vem do que as colunas pedem, e com a
        # propagação desligada esta coluna não pedia altura nenhuma — o cartão
        # dos sensores ficava cortado abaixo do Tracking.
        col_run = tk.Frame(body, bg=BG)
        col_run.pack(side='right', fill='y', padx=(12, 0))
        tk.Frame(col_run, bg=BG, width=340, height=0).pack()
        col_cfg = tk.Frame(body, bg=BG)
        col_cfg.pack(side='left', fill='both', expand=True)

        # ── Rastreio ──────────────────────────────────────────────────
        card_run = self._card(col_run, 'Tracking', expand=False)
        self._imu_track_btn = tk.Button(
            card_run, text='▶  Start tracking', command=self._imu_toggle_tracking,
            bg=OK, fg='white', activebackground=_shade(OK, -0.08),
            activeforeground='white', font=FONT_HEAD, relief='flat', bd=0,
            pady=10, cursor='hand2')
        self._imu_track_btn.pack(fill='x')
        btns = tk.Frame(card_run, bg=PANEL)
        btns.pack(fill='x', pady=(6, 0))
        zero_btn = tk.Button(
            btns, text='⟲  Zero', command=self._imu_zero,
            bg=BTN_NEUTRAL, fg=TEXT, activebackground=_shade(BTN_NEUTRAL, -0.08),
            font=FONT_LBL, relief='flat', bd=0, pady=6, cursor='hand2')
        zero_btn.pack(side='left', fill='x', expand=True, padx=(0, 3))
        _Tooltip(zero_btn, 'Makes the current IMU orientation the reference. '
                 'While tracking, the arm stays where it is and follows from here.')
        home_btn = tk.Button(
            btns, text='⌂  Home', command=self._imu_go_home,
            bg=PRIMARY, fg='white', activebackground=PRIMARY_HV,
            activeforeground='white', font=FONT_LBL, relief='flat', bd=0,
            pady=6, cursor='hand2')
        home_btn.pack(side='left', fill='x', expand=True, padx=(3, 0))
        _Tooltip(home_btn, 'Stops tracking and sends the arm to the Home pose '
                 '(the same one as the Manual Control tab).')
        self._imu_mirror_var = tk.BooleanVar(value=False)
        mirror_chk = tk.Checkbutton(
            card_run, text='→ real CR10', variable=self._imu_mirror_var,
            command=self._imu_on_mirror_change, bg=PANEL, fg=TEXT,
            activebackground=PANEL, selectcolor=PANEL, font=FONT_LBL,
            highlightthickness=0, bd=0, cursor='hand2')
        mirror_chk.pack(anchor='w', pady=(10, 0))
        self._imu_rec_var = tk.BooleanVar(value=bool(saved.get('record', True)))
        rec_chk = self._imu_check(card_run, self._imu_rec_var,
                                  '● Record session (IMUs + CR10)')
        rec_chk.pack(anchor='w', pady=(4, 0))
        _Tooltip(rec_chk, 'From Start to Stop, writes every IMU message, the '
                 'tracking state at 30 Hz and the real CR10 joints (when '
                 'connected) to sensors/Data/IMU_TRACKING/<date_time>/.')
        self._imu_rec_var.trace_add('write', lambda *_a: self._imu_save_config())
        _Tooltip(mirror_chk, 'Unchecked: only the simulation follows the IMUs. '
                 'Checked AND robot in MIRROR: the real CR10 receives a MovJ '
                 'each time the motion settles (80 ms debounce).')
        self._imu_gate_lbl = tk.Label(
            card_run, text='', font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
            anchor='w', justify='left', wraplength=300)
        self._imu_gate_lbl.pack(fill='x', pady=(4, 0))
        self._imu_rec_lbl = tk.Label(
            card_run, text='', font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
            anchor='w', justify='left', wraplength=300)
        self._imu_rec_lbl.pack(fill='x')
        tk.Frame(card_run, bg=BORDER, height=1).pack(fill='x', pady=(10, 8))
        self._imu_cart_lbl = tk.Label(card_run, text='Idle.', font=FONT_MONO_S,
                                      bg=PANEL, fg=TEXT_DIM, anchor='w',
                                      justify='left')
        self._imu_cart_lbl.pack(fill='x')
        if ik_step is None:
            self._imu_cart_lbl.config(text='IK unavailable (manip3d failed to import).',
                                      fg=DANGER)
        self._imu_limits_lbl = tk.Label(
            card_run, font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
            justify='left', wraplength=300,
            text=f'Limits: {IMU_MAX_DEG_S:.0f}°/s per joint, TCP '
                 f'{IMU_MAX_TCP_M_S * 1000:.0f} mm/s and {IMU_MAX_TCP_DEG_S:.0f}°/s, '
                 f'±{IMU_BOX_M[0] * 1000:.0f} mm around the start pose; a sensor '
                 f'silent for {IMU_STALE_S:.1f} s freezes the motion.')
        self._imu_limits_lbl.pack(fill='x', pady=(8, 0))

        # ── Sensores ──────────────────────────────────────────────────
        card_conn = self._card(col_run, 'IMU sensors', expand=False)
        card_conn.master.pack_configure(pady=(12, 0))
        leds = tk.Frame(card_conn, bg=PANEL)
        leds.pack(fill='x')
        self._imu_leds: dict[int, dict] = {}
        for sid in range(1, len(IMU_IDS)):
            cv = tk.Canvas(leds, width=46, height=66, bg=PANEL, highlightthickness=0)
            cv.pack(side='left', expand=True)
            ring = cv.create_oval(6, 4, 38, 36, outline='', width=3)
            body = cv.create_oval(11, 9, 33, 31, fill=LED_OFF, outline=LED_RIM, width=1)
            glint = cv.create_oval(15, 12, 22, 18, fill='', outline='')
            cv.create_text(22, 47, text=f'ID {sid}', font=FONT_SMALL, fill=TEXT_MUTED)
            hz = cv.create_text(22, 60, text='', font=FONT_SMALL, fill=TEXT_DIM)
            self._imu_leds[sid] = {'cv': cv, 'ring': ring, 'body': body,
                                   'glint': glint, 'hz': hz}
        self._imu_msg_count: dict[int, int] = {}
        self._imu_rate_hz: dict[int, float] = {}
        self._imu_rate_t = time.monotonic()
        self._imu_live_lbl = tk.Label(card_conn, text='', font=FONT_SMALL,
                                      bg=PANEL, fg=TEXT_DIM, anchor='w',
                                      justify='left', wraplength=300)
        self._imu_live_lbl.pack(fill='x', pady=(6, 0))
        tk.Label(card_conn, font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
                 justify='left', wraplength=300,
                 text='Green = live, amber = calibrating, grey = off; blue ring = '
                      'used by the current setup. Keep each sensor still for ~2 s '
                      'after it connects.'
                 ).pack(fill='x', pady=(4, 0))

        # ── Calibração ────────────────────────────────────────────────
        tk.Frame(card_conn, bg=BORDER, height=1).pack(fill='x', pady=(10, 8))
        tk.Label(card_conn, text='Calibration (guided, step by step)', font=FONT_LBL,
                 bg=PANEL, fg=TEXT, anchor='w').pack(fill='x')
        cal_btns = tk.Frame(card_conn, bg=PANEL)
        cal_btns.pack(fill='x', pady=(6, 0))
        for kind, text, tip in (
                ('AG', 'Acc + Gyro…', 'Accelerometer and gyroscope offsets, from the '
                 'same 10 s recording: keep the sensor STILL on a FLAT surface.'),
                ('MAG', 'Magnetometer…', 'Hard/soft iron fit, 30 s: rotate the '
                 'sensor SLOWLY in every direction, away from metal.')):
            b = tk.Button(cal_btns, text=text, command=lambda k=kind: self._imu_calibrate(k),
                          bg=BTN_NEUTRAL, fg=TEXT,
                          activebackground=_shade(BTN_NEUTRAL, -0.08),
                          font=FONT_LBL, relief='flat', bd=0, pady=6, cursor='hand2')
            b.pack(side='left', fill='x', expand=True, padx=(0, 3) if kind == 'AG' else (3, 0))
            _Tooltip(b, tip)
        self._imu_cal_lbl = tk.Label(card_conn, text='', font=FONT_SMALL, bg=PANEL,
                                     fg=TEXT_DIM, anchor='w', justify='left',
                                     wraplength=300)
        self._imu_cal_lbl.pack(fill='x', pady=(6, 0))
        self._imu_calib: dict[int, dict] = {}
        self._imu_calib_pub = self.create_publisher(String, CALIB_REQ_TOPIC, 10)
        self.create_subscription(String, CALIB_STATUS_TOPIC, self._imu_on_calib, 10)

        # ── Cadeia → TCP ──────────────────────────────────────────────
        card_cart = self._card(col_cfg, 'Hand → TCP (inverse kinematics)', expand=False)
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

        # ── Juntas travadas ───────────────────────────────────────────
        card_lock = self._card(col_cfg, 'Lock joints', expand=False)
        card_lock.master.pack_configure(pady=(12, 0))
        lock_row = tk.Frame(card_lock, bg=PANEL)
        lock_row.pack(fill='x')
        self._imu_lock_vars: dict[str, tk.BooleanVar] = {}
        for i, j in enumerate(ARM_JOINTS, start=1):
            v = tk.BooleanVar(value=j in locked_cfg)
            # Botão de alternância: travada = âmbar e escrito, à vista de longe.
            btn = tk.Checkbutton(lock_row, variable=v, indicatoron=False,
                                 bg=BTN_NEUTRAL, fg=TEXT, selectcolor=WARN,
                                 activebackground=_shade(BTN_NEUTRAL, -0.08),
                                 font=FONT_LBL, relief='flat', bd=0, pady=6,
                                 highlightthickness=0, cursor='hand2')
            btn.pack(side='left', fill='x', expand=True, padx=2)
            v.trace_add('write', lambda *_a, b=btn, jv=v, n=i: (
                b.config(text=f'J{n} · locked' if jv.get() else f'J{n}'),
                self._imu_save_config()))
            btn.config(text=f'J{i} · locked' if v.get() else f'J{i}')
            self._imu_lock_vars[j] = v
        tk.Label(card_lock, font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
                 justify='left', wraplength=640,
                 text='Locked joints never move; the IK reaches the target with '
                      'the others as well as it can (IK err grows when the locks '
                      'remove a needed direction).').pack(fill='x', pady=(4, 0))

        self._imu_refresh_gate_lbl()
        self._imu_after = self.root.after(IMU_TICK_MS, self._imu_tick)
        if TK_STALL_S > 0:
            threading.Thread(target=self._imu_watchdog, daemon=True,
                             name='imu-tk-watchdog').start()

    def _imu_watchdog(self) -> None:
        """Se o laço do Tk parar (GUI congelada), despeja a pilha de TODAS as
        threads no stderr — vai para o log do launch e mostra onde travou.
        Uma vez por travamento."""
        dumped = False
        while not getattr(self, '_imu_shutting_down', False):
            time.sleep(1.0)
            stalled = time.monotonic() - self._imu_last_tick > TK_STALL_S
            if stalled and not dumped:
                print(f'[IMU] laço do Tk parado há > {TK_STALL_S:.0f} s — pilhas:',
                      file=sys.stderr, flush=True)
                faulthandler.dump_traceback(file=sys.stderr, all_threads=True)
            dumped = stalled

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
            t = time.monotonic()
            self._imu_latest[sensor_id] = (t, q)
            self._imu_msg_count[sensor_id] = self._imu_msg_count.get(sensor_id, 0) + 1
            rec = self._imu_rec
            if rec is not None:
                rec.imu(sensor_id, msg, t)

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
        if not hasattr(self, '_imu_rec_var'):
            return   # ainda construindo
        try:
            cart = {k: v.get() for k, v in self._imu_cart_vars.items()}
            cart['segments'] = {k: {'imu': i.get(), 'length_m': L.get()}
                                for k, (i, L) in self._imu_seg_vars.items()}
        except (tk.TclError, ValueError):
            return   # spinbox no meio da digitação
        data = {
            'cartesian': cart,
            'locked': [j for j, v in self._imu_lock_vars.items() if v.get()],
            'record': bool(self._imu_rec_var.get()),
        }
        try:
            os.makedirs(CONFIG_DIR, exist_ok=True)
            with open(IMU_CONFIG_FILE, 'w') as fh:
                json.dump(data, fh, indent=2)
        except OSError as exc:
            self.get_logger().warn(f'imu config not saved: {exc}')

    def _imu_on_cart_change(self) -> None:
        self._imu_save_config()
        if self._imu_cart is not None:
            self._imu_cart_rezero()

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
        if ik_step is None:
            self._set_status('IMU: tracking needs manip3d (IK).', DANGER)
            return
        if all(self._imu_locked_mask()):
            self._set_status('IMU: all joints are locked.', WARN)
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
        locked = [j for j, v in self._imu_lock_vars.items() if v.get()]
        if locked:
            what += f' (locked: {", ".join(locked)})'
        if self._imu_rec_var.get():
            self._imu_rec_open(q_deg, locked)
        self._imu_tracking = True
        self._imu_track_btn.config(text='■  Stop tracking', bg=DANGER,
                                   activebackground=_shade(DANGER, -0.08))
        self._imu_refresh_gate_lbl()
        self._set_status(f'IMU tracking: {what}.', OK)

    # ── LEDs e calibração ────────────────────────────────────────────
    def _imu_calib_active(self, sid: int, now: float):
        st = self._imu_calib.get(sid)
        if st is None:
            return None
        if st['state'] in ('started', 'progress'):
            return st
        return st if now - st['t'] < CALIB_SHOW_S else None

    def _imu_render_leds(self, now: float, live: set, used: set) -> None:
        blink = int(now * 3) % 2 == 0
        for sid, led in self._imu_leds.items():
            cv, cal = led['cv'], self._imu_calib_active(sid, now)
            busy = cal is not None and cal['state'] in ('started', 'progress')
            if busy:
                fill = WARN if blink else _shade(WARN, 0.35)
            elif sid in live:
                fill = OK
            else:
                fill = LED_OFF
            cv.itemconfig(led['body'], fill=fill)
            cv.itemconfig(led['glint'], fill=_shade(fill, 0.55) if fill != LED_OFF else '')
            cv.itemconfig(led['ring'], outline=PRIMARY if sid in used else '')
            if busy:
                txt, col = f'{cal.get("progress", 0):.0f}%', WARN
            elif sid in live:
                txt, col = f'{self._imu_rate_hz.get(sid, 0):.0f} Hz', TEXT_MUTED
            else:
                txt, col = 'off', TEXT_DIM
            cv.itemconfig(led['hz'], text=txt, fill=col)

    def _imu_on_calib(self, msg: String) -> None:
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        st['t'] = time.monotonic()
        sid = st.get('sensor')
        # Rebind atômico — o tick do Tk lê este dict.
        self._imu_calib = {**self._imu_calib, (int(sid) if sid is not None else 0): st}

    def _imu_render_calib(self, now: float) -> None:
        lines, color = [], TEXT_DIM
        for sid in sorted(self._imu_calib):
            st = self._imu_calib_active(sid, now)
            if st is None:
                continue
            name = CALIB_NAMES.get(st.get('kind'), 'Calibration')
            who = f'IMU {sid}' if sid else 'IMU'
            if st['state'] in ('started', 'progress'):
                lines.append(f'{who}: {name} {st.get("progress", 0):.0f}% — '
                             f'{CALIB_HINT.get(st.get("kind"), "")}')
                color = WARN
            elif st['state'] == 'done':
                cov = f' (radius CoV {st["cov_pct"]:.1f}%)' if 'cov_pct' in st else ''
                lines.append(f'{who}: {name} done{cov}. Keep it still ~2 s.')
                color = OK if color == TEXT_DIM else color
            elif st['state'] == 'cancelled':
                lines.append(f'{who}: {name} cancelled, nothing saved.')
            else:
                lines.append(f'{who}: {name} failed — {st.get("msg", "")}')
                color = DANGER
        self._imu_cal_lbl.config(text='\n'.join(lines), fg=color)

    def _imu_calibrate(self, kind: str) -> None:
        """Abre o assistente (gui_imu_calib) já com o tipo escolhido."""
        if self._imu_tracking:
            self._set_status('IMU: stop tracking before calibrating.', WARN)
            return
        if self._imu_wizard is not None:
            self._imu_wizard.lift()
            return
        self._imu_wizard = ImuCalibWizard(self, kind, targets=sorted(self._imu_live))

    def _imu_calib_send(self, kind: str, target: str) -> None:
        self._imu_calib_pub.publish(String(data=f'{kind} {target}'))

    # ── Gravação ─────────────────────────────────────────────────────
    def _imu_rec_open(self, q_deg: dict[str, float], locked: list[str]) -> None:
        cart = {k: v.get() for k, v in self._imu_cart_vars.items()}
        cart['segments'] = {k: {'imu': i.get(), 'length_m': L.get()}
                            for k, (i, L) in self._imu_seg_vars.items()}
        meta = {
            'cartesian': cart, 'locked': locked,
            'start_q_deg': q_deg,
            'home_deg': dict(getattr(self, '_arm_home_deg', {})),
            'robot_mode': self._robot_mode, 'mirror_to_real': self._imu_mirror_on,
            'limits': {'max_deg_s': IMU_MAX_DEG_S, 'max_tcp_m_s': IMU_MAX_TCP_M_S,
                       'max_tcp_deg_s': IMU_MAX_TCP_DEG_S, 'box_m': IMU_BOX_M,
                       'stale_s': IMU_STALE_S},
        }
        drv = getattr(self, '_real_driver', None)
        reader = (drv.read_joints_urdf_latest
                  if drv is not None and getattr(self, '_robot_connected', False)
                  else None)
        try:
            self._imu_rec = TrackingRecorder(meta, real_reader=reader,
                                             stale_s=IMU_STALE_S)
        except OSError as exc:
            self._imu_rec = None
            self._imu_rec_lbl.config(text=f'Recording failed: {exc}', fg=DANGER)
            return
        self._imu_rec_lbl.config(text=f'● REC  {self._imu_rec.path}', fg=DANGER)

    def _imu_rec_close(self) -> str:
        """Fecha a gravação; devolve um resumo curto para a barra de status."""
        rec, self._imu_rec = self._imu_rec, None
        if rec is None:
            return ''
        meta = rec.close()
        freezes = {sid: v['gaps_over_stale'] for sid, v in meta['imu_summary'].items()
                   if v['gaps_over_stale']}
        frz = (', '.join(f'IMU {k}: {n}×' for k, n in freezes.items())
               if freezes else 'no freezes')
        self._imu_rec_lbl.config(
            text=f'Saved {meta["duration_s"]:.0f} s → {rec.path}\n'
                 f'Gaps > {IMU_STALE_S:.1f} s: {frz}',
            fg=WARN if freezes else TEXT_MUTED)
        return f' Recorded to {rec.path} ({frz}).'

    def _imu_rec_tick(self, now: float, used: set, live: set, published: bool) -> None:
        st = self._imu_cart
        info = st.get('tick') or {}
        self._imu_rec.robot(
            now, frozen=not info, stale_ids=sorted(used - live), published=published,
            q_cmd_deg=np.degrees(st['q']).tolist(),
            q_sim_rad=getattr(self, '_latest_joint_rad', None),
            p_cmd_m=st.get('p_cmd'), tcp_m=info.get('tcp'),
            ik_err_mm=info.get('err'), v_tcp_mms=st['v_tcp'] * 1000.0,
            singular=info.get('singular', False), at_limit=info.get('at_limit', False))

    def _imu_stop_tracking(self, msg: str | None, color: str = TEXT_DIM) -> None:
        self._imu_tracking = False
        self._imu_cart = None
        saved = self._imu_rec_close()
        if msg and saved:
            msg += saved
        if getattr(self, '_imu_track_btn', None) is not None:
            self._imu_track_btn.config(text='▶  Start tracking', bg=OK,
                                       activebackground=_shade(OK, -0.08))
            self._imu_cart_lbl.config(text='Idle.', fg=TEXT_DIM)
            self._imu_refresh_gate_lbl()
        if msg:
            self._set_status(msg, color)

    def _imu_go_home(self) -> None:
        """Para o rastreio e leva o braço à Home da GUI (a do Controle Manual)."""
        if self._imu_tracking:
            self._imu_stop_tracking(None)
        reason = self._imu_block_reason()
        if reason:
            self._set_status(f'Home blocked: {reason}.', WARN)
            return
        self._apply_arm_home()
        self._set_status('IMU: tracking stopped, arm going Home.', OK)

    def _imu_locked_mask(self) -> np.ndarray:
        return np.array([self._imu_lock_vars[j].get() for j in ARM_JOINTS], dtype=bool)

    def _imu_zero(self) -> None:
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
        self._imu_gate_lbl.config(text=txt, fg=color)

    def _imu_shutdown(self) -> None:
        self._imu_shutting_down = True
        if getattr(self, '_imu_after', None) is not None:
            try:
                self.root.after_cancel(self._imu_after)
            except Exception:
                pass
            self._imu_after = None
        self._imu_tracking = False
        rec, self._imu_rec = self._imu_rec, None
        if rec is not None:
            rec.close()   # janela fechando: sem mexer em widgets

    # ── Laço (Tk, 30 Hz) ─────────────────────────────────────────────
    def _imu_tick(self) -> None:
        self._imu_after = None
        now = time.monotonic()
        dt = min(0.1, now - self._imu_last_tick)
        self._imu_last_tick = now

        live = {s for s in self._imu_latest if self._imu_fresh_q(s, now) is not None}
        self._imu_live = live
        ori, segs, *_r, f_ori, f_pos = self._imu_cart_spec()
        used = ({ori} if f_ori else set()) | ({s for s, _L in segs} if f_pos else set())
        if now - self._imu_rate_t >= 1.0:
            counts, self._imu_msg_count = self._imu_msg_count, {}
            span = now - self._imu_rate_t
            self._imu_rate_hz = {k: n / span for k, n in counts.items()}
            self._imu_rate_t = now
        self._imu_render_leds(now, live, used)
        self._imu_render_calib(now)
        missing = sorted(used - live)
        if not live:
            txt, color = 'No IMU data — is imu_node running and are the ESPs on?', WARN
        elif missing:
            txt, color = f'Used but silent: {", ".join(map(str, missing))}', WARN
        else:
            txt, color = f'{len(live)} live, all used sensors OK.', OK
        self._imu_live_lbl.config(text=txt, fg=color)

        if self._imu_tracking:
            reason = self._imu_block_reason()
            if reason:
                self._imu_stop_tracking(f'IMU tracking stopped: {reason}.', WARN)

        if self._imu_tracking and self._imu_cart is not None:
            q_new_deg = self._imu_tick_cartesian(now, dt)
            published = (q_new_deg is not None
                         and self._imu_publish_if_changed(q_new_deg))
            if self._imu_rec is not None:
                self._imu_rec_tick(now, used, live, published)

        self._imu_after = self.root.after(IMU_TICK_MS, self._imu_tick)

    def _imu_tick_cartesian(self, now: float, dt: float):
        st = self._imu_cart
        st['tick'] = None   # só ganha conteúdo se o tick andar (ver gravação)
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
        res = ik_step(st['q'], st['p_cmd'], T_end=st['T_end'], R_lock=st['R_cmd'],
                      locked=self._imu_locked_mask())
        # Teto por junta também no cartesiano: perto de singularidade a IK
        # pode pedir um salto grande de punho por um passo pequeno do TCP.
        max_dq = _math.radians(IMU_MAX_DEG_S) * dt
        st['q'] = st['q'] + np.clip(res.q - st['q'], -max_dq, max_dq)
        v = float(np.linalg.norm(st['p_cmd'] - p_prev)) / max(dt, 1e-3)
        st['v_tcp'] = 0.8 * st['v_tcp'] + 0.2 * v
        d = (st['p_cmd'] - st['p_tcp0']) * 1000.0
        T = tcp_pose(st['q'], T_end=st['T_end'])
        err = float(np.linalg.norm(T[:3, 3] - st['p_cmd'])) * 1000.0
        st['tick'] = {'tcp': T[:3, 3].copy(), 'err': err,
                      'singular': res.singular, 'at_limit': res.at_limit}
        flags = ('  SINGULAR' if res.singular else '') + ('  AT LIMIT' if res.at_limit else '')
        self._imu_cart_lbl.config(
            text=f'ΔTCP  x {d[0]:+5.0f}  y {d[1]:+5.0f}  z {d[2]:+5.0f} mm\n'
                 f'v {st["v_tcp"] * 1000:4.0f} mm/s   IK err {err:4.1f} mm{flags}',
            fg=WARN if (flags or err > 5.0) else TEXT)
        return {j: _math.degrees(float(v)) for j, v in zip(ARM_JOINTS, st['q'])}

    def _imu_publish_if_changed(self, q_deg: dict[str, float]) -> bool:
        pub = self._imu_published_deg or {}
        if not any(abs(q_deg[j] - pub.get(j, 1e9)) > IMU_PUBLISH_DEADBAND_DEG
                   for j in ARM_JOINTS):
            return False
        with self._suppress():
            for j in ARM_JOINTS:
                self.arm_sliders[j].set(round(q_deg[j], 1))
        self._publish_arm_q([_math.radians(q_deg[j]) for j in ARM_JOINTS],
                            MANIP_TRAJ_DURATION_S)
        self._imu_published_deg = dict(q_deg)
        return True
