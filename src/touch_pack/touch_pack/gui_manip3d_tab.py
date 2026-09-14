"""gui_manip3d_tab.py — a aba da viewport 3D: gate, opções e arrasto.

`manip3d.py` desenha e trata o mouse; o que está aqui é a COLA entre essa
viewport e o resto da GUI — de onde ela lê as juntas, o que o arrasto pode
comandar, e sob que condições ele fica disponível.

O gate é a parte que merece atenção: arrastar o TCP publica trajetória no
braço. Ele só libera com a fonte de juntas viva e o modo certo — porque um
arrasto aceito sobre um estado velho move o braço real para onde o cursor
está, e não para onde o operador acha que está apontando.

Recortado de `palpation_gui.py` — os métodos operam sobre `self` como antes.
"""
from __future__ import annotations

from typing import Any

# A cena do URDF é import OPCIONAL (depende de trimesh/urdf_scene). Sem ela a
# viewport abre vazia em vez de a GUI inteira recusar subir.
# A viewport, a FK do TCP e o backend VTK são todos imports OPCIONAIS: numa
# máquina sem eles a aba 3D fica indisponível com motivo, e o resto da GUI
# (que é o que roda o ensaio) abre normalmente.
try:
    from .manip3d import (
        Manip3DView, rpy_deg as _rpy_deg,             # noqa: F401
        MAX_LIN_STEP_M as _MANIP_MAX_LIN_M,
        MAX_JOINT_STEP_RAD as _MANIP_MAX_DQ,
    )
    _MANIP3D_OK = True
except Exception:  # pragma: no cover
    Manip3DView: Any = None
    _rpy_deg: Any = None
    _MANIP_MAX_LIN_M = 0.015
    _MANIP_MAX_DQ = 0.06
    _MANIP3D_OK = False

try:
    from .kinematics import (
        forward_kinematics as _fk_tcp,
        T_TOUCH_TOOL_ATTACH as _T_TCP,
        T_HAND_ATTACH as _T_HAND,
    )
except Exception:  # pragma: no cover
    _fk_tcp: Any = None
    _T_TCP: Any = None
    _T_HAND: Any = None

try:
    from .vtk_render import vtk_available as _manip_vtk_available
except Exception:  # pragma: no cover
    def _manip_vtk_available() -> bool:
        return False

try:
    from .urdf_scene import (
        build_scene as _build_scene,
        coarse_scene as _coarse_scene,
        DEFAULT_TRIANGLE_BUDGET as _SCENE_BUDGET,
    )
except Exception:  # pragma: no cover
    _build_scene: Any = None
    _coarse_scene: Any = None
    _SCENE_BUDGET = 5000
    _URDF_SCENE_OK = False
else:
    _URDF_SCENE_OK = True

import math as _math
import numpy as np
import threading
import tkinter as tk
from .constants import ARM_JOINTS
from .gui_constants import ARM_LIMITS_DEG, MANIP_TRAJ_DURATION_S
from .ui_helpers import (
    BG,
    BORDER,
    BTN_NEUTRAL,
    DANGER,
    FONT_LBL,
    FONT_SMALL,
    OK,
    PANEL,
    PRIMARY,
    PRIMARY_HV,
    TEXT,
    TEXT_DIM,
    TEXT_MUTED,
    WARN,
    _Tooltip,
    _shade,
)


class Manip3DTabMixin:
    """gui_manip3d_tab.py — a aba da viewport 3D: gate, opções e arrasto."""

    def _manip_T_end(self) -> np.ndarray | None:
        """Transform flange→TCP do efetuador com que a célula foi aberta."""
        if self._end_effector == 'hand' and _T_HAND is not None:
            return _T_HAND
        return _T_TCP
    def _build_manip3d_tab(self, root: tk.Frame) -> None:
        body = tk.Frame(root, bg=BG)
        body.pack(fill='both', expand=True, padx=18, pady=18)

        col_view = tk.Frame(body, bg=BG)
        col_side = tk.Frame(body, bg=BG, width=330)
        col_view.pack(side='left', fill='both', expand=True, padx=(0, 12))
        col_side.pack(side='right', fill='both')
        col_side.pack_propagate(False)

        self._manip_view = Manip3DView(
            col_view,
            on_q=self._manip_on_q,
            q_provider=self._manip_q_provider,
            on_state=self._manip_on_state,
            on_drag_change=self._manip_on_drag_change,
            T_end=self._manip_T_end())
        self._manip_view.pack(fill='both', expand=True)

        # TCP ao vivo
        card_tcp = self._card(col_side, 'TCP — live pose', expand=False)
        self._manip_x_lbl = self._kv(card_tcp, 'x', '—')
        self._manip_y_lbl = self._kv(card_tcp, 'y', '—')
        self._manip_z_lbl = self._kv(card_tcp, 'z', '—')
        tk.Frame(card_tcp, bg=BORDER, height=1).pack(fill='x', pady=6)
        self._manip_roll_lbl  = self._kv(card_tcp, 'roll',  '—')
        self._manip_pitch_lbl = self._kv(card_tcp, 'pitch', '—')
        self._manip_yaw_lbl   = self._kv(card_tcp, 'yaw',   '—')
        tk.Frame(card_tcp, bg=BORDER, height=1).pack(fill='x', pady=6)
        self._manip_err_lbl = self._kv(card_tcp, 'tracking lag', '—')
        self._manip_manip_lbl = self._kv(card_tcp, 'manipulability', '—')

        # Opções do arrasto
        card_opt = self._card(col_side, 'Drag options', expand=False)
        self._manip_lock_var = tk.BooleanVar(value=True)
        self._manip_chk(
            card_opt, 'Lock tool orientation', self._manip_lock_var,
            self._manip_apply_options,
            'Keeps the current TCP orientation while you drag: the mouse '
            'moves the POINT, the wrist follows to preserve the attitude. '
            'Unchecking frees the wrist — the arm reaches the point with '
            'whatever orientation the DLS finds, which is looser but goes '
            'further before hitting a joint limit.')
        self._manip_mirror_var = tk.BooleanVar(value=False)
        self._manip_mirror_chk = self._manip_chk(
            card_opt, 'Mirror to the real CR10', self._manip_mirror_var,
            self._manip_apply_options,
            'OFF (default): dragging moves ONLY the simulated arm. ON (and '
            'in MIRROR mode): the pose reached is sent to the real CR10 as '
            'MovJ once the drag settles — the 80 ms debounce means the '
            'hardware follows the RESULT of the drag, not every frame of it.')

        tk.Label(card_opt, text='Drag axis', font=FONT_LBL, bg=PANEL,
                 fg=TEXT, anchor='w').pack(fill='x', pady=(10, 2))
        self._manip_axis_var = tk.StringVar(value='FREE')
        self._manip_axis_btns: dict[str, tk.Button] = {}
        row_axis = tk.Frame(card_opt, bg=PANEL); row_axis.pack(fill='x')
        for code, label in (('FREE', 'Free'), ('X', 'X'),
                            ('Y', 'Y'), ('Z', 'Z')):
            b = tk.Button(row_axis, text=label, font=FONT_LBL,
                          relief='flat', bd=0, padx=8, pady=5,
                          cursor='hand2', highlightthickness=0,
                          command=lambda c=code: self._on_manip_axis(c))
            b.pack(side='left', fill='x', expand=True, padx=1)
            self._manip_axis_btns[code] = b
        _Tooltip(row_axis,
                 'Free = the TCP follows the mouse on the plane parallel to '
                 'the screen. X/Y/Z = the motion is projected onto that world '
                 'axis, so you can descend in Z without drifting sideways.')

        adv = self._collapsible(card_opt, 'Advanced — IK step')
        self._manip_step_var = tk.DoubleVar(
            value=round(_MANIP_MAX_LIN_M * 1000.0, 1))
        self._param_row(adv, label='Max linear step', unit='mm',
                        var=self._manip_step_var,
                        vmin=1.0, vmax=50.0, step=1.0, snap=0.5,
                        hint='Ceiling on the TCP travel attacked per IK '
                             'iteration (6 per 30 ms tick). Lower = the arm '
                             'trails the cursor more softly; higher = it '
                             'snaps to the mouse but may lurch.')
        self._manip_dq_var = tk.DoubleVar(
            value=round(_math.degrees(_MANIP_MAX_DQ), 1))
        self._param_row(adv, label='Max joint step', unit='°',
                        var=self._manip_dq_var,
                        vmin=0.5, vmax=10.0, step=0.5, snap=0.5,
                        hint='Ceiling on each joint per IK iteration. It is '
                             'what keeps the pose continuous near a '
                             'singularity, where the DLS would otherwise ask '
                             'for a huge wrist swing.')
        self._manip_step_var.trace_add(
            'write', lambda *_: self._manip_apply_options())
        self._manip_dq_var.trace_add(
            'write', lambda *_: self._manip_apply_options())

        # Câmera + ações
        card_act = self._card(col_side, 'View & actions', expand=False)
        row_view = tk.Frame(card_act, bg=PANEL); row_view.pack(fill='x')
        for code, label in (('iso', 'Iso'), ('top', 'Top'),
                            ('front', 'Front'), ('side', 'Side')):
            tk.Button(row_view, text=label, font=FONT_LBL,
                      bg=BTN_NEUTRAL, fg=TEXT,
                      activebackground=_shade(BTN_NEUTRAL, -0.08),
                      activeforeground=TEXT,
                      relief='flat', bd=0, padx=8, pady=5, cursor='hand2',
                      highlightthickness=0,
                      command=lambda c=code: self._manip_set_view(c)
                      ).pack(side='left', fill='x', expand=True, padx=1)

        row_act = tk.Frame(card_act, bg=PANEL); row_act.pack(fill='x',
                                                             pady=(8, 0))
        tk.Button(row_act, text='⟲  Sync from scene',
                  command=self._manip_sync_from_scene,
                  bg=_shade(PRIMARY, 0.25), fg=PRIMARY,
                  activebackground=_shade(PRIMARY, 0.15),
                  activeforeground=PRIMARY,
                  font=FONT_LBL, relief='flat', bd=0, padx=10, pady=6,
                  cursor='hand2'
                  ).pack(side='left', fill='x', expand=True, padx=(0, 4))
        tk.Button(row_act, text='⌖  Capture pose',
                  command=self._manip_capture_pose,
                  bg=_shade(OK, 0.25), fg=OK,
                  activebackground=_shade(OK, 0.15), activeforeground=OK,
                  font=FONT_LBL, relief='flat', bd=0, padx=10, pady=6,
                  cursor='hand2'
                  ).pack(side='left', fill='x', expand=True, padx=(4, 0))

        tk.Button(card_act, text='⌂  Home',
                  command=self._manip_go_home,
                  bg=PRIMARY, fg='white',
                  activebackground=PRIMARY_HV, activeforeground='white',
                  font=FONT_LBL, relief='flat', bd=0, padx=10, pady=7,
                  cursor='hand2').pack(fill='x', pady=(4, 0))

        self._manip_gate_lbl = tk.Label(
            col_side, text='', font=FONT_SMALL, bg=BG, fg=TEXT_DIM,
            anchor='w', justify='left', wraplength=310)
        self._manip_gate_lbl.pack(fill='x', pady=(10, 0))

        self._on_manip_axis('FREE')
        self._manip_apply_options()
        self._manip_sync_from_scene(quiet=True)
    def _manip_chk(self, parent, text: str, var: tk.BooleanVar,
                   command, hint: str = '') -> tk.Checkbutton:
        chk = tk.Checkbutton(parent, text=text, variable=var,
                             command=command, bg=PANEL, fg=TEXT,
                             activebackground=PANEL, activeforeground=TEXT,
                             selectcolor=PANEL, font=FONT_LBL,
                             anchor='w', relief='flat', bd=0,
                             highlightthickness=0, cursor='hand2')
        chk.pack(fill='x', pady=2)
        if hint:
            _Tooltip(chk, hint)
        return chk
    def _on_manip_axis(self, code: str) -> None:
        self._manip_axis_var.set(code)
        for c, btn in self._manip_axis_btns.items():
            on = (c == code)
            btn.config(bg=PRIMARY if on else BTN_NEUTRAL,
                       fg='white' if on else TEXT,
                       activebackground=PRIMARY_HV if on
                       else _shade(BTN_NEUTRAL, -0.08),
                       activeforeground='white' if on else TEXT)
        if self._manip_view is not None:
            self._manip_view.axis_constraint = code
    def _manip_set_view(self, name: str) -> None:
        if self._manip_view is not None:
            self._manip_view.set_view(name)
    def _manip_apply_options(self) -> None:
        """Reflete os widgets do painel no estado da viewport."""
        view = self._manip_view
        if view is None:
            return
        view.lock_orientation = bool(self._manip_lock_var.get())
        self._manip_mirror_on = bool(self._manip_mirror_var.get())
        try:
            view.max_lin_m = max(0.001, float(self._manip_step_var.get())
                                 / 1000.0)
        except (tk.TclError, ValueError):
            pass
        try:
            view.max_dq = max(0.005, _math.radians(
                float(self._manip_dq_var.get())))
        except (tk.TclError, ValueError):
            pass
        self._manip_refresh_gate()
    def _manip_gate_reason(self) -> str:
        """Motivo pelo qual o arrasto está bloqueado ('' = liberado)."""
        with self._lock:
            phase = self._latest_phase
        if phase not in ('IDLE', 'DONE', 'ABORTED'):
            return f'palpation running ({phase}) — drag disabled'
        if self._drag_enabled:
            return 'drag teach active — release it first'
        if self._exec_movement_id is not None:
            return 'motion running — drag disabled'
        return ''
    def _manip_refresh_gate(self) -> None:
        view = self._manip_view
        if view is None:
            return
        reason = self._manip_gate_reason()
        view.enabled = not reason
        view.block_reason = reason
        if reason:
            txt, color = reason, WARN
        elif self._manip_mirror_on and self._robot_mode == 'MIRROR':
            txt = ('Mirroring ON — the real CR10 receives a MovJ when the '
                   'drag settles.')
            color = DANGER
        elif self._manip_mirror_on:
            txt = ('Mirroring is checked but the mode is SIM_ONLY — only '
                   'Gazebo moves.')
            color = TEXT_MUTED
        else:
            txt = 'Simulation only — the real arm is not commanded.'
            color = TEXT_DIM
        # Chamado a cada tick ocioso (33 Hz): só toca no widget quando o
        # texto muda de fato.
        if (txt, color) == getattr(self, '_manip_gate_last', None):
            return
        self._manip_gate_last = (txt, color)
        try:
            self._manip_gate_lbl.config(text=txt, fg=color)
        except (AttributeError, tk.TclError):
            pass
    def _manip_q_provider(self):
        """Pose que a viewport desenha quando ninguém está arrastando: a da
        cena (Gazebo ou, em MIRROR, o feedback do braço real que já alimenta
        /joint_states). Sem ela a viewport ficaria congelada enquanto o braço
        se move por outro caminho (palpação, movimento, drag teach)."""
        # Ponto único onde a aba respira quando ociosa: aproveita para
        # reavaliar o gate (fase da palpação, drag teach, modo do robô).
        self._manip_refresh_gate()
        q = self._latest_joint_rad
        if q is None or len(q) < 6:
            try:
                q = [_math.radians(float(self.arm_sliders[j].get()))
                     for j in ARM_JOINTS]
            except (AttributeError, ValueError, tk.TclError):
                return None
        q = [float(v) for v in q[:6]]
        if self._latest_extra_joints and self._manip_view is not None:
            self._manip_view.set_extra_joints(self._latest_extra_joints)
        # Readout ocioso: só reescreve os labels quando a cena mexeu de
        # verdade (o /joint_states do Gazebo chega a 50 Hz com ruído).
        prev = self._manip_readout_q
        if prev is None or any(abs(a - b) > 1e-4 for a, b in zip(q, prev)):
            self._manip_readout_q = q
            self._manip_update_readout(q)
        return q
    def _manip_on_q(self, q_rad) -> None:
        """Cada iteração do arrasto: publica no JTC e sincroniza os sliders."""
        reason = self._manip_gate_reason()
        if reason:
            # O gate pode FECHAR no meio do gesto (a palpação começou, o
            # drag teach foi detectado).
            self._manip_refresh_gate()
            if self._manip_view is not None:
                self._manip_view.abort_drag()
            self._set_status(f'3D drag interrupted — {reason}.', WARN)
            return
        self._publish_arm_q(q_rad, MANIP_TRAJ_DURATION_S)
        self._suppressing = True
        try:
            for i, j in enumerate(ARM_JOINTS):
                lo, hi = ARM_LIMITS_DEG[j]
                deg = _math.degrees(float(q_rad[i]))
                self.arm_sliders[j].set(round(max(lo, min(hi, deg)), 2))
        except (AttributeError, tk.TclError):
            pass
        finally:
            self._suppressing = False
    def _manip_on_state(self, q_rad, res) -> None:
        """Atualiza os números do painel a cada passo de IK."""
        self._manip_update_readout(q_rad, res)
    def _manip_update_readout(self, q_rad, res=None) -> None:
        T_end = self._manip_T_end()
        if _fk_tcp is None or _rpy_deg is None or T_end is None:
            return
        try:
            T = _fk_tcp(np.asarray(q_rad, dtype=float), T_end=T_end)
        except Exception:
            return
        p = T[:3, 3] * 1000.0
        roll, pitch, yaw = _rpy_deg(T[:3, :3])
        try:
            self._manip_x_lbl.config(text=f'{p[0]:+8.1f} mm')
            self._manip_y_lbl.config(text=f'{p[1]:+8.1f} mm')
            self._manip_z_lbl.config(text=f'{p[2]:+8.1f} mm')
            self._manip_roll_lbl.config(text=f'{roll:+7.1f} °')
            self._manip_pitch_lbl.config(text=f'{pitch:+7.1f} °')
            self._manip_yaw_lbl.config(text=f'{yaw:+7.1f} °')
            if res is None:
                self._manip_err_lbl.config(text='—', fg=TEXT)
                self._manip_manip_lbl.config(text='—')
                return
            lag_mm = res.pos_err_m * 1000.0
            if res.singular:
                lag_txt, lag_color = 'singular', DANGER
            elif res.at_limit:
                lag_txt, lag_color = f'{lag_mm:.1f} mm · limit', WARN
            else:
                lag_txt = f'{lag_mm:.1f} mm'
                lag_color = OK if lag_mm < 5.0 else WARN
            self._manip_err_lbl.config(text=lag_txt, fg=lag_color)
            self._manip_manip_lbl.config(text=f'{res.manip:.4f}')
        except (AttributeError, tk.TclError):
            pass
    def _manip_on_drag_change(self, active: bool) -> None:
        """Arma/desarma o gate do espelhamento no braço real."""
        if self._manip_release_after is not None:
            try:
                self.root.after_cancel(self._manip_release_after)
            except (tk.TclError, ValueError):
                pass
            self._manip_release_after = None
        if active:
            self._manip_active = True
            self._manip_refresh_gate()
            return
        self._manip_release_after = self.root.after(
            200, self._manip_clear_active)
    def _manip_clear_active(self) -> None:
        self._manip_release_after = None
        self._manip_active = False
    def _manip_sync_from_scene(self, quiet: bool = False) -> None:
        """Puxa a pose corrente da cena para a viewport (e para o readout)."""
        view = self._manip_view
        if view is None:
            return
        q = self._manip_q_provider()
        if q is None:
            if not quiet:
                self._set_status('No joint state available yet.', WARN)
            return
        view.set_q(q, force=True)
        self._manip_update_readout(q)
        self._manip_refresh_gate()
        if not quiet:
            self._set_status('3D view synced with the scene.', OK)
    def _manip_capture_pose(self) -> None:
        """Salva a pose atual da viewport na aba Poses & Motions."""
        view = self._manip_view
        if view is None:
            return
        q_deg = [_math.degrees(float(v)) for v in view.q]
        self._add_pose(q_deg, prefix='3D')
    def _manip_go_home(self) -> None:
        """Leva o braço à Home da GUI (mesma do Controle Manual)."""
        if self._manip_gate_reason():
            self._manip_refresh_gate()
            return
        self._apply_arm_home()
        q = [_math.radians(float(self._arm_home_deg[j])) for j in ARM_JOINTS]
        if self._manip_view is not None:
            self._manip_view.set_q(q, force=True)
        self._manip_update_readout(q)
    def _manip_load_scene(self) -> None:
        """Dispara a carga das malhas na PRIMEIRA vez que a aba é aberta."""
        if self._manip_scene_state != 'idle' or not _URDF_SCENE_OK:
            return
        self._manip_scene_state = 'loading'
        if self._manip_view is not None:
            self._manip_view.set_scene(None, 'loading robot meshes…')
        threading.Thread(target=self._manip_scene_worker,
                         daemon=True, name='manip3d-scene').start()
    def _manip_scene_worker(self) -> None:
        scene, error = None, ''
        self._manip_scene_coarse = None
        # Com VTK disponível a malha vai INTEIRA para a GPU — é a mesma
        # geometria que o Gazebo carrega, triângulo por triângulo.
        self._manip_exact = _MANIP3D_OK and _manip_vtk_available()
        try:
            scene = _build_scene(
                self._end_effector,
                description_path=self._robot_desc_path,
                triangle_budget=None if self._manip_exact else _SCENE_BUDGET)
            if not self._manip_exact:
                # Segunda malha, mais grossa, para usar durante o arrasto —
                # gerada aqui, na thread, junto com a principal.
                self._manip_scene_coarse = _coarse_scene(scene)
        except Exception as exc:
            error = str(exc)
            self.get_logger().warning(
                f'Viewport 3D: malhas indisponíveis ({error}) — '
                f'usando esqueleto.')
        # A carga da mão leva ~4 s; a janela pode ter sido fechada no meio.
        if self._stop_event.is_set():
            return
        try:
            self.root.after(0, self._manip_scene_done, scene, error)
        except (tk.TclError, RuntimeError) as exc:
            # Nunca engolir em silêncio: se o agendamento falha, a aba fica
            # eternamente em "loading" e ninguém sabe por quê.
            self._manip_scene_state = 'failed'
            self.get_logger().warning(
                f'Viewport 3D: entrega da cena falhou ({exc}).')
    def _manip_scene_done(self, scene, error: str) -> None:
        view = self._manip_view
        if scene is None:
            self._manip_scene_state = 'failed'
            if view is not None:
                view.set_scene(None, 'skeleton view — meshes unavailable')
            return
        self._manip_scene_state = 'ready'
        if view is not None:
            missing = len(getattr(scene, 'missing_meshes', ()))
            status = f'{missing} mesh(es) missing' if missing else ''
            view.set_scene(scene, status, coarse=self._manip_scene_coarse,
                           exact=self._manip_exact)
            if self._manip_exact and not view.rendering_exact:
                # A GPU recusou o contexto depois de a cena exata já estar
                # pronta.
                self._manip_exact = False
                self._manip_scene_state = 'idle'
                self.get_logger().warning(
                    'Viewport 3D: GPU indisponível — recarregando malha '
                    'reduzida para o rasterizador em software.')
                self._manip_load_scene()
                return
        self.get_logger().info(
            f'Viewport 3D: {len(scene.parts)} peças / '
            f'{scene.triangle_count} triângulos ({self._end_effector}, '
            f'{"malha exata / GPU" if self._manip_exact else "reduzida / CPU"}).')
