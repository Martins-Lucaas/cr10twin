"""Testes da aba IMU (gui_imu.IMUMixin) sobre um hospedeiro de mentira.

O mixin é montado com o mínimo que ele consome do PalpationGUI e o laço de
30 Hz é girado à mão, com mensagens sensor_msgs/Imu sintéticas. Cobre o que
importa para a segurança e para o comportamento:
  * modo JOINT: junta segue o twist do IMU, inversão, junta desligada parada,
    teto de velocidade, publica só quando muda, congela com dado velho;
  * modo CARTESIAN: IMU de orientação gira o TCP; a cadeia de segmentos
    desloca o TCP — conferido pela FK da pose publicada;
  * E-STOP interrompe o rastreio.

Pula sozinho onde não há display.
"""
import contextlib
import math
import threading
import time
import tkinter as tk

import numpy as np
import pytest
from sensor_msgs.msg import Imu

from touch_pack import gui_imu
from touch_pack.constants import ARM_JOINTS
from touch_pack.gui_constants import ARM_LIMITS_DEG
from touch_pack.imu_math import rotvec
from touch_pack.kinematics import T_HAND_ATTACH
from touch_pack.manip3d import tcp_pose

DT = gui_imu.IMU_TICK_MS / 1000.0
# Pose de trabalho longe de singularidade (mesma ideia do test_manip3d).
Q_START_DEG = {'joint1': 0.0, 'joint2': -30.0, 'joint3': -90.0,
               'joint4': 0.0, 'joint5': 60.0, 'joint6': 0.0}


@pytest.fixture(scope='module')
def _tk_root():
    try:
        r = tk.Tk()
    except tk.TclError as exc:                       # pragma: no cover
        pytest.skip(f'sem display: {exc}')
    r.withdraw()
    yield r
    r.destroy()


class _Logger:
    def warn(self, *_a):
        pass


class _Host(gui_imu.IMUMixin):
    """O mínimo que o IMUMixin espera do PalpationGUI."""

    def __init__(self, root):
        self.root = root
        self._lock = threading.Lock()
        self._latest_phase = 'IDLE'
        self._estop_latched = False
        self._drag_enabled = False
        self._exec_movement_id = None
        self._robot_mode = 'SIM_ONLY'
        self.arm_sliders = {j: tk.DoubleVar(root, value=Q_START_DEG[j])
                            for j in ARM_JOINTS}
        self.published: list[list[float]] = []
        self.status = ''
        self.subs = []

    def create_subscription(self, _type, topic, cb, _qos):
        self.subs.append((topic, cb))

    def get_logger(self):
        return _Logger()

    def _card(self, parent, _title, expand=False):
        card = tk.Frame(parent)
        card.pack(fill='x')
        inner = tk.Frame(card)
        inner.pack(fill='x')
        return inner

    def _clamp_var(self, var, vmin, vmax, default=None):
        try:
            return max(vmin, min(vmax, float(var.get())))
        except (ValueError, tk.TclError):
            return default

    @contextlib.contextmanager
    def _suppress(self):
        yield

    def _publish_arm_q(self, q_rad, _duration_s):
        self.published.append(list(q_rad))

    def _set_status(self, text, _color=None):
        self.status = text

    def _manip_T_end(self):
        return T_HAND_ATTACH

    # ── utilitários do teste ──
    def feed(self, sid, q_wxyz):
        m = Imu()
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = (
            float(v) for v in q_wxyz)
        self._imu_on_msg(sid, m)

    def tick(self, n=1, feeds=None):
        for _ in range(n):
            for sid, q in (feeds or {}).items():
                self.feed(sid, q)
            self._imu_last_tick = time.monotonic() - DT
            self._imu_tick()
            self.root.after_cancel(self._imu_after)
            self._imu_after = None


def _q(axis, deg):
    a = math.radians(deg) / 2
    ax = np.asarray(axis, dtype=float)
    return np.array([math.cos(a), *(math.sin(a) * ax)])


@pytest.fixture
def host(_tk_root, tmp_path, monkeypatch):
    monkeypatch.setattr(gui_imu, 'IMU_CONFIG_FILE', str(tmp_path / 'imu_config.json'))
    monkeypatch.setattr(gui_imu, 'CONFIG_DIR', str(tmp_path))
    frame = tk.Frame(_tk_root)
    h = _Host(frame)
    h._build_imu_tab(frame)
    h.root.after_cancel(h._imu_after)
    yield h
    frame.destroy()


def test_subscribes_one_topic_per_imu_id(host):
    assert [t for t, _cb in host.subs] == [f'/imu/sensor_{i}' for i in range(1, 7)]


def _map_joint(host, joint, sid, axis='Z', invert=False, on=True):
    r = host._imu_rows[joint]
    r['id'].set(str(sid))
    r['axis'].set(axis)
    r['inv'].set(invert)
    r['on'].set(on)


def test_joint_mode_follows_twist_with_invert_and_disabled_joint(host):
    _map_joint(host, 'joint1', 3)
    _map_joint(host, 'joint4', 3, invert=True)
    _map_joint(host, 'joint2', 5, on=False)
    still = {3: _q((0, 0, 1), 0.0), 5: _q((0, 0, 1), 0.0)}
    host.tick(feeds=still)
    host._imu_toggle_tracking()
    assert host._imu_tracking, host.status
    turned = {3: _q((0, 0, 1), 30.0), 5: _q((1, 0, 0), 50.0)}
    host.tick(40, feeds=turned)
    q = dict(zip(ARM_JOINTS, np.degrees(host.published[-1])))
    assert q['joint1'] == pytest.approx(30.0, abs=0.2)
    assert q['joint4'] == pytest.approx(-30.0, abs=0.2)
    assert q['joint2'] == pytest.approx(Q_START_DEG['joint2'])   # desligada
    assert q['joint3'] == pytest.approx(Q_START_DEG['joint3'])   # sem IMU


def test_joint_mode_respects_speed_limit(host):
    _map_joint(host, 'joint1', 3)
    host.tick(feeds={3: _q((0, 0, 1), 0.0)})
    host._imu_toggle_tracking()
    host.tick(feeds={3: _q((0, 0, 1), 90.0)})
    step = math.degrees(host.published[-1][0])
    assert 0.0 < step <= gui_imu.IMU_MAX_DEG_S * (DT + 0.005)   # dt real ≥ DT


def test_joint_mode_publishes_only_when_target_changes(host):
    _map_joint(host, 'joint1', 3)
    host.tick(feeds={3: _q((0, 0, 1), 0.0)})
    host._imu_toggle_tracking()
    host.tick(40, feeds={3: _q((0, 0, 1), 10.0)})
    n = len(host.published)
    host.tick(20, feeds={3: _q((0, 0, 1), 10.0)})
    assert len(host.published) == n


def test_joint_mode_freezes_on_stale_imu(host, monkeypatch):
    _map_joint(host, 'joint1', 3)
    host.tick(feeds={3: _q((0, 0, 1), 0.0)})
    host._imu_toggle_tracking()
    host.tick(5, feeds={3: _q((0, 0, 1), 20.0)})
    n = len(host.published)
    t0, q0 = host._imu_latest[3]
    host._imu_latest[3] = (t0 - 2 * gui_imu.IMU_STALE_S, q0)   # envelhece
    host.tick(10)
    assert len(host.published) == n
    assert 'frozen' in host.status


def test_start_refused_without_imu_data(host):
    _map_joint(host, 'joint1', 3)
    host._imu_toggle_tracking()
    assert not host._imu_tracking
    assert '3' in host.status


def test_estop_stops_tracking(host):
    _map_joint(host, 'joint1', 3)
    host.tick(feeds={3: _q((0, 0, 1), 0.0)})
    host._imu_toggle_tracking()
    host._estop_latched = True
    host.tick(feeds={3: _q((0, 0, 1), 0.0)})
    assert not host._imu_tracking
    assert 'E-STOP' in host.status


def _fk(q_rad):
    return tcp_pose(np.asarray(q_rad), T_end=T_HAND_ATTACH)


def _start_cartesian(host, feeds):
    host._imu_mode_var.set('CARTESIAN')
    host.tick(feeds=feeds)
    host._imu_toggle_tracking()
    assert host._imu_tracking, host.status
    host.tick(feeds=feeds)   # captura a referência
    return _fk(np.radians([Q_START_DEG[j] for j in ARM_JOINTS]))


def test_cartesian_orientation_imu_rotates_tcp_and_keeps_position(host):
    cv = host._imu_cart_vars
    cv['ori_imu'].set('6')
    cv['follow_pos'].set(False)
    T0 = _start_cartesian(host, {6: _q((0, 0, 1), 0.0)})
    host.tick(90, feeds={6: _q((0, 0, 1), 20.0)})
    T = _fk(host.published[-1])
    dR = T[:3, :3] @ T0[:3, :3].T
    assert np.degrees(rotvec(dR)) == pytest.approx([0.0, 0.0, 20.0], abs=0.5)
    assert np.linalg.norm(T[:3, 3] - T0[:3, 3]) < 0.002


def test_cartesian_chain_moves_tcp_position(host):
    cv = host._imu_cart_vars
    cv['ori_imu'].set('—')
    cv['limb_axis'].set('+X')
    up_id, up_len = host._imu_seg_vars['upper_arm']
    fo_id, fo_len = host._imu_seg_vars['forearm']
    up_id.set('3')
    up_len.set(0.30)
    fo_id.set('4')
    fo_len.set(0.25)
    feeds = {3: _q((0, 0, 1), 0.0), 4: _q((0, 0, 1), 0.0)}
    T0 = _start_cartesian(host, feeds)
    # antebraço sobe 20° (pitch em torno de -y do mundo): a mão sobe e recua
    host.tick(90, feeds={3: _q((0, 0, 1), 0.0), 4: _q((0, -1, 0), 20.0)})
    a = math.radians(20.0)
    expected = 0.25 * np.array([math.cos(a) - 1.0, 0.0, math.sin(a)])
    T = _fk(host.published[-1])
    assert T[:3, 3] - T0[:3, 3] == pytest.approx(expected, abs=0.003)
    # orientação travada (follow orientation sem IMU)
    assert np.linalg.norm(rotvec(T[:3, :3] @ T0[:3, :3].T)) < math.radians(0.5)
    for j, v in zip(ARM_JOINTS, np.degrees(host.published[-1])):
        lo, hi = ARM_LIMITS_DEG[j]
        assert lo <= v <= hi


def test_cartesian_start_refused_without_sources(host):
    cv = host._imu_cart_vars
    cv['ori_imu'].set('—')
    host._imu_mode_var.set('CARTESIAN')
    host._imu_toggle_tracking()
    assert not host._imu_tracking


def test_config_persists(host):
    _map_joint(host, 'joint6', 6, axis='X', invert=True)
    host._imu_cart_vars['heading'].set(45.0)
    data = host._imu_load_config()
    assert data['joints']['joint6'] == {'enabled': True, 'imu': '6',
                                        'axis': 'X', 'invert': True}
    assert data['cartesian']['heading'] == 45.0
