"""Testes da aba IMU (gui_imu.IMUMixin) sobre um hospedeiro de mentira.

O mixin é montado com o mínimo que ele consome do PalpationGUI e o laço de
30 Hz é girado à mão, com mensagens sensor_msgs/Imu sintéticas. Cobre o que
importa para a segurança e para o comportamento:
  * IMU de orientação gira o TCP; a cadeia de segmentos desloca o TCP —
    conferido pela FK da pose publicada;
  * juntas travadas não se movem;
  * publica só quando muda, congela com dado velho, E-STOP interrompe;
  * Home para o rastreio e manda o braço para a Home;
  * a gravação da sessão: IMU, robot.csv e o resumo de buracos no meta.json;
  * LEDs por ID e o pedido/andamento da calibração AG/MAG.

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

import csv
import json
import os

from touch_pack import gui_imu, gui_imu_calib, imu_recorder
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
        self.homed = False
        self.sent = []

    def create_subscription(self, _type, topic, cb, _qos):
        self.subs.append((topic, cb))

    def create_publisher(self, _type, topic, _qos):
        host = self

        class _Pub:
            def publish(self, msg):
                host.sent.append((topic, msg.data))
        return _Pub()

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

    def _apply_arm_home(self):
        self.homed = True

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
    monkeypatch.setattr(imu_recorder, 'RUNS_DIR', str(tmp_path / 'runs'))
    monkeypatch.setattr(gui_imu, 'TK_STALL_S', 0)   # o teste gira o laço à mão
    frame = tk.Frame(_tk_root)
    h = _Host(frame)
    h._build_imu_tab(frame)
    h.root.after_cancel(h._imu_after)
    yield h
    frame.destroy()


def test_subscribes_one_topic_per_imu_id(host):
    topics = [t for t, _cb in host.subs]
    assert topics == [f'/imu/sensor_{i}' for i in range(1, 7)] + ['/imu/calibration']


def _fk(q_rad):
    return tcp_pose(np.asarray(q_rad), T_end=T_HAND_ATTACH)


def _start_cartesian(host, feeds):
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
    host._imu_toggle_tracking()
    assert not host._imu_tracking


def _ori_only(host):
    host._imu_cart_vars['ori_imu'].set('6')
    host._imu_cart_vars['follow_pos'].set(False)


def test_locked_joints_never_move(host):
    _ori_only(host)
    for j in ('joint1', 'joint2', 'joint3'):
        host._imu_lock_vars[j].set(True)
    _start_cartesian(host, {6: _q((0, 0, 1), 0.0)})
    host.tick(90, feeds={6: _q((0, 1, 0), 20.0)})
    assert host.published
    q = dict(zip(ARM_JOINTS, np.degrees(host.published[-1])))
    for j in ('joint1', 'joint2', 'joint3'):
        assert q[j] == pytest.approx(Q_START_DEG[j], abs=1e-6)
    # o punho livre ainda gira
    assert any(abs(q[j] - Q_START_DEG[j]) > 1.0 for j in ('joint4', 'joint5', 'joint6'))


def test_start_refused_with_all_joints_locked(host):
    _ori_only(host)
    for v in host._imu_lock_vars.values():
        v.set(True)
    host.tick(feeds={6: _q((0, 0, 1), 0.0)})
    host._imu_toggle_tracking()
    assert not host._imu_tracking
    assert 'locked' in host.status


def test_start_refused_without_imu_data(host):
    _ori_only(host)
    host._imu_toggle_tracking()
    assert not host._imu_tracking
    assert '6' in host.status


def test_publishes_only_when_target_changes(host):
    _ori_only(host)
    _start_cartesian(host, {6: _q((0, 0, 1), 0.0)})
    host.tick(90, feeds={6: _q((0, 0, 1), 10.0)})
    n = len(host.published)
    host.tick(20, feeds={6: _q((0, 0, 1), 10.0)})
    assert len(host.published) == n


def test_freezes_on_stale_imu(host):
    _ori_only(host)
    _start_cartesian(host, {6: _q((0, 0, 1), 0.0)})
    host.tick(5, feeds={6: _q((0, 0, 1), 20.0)})
    n = len(host.published)
    t0, q0 = host._imu_latest[6]
    host._imu_latest[6] = (t0 - 2 * gui_imu.IMU_STALE_S, q0)   # envelhece
    host.tick(10)
    assert len(host.published) == n
    assert 'frozen' in host._imu_cart_lbl.cget('text')


def test_estop_stops_tracking(host):
    _ori_only(host)
    _start_cartesian(host, {6: _q((0, 0, 1), 0.0)})
    host._estop_latched = True
    host.tick(feeds={6: _q((0, 0, 1), 0.0)})
    assert not host._imu_tracking
    assert 'E-STOP' in host.status


def test_config_persists(host):
    host._imu_lock_vars['joint2'].set(True)
    host._imu_cart_vars['heading'].set(45.0)
    data = host._imu_load_config()
    assert data['locked'] == ['joint2']
    assert data['cartesian']['heading'] == 45.0


def test_home_stops_tracking_and_sends_home(host):
    _ori_only(host)
    _start_cartesian(host, {6: _q((0, 0, 1), 0.0)})
    host._imu_go_home()
    assert not host._imu_tracking
    assert host.homed


def test_home_blocked_while_robot_busy(host):
    host._latest_phase = 'APPROACH'
    host._imu_go_home()
    assert not host.homed
    assert 'busy' in host.status


def _rows(path, name):
    with open(os.path.join(path, name)) as fh:
        return list(csv.DictReader(fh))


def test_session_is_recorded_with_freeze_summary(host):
    _ori_only(host)
    _start_cartesian(host, {6: _q((0, 0, 1), 0.0)})
    rec = host._imu_rec
    assert rec is not None
    host.tick(10, feeds={6: _q((0, 0, 1), 10.0), 2: _q((0, 0, 1), 0.0)})
    t0, q0 = host._imu_latest[6]
    host._imu_latest[6] = (t0 - 2 * gui_imu.IMU_STALE_S, q0)   # IMU 6 congela
    host.tick(3)
    time.sleep(gui_imu.IMU_STALE_S + 0.1)                      # buraco real
    host.tick(5, feeds={6: _q((0, 0, 1), 10.0)})
    host._imu_toggle_tracking()                                # Stop
    assert host._imu_rec is None
    assert rec.path in host.status
    imu = _rows(rec.path, 'imu.csv')
    assert {r['sensor_id'] for r in imu} == {'2', '6'}          # todas as IDs
    robot = _rows(rec.path, 'robot.csv')
    assert len(robot) == 19
    frozen = [r for r in robot if r['frozen'] == '1']
    assert frozen and all(r['stale_ids'] == '6' for r in frozen)
    moving = [r for r in robot if r['frozen'] == '0']
    assert any(r['published'] == '1' for r in moving)
    assert float(moving[-1]['ik_err_mm']) < 5.0
    meta = json.load(open(os.path.join(rec.path, 'meta.json')))
    assert meta['imu_summary']['6']['gaps_over_stale'] == 1
    assert meta['cartesian']['ori_imu'] == '6'
    assert not os.path.exists(os.path.join(rec.path, 'real.csv'))   # sem CR10


def test_recording_can_be_turned_off(host):
    _ori_only(host)
    host._imu_rec_var.set(False)
    _start_cartesian(host, {6: _q((0, 0, 1), 0.0)})
    assert host._imu_rec is None


def _led(host, sid):
    led = host._imu_leds[sid]
    cv = led['cv']
    return cv.itemcget(led['body'], 'fill'), cv.itemcget(led['hz'], 'text')


def test_leds_show_live_and_off_sensors(host):
    host.tick(feeds={3: _q((0, 0, 1), 0.0)})
    assert _led(host, 3)[0] == gui_imu.OK
    assert _led(host, 1) == (gui_imu.LED_OFF, 'off')


def _calib_status(host, **st):
    from std_msgs.msg import String
    cb = dict(host.subs)['/imu/calibration']
    cb(String(data=json.dumps(st)))


def _wizard_page_text(w):
    out = []
    for c in w.body.winfo_children():
        try:
            out.append(c.cget('text'))
        except tk.TclError:
            pass
    return '\n'.join(out)


def _foot_btn(w, prefix):
    return [b for b in w.foot.winfo_children() if b.cget('text').startswith(prefix)][0]


def _prepare_and_start(w):
    start = _foot_btn(w, '▶')
    w._poll()
    assert str(start.cget('state')) == 'disabled'        # checklist vazio
    for v in w._prep_vars:
        v.set(True)
    w._refresh_prepare()
    assert str(start.cget('state')) == 'normal'
    start.invoke()


def test_calibration_wizard_does_mag_one_sensor_at_a_time(host):
    host.tick(feeds={4: _q((0, 0, 1), 0.0), 5: _q((0, 0, 1), 0.0)})
    host._imu_calibrate('MAG')
    w = host._imu_wizard
    assert w is not None and w.step == 0
    w._poll()
    assert w.kind.get() == 'MAG'
    assert w._chosen() == [4, 5]                         # pré-marca os ligados
    assert str(w._sel_cbs[1].cget('state')) == 'disabled'   # desligado
    assert 'ONE sensor at a time' in w._one_msg.cget('text')
    _foot_btn(w, 'Next').invoke()
    assert w.step == 1 and '(1 of 2)' in _wizard_page_text(w)
    assert any(isinstance(c, gui_imu_calib.Sensor3DAnim) for c in w.body.winfo_children())
    _prepare_and_start(w)
    assert host.sent[-1] == ('/imu/calibrate', 'MAG 4') and w.step == 2
    assert w.running == [4]                              # só o da vez
    _calib_status(host, state='progress', kind='MAG', sensor=4, progress=50.0)
    w._refresh_run()
    assert w._bars[4][2].cget('text') == '50%'
    _calib_status(host, state='done', kind='MAG', sensor=4, cov_pct=2.0)
    w._refresh_run()
    assert w.step == 3 and 'good' in _wizard_page_text(w)
    nxt = _foot_btn(w, 'Next sensor: ID 5')
    assert str(nxt.cget('state')) == 'disabled'          # falta confirmar 2 s parado
    w._settled.set(True)
    w._refresh_result()
    nxt.invoke()
    assert w.step == 1 and '(2 of 2)' in _wizard_page_text(w)
    _prepare_and_start(w)
    assert host.sent[-1] == ('/imu/calibrate', 'MAG 5') and w.running == [5]
    _calib_status(host, state='done', kind='MAG', sensor=5, cov_pct=7.0)
    w._refresh_run()
    assert 'poor' in _wizard_page_text(w)
    finish = _foot_btn(w, 'Finish')
    w._settled.set(True)
    w._refresh_result()
    finish.invoke()
    assert host._imu_wizard is None


def test_calibration_wizard_does_acc_gyro_all_at_once(host):
    host.tick(feeds={4: _q((0, 0, 1), 0.0), 5: _q((0, 0, 1), 0.0)})
    host._imu_calibrate('AG')
    w = host._imu_wizard
    _foot_btn(w, 'Next').invoke()
    _prepare_and_start(w)
    assert host.sent[-2:] == [('/imu/calibrate', 'AG 4'), ('/imu/calibrate', 'AG 5')]
    w._on_close()


def test_calibration_wizard_cancel_sends_cancel(host):
    host.tick(feeds={4: _q((0, 0, 1), 0.0)})
    host._imu_calibrate('AG')
    w = host._imu_wizard
    _foot_btn(w, 'Next').invoke()
    for v in w._prep_vars:
        v.set(True)
    w._refresh_prepare()
    _foot_btn(w, '▶').invoke()
    assert host.sent[-1] == ('/imu/calibrate', 'AG 4')
    _foot_btn(w, '✕').invoke()
    assert host.sent[-1] == ('/imu/calibrate', 'CANCEL 4')
    _calib_status(host, state='cancelled', kind='AG', sensor=4)
    w._refresh_run()
    assert w.step == 3 and 'cancelled' in _wizard_page_text(w)
    w._on_close()


def test_led_shows_calibration_progress(host):
    _calib_status(host, state='progress', kind='AG', sensor=4, progress=40.0)
    host.tick()
    assert _led(host, 4)[1] == '40%'
    assert 'Acc + Gyro 40%' in host._imu_cal_lbl.cget('text')


def test_calibration_refused_while_tracking(host):
    _ori_only(host)
    _start_cartesian(host, {6: _q((0, 0, 1), 0.0)})
    host._imu_calibrate('AG')
    assert host._imu_wizard is None
    assert 'stop tracking' in host.status


def test_watchdog_dumps_stacks_when_tk_loop_stalls(host, monkeypatch, capfd):
    monkeypatch.setattr(gui_imu, 'TK_STALL_S', 0.2)
    host._imu_last_tick = time.monotonic() - 5.0     # laço "parado"
    th = threading.Thread(target=host._imu_watchdog, daemon=True)
    th.start()
    time.sleep(1.3)
    host._imu_shutting_down = True
    th.join(timeout=2.0)
    err = capfd.readouterr().err
    assert 'laço do Tk parado' in err and 'Thread' in err


def test_recorder_flushes_while_running(tmp_path, monkeypatch):
    monkeypatch.setattr(imu_recorder, 'FLUSH_S', 0.1)
    rec = imu_recorder.TrackingRecorder({}, base_dir=str(tmp_path))
    rec.robot(time.monotonic(), frozen=False)
    time.sleep(0.6)
    with open(os.path.join(rec.path, 'robot.csv')) as fh:   # ainda aberto
        assert len(fh.read().splitlines()) == 2
    rec.close()


def test_follow_accel_publishes_nothing_until_gravity_is_learned(host):
    cv = host._imu_cart_vars
    assert not cv['follow_acc'].get()          # desligado por padrão
    cv['ori_imu'].set('6')
    cv['follow_pos'].set(False)
    cv['follow_acc'].set(True)
    feeds = {6: _q((0, 0, 1), 0.0)}
    host.tick(feeds=feeds)
    host._imu_toggle_tracking()
    assert host._imu_tracking, host.status
    host.tick(20, feeds={6: _q((0, 0, 1), 30.0)})   # gira, mas sem viés aprendido
    assert host.published == []
    assert 'gravity' in host._imu_cart_lbl.cget('text')


# ── Guarda da IK ─────────────────────────────────────────────────────────
def _guard_state(q0_j1_deg=90.0, q_j1_deg=90.0):
    return {'q0': np.radians([q0_j1_deg, 0, 0, 0, 0, 0]),
            'q': np.radians([q_j1_deg, 0, 0, 0, 0, 0]), 'lost_s': 0.0}


def test_guard_aborts_when_joint1_swings_too_far():
    """Assinatura da gravação 143547: J1 90° → −180° com o alvo parado."""
    st = _guard_state(90.0, 90.0 - gui_imu.IMU_J1_MAX_SWING_DEG - 1.0)
    assert 'joint1' in gui_imu.IMUMixin._imu_ik_guard(st, 0.033, 0.0)
    st = _guard_state(90.0, 90.0 - gui_imu.IMU_J1_MAX_SWING_DEG + 1.0)
    assert gui_imu.IMUMixin._imu_ik_guard(st, 0.033, 0.0) == ''


def test_guard_aborts_only_after_ik_error_persists():
    """A gravação boa (143435) teve picos de 247 mm de ~0,5 s (atraso do teto de
    velocidade das juntas); só erro ACIMA de IMU_IK_LOST_M, por IMU_IK_LOST_S, para."""
    st = _guard_state()
    big = gui_imu.IMU_IK_LOST_M + 0.05
    t = 0.0
    while t < gui_imu.IMU_IK_LOST_S - 0.1:
        assert gui_imu.IMUMixin._imu_ik_guard(st, 0.033, big) == ''
        t += 0.033
    assert gui_imu.IMUMixin._imu_ik_guard(st, 0.033, 0.0) == ''   # recuperou: zera
    assert st['lost_s'] == 0.0
    for _ in range(int(gui_imu.IMU_IK_LOST_S / 0.033) + 2):
        why = gui_imu.IMUMixin._imu_ik_guard(st, 0.033, big)
    assert 'IK lost' in why
    st = _guard_state()
    for _ in range(100):   # erro de 247 mm (< limiar) nunca aborta
        assert gui_imu.IMUMixin._imu_ik_guard(st, 0.033, 0.247) == ''


def test_start_refused_with_joint_near_its_limit(host):
    host.arm_sliders['joint1'].set(-180.0)   # a pose inicial da gravação 143639
    host.tick(feeds={6: _q((0, 0, 1), 0.0)})
    host._imu_cart_vars['ori_imu'].set('6')
    host._imu_toggle_tracking()
    assert not host._imu_tracking
    assert 'unsafe start pose' in host.status and 'joint1' in host.status


def test_start_refused_too_close_to_base_axis(host, monkeypatch):
    r0 = float(np.hypot(*_fk(np.radians([Q_START_DEG[j] for j in ARM_JOINTS]))[:2, 3]))
    monkeypatch.setattr(gui_imu, 'IMU_MIN_RADIUS_M', r0 + 0.05)
    host.tick(feeds={6: _q((0, 0, 1), 0.0)})
    host._imu_cart_vars['ori_imu'].set('6')
    host._imu_toggle_tracking()
    assert not host._imu_tracking
    assert 'base axis' in host.status


@pytest.mark.parametrize('heading', [0.0, 180.0])
def test_target_never_enters_the_forbidden_cylinder(host, monkeypatch, heading):
    """Qualquer que seja o sentido do movimento, p_cmd fica fora do raio mínimo."""
    r0 = float(np.hypot(*_fk(np.radians([Q_START_DEG[j] for j in ARM_JOINTS]))[:2, 3]))
    rmin = r0 - 0.002
    monkeypatch.setattr(gui_imu, 'IMU_MIN_RADIUS_M', rmin)
    cv = host._imu_cart_vars
    cv['ori_imu'].set('—')
    cv['limb_axis'].set('+X')
    cv['heading'].set(heading)
    host._imu_seg_vars['upper_arm'][0].set('3')
    host._imu_seg_vars['upper_arm'][1].set(0.30)
    host._imu_seg_vars['forearm'][0].set('4')
    host._imu_seg_vars['forearm'][1].set(0.25)
    feeds = {3: _q((0, 0, 1), 0.0), 4: _q((0, 0, 1), 0.0)}
    _start_cartesian(host, feeds)
    radii = []
    for ang in (30.0, 60.0):   # antebraço sobe 60°: a mão recua ~125 mm
        for _ in range(60):
            host.tick(feeds={3: feeds[3], 4: _q((0, -1, 0), ang)})
            if host._imu_cart is not None:
                p = host._imu_cart['p_cmd']
                radii.append(float(np.hypot(p[0], p[1])))
    assert radii and min(radii) >= rmin - 1e-9
