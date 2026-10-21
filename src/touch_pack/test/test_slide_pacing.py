"""Velocidade do SLIDING e da descida amarrada ao relógio de parede.

O run SLIDE/20261021_113235 pediu 40 mm a 5 mm/s e andou 25,9 mm a 3,2 mm/s:
cada janela comandava "posição medida + passo", e a medida chega atrasada.
Aqui um braço falso segue o JTC (cada mensagem SUBSTITUI a anterior) com o
feedback atrasado, que é exatamente a condição da bancada.
"""
import collections
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from std_msgs.msg import Float32  # noqa: E402

_LAG_S = 0.040   # atraso do feedback — o dobro do medido, para folga


@pytest.fixture(scope='module')
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class _FakeArm:
    """Segue a última JointTrajectory (interpolação linear entre os pontos,
    partindo da posição no instante da chegada) e devolve a posição de
    `_LAG_S` atrás."""

    def __init__(self, q0):
        self._lock = threading.Lock()
        self._q = np.asarray(q0, float)
        self._plan = None            # (t_rx, q_rx, [(t, q), ...])
        self._hist = collections.deque(maxlen=5000)
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def publish(self, msg):
        pts = [(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                np.asarray(p.positions, float)) for p in msg.points]
        with self._lock:
            self._plan = (time.monotonic(), self._q.copy(), pts)

    def _run(self):
        while not self._stop.is_set():
            now = time.monotonic()
            with self._lock:
                if self._plan is not None:
                    t_rx, q_prev, pts = self._plan
                    tau, t_prev = now - t_rx, 0.0
                    for t_k, q_k in pts:
                        if tau <= t_k:
                            a = (tau - t_prev) / max(t_k - t_prev, 1e-9)
                            self._q = q_prev + a * (q_k - q_prev)
                            break
                        t_prev, q_prev = t_k, q_k
                    else:
                        self._q = pts[-1][1].copy()
                self._hist.append((now, self._q.copy()))
            time.sleep(0.001)

    def measured(self):
        t_want = time.monotonic() - _LAG_S
        with self._lock:
            for t, q in reversed(self._hist):
                if t <= t_want:
                    return q.copy()
            return self._hist[0][1].copy() if self._hist else self._q.copy()


@pytest.fixture()
def node(_ros):
    from touch_pack.tactile_explorer import TactileExplorer
    n = TactileExplorer()
    arm = _FakeArm(np.deg2rad([0, 0, -90, 0, 90, 0]))
    n._arm_traj_pub = arm
    n._q_now = arm.measured
    n._stream_q = lambda *a, **k: None
    n._speed_factor_pct = 100.0
    stop = threading.Event()

    def _force():
        while not stop.is_set():
            n._cb_lc_force_net(Float32(data=1.0))
            time.sleep(0.01)
    threading.Thread(target=_force, daemon=True).start()
    time.sleep(0.05)
    yield n
    stop.set()
    arm._stop.set()
    n.destroy_node()


def test_deslize_cumpre_distancia_e_velocidade_com_feedback_atrasado(node):
    from touch_pack.kinematics import forward_kinematics, T_TOUCH_TOOL_ATTACH
    dist_mm, v_mms = 20.0, 10.0
    with node._params_lock:
        node._slide_speed_mms = v_mms
        node._target_slide_mm = dist_mm
        node._target_force_n = 1.0
        node._slide_dir_vec = np.array([0.0, 1.0])
    p0 = forward_kinematics(node._q_now(), T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]
    t0 = time.time()
    out = node._phase_sliding()
    dur = time.time() - t0
    time.sleep(2 * _LAG_S)
    p1 = forward_kinematics(node._q_now(), T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]
    andou_mm = float(np.linalg.norm(p1[:2] - p0[:2])) * 1e3
    assert out == 'ok'
    assert andou_mm == pytest.approx(dist_mm, abs=0.5)
    # 2 s de curso + settles; o código antigo levava ~3 s e parava antes.
    assert dur < dist_mm / v_mms + 0.8


def test_descida_em_ar_livre_mantem_a_velocidade_com_laco_lento(node):
    """Com o laço a ~1/3 da cadência nominal, o curso em ar livre tem de levar
    o tempo de `depth/v`, não o dobro (era um passo v·dt por iteração)."""
    import touch_pack.tactile_explorer as te
    v = te.crawl_v_ms(te.TactileExplorer._K_RIGID_REF_NM)
    depth_m = 0.6 * v   # 0,6 s de curso a v
    with node._params_lock:
        node._target_depth_mm = depth_m * 1e3
        node._target_force_n = 5.0
    node._cb_lc_force_net(Float32(data=0.0))
    orig = node._fz_corrected

    def lento():
        time.sleep(2 * te._CTRL_DT)   # laço ~3x mais lento que o nominal
        return 0.0
    node._fz_corrected = lento
    node._settle_until_still = lambda *a, **k: None
    t0 = time.time()
    out = node._phase_descending()
    dur = time.time() - t0
    node._fz_corrected = orig
    assert out == 'no_contact'
    # Nominal 0,6 s; o código antigo levava ~1,8 s (um passo por iteração).
    assert dur < 1.0
