"""Velocidade do SLIDING e da descida amarrada ao relógio de parede.

O run SLIDE/20261021_113235 pediu 40 mm a 5 mm/s e andou 25,9 mm a 3,2 mm/s:
cada janela comandava "posição medida + passo", e a medida chega atrasada.
Aqui um braço falso segue o JTC (cada mensagem SUBSTITUI a anterior) com o
feedback atrasado, que é exatamente a condição da bancada.
"""
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
    `_LAG_S` atrás. Tudo no relógio virtual: o resultado não depende da carga
    da máquina."""

    def __init__(self, q0, relogio):
        self._rel = relogio
        self._planos = [(-1e9, np.asarray(q0, float), [])]

    def publish(self, msg):
        pts = [(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                np.asarray(p.positions, float)) for p in msg.points]
        self._planos.append((self._rel.t, self._em(self._rel.t), pts))

    def _em(self, t):
        t_rx, q_prev, pts = next(p for p in reversed(self._planos)
                                 if p[0] <= t)
        tau, t_prev = t - t_rx, 0.0
        for t_k, q_k in pts:
            if tau <= t_k:
                a = (tau - t_prev) / max(t_k - t_prev, 1e-9)
                return q_prev + a * (q_k - q_prev)
            t_prev, q_prev = t_k, q_k
        return q_prev.copy()

    def measured(self):
        return self._em(self._rel.t - _LAG_S)


@pytest.fixture()
def node(_ros, relogio, monkeypatch):
    import touch_pack.tactile_explorer as te
    n = te.TactileExplorer()
    monkeypatch.setattr(te, 'time', relogio)
    arm = _FakeArm(np.deg2rad([0, 0, -90, 0, 90, 0]), relogio)
    n._arm_traj_pub = arm
    n._q_now = arm.measured
    n._stream_q = lambda *a, **k: None
    n._speed_factor_pct = 100.0
    # A célula renova a leitura a cada avanço do relógio.
    relogio.passo = lambda: n._cb_lc_force_net(Float32(data=1.0))
    relogio.sleep(0.05)
    yield n
    n.destroy_node()


def test_deslize_cumpre_distancia_e_velocidade_com_feedback_atrasado(node, relogio):
    from touch_pack.kinematics import forward_kinematics, T_TOUCH_TOOL_ATTACH
    dist_mm, v_mms = 20.0, 10.0
    with node._params_lock:
        node._slide_speed_mms = v_mms
        node._target_slide_mm = dist_mm
        node._target_force_n = 1.0
        node._slide_dir_vec = np.array([0.0, 1.0])
    p0 = forward_kinematics(node._q_now(), T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]
    t0 = relogio.t
    out = node._phase_sliding()
    dur = relogio.t - t0
    relogio.sleep(2 * _LAG_S)
    p1 = forward_kinematics(node._q_now(), T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]
    andou_mm = float(np.linalg.norm(p1[:2] - p0[:2])) * 1e3
    assert out == 'ok'
    assert andou_mm == pytest.approx(dist_mm, abs=0.5)
    # 2 s de curso + settles; o código antigo levava ~3 s e parava antes.
    assert dur < dist_mm / v_mms + 0.8


def test_descida_em_ar_livre_mantem_a_velocidade_com_laco_lento(node, relogio):
    """Com o laço a ~1/3 da cadência nominal, o curso em ar livre tem de levar
    o tempo de `depth/v`, não o dobro (era um passo v·dt por iteração)."""
    import touch_pack.tactile_explorer as te
    v = te.crawl_v_ms(te.TactileExplorer._K_RIGID_REF_NM)
    depth_m = 0.6 * v   # 0,6 s de curso a v
    with node._params_lock:
        node._target_depth_mm = depth_m * 1e3
        node._target_force_n = 5.0
    relogio.passo = lambda: node._cb_lc_force_net(Float32(data=0.0))
    orig = node._fz_corrected

    def lento():
        relogio.sleep(2 * te._CTRL_DT)   # laço ~3x mais lento que o nominal
        return 0.0
    node._fz_corrected = lento
    node._settle_until_still = lambda *a, **k: None
    t0 = relogio.t
    out = node._phase_descending()
    dur = relogio.t - t0
    node._fz_corrected = orig
    assert out == 'no_contact'
    # Nominal 0,6 s; o código antigo levava ~1,8 s (um passo por iteração).
    assert dur < 1.0
