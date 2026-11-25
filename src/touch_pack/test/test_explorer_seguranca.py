"""Regressões da auditoria de 24/11/2026 no tactile_explorer.

Cada teste exercita o código de produção com um braço simulado (a última pose
publicada é a executada) e falhava antes da correção. Nenhum depende de Gazebo
nem de robô.
"""
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from touch_pack.kinematics import (          # noqa: E402
    T_TOUCH_TOOL_ATTACH, forward_kinematics,
    inverse_kinematics,
)
from touch_pack.plane_probe import attack_dir_from_tilt_azimuth  # noqa: E402

_HOME = [0.0, 0.0, -90.0, 0.0, 90.0, 0.0]
_DOWN = np.array([0.0, 0.0, -1.0])


@pytest.fixture(scope='module')
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def node(_ros):
    """Explorer com braço que executa e célula viva, descarregada."""
    from touch_pack.tactile_explorer import TactileExplorer
    n = TactileExplorer()
    n.q_sim = np.deg2rad(_HOME)
    n.fz = 0.0
    n._q_now = lambda: n.q_sim.copy()
    n._stream_q = lambda q, *a, **k: setattr(n, 'q_sim', np.array(q))
    n._arm_traj_pub.publish = lambda m: setattr(
        n, 'q_sim', np.array(m.points[-1].positions))
    n._fz_corrected = lambda: n.fz
    n._force_stale_abort = lambda fase: False
    n._send_hand_pose = lambda *a, **k: None
    n._settle = lambda *a, **k: None
    n._hold_current_position = lambda *a, **k: None
    n._run_protocol = lambda: None
    n._speed_factor_pct = 100.0
    yield n
    n.destroy_node()


def _start_msg(**campos):
    from touch_pack_msgs.msg import PalpationStart
    m = PalpationStart()
    m.mode, m.depth_mm, m.force_n = 'TOUCH', 10.0, 2.0
    m.slide_dist_mm, m.speed_mms = 10.0, 10.0
    m.home_deg = _HOME
    for k, v in campos.items():
        setattr(m, k, v)
    return m


def _aplica(n, **campos):
    n._busy.clear()
    ok = n._start_from_msg(_start_msg(**campos))
    if ok:
        n._protocol_thread.join(2.0)
    return ok


# ── XC1: a IK fica no ramo da semente ─────────────────────────────────

def _homes():
    rng = np.random.default_rng(0)
    out = [np.deg2rad(_HOME)]
    while len(out) < 25:
        j2, j3 = rng.uniform(-40, 0), rng.uniform(-110, -60)
        h = np.radians([rng.uniform(-60, 60), j2, j3, -(j2 + j3) - 90, 90,
                        rng.uniform(-90, 90)])
        if forward_kinematics(h, T_end=T_TOUCH_TOOL_ATTACH)[2, 2] < -0.98:
            out.append(h)
    return out


@pytest.mark.parametrize('tilt,az', [(3, 0), (5, 120), (10, 240)])
def test_ik_do_ataque_nao_troca_de_ramo(tilt, az):
    """3° de inclinação pediam ~100° de J4 e a ponteira varria metros."""
    for h in _homes():
        p = forward_kinematics(h, T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]
        q, ok = inverse_kinematics(
            p, attack_dir_from_tilt_azimuth(tilt, az), q_seed=h,
            T_end=T_TOUCH_TOOL_ATTACH)
        if not ok:
            continue
        assert np.max(np.abs(np.degrees(q - h))) < 45.0
        for s in np.linspace(0, 1, 11):
            tcp = forward_kinematics(h + s * (q - h),
                                     T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]
            assert np.linalg.norm(tcp - p) < 0.02


def test_home_padrao_tem_solucao_no_ramo():
    h = np.deg2rad(_HOME)
    p = forward_kinematics(h, T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]
    q, ok = inverse_kinematics(p, attack_dir_from_tilt_azimuth(3, 0),
                               q_seed=h, T_end=T_TOUCH_TOOL_ATTACH)
    assert ok and np.linalg.norm(
        forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)[:3, 3] - p) < 1e-3


def test_giro_com_salto_de_ramo_e_recusado(node, monkeypatch):
    import touch_pack.tactile_explorer as te
    longe = np.deg2rad([0.0, 0.0, -90.0, 100.0, 90.0, 0.0])
    monkeypatch.setattr(te, 'inverse_kinematics',
                        lambda *a, **k: (longe.copy(), True))
    comandos = []
    node._joint_stream_to = lambda q: comandos.append(q) or 'done'
    assert node._rotate_to_attack(
        attack_dir_from_tilt_azimuth(5, 0), label='T') == 'error'
    assert not comandos


# ── XC5: o excesso nunca é descartado ─────────────────────────────────

def _f32(v):
    from std_msgs.msg import Float32
    return Float32(data=float(v))


def test_batida_acima_de_100N_dispara_a_guarda(node):
    from touch_pack.tactile_explorer import TactileExplorer
    node._cb_lc_force_net(_f32(0.1))
    node._cb_lc_force_net(_f32(150.0))
    assert node._lc_force_net == 150.0
    assert node._force_over_limit(TactileExplorer._fz_corrected(node))


def test_pico_entre_dois_ticks_nao_passa_despercebido(node):
    """A célula entrega ~13 amostras por tick; a guarda olhava a última."""
    node._cb_lc_force_net(_f32(14.0))
    node._cb_lc_force_net(_f32(0.1))
    assert node._force_over_limit(0.1)


def test_nao_finito_nao_vira_leitura(node):
    node._cb_lc_force_net(_f32(1.0))
    node._cb_lc_force_net(_f32(float('nan')))
    assert node._lc_force_net == 1.0


# ── T07: força lateral ────────────────────────────────────────────────

def _wrench(fx, fy):
    from geometry_msgs.msg import WrenchStamped
    m = WrenchStamped()
    m.wrench.force.x, m.wrench.force.y = float(fx), float(fy)
    return m


def test_colisao_lateral_dispara_com_fz_zero(node):
    node._cb_wrench(_wrench(10.0, 10.0))          # 14,1 N de lado
    assert node._force_over_limit(0.0)
    assert node._cartesian_batch_to(
        np.array([0.0, 0.0, 1.0]), 0.005, v_const_ms=0.05,
        lock_ori=True) == 'force'


def test_atrito_de_deslize_nao_dispara(node):
    node._cb_wrench(_wrench(8.0, 0.0))            # 10 N de normal, μ 0,8
    assert not node._force_over_limit(10.0)


def test_fonte_sem_6_eixos_segue_com_o_fz(node):
    assert not node._force_over_limit(1.0)
    assert node._force_over_limit(13.0)


# ── T04 / T08: 'done' é a pose MEDIDA; a HOME também é vigiada ────────

def test_transito_descartado_pelo_jtc_nao_e_done(node, monkeypatch):
    import touch_pack.explorer_guards as g
    monkeypatch.setattr(g, '_ARRIVE_TIMEOUT_S', 0.1)
    node._arm_traj_pub.publish = lambda m: None       # o JTC descartou
    assert node._cartesian_batch_to(
        np.array([0.0, 0.0, 1.0]), 0.005, v_const_ms=0.05,
        lock_ori=True) == 'error'


def test_home_descartada_pelo_jtc_nao_e_concluida(node, monkeypatch):
    import touch_pack.explorer_guards as g
    monkeypatch.setattr(g, '_ARRIVE_TIMEOUT_S', 0.1)
    node._arm_traj_pub.publish = lambda m: None
    assert node._joint_batch_to(node.q_sim + 0.05) is False


def test_home_com_forca_para_no_lugar(node):
    parou = []
    node._settle = lambda *a, **k: parou.append(1)
    node.fz = 13.0
    assert node._joint_batch_to(node.q_sim + 0.05) is False
    assert parou


def test_giro_do_ataque_com_forca_para(node):
    node.fz = 13.0
    assert node._joint_stream_to(node.q_sim + 0.05) == 'force'


def test_recuo_tolera_celula_muda_mas_o_start_nao(node):
    node._force_stale_abort = lambda fase: True
    q = node.q_sim + 0.02
    assert node._joint_batch_to(q.copy()) is False
    assert node._joint_batch_to(q.copy(), require_fresh=False) is True


def test_plano_truncado_no_limite_nao_e_enviado(node):
    """Plano que satura num limite de junta termina longe do alvo."""
    enviados = []
    node._arm_traj_pub.publish = enviados.append
    out = node._cartesian_batch_to(np.array([1.0, 0.0, 0.0]), 1.5,
                                   v_const_ms=0.25, lock_ori=False)
    assert out == 'error' and not enviados


# ── XT24: HOME que falha encerra ABORTED, sem DONE e sem refazer ──────

def test_home_final_falha_nao_vira_done(node):
    node._phase_goto_home = lambda *a, **k: False
    node._unload_before_home = lambda: None
    node._retreat_and_home('DONE')
    assert node._phase == 'ABORTED'


# ── T01: força parada apesar do comando ───────────────────────────────

def test_hold_com_braco_parado_aborta_antes_de_acumular_curso(node):
    """Braço real alarmado com a célula viva: força constante, abaixo do
    alvo. O HOLD comandava sem fim."""
    curso = []
    node._qs_measure_fz = lambda *a, **k: 1.0
    node._relieve_contact = lambda *a, **k: None

    def _step(_d, step_m, *a, **k):
        curso.append(step_m)
        return node._q_now()
    node._qs_step = _step
    node._k_est.k, node._k_est.estimated = 300.0, True
    out, _ = node._qs_regulate(2.0, 0.05, _DOWN, 1.0, np.eye(6),
                               budget_m=None, stable_s=1.0, timeout_s=3.0,
                               phase='HOLD')
    assert out == 'target_lost'
    assert sum(curso) < 0.003


def test_hold_que_ja_comeca_sem_contato_tem_vigia(node):
    """A amostra saiu entre a descida e o HOLD."""
    from touch_pack.tactile_explorer import _TARGET_LOST_FREE_M
    curso = []
    node._qs_measure_fz = lambda *a, **k: 0.0
    node._relieve_contact = lambda *a, **k: None
    node._qs_step = lambda d, s, *a, **k: curso.append(s) or node._q_now()
    out, _ = node._qs_regulate(2.0, 0.05, _DOWN, 1.0, np.eye(6),
                               budget_m=None, stable_s=1.0, timeout_s=3.0,
                               phase='HOLD')
    assert out == 'target_lost'
    assert sum(curso) < 2 * _TARGET_LOST_FREE_M


def test_hold_em_contato_mole_que_responde_nao_aborta(node):
    """Silicone mole: 0,08 N/mm ainda responde mais que o limiar."""
    x = [0.0]
    node._qs_measure_fz = lambda *a, **k: 0.5 + 80.0 * x[0]

    def _step(_d, step_m, *a, **k):
        x[0] += step_m
        return node._q_now()
    node._qs_step = _step
    out, _ = node._qs_regulate(0.8, 0.05, _DOWN, 1.0, np.eye(6),
                               budget_m=None, stable_s=0.0, timeout_s=60.0,
                               phase='HOLD')
    assert out == 'ok'


# ── T03: FREEZE não move mais nada ────────────────────────────────────

def test_freeze_no_matrix_map_nao_alivia_nem_sobe(node):
    movs = []
    node._relieve_contact = lambda *a, **k: movs.append('alivio')
    node._lift_to_safe_z = lambda *a, **k: movs.append('lift') or 'done'
    node._phase_goto_home = lambda *a, **k: movs.append('home') or True
    node._matrix_origin = np.zeros(3)
    node.fz = 2.0
    node._busy.set()
    node._cb_freeze(None)
    node._matrix_bail('stop')
    node._busy.clear()
    assert movs == [] and node._phase == 'FROZEN'


def test_freeze_interrompe_o_alivio(node):
    node.fz = 20.0
    n_cmd = []
    node._stream_q = lambda *a, **k: n_cmd.append(1)
    node._busy.set()
    node._cb_freeze(None)
    node._relieve_contact(_DOWN)
    node._busy.clear()
    assert not n_cmd


# ── T06: start de outra sessão ────────────────────────────────────────

class _Relogio:
    def __init__(self, s):
        self.nanoseconds = int(s * 1e9)

    def now(self):
        return self


@pytest.mark.parametrize('agora,stamp,velho', [
    (0.0, 500, True),          # use_sim_time sem /clock
    (0.0, 0, True),
    (5.0, 500, True),          # /clock reiniciado: start do "futuro"
    (500.0, 100, True),
    (500.0, 495, False),
    (500.0, 0, False),         # publisher antigo, sem stamp
])
def test_idade_do_start(node, agora, stamp, velho):
    m = _start_msg()
    m.stamp.sec = stamp
    real = node.get_clock
    node.get_clock = lambda: _Relogio(agora)
    try:
        assert node._start_is_stale(m) is velho
    finally:
        node.get_clock = real


# ── T11 / T19: entrada validada ───────────────────────────────────────

@pytest.mark.parametrize('campos', [
    {'home_deg': [0.0] * 6},
    {'home_deg': [0.0, 0.0, -400.0, 0.0, 90.0, 0.0]},
    {'home_deg': [0.0, float('nan'), -90.0, 0.0, 90.0, 0.0]},
    {'force_n': float('nan')},
    {'depth_mm': float('inf')},
    {'speed_mms': float('nan')},
    {'hold_stable_s': float('nan')},
])
def test_start_invalido_e_recusado(node, campos):
    assert _aplica(node, **campos) is False
    assert node._phase == 'ABORTED'


def test_velocidade_do_start_tem_teto(node):
    assert _aplica(node, speed_mms=5000.0, approach_speed_mms=5000.0)
    assert node._slide_speed_mms <= 30.0
    assert node.get_parameter('approach_v_max_mms').value <= 30.0


@pytest.mark.parametrize('v', [float('nan'), float('inf')])
def test_setpoint_manual_nao_finito_e_recusado(node, v):
    node._target_force_n = 2.0
    node._cb_set_force(_f32(v))
    assert node._target_force_n == 2.0


def test_joint_states_com_nan_ou_parcial_nao_vira_pose(node):
    from sensor_msgs.msg import JointState
    from touch_pack.constants import ARM_JOINTS
    antes = node._current_q.copy()
    node._cb_joints(JointState(
        name=list(ARM_JOINTS), position=[0.1, float('nan')] + [0.1] * 4))
    node._cb_joints(JointState(name=['dedo'], position=[0.3]))
    assert np.array_equal(node._current_q, antes) and node._q_ts == 0.0


def test_align_do_run_anterior_nao_sobrevive(node):
    _aplica(node, probe_align='MANUAL', probe_align_manual_tilt_deg=15.0)
    assert node._align_msg.get('manual')
    _aplica(node, probe_align='')
    assert not node._align_msg.get('manual')


# ── T12 / T28: FROZEN tem saída explícita; status diz de quem é ───────

def test_stop_encerra_o_frozen_sem_mover(node):
    from std_msgs.msg import String
    movs = []
    node._phase_goto_home = lambda *a, **k: movs.append(1)
    node._set_phase('FROZEN')
    node._cb_stop(String())
    assert node._phase == 'IDLE' and not movs


def test_status_carrega_o_run_id_ate_na_recusa(node):
    from touch_pack.constants import run_id_from_msg
    vistos = []
    node._status_pub.publish = vistos.append
    m = _start_msg(home_deg=[0.0] * 6)
    m.stamp.sec = 1234
    node._busy.clear()
    node._start_from_msg(m)
    assert vistos[-1].phase == 'ABORTED'
    assert vistos[-1].run_id == run_id_from_msg(m)


# ── T14 / T40: a onda de força ────────────────────────────────────────

def _onda(node, cycles=2, hz=1.0):
    from touch_pack.force_wave import _ForceProfile
    prof = _ForceProfile('SINE', 1.0, 2.0, hz, cycles)
    z0 = node._tcp_now()[2]
    node._phase_hold = lambda **k: 'ok'
    node._settle_until_still = lambda *a, **k: True
    node._k_est.k, node._k_est.estimated = 1000.0, True
    node._approach_dir = _DOWN.copy()
    node._fz_corrected = lambda: 1.5 + 1000.0 * (z0 - node._tcp_now()[2])
    node._lc_rate_hz = lambda: 430.0
    node._lc_raw_rate_hz = lambda: 430.0
    return prof


def test_pausa_nao_consome_ciclos_da_onda(node):
    prof = _onda(node)
    base = node._pause_gate
    pausou = []

    def _gate():
        if time.monotonic() - t0 > 0.5 and not pausou:
            pausou.append(1)
            node._pause_requested.set()
            threading.Timer(1.5, node._pause_requested.clear).start()
        return base()
    node._pause_gate = _gate
    t0 = time.monotonic()
    out = node._phase_hold_modulated(prof)
    assert out == 'ok' and pausou
    assert time.monotonic() - t0 >= prof.duration_s + 1.4


def test_onda_para_com_forca_alem_da_margem(node):
    prof = _onda(node, cycles=5)
    alivio = []
    node._relieve_contact = lambda *a, **k: alivio.append(1)
    ok = node._fz_corrected
    t0 = time.monotonic()
    node._fz_corrected = lambda: 13.0 if time.monotonic() - t0 > 1.0 else ok()
    assert node._phase_hold_modulated(prof) == 'force'
    assert alivio and time.monotonic() - t0 < 3.0


def test_exemplos_do_readme_passam_na_validacao():
    """T11: copiados sem `home_deg`, mandavam o braço à pose toda zero. Cada
    `topic pub` do README vira a mensagem real e passa pela guarda real."""
    import re
    from pathlib import Path

    import yaml
    from rosidl_runtime_py import set_message_fields
    from touch_pack_msgs.msg import PalpationStart
    from touch_pack.explorer_guards import GuardsMixin
    txt = (Path(__file__).parents[1] / 'README.md').read_text(encoding='utf-8')
    ex = re.findall(r'PalpationStart \\\n\s*"(\{.*?\})"', txt, re.S)
    assert len(ex) == 3
    for e in ex:
        msg = PalpationStart()
        set_message_fields(msg, yaml.safe_load(e))
        assert GuardsMixin._start_refusal(msg) is None, e
