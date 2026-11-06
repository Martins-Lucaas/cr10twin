"""Ângulo de ataque manual e tare automático depois de mudar a orientação.

A geometria roda sem ROS; o resto confere o que o nó decide: quando recusa,
por onde passa a ferramenta e quando o zero da célula é refeito.
"""
import math
import threading

import numpy as np
import pytest

from touch_pack.plane_probe import (approach_start,
                                    attack_dir_from_tilt_azimuth)


# ── Geometria ─────────────────────────────────────────────────────────

def test_convencao_de_inclinacao_e_azimute():
    assert attack_dir_from_tilt_azimuth(0, 77) == pytest.approx([0, 0, -1])
    s, c = math.sin(math.radians(30)), math.cos(math.radians(30))
    assert attack_dir_from_tilt_azimuth(30, 0) == pytest.approx([s, 0, -c])
    assert attack_dir_from_tilt_azimuth(30, 90) == pytest.approx([0, s, -c])


@pytest.mark.parametrize('tilt', [0, 10, 20, 30])
def test_a_reta_passa_pelo_alvo_e_a_aresta_fica_acima_da_folga(tilt):
    target = np.array([0.4, -0.1, 0.2])
    attack = attack_dir_from_tilt_azimuth(tilt, 40)
    start, travel = approach_start(target, attack, 0.02, 0.011)
    assert start + travel * attack == pytest.approx(target)
    aresta_z = start[2] - 0.011 * math.sin(math.radians(tilt))
    assert aresta_z - target[2] == pytest.approx(0.02)


# ── Nó ────────────────────────────────────────────────────────────────

rclpy = pytest.importorskip('rclpy')


@pytest.fixture(scope='module')
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def node(_ros):
    from touch_pack.tactile_explorer import TactileExplorer
    n = TactileExplorer()
    n._q_now = lambda: np.deg2rad([0, 0, -90, 0, 90, 0]).astype(float)
    n._stream_q = lambda *a, **k: None
    n._settle = lambda *a, **k: None
    n._speed_factor_pct = 10.0
    n._mode = 'TOUCH'
    with n._params_lock:
        n._target_force_n = 2.0
        n._target_depth_mm = 50.0
        n._align_from_msg = True
        n._align_on = True
        n._align_msg = {'manual': True, 'manual_tilt_deg': 20.0,
                        'manual_azimuth_deg': 0.0, 'retract_mm': 20.0}
    yield n
    n.destroy_node()


def _fake_motion(node, log):
    """Braço de mentira: o TCP vai para onde os movimentos mandam."""
    pos = {'p': node._tcp_now()}
    node._tcp_now = lambda: pos['p'].copy()

    def move(delta, _v, *, lock_z=False, label=''):
        pos['p'] = pos['p'] + np.asarray(delta, dtype=float)
        log.append((label, pos['p'].copy()))
        return 'done'
    node._move_linear_world = move
    node._rotate_to_attack = (
        lambda a, label='': log.append(('ROTATE', pos['p'].copy())) or 'ok')
    node._verify_attack_axis = lambda *a: True
    node._tare_here = lambda label: log.append(('TARE', label)) or True
    return pos


@pytest.mark.parametrize('msg,trecho', [
    ({'manual_tilt_deg': 31.0}, 'inclinação'),
    ({'manual_tilt_deg': float('nan')}, 'inclinação'),
    ({'manual_azimuth_deg': float('inf')}, 'azimute'),
])
def test_angulo_invalido_recusa_sem_saturar(node, msg, trecho):
    node._align_msg.update(msg)
    assert trecho in node._manual_attack_refusal(node._align_params())


@pytest.mark.parametrize('mode', ['SLIDE', 'MATRIX_MAP'])
def test_modos_que_usam_a_normal_recusam_antes_de_tocar(node, mode):
    node._mode = mode
    node._probe_plane = lambda *a, **k: pytest.fail('tocou a peça')
    assert node._phase_calibrate_attack() == 'error'


def test_curso_maior_que_o_limite_de_descida_recusa(node):
    with node._params_lock:
        node._target_depth_mm = 20.0
    assert 'curso' in node._manual_attack_refusal(node._align_params())


def test_toque_unico_e_chegada_na_reta_que_passa_pelo_alvo(node):
    log = []
    pos = _fake_motion(node, log)
    target = pos['p'] - np.array([0.0, 0.0, 0.015])
    chamadas = []

    def probe(p_ref, offsets, cfg):
        chamadas.append(offsets)
        return 'ok', [target.copy()]
    node._probe_plane = probe
    refs = []
    node._attack_ref_pub.publish = lambda m: refs.append(m.data)

    assert node._phase_calibrate_attack() == 'ok'
    assert len(chamadas) == 1 and chamadas[0].shape == (1, 2)

    labels = [e[0] for e in log]
    assert labels.index('ROTATE') < labels.index('ATTACK-START')
    assert labels[-1] == 'TARE'           # depois de chegar, nunca antes
    attack = node._attack_dir
    assert attack == pytest.approx(attack_dir_from_tilt_azimuth(20, 0))
    lateral = pos['p'] - target
    lateral -= (lateral @ attack) * attack
    assert np.linalg.norm(lateral) < 1e-9
    assert node._tare_tilted is True and node._attack_engaged is True
    assert len(refs) == 1 and 'target_m' in refs[0]
    assert node._learned_contact_m is None


def test_repeticao_volta_a_partida_em_vez_de_girar_na_home(node):
    log = []
    pos = _fake_motion(node, log)
    node._attack_dir = attack_dir_from_tilt_azimuth(20, 0)
    node._attack_start = pos['p'] + np.array([-0.01, 0.0, -0.005])
    assert node._reapply_attack_orientation() is True
    assert pos['p'] == pytest.approx(node._attack_start)
    assert ('TARE', 'ATTACK') in log


def test_falha_de_tare_impede_a_descida(node):
    pos = _fake_motion(node, [])
    node._tare_here = lambda label: False
    node._attack_dir = attack_dir_from_tilt_azimuth(20, 0)
    node._attack_start = pos['p'].copy()
    assert node._goto_attack_start() == 'error'
    assert node._tare_tilted is False


def test_recuo_pelo_eixo_acontece_uma_vez(node):
    log = []
    pos = _fake_motion(node, log)
    node._unload_before_home = lambda: None
    node._attack_start = pos['p'] + np.array([-0.01, 0.0, 0.03])
    node._attack_engaged = True
    node._withdraw_along_attack()
    node._withdraw_along_attack()
    assert [e[0] for e in log] == ['ATTACK-WITHDRAW']
    assert pos['p'] == pytest.approx(node._attack_start)


# ── FREEZE ────────────────────────────────────────────────────────────

@pytest.fixture()
def frozen_node(node):
    """Trajetórias reais, com o FREEZE chegando no meio delas."""
    node._busy.set()
    node._force_stale_abort = lambda *a, **k: False
    node._send_hand_pose = lambda *a, **k: None
    node._unload_before_home = lambda: None
    node._joint_batch_to = lambda q: pytest.fail('foi à HOME')
    threading.Timer(0.2, node._cb_freeze, args=(None,)).start()
    yield node
    node._busy.clear()


def test_freeze_no_recuo_nao_vai_a_home(frozen_node):
    n = frozen_node
    n._attack_engaged = True
    n._attack_start = n._tcp_now() + np.array([-0.015, 0.0, 0.03])
    assert n._retreat_and_home('DONE') is True
    assert n._phase == 'FROZEN'


def test_freeze_na_repeticao_nao_vai_a_home(frozen_node):
    n = frozen_node
    n._attack_dir = attack_dir_from_tilt_azimuth(20, 0)
    n._attack_start = n._tcp_now() + np.array([0.0, 0.0, -0.05])
    n._rotate_to_attack = lambda a, label='': 'ok'
    assert n._reapply_attack_orientation() is False
    n._abort_to_home()
    assert n._phase == 'FROZEN'


def test_abort_com_freeze_pendente_nao_mexe_o_braco(node):
    node._unload_before_home = lambda: pytest.fail('moveu o braço')
    node._freeze_requested.set()
    node._abort_to_home()
    assert node._phase == 'FROZEN'


def test_run_novo_esquece_o_recuo_do_run_congelado(node):
    vistos = []
    node._attack_engaged = True
    node._attack_start = np.array([0.4, -0.1, 0.25])

    def home():
        vistos.append((node._attack_engaged, node._attack_start))
        return False
    node._phase_goto_home = home
    node._retreat_and_home = lambda fp: None
    node._run_protocol()
    assert vistos == [(False, None)]


def test_home_refaz_o_zero_da_pose_inclinada(node):
    log = []
    node._joint_batch_to = lambda q: True
    node._tare_here = lambda label: log.append(label) or True
    node._tare_tilted = True
    assert node._phase_goto_home() is True
    assert log == ['HOME'] and node._tare_tilted is False
    assert node._phase_goto_home() is True
    assert log == ['HOME']


def test_home_nao_segue_com_zero_errado(node):
    node._joint_batch_to = lambda q: True
    node._tare_here = lambda label: False
    node._tare_tilted = True
    assert node._phase_goto_home() is False
    assert node._tare_tilted is True


# ── O tare em si ──────────────────────────────────────────────────────

@pytest.fixture()
def tare_node(node, monkeypatch):
    import touch_pack.explorer_attack as te
    monkeypatch.setattr(te, '_TARE_SETTLE_S', 0.0)
    monkeypatch.setattr(te, '_TARE_TIMEOUT_S', 0.05)
    node._settle_until_still = lambda *a, **k: True
    node._fz_corrected = lambda: 0.0
    return node


def test_tare_sob_carga_nao_publica(tare_node):
    tare_node._fz_corrected = lambda: 1.0
    tare_node._tare_pub.publish = lambda m: pytest.fail('zerou sob contato')
    assert tare_node._tare_here('T') is False


def test_tare_nao_aceita_resposta_antiga(tare_node):
    tare_node._tare_reply = 'ok;0.0;0.0'
    tare_node._tare_done.set()
    tare_node._tare_pub.publish = lambda m: None
    assert tare_node._tare_here('T') is False


def test_run_oblíquo_em_home_nova_nao_vira_aprendizado(node):
    """O logger descarta runs de aprendizado, e este é o próprio ensaio."""
    from touch_pack_msgs.msg import PalpationStart
    node._start_from_msg = lambda m: True
    node._learned_contact_m = None
    node._learning_run_pub.publish = lambda m: pytest.fail('run descartado')
    node._cb_start(PalpationStart())
    node._busy.clear()


def test_tare_confere_o_desfecho(tare_node):
    from std_msgs.msg import String
    for resposta, esperado in (('ok;0.1;0.0', True), ('err;drifting;0.9', False)):
        tare_node._tare_pub.publish = (
            lambda m, r=resposta: tare_node._cb_tare_result(String(data=r)))
        assert tare_node._tare_here('T') is esperado
