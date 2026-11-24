"""Travas do nó de admitância: fonte de força, habilitar, aborto (precisa do ROS)."""
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('xacro')

from geometry_msgs.msg import WrenchStamped                   # noqa: E402
from rclpy.parameter import Parameter                         # noqa: E402
from std_msgs.msg import Float64                              # noqa: E402
from std_srvs.srv import SetBool                              # noqa: E402
from rehab_pack import admittance_node as an                  # noqa: E402
from rehab_pack.constants import HOME_Q_RAD, T_FT             # noqa: E402

OK = {'conectado': True, 'estop': False, 'sync_ok': True, 'modo': 'MIRROR',
      'ip': '10.0.0.1', 'robot_mode': 5}


@pytest.fixture
def node():
    rclpy.init(domain_id=78)
    n = an.AdmittanceNode()
    n.q = np.array(HOME_Q_RAD)
    n._on_tared(Float64(data=time.monotonic()))
    yield n
    n.destroy_node()
    rclpy.shutdown()


def real(n, fonte, **st):
    n.p['force_source'] = fonte
    n.real, n.real_t = {**OK, **st}, time.monotonic()


def forca(n, fx=0.0, frame='load_cell_link'):
    m = WrenchStamped()
    m.header.frame_id = frame
    m.wrench.force.x = float(fx)
    n._on_ft(m)


def enable(n, on=True):
    return n._srv_enable(SetBool.Request(data=on), SetBool.Response())


@pytest.mark.parametrize('fonte', ['sim'])
def test_mirror_com_forca_de_mentira_recusa_e_aborta(node, fonte):
    real(node, fonte)
    forca(node)
    r = enable(node)
    assert not r.success and 'force_source:=real' in r.message
    real(node, 'real')
    forca(node)
    assert enable(node).success
    real(node, fonte)
    node._tick()
    assert node.state == 'ABORTADO'


def test_robo_de_mentira_pode_ser_ensaiado_com_forca_simulada(node):
    real(node, 'sim', ip='sim')
    forca(node)
    assert enable(node).success


def test_fonte_real_ignora_forca_virtual_de_outro_no(node):
    real(node, 'real')
    forca(node, 10.0, frame='world')
    assert not node.ft_acc and node.ft_t is None


def test_dois_publicadores_de_forca_recusam(node):
    ps = [node.create_publisher(WrenchStamped, T_FT, 1) for _ in range(2)]
    real(node, 'real')
    forca(node)
    t0 = time.monotonic()
    while node.count_publishers(T_FT) < 2 and time.monotonic() - t0 < 3:
        time.sleep(0.05)
    assert 'mais de um' in enable(node).message and ps


def test_habilitar_exige_forca_recente_e_baixa(node):
    real(node, 'real')
    assert 'sem leitura' in enable(node).message
    forca(node, 12.0)
    node._tick()
    assert 'solte o mastro' in enable(node).message


def test_desabilitar_nao_destrava_abortado(node):
    node._set_state('ABORTADO')
    assert not enable(node, False).success and node.state == 'ABORTADO'


def test_pico_de_uma_amostra_aborta_mesmo_com_media_baixa(node):
    for estado in ('ATIVO', 'POSICIONANDO'):
        real(node, 'real')
        node.state, node.jog_end = estado, 1e12
        node._begin_active()
        for _ in range(11):
            forca(node)
        forca(node, 100.0)
        node._tick()
        assert node.state == 'ABORTADO', estado


def test_celula_muda_em_mirror_aborta_e_nao_retoma(node):
    real(node, 'real')
    forca(node)
    assert enable(node).success
    node.ft_t -= 5.0
    node._tick()
    assert node.state == 'ABORTADO'


@pytest.mark.parametrize('k,v', [('abort_force_n', float('nan')), ('v_max_ms', 5.0),
                                 ('v_max_ms', -1.0), ('deadband_n', -5.0),
                                 ('traj_lead_s', -0.1), ('force_source', 'sim')])
def test_parametro_perigoso_recusado(node, k, v):
    assert not node.set_parameters([Parameter(k, value=v)])[0].successful


def test_torque_e_medido_no_centro_da_empunhadura(node):
    r = node.r_grip
    m = WrenchStamped()
    m.header.frame_id = 'load_cell_link'
    m.wrench.force.x = 10.0                  # empurrar pelo centro: r × F na célula,
    m.wrench.torque.y = float(r[2] * 10.0)   # mas torque zero na empunhadura
    node._on_ft(m)
    assert node.m_peak == pytest.approx(0.0, abs=1e-6)
    real(node, 'real')
    node._begin_active()
    node.state = 'ATIVO'
    m.wrench.torque.z = 2.5                  # torção do mastro
    node._on_ft(m)
    node._tick()
    assert node.state == 'ABORTADO'


def test_tara_vale_so_na_orientacao_em_que_foi_feita(node):
    real(node, 'real')
    forca(node)
    node.tare_R = None
    assert 'tare' in enable(node).message
    node._on_tared(Float64(data=node.t_start - 1.0))    # de antes deste nó
    assert node.tare_R is None
    node._on_tared(Float64(data=time.monotonic()))
    node.q = node.q + np.array([0, 0, 0, 0, 0.2, 0])
    assert 'girou' in enable(node).message
    node._on_tared(Float64(data=0.0))                   # force_source reiniciou
    assert node.tare_R is None


def test_celula_baixa_vira_limite(node):
    assert node._clearance(node.q) > 0                  # HOME é válida
    real(node, 'real')
    node._begin_active()
    node.state = 'ATIVO'
    for _ in range(300):
        node.q = node.q_cmd.copy()
        node.real_t = time.monotonic()
        m = WrenchStamped()
        m.header.frame_id = 'load_cell_link'
        m.wrench.force.z = -9.0
        node._on_ft(m)
        node._tick()
        if node.state != 'ATIVO':
            break
    assert node.state == 'LIMITE' and node._clearance(node.q_cmd) > -0.004


def test_simulado_movido_por_outro_no_aborta(node):
    real(node, 'real')
    forca(node)
    assert enable(node).success
    node.q = node.q + 0.2
    node._tick()
    assert node.state == 'ABORTADO'


def test_slider_troca_o_alvo_no_meio_do_movimento_e_home_usa_a_salva(node, monkeypatch):
    from std_msgs.msg import Float64MultiArray
    from rehab_pack import admittance_node as m
    monkeypatch.setattr(type(node.pub_cmd), 'get_subscription_count', lambda s: 1)
    real(node, 'real', modo='SIM_ONLY')
    q = node.q + np.array([0.3, 0, 0, 0, 0, 0])
    node._on_goto(Float64MultiArray(data=[*q, 50.0]))
    assert node.state == 'POSICIONANDO' and np.allclose(node.jog_goal, q)
    node._on_goto(Float64MultiArray(data=[*(q + 0.05), 100.0]))
    assert np.allclose(node.jog_goal, q + 0.05)
    node._on_goto(Float64MultiArray(data=[*q, float('nan')]))   # velocidade inválida
    assert np.allclose(node.jog_goal, q + 0.05)
    node._set_state('PARADO')
    alvo = list(node.q + np.array([0.2, 0, 0, 0, 0, 0]))
    monkeypatch.setattr(m, 'home_q', lambda: alvo)
    from std_srvs.srv import Trigger
    assert node._srv_home(None, Trigger.Response()).success
    assert np.allclose(node.jog_goal, alvo)
