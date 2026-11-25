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
      'ip': '10.0.0.1', 'robot_mode': 5, 'q_real': list(HOME_Q_RAD)}


@pytest.fixture
def node():
    rclpy.init(domain_id=78)
    n = an.AdmittanceNode()
    n.q, n.q_t = np.array(HOME_Q_RAD), np.inf      # simulado sempre em dia
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
    real(node, 'real', q_real=list(node.q))             # o real segue o sim
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


def alvo(n, d=0.3, pct=50.0):
    from std_msgs.msg import Float64MultiArray
    n._on_goto(Float64MultiArray(data=[*(n.q + np.array([d, 0, 0, 0, 0, 0])), pct]))


@pytest.fixture
def com_jtc(node, monkeypatch):
    """Nó com 'controlador' no ar; guarda as trajetórias em vez de publicar."""
    monkeypatch.setattr(type(node.pub_cmd), 'get_subscription_count', lambda s: 1)
    node.msgs = []
    monkeypatch.setattr(type(node.pub_cmd), 'publish', lambda s, m: node.msgs.append(m))
    return node


def test_jog_em_mirror_exige_forca_fresca_e_finita(com_jtc):
    n = com_jtc
    real(n, 'real')
    alvo(n)                                  # nunca chegou força
    assert n.state == 'PARADO'
    forca(n)
    n.ft_t -= 1.0                            # velha
    alvo(n)
    assert n.state == 'PARADO'
    forca(n, float('nan'))
    alvo(n)
    assert n.state == 'PARADO' and not [m for m in n.msgs if hasattr(m, 'points')]
    n._tick()                                # consome o NaN
    forca(n)
    n._tick()
    alvo(n)
    assert n.state == 'POSICIONANDO'
    n._tick()
    assert n.state == 'POSICIONANDO'
    n.ft_t -= 1.0
    n._tick()
    assert n.state == 'ABORTADO'


def test_jog_em_mirror_recusa_forca_que_nao_e_da_celula(com_jtc):
    real(com_jtc, 'sim')
    forca(com_jtc)
    alvo(com_jtc)
    assert com_jtc.state == 'PARADO'


def test_jog_fora_de_mirror_funciona_sem_celula(com_jtc):
    n = com_jtc
    real(n, 'real', modo='SIM_ONLY')
    alvo(n)
    assert n.state == 'POSICIONANDO'
    n._tick()
    assert n.state == 'POSICIONANDO'


def test_jog_manda_velocidade_final_zero_e_freia_antes_de_trocar_o_alvo(com_jtc):
    n = com_jtc
    real(n, 'real', modo='SIM_ONLY')

    def traj():
        return [m for m in n.msgs if hasattr(m, 'points')][-1].points

    def t(p):
        return p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
    alvo(n, 0.3)
    p, = traj()
    assert list(p.velocities) == [0.0] * 6
    assert n.jog_end == pytest.approx(n._now() + t(p) + an.JOG_MARGIN_S, abs=0.05)
    n.dq = np.array([0.2, 0, 0, 0, 0, 0])
    alvo(n, -0.3)
    freio, fim = traj()
    assert freio.positions[0] == pytest.approx(n.q[0] + 0.2 * an.BRAKE_S / 2)
    assert t(freio) == pytest.approx(an.BRAKE_S) and t(fim) > t(freio)
    assert list(freio.velocities) == list(fim.velocities) == [0.0] * 6
    assert n.jog_end == pytest.approx(n._now() + t(fim) + an.JOG_MARGIN_S, abs=0.05)
    # a admitância segue mandando um ponto só com posição
    n._send(n.q, 0.06)
    assert not traj()[0].velocities


def test_joint_states_sem_velocidade_vira_zero(node):
    from sensor_msgs.msg import JointState
    node._on_js(JointState(name=an.ARM_JOINTS, position=HOME_Q_RAD, velocity=[0.1] * 6))
    assert np.allclose(node.dq, 0.0)         # o campo do Gazebo não vale: deriva da posição
    node._on_js(JointState(name=an.ARM_JOINTS, position=HOME_Q_RAD))
    assert np.allclose(node.dq, 0.0)


def test_jog_usa_a_folga_dos_elos(com_jtc, monkeypatch):
    n = com_jtc
    real(n, 'real', modo='SIM_ONLY')
    q0 = n.q[0]
    monkeypatch.setattr(n, '_clearance', lambda q: 0.1 - abs(q[0] - q0))
    alvo(n, 0.3)
    assert n.state == 'PARADO'
    alvo(n, 0.05)
    assert n.state == 'POSICIONANDO'


def guiar(n, d, v_max, ticks):
    """Força constante na direção d, simulado perfeito; devolve o maior |dq| por tick."""
    real(n, 'real')
    n.p['v_max_ms'] = n.ctrl.v_max = v_max
    n._begin_active()
    n.state, n.force_w, pico = 'ATIVO', 9.0 * np.asarray(d, float), 0.0
    for _ in range(ticks):
        q0 = n.q_cmd.copy()
        n._active_step(0.0, np.zeros(3))
        if n.state != 'ATIVO':
            break
        pico = max(pico, float(np.max(np.abs(n.q_cmd - q0))))
        n.q = n.q_cmd.copy()
    return pico


@pytest.mark.parametrize('d', [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1)])
def test_guia_normal_nao_esbarra_no_limite_de_velocidade_das_juntas(node, d):
    pico = guiar(node, d, 0.10, 60)          # até a excursão padrão de 15 cm
    assert node.state == 'ATIVO'
    assert 0 < pico < 0.7 * an.MAX_DQ_RAD_S * node.p['dt_s']


def test_junta_acima_de_24_graus_por_segundo_vira_limite(node):
    assert an.MAX_DQ_RAD_S == pytest.approx(np.radians(24.0))
    guiar(node, (1, 0, 0), 0.25, 30)         # v_max só alcançável por `ros2 param set`
    assert node.state == 'LIMITE'


def test_limite_de_junta_e_a_intersecao_com_o_de_software(node):
    assert np.degrees(node.lim_hi[2]) == pytest.approx(160.0)
    assert np.degrees(node.lim_lo[1]) == pytest.approx(-170.0)
    assert all(node.lim_hi <= [b for _, b in node.chain.limits])


def test_fora_do_limite_de_junta_vira_limite_sem_comando(node, monkeypatch):
    enviados = []
    monkeypatch.setattr(node, '_send', lambda q, t: enviados.append(q))
    real(node, 'real')
    node._begin_active()
    node.state = 'ATIVO'
    node.lim_hi[0] = node.q[0] - 0.1         # o braço já está 0,1 rad além
    node._active_step(0.0, np.zeros(3))
    assert node.state == 'LIMITE' and not enviados


def test_perto_do_limite_de_junta_para_sem_ultrapassar(node):
    node.lim_hi[0] = node.q[0] + 0.02
    node.lim_lo[0] = node.q[0] - 0.02
    guiar(node, (0, 1, 0), 0.10, 60)
    assert node.state == 'LIMITE'
    assert node.lim_lo[0] - 1e-9 <= node.q_cmd[0] <= node.lim_hi[0] + 1e-9


def test_folga_com_a_cinematica_real_recusa_descer_e_deixa_sair(node):
    h = np.array(HOME_Q_RAD)
    for j in (0, 3, 4, 5):
        for d in (-20, 20):
            g = h.copy()
            g[j] += np.radians(d)
            assert an.plan_jog(h, g, node._fk_z, clearance=node._clearance)[0], (j + 1, d)
    # achada por busca: TCP e célula ficam altos, mas o punho/cotovelo invade a mesa
    ruim = np.array([-1.1, -1.18, -2.58, -1.03, 1.31, 2.36])
    assert node._clearance(ruim) < 0
    assert an.plan_jog(h, ruim, node._fk_z)[0]
    ok, motivo, _ = an.plan_jog(h, ruim, node._fk_z, clearance=node._clearance)
    assert not ok and 'cotovelo' in motivo
    # erguer o cotovelo em etapas pode, afundar mais não (a reta até o HOME afundaria antes)
    sobe, desce = (ruim + [0, np.radians(g), 0, 0, 0, 0] for g in (10, -10))
    if node._clearance(sobe) < node._clearance(desce):
        sobe, desce = desce, sobe
    assert node._clearance(desce) < node._clearance(ruim) < node._clearance(sobe)
    assert an.plan_jog(ruim, sobe, node._fk_z, clearance=node._clearance)[0]
    assert not an.plan_jog(ruim, desce, node._fk_z, clearance=node._clearance)[0]


def test_painel_que_trava_com_o_braco_ativo_aborta(node):
    real(node, 'real')
    forca(node)
    node._on_gui(None)
    assert enable(node).success
    node._tick()
    assert node.state == 'ATIVO'
    node.gui_t -= 2.0
    node._tick()
    assert node.state == 'ABORTADO'


def test_sem_painel_desde_o_inicio_nao_aborta(node):
    real(node, 'real')
    forca(node)
    assert enable(node).success
    node._tick()
    assert node.state == 'ATIVO'


def test_tick_usa_tempo_real_e_grava_o_tcp_do_robo_real(node):
    vistos = []
    node.pub_tick.publish = lambda m: vistos.append(dict(zip(an_fields(), m.data)))
    real(node, 'real', q_real=list(HOME_Q_RAD))
    node._tick()
    assert vistos[-1]['t_s'] == pytest.approx(time.monotonic(), abs=0.5)
    assert vistos[-1]['xr_m'] == pytest.approx(vistos[-1]['x_m'])
    node.real_t -= 1.0                                  # status velho não é gravado
    node._tick()
    assert np.isnan(vistos[-1]['xr_m'])


def an_fields():
    from rehab_pack.constants import TICK_FIELDS
    return TICK_FIELDS


def test_velocidade_vem_da_posicao_e_nao_do_campo_do_gazebo(node):
    from sensor_msgs.msg import JointState
    from rehab_pack.constants import ARM_JOINTS
    for i in range(11):
        m = JointState(name=list(ARM_JOINTS), position=[0.4 * 0.01 * i] + [0.0] * 5,
                       velocity=[0.0] * 6)
        m.header.stamp.sec, m.header.stamp.nanosec = 5, i * 10_000_000
        node._on_js(m)
    assert node.dq[0] == pytest.approx(0.4, rel=1e-3)
    m.header.stamp.sec = 9                     # lacuna: não inventa velocidade
    node._on_js(m)
    assert not node.dq.any()
