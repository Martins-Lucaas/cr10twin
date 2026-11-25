"""Regressões da auditoria de 24/11/2026: espelho, admitância, força e trava (precisa do ROS)."""
import importlib.util
import json
import math
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('xacro')

from geometry_msgs.msg import PointStamped, WrenchStamped     # noqa: E402
from rclpy.parameter import Parameter                         # noqa: E402
from sensor_msgs.msg import JointState                        # noqa: E402
from std_msgs.msg import Float64, Float64MultiArray, String   # noqa: E402
from std_srvs.srv import SetBool, Trigger                     # noqa: E402
from trajectory_msgs.msg import JointTrajectory               # noqa: E402
from rehab_pack import admittance_node as an                  # noqa: E402
from rehab_pack import real_mirror                            # noqa: E402
from rehab_pack.admittance import AdmittanceController        # noqa: E402
from rehab_pack.constants import (ARM_JOINTS, HOME_Q_RAD, T_ARM_CMD)   # noqa: E402
from rehab_pack.real_link import RealLink, SimulatedDriver, real_ok     # noqa: E402

HOME = np.array(HOME_Q_RAD)
OK = {'conectado': True, 'estop': False, 'sync_ok': True, 'modo': 'MIRROR',
      'ip': '10.0.0.1', 'robot_mode': 5, 'q_real': list(HOME_Q_RAD)}


def wait(cond, s=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < s:
        if cond():
            return True
        time.sleep(0.01)
    return False


def chama(fn, on=True):
    return fn(SetBool.Request(data=on), SetBool.Response())


# ── XC9, R13, XR15, XR16: espelho ───────────────────────────────────────────
@pytest.fixture
def mirror():
    rclpy.init(domain_id=72, args=['--ros-args', '-p', 'robot_ip:=sim',
                                   '-p', 'control_mode:=mirror'])
    n = real_mirror.RealMirror()
    n.state = 'PARADO'
    yield n
    n.destroy_node()
    rclpy.shutdown()


def sim(n, q):
    n.tick_t = time.monotonic()
    with n.lock:
        n.q_sim, n.q_sim_t = np.array(q, float), time.monotonic()


def segura(n, q, s):
    t0 = time.monotonic()
    while time.monotonic() - t0 < s:
        sim(n, q)
        time.sleep(0.005)


def arma(n):
    chama(n._srv_connect)
    assert wait(lambda: n.q_real is not None)
    drv = n.link.drv
    q0 = drv.q.copy()
    segura(n, q0, 0.15)
    n.state = 'ATIVO'
    segura(n, q0, 0.2)
    assert n.streaming
    return drv, q0


def test_xc9_degrau_do_simulado_nao_vira_servoj_e_trava(mirror):
    drv, q0 = arma(mirror)
    segura(mirror, q0 + np.radians([7, 0, 0, 0, 0, 0]), 0.3)
    passos = np.abs(np.diff(np.vstack([q0, *drv.cmds]), axis=0))
    assert passos.max() < np.radians(1.5)          # antes: 7° num ServoJ de 30 ms
    assert not mirror.streaming and drv.stopped == 1
    segura(mirror, q0, 0.3)                        # o sim voltou: NÃO retoma sozinho
    assert not mirror.streaming
    st = {**OK, 'travado': mirror.halted}
    assert not real_ok(st, 0.0)[0]                 # a admitância aborta e não habilita
    mirror._on_event(String(data='RESET'))         # Reset do operador
    mirror.state = 'ATIVO'
    segura(mirror, q0, 0.3)
    assert mirror.streaming


def test_xc9_degrau_durante_a_rampa_ao_armar(mirror):
    drv, q0 = arma(mirror)                         # ainda dentro de RAMP_S
    segura(mirror, q0 + np.radians([0, 0, 2.5, 0, 0, 0]), 0.2)
    assert not mirror.streaming and mirror.halted


def test_xc9_retomada_so_perto_do_robo_real(mirror):
    drv, q0 = arma(mirror)
    time.sleep(0.25)                               # Gazebo travado: /joint_states velho
    assert not mirror.streaming
    n0 = len(drv.cmds)
    segura(mirror, q0 + np.radians([6, 0, 0, 0, 0, 0]), 0.3)   # volta a 6° (< 8° de seguimento)
    assert len(drv.cmds) == n0


def test_xc9_primeira_amostra_parte_da_pose_real_e_nan_nao_sai():
    drv = SimulatedDriver(np.zeros(6))
    l = RealLink('sim', 0.03)
    l.attach(drv)
    for q in (np.full(6, np.radians(2)), np.full(6, np.nan)):
        with pytest.raises(RuntimeError):
            l.stream(q, np.zeros(6))
    assert not drv.cmds and l.connected
    assert l.stream(np.full(6, np.radians(1)), np.zeros(6))


def test_r13_um_timeout_de_feedback_para_sem_derrubar(mirror):
    from touch_pack import real_driver
    atraso = getattr(real_driver, 'CR10FeedbackTimeout', TimeoutError)
    drv, q0 = arma(mirror)
    le, falhas = drv.read_joints_urdf_latest, [2]

    def atrasa():
        if falhas[0]:
            falhas[0] -= 1
            raise atraso('atraso')
        return le()
    vistos = []
    mirror.pub.publish = lambda m: vistos.append(json.loads(m.data))
    drv.read_joints_urdf_latest = atrasa
    segura(mirror, q0, 0.3)
    assert mirror.link.connected
    ruins = [s for s in vistos if s['q_real'] is None]
    assert len(ruins) == 2 and not any(real_ok(s, 0.0)[0] for s in ruins)
    falhas[0] = 100                                # seguidas: perdida de vez
    n0 = len(drv.cmds)
    assert wait(lambda: not mirror.link.connected)
    assert len(drv.cmds) - n0 <= 2


def test_r13_pose_real_vencida_ou_nao_finita_nao_move(mirror):
    drv, q0 = arma(mirror)
    trava = threading.Event()
    drv.read_joints_urdf_latest = lambda: (trava.wait(0.6), q0)[1]   # leitura presa
    segura(mirror, q0, 0.35)
    assert not mirror.streaming
    trava.set()
    drv.read_joints_urdf_latest = lambda: np.full(6, np.nan)
    segura(mirror, q0, 0.2)
    assert mirror.q_real is None and not mirror.sync_ok


def test_xr15_real_ainda_chegando_em_limite_nao_perde_a_sincronia(mirror):
    drv, q0 = arma(mirror)
    drv.servo_j_urdf = lambda q: None              # o real ficou 3,5° para trás
    q = q0 + np.radians([3.5, 0, 0, 0, 0, 0])
    for s in np.linspace(0, 1, 40):
        sim(mirror, q0 + s * (q - q0))
        time.sleep(0.03)
    assert mirror.streaming and mirror.sync_ok
    mirror.state = 'LIMITE'
    segura(mirror, q, 0.2)
    assert mirror.sync_ok                          # LIMITE é recuperável
    segura(mirror, q, 0.6)
    assert not mirror.sync_ok                      # mas não para sempre


def test_xr16_encerrar_espera_as_threads_e_para_o_robo():
    rclpy.init(domain_id=72, args=['--ros-args', '-p', 'robot_ip:=sim',
                                   '-p', 'control_mode:=mirror'])
    n = real_mirror.RealMirror()
    n.state = 'PARADO'
    drv, _ = arma(n)
    n.destroy_node()
    rclpy.shutdown()
    vivas = [t.name for t in threading.enumerate() if t.name.startswith('rehab-mirror')]
    assert not [v for v in vivas if 'conectar' not in v] and drv.stopped == 1


@pytest.mark.parametrize('p', ['0.0', '-0.03', '.nan', '1.0', '0.001'])
def test_r08_periodo_do_servoj_invalido_nao_sobe(p):
    rclpy.init(domain_id=72, args=['--ros-args', '-p', f'servoj_period_s:={p}'])
    n = None
    try:
        with pytest.raises(ValueError):
            n = real_mirror.RealMirror()
    finally:
        if n:
            n.destroy_node()
        rclpy.shutdown()


# ── admitância ──────────────────────────────────────────────────────────────
@pytest.fixture
def node():
    rclpy.init(domain_id=72)
    n = an.AdmittanceNode()
    js(n, HOME)
    n.p['force_source'] = 'sim'
    n.evs = []
    if hasattr(n, 'pub_event'):
        n.pub_event.publish = lambda m: n.evs.append(m.data)
    yield n
    n.destroy_node()
    rclpy.shutdown()


def js(n, q):
    n._on_js(JointState(name=list(ARM_JOINTS), position=[float(v) for v in q]))


def forca(n, fx=0.0):
    m = WrenchStamped()
    m.header.frame_id = 'load_cell_link'
    m.wrench.force.x = float(fx)
    n._on_ft(m)


def ativa(n):
    js(n, n.q)
    forca(n)
    r = chama(n._srv_enable)
    assert r.success, r.message


@pytest.mark.parametrize('modo,idade', [('MIRROR', 0.2), ('SIM_ONLY', 0.8)])
def test_xc9_joint_states_velho_em_ativo_aborta(node, modo, idade):
    node._on_real(String(data=json.dumps({**OK, 'modo': modo, 'ip': 'sim'})))
    ativa(node)
    node._tick()
    assert node.state == 'ATIVO'
    time.sleep(idade)
    node._on_real(String(data=json.dumps({**OK, 'modo': modo, 'ip': 'sim'})))
    forca(node)
    node._tick()
    assert node.state == 'ABORTADO'


def test_r01_publicador_de_fora_recusa_e_aborta(node):
    estranho = rclpy.create_node('combined_control_gui')
    ativa(node)
    pub = estranho.create_publisher(JointTrajectory, T_ARM_CMD, 10)
    assert wait(lambda: node.count_publishers(T_ARM_CMD) == 2)
    time.sleep(0.3)
    js(node, node.q)
    node._tick()
    assert node.state == 'ABORTADO'
    node._srv_reset(None, Trigger.Response())
    js(node, node.q)
    r = chama(node._srv_enable)
    assert not r.success and 'outro nó' in r.message
    type(node.pub_cmd).get_subscription_count, antes = lambda s: 1, type(
        node.pub_cmd).get_subscription_count
    try:
        node._on_goto(Float64MultiArray(data=[*(node.q + 0.05), 50.0]))
    finally:
        type(node.pub_cmd).get_subscription_count = antes
    assert node.state == 'PARADO' and pub
    estranho.destroy_node()


def test_r02_j6_no_limite_nao_gira_o_mastro(node):
    node.lim_lo[5], node.lim_hi[5] = node.q[5] - 0.01, node.q[5] + 0.01
    R0 = node.chain.fk(node.q)[:3, :3]
    node._begin_active()
    node.state = 'ATIVO'
    giro = 0.0
    estados = set()
    for _ in range(50):
        node.force_w = np.array([0.0, -8.0, 0.0])
        node._active_step(0.0, np.zeros(3))
        node.q = node.q_cmd.copy()
        estados.add(node.state)
        giro = max(giro, np.linalg.norm(an._log(node.chain.fk(node.q)[:3, :3] @ R0.T)))
    assert 'LIMITE' in estados
    assert np.degrees(giro) < 2.2                   # antes: 15°


@pytest.mark.parametrize('pos', [
    [math.nan] * 6, [math.inf] + [0.0] * 5, [0.0] * 3, []])
def test_r09_joint_states_invalido_ou_incompleto_e_ignorado(node, pos):
    q = node.q.copy()
    node._on_js(JointState(name=list(ARM_JOINTS), position=pos))
    assert np.allclose(node.q, q)
    rclpy.logging.get_logger('x')
    m = real_mirror.RealMirror.__new__(real_mirror.RealMirror)
    m.lock, m.q_sim = threading.RLock(), None
    m._on_js(JointState(name=list(ARM_JOINTS), position=pos))
    assert m.q_sim is None


def test_r09_joint_states_vencido_nao_habilita_nem_posiciona(node, monkeypatch):
    monkeypatch.setattr(type(node.pub_cmd), 'get_subscription_count', lambda s: 1)
    forca(node)
    time.sleep(0.6)
    forca(node)
    assert not chama(node._srv_enable).success
    node._on_goto(Float64MultiArray(data=[*(node.q + 0.05), 50.0]))
    assert node.state == 'PARADO'


def ref(n, x):
    m = PointStamped()
    m.point.x, m.point.y, m.point.z = map(float, x)
    n._on_ref(m)


def test_r10_referencia_fora_da_excursao_recusada_e_nao_move_sem_forca(node):
    ativa(node)
    x0 = node.ctrl.x.copy()
    ref(node, x0 + [0.25, 0, 0])
    assert np.allclose(node.ctrl.x_ref, x0)
    ref(node, x0 + [0.05, 0, 0])
    assert node.ctrl.x_ref[0] == pytest.approx(x0[0] + 0.05)


@pytest.mark.parametrize('mudar', [lambda c: c.set_reference([0.25, 0, 0]),
                                   lambda c: setattr(c, 'max_excursion', 0.02)])
def test_r10_sem_mola_e_sem_forca_o_nucleo_nao_anda(mudar):
    c = AdmittanceController()
    for _ in range(40):
        c.step([8, 0, 0])
    for _ in range(40):
        c.step([0, 0, 0])
    x0 = c.x.copy()
    assert x0[0] > 0.03
    mudar(c)
    for _ in range(60):
        c.step([0, 0, 0])
    assert np.linalg.norm(c.x - x0) < 1e-4
    c.step([8, 0, 0])                              # e para fora continua barrado
    assert c.x[0] <= max(x0[0], c.max_excursion) + 1e-4 or c.max_excursion > 0.1


def test_r12_ik_sem_convergir_tem_teto_de_iteracoes_e_vira_limite(node, monkeypatch):
    ativa(node)
    n, real = [0], node.chain.dls_step

    def conta(*a, **k):
        n[0] += 1
        return real(*a, **k)
    monkeypatch.setattr(node.chain, 'dls_step', conta)
    enviados = []
    monkeypatch.setattr(node, '_send', lambda q, t: enviados.append(q))
    monkeypatch.setattr(node.ctrl, 'step', lambda *a: np.array([5.0, 5.0, 5.0]))
    node._active_step(0.0, np.zeros(3))
    assert node.state == 'LIMITE' and not enviados and n[0] <= 5


def test_r17_aborto_publica_o_motivo(node):
    ativa(node)
    forca(node, 40.0)
    node._tick()
    assert node.state == 'ABORTADO'
    assert [e for e in node.evs if e.startswith('ABORT;motivo=pico de força')]
    node._srv_estop(None, Trigger.Response())
    assert len(node.evs) == 1                      # já abortado: não repete


def test_r08_avanco_da_trajetoria_menor_que_o_tick_recusado(node):
    node.p['force_source'] = node.get_parameter('force_source').value
    assert not node.set_parameters([Parameter('traj_lead_s', value=0.02)])[0].successful
    assert node.set_parameters([Parameter('traj_lead_s', value=0.09)])[0].successful


# ── R05, R06: validade da tara ──────────────────────────────────────────────
def tara(n):
    n._on_tared(Float64(data=time.monotonic()))


def test_r06_tara_usa_a_pose_do_robo_que_segura_a_celula(node):
    node.p['force_source'] = 'real'
    torto = HOME + [0, 0, 0, 0, 0.3, 0]
    tara(node)                                     # desconectado: sim no HOME
    assert node.tare_R is not None
    node._on_real(String(data=json.dumps({**OK, 'modo': 'SIM_ONLY'})))   # conectou
    assert node.tare_R is None
    node._on_real(String(data=json.dumps({**OK, 'modo': 'SIM_ONLY', 'q_real': list(torto)})))
    tara(node)                                     # real inclinado, sim no HOME
    node._on_real(String(data=json.dumps({**OK, 'modo': 'SIM_ONLY'})))   # real foi ao HOME
    assert 'girou' in node._tare_check()[1]
    node._on_real(String(data=json.dumps({**OK, 'q_real': None})))
    assert not node._tare_check()[0]               # sem pose real não se sabe
    tara(node)
    assert node.tare_R is None


def test_r05_tara_invalidada_em_ativo_aborta(node):
    node.p['force_source'] = 'real'
    tara(node)
    ativa(node)
    node._tick()
    assert node.state == 'ATIVO'
    node._on_tared(Float64(data=0.0))
    node._tick()
    assert node.state == 'ABORTADO'


@pytest.mark.parametrize('lixo', ['[1]', '3', 'null', '{ruim'])
def test_status_do_robo_que_nao_e_objeto_nao_derruba(node, lixo):
    node._on_real(String(data=lixo))
    node._tick()
    assert node.real is None


# ── R04, R05, R07: força ────────────────────────────────────────────────────
def fonte(*args):
    from rehab_pack.force_source import ForceSource
    rclpy.init(domain_id=72, args=['--ros-args', *(v for a in args for v in ('-p', a))])
    n = ForceSource()
    n.taras = []
    n._pub_tared.publish = lambda m: n.taras.append(m.data)
    return n


def amostra(n, fx=2.5):
    m = WrenchStamped()
    m.wrench.force.x = fx
    n._on_wrench(m)


def estado(n, s):
    if hasattr(n, '_on_state'):
        n._on_state(String(data=s))
    else:
        n._state = s


def faz_tara(n, meio=lambda: None):
    amostra(n)                                     # a célula já estava no ar
    r = Trigger.Response()
    t = threading.Thread(target=lambda: n._on_tare(None, r))
    t.start()
    k = 0
    while t.is_alive():
        amostra(n)
        k += 1
        if k == 20:
            meio()
        time.sleep(0.01)
    return r


@pytest.fixture
def fs():
    n = fonte()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_r04_habilitar_no_meio_da_tara_descarta_a_janela(fs):
    estado(fs, 'PARADO')
    assert faz_tara(fs).success
    assert fs.taras[0] == 0.0 and fs.taras[-1] > 0.0   # 0 no início da janela
    fs._tare.offset[:] = 0.0

    def liga_e_desliga():
        estado(fs, 'ATIVO')
        estado(fs, 'PARADO')
    r = faz_tara(fs, liga_e_desliga)
    assert not r.success and np.allclose(fs._tare.offset, 0.0) and fs.taras[-1] == 0.0


def test_r04_duas_taras_ao_mesmo_tempo(fs):
    estado(fs, 'PARADO')
    segunda = []
    r = faz_tara(fs, lambda: segunda.append(fs._on_tare(None, Trigger.Response())))
    assert r.success and not segunda[0].success


def test_r05_celula_muda_ou_rezerada_invalida_a_tara(fs):
    estado(fs, 'PARADO')
    assert faz_tara(fs).success
    fs.taras.clear()
    time.sleep(0.6)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.4:             # silêncio: só os timers do nó
        rclpy.spin_once(fs, timeout_sec=0.02)
    assert fs.taras == [0.0]
    assert faz_tara(fs).success
    fs.taras.clear()
    time.sleep(0.6)
    amostra(fs)                                    # voltou sem o vigia ter rodado
    assert fs.taras == [0.0]


def test_r05_tare_do_ft_receiver_invalida_pelo_topico(fs):
    estado(fs, 'PARADO')
    assert faz_tara(fs).success
    fs.taras.clear()
    outro = rclpy.create_node('ft_receiver')
    pub = outro.create_publisher(String, '/load_cell/tare_result', 10)
    assert wait(lambda: pub.get_subscription_count() == 1)
    pub.publish(String(data='err;drifting;1'))

    def gira():
        amostra(fs)
        rclpy.spin_once(fs, timeout_sec=0.02)
        return bool(fs.taras)
    assert not wait(gira, 0.3)                     # recusada: o zero não mudou
    pub.publish(String(data='ok;1.0;0.0'))
    assert wait(gira)
    outro.destroy_node()


@pytest.mark.parametrize('p', ['real_sign:=0.0', 'real_sign:=0.5', 'real_sign:=.nan',
                               'real_sign:=-2.0', 'sim_sign:=0.0', 'real_yaw_deg:=.inf'])
def test_r07_sinal_e_giro_invalidos_nao_sobem(p):
    n = None
    try:
        with pytest.raises(ValueError):
            n = fonte(p)
    finally:
        if n:
            n.destroy_node()
        rclpy.shutdown()


# ── R01: todo produtor de comando do braço toma a trava ─────────────────────
@pytest.mark.parametrize('mod', ['hand_pack.combined_gui', 'grasp_ml_pack.manual_control_node',
                                 'grasp_ml_pack.grasp_executor'])
def test_r01_produtores_recusam_com_o_rehab_aberto(mod, monkeypatch, tmp_path):
    import fcntl
    from touch_pack import pack_lock
    m = pytest.importorskip(mod)
    monkeypatch.setenv('CR10_PACK_LOCK_DIR', str(tmp_path))
    monkeypatch.setattr(pack_lock, '_held', {})

    def subiu(*a, **k):
        raise AssertionError('subiu com o rehab aberto')
    monkeypatch.setattr(rclpy, 'init', subiu)
    if hasattr(m, 'tk'):
        monkeypatch.setattr(m.tk, 'Tk', subiu)
    with open(tmp_path / 'rehab_pack.lock', 'a') as f:
        fcntl.flock(f, fcntl.LOCK_SH)
        with pytest.raises(SystemExit):
            m.main()
    with open(tmp_path / 'touch_pack.lock', 'a') as f:       # touch aberto: convive
        fcntl.flock(f, fcntl.LOCK_SH)
        with pytest.raises(AssertionError):
            m.main()


@pytest.mark.parametrize('rel', ['hand_pack/launch/cr10_covvi_gazebo.launch.py',
                                 'grasp_ml_pack/launch/conveyor_cell.launch.py'])
def test_r01_launches_com_controlador_do_braco_recusam(rel, monkeypatch, tmp_path):
    import fcntl
    import pathlib
    pytest.importorskip('launch')
    from touch_pack import pack_lock
    f = pathlib.Path(__file__).parents[2] / rel
    if not f.exists():
        pytest.skip(f'{rel} fora desta árvore')
    spec = importlib.util.spec_from_file_location('l', f)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except ImportError as e:
        pytest.skip(str(e))
    monkeypatch.setenv('CR10_PACK_LOCK_DIR', str(tmp_path))
    monkeypatch.setattr(pack_lock, '_held', {})
    with open(tmp_path / 'rehab_pack.lock', 'a') as h:
        fcntl.flock(h, fcntl.LOCK_SH)
        with pytest.raises(pack_lock.PackLockError):
            mod.generate_launch_description()


@pytest.mark.parametrize('arg', ['ft_sign:=0', 'ft_sign:=0.5', 'ft_sign:=nan', 'ft_yaw_deg:=inf'])
def test_r07_launch_recusa_sinal_e_giro_invalidos(arg, monkeypatch, tmp_path):
    import pathlib
    launch = pytest.importorskip('launch')
    from launch.actions import DeclareLaunchArgument
    from touch_pack import pack_lock
    monkeypatch.setenv('CR10_PACK_LOCK_DIR', str(tmp_path))
    monkeypatch.setattr(pack_lock, '_held', {})
    f = pathlib.Path(__file__).parents[1] / 'launch' / 'joystick_cell.launch.py'
    spec = importlib.util.spec_from_file_location('jl', f)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = launch.LaunchContext()
    k, v = arg.split(':=')
    ctx.launch_configurations[k] = v
    for e in mod.generate_launch_description().entities:
        if isinstance(e, DeclareLaunchArgument):
            e.visit(ctx)
    with pytest.raises(RuntimeError, match='ft_sign|finito'):
        mod.launch_setup(ctx)
