"""Ponte do painel contra serviços ROS de verdade e driver real contra a bancada falsa."""
import importlib.util
import json
import math
import pathlib
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('xacro')

from rcl_interfaces.msg import SetParametersResult             # noqa: E402
from rclpy.executors import SingleThreadedExecutor             # noqa: E402
from std_msgs.msg import String                                # noqa: E402
from std_srvs.srv import SetBool, Trigger                      # noqa: E402
from rehab_pack import constants as C                          # noqa: E402
from rehab_pack import real_link                               # noqa: E402
from rehab_pack.gui_bridge import NODE_ADMITTANCE, GuiBridge   # noqa: E402
from rehab_pack.real_link import RealLink             # noqa: E402


def espera(cond, s=4.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < s:
        if cond():
            return True
        time.sleep(0.01)
    return False


# ── GuiBridge (XR22) ────────────────────────────────────────────────────────
@pytest.fixture
def ponte():
    """GuiBridge e um nó falso com o nome e os serviços pedidos."""
    import threading
    rclpy.init(domain_id=72)
    br, nos, ex, fim = GuiBridge(), [], SingleThreadedExecutor(), threading.Event()

    def no(nome):
        n = rclpy.create_node(nome)
        n.ordem = []
        nos.append(n)
        return n

    def roda():
        for n in nos:
            ex.add_node(n)

        def laco():
            while not fim.is_set():
                ex.spin_once(timeout_sec=0.02)
        t = threading.Thread(target=laco, daemon=True)
        t.start()
        return t
    br.no, br.roda = no, roda
    yield br
    fim.set()
    time.sleep(0.1)
    br.shutdown()
    for n in nos:
        n.destroy_node()
    rclpy.shutdown()


def espelho(br, aceita=True):
    n = br.no(C.NODE_MIRROR)
    n.declare_parameter('robot_ip', '')

    def grava(params):
        n.ordem += [(p.name, p.value) for p in params]
        return SetParametersResult(successful=aceita, reason='' if aceita else 'IP inválido')
    n.add_on_set_parameters_callback(grava)

    def conecta(req, res):
        n.ordem.append(('connect', req.data))
        res.success, res.message = True, 'ok'
        return res
    n.create_service(SetBool, C.S_REAL_CONNECT, conecta)
    br.roda()
    assert espera(lambda: br.service_ready(C.S_REAL_CONNECT)
                  and br._setparams[C.NODE_MIRROR].service_is_ready())
    return n


def test_connect_real_grava_o_ip_antes_de_conectar(ponte):
    n = espelho(ponte)
    ponte.connect_real('10.0.0.9')
    assert espera(lambda: (C.S_REAL_CONNECT, True, 'ok') in ponte.results)
    assert n.ordem == [('robot_ip', '10.0.0.9'), ('connect', True)]


def test_connect_real_com_ip_recusado_nao_conecta(ponte):
    n = espelho(ponte, aceita=False)
    ponte.connect_real('lixo')
    assert espera(lambda: ponte.results)
    time.sleep(0.3)
    assert ('connect', True) not in n.ordem
    rot, ok, msg = ponte.results[0]
    assert rot == f'params:{C.NODE_MIRROR}' and not ok and 'robot_ip: IP inválido' in msg


def test_set_params_manda_double_e_diz_qual_foi_recusado(ponte):
    n = ponte.no(NODE_ADMITTANCE)
    n.declare_parameter('mass_kg', 2.0)
    n.declare_parameter('damping_ns_m', 40.0)
    n.add_on_set_parameters_callback(lambda ps: SetParametersResult(
        successful=ps[0].name == 'mass_kg', reason='instável'))
    ponte.roda()
    assert espera(ponte._setparams[NODE_ADMITTANCE].service_is_ready)
    ponte.set_params(NODE_ADMITTANCE, {'mass_kg': 3, 'damping_ns_m': 1})
    assert espera(lambda: ponte.results)
    assert ponte.results[0] == (f'params:{NODE_ADMITTANCE}', False, 'damping_ns_m: instável')
    assert n.get_parameter('mass_kg').value == 3.0


def test_start_session_chega_depois_do_meta(ponte):
    n = ponte.no('rehab_logger')
    qos = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(String, C.T_SESSION_META,
                          lambda m: n.ordem.append(json.loads(m.data)['subject_id']), qos)

    def inicia(_req, res):
        n.ordem.append('start')
        res.success, res.message = True, '/tmp/s'
        return res
    n.create_service(Trigger, C.S_SESSION_START, inicia)
    ponte.roda()
    assert espera(lambda: ponte.service_ready(C.S_SESSION_START))
    ponte.publish_meta({'subject_id': 'S01'})
    ponte.start_session()
    assert espera(lambda: (C.S_SESSION_START, True, '/tmp/s') in ponte.results)
    assert n.ordem == ['S01', 'start']


def test_sem_os_nos_no_ar_a_ponte_responde_na_hora(ponte):
    ponte.set_params(NODE_ADMITTANCE, {'mass_kg': 3.0})
    ponte.call_enable(True)
    ponte.connect_real('10.0.0.9')
    ponte.start_session()
    assert espera(lambda: len(ponte.results) == 4)
    assert not any(ok for _, ok, _ in ponte.results)


# ── driver real × bancada falsa ─────────────────────────────────────────────
def _script(nome):
    f = pathlib.Path(__file__).parents[1] / 'scripts' / f'{nome}.py'
    spec = importlib.util.spec_from_file_location(nome, f)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.fixture(scope='module')
def robo():
    fb = _script('fake_bench')
    try:
        return fb.FakeCR10([20.3, -52.8, -111.1, -106.1, 90.0, 69.7], host='127.0.0.2')
    except OSError as e:
        pytest.skip(f'porta do CR10 ocupada: {e}')


@pytest.fixture
def link(robo):
    robo.mode, robo.max_step = 4, 0.0
    l = RealLink('127.0.0.2', 0.03)
    assert l.connect_once(), l.last_error
    yield l
    if l.drv is not None:
        l.disconnect()


def test_bancada_conecta_habilita_e_le_as_juntas(link, robo):
    from touch_pack.kinematics import urdf_to_dobot
    q = link.read_q()
    assert q is not None, link.last_error
    assert np.allclose(np.degrees(urdf_to_dobot(q)), robo.q, atol=0.05)
    assert link.robot_mode == 5


def test_bancada_rampa_de_servoj_chega_em_passos_pequenos(link, robo):
    q0 = link.read_q()
    n0 = robo.servo_n
    for i in range(1, 11):
        assert link.stream(q0 + [0.005 * i, 0, 0, 0, 0, 0], q0)
        time.sleep(0.03)
    assert espera(lambda: robo.servo_n - n0 == 10)
    assert 0 < robo.max_step < math.degrees(link.max_step_rad)
    with pytest.raises(real_link.StepTooLarge):
        link.stream(q0 + np.radians([7, 0, 0, 0, 0, 0]), q0)
    time.sleep(0.1)
    assert robo.servo_n - n0 == 10                 # o salto não saiu do PC


def test_bancada_alarme_recusa_servoj_sem_limpar_erro(link, robo):
    q0 = link.read_q()
    assert link.stream(q0 + [0.005, 0, 0, 0, 0, 0], q0)
    robo.mode = 9
    robo.cmds.clear()
    for i in range(2, 8):                          # o ACK pode chegar um ponto depois
        if link.connected:
            link.stream(q0 + [0.005 * i, 0, 0, 0, 0, 0], q0)
            time.sleep(0.03)
    assert not link.connected
    assert not [c for c in robo.cmds if c.startswith(('ClearError', 'EnableRobot', 'Continue'))]


@pytest.mark.parametrize('cmd', ['SetCollisionLevel', 'SpeedFactor'])
def test_bancada_enable_sem_confirmacao_nao_conecta(robo, monkeypatch, cmd):
    robo.mode = 4
    trata = robo._handle
    monkeypatch.setattr(robo, '_handle', lambda c: (
        f'-1,{{}},{c};' if c.startswith(cmd) else trata(c)))
    l = RealLink('127.0.0.2', 0.03)
    assert not l.connect_once() and not l.connected
    assert cmd in l.last_error


def test_bancada_feedback_atrasado_nao_derruba_e_cabo_puxado_derruba(link, robo):
    link.drv._feed.settimeout(0.001)               # menor que o período de 8 ms
    with pytest.raises(Exception) as e:
        for _ in range(20):
            link.drv.read_joints_urdf_latest()
    assert real_link.is_feedback_timeout(e.value)
    link.drv._feed.settimeout(0.05)
    assert link.read_q() is not None               # a mesma conexão continua
    robo.drop()
    for _ in range(5):
        link.read_q()
    assert not link.connected


# ── check_sim_ft --sinal (XR18b) ────────────────────────────────────────────
@pytest.mark.parametrize('s', [1, -1])
def test_sinal_sugerido_leva_a_forca_para_mais_x(s):
    pytest.importorskip('gazebo_msgs')
    m = _script('check_sim_ft')
    urdf = m.build_robot_urdf(m.get_package_share_directory('cra_description'),
                              m.get_package_share_directory('rehab_pack'))[0]
    q = np.array(C.HOME_Q_RAD) + [0.4, 0.1, 0, 0, 0, 0.3]
    R = m.Chain(urdf, tip='load_cell_link').fk(q)[:3, :3]
    sug, d = m.sinal(R.T @ [2.0 * s, 0, 0], q)
    assert sug == s and np.allclose(d * sug, [2, 0, 0])
    assert m.sinal([0.0, 0.0, 0.0], q)[0] == 0     # a força não chegou: não inventa
    assert m.sinal(R.T @ [0, 2.0, 0], q)[0] == 0


def test_skip_de_modulo_nao_zera_a_coleta(tmp_path):
    """Com o launch_testing carregado, um arquivo pulado não leva os outros."""
    import os
    import shutil
    import subprocess
    import sys
    pytest.importorskip('launch_testing')
    shutil.copy(pathlib.Path(__file__).with_name('conftest.py'), tmp_path)
    (tmp_path / 'test_a.py').write_text("import pytest\npytest.importorskip('nao_existe')\n")
    (tmp_path / 'test_b.py').write_text('def test_ok(): pass\n')
    r = subprocess.run([sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider', '.'],
                       cwd=tmp_path, env=dict(os.environ), capture_output=True, text=True)
    assert '1 passed, 1 skipped' in r.stdout, r.stdout + r.stderr
