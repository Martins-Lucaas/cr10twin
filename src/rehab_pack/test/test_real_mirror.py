"""rehab_mirror com driver falso: conectar, modo, sincronizar e armar (precisa do rclpy)."""
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from std_srvs.srv import SetBool                              # noqa: E402
from rehab_pack import real_link, real_mirror                 # noqa: E402
from rehab_pack.constants import HOME_Q_RAD                   # noqa: E402


def wait(cond, s=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < s:
        if cond():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def node():
    rclpy.init(domain_id=77, args=['--ros-args', '-p', 'robot_ip:=sim',
                                   '-p', 'control_mode:=sim_only'])
    n = real_mirror.RealMirror()
    n.state = 'PARADO'
    yield n
    n.destroy_node()
    rclpy.shutdown()


def call(fn, on):
    return fn(SetBool.Request(data=on), SetBool.Response())


def sim_at(n, q):
    n.tick_t = time.monotonic()
    with n.lock:
        n.q_sim, n.q_sim_t = np.array(q, float), time.monotonic()


def test_sim_only_nasce_desconectado_e_conecta_por_servico(node):
    time.sleep(0.3)
    assert not node.link.connected
    assert call(node._srv_connect, True).success
    assert wait(lambda: node.link.connected)
    assert not call(node._srv_connect, True).success
    assert call(node._srv_connect, False).success
    time.sleep(1.3)
    assert not node.link.connected          # não reconecta sozinho


def test_so_espelha_em_mirror_e_troca_so_parado(node):
    call(node._srv_connect, True)
    assert wait(lambda: node.link.connected)
    drv = node.link.drv

    def rodando(estado):
        node.state = estado
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.4:
            sim_at(node, np.array(HOME_Q_RAD) + 0.02 * (time.monotonic() - t0))
            time.sleep(0.01)
    rodando('ATIVO')
    assert not drv.cmds                     # SIM_ONLY
    assert not call(node._srv_mode, True).success
    node.state = 'PARADO'
    assert call(node._srv_mode, True).success
    rodando('ATIVO')
    assert drv.cmds                         # MIRROR
    drv.last_robot_mode, n0 = 9, len(drv.cmds)
    rodando('ATIVO')
    assert len(drv.cmds) == n0              # E-Stop físico: nada sai


def armar(node, offset_rad=0.0, tick=True, s=0.5):
    call(node._srv_connect, True)
    assert wait(lambda: node.link.connected)
    call(node._srv_mode, True)
    drv = node.link.drv
    q0 = drv.q.copy()
    sim_at(node, q0 + offset_rad)
    time.sleep(0.15)
    node.state = 'ATIVO'
    t0 = time.monotonic()
    while time.monotonic() - t0 < s:
        sim_at(node, q0 + offset_rad)
        if not tick:
            node.tick_t = 0.0
        time.sleep(0.01)
    return drv, q0


def test_armar_absorve_a_diferenca_sem_salto(node):
    off = np.radians(2.5)
    drv, q0 = armar(node, off, s=2.2)
    passos = np.max(np.abs(np.diff(np.vstack([q0, *drv.cmds]), axis=0)), axis=1)
    assert passos.max() < np.radians(0.3)
    assert np.allclose(drv.cmds[-1], q0 + off, atol=1e-4)


def test_sem_batimento_da_admitancia_nao_espelha(node):
    drv, _ = armar(node, tick=False)
    assert not drv.cmds


def test_parar_direto_e_desconectar_valem_em_ativo(node):
    from std_msgs.msg import String
    drv, q0 = armar(node)
    node._on_event(String(data='ESTOP;origem=gui'))
    assert drv.stopped == 1
    n0 = len(drv.cmds)
    sim_at(node, q0 + 0.01); time.sleep(0.2)
    assert len(drv.cmds) == n0
    assert call(node._srv_connect, False).success and not node.link.connected


def test_queda_nao_religa_o_robo_sozinha(node):
    call(node._srv_connect, True)
    assert wait(lambda: node.link.connected)

    def cai():
        raise OSError('cabo')
    node.link.drv.read_joints_urdf_latest = cai
    assert wait(lambda: not node.link.connected)
    time.sleep(1.5)
    assert not node.link.connected and not node.want


def test_nao_arma_acima_de_3_graus_mesmo_ja_ativo(node):
    call(node._srv_connect, True)
    assert wait(lambda: node.link.connected)
    call(node._srv_mode, True)
    node.state = 'ATIVO'
    for _ in range(30):
        sim_at(node, np.array(HOME_Q_RAD) + np.radians(5))
        time.sleep(0.01)
    assert not node.link.drv.cmds


def test_conexao_lenta_nao_segura_o_lock_e_pode_ser_cancelada(node):
    drv = real_link.SimulatedDriver()
    drv.enable = lambda: time.sleep(1.0)
    def stop():
        raise OSError('socket caiu')
    drv.stop = stop
    fechado = []
    drv.close = lambda: fechado.append(1)
    with node.lock:
        node.link = real_link.RealLink('sim', 0.03, factory=lambda *a: drv)
        node.want = True
    node.wake.set()
    assert wait(lambda: node.connecting)
    t0 = time.monotonic()
    assert call(node._srv_connect, False).success
    assert time.monotonic() - t0 < 0.3
    assert wait(lambda: fechado) and not node.link.connected
    assert any(t.name.endswith('conectar') and t.is_alive() for t in threading.enumerate())


def test_juntas_zeradas_do_firmware_sao_ignoradas_por_pouco_tempo(node):
    call(node._srv_connect, True)
    assert wait(lambda: node.q_real is not None)
    sim_at(node, node.link.drv.q)
    drv, q = node.link.drv, node.q_real.copy()
    drv.q = np.zeros(6)
    time.sleep(0.1)
    assert np.allclose(node.q_real, q)          # blip: segura a última leitura
    assert wait(lambda: node.q_real is None)    # persistente: sem posição, não arma
