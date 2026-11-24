"""Ligação ao CR10 real: decisão de segurança, IP, reconexão e ServoJ (sem hardware)."""
import json
import math

import numpy as np
import pytest

from rehab_pack.constants import SYNC_TOL_RUN_RAD, SYNC_TOL_START_RAD
from rehab_pack.real_link import (DEADBAND_RAD, RealLink, SimulatedDriver,
                                  real_ok, robot_ip, save_robot_ip,
                                  sync_error_rad)

OK = {'conectado': True, 'estop': False, 'sync_ok': True, 'max_dq_deg': 0.4}


def test_real_ok_so_com_tudo_certo():
    assert real_ok(OK, 0.1) == (True, 'ok')


@pytest.mark.parametrize('st,age,trecho', [
    (None, 0.0, 'sem status'),
    (OK, 5.0, 'sem status'),                                   # status velho
    ({**OK, 'conectado': False}, 0.1, 'desconectado'),
    ({**OK, 'estop': True}, 0.1, 'E-Stop'),
    ({**OK, 'robot_mode': 9}, 0.1, 'RobotMode 9'),              # E-Stop físico / alarme
    ({**OK, 'robot_mode': 11}, 0.1, 'RobotMode 11'),           # colisão
    ({**OK, 'sync_ok': False, 'max_dq_deg': 7.5}, 0.1, '7.5'),
])
def test_real_ok_recusa(st, age, trecho):
    ok, motivo = real_ok(st, age)
    assert not ok and trecho in motivo


def test_estop_vence_mesmo_sincronizado():
    assert not real_ok({**OK, 'estop': True, 'sync_ok': True}, 0.0)[0]


def test_tolerancias_arma_menor_que_seguimento():
    assert SYNC_TOL_START_RAD < SYNC_TOL_RUN_RAD
    assert math.degrees(SYNC_TOL_START_RAD) == pytest.approx(3.0, abs=0.05)


def test_sync_error_e_o_maior_desvio():
    assert sync_error_rad([0, 0, 0, 0, 0, 0], [0, 0.1, -0.3, 0, 0, 0.2]) == pytest.approx(0.3)


def test_robot_ip_prioridade(tmp_path):
    cfg = tmp_path / 'robot.json'
    cfg.write_text(json.dumps({'robot_ip': '10.0.0.7'}))
    assert robot_ip('192.168.1.9', str(cfg)) == '192.168.1.9'      # parâmetro vence
    assert robot_ip('', str(cfg)) == '10.0.0.7'                    # robot.json do touch_pack
    assert robot_ip('  ', str(tmp_path / 'nao_existe.json')) == '192.168.5.2'
    (tmp_path / 'ruim.json').write_text('{ nao e json')
    assert robot_ip('', str(tmp_path / 'ruim.json')) == '192.168.5.2'


class Falso:
    estop_engaged = False

    def __init__(self, falha_em=None):
        self.falha_em, self.log = falha_em, []
        self.q = np.zeros(6)

    def connect(self):
        self.log.append('connect')
        if self.falha_em == 'connect':
            raise RuntimeError('sem rota')

    def enable(self):
        self.log.append('enable')
        if self.falha_em == 'enable':
            raise RuntimeError('modo LOCAL')

    def close(self): self.log.append('close')
    def prepare_servoj(self): self.log.append('prepare')

    def servo_j_urdf(self, q):
        if self.falha_em == 'servo':
            raise RuntimeError('socket caiu')
        self.q = np.array(q)
        self.log.append('servo')

    def read_joints_urdf_latest(self):
        if self.falha_em == 'read':
            raise RuntimeError('feedback fechado')
        return self.q

    def stop_motion(self): self.log.append('stop')


def link(drv, **kw):
    return RealLink('1.2.3.4', factory=lambda ip, per: drv, **kw)


def test_conecta_e_habilita():
    d = Falso(); l = link(d)
    assert l.connect_once() and l.connected
    assert d.log == ['connect', 'enable']


@pytest.mark.parametrize('onde', ['connect', 'enable'])
def test_falha_ao_conectar_fecha_o_que_abriu_e_aumenta_o_backoff(onde):
    # connect() pode abrir sockets antes de enable() falhar: sem close() sobram
    # sessões do dashboard até esgotá-las.
    d = Falso(onde); l = link(d)
    b0 = l.backoff_s()
    assert not l.connect_once()
    assert not l.connected and 'close' in d.log
    assert l.backoff_s() > b0


def test_backoff_satura():
    l = link(Falso('connect'))
    for _ in range(10):
        l.connect_once()
    assert l.backoff_s() == 30.0


def test_servoj_respeita_a_banda_morta():
    d = Falso(); l = link(d); l.connect_once()
    q = np.zeros(6)
    assert l.stream(q) is True
    assert l.stream(q + DEADBAND_RAD / 10) is False       # estacionário: nada de ServoJ redundante
    assert l.stream(q + 0.01) is True
    assert d.log.count('servo') == 2


def test_reset_stream_reenvia_o_mesmo_ponto():
    d = Falso(); l = link(d); l.connect_once()
    l.stream(np.zeros(6)); l.reset_stream()
    assert l.stream(np.zeros(6)) is True


def test_falha_de_servoj_derruba_a_conexao():
    d = Falso('servo'); l = link(d); l.connect_once()
    assert l.stream(np.ones(6)) is False
    assert not l.connected and 'prepare' in d.log          # tentou recuperar antes de desistir


def test_falha_de_leitura_derruba_a_conexao():
    d = Falso('read'); l = link(d); l.connect_once()
    assert l.read_q() is None and not l.connected


def test_sem_conexao_nada_acontece():
    l = link(Falso())
    assert l.read_q() is None and l.stream(np.ones(6)) is False
    l.halt()                                              # não explode


def test_halt_manda_parar_sem_desabilitar():
    d = Falso(); l = link(d); l.connect_once()
    l.halt()
    assert 'stop' in d.log and l.connected


def test_estop_do_driver_aparece_no_link():
    d = Falso(); l = link(d); l.connect_once()
    assert not l.estop
    d.estop_engaged = True
    assert l.estop


def test_robo_simulado_segue_o_servoj():
    d = SimulatedDriver([0, 0, 0, 0, 0, 0]); l = link(d); l.connect_once()
    l.stream(np.full(6, 0.2))
    assert np.allclose(l.read_q(), 0.2)
    l.halt()
    assert d.stopped == 1


def test_tolerancia_depende_de_estar_movendo():
    l = link(Falso())
    assert l.tolerance(False) == SYNC_TOL_START_RAD
    assert l.tolerance(True) == SYNC_TOL_RUN_RAD


def test_assinante_de_forca_aceita_o_qos_do_ft_receiver_do_touch_pack():
    """Regressão: com QoS confiável a FA7155 real não entregava nada."""
    pytest.importorskip('rclpy')
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy, qos_check_compatible,
                           QoSCompatibility)
    from rehab_pack.force_source import WRENCH_QOS
    ft_receiver = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.VOLATILE,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
    reliavel = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)
    for pub in (ft_receiver, reliavel):        # FA7155 real e (sim | mão virtual)
        comp, _ = qos_check_compatible(pub, WRENCH_QOS)
        assert comp != QoSCompatibility.ERROR


def test_erro_de_servoj_no_meio_do_movimento_nao_limpa_alarme():
    d = Falso(); l = link(d); l.connect_once()
    l.stream(np.zeros(6))
    d.falha_em = 'servo'
    assert l.stream(np.ones(6)) is False
    assert 'prepare' not in d.log and not l.connected


def test_queda_de_driver_antigo_nao_derruba_a_conexao_nova():
    velho, novo = Falso(), Falso()
    l = link(novo); l.connect_once()
    l.drop('erro atrasado', only=velho)
    assert l.connected


def test_salvar_ip_preserva_o_resto_e_nao_apaga_arquivo_ruim(tmp_path):
    f = tmp_path / 'robot.json'
    f.write_text(json.dumps({'hand_ip': '1.1.1.1', 'robot_ip': 'a'}))
    save_robot_ip('10.0.0.9', str(f))
    assert json.loads(f.read_text()) == {'hand_ip': '1.1.1.1', 'robot_ip': '10.0.0.9'}
    f.write_text('{ ruim')
    with pytest.raises(ValueError):
        save_robot_ip('b', str(f))
    assert f.read_text() == '{ ruim'
