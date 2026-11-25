"""Regressões da revisão cruzada de 25/11/2026 (XR-CRUZ-Rn no registro). Precisa do ROS."""
import json
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('xacro')

from std_msgs.msg import Bool, String                       # noqa: E402
from std_srvs.srv import Trigger                            # noqa: E402
from rehab_pack import force_source, real_link              # noqa: E402
from rehab_pack.constants import JOG_MAX_DEG_S, JS_STALE_S   # noqa: E402
from rehab_pack.real_link import RealLink, SimulatedDriver, real_ok     # noqa: E402
from test_auditoria_controle import (OK, amostra, arma, chama, estado, faz_tara,   # noqa: E402,F401
                                     fonte, fs, mirror, segura, sim, wait)


# ── XR-CRUZ-R1: Stop não confirmado ─────────────────────────────────────────
class SemStop(SimulatedDriver):
    confirma = True

    def __init__(self, *a):
        super().__init__(*a)
        self.log = []

    def stop_motion(self):
        self.log.append('stop')
        raise RuntimeError('Stop() não confirmado')

    def emergency_stop(self):
        self.log.append('estop')
        self.estop_engaged = True
        if self.confirma is None:
            raise OSError('socket')
        return self.confirma

    def enable(self):
        self.log.append('enable')

    def release_emergency_stop(self):
        self.log.append('release')


@pytest.mark.parametrize('confirma', [True, False, None])
def test_r1_stop_nao_confirmado_vira_estop_e_diz_a_verdade(confirma):
    d = SemStop(np.zeros(6))
    d.confirma = confirma
    link = RealLink('1.2.3.4', factory=lambda *_: d)
    link.connect_once()
    assert link.halt() is bool(confirma)
    assert d.log == ['enable', 'stop', 'estop']
    ok, why = real_ok({**OK, 'parada': link.parada}, 0.0)
    assert not ok and ('PARADA NÃO CONFIRMADA' in why) == (not confirma)
    assert ('E-STOP' in why) == bool(confirma)
    link.drop('cabo')                    # caiu a conexão: o aviso continua, e vem primeiro
    assert real_ok({**OK, 'conectado': False, 'parada': link.parada}, 0.0)[1] == why


def test_r1_stop_confirmado_nao_aciona_o_estop():
    d = SimulatedDriver(np.zeros(6))
    link = RealLink('sim', factory=lambda *_: d)
    link.connect_once()
    assert link.halt() is True and not link.estop and not link.parada


@pytest.mark.parametrize('como', ['aborto', 'parar'])
def test_r1_espelho_publica_a_parada_e_so_o_conectar_rearma(mirror, monkeypatch, como):    # noqa: F811
    d = SemStop()
    d.confirma = False
    monkeypatch.setattr(real_link, 'default_factory', lambda *_: d)
    monkeypatch.setattr(real_link.RealLink.__init__, '__defaults__',
                        (0.030, lambda *_: d, None))
    drv, q0 = arma(mirror)
    vistos = []
    mirror.pub.publish = lambda m: vistos.append(json.loads(m.data))
    if como == 'aborto':
        mirror.state = 'ABORTADO'
    else:
        mirror._on_event(String(data='ESTOP'))
        mirror.state = 'ABORTADO'
    assert wait(lambda: vistos and 'PARADA NÃO CONFIRMADA' in vistos[-1]['parada'])
    assert 'estop' in d.log
    n0 = len(d.cmds)
    mirror._on_event(String(data='RESET'))         # Reset não apaga nem rearma
    mirror.state = 'ATIVO'
    segura(mirror, q0, 0.2)
    assert vistos[-1]['parada'] and not real_ok(vistos[-1], 0.0)[0] and len(d.cmds) == n0
    mirror.state = 'PARADO'
    assert chama(mirror._srv_connect, False).success
    d.estop_engaged = False
    assert chama(mirror._srv_connect).success      # pedido explícito do operador
    assert wait(lambda: mirror.link.connected)
    assert d.log[-1] == 'release' and not mirror.link.parada and not mirror.link.rearmar


# ── XR-CRUZ-R2: teto do passo acompanha o atraso do laço, com limite ────────
@pytest.mark.parametrize('buraco,graus,passa', [
    (0.030, 1.40, True), (0.030, 1.60, False),     # XC9: no período normal nada mudou
    (0.070, 24 * 0.070, True),                     # laço atrasou em jog de 100 %
    (0.100, 2.40, True), (0.100, 3.20, False),
    (30.0, 3.00, True), (30.0, 3.20, False),       # teto absoluto de dt
])
def test_r2_passo_maximo_cresce_com_o_atraso_ate_o_teto(monkeypatch, buraco, graus, passa):
    t = [100.0]
    monkeypatch.setattr(real_link.time, 'monotonic', lambda: t[0])
    d = SimulatedDriver(np.zeros(6))
    link = RealLink('sim', 0.030)
    link.attach(d)
    assert link.stream(np.zeros(6), np.zeros(6))
    t[0] += buraco
    q = np.radians([graus, 0, 0, 0, 0, 0])
    if passa:
        assert link.stream(q)
        assert np.degrees(d.q[0]) == pytest.approx(min(graus, 1.44))
    else:
        with pytest.raises(real_link.StepTooLarge):
            link.stream(q)
        assert len(d.cmds) == 1 and np.allclose(d.q, 0.0)


@pytest.mark.parametrize('dt0', [0.050, 0.070, 0.100])
def test_r6_buraco_em_jog_maximo_e_recuperado_em_pontos_limitados(monkeypatch, dt0):
    t = [100.0]
    monkeypatch.setattr(real_link.time, 'monotonic', lambda: t[0])
    d = SimulatedDriver(np.zeros(6))
    link = RealLink('sim', 0.030)
    link.attach(d)
    dirz = np.array([1, -0.5, 0.25, 0, 0, 1])      # recorte proporcional: a direção não muda
    q = np.zeros(6)
    link.stream(q, q)
    for dt in (dt0, *[0.030] * 8):
        t[0] += dt
        q = q + np.radians(24.0 * dt) * dirz
        assert link.stream(q)
    cmds = np.array(d.cmds)
    assert np.degrees(np.abs(np.diff(cmds, axis=0)).max()) <= 1.44 + 1e-6
    assert np.allclose(cmds[1:] / cmds[1:, :1], dirz) and np.allclose(cmds[-1], q)
    assert not link.stream(q + 1e-6)               # banda morta


@pytest.mark.parametrize('v', [24.0 * 1.9, 60.0])
def test_r6_alvo_que_segue_longe_trava(monkeypatch, v):
    """Buraco e, depois dele, alvo fugindo: recorta por pouco tempo e trava (XC9)."""
    t = [100.0]
    monkeypatch.setattr(real_link.time, 'monotonic', lambda: t[0])
    d = SimulatedDriver(np.zeros(6))
    link = RealLink('sim', 0.030)
    link.attach(d)
    link.stream(np.zeros(6), np.zeros(6))
    q, n = 3.10, 0
    t[0] += 0.100
    with pytest.raises(real_link.StepTooLarge):
        for n in range(10):
            link.stream(np.radians([q, 0, 0, 0, 0, 0]))
            t[0] += 0.030
            q += v * 0.030
    assert n <= 3
    assert np.degrees(np.abs(np.diff(np.array(d.cmds), axis=0)).max()) <= 1.44 + 1e-6


def test_r6_buracos_seguidos_nao_acumulam_folga(monkeypatch):
    t = [100.0]
    monkeypatch.setattr(real_link.time, 'monotonic', lambda: t[0])
    link = RealLink('sim', 0.030)
    link.attach(SimulatedDriver(np.zeros(6)))
    link.stream(np.zeros(6), np.zeros(6))
    for _ in range(20):                            # laço lento com o alvo parado
        t[0] += 0.100
        link.stream(np.zeros(6))
    t[0] += 0.100
    with pytest.raises(real_link.StepTooLarge):
        link.stream(np.radians([3.2, 0, 0, 0, 0, 0]))


def test_r6_reset_do_stream_zera_a_folga(monkeypatch):
    t = [100.0]
    monkeypatch.setattr(real_link.time, 'monotonic', lambda: t[0])
    link = RealLink('sim', 0.030)
    link.attach(SimulatedDriver(np.zeros(6)))
    link.stream(np.zeros(6), np.zeros(6))
    t[0] += 0.100
    link.stream(np.radians([1.0, 0, 0, 0, 0, 0]))
    link.reset_stream()
    with pytest.raises(real_link.StepTooLarge):
        link.stream(np.radians([2.0, 0, 0, 0, 0, 0]), np.zeros(6))


def test_r6_o_laco_nao_junta_dois_pontos_num_periodo(mirror):    # noqa: F811
    drv, q0 = arma(mirror)
    ts, manda, k = [], drv.servo_j_urdf, [0]

    def lento(q):
        ts.append(time.monotonic())
        manda(q)
        if len(ts) % 5 == 0:
            time.sleep(0.05)                       # um ponto demora: o próximo não encurta
    drv.servo_j_urdf = lento
    for k in range(200):
        sim(mirror, q0 + np.radians([0.02 * k, 0, 0, 0, 0, 0]))
        time.sleep(0.005)
    assert len(ts) > 20 and np.diff(ts).min() > 0.029


# ── XR-CRUZ-R7: a parada não segura o lock do espelho ───────────────────────
@pytest.mark.parametrize('como', ['ESTOP', 'ABORTADO', 'salto'])
def test_r7_parada_lenta_nao_bloqueia_status_parar_nem_servicos(mirror, como):    # noqa: F811
    drv, q0 = arma(mirror)
    solta, entrou = threading.Event(), threading.Event()

    def lenta():
        drv.stopped += 1
        entrou.set()
        solta.wait(10.0)
        raise RuntimeError('Stop() não confirmado')
    drv.stop_motion, drv.emergency_stop = lenta, lambda: False
    vistos = []
    mirror.pub.publish = lambda m: vistos.append(json.loads(m.data))
    try:
        if como == 'ESTOP':
            mirror._on_event(String(data='ESTOP'))
            assert mirror.halted
        elif como == 'ABORTADO':
            mirror._on_state(String(data='ABORTADO'))
        else:
            segura(mirror, q0 + np.radians([7, 0, 0, 0, 0, 0]), 0.1)
        assert entrou.wait(2.0)
        n, n_cmd = len(vistos), len(drv.cmds)
        t0 = time.monotonic()
        mirror._on_event(String(data='ESTOP'))     # PARAR de novo: na hora, sem outra parada
        mirror._on_event(String(data='RESET'))
        mirror._on_state(String(data='ATIVO'))     # nem assim sai ServoJ
        assert time.monotonic() - t0 < 0.2
        segura(mirror, q0, 0.4)
        assert len(vistos) - n > 5 and not real_ok(vistos[-1], 0.0)[0]
        assert vistos[-1]['travado'] == 'parada do robô em curso'
        assert len(drv.cmds) == n_cmd and drv.stopped == 1
        le, drv.read_joints_urdf_latest = drv.read_joints_urdf_latest, lambda: 1 / 0
        time.sleep(0.15)                           # feedback caiu: o E-STOP ainda usa a conexão
        assert mirror.link.drv is drv
        drv.read_joints_urdf_latest = le
        mirror._on_state(String(data='PARADO'))
        t0 = time.monotonic()
        assert chama(mirror._srv_connect, False).success
        assert not chama(mirror._srv_connect).success
        assert time.monotonic() - t0 < 0.2 and mirror.link.drv is drv
    finally:
        solta.set()
    assert wait(lambda: mirror.parando is None and not mirror.link.connected)
    assert wait(lambda: 'PARADA NÃO CONFIRMADA' in vistos[-1]['parada'])


def test_r7_encerrar_espera_a_parada(mirror):    # noqa: F811
    drv, q0 = arma(mirror)
    ordem = []
    drv.stop_motion = lambda: (time.sleep(0.3), ordem.append('stop'))
    drv.close = lambda: ordem.append('close')
    mirror._on_event(String(data='ESTOP'))
    mirror.destroy_node()
    assert ordem == ['stop', 'close']


def test_r2_parado_na_banda_morta_nao_acumula_tempo(monkeypatch):
    t = [100.0]
    monkeypatch.setattr(real_link.time, 'monotonic', lambda: t[0])
    link = RealLink('sim', 0.030)
    link.attach(SimulatedDriver(np.zeros(6)))
    link.stream(np.zeros(6), np.zeros(6))
    for _ in range(100):
        t[0] += 0.030
        assert not link.stream(np.zeros(6))
    t[0] += 0.030
    with pytest.raises(real_link.StepTooLarge):
        link.stream(np.radians([2.0, 0, 0, 0, 0, 0]))


def test_r2_primeiro_ponto_nao_ganha_atraso():
    link = RealLink('sim', 0.030)
    link.attach(SimulatedDriver(np.zeros(6)))
    with pytest.raises(real_link.StepTooLarge):
        link.stream(np.radians([2.0, 0, 0, 0, 0, 0]), np.zeros(6))
    assert JOG_MAX_DEG_S * JS_STALE_S == pytest.approx(2.4)


# ── XR-CRUZ-R3: só o Reset solta a trava ────────────────────────────────────
def test_r3_parado_republicado_nao_solta_a_trava(mirror):    # noqa: F811
    arma(mirror)
    mirror._on_event(String(data='ESTOP'))
    for s in ('ATIVO', 'PARADO', 'PARADO', 'ABORTADO', 'PARADO'):
        mirror._on_state(String(data=s))
        assert mirror.halted
    mirror._on_event(String(data='MARK;origem=RESET'))
    assert mirror.halted
    mirror._on_event(String(data='RESET'))
    assert not mirror.halted


@pytest.mark.parametrize('antes', ['ABORTADO', 'PARADO', 'ATIVO'])
def test_r3_o_reset_da_admitancia_anuncia_o_evento(antes):
    from rehab_pack import admittance_node as an
    n = object.__new__(an.AdmittanceNode)
    ev = []
    n.state = antes
    n.pub_event = type('P', (), {'publish': staticmethod(lambda m: ev.append(m.data))})
    n._set_state = lambda s: setattr(n, 'state', s)
    r = n._srv_reset(None, Trigger.Response())
    assert r.success == (antes != 'ATIVO') and ev == (['RESET'] if r.success else [])


# ── XR-CRUZ-R4: zero de hardware da FA7155 ──────────────────────────────────
def test_r4_zero_de_hardware_invalida_a_tara_pelo_topico(fs):    # noqa: F811
    estado(fs, 'PARADO')
    outro = rclpy.create_node('ft_receiver')
    pub = outro.create_publisher(Bool, '/load_cell/tared', 10)
    assert wait(lambda: pub.get_subscription_count() == 1)

    def gira():
        amostra(fs)
        rclpy.spin_once(fs, timeout_sec=0.02)
        return bool(fs.taras)
    pub.publish(Bool(data=True))
    assert not wait(gira, 0.2)
    assert faz_tara(fs).success
    fs.taras.clear()
    pub.publish(Bool(data=True))                   # o anúncio de 1 Hz não invalida
    assert not wait(gira, 0.3)
    pub.publish(Bool(data=False))
    assert wait(gira) and fs.taras == [0.0]
    assert faz_tara(fs).success                    # tarou antes do auto-tare do ft_receiver
    fs.taras.clear()
    pub.publish(Bool(data=True))
    assert wait(gira) and fs.taras == [0.0]
    outro.destroy_node()


def test_r4_zero_no_meio_da_janela_descarta(fs):    # noqa: F811
    estado(fs, 'PARADO')
    r = faz_tara(fs, lambda: fs._on_ft_tared(Bool(data=False)))
    assert not r.success and np.allclose(fs._tare.offset, 0.0) and fs.taras[-1] == 0.0


# ── XR-CRUZ-R5: janela de tara cancelada e reaberta ─────────────────────────
def test_r5_a_tara_cancelada_nao_fecha_a_seguinte(fs):    # noqa: F811
    estado(fs, 'PARADO')
    segunda = Trigger.Response()
    t = threading.Thread(target=lambda: fs._on_tare(None, segunda))

    def cancela_e_reabre():
        fs._invalidate('teste')
        t.start()
    primeira = faz_tara(fs, cancela_e_reabre)
    while t.is_alive():
        amostra(fs)
        t.join(0.01)
    assert not primeira.success and segunda.success
    assert fs.taras[-1] > 0.0 and np.allclose(fs._tare.offset[0], 2.5)


def test_r5_janela_velha_nao_fecha_a_nova():
    t = force_source.Tare()
    velha = t.begin()
    t.cancel()
    nova = t.begin()
    assert t.finish(velha) and t.running
    for _ in range(20):
        t.feed(np.ones(6))
    assert t.finish(nova) == '' and np.allclose(t.offset, 1.0)
