"""Regressões da revisão cruzada de 24/11/2026 (XR-CRUZ-n no registro)."""
import math
import threading
import time
import types

import pytest

pytest.importorskip('rclpy')

import touch_pack.gui_robot as gr                                 # noqa: E402
import touch_pack.mirror_node as mn                               # noqa: E402
import touch_pack.real_driver as rd                               # noqa: E402
from std_msgs.msg import Empty                                    # noqa: E402
from std_srvs.srv import Trigger                                  # noqa: E402
from touch_pack.real_driver import (                              # noqa: E402
    CR10FeedbackTimeout, CR10RealDriverError)
from test_driver_confirmacao import _controlador, _ok             # noqa: E402
from test_espelho_gui_seguranca import (                          # noqa: E402
    HOME, _Drv, _Gui, _Mirror)


def _espera(cond, s=5.0):
    fim = time.monotonic() + s
    while time.monotonic() < fim:
        if cond():
            return True
        time.sleep(0.005)
    return False


# ══ XR-CRUZ-1 — com o E-STOP travado as consultas seguem respondendo ══

def test_robot_mode_responde_com_estop_travado():
    """'' aqui derrubava o heartbeat, que reconectava em laço."""
    drv = _controlador(lambda c: [(0.08, f'0,{{4}},{c};')])
    assert drv.emergency_stop() is True
    for _ in range(3):
        assert '{4}' in drv.robot_mode()


def test_rearme_completa_com_controlador_lento(monkeypatch):
    monkeypatch.setattr(rd.time, 'sleep', lambda s: None)
    # Timer e não sleep: 60 ms de resposta em todo comando.
    drv = _controlador(lambda c: [(
        0.06, f'0,{{{5 if "Mode" in c else ""}}},{c};')])
    drv._feed = object()
    assert drv.emergency_stop() is True
    drv.release_emergency_stop()
    assert drv.estop_engaged is False


def test_estop_fura_ate_o_rearme_mudo():
    """Nem um EmergencyStop(0) sem resposta segura o EmergencyStop(1)."""
    drv = _controlador(lambda c: _ok(c) if '(1)' in c else [],
                       cmd_timeout_s=3.0)
    drv._estop_engaged = True
    t = threading.Thread(target=lambda: drv._send_dash('EmergencyStop(0)'))
    t.start()
    assert _espera(lambda: drv.vistos)
    t0 = time.monotonic()
    assert drv.emergency_stop() is True
    assert time.monotonic() - t0 < 1.0
    t.join(4)


# ══ XR-CRUZ-2 — uma resposta perdida não envenena o comando ═══════════

def _perde_a_primeira(prefixo):
    vistos = []

    def resp(c):
        if c.startswith(prefixo):
            vistos.append(c)
            if len(vistos) == 1:
                return []
        return _ok(c)
    return resp


def test_movj_perdido_nao_envenena_os_seguintes():
    drv = _controlador(_perde_a_primeira('MovJ'), cmd_timeout_s=0.1)
    with pytest.raises(CR10RealDriverError):
        drv.mov_j_joint_deg([0] * 6)
    for k in range(1, 5):
        drv.mov_j_joint_deg([k] * 6)
    assert len(drv.vistos) == 5


def test_movj_identico_volta_a_confirmar_quando_a_divida_vence(monkeypatch):
    monkeypatch.setattr(rd, '_OWED_S', 0.3)
    drv = _controlador(_perde_a_primeira('MovJ'), cmd_timeout_s=0.1)
    with pytest.raises(CR10RealDriverError):
        drv.mov_j_joint_deg([0] * 6)
    with pytest.raises(CR10RealDriverError):
        drv.mov_j_joint_deg([0] * 6)      # ambíguo: não se adivinha
    time.sleep(0.3)
    drv.mov_j_joint_deg([0] * 6)


def test_estop_e_rearme_nunca_ficam_envenenados():
    for perdido in ('EmergencyStop(1)', 'EmergencyStop(0)'):
        drv = _controlador(_perde_a_primeira(perdido), cmd_timeout_s=0.1)
        drv._send_dash(perdido)
        assert drv.emergency_stop() is True
        for _ in range(3):
            assert drv._send_dash('EmergencyStop(0)').startswith('0')
        assert drv.emergency_stop() is True


@pytest.mark.parametrize('a,b', [
    ('EmergencyStop(0)', 'EmergencyStop(1)'),
    ('EmergencyStop(1)', 'EmergencyStop(0)'),
    ('MovJ(joint={1})', 'MovJ(joint={2})')])
def test_atrasada_de_verdade_nao_confirma_outro_comando(a, b):
    drv = _controlador(lambda c: [(0.12, f'0,{{}},{c};')] if c == a else [],
                       cmd_timeout_s=0.05)
    assert drv._send_dash(a) == ''
    drv.cfg.cmd_timeout_s = 0.3
    assert drv._send_dash(b) == ''


@pytest.mark.parametrize('b', ['EmergencyStop(1)', 'MovJ(joint={2})'])
def test_atrasada_descartada_e_a_verdadeira_aceita(b):
    a = {'E': 'EmergencyStop(0)', 'M': 'MovJ(joint={1})'}[b[0]]
    drv = _controlador(lambda c: [(0.12 if c == a else 0.1,
                                   f'{0 if c == a else -2},{{}},{c};')],
                       cmd_timeout_s=0.05)
    drv._send_dash(a)
    drv.cfg.cmd_timeout_s = 0.5
    assert drv._send_dash(b).startswith('-2')


# ══ XR-CRUZ-8 — baixos do driver ═══════════════════════════════════

def test_consulta_muda_nao_segura_o_servoj():
    drv = _controlador(lambda c: _ok(c) if c.startswith('ServoJ') else [])
    drv.servo_j([0.0] * 6)
    t0 = time.monotonic()
    drv._send_dash('GetErrorID()')
    assert time.monotonic() - t0 < 0.6          # era 1 s com o lock na mão


def test_ack_de_erro_drenado_aparece_no_proximo_servoj():
    drv = _controlador(
        lambda c: [(0.03, f'-2,{{}},{c};')] if c.startswith('ServoJ')
        else _ok(c))
    drv.servo_j([0.0] * 6)                      # ACK ainda em voo
    drv._send_dash('RobotMode()')               # a drenagem o consome
    n = len(drv.vistos)
    with pytest.raises(CR10RealDriverError, match='ServoJ'):
        drv.servo_j([0.0] * 6)
    assert len(drv.vistos) == n
    drv._send_dash('ClearError()')
    drv.servo_j([0.0] * 6)


@pytest.mark.parametrize('cmd', [
    lambda d: d.servo_j([0.0] * 6), lambda d: d.mov_j_joint_deg([0] * 6),
    lambda d: d.mov_l_joint_deg([0] * 6)])
def test_estop_pedido_na_espera_do_lock_barra_o_movimento(cmd):
    drv = _controlador(_ok)
    erros = []

    def manda():
        try:
            cmd(drv)
        except CR10RealDriverError as exc:
            erros.append(str(exc))
    drv._dash_lock.acquire()
    t = threading.Thread(target=manda)
    t.start()
    time.sleep(0.1)                             # já passou da 1ª conferência
    drv._estop_engaged = True
    drv._dash_lock.release()
    t.join(2)
    assert erros and 'E-STOP' in erros[0] and drv.vistos == []


def test_estop_durante_o_rearme_vence(monkeypatch):
    monkeypatch.setattr(rd.time, 'sleep', lambda s: None)
    feito = threading.Event()

    def resp(c):
        if c == 'EnableRobot()' and not feito.is_set():
            feito.set()
            threading.Thread(target=drv.emergency_stop).start()
        return _ok(c, 5 if 'Mode' in c else '')
    drv = _controlador(resp)
    drv._feed = object()
    drv._estop_engaged = True
    with pytest.raises(CR10RealDriverError):
        drv.release_emergency_stop()
    assert drv.estop_engaged is True
    assert _espera(lambda: 'EmergencyStop(1)' in drv.vistos)


# ══ XR-CRUZ-4 — pausa/salto do simulador não vira salto de ServoJ ═════

def test_mirror_para_com_joint_states_velho():
    drv = _Drv()
    m = _Mirror(drv)
    m._latest_q_ts = time.monotonic() - 1.0
    m.roda()
    assert drv.enviados == [] and 'joint_states' in m._hold
    assert drv.estop_engaged and m._freeze_pub.msgs


def test_mirror_recusa_salto():
    drv = _Drv()
    m = _Mirror(drv)
    m._last_servoj_q = HOME.copy()
    m._latest_q = list(HOME + 0.5)
    m._latest_q_ts = math.inf                   # idade fora de questão
    m.roda()
    assert drv.enviados == [] and 'salto' in m._hold


class _Tempo:
    def __init__(self):
        self.t = 100.0

    def monotonic(self):
        return self.t

    time = monotonic


@pytest.fixture()
def tempo(monkeypatch):
    t = _Tempo()
    monkeypatch.setattr(gr, 'time', t)
    return t


def _anda(g, drv, tempo, q, dt):
    tempo.t += dt
    g._latest_joint_ts = tempo.t
    g._servo_tick(drv, list(q))


@pytest.mark.parametrize('caso', ['pausa', 'salto'])
def test_gui_pausa_e_salto_travam_ate_o_operador(tempo, caso):
    drv = _Drv()
    g = _Gui(drv, phase='HOLD')
    _anda(g, drv, tempo, HOME, 0.03)
    n = len(drv.enviados)
    if caso == 'pausa':
        tempo.t += 1.0
        g._servo_tick(drv, list(HOME + 0.01))
    else:
        _anda(g, drv, tempo, HOME + 0.5, 0.03)
    assert len(drv.enviados) == n and g._mirror_hold
    g.root.roda()
    assert g._estop_latched


@pytest.mark.parametrize('rad_s', [0.5, 2.0, 3.0])
def test_gui_velocidade_legitima_passa(tempo, rad_s):
    """Até a velocidade nominal das juntas, com jitter de laço (90 ms)."""
    drv = _Drv()
    g = _Gui(drv, phase='HOME')
    q = HOME.copy()
    for k in range(60):
        dt = 0.09 if k % 7 == 0 else 0.03
        q = q + rad_s * dt * (1 if k % 20 < 10 else -1)
        if k == 0:
            drv.q = q.copy()
        _anda(g, drv, tempo, q, dt)
    assert not getattr(g, '_mirror_hold', False)
    assert len(drv.enviados) == 60


# ══ XR-CRUZ-5 — follow_acc salvo ══════════════════════════════════════

def test_follow_acc_salvo_vale_sem_tocar_na_caixa(gui_real):
    import tkinter as tk
    pai = tk.Frame(gui_real.root)
    try:
        gui_real._build_imu_tab(pai)
        assert gui_real._imu_acc_sid_cache == 2
    finally:
        gui_real._imu_shutdown()
        pai.destroy()


# ══ XR-CRUZ-6 — freeze durante connect/enable ═════════════════════════

@pytest.mark.parametrize('quando', ['connect', 'enable'])
def test_freeze_durante_a_conexao_nao_se_perde(quando):
    drv = _Drv()
    m = _Mirror(None)
    m._connected = False
    setattr(drv, quando, lambda: m._cb_freeze(Empty()))
    if quando != 'connect':
        drv.connect = lambda: None
    m._new_driver = lambda ip: drv
    m._robot_ip = lambda: '127.0.0.1'
    t = threading.Thread(target=m._connect_loop)
    t.start()
    assert _espera(lambda: m._driver is drv)
    m._stop.set()
    t.join(3)
    assert m._hold and drv.estop_engaged
    if quando == 'connect':
        assert ('enable',) not in drv.enviados


# ══ XR-CRUZ-7 — freeze durante o rearme ═══════════════════════════════

def test_freeze_no_meio_do_rearme_vence():
    drv = _Drv()
    m = _Mirror(drv)
    m._cb_freeze(Empty())

    def solta():
        drv.estop_engaged = False
        m._cb_freeze(Empty())
    drv.release_emergency_stop = solta
    r = m._srv_rearm(None, types.SimpleNamespace(success=None, message=''))
    assert r.success is False and m._hold and drv.estop_engaged
    # E o próximo rearme, sem freeze, funciona.
    drv.release_emergency_stop = lambda: setattr(drv, 'estop_engaged', False)
    r = m._srv_rearm(None, types.SimpleNamespace(success=None, message=''))
    assert r.success is True and not m._hold


def test_freeze_e_atendido_com_o_rearme_bloqueado(monkeypatch):
    """Pelo executor de `main()`: o rearme preso não segura o freeze."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from touch_pack import pack_lock
    meu = not rclpy.ok()
    if meu:
        rclpy.init()
    drv, solto, dentro = _Drv(), threading.Event(), threading.Event()
    parar, nos, t, cli = threading.Event(), [], None, None
    classe = mn.MirrorNode

    def spin(node, executor=None):      # o de produção, com saída
        ex = executor or SingleThreadedExecutor()
        ex.add_node(node)
        while not parar.is_set():
            ex.spin_once(timeout_sec=0.05)
        ex.shutdown()
    with monkeypatch.context() as mp:
        mp.setattr(pack_lock, 'acquire_or_exit', lambda *_a: None)
        mp.setattr(classe, '_connect_loop', lambda s: None)
        mp.setattr(classe, '_servoj_loop', lambda s: None)
        mp.setattr(mn, 'MirrorNode',
                   lambda: nos.append(classe()) or nos[0])
        mp.setattr(mn, 'rclpy', types.SimpleNamespace(
            init=lambda **k: None, spin=spin, ok=lambda: False))
        try:
            t = threading.Thread(target=mn.main)
            t.start()
            assert _espera(lambda: nos)
            no = nos[0]
            no._driver, no._connected = drv, True
            no._latest_q = list(HOME)
            no._engage_hold('teste')

            def preso():
                drv.estop_engaged = False
                dentro.set()
                solto.wait(10)
            drv.release_emergency_stop = preso
            cli = rclpy.create_node('cli_rearme')
            pub = cli.create_publisher(Empty, '/palpation/freeze', 10)
            srv = cli.create_client(Trigger, '/mirror_node/rearm')
            assert srv.wait_for_service(10)
            assert _espera(lambda: pub.get_subscription_count() >= 1)
            srv.call_async(Trigger.Request())
            assert dentro.wait(10)
            pub.publish(Empty())
            assert _espera(lambda: drv.estop_engaged), \
                'o freeze só seria visto depois do rearme'
            solto.set()
            assert _espera(lambda: no._rearm_lock.acquire(blocking=False))
            assert no._hold
        finally:
            solto.set()
            parar.set()
            if t is not None:
                t.join(5)
            if cli is not None:
                cli.destroy_node()
    if meu:
        rclpy.shutdown()


# ══ XR-CRUZ-3 — HOME legítima com RTF baixo ═══════════════════════════

@pytest.fixture()
def viagem(braco, relogio, monkeypatch):
    """Braço que percorre a trajetória na fração `rtf` do relógio de parede."""
    import touch_pack.explorer_guards as g
    monkeypatch.setattr(g, 'time', relogio)
    braco._motion_guard = lambda *a: None
    parou = []
    braco._settle = lambda *a, **k: parou.append(1)
    q0 = braco.q_sim.copy()
    alvo = q0 + 0.6

    def roda(rtf, dur=8.0, para_em=1.0):
        t0 = relogio.t
        braco._q_now = lambda: q0 + (alvo - q0) * min(
            1.0, para_em, (relogio.t - t0) * rtf / dur)
        out = braco._wait_motion(alvo, dur, 'HOME')
        return out, relogio.t - t0, parou
    return roda


@pytest.mark.parametrize('rtf', [1.0, 0.85, 0.5, 0.15])
def test_home_com_rtf_baixo_chega(viagem, rtf):
    out, durou, parou = viagem(rtf)
    assert out == 'done' and not parou
    assert durou == pytest.approx(8.0 / rtf, rel=0.1)


def test_braco_parado_no_meio_ainda_aborta_logo(viagem):
    out, durou, parou = viagem(1.0, para_em=0.4)
    assert out == 'error' and parou and durou < 12.0


def test_braco_que_rasteja_tem_teto_de_parede(viagem):
    out, durou, parou = viagem(0.03)
    assert out == 'error' and parou and durou < 100.0


# ══ XR-CRUZ-9 — logger reiniciado no meio do run ══════════════════════

def test_logger_reiniciado_grava_o_resto_em_continuacao(tmp_path, monkeypatch):
    import rclpy
    from touch_pack import palpation_logger as PL
    from touch_pack_msgs.msg import PalpationStatus
    from test_logger_columns import _start_msg
    monkeypatch.setattr(PL, 'OUTPUT_DIR', str(tmp_path))
    monkeypatch.setattr(PL.PalpationLogger, '_generate_report',
                        lambda self, path: None)
    meu = not rclpy.ok()
    if meu:
        rclpy.init()
    try:
        start = _start_msg('TOUCH')
        start.run_id = '20261124_101112'

        def sessao(fase):
            lg = PL.PalpationLogger()
            try:
                lg._cb_start(start)
                primeira = lg._run_dir if lg._csv_fh else None
                lg._cb_status(PalpationStatus(
                    run_id=start.run_id, phase=fase))
                return primeira, lg._run_dir if lg._csv_fh else None
            finally:
                lg._csv_fh and lg._csv_fh.flush()
                lg.close()
                lg.destroy_node()
        original, _ = sessao('HOLD')
        assert original
        csv0 = next(tmp_path.rglob(PL.RUN_SAMPLES_CSV))
        csv0.write_text(csv0.read_text() + 'amostra\n')
        antes = csv0.read_bytes()
        nada, cont = sessao('HOLD')
        assert nada is None and cont.endswith('_cont1')
        assert sessao('DONE')[1] is None       # run acabado: nada a continuar
        assert csv0.read_bytes() == antes
    finally:
        if meu:
            rclpy.shutdown()


# ══ XR-CRUZ-10 — o Tk não para ════════════════════════════════════════

@pytest.mark.parametrize('ok', [True, False])
def test_estop_lento_nao_prende_o_tk(ok):
    drv = _Drv()
    libera = threading.Event()

    def lento():
        drv.estop_engaged = True
        libera.wait(5)
        return ok
    drv.emergency_stop = lento
    g = _Gui(drv)
    t0 = time.monotonic()
    g._estop_engage()
    assert time.monotonic() - t0 < 1.0
    assert g._estop_latched
    assert 'robot disabled and alarmed' not in g.status[-1]
    libera.set()
    assert _espera(lambda: g.root.fila)
    g.root.roda()
    assert ('robot disabled and alarmed' in g.status[-1]) is ok
    assert ('NOT alarmed' in g.status[-1]) is not ok


@pytest.mark.parametrize('lento', [True, False])
@pytest.mark.parametrize('erro', ['Stop() não confirmado', ''])
def test_stop_so_anuncia_parada_confirmada(lento, erro):
    drv = _Drv()
    libera = threading.Event()
    if not lento:
        libera.set()

    def para():
        libera.wait(5)
        if erro:
            raise CR10RealDriverError(erro)
    drv.stop_motion = para
    g = _Gui(drv)
    t0 = time.monotonic()
    g._on_stop_palpation()
    assert time.monotonic() - t0 < 1.0
    if lento:
        assert 'stopped by the operator' not in g.status[-1]
        libera.set()
        assert _espera(lambda: g.root.fila)
        g.root.roda()
    assert ('NOT CONFIRMED' in g.status[-1]) is bool(erro)
    assert ('stopped by the operator' in g.status[-1]) is not bool(erro)


# ══ XR-CRUZ-11 — um pacote de feedback perdido não é E-STOP ═══════════

def _mudo():
    raise CR10FeedbackTimeout('sem pacote')


def test_mirror_tolera_feedback_isolado_mas_nao_o_mudo():
    drv = _Drv()
    m = _Mirror(drv)
    falhas = [1, 1]

    def le():
        if falhas:
            falhas.pop()
            _mudo()
        return drv.q.copy()
    drv.read_joints_urdf_latest = le
    m.roda()
    assert not m._hold and [c[0] for c in drv.enviados] == ['ServoJ']

    drv = _Drv()
    drv.read_joints_urdf_latest = _mudo
    m = _Mirror(drv)
    m.roda(0.9)
    assert drv.enviados == [] and m._hold and drv.estop_engaged


def test_gui_tolera_feedback_isolado_mas_nao_o_mudo(tempo):
    drv = _Drv()
    g = _Gui(drv, phase='HOLD')
    bom, drv.read_joints_urdf_latest = drv.read_joints_urdf_latest, _mudo
    _anda(g, drv, tempo, HOME, 0.03)
    _anda(g, drv, tempo, HOME, 0.03)
    assert drv.enviados == [] and not getattr(g, '_mirror_hold', False)
    drv.read_joints_urdf_latest = bom
    _anda(g, drv, tempo, HOME, 0.03)
    assert len(drv.enviados) == 1
    # Novo início de streaming, agora mudo de vez.
    g._servo_synced = False
    drv.read_joints_urdf_latest = _mudo
    for _ in range(25):
        if not getattr(g, '_mirror_hold', False):
            _anda(g, drv, tempo, HOME, 0.03)
    assert g._mirror_hold and len(drv.enviados) == 1


# ══ XR-CRUZ-12 — rearme depois que a conexão caiu com o E-STOP travado ══

class _DrvMorre(_Drv):
    """Depois de `morre()`, nada mais passa: como o driver com _dash=None."""

    def morre(self):
        self.vivo = False

    def release_emergency_stop(self):
        if not self.vivo:
            raise CR10RealDriverError('Dashboard não conectado')
        self.ordem.append('release')
        super().release_emergency_stop()

    def enable(self):
        self.ordem.append('enable')

    def connect(self):
        self.ordem.append('connect')


def _rearma(m):
    return m._srv_rearm(None, types.SimpleNamespace(success=None, message=''))


@pytest.mark.parametrize('falha_antes', [False, True])
def test_rearme_recria_o_driver_morto_e_solta_o_estop(falha_antes):
    velho, novo = _DrvMorre(), _DrvMorre()
    m = _Mirror(velho)
    m._robot_ip = lambda: '127.0.0.1'
    criados = []
    m._new_driver = lambda ip: criados.append(1) or novo
    m._cb_freeze(Empty())
    assert velho.estop_engaged
    velho.morre()
    if falha_antes:                  # a rede ainda fora na 1ª tentativa
        bom, novo.connect = novo.connect, lambda: (_ for _ in ()).throw(
            OSError('sem rede'))
        assert _rearma(m).success is False and m._hold
        novo.connect = bom
        novo.ordem.clear()
    n = len(criados)
    time.sleep(0.05)
    assert len(criados) == n         # só por pedido explícito
    assert _rearma(m).success is True
    assert novo.ordem == ['connect', 'release']      # nunca um enable() cego
    assert 'close' in velho.ordem
    assert m._driver is novo and m._connected and not m._hold


@pytest.mark.parametrize('quando', ['connect', 'release_emergency_stop'])
def test_freeze_vence_o_rearme_com_driver_novo(quando):
    velho, novo = _DrvMorre(), _DrvMorre()
    m = _Mirror(velho)
    m._robot_ip = lambda: '127.0.0.1'
    m._new_driver = lambda ip: novo
    m._cb_freeze(Empty())
    velho.morre()
    bom = getattr(novo, quando)

    def no_meio():
        bom()
        m._cb_freeze(Empty())
    setattr(novo, quando, no_meio)
    assert _rearma(m).success is False
    assert m._hold and novo.estop_engaged
    if quando == 'connect':
        assert 'release' not in novo.ordem and 'enable' not in novo.ordem
    # E o pedido seguinte, sem freeze, solta o E-STOP e libera.
    setattr(novo, quando, bom)
    novo.vivo = True
    novo.ordem.clear()
    assert _rearma(m).success is True
    assert 'release' in novo.ordem and 'enable' not in novo.ordem


# ══ XR-CRUZ-13 — a tolerância de feedback não atravessa fases ═════════

@pytest.mark.parametrize('fora', ['IDLE', 'FROZEN'])
def test_mirror_zera_a_tolerancia_no_fim_da_fase(fora):
    drv = _Drv()
    drv.read_joints_urdf_latest = _mudo
    m = _Mirror(drv)
    m._sync_fail_t = time.monotonic() - 60.0      # resto da fase anterior
    m._phase = fora
    m.roda(0.03)
    m._phase = 'HOLD'
    m.roda(0.03)                                  # 1 pacote perdido: tolera
    assert not m._hold and drv.enviados == []


@pytest.mark.parametrize('fora', ['IDLE', 'FROZEN'])
def test_gui_zera_a_tolerancia_no_fim_da_fase(fora):
    drv = _Drv()
    drv.read_joints_urdf_latest = _mudo
    g = _Gui(drv, phase=fora)
    g._servoj_period_s = 0.002
    g._mirror_follow_until = 0.0
    g._sync_fail_t = time.monotonic() - 60.0
    t = threading.Thread(target=g._mirror_poll_loop)
    t.start()
    time.sleep(0.1)
    with g._lock:
        g._latest_phase = 'HOLD'
    time.sleep(0.1)
    g._stop_event.set()
    t.join(2)
    assert not g._mirror_hold and drv.enviados == []
