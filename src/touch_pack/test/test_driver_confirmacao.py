"""Confirmação, associação e enquadramento do CR10RealDriver (auditoria 24/11/2026).

Tudo contra um controlador de mentira em sockets de verdade: o que se mede é o
que o driver conclui do que CHEGOU, não o texto do código.
"""
import math
import socket
import struct
import threading
import time

import pytest

import touch_pack.real_driver as rd
from touch_pack.real_driver import (
    CR10RealDriver, CR10RealDriverConfig, CR10RealDriverError)


def _controlador(responder, **cfg):
    """Driver ligado a um dashboard falso. `responder(cmd)` → [(atraso, texto)]."""
    ours, robot = socket.socketpair()
    drv = CR10RealDriver(config=CR10RealDriverConfig(**cfg))
    ours.settimeout(drv.cfg.recv_timeout_s)
    drv._dash = ours
    drv.vistos = []

    def serve():
        buf = b''
        while True:
            try:
                data = robot.recv(4096)
            except OSError:
                return
            if not data:
                return
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                cmd = line.decode()
                drv.vistos.append(cmd)
                for dt, txt in responder(cmd) or ():
                    if dt:
                        threading.Timer(dt, robot.sendall, [txt.encode()]).start()
                    else:
                        robot.sendall(txt.encode())
    threading.Thread(target=serve, daemon=True).start()
    drv._robot = robot        # segura a referência
    return drv


def _ok(cmd, val=''):
    return [(0, f'0,{{{val}}},{cmd};')]


# ── XC3 / XC4 — E-STOP ────────────────────────────────────────────────

@pytest.mark.parametrize('resp', ['-10000', '-1', None])
def test_estop_so_confirma_com_zero(resp):
    drv = _controlador(lambda c: [] if resp is None else [(0, f'{resp},{{}},{c};')],
                       cmd_timeout_s=0.1)
    assert drv.emergency_stop() is False
    assert drv.estop_engaged is True
    # caiu para a parada alternativa
    time.sleep(0.05)
    assert 'Stop()' in drv.vistos and 'DisableRobot()' in drv.vistos


def test_estop_confirmado():
    drv = _controlador(_ok)
    assert drv.emergency_stop() is True
    assert drv.vistos == ['EmergencyStop(1)']


def test_estop_com_socket_caindo_durante_a_espera_do_lock():
    drv = _controlador(_ok)
    res = []

    def engata():
        try:
            res.append(drv.emergency_stop())
        except BaseException as exc:
            res.append(repr(exc))
    drv._dash_lock.acquire()
    t = threading.Thread(target=engata)
    t.start()
    time.sleep(0.1)
    drv._drop_dash()
    drv._dash_lock.release()
    t.join(2.0)
    assert res == [False]


def test_estop_nao_espera_a_resposta_lenta_de_outro_comando():
    drv = _controlador(lambda c: _ok(c) if c.startswith('Emerg') else [])
    threading.Thread(target=drv._send_dash, args=('MovJ()',), daemon=True).start()
    time.sleep(0.1)
    t0 = time.monotonic()
    assert drv.emergency_stop() is True
    assert time.monotonic() - t0 < 0.3


def test_soltar_estop_exige_confirmacao():
    drv = _controlador(lambda c: [(0, f'-1,{{}},{c};')])
    drv._estop_engaged = True
    with pytest.raises(CR10RealDriverError):
        drv.release_emergency_stop()
    assert drv.estop_engaged is True
    assert 'EnableRobot()' not in drv.vistos


# ── T05 — silêncio não é sucesso, resposta não troca de dono ──────────

def test_movj_recusado_tarde_levanta():
    drv = _controlador(
        lambda c: [(0.08, f'-2,{{}},{c};')] if c.startswith('MovJ') else _ok(c))
    with pytest.raises(CR10RealDriverError, match='ALARME'):
        drv.mov_j_joint_deg([0] * 6)


@pytest.mark.parametrize('mov', ['mov_j_joint_deg', 'mov_l_joint_deg'])
def test_movimento_sem_resposta_levanta_e_nao_reenvia(mov):
    drv = _controlador(lambda c: [], cmd_timeout_s=0.1)
    with pytest.raises(CR10RealDriverError, match='SEM CONFIRMAÇÃO'):
        getattr(drv, mov)([0] * 6)
    assert len(drv.vistos) == 1


def test_respostas_atrasadas_nao_trocam_de_comando():
    drv = _controlador(lambda c: [(0.08, f'0,{{}},{c};')], cmd_timeout_s=0.05)
    assert drv._send_dash('Cmd_A()') == ''
    drv.cfg.cmd_timeout_s = 0.5
    assert drv._send_dash('Cmd_B()') == '0,{},Cmd_B();'


def test_resposta_atrasada_do_mesmo_comando_nao_vale_pelo_novo():
    """MovJ#1 aceito tarde; MovJ#2 recusado. O '0' do primeiro não pode ser
    lido como sucesso do segundo."""
    n = []

    def resp(c):
        if not c.startswith('MovJ'):
            return _ok(c)
        n.append(c)
        return ([(0.15, f'0,{{}},{c};')] if len(n) == 1
                else [(0.2, f'-2,{{}},{c};')])
    drv = _controlador(resp, cmd_timeout_s=0.1)
    with pytest.raises(CR10RealDriverError):
        drv.mov_j_joint_deg([1] * 6)
    drv.cfg.cmd_timeout_s = 0.6
    with pytest.raises(CR10RealDriverError, match='ALARME'):
        drv.mov_j_joint_deg([2] * 6)


def test_resposta_fragmentada_e_duas_numa_leitura():
    drv = _controlador(lambda c: [(0, '0,{},ServoJ(1);0,{5},Rob'),
                                  (0.07, 'otMode();')])
    assert drv._send_dash('RobotMode()') == '0,{5},RobotMode();'


def test_cauda_truncada_nao_cola_na_proxima_resposta():
    atrasos = iter([[(0, '0,{4},Rob'), (0.15, 'otMode();')],
                    [(0.1, '0,{5},RobotMode();')]])
    drv = _controlador(lambda c: next(atrasos), cmd_timeout_s=0.08)
    assert drv._send_dash('RobotMode()') == ''
    drv.cfg.cmd_timeout_s = 0.6
    assert drv._send_dash('RobotMode()') == '0,{5},RobotMode();'


def test_servoj_ve_erro_em_qualquer_ack_do_buffer():
    drv = _controlador(lambda c: [(0, '0,{},ServoJ();-2,{},ServoJ();')])
    drv.cfg.servoj_recv_timeout_s = 0.2
    with pytest.raises(CR10RealDriverError):
        drv.servo_j([0.0] * 6)


def test_request_control_mudo_nao_e_token():
    drv = _controlador(lambda c: [], cmd_timeout_s=0.05)
    assert drv._request_control_with_retry(retries=2, delay_s=0) is False


def test_enable_exige_nivel_de_colisao_confirmado(monkeypatch):
    monkeypatch.setattr(rd.time, 'sleep', lambda s: None)
    drv = _controlador(lambda c: ([(0, f'-1,{{}},{c};')] if 'Collision' in c
                                  else _ok(c, '5')))
    drv._feed = object()
    with pytest.raises(CR10RealDriverError, match='SetCollisionLevel'):
        drv.enable()


def test_enable_esvazia_a_fila_antes_do_continue(monkeypatch):
    monkeypatch.setattr(rd.time, 'sleep', lambda s: None)
    monkeypatch.setattr(CR10RealDriver, 'apply_runtime_config',
                        lambda self: None)
    drv = _controlador(lambda c: _ok(c, '5'))
    drv._feed = object()
    drv.enable()
    assert drv.vistos.index('Stop()') < drv.vistos.index('Continue()')


# ── T19 — não finito ──────────────────────────────────────────────────

@pytest.mark.parametrize('mov', ['mov_j_joint_deg', 'mov_l_joint_deg', 'servo_j'])
@pytest.mark.parametrize('v', [math.nan, math.inf, -math.inf])
def test_movimento_recusa_nao_finito(mov, v):
    drv = CR10RealDriver(dry_run=True)
    with pytest.raises(CR10RealDriverError):
        getattr(drv, mov)([0, 0, v, 0, 0, 0])


# ── T09 — arrasto sempre devolve a detecção de colisão ────────────────

NIVEL = 'SetCollisionLevel(3)'


@pytest.mark.parametrize('enable', [True, False])
@pytest.mark.parametrize('resp', ['-1', None])
def test_arrasto_que_falha_restaura_colisao(monkeypatch, enable, resp):
    monkeypatch.setattr(rd.time, 'sleep', lambda s: None)
    drv = _controlador(
        lambda c: _ok(c) if not c.startswith('Drag') else
        ([] if resp is None else [(0, f'{resp},{{}},{c};')]), cmd_timeout_s=0.1)
    with pytest.raises(CR10RealDriverError):
        drv.drag_teach(enable)
    assert drv.vistos[-1] == NIVEL


def test_excecao_no_meio_do_arrasto_restaura_colisao():
    drv = _controlador(_ok)
    envia = drv._send_dash

    def quebra(cmd, **k):
        if cmd.startswith('Drag'):
            raise CR10RealDriverError('x')
        return envia(cmd, **k)
    drv._send_dash = quebra
    with pytest.raises(CR10RealDriverError):
        drv.drag_teach(True)
    assert drv.vistos[-1] == NIVEL


def test_arrasto_ligado_mantem_colisao_zero():
    drv = _controlador(_ok)
    drv.drag_teach(True)
    assert 'SetCollisionLevel(0)' in drv.vistos and NIVEL not in drv.vistos
    drv.drag_teach(False)
    assert drv.vistos[-1] == NIVEL


# ── Feedback: T15, T17 e o contrato CR10FeedbackTimeout ───────────────

def _pacote(j1=10.0):
    p = bytearray(1440)
    struct.pack_into('<H', p, 0, 1440)
    struct.pack_into('<Q', p, 24, 5)
    struct.pack_into('<Q', p, 48, 0x0123456789ABCDEF)
    struct.pack_into('<6d', p, 432, j1, 0, 0, 0, 0, 0)
    return bytes(p)


def _feed(timeout=0.2):
    ours, robot = socket.socketpair()
    ours.settimeout(timeout)
    drv = CR10RealDriver()
    drv._feed = ours
    return drv, robot


def test_timeout_de_feedback_e_do_tipo_proprio_e_preserva_a_conexao():
    drv, robot = _feed(0.05)
    with pytest.raises(rd.CR10FeedbackTimeout):
        drv.read_joints_rad()
    assert issubclass(rd.CR10FeedbackTimeout, CR10RealDriverError)
    robot.sendall(_pacote())
    assert math.degrees(drv.read_joints_rad()[0]) == pytest.approx(10.0)


def test_pacote_fragmentado_e_retomado():
    drv, robot = _feed(0.05)
    p = _pacote(20.0)
    robot.sendall(p[:700])
    with pytest.raises(rd.CR10FeedbackTimeout):
        drv.read_joints_rad()
    robot.sendall(p[700:])
    assert math.degrees(drv.read_joints_rad()[0]) == pytest.approx(20.0)


def test_socket_fechado_nao_e_timeout():
    drv, robot = _feed()
    robot.close()
    with pytest.raises(CR10RealDriverError) as e:
        drv.read_joints_rad()
    assert not isinstance(e.value, rd.CR10FeedbackTimeout)


@pytest.mark.parametrize('corte', [1, 300, 1390, 1439])
def test_cabecalho_falso_nao_vira_pose(corte):
    """Um A0 05 no meio dos dados não é fronteira de pacote."""
    p = bytearray(_pacote(30.0))
    p[100:102] = rd.FEEDBACK_MARKER
    p[1395:1397] = rd.FEEDBACK_MARKER
    p = bytes(p)
    drv, robot = _feed()
    robot.sendall(p[corte:] + p + p + p)
    for _ in range(2):
        assert math.degrees(drv.read_joints_rad()[0]) == pytest.approx(30.0)


def test_porta_lenta_entrega_todo_pacote(monkeypatch):
    """30005: um pacote a cada 200 ms. Antes: 34 de 40 leituras falhavam."""
    def servidor(fn):
        s = socket.socket()
        s.bind(('127.0.0.1', 0))
        s.listen(1)

        def run():
            c, _ = s.accept()
            try:
                fn(c)
            except OSError:
                pass
        threading.Thread(target=run, daemon=True).start()
        return s.getsockname()[1]

    def feed(c):
        while True:
            c.sendall(_pacote())
            time.sleep(0.2)
    pd, pf = servidor(lambda c: time.sleep(30)), servidor(feed)
    monkeypatch.setattr(rd, 'FEEDBACK_PORT_SLOW', pf)
    drv = CR10RealDriver(ip='127.0.0.1', config=CR10RealDriverConfig(
        dashboard_port=pd, feedback_port=pf, readonly=True))
    drv.cfg.ip = '127.0.0.1'
    drv.connect()
    try:
        ok = 0
        for _ in range(8):
            drv.read_joints_urdf_latest()
            ok += 1
            time.sleep(0.013)
        assert ok == 8
    finally:
        drv.close()


# ── R11 — keepalive não disputa o lock com o ServoJ ───────────────────

def test_keepalive_pula_com_envio_recente(monkeypatch):
    monkeypatch.setattr(rd, '_KEEPALIVE_IDLE_S', 0.2, raising=False)
    drv = _controlador(_ok)
    drv._feed = object()
    real_wait = drv._keepalive_stop.wait
    # relógio comprimido: o período do keepalive vira 20 ms, seja qual for
    monkeypatch.setattr(drv._keepalive_stop, 'wait',
                        lambda timeout=None: real_wait(0.02))
    drv._start_keepalive()
    try:
        fim = time.monotonic() + 0.5
        while time.monotonic() < fim:
            drv.servo_j([0.0] * 6)
            time.sleep(0.02)
        assert not [c for c in drv.vistos if c.startswith('RobotMode')]
        time.sleep(0.5)              # ocioso: aí sim
        assert [c for c in drv.vistos if c.startswith('RobotMode')]
    finally:
        drv._keepalive_stop.set()


def test_sem_conversao_o_driver_nao_importa():
    """XT24b: sem urdf_to_dobot o driver mandava q URDF (J1 trocado) calado."""
    import subprocess
    import sys
    out = subprocess.run(
        [sys.executable, '-c',
         'import sys; sys.modules["touch_pack.kinematics"] = None\n'
         'import touch_pack.real_driver'],
        capture_output=True, text=True, timeout=30)
    assert out.returncode != 0
    assert 'touch_pack.kinematics' in out.stderr


@pytest.mark.parametrize('resp', ['-1', None])
def test_stop_nao_confirmado_nao_manda_continue(resp):
    """O Continue() depois de um Stop() perdido retomaria a fila."""
    drv = _controlador(lambda c: [] if resp is None else [(0, f'{resp},{{}},{c};')],
                       cmd_timeout_s=0.1)
    with pytest.raises(CR10RealDriverError):
        drv.stop_motion()
    assert drv.vistos and set(drv.vistos) == {'Stop()'}


def test_stop_confirmado_esvazia_e_libera_a_fila():
    drv = _controlador(_ok)
    drv.stop_motion()
    assert drv.vistos == ['Stop()', 'Continue()']
