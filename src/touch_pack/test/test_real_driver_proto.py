"""Testes do protocolo ASCII do CR10RealDriver (porta 29999).

O driver não importa rclpy, então isto roda em qualquer máquina. O foco é o
enquadramento das respostas: é ele que decide se o driver acredita ou não no
que o robô respondeu, e um engano aqui vira modo de robô lido errado.
"""
import socket

import pytest

from touch_pack.real_driver import CR10RealDriver, CR10RealDriverError


class _FakeSock:
    """Socket de mentira: entrega `chunks` em sequência; None = timeout."""

    def __init__(self, chunks):
        self._chunks = list(chunks)
        self.timeouts = []
        self.closed = False
        self.sent = []

    def recv(self, _n):
        if not self._chunks:
            raise socket.timeout()
        c = self._chunks.pop(0)
        if c is None:
            raise socket.timeout()
        return c

    def settimeout(self, t):
        self.timeouts.append(t)

    def gettimeout(self):
        return self.timeouts[-1] if self.timeouts else None

    def sendall(self, data):
        self.sent.append(data)

    def close(self):
        self.closed = True


# ── Enquadramento da resposta ─────────────────────────────────────────

def test_complete_line_terminated_by_semicolon():
    txt, complete = CR10RealDriver._recv_line(_FakeSock([b'0,{5},RobotMode();']))
    assert complete is True
    assert txt == '0,{5},RobotMode();'


def test_complete_line_terminated_by_newline():
    txt, complete = CR10RealDriver._recv_line(_FakeSock([b'0,{},Stop();\n']))
    assert complete is True


def test_line_reassembled_across_chunks():
    txt, complete = CR10RealDriver._recv_line(
        _FakeSock([b'0,{5},Rob', b'otMode();']))
    assert complete is True
    assert txt == '0,{5},RobotMode();'


def test_timeout_with_no_bytes_is_incomplete_and_empty():
    txt, complete = CR10RealDriver._recv_line(_FakeSock([None]))
    assert (txt, complete) == ('', False)


def test_truncated_line_is_flagged_incomplete():
    """O caso perigoso: chegaram bytes, mas a linha não fechou. Devolver isto
    como se fosse resposta inteira faria _wait_mode parsear um modo cortado."""
    txt, complete = CR10RealDriver._recv_line(_FakeSock([b'0,{5},Rob', None]))
    assert complete is False
    assert txt == '0,{5},Rob'


def test_peer_close_ends_the_read():
    txt, complete = CR10RealDriver._recv_line(_FakeSock([b'0,{5}', b'']))
    assert complete is False
    assert txt == '0,{5}'


# ── _send_dash descarta resposta truncada ─────────────────────────────

def _driver_with(sock):
    drv = CR10RealDriver(ip='127.0.0.1')
    drv._dash = sock
    return drv


# `_send_dash` DRENA antes de ler (é o que evita casar comando com resposta
# alheia), então o primeiro None de cada roteiro abaixo é a drenagem batendo
# no fim da fila — a resposta de verdade vem depois dele.

def test_send_dash_returns_a_complete_reply():
    drv = _driver_with(_FakeSock([None, b'0,{5},RobotMode();']))
    assert drv._send_dash('RobotMode()') == '0,{5},RobotMode();'


def test_send_dash_discards_a_truncated_reply():
    """Melhor devolver vazio que devolver meia resposta: quem parseia trata
    vazio como 'não sei', e não como um valor."""
    drv = _driver_with(_FakeSock([None, b'0,{5},Rob', None]))
    assert drv._send_dash('RobotMode()') == ''


def test_send_dash_drains_before_reading():
    """Regressão do casamento comando↔resposta: um ACK velho parado no
    socket não pode ser devolvido como resposta do comando novo."""
    sock = _FakeSock([b'0,{},ServoJ();', None, b'0,{5},RobotMode();'])
    drv = _driver_with(sock)
    assert drv._send_dash('RobotMode()') == '0,{5},RobotMode();'


def test_send_dash_closes_the_socket_when_it_dies():
    class _Dead(_FakeSock):
        def sendall(self, data):
            raise OSError('broken pipe')

    sock = _Dead([])
    drv = _driver_with(sock)
    with pytest.raises(CR10RealDriverError):
        drv._send_dash('RobotMode()')
    assert sock.closed is True      # fechado, não só desreferenciado
    assert drv._dash is None


# ── Drenagem espera ACK em voo só quando houve ServoJ ─────────────────

def test_drain_is_non_blocking_without_servoj():
    sock = _FakeSock([None])
    drv = _driver_with(sock)
    drv._servoj_streamed = False
    drv._drain_stale_responses()
    assert sock.timeouts[0] == 0.0


def test_drain_waits_for_in_flight_acks_after_servoj():
    """Sem esta espera o ACK em voo virava a resposta do PRÓXIMO comando —
    e _wait_mode acreditaria num modo que não é o do robô."""
    sock = _FakeSock([None])
    drv = _driver_with(sock)
    drv._servoj_streamed = True
    drv._drain_stale_responses()
    assert sock.timeouts[0] == pytest.approx(CR10RealDriver._DRAIN_SETTLE_S)


def test_drain_clears_the_servoj_flag():
    drv = _driver_with(_FakeSock([None]))
    drv._servoj_streamed = True
    drv._drain_stale_responses()
    assert drv._servoj_streamed is False


def test_drain_restores_the_normal_timeout():
    sock = _FakeSock([b'0,{},ServoJ();', None])
    drv = _driver_with(sock)
    drv._servoj_streamed = True
    drv._drain_stale_responses()
    assert sock.timeouts[-1] == pytest.approx(drv.cfg.recv_timeout_s)


# ── RequestControl: -10000 é comando inexistente, não token negado ────

def test_request_control_accepts_unsupported_command():
    """O controlador da bancada responde -10000 a RequestControl() — o mesmo
    código que devolve a um comando inventado. Tratar isso como token negado
    recusava a conexão acusando um segundo PC que não existe."""
    sock = _FakeSock([None, b'-10000,{},RequestControl();'])
    drv = _driver_with(sock)
    assert drv._request_control_with_retry(retries=4, delay_s=0) is True
    # Uma tentativa só: retentar comando inexistente nunca muda de resposta.
    assert sock.sent == [b'RequestControl()\n']


def test_request_control_still_fails_on_a_real_denial():
    sock = _FakeSock([None, b'-1,{},RequestControl();',
                      None, b'-1,{},RequestControl();'])
    drv = _driver_with(sock)
    assert drv._request_control_with_retry(retries=2, delay_s=0) is False
