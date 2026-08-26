"""Transporte da 60000 (RS485 do flange): entrega de quadros e reconexão.

Sem rclpy e sem hardware: um FA7155 falso escuta num socket local e cospe
quadros, exatamente como o controlador faria ao repassar a linha 485. O que
está sob teste é só o cano — o parser já tem cobertura em test_ft_frame.py.

O caso que justifica o arquivo é o da FRAGMENTAÇÃO: recv() devolve blocos de
tamanho arbitrário, e um quadro de 28 bytes atravessa a fronteira de dois
recv() com frequência banal. Se o transporte perdesse a metade pendente, a
força chegaria picotada em vez de faltar — falha silenciosa, não exceção.
"""
import socket
import struct
import threading
import time

import pytest

from touch_pack.constants import FT_FRAME_HEADER
from touch_pack.ft_serial import crc16_modbus
from touch_pack.ft_tcp import FtTcpSource


def montar(fx=0.0, fy=0.0, fz=0.0, mx=0.0, my=0.0, mz=0.0):
    corpo = FT_FRAME_HEADER + struct.pack('<6f', fx, fy, fz, mx, my, mz)
    return corpo + crc16_modbus(corpo).to_bytes(2, 'little')


class FalsoFA7155:
    """Servidor local que despeja `payload` em pedaços de `chunk` bytes."""

    def __init__(self, payload: bytes, chunk: int = 4096):
        self._payload = payload
        self._chunk = chunk
        self._srv = socket.socket()
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(('127.0.0.1', 0))
        self._srv.listen(1)
        self.port = self._srv.getsockname()[1]
        self._stop = False
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self):
        while not self._stop:
            try:
                self._srv.settimeout(0.3)
                conn, _ = self._srv.accept()
            except (socket.timeout, OSError):
                continue
            try:
                for i in range(0, len(self._payload), self._chunk):
                    conn.sendall(self._payload[i:i + self._chunk])
                    time.sleep(0.002)
            except OSError:
                pass
            finally:
                conn.close()

    def close(self):
        self._stop = True
        self._srv.close()


def coletar(port, esperado, timeout=4.0):
    """Roda a fonte até juntar `esperado` amostras (ou estourar o timeout)."""
    got = []
    src = FtTcpSource(host='127.0.0.1', tcp_port=port,
                      on_sample=lambda seq, t_us, vals: got.append(
                          (seq, t_us, vals)))
    assert src.start()
    try:
        t0 = time.monotonic()
        while len(got) < esperado and time.monotonic() - t0 < timeout:
            time.sleep(0.01)
    finally:
        src.stop()
    return got, src


def test_entrega_valores_corretos():
    """O caminho inteiro: bytes no socket → floats no callback."""
    srv = FalsoFA7155(montar(fx=1.5, fy=-2.0, fz=3.25,
                             mx=0.5, my=-0.25, mz=0.125) * 10)
    try:
        got, _ = coletar(srv.port, 10)
    finally:
        srv.close()
    assert len(got) >= 10
    _, _, vals = got[0]
    assert vals == pytest.approx((1.5, -2.0, 3.25, 0.5, -0.25, 0.125))


def test_quadro_partido_entre_recv():
    """Quadros picotados em 7 bytes por envio ainda fecham.

    28 não é múltiplo de 7 por acidente: garante que a emenda caia em ponto
    diferente a cada quadro, cobrindo as quatro fronteiras possíveis.
    """
    payload = b''.join(montar(fz=float(i)) for i in range(20))
    srv = FalsoFA7155(payload, chunk=7)
    try:
        got, src = coletar(srv.port, 20)
    finally:
        srv.close()
    assert len(got) == 20
    assert [v[2] for _, _, v in got] == [float(i) for i in range(20)]
    assert src.parser.crc_errors == 0


def test_seq_e_carimbo_monotonicos():
    """seq incrementa de 1 em 1 e t_us não anda para trás dentro do bloco.

    Se todos os quadros de um recv levassem o mesmo carimbo, dt=0 derrubaria
    o One-Euro do receiver para a taxa nominal justamente nos blocos grandes.
    """
    srv = FalsoFA7155(montar(fz=1.0) * 30)
    try:
        got, _ = coletar(srv.port, 30)
    finally:
        srv.close()
    seqs = [s for s, _, _ in got]
    assert seqs == list(range(len(seqs)))
    ts = [t for _, t, _ in got]
    assert all(b >= a for a, b in zip(ts, ts[1:]))


def test_reconecta_quando_o_peer_fecha():
    """O servidor fecha depois de cada rajada; a fonte tem de voltar sozinha.

    É o caso real do controlador derrubando a 60000 (religar a ponta com
    SetToolPower, por exemplo) — sem isso o nó ficaria mudo até o restart.
    """
    srv = FalsoFA7155(montar(fz=7.0) * 3)
    try:
        # 3 por rajada; pedir 6 obriga a pelo menos uma reconexão.
        got, _ = coletar(srv.port, 6, timeout=8.0)
    finally:
        srv.close()
    assert len(got) >= 6
    assert all(v[2] == 7.0 for _, _, v in got)


def test_sem_servidor_nao_explode_e_reporta_erro():
    """Robô inalcançável é estado normal de partida, não exceção."""
    src = FtTcpSource(host='127.0.0.1', tcp_port=1)   # ninguém escuta
    assert src.start()
    time.sleep(0.5)
    try:
        assert not src.connected
        assert src.error
    finally:
        src.stop()
