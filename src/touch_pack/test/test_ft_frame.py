"""Quadro de 28 bytes do FA7155: CRC, parsing e ressincronização.

Sem rclpy de propósito — o que está sob teste é o protocolo, e ele é a única
coisa entre o cabo RS485 e a malha de força. Um erro aqui não aparece como
exceção: aparece como um número de newtons plausível e errado.
"""
import struct

import pytest

from touch_pack.constants import FT_FRAME_HEADER, FT_FRAME_LEN
from touch_pack.ft_serial import (
    FtFrameParser, FtSerialSource, crc16_modbus,
)


def montar(fx=0.0, fy=0.0, fz=0.0, mx=0.0, my=0.0, mz=0.0, *, crc=None):
    """Um quadro válido (ou com CRC forçado, para o teste negativo)."""
    corpo = FT_FRAME_HEADER + struct.pack('<6f', fx, fy, fz, mx, my, mz)
    c = crc16_modbus(corpo) if crc is None else crc
    return corpo + c.to_bytes(2, 'little')


# ── CRC ───────────────────────────────────────────────────────────────

def test_crc_vetor_padrao():
    """Vetor canônico do CRC-16/MODBUS: '123456789' → 0x4B37."""
    assert crc16_modbus(b'123456789') == 0x4B37


def test_crc_cobre_o_cabecalho():
    """O manual calcula sobre os 26 primeiros bytes, cabeçalho INCLUÍDO —
    excluir o cabeçalho daria outro resto e todo quadro seria rejeitado."""
    q = montar(fx=12.5)
    assert crc16_modbus(q[:-2]) == int.from_bytes(q[-2:], 'little')
    assert crc16_modbus(q[2:-2]) != int.from_bytes(q[-2:], 'little')


# ── Exemplo do manual (§4.2, página 6) ────────────────────────────────

def test_exemplo_do_manual_bate_byte_a_byte():
    """Os bytes impressos no manual têm que sair 12,5 N e 13,5 N."""
    assert struct.pack('<f', 12.5) == bytes.fromhex('00004841')
    assert struct.pack('<f', 13.5) == bytes.fromhex('00005841')
    q = montar(fx=12.5, fy=13.5)
    assert q[:2] == bytes.fromhex('5354')
    assert q[2:10] == bytes.fromhex('00004841 00005841'.replace(' ', ''))
    assert len(q) == FT_FRAME_LEN


# ── Parsing ───────────────────────────────────────────────────────────

def test_quadro_inteiro_de_uma_vez():
    p = FtFrameParser()
    out = p.feed(montar(1.0, 2.0, 3.0, 0.1, 0.2, 0.3))
    assert len(out) == 1
    assert out[0] == pytest.approx((1.0, 2.0, 3.0, 0.1, 0.2, 0.3), abs=1e-6)
    assert (p.crc_errors, p.resyncs) == (0, 0)


def test_quadro_partido_em_bytes():
    """O SO entrega o buffer picotado; o quadro só fecha quando fecha."""
    p = FtFrameParser()
    q = montar(fz=-5.25)
    saidas = [p.feed(q[i:i + 1]) for i in range(len(q))]
    assert [len(s) for s in saidas[:-1]] == [0] * (len(q) - 1)
    assert saidas[-1][0][2] == pytest.approx(-5.25)


def test_varios_quadros_num_bloco():
    p = FtFrameParser()
    bloco = b''.join(montar(fx=float(i)) for i in range(5))
    out = p.feed(bloco)
    assert [o[0] for o in out] == pytest.approx([0.0, 1.0, 2.0, 3.0, 4.0])


def test_entrar_no_meio_do_fluxo_ressincroniza():
    """É o caso REAL: o sensor já está falando quando a porta abre."""
    p = FtFrameParser()
    q = montar(fx=7.0)
    out = p.feed(q[13:] + montar(fx=8.0))
    assert len(out) == 1
    assert out[0][0] == pytest.approx(8.0)
    assert p.resyncs >= 1


def test_crc_errado_e_descartado():
    p = FtFrameParser()
    out = p.feed(montar(fx=1.0, crc=0x0000) + montar(fx=2.0))
    assert [o[0] for o in out] == pytest.approx([2.0])
    assert p.crc_errors == 1


def test_cabecalho_falso_dentro_dos_dados_nao_come_o_quadro_seguinte():
    """0x53 0x54 é um valor de força perfeitamente comum. Andar 28 bytes a
    partir de um cabeçalho falso picotaria o quadro real logo atrás; o parser
    anda UM byte e reencontra o alinhamento."""
    falso = struct.unpack('<f', bytes.fromhex('53540000'))[0]
    # Corta o cabeçalho REAL fora: o primeiro 0x53 0x54 que o parser acha é o
    # que está dentro do campo fy, exatamente como ao entrar no meio do fluxo.
    picotado = montar(fy=falso)[3:]
    p = FtFrameParser()
    out = p.feed(picotado + montar(fz=9.0))
    assert len(out) == 1
    assert out[0][2] == pytest.approx(9.0)
    assert p.crc_errors == 1        # o cabeçalho falso foi testado e recusado


def test_nan_nao_passa():
    """Um NaN envenena o One-Euro do receiver para sempre — ele morre aqui."""
    p = FtFrameParser()
    out = p.feed(montar(fx=float('nan')) + montar(fx=3.0))
    assert [o[0] for o in out] == pytest.approx([3.0])
    assert p.bad_values == 1


def test_lixo_puro_nao_estoura_a_memoria():
    p = FtFrameParser()
    for _ in range(200):
        assert p.feed(b'\xAB' * 100) == []
    assert len(p._buf) <= 8192


# ── Laço de leitura e carimbo ─────────────────────────────────────────

class _SerialFalsa:
    """Serial de mentira: entrega blocos na ordem e encerra o laço no fim."""

    def __init__(self, blocos):
        self._blocos = list(blocos)
        self._buf = b''

    def read(self, n):
        if not self._buf and self._blocos:
            self._buf = self._blocos.pop(0)
        if not self._buf:
            raise _FimDoFluxo
        out, self._buf = self._buf[:n], self._buf[n:]
        return out

    @property
    def in_waiting(self):
        return len(self._buf)


class _FimDoFluxo(Exception):
    pass


def _rodar(blocos, rate_hz=250.0):
    got = []
    src = FtSerialSource(rate_hz=rate_hz,
                         on_sample=lambda s, t, v: got.append((s, t, v)))
    src._running = True
    try:
        src._read_loop(_SerialFalsa(blocos))
    except _FimDoFluxo:
        pass
    return got


def test_leitura_numera_e_atravessa_a_fronteira_do_bloco():
    q = montar(fz=1.0) + montar(fz=2.0) + montar(fz=3.0)
    partido = montar(fz=4.0)
    got = _rodar([q, partido[:10], partido[10:]])
    assert [s for s, _, _ in got] == [0, 1, 2, 3]
    assert [v[2] for _, _, v in got] == pytest.approx([1.0, 2.0, 3.0, 4.0])


def test_quadros_do_mesmo_bloco_sao_retrodatados():
    """Carimbar os k quadros de um bloco com o MESMO instante daria dt=0, e o
    One-Euro do receiver cairia na taxa nominal justamente nos blocos — que é
    quando o dt real importa. O período nominal separa os carimbos."""
    got = _rodar([montar(fz=1.0) + montar(fz=2.0) + montar(fz=3.0)],
                 rate_hz=250.0)
    ts = [t for _, t, _ in got]
    assert [ts[i + 1] - ts[i] for i in range(len(ts) - 1)] == [4000, 4000]
