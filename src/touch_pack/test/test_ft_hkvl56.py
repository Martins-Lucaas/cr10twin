"""Vetores DOURADOS do manual do HKVL-56 (Suzhou Hangkai), §4.2.

Estes testes não inventam nada: cada quadro abaixo está impresso no manual do
sensor, com o CRC que o fabricante publicou. Se um dia alguém "consertar" o
enquadramento ou o CRC, é aqui que a mudança aparece — antes de chegar à
linha 485.

O manual documenta só duas funções: 0x03 (ler os seis eixos) e 0x06 (gravar o
ID). Zero, taxa de saída e baud NÃO estão nele.
"""
import struct

import pytest

from touch_pack.constants import FT_PROFILE_HKVL56 as P
from touch_pack.ft_serial import crc16_modbus
from touch_pack import ft_modbus as m


# Exemplo 1 do manual — "0x03 – Read Six-Dimensional Force Value Command"
_EX1_REQ = bytes.fromhex('010300030018b5c0')
# Exemplo 2 do manual — "0x06 Write ID Command", grava ID = 2
_EX2_REQ = bytes.fromhex('01060000000208 0b'.replace(' ', ''))


def _crc(payload: bytes) -> bytes:
    return payload + struct.pack('<H', crc16_modbus(payload))


# ── Os quadros do manual, byte a byte ─────────────────────────────────

def test_requisicao_de_leitura_bate_com_o_manual():
    req = m.build_read(0x01, m.FUNC_READ_HOLDING,
                       P['data_addr'], P['data_count'])
    assert req == _EX1_REQ


def test_requisicao_de_escrita_de_id_bate_com_o_manual():
    req = m.build_write_single(0x01, P['node_id'], 2)
    assert req == _EX2_REQ


def test_crc_publicado_pelo_fabricante_confere():
    """Se este teste cair, o crc16_modbus do repo divergiu do do sensor —
    e nada mais na linha vai funcionar."""
    for req in (_EX1_REQ, _EX2_REQ):
        assert crc16_modbus(req[:-2]) == int.from_bytes(req[-2:], 'little')


# ── Os dois desvios do Modbus padrão ──────────────────────────────────

def test_resposta_de_leitura_tem_30_bytes_nao_29():
    """A nota do §4.2 lista: slave(1) + func(1) + endereço inicial(2) +
    dados(24) + CRC(2). O padrão traria um bytecount de 1 byte no lugar do
    endereço, e a resposta teria 29."""
    assert m.expected_response_len(m.FUNC_READ_HOLDING, P['data_count'],
                                   m.STYLE_HKVL56) == 30
    assert m.expected_response_len(m.FUNC_READ_HOLDING, 12,
                                   m.STYLE_STANDARD) == 29


def test_number_of_reads_conta_bytes_e_nao_registradores():
    """0x0018 = 24. No padrão seriam 24 registradores (48 bytes de dados);
    aqui são 24 BYTES, que é o que cabe em seis float32."""
    assert P['data_count'] == 24
    assert P['data_count'] == 6 * 4


# ── Decodificação dos seis eixos ──────────────────────────────────────

def _resposta(vals) -> bytes:
    """Monta a resposta como o manual descreve."""
    return _crc(bytes([0x01, 0x03]) + struct.pack('>H', P['data_addr'])
                + struct.pack('<6f', *vals))


def test_floats_do_exemplo_do_manual():
    """O manual mostra Fx = 00 00 48 41 e Fy = 00 00 58 41, e diz que a
    leitura resulta em 12,5 e 13,5 — little-endian."""
    frame = _crc(bytes([0x01, 0x03]) + struct.pack('>H', P['data_addr'])
                 + bytes.fromhex('00004841') + bytes.fromhex('00005841')
                 + b'\x00' * 16)
    fx, fy = m.parse_wrench(frame, P['data_count'], m.STYLE_HKVL56)[:2]
    assert fx == pytest.approx(12.5)
    assert fy == pytest.approx(13.5)


def test_seis_eixos_ida_e_volta():
    vals = (1.5, -2.25, 300.0, 0.125, -0.5, 19.75)
    got = m.parse_wrench(_resposta(vals), P['data_count'], m.STYLE_HKVL56)
    assert got == pytest.approx(vals)


def test_payload_curto_e_recusado_em_vez_de_devolver_lixo():
    curto = _crc(bytes([0x01, 0x03, 0x00, 0x03]) + b'\x00' * 8)
    with pytest.raises(m.ModbusError):
        m.parse_wrench(curto, P['data_count'], m.STYLE_HKVL56)


# ── Transação completa no estilo do fabricante ────────────────────────

class _HkvlSlave:
    """Escravo de mentira que responde EXATAMENTE como o manual descreve."""

    def __init__(self, vals=(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)):
        self.vals = vals
        self._pend = bytearray()
        self.ultimo_req = None

    def write(self, data: bytes) -> None:
        self.ultimo_req = data
        if data[1] == m.FUNC_READ_HOLDING:
            self._pend += _resposta(self.vals)
        else:
            self._pend += _crc(data[:6])       # 0x06 ecoa, e isso É padrão

    def read(self, n: int) -> bytes:
        out = bytes(self._pend[:n])
        del self._pend[:n]
        return out


def test_leitura_dos_seis_eixos_ponta_a_ponta():
    sl = _HkvlSlave()
    c = m.FtModbusClient(sl.write, sl.read, slave_id=1, timeout_s=0.3,
                         style=m.STYLE_HKVL56)
    frame = c.read_frame(P['data_addr'], P['data_count'])
    assert m.parse_wrench(frame, P['data_count'],
                          m.STYLE_HKVL56) == pytest.approx(sl.vals)
    assert sl.ultimo_req == _EX1_REQ


def test_escrita_de_id_ponta_a_ponta():
    sl = _HkvlSlave()
    c = m.FtModbusClient(sl.write, sl.read, slave_id=1, timeout_s=0.3,
                         style=m.STYLE_HKVL56)
    c.write_register(P['node_id'], 2)
    assert sl.ultimo_req == _EX2_REQ


def test_o_estilo_padrao_nao_le_a_resposta_do_hkvl():
    """Guarda contra escolher o estilo errado: com a peneira do padrão a
    resposta de 30 bytes não é encontrada, e vira timeout — nunca um valor
    de força errado, que é o desfecho que importa evitar."""
    sl = _HkvlSlave()
    c = m.FtModbusClient(sl.write, sl.read, slave_id=1, timeout_s=0.05,
                         style=m.STYLE_STANDARD)
    with pytest.raises(m.ModbusTimeout):
        c.read_frame(P['data_addr'], P['data_count'])


# ── O perfil em si ────────────────────────────────────────────────────

def test_perfil_registra_o_baud_de_fabrica_do_manual():
    """"...the default baud rate when powering on ... is 1 Mbps" — e NÃO os
    115200 que o resto do repo assume para o FA7155."""
    assert P['baud_default'] == 1_000_000


def test_perfil_nao_finge_conhecer_o_que_o_manual_nao_traz():
    """O manual só documenta 0x03 e 0x06-ID. Zero, taxa e baud continuam
    desconhecidos, e o perfil tem de dizer isso em vez de chutar."""
    for chave in ('zero', 'rate', 'baud', 'stream'):
        assert P[chave] is None
