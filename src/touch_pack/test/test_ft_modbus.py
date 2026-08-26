"""Enquadramento Modbus RTU do canal de comando do FA7155.

O que estes testes protegem: o quadro que sai na linha 485 e a peneira que
acha a resposta no meio do stream "ST". Nada aqui toca hardware — o
transporte é um par de callables, exatamente para isto ser testável.

O mapa de registradores NÃO é testado (ele ainda não existe); o que é testado
é que a trava contra escrever num endereço adivinhado funciona.
"""
import struct

import pytest

from touch_pack.constants import FT_FRAME_HEADER
from touch_pack.ft_serial import crc16_modbus
from touch_pack import ft_modbus as m
from touch_pack import ft_cmd_channel as m_tap


def _crc(payload: bytes) -> bytes:
    return payload + struct.pack('<H', crc16_modbus(payload))


def _stream_frame(vals=(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)) -> bytes:
    """Um quadro "ST" legítimo, para poluir o buffer como na bancada."""
    body = FT_FRAME_HEADER + struct.pack('<6f', *vals)
    return _crc(body)


# ── Construção das requisições ────────────────────────────────────────

def test_read_holding_bate_com_o_quadro_do_manual():
    req = m.build_read(1, m.FUNC_READ_HOLDING, 0x0010, 2)
    assert req[:6] == bytes.fromhex('0103001000 02'.replace(' ', ''))
    assert crc16_modbus(req[:-2]) == int.from_bytes(req[-2:], 'little')


def test_write_single_tem_8_bytes_e_crc_valido():
    req = m.build_write_single(3, 0x0007, 1)
    assert len(req) == 8
    assert req[0] == 3 and req[1] == m.FUNC_WRITE_SINGLE
    assert crc16_modbus(req[:-2]) == int.from_bytes(req[-2:], 'little')


def test_write_multiple_carrega_bytecount_correto():
    req = m.build_write_multiple(1, 0x0020, [0x0000, 0x03E8])
    # slave func addr(2) qtd(2) bytecount valores(4) crc(2)
    assert len(req) == 7 + 4 + 2
    assert req[6] == 4                      # bytecount = 2 regs x 2 bytes
    assert struct.unpack('>H', req[4:6])[0] == 2


def test_valor_fora_de_16_bits_e_recusado():
    with pytest.raises(ValueError):
        m.build_write_single(1, 0x0001, 0x1_0000)
    with pytest.raises(ValueError):
        m.build_write_multiple(1, 0x0001, [0x1_0000])


def test_tamanho_esperado_da_resposta():
    assert m.expected_response_len(m.FUNC_READ_HOLDING, 2) == 9
    assert m.expected_response_len(m.FUNC_WRITE_SINGLE, 1) == 8
    assert m.expected_response_len(m.FUNC_WRITE_MULTIPLE, 2) == 8


# ── A peneira: achar a resposta no meio do stream ─────────────────────

def test_acha_resposta_cercada_de_quadros_de_stream():
    resp = _crc(bytes([1, m.FUNC_READ_HOLDING, 4]) + struct.pack('>2H', 7, 9))
    buf = _stream_frame() + resp + _stream_frame()
    hit = m.find_response(buf, 1, m.FUNC_READ_HOLDING, len(resp))
    assert hit is not None
    frame, _ = hit
    assert m.parse_read_response(frame, 2) == [7, 9]


def test_nao_confunde_dado_do_stream_com_resposta():
    """O byte do escravo e o da função aparecem dentro dos floats; quem
    decide é o CRC, não o casamento de dois bytes."""
    buf = _stream_frame((1.0, 1.0, 1.0, 1.0, 1.0, 1.0)) * 4
    assert m.find_response(buf, 1, m.FUNC_READ_HOLDING, 9) is None


def test_resposta_de_excecao_e_reconhecida():
    exc = _crc(bytes([1, m.FUNC_WRITE_SINGLE | 0x80, 0x02]))
    hit = m.find_response(_stream_frame() + exc, 1, m.FUNC_WRITE_SINGLE, 8)
    assert hit is not None
    assert hit[0][1] == m.FUNC_WRITE_SINGLE | 0x80


def test_resposta_com_crc_errado_nao_passa():
    bad = bytes([1, m.FUNC_READ_HOLDING, 4]) + struct.pack('>2H', 7, 9) + b'\x00\x00'
    assert m.find_response(bad, 1, m.FUNC_READ_HOLDING, len(bad)) is None


# ── Transação completa contra um escravo de mentira ───────────────────

class _FakeLine:
    """Escravo mínimo: responde a leitura e ecoa escrita, misturando stream."""

    def __init__(self, slave=1, responder=None, ruido=True):
        self.slave = slave
        self.responder = responder
        self.ruido = ruido
        self.enviado = []
        self._pend = bytearray()

    def write(self, data: bytes) -> None:
        self.enviado.append(data)
        if self.ruido:
            self._pend += _stream_frame()
        r = self.responder(data) if self.responder else self._default(data)
        if r:
            self._pend += r
        if self.ruido:
            self._pend += _stream_frame()

    def read(self, n: int) -> bytes:
        out = bytes(self._pend[:n])
        del self._pend[:n]
        return out

    def _default(self, req: bytes) -> bytes:
        func = req[1]
        if func in (m.FUNC_READ_HOLDING, m.FUNC_READ_INPUT):
            count = struct.unpack('>H', req[4:6])[0]
            return _crc(bytes([self.slave, func, count * 2])
                        + struct.pack(f'>{count}H', *range(count)))
        return _crc(req[:6])          # escrita: eco dos 6 primeiros bytes


def _client(line, **kw):
    return m.FtModbusClient(line.write, line.read, slave_id=line.slave, **kw)


def test_leitura_atravessa_o_stream():
    line = _FakeLine()
    c = _client(line)
    assert c.read_registers(0x0010, 3) == [0, 1, 2]
    assert c.rx == 1 and c.timeouts == 0


def test_escrita_simples_confirma_pelo_eco():
    line = _FakeLine()
    c = _client(line)
    c.write_register(0x0007, 1)
    assert c.tx == 1 and c.rx == 1


def test_escravo_mudo_da_timeout_e_conta():
    line = _FakeLine(responder=lambda req: b'')
    c = _client(line, timeout_s=0.05)
    with pytest.raises(m.ModbusTimeout):
        c.read_registers(0, 1)
    assert c.timeouts == 1


def test_excecao_do_escravo_vira_erro_legivel():
    line = _FakeLine(
        responder=lambda req: _crc(bytes([1, req[1] | 0x80, 0x02])))
    c = _client(line, timeout_s=0.2)
    with pytest.raises(m.ModbusExceptionResponse) as ei:
        c.write_register(0x0099, 1)
    assert ei.value.code == 0x02
    assert 'endereço de registrador ilegal' in str(ei.value)


def test_resposta_do_escravo_errado_e_ignorada():
    """Dois sensores na mesma linha: a resposta do vizinho não pode valer."""
    line = _FakeLine(slave=1,
                     responder=lambda req: _crc(bytes([2, req[1], 2, 0, 5])))
    c = _client(line, timeout_s=0.05)
    with pytest.raises(m.ModbusTimeout):
        c.read_registers(0, 1)


# ── A trava do mapa não confirmado ────────────────────────────────────

def test_toda_escrita_e_recusada_enquanto_o_mapa_nao_for_confirmado():
    """A razão de existir da trava: endereço adivinhado pode custar o node ID
    ou um baud inacessível, e o sensor não tem reset de fábrica."""
    dev = m.FtDevice(_client(_FakeLine()))
    for chamada in (dev.set_zero,
                    lambda: dev.set_output_rate_hz(100),
                    lambda: dev.set_node_id(2),
                    lambda: dev.set_baud(115200),
                    dev.start_stream,
                    dev.stop_stream):
        with pytest.raises(m.FtModbusMapUnconfirmed):
            chamada()


def test_trava_barra_antes_de_qualquer_byte_sair_na_linha():
    line = _FakeLine()
    dev = m.FtDevice(_client(line))
    with pytest.raises(m.FtModbusMapUnconfirmed):
        dev.set_zero()
    assert line.enviado == []


def test_node_id_fora_da_faixa_e_recusado_antes_da_trava():
    dev = m.FtDevice(_client(_FakeLine()))
    with pytest.raises(ValueError):
        dev.set_node_id(0)
    with pytest.raises(ValueError):
        dev.set_node_id(248)


def test_leitura_continua_liberada_sem_o_mapa():
    """Ler é inofensivo: no pior caso o escravo devolve exceção 0x02."""
    dev = m.FtDevice(_client(_FakeLine()))
    assert dev.read_device_id() is None        # sem endereço no mapa
    assert dev.probe() == {}


def test_u32_ida_e_volta():
    assert m._regs_to_u32(m._u32_to_regs(921600)) == 921600
    assert m._u32_to_regs(0x0001_86A0) == [0x0001, 0x86A0]


# ── Derivação da linha: comando e stream dividem a MESMA porta ────────
# Sem isto o Modbus roubaria bytes do FtFrameParser e picotaria os quadros —
# exatamente o que o parser existe para evitar.

class _Tapped(m_tap.LineTapMixin):
    """Transporte de mentira com a mesma mecânica de FtSerialSource."""

    def __init__(self):
        self._tap_init()
        self.na_linha = []

    def _line_write(self, data: bytes) -> None:
        self.na_linha.append(data)


def test_stream_so_e_derivado_com_sessao_aberta():
    t = _Tapped()
    t._tap_feed(b'perdido')            # fora de sessão: descartado
    with t.command_session() as (_w, read):
        t._tap_feed(b'capturado')
        assert read(64) == b'capturado'


def test_buffer_da_derivacao_e_limpo_ao_fechar():
    t = _Tapped()
    with t.command_session():
        t._tap_feed(b'x' * 10)
    with t.command_session() as (_w, read):
        assert read(64) == b''         # sessão nova não herda o resto


def test_transacao_completa_pela_derivacao():
    """O caminho real: o cliente escreve pelo transporte e lê da derivação
    enquanto o laço de leitura continua despejando quadros "ST"."""
    t = _Tapped()
    with t.command_session() as (write, read):
        c = m.FtModbusClient(write, read, slave_id=1, timeout_s=0.5)

        def alimenta(req: bytes) -> None:
            t._tap_feed(_stream_frame())
            count = struct.unpack('>H', req[4:6])[0]
            t._tap_feed(_crc(bytes([1, req[1], count * 2])
                             + struct.pack(f'>{count}H', *([42] * count))))
            t._tap_feed(_stream_frame())

        req = m.build_read(1, m.FUNC_READ_HOLDING, 0x0002, 2)
        alimenta(req)                  # o "sensor" responde antes da leitura
        frame = c.transact(req, m.FUNC_READ_HOLDING,
                           m.expected_response_len(m.FUNC_READ_HOLDING, 2))
        assert m.parse_read_response(frame, 2) == [42, 42]
    # A requisição saiu de fato na linha, pelo _line_write do transporte.
    assert t.na_linha == [req]


def test_derivacao_nao_cresce_sem_limite():
    """Sessão esquecida aberta com o sensor a 250 Hz não pode virar
    vazamento."""
    t = _Tapped()
    with t.command_session() as (_w, _r):
        for _ in range(200):
            t._tap_feed(b'z' * 1024)
        assert len(t._tap_buf) <= 65536
