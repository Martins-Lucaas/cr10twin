"""Modo POLLED do FA7155 — o caminho que o cliente/HMI de fábrica usa.

Os vetores dourados são os DOIS quadros da tela de comando do HMI, capturada
em 26/08/2026 e conferida byte a byte:

    requisição   01 03 00 03 00 0C B5 CF
    resposta     01 03 18 <24 B> 10 A3

Mesmo espírito do test_ft_hkvl56.py: se o enquadramento mudar, é aqui que
quebra, e a mensagem aponta para a fonte física em vez de um número mágico.

Nada aqui toca hardware — o transporte é um duplo que implementa
`command_session()`, exatamente como o FtSerialSource e o FtTcpSource.
"""
import struct
import threading
import time
from contextlib import contextmanager

import pytest

from touch_pack.constants import (
    FT_MODBUS_DATA_ADDR, FT_MODBUS_DATA_REGS, ft_polled_max_rate_hz,
)
from touch_pack.ft_serial import crc16_modbus
from touch_pack import ft_modbus as m
from touch_pack.ft_polled import FtPolledDriver


# ── Vetores dourados da tela do HMI ───────────────────────────────────
REQ_HMI = bytes.fromhex('01 03 00 03 00 0C B5 CF'.replace(' ', ''))
REP_HMI = bytes.fromhex(
    '010318' + '00004842' * 6 + '10A3')


def _crc(payload: bytes) -> bytes:
    return payload + struct.pack('<H', crc16_modbus(payload))


class FakeLine:
    """Transporte de mentira com a forma do LineTapMixin.

    `responder` recebe a requisição e devolve os bytes que o escravo mandaria
    (ou b'' para simular escravo mudo). `ruido` é pré-carregado no buffer para
    imitar o stream "ST" chegando junto.
    """

    def __init__(self, responder, ruido: bytes = b''):
        self._responder = responder
        self._ruido = ruido
        self._buf = bytearray()
        self._aberta = False
        self.pedidos = []
        self.sessoes = 0

    @contextmanager
    def command_session(self):
        if self._aberta:
            raise RuntimeError('já existe uma sessão de comando aberta')
        self._aberta = True
        self.sessoes += 1
        self._buf = bytearray(self._ruido)
        try:
            yield self._write, self._read
        finally:
            self._aberta = False

    def _write(self, data: bytes) -> None:
        self.pedidos.append(bytes(data))
        self._buf += self._responder(bytes(data))

    def _read(self, n: int) -> bytes:
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out


def _eco_hmi(_req: bytes) -> bytes:
    return REP_HMI


# ── Enquadramento: bate com a tela? ───────────────────────────────────

def test_requisicao_reproduz_o_quadro_da_tela_byte_a_byte():
    req = m.build_read(1, m.FUNC_READ_HOLDING,
                       FT_MODBUS_DATA_ADDR, FT_MODBUS_DATA_REGS)
    assert req == REQ_HMI


def test_os_dois_crcs_da_tela_fecham():
    assert crc16_modbus(REQ_HMI[:-2]) == int.from_bytes(REQ_HMI[-2:], 'little')
    assert crc16_modbus(REP_HMI[:-2]) == int.from_bytes(REP_HMI[-2:], 'little')


def test_resposta_tem_29_bytes_no_estilo_padrao():
    # 30 seria o estilo HKVL-56 (endereço no lugar do bytecount). O 0x18 no
    # terceiro byte é bytecount, então o FA7155 fala Modbus PADRÃO.
    assert len(REP_HMI) == 29
    assert REP_HMI[2] == 0x18
    assert m.expected_response_len(
        m.FUNC_READ_HOLDING, FT_MODBUS_DATA_REGS, m.STYLE_STANDARD) == 29


def test_payload_decodifica_para_os_50_N_da_tela():
    vals = m.parse_wrench(REP_HMI, FT_MODBUS_DATA_REGS, m.STYLE_STANDARD)
    assert vals == (50.0,) * 6


def test_estilo_hkvl56_decodificaria_errado_este_quadro():
    # Guarda de regressão: usar o estilo errado não levanta erro, só devolve
    # números deslocados por um byte. É o tipo de bug que passa despercebido.
    vals = m.parse_wrench(REP_HMI, 24, m.STYLE_HKVL56)
    assert vals != (50.0,) * 6


# ── FtDevice.read_wrench ──────────────────────────────────────────────

def test_read_wrench_manda_o_pedido_certo_e_devolve_os_seis_eixos():
    linha = FakeLine(_eco_hmi)
    with linha.command_session() as (w, r):
        dev = m.FtDevice(m.FtModbusClient(w, r, slave_id=1, timeout_s=0.2))
        vals = dev.read_wrench()
    assert linha.pedidos == [REQ_HMI]
    assert vals == (50.0,) * 6


def test_read_wrench_acha_a_resposta_no_meio_do_stream_ST():
    from touch_pack.constants import FT_FRAME_HEADER
    lixo = _crc(FT_FRAME_HEADER + struct.pack('<6f', 1, 2, 3, .1, .2, .3))
    linha = FakeLine(_eco_hmi, ruido=lixo * 3)
    with linha.command_session() as (w, r):
        dev = m.FtDevice(m.FtModbusClient(w, r, slave_id=1, timeout_s=0.5))
        assert dev.read_wrench() == (50.0,) * 6


def test_read_wrench_nao_precisa_do_mapa_confirmado():
    """Ler é seguro; a trava do FT_MODBUS_MAP guarda só as ESCRITAS."""
    linha = FakeLine(_eco_hmi)
    with linha.command_session() as (w, r):
        dev = m.FtDevice(m.FtModbusClient(w, r, slave_id=1, timeout_s=0.2))
        dev.read_wrench()          # não levanta FtModbusMapUnconfirmed
    with pytest.raises(m.FtModbusMapUnconfirmed):
        m.FtDevice(m.FtModbusClient(lambda d: None, lambda n: b'')).set_zero()


# ── O driver de laço ──────────────────────────────────────────────────

def _roda_driver(linha, n_amostras=3, prazo=2.0, **kw):
    got = []
    ev = threading.Event()

    def cb(seq, t_us, vals):
        got.append((seq, t_us, vals))
        if len(got) >= n_amostras:
            ev.set()

    drv = FtPolledDriver(linha, slave_id=1, timeout_s=0.2, on_sample=cb, **kw)
    drv.start()
    ev.wait(prazo)
    # Fotografado ANTES do stop(), que legitimamente zera `connected`.
    drv.conectado_em_regime = drv.connected
    drv.stop()
    return drv, got


def test_driver_entrega_amostras_com_seq_crescente():
    drv, got = _roda_driver(FakeLine(_eco_hmi))
    assert len(got) >= 3
    assert [s for s, _, _ in got[:3]] == [0, 1, 2]
    assert all(v == (50.0,) * 6 for _, _, v in got)
    assert drv.ok >= 3 and drv.conectado_em_regime


def test_driver_abre_uma_sessao_por_amostra():
    """Segurar a sessão aberta faria todo comando da GUI falhar."""
    linha = FakeLine(_eco_hmi)
    drv, got = _roda_driver(linha)
    assert linha.sessoes >= len(got)


def test_driver_conta_timeout_sem_morrer():
    drv, got = _roda_driver(FakeLine(lambda req: b''), n_amostras=1, prazo=0.8)
    assert got == []
    assert drv.timeouts >= 1
    assert not drv.conectado_em_regime
    assert 'sem resposta' in drv.error


def test_driver_descarta_NaN_em_vez_de_envenenar_o_filtro():
    nan = _crc(b'\x01\x03\x18' + struct.pack('<6f', *([float('nan')] * 6)))
    drv, got = _roda_driver(FakeLine(lambda req: nan), n_amostras=1, prazo=0.6)
    assert got == []
    assert drv.bad_values >= 1


def test_comando_sempre_ganha_a_linha_do_laco():
    """Sem o handoff, um Set_Zero da GUI falharia de forma intermitente."""
    linha = FakeLine(_eco_hmi)
    drv = FtPolledDriver(linha, slave_id=1, timeout_s=0.2)
    drv.start()
    time.sleep(0.05)
    with drv.yield_line() as obtido:
        assert obtido, 'o comando não conseguiu a linha'
        # Com a linha na mão, abrir a sessão tem de funcionar SEMPRE: é isto
        # que o _run_command do nó faz.
        with linha.command_session() as (w, r):
            dev = m.FtDevice(m.FtModbusClient(w, r, slave_id=1, timeout_s=0.2))
            assert dev.read_wrench() == (50.0,) * 6
        time.sleep(0.05)
    drv.stop()
    assert drv.busy_skips >= 1           # o laço recuou em vez de estourar


def test_laco_nao_mata_o_comando_de_fome_sob_disputa():
    """20 comandos seguidos, todos têm de entrar."""
    linha = FakeLine(_eco_hmi)
    drv = FtPolledDriver(linha, slave_id=1, timeout_s=0.2)
    drv.start()
    try:
        for _ in range(20):
            with drv.yield_line(timeout_s=1.0) as obtido:
                assert obtido
    finally:
        drv.stop()


# ── Teto do modo polled ───────────────────────────────────────────────

def test_teto_polled_bate_com_a_conta_de_440_bits():
    assert ft_polled_max_rate_hz(115200) == pytest.approx(115200 / 440)
    assert ft_polled_max_rate_hz(115200) == pytest.approx(261.8, abs=0.1)


def test_polled_cabe_em_115200_e_o_stream_de_1kHz_nao():
    from touch_pack.constants import ft_max_rate_hz
    assert ft_polled_max_rate_hz(115200) > 200      # usável no flange
    assert ft_max_rate_hz(115200) < 1000            # stream a 1 kHz não cabe
