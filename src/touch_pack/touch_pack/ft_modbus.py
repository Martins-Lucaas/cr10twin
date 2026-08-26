"""ft_modbus.py — canal de COMANDO Modbus RTU do FA7155.

Por que este módulo existe: até 26/08/2026 o repo tratava a célula como um
talker passivo ("ele só fala", `ft_receiver_node.py`), e o `ft_serial.py`
nunca escrevia na porta. A análise do cliente de fábrica da FIBOS
(`Six_Axis_FT.exe`, Qt5) mostrou que isso está errado — ele implementa uma
classe `Modbus_Rtu` sobre `QModbusRtuSerialMaster` e manda ao sensor:

    Set_Zero          zera os seis eixos NO HARDWARE  (reset 0/1 success|fail)
    Send_Frequency    muda a taxa de saída            (só após religar)
    Send_ModBus_ID    muda o node ID do escravo       (só após religar)
    Send_Baud_rate    muda o baud da 485              (só após religar)
    StartReading      liga o stream                   (start 0/1 success|fail)
    stopReading       desliga o stream

O stream "ST" e o Modbus convivem na MESMA linha 485: o quadro de dados usa
cabeçalho 0x53 0x54 ("ST") com CRC-16/MODBUS, e o comando usa enquadramento
Modbus RTU normal (slave, função, dados, CRC). Só o CRC é compartilhado.

O que este módulo NÃO faz: escolher endereços de registrador. Eles não estão
aqui nem em `constants.py` como número confiável — ver FT_MODBUS_MAP e
FT_MODBUS_MAP_CONFIRMED lá. Enquanto o mapa não for capturado da bancada,
`FtDevice` recusa toda ESCRITA. O enquadramento abaixo é o Modbus padrão e
vale independentemente do mapa.

Divisão de responsabilidade: este módulo não abre porta nenhuma. Quem tem o
socket ou o `serial.Serial` é o `ft_receiver`, e passa aqui um par de
callables (`write_fn`, `read_fn`). Isso mantém o mesmo código válido para o
conversor USB da mesa e para a 485 do flange (porta 60000), e permite testar
o protocolo inteiro sem hardware.
"""
from __future__ import annotations

import struct
import time
from typing import Callable, Optional, Sequence

from .constants import (
    FT_MODBUS_MAP,
    FT_MODBUS_MAP_CONFIRMED,
    FT_MODBUS_SLAVE_ID,
    FT_MODBUS_TIMEOUT_S,
)
from .ft_serial import crc16_modbus

# Códigos de função usados pelo cliente de fábrica (QModbusClient::send*).
FUNC_READ_HOLDING   = 0x03
FUNC_READ_INPUT     = 0x04
FUNC_WRITE_SINGLE   = 0x06
FUNC_WRITE_MULTIPLE = 0x10

# Texto das exceções Modbus padrão, para o erro chegar legível na GUI em vez
# de "código 2".
_EXCEPTION_TEXT = {
    0x01: 'função ilegal',
    0x02: 'endereço de registrador ilegal',
    0x03: 'valor ilegal',
    0x04: 'falha no dispositivo escravo',
    0x05: 'acknowledge (ocupado, resposta longa)',
    0x06: 'escravo ocupado',
    0x08: 'erro de paridade na memória',
    0x0A: 'gateway sem caminho',
    0x0B: 'gateway sem resposta do alvo',
}


class ModbusError(Exception):
    """Base de tudo que pode dar errado numa transação."""


class ModbusTimeout(ModbusError):
    """Nenhuma resposta válida dentro do prazo."""


class ModbusCrcError(ModbusError):
    """Resposta com o tamanho certo e CRC errado."""


class ModbusExceptionResponse(ModbusError):
    """O escravo respondeu, mas com função|0x80 e um código de exceção."""

    def __init__(self, func: int, code: int):
        self.func = func
        self.code = code
        super().__init__(
            f'exceção Modbus 0x{code:02X} '
            f'({_EXCEPTION_TEXT.get(code, "desconhecida")}) '
            f'na função 0x{func:02X}')


class FtModbusMapUnconfirmed(ModbusError):
    """Escrita pedida antes de o mapa de registradores ser capturado.

    Não é pedantismo: escrever no endereço errado de um sensor de força pode
    trocar o node ID (e você perde o escravo) ou cair num baud que o host não
    fala mais. Ver o bloco FT_MODBUS_MAP em constants.py para o procedimento
    de captura.
    """


def _append_crc(payload: bytes) -> bytes:
    """CRC-16/MODBUS no fim, little-endian — igual ao quadro de stream."""
    return payload + struct.pack('<H', crc16_modbus(payload))


def _check_crc(frame: bytes) -> bool:
    return crc16_modbus(frame[:-2]) == int.from_bytes(frame[-2:], 'little')


def build_read(slave: int, func: int, addr: int, count: int) -> bytes:
    """Requisição de leitura (0x03 holding / 0x04 input)."""
    if func not in (FUNC_READ_HOLDING, FUNC_READ_INPUT):
        raise ValueError(f'função de leitura inválida: 0x{func:02X}')
    if not 1 <= count <= 125:
        raise ValueError(f'count fora de 1..125: {count}')
    return _append_crc(struct.pack('>BBHH', slave, func, addr, count))


def build_write_single(slave: int, addr: int, value: int) -> bytes:
    """Escrita de UM registrador (0x06)."""
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f'valor não cabe em 16 bits: {value}')
    return _append_crc(
        struct.pack('>BBHH', slave, FUNC_WRITE_SINGLE, addr, value))


def build_write_multiple(slave: int, addr: int,
                         values: Sequence[int]) -> bytes:
    """Escrita de N registradores (0x10) — usada quando o parâmetro é de 32
    bits (taxa, baud) e ocupa dois registradores consecutivos."""
    vals = list(values)
    if not 1 <= len(vals) <= 123:
        raise ValueError(f'quantidade fora de 1..123: {len(vals)}')
    body = struct.pack('>BBHHB', slave, FUNC_WRITE_MULTIPLE, addr,
                       len(vals), len(vals) * 2)
    for v in vals:
        if not 0 <= v <= 0xFFFF:
            raise ValueError(f'valor não cabe em 16 bits: {v}')
        body += struct.pack('>H', v)
    return _append_crc(body)


# Estilos de resposta de leitura. O Modbus padrão devolve um BYTECOUNT de 1
# byte antes dos dados; o HKVL-56 devolve, no lugar dele, o ENDEREÇO INICIAL
# de 2 bytes — e conta o campo "number of reads" em BYTES, não em
# registradores. Ver a nota do §4.2 do manual do sensor:
#
#   "The slave response frame contains: 1 byte of slave machine code, 1 byte
#    of function code, 2 bytes of register-type starting address, 24 bytes of
#    six-dimensional force sensor data values (6 float-type data items), and
#    2 bytes of CRC-16/Modbus checksum."
#
# 1+1+2+24+2 = 30 bytes, contra os 29 do padrão. Não é erro de tradução: os
# dois CRCs de exemplo do manual fecham com esta leitura, e os floats de
# exemplo (00 00 48 41 -> 12,5) confirmam o resto do enquadramento.
STYLE_STANDARD = 'standard'
STYLE_HKVL56   = 'hkvl56'


def expected_response_len(func: int, count: int,
                          style: str = STYLE_STANDARD) -> int:
    """Tamanho EXATO da resposta normal, que é o que permite achá-la no meio
    do stream: sabendo quantos bytes esperar, a janela candidata é testada
    pelo CRC em vez de por heurística de cabeçalho.

    `count` segue a convenção do estilo: registradores no padrão, BYTES no
    HKVL-56.
    """
    if func in (FUNC_READ_HOLDING, FUNC_READ_INPUT):
        if style == STYLE_HKVL56:
            return 4 + count + 2          # slave+func+addr(2) .. +CRC
        return 3 + count * 2 + 2          # slave+func+bytecount .. +CRC
    if func in (FUNC_WRITE_SINGLE, FUNC_WRITE_MULTIPLE):
        return 8                          # eco de slave,func,addr,valor/qtd
    raise ValueError(f'função sem tamanho conhecido: 0x{func:02X}')


def find_response(buf: bytes, slave: int, func: int,
                  exp_len: int) -> Optional[tuple[bytes, int]]:
    """Procura em `buf` a resposta desta transação. Devolve (quadro, fim).

    O stream "ST" continua chegando enquanto o comando é respondido, então a
    resposta vem cercada de quadros de dados. A âncora é o par (slave, func) e
    o juiz é o CRC — mesma lógica do `FtFrameParser`, pelo mesmo motivo: o
    byte do escravo aparece dentro de floats com frequência banal.

    A resposta de EXCEÇÃO (func|0x80) tem 5 bytes e é procurada junto, senão
    um pedido recusado só apareceria como timeout.
    """
    n = len(buf)
    for i in range(n):
        if buf[i] != slave:
            continue
        if i + 2 <= n and buf[i + 1] == (func | 0x80):
            if i + 5 <= n and _check_crc(buf[i:i + 5]):
                return bytes(buf[i:i + 5]), i + 5
            continue
        if buf[i + 1:i + 2] != bytes((func,)):
            continue
        if i + exp_len <= n and _check_crc(buf[i:i + exp_len]):
            return bytes(buf[i:i + exp_len]), i + exp_len
    return None


def read_payload(frame: bytes, count: int,
                 style: str = STYLE_STANDARD) -> bytes:
    """Devolve só os bytes de dados de uma resposta 0x03/0x04."""
    if style == STYLE_HKVL56:
        return frame[4:4 + count]
    n_bytes = frame[2]
    if n_bytes != count * 2:
        raise ModbusError(
            f'bytecount inesperado: {n_bytes}, esperado {count * 2}')
    return frame[3:3 + n_bytes]


def parse_read_response(frame: bytes, count: int,
                        style: str = STYLE_STANDARD) -> list[int]:
    """Extrai os registradores de 16 bits de uma resposta 0x03/0x04."""
    data = read_payload(frame, count, style)
    n = len(data) // 2
    return list(struct.unpack(f'>{n}H', data[:n * 2]))


def parse_wrench(frame: bytes, count: int = 24,
                 style: str = STYLE_HKVL56) -> tuple:
    """Os seis eixos de uma resposta de leitura, em float32 LITTLE-endian.

    O manual é explícito ("The parsed collected data is in little-endian
    format") e o exemplo fecha: Fx = 00 00 48 41 -> 12,5 N. É o MESMO
    formato do quadro de stream "ST" (`ft_serial._PAYLOAD`), o que é uma
    coincidência feliz: o resto da cadeia não precisa saber por qual dos dois
    caminhos a amostra chegou.
    """
    data = read_payload(frame, count, style)
    if len(data) < 24:
        raise ModbusError(
            f'resposta com {len(data)} bytes de dados, esperado 24')
    return struct.unpack('<6f', data[:24])


class FtModbusClient:
    """Mestre Modbus RTU sobre um transporte que outro dono já abriu.

    `write_fn(data)` põe bytes na linha; `read_fn(max_bytes)` devolve o que
    houver AGORA (pode ser b'' — não bloqueia até encher). O cliente faz o
    laço de prazo por cima, exatamente como o `_read_loop` do transporte faz
    com o `recv` curto.
    """

    def __init__(self,
                 write_fn: Callable[[bytes], None],
                 read_fn: Callable[[int], bytes],
                 slave_id: int = FT_MODBUS_SLAVE_ID,
                 timeout_s: float = FT_MODBUS_TIMEOUT_S,
                 style: str = STYLE_STANDARD):
        self._write = write_fn
        self._read = read_fn
        self.slave_id = int(slave_id)
        self.timeout_s = float(timeout_s)
        self.style = style
        # Contadores de saúde, lidos pela GUI junto com os do stream.
        self.tx = 0
        self.rx = 0
        self.timeouts = 0
        self.crc_errors = 0

    def transact(self, request: bytes, func: int, exp_len: int) -> bytes:
        """Manda uma requisição e devolve o quadro de resposta validado."""
        self._write(request)
        self.tx += 1
        buf = bytearray()
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            chunk = self._read(512)
            if chunk:
                buf += chunk
                hit = find_response(bytes(buf), self.slave_id, func, exp_len)
                if hit is not None:
                    frame, _ = hit
                    self.rx += 1
                    if frame[1] == (func | 0x80):
                        raise ModbusExceptionResponse(func, frame[2])
                    return frame
                # Buffer sem limite viraria vazamento se o escravo nunca
                # responder: o stream entrega ~700 B/s e o prazo é curto, mas
                # a poda mantém o pior caso constante.
                if len(buf) > 4096:
                    del buf[:len(buf) - 1024]
            else:
                time.sleep(0.002)
        self.timeouts += 1
        raise ModbusTimeout(
            f'sem resposta do escravo {self.slave_id} para a função '
            f'0x{func:02X} em {self.timeout_s:.2f} s')

    def read_registers(self, addr: int, count: int,
                       func: int = FUNC_READ_HOLDING) -> list[int]:
        frame = self.read_frame(addr, count, func)
        return parse_read_response(frame, count, self.style)

    def read_frame(self, addr: int, count: int,
                   func: int = FUNC_READ_HOLDING) -> bytes:
        """Resposta crua — quem precisa dos bytes (parse_wrench) usa esta."""
        req = build_read(self.slave_id, func, addr, count)
        return self.transact(req, func,
                             expected_response_len(func, count, self.style))

    def write_register(self, addr: int, value: int) -> None:
        req = build_write_single(self.slave_id, addr, value)
        self.transact(req, FUNC_WRITE_SINGLE,
                      expected_response_len(FUNC_WRITE_SINGLE, 1, self.style))

    def write_registers(self, addr: int, values: Sequence[int]) -> None:
        req = build_write_multiple(self.slave_id, addr, values)
        self.transact(req, FUNC_WRITE_MULTIPLE,
                      expected_response_len(FUNC_WRITE_MULTIPLE, len(values),
                                            self.style))


def _u32_to_regs(value: int) -> list[int]:
    """32 bits em dois registradores, palavra alta primeiro (big-endian de
    palavra, que é a convenção do Modbus e a que o `>` do struct já usa)."""
    return [(value >> 16) & 0xFFFF, value & 0xFFFF]


def _regs_to_u32(regs: Sequence[int]) -> int:
    return ((regs[0] & 0xFFFF) << 16) | (regs[1] & 0xFFFF)


class FtDevice:
    """Os comandos do cliente de fábrica, com nome de gente.

    Cada método corresponde a um slot do `Six_Axis_FT.exe` (ver
    `fibos-factory-client-analysis`). TODA ESCRITA passa por `_require_map`:
    sem o mapa de registradores confirmado na bancada, o método levanta
    `FtModbusMapUnconfirmed` em vez de escrever num endereço adivinhado.
    """

    def __init__(self, client: FtModbusClient):
        self.client = client

    @staticmethod
    def _require_map(*keys: str) -> None:
        if not FT_MODBUS_MAP_CONFIRMED:
            raise FtModbusMapUnconfirmed(
                'mapa de registradores Modbus do FA7155 ainda não confirmado '
                'na bancada — nenhuma escrita será enviada. Capture os '
                'comandos do cliente de fábrica na serial, preencha '
                'FT_MODBUS_MAP em constants.py e ponha '
                'FT_MODBUS_MAP_CONFIRMED = True.')
        faltando = [k for k in keys if FT_MODBUS_MAP.get(k) is None]
        if faltando:
            raise FtModbusMapUnconfirmed(
                f'FT_MODBUS_MAP sem endereço para: {", ".join(faltando)}')

    # ── Comandos de escrita ───────────────────────────────────────────
    def set_zero(self) -> None:
        """`Set_Zero` — zera os seis eixos NO SENSOR.

        Diferente do tare do `ft_receiver`, que é do host: este some com o
        offset na origem, e sobrevive ao reinício do nó.
        """
        self._require_map('zero')
        self.client.write_register(FT_MODBUS_MAP['zero'],
                                   FT_MODBUS_MAP.get('zero_value', 1))

    def set_output_rate_hz(self, hz: int) -> None:
        """`Send_Frequency`. Só passa a valer depois de religar o sensor."""
        self._require_map('rate')
        self.client.write_registers(FT_MODBUS_MAP['rate'],
                                    _u32_to_regs(int(hz)))

    def set_node_id(self, node_id: int) -> None:
        """`Send_ModBus_ID`. Só vale após religar — e o escravo passa a
        responder no ID NOVO, então o host tem de acompanhar."""
        if not 1 <= int(node_id) <= 247:
            raise ValueError(f'node ID fora de 1..247: {node_id}')
        self._require_map('node_id')
        self.client.write_register(FT_MODBUS_MAP['node_id'], int(node_id))

    def set_baud(self, baud: int) -> None:
        """`Send_Baud_rate`. Só vale após religar, e o host tem de reabrir a
        porta no baud novo."""
        self._require_map('baud')
        self.client.write_registers(FT_MODBUS_MAP['baud'],
                                    _u32_to_regs(int(baud)))

    def start_stream(self) -> None:
        """`StartReading` — liga a emissão contínua dos quadros "ST"."""
        self._require_map('stream')
        self.client.write_register(FT_MODBUS_MAP['stream'],
                                   FT_MODBUS_MAP.get('stream_on', 1))

    def stop_stream(self) -> None:
        """`stopReading`. Útil antes de uma rajada de comandos: sem o stream
        no meio, a resposta chega limpa."""
        self._require_map('stream')
        self.client.write_register(FT_MODBUS_MAP['stream'],
                                   FT_MODBUS_MAP.get('stream_off', 0))

    # ── Leitura ───────────────────────────────────────────────────────
    # Ler é seguro mesmo com o mapa por confirmar: no pior caso o escravo
    # devolve exceção 0x02 (endereço ilegal), que não altera nada nele.
    def read_device_id(self) -> Optional[int]:
        """`Device ID` do painel do cliente de fábrica."""
        addr = FT_MODBUS_MAP.get('device_id')
        if addr is None:
            return None
        return self.client.read_registers(addr, 1)[0]

    def read_output_rate_hz(self) -> Optional[int]:
        addr = FT_MODBUS_MAP.get('rate')
        if addr is None:
            return None
        return _regs_to_u32(self.client.read_registers(addr, 2))

    def probe(self) -> dict:
        """Varre o que o mapa conhecer e devolve só o que respondeu.

        Serve de teste de vida do canal de comando: se NADA responde, o
        problema é a linha (ou o node ID), não o registrador.
        """
        out: dict = {}
        for nome, fn in (('device_id', self.read_device_id),
                         ('rate_hz', self.read_output_rate_hz)):
            try:
                v = fn()
            except ModbusError:
                continue
            if v is not None:
                out[nome] = v
        return out
