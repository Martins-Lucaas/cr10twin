"""ft_serial.py — Transporte RS485 da célula de 6 eixos FA7155 (modo ATIVO).

Diferenças que importam em relação ao lc_serial.py (XIAO + HX711):

* Não há firmware nosso no caminho. O FA7155 é um escravo RS485 que começa a
  cuspir quadros sozinho ao ser energizado ("send automatically", manual
  §4.1) — o PC só escuta. Um conversor USB↔RS485 (ZK-U485/CH340) faz a ponte.
* O quadro é BINÁRIO e de tamanho fixo (28 bytes), não uma linha ASCII: não dá
  para sincronizar por fim de linha. A ressincronização é por cabeçalho + CRC.
* O sensor NÃO carimba nem numera as amostras. `seq` e `t_us` são gerados
  aqui (ver `_stamp_frames`) — o dt sai do relógio do host, não do sensor.
* Os valores já saem em N e N·m, calibrados de fábrica. Não há slope/intercept
  a carregar: o que sobra do lado do host é o TARE.

Formato do quadro (manual §4.2, exemplo da página 6 conferido byte a byte):

    53 54 | fx fy fz mx my mz (6x float32 little-endian) | CRC16 lo hi
     2 B  |                24 B                          |     2 B

O CRC-16/MODBUS é calculado sobre os 26 PRIMEIROS bytes (o cabeçalho ENTRA no
cálculo) e transmitido em little-endian.
"""
from __future__ import annotations

import math
import struct
import time
from typing import Callable, Optional

from .ft_cmd_channel import LineTapMixin
from .line_source import SERIAL_OK as _SERIAL_OK, SerialSource, list_ports
from .constants import (
    FT_FRAME_HEADER,
    FT_FRAME_LEN,
    FT_NOMINAL_RATE_HZ,
    FT_SERIAL_BAUD,
    FT_USB_VIDS,
)

# Teto do buffer de recepção antes de descartar por sujeira. A 1 kHz o link
# entrega 28 kB/s; 8 kB são ~285 quadros — se encheu sem um quadro válido
# sair, o que está na linha não é este protocolo.
_MAX_BUF = 8192

_PAYLOAD = struct.Struct('<6f')


def crc16_modbus(data: bytes) -> int:
    """CRC-16/MODBUS (init 0xFFFF, polinômio refletido 0xA001).

    O manual traz a versão por tabela de 256 entradas; esta é a mesma conta
    bit a bit. 26 bytes x 8 iterações x 1 kHz ~ 208 k operações/s — irrelevante
    perto do custo de uma leitura na serial, e cabe em cinco linhas.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def detect_ft_serial_port() -> Optional[str]:
    """Porta do conversor USB-RS485, achada pelo VID.

    O VID é o do CHIP da ponte (CH340 e afins), não do sensor: o FA7155 não
    aparece na USB. Isso significa que QUALQUER conversor USB-serial genérico
    no mesmo PC é candidato — se houver mais de um, passe a porta explícita
    pelo parâmetro `ft_serial_port` do nó em vez de confiar no auto-detect.
    """
    if not _SERIAL_OK:
        return None
    for p in list_ports.comports():
        if p.vid in FT_USB_VIDS:
            return p.device
    return None


class FtFrameParser:
    """Máquina de ressincronização do fluxo de bytes para quadros de 6 eixos.

    Escutar um talker que já está falando significa entrar no meio de um
    quadro. Achar o cabeçalho não basta: 0x53 0x54 aparece dentro dos floats
    de dados com frequência banal (é só um valor de força específico), então
    quem DECIDE se o alinhamento está certo é o CRC. Cabeçalho falso => anda
    um byte e tenta de novo, em vez de pular 28 e picotar o quadro seguinte.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        # Contadores de saúde do link, lidos pelo nó no relatório periódico.
        self.crc_errors = 0        # quadro com cabeçalho e CRC errado
        self.resyncs = 0           # trechos de lixo descartados
        self.dropped_bytes = 0
        self.bad_values = 0        # CRC ok mas float não finito

    def feed(self, chunk: bytes) -> list[tuple[float, ...]]:
        """Consome bytes crus e devolve os quadros COMPLETOS que fecharam."""
        out: list[tuple[float, ...]] = []
        buf = self._buf
        buf += chunk
        n = len(buf)
        i = 0
        while True:
            j = buf.find(FT_FRAME_HEADER, i)
            if j < 0:
                # Nada de cabeçalho: guarda só o último byte, que pode ser o
                # 0x53 de um cabeçalho partido entre duas leituras.
                keep = 1 if n else 0
                if (n - keep) > i:
                    self.dropped_bytes += (n - keep) - i
                i = max(n - keep, i)
                break
            if j > i:
                self.dropped_bytes += j - i
                self.resyncs += 1
            if n - j < FT_FRAME_LEN:
                i = j            # quadro ainda incompleto — espera mais bytes
                break
            frame = bytes(buf[j:j + FT_FRAME_LEN])
            if crc16_modbus(frame[:-2]) != int.from_bytes(frame[-2:], 'little'):
                self.crc_errors += 1
                i = j + 1        # era dado que parecia cabeçalho
                continue
            vals = _PAYLOAD.unpack(frame[2:26])
            i = j + FT_FRAME_LEN
            # Um NaN envenena o One-Euro do receiver PARA SEMPRE (x_prev vira
            # NaN e toda saída seguinte é NaN até reiniciar o nó) — mesma
            # guarda do lc_serial.py, aqui aplicada aos seis eixos.
            if not all(math.isfinite(v) for v in vals):
                self.bad_values += 1
                continue
            out.append(vals)
        del buf[:i]
        if len(buf) > _MAX_BUF:
            self.dropped_bytes += len(buf)
            self.resyncs += 1
            del buf[:]
        return out


class FtSerialSource(LineTapMixin, SerialSource):
    """Leitor do FA7155 em thread de fundo.

    Mesma API do LoadCellSerialSource (start/stop/connected/last_rx/error),
    para que o nó receptor tenha a mesma forma dos dois lados — hoje porque as
    duas herdam o mesmo `SerialSource`.

    O callback recebe ``(seq, t_us, (fx, fy, fz, mx, my, mz))`` — forças em N,
    momentos em N·m, no referencial da figura 2 do manual (Fz+ saindo da face
    da ferramenta).
    """

    _THREAD_NAME = 'ft-serial'

    def __init__(self, port: Optional[str] = None,
                 baud: int = FT_SERIAL_BAUD,
                 rate_hz: float = FT_NOMINAL_RATE_HZ,
                 on_sample: Optional[
                     Callable[[int, int, tuple], None]] = None):
        self._lifecycle_init()
        self._port_req = port
        self._baud = int(baud)
        self._period_us = 1e6 / max(float(rate_hz), 1.0)
        self._on_sample = on_sample
        self.port: Optional[str] = None
        self.parser = FtFrameParser()
        self._seq = 0
        self._tap_init()

    def _detect_port(self) -> Optional[str]:
        return detect_ft_serial_port()

    def _absent_message(self) -> str:
        return ('conversor USB-RS485 ausente (VIDs aceitos: '
                + ', '.join(f'0x{v:04X}' for v in FT_USB_VIDS) + ')')

    def _on_chunk(self, data: bytes) -> None:
        self._tap_feed(data)
        frames = self.parser.feed(data)
        if not frames:
            return
        self.last_rx = time.monotonic()
        self._stamp_frames(frames)

    def _line_write(self, data: bytes) -> None:
        """Põe bytes na 485 (canal de comando Modbus — ver ft_modbus).

        A 485 é half-duplex: escrever enquanto o sensor fala colide. Na
        prática o conversor arbitra por si (o driver só habilita o TX
        durante o write), e o sensor reenvia o quadro perdido no ciclo
        seguinte — o parser já trata isso como resync.
        """
        self._write_line(data, 'porta serial do FA7155 não está aberta')

    def _stamp_frames(self, frames: list[tuple[float, ...]]) -> None:
        """Numera e carimba os quadros de UMA leitura.

        O sensor não manda relógio nem contador, então o carimbo é do host. Uma
        leitura pode trazer k quadros de uma vez (o SO entrega o buffer em
        blocos): dar a todos o mesmo instante faria dt=0 e o One-Euro cairia na
        taxa nominal justamente nesses blocos. Os quadros são então
        retro-datados pelo período nominal, que é o intervalo real com que o
        sensor os produziu — o erro fica no ATRASO absoluto (jitter do USB),
        não no dt, que é o que o filtro consome.

        Compartilhado com `FtTcpSource`, que importa este método: o raciocínio
        não depende do transporte, só do sensor.
        """
        t_now_us = time.perf_counter() * 1e6
        k = len(frames)
        cb = self._on_sample
        for idx, vals in enumerate(frames):
            t_us = int(t_now_us - (k - 1 - idx) * self._period_us) & 0xFFFFFFFF
            seq = self._seq
            self._seq = (seq + 1) & 0xFFFFFFFF
            if cb is not None:
                cb(seq, t_us, vals)
