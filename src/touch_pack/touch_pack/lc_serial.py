"""lc_serial.py — Transporte USB da célula axial de 100 kg (XIAO + HX711).

Do outro lado do cabo há firmware NOSSO (`sensors/ForceDriver/src/main.cpp`),
e é isso que separa este transporte do `ft_serial.py` da FA7155:

* O quadro é uma LINHA ASCII, não 28 bytes binários — a ressincronização é o
  `\\n`, não cabeçalho + CRC. Entrar no meio de uma linha custa uma linha.
* O sensor NUMERA e CARIMBA. `seq` e `t_us` vêm do MCU, então o `dt` que o
  filtro consome é o do relógio da célula e não o do host — que é o motivo de
  a taxa do HX711 (10 ou 80 Hz, pino RATE) não precisar ser configurada em
  lugar nenhum.
* Há canal de VOLTA: o byte `'Z'` refaz o zero de boot do firmware. É o único
  comando, e é o que o `/load_cell/rezero` vira no fio.
* O valor é TENSÃO da ponte (V, já ×PGA), não newton. A conversão para força
  depende da calibração e mora em `constants.lc_force_n`.

Formato (espelhado no main.cpp):

    F,<seq>,<t_us>,<v_sensor>\\n     amostra
    #<qualquer coisa>\\n             heartbeat de diagnóstico (0,5 Hz)

O `t_us` é o `micros()` do MCU, uint32 COM WRAPAROUND de propósito — quem
calcula o dt subtrai em módulo 2³².
"""
from __future__ import annotations

import math
import threading
import time
from typing import Callable, Optional

try:
    import serial
    from serial.tools import list_ports
    _SERIAL_OK = True
except Exception:  # pragma: no cover - pyserial ausente
    serial = None
    list_ports = None
    _SERIAL_OK = False

from .constants import LC_SERIAL_BAUD, LC_USB_VIDS

# Intervalo entre tentativas de achar/abrir a porta (hot-plug).
_RETRY_S = 2.0

# Teto do buffer antes de descartar por sujeira. A 80 Hz a célula entrega
# ~2,4 kB/s; 8 kB sem um `\n` significa que o que está na linha não é este
# protocolo (placa em modo SERIAL_TEST, monitor de outro programa, lixo de
# boot do ESP).
_MAX_BUF = 8192


def detect_lc_serial_port() -> Optional[str]:
    """Porta do XIAO, achada pelo VID da Espressif.

    Aqui o VID é o do PRÓPRIO MCU (USB CDC nativo do ESP32S3), não o de um
    conversor genérico como no FA7155 — então a detecção é bem mais específica
    e raramente pega o dispositivo errado. Ainda assim: dois ESP32 no mesmo PC
    são dois candidatos, e aí passe a porta pelo parâmetro `lc_serial_port`.
    """
    if not _SERIAL_OK:
        return None
    for p in list_ports.comports():
        if p.vid in LC_USB_VIDS:
            return p.device
    return None


class LcLineParser:
    """Fluxo de bytes → amostras `(seq, t_us, v_sensor)`.

    Tolerante por construção: a primeira linha de uma sessão quase sempre está
    partida (a porta abre no meio de uma transmissão em curso) e o ESP cospe
    texto de boot antes do primeiro `F`. Nada disso é erro — o que conta é a
    linha que fecha e casa com o formato.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        # Contadores de saúde, lidos pelo nó no relatório periódico.
        self.bad_lines = 0        # linha com prefixo F e campos ilegíveis
        self.heartbeats = 0       # linhas '#' (o firmware está vivo)
        self.dropped_bytes = 0
        self.bad_values = 0       # número legível mas não finito

    def feed(self, chunk: bytes) -> list[tuple[int, int, float]]:
        """Consome bytes crus e devolve as amostras das linhas COMPLETAS."""
        out: list[tuple[int, int, float]] = []
        buf = self._buf
        buf += chunk
        while True:
            j = buf.find(b'\n')
            if j < 0:
                break
            line = bytes(buf[:j])
            del buf[:j + 1]
            s = self._parse(line)
            if s is not None:
                out.append(s)
        if len(buf) > _MAX_BUF:
            self.dropped_bytes += len(buf)
            del buf[:]
        return out

    def _parse(self, line: bytes) -> Optional[tuple[int, int, float]]:
        txt = line.decode('ascii', errors='replace').strip()
        if not txt or txt.startswith('#'):
            if txt:
                self.heartbeats += 1
            return None
        parts = txt.split(',')
        if parts[0] != 'F':
            # Texto de boot do ESP, eco do monitor, meia-linha da abertura da
            # porta. Comum e inofensivo — não conta como erro, senão o
            # relatório de saúde acusa link sujo toda vez que alguém repluga.
            self.dropped_bytes += len(line)
            return None
        if len(parts) != 4:
            self.bad_lines += 1
            return None
        try:
            seq = int(parts[1]) & 0xFFFFFFFF
            t_us = int(parts[2]) & 0xFFFFFFFF
            v = float(parts[3])
        except ValueError:
            self.bad_lines += 1
            return None
        # Um NaN envenena o One-Euro do receiver PARA SEMPRE (x_prev vira NaN
        # e toda saída seguinte é NaN até reiniciar o nó). Barrado aqui, na
        # única porta de entrada.
        if not math.isfinite(v):
            self.bad_values += 1
            return None
        return seq, t_us, v


class LoadCellSerialSource:
    """Leitor do XIAO em thread de fundo.

    Mesma API do `FtSerialSource` (start/stop/connected/last_rx/error/port/
    parser), para que os dois receivers tenham a mesma forma — mais o
    `send_command`, que a FA7155 não tem equivalente pela serial.

    O callback recebe ``(seq, t_us, v_sensor)``: contador e `micros()` do MCU,
    tensão da ponte em volts no domínio ×PGA.
    """

    def __init__(self, port: Optional[str] = None,
                 baud: int = LC_SERIAL_BAUD,
                 on_sample: Optional[
                     Callable[[int, int, float], None]] = None):
        self._port_req = port
        self._baud = int(baud)
        self._on_sample = on_sample
        self.port: Optional[str] = None
        self.connected = False
        # time.monotonic() da última amostra válida (0.0 = nunca).
        self.last_rx: float = 0.0
        self.error: str = ''
        self.parser = LcLineParser()
        self._running = False
        self._ser = None
        self._thread: Optional[threading.Thread] = None

    def start(self) -> bool:
        """Arma a thread. False só se pyserial não existe — a ausência da
        placa na USB não é falha: a thread fica tentando."""
        if not _SERIAL_OK:
            self.error = 'pyserial ausente'
            return False
        if self._running:
            return True
        self._running = True
        self._thread = threading.Thread(
            target=self._worker, daemon=True, name='lc-serial')
        self._thread.start()
        return True

    def stop(self) -> None:
        self._running = False
        # Fecha a porta por fora para destravar o read() da thread.
        ser = self._ser
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def send_command(self, data: bytes) -> None:
        """Manda bytes para o firmware (hoje só `b'Z'`, o re-zero).

        A USB é full-duplex e o firmware lê um byte por volta do loop, então
        escrever no meio do stream não colide nem perde amostra.
        """
        ser = self._ser
        if ser is None:
            raise RuntimeError('porta serial do XIAO não está aberta')
        ser.write(data)
        ser.flush()

    def _worker(self) -> None:
        while self._running:
            port = self._port_req or detect_lc_serial_port()
            if port is None:
                self.error = ('XIAO ESP32S3 ausente na USB (VIDs aceitos: '
                              + ', '.join(f'0x{v:04X}' for v in LC_USB_VIDS)
                              + ')')
                time.sleep(_RETRY_S)
                continue
            try:
                # timeout curto: o read(1) volta rápido quando a linha cala, e
                # o laço reavalia self._running em vez de travar no stop().
                ser = serial.Serial(port, self._baud, timeout=0.2)
            except Exception as exc:
                self.error = str(exc)
                time.sleep(_RETRY_S)
                continue
            self._ser = ser
            self.port = port
            self.connected = True
            self.error = ''
            try:
                self._read_loop(ser)
            except Exception as exc:
                # Desconexão (replug) ou porta fechada pelo stop().
                self.error = str(exc)
            finally:
                self.connected = False
                self._ser = None
                try:
                    ser.close()
                except Exception:
                    pass
            if self._running:
                time.sleep(_RETRY_S)

    def _read_loop(self, ser) -> None:
        while self._running:
            # read(1) bloqueante + o que já estiver no buffer: entrega a menor
            # latência possível sem virar espera ocupada.
            data = ser.read(1)
            if not data:
                continue
            waiting = getattr(ser, 'in_waiting', 0)
            if waiting:
                data += ser.read(waiting)
            samples = self.parser.feed(data)
            if not samples:
                continue
            self.last_rx = time.monotonic()
            cb = self._on_sample
            if cb is not None:
                for seq, t_us, v in samples:
                    cb(seq, t_us, v)
