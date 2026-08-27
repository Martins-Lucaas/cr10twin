"""ft_polled.py — aquisição POLLED do FA7155 (o modo do cliente de fábrica).

O `ft_serial.py` implementa o outro caminho: o sensor cospe quadros "ST" de
28 bytes sozinho e o host só escuta. Este módulo implementa o caminho que o
HMI/cliente da FIBOS usa, em que o host é MESTRE e pergunta:

    host   -> 01 03 00 03 00 0C B5 CF          (holding 0x0003, 12 regs)
    sensor -> 01 03 18 <24 B = 6x float32 LE> 10 A3

Os dois quadros estão conferidos byte a byte em constants.py e servem de vetor
dourado no test_ft_polled.py.

POR QUE ISTO NÃO É UM TRANSPORTE NOVO
-------------------------------------
Polled-vs-stream é um MODO; serial-vs-TCP é um MEIO. São ortogonais: dá para
pollar pela 485 do flange (porta 60000) exatamente como pelo conversor USB. O
driver abaixo não abre porta nenhuma — ele dirige o `command_session()` que o
`LineTapMixin` já dá aos dois transportes. Consequência prática: polled passou
a funcionar pelo flange sem uma linha de código a mais em `ft_tcp.py`.

QUANDO ELE COMPENSA
-------------------
O stream a 1 Mbps entrega 1000 Hz e é o padrão. O polled custa, por amostra,
a requisição (8 B), a resposta (29 B) e os dois silêncios de 3,5 caracteres do
Modbus RTU: 440 bits, ou ~262 Hz a 115200 (ver `ft_polled_max_rate_hz`). Em
troca ele cabe em 115200, que é o único baud que a 485 da Yuejiang aceita —
ou seja, é ele que torna a rota do flange viável.

Vantagem lateral que o stream não tem: aqui a amostra é carimbada no instante
em que a resposta CHEGA, uma por transação. O `_stamp_frames` do stream precisa
retro-datar blocos inteiros pelo período nominal porque o SO entrega vários
quadros de uma vez; aqui não há bloco para desempacotar.
"""
from __future__ import annotations

import math
import threading
import time
from contextlib import contextmanager
from typing import Callable, Optional

from .constants import FT_MODBUS_SLAVE_ID, FT_MODBUS_TIMEOUT_S
from .ft_modbus import FtDevice, FtModbusClient, ModbusError, ModbusTimeout


class FtPolledDriver:
    """Lê os seis eixos em laço, perguntando. Mesma forma do FtSerialSource.

    `source` é qualquer transporte com `command_session()` — hoje o
    `FtSerialSource` e o `FtTcpSource`. Ele continua dono da porta e da thread
    de leitura; este driver só dirige transações por cima da derivação.

    O callback recebe ``(seq, t_us, (fx, fy, fz, mx, my, mz))``, idêntico ao
    do stream: quem consome não precisa saber por qual caminho a amostra veio.
    """

    def __init__(self, source,
                 slave_id: int = FT_MODBUS_SLAVE_ID,
                 timeout_s: float = FT_MODBUS_TIMEOUT_S,
                 interval_s: float = 0.0,
                 on_sample: Optional[Callable[[int, int, tuple], None]] = None):
        self._src = source
        self._slave = int(slave_id)
        self._timeout = float(timeout_s)
        # 0 = o mais rápido que a linha permitir. Um valor > 0 espaça as
        # transações, útil para deixar banda ao canal de comando.
        self._interval = max(float(interval_s), 0.0)
        self._on_sample = on_sample

        self.connected = False
        self.last_rx: float = 0.0
        self.error: str = ''
        # Contadores de saúde, no mesmo espírito dos do FtFrameParser. Ficam
        # AQUI e não no FtModbusClient porque o cliente é reconstruído a cada
        # amostra (a sessão abre e fecha para o canal de comando poder entrar).
        self.ok = 0
        self.timeouts = 0
        self.errors = 0
        self.bad_values = 0
        self.busy_skips = 0

        self._seq = 0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        # Arbitragem da linha entre o laço e o canal de comando. O lock é
        # segurado durante UMA transação; o evento impede o laço de pegar a
        # linha de novo enquanto um comando espera, senão o comando poderia
        # ficar preso indefinidamente atrás de um laço que nunca respira.
        self._line_lock = threading.Lock()
        self._yield_req = threading.Event()

    # ── Ciclo de vida ─────────────────────────────────────────────────
    def start(self) -> bool:
        if self._running:
            return True
        self._running = True
        self._thread = threading.Thread(
            target=self._worker, daemon=True, name='ft-polled')
        self._thread.start()
        return True

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            # Prazo maior que o timeout de uma transação: parar no meio de um
            # round-trip deixaria a sessão aberta e o próximo start() acharia
            # a linha ocupada.
            self._thread.join(timeout=self._timeout + 1.0)
            self._thread = None
        self.connected = False

    @contextmanager
    def yield_line(self, timeout_s: float = 2.0):
        """Toma a linha do laço para uma transação de comando.

        Sem isto o `Set_Zero` da GUI falharia de forma intermitente: ele e o
        laço disputam o mesmo `command_session()`, que não aninha, e quem
        perdesse levaria um RuntimeError. Aqui o comando SEMPRE ganha — ele
        pede a linha, espera a transação em voo terminar e só então entra.

        Perder uma amostra de força vale menos que perder um comando.
        """
        self._yield_req.set()
        obtido = self._line_lock.acquire(timeout=max(timeout_s, 0.0))
        try:
            yield obtido
        finally:
            if obtido:
                self._line_lock.release()
            self._yield_req.clear()

    # ── Laço ──────────────────────────────────────────────────────────
    def _worker(self) -> None:
        while self._running:
            t0 = time.monotonic()
            self._poll_once()
            if self._interval:
                folga = self._interval - (time.monotonic() - t0)
                if folga > 0:
                    time.sleep(folga)

    def _poll_once(self) -> None:
        if self._yield_req.is_set():
            # Um comando está esperando. Não basta ceder a vez uma vez: sem
            # esta guarda o laço voltaria a pegar o lock antes de o comando
            # conseguir, e o comando morreria de fome.
            self.busy_skips += 1
            time.sleep(0.005)
            return
        if not self._line_lock.acquire(timeout=0.5):
            self.busy_skips += 1
            return
        try:
            self._transact()
        finally:
            self._line_lock.release()

    def _transact(self) -> None:
        try:
            # A sessão abre e fecha a CADA amostra de propósito. Segurá-la
            # aberta faria todo comando da GUI (Set_Zero, taxa, baud) falhar
            # com "já existe uma sessão de comando aberta": `command_session`
            # não aninha. Abrir e fechar custa um lock e um bytearray.
            with self._src.command_session() as (write_fn, read_fn):
                client = FtModbusClient(write_fn, read_fn,
                                        slave_id=self._slave,
                                        timeout_s=self._timeout)
                vals = FtDevice(client).read_wrench()
        except RuntimeError:
            # O canal de comando pegou a linha primeiro. Ele é curto e ganha:
            # perder uma amostra vale menos que atrasar um Set_Zero.
            self.busy_skips += 1
            time.sleep(0.01)
            return
        except ModbusTimeout as exc:
            self.timeouts += 1
            self.connected = False
            self.error = str(exc)
            return
        except ModbusError as exc:
            self.errors += 1
            self.error = str(exc)
            return
        except Exception as exc:
            # Porta fechada pelo stop() do dono, replug, socket caído.
            self.errors += 1
            self.connected = False
            self.error = f'{type(exc).__name__}: {exc}'
            time.sleep(0.1)
            return

        # Mesma guarda do ft_serial: um NaN envenena o One-Euro do receiver
        # PARA SEMPRE (x_prev vira NaN e toda saída seguinte é NaN).
        if not all(math.isfinite(v) for v in vals):
            self.bad_values += 1
            return

        self.ok += 1
        self.connected = True
        self.error = ''
        self.last_rx = time.monotonic()
        seq = self._seq
        self._seq = (seq + 1) & 0xFFFFFFFF
        if self._on_sample is not None:
            # Carimbo no instante da CHEGADA. Sem retro-datação: uma transação
            # produz uma amostra, então não há bloco para desempacotar.
            t_us = int(time.perf_counter() * 1e6) & 0xFFFFFFFF
            self._on_sample(seq, t_us, tuple(vals))
