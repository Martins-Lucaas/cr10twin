"""line_source.py — o ciclo de vida que os três transportes compartilham.

`FtSerialSource` (FA7155 na USB-RS485), `LoadCellSerialSource` (XIAO na USB) e
`FtTcpSource` (FA7155 pela 60000 do controlador) resolvem o MESMO problema de
disponibilidade: o dispositivo pode não estar lá quando o nó sobe, pode sumir
no meio de um ensaio (replug, cabo, queda de rede) e tem de voltar sozinho
quando reaparece — sem que o nó ROS saiba de nada disso.

A resposta é uma só nos três: uma thread de fundo que tenta abrir, lê até
falhar, fecha, espera e tenta de novo; mais um par `connected`/`error` que a
GUI lê para dizer ao operador o que está acontecendo. Só três coisas mudam
entre eles — como se abre, como se fecha, e o que se faz com os bytes.

Isso estava copiado três vezes. Copiado não é só volume: a `stop()` do TCP
ganhou um `shutdown()` antes do `close()` para destravar um `recv()` já
bloqueado, e as duas seriais nunca receberam o equivalente. Uma correção num
lugar não chegava nos outros, e o sintoma — thread que não morre no shutdown —
só aparece em bancada.

Contrato para quem herda:

  * chamar `_lifecycle_init()` no `__init__`;
  * implementar `_open()` (devolve o handle, ou None com o motivo já em
    `self.error`), `_close(handle)` e `_read_loop(handle)`;
  * opcionalmente sobrescrever `_unavailable()` para recusar a partida quando
    falta uma dependência (pyserial), e `_THREAD_NAME`.

`_read_loop` roda até `self._running` cair ou até levantar — as duas saídas
são normais e caem no mesmo caminho de reconexão.
"""
from __future__ import annotations

import threading
import time
from typing import Optional

try:
    import serial                       # type: ignore
    from serial.tools import list_ports  # type: ignore
    SERIAL_OK = True
except Exception:  # pragma: no cover - pyserial ausente
    serial = None                       # type: ignore
    list_ports = None                   # type: ignore
    SERIAL_OK = False

# Espera entre tentativas de reconexão. Curto o bastante para um replug ser
# quase transparente, longo o bastante para não virar espera ocupada com o
# cabo fora.
RETRY_S = 2.0

# Timeout do `read()`: volta rápido quando a linha cala, e o laço reavalia
# `self._running` em vez de travar no `stop()`.
_SERIAL_TIMEOUT_S = 0.2


class ReconnectingSource:
    """Thread de fundo que mantém um transporte vivo através de quedas."""

    _THREAD_NAME = 'line-source'

    def _lifecycle_init(self) -> None:
        self._running = False
        self._handle = None
        self._thread: Optional[threading.Thread] = None
        self.connected = False
        # time.monotonic() do último dado válido (0.0 = nunca).
        self.last_rx: float = 0.0
        self.error: str = ''

    # ── Pontos de extensão ────────────────────────────────────────────
    def _unavailable(self) -> str:
        """Motivo pelo qual nem vale armar a thread, ou '' se vale.

        Distinto de `_open()` falhar: dependência ausente não se resolve
        esperando, então `start()` devolve False e o chamador sabe na hora.
        """
        return ''

    def _open(self):
        """Handle aberto, ou None com o motivo já posto em `self.error`."""
        raise NotImplementedError

    def _close(self, handle) -> None:
        raise NotImplementedError

    def _read_loop(self, handle) -> None:
        raise NotImplementedError

    # ── Ciclo de vida ─────────────────────────────────────────────────
    def start(self) -> bool:
        """Arma a thread. False só quando falta dependência — dispositivo
        ausente NÃO é falha de partida: a thread fica tentando."""
        motivo = self._unavailable()
        if motivo:
            self.error = motivo
            return False
        if self._running:
            return True
        self._running = True
        self._thread = threading.Thread(
            target=self._worker, daemon=True, name=self._THREAD_NAME)
        self._thread.start()
        return True

    def stop(self) -> None:
        """Fecha o handle POR FORA antes do join: sem isso a thread só
        acordaria no timeout da leitura, e o `destroy_node` esperaria por ela
        a cada desligamento."""
        self._running = False
        handle = self._handle
        if handle is not None:
            self._close(handle)
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _worker(self) -> None:
        while self._running:
            handle = self._open()
            if handle is None:
                time.sleep(RETRY_S)
                continue
            self._handle = handle
            self.connected = True
            self.error = ''
            try:
                self._read_loop(handle)
            except Exception as exc:
                # Desconexão (replug, cabo, peer fechou) ou handle fechado
                # pelo stop(). Os dois caem aqui e valem a mesma reconexão.
                self.error = str(exc)
            finally:
                self.connected = False
                self._handle = None
                self._close(handle)
            if self._running:
                time.sleep(RETRY_S)


def read_chunk(ser) -> bytes:
    """`read(1)` bloqueante mais o que já estiver no buffer.

    Entrega a menor latência possível sem virar espera ocupada: o read(1)
    dorme até o primeiro byte, e o `in_waiting` recolhe o resto do bloco que o
    SO já tinha. Devolve b'' no timeout, que o chamador trata como "linha
    calada" e volta a testar `self._running`.
    """
    data = ser.read(1)
    if not data:
        return b''
    waiting = getattr(ser, 'in_waiting', 0)
    return data + ser.read(waiting) if waiting else data


class SerialSource(ReconnectingSource):
    """Transporte sobre porta serial com detecção automática e hot-plug.

    Quem herda define `_detect_port()` e `_absent_message()` e consome os
    bytes em `_on_chunk()`; o resto (abrir, fechar, reconectar, o timeout de
    leitura) é igual nas duas placas e mora aqui.

    Espera encontrar `self._port_req` (porta fixada pelo operador, ou None
    para detectar) e `self._baud` já postos pelo `__init__` de quem herda.
    """

    def _detect_port(self) -> Optional[str]:
        raise NotImplementedError

    def _absent_message(self) -> str:
        raise NotImplementedError

    def _on_chunk(self, data: bytes) -> None:
        raise NotImplementedError

    def _unavailable(self) -> str:
        return '' if SERIAL_OK else 'pyserial ausente'

    def _open(self):
        port = self._port_req or self._detect_port()
        if port is None:
            self.error = self._absent_message()
            return None
        try:
            ser = serial.Serial(port, self._baud, timeout=_SERIAL_TIMEOUT_S)
        except Exception as exc:
            self.error = str(exc)
            return None
        self.port = port
        return ser

    def _close(self, ser) -> None:
        try:
            ser.close()
        except Exception:
            pass

    def _read_loop(self, ser) -> None:
        while self._running:
            data = read_chunk(ser)
            if data:
                self._on_chunk(data)

    def _write_line(self, data: bytes, vazio: str) -> None:
        """Escreve no handle aberto. `vazio` é a mensagem de erro quando a
        porta não está aberta — cada placa descreve a sua."""
        ser = self._handle
        if ser is None:
            raise RuntimeError(vazio)
        ser.write(data)
        ser.flush()
