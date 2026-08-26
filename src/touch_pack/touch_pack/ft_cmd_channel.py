"""ft_cmd_channel.py — derivação da linha 485 para o canal de comando.

O problema que este módulo resolve: a porta tem UM dono e UMA thread lendo.
`FtSerialSource._read_loop` e `FtTcpSource._read_loop` consomem todos os bytes
e entregam ao `FtFrameParser`. Um segundo `read()` para o Modbus roubaria
bytes do stream e picotaria os quadros — o oposto do que o parser existe para
evitar.

A saída é uma DERIVAÇÃO, não um segundo leitor: enquanto uma sessão de comando
está aberta, o laço de leitura copia cada bloco cru para uma fila antes de
alimentar o parser. O stream continua intacto (o `ft_receiver` não perde um
quadro sequer durante um Set_Zero) e o cliente Modbus lê da fila como se
tivesse a porta só para ele.

Sem dependência de `ft_modbus` — é o `ft_modbus` que consome estes callables.
Assim `ft_serial`/`ft_tcp` podem herdar daqui sem ciclo de import.
"""
from __future__ import annotations

import threading
from contextlib import contextmanager
from typing import Optional


class LineTapMixin:
    """Dá a um transporte o par (write, read) que o cliente Modbus espera.

    Quem herda tem de: chamar `_tap_init()` no __init__, chamar `_tap_feed()`
    no laço de leitura com cada bloco cru, e implementar `_line_write()`.
    """

    def _tap_init(self) -> None:
        self._tap_lock = threading.Lock()
        self._tap_buf: Optional[bytearray] = None   # None = sessão fechada

    # ── Lado do transporte ────────────────────────────────────────────
    def _tap_feed(self, data: bytes) -> None:
        """Chamado pelo laço de leitura. Barato quando não há sessão: um
        teste de None sob lock, ~200 vezes por segundo."""
        if not data:
            return
        with self._tap_lock:
            if self._tap_buf is None:
                return
            self._tap_buf += data
            # Teto de segurança: uma sessão esquecida aberta não pode crescer
            # sem limite a 250 quadros/s.
            if len(self._tap_buf) > 65536:
                del self._tap_buf[:len(self._tap_buf) - 8192]

    def _line_write(self, data: bytes) -> None:
        raise NotImplementedError

    # ── Lado do cliente Modbus ────────────────────────────────────────
    def _tap_read(self, max_bytes: int) -> bytes:
        """Devolve o que houver AGORA (pode ser b''), sem bloquear — o laço de
        prazo é do cliente."""
        with self._tap_lock:
            if not self._tap_buf:
                return b''
            out = bytes(self._tap_buf[:max_bytes])
            del self._tap_buf[:max_bytes]
            return out

    @contextmanager
    def command_session(self):
        """Abre a derivação e devolve `(write_fn, read_fn)`.

        Sessões não aninham: a segunda entrada esvaziaria o buffer da primeira
        no meio de uma transação. Uma sessão por vez, e o erro é explícito em
        vez de virar timeout misterioso.
        """
        with self._tap_lock:
            if self._tap_buf is not None:
                raise RuntimeError(
                    'já existe uma sessão de comando aberta nesta linha')
            self._tap_buf = bytearray()
        try:
            yield self._line_write, self._tap_read
        finally:
            with self._tap_lock:
                self._tap_buf = None
