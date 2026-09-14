"""O ciclo de vida compartilhado pelos três transportes.

`FtSerialSource`, `LoadCellSerialSource` e `FtTcpSource` tinham cada um a sua
cópia de "tenta abrir, lê até cair, fecha, espera, tenta de novo". As três
cópias agora são uma (`line_source.ReconnectingSource`), e é por isso que ela
precisa de teste: um erro aqui não quebra um transporte, quebra os três — e
quebra do jeito mais caro, deixando o nó sem dado no meio de um ensaio sem
dizer que perdeu a conexão.

O que se trava:

  * dispositivo ausente NÃO é falha de partida (a placa entra depois, o cabo
    volta); dependência ausente É, e tem de ser dita na hora;
  * a queda no meio da leitura leva à reconexão em vez de matar a thread;
  * `stop()` fecha o handle POR FORA — sem isso a thread só acordaria no
    timeout e todo desligamento esperaria por ela;
  * o handle é fechado exatamente uma vez por conexão, mesmo quando a leitura
    sai levantando.
"""
import pathlib
import sys
import threading
import time

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from touch_pack.line_source import (                         # noqa: E402
    ReconnectingSource, SerialSource, read_chunk,
)
import touch_pack.line_source as ls                          # noqa: E402


@pytest.fixture(autouse=True)
def _sem_espera(monkeypatch):
    """A espera de reconexão é 2 s de relógio de parede. Encurtar mantém o
    comportamento e tira 20 s da suíte."""
    monkeypatch.setattr(ls, 'RETRY_S', 0.002)


class _Fonte(ReconnectingSource):
    """Transporte de mentira: abre o que mandarem e lê o que mandarem."""

    _THREAD_NAME = 'teste'

    def __init__(self, aberturas, *, indisponivel=''):
        self._lifecycle_init()
        self._aberturas = list(aberturas)   # cada item: handle, ou None
        self._indisponivel = indisponivel
        self.abertos, self.fechados = [], []
        self.lendo = threading.Event()
        self.solte = threading.Event()

    def _unavailable(self):
        return self._indisponivel

    def _open(self):
        h = self._aberturas.pop(0) if self._aberturas else None
        if h is None:
            self.error = 'nada para abrir'
            return None
        self.abertos.append(h)
        return h

    def _close(self, handle):
        self.fechados.append(handle)

    def _read_loop(self, handle):
        self.lendo.set()
        self.solte.wait(timeout=2.0)
        if isinstance(handle, Exception):
            raise handle
        while self._running:
            time.sleep(0.001)


def _espera(cond, prazo=2.0):
    fim = time.monotonic() + prazo
    while time.monotonic() < fim:
        if cond():
            return True
        time.sleep(0.002)
    return False


# ── Partida ───────────────────────────────────────────────────────────
def test_a_missing_dependency_refuses_to_start_and_says_why():
    """pyserial ausente não se resolve esperando. Armar a thread assim
    deixaria um nó girando em vão e um `connected=False` sem explicação."""
    f = _Fonte([], indisponivel='pyserial ausente')
    assert f.start() is False
    assert f.error == 'pyserial ausente'
    assert f._thread is None


def test_a_missing_device_is_not_a_startup_failure():
    """A placa pode entrar na USB depois do launch — e entra, o tempo todo.
    Recusar a partida aqui exigiria reiniciar o nó a cada replug."""
    f = _Fonte([None, None])
    assert f.start() is True
    assert _espera(lambda: f.error == 'nada para abrir')
    f.stop()


def test_start_is_idempotent():
    """A GUI chama `start()` ao reconectar. Uma segunda thread lendo a mesma
    porta picotaria os quadros entre as duas."""
    f = _Fonte(['h1'])
    f.start()
    assert _espera(lambda: f.lendo.is_set())
    t = f._thread
    assert f.start() is True
    assert f._thread is t
    f.solte.set()
    f.stop()


# ── Conexão e queda ───────────────────────────────────────────────────
def test_a_successful_open_marks_connected_and_clears_the_error():
    """`connected`/`error` são o que a GUI mostra ao operador. Um erro velho
    sobrevivendo à reconexão manda procurar um problema que já passou."""
    f = _Fonte([None, 'h1'])
    f.start()
    assert _espera(lambda: f.connected)
    assert f.error == ''
    f.solte.set()
    f.stop()


def test_a_read_failure_reconnects_instead_of_killing_the_thread():
    """Replug, cabo, peer fechando: a queda é o caso NORMAL. Sair da thread
    deixaria o nó vivo e mudo até alguém reiniciar o launch."""
    f = _Fonte([OSError('cabo removido'), 'h2'])
    f.start()
    assert _espera(lambda: f.lendo.is_set())
    f.solte.set()
    assert _espera(lambda: f.abertos == [f.abertos[0], 'h2'])
    assert 'cabo removido' in f.error or f.connected
    f.stop()


def test_the_handle_is_closed_exactly_once_per_connection():
    """Fechar duas vezes esconde erro de handle já fechado; não fechar vaza
    um fd por queda, e uma sessão de bancada tem muitas."""
    f = _Fonte([OSError('caiu')])
    f.start()
    assert _espera(lambda: f.lendo.is_set())
    f.solte.set()
    assert _espera(lambda: len(f.fechados) == 1)
    f.stop()
    assert f.fechados.count(f.abertos[0]) == 1


def test_connected_goes_false_when_the_link_drops():
    f = _Fonte([OSError('caiu')])
    f.start()
    assert _espera(lambda: f.connected)
    f.solte.set()
    assert _espera(lambda: not f.connected)
    f.stop()


# ── Encerramento ──────────────────────────────────────────────────────
def test_stop_closes_the_handle_from_the_outside():
    """A leitura está bloqueada esperando bytes. Sem fechar por fora, a
    thread só acordaria no timeout e todo `destroy_node` esperaria por ela."""
    f = _Fonte(['h1'])
    f.start()
    assert _espera(lambda: f.lendo.is_set())
    f.solte.set()
    assert _espera(lambda: f.connected)
    f.stop()
    assert 'h1' in f.fechados


def test_stop_joins_and_forgets_the_thread():
    f = _Fonte(['h1'])
    f.start()
    assert _espera(lambda: f.lendo.is_set())
    f.solte.set()
    f.stop()
    assert f._thread is None
    assert f._running is False


def test_stop_before_start_does_not_raise():
    """O `destroy_node` chama `stop()` mesmo quando a partida falhou."""
    f = _Fonte([], indisponivel='pyserial ausente')
    f.start()
    f.stop()


# ── Leitura de bloco ──────────────────────────────────────────────────
class _Ser:
    def __init__(self, chunks, waiting=0):
        self._chunks = list(chunks)
        self.in_waiting = waiting

    def read(self, n):
        return self._chunks.pop(0) if self._chunks else b''


def test_read_chunk_returns_empty_on_a_silent_line():
    """b'' é "linha calada", e o laço tem de voltar a testar `_running` em
    vez de tratar isso como fim de conexão."""
    assert read_chunk(_Ser([b''])) == b''


def test_read_chunk_drains_what_the_os_already_had():
    """O read(1) dorme até o primeiro byte; o resto do bloco já está no
    buffer. Devolver um byte por vez multiplicaria por mil as voltas do laço
    a 1 kHz."""
    assert read_chunk(_Ser([b'A', b'BCDE'], waiting=4)) == b'ABCDE'


def test_read_chunk_does_not_ask_for_more_when_nothing_is_waiting():
    assert read_chunk(_Ser([b'A'], waiting=0)) == b'A'


# ── Porta serial ──────────────────────────────────────────────────────
class _SerialFake(SerialSource):
    def __init__(self, porta_detectada):
        self._lifecycle_init()
        self._port_req = None
        self._baud = 115200
        self.port = None
        self._detectada = porta_detectada
        self.chunks = []

    def _detect_port(self):
        return self._detectada

    def _absent_message(self):
        return 'placa ausente'

    def _on_chunk(self, data):
        self.chunks.append(data)


def test_serial_open_reports_the_absent_board_without_raising():
    f = _SerialFake(None)
    assert f._open() is None
    assert f.error == 'placa ausente'


def test_serial_open_publishes_the_port_it_actually_used(monkeypatch):
    """A GUI mostra esse campo. Ficar em None com a placa conectada faz o
    operador procurar um problema de cabo que não existe."""
    aberta = {}

    class _S:
        def __init__(self, port, baud, timeout):
            aberta.update(port=port, baud=baud, timeout=timeout)

    monkeypatch.setattr(ls, 'serial', type('_M', (), {'Serial': _S}))
    f = _SerialFake('/dev/ttyUSB9')
    assert f._open() is not None
    assert f.port == '/dev/ttyUSB9'
    assert aberta['timeout'] > 0, 'sem timeout o stop() trava na leitura'


def test_an_explicit_port_wins_over_detection(monkeypatch):
    """Com dois conversores genéricos no mesmo PC o auto-detect escolhe o
    errado — o parâmetro do nó existe exatamente para isso."""
    vistos = []
    monkeypatch.setattr(ls, 'serial', type('_M', (), {
        'Serial': lambda *a, **k: vistos.append(a[0])}))
    f = _SerialFake('/dev/ttyUSB0')
    f._port_req = '/dev/ttyUSB3'
    f._open()
    assert vistos == ['/dev/ttyUSB3']


def test_a_failed_open_reports_the_reason_and_retries(monkeypatch):
    """Porta ocupada por outro processo é o caso comum. A mensagem do sistema
    é o que diz ao operador que ele deixou uma GUI antiga aberta."""
    def _boom(*a, **k):
        raise PermissionError('porta ocupada')

    monkeypatch.setattr(ls, 'serial', type('_M', (), {'Serial': _boom}))
    f = _SerialFake('/dev/ttyUSB0')
    assert f._open() is None
    assert 'ocupada' in f.error


def test_write_without_an_open_port_says_so():
    """Melhor um RuntimeError com a placa nomeada do que um AttributeError
    em None no meio de uma sessão Modbus."""
    f = _SerialFake('/dev/ttyUSB0')
    with pytest.raises(RuntimeError, match='XIAO'):
        f._write_line(b'Z', 'porta serial do XIAO não está aberta')


# ── As três fontes compartilham mesmo a base ──────────────────────────
def test_all_three_transports_share_one_lifecycle():
    """A guarda do refactor: se alguém reintroduzir um `start`/`stop`/
    `_worker` próprio numa das fontes, as correções de uma param de chegar
    nas outras — que foi como a `stop()` do TCP ganhou o `shutdown()` e as
    seriais ficaram sem."""
    pytest.importorskip('rclpy')
    from touch_pack.ft_serial import FtSerialSource
    from touch_pack.ft_tcp import FtTcpSource
    from touch_pack.lc_serial import LoadCellSerialSource

    for cls in (FtSerialSource, FtTcpSource, LoadCellSerialSource):
        assert issubclass(cls, ReconnectingSource)
        for nome in ('start', 'stop', '_worker'):
            assert nome not in vars(cls), (
                f'{cls.__name__} voltou a ter {nome} próprio')


def test_the_two_ft_transports_stamp_frames_with_the_same_code():
    """O carimbo retro-data os quadros de um bloco pelo período nominal. Duas
    cópias divergindo dariam dt diferentes nos dois transportes e o One-Euro
    filtraria diferente conforme o cabo — impossível de perceber olhando."""
    pytest.importorskip('rclpy')
    from touch_pack.ft_serial import FtSerialSource
    from touch_pack.ft_tcp import FtTcpSource
    assert FtTcpSource._stamp_frames is FtSerialSource._stamp_frames
