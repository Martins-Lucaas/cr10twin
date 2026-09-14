"""O receptor UDP do sensor de toque — o que ele aceita e o que ele descarta.

A porta 8081 escuta em 0.0.0.0 e o que entra vai direto para o CSV do
experimento. Toda a lógica deste nó é um filtro de entrada: origem, tamanho,
finitude e continuidade de sequência. Cada coisa que passa indevidamente vira
uma linha falsa numa campanha que ninguém vai refazer.

Sem rede: o socket é substituído por um dublê que devolve os pacotes de cada
teste e depois encerra o laço. É o único jeito de exercitar `_udp_loop`
inteiro — inclusive o caminho de erro de bind, onde estava um vazamento de fd.
"""
import pathlib
import socket as _socket
import struct
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

rclpy = pytest.importorskip('rclpy')

from touch_pack.constants import TOUCH_PAYLOAD_FMT           # noqa: E402
from touch_pack import touch_receiver_node as trn            # noqa: E402


@pytest.fixture(scope='module')
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class _FakeSocket:
    """Dublê de socket UDP: entrega uma fila de (payload, origem) e depois
    levanta OSError, que é como o laço real termina quando o socket fecha."""

    def __init__(self, pacotes=(), bind_erro: OSError | None = None):
        self._pacotes = list(pacotes)
        self._bind_erro = bind_erro
        self.closed = 0
        self.bound = None

    def setsockopt(self, *a):
        pass

    def settimeout(self, *a):
        pass

    def bind(self, addr):
        if self._bind_erro is not None:
            raise self._bind_erro
        self.bound = addr

    def recvfrom(self, _n):
        if not self._pacotes:
            raise OSError('socket fechado')
        return self._pacotes.pop(0)

    def close(self):
        self.closed += 1


@pytest.fixture()
def node(_ros, monkeypatch):
    """Nó com a thread de rede neutralizada: o `_udp_loop` de verdade é
    chamado à mão por cada teste, com o dublê que aquele teste precisa."""
    monkeypatch.setattr(trn.threading, 'Thread',
                        lambda *a, **k: type('_T', (), {
                            'start': lambda _s: None,
                            'is_alive': lambda _s: False,
                            'join': lambda _s, **_kw: None,
                        })())
    n = trn.TouchReceiverNode()
    n._publicados: list[float] = []
    n._value_pub = type('_Pub', (), {
        'publish': lambda _s, msg: n._publicados.append(float(msg.data)),
    })()
    yield n
    n.destroy_node()


def _pkt(seq: int, valor: float, origem: str = '10.0.0.7'):
    return (struct.pack(TOUCH_PAYLOAD_FMT, seq, valor), (origem, 5000))


# ── Bind ──────────────────────────────────────────────────────────────
def test_failed_bind_closes_the_socket(node, monkeypatch):
    """REGRESSÃO: o `return` do erro de bind saía sem fechar o socket já
    criado. Com o nó em respawn (porta ocupada) vazava um fd por tentativa."""
    fake = _FakeSocket(bind_erro=OSError('address already in use'))
    monkeypatch.setattr(_socket, 'socket', lambda *a, **k: fake)
    node._udp_loop()
    assert fake.closed == 1


def test_successful_bind_listens_on_all_interfaces(node, monkeypatch):
    """A porta tem de ficar em 0.0.0.0: o plotter é outra máquina e o bind em
    localhost silenciaria a campanha inteira sem erro nenhum."""
    fake = _FakeSocket()
    monkeypatch.setattr(_socket, 'socket', lambda *a, **k: fake)
    node._udp_loop()
    assert fake.bound == ('', trn.UDP_PORT)


# ── Filtro de entrada ─────────────────────────────────────────────────
def _roda(node, monkeypatch, pacotes):
    fake = _FakeSocket(pacotes)
    monkeypatch.setattr(_socket, 'socket', lambda *a, **k: fake)
    node._udp_loop()
    return fake


def test_valid_packet_is_republished(node, monkeypatch):
    _roda(node, monkeypatch, [_pkt(1, 2.5)])
    assert node._publicados == [pytest.approx(2.5)]


def test_short_packet_is_dropped(node, monkeypatch):
    """Pacote truncado desempacotaria lixo como float."""
    _roda(node, monkeypatch, [(b'\x01\x02\x03', ('10.0.0.7', 5000))])
    assert node._publicados == []


@pytest.mark.parametrize('valor', [float('nan'), float('inf'), float('-inf')])
def test_non_finite_value_is_dropped(node, monkeypatch, valor):
    """NaN/Inf no fio contaminam o gráfico e o CSV de forma irrecuperável."""
    _roda(node, monkeypatch, [_pkt(1, valor)])
    assert node._publicados == []
    assert node._bad_value == 1


def test_packet_from_an_unexpected_source_is_dropped(node, monkeypatch):
    """Bancada compartilhada: com `allowed_source` fixo, o que vier de outra
    máquina não entra no CSV do experimento."""
    node._allowed_src = '10.0.0.7'
    _roda(node, monkeypatch, [_pkt(1, 1.0, origem='10.0.0.99')])
    assert node._publicados == []
    assert node._foreign == 1


def test_any_source_is_accepted_by_default(node, monkeypatch):
    """O default histórico é aceitar qualquer origem — mudar isso quebraria
    capturas antigas sem aviso."""
    assert node._allowed_src == ''
    _roda(node, monkeypatch, [_pkt(1, 1.0, origem='192.168.1.50')])
    assert node._publicados == [pytest.approx(1.0)]


# ── Continuidade de sequência ─────────────────────────────────────────
def test_consecutive_sequence_counts_no_drops(node, monkeypatch):
    _roda(node, monkeypatch, [_pkt(1, 1.0), _pkt(2, 2.0), _pkt(3, 3.0)])
    assert node._drops == 0
    assert len(node._publicados) == 3


def test_gap_in_sequence_is_counted(node, monkeypatch):
    """O contador de perdas é o que diz depois se a campanha vale — um salto
    silencioso vira um buraco no CSV que ninguém sabe explicar."""
    _roda(node, monkeypatch, [_pkt(10, 1.0), _pkt(14, 2.0)])
    assert node._drops == 3
    assert len(node._publicados) == 2


def test_plotter_restart_is_not_counted_as_lost_packets(node, monkeypatch):
    """Salto enorme = o plotter reiniciou e zerou o contador. Somar isso a
    `_drops` inventaria milhões de perdas que não aconteceram."""
    _roda(node, monkeypatch, [_pkt(50_000, 1.0), _pkt(0, 2.0)])
    assert node._drops == 0
    assert len(node._publicados) == 2


def test_sequence_wrap_around_is_not_a_gap(node, monkeypatch):
    """O seq é uint32 do firmware: 0xFFFFFFFF → 0 é continuidade, não perda.
    Sem a aritmética em 32 bits isto viraria um salto negativo."""
    _roda(node, monkeypatch, [_pkt(0xFFFFFFFF, 1.0), _pkt(0, 2.0)])
    assert node._drops == 0
    assert len(node._publicados) == 2
