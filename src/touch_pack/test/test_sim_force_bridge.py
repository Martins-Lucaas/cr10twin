"""A ponte de força da SIMULAÇÃO — e a razão de ela poder emudecer.

`sim_force_bridge` republica o wrench do plugin do Gazebo como o mesmo
`/load_cell/force_net` que o receiver real publica. O explorer confia nesse
tópico para tudo: a guarda `_force_stale_abort` decide que a célula está muda
pela IDADE da última mensagem recebida, não pelo conteúdo.

Isso cria uma armadilha que só existe na simulação: se a ponte republicar o
último valor num timer fixo, o tópico fica eternamente "fresco" mesmo com o
plugin morto, e a guarda de segurança nunca dispara. O ensaio seguiria
descendo contra uma leitura de força congelada. É esse contrato — *plugin
mudo ⇒ tópico mudo* — que estes testes travam.
"""
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

rclpy = pytest.importorskip('rclpy')

from rclpy.duration import Duration                          # noqa: E402
from geometry_msgs.msg import WrenchStamped                  # noqa: E402

from touch_pack.sim_force_bridge import (                    # noqa: E402
    _WRENCH_STALE_S, SimForceBridge,
)


@pytest.fixture(scope='module')
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def bridge(_ros):
    node = SimForceBridge()
    # O publisher real exigiria um subscriber para se observar. Trocar por um
    # coletor mantém o teste sem rede e sem espera.
    node._published: list[float] = []
    node._pub = type('_Pub', (), {
        'publish': lambda _self, msg: node._published.append(float(msg.data)),
    })()
    yield node
    node.destroy_node()


def _wrench(fz: float) -> WrenchStamped:
    msg = WrenchStamped()
    msg.wrench.force.z = float(fz)
    return msg


def _envelhece(node, segundos: float) -> None:
    """Recua o carimbo do último wrench sem dormir o teste."""
    node._raw_ts = node._raw_ts - Duration(seconds=segundos)


# ── Conversão ─────────────────────────────────────────────────────────
def test_sign_default_turns_plugin_axis_into_positive_compression(bridge):
    """O default `sign=-1` existe porque o plugin entrega o eixo invertido em
    relação à convenção do receiver real (+ = compressão)."""
    bridge._on_wrench(_wrench(-3.0))
    bridge._tick()
    assert bridge._published == [pytest.approx(3.0)]


def test_offset_is_subtracted_after_the_sign(bridge):
    """Ordem importa: o offset é a tara em NEWTONS já na convenção final, não
    no eixo cru do plugin."""
    bridge._sign, bridge._offset = -1.0, 0.5
    bridge._on_wrench(_wrench(-3.0))
    bridge._tick()
    assert bridge._published == [pytest.approx(2.5)]


# ── Silêncio do plugin (regressão) ────────────────────────────────────
def test_publishes_nothing_before_the_first_wrench(bridge):
    """Sem nenhuma leitura não há o que republicar — e o explorer tem de ver
    o tópico mudo, não um zero inventado."""
    bridge._tick()
    bridge._tick()
    assert bridge._published == []


def test_stops_publishing_when_the_plugin_goes_quiet(bridge):
    """REGRESSÃO. Antes, `_tick` republicava `self._raw` a 80 Hz para sempre:
    com o plugin morto o tópico continuava fresco e `_force_stale_abort` do
    explorer nunca disparava em simulação."""
    bridge._on_wrench(_wrench(-2.0))
    bridge._tick()
    assert len(bridge._published) == 1

    _envelhece(bridge, _WRENCH_STALE_S + 0.1)
    for _ in range(40):                      # meio segundo de timer a 80 Hz
        bridge._tick()
    assert len(bridge._published) == 1, (
        'a ponte continuou republicando o último valor com o plugin mudo')


def test_a_reading_just_inside_the_window_still_publishes(bridge):
    """A janela é um teto, não um ritmo: uma leitura mais nova que
    `_WRENCH_STALE_S` continua valendo, senão a ponte piscaria a cada
    engasgo do Gazebo."""
    bridge._on_wrench(_wrench(-2.0))
    _envelhece(bridge, _WRENCH_STALE_S * 0.5)
    bridge._tick()
    assert bridge._published == [pytest.approx(2.0)]


def test_recovers_when_the_plugin_comes_back(bridge):
    """Emudecer não pode ser terminal: o Gazebo volta, e a ponte tem de
    voltar com ele sem reiniciar o nó."""
    bridge._on_wrench(_wrench(-2.0))
    _envelhece(bridge, _WRENCH_STALE_S + 0.1)
    bridge._tick()
    assert bridge._published == []

    bridge._on_wrench(_wrench(-4.0))
    bridge._tick()
    assert bridge._published == [pytest.approx(4.0)]


def test_filter_state_is_dropped_when_the_plugin_goes_quiet(bridge):
    """O passa-baixa guarda o valor anterior. Mantê-lo através de um silêncio
    faria a primeira amostra da volta ser uma média com a força de ANTES da
    pausa — um degrau que não existiu."""
    bridge._filter_hz = 5.0
    bridge._on_wrench(_wrench(-10.0))
    bridge._tick()
    assert bridge._filt is not None

    _envelhece(bridge, _WRENCH_STALE_S + 0.1)
    bridge._tick()
    assert bridge._filt is None

    bridge._on_wrench(_wrench(-1.0))
    bridge._tick()
    # Sem estado antigo, a primeira amostra da volta é ela mesma.
    assert bridge._published[-1] == pytest.approx(1.0)


def test_stale_window_is_not_wider_than_the_consumer_guard():
    """A ponte tem de emudecer ANTES de o explorer decidir que a leitura está
    velha. Se esta janela crescer além da de lá, volta o bug: o explorer vê
    dado fresco que já não descreve a simulação."""
    from touch_pack.tactile_explorer import _FORCE_STALE_S
    assert _WRENCH_STALE_S <= _FORCE_STALE_S
