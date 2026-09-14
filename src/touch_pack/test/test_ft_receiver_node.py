"""O receiver da célula FA7155 — tare, auto-zero e o filtro de quadros.

Este é o nó que alimenta `/load_cell/force_net`, a entrada da malha de força
do explorer. Tudo que ele decide errado vira força errada contra a amostra,
sem log e sem erro:

  * publicar `force_net` ANTES de haver tare regularia contra um zero que
    ninguém conferiu;
  * um tare tirado com a ponteira já encostada embutiria a pré-carga no zero
    — daí a exigência de estabilidade por DERIVA;
  * o auto-zero cancela deriva térmica, mas se rodar durante um HOLD ele
    zera o próprio contato e a malha empurra cada vez mais forte;
  * um quadro que passou no CRC por azar pode trazer 1200 N e envenenar o
    filtro por segundos.

São 870 linhas cujo caminho de dados nunca tinha teste. O que se exercita
aqui é esse caminho, sobre um dublê — o construtor abre porta serial.
"""
import collections
import pathlib
import sys
import threading

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

pytest.importorskip('rclpy')
pytest.importorskip('touch_pack_msgs')

from touch_pack.constants import (                           # noqa: E402
    FT_AXES, FT_RATED_FORCE_N, FT_RATED_TORQUE_NM,
)
from touch_pack.ft_receiver_node import (                    # noqa: E402
    _FORCE_ABSURD_N, _TORQUE_ABSURD_NM, FtReceiverNode,
)

_N = len(FT_AXES)
_AXIS_Z = 2


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _Recv:
    """Portador de estado que reexpõe os métodos reais via `self`."""

    def __init__(self, *, autozero=False, phase='', filtro=False, sign=1.0):
        self._lock = threading.Lock()
        self._buf = collections.deque(maxlen=4000)
        self._tare = [0.0] * _N
        self._tare_done = False
        self._axis_i = _AXIS_Z
        self._axis_name = 'z'
        self._sign = sign
        self._filter_on = filtro
        self._filters = []
        self._last_t_us = None
        self._absurd = 0
        self._rx_frames = 0
        self._rate_win = collections.deque(maxlen=100)
        self._link_ok = False
        self._autozero = autozero
        self._phase = phase
        self._frame_id = 'ft_sensor'

        self._force_pub = _Pub()
        self._force_net_pub = _Pub()
        self._sample_pub = _Pub()
        self._sample_net_pub = _Pub()
        self._wrench_pub = _Pub()
        self._wrench_raw_pub = _Pub()
        self._tare_result_pub = _Pub()

    def get_logger(self):
        return type('_L', (), {'info': lambda _s, *a, **k: None,
                               'warn': lambda _s, *a, **k: None,
                               'error': lambda _s, *a, **k: None})()

    def get_clock(self):
        return type('_C', (), {
            'now': lambda _s: type('_T', (), {
                'to_msg': lambda _t: __import__(
                    'builtin_interfaces.msg', fromlist=['Time']).Time()})()})()

    _CAPTURE_WIN_N = FtReceiverNode._CAPTURE_WIN_N
    _TARE_STABLE_N = FtReceiverNode._TARE_STABLE_N
    _AUTOZERO_BAND_N = FtReceiverNode._AUTOZERO_BAND_N
    _AUTOZERO_RATE = FtReceiverNode._AUTOZERO_RATE
    _AUTOZERO_PHASES = FtReceiverNode._AUTOZERO_PHASES

    _window_drift = staticmethod(FtReceiverNode._window_drift)
    _publish_tare_result = FtReceiverNode._publish_tare_result
    _apply_tare = FtReceiverNode._apply_tare
    _on_tare_request = FtReceiverNode._on_tare_request
    _auto_tare = FtReceiverNode._auto_tare
    _on_sample = FtReceiverNode._on_sample
    _publish_net = FtReceiverNode._publish_net
    _publish_wrench = FtReceiverNode._publish_wrench
    _on_palpation_status = FtReceiverNode._on_palpation_status


def _linha(fz: float, outros: float = 0.0):
    v = [outros] * _N
    v[_AXIS_Z] = fz
    return v


def _enche(recv, fz=0.0, n=None):
    """Enche o buffer com uma janela estável no valor pedido."""
    n = n or recv._CAPTURE_WIN_N
    for _ in range(n):
        recv._buf.append(_linha(fz))


# ── Deriva da janela ──────────────────────────────────────────────────
def test_drift_of_a_flat_window_is_zero():
    assert FtReceiverNode._window_drift([5.0] * 200) == pytest.approx(0.0)


def test_drift_sees_a_ramp_that_peak_to_peak_would_also_see():
    win = [float(i) / 100 for i in range(200)]
    assert FtReceiverNode._window_drift(win) == pytest.approx(1.0, abs=0.02)


def test_drift_ignores_a_spike_that_peak_to_peak_would_reject():
    """A razão de ser mediana e não ptp: um único pico não é instabilidade.
    Com ptp, um glitch isolado recusaria um tare perfeitamente bom e o
    operador ficaria apertando o botão sem entender."""
    win = [0.0] * 200
    win[97] = 50.0
    assert FtReceiverNode._window_drift(win) == pytest.approx(0.0)


# ── Tare ──────────────────────────────────────────────────────────────
def test_tare_zeroes_all_six_axes():
    """Zerar só o eixo de controle deixaria o wrench publicado com os outros
    cinco carregando o peso da ferramenta — e a GUI mostraria momento onde
    não há."""
    recv = _Recv()
    for _ in range(recv._CAPTURE_WIN_N):
        recv._buf.append([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
    ok, drift = recv._apply_tare(list(recv._buf))
    assert ok is True and drift == pytest.approx(0.0)
    assert recv._tare == pytest.approx([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])


def test_tare_is_refused_while_the_signal_drifts():
    """Deriva na janela = alguém encostando, ou a célula ainda aquecendo. Um
    zero tirado aí embute a pré-carga e toda força medida depois sai
    deslocada — sem nenhum sintoma visível."""
    recv = _Recv()
    win = [_linha(float(i) * 0.01) for i in range(recv._CAPTURE_WIN_N)]
    ok, drift = recv._apply_tare(win)
    assert ok is False
    assert drift > recv._TARE_STABLE_N
    assert recv._tare_done is False


def test_tare_stability_is_judged_on_the_control_axis_only():
    """Exigir repouso simultâneo nos seis eixos recusaria tares bons por um
    momento residual do peso da ferramenta, que não participa da malha."""
    recv = _Recv()
    win = []
    for i in range(recv._CAPTURE_WIN_N):
        row = _linha(0.0)
        row[4] = float(i) * 0.01        # my derivando forte
        win.append(row)
    ok, _drift = recv._apply_tare(win)
    assert ok is True


def test_tare_request_without_data_reports_no_data():
    recv = _Recv()
    _enche(recv, n=10)
    recv._on_tare_request(None)
    assert recv._tare_result_pub.msgs[-1].data.startswith('err;no_data')
    assert recv._tare_done is False


def test_tare_request_reports_the_drift_when_it_refuses():
    """O número tem de voltar para a GUI: "deriva 0,8 N" diz ao operador que
    ele encostou; um "err" seco manda ele apertar de novo sem saber por quê."""
    recv = _Recv()
    for i in range(recv._CAPTURE_WIN_N):
        recv._buf.append(_linha(float(i) * 0.01))
    recv._on_tare_request(None)
    campos = recv._tare_result_pub.msgs[-1].data.split(';')
    assert campos[:2] == ['err', 'drifting']
    assert float(campos[2]) > recv._TARE_STABLE_N


def test_successful_tare_reports_ok_and_the_reference():
    recv = _Recv()
    _enche(recv, fz=4.0)
    recv._on_tare_request(None)
    campos = recv._tare_result_pub.msgs[-1].data.split(';')
    assert campos[0] == 'ok'
    assert float(campos[1]) == pytest.approx(4.0)


def test_auto_tare_waits_for_a_full_window():
    """Meia janela mede meio segundo de sinal. Aceitar isso na partida faria
    o zero depender de quando o nó subiu."""
    recv = _Recv()
    _enche(recv, n=recv._CAPTURE_WIN_N - 1)
    assert recv._auto_tare(list(recv._buf)) is False
    assert recv._tare_done is False


def test_auto_tare_on_a_full_stable_window_succeeds():
    """Sem auto-tare nada sai em force_net até alguém apertar o botão — e o
    explorer recusa todo ensaio por leitura ausente."""
    recv = _Recv()
    _enche(recv, fz=1.5)
    assert recv._auto_tare(list(recv._buf)) is True
    assert recv._tare_done is True


# ── Caminho da amostra ────────────────────────────────────────────────
def test_absurd_force_frame_is_dropped_before_the_filter():
    """Quadro que passou no CRC por azar. Deixá-lo entrar envenena o
    passa-baixa por segundos — muito depois de o quadro ruim sumir."""
    recv = _Recv()
    recv._on_sample(1, 1000, tuple(_linha(_FORCE_ABSURD_N + 1.0)))
    assert recv._absurd == 1
    assert len(recv._buf) == 0
    assert recv._force_pub.msgs == []


def test_absurd_torque_frame_is_dropped_too():
    recv = _Recv()
    vals = [0.0] * _N
    vals[4] = _TORQUE_ABSURD_NM + 1.0
    recv._on_sample(1, 1000, tuple(vals))
    assert recv._absurd == 1


def test_the_absurd_thresholds_sit_above_the_rated_range():
    """O corte é para sincronismo perdido, não para sobrecarga: uma leitura
    acima do nominal ainda é física e tem de chegar à malha de segurança."""
    assert _FORCE_ABSURD_N > FT_RATED_FORCE_N
    assert _TORQUE_ABSURD_NM > FT_RATED_TORQUE_NM


def test_nothing_reaches_force_net_before_the_tare():
    """O contrato que sustenta a segurança: sem zero conferido, o explorer
    tem de ver leitura AUSENTE e recusar o ensaio — não um número."""
    recv = _Recv()
    recv._on_sample(1, 1000, tuple(_linha(2.0)))
    assert recv._force_net_pub.msgs == []
    assert recv._sample_net_pub.msgs == []
    assert recv._force_pub.msgs != [], 'a força CRUA sai desde o primeiro quadro'


def test_force_net_flows_once_the_tare_is_done():
    recv = _Recv()
    recv._tare_done = True
    recv._tare = _linha(1.0)
    recv._on_sample(1, 1000, tuple(_linha(3.0)))
    assert recv._force_net_pub.msgs[-1].data == pytest.approx(2.0)


def test_sign_parameter_flips_the_control_axis():
    """A convenção do explorer é + = compressão. O eixo do sensor depende de
    como a ferramenta foi montada, e inverter aqui é o ajuste previsto."""
    recv = _Recv(sign=-1.0)
    recv._tare_done = True
    recv._on_sample(1, 1000, tuple(_linha(3.0)))
    assert recv._force_net_pub.msgs[-1].data == pytest.approx(-3.0)


def test_raw_sample_carries_newtons_not_volts():
    """Campo herdado do HX711, onde era tensão da ponte. No FA7155 é newton,
    e o consumidor (`tactile_explorer._cb_lc_sample_net`) escala por isso."""
    recv = _Recv()
    recv._on_sample(1, 1000, tuple(_linha(7.0)))
    s = recv._sample_pub.msgs[-1]
    assert s.voltage_raw == pytest.approx(7.0)
    assert s.calibrated is True


def test_micros_wrap_around_is_not_a_half_hour_gap():
    """O t_us do host estoura em 32 bits. Sem a máscara o dt sairia enorme,
    cairia fora da janela válida e o filtro usaria a taxa nominal por um
    quadro — um degrau no sinal filtrado."""
    recv = _Recv()
    recv._last_t_us = 0xFFFFFFFF - 999
    recv._on_sample(2, 1000, tuple(_linha(1.0)))
    assert recv._last_t_us == 1000
    assert recv._absurd == 0


def test_sequence_and_timestamp_are_masked_to_32_bits():
    """Os campos da mensagem são uint32. Um valor maior estouraria no
    serializador e derrubaria o nó no meio do ensaio."""
    recv = _Recv()
    recv._on_sample(2 ** 33 + 5, 2 ** 33 + 7, tuple(_linha(1.0)))
    s = recv._sample_pub.msgs[-1]
    assert 0 <= s.seq < 2 ** 32 and 0 <= s.t_us < 2 ** 32


# ── Auto-zero ─────────────────────────────────────────────────────────
def test_autozero_does_not_run_during_a_hold():
    """A guarda que importa. Durante um HOLD a força é REAL e constante: o
    auto-zero a absorveria como deriva, a malha veria erro crescente e
    empurraria cada vez mais forte contra a amostra."""
    recv = _Recv(autozero=True, phase='HOLD')
    recv._tare_done = True
    antes = list(recv._tare)
    recv._publish_net(1, 1000, tuple(_linha(0.1)), _linha(0.1))
    assert recv._tare == pytest.approx(antes)


def test_autozero_does_not_run_outside_the_dead_band():
    """Força acima da banda é contato, não deriva térmica — mesmo em IDLE."""
    recv = _Recv(autozero=True, phase='IDLE')
    recv._tare_done = True
    antes = list(recv._tare)
    fz = recv._AUTOZERO_BAND_N + 0.5
    recv._publish_net(1, 1000, tuple(_linha(fz)), _linha(fz))
    assert recv._tare == pytest.approx(antes)


def test_autozero_creeps_towards_the_reading_when_idle_and_inside_the_band():
    recv = _Recv(autozero=True, phase='IDLE')
    recv._tare_done = True
    recv._publish_net(1, 1000, tuple(_linha(0.1)), _linha(0.1))
    assert recv._tare[_AXIS_Z] == pytest.approx(0.1 * recv._AUTOZERO_RATE)


def test_autozero_is_slow_enough_not_to_eat_a_real_contact():
    """Tau ~4 s a 1 kHz: um passo por amostra mal move o zero. Um passo
    grande demais comeria a força de um contato lento antes de ele ser
    detectado."""
    assert FtReceiverNode._AUTOZERO_RATE < 0.001


def test_autozero_stays_off_when_disabled():
    recv = _Recv(autozero=False, phase='IDLE')
    recv._tare_done = True
    antes = list(recv._tare)
    recv._publish_net(1, 1000, tuple(_linha(0.1)), _linha(0.1))
    assert recv._tare == pytest.approx(antes)


@pytest.mark.parametrize('phase', ['', 'IDLE', 'DONE', 'ABORTED'])
def test_autozero_phases_are_the_ones_with_no_experiment_running(phase):
    """A lista é a definição de "braço parado". Acrescentar uma fase de
    movimento aqui reabriria o bug do auto-zero comendo o contato."""
    assert phase in FtReceiverNode._AUTOZERO_PHASES
    for ativa in ('DESCENDING', 'HOLD', 'SLIDING', 'CALIBRATE_ATTACK'):
        assert ativa not in FtReceiverNode._AUTOZERO_PHASES


def test_palpation_status_updates_the_phase_gate():
    """É por este tópico que o receiver sabe que um ensaio começou. Perder a
    atualização deixaria o auto-zero ligado durante a palpação."""
    recv = _Recv(autozero=True, phase='IDLE')
    recv._on_palpation_status(type('_M', (), {'phase': 'HOLD'})())
    assert recv._phase == 'HOLD'
    recv._tare_done = True
    antes = list(recv._tare)
    recv._publish_net(1, 1000, tuple(_linha(0.1)), _linha(0.1))
    assert recv._tare == pytest.approx(antes)
