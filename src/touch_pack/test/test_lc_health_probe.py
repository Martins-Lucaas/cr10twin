"""A sonda de saúde do link da célula axial — o que ela conta e o que ignora.

Este nó existe para PRODUZIR UM NÚMERO que decide se uma mudança de firmware
melhorou ou piorou o stream do HX711. Um número errado aqui não quebra nada
em runtime: faz aprovar um firmware ruim, ou rejeitar um bom — e a conta só
se refaz com dez minutos de bancada parada.

Três decisões carregam esse risco e são o que estes testes travam:

  * só um salto de seq de EXATAMENTE 1 mede um período de conversão;
  * os contadores do firmware são acumulados desde o boot DELE, então o que
    vale é a diferença entre as pontas da captura;
  * um dt de várias vezes o período é buraco no stream, não jitter — entrar
    na média deformaria o desvio que é justamente a métrica comparada.

O nó é exercitado sobre um dublê: o construtor assina tópicos e lê a
calibração do disco, e nada disso participa da aritmética.
"""
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

pytest.importorskip('rclpy')
pytest.importorskip('touch_pack_msgs')

from touch_pack.lc_health_probe import (                     # noqa: E402
    _DT_OUTLIER_FACTOR, LcHealthProbe, _stats,
)


class _Msg:
    """LoadCellSample reduzido aos campos que a sonda lê."""

    def __init__(self, seq, t_us, v_raw=0.0, v=0.0):
        self.seq, self.t_us = seq, t_us
        self.voltage_raw, self.voltage = v_raw, v


class _Health:
    def __init__(self, texto):
        self.data = texto


class _Probe:
    """Portador de estado que reexpõe os métodos reais via `self`."""

    def __init__(self, slope=0.0):
        self._t0 = 0.0
        self._seq_prev = None
        self._t_us_prev = None
        self._perdidas = 0
        self._resyncs = 0
        self._linhas = []
        self._dts_us = []
        self._v_raw = []
        self._fw_primeiro = {}
        self._fw_ultimo = {}
        self._slope = slope
        self._dur = 0.0
        self._rotulo = ''

    _on_sample = LcHealthProbe._on_sample
    _on_fw_health = LcHealthProbe._on_fw_health
    _diff_fw = LcHealthProbe._diff_fw
    _resultado = LcHealthProbe._resultado


# ── Estatística ───────────────────────────────────────────────────────
def test_stats_of_an_empty_series_is_empty():
    """Devolver zeros faria uma captura vazia parecer uma captura perfeita
    no JSON de comparação."""
    assert _stats([]) == {}


def test_stats_computes_population_sigma():
    """Divisor n, não n−1: as duas capturas sendo comparadas usam o mesmo
    divisor, e trocar um deles mudaria o veredito sem mudar o sinal."""
    s = _stats([2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0])
    assert s['media'] == pytest.approx(5.0)
    assert s['sigma'] == pytest.approx(2.0)
    assert (s['min'], s['max'], s['n']) == (2.0, 9.0, 8)


def test_p99_never_walks_off_the_end():
    """Com poucas amostras `int(0.99*n)` chega em n — o clamp é o que impede
    o IndexError logo na primeira captura curta de teste."""
    for n in range(1, 12):
        s = _stats([float(i) for i in range(n)])
        assert s['p99'] == float(n - 1) or s['p99'] <= float(n - 1)


# ── Sequência e dt ────────────────────────────────────────────────────
def test_consecutive_samples_measure_one_conversion_period():
    p = _Probe()
    p._on_sample(_Msg(1, 1_000))
    p._on_sample(_Msg(2, 11_000))
    assert p._dts_us == [pytest.approx(10_000.0)]
    assert p._perdidas == 0


def test_the_first_sample_measures_no_interval():
    """Sem par anterior não há período; inventar um a partir do t0 do nó
    colocaria o tempo de subida do launch dentro da estatística."""
    p = _Probe()
    p._on_sample(_Msg(7, 500))
    assert p._dts_us == []


def test_a_gap_is_counted_and_does_not_produce_a_dt():
    """O dt através de um buraco cobriria DUAS conversões e entraria na
    estatística como jitter — exatamente a leitura errada que faria um
    firmware sadio parecer dessincronizado."""
    p = _Probe()
    p._on_sample(_Msg(1, 1_000))
    p._on_sample(_Msg(4, 31_000))
    assert p._perdidas == 2
    assert p._dts_us == []


def test_a_huge_jump_is_a_resync_not_lost_samples():
    """Placa reiniciada zera o seq. Contar isso como perda inventaria
    milhões de amostras perdidas e jogaria `perdidas_frac` para 1."""
    p = _Probe()
    p._on_sample(_Msg(50_000, 1_000))
    p._on_sample(_Msg(3, 2_000))
    assert p._resyncs == 1
    assert p._perdidas == 0


def test_sequence_wrap_around_is_continuity():
    """seq é uint32 do firmware: 0xFFFFFFFF → 0 é a amostra seguinte."""
    p = _Probe()
    p._on_sample(_Msg(0xFFFFFFFF, 1_000))
    p._on_sample(_Msg(0, 11_000))
    assert (p._perdidas, p._resyncs) == (0, 0)
    assert p._dts_us == [pytest.approx(10_000.0)]


def test_micros_wrap_around_does_not_produce_a_negative_interval():
    """O t_us do firmware também estoura em 32 bits. Sem a máscara o dt
    sairia negativo e arrastaria a média para baixo."""
    p = _Probe()
    p._on_sample(_Msg(1, 0xFFFFFFFF - 4_999))
    p._on_sample(_Msg(2, 5_000))
    assert p._dts_us == [pytest.approx(10_000.0)]


# ── Heartbeat do firmware ─────────────────────────────────────────────
def test_fw_counters_are_differenced_across_the_capture():
    """Os contadores da placa acumulam desde o boot DELA. Sem a subtração,
    entrar no meio de uma sessão já rodando reportaria como "desta captura"
    todos os resets da tarde inteira."""
    p = _Probe()
    p._on_fw_health(_Health('resets=40 timeouts=7'))
    p._on_fw_health(_Health('resets=43 timeouts=7'))
    assert p._diff_fw('resets') == pytest.approx(3.0)
    assert p._diff_fw('timeouts') == pytest.approx(0.0)


def test_fw_health_ignores_malformed_tokens():
    """A linha vem do fio. Um token sem `=` ou com lixo no valor não pode
    derrubar o nó no meio de uma captura de dez minutos."""
    p = _Probe()
    p._on_fw_health(_Health('resets=2 lixo conv_us=abc zeroed=1'))
    assert p._fw_ultimo == {'resets': 2.0, 'zeroed': 1.0}


def test_a_line_with_nothing_usable_does_not_become_the_baseline():
    """Se uma linha vazia virasse `_fw_primeiro`, a diferença passaria a ser
    contra o zero e devolveria o acumulado desde o boot da placa."""
    p = _Probe()
    p._on_fw_health(_Health('sem nenhum sinal de igual'))
    assert p._fw_primeiro == {}
    p._on_fw_health(_Health('resets=40'))
    p._on_fw_health(_Health('resets=41'))
    assert p._diff_fw('resets') == pytest.approx(1.0)


def test_no_heartbeat_at_all_reports_zero_not_a_crash():
    p = _Probe()
    assert p._diff_fw('resets') == 0.0


# ── Resultado ─────────────────────────────────────────────────────────
def _com_amostras(p, dts_us):
    t = 0
    p._on_sample(_Msg(1, t))
    for i, dt in enumerate(dts_us, start=2):
        t += int(dt)
        p._on_sample(_Msg(i, t))
    return p


def test_dt_outliers_are_kept_out_of_the_compared_statistic():
    """O desvio do dt é A métrica comparada entre firmwares. Um buraco de
    10× o período dentro dela mascararia qualquer diferença real."""
    p = _com_amostras(_Probe(), [10_000] * 20 + [10_000 * 10])
    r = p._resultado()
    assert r['dt_outliers'] == 1
    assert r['dt_us_sem_outliers']['sigma'] == pytest.approx(0.0, abs=1e-9)
    assert r['dt_us']['sigma'] > 0.0, 'a série crua tem de manter o buraco'


def test_outlier_cut_is_relative_to_the_median():
    """Corte pelo p50 e não pela média: a média já está contaminada pelos
    buracos que o corte deveria remover."""
    p = _com_amostras(_Probe(), [10_000] * 10)
    r = p._resultado()
    assert r['dt_us']['p50'] == pytest.approx(10_000.0)
    assert _DT_OUTLIER_FACTOR > 1.0


def test_noise_in_newtons_is_omitted_without_a_calibration():
    """Sem a reta não há newton. Inventar um número aqui seria pior que
    devolver None: ele seguiria para a comparação como se valesse."""
    p = _Probe(slope=0.0)
    p._v_raw = [1.0, 2.0, 3.0]
    r = p._resultado()
    assert r['sigma_n'] is None and r['tres_sigma_n'] is None
    assert r['sigma_v'] > 0.0, 'o ruído em volts vale mesmo sem calibração'


def test_noise_in_newtons_uses_the_calibration_slope():
    p = _Probe(slope=2.0)
    p._v_raw = [0.0, 4.0]          # sigma = 2 V
    r = p._resultado()
    assert r['sigma_n'] == pytest.approx(1.0)
    assert r['tres_sigma_n'] == pytest.approx(3.0)


def test_lost_fraction_counts_the_samples_that_never_arrived():
    """Denominador = recebidas + perdidas. Usar só as recebidas daria 0 %
    de perda justamente na captura em que tudo se perdeu."""
    p = _Probe()
    p._v_raw = [0.0] * 90
    p._perdidas = 10
    assert p._resultado()['perdidas_frac'] == pytest.approx(0.10)
