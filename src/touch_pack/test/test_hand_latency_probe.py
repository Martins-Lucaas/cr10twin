"""A medida de atraso SIM↔REAL da mão COVVI.

A saída deste nó é um número que vai para a dissertação: "a mão real segue a
simulada com X ms de atraso". Ele não tem verificação natural — se sair
errado, sai errado com a mesma cara de sempre. Por isso os testes aqui
injetam séries com o atraso CONHECIDO e exigem que a análise o recupere.

O que carrega o risco:

  * o sinal do lag decide QUEM lidera. Invertê-lo troca "a mão real atrasa"
    por "a mão real antecipa", que é fisicamente impossível e passaria;
  * a junta analisada é escolhida pela variância — um dedo parado tem
    correlação de ruído e devolveria um lag aleatório com cara de medida;
  * as duas séries chegam em relógios e taxas diferentes; a reamostragem na
    grade comum é o que torna a correlação comparável.

Nada aqui toca a mão real nem o driver: as séries são sintéticas.
"""
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

rclpy = pytest.importorskip('rclpy')

from touch_pack.constants import HAND_JOINTS                 # noqa: E402
from touch_pack.hand_latency_probe import (                  # noqa: E402
    MIN_AMP_DEG, HandLatencyProbe,
)

_N = len(HAND_JOINTS)
_DT = 0.004


@pytest.fixture(scope='module')
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def probe(_ros):
    n = HandLatencyProbe()
    yield n
    n.destroy_node()


def _serie(probe, atraso_s: float, *, dur_s: float = 8.0,
           amp_rad: float = 0.5, junta: int = 0, taxa_hz: float = 100.0):
    """Preenche o nó com uma senoide na SIM e a mesma senoide na REAL,
    deslocada de `atraso_s`. As outras juntas ficam paradas."""
    t = np.arange(0.0, dur_s, 1.0 / taxa_hz)
    base = 1000.0                       # relógio arbitrário: só o delta importa
    with probe._lock:
        probe._sim = [
            (base + float(ti),
             [amp_rad * np.sin(2 * np.pi * 0.5 * ti) if j == junta else 0.0
              for j in range(_N)])
            for ti in t]
        probe._real = [
            (base + float(ti),
             [amp_rad * np.sin(2 * np.pi * 0.5 * (ti - atraso_s))
              if j == junta else 0.0
              for j in range(_N)])
            for ti in t]


# ── Coleta ────────────────────────────────────────────────────────────
def test_arm_only_joint_state_is_ignored(probe):
    """`/joint_states` carrega braço e mão. Uma mensagem só do braço não tem
    as juntas da mão e não pode virar uma amostra de zeros — ela achataria a
    variância e roubaria a junta escolhida."""
    msg = type('_M', (), {'name': ['j1', 'j2'], 'position': [0.1, 0.2]})()
    probe._cb_joints(msg)
    assert probe._sim == []


def test_hand_joint_state_is_collected(probe):
    msg = type('_M', (), {'name': list(HAND_JOINTS),
                          'position': [0.1] * _N})()
    probe._cb_joints(msg)
    assert len(probe._sim) == 1
    assert probe._sim[0][1] == [pytest.approx(0.1)] * _N


def test_position_shorter_than_name_does_not_raise(probe):
    """`JointState` permite `position` vazio. O `zip` trunca em vez de
    estourar IndexError dentro do callback — o nó tem de sobreviver a um
    publicador que só manda velocidade."""
    msg = type('_M', (), {'name': list(HAND_JOINTS), 'position': []})()
    probe._cb_joints(msg)
    assert probe._sim == []


# ── Análise ───────────────────────────────────────────────────────────
def test_too_few_samples_returns_none_instead_of_a_number(probe):
    """Um lag calculado sobre 10 amostras é ruído com cara de medida. Melhor
    devolver nada e mandar repetir a captura."""
    _serie(probe, 0.05, dur_s=0.2, taxa_hz=100.0)
    assert probe.analyze() is None


def test_insufficient_overlap_returns_none(probe):
    """As duas séries vêm de relógios que começaram em momentos diferentes.
    Com menos de 2 s em comum a correlação não tem sobre o que deslizar."""
    _serie(probe, 0.05)
    with probe._lock:
        probe._real = [(t + 60.0, q) for t, q in probe._real]
    assert probe.analyze() is None


@pytest.mark.parametrize('atraso_s', [0.02, 0.05, 0.12])
def test_recovers_a_known_delay(probe, atraso_s):
    """O teste que dá sentido ao nó: atraso injetado tem de voltar na saída,
    dentro de um período da grade."""
    _serie(probe, atraso_s)
    res = probe.analyze()
    assert res is not None
    assert res['latency_ms'] == pytest.approx(atraso_s * 1e3, abs=_DT * 1e3)


def test_real_lagging_behind_sim_says_sim_leads(probe):
    """Sinal do lag = quem lidera. O caso físico real é este: a mão de verdade
    segue a simulada. Inverter isto publicaria uma conclusão impossível."""
    _serie(probe, 0.05)
    res = probe.analyze()
    assert res['leads'] == 'SIM'
    assert res['detected'] == 'sim_to_real'
    assert res['latency_ms'] > 0


def test_real_leading_sim_is_reported_as_the_other_direction(probe):
    """Acontece quando a captura mede o caminho inverso (comando entrando
    pela mão). O nó tem de nomear a direção, não devolver um atraso
    negativo que ninguém interpretaria."""
    _serie(probe, -0.05)
    res = probe.analyze()
    assert res['leads'] == 'REAL'
    assert res['detected'] == 'real_to_sim'
    assert res['latency_ms'] > 0, 'a latência reportada é sempre magnitude'


def test_the_moving_joint_is_the_one_analysed(probe):
    """Escolher pela variância é o que impede medir o atraso de um dedo
    parado, cuja correlação é ruído puro."""
    _serie(probe, 0.05, junta=3)
    res = probe.analyze()
    assert res['joint'] == 3
    assert res['joint_name'] == HAND_JOINTS[3]


def test_an_explicit_joint_index_overrides_the_variance_pick(probe):
    """O operador pode saber qual dedo interessa mesmo que outro se mexa
    mais — o parâmetro tem de vencer a heurística."""
    _serie(probe, 0.05, junta=3)
    probe.set_parameters([rclpy.parameter.Parameter(
        'joint_index', rclpy.Parameter.Type.INTEGER, 1)])
    assert probe.analyze()['joint'] == 1


def test_an_out_of_range_joint_index_falls_back_to_the_variance_pick(probe):
    """`-p joint_index:=9` é erro de digitação, não pedido para estourar
    IndexError no meio de uma captura de 20 s já concluída."""
    _serie(probe, 0.05, junta=3)
    probe.set_parameters([rclpy.parameter.Parameter(
        'joint_index', rclpy.Parameter.Type.INTEGER, 99)])
    assert probe.analyze()['joint'] == 3


def test_still_fingers_are_left_out_of_the_per_joint_table(probe):
    """A tabela por dedo é a redundância que valida o número principal. Um
    dedo parado entrando nela com um lag de ruído destruiria essa função."""
    _serie(probe, 0.05, junta=0)
    res = probe.analyze()
    assert list(res['per_joint']) == [HAND_JOINTS[0]]
    assert res['per_joint'][HAND_JOINTS[0]]['amp_deg'] >= MIN_AMP_DEG


def test_series_with_different_rates_are_resampled_to_a_common_grid(probe):
    """SIM vem do `/joint_states` (~50 Hz) e REAL da telemetria ECI (outra
    taxa, outro relógio). Sem a reamostragem a correlação compararia
    amostras que não são do mesmo instante."""
    _serie(probe, 0.05, taxa_hz=100.0)
    t = np.arange(0.0, 8.0, 1.0 / 37.0)          # taxa diferente e não múltipla
    with probe._lock:
        probe._real = [
            (1000.0 + float(ti),
             [0.5 * np.sin(2 * np.pi * 0.5 * (ti - 0.05))] + [0.0] * (_N - 1))
            for ti in t]
    res = probe.analyze()
    assert res is not None
    assert res['latency_ms'] == pytest.approx(50.0, abs=15.0)
    assert res['real_hz'] == pytest.approx(37.0, rel=0.05)
    assert res['sim_hz'] == pytest.approx(100.0, rel=0.05)


def test_peak_correlation_is_high_for_a_clean_shift(probe):
    """Correlação baixa com sinal limpo significaria que a grade ou o
    alinhamento estão errados — é o detector de que o número não vale."""
    _serie(probe, 0.05)
    assert probe.analyze()['peak_corr'] > 0.95
