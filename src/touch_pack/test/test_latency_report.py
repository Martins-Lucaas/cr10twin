"""Testes do touch_pack.latency_report — estimativa de atraso e agregação.

O relatório reimplementa a correlação cruzada do `latency_probe` para rodar
sem ROS. O primeiro teste é o que amarra as duas implementações: se uma mudar
sem a outra, as tabelas do artigo deixam de bater com as capturas.
"""
import csv
import math

import numpy as np
import pytest

from touch_pack.constants import ARM_JOINTS, HAND_JOINTS
from touch_pack.latency_report import (
    GRID_DT_S, analyze_capture, xcorr_lag, _agg,
)


def _synth_raw(tmp_path, lag_s, amp_deg=10.0, dur_s=20.0, joint=3):
    """Grava um *_raw.csv sintético: o REAL é o SIM deslocado de `lag_s`.

    lag_s > 0 → o real atrasa (Sim-to-Real); < 0 → o sim atrasa (Real-to-Sim).
    """
    t = np.arange(0.0, dur_s, 0.008)
    wave = math.radians(amp_deg) * np.sin(2 * math.pi * 0.25 * t)
    path = tmp_path / 'latency_synth_20260101_000000_raw.csv'
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t_mono_s', 'source'] + [f'{j}_rad' for j in ARM_JOINTS])
        for k, tk in enumerate(t):
            q = [0.0] * 6
            q[joint] = float(wave[k])
            w.writerow([f'{tk:.6f}', 'sim'] + [f'{v:.6f}' for v in q])
        for k, tk in enumerate(t):
            q = [0.0] * 6
            # real(t) = sim(t - lag) → o real repete o sim `lag` depois
            q[joint] = float(math.radians(amp_deg)
                             * math.sin(2 * math.pi * 0.25 * (tk - lag_s)))
            w.writerow([f'{tk:.6f}', 'real'] + [f'{v:.6f}' for v in q])
    return str(path)


def test_xcorr_lag_matches_latency_probe():
    """A cópia no relatório tem de devolver o MESMO lag que a do probe."""
    from touch_pack.latency_probe import LatencyProbe

    rng = np.random.default_rng(7)
    s = np.cumsum(rng.normal(0, 1, 4000))
    r = np.roll(s, 12)
    a = xcorr_lag(s, r, GRID_DT_S, 100)
    b = LatencyProbe._xcorr_lag(s, r, GRID_DT_S, 100)
    assert a[0] == pytest.approx(b[0], abs=1e-12)
    assert a[1] == pytest.approx(b[1], abs=1e-12)


@pytest.mark.parametrize('lag_s', [0.072, -0.072, -0.0782])
def test_analyze_recovers_known_lag(tmp_path, lag_s):
    """Atraso conhecido volta com erro abaixo de meia amostra da grade."""
    path = _synth_raw(tmp_path, lag_s)
    rows = analyze_capture(path, min_amp_deg=0.3)
    assert len(rows) == 1                       # só a junta que se moveu
    got = rows[0]['lag_ms'] / 1e3
    assert got == pytest.approx(lag_s, abs=GRID_DT_S / 2)


def test_compensation_collapses_the_error(tmp_path):
    """Compensado o atraso, o erro some; sem compensar, ele domina.

    É o argumento central do artigo: os ~71 ms são atraso, não infidelidade.
    """
    path = _synth_raw(tmp_path, -0.072, amp_deg=20.0)
    row = analyze_capture(path, min_amp_deg=0.3)[0]
    assert row['rmse_deg'] < 0.05
    assert row['rmse_sem_compensar_deg'] > 20 * row['rmse_deg']


def test_agg_uses_lag_magnitude():
    """A agregação reporta |Δt|: sinais opostos não podem se cancelar."""
    rows = [
        {'lag_ms': -71.0, 'amp_deg': 5.0, 'mae_deg': 0.01, 'rmse_deg': 0.02,
         'emax_deg': 0.1, 'r_pearson': 0.999, 'rmse_sem_compensar_deg': 0.5},
        {'lag_ms': -73.0, 'amp_deg': 8.0, 'mae_deg': 0.01, 'rmse_deg': 0.02,
         'emax_deg': 0.2, 'r_pearson': 0.998, 'rmse_sem_compensar_deg': 0.6},
    ]
    a = _agg(rows)
    assert a['n'] == 2
    assert a['lag_m'] == pytest.approx(72.0)
    assert a['emax'] == pytest.approx(0.2)
    assert a['r_min'] == pytest.approx(0.998)


def _synth_hand_raw(tmp_path, lag_s, amp_deg=20.0, dur_s=20.0, digit=1):
    """*_raw.csv sintético no formato do `hand_latency_probe` (Thumb..Rotate).

    O relatório tem de descobrir o conjunto de juntas pelo CABEÇALHO: sem isso
    ele leria `joint1_rad` num arquivo da mão e quebraria.
    """
    t = np.arange(0.0, dur_s, 0.008)
    path = tmp_path / 'hand_latency_synth_20260101_000000_raw.csv'
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t_mono_s', 'source'] + [f'{j}_rad' for j in HAND_JOINTS])
        for src, shift in (('sim', 0.0), ('real', lag_s)):
            for tk in t:
                q = [0.0] * len(HAND_JOINTS)
                q[digit] = math.radians(amp_deg) * math.sin(
                    2 * math.pi * 0.25 * (tk - shift))
                w.writerow([f'{tk:.6f}', src] + [f'{v:.6f}' for v in q])
    return str(path)


@pytest.mark.parametrize('lag_s', [0.048, -0.048])
def test_analyze_reads_hand_captures(tmp_path, lag_s):
    """Captura da mão: mesma estimativa, rotulada com o nome do dedo."""
    rows = analyze_capture(_synth_hand_raw(tmp_path, lag_s), min_amp_deg=0.5)
    assert len(rows) == 1
    assert rows[0]['junta'] == 'Index'
    assert rows[0]['lag_ms'] / 1e3 == pytest.approx(lag_s, abs=GRID_DT_S / 2)


def test_hand_deg_to_driver_rad_spans_the_urdf_range():
    """Grau de ponta de dedo → junta driver: extremos e ausência de saturação.

    O bug de 10/09/2026 foi mandar grau de dedo (0–90°) direto para uma junta
    que só vai a 1,0 rad: tudo acima de 57,3° ceifava e o gêmeo perdia 30% da
    excursão. Este teste trava os dois extremos no lugar.
    """
    from touch_pack.constants import (
        HAND_DRIVER_LOWER_RAD, HAND_DRIVER_UPPER_RAD, HAND_SPAN_DEG,
        hand_deg_to_driver_rad)

    for j in HAND_JOINTS:
        assert hand_deg_to_driver_rad(j, 0.0) == pytest.approx(
            HAND_DRIVER_LOWER_RAD[j])
        assert hand_deg_to_driver_rad(j, HAND_SPAN_DEG[j]) == pytest.approx(
            HAND_DRIVER_UPPER_RAD[j])
        # Meio do curso cai no meio da faixa — mapeamento linear, sem degrau.
        assert hand_deg_to_driver_rad(j, HAND_SPAN_DEG[j] / 2) == pytest.approx(
            (HAND_DRIVER_LOWER_RAD[j] + HAND_DRIVER_UPPER_RAD[j]) / 2)
        # Curso cheio da mão real nunca satura o teto do driver.
        assert hand_deg_to_driver_rad(j, HAND_SPAN_DEG[j]) <= \
            HAND_DRIVER_UPPER_RAD[j] + 1e-9


def test_urdf_clamp_and_conversion_share_one_source():
    """O clamp do URDF e a conversão TÊM de usar o mesmo número.

    Foi a divergência entre os dois que produziu o ganho 0,77 da campanha da
    mão. Se alguém reintroduzir uma cópia local em hand_pack, isto quebra.
    """
    pytest.importorskip('hand_pack')
    from hand_pack.urdf_helpers import HAND_DRIVER_LIMITS, HAND_DRIVER_LOWER
    from touch_pack.constants import (
        HAND_DRIVER_LOWER_RAD, HAND_DRIVER_UPPER_RAD)

    assert HAND_DRIVER_LIMITS == HAND_DRIVER_UPPER_RAD
    assert HAND_DRIVER_LOWER == HAND_DRIVER_LOWER_RAD


def test_eci_posn_to_deg_matches_the_calibrated_span():
    """Fim de curso aberto → 0°, fechado → curso total; o meio é linear."""
    from touch_pack.constants import (
        ECI_POSN_CLOSED, ECI_POSN_OPEN, HAND_SPAN_DEG, eci_posn_to_deg)

    for j in HAND_JOINTS:
        assert eci_posn_to_deg(j, ECI_POSN_OPEN[j]) == pytest.approx(0.0)
        assert eci_posn_to_deg(j, ECI_POSN_CLOSED[j]) == pytest.approx(
            HAND_SPAN_DEG[j])
    # Fora do curso mecânico o valor satura em vez de extrapolar.
    assert eci_posn_to_deg('Index', 10) == 0.0
    assert eci_posn_to_deg('Index', 255) == pytest.approx(90.0)
