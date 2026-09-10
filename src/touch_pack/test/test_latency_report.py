"""Testes do touch_pack.latency_report — estimativa de atraso e agregação.

O relatório reimplementa a correlação cruzada do `latency_probe` para rodar
sem ROS. O primeiro teste é o que amarra as duas implementações: se uma mudar
sem a outra, as tabelas do artigo deixam de bater com as capturas.
"""
import csv
import math

import numpy as np
import pytest

from touch_pack.constants import ARM_JOINTS
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
