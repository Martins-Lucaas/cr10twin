"""palp_char: K do contato e pico espectral do deslize em dados sintéticos."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
# Sem importorskip aqui: com o plugin launch_testing (ROS sourced), um skip em
# nível de módulo zera a coleta da sessão INTEIRA, não só deste arquivo.
try:
    import pandas as pd
    import palp_char as PC  # noqa: E402
except ModuleNotFoundError as exc:
    if (exc.name or '').split('.')[0] not in ('pandas', 'scipy'):
        raise
    pd = PC = None

pytestmark = pytest.mark.skipif(PC is None, reason='sem pandas/scipy')


def _run(K=10.0, f_tex=12.0, v=5.0):
    fs = 400.0
    # DESCENDING: entra 0,3 mm a 0,1 mm/s; F = K·pen (N/mm).
    t1 = np.arange(0, 3.0, 1 / fs)
    pen = 0.1 * t1
    z1 = 0.100 - pen * 1e-3
    f1 = K * pen
    # SLIDING: 4 strokes de 10 mm a v mm/s em ±Y, Fn = 2 + 0,1·sin(2π f_tex t).
    T = 10.0 / v
    t2 = np.arange(0, 4 * T, 1 / fs)
    tri = np.abs(((t2 / T) % 2) - 1)            # 1→0→1 …
    y2 = (tri * 10.0) * 1e-3
    f2 = 2.0 + 0.1 * np.sin(2 * np.pi * f_tex * t2)
    t = np.r_[t1, t1[-1] + 1 / fs + t2]
    s = pd.DataFrame({
        't_unix': t, 't_rel_s': t,
        'phase': np.r_[np.full(t1.size, 1), np.full(t2.size, 3)],
        'force_net_n': np.r_[f1, f2], 'setpoint_n': 2.0,
        'tcp_x': 0.0, 'tcp_y': np.r_[np.zeros(t1.size), y2],
        'tcp_z': np.r_[z1, np.full(t2.size, z1[-1])],
        'q1': t, 'q2': 0.0, 'q3': 0.0,
        'lc_t_us': t * 1e6, 'lc_voltage_raw_v': np.r_[f1, f2]})
    w = pd.DataFrame({'t_unix': t, 'ft_fx_n': 0.0,
                      'ft_fy_n': 0.1 * np.r_[np.zeros(t1.size), np.ones(t2.size)],
                      'ft_fz_n': -np.r_[f1, f2]})
    return {'run': 'sint', 'p': {'slide_dir': '+Y', 'speed_mms': v},
            's': s, 'w': w}


def test_k_e_pico():
    d = _run()
    c = PC.contact(d)
    assert c['K_sec_Nmm'] == pytest.approx(10.0, rel=0.05)
    sl = PC.sliding(d)
    assert sl['n_strokes'] >= 3
    assert sl['peak_Hz'] == pytest.approx(12.0, abs=sl['df_Hz'])
    assert sl['Fn_mean_N'] == pytest.approx(2.0, abs=0.02)
    assert sl['mu'] == pytest.approx(0.05, rel=0.05)
