#!/usr/bin/env python3
"""palp_char.py — Caracterização força × altura × deslize de runs gravados.

Estágio 1 do controle de força (identificação, sem controlador). Só LÊ os
arquivos do run; nunca fala com o braço.

Uso:
  python3 src/touch_pack/scripts/palp_char.py RUN_DIR [RUN_DIR ...] [--plot]

Por run imprime e grava <run>/char.json; com --plot grava <run>/char.png.
Com vários runs imprime também a tabela comparativa (efeito da velocidade).

Fontes (todas com t_unix — é por ele que se sincroniza):
  samples.csv  força filtrada (força_net_n, N, + compressão), fase, q, tcp (m)
  sensors.csv  wrench 6 eixos da FA7155 (N, eixo z com sinal do sensor)
  cmd.csv      posição COMANDADA (só runs gravados depois de 2026-11-23)

Métricas:
  taxas        célula (lc_t_us), pose, comando; quantum de z gravado
  contato      K secante F/indentação na subida do DESCENDING+HOLD (N/mm),
               rigidez local nos terços inferior/superior (não linearidade)
  hold         deriva (relaxação) e ruído destendenciado do sinal CRU
  sliding      por stroke: Fn média/σ, assimetria ida×volta, dFn/dx (N/mm),
               μ = |Ft|/Fn, RMS por banda 0–2 / 2–20 / 20–80 Hz, pico, e a
               coerência de Fn com z e com a velocidade medida
  atraso       correlação cruzada comando × medido no eixo do deslize
"""
from __future__ import annotations

import json
import os
import sys
import warnings

import numpy as np
import pandas as pd
from scipy.signal import coherence, correlate, detrend, welch

warnings.filterwarnings('ignore')
BANDS = ((0.0, 2.0), (2.0, 20.0), (20.0, 80.0))   # Hz — ver relatório
PH_DESC, PH_HOLD, PH_SLIDE = 1, 2, 3


def _segments(mask: np.ndarray, min_len: int) -> list[np.ndarray]:
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return []
    return [g for g in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
            if len(g) >= min_len]


def _band_rms(f, P, lo, hi):
    m = (f >= lo) & (f < hi)
    return float(np.sqrt(np.trapz(P[m], f[m]))) if m.sum() > 1 else float('nan')


def load(run: str) -> dict:
    p = json.load(open(os.path.join(run, 'params.json')))
    s = pd.read_csv(os.path.join(run, 'samples.csv'),
                    usecols=lambda c: not c.startswith('taxel'))
    d = {'run': run, 'p': p, 's': s}
    fs_path = os.path.join(run, 'sensors.csv')
    if os.path.exists(fs_path):
        w = pd.read_csv(fs_path, usecols=['t_unix', 'ft_fx_n', 'ft_fy_n',
                                          'ft_fz_n'])
        # A GUI grava a ~1 kHz repetindo a última amostra: só as novas.
        d['w'] = w[w.ft_fz_n.diff().fillna(1) != 0].reset_index(drop=True)
    c_path = os.path.join(run, 'cmd.csv')
    if os.path.exists(c_path):
        d['c'] = pd.read_csv(c_path)
    return d


def rates(d: dict) -> dict:
    s = d['s']
    pose = s[s[['q1', 'q2', 'q3']].diff().abs().sum(axis=1) > 0]
    dz = np.abs(np.diff(s.tcp_z.values))
    out = {'lc_hz': 1e6 / float(np.median(np.diff(s.lc_t_us))),
           'pose_hz': 1 / float(np.median(np.diff(pose.t_unix))),
           'z_quantum_um': float(dz[dz > 0].min() * 1e6)}
    if 'w' in d:
        out['wrench_hz'] = 1 / float(np.median(np.diff(d['w'].t_unix)))
    if 'c' in d:
        out['cmd_hz'] = 1 / float(np.median(np.diff(d['c'].t_unix)))
    return out


def contact(d: dict) -> dict:
    """K da subida de força com o braço indo para dentro (DESCENDING+HOLD)."""
    s = d['s']
    ks, k_lo, k_hi, ind = [], [], [], []
    for g in _segments(s.phase.isin([PH_DESC, PH_HOLD]).values, 100):
        z = s.tcp_z.values[g] * 1e3
        f = s.force_net_n.values[g]
        c = np.flatnonzero(f > 0.3)
        if c.size < 20:
            continue
        pen = z[c[0]] - z[c]                # mm, + = para dentro
        fc = f[c]
        if np.ptp(pen) < 0.02:
            continue
        ks.append(np.ptp(fc) / np.ptp(pen))
        ind.append(float(pen.max()))
        o = np.argsort(pen)
        third = len(o) // 3
        for sel, acc in ((o[:third], k_lo), (o[-third:], k_hi)):
            if np.ptp(pen[sel]) > 0.01:
                acc.append(np.polyfit(pen[sel], fc[sel], 1)[0])
    if not ks:
        return {}
    return {'n': len(ks), 'K_sec_Nmm': float(np.median(ks)),
            'K_sec_p10_p90': [float(np.percentile(ks, 10)),
                              float(np.percentile(ks, 90))],
            'K_low_Nmm': float(np.median(k_lo)) if k_lo else None,
            'K_high_Nmm': float(np.median(k_hi)) if k_hi else None,
            'indent_mm': float(np.median(ind))}


def hold(d: dict) -> dict:
    s = d['s']
    raw = s.lc_voltage_raw_v.values      # FA7155: newtons, sem filtro
    drift, noise, fmean = [], [], []
    for g in _segments(s.phase.values == PH_HOLD, 200):
        t = s.t_rel_s.values[g]
        tail = g[t >= t[-1] - min(2.0, (t[-1] - t[0]) / 2)]
        drift.append(np.polyfit(s.t_rel_s.values[tail],
                                s.force_net_n.values[tail], 1)[0])
        noise.append(np.std(detrend(raw[tail])))
        fmean.append(s.force_net_n.values[tail].mean())
    if not drift:
        return {}
    return {'n': len(drift), 'F_mean_N': float(np.mean(fmean)),
            'setpoint_N': float(s.setpoint_n.median()),
            'drift_N_s': float(np.median(drift)),
            'noise_raw_std_N': float(np.median(noise))}


def sliding(d: dict) -> dict:
    s, p = d['s'], d['p']
    if 'w' not in d or not (s.phase == PH_SLIDE).any():
        return {}
    w = d['w']
    ax = 'tcp_y' if str(p.get('slide_dir', 'Y'))[-1].upper() == 'Y' else 'tcp_x'
    t = w.t_unix.values
    fs = 1 / float(np.median(np.diff(t)))
    pos = np.interp(t, s.t_unix, s[ax]) * 1e3
    ph = np.round(np.interp(t, s.t_unix, s.phase))
    # Velocidade de uma posição suavizada em 50 ms (a pose tem ruído de
    # quantização que, derivado direto, vira ±5 mm/s).
    k = max(3, int(0.05 * fs))
    vel = np.gradient(np.convolve(pos, np.ones(k) / k, 'same'), t)
    v_cmd = float(p.get('speed_mms', 0.0))
    sg = np.sign(vel) * ((ph == PH_SLIDE) & (np.abs(vel) > 0.5 * v_cmd))
    strokes = [g for g in np.split(np.arange(len(t)),
                                   np.flatnonzero(np.diff(sg) != 0) + 1)
               if sg[g[0]] != 0 and len(g) > 0.25 * fs]
    # Corta 10 % de cada ponta: aceleração/inversão não é regime.
    strokes = [g[len(g) // 10: -max(1, len(g) // 10)] for g in strokes]
    if not strokes:
        return {}
    # Fragmentos (stroke partido por um vale de velocidade) encurtariam a
    # janela do Welch de todos: só os de pelo menos metade da mediana.
    med = np.median([len(g) for g in strokes])
    strokes = [g for g in strokes if len(g) >= 0.5 * med]
    fn = -w.ft_fz_n.values                   # sinal do sensor: − = compressão
    ft = (w.ft_fy_n if ax == 'tcp_y' else w.ft_fx_n).values
    nps = min(min(len(g) for g in strokes), 512)
    fr = welch(fn[strokes[0]][:nps], fs=fs, nperseg=nps)[0]
    P = np.mean([welch(detrend(fn[g]), fs=fs, nperseg=nps)[1]
                 for g in strokes], 0)
    dirn = np.array([sg[g[0]] for g in strokes])
    fm = np.array([fn[g].mean() for g in strokes])
    slope = np.array([np.polyfit(pos[g], fn[g], 1)[0] for g in strokes])
    mu = np.array([abs(ft[g].mean()) / max(fn[g].mean(), 1e-3)
                   for g in strokes])
    b = (fr >= 2) & (fr <= 80)
    out = {'n_strokes': len(strokes), 'stroke_s': nps / fs, 'df_Hz': float(fr[1]),
           'v_cmd_mms': v_cmd,
           'v_meas_mms': float(np.median([abs(np.polyfit(t[g], pos[g], 1)[0])
                                          for g in strokes])),
           'Fn_mean_N': float(fm.mean()),
           'Fn_pos_dir_N': float(fm[dirn > 0].mean()) if (dirn > 0).any() else None,
           'Fn_neg_dir_N': float(fm[dirn < 0].mean()) if (dirn < 0).any() else None,
           'Fn_std_in_stroke_N': float(np.median([fn[g].std() for g in strokes])),
           'dFn_dx_Nmm': float(np.median(slope)),
           'mu': float(np.median(mu)),
           'peak_Hz': float(fr[b][np.argmax(P[b])]),
           'rms_band_N': {f'{lo:g}-{hi:g}': _band_rms(fr, P, lo, hi)
                          for lo, hi in BANDS}}
    # Coerência com a cinemática medida (reamostrado a 100 Hz, fase inteira).
    s3 = s[(s.phase == PH_SLIDE)]
    s3 = s3[s3[[ax, 'tcp_z']].diff().abs().sum(axis=1) > 0]
    if len(s3) > 600:
        tt = np.arange(s3.t_unix.min(), s3.t_unix.max(), 0.01)
        vv = np.gradient(np.interp(tt, s3.t_unix, s3[ax]) * 1e3, tt)
        zz = np.interp(tt, s3.t_unix, s3.tcp_z) * 1e3
        ff = np.interp(tt, t, fn)
        for name, x in (('coh_Fn_v', vv), ('coh_Fn_z', zz)):
            f, C = coherence(ff, x, fs=100, nperseg=512)
            m = (f > 2) & (f < 20)
            out[name] = [float(C[m].max()), float(f[m][np.argmax(C[m])])]
    return out


def delay(d: dict) -> dict:
    """Atraso comando→medido no eixo do deslize (correlação das velocidades)."""
    if 'c' not in d:
        return {}
    s, c, p = d['s'], d['c'], d['p']
    ax = 'y' if str(p.get('slide_dir', 'Y'))[-1].upper() == 'Y' else 'x'
    t0, t1 = max(s.t_unix.min(), c.t_unix.min()), min(s.t_unix.max(),
                                                      c.t_unix.max())
    tt = np.arange(t0, t1, 0.005)
    if len(tt) < 400:
        return {}
    # O comando vale para t_unix + time_from_start.
    cm = np.interp(tt, c.t_unix + c.cmd_tfs_s, c[f'cmd_tcp_{ax}'])
    me = np.interp(tt, s.t_unix, s[f'tcp_{ax}'])
    a, b = np.gradient(cm), np.gradient(me)
    xc = correlate(b - b.mean(), a - a.mean(), 'full')
    lags = (np.arange(len(xc)) - (len(a) - 1)) * 0.005
    m = (lags >= 0) & (lags < 0.5)
    return {'cmd_to_meas_s': float(lags[m][np.argmax(xc[m])]),
            'track_err_rms_mm': float(np.std((me - cm)) * 1e3)}


def plot(d: dict, res: dict, out: str) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    s = d['s']
    t = s.t_rel_s.values
    v = lambda x: np.asarray(x, dtype=float)
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    ax[0].plot(t, v(s.force_net_n), lw=.5, label='Fn medida (filtrada)')
    ax[0].plot(t, v(s.setpoint_n), 'k--', lw=.8, label='setpoint')
    ax[0].set_ylabel('N'); ax[0].legend(loc='upper right')
    ax[1].plot(t, v(s.tcp_z) * 1e3, lw=.6, label='z medido')
    if 'c' in d:
        c = d['c']
        ax[1].plot(v(c.t_rel_s + c.cmd_tfs_s), v(c.cmd_tcp_z) * 1e3, lw=.6,
                   label='z comandado')
    ax[1].set_ylabel('z (mm)'); ax[1].legend(loc='upper right')
    for k in ('tcp_x', 'tcp_y'):
        ax[2].plot(t, v(s[k]) * 1e3, lw=.6, label=k)
    ax[2].set_ylabel('mm'); ax[2].legend(loc='upper right')
    ax[3].plot(t, v(s.phase), lw=.8); ax[3].set_ylabel('fase'); ax[3].set_xlabel('t (s)')
    fig.suptitle(d['run'])
    fig.tight_layout(); fig.savefig(out, dpi=90); plt.close(fig)


def analyse(run: str, do_plot: bool) -> dict:
    d = load(run)
    res = {'rates': rates(d), 'contact': contact(d), 'hold': hold(d),
           'sliding': sliding(d), 'delay': delay(d)}
    json.dump(res, open(os.path.join(run, 'char.json'), 'w'), indent=1)
    if do_plot:
        plot(d, res, os.path.join(run, 'char.png'))
    return res


def main(argv: list[str]) -> None:
    do_plot = '--plot' in argv
    runs = [a.rstrip('/') for a in argv if not a.startswith('--')]
    rows = []
    for r in runs:
        res = analyse(r, do_plot)
        print(f'== {r}\n{json.dumps(res, indent=1)}')
        sl = res['sliding']
        rows.append((r, res['contact'].get('K_sec_Nmm'), sl.get('v_cmd_mms'),
                     sl.get('v_meas_mms'), sl.get('Fn_mean_N'),
                     sl.get('Fn_std_in_stroke_N'), sl.get('mu'),
                     sl.get('peak_Hz'), (sl.get('rms_band_N') or {}).get('2-20')))
    if len(rows) > 1:
        print('\nrun | K N/mm | v cmd→med mm/s | Fn N | σ N | μ | pico Hz | rms 2–20 Hz')
        for r in rows:
            print(' | '.join('—' if v is None else
                             (f'{v:.3g}' if isinstance(v, float) else str(v))
                             for v in r))


if __name__ == '__main__':
    main(sys.argv[1:])
