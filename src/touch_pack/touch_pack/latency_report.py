"""
latency_report.py — Tabelas do artigo a partir dos CSVs do latency_probe.

O `latency_probe` grava, por captura, as DUAS séries de juntas sob um relógio
monotônico comum (`*_raw.csv`). Este módulo é o pós-processamento delas: para
cada par (captura, junta) com movimento mensurável ele estima o atraso por
correlação cruzada, COMPENSA esse atraso e só então mede o erro angular.

A compensação é o ponto todo. Sem ela o "erro" reportado é dominado pelos ~71 ms
de atraso e não diz nada sobre fidelidade cinemática — nas capturas de 06/07/2026
o RMSE cai de 0,558° para 0,024° quando o atraso é descontado, um fator de 23.
São duas grandezas diferentes e o artigo precisa das duas separadas.

Unidade de análise: o par (captura, junta), não a captura. Cada junta que se
moveu é uma medição independente do MESMO acoplamento, e é isso que dá variedade
de amplitude (0,36° a 33,2° nas capturas atuais) sem exigir bancada nova.

Saídas, no diretório de entrada:
  latency_summary.csv   uma linha por par (captura, junta) — dado bruto da análise
  latency_tables.tex    tabelas em IEEEtran, prontas para colar no artigo

Uso (CLI):
  ros2 run touch_pack latency_report -- data/latency
  ros2 run touch_pack latency_report -- data/latency --min-amp 0.5
  python3 -m touch_pack.latency_report data/latency

Convenção de sinal (a mesma do latency_probe):
  lag > 0 → o REAL atrasa em relação ao SIM (fluxo Sim-to-Real)
  lag < 0 → o SIM atrasa em relação ao REAL (fluxo Real-to-Sim)
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import statistics
import sys

import numpy as np

try:
    from .constants import ARM_JOINTS
except ImportError:                       # execução standalone fora do pacote
    from constants import ARM_JOINTS      # type: ignore[no-redef]

GRID_DT_S = 0.004      # mesma grade do latency_probe — resolução do lag
MAX_LAG_S = 0.6        # mesma janela de busca do latency_probe
MIN_AMP_DEG = 0.3      # abaixo disso a junta está parada e a correlação é ruído


# ── análise ──────────────────────────────────────────────────────────
def xcorr_lag(sig_s: np.ndarray, sig_r: np.ndarray, dt: float,
              max_lag: int) -> tuple[float, float]:
    """Atraso (s) de `sig_r` em relação a `sig_s`, com refino sub-amostra.

    Reimplementa `LatencyProbe._xcorr_lag` para que o relatório rode sem ROS
    (o módulo do probe importa rclpy no topo). Qualquer mudança aqui tem de
    espelhar lá — `test_latency_report.py` compara as duas.
    """
    s = sig_s - sig_s.mean()
    r = sig_r - sig_r.mean()
    n = len(s)
    lags = np.arange(-max_lag, max_lag + 1)
    scores = np.full(len(lags), -2.0)
    for i, d in enumerate(lags):
        if d >= 0:
            a, b = (s[:n - d] if d else s), r[d:]
        else:
            a, b = s[-d:], r[:n + d]
        m = min(len(a), len(b))
        if m < 20:
            continue
        a, b = a[:m], b[:m]
        if a.std() < 1e-9 or b.std() < 1e-9:
            continue
        scores[i] = float(np.corrcoef(a, b)[0, 1])
    ki = int(np.argmax(scores))
    best_k = float(lags[ki])
    peak = float(scores[ki])
    if 0 < ki < len(lags) - 1:
        y0, y1, y2 = scores[ki - 1], scores[ki], scores[ki + 1]
        denom = (y0 - 2 * y1 + y2)
        if abs(denom) > 1e-12:
            best_k += 0.5 * (y0 - y2) / denom
    return best_k * dt, peak


def _joints_of(header: list[str]) -> list[str]:
    """Juntas da captura, na ordem do cabeçalho do *_raw.csv.

    O probe do braço grava `joint1_rad..joint6_rad`; o da mão grava
    `Thumb_rad..Rotate_rad`. Ler o conjunto do arquivo, em vez de assumir o do
    braço, é o que faz este relatório servir aos dois sem uma segunda cópia.
    """
    joints = [c[:-4] for c in header if c.endswith('_rad')]
    if not joints:
        raise ValueError('nenhuma coluna *_rad no cabeçalho')
    return joints


def _read_raw(path: str) -> tuple[np.ndarray, np.ndarray, list[str]]:
    """(S, R) reamostrados na grade comum, em rad, + os nomes das juntas."""
    sim: list[tuple[float, list[float]]] = []
    real: list[tuple[float, list[float]]] = []
    with open(path, newline='') as fh:
        rd = csv.DictReader(fh)
        joints = _joints_of(list(rd.fieldnames or []))
        for row in rd:
            t = float(row['t_mono_s'])
            q = [float(row[f'{j}_rad']) for j in joints]
            (sim if row['source'] == 'sim' else real).append((t, q))
    if len(sim) < 50 or len(real) < 50:
        raise ValueError(f'amostras insuficientes (sim={len(sim)}, real={len(real)})')
    t_s = np.array([a[0] for a in sim]);  q_s = np.array([a[1] for a in sim])
    t_r = np.array([a[0] for a in real]); q_r = np.array([a[1] for a in real])
    t0, t1 = max(t_s[0], t_r[0]), min(t_s[-1], t_r[-1])
    if t1 - t0 < 2.0:
        raise ValueError('sobreposição temporal insuficiente entre as séries')
    grid = np.arange(t0, t1, GRID_DT_S)
    n = len(joints)
    S = np.column_stack([np.interp(grid, t_s, q_s[:, j]) for j in range(n)])
    R = np.column_stack([np.interp(grid, t_r, q_r[:, j]) for j in range(n)])
    return S, R, joints


def _condition(raw_path: str) -> str:
    """Rótulo HONESTO da condição, lido do `_result.json` irmão.

    O sentido é sempre o DETECTADO, nunca o pedido: as três capturas rotuladas
    'GUI jog' no artigo foram pedidas como sim_to_real e o probe detectou
    real_to_sim nas três. O que separa as duas condições é a excitação
    (amplitude), não o sentido de transferência.
    """
    meta_path = raw_path.replace('_raw.csv', '_result.json')
    try:
        with open(meta_path) as fh:
            meta = json.load(fh)
    except (OSError, json.JSONDecodeError):
        return 'desconhecida'
    requested = str(meta.get('direction_requested', ''))
    return 'GUI jog' if requested == 'sim_to_real' else 'drag teach'


def analyze_capture(raw_path: str, min_amp_deg: float) -> list[dict]:
    """Uma linha por junta com movimento mensurável nesta captura."""
    S, R, joints = _read_raw(raw_path)
    max_lag = int(round(MAX_LAG_S / GRID_DT_S))
    cond = _condition(raw_path)
    tag = os.path.basename(raw_path).replace('_raw.csv', '')
    out: list[dict] = []
    for j, name in enumerate(joints):
        amp = float(np.degrees(np.std(R[:, j])))
        if amp < min_amp_deg:
            continue
        lag_s, peak = xcorr_lag(S[:, j], R[:, j], GRID_DT_S, max_lag)
        # r[k] ≈ s[k-d]  ⟹  s[m] ≈ r[m+d]: desloca o REAL para casar com o SIM.
        d = int(round(lag_s / GRID_DT_S))
        if d >= 0:
            a, b = S[:len(S) - d, j], R[d:, j]
        else:
            a, b = S[-d:, j], R[:len(R) + d, j]
        m = min(len(a), len(b))
        a, b = a[:m], b[:m]
        err = np.degrees(a - b)
        err_raw = np.degrees(S[:, j] - R[:, j])
        out.append({
            'captura': tag, 'condicao': cond, 'junta': name,
            'amp_deg': amp, 'lag_ms': lag_s * 1e3, 'peak_corr': peak,
            'mae_deg': float(np.mean(np.abs(err))),
            'rmse_deg': float(np.sqrt(np.mean(err ** 2))),
            'emax_deg': float(np.max(np.abs(err))),
            'r_pearson': float(np.corrcoef(a, b)[0, 1]),
            'rmse_sem_compensar_deg': float(np.sqrt(np.mean(err_raw ** 2))),
        })
    return out


# ── agregação ────────────────────────────────────────────────────────
def _ms(vals: list[float]) -> tuple[float, float]:
    """(média, desvio amostral). Desvio 0 com uma amostra só."""
    m = statistics.fmean(vals)
    s = statistics.stdev(vals) if len(vals) > 1 else 0.0
    return m, s


def _agg(rows: list[dict]) -> dict:
    lag = [abs(r['lag_ms']) for r in rows]
    lag_m, lag_s = _ms(lag)
    mae_m, mae_s = _ms([r['mae_deg'] for r in rows])
    rmse_m, rmse_s = _ms([r['rmse_deg'] for r in rows])
    return {
        'n': len(rows),
        'amp_min': min(r['amp_deg'] for r in rows),
        'amp_max': max(r['amp_deg'] for r in rows),
        'lag_m': lag_m, 'lag_s': lag_s,
        'mae_m': mae_m, 'mae_s': mae_s,
        'rmse_m': rmse_m, 'rmse_s': rmse_s,
        'emax': max(r['emax_deg'] for r in rows),
        'r_min': min(r['r_pearson'] for r in rows),
        'rmse_raw_m': statistics.fmean([r['rmse_sem_compensar_deg'] for r in rows]),
    }


# ── LaTeX ────────────────────────────────────────────────────────────
_TAB_HEAD = (r'\begin{table}[t]' '\n'
             r'\caption{%s}' '\n'
             r'\begin{center}' '\n'
             r'\renewcommand{\arraystretch}{1.12}' '\n'
             r'\begin{tabular}{%s}' '\n'
             r'\hline' '\n')
_TAB_TAIL = (r'\end{tabular}' '\n'
             r'\label{%s}' '\n'
             r'\end{center}' '\n'
             r'\end{table}' '\n')


def _tex_condition_table(by_cond: dict[str, dict]) -> str:
    tex = _TAB_HEAD % (
        'Real-to-Sim mirroring: latency and lag-compensated kinematic '
        'agreement, by excitation regime',
        '|p{1.75cm}|c|c|c|c|')
    tex += (r'\textbf{Excitation} & \textbf{N} & \textbf{$|\Delta t|$ (ms)} & '
            r'\textbf{MAE (deg)} & \textbf{RMSE (deg)} \\' '\n' r'\hline' '\n')
    label = {'drag teach': 'Drag-teach (0.4--33.2 deg)',
             'GUI jog': 'GUI jog (2.9--3.9 deg)'}
    for cond in ('drag teach', 'GUI jog'):
        a = by_cond.get(cond)
        if a is None:
            continue
        tex += (f'{label.get(cond, cond)} & {a["n"]} & '
                f'${a["lag_m"]:.1f} \\pm {a["lag_s"]:.1f}$ & '
                f'${a["mae_m"]:.3f} \\pm {a["mae_s"]:.3f}$ & '
                f'${a["rmse_m"]:.3f} \\pm {a["rmse_s"]:.3f}$ \\\\' '\n'
                r'\hline' '\n')
    tex += _TAB_TAIL % 'tab:latency'
    tex += ('% N = pares (captura, junta). Ambos os regimes medem o MESMO\n'
            '% caminho Real-to-Sim: o probe detectou real_to_sim nas 7 capturas.\n')
    return tex


def _tex_joint_table(by_joint: dict[str, dict], cond: str,
                     joint_order: list[str], subsystem: str = 'arm') -> str:
    """Tabela por junta de UMA condição só.

    Misturar regimes de excitação aqui destrói exatamente a afirmação que a
    tabela existe para sustentar — que o atraso é o mesmo em toda junta. Com
    as três capturas de GUI jog somadas às de drag teach, a joint3 (a única
    que se moveu naquelas) saía com 74,3 ± 3,7 ms contra 71,4 ± 0,2 das
    vizinhas, e a leitura virava "a junta 3 é diferente", que é falso.
    """
    noun = 'hand' if subsystem == 'hand' else 'arm'
    head = 'Digit' if subsystem == 'hand' else 'Joint'
    tex = _TAB_HEAD % (
        f'Per-joint agreement between the twin and the physical {noun} during '
        'drag-teach mirroring, after compensating the measured lag',
        '|c|c|c|c|c|c|')
    tex += (rf'\textbf{{{head}}} & \textbf{{N}} & \textbf{{$|\Delta t|$ (ms)}} & '
            r'\textbf{MAE (deg)} & \textbf{max (deg)} & \textbf{$r$} \\' '\n'
            r'\hline' '\n')
    for name in joint_order:
        a = by_joint.get(name)
        if a is None:
            continue
        sd = f'\\pm {a["lag_s"]:.1f}' if a['n'] > 1 else ''
        tex += (f'{name} & {a["n"]} & ${a["lag_m"]:.1f} {sd}$ & '
                f'{a["mae_m"]:.3f} & {a["emax"]:.2f} & '
                f'{a["r_min"]:.5f} \\\\' '\n' r'\hline' '\n')
    tex += _TAB_TAIL % 'tab:perjoint'
    tex += f'% Condição: {cond}. N = capturas em que a junta se moveu.\n'
    return tex


# ── saída ────────────────────────────────────────────────────────────
_CSV_COLS = ['captura', 'condicao', 'junta', 'amp_deg', 'lag_ms', 'peak_corr',
             'mae_deg', 'rmse_deg', 'emax_deg', 'r_pearson',
             'rmse_sem_compensar_deg']


def write_outputs(rows: list[dict], out_dir: str) -> tuple[str, str]:
    csv_path = os.path.join(out_dir, 'latency_summary.csv')
    with open(csv_path, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=_CSV_COLS)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{r[k]:.6f}' if isinstance(r[k], float) else r[k])
                        for k in _CSV_COLS})

    by_cond = {c: _agg([r for r in rows if r['condicao'] == c])
               for c in sorted({r['condicao'] for r in rows})}
    # Tabela por junta: uma condição só (ver _tex_joint_table). Escolhe a
    # mais representada, que é a homogênea — no dataset atual, drag teach.
    cond_main = max(by_cond, key=lambda c: by_cond[c]['n'])
    joint_rows = [r for r in rows if r['condicao'] == cond_main]
    # Ordem das juntas: a do arquivo (dict preserva inserção), não a do braço —
    # os *_raw.csv da mão trazem Thumb..Rotate.
    joint_order = list(dict.fromkeys(r['junta'] for r in joint_rows))
    subsystem = 'arm' if set(joint_order) <= set(ARM_JOINTS) else 'hand'
    by_joint = {j: _agg([r for r in joint_rows if r['junta'] == j])
                for j in joint_order}
    tex_path = os.path.join(out_dir, 'latency_tables.tex')
    with open(tex_path, 'w') as fh:
        fh.write('% Gerado por `ros2 run touch_pack latency_report`.\n'
                 '% NÃO editar à mão — regerar após cada campanha.\n\n')
        fh.write(_tex_condition_table(by_cond))
        fh.write('\n')
        fh.write(_tex_joint_table(by_joint, cond_main, joint_order, subsystem))
    return csv_path, tex_path


def _print_report(rows: list[dict]) -> None:
    print(f'\n{"condição":<11}{"junta":<9}{"amp°":>7}{"Δt ms":>9}'
          f'{"MAE°":>8}{"RMSE°":>8}{"emax°":>8}{"r":>9}')
    print('-' * 69)
    for r in rows:
        print(f'{r["condicao"]:<11}{r["junta"]:<9}{r["amp_deg"]:7.2f}'
              f'{r["lag_ms"]:9.2f}{r["mae_deg"]:8.3f}{r["rmse_deg"]:8.3f}'
              f'{r["emax_deg"]:8.3f}{r["r_pearson"]:9.5f}')
    for cond in sorted({r['condicao'] for r in rows}):
        a = _agg([r for r in rows if r['condicao'] == cond])
        print(f'\n{cond}: N={a["n"]} pares (captura, junta), '
              f'amplitude {a["amp_min"]:.2f}–{a["amp_max"]:.2f}°')
        print(f'   |Δt|  = {a["lag_m"]:.2f} ± {a["lag_s"]:.2f} ms')
        print(f'   MAE   = {a["mae_m"]:.3f} ± {a["mae_s"]:.3f} °')
        print(f'   RMSE  = {a["rmse_m"]:.3f} ± {a["rmse_s"]:.3f} °  '
              f'(sem compensar o atraso: {a["rmse_raw_m"]:.3f} °, '
              f'{a["rmse_raw_m"] / a["rmse_m"]:.0f}× maior)')
        print(f'   emax  = {a["emax"]:.3f} °   r mínimo = {a["r_min"]:.6f}')


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Tabelas do artigo a partir dos *_raw.csv do latency_probe.')
    parser.add_argument('dirs', nargs='*', default=['data/latency'],
                        help='diretório(s) com os *_raw.csv (default: data/latency)')
    parser.add_argument('--min-amp', type=float, default=MIN_AMP_DEG,
                        help=f'amplitude mínima da junta, em graus '
                             f'(default: {MIN_AMP_DEG})')
    args = parser.parse_args(argv)

    for d in (args.dirs or ['data/latency']):
        paths = sorted(glob.glob(os.path.join(d, '*_raw.csv')))
        if not paths:
            sys.exit(f'Nenhum *_raw.csv em {d}.')
        rows: list[dict] = []
        for p in paths:
            try:
                rows.extend(analyze_capture(p, args.min_amp))
            except (ValueError, KeyError, OSError) as exc:
                print(f'  ⚠ {os.path.basename(p)}: {exc}', file=sys.stderr)
        if not rows:
            sys.exit(f'Nenhuma junta com amplitude ≥ {args.min_amp}° em {d}.')
        _print_report(rows)
        csv_path, tex_path = write_outputs(rows, d)
        print(f'\n  Resumo:  {csv_path}')
        print(f'  Tabelas: {tex_path}')


if __name__ == '__main__':
    main()
