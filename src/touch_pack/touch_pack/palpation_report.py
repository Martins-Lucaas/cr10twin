"""
palpation_report.py — Análise pós-run dos CSVs do palpation_logger.

Para cada run (samples.csv + params.json, na pasta do run — ver
constants.run_dir) produz, AO LADO deles:
  summary.json   métricas por ciclo/fase (ver compute_summary)
  plot.png       força×tempo colorido por fase (requer matplotlib;
                 sem ele o relatório gera só o JSON)

Runs no layout antigo (<ts>__samples.csv solto na raiz de sensors/Data)
continuam sendo lidos, e seus derivados continuam saindo com o nome antigo.

Métricas calculadas (por ciclo, nas fases com controle de força):
  DESCENDING  duração, força máx, overshoot vs setpoint
  HOLD        duração (≈ tempo de estabilização do setpoint), força média/
              desvio no último segundo (qualidade da estabilização)
  SLIDING     duração, força média/desvio/mín/máx, erro médio absoluto vs
              setpoint, distância lateral percorrida (via TCP da FK)

Uso (CLI):
  ros2 run touch_pack palpation_report -- --latest
  ros2 run touch_pack palpation_report -- ~/touch_pack_runs/20260611_*.csv
  ros2 run touch_pack palpation_report -- --latest --no-plot

O logger também chama generate_report() automaticamente ao fechar cada run.

Compatibilidade: lê tanto o schema novo (t_rel_s, cycle, phase, force_net_n,
q1..q6, tcp_*, touch_value, touch_age_ms) quanto os antigos (sem as colunas
de touch; ou t_rel_s, phase, fx..tz — usa |fz| como força, cycle=1, sem TCP).
Quando o run tem touch_value, cada fase ganha um bloco 'touch' com as mesmas
estatísticas da força e o plot ganha um eixo direito com o sinal do toque.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import re
import statistics
import sys

try:
    from .constants import (RUNS_DIR as OUTPUT_DIR, PHASE_NAMES,
                            RUN_SAMPLES_CSV, RUN_PARAMS_JSON,
                            RUN_SUMMARY_JSON, RUN_PLOT_PNG)
except ImportError:                       # execução standalone fora do pacote
    from constants import (RUNS_DIR as OUTPUT_DIR, PHASE_NAMES,
                           RUN_SAMPLES_CSV, RUN_PARAMS_JSON,
                           RUN_SUMMARY_JSON, RUN_PLOT_PNG)


def _sibling(csv_path: str, new_name: str, legacy_suffix: str) -> str:
    """Caminho de um arquivo IRMÃO do samples.csv.

    No layout por pasta os irmãos têm nome fixo e moram no mesmo diretório;
    no layout antigo eles são o mesmo prefixo com outro sufixo. A escolha é
    pelo NOME do CSV, não por um flag — assim um run antigo passado na linha
    de comando continua gerando os derivados com o nome dele."""
    if os.path.basename(csv_path) == RUN_SAMPLES_CSV:
        return os.path.join(os.path.dirname(csv_path), new_name)
    return csv_path.replace('__samples.csv', legacy_suffix)

# Fases com controle/medição de força — as únicas resumidas por ciclo.
# MODULATING é o trecho em que a onda trigonométrica roda: força controlada
# como as demais, e a que mais precisa aparecer no resumo.
_FORCE_PHASES = ('DESCENDING', 'HOLD', 'SLIDING', 'MODULATING')

# Cores por fase no gráfico — espelham a paleta da GUI.
_PHASE_COLORS = {
    'HOME': '#94a3b8', 'DESCENDING': '#d97706', 'HOLD': '#16a34a',
    'SLIDING': '#2563eb', 'RETRACT': '#64748b', 'MODULATING': '#9333ea',
    'IDLE': '#cbd5e1', 'DONE': '#16a34a', 'ABORTED': '#dc2626',
}


# Leitura

def _phase_name(raw: str) -> str:
    """Normaliza a fase para NOME. Aceita o schema novo (código numérico — ex.
    '2' → 'HOLD', ver PHASE_NAMES) e o antigo (string). RETRACT → HOME."""
    raw = (raw or '?').strip()
    try:
        return PHASE_NAMES.get(int(raw), '?')
    except ValueError:
        return 'HOME' if raw == 'RETRACT' else raw


# Colunas de taxel do CSV: `taxel_<índice>`. O prefixo sozinho NÃO serve —
# `taxel_age_ms` também começa com ele, e entrava na conta como se fosse um
# canal. Uma idade em milissegundos (≈ 1) misturada a leituras de ADC (≈ 4000)
# puxava a média e cravava o mínimo da matriz perto de zero em toda amostra.
_TAXEL_RE = re.compile(r'^taxel_\d+$')


def _taxel_values(r: dict, taxel_keys: list[str]) -> list[float]:
    """Leituras cruas dos taxels daquela linha (vazio se não houve frame)."""
    vals = []
    for k in taxel_keys:
        v = r.get(k)
        if v:
            try:
                vals.append(float(v))
            except (TypeError, ValueError):
                pass
    return vals


def _touch_proxy(r: dict, taxel_keys: list[str]) -> float | None:
    """Escalar do toque para o summary: a coluna touch_value (schema antigo)
    ou a média dos taxels do frame ADC. None se a linha não trouxe amostra."""
    if r.get('touch_value'):
        try:
            return float(r['touch_value'])
        except (TypeError, ValueError):
            return None
    vals = _taxel_values(r, taxel_keys)
    return (sum(vals) / len(vals)) if vals else None


def _load_rows(csv_path: str) -> list[dict]:
    """Lê o CSV num formato normalizado:
    [{t, cycle, phase, force, tcp(x,y,z)|None, touch|None}, ...]"""
    rows: list[dict] = []
    with open(csv_path, newline='') as fh:
        reader = csv.DictReader(fh)
        fields = reader.fieldnames or []
        new_schema = 'force_net_n' in fields
        taxel_keys = [f for f in fields if _TAXEL_RE.match(f)]
        for r in reader:
            try:
                t = float(r['t_rel_s'])
                if new_schema:
                    force = float(r['force_net_n'])
                    cycle = int(r.get('cycle') or 1) or 1
                    tcp = None
                    if r.get('tcp_x'):
                        tcp = (float(r['tcp_x']), float(r['tcp_y']),
                               float(r['tcp_z']))
                else:   # schema antigo: wrench — |fz| era a força normal
                    force = abs(float(r['fz']))
                    cycle = 1
                    tcp = None
                touch = _touch_proxy(r, taxel_keys)
                taxels = _taxel_values(r, taxel_keys)
            except (KeyError, TypeError, ValueError):
                continue
            # setpoint_n é OPCIONAL: não existe nos CSVs antigos e vem vazio
            # nas amostras anteriores ao primeiro setpoint do run. Ausência
            # não invalida a linha — só cai para a referência estática.
            sp_raw = r.get('setpoint_n')
            try:
                sp = float(sp_raw) if sp_raw not in (None, '') else None
            except (TypeError, ValueError):
                sp = None
            rows.append({'t': t, 'cycle': cycle,
                         'phase': _phase_name(r.get('phase', '?')),
                         'force': force, 'sp': sp, 'tcp': tcp,
                         'touch': touch, 'taxels': taxels})
    return rows


def _load_params(csv_path: str) -> dict:
    params_path = _sibling(csv_path, RUN_PARAMS_JSON, '__params.json')
    try:
        with open(params_path) as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def _segments(rows: list[dict]) -> list[dict]:
    """Agrupa amostras contíguas de mesma (cycle, phase)."""
    segs: list[dict] = []
    for row in rows:
        if (segs and segs[-1]['phase'] == row['phase']
                and segs[-1]['cycle'] == row['cycle']):
            segs[-1]['rows'].append(row)
        else:
            segs.append({'cycle': row['cycle'], 'phase': row['phase'],
                         'rows': [row]})
    return segs


# Métricas

def _stats(forces: list[float]) -> dict:
    return {
        'mean_n': round(statistics.fmean(forces), 3),
        'std_n':  round(statistics.pstdev(forces), 3) if len(forces) > 1 else 0.0,
        'min_n':  round(min(forces), 3),
        'max_n':  round(max(forces), 3),
        'n_samples': len(forces),
    }


def _seg_summary(seg: dict, target: float | None) -> dict:
    rows = seg['rows']
    forces = [r['force'] for r in rows]
    t0, t1 = rows[0]['t'], rows[-1]['t']
    out = {'duration_s': round(t1 - t0, 2), **_stats(forces)}

    # Referência do erro: o setpoint DAQUELA amostra (coluna setpoint_n), que
    # acompanha modulação senoidal, escada de degraus e mudanças on-the-fly do
    # modo MANUAL. O force_n do params.json é um valor ÚNICO do início do run e
    # só serve de reserva para CSVs antigos, sem a coluna: usá-lo num run
    # modulado mede o erro contra uma reta no meio de uma senoide, e devolve a
    # AMPLITUDE da onda disfarçada de erro.
    pairs = [(r['force'], r['sp']) for r in rows if r.get('sp') is not None]
    if pairs:
        out['setpoint_ref'] = 'per_sample'
        sps = [s for _, s in pairs]
        lo, hi = min(sps), max(sps)
        if hi - lo > 1e-6:
            # Setpoint que VARIA: o ensaio não persegue um alvo, percorre uma
            # FAIXA. Reportá-la é o que deixa o segmento legível sozinho e
            # impede ler o std_n da força como erro quando ele é a excursão.
            out['setpoint'] = {
                'min_n': round(lo, 3), 'max_n': round(hi, 3),
                'span_n': round(hi - lo, 3),
                'mean_n': round(statistics.fmean(sps), 3),
            }
        else:
            out['target_n'] = round(lo, 3)
        if seg['phase'] == 'DESCENDING':
            out['overshoot_n'] = round(max(f - s for f, s in pairs), 3)
        if seg['phase'] == 'SLIDING':
            out['mae_vs_setpoint_n'] = round(
                statistics.fmean(abs(f - s) for f, s in pairs), 3)
    elif target is not None:
        out['setpoint_ref'] = 'static_params'
        out['target_n'] = target
        if seg['phase'] == 'DESCENDING':
            out['overshoot_n'] = round(max(forces) - target, 3)
        if seg['phase'] == 'SLIDING':
            out['mae_vs_setpoint_n'] = round(
                statistics.fmean(abs(f - target) for f in forces), 3)
    if seg['phase'] == 'HOLD':
        # Qualidade da estabilização: estatística do último segundo do HOLD
        # (a janela que o critério de _HOLD_STABLE_S validou).
        tail = [r['force'] for r in rows if r['t'] >= t1 - 1.0]
        if tail:
            out['final_window'] = _stats(tail)
    if seg['phase'] == 'SLIDING':
        tcps = [r['tcp'] for r in rows if r['tcp'] is not None]
        if len(tcps) >= 2:
            dx = tcps[-1][0] - tcps[0][0]
            dy = tcps[-1][1] - tcps[0][1]
            out['lateral_dist_mm'] = round(math.hypot(dx, dy) * 1e3, 1)
    touch_vals = [r['touch'] for r in rows if r.get('touch') is not None]
    if touch_vals:
        out['touch'] = _stats(touch_vals)
    return out


def compute_summary(rows: list[dict], params: dict) -> dict:
    target = params.get('force_n')
    target = float(target) if target is not None else None

    cycles: dict[int, dict] = {}
    for seg in _segments(rows):
        if seg['phase'] not in _FORCE_PHASES or not seg['rows']:
            continue
        cyc = cycles.setdefault(seg['cycle'], {})
        # Fases repetidas no mesmo ciclo (não deveria ocorrer) ganham sufixo.
        key = seg['phase']
        k = 2
        while key in cyc:
            key = f'{seg["phase"]}_{k}'; k += 1
        cyc[key] = _seg_summary(seg, target)

    summary: dict = {
        'n_samples': len(rows),
        'duration_s': round(rows[-1]['t'] - rows[0]['t'], 2) if rows else 0.0,
        'cycles_detected': len(cycles),
        'target_force_n': target,
        'params': params,
        'cycles': {str(c): cycles[c] for c in sorted(cycles)},
    }

    # Repetibilidade entre ciclos: força média do SLIDING por ciclo.
    sl_means = [c['SLIDING']['mean_n'] for c in cycles.values()
                if 'SLIDING' in c]
    if len(sl_means) >= 2:
        summary['sliding_repeatability'] = {
            'mean_of_means_n': round(statistics.fmean(sl_means), 3),
            'std_of_means_n': round(statistics.pstdev(sl_means), 3),
        }
    return summary


# Gráfico

def _target_band(params: dict) -> tuple[float, float] | None:
    """Faixa de força ALVO do ensaio modulado (f_min, f_max), do params.json.

    Num run modulado não existe um alvo único: o que caracteriza o ensaio é a
    EXCURSÃO pedida entre os dois limiares. Desenhá-la como banda é o que
    permite ver, de relance, se a onda entregue cobriu a faixa ou ficou
    achatada no meio dela."""
    shape = str(params.get('force_mod_shape') or '').upper().strip()
    if shape in ('', 'OFF'):
        return None
    try:
        lo = float(params['force_mod_min_n'])
        hi = float(params['force_mod_max_n'])
    except (KeyError, TypeError, ValueError):
        return None
    return (lo, hi) if hi > lo else None


def _plot_force(ax, rows: list[dict], summary: dict) -> list:
    """Painel da força: curva colorida por fase + limiares alvo. Devolve os
    handles da legenda."""
    from matplotlib.patches import Patch

    phases_seen: list[str] = []
    for seg in _segments(rows):
        srows = seg['rows']
        color = _PHASE_COLORS.get(seg['phase'], '#0f172a')
        ax.plot([r['t'] for r in srows], [r['force'] for r in srows],
                color=color, linewidth=1.0)
        if seg['phase'] not in phases_seen:
            phases_seen.append(seg['phase'])

    handles = [Patch(color=_PHASE_COLORS.get(p, '#0f172a'), label=p)
               for p in phases_seen]

    # Banda dos limiares pedidos (só no ensaio modulado), desenhada ATRÁS de
    # tudo para não competir com a curva.
    band = _target_band(summary.get('params') or {})
    if band:
        lo, hi = band
        ax.axhspan(lo, hi, color='#dc2626', alpha=0.08, zorder=0)
        for y in (lo, hi):
            ax.axhline(y, color='#dc2626', linestyle=':', linewidth=0.8,
                       alpha=0.6, zorder=1)
        handles.append(Patch(color='#dc2626', alpha=0.25,
                             label=f'faixa alvo {lo:g}–{hi:g} N'))

    # Setpoint: SÉRIE real quando ele varia (senoide, escada, MANUAL); reta só
    # quando é mesmo constante. Uma axhline sobre uma senoide desenharia um
    # alvo que nunca existiu, e era isso que fazia o gráfico parecer um run
    # com erro enorme.
    target = summary.get('target_force_n')
    sp_pts = [(r['t'], r['sp']) for r in rows if r.get('sp') is not None]
    sp_varies = (len(sp_pts) > 1
                 and (max(p[1] for p in sp_pts)
                      - min(p[1] for p in sp_pts)) > 1e-6)
    if sp_varies:
        ax.plot([p[0] for p in sp_pts], [p[1] for p in sp_pts],
                color='#dc2626', linestyle='--', linewidth=0.9,
                label='setpoint (por amostra)')
        handles += ax.get_legend_handles_labels()[0]
    elif target is not None:
        ax.axhline(target, color='#dc2626', linestyle='--',
                   linewidth=0.9, label=f'setpoint {target:g} N')
        handles += ax.get_legend_handles_labels()[0]

    ax.set_ylabel('força de compressão (N)')
    ax.grid(alpha=0.25)
    return handles


# Força abaixo da qual a sonda ainda não está na peça — as amostras que
# definem o repouso de cada taxel.
_TOUCH_BASE_FORCE_N = 0.05


def _taxel_baseline(rows: list[dict], n_ch: int) -> list[float] | None:
    """Repouso de CADA taxel: sua mediana nas amostras anteriores ao contato.

    Cada canal tem seu próprio offset (aqui variam de ~3700 a ~4094 contagens),
    então o valor cru de um taxel não diz nada sozinho — o que mede o toque é o
    quanto ele se afastou do PRÓPRIO repouso. Sem essa referência a matriz
    aparece como uma faixa larga e imóvel, com os canais que não respondem
    fixando os extremos e escondendo os que respondem."""
    pre = [r['taxels'] for r in rows
           if r.get('taxels') and len(r['taxels']) == n_ch
           and r['force'] < _TOUCH_BASE_FORCE_N]
    if len(pre) < 5:
        return None
    return [statistics.median([f[i] for f in pre]) for i in range(n_ch)]


def _plot_touch(ax, rows: list[dict]) -> None:
    """Painel do toque: resposta da matriz relativa ao repouso de cada taxel —
    média dos canais e envelope do canal mais e do menos solicitado."""
    frames = [r for r in rows if r.get('taxels')]
    if not frames:
        return
    n_ch = max(len(r['taxels']) for r in frames)
    frames = [r for r in frames if len(r['taxels']) == n_ch]
    base = _taxel_baseline(frames, n_ch)

    t = [r['t'] for r in frames]
    if base is None:
        # Sem trecho pré-contato (run começado já encostado): resta o valor
        # cru. O eixo fica na escala do ADC e o rótulo diz isso.
        series = [r['taxels'] for r in frames]
        ax.set_ylabel('toque cru (u.a.)')
    else:
        series = [[v - b for v, b in zip(r['taxels'], base)] for r in frames]
        ax.axhline(0.0, color='#94a3b8', linewidth=0.8)
        ax.set_ylabel('toque − repouso (u.a.)')

    ax.fill_between(t, [min(s) for s in series], [max(s) for s in series],
                    color='#7c3aed', alpha=0.20, linewidth=0,
                    label=f'envelope dos {n_ch} taxels')
    ax.plot(t, [sum(s) / len(s) for s in series],
            color='#7c3aed', linewidth=0.9, label='média dos taxels')
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, ncol=2, loc='upper right')


def _make_plot(rows: list[dict], summary: dict, out_png: str) -> bool:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    # O toque ganha um PAINEL PRÓPRIO, não um eixo gêmeo: na escala crua do
    # STM32 (offset de milhares de contagens) a curva do toque atravessava o
    # painel inteiro e cobria a força, que é o dado principal do run.
    has_touch = any(r.get('touch') is not None for r in rows)
    if has_touch:
        fig, (ax_f, ax_t) = plt.subplots(
            2, 1, figsize=(11, 6.5), dpi=110, sharex=True,
            gridspec_kw={'height_ratios': [2, 1]})
    else:
        fig, ax_f = plt.subplots(figsize=(11, 4.5), dpi=110)
        ax_t = None

    handles = _plot_force(ax_f, rows, summary)
    if ax_t is not None:
        _plot_touch(ax_t, rows)
        ax_t.set_xlabel('tempo (s)')
    else:
        ax_f.set_xlabel('tempo (s)')

    target = summary.get('target_force_n')
    sp_pts = [r['sp'] for r in rows if r.get('sp') is not None]
    n_cyc = summary.get('cycles_detected', 0)
    date = os.path.basename(out_png).split('__')[0]
    band = _target_band(summary.get('params') or {})
    if band:
        sp = f'  —  faixa alvo {band[0]:g}–{band[1]:g} N'
    elif len(sp_pts) > 1 and (max(sp_pts) - min(sp_pts)) > 1e-6:
        sp = f'  —  setpoint {min(sp_pts):g}–{max(sp_pts):g} N'
    else:
        sp = f'  —  setpoint {target:g} N' if target is not None else ''
    ax_f.set_title(f'Palpação — {date}{sp}'
                   + (f'  ({n_cyc} ciclos)' if n_cyc > 1 else ''))
    ax_f.legend(handles=handles, fontsize=8, ncol=min(6, len(handles)),
                loc='upper right')
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)
    return True


# API pública + CLI

def generate_report(csv_path: str, make_plot: bool = True) -> dict:
    """Gera o summary.json (+ plot.png) ao lado do CSV do run.
    Retorna o summary (com as chaves extras summary_path/plot_path)."""
    rows = _load_rows(csv_path)
    if not rows:
        raise ValueError(f'CSV sem amostras válidas: {csv_path}')
    params = _load_params(csv_path)
    summary = compute_summary(rows, params)

    summary_path = _sibling(csv_path, RUN_SUMMARY_JSON, '__summary.json')
    with open(summary_path, 'w') as fh:
        json.dump(summary, fh, indent=2, sort_keys=True)
    summary['summary_path'] = summary_path

    if make_plot:
        plot_path = _sibling(csv_path, RUN_PLOT_PNG, '__plot.png')
        if _make_plot(rows, summary, plot_path):
            summary['plot_path'] = plot_path
    return summary


def _print_summary(summary: dict) -> None:
    tgt = summary.get('target_force_n')
    print(f'  amostras: {summary["n_samples"]}  '
          f'duração: {summary["duration_s"]:.1f}s  '
          f'ciclos: {summary["cycles_detected"]}  '
          f'setpoint: {tgt if tgt is not None else "?"} N')
    for cyc, phases in summary.get('cycles', {}).items():
        for phase, m in phases.items():
            extra = ''
            if 'setpoint' in m:
                s = m['setpoint']
                extra = (f'  sp={s["min_n"]:.2f}–{s["max_n"]:.2f}N'
                         f' (faixa {s["span_n"]:.2f}N)')
            if 'overshoot_n' in m:
                extra += f'  overshoot={m["overshoot_n"]:+.2f}N'
            if 'mae_vs_setpoint_n' in m:
                extra += f'  MAE={m["mae_vs_setpoint_n"]:.2f}N'
            if 'lateral_dist_mm' in m:
                extra += f'  percurso={m["lateral_dist_mm"]:.0f}mm'
            if 'touch' in m:
                extra += (f'  touch={m["touch"]["mean_n"]:.2f}'
                          f'±{m["touch"]["std_n"]:.2f}u.a.')
            print(f'    ciclo {cyc} {phase:<11} {m["duration_s"]:5.1f}s  '
                  f'F={m["mean_n"]:.2f}±{m["std_n"]:.2f}N '
                  f'[{m["min_n"]:.2f}, {m["max_n"]:.2f}]{extra}')
    rep = summary.get('sliding_repeatability')
    if rep:
        print(f'    repetibilidade SLIDING: '
              f'{rep["mean_of_means_n"]:.2f} ± {rep["std_of_means_n"]:.2f} N '
              '(desvio entre ciclos)')


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Gera summary JSON + gráfico de runs de palpação.')
    parser.add_argument('csvs', nargs='*',
                        help='CSV(s) de run (samples.csv)')
    parser.add_argument('--latest', action='store_true',
                        help=f'usa o run mais recente de {OUTPUT_DIR}')
    parser.add_argument('--no-plot', action='store_true',
                        help='gera apenas o summary JSON')
    args = parser.parse_args(argv)

    paths = list(args.csvs)
    if args.latest or not paths:
        # Os dois layouts: pasta por run (<MODO>/<run_id>/samples.csv) e os
        # runs antigos soltos na raiz. Ordena por mtime porque os nomes das
        # duas famílias não se comparam entre si.
        candidates = sorted(
            glob.glob(os.path.join(OUTPUT_DIR, '*', '*', RUN_SAMPLES_CSV))
            + glob.glob(os.path.join(OUTPUT_DIR, '*__samples.csv')),
            key=os.path.getmtime)
        if not candidates:
            sys.exit(f'Nenhum run encontrado em {OUTPUT_DIR}.')
        paths = [candidates[-1]]

    for path in paths:
        print(f'▶ {os.path.relpath(path, OUTPUT_DIR)}')
        try:
            summary = generate_report(path, make_plot=not args.no_plot)
        except (OSError, ValueError) as exc:
            print(f'  ERRO: {exc}')
            continue
        _print_summary(summary)
        print(f'  → {os.path.relpath(summary["summary_path"], OUTPUT_DIR)}'
              + (f'  +  {os.path.basename(summary["plot_path"])}'
                 if 'plot_path' in summary else '  (sem matplotlib: só JSON)'))


if __name__ == '__main__':
    main()
