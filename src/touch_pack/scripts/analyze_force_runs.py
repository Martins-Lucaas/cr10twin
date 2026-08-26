#!/usr/bin/env python3
"""
analyze_force_runs.py — Consistência do setpoint de força entre runs.

Agrega todos os `*__summary.json` de sensors/Data e responde: "o HOLD cravou
a força pedida?" — por run e no agregado, classificando cada run numa das
assinaturas de falha conhecidas:

  BOM      janela final no alvo (erro ≤ 15 %) e quieta (std ≤ 0,5 N)
  STALL    estável no valor ERRADO (std < 0,5 N, erro > 30 %) — assinatura do
           quantum de força: K_contato × passo mínimo do atuador > banda
           pedida; o regulador quase-estático estaciona (teto _QS_DF_HARD_N)
           e o timeout entrega a medição fora do alvo.
  QUIQUE   oscilação (std ≥ 1,0 N) — re-impactos contra contato rígido.
  MARGINAL nenhum dos acima.

Uso:
  python3 src/touch_pack/scripts/analyze_force_runs.py [dir_dados]
  (default: <repo>/sensors/Data)

Sem dependências além da stdlib. Ver docs/estrategia_setpoint_forca.md para
a interpretação e o plano de correção.
"""
from __future__ import annotations

import glob
import json
import os
import statistics as st
import sys


def _find_data_dir() -> str:
    if len(sys.argv) > 1:
        return sys.argv[1]
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        cand = os.path.join(d, 'sensors', 'Data')
        if os.path.isdir(cand):
            return cand
        d = os.path.dirname(d)
    return '.'


def _load_runs(data_dir: str) -> list[dict]:
    runs = []
    # Dois layouts: pasta por run (<MODO>/[<sub>/]<run_id>/summary.json) e os
    # runs antigos soltos na raiz (<ts>__summary.json). A sub-pasta opcional é
    # como os runs TOUCH ficam agrupados por frequência (TOUCH/1hz/<run_id>);
    # sem ela no glob a agregação omitia em silêncio justamente os ensaios de
    # força modulada.
    paths = (glob.glob(os.path.join(data_dir, '*', '*', 'summary.json'))
             + glob.glob(os.path.join(data_dir, '*', '*', '*', 'summary.json'))
             + glob.glob(os.path.join(data_dir, '*__summary.json')))
    for p in sorted(paths, key=os.path.getmtime):
        base = os.path.basename(p)
        ts = (os.path.basename(os.path.dirname(p)) if base == 'summary.json'
              else base.split('__')[0])
        try:
            d = json.load(open(p))
        except (OSError, json.JSONDecodeError):
            continue
        prm = d.get('params', {})
        tgt = float(d.get('target_force_n', prm.get('force_n', 0)) or 0)
        holds = [c['HOLD'] for c in d.get('cycles', {}).values() if 'HOLD' in c]
        descs = [c['DESCENDING'] for c in d.get('cycles', {}).values()
                 if 'DESCENDING' in c]
        if not holds or tgt <= 0:
            continue
        # final_window = janela final do HOLD — é a medição que interessa.
        fw = [h.get('final_window', h) for h in holds]
        fw_mean = st.mean(x.get('mean_n', float('nan')) for x in fw)
        fw_std = st.mean(x.get('std_n', float('nan')) for x in fw)
        ov = max((x.get('overshoot_n', 0) or 0) for x in descs) if descs else 0.0
        err_pct = 100.0 * abs(fw_mean - tgt) / tgt
        if err_pct <= 15.0 and fw_std <= 0.5:
            cls = 'BOM'
        elif err_pct > 30.0 and fw_std < 0.5:
            cls = 'STALL'
        elif fw_std >= 1.0:
            cls = 'QUIQUE'
        else:
            cls = 'MARGINAL'
        runs.append(dict(
            ts=ts, mode=prm.get('mode', '?'), tgt=tgt,
            vap=prm.get('approach_speed_mms', 0),
            tol=prm.get('hold_tol_n', 0) or 0.15,
            fw_mean=fw_mean, fw_std=fw_std, overshoot=ov,
            err_pct=err_pct, cls=cls, ncyc=len(holds)))
    return runs


def main() -> None:
    data_dir = _find_data_dir()
    runs = _load_runs(data_dir)
    if not runs:
        print(f'Nenhum *__summary.json com HOLD em {data_dir}')
        return

    print(f'{len(runs)} runs em {data_dir}\n')
    hdr = (f"{'ts':<16}{'md':<6}{'alvo':>5}{'vap':>5}{'ov_N':>7}"
           f"{'fw_N':>7}{'std':>6}{'err%':>7}  classe")
    print(hdr)
    print('-' * len(hdr))
    for r in runs:
        print(f"{r['ts']:<16}{r['mode']:<6}{r['tgt']:>5.1f}{r['vap']:>5.0f}"
              f"{r['overshoot']:>7.2f}{r['fw_mean']:>7.2f}{r['fw_std']:>6.2f}"
              f"{r['err_pct']:>7.1f}  {r['cls']}")

    print()
    errs = [r['err_pct'] for r in runs]
    ovs = [r['overshoot'] for r in runs]
    print(f'erro mediano da janela final : {st.median(errs):6.1f} %')
    print(f'overshoot mediano (1º toque) : {st.median(ovs):6.2f} N')
    for cls in ('BOM', 'MARGINAL', 'STALL', 'QUIQUE'):
        sel = [r for r in runs if r['cls'] == cls]
        if sel:
            print(f'{cls:<9}: {len(sel):>3} runs ({100 * len(sel) / len(runs):.0f} %)')

    n_bad = sum(r['cls'] in ('STALL', 'QUIQUE') for r in runs)
    if n_bad:
        print(
            '\nSTALL/QUIQUE dominantes indicam contato rígido demais para a '
            'banda pedida\n(quantum de força = K_contato × passo mínimo do '
            'atuador). Ação: adicionar\ncomplacência mecânica no contato — '
            'ver docs/estrategia_setpoint_forca.md.')
    # Critério de aceite do protocolo de verificação (pós-correção):
    ok = (st.median(errs) < 10.0
          and sum(r['cls'] == 'BOM' for r in runs) / len(runs) >= 0.9)
    print(f'\nCRITÉRIO DE ACEITE (err mediano <10% e ≥90% BOM): '
          f'{"APROVADO" if ok else "REPROVADO"}')


if __name__ == '__main__':
    main()
