#!/usr/bin/env python3
"""
migrate_runs_layout.py — Reorganiza os runs antigos no layout por pasta.

De (tudo solto numa pasta só, duas convenções de nome):
    sensors/Data/20260812_143012__samples.csv
    sensors/Data/20260812_143012__params.json
    sensors/Data/adc_20260812_143012.csv
    ...
Para (uma pasta por MODO, uma por RUN, arquivos com nome curto):
    sensors/Data/MATRIX_MAP/20260812_143012/samples.csv
    sensors/Data/MATRIX_MAP/20260812_143012/params.json
    sensors/Data/MATRIX_MAP/20260812_143012/adc.csv
    ...

O MODO de cada run sai do próprio params.json. Um grupo SEM params.json e
SEM samples.csv é gravação avulsa (botão "Record data" fora de um run) e vai
para RECORDING/.

Nada é sobrescrito: se o destino já existe, o arquivo é DEIXADO onde está e
reportado no fim. Sem `--apply` o script só mostra o que faria.

Uso:
  python3 src/touch_pack/scripts/migrate_runs_layout.py [dir_dados]
  python3 src/touch_pack/scripts/migrate_runs_layout.py [dir_dados] --apply
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import sys
import time

# Nomes novos, por sufixo antigo. Espelha os RUN_*_CSV de touch_pack.constants
# — este script roda sem ROS no PATH, então não os importa.
_SUFFIX_MAP = {
    '__samples.csv': 'samples.csv',
    '__sensors.csv': 'sensors.csv',
    '__matrix.csv': 'matrix.csv',
    '__params.json': 'params.json',
    '__summary.json': 'summary.json',
    '__plot.png': 'plot.png',
}
_PREFIX_MAP = {
    'adc_': 'adc.csv',
    'spikes_': 'spikes.csv',
    'cuneiformes_': 'cuneiformes.csv',
}
_MODES = ('SLIDE', 'TOUCH', 'MANUAL', 'MATRIX_MAP')
_REC_DIR = 'RECORDING'
_TS_RE = re.compile(r'^\d{8}_\d{6}$')


def _find_data_dir() -> str:
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        cand = os.path.join(d, 'sensors', 'Data')
        if os.path.isdir(cand):
            return cand
        d = os.path.dirname(d)
    return '.'


def _group(data_dir: str) -> dict[str, dict[str, str]]:
    """{run_id: {nome_novo: caminho_antigo}} dos arquivos SOLTOS na raiz."""
    runs: dict[str, dict[str, str]] = {}
    for name in sorted(os.listdir(data_dir)):
        path = os.path.join(data_dir, name)
        if not os.path.isfile(path):
            continue
        ts = new = None
        for suf, newname in _SUFFIX_MAP.items():
            if name.endswith(suf):
                ts, new = name[:-len(suf)], newname
                break
        else:
            for pre, newname in _PREFIX_MAP.items():
                if name.startswith(pre) and name.endswith('.csv'):
                    ts, new = name[len(pre):-len('.csv')], newname
                    break
        if ts is None or not _TS_RE.match(ts):
            continue
        runs.setdefault(ts, {})[new] = path
    return runs


def _mode_of(files: dict[str, str]) -> str:
    """Modo do run, lido do params.json. Sem ele: RECORDING quando não há
    sequer samples.csv (gravação avulsa), SLIDE no resto — que é o default
    do próprio logger para `mode` vazio."""
    params = files.get('params.json')
    if params:
        try:
            with open(params) as fh:
                mode = str(json.load(fh).get('mode', '') or '').upper().strip()
            if mode in _MODES:
                return mode
        except (OSError, json.JSONDecodeError, AttributeError):
            pass
        return 'SLIDE'
    return 'SLIDE' if 'samples.csv' in files else _REC_DIR


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('data_dir', nargs='?', default=None)
    ap.add_argument('--apply', action='store_true',
                    help='move os arquivos (sem isto, só mostra o plano)')
    ap.add_argument('--min-age-s', type=float, default=60.0,
                    help='idade mínima do arquivo mais recente do run para '
                         'ele ser movido (default 60 s)')
    args = ap.parse_args(argv)

    data_dir = args.data_dir or _find_data_dir()
    if not os.path.isdir(data_dir):
        sys.exit(f'diretório inexistente: {data_dir}')

    _now = time.time()
    runs = _group(data_dir)
    if not runs:
        print(f'Nada solto para migrar em {data_dir}.')
        return 0

    moved = skipped = 0
    for ts in sorted(runs):
        files = runs[ts]
        # Run ainda em curso: o logger e a GUI estão com os arquivos abertos.
        # Mover não corromperia os dados (o handle segue o inode), mas o
        # relatório pós-run sairia no caminho velho — recriando a bagunça.
        age = min(_now - os.path.getmtime(p) for p in files.values())
        if age < args.min_age_s:
            print(f'… {ts} escrito há {age:.0f}s — run possivelmente em '
                  f'curso, deixado para depois')
            skipped += len(files)
            continue
        mode = _mode_of(files)
        dest_dir = os.path.join(data_dir, mode, ts)
        print(f'{mode}/{ts}  ({len(files)} arquivos)')
        for new_name in sorted(files):
            src = files[new_name]
            dst = os.path.join(dest_dir, new_name)
            if os.path.exists(dst):
                print(f'    ! {new_name:<16} destino já existe — mantido em '
                      f'{os.path.basename(src)}')
                skipped += 1
                continue
            print(f'    {os.path.basename(src):<40} → {new_name}')
            if args.apply:
                os.makedirs(dest_dir, exist_ok=True)
                shutil.move(src, dst)
            moved += 1

    verb = 'movidos' if args.apply else 'a mover'
    print(f'\n{moved} arquivos {verb} em {len(runs)} runs.'
          + (f'  {skipped} pulados (destino ocupado).' if skipped else ''))
    if not args.apply:
        print('Nada foi movido — repita com --apply.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
