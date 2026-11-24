"""Estatísticas de sessão (lógica PURA, sem ROS): resumo por alcance e totais.

Um alcance vai de TARGET_ON até o próximo TARGET_HIT/TARGET_MISS. As métricas
usam a posição medida (x_m..z_m) e a velocidade do setpoint (vx_ms..vz_ms).
"""
from __future__ import annotations

import csv
import math

LIMIAR_MOVIMENTO = 0.05   # fração do pico de velocidade que conta como "em movimento"
PROMINENCIA_PICO = 0.10   # queda mínima (fração do pico) para separar dois picos


def contar_picos(v, prominencia):
    """Nº de máximos locais de v separados por vales de ao menos `prominencia`."""
    if len(v) < 2 or prominencia <= 0:
        return 0
    n = 0
    subindo = True
    vmax = vmin = v[0]
    for x in v[1:]:
        if subindo:
            if x > vmax:
                vmax = x
            elif vmax - x >= prominencia:
                if vmax >= prominencia:
                    n += 1
                subindo = False
                vmin = x
        else:
            if x < vmin:
                vmin = x
            elif x - vmin >= prominencia:
                subindo = True
                vmax = x
    if subindo and vmax - vmin >= prominencia and vmax >= prominencia:
        n += 1
    return n


def metricas_alcance(t, pos, vel):
    """t: lista de s; pos/vel: listas de (x, y, z). Devolve dict ou None (<2 amostras)."""
    if len(t) < 2:
        return None
    caminho = sum(math.dist(pos[i], pos[i + 1]) for i in range(len(pos) - 1))
    corda = math.dist(pos[0], pos[-1])
    vel_n = [math.sqrt(sum(c * c for c in v)) for v in vel]
    pico = max(vel_n)
    mov = [i for i, s in enumerate(vel_n) if s > LIMIAR_MOVIMENTO * pico] if pico > 0 else []
    return {
        'tempo_alcance_s': t[-1] - t[0],
        'tempo_movimento_s': (t[mov[-1]] - t[mov[0]]) if mov else 0.0,
        'comprimento_caminho_m': caminho,
        'retilinearidade': (corda / caminho) if caminho > 1e-9 else None,
        'pico_velocidade_ms': pico,
        'n_picos_velocidade': contar_picos(vel_n, PROMINENCIA_PICO * pico),
    }


def ler_colunas(caminho):
    """CSV -> dict nome -> lista de float (campos não numéricos ficam como str)."""
    cols = {}
    with open(caminho, newline='', encoding='utf-8') as f:
        rd = csv.reader(f)
        nomes = next(rd, [])
        cols = {n: [] for n in nomes}
        for lin in rd:
            for n, v in zip(nomes, lin):
                try:
                    cols[n].append(float(v))
                except ValueError:
                    cols[n].append(v)
    return cols


def ler_eventos(caminho):
    """events.csv -> lista de (t_s, tipo, detalhe)."""
    with open(caminho, newline='', encoding='utf-8') as f:
        rd = csv.reader(f)
        next(rd, None)
        return [(float(l[0]), l[1], l[2] if len(l) > 2 else '') for l in rd if l]


def _tempo_fora_tunel(eventos, t_ini, t_fim):
    """Soma dos intervalos TUNNEL_EXIT -> TUNNEL_ENTER (ou fim). None sem eventos de túnel."""
    if not any(e[1] in ('TUNNEL_EXIT', 'TUNNEL_ENTER') for e in eventos):
        return None
    fora = 0.0
    saiu = None
    for t, tipo, _ in eventos:
        if tipo == 'TUNNEL_EXIT' and saiu is None:
            saiu = t
        elif tipo == 'TUNNEL_ENTER' and saiu is not None:
            fora += t - saiu
            saiu = None
    if saiu is not None:
        fora += max(0.0, t_fim - saiu)
    return min(fora, t_fim - t_ini)


def resumir(amostras, eventos):
    """amostras: dict de colunas (SAMPLES_COLUMNS); eventos: [(t_s, tipo, detalhe)]."""
    t = amostras.get('t_s', [])
    n = len(t)
    pos = list(zip(amostras['x_m'], amostras['y_m'], amostras['z_m'])) if n else []
    vel = list(zip(amostras['vx_ms'], amostras['vy_ms'], amostras['vz_ms'])) if n else []

    alcances = []
    aberto = None
    for te, tipo, detalhe in sorted(eventos, key=lambda e: e[0]):
        if tipo == 'TARGET_ON':
            aberto = (te, detalhe)
        elif tipo in ('TARGET_HIT', 'TARGET_MISS') and aberto is not None:
            t0, det0 = aberto
            aberto = None
            idx = [i for i in range(n) if t0 <= t[i] <= te]
            m = metricas_alcance([t[i] for i in idx], [pos[i] for i in idx],
                                 [vel[i] for i in idx])
            alcances.append({'t_inicio_s': t0, 't_fim_s': te,
                             'resultado': 'acerto' if tipo == 'TARGET_HIT' else 'erro',
                             'alvo': det0, **(m or {})})

    totais = {
        'duracao_s': (t[-1] - t[0]) if n > 1 else 0.0,
        'n_amostras': n,
        'f_media_n': None, 'f_pico_n': None,
        'pct_tempo_no_tunel': None,
        'acertos': sum(1 for e in eventos if e[1] == 'TARGET_HIT'),
        'erros': sum(1 for e in eventos if e[1] == 'TARGET_MISS'),
    }
    if n:
        fn = [math.sqrt(a * a + b * b + c * c) for a, b, c in
              zip(amostras['fx_n'], amostras['fy_n'], amostras['fz_n'])]
        totais['f_media_n'] = sum(fn) / n
        totais['f_pico_n'] = max(fn)
        if n > 1:
            fora = _tempo_fora_tunel(eventos, t[0], t[-1])
            if fora is not None and t[-1] > t[0]:
                totais['pct_tempo_no_tunel'] = 100.0 * (1.0 - fora / (t[-1] - t[0]))
    return {'alcances': alcances, 'totais': totais}


def resumir_arquivos(samples_csv, events_csv):
    return resumir(ler_colunas(samples_csv), ler_eventos(events_csv))
