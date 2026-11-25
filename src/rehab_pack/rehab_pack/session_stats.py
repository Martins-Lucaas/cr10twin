"""Estatísticas de sessão (lógica PURA, sem ROS): resumo por alcance e totais.

Um alcance vai de TARGET_ON até o próximo TARGET_HIT/TARGET_MISS. As métricas
usam o TCP do robô REAL (xr_m..zr_m) quando gravado em todo o trecho, com a velocidade
derivada dele; senão o simulado (x_m..z_m) e a velocidade do setpoint (vx_ms..vz_ms).
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


def velocidade(t, pos, janela=7):
    """Velocidade por inclinação da reta em `janela` amostras (~0,2 s): o status do robô
    e o tick não são síncronos, e a diferença simples criaria picos falsos."""
    m, v = janela // 2, []
    for i in range(len(t)):
        a, b = max(0, i - m), min(len(t), i + m + 1)
        tm = sum(t[a:b]) / (b - a)
        den = sum((s - tm) ** 2 for s in t[a:b])
        v.append(tuple(sum((t[k] - tm) * pos[k][e] for k in range(a, b)) / den if den > 0 else 0.0
                       for e in range(3)))
    return v


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


def _tarefas(eventos, t_ini, t_fim):
    """[(início, fim)] de cada tarefa; a sessão inteira se não houver TASK_START."""
    out, ini = [], None
    for t, tipo, _ in eventos:
        if tipo == 'TASK_START':
            if ini is not None:
                out.append((ini, t))
            ini = t
        elif tipo == 'TASK_END' and (ini is not None or not out):
            # fim sem início: a tarefa começou antes da sessão
            out.append((t_ini if ini is None else ini, t))
            ini = None
    if ini is not None:
        out.append((ini, t_fim))
    return out or [(t_ini, t_fim)]


def _tempo_fora_tunel(eventos, t_ini, t_fim):
    """Soma dos intervalos TUNNEL_EXIT -> TUNNEL_ENTER (ou fim). None sem eventos de túnel."""
    if not any(e[1] in ('TUNNEL_EXIT', 'TUNNEL_ENTER') for e in eventos):
        return None
    fora = 0.0
    # 1º evento é ENTER: já estava fora quando o trecho começou
    saiu = t_ini if next(e[1] for e in eventos if e[1].startswith('TUNNEL_')) == 'TUNNEL_ENTER' else None
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

    real = (list(zip(amostras['xr_m'], amostras['yr_m'], amostras['zr_m']))
            if n and 'xr_m' in amostras else [])

    alcances = []
    aberto = None
    for te, tipo, detalhe in sorted(eventos, key=lambda e: e[0]):
        if tipo == 'TARGET_ON':
            aberto = (te, detalhe)
        elif tipo in ('TARGET_HIT', 'TARGET_MISS') and aberto is not None:
            t0, det0 = aberto
            aberto = None
            idx = [i for i in range(n) if t0 <= t[i] <= te]
            ts, p, v = [t[i] for i in idx], [pos[i] for i in idx], [vel[i] for i in idx]
            usa_real = bool(real and idx) and all(math.isfinite(c) for i in idx for c in real[i])
            if usa_real:
                p = [real[i] for i in idx]
                v = velocidade(ts, p)
            m = metricas_alcance(ts, p, v)
            alcances.append({'t_inicio_s': t0, 't_fim_s': te,
                             'resultado': 'acerto' if tipo == 'TARGET_HIT' else 'erro',
                             'alvo': det0, 'posicao': 'real' if usa_real else 'simulada',
                             **(m or {})})

    totais = {
        'duracao_s': (t[-1] - t[0]) if n > 1 else 0.0,
        'n_amostras': n,
        'f_media_n': None, 'f_pico_n': None,
        'pct_tempo_no_tunel': None,
        'acertos': sum(1 for e in eventos if e[1] == 'TARGET_HIT'
                       and 'fase=volta' not in e[2]),
        'erros': sum(1 for e in eventos if e[1] == 'TARGET_MISS'
                     and 'fase=volta' not in e[2]),
    }
    if n:
        fn = [math.sqrt(a * a + b * b + c * c) for a, b, c in
              zip(amostras['fx_n'], amostras['fy_n'], amostras['fz_n'])]
        fn = [f for f in fn if math.isfinite(f)]
        if fn:
            totais['f_media_n'] = sum(fn) / len(fn)
            totais['f_pico_n'] = max(fn)
        if n > 1:
            # só o tempo das tarefas de túnel conta: nem intervalos, nem outras tarefas
            fora = total = 0.0
            for ini, fim in _tarefas(eventos, t[0], t[-1]):
                # início aberto em TASK_END: o EXIT de uma tarefa não vaza para a seguinte
                f = _tempo_fora_tunel([e for e in eventos if ini <= e[0] <= fim], ini, fim)
                if f is None and any(
                        (e[1] == 'TASK_START' and ini == e[0] and 'tunel' in e[2])
                        or (e[1] == 'TASK_END' and fim == e[0]
                            and ('dentro_pct' in e[2] or 'progresso' in e[2])) for e in eventos):
                    f = 0.0           # túnel sem nenhuma saída
                if f is not None:
                    fora, total = fora + f, total + fim - ini
            if total > 0:
                totais['pct_tempo_no_tunel'] = 100.0 * (1.0 - fora / total)
    return {'alcances': alcances, 'totais': totais}


def pico_de_forca(wrench_csv):
    """Maior |F| finita das amostras cruas (samples.csv é a média do tick); None sem dados."""
    pico = None
    with open(wrench_csv, newline='', encoding='utf-8') as f:
        for lin in csv.DictReader(f):
            try:
                n = math.hypot(*(float(lin[k]) for k in ('fx_n', 'fy_n', 'fz_n')))
            except (TypeError, ValueError):     # última linha cortada
                continue
            if math.isfinite(n) and (pico is None or n > pico):
                pico = n
    return pico


def resumir_arquivos(samples_csv, events_csv, wrench_csv=None):
    r = resumir(ler_colunas(samples_csv), ler_eventos(events_csv))
    pico = pico_de_forca(wrench_csv) if wrench_csv else None
    if pico is not None:
        r['totais']['f_pico_n'] = pico
    return r
