"""Posicionamento por juntas: validação e duração (sem ROS).

O nó de admitância chama `plan_jog` antes de aceitar um alvo. Recusa o que sai
dos limites de software, faz o TCP/célula descer abaixo do piso ou leva cotovelo/punho à mesa
ao longo do caminho (reta em juntas, a mesma que o JTC percorre). Quem já está
numa zona invadida só pode sair dela: assim um braço baixo ainda pode ser erguido.

O JTC interpola ponto só com posição em rampa LINEAR (degrau de velocidade) e ponto
com velocidade por spline CÚBICA: por isso o alvo vai com velocidade final zero. De
parado a cúbica é reta em juntas e seu pico é 1,5× a média.

NÃO checa a cadeira nem pessoas (voluntário, operador). Posicione em etapas e com a
mão no E-Stop.
"""
from __future__ import annotations

import math

import numpy as np

from .constants import (JOG_MAX_DEG_S, JOG_SPEED_DEFAULT_PCT, JOG_MIN_DELTA_DEG, JOG_MIN_S,
                        JOINT_LIMITS_DEG, MIN_CELL_Z, MIN_TCP_Z)

SAMPLES = 25
SPLINE_PEAK = 1.5         # pico/média da cúbica parado→parado
BRAKE_S = 0.3             # freada ao trocar o alvo no meio do movimento


def max_delta_deg(q_now, q_goal) -> tuple[float, int]:
    """(maior deslocamento em graus, junta 1..6)."""
    d = np.degrees(np.abs(np.asarray(q_goal, float) - np.asarray(q_now, float)))
    k = int(np.argmax(d))
    return float(d[k]), k + 1


def duration_s(q_now, q_goal, speed_pct: float = JOG_SPEED_DEFAULT_PCT) -> float:
    return max(JOG_MIN_S, SPLINE_PEAK * max_delta_deg(q_now, q_goal)[0]
               / (JOG_MAX_DEG_S * speed_pct / 100.0))


def brake_point(q_now, dq_now):
    """Onde o braço para em BRAKE_S com desaceleração constante: a cúbica até aqui
    (velocidade dq_now → 0) é reta em juntas e nunca passa de dq_now."""
    return np.asarray(q_now, float) + np.asarray(dq_now, float) * BRAKE_S / 2.0


def plan_jog(q_now, q_goal, fk_z, limits_deg=JOINT_LIMITS_DEG,
             min_tcp_z: float = MIN_TCP_Z, min_cell_z: float = MIN_CELL_Z,
             speed_pct: float = JOG_SPEED_DEFAULT_PCT, clearance=None,
             q_brake=None) -> tuple[bool, str, float]:
    """(ok, motivo, duração em s). `fk_z(q)` devolve (z do TCP, z da célula);
    `clearance(q)`, a folga dos elos à mesa (< 0 = invadiu). Com `q_brake` (braço em
    movimento) o caminho é q_now → q_brake → q_goal e a duração soma a freada."""
    q0 = np.asarray(q_now, float).reshape(-1)
    q1 = np.asarray(q_goal, float).reshape(-1)
    if q1.shape != (6,) or q0.shape != (6,):
        return False, 'o alvo precisa ter 6 juntas', 0.0
    if not (np.all(np.isfinite(q1)) and np.all(np.isfinite(q0))):
        return False, 'alvo com valor não finito', 0.0
    qb = q0 if q_brake is None else np.asarray(q_brake, float)
    if not np.all(np.isfinite(qb)):
        return False, 'velocidade medida não finita', 0.0
    for q in ((q1,) if q_brake is None else (q1, qb)):
        for i, (lo, hi) in enumerate(limits_deg):
            g = math.degrees(q[i])
            if not lo <= g <= hi:
                return False, (f'J{i + 1} = {g:.1f}° fora do limite '
                               f'({lo:.0f}° a {hi:.0f}°)'), 0.0
    delta, j = max_delta_deg(qb, q1)
    if q_brake is None and delta < JOG_MIN_DELTA_DEG:
        return False, 'já está nessa posição (< 0,5°)', 0.0
    path = [(1.0 - s) * qb + s * q1 for s in np.linspace(0.0, 1.0, SAMPLES)]
    if q_brake is not None:
        path = [q0, (q0 + qb) / 2.0] + path
    prev = None                    # alturas e folga da amostra anterior
    for q in path:
        z_tcp, z_cell = fk_z(q)
        folga = clearance(q) if clearance else math.inf
        ant, prev = prev, (z_tcp, z_cell, folga)
        if ant is None:            # onde o braço já está: só vale se ele sair daqui
            continue
        if z_tcp < min_tcp_z and z_tcp <= ant[0]:
            return False, (f'no caminho o TCP desceria a {z_tcp:.2f} m '
                           f'(mínimo {min_tcp_z:.2f} m): mova em etapas'), 0.0
        if z_cell < min_cell_z and z_cell <= ant[1]:
            return False, (f'no caminho a célula desceria a {z_cell:.2f} m '
                           f'(mínimo {min_cell_z:.2f} m): mova em etapas'), 0.0
        # como na admitância: mover-se dentro da zona sem piorar é aceito
        if folga < 0.0 and folga < ant[2] - 1e-6:
            return False, (f'no caminho o cotovelo ou o punho chegaria à mesa '
                           f'(folga {folga * 100:.0f} cm): mova em etapas'), 0.0
    dur = duration_s(qb, q1, speed_pct) + (BRAKE_S if q_brake is not None else 0.0)
    return True, f'J{j} é o maior movimento: {delta:.1f}°', dur
