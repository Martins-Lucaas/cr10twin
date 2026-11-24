"""Posicionamento por juntas: validação e duração (sem ROS).

O nó de admitância chama `plan_jog` antes de aceitar um alvo. Recusa o que sai
dos limites de software ou faz o TCP/célula descer abaixo do piso ao longo do
caminho (interpolação linear em juntas, a mesma que o JTC percorre).

NÃO checa colisão do cotovelo e dos elos com a mesa, a cadeira ou o voluntário:
só o TCP e a célula. Posicione em etapas e com a mão no E-Stop.
"""
from __future__ import annotations

import math

import numpy as np

from .constants import (JOG_MAX_DEG_S, JOG_MIN_DELTA_DEG, JOG_MIN_S,
                        JOINT_LIMITS_DEG, MIN_CELL_Z, MIN_TCP_Z)

SAMPLES = 25


def max_delta_deg(q_now, q_goal) -> tuple[float, int]:
    """(maior deslocamento em graus, junta 1..6)."""
    d = np.degrees(np.abs(np.asarray(q_goal, float) - np.asarray(q_now, float)))
    k = int(np.argmax(d))
    return float(d[k]), k + 1


def duration_s(q_now, q_goal) -> float:
    return max(JOG_MIN_S, max_delta_deg(q_now, q_goal)[0] / JOG_MAX_DEG_S)


def plan_jog(q_now, q_goal, fk_z, limits_deg=JOINT_LIMITS_DEG,
             min_tcp_z: float = MIN_TCP_Z, min_cell_z: float = MIN_CELL_Z
             ) -> tuple[bool, str, float]:
    """(ok, motivo, duração em s). `fk_z(q)` devolve (z do TCP, z da célula)."""
    q0 = np.asarray(q_now, float).reshape(-1)
    q1 = np.asarray(q_goal, float).reshape(-1)
    if q1.shape != (6,) or q0.shape != (6,):
        return False, 'o alvo precisa ter 6 juntas', 0.0
    if not (np.all(np.isfinite(q1)) and np.all(np.isfinite(q0))):
        return False, 'alvo com valor não finito', 0.0
    for i, (lo, hi) in enumerate(limits_deg):
        g = math.degrees(q1[i])
        if not lo <= g <= hi:
            return False, (f'J{i + 1} = {g:.1f}° fora do limite '
                           f'({lo:.0f}° a {hi:.0f}°)'), 0.0
    delta, j = max_delta_deg(q0, q1)
    if delta < JOG_MIN_DELTA_DEG:
        return False, 'já está nessa posição (< 0,5°)', 0.0
    for s in np.linspace(0.0, 1.0, SAMPLES):
        z_tcp, z_cell = fk_z((1.0 - s) * q0 + s * q1)
        if z_tcp < min_tcp_z:
            return False, (f'no caminho o TCP desceria a {z_tcp:.2f} m '
                           f'(mínimo {min_tcp_z:.2f} m): mova em etapas'), 0.0
        if z_cell < min_cell_z:
            return False, (f'no caminho a célula desceria a {z_cell:.2f} m '
                           f'(mínimo {min_cell_z:.2f} m): mova em etapas'), 0.0
    return True, f'J{j} é o maior movimento: {delta:.1f}°', duration_s(q0, q1)
