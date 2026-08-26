"""
plane_probe.py — geometria da CALIBRAÇÃO DINÂMICA DO ÂNGULO DE ATAQUE.

Módulo PURO (só numpy): sem ROS, sem estado, sem I/O. Quem move o braço é o
`tactile_explorer`; aqui mora apenas a conta que transforma N pontos de
contato no eixo de ataque corrigido — e as guardas que dizem quando essa
conta NÃO pode ser confiada.

A separação existe para que a parte perigosa (girar o punho com a ponteira
perto da peça) dependa de uma função que roda em teste unitário, sem robô.

Convenções (todas no MUNDO URDF, metros):
  • `normal`  — unitário, orientado para FORA da superfície, isto é, para o
                lado de onde a ferramenta chega (componente +Z positiva).
  • `ataque`  — unitário, o eixo Z do TCP: aponta PARA DENTRO da superfície.
                É exatamente `-normal`, e é o que `inverse_kinematics` recebe
                como `approach_vec`.
  • `desvio`  — ângulo entre a normal medida e a vertical do mundo (+Z), que
                é a normal ESPERADA quando o alvo está perpendicular à home.

API pública:
    probe_pattern(n_points, radius_m)   → offsets XY (n, 2)
    probe_ring_from_grid(nodes_xy, …)   → offsets XY (4, 2)
    fit_plane(points, ref_normal=+Z)    → PlaneFit
    attack_dir_from_normal(normal)      → (3,) unitário
    angle_between_deg(a, b)             → float
    validate_fit(fit, ...)              → (ok, motivo)
    slope_along_deg(normal, dir_xy)     → float
"""
from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

# Mínimo GEOMÉTRICO: três pontos não colineares definem um plano. Abaixo
# disso não há plano nenhum, então não é opção de configuração.
MIN_PROBE_POINTS = 3
# Recomendado: com 4+ pontos o ajuste passa a ser de MÍNIMOS QUADRADOS e o
# resíduo vira um número que se pode olhar — com exatamente 3 ele é sempre
# zero e não diz nada sobre a qualidade da medição.
DEFAULT_PROBE_POINTS = 4

# Vertical do mundo URDF — a normal esperada de um alvo perpendicular à home.
Z_UP = np.array([0.0, 0.0, 1.0])

# ── Guardas padrão do ajuste ──────────────────────────────────────────
# Desvio máximo admitido. Acima disso a peça está tão inclinada que o
# problema é de montagem/calço, não de software: o punho teria de girar
# muito com a ponteira perto da superfície, e o alcance do J5 vira o
# limitante antes da geometria.
TILT_MAX_DEG_DEFAULT = 20.0
# Resíduo RMS máximo do ajuste. Um plano que não fecha em meio milímetro não
# é um plano: ou a peça é curva, ou um dos toques pegou uma borda/sujeira.
# Atacar segundo essa "normal" é pior que atacar na vertical.
RMS_MAX_M_DEFAULT = 5.0e-4
# Espalhamento mínimo (σ₂/σ₁ dos pontos centrados): mede quão LONGE de
# colineares os pontos estão. Três pontos quase em linha definem um plano
# matematicamente, mas a normal gira descontroladamente com poucos µm de
# ruído — é o caso degenerado que precisa ser recusado, não ajustado.
SPREAD_MIN_DEFAULT = 0.10

# Fração do passo que o anel de sondagem fica FORA do último nó da grade
# (ver probe_ring_from_grid). Meio passo é a menor folga que garante que
# nenhum toque de sonda cai sobre um ponto que o ensaio vai identar, e a
# maior que mantém a sondagem na vizinhança imediata do que será medido.
GRID_RING_STEP_FRAC = 0.5


class PlaneFit(NamedTuple):
    """Resultado do ajuste do plano de contato.

    normal    (3,) unitário, orientado para fora da superfície
    centroid  (3,) centroide dos pontos de contato (m, mundo URDF)
    rms_m     resíduo ortogonal RMS (m) — 0 quando N == 3
    tilt_deg  desvio angular entre `normal` e a referência (graus)
    spread    σ₂/σ₁ dos pontos centrados ∈ [0, 1]; 0 = colineares
    n_points  quantidade de pontos usados no ajuste
    """
    normal: np.ndarray
    centroid: np.ndarray
    rms_m: float
    tilt_deg: float
    spread: float
    n_points: int

    @property
    def least_squares(self) -> bool:
        """True quando o ajuste teve graus de liberdade sobrando (N > 3) —
        é o caso em que `rms_m` mede alguma coisa."""
        return self.n_points > MIN_PROBE_POINTS


def _unit(v) -> np.ndarray:
    """Versor de `v`. Levanta ValueError em vetor nulo — silenciar isso
    devolveria uma direção arbitrária para o robô seguir."""
    a = np.asarray(v, dtype=float).flatten()
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        raise ValueError('vetor nulo não tem direção')
    return a / n


def angle_between_deg(a, b) -> float:
    """Ângulo entre dois vetores, em graus, no intervalo [0, 180]."""
    ua, ub = _unit(a), _unit(b)
    return math.degrees(math.acos(float(np.clip(ua @ ub, -1.0, 1.0))))


def probe_pattern(n_points: int, radius_m: float) -> np.ndarray:
    """Offsets XY (n, 2) dos toques de sonda: polígono regular de raio
    `radius_m` centrado no ponto de aproximação inicial.

    O polígono regular é a distribuição mais bem condicionada para o mesmo
    número de toques — σ₁ ≈ σ₂ nos pontos centrados, ou seja, a normal fica
    igualmente determinada nas duas direções do plano. Uma linha de pontos,
    pelo mesmo custo em tempo de robô, deixaria um eixo indeterminado.

    O centro NÃO entra no padrão: ele não acrescenta nada à normal (está no
    centroide) e custaria uma identação a mais na amostra.
    """
    n = max(MIN_PROBE_POINTS, int(n_points))
    r = float(radius_m)
    if not math.isfinite(r) or r <= 0.0:
        raise ValueError(f'raio de sondagem inválido: {radius_m!r}')
    ang = 2.0 * math.pi * np.arange(n) / n
    return np.column_stack([r * np.cos(ang), r * np.sin(ang)])


def _axis_geometry(v: np.ndarray) -> tuple[float, float, float]:
    """(mínimo, máximo, passo) de um eixo da grade.

    O passo é reconstruído do vão e da contagem de coordenadas DISTINTAS —
    numa grade regular isso devolve exatamente o passo que a gerou, sem
    precisar recebê-lo por fora. A tolerância de 1 µm no `unique` é o que
    impede que ruído de ponto flutuante conte a mesma coluna duas vezes e
    devolva um passo menor que o real.
    """
    lo, hi = float(np.min(v)), float(np.max(v))
    n_uniq = len(np.unique(np.round(v, 6)))
    step = (hi - lo) / (n_uniq - 1) if n_uniq > 1 else 0.0
    return lo, hi, step


def probe_ring_from_grid(nodes_xy, *, min_half_extent_m: float,
                         step_frac: float = GRID_RING_STEP_FRAC
                         ) -> np.ndarray:
    """Offsets XY (4, 2) dos toques de sonda derivados da PRÓPRIA grade de
    identações: os quatro cantos do retângulo que a contém, afastados
    `step_frac` de um passo para fora em cada eixo.

    Por que não o polígono de `probe_pattern`: o raio dele é um número
    solto ao lado da grade. Com passo de 5 mm e 4×4 o ensaio percorre 15 mm,
    mas o raio default manda a sonda tocar a 15 mm do centro — possivelmente
    FORA da amostra, medindo o plano de uma região que o ensaio nunca visita.
    O anel derivado da grade mede o plano de exatamente onde se vai medir.

    Por que meio passo para fora e não os cantos em si: sondar um nó o
    pré-condiciona (toque de sonda + identação no mesmo ponto), e em tecido
    viscoelástico esse ponto deixa de ser comparável aos demais.

    Os quatro cantos são também o conjunto MELHOR CONDICIONADO disponível
    dentro da grade — máximo braço de alavanca nas duas direções do plano.

    Args:
        nodes_xy:          (N, 2) nós da grade, INCLUINDO a origem (0, 0),
                           na mesma unidade de `min_half_extent_m`.
        min_half_extent_m: braço de alavanca mínimo aceitável, medido do
                           centro do anel até o lado mais próximo. Abaixo
                           dele a incerteza angular do ajuste (≈ σ_z/braço)
                           fica pior que a do polígono, e quem chama deve
                           cair de volta nele.

    Raises:
        ValueError: grade vazia, com coordenadas não finitas, ou curta
                    demais num dos eixos para servir de padrão de sondagem.
    """
    P = np.asarray(nodes_xy, dtype=float).reshape(-1, 2)
    if len(P) == 0:
        raise ValueError('grade sem nós')
    if not np.all(np.isfinite(P)):
        raise ValueError('nós da grade não finitos')

    x0, x1, step_x = _axis_geometry(P[:, 0])
    y0, y1, step_y = _axis_geometry(P[:, 1])
    mx = float(step_frac) * step_x
    my = float(step_frac) * step_y

    half_x = (x1 - x0) / 2.0 + mx
    half_y = (y1 - y0) / 2.0 + my
    lim = float(min_half_extent_m)
    if min(half_x, half_y) < lim:
        raise ValueError(
            f'anel de {2 * half_x:.4f} × {2 * half_y:.4f} tem lado menor '
            f'que o braço mínimo de {2 * lim:.4f}')

    # Volta fechada a partir do canto mais próximo da origem da grade — o
    # mesmo critério de trânsito curto da serpentina.
    return np.array([
        [x0 - mx, y0 - my],
        [x1 + mx, y0 - my],
        [x1 + mx, y1 + my],
        [x0 - mx, y1 + my],
    ])


def fit_plane(points, ref_normal=Z_UP) -> PlaneFit:
    """Ajusta o plano aos pontos de contato e devolve a normal medida.

    O ajuste é ORTOGONAL (distância perpendicular ao plano), obtido pela SVD
    dos pontos centrados: a normal é o vetor singular à direita de menor
    valor singular. NÃO é a regressão de z sobre (x, y) — aquela minimiza o
    erro vertical, e num plano inclinado isso enviesa a normal justamente na
    direção da inclinação, que é o que se quer medir.

    Com exatamente 3 pontos o resultado é o plano exato (resíduo zero); com
    4 ou mais é o ajuste de mínimos quadrados que filtra o ruído mecânico de
    cada toque. É o MESMO caminho de código — não há dois algoritmos para
    manter em sincronia.

    Args:
        points:     (N, 3) coordenadas dos contatos, N >= 3, mundo URDF (m)
        ref_normal: normal ESPERADA; orienta o sinal do resultado e é a
                    referência do desvio angular. Default: vertical do mundo.

    Raises:
        ValueError: menos de 3 pontos, coordenadas não finitas, ou nuvem
                    degenerada a ponto de não definir normal alguma.
    """
    P = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(P) < MIN_PROBE_POINTS:
        raise ValueError(
            f'{len(P)} ponto(s) de contato — o ajuste do plano precisa de '
            f'pelo menos {MIN_PROBE_POINTS}')
    if not np.all(np.isfinite(P)):
        raise ValueError('coordenadas de contato não finitas')

    centroid = P.mean(axis=0)
    Q = P - centroid
    # full_matrices=False basta: com 3 colunas, vt sempre sai (3, 3) para
    # N >= 3, então vt[2] existe.
    _u, s, vt = np.linalg.svd(Q, full_matrices=False)

    if float(s[0]) < 1e-9:
        raise ValueError('todos os pontos de contato coincidem')
    spread = float(s[1] / s[0])

    normal = np.asarray(vt[2], dtype=float)
    nn = float(np.linalg.norm(normal))
    if nn < 1e-12:
        raise ValueError('nuvem de contatos degenerada — normal indefinida')
    normal /= nn

    ref = _unit(ref_normal)
    # Sinal: a SVD não escolhe lado. A normal tem de apontar para o lado de
    # onde a ferramenta chega, senão o ataque sairia invertido.
    if float(normal @ ref) < 0.0:
        normal = -normal

    resid = Q @ normal
    rms_m = float(np.sqrt(float(np.mean(resid * resid))))

    return PlaneFit(
        normal=normal,
        centroid=centroid,
        rms_m=rms_m,
        tilt_deg=angle_between_deg(normal, ref),
        spread=spread,
        n_points=int(len(P)),
    )


def attack_dir_from_normal(normal) -> np.ndarray:
    """Eixo de ataque (Z do TCP) a partir da normal da superfície.

    Atacar 100 % alinhado é encostar ao longo de −normal: a face da ponteira
    fica paralela à superfície e a força medida pela célula é a força NORMAL,
    não a projeção dela num contato de canto.
    """
    return -_unit(normal)


def slope_along_deg(normal, dir_xy) -> float:
    """Inclinação do plano medido ao longo de uma direção lateral (graus).

    Positivo = a superfície SOBE no sentido da marcha, que é a convenção do
    campo `slide_slope_deg` do SLIDING. Serve para alimentar o deslize com a
    rampa MEDIDA em vez da declarada à mão.

    Raises:
        ValueError: normal perpendicular à vertical (plano "de pé": não há
                    função z(x, y)) ou direção lateral nula.
    """
    n = _unit(normal)
    u = np.asarray(dir_xy, dtype=float).flatten()[:2]
    un = float(np.linalg.norm(u))
    if un < 1e-12:
        raise ValueError('direção de deslize nula')
    u = u / un
    if abs(float(n[2])) < 1e-6:
        raise ValueError('plano vertical — inclinação lateral indefinida')
    # Plano: n·(p − c) = 0  ⇒  dz/ds ao longo de u = −(nx·ux + ny·uy)/nz.
    dz_per_m = -(float(n[0]) * float(u[0]) + float(n[1]) * float(u[1])) / float(n[2])
    return math.degrees(math.atan(dz_per_m))


def validate_fit(fit: PlaneFit, *,
                 tilt_max_deg: float = TILT_MAX_DEG_DEFAULT,
                 rms_max_m: float = RMS_MAX_M_DEFAULT,
                 spread_min: float = SPREAD_MIN_DEFAULT) -> tuple[bool, str]:
    """Diz se o plano ajustado pode ser usado para reorientar a ferramenta.

    Devolve (ok, motivo). `motivo` é vazio quando ok, e é a frase que vai
    para o log de erro quando não — cada recusa aponta a ação corretiva,
    porque todas elas são de MONTAGEM, não de software.

    A ordem das guardas é a de gravidade: um desvio acima do limite é risco
    de junta/cisalhamento; um resíduo alto ou pontos colineares são medição
    que não sustenta conclusão nenhuma.
    """
    if not math.isfinite(fit.tilt_deg) or fit.tilt_deg > float(tilt_max_deg):
        return False, (
            f'desvio de {fit.tilt_deg:.2f}° acima do limite de segurança de '
            f'{float(tilt_max_deg):.1f}° — recalce a peça ou reveja a home '
            f'antes de insistir')
    if fit.spread < float(spread_min):
        return False, (
            f'pontos de contato quase colineares (espalhamento '
            f'{fit.spread:.3f} < {float(spread_min):.2f}) — a normal fica '
            f'indeterminada; aumente o raio de sondagem')
    if fit.rms_m > float(rms_max_m):
        return False, (
            f'resíduo RMS de {fit.rms_m * 1e3:.3f} mm acima de '
            f'{float(rms_max_m) * 1e3:.3f} mm — a superfície sondada não é '
            f'plana (curvatura, borda ou sujeira num dos toques)')
    return True, ''
