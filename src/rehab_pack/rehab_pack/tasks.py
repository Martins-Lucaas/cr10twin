"""Lógica pura das tarefas de reabilitação (sem Tk, sem ROS).

Frame `world`, SI, plano XY do TCP. A GUI alimenta `update(t, x, y)` a cada
tick da admitância (t = t_s do tick, time.monotonic()) e publica os
eventos devolvidos em /rehab/event.

Convenção de tela (vista de cima, como o voluntário enxerga: ele fica em
x = 1,2 m olhando para −x): cima = −x do mundo, direita = +y.
"""
from __future__ import annotations

import math
import random
import re

from .admittance import arm_stiffness_limit
from .constants import (ARM_STIFFNESS_MIN_N_M, EVENT_TYPES,
                        FORCE_ABORT_LIMIT_N)

TASK_NAMES = ('livre', 'centro_fora', 'tunel_reta', 'tunel_circulo',
              'tunel_oito', 'alcance')

# Presets de admitância (campos = parâmetros do nó rehab_admittance).
PRESETS = {
    'Livre': dict(mass_kg=2.0, damping_ns_m=40.0, stiffness_n_m=0.0,
                  deadband_n=0.5, v_max_ms=0.10, max_excursion_m=0.15),
    'Leve': dict(mass_kg=1.0, damping_ns_m=20.0, stiffness_n_m=0.0,
                 deadband_n=0.3, v_max_ms=0.10, max_excursion_m=0.15),
    'Assistido': dict(mass_kg=2.0, damping_ns_m=40.0, stiffness_n_m=50.0,
                      deadband_n=0.5, v_max_ms=0.10, max_excursion_m=0.15),
    'Resistivo': dict(mass_kg=3.0, damping_ns_m=120.0, stiffness_n_m=0.0,
                      deadband_n=0.5, v_max_ms=0.10, max_excursion_m=0.15),
}


# ── Utilidades ──────────────────────────────────────────────────────────────
def validate_subject_id(s: str) -> bool:
    """ID anônimo: letras, números, '_' ou '-', 1 a 16 caracteres."""
    return re.fullmatch(r'[A-Za-z0-9_-]{1,16}', s or '') is not None


def format_event(tipo: str, **kv) -> str:
    """'TIPO;k=v;k=v' (formato de /rehab/event). ';' e '=' nos valores viram ' '."""
    if tipo not in EVENT_TYPES:
        raise ValueError(f'tipo de evento desconhecido: {tipo}')
    parts = [tipo]
    for k, v in kv.items():
        if isinstance(v, float):
            v = f'{v:.4g}'
        v = str(v).replace(';', ' ').replace('=', ' ')
        parts.append(f'{k}={v}')
    return ';'.join(parts)


def stability(mass_kg: float, damping_ns_m: float) -> dict:
    """Rigidez máxima do braço com a malha estável, para o atraso estimado."""
    k = arm_stiffness_limit(mass_kg, damping_ns_m)
    return {'k_max': k, 'ok': k >= ARM_STIFFNESS_MIN_N_M}


def preset_refusal(nome: str, params, ativo: bool) -> str:
    """Motivo para não aplicar o preset ('' = pode): com o braço ativo a mola não muda."""
    if ativo and (params or {}).get('stiffness_n_m') != PRESETS[nome]['stiffness_n_m']:
        return f'Perfil {nome} muda a mola (K): desabilite o braço antes.'
    return ''


def reach_limit(name: str, max_exc_m: float, center, ref) -> float:
    """Maior alcance/tamanho [m] da tarefa que cabe na excursão em torno de `ref`."""
    livre = max_exc_m - math.dist(center, ref)
    return 2.0 * livre if name == 'tunel_oito' else livre   # o oito se afasta só meio tamanho


def abort_limit_ok(limit_n: float) -> bool:
    return 0.0 < limit_n <= FORCE_ABORT_LIMIT_N


def decimate_minmax(ts, vs, cols: int):
    """Envelope min/max por coluna de pixel: [(t, v), …] com no máx. 2·cols
    pontos, em ordem temporal. Séries curtas saem intactas (picos preservados)."""
    n = len(vs)
    if n <= 2 * cols:
        return list(zip(ts, vs))
    out = []
    for j in range(cols):
        a, b = j * n // cols, max((j + 1) * n // cols, j * n // cols + 1)
        seg = vs[a:b]
        lo = a + seg.index(min(seg))
        hi = a + seg.index(max(seg))
        for i in sorted({lo, hi}):
            out.append((ts[i], vs[i]))
    return out


def shuffled_order(n: int, seed: int):
    """Ordem determinística dos alvos para um `seed`."""
    idx = list(range(n))
    random.Random(seed).shuffle(idx)
    return idx


# ── Área explorada ──────────────────────────────────────────────────────────
class ExploredArea:
    """Grade de ocupação centrada em `center`; `cell_m` de lado."""

    def __init__(self, center, half_m=0.25, cell_m=0.01):
        self.cx, self.cy = center
        self.half, self.cell = half_m, cell_m
        self.cells = set()

    def add(self, x: float, y: float) -> bool:
        """Marca a célula de (x, y). True se for nova."""
        if abs(x - self.cx) > self.half or abs(y - self.cy) > self.half:
            return False
        c = (int(math.floor((x - self.cx) / self.cell)),
             int(math.floor((y - self.cy) / self.cell)))
        if c in self.cells:
            return False
        self.cells.add(c)
        return True

    @property
    def area_m2(self) -> float:
        return len(self.cells) * self.cell ** 2


# ── Centro-fora ─────────────────────────────────────────────────────────────
class CenterOutTask:
    """8 alvos na periferia, cada ida seguida de volta ao centro.

    Acerto só após `hold_s` contínuos dentro do raio; sair zera a contagem.
    Passou `timeout_s` sem acertar -> TARGET_MISS e segue.
    Eventos: (tipo, dict) com tipos de EVENT_TYPES.
    """

    def __init__(self, center, reach_m=0.10, target_radius_m=0.02,
                 hold_s=0.5, timeout_s=10.0, n_targets=8, seed=0):
        self.cx, self.cy = center
        self.reach_m, self.radius = reach_m, target_radius_m
        self.hold_s, self.timeout_s = hold_s, timeout_s
        self.n_targets, self.seed = n_targets, seed
        self.order = shuffled_order(n_targets, seed)
        # Metas: (fase, índice do alvo, (x, y)); alvo k fica no ângulo k·360/n.
        self.goals = []
        for k in self.order:
            ang = 2.0 * math.pi * k / n_targets
            # ângulo 0 = direita da tela (+y); 90° = cima da tela (−x)
            pos = (self.cx - reach_m * math.sin(ang),
                   self.cy + reach_m * math.cos(ang))
            self.goals.append(('ida', k, pos))
            self.goals.append(('volta', k, (self.cx, self.cy)))
        self.target_positions = [
            (self.cx - reach_m * math.sin(2 * math.pi * k / n_targets),
             self.cy + reach_m * math.cos(2 * math.pi * k / n_targets))
            for k in range(n_targets)]
        self.i = -1
        self.started = False
        self.done = False
        self.hits = self.misses = 0           # só alvos periféricos (ida)
        self._t0 = self._t_goal = 0.0
        self._enter = None

    @property
    def goal(self):
        return None if self.done or self.i < 0 else self.goals[self.i]

    def current_target(self):
        g = self.goal
        return None if g is None else g[2]

    def start(self, t):
        self.started, self._t0 = True, t
        ev = [('TASK_START', dict(tarefa='centro_fora', n=self.n_targets, alcance=self.reach_m,
                                  raio=self.radius, permanencia_s=self.hold_s,
                                  limite_s=self.timeout_s, semente=self.seed))]
        return ev + self._next(t)

    def _next(self, t):
        self.i += 1
        self._enter = None
        if self.i >= len(self.goals):
            self.done = True
            return [('TASK_END', dict(motivo='concluida', hits=self.hits,
                                      misses=self.misses,
                                      dur_s=round(t - self._t0, 2)))]
        self._t_goal = t
        fase, k, pos = self.goals[self.i]
        return [('TARGET_ON', dict(fase=fase, alvo=k, x=pos[0], y=pos[1],
                                   raio=self.radius))]

    def stop(self, t):
        if self.done or not self.started:
            return []
        self.done = True
        return [('TASK_END', dict(motivo='interrompida', hits=self.hits,
                                  misses=self.misses))]

    def update(self, t, x, y):
        if not self.started or self.done:
            return []
        fase, k, pos = self.goals[self.i]
        if math.hypot(x - pos[0], y - pos[1]) <= self.radius:
            if self._enter is None:
                self._enter = t
            if t - self._enter >= self.hold_s:
                if fase == 'ida':
                    self.hits += 1
                return [('TARGET_HIT', dict(fase=fase, alvo=k,
                        tempo_s=round(t - self._t_goal, 2)))] + self._next(t)
        else:
            self._enter = None
        if t - self._t_goal >= self.timeout_s:
            if fase == 'ida':
                self.misses += 1
            return [('TARGET_MISS', dict(fase=fase, alvo=k))] + self._next(t)
        return []

    def hold_progress(self, t):
        """0..1 da permanência no alvo atual (para a GUI)."""
        if self._enter is None or self.done:
            return 0.0
        return min(1.0, (t - self._enter) / self.hold_s)


# ── Túnel ───────────────────────────────────────────────────────────────────
def tunnel_path(shape: str, size_m: float, start, n: int = 400):
    """Trajetória [(x, y)…] que COMEÇA em `start` (o cursor já nasce dentro).

    reta: de `start` até +size_m em y (direita da tela).
    circulo: diâmetro size_m, centro em start + (0, size/2); sentido
    anti-horário na tela (desce primeiro).
    oito: lemniscata, largura 2·size/2 em y e size/2 em x, cruzando em `start`.
    """
    x0, y0 = start
    pts = []
    for i in range(n):
        u = i / (n - 1)
        if shape == 'reta':
            pts.append((x0, y0 + size_m * u))
        elif shape == 'circulo':
            r, th = size_m / 2.0, -math.pi / 2.0 + 2.0 * math.pi * u
            pts.append((x0 + r * math.cos(th), y0 + r + r * math.sin(th)))
        elif shape == 'oito':
            a, th = size_m / 2.0, 2.0 * math.pi * u
            pts.append((x0 + 0.5 * a * math.sin(2.0 * th), y0 + a * math.sin(th)))
        else:
            raise ValueError(f'forma desconhecida: {shape}')
    return pts


class TunnelTask:
    """Seguir a trajetória dentro de um túnel de largura `width_m`.

    O progresso só avança DENTRO do túnel, por uma janela à frente do último índice (não
    pula o cruzamento do oito) e no ritmo do cursor: cortar caminho não conta. Quem sai e
    volta mais à frente só avança o que andou na direção do túnel. Termina ao chegar ao fim. Dentro/fora gera
    TUNNEL_EXIT/TUNNEL_ENTER; `reference()` dá o ponto da assistência.
    """

    WINDOW = 0.15       # fração da trajetória procurada à frente
    GAIN = 2.0          # avanço no caminho por deslocamento do cursor (curva por dentro)
    HYST_M = 0.002      # folga para contar a volta: ruído na borda não vira saída

    def __init__(self, shape, start, size_m=0.20, width_m=0.04,
                 lookahead_m=0.02, n=400):
        self.shape, self.width, self.size = shape, width_m, size_m
        self._xy = tuple(start)      # cursor no tick anterior
        self._saiu = None            # onde o cursor deixou o túnel
        self.path = tunnel_path(shape, size_m, start, n)
        self.n = len(self.path)
        seg = sum(math.dist(self.path[i], self.path[i + 1]) for i in range(n - 1))
        self.length_m = seg
        self._ds = seg / (n - 1)
        self._look = max(1, int(round(lookahead_m / self._ds)))
        self.idx = 0
        self.started = self.done = False
        self.inside = True
        self.exits = 0
        self.t_in = self.t_total = 0.0
        self._t0 = self._t_last = 0.0
        self.dist = 0.0

    def start(self, t):
        self.started, self._t0, self._t_last = True, t, t
        return [('TASK_START', dict(tarefa='tunel_' + self.shape,
                                    largura=self.width, tamanho=self.size))]

    def stop(self, t):
        if self.done or not self.started:
            return []
        self.done = True
        return [('TASK_END', dict(motivo='interrompida',
                                  progresso=round(self.progress, 3)))]

    @property
    def progress(self):
        return self.idx / (self.n - 1)

    def reference(self):
        return self.path[min(self.idx + self._look, self.n - 1)]

    def update(self, t, x, y):
        if not self.started or self.done:
            return []
        lo = max(0, self.idx - 5)
        passo, antes, self._xy = math.dist(self._xy, (x, y)), self._xy, (x, y)
        alem = 0.0                   # andado no sentido do túnel desde a saída
        if self._saiu:
            a, b = self.path[max(0, self.idx - 1)], self.path[min(self.idx + 1, self.n - 1)]
            alem = ((x - self._saiu[0]) * (b[0] - a[0]) + (y - self._saiu[1]) * (b[1] - a[1])
                    ) / math.dist(a, b)
        hi = min(self.n, self.idx + max(int(self.WINDOW * self.n), int(alem / self._ds)) + 1)
        best = min(range(lo, hi),
                   key=lambda i: (self.path[i][0] - x) ** 2 + (self.path[i][1] - y) ** 2)
        self.dist = math.hypot(self.path[best][0] - x, self.path[best][1] - y)
        if self.dist <= self.width / 2.0:
            self.idx = max(self.idx, min(
                best, self.idx + math.ceil(max(self.GAIN * passo, alem) / self._ds)))
            self._saiu = None
        elif self._saiu is None:
            self._saiu = antes
        dt = t - self._t_last
        self._t_last = t
        self.t_total += dt
        ev = []
        now_in = self.dist <= self.width / 2.0 - (0.0 if self.inside else self.HYST_M)
        if now_in:
            self.t_in += dt
        if self.inside and not now_in:
            self.exits += 1
            ev.append(('TUNNEL_EXIT', dict(dist_m=self.dist,
                                           progresso=round(self.progress, 3))))
        elif not self.inside and now_in:
            ev.append(('TUNNEL_ENTER', dict(progresso=round(self.progress, 3))))
        self.inside = now_in
        if self.idx >= self.n - 2:
            self.done = True
            ev.append(('TASK_END', dict(motivo='concluida', saidas=self.exits,
                                        dentro_pct=round(self.inside_pct, 1),
                                        dur_s=round(t - self._t0, 2))))
        return ev

    @property
    def inside_pct(self):
        return 100.0 * self.t_in / self.t_total if self.t_total > 0 else 100.0


# ── Avaliação de alcance ────────────────────────────────────────────────────
class ReachTask:
    """Explorar o espaço por `duration_s`; mede a área alcançada."""

    def __init__(self, center, duration_s=60.0, half_m=0.25, cell_m=0.01):
        self.area = ExploredArea(center, half_m, cell_m)
        self.duration_s = duration_s
        self.started = self.done = False
        self._t0 = 0.0

    def start(self, t):
        self.started, self._t0 = True, t
        return [('TASK_START', dict(tarefa='alcance', dur=self.duration_s,
                                    meia_area=self.area.half, celula=self.area.cell))]

    def stop(self, t):
        if self.done or not self.started:
            return []
        self.done = True
        return [('TASK_END', dict(motivo='interrompida',
                                  area_cm2=round(self.area.area_m2 * 1e4, 1)))]

    def update(self, t, x, y):
        if not self.started or self.done:
            return []
        self.area.add(x, y)
        if t - self._t0 >= self.duration_s:
            self.done = True
            return [('TASK_END', dict(motivo='concluida',
                                      area_cm2=round(self.area.area_m2 * 1e4, 1)))]
        return []
