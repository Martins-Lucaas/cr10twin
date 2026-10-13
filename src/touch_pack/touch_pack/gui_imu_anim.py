"""gui_imu_anim.py — sensor 3D animado que mostra os movimentos da calibração MAG.

Um Canvas do Tk, sem dependência nova: uma placa (o IMU) sombreada, com a seta
da frente no topo, um "chão" de referência e a trajetória do movimento atual
desenhada em volta (arco com seta para girar/inclinar, ∞ para o oito). Projeção
ortográfica e algoritmo do pintor — com 6 faces não precisa de mais nada.

As fases e a duração de cada uma são as mesmas das dicas do assistente
(`MAG_PHASES`, `MAG_PHASE_S`), então a animação e o texto andam juntos.
"""
from __future__ import annotations

import math
import time
import tkinter as tk

import numpy as np

from .ui_helpers import FONT_SMALL, PANEL, PRIMARY, TEXT_MUTED

MAG_PHASE_S = 6.0
MAG_PHASES = (
    ('roll', 'Roll it slowly around its long axis ↻'),
    ('pitch', 'Tilt the nose up and down ⤴⤵'),
    ('yaw', 'Turn it left and right, flat (yaw) ⟲'),
    ('eight', 'Draw slow figure-eights with the nose ∞'),
    ('flip', 'Upside down: tilt and roll again ↻'),
)

HALF = np.array([1.0, 0.6, 0.22])      # placa: x = eixo longo (a "frente")
_FACES = (  # (vértices, normal no frame do sensor, cor); vértice i = bits (x, y, z)
    ((1, 5, 7, 3), (0, 0, 1), PRIMARY),        # topo
    ((0, 2, 6, 4), (0, 0, -1), '#475569'),     # fundo
    ((4, 6, 7, 5), (1, 0, 0), '#f59e0b'),      # frente (+x)
    ((0, 1, 3, 2), (-1, 0, 0), '#94a3b8'),     # trás
    ((2, 3, 7, 6), (0, 1, 0), '#cbd5e1'),
    ((0, 4, 5, 1), (0, -1, 0), '#cbd5e1'),
)
_LIGHT = np.array([0.35, -0.45, 0.82]) / np.linalg.norm([0.35, -0.45, 0.82])


def _rx(d):
    a = math.radians(d)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def _ry(d):
    a = math.radians(d)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def _rz(d):
    a = math.radians(d)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def phase_pose(phase: str, t: float) -> np.ndarray:
    """Orientação do sensor (3×3) em `t` segundos dentro da fase."""
    u = (t % MAG_PHASE_S) / MAG_PHASE_S
    s1, s2 = math.sin(2 * math.pi * u), math.sin(4 * math.pi * u)
    if phase == 'roll':
        return _rx(360.0 * u)
    if phase == 'pitch':
        return _ry(65.0 * s2)
    if phase == 'yaw':
        return _rz(360.0 * u)
    if phase == 'eight':
        return _rz(50.0 * s1) @ _ry(30.0 * s2)
    return _rx(180.0) @ _ry(55.0 * s2) @ _rx(40.0 * s1)   # flip


def _tint(hex_color: str, k: float) -> str:
    h = hex_color.lstrip('#')
    r, g, b = (min(255, int(int(h[i:i + 2], 16) * k)) for i in (0, 2, 4))
    return f'#{r:02x}{g:02x}{b:02x}'


class Sensor3DAnim(tk.Canvas):
    """Placa 3D girando. `demo=True` percorre as fases sozinha (passo de
    preparo); senão `set_clock(t0)` sincroniza com a coleta (fase = tempo
    desde t0 / MAG_PHASE_S)."""

    def __init__(self, parent, width=300, height=220, demo=True):
        super().__init__(parent, width=width, height=height, bg=PANEL,
                         highlightthickness=0)
        self.w, self.h, self.scale = width, height, min(width, height) * 0.30
        self.demo = demo
        self.t0 = time.monotonic()
        el, az = math.radians(24.0), math.radians(-38.0)
        c = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        self._fwd = -c
        self._right = np.cross(self._fwd, [0, 0, 1])
        self._right /= np.linalg.norm(self._right)
        self._up = np.cross(self._right, self._fwd)
        self._verts = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                                for sz in (-1, 1)], float) * HALF
        self._after = None
        self.bind('<Destroy>', lambda _e: self._stop())
        self._tick()

    def set_clock(self, t0: float) -> None:
        self.t0 = t0

    def phase_index(self, now: float | None = None) -> int:
        el = (now or time.monotonic()) - self.t0
        return int(el // MAG_PHASE_S) % len(MAG_PHASES)

    # ── Projeção ─────────────────────────────────────────────────────
    def _xy(self, p):
        p = np.atleast_2d(p)
        x = self.w / 2 + self.scale * (p @ self._right)
        y = self.h / 2 - 10 - self.scale * (p @ self._up)
        return np.column_stack([x, y])

    def _poly(self, pts, **kw):
        return self.create_polygon(*self._xy(pts).ravel().tolist(), **kw)

    def _line(self, pts, **kw):
        return self.create_line(*self._xy(pts).ravel().tolist(), **kw)

    # ── Desenho ──────────────────────────────────────────────────────
    def _guide(self, phase: str, R: np.ndarray) -> None:
        """Trajetória do movimento atual, em volta da placa."""
        a = np.linspace(0.15, 2 * math.pi - 0.35, 40)
        kw = dict(fill=TEXT_MUTED, width=2, dash=(5, 3), arrow='last',
                  arrowshape=(10, 12, 4), smooth=True)
        if phase in ('roll', 'flip'):
            pts = np.column_stack([np.full_like(a, -1.45), 0.75 * np.cos(a), 0.75 * np.sin(a)])
        elif phase == 'pitch':
            b = np.linspace(-1.0, 1.0, 30)
            pts = np.column_stack([1.45 * np.cos(b), np.zeros_like(b), 1.45 * np.sin(b)])
            self._line(pts[::-1], **kw)
        elif phase == 'yaw':
            pts = np.column_stack([1.4 * np.cos(a), 1.4 * np.sin(a), np.full_like(a, -0.35)])
        else:   # oito: o caminho que a ponta (nariz) percorre
            u = np.linspace(0, 2 * math.pi, 80)
            tips = np.array([phase_pose('eight', x / (2 * math.pi) * MAG_PHASE_S)
                             @ [1.25, 0, 0] for x in u])
            self._line(tips, fill=TEXT_MUTED, width=2, dash=(5, 3), smooth=True)
            return
        self._line(pts, **kw)

    def _draw(self, now: float) -> None:
        self.delete('all')
        idx = self.phase_index(now)
        phase, tip = MAG_PHASES[idx]
        R = phase_pose(phase, now - self.t0)
        # chão de referência (dá profundidade)
        g = np.linspace(0, 2 * math.pi, 48)
        self._line(np.column_stack([1.7 * np.cos(g), 1.7 * np.sin(g), np.full_like(g, -0.95)]),
                   fill='#e2e8f0', width=2, smooth=True)
        self._guide(phase, R)
        V = self._verts @ R.T
        faces = []
        for idxs, n, color in _FACES:
            nw = R @ np.array(n, float)
            if nw @ self._fwd >= -1e-6:      # de costas para a câmera
                continue
            depth = float(V[list(idxs)].mean(axis=0) @ self._fwd)
            faces.append((depth, idxs, nw, color))
        for _d, idxs, nw, color in sorted(faces, key=lambda f: -f[0]):
            k = 0.5 + 0.6 * max(0.0, float(nw @ _LIGHT))
            self._poly(V[list(idxs)], fill=_tint(color, k), outline='#1e293b', width=1)
            if idxs == _FACES[0][0]:          # seta da frente no topo
                arrow = np.array([[-0.55, -0.12, 0], [0.25, -0.12, 0], [0.25, -0.3, 0],
                                  [0.7, 0, 0], [0.25, 0.3, 0], [0.25, 0.12, 0],
                                  [-0.55, 0.12, 0]]) + [0, 0, HALF[2] + 0.01]
                self._poly(arrow @ R.T, fill='white', outline='')
        self.create_text(self.w / 2, self.h - 12, font=FONT_SMALL, fill=TEXT_MUTED,
                         text=f'{idx + 1}/{len(MAG_PHASES)}  {tip}')

    def _tick(self) -> None:
        self._after = None
        try:
            self._draw(time.monotonic())
        except tk.TclError:
            return
        self._after = self.after(40, self._tick)

    def _stop(self) -> None:
        if self._after is not None:
            try:
                self.after_cancel(self._after)
            except tk.TclError:
                pass
            self._after = None
