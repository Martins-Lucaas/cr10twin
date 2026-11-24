"""Gráfico de linhas ao vivo (tk.Canvas) do painel rehab_gui.

Janela deslizante de 10 s, decimação min/max por coluna de pixel
(tasks.decimate_minmax), eixo y com passos "redondos" (1·2·5), linha do zero,
grade discreta, legenda com o valor atual de cada série e cruz de leitura
ao passar o mouse.
"""
from __future__ import annotations

import collections
import math
import tkinter as tk

from . import gui_theme as th
from .tasks import decimate_minmax


def nice_ticks(lo: float, hi: float, n: int = 5):
    """Marcas do eixo em passos 1·2·2,5·5×10^k dentro de [lo, hi]
    (≈ n intervalos; nunca mais que n + 1 marcas)."""
    if not hi > lo:
        return [lo]
    raw = (hi - lo) / max(n, 1)
    mag = 10 ** math.floor(math.log10(raw))
    step = next(m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw)
    first = math.ceil(lo / step - 1e-9) * step
    out, v = [], first
    while v <= hi + 1e-9 * step:
        out.append(0.0 if abs(v) < step * 1e-9 else v)
        v += step
    return out


def fmt_num(v: float, step: float) -> str:
    dec = 0
    while dec < 4 and abs(step * 10 ** dec - round(step * 10 ** dec)) > 1e-6:
        dec += 1
    return f'{v:.{dec}f}'.replace('-', '−')


class LineChart(tk.Canvas):
    """Séries (nome, cor, espessura) contra o tempo; unidade no título."""

    L, R, TOP, BOT = 56, 12, 30, 22

    def __init__(self, parent, titulo, unidade, series, window_s=10.0,
                 min_span=1.0, height=170):
        super().__init__(parent, height=height, bg=th.PANEL, highlightthickness=1,
                         highlightbackground=th.BORDER)
        self.titulo, self.unidade, self.series = titulo, unidade, series
        self.window_s, self.min_span = window_s, min_span
        self.ts = collections.deque()
        self.vs = [collections.deque() for _ in series]
        self._hover_x = None
        self.bind('<Motion>', lambda e: setattr(self, '_hover_x', e.x))
        self.bind('<Leave>', lambda e: setattr(self, '_hover_x', None))

    def clear(self):
        self.ts.clear()
        for v in self.vs:
            v.clear()

    def add(self, t, valores):
        if self.ts and t < self.ts[-1] - 1.0:      # relógio voltou (nova simulação)
            self.clear()
        self.ts.append(t)
        for d, v in zip(self.vs, valores):
            d.append(v)
        while self.ts and self.ts[0] < t - self.window_s:
            self.ts.popleft()
            for d in self.vs:
                d.popleft()

    def _legend(self, w, valores):
        """Legenda à direita do título: traço na cor + nome + valor (texto em
        tinta neutra; a cor só no traço)."""
        x = w - self.R
        for (nome, cor, larg), v in reversed(list(zip(self.series, valores))):
            txt = f'{nome} {v:+.1f}'.replace('-', '−') if v is not None else nome
            it = self.create_text(x, 15, anchor='e', text=txt, fill=th.TEXT,
                                  font=th.FONT_MONO)
            x0 = self.bbox(it)[0]
            self.create_line(x0 - 22, 15, x0 - 6, 15, fill=cor, width=max(3, larg + 1))
            x = x0 - 34

    def redraw(self):
        self.delete('all')
        w, h = max(self.winfo_width(), 240), max(self.winfo_height(), 100)
        L, R, TOP, BOT = self.L, self.R, self.TOP, self.BOT
        self.create_text(L - 44, 15, anchor='w', text=f'{self.titulo} ({self.unidade})',
                         fill=th.TEXT, font=th.FONT_TITLE)
        if len(self.ts) < 2:
            self._legend(w, [None] * len(self.series))
            self.create_rectangle(L, TOP, w - R, h - BOT, outline=th.GRID)
            self.create_text((L + w - R) / 2, (TOP + h - BOT) / 2, fill=th.MUTED,
                             text='Sem dados: aguardando o nó de admitância '
                                  '(/rehab/admittance/tick).')
            return
        self._legend(w, [d[-1] for d in self.vs])
        lo = min(min(d) for d in self.vs)
        hi = max(max(d) for d in self.vs)
        if hi - lo < self.min_span:
            mid = (hi + lo) / 2
            lo, hi = mid - self.min_span / 2, mid + self.min_span / 2
        pad = 0.06 * (hi - lo)
        lo, hi = lo - pad, hi + pad
        t1 = self.ts[-1]
        t0 = t1 - self.window_s
        pw, ph = w - L - R, h - TOP - BOT
        X = lambda t: L + (t - t0) / self.window_s * pw
        Y = lambda v: h - BOT - (v - lo) / (hi - lo) * ph
        ticks = nice_ticks(lo, hi, max(3, int(ph / 28)))
        step = ticks[1] - ticks[0] if len(ticks) > 1 else 1.0
        for v in ticks:
            y = Y(v)
            self.create_line(L, y, w - R, y, fill=th.BORDER if v == 0 else th.GRID,
                             width=1)
            self.create_text(L - 6, y, anchor='e', text=fmt_num(v, step),
                             fill=th.MUTED, font=th.FONT_SMALL)
        for k in range(0, int(self.window_s) + 1, 2):
            x = X(t0 + k)
            self.create_line(x, h - BOT, x, h - BOT + 4, fill=th.BORDER)
            rot = 'agora' if k == int(self.window_s) else f'−{self.window_s - k:.0f} s'
            self.create_text(x, h - BOT + 5, anchor='ne' if k == int(self.window_s) else 'n',
                             text=rot,
                             fill=th.MUTED, font=th.FONT_SMALL)
        ts = list(self.ts)
        cols = max(10, int(pw))
        for (nome, cor, larg), d in zip(self.series, self.vs):
            pts = decimate_minmax(ts, list(d), cols)
            flat = [c for t, v in pts for c in (X(t), Y(v))]
            if len(flat) >= 4:
                self.create_line(*flat, fill=cor, width=larg)
        self._hover(ts, X, h, L, w - R, TOP)

    def _hover(self, ts, X, h, x_lo, x_hi, top):
        """Cruz de leitura: linha vertical + valores no instante sob o mouse."""
        hx = self._hover_x
        if hx is None or not x_lo <= hx <= x_hi:
            return
        i = min(range(len(ts)), key=lambda k: abs(X(ts[k]) - hx))
        x = X(ts[i])
        self.create_line(x, top, x, h - self.BOT, fill=th.MUTED, dash=(2, 2))
        linhas = [f'{ts[i] - ts[-1]:+.2f} s'.replace('-', '−')] + [
            f'{nome:>3} {d[i]:+8.2f}'.replace('-', '−')
            for (nome, _c, _l), d in zip(self.series, self.vs)]
        anchor = 'nw' if x < (x_lo + x_hi) / 2 else 'ne'
        it = self.create_text(x + (8 if anchor == 'nw' else -8), top + 4, anchor=anchor,
                              text='\n'.join(linhas), fill=th.TEXT, font=th.FONT_MONO)
        bx = self.bbox(it)
        bg = self.create_rectangle(bx[0] - 4, bx[1] - 3, bx[2] + 4, bx[3] + 3,
                                   fill=th.PANEL, outline=th.BORDER)
        self.tag_lower(bg, it)
