"""gui_ft_charts.py — gráficos da aba "6 Axes", em paridade com a FIBOS.

O `Six_Axis_FT.exe` desenha os seis canais com o qwt em dois gráficos (força e
torque, escalas diferentes) mais uma vista de colunas. Os defaults de escala
dele ficam em `Csv/Range.csv` da instalação, e é de lá que saem os números em
`FT_CHART_*` (ver constants.py).

Por que isto não mora no gui_loadcell.py: aquele arquivo já tem 792 linhas e é
o recorte de um arquivo que tinha 234 métodos numa classe só. Gráfico é outro
assunto — buffer circular, decimação e canvas — e cabe num mixin próprio, como
o MatrixMixin.

DECIMAÇÃO, E POR QUE ELA NÃO É OPCIONAL
---------------------------------------
A janela de fábrica é de 2000 amostras. A 1 kHz isso são 2 s, e redesenhar
2000 pontos x 6 traços a 10 Hz satura a thread do Tk — foi exatamente o que já
travou a GUI inteira no heatmap do toque (ver _build_sensors_tab). Mas
subamostrar por passo (pegar 1 a cada k) ESCONDE PICO, que num sensor de força
é justamente o dado que importa.

A saída é decimação por min/max: cada coluna de pixel vira dois pontos, o menor
e o maior daquela fatia. O envelope do sinal fica intacto, o custo passa a
depender do número de PIXELS em vez do número de amostras, e um pico de 1 ms
dentro de um buffer de 2 s continua visível.
"""
from __future__ import annotations

import collections
import tkinter as tk

from .constants import (
    FT_AXES, FT_AXIS_LABELS, FT_CHART_FORCE_MAX, FT_CHART_TORQUE_MAX,
    FT_CHART_WINDOW_N, FT_RATED_FORCE_N, FT_RATED_TORQUE_NM, ft_axis_rated,
)
from .ui_helpers import (
    PANEL, TEXT, TEXT_MUTED, TEXT_DIM, PRIMARY, PRIMARY_HV,
    DANGER, BORDER, FONT_LBL, FONT_SMALL, FONT_MONO_S,
)

# Cor por canal. Seis matizes distinguíveis sobre o branco do PANEL, com as
# três de força ecoando as três de torque no mesmo eixo (x azul, y verde, z
# vermelho) para a leitura cruzada entre os dois gráficos ser imediata.
FT_SERIES_COLOR = {
    'fx': '#2563eb', 'fy': '#16a34a', 'fz': '#dc2626',
    'mx': '#7c3aed', 'my': '#0891b2', 'mz': '#ea580c',
}

_CHART_H = 190          # altura útil de cada gráfico, em px
_CHART_COLS = 360       # colunas de decimação (até 2 pontos por coluna)
_COL_W = 46             # largura de cada barra da vista de colunas
_COL_H = 150


def decimate_minmax(seq, cols: int = _CHART_COLS):
    """Envelope de `seq` em no máximo `cols` colunas, 2 pontos por coluna.

    Devolve [(frac_x, valor), …] com frac_x em 0..1, para o desenho não
    precisar saber a largura do canvas — é o que deixa o gráfico responder a
    redimensionamento sem recalcular a decimação.

    Sequências menores que `cols` saem intactas: decimar o que já cabe só
    perderia resolução de graça.
    """
    n = len(seq)
    if n == 0:
        return []
    if n <= cols:
        return [((i / (n - 1)) if n > 1 else 0.0, v) for i, v in enumerate(seq)]
    out = []
    for j in range(cols):
        ini = j * n // cols
        fim = max((j + 1) * n // cols, ini + 1)
        fatia = seq[ini:fim]
        x = j / (cols - 1)
        lo, hi = min(fatia), max(fatia)
        # Ordem lo -> hi mantém a polilinha contínua entre colunas vizinhas.
        out.append((x, lo))
        if hi != lo:
            out.append((x, hi))
    return out


class FtChartsMixin:
    """Gráficos de linha, vista de colunas e as escalas do Range.csv."""

    # ── Estado ────────────────────────────────────────────────────────
    def _ft_charts_init(self) -> None:
        """Buffers circulares. Alimentados pela thread do ROS e lidos pela do
        Tk — sempre sob o mesmo `self._lock` do resto da aba."""
        self._ft_hist = {a: collections.deque(maxlen=FT_CHART_WINDOW_N)
                         for a in FT_AXES}
        self._ft_chart_win = FT_CHART_WINDOW_N
        self._ft_chart_fmax = FT_CHART_FORCE_MAX
        self._ft_chart_mmax = FT_CHART_TORQUE_MAX
        self._ft_chart_paused = False
        self._ft_chart_lines = {}      # eixo -> id da polilinha no canvas
        self._ft_chart_cv = {}
        self._ft_col_widgets = {}

    def _ft_charts_feed(self, vals) -> None:
        """Uma amostra dos seis eixos.

        Chamado de DENTRO do lock de `_ft_feed_processing`: o lock em uso é um
        `threading.Lock` simples, não reentrante, então pegá-lo de novo aqui
        travaria a thread do sensor de vez.
        """
        hist = getattr(self, '_ft_hist', None)
        if hist is None:
            return                      # aba ainda não construída
        if self._ft_chart_paused:
            return
        for a, v in zip(FT_AXES, vals):
            hist[a].append(v)

    # ── Gráficos de linha ─────────────────────────────────────────────
    def _build_ft_chart_card(self, root: tk.Frame) -> None:
        card = self._card(root, 'Charts — force & torque', expand=False)

        tk.Label(
            card,
            text=('Defaults from the factory install (Csv/Range.csv): '
                  f'{FT_CHART_WINDOW_N} samples, '
                  f'±{FT_CHART_FORCE_MAX:.0f} N, '
                  f'±{FT_CHART_TORQUE_MAX:.0f} N·m. Those are series '
                  f'defaults, not this unit: rated is '
                  f'±{FT_RATED_FORCE_N:.0f} N / '
                  f'±{FT_RATED_TORQUE_NM:.0f} N·m.'),
            font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
            justify='left', wraplength=780).pack(fill='x', pady=(6, 6))

        for grupo, titulo, unidade in (
                ('force', 'Force', 'N'), ('torque', 'Torque', 'N·m')):
            tk.Label(card, text=f'{titulo}  [{unidade}]', font=FONT_LBL,
                     bg=PANEL, fg=TEXT_MUTED, anchor='w').pack(
                fill='x', pady=(6, 2))
            cv = tk.Canvas(card, height=_CHART_H, bg=PANEL,
                           highlightthickness=1, highlightbackground=BORDER)
            cv.pack(fill='x')
            self._ft_chart_cv[grupo] = cv
            cv.create_line(0, _CHART_H // 2, 4000, _CHART_H // 2,
                           fill=BORDER, tags='zero')
            cv.create_text(4, 8, anchor='nw', text='', font=FONT_MONO_S,
                           fill=TEXT_DIM, tags='top')
            cv.create_text(4, _CHART_H - 8, anchor='sw', text='',
                           font=FONT_MONO_S, fill=TEXT_DIM, tags='bot')

        # As polilinhas são criadas UMA vez e depois só recebem coords(). O
        # ciclo delete/create a 10 Hz fragmenta a display list do Tk e é o que
        # faz a GUI engasgar depois de alguns minutos.
        for axis, _label, _u in FT_AXIS_LABELS:
            grupo = 'force' if axis.startswith('f') else 'torque'
            self._ft_chart_lines[axis] = self._ft_chart_cv[grupo].create_line(
                0, 0, 0, 0, fill=FT_SERIES_COLOR[axis], width=1.4,
                smooth=False)

        leg = tk.Frame(card, bg=PANEL)
        leg.pack(fill='x', pady=(6, 2))
        for axis, label, _unidade in FT_AXIS_LABELS:
            cel = tk.Frame(leg, bg=PANEL)
            cel.pack(side='left', padx=(0, 14))
            tk.Canvas(cel, width=14, height=3, bg=FT_SERIES_COLOR[axis],
                      highlightthickness=0).pack(side='left', pady=(6, 0))
            tk.Label(cel, text=f' {label}', font=FONT_SMALL, bg=PANEL,
                     fg=TEXT_MUTED).pack(side='left')

        # Controles de escala — o equivalente editável do Range.csv.
        row = tk.Frame(card, bg=PANEL)
        row.pack(fill='x', pady=(8, 2))
        self._ft_chart_win_var = tk.StringVar(value=str(FT_CHART_WINDOW_N))
        self._ft_chart_fmax_var = tk.StringVar(
            value=f'{FT_CHART_FORCE_MAX:.0f}')
        self._ft_chart_mmax_var = tk.StringVar(
            value=f'{FT_CHART_TORQUE_MAX:.0f}')
        for rotulo, var, larg in (
                ('window (samples)', self._ft_chart_win_var, 7),
                ('±N', self._ft_chart_fmax_var, 6),
                ('±N·m', self._ft_chart_mmax_var, 6)):
            tk.Label(row, text=rotulo, font=FONT_SMALL, bg=PANEL,
                     fg=TEXT_DIM).pack(side='left', padx=(0, 4))
            tk.Entry(row, textvariable=var, width=larg,
                     font=FONT_MONO_S).pack(side='left', padx=(0, 14))
        tk.Button(row, text='Apply', command=self._ft_apply_chart_scale,
                  bg=PRIMARY, fg='white', activebackground=PRIMARY_HV,
                  activeforeground='white', font=FONT_SMALL, relief='flat',
                  bd=0, padx=10, pady=3, cursor='hand2').pack(side='left')
        tk.Button(row, text='Factory defaults',
                  command=self._ft_reset_chart_scale,
                  bg=PANEL, fg=TEXT_MUTED, font=FONT_SMALL, relief='flat',
                  bd=1, padx=10, pady=3, cursor='hand2').pack(
            side='left', padx=(8, 0))
        self._ft_chart_pause_var = tk.BooleanVar(value=False)
        tk.Checkbutton(row, text='Freeze', variable=self._ft_chart_pause_var,
                       command=self._ft_toggle_chart_pause, bg=PANEL, fg=TEXT,
                       selectcolor=PANEL, activebackground=PANEL,
                       activeforeground=TEXT, font=FONT_SMALL,
                       highlightthickness=0, bd=0).pack(side='left',
                                                        padx=(16, 0))
        self._ft_chart_lbl = tk.Label(card, text='', font=FONT_SMALL,
                                      bg=PANEL, fg=TEXT_DIM, anchor='w')
        self._ft_chart_lbl.pack(fill='x', pady=(4, 2))

    def _ft_apply_chart_scale(self) -> None:
        try:
            win = int(self._ft_chart_win_var.get())
            fmax = float(self._ft_chart_fmax_var.get())
            mmax = float(self._ft_chart_mmax_var.get())
        except ValueError:
            self._ft_chart_lbl.config(text='Scale values must be numbers.',
                                      fg=DANGER)
            return
        if win < 10 or fmax <= 0 or mmax <= 0:
            self._ft_chart_lbl.config(
                text='Window must be ≥ 10 samples and scales > 0.', fg=DANGER)
            return
        with self._lock:
            self._ft_chart_fmax, self._ft_chart_mmax = fmax, mmax
            if win != self._ft_chart_win:
                # `maxlen` é imutável: mudar a janela recria os buffers,
                # preservando as amostras que ainda couberem.
                self._ft_chart_win = win
                self._ft_hist = {
                    a: collections.deque(self._ft_hist[a], maxlen=win)
                    for a in FT_AXES}
        taxa = max(float(getattr(self, '_ft_rate_hz', 0.0) or 0.0), 1.0)
        self._ft_chart_lbl.config(
            text=(f'Window {win} samples ≈ {win / taxa:.1f} s at the measured '
                  f'rate  ·  ±{fmax:g} N  ·  ±{mmax:g} N·m'), fg=TEXT_DIM)

    def _ft_reset_chart_scale(self) -> None:
        self._ft_chart_win_var.set(str(FT_CHART_WINDOW_N))
        self._ft_chart_fmax_var.set(f'{FT_CHART_FORCE_MAX:.0f}')
        self._ft_chart_mmax_var.set(f'{FT_CHART_TORQUE_MAX:.0f}')
        self._ft_apply_chart_scale()

    def _ft_toggle_chart_pause(self) -> None:
        with self._lock:
            self._ft_chart_paused = bool(self._ft_chart_pause_var.get())

    # ── Vista de colunas ──────────────────────────────────────────────
    def _build_ft_columns_card(self, root: tk.Frame) -> None:
        card = self._card(root, 'Columns — % of rated', expand=False)
        tk.Label(
            card,
            text=('Each bar is the axis against the rated range of THIS unit '
                  f'(±{FT_RATED_FORCE_N:.0f} N, '
                  f'±{FT_RATED_TORQUE_NM:.0f} N·m) — not the chart scale '
                  'above, which carries the generic factory defaults.'),
            font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
            justify='left', wraplength=780).pack(fill='x', pady=(6, 8))

        row = tk.Frame(card, bg=PANEL)
        row.pack(fill='x')
        for axis, label, unidade in FT_AXIS_LABELS:
            cel = tk.Frame(row, bg=PANEL)
            cel.pack(side='left', padx=(0, 10))
            cv = tk.Canvas(cel, width=_COL_W, height=_COL_H, bg=PANEL,
                           highlightthickness=1, highlightbackground=BORDER)
            cv.pack()
            meio = _COL_H // 2
            cv.create_line(0, meio, _COL_W, meio, fill=BORDER)
            barra = cv.create_rectangle(6, meio, _COL_W - 6, meio,
                                        fill=FT_SERIES_COLOR[axis],
                                        outline='')
            val = tk.Label(cel, text='—', font=FONT_MONO_S, bg=PANEL,
                           fg=TEXT_DIM)
            val.pack()
            tk.Label(cel, text=label, font=FONT_LBL, bg=PANEL,
                     fg=TEXT_MUTED).pack()
            self._ft_col_widgets[axis] = {
                'cv': cv, 'bar': barra, 'val': val, 'unit': unidade,
                'rated': ft_axis_rated(axis)}

    # ── Repintura, chamada pelo _refresh_ft_axes (10 Hz) ──────────────
    def _refresh_ft_charts(self, shown: dict, live: bool) -> None:
        if not getattr(self, '_ft_chart_lines', None):
            return
        with self._lock:
            dados = {a: list(self._ft_hist[a]) for a in FT_AXES}
            fmax, mmax = self._ft_chart_fmax, self._ft_chart_mmax

        for grupo, escala in (('force', fmax), ('torque', mmax)):
            cv = self._ft_chart_cv[grupo]
            larg = max(cv.winfo_width(), 2)
            cv.itemconfigure('top', text=f'+{escala:g}')
            cv.itemconfigure('bot', text=f'-{escala:g}')
            cv.coords('zero', 0, _CHART_H / 2, larg, _CHART_H / 2)

        for axis, _label, _u in FT_AXIS_LABELS:
            grupo = 'force' if axis.startswith('f') else 'torque'
            escala = fmax if grupo == 'force' else mmax
            cv = self._ft_chart_cv[grupo]
            larg = max(cv.winfo_width(), 2)
            pts = decimate_minmax(dados[axis])
            linha = self._ft_chart_lines[axis]
            if len(pts) < 2:
                # create_line exige dois pontos; um traço degenerado some.
                cv.coords(linha, 0, _CHART_H / 2, 0, _CHART_H / 2)
                continue
            coords = []
            meio = _CHART_H / 2
            for fx, v in pts:
                y = meio - (v / escala) * (meio - 4)
                # Saturar no canvas em vez de deixar o Tk desenhar fora: um
                # valor de sobrecarga levaria a polilinha para coordenadas
                # absurdas e o widget inteiro engasga.
                coords += [fx * larg, max(1.0, min(_CHART_H - 1.0, y))]
            cv.coords(linha, *coords)

        for axis, wid in self._ft_col_widgets.items():
            v = shown.get(axis)
            meio = _COL_H / 2
            if v is None or not live:
                wid['val'].config(text='—', fg=TEXT_DIM)
                wid['cv'].coords(wid['bar'], 6, meio, _COL_W - 6, meio)
                continue
            rated = wid['rated']
            frac = 0.0 if rated <= 0 else max(-1.0, min(1.0, v / rated))
            y = meio - frac * (meio - 4)
            wid['cv'].coords(wid['bar'], 6, min(y, meio), _COL_W - 6,
                             max(y, meio))
            wid['val'].config(text=f'{v:+.2f}',
                              fg=DANGER if abs(frac) >= 1.0 else TEXT)
