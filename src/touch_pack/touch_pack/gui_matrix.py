"""
gui_matrix.py — configurador visual da grade do MATRIX_MAP.

Fatia de `palpation_gui.py` (ver o cabecalho de gui_loadcell.py para o
porque do recorte). Aqui mora a geometria da grade — nos, serpentina,
ordem de visita — e o preview desenhado no canvas.

Os waypoints gerados aqui sao coordenadas NO PLANO DA AMOSTRA, nao no XY
do mundo: quem os executa (tactile_explorer._matrix_plane_basis) monta a
base do plano medido. Sem calibracao do angulo de ataque o plano e o
horizontal e as duas coisas coincidem.
"""
from __future__ import annotations

from .constants import (MATRIX_SAFE_Z_MM_DEFAULT, MATRIX_SAFE_Z_MM_MIN, MATRIX_SAFE_Z_MM_MAX, MATRIX_TRANSIT_MMS_MIN, MATRIX_TRANSIT_MMS_MAX, MATRIX_MAX_POINTS, MATRIX_SPAN_MAX_MM, PROBE_ALIGN_RADIUS_MM_MIN)
from .plane_probe import probe_ring_from_grid
from .ui_helpers import (BG, PANEL, TEXT, TEXT_MUTED, TEXT_DIM, PRIMARY, PRIMARY_HV, OK, WARN, DANGER, BORDER, BTN_NEUTRAL, FONT_LBL, FONT_SMALL, _Tooltip)
import tkinter as tk

from .gui_constants import (
    MATRIX_STEP_MIN, MATRIX_STEP_MAX, MATRIX_N_MIN, MATRIX_N_MAX,
    MATRIX_SHAPES, MATRIX_SIZING_MODES, MATRIX_SIZE_MIN, MATRIX_SIZE_MAX,
    MATRIX_PATH_ORDERS, FORCE_SP_DEFAULT,
)


class MatrixMixin:
    """Mixin de `PalpationGUI` — todo o estado vive no host."""

    def _build_matrix_group(self, parent) -> None:
        """Bloco do MATRIX_MAP: formato + passos/linhas/colunas + Safe Z +
        velocidade de trânsito, com preview em Canvas das identações.
        """
        # ── Formato do plano ─────────────────────────────────────────
        row = tk.Frame(parent, bg=PANEL); row.pack(fill='x', pady=(8, 2))
        top = tk.Frame(row, bg=PANEL); top.pack(fill='x')
        shape_lbl = tk.Label(top, text='Plane Shape', font=FONT_LBL,
                             bg=PANEL, fg=TEXT, anchor='w')
        shape_lbl.pack(side='left')
        info = tk.Label(top, text='ⓘ', font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM)
        info.pack(side='left', padx=(5, 0))
        _hint = ('Square: one step and one count for both axes. '
                 'Rectangle: independent X and Y steps and counts. '
                 'Points are laid out from the origin along +X/+Y and visited '
                 'in a serpentine (boustrophedon) path to minimise travel.')
        _Tooltip(shape_lbl, _hint)
        _Tooltip(info, _hint)
        btns = tk.Frame(row, bg=PANEL); btns.pack(fill='x', pady=(4, 2))
        self._matrix_shape_btns: dict[str, tk.Button] = {}
        for key, txt in (('SQUARE', 'Square'), ('RECT', 'Rectangle')):
            b = tk.Button(btns, text=txt,
                          command=lambda k=key: self._on_matrix_shape(k),
                          bg=BTN_NEUTRAL, fg=TEXT, font=FONT_LBL,
                          activebackground=PRIMARY_HV,
                          activeforeground='white',
                          relief='flat', bd=0, padx=14, pady=6,
                          cursor='hand2')
            b.pack(side='left', fill='x', expand=True,
                   padx=(0 if key == 'SQUARE' else 4, 0))
            self._matrix_shape_btns[key] = b

        # ── Dimensionamento: por passo ou pelas dimensões do alvo ────
        row = tk.Frame(parent, bg=PANEL); row.pack(fill='x', pady=(8, 2))
        top = tk.Frame(row, bg=PANEL); top.pack(fill='x')
        siz_lbl = tk.Label(top, text='Grid From', font=FONT_LBL,
                           bg=PANEL, fg=TEXT, anchor='w')
        siz_lbl.pack(side='left')
        siz_info = tk.Label(top, text='ⓘ', font=FONT_SMALL, bg=PANEL,
                            fg=TEXT_DIM)
        siz_info.pack(side='left', padx=(5, 0))
        _shint = ('Step: you set the spacing and the covered area follows. '
                  'Target size: you set the sample dimensions and the '
                  'spacing is derived as size/(count−1), so the grid always '
                  'spans the piece edge to edge.')
        _Tooltip(siz_lbl, _shint)
        _Tooltip(siz_info, _shint)
        sbtns = tk.Frame(row, bg=PANEL); sbtns.pack(fill='x', pady=(4, 2))
        self._matrix_sizing_btns: dict[str, tk.Button] = {}
        for key, txt in (('STEP', 'Step'), ('SIZE', 'Target size')):
            b = tk.Button(sbtns, text=txt,
                          command=lambda k=key: self._on_matrix_sizing(k),
                          bg=BTN_NEUTRAL, fg=TEXT, font=FONT_LBL,
                          activebackground=PRIMARY_HV,
                          activeforeground='white',
                          relief='flat', bd=0, padx=14, pady=6,
                          cursor='hand2')
            b.pack(side='left', fill='x', expand=True,
                   padx=(0 if key == 'STEP' else 4, 0))
            self._matrix_sizing_btns[key] = b

        # ── Ordem de visita ──────────────────────────────────────────
        row = tk.Frame(parent, bg=PANEL); row.pack(fill='x', pady=(8, 2))
        top = tk.Frame(row, bg=PANEL); top.pack(fill='x')
        pth_lbl = tk.Label(top, text='Visit Order', font=FONT_LBL,
                           bg=PANEL, fg=TEXT, anchor='w')
        pth_lbl.pack(side='left')
        pth_info = tk.Label(top, text='ⓘ', font=FONT_SMALL, bg=PANEL,
                            fg=TEXT_DIM)
        pth_info.pack(side='left', padx=(5, 0))
        _phint = ('Corners first: touches the four extreme points of the '
                  'grid before anything else, as a registration check — if '
                  'the target dimensions or the origin are wrong, the '
                  'corners are where the grid leaves the sample first, and '
                  'you find out after 4 indentations instead of a whole '
                  'matrix measured off the piece. They are not re-touched '
                  'during the sweep. Serpentine: plain row-by-row sweep.')
        _Tooltip(pth_lbl, _phint)
        _Tooltip(pth_info, _phint)
        pbtns = tk.Frame(row, bg=PANEL); pbtns.pack(fill='x', pady=(4, 2))
        self._matrix_path_btns: dict[str, tk.Button] = {}
        for key, txt in (('CORNERS', 'Corners first'),
                         ('SERPENTINE', 'Serpentine')):
            b = tk.Button(pbtns, text=txt,
                          command=lambda k=key: self._on_matrix_path(k),
                          bg=BTN_NEUTRAL, fg=TEXT, font=FONT_LBL,
                          activebackground=PRIMARY_HV,
                          activeforeground='white',
                          relief='flat', bd=0, padx=14, pady=6,
                          cursor='hand2')
            b.pack(side='left', fill='x', expand=True,
                   padx=(0 if key == 'CORNERS' else 4, 0))
            self._matrix_path_btns[key] = b

        # ── Dimensões do alvo (modo SIZE) ────────────────────────────
        self._matrix_row_width = self._param_row(
            parent, label='Target Width (X)', unit='mm',
            var=self.matrix_width_var,
            vmin=MATRIX_SIZE_MIN, vmax=MATRIX_SIZE_MAX, step=1.0, snap=0.5,
            hint='Size of the sample along world +X, measured FROM the '
                 'origin. The first and last columns land exactly on the '
                 'two edges.')
        self._matrix_row_height = self._param_row(
            parent, label='Target Height (Y)', unit='mm',
            var=self.matrix_height_var,
            vmin=MATRIX_SIZE_MIN, vmax=MATRIX_SIZE_MAX, step=1.0, snap=0.5,
            hint='Size of the sample along world +Y. In Square mode this '
                 'follows Target Width.')

        # ── Passos e contagens ───────────────────────────────────────
        # Cada alteração redesenha o preview: o usuário vê a grade que vai
        # rodar ANTES do Start, que é o ponto do configurador visual.
        self._matrix_row_step_x = self._param_row(
            parent, label='Step X', unit='mm', var=self.matrix_step_x_var,
            vmin=MATRIX_STEP_MIN, vmax=MATRIX_STEP_MAX, step=0.5, snap=0.5,
            hint='Distance between adjacent indentations along world +X, '
                 'measured from the origin.')
        self._matrix_row_step_y = self._param_row(
            parent, label='Step Y', unit='mm', var=self.matrix_step_y_var,
            vmin=MATRIX_STEP_MIN, vmax=MATRIX_STEP_MAX, step=0.5, snap=0.5,
            hint='Distance between adjacent indentations along world +Y. '
                 'In Square mode this follows Step X.')
        self._matrix_row_cols = self._param_row(
            parent, label='Columns (X)', unit='×', var=self.matrix_cols_var,
            vmin=MATRIX_N_MIN, vmax=MATRIX_N_MAX, step=1, integer=True,
            hint='Number of points along X, origin included.')
        self._matrix_row_rows = self._param_row(
            parent, label='Rows (Y)', unit='×', var=self.matrix_rows_var,
            vmin=MATRIX_N_MIN, vmax=MATRIX_N_MAX, step=1, integer=True,
            hint='Number of points along Y, origin included. '
                 'In Square mode this follows Columns.')
        self._matrix_row_safe_z = self._param_row(
            parent, label='Safe Z (transit height)', unit='mm',
            var=self.matrix_safe_z_var,
            vmin=MATRIX_SAFE_Z_MM_MIN, vmax=MATRIX_SAFE_Z_MM_MAX, step=1.0,
            hint='Height ABOVE THE ORIGIN at which the probe travels between '
                 'points. It must clear the tallest feature of the sample — '
                 'the transit is blind, so anything taller than this gets '
                 'dragged over. It is also the descent travel at each point.')
        self._param_row(
            parent, label='Transit Speed (XY in air)', unit='mm/s',
            var=self.matrix_transit_var,
            vmin=MATRIX_TRANSIT_MMS_MIN, vmax=MATRIX_TRANSIT_MMS_MAX,
            step=1.0,
            hint='Speed of the in-air XY moves at Safe Z. It does not affect '
                 'the descent or the force regulation — those keep using '
                 'Descent Speed and the quasi-static micro-steps.')

        # Redesenha a cada mudança de qualquer parâmetro da grade.
        for var in (self.matrix_step_x_var, self.matrix_step_y_var,
                    self.matrix_width_var, self.matrix_height_var,
                    self.matrix_cols_var, self.matrix_rows_var,
                    self.matrix_safe_z_var, self.force_sp_var):
            var.trace_add('write', self._on_matrix_param_change)

        # ── Preview ──────────────────────────────────────────────────
        prev_head = tk.Frame(parent, bg=PANEL)
        prev_head.pack(fill='x', pady=(10, 2))
        tk.Label(prev_head, text='Indentation Preview (relative to origin)',
                 font=FONT_LBL, bg=PANEL, fg=TEXT, anchor='w').pack(side='left')
        # 240 px: com o contorno do alvo desenhado em proporção real, um
        # objeto alto (ex.: 100×150) precisa de folga vertical para as cotas
        # não encostarem na borda.
        self._matrix_canvas = tk.Canvas(
            parent, height=240, bg=BG, highlightthickness=1,
            highlightbackground=BORDER, bd=0)
        self._matrix_canvas.pack(fill='x', pady=(2, 4))
        # O Canvas nasce com largura 1; só depois do <Configure> ele tem
        # geometria real.
        self._matrix_canvas.bind(
            '<Configure>', lambda _e: self._redraw_matrix_preview())
        self._matrix_info_lbl = tk.Label(
            parent, text='', font=FONT_SMALL, bg=PANEL, fg=TEXT_MUTED,
            anchor='w', justify='left')
        self._matrix_info_lbl.pack(fill='x')

        tk.Label(parent,
                 text=('Jog the probe above the FIRST point before Start — '
                       'the first contact defines the origin (0,0).'),
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
                 anchor='w', justify='left', wraplength=340).pack(
                     fill='x', pady=(2, 0))

        # Destaque inicial do toggle de dimensionamento. _on_matrix_sizing
        # sai cedo quando o modo pedido já é o corrente, então não serve
        # para pintar o estado inicial.
        for _k, _b in self._matrix_sizing_btns.items():
            _sel = _k == self.matrix_sizing_var.get()
            _b.config(bg=PRIMARY if _sel else BTN_NEUTRAL,
                      fg='white' if _sel else TEXT)
        self._on_matrix_path(self.matrix_path_var.get())
        self._on_matrix_shape(self.matrix_shape_var.get())
    def _matrix_mirror_x_onto_y(self) -> None:
        """Em SQUARE, copia os valores do eixo X para o Y — inclusive nos
        campos ocultos, para que a grade gerada seja quadrada mesmo quando
        as variáveis de Y guardam valores antigos de um RECT anterior."""
        if self.matrix_shape_var.get() != 'SQUARE' \
                or getattr(self, '_suppressing', False):
            return
        self._suppressing = True
        try:
            self.matrix_step_y_var.set(self.matrix_step_x_var.get())
            self.matrix_height_var.set(self.matrix_width_var.get())
            self.matrix_rows_var.set(self.matrix_cols_var.get())
        except tk.TclError:
            pass
        finally:
            self._suppressing = False
    def _matrix_apply_row_visibility(self) -> None:
        """Mostra só as linhas que mandam na grade atual."""
        square  = self.matrix_shape_var.get() == 'SQUARE'
        by_size = self.matrix_sizing_var.get() == 'SIZE'
        cols   = getattr(self, '_matrix_row_cols', None)
        safe_z = getattr(self, '_matrix_row_safe_z', None)
        rows_w = getattr(self, '_matrix_row_rows', None)

        # Linhas do bloco de dimensionamento, na ordem em que devem aparecer.
        sizing_rows = (
            (getattr(self, '_matrix_row_width', None),  by_size),
            (getattr(self, '_matrix_row_height', None), by_size and not square),
            (getattr(self, '_matrix_row_step_x', None), not by_size),
            (getattr(self, '_matrix_row_step_y', None),
             (not by_size) and not square),
        )
        for widget, _ in sizing_rows:
            if widget is not None:
                widget.pack_forget()
        for widget, visible in sizing_rows:
            if widget is None or not visible:
                continue
            # Cada pack(before=cols) insere logo ANTES de Columns, então
            # chamar na ordem da tupla preserva a ordem na tela.
            if cols is not None:
                widget.pack(fill='x', pady=(5, 3), before=cols)
            else:
                widget.pack(fill='x', pady=(5, 3))

        if rows_w is not None:
            if square:
                rows_w.pack_forget()
            elif safe_z is not None:
                rows_w.pack(fill='x', pady=(5, 3), before=safe_z)
            else:
                rows_w.pack(fill='x', pady=(5, 3))
    def _on_matrix_shape(self, shape: str) -> None:
        """Aplica o formato: destaca o botão, reordena as linhas visíveis e,
        em SQUARE, espelha o eixo X no Y."""
        if shape not in MATRIX_SHAPES:
            shape = 'SQUARE'
        self.matrix_shape_var.set(shape)
        for k, b in self._matrix_shape_btns.items():
            if k == shape:
                b.config(bg=PRIMARY, fg='white')
            else:
                b.config(bg=BTN_NEUTRAL, fg=TEXT)
        self._matrix_apply_row_visibility()
        self._matrix_mirror_x_onto_y()
        self._redraw_matrix_preview()
    def _on_matrix_sizing(self, mode: str) -> None:
        """Alterna entre dimensionar por passo e pelas dimensões do alvo."""
        if mode not in MATRIX_SIZING_MODES:
            mode = 'STEP'
        if mode == self.matrix_sizing_var.get():
            return
        geom = dict(getattr(self, '_matrix_geom', {}) or {})
        self.matrix_sizing_var.set(mode)
        for k, b in self._matrix_sizing_btns.items():
            if k == mode:
                b.config(bg=PRIMARY, fg='white')
            else:
                b.config(bg=BTN_NEUTRAL, fg=TEXT)
        # Só converte com uma geometria válida em mãos: sem ela (grade em
        # erro), preservar os valores digitados é menos surpreendente do que
        # sobrescrevê-los com zeros.
        self._suppressing = True
        try:
            if mode == 'SIZE' and geom.get('span_x') is not None:
                self.matrix_width_var.set(
                    round(max(MATRIX_SIZE_MIN,
                              min(MATRIX_SIZE_MAX, geom['span_x'])), 2))
                self.matrix_height_var.set(
                    round(max(MATRIX_SIZE_MIN,
                              min(MATRIX_SIZE_MAX, geom['span_y'])), 2))
            elif mode == 'STEP' and geom.get('step_x') is not None:
                self.matrix_step_x_var.set(
                    round(max(MATRIX_STEP_MIN,
                              min(MATRIX_STEP_MAX, geom['step_x'])), 2))
                self.matrix_step_y_var.set(
                    round(max(MATRIX_STEP_MIN,
                              min(MATRIX_STEP_MAX, geom['step_y'])), 2))
        except tk.TclError:
            pass
        finally:
            self._suppressing = False
        self._matrix_apply_row_visibility()
        self._matrix_mirror_x_onto_y()
        self._redraw_matrix_preview()
    def _on_matrix_path(self, path: str) -> None:
        """Alterna a ordem de visita. Só reordena — a geometria da grade (e
        portanto o número de identações) não muda."""
        if path not in MATRIX_PATH_ORDERS:
            path = 'CORNERS'
        self.matrix_path_var.set(path)
        for k, b in self._matrix_path_btns.items():
            if k == path:
                b.config(bg=PRIMARY, fg='white')
            else:
                b.config(bg=BTN_NEUTRAL, fg=TEXT)
        self._redraw_matrix_preview()
    def _on_matrix_param_change(self, *_) -> None:
        """Trace dos parâmetros da grade: em SQUARE mantém Y == X e
        redesenha. Debounce não é necessário — o preview é um Canvas com
        algumas centenas de itens, redesenhar é barato."""
        if getattr(self, '_matrix_canvas', None) is None:
            return
        self._matrix_mirror_x_onto_y()
        self._redraw_matrix_preview()
    def _matrix_grid_nodes(self) -> tuple[list[tuple[float, float]], str]:
        """Nós da grade em mm relativos à origem, na ORDEM DE VISITA."""
        square = self.matrix_shape_var.get() == 'SQUARE'
        by_size = self.matrix_sizing_var.get() == 'SIZE'
        try:
            n_cols = int(self.matrix_cols_var.get())
            n_rows = int(self.matrix_rows_var.get())
            if by_size:
                width  = float(self.matrix_width_var.get())
                height = float(self.matrix_height_var.get())
            else:
                step_x = float(self.matrix_step_x_var.get())
                step_y = float(self.matrix_step_y_var.get())
        except (tk.TclError, ValueError):
            return [], 'invalid grid values'
        n_cols = max(MATRIX_N_MIN, min(MATRIX_N_MAX, n_cols))
        n_rows = max(MATRIX_N_MIN, min(MATRIX_N_MAX, n_rows))
        if square:
            n_rows = n_cols

        if by_size:
            if square:
                height = width
            width  = max(MATRIX_SIZE_MIN, min(MATRIX_SIZE_MAX, width))
            height = max(MATRIX_SIZE_MIN, min(MATRIX_SIZE_MAX, height))
            # step = span/(n−1): com n pontos incluindo as DUAS bordas, há
            # n−1 intervalos.
            step_x = width  / (n_cols - 1) if n_cols > 1 else 0.0
            step_y = height / (n_rows - 1) if n_rows > 1 else 0.0
            # Aqui NÃO se clampeia o passo: silenciar isso entregaria uma
            # grade de tamanho diferente do que o usuário declarou.
            too_fine = [
                f'{ax} step {s:.2f} mm'
                for ax, s, n in (('X', step_x, n_cols), ('Y', step_y, n_rows))
                if n > 1 and s < MATRIX_STEP_MIN]
            if too_fine:
                return [], (f'{" and ".join(too_fine)} below the '
                            f'{MATRIX_STEP_MIN:.1f} mm floor — enlarge the '
                            'target or reduce Columns/Rows')
        else:
            if square:
                step_y = step_x
            step_x = max(MATRIX_STEP_MIN, min(MATRIX_STEP_MAX, step_x))
            step_y = max(MATRIX_STEP_MIN, min(MATRIX_STEP_MAX, step_y))

        total = n_cols * n_rows
        if total < 2:
            return [], ('a 1x1 grid has no points beyond the origin — '
                        'increase Columns/Rows')
        if total > MATRIX_MAX_POINTS:
            return [], (f'{total} points exceeds the {MATRIX_MAX_POINTS}-point '
                        'cap — reduce Columns/Rows')
        span_x = step_x * (n_cols - 1)
        span_y = step_y * (n_rows - 1)
        if max(span_x, span_y) > MATRIX_SPAN_MAX_MM:
            return [], (f'grid spans {max(span_x, span_y):.0f} mm, over the '
                        f'{MATRIX_SPAN_MAX_MM:.0f} mm envelope')

        # Geometria efetiva para o rodapé do preview.
        self._matrix_geom = {'step_x': step_x, 'step_y': step_y,
                             'span_x': span_x, 'span_y': span_y,
                             'by_size': by_size,
                             'width': width if by_size else None,
                             'height': height if by_size else None,
                             'n_cols': n_cols, 'n_rows': n_rows}

        order = self._matrix_index_order(n_cols, n_rows,
                                        self.matrix_path_var.get())
        nodes = [(ix * step_x, iy * step_y) for ix, iy in order]
        return nodes, ''
    @staticmethod
    def _matrix_serpentine(ix0: int, ix1: int, iy0: int, iy1: int
                           ) -> list[tuple[int, int]]:
        """Serpentina (boustrophedon) no retângulo de índices [ix0..ix1] ×
        [iy0..iy1]: a 1ª linha vai no +X, a 2ª volta no −X, e assim por
        diante — o trânsito entre pontos consecutivos é sempre um passo
        curto, em vez de um retorno ao começo da linha. Retângulo vazio
        (ix0 > ix1 ou iy0 > iy1) devolve lista vazia."""
        out: list[tuple[int, int]] = []
        for k, iy in enumerate(range(iy0, iy1 + 1)):
            cols = (range(ix0, ix1 + 1) if k % 2 == 0
                    else range(ix1, ix0 - 1, -1))
            out.extend((ix, iy) for ix in cols)
        return out
    @staticmethod
    def _matrix_corners(n_cols: int, n_rows: int) -> list[tuple[int, int]]:
        """Os 4 EXTREMOS da grade, na volta mais curta a partir da origem:
        (0,0) → (max,0) → (max,max) → (0,max).
        """
        xs = sorted({0, n_cols - 1})
        ys = sorted({0, n_rows - 1})
        if len(xs) == 1 and len(ys) == 1:
            return [(0, 0)]
        if len(xs) == 1:
            return [(0, ys[0]), (0, ys[1])]
        if len(ys) == 1:
            return [(xs[0], 0), (xs[1], 0)]
        return [(xs[0], ys[0]), (xs[1], ys[0]),
                (xs[1], ys[1]), (xs[0], ys[1])]
    @classmethod
    def _matrix_index_order(cls, n_cols: int, n_rows: int, path: str
                            ) -> list[tuple[int, int]]:
        """Ordem de visita em ÍNDICES (ix, iy)."""
        full = cls._matrix_serpentine(0, n_cols - 1, 0, n_rows - 1)
        if path != 'CORNERS':
            return full
        corners = cls._matrix_corners(n_cols, n_rows)
        seen = set(corners)
        return corners + [p for p in full if p not in seen]
    def _matrix_waypoints(self) -> tuple[list[tuple[float, float]], str]:
        """Waypoints ENVIADOS ao explorer (mm, relativos à origem)."""
        nodes, err = self._matrix_grid_nodes()
        if err:
            return [], err
        return nodes[1:], ''
    def _align_ring_nodes(self) -> list[tuple[float, float]]:
        """Os 4 toques da calibração do ângulo de ataque (mm, relativos à
        origem), ou [] quando ela está desligada ou a grade não os sustenta.

        NÃO são enviados ao explorer: ele deriva o mesmo anel da mesma grade
        (tactile_explorer._align_offsets), pela MESMA função. Aqui isso só é
        desenhado — a sonda vai encostar nesses quatro pontos, e enquadrar o
        preview sem eles esconderia metade do que o robô faz no run.
        """
        var = getattr(self, 'align_on_var', None)     # criada após este bloco
        if var is None or not bool(var.get()):
            return []
        nodes, err = self._matrix_grid_nodes()
        if err or not nodes:
            return []
        try:
            ring = probe_ring_from_grid(
                [(x * 1e-3, y * 1e-3) for x, y in nodes],
                min_half_extent_m=PROBE_ALIGN_RADIUS_MM_MIN * 1e-3)
        except ValueError:
            return []                                 # o explorer cai no raio
        return [(float(p[0]) * 1e3, float(p[1]) * 1e3) for p in ring]
    def _redraw_matrix_preview(self) -> None:
        """Desenha a grade no Canvas: origem, ordem de visita e ponto ativo.

        Escala automática para caber no widget, com Y para CIMA (convenção
        do mundo, não a do Canvas, cujo eixo y cresce para baixo)."""
        cv = getattr(self, '_matrix_canvas', None)
        if cv is None:
            return
        try:
            cv.delete('all')
        except tk.TclError:
            return
        w = max(1, int(cv.winfo_width()))
        h = max(1, int(cv.winfo_height()))
        if w <= 2 or h <= 2:
            return   # ainda não mapeado; o <Configure> chama de novo

        nodes, err = self._matrix_grid_nodes()
        lbl = getattr(self, '_matrix_info_lbl', None)
        if err or not nodes:
            cv.create_text(w // 2, h // 2, text=err or 'no points',
                           fill=DANGER, font=FONT_SMALL)
            if lbl is not None:
                lbl.config(text=err, fg=DANGER)
            return

        xs = [p[0] for p in nodes]
        ys = [p[1] for p in nodes]
        _g0 = getattr(self, '_matrix_geom', None) or {}
        obj_w, obj_h = _g0.get('width'), _g0.get('height')

        # Enquadramento: com o alvo declarado, quem manda no desenho é o
        # OBJETO (0,0)–(W,H), não a nuvem de pontos.
        if obj_w and obj_h:
            ext_x0, ext_y0 = 0.0, 0.0
            ext_x1, ext_y1 = float(obj_w), float(obj_h)
        else:
            ext_x0, ext_y0 = min(xs), min(ys)
            ext_x1, ext_y1 = max(xs), max(ys)
        span_x = max(ext_x1 - ext_x0, 1e-6)
        span_y = max(ext_y1 - ext_y0, 1e-6)

        # O anel da calibração é TOCADO no run e mora fora da grade, então
        # entra no enquadramento — mas não no span do rodapé, que é a cota
        # do que se vai medir.
        ring = self._align_ring_nodes()
        vx0, vy0, vx1, vy1 = ext_x0, ext_y0, ext_x1, ext_y1
        if ring:
            vx0 = min(vx0, min(p[0] for p in ring))
            vy0 = min(vy0, min(p[1] for p in ring))
            vx1 = max(vx1, max(p[0] for p in ring))
            vy1 = max(vy1, max(p[1] for p in ring))
        view_x = max(vx1 - vx0, 1e-6)
        view_y = max(vy1 - vy0, 1e-6)
        # Folga maior com o alvo desenhado: as cotas moram FORA do contorno.
        pad = 38 if (obj_w and obj_h) else 26
        scale = min((w - 2 * pad) / view_x, (h - 2 * pad) / view_y)
        # Grades degeneradas (1 linha) dariam escala enorme; limita para o
        # desenho não virar dois pontos nas bordas opostas.
        if not (obj_w and obj_h):
            scale = min(scale, (min(w, h) - 2 * pad) / 2.0)
        cx0 = (w - view_x * scale) / 2.0
        cy0 = (h + view_y * scale) / 2.0

        def _px(mx: float, my: float) -> tuple[float, float]:
            return cx0 + (mx - vx0) * scale, cy0 - (my - vy0) * scale

        # Contorno do alvo + cotas. Vem primeiro para ficar ATRÁS dos pontos.
        if obj_w and obj_h:
            ox0, oy0 = _px(0.0, 0.0)
            ox1, oy1 = _px(float(obj_w), float(obj_h))
            cv.create_rectangle(ox0, oy0, ox1, oy1,
                                outline=TEXT_DIM, width=1, dash=(4, 3))
            # Cotas FORA do contorno: a largura embaixo, a altura à esquerda
            # (rotacionada).
            cv.create_text((ox0 + ox1) / 2.0, oy0 + 18,
                           text=f'{float(obj_w):.1f} mm',
                           fill=TEXT_DIM, font=FONT_SMALL)
            cv.create_text(ox0 - 22, (oy0 + oy1) / 2.0,
                           text=f'{float(obj_h):.1f} mm', angle=90,
                           fill=TEXT_DIM, font=FONT_SMALL)

        # Anel da calibração do ângulo de ataque. Antes dos pontos, como o
        # contorno do alvo: é fundo, não waypoint. Cruz — nenhum outro
        # símbolo do preview usa —, porque esses toques NÃO são identações
        # da grade e não têm número de ordem.
        if ring:
            rp = [_px(*p) for p in ring]
            cv.create_polygon([c for p in rp for c in p], fill='',
                              outline=TEXT_DIM, dash=(2, 3), width=1)
            for rx, ry in rp:
                cv.create_line(rx - 5, ry - 5, rx + 5, ry + 5, fill=WARN)
                cv.create_line(rx - 5, ry + 5, rx + 5, ry - 5, fill=WARN)
            cv.create_text(rp[0][0] + 8, rp[0][1] + 9, text='align',
                           anchor='w', fill=WARN, font=FONT_SMALL)

        # Caminho de visita — mostra a serpentina.
        path = []
        for p in nodes:
            path.extend(_px(*p))
        if len(path) >= 4:
            cv.create_line(*path, fill=BORDER, width=1)

        # Números dos extremos vão para FORA da figura, afastando-se do
        # centro: colados no ponto eles brigavam com as setas dos eixos e com
        # a própria fileira de pontos da borda.
        _mid_x = (min(xs) + max(xs)) / 2.0
        _mid_y = (min(ys) + max(ys)) / 2.0
        _mid_px, _mid_py = _px(_mid_x, _mid_y)

        def _corner_label_xy(px: float, py: float) -> tuple[float, float]:
            dx = 13.0 if px >= _mid_px else -13.0
            dy = 14.0 if py >= _mid_py else -14.0
            return px + dx, py + dy

        live = int(getattr(self, '_matrix_live_index', 0) or 0)
        # Quantos nós iniciais são a conferência de registro (0 em
        # serpentina).
        _g = getattr(self, '_matrix_geom', None) or {}
        n_check = (len(self._matrix_corners(_g.get('n_cols', 1),
                                            _g.get('n_rows', 1)))
                   if self.matrix_path_var.get() == 'CORNERS' else 0)
        for i, (mx, my) in enumerate(nodes):
            px, py = _px(mx, my)
            if i == 0:
                # Origem: alvo em cruz, para não ser confundida com um
                # waypoint comum — ela é medida na descida de descoberta.
                cv.create_oval(px - 6, py - 6, px + 6, py + 6,
                               outline=PRIMARY, width=2)
                cv.create_line(px - 9, py, px + 9, py, fill=PRIMARY)
                cv.create_line(px, py - 9, px, py + 9, fill=PRIMARY)
                if n_check:
                    # Em CORNERS a origem é o extremo nº 1 — numerá-la deixa
                    # a ordem da conferência legível de ponta a ponta.
                    lx, ly = _corner_label_xy(px, py)
                    cv.create_text(lx, ly, text='1', fill=WARN,
                                   font=FONT_SMALL)
                if not (obj_w and obj_h):
                    # Com o contorno do alvo desenhado a palavra colide com a
                    # cota da largura; ali a cruz azul já identifica a origem
                    # sozinha (e o rodapé a nomeia).
                    cv.create_text(px + 12, py + 10, text='origin',
                                   anchor='w', fill=PRIMARY, font=FONT_SMALL)
                continue
            # nodes[i] com i>=1 é o waypoint i (1-based) enviado ao explorer.
            active = live == i
            if i < n_check:
                # Extremos da conferência de registro: quadrado, para não
                # se confundirem com os pontos da varredura.
                cv.create_rectangle(px - 5, py - 5, px + 5, py + 5,
                                    fill=OK if active else PANEL,
                                    outline=OK if active else WARN,
                                    width=2)
                lx, ly = _corner_label_xy(px, py)
                cv.create_text(lx, ly, text=str(i + 1), fill=WARN,
                               font=FONT_SMALL)
                continue
            cv.create_oval(px - 4, py - 4, px + 4, py + 4,
                           fill=OK if active else PANEL,
                           outline=OK if active else TEXT_MUTED,
                           width=2 if active else 1)

        # Eixos: seta +X/+Y ancorada na origem, para orientar o usuário.
        ox, oy = _px(0.0, 0.0)
        cv.create_line(ox, oy, ox + 22, oy, fill=TEXT_DIM, arrow='last')
        cv.create_text(ox + 26, oy, text='+X', anchor='w',
                       fill=TEXT_DIM, font=FONT_SMALL)
        cv.create_line(ox, oy, ox, oy - 22, fill=TEXT_DIM, arrow='last')
        cv.create_text(ox, oy - 30, text='+Y', fill=TEXT_DIM, font=FONT_SMALL)

        if lbl is not None:
            try:
                force = float(self.force_sp_var.get())
            except (tk.TclError, ValueError):
                force = FORCE_SP_DEFAULT
            try:
                safe_z = float(self.matrix_safe_z_var.get())
            except (tk.TclError, ValueError):
                safe_z = MATRIX_SAFE_Z_MM_DEFAULT
            # Em SIZE o passo é a grandeza derivada, e é ela que o usuário
            # precisa conferir (resolução espacial do mapa); em STEP quem
            # deriva é o span. Mostra sempre o número que NÃO foi digitado.
            geom = getattr(self, '_matrix_geom', None) or {}
            if geom.get('by_size'):
                derived = (f'target {span_x:.1f} × {span_y:.1f} mm · '
                           f'step {geom.get("step_x", 0.0):.2f} × '
                           f'{geom.get("step_y", 0.0):.2f} mm')
            else:
                derived = f'span {span_x:.1f} × {span_y:.1f} mm'
            lbl.config(
                text=(f'{len(nodes)} indentations '
                      f'(origin + {len(nodes) - 1} waypoints) · '
                      f'{derived} · '
                      f'F = {force:.2f} N at every point · '
                      f'Safe Z +{safe_z:.1f} mm'),
                fg=TEXT_MUTED)
