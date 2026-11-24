"""Tema do painel rehab_gui: tokens (cor, fonte, espaçamento) e widgets base.

Mesma família visual do painel do touch_pack (palpation_gui / ui_helpers):
fundo cinza-claro, cartões brancos com barra de destaque azul, botões planos,
abas com a selecionada em branco e texto azul, barra de estado no rodapé.
Diferenças deliberadas, por acessibilidade (WCAG 2.1 AA):
- cabeçalho ardósia em vez de azul: o vermelho do PARAR sobre o azul do
  touch_pack tem contraste 1,4:1; sobre ardósia, 3,0:1 (e ganha anel branco);
- texto de dica em MUTED (6,9:1) em vez do TEXT_DIM (2,6:1, ilegível);
- verde/âmbar com texto branco escurecidos (5,0:1 em vez de 3,3 e 3,2).

Fontes: só named fonts do Tk. O touch_pack registrou que fontes em tupla
(família, tamanho) criadas por widget corrompem o heap do Xft nesta máquina;
aqui as duas fontes grandes são criadas UMA vez, com nome, em `init()`.
"""
from __future__ import annotations

import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk

# ── Cores ───────────────────────────────────────────────────────────────────
BG = '#f1f5f9'           # fundo da janela
PANEL = '#ffffff'        # cartões
SUNKEN = '#f8fafc'       # áreas de desenho (quadro do jogo, gráficos)
HEADER = '#1e293b'       # cabeçalho (ardósia)
HEADER_FG = '#ffffff'
HEADER_MUTED = '#cbd5e1'
TEXT = '#0f172a'
MUTED = '#475569'        # texto secundário e dicas (≥ 4,5:1 em BG e PANEL)
DIM = '#94a3b8'          # só decoração (nunca texto essencial)
BORDER = '#cbd5e1'
GRID = '#e2e8f0'
BTN = '#e2e8f0'
BTN_HV = '#cbd5e1'
PRIMARY = '#2563eb'
PRIMARY_HV = '#1d4ed8'
OK = '#15803d'
WARN = '#b45309'
DANGER = '#dc2626'
DANGER_HV = '#b91c1c'

# LEDs no cabeçalho (sobre ardósia, ≥ 3:1)
LED_ON, LED_OFF = '#22c55e', '#ef4444'

# Estado da admitância: (fundo do selo, texto do selo, ícone, fundo da faixa,
# texto da faixa). Ícone + palavra: a cor nunca carrega o estado sozinha.
STATE_STYLE = {
    'SEM DADOS': ('#475569', '#ffffff', '○', '#e2e8f0', '#1e293b'),
    'PARADO': ('#475569', '#ffffff', '■', '#e2e8f0', '#1e293b'),
    'ATIVO': (OK, '#ffffff', '●', '#dcfce7', '#14532d'),
    'SEM_FORCA': (WARN, '#ffffff', '▲', '#fef3c7', '#78350f'),
    'LIMITE': (WARN, '#ffffff', '▲', '#fef3c7', '#78350f'),
    'ABORTADO': (DANGER, '#ffffff', '✖', DANGER, '#ffffff'),
    'POSICIONANDO': ('#1d4ed8', '#ffffff', '↻', '#dbeafe', '#1e3a8a'),
}

# Séries dos gráficos (paleta categórica validada com o validador do skill
# dataviz: slots 1–3, todos os pares, ΔE CVD ≥ 9,2). Módulo = tinta do texto.
SERIES_MAG = TEXT
SERIES_X, SERIES_Y, SERIES_Z = '#2a78d6', '#eb6834', '#1baf7a'

# Jogo
TARGET_FILL, TARGET_LINE = '#fef3c7', '#b45309'
HIT_FILL = '#dcfce7'
CURSOR = PRIMARY
TRAIL = '#93c5fd'
TUNNEL_FILL, TUNNEL_EDGE, TUNNEL_DONE = '#dbeafe', '#60a5fa', '#bbf7d0'
CELL = '#bfdbfe'

# ── Fontes (named fonts) ────────────────────────────────────────────────────
FONT = 'TkDefaultFont'            # 10 pt
FONT_SMALL = 'TkSmallCaptionFont'  # 9 pt
FONT_TITLE = 'TkCaptionFont'      # 12 pt negrito
FONT_MONO = 'TkFixedFont'
FONT_NUM = 'RehabNumber'          # leituras numéricas grandes (criada em init)
FONT_DISPLAY = 'RehabDisplay'     # PARAR e estado (criada em init)

# ── Espaçamento (px) ────────────────────────────────────────────────────────
PAD_S, PAD, PAD_L = 4, 8, 14


def init(root: tk.Tk):
    """Cria as fontes com nome e configura os estilos ttk. Chamar uma vez."""
    base = tkfont.nametofont('TkDefaultFont').actual()
    # Guardar a referência: o tkinter apaga a fonte nomeada quando o objeto
    # Font é coletado (e os widgets voltam silenciosamente à fonte padrão).
    root._rehab_fonts = [
        tkfont.Font(root=root, name=name, family=base['family'], size=size,
                    weight='bold')
        for name, size in ((FONT_NUM, 15), (FONT_DISPLAY, 18))
        if name not in tkfont.names(root)]
    s = ttk.Style(root)
    s.theme_use('clam')
    s.configure('.', background=PANEL, foreground=TEXT, font=FONT,
                focuscolor=PRIMARY)
    s.configure('TFrame', background=BG)
    s.configure('Panel.TFrame', background=PANEL)
    s.configure('TLabel', background=PANEL, foreground=TEXT)
    s.configure('TCheckbutton', background=PANEL, foreground=TEXT)
    s.map('TCheckbutton', background=[('active', PANEL)])
    s.configure('R.TNotebook', background=BG, borderwidth=0, tabmargins=(0, 4, 0, 0))
    s.configure('R.TNotebook.Tab', padding=(16, 7), background=BTN,
                foreground=TEXT, borderwidth=0, font=FONT)
    s.map('R.TNotebook.Tab', background=[('selected', PANEL), ('active', BTN_HV)],
          foreground=[('selected', PRIMARY)])
    # Foco por teclado: borda de 2 px em tinta escura (visível em qualquer
    # variante; WCAG 2.4.7). Sem foco a borda tem a cor do próprio botão.
    for name, bg, hv, fg, pad in (('TButton', BTN, BTN_HV, TEXT, (12, 6)),
                                  ('Primary.TButton', PRIMARY, PRIMARY_HV, '#ffffff', (12, 6)),
                                  ('Seg.TButton', BTN, BTN_HV, TEXT, (10, 6)),
                                  ('SegOn.TButton', PRIMARY, PRIMARY_HV, '#ffffff', (10, 6))):
        s.configure(name, background=bg, foreground=fg, borderwidth=2,
                    bordercolor=bg, relief='solid', padding=pad,
                    lightcolor=bg, darkcolor=bg, focuscolor=bg)
        s.map(name, background=[('disabled', GRID), ('pressed', hv), ('active', hv)],
              foreground=[('disabled', DIM)],
              bordercolor=[('focus', TEXT)], lightcolor=[('focus', TEXT)],
              darkcolor=[('focus', TEXT)])
    s.configure('Horizontal.TScale', background=PANEL, troughcolor=GRID,
                bordercolor=BORDER, lightcolor=PANEL, darkcolor=BORDER)
    s.map('Horizontal.TScale', background=[('focus', PRIMARY), ('active', BTN_HV)],
          bordercolor=[('focus', TEXT)])
    s.configure('TEntry', fieldbackground=PANEL, bordercolor=BORDER,
                lightcolor=BORDER)
    s.map('TEntry', bordercolor=[('focus', PRIMARY)], lightcolor=[('focus', PRIMARY)])
    s.configure('TSpinbox', fieldbackground=PANEL, bordercolor=BORDER,
                arrowsize=12)
    s.map('TSpinbox', bordercolor=[('focus', PRIMARY)])
    s.configure('TCombobox', fieldbackground=PANEL, background=BTN,
                bordercolor=BORDER, arrowsize=14)
    s.map('TCombobox', fieldbackground=[('readonly', PANEL)],
          bordercolor=[('focus', PRIMARY)])
    s.configure('Stab.Horizontal.TProgressbar', troughcolor=GRID, background=OK,
                bordercolor=GRID, lightcolor=OK, darkcolor=OK, thickness=10)
    s.configure('Task.Horizontal.TProgressbar', troughcolor=GRID, background=PRIMARY,
                bordercolor=GRID, lightcolor=PRIMARY, darkcolor=PRIMARY, thickness=10)
    root.configure(bg=BG)
    root.option_add('*TCombobox*Listbox.font', FONT)


def card(parent, titulo, **pack):
    """Cartão no padrão do touch_pack: barra azul à esquerda + título em
    negrito + filete. Devolve o corpo (Frame branco) para os filhos."""
    outer = tk.Frame(parent, bg=PANEL, highlightthickness=1,
                     highlightbackground=BORDER, highlightcolor=BORDER)
    outer.pack(**({'fill': 'x', 'padx': PAD_S, 'pady': PAD_S} | pack))
    head = tk.Frame(outer, bg=PANEL)
    head.pack(fill='x', padx=PAD_L, pady=(PAD + 2, PAD_S))
    tk.Frame(head, bg=PRIMARY, width=4).pack(side='left', fill='y', padx=(0, PAD))
    tk.Label(head, text=titulo, bg=PANEL, fg=TEXT, font=FONT_TITLE).pack(side='left')
    tk.Frame(outer, bg=GRID, height=1).pack(fill='x', padx=PAD_L)
    body = tk.Frame(outer, bg=PANEL)
    body.pack(fill='both', expand=True, padx=PAD_L, pady=(PAD, PAD_L - 2))
    body.head = head
    return body


def label(parent, texto='', muted=False, **kw):
    kw.setdefault('font', FONT)
    return tk.Label(parent, text=texto, bg=parent.cget('bg'),
                    fg=MUTED if muted else TEXT, anchor='w', justify='left', **kw)


def readout(parent, rotulo, unidade=''):
    """Leitura numérica grande: rótulo pequeno em cima, valor + unidade.
    Devolve o Label do valor."""
    f = tk.Frame(parent, bg=parent.cget('bg'))
    tk.Label(f, text=rotulo, bg=f.cget('bg'), fg=MUTED, font=FONT_SMALL).pack(anchor='w')
    row = tk.Frame(f, bg=f.cget('bg'))
    row.pack(anchor='w')
    v = tk.Label(row, text='—', bg=f.cget('bg'), fg=TEXT, font=FONT_NUM)
    v.pack(side='left')
    if unidade:
        tk.Label(row, text=' ' + unidade, bg=f.cget('bg'), fg=MUTED,
                 font=FONT).pack(side='left', anchor='s', pady=(0, 3))
    v.frame = f
    return v
