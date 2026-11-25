"""Painel Tkinter do rehab_pack: sessão, admitância, jogo, gráficos e juntas.

`ros2 run rehab_pack rehab_gui` abre mesmo sem nenhum outro nó no ar (LEDs
vermelhos). Toda a lógica de tarefa está em tasks.py; a comunicação ROS, em
gui_bridge.py (thread própria). Esta thread só desenha e consulta filas.
Tema e widgets base: gui_theme.py; gráficos: gui_charts.py.

Atalhos (fora de campos de texto): Espaço = PARAR (parada de emergência),
M = marcador, 1–5 = abas. Tab percorre os
controles e Enter aciona o botão em foco (Espaço nunca aciona botão: é PARAR).
"""
from __future__ import annotations

import collections
import json
import math
import signal
import sys
import time
import traceback
import tkinter as tk
from tkinter import ttk

from . import constants as C
from . import gui_theme as th
from . import tasks as T
from .gui_charts import LineChart

REC_STALE_S = 1.5            # registrador calado por mais que isso: não está gravando
LIVE_S = 0.5                 # LED verde se a última mensagem tem menos que isto
POLL_MS = 33
VIEW_SPAN_M = 0.56           # largura do mundo visível no quadro do jogo
TRAIL_S = 3.0                # rastro do cursor

# (chave do parâmetro, rótulo, mín, máx, passo, unidade)
PARAMS = (
    ('mass_kg', 'Massa virtual M', 0.5, 10.0, 0.1, 'kg'),
    ('damping_ns_m', 'Amortecimento B', 0.0, 150.0, 1.0, 'N·s/m'),
    ('stiffness_n_m', 'Rigidez K (0 = livre)', 0.0, 200.0, 1.0, 'N/m'),
    ('deadband_n', 'Zona morta', 0.0, 3.0, 0.1, 'N'),
    ('v_max_ms', 'Velocidade máxima', 0.02, 0.15, 0.01, 'm/s'),
    ('max_excursion_m', 'Excursão máxima', 0.02, 0.30, 0.01, 'm'),
    ('abort_force_n', 'Limite de aborto', 5.0, C.FORCE_ABORT_LIMIT_N, 0.5, 'N'),
    ('abort_torque_nm', 'Limite de torque', 0.2, C.TORQUE_ABORT_LIMIT_NM, 0.1, 'N·m'),
)
PRESET_HELP = {
    'Livre': 'Movimento livre, amortecido (K = 0).',
    'Leve': 'Menos massa e amortecimento: exige pouca força.',
    'Assistido': 'Mola (K > 0) puxa para a referência: use com a assistência do túnel.',
    'Resistivo': 'Amortecimento alto: o voluntário faz mais força.',
}
TASK_LABELS = {
    'centro_fora': 'Centro-fora (8 alvos)', 'tunel_reta': 'Túnel — reta',
    'tunel_circulo': 'Túnel — círculo', 'tunel_oito': 'Túnel — oito',
    'alcance': 'Avaliação de alcance',
}
# (chave, rótulo, inicial, mín, máx, passo, tarefas em que vale)
TASK_PARAMS = (
    ('raio', 'Raio do alvo [cm]', 2.0, 1.0, 5.0, 0.5, ('centro_fora',)),
    ('alcance', 'Distância dos alvos [cm]', 10.0, 5.0, 20.0, 1.0, ('centro_fora',)),
    ('limite', 'Tempo limite por alvo [s]', 10.0, 3.0, 30.0, 1.0, ('centro_fora',)),
    ('seed', 'Semente da ordem', 1, 0, 999, 1, ('centro_fora',)),
    ('largura', 'Largura do túnel [cm]', 4.0, 2.0, 10.0, 0.5, ('tunel',)),
    ('tamanho', 'Tamanho da trajetória [cm]', 20.0, 10.0, 22.0, 1.0, ('tunel',)),
    ('duracao', 'Duração [s]', 60.0, 10.0, 300.0, 10.0, ('alcance',)),
)
BANNER = {
    'SEM DADOS': 'Admitância não está no ar.',
    'PARADO': 'Parado: Tarar e Habilitar com o mastro solto.',
    'ATIVO': 'O braço segue a mão.',
    'SEM_FORCA': 'Sem leitura da célula: braço parado.',
    'LIMITE': 'Limite de alcance: volte em direção ao centro.',
    'POSICIONANDO': 'Movendo as juntas. Espaço = PARAR.',
    'ABORTADO': 'PARADA DE SEGURANÇA: solte o mastro e clique em Reset.',
}
STATE_LABEL = {'SEM_FORCA': 'SEM FORÇA'}
# Nome da ação e nó responsável por serviço (mensagens de erro úteis).
ACTION = {C.S_TARE: ('Tara', 'rehab_force_source'),
          C.S_ENABLE: ('Habilitar/Desabilitar', 'rehab_admittance'),
          C.S_ESTOP: ('PARAR', 'rehab_admittance'),
          C.S_RESET: ('Reset', 'rehab_admittance'),
          C.S_REAL_CONNECT: ('Robô real', C.NODE_MIRROR),
          C.S_REAL_MODE: ('Modo do robô real', C.NODE_MIRROR),
          C.S_SESSION_START: ('Iniciar sessão', 'rehab_logger'),
          C.S_SESSION_STOP: ('Encerrar sessão', 'rehab_logger')}
SHORTCUTS = 'Espaço: PARAR'


class RehabGui:
    def __init__(self, root: tk.Tk, bridge):
        self.root, self.br = root, bridge
        root.title('Reabilitação — CR10 + FA7155')
        root.geometry('1180x860')
        root.minsize(1000, 800)
        th.init(root)

        self.tick = None                 # último tick (dict campo -> valor)
        self.center = None               # (x, y, z) de referência da vista
        self.task, self.task_name = None, None
        self._jog_t, self._jog_after, self._jog_raw = 0.0, None, None   # última mexida do operador; envio agendado
        self._last_estop = 0.0
        self._estop_wait = 0.0           # PARAR enviado, ainda sem resposta
        self._estop_ok = None            # resposta do último PARAR (None = ainda não veio)
        self._estop_alert = ''           # PARAR não confirmado: fica no banner
        self._enable_wait, self._enable_done = 0.0, False   # pedido do botão em curso
        self._jog_synced = False         # os 6 sliders já mostraram a pose do braço
        self._jog_js = set()             # juntas mexidas desde o último envio (vazio = as 6)
        self._jog_alvo = self._jog_alvo_ant = None   # último alvo enviado [°] e o anterior a ele
        self._task_z = 0.0
        self.origin = None
        self._dlg = None
        self._last_target_pub = 0.0
        self._params_loaded = False
        self._ref_sent = False           # a assistência moveu x_ref nesta tarefa
        self._summary_shown = None
        self._n_poll = 0
        self._drawn = set()
        self._closing = False
        self.session_open, self._session_t = False, 0.0   # aberta?; última mudança vinda de um serviço
        self.enabled = self.moving = False
        self.trail = collections.deque()
        self._co_result = {}             # alvo -> 'hit' | 'miss' (centro-fora)
        self._flash = None               # (instante, x, y, texto, cor)
        self._scale = 1000.0             # px por metro no quadro do jogo
        self._preset = 'Livre'

        self._build_header()
        self._build_statusbar()
        self.nb = ttk.Notebook(root, style='R.TNotebook')
        self.nb.pack(fill='both', expand=True, padx=th.PAD, pady=(th.PAD_S, 0))
        self._build_session()
        self._build_admittance()
        self._build_tasks()
        self._build_charts()
        self._build_jog(self._page('5  Juntas'))

        root.bind_all('<KeyPress>', self._on_key_press)
        # Espaço em botão/caixa com foco = PARAR (nunca aciona o widget); Enter aciona.
        for cls in ('TButton', 'TCheckbutton', 'TRadiobutton', 'Button'):
            root.bind_class(cls, '<KeyPress-space>', self._space_on_widget)
            root.bind_class(cls, '<Return>', lambda e: e.widget.invoke())
        root.protocol('WM_DELETE_WINDOW', self.close)
        self._poll()

    # ── helpers ─────────────────────────────────────────────────────────
    def _page(self, titulo):
        f = tk.Frame(self.nb, bg=th.BG)
        self.nb.add(f, text=titulo)
        return f

    def _btn(self, parent, texto, cmd, kind='', **kw):
        return ttk.Button(parent, text=texto, command=cmd,
                          style=f'{kind}.TButton' if kind else 'TButton', **kw)

    def _say(self, texto, ok=True):
        self.status.configure(text=f'{time.strftime("%H:%M:%S")}   {texto}',
                              fg=th.MUTED if ok else th.DANGER_HV)

    def _typing(self):
        try:
            w = self.root.focus_get()
        except KeyError:                 # widget interno do Tk (lista do Combobox): não é texto
            return False
        return w is not None and w.winfo_class() in ('Entry', 'TEntry', 'Text', 'TSpinbox', 'Spinbox',
                                                     'TCombobox')

    def _confirm(self, titulo, texto, ok='Confirmar'):
        """Confirmação SEM grab: PARAR (botão e Espaço) segue acessível, e a cancela."""
        self._confirm_cancel()
        w = self._dlg = tk.Toplevel(self.root, bg=th.PANEL)
        w.title(titulo)
        if self.root.winfo_viewable():   # transient de janela minimizada nasceria escondida
            w.transient(self.root)
        th.label(w, texto, wraplength=420).pack(padx=18, pady=14)
        row = tk.Frame(w, bg=th.PANEL)
        row.pack(pady=(0, 14))
        sim = []
        self._btn(row, ok, lambda: (sim.append(1), w.destroy()), 'Primary').pack(side='left', padx=th.PAD)
        nao = self._btn(row, 'Cancelar', w.destroy)
        nao.pack(side='left')
        nao.focus_set()
        self.root.wait_window(w)
        return bool(sim)

    def _confirm_cancel(self):
        if self._dlg is not None and self._dlg.winfo_exists():
            self._dlg.destroy()

    def _tab(self):
        return self.nb.index(self.nb.select())

    # ── cabeçalho: estado e PARAR sempre visíveis ───────────────────────
    def _build_header(self):
        H = th.HEADER
        hdr = tk.Frame(self.root, bg=H)
        hdr.pack(fill='x', side='top')
        top = tk.Frame(hdr, bg=H)
        top.pack(fill='x', padx=18, pady=(10, 10))

        # PARAR: vermelho com anel branco (contraste sobre a ardósia), à direita.
        stop = tk.Frame(top, bg=H)
        stop.pack(side='right')
        ring = tk.Frame(stop, bg='#ffffff', padx=2, pady=2)
        ring.pack()
        self.btn_estop = tk.Button(
            ring, text='■  PARAR', command=self.estop, font=th.FONT_DISPLAY,
            bg=th.DANGER, fg='#ffffff', activebackground=th.DANGER_HV,
            activeforeground='#ffffff', relief='flat', bd=0, padx=34, pady=10,
            highlightthickness=0, cursor='hand2')
        self.btn_estop.pack()
        tk.Label(stop, text='tecla Espaço', bg=H, fg=th.HEADER_MUTED,
                 font=th.FONT_SMALL).pack(pady=(3, 0))

        left = tk.Frame(top, bg=H)
        left.pack(side='left', anchor='n')
        tk.Label(left, text='Reabilitação — CR10 + FA7155', bg=H, fg=th.HEADER_FG,
                 font=th.FONT_TITLE).pack(anchor='w')
        self.lbl_sub = tk.Label(left, text='Nenhuma sessão de coleta aberta', bg=H,
                                fg=th.HEADER_MUTED, font=th.FONT_SMALL)
        self.lbl_sub.pack(anchor='w')
        leds = tk.Frame(left, bg=H)
        leds.pack(anchor='w', pady=(8, 0))
        self.leds = {}
        for key, nome in (('adm', 'Admitância'), ('ft', 'Força'),
                          ('arm', 'Braço'), ('log', 'Registrador')):
            cell = tk.Frame(leds, bg=H)
            cell.pack(side='left', padx=(0, 18))
            row = tk.Frame(cell, bg=H)
            row.pack(anchor='w')
            dot = tk.Label(row, text='○', bg=H, fg=th.LED_OFF, font=th.FONT)
            dot.pack(side='left')
            tk.Label(row, text=nome, bg=H, fg=th.HEADER_FG, font=th.FONT).pack(side='left', padx=(3, 0))
            word = tk.Label(cell, text='fora do ar', bg=H, fg=th.HEADER_MUTED,
                            font=th.FONT_SMALL)
            word.pack(anchor='w', padx=(16, 0))
            self.leds[key] = (dot, word)

        # Estado + força (leituras grandes), entre o título e o PARAR.
        mid = tk.Frame(top, bg=H)
        mid.pack(side='right', padx=(0, 28))
        fbox = tk.Frame(mid, bg=H)
        fbox.pack(side='right', padx=(24, 0))
        tk.Label(fbox, text='FORÇA |F|', bg=H, fg=th.HEADER_MUTED,
                 font=th.FONT_SMALL).pack(anchor='w')
        frow = tk.Frame(fbox, bg=H)
        frow.pack(anchor='w')
        self.lbl_fmag = tk.Label(frow, text='—', bg=H, fg=th.HEADER_FG,
                                 font=th.FONT_DISPLAY, width=5, anchor='e')
        self.lbl_fmag.pack(side='left')
        tk.Label(frow, text=' N', bg=H, fg=th.HEADER_MUTED, font=th.FONT).pack(side='left', anchor='s', pady=(0, 4))
        self.lbl_force = tk.Label(fbox, text='axial —   idade —', bg=H,
                                  fg=th.HEADER_MUTED, font=th.FONT_SMALL)
        self.lbl_force.pack(anchor='w')
        sbox = tk.Frame(mid, bg=H)
        sbox.pack(side='right')
        tk.Label(sbox, text='ESTADO DO BRAÇO', bg=H, fg=th.HEADER_MUTED,
                 font=th.FONT_SMALL).pack(anchor='w')
        self.pill = tk.Frame(sbox, bg=th.MUTED)
        self.pill.pack(anchor='w', pady=(2, 0))
        self.lbl_state_icon = tk.Label(self.pill, text='○', bg=th.MUTED, fg='#ffffff',
                                       font=th.FONT_DISPLAY, padx=6)
        self.lbl_state_icon.pack(side='left', padx=(6, 0))
        self.lbl_state = tk.Label(self.pill, text='SEM DADOS', bg=th.MUTED, fg='#ffffff',
                                  font=th.FONT_DISPLAY, padx=6, pady=2, width=10, anchor='w')
        self.lbl_state.pack(side='left', padx=(0, 6))

        # Robô real: em todas as abas, como no touch_pack.
        row = tk.Frame(hdr, bg=H)
        row.pack(fill='x', padx=18, pady=(0, 10))
        tk.Label(row, text='IP do CR10', bg=H, fg=th.HEADER_FG, font=th.FONT).pack(side='left')
        self.var_ip = tk.StringVar()     # o rehab_mirror informa: robot_ip do launch ou o robot.json
        ttk.Entry(row, textvariable=self.var_ip, width=15).pack(side='left', padx=th.PAD)
        self.btn_real = self._btn(row, 'Conectar', self.toggle_real)
        self.btn_real.pack(side='left')
        self.btn_mode = self._btn(row, 'Modo: SIM_ONLY', self.toggle_mode)
        self.btn_mode.pack(side='left', padx=th.PAD)
        self.lbl_env = tk.Label(row, text='ROBÔ REAL: DESCONHECIDO', bg=th.WARN, fg='#ffffff',
                                font=th.FONT_TITLE, padx=10, pady=3)
        self.lbl_env.pack(side='left', padx=th.PAD)

        # Faixa de estado: o que está acontecendo e o que fazer.
        self.banner = tk.Frame(self.root, bg=th.GRID)
        self.banner.pack(fill='x', side='top')
        self.lbl_banner = tk.Label(self.banner, text='', bg=th.GRID, fg=th.TEXT,
                                   font=th.FONT, anchor='w', justify='left')
        self.lbl_banner.pack(side='left', fill='x', expand=True, padx=18, pady=7)
        self.btn_banner = self._btn(self.banner, '', None)
        self._banner_action = None

    def _build_statusbar(self):
        bar = tk.Frame(self.root, bg=th.PANEL, highlightthickness=1,
                       highlightbackground=th.BORDER)
        bar.pack(fill='x', side='bottom')
        self.status = tk.Label(bar, text='Pronto.', anchor='w', bg=th.PANEL, fg=th.MUTED,
                               font=th.FONT)
        self.status.pack(side='left', fill='x', expand=True, padx=12, pady=5)
        tk.Label(bar, text=SHORTCUTS, bg=th.PANEL, fg=th.MUTED,
                 font=th.FONT_SMALL).pack(side='right', padx=12)

    # ── aba 1: sessão ───────────────────────────────────────────────────
    def _build_session(self):
        p = self._page('1  Sessão')
        p.columnconfigure(0, weight=1, uniform='c')
        p.columnconfigure(1, weight=1, uniform='c')
        p.rowconfigure(0, weight=1)
        left = tk.Frame(p, bg=th.BG)
        left.grid(row=0, column=0, sticky='nsew', padx=(th.PAD_S, 2), pady=th.PAD_S)
        right = tk.Frame(p, bg=th.BG)
        right.grid(row=0, column=1, sticky='nsew', padx=(2, th.PAD_S), pady=th.PAD_S)

        c = th.card(left, 'Preparar o braço')
        c.columnconfigure(2, weight=1)
        steps = (('1', 'Tarar sensor', self.tare, '', 'Com o mastro solto.'),
                 ('2', 'Habilitar', self.toggle_enable, 'Primary', 'Mastro solto; depois segurar.'))
        for r, (n, txt, cmd, kind, hint) in enumerate(steps):
            tk.Label(c, text=n, bg=th.GRID, fg=th.TEXT, font=th.FONT_TITLE, width=2
                     ).grid(row=r, column=0, sticky='w', pady=4)
            b = self._btn(c, txt, cmd, kind, width=14)
            b.grid(row=r, column=1, sticky='w', padx=th.PAD, pady=4)
            th.label(c, hint, muted=True, wraplength=230).grid(row=r, column=2, sticky='w')
            if n == '2':
                self.btn_enable = b
        tk.Frame(c, bg=th.GRID, height=1).grid(row=2, column=0, columnspan=3, sticky='ew', pady=th.PAD)
        for r, (txt, cmd, hint) in enumerate((
                ('Reset', self.reset, 'Destrava o ABORTADO.'),
                ('Home', self.jog_home, 'Vai à Home salva.')), start=3):
            self._btn(c, txt, cmd, width=14).grid(row=r, column=1, sticky='w', padx=th.PAD, pady=3)
            th.label(c, hint, muted=True, wraplength=230).grid(row=r, column=2, sticky='w')

        c = th.card(left, 'Conexões')
        c.columnconfigure(1, weight=1)
        self.conn = {}
        for r, (key, nome) in enumerate((('adm', 'Admitância'),
                                         ('ft', 'Força'),
                                         ('arm', 'Braço'),
                                         ('log', 'Registrador'),
                                         ('real', 'Robô real'),
                                         ('ftraw', 'Célula FA7155'))):
            th.label(c, nome).grid(row=r, column=0, sticky='w', pady=2)
            v = th.label(c, '○ fora do ar')
            v.grid(row=r, column=1, sticky='w', padx=th.PAD_L)
            self.conn[key] = v
        th.label(c, 'Fonte de força').grid(row=6, column=0, sticky='w', pady=(th.PAD, 2))
        self.lbl_source = th.label(c, '—', muted=True)
        self.lbl_source.grid(row=6, column=1, sticky='w', padx=th.PAD_L, pady=(th.PAD, 2))

        c = th.card(right, 'Sessão de coleta')
        c.columnconfigure(1, weight=1)
        th.label(c, 'ID anônimo').grid(row=0, column=0, sticky='w', pady=3)
        g = tk.Frame(c, bg=th.PANEL)
        g.grid(row=0, column=1, sticky='w', padx=th.PAD)
        self.var_id = tk.StringVar(value='S01')
        ttk.Entry(g, textvariable=self.var_id, width=12).pack(side='left')
        th.label(g, '  ex.: S01', muted=True).pack(side='left')
        th.label(c, 'Lado parético').grid(row=1, column=0, sticky='w', pady=3)
        g = tk.Frame(c, bg=th.PANEL)
        g.grid(row=1, column=1, sticky='w', padx=th.PAD)
        self.var_side = tk.StringVar(value='D')
        for val, txt in (('D', 'Direito'), ('E', 'Esquerdo')):
            ttk.Radiobutton(g, text=txt, value=val, variable=self.var_side).pack(side='left', padx=(0, 12))
        th.label(c, 'Tarefa').grid(row=2, column=0, sticky='w', pady=3)
        self.var_task_meta = tk.StringVar(value='livre')
        ttk.Combobox(c, textvariable=self.var_task_meta, values=T.TASK_NAMES, width=16,
                     state='readonly').grid(row=2, column=1, sticky='w', padx=th.PAD, pady=3)
        th.label(c, 'Notas').grid(row=3, column=0, sticky='nw', pady=3)
        self.txt_notes = tk.Text(c, width=34, height=3, wrap='word', font=th.FONT,
                                 relief='flat', highlightthickness=1,
                                 highlightbackground=th.BORDER, highlightcolor=th.PRIMARY)
        self.txt_notes.grid(row=3, column=1, sticky='ew', padx=th.PAD, pady=3)
        # Tab sai do campo de notas (não insere tabulação): sem armadilha de foco.
        self.txt_notes.bind('<Tab>', lambda e: (e.widget.tk_focusNext().focus_set(), 'break')[1])
        row = tk.Frame(c, bg=th.PANEL)
        row.grid(row=4, column=0, columnspan=2, sticky='w', pady=(th.PAD, 2))
        self.btn_start = self._btn(row, 'Iniciar sessão', self.session_start, 'Primary')
        self.btn_start.pack(side='left')
        self.btn_stop = self._btn(row, 'Encerrar sessão', lambda: self.br.call_trigger(C.S_SESSION_STOP))
        self.btn_stop.pack(side='left', padx=th.PAD)
        self.lbl_session = th.label(c, '○ Nenhuma sessão aberta.', muted=True, wraplength=460)
        self.lbl_session.grid(row=5, column=0, columnspan=2, sticky='w', pady=(2, 0))
        tk.Frame(c, bg=th.GRID, height=1).grid(row=6, column=0, columnspan=2, sticky='ew', pady=th.PAD)
        th.label(c, 'Marcador').grid(row=7, column=0, sticky='w')
        row = tk.Frame(c, bg=th.PANEL)
        row.grid(row=7, column=1, sticky='ew', padx=th.PAD)
        self.var_mark = tk.StringVar()
        ttk.Entry(row, textvariable=self.var_mark).pack(side='left', fill='x', expand=True)
        self._btn(row, 'Marcar  (M)', self.mark).pack(side='left', padx=(th.PAD, 0))

        c = th.card(right, 'Resumo da última sessão', fill='both', expand=True)
        self.txt_summary = tk.Text(c, height=10, width=48, font=th.FONT_MONO, bg=th.SUNKEN,
                                   fg=th.TEXT, relief='flat', padx=10, pady=8,
                                   highlightthickness=0, wrap='word')
        self.txt_summary.tag_configure('k', foreground=th.MUTED)
        self.txt_summary.pack(fill='both', expand=True)
        self.txt_summary.insert('1.0', 'Nenhuma sessão encerrada ainda.\n'
                                       'O resumo aparece aqui quando o registrador fecha a sessão.')
        self.txt_summary.tag_add('k', '1.0', 'end')
        self.txt_summary.configure(state='disabled')

    # ── aba 2: admitância ───────────────────────────────────────────────
    def _build_admittance(self):
        p = self._page('2  Admitância')
        p.columnconfigure(0, weight=3)
        p.columnconfigure(1, weight=2)
        p.rowconfigure(0, weight=1)
        left = tk.Frame(p, bg=th.BG)
        left.grid(row=0, column=0, sticky='nsew', padx=(th.PAD_S, 2), pady=th.PAD_S)
        right = tk.Frame(p, bg=th.BG)
        right.grid(row=0, column=1, sticky='nsew', padx=(2, th.PAD_S), pady=th.PAD_S)

        c = th.card(left, 'Perfil')
        row = tk.Frame(c, bg=th.PANEL)
        row.pack(fill='x')
        self.preset_btns = {}
        for nome in T.PRESETS:
            b = self._btn(row, nome, lambda n=nome: self.apply_preset(n), 'Seg')
            b.pack(side='left', fill='x', expand=True, padx=(0, 2))
            self.preset_btns[nome] = b
        self.lbl_preset = th.label(c, '', muted=True)
        self.lbl_preset.pack(anchor='w', pady=(th.PAD, 0))

        c = th.card(left, 'Parâmetros   M·a + B·v + K·(x − x_ref) = F', fill='both', expand=True)
        c.columnconfigure(0, weight=1)
        self.pvar, self.plbl, self.pnode = {}, {}, {}
        for r, (key, rot, lo, hi, st, un) in enumerate(PARAMS):
            th.label(c, rot).grid(row=2 * r, column=0, sticky='w', pady=(1, 0))
            self.pnode[key] = th.label(c, '', muted=True, font=th.FONT_SMALL)
            self.pnode[key].grid(row=2 * r, column=1, sticky='e', padx=th.PAD, pady=(1, 0))
            self.plbl[key] = tk.Label(c, text='', bg=th.PANEL, fg=th.TEXT, width=11,
                                      anchor='e', font=th.FONT_MONO)
            self.plbl[key].grid(row=2 * r, column=2, sticky='e', pady=(1, 0))
            v = tk.DoubleVar(value=T.PRESETS['Livre'].get(
                key, C.TORQUE_ABORT_LIMIT_NM if key == 'abort_torque_nm' else C.FORCE_ABORT_LIMIT_N))
            self.pvar[key] = v
            ttk.Scale(c, from_=lo, to=hi, variable=v,
                      command=lambda _x, k=key, s=st: self._slide(k, s, manual=True)
                      ).grid(row=2 * r + 1, column=0, columnspan=3, sticky='ew')
            self._slide(key, st)

        c = th.card(right, 'Estabilidade do controlador')
        self.lbl_ratio = th.readout(c, 'Rigidez máx. do braço [N/m]')
        self.lbl_ratio.frame.pack(anchor='w')
        th.label(c, 'Folga com o atraso estimado da malha', muted=True, font=th.FONT_SMALL).pack(anchor='w', pady=(th.PAD, 2))
        self.bar_stab = ttk.Progressbar(c, style='Stab.Horizontal.TProgressbar',
                                        maximum=100, mode='determinate')
        self.bar_stab.pack(fill='x')
        self.lbl_stab = th.label(c, '', font=th.FONT_TITLE)
        self.lbl_stab.pack(anchor='w', pady=(th.PAD, 0))

        c = th.card(right, 'Nó de admitância', fill='both', expand=True)
        row = tk.Frame(c, bg=th.PANEL)
        row.pack(fill='x')
        self.btn_apply = self._btn(row, 'Aplicar ao nó', self.apply_params, 'Primary')
        self.btn_apply.pack(side='left')
        self._btn(row, 'Ler do nó', self.load_params).pack(side='left', padx=th.PAD)
        self.lbl_node_params = th.label(c, 'Admitância fora do ar.',
                                        muted=True, wraplength=360)
        self.lbl_node_params.pack(anchor='w', pady=(th.PAD, 0))
        self._refresh_stab()
        self._mark_preset()

    def _slide(self, key, step, manual=False):
        v = round(round(self.pvar[key].get() / step) * step, 6)
        self.pvar[key].set(v)
        un = next(p[5] for p in PARAMS if p[0] == key)
        self.plbl[key].configure(text=f'{v:g} {un}')
        if manual and self._preset and T.PRESETS[self._preset].get(key, v) != v:
            self._preset = None
            self._mark_preset()
        if hasattr(self, 'lbl_stab'):
            self._refresh_stab()

    def _mark_preset(self):
        for n, b in self.preset_btns.items():
            b.configure(style='SegOn.TButton' if n == self._preset else 'Seg.TButton')
        self.lbl_preset.configure(text=PRESET_HELP.get(self._preset,
                                                       'Personalizado: valores ajustados à mão.'))

    def _refresh_stab(self):
        g = {k: v.get() for k, v in self.pvar.items()}
        s = T.stability(g['mass_kg'], g['damping_ns_m'])
        k = s['k_max']
        m = min(100 * k / (2 * C.ARM_STIFFNESS_REF_N_M), 100)
        if k >= C.ARM_STIFFNESS_REF_N_M:
            cor, txt = th.OK, f'✔  Estável até {k:.0f} N/m'
        elif s['ok']:
            cor, txt = th.WARN, f'▲  Perto do limite: {k:.0f} N/m'
        else:
            cor, txt = th.DANGER_HV, '✖  Instável — não será aplicado'
        self.lbl_ratio.configure(text=f'{k:.0f}')
        self.bar_stab.configure(value=m)
        ttk.Style().configure('Stab.Horizontal.TProgressbar', background=cor,
                              lightcolor=cor, darkcolor=cor)
        self.lbl_stab.configure(text=txt, fg=cor)
        self.btn_apply.state(['!disabled'] if s['ok'] else ['disabled'])

    def apply_preset(self, nome):
        why = T.preset_refusal(nome, self.br.last.get('params'), self.enabled or bool(self._enable_wait))
        if why:
            self._say(why, False)
            return
        for k, v in T.PRESETS[nome].items():
            self.pvar[k].set(v)
            self._slide(k, next(p[4] for p in PARAMS if p[0] == k))
        self._preset = nome
        self._mark_preset()
        # só o preset: um limite de aborto mexido e não aplicado não vai de carona
        from .gui_bridge import NODE_ADMITTANCE
        self.br.set_params(NODE_ADMITTANCE, dict(T.PRESETS[nome]))
        self._say(f'Perfil {nome} enviado ao nó.')

    def apply_params(self):
        g = {k: v.get() for k, v in self.pvar.items()}
        if not T.stability(g['mass_kg'], g['damping_ns_m'])['ok']:
            self._say('Combinação instável: não aplicada. Aumente B ou reduza M.', False)
            return
        if not T.abort_limit_ok(g['abort_force_n']):
            self._say(f'Limite de aborto deve ser ≤ {C.FORCE_ABORT_LIMIT_N:g} N.', False)
            return
        if ((self.enabled or self._enable_wait)
                and (self.br.last.get('params') or {}).get('stiffness_n_m') != g['stiffness_n_m']):
            self._say('A mola (K) não muda com o braço ativo: desabilite antes.', False)
            return
        from .gui_bridge import NODE_ADMITTANCE
        self.br.set_params(NODE_ADMITTANCE, g)

    def load_params(self):
        p = self.br.last.get('params')
        if not p:
            self._say('Sem parâmetros do nó: rehab_admittance não está no ar.', False)
            return
        for key, *_r, st, _u in PARAMS:
            if key in p:
                self.pvar[key].set(p[key])
                self._slide(key, st)
        self._preset = next((n for n, d in T.PRESETS.items()
                             if all(abs(self.pvar[k].get() - v) < 1e-9 for k, v in d.items())), None)
        self._mark_preset()

    # ── aba 3: tarefas ──────────────────────────────────────────────────
    def _build_tasks(self):
        p = self._page('3  Tarefas')
        side = tk.Frame(p, bg=th.BG, width=330)
        side.pack(side='left', fill='y', padx=(th.PAD_S, 0), pady=th.PAD_S)
        side.pack_propagate(False)
        self.cv = tk.Canvas(p, bg=th.SUNKEN, highlightthickness=1,
                            highlightbackground=th.BORDER)
        self.cv.pack(side='left', fill='both', expand=True, padx=th.PAD, pady=th.PAD)
        self.cv.bind('<Configure>', self._cv_resize)

        c = th.card(side, 'Tarefa')
        self.var_task = tk.StringVar(value='centro_fora')
        self.var_task_label = tk.StringVar(value=TASK_LABELS['centro_fora'])
        cb = ttk.Combobox(c, textvariable=self.var_task_label, state='readonly',
                          values=list(TASK_LABELS.values()))
        cb.pack(fill='x')
        cb.bind('<<ComboboxSelected>>', lambda e: self.var_task.set(
            next(k for k, v in TASK_LABELS.items() if v == self.var_task_label.get())))
        self.var_task.trace_add('write', lambda *_: (
            self._task_changed(), self.var_task_meta.set(self.var_task.get())))
        self.tp_frame = tk.Frame(c, bg=th.PANEL)
        self.tp_frame.pack(fill='x', pady=(th.PAD, 0))
        self.tv, self.tp_rows = {}, []
        for k, rot, ini, lo, hi, st, quais in TASK_PARAMS:
            lab = th.label(self.tp_frame, rot)
            v = tk.DoubleVar(value=ini)
            self.tv[k] = v
            sp = ttk.Spinbox(self.tp_frame, from_=lo, to=hi, increment=st, textvariable=v, width=6)
            self.tp_rows.append((quais, lab, sp))
        self.var_assist = tk.BooleanVar(value=False)
        self.var_assist.trace_add('write', lambda *_: self._assist_changed())
        self.chk_assist = ttk.Checkbutton(self.tp_frame, text='Assistência (puxa para o túnel;\n'
                                                              'use o perfil Assistido, K > 0)',
                                          variable=self.var_assist)
        row = tk.Frame(c, bg=th.PANEL)
        row.pack(fill='x', pady=(th.PAD, 0))
        self._btn(row, 'Iniciar', self.task_start, 'Primary').pack(side='left', fill='x', expand=True)
        self._btn(row, 'Parar tarefa', self.task_stop).pack(side='left', fill='x', expand=True, padx=(4, 0))
        self._btn(c, 'Recentrar na posição atual', self.recenter).pack(fill='x', pady=(4, 0))

        c = th.card(side, 'Placar', fill='both', expand=True)
        g = tk.Frame(c, bg=th.PANEL)
        g.pack(fill='x')
        self.score = []
        for i in range(3):
            v = th.readout(g, '')
            v.frame.grid(row=0, column=i, sticky='w', padx=(0, 14))
            v.caption = v.frame.winfo_children()[0]
            self.score.append(v)
        self.bar_task = ttk.Progressbar(c, style='Task.Horizontal.TProgressbar', maximum=100)
        self.bar_task.pack(fill='x', pady=(th.PAD, 2))
        self.lbl_score = th.label(c, 'Nenhuma tarefa em andamento.', muted=True, wraplength=280)
        self.lbl_score.pack(anchor='w')
        self._task_changed()

    def _task_changed(self):
        name = self.var_task.get()
        if name in TASK_LABELS:
            self.var_task_label.set(TASK_LABELS[name])
        grupo = 'tunel' if name.startswith('tunel_') else name
        for w in self.tp_frame.winfo_children():
            w.grid_forget()
        r = 0
        for quais, lab, sp in self.tp_rows:
            if grupo in quais:
                lab.grid(row=r, column=0, sticky='w', pady=2)
                sp.grid(row=r, column=1, sticky='e', pady=2)
                r += 1
        self.tp_frame.columnconfigure(1, weight=1)
        if grupo == 'tunel':
            self.chk_assist.grid(row=r, column=0, columnspan=2, sticky='w', pady=(4, 0))

    def _assist_changed(self):
        if isinstance(self.task, T.TunnelTask) and not self.task.done:
            self._emit([('PARAMS', dict(assistencia=int(self.var_assist.get())))])

    def _cv_resize(self, e):
        self._scale = min(e.width, e.height) / VIEW_SPAN_M
        self.cv.delete('trace')
        self._drawn = set()
        self._draw_grid()

    def _draw_grid(self):
        """Grade de 5 cm, barra de escala e orientação (estáticos)."""
        cv = self.cv
        cv.delete('grid')
        w, h = cv.winfo_width(), cv.winfo_height()
        step = 0.05 * self._scale
        cx, cy = w / 2, h / 2
        k = 0
        while k * step < max(w, h):
            for s in (1, -1) if k else (1,):
                cv.create_line(cx + s * k * step, 0, cx + s * k * step, h, fill=th.GRID, tags='grid')
                cv.create_line(0, cy + s * k * step, w, cy + s * k * step, fill=th.GRID, tags='grid')
            k += 1
        cv.create_text(w / 2, 12, text='▲ longe do voluntário (−x)', fill=th.MUTED,
                       font=th.FONT_SMALL, tags='grid')
        cv.create_text(w / 2, h - 12, text='▼ perto do voluntário (+x)', fill=th.MUTED,
                       font=th.FONT_SMALL, tags='grid')
        cv.create_text(w - 10, h / 2 - 12, anchor='e', text='direita (+y) ▶', fill=th.MUTED,
                       font=th.FONT_SMALL, tags='grid')
        cv.create_line(16, h - 30, 16 + step, h - 30, fill=th.MUTED, width=2, tags='grid')
        for x in (16, 16 + step):
            cv.create_line(x, h - 34, x, h - 26, fill=th.MUTED, width=2, tags='grid')
        cv.create_text(16 + step / 2, h - 38, text='5 cm', fill=th.MUTED,
                       font=th.FONT_SMALL, tags='grid')
        cv.tag_lower('grid')

    def _w2s(self, x, y):
        """Mundo -> pixel: cima = −x, direita = +y, centrado em self.center."""
        cx, cy = self.center[:2]
        return (self.cv.winfo_width() / 2 + (y - cy) * self._scale,
                self.cv.winfo_height() / 2 + (x - cx) * self._scale)

    def recenter(self):
        if self.tick:
            self.center = (self.tick['x_m'], self.tick['y_m'], self.tick['z_m'])
            for ch in self.charts:
                ch.clear()
            self.cv.delete('trace')
            self._drawn = set()
            self.trail.clear()

    def _center_ok(self):
        return self.center is not None and all(math.isfinite(v) for v in self.center)

    # ── tarefas: ciclo de vida ──────────────────────────────────────────
    def task_start(self):
        if not self.tick:
            self._say('Sem posição do braço: o nó rehab_admittance não está no ar.', False)
            return
        if self._state_name(self._live()) != 'ATIVO':
            self._say('Habilite o braço (estado ATIVO) antes de iniciar a tarefa.', False)
            return
        if self.task and not self.task.done:
            self.task_stop()
        self.recenter()
        cx, cy, cz = self.center
        t, name = self.tick['t_s'], self.var_task.get()
        try:
            g = {k: float(v.get()) for k, v in self.tv.items()}
        except (tk.TclError, ValueError):
            self._say('Parâmetro da tarefa vazio ou inválido.', False)
            return
        for k, rot, _ini, lo, hi, _st, _q in TASK_PARAMS:
            if not lo <= g[k] <= hi:
                self._say(f'{rot}: use de {lo:g} a {hi:g}.', False)
                return
        exc = (self.br.last.get('params') or {}).get('max_excursion_m')
        if exc is None:
            self._say('Excursão máxima desconhecida: sem parâmetros do nó.', False)
            return
        chave = 'alcance' if name == 'centro_fora' else 'tamanho' if name.startswith('tunel_') else None
        if chave:
            d, (lo, hi) = self.tick, next(p[3:5] for p in TASK_PARAMS if p[0] == chave)
            ref = (d['rx_m'], d['ry_m'], d['rz_m'])
            cabe = 100 * min(T.reach_limit(name, exc, self.center, c) for c in (ref, self.origin or ref))
            if not cabe >= lo:           # NaN também recusa
                self._say(f'Só há {max(cabe, 0.0):.0f} cm dentro da excursão máxima: desabilite e '
                          'habilite mais perto do centro.', False)
                return
            g[chave] = min(g[chave], cabe)
        if name == 'centro_fora':
            self.task = T.CenterOutTask(
                (cx, cy), reach_m=g['alcance'] / 100, target_radius_m=g['raio'] / 100,
                timeout_s=g['limite'], seed=int(g['seed']))
        elif name.startswith('tunel_'):
            shape = name.split('_', 1)[1]
            self.task = T.TunnelTask(shape, (cx, cy), size_m=g['tamanho'] / 100,
                                     width_m=g['largura'] / 100)
        else:
            self.task = T.ReachTask((cx, cy), duration_s=g['duracao'])
        self.task_name, self._task_z = name, cz
        self.cv.delete('trace')
        self._drawn = set()
        self._co_result, self._flash = {}, None
        ev = self.task.start(t)
        if isinstance(self.task, T.TunnelTask):
            ev[0][1]['assistencia'] = int(self.var_assist.get())
        ev[0][1].update(x0=cx, y0=cy)
        self._emit(ev)
        self._say(f'Tarefa iniciada: {TASK_LABELS.get(name, name)}'
                  + (f' ({g[chave]:.0f} cm).' if chave else '.'))

    def task_stop(self):
        if self.task and self.tick:
            self._emit(self.task.stop(self.tick['t_s']))
        self._task_finished()

    def _task_finished(self):
        if self.task_name and self.tick:
            nan = float('nan')
            self.br.publish_target((nan, nan), nan)
            self._ref_recenter()
        self.task_name = None

    def _ref_recenter(self):
        """Referência no ponto atual: com K > 0 não puxa o braço para o fim da trajetória.
        O nó só a aceita em ATIVO; em SEM_FORCA/LIMITE espera a volta."""
        st = self._state_name(self._live())
        if not self._ref_sent or st in ('SEM_FORCA', 'LIMITE'):
            return
        if st == 'ATIVO':
            self.br.publish_reference((self.tick['x_m'], self.tick['y_m']), self.tick['z_m'])
        self._ref_sent = False           # nos outros, reabilitar já zera a referência

    def _emit(self, events):
        for tipo, kv in events:
            self.br.publish_event(T.format_event(tipo, **kv))
            if tipo == 'TARGET_ON':
                self._publish_target()
            elif tipo in ('TARGET_HIT', 'TARGET_MISS') and isinstance(self.task, T.CenterOutTask):
                self._on_target_result(tipo == 'TARGET_HIT', kv)

    def _on_target_result(self, hit, kv):
        """Feedback de acerto/erro (só visual; a lógica está em tasks.py)."""
        k = kv['alvo']
        pos = self.task.target_positions[k] if kv['fase'] == 'ida' else (self.task.cx, self.task.cy)
        if kv['fase'] == 'ida':
            self._co_result[k] = 'hit' if hit else 'miss'
        self._flash = (time.monotonic(), pos[0], pos[1],
                       'Acertou!' if hit else 'Tempo esgotado', th.OK if hit else th.MUTED)

    def _publish_target(self):
        if isinstance(self.task, T.CenterOutTask) and self.task.current_target() and self.center:
            self.br.publish_target(self.task.current_target(), self.center[2])
            self._last_target_pub = time.monotonic()

    # ── aba 4: gráficos ─────────────────────────────────────────────────
    def _build_charts(self):
        p = self._page('4  Gráficos')
        th.label(p, 'Últimos 10 s', muted=True).pack(anchor='w', padx=th.PAD_L, pady=(th.PAD, 0))
        mag = (th.SERIES_MAG, 2)
        xyz = ((th.SERIES_X, 1.5), (th.SERIES_Y, 1.5), (th.SERIES_Z, 1.5))
        self.charts = [
            LineChart(p, 'Força no mundo', 'N',
                      [('|F|', *mag)] + [(n, *s) for n, s in zip(('Fx', 'Fy', 'Fz'), xyz)],
                      min_span=2.0, height=120),
            LineChart(p, 'Velocidade do TCP', 'mm/s',
                      [('|v|', *mag)] + [(n, *s) for n, s in zip(('vx', 'vy', 'vz'), xyz)],
                      min_span=20.0, height=120),
            LineChart(p, 'Posição do TCP em relação ao centro', 'cm',
                      [(n, *s) for n, s in zip(('Δx', 'Δy', 'Δz'), xyz)], min_span=4.0,
                      height=120)]
        for ch in self.charts:
            ch.pack(fill='both', expand=True, padx=th.PAD, pady=th.PAD_S)

    # ── aba 5: mão virtual ──────────────────────────────────────────────
    # ── aba 5 / Posicionar juntas ───────────────────────────────────────
    def _build_jog(self, f):
        c = th.card(f, 'Juntas (graus)')
        c.pack(fill='x', padx=th.PAD_S, pady=th.PAD_S)
        c.columnconfigure(2, weight=1)
        self.lbl_jog_mode = th.label(c, '', muted=True)
        self.lbl_jog_mode.grid(row=0, column=0, columnspan=5, sticky='w', pady=(0, th.PAD))
        for col, t in enumerate(('Junta', 'Atual', 'Alvo', '', 'Real')):
            th.label(c, t, muted=True).grid(row=1, column=col, sticky='w', padx=th.PAD_L)
        self.jog_cur, self.jog_real, self.jog_var, self.jog_spin = [], [], [], []
        self.jog_widgets = []
        for j, (lo, hi) in enumerate(C.JOINT_LIMITS_DEG):
            r = 2 + j
            th.label(c, f'J{j + 1}', font=th.FONT_TITLE).grid(row=r, column=0, sticky='w', pady=3)
            cur = tk.Label(c, text='—', bg=th.PANEL, fg=th.TEXT, font=th.FONT_MONO,
                           width=9, anchor='e')
            cur.grid(row=r, column=1, padx=th.PAD_L)
            v = tk.DoubleVar(value=0.0)
            sc = ttk.Scale(c, from_=lo, to=hi, variable=v, command=lambda _x, j=j: self._jog_user(j))
            sc.grid(row=r, column=2, sticky='ew', padx=th.PAD_L)
            sp = ttk.Spinbox(c, from_=lo, to=hi, increment=1.0, textvariable=v, width=8,
                             format='%.1f', command=lambda j=j: self._jog_user(j))
            sp.grid(row=r, column=3, padx=th.PAD_L)
            # só no Enter: digitar '120' passaria por 1° e 12°
            sp.bind('<Return>', lambda e, j=j: self._jog_user(j))
            for ev in ('<MouseWheel>', '<Button-4>', '<Button-5>'):
                sp.bind(ev, lambda e: 'break')   # rolar a página não move o braço
            sp.bind('<Key>', lambda e: setattr(self, '_jog_t', time.monotonic()))
            self.jog_widgets += [sc, sp]
            real = tk.Label(c, text='—', bg=th.PANEL, fg=th.MUTED, font=th.FONT_MONO,
                            width=9, anchor='e')
            real.grid(row=r, column=4, padx=th.PAD_L)
            self.jog_cur.append(cur); self.jog_var.append(v)
            self.jog_real.append(real); self.jog_spin.append(sp)
        bt = tk.Frame(c, bg=th.PANEL)
        bt.grid(row=8, column=0, columnspan=6, sticky='w', pady=(th.PAD_L, 0))
        self._btn(bt, '⌂  Home', self.jog_home, 'Primary').pack(side='left')
        self._btn(bt, '✔  Salvar Home', self.jog_save_home).pack(side='left', padx=th.PAD)
        self._btn(bt, '⌖  Capturar do robô', self.jog_capture).pack(side='left')
        th.label(bt, 'Velocidade').pack(side='left', padx=(th.PAD_L, 4))
        self.jog_speed = tk.DoubleVar(value=C.JOG_SPEED_DEFAULT_PCT)
        ttk.Scale(bt, from_=C.JOG_SPEED_MIN_PCT, to=100.0, variable=self.jog_speed,
                  length=160).pack(side='left')
        self.lbl_speed = th.label(bt, '', muted=True)
        self.lbl_speed.pack(side='left', padx=4)
        self.lbl_jog_est = th.label(c, '', muted=True)
        self.lbl_jog_est.grid(row=9, column=0, columnspan=6, sticky='w', pady=(th.PAD, 0))
        self.lbl_jog_res = th.label(c, '')
        self.lbl_jog_res.grid(row=10, column=0, columnspan=6, sticky='w', pady=(2, 0))
        th.label(c, 'Não confere cadeira nem pessoas: mova aos poucos.', muted=True,
                 font=th.FONT_SMALL).grid(row=11, column=0, columnspan=6, sticky='w',
                                          pady=(th.PAD, 0))

    def _jog_user(self, j=None):
        """Slider, seta ou Enter na junta `j` (None = as 6, Home): envia o alvo 80 ms depois
        da última mexida (como o touch_pack)."""
        self._jog_t = time.monotonic()
        pend = self._jog_js if self._jog_after is not None else set()
        self._jog_cancel()
        self._jog_js = set(range(6)) if j is None else pend | {j}
        self._jog_after = self.root.after(80, self._jog_send)

    def _jog_cancel(self):
        if self._jog_after is not None:
            self.root.after_cancel(self._jog_after)
            self._jog_after = None

    def _jog_send(self):
        self._jog_after = None
        estado = self._state_name(self._live())
        goal = self._jog_goal_deg(estado)
        if goal is None:
            self._jog_msg('Valor inválido em algum campo.', False)
        elif not self._jog_synced or self._jog_current_deg() is None:
            # sem isto os sliders não mexidos mandariam 0°
            self._jog_msg('Pose atual das juntas desconhecida: espere os sliders seguirem o braço.', False)
        elif estado not in ('PARADO', 'POSICIONANDO'):
            self._jog_msg('Só com o braço PARADO.', False)
        else:
            self._jog_alvo_ant, self._jog_alvo = self._jog_alvo, goal
            self.br.publish_goto([math.radians(g) for g in goal], self.jog_speed.get())

    def _jog_msg(self, texto, ok=True):
        self.lbl_jog_res.configure(text=texto, fg=th.OK if ok else th.DANGER_HV)
        self._say(texto, ok)

    def _jog_set(self, deg):
        for v, g in zip(self.jog_var, deg):
            v.set(round(g, 1))

    def jog_home(self):
        if not self._confirm('Ir para Home', 'As 6 juntas vão para a Home salva, sem conferir cadeira '
                             'nem pessoas. Em MIRROR o CR10 REAL vai junto. Área livre e mão no E-Stop?'):
            return
        self._jog_set([math.degrees(q) for q in C.home_q()])
        self._jog_user()

    def jog_save_home(self):
        self._save_home(self._jog_current_deg(), 'Sem /joint_states: o braço não está no ar.')

    def jog_capture(self):
        st = self._real_status() or {}
        q = st.get('q_real')
        self._save_home([math.degrees(v) for v in q] if q else None,
                        'Conecte o CR10 antes de capturar a posição.')

    def _save_home(self, deg, sem):
        if deg is None:
            return self._jog_msg(sem, False)
        if not all(lo <= g <= hi for g, (lo, hi) in zip(deg, C.JOINT_LIMITS_DEG)):
            return self._jog_msg('Posição fora dos limites de junta: Home não salva.', False)
        try:
            C.save_home([math.radians(g) for g in deg])
        except OSError as e:
            return self._jog_msg(f'Home não salva: {e}', False)
        self._jog_msg('Home salva: ' + ' / '.join(f'{g:+.0f}°' for g in deg))

    def _refresh_jog(self, live_adm):
        br = self.br
        cur = self._jog_current_deg()
        for j, lb in enumerate(self.jog_cur):
            lb.configure(text='—' if cur is None else f'{cur[j]:+7.1f}°')
        st = self._real_status() or {}
        q_real = st.get('q_real')
        for j, lb in enumerate(self.jog_real):
            if not q_real:
                lb.configure(text='—', fg=th.MUTED)
                continue
            dr = math.degrees(q_real[j])
            off = cur is not None and abs(dr - cur[j]) > 3.0
            lb.configure(text=f'{dr:+7.1f}°', fg=th.DANGER_HV if off else th.TEXT)
        espelho = st.get('conectado') and st.get('modo') != 'SIM_ONLY'
        self.lbl_jog_mode.configure(
            text=('Robô real: DESCONHECIDO' if not st else
                  'O robô REAL segue o slider' if espelho else 'Só simulação'),
            fg=th.DANGER_HV if espelho or not st else th.MUTED)
        self.lbl_speed.configure(text=f'{self.jog_speed.get():.0f} %')
        nome = self._state_name(live_adm)
        pode = nome in ('PARADO', 'POSICIONANDO')
        for w in self.jog_widgets:
            w.state(['!disabled'] if pode else ['disabled'])
        # parado e sem mexida recente: os sliders mostram onde o braço está
        if cur is not None and nome != 'POSICIONANDO' and time.monotonic() - self._jog_t > 2.0:
            self._jog_set(cur)
            self._jog_synced = True
        elif cur is None:
            self._jog_synced = False
        raw = br.last.get('jog')
        if raw and br.age('jog') < 30.0 and raw != self._jog_raw:
            self._jog_raw = raw
            try:
                r = json.loads(raw)
                if not r.get('ok') and not r.get('fim'):   # recusado: segue valendo o anterior
                    self._jog_alvo = self._jog_alvo_ant
                self.lbl_jog_res.configure(
                    text=('✔ ' if r.get('ok') else '✖ ') + str(r.get('motivo', '')),
                    fg=th.OK if r.get('ok') else th.DANGER_HV)
            except ValueError:
                pass
        self.lbl_jog_est.configure(text='' if pode else
                                   f'Só em PARADO (agora: {STATE_LABEL.get(nome, nome)}).')

    def _jog_goal_deg(self, estado='PARADO'):
        """Só as juntas mexidas vêm dos campos: um valor meio digitado noutra não vai de carona."""
        js = self._jog_js or set(range(6))
        base = (self._jog_alvo if estado == 'POSICIONANDO' else None) or self._jog_current_deg()
        try:
            return [float(v.get()) if j in js or base is None else base[j]
                    for j, v in enumerate(self.jog_var)]
        except (tk.TclError, ValueError):
            return None

    def _jog_current_deg(self):
        m = self.br.last.get('joints')
        if m is None or self.br.age('joints') > 1.0:
            return None
        try:
            return [math.degrees(m.position[m.name.index(j)]) for j in C.ARM_JOINTS]
        except (ValueError, IndexError):
            return None

    def _real_status(self):
        if self.br.age('real') > 1.0:
            return None
        try:
            return json.loads(self.br.last['real'])
        except (ValueError, KeyError, TypeError):
            return None

    # ── ações de sessão e braço ─────────────────────────────────────────
    def estop(self):
        now = time.monotonic()
        if now - self._last_estop < 0.3:
            return
        self._last_estop = self._estop_wait = now
        self._estop_ok = None
        # dois caminhos independentes: um que falhe (contexto ROS caído) não impede o outro
        for fn in (lambda: self.br.call_trigger(C.S_ESTOP),
                   lambda: self.br.publish_event(T.format_event('ESTOP', origem='gui'))):
            try:
                fn()
            except Exception as e:
                self.br.results.append((C.S_ESTOP, False, str(e)))
        self._say('PARAR enviado ao braço.', False)
        self._jog_cancel()
        self._confirm_cancel()

    def tare(self):
        if self._confirm(
                'Tarar sensor',
                'Solte o mastro: ninguém pode tocar na empunhadura.\n'
                'A tara mede a força por 0,5 s e vale até mover as juntas (aba 5): refaça depois.\n\nTarar agora?'):
            self.br.call_trigger(C.S_TARE)

    def toggle_real(self):
        st = self._real_status()
        if not st:                       # sem status: não se sabe se pede conectar ou desconectar
            self._say('Estado do robô real desconhecido: o nó rehab_mirror não responde.', False)
            return
        if st.get('conectado') or st.get('conectando'):
            self.br.call_bool(C.S_REAL_CONNECT, False)
            return
        ip = self.var_ip.get().strip()
        if not ip:                       # vazio cairia no robot.json, sem o operador ver qual
            self._say('Informe o IP do CR10 antes de conectar.', False)
        elif self._confirm(
                'Conectar ao CR10 real', f'IP {ip}: o robô será energizado e '
                'habilitado (freios soltos). Área livre e mão no E-Stop?'):
            self.br.connect_real(ip)

    def toggle_mode(self):
        st = self._real_status()
        if not st:
            self._say('Estado do robô real desconhecido: o nó rehab_mirror não responde.', False)
            return
        on = st.get('modo') != 'MIRROR'
        src = (self.br.last.get('params') or {}).get('force_source')
        if on and src != 'real' and self.var_ip.get().strip() != C.SIM_ROBOT_IP:
            self._say(f'MIRROR recusado: a força é "{src}". Relance com force_source:=real.', False)
            return
        if not on or self._confirm(
                'Modo MIRROR', 'O CR10 REAL passará a seguir o simulado quando a admitância '
                'estiver ativa. Mão no E-Stop físico?'):
            self.br.call_bool(C.S_REAL_MODE, on)

    def toggle_enable(self):
        """Pede o que o rótulo diz; trava o botão para o 2º clique não pedir o contrário."""
        if self._enable_wait:
            return
        self._enable_wait, self._enable_done = time.monotonic(), False
        self.btn_enable.state(['disabled'])
        self.br.call_enable(self.btn_enable.cget('text') == 'Habilitar')

    def disable(self):
        self.br.call_enable(False)

    def _enable_release(self):
        # 0,8 s no mínimo: a resposta chega antes do 2º clique de um duplo clique
        dt = time.monotonic() - self._enable_wait
        if self._enable_wait and dt > (0.8 if self._enable_done else 3.0):
            self._enable_wait = 0.0
            self.btn_enable.state(['!disabled'])

    def reset(self):
        self.br.call_trigger(C.S_RESET)

    def mark(self):
        txt = self.var_mark.get().strip()
        if not self.session_open:
            self._say('Marcador NÃO gravado: nenhuma sessão gravando.', False)
            return
        self.br.publish_event(T.format_event('MARK', texto=txt))
        self._say(f'Marcador enviado ao registrador: {txt or "(sem texto)"}')

    def session_start(self):
        sid = self.var_id.get().strip()
        if not T.validate_subject_id(sid):
            self._say('ID inválido: use de 1 a 16 letras, números, _ ou - '
                      '(nunca o nome do voluntário). Ex.: S01', False)
            return
        self.br.publish_meta({'subject_id': sid, 'lado_paretico': self.var_side.get(),
                              'tarefa': self.var_task_meta.get(),
                              'notas': self.txt_notes.get('1.0', 'end').strip()})
        self.br.start_session()

    # ── teclado ─────────────────────────────────────────────────────────
    def _space_on_widget(self, e):
        self.estop()
        return 'break'

    def _on_key_press(self, e):
        k = e.keysym.lower()
        # com o braço se movendo, Espaço é PARAR até dentro de um campo de texto
        if k == 'space' and (self.moving or not self._typing()):
            self.estop()
            return 'break'
        if self._typing():
            return None
        if k == 'm':
            self.mark()
        elif k in ('1', '2', '3', '4', '5'):
            self.nb.select(int(k) - 1)
        return None

    # ── laço periódico ──────────────────────────────────────────────────
    def _poll(self):
        if self._closing:
            return
        self._n_poll += 1
        try:
            self.br.publish_alive()      # do laço do Tk: tela travada para de bater
            self._drain_results()
            self._drain_ticks()
            if self.task and not self.task.done and self.tick:
                self._task_housekeeping()
            elif self.task_name and self.task and self.task.done:
                self._task_finished()
            elif not self.task_name and self.tick:
                self._ref_recenter()
            if (self.task and not self.task.done and self.tick and
                    self._state_name(self._live()) in ('PARADO', 'ABORTADO', 'POSICIONANDO', 'SEM DADOS')):
                self.task_stop()         # nada de referência velha ao reabilitar
            if self._estop_wait and time.monotonic() - self._estop_wait > 0.5:
                self._estop_wait = 0.0
                self._estop_alert = ('PARAR SEM RESPOSTA da admitância: use a parada de emergência '
                                     'física do robô.')
                self._say(self._estop_alert, False)
            if self._n_poll % 3 == 0:
                self._refresh_ui()
        except Exception as e:           # o laço nunca morre: congelaria força e estado
            if self._closing:
                return
            traceback.print_exc()
            self._say(f'Erro interno na tela: {e}', False)
        self.root.after(POLL_MS, self._poll)

    def _drain_results(self):
        while self.br.results:
            nome, ok, msg = self.br.results.popleft()
            if nome == C.S_ESTOP:
                self._estop_wait, self._estop_ok = 0.0, ok
                if ok:
                    self._estop_alert = ''
            if nome == C.S_ENABLE:
                self._enable_done = True
            acao, no = ACTION.get(nome, ('Parâmetros' if nome.startswith('params:') else nome,
                                         'rehab_admittance'))
            if ok:
                self._say(f'{acao}: {msg or "feito."}')
            elif nome == C.S_ESTOP:
                self._estop_alert = (f'PARAR NÃO CHEGOU AO BRAÇO: o nó {no} não responde. '
                                     'Use a parada de emergência física do robô.')
                self._say(self._estop_alert, False)
            elif 'indispon' in msg:
                self._say(f'{acao} não executado: o nó {no} não está no ar.', False)
            else:
                self._say(f'{acao} falhou: {msg}', False)
            if nome == C.S_SESSION_START and ok:
                self.session_open, self._session_t = True, time.monotonic()
                self.lbl_session.configure(text=f'● Sessão aberta — gravando em {msg}', fg=th.OK)
                self.lbl_sub.configure(text=f'Sessão aberta: {self.var_id.get()} · gravando')
            if nome == C.S_SESSION_STOP and ok:
                self.session_open, self._session_t = False, time.monotonic()
                self.lbl_session.configure(text='○ Sessão encerrada.', fg=th.MUTED)
                self.lbl_sub.configure(text='Nenhuma sessão de coleta aberta')

    def _drain_ticks(self):
        q = self.br.ticks
        while q:
            raw = q.popleft()
            if len(raw) < len(C.TICK_FIELDS):
                continue
            d = dict(zip(C.TICK_FIELDS, raw))
            self.tick = d
            if self._state_name(True) not in ('ATIVO', 'SEM_FORCA', 'LIMITE'):
                self.origin = None
            elif self.origin is None:    # onde se habilitou: centro da excursão
                self.origin = (d['rx_m'], d['ry_m'], d['rz_m'])
            if not all(math.isfinite(d[k]) for k in ('t_s', 'x_m', 'y_m', 'z_m')):
                continue                 # ainda sem /joint_states: NaN não vai às tarefas
            if not self._center_ok():
                self.center = (d['x_m'], d['y_m'], d['z_m'])
            cx, cy, cz = self.center
            t = d['t_s']
            fm = math.sqrt(d['fx_n'] ** 2 + d['fy_n'] ** 2 + d['fz_n'] ** 2)
            vm = 1000 * math.sqrt(d['vx_ms'] ** 2 + d['vy_ms'] ** 2 + d['vz_ms'] ** 2)
            self.charts[0].add(t, [fm, d['fx_n'], d['fy_n'], d['fz_n']])
            self.charts[1].add(t, [vm, 1000 * d['vx_ms'], 1000 * d['vy_ms'], 1000 * d['vz_ms']])
            self.charts[2].add(t, [100 * (d['x_m'] - cx), 100 * (d['y_m'] - cy),
                                   100 * (d['z_m'] - cz)])
            self.trail.append((t, d['x_m'], d['y_m']))
            while self.trail and (self.trail[0][0] < t - TRAIL_S or self.trail[0][0] > t):
                self.trail.popleft()
            if self.task and not self.task.done:
                self._emit(self.task.update(t, d['x_m'], d['y_m']))
                if self.task.done:
                    self._task_finished()

    def _task_housekeeping(self):
        if isinstance(self.task, T.CenterOutTask):
            if time.monotonic() - self._last_target_pub > 0.2:
                self._publish_target()
        elif isinstance(self.task, T.TunnelTask) and self.var_assist.get():
            rx, ry = self.task.reference()
            self.br.publish_reference((rx, ry), self._task_z)   # Recentrar não muda o z
            self._ref_sent = True
        else:                            # assistência desmarcada: solta a referência aqui
            self._ref_recenter()

    # ── atualização visual (≈ 10 Hz) ────────────────────────────────────
    def _live(self):
        return self.br.age('tick') < LIVE_S

    def _state_name(self, live_adm):
        d = self.tick
        if not (live_adm and d):
            return 'SEM DADOS'
        i = int(d['state']) if math.isfinite(d['state']) else -1
        return C.STATES[i] if 0 <= i < len(C.STATES) else 'SEM DADOS'

    def _refresh_real(self, br):
        """Linhas 'Robô real' e 'Célula FA7155' do cartão Conexões."""
        st = None
        self._parada = ''
        if br.age('real') < 1.0:
            try:
                st = json.loads(br.last['real'])
            except (ValueError, KeyError, TypeError):
                st = None
        if st is None:
            self.lbl_env.configure(text='ROBÔ REAL: DESCONHECIDO', bg=th.WARN)
            self.conn['real'].configure(
                text='○ fora do ar',
                fg=th.MUTED)
        else:
            self._parada = st.get('parada') or ''
            if not self.var_ip.get() and st.get('ip'):
                self.var_ip.set(st['ip'])
            ligado = st.get('conectado') or st.get('conectando')
            real, espelho = st.get('conectado'), st.get('modo') != 'SIM_ONLY'
            self.lbl_env.configure(
                text=('⚠ ROBÔ REAL segue o simulado' if real and espelho else
                      'Robô real conectado (parado: SIM_ONLY)' if real else 'SIMULAÇÃO'),
                bg=th.DANGER if real and espelho else th.HEADER)
            self.btn_real.configure(text='Desconectar' if ligado else 'Conectar')
            self.btn_mode.configure(text=f"Modo: {st.get('modo', 'MIRROR')}")
            partes = ['conectado' if st.get('conectado') else
                      'conectando…' if st.get('conectando') else 'DESCONECTADO']
            if st.get('parada'):
                partes.insert(0, st['parada'])
            if st.get('erro') and not st.get('conectado'):
                partes.append(str(st['erro'])[:60])
            if st.get('robot_mode') not in (None, *C.ROBOT_MODES_OK):
                partes.append(f"RobotMode {st['robot_mode']}")
            if st.get('estop'):
                partes.append('E-STOP ACIONADO')
            if st.get('travado'):
                partes.append(st['travado'])
            partes.append('em sincronia' if st.get('sync_ok')
                          else 'FORA DE SINCRONIA %.1f°' % st.get('max_dq_deg', 0.0))
            if st.get('armado'):
                partes.append('ARMADO (o real segue o sim)')
            bom = (st.get('robot_mode') in (None, *C.ROBOT_MODES_OK)
                   and st.get('conectado') and st.get('sync_ok') and not st.get('estop')
                   and not st.get('travado') and not st.get('parada'))
            if st.get('modo') == 'SIM_ONLY' and not ligado and not st.get('erro') and not st.get('parada'):
                self.conn['real'].configure(text='○ SIM_ONLY · desconectado', fg=th.MUTED)
            else:
                self.conn['real'].configure(
                    text=('● ' if bom else '✖ ') + ' · '.join(partes),
                    fg=th.OK if bom else th.DANGER_HV)
        a = br.age('ftraw')
        if a < LIVE_S:
            self.conn['ftraw'].configure(text=f'● recebendo  · {a:.2f} s', fg=th.OK)
        else:
            self.conn['ftraw'].configure(
                text='○ sem dados',
                fg=th.MUTED)

    def _refresh_ui(self):
        br = self.br
        ages = {'adm': br.age('tick'), 'ft': br.age('ft'), 'arm': br.age('joints')}
        live = {k: a < LIVE_S for k, a in ages.items()}
        live['log'] = br.service_ready(C.S_SESSION_START)
        for k, (dot, word) in self.leds.items():
            dot.configure(text='●' if live[k] else '○', fg=th.LED_ON if live[k] else th.LED_OFF)
            word.configure(text='no ar' if live[k] else 'fora do ar')
            det = ''
            if live[k] and k in ages:
                det = f'  · {ages[k]:.2f} s'
            self.conn[k].configure(text=('● no ar' if live[k] else '○ fora do ar') + det,
                                   fg=th.OK if live[k] else th.DANGER_HV)

        self._refresh_real(br)
        self._refresh_jog(live['adm'])

        nome = self._state_name(live['adm'])
        pill_bg, pill_fg, icon, ban_bg, ban_fg = th.STATE_STYLE[nome]
        if nome == 'ABORTADO':
            self._estop_alert = ''
        alerta = self._estop_alert or self._parada
        if alerta and nome != 'ABORTADO':          # fica até o braço confirmar a parada
            ban_bg, ban_fg = th.STATE_STYLE['ABORTADO'][3:]
        self.pill.configure(bg=pill_bg)
        self.lbl_state_icon.configure(text=icon, bg=pill_bg, fg=pill_fg)
        self.lbl_state.configure(text=STATE_LABEL.get(nome, nome), bg=pill_bg, fg=pill_fg)
        self.banner.configure(bg=ban_bg)
        self.lbl_banner.configure(text=f'{icon}  {alerta or BANNER[nome]}', bg=ban_bg, fg=ban_fg,
                                  font=th.FONT_TITLE if nome == 'ABORTADO' or alerta else th.FONT)
        self._set_banner_action(nome)
        d = self.tick
        if nome != 'SEM DADOS':
            self.lbl_fmag.configure(text='%.1f' % math.sqrt(
                d['fx_n'] ** 2 + d['fy_n'] ** 2 + d['fz_n'] ** 2))
            self.lbl_force.configure(text='axial %+.1f N   idade %.2f s' % (
                d['f_axial_n'], d['force_age_s']))
            antes, self.enabled = self.enabled, nome in ('ATIVO', 'SEM_FORCA', 'LIMITE')
            self._enable_done |= antes != self.enabled
            self.moving = self.enabled or nome == 'POSICIONANDO'
        else:
            self.lbl_fmag.configure(text='—')
            self.lbl_force.configure(text='axial —   idade —')
        self._enable_release()
        self.btn_enable.configure(text='Desabilitar' if self.enabled else 'Habilitar')

        p = br.last.get('params')
        if p:
            self.lbl_source.configure(text={'real': 'Célula FA7155', 'sim': 'Simulada'}.get(p.get('force_source'), '?'),
                                      fg=th.TEXT)
            self.lbl_node_params.configure(text='▲ = ainda não aplicado')
            for key, *_r, un in PARAMS:
                v = p.get(key)
                if isinstance(v, (int, float)):
                    dif = abs(v - self.pvar[key].get()) > 1e-9
                    self.pnode[key].configure(text=f"{'▲ ' if dif else ''}nó: {v:g} {un}",
                                              fg=th.WARN if dif else th.MUTED)
            if not self._params_loaded:
                self._params_loaded = True
                self.load_params()
        s = br.last.get('summary')
        if s and s != self._summary_shown:
            self._summary_shown = s
            if not self._rec():          # latched antigo: quem diz se há sessão é o T_SESSION_REC
                self._session_closed()
            self._show_summary(s)
        self._refresh_session()

        tab = self._tab()
        if tab == 3:
            for ch in self.charts:
                ch.redraw()
        elif tab == 2:
            self._draw_game()

    def _rec(self):
        """Pasta em gravação, '' se nenhuma, None se o registrador não responde."""
        return self.br.last.get('rec') if self.br.age('rec') < REC_STALE_S else None

    def _session_closed(self):
        self.session_open = False
        self.lbl_session.configure(text='○ Sessão fechada: não está gravando.', fg=th.MUTED)
        self.lbl_sub.configure(text='Nenhuma sessão de coleta aberta')

    def _refresh_session(self):
        """O que o registrador diz vence o que a tela lembra."""
        rec = self._rec()
        if time.monotonic() - self._session_t < REC_STALE_S:
            return                       # acabou de abrir/encerrar: o aviso ainda é o anterior
        if rec and not self.session_open:     # painel reaberto no meio da sessão
            self.session_open = True
            self.lbl_session.configure(text=f'● Sessão aberta — gravando em {rec}', fg=th.OK)
            self.lbl_sub.configure(text='Sessão aberta · gravando')
        elif rec == '' and self.session_open:
            self._session_closed()
        elif rec is None and self.session_open:
            self.session_open = False
            self.lbl_session.configure(text='✖ GRAVAÇÃO PERDIDA: o registrador não responde. Relance-o '
                                       'e inicie outra sessão.', fg=th.DANGER_HV)
            self.lbl_sub.configure(text='GRAVAÇÃO PERDIDA')
            self._say('GRAVAÇÃO PERDIDA: o registrador não responde.', False)

    def _set_banner_action(self, nome):
        # sem Habilitar aqui: um duplo clique em Reset habilitaria o braço
        act = {'ATIVO': ('Desabilitar', self.disable, 'TButton'),
               'SEM_FORCA': ('Desabilitar', self.disable, 'TButton'),
               'LIMITE': ('Desabilitar', self.disable, 'TButton'),
               'ABORTADO': ('Reset', self.reset, 'TButton')}.get(nome)
        if act == self._banner_action:
            return
        self._banner_action = act
        if act is None:
            self.btn_banner.pack_forget()
        else:
            self.btn_banner.configure(text=act[0], command=act[1], style=act[2])
            # antes do texto: senão uma mensagem longa espreme o botão para fora
            self.btn_banner.pack(side='right', padx=18, pady=4, before=self.lbl_banner)

    def _show_summary(self, s):
        tx = self.txt_summary
        tx.configure(state='normal')
        tx.delete('1.0', 'end')

        def put(o, ind=0):
            pre = '  ' * ind
            if isinstance(o, dict):
                for k, v in o.items():
                    tx.insert('end', f'{pre}{k}:', 'k')
                    if isinstance(v, (dict, list)):
                        tx.insert('end', '\n')
                        put(v, ind + 1)
                    else:
                        tx.insert('end', f' {v}\n')
            elif isinstance(o, list):
                for v in o:
                    tx.insert('end', f'{pre}–', 'k')
                    if isinstance(v, (dict, list)):
                        tx.insert('end', '\n')
                        put(v, ind + 1)
                    else:
                        tx.insert('end', f' {v}\n')
            else:
                tx.insert('end', f'{pre}{o}\n')
        try:
            put(json.loads(s))
        except ValueError:
            tx.insert('end', s)
        tx.configure(state='disabled')

    # ── desenho do jogo ─────────────────────────────────────────────────
    def _draw_game(self):
        cv = self.cv
        cv.delete('dyn')
        w, h = cv.winfo_width(), cv.winfo_height()
        if self.center is None:
            cv.create_text(w / 2, h / 2, fill=th.MUTED, tags='dyn', font=th.FONT_TITLE,
                           justify='center',
                           text='Sem posição do braço\n(o nó rehab_admittance não está no ar)')
            self._update_score(None, 0.0)
            return
        S = self._scale
        task, d = self.task, self.tick
        t = d['t_s'] if d else 0.0
        if isinstance(task, T.CenterOutTask):
            self._draw_center_out(task, t, S)
        elif isinstance(task, T.TunnelTask):
            self._draw_tunnel(task, S)
        elif isinstance(task, T.ReachTask):
            self._draw_reach(task, S)
        if len(self.trail) >= 2:
            pts = [c for _t, x, y in self.trail for c in self._w2s(x, y)]
            cv.create_line(*pts, fill=th.TRAIL, width=3, capstyle='round',
                           joinstyle='round', tags='dyn')
        if self._flash and time.monotonic() - self._flash[0] < 0.8:
            age = time.monotonic() - self._flash[0]
            sx, sy = self._w2s(self._flash[1], self._flash[2])
            rr = (task.radius * S if isinstance(task, T.CenterOutTask) else 20) + 30 * age
            cv.create_oval(sx - rr, sy - rr, sx + rr, sy + rr, outline=self._flash[4],
                           width=3, tags='dyn')
            cv.create_text(sx, sy - rr - 12, text=self._flash[3], fill=self._flash[4],
                           font=th.FONT_TITLE, tags='dyn')
        if d:
            sx, sy = self._w2s(d['x_m'], d['y_m'])
            out = isinstance(task, T.TunnelTask) and not task.done and not task.inside
            cv.create_oval(sx - 10, sy - 10, sx + 10, sy + 10, fill=th.CURSOR,
                           outline=th.DANGER if out else '#ffffff', width=4 if out else 2,
                           tags='dyn')
            if out:
                cv.create_text(sx + 16, sy, anchor='w', text='fora do túnel',
                               fill=th.DANGER_HV, font=th.FONT_TITLE, tags='dyn')
        if task is None or task.done:
            msg = ('Tarefa encerrada. Escolha outra ou clique em Iniciar para repetir.'
                   if task else 'Escolha uma tarefa à esquerda e clique em Iniciar.')
            cv.create_text(w / 2, h - 34, text=msg, fill=th.MUTED, font=th.FONT, tags='dyn')
        self._update_score(task, t)

    def _draw_center_out(self, task, t, S):
        cv = self.cv
        r = task.radius * S
        cx, cy = self._w2s(task.cx, task.cy)
        cv.create_oval(cx - r, cy - r, cx + r, cy + r, outline=th.MUTED, dash=(4, 3), tags='dyn')
        for k, (x, y) in enumerate(task.target_positions):
            sx, sy = self._w2s(x, y)
            res = self._co_result.get(k)
            if res == 'hit':
                cv.create_oval(sx - r, sy - r, sx + r, sy + r, fill=th.HIT_FILL,
                               outline=th.OK, width=2, tags='dyn')
                cv.create_text(sx, sy, text='✔', fill=th.OK, font=th.FONT_TITLE, tags='dyn')
            elif res == 'miss':
                cv.create_oval(sx - r, sy - r, sx + r, sy + r, outline=th.MUTED, width=2, tags='dyn')
                cv.create_text(sx, sy, text='✖', fill=th.MUTED, font=th.FONT_TITLE, tags='dyn')
            else:
                cv.create_oval(sx - r, sy - r, sx + r, sy + r, outline=th.MUTED,
                               dash=(4, 3), tags='dyn')
        if task.done:
            return
        nxt = task.goals[task.i + 1] if task.i + 1 < len(task.goals) else None
        if nxt and nxt[0] == 'ida':
            sx, sy = self._w2s(*nxt[2])
            cv.create_oval(sx - r - 4, sy - r - 4, sx + r + 4, sy + r + 4, outline=th.PRIMARY,
                           width=2, dash=(6, 4), tags='dyn')
            cv.create_text(sx, sy + r + 14, text='próximo', fill=th.PRIMARY,
                           font=th.FONT_SMALL, tags='dyn')
        tg = task.current_target()
        if tg:
            sx, sy = self._w2s(*tg)
            prog = task.hold_progress(t)
            cv.create_oval(sx - r, sy - r, sx + r, sy + r, outline=th.TARGET_LINE, width=3,
                           fill=th.HIT_FILL if prog > 0 else th.TARGET_FILL, tags='dyn')
            if prog > 0:
                cv.create_arc(sx - r - 6, sy - r - 6, sx + r + 6, sy + r + 6, start=90,
                              extent=-359.9 * prog, style='arc', outline=th.OK, width=5, tags='dyn')
            rot = 'VOLTE AO CENTRO' if task.goal[0] == 'volta' else 'ALVO'
            cv.create_text(sx, sy - r - 16, text=rot, fill=th.TARGET_LINE,
                           font=th.FONT_TITLE, tags='dyn')

    def _draw_tunnel(self, task, S):
        cv = self.cv
        wpx = max(4.0, task.width * S)
        pts = [c for p in task.path for c in self._w2s(*p)]
        cv.create_line(*pts, fill=th.TUNNEL_EDGE, width=wpx + 4, capstyle='round',
                       joinstyle='round', tags='dyn')
        cv.create_line(*pts, fill=th.TUNNEL_FILL, width=wpx, capstyle='round',
                       joinstyle='round', tags='dyn')
        done = pts[:2 * (task.idx + 1)]
        if len(done) >= 4:
            cv.create_line(*done, fill=th.TUNNEL_DONE, width=wpx, capstyle='round',
                           joinstyle='round', tags='dyn')
        cv.create_line(*pts, fill=th.MUTED, width=1, dash=(4, 4), tags='dyn')
        ex, ey = self._w2s(*task.path[-1])
        cv.create_oval(ex - 7, ey - 7, ex + 7, ey + 7, fill=th.TEXT, outline='#ffffff', tags='dyn')
        cv.create_text(ex, ey + wpx / 2 + 14, text='FIM', fill=th.TEXT, font=th.FONT_SMALL, tags='dyn')
        sx, sy = self._w2s(*task.path[0])
        cv.create_text(sx, sy - wpx / 2 - 14, text='INÍCIO', fill=th.TEXT, font=th.FONT_SMALL, tags='dyn')
        # Seta de sentido logo após o início.
        k = min(len(task.path) - 1, max(2, len(task.path) // 25))
        cv.create_line(sx, sy, *self._w2s(*task.path[k]), fill=th.TEXT, width=2, arrow='last',
                       arrowshape=(10, 12, 5), tags='dyn')
        if self.var_assist.get() and not task.done:
            rx, ry = self._w2s(*task.reference())
            cv.create_oval(rx - 5, ry - 5, rx + 5, ry + 5, fill=th.OK, outline='#ffffff', tags='dyn')

    def _draw_reach(self, task, S):
        cv = self.cv
        a = task.area
        x0, y0 = self._w2s(a.cx - a.half, a.cy - a.half)
        x1, y1 = self._w2s(a.cx + a.half, a.cy + a.half)
        cv.create_rectangle(x0, y0, x1, y1, outline=th.MUTED, dash=(4, 3), tags='dyn')
        cs = a.cell * S
        for (i, j) in a.cells - self._drawn:     # só as células novas
            sx, sy = self._w2s(a.cx + i * a.cell, a.cy + j * a.cell)
            cv.create_rectangle(sx, sy, sx + cs, sy + cs, fill=th.CELL, outline='', tags='trace')
            self._drawn.add((i, j))
        cv.tag_lower('trace')
        cv.tag_lower('grid')

    def _update_score(self, task, t):
        """Placar em leituras grandes (rótulo, valor) + barra de progresso."""
        if isinstance(task, T.CenterOutTask):
            n = len(task.goals)
            atual = n if task.done else max(task.i, 0) + 1
            vals = (('ACERTOS', task.hits), ('ERROS', task.misses), ('META', f'{atual}/{n}'))
            prog = 100 * (atual - (0 if task.done else 1)) / n
        elif isinstance(task, T.TunnelTask):
            vals = (('PROGRESSO', f'{100 * task.progress:.0f}%'),
                    ('DENTRO', f'{task.inside_pct:.0f}%'), ('SAÍDAS', task.exits))
            prog = 100 * task.progress
        elif isinstance(task, T.ReachTask):
            el = min(task.duration_s, t - task._t0) if task.started else 0.0
            vals = (('ÁREA', f'{task.area.area_m2 * 1e4:.0f} cm²'),
                    ('RESTAM', f'{task.duration_s - el:.0f} s'), ('', ''))
            prog = 100 * el / task.duration_s
        else:
            vals, prog = (('', '—'), ('', ''), ('', '')), 0
        for lab, (cap, v) in zip(self.score, vals):
            lab.caption.configure(text=cap)
            lab.configure(text=str(v))
        self.bar_task.configure(value=prog)
        if task is None:
            self.lbl_score.configure(text='Nenhuma tarefa em andamento.')
        else:
            self.lbl_score.configure(text=(TASK_LABELS.get(self.var_task.get(), '') if not task.done
                                           else 'Tarefa encerrada.'))

    # ── encerramento ────────────────────────────────────────────────────
    def close(self):
        if self._closing:
            return
        self._jog_cancel()
        # sair da tela não pode deixar o braço se movendo (nem um pedido recém-enviado)
        if ((self.moving or self._enable_wait or time.monotonic() - self._jog_t < 1.0)
                and not self._stop_confirmed()
                and not self._confirm('PARAR não confirmado', 'A admitância não confirmou o PARAR: o braço pode '
                                      'estar se movendo. Use a parada de emergência física. Fechar mesmo assim?',
                                      'Fechar mesmo assim')):
            return
        if self.task and not self.task.done:
            self.task_stop()             # TASK_END e alvo NaN antes de sair
        self._closing = True
        self.br.shutdown()
        self.root.destroy()

    def _stop_confirmed(self, limite_s=1.0):
        self.estop()
        t0 = time.monotonic()
        while self._estop_ok is None and time.monotonic() - t0 < limite_s:
            time.sleep(0.02)
            self._drain_results()
        return bool(self._estop_ok)


def main(argv=None):
    from .pack_lock import acquire_or_exit
    acquire_or_exit('rehab_pack')
    from .gui_bridge import GuiBridge
    br = GuiBridge()
    root = tk.Tk()
    app = RehabGui(root, br)
    sig = {'stop': False}
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda *_: sig.update(stop=True))

    def watch():
        if sig['stop']:
            sig['stop'] = False
            app.close()
        if not app._closing:
            root.after(200, watch)
    root.after(200, watch)
    try:
        root.mainloop()
    finally:
        if not app._closing:
            app.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
