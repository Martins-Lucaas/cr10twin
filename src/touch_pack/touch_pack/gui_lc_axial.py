"""
gui_lc_axial.py — aba "Load Cell" da célula axial de 100 kg (XIAO + HX711).

Fatia de `palpation_gui.py`, no mesmo modelo de `gui_loadcell.py` (que é a aba
da FA7155 de 6 eixos): os métodos continuam operando sobre `self`, e o estado
que o resto da GUI lê mora no host.

DUAS CÉLULAS, DUAS ABAS, UMA DE CADA VEZ. `palpation_gui` recebe o parâmetro
`force_sensor` do launch — o MESMO argumento que decide qual receiver sobe — e
monta as sub-abas da célula que está no cabo:

    force_sensor:=load_cell  →  Reading + Calibration  (este arquivo)
    force_sensor:=ft6        →  6 Axes                 (gui_loadcell.py)

Montar as duas seria pior que escolher: a aba de seis eixos com a célula axial
no cabo mostraria seis canais mortos, e o wizard com a FA7155 no cabo ofereceria
calibrar um sensor que já vem calibrado de fábrica.

── Reading ───────────────────────────────────────────────────────────
O que a viga S tem para mostrar é UM número — e é o número que a malha de
segurança consome. O painel mostra os três estágios dele lado a lado (tensão da
ponte → força crua → força pós-tare), porque é a diferença entre eles que
diagnostica: crua ≠ pós-tare denuncia zero deslocado sem precisar descarregar a
ponteira, e tensão parada com força variando denuncia calibração trocada.

── Calibration ───────────────────────────────────────────────────────
POR QUE ESTA ABA EXISTE E A DA FA7155 NÃO PRECISA DELA. A célula de 6 eixos
sai de fábrica entregando newton; a viga S entrega MILIVOLT, e a reta que
converte um no outro é da peça — muda com a célula, com o HX711 e com o aperto
do parafuso. Sem calibrar, `/load_cell/force_net` não existe: o
`force_receiver` recusa publicar (ver o gate `calibrated and tare_done` lá).

O QUE O WIZARD FAZ. Coleta o V₀ (célula vazia) e pares (massa padrão, tensão
média), ajusta `v = slope·F + V₀` com o V₀ FIXO, e grava em
`sensors/load_cell_calib.json` — o arquivo VERSIONADO do repo, o mesmo que o
`force_receiver` lê (`constants.LC_CALIB_FILE`). O receiver o relê sozinho
(timer de 5 s, por mtime), então a calibração vale sem derrubar o nó — que é
dono da porta e não pode ser reiniciado de leve.

ELE ABRE COM A CALIBRAÇÃO EM VIGOR DENTRO. Pontos, V₀ e reta são lidos do
arquivo na montagem da aba (`_lc_load_previous`). Abrir em branco sobre um
sistema calibrado dizia "No fit yet" e convidava a refazer sete massas sem
precisar; e, pior, permitia reajustar por um método diferente do que produziu
a reta em uso. Só o slope é ajustado, e o V₀ entra medido — o porquê está em
`constants.lc_fit_slope`, com o número: nos 7 pontos desta célula o ajuste
livre de dois parâmetros move a escala de força em 0,8 %.

PROCEDIMENTO, e ele NÃO é opcional: a célula tem de estar APONTADA PARA CIMA,
com as massas apoiadas sobre ela. É isso que define "compressão positiva" para
o sistema inteiro — o slope absorve a polaridade da fiação, então uma
calibração feita em tração inverte o sinal da força e a parada de segurança
de 15 N passa a olhar para o lado errado.

TRÊS GUARDAS ANTES DE GRAVAR, todas por erro já visto na bancada:

  1. V₀ capturado, e mínimo de LC_CALIB_MIN_POINTS pontos COM MASSA. Dois
     pontos sempre dão uma reta perfeita; com três já existe resíduo, e é o
     resíduo que denuncia massa digitada errada.
  2. Slope dentro de ±LC_SLOPE_TOL_FRAC do nominal da placa (2 mV/V, 100 kg).
     Pega o erro de digitar grama onde se pede quilo: a reta continua
     lindíssima, só a escala do mundo muda por 1000.
  3. Duas massas iguais são recusadas na coleta — repetir o mesmo ponto não
     acrescenta reta nenhuma e ainda mascara o resíduo.

O V₀ DA RETA NÃO É O ZERO DE OPERAÇÃO. O V₀ é a tensão de repouso MEDIDA na
calibração — a origem física da reta, gravada no arquivo. O zero de operação é
o TARE, que o `force_receiver` refaz a cada partida e que tira o peso do que
estiver montado na hora. Os dois convivem e nenhum substitui o outro: sem V₀
não há newton, sem tare o newton está deslocado pelo peso da ferramenta.
"""
from __future__ import annotations

import json
import os
import time
import tkinter as tk

from std_msgs.msg import Empty, Float32
from .constants import (
    CONFIG_DIR, CONTACT_ON_N, FORCE_ABORT_LIMIT_N, LC_CALIB_FILE,
    LC_CALIB_SHARED_SOURCES, LC_CALIB_SOURCE, LC_CALIB_MIN_POINTS,
    lc_calib_fingerprint,
    LC_FS_VOLTAGE_V, LC_FW_VOLTAGE_SCALE, LC_NOMINAL_RATE_HZ,
    LC_NOMINAL_V_PER_N, LC_RATED_LOAD_KG, LC_SLOPE_TOL_FRAC, G_N_PER_KG,
    lc_fit_slope, lc_force_n, lc_load_calibration,
)
from .ui_helpers import (
    PANEL, TEXT, TEXT_MUTED, TEXT_DIM,
    PRIMARY, PRIMARY_HV, OK, WARN, DANGER, BORDER,
    FONT_BIG, FONT_LBL, FONT_SMALL, FONT_MONO, FONT_MONO_S,
)

# Quanto tempo de sinal entra em UM ponto. 1,5 s a 10 Hz são 15 amostras: o
# suficiente para a média valer mais que o ruído sem o operador ter de segurar
# a massa parada por meia hora. O botão fica desabilitado durante a coleta,
# então o número também é o tempo que a tela fica "ocupada".
_CAPTURE_S = 1.5
# Idade máxima da última tensão para a aba se considerar viva.
_STALE_S = 3.0
# Barra da força: escala de 0 ao limite de ABORTO, que é a única referência que
# significa alguma coisa numa célula de um eixo só. Escalar pelo fundo de escala
# (980 N) faria uma palpação de 1 N ocupar 0,1 % da barra.
_BAR_W = 320
_BAR_H = 18


class LcAxialMixin:
    """Mixin de `PalpationGUI` — a aba da célula axial de 100 kg."""

    # ── Estado + callback ─────────────────────────────────────────────
    # Divisão de estado: o que a CALLBACK do ROS toca (`_lc_voltage`,
    # `_lc_voltage_ts`, `_lc_capture`, `_lc_arrivals`) nasce no host, porque a
    # assinatura é criada no __init__ dele e o rclpy roda numa thread — a
    # primeira amostra pode chegar muito antes de qualquer aba existir. O que
    # só o wizard toca nasce aqui, quando a aba é montada.
    def _lc_calib_init(self) -> None:
        # Pontos com massa: (massa_kg, forca_N, tensao_V).
        self._lc_calib_points: list[tuple[float, float, float]] = []
        # V₀: a tensão de repouso, MEDIDA e não ajustada. Fica fora da lista
        # de pontos porque não é um ponto como os outros — é o parâmetro que
        # o ajuste segura fixo (ver constants.lc_fit_slope).
        self._lc_calib_zero: float | None = None
        self._lc_calib_fit: tuple[float, float, float] | None = None
        self._lc_calib_path = LC_CALIB_FILE

    def _lc_load_previous(self) -> None:
        """Traz a calibração EM VIGOR do arquivo para dentro do wizard.

        A aba abria em branco — "No fit yet" — enquanto o receiver já regulava
        contra uma reta de 7 pontos gravada no repo. Quem abrisse concluiria
        que não havia calibração e refaria as sete massas sem precisar.

        Vêm os PONTOS junto com a reta, não só o slope: são eles que permitem
        acrescentar uma massa a uma calibração existente em vez de recomeçar.
        E o resíduo é RECALCULADO aqui em vez de lido do arquivo, porque um
        arquivo escrito à mão pode ter os dois em desacordo — quem manda é o
        que os pontos dizem.
        """
        cal = lc_load_calibration(self._lc_calib_path)
        if cal is None:
            self._lc_refresh_fit_label(
                'No calibration file yet — capture the zero with the cell '
                'empty, then one point per standard mass.', TEXT_DIM)
            return
        slope, v0, pontos = cal
        self._lc_calib_zero = v0
        self._lc_calib_points = list(pontos)
        pior = None
        if pontos:
            ajuste = lc_fit_slope([(f, v) for _m, f, v in pontos], v0)
            if ajuste is not None:
                pior = ajuste[1]
        self._lc_calib_fit = (slope, v0, pior or 0.0)
        self._refresh_lc_points()
        origem = (f'{len(pontos)} points' if pontos
                  else 'no points stored — the line came without them')
        self._lc_refresh_fit_label(
            f'In force [{lc_calib_fingerprint(self._lc_calib_path)}], '
            f'loaded from {self._lc_calib_path} (source: {LC_CALIB_SOURCE}):\n'
            f'slope     = {slope:+.6e} V/N\n'
            f'V0 (zero) = {v0:+.6e} V   (measured, held fixed by the fit)\n'
            + (f'worst residual = {pior * 1e3:.1f} mN over {origem}'
               if pior is not None else origem)
            + '\n"Clear points" starts a fresh set; capturing a new mass adds '
              'to this one.', OK)

    def _lc_refresh_fit_label(self, texto: str, cor: str) -> None:
        """O rótulo do ajuste só existe depois do card — e a carga do arquivo
        roda na montagem da aba, que pode chamar antes."""
        lbl = getattr(self, '_lc_fit_lbl', None)
        if lbl is not None:
            lbl.configure(text=texto, fg=cor)

    def _cb_lc_voltage(self, msg: Float32) -> None:
        """`/load_cell/voltage` — tensão da ponte JÁ filtrada, publicada pelo
        force_receiver. É a mesma grandeza que o wizard grava no JSON, e tem
        de ser: calibrar contra o sinal cru e medir com o filtrado deslocaria
        a reta pelo atraso do One-Euro."""
        agora = time.time()
        with self._lock:
            self._lc_voltage = float(msg.data)
            self._lc_voltage_ts = agora
            # Taxa MEDIDA no host. O firmware carimba `t_us`, mas ele não diz
            # quantas amostras chegaram — e é a cadência de CHEGADA que
            # responde "o pino RATE está em GND ou em DVDD?".
            self._lc_arrivals.append(agora)
            cap = self._lc_capture
            if cap is not None:
                cap[1].append(float(msg.data))

    def _cb_lc_calibrated(self, msg) -> None:
        """`/load_cell/calibrated` — o receiver dizendo se tem reta.

        Vem do NÓ e não do wizard de propósito: o que importa para quem está
        olhando a leitura é o que o driver carregou do arquivo, não o que
        alguém ajustou nesta janela e talvez não gravou.
        """
        with self._lock:
            self._lc_calibrated = bool(msg.data)

    # ══════════════════════════════════════════════════════════════════
    # ABA "READING" — a leitura ao vivo da viga S
    # ══════════════════════════════════════════════════════════════════
    def _build_lc_reading_tab(self, root: tk.Frame) -> None:
        self._build_lc_live_card(root)
        self._build_lc_zero_card(root)
        self._build_lc_link_card(root)
        self.root.after(100, self._refresh_lc_reading)

    def _build_lc_live_card(self, root: tk.Frame) -> None:
        # Com force_source:=sim quem publica é o sim_force_bridge, e o número
        # é o wrench do plugin FT do Gazebo — não a viga S. Sem esta marca a
        # tela mostrava ~5,5 N (peso da pilha abaixo da célula) com a célula
        # física desligada, indistinguível de uma leitura real.
        titulo = '100 kg axial cell — live'
        if self._force_source == 'sim':
            titulo += '   ⚠ SIMULADA (Gazebo)'
        card = self._card(root, titulo, expand=False)

        # O número grande é a força PÓS-TARE: é ela que o explorer regula e a
        # que o corte de 15 N observa. Os outros dois estágios ficam ao lado,
        # menores, porque servem para diagnosticar e não para operar.
        self._lc_net_lbl = tk.Label(card, text='—   N', font=FONT_BIG,
                                    bg=PANEL, fg=TEXT_DIM, anchor='w')
        self._lc_net_lbl.pack(fill='x', pady=(6, 0))
        self._lc_net_status = tk.Label(
            card, text='waiting for /load_cell/force_net', font=FONT_SMALL,
            bg=PANEL, fg=TEXT_DIM, anchor='w')
        self._lc_net_status.pack(fill='x')

        self._lc_bar = tk.Canvas(card, width=_BAR_W, height=_BAR_H,
                                 bg=PANEL, highlightthickness=0, bd=0)
        self._lc_bar.pack(anchor='w', pady=(8, 2))
        self._lc_bar_items = {
            'trilho': self._lc_bar.create_rectangle(
                0, 0, _BAR_W, _BAR_H, fill=BORDER, outline=''),
            'nivel': self._lc_bar.create_rectangle(
                0, 0, 0, _BAR_H, fill=OK, outline=''),
            # Marca do limiar de CONTATO: sem ela a barra não diz onde
            # "encostou" começa, que é a única fronteira útil na parte baixa.
            'contato': self._lc_bar.create_line(
                0, 0, 0, _BAR_H, fill=TEXT_MUTED, width=1),
        }
        x_ct = _BAR_W * CONTACT_ON_N / FORCE_ABORT_LIMIT_N
        self._lc_bar.coords(self._lc_bar_items['contato'],
                            x_ct, 0, x_ct, _BAR_H)
        tk.Label(card,
                 text=(f'0 … {FORCE_ABORT_LIMIT_N:.0f} N (abort limit)  ·  '
                       f'tick at the {CONTACT_ON_N:.2f} N contact threshold'),
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w').pack(
            fill='x')

        tk.Frame(card, bg=BORDER, height=1).pack(fill='x', pady=(10, 8))

        grid = tk.Frame(card, bg=PANEL)
        grid.pack(fill='x')
        for c in range(3):
            grid.columnconfigure(c, weight=1, uniform='lc_live')
        self._lc_live_lbls = {}
        for col, (key, title, hint) in enumerate((
                ('v',   'Bridge voltage',
                 'what the HX711 measures'),
                ('raw', 'Force before tare',
                 'same reading, no zero applied'),
                ('kgf', 'Net force in kgf',
                 'for comparing against the standard masses'))):
            cell = tk.Frame(grid, bg=PANEL)
            cell.grid(row=0, column=col, sticky='ew', padx=(0, 10))
            tk.Label(cell, text=title, font=FONT_SMALL, bg=PANEL,
                     fg=TEXT_DIM, anchor='w').pack(fill='x')
            lbl = tk.Label(cell, text='—', font=FONT_MONO, bg=PANEL,
                           fg=TEXT_DIM, anchor='w')
            lbl.pack(fill='x')
            tk.Label(cell, text=hint, font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
                     anchor='w', justify='left', wraplength=230).pack(
                fill='x')
            self._lc_live_lbls[key] = lbl

        tk.Label(card,
                 text=('The gap between "before tare" and the big number IS '
                       'the current zero — it drifts without the cell being '
                       'touched, and watching it is how you catch drift '
                       'without unloading the tip.'),
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
                 justify='left', wraplength=780).pack(fill='x', pady=(8, 0))

    def _build_lc_zero_card(self, root: tk.Frame) -> None:
        """Os DOIS zeros, lado a lado e nomeados — porque eles não são a mesma
        coisa e trocá-los custa uma medição inteira."""
        card = self._card(root, 'Zeroing', expand=False)

        row = tk.Frame(card, bg=PANEL)
        row.pack(fill='x', pady=(6, 4))
        tk.Button(row, text='⊘  Tare (host)', command=self._lc_do_tare,
                  bg=PRIMARY, fg='white', activebackground=PRIMARY_HV,
                  activeforeground='white', font=FONT_LBL, relief='flat',
                  bd=0, padx=14, pady=6, cursor='hand2').pack(side='left')
        tk.Button(row, text="↻  Re-zero the firmware ('Z')",
                  command=self._lc_do_rezero,
                  bg=PANEL, fg=TEXT, activebackground=PANEL,
                  activeforeground=TEXT, font=FONT_LBL, relief='flat', bd=0,
                  padx=14, pady=6, cursor='hand2',
                  highlightthickness=1, highlightbackground=BORDER).pack(
            side='left', padx=(10, 0))
        self._lc_tare_state_lbl = tk.Label(
            row, text='tare: —', font=FONT_LBL, bg=PANEL, fg=TEXT_DIM)
        self._lc_tare_state_lbl.pack(side='right')

        tk.Label(
            card,
            text=('Tare (host) is a software offset living in force_receiver: '
                  'it is redone at every start and vanishes when the node '
                  'restarts. Re-zero sends the byte Z down the wire and makes '
                  'the MCU re-collect the bridge offset — the only one of the '
                  'two that removes THERMAL drift, and the firmware stops '
                  'transmitting until it locks. Both need the cell unloaded.'),
            font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
            justify='left', wraplength=780).pack(fill='x', pady=(6, 0))

    def _build_lc_link_card(self, root: tk.Frame) -> None:
        card = self._card(root, 'XIAO link and calibration', expand=False)

        grid = tk.Frame(card, bg=PANEL)
        grid.pack(fill='x', pady=(6, 2))
        for c in range(4):
            grid.columnconfigure(c, weight=1, uniform='lc_link')
        self._lc_link_lbls = {}
        for col, (key, title) in enumerate((
                ('board', 'Board'),
                ('rate',  'Measured rate'),
                ('age',   'Last sample'),
                # Os oito hex identificam a RETA em uso. Duas máquinas com a
                # mesma impressão medem igual — é assim que se confere "a
                # mesma calibração em qualquer computador" sem abrir arquivo.
                ('calib', 'Calibration'))):
            cell = tk.Frame(grid, bg=PANEL)
            cell.grid(row=0, column=col, sticky='ew', padx=(0, 10))
            tk.Label(cell, text=title, font=FONT_SMALL, bg=PANEL,
                     fg=TEXT_DIM, anchor='w').pack(fill='x')
            lbl = tk.Label(cell, text='—', font=FONT_LBL, bg=PANEL,
                           fg=TEXT_DIM, anchor='w')
            lbl.pack(fill='x')
            self._lc_link_lbls[key] = lbl

        self._lc_link_note = tk.Label(
            card, text='', font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
            anchor='w', justify='left', wraplength=780)
        self._lc_link_note.pack(fill='x', pady=(6, 0))

    def _lc_do_rezero(self) -> None:
        """PEDE o re-zero. Quem escreve o 'Z' no fio é o force_receiver, e
        ele pode falhar (porta fechada) — a confirmação vem de volta por
        `/load_cell/tare_result`, tratada em `_cb_lc_tare_result`. Publicar e
        já anunciar sucesso era verdade sobre o tópico e mentira sobre o
        fio."""
        self._lc_rezero_pub.publish(Empty())
        self._set_status(
            "Re-zero requested — waiting for the receiver to put 'Z' on the "
            'wire. Keep the cell unloaded.', WARN)

    def _refresh_lc_reading(self) -> None:
        """Tick do painel de leitura, 10 Hz."""
        agora = time.time()
        with self._lock:
            f_net = self._lc_force_net
            f_ts = self._lc_force_net_ts
            f_raw = self._lc_force_raw
            tared = self._lc_tare_done
            v = self._lc_voltage
            v_ts = self._lc_voltage_ts
            calibrado = self._lc_calibrated
            # Janela de 2 s: a taxa nominal é 10 Hz, então uma janela curta
            # daria um número que pula de 9 para 11 a cada tick.
            while self._lc_arrivals and (agora - self._lc_arrivals[0]) > 2.0:
                self._lc_arrivals.popleft()
            n_arr = len(self._lc_arrivals)

        if not self._tab_visible(getattr(self, '_lc_tab_frame', None)):
            self.root.after(200, self._refresh_lc_reading)
            return

        vivo = v_ts > 0.0 and (agora - v_ts) < _STALE_S
        tem_forca = f_ts > 0.0 and (agora - f_ts) < _STALE_S

        if not tem_forca:
            self._lc_net_lbl.configure(text='—   N', fg=TEXT_DIM)
            if not vivo:
                falta = 'waiting for /load_cell/force_net'
            elif not calibrado:
                falta = ('bridge alive but no calibration loaded — fit and '
                         'save one in the Calibration tab')
            else:
                falta = 'bridge alive and calibrated — waiting for the tare'
            self._lc_net_status.configure(
                text=falta, fg=WARN if vivo else TEXT_DIM)
            self._lc_bar.coords(self._lc_bar_items['nivel'], 0, 0, 0, _BAR_H)
        else:
            if not tared:
                cor, estado = WARN, 'tare not done'
            elif f_net > FORCE_ABORT_LIMIT_N * 0.9:
                cor, estado = DANGER, f'near the limit ({FORCE_ABORT_LIMIT_N:.0f} N)'
            elif f_net >= CONTACT_ON_N:
                cor, estado = OK, 'in contact'
            else:
                cor, estado = TEXT_MUTED, 'no contact'
            self._lc_net_lbl.configure(text=f'{f_net:+7.3f}   N', fg=cor)
            self._lc_net_status.configure(text=estado, fg=cor)
            frac = min(max(f_net / FORCE_ABORT_LIMIT_N, 0.0), 1.0)
            self._lc_bar.coords(self._lc_bar_items['nivel'],
                                0, 0, _BAR_W * frac, _BAR_H)
            self._lc_bar.itemconfigure(self._lc_bar_items['nivel'], fill=cor)

        self._lc_live_lbls['v'].configure(
            text=(f'{v * 1e3:+.4f} mV' if vivo else '—'),
            fg=TEXT if vivo else TEXT_DIM)
        self._lc_live_lbls['raw'].configure(
            text=(f'{f_raw:+.3f} N' if tem_forca else '—'),
            fg=TEXT if tem_forca else TEXT_DIM)
        self._lc_live_lbls['kgf'].configure(
            text=(f'{f_net / G_N_PER_KG:+.4f} kgf' if tem_forca else '—'),
            fg=TEXT if tem_forca else TEXT_DIM)
        self._lc_tare_state_lbl.configure(
            text=f'tare: {"done" if tared else "not done"}',
            fg=OK if tared else WARN)

        taxa = n_arr / 2.0
        self._lc_link_lbls['board'].configure(
            text='ONLINE' if vivo else 'OFFLINE', fg=OK if vivo else TEXT_DIM)
        self._lc_link_lbls['rate'].configure(
            text=(f'{taxa:.1f} Hz' if vivo else '—'),
            fg=TEXT if vivo else TEXT_DIM)
        self._lc_link_lbls['age'].configure(
            text=(f'{agora - v_ts:.1f} s' if v_ts > 0.0 else 'never'),
            fg=TEXT if vivo else TEXT_DIM)
        fp = lc_calib_fingerprint(self._lc_calib_path) if calibrado else ''
        self._lc_link_lbls['calib'].configure(
            text=(fp or ('LOADED' if calibrado else 'MISSING')),
            fg=OK if calibrado else WARN)

        if not vivo:
            self._lc_link_note.configure(
                text=('No samples from the XIAO. The board must be on the USB '
                      'cable — there is no network fallback — and '
                      'force_receiver owns the port, so nothing else may have '
                      'it open (the PlatformIO monitor is the usual culprit).'),
                fg=WARN)
        elif taxa < LC_NOMINAL_RATE_HZ * 0.5:
            self._lc_link_note.configure(
                text=(f'Arriving at {taxa:.1f} Hz against the '
                      f'{LC_NOMINAL_RATE_HZ:.0f} Hz nominal. Below half, this '
                      'is not the RATE pin — it is samples being lost on the '
                      'way (cable, hub, or DOUT floating).'),
                fg=WARN)
        else:
            self._lc_link_note.configure(
                text=(f'HX711 RATE pin: GND = 10 Hz, DVDD = 80 Hz. At 10 Hz '
                      'no filter tuning buys a fast response — median-of-3 '
                      'alone costs 100 ms.'),
                fg=TEXT_DIM)
        self.root.after(100, self._refresh_lc_reading)

    # ── A aba ─────────────────────────────────────────────────────────
    def _build_lc_calibration_tab(self, root: tk.Frame) -> None:
        self._lc_calib_init()
        self._build_lc_calib_live_card(root)
        self._build_lc_calib_points_card(root)
        self._build_lc_calib_fit_card(root)
        # Depois dos cards: a carga escreve na tabela e no rótulo do ajuste.
        self._lc_load_previous()
        self.root.after(120, self._refresh_lc_calib)

    def _build_lc_calib_live_card(self, root: tk.Frame) -> None:
        card = self._card(root, 'Bridge voltage — live', expand=False)

        tk.Label(
            card,
            text=('Point the cell UP and rest the standard masses on it. The '
                  'calibration is done in COMPRESSION: that is what makes '
                  'positive force mean compression everywhere downstream, '
                  'including the 15 N abort.'),
            font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
            justify='left', wraplength=780).pack(fill='x', pady=(6, 8))

        grid = tk.Frame(card, bg=PANEL)
        grid.pack(fill='x')
        for c in range(3):
            grid.columnconfigure(c, weight=1, uniform='lc_cal')
        self._lc_calib_lbls = {}
        for col, (key, title) in enumerate((
                ('v',    'Voltage (bridge × PGA)'),
                ('f',    'Force with the loaded fit'),
                ('age',  'Last sample'))):
            cell = tk.Frame(grid, bg=PANEL)
            cell.grid(row=0, column=col, sticky='ew', padx=(0, 10))
            tk.Label(cell, text=title, font=FONT_SMALL, bg=PANEL,
                     fg=TEXT_DIM, anchor='w').pack(fill='x')
            lbl = tk.Label(cell, text='—', font=FONT_BIG, bg=PANEL,
                           fg=TEXT_DIM, anchor='w')
            lbl.pack(fill='x')
            self._lc_calib_lbls[key] = lbl

        self._lc_calib_note = tk.Label(
            card, text='', font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
            anchor='w', justify='left', wraplength=780)
        self._lc_calib_note.pack(fill='x', pady=(6, 0))

    def _build_lc_calib_points_card(self, root: tk.Frame) -> None:
        card = self._card(root, 'Calibration points')

        row = tk.Frame(card, bg=PANEL)
        row.pack(fill='x', pady=(4, 8))
        tk.Label(row, text='Mass on the cell (kg)', font=FONT_LBL, bg=PANEL,
                 fg=TEXT).pack(side='left')
        self._lc_mass_var = tk.StringVar(value='')
        tk.Entry(row, textvariable=self._lc_mass_var, width=10,
                 font=FONT_MONO, bg=PANEL, fg=TEXT,
                 insertbackground=TEXT, relief='flat',
                 highlightthickness=1, highlightbackground=BORDER).pack(
            side='left', padx=(8, 12))
        self._lc_capture_btn = tk.Button(
            row, text=f'⌷  Capture point ({_CAPTURE_S:.1f} s)',
            command=self._lc_start_capture,
            bg=PRIMARY, fg='white', activebackground=PRIMARY_HV,
            activeforeground='white', font=FONT_LBL, relief='flat', bd=0,
            padx=14, pady=6, cursor='hand2')
        self._lc_capture_btn.pack(side='left')
        tk.Button(row, text='Clear points', command=self._lc_clear_points,
                  bg=PANEL, fg=TEXT_MUTED, activebackground=PANEL,
                  activeforeground=TEXT, font=FONT_SMALL, relief='flat',
                  bd=0, padx=10, pady=6, cursor='hand2').pack(
            side='right')

        tk.Label(card,
                 text=('Mass 0 captures the ZERO (V0), not a point: the fit '
                       'holds V0 fixed rather than fitting it, because a '
                       'measured no-load average is better determined than '
                       'anything a two-parameter regression can infer from '
                       'the loaded points. Capture it with the cell empty '
                       'before any mass; recapturing replaces it.'),
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
                 justify='left', wraplength=780).pack(fill='x', pady=(0, 6))

        self._lc_points_txt = tk.Text(
            card, height=9, font=FONT_MONO_S, bg=PANEL, fg=TEXT_MUTED,
            relief='flat', highlightthickness=1, highlightbackground=BORDER,
            wrap='none')
        self._lc_points_txt.pack(fill='both', expand=True)
        self._lc_points_txt.configure(state='disabled')

    def _build_lc_calib_fit_card(self, root: tk.Frame) -> None:
        card = self._card(root, 'Fit and save', expand=False)

        self._lc_fit_lbl = tk.Label(
            card, text='No fit yet.', font=FONT_MONO, bg=PANEL, fg=TEXT_DIM,
            anchor='w', justify='left', wraplength=780)
        self._lc_fit_lbl.pack(fill='x', pady=(4, 8))

        row = tk.Frame(card, bg=PANEL)
        row.pack(fill='x')
        tk.Button(row, text='Fit', command=self._lc_do_fit,
                  bg=PANEL, fg=TEXT, activebackground=PANEL,
                  activeforeground=TEXT, font=FONT_LBL, relief='flat', bd=0,
                  padx=14, pady=6, cursor='hand2',
                  highlightthickness=1, highlightbackground=BORDER).pack(
            side='left')
        self._lc_save_btn = tk.Button(
            row, text='💾  Fit and save', command=self._lc_save_calibration,
            bg=PRIMARY, fg='white', activebackground=PRIMARY_HV,
            activeforeground='white', font=FONT_LBL, relief='flat', bd=0,
            padx=14, pady=6, cursor='hand2')
        self._lc_save_btn.pack(side='left', padx=(10, 0))
        tk.Label(row, text=f'→ {self._lc_calib_path}',
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM).pack(
            side='left', padx=(12, 0))

        tk.Label(card,
                 text=('The receiver reloads the file on its own (5 s timer, '
                       'by mtime) — no need to restart it, and restarting it '
                       'would drop the serial port it owns.'),
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
                 justify='left', wraplength=780).pack(fill='x', pady=(8, 0))

    # ── Coleta ────────────────────────────────────────────────────────
    def _lc_start_capture(self) -> None:
        """Arma a janela de coleta. A média é feita no callback da tensão, e
        não lendo o último valor N vezes daqui: o Tk roda a 8 Hz e leria a
        MESMA amostra várias vezes, o que reduz ruído nenhum."""
        try:
            massa = float(self._lc_mass_var.get().replace(',', '.'))
        except ValueError:
            self._set_status('Mass must be a number in kilograms.', WARN)
            return
        if massa < 0.0 or massa > LC_RATED_LOAD_KG:
            self._set_status(
                f'Mass out of range: the cell is rated for '
                f'{LC_RATED_LOAD_KG:.0f} kg.', WARN)
            return
        # Massa 0 não é um ponto: é o V₀, e recapturá-lo SUBSTITUI o anterior
        # (é a operação normal — o zero deriva, os pontos não).
        if massa > 0.0 and any(abs(m - massa) < 1e-9
                               for m, _f, _v in self._lc_calib_points):
            self._set_status(
                f'{massa:g} kg is already in the table — a repeated point '
                'adds no line and hides the residual. Use a different mass.',
                WARN)
            return
        with self._lock:
            ts = self._lc_voltage_ts
        if ts <= 0.0 or (time.time() - ts) > _STALE_S:
            self._set_status(
                'No voltage from the cell — is force_receiver up and is the '
                'XIAO on the USB cable?', WARN)
            return
        with self._lock:
            self._lc_capture = (time.time() + _CAPTURE_S, [])
        self._lc_capture_btn.configure(state='disabled')
        self._set_status(
            f'Capturing {massa:g} kg — hold it still…', WARN)

    def _lc_finish_capture(self, amostras: list[float]) -> None:
        self._lc_capture_btn.configure(state='normal')
        if len(amostras) < 3:
            self._set_status(
                f'Only {len(amostras)} samples in {_CAPTURE_S:.1f} s — the '
                'cell is too slow or the link dropped. Point discarded.',
                WARN)
            return
        try:
            massa = float(self._lc_mass_var.get().replace(',', '.'))
        except ValueError:
            return
        v = sum(amostras) / len(amostras)
        if abs(v) > LC_FS_VOLTAGE_V:
            self._set_status(
                'Bridge saturated — check the 4-wire wiring before '
                'calibrating.', DANGER)
            return
        self._lc_calib_fit = None
        if massa == 0.0:
            self._lc_calib_zero = v
            self._refresh_lc_points()
            self._set_status(
                f'Zero captured: V0 = {v * 1e3:+.4f} mV '
                f'({len(amostras)} samples). The fit holds it fixed.', OK)
            return
        self._lc_calib_points.append((massa, massa * G_N_PER_KG, v))
        self._lc_calib_points.sort()
        self._refresh_lc_points()
        self._set_status(
            f'Point captured: {massa:g} kg → {v * 1e3:+.4f} mV '
            f'({len(amostras)} samples).', OK)

    def _lc_clear_points(self) -> None:
        """Recomeça do zero — inclusive o V₀. Manter o V₀ de uma calibração
        antiga sob pontos novos misturaria duas sessões de bancada, que é
        justamente o que "clear" existe para não deixar acontecer."""
        self._lc_calib_points = []
        self._lc_calib_zero = None
        self._lc_calib_fit = None
        self._refresh_lc_points()
        self._lc_refresh_fit_label(
            'Cleared. Capture the zero with the cell empty, then one point '
            'per standard mass.', TEXT_DIM)

    # ── Ajuste ────────────────────────────────────────────────────────
    # A conta em si mora em `constants.lc_fit_slope`: o V₀ é MEDIDO e fica
    # fixo, e só o slope é ajustado. O porquê está lá — em resumo, deixar o
    # V₀ flutuar num ajuste de dois parâmetros contradiz a medição e move a
    # escala de força em 0,8 % nos pontos que vieram com esta célula.
    def _lc_do_fit(self) -> tuple[float, float, float] | None:
        pts = self._lc_calib_points
        v0 = self._lc_calib_zero
        if v0 is None:
            self._lc_refresh_fit_label(
                'No zero captured. Type 0 as the mass, with the cell empty, '
                'and capture — the fit holds V0 fixed, so it needs the '
                'measured one.', WARN)
            return None
        if len(pts) < LC_CALIB_MIN_POINTS:
            self._lc_refresh_fit_label(
                f'{len(pts)} loaded point(s) — at least '
                f'{LC_CALIB_MIN_POINTS} are needed. Two points always give a '
                'perfect line; the third is what produces a residual, and the '
                'residual is what catches a mistyped mass.', WARN)
            return None
        fit = lc_fit_slope([(f, v) for _m, f, v in pts], v0)
        if fit is None:
            self._lc_refresh_fit_label(
                'Degenerate fit — the points carry no force span.', DANGER)
            return None
        slope, pior = fit
        intercept = v0
        desvio = abs(abs(slope) - LC_NOMINAL_V_PER_N) / LC_NOMINAL_V_PER_N
        linhas = [
            f'slope     = {slope:+.6e} V/N   '
            f'({desvio * 100:.1f} % from the {LC_NOMINAL_V_PER_N:.4e} nominal '
            f'of a {LC_RATED_LOAD_KG:.0f} kg / 2 mV/V cell)',
            f'V0 (zero) = {intercept:+.6e} V   (measured, held fixed)',
            f'worst residual = {pior * 1e3:.1f} mN over {len(pts)} points',
        ]
        if desvio > LC_SLOPE_TOL_FRAC:
            linhas.append(
                'REFUSED: the slope is off the plate by more than '
                f'{LC_SLOPE_TOL_FRAC * 100:.0f} %. The classic cause is a '
                'mass typed in grams where kilograms were asked — the line '
                'still looks perfect, only the scale of everything changes.')
            self._lc_refresh_fit_label('\n'.join(linhas), DANGER)
            self._lc_calib_fit = None
            return None
        if slope < 0.0:
            linhas.append(
                'Negative slope: the bridge wiring is inverted relative to '
                'the loading direction. Harmless — the sign is absorbed here '
                'and compression stays positive — as long as the masses were '
                'really resting ON the cell.')
        self._lc_refresh_fit_label('\n'.join(linhas), OK)
        self._lc_calib_fit = (slope, intercept, pior)
        return self._lc_calib_fit

    def _lc_save_calibration(self) -> None:
        fit = self._lc_do_fit()
        if fit is None:
            self._set_status('Fit refused — nothing was written.', WARN)
            return
        slope, intercept, pior = fit
        data = {
            'slope': slope,
            'intercept': intercept,
            # Alias histórico do intercepto: arquivos antigos trazem só este
            # nome, e o force_receiver aceita os dois. Gravar os dois mantém
            # um JSON novo legível por qualquer versão.
            'zero_voltage': intercept,
            # A MESMA escala counts→V do firmware: é ela que faz o volt
            # do arquivo significar o volt do fio.
            'voltage_scale': LC_FW_VOLTAGE_SCALE,
            'voltage_offset': 0.0,
            'load_direction': 'compression',
            'n_points': len(self._lc_calib_points),
            'fit': 'slope only, V0 held fixed at the measured zero',
            'max_residual_n': pior,
            'points': [{'mass_kg': m, 'force_n': f, 'v_sensor': v}
                       for m, f, v in self._lc_calib_points],
        }
        backup = self._lc_backup_previous()
        try:
            os.makedirs(os.path.dirname(self._lc_calib_path), exist_ok=True)
            with open(self._lc_calib_path, 'w', encoding='utf-8') as fh:
                json.dump(data, fh, indent=2)
        except OSError as exc:
            self._set_status(f'Could not write the calibration: {exc}',
                             DANGER)
            return
        onde = (f' Previous one kept at {backup}.' if backup else '')
        fp = lc_calib_fingerprint(self._lc_calib_path)
        if LC_CALIB_SOURCE in LC_CALIB_SHARED_SOURCES:
            self._set_status(
                f'Calibration [{fp}] saved to {self._lc_calib_path} — the '
                f'receiver picks it up within 5 s.{onde} Commit that file to '
                'carry this calibration to the other machines.', OK)
        else:
            # `config` é a única origem que não viaja: quem calibrar aqui
            # calibra SÓ esta máquina, e o sintoma na outra é medir diferente
            # sem nada acusar.
            self._set_status(
                f'Calibration [{fp}] saved to {self._lc_calib_path}, which is '
                'LOCAL to this computer and does NOT propagate: the other '
                'machines keep measuring with the old line. Copy it over '
                f'sensors/load_cell_calib.json in the repo.{onde}', WARN)

    def _lc_backup_previous(self) -> str | None:
        """Copia a calibração vigente antes de sobrescrevê-la. Devolve o
        caminho da cópia, ou None se não havia nada a copiar.

        O alvo é `sensors/load_cell_calib.json`, que é VERSIONADO — e isso
        corta os dois jeitos de perder a calibração: um `git checkout`
        distraído apaga a nova, e um Save em cima apaga a antiga. A cópia vai
        para o CONFIG_DIR, fora do git: não suja o repo, não entra em commit
        por engano, e sobrevive ao checkout.
        """
        try:
            if not os.path.exists(self._lc_calib_path):
                return None
            os.makedirs(CONFIG_DIR, exist_ok=True)
            alvo = os.path.join(
                CONFIG_DIR,
                time.strftime('load_cell_calib.%Y%m%d_%H%M%S.json'))
            with open(self._lc_calib_path, encoding='utf-8') as src, \
                    open(alvo, 'w', encoding='utf-8') as dst:
                dst.write(src.read())
            return alvo
        except OSError:
            # Backup é rede de segurança, não pré-requisito: falhar aqui não
            # pode impedir de gravar uma calibração boa.
            return None

    # ── Refresh ───────────────────────────────────────────────────────
    def _refresh_lc_points(self) -> None:
        txt = self._lc_points_txt
        txt.configure(state='normal')
        txt.delete('1.0', 'end')
        v0 = self._lc_calib_zero
        txt.insert('end', f'{"zero (V0)":>10}  {"":>10}  '
                          f'{v0 * 1e3:12.5f}\n' if v0 is not None
                   else f'{"zero (V0)":>10}  {"":>10}  {"not captured":>12}\n')
        if not self._lc_calib_points:
            txt.insert('end', '(no mass points yet)\n')
        else:
            txt.insert('end', f'{"mass kg":>10}  {"force N":>10}  '
                              f'{"bridge mV":>12}\n')
            for m, f, v in self._lc_calib_points:
                txt.insert('end', f'{m:10.4f}  {f:10.4f}  {v * 1e3:12.5f}\n')
        txt.configure(state='disabled')

    def _refresh_lc_calib(self) -> None:
        """Tick da aba.

        Gate de visibilidade como no `_refresh_ft_axes`: com a aba escondida
        não há o que repintar. A COLETA, porém, não pode parar junto — ela é
        alimentada na callback do ROS e só é FECHADA aqui, e um operador que
        troca de aba no meio dos 1,5 s deixaria o botão travado em
        'disabled' para sempre. Por isso o fechamento vem antes do gate.
        """
        with self._lock:
            v = self._lc_voltage
            ts = self._lc_voltage_ts
            cap = self._lc_capture
            if cap is not None and time.time() >= cap[0]:
                self._lc_capture = None
        if cap is not None and self._lc_capture is None:
            self._lc_finish_capture(cap[1])

        if not self._tab_visible(getattr(self, '_lc_tab_frame', None)):
            self.root.after(200, self._refresh_lc_calib)
            return

        vivo = ts > 0.0 and (time.time() - ts) < _STALE_S
        cor = TEXT if vivo else TEXT_DIM
        self._lc_calib_lbls['v'].configure(
            text=(f'{v * 1e3:+.4f} mV' if vivo else '—'), fg=cor)
        fit = self._lc_calib_fit
        if vivo and fit is not None:
            f_n = lc_force_n(v, fit[0], fit[1])
            self._lc_calib_lbls['f'].configure(text=f'{f_n:+.3f} N', fg=cor)
        else:
            self._lc_calib_lbls['f'].configure(
                text=('fit first' if vivo else '—'), fg=TEXT_DIM)
        idade = (time.time() - ts) if ts > 0.0 else -1.0
        self._lc_calib_lbls['age'].configure(
            text=(f'{idade:.1f} s' if idade >= 0.0 else 'never'), fg=cor)
        if not vivo:
            self._lc_calib_note.configure(
                text=('Waiting for /load_cell/voltage. This tab needs the '
                      'force_receiver — with force_sensor:=ft6 the FA7155 is '
                      'factory-calibrated and there is nothing to fit here.'),
                fg=WARN)
        else:
            self._lc_calib_note.configure(text='', fg=TEXT_DIM)
        self.root.after(120, self._refresh_lc_calib)

