"""
gui_loadcell.py — aba "6 Axes" da célula FA7155 (RS485, seis eixos).

Recorte de `palpation_gui.py`, que era uma classe só com 234 métodos.

26/08/2026 — PARIDADE COM O CLIENTE DE FÁBRICA. Esta aba era só de LEITURA,
sob a premissa (escrita no `ft_receiver_node`) de que o sensor "só fala". A
análise do `Six_Axis_FT.exe` da FIBOS mostrou que ele é também um escravo
Modbus RTU na mesma linha 485, com Set_Zero, taxa de saída, node ID, baud e
start/stop do stream. A aba passou a ter o que o cliente de fábrica tem:

  Painel                    equivalente na FIBOS
  ─────────────────────────────────────────────────────────────────
  Acquisition mode          Serial communication (stream x polled 0x03)
  Command channel           Modbus correspondence + Device ID
  Sensor commands           Set_Zero, Send_Frequency, Send_ModBus_ID,
                            Send_Baud_rate, StartReading/stopReading
  Register scan             (não tem equivalente — ver abaixo)
  Charts                    gráficos qwt de força e torque
  Columns                   vista de colunas
  Force vector 3D           Qt5OpenGL + OBJ/arrowhead.obj
  Display filter            Savitzky-Golay (janela ímpar, ordem < janela)
  Statistics                Mean_Num / MAX_Num
  Recording                 Save_CsvMsg (carimbo yyyy-MM-dd hh:mm:ss.zzz)
  Chart ranges              Csv/Range.csv

DOIS PONTOS EM QUE ESTA ABA NÃO COPIA O FABRICANTE, DE PROPÓSITO:

* MODO DE AQUISIÇÃO. O cliente de fábrica só pergunta (Modbus 0x03 em laço,
  ~262 Hz de teto a 115200). Aqui o default continua sendo o stream "ST" a
  1 Mbps, que entrega 1 kHz; polled é escolha, não imposição. O painel
  "Acquisition mode" troca os dois a quente e mostra os dois tetos.

* REGISTER SCAN não existe na FIBOS. Ele é a saída para a trava abaixo sem
  sniffer: varre o espaço de registradores SÓ LENDO (endereço inexistente
  devolve exceção 0x02, que não altera nada no escravo) e casa os valores
  lidos com o baud, a taxa e o node ID que o host já conhece. O que ele
  produz são CANDIDATOS para o FT_MODBUS_MAP, nunca conclusão.

TRAVA: enquanto `FT_MODBUS_MAP_CONFIRMED` for False em constants.py, os
botões de ESCRITA ficam desabilitados e o painel diz por quê. Endereço de
registrador adivinhado pode custar o node ID ou um baud inacessível, e o
sensor não tem reset de fábrica. Leitura (`Probe`) segue liberada.

Fronteira que não mudou: quem tem a porta é o `ft_receiver`. A GUI PEDE, por
`/ft_sensor/command`, e escuta `/ft_sensor/command_result`. O tare do host
continua sendo outra coisa, e o painel diz a diferença.
"""
from __future__ import annotations

import json
import math
import os
import time
import tkinter as tk
from tkinter import ttk

from std_msgs.msg import Bool, Empty, Float32, String
from .constants import (
    FT_AXES, FT_AXIS_LABELS, FT_BAUD_CHOICES, FT_MODBUS_MAP_CONFIRMED,
    FT_MODBUS_SLAVE_ID, FT_NOMINAL_RATE_HZ, FT_MAX_RATE_HZ, FT_MIN_RATE_HZ,
    FT_RATE_CHOICES_HZ, FT_RATED_FORCE_N, FT_RATED_TORQUE_NM,
    FT_SAFE_OVERLOAD_PCT, FT_SERIAL_BAUD, FT_SG_ORDER_DEFAULT,
    FT_SG_WINDOW_DEFAULT, FT_STATS_WINDOW_DEFAULT, RUNS_DIR, ft_axis_rated,
    FT_MODE_POLLED, FT_MODE_STREAM, ft_max_rate_hz, ft_polled_max_rate_hz,
)
from .ft_stats import RollingStats, StreamingSavGol, validate_savgol
from .ui_helpers import (
    PANEL, TEXT, TEXT_MUTED, TEXT_DIM,
    PRIMARY, PRIMARY_HV, OK, WARN, DANGER, BORDER,
    FONT_BIG, FONT_HEAD, FONT_LBL, FONT_SMALL, FONT_MONO, FONT_MONO_S,
    _shade,
)


# ═══════════════════════════════════════════════════════════════════════════
# ABA "6 AXES" — célula FA7155 (RS485, seis eixos)
# ═══════════════════════════════════════════════════════════════════════════

# Geometria da barra bipolar. O zero fica no MEIO: força de compressão e de
# tração ocupam metades opostas, então o sinal é lido pela direção, não pelo
# rótulo.
_BAR_W = 260
_BAR_H = 16

# Carimbo do CSV, igual ao do cliente de fábrica — para as duas planilhas
# poderem ser comparadas linha a linha sem conversão.
_CSV_TS_FMT = '%Y-%m-%d %H:%M:%S'


def _stamp(t: float) -> str:
    return f'{time.strftime(_CSV_TS_FMT, time.localtime(t))}.{int(t % 1 * 1000):03d}'


class FtAxesMixin:
    """Mixin de `PalpationGUI` — todo o estado vive no host, como no resto
    dos recortes da GUI."""

    # ── Tare (zeragem dos seis eixos NO HOST) ─────────────────────────
    # Diferente do Zero do SENSOR (comando Modbus, mais abaixo): o tare é uma
    # subtração no software, some quando o ft_receiver reinicia, e é o que a
    # malha do explorer consome. Os dois existem, e o painel diz qual é qual.
    def _lc_do_tare(self) -> None:
        self._lc_tare_req_pub.publish(Empty())
        self._set_status('Taring — hold the probe unloaded…', WARN)

    def _cb_lc_tare_result(self, msg: String) -> None:
        """'ok;<ref_N>;<deriva_N>' ou 'err;<causa>;<valor>' do ft_receiver."""
        parts = str(msg.data).split(';')
        kind = parts[0] if parts else ''
        if kind == 'ok' and len(parts) >= 3:
            try:
                ref, drift = float(parts[1]), float(parts[2])
            except ValueError:
                return
            self._set_status(
                f'Six axes tared — reference {ref:+.3f} N '
                f'(drift {drift:.3f} N across the window).', OK)
            return
        if kind != 'err' or len(parts) < 3:
            return
        cause, value = parts[1], parts[2]
        if cause == 'no_data':
            self._set_status(
                'No frames from the cell — check the 24 V supply, the RS485 '
                'pair and the common ground.', WARN)
        elif cause == 'drifting':
            self._set_status(
                f'Cell drifting ({value} N across the window) — unload it and '
                'wait for it to settle before taring.', WARN)

    def _cb_lc_tared(self, msg: Bool) -> None:
        with self._lock:
            self._lc_tare_done = bool(msg.data)

    def _cb_lc_force_net_gui(self, msg: Float32) -> None:
        """Força do eixo de controle PÓS-tare — a que o explorer regula."""
        with self._lock:
            self._lc_force_net = float(msg.data)
            self._lc_force_net_ts = time.time()

    def _cb_lc_force_raw_gui(self, msg: Float32) -> None:
        """A MESMA leitura antes do tare. A diferença entre as duas é o zero
        corrente — é ela que denuncia deriva sem precisar descarregar."""
        with self._lock:
            self._lc_force_raw = float(msg.data)

    # ══════════════════════════════════════════════════════════════════
    # Processamento por amostra (SG, estatísticas, CSV)
    # ══════════════════════════════════════════════════════════════════
    def _ft_processing_init(self) -> None:
        """Chamado uma vez na construção da aba. Estado separado do desenho
        porque ele é alimentado pela thread do executor ROS, não pelo Tk."""
        self._ft_sg = {a: StreamingSavGol(FT_SG_WINDOW_DEFAULT,
                                          FT_SG_ORDER_DEFAULT)
                       for a in FT_AXES}
        self._ft_sg_on = False
        self._ft_stats = {a: RollingStats(FT_STATS_WINDOW_DEFAULT)
                          for a in FT_AXES}
        self._ft_smooth = {a: 0.0 for a in FT_AXES}
        self._ft_csv = None            # file handle da gravação
        self._ft_csv_path = ''
        self._ft_csv_rows = 0
        # Sinalizado pela thread do ROS, consumido pela do Tk: widget não
        # pode ser tocado fora da thread da GUI.
        self._ft_csv_error = ''
        self._ft_charts_init()

    def _ft_feed_processing(self, now: float, vals: tuple) -> None:
        """Uma amostra dos seis eixos, na taxa do sensor (~1 kHz).

        Roda na thread do executor ROS. Tudo aqui é O(1) por eixo — o SG é um
        produto interno de 11 termos e a estatística é um append em deque; a
        varredura da janela só acontece no snapshot, a 10 Hz, na thread do Tk.
        """
        proc = getattr(self, '_ft_sg', None)
        if proc is None:
            return                      # aba ainda não construída
        with self._lock:
            sg_on = self._ft_sg_on
            mostrados = []
            for axis, v in zip(FT_AXES, vals):
                sv = self._ft_sg[axis].update(v) if sg_on else v
                self._ft_smooth[axis] = sv
                # A estatística é do sinal CRU: ela mede o sensor, não o
                # filtro de exibição.
                self._ft_stats[axis].update(v)
                mostrados.append(sv)
            # Os gráficos recebem o valor mostrado — é o que o cliente de
            # fábrica plota, e o que o CSV de exibição guarda.
            self._ft_charts_feed(mostrados)
            csv = self._ft_csv
        if csv is not None:
            try:
                csv.write(f'{_stamp(now)},'
                          + ','.join(f'{v:.6f}' for v in vals) + '\n')
                with self._lock:
                    self._ft_csv_rows += 1
            except Exception as exc:
                # Disco cheio ou pendrive removido no meio do ensaio: parar
                # a gravação é melhor do que derrubar o callback do sensor,
                # que levaria /ft_sensor/wrench junto. Só o ARQUIVO é
                # fechado aqui — quem repinta é _refresh_ft_axes, na thread
                # do Tk. Mexer em widget daqui trava a GUI de formas que
                # não aparecem em teste.
                with self._lock:
                    self._ft_csv = None
                    self._ft_csv_error = str(exc)
                try:
                    csv.close()
                except Exception:
                    pass

    # ══════════════════════════════════════════════════════════════════
    # Canal de comando (Modbus) — pedidos e respostas
    # ══════════════════════════════════════════════════════════════════
    def _ft_send_cmd(self, cmd: str, **args) -> None:
        self._ft_cmd_pub.publish(String(data=json.dumps(
            {'cmd': cmd, 'args': args})))
        self._ft_cmd_lbl.config(text=f'{cmd}: aguardando…', fg=TEXT_MUTED)
        self._set_status(f'Sensor command "{cmd}" sent…', WARN)

    def _cb_ft_cmd_result(self, msg: String) -> None:
        """Resposta do ft_receiver. Sempre chega, inclusive na recusa."""
        try:
            r = json.loads(str(msg.data))
        except Exception:
            return
        cmd = str(r.get('cmd', '?'))
        ok = bool(r.get('ok'))
        detail = str(r.get('detail', ''))
        data = dict(r.get('data') or {})
        cor = OK if ok else (WARN if data.get('reason') == 'map_unconfirmed'
                             else DANGER)
        try:
            self._ft_cmd_lbl.config(text=f'{cmd}: {detail}', fg=cor)
        except (AttributeError, tk.TclError):
            pass
        self._set_status(f'{cmd}: {detail}', cor)
        if cmd == 'scan':
            if ok:
                sug = data.get('suggest') or {}
                linhas = ['CANDIDATES (verify against the factory client '
                          'before filling FT_MODBUS_MAP):']
                for chave in ('node_id', 'rate', 'baud'):
                    v = sug.get(chave) or []
                    achado = ', '.join(v) if v else 'nothing matched'
                    linhas.append(f'  {chave:8s} {achado}')
                linhas += ['', 'ADDRESSES THAT ANSWERED:',
                           str(data.get('text', ''))]
                self._ft_set_scan_text(chr(10).join(linhas))
            else:
                self._ft_set_scan_text(f'scan failed: {detail}')
            return

        if cmd == 'mode' and ok:
            novo = str(data.get('mode', ''))
            if novo:
                try:
                    self._ft_mode_var.set(novo)
                except (AttributeError, tk.TclError):
                    pass
            return

        if cmd == 'probe' and ok:
            txt = '  ·  '.join(f'{k} = {v}' for k, v in data.items())
            try:
                self._ft_dev_lbl.config(text=txt or '—', fg=TEXT)
            except (AttributeError, tk.TclError):
                pass

    # ══════════════════════════════════════════════════════════════════
    # Construção
    # ══════════════════════════════════════════════════════════════════
    def _build_lc_axes_tab(self, root: tk.Frame) -> None:
        """Seis eixos ao vivo, saúde do link e o painel de comando."""
        self._ft_axis_widgets = {}
        self._ft_processing_init()

        self._build_ft_link_card(root)
        self._build_ft_mode_card(root)
        self._build_ft_axes_card(root)
        self._build_ft_chart_card(root)
        self._build_ft_columns_card(root)
        self._build_ft_arrow_card(root)
        self._build_ft_command_card(root)
        self._build_ft_filter_card(root)
        self._build_ft_stats_card(root)
        self._build_ft_record_card(root)
        self._build_ft_capacity_card(root)

    # ── Saúde do link ─────────────────────────────────────────────────
    def _build_ft_link_card(self, root: tk.Frame) -> None:
        card_link = self._card(root, 'FA7155 — RS485 Link', expand=False)

        grid = tk.Frame(card_link, bg=PANEL)
        grid.pack(fill='x', pady=(6, 2))
        for c in range(4):
            grid.columnconfigure(c, weight=1, uniform='ft_link')

        self._ft_stat_lbls = {}
        for col, (key, title) in enumerate((
                ('rate',   'Measured rate'),
                ('frames', 'Frames OK'),
                ('bad',    'CRC / resync errors'),
                ('age',    'Last frame'))):
            cell = tk.Frame(grid, bg=PANEL)
            cell.grid(row=0, column=col, sticky='ew', padx=(0, 10))
            tk.Label(cell, text=title, font=FONT_SMALL, bg=PANEL,
                     fg=TEXT_DIM, anchor='w').pack(fill='x')
            lbl = tk.Label(cell, text='—', font=FONT_BIG, bg=PANEL,
                           fg=TEXT_DIM, anchor='w')
            lbl.pack(fill='x')
            self._ft_stat_lbls[key] = lbl

        self._ft_link_note = tk.Label(
            card_link,
            text=(f'Nominal {FT_NOMINAL_RATE_HZ:.0f} Hz  ·  '
                  f'link ceiling {FT_MAX_RATE_HZ:.0f} Hz @ {FT_SERIAL_BAUD} baud'),
            font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w', justify='left')
        self._ft_link_note.pack(fill='x', pady=(6, 0))

    # ── Modo de aquisição (stream x polled) ─────────────────
    def _build_ft_mode_card(self, root: tk.Frame) -> None:
        """Stream x polled — o par do painel "Serial communication" da FIBOS.

        O cliente de fábrica adquire PERGUNTANDO (Modbus 0x03 em laço). Este
        repo nasceu ESCUTANDO o stream "ST" a 1 Mbps. Os dois caminhos existem
        no sensor e convivem na mesma linha 485; aqui se escolhe qual está no
        ar. A troca é a quente: o driver polled não é dono da porta, então
        ligar e desligar o laço não reabre nada.
        """
        card = self._card(root, 'Acquisition mode', expand=False)

        teto_s = ft_max_rate_hz(FT_SERIAL_BAUD)
        teto_p = ft_polled_max_rate_hz(115200)
        tk.Label(
            card,
            text=(f'stream — the cell talks on its own, 28-byte "ST" frames. '
                  f'Wire ceiling {teto_s:.0f} Hz at {FT_SERIAL_BAUD} baud.\n'
                  f'polled — the host asks (01 03 00 03 00 0C), exactly like '
                  f'the factory client. Wire ceiling {teto_p:.0f} Hz at '
                  f'115200, and the MEASURED rate lands well below it because '
                  f'USB latency is not in that number. This is the mode that '
                  f'fits the 115200 of the flange 485.'),
            font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
            justify='left', wraplength=780).pack(fill='x', pady=(6, 8))

        row = tk.Frame(card, bg=PANEL)
        row.pack(fill='x')
        self._ft_mode_var = tk.StringVar(value=FT_MODE_STREAM)
        for valor, rotulo in ((FT_MODE_STREAM, 'stream (listen)'),
                              (FT_MODE_POLLED, 'polled (ask)')):
            tk.Radiobutton(row, text=rotulo, value=valor,
                           variable=self._ft_mode_var, bg=PANEL, fg=TEXT,
                           selectcolor=PANEL, activebackground=PANEL,
                           activeforeground=TEXT, font=FONT_LBL,
                           highlightthickness=0, bd=0).pack(side='left',
                                                            padx=(0, 18))
        tk.Button(row, text='Apply mode',
                  command=lambda: self._ft_send_cmd(
                      'mode', mode=self._ft_mode_var.get()),
                  bg=PRIMARY, fg='white', activebackground=PRIMARY_HV,
                  activeforeground='white', font=FONT_LBL, relief='flat',
                  bd=0, padx=12, pady=5, cursor='hand2').pack(side='left')
        tk.Label(row, text='Switches live — the port is not reopened.',
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM).pack(
            side='left', padx=(12, 0))

    # ── Os seis eixos ─────────────────────────────────────────────────
    def _build_ft_axes_card(self, root: tk.Frame) -> None:
        card_ax = self._card(root, 'Six Axes — live')

        hdr = tk.Frame(card_ax, bg=PANEL)
        hdr.pack(fill='x', pady=(4, 2))
        tk.Label(hdr, text='axis', font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
                 width=5, anchor='w').pack(side='left')
        tk.Label(hdr, text='value', font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
                 width=14, anchor='e').pack(side='left', padx=(0, 10))
        tk.Label(hdr, text='0 centred  ·  ends = ±full scale',
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
                 anchor='w').pack(side='left')

        for i, (axis, label, unit) in enumerate(FT_AXIS_LABELS):
            if i == 3:
                # Separador força/torque: são grandezas diferentes e escalas
                # diferentes; juntá-las sem marca convida a leitura errada.
                tk.Frame(card_ax, bg=BORDER, height=1).pack(fill='x', pady=6)
            self._ft_axis_widgets[axis] = self._build_ft_axis_row(
                card_ax, axis, label, unit)

        tk.Frame(card_ax, bg=BORDER, height=1).pack(fill='x', pady=(8, 6))

        mag = tk.Frame(card_ax, bg=PANEL)
        mag.pack(fill='x')
        self._ft_fmag_lbl = self._build_ft_magnitude(mag, '|F|', 'N')
        self._ft_mmag_lbl = self._build_ft_magnitude(mag, '|M|', 'N·m')

        btn_row = tk.Frame(card_ax, bg=PANEL)
        btn_row.pack(fill='x', pady=(10, 2))
        tk.Button(btn_row, text='⊘  Tare all axes (host)',
                  command=self._lc_do_tare,
                  bg=PRIMARY, fg='white',
                  activebackground=PRIMARY_HV, activeforeground='white',
                  font=FONT_LBL, relief='flat', bd=0, padx=14, pady=6,
                  cursor='hand2').pack(side='left')
        self._ft_tare_note = tk.Label(
            btn_row,
            text=('Software offset, lives in ft_receiver — do it unloaded. '
                  'The sensor-side zero is in Sensor commands below.'),
            font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, justify='left')
        self._ft_tare_note.pack(side='left', padx=(12, 0))

    # ── Canal de comando + comandos do sensor ─────────────────────────
    def _build_ft_command_card(self, root: tk.Frame) -> None:
        card = self._card(root, 'Sensor commands — Modbus RTU', expand=False)

        # Banner da trava. É a primeira coisa do painel de propósito: sem ele
        # os botões cinzentos pareceriam bug.
        if not FT_MODBUS_MAP_CONFIRMED:
            banner = tk.Label(
                card,
                text=('⚠  Register map NOT confirmed — every WRITE below is '
                      'blocked.\nCapture the factory client on the serial '
                      'line, fill FT_MODBUS_MAP in constants.py and set '
                      'FT_MODBUS_MAP_CONFIRMED = True. A wrong address can '
                      'cost the node ID or leave the cell on a baud the host '
                      'cannot speak, and there is no factory reset.'),
                font=FONT_SMALL, bg=PANEL, fg=WARN, anchor='w',
                justify='left', wraplength=780)
            banner.pack(fill='x', pady=(6, 8))

        info = tk.Frame(card, bg=PANEL)
        info.pack(fill='x', pady=(2, 6))
        tk.Label(info, text=f'Slave ID {FT_MODBUS_SLAVE_ID}', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(side='left')
        tk.Button(info, text='Probe', command=lambda: self._ft_send_cmd('probe'),
                  bg=PRIMARY, fg='white', activebackground=PRIMARY_HV,
                  activeforeground='white', font=FONT_SMALL, relief='flat',
                  bd=0, padx=10, pady=3, cursor='hand2').pack(
            side='left', padx=(12, 8))
        self._ft_dev_lbl = tk.Label(info, text='—', font=FONT_MONO_S,
                                    bg=PANEL, fg=TEXT_DIM)
        self._ft_dev_lbl.pack(side='left')

        self._ft_write_btns = []
        st = 'disabled' if not FT_MODBUS_MAP_CONFIRMED else 'normal'

        # Zero do SENSOR — o par do Set_Zero do cliente de fábrica.
        r0 = tk.Frame(card, bg=PANEL); r0.pack(fill='x', pady=3)
        self._ft_btn(r0, '⊙  Zero sensor', lambda: self._ft_send_cmd('zero'), st)
        tk.Label(r0, text=('Hardware zero — survives a node restart and clears '
                           'the host tare.'),
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM).pack(
            side='left', padx=(10, 0))

        # Taxa de saída.
        r1 = tk.Frame(card, bg=PANEL); r1.pack(fill='x', pady=3)
        self._ft_rate_var = tk.StringVar(value=str(int(FT_NOMINAL_RATE_HZ)))
        ttk.Combobox(r1, textvariable=self._ft_rate_var, width=8,
                     state='readonly',
                     values=[str(v) for v in FT_RATE_CHOICES_HZ]).pack(
            side='left')
        self._ft_btn(r1, 'Set rate (Hz)',
                     lambda: self._ft_send_cmd(
                         'rate', hz=int(self._ft_rate_var.get())), st)
        tk.Label(r1, text='Takes effect after a power cycle.',
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM).pack(
            side='left', padx=(10, 0))

        # Node ID.
        r2 = tk.Frame(card, bg=PANEL); r2.pack(fill='x', pady=3)
        self._ft_node_var = tk.StringVar(value=str(FT_MODBUS_SLAVE_ID))
        tk.Entry(r2, textvariable=self._ft_node_var, width=8,
                 font=FONT_MONO_S).pack(side='left')
        self._ft_btn(r2, 'Set node ID',
                     lambda: self._ft_send_cmd(
                         'node_id', node_id=int(self._ft_node_var.get())), st)
        tk.Label(r2, text=('1–247. After the power cycle the cell answers on '
                           'the NEW id — update ft_modbus_slave_id too.'),
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM).pack(
            side='left', padx=(10, 0))

        # Baud.
        r3 = tk.Frame(card, bg=PANEL); r3.pack(fill='x', pady=3)
        self._ft_baud_var = tk.StringVar(value=str(FT_SERIAL_BAUD))
        ttk.Combobox(r3, textvariable=self._ft_baud_var, width=8,
                     state='readonly',
                     values=[str(v) for v in FT_BAUD_CHOICES]).pack(side='left')
        self._ft_btn(r3, 'Set baud',
                     lambda: self._ft_send_cmd(
                         'baud', baud=int(self._ft_baud_var.get())), st)
        tk.Label(r3, text='Power cycle, then reopen the port at the new baud.',
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM).pack(
            side='left', padx=(10, 0))

        # Stream on/off.
        r4 = tk.Frame(card, bg=PANEL); r4.pack(fill='x', pady=3)
        self._ft_btn(r4, '▶  Start stream',
                     lambda: self._ft_send_cmd('stream_start'), st)
        self._ft_btn(r4, '■  Stop stream',
                     lambda: self._ft_send_cmd('stream_stop'), st)
        tk.Label(r4, text='Stopping the stream also stops /ft_sensor/wrench.',
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM).pack(
            side='left', padx=(10, 0))

        # ── Varredura de registradores ────────────────────
        # É a saída para a trava acima sem sniffer e sem segundo adaptador:
        # LER é inofensivo (endereço inexistente devolve exceção 0x02, que
        # não altera nada no escravo), então dá para descobrir QUAIS
        # endereços existem antes de escrever em qualquer um.
        tk.Frame(card, bg=BORDER, height=1).pack(fill='x', pady=(10, 8))
        tk.Label(card,
                 text=('Register scan — read-only, safe with the map still '
                       'unconfirmed. It finds which addresses EXIST and '
                       'matches their values against the baud, rate and node '
                       'id this host already knows. The result is a list of '
                       'CANDIDATES, never a conclusion.'),
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
                 justify='left', wraplength=780).pack(fill='x', pady=(0, 6))

        r5 = tk.Frame(card, bg=PANEL)
        r5.pack(fill='x', pady=3)
        self._ft_scan_ini = tk.StringVar(value='0')
        self._ft_scan_fim = tk.StringVar(value='64')
        for rotulo, var in (('from', self._ft_scan_ini),
                            ('to', self._ft_scan_fim)):
            tk.Label(r5, text=rotulo, font=FONT_SMALL, bg=PANEL,
                     fg=TEXT_DIM).pack(side='left', padx=(0, 4))
            tk.Entry(r5, textvariable=var, width=6,
                     font=FONT_MONO_S).pack(side='left', padx=(0, 12))
        tk.Button(r5, text='⌕  Scan registers', command=self._ft_do_scan,
                  bg=PRIMARY, fg='white', activebackground=PRIMARY_HV,
                  activeforeground='white', font=FONT_LBL, relief='flat',
                  bd=0, padx=12, pady=5, cursor='hand2').pack(side='left')
        tk.Label(r5, text='Decimal addresses. Takes a few seconds.',
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM).pack(
            side='left', padx=(10, 0))

        self._ft_scan_txt = tk.Text(card, height=9, font=FONT_MONO_S,
                                    bg=PANEL, fg=TEXT, relief='flat',
                                    highlightthickness=1,
                                    highlightbackground=BORDER, wrap='word')
        self._ft_scan_txt.pack(fill='x', pady=(6, 2))
        self._ft_scan_txt.insert('1.0', 'No scan yet.')
        self._ft_scan_txt.config(state='disabled')

        self._ft_cmd_lbl = tk.Label(card, text='—', font=FONT_SMALL, bg=PANEL,
                                    fg=TEXT_DIM, anchor='w', justify='left',
                                    wraplength=780)
        self._ft_cmd_lbl.pack(fill='x', pady=(8, 2))

    def _ft_do_scan(self) -> None:
        try:
            ini = int(self._ft_scan_ini.get())
            fim = int(self._ft_scan_fim.get())
        except ValueError:
            self._ft_cmd_lbl.config(text='scan: addresses must be integers.',
                                    fg=DANGER)
            return
        self._ft_set_scan_text('Scanning…')
        self._ft_send_cmd('scan', start=ini, end=fim)

    def _ft_set_scan_text(self, txt: str) -> None:
        """O Text fica 'disabled' para ser só de leitura, e o Tk recusa
        insert nesse estado — por isso o normal/insert/disabled."""
        try:
            self._ft_scan_txt.config(state='normal')
            self._ft_scan_txt.delete('1.0', 'end')
            self._ft_scan_txt.insert('1.0', txt)
            self._ft_scan_txt.config(state='disabled')
        except (AttributeError, tk.TclError):
            pass

    def _ft_btn(self, parent, text, command, state):
        b = tk.Button(parent, text=text, command=command, state=state,
                      bg=PRIMARY if state == 'normal' else _shade(BORDER, 0.3),
                      fg='white' if state == 'normal' else TEXT_DIM,
                      activebackground=PRIMARY_HV, activeforeground='white',
                      disabledforeground=TEXT_DIM,
                      font=FONT_LBL, relief='flat', bd=0, padx=12, pady=5,
                      cursor='hand2' if state == 'normal' else 'arrow')
        b.pack(side='left', padx=(0, 8))
        self._ft_write_btns.append(b)
        return b

    # ── Filtro de exibição (Savitzky-Golay) ───────────────────────────
    def _build_ft_filter_card(self, root: tk.Frame) -> None:
        card = self._card(root, 'Display filter — Savitzky-Golay',
                          expand=False)
        tk.Label(card, text=('Display and CSV only. The control loop keeps the '
                             'median + One-Euro of lc_filter — changing that '
                             'one changes what the explorer gains mean.'),
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
                 justify='left', wraplength=780).pack(fill='x', pady=(6, 6))

        row = tk.Frame(card, bg=PANEL); row.pack(fill='x')
        self._ft_sg_var = tk.BooleanVar(value=False)
        tk.Checkbutton(row, text='Enabled', variable=self._ft_sg_var,
                       command=self._ft_apply_sg, bg=PANEL, fg=TEXT,
                       selectcolor=PANEL, activebackground=PANEL,
                       activeforeground=TEXT, font=FONT_LBL,
                       highlightthickness=0, bd=0).pack(side='left')

        tk.Label(row, text='window', font=FONT_SMALL, bg=PANEL,
                 fg=TEXT_DIM).pack(side='left', padx=(16, 4))
        self._ft_sg_win = tk.StringVar(value=str(FT_SG_WINDOW_DEFAULT))
        tk.Spinbox(row, from_=3, to=101, increment=2, width=5,
                   textvariable=self._ft_sg_win, font=FONT_MONO_S,
                   command=self._ft_apply_sg).pack(side='left')

        tk.Label(row, text='order', font=FONT_SMALL, bg=PANEL,
                 fg=TEXT_DIM).pack(side='left', padx=(16, 4))
        self._ft_sg_ord = tk.StringVar(value=str(FT_SG_ORDER_DEFAULT))
        tk.Spinbox(row, from_=1, to=9, width=5, textvariable=self._ft_sg_ord,
                   font=FONT_MONO_S, command=self._ft_apply_sg).pack(
            side='left')

        self._ft_sg_lbl = tk.Label(card, text='', font=FONT_SMALL, bg=PANEL,
                                   fg=TEXT_DIM, anchor='w')
        self._ft_sg_lbl.pack(fill='x', pady=(6, 2))

    def _ft_apply_sg(self) -> None:
        """Valida com as MESMAS regras (e o mesmo texto) do cliente FIBOS."""
        try:
            w, o = int(self._ft_sg_win.get()), int(self._ft_sg_ord.get())
            validate_savgol(w, o)
        except ValueError as exc:
            self._ft_sg_lbl.config(text=str(exc), fg=DANGER)
            self._ft_sg_var.set(False)
            with self._lock:
                self._ft_sg_on = False
            return
        with self._lock:
            for f in self._ft_sg.values():
                f.configure(w, o)
            self._ft_sg_on = bool(self._ft_sg_var.get())
        lag = 0.0 if FT_NOMINAL_RATE_HZ <= 0 else (w - 1) / FT_NOMINAL_RATE_HZ
        self._ft_sg_lbl.config(
            text=(f'window {w}, order {o} — evaluated at the window edge, so '
                  f'no lag; it spans {lag * 1000:.0f} ms of history.'),
            fg=TEXT_DIM)

    # ── Estatísticas (Mean_Num / MAX_Num) ─────────────────────────────
    def _build_ft_stats_card(self, root: tk.Frame) -> None:
        card = self._card(root, 'Statistics — rolling window', expand=False)

        row = tk.Frame(card, bg=PANEL); row.pack(fill='x', pady=(6, 6))
        tk.Label(row, text='samples', font=FONT_SMALL, bg=PANEL,
                 fg=TEXT_DIM).pack(side='left', padx=(0, 4))
        self._ft_stats_win = tk.StringVar(value=str(FT_STATS_WINDOW_DEFAULT))
        tk.Spinbox(row, from_=10, to=5000, increment=10, width=7,
                   textvariable=self._ft_stats_win, font=FONT_MONO_S,
                   command=self._ft_apply_stats_window).pack(side='left')
        tk.Button(row, text='Reset', command=self._ft_reset_stats,
                  bg=PRIMARY, fg='white', activebackground=PRIMARY_HV,
                  activeforeground='white', font=FONT_SMALL, relief='flat',
                  bd=0, padx=10, pady=3, cursor='hand2').pack(
            side='left', padx=(12, 0))
        self._ft_stats_span = tk.Label(row, text='', font=FONT_SMALL,
                                       bg=PANEL, fg=TEXT_DIM)
        self._ft_stats_span.pack(side='left', padx=(12, 0))

        grid = tk.Frame(card, bg=PANEL); grid.pack(fill='x')
        cols = ('axis', 'mean', '|max|', 'min', 'max', 'rms', 'p-p')
        for c, title in enumerate(cols):
            grid.columnconfigure(c, weight=1, uniform='ft_stats')
            tk.Label(grid, text=title, font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
                     anchor='e' if c else 'w').grid(
                row=0, column=c, sticky='ew', padx=2)

        self._ft_stat_cells = {}
        for r, (axis, label, unit) in enumerate(FT_AXIS_LABELS, start=1):
            tk.Label(grid, text=label, font=FONT_LBL, bg=PANEL, fg=TEXT,
                     anchor='w').grid(row=r, column=0, sticky='ew', padx=2)
            cells = {}
            for c, key in enumerate(('mean', 'max_abs', 'min', 'max', 'rms',
                                     'pp'), start=1):
                lbl = tk.Label(grid, text='—', font=FONT_MONO_S, bg=PANEL,
                               fg=TEXT_DIM, anchor='e')
                lbl.grid(row=r, column=c, sticky='ew', padx=2)
                cells[key] = lbl
            self._ft_stat_cells[axis] = cells

    def _ft_apply_stats_window(self) -> None:
        try:
            n = int(self._ft_stats_win.get())
        except ValueError:
            return
        with self._lock:
            for s in self._ft_stats.values():
                s.resize(n)

    def _ft_reset_stats(self) -> None:
        with self._lock:
            for s in self._ft_stats.values():
                s.reset()
            for f in self._ft_sg.values():
                f.reset()

    # ── Gravação CSV (Save_CsvMsg) ────────────────────────────────────
    def _build_ft_record_card(self, root: tk.Frame) -> None:
        card = self._card(root, 'Recording — CSV', expand=False)
        row = tk.Frame(card, bg=PANEL); row.pack(fill='x', pady=(6, 4))
        self._ft_rec_btn = tk.Button(
            row, text='●  Start recording', command=self._ft_toggle_recording,
            bg=PRIMARY, fg='white', activebackground=PRIMARY_HV,
            activeforeground='white', font=FONT_LBL, relief='flat', bd=0,
            padx=14, pady=6, cursor='hand2')
        self._ft_rec_btn.pack(side='left')
        self._ft_rec_lbl = tk.Label(
            row, text='Idle.', font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
            anchor='w', justify='left')
        self._ft_rec_lbl.pack(side='left', padx=(12, 0))
        tk.Label(card,
                 text=(f'Raw six axes at the sensor rate, timestamped '
                       f'{_CSV_TS_FMT}.zzz like the factory client. Written '
                       f'under {os.path.join(RUNS_DIR, "ft_csv")}.'),
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
                 justify='left', wraplength=780).pack(fill='x', pady=(2, 2))

    def _ft_toggle_recording(self) -> None:
        if getattr(self, '_ft_csv', None) is None:
            self._ft_start_recording()
        else:
            self._ft_stop_recording()

    def _ft_start_recording(self) -> None:
        d = os.path.join(RUNS_DIR, 'ft_csv')
        try:
            os.makedirs(d, exist_ok=True)
            path = os.path.join(
                d, f'ft_{time.strftime("%Y%m%d_%H%M%S")}.csv')
            fh = open(path, 'w', encoding='utf-8', newline='')
            fh.write('timestamp,' + ','.join(FT_AXES) + '\n')
        except OSError as exc:
            self._ft_rec_lbl.config(text=f'Cannot record: {exc}', fg=DANGER)
            return
        with self._lock:
            self._ft_csv = fh
            self._ft_csv_path = path
            self._ft_csv_rows = 0
            self._ft_csv_error = ''
        self._ft_rec_btn.config(text='■  Stop recording', bg=DANGER)
        self._ft_rec_lbl.config(text=f'Recording → {path}', fg=OK)

    def _ft_stop_recording(self) -> None:
        """Só da thread do Tk (botão). O aborto por erro de escrita vem da
        thread do ROS e é repintado por _refresh_ft_axes."""
        with self._lock:
            fh, path, rows = (self._ft_csv, self._ft_csv_path,
                              self._ft_csv_rows)
            self._ft_csv = None
        if fh is not None:
            try:
                fh.close()
            except Exception:
                pass
        self._ft_rec_btn.config(text='●  Start recording', bg=PRIMARY)
        self._ft_rec_lbl.config(text=f'Saved {rows} rows → {path}', fg=OK)


    # ── Capacidade ────────────────────────────────────────────────────
    def _build_ft_capacity_card(self, root: tk.Frame) -> None:
        card_cap = self._card(root, 'Capacity', expand=False)
        cap_txt = (f'Full scale:  ±{FT_RATED_FORCE_N:.0f} N  (Fx, Fy, Fz)   ·   '
                   f'±{FT_RATED_TORQUE_NM:.1f} N·m  (Mx, My, Mz)')
        tk.Label(card_cap, text=cap_txt, font=FONT_LBL, bg=PANEL, fg=TEXT,
                 anchor='w').pack(fill='x', pady=(6, 2))
        if FT_SAFE_OVERLOAD_PCT is None:
            warn = ('Safe overload NOT configured. Full scale is the rated '
                    'capacity, not the deformation limit — the datasheet '
                    'figure for safe overload is still missing, so this panel '
                    'will not draw a damage threshold rather than invent one. '
                    'Set FT_SAFE_OVERLOAD_PCT in constants.py once you have it.')
            fg = WARN
        else:
            warn = (f'Safe overload: {FT_SAFE_OVERLOAD_PCT:.0f}% of full scale.')
            fg = TEXT_MUTED
        tk.Label(card_cap, text=warn, font=FONT_SMALL, bg=PANEL, fg=fg,
                 anchor='w', justify='left', wraplength=760).pack(
            fill='x', pady=(0, 4))

    def _build_ft_axis_row(self, parent, axis: str, label: str, unit: str):
        """Uma linha: rótulo, valor numérico, barra bipolar, % do fundo."""
        row = tk.Frame(parent, bg=PANEL)
        row.pack(fill='x', pady=3)

        tk.Label(row, text=label, font=FONT_HEAD, bg=PANEL, fg=TEXT,
                 width=5, anchor='w').pack(side='left')

        val = tk.Label(row, text=f'—  {unit}', font=FONT_MONO, bg=PANEL,
                       fg=TEXT_DIM, width=14, anchor='e')
        val.pack(side='left', padx=(0, 10))

        cv = tk.Canvas(row, width=_BAR_W, height=_BAR_H, bg=PANEL,
                       highlightthickness=0, bd=0)
        cv.pack(side='left')
        # Trilho + marca do zero, desenhados uma vez.
        # _shade recebe fator em [-1, 1]; acima disso o canal estoura 255 e
        # devolve uma string hex malformada, que o Tk pinta de PRETO.
        cv.create_rectangle(0, 3, _BAR_W, _BAR_H - 3,
                            fill=_shade(BORDER, 0.55), outline='')
        bar = cv.create_rectangle(_BAR_W // 2, 3, _BAR_W // 2, _BAR_H - 3,
                                  fill=PRIMARY, outline='')
        cv.create_line(_BAR_W // 2, 0, _BAR_W // 2, _BAR_H,
                       fill=TEXT_MUTED, width=1)

        pct = tk.Label(row, text='', font=FONT_MONO_S, bg=PANEL, fg=TEXT_DIM,
                       width=9, anchor='e')
        pct.pack(side='left', padx=(10, 0))

        return {'val': val, 'canvas': cv, 'bar': bar, 'pct': pct,
                'unit': unit, 'rated': ft_axis_rated(axis)}

    def _build_ft_magnitude(self, parent, title: str, unit: str):
        box = tk.Frame(parent, bg=PANEL)
        box.pack(side='left', padx=(0, 28))
        tk.Label(box, text=f'{title}  ({unit})', font=FONT_SMALL, bg=PANEL,
                 fg=TEXT_DIM, anchor='w').pack(fill='x')
        lbl = tk.Label(box, text='—', font=FONT_BIG, bg=PANEL, fg=TEXT_DIM,
                       anchor='w')
        lbl.pack(fill='x')
        return lbl

    # ── Atualização ───────────────────────────────────────────────────
    def _refresh_ft_axes(self) -> None:
        """Repinta os seis eixos, a saúde do link e as estatísticas.

        Roda a 10 Hz e NÃO a cada mensagem: o sensor entrega ~1000 quadros/s e
        redesenhar canvas nessa taxa satura a thread do Tk — foi o que já
        travou a GUI inteira no heatmap do toque (ver _build_sensors_tab)."""
        if not getattr(self, '_ft_axis_widgets', None):
            return
        # Gate de visibilidade, como nas outras abas. Sem ele estes seis
        # canvases de eixo, os dois gráficos e a seta 3D eram repintados a
        # 10 Hz PARA NINGUÉM enquanto o usuário estava noutra aba: medido em
        # 27/08/2026 com a aba Sensores à vista, 4,86 ms por chamada, 4,0% da
        # thread que pinta a GUI inteira. Os HISTÓRICOS não passam por aqui
        # (são alimentados na callback do ROS), então nada abre buraco.
        if not self._tab_visible(getattr(self, '_lc_tab_frame', None)):
            self.root.after(100, self._refresh_ft_axes)
            return
        now = time.time()
        with self._lock:
            w = dict(self._ft_wrench)
            smooth = dict(self._ft_smooth)
            sg_on = self._ft_sg_on
            snaps = {a: s.snapshot() for a, s in self._ft_stats.items()}
            ts = self._ft_last_ts
            n_ok = self._ft_frames_ok
            n_bad = self._ft_frames_bad
            rate = self._ft_rate_hz
            rec_rows = self._ft_csv_rows if self._ft_csv is not None else None
            rec_err, self._ft_csv_error = self._ft_csv_error, ''
            rec_path = self._ft_csv_path

        age = (now - ts) if ts > 0.0 else None
        live = age is not None and age < 1.0
        # Com o SG ligado o painel mostra o valor FILTRADO, que é o que o
        # cliente de fábrica plota — e o que o CSV de exibição guarda.
        shown = smooth if sg_on else w

        # ── Eixos ────────────────────────────────────────────────────
        for axis, wid in self._ft_axis_widgets.items():
            v = shown.get(axis)
            rated = wid['rated']
            if v is None or not live:
                wid['val'].config(text=f'—  {wid["unit"]}', fg=TEXT_DIM)
                wid['pct'].config(text='', fg=TEXT_DIM)
                wid['canvas'].coords(wid['bar'],
                                     _BAR_W // 2, 3, _BAR_W // 2, _BAR_H - 3)
                continue

            frac = 0.0 if rated <= 0 else max(-1.0, min(1.0, v / rated))
            mid = _BAR_W / 2.0
            x = mid + frac * mid
            wid['canvas'].coords(wid['bar'],
                                 min(mid, x), 3, max(mid, x), _BAR_H - 3)

            over = abs(v) / rated if rated > 0 else 0.0
            if over >= 1.0:
                cor = DANGER          # passou do fundo de escala
            elif over >= 0.8:
                cor = WARN
            else:
                cor = PRIMARY
            wid['canvas'].itemconfig(wid['bar'], fill=cor)
            # 4 casas para torque (valores pequenos), 3 para força.
            casas = 4 if wid['unit'] != 'N' else 3
            wid['val'].config(text=f'{v:+.{casas}f}  {wid["unit"]}',
                              fg=DANGER if over >= 1.0 else TEXT)
            wid['pct'].config(text=f'{over * 100:5.1f}% FS',
                              fg=cor if over >= 0.8 else TEXT_DIM)

        # ── Módulos ──────────────────────────────────────────────────
        if live:
            fmag = math.sqrt(sum(shown[a] ** 2 for a in ('fx', 'fy', 'fz')))
            mmag = math.sqrt(sum(shown[a] ** 2 for a in ('mx', 'my', 'mz')))
            self._ft_fmag_lbl.config(text=f'{fmag:.3f}', fg=TEXT)
            self._ft_mmag_lbl.config(text=f'{mmag:.4f}', fg=TEXT)
        else:
            self._ft_fmag_lbl.config(text='—', fg=TEXT_DIM)
            self._ft_mmag_lbl.config(text='—', fg=TEXT_DIM)

        # ── Estatísticas ─────────────────────────────────────────────
        for axis, cells in getattr(self, '_ft_stat_cells', {}).items():
            snap = snaps.get(axis, {})
            unit_force = axis in ('fx', 'fy', 'fz')
            casas = 3 if unit_force else 4
            for key, lbl in cells.items():
                v = snap.get(key)
                lbl.config(text='—' if v is None else f'{v:+.{casas}f}',
                           fg=TEXT_DIM if v is None else TEXT)
        n = snaps.get('fx', {}).get('n', 0)
        if getattr(self, '_ft_stats_span', None) is not None:
            span = (n / rate) if (rate and rate > 0) else None
            self._ft_stats_span.config(
                text=(f'{n} samples' if span is None
                      else f'{n} samples  ·  {span:.2f} s'))

        # ── Gravação ─────────────────────────────────────────────────
        if getattr(self, '_ft_rec_lbl', None):
            if rec_err:
                # A thread do ROS já fechou o arquivo; aqui só se conta o
                # que houve, na thread que pode mexer em widget.
                self._ft_rec_btn.config(text='●  Start recording',
                                        bg=PRIMARY)
                self._ft_rec_lbl.config(
                    text=f'Recording aborted ({rec_err}) — {rec_path}',
                    fg=DANGER)
            elif rec_rows is not None:
                self._ft_rec_lbl.config(
                    text=f'Recording → {rec_path}  ({rec_rows} rows)',
                    fg=OK)

        # ── Saúde do link ────────────────────────────────────────────
        if rate is None or not live:
            self._ft_stat_lbls['rate'].config(text='—', fg=TEXT_DIM)
        else:
            # Fora da faixa esperada é sintoma, não detalhe: baud errado,
            # cabo ruim ou taxa de fábrica diferente da configurada.
            if rate < FT_MIN_RATE_HZ:
                cor = DANGER
            elif rate > FT_MAX_RATE_HZ:
                cor = DANGER          # não cabe no link: chega picotado
            elif abs(rate - FT_NOMINAL_RATE_HZ) > 0.2 * FT_NOMINAL_RATE_HZ:
                cor = WARN
            else:
                cor = OK
            self._ft_stat_lbls['rate'].config(text=f'{rate:.1f} Hz', fg=cor)

        self._ft_stat_lbls['frames'].config(
            text=f'{n_ok}', fg=TEXT if n_ok else TEXT_DIM)
        self._ft_stat_lbls['bad'].config(
            text=f'{n_bad}', fg=DANGER if n_bad else TEXT_DIM)
        if age is None:
            self._ft_stat_lbls['age'].config(text='never', fg=TEXT_DIM)
        else:
            self._ft_stat_lbls['age'].config(
                text=f'{age * 1000:.0f} ms' if age < 10 else 'stale',
                fg=OK if live else DANGER)

        # O aviso de "dois publicadores em /ft_sensor/wrench" saiu junto com
        # o bridge do CR10 (24/08/2026): o único produtor é o ft_receiver.

        self._refresh_ft_charts(shown, live)
        self._refresh_ft_arrow(shown, live)

        self.root.after(100, self._refresh_ft_axes)
