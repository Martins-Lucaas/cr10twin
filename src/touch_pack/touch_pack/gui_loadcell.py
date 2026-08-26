"""
gui_loadcell.py — aba "6 Axes" da célula FA7155 (RS485, seis eixos).

Recorte de `palpation_gui.py`, que era uma classe só com 234 métodos. O
wizard de calibração que morava aqui saiu junto com a célula axial de 1 eixo
(XIAO + HX711), removida em 20/08/2026: a FA7155 entrega N e N·m calibrados
de fábrica e o único ajuste do host é o TARE. Por isso esta aba é de LEITURA
— seis eixos ao vivo mais a saúde do link, que é o que costuma falhar num
RS485 com conversor USB.

A referência do tare e o auto-zero NÃO moram aqui: são do `ft_receiver`, dono
exclusivo da porta. A GUI pede e exibe.
"""
from __future__ import annotations

import json
import math
import os
import time
import tkinter as tk

import numpy as np
from std_msgs.msg import Bool, Empty, Float32, String
from .constants import (
    FT_AXIS_LABELS, FT_NOMINAL_RATE_HZ, FT_MAX_RATE_HZ, FT_MIN_RATE_HZ,
    FT_RATED_FORCE_N, FT_RATED_TORQUE_NM, FT_SAFE_OVERLOAD_PCT,
    FT_SERIAL_BAUD, ft_axis_rated,
)
from .ui_helpers import (
    BG, PANEL, TEXT, TEXT_MUTED, TEXT_DIM,
    PRIMARY, PRIMARY_HV, OK, WARN, DANGER, BORDER, BTN_NEUTRAL,
    FONT_BIG, FONT_HEAD, FONT_LBL, FONT_SMALL, FONT_MONO, FONT_MONO_S,
    _shade,
)


# ═══════════════════════════════════════════════════════════════════════════
# ABA "6 AXES" — célula FA7155 (RS485, seis eixos)
# ═══════════════════════════════════════════════════════════════════════════
# Não há nada a ajustar aqui: o FA7155 entrega N e N·m calibrados de fábrica e
# o único ajuste do host é o tare. A aba existe para (a) VER os seis eixos ao
# mesmo tempo e (b) julgar a SAÚDE do link, que é o que costuma falhar num
# RS485 com conversor USB.

# Geometria da barra bipolar. O zero fica no MEIO: força de compressão e de
# tração ocupam metades opostas, então o sinal é lido pela direção, não pelo
# rótulo.
_BAR_W = 260
_BAR_H = 16


class FtAxesMixin:
    """Mixin de `PalpationGUI` — todo o estado vive no host, como no resto
    dos recortes da GUI."""

    # ── Tare (zeragem dos seis eixos) ─────────────────────────────────
    # A FA7155 vem calibrada de fábrica: não há slope/intercept, e o único
    # ajuste do host é o zero. Quem tem a janela de amostras é o ft_receiver,
    # dono da porta — a GUI pede e traduz o desfecho em frase.
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

    # ── Construção ────────────────────────────────────────────────────
    def _build_lc_axes_tab(self, root: tk.Frame) -> None:
        """Seis eixos ao vivo + saúde do link RS485."""
        self._ft_axis_widgets = {}

        # ── Saúde do link ─────────────────────────────────────────────
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

        # ── Os seis eixos ─────────────────────────────────────────────
        card_ax = self._card(root, 'Six Axes — live')

        hdr = tk.Frame(card_ax, bg=PANEL)
        hdr.pack(fill='x', pady=(4, 2))
        tk.Label(hdr, text='axis', font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
                 width=5, anchor='w').pack(side='left')
        tk.Label(hdr, text='value', font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
                 width=14, anchor='e').pack(side='left', padx=(0, 10))
        tk.Label(hdr, text=f'0 centred  ·  ends = ±full scale',
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM,
                 anchor='w').pack(side='left')

        for i, (axis, label, unit) in enumerate(FT_AXIS_LABELS):
            if i == 3:
                # Separador força/torque: são grandezas diferentes e escalas
                # diferentes; juntá-las sem marca convida a leitura errada.
                tk.Frame(card_ax, bg=BORDER, height=1).pack(fill='x', pady=6)
            self._ft_axis_widgets[axis] = self._build_ft_axis_row(
                card_ax, axis, label, unit)

        # ── Módulos e tare ────────────────────────────────────────────
        tk.Frame(card_ax, bg=BORDER, height=1).pack(fill='x', pady=(8, 6))

        mag = tk.Frame(card_ax, bg=PANEL)
        mag.pack(fill='x')
        self._ft_fmag_lbl = self._build_ft_magnitude(mag, '|F|', 'N')
        self._ft_mmag_lbl = self._build_ft_magnitude(mag, '|M|', 'N·m')

        btn_row = tk.Frame(card_ax, bg=PANEL)
        btn_row.pack(fill='x', pady=(10, 2))
        tk.Button(btn_row, text='⊘  Tare all axes',
                  command=self._lc_do_tare,
                  bg=PRIMARY, fg='white',
                  activebackground=PRIMARY_HV, activeforeground='white',
                  font=FONT_LBL, relief='flat', bd=0, padx=14, pady=6,
                  cursor='hand2').pack(side='left')
        self._ft_tare_note = tk.Label(
            btn_row,
            text='Tare zeroes the six axes at once — do it unloaded.',
            font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM)
        self._ft_tare_note.pack(side='left', padx=(12, 0))

        # ── Capacidade ────────────────────────────────────────────────
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
        """Repinta os seis eixos e a saúde do link. Reagendado sozinho.

        Roda a 10 Hz e NÃO a cada mensagem: o sensor entrega ~250 quadros/s e
        redesenhar canvas nessa taxa satura a thread do Tk — foi o que já
        travou a GUI inteira no heatmap do toque (ver _build_sensors_tab)."""
        if not getattr(self, '_ft_axis_widgets', None):
            return
        now = time.time()
        with self._lock:
            w = dict(self._ft_wrench)
            ts = self._ft_last_ts
            n_ok = self._ft_frames_ok
            n_bad = self._ft_frames_bad
            rate = self._ft_rate_hz

        age = (now - ts) if ts > 0.0 else None
        live = age is not None and age < 1.0

        # ── Eixos ────────────────────────────────────────────────────
        for axis, wid in self._ft_axis_widgets.items():
            v = w.get(axis)
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
            fmag = math.sqrt(w['fx'] ** 2 + w['fy'] ** 2 + w['fz'] ** 2)
            mmag = math.sqrt(w['mx'] ** 2 + w['my'] ** 2 + w['mz'] ** 2)
            self._ft_fmag_lbl.config(text=f'{fmag:.3f}', fg=TEXT)
            self._ft_mmag_lbl.config(text=f'{mmag:.4f}', fg=TEXT)
        else:
            self._ft_fmag_lbl.config(text='—', fg=TEXT_DIM)
            self._ft_mmag_lbl.config(text='—', fg=TEXT_DIM)

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

        self.root.after(100, self._refresh_ft_axes)
