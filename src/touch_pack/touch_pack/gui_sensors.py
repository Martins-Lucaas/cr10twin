"""gui_sensors.py — o sensor de toque: aquisição, painel ao vivo e gravação.

Duas responsabilidades que parecem separadas e não são: o que aparece na tela
e o que vai para o CSV saem da MESMA fonte, e é isso que garante que o gráfico
que o operador olhou durante o ensaio descreve as linhas que ficaram gravadas.

Pontos que não se deduzem lendo o código:

  * o `touch_source` pode não estar lá quando a GUI sobe (placa fora, plotter
    noutra máquina). A GUI insiste em segundo plano em vez de recusar — o
    ensaio de força não depende do toque;
  * a animação do painel é instrumentada e retunada porque o Tk redesenhando
    a 30 Hz com blit errado come CPU que o laço de controle precisa;
  * parar a gravação fecha os CSV de referência. Um arquivo aberto ao fechar a
    GUI perde a última linha — e a última linha é o fim do ensaio.

Recortado de `palpation_gui.py` — os métodos operam sobre `self` como antes.
"""
from __future__ import annotations

import logging
import os
import re
from typing import Any

import csv
import signal
import subprocess
import threading
import time
import tkinter as tk
from .constants import (
    FORCE_ABORT_LIMIT_N as _FORCE_ABORT_LIMIT_N,
    RUN_ADC_CSV,
    RUN_CN_CSV,
    RUN_SENSORS_CSV,
    RUN_SPIKES_CSV,
    new_run_id,
    run_dir,
    taxel_frame_to_physical,
    taxel_index_to_physical,
)
from .ui_helpers import (
    BG,
    BORDER,
    BTN_NEUTRAL,
    DANGER,
    FONT_BIG,
    FONT_HEAD,
    FONT_LBL,
    FONT_SMALL,
    OK,
    PANEL,
    TEXT,
    TEXT_DIM,
    TEXT_MUTED,
    WARN,
)
from std_msgs.msg import Float32, String
from touch_pack_msgs.msg import TouchFrame


# Figura do toque + matplotlib: import OPCIONAL. Numa máquina sem matplotlib a
# aba Sensores some, mas a GUI abre e o ensaio de força roda — o toque não é
# pré-requisito dele. `_TOUCH_PLOT_OK` guarda todo uso.
try:
    from .touch_source import TouchFigure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.animation import FuncAnimation
    _TOUCH_PLOT_OK = True
except Exception:  # pragma: no cover
    TouchFigure: Any = None
    FigureCanvasTkAgg: Any = None
    FuncAnimation: Any = None
    _TOUCH_PLOT_OK = False

from .constants import RUNS_DIR
from .lc_filter import QOS_SENSOR

log = logging.getLogger('touch_pack.palpation_gui')   # mesmo canal do host

# Ritmo da animação do touch sensor (aba Sensores) — ver _retune_touch_anim.
# O período do desenho é custo_medido/_TOUCH_ANIM_DUTY, limitado a esta faixa:
# nunca mais que 30 fps (piso de 33 ms) nem menos que 4 fps (teto de 250 ms).
_TOUCH_ANIM_MIN_MS = 33
_TOUCH_ANIM_MAX_MS = 250
_TOUCH_ANIM_DUTY = 0.40

# Regex p/ os CSVs "crus" (ADC, spikes RA/SA, cuneiformes) — gravados junto
# do sensors.csv quando o usuário aperta "Salvar dados".
_REF_SPIKE_RE = re.compile(r"idx=(\d+),adc=(\d+),t=(\d+)")
_REF_T_RE = re.compile(r"t=(\d+)")


def _rel_run(path: str | None) -> str:
    """<MODO>/<run_id>/<arquivo> para mostrar na barra de status — o
    basename sozinho ('sensors.csv') não diz de qual run se trata."""
    if not path:
        return '?'
    try:
        return os.path.relpath(path, RUNS_DIR)
    except (ValueError, TypeError):
        return os.path.basename(path)


class SensorsMixin:
    """gui_sensors.py — o sensor de toque: aquisição, painel ao vivo e gravação."""

    # Touch sensor — fonte serial + publicação ROS
    def _start_touch_source(self) -> None:
        """Abre a serial do STM32. Sem ela não há dado tátil — não existe mais
        queda para o modo rede."""
        if not _TOUCH_PLOT_OK or self._touch_source is None:
            log.info('[TOUCH] matplotlib ausente — figura desabilitada')
            return
        if self._touch_source.start():
            self._touch_serial_ok = True
            log.info('[TOUCH] serial em %s — publicando /touch_sensor/value',
                     self._touch_source.port)
        else:
            self._touch_serial_ok = False
            # ERROR, não INFO: era exatamente esta linha, em INFO e seguida de
            # um fallback mudo, que deixou o run 20260807_185000 ser gravado
            # com 2,7 quadros/s sem ninguém perceber.
            log.error('[TOUCH] SEM SENSOR DE TOQUE: %s. O transporte é a USB '
                      'do STM32 e não há caminho alternativo — confira o cabo. '
                      'Qualquer coleta iniciada agora sai SEM dado tátil.',
                      self._touch_source.error)
    def _reconcile_touch_echo_sub(self) -> None:
        """Assina /touch_sensor/value SÓ quando não há serial local.

        Esta GUI PUBLICA /touch_sensor/value a cada amostra (~1 kHz, ver
        _touch_pub_period) e ASSINAVA o mesmo tópico ao mesmo tempo. O
        assinante existe para o caso de o STM32 estar noutra máquina e alguém
        republicar o escalar na rede; com a serial local viva, ele só recebia
        o ECO da própria GUI — ~800 mensagens por segundo entregues a si
        mesma, cada uma um despertar do executor e um callback Python na
        thread do ROS.

        Isso não aparecia na thread do ROS: aparecia na do Tk, que disputa o
        GIL com ela. Medido em 27/08/2026, aba Sensores à vista, mesma figura
        e mesmo sensor: com o eco, `Text.draw` custava 0,95-1,14 ms (25 por
        quadro) e o quadro inteiro 29-39 ms; sem a thread do ROS, 0,29 ms e
        17 ms — e 0,27 ms é o que a GUI do openARM mede na mesma máquina, que
        publica o mesmo escalar mas NÃO o assina. O desenho nunca foi o
        gargalo.

        Reconciliado no hot-plug (a cada 2 s), porque a serial pode cair e o
        caminho de rede tem de voltar sozinho."""
        # getattr: este método roda ANTES de a fonte existir (o bloco de
        # assinaturas do __init__ vem antes do bloco que a cria).
        src = getattr(self, '_touch_source', None)
        serial_ok = src is not None and src.connected
        if serial_ok and self._touch_value_sub is not None:
            try:
                self.destroy_subscription(self._touch_value_sub)
            except Exception as exc:
                log.debug('destroy_subscription do eco falhou: %s', exc)
            self._touch_value_sub = None
        elif not serial_ok and self._touch_value_sub is None:
            self._touch_value_sub = self.create_subscription(
                Float32, '/touch_sensor/value', self._cb_touch_value,
                QOS_SENSOR)
    def _retry_touch_source(self) -> None:
        """Hot-plug: a cada 2 s reconcilia a fonte do toque com o hardware."""
        try:
            self._reconcile_touch_echo_sub()
            src = self._touch_source
            if src is None:
                return
            if not src.connected:
                # Nunca abriu, ou a serial caiu (replug/porta renomeada).
                src.stop()
                was_ok = self._touch_serial_ok
                self._touch_serial_ok = src.start()
                if self._touch_serial_ok:
                    log.info('[TOUCH] hot-plug: serial em %s', src.port)
                elif was_ok:
                    log.error('[TOUCH] a serial do toque CAIU (%s) — sem dado '
                              'tátil até o cabo voltar.', src.error)
        except Exception as exc:
            log.debug('retry touch source falhou: %s', exc)
        finally:
            self.root.after(2000, self._retry_touch_source)
    def _on_touch_sample(self, i_final: float) -> None:
        """Callback da thread serial: republica I_final em ROS e atualiza o
        estado interno, sem tocar em widgets Tk.
        """
        if self._touch_pub_period > 0.0:
            now = time.monotonic()
            if now - self._touch_pub_last < self._touch_pub_period:
                return
            self._touch_pub_last = now
        try:
            msg = Float32(); msg.data = float(i_final)
            self._touch_value_pub.publish(msg)
        except Exception:
            pass
        with self._lock:
            self._touch_value = float(i_final)
            self._touch_last_ts = time.time()
        # Gravação do stream força+toque a 1 kHz (se ligada) — fora do lock acima
        # porque _record_row pega self._lock por conta própria.
        if self._rec_writer is not None:
            self._record_row(i_final)
    # Aba "Sensores": todos os plots lado a lado
    def _build_sensors_tab(self, root: tk.Frame) -> None:
        """Dashboard: os quatro gráficos do touch sensor (heatmap, raster
        RA/SA, I_final, neurônio pós) embutidos via matplotlib, lado a lado
        com a leitura ao vivo da célula de carga."""
        body = tk.Frame(root, bg=BG)
        body.pack(fill='both', expand=True, padx=8, pady=8)

        # Esquerda: figura do touch sensor
        left = tk.Frame(body, bg=BG)
        left.pack(side='left', fill='both', expand=True, padx=(0, 8))

        hdr = tk.Frame(left, bg=BG); hdr.pack(fill='x')
        tk.Label(hdr, text='Touch Sensor (STM32) — Izhikevich',
                 font=FONT_HEAD, bg=BG, fg=TEXT).pack(side='left')
        self._sens_touch_status_lbl = tk.Label(
            hdr, text='', font=FONT_SMALL, bg=BG, fg=TEXT_DIM)
        self._sens_touch_status_lbl.pack(side='right')

        plot_holder = tk.Frame(left, bg=PANEL, highlightthickness=1,
                               highlightbackground=BORDER)
        plot_holder.pack(fill='both', expand=True, pady=(6, 0))
        if (_TOUCH_PLOT_OK and self._touch_source is not None
                and TouchFigure is not None):
            try:
                self._touch_figure = TouchFigure(
                    self._touch_source, facecolor=PANEL)
                self._touch_canvas = FigureCanvasTkAgg(
                    self._touch_figure.fig, master=plot_holder)
                self._touch_canvas.get_tk_widget().pack(
                    fill='both', expand=True)
                self._touch_canvas.draw()
                # blit=True: só os artistas animados são redesenhados. O
                # redraw completo custava ~80 ms por frame nesta figura (4
                # eixos + colorbar + legenda) e era pedido a cada 50 ms — o
                # laço do Tk ficava saturado e a GUI INTEIRA travava. Só é
                # válido porque os limites dos eixos são fixos: o raster usa
                # tempo relativo a agora, não absoluto (ver TouchFigure).
                #
                # O intervalo aqui é só o CHUTE INICIAL (30 fps, o teto da
                # faixa): a partir do primeiro frame ele passa a ser o custo
                # medido do desenho, em _retune_touch_anim. Um intervalo fixo
                # supõe um custo de frame fixo, e ele não é — escala com a área
                # em pixels da figura (que segue o tamanho da janela) e com a
                # grade do sensor. Os 33 ms cravados que havia aqui vinham de um
                # frame de 11,1 ms p50 / 16,4 ms p99 (10/08/2026); na mesma
                # célula com a janela maximizada e grade 5×5 o frame mede 20,2
                # ms p50 / 26,0 ms p99 (27/08/2026), e a 30 fps isso ocupava
                # quase toda a thread que pinta a GUI inteira.
                self._touch_anim = FuncAnimation(
                    self._touch_figure.fig,
                    self._touch_anim_frame,
                    init_func=self._touch_figure.init_blit,
                    interval=_TOUCH_ANIM_MIN_MS, blit=True,
                    cache_frame_data=False)
                self._touch_anim_running = True
                self._instrument_touch_blit()
            except Exception as exc:
                log.warning('[TOUCH] falha ao embutir figura: %s', exc)
                self._touch_figure = None
                self._touch_canvas = None
                self._touch_anim = None
                tk.Label(plot_holder,
                         text=f'Figure unavailable: {exc}',
                         font=FONT_LBL, bg=PANEL, fg=TEXT_DIM).pack(
                    expand=True, pady=40)
        else:
            tk.Label(plot_holder,
                     text='matplotlib/pyserial missing — '
                          'install them to see the touch charts.',
                     font=FONT_LBL, bg=PANEL, fg=TEXT_DIM).pack(
                expand=True, pady=40)

        # Direita: célula de carga ao vivo
        right = tk.Frame(body, bg=BG, width=270)
        right.pack(side='right', fill='y')
        right.pack_propagate(False)

        card = self._card(right, 'Load Cell — live')
        tk.Label(card, text='Compression Force (tare)', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(anchor='w', pady=(4, 0))
        self._sens_force_lbl = tk.Label(
            card, text='—   N', font=FONT_BIG, bg=PANEL, fg=TEXT_DIM)
        self._sens_force_lbl.pack(anchor='w', pady=(2, 2))
        self._sens_status_lbl = tk.Label(
            card, text='waiting for /load_cell/force_net',
            font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM)
        self._sens_status_lbl.pack(anchor='w')

        tk.Frame(card, bg=BORDER, height=1).pack(fill='x', pady=8)
        self._sens_raw_lbl   = self._kv(card, 'LC raw',  '—  N')
        self._sens_volt_lbl  = self._kv(card, 'LC Voltage', '—  V')
        self._sens_touch_lbl = self._kv(card, 'Toque I_final', '—')

        tk.Frame(card, bg=BORDER, height=1).pack(fill='x', pady=8)
        tk.Label(card, text='Force — last 30 s', font=FONT_SMALL,
                 bg=PANEL, fg=TEXT_MUTED, anchor='w').pack(fill='x')
        self._sens_force_spark = tk.Canvas(
            card, height=80, bg=PANEL, highlightthickness=1,
            highlightbackground=BORDER)
        self._sens_force_spark.pack(fill='x', pady=(4, 2))
    def _refresh_sensors_tab(self) -> None:
        """Loop da aba Sensores. A figura do toque é desenhada pela
        FuncAnimation (blit); aqui só a pausamos/retomamos conforme a aba
        esteja visível (poupa CPU) e atualizamos os números da célula."""
        try:
            nb = getattr(self, '_nb', None)
            frame = self._sensors_tab_frame
            visible = (nb is not None and frame is not None
                       and str(nb.select()) == str(frame))
            # Anima sempre que a aba estiver à vista — INCLUSIVE durante a
            # coleta, que é justamente quando se quer olhar o sinal. O
            # `not self._experiment_active()` que havia aqui datava de quando
            # a fonte entregava 2,7 quadros/s por UDP; medido em 10/08/2026
            # sobre o stream serial real (834 quadros/s), animar não custa
            # amostra nenhuma: 800 quadros/s com e sem animação, e o
            # `snapshot()` só segura o lock da fonte 0,26 ms (p99 0,60 ms).
            self._set_touch_anim(visible)
            if visible:
                # Daqui, e não de dentro do callback da animação: ver a
                # advertência sobre o timer em _retune_touch_anim.
                self._retune_touch_anim()
                self._update_sensors_panel()
        finally:
            self._sensors_after = self.root.after(
                80, self._refresh_sensors_tab)
    def _set_touch_anim(self, run: bool) -> None:
        """Liga/desliga a animação do touch sensor (idempotente)."""
        anim = getattr(self, '_touch_anim', None)
        if anim is None or run == self._touch_anim_running:
            return
        try:
            if run:
                anim.resume()
            else:
                anim.pause()
            self._touch_anim_running = run
            # O frame em curso não fecha atravessando uma pausa: sem isto o
            # próximo blit mediria também o tempo com a aba escondida.
            self._touch_frame_t0 = 0.0
            self._touch_frame_cost = None
        except Exception as exc:
            log.debug('touch anim toggle falhou: %s', exc)
    def _instrument_touch_blit(self) -> None:
        canvas = self._touch_canvas
        if canvas is None:
            return
        orig_blit = canvas.blit

        def timed_blit(bbox=None):
            orig_blit(bbox)
            t0 = self._touch_frame_t0
            if t0:
                self._touch_frame_cost = time.perf_counter() - t0

        canvas.blit = timed_blit
    def _touch_anim_frame(self, *args):
        """Callback da FuncAnimation: fecha a medição do frame anterior e
        desenha o atual. A média é exponencial (α=0,2) porque o custo oscila
        com a quantidade de spikes na janela e não se quer o intervalo
        pulando a cada frame."""
        cost = self._touch_frame_cost
        if cost is not None:
            self._touch_frame_cost = None
            ema = self._touch_frame_ema
            self._touch_frame_ema = (cost if ema is None
                                     else 0.8 * ema + 0.2 * cost)
        self._touch_frame_t0 = time.perf_counter()
        return self._touch_figure.update(*args)
    def _retune_touch_anim(self) -> None:
        """Ajusta o intervalo da animação ao custo MEDIDO do frame, para o
        desenho do toque nunca passar de _TOUCH_ANIM_DUTY da thread do Tk.

        Medido em 27/08/2026 nesta célula, janela maximizada e grade 5×5
        (figura de 1236×875 px), o frame custava 25,9 ms p50 / 33,6 ms p99
        contra o intervalo fixo de 33 ms: a thread que pinta a GUI INTEIRA
        ficava ocupada ~80–100% do tempo só com esta figura, e a aba inteira
        respondia com atraso. Com o período em custo/_TOUCH_ANIM_DUTY sobra
        sempre ~60% do laço para o resto, em qualquer máquina e qualquer
        tamanho de janela — que é a variável que muda o custo.

        SÓ PODE SER CHAMADO DE FORA do callback da animação. O setter de
        `interval` reinicia o timer, e o TimerTk reagenda outro ao fim do
        callback: chamado lá dentro, os dois se somam e a taxa dobra a cada
        frame. Daqui (laço `after` da aba) há um único timer pendente."""
        anim = getattr(self, '_touch_anim', None)
        src = getattr(anim, 'event_source', None) if anim is not None else None
        ema = self._touch_frame_ema
        # `anim is None` explícito: `src` só é não-None quando `anim` também
        # é, mas isso vem da expressão acima e não sobrevive à leitura de
        # `anim._interval` lá embaixo.
        if anim is None or src is None or not ema:
            return
        cost_ms = ema * 1e3
        period = min(max(cost_ms / _TOUCH_ANIM_DUTY, _TOUCH_ANIM_MIN_MS),
                     _TOUCH_ANIM_MAX_MS)
        want = max(int(period - cost_ms), 1)
        cur = getattr(src, 'interval', 0)
        # Histerese: mexer no interval reinicia o timer do Tk, então só quando
        # a correção for de verdade — não a cada 80 ms.
        if cur <= 0 or abs(want - cur) / cur > 0.20:
            # Os DOIS, nesta ordem: `TimedAnimation._step` reescreve
            # `event_source.interval = self._interval` ao fim de CADA frame,
            # então mexer só no timer dura um frame e some (verificado no
            # matplotlib 3.5.1 e ainda presente nas 3.x). `_interval` é o
            # campo que a animação restaura; o setter do timer é o que faz o
            # novo valor valer já no próximo disparo.
            anim._interval = want
            src.interval = want
    def _touch_source_status(self, scalar_fresh: bool) -> tuple[str, str]:
        """Texto/cor honestos da fonte do toque, do estado AO VIVO da fonte."""
        src = self._touch_source
        if src is not None and src.connected:
            base = f'serial {src.port}'
            if src.is_fresh():
                # Frames truncados não aparecem em lugar nenhum se não forem
                # ditos aqui: o descarte é correto, mas uma coleta que perdeu
                # 19% dos frames não pode parecer verde.
                bad, ok = src.frames_bad, src.frames_ok
                total = bad + ok
                if bad and total:
                    pct = 100.0 * bad / total
                    return (f'{base} — {pct:.1f}% dos frames perdidos '
                            f'({bad}/{total})', WARN if pct < 1.0 else DANGER)
                return base, OK
            # Ligado mas mudo: porta serial errada ou STM mudo.
            return f'{base} (no data)', WARN
        if scalar_fresh:
            return 'via /touch_sensor/value', OK
        # Sem serial não há tátil nenhum. Vermelho, não cinza: em cinza isto
        # passa por "ainda não ligou" e a coleta sai vazia (ver 07/08/2026).
        return 'SEM SENSOR DE TOQUE (confira o cabo USB)', DANGER
    def _update_sensors_panel(self) -> None:
        """Atualiza os números da célula de carga + sparkline na aba Sensores."""
        with self._lock:
            f_net     = self._lc_force_net
            lc_ts     = self._lc_force_net_ts
            f_raw     = self._lc_force_raw
            tare_done = self._lc_tare_done
            touch_val = self._touch_value
            touch_ts  = self._touch_last_ts

        has_data = lc_ts > 0.0 and (time.time() - lc_ts) < 3.0
        if has_data:
            if not tare_done:
                color, status = WARN, 'tare not done'
            elif f_net > _FORCE_ABORT_LIMIT_N * 0.9:
                color, status = DANGER, f'near the limit ({_FORCE_ABORT_LIMIT_N:.0f} N)'
            elif self._contact_indicator(f_net):
                color, status = OK, 'in contact'
            else:
                color, status = TEXT_MUTED, 'no contact'
            self._sens_force_lbl.config(text=f'{f_net:+6.2f}  N', fg=color)
            self._sens_status_lbl.config(text=status, fg=color)
            self._sens_raw_lbl.config(text=f'{f_raw:+6.2f} N')
            self._sens_volt_lbl.config(text=f'{f_raw - f_net:+6.3f} N')
        else:
            self._sens_force_lbl.config(text='—   N', fg=TEXT_DIM)
            self._sens_status_lbl.config(
                text='waiting for /load_cell/force_net', fg=TEXT_DIM)
            self._sens_raw_lbl.config(text='—  N')
            self._sens_volt_lbl.config(text='—  N')

        touch_fresh = touch_ts > 0.0 and (time.time() - touch_ts) < 3.0
        self._sens_touch_lbl.config(
            text=f'{touch_val:+.3f}' if touch_fresh else '—')

        label, fg = self._touch_source_status(touch_fresh)
        self._sens_touch_status_lbl.config(text=label, fg=fg)

        self._draw_force_spark(self._sens_force_spark)
    def _toggle_recording(self) -> None:
        if self._rec_fh is not None:
            self._stop_recording()
        else:
            self._start_recording()
    def _start_recording(self, run_id: str | None = None,
                         mode: str | None = None,
                         auto: bool = False) -> None:
        """Abre o sensors.csv + os CSVs crus NA PASTA DO RUN.

        `run_id`/`mode` vindos do start são os mesmos que o palpation_logger
        recebe pela mensagem, e é isso que põe estes arquivos na mesma pasta
        que o samples.csv dele. Sem eles (botão "Record data" fora de um
        run) a gravação vai para RECORDING/<carimbo novo>.

        `auto` marca que quem abriu foi o início da palpação, e não o botão —
        só a gravação automática é fechada sozinha no fim do run."""
        try:
            out_dir = run_dir(mode or '', run_id or new_run_id())
            path = os.path.join(out_dir, RUN_SENSORS_CSV)
            fh = open(path, 'w', newline='')
            writer = csv.writer(fh)
            writer.writerow(self._rec_header)
        except OSError as exc:
            self._set_rec_status(f'failed to open CSV: {exc}', DANGER)
            return
        # CSVs crus (ADC / spikes / cuneiformes), na MESMA pasta.
        # Best-effort: se algum falhar, o sensors.csv segue gravando.
        ref = self._open_reference_csvs(out_dir)
        with self._lock:
            self._rec_fh = fh
            self._rec_writer = writer
            self._rec_path = path
            self._rec_t0 = time.time()
            self._rec_count = 0
            self._rec_auto = bool(auto)
            (self._ref_adc_fh, self._ref_adc_writer,
             self._ref_spike_fh, self._ref_spike_writer,
             self._ref_cn_fh, self._ref_cn_writer) = ref
        self.rec_btn.config(text='■ Stop recording', bg=DANGER, fg='white')
        self._set_rec_status(f'recording → {_rel_run(path)}', OK)
        self._rec_after = self.root.after(
            self._REC_STATUS_MS, self._recording_status_tick)
    def _record_row(self, i_final: float) -> None:
        """Grava UMA linha do stream força+toque. Chamado por _on_touch_sample
        (thread serial, ~1 kHz) — é isto que dá ao sensors.csv a taxa de 1 kHz.
        """
        if self._rec_writer is None:
            return   # fast-path sem lock; reconferido sob o lock abaixo
        now = time.time()
        if self._touch_source is not None and self._touch_source.connected:
            # Tensões + relógio do STM32 sob o MESMO lock: o timestamp do
            # firmware (1 kHz) é o que data a amostra na planilha.
            volt, t_stm = self._touch_source.latest_voltages_and_time()
            volt_cols = [f'{volt[r, c]:.4f}'
                         for r in range(self._touch_rows)
                         for c in range(self._touch_cols)]
        else:
            t_stm = 0.0
            volt_cols = [''] * self._touch_taxels
        with self._lock:
            if self._rec_writer is None:
                return   # _stop_recording correu entre o fast-path e aqui
            f_net    = self._lc_force_net
            lc_bruto = self._lc_force_raw       # pré-tare, compressão positiva
            ftw = dict(self._ft_wrench)
            ft_age_ms = ((now - self._ft_last_ts) * 1000.0
                         if self._ft_last_ts > 0.0 else -1.0)
            try:
                self._rec_writer.writerow([
                    f'{now - self._rec_t0:.4f}', f'{now:.4f}',
                    f'{t_stm:.6f}',
                    f'{f_net:.4f}', f'{lc_bruto:.4f}',
                    f'{float(i_final):.4f}',
                    f'{ftw["fx"]:.5f}', f'{ftw["fy"]:.5f}', f'{ftw["fz"]:.5f}',
                    f'{ftw["mx"]:.6f}', f'{ftw["my"]:.6f}', f'{ftw["mz"]:.6f}',
                    f'{ft_age_ms:.1f}',
                    *volt_cols,
                ])
                self._rec_count += 1
                # Flush a cada ~1 s (1000 amostras @ 1 kHz).
                if self._rec_count % 1000 == 0 and self._rec_fh is not None:
                    self._rec_fh.flush()
            except (ValueError, OSError) as exc:
                log.warning('falha ao gravar amostra sincronizada: %s', exc)
    # CSVs "crus" (ADC / spikes / cuneiformes), iguais ao standalone
    def _open_reference_csvs(self, out_dir: str) -> tuple:
        """Abre os três CSVs crus com o cabeçalho do plotter de coleta
        standalone (adc, spikes, cuneiformes) e devolve a tupla
        (adc_fh, adc_writer, spike_fh, spike_writer, cn_fh, cn_writer).
        """
        try:
            adc_fh = open(os.path.join(out_dir, RUN_ADC_CSV), 'w', newline='')
            adc_w = csv.writer(adc_fh)
            adc_w.writerow(['tempo']
                           + [f'taxel_{i}' for i in range(self._touch_taxels)])
            spike_fh = open(os.path.join(out_dir, RUN_SPIKES_CSV),
                            'w', newline='')
            spike_w = csv.writer(spike_fh)
            spike_w.writerow(['tempo', 'tipo', 'idx', 'adc'])
            cn_fh = open(os.path.join(out_dir, RUN_CN_CSV),
                         'w', newline='')
            cn_w = csv.writer(cn_fh)
            cn_w.writerow(['tempo', 'tipo'])
        except OSError as exc:
            log.warning('falha ao abrir CSVs crus: %s', exc)
            return (None, None, None, None, None, None)
        return (adc_fh, adc_w, spike_fh, spike_w, cn_fh, cn_w)
    def _on_raw_lines(self, lines: list) -> None:
        """Tap das linhas brutas do firmware (thread serial, ~1 kHz por chunk)."""
        # Republica o tátil completo em ROS SÓ quando há
        # experimento/gravação (_experiment_active): é o que o
        # palpation_logger assina para juntar taxels+eventos no CSV do
        # experimento, e ele SÓ grava durante um run.
        if self._experiment_active():
            for line in lines:
                self._publish_tactile_line(line.strip())
        if self._ref_adc_writer is None:
            return  # fast-path: nada a gravar nos CSVs crus
        with self._lock:
            adc_w = self._ref_adc_writer
            spike_w = self._ref_spike_writer
            cn_w = self._ref_cn_writer
            if adc_w is None:
                return  # _stop_recording correu entre o fast-path e aqui
            try:
                for line in lines:
                    self._write_reference_line(line.strip(), adc_w, spike_w, cn_w)
            except (ValueError, OSError) as exc:
                log.warning('falha ao gravar CSV cru: %s', exc)
    def _parse_adc_frame(self, line: str) -> tuple[list, int] | None:
        """Extrai os N taxels + o t_us de 'ADC,v0,...,vN-1,t=micros'.

        Devolve (taxels, t_us), ou None — e CONTA — se a linha não trouxer
        exatamente
        ``self._touch_taxels`` inteiros. Um frame truncado não é um frame
        incompleto do qual se aproveita o começo: quando a serial perde bytes,
        o que sobra depois do buraco é *emendado* de outro frame, então os
        valores a partir do ponto de corte estão trocados de taxel. Medido na
        coleta de 07/08/2026: no frame parcial o erro contra o último frame
        completo passa de ~30 ADC em taxel_0..3 para >1000 ADC em taxel_16.
        Descartar o frame inteiro é a única leitura honesta.

        Note que os tokens são convertidos SEM filtro: um valor corrompido no
        meio derruba a linha (ValueError, tratado no chamador) em vez de
        encurtar silenciosamente a lista — era assim que uma linha suja virava
        um frame "curto" e depois uma linha de CSV desalinhada."""
        parts = line.split(',')
        try:
            vals = [int(v.strip()) for v in parts[1:-1]]
        except ValueError:
            vals = None
        # t_us do próprio frame. Um `t=` ilegível NÃO derruba o frame: os
        # taxels continuam válidos e o consumidor trata t_us=0 como ausente.
        try:
            t_us = int(parts[-1].replace('t=', '').strip()) & 0xFFFFFFFF
        except (ValueError, IndexError):
            t_us = 0
        if vals is not None and len(vals) == self._touch_taxels:
            self._adc_pub_ok += 1
            # Sai na numeração FÍSICA (taxel 0 = 00): é o que vai para o
            # TouchFrame e daí para as colunas taxel_* do samples.csv.
            return (taxel_frame_to_physical(
                vals, self._touch_rows, self._touch_cols), t_us)
        self._adc_pub_bad += 1
        now = time.monotonic()
        if now - self._adc_bad_warn_t > 2.0:
            self._adc_bad_warn_t = now
            total = self._adc_pub_ok + self._adc_pub_bad
            got = 'ilegível' if vals is None else f'{len(vals)}'
            self.get_logger().warn(
                f'[TOUCH] frame ADC corrompido descartado '
                f'({got}/{self._touch_taxels} taxels); '
                f'{self._adc_pub_bad} de {total} frames perdidos neste run — '
                f'bytes perdidos na serial.')
        return None
    def _publish_tactile_line(self, line: str) -> None:
        """Parseia UMA linha do firmware e republica em ROS para o logger:
        frame ADC → TouchFrame; cada spike/cuneiforme → String com o tipo
        (RA|SA|CN_MM|CN_RA|CN_SA). Best-effort: linha malformada é ignorada."""
        if not line:
            return
        try:
            if line.startswith('ADC'):
                parsed = self._parse_adc_frame(line)
                if parsed is not None:
                    vals, t_us = parsed
                    msg = TouchFrame()
                    msg.taxels = vals
                    msg.t_us = t_us
                    msg.rows = self._touch_rows
                    msg.cols = self._touch_cols
                    self._touch_frame_pub.publish(msg)
            elif (line.startswith('CN_MM') or line.startswith('CN_RA')
                  or line.startswith('CN_SA')):
                self._touch_event_pub.publish(String(data=line[:5]))
            elif line.startswith('RA') or line.startswith('SA'):
                self._touch_event_pub.publish(String(data=line[:2]))
        except (ValueError, IndexError):
            pass
    def _write_reference_line(self, line, adc_w, spike_w, cn_w) -> None:
        """Parseia UMA linha e grava no CSV cru correspondente (sob self._lock)."""
        if not line:
            return
        if line.startswith('ADC'):
            parts = line.split(',')
            try:
                tstamp = int(parts[-1].replace('t=', '').strip()) / 1e6
            except (ValueError, IndexError):
                return
            vals = [int(v.strip()) for v in parts[1:-1] if v.strip().isdigit()]
            if len(vals) != self._touch_taxels:
                return
            adc_w.writerow([tstamp, *taxel_frame_to_physical(
                vals, self._touch_rows, self._touch_cols)])
        elif line.startswith('CN_MM') or line.startswith('CN_RA') \
                or line.startswith('CN_SA'):
            m = _REF_T_RE.search(line)
            t = int(m.group(1)) / 1e6 if m else 0.0
            cn_w.writerow([t, line[:5]])
        elif line.startswith('RA') or line.startswith('SA'):
            m = _REF_SPIKE_RE.search(line)
            if m:
                spike_w.writerow([int(m.group(3)) / 1e6, line[:2],
                                  taxel_index_to_physical(
                                      int(m.group(1)),
                                      self._touch_rows, self._touch_cols),
                                  int(m.group(2))])
    def _recording_status_tick(self) -> None:
        """Só atualiza o rótulo de status (na thread Tk); as linhas são gravadas
        pelo callback do toque, não aqui."""
        if self._rec_fh is None:
            return
        self._set_rec_status(
            f'recording {self._rec_count} samples → '
            f'{os.path.basename(self._rec_path or "?")}', OK)
        self._rec_after = self.root.after(
            self._REC_STATUS_MS, self._recording_status_tick)
    def _stop_recording(self) -> None:
        if self._rec_after is not None:
            try:
                self.root.after_cancel(self._rec_after)
            except Exception:
                pass
            self._rec_after = None
        # Zera o writer SOB o lock: a thread serial (_record_row) o checa sob o
        # mesmo lock, então depois daqui ela não escreve mais e podemos fechar.
        with self._lock:
            fh = self._rec_fh
            path = self._rec_path
            n = self._rec_count
            self._rec_fh = None
            self._rec_writer = None
            self._rec_path = None
            self._rec_auto = False
            ref_fhs = [self._ref_adc_fh, self._ref_spike_fh, self._ref_cn_fh]
            self._ref_adc_fh = self._ref_adc_writer = None
            self._ref_spike_fh = self._ref_spike_writer = None
            self._ref_cn_fh = self._ref_cn_writer = None
        if fh is not None:
            try:
                fh.flush(); fh.close()
            except OSError:
                pass
        for rfh in ref_fhs:
            if rfh is not None:
                try:
                    rfh.flush(); rfh.close()
                except OSError:
                    pass
        try:
            self.rec_btn.config(
                text='●  Record data (force+touch)', bg=BTN_NEUTRAL, fg=TEXT)
            self._set_rec_status(
                f'saved: {n} samples to {_rel_run(path)}', TEXT_MUTED)
        except tk.TclError:
            pass
    def _set_rec_status(self, text: str, color: str) -> None:
        lbl = getattr(self, 'rec_status_lbl', None)
        if lbl is not None:
            try:
                lbl.config(text=text, fg=color)
            except tk.TclError:
                pass
    def _cb_touch_value(self, msg: Float32) -> None:
        """Recebe /touch_sensor/value de um receptor EXTERNO (touch_receiver,
        UDP 8081). Quando a GUI lê a serial diretamente, ela é a própria
        publicadora — ignoramos o eco para não processar o loopback nem
        sobrescrever o valor já atualizado (a taxa limitada) em
        _on_touch_sample."""
        if self._touch_source is not None and self._touch_source.connected:
            return
        with self._lock:
            self._touch_value = float(msg.data)
            self._touch_last_ts = time.time()
    def _spawn_touch_receiver(self) -> None:
        """Inicia o touch_receiver_node (UDP 8081) junto com o force_receiver.
        Best-effort: o touch sensor é opcional — falha aqui não bloqueia a
        célula de carga; o painel apenas fica em 'aguardando'."""
        if self._touch_rx_proc is not None and self._touch_rx_proc.poll() is None:
            return
        try:
            if self.count_publishers('/touch_sensor/value') > 0:
                return      # já existe um receptor (launch) — não duplicar
        except Exception:
            pass
        try:
            self._touch_rx_proc = subprocess.Popen(
                ['ros2', 'run', 'touch_pack', 'touch_receiver'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                start_new_session=True)
        except FileNotFoundError:
            self._touch_rx_proc = None
            return

        def _pipe_log(proc=self._touch_rx_proc):
            assert proc.stdout is not None   # criado com stdout=PIPE acima
            for raw in proc.stdout:
                log.info('[TOUCH-RX] %s',
                         raw.decode('utf-8', errors='replace').rstrip())
        threading.Thread(target=_pipe_log, daemon=True,
                         name='touch-rx-log').start()
    def _kill_touch_receiver(self) -> None:
        proc = self._touch_rx_proc
        self._touch_rx_proc = None
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except OSError:
                    pass
            except OSError:
                pass
    def _draw_touch_spark(self) -> None:
        """Redesenha o gráfico do touch sensor (Canvas puro, 10 Hz) —
        mesmo desenho do sparkline da célula, sem linha de setpoint e com
        autoescala plena (a unidade do STM32 é arbitrária)."""
        cv = getattr(self, 'touch_spark_canvas', None)
        if cv is None:
            return
        try:
            w = cv.winfo_width()
            h = cv.winfo_height()
            cv.delete('all')
        except tk.TclError:
            return
        if w <= 10 or h <= 10:
            return
        now = time.time()
        window = 30.0
        pts = [(t, v) for t, v in self._touch_spark_data if now - t <= window]
        vals = [v for _, v in pts]
        v_hi = max(vals) if vals else 1.0
        v_lo = min(vals + [0.0]) if vals else 0.0
        rng = max(v_hi - v_lo, 1e-3)

        def xy(t: float, v: float) -> tuple[float, float]:
            x = w - (now - t) / window * w
            y = (h - 4) - (v - v_lo) / rng * (h - 8)
            return x, y

        y_zero = xy(now, 0.0)[1]
        cv.create_line(0, y_zero, w, y_zero, fill=BORDER)
        if len(pts) >= 2:
            coords: list[float] = []
            for t, v in pts:
                coords.extend(xy(t, v))
            cv.create_line(*coords, fill=OK, width=2)
