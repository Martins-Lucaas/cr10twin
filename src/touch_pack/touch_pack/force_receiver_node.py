"""force_receiver_node.py — Driver da célula axial de 100 kg (XIAO + HX711).

Papel: dono EXCLUSIVO da porta USB do XIAO (uma tty admite um leitor só) e a
única fonte de `/load_cell/force_net` quando roda. NUNCA suba este nó junto
com o `ft_receiver`: dois publicadores no tópico da malha de segurança fazem o
explorer regular contra a MÉDIA de duas células.

O que muda em relação ao `ft_receiver` (FA7155 de 6 eixos):

* Um eixo só. Não há wrench nem momentos — o que sai é a força axial, e por
  isso não existe aqui o par `ft_force_axis`/`ft_force_sign`: a compressão
  positiva vem do SINAL DO SLOPE, medido na calibração (que é feita em
  compressão, massas padrão sobre a célula apontada para cima).
* HÁ arquivo de calibração, e sem ele não há força. `slope`/`intercept` saem
  de `sensors/load_cell_calib.json` (ver `constants.LC_CALIB_FILE`), o mesmo
  arquivo versionado que o wizard da aba Calibration reescreve, e
  `/load_cell/calibrated` significa literalmente "existe calibração válida
  carregada" — não "chegam quadros".
* O que sai em `/load_cell/voltage` é TENSÃO (V) de verdade, e os campos
  `voltage_raw`/`voltage` da LoadCellSample também. As colunas
  `lc_voltage_raw_v`/`lc_voltage_v` do CSV são volts nos runs feitos com esta
  célula — ao contrário dos runs com a FA7155, onde carregam newtons.
* Há DOIS zeros e eles são coisas diferentes. `/load_cell/tare` é o tare do
  HOST (subtração em software, some ao reiniciar o nó). `/load_cell/rezero` é
  o zero do FIRMWARE: vira o byte `'Z'` no fio e refaz o offset de boot dentro
  do MCU, que é o que tira a deriva térmica da ponte da conta.

Tudo a jusante de `/load_cell/force_net` é indiferente a qual célula está no
cabo: mesma convenção (compressão POSITIVA, tare aplicado), mesmo filtro
mediana + One-Euro do `lc_filter`, mesmos 15 N de aborto.
"""
from __future__ import annotations

import collections
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Empty, String
from touch_pack_msgs.msg import LoadCellSample, PalpationStatus

from .constants import (
    LC_CALIB_FILE,
    LC_CALIB_SOURCE,
    LC_CALIB_SHARED_SOURCES,
    lc_calib_fingerprint,
    lc_load_calibration,
    LC_FS_VOLTAGE_V,
    LC_MIN_RATE_HZ,
    LC_NOMINAL_RATE_HZ,
    LC_NOMINAL_V_PER_N,
    LC_SERIAL_BAUD,
    lc_force_n,
)
from .lc_filter import _LoadCellFilter, QOS_SENSOR
from .lc_serial import LoadCellSerialSource, detect_lc_serial_port


# Janela do tare/auto-tare, em SEGUNDOS — e não em amostras: o mesmo número
# tem de valer com o pino RATE do HX711 em GND (10 Hz) e em DVDD (80 Hz).
_TARE_WIN_S = 2.0
# Constante de tempo do auto-zero. 4 s: bem mais lento que qualquer contato
# real (o mais lento do explorer é o HOLD, na casa do segundo) e bem mais
# rápido que a deriva térmica da ponte, que é de minutos.
_AUTOZERO_TAU_S = 4.0


class ForceReceiverNode(Node):

    def __init__(self):
        super().__init__('force_receiver')

        # ── Contrato da célula (o que a modulação consome) ────────────
        self._voltage_pub = self.create_publisher(
            Float32, '/load_cell/voltage', QOS_SENSOR)
        self._force_pub = self.create_publisher(
            Float32, '/load_cell/force', QOS_SENSOR)
        self._force_net_pub = self.create_publisher(
            Float32, '/load_cell/force_net', QOS_SENSOR)
        self._sample_pub = self.create_publisher(
            LoadCellSample, '/load_cell/sample', QOS_SENSOR)
        self._sample_net_pub = self.create_publisher(
            LoadCellSample, '/load_cell/sample_net', QOS_SENSOR)
        self._calib_pub = self.create_publisher(
            Bool, '/load_cell/calibrated', 10)
        self._tared_pub = self.create_publisher(Bool, '/load_cell/tared', 10)
        self._tare_result_pub = self.create_publisher(
            String, '/load_cell/tare_result', 10)

        # ── Parâmetros ────────────────────────────────────────────────
        port = str(self.declare_parameter('lc_serial_port', '').value).strip()
        baud = int(self.declare_parameter('lc_baud', LC_SERIAL_BAUD).value)
        # Só dimensiona janelas e o passo do auto-zero: o dt REAL de cada
        # amostra vem do `t_us` do firmware, então errar este número não
        # deforma o filtro.
        rate = float(self.declare_parameter(
            'lc_rate_hz', LC_NOMINAL_RATE_HZ).value)
        self._calib_path = str(self.declare_parameter(
            'lc_calib_path', LC_CALIB_FILE).value).strip() or LC_CALIB_FILE

        self._rate_nom = max(rate, 1.0)
        # Janelas em TEMPO, não em amostras: o mesmo código serve para o
        # HX711 a 10 Hz (RATE em GND) e a 80 Hz (RATE em DVDD), e trocar o
        # jumper não muda o significado de "janela de 2 s".
        self._capture_win_n = max(8, int(self._rate_nom * _TARE_WIN_S))
        # 4× a janela do tare: sobra para o auto-tare olhar uma janela cheia
        # sem competir com o tare pedido pelo botão.
        self._buf: collections.deque = collections.deque(
            maxlen=self._capture_win_n * 4)
        # Passo do auto-zero para tau ≈ _AUTOZERO_TAU_S, qualquer que seja a
        # taxa. Cravar um passo por amostra faria o auto-zero ser 8× mais
        # rápido só por alguém ter mudado o pino RATE.
        self._autozero_rate = 1.0 / (_AUTOZERO_TAU_S * self._rate_nom)

        # ── Estado ────────────────────────────────────────────────────
        self._filter = _LoadCellFilter()
        # Escala V/N do termo adaptativo. O default do lc_filter é 1,0 porque
        # ele nasceu para a FA7155, que já entrega newtons; aqui a entrada é
        # tensão de ponte, e sem a sensibilidade certa o beta do One-Euro
        # degenera num passa-baixa fixo. Vale o NOMINAL até a calibração
        # chegar, e o slope medido depois dela.
        self._filter.set_sensitivity(LC_NOMINAL_V_PER_N)
        self._last_t_us: int | None = None

        self._lock = threading.Lock()
        self._slope: float = 0.0
        self._intercept: float = 0.0
        self._calib_mtime: float = 0.0
        self._tare = 0.0
        self._tare_done = False

        self._rx_lines = 0
        self._rate_warned = False
        self._last_counts = (0, 0, 0)   # bad_lines, bad_values, dropped
        self._last_heartbeats = 0
        self._saturated = 0

        self._phase: str = ''
        self.create_subscription(PalpationStatus, '/palpation/status',
                                 self._on_palpation_status, 10)
        self.create_subscription(Empty, '/load_cell/tare',
                                 self._on_tare_request, 10)
        # Zero do FIRMWARE. O tare do host é o de cima; este vai no fio.
        self.create_subscription(Empty, '/load_cell/rezero',
                                 self._on_rezero, 10)

        self._reload_calibration()

        self._serial = LoadCellSerialSource(port=(port or None), baud=baud,
                                            on_sample=self._on_sample)
        detected = port or detect_lc_serial_port() or \
            '(procurando o XIAO na USB)'
        if not self._serial.start():
            self.get_logger().error(
                f'Transporte serial indisponível ({self._serial.error}). '
                'Instale pyserial (pip install pyserial) e reinicie o nó.')
        self.get_logger().info(
            f'ForceReceiver: {detected} @ {baud} | {rate:.0f} Hz nominais | '
            f'calibração: {self._calib_desc()}')

        self._last_rx_warn = False
        self.create_timer(1.0, self._publish_status)
        self.create_timer(5.0, self._check_link)
        self.create_timer(5.0, self._reload_calibration)
        self.create_timer(10.0, self._report_link_health)

    # ── Calibração ────────────────────────────────────────────────────
    def _calib_desc(self) -> str:
        with self._lock:
            slope, intercept = self._slope, self._intercept
        if slope:
            # A IMPRESSÃO é o que torna "a mesma calibração em qualquer
            # computador" verificável: duas máquinas com os mesmos oito hex
            # medem com a mesma reta e os mesmos pontos. A origem diz se ela
            # se propaga — `config` é local àquela máquina.
            aviso = ('' if LC_CALIB_SOURCE in LC_CALIB_SHARED_SOURCES else
                     ' — origem LOCAL desta máquina, NÃO se propaga para as '
                     'outras; a fonte compartilhada é sensors/ no repo')
            return (f'[{lc_calib_fingerprint(self._calib_path)}] '
                    f'slope {slope:.6e} V/N, V₀ {intercept:+.6e} V '
                    f'(origem: {LC_CALIB_SOURCE}{aviso})')
        # Ausência não é um detalhe: sem reta este nó não publica
        # /load_cell/force_net, e o explorer recusa o ensaio por leitura
        # ausente. Dizer a CONSEQUÊNCIA junto com o caminho é o que separa
        # "falta calibrar" de "a placa morreu" na cabeça de quem lê o log.
        extra = ('' if LC_CALIB_SOURCE == 'repo' else
                 ' — o pacote está instalado FORA da árvore do repo, então '
                 'este caminho é o do ~/.config e não o sensors/ versionado; '
                 'copie a calibração para lá ou rode do repo')
        return (f'AUSENTE ({self._calib_path}{extra}). Sem ela NÃO haverá '
                '/load_cell/force_net e o ensaio será recusado por leitura '
                'ausente — calibre na aba Load Cell → Calibration.')

    def _reload_calibration(self) -> None:
        """Relê o JSON quando ele muda no disco.

        Existe para o wizard da GUI valer SEM reiniciar o nó: gravar a
        calibração e ter de derrubar o driver (dono da porta) para ela pegar
        era o caminho mais curto para calibrar e sair medindo com a reta
        velha.
        """
        try:
            mtime = os.path.getmtime(self._calib_path)
        except OSError:
            return
        if mtime == self._calib_mtime:
            return
        cal = lc_load_calibration(self._calib_path)
        self._calib_mtime = mtime
        if cal is None:
            self.get_logger().warn(
                f'{self._calib_path} existe mas não traz um slope utilizável '
                '— mantendo a calibração anterior.')
            return
        slope, intercept, _pontos = cal
        with self._lock:
            self._slope, self._intercept = slope, intercept
        # O termo adaptativo do One-Euro precisa saber quantos volts valem 1 N
        # para o beta continuar sendo Hz por (N/s). Sem isto o filtro degenera
        # num passa-baixa fixo e a resposta ao contato fica lenta.
        self._filter.set_sensitivity(abs(slope))
        self.get_logger().info(f'Calibração carregada: {self._calib_desc()}')

    # ── Fase da palpação (só para gatear o auto-zero) ─────────────────
    _AUTOZERO_PHASES = ('', 'IDLE', 'DONE', 'ABORTED')

    def _on_palpation_status(self, msg) -> None:
        self._phase = str(msg.phase or '')

    # ── TARE ──────────────────────────────────────────────────────────
    # Estabilidade por DERIVA (mediana da 2ª metade vs a 1ª) e não por
    # pico-a-pico: o ptp cresce com o tamanho da janela mesmo num sinal
    # perfeitamente estacionário, então ele recusaria tares bons só por a
    # janela ser longa.
    _TARE_STABLE_N = 0.10
    # Auto-tare de partida: além de estável, o repouso tem de estar PERTO do
    # V₀ da calibração. É a guarda que o ft_receiver não tem como fazer — sem
    # ela, ligar o nó com a ponteira apoiada na mesa zeraria a força de apoio
    # e o explorer desceria contra um contato que já existia.
    _AUTOTARE_MAX_N = 2.0
    # Auto-zero lento: cancela deriva térmica sem comer força real. Só atua em
    # repouso E dentro da banda morta — as duas condições, senão ele zeraria o
    # próprio contato durante um HOLD.
    _AUTOZERO_BAND_N = 0.30

    @staticmethod
    def _window_drift(win: list[float]) -> float:
        """Deriva da janela: |mediana da 2ª metade − mediana da 1ª|."""
        half = len(win) // 2
        m1 = sorted(win[:half])[half // 2]
        tail = win[half:]
        m2 = sorted(tail)[len(tail) // 2]
        return abs(m2 - m1)

    def _publish_tare_result(self, *fields) -> None:
        m = String()
        m.data = ';'.join(str(f) for f in fields)
        self._tare_result_pub.publish(m)

    def _force_of(self, v: float) -> float:
        with self._lock:
            slope, intercept = self._slope, self._intercept
        return lc_force_n(v, slope, intercept)

    def _apply_tare(self, win: list[float]) -> tuple[bool, float]:
        """Zera pela média da janela. Devolve (ok, deriva_N).

        O tare é guardado em NEWTONS, não em volts: assim uma recalibração no
        meio da sessão não reinterpreta silenciosamente um zero que foi tirado
        com a reta antiga.
        """
        forces = [self._force_of(v) for v in win]
        drift = self._window_drift(forces)
        if drift > self._TARE_STABLE_N:
            return False, drift
        with self._lock:
            self._tare = sum(forces) / float(len(forces))
            self._tare_done = True
        return True, drift

    def _on_tare_request(self, _msg: Empty) -> None:
        """Tare PEDIDO (botão da GUI): exige calibração, dados e estabilidade."""
        with self._lock:
            slope = self._slope
            win = list(self._buf)[-self._capture_win_n:]
        if not slope:
            self._publish_tare_result('err', 'no_calib', 0)
            return
        if len(win) < 8:
            self._publish_tare_result('err', 'no_data', len(win))
            return
        ok, drift = self._apply_tare(win)
        if not ok:
            self._publish_tare_result('err', 'drifting', round(drift, 4))
            return
        with self._lock:
            ref = self._tare
        self.get_logger().info(
            f'Tare aplicado: referência {ref:+.3f} N '
            f'(deriva {drift:.3f} N na janela).')
        self._publish_tare_result('ok', repr(ref), round(drift, 4))

    def _on_rezero(self, _msg: Empty) -> None:
        """`/load_cell/rezero`: o zero do FIRMWARE, não o tare do host.

        O byte `'Z'` manda o MCU recoletar o offset de repouso da ponte, que
        é o único jeito de tirar deriva TÉRMICA da conta — o tare do host
        subtrai o sintoma, o `'Z'` refaz a referência. Requisito do firmware:
        a célula tem de estar descarregada; ele não transmite amostra nenhuma
        até travar o novo zero.
        """
        try:
            self._serial.send_command(b'Z')
        except Exception as exc:
            self.get_logger().warn(
                f'Re-zero do firmware não pôde ser enviado: {exc}')
            self._publish_tare_result('err', 'no_link', 0)
            return
        # Confirma no MESMO canal do tare. Sem esta linha a GUI só sabia da
        # FALHA: no caminho feliz ela não recebia nada e por isso anunciava
        # sucesso ao publicar o pedido — uma mensagem que era verdade sobre o
        # tópico e mentira sobre o fio.
        self._publish_tare_result('rezero', 'ok', 0)
        # O firmware para de transmitir enquanto coleta o novo zero, então o
        # tare do host que existia foi tirado contra outra referência.
        with self._lock:
            self._tare_done = False
            self._tare = 0.0
        self.get_logger().info(
            "Re-zero enviado ao firmware ('Z') — mantenha a célula "
            'descarregada. O tare do host será refeito quando as amostras '
            'voltarem.')

    def _auto_tare(self, win: list[float]) -> bool:
        """Tare AUTOMÁTICO de partida.

        Sem ele nada sai em `/load_cell/force_net` até alguém apertar o botão,
        e o explorer trata leitura ausente como falha. Duas guardas: janela
        estável E repouso perto do V₀ da calibração — a segunda é o que
        impede zerar uma carga real que já estava sobre a ponteira.
        """
        if len(win) < self._capture_win_n:
            return False
        bruto = sum(self._force_of(v) for v in win) / float(len(win))
        if abs(bruto) > self._AUTOTARE_MAX_N:
            return False
        ok, drift = self._apply_tare(win)
        if not ok:
            return False
        with self._lock:
            ref = self._tare
        self.get_logger().info(
            f'[LC] auto-tare na inicialização: referência {ref:+.3f} N '
            f'(deriva {drift:.3f} N na janela).')
        return True

    # ── Caminho da amostra ────────────────────────────────────────────
    def _on_sample(self, seq: int, t_us: int, v_raw: float) -> None:
        """Uma linha do firmware. Chamado NA THREAD 'lc-serial' — os
        publishers do rclpy são seguros aqui, e o filtro/_last_t_us só esta
        thread toca."""
        # Além do fundo de escala do ADC não há força possível: é entrada
        # saturada, ponte mal ligada ou HX711 sem célula. Fora antes do filtro.
        if abs(v_raw) > LC_FS_VOLTAGE_V:
            self._saturated += 1
            return

        # dt real pelo carimbo do FIRMWARE (wrap de uint32 tratado). Fora de
        # (0, 0.5 s] — 1ª linha ou religamento — cai na taxa nominal.
        dt = None
        if self._last_t_us is not None:
            d_us = (t_us - self._last_t_us) & 0xFFFFFFFF
            if 0 < d_us <= 500_000:
                dt = d_us / 1e6
        self._last_t_us = t_us

        v_filt = self._filter.update(float(v_raw), dt)

        m = Float32(); m.data = float(v_filt)
        self._voltage_pub.publish(m)

        with self._lock:
            self._rx_lines += 1
            self._buf.append(v_filt)
            calibrated = bool(self._slope)
            tare_done = self._tare_done
            win = (list(self._buf)[-self._capture_win_n:]
                   if (calibrated and not tare_done) else None)
        if win is not None and self._auto_tare(win):
            tare_done = True

        # Força CRUA (sem tare) — o que denuncia deriva sem descarregar.
        f_raw = self._force_of(v_filt)
        m = Float32(); m.data = float(f_raw)
        self._force_pub.publish(m)

        s = LoadCellSample()
        s.seq = int(seq) & 0xFFFFFFFF
        s.t_us = int(t_us) & 0xFFFFFFFF
        s.voltage_raw = float(v_raw)     # V de verdade, ao contrário da FA7155
        s.voltage = float(v_filt)
        s.force_net_n = 0.0
        s.calibrated = calibrated
        self._sample_pub.publish(s)

        if not (calibrated and tare_done):
            # Sem calibração ou sem tare NADA sai em force_net, e é o
            # comportamento certo: o explorer recusa o ensaio por leitura
            # ausente em vez de regular contra um zero que ninguém conferiu.
            return
        self._publish_net(seq, t_us, v_raw, v_filt, f_raw)

    def _publish_net(self, seq: int, t_us: int, v_raw: float,
                     v_filt: float, f_raw: float) -> None:
        """Força tare-compensada — a entrada da malha do explorer."""
        with self._lock:
            tare = self._tare
        f_net = f_raw - tare

        # Auto-zero lento. As duas guardas são necessárias — em repouso E
        # dentro da banda —, senão ele zeraria o próprio contato num HOLD.
        if (self._phase in self._AUTOZERO_PHASES
                and abs(f_net) < self._AUTOZERO_BAND_N):
            with self._lock:
                self._tare += self._autozero_rate * (f_raw - self._tare)
                tare = self._tare
            f_net = f_raw - tare

        m = Float32(); m.data = float(f_net)
        self._force_net_pub.publish(m)

        sn = LoadCellSample()
        sn.seq = int(seq) & 0xFFFFFFFF
        sn.t_us = int(t_us) & 0xFFFFFFFF
        sn.voltage_raw = float(v_raw)
        sn.voltage = float(v_filt)
        sn.force_net_n = float(f_net)
        sn.calibrated = True
        self._sample_net_pub.publish(sn)

    # ── Saúde do link ─────────────────────────────────────────────────
    def _publish_status(self) -> None:
        """`calibrated` aqui significa "há calibração válida carregada" — que
        é o que decide se a força publicada quer dizer alguma coisa. Ao
        contrário da FA7155, onde o mesmo tópico responde "chegam quadros"."""
        with self._lock:
            cal, tared = bool(self._slope), self._tare_done
        c = Bool(); c.data = cal
        self._calib_pub.publish(c)
        t = Bool(); t.data = bool(tared)
        self._tared_pub.publish(t)

    def _check_link(self) -> None:
        # Ao contrário do ft_receiver, a vivacidade do link NÃO vira tópico:
        # aqui `/load_cell/calibrated` responde "existe reta carregada", que é
        # o que decide se sai força. Quem quer saber se a placa está viva olha
        # a idade de `/load_cell/voltage` — é o que a aba Reading faz.
        alive = (self._serial.connected
                 and self._serial.last_rx > 0.0
                 and (time.monotonic() - self._serial.last_rx) < 3.0)
        if not alive and not self._last_rx_warn:
            self._last_rx_warn = True
            self.get_logger().warn(
                'Sem amostras do XIAO: '
                + (self._serial.error or 'porta aberta mas a linha está muda')
                + '. Confira: a placa NO CABO USB (não há queda para a rede), '
                  'DT/SCK do HX711 em D1/D3 e a ponte de 4 fios na entrada — '
                  'sem HX711 o firmware nem começa a transmitir, porque o '
                  'zero de boot nunca fecha.')
        elif alive and self._last_rx_warn:
            self._last_rx_warn = False
            self.get_logger().info(f'XIAO de volta em {self._serial.port}.')

    def _report_link_health(self) -> None:
        p = self._serial.parser
        counts = (p.bad_lines, p.bad_values, p.dropped_bytes)
        rotulos = ('linhas malformadas', 'valores inválidos',
                   'bytes descartados')
        deltas = tuple(counts[i] - self._last_counts[i] for i in range(3))
        self._last_counts = counts
        hb = p.heartbeats
        d_hb, self._last_heartbeats = hb - self._last_heartbeats, hb
        with self._lock:
            rx = self._rx_lines
            self._rx_lines = 0
        rate = rx / 10.0
        # HEARTBEAT SEM AMOSTRA: o firmware emite '#' a 0,5 Hz independente do
        # HX711. Chegar heartbeat e NÃO chegar amostra separa dois defeitos que
        # de fora parecem o mesmo silêncio — a placa está viva e falando, o que
        # não está respondendo é a ponte. Sem esta linha o contador existia e
        # não dizia nada a ninguém.
        if d_hb and not rx:
            self.get_logger().warn(
                f'{d_hb} heartbeats do XIAO nos últimos 10 s e NENHUMA '
                'amostra: o firmware está vivo na USB, quem não responde é o '
                'HX711. Confira DT/SCK em D1/D3 e a ponte de 4 fios — o zero '
                'de boot não fecha sem amostra, e sem ele nada é transmitido.')
        if rx and rate < LC_MIN_RATE_HZ and not self._rate_warned:
            self._rate_warned = True
            self.get_logger().warn(
                f'Taxa do HX711 ≈ {rate:.1f} Hz (nominal '
                f'{self._rate_nom:.0f} Hz). Abaixo disto não é escolha de '
                'pino RATE, é amostra perdida no caminho — cabo USB, hub ou '
                'DOUT flutuando.')
        elif rx and rate >= LC_MIN_RATE_HZ:
            self._rate_warned = False
        if any(deltas):
            detalhe = ', '.join(f'{n} {r}' for n, r in zip(deltas, rotulos))
            self.get_logger().warn(
                f'Linha USB suja nos últimos 10 s: {detalhe}. Costuma ser '
                'outro programa com a mesma tty aberta (monitor do '
                'PlatformIO) ou a placa gravada em modo SERIAL_TEST.')
        if self._saturated:
            self.get_logger().warn(
                f'{self._saturated} amostras descartadas por saturação do '
                f'ADC (|v| > {LC_FS_VOLTAGE_V * 1e3:.2f} mV) nos últimos '
                '10 s.')
            self._saturated = 0

    def destroy_node(self):
        try:
            self._serial.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ForceReceiverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
