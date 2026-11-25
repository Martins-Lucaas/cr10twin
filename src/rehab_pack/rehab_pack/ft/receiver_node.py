"""Driver da célula de 6 eixos FA7155 (RS485 → USB) do rehab_pack.

Cópia própria do ft_receiver do touch_pack, só com o que o rehab usa: serial,
modo stream/polled, filtro e tare. Dono EXCLUSIVO da porta do conversor. Publica
`/ft_sensor/wrench` (tarado), `/ft_sensor/wrench_raw` e o estado do tare em
`/load_cell/tared` e `/load_cell/tare_result`. O `rehab_force_source` consome.
"""
from __future__ import annotations

import collections
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32, Bool, Empty, String

from .constants import (
    FT_AXES,
    FT_MODBUS_SLAVE_ID,
    FT_MODBUS_TIMEOUT_S,
    FT_FORCE_AXIS_DEFAULT,
    FT_FORCE_SIGN_DEFAULT,
    FT_FRAME_LEN,
    FT_MIN_RATE_HZ,
    FT_NOMINAL_RATE_HZ,
    FT_RATED_FORCE_N,
    FT_RATED_TORQUE_NM,
    FT_SERIAL_BAUD,
    FT_MODE_CHOICES,
    FT_MODE_POLLED,
    FT_MODE_STREAM,
    ft_max_rate_hz,
    ft_polled_max_rate_hz,
)
from .lc_filter import _LoadCellFilter, QOS_SENSOR, window_drift
from .ft_polled import FtPolledDriver
from .ft_serial import FtSerialSource, detect_ft_serial_port

# Índice de cada canal dentro do quadro (fx, fy, fz, mx, my, mz).
_AXIS_INDEX = {'x': 0, 'y': 1, 'z': 2}

# Sanidade por canal: acima do fundo de escala com folga de overload (300 %FS,
# manual §3.1) o que chegou não é força, é quadro mal sincronizado que passou
# no CRC por azar. Descartado antes de entrar no filtro.
_FORCE_ABSURD_N   = 3.0 * FT_RATED_FORCE_N
_TORQUE_ABSURD_NM = 3.0 * FT_RATED_TORQUE_NM


class FtReceiverNode(Node):

    def __init__(self):
        super().__init__('ft_receiver')

        # ── Seis eixos ────────────────────────────────────────────────
        self._wrench_pub = self.create_publisher(
            WrenchStamped, '/ft_sensor/wrench', QOS_SENSOR)
        self._wrench_raw_pub = self.create_publisher(
            WrenchStamped, '/ft_sensor/wrench_raw', QOS_SENSOR)

        # ── Contrato /load_cell/* (o que a modulação consome) ─────────
        self._force_pub = self.create_publisher(
            Float32, '/load_cell/force', QOS_SENSOR)
        self._force_net_pub = self.create_publisher(
            Float32, '/load_cell/force_net', QOS_SENSOR)
        self._calib_pub = self.create_publisher(
            Bool, '/load_cell/calibrated', 10)
        self._tared_pub = self.create_publisher(Bool, '/load_cell/tared', 10)
        # Taxa MEDIDA na thread de aquisição. Existe porque quem estimava era
        # a GUI, pelos instantes em que a callback dela rodava — e isso mede o
        # agendamento do executor, não a aquisição: com o laço a 400,00 Hz
        # (desvio 0,019 Hz no driver) a tela balançava 11 Hz. Aqui o número
        # sai dos mesmos carimbos que o laço usa, antes de filtro, tara e das
        # seis publicações, que é o trabalho de duração variável no caminho.
        self._rate_pub = self.create_publisher(
            Float32, '/load_cell/rate_hz', 10)
        self._tare_result_pub = self.create_publisher(
            String, '/load_cell/tare_result', 10)

        # ── Parâmetros ────────────────────────────────────────────────
        port = str(self.declare_parameter('ft_serial_port', '').value).strip()
        baud = int(self.declare_parameter('ft_baud', FT_SERIAL_BAUD).value)
        rate = float(self.declare_parameter(
            'ft_rate_hz', FT_NOMINAL_RATE_HZ).value)
        axis = str(self.declare_parameter(
            'ft_force_axis', FT_FORCE_AXIS_DEFAULT).value).strip().lower()
        sign = float(self.declare_parameter(
            'ft_force_sign', FT_FORCE_SIGN_DEFAULT).value)
        self._frame_id = str(self.declare_parameter(
            'ft_frame_id', 'ft_sensor_link').value)
        # MODO de aquisição. 'stream' = escutar os
        # quadros "ST" que o sensor emite sozinho (default histórico, 1 kHz a
        # 1 Mbps); 'polled' = perguntar em laço com 0x03, que é o que o
        # cliente/HMI de fábrica faz. O polled cabe em 115200 e por isso é o
        # que torna a rota do flange viável — ao custo de ~262 Hz de teto.
        mode = str(self.declare_parameter(
            'ft_mode', FT_MODE_STREAM).value).strip().lower()

        if axis not in _AXIS_INDEX:
            self.get_logger().warn(
                f"ft_force_axis='{axis}' não é x, y ou z — usando "
                f"'{FT_FORCE_AXIS_DEFAULT}'.")
            axis = FT_FORCE_AXIS_DEFAULT
        self._axis_name = axis
        self._axis_i = _AXIS_INDEX[axis]
        # Só o SINAL importa; qualquer ganho aqui reescalaria a força e o
        # setpoint da GUI passaria a significar outra coisa.
        self._sign = -1.0 if sign < 0 else 1.0
        # Teto calculado sobre o baud EM USO, não sobre o FT_SERIAL_BAUD:
        # com `ft_baud` alterado, o aviso citava um teto que não era o do link
        # que ele mandava conferir.
        link_max_hz = ft_max_rate_hz(baud)
        if rate > link_max_hz:
            self.get_logger().warn(
                f'ft_rate_hz={rate:.0f} Hz não cabe em {baud} baud (teto '
                f'{link_max_hz:.0f} Hz, {FT_FRAME_LEN} B por quadro). Os '
                'quadros vão chegar picotados; baixe a taxa de saída do '
                'sensor ou suba o baud.')

        # ── Estado ────────────────────────────────────────────────────
        # Um filtro por canal — o wrench inteiro precisa ser utilizável, não
        # só o eixo de controle. sensitivity=1 porque o sinal já vem em N
        # (o beta do One-Euro é expresso em Hz por (N/s)).
        self._filters = []
        for _ in FT_AXES:
            f = _LoadCellFilter()
            f.set_sensitivity(1.0)
            self._filters.append(f)
        self._last_t_us: int | None = None
        # ~1 s de carimbos de chegada, para a taxa publicada.
        self._rate_win: collections.deque = collections.deque(
            maxlen=max(int(FT_NOMINAL_RATE_HZ), 8))

        self._lock = threading.Lock()
        self._tare = [0.0] * len(FT_AXES)
        self._tare_done = False
        self._link_ok = False
        # Janela dos SEIS eixos filtrados, para o tare e o auto-tare.
        # 1600 = 1,6 s a 1 kHz, o dobro de _CAPTURE_WIN_N (era 400/200 quando
        # o exemplar era lido a 250 Hz — a proporção é que importa).
        self._buf: collections.deque = collections.deque(maxlen=1600)

        self._rx_frames = 0
        self._rate_hz = 0.0
        self._rate_warned = False
        self._last_counts = (0, 0, 0)   # crc_errors, resyncs, bad_values
        self._absurd = 0

        self.create_subscription(Empty, '/load_cell/tare',
                                 self._on_tare_request, 10)
        self.create_subscription(Empty, '/load_cell/rezero',
                                 self._on_rezero, 10)
        # Auto-zero lento ligado/desligado. Ele cancela deriva térmica, mas
        # em repouso prende a leitura em zero exato — e numa bancada em que se
        # OLHA o sinal parado isso é indistinguível de célula morta. Quem quer
        # ver a força contínua desliga; quem fecha malha longa deixa ligado.
        self._autozero = bool(self.declare_parameter('ft_autozero', True).value)
        # One-Euro por canal ligado/desligado. Ele custa ATRASO — é um passa-
        # baixa adaptativo, e o que ele tira de ruído ele tira também de
        # degrau. Quem quer o sinal do sensor como ele sai do fio desliga; o
        # /ft_sensor/wrench_raw continua cru dos dois jeitos, mas ele também é
        # pré-tare, então não serve a quem precisa do zero.
        self._filter_on = bool(
            self.declare_parameter('ft_filter', True).value)
        self._modbus_slave = int(self.declare_parameter(
            'ft_modbus_slave_id', FT_MODBUS_SLAVE_ID).value)
        self._modbus_timeout = float(self.declare_parameter(
            'ft_modbus_timeout_s', FT_MODBUS_TIMEOUT_S).value)
        # Passo do laço polled. 0 = livre, o mais rápido que a linha aceitar —
        # e era o que estava aqui. Livre a taxa é o que SOBRA depois da
        # latência do USB, então ela balança com a carga da máquina: cada
        # transação leva o tempo que levar, e um pico de CPU vira um dt maior.
        # Com passo fixo o laço espera a diferença e o dt fica igual, ao custo
        # de ficar abaixo do teto. Como o dt é o que o filtro e o explorer
        # consomem, taxa ESTÁVEL vale mais que taxa máxima.
        rate_alvo = float(self.declare_parameter(
            'ft_poll_rate_hz', FT_NOMINAL_RATE_HZ).value)
        self._poll_interval = (1.0 / rate_alvo) if rate_alvo > 0.0 else 0.0

        if mode not in FT_MODE_CHOICES:
            self.get_logger().warn(
                f"ft_mode='{mode}' não é "
                f"{' nem '.join(repr(m) for m in FT_MODE_CHOICES)} — "
                f"usando '{FT_MODE_STREAM}'.")
            mode = FT_MODE_STREAM
        self._mode = mode

        self._serial = FtSerialSource(port=(port or None), baud=baud,
                                      rate_hz=rate,
                                      on_sample=self._on_sample)
        detected = port or detect_ft_serial_port() or \
            '(procurando o conversor RS485)'
        if not self._serial.start():
            self.get_logger().error(
                f'Porta serial indisponível ({self._serial.error}). Instale '
                'pyserial (pip install pyserial) e reinicie o nó.')
        # O driver polled NÃO abre porta: ele dirige o command_session() do
        # transporte acima, que continua dono da linha e da thread de leitura.
        # É por isso que polled funciona igual pela serial e pela TCP do
        # flange, sem uma linha a mais no ft_tcp.py.
        self._polled: FtPolledDriver | None = None
        if mode == FT_MODE_POLLED:
            self._polled = FtPolledDriver(
                self._serial,
                slave_id=self._modbus_slave,
                timeout_s=self._modbus_timeout,
                interval_s=self._poll_interval,
                on_sample=self._on_sample)
            self._polled.start()

        if mode == FT_MODE_POLLED:
            teto = ft_polled_max_rate_hz(baud)
            extra = (f' | polled, teto do fio {teto:.0f} Hz '
                     f'(a taxa MEDIDA fica abaixo: a latência de USB não '
                     f'entra nessa conta)')
        else:
            extra = f' | stream, teto do fio {ft_max_rate_hz(baud):.0f} Hz'
        self.get_logger().info(
            f'FtReceiver: {detected} @ '
            f'{baud} | {rate:.0f} Hz | força de controle = '
            f'{"+" if self._sign > 0 else "-"}F{axis}{extra}')

        self._last_rx_warn = False
        self.create_timer(1.0, self._publish_status)
        self.create_timer(5.0, self._check_link)
        self.create_timer(10.0, self._report_link_health)

    # ── TARE ──────────────────────────────────────────────────────────
    # Janela do tare: ~0,8 s a 1 kHz (eram 200 amostras quando o exemplar
    # era lido a 250 Hz — a janela é definida em TEMPO, não em amostras).
    _CAPTURE_WIN_N = 800
    # Estabilidade por DERIVA (mediana da 2ª metade vs a 1ª), não por
    # pico-a-pico: o ptp cresce com a janela mesmo em sinal estacionário.
    _TARE_STABLE_N = 0.50
    # Auto-zero lento: cancela deriva térmica sem comer força real. Só atua em
    # repouso E dentro da banda morta — as duas condições, senão ele zeraria o
    # próprio contato durante um HOLD.
    _AUTOZERO_BAND_N = 0.30
    _AUTOZERO_RATE = 0.00025    # passo/amostra (tau ~ 4 s a 1 kHz; era
                                # 0,001 quando a taxa era 250 Hz)

    # Critério de estabilidade do tare — o mesmo nos dois receivers, por isso
    # vive em lc_filter. Continua exposto aqui como `self._window_drift`.
    _window_drift = staticmethod(window_drift)

    def _publish_tare_result(self, *fields) -> None:
        m = String()
        m.data = ';'.join(str(f) for f in fields)
        self._tare_result_pub.publish(m)

    def _apply_tare(self, win: list[list[float]]) -> tuple[bool, float]:
        """Zera os SEIS eixos pela média da janela. Devolve (ok, deriva_N).

        A estabilidade é julgada no eixo de CONTROLE: é ele que a malha usa, e
        exigir repouso simultâneo nos seis recusaria tares perfeitamente bons
        por causa de um momento residual do peso da ferramenta.
        """
        drift = self._window_drift([row[self._axis_i] for row in win])
        if drift > self._TARE_STABLE_N:
            return False, drift
        n = float(len(win))
        with self._lock:
            self._tare = [sum(row[k] for row in win) / n
                          for k in range(len(FT_AXES))]
            self._tare_done = True
        return True, drift

    def _on_tare_request(self, _msg: Empty) -> None:
        """Tare PEDIDO (botão da GUI): exige dados e estabilidade."""
        with self._lock:
            win = list(self._buf)[-self._CAPTURE_WIN_N:]
        if len(win) < 30:
            self._publish_tare_result('err', 'no_data', len(win))
            return
        ok, drift = self._apply_tare(win)
        if not ok:
            self._publish_tare_result('err', 'drifting', round(drift, 4))
            return
        with self._lock:
            ref = self._tare[self._axis_i]
        self.get_logger().info(
            f'Tare aplicado nos 6 eixos: F{self._axis_name} de referência '
            f'{ref:+.3f} N (deriva {drift:.3f} N na janela).')
        self._publish_tare_result('ok', repr(ref), round(drift, 4))

    def _on_rezero(self, _msg: Empty) -> None:
        """`/load_cell/rezero`. O FA7155 em modo ativo não aceita comando,
        então o zero possível é o tare do host."""
        self.get_logger().info(
            'Re-zero pedido: o FA7155 não tem comando de zero (modo ativo, '
            'só transmite) — refazendo o TARE. Mantenha a célula descarregada.')
        self._on_tare_request(Empty())

    def _auto_tare(self, win: list[list[float]]) -> bool:
        """Tare AUTOMÁTICO de partida.

        Sem ele nada sai em `/load_cell/force_net` até alguém apertar o botão,
        e o explorer trata leitura ausente como falha. Não há V₀ de
        calibração com que comparar a deriva: o critério é só a
        ESTABILIDADE da janela — que é o que o zero de um sensor já calibrado
        de fábrica precisa.
        """
        if len(win) < self._CAPTURE_WIN_N:
            return False
        ok, drift = self._apply_tare(win)
        if not ok:
            return False
        with self._lock:
            ref = self._tare[self._axis_i]
        self.get_logger().info(
            f'[FT] auto-tare na inicialização: F{self._axis_name} de '
            f'referência {ref:+.3f} N (deriva {drift:.3f} N na janela).')
        return True

    # ── Caminho da amostra ────────────────────────────────────────────
    def _on_sample(self, seq: int, t_us: int, vals: tuple) -> None:
        """Um quadro do FA7155. Chamado NA THREAD 'ft-serial' — os publishers
        do rclpy são seguros aqui, e os filtros/_last_t_us só esta thread
        toca."""
        # Quadro que passou no CRC mas traz valor fora de qualquer física
        # possível: sincronismo por azar. Fora antes de entrar no filtro.
        if (any(abs(v) > _FORCE_ABSURD_N for v in vals[:3])
                or any(abs(v) > _TORQUE_ABSURD_NM for v in vals[3:])):
            self._absurd += 1
            return

        # dt real pelo carimbo do host (wrap de uint32 tratado). Fora de
        # (0, 0.5 s] — 1º quadro ou religamento — cai na taxa nominal.
        dt = None
        if self._last_t_us is not None:
            d_us = (t_us - self._last_t_us) & 0xFFFFFFFF
            if 0 < d_us <= 500_000:
                dt = d_us / 1e6
        self._last_t_us = t_us

        if self._filter_on:
            filt = [self._filters[k].update(float(vals[k]), dt)
                    for k in range(len(FT_AXES))]
        else:
            filt = [float(v) for v in vals]

        self._publish_wrench(self._wrench_raw_pub, vals)

        with self._lock:
            self._rx_frames += 1
            self._rate_win.append(time.perf_counter())
            self._link_ok = True
            self._buf.append(filt)
            tare_done = self._tare_done
            win = (list(self._buf)[-self._CAPTURE_WIN_N:]
                   if not tare_done else None)
        if win is not None and self._auto_tare(win):
            tare_done = True

        # Força CRUA do eixo de controle (sem tare) em `/load_cell/force`.
        f_raw = self._sign * filt[self._axis_i]
        m = Float32(); m.data = float(f_raw)
        self._force_pub.publish(m)

        if not tare_done:
            # Sem tare NADA sai em force_net, e é o comportamento certo: o
            # explorer recusa o ensaio por leitura ausente em vez de regular
            # contra um zero que ninguém conferiu.
            return
        self._publish_net(filt)

    def _publish_net(self, filt: list[float]) -> None:
        """Wrench tarado + a força do eixo de controle (entrada da malha)."""
        with self._lock:
            tare = list(self._tare)
        net = [filt[k] - tare[k] for k in range(len(FT_AXES))]
        f_net = self._sign * net[self._axis_i]

        # Auto-zero lento. As duas guardas são necessárias — em repouso E
        # dentro da banda —, senão ele zeraria o próprio contato num HOLD.
        if (self._autozero
                and abs(f_net) < self._AUTOZERO_BAND_N):
            with self._lock:
                for k in range(len(FT_AXES)):
                    self._tare[k] += self._AUTOZERO_RATE * (filt[k]
                                                            - self._tare[k])
                tare = list(self._tare)
            net = [filt[k] - tare[k] for k in range(len(FT_AXES))]
            f_net = self._sign * net[self._axis_i]

        self._publish_wrench(self._wrench_pub, net)

        m = Float32(); m.data = float(f_net)
        self._force_net_pub.publish(m)

    def _publish_wrench(self, pub, v) -> None:
        w = WrenchStamped()
        w.header.stamp = self.get_clock().now().to_msg()
        w.header.frame_id = self._frame_id
        w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = (
            float(v[0]), float(v[1]), float(v[2]))
        w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z = (
            float(v[3]), float(v[4]), float(v[5]))
        pub.publish(w)

    # ── Saúde do link ─────────────────────────────────────────────────
    def _publish_status(self) -> None:
        """`calibrated` aqui significa "há quadros válidos chegando": o
        FA7155 já vem calibrado de fábrica, então o que a GUI precisa saber é
        se a força publicada tem origem, não se um arquivo foi lido."""
        with self._lock:
            ok, tared = self._link_ok, self._tare_done
            win = list(self._rate_win)
        if len(win) >= 8:
            span = win[-1] - win[0]
            if span > 0:
                r = Float32(); r.data = float((len(win) - 1) / span)
                self._rate_pub.publish(r)
        c = Bool(); c.data = bool(ok)
        self._calib_pub.publish(c)
        t = Bool(); t.data = bool(tared)
        self._tared_pub.publish(t)

    def _check_link(self) -> None:
        # De QUEM é o last_rx: em polled o transporte só entrega bytes pela
        # derivação (ft_cmd_channel), e o parser de stream nunca casa um
        # quadro 'ST' — o `last_rx` do FtSerialSource fica em 0.0 para sempre.
        # Olhá-lo nesse modo fazia o nó gritar "a linha está muda, confira 24 V
        # e A/B" a cada partida com o sensor respondendo 458 amostras/s.
        src = self._polled if self._polled is not None else self._serial
        alive = (src.connected
                 and src.last_rx > 0.0
                 and (time.monotonic() - src.last_rx) < 3.0)
        with self._lock:
            self._link_ok = alive
        if not alive and not self._last_rx_warn:
            self._last_rx_warn = True
            self.get_logger().warn(
                'Sem quadros do FA7155: '
                + (self._serial.error or 'porta aberta mas a linha está muda')
                + '. Confira: 24 V na célula (o +5 V do conversor NÃO a '
                  'alimenta), A/B do RS485 não trocados e GND comum entre a '
                  'fonte e o conversor.')
        elif alive and self._last_rx_warn:
            self._last_rx_warn = False
            self.get_logger().info(
                f'FA7155 de volta em {self._serial.port}.')

    def _report_link_health(self) -> None:
        # Cada modo tem a SUA patologia. No stream o que dói é CRC e
        # ressincronismo (reflexão, aterramento, par trocado); no polled não
        # existe ressincronismo — o que dói é timeout, porque cada amostra é
        # uma transação que ou volta ou não. Relatar os contadores do parser
        # em modo polled mostraria três zeros eternos e esconderia o problema.
        if self._polled is not None:
            d = self._polled
            counts = (d.timeouts, d.errors, d.bad_values)
            rotulos = ('timeouts', 'erros', 'valores inválidos')
        else:
            p = self._serial.parser
            counts = (p.crc_errors, p.resyncs, p.bad_values)
            rotulos = ('CRC', 'ressincs', 'valores inválidos')
        d_crc, d_res, d_bad = (counts[i] - self._last_counts[i]
                               for i in range(3))
        self._last_counts = counts
        with self._lock:
            rx = self._rx_frames
            self._rx_frames = 0
        rate = rx / 10.0
        self._rate_hz = rate
        if rx and rate < FT_MIN_RATE_HZ and not self._rate_warned:
            self._rate_warned = True
            if self._polled is not None:
                # Em polled a referência não é a taxa de fábrica: é o teto do
                # fio. Citar os 1000 Hz nominais aqui mandaria procurar um
                # problema que não existe.
                self.get_logger().warn(
                    f'Taxa polled do FA7155 ≈ {rate:.0f} Hz. Cada amostra é '
                    'um round-trip, então isto é latência da linha (USB, '
                    'timeout do escravo) e não perda de quadro. Suba o baud '
                    'ou passe ft_mode:=stream se precisar de mais.')
            else:
                self.get_logger().warn(
                    f'Taxa do FA7155 ≈ {rate:.0f} Hz (nominal '
                    f'{FT_NOMINAL_RATE_HZ:.0f} Hz). Baud errado, cabo longo '
                    'sem terminação ou taxa de fábrica diferente — a banda '
                    f'útil do controle cai para ~{rate / 4.0:.0f} Hz.')
        elif rx and rate >= FT_MIN_RATE_HZ:
            self._rate_warned = False
        if d_crc or d_res or d_bad:
            # Nem CRC errado nem timeout são ruído de software: numa RS485 é
            # reflexão (falta terminação de 120 Ω), aterramento ou par A/B
            # trocado. O remédio é o mesmo nos dois modos; só o sintoma muda.
            detalhe = ', '.join(
                f'{n} {r}' for n, r in zip((d_crc, d_res, d_bad), rotulos))
            self.get_logger().warn(
                f'Link RS485 sujo nos últimos 10 s: {detalhe}. Cheque a '
                'terminação de 120 Ω nas duas pontas, a blindagem e o par '
                'A/B.')
        if self._absurd:
            self.get_logger().warn(
                f'{self._absurd} quadros descartados por valor fora do fundo '
                'de escala nos últimos 10 s.')
            self._absurd = 0

    def destroy_node(self):
        # Antes do transporte: o driver dirige o command_session() dele, e
        # parar a porta primeiro deixaria a transação em voo estourando.
        if self._polled is not None:
            self._polled.stop()
        try:
            self._serial.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FtReceiverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
