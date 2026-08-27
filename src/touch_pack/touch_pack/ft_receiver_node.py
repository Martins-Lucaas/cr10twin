"""ft_receiver_node.py — Driver da célula de 6 eixos FA7155 (RS485 → USB).

Papel: é o `force_receiver` da célula nova. Dono EXCLUSIVO da porta do
conversor USB-RS485, e a única fonte de `/load_cell/force_net` quando roda.
NUNCA suba os dois receivers ao mesmo tempo — dois publicadores no tópico da
malha de segurança fazem o explorer regular contra a média de duas células.

O que muda em relação ao force_receiver (XIAO + HX711):

* Seis canais em vez de um. Todos são publicados em `/ft_sensor/wrench`
  (geometry_msgs/WrenchStamped), filtrados e tarados; o cru sai em
  `/ft_sensor/wrench_raw`.
* Não há arquivo de calibração. O FA7155 entrega N e N·m calibrados de
  fábrica, então `slope`/`intercept` deixam de existir e `/load_cell/calibrated`
  passa a significar "há quadros válidos chegando". O que sobra do lado do
  host é o TARE — que aqui zera os SEIS eixos, não só um.
* Há DOIS zeros, e eles são coisas diferentes. O `/load_cell/rezero` refaz o
  TARE do host (subtração no software, some ao reiniciar o nó). O zero do
  SENSOR é o `Set_Zero` do canal Modbus — ver `ft_modbus.py` e o tópico
  `/ft_sensor/command`. Até 26/08/2026 este arquivo afirmava que o sensor "só
  fala"; a análise do cliente de fábrica da FIBOS mostrou que ele também é um
  escravo Modbus RTU na mesma linha 485.

Adaptação para a modulação de força EXISTENTE (tactile_explorer): um dos eixos
faz o papel da antiga célula axial e é publicado em `/load_cell/force_net` com
a MESMA convenção (COMPRESSÃO POSITIVA, tare aplicado, filtro mediana+One-Euro).
Qual eixo e com que sinal é decidido pelos parâmetros `ft_force_axis` /
`ft_force_sign` — ver FT_FORCE_AXIS_DEFAULT em constants.py e conferir na
bancada com `scripts/ft_probe.py`. Do explorer para baixo, nada precisa saber
que a célula mudou.

ATENÇÃO ao `/load_cell/sample_net` (o que alimenta o samples.csv): os campos
`voltage_raw` e `voltage` da LoadCellSample carregam aqui a força do eixo de
controle em NEWTONS (crua e filtrada), não volts. Zerá-los seria mais "honesto"
quanto ao nome, mas jogaria fora exatamente o que esses campos existem para
guardar — o valor SEM o atraso do filtro, que é o que permite refazer a força
offline. As colunas `lc_voltage_raw_v`/`lc_voltage_v` do CSV devem ser lidas
como newtons nos runs feitos com esta célula.
"""
from __future__ import annotations

import collections
import json
import threading
import time
from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32, Bool, Empty, String
from touch_pack_msgs.msg import LoadCellSample, PalpationStatus

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
    FT_TCP_HOST,
    FT_TCP_PORT,
    FT_MODE_CHOICES,
    FT_MODE_POLLED,
    FT_MODE_STREAM,
    ft_max_rate_hz,
    ft_polled_max_rate_hz,
)
from .lc_filter import _LoadCellFilter, QOS_SENSOR
from .ft_modbus import (
    FtDevice, FtModbusClient, ModbusError, FtModbusMapUnconfirmed,
    format_scan, scan_registers, suggest_map,
)
from .ft_polled import FtPolledDriver
from .ft_serial import FtSerialSource, detect_ft_serial_port
from .ft_tcp import FtTcpSource, configure_tool_485

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

        # ── Contrato da célula axial (o que a modulação consome) ──────
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
        # Canal de COMANDO do sensor (Modbus RTU pela mesma 485) — JSON nos
        # dois sentidos porque os comandos têm argumentos de tipos diferentes,
        # e um 'ok;a;b' posicional já não dá conta.
        self._cmd_result_pub = self.create_publisher(
            String, '/ft_sensor/command_result', 10)

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
        # Transporte: 'serial' = conversor USB-RS485 na mesa (default
        # histórico); 'tcp' = RS485 do flange, exposta pelo controlador na
        # porta 60000. Ver ft_tcp.py.
        transport = str(self.declare_parameter(
            'ft_transport', 'serial').value).strip().lower()
        # MODO de aquisição, ortogonal ao MEIO acima. 'stream' = escutar os
        # quadros "ST" que o sensor emite sozinho (default histórico, 1 kHz a
        # 1 Mbps); 'polled' = perguntar em laço com 0x03, que é o que o
        # cliente/HMI de fábrica faz. O polled cabe em 115200 e por isso é o
        # que torna a rota do flange viável — ao custo de ~262 Hz de teto.
        mode = str(self.declare_parameter(
            'ft_mode', FT_MODE_STREAM).value).strip().lower()
        tcp_host = str(self.declare_parameter(
            'ft_tcp_host', FT_TCP_HOST).value).strip()
        tcp_port = int(self.declare_parameter('ft_tcp_port', FT_TCP_PORT).value)
        # Opt-in: manda SetToolMode/SetTool485/SetToolPower na 29999 antes de
        # abrir a 60000. Deixe FALSO com o real_driver no ar — a 29999 é dele,
        # e dois donos embaralham comando e resposta no dashboard.
        tcp_configure = bool(self.declare_parameter(
            'ft_tcp_configure', False).value)

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

        self._phase: str = ''
        self.create_subscription(PalpationStatus, '/palpation/status',
                                 self._on_palpation_status, 10)
        self.create_subscription(Empty, '/load_cell/tare',
                                 self._on_tare_request, 10)
        # Tare do HOST (software). O zero do SENSOR é o cmd 'zero' abaixo.
        self.create_subscription(Empty, '/load_cell/rezero',
                                 self._on_rezero, 10)
        self.create_subscription(String, '/ft_sensor/command',
                                 self._on_command, 10)
        self._modbus_slave = int(self.declare_parameter(
            'ft_modbus_slave_id', FT_MODBUS_SLAVE_ID).value)
        self._modbus_timeout = float(self.declare_parameter(
            'ft_modbus_timeout_s', FT_MODBUS_TIMEOUT_S).value)
        # Uma transação por vez: a derivação da linha (ft_cmd_channel) não
        # aninha, e serializar aqui dá erro claro em vez de RuntimeError.
        self._cmd_busy = threading.Lock()

        if mode not in FT_MODE_CHOICES:
            self.get_logger().warn(
                f"ft_mode='{mode}' não é "
                f"{' nem '.join(repr(m) for m in FT_MODE_CHOICES)} — "
                f"usando '{FT_MODE_STREAM}'.")
            mode = FT_MODE_STREAM
        self._mode = mode

        if transport not in ('serial', 'tcp'):
            self.get_logger().warn(
                f"ft_transport='{transport}' não é 'serial' nem 'tcp' — "
                "usando 'serial'.")
            transport = 'serial'

        if transport == 'tcp':
            if tcp_configure:
                # A 485 do flange fica MUDA até estes três comandos. Falhar
                # aqui não é fatal: pode ser que alguém já a tenha configurado.
                try:
                    for cmd, resp in configure_tool_485(host=tcp_host,
                                                        baud=baud):
                        self.get_logger().info(f'[485] {cmd} -> {resp}')
                except Exception as exc:
                    self.get_logger().error(
                        f'Falha ao configurar a 485 do flange em {tcp_host}: '
                        f'{exc}. Se o real_driver estiver no ar, a 29999 é '
                        'dele — configure pelo ft_probe.py --tcp --configure '
                        'com o robô parado.')
            self._serial = FtTcpSource(host=tcp_host, tcp_port=tcp_port,
                                       rate_hz=rate,
                                       on_sample=self._on_sample)
            detected = f'{tcp_host}:{tcp_port} (485 do flange)'
        else:
            self._serial = FtSerialSource(port=(port or None), baud=baud,
                                          rate_hz=rate,
                                          on_sample=self._on_sample)
            detected = port or detect_ft_serial_port() or \
                '(procurando o conversor RS485)'
        if not self._serial.start():
            # Só o transporte SERIAL tem dependência opcional; o TCP é
            # stdlib e o seu start() nunca falha (robô inalcançável vira
            # retentativa, não falha de partida). Mandar "instale pyserial"
            # num nó configurado para a 60000 era conselho para o problema
            # errado.
            remedio = ('Instale pyserial (pip install pyserial) e reinicie '
                       'o nó.' if transport == 'serial' else
                       'Confira ft_tcp_host/ft_tcp_port e a rede até o '
                       'controlador.')
            self.get_logger().error(
                f'Transporte {transport} indisponível '
                f'({self._serial.error}). {remedio}')
        # O driver polled NÃO abre porta: ele dirige o command_session() do
        # transporte acima, que continua dono da linha e da thread de leitura.
        # É por isso que polled funciona igual pela serial e pela TCP do
        # flange, sem uma linha a mais no ft_tcp.py.
        # Guardados para o 'scan' (que casa valores lidos com o estado
        # conhecido) e para o relatório de modo.
        self._baud, self._rate = baud, rate
        self._polled: FtPolledDriver | None = None
        if mode == FT_MODE_POLLED:
            self._polled = FtPolledDriver(
                self._serial,
                slave_id=self._modbus_slave,
                timeout_s=self._modbus_timeout,
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

    # ── Fase da palpação (só para gatear o auto-zero) ─────────────────
    _AUTOZERO_PHASES = ('', 'IDLE', 'DONE', 'ABORTED')

    def _on_palpation_status(self, msg) -> None:
        self._phase = str(msg.phase or '')

    # ── TARE ──────────────────────────────────────────────────────────
    # Janela do tare: ~0,8 s a 1 kHz (eram 200 amostras quando o exemplar
    # era lido a 250 Hz — a janela é definida em TEMPO, não em amostras).
    _CAPTURE_WIN_N = 800
    # Estabilidade por DERIVA (mediana da 2ª metade vs a 1ª), não por
    # pico-a-pico — mesma escolha do force_receiver, pelo mesmo motivo: o ptp
    # cresce com a janela mesmo em sinal estacionário.
    _TARE_STABLE_N = 0.50
    # Auto-zero lento: cancela deriva térmica sem comer força real. Só atua em
    # repouso E dentro da banda morta — as duas condições, senão ele zeraria o
    # próprio contato durante um HOLD.
    _AUTOZERO_BAND_N = 0.30
    _AUTOZERO_RATE = 0.00025    # passo/amostra (tau ~ 4 s a 1 kHz; era
                                # 0,001 quando a taxa era 250 Hz)

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
        """`/load_cell/rezero` da GUI. No HX711 isto ia para o firmware; o
        FA7155 em modo ativo não aceita comando, então o zero possível é o
        tare do host."""
        self.get_logger().info(
            'Re-zero pedido: o FA7155 não tem comando de zero (modo ativo, '
            'só transmite) — refazendo o TARE. Mantenha a célula descarregada.')
        self._on_tare_request(Empty())

    def _auto_tare(self, win: list[list[float]]) -> bool:
        """Tare AUTOMÁTICO de partida.

        Sem ele nada sai em `/load_cell/force_net` até alguém apertar o botão,
        e o explorer trata leitura ausente como falha. Diferente do HX711, não
        há V₀ de calibração com que comparar a deriva: o critério é só a
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

        filt = [self._filters[k].update(float(vals[k]), dt)
                for k in range(len(FT_AXES))]

        self._publish_wrench(self._wrench_raw_pub, vals)

        with self._lock:
            self._rx_frames += 1
            self._link_ok = True
            self._buf.append(filt)
            tare_done = self._tare_done
            win = (list(self._buf)[-self._CAPTURE_WIN_N:]
                   if not tare_done else None)
        if win is not None and self._auto_tare(win):
            tare_done = True

        # Força CRUA do eixo de controle (sem tare) — espelha o
        # `/load_cell/force` do HX711, que também é pré-tare.
        f_raw = self._sign * filt[self._axis_i]
        m = Float32(); m.data = float(f_raw)
        self._force_pub.publish(m)

        s = LoadCellSample()
        s.seq = int(seq) & 0xFFFFFFFF
        s.t_us = int(t_us) & 0xFFFFFFFF
        s.voltage_raw = float(self._sign * vals[self._axis_i])   # N, não V
        s.voltage = float(f_raw)                                  # N, não V
        s.force_net_n = 0.0
        s.calibrated = True
        self._sample_pub.publish(s)

        if not tare_done:
            # Sem tare NADA sai em force_net, e é o comportamento certo: o
            # explorer recusa o ensaio por leitura ausente em vez de regular
            # contra um zero que ninguém conferiu.
            return
        self._publish_net(seq, t_us, vals, filt)

    def _publish_net(self, seq: int, t_us: int, vals: tuple,
                     filt: list[float]) -> None:
        """Wrench tarado + a força do eixo de controle (entrada da malha)."""
        with self._lock:
            tare = list(self._tare)
        net = [filt[k] - tare[k] for k in range(len(FT_AXES))]
        f_net = self._sign * net[self._axis_i]

        # Auto-zero lento. As duas guardas são necessárias — em repouso E
        # dentro da banda —, senão ele zeraria o próprio contato num HOLD.
        if (self._phase in self._AUTOZERO_PHASES
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

        sn = LoadCellSample()
        sn.seq = int(seq) & 0xFFFFFFFF
        sn.t_us = int(t_us) & 0xFFFFFFFF
        sn.voltage_raw = float(self._sign * vals[self._axis_i])   # N, não V
        sn.voltage = float(self._sign * filt[self._axis_i])       # N, não V
        sn.force_net_n = float(f_net)
        sn.calibrated = True
        self._sample_net_pub.publish(sn)

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
        c = Bool(); c.data = bool(ok)
        self._calib_pub.publish(c)
        t = Bool(); t.data = bool(tared)
        self._tared_pub.publish(t)

    def _check_link(self) -> None:
        alive = (self._serial.connected
                 and self._serial.last_rx > 0.0
                 and (time.monotonic() - self._serial.last_rx) < 3.0)
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

    # ── Canal de comando Modbus (Set_Zero, taxa, node ID, baud) ───────
    # Contrato de /ft_sensor/command: {"cmd": <nome>, "args": {...}}.
    # A resposta sai em /ft_sensor/command_result como
    # {"cmd":…, "ok":bool, "detail":str, "data":{...}} — sempre, inclusive na
    # recusa, para a GUI nunca ficar esperando.
    _COMMANDS = ('zero', 'rate', 'node_id', 'baud',
                 'stream_start', 'stream_stop', 'probe', 'scan', 'mode')

    def _reply(self, cmd: str, ok: bool, detail: str, data: dict | None = None):
        self._cmd_result_pub.publish(String(data=json.dumps(
            {'cmd': cmd, 'ok': bool(ok), 'detail': detail,
             'data': data or {}}, ensure_ascii=False)))

    def _on_command(self, msg: String) -> None:
        try:
            req = json.loads(str(msg.data))
            cmd = str(req.get('cmd', '')).strip()
            args = dict(req.get('args', {}) or {})
        except Exception as exc:
            self._reply('?', False, f'pedido malformado: {exc}')
            return
        if cmd not in self._COMMANDS:
            self._reply(cmd, False,
                        f'comando desconhecido (aceitos: '
                        f'{", ".join(self._COMMANDS)})')
            return
        if not self._cmd_busy.acquire(blocking=False):
            self._reply(cmd, False, 'outra transação em andamento')
            return
        # Fora da thread do executor: a transação espera até
        # ft_modbus_timeout_s pela resposta no meio do stream, e segurar o
        # callback por meio segundo atrasaria /ft_sensor/wrench junto.
        threading.Thread(target=self._run_command, args=(cmd, args),
                         daemon=True, name='ft-cmd').start()

    @contextmanager
    def _hold_line(self):
        """Garante a linha 485 para UMA transação de comando.

        Em modo stream não há disputa: o laço de leitura só escuta, e a
        derivação do ft_cmd_channel já basta. Em modo POLLED o driver está
        perguntando na mesma linha, e `command_session()` não aninha — sem
        este handoff o Set_Zero falharia de forma INTERMITENTE, que é pior de
        diagnosticar que falhar sempre. `yield_line` faz o laço recuar e
        espera a transação em voo terminar.
        """
        if self._polled is None:
            yield
            return
        with self._polled.yield_line(
                timeout_s=self._modbus_timeout + 1.0) as obtido:
            if not obtido:
                raise ModbusError(
                    'o laço de aquisição polled não liberou a linha 485 a '
                    'tempo. Passe ft_mode:=stream ou aumente '
                    'ft_modbus_timeout_s.')
            yield

    def _run_command(self, cmd: str, args: dict) -> None:
        try:
            if cmd == 'mode':
                # Não toca na 485: só liga/desliga o laço que pergunta. Fica
                # ANTES do _hold_line porque tomar a linha do driver que
                # estamos prestes a destruir não faria sentido.
                ok, detail, data = self._switch_mode(args)
                self._reply(cmd, ok, detail, data)
                return
            with self._hold_line():
                with self._serial.command_session() as (write_fn, read_fn):
                    client = FtModbusClient(write_fn, read_fn,
                                            slave_id=self._modbus_slave,
                                            timeout_s=self._modbus_timeout)
                    ok, detail, data = self._dispatch(
                        FtDevice(client), cmd, args)
            self._reply(cmd, ok, detail, data)
        except FtModbusMapUnconfirmed as exc:
            # Caminho esperado enquanto o mapa não vier da bancada. Não é
            # falha do link: é a trava fazendo o trabalho dela.
            self._reply(cmd, False, str(exc), {'reason': 'map_unconfirmed'})
        except ModbusError as exc:
            self._reply(cmd, False, str(exc), {'reason': 'modbus'})
        except Exception as exc:
            self._reply(cmd, False, f'{type(exc).__name__}: {exc}',
                        {'reason': 'transport'})
        finally:
            self._cmd_busy.release()

    def _dispatch(self, dev, cmd: str, args: dict):
        """Executa um comando já validado. Devolve (ok, detalhe, dados)."""
        if cmd == 'zero':
            dev.set_zero()
            # O zero é no sensor, mas o tare do host guardava o offset ANTIGO:
            # mantê-lo somaria as duas correções e o zero sairia errado.
            with self._lock:
                self._tare = [0.0] * len(FT_AXES)
                self._tare_done = False
            self._tared_pub.publish(Bool(data=False))
            return True, 'zero aplicado no sensor; tare do host limpo', {}

        if cmd == 'rate':
            hz = int(args['hz'])
            dev.set_output_rate_hz(hz)
            return True, (f'taxa de saída = {hz} Hz. Só vale após RELIGAR o '
                          'sensor; depois ajuste ft_rate_hz do nó.'), {'hz': hz}

        if cmd == 'node_id':
            nid = int(args['node_id'])
            dev.set_node_id(nid)
            return True, (f'node ID = {nid}. Só vale após RELIGAR o sensor; '
                          'depois ajuste ft_modbus_slave_id do nó, senão o '
                          'canal de comando fala com um escravo que não '
                          'existe mais.'), {'node_id': nid}

        if cmd == 'baud':
            baud = int(args['baud'])
            dev.set_baud(baud)
            return True, (f'baud = {baud}. Só vale após RELIGAR o sensor; '
                          'depois ajuste ft_baud do nó e reabra a porta.'),                 {'baud': baud}

        if cmd == 'stream_start':
            dev.start_stream()
            return True, 'stream ligado', {}

        if cmd == 'stream_stop':
            dev.stop_stream()
            return True, ('stream desligado — /ft_sensor/wrench vai parar '
                          'até um stream_start.'), {}

        if cmd == 'probe':
            # Único que funciona sem o mapa confirmado: ler é inofensivo.
            info = dev.probe()
            if not info:
                return False, ('nenhum registrador conhecido respondeu — '
                               'confira o node ID e a fiação da 485, ou '
                               'preencha FT_MODBUS_MAP.'), {}
            return True, 'canal de comando vivo', info

        if cmd == 'scan':
            ini = int(args.get('start', 0x0000))
            fim = int(args.get('end', 0x0040))
            if not 0 <= ini <= fim <= 0xFFFF or (fim - ini) > 512:
                return False, 'faixa inválida (máximo 512 endereços)', {}
            # Timeout curto SÓ para a varredura: com o padrão de 0,5 s uma
            # faixa toda muda levaria meio minuto. Restaurado no finally
            # porque o mesmo cliente serve o resto da sessão.
            antes = dev.client.timeout_s
            dev.client.timeout_s = float(args.get('timeout_s', 0.12))
            try:
                scan = scan_registers(dev.client, ini, fim)
            finally:
                dev.client.timeout_s = antes
            palpites = suggest_map(scan, baud=self._baud,
                                   rate_hz=int(self._rate),
                                   node_id=self._modbus_slave)
            vivos = {a: v for a, (st, v) in scan.items() if st == 'ok'}
            return True, (
                f'{len(vivos)} de {len(scan)} endereços responderam. Os '
                'palpites são CANDIDATOS, não conclusão: confira contra o '
                'painel do cliente de fábrica antes de preencher '
                'FT_MODBUS_MAP.'), {
                'scan': {f'0x{a:04X}': v for a, v in vivos.items()},
                'suggest': {k: [f'0x{a:04X}:{como}' for a, como in v]
                            if k != 'node_id' else [f'0x{a:04X}' for a in v]
                            for k, v in palpites.items()},
                'text': format_scan(scan)}

        raise AssertionError(f'comando sem tratamento: {cmd}')

    def _switch_mode(self, args: dict):
        """Troca stream <-> polled em quente, sem reabrir a porta.

        Dá para fazer isto barato porque o driver polled não é dono de nada:
        ele dirige o `command_session()` do transporte, que continua no ar nos
        dois modos. Trocar de modo é só ligar ou desligar o laço.
        """
        novo = str(args.get('mode', '')).strip().lower()
        if novo not in FT_MODE_CHOICES:
            return False, (f'modo inválido: {novo!r} (aceitos: '
                           f'{", ".join(FT_MODE_CHOICES)})'), {}
        if novo == self._mode:
            return True, f'já estava em {novo}', {'mode': novo}
        if novo == FT_MODE_POLLED:
            self._polled = FtPolledDriver(
                self._serial, slave_id=self._modbus_slave,
                timeout_s=self._modbus_timeout, on_sample=self._on_sample)
            self._polled.start()
            teto = ft_polled_max_rate_hz(self._baud)
            detalhe = (f'polled ligado. Teto do fio {teto:.0f} Hz a '
                       f'{self._baud} baud; a taxa MEDIDA fica abaixo, '
                       'porque a latência de USB não entra nessa conta.')
        else:
            if self._polled is not None:
                self._polled.stop()
                self._polled = None
            detalhe = ('stream ligado. Se o sensor estiver parado, use '
                       'Start stream — em polled ele não precisa emitir.')
        self._mode = novo
        self._rate_warned = False
        return True, detalhe, {'mode': novo}

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
