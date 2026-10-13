"""imu_node — IMUs do WBAN-Inercial (ESPs por Wi-Fi) → sensor_msgs/Imu.

Publica um tópico por sensor, `/imu/sensor_<id>` (frame `imu_<id>`):
  orientation          Madgwick do WBAN (com magnetômetro onde houver
                       calibração MAG), sensor → Terra (x norte, z cima)
  angular_velocity     rad/s, já com o offset de calibração do giro
  linear_acceleration  m/s², já com o offset de calibração do acelerômetro

A aquisição e a fusão NÃO são reimplementadas aqui: o nó importa o
`core/` do projeto WBAN-Inercial (parâmetro `wban_dir`) — o mesmo código
da GUI KINESIS. Esse código é PyQt6 (QObject/sinais/QThread), então o laço
principal deste processo é o do Qt e o rclpy gira numa thread.

Cada ESP descoberto por multicast é posto em RUN (AGM, `rate_hz`) assim que
o TCP conecta; ao encerrar, todos voltam a HOLD.

Calibração (a mesma rotina do WBAN, `core/sensorcalibration.py`), pedida por
std_msgs/String em CALIB_REQ_TOPIC: "AG <id>" | "MAG <id>" (id = all → todos
os conectados); "CANCEL <id>" aborta sem gravar nada. AG = sensor PARADO e plano por 10 s a 500 Hz (offsets de acc e
giro); MAG = girar devagar em todas as direções por 30 s (elipsoide hard/soft
iron). Durante a coleta o sensor NÃO publica /imu/sensor_<id>; no fim o
calibration.json é recarregado e a orientação recomeça do zero (parado ~2 s).
Andamento e resultado saem em CALIB_STATUS_TOPIC (JSON).

Dependências no python do ROS (numpy<2!):
    pip install --user PyQt6==6.10.2 ahrs==0.3.1 "numpy<2"
"""
from __future__ import annotations

import json
import os
import signal
import sys
import threading

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from .lc_filter import QOS_SENSOR

DEFAULT_WBAN_DIR = os.path.expanduser(
    '~/WBAN-Inercial/Software main/Sensores Inerciais Nova Interface')
RATES_HZ = (500, 400, 300, 200, 100)
CALIB_REQ_TOPIC = '/imu/calibrate'
CALIB_STATUS_TOPIC = '/imu/calibration'
CALIB_KINDS = ('AG', 'MAG')


def imu_topic(sensor_id: int) -> str:
    return f'/imu/sensor_{int(sensor_id)}'


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.wban_dir = str(self.declare_parameter('wban_dir', DEFAULT_WBAN_DIR).value)
        rate = int(self.declare_parameter('rate_hz', 100).value)
        if rate not in RATES_HZ:
            self.get_logger().warn(f'rate_hz={rate} não suportado pelo firmware; usando 100')
            rate = 100
        self.rate_hz = rate
        # Vazio = config/calibration.json do próprio WBAN.
        self.calibration_file = str(self.declare_parameter('calibration_file', '').value)
        # Interface (IP local) onde estão os ESPs. Vazio = escolha do WBAN, que
        # prefere a rota default — com cabo e Wi-Fi ligados, costuma ser o cabo.
        self.iface_ip = str(self.declare_parameter('iface_ip', '').value).strip()
        self._pubs: dict[int, object] = {}
        self.calib_status_pub = self.create_publisher(String, CALIB_STATUS_TOPIC, 10)
        self.on_calib_request = None   # ligado em main(), já com o Qt de pé
        self.create_subscription(String, CALIB_REQ_TOPIC, self._cb_calib, 10)

    def _cb_calib(self, msg: String) -> None:
        if self.on_calib_request is not None:
            self.on_calib_request(msg.data)

    def calib_status(self, **fields) -> None:
        self.calib_status_pub.publish(String(data=json.dumps(fields)))

    def publish(self, sensor_id: int, q_wxyz, gyr_dps, acc_ms2) -> None:
        pub = self._pubs.get(sensor_id)
        if pub is None:
            pub = self.create_publisher(Imu, imu_topic(sensor_id), QOS_SENSOR)
            self._pubs[sensor_id] = pub
            self.get_logger().info(f'sensor {sensor_id} → {imu_topic(sensor_id)}')
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'imu_{sensor_id}'
        w, x, y, z = (float(v) for v in q_wxyz)
        msg.orientation.w, msg.orientation.x = w, x
        msg.orientation.y, msg.orientation.z = y, z
        gx, gy, gz = np.radians(gyr_dps)
        msg.angular_velocity.x, msg.angular_velocity.y = float(gx), float(gy)
        msg.angular_velocity.z = float(gz)
        ax, ay, az = (float(v) for v in acc_ms2)
        msg.linear_acceleration.x, msg.linear_acceleration.y = float(ax), float(ay)
        msg.linear_acceleration.z = float(az)
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    log = node.get_logger()

    if not os.path.isdir(node.wban_dir):
        log.error(f'wban_dir não existe: {node.wban_dir}')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    sys.path.insert(0, node.wban_dir)
    # sensorcalibration importa pyqtgraph.opengl só para o gráfico 3D do
    # modo linha de comando; aqui ele nunca é usado e o pacote não existe no
    # python do ROS.
    import types
    for name in ('pyqtgraph', 'pyqtgraph.opengl'):
        sys.modules.setdefault(name, types.ModuleType(name))
    try:
        from core import commands
        from core import sensorcalibration
        from core import communications as wban_comm
        from core.communications import (
            CentralCtrl, CentralCtrlConfigServer, CentralCtrlSensorClient,
        )
        from core.dataprocessing import FormatData
        from PyQt6.QtCore import (
            QCoreApplication, QObject, QThread, QTimer, pyqtSignal, pyqtSlot,
        )
    except ImportError as exc:
        log.error(f'Não foi possível importar o WBAN ({exc}). No python do ROS: '
                  'pip install --user PyQt6==6.10.2 ahrs==0.3.1 "numpy<2"')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    if node.iface_ip:
        # communications importou as funções pelo nome: é lá que se troca.
        wban_comm.get_default_ip = lambda: node.iface_ip
        wban_comm.get_default_ip_with_warning = lambda: node.iface_ip
    state_run = commands.STATE_NAMES.index('RUN')
    state_hold = commands.STATE_NAMES.index('HOLD')
    rate_id = commands.RATE_NAMES.index(str(node.rate_hz))
    axis_agm = commands.AXIS_NAMES.index('AGM')
    acc_fsr = FormatData.DEFAULT_AFS_SEL
    gyr_fsr = FormatData.DEFAULT_FS_SEL

    app = QCoreApplication(sys.argv[:1])

    if node.calibration_file:
        FormatData.CALIBRATION_FILE = node.calibration_file
        sensorcalibration.CALIBRATION_FILE = node.calibration_file
    formatter = FormatData()
    formatter.update_sampling_rate(node.rate_hz)
    calib = sensorcalibration.CalibrationManager()
    calibrating: dict[int, str] = {}   # sensor → 'AG' | 'MAG', só na thread do Qt

    class Sink(QObject):
        """Vive na thread do Qt: os pacotes chegam das threads dos clientes
        por conexão enfileirada, e o quaternion sai de join_data na mesma
        chamada (conexão direta), já pareado com a amostra que o gerou."""

        def __init__(self):
            super().__init__()
            self._q = None

        @pyqtSlot(int, np.ndarray)
        def on_quat(self, _sensor, q):
            self._q = q

        @pyqtSlot(object)
        def on_data(self, data):
            sid = int(data.sensor_id)
            if sid in calibrating:
                calib.process_data(data)
                return
            acc = formatter.convert_data(
                np.array([data.ax, data.ay, data.az], dtype=float), 'A', sensor_id=sid)
            gyr = formatter.convert_data(
                np.array([data.gx, data.gy, data.gz], dtype=float), 'G', sensor_id=sid)
            self._q = None
            formatter.join_data(data)
            if self._q is not None:   # None nas ~2 s de orientação inicial
                node.publish(sid, self._q, gyr, acc)

    sink = Sink()
    formatter.quaternium.connect(sink.on_quat)

    central_ctrl = CentralCtrl()
    config_server = CentralCtrlConfigServer()
    thread_config_server = QThread()
    config_server.moveToThread(thread_config_server)
    thread_config_server.started.connect(config_server.run)
    config_server.config_rxq.connect(central_ctrl.get_config_rxq)

    sensor_threads: dict[int, QThread] = {}
    sensor_clients: dict[int, object] = {}

    def on_client_connected(params):
        sensor_id, ok = params
        if ok:
            log.info(f'sensor {sensor_id} conectado → RUN {node.rate_hz} Hz AGM '
                     '(mantenha parado ~2 s para a orientação inicial)')
            central_ctrl.send_config((sensor_id, state_run, rate_id, axis_agm,
                                      acc_fsr, gyr_fsr))

    def create_sensor_client(params):
        sensor_id, _ip = params
        if sensor_id in sensor_threads:
            return
        thread = QThread()
        client = CentralCtrlSensorClient(params)
        client.moveToThread(thread)
        client.connected.connect(central_ctrl.client_connect)
        client.connected.connect(on_client_connected)
        client.decoded_data.connect(sink.on_data)
        thread.started.connect(client.run)
        thread.start()
        sensor_threads[sensor_id] = thread
        sensor_clients[sensor_id] = client

    def remove_sensor_client(sensor_id):
        thread = sensor_threads.pop(sensor_id, None)
        client = sensor_clients.pop(sensor_id, None)
        if client is not None:
            client.close_loop(True)
        if thread is not None and thread.isRunning():
            thread.quit()
            thread.wait(1500)
        # Na reconexão a orientação recomeça do zero, com o sensor parado,
        # em vez de continuar de um quaternion velho.
        formatter.Q.pop(sensor_id, None)
        formatter.init_buffer.pop(sensor_id, None)
        if calibrating.pop(sensor_id, None):
            calib.active_sensors.pop(sensor_id, None)
            node.calib_status(state='error', kind=None, sensor=sensor_id,
                              msg='sensor perdido durante a calibração')
        log.warn(f'sensor {sensor_id} perdido')

    # ── Calibração ────────────────────────────────────────────────────
    def send_rate(sid: int, hz: int) -> None:
        central_ctrl.send_config((sid, state_run, commands.RATE_NAMES.index(str(hz)),
                                  axis_agm, acc_fsr, gyr_fsr))

    def cancel_calibration(arg: str) -> None:
        sids = list(calibrating) if arg.lower() == 'all' else (
            [int(arg)] if arg.isdigit() else [])
        for sid in sids:
            kind = calibrating.pop(sid, None)
            if kind is None:
                continue
            calib.active_sensors.pop(sid, None)
            formatter.Q.pop(sid, None)
            formatter.init_buffer.pop(sid, None)
            if sid in sensor_clients:
                send_rate(sid, node.rate_hz)
            node.calib_status(state='cancelled', kind=kind, sensor=sid)
            log.info(f'sensor {sid}: calibração {kind} cancelada')

    def start_calibration(text: str) -> None:
        parts = text.split()
        kind = parts[0].upper() if parts else ''
        if kind == 'CANCEL' and len(parts) == 2:
            cancel_calibration(parts[1])
            return
        if kind not in CALIB_KINDS or len(parts) != 2:
            node.calib_status(state='error', kind=kind, sensor=None,
                              msg=f'pedido inválido: {text!r} (use "AG 4", "MAG all")')
            return
        if calibrating and kind not in calibrating.values():
            # O CalibrationManager guarda UM tipo por vez.
            node.calib_status(state='error', kind=kind, sensor=None,
                              msg='outra calibração (de outro tipo) em andamento')
            return
        targets = (sorted(sensor_clients) if parts[1].lower() == 'all'
                   else [int(parts[1])] if parts[1].isdigit() else [])
        if not targets:
            node.calib_status(state='error', kind=kind, sensor=None,
                              msg='nenhum sensor conectado' if parts[1].lower() == 'all'
                              else f'sensor inválido: {parts[1]}')
            return
        for sid in targets:
            if sid not in sensor_clients:
                node.calib_status(state='error', kind=kind, sensor=sid,
                                  msg='sensor não conectado')
                continue
            if sid in calibrating:
                continue
            hz = sensorcalibration.FREQ_AG if kind == 'AG' else sensorcalibration.FREQ_MAG
            calibrating[sid] = kind
            calib.start_calibration(sid, kind, hz)
            send_rate(sid, hz)
            log.info(f'sensor {sid}: calibração {kind} a {hz} Hz')

    def on_calib_started(sid, kind, duration):
        node.calib_status(state='started', kind=kind, sensor=sid, duration_s=duration)

    def on_calib_progress(sid, kind, progress):
        node.calib_status(state='progress', kind=kind, sensor=sid,
                          progress=round(float(progress), 1))

    def on_calib_finished(result):
        sid, kind = result.get('sensor_id'), result.get('type')
        calibrating.pop(sid, None)
        formatter.reload_calibration()
        # Offsets novos → a orientação recomeça do zero, com o sensor parado.
        formatter.Q.pop(sid, None)
        formatter.init_buffer.pop(sid, None)
        if sid in sensor_clients:
            send_rate(sid, node.rate_hz)
        ok = result.get('status') == 'success'
        extra = {}
        if ok and kind == 'MAG':
            m = (result['A_matrix'] @ (result['M_raw'] - result['V_bias']).T).T
            r = np.linalg.norm(m, axis=1)
            # Coeficiente de variação do raio: a métrica do WBAN (ideal < 3 %).
            extra['cov_pct'] = round(float(r.std() / r.mean() * 100.0), 2)
        node.calib_status(state='done' if ok else 'error', kind=kind, sensor=sid,
                          msg='' if ok else 'coleta sem dados suficientes', **extra)
        log.info(f'sensor {sid}: calibração {kind} '
                 f'{"concluída" if ok else "FALHOU"} {extra or ""}')

    calib.calibration_started.connect(on_calib_started)
    calib.calibration_progress.connect(on_calib_progress)
    calib.calibration_finished.connect(on_calib_finished)

    class CalibBridge(QObject):
        """Pedido chega na thread do rclpy; o sinal o entrega na do Qt."""
        request = pyqtSignal(str)

    bridge = CalibBridge()
    bridge.request.connect(start_calibration)
    node.on_calib_request = bridge.request.emit

    central_ctrl.list_sensor.connect(create_sensor_client)
    central_ctrl.remove_sensor.connect(remove_sensor_client)

    housekeeping = QTimer()
    housekeeping.timeout.connect(central_ctrl.check_queues)
    housekeeping.start(1000)

    def shutdown(*_):
        central_ctrl.send_config((0xFF, state_hold, rate_id, axis_agm, acc_fsr, gyr_fsr))
        for sid in list(sensor_threads):
            remove_sensor_client(sid)
        config_server.close_loop(True)
        thread_config_server.quit()
        thread_config_server.wait(3000)
        app.quit()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    # Sinais Python só são tratados entre bytecodes: sem este timer, o laço
    # C++ do Qt seguraria o SIGINT do launch até chegar algum evento.
    wake = QTimer()
    wake.timeout.connect(lambda: None)
    wake.start(200)

    # Executor próprio (e não rclpy.spin numa thread solta): precisa ser
    # parado e juntado antes do shutdown, senão o processo aborta na saída.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    thread_config_server.start()
    log.info(f'imu_node pronto (WBAN em {node.wban_dir}); aguardando ESPs…')
    try:
        app.exec()
    finally:
        executor.shutdown()
        spin.join(timeout=2)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
