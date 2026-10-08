#!/usr/bin/env python3
"""
imu_arm_teleop.py — Bridge: IMUs do WBAN-Inercial -> teleop cartesiano do CR10.

Lê os sensores inerciais (ESPs) com o protocolo e a fusão (Madgwick) do
projeto WBAN-Inercial, reduz shoulder+arm+forearm (relativos ao tronco) a um
vetor de posição da "mão" humana, e usa esse vetor (escalado e zerado no
início) como alvo cartesiano para a IK diferencial do touch_pack
(`manip3d.ik_step`), streamando o resultado pro CR10 real via ServoJ
(`real_driver.CR10RealDriver`).

FASE 1 (este script): só POSIÇÃO. A orientação do TCP fica travada na pose
inicial (R_lock). Seguir a orientação da mão é trabalho futuro, depois de
validar a posição com segurança.

LIMITAÇÃO CONHECIDA: a fusão do WBAN está rodando em modo Acc+Gyro apenas
(ver `core/dataprocessing.py`, a linha que zera o magnetômetro) — ou seja,
SEM referência de heading. O yaw de cada sensor deriva com o tempo, e isso
vaza pra estimativa de posição da mão. Espere deriva horizontal em sessões
longas; use `z<Enter>` para re-zerar sem reiniciar o processo, e prefira
sessões curtas até isso ser corrigido no WBAN.

SEGURANÇA — padrões conservadores:
  - dry_run=True por padrão. Só sai do dry-run com --live explícito.
  - Mesmo com --live, o braço só começa a seguir os sensores depois de você
    digitar 'start' no terminal.
  - Alvo é travado numa caixa de +-safety_box_m em torno da pose inicial.
  - Se o 'arm' ou o 'forearm' (os elos com length_m > 0) ficarem sem dado
    novo por > stale_s, o braço CONGELA (mantém a última pose comandada).

PRÉ-REQUISITOS (ver README.md ao lado):
  - Os ESPs calibrados (AG e, se for usar magnetômetro depois, MAG) via
    core/sensorcalibration.py do WBAN-Inercial.
  - PC na mesma rede Wi-Fi dos sensores.
  - config.json preenchido a partir de config.example.json.

Uso:
    python3 imu_arm_teleop.py --config config.json            # dry-run
    python3 imu_arm_teleop.py --config config.json --live     # braço real
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
import threading
import time
from pathlib import Path

import numpy as np

log = logging.getLogger('imu_arm_teleop')

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent  # cr10twin/
TOUCH_PACK_SRC_DEFAULT = REPO_ROOT / 'src' / 'touch_pack'


def _load_config(path: Path) -> dict:
    with open(path, 'r') as f:
        return json.load(f)


def _inject_sys_path(cfg: dict, touch_pack_src: Path) -> None:
    wban_dir = Path(cfg['wban_dir'])
    if not wban_dir.is_dir():
        raise SystemExit(f"wban_dir não existe: {wban_dir}")
    if not touch_pack_src.is_dir():
        raise SystemExit(f"touch_pack_src não existe: {touch_pack_src}")
    for p in (wban_dir, touch_pack_src):
        sp = str(p)
        if sp not in sys.path:
            sys.path.insert(0, sp)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', default=str(HERE / 'config.json'))
    ap.add_argument('--touch-pack-src', default=str(TOUCH_PACK_SRC_DEFAULT))
    ap.add_argument('--live', action='store_true',
                     help='Sai do dry-run e conecta no robô real.')
    ap.add_argument('--robot-ip', default=None, help='Sobrescreve robot_ip do config.')
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s %(levelname)-7s %(name)s: %(message)s',
    )

    cfg_path = Path(args.config)
    if not cfg_path.is_file():
        raise SystemExit(
            f"Config não encontrado: {cfg_path}\n"
            f"Copie {HERE / 'config.example.json'} para {cfg_path} e ajuste.")
    cfg = _load_config(cfg_path)

    _inject_sys_path(cfg, Path(args.touch_pack_src))

    # Imports tardios: só resolvem depois do sys.path acima.
    from core.communications import (  # noqa: E402
        CentralCtrl, CentralCtrlConfigServer, CentralCtrlSensorClient,
    )
    from core.dataprocessing import FormatData  # noqa: E402
    from PyQt6.QtCore import QCoreApplication, QThread, QTimer  # noqa: E402

    from touch_pack import manip3d  # noqa: E402
    from touch_pack import kinematics as tp_kin  # noqa: E402
    from touch_pack import real_driver as tp_driver  # noqa: E402

    bridge = TeleopBridge(cfg, manip3d, tp_kin, tp_driver,
                           robot_ip=args.robot_ip or cfg['robot_ip'],
                           live=args.live)

    app = QCoreApplication(sys.argv)

    # --- fiação WBAN: config server (descoberta multicast) + central_ctrl ---
    central_ctrl = CentralCtrl()
    config_server = CentralCtrlConfigServer()
    thread_config_server = QThread()
    config_server.moveToThread(thread_config_server)
    thread_config_server.started.connect(config_server.run)
    config_server.config_rxq.connect(central_ctrl.get_config_rxq)

    formatter = FormatData()
    formatter.quaternium.connect(bridge.on_quaternion)

    sensor_threads: dict[int, QThread] = {}
    sensor_clients: dict[int, object] = {}

    def create_sensor_client(params):
        sensor_id, ip = params
        if sensor_id in sensor_threads:
            return
        log.info('Sensor %s detectado em %s — abrindo stream de dados', sensor_id, ip)
        thread = QThread()
        client = CentralCtrlSensorClient(params)
        client.moveToThread(thread)
        client.connected.connect(central_ctrl.client_connect)
        client.decoded_data.connect(formatter.join_data)
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
            thread.wait(1000)
        bridge.on_sensor_removed(sensor_id)

    central_ctrl.list_sensor.connect(create_sensor_client)
    central_ctrl.remove_sensor.connect(remove_sensor_client)

    # live_list housekeeping (timeout de sensores) — mesmo ritmo da GUI original.
    housekeeping = QTimer()
    housekeeping.timeout.connect(central_ctrl.check_queues)
    housekeeping.start(1000)

    thread_config_server.start()

    # --- robô: conecta e captura a pose/orientação "home" ---
    bridge.connect_robot()

    # --- terminal: start / z (rezero) / stop ---
    def terminal_listener():
        mode = 'LIVE' if args.live else 'DRY-RUN'
        print(f"\n[{mode}] Pronto. Pose home capturada. Comandos:")
        print("  start<Enter>  — começa a seguir os sensores")
        print("  z<Enter>      — re-zera a pose de referência (corrige deriva)")
        print("  stop<Enter> ou Ctrl+C — para e sai\n")
        while True:
            try:
                cmd = input().strip().lower()
            except (EOFError, KeyboardInterrupt):
                cmd = 'stop'
            if cmd == 'start':
                bridge.start_following()
            elif cmd == 'z':
                bridge.rezero()
            elif cmd == 'stop':
                # QCoreApplication.quit() é thread-safe (posta um evento),
                # diferente de QTimer.singleShot chamado fora da thread Qt
                # (que nunca dispararia, porque esta thread não tem event loop).
                app.quit()
                break

    listener = threading.Thread(target=terminal_listener, daemon=True)
    listener.start()

    # --- laço de controle a rate_hz ---
    control_timer = QTimer()
    control_timer.timeout.connect(bridge.tick)
    control_timer.start(int(1000 / cfg.get('rate_hz', 30)))

    try:
        app.exec()
    finally:
        housekeeping.stop()
        control_timer.stop()
        for sid in list(sensor_threads):
            remove_sensor_client(sid)
        bridge.shutdown()
        # config_server.run() é um laço bloqueante próprio, não um event loop
        # Qt — quit()/wait() não se aplicam a ele. Mesmo padrão do
        # sensorcalibration.py do WBAN: sinaliza a flag e força a thread.
        config_server.close_loop(True)
        thread_config_server.terminate()
        thread_config_server.wait(1000)


class TeleopBridge:
    """Funde IMUs -> alvo cartesiano -> IK -> ServoJ. Sem estado de GUI."""

    def __init__(self, cfg: dict, manip3d, tp_kin, tp_driver,
                 robot_ip: str, live: bool):
        self.cfg = cfg
        self.manip3d = manip3d
        self.tp_kin = tp_kin
        self.tp_driver = tp_driver

        self.segment_to_sensor: dict[str, int] = cfg['segment_to_sensor']
        self.sensor_to_segment = {v: k for k, v in self.segment_to_sensor.items()}
        self.chain: list[tuple[str, float]] = [
            (link['segment'], float(link['length_m'])) for link in cfg['chain']
        ]
        self.axis_local = np.asarray(cfg['limb_axis_local'], dtype=float)
        self.motion_scale = float(cfg.get('motion_scale', 1.0))
        self.R_h2r = np.asarray(cfg['r_human_to_robot'], dtype=float)
        self.safety_box_m = np.asarray(cfg['safety_box_m'], dtype=float)
        self.stale_s = float(cfg.get('stale_s', 0.5))

        # T_end PRECISA ser o mesmo em toda FK/IK deste script — a pose "home"
        # e os alvos de IK têm que estar no mesmo referencial de TCP, senão o
        # alvo deriva pelo offset entre os dois acoplamentos (COVVI hand vs.
        # touch_tool) a cada tick, mesmo com os sensores parados.
        ee = cfg.get('tcp_end_effector', 'touch_tool')
        if ee == 'touch_tool':
            self.T_end = tp_kin.T_TOUCH_TOOL_ATTACH
        elif ee == 'hand':
            self.T_end = tp_kin.T_HAND_ATTACH
        else:
            raise ValueError(f"tcp_end_effector inválido: {ee!r} (use 'touch_tool' ou 'hand')")

        self._quat: dict[int, np.ndarray] = {}      # sensor_id -> [w,x,y,z]
        self._quat_t: dict[int, float] = {}          # sensor_id -> timestamp local
        self._lock = threading.Lock()

        self._zero_hand_vec: np.ndarray | None = None
        self._following = False
        self._frozen_logged = False

        self.drv = tp_driver.CR10RealDriver(ip=robot_ip, dry_run=not live)
        self.q = None          # pose articular atual (URDF, rad)
        self.home_T = None     # pose homogênea 4x4 no início
        self.home_q = None

    # --- robô ---

    def connect_robot(self) -> None:
        self.drv.connect()
        if not self.drv.dry_run:
            self.drv.enable()
            self.q = self.drv.read_joints_urdf()
        else:
            home = self._load_home_pose_deg()
            q_dobot = np.deg2rad(home)
            self.q = self.tp_kin.dobot_to_urdf(q_dobot)
        self.home_q = self.q.copy()
        self.home_T = self.tp_kin.forward_kinematics(self.q, T_end=self.T_end)
        log.info('Pose home capturada. TCP em %s (m)',
                  np.round(self.home_T[:3, 3], 4))

    def _load_home_pose_deg(self) -> np.ndarray:
        home_path = Path.home() / '.config' / 'touch_pack' / 'home_pose.json'
        try:
            with open(home_path) as f:
                d = json.load(f)
            from touch_pack.constants import ARM_JOINTS
            return np.array([d[j] for j in ARM_JOINTS], dtype=float)
        except Exception as exc:
            log.warning('Sem home_pose.json (%s); usando zeros para o dry-run.', exc)
            return np.zeros(6)

    def shutdown(self) -> None:
        try:
            self.drv.stop()
        except Exception:
            pass
        try:
            self.drv.close()
        except Exception:
            pass

    # --- comandos do terminal ---

    def start_following(self) -> None:
        self.rezero()
        self._following = True
        log.info('Seguindo os sensores.')

    def rezero(self) -> None:
        with self._lock:
            self._zero_hand_vec, _stale = self._hand_vec_locked()
        log.info('Pose de referência re-zerada.')

    # --- recepção de dados ---

    def on_quaternion(self, sensor_id: int, quat: np.ndarray) -> None:
        with self._lock:
            self._quat[sensor_id] = np.asarray(quat, dtype=float)
            self._quat_t[sensor_id] = time.monotonic()

    def on_sensor_removed(self, sensor_id: int) -> None:
        with self._lock:
            self._quat.pop(sensor_id, None)
            self._quat_t.pop(sensor_id, None)
        seg = self.sensor_to_segment.get(sensor_id)
        if seg:
            log.warning('Sensor do segmento "%s" (id=%s) desconectou.', seg, sensor_id)

    # --- matemática ---

    @staticmethod
    def _quat_to_R(q: np.ndarray) -> np.ndarray:
        w, x, y, z = q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def _hand_vec_locked(self) -> tuple[np.ndarray, bool]:
        """Vetor mão<-tronco no frame do tronco (soma da cadeia). Chamar com _lock.

        Retorna (vetor, stale) — stale=True se algum elo com length_m>0 não
        tiver dado (ausente ou obsoleto).
        """
        now = time.monotonic()
        trunk_id = self.segment_to_sensor.get('trunk')
        q_trunk = self._quat.get(trunk_id) if trunk_id is not None else None
        R_trunk = self._quat_to_R(q_trunk) if q_trunk is not None else np.eye(3)

        vec = np.zeros(3)
        stale = False
        for seg, length_m in self.chain:
            if length_m == 0.0:
                continue
            sid = self.segment_to_sensor.get(seg)
            q_seg = self._quat.get(sid) if sid is not None else None
            t_seg = self._quat_t.get(sid) if sid is not None else None
            if q_seg is None or t_seg is None or (now - t_seg) > self.stale_s:
                stale = True
                continue
            R_rel = R_trunk.T @ self._quat_to_R(q_seg)
            vec += R_rel @ (-length_m * self.axis_local)
        return vec, stale

    # --- laço de controle ---

    def tick(self) -> None:
        if self.q is None:
            return
        with self._lock:
            hand_vec, stale = self._hand_vec_locked()

        if not self._following:
            return

        if self._zero_hand_vec is None:
            self._zero_hand_vec = hand_vec

        if stale:
            if not self._frozen_logged:
                log.warning('Segmento rastreado obsoleto/ausente — braço CONGELADO.')
                self._frozen_logged = True
            return
        self._frozen_logged = False

        delta_human = hand_vec - self._zero_hand_vec
        delta_robot = self.R_h2r @ (self.motion_scale * delta_human)

        target = self.home_T[:3, 3] + delta_robot
        target = np.clip(target,
                          self.home_T[:3, 3] - self.safety_box_m,
                          self.home_T[:3, 3] + self.safety_box_m)

        result = self.manip3d.ik_step(self.q, target, T_end=self.T_end,
                                       R_lock=self.home_T[:3, :3])
        self.q = result.q
        if result.singular:
            log.warning('IK: Jacobiano singular neste tick.')
        if result.at_limit:
            log.debug('IK: junta saturou no limite.')

        try:
            self.drv.servo_j_urdf(self.q)
        except Exception as exc:
            log.error('Falha ao enviar ServoJ: %s — parando.', exc)
            self._following = False


if __name__ == '__main__':
    main()
