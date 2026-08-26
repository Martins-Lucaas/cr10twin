"""
mirror_node.py — Espelhamento sim → CR10 real SEM a GUI.

Antes, o modo MIRROR morava inteiro na palpation_gui (poll loop +
debounce MovJ), então `no_gui:=true` quebrava o espelhamento. Este nó
standalone reproduz o núcleo daquele comportamento:

  • Fase de palpação ATIVA (tudo que NÃO é IDLE/DONE/ABORTED — inclui
    CALIBRATING, TRANSIT e MODULATING): ServoJ a 33 Hz com a posição lida
    de /joint_states — latência mínima para o controle de força.
  • Fase inativa (IDLE/DONE/ABORTED): MovJ com debounce de 80 ms a partir
    do ÚLTIMO ponto publicado em /cr10_group_controller/joint_trajectory
    (cobre jog de outros publishers), idêntico ao padrão da DobotAPI.

Recursos exclusivos da GUI (drag teach, execução de movimentos salvos,
bridge de força) NÃO são replicados aqui.

Uso (launch): sobe automaticamente com control_mode:=mirror no_gui:=true.
  ros2 launch touch_pack tactile_cell.launch.py \
      end_effector:=touch_tool control_mode:=mirror no_gui:=true

Parâmetros ROS:
  robot_ip         ''     IP do CR10; vazio → ~/.config/touch_pack/robot.json
  servoj_period_s  0.030  Período do ServoJ. É ESTA taxa que governa o braço
                          real: o explorer pode publicar mais rápido que o
                          laço abaixo amostra o último q no ritmo daqui e
                          descarta o resto. Baixe-a (junto com o tick da onda
                          no explorer) para ondas trigonométricas de alta
                          frequência — vale para TODAS as fases, não só a onda.
"""
from __future__ import annotations

import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from touch_pack_msgs.msg import PalpationStatus

from .constants import ARM_JOINTS, PHASE_CODES, ROBOT_CONFIG_FILE
from .kinematics import urdf_to_dobot
from .real_driver import (
    CR10RealDriver, CR10RealDriverConfig, CR10RealDriverError,
    SERVOJ_T_MIN_S,
)

_PERIOD_S = 0.030          # 33 Hz — mesmo período do streaming do explorer
_MOVJ_DEBOUNCE_S = 0.08    # coalesce de publicações em rajada no jog
# Banda morta do ServoJ: abaixo dela o alvo é considerado igual ao anterior e
# o comando não é reenviado. Era 1e-4 rad — que num CR10 de ~1,2 m de alcance
# vale ~120 µm de TCP. Isso é da ORDEM da onda trigonométrica inteira: uma
# amplitude de 700 µm sairia quantizada em ~6 degraus, e qualquer onda menor
# seria suprimida por completo, silenciosamente.
#
# 1e-5 rad ≈ 12 µm de TCP, ainda acima do ruído de encoder (~1,7e-5 rad por
# LSB de 0,001°) mas fino o bastante para a onda. Reenviar um ServoJ
# redundante é barato — o envio é fire-and-forget.
_SERVOJ_DEADBAND_RAD = 1.0e-5
_RECONNECT_BACKOFF_S = (2.0, 5.0, 10.0, 30.0)

# Fases em que o braço NÃO está sendo comandado pelo explorer. Tudo o que não
# está aqui é ativo, e a decisão é do complemento DE PROPÓSITO: enquanto isto
# era uma allowlist ('HOME', 'DESCENDING', ...), toda fase nova nascia
# invisível para o espelho. Foi o que aconteceu com CALIBRATING e TRANSIT —
# as duas percorrem retas cartesianas (_cartesian_batch_to) publicadas como UMA
# JointTrajectory, e com elas fora da lista o ServoJ calava e quem assumia era
# o MovJ debounced do _cb_trajectory: o braço real fazia um PTP ARTICULAR até
# o ponto final em vez da reta, e o primeiro ServoJ da fase seguinte saía sem
# banda morta (o _last_servoj_q tinha sido zerado) para uma pose dezenas de mm
# adiante, com t=30 ms. Fase desconhecida agora conta como ATIVA, que é o lado
# seguro e é também a regra que a palpation_gui já usava.
_IDLE_PHASES = ('IDLE', 'DONE', 'ABORTED')
# Derivada, para log e teste: a lista explícita do que É ativo hoje.
_ACTIVE_PHASES = tuple(sorted(p for p in PHASE_CODES
                              if p not in _IDLE_PHASES))


def _is_active(phase: str) -> bool:
    """True quando o explorer está comandando o braço nesta fase."""
    return phase not in _IDLE_PHASES


class MirrorNode(Node):

    def __init__(self):
        super().__init__('mirror_node')
        self.declare_parameter('robot_ip', '')
        # Período do ServoJ, em segundos. Default 0,030 (33 Hz) — o mesmo de
        # sempre, então o comportamento não muda sem alguém pedir.
        #
        # Existe porque a onda trigonométrica de alta frequência precisa de
        # pontos por período, e ESTA é a taxa que governa o braço real: o
        # explorer pode publicar a 250 Hz que o laço abaixo amostra o último
        # q no ritmo dele e descarta o resto. Para uma onda de f Hz com 8
        # pontos por período, os TRÊS têm de acompanhar: o tick da onda no
        # explorer, este período, e o `t=` do ServoJ no driver (que este nó
        # repassa). Baixar isto acelera o ServoJ de TODAS as fases, não só da
        # onda — por isso é opt-in.
        self.declare_parameter('servoj_period_s', _PERIOD_S)

        self._servoj_period_s = float(
            self.get_parameter('servoj_period_s').value or _PERIOD_S)
        # SATURADO na faixa do firmware: o `t` do ServoJ vale [0.02, 3600] s
        # ("Dobot TCP/IP Remote Control Interface Guide V4.5.1"). Um período
        # menor não acelera o braço — o controlador recusa o ponto —, então
        # aceitá-lo aqui só produzia um laço publicando no vazio.
        if self._servoj_period_s < SERVOJ_T_MIN_S:
            self.get_logger().warn(
                f'servoj_period_s={self._servoj_period_s*1e3:.1f} ms abaixo do '
                f'mínimo do ServoJ ({SERVOJ_T_MIN_S*1e3:.0f} ms) — saturado. '
                f'A maior frequência de onda rastreável continua sendo '
                f'{1.0/(SERVOJ_T_MIN_S*8):.2f} Hz.')
            self._servoj_period_s = SERVOJ_T_MIN_S
        self._lock = threading.Lock()
        self._phase: str = 'IDLE'
        self._latest_q: list[float] | None = None
        self._driver: CR10RealDriver | None = None
        self._connected = False
        self._stop = threading.Event()
        self._last_servoj_q: np.ndarray | None = None
        self._movj_timer: threading.Timer | None = None
        self._movj_lock = threading.Lock()

        self.create_subscription(
            JointState, '/joint_states', self._cb_joints, 50)
        self.create_subscription(
            PalpationStatus, '/palpation/status', self._cb_status, 10)
        self.create_subscription(
            JointTrajectory, '/cr10_group_controller/joint_trajectory',
            self._cb_trajectory, 1)

        threading.Thread(target=self._connect_loop, daemon=True,
                         name='mirror-connect').start()
        threading.Thread(target=self._servoj_loop, daemon=True,
                         name='mirror-servoj').start()
        self.get_logger().info('mirror_node ativo — aguardando CR10 real.')

    # ── conexão ──────────────────────────────────────────────────────
    def _robot_ip(self) -> str:
        ip = str(self.get_parameter('robot_ip').value or '').strip()
        if ip:
            return ip
        try:
            with open(ROBOT_CONFIG_FILE) as fh:
                ip = str(json.load(fh).get('robot_ip', '')).strip()
        except (OSError, json.JSONDecodeError, AttributeError):
            ip = ''
        return ip or '192.168.5.2'

    def _connect_loop(self) -> None:
        """Conecta (e reconecta com backoff) ao controlador CR10."""
        attempt = 0
        while not self._stop.is_set():
            if self._connected:
                # wait() e não sleep(): o shutdown não fica preso 1 s aqui.
                self._stop.wait(1.0)
                continue
            ip = self._robot_ip()
            drv = None
            try:
                drv = CR10RealDriver(
                    ip=ip,
                    config=CR10RealDriverConfig(
                        servoj_period_s=self._servoj_period_s))
                drv.connect()
                drv.enable()
                with self._lock:
                    self._driver = drv
                    self._connected = True
                drv = None          # publicado: quem fecha agora é _drop_connection
                attempt = 0
                self.get_logger().info(f'CR10 real conectado em {ip}.')
            except Exception as exc:
                # connect() pode ter aberto os dois sockets e subido o
                # keepalive antes de enable() falhar — típico com o
                # controlador em modo LOCAL, onde enable() só levanta depois
                # de ~16 s. Sem este close cada tentativa deixaria 2 sockets e
                # uma thread vivos para sempre, até esgotar as sessões do
                # dashboard do robô.
                if drv is not None:
                    try:
                        drv.close()
                    except Exception:
                        pass
                wait = _RECONNECT_BACKOFF_S[
                    min(attempt, len(_RECONNECT_BACKOFF_S) - 1)]
                attempt += 1
                self.get_logger().warning(
                    f'CR10 em {ip} indisponível ({exc}) — '
                    f'nova tentativa em {wait:.0f}s.')
                self._stop.wait(wait)

    def _drop_connection(self, exc: Exception) -> None:
        self.get_logger().warning(
            f'Conexão com o CR10 perdida ({exc}) — reconectando.')
        with self._lock:
            drv = self._driver
            self._driver = None
            self._connected = False
        if drv is not None:
            try:
                drv.close()
            except Exception:
                pass

    # ── callbacks ────────────────────────────────────────────────────
    def _cb_status(self, msg: PalpationStatus) -> None:
        with self._lock:
            self._phase = msg.phase

    def _cb_joints(self, msg: JointState) -> None:
        pos = dict(zip(msg.name, msg.position))
        try:
            q = [float(pos[j]) for j in ARM_JOINTS]
        except KeyError:
            return   # mensagem parcial (mão) — ignorar
        with self._lock:
            self._latest_q = q

    def _cb_trajectory(self, msg: JointTrajectory) -> None:
        """Jog (fase inativa): MovJ debounced para o último alvo."""
        with self._lock:
            phase = self._phase
            connected = self._connected
        if not connected or _is_active(phase):
            return   # palpação ativa → ServoJ loop assume
        if not msg.points:
            return
        target = list(msg.points[-1].positions)
        if len(target) < 6:
            return
        with self._movj_lock:
            if self._movj_timer is not None:
                self._movj_timer.cancel()
            self._movj_timer = threading.Timer(
                _MOVJ_DEBOUNCE_S, self._movj_send, args=[target[:6]])
            self._movj_timer.daemon = True
            self._movj_timer.start()

    def _movj_send(self, q_urdf: list[float]) -> None:
        with self._lock:
            drv = self._driver
            phase = self._phase
        if drv is None or _is_active(phase):
            return   # race guard: fase mudou durante o debounce
        try:
            q_dobot_deg = list(np.degrees(
                urdf_to_dobot(np.asarray(q_urdf, dtype=np.float64))))
            drv.mov_j_joint_deg(q_dobot_deg)
        except CR10RealDriverError as exc:
            self._drop_connection(exc)

    # ── ServoJ loop (palpação ativa) ─────────────────────────────────
    def _servoj_loop(self) -> None:
        period = self._servoj_period_s
        t_next = time.monotonic() + period
        while not self._stop.is_set():
            now = time.monotonic()
            self._stop.wait(max(0.0, t_next - now))
            t_next += period
            if t_next < time.monotonic():
                t_next = time.monotonic() + period

            with self._lock:
                drv = self._driver
                connected = self._connected
                phase = self._phase
                q = self._latest_q
            if not connected or drv is None or q is None:
                continue
            if not _is_active(phase):
                self._last_servoj_q = None
                continue
            q_new = np.asarray(q, dtype=np.float64)
            last = self._last_servoj_q
            if (last is not None
                    and float(np.max(np.abs(q_new - last)))
                    < _SERVOJ_DEADBAND_RAD):
                continue   # estacionário — sem ServoJ redundante
            try:
                try:
                    drv.servo_j_urdf(q)
                except CR10RealDriverError:
                    drv.prepare_servoj()
                    drv.servo_j_urdf(q)
                self._last_servoj_q = q_new
            except CR10RealDriverError as exc:
                self._drop_connection(exc)

    def destroy_node(self):
        self._stop.set()
        with self._lock:
            drv = self._driver
            self._driver = None
        if drv is not None:
            try:
                drv.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MirrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
