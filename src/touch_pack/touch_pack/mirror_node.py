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
  payload_kg, payload_cog_z_mm   carga no flange (SetPayload)

Falha de comando, perda de conexão em palpação ou /palpation/freeze TRAVAM o
espelho: nenhum ServoJ, MovJ ou enable sai até o serviço ~/rearm.
"""
from __future__ import annotations

import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from touch_pack_msgs.msg import PalpationStatus

from .constants import (
    ARM_JOINTS, PHASE_CODES, ROBOT_CONFIG_FILE, SERVOJ_DEADBAND_RAD,
    SERVO_SYNC_TOL_RAD, SYNC_FEEDBACK_GRACE_S, mirror_step_error,
)
from .kinematics import urdf_to_dobot
from .real_driver import (
    CR10FeedbackTimeout, CR10RealDriver, CR10RealDriverConfig,
    CR10RealDriverError,
    FEEDBACK_PORT, SERVOJ_T_MIN_S,
)

_PERIOD_S = 0.030          # 33 Hz — mesmo período do streaming do explorer
_MOVJ_DEBOUNCE_S = 0.08    # coalesce de publicações em rajada no jog
# Banda morta do ServoJ — importada de constants.py, onde mora a fonte única
# (a palpation_gui comanda o MESMO braço e o tactile_explorer avisa quando a
# onda pedida é menor que ela). O alias local existe porque este módulo e os
# testes já a chamam assim.
_SERVOJ_DEADBAND_RAD = SERVOJ_DEADBAND_RAD
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
        # 30004 (8 ms) ou 30005 (200 ms) — ver FEEDBACK_PORT_SLOW no driver.
        self.declare_parameter('feedback_port', FEEDBACK_PORT)
        self._feedback_port = int(
            self.get_parameter('feedback_port').value or FEEDBACK_PORT)

        self._payload_kg = float(self.declare_parameter(
            'payload_kg', 0.5).value or 0.5)
        self._payload_cog_z_mm = float(self.declare_parameter(
            'payload_cog_z_mm', 50.0).value or 50.0)

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
        self._hold = ''             # motivo da trava; '' = solto
        self._rearm_lock = threading.Lock()
        self._rearm_drv: CR10RealDriver | None = None
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
        self.create_subscription(
            Empty, '/palpation/freeze', self._cb_freeze, 10,
            # Grupo próprio: o rearme bloqueia por segundos, e um freeze no
            # meio dele tem de ser atendido na hora.
            callback_group=MutuallyExclusiveCallbackGroup())
        self._freeze_pub = self.create_publisher(Empty, '/palpation/freeze', 10)
        self.create_service(
            Trigger, '~/rearm', self._srv_rearm,
            callback_group=MutuallyExclusiveCallbackGroup())

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
            if self._connected or self._hold:
                # wait() e não sleep(): o shutdown não fica preso 1 s aqui.
                self._stop.wait(1.0)
                continue
            ip = self._robot_ip()
            drv = None
            try:
                drv = self._new_driver(ip)
                drv.connect()
                if not self._hold:
                    drv.enable()
                with self._lock:
                    self._driver = pub = drv
                    self._connected = True
                    preso = self._hold
                drv = None          # publicado: quem fecha agora é _drop_connection
                if preso:           # freeze veio durante connect/enable
                    self._estop_driver(pub)
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

    def _new_driver(self, ip: str) -> CR10RealDriver:
        return CR10RealDriver(
            ip=ip,
            config=CR10RealDriverConfig(
                servoj_period_s=self._servoj_period_s,
                feedback_port=self._feedback_port,
                payload_kg=self._payload_kg,
                payload_cog_m=(0.0, 0.0, self._payload_cog_z_mm * 1e-3)))

    # ── trava ────────────────────────────────────────────────────────
    def _engage_hold(self, motivo: str) -> None:
        """Trava o espelho e para o braço. Quem solta é o ~/rearm."""
        with self._lock:
            self._hold = motivo
            drv = self._driver
        with self._movj_lock:
            if self._movj_timer is not None:
                self._movj_timer.cancel()
        self._last_servoj_q = None
        self.get_logger().error(
            f'[MIRROR] TRAVADO: {motivo}. Rearme: ros2 service call '
            f'{self.get_name()}/rearm std_srvs/srv/Trigger')
        if drv is not None:
            self._estop_driver(drv)

    def _estop_driver(self, drv) -> None:
        self._estop_pedido = True
        if not drv.emergency_stop():
            self.get_logger().error(
                '[MIRROR] E-STOP não confirmado pelo controlador — use o '
                'botão físico.')

    def _cb_freeze(self, _msg: Empty) -> None:
        self._freeze_seq += 1
        if not self._hold:         # o próprio freeze também volta por aqui
            self._engage_hold('/palpation/freeze')
            return
        drv = self._rearm_drv      # rearme em curso: o freeze o vence
        if drv is not None:
            self._estop_driver(drv)

    # Classe, não instância: valores antes da 1ª mensagem.
    _freeze_seq = 0
    _rearm_drv = None
    # E-STOP mandado e ainda não solto: sobrevive ao driver que o mandou.
    _estop_pedido = False
    _latest_q_ts = math.inf
    _last_servoj_t = 0.0
    _sync_fail_t = None

    def _srv_rearm(self, _req, resp):
        """Rearme explícito: reconecta se preciso, solta o E-STOP, habilita e
        só libera se a pose real bate com a do simulador."""
        if not self._rearm_lock.acquire(blocking=False):
            resp.success, resp.message = False, 'rearme já em curso'
            return resp
        novo = None
        seq = self._freeze_seq
        try:
            with self._lock:
                drv, q = self._driver, self._latest_q
            if not self._hold:
                resp.success, resp.message = True, 'não estava travado'
                return resp
            if drv is not None and not drv.is_connected():
                # Socket morto não volta: descarta e abre outro.
                self._estop_pedido |= drv.estop_engaged
                self._drop_connection(CR10RealDriverError('socket morto'))
                drv = None
            if drv is None:
                novo = drv = self._new_driver(self._robot_ip())
                drv.connect()
            self._rearm_drv = drv
            if self._freeze_seq != seq:      # veio durante o connect
                self._estop_driver(drv)
                raise CR10RealDriverError('FREEZE durante o rearme')
            # Driver novo não sabe do E-STOP que o anterior deixou acionado.
            if drv.estop_engaged or self._estop_pedido:
                drv.release_emergency_stop()
            else:
                drv.enable()
            if q is None:
                raise CR10RealDriverError('sem /joint_states do simulador')
            self._check_sync(drv, q)
            with self._lock:
                if self._freeze_seq == seq:
                    self._hold, self._estop_pedido = '', False
                self._driver, self._connected = drv, True
            novo = None
            if self._hold:
                self._estop_driver(drv)
                raise CR10RealDriverError('FREEZE durante o rearme')
            resp.success, resp.message = True, 'espelho rearmado'
        except (CR10RealDriverError, OSError) as exc:
            resp.success, resp.message = False, f'segue travado: {exc}'
        finally:
            self._rearm_drv = None
            if novo is not None:
                novo.close()
            self._rearm_lock.release()
        return resp

    @staticmethod
    def _check_sync(drv, q) -> None:
        dif = float(np.max(np.abs(
            drv.read_joints_urdf_latest() - np.asarray(q))))
        if dif > SERVO_SYNC_TOL_RAD:
            raise CR10RealDriverError(
                f'sim e braço real divergem {np.degrees(dif):.1f}°')

    def _fault(self, exc: Exception) -> None:
        """Comando recusado ou sem resposta: NÃO limpa o erro nem reenvia."""
        with self._lock:
            drv = self._driver
        self._engage_hold(str(exc))
        self._freeze_pub.publish(Empty())
        if drv is not None and not drv.is_connected():
            self._drop_connection(exc)

    def _drop_connection(self, exc: Exception) -> None:
        self.get_logger().warning(
            f'Conexão com o CR10 perdida ({exc}).')
        with self._lock:
            drv = self._driver
            self._driver = None
            self._connected = False
            self._last_servoj_q = None
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
            self._latest_q_ts = time.monotonic()

    def _cb_trajectory(self, msg: JointTrajectory) -> None:
        """Jog (fase inativa): MovJ debounced para o último alvo."""
        with self._lock:
            phase = self._phase
            connected = self._connected
        if not connected or self._hold or _is_active(phase):
            return   # palpação ativa → ServoJ loop assume
        if not msg.points:
            return
        target = list(list(msg.points)[-1].positions)
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
        if drv is None or self._hold or _is_active(phase):
            return   # race guard: fase mudou durante o debounce
        try:
            q_dobot_deg = list(np.degrees(
                urdf_to_dobot(np.asarray(q_urdf, dtype=np.float64))))
            drv.mov_j_joint_deg(q_dobot_deg)
        except CR10RealDriverError as exc:
            self._fault(exc)

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
                q, q_ts = self._latest_q, self._latest_q_ts
            if not connected or drv is None or q is None:
                continue
            # Em FROZEN o braço fica onde parou: nada a comandar.
            if self._hold or phase == 'FROZEN' or not _is_active(phase):
                self._last_servoj_q = self._sync_fail_t = None
                continue
            q_new = np.asarray(q, dtype=np.float64)
            last = self._last_servoj_q
            agora = time.monotonic()
            erro = mirror_step_error(
                0.0 if last is None else float(np.max(np.abs(q_new - last))),
                max(agora - self._last_servoj_t, period), agora - q_ts)
            if erro:
                self._fault(CR10RealDriverError(erro))
                continue
            if (last is not None
                    and float(np.max(np.abs(q_new - last)))
                    < _SERVOJ_DEADBAND_RAD):
                continue   # estacionário — sem ServoJ redundante
            try:
                if last is None:
                    try:
                        self._check_sync(drv, q)
                    except CR10FeedbackTimeout:
                        # Um pacote perdido não é falha; nada saiu ainda.
                        if self._sync_fail_t is None:
                            self._sync_fail_t = agora
                        if agora - self._sync_fail_t < SYNC_FEEDBACK_GRACE_S:
                            continue
                        raise
                self._sync_fail_t = None
                drv.servo_j_urdf(q)
                self._last_servoj_q, self._last_servoj_t = q_new, agora
            except CR10RealDriverError as exc:
                self._fault(exc)

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
    from .pack_lock import acquire_or_exit
    acquire_or_exit('touch_pack')
    rclpy.init(args=args)
    node = MirrorNode()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor(num_threads=4))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
