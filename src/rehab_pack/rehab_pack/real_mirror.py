"""Nó `rehab_mirror`: espelha o braço simulado no CR10 real (como o mirror_node
do touch_pack), mas ligado ao estado da admitância.

  • /joint_states (sim) → ServoJ a 33 Hz, SÓ com a admitância ATIVA, o robô
    conectado, sem E-Stop e sim ≈ real (ver real_link.real_ok);
  • ao entrar em ABORTADO: Stop()+Continue() no robô (para e limpa a fila); Stop não
    confirmado vira E-STOP e aparece na tela;
  • conecta/desconecta e troca SIM_ONLY/MIRROR em serviço (S_REAL_CONNECT, S_REAL_MODE);
  • em MIRROR e PARADO, leva o braço simulado à pose do real (como o touch_pack);
  • publica /rehab/real/status (JSON, 33 Hz) — o nó de admitância o usa para
    recusar o enable e para abortar se a condição se perder;
  • sem hardware: robot_ip:=sim usa um robô real simulado.

Parâmetros: robot_ip ('' → ~/.config/touch_pack/robot.json → 192.168.5.2),
servoj_period_s (0,030).
"""
from __future__ import annotations

import json
import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from .constants import (ARM_JOINTS, FEEDBACK_TIMEOUT_MAX, JS_STALE_S, RAMP_S, REAL_MODES,
                        ROBOT_MODES_OK, S_REAL_CONNECT, S_REAL_MODE, SERVOJ_PERIOD_RANGE_S,
                        STATES, SYNC_TOL_START_RAD, T_ARM_CMD, T_EVENT,
                        T_JOINT_STATES, T_REAL_STATUS, T_STATE, T_TICK,
                        TICK_STALE_S)
from .real_link import (RealLink, StepTooLarge, arm_intruders, is_feedback_timeout,
                        robot_ip, sync_error_rad)

ACTIVE = STATES[1]      # ATIVO
ABORTED = STATES[4]     # ABORTADO
JOG = STATES[5]         # POSICIONANDO
STREAMING = (ACTIVE, JOG)
PARKED = (None, STATES[0], ABORTED)   # estados em que se troca modo/conexão
STATUS_HZ = 33.0        # a admitância usa q_real para medir a velocidade
ZERO_BLIP_RAD, ZERO_BLIP_MAX = 0.05, 10     # até ~0,3 s seguidos
SYNC_MOVE_S = 3
TOUCH_NODES = {'mirror_node', 'tactile_explorer'}   # levariam o sim (ou o real) junto
Q_SIM_STALE_S = 0.5
Q_REAL_STALE_S = 0.2     # pose real mais velha que isso não move o robô
RUN_TOL_HOLD_S = 0.5     # parou de mandar ServoJ: o real ainda está chegando no sim


class RealMirror(Node):
    def __init__(self):
        super().__init__('rehab_mirror')
        self.declare_parameter('robot_ip', '')
        self.declare_parameter('servoj_period_s', 0.030, ParameterDescriptor(read_only=True))
        self.declare_parameter('control_mode', 'mirror')
        self.mirror = str(self.get_parameter('control_mode').value) == 'mirror'
        self.want = False            # energizar o CR10 é sempre pedido do operador (Conectar)
        self.connecting = False
        self._sync_t = 0.0
        self.tick_t = 0.0            # último tick da admitância (batimento)
        self.q_real, self.q_real_t = None, 0.0
        self.ramp = None             # (diferença real−sim, início) ao armar
        self.halted = ''             # motivo da trava (PARAR, tranco), até o operador dar Reset
        self.parando = None          # thread do Stop/E-STOP em curso: nada de ServoJ
        self._was_connected = False
        self.period = float(self.get_parameter('servoj_period_s').value)
        lo, hi = SERVOJ_PERIOD_RANGE_S
        if not lo <= self.period <= hi:          # NaN também cai aqui
            raise ValueError(f'servoj_period_s={self.period} fora de [{lo}; {hi}]')
        ip = robot_ip(str(self.get_parameter('robot_ip').value))
        self.link = RealLink(ip, self.period, log=self.get_logger().info)
        self.lock = threading.RLock()
        self.stop_ev = threading.Event()
        self.wake = threading.Event()

        self.state = None
        self.q_sim = None
        self.q_sim_t = 0.0
        self.sync_ok = False
        self.max_dq = 0.0
        self.streaming, self.stream_t = False, 0.0
        self._was_aborted = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(String, T_REAL_STATUS, 5)
        self.pub_arm = self.create_publisher(JointTrajectory, T_ARM_CMD, 10)
        self.create_subscription(Float64MultiArray, T_TICK, self._on_tick, 10)
        self.create_subscription(String, T_EVENT, self._on_event, 10)
        self.create_service(SetBool, S_REAL_CONNECT, self._srv_connect)
        self.create_service(SetBool, S_REAL_MODE, self._srv_mode)
        self.create_subscription(JointState, T_JOINT_STATES, self._on_js, 20)
        self.create_subscription(String, T_STATE, self._on_state, latched)
        self.threads = [threading.Thread(target=target, daemon=True,
                                         name=f'rehab-mirror-{name}')
                        for target, name in ((self._connect_loop, 'conectar'),
                                             (self._servoj_loop, 'servoj'),
                                             (self._status_loop, 'status'))]
        for t in self.threads:
            t.start()
        self.get_logger().info(f'rehab_mirror ativo — CR10 em {ip}; o ServoJ '
                               'só corre com a admitância ATIVA e sim ≈ real.')

    # ── entradas ────────────────────────────────────────────────────────
    def _on_js(self, msg: JointState):
        try:
            q = np.array([msg.position[msg.name.index(j)] for j in ARM_JOINTS])
        except (ValueError, IndexError):     # junta ausente ou posição faltando
            return
        if not np.all(np.isfinite(q)):
            return
        with self.lock:
            self.q_sim, self.q_sim_t = q, time.monotonic()

    def _on_state(self, msg: String):
        with self.lock:
            self.state = msg.data

    def _on_tick(self, _msg):
        self.tick_t = time.monotonic()

    def _on_event(self, msg: String):
        # segundo caminho do PARAR: não depende do nó de admitância
        if msg.data.startswith('ESTOP'):
            with self.lock:
                self._travar('PARAR do painel')
        elif msg.data == 'RESET':    # só o Reset: PARADO sozinho é republicado a toda hora
            with self.lock:
                self.halted = ''

    def _travar(self, why):
        self.halted = f'ServoJ travado ({why}): dê Reset para liberar'
        self.streaming = False
        self._parar()

    def _parar(self):
        """Com o lock: daqui em diante nenhum ServoJ. A espera pela rede (segundos, se ela
        emudecer) fica numa thread, sem segurar o status nem o PARAR."""
        link = self.link
        if self.parando is not None or not link.connected:
            return

        def para():
            try:
                link.halt()
            finally:
                with self.lock:
                    self.parando = None
        self.parando = threading.Thread(target=para, daemon=True, name='rehab-mirror-parar')
        self.parando.start()

    # ── serviços ────────────────────────────────────────────────────────
    def _srv_connect(self, req, res):
        with self.lock:
            if not req.data:             # desconectar vale sempre: para e desabilita
                self.want = False
                if self.parando is not None:       # depois do Stop em curso, sem prender o nó
                    threading.Thread(target=self._desconecta_depois, daemon=True,
                                     args=(self.parando, self.link)).start()
                    res.success, res.message = True, 'desconectando: parada em curso'
                else:
                    self.link.disconnect()
                    res.success, res.message = True, 'desconectado'
            elif self.state not in PARKED:
                res.success, res.message = False, f'só em PARADO (agora {self.state})'
            elif self.link.connected or self.connecting:
                res.success, res.message = False, 'já conectado ou conectando'
            else:
                ip = robot_ip(str(self.get_parameter('robot_ip').value))
                velho, self.link = self.link, RealLink(ip, self.period,
                                                       log=self.get_logger().info)
                self.link.parada = velho.parada      # vale até confirmar a conexão
                self.link.rearmar = velho.rearmar and velho.ip == ip
                self.want = True
                self.wake.set()
                res.success, res.message = True, f'conectando em {ip}'
        return res

    def _desconecta_depois(self, parada, link):
        parada.join()
        link.disconnect()

    def _srv_mode(self, req, res):
        with self.lock:
            if self.state not in PARKED:
                res.success, res.message = False, f'só em PARADO (agora {self.state})'
            else:
                self.mirror = bool(req.data)
                res.success, res.message = True, REAL_MODES[self.mirror]
        return res

    def _sync_sim(self, q_real):
        """Leva o SIMULADO à pose do real."""
        if (time.monotonic() - self._sync_t < SYNC_MOVE_S + 1.0
                or self.pub_arm.get_subscription_count() == 0):
            return
        self._sync_t = time.monotonic()
        outros = arm_intruders(self) or sorted(TOUCH_NODES & set(self.get_node_names()))
        if outros:
            self.get_logger().warning(f'sincronia recusada: outro nó comanda o braço ({outros})')
            return
        pt = JointTrajectoryPoint(positions=[float(v) for v in q_real],
                                  velocities=[0.0] * 6,
                                  time_from_start=Duration(sec=SYNC_MOVE_S))
        self.pub_arm.publish(JointTrajectory(joint_names=list(ARM_JOINTS),
                                             points=[pt]))

    # ── threads ─────────────────────────────────────────────────────────
    def _connect_loop(self):
        while not self.stop_ev.is_set():
            with self.lock:
                link = self.link
                if self._was_connected and not link.connected and link.last_error:
                    self.want = False        # caiu: só o operador religa o robô
                self._was_connected = link.connected
                todo = self.want and not link.connected
                self.connecting = todo
            if not todo:
                self.wake.wait(1.0)
                self.wake.clear()
                continue
            drv, err = link.open_driver()      # até ~16 s: fora do lock
            ok = False
            with self.lock:
                self.connecting = False
                if drv is None:
                    if self.want:
                        link.fail(err)
                        # uma tentativa por pedido: repetir o enable() energizaria
                        # o braço sozinho quando alguém soltasse o E-Stop
                        self.want = False
                elif self.want and not self.stop_ev.is_set():
                    link.attach(drv)
                    self._was_connected = True
                    ok = True
            if drv is not None and not ok:     # desistiram durante a espera
                for fn in (drv.stop, drv.close):
                    try:
                        fn()
                    except Exception as exc:
                        self.get_logger().warning(f'conexão cancelada: {exc}')

    def _servoj_loop(self):
        t_next = time.monotonic() + self.period
        while not self.stop_ev.is_set():
            self.stop_ev.wait(max(0.0, t_next - time.monotonic()))
            with self.lock:
                # do ponto que sai agora: recuperar atraso juntaria dois pontos num período
                t_next = time.monotonic() + self.period
                state, q = self.state, self.q_sim
                now = time.monotonic()
                if state == ABORTED:
                    if not self._was_aborted:        # só na transição
                        self._parar()
                    self._was_aborted = True
                else:
                    self._was_aborted = False
                go = (self.mirror and state in STREAMING and q is not None
                      and now - self.q_sim_t < JS_STALE_S
                      and self.link.connected and self.sync_ok
                      and self.q_real is not None and now - self.q_real_t < Q_REAL_STALE_S
                      # armar ou retomar: só perto do real; depois quem limita é o passo
                      and (self.ramp is not None
                           or sync_error_rad(q, self.q_real) <= SYNC_TOL_START_RAD)
                      and not self.link.estop and self._mode_ok()
                      and not self.halted and self.parando is None and not self.link.parada
                      and now - self.tick_t < TICK_STALE_S)
                self.streaming = go
                if go:
                    self.stream_t = now
                    try:
                        self.link.stream(self._ramped(q), self.q_real)
                    except StepTooLarge as exc:
                        self.get_logger().error(f'ServoJ travado: {exc}')
                        self._travar(str(exc))
                        go = False
                if not go:
                    self.ramp = None
                    self.link.reset_stream()

    def _ramped(self, q):
        """Parte da pose REAL e some com a diferença em RAMP_S: sem salto ao armar."""
        now = time.monotonic()
        if self.ramp is None:
            self.ramp = (self.q_real - q, now)
        s = min((now - self.ramp[1]) / RAMP_S, 1.0)
        return q + self.ramp[0] * (1.0 - s * s * (3.0 - 2.0 * s))

    def _mode_ok(self):
        m = self.link.robot_mode
        return m is None or m in ROBOT_MODES_OK

    def _status_loop(self):
        blips = timeouts = 0
        while not self.stop_ev.is_set():
            with self.lock:
                drv = self.link.drv
            q_real, err = None, ''
            if drv is not None:
                try:                           # fora do lock: não atrasa o ServoJ
                    q_real = drv.read_joints_urdf_latest()
                except Exception as exc:
                    err = exc
            # soluço de rede: sem pose (logo sem ServoJ e sem admitância), conexão mantida
            timeouts = timeouts + 1 if is_feedback_timeout(err) else 0
            if 0 < timeouts < FEEDBACK_TIMEOUT_MAX:
                err = ''
            if q_real is not None and not np.all(np.isfinite(q_real)):
                q_real = None
            lido = q_real is not None
            # o firmware manda juntas zeradas em transições (touch_pack: 'zero-blip')
            if q_real is not None and np.linalg.norm(q_real) < ZERO_BLIP_RAD:
                blips += 1
                q_real = self.q_real if blips <= ZERO_BLIP_MAX else None
                lido = False
            else:
                blips = 0
            with self.lock:
                if err and self.parando is None:     # o E-STOP ainda precisa da conexão
                    self.link.drop(err, only=drv)
                if drv is not self.link.drv:
                    q_real = None
                self.q_real = q_real
                if lido:
                    self.q_real_t = time.monotonic()
                q_sim = self.q_sim
                fresh = time.monotonic() - self.q_sim_t < Q_SIM_STALE_S
                if q_real is not None and q_sim is not None and fresh:
                    self.max_dq = sync_error_rad(q_sim, q_real)
                    self.sync_ok = self.max_dq <= self.link.tolerance(
                        self.streaming or time.monotonic() - self.stream_t < RUN_TOL_HOLD_S)
                    if (self.mirror and not self.sync_ok
                            and self.state == STATES[0]):
                        self._sync_sim(q_real)
                else:
                    self.sync_ok = False
                st = {'conectado': self.link.connected, 'ip': self.link.ip,
                      'modo': REAL_MODES[self.mirror],
                      'conectando': self.want and not self.link.connected,
                      'erro': self.link.last_error,
                      'robot_mode': self.link.robot_mode,
                      'estop': self.link.estop, 'sync_ok': self.sync_ok,
                      'max_dq_deg': float(np.degrees(self.max_dq)),
                      'travado': self.halted or (
                          'parada do robô em curso' if self.parando is not None else ''),
                      'parada': self.link.parada,
                      'armado': bool(self.streaming),
                      'q_real': ([float(v) for v in q_real]
                                 if q_real is not None else None)}
            self.pub.publish(String(data=json.dumps(st)))
            self.stop_ev.wait(1.0 / STATUS_HZ)

    def destroy_node(self):
        self.stop_ev.set()
        self.wake.set()
        with self.lock:
            if self.streaming:           # encerrar em movimento para o robô antes de soltá-lo
                self._parar()
            parada = self.parando
        if parada is not None:
            parada.join()
        with self.lock:
            self.link.close()
        for t in self.threads:           # senão a de status publica num nó já destruído
            t.join(1.0)
        super().destroy_node()


def main():
    from touch_pack.pack_lock import acquire_or_exit   # sem o touch_pack: recusa
    acquire_or_exit('rehab_pack')
    rclpy.init()
    node = RealMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():               # no Ctrl-C o contexto já caiu: sai sem traceback
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
