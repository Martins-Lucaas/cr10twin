"""Nó `rehab_mirror`: espelha o braço simulado no CR10 real (como o mirror_node
do touch_pack), mas ligado ao estado da admitância.

  • /joint_states (sim) → ServoJ a 33 Hz, SÓ com a admitância ATIVA, o robô
    conectado, sem E-Stop e sim ≈ real (ver real_link.real_ok);
  • ao entrar em ABORTADO: Stop()+Continue() no robô (para e limpa a fila);
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
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from .constants import (ARM_JOINTS, RAMP_S, REAL_MODES, ROBOT_MODES_OK, S_REAL_CONNECT,
                        S_REAL_MODE, SIM_ROBOT_IP, STATES, T_ARM_CMD, T_EVENT,
                        T_JOINT_STATES, T_REAL_STATUS, T_STATE, T_TICK,
                        TICK_STALE_S)
from .real_link import RealLink, robot_ip, save_robot_ip, sync_error_rad

ACTIVE = STATES[1]      # ATIVO
ABORTED = STATES[4]     # ABORTADO
JOG = STATES[5]         # POSICIONANDO
STREAMING = (ACTIVE, JOG)
PARKED = (None, STATES[0], ABORTED)   # estados em que se troca modo/conexão
STATUS_HZ = 33.0        # a admitância usa q_real para medir a velocidade
ZERO_BLIP_RAD, ZERO_BLIP_MAX = 0.05, 10     # até ~0,3 s seguidos
SYNC_MOVE_S = 3
Q_SIM_STALE_S = 0.5


class RealMirror(Node):
    def __init__(self):
        super().__init__('rehab_mirror')
        self.declare_parameter('robot_ip', '')
        self.declare_parameter('servoj_period_s', 0.030)
        self.declare_parameter('control_mode', 'mirror')
        self.mirror = self.want = (
            str(self.get_parameter('control_mode').value) == 'mirror')
        self.connecting = False
        self._sync_t = 0.0
        self.tick_t = 0.0            # último tick da admitância (batimento)
        self.q_real = None
        self.ramp = None             # (diferença real−sim, início) ao armar
        self.halted = False          # PARAR direto da GUI, até voltar a PARADO
        self._was_connected = False
        self.period = float(self.get_parameter('servoj_period_s').value)
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
        self.streaming = False
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
        for target, name in ((self._connect_loop, 'conectar'),
                             (self._servoj_loop, 'servoj'),
                             (self._status_loop, 'status')):
            threading.Thread(target=target, daemon=True,
                             name=f'rehab-mirror-{name}').start()
        self.get_logger().info(f'rehab_mirror ativo — CR10 em {ip}; o ServoJ '
                               'só corre com a admitância ATIVA e sim ≈ real.')

    # ── entradas ────────────────────────────────────────────────────────
    def _on_js(self, msg: JointState):
        try:
            q = np.array([msg.position[msg.name.index(j)] for j in ARM_JOINTS])
        except ValueError:
            return
        with self.lock:
            self.q_sim, self.q_sim_t = q, time.monotonic()

    def _on_state(self, msg: String):
        with self.lock:
            self.state = msg.data
            if msg.data == STATES[0]:
                self.halted = False

    def _on_tick(self, _msg):
        self.tick_t = time.monotonic()

    def _on_event(self, msg: String):
        # segundo caminho do PARAR: não depende do nó de admitância
        if msg.data.startswith('ESTOP'):
            with self.lock:
                self.halted = True
                self.link.halt()

    # ── serviços ────────────────────────────────────────────────────────
    def _srv_connect(self, req, res):
        with self.lock:
            if not req.data:             # desconectar vale sempre: para e desabilita
                self.want = False
                self.link.disconnect()
                res.success, res.message = True, 'desconectado'
            elif self.state not in PARKED:
                res.success, res.message = False, f'só em PARADO (agora {self.state})'
            elif self.link.connected or self.connecting:
                res.success, res.message = False, 'já conectado ou conectando'
            else:
                ip = robot_ip(str(self.get_parameter('robot_ip').value))
                self.link = RealLink(ip, self.period, log=self.get_logger().info)
                self.want = True
                self.wake.set()
                res.success, res.message = True, f'conectando em {ip}'
        return res

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
            wait, ok = 0.0, False
            with self.lock:
                self.connecting = False
                if drv is None:
                    if self.want:
                        link.fail(err)
                        wait = link.backoff_s()
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
            if ok and link.ip != SIM_ROBOT_IP:
                try:
                    from touch_pack.constants import ROBOT_CONFIG_FILE
                    save_robot_ip(link.ip, ROBOT_CONFIG_FILE)
                except Exception as exc:
                    self.get_logger().warning(f'IP não salvo: {exc}')
            if wait:
                self.wake.wait(wait)
                self.wake.clear()

    def _servoj_loop(self):
        t_next = time.monotonic() + self.period
        while not self.stop_ev.is_set():
            self.stop_ev.wait(max(0.0, t_next - time.monotonic()))
            t_next += self.period
            if t_next < time.monotonic():
                t_next = time.monotonic() + self.period
            with self.lock:
                state, q = self.state, self.q_sim
                fresh = time.monotonic() - self.q_sim_t < Q_SIM_STALE_S
                if state == ABORTED:
                    if not self._was_aborted:        # só na transição
                        self.link.halt()
                    self._was_aborted = True
                else:
                    self._was_aborted = False
                go = (self.mirror and state in STREAMING and q is not None and fresh
                      and self.link.connected and self.sync_ok
                      and self.q_real is not None
                      and not self.link.estop and self._mode_ok()
                      and not self.halted
                      and time.monotonic() - self.tick_t < TICK_STALE_S)
                self.streaming = go
                if go:
                    self.link.stream(self._ramped(q))
                else:
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
        blips = 0
        while not self.stop_ev.is_set():
            with self.lock:
                drv = self.link.drv
            q_real, err = None, ''
            if drv is not None:
                try:                           # fora do lock: não atrasa o ServoJ
                    q_real = drv.read_joints_urdf_latest()
                except Exception as exc:
                    err = exc
            # o firmware manda juntas zeradas em transições (touch_pack: 'zero-blip')
            if q_real is not None and np.linalg.norm(q_real) < ZERO_BLIP_RAD:
                blips += 1
                q_real = self.q_real if blips <= ZERO_BLIP_MAX else None
            else:
                blips = 0
            with self.lock:
                if err:
                    self.link.drop(err, only=drv)
                if drv is not self.link.drv:
                    q_real = None
                self.q_real = q_real
                q_sim = self.q_sim
                fresh = time.monotonic() - self.q_sim_t < Q_SIM_STALE_S
                if q_real is not None and q_sim is not None and fresh:
                    self.max_dq = sync_error_rad(q_sim, q_real)
                    self.sync_ok = self.max_dq <= self.link.tolerance(
                        self.streaming)
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
                      'armado': bool(self.streaming),
                      'q_real': ([float(v) for v in q_real]
                                 if q_real is not None else None)}
            self.pub.publish(String(data=json.dumps(st)))
            self.stop_ev.wait(1.0 / STATUS_HZ)

    def destroy_node(self):
        self.stop_ev.set()
        self.wake.set()
        with self.lock:
            self.link.close()
        super().destroy_node()


def main():
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
