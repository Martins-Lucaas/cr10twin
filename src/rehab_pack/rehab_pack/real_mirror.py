"""Nó `rehab_mirror`: espelha o braço simulado no CR10 real (como o mirror_node
do touch_pack), mas ligado ao estado da admitância.

  • /joint_states (sim) → ServoJ a 33 Hz, SÓ com a admitância ATIVA, o robô
    conectado, sem E-Stop e sim ≈ real (ver real_link.real_ok);
  • ao entrar em ABORTADO: Stop()+Continue() no robô (para e limpa a fila);
  • publica /rehab/real/status (JSON, 10 Hz) — o nó de admitância o usa para
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
from std_msgs.msg import String

from .constants import (ARM_JOINTS, T_JOINT_STATES, T_REAL_STATUS, T_STATE,
                        STATES)
from .real_link import RealLink, robot_ip, sync_error_rad

ACTIVE = STATES[1]      # ATIVO
ABORTED = STATES[4]     # ABORTADO
JOG = STATES[5]         # POSICIONANDO
STREAMING = (ACTIVE, JOG)
STATUS_HZ = 10.0
Q_SIM_STALE_S = 0.5


class RealMirror(Node):
    def __init__(self):
        super().__init__('rehab_mirror')
        self.declare_parameter('robot_ip', '')
        self.declare_parameter('servoj_period_s', 0.030)
        self.period = float(self.get_parameter('servoj_period_s').value)
        ip = robot_ip(str(self.get_parameter('robot_ip').value))
        self.link = RealLink(ip, self.period, log=self.get_logger().info)
        self.lock = threading.RLock()
        self.stop_ev = threading.Event()

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

    # ── threads ─────────────────────────────────────────────────────────
    def _connect_loop(self):
        while not self.stop_ev.is_set():
            with self.lock:
                connected = self.link.connected
            if connected:
                self.stop_ev.wait(1.0)
                continue
            with self.lock:
                ok = self.link.connect_once()
                wait = 0.0 if ok else self.link.backoff_s()
            self.stop_ev.wait(wait if wait else 1.0)

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
                go = (state in STREAMING and q is not None and fresh
                      and self.link.connected and self.sync_ok)
                self.streaming = go
                if go:
                    self.link.stream(q)
                else:
                    self.link.reset_stream()

    def _status_loop(self):
        while not self.stop_ev.is_set():
            with self.lock:
                q_real = self.link.read_q()
                q_sim = self.q_sim
                fresh = time.monotonic() - self.q_sim_t < Q_SIM_STALE_S
                if q_real is not None and q_sim is not None and fresh:
                    self.max_dq = sync_error_rad(q_sim, q_real)
                    self.sync_ok = self.max_dq <= self.link.tolerance(
                        self.state in STREAMING)
                else:
                    self.sync_ok = False
                st = {'conectado': self.link.connected, 'ip': self.link.ip,
                      'estop': self.link.estop, 'sync_ok': self.sync_ok,
                      'max_dq_deg': float(np.degrees(self.max_dq)),
                      'armado': bool(self.streaming),
                      'q_real': ([float(v) for v in q_real]
                                 if q_real is not None else None)}
            self.pub.publish(String(data=json.dumps(st)))
            self.stop_ev.wait(1.0 / STATUS_HZ)

    def destroy_node(self):
        self.stop_ev.set()
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
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
