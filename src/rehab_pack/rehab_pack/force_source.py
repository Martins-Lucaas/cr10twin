"""Fonte de força: escolhe virtual | sim | real, tara e reestampa.

Publica /rehab/ft/wrench a cada mensagem recebida, com o relógio do nó.
O sinal físico é "força da mão sobre o mastro" (compressão positiva).
"""
from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from .constants import (S_TARE, T_FT, T_REAL_WRENCH, T_SIM_WRENCH,
                        T_VIRTUAL_WRENCH)

TARE_WINDOW_S = 0.5
# fonte -> (tópico de origem, frame de saída)
SOURCES = {'virtual': (T_VIRTUAL_WRENCH, 'world'),
           'sim': (T_SIM_WRENCH, 'load_cell_link'),
           'real': (T_REAL_WRENCH, 'load_cell_link')}


# BEST_EFFORT assina publicadores confiáveis E best-effort. O ft_receiver do
# touch_pack publica a FA7155 em best-effort (QOS_SENSOR): um assinante confiável
# nunca receberia nada dele — só um warning de QoS, sem erro.
WRENCH_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)


class Tare:
    """Offset de 6 eixos: média da janela de tara, subtraída das amostras."""

    def __init__(self):
        self.offset = np.zeros(6)
        self._acc = None

    def begin(self):
        self._acc = []

    def feed(self, w):
        if self._acc is not None:
            self._acc.append(np.asarray(w, float))

    def finish(self) -> bool:
        """Fecha a janela. False (e offset intacto) se não houve amostras."""
        acc, self._acc = self._acc, None
        if not acc:
            return False
        self.offset = np.mean(acc, axis=0)
        return True

    def apply(self, w) -> np.ndarray:
        return np.asarray(w, float) - self.offset


class ForceSource(Node):
    def __init__(self):
        super().__init__('rehab_force_source')
        ro = ParameterDescriptor(read_only=True)
        self.declare_parameter('source', 'virtual', ro)
        self.declare_parameter('sim_sign', 1.0)
        self.declare_parameter('real_sign', 1.0)
        src = self.get_parameter('source').value
        if src not in SOURCES:
            raise ValueError(f'source={src!r}: use virtual, sim ou real')
        topic, self._frame = SOURCES[src]
        self._src = src
        self.declare_parameter('source_topic', topic, ro)
        self._tare = Tare()
        self._lock = threading.Lock()
        self._pub = self.create_publisher(WrenchStamped, T_FT, 50)
        grp = ReentrantCallbackGroup()
        self.create_subscription(WrenchStamped, topic, self._on_wrench,
                                 WRENCH_QOS, callback_group=grp)
        self.create_service(Trigger, S_TARE, self._on_tare,
                            callback_group=grp)
        self.get_logger().info(f'fonte={src} topico={topic} frame={self._frame}')

    def _sign(self) -> float:
        return {'virtual': 1.0,
                'sim': self.get_parameter('sim_sign').value,
                'real': self.get_parameter('real_sign').value}[self._src]

    def _on_wrench(self, msg: WrenchStamped):
        f, t = msg.wrench.force, msg.wrench.torque
        w = self._sign() * np.array([f.x, f.y, f.z, t.x, t.y, t.z])
        with self._lock:
            self._tare.feed(w)
            w = self._tare.apply(w)
        out = WrenchStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame
        (out.wrench.force.x, out.wrench.force.y, out.wrench.force.z,
         out.wrench.torque.x, out.wrench.torque.y,
         out.wrench.torque.z) = (float(v) for v in w)
        self._pub.publish(out)

    def _on_tare(self, _req, res):
        with self._lock:
            self._tare.begin()
        # espera em tempo de relógio do nó (sim ou parede, conforme use_sim_time)
        t0 = self.get_clock().now()
        wall0 = time.monotonic()
        while ((self.get_clock().now() - t0).nanoseconds * 1e-9 < TARE_WINDOW_S
               and time.monotonic() - wall0 < 10 * TARE_WINDOW_S):
            time.sleep(0.02)
        with self._lock:
            ok = self._tare.finish()
        res.success = ok
        res.message = ('tara feita' if ok else
                       'sem amostras de força na janela de 0,5 s')
        return res


def main():
    rclpy.init()
    node = ForceSource()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
