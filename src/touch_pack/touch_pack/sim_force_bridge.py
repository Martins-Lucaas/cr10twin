"""
sim_force_bridge.py — fecha a malha de força NA SIMULAÇÃO.

O `force_receiver` real lê a célula pela USB; em Gazebo não há porta serial,
então sem este nó `/load_cell/force_net` fica mudo e o explorer recusa todo
ensaio por "leitura velha".

Este nó converte o wrench do plugin `libgazebo_ros_ft_sensor.so` (montado na
junta `load_cell_attach` do touch_tool_tcp.urdf) na mesma grandeza escalar
que o receiver real publica:

    /sim/load_cell/wrench  (geometry_msgs/WrenchStamped)
        └─ force.z  →  sign · force.z − offset  →  /load_cell/force_net (Float32)

Parâmetros (todos com default sensato):
    sign        -1.0   converte o eixo do plugin p/ "compressão positiva".
                       O sinal certo depende da convenção de frame do plugin
                       na SUA build — se o ensaio empurrar e a força der
                       negativa, troque para +1.0.
    offset       0.0   N a subtrair (tara). O peso da pilha abaixo da célula
                       ≈ 5,5 N com a ferramenta na vertical; deixe 0.0 e use
                       a compensação de gravidade do explorer, ou meça e fixe.
    filter_hz    0.0   1 polo passa-baixa (Hz) p/ imitar a banda da HX711.
                       0 = sem filtro (o plugin já traz ruído gaussiano).
    rate_hz     80.0   taxa de republicação.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import WrenchStamped

from .lc_filter import QOS_SENSOR   # BEST_EFFORT/KEEP_LAST(1), mesmo do receiver real


class SimForceBridge(Node):
    def __init__(self) -> None:
        super().__init__('sim_force_bridge')
        self._sign = float(self.declare_parameter('sign', -1.0).value)
        self._offset = float(self.declare_parameter('offset', 0.0).value)
        self._filter_hz = float(self.declare_parameter('filter_hz', 0.0).value)
        rate_hz = float(self.declare_parameter('rate_hz', 80.0).value)

        self._raw: float | None = None
        self._filt: float | None = None
        self._t_prev = self.get_clock().now()

        self._pub = self.create_publisher(
            Float32, '/load_cell/force_net', QOS_SENSOR)
        self.create_subscription(
            WrenchStamped, '/sim/load_cell/wrench', self._on_wrench, QOS_SENSOR)
        self.create_timer(1.0 / max(1.0, rate_hz), self._tick)

        self.get_logger().info(
            f'sim_force_bridge: /sim/load_cell/wrench.z → /load_cell/force_net '
            f'(sign={self._sign:+.0f}, offset={self._offset:.2f} N, '
            f'filter={"off" if self._filter_hz <= 0 else f"{self._filter_hz:.0f} Hz"})')

    def _on_wrench(self, msg: WrenchStamped) -> None:
        self._raw = self._sign * float(msg.wrench.force.z) - self._offset

    def _tick(self) -> None:
        if self._raw is None:
            return
        now = self.get_clock().now()
        if self._filter_hz > 0.0:
            dt = max(1e-4, (now - self._t_prev).nanoseconds * 1e-9)
            a = 1.0 - math.exp(-2.0 * math.pi * self._filter_hz * dt)
            self._filt = (self._raw if self._filt is None
                          else self._filt + a * (self._raw - self._filt))
            out = self._filt
        else:
            out = self._raw
        self._t_prev = now
        self._pub.publish(Float32(data=float(out)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimForceBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
