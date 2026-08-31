"""
grasp_sense.py — sinais de preensão SENTIDOS a partir dos streams da COVVI.

Somente LEITURA — não comanda a mão. Consome os streams ECI que hoje só
aparecem na GUI e publica os derivados que o `grasp_executor` precisa para
confirmar posse e detectar escorregamento:

  sub  {eci_prefix}/DigitTouchAllMsg   (uint8 0-255 por dedo)
  sub  {eci_prefix}/DigitStatusAllMsg  (fault/touch/stall/gripping por dedo)

  pub  /grasp/holding        std_msgs/Bool   ≥ min_digits em contato
  pub  /grasp/slip           std_msgs/Bool   pulso: estava segurando e perdeu
  pub  /grasp/contact_count  std_msgs/Int32  nº de dedos em contato
  pub  /grasp/fault          std_msgs/Bool   algum dedo em fault

Sem ECI no ar (sim puro, ou driver desligado) os subs ficam vazios e o nó
publica holding=False / contact_count=0 — o executor volta ao comportamento
anterior (checagem só geométrica).

Parâmetros:
  eci_prefix     '/covvi/hand'
  touch_on       12     limiar (0-255) de "contato leve" no tátil do dedo
  min_digits     2      dedos em contato para declarar `holding`
  slip_window_s  0.4     janela p/ o pulso de slip após perder o holding
  rate_hz        20.0
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

_FINGERS = ('thumb', 'index', 'middle', 'ring', 'little')

try:
    from covvi_interfaces.msg import DigitTouchAllMsg, DigitStatusAllMsg
    _HAVE_ECI = True
except Exception:                       # covvi_interfaces não compilado
    DigitTouchAllMsg = DigitStatusAllMsg = None
    _HAVE_ECI = False


class GraspSense(Node):
    def __init__(self) -> None:
        super().__init__('grasp_sense')
        pfx = self.declare_parameter('eci_prefix', '/covvi/hand').value
        self._touch_on = int(self.declare_parameter('touch_on', 12).value)
        self._min_digits = int(self.declare_parameter('min_digits', 2).value)
        self._slip_window = float(self.declare_parameter('slip_window_s', 0.4).value)
        rate = float(self.declare_parameter('rate_hz', 20.0).value)

        self._touch: dict[str, int] = {f: 0 for f in _FINGERS}
        self._status = None
        self._holding = False
        self._t_last_hold = None          # instante do último holding=True

        self._pub_hold = self.create_publisher(Bool, '/grasp/holding', 10)
        self._pub_slip = self.create_publisher(Bool, '/grasp/slip', 10)
        self._pub_cnt = self.create_publisher(Int32, '/grasp/contact_count', 10)
        self._pub_fault = self.create_publisher(Bool, '/grasp/fault', 10)

        if _HAVE_ECI:
            self.create_subscription(
                DigitTouchAllMsg, f'{pfx}/DigitTouchAllMsg', self._on_touch, 10)
            self.create_subscription(
                DigitStatusAllMsg, f'{pfx}/DigitStatusAllMsg', self._on_status, 10)
            self.get_logger().info(
                f'grasp_sense: ouvindo {pfx}/DigitTouchAllMsg + DigitStatusAllMsg '
                f'(touch_on={self._touch_on}, min_digits={self._min_digits})')
        else:
            self.get_logger().warn(
                'grasp_sense: covvi_interfaces indisponível — publicando '
                'holding=False/contact_count=0 (o executor cai na checagem '
                'geométrica).')

        self.create_timer(1.0 / max(1.0, rate), self._tick)

    def _on_touch(self, msg) -> None:
        for f in _FINGERS:
            self._touch[f] = int(getattr(msg, f'{f}_touch', 0))

    def _on_status(self, msg) -> None:
        self._status = msg

    def _in_contact(self, f: str) -> bool:
        if self._touch[f] >= self._touch_on:
            return True
        s = self._status
        if s is not None and (getattr(s, f'{f}_touch', False)
                              or getattr(s, f'{f}_stall', False)):
            return True
        return False

    def _tick(self) -> None:
        cnt = sum(self._in_contact(f) for f in _FINGERS)
        holding = cnt >= self._min_digits
        now = self.get_clock().now()

        slip = False
        if holding:
            self._t_last_hold = now
        elif (self._holding and self._t_last_hold is not None
              and (now - self._t_last_hold).nanoseconds * 1e-9 <= self._slip_window):
            slip = True                   # tinha holding, perdeu dentro da janela
        self._holding = holding

        fault = False
        if self._status is not None:
            fault = any(getattr(self._status, f'{f}_fault', False)
                        for f in (*_FINGERS, 'rotate'))

        self._pub_hold.publish(Bool(data=holding))
        self._pub_slip.publish(Bool(data=slip))
        self._pub_cnt.publish(Int32(data=int(cnt)))
        self._pub_fault.publish(Bool(data=fault))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspSense()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
