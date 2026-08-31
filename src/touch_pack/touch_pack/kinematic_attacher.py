"""
kinematic_attacher.py — "pegar objetos" no Gazebo Classic sem física de contato.

Gazebo Classic + mão underactuada (mimic não-enforçado) não segura objeto por
atrito. Este nó cola um modelo do Gazebo a um ELO do robô enquanto "preso":
lê a pose do elo em /gazebo/link_states e reescreve a pose do objeto em
`T_elo · offset` a `rate_hz`, com velocidade zerada. É a rota padrão de
"grasp em sim" (equivale ao gazebo_ros_link_attacher / attached collision
object do MoveIt).

Serviços:
  /kinematic_attach/attach   std_srvs/Trigger  — captura o offset atual e cola
  /kinematic_attach/detach   std_srvs/Trigger  — solta (objeto cai por gravidade)
Tópico:
  /kinematic_attach/attached std_msgs/Bool

Parâmetros:
  object_name  'pick_object'      modelo do Gazebo a colar
  hand_link    'hand_base_link'   sufixo do elo (casa por `::<hand_link>`)
  rate_hz      100.0

Sem /gazebo/set_entity_state (mundo sem libgazebo_ros_state.so) o attach
recusa com mensagem — nada trava.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

try:
    from gazebo_msgs.msg import ModelStates, LinkStates
    from gazebo_msgs.srv import SetEntityState
except ImportError:                       # gazebo_msgs ausente
    ModelStates = LinkStates = SetEntityState = None


# ── quaternion (x, y, z, w) — sem scipy ─────────────────────────────
def _q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz], dtype=float)


def _q_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def _q_rot(q, v):
    qv = np.array([v[0], v[1], v[2], 0.0], dtype=float)
    return _q_mul(_q_mul(q, qv), _q_conj(q))[:3]


class KinematicAttacher(Node):
    def __init__(self) -> None:
        super().__init__('kinematic_attacher')
        self._obj = str(self.declare_parameter('object_name', 'pick_object').value)
        self._link = str(self.declare_parameter('hand_link', 'hand_base_link').value)
        rate = float(self.declare_parameter('rate_hz', 100.0).value)

        self._hand_pose: tuple | None = None
        self._obj_pose: tuple | None = None
        self._active = False
        self._p_off = np.zeros(3)
        self._q_off = np.array([0.0, 0.0, 0.0, 1.0])
        self._cli = None

        self._pub = self.create_publisher(Bool, '/kinematic_attach/attached', 10)
        self.create_service(Trigger, '/kinematic_attach/attach', self._cb_attach)
        self.create_service(Trigger, '/kinematic_attach/detach', self._cb_detach)

        if LinkStates is None:
            self.get_logger().error('gazebo_msgs indisponível — nó inerte.')
            return
        self.create_subscription(LinkStates, '/gazebo/link_states',
                                 self._cb_links, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states',
                                 self._cb_models, 10)
        self._cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        if not self._cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                '/gazebo/set_entity_state ausente — attach ficará indisponível '
                '(mundo sem libgazebo_ros_state.so).')
            self._cli = None
        self.create_timer(1.0 / max(1.0, rate), self._tick)
        self.get_logger().info(
            f'kinematic_attacher: {self._obj} ↔ ::{self._link} '
            f'({"pronto" if self._cli else "sem set_entity_state"}).')

    def _cb_links(self, msg) -> None:
        suf = '::' + self._link
        for name, pose in zip(msg.name, msg.pose):
            if name.endswith(suf):
                self._hand_pose = self._as_pose(pose)
                return

    def _cb_models(self, msg) -> None:
        try:
            i = list(msg.name).index(self._obj)
        except ValueError:
            self._obj_pose = None
            return
        self._obj_pose = self._as_pose(msg.pose[i])

    @staticmethod
    def _as_pose(p):
        return (np.array([p.position.x, p.position.y, p.position.z], float),
                np.array([p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w], float))

    def _cb_attach(self, _req, resp):
        if self._cli is None:
            resp.success = False
            resp.message = 'set_entity_state indisponível neste mundo.'
            return resp
        if self._hand_pose is None or self._obj_pose is None:
            resp.success = False
            resp.message = (f'sem pose de ::{self._link} ou de {self._obj} '
                            '(/gazebo/*_states).')
            return resp
        p_h, q_h = self._hand_pose
        p_o, q_o = self._obj_pose
        qh_inv = _q_conj(q_h)
        self._p_off = _q_rot(qh_inv, p_o - p_h)
        self._q_off = _q_mul(qh_inv, q_o)
        self._active = True
        resp.success = True
        resp.message = f'{self._obj} colado a ::{self._link}.'
        self.get_logger().info(resp.message)
        return resp

    def _cb_detach(self, _req, resp):
        self._active = False
        resp.success = True
        resp.message = f'{self._obj} solto.'
        self.get_logger().info(resp.message)
        return resp

    def _tick(self) -> None:
        self._pub.publish(Bool(data=self._active))
        if not self._active or self._cli is None or self._hand_pose is None:
            return
        p_h, q_h = self._hand_pose
        p_o = p_h + _q_rot(q_h, self._p_off)
        q_o = _q_mul(q_h, self._q_off)
        req = SetEntityState.Request()
        req.state.name = self._obj
        req.state.reference_frame = 'world'
        req.state.pose.position.x = float(p_o[0])
        req.state.pose.position.y = float(p_o[1])
        req.state.pose.position.z = float(p_o[2])
        req.state.pose.orientation.x = float(q_o[0])
        req.state.pose.orientation.y = float(q_o[1])
        req.state.pose.orientation.z = float(q_o[2])
        req.state.pose.orientation.w = float(q_o[3])
        self._cli.call_async(req)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KinematicAttacher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
