"""
Executor de grasp para célula de manufatura com esteira.

Máquina de estados por ciclo (acionado via serviço /cell/execute_grasp):
  IDLE → PICK → LIFT → PLACE → HOME

Grips determinísticos (sem ML):
  frasco → palm_grip      → Box 1
  tubo   → claw_grip      → Box 2
  ampola → fingertip_grip → Box 3

Serviços expostos (não bloqueiam a GUI):
  /cell/execute_grasp  (std_srvs/Trigger) → inicia ciclo completo em thread
  /cell/go_home        (std_srvs/Trigger) → envia braço ao home

Publica:
  /cell/status   (std_msgs/String JSON) — estado e progresso do executor

Subscreve:
  /detected_objects  (vision_msgs/Detection2DArray) — classe do objeto atual
  /joint_states      (sensor_msgs/JointState)        — posição das juntas

Mão: 31 juntas (6 primárias + 25 mimic) com as razões de mimic do URDF COVVI.
Cinemática: IK analítica + refinamento numérico DLS do módulo kinematics.
"""

from __future__ import annotations

import json
import math
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from vision_msgs.msg import Detection2DArray

try:
    from gazebo_msgs.msg import ModelStates
    _GAZEBO_OK = True
except ImportError:
    _GAZEBO_OK = False
    ModelStates = None

from .kinematics import (
    inverse_kinematics,
    forward_kinematics,
    HAND_CONFIGS, HAND_LIMITS, HAND_LOWER, hand_ik,
)
from .perfect_grasp import PerfectGrasp
from .cage_check import cage_status
# Poses canônicas (SDD §4) — fonte única de verdade do approach/pick.
from .poses import (
    PRE_APPROACH_POSES_DEG as _POSES_PRE_APPROACH_DEG,
    APPROACH_POSES_DEG     as _POSES_APPROACH_DEG,
    PICK_POSES_DEG         as _POSES_PICK_DEG,
    ARM_JOINTS             as _POSES_ARM_JOINTS,
)


def _deg_dict_to_rad(deg_dict: dict) -> np.ndarray:
    """Converte {jointN: graus} → numpy.array (6,) em rad na ordem ARM_JOINTS."""
    return np.array([math.radians(deg_dict[j]) for j in _POSES_ARM_JOINTS])

# Juntas do braço CR10
_ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Juntas primárias da mão
_HAND_PRIMARY = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']

# Mapa de mimic joints: nome → (junta primária, multiplicador)
# Extraído do URDF linear_covvi_hand_gazebo.urdf
_MIMIC_MAP: dict[str, tuple[str, float]] = {
    '_lisa_j01':            ('Rotate', 1.07338),
    '_thumb_chassis_j01':   ('Rotate', 1.53340),
    '_thumb_proximal_j01':  ('Thumb',  0.72022),
    '_thumb_distal_j01':    ('Thumb',  1.06686),
    '_thumb_link_j01':      ('Thumb',  0.76799),
    '_thumb_follower_j01':  ('Thumb',  0.93733),
    '_index_proximal_j01':  ('Index',  1.51604),
    '_index_distal_j01':    ('Index',  1.33574),
    '_index_knuckle_j01':   ('Index',  1.25182),
    '_index_follower_j01':  ('Index',  0.26423),
    '_index_link_j01':      ('Index',  1.33574),
    '_middle_proximal_j01': ('Middle', 1.51604),
    '_middle_distal_j01':   ('Middle', 1.34986),
    '_middle_knuckle_j01':  ('Middle', 1.25181),
    '_middle_follower_j01': ('Middle', 0.26423),
    '_middle_link_j01':     ('Middle', 1.34986),
    '_ring_proximal_j01':   ('Ring',   1.51604),
    '_ring_distal_j01':     ('Ring',   1.34878),
    '_ring_knuckle_j01':    ('Ring',   1.25182),
    '_ring_follower_j01':   ('Ring',   0.26423),
    '_ring_link_j01':       ('Ring',   1.34878),
    '_little_proximal_j01': ('Little', 1.51604),
    '_little_distal_j01':   ('Little', 1.31664),
    '_little_knuckle_j01':  ('Little', 1.25182),
    '_little_follower_j01': ('Little', 0.26423),
    '_little_link_j01':     ('Little', 1.31664),
}

# Parâmetros de movimento Home seguro — braço erguido para trás (TCP ≈
# (−0.69, −0.19, 1.31) m robot frame). q2=0 URDF ↔ braço superior vertical;
# q3=π/2 dobra o antebraço para trás.
_HOME_Q         = np.array([0.0,  0.0,  math.pi/2,
                             -math.pi/2, -math.pi/2, 0.0])
# Seeds compactos por objeto para IK de pick (pick_xy=(0.75,0)).
# q_pick é calculado primeiro com o seed do objeto; approach/via/lift seeded
# a partir de q_pick. frasco/ampola usam ramo q4<0; tubo usa o único ramo estável.
_PICK_SEED_Q = {
    'frasco': np.array([0.411, -0.277, -1.335, -1.529,  0.411,  0.0  ]),
    'tubo':   np.array([0.411, -0.381, -1.652,  2.032, -0.411,  3.142]),
    'ampola': np.array([0.411, -0.277, -1.341, -1.524,  0.411,  0.0  ]),
}
# Seed compacto para IK de approach_box: q2/q3 negativos mantêm Link2/Link3
# acima da sort_shelf (z≤0.48m) para todos os três boxes.
_APPROACH_BOX_SEED_Q = np.array([0.0, -0.4, -1.5, -1.3, 0.0, 0.0])
# Seed para via_box (z=1.15m world, diretamente acima do box em transit
# altitude).
_VIA_BOX_SEED_Q = np.array([0.5, -0.5, -0.8, -1.5, 0.5, 0.0])
_APPROACH_CLEAR = 0.15    # m — altura de pré-abordagem (acima da parede das caixas: 0.705m)
_CLOSE_EXTRA    = 0.05    # fração extra de fechamento sobre o cfg nominal
_MAX_JOINT_VEL  = 1.40    # rad/s — agressivo para fluidez
_N_TRAJ_STEPS   = 8       # waypoints por segmento de trajetória
_N_CART_VIA     = 6       # waypoints Cartesianos para transições longas
_ARM_DUR_FLOOR  = 0.50    # s — duração mínima de uma trajetória articular
_CART_DUR_FLOOR = 0.90    # s — duração mínima de uma trajetória Cartesiana

# Approach vector padrão: de cima para baixo (esteira horizontal).
_AV_DOWN = np.array([0.0, 0.0, -1.0])

# Altura do base_link do robô no world frame do Gazebo. Spawn z=0.375; URDF
# world_joint xyz=[0,0,0.03] → base_link em z=0.405.
_ROBOT_BASE_Z = 0.405

# Altura de trânsito via_box em robot frame (= 1.15m world).
_TRANSIT_Z = 1.15 - _ROBOT_BASE_Z   # 0.745 m robot frame


# Bounding boxes — re-exportadas de `collision.py` (fonte única)
from .collision import (
    PICK_OBJ_BBOX   as _PICK_OBJ_BBOX,
    BIN_BBOX        as _BIN_BBOX,
    WORLD_OBSTACLES as _WORLD_OBSTACLES,
)

# Offset vertical entre TCP e centro do objeto preso (TCP_world − obj_center).
# Após refactor 2026-05-18 (top-down frasco):
#   frasco: TCP em z=0.921 (palm height), obj_center em z=0.851 → 0.070 m
#   tubo:   pick_z (0.866) = obj_center → 0.000 m (lateral)
#   ampola: pick_z (0.881) = obj_top    → +0.038 m (centro 38 mm abaixo do TCP)
_HELD_OFFSET_Z: dict[str, float] = {
    'frasco': 0.070,
    'tubo':   0.000,
    'ampola': 0.038,
}


def _w2r(pos: np.ndarray) -> np.ndarray:
    """World frame → robot base frame (subtrai altura da base no mundo)."""
    return np.array([pos[0], pos[1], pos[2] - _ROBOT_BASE_Z])

# Mapeamento classe → (grip_type, box_key, obj_diameter_m)
# frasco (frasco de medicamento) → palm_grip  → Box 1  (recipiente volumoso)
# tubo   (tubo de ensaio)        → claw_grip  → Box 2  (cilindro médio)
# ampola (ampola farmacêutica)   → fingertip_grip → Box 3 (objeto delicado fino)
_OBJECT_MAP: dict[str, tuple[str, str, float]] = {
    # terceiro elemento: "diâmetro efetivo de preensão" (não o diâmetro externo do objeto,
    # mas a zona de contato dos dedos — palm_grip vem de cima, não envolve o perímetro total)
    'frasco': ('palm_grip',      'box1', 0.060),
    'tubo':   ('claw_grip',      'box2', 0.024),
    'ampola': ('fingertip_grip', 'box3', 0.010),
}


# Helpers de trajetória

def _make_smooth_arm_goal(q_start: np.ndarray,
                          q_end: np.ndarray) -> tuple[FollowJointTrajectory.Goal, float]:
    """Trajetória multi-ponto com ease-in/out sinusoidal."""
    max_delta = float(np.max(np.abs(q_end - q_start)))
    duration  = max(max_delta / _MAX_JOINT_VEL, _ARM_DUR_FLOOR)

    traj = JointTrajectory()
    traj.joint_names = _ARM_JOINTS

    for i in range(1, _N_TRAJ_STEPS + 1):
        alpha  = i / _N_TRAJ_STEPS
        smooth = 0.5 * (1.0 - math.cos(math.pi * alpha))
        q_i    = q_start + smooth * (q_end - q_start)
        t      = duration * alpha
        pt     = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q_i]
        if i == _N_TRAJ_STEPS:
            pt.velocities    = [0.0] * 6
            pt.accelerations = [0.0] * 6
        sec = int(t)
        pt.time_from_start = Duration(sec=sec, nanosec=int((t - sec) * 1e9))
        traj.points.append(pt)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    return goal, duration


def _make_hand_goal(cfg: dict[str, float],
                    duration: float) -> FollowJointTrajectory.Goal:
    """Goal para as 31 juntas da mão (6 primárias + 25 mimic)."""
    all_names: list[str] = list(_HAND_PRIMARY) + list(_MIMIC_MAP.keys())
    positions: list[float] = []

    for j in _HAND_PRIMARY:
        positions.append(float(cfg.get(j, 0.0)))
    for j, (primary, mult) in _MIMIC_MAP.items():
        positions.append(float(cfg.get(primary, 0.0) * mult))

    pt = JointTrajectoryPoint()
    pt.positions = positions
    sec = int(duration)
    pt.time_from_start = Duration(sec=sec, nanosec=int((duration - sec) * 1e9))

    traj = JointTrajectory()
    traj.joint_names = all_names
    traj.points.append(pt)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    return goal


def _close_extra(cfg: dict[str, float]) -> dict[str, float]:
    """Adiciona margem de fechamento sobre a configuração de grasp."""
    return {j: float(min(cfg.get(j, 0.0) + _CLOSE_EXTRA * HAND_LIMITS[j],
                         HAND_LIMITS[j]))
            for j in _HAND_PRIMARY}


# Verificação de colisão — delega ao módulo `collision.py`
from .collision import (
    bbox_overlap     as _bbox_overlap,
    arm_clears_bbox  as _arm_clears_bbox,
    arm_clears_world as _arm_clears_world,
)


# Trajetória Cartesiana

def _cartesian_arm_goal(q_start: np.ndarray,
                        q_end: np.ndarray,
                        n_via: int = _N_CART_VIA
                        ) -> tuple[FollowJointTrajectory.Goal, float]:
    """
    Trajetória com n_via waypoints interpolados no espaço Cartesiano do TCP.

    Cada waypoint é calculado por IK ao longo da linha reta entre FK(q_start)
    e FK(q_end), com seed propagado da solução anterior. Isso garante que:
      1. O TCP percorra uma trajetória previsível (linha reta Cartesiana).
      2. As mudanças de junta por passo sejam pequenas (seed contínuo).
      3. Nenhum link do braço varra regiões de obstáculos durante transições
         de grande amplitude no espaço de juntas (ex.: HOME → via_pick).

    A mudança de branch de IK (e.g., "home branch" → "pick branch") ocorre
    gradualmente ao longo dos n_via passos, nunca num único salto de 3 rad.
    """
    T0 = forward_kinematics(q_start)
    T1 = forward_kinematics(q_end)
    p0 = T0[:3, 3]
    p1 = T1[:3, 3]

    configs: list[np.ndarray] = []
    q_prev = q_start.copy()

    for i in range(1, n_via + 1):
        alpha = float(i) / n_via
        p_i = p0 + alpha * (p1 - p0)
        q_i, _ = inverse_kinematics(p_i, _AV_DOWN, q_seed=q_prev, elbow_up=False)
        configs.append(q_i)
        q_prev = q_i

    # Duração: maior delta de junta em qualquer passo consecutivo
    q_all = [q_start] + configs
    max_delta = max(
        float(np.max(np.abs(q_all[i + 1] - q_all[i])))
        for i in range(len(q_all) - 1)
    )
    step_dur = max(max_delta / _MAX_JOINT_VEL, 0.20)
    total_dur = max(step_dur * len(configs), _CART_DUR_FLOOR)

    traj = JointTrajectory()
    traj.joint_names = _ARM_JOINTS

    for i, q in enumerate(configs, 1):
        smooth = 0.5 * (1.0 - math.cos(math.pi * i / len(configs)))
        t = total_dur * smooth
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        if i == len(configs):
            pt.velocities    = [0.0] * 6
            pt.accelerations = [0.0] * 6
        sec = int(t)
        pt.time_from_start = Duration(sec=sec, nanosec=int((t - sec) * 1e9))
        traj.points.append(pt)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    return goal, total_dur


# Nó executor

class GraspExecutorNode(Node):

    def __init__(self):
        super().__init__('grasp_executor')

        # Parâmetros do sistema (todos declarados antes do uso)
        self.declare_parameter('sim_only', True)
        self.declare_parameter('pick_x', 0.65)
        self.declare_parameter('pick_y', 0.00)
        # `pick_z_*` = CENTRO Z DO OBJETO na pose canônica (belt + half_h),
        # usado como `ref_obj.z` em `_solve_grasp_poses_at` — os deltas
        # PICK_TCP_WORLD − ref_obj representam centro-do-objeto → TCP.
        # NÃO armazenar o TCP_z aqui (delta_z zeraria e o TCP cairia no
        # centro do objeto em vez de acima do topo).
        #   frasco — center_z = belt(0.806) + half_h(0.045) = 0.851
        #   tubo   — center_z = belt(0.806) + half_h(0.060) = 0.866
        #   ampola — center_z = belt(0.806) + half_h(0.0375) = 0.844
        self.declare_parameter('pick_z_frasco', 0.851)
        self.declare_parameter('pick_z_tubo',   0.866)
        self.declare_parameter('pick_z_ampola', 0.844)
        self.declare_parameter('box1_x', -0.05)
        self.declare_parameter('box1_y',  0.65)
        self.declare_parameter('box1_z',  0.60)
        self.declare_parameter('box2_x',  0.25)
        self.declare_parameter('box2_y',  0.65)
        self.declare_parameter('box2_z',  0.60)
        self.declare_parameter('box3_x',  0.55)
        self.declare_parameter('box3_y',  0.65)
        self.declare_parameter('box3_z',  0.60)

        self._pick_xy = np.array([
            self.get_parameter('pick_x').value,
            self.get_parameter('pick_y').value,
        ])
        self._pick_z: dict[str, float] = {
            'frasco': self.get_parameter('pick_z_frasco').value,
            'tubo':   self.get_parameter('pick_z_tubo').value,
            'ampola': self.get_parameter('pick_z_ampola').value,
        }
        self._boxes: dict[str, np.ndarray] = {
            'box1': np.array([self.get_parameter('box1_x').value,
                              self.get_parameter('box1_y').value,
                              self.get_parameter('box1_z').value]),
            'box2': np.array([self.get_parameter('box2_x').value,
                              self.get_parameter('box2_y').value,
                              self.get_parameter('box2_z').value]),
            'box3': np.array([self.get_parameter('box3_x').value,
                              self.get_parameter('box3_y').value,
                              self.get_parameter('box3_z').value]),
        }

        cb = ReentrantCallbackGroup()

        # Action clients para os controllers do braço e mão
        self._arm_ac = ActionClient(
            self, FollowJointTrajectory,
            '/cr10_group_controller/follow_joint_trajectory',
            callback_group=cb)
        self._hand_ac = ActionClient(
            self, FollowJointTrajectory,
            '/hand_position_controller/follow_joint_trajectory',
            callback_group=cb)

        # Cliente para retirar o objeto da pick station após grasp
        self._retreat_cli = self.create_client(
            Trigger, '/conveyor/retreat', callback_group=cb)

        # Módulo de fechamento incremental com detecção de contato por lag
        # articular (commanded vs actual).
        self._perfect_grasp = PerfectGrasp(
            self, send_hand_fn=self._send_hand)

        # Serviços expostos à GUI (não bloqueantes — iniciam thread)
        self.create_service(Trigger, '/cell/execute_grasp',
                            self._cb_execute, callback_group=cb)
        self.create_service(Trigger, '/cell/go_home',
                            self._cb_home, callback_group=cb)

        # Modo manual — apenas mão (sem mover o braço).
        self.create_service(Trigger, '/cell/close_hand',
                            self._cb_close_hand, callback_group=cb)
        self.create_service(Trigger, '/cell/open_hand',
                            self._cb_open_hand, callback_group=cb)

        # Estado interno
        self._current_q      = _HOME_Q.copy()
        self._last_detection: str | None = None
        self._last_pick_pos:  np.ndarray | None = None  # posição 3D da câmera
        # Posição mundial do entity Gazebo `pick_object` (lida do
        # /gazebo/model_states a cada tick).
        self._world_obj_pos: np.ndarray | None = None
        self._world_obj_lock = threading.Lock()
        # Serializa o check-and-set de _busy entre os serviços (executados
        # concorrentemente sob ReentrantCallbackGroup +
        # MultiThreadedExecutor).
        self._busy_lock      = threading.Lock()
        self._busy           = False
        self._status_msg     = 'IDLE'

        # Posição de spawn publicada pelo conveyor (status JSON, campo
        # `obj_pos`).
        self._conveyor_obj_pos: np.ndarray | None = None

        # Subscriptions
        self.create_subscription(
            JointState, '/joint_states', self._cb_joint_state, 10,
            callback_group=cb)
        self.create_subscription(
            String, '/conveyor/status', self._cb_conveyor_status, 10,
            callback_group=cb)
        self.create_subscription(
            Detection2DArray, '/detected_objects', self._cb_detection, 10,
            callback_group=cb)
        # Posição real (ground-truth) do objeto picável via Gazebo state.
        if ModelStates is not None:
            self.create_subscription(
                ModelStates, '/gazebo/model_states',
                self._cb_model_states, 10, callback_group=cb)

        self._pub_status = self.create_publisher(
            String, '/cell/status', 10)
        self.create_timer(0.5, self._tick_status)

        self.get_logger().info('GraspExecutor — aguardando action servers...')
        self._arm_ac.wait_for_server(timeout_sec=20.0)
        self._hand_ac.wait_for_server(timeout_sec=20.0)
        self.get_logger().info('GraspExecutor pronto.')

    def _tick_status(self):
        self._pub_status.publish(String(data=json.dumps({
            'state': self._status_msg,
            'busy': self._busy,
            'last_obj': self._last_detection,
        })))

    def _cb_joint_state(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in _ARM_JOINTS:
                self._current_q[_ARM_JOINTS.index(name)] = msg.position[i]
        # Alimenta o módulo de fechamento com a leitura atual da mão
        # (necessário para o cálculo de lag durante close_until_contact).
        self._perfect_grasp.update_from_joint_state(msg)

    def _cb_model_states(self, msg) -> None:
        """Atualiza a posição mundial do `pick_object` direto do Gazebo."""
        try:
            names = list(msg.name)
            idx = names.index('pick_object')
        except (ValueError, AttributeError):
            with self._world_obj_lock:
                self._world_obj_pos = None
            return
        pose = msg.pose[idx]
        with self._world_obj_lock:
            self._world_obj_pos = np.array([
                pose.position.x, pose.position.y, pose.position.z],
                dtype=float)

    def _get_world_obj_pos(self) -> np.ndarray | None:
        with self._world_obj_lock:
            return None if self._world_obj_pos is None else self._world_obj_pos.copy()

    def _cb_conveyor_status(self, msg: String) -> None:
        """Guarda a posição de spawn (`obj_pos`) do status do conveyor."""
        try:
            data = json.loads(msg.data)
            pos = data.get('obj_pos')
            self._conveyor_obj_pos = (
                np.array(pos, dtype=float)
                if (data.get('has_object') and pos) else None)
        except (json.JSONDecodeError, ValueError, TypeError):
            pass

    def _cb_detection(self, msg: Detection2DArray):
        if msg.detections:
            det  = msg.detections[0]
            hyp  = det.results[0]
            self._last_detection = hyp.hypothesis.class_id
            pos = hyp.pose.pose.position
            # Aceita posição 3D somente se o detector a estimou (z != 0)
            if abs(pos.z) > 1e-3:
                self._last_pick_pos = np.array([pos.x, pos.y, pos.z])
            else:
                self._last_pick_pos = None
        else:
            self._last_detection = None
            self._last_pick_pos  = None

    def _cb_execute(self, _request, response: Trigger.Response):
        obj_class = self._last_detection
        if obj_class is None or obj_class not in _OBJECT_MAP:
            response.success = False
            response.message = (
                f'Nenhum objeto válido detectado '
                f'(detectado: {self._last_detection!r}). '
                'Verifique a esteira antes de agarrar.')
            return response

        with self._busy_lock:
            if self._busy:
                response.success = False
                response.message = 'Executor ocupado — aguarde o ciclo atual terminar.'
                return response
            self._busy = True
        threading.Thread(
            target=self._run_cycle, args=(obj_class,), daemon=True).start()

        response.success = True
        response.message = f'Ciclo iniciado para: {obj_class}'
        return response

    def _cb_home(self, _request, response: Trigger.Response):
        with self._busy_lock:
            if self._busy:
                response.success = False
                response.message = 'Executor ocupado.'
                return response
            self._busy = True
        threading.Thread(target=self._do_home, daemon=True).start()
        response.success = True
        response.message = 'Indo para home.'
        return response

    # Modo manual — somente mão (sem mover o braço)
    def _cb_close_hand(self, _request, response: Trigger.Response):
        """
        Fecha a mão na configuração de grip equivalente ao objeto detectado
        na esteira. Não move o braço. Bloqueado durante ciclos automáticos
        (`_busy`).

        Mapeamento (apresentação SLIDE 7):
          frasco → palm_grip      → preensão palmar para frasco de medicamento
          tubo   → claw_grip      → preensão em garra para tubo de ensaio
          ampola → fingertip_grip → preensão de pinça fina para ampola
        """
        obj = self._last_detection
        if obj is None or obj not in _OBJECT_MAP:
            response.success = False
            response.message = (f'Nenhum objeto válido detectado na esteira '
                                f'(detectado: {obj!r}).')
            return response

        with self._busy_lock:
            if self._busy:
                response.success = False
                response.message = 'Executor ocupado.'
                return response
            self._busy = True

        grip_type, _, obj_diam = _OBJECT_MAP[obj]
        cfg_grasp  = hand_ik(grip_type, obj_diam)
        cfg_closed = _close_extra(cfg_grasp)

        # Executa em thread para não bloquear o serviço
        def _do():
            try:
                self._status_msg = f'CLOSE_HAND:{obj}({grip_type})'
                self.get_logger().info(
                    f'[MANUAL] Fechando mão em {grip_type!r} para {obj!r} '
                    f'(PerfectGrasp + contact detection)')
                # PerfectGrasp: pré-shape + fechamento incremental com
                # parada por contato. Evita ejeção do objeto.
                from .poses import HAND_PRESHAPE, OBJ_GRIP
                grip_label = OBJ_GRIP.get(obj)
                preshape_slider = (HAND_PRESHAPE.get(grip_label)
                                    if grip_label else None)
                preshape_rad = (
                    {j: HAND_LOWER[j] + float(preshape_slider.get(j, 0)) / 200.0
                        * (HAND_LIMITS[j] - HAND_LOWER[j])
                     for j in _HAND_PRIMARY}
                    if preshape_slider else None)

                # Cage check antes de fechar — mesmo padrão da Fase 2 do
                # ciclo autônomo.
                cage = cage_status(self._current_q,
                                   preshape_rad or cfg_closed,
                                   obj,
                                   world_obj_pos=self._get_world_obj_pos())
                if not cage.valid:
                    self.get_logger().warn(f'[MANUAL:CAGE] {cage.summary()}')
                else:
                    self.get_logger().info('[MANUAL:CAGE] gaiola válida — fechando')

                self._perfect_grasp.close_until_contact(
                    cfg_closed,
                    label=f'{obj}/{grip_type}',
                    preshape_cfg_rad=preshape_rad)
            finally:
                self._status_msg = 'IDLE'
                self._busy = False

        threading.Thread(target=_do, daemon=True).start()
        response.success = True
        response.message = f'{obj} → {grip_type}'
        return response

    def _cb_open_hand(self, _request, response: Trigger.Response):
        """Abre completamente a mão (cfg `open`)."""
        with self._busy_lock:
            if self._busy:
                response.success = False
                response.message = 'Executor ocupado.'
                return response
            self._busy = True

        def _do():
            try:
                self._status_msg = 'OPEN_HAND'
                self.get_logger().info('[MANUAL] Abrindo mão')
                self._send_hand(HAND_CONFIGS['open'], 1.0)
            finally:
                self._status_msg = 'IDLE'
                self._busy = False

        threading.Thread(target=_do, daemon=True).start()
        response.success = True
        response.message = 'Mão aberta'
        return response

    def _solve_grasp_poses_at(self, obj_class: str, obj_world: np.ndarray):
        """Computa PRE/APPR/PICK em rad para o objeto na posição
        `obj_world` (x, y, z = centro do cilindro em world frame).

        Retorna (q_pre, q_appr, q_pick) numpy arrays ou (None,)*3.
        """
        from .poses import solve_grasp_poses_at_world
        d_pre, d_appr, d_pick = solve_grasp_poses_at_world(
            obj_class, obj_world, relax=True)
        if d_pick is None:
            return (None, None, None)
        return (_deg_dict_to_rad(d_pre),
                _deg_dict_to_rad(d_appr),
                _deg_dict_to_rad(d_pick))

    def _run_cycle(self, obj_class: str):
        """Ciclo completo: Pick → Lift → Place → Home."""
        grip_type, box_key, obj_diam = _OBJECT_MAP[obj_class]

        # Posição mundial do objeto picável — prioridade:
        #   1. Ground-truth do Gazebo (/gazebo/model_states)
        #   2. Posição de SPAWN publicada pelo conveyor (obj_pos)
        #   3. Detecção 3D do object_detector (se disponível)
        #   4. Coordenadas hardcoded (fallback)
        world_obj = self._get_world_obj_pos()
        if world_obj is not None:
            pick_w = world_obj.copy()
            self.get_logger().info(
                f'[CICLO] {obj_class} — usando posição GAZEBO '
                f'({pick_w[0]:.3f}, {pick_w[1]:.3f}, {pick_w[2]:.3f})')
        elif self._conveyor_obj_pos is not None:
            world_obj = self._conveyor_obj_pos.copy()
            pick_w = world_obj.copy()
            self.get_logger().info(
                f'[CICLO] {obj_class} — usando posição de SPAWN do conveyor '
                f'({pick_w[0]:.3f}, {pick_w[1]:.3f}, {pick_w[2]:.3f})')
        elif self._last_pick_pos is not None:
            pick_w = self._last_pick_pos.copy()
            self.get_logger().info(
                f'[CICLO] {obj_class} — usando posição DETECTOR '
                f'({pick_w[0]:.3f}, {pick_w[1]:.3f}, {pick_w[2]:.3f})')
        else:
            pick_w = np.array([self._pick_xy[0], self._pick_xy[1],
                               self._pick_z[obj_class]])
            self.get_logger().warn(
                f'[CICLO] {obj_class} — caindo no FALLBACK hardcoded '
                f'({pick_w[0]:.3f}, {pick_w[1]:.3f}, {pick_w[2]:.3f})')
        box_w = self._boxes[box_key]

        # Converter world → robot base frame antes de chamar o IK.
        # O IK calcula posições relativas ao base_link do robô, que está
        # em world z = _ROBOT_BASE_Z (0.405 m).
        p_pick = _w2r(pick_w)
        p_box  = _w2r(box_w)

        success = False
        self._status_msg = f'PICKING:{obj_class}'
        self.get_logger().info(
            f'[CICLO] {obj_class} → {grip_type} → {box_key} | '
            f'pick_robot={np.round(p_pick, 3).tolist()}  '
            f'box_robot={np.round(p_box, 3).tolist()}')

        try:
            # Calcular todas as poses IK em robot frame
            # via_box: altura fixa _TRANSIT_Z (1.15m world) sobre a caixa.
            via_pos       = np.array([p_box[0],  p_box[1],  _TRANSIT_Z])
            approach_box  = p_box + np.array([0.0, 0.0, _APPROACH_CLEAR])

            # Lado do PICK: IK DINÂMICA na posição mundial real Tenta
            # resolver as 3 poses (pre/appr/pick) centradas na posição REAL
            # do objeto (pick_w vem do Gazebo ou detector).
            if obj_class not in _POSES_PICK_DEG:
                raise RuntimeError(f'Objeto {obj_class} sem pose em poses.py')

            q_dyn_pre, q_dyn_appr, q_dyn_pick = (None, None, None)
            if world_obj is not None:
                q_dyn_pre, q_dyn_appr, q_dyn_pick = self._solve_grasp_poses_at(
                    obj_class, world_obj)

            if q_dyn_pick is not None:
                q_pick = q_dyn_pick
                q_ap   = q_dyn_appr
                q_lift = q_dyn_pre
                self.get_logger().info(
                    f'[CICLO] IK dinâmica resolvida para {obj_class} '
                    f'em ({world_obj[0]:.3f},{world_obj[1]:.3f},{world_obj[2]:.3f})')
            elif world_obj is not None:
                # Conhecemos a posição real e a IK falhou: usar as poses
                # canônicas garantiria ERRAR o objeto — melhor abortar
                # com mensagem clara do que agarrar o ar.
                raise RuntimeError(
                    f'IK dinâmica falhou para {obj_class} em '
                    f'({world_obj[0]:.3f},{world_obj[1]:.3f},{world_obj[2]:.3f}) '
                    f'— fora do alcance/sem ramo. Ciclo abortado.')
            else:
                q_pick = _deg_dict_to_rad(_POSES_PICK_DEG[obj_class])
                q_ap   = _deg_dict_to_rad(_POSES_APPROACH_DEG[obj_class])
                q_lift = _deg_dict_to_rad(_POSES_PRE_APPROACH_DEG[obj_class])
                self.get_logger().warn(
                    f'[CICLO] {obj_class} sem posição mundial — usando '
                    f'poses cacheadas (posição canônica).')

            # Lado da caixa: approach_box com seed compacto; via_box com
            # seed _VIA_BOX_SEED_Q que converge para ramo compacto (q2≈-0.5,
            # q3≈-0.8) para TODOS os três boxes.
            q_ab,   ok3     = inverse_kinematics(approach_box,   _AV_DOWN, _APPROACH_BOX_SEED_Q, elbow_up=False)
            if not ok3:
                raise RuntimeError(f'IK abordagem box falhou: {approach_box}')
            q_via,  ok_via  = inverse_kinematics(via_pos,        _AV_DOWN, _VIA_BOX_SEED_Q,      elbow_up=False)
            if not ok_via:
                raise RuntimeError(f'IK via_box falhou: {via_pos}')

            # ── Verificação estática: nenhum link do braço toca o objeto ─
            # O check é feito nos waypoints IK calculados. A FASE 0 (caminho
            # Cartesiano) garante que nenhum waypoint intermédio viola esses
            # limites. Qualquer violação nos waypoints fixos levanta RuntimeError.
            obj_bbox = _PICK_OBJ_BBOX.get(obj_class)
            if obj_bbox is not None:
                ok_c, msg_c = _arm_clears_bbox(q_ap, obj_bbox)
                if not ok_c:
                    raise RuntimeError(
                        f'Colisão de braço com objeto [{obj_class}] em approach_pick: {msg_c}')
                # pick: mão toca o objeto (esperado); braço NÃO deve tocar
                ok_c, msg_c = _arm_clears_bbox(q_pick, obj_bbox, links=(1, 2, 3, 4, 5))
                if not ok_c:
                    raise RuntimeError(
                        f'Colisão de braço com objeto [{obj_class}] em pick: {msg_c}')

            bin_bbox = _BIN_BBOX.get(box_key)
            if bin_bbox is not None:
                for wp_name, q_wp in (('via_box', q_via), ('approach_box', q_ab)):
                    ok_c, msg_c = _arm_clears_bbox(q_wp, bin_bbox)
                    if not ok_c:
                        raise RuntimeError(
                            f'Colisão de braço com caixa [{box_key}] em {wp_name}: {msg_c}')

            # ── Verificação contra TODOS os obstáculos estáticos do mundo ─
            # Pontos imutáveis (esteira, prateleira, pedestal, câmera,
            # paredes).
            from .poses import PICK_SKIP_OBSTACLES
            _skip_pick = PICK_SKIP_OBSTACLES.get(obj_class, {'belt_surface'})
            for wp_name, q_wp, skip_w in (
                    ('approach_pick', q_ap,   _skip_pick),
                    ('pick',          q_pick, _skip_pick),
                    ('lift',          q_lift, _skip_pick),
                    ('via_box',       q_via,  None),
                    ('approach_box',  q_ab,   None),
                    ('home',          _HOME_Q, None)):
                ok_w, msg_w = _arm_clears_world(q_wp, skip=skip_w)
                if not ok_w:
                    raise RuntimeError(f'[{wp_name}] {msg_w}')

            # Configurações da mão
            cfg_open   = HAND_CONFIGS['open']
            cfg_grasp  = hand_ik(grip_type, obj_diam)
            cfg_closed = _close_extra(cfg_grasp)

            # FASE 1: HOME → APPROACH (acima do objeto, mão aberta) Em vez
            # de ir direto ao PICK, paramos 60 mm acima — aí pre-fechamos a
            # mão em CUP shape (driver ~0.5, ~50% curl) FORA do volume do
            # objeto.
            self._validate_sweep(_HOME_Q, q_ap, n_steps=20,
                                 name='HOME→approach', skip=_skip_pick)
            # margem 5 mm (= margin_wrist de pose_is_safe): na descida o
            # punho passa legitimamente a ~8 mm do AABB do belt_frame
            # (visual-only) — a margem default de 10 mm gerava falso
            # positivo e abortava o ciclo do frasco.
            self._validate_sweep(q_ap, q_pick, n_steps=10,
                                 name='approach→pick', skip=_skip_pick,
                                 margin=0.005)
            self._status_msg = f'APPROACH:{obj_class}'
            self.get_logger().info(
                '[F1] HOME → APPROACH (60 mm acima do PICK, mão aberta)')
            self._send_hand_async(cfg_open, 1.0)
            self._send_arm(q_ap)

            # FASE 1.5: Pre-close na APPROACH (cup shape em ar) Computa
            # pre-shape a partir de HAND_PRESHAPE (poses.py).
            from .poses import HAND_PRESHAPE, OBJ_GRIP
            grip_label = OBJ_GRIP.get(obj_class)
            preshape_slider = HAND_PRESHAPE.get(grip_label) if grip_label else None
            preshape_rad = (
                {j: HAND_LOWER[j] + float(preshape_slider.get(j, 0)) / 200.0
                    * (HAND_LIMITS[j] - HAND_LOWER[j])
                 for j in _HAND_PRIMARY}
                if preshape_slider else None)
            if preshape_rad is not None:
                self._status_msg = f'PRESHAPE:{obj_class}'
                self.get_logger().info(
                    '[F1.5] Pre-shape (cup) na APPROACH antes da descida')
                self._send_hand(preshape_rad, 0.8)
                time.sleep(0.85)

            # NOTA (2026-07-05): a antiga FASE 1.55 (tubo step-aside −X 50
            # mm) foi REMOVIDA.

            # FASE 1.6: APPROACH → PICK (descida com mão em cup) Mão FIXA em
            # preshape durante a descida vertical de 60 mm.
            self._status_msg = f'DESCEND:{obj_class}'
            self.get_logger().info(
                '[F1.6] APPROACH → PICK (descida, mão fixa em cup-shape)')
            self._send_arm(q_pick)

            # ── FASE 2: Fechar mão com detecção de contato (PerfectGrasp) ─
            # Agora os fingertips estão em cup-shape AO REDOR do objeto.
            # O fechamento final pega o objeto pelo lado — não por cima.
            self._status_msg = f'GRASPING:{obj_class}'

            # Cage check: garante que os fingertips estão em torno do objeto
            # (não acima, não penetrando, dentro do alcance).
            cage = cage_status(self._current_q,
                               preshape_rad or cfg_closed,
                               obj_class,
                               world_obj_pos=self._get_world_obj_pos())
            if not cage.valid:
                self.get_logger().warn(f'[F2:CAGE] {cage.summary()}')
            else:
                self.get_logger().info('[F2:CAGE] gaiola válida — fechando')

            self.get_logger().info(
                '[F2] Fechamento final sobre o objeto (PerfectGrasp + contato)')
            result = self._perfect_grasp.close_until_contact(
                cfg_closed,
                label=f'{obj_class}/{grip_type}',
                preshape_cfg_rad=None)  # já em preshape — não re-aplicar
            if not result.contact_detected:
                self.get_logger().warn(
                    f'[F2] {obj_class}: nenhum contato detectado '
                    f'(stalled={[j for j,s in result.stalled.items() if s]}). '
                    f'O objeto pode escorregar — prosseguindo mesmo assim.')
            time.sleep(0.15)   # estabiliza atrito antes de levantar

            # FASE 3: Levantar com objeto
            self._status_msg = f'LIFTING:{obj_class}'
            self.get_logger().info('[F3] Levantando (objeto preso)')
            self._send_arm(q_lift)

            # FASE 4: Trânsito lateral — pick area → via_box Caminho
            # Cartesiano: TCP percorre linha reta de lift_pos até via_box.
            self._status_msg = f'TRANSIT:{obj_class}→{box_key}'
            self.get_logger().info(f'[F4] Trânsito lateral → {box_key} (Cartesiano)')
            self._send_arm_cartesian(q_via)

            # FASE 5: Descer para abordagem da caixa
            self.get_logger().info(f'[F5] Descida abordagem → {box_key} (Cartesiano)')
            self._send_arm_cartesian(q_ab)

            # FASE 6: Soltar acima da caixa Sem detach cinemático: basta
            # abrir a mão.
            self._status_msg = f'PLACING:{obj_class}'
            self.get_logger().info(f'[F6] Soltando acima de {box_key}')
            self._send_hand(cfg_open, 1.0)
            time.sleep(0.4)
            self._call_retreat()

            # FASE 7: Retorno → HOME (Cartesiano)
            self._status_msg = 'HOMING'
            self.get_logger().info('[F7] Retorno → HOME (Cartesiano)')
            self._send_arm_cartesian(_HOME_Q)
            success = True
            self.get_logger().info(f'[SUCESSO] {obj_class} ({grip_type}) → {box_key}')

        except Exception as exc:
            self.get_logger().error(f'[FALHA] {exc}')
            self._send_hand(HAND_CONFIGS['open'], 1.0)
            self._do_home()

        finally:
            self._status_msg = 'IDLE'
            self._busy = False
            self._pub_status.publish(String(data=json.dumps({
                'state': 'CYCLE_DONE',
                'object': obj_class,
                'success': success,
                'grip': grip_type,
                'box': box_key if success else 'none',
            })))

    def _call_retreat(self):
        """Pede ao conveyor_controller para deletar o objeto da pick station."""
        if not self._retreat_cli.service_is_ready():
            self.get_logger().warn('[GRASP] /conveyor/retreat indisponível — objeto não removido.')
            return
        future = self._retreat_cli.call_async(Trigger.Request())
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 5.0:
            time.sleep(0.05)
        if future.done() and future.result() is not None:
            result = future.result()
            if result.success:
                self.get_logger().info('[GRASP] Objeto removido da pick station.')
            else:
                self.get_logger().warn(f'[GRASP] Retreat: {result.message}')

    def _do_home(self):
        """Envia braço ao home via caminho Cartesiano e libera busy."""
        # Caminho Cartesiano para HOME: evita que o braço desça abaixo
        # de obstáculos durante a transição de qualquer configuração para HOME.
        self._send_arm_cartesian(_HOME_Q)
        time.sleep(0.3)
        if self._busy and self._status_msg == 'HOMING':
            self._status_msg = 'IDLE'
            self._busy = False

    def _send_arm(self, q: np.ndarray):
        """Envia trajetória ao braço e bloqueia até o goal ser concluído."""
        goal, _ = _make_smooth_arm_goal(self._current_q, q)
        self._arm_ac.send_goal(goal)  # blocking in this rclpy version
        self._current_q = q.copy()

    def _validate_sweep(self, q_start: np.ndarray, q_end: np.ndarray,
                         n_steps: int = 20, name: str = 'sweep',
                         skip: set | None = None,
                         margin: float = 0.010):
        """
        Amostra a varredura articular `q_start → q_end` e verifica colisão
        contra todos os obstáculos do mundo. Levanta `RuntimeError` se algum
        ponto intermediário invadir um obstáculo — necessário para movimentos
        em espaço articular onde o TCP não segue trajetória previsível.
        `skip` ignora obstáculos específicos (e.g. PICK_SKIP_OBSTACLES na
        descida approach→pick, onde o flange chega ao nível da correia).
        """
        for i in range(1, n_steps + 1):
            alpha = float(i) / n_steps
            q_i = q_start + alpha * (q_end - q_start)
            ok, msg = _arm_clears_world(q_i, skip=skip, margin=margin)
            if not ok:
                raise RuntimeError(
                    f'Varredura {name} colide em alpha={alpha:.2f}: {msg}')

    def _send_arm_cartesian(self, q_target: np.ndarray,
                             n_via: int = _N_CART_VIA):
        """
        Envia o braço para q_target via trajetória Cartesiana.

        Calcula n_via waypoints intermediários ao longo da linha reta
        TCP_atual → TCP_alvo, resolvendo IK com seed propagado.
        """
        goal, _ = _cartesian_arm_goal(self._current_q, q_target, n_via)
        self._arm_ac.send_goal(goal)  # blocking
        self._current_q = q_target.copy()

    def _send_hand(self, cfg: dict[str, float], duration: float):
        """Envia posição à mão e bloqueia até o goal ser concluído."""
        self._hand_ac.send_goal(_make_hand_goal(cfg, duration))  # blocking

    def _send_hand_async(self, cfg: dict[str, float], duration: float):
        """Dispara goal da mão sem bloquear (fire-and-forget)."""
        self._hand_ac.send_goal_async(_make_hand_goal(cfg, duration))


def main(args=None):
    rclpy.init(args=args)
    node = GraspExecutorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
