"""Nó de admitância: força → setpoint cartesiano → IK → JointTrajectory.

Camadas de segurança: força limite, idade da força, e-stop, piso z e
resíduo da IK. Um ponto de trajetória por tick de 30 ms; a IK integra a
partir do último q COMANDADO (o medido só serve para FK, log e reset).
"""
from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped, WrenchStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from .admittance import (AdmittanceAbort, AdmittanceController,
                         arm_stiffness_limit)
from .constants import (ARM_JOINTS, ARM_STIFFNESS_MIN_N_M, DT_S,
                        ENABLE_MAX_N, LINK_CLEARANCE_M, MIN_CELL_Z, SIM_LAG_MAX_RAD,
                        TABLE_HALF_XY, TARE_MAX_TILT_RAD,
                        TORQUE_ABORT_LIMIT_NM, T_TARED, FORCE_ABORT_LIMIT_N, FORCE_STALE_S, SIM_ROBOT_IP,
                        HOME_Q_RAD, JOG_SPEED_DEFAULT_PCT, JOG_SPEED_MIN_PCT,
                        MIN_TCP_Z, S_ENABLE, S_ESTOP, S_HOME,
                        S_RESET, STATES, T_ARM_CMD, T_FT, T_JOG_GOTO, T_JOG_RESULT,
                        T_JOINT_STATES, T_REAL_STATUS, JOG_MARGIN_S,
                        T_PARAMS, T_REFERENCE, T_STATE, T_TCP_POSE, T_TICK,
                        home_q)
from .kinematics import Chain, _log
from .jog import plan_jog
from .real_link import real_ok
from .urdf_builder import build_robot_urdf

IK_RESIDUAL_M = 0.005     # acima disto: LIMITE
MAX_DQ_TICK = 0.05        # rad por tick por junta; acima: LIMITE
P, A, S_, L, AB, J = STATES  # PARADO ATIVO SEM_FORCA LIMITE ABORTADO POSICIONANDO

# parâmetros ROS = campos do JSON; valor = padrão
DEFAULTS = {'mass_kg': 2.0, 'damping_ns_m': 40.0, 'stiffness_n_m': 0.0,
            'deadband_n': 0.5, 'v_max_ms': 0.10, 'max_excursion_m': 0.15,
            'abort_force_n': FORCE_ABORT_LIMIT_N,
            'abort_torque_nm': TORQUE_ABORT_LIMIT_NM, 'dt_s': DT_S,
            'traj_lead_s': 0.06, 'force_source': 'real'}


# faixa aceita por parâmetro (o `ros2 param set` não passa pelos limites da GUI)
RANGES = {'mass_kg': (0.1, 50.0), 'damping_ns_m': (0.0, 1000.0),
          'stiffness_n_m': (0.0, 2000.0), 'deadband_n': (0.0, 5.0),
          'v_max_ms': (0.01, 0.25), 'max_excursion_m': (0.01, 0.5),
          'abort_torque_nm': (0.2, TORQUE_ABORT_LIMIT_NM),
          'traj_lead_s': (0.02, 0.5), 'dt_s': (0.005, 0.1)}


def _quat(R):
    """Quaternion (x, y, z, w) de uma rotação 3×3."""
    t = np.trace(R)
    if t > 0:
        s = 2 * math.sqrt(t + 1)
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, s / 4)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0] * 4
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


class AdmittanceNode(Node):
    def __init__(self):
        super().__init__('rehab_admittance')
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        p = self._params()
        self.ctrl = self._make_ctrl(p)  # levanta ValueError se instável
        self.p = p

        urdf, _ = build_robot_urdf(get_package_share_directory('cra_description'),
                                   get_package_share_directory('rehab_pack'))
        self.chain = Chain(urdf, tip='tcp_link')
        self.chain_ft = Chain(urdf, tip='load_cell_link')
        self.chain_links = [Chain(urdf, tip=t) for t in ('Link3', 'Link4', 'Link5')]
        # célula → centro da empunhadura, no frame da célula
        home = np.array(HOME_Q_RAD)
        self.r_grip = (np.linalg.inv(self.chain_ft.fk(home)) @ self.chain.fk(home))[:3, 3]
        if self.chain.names != ARM_JOINTS:
            raise RuntimeError(f'cadeia {self.chain.names} != {ARM_JOINTS}')

        self.state = P
        self.q = None            # medido
        self.q_cmd = None        # último comandado
        self.R0 = None           # orientação a manter
        self.force_w = np.zeros(3)   # força no world
        self.f_axial = 0.0
        self.ft_t = None             # chegada da última força (relógio de parede)
        self.ft_acc = []             # amostras desde o último tick
        self.x_prev = None           # (fonte, posição) do tick anterior
        self._sim_last, self._wall_last = 0.0, time.monotonic()
        self.m_peak = 0.0            # idem, |M| no centro da empunhadura
        self.tare_R = None           # orientação da célula na última tara
        self.t_start = time.monotonic()
        self.f_peak = 0.0            # maior |F| de amostra desde o último tick
        self.last_msg = ''
        # modo mirror (CR10 real): sem robô conectado, sem E-Stop e em sincronia
        # com o sim, nada de habilitar — e se perder isso em ATIVO, aborta.
        self.declare_parameter('require_real', False)
        self.require_real = bool(self.get_parameter('require_real').value)
        self.real, self.real_t = None, 0.0
        self.jog_goal, self.jog_end = None, 0.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_cmd = self.create_publisher(JointTrajectory, T_ARM_CMD, 10)
        self.pub_tick = self.create_publisher(Float64MultiArray, T_TICK, 10)
        self.pub_pose = self.create_publisher(PoseStamped, T_TCP_POSE, 10)
        self.pub_state = self.create_publisher(String, T_STATE, latched)
        self.pub_params = self.create_publisher(String, T_PARAMS, latched)
        self.pub_jog = self.create_publisher(String, T_JOG_RESULT, 10)
        self.create_subscription(WrenchStamped, T_FT, self._on_ft, 50)
        self.create_subscription(JointState, T_JOINT_STATES, self._on_js, 20)
        self.create_subscription(PointStamped, T_REFERENCE, self._on_ref, 10)
        self.create_subscription(Float64, T_TARED, self._on_tared, latched)
        self.create_subscription(String, T_REAL_STATUS, self._on_real, 5)
        self.create_subscription(Float64MultiArray, T_JOG_GOTO, self._on_goto, 5)
        self.create_service(SetBool, S_ENABLE, self._srv_enable)
        self.create_service(Trigger, S_ESTOP, self._srv_estop)
        self.create_service(Trigger, S_RESET, self._srv_reset)
        self.create_service(Trigger, S_HOME, self._srv_home)
        self.add_on_set_parameters_callback(self._on_params)
        self._publish_params()
        self._set_state(P)
        # Timer em relógio de parede: o /clock do Gazebo sai a 10 Hz e um
        # timer em tempo simulado cairia de 33 para ~10 Hz em rajadas.
        self.create_timer(p['dt_s'], self._tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('admitância pronta (PARADO); habilite em '
                               f'{S_ENABLE}')

    # ── parâmetros ──────────────────────────────────────────────────────
    def _params(self) -> dict:
        return {k: self.get_parameter(k).value for k in DEFAULTS}

    def _make_ctrl(self, p) -> AdmittanceController:
        for k, (lo, hi) in RANGES.items():
            if not lo <= p[k] <= hi:          # NaN também cai aqui
                raise ValueError(f'{k}={p[k]} fora de [{lo}; {hi}]')
        if not math.isfinite(p['abort_force_n']) or p['abort_force_n'] > FORCE_ABORT_LIMIT_N or p['abort_force_n'] <= 0:
            raise ValueError(f'abort_force_n deve estar em (0; '
                             f'{FORCE_ABORT_LIMIT_N}] N')
        if p['mass_kg'] > 0 and arm_stiffness_limit(
                p['mass_kg'], p['damping_ns_m']) < ARM_STIFFNESS_MIN_N_M:
            raise ValueError('instável com o atraso da malha: aumente B ou reduza M')
        return AdmittanceController(
            mass_kg=p['mass_kg'], damping_ns_m=p['damping_ns_m'],
            stiffness_n_m=p['stiffness_n_m'], dt=p['dt_s'],
            deadband_n=p['deadband_n'], v_max_ms=p['v_max_ms'],
            max_excursion_m=p['max_excursion_m'],
            abort_force_n=p['abort_force_n'], stale_s=FORCE_STALE_S)

    def _on_params(self, params) -> SetParametersResult:
        new = self._params()
        new.update({q.name: q.value for q in params if q.name in DEFAULTS})
        try:
            if new['dt_s'] != self.p['dt_s']:
                raise ValueError('dt_s é fixo (relance o nó)')
            if new['force_source'] != self.p['force_source']:
                raise ValueError('force_source vem do launch')
            c = self._make_ctrl(new)
        except ValueError as e:
            return SetParametersResult(successful=False, reason=str(e))
        for k in ('x', 'x_ref', 'v', 'scale', '_p', '_n'):
            setattr(c, k, getattr(self.ctrl, k))
        self.ctrl, self.p = c, new
        self._publish_params()
        return SetParametersResult(successful=True)

    def _publish_params(self):
        self.pub_params.publish(String(data=json.dumps(self.p)))

    def _set_state(self, s):
        prev, self.state = self.state, s
        if s == AB and prev == J and self.q is not None:
            self._send(self.q, 0.3)   # cancela a trajetória do JTC em curso
        self.pub_state.publish(String(data=s))

    # ── entradas ────────────────────────────────────────────────────────
    def _on_js(self, msg: JointState):
        try:
            self.q = np.array([msg.position[msg.name.index(j)]
                               for j in ARM_JOINTS])
        except ValueError:
            pass

    def _on_tared(self, msg):
        # tara anterior a este nó (mensagem latched): orientação desconhecida
        ok = msg.data > self.t_start and self.q is not None
        self.tare_R = self.chain_ft.fk(self.q)[:3, :3] if ok else None

    def _tare_check(self):
        if self.p['force_source'] != 'real':
            return True, ''
        if self.tare_R is None:
            return False, 'tare a célula antes de habilitar'
        ang = float(np.linalg.norm(_log(self.chain_ft.fk(self.q)[:3, :3] @ self.tare_R.T)))
        if ang > TARE_MAX_TILT_RAD:
            return False, f'a célula girou {math.degrees(ang):.0f}° desde a tara: tare de novo'
        return True, ''

    def _clearance(self, q) -> float:
        """Menor folga [m] da célula ao piso e do cotovelo/punho à mesa (< 0 = invadiu)."""
        folga = float(self.chain_ft.fk(q)[2, 3]) - MIN_CELL_Z
        ps = [c.fk(q)[:3, 3] for c in self.chain_links]
        for p in (*ps, (ps[0] + ps[1]) / 2.0):
            if all(abs(p[i]) < TABLE_HALF_XY[i] + LINK_CLEARANCE_M for i in (0, 1)):
                folga = min(folga, float(p[2]) - LINK_CLEARANCE_M)
        return folga

    def _on_real(self, msg: String):
        try:
            self.real, self.real_t = json.loads(msg.data), time.monotonic()
        except ValueError:
            pass

    def _mirror(self) -> bool:
        """Último modo visto do rehab_mirror; antes disso, o do launch. Se o nó
        cair em MIRROR, continua MIRROR (e o status velho aborta)."""
        return (self.require_real if self.real is None
                else self.real.get('modo', 'MIRROR') == 'MIRROR')

    def _v_meas(self, x):
        """Velocidade medida do TCP: do robô real em MIRROR, senão do simulado."""
        src = 'sim'
        if self._mirror() and self.real and self.real.get('q_real'):
            src, x = 'real', self.chain.fk(self.real['q_real'])[:3, 3]
        prev, self.x_prev = self.x_prev, (src, np.array(x))
        if prev is None or prev[0] != src:
            return np.zeros(3)
        return (self.x_prev[1] - prev[1]) / self.p['dt_s']

    def _real_check(self):
        """(ok, motivo); sempre ok fora do modo mirror."""
        if not self._mirror():
            return True, 'ok'
        return real_ok(self.real, time.monotonic() - self.real_t)

    def _active_check(self):
        """(ok, motivo) para a admitância: robô real em dia E força confiável."""
        ok, why = self._real_check()
        if not ok:
            return False, f'robô real: {why}'
        if (self._mirror() and self.p['force_source'] != 'real'
                and (self.real or {}).get('ip') != SIM_ROBOT_IP):
            return False, (f"MIRROR com força '{self.p['force_source']}': o robô real só "
                           'segue a célula (relance com force_source:=real)')
        if self.count_publishers(T_FT) > 1:
            return False, f'mais de um nó publica {T_FT}: feche o outro launch'
        return True, 'ok'

    def _fk_z(self, q):
        return (float(self.chain.fk(q)[2, 3]), float(self.chain_ft.fk(q)[2, 3]))

    def _jog_reply(self, ok, motivo, dur=0.0, fim=False):
        self.pub_jog.publish(String(data=json.dumps(
            {'ok': ok, 'motivo': motivo, 'duracao_s': round(dur, 2), 'fim': fim})))
        # um call site por severidade: o rclpy levanta ValueError se a MESMA linha
        # de log mudar de nível (e isso derrubava o nó no timer)
        if ok:
            self.get_logger().info(f'jog: {motivo}')
        else:
            self.get_logger().warn(f'jog: {motivo}')
        return ok, motivo

    def _on_goto(self, msg: Float64MultiArray):
        """Posiciona as juntas (aba 5): só em PARADO, com o robô real em dia."""
        if len(msg.data) not in (6, 7):
            return self._jog_reply(False, 'o alvo precisa ter 6 juntas')
        pct = msg.data[6] if len(msg.data) == 7 else JOG_SPEED_DEFAULT_PCT
        self._goto(np.array(msg.data[:6], float), pct)

    def _goto(self, goal, pct):
        # em POSICIONANDO troca o alvo: o slider é contínuo, como no touch_pack
        if self.state not in (P, J):
            return self._jog_reply(False, f'só em PARADO (agora {self.state})')
        if not JOG_SPEED_MIN_PCT <= pct <= 100.0:
            return self._jog_reply(False, f'velocidade {pct}% fora de '
                                   f'[{JOG_SPEED_MIN_PCT:g}; 100]')
        if self.q is None:
            return self._jog_reply(False, 'sem /joint_states')
        ok_real, why = self._real_check()
        if not ok_real:
            return self._jog_reply(False, f'robô real: {why}')
        if self.pub_cmd.get_subscription_count() == 0:
            # sem assinante a trajetória se perde e o braço não sai do lugar
            return self._jog_reply(False, 'o controlador do braço '
                                   '(cr10_group_controller) ainda não está no ar')
        ok, motivo, dur = plan_jog(self.q, goal, self._fk_z, speed_pct=pct)
        if not ok:
            return self._jog_reply(False, motivo)
        self.jog_goal = goal
        self.jog_end = self._now() + dur + JOG_MARGIN_S
        self._send(goal, dur)
        if self.state != J:
            self._set_state(J)
        return self._jog_reply(True, motivo, dur)

    def _on_ref(self, msg: PointStamped):
        ref = np.array([msg.point.x, msg.point.y, msg.point.z])
        if self.state == A and np.all(np.isfinite(ref)):
            self.ctrl.set_reference(ref)

    def _on_ft(self, msg: WrenchStamped):
        f = np.array([msg.wrench.force.x, msg.wrench.force.y,
                      msg.wrench.force.z])
        m = np.array([msg.wrench.torque.x, msg.wrench.torque.y,
                      msg.wrench.torque.z])
        if msg.header.frame_id != 'load_cell_link' or self.q is None:
            return                       # não veio da célula (ou ainda sem pose)
        m = m - np.cross(self.r_grip, f)     # da célula para o centro da empunhadura
        f = self.chain_ft.fk(self.q)[:3, :3] @ f
        self.ft_acc.append(f)
        n = float(np.linalg.norm(f))
        self.f_peak = max(self.f_peak, n) if math.isfinite(n) else math.inf
        n = float(np.linalg.norm(m))
        self.m_peak = max(self.m_peak, n) if math.isfinite(n) else math.inf
        self.ft_t = time.monotonic()

    # ── serviços ────────────────────────────────────────────────────────
    def _now(self) -> float:
        """Tempo simulado (use_sim_time) com resolução fina: o /clock sai a
        10 Hz, então interpola com o relógio de parede entre as atualizações."""
        sim = self.get_clock().now().nanoseconds * 1e-9
        wall = time.monotonic()
        if sim != self._sim_last:
            self._sim_last, self._wall_last = sim, wall
        return self._sim_last + min(wall - self._wall_last, 0.1)

    def _begin_active(self):
        T = self.chain.fk(self.q)
        self.ctrl.reset(T[:3, 3])
        self.R0 = T[:3, :3].copy()
        self.q_cmd = self.q.copy()

    def _srv_enable(self, req, res):
        if self.state == J:
            res.success, res.message = False, 'posicionando as juntas: use PARAR para interromper'
        elif self.state == AB:
            res.success, res.message = False, 'ABORTADO: faça reset antes'
        elif not req.data:
            self._set_state(P)
            res.success, res.message = True, 'admitância parada'
        elif self.q is None:
            res.success, res.message = False, 'sem /joint_states'
        elif not self._active_check()[0]:
            res.success, res.message = False, self._active_check()[1]
        elif not self._tare_check()[0]:
            res.success, res.message = False, self._tare_check()[1]
        elif self._clearance(self.q) < 0.0:
            res.success, res.message = False, 'célula ou cotovelo baixo demais: posicione as juntas'
        elif self.ft_t is None or time.monotonic() - self.ft_t > FORCE_STALE_S:
            res.success, res.message = False, 'sem leitura de força'
        elif not np.linalg.norm(self.force_w) <= ENABLE_MAX_N:
            res.success, res.message = False, (
                f'força de {np.linalg.norm(self.force_w):.1f} N com o braço parado '
                f'(máx {ENABLE_MAX_N:g}): solte o mastro e tare')
        elif self.chain.fk(self.q)[2, 3] < MIN_TCP_Z:
            res.success, res.message = False, 'TCP abaixo do piso: posicione as juntas'
        else:
            self._begin_active()
            self._set_state(A)
            res.success, res.message = True, 'admitância ativa'
        return res

    def _srv_estop(self, _req, res):
        self._set_state(AB)
        res.success, res.message = True, 'e-stop: ABORTADO'
        return res

    def _srv_reset(self, _req, res):
        if self.state == AB:
            self._set_state(P)
            res.success, res.message = True, 'PARADO'
        else:
            res.success, res.message = False, 'não está ABORTADO'
        return res

    def _srv_home(self, _req, res):
        res.success, res.message = self._goto(np.array(home_q()), JOG_SPEED_DEFAULT_PCT)
        return res

    # ── comando ─────────────────────────────────────────────────────────
    def _send(self, q, t_s):
        m = JointTrajectory()
        m.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.time_from_start = Duration(sec=int(t_s),
                                      nanosec=int((t_s % 1) * 1e9))
        m.points = [pt]
        self.pub_cmd.publish(m)

    def _tick(self):
        now = self._now()
        # relógio de parede: com o /clock parado a idade em tempo simulado congela
        age = min(time.monotonic() - self.ft_t, 99.0) if self.ft_t else 99.0
        if self.ft_acc:      # média do tick: anti-aliasing 400 → 33 Hz, 15 ms de atraso
            self.force_w = np.mean(self.ft_acc, axis=0)
            self.ft_acc.clear()
        v_meas = np.zeros(3)
        x = np.full(3, np.nan)
        T = None
        if self.q is not None:
            T = self.chain.fk(self.q)
            x = T[:3, 3]
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = 'world'
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x
            (ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w) = _quat(T[:3, :3])
            self.pub_pose.publish(ps)
            v_meas = self._v_meas(x)
            # força axial da ferramenta: compressão (mão empurrando o mastro
            # para baixo, −z da ferramenta) é positiva
            self.f_axial = float(-(T[:3, :3].T @ self.force_w)[2])

        peak, self.f_peak = self.f_peak, 0.0
        mpeak, self.m_peak = self.m_peak, 0.0
        why = ''
        if self.state in (A, S_, L, J):
            ok, msg = self._real_check() if self.state == J else self._active_check()
            if not ok:
                why = msg
            elif peak > self.p['abort_force_n']:
                why = f'pico de força {peak:.1f} N'
            elif mpeak > self.p['abort_torque_nm']:
                why = f'pico de torque {mpeak:.1f} N·m'
            elif (self.state == A and self.q is not None and
                  np.max(np.abs(self.q - self.q_cmd)) > SIM_LAG_MAX_RAD):
                why = 'o braço simulado não segue o comando (outro nó o move?)'
            elif self.state != J and self._mirror() and age > FORCE_STALE_S:
                why = 'sem força em MIRROR (não retoma sozinho)'
        if why:
            self.get_logger().error(f'ABORTADO: {why}')
            self._set_state(AB)
        elif self.state == J:
            if self._now() >= self.jog_end:
                err = float(np.degrees(np.max(np.abs(self.q - self.jog_goal))))
                self._set_state(P)
                self._jog_reply(err < 2.0, f'fim do movimento; erro máximo {err:.1f}°',
                                fim=True)
        elif self.state in (A, S_, L):
            if self.q is None:
                pass
            else:
                self._active_step(age, v_meas)
        elif self.state == P and self.q is not None:
            self.ctrl.reset(x)   # fica pronto para habilitar sem salto
        self._publish_tick(now, x, age)

    def _active_step(self, age, v_meas):
        c = self.ctrl
        try:
            sp = c.step(self.force_w, age, v_meas)
        except AdmittanceAbort as e:
            self.get_logger().error(f'ABORTADO: {e}')
            self._set_state(AB)
            return
        if age > FORCE_STALE_S:
            if self.state != S_:
                self._set_state(S_)
            return                          # v=0, sem comando novo
        if sp[2] < MIN_TCP_Z:
            sp[2] = MIN_TCP_Z
            c.x[2] = MIN_TCP_Z
            c.v[2] = max(c.v[2], 0.0)
        q, res = self.chain.ik(self.q_cmd, sp, self.R0)
        lo = np.array([a for a, _ in self.chain.limits])
        hi = np.array([b for _, b in self.chain.limits])
        q = np.clip(q, lo, hi)
        res = float(np.linalg.norm(self.chain.fk(q)[:3, 3] - sp))
        # invadir a folga só é aceito se estiver saindo dela
        folga = self._clearance(q)
        baixo = folga < 0.0 and folga < self._clearance(self.q_cmd)
        if (baixo or res > IK_RESIDUAL_M
                or np.max(np.abs(q - self.q_cmd)) > MAX_DQ_TICK):
            self.get_logger().warn(
                f'LIMITE: resíduo IK {res * 1e3:.1f} mm, folga {folga * 1e3:.0f} mm',
                throttle_duration_sec=1.0)
            if self.state != L:
                self._set_state(L)
            c.reset(self.chain.fk(self.q)[:3, 3])
            self.q_cmd = self.q.copy()
            return
        self.q_cmd = q
        self._send(q, self.p['traj_lead_s'])
        if self.state != A:
            self._set_state(A)

    def _publish_tick(self, now, x, age):
        c = self.ctrl
        f = self.force_w
        v = [now, f[0], f[1], f[2], self.f_axial, x[0], x[1], x[2],
             c.x[0], c.x[1], c.x[2], c.v[0], c.v[1], c.v[2],
             c.x_ref[0], c.x_ref[1], c.x_ref[2],
             float(STATES.index(self.state)), age, c.scale]
        self.pub_tick.publish(Float64MultiArray(data=[float(a) for a in v]))


def main():
    rclpy.init()
    node = AdmittanceNode()
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
