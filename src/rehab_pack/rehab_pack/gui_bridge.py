"""Ponte ROS 2 do painel: nó rclpy numa thread própria, comunicação por filas.

A thread do Tk nunca chama o rclpy: lê `ticks`/`results` e as últimas
mensagens (`last`), e dispara serviços/publicações por métodos não bloqueantes.
Sem os outros nós no ar nada trava: serviço indisponível vira um resultado
imediato de falha.
"""
from __future__ import annotations

import collections
import json
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped, WrenchStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from . import constants as C

# Nó alvo de set_parameters (a fonte de força é só leitura: vem do launch).
NODE_ADMITTANCE = 'rehab_admittance'
_SETBOOL = (C.S_ENABLE, C.S_REAL_CONNECT, C.S_REAL_MODE)

_QOS_LATCHED = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                          history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class GuiBridge:
    def __init__(self):
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self.node = Node('rehab_gui')
        n = self.node
        self.ticks = collections.deque(maxlen=4000)   # list[float] por tick
        self.results = collections.deque()            # (rótulo, ok, mensagem)
        self.last = {}                                # chave -> valor mais recente
        self.seen = {}                                # chave -> time.monotonic()

        n.create_subscription(Float64MultiArray, C.T_TICK, self._on_tick, 10)
        n.create_subscription(String, C.T_STATE, self._latch('state'), _QOS_LATCHED)
        n.create_subscription(String, C.T_PARAMS, self._on_params, _QOS_LATCHED)
        n.create_subscription(String, C.T_SESSION_SUMMARY,
                              self._latch('summary'), _QOS_LATCHED)
        n.create_subscription(WrenchStamped, C.T_FT, self._touch('ft'), 10)
        n.create_subscription(JointState, C.T_JOINT_STATES, self._touch('joints'), 10)
        # modo mirror: status do rehab_mirror (JSON) e a FA7155 crua do ft_receiver
        # (best-effort: é assim que o touch_pack a publica)
        n.create_subscription(String, C.T_REAL_STATUS, self._latch('real'), 5)
        n.create_subscription(String, C.T_JOG_RESULT, self._latch('jog'), 10)
        n.create_subscription(WrenchStamped, C.T_REAL_WRENCH, self._touch('ftraw'),
                              qos_profile_sensor_data)

        self._pub_target = n.create_publisher(PointStamped, C.T_TARGET, 10)
        self._pub_ref = n.create_publisher(PointStamped, C.T_REFERENCE, 10)
        self._pub_event = n.create_publisher(String, C.T_EVENT, 10)
        self._pub_alive = n.create_publisher(Empty, C.T_GUI_ALIVE, 1)
        self._pub_goto = n.create_publisher(Float64MultiArray, C.T_JOG_GOTO, 10)
        self._pub_meta = n.create_publisher(String, C.T_SESSION_META, _QOS_LATCHED)

        self._trig = {s: n.create_client(Trigger, s) for s in (
            C.S_TARE, C.S_ESTOP, C.S_RESET, C.S_HOME,
            C.S_SESSION_START, C.S_SESSION_STOP)}
        self._bool = {s: n.create_client(SetBool, s) for s in _SETBOOL}
        self._setparams = {
            name: n.create_client(SetParameters, f'/{name}/set_parameters')
            for name in (NODE_ADMITTANCE, C.NODE_MIRROR)}

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(n)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    # ── thread do rclpy ─────────────────────────────────────────────────
    def _spin(self):
        while not self._stop.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.05)
            except Exception as e:  # nunca derruba a GUI
                self.results.append(('ros', False, f'erro interno: {e}'))
                if not rclpy.ok():      # contexto derrubado: sem tique, a tela fica SEM DADOS
                    return
                self._stop.wait(0.2)    # erro repetido não vira laço sem pausa

    def _touch(self, key):
        def cb(msg):
            self.seen[key] = time.monotonic()
            self.last[key] = msg
        return cb

    def _latch(self, key):
        def cb(msg):
            self.seen[key] = time.monotonic()
            self.last[key] = msg.data
        return cb

    def _on_tick(self, msg):
        self.seen['tick'] = time.monotonic()
        self.ticks.append(list(msg.data))

    def _on_params(self, msg):
        self.seen['params'] = time.monotonic()
        try:
            self.last['params'] = json.loads(msg.data)
        except ValueError:
            pass

    # ── API para a GUI (chamável da thread do Tk) ───────────────────────
    def age(self, key):
        """Segundos desde a última mensagem de `key` (inf se nunca)."""
        t = self.seen.get(key)
        return float('inf') if t is None else time.monotonic() - t

    def service_ready(self, name):
        cli = self._bool.get(name) or self._trig[name]
        return cli.service_is_ready()

    def call_trigger(self, name):
        cli = self._trig[name]
        if not cli.service_is_ready():
            self.results.append((name, False, 'serviço indisponível (nó não está no ar)'))
            return
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: self._done(name, f))

    def start_session(self):
        # tópico e serviço não têm ordem entre si: deixa a meta chegar antes
        threading.Timer(0.2, self.call_trigger, args=(C.S_SESSION_START,)).start()

    def call_enable(self, on: bool):
        self.call_bool(C.S_ENABLE, on)

    def call_bool(self, name, on: bool):
        cli = self._bool[name]
        if not cli.service_is_ready():
            self.results.append((name, False, 'serviço indisponível (nó não está no ar)'))
            return
        req = SetBool.Request()
        req.data = bool(on)
        fut = cli.call_async(req)
        fut.add_done_callback(lambda f: self._done(name, f))

    def connect_real(self, ip: str):
        """Grava o IP no rehab_mirror e só então conecta (serviços sem ordem entre si)."""
        self.set_params(C.NODE_MIRROR, {'robot_ip': ip},
                        then=lambda: self.call_bool(C.S_REAL_CONNECT, True))

    def _done(self, name, fut):
        try:
            r = fut.result()
            self.results.append((name, bool(r.success), r.message))
        except Exception as e:
            self.results.append((name, False, f'falha na chamada: {e}'))

    def set_params(self, node_name, values: dict, then=None):
        """set_parameters (double ou string) no nó `node_name`."""
        cli = self._setparams[node_name]
        if not cli.service_is_ready():
            self.results.append((f'params:{node_name}', False,
                                 'nó indisponível (não está no ar)'))
            return
        req = SetParameters.Request()
        for k, v in values.items():
            pv = ParameterValue()
            if isinstance(v, str):
                pv.type, pv.string_value = ParameterType.PARAMETER_STRING, v
            else:
                pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(v)
            req.parameters.append(Parameter(name=k, value=pv))
        fut = cli.call_async(req)

        def done(f, names=list(values)):
            try:
                bad = [f'{n}: {r.reason}' for n, r in zip(names, f.result().results)
                       if not r.successful]
                self.results.append((f'params:{node_name}', not bad,
                                     '; '.join(bad) or 'parâmetros aplicados'))
                if then and not bad:
                    then()
            except Exception as e:
                self.results.append((f'params:{node_name}', False, f'falha: {e}'))
        fut.add_done_callback(done)

    def _point(self, xy, z):
        m = PointStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.point.x, m.point.y, m.point.z = float(xy[0]), float(xy[1]), float(z)
        return m

    def publish_target(self, xy, z):
        self._pub_target.publish(self._point(xy, z))

    def publish_reference(self, xy, z):
        self._pub_ref.publish(self._point(xy, z))

    def publish_goto(self, q_rad, speed_pct):
        """Pede ao nó de admitância que posicione as 6 juntas (rad, URDF)."""
        self._pub_goto.publish(Float64MultiArray(data=[float(v) for v in q_rad] + [float(speed_pct)]))

    def publish_alive(self):
        self._pub_alive.publish(Empty())

    def publish_event(self, text):
        self._pub_event.publish(String(data=text))

    def publish_meta(self, meta: dict):
        self._pub_meta.publish(String(data=json.dumps(meta, ensure_ascii=False)))

    def shutdown(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self._executor.shutdown()
            self.node.destroy_node()
            if self._owns_rclpy and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
