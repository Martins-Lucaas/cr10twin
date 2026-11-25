"""Fonte de força: escolhe sim | real, tara e reestampa.

Publica /rehab/ft/wrench a cada mensagem recebida, com o relógio do nó.
O sinal físico é "força da mão sobre o mastro" (compressão positiva).
"""
from __future__ import annotations

import json
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

from .constants import (FORCE_STALE_S, S_TARE, STATES, T_FT, T_FT_CONFIG, T_REAL_TARE_RESULT,
                        T_REAL_TARED, T_REAL_WRENCH, T_SIM_WRENCH, T_STATE, T_TARED)

TARE_WINDOW_S = 0.5
TARE_MIN_N = 10
TARE_STATES = (None, STATES[0], STATES[4])   # PARADO, ABORTADO ou sem admitância
TARE_STABLE_N = 0.50          # deriva máxima na janela: o critério do ft_receiver
# fonte -> (tópico de origem, frame de saída)
SOURCES = {'sim': (T_SIM_WRENCH, 'load_cell_link'),
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

    def begin(self) -> list:
        """Abre a janela e a devolve: é o que `finish` fecha."""
        self._acc = []
        return self._acc

    def feed(self, w):
        if self._acc is not None:
            self._acc.append(np.asarray(w, float))

    @property
    def running(self):
        return self._acc is not None

    def cancel(self):
        self._acc = None

    def finish(self, janela=None) -> str:
        """Fecha a janela. '' se tarou; senão o motivo, com o offset intacto.
        `janela`: só fecha se ainda for essa (cancelada e reaberta é outra)."""
        if janela is not None and janela is not self._acc:
            acc = None
        else:
            acc, self._acc = self._acc, None
        if acc is None:
            return 'tara cancelada: o braço saiu de PARADO ou a célula mudou o zero'
        if len(acc) < TARE_MIN_N:
            return f'poucas amostras de força em {TARE_WINDOW_S:g} s'
        acc = np.array(acc)
        half = len(acc) // 2
        drift = float(np.max(np.abs(np.median(acc[half:, :3], axis=0)
                                    - np.median(acc[:half, :3], axis=0))))
        if not drift <= TARE_STABLE_N:       # NaN também recusa
            return f'força variando ({drift:.2f} N): solte o mastro e repita'
        self.offset = np.mean(acc, axis=0)
        return ''

    def apply(self, w) -> np.ndarray:
        return np.asarray(w, float) - self.offset


def rot_z6(yaw_rad: float) -> np.ndarray:
    """Gira força e torque em torno do eixo z da célula."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    r = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return np.kron(np.eye(2), r)


class ForceSource(Node):
    def __init__(self):
        super().__init__('rehab_force_source')
        ro = ParameterDescriptor(read_only=True)
        self.declare_parameter('source', 'real', ro)
        # só leitura: mudar depois da tara deixaria o zero com sinal/eixo trocado
        self.declare_parameter('sim_sign', 1.0, ro)
        self.declare_parameter('real_sign', 1.0, ro)
        self.declare_parameter('real_yaw_deg', 0.0, ro)
        src = self.get_parameter('source').value
        if src not in SOURCES:
            raise ValueError(f'source={src!r}: use sim ou real')
        for k in ('sim_sign', 'real_sign'):      # 0 cegaria o aborto; 0,5 dobraria o limite
            if self.get_parameter(k).value not in (1.0, -1.0):
                raise ValueError(f'{k} deve ser 1 ou -1')
        if not math.isfinite(self.get_parameter('real_yaw_deg').value):
            raise ValueError('real_yaw_deg não finito')
        topic, self._frame = SOURCES[src]
        self._src = src
        yaw_deg = self.get_parameter('real_yaw_deg').value if src == 'real' else 0.0
        self._map = self._sign() * rot_z6(math.radians(yaw_deg))
        self._tare = Tare()
        self._lock = threading.Lock()
        self._tared = False          # há tara anunciada em T_TARED
        self._last_t = 0.0           # chegada da última amostra
        self._pub = self.create_publisher(WrenchStamped, T_FT, 50)
        grp = ReentrantCallbackGroup()
        self.create_subscription(WrenchStamped, topic, self._on_wrench,
                                 WRENCH_QOS, callback_group=grp)
        self._pub_tared = self.create_publisher(
            Float64, T_TARED,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._pub_tared.publish(Float64(data=0.0))
        # para o meta da sessão; os parâmetros são só leitura, uma vez basta
        self._pub_config = self.create_publisher(
            String, T_FT_CONFIG,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._pub_config.publish(String(data=json.dumps(
            {'ft_sign': self._sign(), 'ft_yaw_deg': yaw_deg})))
        self._state = None
        self.create_subscription(
            String, T_STATE, self._on_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=grp)
        if src == 'real':            # outro zero no ft_receiver: a nossa tara não vale mais
            self.create_subscription(
                String, T_REAL_TARE_RESULT,
                lambda m: m.data.startswith('ok') and self._invalidate('o ft_receiver re-zerou'),
                10, callback_group=grp)
            self._ft_tared = None
            self.create_subscription(Bool, T_REAL_TARED, self._on_ft_tared,
                                     10, callback_group=grp)
        # parede: com o /clock parado o silêncio passaria despercebido
        self.create_timer(0.1, self._watch, callback_group=grp,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_service(Trigger, S_TARE, self._on_tare,
                            callback_group=grp)
        self.get_logger().info(f'fonte={src} topico={topic} frame={self._frame}')

    def _sign(self) -> float:
        return {'sim': self.get_parameter('sim_sign').value,
                'real': self.get_parameter('real_sign').value}[self._src]

    def _on_state(self, msg):
        with self._lock:
            self._state = msg.data
            if self._state not in TARE_STATES:   # habilitaram no meio da janela
                self._tare.cancel()

    def _invalidate(self, why):
        """A tara deixa de valer: quem a usa recebe T_TARED = 0."""
        with self._lock:
            self._tare.cancel()
            tinha, self._tared = self._tared, False
            if tinha:
                self._pub_tared.publish(Float64(data=0.0))
        if tinha:
            self.get_logger().warning(f'tara invalidada: {why}')

    def _on_ft_tared(self, msg):
        """False = o zero de hardware (ou a partida) apagou a tara do ft_receiver, e ele
        fará outra sozinho: a nossa não vale em nenhum dos dois momentos."""
        antes, self._ft_tared = self._ft_tared, msg.data
        if not msg.data or antes is False:
            self._invalidate('o ft_receiver mudou o zero da célula')

    def _watch(self):
        if self._tared and time.monotonic() - self._last_t > FORCE_STALE_S:
            self._invalidate('célula muda (o ft_receiver pode ter reiniciado)')

    def _on_wrench(self, msg: WrenchStamped):
        if time.monotonic() - self._last_t > FORCE_STALE_S:
            self._invalidate('célula voltou depois de muda')
        self._last_t = time.monotonic()
        f, t = msg.wrench.force, msg.wrench.torque
        w = self._map @ np.array([f.x, f.y, f.z, t.x, t.y, t.z])
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
            if self._state not in TARE_STATES:
                res.success, res.message = False, f'tara só em PARADO (agora {self._state})'
                return res
            if self._tare.running:
                res.success, res.message = False, 'já há uma tara em andamento'
                return res
            janela = self._tare.begin()
            # enquanto mede não há tara: a admitância não habilita no meio da janela
            self._tared = False
            self._pub_tared.publish(Float64(data=0.0))
        # espera em tempo de relógio do nó (sim ou parede, conforme use_sim_time)
        t0 = self.get_clock().now()
        wall0 = time.monotonic()
        while ((self.get_clock().now() - t0).nanoseconds * 1e-9 < TARE_WINDOW_S
               and time.monotonic() - wall0 < 10 * TARE_WINDOW_S):
            time.sleep(0.02)
        with self._lock:
            err = self._tare.finish(janela)
            if not err:
                self._tared = True
                self._pub_tared.publish(Float64(data=time.monotonic()))
        res.success = not err
        res.message = err or 'tara feita'
        return res


def main():
    # trava touch_pack × rehab_pack
    from .pack_lock import acquire_or_exit
    acquire_or_exit('rehab_pack')
    rclpy.init()
    node = ForceSource()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():               # no Ctrl-C o contexto já caiu: sai sem traceback
            raise
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
