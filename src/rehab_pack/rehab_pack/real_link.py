"""Ligação ao CR10 real, sem ROS (testável com driver falso).

Mesmo sistema do touch_pack: `CR10RealDriver` (TCP 29999, feedback 30004),
IP em ~/.config/touch_pack/robot.json, conexão só a pedido e espelhamento
sim → real por ServoJ a 33 Hz a partir de /joint_states. O protocolo do robô
mora em `real_driver` (cópia própria do rehab): o import é adiado para o
momento de conectar, então estas funções (e o nó de admitância) não exigem o
touch_pack.

Decisão de segurança (`real_ok`): o ServoJ só corre com o robô conectado, sem
E-Stop e com sim e real em sincronia; o nó de admitância recusa habilitar, e
aborta se perder qualquer uma dessas condições.
"""
from __future__ import annotations

import json
import math
import os
import time

import numpy as np

from .constants import (ARM_PUBS, GRIP_MASS_KG, HOME_Q_RAD, JOG_MAX_DEG_S, JS_STALE_S, PLATE_MASS_KG,
                        REAL_STATUS_STALE_S, ROBOT_MODES_OK, SERVO_STEP_SLACK, SIM_ROBOT_IP,
                        SYNC_TOL_RUN_RAD, SYNC_TOL_START_RAD, T_ARM_CMD)

DEADBAND_RAD = 1.0e-5          # o mesmo SERVOJ_DEADBAND_RAD do touch_pack
DEFAULT_IP = '192.168.5.2'
# Carga no flange (detecção de colisão do CR10): flange + célula 0,300 kg + placa + empunhadura,
# centro de massa pelas alturas da pilha mecânica do README.
CELL_MASS_KG = 0.300
FLANGE_MASS_KG = 0.0518         # flange fixo da FA7155 (joystick_tool.urdf)
PAYLOAD_KG = FLANGE_MASS_KG + CELL_MASS_KG + PLATE_MASS_KG + GRIP_MASS_KG
PAYLOAD_COG_Z_M = (FLANGE_MASS_KG * 0.00295 + CELL_MASS_KG * 0.0197 + PLATE_MASS_KG * 0.0382
                   + GRIP_MASS_KG * 0.1067) / PAYLOAD_KG


def sync_error_rad(q_sim, q_real) -> float:
    """Maior diferença articular entre o braço simulado e o real."""
    return float(np.max(np.abs(np.asarray(q_sim, float)
                               - np.asarray(q_real, float))))


class StepTooLarge(RuntimeError):
    """O próximo ServoJ pediria um tranco: quem chama para e espera o operador."""


def is_feedback_timeout(exc) -> bool:
    """Leitura de feedback atrasada, com a conexão mantida pelo driver?"""
    try:
        from .real_driver import CR10FeedbackTimeout
    except ImportError:
        return False
    return isinstance(exc, CR10FeedbackTimeout)


def arm_intruders(node) -> list:
    """Publicadores de T_ARM_CMD que não são do rehab (ou repetidos)."""
    pubs = [p.node_name for p in node.get_publishers_info_by_topic(T_ARM_CMD)]
    return [p for i, p in enumerate(pubs) if p not in ARM_PUBS or p in pubs[:i]]


def real_ok(status: dict | None, age_s: float,
            stale_s: float = REAL_STATUS_STALE_S) -> tuple[bool, str]:
    """(ok, motivo) a partir do JSON de /rehab/real/status."""
    if status is None or age_s > stale_s:
        return False, 'sem status do robô real (rehab_mirror fora do ar?)'
    if status.get('parada'):             # antes de tudo: o braço pode estar andando
        return False, status['parada']
    if not status.get('conectado'):
        return False, 'desconectado (IP inalcançável ou controlador desligado)'
    if status.get('estop'):
        return False, 'E-Stop do robô real acionado'
    if status.get('travado'):
        return False, status['travado']
    modo = status.get('robot_mode')
    if modo is not None and modo not in ROBOT_MODES_OK:
        return False, (f'RobotMode {modo} (E-Stop físico, alarme, colisão ou pausa): '
                       'resolva no robô, depois Desconectar e Conectar')
    if not status.get('sync_ok'):
        return False, (f"sim e real diferem {status.get('max_dq_deg', 0.0):.1f}° "
                       f'(máx {math.degrees(SYNC_TOL_START_RAD):.0f}° para '
                       'armar): aguarde a sincronização')
    return True, 'ok'


def robot_ip(param: str, config_file: str | None = None) -> str:
    """IP do parâmetro; senão o do robot.json do touch_pack; senão o padrão."""
    ip = (param or '').strip()
    if ip:
        return ip
    if config_file is None:
        try:
            from touch_pack.constants import ROBOT_CONFIG_FILE as config_file
        except Exception:
            config_file = os.path.expanduser('~/.config/touch_pack/robot.json')
    try:
        with open(config_file) as fh:
            ip = str(json.load(fh).get('robot_ip', '')).strip()
    except (OSError, ValueError, AttributeError):
        ip = ''
    return ip or DEFAULT_IP


class SimulatedDriver:
    """Robô 'real' de mentira para ensaiar o fluxo (robot_ip:='sim'): segue o
    último ServoJ sem atraso. Não substitui o teste com hardware."""

    estop_engaged = False
    last_robot_mode = 5

    def __init__(self, q0=HOME_Q_RAD, offset_j1_deg: float = 0.0):
        self.q = np.array(q0, float)
        self.q[0] += np.radians(offset_j1_deg)
        self.cmds: list[np.ndarray] = []
        self.stopped = 0

    def connect(self): pass
    def enable(self): pass
    def close(self): pass
    def stop(self): pass

    def servo_j_urdf(self, q):
        self.q = np.array(q, float)
        self.cmds.append(self.q.copy())

    def read_joints_urdf_latest(self):
        return self.q.copy()

    def stop_motion(self):
        self.stopped += 1

    def emergency_stop(self):
        self.estop_engaged = True
        return True

    def release_emergency_stop(self):
        self.estop_engaged = False


def default_factory(ip: str, period_s: float):
    """Driver real do rehab_pack (import adiado) ou o simulado."""
    if ip == SIM_ROBOT_IP:
        return SimulatedDriver()
    from .real_driver import CR10RealDriver, CR10RealDriverConfig
    return CR10RealDriver(ip=ip, config=CR10RealDriverConfig(
        payload_kg=PAYLOAD_KG, payload_cog_m=(0.0, 0.0, PAYLOAD_COG_Z_M),
        servoj_period_s=period_s))


class RealLink:
    """Conexão + ServoJ. Thread-safety fica com quem chama (o nó usa um lock)."""

    def __init__(self, ip: str, period_s: float = 0.030, factory=default_factory,
                 log=None):
        self.ip, self.period_s, self._factory = ip, period_s, factory
        self.log = log or (lambda m: None)
        self.drv = None
        self._last_q, self._last_t = None, 0.0
        self._folga = 0.0            # atraso do laço ainda por recuperar [rad]
        self.last_error = ''
        self.parada = ''             # Stop não confirmado: o que fazer; até reconectar
        self.rearmar = False         # o E-STOP de software é nosso: o Conectar o solta
        self.max_step_rad = math.radians(JOG_MAX_DEG_S) * period_s * SERVO_STEP_SLACK

    @property
    def connected(self) -> bool:
        return self.drv is not None

    @property
    def estop(self) -> bool:
        return bool(self.drv is not None
                    and getattr(self.drv, 'estop_engaged', False))

    def open_driver(self):
        """Cria, conecta e habilita o driver; pode BLOQUEAR por segundos (com o
        controlador em modo LOCAL, enable() só levanta depois de ~16 s). Não
        toca em `self.drv`, então o chamador não precisa segurar um lock durante
        a espera. Em falha fecha o que abriu (senão sobram sockets e threads
        até esgotar as sessões do dashboard). Devolve (driver, '') ou (None, erro)."""
        drv = None
        try:
            drv = self._factory(self.ip, self.period_s)
            drv.connect()
            if self.rearmar:         # EmergencyStop(0) + ClearError + enable
                drv.release_emergency_stop()
            else:
                drv.enable()
            return drv, ''
        except Exception as exc:
            if drv is not None:
                try:
                    drv.close()
                except Exception:
                    pass
            return None, str(exc)

    def attach(self, drv) -> None:
        self.drv, self.last_error = drv, ''
        self.parada, self.rearmar = '', False
        self.log(f'CR10 conectado em {self.ip}.')

    def fail(self, err: str) -> None:
        self.last_error = err
        self.log(f'CR10 em {self.ip} indisponível ({err}) — sem nova '
                 'tentativa automática: conecte de novo.')

    def connect_once(self) -> bool:
        """Uma tentativa completa (bloqueante), para quem não precisa de lock."""
        drv, err = self.open_driver()
        if drv is None:
            self.fail(err)
            return False
        self.attach(drv)
        return True

    @property
    def robot_mode(self):
        return getattr(self.drv, 'last_robot_mode', None)

    def drop(self, why, only=None) -> None:
        """`only`: ignora se a conexão já não é a deste driver."""
        if only is not None and only is not self.drv:
            return
        self.last_error = f'conexão perdida: {why}'
        self.log(f'Conexão com o CR10 perdida ({why}) — conecte de novo.')
        drv, self.drv, self._last_q = self.drv, None, None
        if drv is not None:
            try:
                drv.close()
            except Exception:
                pass

    def read_q(self):
        """Juntas reais (URDF) ou None. Falha de leitura derruba a conexão."""
        if self.drv is None:
            return None
        try:
            return np.asarray(self.drv.read_joints_urdf_latest(), float)
        except Exception as exc:
            self.drop(exc)
            return None

    def reset_stream(self) -> None:
        self._last_q = None

    def stream(self, q_sim, q_real=None) -> bool:
        """Um ponto de ServoJ. False se estacionário (banda morta) ou sem robô.

        StepTooLarge se o ponto salta do último enviado (no 1º, da pose real)."""
        if self.drv is None:
            return False
        q = np.asarray(q_sim, float)
        ref = self._last_q if self._last_q is not None else q_real
        passo = 0.0 if ref is None else sync_error_rad(q, ref)
        # atraso do laço: o alvo pode estar mais longe o que o jog anda nesse tempo (até
        # JS_STALE_S), mas cada ponto leva no máximo max_step_rad. A folga some a um passo
        # de jog por ponto: alvo que segue longe trava.
        agora, v = time.monotonic(), math.radians(JOG_MAX_DEG_S)
        if self._last_q is None:
            self._folga = 0.0
        else:
            extra = min(agora - self._last_t, JS_STALE_S) - self.period_s
            self._folga = min(max(self._folga - v * self.period_s, 0.0) + v * max(extra, 0.0),
                              v * max(JS_STALE_S - self.period_s, 0.0))
        teto = self.max_step_rad + self._folga + 1e-9
        if not np.all(np.isfinite(q)) or not passo <= teto:   # NaN reprova
            raise StepTooLarge(f'salto de {math.degrees(passo):.1f}° num ServoJ '
                               f'(máx {math.degrees(teto):.1f}°)')
        self._last_t = agora
        if (self._last_q is not None
                and float(np.max(np.abs(q - self._last_q))) < DEADBAND_RAD):
            return False
        if passo > self.max_step_rad:
            q = ref + (q - np.asarray(ref, float)) * (self.max_step_rad / passo)
        try:
            self.drv.servo_j_urdf(q)
            self._last_q = q
            return True
        except Exception as exc:
            self.drop(exc)     # nunca limpa alarme sozinho: só o enable() do Conectar
            return False

    def halt(self) -> bool:
        """Para o movimento e limpa a fila, sem desabilitar o robô. Stop não confirmado
        vira E-STOP. False: nem o E-STOP foi confirmado."""
        drv = self.drv
        if drv is None:
            return True
        try:
            drv.stop_motion()
            return True
        except Exception as exc:
            self.log(f'stop_motion falhou: {exc}')
        self.rearmar = True
        try:
            ok = bool(drv.emergency_stop())
        except Exception as exc:
            self.log(f'E-STOP falhou: {exc}')
            ok = False
        self.parada = (
            'Stop não confirmado: E-STOP do robô acionado. Desconectar e Conectar para rearmar'
            if ok else 'PARADA NÃO CONFIRMADA: o braço pode estar andando, use o E-Stop físico')
        self.log(self.parada)
        return ok

    def disconnect(self) -> None:
        """Pedido do operador: Stop + DisableRobot e fecha, como o touch_pack."""
        drv, self.drv, self._last_q = self.drv, None, None
        if drv is None:
            return
        for fn in (drv.stop, drv.close):
            try:
                fn()
            except Exception as exc:
                self.log(f'desconexão: {exc}')
        self.log(f'CR10 em {self.ip} desconectado.')

    def close(self) -> None:
        if self.drv is not None:
            self.drop('encerrando')

    def tolerance(self, streaming: bool) -> float:
        return SYNC_TOL_RUN_RAD if streaming else SYNC_TOL_START_RAD
