"""Ligação ao CR10 real, sem ROS (testável com driver falso).

Mesmo sistema do touch_pack: `CR10RealDriver` (TCP 29999, feedback 30004),
IP em ~/.config/touch_pack/robot.json, reconexão com backoff e espelhamento
sim → real por ServoJ a 33 Hz a partir de /joint_states. O protocolo do robô
mora no driver do touch_pack e NÃO é copiado: o import é adiado para o
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

import numpy as np

from .constants import (GRIP_MASS_KG, HOME_Q_RAD, PLATE_MASS_KG,
                        REAL_STATUS_STALE_S, ROBOT_MODES_OK, SIM_ROBOT_IP,
                        SYNC_TOL_RUN_RAD, SYNC_TOL_START_RAD)

BACKOFF_S = (2.0, 5.0, 10.0, 30.0)
DEADBAND_RAD = 1.0e-5          # o mesmo SERVOJ_DEADBAND_RAD do touch_pack
DEFAULT_IP = '192.168.5.2'
# Carga no flange (detecção de colisão do CR10): célula 0,300 kg + placa + empunhadura,
# centro de massa pelas alturas da pilha mecânica do README.
CELL_MASS_KG = 0.300
PAYLOAD_KG = CELL_MASS_KG + PLATE_MASS_KG + GRIP_MASS_KG
PAYLOAD_COG_Z_M = (CELL_MASS_KG * 0.0187 + PLATE_MASS_KG * 0.0382
                   + GRIP_MASS_KG * 0.1142) / PAYLOAD_KG


def sync_error_rad(q_sim, q_real) -> float:
    """Maior diferença articular entre o braço simulado e o real."""
    return float(np.max(np.abs(np.asarray(q_sim, float)
                               - np.asarray(q_real, float))))


def real_ok(status: dict | None, age_s: float,
            stale_s: float = REAL_STATUS_STALE_S) -> tuple[bool, str]:
    """(ok, motivo) a partir do JSON de /rehab/real/status."""
    if status is None or age_s > stale_s:
        return False, 'sem status do robô real (rehab_mirror fora do ar?)'
    if not status.get('conectado'):
        return False, 'desconectado (IP inalcançável ou controlador desligado)'
    if status.get('estop'):
        return False, 'E-Stop do robô real acionado'
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


def save_robot_ip(ip: str, config_file: str) -> None:
    """Grava o IP que conectou no robot.json do touch_pack, sem tocar no resto."""
    try:
        with open(config_file) as fh:
            cfg = json.load(fh)
    except FileNotFoundError:
        cfg = {}
    if cfg.get('robot_ip') == ip:
        return
    cfg['robot_ip'] = ip
    os.makedirs(os.path.dirname(config_file), exist_ok=True)
    with open(config_file, 'w') as fh:
        json.dump(cfg, fh, indent=2, sort_keys=True)


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
    def prepare_servoj(self): pass
    def stop(self): pass

    def servo_j_urdf(self, q):
        self.q = np.array(q, float)
        self.cmds.append(self.q.copy())

    def read_joints_urdf_latest(self):
        return self.q.copy()

    def stop_motion(self):
        self.stopped += 1


def default_factory(ip: str, period_s: float, feedback_port: int | None = None,
                    sim_offset_j1_deg: float = 0.0):
    """Driver real do touch_pack (import adiado) ou o simulado."""
    if ip == SIM_ROBOT_IP:
        return SimulatedDriver(offset_j1_deg=sim_offset_j1_deg)
    from touch_pack.real_driver import (CR10RealDriver, CR10RealDriverConfig,
                                        FEEDBACK_PORT)
    return CR10RealDriver(ip=ip, config=CR10RealDriverConfig(
        payload_kg=PAYLOAD_KG, payload_cog_m=(0.0, 0.0, PAYLOAD_COG_Z_M),
        servoj_period_s=period_s, feedback_port=feedback_port or FEEDBACK_PORT))


class RealLink:
    """Conexão + ServoJ. Thread-safety fica com quem chama (o nó usa um lock)."""

    def __init__(self, ip: str, period_s: float = 0.030, factory=default_factory,
                 log=None):
        self.ip, self.period_s, self._factory = ip, period_s, factory
        self.log = log or (lambda m: None)
        self.drv = None
        self._last_q = None
        self._attempt = 0
        self.last_error = ''

    @property
    def connected(self) -> bool:
        return self.drv is not None

    @property
    def estop(self) -> bool:
        return bool(self.drv is not None
                    and getattr(self.drv, 'estop_engaged', False))

    def backoff_s(self) -> float:
        return BACKOFF_S[min(self._attempt, len(BACKOFF_S) - 1)]

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
        self.drv, self._attempt, self.last_error = drv, 0, ''
        self.log(f'CR10 conectado em {self.ip}.')

    def fail(self, err: str) -> None:
        self.last_error = err
        self.log(f'CR10 em {self.ip} indisponível ({err}) — nova tentativa '
                 f'em {self.backoff_s():.0f}s.')
        self._attempt += 1

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
        self.log(f'Conexão com o CR10 perdida ({why}) — reconectando.')
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

    def stream(self, q_sim) -> bool:
        """Um ponto de ServoJ. False se estacionário (banda morta) ou sem robô."""
        if self.drv is None:
            return False
        q = np.asarray(q_sim, float)
        if (self._last_q is not None
                and float(np.max(np.abs(q - self._last_q))) < DEADBAND_RAD):
            return False
        try:
            try:
                self.drv.servo_j_urdf(q)
            except Exception:
                if self._last_q is not None:
                    raise      # no meio do movimento: limpar alarme e seguir é inseguro
                self.drv.prepare_servoj()
                self.drv.servo_j_urdf(q)
            self._last_q = q
            return True
        except Exception as exc:
            self.drop(exc)
            return False

    def halt(self) -> None:
        """Para o movimento e limpa a fila, sem desabilitar o robô."""
        if self.drv is not None:
            try:
                self.drv.stop_motion()
            except Exception as exc:
                self.log(f'stop_motion falhou: {exc}')

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
