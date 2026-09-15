"""gui_robot.py — o CR10 real: conexão, heartbeat, modo MIRROR e E-STOP.

O braço real é a única parte da GUI que pode machucar alguém, e tudo aqui
gira em torno de saber, a cada instante, se ainda há alguém do outro lado do
cabo.

  * o heartbeat existe porque um controlador que caiu não recusa comando —
    ele aceita e não move, ou pior, acumula. Sem o batimento, a GUI seguiria
    publicando ServoJ contra o vazio e o operador só descobriria pelo braço
    parado;
  * o MIRROR replica no braço real o que o operador faz no simulado. Cada
    comando passa pelo dedup do último alvo: sem ele, o poll a 20 Hz
    reenviaria a mesma pose e o controlador ficaria sem janela para executar;
  * o E-STOP é o caminho que NÃO pode depender de nada — nem do Tk
    responsivo, nem do ROS entregando, nem do modo atual. Ele mora junto com
    a conexão porque é a conexão que ele precisa alcançar.

Recortado de `palpation_gui.py` — os métodos operam sobre `self` como antes.
"""
from __future__ import annotations

import logging
from typing import Any

# O driver do CR10 é import OPCIONAL: a GUI tem de abrir numa máquina sem ele
# (simulação pura). Sem o fallback, o `except CR10RealDriverError` viraria
# NameError e engoliria o erro real da conexão.
try:
    from .real_driver import (
        CR10RealDriver, CR10RealDriverConfig, CR10RealDriverError,
    )
    _REAL_DRIVER_OK = True
except Exception:  # pragma: no cover
    CR10RealDriver: Any = None
    CR10RealDriverConfig: Any = None
    CR10RealDriverError = Exception
    _REAL_DRIVER_OK = False

# `urdf_to_dobot` vem do mesmo import opcional do driver: converter a pose só
# faz sentido com o braço real no ar, e a GUI abre sem ele.
try:
    from .kinematics import urdf_to_dobot as _urdf_to_dobot
except Exception:  # pragma: no cover
    _urdf_to_dobot = None

import json
import math
import numpy as np
import os
import re
import threading
import time
import tkinter as tk
from .constants import ARM_JOINTS, ROBOT_CONFIG_FILE
from .gui_constants import ARM_LIMITS_DEG
from .ui_helpers import (
    BTN_NEUTRAL,
    DANGER,
    OK,
    PRIMARY,
    TEXT,
    TEXT_DIM,
    WARN,
    _shade,
)
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


from .gui_constants import ROBOT_CONFIG_DEFAULTS
from .gui_constants import (
    SPEED_FACTOR_MIN, SPEED_FACTOR_MAX, SPEED_FACTOR_DEFAULT,
)

log = logging.getLogger('touch_pack.palpation_gui')   # mesmo canal do host

# Abaixo disto o alvo é o mesmo ponto: o feedback das juntas do CR10 é
# quantizado em 1e-5 rad, e reenviar um ServoJ que não move nada só tira do
# controlador a janela para executar o anterior.
SERVOJ_DEADBAND_RAD = 1.0e-5


class RobotMixin:
    """gui_robot.py — o CR10 real: conexão, heartbeat, modo MIRROR e E-STOP."""

    # Mirror MovJ (MIRROR mode — braço real segue os sliders)
    def _mirror_movj_debounced(self, positions_rad: list[float]) -> None:
        """Agenda MovJ ao braço real com debounce de 80 ms."""
        q_new = np.asarray(positions_rad, dtype=np.float64)
        with self._mirror_timer_lock:
            if self._mirror_timer is not None:
                self._mirror_timer.cancel()
            self._mirror_timer = threading.Timer(
                0.08, self._mirror_movj_send, args=[q_new.tolist()])
            self._mirror_timer.daemon = True
            self._mirror_timer.start()
    def _mirror_movj_send(self, positions_rad: list[float]) -> None:
        """Converte URDF→DOBOT, define SpeedFactor e envia MovJ ao braço real."""
        try:
            q_dobot_rad = _urdf_to_dobot(
                np.array(positions_rad, dtype=np.float64))
            q_dobot_deg = [math.degrees(float(v)) for v in q_dobot_rad]
            try:
                speed_pct = int(max(SPEED_FACTOR_MIN,
                                    min(SPEED_FACTOR_MAX,
                                        self.speed_factor_var.get())))
            except (ValueError, tk.TclError):
                speed_pct = SPEED_FACTOR_DEFAULT
            with self._real_lock:
                drv = self._real_driver
                if (drv is None or not self._robot_connected
                        or self._robot_mode != 'MIRROR'):
                    return
                # Race guard: o timer de debounce pode disparar após a fase
                # mudar para HOME/CONTACT/etc. — MovJ durante ServoJ causa solavanco.
                with self._lock:
                    if self._latest_phase not in ('IDLE', 'DONE', 'ABORTED'):
                        return
                drv._send_dash(f'SpeedFactor({speed_pct})')
                drv.mov_j_joint_deg(q_dobot_deg)
                self._last_robot_cmd_t = time.monotonic()
                if self._drag_enabled:
                    self._drag_enabled = False
                    self.root.after(0, self._update_drag_btn_auto, False)
            self._mirror_last_target = np.asarray(
                positions_rad, dtype=np.float64)
            # Abre a janela de follow real→sim: o poll loop passa a espelhar
            # o feedback do braço até o MovJ assentar (ou 15 s de teto).
            self._follow_still_ticks = 0
            self._follow_moved = False
            self._mirror_follow_until = time.monotonic() + 15.0
        except CR10RealDriverError as exc:
            self.get_logger().warning(f'Mirror MovJ falhou: {exc}')
    def _mirror_poll_loop(self) -> None:
        """Envia ServoJ ao braço real a 33 Hz APENAS durante palpação ativa."""
        _diag_count = 0
        _drag_read_failures = 0
        _PERIOD = 0.030   # 33 Hz
        _t_next = time.monotonic() + _PERIOD
        while not self._stop_event.is_set():
            # Drift-compensated sleep: corrige jitter acumulado do SO.
            # wait(0.030) pode demorar 31–40 ms no Linux com carga, causando
            # descontinuidades no ServoJ que levam a sons e solavancos no real.
            now = time.monotonic()
            sleep_s = max(0.0, _t_next - now)
            self._stop_event.wait(sleep_s)
            _t_next += _PERIOD
            # Evita recuperar múltiplos ticks atrasados de uma vez.
            if _t_next < time.monotonic():
                _t_next = time.monotonic() + _PERIOD
            if (self._robot_mode != 'MIRROR' or not self._robot_connected
                    or self._real_driver is None or _urdf_to_dobot is None):
                continue
            # Drag teach ativo → lê posição real e espelha para o Gazebo.
            if self._drag_enabled:
                drv = self._real_driver
                if drv is None or not self._robot_connected:
                    continue
                try:
                    q_urdf = drv.read_joints_urdf_latest()
                    _drag_read_failures = 0  # leitura válida — reset contador
                    now = time.monotonic()
                    # Guard: firmware zero-blip — ignorar mas não desativar drag.
                    if np.linalg.norm(q_urdf) < 0.05:
                        continue
                    # Guard: salto fisicamente impossível (>60° em 30 ms).
                    _last = self._drag_last_valid_q
                    _last_t = self._drag_last_t
                    if (_last is not None
                            and np.max(np.abs(q_urdf - _last)) > math.radians(60)):
                        continue
                    # Velocidade por diferença finita para interpolação suave no JTC.
                    if _last is not None and _last_t is not None:
                        dt = min(max(now - _last_t, 0.005), 0.2)
                        vel = (q_urdf - _last) / dt
                        vel = np.clip(vel, -2.5, 2.5)
                    else:
                        vel = np.zeros(6)
                    self._drag_last_valid_q = q_urdf
                    self._drag_last_t = now
                    msg = JointTrajectory()
                    msg.joint_names = ARM_JOINTS
                    pt = JointTrajectoryPoint()
                    pt.positions = [float(v) for v in q_urdf]
                    pt.velocities = [float(v) for v in vel]
                    pt.time_from_start = Duration(sec=0, nanosec=60_000_000)
                    msg.points = [pt]
                    self._arm_pub.publish(msg)
                    # Espelha posição real → sliders da GUI (Tk-safe via after).
                    self.root.after(0, self._update_sliders_from_q,
                                    q_urdf.copy())
                except CR10RealDriverError as exc:
                    # Leitura inválida (buffer desalinhado no início, transitório) —
                    # pular este tick. Só desativar drag após 5 falhas consecutivas.
                    _drag_read_failures += 1
                    if _drag_read_failures >= 5:
                        self.get_logger().warning(
                            f'[DRAG] {_drag_read_failures} falhas consecutivas — '
                            f'drag desativado: {exc}')
                        self._drag_enabled = False
                        _drag_read_failures = 0
                        self.root.after(0, self._update_drag_btn_auto, False)
                    else:
                        self.get_logger().debug(
                            f'[DRAG] leitura inválida (tentativa {_drag_read_failures}/5), '
                            f'aguardando alinhamento do buffer: {exc}')
                except Exception as exc:
                    self.get_logger().debug(f'[DRAG] Erro inesperado no tracking: {exc}')
                continue
            # Execução de movimento em andamento → worker controla o braço real.
            if self._exec_movement_id is not None:
                continue
            # Jog manual: MovJ via _cb_arm_trajectory cuida do espelhamento;
            # enquanto o MovJ viaja, o follow espelha o feedback real → sim.
            with self._lock:
                phase = self._latest_phase
            if phase in ('IDLE', 'DONE', 'ABORTED'):
                self._mirror_follow_tick()
                continue
            positions = self._latest_joint_rad
            if positions is None:
                continue
            q_new = np.asarray(positions, dtype=np.float64)
            last = self._mirror_last_target
            if last is not None and \
                    np.max(np.abs(q_new - last)) < SERVOJ_DEADBAND_RAD:
                continue   # braço estacionário — sem ServoJ redundante
            # Captura referência local: evita corrida com connect/disconnect sem
            # segurar _real_lock no caminho quente (servo_j usa _dash_lock interno).
            drv = self._real_driver
            if drv is None or not self._robot_connected:
                continue
            try:
                try:
                    drv.servo_j_urdf(positions)
                except CR10RealDriverError:
                    drv.prepare_servoj()
                    drv.servo_j_urdf(positions)
                self._last_robot_cmd_t = time.monotonic()
                if self._drag_enabled:
                    self._drag_enabled = False
                    self.root.after(0, self._update_drag_btn_auto, False)
            except CR10RealDriverError as exc:
                self.get_logger().warning(f'ServoJ falhou: {exc}')
                continue
            self._mirror_last_target = q_new
            _diag_count += 1
            if _diag_count >= 330:   # ~10 s (era 90 = 2.7 s — causava jitter periódico)
                _diag_count = 0
                # Diagnóstico fora do caminho crítico: apenas loga, não bloqueia ServoJ.
                try:
                    ang = drv.get_angle_deg()
                    self.get_logger().info(f'[MIRROR-POS] GetAngle real: {ang}')
                except Exception:
                    pass
    def _mirror_follow_tick(self) -> None:
        """Espelha o feedback do braço real → Gazebo durante um MovJ de jog."""
        now = time.monotonic()
        if now >= self._mirror_follow_until:
            self._mirror_following = False
            self._follow_last_q = None
            self._follow_last_t = None
            return
        drv = self._real_driver
        if drv is None or not self._robot_connected:
            self._mirror_following = False
            return
        try:
            q_urdf = drv.read_joints_urdf_latest()
        except Exception:
            return   # leitura transitória inválida — tenta no próximo tick
        # Guard: firmware zero-blip — ignorar tick.
        if np.linalg.norm(q_urdf) < 0.05:
            return
        last = self._follow_last_q
        last_t = self._follow_last_t
        # Guard: salto fisicamente impossível (>60° em um tick de 30 ms).
        if last is not None and np.max(np.abs(q_urdf - last)) > math.radians(60):
            return
        self._mirror_following = True
        moved_now = last is None or np.max(np.abs(q_urdf - last)) >= 1e-4
        if moved_now:
            self._follow_still_ticks = 0
            if last is not None:
                self._follow_moved = True
        else:
            self._follow_still_ticks += 1
            # Assentou: só encerra depois de o braço ter efetivamente se
            # movido — logo após o MovJ ele ainda está parado no ponto de
            # partida e encerrar aí congelaria o sim na pose antiga.
            if self._follow_moved and self._follow_still_ticks >= 15:
                self._mirror_follow_until = 0.0
                self._mirror_following = False
                self._follow_last_q = None
                self._follow_last_t = None
                return
        # Velocidade por diferença finita para interpolação suave no JTC
        # (mesma técnica do drag teach).
        if last is not None and last_t is not None:
            dt = min(max(now - last_t, 0.005), 0.2)
            vel = np.clip((q_urdf - last) / dt, -2.5, 2.5)
        else:
            vel = np.zeros(6)
        self._follow_last_q = q_urdf
        self._follow_last_t = now
        if not moved_now:
            return   # braço estacionário — sem republicação redundante
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q_urdf]
        pt.velocities = [float(v) for v in vel]
        pt.time_from_start = Duration(sec=0, nanosec=60_000_000)
        msg.points = [pt]
        self._arm_pub.publish(msg)
    # Persistência de IPs e modo (~/.config/touch_pack/robot.json)
    def _load_robot_config(self) -> None:
        """Carrega `_robot_cfg` (mescla defaults com JSON salvo). Silencioso
        se o arquivo não existir ou estiver corrompido — só preenche faltantes."""
        try:
            if not os.path.exists(ROBOT_CONFIG_FILE):
                return
            with open(ROBOT_CONFIG_FILE) as fh:
                data = json.load(fh)
            if not isinstance(data, dict):
                return
            for k, default in ROBOT_CONFIG_DEFAULTS.items():
                v = data.get(k)
                if isinstance(v, str) and v.strip():
                    self._robot_cfg[k] = v.strip()
            self.get_logger().info(
                f'Config robô carregada de {ROBOT_CONFIG_FILE}: '
                f'hand={self._robot_cfg["hand_ip"]} '
                f'robot={self._robot_cfg["robot_ip"]} '
                f'mode={self._robot_cfg["robot_mode"]}')
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f'Falha ao ler robot.json: {exc}')
    def _save_robot_config(self) -> None:
        """Persiste `_robot_cfg` em `ROBOT_CONFIG_FILE`. Atualiza os campos
        a partir dos StringVars antes de gravar."""
        try:
            ip_hand = (self._hand_ip_var.get() or '').strip()
            ip_robot = (self._robot_ip_var.get() or '').strip()
        except tk.TclError:
            return
        if ip_hand:
            self._robot_cfg['hand_ip'] = ip_hand
        if ip_robot:
            self._robot_cfg['robot_ip'] = ip_robot
        self._robot_cfg['robot_mode'] = self._robot_mode
        try:
            os.makedirs(os.path.dirname(ROBOT_CONFIG_FILE), exist_ok=True)
            with open(ROBOT_CONFIG_FILE, 'w') as fh:
                json.dump(self._robot_cfg, fh, indent=2, sort_keys=True)
        except OSError as exc:    # pragma: no cover
            self.get_logger().warn(f'Falha ao salvar robot.json: {exc}')
    # ROBÔ CR10 — conexão TCP/IP
    def _connect_real_robot(self) -> None:
        if not _REAL_DRIVER_OK:
            self._set_status(
                'CR10 driver unavailable (real_driver module did not load).',
                DANGER)
            return
        if self._robot_connected and self._real_driver is not None:
            self._disconnect_real_robot()
            return
        ip = (self._robot_ip_var.get() or '').strip()
        if not ip:
            self._set_status('Enter the CR10 controller IP.', DANGER)
            return
        if self._robot_connecting:
            return
        # Conexão em background — evita congelar a GUI durante os ~5 s de
        # handshake TCP + sequência ClearError/EnableRobot/SpeedFactor.
        self._robot_connecting = True
        self._robot_connect_btn.set_state('…', 'Conectando…', BTN_NEUTRAL, TEXT)
        self._set_status(f'Opening sockets to CR10 at {ip}…', PRIMARY)
        threading.Thread(
            target=self._connect_robot_worker, args=(ip,), daemon=True).start()
    @staticmethod
    def _close_driver_quietly(drv) -> None:
        """Fecha um driver que não chegou a ser publicado. connect() pode ter
        aberto os dois sockets e subido o keepalive antes de enable() falhar
        (típico com o controlador em modo LOCAL); sem este close a thread de
        keepalive segura o driver vivo, os sockets ficam pendurados no
        controlador e nem o GC recolhe."""
        if drv is None:
            return
        try:
            drv.close()
        except Exception as exc:
            log.debug('[ROBOT] close() do driver parcial falhou: %s', exc)
    def _connect_robot_worker(self, ip: str) -> None:
        """Roda em thread daemon — conecta e habilita o CR10 sem bloquear a GUI."""
        log.info('[ROBOT] Iniciando conexão com CR10 em %s', ip)
        drv = None
        try:
            cfg = CR10RealDriverConfig(ip=ip)
            log.info('[ROBOT] Config: timeout=%.1fs, speed=%d%%, '
                     'payload=%.2fkg, collision=%d',
                     cfg.connect_timeout_s, cfg.speed_factor,
                     cfg.payload_kg, cfg.collision_level)
            drv = CR10RealDriver(ip=ip, dry_run=False, config=cfg)

            log.info('[ROBOT] Abrindo sockets TCP '
                     '(29999 dashboard / 30004 feedback)…')
            self.root.after(0, lambda: self._set_status(
                f'Conectando sockets TCP em {ip}:29999/30004…', PRIMARY))
            drv.connect()
            log.info('[ROBOT] Sockets abertos com sucesso')
            self.root.after(0, lambda: self._set_status(
                f'CR10 {ip}: sockets OK — enviando ClearError/EnableRobot…',
                PRIMARY))

            log.info('[ROBOT] Executando sequência de enable '
                     '(ClearError → EnableRobot → SpeedFactor → SetCollisionLevel → PayLoad)…')
            drv.enable()
            log.info('[ROBOT] Enable concluído')

            # Aguarda o firmware completar EnableRobot antes de ler o modo.
            log.info('[ROBOT] Aguardando firmware (1.5 s)…')
            time.sleep(1.5)

            mode_raw = drv.robot_mode() or ''
            log.info('[ROBOT] RobotMode() → %r', mode_raw)
            self.root.after(
                0, lambda d=drv, m=mode_raw: self._finish_robot_connect(ip, d, m))
            drv = None      # entregue à GUI: quem fecha agora é o disconnect
        except CR10RealDriverError as exc:
            log.error('[ROBOT] Falha na conexão: %s', exc)
            self._close_driver_quietly(drv)
            self.root.after(0, lambda e=str(exc): self._fail_robot_connect(e))
        except Exception as exc:
            log.exception('[ROBOT] Erro inesperado durante conexão')
            self._close_driver_quietly(drv)
            self.root.after(
                0, lambda e=str(exc): self._fail_robot_connect(
                    f'Unexpected error: {e}'))
    def _finish_robot_connect(self, ip: str, drv,
                               mode_raw: str) -> None:
        """Callback no thread Tkinter após conexão bem-sucedida."""
        log.warning('[DBG] _finish_robot_connect: ip=%s mode_raw=%r robot_mode=%r',
                    ip, mode_raw, self._robot_mode)
        self._robot_connecting = False
        self._robot_reconnecting = False
        self._real_driver = drv
        self._robot_connected = True
        # Ponto ÚNICO em que um driver novo entra em serviço — e portanto o
        # único lugar onde a trava de E-STOP pode ser reimposta nele. Sem
        # isto o `_estop_engaged` do driver nasce False e o E-STOP que o
        # operador engatou some junto com o objeto antigo.
        if getattr(self, '_estop_latched', False):
            try:
                drv.emergency_stop()
            except CR10RealDriverError as exc:
                self.get_logger().error(
                    f'[ROBOT] não foi possível reimpor o E-STOP no driver '
                    f'reconectado: {exc}')
            self._set_status(
                'CR10 reconnected with E-STOP still ENGAGED — the arm stays '
                'disabled. Press E-STOP again to release and re-enable.',
                DANGER)
            self._refresh_estop_button()
        # (Re)conexão pode significar remontagem/reboot — invalida a
        # calibração de frame do modo MovL (refeita na próxima HOME).
        # Robô acabou de (re)conectar — drag nunca está ativo no hardware a
        # esta altura (enable() colocou o robô em idle).
        if self._drag_enabled:
            self._drag_enabled = False
            self._publish_drag_state(False)
            btn = self._drag_btn
            if btn is not None:
                btn.config(text='✋ Drag OFF', bg=BTN_NEUTRAL, fg=TEXT,
                           activebackground=_shade(BTN_NEUTRAL, -0.08))
        log.warning('[DBG] _finish_robot_connect: _robot_connected=True drv=%s', drv)
        self._robot_connect_btn.set_state('⚡', 'Disconnect', OK, 'white')
        # Conexão deu certo — persistir o IP para reusar no próximo boot.
        self._save_robot_config()
        # Heartbeat só inicia após uma conexão saudável; se cair, tenta
        # reconectar com backoff automaticamente.
        self._start_robot_heartbeat()
        # Aplica SpeedFactor do slider GUI ao braço real imediatamente após a
        # conexão (enable() usa SPEED_FACTOR_DEFAULT=10%; aqui sincronizamos com o slider).
        try:
            sf = int(max(SPEED_FACTOR_MIN,
                         min(SPEED_FACTOR_MAX, self.speed_factor_var.get())))
            drv._send_dash(f'SpeedFactor({sf})')
            log.warning('[CONNECT] SpeedFactor(%d)%% aplicado ao CR10', sf)
        except Exception as exc:
            log.warning('[CONNECT] SpeedFactor falhou na conexão: %s', exc)
            sf = drv.cfg.speed_factor
        # Modo 5 = ENABLE (pronto); 9 = ERROR no Dobot CR.
        # Usa regex \{9\} para evitar falso-positivo em IPs ou timestamps que
        # contenham '9' (ex.: 192.168.1.9 → '9' in mode_raw = True erroneamente).
        mode_note = f'  [RobotMode: {mode_raw[:60].strip()}]' if mode_raw else ''
        color = DANGER if re.search(r'\{9\}', mode_raw) else OK
        self._set_status(
            f'CR10 connected at {ip} '
            f'(SpeedFactor={sf}%){mode_note}.', color)
        # O wrench do CR10 (read_tcp_force) NÃO é publicado: quem entrega
        # /ft_sensor/wrench é o ft_receiver, com a FA7155.
        if self._robot_mode == 'MIRROR':
            self._set_status(
                f'CR10 connected at {ip} — MIRROR mode active '
                f'(SpeedFactor={sf}%): move the sliders or start palpation.', OK)

        # Sincronizar Gazebo com posição real do robô via JTC.
        # Mais robusto que set_model_configuration: usa o controller já ativo.
        threading.Thread(target=self._sync_gazebo_to_real, args=(drv,),
                         daemon=True, name='gazebo-sync').start()
    def _sync_gazebo_to_real(self, drv) -> None:
        """Lê as juntas do robô real, move o Gazebo via JTC e atualiza os sliders."""
        time.sleep(2.0)
        try:
            q_urdf = drv.read_joints_urdf()   # 6 valores em RADIANOS (URDF)
        except Exception as exc:
            self.get_logger().warning(f'[SYNC] Leitura de juntas falhou: {exc}')
            return

        # 1. Move o Gazebo via JTC (radianos — formato exigido pelo controller).
        try:
            msg = JointTrajectory()
            msg.joint_names = list(ARM_JOINTS)
            pt = JointTrajectoryPoint()
            pt.positions  = [float(v) for v in q_urdf]
            pt.velocities = [0.0] * 6
            pt.time_from_start = Duration(sec=3, nanosec=0)
            msg.points = [pt]
            self._arm_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warning(f'[SYNC] Publicação JTC falhou: {exc}')

        # 2. Converte para graus e atualiza os sliders da GUI no thread Tk.
        q_deg = {j: math.degrees(float(q_urdf[i])) for i, j in enumerate(ARM_JOINTS)}
        deg_str = '  '.join(f'{j[-1]}={v:+.1f}°' for j, v in q_deg.items())
        self.get_logger().info(f'[SYNC] Gazebo → posição real: {deg_str}')

        def _update_sliders():
            self._suppressing = True
            try:
                for j in ARM_JOINTS:
                    lo, hi = ARM_LIMITS_DEG[j]
                    clamped = max(lo, min(hi, q_deg[j]))
                    self.arm_sliders[j].set(clamped)
            finally:
                self._suppressing = False

        self.root.after(0, _update_sliders)
    def _fail_robot_connect(self, error: str) -> None:
        """Callback no thread Tkinter após falha na conexão."""
        self._robot_connecting = False
        self._robot_connect_btn.set_state('⚡', 'Connect', PRIMARY, 'white')
        self._set_status(f'Failed to connect CR10: {error}', DANGER)
    def _disconnect_real_robot(self) -> None:
        # Mirror timer e bridge precisam parar ANTES de fechar os sockets —
        # senão as threads ainda tentam I/O em socket morto.
        self._stop_robot_heartbeat()
        self._robot_reconnecting = False
        with self._mirror_timer_lock:
            if self._mirror_timer is not None:
                self._mirror_timer.cancel()
                self._mirror_timer = None
            self._mirror_last_target = None
        drv = self._real_driver
        if drv is None:
            self._robot_connected = False
            self._robot_connect_btn.set_state(
                '⚡', 'Connect', PRIMARY, 'white')
            return
        try:
            drv.stop()
        except CR10RealDriverError as exc:
            self.get_logger().debug(f'drv.stop() falhou no disconnect: {exc}')
        try:
            drv.close()
        except OSError as exc:
            self.get_logger().debug(f'drv.close() falhou no disconnect: {exc}')
        self._real_driver = None
        self._robot_connected = False
        self._robot_connect_btn.set_state('⚡', 'Connect', PRIMARY, 'white')
        self._set_status('CR10 desconectado.', TEXT_DIM)
    # Heartbeat + reconexão automática (braço CR10)
    def _start_robot_heartbeat(self) -> None:
        """Inicia thread daemon que sonda `RobotMode()` a 5 Hz (200 ms).
        Após MAX_FAILURES (40 ≈ 8 s) consecutivas, dispara a reconexão."""
        thr = self._robot_heartbeat_thread
        if thr is not None and thr.is_alive():
            return
        self._robot_heartbeat_stop.clear()
        self._robot_heartbeat_thread = threading.Thread(
            target=self._robot_heartbeat_loop, daemon=True)
        self._robot_heartbeat_thread.start()
    def _stop_robot_heartbeat(self) -> None:
        self._robot_heartbeat_stop.set()
        thr = self._robot_heartbeat_thread
        if thr is not None and thr is not threading.current_thread():
            thr.join(timeout=0.5)
        self._robot_heartbeat_thread = None
    def _robot_heartbeat_loop(self) -> None:
        """Heartbeat a 1 Hz: verifica conexão e detecta drag por movimento."""
        HEARTBEAT_PERIOD_S = 0.2   # 5 Hz — detecção de drag em ~200 ms
        MAX_FAILURES = 40          # 8 s antes de reconectar (40 × 200 ms)
        DRAG_THRESH_RAD  = math.radians(0.8)  # 0.8° por junta — ignora ruído estático
        DRAG_SILENCE_S   = 2.0                # segundos sem comando do PC
        failures  = 0
        q_prev: np.ndarray | None = None

        while not self._robot_heartbeat_stop.is_set():
            if self._robot_heartbeat_stop.wait(HEARTBEAT_PERIOD_S):
                return
            if not self._robot_connected or self._real_driver is None:
                return
            drv = self._real_driver
            if drv is None:
                return

            # Heartbeat: RobotMode() serve como keep-alive do dashboard
            ok = False
            try:
                resp = drv.robot_mode()
                ok = bool(resp) and '{' in resp
            except (CR10RealDriverError, OSError):
                ok = False

            if not ok:
                failures += 1
                self.get_logger().warn(
                    f'Heartbeat CR10 falhou ({failures}/{MAX_FAILURES}).')
                if failures >= MAX_FAILURES:
                    self.root.after(0, self._on_robot_connection_lost)
                    return
                continue
            failures = 0

            # Detecção de drag por movimento de juntas
            try:
                q_now = drv.read_joints_urdf_latest()
            except Exception:
                q_prev = None
                continue

            # Guard: firmware retorna zeros durante transições — ignorar.
            if np.linalg.norm(q_now) < 0.05:
                continue

            if q_prev is not None:
                movement = float(np.max(np.abs(q_now - q_prev)))

                # Enquanto o robô se aproxima do alvo comandado pelo slider
                # (dist diminuindo), mantém o silence clock zerado para
                # evitar falso drag durante execução de MovJ (que pode levar
                # >2 s).
                target = self._mirror_last_target
                if target is not None:
                    dist_now  = float(np.max(np.abs(q_now  - target)))
                    dist_prev = float(np.max(np.abs(q_prev - target)))
                    if dist_now < dist_prev and dist_now > math.radians(1.5):
                        self._last_robot_cmd_t = time.monotonic()

                silence = time.monotonic() - self._last_robot_cmd_t
                with self._lock:
                    phase = self._latest_phase

                if movement > DRAG_THRESH_RAD and silence > DRAG_SILENCE_S:
                    # Juntas em movimento sem comando do PC → drag físico detectado.
                    if not self._drag_enabled and phase in ('IDLE', 'DONE', 'ABORTED'):
                        self.get_logger().warning(
                            f'[DRAG] Movimento sem comando detectado '
                            f'(max_dq={math.degrees(movement):.2f}°, '
                            f'silêncio={silence:.1f}s) — drag ativado.')
                        self._drag_last_valid_q = None
                        self._drag_last_t = None
                        self._drag_enabled = True
                        self.root.after(0, self._update_drag_btn_auto, True)

            q_prev = q_now
    def _on_robot_connection_lost(self) -> None:
        """Callback Tk — heartbeat detectou perda. Marca desconectado,
        derruba os recursos dependentes e dispara reconexão automática."""
        if self._robot_reconnecting or not self._robot_connected:
            return
        self._robot_reconnecting = True
        self._robot_connected = False
        # Drag não pode continuar ativo sem conexão — reset estado e botão.
        if self._drag_enabled:
            self._drag_enabled = False
            self._publish_drag_state(False)
            btn = self._drag_btn
            if btn is not None:
                btn.config(text='✋ Drag OFF', bg=BTN_NEUTRAL, fg=TEXT,
                           activebackground=_shade(BTN_NEUTRAL, -0.08))
        self._robot_connect_btn.set_state(
            '…', 'Reconnecting…', WARN, 'white')
        self._set_status(
            'CR10 connection lost — trying to reconnect automatically…',
            WARN)
        drv = self._real_driver
        self._real_driver = None
        if drv is not None:
            try:
                drv.close()
            except OSError:
                pass
        # Heartbeat acabou de sair (return após dispatch). Não precisa
        # parar de novo — apenas dispara o worker.
        self._spawn_robot_reconnect()
    def _spawn_robot_reconnect(self) -> None:
        """Inicia worker que tenta reconectar com backoff exponencial."""
        thr = self._robot_reconnect_thread
        if thr is not None and thr.is_alive():
            return
        ip = (self._robot_ip_var.get()
              or self._robot_cfg.get('robot_ip', '192.168.5.2')).strip()
        self._robot_reconnect_thread = threading.Thread(
            target=self._robot_reconnect_worker, args=(ip,), daemon=True)
        self._robot_reconnect_thread.start()
    def _robot_reconnect_worker(self, ip: str) -> None:
        """Backoff exponencial 2→3→4.5→…→30 s. Para quando reconectar
        ou quando o usuário desconecta/fecha (cancela via flag)."""
        backoff = 2.0
        max_backoff = 30.0
        attempt = 0
        while (not self._stop_event.is_set()
               and self._robot_reconnecting):
            attempt += 1
            self.get_logger().info(
                f'[ROBOT] Reconexão tentativa {attempt} → {ip}')
            drv = None
            try:
                cfg = CR10RealDriverConfig(ip=ip)
                drv = CR10RealDriver(ip=ip, dry_run=False, config=cfg)
                drv.connect()
                # E-STOP travado: NÃO reabilitar. `enable()` faz PowerOn +
                # EnableRobot e espera o modo 5 — isto é, a reconexão
                # automática rearmava o braço por conta própria, com a trava
                # da GUI ainda acesa e o operador lendo "RECONECTAR" no
                # botão. Como o driver é NOVO, a trava de software do antigo
                # também se perdia: o braço voltava aceitando ServoJ.
                if getattr(self, '_estop_latched', False):
                    self.get_logger().warn(
                        '[ROBOT] reconectado com E-STOP TRAVADO — braço '
                        'deixado desabilitado. Solte o E-STOP (segundo toque) '
                        'para rearmar.')
                else:
                    drv.enable()
                time.sleep(1.5)
                mode_raw = drv.robot_mode() or ''
                self.root.after(
                    0, lambda d=drv, m=mode_raw: self._finish_robot_connect(
                        ip, d, m))
                return
            except CR10RealDriverError as exc:
                # Sem este close, cada volta do backoff deixaria 2 sockets e
                # uma thread de keepalive vivos — e este laço roda até o
                # usuário desistir.
                self._close_driver_quietly(drv)
                self.get_logger().warn(
                    f'Reconexão {attempt} falhou: {exc} '
                    f'(próxima em {backoff:.0f} s)')
                self.root.after(0, lambda a=attempt, b=backoff: self._set_status(
                    f'Reconnecting CR10 — attempt {a} failed, '
                    f'next in {b:.0f} s.', WARN))
            if self._stop_event.wait(backoff):
                return
            backoff = min(max_backoff, backoff * 1.5)
        self.root.after(0, lambda: self._robot_connect_btn.set_state(
            '⚡', 'Connect', PRIMARY, 'white'))
    def _set_robot_mode(self, selected: str) -> None:
        mode = (selected or '').strip().upper()
        if mode not in ('SIM_ONLY', 'MIRROR'):
            return
        if self._robot_connecting:
            # Conexão em andamento — recusa a troca para não corrermos
            # com o worker que ainda vai setar `_real_driver`.
            self._robot_mode_var.set(self._robot_mode)
            self._set_status(
                'Wait for the connection to finish before switching modes.', WARN)
            return
        # Palpação em curso — trocar de SIM_ONLY ↔ MIRROR no meio do
        # experimento poderia perder/comandar o braço real fora de hora.
        if self._latest_phase not in ('IDLE', 'DONE', 'ABORTED'):
            self._robot_mode_var.set(self._robot_mode)
            self._set_status(
                f'Palpation in progress (phase {self._latest_phase}) — '
                'wait for it to finish before switching modes.', WARN)
            return
        self._robot_mode = mode
        self._save_robot_config()
        if mode == 'MIRROR':
            self._set_status(
                'MIRROR mode — move the sliders to control the real arm.',
                WARN if not self._robot_connected else OK)
        else:
            with self._mirror_timer_lock:
                if self._mirror_timer is not None:
                    self._mirror_timer.cancel()
                    self._mirror_timer = None
                self._mirror_last_target = None
            self._set_status(
                'SIM_ONLY mode — commands go to the simulation only.', OK)
    # E-STOP (combina parada do robô + abertura da mão)
    def _estop(self) -> None:
        """Botão de E-STOP — CHAVE COM TRAVA, não pulso.

        O firmware modela assim: EmergencyStop(1) pressiona, EmergencyStop(0)
        solta, e o guia V4.5.1 é explícito sobre o rearme — "After the
        emergency stop, the robot arm will be disabled and then alarm. You
        need to release the E-Stop switch and clear the alarm to re-enable the
        robot arm."

        Primeiro toque  → pressiona: braço desabilitado + alarme, e o driver
                          passa a RECUSAR todo comando de movimento.
        Segundo toque   → solta e rearma: EmergencyStop(0) + ClearError +
                          enable(), só então o braço volta a aceitar comandos.

        Antes o botão fazia StopRobot+DisableRobot, que para o braço mas não
        deixa nenhum estado travado: nada exigia uma ação deliberada para
        voltar a mover, e um Start seguinte simplesmente religava tudo.
        """
        if getattr(self, '_estop_latched', False):
            self._estop_release()
            return
        self._estop_engage()
    def _estop_engage(self) -> None:
        """Primeiro toque: pressiona a chave e congela tudo."""
        # 1. FREEZE do tactile_explorer FSM: congela NO LUGAR, sem tentar ir à
        #    HOME (o STOP normal recuaria à home, arrastando a ferramenta sobre
        #    a superfície se a pose estiver comprometida). Sem isso o explorer
        #    continuaria publicando setpoints no JTC.
        self._freeze_pub.publish(Empty())

        # 2. Congela o mirror poll loop (evita ServoJ após a parada).
        cur = self._latest_joint_rad
        if cur is not None:
            self._mirror_last_target = np.asarray(cur, dtype=np.float64)

        # 3. PRESSIONA a chave de E-Stop no braço real. Depois disto o driver
        #    recusa qualquer movimento até a soltura explícita.
        #
        #    `hw_ok` separa duas coisas que estavam sendo anunciadas como
        #    uma: a trava de SOFTWARE (o driver marca _estop_engaged antes de
        #    enviar, então ServoJ/_send_motion/drag param de qualquer jeito) e
        #    a chave no CONTROLADOR. Se o EmergencyStop(1) não chegou, o braço
        #    pode continuar habilitado — dizer "robot disabled and alarmed"
        #    ali é afirmar sobre o hardware algo que não se verificou.
        #    A condição é `driver existe`, NÃO `_robot_connected`. Durante a
        #    janela de reconexão automática (heartbeat caiu, worker tentando
        #    de novo com backoff de até 30 s) `_robot_connected` é False e o
        #    braço pode estar executando o que já está na fila de motion — era
        #    exatamente aí que o E-STOP não fazia NADA no hardware e ainda
        #    anunciava "robot disabled and alarmed". Chamar sempre também arma
        #    a trava local do driver, que é o que impede o streaming de
        #    recomeçar quando a conexão voltar.
        hw_ok = False
        hw_err = 'no driver object — nothing was sent to the controller'
        if self._real_driver is not None:
            try:
                hw_ok = self._real_driver.emergency_stop()
                if not hw_ok:
                    hw_err = ('EmergencyStop(1) did not reach the controller '
                              '(no dashboard session)')
            except CR10RealDriverError as exc:
                hw_ok = False
                hw_err = str(exc)
                self.get_logger().error(f'E-STOP real falhou: {exc}')
        elif self._robot_mode != 'MIRROR':
            # Sem braço real configurado não há hardware a alarmar, e dizer
            # que houve seria pior que não dizer nada.
            hw_err = 'simulation only — there is no real arm to alarm'

        # 4. Abre a mão via ECI.
        if self._eci_enabled and self._cli_eci_grip is not None \
                and self._eci_srv is not None:
            try:
                grip = self._eci_msg.CurrentGripID()
                grip.value = 11   # 11 = GLOVE (mão totalmente aberta)
                req = self._eci_srv.SetCurrentGrip.Request()
                req.grip_id = grip
                self._cli_eci_grip.call_async(req)
            except Exception:
                pass
        # A trava local vale mesmo com o hardware fora do ar: ela é o que
        # bloqueia o Start e o que o driver já usa para recusar movimento.
        self._estop_latched = True
        self._refresh_estop_button()
        if hw_ok:
            self._set_status(
                'E-STOP ENGAGED — robot disabled and alarmed. Press E-STOP '
                'again to release and re-enable.', DANGER)
        elif self._real_driver is None and self._robot_mode != 'MIRROR':
            self._set_status(
                'E-STOP ENGAGED — palpation frozen in place. No real arm is '
                'connected, so nothing was sent to a controller.', DANGER)
        else:
            self._set_status(
                f'E-STOP: the controller was NOT alarmed ({hw_err}). Motion '
                'is blocked in software (driver + explorer frozen) and will '
                'stay blocked through a reconnect, but anything already in '
                'the motion queue KEEPS RUNNING. Use the physical E-Stop.',
                DANGER)
    def _estop_release(self) -> None:
        """Segundo toque: solta a chave e rearma o braço.

        A trava local só cai se o rearme REALMENTE completou — `enable()`
        espera o modo 5. Um E-Stop que "soltou" sem reabilitar deixaria a GUI
        anunciando pronto com o braço ainda em alarme.
        """
        if self._real_driver is not None and self._robot_connected:
            try:
                self._real_driver.release_emergency_stop()
            except CR10RealDriverError as exc:
                self.get_logger().error(f'Rearme após E-STOP falhou: {exc}')
                self._set_status(
                    f'E-STOP release FAILED: {exc} — robot still disabled.',
                    DANGER)
                self._refresh_estop_button()
                return
        self._estop_latched = False
        self._refresh_estop_button()
        self._set_status(
            'E-STOP released — robot re-enabled. Experiments can start again.',
            OK)
    def _refresh_estop_button(self) -> None:
        """Rótulo/cor do botão dizem em qual metade do ciclo ele está.

        Passa por `set_state` e não por `config(text=…)`: o texto do botão é
        ' <ícone>  <rótulo> ', então escrever só o ícone apagava o nome — e
        como este refresh roda já no fim do `_build_header`, o botão nunca
        chegava a exibir 'E-STOP'. `set_state` também registra a cor nova no
        estado interno do widget, que é o que o hover do `_hdr_btn` restaura
        ao sair do mouse; com `config` a cor da trava se perdia no <Leave>.
        """
        btn = getattr(self, '_estop_btn', None)
        if btn is None:
            return
        try:
            if getattr(self, '_estop_latched', False):
                # Travado: o próximo toque rearma o braço, não para nada.
                btn.set_state('⟳', 'RECONECTAR', WARN)
            else:
                btn.set_state('■', 'E-STOP', DANGER)
        except tk.TclError:
            pass          # janela fechando
