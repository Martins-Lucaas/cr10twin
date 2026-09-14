"""gui_hand.py — a mão COVVI real: conexão, energia, espelho e watchdog.

A mão é um subprocesso com driver próprio, e quase tudo aqui existe por causa
disso: ela pode morrer sem avisar, pode ficar energizada depois que a GUI
fechou, e pode aceitar um comando estando meio-conectada.

Três coisas não são negociáveis e moram juntas por isso:

  * desligar a energia é BLOQUEANTE no encerramento. Deixar assíncrono faz a
    GUI fechar antes de o comando sair, e a mão fica energizada na bancada;
  * o watchdog existe porque um driver morto não devolve erro — devolve
    silêncio, e o espelho continuaria publicando contra ninguém;
  * o espelho só liga com a mão conectada E respondendo. Espelhar contra uma
    mão ausente não falha: simplesmente não move, e o operador ajusta o slider
    achando que a mão real acompanha.

Recortado de `palpation_gui.py` — os métodos operam sobre `self` como antes.
"""
from __future__ import annotations

import logging

from .gui_constants import COVVI_GRIPS, SPEED_FACTOR_DEFAULT

# A lista de juntas MIMIC vem junto do driver: sem ele a mão real não conecta,
# e a simulada não precisa dela.
try:
    from .kinematics import MIMIC_LIST
except Exception:  # pragma: no cover
    MIMIC_LIST = []

import math as _math
import os
import signal
import subprocess
import threading
import time
import tkinter as tk
from .constants import (
    ECI_POSN_CLOSED,
    ECI_POSN_OPEN,
    HAND_JOINTS,
    eci_posn_to_deg as _eci_posn_to_deg,
    hand_deg_to_driver_rad as _hand_deg_to_driver_rad,
)
from .gui_constants import HAND_LIMITS_DEG
from .ui_helpers import BTN_NEUTRAL, DANGER, OK, PRIMARY, TEXT, TEXT_DIM, WARN
from collections.abc import Mapping
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


log = logging.getLogger('touch_pack.palpation_gui')   # mesmo canal do host


class HandMixin:
    """gui_hand.py — a mão COVVI real: conexão, energia, espelho e watchdog."""

    def _apply_covvi_grip(self):
        """Aplica o grip-pattern COVVI selecionado no combobox."""
        if getattr(self, '_covvi_grip_var', None) is None:
            return   # modo touch_tool — sem painel da mão
        name = self._covvi_grip_var.get()
        spec = COVVI_GRIPS.get(name)
        if spec is None:
            return
        eci_id, deg = spec
        self._apply_hand_preset(deg, eci_grip_id=eci_id)
        self._set_status(f'Grip COVVI > {name} (id={eci_id})', OK)
    def _publish_hand_from_sliders(self, *, allow_real: bool = True):
        if self._suppressing:
            return
        # No modo touch_tool a coluna da mão não é construída (sem sliders).
        if not getattr(self, 'hand_sliders', None):
            return
        self._suppressing = True
        try:
            primary_deg: dict[str, float] = {}
            primary_rad: dict[str, float] = {}
            for j in HAND_JOINTS:
                lo, hi = HAND_LIMITS_DEG[j]
                v = self._clamp_var(self.hand_sliders[j], lo, hi)
                if v is None:
                    return
                primary_deg[j] = float(v)
                primary_rad[j] = _math.radians(v)
            duration_s = self._move_duration_seconds()
        finally:
            self._suppressing = False
        # Versão B (mirror real→sim): quando a telemetria DigitPosnAll está
        # chegando, a mão simulada segue a POSIÇÃO MEDIDA da mão real (em
        # _on_real_hand_posn) — assim o sim acompanha a velocidade física.
        if not self._hand_mirror_live():
            self._publish_sim_hand(primary_rad, duration_s)
        # Envia para a mão real via ECI (SetDigitPosn) se ativo. `allow_real`
        # deixa o teleop por câmera comandar só o sim quando a checkbox
        # "→ real" está desmarcada.
        if self._eci_enabled and allow_real:
            self._schedule_eci_posn(primary_deg)
    def _publish_sim_hand(self, primary_rad: dict[str, float],
                           duration_s: float) -> None:
        """Publica a trajetória da mão no Gazebo a partir das 6 juntas
        primárias (rad), expandindo as juntas mimic do URDF. Usado tanto pelo
        comando do slider (sim-only) quanto pelo mirror real→sim (Versão B).

        `primary_rad` vem em graus de PONTA DE DEDO (0–90°, a escala do slider
        e da mão física). O Gazebo move a junta DRIVER, que vai só até 1,0 rad
        — mandar o ângulo de dedo direto satura o teto e ceifa a excursão."""
        names = list(HAND_JOINTS)
        driver_rad = {j: _hand_deg_to_driver_rad(j, _math.degrees(primary_rad[j]))
                      for j in HAND_JOINTS}
        positions = [driver_rad[j] for j in HAND_JOINTS]
        # Expande as 26 juntas mimic com as razões do URDF. O multiplicador é
        # relativo ao DRIVER, igual ao clamp em hand_pack.urdf_helpers.
        for mimic_name, driver, mult in MIMIC_LIST:
            names.append(mimic_name)
            positions.append(driver_rad[driver] * mult)
        msg = JointTrajectory()
        # stamp=zero → controller starts immediately (sim-time-safe).
        msg.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start = self._duration_msg(duration_s)
        msg.points = [pt]
        self._hand_pub.publish(msg)
    # Versão B: mirror real→sim da mão (telemetria DigitPosnAll)
    def _hand_mirror_live(self) -> bool:
        """True se o mirror real→sim está ativo E recebeu telemetria
        DigitPosnAll há menos de 0.5 s. Caso contrário o slider volta a
        comandar o sim diretamente (fallback robusto se a telemetria parar)."""
        if not self._hand_mirror_active:
            return False
        last = self._hand_mirror_last_rx
        return last is not None and (time.monotonic() - last) < 0.5
    def _on_real_hand_posn(self, msg) -> None:
        """Callback do tópico DigitPosnAll: converte a posição MEDIDA dos
        dedos (escala ECI 0–200) para rad e dirige a mão simulada. O sim
        passa a seguir a velocidade real da mão física (Versão B)."""
        now = time.monotonic()
        self._hand_mirror_last_rx = now

        _deg = _eci_posn_to_deg

        primary_rad = {
            'Thumb':  _math.radians(_deg('Thumb',  msg.thumb_pos)),
            'Index':  _math.radians(_deg('Index',  msg.index_pos)),
            'Middle': _math.radians(_deg('Middle', msg.middle_pos)),
            'Ring':   _math.radians(_deg('Ring',   msg.ring_pos)),
            'Little': _math.radians(_deg('Little', msg.little_pos)),
            'Rotate': _math.radians(_deg('Rotate', msg.rotate_pos)),
        }
        # Horizonte de interpolação ~ período de chegada das mensagens:
        # mantém o sim "colado" à posição real sem solavanco entre amostras.
        last = self._hand_mirror_last_pub
        self._hand_mirror_last_pub = now
        dt = (now - last) if last is not None else 0.05
        duration_s = min(0.15, max(0.03, dt))
        self._publish_sim_hand(primary_rad, duration_s)
    def _enable_hand_mirror(self, attempt: int = 0) -> None:
        """Habilita o streaming digit_posn no driver e assina o tópico
        DigitPosnAll para espelhar a mão real → sim (Versão B).
        """
        if self._hand_mirror_active or not self._eci_enabled or self._eci_msg is None:
            return
        # 1) Pede ao driver para emitir digit_posn em realtime (preservando os
        #    streams que o driver já liga no startup: digit_touch/env/orient).
        cli = self._cli_eci_realtime
        if cli is None or not cli.service_is_ready():
            if attempt < 20:
                self.root.after(
                    500, lambda: self._enable_hand_mirror(attempt + 1))
            else:
                self.get_logger().warning(
                    'SetRealtimeCfg indisponível — mirror da mão sem stream '
                    'digit_posn (sim não seguirá a mão real).')
            return
        try:
            req = self._eci_srv.SetRealtimeCfg.Request()
            req.digit_posn    = True
            req.digit_touch   = True
            req.environmental = True
            req.orientation   = True
            cli.call_async(req)
        except Exception as exc:
            self.get_logger().warning(
                f'SetRealtimeCfg(digit_posn) falhou: {exc}')
            return
        # 2) Assina o tópico de posição medida da mão.
        if self._sub_real_hand_posn is None:
            self._sub_real_hand_posn = self.create_subscription(
                self._eci_msg.DigitPosnAllMsg,
                f'{self._eci_prefix}/DigitPosnAllMsg',
                self._on_real_hand_posn, 10)
        self._hand_mirror_active = True
        self._hand_mirror_last_rx = None
        self.get_logger().info('[HAND-MIRROR] real→sim ativo (DigitPosnAll).')
    def _disable_hand_mirror(self) -> None:
        """Desliga o mirror real→sim e devolve o comando do sim ao slider."""
        self._hand_mirror_active = False
        self._hand_mirror_last_rx = None
        sub = self._sub_real_hand_posn
        self._sub_real_hand_posn = None
        if sub is not None:
            try:
                self.destroy_subscription(sub)
            except Exception:
                pass
    def _schedule_eci_posn(self, deg_dict: dict) -> None:
        """Debounce de 60 ms para SetDigitPosn — evita flood de serviço."""
        if not self._eci_enabled or self._cli_eci_posn is None:
            return
        if self._eci_posn_after is not None:
            try:
                self.root.after_cancel(self._eci_posn_after)
            except Exception:
                pass
        self._eci_posn_after = self.root.after(
            60, lambda v=dict(deg_dict): self._send_eci_posn_now(v))
    def _send_eci_posn_now(self, deg_dict: dict) -> None:
        """Envia SetDigitPosn convertendo graus → escala ECI 0-200."""
        self._eci_posn_after = None
        if not self._eci_enabled or self._cli_eci_posn is None:
            return
        if not self._cli_eci_posn.service_is_ready():
            return

        def _to_eci(joint: str, deg: float) -> int:
            max_deg = 60.0 if joint == 'Rotate' else 90.0
            lo, hi = ECI_POSN_OPEN[joint], ECI_POSN_CLOSED[joint]
            pos = lo + deg / max_deg * (hi - lo)
            return max(0, min(255, int(round(pos))))

        req = self._eci_srv.SetDigitPosn.Request()
        req.speed = self._eci_msg.Speed()
        try:
            sf = float(self.speed_factor_var.get())
        except (ValueError, tk.TclError):
            sf = SPEED_FACTOR_DEFAULT
        # O firmware COVVI clampa velocidades abaixo de Speed.MIN=15 para 15
        # (eci/primitives/speed.py) — clampar aqui evita depender do warning
        # silencioso do driver e deixa o valor efetivo explícito.
        req.speed.value = max(15, min(100, int(sf)))
        req.thumb  = _to_eci('Thumb',  deg_dict.get('Thumb',  0.0))
        req.index  = _to_eci('Index',  deg_dict.get('Index',  0.0))
        req.middle = _to_eci('Middle', deg_dict.get('Middle', 0.0))
        req.ring   = _to_eci('Ring',   deg_dict.get('Ring',   0.0))
        req.little = _to_eci('Little', deg_dict.get('Little', 0.0))
        req.rotate = _to_eci('Rotate', deg_dict.get('Rotate', 0.0))
        self._cli_eci_posn.call_async(req)
    def _send_eci_grip(self, grip_id: int, label: str = '') -> None:
        """Chama SetCurrentGrip via ECI de forma assíncrona."""
        if not self._eci_enabled or self._cli_eci_grip is None:
            return
        if not self._cli_eci_grip.service_is_ready():
            self._set_status('ECI SetCurrentGrip unavailable (wait).',
                              WARN)
            return
        try:
            grip = self._eci_msg.CurrentGripID()
            grip.value = grip_id
            req = self._eci_srv.SetCurrentGrip.Request()
            req.grip_id = grip
            self._cli_eci_grip.call_async(req)
            if label:
                self._set_status(f'ECI > {label} (id={grip_id})', OK)
        except Exception as exc:
            self.get_logger().error(f'SetCurrentGrip falhou: {exc}')
    def _apply_hand_preset(self, preset_deg: Mapping[str, float],
                            *, eci_grip_id: int | None = None):
        """Aplica um preset de mão (Abrir/Apontar/Fechar)."""
        if not getattr(self, 'hand_sliders', None):
            return   # modo touch_tool — sem painel da mão
        self._suppressing = True
        try:
            for j in HAND_JOINTS:
                self.hand_sliders[j].set(preset_deg.get(j, 0))
        finally:
            self._suppressing = False
        self._publish_hand_from_sliders()
        if eci_grip_id is not None:
            self._send_eci_grip(eci_grip_id)
    # MÃO COVVI — conexão / ECI / PWR
    def _connect_real_hand(self) -> None:
        """Sobe `ros2 run covvi_hand_driver server <IP>` em subprocesso."""
        if self._hand_proc is not None and self._hand_proc.poll() is None:
            self._disconnect_real_hand()
            return
        ip = (self._hand_ip_var.get() or '').strip()
        if not ip:
            self._set_status('Enter the COVVI hand IP.', DANGER)
            return
        # Quebra o eci_prefix em namespace + node name, igual ao
        # manual_control_node do grasp_ml_pack (referência funcional).
        parts = self._eci_prefix.strip('/').split('/')
        _ns   = '/' + parts[0]
        _name = parts[1] if len(parts) > 1 else 'server'
        cmd = ['ros2', 'run', 'covvi_hand_driver', 'server', ip,
               '--ros-args',
               '--remap', f'__ns:={_ns}',
               '--remap', f'__name:={_name}']
        # O covvi_hand_driver vive num workspace separado (~/install).
        covvi_ws = os.path.expanduser('~/install/setup.bash')
        if (os.path.isfile(covvi_ws)
                and '/install/covvi_hand_driver'
                not in os.environ.get('AMENT_PREFIX_PATH', '')):
            cmd = ['bash', '-c',
                   f'source "{covvi_ws}" >/dev/null 2>&1 && exec "$@"',
                   'covvi-env'] + cmd
        log.warning('[DBG] _connect_real_hand: cmd=%s', cmd)
        try:
            self._hand_proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, start_new_session=True)
            # Thread daemon lê stdout+stderr do driver e redireciona para o log
            def _pipe_hand_log(proc=self._hand_proc):
                assert proc.stdout is not None   # stdout=PIPE logo acima
                for raw in proc.stdout:
                    line = raw.decode('utf-8', errors='replace').rstrip()
                    log.warning('[HAND-PROC] %s', line)
            threading.Thread(target=_pipe_hand_log, daemon=True,
                             name='hand-proc-log').start()
        except FileNotFoundError:
            self._set_status('ros2 is not on PATH (source the workspace).',
                              DANGER)
            self._hand_proc = None
            return
        self._hand_should_be_alive = True
        self._start_hand_watchdog()
        self._set_status(f'covvi_hand_driver server {ip} starting…', PRIMARY)
        self.root.after(2200, self._post_connect_real_hand)
    def _post_connect_real_hand(self) -> None:
        proc = self._hand_proc
        if proc is None or proc.poll() is not None:
            self._set_status(
                'Hand driver failed to start — check the IP / ECI box.',
                DANGER)
            self._hand_proc = None
            return
        self._hand_connect_btn.set_state('⚡', 'Disconnect', OK, 'white')
        # Conexão deu certo — persistir o IP para reusar no próximo boot.
        self._save_robot_config()
        # Ativa ECI automaticamente (como o manual_control_node do grasp_ml_pack)
        # _toggle_eci já agenda o auto-power-on em 800 ms
        if not self._eci_enabled:
            self._toggle_eci()
        self._set_status(
            f'Hand driver active ({self._eci_prefix}) — power ON soon…', OK)
    def _disconnect_real_hand(self) -> None:
        """Inicia desconexão limpa da mão COVVI."""
        self._hand_should_be_alive = False
        self._stop_hand_watchdog()

        eci_was_enabled = self._eci_enabled
        self._eci_enabled = False
        self._hand_powered = False
        self._disable_hand_mirror()
        self._eci_btn.set_state('◉', 'ECI OFF', BTN_NEUTRAL, TEXT)
        self._pwr_btn.set_state('⊙', 'PWR OFF', BTN_NEUTRAL, TEXT)
        self._hand_connect_btn.set_state('…', 'Disconnecting…', BTN_NEUTRAL, TEXT)
        self._set_status('Disconnecting COVVI hand…', TEXT_DIM)

        threading.Thread(
            target=self._disconnect_hand_worker,
            args=(eci_was_enabled,), daemon=True).start()
    def _disconnect_hand_worker(self, eci_was_enabled: bool) -> None:
        """Thread daemon: PowerOff síncrono → SIGINT/SIGTERM → wait → pausa — não bloqueia a GUI."""
        if eci_was_enabled:
            self._send_hand_poweroff_blocking(timeout_s=3.0)
        self._terminate_hand_subprocess()
        # Com o driver agora chamando eci.stop() em `finally` no shutdown
        # (covvi_server_node.main), a caixa ECI libera a sessão de imediato
        # — não é mais preciso esperar o TIME_WAIT longo.
        ECI_RESET_S = 2
        for remaining in range(ECI_RESET_S, 0, -1):
            self.root.after(0, lambda r=remaining: self._set_status(
                f'Waiting for ECI box reset — {r} s left…', TEXT_DIM))
            time.sleep(1.0)
        self.root.after(0, self._finish_hand_disconnect)
    def _finish_hand_disconnect(self) -> None:
        """Callback Tkinter: atualiza botão após o worker de desconexão concluir."""
        self._hand_connect_btn.set_state('⚡', 'Connect', PRIMARY, 'white')
        self._set_status('Hand driver disconnected — LED off.', TEXT_DIM)
    # Watchdog + re-spawn automático (mão COVVI)
    def _start_hand_watchdog(self) -> None:
        thr = self._hand_watchdog_thread
        if thr is not None and thr.is_alive():
            return
        self._hand_watchdog_stop.clear()
        self._hand_watchdog_thread = threading.Thread(
            target=self._hand_watchdog_loop, daemon=True)
        self._hand_watchdog_thread.start()
    def _stop_hand_watchdog(self) -> None:
        self._hand_watchdog_stop.set()
        thr = self._hand_watchdog_thread
        if thr is not None and thr is not threading.current_thread():
            thr.join(timeout=0.5)
        self._hand_watchdog_thread = None
    def _hand_watchdog_loop(self) -> None:
        """Poll @2 s do `Popen.poll()`. Se o driver morrer sem desconexão
        deliberada, dispara re-spawn no thread Tk."""
        WATCHDOG_PERIOD_S = 2.0
        while not self._hand_watchdog_stop.is_set():
            if self._hand_watchdog_stop.wait(WATCHDOG_PERIOD_S):
                return
            if not self._hand_should_be_alive:
                return
            proc = self._hand_proc
            if proc is None:
                continue   # ainda subindo / já encerrado
            if proc.poll() is not None:
                self.get_logger().error(
                    f'covvi_hand_driver morreu (rc={proc.returncode}). '
                    'Tentando re-spawn automático.')
                self.root.after(0, self._on_hand_died)
                return
    def _on_hand_died(self) -> None:
        """Callback Tk: limpa estado interno (ECI/power perdidos junto com
        o driver) e tenta reconectar. Preserva `_hand_should_be_alive`
        para o watchdog seguir monitorando após o re-spawn."""
        if not self._hand_should_be_alive:
            return
        # Estado de software (já estava out-of-sync com o driver morto).
        self._hand_proc = None
        self._eci_enabled = False
        self._hand_powered = False
        self._disable_hand_mirror()
        self._eci_btn.set_state('◉', 'ECI OFF', BTN_NEUTRAL, TEXT)
        self._pwr_btn.set_state('⊙', 'PWR OFF', BTN_NEUTRAL, TEXT)
        self._hand_connect_btn.set_state('…', 'Reconnecting…', WARN, 'white')
        # Aguarda 15 s antes de re-spawnar: a caixa ECI precisa desse tempo
        # para liberar o estado TCP após a conexão quebrada (ExistingConnectionError).
        self._set_status(
            'Hand driver crashed — automatic re-spawn in 15 s…', WARN)
        self.root.after(15000, self._on_hand_respawn)
    def _on_hand_respawn(self) -> None:
        """Callback Tk: re-spawn da mão após o delay de reset da caixa ECI."""
        if not self._hand_should_be_alive:
            return
        self._set_status('Automatic hand-driver re-spawn…', WARN)
        self._connect_real_hand()
    def _send_hand_poweroff_blocking(self, timeout_s: float) -> None:
        """Chama SetHandPowerOff e espera o future completar (com timeout)."""
        if self._cli_hand_pwr_off is None or self._eci_srv is None:
            return
        try:
            if not self._cli_hand_pwr_off.service_is_ready():
                # Sem serviço pronto não há como cortar o power via ECI;
                # ainda assim seguimos para o SIGTERM.
                return
            future = self._cli_hand_pwr_off.call_async(
                self._eci_srv.SetHandPowerOff.Request())
        except Exception as exc:
            self.get_logger().warning(f'PowerOff falhou: {exc}')
            return
        deadline = time.time() + max(0.05, timeout_s)
        while time.time() < deadline:
            if future.done():
                return
            time.sleep(0.02)
        self.get_logger().warning(
            f'PowerOff não concluiu em {timeout_s:.1f} s — '
            'driver será terminado mesmo assim.')
    def _terminate_hand_subprocess(self) -> None:
        """SIGINT → espera 2 s (shutdown ROS2 gracioso, fecha sockets ECI);
        se ainda vivo, SIGTERM → espera 2 s; por último SIGKILL. Idempotente."""
        proc = self._hand_proc
        self._hand_proc = None
        if proc is None or proc.poll() is not None:
            return
        # SIGINT first: triggers rclpy shutdown handlers → sockets closed cleanly
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except (OSError, ProcessLookupError) as exc:
            self.get_logger().debug(f'SIGINT da mão ignorado ({exc}).')
        try:
            proc.wait(timeout=2.0)
            return
        except subprocess.TimeoutExpired:
            pass
        # Fallback: SIGTERM
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (OSError, ProcessLookupError) as exc:
            self.get_logger().debug(f'SIGTERM da mão ignorado ({exc}).')
        try:
            proc.wait(timeout=2.0)
            return
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                'Driver da mão não saiu em 2 s após SIGTERM — forçando SIGKILL.')
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except (OSError, ProcessLookupError):
            pass
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            self.get_logger().error(
                'Driver da mão ficou zumbi após SIGKILL.')
    def _toggle_eci(self) -> None:
        """Liga/desliga o canal lógico ECI (cliente dos serviços COVVI)."""
        if self._eci_enabled:
            # Cortar alimentação antes de desativar o canal
            if self._hand_powered and self._cli_hand_pwr_off is not None:
                try:
                    if self._cli_hand_pwr_off.service_is_ready():
                        self._cli_hand_pwr_off.call_async(
                            self._eci_srv.SetHandPowerOff.Request())
                except Exception:
                    pass
            self._hand_powered = False
            self._pwr_btn.set_state('⊙', 'PWR OFF', BTN_NEUTRAL, TEXT)
            self._eci_enabled = False
            self._disable_hand_mirror()
            self._eci_btn.set_state('◉', 'ECI OFF', BTN_NEUTRAL, TEXT)
            self._set_status('ECI channel disabled — power cut.', TEXT_DIM)
            return
        try:
            import covvi_interfaces.srv as _eci_srv
            import covvi_interfaces.msg as _eci_msg
        except ImportError:
            self._set_status(
                'covvi_interfaces not available — source the workspace.',
                DANGER)
            return
        self._eci_srv = _eci_srv
        self._eci_msg = _eci_msg
        # Nomes CamelCase conforme o covvi_hand_driver expõe no grafo ROS2
        if self._cli_eci_grip is None:
            self._cli_eci_grip = self.create_client(
                _eci_srv.SetCurrentGrip,
                f'{self._eci_prefix}/SetCurrentGrip')
        if self._cli_eci_posn is None:
            self._cli_eci_posn = self.create_client(
                _eci_srv.SetDigitPosn,
                f'{self._eci_prefix}/SetDigitPosn')
        if self._cli_hand_pwr_on is None:
            self._cli_hand_pwr_on = self.create_client(
                _eci_srv.SetHandPowerOn,
                f'{self._eci_prefix}/SetHandPowerOn')
        if self._cli_hand_pwr_off is None:
            self._cli_hand_pwr_off = self.create_client(
                _eci_srv.SetHandPowerOff,
                f'{self._eci_prefix}/SetHandPowerOff')
        if self._cli_eci_realtime is None:
            # Versão B: usado para habilitar o stream digit_posn (mirror mão).
            self._cli_eci_realtime = self.create_client(
                _eci_srv.SetRealtimeCfg,
                f'{self._eci_prefix}/SetRealtimeCfg')
        self._eci_enabled = True
        self._eci_btn.set_state('◉', 'ECI ON', OK, 'white')
        self._set_status('ECI channel active — waiting for hand power…', OK)
        # Aguarda o driver registrar os serviços no grafo ROS2 antes de ligar
        self.root.after(800, self._auto_power_on_hand)
    def _auto_power_on_hand(self, attempt: int = 0) -> None:
        """Auto-power-on da mão 800 ms após o ECI ser ativado."""
        if not self._eci_enabled or self._cli_hand_pwr_on is None or self._hand_powered:
            return
        if not self._cli_hand_pwr_on.service_is_ready():
            if attempt < 15:
                self._set_status(
                    'ECI active — waiting for hand services…', WARN)
                self.root.after(
                    800, lambda: self._auto_power_on_hand(attempt + 1))
            else:
                self._set_status(
                    'ECI active — power service unavailable '
                    '(check the IP and the hand driver).', WARN)
            return
        self._cli_hand_pwr_on.call_async(self._eci_srv.SetHandPowerOn.Request())
        self._hand_powered = True
        self._pwr_btn.set_state('⊙', 'PWR ON', OK, 'white')
        self._set_status('ECI channel active — power on (blue LED lit).', OK)
        # Versão B: liga o mirror real→sim da mão ~600 ms depois (tempo para
        # o serviço SetRealtimeCfg e o tópico DigitPosnAll subirem no grafo).
        self.root.after(600, self._enable_hand_mirror)
    def _toggle_hand_power(self) -> None:
        """Liga/desliga a alimentação da mão COVVI via SetHandPowerOn/Off."""
        if not self._eci_enabled:
            self._set_status(
                'Enable the ECI channel before powering the hand.', WARN)
            return
        if self._hand_powered:
            cli = self._cli_hand_pwr_off
            req = self._eci_srv.SetHandPowerOff.Request()
            target_on = False
        else:
            cli = self._cli_hand_pwr_on
            req = self._eci_srv.SetHandPowerOn.Request()
            target_on = True
        if cli is None or not cli.service_is_ready():
            self._set_status(
                'Power service unavailable (wait for initialization).',
                WARN)
            return
        cli.call_async(req)
        self._hand_powered = target_on
        if target_on:
            self._pwr_btn.set_state('⊙', 'PWR ON', OK, 'white')
            self._set_status('Hand power ON (blue LED lit).', OK)
            # Power-on manual também ativa o mirror real→sim da mão (antes
            # só o auto-power-on ativava — ligar pelo botão deixava o sim
            # animando pela heurística de duração, dessincronizado do real).
            self.root.after(600, self._enable_hand_mirror)
        else:
            self._pwr_btn.set_state('⊙', 'PWR OFF', BTN_NEUTRAL, TEXT)
            self._set_status('Hand power OFF.', TEXT_DIM)
            self._disable_hand_mirror()
