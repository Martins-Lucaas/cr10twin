"""
real_driver.py — Camada de comunicação com o controlador Dobot CR10 real.

Baseado em:
  - Dobot TCP-IP-Python-V4 SDK (github.com/Dobot-Arm/TCP-IP-Python-V4)
  - Dobot TCP/IP Remote Control Interface Guide V3 (2025-05-08)
  - Dobot CRStudio User Guide V4.13.0_V2.14.0

Portas TCP — firmware V4.x (CR10a V4.5.1):
    29999  TODOS os comandos    dashboard + motion — único socket de controlo
    30004  feedback @8 ms       struct 1440 B com q_actual, TCPForce (125 Hz)
    30005  feedback @200 ms     mesmo struct, taxa reduzida

Sintaxe dos comandos de motion no firmware V4 (DIFERENTE do V3):
    ServoJ  →  ServoJ(J1,...,J6,t=<s>,aheadtime=<n>,gain=<n>)  [keyword args]
    MovJ    →  MovJ(joint={J1,...,J6})                           [braces obrigatórias]
    V3 usava JointMovJ(…) e ServoJ posicional — retornam -10000/-50001 em V4.

Modo de uso típico (do GraspExecutor ou da GUI manual):

    drv = CR10RealDriver(ip='192.168.5.2', dry_run=False)
    drv.connect()
    drv.enable()                                # ClearError + EnableRobot + presets
    drv.servo_j([0, math.pi/2, 0, math.pi/2, 0, 0])  # convenção DOBOT, RADIANO
    q = drv.read_joints_rad()                   # 6 floats em radianos
    drv.stop()                                  # DisableRobot
    drv.close()

Observações:
    - ServoJ NÃO é afetado por SpeedFactor; o ritmo é dado pelo seu intervalo
      de envio (recomendado 30 ms / 33 Hz).
    - Use `dry_run=True` para validar o pipeline sem hardware — todos os
      sends apenas vão para o log.
    - O conversor URDF↔DOBOT está em `kinematics.urdf_to_dobot` / `dobot_to_urdf`
      (offsets das juntas 2 e 4 = ±π/2). NUNCA passe q_urdf direto: use
      `drv.servo_j_urdf(q_urdf)` ou converta antes.
"""
from __future__ import annotations

import logging
import math
import re
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

# Tentar reusar a conversão URDF↔DOBOT do módulo kinematics; queda atrasada
# para evitar import circular se este módulo for usado isoladamente.
try:
    from .kinematics import urdf_to_dobot, dobot_to_urdf  # noqa: F401
    _HAS_CONV = True
except Exception:  # pragma: no cover
    _HAS_CONV = False

log = logging.getLogger('touch_pack.real_driver')


# Portas TCP do controlador CR10
DASH_PORT     = 29999   # todos os comandos (dashboard + motion)
FEEDBACK_PORT = 30004   # struct 1440 B @ 8 ms (125 Hz)

# Offset (em bytes) do campo `q_actual` (6 × float64) dentro do struct de
# 1440 B do feedback.
FEEDBACK_Q_ACTUAL_OFFSET = 432
# Offset (em bytes) do campo `actual_TCPForce` (6 × float64 = [Fx,Fy,Fz,Tx,Ty,Tz])
# dentro do struct de 1440 B do feedback. Assume layout:
#   q_actual(432) → qd_actual(480) → qdd_actual(528) → i_actual(576)
#   → tool_vector_actual(624) → TCPSpeed_actual(672) → TCPForce(720)
FEEDBACK_TCP_FORCE_OFFSET = 720
FEEDBACK_PACKET_SIZE = 1440

# Modo DragTeach reportado por RobotMode() no firmware V4.5.1.
ROBOT_MODE_DRAG = 9

# Todo pacote do feedback começa com MessageSize == 1440 (uint16 LE). É o
# único marcador de fronteira que o stream oferece, e é o que permite
# RESSINCRONIZAR: ler outros 1440 B depois de um desalinhamento preserva o
# mesmo deslocamento, então é preciso procurar o marcador dentro do bloco.
FEEDBACK_MARKER = struct.pack('<H', FEEDBACK_PACKET_SIZE)


@dataclass
class CR10RealDriverConfig:
    ip: str = '192.168.5.2'
    dashboard_port: int = DASH_PORT      # 29999 — todos os comandos
    feedback_port: int = FEEDBACK_PORT   # 30004 — struct 1440 B
    connect_timeout_s: float = 3.0
    recv_timeout_s: float = 0.050    # timeout geral recv (era 1.0 — bloqueava o loop)
    servoj_recv_timeout_s: float = 0.008  # ServoJ: desiste da leitura em 8 ms
    speed_factor: int = 10           # 10% — responsivo para mirror slider
    collision_level: int = 3         # 0 = off; 3 = padrão CR
    payload_kg: float = 0.5          # mão COVVI ≈ 0.5 kg
    payload_cog_m: tuple = (0.0, 0.0, 0.05)
    servoj_period_s: float = 0.030   # 33 Hz recomendado
    servoj_lookahead: int = 20       # [20,100]; 20 = resposta imediata (era 50)
    servoj_gain: int = 500           # [200, 1000]
    readonly: bool = False           # True = só leitura (pula RequestControl)


# Faixa do `t` do ServoJ, do "Dobot TCP/IP Remote Control Interface Guide
# V4.5.1": "value range: [0.02,3600.0]". O mesmo documento recomenda 33 Hz
# (30 ms) como cadência de chamada. Consequência prática que vale registrar:
# com o mínimo de 20 ms e os 8 pontos por período que a onda exige, a maior
# frequência de modulação FISICAMENTE rastreável é 1/(0,02·8) = 6,25 Hz.
SERVOJ_T_MIN_S = 0.020
SERVOJ_T_MAX_S = 3600.0


class CR10RealDriverError(RuntimeError):
    """Erro genérico da camada CR10."""


class CR10RealDriver:
    """Encapsula os dois sockets TCP do controlador CR10."""

    def __init__(self, ip: str = '192.168.5.2', dry_run: bool = False,
                 config: CR10RealDriverConfig | None = None):
        self.cfg = config or CR10RealDriverConfig()
        self.cfg.ip = ip
        self.dry_run = dry_run

        self._dash: socket.socket | None = None
        self._feed: socket.socket | None = None

        self._dash_lock = threading.Lock()    # serializa sends/recvs em 29999
        self._feed_lock = threading.Lock()    # serializa recv no feedback (30004)
        self._enabled = False
        self._last_send_t = 0.0
        # Houve ServoJ desde a última drenagem? Decide se a drenagem precisa
        # ESPERAR por ACKs em voo ou pode ser não-bloqueante (ver
        # _drain_stale_responses).
        self._servoj_streamed = False

        self._keepalive_thread: threading.Thread | None = None
        self._keepalive_stop = threading.Event()
        # ServoJ roda a 33 Hz: o aviso de `t` fora da faixa sai UMA vez, senão
        # inunda o log a cada ponto.
        self._servoj_t_warned = False
        # E-STOP com trava: True entre EmergencyStop(1) e EmergencyStop(0).
        # Enquanto estiver ligado, nenhum comando de movimento deve sair.
        self._estop_engaged = False

    # conexão
    def _request_control_with_retry(self, retries: int = 4,
                                    delay_s: float = 0.5) -> bool:
        """Tenta obter o token de controle exclusivo, com backoff entre tentativas.

        Retorna True se obteve o token; False se esgotou as tentativas.
        O token é retido pela sessão anterior até o TCP detectar a desconexão
        (pode levar segundos a minutos). O retry resolve o caso mais comum.
        """
        for attempt in range(1, retries + 1):
            resp = self._send_dash('RequestControl()')
            log.info('[DASH] RequestControl (tentativa %d/%d) → %s',
                     attempt, retries, resp)
            if not resp or resp.startswith('0'):
                log.info('[DASH] Token de controle obtido na tentativa %d', attempt)
                return True
            if resp.startswith('-10000'):
                # -10000 é "comando desconhecido" (ver cabeçalho deste módulo),
                # não "token negado": o controlador desta bancada não implementa
                # RequestControl() — só firmwares com sessão exclusiva o têm.
                # Sem token a adquirir, a sessão de controle é válida como está;
                # retentar um comando inexistente 4× só atrasava a conexão e
                # produzia um erro que acusava um segundo PC que não existe.
                log.info('[DASH] RequestControl() não suportado por este '
                         'firmware (-10000) — controlador sem token exclusivo, '
                         'seguindo sem ele.')
                return True
            if attempt < retries:
                time.sleep(delay_s)
        log.warning(
            '[DASH] RequestControl: token não obtido após %d tentativas. '
            'Causa provável: controlador em modo LOCAL (pendant tem prioridade). '
            'Para usar DragTeachSwitch via software: mude para modo REMOTE no '
            'teach pendant (Settings → Operate Mode → Remote) ou na interface '
            'web http://192.168.5.2. Alternativa: use o botão físico de drag '
            'no antebraço do robô (não exige token TCP).',
            retries)
        return False

    def is_connected(self) -> bool:
        """True se dashboard (29999) e feedback (30004) estão abertos."""
        if self.dry_run:
            return True
        return self._dash is not None and self._feed is not None

    def connect(self) -> None:
        """Abre as conexões TCP nas portas 29999 e 30004. Idempotente."""
        if self.dry_run:
            log.info('[DRY-RUN] connect() → noop')
            return
        if self.is_connected():
            return
        try:
            self._dash = socket.create_connection(
                (self.cfg.ip, self.cfg.dashboard_port),
                timeout=self.cfg.connect_timeout_s)
            self._feed = socket.create_connection(
                (self.cfg.ip, self.cfg.feedback_port),
                timeout=self.cfg.connect_timeout_s)
            self._dash.settimeout(self.cfg.recv_timeout_s)
            self._feed.settimeout(self.cfg.recv_timeout_s)
            # O dashboard envia um banner na conexão; descartá-lo antes de
            # enviar comandos para não deslocar o emparelhamento cmd→resposta.
            self._drain_welcome()
            # RequestControl() obtém o token EXCLUSIVO de controle. Falhar
            # aqui e conectar assim mesmo era o pior dos mundos: a GUI
            # mostrava "conectado", o operador começava um ensaio e cada
            # comando de movimento era recusado pelo controlador — que já
            # pertencia a outro PC (ou ao pendant em modo LOCAL). O sintoma
            # chegava como "o robô não se mexe", longe da causa.
            #
            # Sessão de CONTROLE sem token não é uma sessão degradada, é uma
            # sessão inválida: recusa a conexão e diz quem provavelmente está
            # com o token. Quem só quer observar usa readonly=True, que nem
            # pede o token e continua conectando.
            if not self.cfg.readonly:
                if not self._request_control_with_retry():
                    raise CR10RealDriverError(
                        f'Controle EXCLUSIVO de {self.cfg.ip} negado: o token '
                        'já pertence a outra sessão. Causas, nesta ordem: '
                        '(1) outro PC está conectado ao robô — desconecte-o '
                        'primeiro, ou espere o TCP daquela sessão expirar; '
                        '(2) o controlador está em modo LOCAL e o teach '
                        'pendant tem prioridade — mude para REMOTE em '
                        f'Settings -> Operate Mode, ou em http://{self.cfg.ip}. '
                        'Para só LER o estado sem controlar, conecte com '
                        'readonly=True.')
            # Keep-alive: envia RobotMode() a cada 50 s para evitar timeout.
            self._start_keepalive()
        except OSError as exc:
            self.close()
            raise CR10RealDriverError(
                f'Falha ao abrir sockets para {self.cfg.ip}: {exc}') from exc
        except BaseException:
            # _request_control_with_retry() levanta CR10RealDriverError, que o
            # handler acima não pega — e a essa altura os sockets já estão
            # abertos (e o keepalive talvez vivo). Sem este close o driver fica
            # meio-aberto: a thread de keepalive segura uma referência a `self`,
            # então nem o GC recolhe e os sockets ficam pendurados no
            # controlador. Fecha e repropaga sem alterar o erro.
            self.close()
            raise

    def close(self) -> None:
        """Fecha as conexões TCP. Não desabilita o robô — chame stop() antes."""
        self._stop_keepalive()
        for attr in ('_dash', '_feed'):
            s = getattr(self, attr)
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass
            setattr(self, attr, None)
        self._enabled = False
        # Sockets fechados: não há ACK em voo para a próxima conexão esperar.
        self._servoj_streamed = False

    # keep-alive
    def _start_keepalive(self) -> None:
        """Inicia thread daemon que envia RobotMode() a cada 50 s."""
        if self.dry_run:
            return
        self._keepalive_stop.clear()
        self._keepalive_thread = threading.Thread(
            target=self._keepalive_loop, daemon=True, name='cr10-keepalive')
        self._keepalive_thread.start()

    def _stop_keepalive(self) -> None:
        self._keepalive_stop.set()
        t = self._keepalive_thread
        if t is not None and t.is_alive():
            t.join(timeout=1.0)
        self._keepalive_thread = None

    def _keepalive_loop(self) -> None:
        while not self._keepalive_stop.wait(timeout=50.0):
            if not self.is_connected():
                break
            try:
                # MUST read response (expect_reply=True default); sending without
                # reading leaves the response in the socket buffer and the next
                # command reads the wrong (stale) response.
                resp = self._send_dash('RobotMode()')
                log.debug('[KA] RobotMode → %s', resp.strip())
            except CR10RealDriverError:
                break

    def _drain_welcome(self) -> None:
        """Descarta o banner inicial enviado pelo dashboard (porta 29999)."""
        if self._dash is None:
            return
        self._dash.settimeout(0.5)
        try:
            buf = b''
            while True:
                chunk = self._dash.recv(4096)
                if not chunk:
                    break
                buf += chunk
                if b'\n' in buf or b';' in buf:
                    break
        except socket.timeout:
            pass
        finally:
            self._dash.settimeout(self.cfg.recv_timeout_s)
        if buf:
            log.info('[DASH] welcome: %s',
                     buf.decode('ascii', errors='replace').strip())

    # primitivas TCP ASCII
    @staticmethod
    def _recv_line(sock: socket.socket) -> tuple[str, bool]:
        """Lê uma linha ASCII terminada em ';' ou '\\n'.

        Devolve ``(texto, completa)``. O segundo campo existe porque o
        timeout é engolido AQUI: sem ele, quem chama não distinguia três
        situações que exigem tratamento diferente — resposta inteira,
        resposta TRUNCADA no meio (timeout com bytes no buffer) e silêncio
        total. Tratar uma resposta truncada como inteira é o caso perigoso:
        `_wait_mode` parseia o número do modo e passaria a acreditar num
        valor cortado ao meio.
        """
        buf = b''
        complete = False
        try:
            while b'\n' not in buf and b';' not in buf:
                chunk = sock.recv(2048)
                if not chunk:
                    break          # peer fechou: o que veio é o que há
                buf += chunk
            else:
                complete = True
        except socket.timeout:
            pass
        return buf.decode('ascii', errors='replace').strip(), complete

    # Espera pelo ACK de ServoJ que já saiu do robô mas ainda não chegou.
    # Um round-trip de LAN é da ordem de 1 ms; 50 ms cobre com folga sem
    # pesar, porque só é gasto depois de ter havido streaming.
    _DRAIN_SETTLE_S = 0.05

    def _drop_dash(self) -> None:
        """Descarta o socket do dashboard FECHANDO-O antes de soltar a
        referência. Confiar no GC para fechar é implícito demais para um
        recurso de SO — e, em quem não tem refcount imediato, vaza."""
        s = self._dash
        self._dash = None
        if s is not None:
            try:
                s.close()
            except OSError:
                pass

    def _drain_stale_responses(self) -> None:
        """Consome respostas pendentes de ServoJ (não lidas por timeout) do
        socket 29999.

        Drenar em modo NÃO-BLOQUEANTE só varre o que já chegou. Depois de um
        streaming de ServoJ a 33 Hz há ACKs EM VOO — enviados pelo robô,
        ainda não recebidos —, e esses escapavam da varredura e viravam a
        resposta do comando seguinte. Em `stop()` isso é inócuo (a resposta é
        ignorada), mas `_wait_mode` PARSEIA a resposta: um ACK velho ali faz
        o driver ler um modo errado e concluir que habilitou quando não
        habilitou. Por isso, quando houve ServoJ desde a última drenagem, a
        varredura espera `_DRAIN_SETTLE_S` pelo que está a caminho.
        """
        if self._dash is None:
            return
        settle_s = self._DRAIN_SETTLE_S if self._servoj_streamed else 0.0
        self._dash.settimeout(settle_s)
        drained = 0
        try:
            while True:
                chunk = self._dash.recv(4096)
                if not chunk:
                    break
                drained += len(chunk)
                # Já esvaziou o que estava em voo: o resto é não-bloqueante.
                self._dash.settimeout(0.0)
        except (socket.timeout, BlockingIOError, OSError):
            pass
        finally:
            if self._dash is not None:
                self._dash.settimeout(self.cfg.recv_timeout_s)
        self._servoj_streamed = False
        if drained:
            log.debug('[DRAIN] %d bytes de ACKs ServoJ descartados', drained)

    def _send_dash(self, cmd: str, expect_reply: bool = True) -> str:
        """Envia Immediate command ao dashboard (29999) e devolve a resposta."""
        if self.dry_run:
            log.info('[DRY-RUN dash] %s', cmd)
            return ''
        if self._dash is None:
            raise CR10RealDriverError('Dashboard não conectado')
        with self._dash_lock:
            # Drena ACKs de ServoJ acumulados antes de enviar um comando que
            # espera sua própria resposta — evita cmd→resposta mismatch.
            if expect_reply:
                self._drain_stale_responses()
            log.debug('[DASH→] %s', cmd)
            try:
                self._dash.sendall((cmd + '\n').encode('ascii'))
            except OSError as exc:
                # BrokenPipeError, ConnectionResetError, etc. — socket
                # perdido.
                self._drop_dash()
                raise CR10RealDriverError(
                    f'Socket dashboard perdido ao enviar "{cmd}": {exc}') from exc
            if not expect_reply:
                return ''
            try:
                resp, complete = self._recv_line(self._dash)
            except OSError as exc:
                self._drop_dash()
                raise CR10RealDriverError(
                    f'Socket dashboard perdido ao receber resposta de "{cmd}": {exc}') from exc
            if resp and not complete:
                # Resposta cortada ao meio pelo timeout. Devolvê-la seria pior
                # que devolver nada: quem parseia (ex.: _wait_mode) acreditaria
                # num valor truncado. O resto da linha vira lixo no socket e é
                # drenado antes do próximo comando.
                log.warning('[DASH←] resposta TRUNCADA de "%s": %r — descartada',
                            cmd, resp)
                return ''
            log.debug('[DASH←] %s', resp)
            return resp

    def _send_motion(self, cmd: str) -> str:
        """Envia comando de motion pela porta 29999 (igual ao SDK de referência).

        Ponto único onde a trava do E-Stop morde: com a chave pressionada o
        braço está desabilitado e em alarme, então o comando só engordaria a
        fila para ser executado no rearme — que é exatamente o movimento
        inesperado que um E-Stop existe para impedir.
        """
        if self._estop_engaged:
            raise CR10RealDriverError(
                f'E-STOP ATIVO — comando de movimento recusado ({cmd[:40]}). '
                'Solte a chave (segundo toque no botão de E-STOP) para '
                'rearmar o braço.')
        self._last_send_t = time.time()
        return self._send_dash(cmd, expect_reply=True)

    # sequência de bring-up
    def _wait_mode(self, target: int, timeout_s: float = 8.0) -> bool:
        """Espera até RobotMode() == target. Retorna True se alcançado."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            resp = self.robot_mode() or ''
            m = re.search(r'\{(\d+)\}', resp)
            if m and int(m.group(1)) == target:
                return True
            time.sleep(0.2)
        return False

    # ── E-STOP com trava ─────────────────────────────────────────────────
    # O firmware modela o E-Stop como uma CHAVE, não como um pulso. Do guia
    # V4.5.1, EmergencyStop(mode): "mode int — Emergency stop mode. 1: press
    # the E-Stop switch, 0: release the E-Stop switch", e a descrição é
    # explícita sobre o que a soltura exige: "After the emergency stop, the
    # robot arm will be disabled and then alarm. You need to release the
    # E-Stop switch AND clear the alarm to re-enable the robot arm."
    #
    # Por isso rearmar é uma sequência de três passos, e não basta soltar:
    #   EmergencyStop(0) → ClearError() → enable()
    # O StopRobot+DisableRobot que o E-STOP da GUI usava antes NÃO é isto:
    # ele para o braço mas não engata a chave, então não havia estado que
    # exigisse uma ação deliberada para destravar.

    @property
    def estop_engaged(self) -> bool:
        """True enquanto a chave de E-Stop estiver PRESSIONADA por software."""
        return self._estop_engaged

    def emergency_stop(self) -> None:
        """Pressiona a chave de E-Stop — EmergencyStop(1). Idempotente.

        O braço fica desabilitado e em alarme; nenhum movimento é aceito até
        `release_emergency_stop()`. Deliberadamente NÃO desconecta: soltar a
        chave exige a mesma sessão de dashboard.
        """
        self._estop_engaged = True
        self._enabled = False
        if self.dry_run:
            log.info('[DRY-RUN dash] EmergencyStop(1)')
            return
        if self._dash is None:
            log.warning('[DASH] EmergencyStop(1) sem dashboard — só o estado '
                        'local foi travado.')
            return
        resp = self._send_dash('EmergencyStop(1)')
        log.warning('[DASH] EmergencyStop(1) → %s — chave PRESSIONADA; '
                    'rearmar exige EmergencyStop(0) + ClearError + enable.',
                    resp)

    def release_emergency_stop(self) -> None:
        """Solta a chave e REARMA o braço — EmergencyStop(0) + ClearError +
        enable(). É o segundo toque no botão de E-STOP.

        Levanta CR10RealDriverError se o rearme não completar: um E-Stop que
        "soltou" sem reabilitar deixaria a GUI dizendo que está pronta com o
        braço ainda em alarme.
        """
        if self.dry_run:
            self._estop_engaged = False
            log.info('[DRY-RUN dash] EmergencyStop(0) + ClearError + enable')
            return
        if self._dash is None:
            raise CR10RealDriverError(
                'Dashboard não conectado — não há como soltar o E-Stop.')
        resp = self._send_dash('EmergencyStop(0)')
        log.info('[DASH] EmergencyStop(0) → %s', resp)
        resp = self._send_dash('ClearError()')
        log.info('[DASH] ClearError (pós E-Stop) → %s', resp)
        time.sleep(0.3)          # o alarme cai de forma assíncrona
        # enable() refaz PowerOn/EnableRobot e ESPERA o modo 5; se o braço não
        # rearmar, ele levanta e o _estop_engaged fica True de propósito.
        self.enable()
        self._estop_engaged = False
        log.info('[DASH] E-Stop SOLTO e braço reabilitado.')

    def enable(self) -> None:
        """Executa a sequência de enable do CR10 (protocolo V4, firmware V4.5.1)."""
        if not self.is_connected() and not self.dry_run:
            raise CR10RealDriverError('Driver não está conectado')

        resp = self._send_dash('ClearError()')
        log.info('[DASH] ClearError → %s', resp)
        # Continue() after ClearError is required: ClearError clears the alarm
        # but leaves the motion queue paused — Continue() resumes it so MovJ
        # commands actually execute instead of being silently queued forever.
        resp = self._send_dash('Continue()')
        log.info('[DASH] Continue (pós-ClearError) → %s', resp)

        # PowerOn() ativa o subsistema de potência em V4. Ignorar erro se
        # já estiver ligado (pode retornar -2 / "already on").
        resp = self._send_dash('PowerOn()')
        log.info('[DASH] PowerOn → %s', resp)
        time.sleep(0.5)   # PowerOn é assíncrono; dar tempo ao firmware

        # EnableRobot() — V4: assíncrono, retorna imediatamente.
        # Tentar primeiro sem parâmetros (forma mais simples e compatível).
        resp = self._send_dash('EnableRobot()')
        log.info('[DASH] EnableRobot() → %s', resp)

        # Aguardar modo 5 (ENABLE). EnableRobot é assíncrono em V4.
        if not self._wait_mode(5, timeout_s=8.0):
            log.warning('[DASH] Modo 5 não atingido após EnableRobot() — '
                        'tentando EnableRobot(load,cx,cy,cz)...')
            cx, cy, cz = (v * 1000.0 for v in self.cfg.payload_cog_m)
            resp2 = self._send_dash(
                f'EnableRobot({self.cfg.payload_kg:.3f},{cx:.1f},{cy:.1f},{cz:.1f})')
            log.info('[DASH] EnableRobot(load,cog) → %s', resp2)
            if not self._wait_mode(5, timeout_s=8.0):
                mode = self.robot_mode()
                raise CR10RealDriverError(
                    f'EnableRobot falhou — modo atual: {mode}. '
                    f'Verifique E-STOP / botão físico no controlador.')

        resp = self._send_dash(f'SpeedFactor({self.cfg.speed_factor})')
        log.info('[DASH] SpeedFactor → %s', resp)
        resp = self._send_dash(f'SetCollisionLevel({self.cfg.collision_level})')
        log.info('[DASH] SetCollisionLevel → %s', resp)
        self._enabled = True
        log.info('CR10 habilitado em %s (SpeedFactor=%d, Coll=%d, Payload=%.2fkg)',
                 self.cfg.ip, self.cfg.speed_factor, self.cfg.collision_level,
                 self.cfg.payload_kg)

    def prepare_servoj(self) -> None:
        """Reinicia estado interno antes de iniciar o streaming ServoJ."""
        if self.dry_run:
            return
        resp = self._send_dash('ClearError()')
        log.info('[DASH] prepare_servoj ClearError → %s', resp)
        resp = self._send_dash('Continue()')
        log.info('[DASH] prepare_servoj Continue → %s', resp)
        time.sleep(0.050)   # era 0.200 — 50 ms é suficiente para estabilizar
        mode = self.robot_mode()
        log.info('[DASH] RobotMode antes do primeiro ServoJ: %s', mode)

    def stop(self) -> None:
        """Parada por software — Stop() seguido de DisableRobot()."""
        if self.dry_run:
            self._enabled = False
            return
        if self._dash is None:
            self._enabled = False
            return
        with self._dash_lock:
            # Depois de streaming ServoJ o socket está cheio de ACKs não
            # lidos; sem drenar, os _recv_line abaixo consomem ACKs velhos e
            # deixam as respostas de Stop/DisableRobot para o próximo comando.
            self._drain_stale_responses()
            n_sent = 0
            try:
                self._dash.sendall(b'Stop()\n')
                n_sent += 1
            except OSError:
                pass
            try:
                self._dash.sendall(b'DisableRobot()\n')
                n_sent += 1
            except OSError:
                pass
            # Drena as respostas pendentes com timeout curto. `_recv_line`
            # engole o timeout e devolve ('', False), então o laço termina
            # sozinho — não há exceção a capturar aqui.
            orig_to = self._dash.gettimeout()
            self._dash.settimeout(0.15)
            try:
                for _ in range(n_sent):
                    self._recv_line(self._dash)
            finally:
                # Restaurar o timeout num `finally`: se o socket morrer no
                # meio da drenagem, deixá-lo em 0,15 s envenenaria todo
                # comando seguinte que reusasse este objeto.
                if self._dash is not None:
                    self._dash.settimeout(orig_to)
        self._enabled = False

    # movimentação
    def servo_j(self, q_rad: Sequence[float]) -> None:
        """ServoJ — fluxo de setpoints articulares em RADIANO (convenção DOBOT)."""
        q = list(q_rad)
        if len(q) != 6:
            raise ValueError(f'servo_j requer 6 valores, recebeu {len(q)}')
        if self._estop_engaged:
            # ServoJ não passa por _send_motion (tem o seu próprio caminho de
            # baixa latência), então a trava precisa ser repetida aqui — é
            # justamente o fluxo que continuaria empurrando o braço a 33 Hz.
            raise CR10RealDriverError(
                'E-STOP ATIVO — streaming ServoJ recusado. Solte a chave '
                '(segundo toque no botão de E-STOP) para rearmar.')
        q_deg = [math.degrees(v) for v in q]
        # `t` SATURADO na faixa que o firmware aceita — última linha de defesa,
        # já que é aqui que o comando vira texto. "Dobot TCP/IP Remote Control
        # Interface Guide V4.5.1", ServoJ: "t (float): Running time of the
        # point, unit: s, value range: [0.02,3600.0]". Um período menor vindo
        # da config não acelera nada: o controlador recusa o ponto, e o
        # sintoma no braço é uma onda que simplesmente não sai.
        t_s = min(max(float(self.cfg.servoj_period_s), SERVOJ_T_MIN_S),
                  SERVOJ_T_MAX_S)
        if not self._servoj_t_warned and t_s != float(self.cfg.servoj_period_s):
            self._servoj_t_warned = True
            log.warning(
                '[DASH] servoj_period_s=%.4f s fora da faixa [%.2f, %.1f] do '
                'ServoJ — saturado em %.4f s. Corrija a configuração: o '
                'período pedido NÃO estava sendo executado.',
                self.cfg.servoj_period_s, SERVOJ_T_MIN_S, SERVOJ_T_MAX_S, t_s)
        cmd = 'ServoJ({values},t={t:.3f},aheadtime={la},gain={g})'.format(
            values=','.join(f'{v:.6f}' for v in q_deg),
            t=t_s,
            la=self.cfg.servoj_lookahead,
            g=self.cfg.servoj_gain)
        self._last_send_t = time.time()
        if self.dry_run:
            log.info('[DRY-RUN dash] %s', cmd)
            return
        if self._dash is None:
            raise CR10RealDriverError('Dashboard não conectado')
        with self._dash_lock:
            log.debug('[DASH→] %s', cmd)
            try:
                self._dash.sendall((cmd + '\n').encode('ascii'))
            except OSError as exc:
                self._drop_dash()
                raise CR10RealDriverError(
                    f'Socket dashboard perdido (ServoJ): {exc}') from exc
            # A partir daqui pode haver ACK deste ServoJ em VOO — ver
            # _drain_stale_responses.
            self._servoj_streamed = True
            # Lê resposta com timeout curto — não bloqueia o ciclo de 30 ms.
            self._dash.settimeout(self.cfg.servoj_recv_timeout_s)
            try:
                # Sem timeout no ACK: `_recv_line` devolve ('', False) e o
                # ACK chega depois, drenado antes do próximo comando. Não há
                # `except socket.timeout` aqui de propósito — `_recv_line`
                # engole o timeout e nunca o propaga.
                resp, complete = self._recv_line(self._dash)
                if resp and complete and not resp.startswith('0'):
                    code = resp.split(',')[0].strip()
                    log.warning('[MOVE] ServoJ erro %s', code)
                    if code in ('-50001', '-1', '-2', '-3'):
                        raise CR10RealDriverError(
                            f'ServoJ não executável ({code})')
            finally:
                if self._dash is not None:
                    self._dash.settimeout(self.cfg.recv_timeout_s)

    def servo_j_urdf(self, q_urdf: Sequence[float]) -> None:
        """Wrapper que aplica `urdf_to_dobot` antes de chamar `servo_j`."""
        q = np.asarray(q_urdf, dtype=np.float64)
        if _HAS_CONV:
            q = urdf_to_dobot(q)
        self.servo_j(q.tolist())

    def mov_j_joint_deg(self, q_deg: Sequence[float]) -> None:
        """MovJ articular — PTP em GRAUS (convenção DOBOT)."""
        q = list(q_deg)
        if len(q) != 6:
            raise ValueError('mov_j_joint_deg requer 6 valores')
        cmd = 'MovJ(joint={{{values}}})'.format(
            values=','.join(f'{v:.6f}' for v in q))
        resp = self._send_motion(cmd)
        log.debug('[MOVE] MovJ(joint) resp: %s', resp)
        if resp and not resp.startswith('0'):
            code = resp.split(',')[0].strip()
            err_id = self.get_error_id() or ''
            log.warning('[MOVE] MovJ(joint) falhou (code=%s, GetErrorID=%s) cmd=%s',
                        code, err_id.strip(), cmd)
            raise CR10RealDriverError(f'MovJ falhou: code={code}, GetErrorID={err_id.strip()}')

    def mov_j_cartesian(self, x: float, y: float, z: float,
                         rx: float, ry: float, rz: float) -> None:
        """MovJ Cartesiano — PTP até pose (x,y,z,rx,ry,rz) em mm/graus.

        Usa a sintaxe V4: MovJ(pose={x,y,z,rx,ry,rz}).
        """
        cmd = f'MovJ(pose={{{x:.3f},{y:.3f},{z:.3f},{rx:.3f},{ry:.3f},{rz:.3f}}})'
        resp = self._send_motion(cmd)
        if resp and not resp.startswith('0'):
            log.warning('[MOVE] MovJ(pose) erro: %s', resp.split(',')[0].strip())

    def mov_l_cartesian(self, x: float, y: float, z: float,
                         rx: float, ry: float, rz: float) -> None:
        """MovL Cartesiano — interpolação linear até pose em mm/graus."""
        cmd = f'MovL(pose={{{x:.3f},{y:.3f},{z:.3f},{rx:.3f},{ry:.3f},{rz:.3f}}})'
        resp = self._send_motion(cmd)
        if resp and not resp.startswith('0'):
            log.warning('[MOVE] MovL(pose) erro: %s', resp.split(',')[0].strip())

    def rel_movl_user(self, dx: float, dy: float, dz: float,
                       drx: float = 0.0, dry: float = 0.0,
                       drz: float = 0.0) -> None:
        """RelMovLUser — movimento relativo em mm/graus no frame usuário (User0)."""
        cmd = f'RelMovLUser({dx:.3f},{dy:.3f},{dz:.3f},{drx:.3f},{dry:.3f},{drz:.3f})'
        resp = self._send_motion(cmd)
        if resp and not resp.startswith('0'):
            log.warning('[MOVE] RelMovLUser erro: %s', resp.split(',')[0].strip())

    def halt(self) -> None:
        """Halt() — pausa o movimento atual sem desabilitar o robô."""
        try:
            resp = self._send_dash('Halt()')
            log.info('[DASH] Halt → %s', resp)
        except CR10RealDriverError as exc:
            log.warning('[DASH] Halt falhou: %s', exc)

    def stop_motion(self) -> None:
        """Stop() + Continue() — para o movimento E LIMPA a fila de motion,
        sem desabilitar o robô.
        """
        try:
            resp = self._send_dash('Stop()')
            log.info('[DASH] Stop → %s', resp)
        except CR10RealDriverError as exc:
            log.warning('[DASH] Stop falhou: %s', exc)
        try:
            resp = self._send_dash('Continue()')
            log.info('[DASH] Continue → %s', resp)
        except CR10RealDriverError as exc:
            log.warning('[DASH] Continue falhou: %s', exc)

    def drag_teach(self, enable: bool) -> None:
        """DragTeachSwitch(1|0) — habilita/desabilita modo de arrasto livre.

        LIGAR está sob a mesma trava de `_send_motion`: com a chave de E-Stop
        pressionada o braço está desabilitado e em alarme, e soltar os freios
        do arrasto ali é justamente o movimento inesperado que o E-Stop existe
        para impedir. DESLIGAR passa sempre — é o sentido seguro, e recusá-lo
        deixaria o modo de arrasto pendurado.
        """
        if enable and self._estop_engaged:
            raise CR10RealDriverError(
                'E-STOP ATIVO — drag teach recusado. Solte a chave (segundo '
                'toque no botão de E-STOP) para rearmar o braço.')
        if self.dry_run:
            log.info('[DRY-RUN] DragTeachSwitch(%d)', int(enable))
            return
        status = 1 if enable else 0
        if enable:
            # Retentar token de controle — pode não ter sido obtido na conexão.
            self._request_control_with_retry(retries=2, delay_s=0.3)
            resp_cl = self._send_dash('SetCollisionLevel(0)')
            log.info('[DASH] SetCollisionLevel(0) pré-drag → %s', resp_cl)
        resp = self._send_dash(f'DragTeachSwitch({status})')
        log.info('[DASH] DragTeachSwitch(%d) → %s', status, resp)
        if resp and not resp.startswith('0'):
            if enable:
                # Re-enable path failed — restore collision level and report.
                self._send_dash(f'SetCollisionLevel({self.cfg.collision_level})')
                code = resp.split(',')[0].strip()
                raise CR10RealDriverError(f'DragTeachSwitch(1) falhou (code={code})')
            else:
                # -1000/-1 on disable: gravity may have triggered servo alarms
                log.warning('[DASH] DragTeachSwitch(0) retornou %s — '
                            'ClearError + retry', resp.split(',')[0].strip())
                try:
                    self._send_dash('ClearError()')
                    self._send_dash('Continue()')
                    time.sleep(0.1)
                except CR10RealDriverError as _e:
                    log.warning('[DASH] ClearError pré-drag-off falhou: %s', _e)
                resp = self._send_dash('DragTeachSwitch(0)')
                log.info('[DASH] DragTeachSwitch(0) retry → %s', resp)
                if resp and not resp.startswith('0'):
                    code = resp.split(',')[0].strip()
                    raise CR10RealDriverError(
                        f'DragTeachSwitch(0) falhou após retry (code={code})')
        if enable:
            # Firmware briefly reports q_actual=0 during drag mode transition.
            # Wait for the controller to stabilise before the first feedback read.
            time.sleep(0.15)
        if not enable:
            resp_cl = self._send_dash(f'SetCollisionLevel({self.cfg.collision_level})')
            log.info('[DASH] SetCollisionLevel(%d) restaurado → %s',
                     self.cfg.collision_level, resp_cl)

    def pause(self) -> None:
        """Pause() — pausa a fila de movimentos (retomável com Continue())."""
        try:
            resp = self._send_dash('Pause()')
            log.info('[DASH] Pause → %s', resp)
        except CR10RealDriverError as exc:
            log.warning('[DASH] Pause falhou: %s', exc)

    def resume(self) -> None:
        """Continue() — retoma após Pause()."""
        try:
            resp = self._send_dash('Continue()')
            log.info('[DASH] Continue → %s', resp)
        except CR10RealDriverError as exc:
            log.warning('[DASH] Continue falhou: %s', exc)

    def sync(self, timeout_s: float = 30.0) -> None:
        """Bloqueia até o robô terminar o movimento (RobotMode == 5)."""
        if self.dry_run:
            return
        # The firmware takes a few hundred ms to process a MovJ command and
        # enter mode 7 (RUNNING).
        time.sleep(0.5)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            resp = self.robot_mode() or ''
            # resposta: "0,{5},RobotMode();" — extrair o inteiro entre { }
            m = re.search(r'\{(\d+)\}', resp)
            if m and int(m.group(1)) not in (7, 8):
                return
            time.sleep(0.1)
        log.warning('sync() timeout após %.1f s', timeout_s)

    # leitura
    def _recv_exact(self, n: int) -> bytes:
        """Lê exatamente `n` bytes do feedback. Chamador segura _feed_lock."""
        buf = b''
        while len(buf) < n:
            chunk = self._feed.recv(n - len(buf))
            if not chunk:
                raise CR10RealDriverError('Feedback port fechado')
            buf += chunk
        return buf

    def _realign_feedback(self, buf: bytes) -> bytes:
        """Descarta o prefixo até o próximo header e recompleta o pacote.

        Chamador segura _feed_lock. Buscar o marcador é o que efetivamente
        ressincroniza: sem isso, um deslocamento de k bytes sobrevive a
        qualquer número de leituras de 1440 B.
        """
        idx = buf.find(FEEDBACK_MARKER, 1)
        if idx < 0:
            # Nenhum header COMPLETO no bloco. Descartar tudo perderia um
            # marcador PARTIDO na fronteira (só o 1º byte coube no bloco), e
            # aí o deslocamento se perpetuaria — exatamente o defeito que
            # esta função existe para corrigir. Preserva a cauda do tamanho
            # do marcador menos 1.
            idx = FEEDBACK_PACKET_SIZE - (len(FEEDBACK_MARKER) - 1)
        tail = buf[idx:]
        return tail + self._recv_exact(FEEDBACK_PACKET_SIZE - len(tail))

    def read_feedback_raw(self) -> bytes:
        """Lê um pacote completo de 1440 B do feedback port, sincronizado."""
        if self.dry_run:
            return b'\x00' * FEEDBACK_PACKET_SIZE
        if self._feed is None:
            raise CR10RealDriverError('Feedback port não conectado')
        with self._feed_lock:
            buf = self._recv_exact(FEEDBACK_PACKET_SIZE)
            for attempt in range(4):
                msg_size = struct.unpack_from('<H', buf, 0)[0]
                if msg_size == FEEDBACK_PACKET_SIZE:
                    return buf
                log.debug('[FEED] pacote desalinhado (MessageSize=%d, attempt=%d)'
                          ' — ressincronizando pelo marcador', msg_size, attempt + 1)
                buf = self._realign_feedback(buf)
            # Devolver "melhor esforço" aqui alimentaria read_tcp_force() com
            # lixo silencioso (ele não valida faixa como read_joints_rad).
            raise CR10RealDriverError(
                'Feedback dessincronizado: 4 tentativas de realinhamento sem '
                'header válido')

    def read_joints_rad(self) -> np.ndarray:
        """Lê as 6 juntas atuais em radianos (convenção DOBOT)."""
        if self.dry_run:
            return np.zeros(6, dtype=np.float64)
        buf = self.read_feedback_raw()
        q_deg = np.frombuffer(
            buf, offset=FEEDBACK_Q_ACTUAL_OFFSET,
            count=6, dtype='<f8').copy()
        if not np.all(np.isfinite(q_deg)) or np.any(np.abs(q_deg) > 400.0):
            raise CR10RealDriverError(
                f'Leitura de juntas inválida (desalinhamento?): {q_deg}')
        return np.deg2rad(q_deg)

    def read_joints_urdf(self) -> np.ndarray:
        """Idem, mas já na convenção URDF (joint2 e joint4 ajustados)."""
        q = self.read_joints_rad()
        if _HAS_CONV:
            q = dobot_to_urdf(q)
        return q

    def read_joints_urdf_latest(self) -> np.ndarray:
        """Como read_joints_urdf() mas drena o backlog antes de ler."""
        if self.dry_run:
            return np.zeros(6, dtype=np.float64)
        if self._feed is None:
            raise CR10RealDriverError('Feedback port não conectado')
        with self._feed_lock:
            orig_to = self._feed.gettimeout()
            # flush não-bloqueante: descarta todo o backlog
            flushed = 0
            self._feed.settimeout(0.0)
            try:
                while True:
                    chunk = self._feed.recv(65536)
                    if not chunk:
                        raise CR10RealDriverError('Feedback port fechado')
                    flushed += len(chunk)
            except (BlockingIOError, socket.timeout):
                pass
            finally:
                self._feed.settimeout(orig_to)
            if flushed:
                log.debug('[FEED] drain: %d bytes descartados', flushed)
            # Lê o próximo pacote fresco. O flush pode ter parado no MEIO de
            # um pacote, então o alinhamento não é garantido: ressincroniza
            # pelo marcador, igual a read_feedback_raw.
            buf = self._recv_exact(FEEDBACK_PACKET_SIZE)
            for _attempt in range(4):
                msg_size = struct.unpack_from('<H', buf, 0)[0]
                if msg_size == FEEDBACK_PACKET_SIZE:
                    break
                log.debug('[FEED] latest: desalinhado após flush '
                          '(MessageSize=%d, attempt=%d)', msg_size, _attempt + 1)
                buf = self._realign_feedback(buf)
        q_deg = np.frombuffer(buf, offset=FEEDBACK_Q_ACTUAL_OFFSET,
                              count=6, dtype='<f8').copy()
        if not np.all(np.isfinite(q_deg)) or np.any(np.abs(q_deg) > 400.0):
            raise CR10RealDriverError(
                f'Leitura de juntas inválida: {q_deg}')
        q = np.deg2rad(q_deg)
        if _HAS_CONV:
            q = dobot_to_urdf(q)
        return q

    def read_tcp_force(self) -> np.ndarray:
        """Lê o wrench externo estimado no TCP a partir do feedback do CR10.

        Returns:
            np.ndarray (6,) — [Fx, Fy, Fz, Tx, Ty, Tz] em N / N·m no
            frame do TCP. Em `dry_run`, retorna zeros.
        """
        if self.dry_run:
            return np.zeros(6, dtype=np.float64)
        buf = self.read_feedback_raw()
        return np.frombuffer(
            buf, offset=FEEDBACK_TCP_FORCE_OFFSET,
            count=6, dtype='<f8').copy()

    # diagnóstico
    def robot_mode(self) -> str | None:
        """RobotMode() — retorna a string crua do dashboard (5 = habilitado)."""
        try:
            return self._send_dash('RobotMode()')
        except CR10RealDriverError:
            return None

    def get_angle_deg(self) -> str | None:
        """GetAngle() — útil como sanity check fora do feedback estruturado."""
        try:
            return self._send_dash('GetAngle()')
        except CR10RealDriverError:
            return None

    def get_error_id(self) -> str | None:
        """GetErrorID() — códigos de alarme activos no controlador."""
        try:
            return self._send_dash('GetErrorID()')
        except CR10RealDriverError:
            return None

    # DO da flange (24 V já alimenta a COVVI; ToolDOExecute opcional)
    def tool_do(self, index: int, on: bool) -> None:
        """ToolDOExecute(idx, 1|0) — DO_1/DO_2 do conector aviation M8."""
        self._send_dash(f'ToolDOExecute({index},{1 if on else 0})')

    # context manager
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.stop()
        finally:
            self.close()


# helpers livres
def resample_trajectory(q_points: Iterable[np.ndarray],
                         t_points: Iterable[float],
                         period_s: float = 0.030) -> list[np.ndarray]:
    """Reamostra uma trajetória articular (q_i, t_i) para uma malha uniforme."""
    qs = [np.asarray(q, dtype=np.float64) for q in q_points]
    ts = list(t_points)
    if not qs or len(qs) != len(ts):
        return []
    t0, tf = ts[0], ts[-1]
    n = max(2, int(round((tf - t0) / period_s)) + 1)
    out: list[np.ndarray] = []
    for i in range(n):
        t = t0 + i * period_s
        if t >= tf:
            out.append(qs[-1])
            break
        # busca segmento
        j = 0
        while j + 1 < len(ts) and ts[j + 1] < t:
            j += 1
        a = (t - ts[j]) / max(1e-9, ts[j + 1] - ts[j])
        out.append(qs[j] + a * (qs[j + 1] - qs[j]))
    return out
