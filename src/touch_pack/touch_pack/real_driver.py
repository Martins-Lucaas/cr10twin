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
from typing import Sequence

import numpy as np

# Sem fallback: sem a conversão o driver mandaria ângulos URDF como se fossem
# DOBOT (J1 de sinal trocado, J2/J4 com 90° a menos) em silêncio.
from .kinematics import urdf_to_dobot, dobot_to_urdf

log = logging.getLogger('touch_pack.real_driver')


# Portas TCP do controlador CR10
DASH_PORT     = 29999   # todos os comandos (dashboard + motion)
FEEDBACK_PORT = 30004   # struct 1440 B @ 8 ms (125 Hz)
# Mesmo struct a 200 ms. Para link SEM FIO: a 30004 são 180 kB/s contínuos
# CONTRA o fluxo de comandos, e Wi-Fi é half-duplex. Aqui o mesmo dado custa
# ~7 kB/s. Quem MEDE latência (latency_probe) tem de ficar na 30004.
FEEDBACK_PORT_SLOW = 30005

# Offset (em bytes) do campo `q_actual` (6 × float64) dentro do struct de
# 1440 B do feedback.
FEEDBACK_ROBOT_MODE_OFFSET = 24   # uint64, guia V4.5.1
FEEDBACK_Q_ACTUAL_OFFSET = 432
FEEDBACK_PACKET_SIZE = 1440

# Modo DragTeach reportado por RobotMode() no firmware V4.5.1.
ROBOT_MODE_DRAG = 9

# Todo pacote do feedback começa com MessageSize == 1440 (uint16 LE). É o
# único marcador de fronteira que o stream oferece, e é o que permite
# RESSINCRONIZAR: ler outros 1440 B depois de um desalinhamento preserva o
# mesmo deslocamento, então é preciso procurar o marcador dentro do bloco.
FEEDBACK_MARKER = struct.pack('<H', FEEDBACK_PACKET_SIZE)
# Os dois bytes do MessageSize também aparecem dentro dos dados. O guia V4.5.1
# põe uma constante em 48–55 ("Memory test standard value"), e é ela que
# distingue o header verdadeiro de um par A0 05 casual.
FEEDBACK_TEST_VALUE_OFFSET = 48
FEEDBACK_TEST_VALUE = struct.pack('<Q', 0x0123456789ABCDEF)


def _feedback_aligned(buf: bytes, at: int = 0) -> bool:
    i = at + FEEDBACK_TEST_VALUE_OFFSET
    return (buf[at:at + 2] == FEEDBACK_MARKER
            and buf[i:i + 8] == FEEDBACK_TEST_VALUE)


@dataclass
class CR10RealDriverConfig:
    ip: str = '192.168.5.2'
    dashboard_port: int = DASH_PORT      # 29999 — todos os comandos
    feedback_port: int = FEEDBACK_PORT   # 30004 — struct 1440 B
    connect_timeout_s: float = 3.0
    recv_timeout_s: float = 0.050    # feedback, e dashboard durante ServoJ
    # Resposta de comando do dashboard fora do streaming. 50 ms tomava resposta
    # lenta por silêncio, e o comando seguinte lia a resposta do anterior.
    cmd_timeout_s: float = 1.0
    servoj_recv_timeout_s: float = 0.008  # ServoJ: desiste da leitura em 8 ms
    speed_factor: int = 10           # 10% — responsivo para mirror slider
    collision_level: int = 3         # 0 = off; 3 = padrão CR
    # Distância de recuo após colisão (mm, [0,50]) e o que fazer em seguida.
    # SetPostCollisionMode só aceita 0=stop e 1=pause pelo TCP/IP; os modos
    # "entra em drag" e "fallback" do DobotStudio Pro não têm comando remoto.
    # 1 (pause) é o certo para palpação: a ponta para ENCOSTADA e o operador
    # decide, em vez de o projeto morrer no meio de um toque.
    collision_backoff_mm: float = 10.0
    post_collision_mode: int = 1
    payload_kg: float = 0.5          # mão COVVI ≈ 0.5 kg
    payload_cog_m: tuple = (0.0, 0.0, 0.05)
    # Razões de aceleração (%) de junta e linear, e o raio de continuous path.
    # O jerk, que o DobotStudio Pro expõe em Playback Setting, NÃO tem comando
    # no TCP/IP V4.5.1 — só pela GUI do fabricante.
    acc_j: int = 100
    acc_l: int = 100
    cp_ratio: int = 0                # 0 = para em cada ponto (padrão)
    # Índice do sistema de coordenadas de ferramenta escrito com o resultado
    # da calibração de pivô (tcp_calib). 0 é o flange e não pode ser alterado.
    tool_index: int = 1
    # False com outra ferramenta no flange (mão): o TCP medido é da ponteira.
    touch_tool: bool = True
    servoj_period_s: float = 0.030   # 33 Hz recomendado
    # [20,100]; 50 é o default do firmware (era 20 aqui até 17/09/2026). Os 20
    # supunham pontos chegando regulares — verdade no cabo, falsa no Wi-Fi, e
    # o guia V4.5.1 nomeia o preço: "A smaller aheadtime value [...] may cause
    # instability and jitter". Volte a 20 só com link cabeado.
    servoj_lookahead: int = 50
    servoj_gain: int = 500           # [200, 1000]
    readonly: bool = False           # True = só leitura (pula RequestControl)


# Faixa do `t` do ServoJ, do "Dobot TCP/IP Remote Control Interface Guide
# V4.5.1": "value range: [0.02,3600.0]". O mesmo documento recomenda 33 Hz
# (30 ms) como cadência de chamada. Consequência prática que vale registrar:
# com o mínimo de 20 ms e os 8 pontos por período que a onda exige, a maior
# frequência de modulação FISICAMENTE rastreável é 1/(0,02·8) = 6,25 Hz.
SERVOJ_T_MIN_S = 0.020
SERVOJ_T_MAX_S = 3600.0


# ErrorID de RETORNO de comando ("Dobot TCP/IP Remote Control Interface Guide
# V4.5.1", §5 Error Code). É o número que vem na primeira posição de toda
# resposta do dashboard — o que hoje aparecia cru em "MovJ falhou: code=-2".
# Não confundir com o GetErrorID(), que devolve as coleções de ALARME do
# controlador/servo (essas precisam do alarm_controller.json do controlador,
# que não acompanha a documentação: ver README).
DASH_ERROR_TEXT = {
    0: 'sem erro',
    -1: 'comando recebido mas falhou ao executar',
    -2: 'braço em ALARME — limpe o alarme (ClearError) e reenvie',
    -3: 'braço em PARADA DE EMERGÊNCIA — solte a chave, limpe o alarme',
    -4: 'braço SEM POTÊNCIA — ligue (PowerOn)',
    -5: 'script rodando/pausado — pare o script primeiro',
    -10000: 'comando inexistente',
    -20000: 'número de parâmetros errado',
    -30001: 'tipo do 1º parâmetro errado',
    -30002: 'tipo do 2º parâmetro errado',
    -40001: '1º parâmetro fora da faixa',
    -40002: '2º parâmetro fora da faixa',
    -50001: 'tipo do 1º parâmetro opcional errado',
    -50002: 'tipo do 2º parâmetro opcional errado',
    -60001: '1º parâmetro opcional fora da faixa',
    -60002: '2º parâmetro opcional fora da faixa',
}


def dash_error_text(resp: str) -> str:
    """Resposta crua do dashboard → 'code=-2 (braço em ALARME — …)'.

    Devolve a descrição quando o código é conhecido e o código sozinho quando
    não é: inventar texto para um número não documentado seria pior que o
    número cru.
    """
    code_txt = (resp or '').split(',')[0].strip()
    try:
        code = int(code_txt)
    except ValueError:
        return f'code={code_txt or "?"}'
    texto = DASH_ERROR_TEXT.get(code)
    return f'code={code}' + (f' ({texto})' if texto else '')


class CR10RealDriverError(RuntimeError):
    """Erro genérico da camada CR10."""


class CR10FeedbackTimeout(CR10RealDriverError):
    """O feedback não entregou um pacote a tempo. A conexão segue aberta e a
    leitura seguinte continua de onde esta parou; quem chama não tem q_real e
    não pode mover com base nele."""


# Janela após um envio em que o dashboard é de quem está comandando: consultas
# periódicas (keepalive, RobotMode) não podem segurar o lock do ServoJ.
_STREAMING_WINDOW_S = 1.0
_KEEPALIVE_IDLE_S = 50.0
# Prazo de resposta com o ServoJ fluindo: além disto o braço fica sem ponto.
_STREAMING_REPLY_S = 0.2
# Por quanto tempo uma resposta que não veio ainda pode chegar atrasada.
_OWED_S = 2.0
# Repetir estes é inócuo: a resposta atrasada de um vale por outro IGUAL, e um
# E-STOP ou rearme nunca fica sem confirmação por causa de uma resposta perdida.
_IDEMPOTENTES = frozenset((
    'emergencystop', 'stop', 'continue', 'clearerror', 'poweron',
    'enablerobot', 'disablerobot'))


def _reply_name(line: str) -> str:
    """Nome ecoado em 'ErrorID,{valores},Nome(params)' — '' se não houver."""
    m = re.search(r'\}\s*,\s*([A-Za-z_]\w*)\s*\(', line)
    return m.group(1).lower() if m else ''


def _cmd_key(cmd: str) -> str:
    """Nome + argumentos, sem espaços: o que o firmware ecoa."""
    return re.sub(r'\s', '', cmd).lower()


def _reply_key(line: str) -> str:
    return _cmd_key(line.split('}', 1)[1].lstrip(' ,')) if '}' in line else ''


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
        self._last_send_t = self._last_servoj_t = 0.0
        # Bytes do dashboard ainda sem terminador, e respostas devidas por
        # comandos que estouraram o prazo: (nome+args, validade).
        self._rx = b''
        self._owed: list[tuple[str, float]] = []
        # Comandos cujo eco já se viu trazer os argumentos exatos.
        self._eco_exato: set[str] = set()
        # ACK de erro de ServoJ visto fora do servo_j: o próximo o levanta.
        self._servoj_erro = ''
        # Pacote de feedback incompleto, retomado na leitura seguinte.
        self._feed_rx = b''
        self.last_robot_mode: int | None = None
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
        self._estop_gen = 0               # quantos E-STOP já foram pedidos
        self._estop_esperando = False     # um deles aguarda o _dash_lock

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
            if resp.startswith('0') or self.dry_run:
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
            # Sem NODELAY, Nagle retém o ServoJ (~70 B) enquanto houver
            # segmento não confirmado: a cadência sai regular a 33 Hz e chega
            # agrupada. Invisível a 1 ms de RTT, é o trava/solavanco a 20 ms.
            # Só na 29999 — pela 30004 nunca se envia nada.
            self._dash.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._dash.settimeout(self.cfg.recv_timeout_s)
            # A 30005 publica a cada 200 ms: o prazo curto da 30004 venceria
            # antes de quase todo pacote.
            self._feed.settimeout(
                max(self.cfg.recv_timeout_s, 0.300)
                if self.cfg.feedback_port == FEEDBACK_PORT_SLOW
                else self.cfg.recv_timeout_s)
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
            # Keep-alive: RobotMode() após 50 s ocioso, para evitar timeout.
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
        self._rx = self._feed_rx = b''
        self._owed.clear()
        self._servoj_erro = ''

    # keep-alive
    def _start_keepalive(self) -> None:
        """Inicia thread daemon que envia RobotMode() após 50 s sem envio."""
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
        while not self._keepalive_stop.wait(timeout=_KEEPALIVE_IDLE_S / 5):
            if not self.is_connected():
                break
            # Com ServoJ fluindo o link já está vivo, e o RobotMode seguraria
            # o lock do dashboard por dezenas de ms no meio do streaming.
            if time.time() - self._last_send_t < _KEEPALIVE_IDLE_S:
                continue
            try:
                # MUST read response (expect_reply=True default); sending without
                # reading leaves the response in the socket buffer and the next
                # command reads the wrong (stale) response.
                self._last_send_t = time.time()
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
        drained, buf = 0, self._rx
        try:
            while True:
                chunk = self._dash.recv(4096)
                if not chunk:
                    break
                drained += len(chunk)
                buf += chunk
                # Já esvaziou o que estava em voo: o resto é não-bloqueante.
                self._dash.settimeout(0.0)
        except (socket.timeout, BlockingIOError, OSError):
            pass
        finally:
            if self._dash is not None:
                self._dash.settimeout(self.cfg.recv_timeout_s)
        self._servoj_streamed = False
        # A cauda sem terminador fica: o resto dela ainda vai chegar, e colaria
        # na frente da resposta do próximo comando.
        self._rx = buf
        for line in self._split_rx():
            self._alheia(line)
        if drained:
            log.debug('[DRAIN] %d bytes de ACKs ServoJ descartados', drained)

    def _split_rx(self) -> list[str]:
        """Linhas completas acumuladas em `_rx`; a cauda parcial fica lá."""
        *lines, self._rx = re.split(rb'[;\n]', self._rx)
        return [s for s in (ln.decode('ascii', errors='replace').strip()
                            for ln in lines) if s]

    def _settle_owed(self, key: str) -> float:
        """Dá baixa na resposta atrasada `key` (nome+args; só o nome casa
        qualquer argumento). Devolve a validade dela, 0.0 se não havia."""
        now = time.monotonic()
        self._owed = [o for o in self._owed if o[1] > now]
        for i, (k, ate) in enumerate(self._owed):
            if k == key or k.split('(')[0] == key:
                del self._owed[i]
                return ate
        return 0.0

    def _alheia(self, line: str) -> None:
        """Linha que não é a resposta esperada por ninguém."""
        if _reply_name(line) == 'servoj':
            if not line.startswith('0'):
                self._servoj_erro = line
        else:
            self._settle_owed(_reply_key(line))

    def _read_reply(self, cmd: str, timeout_s: float) -> str:
        """Resposta ASSOCIADA a `cmd` pelo eco, ou '' no fim do prazo.

        Linhas de outros comandos (ACK de ServoJ, resposta atrasada, o mesmo
        comando com outros argumentos) são descartadas. Um comando idêntico a
        um que ficou sem resposta é ambíguo: a 1ª linha só conta se outra vier
        atrás dela, e a dívida vence — não é renovada. Chamador segura
        `_dash_lock`.
        """
        deadline = time.monotonic() + timeout_s
        sock = self._dash
        key = _cmd_key(cmd)
        name = key.split('(')[0]
        herdada = 0.0
        while True:
            lines = self._split_rx()
            for i, line in enumerate(lines):
                got, eco = _reply_name(line), _reply_key(line)
                if got and got != name:
                    self._alheia(line)
                    log.debug('[DASH←] fora de associação (esperava %s): %s',
                              name, line)
                    continue
                atrasada = 0.0
                if got and eco != key:
                    atrasada = self._settle_owed(eco)
                    if not atrasada and (
                            name in self._eco_exato
                            # pressionar e soltar nunca se confirmam
                            or name == 'emergencystop'
                            and not eco.endswith('()')):
                        atrasada = -1.0        # outros argumentos: não é nossa
                    if not atrasada and name not in _IDEMPOTENTES:
                        atrasada = self._settle_owed(name)
                elif got and name not in _IDEMPOTENTES:
                    herdada = atrasada = self._settle_owed(key)
                if atrasada:
                    log.warning('[DASH←] resposta atrasada de %s '
                                'descartada: %s', name, line)
                else:
                    if got and eco == key and '()' not in key:
                        self._eco_exato.add(name)
                    # O que veio atrás não é nosso: fica para a drenagem.
                    self._rx = b''.join(s.encode() + b';'
                                        for s in lines[i + 1:]) + self._rx
                    return line + ';'
            left = deadline - time.monotonic()
            # Um E-STOP espera este lock: quem aguarda resposta cede a vez.
            if left <= 0 or (self._estop_esperando
                             and key != 'emergencystop(1)'):
                break
            sock.settimeout(min(left, 0.050))
            try:
                chunk = sock.recv(2048)
            except socket.timeout:
                continue
            if not chunk:
                raise OSError('conexão encerrada pelo controlador')
            self._rx += chunk
        if self._rx:
            log.warning('[DASH←] resposta TRUNCADA de "%s": %r — descartada',
                        name, self._rx)
        self._owed.append((key, herdada or time.monotonic() + _OWED_S))
        return ''

    def _recusa_com_estop(self, cmd: str) -> None:
        if self._estop_engaged:
            raise CR10RealDriverError(
                f'E-STOP ATIVO — comando de movimento recusado ({cmd[:40]}). '
                'Solte a chave (segundo toque no botão de E-STOP) para '
                'rearmar o braço.')

    def _send_dash(self, cmd: str, expect_reply: bool = True,
                   motion: bool = False) -> str:
        """Envia Immediate command ao dashboard (29999) e devolve a resposta.

        '' = sem confirmação. Nunca repete o comando: o controlador pode tê-lo
        executado sem que a resposta chegasse.
        """
        if self.dry_run:
            log.info('[DRY-RUN dash] %s', cmd)
            return ''
        with self._dash_lock:
            if cmd == 'EmergencyStop(1)':
                self._estop_esperando = False
            # Conferido COM o lock: quem o segurava pode ter derrubado o
            # socket, e o E-STOP pode ter vindo durante a espera.
            if self._dash is None:
                raise CR10RealDriverError('Dashboard não conectado')
            if motion:
                self._recusa_com_estop(cmd)
            # Drena ACKs de ServoJ acumulados antes de enviar um comando que
            # espera sua própria resposta — evita cmd→resposta mismatch.
            if expect_reply:
                self._drain_stale_responses()
            log.debug('[DASH→] %s', cmd)
            name = cmd.split('(')[0].strip().lower()
            timeout_s = self.cfg.cmd_timeout_s
            agora = time.time()
            if (name == 'robotmode'
                    and agora - self._last_send_t < _STREAMING_WINDOW_S):
                timeout_s = self.cfg.recv_timeout_s
            elif (name not in _IDEMPOTENTES
                    and agora - self._last_servoj_t < _STREAMING_WINDOW_S):
                timeout_s = min(timeout_s, _STREAMING_REPLY_S)
            if name == 'clearerror':
                self._servoj_erro = ''
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
                resp = self._read_reply(cmd, timeout_s)
                self._dash.settimeout(self.cfg.recv_timeout_s)
            except OSError as exc:
                self._drop_dash()
                raise CR10RealDriverError(
                    f'Socket dashboard perdido ao receber resposta de "{cmd}": {exc}') from exc
            log.debug('[DASH←] %s', resp)
            return resp

    def _send_motion(self, cmd: str) -> str:
        """Envia comando de motion pela porta 29999 (igual ao SDK de referência).

        Ponto único onde a trava do E-Stop morde: com a chave pressionada o
        braço está desabilitado e em alarme, então o comando só engordaria a
        fila para ser executado no rearme — que é exatamente o movimento
        inesperado que um E-Stop existe para impedir.
        """
        self._recusa_com_estop(cmd)
        self._last_send_t = time.time()
        resp = self._send_dash(cmd, expect_reply=True, motion=True)
        if not resp and not self.dry_run:
            raise CR10RealDriverError(
                f'SEM CONFIRMAÇÃO de {cmd.split("(")[0]}: o controlador não '
                f'respondeu em {self.cfg.cmd_timeout_s:.1f} s. O movimento PODE '
                'ter sido aceito — não reenviado; confira a pose antes de '
                'comandar de novo.')
        return resp

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

    def emergency_stop(self) -> bool:
        """Pressiona a chave de E-Stop — EmergencyStop(1). Idempotente.

        O braço fica desabilitado e em alarme; nenhum movimento é aceito até
        `release_emergency_stop()`. Deliberadamente NÃO desconecta: soltar a
        chave exige a mesma sessão de dashboard.

        Devolve True SÓ quando o controlador confirmou o EmergencyStop(1) com
        ErrorID 0, e False quando só a trava de software está garantida (sem
        dashboard, socket perdido, recusa, silêncio, dry-run). São coisas diferentes e não podem ser anunciadas
        como uma: sem o comando no controlador o braço pode seguir executando
        o que já estava na fila de motion, e quem chama precisa dizer isso ao
        operador em vez de "robot disabled and alarmed".

        A trava local é armada ANTES de qualquer I/O — ela é o que faz
        `_send_motion`, `servo_j` e `drag_teach` recusarem comando na mesma
        hora, sem depender da rede.
        """
        self._estop_gen += 1
        self._estop_engaged = True
        self._enabled = False
        if self.dry_run:
            log.info('[DRY-RUN dash] EmergencyStop(1)')
            return False
        if self._dash is None:
            log.warning('[DASH] EmergencyStop(1) sem dashboard — só o estado '
                        'local foi travado. O braço NÃO foi alarmado: se '
                        'houver movimento na fila, ele continua.')
            return False
        try:
            # Repetir é seguro: pressionar a chave duas vezes é uma parada só.
            for _ in range(2):
                self._estop_esperando = True
                resp = self._send_dash('EmergencyStop(1)')
                if resp.startswith('0'):
                    log.warning('[DASH] EmergencyStop(1) → %s — chave '
                                'PRESSIONADA; rearmar exige EmergencyStop(0) '
                                '+ ClearError + enable.', resp)
                    return True
        except CR10RealDriverError as exc:
            # A trava local já está armada; o que falhou foi só o alcance ao
            # controlador. Não propaga: quem chama tem de seguir com o resto
            # do E-STOP (mão, mirror, trava da GUI) e avisar.
            log.error('[DASH] EmergencyStop(1) NÃO chegou ao controlador '
                      '(%s) — trava só em software.', exc)
            return False
        log.error('[DASH] EmergencyStop(1) sem confirmação (%s) — tentando '
                  'Stop + DisableRobot.', dash_error_text(resp) if resp
                  else 'sem resposta')
        self.stop()
        return False

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
        gen = self._estop_gen
        resp = self._send_dash('EmergencyStop(0)')
        log.info('[DASH] EmergencyStop(0) → %s', resp)
        if not resp.startswith('0'):
            raise CR10RealDriverError(
                'EmergencyStop(0) não confirmado '
                f'({dash_error_text(resp) if resp else "sem resposta"}) — '
                'o braço continua em parada.')
        resp = self._send_dash('ClearError()')
        log.info('[DASH] ClearError (pós E-Stop) → %s', resp)
        time.sleep(0.3)          # o alarme cai de forma assíncrona
        # enable() refaz PowerOn/EnableRobot e ESPERA o modo 5; se o braço
        # não rearmar, ele levanta e o _estop_engaged fica True de propósito.
        self.enable()
        # Um E-STOP pedido no meio do rearme vence o rearme.
        if self._estop_gen != gen:
            raise CR10RealDriverError('E-STOP pedido durante o rearme')
        self._estop_engaged = False
        log.info('[DASH] E-Stop SOLTO e braço reabilitado.')

    def enable(self) -> None:
        """Executa a sequência de enable do CR10 (protocolo V4, firmware V4.5.1)."""
        if not self.is_connected() and not self.dry_run:
            raise CR10RealDriverError('Driver não está conectado')

        resp = self._send_dash('ClearError()')
        log.info('[DASH] ClearError → %s', resp)
        # Stop() ANTES do Continue(): sem esvaziar a fila, o Continue retomaria
        # o movimento que estava nela quando o alarme caiu.
        resp = self._send_dash('Stop()')
        log.info('[DASH] Stop (pré-Continue) → %s', resp)
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

        # Estes dois são PROTEÇÃO: sem confirmação o braço estaria habilitado
        # com a velocidade e a detecção de colisão de quem conectou antes.
        for cmd in (f'SpeedFactor({self.cfg.speed_factor})',
                    f'SetCollisionLevel({self.cfg.collision_level})'):
            resp = self._send_dash(cmd)
            log.info('[DASH] %s → %s', cmd, resp)
            if not resp.startswith('0') and not self.dry_run:
                raise CR10RealDriverError(
                    f'{cmd} não confirmado '
                    f'({dash_error_text(resp) if resp else "sem resposta"})')
        self.apply_runtime_config()
        self._enabled = True
        log.info('CR10 habilitado em %s (SpeedFactor=%d, Coll=%d, Payload=%.2fkg)',
                 self.cfg.ip, self.cfg.speed_factor, self.cfg.collision_level,
                 self.cfg.payload_kg)

    def apply_runtime_config(self) -> None:
        """Carga, tratamento pós-colisão, acelerações e TCP calibrado.

        Tudo que o DobotStudio Pro guarda no controlador e que o modo TCP/IP
        não herda automaticamente. Nenhum destes comandos é fatal: um firmware
        sem algum deles responde erro e o braço continua utilizável, então
        falha vira warning em vez de exceção.
        """
        cx, cy, cz = (v * 1000.0 for v in self.cfg.payload_cog_m)
        cmds = [
            # A carga entra na detecção de colisão e na estimativa de TCPForce
            # que o próprio braço publica no feedback — com ela errada, as
            # duas ficam erradas junto.
            f'SetPayload({self.cfg.payload_kg:.3f},{cx:.1f},{cy:.1f},{cz:.1f})',
            f'SetBackDistance({self.cfg.collision_backoff_mm:.1f})',
            f'SetPostCollisionMode({self.cfg.post_collision_mode})',
            f'AccJ({self.cfg.acc_j})',
            f'AccL({self.cfg.acc_l})',
            f'CP({self.cfg.cp_ratio})',
        ]
        for cmd in cmds:
            try:
                resp = self._send_dash(cmd)
            except CR10RealDriverError as exc:
                log.warning('[DASH] %s não enviado: %s', cmd.split('(')[0], exc)
                continue
            if resp and not resp.startswith('0'):
                log.warning('[DASH] %s → %s', cmd, dash_error_text(resp))
            else:
                log.info('[DASH] %s → %s', cmd, resp)
        self.apply_tool_offset()

    def apply_tool_offset(self) -> None:
        """Escreve o TCP calibrado no Tool(index) do controlador e o seleciona.

        Só afeta MovL/RelMovL*/leitura de pose do CONTROLADOR — a IK do pacote
        usa o mesmo número por outro caminho (kinematics.T_TOUCH_TOOL_ATTACH,
        que o tcp_calib também corrige). Sem calibração salva não faz nada:
        mexer no Tool do controlador sem ter medido seria trocar um número
        errado por outro.
        """
        if not self.cfg.touch_tool:
            return
        from .tcp_calib import load_offset_mm
        off = load_offset_mm()
        if off is None:
            log.info('[DASH] sem tcp_calib.json — Tool() do controlador intacto')
            return
        x, y, z = off
        idx = self.cfg.tool_index
        # type=1 grava globalmente: o valor sobrevive à saída do modo TCP/IP,
        # que é o que se espera de uma calibração de ferramenta.
        for cmd in (f'SetTool({idx},{{{x:.3f},{y:.3f},{z:.3f},0,0,0}},1)',
                    f'Tool({idx})'):
            try:
                resp = self._send_dash(cmd)
            except CR10RealDriverError as exc:
                log.warning('[DASH] %s não enviado: %s', cmd.split('(')[0], exc)
                return
            if resp and not resp.startswith('0'):
                log.warning('[DASH] %s → %s', cmd, dash_error_text(resp))
                return
        log.info('[DASH] TCP calibrado (%.2f, %.2f, %.2f) mm → Tool(%d)',
                 x, y, z, idx)

    def brake(self, axis_id: int, released: bool) -> None:
        """Solta (True) ou trava (False) o freio de UMA junta.

        BrakeControl(axisID,value): value=1 solta o freio e a junta pode ser
        arrastada à mão. Só funciona com o braço DESABILITADO — o controlador
        devolve -1 caso contrário. Segure a junta antes de soltar: ela cai
        pelo próprio peso.
        """
        if not 1 <= int(axis_id) <= 6:
            raise ValueError(f'axis_id fora de [1,6]: {axis_id}')
        resp = self._send_dash(f'BrakeControl({int(axis_id)},{1 if released else 0})')
        if not resp.startswith('0') and not self.dry_run:
            raise CR10RealDriverError(
                f'BrakeControl(J{axis_id}) falhou: {dash_error_text(resp)} — '
                f'o braço precisa estar DESABILITADO para mexer nos freios.')
        log.info('[DASH] BrakeControl(J%d, %s) → %s',
                 axis_id, 'solto' if released else 'travado', resp)

    def stop(self) -> None:
        """Parada por software — Stop() seguido de DisableRobot()."""
        if self.dry_run:
            self._enabled = False
            return
        if self._dash is None:
            self._enabled = False
            return
        with self._dash_lock:
            if self._dash is None:
                self._enabled = False
                return
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
            except OSError:
                pass            # socket morreu: não há o que drenar
            finally:
                # Restaurar o timeout num `finally`: se o socket morrer no
                # meio da drenagem, deixá-lo em 0,15 s envenenaria todo
                # comando seguinte que reusasse este objeto.
                if self._dash is not None:
                    self._dash.settimeout(orig_to)
        self._enabled = False

    # movimentação
    @staticmethod
    def _require_finite(cmd: str, q) -> None:
        if not all(math.isfinite(v) for v in q):
            raise CR10RealDriverError(
                f'{cmd} recusado: junta(s) não-finita(s) em {list(q)}. '
                'A origem é a fonte das juntas (/joint_states ou a IK), '
                'não o driver.')

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
        # NÃO-FINITO barrado aqui, e não a montante: o deadband dos
        # espelhamentos compara `max|q - last| < eps`, e toda comparação com
        # NaN é falsa — um NaN vindo do /joint_states não é filtrado por ele,
        # é justamente o que o atravessa. Sem esta guarda o comando vira o
        # texto `ServoJ(nan,...)` e o que acontece com o braço passa a depender
        # de como o firmware analisa a linha. Erguer o mesmo erro do E-STOP põe
        # o caminho no tratamento que os chamadores já têm.
        self._require_finite('ServoJ', q)
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
        with self._dash_lock:
            if self._dash is None:
                raise CR10RealDriverError('Dashboard não conectado')
            self._recusa_com_estop('ServoJ')     # pode ter vindo na espera
            if self._servoj_erro:
                erro, self._servoj_erro = self._servoj_erro, ''
                raise CR10RealDriverError(
                    f'ServoJ não executável ({dash_error_text(erro)})')
            log.debug('[DASH→] %s', cmd)
            try:
                self._dash.sendall((cmd + '\n').encode('ascii'))
            except OSError as exc:
                self._drop_dash()
                raise CR10RealDriverError(
                    f'Socket dashboard perdido (ServoJ): {exc}') from exc
            self._last_servoj_t = time.time()
            # A partir daqui pode haver ACK deste ServoJ em VOO — ver
            # _drain_stale_responses.
            self._servoj_streamed = True
            # Lê resposta com timeout curto — não bloqueia o ciclo de 30 ms.
            self._dash.settimeout(self.cfg.servoj_recv_timeout_s)
            try:
                # Sem ACK em 8 ms segue-se: ele chega depois e é lido na
                # próxima chamada. Por isso TODAS as linhas são conferidas — o
                # buffer pode trazer os ACKs de vários pontos, e o erro estar
                # em qualquer um.
                try:
                    chunk = self._dash.recv(2048)
                    if not chunk:
                        raise OSError('conexão encerrada pelo controlador')
                    self._rx += chunk
                except socket.timeout:
                    pass
                for line in self._split_rx():
                    name = _reply_name(line)
                    if name and name != 'servoj':
                        self._alheia(line)
                    elif not line.startswith('0'):
                        raise CR10RealDriverError(
                            f'ServoJ não executável ({dash_error_text(line)})')
            except OSError as exc:
                self._drop_dash()
                raise CR10RealDriverError(
                    f'Socket dashboard perdido (ServoJ): {exc}') from exc
            finally:
                if self._dash is not None:
                    self._dash.settimeout(self.cfg.recv_timeout_s)

    def servo_j_urdf(self, q_urdf: Sequence[float]) -> None:
        """Wrapper que aplica `urdf_to_dobot` antes de chamar `servo_j`."""
        q = urdf_to_dobot(np.asarray(q_urdf, dtype=np.float64))
        self.servo_j(q.tolist())

    def mov_j_joint_deg(self, q_deg: Sequence[float]) -> None:
        """MovJ articular — PTP em GRAUS (convenção DOBOT)."""
        q = list(q_deg)
        if len(q) != 6:
            raise ValueError('mov_j_joint_deg requer 6 valores')
        self._require_finite('MovJ', q)
        cmd = 'MovJ(joint={{{values}}})'.format(
            values=','.join(f'{v:.6f}' for v in q))
        resp = self._send_motion(cmd)
        log.debug('[MOVE] MovJ(joint) resp: %s', resp)
        if resp and not resp.startswith('0'):
            code = dash_error_text(resp)
            err_id = self.get_error_id() or ''
            log.warning('[MOVE] MovJ(joint) falhou (%s, GetErrorID=%s) cmd=%s',
                        code, err_id.strip(), cmd)
            raise CR10RealDriverError(f'MovJ falhou: {code}, GetErrorID={err_id.strip()}')

    def mov_l_joint_deg(self, q_deg: Sequence[float]) -> None:
        """MovL com alvo ARTICULAR (graus, convenção DOBOT): o controlador faz
        a FK do alvo e anda em linha reta até ele. Mesmo ponto final do MovJ,
        mas o TCP não faz arco — é o que o jog cartesiano promete."""
        q = list(q_deg)
        if len(q) != 6:
            raise ValueError('mov_l_joint_deg requer 6 valores')
        self._require_finite('MovL', q)
        cmd = 'MovL(joint={{{values}}})'.format(
            values=','.join(f'{v:.6f}' for v in q))
        resp = self._send_motion(cmd)
        if resp and not resp.startswith('0'):
            code = dash_error_text(resp)
            err_id = self.get_error_id() or ''
            log.warning('[MOVE] MovL(joint) falhou (%s, GetErrorID=%s) cmd=%s',
                        code, err_id.strip(), cmd)
            raise CR10RealDriverError(f'MovL falhou: {code}, GetErrorID={err_id.strip()}')

    def stop_motion(self) -> None:
        """Stop() + Continue() — para o movimento E LIMPA a fila de motion,
        sem desabilitar o robô.

        Levanta se o Stop não for confirmado (repete uma vez: parar é
        idempotente), e aí NÃO manda o Continue — ele retomaria a fila.
        """
        for _ in range(2):
            try:
                resp = self._send_dash('Stop()')
                log.info('[DASH] Stop → %s', resp)
            except CR10RealDriverError as exc:
                resp = str(exc)
                log.warning('[DASH] Stop falhou: %s', exc)
            if self.dry_run or resp.startswith('0'):
                break
        else:
            raise CR10RealDriverError(
                f'Stop() não confirmado ({resp or "sem resposta"}): o braço '
                'pode estar em movimento — use o E-STOP')
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
        level = f'SetCollisionLevel({self.cfg.collision_level})'
        ok = False
        try:
            if enable:
                # Retentar token — pode não ter sido obtido na conexão.
                self._request_control_with_retry(retries=2, delay_s=0.3)
                resp_cl = self._send_dash('SetCollisionLevel(0)')
                log.info('[DASH] SetCollisionLevel(0) pré-drag → %s', resp_cl)
                resp = self._send_dash('DragTeachSwitch(1)')
                log.info('[DASH] DragTeachSwitch(1) → %s', resp)
                if not resp.startswith('0'):
                    raise CR10RealDriverError(
                        'DragTeachSwitch(1) falhou '
                        f'({dash_error_text(resp) if resp else "sem resposta"})')
                ok = True
                # Firmware briefly reports q_actual=0 during drag mode
                # transition. Wait for it before the first feedback read.
                time.sleep(0.15)
                return
            resp = self._send_dash('DragTeachSwitch(0)')
            log.info('[DASH] DragTeachSwitch(0) → %s', resp)
            if not resp.startswith('0'):
                # -1000/-1 on disable: gravity may have triggered servo alarms
                log.warning('[DASH] DragTeachSwitch(0) retornou %r — '
                            'ClearError + retry', resp)
                for cmd in ('ClearError()', 'Stop()', 'Continue()'):
                    self._send_dash(cmd)
                time.sleep(0.1)
                resp = self._send_dash('DragTeachSwitch(0)')
                log.info('[DASH] DragTeachSwitch(0) retry → %s', resp)
                if not resp.startswith('0'):
                    raise CR10RealDriverError(
                        'DragTeachSwitch(0) falhou após retry '
                        f'({dash_error_text(resp) if resp else "sem resposta"})')
        finally:
            # Nível 0 só vale com o arrasto LIGADO e confirmado; todo outro
            # desfecho, inclusive exceção, devolve a detecção de colisão.
            if not ok:
                try:
                    resp_cl = self._send_dash(level)
                    log.info('[DASH] %s restaurado → %s', level, resp_cl)
                except CR10RealDriverError as exc:
                    log.error('[DASH] %s NÃO restaurado: %s', level, exc)

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

    def _recv_exact(self, n: int) -> bytes:
        """Lê exatamente `n` bytes do feedback. Chamador segura _feed_lock."""
        buf, self._feed_rx = self._feed_rx, b''
        while len(buf) < n:
            try:
                chunk = self._feed.recv(n - len(buf))
            except (socket.timeout, BlockingIOError) as exc:
                self._feed_rx = buf       # pacote fragmentado: retoma depois
                raise CR10FeedbackTimeout(
                    'Feedback sem pacote completo no prazo '
                    f'({len(buf)}/{n} bytes)') from exc
            except OSError as exc:
                raise CR10RealDriverError(f'Feedback perdido: {exc}') from exc
            if not chunk:
                raise CR10RealDriverError('Feedback port fechado')
            buf += chunk
        self._feed_rx = buf[n:]
        return buf[:n]

    def _realign_feedback(self, buf: bytes) -> bytes:
        """Descarta o prefixo até o próximo header e recompleta o pacote.

        Chamador segura _feed_lock. Buscar o marcador é o que efetivamente
        ressincroniza: sem isso, um deslocamento de k bytes sobrevive a
        qualquer número de leituras de 1440 B.
        """
        idx = buf.find(FEEDBACK_MARKER, 1)
        # Par A0 05 sem o TestValue atrás é dado, não header. Perto do fim do
        # bloco não dá para conferir: fica como candidato para a próxima volta.
        fim = len(buf) - FEEDBACK_TEST_VALUE_OFFSET - len(FEEDBACK_TEST_VALUE)
        while 0 <= idx <= fim and not _feedback_aligned(buf, idx):
            idx = buf.find(FEEDBACK_MARKER, idx + 1)
        if idx < 0:
            # Nenhum header COMPLETO no bloco. Descartar tudo perderia um
            # marcador PARTIDO na fronteira (só o 1º byte coube no bloco), e
            # aí o deslocamento se perpetuaria — exatamente o defeito que
            # esta função existe para corrigir. Preserva a cauda do tamanho
            # do marcador menos 1.
            idx = FEEDBACK_PACKET_SIZE - (len(FEEDBACK_MARKER) - 1)
        self._feed_rx = buf[idx:] + self._feed_rx
        return self._recv_exact(FEEDBACK_PACKET_SIZE)

    def _recv_packet(self) -> bytes:
        """Um pacote de feedback ALINHADO. Chamador segura _feed_lock."""
        buf = self._recv_exact(FEEDBACK_PACKET_SIZE)
        for _ in range(32):     # mais que os candidatos que cabem numa cauda
            if _feedback_aligned(buf):
                return buf
            log.debug('[FEED] pacote desalinhado — ressincronizando')
            buf = self._realign_feedback(buf)
        # Devolver "melhor esforço" aqui alimentaria quem lê com lixo.
        raise CR10RealDriverError(
            'Feedback dessincronizado: realinhamento sem header válido')

    def read_feedback_raw(self) -> bytes:
        """Lê um pacote completo de 1440 B do feedback port, sincronizado."""
        if self.dry_run:
            return b'\x00' * FEEDBACK_PACKET_SIZE
        if self._feed is None:
            raise CR10RealDriverError('Feedback port não conectado')
        with self._feed_lock:
            return self._recv_packet()

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
        return dobot_to_urdf(q)

    def read_joints_urdf_latest(self) -> np.ndarray:
        """Como read_joints_urdf() mas drena o backlog antes de ler."""
        if self.dry_run:
            return np.zeros(6, dtype=np.float64)
        if self._feed is None:
            raise CR10RealDriverError('Feedback port não conectado')
        with self._feed_lock:
            orig_to = self._feed.gettimeout()
            # flush não-bloqueante: descarta todo o backlog
            flushed = len(self._feed_rx)
            self._feed_rx = b''
            self._feed.settimeout(0.0)
            try:
                while True:
                    chunk = self._feed.recv(65536)
                    if not chunk:
                        raise CR10RealDriverError('Feedback port fechado')
                    flushed += len(chunk)
            except (BlockingIOError, socket.timeout):
                pass
            except OSError as exc:
                raise CR10RealDriverError(f'Feedback perdido: {exc}') from exc
            finally:
                self._feed.settimeout(orig_to)
            if flushed:
                log.debug('[FEED] drain: %d bytes descartados', flushed)
            # Lê o próximo pacote fresco. O flush pode ter parado no MEIO de
            # um pacote, então o alinhamento não é garantido: ressincroniza
            # pelo marcador, igual a read_feedback_raw.
            buf = self._recv_packet()
        self.last_robot_mode = struct.unpack_from(
            '<Q', buf, FEEDBACK_ROBOT_MODE_OFFSET)[0]
        q_deg = np.frombuffer(buf, offset=FEEDBACK_Q_ACTUAL_OFFSET,
                              count=6, dtype='<f8').copy()
        if not np.all(np.isfinite(q_deg)) or np.any(np.abs(q_deg) > 400.0):
            raise CR10RealDriverError(
                f'Leitura de juntas inválida: {q_deg}')
        q = np.deg2rad(q_deg)
        return dobot_to_urdf(q)

    # diagnóstico
    def robot_mode(self) -> str | None:
        """RobotMode() — retorna a string crua do dashboard (5 = habilitado)."""
        try:
            return self._send_dash('RobotMode()')
        except CR10RealDriverError:
            return None

    def get_error_id(self) -> str | None:
        """GetErrorID() — códigos de alarme activos no controlador."""
        try:
            return self._send_dash('GetErrorID()')
        except CR10RealDriverError:
            return None

    # context manager
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.stop()
        finally:
            self.close()
