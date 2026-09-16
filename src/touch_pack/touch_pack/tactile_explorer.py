"""
tactile_explorer.py — Backend ROS 2 da célula de palpação tátil.

Coreografia (Gupta et al., 2021) sobre uma SUPERFÍCIE HORIZONTAL.

    IDLE  →  HOME  →  DESCENDING  →  HOLD  →  SLIDING  →  HOME  →  IDLE

  Modo MANUAL/dinâmico: IDLE → HOME → DESCENDING → HOLD INFINITO → (STOP) →
  HOME → IDLE. No HOLD infinito o setpoint de força é atualizável on-the-fly
  via /palpation/set_force (Float32) — a transição para o novo alvo vem pelos
  micro-passos, sem reiniciar a FSM. Encerra só em STOP/force/stale e SEMPRE
  retorna à HOME.

Arquitetura de controle:
  Todas as fases de movimento usam STREAMING DIRETO de setpoints a 33 Hz
  (publicação em /cr10_group_controller/joint_trajectory, 1 ponto por
  mensagem). Não há action server nem trajetórias pré-planejadas:
  cada passo é calculado e enviado individualmente no loop de controle.

  Vantagens:
    - Sem fila de movimentos acumulada no controlador.
    - Nenhum movimento residual de uma fase carrega para a próxima
      (_settle() publica a posição atual repetidamente antes de toda
      transição, zerando qualquer lookahead pendente).
    - Velocidade explicitamente limitada pelo tamanho do passo
      (step_m = v_ms × dt), independente do SpeedFactor do controlador.

Fases:
  HOME         Interpolação linear no espaço de juntas a ≤ 0.3 rad/s.
  DESCENDING   Aproxima rápido até o contato; então RAMPA a velocidade
               constante leva a compressão ao setpoint (profundidade da
               GUI = curso máximo).
  HOLD         Congela a posição no setpoint, defende o patamar contra a
               relaxação viscoelástica e cumpre um dwell de medição.
  SLIDING      Streaming Jacobiano lateral com ALTURA TRAVADA em posição —
               a força fica livre para variar com a textura (sinal medido).

Controle de força (DESCENDING/HOLD/ESCADA/MANUAL):
  Setpoint selecionável na GUI (force_n, máx. 10 N). A regulação é
  QUASE-ESTÁTICA por RAMPA A VELOCIDADE CONSTANTE (ver _qs_regulate): o TCP
  desce ao longo do eixo de ataque a `hold_ramp_mms` mm/s até a força medida
  CRUZAR o setpoint, e então congela; se a força relaxa para fora da banda,
  a rampa é retomada sem reiniciar o relógio do patamar. Não há termo
  proporcional nem estimativa de rigidez na decisão — a rigidez só entra num
  clamp de segurança por tick (_QS_RAMP_DF_CAP_N). Overshoot = 1 tick de
  curso; tempo de patamar = curso/v + janela estável.

  A lei ANTERIOR (Δx=relax·err/K_est com tetos por k_upper — ver o bloco
  NÃO-ULTRAPASSAGEM nas constantes e _StiffnessEstimator) foi trocada: K_est
  é uma EMA do trecho JÁ percorrido de um contato que enrijece, subestima a
  inclinação seguinte e o passo err/K_est atravessava o alvo. O
  _StiffnessEstimator permanece porque a onda SINE/COSINE do modo TOUCH
  (feedforward Δx=ΔF/K, fora de _qs_regulate) ainda depende dele. No SLIDING
  a força NÃO é regulada — só monitorada por segurança. A medição é
  CANCELADA se a compressão exceder 15 N (_FORCE_ABORT_LIMIT_N).

Interface ROS:
  sub /palpation/start    touch_pack_msgs/PalpationStart
  sub /palpation/stop     std_msgs/String
  sub /palpation/pause    std_msgs/Bool     true=pausa (segura posição), false=retoma
  sub /palpation/freeze   std_msgs/Empty    parada DURA: congela no lugar, sem ir à HOME
  sub /palpation/set_force std_msgs/Float32  setpoint de força on-the-fly (modo MANUAL)
  sub /palpation/forget_contact std_msgs/Empty  esquece o contato aprendido da home corrente (checkbox "Home conhecida" da GUI)
  sub /load_cell/force_net std_msgs/Float32
  sub /joint_states       sensor_msgs/JointState
  pub /palpation/status   touch_pack_msgs/PalpationStatus
  pub /cr10_group_controller/joint_trajectory  (streaming direto)

Parâmetros ROS:
  approach_v_max_mms   50.0   velocidade inicial da descida (mm/s)
  approach_v_min_mms    5.0   velocidade final da descida (mm/s)

Calibração dinâmica do ângulo de ataque (opcional, desligada por padrão):
  Uma fase CALIBRATING roda UMA vez por experimento, logo após a HOME, e
  substitui a suposição "alvo perpendicular à home" por uma medição: N
  toques leves em torno do ponto de aproximação, ajuste do plano e ataque
  ao longo da normal medida. Ver o bloco de constantes _ALIGN_* e
  _phase_calibrate_attack.
  Configurável pela GUI (campos probe_align_* da PalpationStart, aba
  Palpação → Advanced); os parâmetros abaixo só valem quando a mensagem
  vem SEM o campo probe_align — o fluxo sem GUI.
  probe_align_enable        False  liga a calibração
  probe_align_points          4    toques de sonda (mín. 3)
  probe_align_radius_mm      15.0  raio do polígono de sondagem (no
                                   MATRIX_MAP o padrão vem da grade — ver
                                   _align_offsets)
  probe_align_force_n         1.0  setpoint dos toques (≤ force_n do run)
  probe_align_retract_mm     20.0  retração linear antes de girar o punho
  probe_align_tilt_max_deg   20.0  desvio máximo aceito (teto duro 30°)
"""
from __future__ import annotations

import collections
import json
import math
import os
import sys
import threading
import time
import traceback
from datetime import datetime

import numpy as np
if tuple(int(x) for x in np.__version__.split(".")[:2]) >= (2, 0):
    sys.exit(
        f"[ERRO] NumPy {np.__version__} detectado — ABI incompatível com "
        "ROS 2 Humble.\n"
        "Corrija: pip install 'numpy<2'\n"
        "Confirme com: python3 -c \"import numpy; print(numpy.__version__)\""
    )
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)

_QOS_COMMAND = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
_QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)

from std_msgs.msg import String, Float32, Bool, Empty
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from touch_pack_msgs.msg import (PalpationStart, PalpationStatus, MatrixPoint,
                                 LoadCellSample)

from .kinematics import (
    forward_kinematics, inverse_kinematics, jacobian,
    JOINT_MIN, JOINT_MAX, MIMIC_LIST as _MIMIC_LIST,
    T_TOUCH_TOOL_ATTACH,
)
from .plane_probe import (
    angle_between_deg as _angle_between_deg,
    attack_dir_from_normal as _attack_dir_from_normal,
    fit_plane as _fit_plane,
    probe_pattern as _probe_pattern,
    sagitta_m as _sagitta_m,
    SAGITTA_MAX_M_DEFAULT as _SAGITTA_MAX_M,
    probe_ring_from_grid as _probe_ring_from_grid,
    slope_along_deg as _slope_along_deg,
    validate_fit as _validate_fit,
)
from .lc_filter import (   # noqa: F401 — _MEDIAN_N é reexportado
    MEDIAN_N as _MEDIAN_N,
    ONE_EURO_MAXCUTOFF_HZ as _ONE_EURO_MAXCUTOFF_HZ,
)
from .constants import (   # noqa: F401 — alguns só são reexportados
    LC_NOMINAL_RATE_HZ as _LC_NOMINAL_RATE_HZ,
    SERVOJ_DEADBAND_RAD as _SERVOJ_DEADBAND_RAD,
    ARM_REACH_M as _ARM_REACH_M,
    ARM_JOINTS as _ARM_JOINTS,
    HAND_JOINTS as _HAND_PRIMARY,
    HAND_POINTING_RAD as _HAND_POINTING_RAD,
    POINTING_SEED_DEG as _POINTING_SEED_DEG,
    FORCE_ABORT_LIMIT_N as _FORCE_ABORT_LIMIT_N,
    FORCE_SETPOINT_MAX_N as _FORCE_SETPOINT_MAX_N,
    CONTACT_ON_N as _CONTACT_ON_N,
    FORCE_NOISE_SIGMA_N as _FORCE_NOISE_SIGMA_N,
    FORCE_CTRL_SIGMA_N as _FORCE_CTRL_SIGMA_N,
    HOLD_TOL_SIGMA as _HOLD_TOL_SIGMA,
    HOLD_TOL_N as _HOLD_TOL_N,
    HOLD_TOL_PCT as _HOLD_TOL_PCT,
    LEARNED_CONTACT_FILE as _LEARNED_CONTACT_FILE,
    tool_stamp as _tool_stamp,
    tool_stamp_mismatch as _tool_stamp_mismatch,
    MATRIX_SAFE_Z_MM_DEFAULT as _MATRIX_SAFE_Z_MM_DEFAULT,
    MATRIX_SAFE_Z_MM_MIN as _MATRIX_SAFE_Z_MM_MIN,
    MATRIX_SAFE_Z_MM_MAX as _MATRIX_SAFE_Z_MM_MAX,
    MATRIX_TRANSIT_MMS_DEFAULT as _MATRIX_TRANSIT_MMS_DEFAULT,
    MATRIX_TRANSIT_MMS_MIN as _MATRIX_TRANSIT_MMS_MIN,
    MATRIX_TRANSIT_MMS_MAX as _MATRIX_TRANSIT_MMS_MAX,
    MATRIX_MAX_POINTS as _MATRIX_MAX_POINTS,
    MATRIX_SPAN_MAX_MM as _MATRIX_SPAN_MAX_MM,
    PROBE_ALIGN_POINTS_DEFAULT as _ALIGN_POINTS_DEFAULT,
    PROBE_ALIGN_POINTS_MIN as _ALIGN_MIN_POINTS,
    PROBE_ALIGN_POINTS_MAX as _ALIGN_MAX_POINTS,
    PROBE_ALIGN_RADIUS_MM_DEFAULT as _ALIGN_RADIUS_MM,
    PROBE_ALIGN_RADIUS_MM_MIN as _ALIGN_RADIUS_MM_MIN,
    PROBE_ALIGN_RADIUS_MM_MAX as _ALIGN_RADIUS_MM_MAX,
    PROBE_ALIGN_FORCE_N_DEFAULT as _ALIGN_PROBE_FORCE_N,
    PROBE_ALIGN_RETRACT_MM_DEFAULT as _ALIGN_RETRACT_MM,
    PROBE_ALIGN_RETRACT_MM_MIN as _ALIGN_RETRACT_MM_MIN,
    PROBE_ALIGN_RETRACT_MM_MAX as _ALIGN_RETRACT_MM_MAX,
    PROBE_ALIGN_TILT_MAX_DEG_DEFAULT as _ALIGN_TILT_MAX_DEG,
    PROBE_ALIGN_TILT_HARD_MAX_DEG as _ALIGN_TILT_HARD_MAX_DEG,
    STEP_MAX_LEVELS,
    STAIRCASE_MODES,
    FMOD_MODES,
    staircase_levels,
)

# Os números do protocolo e os dois blocos que saíram daqui — a onda de
# força e a estimativa de rigidez. Reexportados de propósito:
# `tactile_explorer.X` continua resolvendo para quem já importava de cá.
from .explorer_constants import (   # noqa: F401 — reexportação
    _POINTING_SEED_Q, _SLIDING_SAFETY_M, _FORCE_SAFE_LIMIT_N,
    _K_DEFAULT_NM, _K_MIN_NM, _K_MAX_NM, _K_EMA_ALPHA,
    _K_PAIR_MIN_DF_N, _K_PAIR_MIN_DX_M, _FX_MIN_POINTS, _FX_MIN_SPAN_N,
    _FX_MIN_SEG_DF_N, _DEADBEAT_DX_MAX_M, _SLIDE_MAX_SINK_M,
    _SLIDE_CONTACT_MIN_FRAC, _SLIDE_LOST_BUDGET_M, _SLIDE_LOST_ABORTS,
    _SLIDE_SLOPE_MAX_DEG, _Z_HAT, _TARGET_LOST_FREE_M,
    _TARGET_LOST_DROP_S, _TARGET_LOST_DROP_FRAC, _CONTACT_ON_SAMPLES,
    _CONTACT_CONFIRM_S, _CONTACT_FALSE_MAX, _DESCEND_CONTACT_V_MAX_MS,
    _CONTACT_ZONE_MARGIN_M, _ZONE_REACTION_S, _CONTACT_MARGIN_MIN_PTS,
    _CONTACT_MARGIN_K, _CONTACT_MARGIN_FLOOR_M, _CONTACT_MARGIN_WINDOW,
    _LEARNED_TCP_TOL_M, _LEARNED_FLATNESS_M, _DESCEND_TOUCH_V_MS,
    _STREAM_HALT_LAT_S, _DESCEND_DECEL_ZONE_M, _DESCEND_CRAWL_V_MIN_MS,
    _QS_SETTLE_TICKS, _QS_MEDIAN_N, _QS_MEASURE_MAX_TICKS,
    _QS_SETTLE_NEAR_MULT, _QS_SETTLE_MAX_TICKS, _QS_SETTLE_DRIFT_N,
    _QS_RELAX, _QS_DF_MAX_N, _QS_DX_MAX_M, _QS_FREE_STEP_M,
    _QS_RELIEF_FLOOR_N, _QS_DF_DEAD_N, _QS_BOOST_MAX, _QS_DF_HARD_N,
    _QS_DF_TARGET_FRAC, _QS_DX_PROBE_M, _QS_DX_PROBE_MAX_M,
    _QS_FREE_STEP_MAX_M, _QS_K_PUSH_MARGIN, _QS_NO_CROSS_FRAC,
    _QS_STEP_GROWTH, _QS_DX_FLOOR_M, _QS_FREE_RESET_TICKS,
    _QS_ARRIVE_S, _QS_TIMEOUT_S, _QS_RAMP_V_MS, _QS_RAMP_DF_CAP_N,
    _HOLD_STABLE_S, _HOLD_TIMEOUT_S, _HOLD_DWELL_S, _ALIGN_ORI_TOL_DEG,
    _ALIGN_TRANSIT_MS, _ALIGN_ZONE_EXTRA_M, _STEP_START_DEFAULT_N,
    _STEP_DWELL_DEFAULT_S, _FMOD_SHAPES, _FMOD_MIN_PTS_PER_CYCLE,
    _FMOD_DT_MIN_S, _SERVOJ_T_MIN_S, _FMOD_K_ADAPT_ALPHA,
    _FMOD_K_ADAPT_MIN_DF_N, _FMOD_MAX_AMP_N, _FMOD_DF_STEP_MAX_N,
    _FMOD_RAMP_MAX_TICKS, _FMOD_V_MAX_MMS, _FMOD_FREQ_TOL_FRAC,
    _FMOD_AMP_RAMP_START, _FMOD_AMP_RAMP_CYCLES, _FMOD_ILC_BINS,
    _FMOD_ILC_ALPHA, _FMOD_ILC_MAX_FRAC, _FMOD_ILC_WARMUP_CYCLES,
    _FMOD_ILC_MIN_MEAS_GAIN, _FMOD_BAND_TOL_FRAC, _FMOD_BAND_TOL_MIN_N,
    _FMOD_V_PEAK_WARN_MMS, _FMOD_V_PEAK_MAX_MMS, _FMOD_CYCLES_DEFAULT,
    _FMOD_CLIP_LAG_FRAC, _FMOD_LIMIT_RECOVER_CYCLES,
    _FMOD_LIMIT_RECOVER_STEP, _FMOD_MIN_MEAS_RATE_MULT,
    _FMOD_DEADBAND_MIN_RATIO,
    _FMOD_QUIET_FLOOR_M, _FORCE_STALE_S, _START_MAX_AGE_S, _CTRL_DT,
    _CTRL_LOOK, _CTRL_WIN, _SLIDE_WIN, _JAC_LAM, _ORI_GAIN,
    _Z_CORR_GAIN, _HOME_MAX_RAD_S, _SETTLE_TICKS,
    _SETTLE_STILL_TOL_RAD_S, _SETTLE_STILL_MAX_TICKS,
    _MAX_JOINT_VEL_RAD_S, _FX_GAIN_MIN, _FX_GAIN_MAX,
)
from .force_wave import (           # noqa: F401 — reexportação
    _ForceProfile, _WaveILC, _WaveLockIn, _fmod_max_freq_hz,
    _fmod_sampling_gain, fmod_measure_gain, fmod_measure_lag_s,
    fmod_preflight
)
from .stiffness import (            # noqa: F401 — reexportação
    _ContactCurve, _StiffnessEstimator, crawl_v_ms, impact_peak_n,
    setpoint_resolvable
)


class TactileExplorer(Node):

    def __init__(self):
        super().__init__('tactile_explorer')

        self.declare_parameter('retract_mm',          80.0)
        self.declare_parameter('arm_base_z',          0.78)
        self.declare_parameter('approach_v_max_mms',  50.0)
        self.declare_parameter('approach_v_min_mms',   5.0)
        # Persistência do contato aprendido por home (ver _load_learned).
        # max_age_h = 0 desliga o vencimento (entradas valem para sempre).
        #
        # NASCE DESLIGADA, e o motivo é segurança. Um contato aprendido é a
        # licença para descer em VELOCIDADE CHEIA até a margem antes dele —
        # e essa licença vale sob uma hipótese que o software não tem como
        # verificar: que a peça e a fixação continuam onde estavam. Entre uma
        # sessão e a seguinte é exatamente o que muda (trocou a amostra,
        # reapertou o calço, remontou a ponteira), e o custo dos dois erros é
        # assimétrico: esquecer o aprendizado custa uma descida lenta, e
        # confiar nele com a peça mais alta custa uma colisão a 15 mm/s.
        #
        # O preço é conhecido: sem aprendizado a descida INTEIRA roda no
        # rastejo (v_unlearned_ms = v_slow_ms, ver _phase_descending), ~50
        # µm/s. De uma home a 24 mm do contato são ~8 min — uma vez por home
        # por sessão, porque dentro da sessão o aprendizado em memória segue
        # valendo (_remember_contact não consulta este parâmetro).
        #
        # Quem tiver uma bancada que NÃO se mexe entre sessões e quiser o
        # tempo de volta liga com learned_contact_persist:=true, e aí o aviso
        # do _load_learned diz sob que hipótese o arquivo está sendo lido.
        self.declare_parameter('learned_contact_persist', False)
        self.declare_parameter('learned_contact_max_age_h', 24.0)
        self.declare_parameter('descent_speed_mms', 5.0)
        # Velocidade NOMINAL da rampa fina de _qs_regulate até o setpoint
        # (mm/s). Efetivo ≈ 1/6 — ver _QS_RAMP_V_MS.
        self.declare_parameter('hold_ramp_mms', _QS_RAMP_V_MS * 1e3)
        self.declare_parameter('home_speed_rad_s', _HOME_MAX_RAD_S)
        # Calibração dinâmica do ângulo de ataque — ver o bloco _ALIGN_*.
        # Desligada por padrão: ligá-la muda a aproximação de TODO run.
        # Só valem quando a PalpationStart NÃO traz o campo probe_align
        # (fluxo sem GUI); com ele, a mensagem manda — ver _align_params.
        self.declare_parameter('probe_align_enable', False)
        self.declare_parameter('probe_align_points', _ALIGN_POINTS_DEFAULT)
        self.declare_parameter('probe_align_radius_mm', _ALIGN_RADIUS_MM)
        self.declare_parameter('probe_align_force_n', _ALIGN_PROBE_FORCE_N)
        self.declare_parameter('probe_align_retract_mm', _ALIGN_RETRACT_MM)
        self.declare_parameter('probe_align_tilt_max_deg', _ALIGN_TILT_MAX_DEG)
        # CONFERÊNCIA do ângulo de ataque: re-sonda o plano DEPOIS de girar o
        # punho e recusa o ensaio se a face não estiver perpendicular ao eixo
        # corrigido. É a única checagem de ÂNGULO de ponta a ponta — todo o
        # resto confere elos isolados (a solução da IK, a pose entregue) e
        # supõe que a primeira medição estava certa.
        #
        # Custa outra rodada de toques na amostra, por isso fica desligada:
        # é a troca entre certificar o alinhamento e não marcar a peça duas
        # vezes. Ligue quando o valor absoluto da força for o resultado —
        # num contato de canto a célula mede a PROJEÇÃO da força normal.
        #   ros2 param set /tactile_explorer probe_align_verify true
        self.declare_parameter('probe_align_verify', False)

        self._phase: str = 'IDLE'
        self._busy = threading.Event()
        # Serializa o checar-e-marcar do _busy em _cb_start. Lock próprio (e
        # não o _params_lock) para não ficar preso atrás do parsing da
        # mensagem, que é longo e roda com o _params_lock na mão.
        self._start_lock = threading.Lock()
        self._params_lock = threading.Lock()
        self._target_depth_mm: float = 5.0
        self._target_force_n:  float = 2.0   # setpoint de força (≤ 10 N)
        # Rigidez de contato estimada durante o DESCENDING e reusada (congelada)
        # no HOLD/SLIDING para o passo deadbeat normalizado pela rigidez.
        self._k_est = _StiffnessEstimator()
        # Curva F(x) do contato corrente, alimentada pelos micro-passos da
        # regulação quase-estática e consumida pela onda como feedforward no
        # lugar do escalar K — ver _ContactCurve.
        self._fx_curve = _ContactCurve()
        # Profundidade (m ao longo do approach) onde o último DESCENDING
        # tocou NA HOME CORRENTE.
        self._learned_contact_m: float | None = None
        # Profundidades dos contatos recentes de uma MESMA geometria de
        # partida, para medir a dispersão real da superfície e dimensionar a
        # margem de incerteza da zona lenta (ver _contact_margin_m). Zerada
        # junto com _learned_contact_m: quando o ponto de partida muda, as
        # profundidades antigas deixam de ser comparáveis.
        self._contact_depths: collections.deque = collections.deque(
            maxlen=_CONTACT_MARGIN_WINDOW)
        # Contato aprendido POR HOME: {chave da home (TCP mm) → profundidade
        # m}.
        self._learned_by_home: dict[tuple, float] = {}
        # Carimbo de tempo (time.time()) de cada entrada — alimenta o
        # vencimento e o log de procedência ("aprendido há N h").
        self._learned_ts: dict[tuple, float] = {}
        # {chave → home_deg que a originou}: a chave é derivada (TCP), então o
        # arquivo tem de guardar os ÂNGULOS para poder ser relido.
        self._learned_home_deg: dict[tuple, list] = {}
        self._home_key_cur: tuple | None = None
        self._home_deg_cur: list | None = None
        # Escrita em disco é ADIADA para o timer de _flush_learned (1 Hz):
        # gravar dentro de _remember_contact poria I/O de arquivo no caminho
        # de controle, logo após a detecção de contato.
        self._learned_dirty = False
        # Serializa a escrita do JSON entre os timers e a thread do protocolo
        # — ver _flush_learned.
        self._flush_lock = threading.Lock()
        self._target_slide_mm: float = 50.0
        self._slide_speed_mms: float = 10.0
        # Inclinação da superfície ao longo da direção do deslize (graus, +
        # = sobe no sentido da marcha).
        self._slide_slope_deg: float = 0.0
        # Normal MEDIDA do plano da amostra (mundo URDF, unitária, apontando
        # para fora da superfície). Preenchida pela calibração do ângulo de
        # ataque; None = o plano do deslize vem da inclinação declarada.
        self._slide_plane_n: np.ndarray | None = None
        # ── MANUAL em DEGRAU ────────────────────────────────────────
        # step_size_n <= 0 desliga a escada: o MANUAL volta a ser o HOLD
        # infinito com setpoint vindo de /palpation/set_force.
        self._step_start_n: float = _STEP_START_DEFAULT_N
        self._step_size_n: float = 0.0
        self._step_max_n: float = 0.0
        self._step_dwell_s: float = _STEP_DWELL_DEFAULT_S
        # Modo do experimento: 'SLIDE' (deslizamento), 'TOUCH' (toque),
        # 'MANUAL' (hold infinito) ou 'MATRIX_MAP' (grade de identações).
        self._mode: str = 'SLIDE'
        # ═══ FORÇA MODULADA (modo TOUCH) ════════════════════════════
        # DUAS fontes, nesta precedência (resolvida em _force_profile):
        #  1. Campos da PalpationStart — é o que a GUI preenche (aba
        #     Palpação, bloco "Modulated Force", visível só em TOUCH).
        #  2. PARÂMETROS ROS abaixo — usados quando a mensagem vem com
        #     force_mod_shape VAZIO (mensagem antiga, ou `ros2 topic pub`
        #     sem os campos). Mantém vivo o fluxo sem GUI:
        #       ros2 param set /tactile_explorer force_mod_shape SINE
        #       ros2 param set /tactile_explorer force_mod_hz 3.0
        #       ros2 param set /tactile_explorer force_mod_min_n 2.0
        #       ros2 param set /tactile_explorer force_mod_max_n 3.0
        #       ros2 param set /tactile_explorer force_mod_cycles 20
        # Os parâmetros são lidos A CADA TOQUE, então dá para mudar a onda
        # entre repetições com o run em andamento.
        self.declare_parameter('force_mod_shape', 'OFF')
        self.declare_parameter('force_mod_hz', 10.0)
        self.declare_parameter('force_mod_min_n', 2.0)
        self.declare_parameter('force_mod_max_n', 3.0)
        self.declare_parameter('force_mod_cycles', _FMOD_CYCLES_DEFAULT)
        # Período REAL do laço ServoJ do mirror_node. O explorer não tem como
        # descobri-lo (o mirror é outro nó), e é ele que define tanto o tick
        # mínimo útil da onda quanto a frequência máxima rastreável: 4,17 Hz
        # com os 30 ms padrão. Suba os DOIS juntos — mirror_node com
        # servoj_period_s:=0.025 e este parâmetro com o mesmo valor — senão a
        # onda é publicada mais rápido do que o braço é comandado.
        self.declare_parameter('servoj_period_s', _CTRL_DT)
        # Perfil vindo da ÚLTIMA PalpationStart. _fmod_from_msg=False mantém
        # o caminho dos parâmetros ROS; _cb_start o liga quando a mensagem
        # traz um shape válido. Protegidos por _params_lock, como os demais.
        self._fmod_from_msg: bool = False
        self._fmod_shape: str = 'OFF'
        self._fmod_hz: float = 0.0
        self._fmod_min_n: float = 0.0
        self._fmod_max_n: float = 0.0
        self._fmod_cycles: int = 0
        # Setpoint INSTANTÂNEO durante a modulação (None = usa o fixo). O
        # status publica este valor, então ele vira a coluna setpoint_n do CSV.
        # É a onda ENTREGUE — reconstruída da penetração medida por FK —, não a
        # comandada: em MovL o comando é aceito antes de executar, e gravar a
        # onda pedida desalinharia setpoint_n de force_net_n no tempo.
        self._force_sp_live: float | None = None
        # ─── MATRIX_MAP ─────────────────────────────────────────────
        # Waypoints do plano em METROS, RELATIVOS à origem descoberta no
        # início do run. Array (N, 2) — só XY; a descida é sempre vertical.
        self._matrix_wps: np.ndarray = np.zeros((0, 2))
        self._matrix_safe_z_m: float = _MATRIX_SAFE_Z_MM_DEFAULT / 1000.0
        self._matrix_transit_ms: float = _MATRIX_TRANSIT_MMS_DEFAULT / 1000.0
        self._matrix_shape: str = ''
        # Origem = TCP no PRIMEIRO contato (mundo URDF, m). É o (0,0,0) do
        # plano; None até ser descoberta. Só o thread do protocolo escreve.
        self._matrix_origin: np.ndarray | None = None
        self._wp_index: int = 0        # 1-based; 0 = fora da matriz
        self._wp_total: int = 0
        self._wp_target: np.ndarray = np.zeros(2)   # alvo planejado (m, rel.)
        self._slide_dir_vec: np.ndarray = np.array([0.0, 1.0])
        self._approach_dir: np.ndarray | None = None
        # ─── Ângulo de ataque calibrado (ver _phase_calibrate_attack) ───
        # Unitário no mundo URDF apontando PARA DENTRO da superfície medida.
        # None = aproximação estritamente vertical, o comportamento de
        # sempre. Zerado no início de cada experimento: a amostra pode ter
        # sido trocada, e um plano medido no run anterior não vale para ela.
        self._attack_dir: np.ndarray | None = None
        # Configuração da calibração vinda da ÚLTIMA PalpationStart.
        # _align_from_msg=False mantém o caminho dos parâmetros ROS (fluxo
        # sem GUI); _cb_start o liga quando a mensagem traz probe_align.
        # Mesma precedência de _fmod_from_msg.
        self._align_from_msg: bool = False
        self._align_on: bool = False
        self._align_msg: dict = {}
        self._user_home_q: np.ndarray | None = None
        self._speed_factor_pct: float = 10.0   # % do slider da GUI (padrão 10 %)
        # Repetições automáticas do experimento (campo 'repeats' da GUI).
        self._repeats: int = 1
        self._cycle: int = 0
        self._cycles_total: int = 1
        # Overrides de estabilização do HOLD vindos do PalpationStart
        # (0.0 no msg = "usar default" → None aqui).
        self._hold_tol_n: float | None = None
        self._hold_stable_s: float | None = None
        self._hold_timeout_s: float | None = None
        self._lc_lock = threading.Lock()
        self._lc_force_net: float = 0.0   # COM SINAL: + compressão, − tração
        self._lc_force_ts: float = 0.0    # time.monotonic() da última leitura
        # Contador de amostras distintas — o debounce de contato precisa contar
        # LEITURAS, não iterações do loop (33 Hz de loop vs 10 Hz do HX711).
        self._lc_force_seq: int = 0
        # Taxa MEDIDA de chegada do Float32 (Hz, EMA). Existe porque o atraso
        # da mediana do lc_filter é meia janela DA TAXA DA FONTE, e a fonte
        # deixou de ser uma só: 24 Hz na HX711, ~400 Hz na FA7155. Quem
        # consome é fmod_measure_lag_s, via _lc_rate_hz(). Medir em vez de
        # ler a constante da célula também cobre o caso real da FA7155, cuja
        # entrega em polled oscila com a carga da máquina (458 Hz ocioso,
        # 405 Hz com a GUI no ar — ver FT_NOMINAL_RATE_HZ).
        self._lc_rate_ema_hz: float = 0.0
        # ── sinal CRU da célula, para a onda ──────────────────────────
        # O Float32 acima é filtrado (mediana + One-Euro travado em 2 Hz) e
        # serve para tudo que é quase-estático, onde o filtro está certo. A
        # ONDA não pode usá-lo: acima de 2 Hz ele come a amplitude que a
        # correção por ciclo está justamente tentando medir (ver
        # fmod_measure_gain). /load_cell/sample_net carrega `voltage_raw`,
        # que é a mesma grandeza SEM filtro, e `t_us`, o relógio do firmware.
        self._lc_raw_net: float | None = None
        self._lc_raw_ts: float = 0.0
        self._lc_raw_seq: int = 0
        # Taxa MEDIDA de chegada do sinal CRU (Hz, EMA) — separada da do
        # Float32 porque as duas podem ter fontes diferentes no fio, e é esta
        # que decide se a onda é MEDÍVEL (ver _FMOD_MIN_MEAS_RATE_MULT).
        self._lc_raw_rate_ema_hz: float = 0.0
        # Histórico curto (t_monotônico, força crua) para a onda.
        #
        # POR QUE UM BUFFER, e não o último valor. O laço da onda roda no tick
        # do COMANDO — 5 pontos por período, 50 Hz a 10 Hz. Ler a célula ali
        # amostra 400 Hz de sinal a 50 Hz, e o que a interpolação linear do
        # comando gera de harmônico cai justamente em 4f e 6f: a 5 amostras
        # por período os dois DOBRAM sobre a própria fundamental. O lock-in
        # passaria a medir a amplitude contaminada pela distorção que ele
        # deveria denunciar, e o ILC fecharia contra esse número.
        #
        # Com o histórico, cada amostra entra no lock-in com o SEU instante e
        # a medida roda na taxa da célula (~400 Hz), independente do tick do
        # comando. 2 s cobrem o pior caso útil (ver _FORCE_STALE_S de folga) a
        # 400 Hz sem crescer sem limite.
        self._lc_raw_hist: collections.deque = collections.deque(maxlen=1024)
        # ── canal LENTO da célula (média móvel larga) ─────────────────
        # Mesma força e MESMO tare do Float32 rápido, com σ ~19 mN em vez de
        # ~33 mN (ver o bloco CANAL LENTO em lc_filter.py). Usado SÓ na
        # medida assentada de _qs_measure_fz: perto do alvo o que falta é
        # resolução, e ali o braço já está congelado. Longe do alvo, e em
        # tudo que é segurança/contato/SLIDING, continua valendo o rápido.
        # `None` enquanto ninguém publicar — é o caso do ft_receiver, e o
        # consumidor CAI no canal rápido em vez de parar.
        self._lc_slow_net: float | None = None
        self._lc_slow_ts: float = 0.0
        self._lc_slow_win_s: float | None = None
        # N por unidade de `voltage_raw`. 1,0 na FA7155 (já entrega newtons);
        # no HX711 é o N/V da calibração da ponte. Ver _cb_lc_sample_net.
        self._lc_raw_scale: float = float(
            self.declare_parameter('lc_raw_scale_n_per_unit', 1.0).value or 1.0)
        # Acumuladores da regressão que CONFERE a escala acima.
        self._lc_scale_sxy = 0.0
        self._lc_scale_sxx = 0.0
        self._lc_scale_n = 0
        self._q_lock = threading.Lock()
        self._current_q = _POINTING_SEED_Q.copy()
        # Chegada da última /joint_states. Quem mede velocidade de junta mede
        # entre LEITURAS NOVAS, não entre ticks do laço — ver _q_sample.
        self._q_ts = 0.0
        self._stop_requested = threading.Event()
        # FREEZE (parada dura): congela no lugar SEM ir à HOME — distinto do
        # STOP normal, que recua à home (Regra de Ouro). Usado pelo E-STOP.
        self._freeze_requested = threading.Event()
        self._pause_requested = threading.Event()
        self._protocol_thread: threading.Thread | None = None

        cb = ReentrantCallbackGroup()

        self.create_subscription(PalpationStart, '/palpation/start',
                                  self._cb_start, _QOS_COMMAND, callback_group=cb)
        self.create_subscription(String, '/palpation/stop',
                                  self._cb_stop, 10, callback_group=cb)
        self.create_subscription(Bool, '/palpation/pause',
                                  self._cb_pause, 10, callback_group=cb)
        self.create_subscription(Empty, '/palpation/freeze',
                                  self._cb_freeze, 10, callback_group=cb)
        self.create_subscription(Float32, '/palpation/set_force',
                                  self._cb_set_force, 10, callback_group=cb)
        self.create_subscription(Empty, '/palpation/forget_contact',
                                  self._cb_forget_contact, 10, callback_group=cb)
        self.create_subscription(Float32, '/load_cell/force_net',
                                  self._cb_lc_force_net, _QOS_SENSOR, callback_group=cb)
        self.create_subscription(LoadCellSample, '/load_cell/sample_net',
                                  self._cb_lc_sample_net, _QOS_SENSOR,
                                  callback_group=cb)
        self.create_subscription(Float32, '/load_cell/force_net_slow',
                                  self._cb_lc_force_net_slow, _QOS_SENSOR,
                                  callback_group=cb)
        self.create_subscription(Float32, '/load_cell/slow_win_s',
                                  self._cb_lc_slow_win, _QOS_COMMAND,
                                  callback_group=cb)
        self.create_subscription(JointState, '/joint_states',
                                  self._cb_joints, 50, callback_group=cb)

        self._status_pub = self.create_publisher(
            PalpationStatus, '/palpation/status', 10)
        # Um registro por identação concluída do MATRIX_MAP. RELIABLE com fila
        # funda: o logger não pode perder pontos — é o que casa cada trecho da
        # curva de força com a coordenada onde ela foi medida.
        self._matrix_pub = self.create_publisher(
            MatrixPoint, '/palpation/matrix_point', 50)


        # Publisher direto no tópico do controller — sem action server.
        # depth=1: sem fila; cada nova mensagem substitui a anterior para
        # evitar rajada de setpoints antigos após jitter do SO.
        self._arm_traj_pub = self.create_publisher(
            JointTrajectory,
            '/cr10_group_controller/joint_trajectory', 1)
        self._hand_pub = self.create_publisher(
            JointTrajectory,
            '/hand_position_controller/joint_trajectory', 5)

        self._load_learned()
        self.get_logger().info('tactile_explorer pronto — streaming 33 Hz')
        self.create_timer(0.10, self._publish_status, callback_group=cb)
        # Persistência do aprendizado em timer SEPARADO do status: o status é
        # republicado por _set_phase na thread do protocolo, e a I/O não pode
        # pegar carona nesse caminho.
        self.create_timer(1.0, self._flush_learned, callback_group=cb)

    # Callbacks
    _LC_MAX_PLAUSIBLE_N = 100.0

    def _cb_lc_force_net(self, msg: Float32) -> None:
        """Recebe /load_cell/force_net — COM SINAL (+ compressão, − tração)."""
        val = float(msg.data)
        if not math.isfinite(val) or abs(val) > self._LC_MAX_PLAUSIBLE_N:
            return
        now = time.monotonic()
        with self._lc_lock:
            # Intervalo entre LEITURAS. A janela de validade descarta o
            # primeiro quadro (ts = 0) e as pausas do nó, que virariam uma
            # taxa absurdamente baixa e contaminariam a EMA por muitos ciclos.
            dt = now - self._lc_force_ts
            if self._lc_force_ts > 0.0 and 1e-4 < dt < 1.0:
                r = 1.0 / dt
                self._lc_rate_ema_hz = (
                    r if self._lc_rate_ema_hz <= 0.0
                    else 0.98 * self._lc_rate_ema_hz + 0.02 * r)
            self._lc_force_net = val
            self._lc_force_ts = now
            self._lc_force_seq += 1

    def _lc_rate_hz(self) -> float:
        """Taxa (Hz) da fonte que alimenta /load_cell/force_net.

        MEDIDA, não deduzida da célula configurada: é ela que fixa o atraso da
        mediana em fmod_measure_lag_s, e um nó com a fonte errada no fio
        giraria a correção do ILC em fase sem avisar. Enquanto nada chegou —
        onda nenhuma roda nesse estado, o _FORCE_STALE_S barra antes — vale o
        nominal da HX711, que é o pior caso (maior atraso).
        """
        with self._lc_lock:
            r = self._lc_rate_ema_hz
        return r if r > 0.0 else _LC_NOMINAL_RATE_HZ

    def _cb_lc_force_net_slow(self, msg: Float32) -> None:
        """Recebe /load_cell/force_net_slow — mesma convenção de sinal e o
        MESMO tare do canal rápido, só com a média móvel larga."""
        val = float(msg.data)
        if not math.isfinite(val) or abs(val) > self._LC_MAX_PLAUSIBLE_N:
            return
        with self._lc_lock:
            self._lc_slow_net = val
            self._lc_slow_ts = time.monotonic()

    def _cb_lc_slow_win(self, msg: Float32) -> None:
        """Janela do canal lento, anunciada pelo receiver (latched).

        Vem do fio e não de um parâmetro daqui de propósito: duplicar o
        número nos dois nós faria uma divergência silenciosa congelar o braço
        por menos tempo que a média precisa para se limpar das amostras de
        ANTES do micro-passo — e a leitura sairia enviesada pela posição
        anterior, que é exatamente o erro que este canal existe para não
        cometer."""
        win = float(msg.data)
        if not math.isfinite(win) or not 0.0 < win <= 30.0:
            return
        with self._lc_lock:
            self._lc_slow_win_s = win

    def _cb_lc_sample_net(self, msg: LoadCellSample) -> None:
        """Recebe /load_cell/sample_net e extrai a força CRUA, tare-compensada.

        A mensagem traz três grandezas do MESMO instante:

            voltage_raw   sem filtro, SEM tare
            voltage       filtrada,   SEM tare
            force_net_n   filtrada,   COM tare  (N)

        O tare não vem no fio, mas sai da diferença: `voltage - force_net_n` é
        o tare expresso nas unidades do sensor. Somar a esse zero a distância
        entre cru e filtrado devolve o cru tarado:

            raw_net = force_net_n + (voltage_raw - voltage) * escala

        A ESCALA existe porque os dois receivers discordam da unidade. A
        FA7155 entrega NEWTONS em voltage_raw (`ft_receiver` comenta "N, não
        V" nos dois campos), então a escala é 1,0. O HX711 entrega VOLTS da
        ponte, e ali a escala é o N/V da calibração. Errar isso não degrada:
        inverte o sinal da correção. Por isso a escala é parâmetro explícito e
        o log de cada onda imprime a estimada por regressão ao lado dela.
        """
        if not msg.calibrated:
            return
        raw = float(msg.voltage_raw)
        filt = float(msg.voltage)
        net = float(msg.force_net_n)
        if not (math.isfinite(raw) and math.isfinite(filt)
                and math.isfinite(net)):
            return
        val = net + (raw - filt) * self._lc_raw_scale
        if abs(val) > self._LC_MAX_PLAUSIBLE_N:
            return
        now = time.monotonic()
        with self._lc_lock:
            dt = now - self._lc_raw_ts
            if self._lc_raw_ts > 0.0 and 1e-5 < dt < 1.0:
                r = 1.0 / dt
                self._lc_raw_rate_ema_hz = (
                    r if self._lc_raw_rate_ema_hz <= 0.0
                    else 0.98 * self._lc_raw_rate_ema_hz + 0.02 * r)
            self._lc_raw_net = val
            self._lc_raw_ts = now
            self._lc_raw_seq += 1
            self._lc_raw_hist.append((now, val))
            # Pares (filtrado_sensor, filtrado_N) do MESMO filtro: a razão
            # entre as duas excursões é a escala N/unidade, e o filtro se
            # cancela por estar nos dois lados. É a conferência da escala
            # acima, acumulada aqui e resolvida no fim da onda.
            self._lc_scale_sxy += filt * net
            self._lc_scale_sxx += filt * filt
            self._lc_scale_n += 1

    def _fz_raw(self) -> float | None:
        """Força CRUA tare-compensada (N), ou None se ninguém publica
        /load_cell/sample_net. É o sinal que a onda usa para se corrigir; a
        segurança e as fases quase-estáticas seguem no filtrado."""
        with self._lc_lock:
            if (self._lc_raw_net is None
                    or time.monotonic() - self._lc_raw_ts > _FORCE_STALE_S):
                return None
            return self._lc_raw_net

    def _lc_raw_rate_hz(self) -> float:
        """Taxa (Hz) de chegada do sinal CRU, ou 0,0 se ninguém o publica.

        Zero é informação, não ausência de informação: é o que distingue
        "célula lenta" de "não há canal cru nenhum no fio", e os dois exigem
        respostas diferentes da onda."""
        with self._lc_lock:
            return self._lc_raw_rate_ema_hz if self._lc_raw_net is not None \
                else 0.0

    def _lc_raw_since(self, t_mono: float) -> list[tuple[float, float]]:
        """Amostras cruas (t_monotônico, N) chegadas DEPOIS de `t_mono`.

        O consumidor é o lock-in da onda, que precisa de cada amostra com o
        SEU carimbo — ver o comentário de `_lc_raw_hist`. Devolve lista (não
        iterador) de propósito: o lock sai de cena antes de o chamador fazer
        trigonometria em cima."""
        with self._lc_lock:
            return [(t, v) for t, v in self._lc_raw_hist if t > t_mono]

    def _cb_joints(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        with self._q_lock:
            for i, j in enumerate(_ARM_JOINTS):
                if j in idx:
                    self._current_q[i] = float(msg.position[idx[j]])
            self._q_ts = time.monotonic()

    def _cb_set_force(self, msg: Float32) -> None:
        """Atualiza o setpoint de força ON-THE-FLY (modo MANUAL/dinâmico)."""
        with self._params_lock:
            self._target_force_n = float(np.clip(
                float(msg.data), _CONTACT_ON_N, _FORCE_SETPOINT_MAX_N))
        self.get_logger().info(
            f'[MANUAL] novo setpoint de força = {self._target_force_n:.2f} N')

    def _cb_stop(self, msg: String) -> None:
        if self._busy.is_set():
            self._stop_requested.set()
            self._pause_requested.clear()   # stop vence pausa
            self.get_logger().warn('[STOP] Parada solicitada.')

    def _cb_freeze(self, msg: Empty) -> None:
        """FREEZE (parada dura): congela a posição atual IMEDIATAMENTE e aborta
        o protocolo SEM ir à HOME — ao contrário do STOP, que recua à home
        (Regra de Ouro). Evita arrastar a ferramenta sobre a superfície quando
        a pose está comprometida. Usado pelo E-STOP. No robô real, o
        StopRobot/DisableRobot do hardware é feito em paralelo pela GUI."""
        if self._busy.is_set():
            self._freeze_requested.set()
            self._stop_requested.set()      # quebra os loops das fases
            self._pause_requested.clear()
            self.get_logger().warn(
                '[FREEZE] congelando no lugar (sem retorno à HOME).')

    def _cb_forget_contact(self, msg: Empty) -> None:
        """GUI desmarcou "Home conhecida": descarta o contato aprendido da
        home corrente (e das vizinhas dentro da tolerância, no disco também).
        A próxima descida rasteja do início e re-aprende. Pensado para ser
        usado ENTRE runs; chamado durante um run só afeta a descida seguinte.
        """
        self._forget_contact('operador desmarcou "Home conhecida" na GUI')

    def _cb_pause(self, msg: Bool) -> None:
        """Pausa/retoma o experimento — as fases seguram a posição atual
        enquanto pausadas (ver _pause_gate)."""
        if bool(msg.data):
            if self._busy.is_set():
                self._pause_requested.set()
        else:
            self._pause_requested.clear()

    # ── Contato aprendido POR HOME ────────────────────────────────────
    # Home nova ⇒ a descida inteira rasteja (fine_speed_mms /
    # _DESCEND_CONTACT_V_MAX_MS) e aprende onde está a superfície; home já
    # vista ⇒ estágio RÁPIDO no ritmo do slider até PARAR na folga fina
    # antes do contato daquela home. Ver o bloco
    # de _CONTACT_ZONE_MARGIN_M (streaming).

    @staticmethod
    def _tcp_key(p) -> tuple:
        """Chave de aprendizado a partir de uma posição de TCP (m).

        A indexação do contato aprendido é POSICIONAL, não "por home": o que
        identifica o ponto de partida é onde o TCP está, e qualquer pose serve
        de chave — inclusive a pose de JOG de onde o MATRIX_MAP parte.
        """
        return tuple(round(float(v) * 1e3, 1) for v in p)

    @staticmethod
    def _home_key(home_deg) -> tuple | None:
        """Chave da home: a POSIÇÃO DO TCP (mm, 0,1 mm) via FK, não os ângulos."""
        try:
            q = np.array([math.radians(float(v)) for v in home_deg],
                         dtype=np.float64)
            p = forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]
        except Exception:
            return None
        return TactileExplorer._tcp_key(p)

    def _lookup_learned(self, key: tuple | None,
                        approach_dir=None) -> tuple[float | None, float]:
        """Profundidade aprendida para `key`, aceitando home VIZINHA."""
        if key is None:
            return None, 0.0
        exact = self._learned_by_home.get(key)
        if exact is not None:
            return exact, self._learned_ts.get(key, 0.0)
        d = None if approach_dir is None else np.asarray(approach_dir, float)
        p = np.array(key, dtype=float)          # mm
        best_k, best_lat = None, _LEARNED_TCP_TOL_M * 1e3
        for k in self._learned_by_home:
            delta = p - np.array(k, dtype=float)          # mm
            lat = (float(np.linalg.norm(delta)) if d is None else
                   float(np.linalg.norm(delta - np.dot(delta, d) * d)))
            if lat < best_lat:
                best_k, best_lat = k, lat
        if best_k is None:
            return None, 0.0
        depth_m = self._learned_by_home[best_k]
        if d is None:
            depth_m -= best_lat * 1e-3            # tudo tratado como axial
        else:
            delta = p - np.array(best_k, dtype=float)
            depth_m -= float(np.dot(delta, d)) * 1e-3   # correção exata
            depth_m -= _LEARNED_FLATNESS_M
        if depth_m <= 0.0:
            return None, 0.0
        self.get_logger().info(
            f'[APRENDIZADO] home vizinha reaproveitada (offset lateral '
            f'{best_lat:.2f} mm): contato aprendido em '
            f'{self._learned_by_home[best_k]*1e3:.1f} mm, usado '
            f'{depth_m*1e3:.1f} mm.')
        return depth_m, self._learned_ts.get(best_k, 0.0)

    def _contact_margin_m(self) -> float:
        """Margem de INCERTEZA da zona lenta — quanto o contato real pode
        estar longe do estimado, em metros.

        Sem evidência suficiente devolve `_CONTACT_ZONE_MARGIN_M`, o palpite
        conservador de sempre. Com `_CONTACT_MARGIN_MIN_PTS` contatos da mesma
        geometria de partida, devolve o que a peça MEDIU: o maior desvio em
        relação à mediana, multiplicado por `_CONTACT_MARGIN_K`.

        Usa desvio-vs-mediana (e não desvio-padrão) de propósito: o que importa
        é o pior ponto, não o típico — um degrau isolado na amostra tem de
        alargar a margem sozinho, e a mediana o mantém fora da referência.
        Nunca aumenta a margem além do palpite inicial: se a dispersão for
        grande, quem manda continua sendo o conservador.
        """
        n = len(self._contact_depths)
        if n < _CONTACT_MARGIN_MIN_PTS:
            return _CONTACT_ZONE_MARGIN_M
        d = sorted(self._contact_depths)
        med = d[n // 2] if n % 2 else 0.5 * (d[n // 2 - 1] + d[n // 2])
        spread = max(abs(x - med) for x in d)
        return float(min(_CONTACT_ZONE_MARGIN_M,
                         max(_CONTACT_MARGIN_FLOOR_M,
                             _CONTACT_MARGIN_K * spread)))

    def _remember_contact(self, depth_m: float) -> None:
        """Memoriza a profundidade de contato DA HOME CORRENTE."""
        self._learned_contact_m = depth_m
        self._contact_depths.append(float(depth_m))
        if self._home_key_cur is not None:
            self._learned_by_home[self._home_key_cur] = depth_m
            self._learned_ts[self._home_key_cur] = time.time()
            if self._home_deg_cur is not None:
                self._learned_home_deg[self._home_key_cur] = self._home_deg_cur
            self._learned_dirty = True

    # ── Persistência (~/.config/touch_pack/learned_contact.json) ──────
    # Poupa o rastejo da 1ª descida de cada home a cada reinício do nó.

    def _persist_enabled(self) -> bool:
        try:
            return bool(self.get_parameter('learned_contact_persist').value)
        except Exception:
            return False

    def _load_learned(self) -> None:
        """Lê o arquivo na subida. Ausente/corrompido = começa vazio (a
        consequência é rastejar, nunca bater), então nada aqui é fatal."""
        if not self._persist_enabled():
            self.get_logger().info(
                '[APRENDIZADO] persistência desligada '
                '(learned_contact_persist:=false) — só memória.')
            return
        try:
            with open(_LEARNED_CONTACT_FILE) as fh:
                data = json.load(fh)
            entries = data.get('entries', []) if isinstance(data, dict) else []
        except FileNotFoundError:
            self.get_logger().info(
                f'[APRENDIZADO] sem histórico em {_LEARNED_CONTACT_FILE} — '
                'a 1ª descida de cada home rasteja e aprende.')
            return
        except (OSError, ValueError) as exc:
            self.get_logger().warning(
                f'[APRENDIZADO] {_LEARNED_CONTACT_FILE} ilegível ({exc}) — '
                'começando vazio; toda home vai rastejar.')
            return

        aviso = _tool_stamp_mismatch(data, what='o contato aprendido')
        if aviso:
            self.get_logger().warning(
                f'[FERRAMENTA] {aviso} As entradas NÃO são descartadas: '
                '_lookup_learned corrige o deslocamento ao longo da '
                'aproximação, e para uma ferramenta mais curta a correção '
                'aprofunda a descida na medida certa. Mas a correção supõe '
                'que a AMOSTRA não se moveu — se ela também mudou, apague '
                'o arquivo.')

        max_age_h = float(self.get_parameter('learned_contact_max_age_h').value)
        now = time.time()
        loaded = expired = 0
        for e in entries:
            try:
                home_deg = list(e['home_deg'])
                key = self._home_key(home_deg)
                depth_m = float(e['contact_mm']) / 1e3
                ts = float(e.get('t_unix', 0.0))
            except (KeyError, TypeError, ValueError):
                continue          # registro torto: ignora, não derruba o resto
            if key is None or depth_m <= 0.0:
                continue
            if max_age_h > 0.0 and (now - ts) > max_age_h * 3600.0:
                expired += 1
                continue
            # Chave derivada do TCP: entradas de arquivos ANTIGOS (chaveados
            # por ângulo) podem colapsar na mesma chave. Fica a mais recente.
            if ts < self._learned_ts.get(key, 0.0):
                continue
            self._learned_by_home[key] = depth_m
            self._learned_ts[key] = ts
            self._learned_home_deg[key] = home_deg
            loaded += 1
        if expired:
            self._learned_dirty = True    # poda os vencidos no próximo flush
        self.get_logger().info(
            f'[APRENDIZADO] {loaded} home(s) com contato aprendido carregadas '
            f'de {_LEARNED_CONTACT_FILE}'
            + (f'; {expired} vencida(s) (> {max_age_h:.0f} h) descartada(s) — '
               'essas homes voltam a rastejar' if expired else '')
            + '. ATENÇÃO: a persistência foi LIGADA explicitamente '
              '(learned_contact_persist:=true), e ela assume que a peça e a '
              'fixação NÃO mudaram desde a última sessão — hipótese que este '
              'nó não tem como verificar. Se algo foi remontado, desligue o '
              'parâmetro (é o default) ou apague o arquivo.')

    def _flush_learned(self) -> None:
        """Grava o dicionário se houver mudança pendente. Tem TIMER PRÓPRIO
        (1 Hz), e não carona no de status: o status também é publicado por
        `_set_phase`, na thread do protocolo, e isso punha I/O de arquivo no
        caminho de transição de fase — exatamente o que `_remember_contact`
        adia de propósito.

        Escrita atômica: um crash no meio não deixa um JSON meio escrito no
        lugar do bom. O `_flush_lock` é o que torna essa atomicidade real —
        `_learned_dirty` sozinho não serializa nada, porque é limpo dentro do
        `_params_lock` e SOLTO antes da escrita: dois flushes concorrentes
        escreviam o MESMO arquivo .tmp e o os.replace movia um arquivo
        parcial.
        """
        with self._params_lock:
            if not self._learned_dirty:
                return
            self._learned_dirty = False
            # A chave é DERIVADA (TCP via FK) e não dá para invertê-la: o
            # que vai para o disco são os ângulos que a geraram, e o
            # _load_learned recalcula a chave a partir deles.
            snapshot = [
                {'home_deg': list(self._learned_home_deg[k]),
                 'tcp_mm': list(k),
                 'contact_mm': round(v * 1e3, 3),
                 't_unix': round(self._learned_ts.get(k, 0.0), 3),
                 't_iso': datetime.fromtimestamp(
                     self._learned_ts.get(k, 0.0)).isoformat(timespec='seconds')}
                for k, v in self._learned_by_home.items()
                if k in self._learned_home_deg
            ]
        if not self._persist_enabled():
            return
        payload = {'version': 1, **_tool_stamp(), 'entries': sorted(
            snapshot, key=lambda e: e['home_deg'])}
        tmp = f'{_LEARNED_CONTACT_FILE}.tmp'
        with self._flush_lock:
            try:
                os.makedirs(os.path.dirname(_LEARNED_CONTACT_FILE),
                            exist_ok=True)
                with open(tmp, 'w') as fh:
                    json.dump(payload, fh, indent=2, sort_keys=True)
                os.replace(tmp, _LEARNED_CONTACT_FILE)
            except OSError as exc:
                self.get_logger().warning(
                    f'[APRENDIZADO] falha ao gravar {_LEARNED_CONTACT_FILE}: '
                    f'{exc} — o aprendizado desta sessão fica só em memória.')

    def _forget_contact(self, motivo: str) -> None:
        """Descarta o contato aprendido da home corrente — a próxima descida
        volta a rastejar e re-aprende.
        """
        with self._params_lock:
            had = self._learned_contact_m is not None
            self._learned_contact_m = None
            self._contact_depths.clear()
            if self._home_key_cur is not None:
                # Purga a chave corrente E as VIZINHAS dentro da tolerância: se
                # a descida reaproveitou o valor de uma home vizinha e mesmo
                # assim bateu, a entrada culpada é a vizinha — apagar só a
                # chave exata a deixaria viva para repetir a batida.
                tol_mm = _LEARNED_TCP_TOL_M * 1e3
                doomed = [k for k in self._learned_by_home
                          if math.dist(k, self._home_key_cur) <= tol_mm]
                for k in doomed:
                    self._learned_by_home.pop(k, None)
                    self._learned_ts.pop(k, None)
                    self._learned_home_deg.pop(k, None)
                # Some do DISCO também: um valor que causou batida não pode
                # ressuscitar no próximo boot do nó.
                self._learned_dirty = True
        if had:
            self.get_logger().warn(
                f'[APRENDIZADO] contato desta home descartado ({motivo}) — '
                'a próxima descida rasteja a partir do início e re-aprende.')

    def _parse_matrix_waypoints(self, msg: PalpationStart) -> np.ndarray | None:
        """Valida e converte `msg.waypoints` (geometry_msgs/Point[], metros,
        RELATIVOS à origem) num array (N, 2) de XY.
        """
        pts = list(getattr(msg, 'waypoints', []) or [])
        if not pts:
            self.get_logger().error(
                '[MATRIX] modo MATRIX_MAP sem waypoints — run RECUSADO. '
                'Gere a grade no configurador da aba de Palpação antes do Start.')
            return None
        if len(pts) > _MATRIX_MAX_POINTS:
            self.get_logger().error(
                f'[MATRIX] {len(pts)} waypoints excede o teto de '
                f'{_MATRIX_MAX_POINTS} — run RECUSADO. Reduza as linhas/colunas.')
            return None
        try:
            xy = np.array([[float(p.x), float(p.y)] for p in pts],
                          dtype=np.float64)
        except (AttributeError, TypeError, ValueError) as exc:
            self.get_logger().error(
                f'[MATRIX] waypoints malformados ({exc}) — run RECUSADO.')
            return None
        if not np.all(np.isfinite(xy)):
            self.get_logger().error(
                '[MATRIX] waypoints com NaN/Inf — run RECUSADO.')
            return None
        span_max_m = _MATRIX_SPAN_MAX_MM / 1000.0
        if float(np.max(np.abs(xy))) > span_max_m:
            worst = float(np.max(np.abs(xy))) * 1e3
            self.get_logger().error(
                f'[MATRIX] waypoint a {worst:.1f} mm da origem excede o '
                f'envelope de ±{_MATRIX_SPAN_MAX_MM:.0f} mm — run RECUSADO.')
            return None
        return xy

    def _cb_start(self, msg: PalpationStart):
        """Porta de entrada do /palpation/start: recusa comando repetido e
        comando VELHO, e só então delega o parsing.

        Checar-e-marcar precisa ser ATÔMICO: os callbacks rodam num
        MultiThreadedExecutor(3) com ReentrantCallbackGroup, então dois starts
        quase simultâneos passariam os dois por um `if _busy.is_set()` solto e
        subiriam DUAS FSMs transmitindo no mesmo controlador.
        """
        if self._start_is_stale(msg):
            return
        with self._start_lock:
            if self._busy.is_set():
                self.get_logger().warn(
                    f'Recebido /palpation/start mas explorer está em '
                    f'{self._phase}. Ignorando.')
                return
            # Os três Events são limpos AQUI, antes de marcar o _busy, e não
            # no fim do parsing. `_cb_stop`, `_cb_freeze` e `_cb_pause` só
            # marcam com o _busy setado, então este é o único ponto em que
            # limpar é atômico: nada pode marcá-los antes do _busy.set()
            # logo abaixo, e tudo o que marcar DEPOIS sobrevive.
            #
            # Limpar no fim do _start_from_msg abria uma janela em que um
            # FREEZE (E-STOP) chegado durante o parsing — set_parameters,
            # validação da matriz, lookup do aprendizado — era marcado e
            # depois APAGADO, e o run arrancava como se ninguém tivesse
            # apertado nada. É a única mensagem do sistema que não pode ser
            # perdida por causa de uma janela de milissegundos.
            self._pause_requested.clear()
            self._stop_requested.clear()
            self._freeze_requested.clear()
            self._busy.set()
        started = False
        try:
            started = self._start_from_msg(msg)
        finally:
            # Só a thread do protocolo limpa o _busy quando ela existe; se o
            # parsing recusou o run (ou explodiu), libera aqui — senão o nó
            # ficaria ocupado para sempre sem nada rodando.
            if not started:
                self._busy.clear()

    def _start_is_stale(self, msg: PalpationStart) -> bool:
        """True para um start LATCHED de outra sessão.

        /palpation/start é TRANSIENT_LOCAL de propósito — é o que faz o
        palpation_logger, que sobe depois do publish, ainda receber o começo do
        run. Mas o explorer também é assinante: reiniciando com a GUI viva ele
        recebia na hora o último comando e saía movendo o braço a partir de uma
        pose que já não é a de quando o comando foi emitido.

        `stamp` zerado = publisher antigo ou `ros2 topic pub` sem o campo:
        aceita, como os demais campos novos. Stamp no futuro (relógios
        diferentes) também passa — só o passado distante é recusado.
        """
        stamp = getattr(msg, 'stamp', None)
        if stamp is None:
            return False
        t_msg = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if t_msg <= 0.0:
            return False
        age_s = self.get_clock().now().nanoseconds * 1e-9 - t_msg
        if age_s <= _START_MAX_AGE_S:
            return False
        self.get_logger().warn(
            f'/palpation/start ignorado: comando de {age_s:.0f} s atrás '
            f'(teto {_START_MAX_AGE_S:.0f} s). É o latch TRANSIENT_LOCAL de '
            'uma sessão anterior, não um pedido novo — publique de novo pela '
            'GUI para rodar.')
        return True

    def _start_from_msg(self, msg: PalpationStart) -> bool:
        """Aplica os parâmetros e arranca a thread do protocolo.

        Devolve True se a thread arrancou — é o que diz ao _cb_start se o
        _busy fica marcado ou deve ser liberado.
        """
        with self._params_lock:
            self._target_depth_mm = float(msg.depth_mm)
            # Setpoint de força — saturado no máximo selecionável.
            self._target_force_n = float(np.clip(
                float(msg.force_n), _CONTACT_ON_N, _FORCE_SETPOINT_MAX_N))
            self._target_slide_mm = float(msg.slide_dist_mm)
            self._slide_speed_mms = float(msg.speed_mms)
            # Campo novo (06/08/2026): mensagens antigas não o têm, e ausente
            # significa 0° — exatamente o comportamento anterior.
            self._slide_slope_deg = float(np.clip(
                float(getattr(msg, 'slide_slope_deg', 0.0) or 0.0),
                -_SLIDE_SLOPE_MAX_DEG, _SLIDE_SLOPE_MAX_DEG))
            mode = str(msg.mode).upper().strip()
            if mode not in ('SLIDE', 'TOUCH', 'MANUAL', 'MATRIX_MAP'):
                # Todo campo vizinho que sofre fallback avisa; este não
                # avisava, e um typo num `ros2 topic pub` rodava um DESLIZE
                # que ninguém pediu — com a lateral inteira sobre a amostra.
                self.get_logger().warn(
                    f'mode inválido "{mode or "(vazio)"}" — usando SLIDE. '
                    'Os modos válidos são SLIDE, TOUCH, MANUAL e MATRIX_MAP; '
                    'se você queria um deles, PARE agora: este run vai '
                    'deslizar sobre a amostra.')
                mode = 'SLIDE'
            self._mode = mode
            # ── MANUAL em DEGRAU ────────────────────────────────────
            # Campos ausentes (mensagem antiga) ⇒ 0 ⇒ escada desligada e
            # MANUAL segue sendo o HOLD infinito de antes.
            self._step_start_n = float(np.clip(
                float(getattr(msg, 'step_start_n', 0.0) or
                      _STEP_START_DEFAULT_N),
                _CONTACT_ON_N, _FORCE_SETPOINT_MAX_N))
            self._step_size_n = max(0.0, float(
                getattr(msg, 'step_size_n', 0.0) or 0.0))
            self._step_max_n = float(np.clip(
                float(getattr(msg, 'step_max_n', 0.0) or 0.0),
                0.0, _FORCE_SETPOINT_MAX_N))
            self._step_dwell_s = float(np.clip(
                float(getattr(msg, 'step_dwell_s', 0.0) or
                      _STEP_DWELL_DEFAULT_S),
                0.0, 600.0))
            # ═══ FORÇA MODULADA — perfil vindo da mensagem ══════════
            # getattr com default mantém compatível com mensagem antiga.
            # Shape vazio/desconhecido => _fmod_from_msg=False, e
            # _force_profile() cai nos parâmetros ROS (fluxo sem GUI).
            _shape = str(getattr(msg, 'force_mod_shape', '') or '')
            _shape = _shape.upper().strip()
            self._fmod_from_msg = _shape in _FMOD_SHAPES
            if self._fmod_from_msg:
                self._fmod_shape  = _shape
                self._fmod_hz     = float(getattr(msg, 'force_mod_hz', 0.0))
                self._fmod_min_n  = float(getattr(msg, 'force_mod_min_n', 0.0))
                self._fmod_max_n  = float(getattr(msg, 'force_mod_max_n', 0.0))
                self._fmod_cycles = int(getattr(msg, 'force_mod_cycles', 0))
            # ═══ CALIBRAÇÃO DO ÂNGULO DE ATAQUE ════════════════════
            # Mesma precedência da modulação: campo vazio (mensagem antiga
            # ou `ros2 topic pub` sem ele) mantém o caminho dos parâmetros
            # ROS probe_align_*. Os numéricos em 0 = default do explorer,
            # resolvido em _align_params.
            _align = str(getattr(msg, 'probe_align', '') or '').upper().strip()
            self._align_from_msg = _align in ('ON', 'OFF')
            self._align_on = (_align == 'ON')
            if self._align_from_msg:
                self._align_msg = {
                    'points': int(getattr(msg, 'probe_align_points', 0) or 0),
                    'radius_mm': float(
                        getattr(msg, 'probe_align_radius_mm', 0.0) or 0.0),
                    'force_n': float(
                        getattr(msg, 'probe_align_force_n', 0.0) or 0.0),
                    'retract_mm': float(
                        getattr(msg, 'probe_align_retract_mm', 0.0) or 0.0),
                    'tilt_max_deg': float(
                        getattr(msg, 'probe_align_tilt_max_deg', 0.0) or 0.0),
                }
            if msg.approach_speed_mms > 0.0:
                # Slider "Descent Speed" da GUI (mm/s) = velocidade do estágio
                # RÁPIDO da descida. O rastejo não sai daqui: ele é derivado do
                # limiar de contato e da rigidez de referência (crawl_v_ms) —
                # e NÃO do setpoint, para o primeiro impacto ser o mesmo em
                # qualquer ensaio.
                v_max = max(1.0, float(msg.approach_speed_mms))
                v_min = max(0.5, v_max * 0.2)
                self.set_parameters([
                    rclpy.parameter.Parameter(
                        'approach_v_max_mms',
                        rclpy.parameter.Parameter.Type.DOUBLE, v_max),
                    rclpy.parameter.Parameter(
                        'approach_v_min_mms',
                        rclpy.parameter.Parameter.Type.DOUBLE, v_min),
                ])
            if msg.speed_factor_pct > 0.0:
                self._speed_factor_pct = float(
                    max(1.0, min(100.0, float(msg.speed_factor_pct))))
            self._repeats = int(np.clip(int(msg.repeats) or 1, 1, 100))
            slide_dir = str(msg.slide_dir).upper().strip() or '+Y'
            _DIR_MAP = {
                '+X': (1.0, 0.0), '-X': (-1.0, 0.0),
                '+Y': (0.0, 1.0), '-Y': (0.0, -1.0),
            }
            if slide_dir in _DIR_MAP:
                self._slide_dir_vec = np.array(_DIR_MAP[slide_dir])
            else:
                self.get_logger().warn(
                    f'slide_dir inválido "{slide_dir}" — usando +Y.')
                self._slide_dir_vec = np.array([0.0, 1.0])
            self._user_home_q = np.array(
                [math.radians(float(v)) for v in msg.home_deg],
                dtype=np.float64)
            # Contato aprendido é POR HOME: trocar a home muda a geometria
            # sob a sonda, então o que foi aprendido na home anterior NÃO
            # vale aqui.
            self._home_deg_cur = [float(v) for v in msg.home_deg]
            self._home_key_cur = self._home_key(self._home_deg_cur)
            self._learned_contact_m, _learned_ts = self._lookup_learned(
                self._home_key_cur)
            _learned_now = self._learned_contact_m
        if _learned_now is None:
            self.get_logger().info(
                '[APRENDIZADO] home ainda não aprendida — a descida inteira '
                'roda no rastejo e memoriza a profundidade do contato; os '
                'runs seguintes NESTA home usam o estágio rápido.')
        else:
            # Idade é a informação que decide se dá para confiar: um valor de
            # horas atrás quase certamente é da mesma montagem; um de ontem
            # pode ser de outra amostra na mesma home.
            _age_h = (time.time() - _learned_ts) / 3600.0 if _learned_ts else 0.0
            _proc = (f'aprendido há {_age_h:.1f} h' if _age_h >= 0.1
                     else 'aprendido agora há pouco')
            self.get_logger().info(
                f'[APRENDIZADO] home conhecida — contato em '
                f'{_learned_now * 1e3:.1f} mm ({_proc}): estágio rápido no '
                f'ritmo do slider até a zona lenta e rastejo só no trecho '
                f'final. Se a peça mudou desde então, PARE agora.')
        with self._params_lock:
            # Estabilização do HOLD — 0.0 no msg = usar default do explorer.
            # Banda do HOLD. O override da mensagem vale, mas NUNCA abaixo do
            # ruído da própria célula: uma banda mais estreita que a incerteza
            # da medida não é um critério mais apertado, é um critério que a
            # medição não consegue avaliar — o hold ficaria reiniciando a
            # janela de estabilidade em cima do próprio σ, para sempre.
            _tol_msg = float(msg.hold_tol_n)
            if 0.0 < _tol_msg < _HOLD_TOL_N:
                self.get_logger().warn(
                    f'hold_tol_n={_tol_msg:.3f} N pedido abaixo do piso de '
                    f'ruído da célula ({_HOLD_TOL_SIGMA:.0f}σ = '
                    f'{_HOLD_TOL_N:.3f} N, σ={_FORCE_NOISE_SIGMA_N:.3f} N) — '
                    'elevado ao piso. Para bandas menores é preciso RE-MEDIR '
                    'σ com a FA7155, não apertar o número.')
                _tol_msg = _HOLD_TOL_N
            self._hold_tol_n = _tol_msg if _tol_msg > 0.0 else None
            self._hold_stable_s = (float(msg.hold_stable_s)
                                   if msg.hold_stable_s > 0.0 else None)
            self._hold_timeout_s = (float(msg.hold_timeout_s)
                                    if msg.hold_timeout_s > 0.0 else None)

        # ── MATRIX_MAP: geometria da grade ───────────────────────────
        # A validação acontece AQUI, antes de qualquer movimento: uma matriz
        # inválida recusa o run inteiro em vez de abortar com o braço no meio
        # do plano. Nos demais modos os campos são simplesmente ignorados.
        if self._mode == 'MATRIX_MAP':
            wps = self._parse_matrix_waypoints(msg)
            if wps is None:
                self._set_phase('ABORTED')
                return False
            with self._params_lock:
                self._matrix_wps = wps
                self._matrix_safe_z_m = float(np.clip(
                    float(msg.safe_z_mm) or _MATRIX_SAFE_Z_MM_DEFAULT,
                    _MATRIX_SAFE_Z_MM_MIN, _MATRIX_SAFE_Z_MM_MAX)) / 1000.0
                self._matrix_transit_ms = float(np.clip(
                    float(msg.transit_speed_mms)
                    or _MATRIX_TRANSIT_MMS_DEFAULT,
                    _MATRIX_TRANSIT_MMS_MIN,
                    _MATRIX_TRANSIT_MMS_MAX)) / 1000.0
                self._matrix_shape = str(msg.grid_shape).upper().strip()
                # A matriz é UM experimento; `repeats` não a multiplica (o
                # número de identações é len(waypoints)).
                self._repeats = 1
                # Cada identação parte do Safe Z, então o curso da descida
                # (depth_mm) é gasto ATRAVESSANDO a folga antes de encostar:
                # com depth ≤ safe_z todo waypoint esgota o curso em ar livre e
                # aborta por 'no_contact'. A GUI já corrige isso, mas ela não é
                # o único publisher — um `ros2 topic pub` chegava aqui sem
                # guarda nenhuma e o run inteiro falhava no primeiro ponto.
                _min_depth_mm = self._matrix_safe_z_m * 1e3 * 1.5
                if self._target_depth_mm < _min_depth_mm:
                    self.get_logger().warn(
                        f'[MATRIX] profundidade de {self._target_depth_mm:.1f} mm '
                        f'é menor que 1,5x o Safe Z '
                        f'({self._matrix_safe_z_m*1e3:.1f} mm) — a descida '
                        f'esgotaria o curso antes de tocar. Elevada para '
                        f'{_min_depth_mm:.1f} mm.')
                    self._target_depth_mm = _min_depth_mm
            self.get_logger().info(
                f'[MATRIX] {len(wps)} pontos ({self._matrix_shape or "CUSTOM"}), '
                f'setpoint {self._target_force_n:.2f} N em cada um, '
                f'Safe Z +{self._matrix_safe_z_m * 1e3:.1f} mm sobre a origem, '
                f'trânsito XY a {self._matrix_transit_ms * 1e3:.1f} mm/s. '
                'A ORIGEM (0,0,0) do plano será o primeiro contato — deixe a '
                'sonda parada acima do ponto inicial.')
        else:
            with self._params_lock:
                self._matrix_wps = np.zeros((0, 2))
                self._matrix_origin = None
                self._wp_index = 0
                self._wp_total = 0

        # Os Events já foram limpos em _cb_start, sob o _start_lock e antes
        # do _busy.set() — ver o bloco lá. Limpá-los aqui de novo apagaria um
        # FREEZE legítimo chegado durante este parsing.
        self._protocol_thread = threading.Thread(
            target=self._run_protocol, daemon=True)
        self._protocol_thread.start()
        return True

    # Status
    def _publish_status(self):
        # NÃO grava nada em disco: `_set_phase` também chama este método, na
        # thread do protocolo, e a persistência do aprendizado tem timer
        # próprio (ver _flush_learned).
        with self._lc_lock:
            force_net = self._lc_force_net
        with self._params_lock:
            depth_mm  = float(self._target_depth_mm)
            speed_mms = float(self._slide_speed_mms)
            # Sob modulação o alvo é o valor da onda NESTE instante.
            live = self._force_sp_live
            target_f  = float(self._target_force_n if live is None else live)
            # A home corrente tem contato aprendido? (descida em 2 estágios).
            # Espelhado pela checkbox "Home conhecida" da GUI.
            home_known = self._learned_contact_m is not None
        msg = PalpationStatus()
        msg.phase = self._phase
        msg.cycle = int(self._cycle)
        msg.cycles_total = int(self._cycles_total)
        msg.target_depth_mm = depth_mm
        msg.target_force_n = target_f
        msg.force_net_n = float(force_net)
        msg.speed_mms = speed_mms
        msg.paused = self._pause_requested.is_set()
        msg.home_known = home_known
        # MATRIX_MAP: waypoint corrente + origem descoberta. O logger usa
        # wp_index para carimbar cada amostra de força com o ponto da grade
        # que a produziu; a GUI usa para acender o ponto no preview.
        msg.wp_index = int(self._wp_index)
        msg.wp_total = int(self._wp_total)
        msg.wp_x_mm = float(self._wp_target[0] * 1e3)
        msg.wp_y_mm = float(self._wp_target[1] * 1e3)
        origin = self._matrix_origin
        msg.origin_valid = origin is not None
        if origin is not None:
            msg.origin_x_m = float(origin[0])
            msg.origin_y_m = float(origin[1])
            msg.origin_z_m = float(origin[2])
        self._status_pub.publish(msg)

    def _fz_corrected(self) -> float:
        """Força de contato tare-compensada (N), COM SINAL: + compressão,
        − tração. Para a trava de 15 N use _force_over_limit (magnitude)."""
        with self._lc_lock:
            return self._lc_force_net

    def _fz_slow(self) -> tuple[float, float] | None:
        """(força, janela_s) do canal LENTO, ou None se ele não está no ar.

        None cobre três casos que dão no mesmo para quem chama — nenhum
        publicador (ft_receiver), a janela ainda não anunciada, ou leitura
        velha —, e todos caem no canal rápido."""
        with self._lc_lock:
            val, ts, win = (self._lc_slow_net, self._lc_slow_ts,
                            self._lc_slow_win_s)
        if val is None or win is None:
            return None
        if time.monotonic() - ts > _FORCE_STALE_S:
            return None
        return val, win

    def _force_over_limit(self, fz: float | None = None) -> bool:
        """True se a MAGNITUDE da força cruzou a margem de segurança."""
        v = self._fz_corrected() if fz is None else fz
        return abs(v) > _FORCE_SAFE_LIMIT_N

    def _contact_confirm(self) -> tuple[bool, float]:
        """Confirma (ou desmente) o gatilho de contato com o braço JÁ PARADO."""
        with self._lc_lock:
            seen = self._lc_force_seq
        reads: list[float] = []
        t_end = time.time() + _CONTACT_CONFIRM_S
        while len(reads) < _CONTACT_ON_SAMPLES and time.time() < t_end:
            with self._lc_lock:
                fz, seq = self._lc_force_net, self._lc_force_seq
            if seq != seen:
                seen = seq
                reads.append(fz)
            else:
                time.sleep(_CTRL_DT)
        if not reads:
            self.get_logger().warn(
                '[CONTATO] nenhuma leitura nova em '
                f'{_CONTACT_CONFIRM_S:.1f} s para confirmar — tratando como '
                'falso gatilho.')
            return False, 0.0
        med = float(np.median(reads))
        return med > _CONTACT_ON_N, med

    def _force_stale_abort(self, phase: str) -> bool:
        """True se a leitura de força está velha/ausente — a fase chamadora
        deve abortar com outcome 'stale'. Loga o motivo uma única vez."""
        with self._lc_lock:
            ts = self._lc_force_ts
        if ts > 0.0:
            age = time.monotonic() - ts
            if age <= _FORCE_STALE_S:
                return False
            detail = f'última leitura há {age:.1f} s (> {_FORCE_STALE_S:.1f} s)'
        else:
            detail = 'nenhuma leitura recebida em /load_cell/force_net'
        self.get_logger().error(
            f'SEGURANÇA [{phase}]: célula de carga sem dados frescos — '
            f'{detail}. Controle por força não confiável; abortando. '
            'Verifique a placa da célula e o force_receiver.')
        return True

    def _pause_gate(self) -> bool:
        """Bloqueia enquanto o experimento estiver pausado, segurando a
        posição atual (re-publica o setpoint corrente como o _settle).

        Retorna False se um STOP chegar durante a pausa, ou se a força
        estourar / a célula ficar muda — as mesmas guardas que rodam em todas
        as outras fases. Segurar posição não é motivo para ficar cego: a
        amostra pode ceder, ou alguém pode empurrar a ferramenta, e a pausa
        não tem duração máxima. O chamador trata False como 'stop' (abortar);
        o motivo real fica no log.
        """
        if not self._pause_requested.is_set():
            return True
        self.get_logger().warn('[PAUSE] experimento pausado — segurando posição.')
        q_hold = self._q_now()
        zero_vel = np.zeros(6)
        while self._pause_requested.is_set():
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self.get_logger().warn('[PAUSE] stop durante a pausa.')
                return False
            if self._force_stale_abort('PAUSE'):
                return False
            if self._force_over_limit():
                self.get_logger().error(
                    f'[PAUSE] força {self._fz_corrected():+.1f} N durante a '
                    'pausa — abortando o experimento.')
                return False
            self._stream_q(q_hold, _CTRL_LOOK + _CTRL_DT, velocities=zero_vel)
            time.sleep(_CTRL_DT)
        self.get_logger().info('[PAUSE] experimento retomado.')
        return True

    def _set_phase(self, phase: str):
        self._phase = phase
        self.get_logger().info(f'[FSM] → {phase}')
        self._publish_status()

    # Primitiva de streaming — 1 ponto por mensagem, substitui o goal
    # atual no controller (sem queue). Chamada a cada _CTRL_DT segundos.
    def _stream_q(self, q: np.ndarray, dt_s: float,
                  velocities: np.ndarray | None = None) -> None:
        """Publica 1 setpoint. time_from_start = dt_s (lookahead do ctrl)."""
        msg = JointTrajectory()
        msg.joint_names = list(_ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        if velocities is not None:
            pt.velocities = [float(v) for v in velocities]
        sec = int(dt_s)
        pt.time_from_start = Duration(sec=sec,
                                       nanosec=int((dt_s - sec) * 1e9))
        msg.points.append(pt)
        self._arm_traj_pub.publish(msg)

    def _q_now(self) -> np.ndarray:
        with self._q_lock:
            return self._current_q.copy()

    def _q_sample(self) -> tuple[np.ndarray, float]:
        """Posição E instante de chegada da leitura, atômicos. Quem mede
        velocidade precisa do par: lidos separados, uma /joint_states que
        chegue no meio casa a pose nova com o timestamp da anterior e a
        velocidade sai errada."""
        with self._q_lock:
            return self._current_q.copy(), self._q_ts

    # _settle: publica posição atual por N ticks para zerar lookahead
    # e movimento residual antes de cada transição de fase.
    def _settle(self, ticks: int = _SETTLE_TICKS) -> None:
        q = self._q_now()
        zero_vel = np.zeros(6)
        for _ in range(ticks):
            self._stream_q(q, _CTRL_LOOK + _CTRL_DT, velocities=zero_vel)
            time.sleep(_CTRL_DT)

    def _settle_until_still(self, max_ticks: int = _SETTLE_STILL_MAX_TICKS,
                            tol_rad_s: float = _SETTLE_STILL_TOL_RAD_S,
                            quiet_ticks: int = 3) -> bool:
        """Trava a posição atual (velocidade zero) e ESPERA o braço PARAR de
        fato. Sai quando TODAS as juntas ficam abaixo de `tol_rad_s` por
        `quiet_ticks` leituras consecutivas. Devolve True se parou, False se
        estourou `max_ticks` (aí quem chamou decide o que fazer).

        É o `_settle` com o critério que faltava. O `_settle` publica a pose
        travada por 6 ticks fixos e volta — freia, mas não verifica. Enquanto
        houver velocidade residual, quem manda no TCP não é o eixo comandado:
        o DESCENDING só pede Z, então a junta que ainda freia empurra o TCP
        para os lados. Foi o que aconteceu em 20260901_094646.

        Mede entre LEITURAS NOVAS de /joint_states, não entre ticks: o tópico
        chega a ~24 Hz contra os 33 Hz do laço, então reler a mesma pose daria
        Δq = 0 e declararia parado um braço em movimento."""
        q_hold, ts_prev = self._q_sample()
        q_prev = q_hold
        zero_vel = np.zeros(6)
        quiet = 0
        for _ in range(max_ticks):
            # Segura SEMPRE a pose de entrada: é ela que freia. Re-travar na
            # pose corrente a cada tick perseguiria a deriva em vez de parar.
            self._stream_q(q_hold, _CTRL_LOOK + _CTRL_DT, velocities=zero_vel)
            time.sleep(_CTRL_DT)
            q_now, ts_now = self._q_sample()
            if ts_now <= ts_prev:
                continue                  # nada novo chegou: não é amostra
            v_max = float(np.max(np.abs(q_now - q_prev))) / (ts_now - ts_prev)
            q_prev, ts_prev = q_now, ts_now
            if v_max < tol_rad_s:
                quiet += 1
                if quiet >= quiet_ticks:
                    return True
            else:
                quiet = 0
        return False

    def _settle_until_quiet(self, max_ticks: int = 20,
                            dfdt_tol_n: float = 0.05,
                            quiet_ticks: int = 3) -> None:
        """Trava a posição atual (velocidade zero) e ESPERA a força assentar
        antes de devolver o controle ao loop de força. Sai quando |ΔF| entre
        ticks fica < `dfdt_tol_n` por `quiet_ticks` consecutivos, ou ao atingir
        `max_ticks`. Absorve a inércia/lookahead herdados do DESCENDING contra
        contato rígido — é o que evita o pico no handoff DESCENDING→HOLD."""
        q = self._q_now()
        zero_vel = np.zeros(6)
        f_prev = self._fz_corrected()
        quiet = 0
        for _ in range(max_ticks):
            self._stream_q(q, _CTRL_LOOK + _CTRL_DT, velocities=zero_vel)
            time.sleep(_CTRL_DT)
            f_now = self._fz_corrected()
            if abs(f_now - f_prev) < dfdt_tol_n:
                quiet += 1
                if quiet >= quiet_ticks:
                    return
            else:
                quiet = 0
            f_prev = f_now

    @staticmethod
    def _deriva(win: list[float]) -> float:
        """|mediana da 2ª metade − mediana da 1ª|. Mede TENDÊNCIA, que é o que
        distingue creep de ruído: o ruído não tem sinal preferido entre as
        metades, o creep tem."""
        n = len(win)
        half = n // 2
        if half < 1:
            return 0.0
        return abs(float(np.median(win[half:])) - float(np.median(win[:half])))

    def _qs_measure_settled(self, q: np.ndarray) -> float | None:
        """Medida assentada pelo CANAL LENTO. None se ele não está no ar.

        Congela o braço pela janela INTEIRA da média móvel antes de ler: a
        média que chega agora ainda contém amostras de ANTES do micro-passo,
        e lê-la cedo devolveria a força da posição anterior. É o preço do σ
        menor, e ele é de graça aqui — só o caminho `settle` chama (perto do
        alvo), e o assentamento MECÂNICO do contato é da ordem de 4,5 s,
        maior que a janela.

        Depois da janela, mantém o teste de deriva do caminho antigo: encher
        a média não garante que a FÍSICA parou, e é a deriva que diz isso.
        """
        got = self._fz_slow()
        if got is None:
            return None
        _, win_s = got
        zero_vel = np.zeros(6)

        def _tick() -> tuple[float, bool] | None:
            """Um tick congelado. Devolve (fz_lento, pare_agora) ou None se o
            canal lento sumiu. `pare_agora` cobre o limite de força e o STOP
            do usuário — sem ele a espera pela janela seguraria um pedido de
            parada por até `win_s`, contra o ~1 s do caminho rápido. NÃO
            limpa o evento: quem o consome é o _qs_regulate, na volta
            seguinte."""
            self._stream_q(q, _CTRL_LOOK + _CTRL_DT, velocities=zero_vel)
            time.sleep(_CTRL_DT)
            g = self._fz_slow()
            if g is None:              # canal caiu no meio — quem chama
                return None            # refaz pelo caminho rápido
            return g[0], (self._force_over_limit(g[0])
                          or self._stop_requested.is_set())

        t_end = time.time() + win_s
        while time.time() < t_end:
            r = _tick()
            if r is None:
                return None
            if r[1]:
                return r[0]
            got = (r[0], win_s)
        reads: list[float] = [got[0]]
        for _ in range(_QS_SETTLE_MAX_TICKS):
            r = _tick()
            if r is None:
                return None
            if r[1]:
                return r[0]
            reads.append(r[0])
            if len(reads) >= 2 * _QS_MEDIAN_N and \
                    self._deriva(reads) < _QS_SETTLE_DRIFT_N:
                break
        return reads[-1]

    def _qs_measure_fz(self, q_hold: np.ndarray | None = None,
                       settle: bool = False) -> float:
        """Mede a força em repouso (modo quase-estático): congela o braço por
        _QS_SETTLE_TICKS (o pipeline One-Euro + JTC esvazia) e devolve a
        mediana das últimas _QS_MEDIAN_N leituras DISTINTAS.

        Distintas por `seq`, e não por posição na lista: o tick de controle é
        mais rápido que a amostra da célula, então ler duas vezes na mesma
        janela devolve duas cópias do mesmo número. Uma mediana de 3 com uma
        repetição dentro é uma mediana de 2 — perde a rejeição de outlier
        exatamente onde ela importa. Ver _QS_MEASURE_MAX_TICKS.

        `settle` (perto do alvo) prefere o CANAL LENTO, que troca latência —
        de graça com o braço já congelado — por σ ~19 mN em vez de ~33 mN.
        Sem esse canal no ar (ft_receiver), cai no caminho abaixo, que é o
        comportamento de antes.
        """
        q = self._q_now() if q_hold is None else q_hold
        if settle:
            fz = self._qs_measure_settled(q)
            if fz is not None:
                return fz
        zero_vel = np.zeros(6)
        reads: list[float] = []
        with self._lc_lock:
            seen = self._lc_force_seq
        max_ticks = (_QS_SETTLE_MAX_TICKS if settle else _QS_MEASURE_MAX_TICKS)
        for tick in range(max_ticks):
            self._stream_q(q, _CTRL_LOOK + _CTRL_DT, velocities=zero_vel)
            time.sleep(_CTRL_DT)
            with self._lc_lock:
                fz, seq = self._lc_force_net, self._lc_force_seq
            if self._force_over_limit(fz):
                return fz
            if seq != seen:
                seen = seq
                reads.append(fz)
            # Os _QS_SETTLE_TICKS são o assentamento e não são negociáveis: é
            # o tempo que o One-Euro + a fila do JTC levam para esvaziar. Só
            # depois deles é que ter as amostras basta para sair.
            if tick + 1 < _QS_SETTLE_TICKS or len(reads) < _QS_MEDIAN_N:
                continue
            if not settle:
                break
            # Modo assentado: só sai quando a força PARA de derivar. Precisa de
            # amostras suficientes para as duas metades da janela dizerem algo.
            if len(reads) >= 2 * _QS_MEDIAN_N and \
                    self._deriva(reads) < _QS_SETTLE_DRIFT_N:
                break
        if not reads:
            # Célula muda. Devolver a última leitura conhecida mantém o
            # comportamento anterior; quem chamou aborta por _force_stale_abort,
            # que é o guarda certo para isso e já roda a cada volta do
            # _qs_regulate.
            return self._fz_corrected()
        return float(np.median(reads[-_QS_MEDIAN_N:]))

    def _qs_step(self, approach_dir: np.ndarray, step_m: float,
                 v_lim: float, I6: np.ndarray,
                 q_from: np.ndarray | None = None,
                 dt: float | None = None,
                 deadline: float | None = None) -> np.ndarray | None:
        """Executa UM micro-passo ao longo do approach em 1 tick, partindo da
        posição COMANDADA `q_from` (ou da medida, se None). Devolve o novo q
        comandado — o chamador congela NELE até o próximo passo.

        `dt` é o período do tick. Default `_CTRL_DT` (33 Hz), que é a cadência
        da regulação quase-estática; a onda trigonométrica passa o seu próprio,
        derivado da frequência pedida — a 33 Hz nem 5 Hz tem pontos suficientes
        por período.

        `deadline` (time.monotonic absoluto) troca o sleep RELATIVO por espera
        até um instante fixo. A diferença só importa na onda: dormir `dt`
        DEPOIS do Jacobiano, da IK e do publish faz o período real ser
        dt + trabalho, sempre. A 10 Hz o `dt` já está no piso de 20 ms do
        ServoJ e não há folga nenhuma, então esse excedente derruba a onda
        abaixo dos 5 pontos por período E faz a grade de fase escorregar
        debaixo do ILC, que indexa a correção por fase. Com o deadline o erro
        não acumula: cada tick corrige o atraso do anterior.
        """
        dt = _CTRL_DT if dt is None else dt

        def _wait() -> None:
            if deadline is None:
                time.sleep(dt)
                return
            rem = deadline - time.monotonic()
            if rem > 0.0:
                time.sleep(rem)

        q = self._q_now() if q_from is None else q_from
        if step_m == 0.0:
            _wait()
            return q
        tw = np.zeros(6)
        tw[:3] = approach_dir * step_m
        J = jacobian(q, T_end=T_TOUCH_TOOL_ATTACH)
        try:
            dq = J.T @ np.linalg.solve(J @ J.T + _JAC_LAM**2 * I6, tw)
        except np.linalg.LinAlgError:
            return None
        q_new = np.clip(q + dq, JOINT_MIN, JOINT_MAX)
        vel = np.clip((q_new - q) / dt, -v_lim, v_lim)
        self._stream_q(q_new, dt, velocities=vel)
        _wait()
        return q_new

    def _qs_regulate(self, target_f: float, tol_n: float,
                     approach_dir: np.ndarray, v_lim: float, I6: np.ndarray,
                     *, budget_m: float | None, stable_s: float,
                     timeout_s: float, phase: str,
                     dynamic: bool = False,
                     feed_curve: bool = False) -> tuple[str, float]:
        """Regulação de força por RAMPA A VELOCIDADE CONSTANTE (quase-estática).

        Substitui a lei proporcional Δx = relax·err/K_est (ver o bloco
        NÃO-ULTRAPASSAGEM e os _QS_* daquela lei): o overshoot dela era
        estrutural, porque K_est é uma EMA do trecho JÁ percorrido de um
        contato que enrijece e o passo err/K_est atravessava o alvo.

        Agora:
          ETAPA A — move o TCP ao longo do eixo de ataque a velocidade
            CONSTANTE (`hold_ramp_mms`, efetivo ≈ 1/6 pelo custo de medida),
            no sentido de sinal(alvo − fz), até a força medida CRUZAR o
            setpoint. O passo é v·dt FIXO; a rigidez NÃO é um ganho — entra
            só em dois clamps que ENCURTAM esse passo: ΔF projetado ≤
            _QS_RAMP_DF_CAP_N e NÃO-ULTRAPASSAGEM (o passo não cruza o alvo
            nem na pior rigidez, _k_est.k_upper). Perto do alvo isso torna a
            aproximação geométrica POR BAIXO — sem overshoot mesmo com K
            errada.
          ETAPA B — congela a posição por `stable_s` s de relógio. Se a força
            sai da banda (relaxa para baixo num degrau de carga, RECUPERA
            para cima depois de um degrau de descarga), retoma a rampa —
            limitada pela não-ultrapassagem, minúscula perto do alvo — até
            recruzar e recongela; o relógio de `stable_s` NÃO reinicia.
            `dynamic=True` faz o mesmo mas NUNCA retorna sozinho: segue o
            setpoint corrente (/palpation/set_force) até stop/force/stale/
            target_lost.

        Overshoot ≤ 1 tick de curso e tende a zero perto do alvo; tempo de
        patamar = curso/v + stable_s. As duas coisas que a lei proporcional
        não dava (lá o pico passava 0,2–0,6 N e o patamar durava de 16 a 53 s
        para um dwell de 10 s).

        `feed_curve` alimenta a curva F(x) (_fx_curve) com os pares
        (deepened_m, fz) — só a DESCIDA liga isto, para a onda SINE/COSINE. O
        estimador de rigidez (_k_est) é alimentado SEMPRE: não governa a
        velocidade, mas os dois clamps de segurança o usam.

        Retorna (status, fz): 'ok' | 'timeout' | 'budget' | 'force'
                              | 'stale' | 'stop' | 'target_lost'.
        """
        dt = _CTRL_DT
        try:
            v_ramp = max(1.0e-5,
                         float(self.get_parameter('hold_ramp_mms').value) / 1e3)
        except Exception:
            v_ramp = _QS_RAMP_V_MS

        t_start = time.time()
        t_cross: float | None = None   # instante em que a força cruzou o alvo
        self._qs_ever_contact = False
        deepened_m = 0.0
        fz_prev: float | None = None
        step_prev = 0.0
        err_prev: float | None = None
        # Sentido da rampa da ETAPA A: fixo para um alvo constante, recalculado
        # a cada tick no MANUAL dinâmico. +1 = aprofundar, −1 = recuar.
        sign0 = 0.0
        # Vigia de ALVO RETIRADO (ver _TARGET_LOST_*): curso livre acumulado
        # DEPOIS de o contato ter sido firmado, e o instante em que a força
        # ainda estava carregada.
        free_since_contact_m = 0.0
        t_last_loaded: float | None = None
        q_cmd: np.ndarray | None = None   # última posição COMANDADA — congela
                                          # nela, senão o freeze desfaz passos
                                          # sub-LSB ainda não executados
        while True:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self.get_logger().warn(f'[STOP] {phase} interrompido pelo usuário.')
                return 'stop', 0.0
            if not self._pause_gate():
                return 'stop', 0.0
            if self._force_stale_abort(phase):
                return 'stale', 0.0
            if dynamic:
                # MANUAL: segue o setpoint atualizado on-the-fly, sem reiniciar
                # a FSM. A tolerância acompanha o novo alvo.
                with self._params_lock:
                    target_f = float(self._target_force_n)
                tol_n = (self._hold_tol_n if self._hold_tol_n is not None
                         else max(_HOLD_TOL_N, _HOLD_TOL_PCT * target_f))

            # Perto do alvo a medida ESPERA a força assentar (rejeita o creep
            # que causava overshoot); longe dele mede rápido. `err_prev` é o
            # erro da volta anterior — uma volta de atraso não muda o regime.
            perto = (err_prev is not None
                     and abs(err_prev) <= _QS_SETTLE_NEAR_MULT * tol_n)
            fz = self._qs_measure_fz(q_cmd, settle=perto)
            if self._force_over_limit(fz):
                self._relieve_contact(approach_dir)
                self.get_logger().error(
                    f'SEGURANÇA: força {fz:+.1f} N além da margem de '
                    f'{_FORCE_SAFE_LIMIT_N:.0f} N (teto '
                    f'{_FORCE_ABORT_LIMIT_N:.0f} N) — medição cancelada.')
                return 'force', fz

            in_contact = fz > _CONTACT_ON_N
            # ── vigia de ALVO RETIRADO ───────────────────────────────
            # Só arma DEPOIS de o contato ter existido: antes disso "sem
            # força" é a aproximação normal, não uma anomalia.
            if in_contact:
                free_since_contact_m = 0.0
                self._qs_ever_contact = True
                if fz >= _TARGET_LOST_DROP_FRAC * target_f:
                    t_last_loaded = time.time()
                # A curva F(x): `deepened_m` é a penetração COMANDADA desta
                # fase e `fz` foi lida em REPOUSO logo acima — o par que o
                # move-then-measure produz de graça. A onda o interpola no
                # lugar do escalar K.
                if feed_curve:
                    self._fx_curve.add(deepened_m, fz)
            elif self._qs_ever_contact:
                # Sem contato depois de ter havido: dois critérios, basta um.
                lost_by_travel = free_since_contact_m >= _TARGET_LOST_FREE_M
                lost_by_drop = (t_last_loaded is not None
                                and (time.time() - t_last_loaded)
                                <= _TARGET_LOST_DROP_S
                                and free_since_contact_m > 0.0)
                if lost_by_travel or lost_by_drop:
                    motivo = ('avançou '
                              f'{free_since_contact_m*1e3:.2f} mm sem nenhuma '
                              'reação de força'
                              if lost_by_travel else
                              f'a força caiu de ≥{_TARGET_LOST_DROP_FRAC*target_f:.2f} N '
                              f'para {fz:+.2f} N em menos de '
                              f'{_TARGET_LOST_DROP_S:.1f} s')
                    self.get_logger().error(
                        f'SEGURANÇA [{phase}]: ALVO PERDIDO — {motivo}. A '
                        'rigidez do contato colapsou a zero, que é a '
                        'assinatura da amostra ter sido retirada (ou da '
                        'fixação ter cedido). O ServoJ segue posição e não '
                        'perceberia isto sozinho: abortando antes que o braço '
                        'continue descendo no vazio.')
                    self._relieve_contact(approach_dir, floor_n=_CONTACT_ON_N)
                    return 'target_lost', fz

            # Estimador de rigidez — alimentado SEMPRE (não governa o passo,
            # mas o clamp de segurança abaixo o usa). Mesmo gate do regulador
            # anterior: par válido com o contato estabelecido, ou passo que
            # atravessou a fronteira empurrando.
            if fz_prev is not None and step_prev != 0.0 \
                    and (in_contact or step_prev > 0.0):
                self._k_est.update_pair(step_prev, fz - fz_prev)

            err = target_f - fz
            err_prev = err
            now = time.time()
            in_band = abs(err) <= tol_n

            # ── ETAPA B: a força já cruzou o alvo → defender o patamar ──
            if t_cross is None and (in_band
                                    or (sign0 != 0.0 and sign0 * err <= 0.0)):
                t_cross = now
                self.get_logger().info(
                    f'{phase}: alvo cruzado (fz={fz:.2f} N, alvo '
                    f'{target_f:.2f} ± {tol_n:.2f}) — defendendo '
                    f'{stable_s:.1f} s.')
            if t_cross is not None:
                if not dynamic and now - t_cross >= stable_s:
                    # A defesa mantém o relógio de stable_s correndo mesmo
                    # re-rampando, então "chegou ao fim" não garante "em
                    # banda AGORA". Devolver 'timeout' quando a última
                    # leitura está fora preserva o aviso "trate com ressalva"
                    # do chamador (escada/HOLD) sem reiniciar o relógio.
                    return ('ok' if abs(err) <= tol_n else 'timeout'), fz
                # Defesa DOS DOIS LADOS: a relaxação viscoelástica tira a
                # força para baixo num degrau de carga e a RECUPERA para cima
                # depois de um degrau de descarga — as duas precisam de
                # correção. O passo continua limitado por _QS_NO_CROSS_FRAC·
                # |err|/k_upper, que é minúsculo perto do alvo, então recuar
                # aqui não larga o contato (o risco que a lei antiga tinha,
                # com passo err/k_push grande). Relógio de stable_s NÃO
                # reinicia.
                if err > tol_n:
                    sign_now = 1.0
                elif err < -tol_n:
                    sign_now = -1.0
                else:
                    sign_now = 0.0
            else:
                # ── ETAPA A: ainda não cruzou o alvo ──
                if not dynamic and now - t_start >= timeout_s:
                    return 'timeout', fz
                if sign0 == 0.0:
                    sign0 = 1.0 if err > 0.0 else -1.0
                sign_now = (1.0 if err > 0.0 else -1.0) if dynamic else sign0

            # ── passo de rampa a velocidade constante ──
            if sign_now == 0.0:
                step_m = 0.0
            else:
                # O passo é v·dt FIXO (velocidade constante). Dois clamps de
                # SEGURANÇA — nenhum é ganho, só ENCURTAM o passo fixo:
                #  (a) ΔF projetado ≤ _QS_RAMP_DF_CAP_N pela rigidez local. A
                #      cota é CONSERVADORA até haver medida: `k_upper` vale
                #      _K_MAX_NM enquanto nada foi percorrido, então o 1º tick
                #      num contato de rigidez DESCONHECIDA projeta ≤
                #      _QS_RAMP_DF_CAP_N mesmo que ele seja rígido; a cota do
                #      "resultado nulo" em `k_upper` afrouxa sozinha conforme
                #      o curso sem resposta cresce, então contato mole não
                #      trava. Medida a rigidez, usa a EMA (`value`).
                #  (b) NÃO-ULTRAPASSAGEM: o passo não cruza o alvo nem na pior
                #      rigidez (`k_upper`). Perto do alvo (a) e (b) fazem a
                #      aproximação virar geométrica POR BAIXO, sem overshoot
                #      mesmo com K errada.
                step_mag = v_ramp * dt
                if in_contact:
                    k_cap = (self._k_est.value if self._k_est.estimated
                             else self._k_est.k_upper)
                    step_mag = min(
                        step_mag,
                        _QS_RAMP_DF_CAP_N / max(k_cap, _K_MIN_NM),
                        _QS_NO_CROSS_FRAC * abs(err)
                        / max(self._k_est.k_upper, _K_MIN_NM))
                else:
                    # Fora do contato (re-aproximação após perda): passo curto,
                    # senão reentrar num contato rígido a v·dt cheio bate —
                    # o quique de ~10 N da bancada. O clamp em contato assume
                    # no tick seguinte.
                    step_mag = min(step_mag, _QS_FREE_STEP_MAX_M)
                step_m = sign_now * step_mag

            if budget_m is not None and deepened_m + step_m > budget_m:
                step_m = budget_m - deepened_m
                if step_m <= 1e-7 and sign_now > 0.0:
                    return 'budget', fz
            q_new = self._qs_step(approach_dir, step_m, v_lim, I6,
                                  q_from=q_cmd)
            if q_new is not None:
                q_cmd = q_new
                deepened_m += step_m
                fz_prev, step_prev = fz, step_m
                if not in_contact and step_m > 0.0:
                    # Avanço em busca de contato: é ESTE curso que o vigia de
                    # alvo retirado mede. Só aprofundar conta.
                    free_since_contact_m += step_m

    @staticmethod
    def _free_descent_v(descended_m: float,
                        ramp_from_m: float | None,
                        crawl_from_m: float | None,
                        v_fast_ms: float, v_slow_ms: float,
                        v_unlearned_ms: float) -> float:
        """Velocidade do próximo passo em AR LIVRE — o perfil de três estágios.

        `ramp_from_m is None` = home sem contato aprendido: a descida inteira
        respeita o orçamento de impacto, porque o contato pode vir a qualquer
        momento. Com contato aprendido:

          descended < ramp_from   RÁPIDO   v_fast (slider da GUI)
          até crawl_from          FRENAGEM rampa linear v_fast → v_slow
          daí em diante           RASTEJO  v_slow (= crawl_v_ms)

        A INVARIANTE que importa: de `crawl_from_m` em diante — a janela
        `learned ± margem`, a única onde o contato pode estar — a velocidade é
        exatamente `v_slow`, nunca mais que isso. É ela que mantém o pico do
        primeiro toque em `v·T_halt·K = _CONTACT_ON_N`. A frenagem corre mais
        rápido só porque está inteira ACIMA dessa janela.

        Função pura, e é por isso que ela é uma função: essa invariante precisa
        ser verificável sem rodar a FSM (mesma razão de `_qs_relief_step`)."""
        if ramp_from_m is None or crawl_from_m is None:
            return v_unlearned_ms
        if descended_m < ramp_from_m:
            return v_fast_ms
        if descended_m >= crawl_from_m:
            return v_slow_ms
        frac = ((descended_m - ramp_from_m)
                / max(crawl_from_m - ramp_from_m, 1e-9))
        return v_fast_ms + (v_slow_ms - v_fast_ms) * min(max(frac, 0.0), 1.0)

    def _relieve_contact(self, approach_dir: np.ndarray,
                         max_ticks: int = 20, *,
                         floor_n: float | None = None) -> None:
        """Alívio de EMERGÊNCIA ao cruzar a margem de força: recua ao longo
        do approach a passo cheio (0.25 mm/tick ≈ 8 mm/s) até a compressão
        cair abaixo de metade da margem, então trava a posição. Congelar no
        lugar (_settle) MANTINHA a compressão — na coleta de 02/07 a força
        ficou 90 ms acima do teto de 15 N esperando o abort subir à home.

        `floor_n` é a força ABAIXO da qual o recuo para. O default (metade da
        margem, 6 N) é o da emergência: o objetivo lá é sair do teto depressa,
        não descarregar. Quem quer DESCARREGAR de fato — o retorno à HOME —
        passa `_CONTACT_ON_N`; com o default, um contato normal de 2 N já
        entra abaixo do limiar e o laço não recua nada."""
        floor = (0.5 * _FORCE_SAFE_LIMIT_N if floor_n is None
                 else float(floor_n))
        I6 = np.eye(6)
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        for _ in range(max_ticks):
            if abs(self._fz_corrected()) < floor:
                break
            tw = np.zeros(6)
            tw[:3] = -approach_dir * _DEADBEAT_DX_MAX_M
            q = self._q_now()
            J = jacobian(q, T_end=T_TOUCH_TOOL_ATTACH)
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + _JAC_LAM**2 * I6, tw)
            except np.linalg.LinAlgError:
                break
            q_new = np.clip(q + dq, JOINT_MIN, JOINT_MAX)
            vel = np.clip((q_new - q) / _CTRL_DT, -v_lim, v_lim)
            self._stream_q(q_new, _CTRL_DT, velocities=vel)
            time.sleep(_CTRL_DT)
        self._settle()

    def _home_v_rad_s(self) -> float:
        """Velocidade máxima por junta dos retornos HOME (rad/s), saturada."""
        try:
            v = float(self.get_parameter('home_speed_rad_s').value)
        except Exception:
            v = _HOME_MAX_RAD_S
        return float(min(max(v, 0.01), 0.30))

    # Movimento no espaço de juntas: interpola linearmente q_from → q_to
    # a velocidade máxima de home_speed_rad_s rad/s por junta.
    def _joint_stream_to(self, q_target: np.ndarray) -> bool:
        q_from = self._q_now()
        delta = np.asarray(q_target, float) - q_from
        max_d = float(np.max(np.abs(delta)))
        if max_d < 0.001:
            return True
        n_steps = max(1, int(math.ceil(max_d / (self._home_v_rad_s() * _CTRL_DT))))
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        vel_peak = np.clip(delta / n_steps / _CTRL_DT, -v_lim, v_lim)
        # Rampa trapezoidal: ~20 % de aceleração/desaceleração (máx 8 passos = 240 ms).
        # Evita o solavanco de arranque causado por velocidade constante desde t=0.
        ramp = min(max(1, n_steps // 5), 8)
        for i in range(1, n_steps + 1):
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                return False
            alpha = i / n_steps
            q = np.clip(q_from + alpha * delta, JOINT_MIN, JOINT_MAX)
            if i <= ramp:
                scale = i / ramp
            elif i >= n_steps - ramp + 1:
                scale = (n_steps - i + 1) / ramp
            else:
                scale = 1.0
            step_vel = vel_peak * scale if i < n_steps else np.zeros(6)
            self._stream_q(q, _CTRL_DT, velocities=step_vel)
            time.sleep(_CTRL_DT)
        return True

    def _cartesian_stream(self, direction: np.ndarray, total_m: float, *,
                           v_const_ms: float | None = None,
                           v_max_ms: float | None = None,
                           v_min_ms: float | None = None,
                           lock_ori: bool = False,
                           lock_z: bool = False,
                           lock_perp: bool = False,
                           force_threshold_n: float | None = None,
                           win: int = _CTRL_WIN) -> str:
        """Streaming Jacobiano a 33 Hz, sem pré-planejamento. Retorna
        'done' | 'force' | 'stop' | 'error'."""
        d = np.asarray(direction, dtype=float).flatten()
        nd = float(np.linalg.norm(d))
        if nd < 1e-9 or total_m <= 0.0:
            self.get_logger().error('_cartesian_stream: direção/distância inválida.')
            return 'error'
        d /= nd

        constant = v_const_ms is not None
        if not constant and (v_max_ms is None or v_min_ms is None):
            self.get_logger().error(
                '_cartesian_stream: forneça v_const_ms OU (v_max_ms, v_min_ms).')
            return 'error'

        I6 = np.eye(6)

        # p_start mede o progresso real do TCP via FK, em vez de integrar
        # passos comandados (que acumula erro do Jacobiano e do clipping).
        T0 = forward_kinematics(self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)
        p_start = T0[:3, 3].copy()

        R0: np.ndarray | None = None
        z0: float | None = None
        perp_dir: np.ndarray | None = None
        p0_perp: float | None = None
        if lock_ori:
            R0 = T0[:3, :3].copy()
        if lock_z:
            z0 = float(T0[2, 3])
        if lock_perp:
            # Perpendicular a d no plano XY. Para d = [0,0,±1] a norma é zero
            # e a correção é suprimida automaticamente.
            perp = np.array([-d[1], d[0], 0.0])
            pnorm = float(np.linalg.norm(perp))
            if pnorm > 1e-9:
                perp_dir = perp / pnorm
                p0_perp = float(p_start @ perp_dir)

        # Safety: timeout baseado em 10× o tempo nominal + margem de 30 s.
        v_est = float(v_const_ms) if constant else float(v_max_ms)
        v_est = max(1e-4, v_est)
        _timeout_s = max(30.0, (total_m / v_est) * 10.0)
        _t0 = time.time()
        # Detecção de direção errada: > 5 mm na direção negativa por > 3 s.
        _neg_ticks = 0
        _NEG_MAX = int(3.0 / _CTRL_DT)
        # Log diagnóstico a cada 1 s.
        _log_every = max(1, int(1.0 / _CTRL_DT))
        _tick = 0

        self.get_logger().info(
            f'_cartesian_stream: d={d.round(3)} total={total_m*1e3:.1f}mm '
            f'v={v_est*1e3:.1f}mm/s p_start={p_start.round(4)} '
            f'TCP_Z={T0[:3,2].round(3)}')

        # Progresso real do TCP na direção d (metros, medido via FK a cada tick).
        progress = 0.0

        while progress < total_m:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                return 'stop'

            # Timeout global — evita loop eterno se o robô não se move.
            if time.time() - _t0 > _timeout_s:
                self.get_logger().error(
                    f'_cartesian_stream: timeout {_timeout_s:.0f}s '
                    f'(progress={progress*1e3:.1f}mm/{total_m*1e3:.1f}mm). Abortando.')
                return 'error'

            if force_threshold_n is not None:
                if self._fz_corrected() >= force_threshold_n:
                    return 'force'

            q = self._q_now().copy()

            # FK do tick atual — única chamada por iteração.
            # Serve tanto para medir o progresso real quanto para as correções
            # de orientação, Z e perpendicular.
            T_cur = forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)
            progress = float(np.dot(T_cur[:3, 3] - p_start, d))

            # Detecção de direção errada: se TCP persistentemente se afasta
            # de p_start na direção oposta a d, o Jacobiano provavelmente está
            # sendo calculado numa configuração errada. Aborta para não bloquear.
            if progress < -0.005:
                _neg_ticks += 1
                if _neg_ticks > _NEG_MAX:
                    self.get_logger().error(
                        f'_cartesian_stream: TCP na direção errada '
                        f'(progress={progress*1e3:.1f}mm por >{3.0:.0f}s). '
                        f'TCP_cur={T_cur[:3,3].round(4)} p_start={p_start.round(4)}. '
                        'Abortando.')
                    return 'error'
            else:
                _neg_ticks = 0

            # Log periódico para diagnóstico.
            if _tick % _log_every == 0:
                self.get_logger().debug(
                    f'  t={_tick*_CTRL_DT:.1f}s progress={progress*1e3:.2f}mm '
                    f'TCP={T_cur[:3,3].round(4)}')
            _tick += 1

            # Perfil de velocidade usa o progresso FK (não passos acumulados).
            u = max(0.0, min(1.0, progress / total_m))
            if constant:
                v = float(v_const_ms)
            else:
                v = float(v_min_ms) + (float(v_max_ms) - float(v_min_ms)) * (1.0 - u) ** 2
            v = max(1e-4, v)
            step = v * _CTRL_DT

            # ── Batch de `win` waypoints (janela deslizante) ─────────────────
            # Cada mensagem contém `win` pontos com timestamps cumulativos.
            msg = JointTrajectory()
            msg.joint_names = list(_ARM_JOINTS)
            q_iter = q.copy()
            T_iter = T_cur
            v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
            singular = False
            for k in range(1, win + 1):
                tw = np.zeros(6)
                tw[:3] = d * step
                if R0 is not None:
                    R_err = R0 @ T_iter[:3, :3].T
                    tw[3:] = _ORI_GAIN * 0.5 * np.array([
                        R_err[2, 1] - R_err[1, 2],
                        R_err[0, 2] - R_err[2, 0],
                        R_err[1, 0] - R_err[0, 1],
                    ])
                if z0 is not None:
                    tw[2] += _Z_CORR_GAIN * (z0 - float(T_iter[2, 3]))
                if perp_dir is not None and p0_perp is not None:
                    perp_err = p0_perp - float(T_iter[:3, 3] @ perp_dir)
                    tw[:3] += _Z_CORR_GAIN * perp_err * perp_dir
                J_k = jacobian(q_iter, T_end=T_TOUCH_TOOL_ATTACH)
                try:
                    dq_k = J_k.T @ np.linalg.solve(
                        J_k @ J_k.T + _JAC_LAM**2 * I6, tw)
                except np.linalg.LinAlgError:
                    singular = True
                    break
                q_next = np.clip(q_iter + dq_k, JOINT_MIN, JOINT_MAX)
                vel_k = np.clip((q_next - q_iter) / _CTRL_DT, -v_lim, v_lim)
                pt = JointTrajectoryPoint()
                pt.positions = [float(x) for x in q_next]
                pt.velocities = [float(x) for x in vel_k]
                t_k = k * _CTRL_DT
                pt.time_from_start = Duration(
                    sec=int(t_k), nanosec=int((t_k - int(t_k)) * 1e9))
                msg.points.append(pt)
                q_iter = q_next
                if k < win:
                    T_iter = forward_kinematics(q_iter, T_end=T_TOUCH_TOOL_ATTACH)
            if singular:
                self.get_logger().warn('Jacobiano singular — passo descartado.')
                time.sleep(_CTRL_DT)
                continue
            if msg.points:
                self._arm_traj_pub.publish(msg)
            time.sleep(_CTRL_DT)

        return 'done'

    def _cartesian_batch_to(self, direction: np.ndarray, total_m: float, *,
                              v_const_ms: float | None = None,
                              v_max_ms: float | None = None,
                              v_min_ms: float | None = None,
                              lock_ori: bool = False,
                              lock_z: bool = False,
                              lock_perp: bool = False) -> str:
        """Pré-computa todos os waypoints via Jacobiano iterado e envia em
        UMA JointTrajectory (JTC planeja a S-curve sobre o conjunto inteiro).

        A trajetória não é replanejada durante a execução, mas a espera É
        vigiada: força, frescor da célula, STOP e PAUSA são checados a cada
        tick, e qualquer um deles substitui o goal do JTC (via _settle ou
        _pause_gate), o que PARA o braço no meio do percurso. Sem isso um
        relevo mais alto que a folga de trânsito era percorrido inteiro
        contra a peça antes de alguém notar — e a checagem de força de quem
        chama só via o estrago no fim.

        Retorna 'done' | 'stop' | 'force' | 'stale' | 'paused'  | 'error'.
        'paused' = a pausa foi servida e o goal se perdeu com ela: quem
        chama tem de replanejar o que faltava (ver _move_linear_world)."""
        d = np.asarray(direction, dtype=float).flatten()
        nd = float(np.linalg.norm(d))
        if nd < 1e-9 or total_m <= 0.0:
            self.get_logger().error('_cartesian_batch_to: direção/distância inválida.')
            return 'error'
        d /= nd

        constant = v_const_ms is not None
        if not constant and (v_max_ms is None or v_min_ms is None):
            self.get_logger().error(
                '_cartesian_batch_to: forneça v_const_ms OU (v_max_ms, v_min_ms).')
            return 'error'

        v_ref = float(v_const_ms) if constant else float(v_max_ms)
        v_ref = max(1e-4, v_ref)
        N = max(1, int(math.ceil(total_m / (v_ref * _CTRL_DT))))

        q = self._q_now()
        T0 = forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)

        R0 = T0[:3, :3].copy() if lock_ori else None
        z0 = float(T0[2, 3]) if lock_z else None
        perp_dir: np.ndarray | None = None
        p0_perp: float | None = None
        if lock_perp:
            perp = np.array([-d[1], d[0], 0.0])
            pnorm = float(np.linalg.norm(perp))
            if pnorm > 1e-9:
                perp_dir = perp / pnorm
                p0_perp = float(T0[:3, 3] @ perp_dir)

        I6 = np.eye(6)
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        self.get_logger().info(
            f'_cartesian_batch_to: pré-computando {N} waypoints '
            f'({total_m*1e3:.1f}mm @ {v_ref*1e3:.1f}mm/s) ...')

        msg = JointTrajectory()
        msg.joint_names = list(_ARM_JOINTS)
        q_iter = q.copy()
        T_iter = T0

        for k in range(1, N + 1):
            u = (k - 1) / max(1, N - 1)
            if constant:
                v_k = float(v_const_ms)
            else:
                v_k = float(v_min_ms) + (float(v_max_ms) - float(v_min_ms)) * (1.0 - u) ** 2
            v_k = max(1e-4, v_k)
            step = v_k * _CTRL_DT

            tw = np.zeros(6)
            tw[:3] = d * step
            if R0 is not None:
                R_err = R0 @ T_iter[:3, :3].T
                tw[3:] = _ORI_GAIN * 0.5 * np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ])
            if z0 is not None:
                tw[2] += _Z_CORR_GAIN * (z0 - float(T_iter[2, 3]))
            if perp_dir is not None and p0_perp is not None:
                perp_err = p0_perp - float(T_iter[:3, 3] @ perp_dir)
                tw[:3] += _Z_CORR_GAIN * perp_err * perp_dir

            J_k = jacobian(q_iter, T_end=T_TOUCH_TOOL_ATTACH)
            try:
                dq_k = J_k.T @ np.linalg.solve(J_k @ J_k.T + _JAC_LAM**2 * I6, tw)
            except np.linalg.LinAlgError:
                self.get_logger().warn(f'Batch: Jacobiano singular no passo {k} — truncando.')
                break

            q_next = np.clip(q_iter + dq_k, JOINT_MIN, JOINT_MAX)
            vel_k = np.clip((q_next - q_iter) / _CTRL_DT, -v_lim, v_lim)
            if k == N:
                vel_k = np.zeros(6)

            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q_next]
            pt.velocities = [float(x) for x in vel_k]
            t_k = k * _CTRL_DT
            pt.time_from_start = Duration(
                sec=int(t_k), nanosec=int((t_k - int(t_k)) * 1e9))
            msg.points.append(pt)
            q_iter = q_next
            T_iter = forward_kinematics(q_iter, T_end=T_TOUCH_TOOL_ATTACH)

        if not msg.points:
            return 'error'

        self._arm_traj_pub.publish(msg)
        self.get_logger().info(
            f'_cartesian_batch_to: {len(msg.points)} pts publicados '
            f'(duração {len(msg.points)*_CTRL_DT:.1f}s)')

        t_end = time.monotonic() + len(msg.points) * _CTRL_DT + 0.5
        while time.monotonic() < t_end:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self._settle()
                return 'stop'
            if self._pause_requested.is_set():
                # _pause_gate segura a posição, o que descarta o resto da
                # trajetória: devolve 'paused' para quem chama replanejar.
                if not self._pause_gate():
                    return 'stop'
                return 'paused'
            if self._force_stale_abort('TRANSIT'):
                self._settle()
                return 'stale'
            if self._force_over_limit():
                self._settle()          # substitui o goal: PARA no lugar
                self.get_logger().error(
                    f'SEGURANÇA [TRANSIT]: força {self._fz_corrected():+.1f} N '
                    f'além da margem de {_FORCE_SAFE_LIMIT_N:.0f} N durante um '
                    'trânsito em ar livre — algo está no caminho. Trajetória '
                    'interrompida no lugar.')
                return 'force'
            time.sleep(_CTRL_DT)
        return 'done'

    # Mão COVVI
    def _send_hand_pose(self, primary_rad: dict[str, float],
                         duration_s: float | None = None) -> None:
        if duration_s is None:
            # Escala inversa ao speed_factor_pct: 10 % → 2.0 s, 100 % → 0.2 s
            duration_s = max(0.3, 2.0 * (10.0 / max(1.0, self._speed_factor_pct)))
        names = list(_HAND_PRIMARY)
        positions = [float(primary_rad.get(j, 0.0)) for j in _HAND_PRIMARY]
        for mimic_name, driver, mult in _MIMIC_LIST:
            names.append(mimic_name)
            positions.append(float(primary_rad.get(driver, 0.0)) * mult)
        msg = JointTrajectory()
        msg.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        dur = max(0.1, float(duration_s))
        pt.time_from_start = Duration(
            sec=int(dur), nanosec=int((dur - int(dur)) * 1e9))
        msg.points.append(pt)
        self._hand_pub.publish(msg)

    # HOME: trajectória batch (uma mensagem multi-ponto → JTC planeia S-curve)
    def _joint_batch_to(self, q_target: np.ndarray) -> bool:
        """Leva as juntas a `q_target`, replanejando depois de cada pausa.

        Segurar posição durante a pausa substitui o goal do JTC, então o que
        faltava do percurso morre com ela. Como o alvo articular é fixo,
        retomar é só reemitir a partir de onde o braço parou — sem isso o ⏸
        no meio de um retorno à HOME deixava o braço a meio caminho e a fase
        seguinte começava de uma pose que ninguém escolheu.
        """
        for _ in range(self._LINEAR_REPLAN_MAX):
            out = self._joint_batch_once(q_target)
            if out != 'paused':
                return out == 'done'
        self.get_logger().error(
            f'[HOME] {self._LINEAR_REPLAN_MAX} pausas seguidas sem concluir '
            'o movimento articular — abortando.')
        return False

    def _joint_batch_once(self, q_target: np.ndarray) -> str:
        """Uma tentativa: envia uma única JointTrajectory com todos os
        waypoints ao JTC. Retorna 'done' | 'stop' | 'paused'."""
        q_from = self._q_now()
        delta = np.asarray(q_target, float) - q_from
        max_d = float(np.max(np.abs(delta)))
        if max_d < 0.001:
            return 'done'
        n_steps = max(2, int(math.ceil(max_d / (self._home_v_rad_s() * _CTRL_DT))))
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        vel_peak = np.clip(delta / n_steps / _CTRL_DT, -v_lim, v_lim)
        ramp = min(max(1, n_steps // 5), 8)

        msg = JointTrajectory()
        msg.joint_names = list(_ARM_JOINTS)
        for i in range(1, n_steps + 1):
            alpha = i / n_steps
            q = np.clip(q_from + alpha * delta, JOINT_MIN, JOINT_MAX)
            if i <= ramp:
                scale = i / ramp
            elif i >= n_steps - ramp + 1:
                scale = (n_steps - i + 1) / ramp
            else:
                scale = 1.0
            step_vel = vel_peak * scale if i < n_steps else np.zeros(6)
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.velocities = [float(v) for v in step_vel]
            t_s = i * _CTRL_DT
            pt.time_from_start = Duration(sec=int(t_s),
                                          nanosec=int((t_s - int(t_s)) * 1e9))
            msg.points.append(pt)

        self._arm_traj_pub.publish(msg)

        # Aguardar a execução, monitorizando stop e PAUSA a cada tick.
        t_end = time.monotonic() + n_steps * _CTRL_DT + 0.3
        while time.monotonic() < t_end:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self._settle()
                return 'stop'
            if self._pause_requested.is_set():
                if not self._pause_gate():
                    return 'stop'
                return 'paused'
            time.sleep(_CTRL_DT)
        return 'done'

    # Fases
    def _phase_goto_home(self) -> bool:
        """HOME — trajectória batch ao JTC (S-curve interna) a ≤ 0.3 rad/s."""
        self._set_phase('HOME')
        self._send_hand_pose(_HAND_POINTING_RAD)

        q_home = (self._user_home_q.copy()
                  if self._user_home_q is not None
                  else _POINTING_SEED_Q.copy())

        # Settle antes de mover — garante que não há lookahead residual.
        self._settle()

        if not self._joint_batch_to(q_home):
            return False

        # Settle final para estabilizar antes do CONTACT.
        self._settle(ticks=_SETTLE_TICKS * 3)

        # Verificação de orientação: o TCP deve estar apontando para baixo
        # (componente -Z da terceira coluna de R deve ser ≤ −0.7).
        q_actual = self._q_now()
        R_tcp = forward_kinematics(q_actual, T_end=T_TOUCH_TOOL_ATTACH)[:3, :3]
        tcp_z_world = R_tcp[:, 2]   # terceira coluna = eixo Z do TCP no frame mundo
        if tcp_z_world[2] > -0.5:
            self.get_logger().warn(
                f'HOME: TCP não está apontando para baixo '
                f'(tcp_z_world[2]={tcp_z_world[2]:.2f}, esperado < −0.5). '
                'Palpação continua mas a descida pode ser incorreta.')
        else:
            self.get_logger().info(
                f'HOME: orientação OK — tcp_z_world[2]={tcp_z_world[2]:.2f}')

        # NÃO sobrescrever _current_q: _cb_joints já mantém o valor correto
        # a partir de /joint_states.
        return True

    # ══════════════════════════════════════════════════════════════════
    # CALIBRAÇÃO DINÂMICA DO ÂNGULO DE ATAQUE
    # ══════════════════════════════════════════════════════════════════
    #
    # Por que existe: a HOME verifica que o TCP aponta para baixo e a
    # descida assume que o alvo está perpendicular a ela. Quando não está —
    # peça empenada, calço torto, fixação que cedeu — a ponteira encosta de
    # CANTO. As consequências são todas silenciosas: a célula lê a projeção
    # da força normal (mede menos do que aplica), o contato acontece numa
    # aresta em vez da face, e o SLIDING sai do plano no meio do curso.
    #
    # A calibração mede o plano em vez de supô-lo, e só depois ataca.
    # A ordem dos passos é imposta pela SEGURANÇA, não pela conveniência:
    #   1. registra o XYZ de referência no mundo (a pose de partida);
    #   2. N toques leves em torno dela (_align_offsets diz ONDE) — cada
    #      um é o MESMO
    #      _phase_descending() das outras fases, no setpoint de sonda, para
    #      que a penetração seja igual nos N pontos e o plano ajustado saia
    #      PARALELO ao real (é a igualdade que importa, não o valor);
    #   3. ajuste ortogonal do plano e desvio angular contra a vertical;
    #   4. RETRAÇÃO LINEAR, depois rotação do punho, depois retorno ao ponto
    #      de referência. Girar antes de retrair arrastaria a ponteira sobre
    #      a peça — o punho gira em torno do pulso, não da ponta, então a
    #      ponta varre um arco de raio ≈ comprimento da ferramenta.

    def _align_params(self) -> dict | None:
        """Configuração saturada da calibração, ou None se desligada.

        Fonte: os campos probe_align_* da PalpationStart quando a mensagem os
        trouxe (self._align_from_msg, gravado por _cb_start); senão os
        parâmetros ROS probe_align_*, que é o caminho de quem roda sem GUI.
        Mesma precedência de _force_profile.

        Os tetos aqui não são estética de validação: `points` abaixo de 3 não
        define plano, e `tilt_max_deg` acima de _ALIGN_TILT_HARD_MAX_DEG
        autorizaria uma rotação de punho que a ponteira não sobrevive. Um
        valor fora da faixa é SATURADO, não recusado — a calibração é
        opcional e não deve derrubar o experimento por um typo.
        """
        with self._params_lock:
            from_msg = self._align_from_msg
            on = self._align_on
            raw = dict(self._align_msg)
        try:
            if from_msg:
                if not on:
                    return None
                src = 'GUI'
            else:
                if not bool(self.get_parameter('probe_align_enable').value):
                    return None
                src = 'param'
                raw = {
                    'points': int(
                        self.get_parameter('probe_align_points').value),
                    'radius_mm': float(
                        self.get_parameter('probe_align_radius_mm').value),
                    'force_n': float(
                        self.get_parameter('probe_align_force_n').value),
                    'retract_mm': float(
                        self.get_parameter('probe_align_retract_mm').value),
                    'tilt_max_deg': float(
                        self.get_parameter('probe_align_tilt_max_deg').value),
                }
        except Exception as exc:
            self.get_logger().warn(
                f'[ALIGN] configuração ilegível ({exc}) — calibração '
                'desligada; a descida segue na vertical.')
            return None

        def _or_default(key: str, default: float) -> float:
            """0/ausente = usar o default, como nos demais campos numéricos
            da PalpationStart."""
            try:
                v = float(raw.get(key) or 0.0)
            except (TypeError, ValueError):
                v = 0.0
            return v if v > 0.0 else float(default)

        return {
            'src': src,
            'n': int(np.clip(int(_or_default('points', _ALIGN_POINTS_DEFAULT)),
                             _ALIGN_MIN_POINTS, _ALIGN_MAX_POINTS)),
            'radius_m': float(np.clip(
                _or_default('radius_mm', _ALIGN_RADIUS_MM),
                _ALIGN_RADIUS_MM_MIN, _ALIGN_RADIUS_MM_MAX)) * 1e-3,
            'force_n': float(np.clip(
                _or_default('force_n', _ALIGN_PROBE_FORCE_N),
                3.0 * _CONTACT_ON_N, _FORCE_SETPOINT_MAX_N)),
            'retract_m': float(np.clip(
                _or_default('retract_mm', _ALIGN_RETRACT_MM),
                _ALIGN_RETRACT_MM_MIN, _ALIGN_RETRACT_MM_MAX)) * 1e-3,
            'tilt_max_deg': float(np.clip(
                _or_default('tilt_max_deg', _ALIGN_TILT_MAX_DEG),
                1.0, _ALIGN_TILT_HARD_MAX_DEG)),
        }

    def _align_offsets(self, cfg: dict) -> tuple[np.ndarray, str, bool]:
        """Onde a sonda vai encostar (offsets XY relativos à referência) e a
        frase que descreve isso no log.

        No MATRIX_MAP a geometria já foi desenhada pelo usuário: o anel meio
        passo FORA dos cantos da grade mede o plano da região que o ensaio
        realmente percorre, sem tocar nenhum ponto que será identado depois.
        O raio da GUI é um número solto ao lado da grade — capaz de mandar
        os toques para fora da amostra numa grade pequena, e de medir menos
        inclinação do que existe numa grande.

        Fora do MATRIX_MAP não há grade, e o polígono do raio é a única
        geometria possível. Grade curta demais também cai nele: um braço de
        alavanca menor que o piso do próprio raio deixaria a inclinação
        enterrada no ruído dos toques.

        O terceiro elemento diz se o PRIMEIRO offset é o centro (0, 0). Ele
        vem no polígono e não no anel da grade: no polígono o centro é o
        próprio alvo do ensaio — que a descida de trabalho vai identar logo
        depois de qualquer jeito — enquanto no MATRIX o centro do anel cai
        no meio da grade, possivelmente sobre um ponto que ainda será medido,
        e pré-condicioná-lo tiraria esse ponto da comparação. A curvatura no
        MATRIX aparece sozinha: a penetração de CADA waypoint contra o plano
        da origem já vai para o matrix.csv.
        """
        with self._params_lock:
            wps = self._matrix_wps.copy()
        if self._mode == 'MATRIX_MAP' and len(wps):
            # A origem (0, 0) não vai na lista de waypoints, mas é um nó da
            # grade e entra no retângulo que o anel envolve.
            nodes = np.vstack([np.zeros((1, 2)), wps])
            try:
                ring = _probe_ring_from_grid(
                    nodes, min_half_extent_m=_ALIGN_RADIUS_MM_MIN * 1e-3)
            except ValueError as exc:
                self.get_logger().warn(
                    f'[ALIGN] a grade não serve de padrão de sondagem '
                    f'({exc}) — sondando o polígono da GUI.')
            else:
                lo, hi = ring.min(axis=0), ring.max(axis=0)
                return ring, (
                    f'{len(ring)} toques nos cantos da grade, meio passo '
                    f'para fora ({(hi[0]-lo[0])*1e3:.1f} × '
                    f'{(hi[1]-lo[1])*1e3:.1f} mm)'), False
        return (_probe_pattern(cfg['n'], cfg['radius_m'], with_center=True),
                f'1 toque no CENTRO (o alvo) + {cfg["n"]} num círculo de '
                f'{cfg["radius_m"]*1e3:.1f} mm de raio em torno dele',
                True)

    def _probe_plane(self, p_ref: np.ndarray, offsets: np.ndarray,
                     cfg: dict) -> tuple[str, list]:
        """Executa os toques de sonda e devolve ('ok', pontos de contato).

        Todo o trânsito acontece na ALTURA DE REFERÊNCIA — a pose em que o
        usuário deixou a sonda. É a única altura comprovadamente livre sobre
        a peça (foi de lá que a aproximação partiu), e usá-la dispensa
        inventar um Safe Z para uma superfície cuja inclinação é justamente
        o que ainda não se conhece.

        Em falha devolve o outcome da fase que falhou junto com os pontos já
        colhidos — quem chama aborta, mas o log fica com o que se mediu.
        """
        pts: list[np.ndarray] = []
        n_total = len(offsets)
        # Onde o estágio rápido das descidas seguintes tem de largar a
        # velocidade: a inclinação máxima admitida faz o contato variar
        # ±braço·tan(θ) ao longo do padrão, e o rastejo precisa começar
        # acima do ponto mais ALTO que a peça pode ter. O braço sai dos
        # PRÓPRIOS offsets (o toque mais distante da referência) — o anel da
        # grade não é centrado nela, então o raio da config não o descreve.
        lever_m = float(np.max(np.linalg.norm(
            np.asarray(offsets, dtype=float), axis=1)))
        margin_m = (lever_m
                    * math.tan(math.radians(cfg['tilt_max_deg']))
                    + _ALIGN_ZONE_EXTRA_M)

        for k, off in enumerate(offsets, start=1):
            self._set_phase('CALIBRATING')
            target = np.array([p_ref[0] + float(off[0]),
                               p_ref[1] + float(off[1]),
                               p_ref[2]])
            out = self._move_linear_world(
                target - self._tcp_now(), _ALIGN_TRANSIT_MS, lock_z=True,
                label=f'ALIGN-XY{k}')
            if out != 'done':
                return out, pts

            out = self._phase_descending()
            if out != 'ok':
                return out, pts

            p_hit = self._tcp_now()
            pts.append(p_hit)
            self.get_logger().info(
                f'[ALIGN] toque {k}/{n_total}: contato em '
                f'x={p_hit[0]*1e3:+.2f} y={p_hit[1]*1e3:+.2f} '
                f'z={p_hit[2]*1e3:+.2f} mm '
                f'({(p_ref[2] - p_hit[2])*1e3:.1f} mm sob a referência).')

            # Volta à altura de referência antes do próximo trânsito.
            self._set_phase('CALIBRATING')
            out = self._move_linear_world(
                np.array([0.0, 0.0, float(p_ref[2] - p_hit[2])]),
                _ALIGN_TRANSIT_MS, label=f'ALIGN-UP{k}')
            if out != 'done':
                return out, pts

            # Descidas seguintes partem da MESMA altura, então o curso até o
            # contato já é conhecido: rápido até a margem acima do contato
            # mais RASO já visto, rastejo no resto. Mesmo critério do
            # MATRIX_MAP (ver _MATRIX_RELIEF_MARGIN_M), com a margem vinda da
            # geometria em vez de um relevo suposto.
            fast_m = min(float(p_ref[2] - p[2]) for p in pts) - margin_m
            with self._params_lock:
                self._learned_contact_m = fast_m if fast_m > 0.0 else None
                # Estimativa vinda da GEOMETRIA, não de contatos medidos a
                # partir desta altura: a janela recomeça e a margem volta ao
                # palpite conservador até a peça se provar plana de novo.
                self._contact_depths.clear()

        return 'ok', pts

    def _attack_axis_err_deg(self, attack_dir: np.ndarray) -> float:
        """Desvio ENTREGUE: ângulo entre o eixo Z do TCP calculado da pose
        MEDIDA em /joint_states e o eixo de ataque pedido."""
        z_tcp = forward_kinematics(
            self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)[:3, 2]
        return _angle_between_deg(z_tcp, attack_dir)

    def _verify_attack_axis(self, attack_dir: np.ndarray, label: str) -> bool:
        """Confere no BRAÇO que o eixo de ataque é o que se pediu.

        Tudo o que vinha antes disto é intenção: a IK resolve, a FK da
        SOLUÇÃO confirma que a solução aponta certo, e o comando é
        publicado. Nada media o que o braço fez com ele. Entre o comando e a
        pose real cabem um limite de junta alcançado na execução, um JTC que
        não fechou o último grau, o lock de orientação (proporcional, ver
        _ORI_GAIN) cedendo durante um trânsito linear, e o braço real
        atrasado em relação ao simulado.

        Falhar em silêncio aqui é o pior desfecho possível da fase: a
        descida, a regulação de força e o alívio de emergência passam todos
        a correr sobre `_attack_dir`, um eixo que a ferramenta não tem. O
        contato sai de canto, a célula lê a PROJEÇÃO da força normal — mede
        menos do que aplica — e nada no log denuncia.

        Roda sobre /joint_states, sem mover nada e sem tocar a amostra.
        """
        err_deg = self._attack_axis_err_deg(attack_dir)
        if err_deg > _ALIGN_ORI_TOL_DEG:
            self.get_logger().error(
                f'[{label}] eixo de ataque NÃO confirmado no braço: a pose '
                f'medida erra o eixo pedido em {err_deg:.2f}° (tolerância '
                f'{_ALIGN_ORI_TOL_DEG:.1f}°). O comando foi aceito mas a '
                'pose entregue é outra — limite de junta na execução, JTC '
                'que não fechou, ou o braço real atrasado em relação ao '
                'simulado. Abortando: descer sobre um eixo que a ferramenta '
                'não tem faz a célula ler a projeção da força normal, sem '
                'nenhum sintoma no log.')
            return False
        self.get_logger().info(
            f'[{label}] eixo de ataque CONFIRMADO na pose medida — erro '
            f'{err_deg:.2f}° (tolerância {_ALIGN_ORI_TOL_DEG:.1f}°).')
        return True

    def _rotate_to_attack(self, attack_dir: np.ndarray, *,
                          label: str = 'ALIGN') -> str:
        """Gira o punho até o eixo Z do TCP coincidir com `attack_dir`,
        MANTENDO a posição do TCP.

        NÃO retrai: quem chama é responsável por garantir o espaço livre —
        a ponta descreve um arco de raio ≈ comprimento da ferramenta durante
        a rotação (67,7 mm até a face da ponteira, ver
        T_TOUCH_TOOL_ATTACH).

        Retorna 'ok' | 'stop' | 'error'.
        """
        p_now = self._tcp_now()
        q_target, ik_ok = inverse_kinematics(
            p_now, attack_dir, q_seed=self._q_now(),
            T_end=T_TOUCH_TOOL_ATTACH)
        if not ik_ok:
            self.get_logger().error(
                f'[{label}] a IK não convergiu para o eixo de ataque pedido '
                '— abortando sem girar o punho.')
            return 'error'
        # A IK SATURA nos limites articulares em vez de falhar, então
        # convergência não é o mesmo que apontar para onde se pediu:
        # confere a SOLUÇÃO antes de mandar o braço para ela.
        z_tcp = forward_kinematics(q_target, T_end=T_TOUCH_TOOL_ATTACH)[:3, 2]
        err_deg = _angle_between_deg(z_tcp, attack_dir)
        if err_deg > _ALIGN_ORI_TOL_DEG:
            self.get_logger().error(
                f'[{label}] a solução da IK erra o eixo de ataque em '
                f'{err_deg:.2f}° (tolerância {_ALIGN_ORI_TOL_DEG:.1f}°) — '
                'provável limite de junta nesta pose. Abortando sem girar '
                'o punho.')
            return 'error'
        if not self._joint_stream_to(q_target):
            return 'stop'
        self._settle(ticks=_SETTLE_TICKS * 2)
        # E agora a pose ENTREGUE, que é outra coisa: a checagem acima olhou
        # a solução da IK, não o que o braço fez com ela.
        if not self._verify_attack_axis(attack_dir, label):
            return 'error'
        return 'ok'

    def _reapply_attack_orientation(self) -> bool:
        """Reaplica o ângulo de ataque calibrado depois de uma ida à HOME.

        A HOME é uma pose ARTICULAR completa: levar o braço até ela desfaz a
        rotação do punho e devolve a ferramenta à vertical. Como
        `_attack_dir` sobrevive ao retorno, sem esta reaplicação o ciclo
        seguinte desceria na DIAGONAL com a ponteira apontando para baixo —
        pior que não calibrar, porque soma o contato de canto ao arrasto
        lateral da translação diagonal.

        Roda em ar livre na própria home, longe da peça, então dispensa a
        retração que a calibração faz. True = pode seguir.
        """
        attack = self._attack_dir
        if attack is None:
            return True
        out = self._rotate_to_attack(attack, label='ALIGN-REAPPLY')
        if out != 'ok':
            self.get_logger().error(
                '[ALIGN] não foi possível reaplicar o ângulo de ataque após '
                'a HOME — o ciclo desceria desalinhado. Abortando.')
            return False
        self.get_logger().info(
            f'[ALIGN] ângulo de ataque reaplicado após a HOME — eixo '
            f'({attack[0]:+.4f}, {attack[1]:+.4f}, {attack[2]:+.4f}).')
        return True

    def _align_reorient(self, attack_dir: np.ndarray, retract_m: float,
                        p_ref: np.ndarray) -> str:
        """Retração linear → rotação do punho → retorno ao ponto de
        referência, já com o eixo de ataque alinhado à normal medida.

        Retorna 'ok' | 'stop' | 'force' | 'stale' | 'error'.
        """
        # 1. RETRAÇÃO. O punho gira em torno do pulso; a ponta descreve um
        # arco de raio ≈ comprimento da ferramenta (67,7 mm até a face da
        # ponteira, ver T_TOUCH_TOOL_ATTACH). Sem afastar antes, esse arco
        # passa DENTRO da peça e cisalha a ponteira.
        out = self._move_linear_world(
            np.array([0.0, 0.0, float(retract_m)]), _ALIGN_TRANSIT_MS,
            label='ALIGN-RETRACT')
        if out != 'done':
            return out

        # 2. ROTAÇÃO em ar livre, mantendo a posição do TCP.
        out = self._rotate_to_attack(attack_dir, label='ALIGN')
        if out != 'ok':
            return out

        # 3. RETORNO ao ponto de referência. Um único movimento linear
        # devolve o TCP ao XYZ de partida (a rotação o deixou deslocado), de
        # modo que o curso da descida de trabalho continue sendo o mesmo
        # `depth_mm` que o usuário pediu — só que agora ao longo da normal.
        out = self._move_linear_world(
            p_ref - self._tcp_now(), _ALIGN_TRANSIT_MS,
            label='ALIGN-RETURN')
        if out != 'done':
            return out
        # O lock de orientação do trânsito é PROPORCIONAL (_ORI_GAIN): ele
        # persegue R0, não a garante. Uma deriva acumulada no percurso sai
        # daqui direto para a descida de trabalho — este é o último ponto
        # antes disso em que ainda dá para recusar.
        return 'ok' if self._verify_attack_axis(
            attack_dir, 'ALIGN-RETURN') else 'error'

    def _phase_calibrate_attack(self) -> str:
        """CALIBRATING — palpação espacial do alvo e correção do ataque.

        Retorna 'ok' (eixo corrigido, ou calibração desligada) | 'stop' |
                'force' | 'stale' | 'no_contact' | 'error'.
        """
        cfg = self._align_params()
        if cfg is None:
            return 'ok'

        # A sondagem tem de acontecer na VERTICAL: é ela que mede a
        # inclinação, então não pode partir de uma correção anterior.
        self._attack_dir = None

        self._set_phase('CALIBRATING')
        self._settle()
        p_ref = self._tcp_now()
        offsets, padrao, tem_centro = self._align_offsets(cfg)
        with self._params_lock:
            # A sonda nunca é mais pesada que o próprio ensaio: com setpoint
            # de 0,5 N, sondar a 1 N marcaria a amostra antes da medição.
            probe_f = min(cfg['force_n'], float(self._target_force_n))
            saved = (self._home_key_cur, self._home_deg_cur,
                     self._learned_contact_m, self._target_force_n,
                     self._contact_depths.copy())
            # A profundidade aprendida é indexada POR HOME e mede o curso da
            # home até o contato. Os toques de sonda partem de outros XY, na
            # altura de referência — nem consomem nem alimentam aquele
            # histórico, senão a home ficaria "aprendida" com um número que
            # não veio dela.
            self._home_key_cur = None
            self._home_deg_cur = None
            self._learned_contact_m = None
            self._contact_depths.clear()
            self._target_force_n = probe_f

        self.get_logger().info(
            f'[ALIGN] ({cfg["src"]}) referência do mundo em '
            f'x={p_ref[0]*1e3:+.2f} y={p_ref[1]*1e3:+.2f} '
            f'z={p_ref[2]*1e3:+.2f} mm — {padrao}, a '
            f'{probe_f:.2f} N cada (desvio máximo aceito '
            f'{cfg["tilt_max_deg"]:.1f}°).')
        try:
            out, pts = self._probe_plane(p_ref, offsets, cfg)
        finally:
            with self._params_lock:
                (self._home_key_cur, self._home_deg_cur,
                 self._learned_contact_m, self._target_force_n,
                 self._contact_depths) = saved
        if out != 'ok':
            self.get_logger().error(
                f'[ALIGN] palpação interrompida em {len(pts)}/{len(offsets)} '
                f'pontos ({out}) — sem plano, sem correção de ataque.')
            return out

        # O CENTRO não entra no ajuste da normal — está no centroide e não
        # acrescenta braço de alavanca. Ele serve para a única coisa que o
        # anel não enxerga: CURVATURA. Ver _probe_pattern/sagitta_m.
        p_centro = np.asarray(pts[0], float) if tem_centro else None
        pts_anel = pts[1:] if tem_centro else pts
        try:
            fit = _fit_plane(np.asarray(pts_anel, dtype=float))
        except ValueError as exc:
            self.get_logger().error(
                f'[ALIGN] ajuste do plano falhou: {exc}.')
            return 'error'

        # ── Curvatura: o centro contra o plano do anel ───────────────
        if p_centro is not None:
            sag = _sagitta_m(fit, p_centro)
            self.get_logger().info(
                f'[ALIGN] toque central {sag * 1e3:+.3f} mm '
                f'{"ACIMA" if sag > 0 else "abaixo"} do plano do anel '
                f'(teto {_SAGITTA_MAX_M * 1e3:.3f} mm) — é a medida de '
                'curvatura, e o anel sozinho é cego para ela.')
            if abs(sag) > _SAGITTA_MAX_M:
                self.get_logger().error(
                    f'[ALIGN] calibração RECUSADA: o centro está a '
                    f'{sag * 1e3:+.3f} mm do plano ajustado pelos '
                    f'{fit.n_points} toques do anel, acima do teto de '
                    f'{_SAGITTA_MAX_M * 1e3:.3f} mm. A superfície é CURVA, '
                    'não inclinada, e uma calota não tem uma normal — tem '
                    'uma por ponto. Alinhar o ataque a um plano ajustado '
                    'aqui não significa nada: a ponteira encostaria no '
                    'ápice de qualquer jeito, que é o contato de canto que '
                    'esta fase existe para evitar. Repare que o ANEL passou '
                    f'em tudo (desvio {fit.tilt_deg:.3f}°, resíduo '
                    f'{fit.rms_m*1e3:.3f} mm) — numa calota simétrica todos '
                    'os pontos dele caem na mesma altura. Reduza o raio de '
                    'sondagem para uma região que seja plana, ou calce a '
                    'peça.')
                return 'error'

        # O desvio sozinho não diz se o alvo está reto: ele só é conhecido
        # até onde a sondagem o resolve (ver PlaneFit.tilt_unc_deg).
        unc = fit.tilt_unc_deg
        unc_txt = (f'± {unc:.2f}°' if math.isfinite(unc)
                   else '± INDETERMINADO')
        self.get_logger().info(
            f'[ALIGN] plano ajustado com {fit.n_points} pontos por '
            f'{"mínimos quadrados" if fit.least_squares else "solução exata"}'
            f': normal=({fit.normal[0]:+.4f}, {fit.normal[1]:+.4f}, '
            f'{fit.normal[2]:+.4f})  desvio da vertical={fit.tilt_deg:.2f}° '
            f'{unc_txt}  resíduo RMS={fit.rms_m*1e3:.3f} mm  '
            f'braço={fit.lever_m*1e3:.1f} mm  '
            f'espalhamento={fit.spread:.2f}.')
        if not math.isfinite(unc):
            self.get_logger().warn(
                f'[ALIGN] com {fit.n_points} pontos o plano passa EXATO por '
                'eles e o resíduo sai estruturalmente zero: o desvio medido '
                'não tem barra de erro. O ajuste vale, mas ele não certifica '
                f'perpendicularidade nenhuma. Use {_ALIGN_POINTS_DEFAULT}+ '
                'toques para que o resíduo passe a medir alguma coisa.')
        elif unc > _ALIGN_ORI_TOL_DEG:
            self.get_logger().warn(
                f'[ALIGN] a sondagem resolve o desvio só até ±{unc:.2f}°, '
                f'PIOR que a tolerância de {_ALIGN_ORI_TOL_DEG:.1f}° com que '
                'ela decide girar ou não o punho: dentro dessa barra o alvo '
                'pode estar reto ou torto e a medida não separa os dois '
                'casos. Compra-se resolução aumentando o braço (raio de '
                'sondagem) ou reduzindo o resíduo — ela cai com σ/braço.')

        ok, motivo = _validate_fit(fit, tilt_max_deg=cfg['tilt_max_deg'])
        if not ok:
            self.get_logger().error(
                f'[ALIGN] calibração RECUSADA: {motivo}. Atacar por um plano '
                'que a medição não sustenta é pior que atacar na vertical — '
                'abortando o experimento antes de girar o punho.')
            return 'error'

        # O plano medido vale para o SLIDING SEMPRE que o ajuste passou —
        # inclusive quando o desvio não paga uma rotação de punho. A escala
        # das duas decisões é diferente: 1,9° não justificam girar o punho,
        # mas esgotam a reserva de indentação (alvo/K, dezenas de µm contra
        # contato rígido) em menos de 2 mm de curso lateral. Medir e depois
        # deslizar na horizontal seria jogar fora a medição.
        self._align_set_slide_plane(fit)

        attack = _attack_dir_from_normal(fit.normal)
        if fit.tilt_deg <= _ALIGN_ORI_TOL_DEG:
            # Já alinhado dentro do que a própria IK entrega: girar o punho
            # por menos que isso é movimento (e risco) sem ganho.
            self.get_logger().info(
                f'[ALIGN] desvio de {fit.tilt_deg:.2f}° {unc_txt} dentro da '
                f'tolerância de {_ALIGN_ORI_TOL_DEG:.1f}° — o alvo já está '
                'perpendicular à home; ataque mantido na vertical (o plano '
                'medido segue valendo para o deslize). O ataque entra na '
                f'descida com ERRO RESIDUAL de {fit.tilt_deg:.2f}° contra a '
                'face: é o preço aceito por não girar o punho.')
            # Aqui nada foi comandado, então não há "pose entregue" a checar
            # — mas a descida vai supor a vertical do mundo, e quem entrega
            # essa vertical é a HOME. Vale medir e dizer, sem abortar: uma
            # home fora de esquadro é assunto de _phase_goto_home.
            vert_err = self._attack_axis_err_deg(np.array([0.0, 0.0, -1.0]))
            if vert_err > _ALIGN_ORI_TOL_DEG:
                self.get_logger().warn(
                    f'[ALIGN] a ferramenta está {vert_err:.2f}° fora da '
                    'vertical do mundo nesta pose, e é nessa vertical que a '
                    'descida vai correr. O erro contra a face é a SOMA deste '
                    f'com o desvio do plano ({fit.tilt_deg:.2f}°), não o '
                    'desvio sozinho — revise a home antes de confiar no '
                    'valor de força.')
            self._set_phase('CALIBRATING')
            out = self._move_linear_world(
                p_ref - self._tcp_now(), _ALIGN_TRANSIT_MS,
                label='ALIGN-RETURN')
            return 'ok' if out == 'done' else out

        self._set_phase('CALIBRATING')
        out = self._align_reorient(attack, cfg['retract_m'], p_ref)
        if out != 'ok':
            return out

        self._attack_dir = attack
        # O contato aprendido é o CURSO até tocar, medido ao longo do eixo de
        # aproximação, e está indexado por posição do TCP na home — não pelo
        # eixo. Inclinar o ataque alonga esse curso por 1/cos(desvio) (6,4 %
        # a 20°: 1,9 mm num contato de 30 mm), o bastante para o estágio
        # rápido invadir o contato aprendido quando a zona lenta está no piso
        # de 1,5 mm. Soltar a chave da home descarta o histórico VERTICAL sem
        # apagá-lo do disco (ele continua correto para runs sem calibração);
        # o aprendizado DENTRO deste run segue vivo em _learned_contact_m,
        # que _remember_contact preenche mesmo sem chave.
        with self._params_lock:
            self._home_key_cur = None
            self._home_deg_cur = None
            self._learned_contact_m = None
            # Os cursos da janela foram medidos ao longo do eixo ANTIGO; o
            # novo os alonga por 1/cos(desvio), o que viraria dispersão
            # falsa. Recomeça com o palpite conservador.
            self._contact_depths.clear()
        self.get_logger().info(
            f'[ALIGN] ataque corrigido em {fit.tilt_deg:.2f}° {unc_txt} — '
            f'eixo ({attack[0]:+.4f}, {attack[1]:+.4f}, {attack[2]:+.4f}), '
            f'CONFERIDO na pose medida com erro '
            f'{self._attack_axis_err_deg(attack):.2f}°. Perpendicularidade '
            f'garantida até a soma dos dois: o erro da medição do plano '
            f'({unc_txt.replace("± ", "")}) mais o da pose entregue. '
            'A descida de trabalho, a regulação de força e o alívio de '
            'emergência passam a correr sobre este eixo.')
        return self._verify_attack_plane(p_ref, offsets, cfg)

    def _verify_attack_plane(self, p_ref: np.ndarray, offsets: np.ndarray,
                             cfg: dict) -> str:
        """CONFERÊNCIA de ponta a ponta: re-sonda o plano JÁ com o ataque
        corrigido e mede o ângulo que sobrou entre a ferramenta e a face.

        Tudo o que roda antes disto confere um ELO: a solução da IK aponta
        certo, a pose entregue é a comandada, o ajuste do plano fecha dentro
        do resíduo. Nenhum deles confere o PRODUTO — que a ferramenta esteja
        perpendicular à peça — porque todos partem da mesma primeira medição
        e herdam o erro dela. Um calço que cedeu entre a sondagem e a
        rotação, uma amostra que escorregou, um offset de ferramenta errado
        no T_TOUCH_TOOL_ATTACH: nada disso aparece nos elos, e todos aparecem
        aqui.

        A conta é direta: os novos contatos são colhidos DESCENDO sobre o
        eixo corrigido, então o ângulo entre −_attack_dir e a normal do
        segundo ajuste é o desvio residual do ataque. Zero = perpendicular.

        Custa outra rodada de toques, por isso é opt-in
        (`probe_align_verify`). Devolve 'ok' quando desligada — o contrato
        de quem chama não muda.
        """
        try:
            if not bool(self.get_parameter('probe_align_verify').value):
                return 'ok'
        except Exception:
            return 'ok'

        attack = self._attack_dir
        if attack is None:                 # não girou: nada a conferir
            return 'ok'

        self.get_logger().info(
            '[ALIGN-VERIFY] re-sondando o plano com o ataque JÁ corrigido — '
            'é a única medição que enxerga o ângulo ENTRE a ferramenta e a '
            'face, em vez de conferir os elos que levaram até ele.')
        self._set_phase('CALIBRATING')
        with self._params_lock:
            saved = (self._home_key_cur, self._home_deg_cur,
                     self._learned_contact_m, self._target_force_n,
                     self._contact_depths.copy())
            # Mesmo isolamento da primeira sondagem: estes toques partem de
            # outros XY e não pertencem ao histórico de nenhuma home.
            self._home_key_cur = None
            self._home_deg_cur = None
            self._learned_contact_m = None
            self._contact_depths.clear()
            self._target_force_n = min(cfg['force_n'],
                                       float(self._target_force_n))
        try:
            out, pts = self._probe_plane(p_ref, offsets, cfg)
        finally:
            with self._params_lock:
                (self._home_key_cur, self._home_deg_cur,
                 self._learned_contact_m, self._target_force_n,
                 self._contact_depths) = saved
        if out != 'ok':
            self.get_logger().error(
                f'[ALIGN-VERIFY] a conferência parou em {len(pts)}/'
                f'{len(offsets)} pontos ({out}) — sem conferência não há '
                'garantia de perpendicularidade, e o ensaio foi pedido COM '
                'ela. Abortando.')
            return out

        try:
            fit2 = _fit_plane(np.asarray(pts, dtype=float))
        except ValueError as exc:
            self.get_logger().error(
                f'[ALIGN-VERIFY] ajuste da conferência falhou: {exc}.')
            return 'error'

        # Desvio RESIDUAL: ângulo entre o eixo que a ferramenta está usando
        # e a normal recém-medida da face. É o número que o ensaio queria.
        resid_deg = _angle_between_deg(-np.asarray(attack, float), fit2.normal)
        unc2 = fit2.tilt_unc_deg
        unc2_txt = (f'± {unc2:.2f}°' if math.isfinite(unc2)
                    else '± INDETERMINADO')
        if resid_deg > _ALIGN_ORI_TOL_DEG:
            self.get_logger().error(
                f'[ALIGN-VERIFY] REPROVADO: com o ataque corrigido a face '
                f'ainda faz {resid_deg:.2f}° {unc2_txt} com a ferramenta '
                f'(tolerância {_ALIGN_ORI_TOL_DEG:.1f}°). A primeira '
                'sondagem e a rotação fecharam cada uma por si, mas o '
                'produto não é perpendicular — peça que se moveu entre as '
                'duas medições, calço que cedeu sob o toque, ou offset de '
                'ferramenta errado. Abortando: a célula leria a projeção da '
                'força normal, medindo menos do que aplica.')
            return 'error'
        self.get_logger().info(
            f'[ALIGN-VERIFY] APROVADO: ataque perpendicular à face dentro de '
            f'{resid_deg:.2f}° {unc2_txt} (tolerância '
            f'{_ALIGN_ORI_TOL_DEG:.1f}°), medido por {fit2.n_points} contatos '
            f'novos com resíduo RMS de {fit2.rms_m*1e3:.3f} mm. Este é o '
            'ângulo ENTRE ferramenta e peça, não a soma dos elos.')
        return 'ok'

    def _align_set_slide_plane(self, fit) -> None:
        """Entrega ao SLIDING o plano MEDIDO da amostra.

        O deslize percorre uma reta contida no plano (ver o bloco "Plano do
        deslize" nas constantes). Sem esta entrega ele cairia na inclinação
        DECLARADA na GUI, e o alinhamento conquistado na descida se perderia
        no primeiro milímetro de curso.

        A declaração do usuário NÃO é sobrescrita: ela continua em
        `_slide_slope_deg` e volta a valer se a calibração não rodar. Só o
        modo SLIDE consome isto.
        """
        with self._params_lock:
            if self._mode != 'SLIDE':
                return
            dir_xy = self._slide_dir_vec.copy()
            declarado = float(self._slide_slope_deg)
            self._slide_plane_n = np.asarray(fit.normal, dtype=float).copy()
        try:
            medido = _slope_along_deg(fit.normal, dir_xy)
        except ValueError:
            # O plano existe e já foi guardado; só a leitura em GRAUS ao
            # longo do curso é que não faz sentido (plano de pé). O deslize
            # usa o vetor, não este número — que é só para o log.
            self.get_logger().info(
                '[ALIGN] plano do deslize definido pela normal medida '
                '(inclinação ao longo do curso indefinida).')
            return
        self.get_logger().info(
            f'[ALIGN] plano do deslize definido pela normal medida: o curso '
            f'segue uma reta a {medido:+.2f}° na direção pedida (o valor '
            f'declarado na GUI, {declarado:+.2f}°, fica de reserva). O curso '
            'pedido passa a ser distância sobre a SUPERFÍCIE.')

    def _calibrate_or_abort(self) -> bool:
        """Roda a calibração e trata o desfecho no padrão das outras fases.

        Devolve False quando o experimento JÁ FOI encerrado (a fase final
        está marcada) e quem chamou deve apenas retornar.
        """
        out = self._phase_calibrate_attack()
        if out == 'ok':
            return True
        if out == 'stop':   # STOP → HOME (Regra de Ouro) · FREEZE → congela
            self._finalize_interrupt('ABORTED')
        else:
            self._abort_to_home()
        return False

    def _phase_descending(self) -> str:
        """DESCENDING — desce ao longo do approach até a força atingir o setpoint.

        Retorna: 'ok' (setpoint atingido) | 'no_contact' (curso esgotado)
                 | 'force' (> 15 N) | 'stale' (célula sem dados frescos)
                 | 'stop' (usuário).
        """
        self._set_phase('DESCENDING')
        # Parada VERIFICADA, não os 180 ms fixos do _settle: a descida comanda
        # só Z, então qualquer velocidade residual do HOME sai como movimento
        # lateral do TCP (ver _SETTLE_STILL_TOL_RAD_S). Com o braço já parado
        # — o caso normal — isto sai em 3 ticks e não muda nada.
        if not self._settle_until_still():
            self.get_logger().warn(
                f'[DESCENDING] o braço ainda não parou depois de '
                f'{_SETTLE_STILL_MAX_TICKS * _CTRL_DT:.1f} s travado: alguma '
                f'junta segue acima de {_SETTLE_STILL_TOL_RAD_S:.4f} rad/s. '
                'Descendo assim mesmo, mas os primeiros passos vão sair fora '
                'da vertical — a velocidade residual soma ao Z comandado. '
                'Se repetir, o suspeito é o HOME não convergir (checar '
                'home_speed_rad_s e o overshoot das juntas do punho).')

        # Aproximação ao longo do EIXO DE ATAQUE: a vertical do mundo (para
        # baixo) quando a calibração está desligada ou ainda não rodou, e a
        # normal MEDIDA do alvo depois que _phase_calibrate_attack a
        # determinou. Todo o resto da fase — regulação quase-estática, alívio
        # de emergência, HOLD — já trabalha sobre este vetor, então a
        # correção de ângulo não pede caminho de controle próprio.
        approach_dir = (self._attack_dir.copy()
                        if self._attack_dir is not None
                        else np.array([0.0, 0.0, -1.0]))
        self._approach_dir = approach_dir.copy()

        with self._params_lock:
            depth_m      = float(self._target_depth_mm) / 1000.0
            target_f     = float(self._target_force_n)
            approach_mms = float(self.get_parameter('approach_v_max_mms').value)
            # O slider "Descent Speed" é mm/s ABSOLUTOS, igual ao caminho
            # MovL (ver o contrato em _cb_start).
            v_fast_ms = max(0.001, approach_mms / 1000.0)
            # Re-resolve COM a direção de approach conhecida — o único ponto
            # onde isso acontece, para os dois caminhos (streaming e MovL).
            # Em _cb_start a direção ainda não existe, e sem ela
            # _lookup_learned trata TODO o offset de uma home vizinha como
            # axial. O MovL corrigia isso por conta própria e o streaming não:
            # os dois desciam com perfis diferentes a partir dos mesmos dados.
            learned_m, _ = self._lookup_learned(self._home_key_cur, approach_dir)
            if learned_m is None:
                learned_m = self._learned_contact_m
            else:
                self._learned_contact_m = learned_m

        # Perfil de velocidade em ar livre (dois estágios):
        #   COM profundidade aprendida — v_fast (GUI) até a margem antes do
        #   ponto de contato conhecido, depois rastejo (_DESCEND_TOUCH_V_MS).
        #   SEM ela (home ainda não aprendida) — o contato pode vir a qualquer
        #   momento: toda a descida respeita o teto de contato. O 1º tick após
        #   tocar penetra v_app·dt antes do loop reagir, gerando transiente
        #   ≈ v_app·lat·K — a velocidade no toque é o que limita esse pico.
        # Banda de chegada: termina o DESCENDING já DENTRO da tolerância do
        # HOLD (não em fz>=alvo, que garante overshoot).
        with self._params_lock:
            tol_override = self._hold_tol_n
        exit_tol = (tol_override if tol_override is not None
                    else max(_HOLD_TOL_N, _HOLD_TOL_PCT * target_f))

        # Velocidade DERIVADA do pico que o primeiro impacto pode ter, que é
        # o LIMIAR DE CONTATO e não a banda do alvo: o toque detecta, o
        # regulador é que sobe até o setpoint. Sem contato aprendido a descida
        # INTEIRA roda nela — é o preço, e ele é linear em T_halt (ver
        # crawl_v_ms).
        v_slow_ms      = min(v_fast_ms,
                             crawl_v_ms(self._K_RIGID_REF_NM))
        v_unlearned_ms = v_slow_ms
        # O orçamento do impacto é o limiar de contato, mas quem entrega é a
        # velocidade DEPOIS do clip. Acima de ~33 kN/m o piso de
        # _DESCEND_CRAWL_V_MIN_MS passa a mandar e o orçamento deixa de ser
        # atingível — em silêncio, porque crawl_v_ms só clipa. Este é o aviso.
        pico_prev = impact_peak_n(v_slow_ms, self._K_RIGID_REF_NM)
        if pico_prev > _CONTACT_ON_N * 1.05:
            self.get_logger().warn(
                f'[TOQUE] o primeiro impacto deve parar em '
                f'{_CONTACT_ON_N:.2f} N, mas a {self._K_RIGID_REF_NM/1e3:.0f} '
                f'N/mm e {v_slow_ms*1e6:.1f} µm/s o pico previsto é '
                f'{pico_prev:.2f} N ({pico_prev/_CONTACT_ON_N:.1f}x o '
                f'orçamento). A velocidade está no piso de '
                f'{_DESCEND_CRAWL_V_MIN_MS*1e6:.0f} µm/s, que existe para a '
                'descida não sumir no quantum de 10 µm da FK — descer mais '
                'devagar não é opção. O que compra o orçamento de volta é '
                f'medir T_halt (hoje {_STREAM_HALT_LAT_S:.2f} s, EMPRESTADO '
                'da frenagem e nunca medido) com latency_probe.py: o pico é '
                'linear nele.')
        # Margem da zona lenta ESCALADA pela velocidade do estágio rápido —
        # ver o bloco de _ZONE_REACTION_S — e LIMITADA pelo curso disponível.
        #
        # O teto pelo curso não é refinamento: sem ele a escala por velocidade
        # se autodestrói. Com approach_speed_mms=20 a zona vale 6 mm; o curso
        # em ar livre até o contato aprendido no run 20260814_115804 era de
        # 4,5 mm, então `learned_m - zone_m` caiu antes do ponto de partida, o
        # estágio rápido NUNCA executou e os 4,5 mm inteiros foram rastejados
        # a 0,2 mm/s — 22,8 s de descida contra os ~2 s que a velocidade
        # pedida daria. Na prática, qualquer approach acima de ~5 mm/s era
        # silenciosamente ignorado.
        #
        # Metade do curso é o repartidor: garante que o estágio rápido exista
        # sempre que houver curso para ele, e que a zona lenta nunca encolha
        # abaixo do que a desaceleração precisa quando o curso é curto.
        #
        # As duas parcelas SOMAM (ver o bloco de _CONTACT_MARGIN_*): a
        # frenagem é o curso gasto largando a velocidade do estágio rápido, e
        # a margem é o que sobra para rastejar de fato. Com max() a margem
        # sumia sempre que a frenagem passasse dela — a 20 mm/s o braço
        # terminava a rampa em cima do contato estimado, sem rastejo nenhum.
        brake_m = v_fast_ms * _ZONE_REACTION_S
        margin_m = self._contact_margin_m()
        # Piso de 3 mm. O clip antigo em 0,5·learned_m encolhia a zona
        # justamente na peça de contato raso; com o piso, contato raso rasteja
        # a descida inteira — o resultado seguro.
        zone_m = max(_DESCEND_DECEL_ZONE_M, brake_m + margin_m)
        # A zona lenta tem DUAS partes com papéis diferentes, e só uma precisa
        # da velocidade do orçamento de impacto:
        #   FRENAGEM (zone_m − margin_m) — curso para largar v_fast. O contato
        #     NÃO cabe aqui: ele mora em learned_m ± margin_m, e esta parte
        #     está inteira ACIMA dessa janela. Rastejá-la não compra segurança
        #     nenhuma, só tempo.
        #   MARGEM (margin_m) — a janela onde o contato PODE estar. Aqui a
        #     velocidade tem de ser crawl_v_ms, porque qualquer passo pode ser
        #     o que toca.
        # Até 01/09/2026 as duas rodavam a v_slow. Com approach de 15 mm/s a
        # frenagem vale 4,5 mm, e a 50 µm/s são 90 s de rastejo em ar livre
        # onde não há o que tocar: o run 20260901_103554 gastou 118 s de
        # DESCENDING para 11,6 mm, dos quais ~90 s foram isso.
        # Agora a frenagem é uma RAMPA v_fast→v_slow e só a margem rasteja.
        # A rampa linear em distância leva
        #   t = (frenagem/(v_fast−v_slow))·ln(v_fast/v_slow) ≈ 1,7 s
        # nos valores default — folgadamente acima de _ZONE_REACTION_S, que é
        # o que a frenagem precisa. Chega em crawl_from_m já em v_slow, então
        # o orçamento do primeiro impacto não muda.
        crawl_from_m = None if learned_m is None else learned_m - margin_m
        ramp_from_m = None if learned_m is None else learned_m - zone_m

        if depth_m <= 0.0:
            self.get_logger().warn('DESCENDING: profundidade = 0 mm — pulando fase.')
            return 'ok'
        I6    = np.eye(6)
        dt    = _CTRL_DT
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S

        # Estimador de rigidez e curva F(x) começam zerados a cada toque (o
        # objeto pode mudar entre ciclos, e uma curva do contato anterior
        # levaria a onda a comandar a penetração da amostra errada).
        self._k_est.reset()
        self._fx_curve.reset()

        descended_m = 0.0
        false_halts = 0   # gatilhos que não confirmaram (ver _contact_confirm)
        # Velocidade em ar livre do ÚLTIMO tick comandado — é o que decide se
        # o contato foi tocado no rastejo ou no estágio rápido. Começa em 0:
        # antes do 1º passo nada foi comandado, então um gatilho logo na
        # entrada da fase é ponteira JÁ apoiada, não impacto.
        v_at_touch = 0.0
        # Posição COMANDADA acumulada da descida em ar livre. Ressincroniza
        # com a medida se divergir (pausa/JTC).
        q_cmd_free: np.ndarray | None = None
        if learned_m is not None:
            _n_obs = len(self._contact_depths)
            _fonte = (f'dispersão medida em {_n_obs} contatos'
                      if _n_obs >= _CONTACT_MARGIN_MIN_PTS
                      else f'ainda no palpite conservador, {_n_obs} contato(s) '
                           f'de {_CONTACT_MARGIN_MIN_PTS}')
            _t_crawl = margin_m / max(v_slow_ms, 1e-9)
            zona = (f'RÁPIDA a {v_fast_ms*1e3:.1f} mm/s até '
                    f'{ramp_from_m*1e3:.1f} mm, FRENAGEM em rampa '
                    f'{v_fast_ms*1e3:.1f}→{v_slow_ms*1e3:.2f} mm/s até '
                    f'{crawl_from_m*1e3:.1f} mm, depois RASTEJO a '
                    f'{v_slow_ms*1e3:.2f} mm/s '
                    f'(contato aprendido em {learned_m*1e3:.1f} mm). '
                    f'Zona lenta {zone_m*1e3:.2f} mm = '
                    f'{brake_m*1e3:.2f} de frenagem + {margin_m*1e3:.2f} de '
                    f'margem ({_fonte}); o rastejo sozinho custa '
                    f'~{_t_crawl:.0f} s')
        else:
            zona = (f'{v_unlearned_ms*1e3:.1f} mm/s até contato '
                    f'(home ainda não aprendida — descida inteira rasteja e aprende)')
        self.get_logger().info(
            f'DESCENDING: alvo={target_f:.2f} ± {exit_tol:.2f} N  '
            f'curso máx={depth_m * 1000:.1f} mm  aproximação {zona}, '
            f'em contato QUASE-ESTÁTICO (K0={_K_DEFAULT_NM/1000:.0f} N/mm)  '
            f'(approach={approach_mms:.0f} mm/s × {self._speed_factor_pct:.0f}%)')

        while descended_m < depth_m:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self.get_logger().warn('[STOP] DESCENDING interrompido pelo usuário.')
                return 'stop'
            if not self._pause_gate():
                return 'stop'

            t0 = time.time()

            if self._force_stale_abort('DESCENDING'):
                return 'stale'
            fz = self._fz_corrected()  # + compressão, − tração
            if self._force_over_limit(fz):
                self._relieve_contact(approach_dir)   # recua NA HORA
                self.get_logger().error(
                    f'SEGURANÇA: força {fz:+.1f} N além da margem de '
                    f'{_FORCE_SAFE_LIMIT_N:.0f} N (teto {_FORCE_ABORT_LIMIT_N:.0f} N) '
                    f'— medição cancelada.')
                # Bater DURANTE a descida em ar livre = a superfície subiu
                # em relação ao aprendido nesta home.
                self._forget_contact('impacto na descida em ar livre')
                return 'force'

            if fz > _CONTACT_ON_N:
                # GATILHO: para de comandar descida NA HORA (o _settle trava a
                # posição atual) e SÓ ENTÃO confirma — ver o bloco
                # "HALTA PRIMEIRO, CONFIRMA DEPOIS" nas constantes.
                self._settle()
                ok, fz_conf = self._contact_confirm()
                if not ok:
                    false_halts += 1
                    self.get_logger().info(
                        f'DESCENDING: gatilho em {descended_m*1e3:.1f} mm '
                        f'(fz={fz:.2f} N) NÃO confirmou com o braço parado '
                        f'(fz={fz_conf:.2f} N) — ruído; retomando a descida '
                        f'({false_halts}/{_CONTACT_FALSE_MAX}).')
                    if false_halts >= _CONTACT_FALSE_MAX:
                        self.get_logger().error(
                            f'DESCENDING: {false_halts} falsos gatilhos — '
                            f'o limiar de {_CONTACT_ON_N:.2f} N está dentro '
                            'do ruído da célula. Abortando; suba '
                            '_CONTACT_ON_N ou refaça o tare.')
                        return 'error'
                    # Ressincroniza o comandado com o medido: o _settle pode
                    # ter deixado o JTC alcançar a posição.
                    q_cmd_free = None
                    continue
                fz = fz_conf
                if v_at_touch > _DESCEND_CONTACT_V_MAX_MS:
                    # O rastejo NÃO chegou a engatar: a superfície estava
                    # ACIMA do contato aprendido desta home e o toque
                    # aconteceu na velocidade do slider. O pico é
                    # v·latência·K e nenhuma malha o desfaz depois — no run
                    # MANUAL/20260817_113940 foram 4,9 N num alvo de 0,3 N,
                    # com a peça 3,2 mm mais alta que o aprendido (e o valor
                    # veio de uma home VIZINHA, cuja tolerância lateral não
                    # cobre desnível).
                    # Purgar a chave culpada (e as vizinhas) ANTES de
                    # memorizar o valor medido agora deixa a próxima descida
                    # com o contato certo em vez de repetir a batida.
                    self.get_logger().warn(
                        f'DESCENDING: contato a {v_at_touch*1e3:.1f} mm/s, '
                        f'acima do teto de toque de '
                        f'{_DESCEND_CONTACT_V_MAX_MS*1e3:.1f} mm/s '
                        f'(fz={fz:.2f} N). A superfície está ACIMA do contato '
                        'aprendido: o pico de impacto DESTA descida não é '
                        'controlável pela regulação.')
                    self._forget_contact(
                        'toque no estágio rápido — aprendido fundo demais')
                if descended_m > 0.001:
                    with self._params_lock:
                        self._remember_contact(descended_m)
                self.get_logger().info(
                    f'DESCENDING: contato em {descended_m * 1000:.1f} mm '
                    f'(fz={fz:.2f} N confirmado com o braço parado) — '
                    f'engatando regulação quase-estática.')
                out, fz_end = self._qs_regulate(
                    target_f, exit_tol, approach_dir, v_lim, I6,
                    budget_m=depth_m - descended_m,
                    stable_s=_QS_ARRIVE_S, timeout_s=_QS_TIMEOUT_S,
                    phase='DESCENDING-QS', feed_curve=True)
                if out == 'ok':
                    self.get_logger().info(
                        f'DESCENDING: alvo atingido — fz={fz_end:.2f} N '
                        f'(alvo {target_f:.2f} ± {exit_tol:.2f} N, '
                        f'{_QS_ARRIVE_S:.2f} s settled em banda)  '
                        f'K_est={self._k_est.value/1000:.0f} N/mm.')
                    return 'ok'
                if out == 'timeout':
                    if not self._qs_ever_contact:
                        # Nunca encostou de verdade: o gatilho foi espúrio.
                        self.get_logger().error(
                            f'DESCENDING-QS: {_QS_TIMEOUT_S:.0f} s sem NENHUMA '
                            f'leitura acima de {_CONTACT_ON_N:.2f} N '
                            f'(fz={fz_end:.2f} N) — o gatilho de contato foi '
                            'espúrio. Abortando como sem-contato.')
                        return 'no_contact'
                    self.get_logger().warn(
                        f'DESCENDING-QS: sem estabilizar em '
                        f'{_QS_TIMEOUT_S:.0f} s (fz={fz_end:.2f} N) — '
                        'entregando ao HOLD, que continua regulando.')
                    return 'ok'
                if out == 'budget':
                    self.get_logger().warn(
                        f'DESCENDING-QS: curso máximo esgotado sem sustentar '
                        f'{target_f:.2f} N (fz={fz_end:.2f} N).')
                    return 'no_contact'
                return out   # 'force' | 'stale' | 'stop'

            # Ar livre — três estágios (ver o perfil acima do loop).
            v_free = self._free_descent_v(
                descended_m, ramp_from_m, crawl_from_m,
                v_fast_ms, v_slow_ms, v_unlearned_ms)
            v_at_touch = v_free
            step_m = min(v_free * dt, depth_m - descended_m)

            tw = np.zeros(6)
            tw[:3] = approach_dir * step_m

            q_meas = self._q_now()
            q = q_meas if q_cmd_free is None else q_cmd_free
            # Divergência comandado×medido (pausa, JTC atrasado): resync.
            if q_cmd_free is not None and \
                    float(np.max(np.abs(q_cmd_free - q_meas))) > 0.02:
                q = q_meas
            J = jacobian(q, T_end=T_TOUCH_TOOL_ATTACH)
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + _JAC_LAM**2 * I6, tw)
            except np.linalg.LinAlgError:
                time.sleep(dt)
                continue

            q_new = np.clip(q + dq, JOINT_MIN, JOINT_MAX)
            vel   = np.clip((q_new - q) / dt, -v_lim, v_lim)
            self._stream_q(q_new, dt, velocities=vel)
            q_cmd_free = q_new
            descended_m += step_m

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

        self.get_logger().warn(
            f'DESCENDING: curso máximo de {descended_m * 1000:.1f} mm esgotado '
            f'sem atingir {target_f:.2f} N (fz={self._fz_corrected():.2f} N) — '
            'abortando com retorno lento à home.')
        return 'no_contact'

    # Rigidez de referência do contato RÍGIDO (ponteira sem silicone),
    # medida em 13/08/2026 no run TOUCH/20260813_153811. É o K conservador
    # com que a descida dimensiona a velocidade de rastejo antes de existir
    # qualquer estimativa — ver crawl_v_ms.
    _K_RIGID_REF_NM    = 28_000.0  # N/m (28 N/mm)


    def _phase_hold(self, timeout_s: float = _HOLD_TIMEOUT_S,
                    dwell_s: float | None = None) -> str:
        """HOLD — a rampa quase-estática leva a compressão ao setpoint, CONFIRMA
        a chegada e MANTÉM por `dwell_s` (medição) antes de liberar.

        Retorna: 'ok' | 'force' (> 15 N) | 'stale' (célula sem dados)
                 | 'stop' (usuário).
        """
        self._set_phase('HOLD')
        # Handoff DESCENDING→HOLD: espera a força ASSENTAR (dF/dt≈0) com
        # posição travada antes de devolver o controle.
        self._settle_until_quiet()
        with self._params_lock:
            target_f = float(self._target_force_n)
            # Overrides do PalpationStart (avançados da GUI); None = default.
            tol_override = self._hold_tol_n
            if self._hold_timeout_s is not None:
                timeout_s = self._hold_timeout_s
            # `dwell_s=None` = "use o que a GUI pediu". O campo `hold_stable_s`
            # da mensagem CHEGAVA aqui e não era lido por ninguém: as três
            # chamadas de _phase_hold omitem o argumento, então valia sempre o
            # _HOLD_DWELL_S embutido. O campo existia, era publicado, ia para o
            # params.json — e não governava nada. Quem passa dwell_s explícito
            # (a escada, com 0.0) continua mandando.
            if dwell_s is None:
                dwell_s = (self._hold_stable_s
                           if self._hold_stable_s is not None
                           else _HOLD_DWELL_S)

        tol_n = (tol_override if tol_override is not None
                 else max(_HOLD_TOL_N, _HOLD_TOL_PCT * target_f))
        approach_dir = (self._approach_dir if self._approach_dir is not None
                        else np.array([0., 0., -1.]))
        I6 = np.eye(6)
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S

        self.get_logger().info(
            f'HOLD-QS: alvo {target_f:.2f} ± {tol_n:.2f} N  '
            f'K_est={self._k_est.value/1000:.0f} N/mm  '
            f'dwell {dwell_s:.1f} s (timeout {timeout_s:.0f} s)')

        t_start = time.time()
        # Etapa 1: só CONFIRMAR a chegada (rampa monótona cruza limpo, não
        # precisa da janela `stable_s` da lei antiga). A medição de verdade é
        # o dwell abaixo. `stable_s` da GUI não some — o dwell continua sendo
        # o que ela dimensiona.
        out, fz = self._qs_regulate(target_f, tol_n, approach_dir,
                                    v_lim, I6,
                                    budget_m=None, stable_s=_QS_ARRIVE_S,
                                    timeout_s=timeout_s, phase='HOLD-QS')
        if out in ('force', 'stale', 'stop', 'target_lost'):
            return out

        # ── Dwell de medição: mantém o setpoint por dwell_s ──────────
        # 'timeout' da etapa acima NÃO pula o dwell (é o que a escada já
        # faz): pular entregava o SLIDING fora do alvo. Mede na mesma, e o
        # aviso final sai do resultado DESTA janela.
        if dwell_s > 0.0:
            self.get_logger().info(
                f'HOLD-QS: mantendo {dwell_s:.1f} s (medição).')
            out, fz = self._qs_regulate(
                target_f, tol_n, approach_dir, v_lim, I6,
                budget_m=None, stable_s=dwell_s,
                timeout_s=dwell_s + timeout_s, phase='HOLD-QS-DWELL')
            if out in ('force', 'stale', 'stop', 'target_lost'):
                return out
        timed_out = out == 'timeout'

        if timed_out:
            self.get_logger().warn(
                f'HOLD-QS: janela de medição fechou FORA da banda — '
                f'fz={self._fz_corrected():.2f} N '
                f'(alvo {target_f:.2f} ± {tol_n:.2f} N). Prosseguindo: '
                'o deadbeat do SLIDING continua corrigindo.')
        else:
            self.get_logger().info(
                f'HOLD-QS: medição concluída — '
                f'fz={self._fz_corrected():.2f} N '
                f'(alvo {target_f:.2f} ± {tol_n:.2f} N, dwell {dwell_s:.1f} s) '
                f'em {time.time() - t_start:.1f} s.')
        return 'ok'

    def _fmod_configured(self) -> bool:
        """True se um perfil trigonométrico foi PEDIDO (forma SINE/COSINE).

        Checagem barata e sem efeito colateral: não valida faixa nem loga, ao
        contrário de _force_profile. Serve só para escolher o caminho de
        execução do run antes de a primeira fase começar."""
        with self._params_lock:
            if self._fmod_from_msg:
                shape = self._fmod_shape
            else:
                try:
                    shape = self.get_parameter('force_mod_shape').value
                except Exception:
                    shape = None
        return str(shape or '').upper().strip() in ('SINE', 'COSINE')

    def _force_profile(self) -> '_ForceProfile | None':
        """Perfil de força modulada configurado, ou None se desligado.

        Fonte: os campos da PalpationStart quando a mensagem os trouxe
        (self._fmod_from_msg, gravado por _cb_start); senão os parâmetros
        ROS force_mod_*, que é o caminho de quem roda sem GUI.
        """
        with self._params_lock:
            from_msg = self._fmod_from_msg
            msg_vals = (self._fmod_shape, self._fmod_min_n,
                        self._fmod_max_n, self._fmod_hz, self._fmod_cycles)
        try:
            if from_msg:
                shape, f_min, f_max, hz, cycles = msg_vals
                src = 'GUI'
            else:
                shape = str(self.get_parameter('force_mod_shape').value)
                f_min = float(self.get_parameter('force_mod_min_n').value)
                f_max = float(self.get_parameter('force_mod_max_n').value)
                hz = float(self.get_parameter('force_mod_hz').value)
                cycles = int(self.get_parameter('force_mod_cycles').value)
                src = 'param'
            shape = str(shape).upper().strip()
            if shape not in _FMOD_SHAPES or shape == 'OFF':
                return None
            prof = _ForceProfile(shape, f_min, f_max, hz, cycles)
        except Exception as exc:                     # parâmetro ausente/inválido
            self.get_logger().error(
                f'[FMOD] perfil de força inválido ({exc}) — modulação OFF.')
            return None
        if prof.amp_n <= 1e-3 or prof.freq_hz <= 0.0 or prof.cycles <= 0:
            self.get_logger().warn(
                '[FMOD] perfil sem amplitude/frequência/ciclos — modulação OFF.')
            return None
        if prof.f_max_n > _FORCE_SETPOINT_MAX_N or prof.f_min_n < _CONTACT_ON_N:
            # O piso é _CONTACT_ON_N e NÃO um literal. Ele era '0,1' escrito à
            # mão nesta mensagem, e em 28/08/2026 _CONTACT_ON_N subiu para
            # 0,12: o run TOUCH/20260828_134305 pediu 0,10–2,00 N, foi
            # recusado, caiu no HOLD comum — e o log dizia "faixa 0.10–2.00 N
            # fora do permitido (0,1–10 N)", uma frase que recusa exatamente o
            # valor que ela mesma anuncia como permitido. Quem leu não tinha
            # como descobrir a causa, e o ensaio saiu sem onda nenhuma.
            self.get_logger().error(
                f'[FMOD] faixa {prof.f_min_n:.2f}–{prof.f_max_n:.2f} N fora do '
                f'permitido ({_CONTACT_ON_N:.2f}–{_FORCE_SETPOINT_MAX_N:.0f} N) '
                f'— modulação OFF, o ensaio vai rodar como HOLD comum. '
                f'O piso é o limiar de contato (CONTACT_ON_N = '
                f'{_CONTACT_ON_N:.2f} N): abaixo dele a onda pediria uma força '
                f'que o sistema não distingue de ar livre.')
            return None
        if prof.amp_n > _FMOD_MAX_AMP_N:
            self.get_logger().error(
                f'[FMOD] amplitude {prof.amp_n:.2f} N acima do teto '
                f'({_FMOD_MAX_AMP_N:.1f} N) — modulação OFF.')
            return None
        self.get_logger().info(f'[FMOD] ({src}) {prof.describe()}')
        return prof

    def _phase_hold_modulated(self, prof: '_ForceProfile') -> str:
        """TOUCH com força MODULADA: estabiliza na força média e depois oscila.

        A oscilação é FEEDFORWARD de posição: com a rigidez de contato K
        medida na descida, pedir ΔF de força é pedir Δx = ΔF/K de penetração,
        e o braço percorre essa senoide em posição. Não há realimentação de
        força na onda — a célula (10/80 Hz, com filtro) não fecha malha na
        frequência pedida, e realimentar aqui só produziria atraso de fase.
        A força medida continua sendo lida a cada tick para SEGURANÇA e vai
        para o CSV (force_net_n) ao lado de setpoint_n, que carrega a onda
        ENTREGUE: ela é reconstruída da penetração MEDIDA por FK (×K), não do
        valor que o laço acabou de enfileirar. É isso que mantém as duas
        colunas alinhadas no tempo e faz a frequência lida do CSV ser a
        realmente entregue — em MovL o comando corre à frente da execução.

        Neste modo não há um alvo único a perseguir: o que caracteriza o
        ensaio é a EXCURSÃO entre f_min e f_max e a frequência com que ela é
        percorrida. O log de fim audita as duas com TRÊS números, e a
        separação entre eles é o que permite achar a causa quando algo sai
        errado:

          1. amplitude MEDIDA pela célula — a única que não passa por K;
         1b. FORMA: fundamental e THD por lock-in nos harmônicos 1..3 — é
             ela que diz se saiu uma senoide ou uma senoide entalhada;
          2. rastreamento de POSIÇÃO (percorrido/comandado) — diz se o braço
             executou o movimento, e é independente de K por construção;
          3. frequência ENTREGUE, contada por cruzamentos da penetração.

        (1) baixo com (2) perto de 100 % significa braço certo e K errado;
        (1) certo com (1b) baixo significa excursão certa e FORMA errada —
        o p-p é cego a entalhe, a fundamental não; (2) baixo significa
        executor lento, e aí (3) diz se ele ao menos sustentou a frequência.

        A fase é MODULATING, não HOLD: sem isso o CSV não separa o
        assentamento inicial na força média do trecho em que a onda roda.

        Retorna: 'ok' | 'force' | 'stale' | 'stop' | 'timeout'.
        """
        # 1) Chega à força MÉDIA da onda pelo HOLD normal: é ele que garante
        #    contato estável e deixa K_est medido, que a onda usa em seguida.
        with self._params_lock:
            f_user = float(self._target_force_n)
            self._target_force_n = prof.mean_n
        try:
            out = self._phase_hold(dwell_s=0.0)
        finally:
            with self._params_lock:
                self._target_force_n = f_user
        if out != 'ok':
            return out

        k_nm = float(self._k_est.value)
        k0_nm = k_nm          # guardado p/ o log comparar com o adaptado
        # Feedforward pela CURVA F(x) medida na descida, quando ela existe.
        # O escalar K continua como reserva (curva curta demais, contato
        # perdido no meio da descida), mas ele é a origem da amplitude errada
        # documentada em _ContactCurve — não é o caminho preferido.
        curve = self._fx_curve
        use_curve = curve.usable
        fx_gain = 1.0        # correção da curva, adaptada por ciclo
        if use_curve:
            amp_m = 0.5 * abs(curve.dx_between(prof.f_min_n, prof.f_max_n))
            f_lo, f_hi = curve.f_range
            self.get_logger().info(
                f'[FMOD] curva F(x) da descida: {f_lo:.2f}–{f_hi:.2f} N '
                f'medidos, secante {curve.k_secant(f_lo, f_hi)/1e3:.2f} N/mm; '
                f'na faixa da onda ({prof.f_min_n:.2f}–{prof.f_max_n:.2f} N) '
                f'ela pede {2*amp_m*1e3:.2f} mm p-p. O escalar K_est '
                f'({k_nm/1e3:.2f} N/mm) pediria '
                f'{2*prof.amp_n/k_nm*1e3:.2f} mm — a razão entre os dois é a '
                f'não-linearidade que o escalar não enxerga.')
            # A descida para no ALVO dela, que costuma ser menor que o f_max
            # da onda (1,5 N contra 3,0 N no run 20260814_115804). O trecho
            # acima é EXTRAPOLADO pela secante quase-estática da ponta, que
            # num viscoelástico é mais mole que a rigidez dinâmica — a
            # extrapolação erra pedindo curso a MAIS. Quem corrige é o ganho
            # por ciclo, e a rampa é quem segura enquanto ele não corrigiu;
            # ainda assim o operador precisa saber que parte da faixa não foi
            # medida. Descer com target_force_n ≥ f_max elimina o problema.
            if prof.f_max_n > f_hi + 1e-6 or prof.f_min_n < f_lo - 1e-6:
                self.get_logger().warn(
                    f'[FMOD] a onda percorre '
                    f'{prof.f_min_n:.2f}–{prof.f_max_n:.2f} N mas a curva só '
                    f'mediu {f_lo:.2f}–{f_hi:.2f} N — o resto é extrapolado '
                    f'pela secante da ponta e tende a pedir curso a MAIS. '
                    f'Para medir a faixa inteira, desça com '
                    f'target_force_n ≥ {prof.f_max_n:.2f} N.')
        else:
            amp_m = prof.amp_n / k_nm
            self.get_logger().warn(
                f'[FMOD] curva F(x) insuficiente '
                f'(<{_FX_MIN_POINTS} pontos ou <{_FX_MIN_SPAN_N:.2f} N de '
                f'excursão) — caindo no escalar K={k_nm/1e3:.2f} N/mm. Num '
                f'contato não-linear isto erra a amplitude nas pontas da '
                f'faixa; confira a amplitude MEDIDA no log de fim.')

        approach_dir = (self._approach_dir if self._approach_dir is not None
                        else np.array([0., 0., -1.]))
        I6 = np.eye(6)
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S

        # Tick da ONDA — não o do QS. Derivado da frequência pedida para dar
        # _FMOD_MIN_PTS_PER_CYCLE pontos por período, com piso no período REAL
        # do ServoJ; ver _ForceProfile.wave_dt.
        servoj_period_s = float(
            self.get_parameter('servoj_period_s').value or _CTRL_DT)
        if servoj_period_s < _SERVOJ_T_MIN_S:
            # O `t` do ServoJ tem faixa [0.02, 3600] s no firmware do CR10 (ver
            # _SERVOJ_T_MIN_S). Um parâmetro abaixo disso não acelera o braço:
            # o controlador recusa o ponto, e aqui só faria o explorer publicar
            # mais rápido do que o mirror consegue comandar.
            self.get_logger().warn(
                f'[FMOD] servoj_period_s={servoj_period_s*1e3:.1f} ms está '
                f'abaixo do mínimo do ServoJ ({_SERVOJ_T_MIN_S*1e3:.0f} ms, '
                f'faixa [0.02, 3600] s do firmware) — tratando como '
                f'{_SERVOJ_T_MIN_S*1e3:.0f} ms. Suba o mirror_node para um '
                'valor válido.')
            servoj_period_s = _SERVOJ_T_MIN_S
        wave_dt = prof.wave_dt(servoj_period_s)
        pts_a_priori = prof.pts_per_cycle_at(wave_dt)

        # ── PREFLIGHT: a onda pedida existe nesta bancada? ─────────────
        # Seis checagens que são a mesma pergunta por ângulos diferentes —
        # frequência, velocidade, medida, correção, quantização, resolução —
        # e todas função de números que já estão na mão. Moram em
        # `force_wave.fmod_preflight`, que é pura: rodam em teste sem subir
        # nó nenhum, e o log sai na ordem em que a cadeia quebra.
        ilc_raw = self._fz_raw() is not None
        amp_pre, findings = fmod_preflight(
            prof,
            servoj_period_s=servoj_period_s, amp_m=amp_m, k_nm=k_nm,
            use_curve=use_curve, has_raw=ilc_raw,
            # A taxa da fonte que a ONDA vai ler: o canal cru quando existe,
            # senão o Float32. Ler a taxa do canal errado aqui aprovaria um
            # ensaio que a célula no fio não consegue medir.
            meas_rate_hz=(self._lc_raw_rate_hz() if ilc_raw
                          else self._lc_rate_hz()),
            deadband_tcp_m=_SERVOJ_DEADBAND_RAD * _ARM_REACH_M)
        for _level, _text in findings:
            getattr(self.get_logger(), _level)(_text)
        # Os achados saem TODOS antes da recusa, de propósito: um ensaio pode
        # esbarrar em dois limites ao mesmo tempo, e descobrir um por run é o
        # que o preflight existe para evitar.
        if any(_level == 'error' for _level, _ in findings):
            return 'error'

        q_cmd = self._q_now()
        # Teto do passo por tick, em POSIÇÃO. Pela curva quando ela existe: o
        # mesmo ΔF vale penetrações muito diferentes no pé e no topo da faixa,
        # e derivar o teto do escalar o tornava inútil (1,5 N / 0,70 N/mm =
        # 2,14 mm por tick, mais que a onda inteira).
        if use_curve:
            step_cap_m = abs(curve.dx_between(
                prof.mean_n, prof.mean_n + _FMOD_DF_STEP_MAX_N))
        else:
            step_cap_m = _FMOD_DF_STEP_MAX_N / k_nm
        dx_applied = 0.0        # penetração já comandada além da média (m)
        # Penetração ZERO da onda: onde o HOLD inicial deixou o braço, na força
        # média. Toda a excursão medida é contada a partir daqui.
        p_wave0 = forward_kinematics(
            self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)[:3, 3].copy()
        dx_exec_min = dx_exec_max = 0.0   # excursão ENTREGUE (m), para o log
        # Excursão MEDIDA pela célula. É a única verificação de amplitude que
        # não passa por K: a excursão em posição, multiplicada de volta por K
        # para virar newtons, usa o MESMO K que gerou o comando, então o fator
        # se cancela e ela reporta 100 % mesmo com K errado por ordens de
        # grandeza. Quem responde "a onda de força pedida aconteceu?" é a
        # célula, e ela já era lida a cada tick — só estava sendo descartada.
        meas = _WaveLockIn(prof.freq_hz)
        # Acumuladores do ciclo corrente para a adaptação (ver abaixo). São
        # LOCK-IN na frequência da onda — produto interno com sin/cos de f0 —
        # e não mais o pico-a-pico. Motivo medido nos runs de 17/08/2026: o
        # p-p é a estatística mais frágil que existe (dois pontos, os dois
        # extremos, os dois no ruído) e é cega à FORMA. Naqueles runs o p-p
        # da força batia a faixa pedida enquanto a FUNDAMENTAL entregava 81 a
        # 95 % — a onda tinha um entalhe no topo e o p-p não via. Adaptar
        # pelo p-p é perseguir o alvo errado; a amplitude que caracteriza uma
        # senoide é a da fundamental.
        cyc_xi = cyc_xq = 0.0     # lock-in da PENETRAÇÃO ENTREGUE (FK)
        # Lock-in da penetração COMANDADA. Parece redundante com o de cima e
        # não é: os dois medem coisas diferentes e servem a consumidores
        # diferentes.
        #
        #   entregue → adaptação de K.   K = ΔF/Δx REAL, e usar o comando ali
        #                                daria a rigidez de um braço ideal.
        #   comandada → atraso do ILC.   O ILC corrige o COMANDO, então a
        #                                fase que ele precisa é a de
        #                                comando→força. A fase contra a
        #                                posição entregue mede só
        #                                posição→força, que é o pedaço do
        #                                material e deixa de fora o servo.
        #
        # Trocar um pelo outro não degrada de leve: simulado a 10 Hz, usar a
        # entregue dá 168 % de fundamental e 34 % de THD (diverge), contra
        # 99 % e 2,6 % com a comandada.
        cyc_ci = cyc_cq = 0.0
        # Contagem PRÓPRIA dos lock-ins de posição, porque as duas medidas
        # têm cadências diferentes: o `meas` conta amostras da CÉLULA (~400 Hz)
        # e `cyc_xn` conta TICKS do laço, que é onde a posição entregue e a
        # comandada são amostradas. Normalizar um lock-in pelo contador do
        # outro erraria a amplitude pela razão entre as taxas — fator 8 a
        # 10 Hz.
        cyc_xn = 0
        cyc_idx = 0
        # Fração da amplitude em vigor no ciclo corrente — a adaptação compara
        # o ΔF medido com o que a rampa PEDIU, não com a amplitude cheia.
        amp_scale = cyc_amp_scale = _FMOD_AMP_RAMP_START
        k_adapts = 0
        band_clips = 0     # passos cortados por já estar fora da faixa
        vel_clips = 0      # passos cortados pelo teto de velocidade
        # Limites por ESCALA, não por corte: zerar o passo no pico abre um
        # entalhe, e o entalhe é o que derruba a fundamental sem derrubar o
        # pico-a-pico (assinatura dos runs de 17/08/2026). Ver o guarda de
        # excursão por ciclo, no fechamento do ciclo.
        band_clips_cycle = 0     # cortes DENTRO do ciclo corrente
        limit_scale = 1.0        # fator de amplitude imposto pelos limites
        limit_backoffs = 0

        # Contagem de cruzamentos da onda ENTREGUE, para medir a frequência
        # que o braço de fato percorreu. O laço de controle não serve de
        # relógio aqui: em MovL cada micro-passo é um RelMovL que o executor
        # da GUI consome no seu próprio ritmo (e coalesce os que se
        # acumularam), então o laço pode girar a 33 Hz enquanto o braço
        # percorre a onda a uma fração disso. Só a posição medida sabe.
        cross_count = 0
        cross_sign = 0
        # Banda morta do detector: abaixo dela o "cruzamento" seria ruído da
        # FK, não a onda.
        cross_band_m = max(0.25 * amp_m, _FMOD_QUIET_FLOOR_M)

        # ── CONTROLE REPETITIVO (ILC) ────────────────────────────────
        # A correção indexada por FASE que `fx_gain` não consegue fazer: ele é
        # UM escalar ajustado pelo módulo do lock-in, então corrige amplitude e
        # nada mais (ver o bloco _FMOD_ILC_*). Centro, fase e forma ficam onde
        # estavam, e foi exatamente isso que o run TOUCH/20260828_154934
        # mediu — amplitude certa com THD de 32 % e centro derivando.
        #
        # O teto da correção acompanha a amplitude em posição: corrigir mais
        # que _FMOD_ILC_MAX_FRAC dela não é erro de execução, é outro problema
        # (contato perdido, K absurda, tare errado), e insistir afunda a
        # ponteira.
        # Bins de fase: no MÁXIMO um por ponto COMANDADO por período. Corrigir
        # numa grade mais fina que a grade em que o comando existe não dá
        # resolução nenhuma — dá graus de liberdade que só o ruído preenche, e
        # o vetor passa a realimentar ruído nos harmônicos onde a planta não
        # responde. A 1 Hz o tick do QS dá ~33 pontos e valem os
        # _FMOD_ILC_BINS inteiros; a 10 Hz o período tem exatamente
        # _FMOD_MIN_PTS_PER_CYCLE pontos, e são 5 bins.
        #
        # Era fixo em 24, com o comentário de _FMOD_ILC_BINS já dizendo que
        # quem limita é o comando — só que o número não acompanhava.
        ilc_bins = int(min(max(round(pts_a_priori), 4), _FMOD_ILC_BINS))
        ilc = _WaveILC(n_bins=ilc_bins, clip_m=_FMOD_ILC_MAX_FRAC * amp_m)
        with self._lc_lock:
            self._lc_scale_sxy = self._lc_scale_sxx = 0.0
            self._lc_scale_n = 0
        # Atraso do pipeline de medida na frequência da onda. É o que separa o
        # setpoint da leitura que ele causou; sem descontá-lo o erro entra no
        # bin errado e o ILC aprende uma correção girada em fase.
        ilc_lag_s = fmod_measure_lag_s(prof.freq_hz, self._lc_rate_hz())
        ilc_learning = False   # vira True depois do warmup (ver abaixo)
        ilc_rms_m = 0.0
        # `ilc_raw`, `meas_gain` e `ilc_allowed` já saíram no preflight, que é
        # onde eles decidem se o ensaio roda. Aqui só se usa o resultado.
        #
        # ATENÇÃO ao que o sinal cru resolve e ao que NÃO resolve. Ele tira o
        # One-Euro do caminho, então o atraso do FILTRO deixa de existir. Não
        # tira o transporte: executor, ServoJ e o próprio material continuam
        # atrasando a força em relação ao comando, e a 10 Hz basta 5 ms disso
        # para valer 18°.
        #
        # E o atraso é o parâmetro CRÍTICO desta malha, não um detalhe de
        # sintonia. Simulado a 10 Hz com o resto perfeito:
        #
        #     erro de fase    fundamental entregue     THD
        #         0°               100 %              2 %
        #        50°               144 %             32 %
        #       194°               169 %             35 %   (diverge)
        #
        # Um ILC indexado por fase que recebe a fase errada não corrige menos:
        # ele realimenta positivamente. Por isso o atraso é MEDIDO no warmup
        # (ver o bloco do lock-in) e não estimado por fórmula. Até a primeira
        # medida sair, o valor analítico serve de semente.
        if ilc_raw:
            ilc_lag_s = 0.0
        ilc_lag_measured = False
        if ilc_raw:
            self.get_logger().info(
                f'[FMOD] medida CRUA disponível (/load_cell/sample_net): o '
                f'ILC fecha contra o sinal sem o One-Euro, então o ganho da '
                f'medida é 1,00 em vez de {fmod_measure_gain(prof.freq_hz):.2f} '
                f'a {prof.freq_hz:.2f} Hz. Escala em uso '
                f'{self._lc_raw_scale:.3f} N por unidade de voltage_raw '
                f'(1,0 = FA7155, que já entrega newtons).')

        # A fase ganha código próprio (MODULATING). Antes a onda herdava o
        # HOLD do assentamento inicial: no CSV o trecho em que o braço busca a
        # força média e o trecho em que ele percorre a onda eram a mesma fase,
        # e não havia como recortar a onda para analisá-la.
        self._set_phase('MODULATING')

        def _sp_executed() -> tuple[float, float]:
            """(setpoint entregue agora, penetração medida) — lidos da POSIÇÃO.

            A onda é feedforward de posição (Δx = ΔF/K), então a penetração
            medida por FK, multiplicada de volta por K, é o alvo que o robô
            de fato realizou NESTE instante. É isto que vai para o status (e
            daí para a coluna setpoint_n), e não o valor que o laço acabou de
            enfileirar: em MovL o comando é aceito antes de ser executado, e a
            onda comandada corre à frente da entregue. Publicar a comandada
            desalinhava setpoint_n de force_net_n no tempo e fazia a
            frequência lida do CSV ser a PEDIDA, não a entregue.
            """
            p = forward_kinematics(
                self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]
            dx = float(np.dot(p - p_wave0, approach_dir))
            return prof.mean_n + k_nm * dx, dx

        def _dx_of_sp(sp_n: float) -> float:
            """Penetração (m, contada da média) que o feedforward pede para a
            força `sp_n` — a ÚNICA fonte, usada pela rampa de arranque E pelo
            laço da onda.

            Ter uma só é o ponto: a rampa derivava a penetração do escalar
            `k_nm` enquanto o laço já derivava da curva. Os dois diferem pelo
            fator que é a própria razão de ser da _ContactCurve (34x de
            secante dentro da faixa do ensaio), então num COSINE — que abre em
            mean+amp — a rampa levava o braço a uma penetração calculada com o
            K do PÉ da curva e o primeiro tick do laço tinha de desfazê-la.
            Era exatamente o degrau que a rampa existe para eliminar; e como o
            K do pé é subestimado, o degrau apontava para DENTRO da amostra.

            Lê `fx_gain`/`k_nm` por closure, não por cópia: os dois são
            adaptados por ciclo dentro do laço, e a volta à média no fim
            precisa enxergar o valor corrente.
            """
            if use_curve:
                return fx_gain * curve.dx_between(prof.mean_n, sp_n)
            return (sp_n - prof.mean_n) / k_nm

        # 2) Arranque EM FASE. COSINE vale mean+amp em t=0, mas o HOLD deixou o
        #    braço na MÉDIA: entrar direto no laço pedia a amplitude inteira no
        #    primeiro tick, e o teto de _FMOD_DF_STEP_MAX_N por tick espalhava
        #    esse salto pelos primeiros ticks como um degrau de força que não
        #    faz parte da onda. Levar a penetração até o valor de t=0 ANTES de
        #    o relógio começar faz a onda começar exatamente na fase pedida.
        #    Para SINE, setpoint_n(0) == mean e a rampa é um no-op.
        #
        #    O alvo sai de _dx_of_sp — a MESMA fonte do laço — e já entra
        #    escalado por _FMOD_AMP_RAMP_START, que é a fração de amplitude em
        #    vigor no primeiro tick. Sem a escala, a rampa levava o braço à
        #    amplitude CHEIA e o laço, abrindo em 25 %, começava recuando: o
        #    arranque criava o degrau que devia evitar.
        dx0 = _dx_of_sp(prof.setpoint_n(0.0)) * _FMOD_AMP_RAMP_START * amp_pre
        # Teto de velocidade também no arranque: ele comanda a mesma
        # penetração que a onda, e uma K subestimada o torna tão rápido
        # quanto ela. Antes só STOP e o teto de 12 N o limitavam.
        v_cap_ramp_m = _FMOD_V_MAX_MMS * 1e-3 * wave_dt
        ramp_ticks = 0
        while (abs(dx0 - dx_applied) > 1e-9
               and ramp_ticks < _FMOD_RAMP_MAX_TICKS):
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                self.get_logger().warn(
                    '[STOP] FMOD interrompido no arranque em fase.')
                return 'stop'
            if self._force_over_limit():
                self._relieve_contact(approach_dir)
                self.get_logger().error(
                    f'SEGURANÇA: força além da margem de '
                    f'{_FORCE_SAFE_LIMIT_N:.0f} N no arranque em fase — '
                    f'modulação cancelada.')
                return 'force'
            # Limitador de faixa, igual ao do laço: o arranque aprofunda até
            # f_max e não além. A margem de 12 N do _force_over_limit é o teto
            # da MÁQUINA; o do ENSAIO é a faixa pedida, e era ela que ficava
            # sem guarda aqui — uma onda de 0,1–3,0 N podia ver 12 N antes do
            # primeiro tick.
            fz_ramp = self._fz_corrected()
            if fz_ramp > prof.f_max_n and dx0 > dx_applied:
                self.get_logger().warn(
                    f'[FMOD] arranque em fase interrompido em '
                    f'{dx_applied*1e6:+.0f} µm de {dx0*1e6:+.0f}: a força já '
                    f'está em {fz_ramp:.2f} N, no topo da faixa pedida '
                    f'({prof.f_max_n:.2f} N). A onda abre daqui — a rigidez '
                    'real é maior que a estimada na descida.')
                break
            step_m = float(np.clip(dx0 - dx_applied, -step_cap_m, step_cap_m))
            step_m = float(np.clip(step_m, -v_cap_ramp_m, v_cap_ramp_m))
            q_new = self._qs_step(approach_dir, step_m, v_lim, I6, q_from=q_cmd,
                                  dt=wave_dt)
            if q_new is None:
                self.get_logger().error(
                    '[FMOD] Jacobiano singular no arranque em fase — parando.')
                return 'error'
            q_cmd = q_new
            dx_applied += step_m
            ramp_ticks += 1
        if ramp_ticks:
            self.get_logger().info(
                f'[FMOD] arranque em fase: {dx_applied*1e6:+.0f} µm em '
                f'{ramp_ticks} ticks para abrir em '
                f'{prof.setpoint_n(0.0):.2f} N a '
                f'{100.0*_FMOD_AMP_RAMP_START:.0f} % de amplitude '
                f'({prof.shape} vale mean+amp em t=0, e a rampa de amplitude '
                f'abre nessa fração). Sem isto a onda abriria com um degrau.')

        def _observe(t_s: float, fz_n: float) -> None:
            """Uma amostra da célula no lock-in e no ILC.

            O erro vai para a fase que o CAUSOU: esta leitura é a resposta ao
            comando de ilc_lag_s atrás, então tanto o alvo quanto o índice de
            fase saem de (t_s − atraso). Aprender só depois do warmup —
            durante a rampa a onda pedida NÃO é a onda final."""
            meas.add(t_s, fz_n)
            t_cause = t_s - ilc_lag_s
            if ilc_learning and t_cause >= 0.0:
                ilc.observe(t_cause * prof.freq_hz,
                            prof.setpoint_n(t_cause) - fz_n, k_nm)

        # Relógio ÚNICO e monotônico. Era um t0 de parede para a fase da onda
        # e um t0_mono para a grade de ticks: dois relógios que podem divergir
        # (NTP mexe no de parede, não no monotônico), e agora a medida traz
        # carimbos monotônicos das amostras da célula, que precisam cair na
        # MESMA régua da fase. Um relógio só elimina a conversão e a deriva.
        t0_mono = time.monotonic()
        # Fronteira do dreno do histórico da célula: só amostras posteriores a
        # ela entram na medida, para nenhuma ser contada duas vezes.
        lc_drained_at = t0_mono
        ticks = 0               # ticks efetivos — dá o dt MEDIDO do laço
        # Ciclos consecutivos sem corte nem estouro de faixa, para o recuo de
        # amplitude poder voltar a subir.
        clean_cycles = 0
        outcome = 'ok'
        while True:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                # Fura a fila do executor MovL: os micro-passos já enfileirados
                # são da onda que acabou de ser cancelada. Sem isto o braço
                # real seguiria executando a onda depois do STOP.
                self.get_logger().warn('[STOP] FMOD interrompido pelo usuário.')
                outcome = 'stop'
                break
            if not self._pause_gate():
                # _pause_gate já dá halt ao pausar; aqui a saída é definitiva.
                outcome = 'stop'
                break
            if self._force_stale_abort('FMOD'):
                outcome = 'stale'
                break
            fz = self._fz_corrected()
            if self._force_over_limit(fz):
                self._relieve_contact(approach_dir)
                self.get_logger().error(
                    f'SEGURANÇA: força {fz:+.1f} N além da margem de '
                    f'{_FORCE_SAFE_LIMIT_N:.0f} N (teto '
                    f'{_FORCE_ABORT_LIMIT_N:.0f} N) — modulação cancelada.')
                outcome = 'force'
                break
            # DUAS leituras, de propósito. `fz` (filtrado) governa SEGURANÇA:
            # o One-Euro tem ganho 1 em DC, então uma sobrecarga sustentada
            # aparece nele inteira, e o filtro é justamente o que impede um
            # glitch de uma amostra de abortar o ensaio. `fz_meas` (cru,
            # quando existe) é o que MEDE a onda e alimenta as correções —
            # ali o filtro seria o erro, não a proteção.
            fz_meas = self._fz_raw()
            if fz_meas is None:
                fz_meas = fz

            t = time.monotonic() - t0_mono
            if t >= prof.duration_s:
                break
            sp = prof.setpoint_n(t)   # onda COMANDADA — só para gerar o passo
            # ── rampa de AMPLITUDE dos primeiros ciclos ──────────────
            # A adaptação abaixo corrige um ciclo por vez; a rampa é o que
            # segura a onda enquanto ela ainda não corrigiu. Escala só a
            # EXCURSÃO em torno da média — a fase, a frequência e o centro
            # ficam intactos, então o ensaio é o mesmo, com os primeiros
            # ciclos menores.
            amp_scale = min(1.0, _FMOD_AMP_RAMP_START
                            + (1.0 - _FMOD_AMP_RAMP_START)
                            * (t * prof.freq_hz) / _FMOD_AMP_RAMP_CYCLES)
            _, dx_exec = _sp_executed()
            dx_exec_min = min(dx_exec_min, dx_exec)
            dx_exec_max = max(dx_exec_max, dx_exec)
            # ── MEDIDA: cada amostra da célula com o SEU instante ────
            # Ler a célula NO TICK amostrava 400 Hz de sinal a 50 e dobrava os
            # harmônicos da interpolação sobre a fundamental — ver _WaveLockIn.
            # Drenando o histórico, a medida roda na taxa da CÉLULA. As
            # amostras que já pertencem ao próximo ciclo ficam para depois do
            # fechamento: entrar no ciclo que está terminando as poria no bin
            # errado, que é o erro que o dreno existe para não cometer.
            samples = self._lc_raw_since(lc_drained_at)
            if samples:
                lc_drained_at = samples[-1][0]
            else:
                # Sem canal cru no fio a única medida é a deste tick, na
                # cadência do laço. O preflight já garante que isso só
                # acontece abaixo de ONE_EURO_MAXCUTOFF_HZ, onde o tick do QS
                # dá amostras de sobra por período.
                samples = [(time.monotonic(), fz_meas)]
            carry = []
            for _ts, _fz in samples:
                _t_s = _ts - t0_mono
                if int(_t_s * prof.freq_hz) > cyc_idx:
                    carry.append((_t_s, _fz))
                else:
                    _observe(_t_s, _fz)
            # Cruzamentos da onda ENTREGUE (dão a frequência que o braço
            # percorreu), na penetração medida por FK deste tick.
            if dx_exec > cross_band_m and cross_sign <= 0:
                cross_sign = 1
                cross_count += 1
            elif dx_exec < -cross_band_m and cross_sign >= 0:
                cross_sign = -1
                cross_count += 1

            # ── K adapta A CADA CICLO ────────────────────────────────
            # A secante ΔF_medido / Δx_entregue do ciclo que acabou é a
            # rigidez NA amplitude e NA frequência deste ensaio. Num material
            # viscoelástico (silicone) ela não é a mesma coisa que a medida na
            # descida quase-estática, e era essa que a onda usava — congelada,
            # do início ao fim.
            #
            # Isto NÃO é realimentação de força na onda: a célula não fecha
            # malha na frequência do ensaio, e tentar isso só adicionaria
            # atraso de fase. É um PARÂMETRO atualizado uma vez por período,
            # com EMA — ordens de grandeza mais lento que a onda.
            # Lock-in da POSIÇÃO ENTREGUE: fica no tick, e é o lugar certo.
            # Ele sai da FK do feedback de juntas, que chega na cadência do
            # laço — não há histórico mais fino a drenar, e o consumidor
            # (adaptação de K) é por ciclo.
            _w = 2.0 * math.pi * prof.freq_hz * t
            _s, _c = math.sin(_w), math.cos(_w)
            cyc_xi += dx_exec * _s
            cyc_xq += dx_exec * _c
            cyc_xn += 1
            if int(t * prof.freq_hz) > cyc_idx:
                # Amplitude de PICO da fundamental = 2·|Σ x·e^{-jωt}|/N. A
                # fase não entra (o módulo a descarta), então o atraso de
                # transporte do executor — ~85 ms medidos, que a 2 Hz já
                # valem 60° — não contamina a medida como contaminaria uma
                # comparação instantânea comandado × medido.
                df_pp = meas.cycle_pp_n()
                dx_pp = 4.0 * math.hypot(cyc_xi, cyc_xq) / max(cyc_xn, 1)
                # ── ATRASO MEDIDO (identificação do plano, de graça) ──
                # Os dois lock-ins acima são os fasores da FORÇA e da
                # PENETRAÇÃO no mesmo ciclo. A diferença de fase entre eles é
                # ∠G(jω) — a fase do plano comandado→entregue, incluindo
                # executor, ServoJ e material. O `hypot` de cada um a joga
                # fora; ela é justamente o que o ILC precisa.
                #
                # Basta o valor MODULO um período: o ILC indexa a correção
                # por fase, então distinguir 54 ms de 154 ms a 10 Hz não muda
                # em que bin o erro cai. É por isso que medir aqui basta e
                # não é preciso desenrolar o número de ciclos do atraso.
                #
                # Só durante o warmup: depois o ILC já está corrigindo e a
                # fase que ele impõe contaminaria a medida do plano.
                if (not ilc_learning and df_pp >= _FMOD_K_ADAPT_MIN_DF_N
                        and math.hypot(cyc_ci, cyc_cq) > 1e-9):
                    _ph = meas.cycle_phase() - math.atan2(cyc_cq, cyc_ci)
                    # para (-pi, pi]; a força ATRASA, então a fase é negativa
                    _ph = (_ph + math.pi) % (2.0 * math.pi) - math.pi
                    _lag = (-_ph / (2.0 * math.pi * prof.freq_hz)) % (
                        1.0 / prof.freq_hz)
                    ilc_lag_s = _lag
                    ilc_lag_measured = True
                # PASSAGEM DE BASTÃO: `fx_gain`/K e o ILC corrigem a MESMA
                # grandeza (a amplitude da fundamental — o ILC pelo seu
                # componente em h1). Deixar os dois adaptando ao mesmo tempo
                # é pôr dois integradores no mesmo grau de liberdade, que
                # disputam e oscilam. O escalar faz o trabalho grosso
                # enquanto a rampa sobe; quando o ILC começa a aprender, ele
                # congela e o vetor assume.
                if (df_pp >= _FMOD_K_ADAPT_MIN_DF_N and dx_pp > 1e-7
                        and not ilc_learning):
                    if use_curve:
                        # Com a curva, o que se adapta é um GANHO sobre ela,
                        # não uma rigidez: a forma da não-linearidade já está
                        # na curva e não deve ser reachatada num escalar. O
                        # alvo é o ΔF que a rampa pedia NESTE ciclo — comparar
                        # com a amplitude cheia enquanto a rampa está em 25 %
                        # faria o ganho perseguir um alvo que ninguém pediu.
                        want_pp = 2.0 * prof.amp_n * cyc_amp_scale
                        if want_pp > 1e-6:
                            fx_gain = float(np.clip(
                                (1.0 - _FMOD_K_ADAPT_ALPHA) * fx_gain
                                + _FMOD_K_ADAPT_ALPHA * fx_gain
                                * (want_pp / df_pp),
                                _FX_GAIN_MIN, _FX_GAIN_MAX))
                            amp_m = 0.5 * fx_gain * abs(curve.dx_between(
                                prof.f_min_n, prof.f_max_n))
                            step_cap_m = fx_gain * abs(curve.dx_between(
                                prof.mean_n,
                                prof.mean_n + _FMOD_DF_STEP_MAX_N))
                            cross_band_m = max(0.25 * amp_m,
                                               _FMOD_QUIET_FLOOR_M)
                            k_adapts += 1
                    else:
                        k_meas = df_pp / dx_pp
                        if _K_MIN_NM <= k_meas <= _K_MAX_NM:
                            k_nm = ((1.0 - _FMOD_K_ADAPT_ALPHA) * k_nm
                                    + _FMOD_K_ADAPT_ALPHA * k_meas)
                            # Tudo que deriva de K acompanha, senão a onda
                            # passa a comandar com um K e a medir com outro.
                            amp_m = prof.amp_n / k_nm
                            step_cap_m = _FMOD_DF_STEP_MAX_N / k_nm
                            cross_band_m = max(0.25 * amp_m,
                                               _FMOD_QUIET_FLOOR_M)
                            # Devolve ao estimador: o próximo toque começa já
                            # com a rigidez vista NA onda, não com a da descida.
                            self._k_est.k = k_nm
                            self._k_est.estimated = True
                            k_adapts += 1
                # ── guarda de EXCURSÃO por ciclo ─────────────────────
                # Roda ANTES do ILC porque o resultado dele decide se este
                # ciclo pode ser aprendido: um ciclo em que a amplitude
                # comandada mudou não é um ciclo do plano.
                #
                # O gatilho é o EXTREMO MEDIDO do ciclo, não mais "houve
                # corte". Os dois motivos:
                #
                #  • o corte por passo deixa de existir em alta frequência
                #    (ver _FMOD_CLIP_LAG_FRAC), e amarrar o recuo a ele
                #    deixaria a onda sem guarda nenhum justamente onde ela é
                #    mais perigosa — o caso de K subestimada que mediu 188 %
                #    de estouro em bancada;
                #  • o extremo do ciclo é insensível a FASE por construção,
                #    então funciona com qualquer atraso de transporte, que é o
                #    que o guarda por passo não consegue fazer.
                _tol_n = max(_FMOD_BAND_TOL_FRAC * prof.amp_n,
                             _FMOD_BAND_TOL_MIN_N)
                _over = 0.0
                if meas.cyc_max is not None and meas.cyc_min is not None:
                    _over = max(meas.cyc_max - (prof.f_max_n + _tol_n),
                                (prof.f_min_n - _tol_n) - meas.cyc_min, 0.0)
                limit_moved = False
                if _over > 0.0:
                    # Recuo proporcional ao estouro MEDIDO: a onda encolhe
                    # INTEIRA, em vez de ser achatada na ponta. A forma é
                    # preservada — é a excursão que passa a valer outra.
                    _want = max(0.3, limit_scale
                                * (1.0 - _over / max(prof.amp_n, 1e-9)))
                    if _want < limit_scale:
                        limit_scale = _want
                        limit_backoffs += 1
                        limit_moved = True
                        self.get_logger().warn(
                            f'[FMOD] limites: o ciclo estourou a faixa em '
                            f'{_over:.2f} N ({band_clips_cycle} cortes por '
                            f'passo). Amplitude comandada recuada para '
                            f'{100*limit_scale:.0f} % — a onda encolhe '
                            f'INTEIRA, em vez de ser achatada na ponta. '
                            f'Este ciclo não foi aprendido pelo ILC.')
                    clean_cycles = 0
                elif band_clips_cycle:
                    clean_cycles = 0
                else:
                    clean_cycles += 1
                    # Recuperação. `limit_scale` só descia, e um único ciclo
                    # ruim no warmup — quando K ainda não adaptou e a onda
                    # ainda está na rampa — encolhia o ensaio INTEIRO de forma
                    # irreversível: o operador recebia uma senoide de outra
                    # amplitude sem ter mudado nada. Depois de
                    # _FMOD_LIMIT_RECOVER_CYCLES ciclos limpos ele volta a
                    # subir, em passos pequenos — descer é segurança, subir é
                    # conveniência, e as duas não têm a mesma pressa.
                    if (limit_scale < 1.0
                            and clean_cycles >= _FMOD_LIMIT_RECOVER_CYCLES):
                        limit_scale = min(
                            1.0, limit_scale + _FMOD_LIMIT_RECOVER_STEP)
                        limit_moved = True
                        clean_cycles = 0
                # ── ILC: fecha o ciclo ───────────────────────────────
                # `commit` só depois do warmup, e o warmup cobre a rampa de
                # amplitude inteira mais um ciclo — o primeiro ciclo em que a
                # onda pedida é de fato a onda final.
                # ── ETAPA 5: ciclo CORTADO não é ciclo aprendido ─────
                # `_acc`/`_cnt` do ILC guardam o erro observado neste ciclo.
                # Se houve corte — ou se a amplitude comandada acabou de mudar
                # — parte desse erro não é do plano: descartar é mais barato
                # (e muito mais seguro) do que separar as contribuições.
                if ilc_learning and (band_clips_cycle or limit_moved):
                    ilc.discard()
                elif ilc_learning:
                    ilc_rms_m = ilc.commit()
                elif (ilc_lag_measured
                        and t * prof.freq_hz >= _FMOD_ILC_WARMUP_CYCLES):
                    ilc_learning = True
                    # O teto sai do amp_m QUE VALE AGORA: durante o warmup o
                    # escalar adaptou e a amplitude em posição mudou junto, e
                    # um teto calculado antes disso seria de outra onda.
                    ilc.clip_m = abs(_FMOD_ILC_MAX_FRAC * amp_m)
                    _frozen = (f'ganho da curva congelado em {fx_gain:.2f}'
                               if use_curve else
                               f'K congelado em {k_nm / 1e3:.2f} N/mm')
                    self.get_logger().info(
                        f'[FMOD] ILC ligado no ciclo '
                        f'{int(t * prof.freq_hz)} (warmup de '
                        f'{_FMOD_ILC_WARMUP_CYCLES:.0f} ciclos cumprido). '
                        f'Correção indexada por fase, {ilc_bins} bins '
                        f'(teto {_FMOD_ILC_BINS}; a grade acompanha os '
                        f'{pts_a_priori:.0f} pontos comandados por período), '
                        f'teto ±{_FMOD_ILC_MAX_FRAC * amp_m * 1e6:.0f} µm; '
                        f'atraso MEDIDO no warmup '
                        f'{ilc_lag_s * 1e3:.0f} ms '
                        f'({360.0 * ilc_lag_s * prof.freq_hz:.0f}° a '
                        f'{prof.freq_hz:.2f} Hz; a fórmula previa '
                        f'{fmod_measure_lag_s(prof.freq_hz, self._lc_rate_hz())*1e3:.0f} ms). '
                        f'{_frozen} — daqui em diante quem corrige é o '
                        f'vetor.')
                band_clips_cycle = 0
                meas.reset_cycle()
                cyc_idx = int(t * prof.freq_hz)
                cyc_xi = cyc_xq = cyc_ci = cyc_cq = 0.0
                cyc_xn = 0
                cyc_amp_scale = amp_scale

            # Amostras que já eram do ciclo NOVO entram agora, depois do
            # fechamento — é o outro lado do `carry` lá em cima.
            for _t_s, _fz in carry:
                _observe(_t_s, _fz)

            # `setpoint_n` do CSV é a onda COMANDADA — o alvo, limpo, dentro
            # da faixa pedida.
            #
            # Antes publicava-se aqui a onda RECONSTRUÍDA da posição medida
            # (média + K·Δx). O motivo era real: em MovL o comando corria à
            # frente da execução, e publicar o comandado desalinharia
            # setpoint_n de force_net_n no tempo. Duas coisas mudaram:
            #
            #  • com a auto-cadência pelo executor o comando NÃO corre mais à
            #    frente — ele espera a fila esvaziar, então já está alinhado;
            #  • a reconstrução herdava o erro de K e o ruído da FK, e saía
            #    FORA DA FAIXA: medido em 14/08/2026, −1,085 a 4,199 N numa
            #    onda pedida de 0,1 a 3,0 N. Uma coluna chamada `setpoint_n`
            #    que sai da faixa pedida não é um setpoint.
            #
            # O que a onda ENTREGOU continua auditável, e por medidas
            # independentes: force_net_n (célula), tcp_* (posição) e os três
            # números do log de fim.
            with self._params_lock:
                self._force_sp_live = sp
            # Passo do tick = quanto falta para a penetração pedida agora.
            # Pela CURVA quando ela existe: `dx_between` já embute a
            # não-linearidade, então o mesmo ΔF de setpoint vale penetrações
            # diferentes no pé e no topo da faixa — que é justamente o que o
            # escalar não conseguia representar.
            # `amp_pre` devolve o que a interpolação entre pontos come da
            # fundamental (sinc², ver _fmod_sampling_gain). É malha aberta e
            # constante, então entra AQUI e não na adaptação por ciclo — que
            # continua livre para corrigir o que é do material.
            dx_target = _dx_of_sp(sp) * amp_scale * amp_pre * limit_scale
            # Correção do ciclo ANTERIOR, na fase corrente. Somada ao
            # feedforward, não realimentada: o vetor só muda no `commit`, uma
            # vez por período. É por isso que o atraso de 154 ms que impede a
            # malha fechada não desestabiliza isto — o ILC não corrige DENTRO
            # do ciclo, corrige o ciclo seguinte.
            dx_target += ilc.value(t * prof.freq_hz)
            # Lock-in da penetração COMANDADA, com o MESMO `_s`/`_c` deste
            # tick — o que fixa a fase é o `t` da amostra, não o ponto do
            # corpo do laço em que ela é somada. Acumular aqui é o que
            # permite usar `dx_target`, que só existe a partir desta linha.
            cyc_ci += dx_target * _s
            cyc_cq += dx_target * _c
            step_m = float(np.clip(dx_target - dx_applied,
                                   -step_cap_m, step_cap_m))

            # ── limitador de excursão pela força MEDIDA ──────────────
            # NÃO é malha fechada na onda: só corta o passo que aprofundaria
            # além do máximo pedido, ou aliviaria além do mínimo. Como age
            # apenas nos EXTREMOS, não introduz atraso de fase no corpo da
            # senoide — que é a razão de a onda ser feedforward.
            #
            # Existe porque a amplitude entregue depende de K, e K vem da
            # descida, medida a força BAIXA. O silicone enrijece sob carga: a
            # descida via 1,25 N/mm e a onda, operando entre 0,1 e 3,0 N,
            # media 2,0–2,9 N/mm. Com K subestimada o passo Δx = ΔF/K sai
            # grande demais e a faixa estoura — 188 % medidos em bancada em
            # 14/08/2026. O limitador tampa isso enquanto a adaptação
            # converge.
            #
            # Ao cortar é OBRIGATÓRIO ressincronizar `dx_applied` com a
            # penetração MEDIDA. Zerar só o passo deixava `dx_applied` parado
            # enquanto `dx_target` seguia correndo no relógio de parede: na
            # volta, o passo era calculado contra uma posição comandada que o
            # braço nunca ocupou, e a onda RETIFICAVA — medido em 14/08/2026
            # no run 20260814_115804, o topo estourou em +30 % e o vale nunca
            # chegou perto do piso (mínimo 0,418 N contra 0,10 N pedidos).
            # Com o resync o corte é simétrico: perde-se excursão na ponta
            # cortada, sem deslocar o resto da onda.
            #
            # O resync vale só no STREAMING. Em MovL o comando é aceito antes
            # de ser executado, então `dx_exec` mede uma fila pela metade e
            # não a posição que o laço comandou — ressincronizar ali
            # re-emitiria passos já enfileirados. Aquele caminho fica com o
            # comportamento antigo, que é o menor dos males enquanto a onda
            # em MovL não for confiável por outros motivos.
            #
            # A faixa tem TOLERÂNCIA (_FMOD_BAND_TOL_FRAC). Sem ela o
            # limitador mordia todo ciclo, e não nas emergências para as
            # quais foi feito: a leitura chega ~85 ms atrasada em relação ao
            # comando, então o pico da força medida cai DEPOIS do pico da
            # onda e passa do f_max por uma fração da amplitude só por causa
            # do atraso. Cortar ali abre um ENTALHE no topo — visível no
            # ciclo médio dos runs de 17/08/2026 (1 Hz: 2,31 → 2,05 → 2,28 N
            # no pico) — e o entalhe é justamente o que derruba a
            # fundamental sem derrubar o pico-a-pico. Com a tolerância o
            # limitador volta a ser o que diz ser: guarda de EXCURSÃO, não
            # regulador de ciclo.
            band_tol_n = max(_FMOD_BAND_TOL_FRAC * prof.amp_n,
                             _FMOD_BAND_TOL_MIN_N)
            # E ele só vale enquanto a leitura e o comando estão na MESMA
            # parte do ciclo. O atraso de transporte é ~85 ms constantes: 29°
            # a 1 Hz, mas 306° a 10 Hz. Naquele ponto a leitura que autoriza o
            # corte veio de quase um ciclo inteiro atrás, não tem relação com
            # o passo que está sendo cortado, e o guarda passa a abrir
            # entalhes em fase aleatória — que é exatamente o que derruba a
            # fundamental sem derrubar o pico-a-pico.
            #
            # Acima de _FMOD_CLIP_LAG_FRAC do período ele se cala e quem
            # guarda a excursão é o recuo POR CICLO, que é insensível a fase
            # por construção. A força segue vigiada a cada tick pelo
            # _force_over_limit, que é o teto da MÁQUINA e não do ensaio.
            clip_in_phase = ilc_lag_s * prof.freq_hz <= _FMOD_CLIP_LAG_FRAC
            if clip_in_phase and (
                    (step_m > 0.0 and fz_meas > prof.f_max_n + band_tol_n)
                    or (step_m < 0.0
                        and fz_meas < prof.f_min_n - band_tol_n)):
                step_m = 0.0
                dx_applied = dx_exec
                band_clips += 1
                band_clips_cycle += 1

            # ── teto de VELOCIDADE do TCP ───────────────────────────
            # Os tetos acima limitam FORÇA por passo. Com o tick caindo para
            # 4 ms em alta frequência, o mesmo ΔF por passo vale uma
            # velocidade 7x maior — e uma K subestimada aumenta o passo ainda
            # mais. Este teto é independente dos outros e é o que impede que
            # um erro de estimativa vire movimento rápido.
            v_cap_m = _FMOD_V_MAX_MMS * 1e-3 * wave_dt
            if abs(step_m) > v_cap_m:
                step_m = math.copysign(v_cap_m, step_m)
                vel_clips += 1

            # ── auto-cadência pelo EXECUTOR (modo MovL) ──────────────
            # Emitir um passo por tick enche a fila: o produtor roda a 33 Hz e
            # o executor consome ~7,5/s (cada RelMovL é um round-trip de
            # dashboard). Medido em bancada em 14/08/2026: 10 s de onda
            # comandada levaram 54 s para executar, e a frequência ENTREGUE
            # caiu para 0,18 Hz contra 1,00 pedida — 82 % de desvio.
            #
            # Aqui o passo só sai quando o executor está livre. O alvo continua
            # vindo do RELÓGIO DE PAREDE (dx_target = f(t)), então o que a onda
            # perde é RESOLUÇÃO — menos pontos por período — e não FREQUÊNCIA.
            # Trocar fase por resolução é o negócio certo: uma senoide grosseira
            # a 1 Hz ainda é uma senoide a 1 Hz; uma senoide fina entregue a
            # 0,18 Hz é outro ensaio.
            q_new = self._qs_step(approach_dir, step_m, v_lim, I6, q_from=q_cmd,
                                  dt=wave_dt,
                                  deadline=t0_mono + (ticks + 1) * wave_dt)
            if q_new is None:
                self.get_logger().error('[FMOD] Jacobiano singular — parando.')
                outcome = 'error'
                break
            q_cmd = q_new
            dx_applied += step_m
            ticks += 1
            # Cadência MEDIDA: o sleep de wave_dt vem depois do trabalho do
            # tick, então o período real é sempre maior que a constante. Com
            # 10 ticks já dá para dizer se a onda tem pontos suficientes —
            # avisa uma vez, ainda durante o toque.
            if ticks == 10:
                dt_meas = (time.monotonic() - t0_mono) / ticks
                pts_meas = 1.0 / max(prof.freq_hz * dt_meas, 1e-9)
                if pts_meas < _FMOD_MIN_PTS_PER_CYCLE:
                    self.get_logger().warn(
                        f'[FMOD] cadência MEDIDA {1.0/dt_meas:.1f} Hz '
                        f'(nominal {1.0/wave_dt:.1f}) dá {pts_meas:.1f} '
                        f'pontos por período — abaixo de '
                        f'{_FMOD_MIN_PTS_PER_CYCLE}. A onda executada é mais '
                        'grosseira que a pedida; use setpoint_n do CSV. '
                        'Este é o relógio do PRODUTOR: em MovL o braço pode '
                        'ir ainda mais devagar, e quem diz isso é a '
                        'frequência ENTREGUE, no log de fim.')

        el_cmd = time.monotonic() - t0_mono

        # Volta à média e devolve o setpoint fixo ao status.
        with self._params_lock:
            self._force_sp_live = None
        # Volta à média em passos LIMITADOS. A retração antiga era um único
        # _qs_step de -dx_applied, fora do teto de _FMOD_DF_STEP_MAX_N que
        # governa todos os outros passos da onda: num COSINE, que termina no
        # pico, isso soltava a amplitude inteira num comando só.
        if outcome == 'ok' and abs(dx_applied) > 1e-7:
            back_ticks = 0
            while (abs(dx_applied) > 1e-9
                   and back_ticks < _FMOD_RAMP_MAX_TICKS):
                step_m = float(np.clip(-dx_applied, -step_cap_m, step_cap_m))
                q_new = self._qs_step(approach_dir, step_m, v_lim, I6,
                                      q_from=q_cmd, dt=wave_dt)
                if q_new is None:
                    break
                q_cmd = q_new
                dx_applied += step_m
                back_ticks += 1
        self._settle()
        if outcome == 'ok':
            dt_meas = el_cmd / max(ticks, 1)
            el_wave = el_cmd
            self.get_logger().info(
                f'[FMOD] modulação concluída: {prof.cycles} ciclos comandados '
                f'em {el_cmd:.1f} s, {ticks} ticks '
                f'a {1.0/dt_meas:.1f} Hz medidos '
                f'({1.0/max(prof.freq_hz*dt_meas, 1e-9):.1f} pontos por '
                f'período; {pts_a_priori:.1f} no melhor caso).')
            if band_clips:
                self.get_logger().info(
                    f'[FMOD] {band_clips} passos cortados por já estar fora '
                    f'da faixa {prof.f_min_n:.2f}–{prof.f_max_n:.2f} N '
                    f'(de {ticks} emitidos).')
            if limit_backoffs:
                self.get_logger().warn(
                    f'[FMOD] amplitude recuada {limit_backoffs}x pelos '
                    f'limites, terminando em {100*limit_scale:.0f} % da '
                    f'pedida. A onda entregue é uma senoide DE OUTRA '
                    f'amplitude, não a pedida achatada — a forma vale, a '
                    f'excursão não. Se isto se repete, a faixa de força '
                    f'pedida não cabe neste material nesta frequência.')
            # ── conferência da escala do sinal CRU ────────────────────
            # A regressão usa os dois campos FILTRADOS da mesma mensagem, e o
            # filtro se cancela na razão. Se ela discordar do parâmetro, a
            # correção do ILC está sendo aplicada com o ganho errado — e no
            # limite com o SINAL errado, que é a única forma de esta etapa
            # piorar a onda em vez de melhorá-la.
            with self._lc_lock:
                _sxx, _sxy, _sn = (self._lc_scale_sxx, self._lc_scale_sxy,
                                   self._lc_scale_n)
            if ilc_raw and _sn > 20 and _sxx > 1e-12:
                _scale_meas = _sxy / _sxx
                _rel = abs(_scale_meas - self._lc_raw_scale) / max(
                    abs(self._lc_raw_scale), 1e-9)
                _sline = (f'[FMOD] escala do sinal cru: parâmetro '
                          f'{self._lc_raw_scale:.3f}, regressão '
                          f'{_scale_meas:.3f} N por unidade '
                          f'({_sn} amostras).')
                if _rel > 0.25:
                    self.get_logger().warn(
                        _sline + f' Discordância de {100*_rel:.0f} % — o ILC '
                        f'corrigiu com o ganho errado. Ajuste '
                        f'lc_raw_scale_n_per_unit:={_scale_meas:.3f} e '
                        f'repita; a onda deste run não vale.')
                else:
                    self.get_logger().info(_sline)
            if ilc.cycles:
                self.get_logger().info(
                    f'[FMOD] ILC aprendeu por {ilc.cycles} ciclos: correção '
                    f'RMS de {ilc_rms_m*1e6:.0f} µm, pico '
                    f'{np.abs(ilc.corr).max()*1e6:.0f} µm, contra uma '
                    f'amplitude de {amp_m*1e6:.0f} µm '
                    f'({100.0*np.abs(ilc.corr).max()/max(amp_m, 1e-9):.0f} % '
                    f'dela; teto {100.0*_FMOD_ILC_MAX_FRAC:.0f} %). Uma '
                    f'correção que ENCOSTA no teto não é erro de execução — '
                    f'procure contato perdido, K absurda ou tare errado '
                    f'antes de acreditar na onda.')
            elif not ilc_lag_measured:
                self.get_logger().warn(
                    f'[FMOD] ILC não ligou: a fase do plano nunca pôde ser '
                    f'medida (fundamental abaixo de '
                    f'{_FMOD_K_ADAPT_MIN_DF_N:.2f} N ou penetração parada nos '
                    f'ciclos de warmup). Ligar sem ela seria corrigir contra '
                    f'uma fase chutada, que a 10 Hz DIVERGE em vez de '
                    f'convergir. A onda rodou em malha aberta — confira a '
                    f'amplitude e o contato.')
            else:
                self.get_logger().warn(
                    f'[FMOD] ILC não fechou nenhum ciclo: a onda tem '
                    f'{prof.cycles} ciclos e o warmup consome '
                    f'{_FMOD_ILC_WARMUP_CYCLES:.0f}. Rode com pelo menos '
                    f'{int(_FMOD_ILC_WARMUP_CYCLES) + 3} ciclos para ele '
                    f'chegar a corrigir alguma coisa.')
            if vel_clips:
                self.get_logger().warn(
                    f'[FMOD] {vel_clips} passos cortados pelo teto de '
                    f'{_FMOD_V_MAX_MMS:.0f} mm/s. A onda pedida exige mais '
                    f'velocidade do que o teto permite — a amplitude entregue '
                    f'fica abaixo da pedida. Reduza a amplitude ou a '
                    f'frequência.')
            if k_adapts and use_curve:
                self.get_logger().info(
                    f'[FMOD] ganho da curva F(x) adaptou {k_adapts}x: '
                    f'1,00 → {fx_gain:.2f} ({100.0*(fx_gain-1.0):+.0f} %). '
                    f'A amplitude comandada acompanhou, terminando em '
                    f'±{amp_m*1e6:.0f} µm. Um ganho perto de 1 significa que '
                    f'a curva da descida já descrevia o contato na amplitude '
                    f'e na frequência do ensaio.')
            elif k_adapts:
                self.get_logger().info(
                    f'[FMOD] K adaptou {k_adapts}x durante a onda: '
                    f'{k0_nm/1e3:.2f} → {k_nm/1e3:.2f} N/mm '
                    f'({100.0*(k_nm-k0_nm)/max(k0_nm, 1e-9):+.0f} %). A '
                    f'amplitude comandada acompanhou: ±{prof.amp_n/k0_nm*1e6:.0f} '
                    f'→ ±{amp_m*1e6:.0f} µm.')
            else:
                self.get_logger().warn(
                    f'[FMOD] K NÃO adaptou nenhuma vez (segue em '
                    f'{k_nm/1e3:.2f} N/mm, da descida). Ou a onda não '
                    f'completou um ciclo, ou a fundamental medida ficou '
                    f'abaixo de {_FMOD_K_ADAPT_MIN_DF_N:.2f} N — nos dois casos a '
                    f'amplitude comandada saiu de uma rigidez não confirmada.')

            # ── 1. Amplitude: o que a CÉLULA mediu ────────────────────
            # Esta é a única conferência de amplitude que não passa por K.
            # A excursão em posição vezes K (abaixo) NÃO serve para isso: o
            # comando é dx = ΔF/K e a leitura é ΔF' = dx·K, então K se
            # cancela e a razão dá ~100 % mesmo com K errado por ordens de
            # grandeza — que é exatamente o erro que se quer pegar.
            want_pp_n = 2.0 * prof.amp_n
            if meas.f_min is not None and meas.f_max is not None:
                meas_pp_n = meas.f_max - meas.f_min
                meas_frac = 100.0 * meas_pp_n / max(want_pp_n, 1e-9)
                fline = (f'[FMOD] amplitude MEDIDA pela célula: '
                         f'{meas_pp_n:.2f} N pico-a-pico contra '
                         f'{want_pp_n:.2f} N pedidos — {meas_frac:.0f} %. '
                         f'Independente de K (K={k_nm/1e3:.1f} N/mm foi só '
                         f'quem gerou o passo).')
                if meas_frac < 50.0 or meas_frac > 200.0:
                    self.get_logger().warn(
                        fline + ' Fora da faixa 50–200 %: se o rastreamento '
                        'de posição abaixo estiver perto de 100 %, o braço '
                        'fez o movimento pedido e quem está errado é K — '
                        'refaça a estimativa de rigidez.')
                else:
                    self.get_logger().info(fline)

            # ── 1b. A onda saiu SENOIDE? ──────────────────────────────
            # O p-p acima responde "houve excursão"; não responde "com que
            # FORMA". Quem responde é o lock-in nos harmônicos 1..3 da força
            # medida: a fundamental é a amplitude que caracteriza a senoide,
            # e a raiz da soma dos harmônicos sobre ela é a distorção. Nos
            # runs de 17/08/2026 as duas divergiam muito — p-p em 100 % da
            # faixa com a fundamental em 81 %, THD de 15 a 30 % — e era o
            # entalhe do limitador de faixa aparecendo na forma sem aparecer
            # no p-p.
            if meas.tot_n > 0:
                _amps = meas.harmonic_amps()
                if _amps[0] > 1e-6:
                    thd = 100.0 * math.hypot(_amps[1], _amps[2]) / _amps[0]
                    fund_frac = 100.0 * _amps[0] / max(prof.amp_n, 1e-9)
                    hline = (f'[FMOD] FORMA da onda entregue: fundamental '
                             f'{_amps[0]:.2f} N de amplitude contra '
                             f'{prof.amp_n:.2f} N pedidos ({fund_frac:.0f} %), '
                             f'harmônicos 2º/3º {_amps[1]:.2f}/{_amps[2]:.2f} N '
                             f'⇒ THD {thd:.0f} %, de {meas.tot_n} amostras '
                             f'da célula '
                             f'({meas.tot_n/max(prof.cycles, 1):.0f} por '
                             f'período). O piso de THD desta cadência é o da '
                             f'interpolação entre os {pts_a_priori:.0f} pontos '
                             f'comandados (~7 % a 5 pontos, ~2 % a 8) e NÃO é '
                             f'compensável em amplitude.')
                    if fund_frac < 90.0 or fund_frac > 110.0 or thd > 15.0:
                        self.get_logger().warn(
                            hline + ' Fundamental fora de ±10 % ou THD acima '
                            'de 15 %: a onda entregue não é a senoide pedida. '
                            'Confira pontos por período (interpolação), o '
                            'número de cortes de faixa e a convergência do '
                            'ganho — nessa ordem.')
                    else:
                        self.get_logger().info(hline)

            # ── 2. Rastreamento: o braço fez o MOVIMENTO pedido? ──────
            # Métrica de POSIÇÃO, não de força: é a razão entre a excursão
            # percorrida e a comandada. Serve para separar "o braço não
            # executou" de "o braço executou, mas K estava errado".
            exec_pp_m = dx_exec_max - dx_exec_min
            want_pp_m = 2.0 * amp_m
            track_frac = 100.0 * exec_pp_m / max(want_pp_m, 1e-9)
            tline = (f'[FMOD] rastreamento de POSIÇÃO: {exec_pp_m*1e6:.0f} µm '
                     f'percorridos contra {want_pp_m*1e6:.0f} µm comandados — '
                     f'{track_frac:.0f} %. setpoint_n no CSV é a onda '
                     f'ENTREGUE, não a comandada.')
            if track_frac < 50.0:
                self.get_logger().warn(
                    tline + ' Abaixo de 50 %: o braço não está executando a '
                    'onda pedida — reduza a frequência ou aumente a '
                    'amplitude, e trate o CSV como a fonte do que ocorreu.')
            else:
                self.get_logger().info(tline)

            # ── 3. Frequência: a que o braço de fato percorreu ────────
            # Cada meio-período da onda entregue vira um cruzamento da banda
            # morta em torno da penetração zero. O laço de controle NÃO serve
            # de relógio aqui: em MovL cada micro-passo é um RelMovL que o
            # executor da GUI consome no seu próprio ritmo, somando os que se
            # acumularam — o que preserva a frequência e corta a amplitude
            # enquanto ele acompanha, e perde as duas quando não acompanha.
            # A contagem tem incerteza de ±1 meio-período (as bordas), o que
            # a 20 ciclos vale ~2,5 %.
            if el_wave > 0.0 and cross_count >= 2:
                freq_meas = cross_count / (2.0 * el_wave)
                dev = abs(freq_meas - prof.freq_hz) / max(prof.freq_hz, 1e-9)
                qline = (f'[FMOD] frequência ENTREGUE: {freq_meas:.2f} Hz '
                         f'({cross_count} meios-períodos em {el_wave:.1f} s) '
                         f'contra {prof.freq_hz:.2f} Hz pedidos.')
                if dev > _FMOD_FREQ_TOL_FRAC:
                    self.get_logger().warn(
                        qline + f' Desvio de {100.0*dev:.0f} % — o executor '
                        'não sustenta a frequência pedida. Reduza a '
                        'frequência ou os ciclos; a coluna setpoint_n do CSV '
                        'carrega a onda que realmente saiu.')
                else:
                    self.get_logger().info(qline)
            else:
                self.get_logger().warn(
                    f'[FMOD] frequência ENTREGUE não medível: só '
                    f'{cross_count} cruzamento(s) da banda de '
                    f'{cross_band_m*1e6:.1f} µm em {el_wave:.1f} s. A onda '
                    f'ficou no piso de ruído da FK — aumente a amplitude '
                    f'para poder auditar a frequência.')
        return outcome

    def _phase_hold_dynamic(self) -> str:
        """HOLD INFINITO (modo MANUAL): regula continuamente para o setpoint
        CORRENTE (self._target_force_n, atualizável via /palpation/set_force)
        por micro-passos quase-estáticos. Sem janela de estabilização nem
        timeout — encerra APENAS em STOP do usuário, force (> margem de
        segurança) ou stale. A mudança de setpoint (ex.: 1 N → 2 N) é seguida
        sem reiniciar a FSM nem refazer a descida.

        Retorna: 'stop' | 'force' | 'stale'.
        """
        self._set_phase('HOLD')
        # Mesmo handoff do HOLD automático: assenta a força antes de regular.
        self._settle_until_quiet()
        with self._params_lock:
            target_f = float(self._target_force_n)
        tol_n = (self._hold_tol_n if self._hold_tol_n is not None
                 else max(_HOLD_TOL_N, _HOLD_TOL_PCT * target_f))
        approach_dir = (self._approach_dir if self._approach_dir is not None
                        else np.array([0., 0., -1.]))
        I6 = np.eye(6)
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        self.get_logger().info(
            f'HOLD-MANUAL: regulação infinita, alvo inicial {target_f:.2f} N '
            '(ajustável via /palpation/set_force). Encerra em STOP/force/stale.')
        # stable_s=inf/timeout_s=inf → nunca sai por estabilidade/timeout;
        # dynamic=True → segue o setpoint corrente pelos micro-passos.
        out, _ = self._qs_regulate(
            target_f, tol_n, approach_dir, v_lim, I6,
            budget_m=None, stable_s=float('inf'),
            timeout_s=float('inf'), phase='HOLD-MANUAL', dynamic=True)
        return out

    def _phase_hold_staircase(self, levels: list[float],
                              dwell_s: float) -> str:
        """MANUAL em DEGRAU: percorre `levels` parando `dwell_s` em cada um.

        Cada patamar é o HOLD quase-estático de sempre — assenta na banda e
        só então cumpre o dwell de medição. Nada de cinemática nova: o que
        muda é quem escolhe o setpoint (a escada, não o tópico
        /palpation/set_force).

        `_target_force_n` é atualizado a cada degrau, então o status já
        publica o patamar corrente em `target_force_n` e a coluna
        `setpoint_n` do CSV carimba cada amostra com o degrau que a
        produziu — é o que permite fatiar a curva por nível depois.

        Retorna: 'ok' | 'force' | 'stale' | 'stop' | 'timeout'.
        """
        self._set_phase('HOLD')
        self._settle_until_quiet()

        with self._params_lock:
            f_user = float(self._target_force_n)
            tol_override = self._hold_tol_n
            timeout_s = (self._hold_timeout_s
                         if self._hold_timeout_s is not None
                         else _HOLD_TIMEOUT_S)
        approach_dir = (self._approach_dir if self._approach_dir is not None
                        else np.array([0., 0., -1.]))
        I6 = np.eye(6)
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S

        n = len(levels)
        peak = max(levels) if levels else 0.0
        self.get_logger().info(
            f'[DEGRAU] {n} patamares de {levels[0]:.2f} a {peak:.2f} N e de '
            f'volta, {dwell_s:.1f} s em cada — duração mínima estimada '
            f'{n * dwell_s:.0f} s sem contar a acomodação.')

        outcome = 'ok'
        try:
            for i, level in enumerate(levels, start=1):
                if self._stop_requested.is_set():
                    self._stop_requested.clear()
                    self.get_logger().warn(
                        f'[DEGRAU] STOP no patamar {i}/{n}.')
                    outcome = 'stop'
                    break
                if not self._pause_gate():
                    outcome = 'stop'
                    break

                with self._params_lock:
                    self._target_force_n = float(level)
                tol_n = (tol_override if tol_override is not None
                         else max(_HOLD_TOL_N, _HOLD_TOL_PCT * level))
                arrow = '↑' if i <= (n + 1) // 2 else '↓'
                self.get_logger().info(
                    f'[DEGRAU] {arrow} patamar {i}/{n}: {level:.2f} '
                    f'± {tol_n:.2f} N')

                # 1) Rampa até o patamar + confirmação curta da chegada. A
                # medição é o dwell (etapa 2); a janela `stable_s` da lei
                # antiga não é mais necessária — a rampa cruza monótona.
                out, _ = self._qs_regulate(
                    level, tol_n, approach_dir, v_lim, I6,
                    budget_m=None, stable_s=_QS_ARRIVE_S, timeout_s=timeout_s,
                    phase=f'DEGRAU-{i}')
                if out in ('force', 'stale', 'stop', 'target_lost'):
                    outcome = out
                    break
                if out == 'timeout':
                    # Não aborta o ensaio: registra e segue. Um patamar que
                    # não chega ao alvo costuma ser o quantum de força do
                    # atuador, não falha — e os demais degraus ainda valem.
                    self.get_logger().warn(
                        f'[DEGRAU] patamar {i}/{n} ({level:.2f} N) não '
                        f'chegou ao alvo em {timeout_s:.0f} s; medindo assim mesmo.')

                # 2) Dwell de medição — é o patamar propriamente dito.
                if dwell_s > 0.0:
                    out, _ = self._qs_regulate(
                        level, tol_n, approach_dir, v_lim, I6,
                        budget_m=None, stable_s=dwell_s,
                        timeout_s=dwell_s + timeout_s,
                        phase=f'DEGRAU-{i}-DWELL')
                    if out in ('force', 'stale', 'stop', 'target_lost'):
                        outcome = out
                        break
                    if out == 'timeout':
                        # O dwell correu os `dwell_s` de relógio (a defesa
                        # re-rampa sem reiniciá-lo), mas a força TERMINOU fora
                        # da banda — a relaxação/recuperação viscoelástica
                        # ganhou da rampa neste patamar. Passar adiante em
                        # silêncio carimbava no CSV um patamar "medido" cuja
                        # janela fechou fora do alvo.
                        self.get_logger().warn(
                            f'[DEGRAU] patamar {i}/{n} ({level:.2f} N): a '
                            f'janela de {dwell_s:.1f} s fechou com a força '
                            'FORA da banda. As amostras existem, mas o '
                            'patamar não assentou — trate-o com ressalva na '
                            'análise.')
        finally:
            # Devolve o setpoint que esta fase recebeu (o _run_protocol o
            # trocou pelo primeiro patamar antes da descida e restaura o do
            # usuário depois) e a velocidade do slider.
            with self._params_lock:
                self._target_force_n = f_user

        if outcome == 'ok':
            self.get_logger().info(
                f'[DEGRAU] escada concluída: {n} patamares medidos.')
        return outcome



    def _slide_frame(self, dir_xy: np.ndarray
                     ) -> tuple[np.ndarray, np.ndarray, np.ndarray, str] | None:
        """Base ortonormal do deslize — (u, w, n, procedência).

        u = avanço: a direção pedida PROJETADA no plano da amostra. Como u já
            carrega a componente vertical, o curso pedido vira distância
            sobre a SUPERFÍCIE, e não a projeção horizontal dela.
        w = transversal, no plano (n × u): trava o desvio lateral.
        n = normal do plano, apontando para FORA da superfície: trava a
            profundidade do contato.

        A origem do plano segue a precedência do bloco "Plano do deslize":
        normal medida pela calibração > inclinação declarada > horizontal.

        None quando a direção pedida é degenerada (nula, ou perpendicular ao
        plano — deslizar "para dentro" da peça não é deslizar).
        """
        u_xy = np.array([float(dir_xy[0]), float(dir_xy[1]), 0.0])
        if float(np.linalg.norm(u_xy)) < 1e-9:
            self.get_logger().error('SLIDING: direção inválida.')
            return None
        u_xy /= float(np.linalg.norm(u_xy))

        with self._params_lock:
            n_meas = (None if self._slide_plane_n is None
                      else self._slide_plane_n.copy())
            slope_deg = float(self._slide_slope_deg)
        if n_meas is not None:
            n = np.asarray(n_meas, dtype=float)
            src = 'medido'
        else:
            # Plano descrito pela inclinação declarada ao longo do curso: a
            # normal que dá subida de tan(θ) na direção u_xy é ẑ − tanθ·u_xy.
            n = _Z_HAT - math.tan(math.radians(slope_deg)) * u_xy
            src = (f'declarado {slope_deg:+.2f}°' if abs(slope_deg) > 1e-9
                   else 'horizontal')
        nn = float(np.linalg.norm(n))
        if nn < 1e-9:
            self.get_logger().error('SLIDING: normal do plano degenerada.')
            return None
        n = n / nn
        if float(n @ _Z_HAT) < 0.0:
            n = -n     # sempre para FORA da superfície (lado da ferramenta)

        # Avanço = direção pedida projetada no plano.
        u = u_xy - float(u_xy @ n) * n
        un = float(np.linalg.norm(u))
        if un < 1e-6:
            self.get_logger().error(
                'SLIDING: a direção pedida é perpendicular ao plano medido — '
                'não há curso lateral possível.')
            return None
        u = u / un
        w = np.cross(n, u)
        wn = float(np.linalg.norm(w))
        if wn < 1e-9:
            self.get_logger().error('SLIDING: base do deslize degenerada.')
            return None
        return u, w / wn, n, src

    class _ContactWatch:
        """Vigia de perda de contato do SLIDING."""

        def __init__(self, target_f: float, budget_m: float,
                     min_frac: float):
            self.floor_n = max(_CONTACT_ON_N, min_frac * abs(target_f))
            self.budget_m = budget_m
            self.lost_from: float | None = None   # progresso onde caiu
            self.lost_at: float | None = None     # progresso da 1ª perda
            self.worst_m = 0.0

        def update(self, progress_m: float, fz: float) -> float:
            """Devolve a extensão contínua SEM contato (m) até aqui."""
            if abs(fz) >= self.floor_n:
                self.lost_from = None
                return 0.0
            if self.lost_from is None:
                self.lost_from = progress_m
                if self.lost_at is None:
                    self.lost_at = progress_m
            run_m = max(0.0, progress_m - self.lost_from)
            self.worst_m = max(self.worst_m, run_m)
            return run_m

        def exceeded(self, progress_m: float, fz: float) -> bool:
            return self.update(progress_m, fz) > self.budget_m

    def _slide_lost_contact_report(self, watch: '_ContactWatch',
                                   target_f: float, progress_m: float) -> None:
        """Loga a perda de contato COM o número acionável: a inclinação da
        superfície que explicaria ter perdido a indentação naquela distância.
        """
        k_nm = float(self._k_est.value or _K_DEFAULT_NM)
        indent_m = abs(target_f) / max(k_nm, 1.0)
        d_m = watch.lost_at if watch.lost_at else progress_m
        slope_deg = (math.degrees(math.atan2(indent_m, d_m))
                     if d_m > 1e-6 else float('nan'))
        verdict = ('O trecho restante NÃO é medição — run abortado. '
                   if _SLIDE_LOST_ABORTS else
                   'Guarda em modo AVISO (_SLIDE_LOST_ABORTS=False): o curso '
                   'segue até o fim e o run termina normalmente, mas o trecho '
                   'sem contato NÃO é medição — corte-o na análise. ')
        log = (self.get_logger().error if _SLIDE_LOST_ABORTS
               else self.get_logger().warn)
        log(
            f'[SLIDING] CONTATO PERDIDO: força abaixo de '
            f'{watch.floor_n:.2f} N por {watch.worst_m*1e3:.1f} mm contínuos '
            f'(orçamento {watch.budget_m*1e3:.0f} mm), a partir de '
            f'{(watch.lost_at or 0.0)*1e3:.1f} mm de curso. '
            f'{verdict}'
            f'Indentação do HOLD ≈ {indent_m*1e6:.0f} µm (K={k_nm/1e3:.1f} '
            f'N/mm): perdê-la em {d_m*1e3:.1f} mm equivale a uma superfície '
            f'inclinada {slope_deg:.2f}° contra o plano do deslize. '
            f'Corrija calçando a amostra OU informe esse ângulo em '
            f'"Slide Slope" para o SLIDING acompanhar o plano.')


    def _phase_sliding(self) -> str:
        """SLIDING — percorre uma RETA CONTIDA NO PLANO da amostra, travada
        em posição (ver o bloco "Plano do deslize" nas constantes).

        Retorna: 'ok' | 'force' (> 15 N) | 'stale' | 'stop' (usuário)
                 | 'error'.
        """
        self._set_phase('SLIDING')
        self._settle()

        with self._params_lock:
            speed_ms   = max(0.001, self._slide_speed_mms * 1e-3)
            dir_xy     = self._slide_dir_vec.copy()
            slide_lim_m = min(float(self._target_slide_mm) / 1000.0,
                              _SLIDING_SAFETY_M)
            target_f   = float(self._target_force_n)

        frame = self._slide_frame(dir_xy)
        if frame is None:
            return 'error'
        dir_world, perp_dir, plane_n, plane_src = frame
        # A profundidade é medida ao longo da normal: o ataque efetivo do
        # deslize é perpendicular ao plano, por construção. É o que
        # _relieve_contact recebe para recuar na direção certa.
        approach_dir_eff = -plane_n

        T_start = forward_kinematics(self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)
        R0     = T_start[:3, :3].copy()
        p_start = T_start[:3, 3].copy()
        p0_perp = float(p_start @ perp_dir)

        I6 = np.eye(6)
        v_lim = (self._speed_factor_pct / 100.0) * _MAX_JOINT_VEL_RAD_S
        dt = _CTRL_DT

        dist_planned_m   = 0.0   # distância planejada acumulada (não depende de FK)
        step_m = speed_ms * dt   # deslocamento por tick no plano

        self.get_logger().info(
            f'SLIDING: speed={speed_ms*1e3:.1f} mm/s  '
            f'alvo={slide_lim_m*1e3:.0f} mm sobre a superfície  '
            f'reta=({dir_world[0]:+.3f},{dir_world[1]:+.3f},'
            f'{dir_world[2]:+.3f}) no plano [{plane_src}]  '
            f'normal=({plane_n[0]:+.3f},{plane_n[1]:+.3f},{plane_n[2]:+.3f})  '
            f'({target_f:.2f} N no início)')

        watch = self._ContactWatch(target_f, _SLIDE_LOST_BUDGET_M,
                                   _SLIDE_CONTACT_MIN_FRAC)
        lost_reported = False
        outcome = 'ok'
        while True:
            if self._stop_requested.is_set():
                self._stop_requested.clear()
                outcome = 'stop'
                break
            if not self._pause_gate():
                outcome = 'stop'
                break

            t0 = time.time()

            # Verificação 1: distância planejada acumulada (determinístico).
            if dist_planned_m >= slide_lim_m:
                self.get_logger().info(
                    f'SLIDING: {slide_lim_m*1e3:.0f} mm planejados — parando.')
                break

            # Verificação 2: posição real via FK (segurança extra).
            q = self._q_now()
            T_cur = forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)
            progress = float(np.dot(T_cur[:3, 3] - p_start, dir_world))
            if progress >= slide_lim_m:
                self.get_logger().info(
                    f'SLIDING: {slide_lim_m*1e3:.0f} mm (FK) atingidos.')
                break

            # Verificação 3: afundamento máximo abaixo do PLANO. Medir ao
            # longo da normal já desconta a mudança de altura legítima de uma
            # superfície inclinada — que contra o plano horizontal do mundo
            # acusaria afundamento falso.
            sink_m = float(np.dot(T_cur[:3, 3] - p_start, approach_dir_eff))
            if sink_m > _SLIDE_MAX_SINK_M:
                self.get_logger().warn(
                    f'SLIDING: TCP afundou {sink_m*1e3:.1f} mm '
                    f'(> {_SLIDE_MAX_SINK_M*1e3:.0f} mm) abaixo do plano '
                    f'inicial após {progress*1e3:.1f} mm — terminando o '
                    'deslize.')
                break

            # Força só como SEGURANÇA: sem célula fresca não há monitor
            # confiável (aborta); acima da margem, pico da textura mais alto
            # que o plano de contato — trava e cancela.
            if self._force_stale_abort('SLIDING'):
                outcome = 'stale'
                break
            fz_corr = self._fz_corrected()

            if self._force_over_limit(fz_corr):
                self._relieve_contact(approach_dir_eff)   # recua NA HORA
                self.get_logger().error(
                    f'SEGURANÇA: força {fz_corr:+.1f} N além da margem de '
                    f'{_FORCE_SAFE_LIMIT_N:.0f} N (teto {_FORCE_ABORT_LIMIT_N:.0f} N) '
                    f'— medição cancelada.')
                outcome = 'force'
                break

            # Perda de contato: mesma guarda do caminho MovL. Sem ela o
            # deslize percorre o curso inteiro medindo ar e devolve 'ok';
            # em modo AVISO ele percorre igual, mas o log diz que percorreu.
            if watch.exceeded(progress, fz_corr):
                if not lost_reported:
                    lost_reported = True
                    self._slide_lost_contact_report(watch, target_f, progress)
                if _SLIDE_LOST_ABORTS:
                    outcome = 'contact_lost'
                    break

            # ── Rolling-window de _SLIDE_WIN waypoints ──────────────────
            msg = JointTrajectory()
            msg.joint_names = list(_ARM_JOINTS)
            q_iter = q.copy()
            T_iter = T_cur
            singular = False

            for k in range(1, _SLIDE_WIN + 1):
                tw = np.zeros(6)
                # Passo lateral — limita o último passo para não ultrapassar alvo
                remaining = max(0.0, slide_lim_m - dist_planned_m - (k - 1) * step_m)
                lateral   = min(step_m, remaining)
                tw[:3] = dir_world * lateral
                # Lock de orientação
                R_err = R0 @ T_iter[:3, :3].T
                tw[3:] = _ORI_GAIN * 0.5 * np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ])
                # Lock de PROFUNDIDADE, ao longo da normal do plano: mantém a
                # indentação que o HOLD deixou em p_start durante todo o
                # percurso. Como o avanço (dir_world) já está NO plano, não há
                # termo de rampa a compensar — a reta é a própria trajetória.
                depth_err = float(np.dot(
                    p_start - T_iter[:3, 3], approach_dir_eff))
                tw[:3] += _Z_CORR_GAIN * depth_err * approach_dir_eff
                # Lock TRANSVERSAL, ao longo de w (no plano), no mesmo ganho:
                # é o que impede o curso de arquear para fora da direção
                # pedida. Com os dois locks o TCP fica preso à reta.
                perp_err = p0_perp - float(T_iter[:3, 3] @ perp_dir)
                tw[:3] += _Z_CORR_GAIN * perp_err * perp_dir

                J_k = jacobian(q_iter, T_end=T_TOUCH_TOOL_ATTACH)
                try:
                    dq_k = J_k.T @ np.linalg.solve(
                        J_k @ J_k.T + _JAC_LAM**2 * I6, tw)
                except np.linalg.LinAlgError:
                    singular = True
                    break

                q_next = np.clip(q_iter + dq_k, JOINT_MIN, JOINT_MAX)
                vel_k  = np.clip((q_next - q_iter) / dt, -v_lim, v_lim)
                if k == _SLIDE_WIN:
                    vel_k = np.zeros(6)

                pt = JointTrajectoryPoint()
                pt.positions  = [float(x) for x in q_next]
                pt.velocities = [float(x) for x in vel_k]
                t_k = k * dt
                pt.time_from_start = Duration(
                    sec=int(t_k), nanosec=int((t_k - int(t_k)) * 1e9))
                msg.points.append(pt)
                q_iter = q_next
                if k < _SLIDE_WIN:
                    T_iter = forward_kinematics(q_iter, T_end=T_TOUCH_TOOL_ATTACH)

            if singular:
                self.get_logger().warn('SLIDING: Jacobiano singular — passo descartado.')
            elif msg.points:
                self._arm_traj_pub.publish(msg)
                # A janela desliza 1 passo por tick (cada mensagem SUBSTITUI
                # a anterior no JTC — só ~1 segmento executa antes da
                # próxima).
                dist_planned_m += min(step_m,
                                      max(0.0, slide_lim_m - dist_planned_m))

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

        self._settle()
        return outcome

    # RETRACT removido: o experimento (toque e deslizamento) vai DIRETO à
    # home ao terminar e entre ciclos — ver _retreat_and_home e _run_protocol.
    # O _phase_goto_home já afasta da superfície ao subir.

    # Orquestração
    def destroy_node(self):
        self._stop_requested.set()
        if self._protocol_thread is not None:
            self._protocol_thread.join(timeout=2.0)
        super().destroy_node()

    def _unload_before_home(self) -> None:
        """Regra de Ouro: DESCARREGA o contato antes de qualquer movimento
        articular para a HOME.

        `_phase_goto_home` é interpolação NO ESPAÇO DE JUNTAS: partindo de uma
        pose indentada, o caminho até a home não sobe primeiro ao longo da
        normal do contato — ele pode arrastar a ponteira lateralmente sobre a
        amostra com a carga do ensaio (até 10 N) ainda aplicada.

        O MATRIX_MAP já fazia isto explicitamente (_matrix_relieve_and_lift) e
        documentava o motivo; SLIDE, TOUCH e MANUAL terminavam o HOLD no
        setpoint e iam direto para as juntas. Aqui a regra passa a valer nos
        quatro modos, que é o que o comentário do _matrix_relieve_and_lift já
        afirmava ser obrigatório.

        Nunca levanta: um alívio que falha não pode impedir o recuo — o
        movimento para a home é justamente o que tira o braço de lá.
        """
        try:
            fz = self._fz_corrected()
            if abs(fz) <= _CONTACT_ON_N:
                return
            self.get_logger().info(
                f'[HOME] descarregando o contato (fz={fz:+.2f} N) antes do '
                'retorno articular — o caminho até a home não sobe pela '
                'normal e arrastaria a ponteira sob carga.')
            self._relieve_contact(
                self._approach_dir if self._approach_dir is not None
                else np.array([0.0, 0.0, -1.0]),
                floor_n=_CONTACT_ON_N)
        except Exception as exc:      # alívio nunca bloqueia o recuo
            self.get_logger().warn(
                f'[HOME] descarregamento falhou ({exc}) — retorno à home '
                'segue mesmo assim.')

    def _retreat_and_home(self, final_phase: str) -> None:
        """Término com SUCESSO: retorna DIRETO à home lentamente, sem RETRACT."""
        self._unload_before_home()
        self._phase_goto_home()
        self._set_phase(final_phase)

    def _abort_to_home(self) -> None:
        """Falha do experimento (qualquer motivo): sem RETRACT — retorna
        direto à home lentamente (≤ home_speed_rad_s por junta) e marca
        ABORTED."""
        self._unload_before_home()
        self._phase_goto_home()
        self._set_phase('ABORTED')

    def _hold_current_position(self) -> None:
        """FREEZE: segura a posição atual NO LUGAR (sem ir à HOME) — re-publica
        q_now com velocidade zero e o JTC mantém a última pose comandada."""
        self._settle()

    def _finalize_interrupt(self, graceful_phase: str) -> bool:
        """Encerramento por interrupção do usuário. FREEZE congela no lugar
        (fase FROZEN, sem homing); STOP normal recua à HOME (Regra de Ouro) e
        marca `graceful_phase`. Devolve True se congelou."""
        if self._freeze_requested.is_set():
            self._freeze_requested.clear()
            self._hold_current_position()
            self._set_phase('FROZEN')
            return True
        self._retreat_and_home(graceful_phase)
        return False

    # ══════════════════════════════════════════════════════════════════
    # MATRIX_MAP — mapeamento tátil em grade
    # ══════════════════════════════════════════════════════════════════
    #
    # MOVIMENTO CARTESIANO LINEAR entre pontos: `_cartesian_batch_to`, a mesma
    # primitiva do SLIDING, só que em ar livre. Traduz o twist cartesiano em Δq
    # pelo Jacobiano amortecido (`_JAC_LAM`) iterando a FK a cada waypoint, e
    # publica a trajetória inteira numa JointTrajectory para o JTC interpolar.
    # `lock_ori=True` mantém a orientação de ataque; `lock_z=True` (só sobre
    # plano HORIZONTAL) realimenta a altura para o Safe Z não derivar.

    # Margem de relevo da peça assumida entre pontos vizinhos: depois da
    # primeira identação, a descida seguinte só corre em velocidade de
    # slider até (menor contato já visto − esta margem) e RASTEJA o resto.
    _MATRIX_RELIEF_MARGIN_M = 0.003   # 3 mm


    def _tcp_now(self) -> np.ndarray:
        """Posição do TCP no mundo URDF (m), via FK do estado corrente."""
        return forward_kinematics(
            self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)[:3, 3].copy()

    # Quantas vezes um trânsito linear pode ser replanejado depois de uma
    # PAUSA. Segurar posição substitui o goal do JTC, então o que faltava do
    # percurso morre com ele e precisa ser reemitido do ponto onde parou.
    # O teto existe só para que pausar em loop não vire movimento eterno.
    _LINEAR_REPLAN_MAX = 20

    def _move_linear_world(self, delta_m: np.ndarray, v_ms: float, *,
                           lock_z: bool = False,
                           label: str = 'LINEAR') -> str:
        """Translação retilínea do TCP por `delta_m` (vetor XYZ no mundo
        URDF, metros), à velocidade `v_ms`, com a orientação travada.

        Retorna 'done' | 'stop' | 'force' | 'stale' | 'error'.
        """
        d = np.asarray(delta_m, dtype=float).flatten()
        if float(np.linalg.norm(d)) < 1e-5:   # < 10 µm: já está lá
            return 'done'
        # ALVO, não deslocamento: uma pausa no meio do caminho mata o que
        # restava da trajetória, e reemitir `delta_m` a partir de onde o
        # braço parou andaria o percurso duas vezes.
        target = self._tcp_now() + d
        v_ms = max(1e-4, float(v_ms))

        out = 'done'
        for _ in range(self._LINEAR_REPLAN_MAX):
            rest = target - self._tcp_now()
            dist = float(np.linalg.norm(rest))
            if dist < 1e-5:
                out = 'done'
                break
            out = self._cartesian_batch_to(
                rest / dist, dist, v_const_ms=v_ms, lock_ori=True,
                lock_z=lock_z)
            if out != 'paused':
                break
            self.get_logger().info(
                f'[{label}] retomado após pausa — replanejando os '
                f'{dist * 1e3:.1f} mm que faltavam.')
        else:
            self.get_logger().error(
                f'[{label}] {self._LINEAR_REPLAN_MAX} pausas seguidas sem '
                'concluir o trânsito — abortando.')
            return 'error'
        # Rede de segurança do trânsito: o monitor de força de
        # _cartesian_batch_to para o braço no tick em que a carga aparece,
        # mas um contato que se firma DEPOIS do último waypoint só é visto
        # aqui.
        if out == 'done' and self._force_over_limit():
            self.get_logger().error(
                f'[{label}] força {self._fz_corrected():+.1f} N após o '
                'trânsito — contato inesperado em ar livre.')
            return 'force'
        return out


    # ── Geometria da grade: coordenadas DO PLANO, não do mundo ──────────
    # O Safe Z é uma FOLGA MEDIDA AO LONGO DA NORMAL, e os waypoints são
    # deslocamentos NO PLANO da amostra. Enquanto eram altura e XY do mundo,
    # uma amostra inclinada quebrava as duas coisas:
    #
    #  • o plano de trânsito, sendo horizontal, cruzava a superfície que
    #    sobe: a partir de safe_z/tan(θ) morro acima da origem (27,5 mm a
    #    20° com Safe Z de 10 mm) o "trânsito em ar livre" arrastava sobre
    #    a peça;
    #  • a descida ao longo do eixo inclinado desloca o toque lateralmente
    #    por um valor que DEPENDE da altura local da superfície, então a
    #    grade saía comprimida por cos²(θ) (11,7 % a 20°) e deslocada por
    #    sen(θ)cos(θ)·safe_z (3,2 mm) — o relatório mostrava a grade
    #    planejada, que não era a medida.
    #
    # Com a base do plano os dois somem: a folga fica uniforme e o passo da
    # grade é exato. Sem calibração a normal é a vertical do mundo e a base
    # degenera em (x̂, ŷ, ẑ) — byte a byte o comportamento anterior.

    def _matrix_plane_basis(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Base (e1, e2, n) do plano da grade, no mundo URDF.

        n vem do eixo de ataque calibrado; sem calibração é a vertical, e
        e1/e2 caem exatamente em x̂/ŷ. e1 é a projeção de x̂ no plano — uma
        escolha determinística, para que a mesma grade caia sempre nos
        mesmos pontos entre runs.
        """
        attack = self._attack_dir
        n = (_Z_HAT.copy() if attack is None
             else -np.asarray(attack, dtype=float))
        nn = float(np.linalg.norm(n))
        n = _Z_HAT.copy() if nn < 1e-9 else n / nn
        ref = np.array([1.0, 0.0, 0.0])
        if abs(float(ref @ n)) > 0.9:      # x̂ quase paralelo à normal
            ref = np.array([0.0, 1.0, 0.0])
        e1 = ref - float(ref @ n) * n
        e1 /= float(np.linalg.norm(e1))
        e2 = np.cross(n, e1)
        return e1, e2 / float(np.linalg.norm(e2)), n

    def _matrix_safe_pose(self, wp_xy: np.ndarray) -> np.ndarray | None:
        """Ponto de trânsito do waypoint `wp_xy` (coordenadas do plano, m):
        o ponto da grade afastado do plano por Safe Z ao longo da normal."""
        origin = self._matrix_origin
        if origin is None:
            return None
        e1, e2, n = self._matrix_plane_basis()
        return (origin + float(wp_xy[0]) * e1 + float(wp_xy[1]) * e2
                + self._matrix_safe_z_m * n)

    def _lift_to_safe_z(self, *, label: str = 'LIFT') -> str:
        """Afasta o TCP do plano ao longo da NORMAL até a folga do Safe Z."""
        origin = self._matrix_origin
        if origin is None:
            return 'error'
        _e1, _e2, n = self._matrix_plane_basis()
        # Folga corrente = componente normal do vetor origem→TCP.
        gap_m = float((self._tcp_now() - origin) @ n)
        d_m = self._matrix_safe_z_m - gap_m
        if d_m <= 1e-5:
            return 'done'
        # A subida de alívio usa a velocidade de trânsito, não a de descida:
        # sair do contato depressa reduz o tempo sob carga.
        return self._move_linear_world(
            n * d_m, self._matrix_transit_ms, label=label)

    def _move_to_wp_safe(self, wp_xy: np.ndarray) -> str:
        """Trânsito no ar até a pose de Safe Z do waypoint `wp_xy`
        (coordenadas DO PLANO, metros, relativas à origem)."""
        target = self._matrix_safe_pose(wp_xy)
        if target is None:
            return 'error'
        delta = target - self._tcp_now()
        if float(np.linalg.norm(delta)) < 1e-5:
            return 'done'
        self._set_phase('TRANSIT')
        # lock_z trava o Z DO MUNDO, o que só é correto quando o plano é
        # horizontal; num plano inclinado o trânsito muda de altura de
        # propósito e a trava brigaria com o próprio comando.
        _e1, _e2, n = self._matrix_plane_basis()
        horizontal = abs(float(n @ _Z_HAT)) > 1.0 - 1e-9
        return self._move_linear_world(
            delta, self._matrix_transit_ms,
            lock_z=(horizontal and abs(delta[2]) < 1e-4),
            label='TRANSIT')

    def _matrix_find_origin(self) -> str:
        """Descida exploratória inicial: acha o plano e define a ORIGEM.

        Retorna 'ok' | 'no_contact' | 'force' | 'stale' | 'timeout' | 'stop'
                | 'error'.
        """
        # A pose de JOG de onde o MATRIX parte é uma chave de aprendizado como
        # qualquer outra (ver _tcp_key): o índice é POSICIONAL, não "por home".
        # Zerar a chave aqui — o que se fazia por o MATRIX não passar pela HOME
        # — condenava a descida da origem a rastejar o curso inteiro no estágio
        # fino, mesmo com a bancada tendo medido o contato minutos antes pelo
        # modo MANUAL: no run 20260817_152309 o log previu 175 s para os 50 mm
        # a 0,3 mm/s, enquanto o MANUAL na mesma peça ia a 10 mm/s até 7,5 mm.
        #
        # Com a chave preenchida, _lookup_learned reaproveita uma partida
        # VIZINHA dentro de _LEARNED_TCP_TOL_M corrigindo o offset axial
        # exatamente e descontando _LEARNED_FLATNESS_M — o MESMO caminho do
        # MANUAL, com o mesmo vencimento por idade e o mesmo aviso de "se a
        # peça mudou, PARE agora". Sem vizinha conhecida nada muda: a descida
        # exploratória lenta continua sendo o comportamento.
        p_jog = self._tcp_now()
        with self._params_lock:
            self._home_key_cur = self._tcp_key(p_jog)
            self._home_deg_cur = None
            self._learned_contact_m = None
            self._contact_depths.clear()

        out = self._phase_descending()
        if out != 'ok':
            return out
        out = self._phase_hold()
        if out != 'ok':
            return out

        origin = self._tcp_now()
        self._matrix_origin = origin
        with self._params_lock:
            # Travessia seguinte parte do Safe Z, a uma folga CONHECIDA sobre
            # o plano e medida ao longo da normal — que é a MESMA direção da
            # descida. O curso até o contato passa a ser exatamente o Safe Z,
            # em vez de uma projeção que variava com a inclinação.
            fast_m = self._matrix_safe_z_m - self._MATRIX_RELIEF_MARGIN_M
            self._learned_contact_m = fast_m if fast_m > 0.0 else None
            # A origem desceu da pose de JOG, não do Safe Z: a profundidade
            # que ela mediu não é comparável com a dos pontos da grade, e
            # entraria na dispersão como um outlier gigante.
            self._contact_depths.clear()
            # Solta a chave da partida: daqui em diante os pontos descem do
            # Safe Z, e gravar o curso DELES sob a chave da pose de jog
            # corromperia o histórico que a origem acabou de deixar (alturas
            # de partida diferentes). Os pontos seguem aprendendo entre si por
            # _learned_contact_m, que _remember_contact preenche sem chave.
            self._home_key_cur = None
        _e1, _e2, n = self._matrix_plane_basis()
        self.get_logger().info(
            f'[MATRIX] ORIGEM definida no contato: '
            f'x={origin[0]*1e3:+.2f} y={origin[1]*1e3:+.2f} '
            f'z={origin[2]*1e3:+.2f} mm (mundo URDF). Grade no plano de '
            f'normal ({n[0]:+.3f},{n[1]:+.3f},{n[2]:+.3f}), com folga de '
            f'{self._matrix_safe_z_m*1e3:.1f} mm medida ao longo dela.')
        return 'ok'

    def _publish_matrix_point(self, index: int, plan_xy: np.ndarray,
                              tcp: np.ndarray, force_n: float,
                              t_start: float, outcome: str,
                              setpoint_n: float | None = None) -> None:
        """Publica o registro da identação para o palpation_logger.

        `setpoint_n` sobrepõe o setpoint corrente do nó. Existe por causa da
        escada: ao fim dela `_target_force_n` vale o primeiro patamar (a
        escada sobe e volta), e gravar esse valor descreveria a identação pela
        força que ela apenas atravessou. Quem chama passa o PICO.
        """
        origin = (self._matrix_origin if self._matrix_origin is not None
                  else np.zeros(3))
        m = MatrixPoint()
        m.index = int(index)
        m.total = int(self._wp_total)
        m.plan_x_mm = float(plan_xy[0] * 1e3)
        m.plan_y_mm = float(plan_xy[1] * 1e3)
        m.origin_x_m = float(origin[0])
        m.origin_y_m = float(origin[1])
        m.origin_z_m = float(origin[2])
        m.tcp_x_m = float(tcp[0])
        m.tcp_y_m = float(tcp[1])
        m.tcp_z_m = float(tcp[2])
        if setpoint_n is not None:
            m.setpoint_n = float(setpoint_n)
        else:
            with self._params_lock:
                m.setpoint_n = float(self._target_force_n)
        m.force_n = float(force_n)
        # Penetração medida ao longo da NORMAL do plano: num plano inclinado
        # a diferença de Z do mundo mistura a penetração com a mudança de
        # altura legítima da grade.
        _e1, _e2, n = self._matrix_plane_basis()
        m.depth_mm = float(((origin - np.asarray(tcp, float)) @ n) * 1e3)
        m.t_start_unix = float(t_start)
        m.t_end_unix = float(time.time())
        m.outcome = str(outcome)
        self._matrix_pub.publish(m)

    def _matrix_relieve_and_lift(self) -> None:
        """Alívio de emergência do MATRIX_MAP: tira a carga da célula e sobe
        em +Z. Chamado antes de qualquer retorno à HOME — a Regra de Ouro
        exige aliviar o contato ANTES de mover as juntas para a home, porque
        o caminho articular até lá pode passar RASPANDO a peça."""
        try:
            if abs(self._fz_corrected()) > _CONTACT_ON_N:
                # floor_n explícito: com o default (6 N) um contato normal de
                # 2 N já entra abaixo do limiar e o recuo não acontecia — o
                # `if` acima disparava e o alívio não fazia nada.
                self._relieve_contact(
                    self._approach_dir if self._approach_dir is not None
                    else np.array([0.0, 0.0, -1.0]),
                    floor_n=_CONTACT_ON_N)
        except Exception as exc:      # alívio nunca bloqueia o recuo
            self.get_logger().warn(f'[MATRIX] alívio falhou: {exc}')
        if self._matrix_origin is None:
            return
        # STOP já foi consumido pelo chamador; o lift precisa rodar com o
        # evento limpo, senão ele aborta a si mesmo no primeiro tick.
        self._stop_requested.clear()
        out = self._lift_to_safe_z(label='LIFT-ABORT')
        if out != 'done':
            self.get_logger().warn(
                f'[MATRIX] subida de alívio terminou em {out!r} — o retorno '
                'à HOME segue mesmo assim.')

    def _run_matrix_protocol(self) -> None:
        """FSM do MATRIX_MAP.

        Cada ponto da grade é uma identação: DESCENDING → HOLD. Com a escada
        de força configurada, o HOLD de patamar único dá lugar à escada
        completa (sobe de step_start a step_max e volta), medindo a curva de
        carga/descarga EM CADA COORDENADA — é o mapa de histerese da peça.
        """
        with self._params_lock:
            wps = self._matrix_wps.copy()
            target_f = float(self._target_force_n)
            st_start = float(self._step_start_n)
            st_size = float(self._step_size_n)
            st_max = float(self._step_max_n)
            st_dwell = float(self._step_dwell_s)
        self._wp_total = int(len(wps))
        self._wp_index = 0
        self._matrix_origin = None
        self._cycles_total = self._wp_total
        self._cycle = 0

        # ── Escada de força por ponto ────────────────────────────────
        # Resolvida ANTES de qualquer movimento, como no MANUAL: ela decide a
        # força da descida de cada ponto e pode recusar o ensaio inteiro.
        levels = (staircase_levels(st_start, st_size, st_max)
                  if st_size > 0.0 and st_max > st_start else [])
        # Setpoint que RESUME o ponto no matrix.csv. Com escada, o número que
        # descreve a identação é o PICO percorrido — o valor corrente de
        # _target_force_n ao fim dela é o primeiro patamar (a escada volta), e
        # gravar isso faria a matriz parecer medida a uma força que ela só
        # tocou de passagem. A curva completa continua no samples.csv, ponto a
        # ponto, via wp_index + setpoint_n.
        point_sp_n = max(levels) if levels else target_f
        if st_size > 0.0 and st_max > st_start and not levels:
            self.get_logger().error(
                f'[MATRIX] escada de {st_start:.2f} a {st_max:.2f} N em passos '
                f'de {st_size:.3f} N excede {STEP_MAX_LEVELS} patamares — '
                'aumente o passo. Ensaio recusado.')
            self._set_phase('ABORTED')
            return
        if levels:
            # O produto N x M x dwell cresce depressa e o operador precisa
            # saber ANTES: uma grade 5x5 com 19 patamares de 5 s já passa de
            # 40 min só de dwell, sem contar acomodação e trânsito.
            _min_s = self._wp_total * len(levels) * st_dwell
            self.get_logger().info(
                f'[MATRIX] escada ATIVA em cada ponto: {len(levels)} patamares '
                f'de {levels[0]:.2f} a {max(levels):.2f} N e de volta, '
                f'{st_dwell:.1f} s cada. {self._wp_total} pontos x '
                f'{len(levels)} patamares = pelo menos '
                f'{_min_s / 60.0:.0f} min só de dwell, fora acomodação e '
                'trânsito. A curva de cada ponto sai do samples.csv filtrando '
                'por wp_index e setpoint_n.')

        # A sonda começa onde o usuário a deixou — NÃO passamos pela HOME,
        # que é onde os outros modos calibram o frame mundo→DOBOT.
        # Mão na pose de palpação (Index estendido), como faz a HOME.
        self._send_hand_pose(_HAND_POINTING_RAD)
        self._settle(ticks=_SETTLE_TICKS * 3)

        # ── 0. Ângulo de ataque ──────────────────────────────────────
        # Roda ANTES da descoberta da origem para que a origem — e portanto
        # toda a grade referida a ela — já nasça do ataque alinhado.
        out = self._phase_calibrate_attack()
        if out != 'ok':
            self._matrix_relieve_and_lift()
            if out == 'stop':
                self._finalize_interrupt('ABORTED')
            else:
                self._abort_to_home()
            return

        # ── 1. Descoberta da origem ──────────────────────────────────
        # A origem estabelece o PLANO de onde saem o Safe Z e todas as
        # penetrações. Com escada, ela é medida no PRIMEIRO PATAMAR e com hold
        # simples: é geometria de referência, não um ponto de medição, e
        # levá-la ao pico da escada deformaria justamente o zero de todo o
        # resto. Os pontos da grade é que percorrem a escada.
        if levels:
            with self._params_lock:
                self._target_force_n = float(levels[0])
            self.get_logger().info(
                f'[MATRIX] origem estabelecida no primeiro patamar '
                f'({levels[0]:.2f} N), sem escada — ela define o plano de '
                'referência, e medir o zero sob carga de pico deslocaria '
                'todas as penetrações da grade.')
        self.get_logger().info(
            '[MATRIX] procurando o plano: descida exploratória a partir da '
            'pose de jog. O contato define a ORIGEM (0,0,0).')
        t_origin = time.time()
        out = self._matrix_find_origin()
        if out == 'ok':
            self._publish_matrix_point(
                0, np.zeros(2), self._tcp_now(), self._fz_corrected(),
                t_origin, 'origin')
        elif out in ('force', 'no_contact', 'stale', 'timeout', 'error', 'target_lost'):
            self.get_logger().error(
                f'[MATRIX] falha ao encontrar a origem ({out}) — abortando.')
            self._matrix_relieve_and_lift()
            self._abort_to_home()
            return
        else:                                   # 'stop' — STOP/FREEZE
            self._matrix_relieve_and_lift()
            self._finalize_interrupt('ABORTED')
            return

        # ── 2. Sobe ao Safe Z antes do primeiro trânsito ─────────────
        out = self._lift_to_safe_z(label='LIFT-ORIGIN')
        if out != 'done':
            self._matrix_relieve_and_lift()
            if out == 'stop':
                self._finalize_interrupt('ABORTED')
            else:
                self._abort_to_home()
            return

        # ── 3. Laço da matriz ────────────────────────────────────────
        for i, wp in enumerate(wps, start=1):
            self._wp_index = i
            self._cycle = i
            self._wp_target = np.asarray(wp, dtype=float).copy()
            self.get_logger().info(
                f'[MATRIX] ponto {i}/{self._wp_total} — alvo '
                f'({wp[0]*1e3:+.1f}, {wp[1]*1e3:+.1f}) mm da origem, '
                + (f'escada {levels[0]:.2f} → {max(levels):.2f} N '
                   f'({len(levels)} patamares).' if levels
                   else f'setpoint {target_f:.2f} N.'))

            # 3a. Trânsito no ar até a pose de Safe Z do waypoint —
            # deslocamento NO PLANO, folga ao longo da normal.
            out = self._move_to_wp_safe(np.asarray(wp, float))
            if out != 'done':
                self._matrix_relieve_and_lift()
                if out == 'stop':
                    self._finalize_interrupt('ABORTED')
                else:
                    self._abort_to_home()
                return

            # 3b. Identação: DESCENDING → HOLD (patamar único) ou ESCADA.
            # As fases são as MESMAS dos outros modos — nem o regulador de
            # força nem a escada são reimplementados aqui.
            #
            # Com escada, a descida vai ao PRIMEIRO PATAMAR e não ao setpoint
            # da GUI: descer ao pico antes do degrau 1 transformaria o
            # primeiro degrau num descarregamento e pré-condicionaria a
            # amostra — o mesmo erro que o modo MANUAL tinha.
            if levels:
                with self._params_lock:
                    self._target_force_n = float(levels[0])
            t_start = time.time()
            out = self._phase_descending()
            if out != 'ok':
                self._publish_matrix_point(
                    i, wp, self._tcp_now(), self._fz_corrected(),
                    t_start, out, setpoint_n=point_sp_n)
                self._matrix_relieve_and_lift()
                if out == 'stop':
                    self._finalize_interrupt('ABORTED')
                else:
                    self._abort_to_home()
                return

            out = (self._phase_hold_staircase(levels, st_dwell) if levels
                   else self._phase_hold())
            tcp_touch = self._tcp_now()
            fz_touch = self._fz_corrected()
            self._publish_matrix_point(i, wp, tcp_touch, fz_touch,
                                       t_start, out, setpoint_n=point_sp_n)
            if out != 'ok':
                self._matrix_relieve_and_lift()
                if out == 'stop':
                    self._finalize_interrupt('ABORTED')
                else:
                    self._abort_to_home()
                return
            _e1, _e2, _n = self._matrix_plane_basis()
            _pen_mm = float(((self._matrix_origin - tcp_touch) @ _n) * 1e3)
            self.get_logger().info(
                f'[MATRIX] ponto {i}/{self._wp_total} OK — '
                + (f'escada de {len(levels)} patamares até '
                   f'{max(levels):.2f} N concluída, F final {fz_touch:.2f} N'
                   if levels else
                   f'F={fz_touch:.2f} N (alvo {target_f:.2f} N)')
                + f', penetração {_pen_mm:+.3f} mm sob o plano da origem.')

            # 3c. Volta ao Safe Z antes do próximo trânsito.
            out = self._lift_to_safe_z(label='LIFT-WP')
            if out != 'done':
                self._matrix_relieve_and_lift()
                if out == 'stop':
                    self._finalize_interrupt('ABORTED')
                else:
                    self._abort_to_home()
                return

        # ── 4. Fim da matriz: Regra de Ouro (volta à HOME articular) ──
        self.get_logger().info(
            f'[MATRIX] matriz concluída: {self._wp_total} identações'
            + (f', cada uma com a escada completa de {len(levels)} patamares '
               f'até {max(levels):.2f} N.' if levels
               else f' a {target_f:.2f} N.'))
        self._wp_index = 0
        self._retreat_and_home('DONE')
        time.sleep(0.5)
        self._set_phase('IDLE')

    def _run_protocol(self):
        self._busy.set()
        # Ângulo de ataque volta à vertical a cada experimento, e o plano do
        # deslize volta à inclinação declarada: a amostra pode ter sido
        # trocada, e uma medição do run anterior não vale para ela. Se a
        # calibração estiver ligada, ela remede.
        self._attack_dir = None
        with self._params_lock:
            self._slide_plane_n = None
        # Snapshot do modo MovL para TODO o experimento — não muda no meio
        # de uma fase se a GUI desconectar (as fases abortam por timeout).
        try:
            with self._params_lock:
                repeats = int(self._repeats)
                mode = self._mode
                _tgt = float(self._target_force_n)
                _tol_ovr = self._hold_tol_n
            self._cycles_total = repeats
            # Uma vez por experimento: o setpoint pedido é distinguível do
            # limiar de contato? Só avisa — a decisão é do operador.
            _tol_run = (_tol_ovr if _tol_ovr is not None
                        else max(_HOLD_TOL_N, _HOLD_TOL_PCT * _tgt))
            _ok_sp, _porque_sp = setpoint_resolvable(_tgt, _tol_run)
            if not _ok_sp:
                self.get_logger().warn(f'[SETPOINT] {_porque_sp}')
            # Recurso PEDIDO que este modo não executa: avisa uma vez, aqui,
            # antes de qualquer movimento. A GUI já zera os campos fora dos
            # modos que os rodam, mas ela não é o único publisher — por
            # `ros2 topic pub` um perfil de onda ou uma escada chegavam e
            # eram descartados sem uma linha de log, e o ensaio saía estático
            # com o operador achando que tinha medido a onda.
            with self._params_lock:
                _step_pedido = float(self._step_size_n) > 0.0
            if _step_pedido and mode not in STAIRCASE_MODES:
                self.get_logger().warn(
                    f'[DEGRAU] escada configurada, mas o modo {mode} identa '
                    f'com força CONSTANTE: ela NÃO vai rodar. A escada existe '
                    f'em {" e ".join(STAIRCASE_MODES)}.')
            if self._fmod_configured() and mode not in FMOD_MODES:
                self.get_logger().warn(
                    f'[FMOD] perfil trigonométrico configurado, mas o modo '
                    f'{mode} identa com força CONSTANTE: a onda NÃO vai '
                    f'rodar. A modulação existe em '
                    f'{" e ".join(FMOD_MODES)} — desligue-a para silenciar '
                    'este aviso.')

            # Em TOUCH, cada "ciclo" é um toque (descida → hold → recuo).
            label = 'TOQUE' if mode == 'TOUCH' else 'CICLO'

            # ── MATRIX_MAP: grade de identações a partir de uma origem
            # descoberta pelo próprio robô. NÃO passa pela HOME no início —
            # a pose de partida é a que o usuário deixou no jog manual, logo
            # acima do primeiro ponto.
            if mode == 'MATRIX_MAP':
                # A escada por ponto reescreve _target_force_n a cada patamar;
                # restaurar aqui cobre TODOS os caminhos de saída da matriz
                # sem reindentar a FSM inteira num try/finally.
                with self._params_lock:
                    _f_user_matrix = float(self._target_force_n)
                try:
                    self._run_matrix_protocol()
                finally:
                    with self._params_lock:
                        self._target_force_n = _f_user_matrix
                return

            # ── Modo MANUAL/DINÂMICO: HOME → DESCENDING → HOLD INFINITO com
            # setpoint atualizável (/palpation/set_force). Encerra em STOP
            # (→ HOME, Regra de Ouro), FREEZE (→ congela no lugar) ou
            # force/stale (→ recuo + HOME).
            if mode == 'MANUAL':
                self._cycles_total = 1
                self._cycle = 1
                # A escada é resolvida ANTES de qualquer movimento: ela decide
                # a força da DESCIDA (abaixo) e pode recusar o ensaio. Fazer
                # isso depois de descer, como antes, significava identar a
                # amostra para só então dizer que o ensaio era inválido.
                with self._params_lock:
                    st_start = float(self._step_start_n)
                    st_size = float(self._step_size_n)
                    st_max = float(self._step_max_n)
                    st_dwell = float(self._step_dwell_s)
                levels = (staircase_levels(st_start, st_size, st_max)
                          if st_size > 0.0 and st_max > st_start else [])
                if st_size > 0.0 and st_max > st_start and not levels:
                    # Escada pedida excede o teto de patamares. Truncar
                    # produziria um salto de vários newtons até o pico.
                    self.get_logger().error(
                        f'[DEGRAU] passo {st_size:.3f} N de {st_start:.2f} a '
                        f'{st_max:.2f} N excede {STEP_MAX_LEVELS} patamares — '
                        'aumente o passo. Ensaio recusado.')
                    self._set_phase('ABORTED'); return
                if not self._phase_goto_home():
                    self._finalize_interrupt('ABORTED'); return
                if not self._calibrate_or_abort():
                    return
                # Com escada, a descida vai ao PRIMEIRO PATAMAR, não ao
                # setpoint da GUI. Descer ao setpoint (tipicamente o pico da
                # escada) levava a amostra ao topo da carga ANTES do primeiro
                # degrau: o "degrau 1" virava um DESCARREGAMENTO, e a curva de
                # carga era medida num material já pré-condicionado no pico —
                # efeito Mullins em silicone. Como a razão de existir da
                # ida-e-volta é justamente medir histerese/relaxação, isso
                # contaminava o único resultado do ensaio.
                f_user_run: float | None = None
                if levels:
                    with self._params_lock:
                        f_user_run = float(self._target_force_n)
                        self._target_force_n = float(levels[0])
                    self.get_logger().info(
                        f'[DEGRAU] descida ao primeiro patamar '
                        f'({levels[0]:.2f} N) em vez do setpoint da GUI '
                        f'({f_user_run:.2f} N) — a escada mede a curva de '
                        'carga desde o pé, e passar pelo pico antes '
                        'pré-condicionaria a amostra.')
                try:
                    out = self._phase_descending()
                    if out in ('force', 'no_contact', 'stale', 'error', 'target_lost'):
                        self._abort_to_home(); return
                    if out != 'ok':   # STOP/FREEZE durante a descida
                        self._finalize_interrupt('ABORTED'); return
                    # Escada configurada ⇒ percorre os patamares sozinho e
                    # termina; senão, HOLD infinito de sempre (bloqueia até
                    # STOP/force/stale, com o setpoint vindo do tópico).
                    if levels:
                        out = self._phase_hold_staircase(levels, st_dwell)
                    else:
                        out = self._phase_hold_dynamic()
                finally:
                    # Devolve o setpoint do usuário ao status/CSV em qualquer
                    # saída — inclusive nos returns acima.
                    if f_user_run is not None:
                        with self._params_lock:
                            self._target_force_n = f_user_run
                if out in ('force', 'stale', 'target_lost'):
                    self._abort_to_home(); return
                # STOP normal → HOME (DONE); FREEZE → congela (FROZEN).
                if not self._finalize_interrupt('DONE'):
                    time.sleep(0.5)
                    self._set_phase('IDLE')
                return

            for cycle in range(1, repeats + 1):
                self._cycle = cycle
                if repeats > 1:
                    self.get_logger().info(
                        f'[{label}] {cycle}/{repeats}')

                if not self._phase_goto_home():
                    # FREEZE também chega aqui (ele seta _stop_requested para
                    # quebrar os laços): sem passar pelo _finalize_interrupt a
                    # fase saía ABORTED e o E-STOP não aparecia nem no status
                    # nem no CSV.
                    self._finalize_interrupt('ABORTED'); return

                # A calibração mede o plano UMA vez por experimento: a peça
                # não se move entre repetições, e refazê-la a cada ciclo
                # gastaria N identações extras na amostra por ciclo.
                #
                # O EIXO sobrevive ao retorno à home, mas a ORIENTAÇÃO não:
                # a HOME é uma pose articular e devolve a ferramenta à
                # vertical. Por isso os ciclos seguintes reaplicam a rotação
                # em vez de re-sondar.
                if cycle == 1:
                    if not self._calibrate_or_abort():
                        return
                elif not self._reapply_attack_orientation():
                    self._abort_to_home(); return

                out = self._phase_descending()
                if out in ('force', 'no_contact', 'stale', 'error', 'target_lost'):
                    self._abort_to_home(); return
                if out != 'ok':   # STOP → HOME (Regra de Ouro) · FREEZE → congela
                    self._finalize_interrupt('ABORTED'); return

                # TOUCH com perfil trigonométrico configurado: o HOLD de força
                # constante dá lugar ao HOLD modulado (que começa fazendo o
                # HOLD normal na força média). Qualquer outro modo, ou perfil
                # OFF, segue no caminho de sempre.
                fmod = (self._force_profile()
                        if mode in FMOD_MODES else None)
                out = (self._phase_hold_modulated(fmod) if fmod is not None
                       else self._phase_hold())
                if out in ('force', 'stale', 'timeout', 'error', 'target_lost'):
                    self._abort_to_home(); return
                if out != 'ok':   # STOP → HOME (Regra de Ouro) · FREEZE → congela
                    self._finalize_interrupt('ABORTED'); return

                # Modo TOUCH: só toca a mesa com força controlada (DESCENDING
                # + HOLD) e recua — sem deslizamento lateral.
                if mode != 'TOUCH':
                    out = self._phase_sliding()
                    if out in ('force', 'error', 'stale', 'contact_lost', 'target_lost'):
                        self._abort_to_home(); return
                    if out != 'ok':   # STOP → HOME (Regra de Ouro) · FREEZE → congela
                        self._finalize_interrupt('ABORTED'); return

                if cycle < repeats:
                    # Entre ciclos (coleta de dados): vai DIRETO à home, sem
                    # RETRACT — o HOME já afasta da superfície ao subir, e o
                    # próximo ciclo refaz a re-aproximação a partir da home.
                    if not self._phase_goto_home():
                        self._finalize_interrupt('ABORTED'); return
                    # Stop pedido durante o retorno → não inicia o próximo.
                    if self._stop_requested.is_set():
                        self._stop_requested.clear()
                        self._set_phase('ABORTED'); return

            self._retreat_and_home('DONE')
            time.sleep(0.5)
            self._set_phase('IDLE')
        except BaseException as exc:
            # Rede de segurança: SEM isto, qualquer exceção não prevista (uma
            # divisão degenerada no estimador, um NaN vindo da IK) matava esta
            # thread com a ponteira DENTRO da amostra — sem alívio, sem
            # retorno à home e sem publicar ABORTED, então a GUI seguia
            # mostrando HOLD enquanto o _busy era liberado e um novo Start
            # era aceito com o braço ainda em contato.
            self.get_logger().error(
                f'[FSM] exceção não tratada no protocolo ({type(exc).__name__}: '
                f'{exc}) — aliviando o contato e recuando à home. Isto é um '
                'BUG: reporte o traceback abaixo.')
            self.get_logger().error(traceback.format_exc())
            try:
                self._abort_to_home()
            except Exception:
                # O recuo falhou também: não há mais nada a fazer aqui além
                # de deixar a fase honesta para a GUI e o CSV.
                self._set_phase('ABORTED')
            raise
        finally:
            self._cycle = 0
            self._cycles_total = 1
            self._wp_index = 0
            self._busy.clear()


def main(args=None):
    rclpy.init(args=args)
    node = TactileExplorer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
