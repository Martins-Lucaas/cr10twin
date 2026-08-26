"""
palpation_gui.py — Painel Tkinter (tema claro) da célula de palpação tátil.

Funcionalidades:
  • Spinbox + Slider sincronizados para Velocidade / Força.
  • Botão "▶ Iniciar Palpação" (publica /palpation/start).
  • Feedback em tempo real da célula de carga (/load_cell/force_net) e do
    touch sensor (/touch_sensor/value) — semáforo OK/WARN/DANGER + sparklines.
  • Indicador de fase (IDLE/CONTACT/CALIBRATING/SLIDING/RETRACT/DONE/ABORTED).
  • Painel de conexão à MÃO COVVI real (IP + Conectar + ECI + PWR)
    — sobe o subprocesso `covvi_hand_driver server <IP>` e ativa o ECI.
  • Painel de conexão ao ROBÔ CR10 real (IP + Conectar + dropdown de modo
    SIM_ONLY / MIRROR / REAL_FROM_SIM) — abre as 3 sockets TCP do
    controlador e executa a sequência ClearError + EnableRobot.
  • Botão ■ E-STOP — chama StopRobot+DisableRobot e abre a mão.

Comunicação ROS:
  pub  /palpation/start    std_msgs/String   JSON {depth_mm, speed_mms, slide_dir}
  sub  /palpation/status   std_msgs/String   JSON {phase, measured_force_normal_n,...}
  sub  /load_cell/force_net  std_msgs/Float32  força tare-compensada (painel)
  sub  /touch_sensor/value   std_msgs/Float32  touch sensor STM32 (painel)
  pub  /ft_sensor/wrench   geometry_msgs/WrenchStamped (bridge do CR10 real)
  cli  covvi_interfaces/SetCurrentGrip   (lazy)
  cli  covvi_interfaces/SetHandPowerOn   (lazy)
  cli  covvi_interfaces/SetHandPowerOff  (lazy)
"""
from __future__ import annotations

import collections
import csv
import json
import queue as _queue
import logging
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

import numpy as np
if tuple(int(x) for x in np.__version__.split(".")[:2]) >= (2, 0):
    sys.exit(
        f"[ERRO] NumPy {np.__version__} detectado — ABI incompatível com "
        "ROS 2 Humble / cv_bridge.\n"
        "Corrija: pip install 'numpy<2'\n"
        "Confirme com: python3 -c \"import numpy; print(numpy.__version__)\""
    )
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)

from std_msgs.msg import String, Float32, Bool, Empty
from geometry_msgs.msg import WrenchStamped, Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from touch_pack_msgs.msg import (
    PalpationStart, PalpationStatus, LoadCellSample, TouchFrame)

# QoS para comando crítico (/palpation/start): RELIABLE + TRANSIENT_LOCAL
# faz com que o último start fique persistido — se o explorer subir
# depois da GUI publicar, ele ainda recebe o último comando.
QOS_COMMAND = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
# QoS para stream de sensor (/ft_sensor/wrench): BEST_EFFORT + depth=1
# minimiza latência e nunca trava o publisher por reentrega — só o
# pacote mais recente importa para o controle de força.
QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)

# Fases em que o run ACABOU — o cronômetro e o odômetro param nelas. FROZEN
# (o E-STOP congela no lugar, sem ir à HOME) entra aqui: enquanto ficava de
# fora, o cronômetro contava para sempre depois de um E-STOP.
# Não confundir com a lista de fases INATIVAS do espelhamento
# (mirror_node._IDLE_PHASES): num FROZEN o braço continua sob comando do
# explorer, então o espelho segue ativo.
_PHASE_ENDED = ('IDLE', 'DONE', 'ABORTED', 'FROZEN')

# Constantes compartilhadas (fonte única GUI ↔ explorer ↔ nós auxiliares).
from .constants import (
    taxel_frame_to_physical,
    taxel_index_to_physical,
    ARM_JOINTS, HAND_JOINTS, HAND_POINT_DEG, POINTING_SEED_DEG,
    FORCE_ABORT_LIMIT_N as _FORCE_ABORT_LIMIT_N,
    CONTACT_ON_N as _CONTACT_ON_N,
    CONTACT_OFF_FRAC as _CONTACT_OFF_FRAC,
    FORCE_NOISE_SIGMA_N as _FORCE_NOISE_SIGMA_N,
    HOLD_TOL_N as _HOLD_TOL_N,
    HOLD_TOL_PCT as _HOLD_TOL_PCT,
    HOLD_TOL_SIGMA as _HOLD_TOL_SIGMA,
    hold_tol_n as _hold_tol_n_for,
    FORCE_SETPOINT_MAX_N,
    HOME_POSE_FILE, ROBOT_CONFIG_FILE, POSES_FILE,
    tool_stamp, tool_stamp_mismatch,
    PALPATION_PARAMS_FILE, RUNS_DIR,
    run_dir, new_run_id,
    RUN_SENSORS_CSV, RUN_ADC_CSV, RUN_SPIKES_CSV, RUN_CN_CSV,
    TOUCH_FRAME_TOPIC, TOUCH_EVENT_TOPIC,
    MATRIX_SAFE_Z_MM_DEFAULT, MATRIX_SAFE_Z_MM_MIN, MATRIX_SAFE_Z_MM_MAX,
    MATRIX_TRANSIT_MMS_DEFAULT, MATRIX_TRANSIT_MMS_MIN,
    MATRIX_TRANSIT_MMS_MAX, MATRIX_SPAN_MAX_MM,
    PROBE_ALIGN_POINTS_DEFAULT, PROBE_ALIGN_POINTS_MIN,
    PROBE_ALIGN_POINTS_MAX, PROBE_ALIGN_RADIUS_MM_DEFAULT,
    PROBE_ALIGN_RADIUS_MM_MIN, PROBE_ALIGN_RADIUS_MM_MAX,
    PROBE_ALIGN_FORCE_N_DEFAULT, PROBE_ALIGN_RETRACT_MM_DEFAULT,
    PROBE_ALIGN_RETRACT_MM_MIN, PROBE_ALIGN_RETRACT_MM_MAX,
    PROBE_ALIGN_TILT_MAX_DEG_DEFAULT, PROBE_ALIGN_TILT_HARD_MAX_DEG,
    STEP_MAX_LEVELS, staircase_levels,
)

# Driver TCP/IP do CR10 real (cabeada via 192.168.5.1 / LAN1).
try:
    from .real_driver import (
        CR10RealDriver, CR10RealDriverConfig, CR10RealDriverError,
    )
    from .kinematics import urdf_to_dobot as _urdf_to_dobot, MIMIC_LIST
    from .kinematics import fk_partial as _fk_partial
    from .kinematics import JOINT_MIN as _JOINT_MIN, JOINT_MAX as _JOINT_MAX
    _REAL_DRIVER_OK = True
    # FK do TCP para o odômetro do painel (distância percorrida). Importada
    # à parte do bloco acima: o painel tem de funcionar em simulação pura,
    # onde o real_driver pode nem importar.
except Exception:  # pragma: no cover
    CR10RealDriver = None
    CR10RealDriverConfig = None
    CR10RealDriverError = Exception
    _urdf_to_dobot = None
    MIMIC_LIST = []
    _fk_partial = None
    _JOINT_MIN = None
    _JOINT_MAX = None
    _REAL_DRIVER_OK = False

try:
    from .kinematics import (
        forward_kinematics as _fk_tcp,
        T_TOUCH_TOOL_ATTACH as _T_TCP,
        T_HAND_ATTACH as _T_HAND,
    )
except Exception:  # pragma: no cover
    _fk_tcp = None
    _T_TCP = None
    _T_HAND = None

# N → kgf: o painel mostra as DUAS unidades ao mesmo tempo. kgf (e não "kg")
# porque a célula mede FORÇA; é o número que uma balança marcaria sob a mesma
# carga. Mesma constante da calibração (massas padrão → N).
_N_PER_KGF = 9.80665


# Tema + widgets compartilhados (cores, named fonts do Tk — ver o aviso
# sobre o bug do fontconfig em ui_helpers — tooltip e botões do header).
from .gui_loadcell import FtAxesMixin
from .gui_matrix import MatrixMixin
from .gui_constants import (
    MATRIX_STEP_DEFAULT, MATRIX_N_DEFAULT, MATRIX_SHAPES,
    MATRIX_SIZING_MODES, MATRIX_SIZE_DEFAULT, MATRIX_PATH_ORDERS,
    ARM_LIMITS_DEG, MANIP_TRAJ_DURATION_S,
    FORCE_SP_MIN, FORCE_SP_MAX, FORCE_SP_DEFAULT,
)
from .ui_helpers import (
    BG, PANEL, HEADER, HEADER_FG, TEXT, TEXT_MUTED, TEXT_DIM,
    PRIMARY, PRIMARY_HV, OK, WARN, DANGER, DANGER_HV, BORDER, BTN_NEUTRAL,
    FONT_TITLE, FONT_HEAD, FONT_LBL, FONT_SMALL, FONT_BIG,
    FONT_MONO, FONT_MONO_S,
    _shade, _Tooltip, _hdr_btn,
)

# Viewport 3D + IK diferencial (aba "3D Manipulation"). Guardado como os
# demais opcionais: sem numpy/kinematics a aba some e o resto da GUI segue.
try:
    from .manip3d import (
        Manip3DView, rpy_deg as _rpy_deg,
        MAX_LIN_STEP_M as _MANIP_MAX_LIN_M,
        MAX_JOINT_STEP_RAD as _MANIP_MAX_DQ,
    )
    _MANIP3D_OK = True
except Exception:  # pragma: no cover
    Manip3DView = None
    _rpy_deg = None
    _MANIP_MAX_LIN_M = 0.015
    _MANIP_MAX_DQ = 0.06
    _MANIP3D_OK = False

# Cena de malhas do URDF real da célula (mesmas STL do Gazebo).
try:
    from .urdf_scene import (
        build_scene as _build_scene,
        coarse_scene as _coarse_scene,
        DEFAULT_TRIANGLE_BUDGET as _SCENE_BUDGET,
    )
    _URDF_SCENE_OK = True
except Exception:  # pragma: no cover
    _build_scene = None
    _coarse_scene = None
    _SCENE_BUDGET = 5000
    _URDF_SCENE_OK = False

try:
    from .vtk_render import vtk_available as _manip_vtk_available
except Exception:  # pragma: no cover
    def _manip_vtk_available() -> bool:
        return False

# Fonte serial do touch sensor + figura matplotlib reaproveitável.
try:
    from .touch_source import (
        TouchSensorSource, TouchFigure,
        ROWS as TOUCH_ROWS, COLS as TOUCH_COLS, NUM_TAXELS as TOUCH_TAXELS,
    )
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.animation import FuncAnimation
    _TOUCH_PLOT_OK = True
except Exception:  # pragma: no cover
    TouchSensorSource = None
    TouchFigure = None
    FigureCanvasTkAgg = None
    FuncAnimation = None
    TOUCH_ROWS = TOUCH_COLS = 4
    TOUCH_TAXELS = 16
    _TOUCH_PLOT_OK = False

# A GUI NÃO ABRE a serial da célula — uma tty admite um leitor só, e o
# force_receiver é o dono exclusivo da porta.

log = logging.getLogger('touch_pack.palpation_gui')


def _rel_run(path: str | None) -> str:
    """<MODO>/<run_id>/<arquivo> para mostrar na barra de status — o
    basename sozinho ('sensors.csv') não diz de qual run se trata."""
    if not path:
        return '?'
    try:
        return os.path.relpath(path, RUNS_DIR)
    except (ValueError, TypeError):
        return os.path.basename(path)

# Regex p/ os CSVs "crus" (ADC, spikes RA/SA, cuneiformes) — gravados junto
# do sensors.csv quando o usuário aperta "Salvar dados".
_REF_SPIKE_RE = re.compile(r"idx=(\d+),adc=(\d+),t=(\d+)")
_REF_T_RE = re.compile(r"t=(\d+)")

# Faixas dos parâmetros — adequadas ao protocolo Gupta et al. 2021.
SPEED_MIN, SPEED_MAX, SPEED_DEFAULT = 1.0,  30.0,  10.0    # mm/s
FORCE_MIN, FORCE_MAX, FORCE_DEFAULT = 0.2,   5.0,   1.0    # N (apenas display)
# Velocidade da DESCIDA no ar livre (fase PROBE), em mm/s.
APPROACH_MIN, APPROACH_MAX, APPROACH_DEFAULT = 1.0, 30.0, 8.0   # mm/s

SPEED_FACTOR_MIN, SPEED_FACTOR_MAX, SPEED_FACTOR_DEFAULT = 1, 100, 10  # %
# At SPEED_FACTOR_DEFAULT (10 %), Gazebo trajectory duration = 3.0 s.
_VEL_BASE_S = 3.0   # duration at 10 %

# Curso máximo da descida — o término é por força; isto é só segurança.
DEPTH_MIN,  DEPTH_MAX,  DEPTH_DEFAULT  = 0.0, 120.0,  5.0   # mm
# Repetições automáticas do experimento (ciclos descida→deslizamento→recuo).
REPEAT_MIN, REPEAT_MAX, REPEAT_DEFAULT = 1, 50, 1
SLIDE_DIST_MIN, SLIDE_DIST_MAX, SLIDE_DIST_DEFAULT = 1.0, 300.0, 50.0  # mm
# Controle de força: setpoint selecionável (máx. FORCE_SETPOINT_MAX_N); a
# medição é cancelada se a compressão exceder FORCE_ABORT_LIMIT_N — ambos
# vêm de constants.py (fonte única com o explorer).

# ── FORÇA MODULADA (modo TOUCH) — faixas do painel ─────────────────────
# Formas aceitas; espelham _FMOD_SHAPES do tactile_explorer.
FMOD_SHAPES = ('OFF', 'SINE', 'COSINE')
# Frequência da onda. O teto é do FIRMWARE, não do painel: o `t` do ServoJ
# tem faixa [0.02, 3600] s ("Dobot TCP/IP Remote Control Interface Guide
# V4.5.1"), e com os 5 pontos por período que a onda exige isso trava a
# frequência máxima em 1/(0,02 x 5) = 10,0 Hz.
#
# O teto anterior era 30 Hz. Ele nunca foi alcançável: pedir 24 Hz exigiria
# ServoJ com t = 5,2 ms, que o controlador recusa — e o conselho que o
# explorer dava ("suba o mirror com servoj_period_s:=<1/(f*8)>") mandava
# configurar justamente um valor inválido. Deixar pedir o impossível não é
# permissividade: é gravar um CSV com uma frequência que não aconteceu.
#
# De 6,25 para 10,0 Hz: o que mudou não foi o firmware, foi o mínimo de
# pontos por período (8 → 5, ver _FMOD_MIN_PTS_PER_CYCLE). 10 Hz SÓ acontece
# com o mirror_node em servoj_period_s:=0.020; com os 30 ms padrão o teto
# continua sendo 6,7 Hz e o explorer recusa o resto.
FMOD_HZ_MIN, FMOD_HZ_MAX = 0.1, 10.0
# Períodos por toque; duração da oscilação = cycles / hz.
FMOD_CYCLES_MIN, FMOD_CYCLES_MAX, FMOD_CYCLES_DEFAULT = 1, 200, 20
# Espelham _CTRL_DT, _FMOD_DT_MIN_S e _FMOD_MIN_PTS_PER_CYCLE do
# tactile_explorer — só para o preview do painel prever a mesma coisa que o
# explorer vai executar. Se lá mudarem, aqui muda junto (o explorer segue
# sendo a fonte da verdade: ele mede a cadência real e loga).
FMOD_CTRL_DT_S = 0.030          # tick da regulação quase-estática
FMOD_DT_MIN_S = 0.004           # piso do laço Python (250 Hz) — dominado
                                # pelo piso do ServoJ abaixo
FMOD_MIN_PTS_PER_CYCLE = 5
# Espelha SERVOJ_T_MIN_S do real_driver: mínimo do `t` do ServoJ no firmware.
SERVOJ_T_MIN_S = 0.020
# Banda morta do espelhamento ServoJ. Espelha _SERVOJ_DEADBAND_RAD do
# mirror_node — os dois caminhos comandam o MESMO braço e uma banda maior
# aqui engoliria a onda que o outro deixa passar. Os 1e-4 antigos valiam
# ~120 µm de TCP, da ordem da onda inteira.
SERVOJ_DEADBAND_RAD = 1.0e-5

# Quanto tempo o botão Start fica surdo depois de publicar. Cobre o trânsito
# do /palpation/start até o explorer marcar-se ocupado e o primeiro status
# não-IDLE voltar (o status é publicado a 10 Hz). Curto o bastante para não
# atrapalhar quem realmente quer reiniciar após um abort imediato.
START_REPUBLISH_LOCKOUT_S = 2.0


def fmod_wave_dt(hz: float, servoj_period_s: float = FMOD_CTRL_DT_S) -> float:
    """Mesma regra do _ForceProfile.wave_dt do explorer: o tick da onda é
    derivado da frequência pedida, entre o piso do laço (agora incluindo o
    período REAL do ServoJ) e o tick do QS."""
    want = 1.0 / max(hz * FMOD_MIN_PTS_PER_CYCLE, 1e-9)
    return min(max(want, FMOD_DT_MIN_S, SERVOJ_T_MIN_S, servoj_period_s),
               FMOD_CTRL_DT_S)


def fmod_max_freq_hz(servoj_period_s: float = FMOD_CTRL_DT_S) -> float:
    """Frequência máxima rastreável — espelha _fmod_max_freq_hz do explorer.
    Com os 30 ms padrão são 4,17 Hz; acima disso o explorer RECUSA a onda."""
    return 1.0 / max(servoj_period_s * FMOD_MIN_PTS_PER_CYCLE, 1e-9)

# ── MATRIX_MAP — faixas do configurador de grade ───────────────────────
# Passo entre identações. O piso de 0,5 mm é a resolução prática do
# posicionamento cartesiano do CR10; o teto vem do envelope da matriz.


# Controle Manual — definições do braço CR10 e da mão COVVI
import math as _math   # alias para evitar sombrear `math` global do escopo

# Home default (= POINTING_SEED_DEG do constants.py). Sobrescrita em
# runtime por HOME_POSE_FILE quando existe ("✔ Salvar Home").
ARM_HOME_DEG = dict(POINTING_SEED_DEG)
ROBOT_CONFIG_DEFAULTS = {
    'hand_ip':    '192.168.5.103',
    'robot_ip':   '192.168.5.2',
    'robot_mode': 'SIM_ONLY',
}

# Faixas de slider da mão COVVI, em graus (HAND_JOINTS e a pose POINTING
# HAND_POINT_DEG vêm de constants.py — mesma fonte do explorer).
HAND_LIMITS_DEG = {
    'Thumb':  (0, 90), 'Index':  (0, 90), 'Middle': (0, 90),
    'Ring':   (0, 90), 'Little': (0, 90), 'Rotate': (0, 60),
}
HAND_OPEN_DEG  = {j: 0 for j in HAND_JOINTS}
HAND_CLOSE_DEG = {'Thumb': 70, 'Index': 80, 'Middle': 80,
                  'Ring':  80, 'Little': 80, 'Rotate': 0}

# Escala ECI real dos dígitos (calibrada na mão física em 06/07/2026): a
# telemetria DigitPosnAll NÃO vai de 0 a 200 — o fim de curso mecânico
# aberto lê ~47 (rotate ~67) e o fechado ~198 (rotate ~197).
ECI_POSN_OPEN = {'Thumb': 47, 'Index': 47, 'Middle': 47,
                 'Ring':  47, 'Little': 47, 'Rotate': 67}
ECI_POSN_CLOSED = {'Thumb': 198, 'Index': 198, 'Middle': 198,
                   'Ring':  198, 'Little': 198, 'Rotate': 197}

# Grip-patterns embutidos da mão COVVI (CurrentGripID 1–14)
# Para cada padrão de pega:
#   • eci_id → SetCurrentGrip move a MÃO REAL via ECI (id de fábrica COVVI)
#   • graus  → pose equivalente para visualizar no sim Gazebo (juntas primárias)
COVVI_GRIPS: dict[str, tuple[int | None, dict[str, float]]] = {
    'Tripod':       (1,    {'Thumb': 56, 'Index': 52, 'Middle': 52, 'Ring':  0, 'Little':  0, 'Rotate': 44}),
    'Power':        (2,    {'Thumb': 70, 'Index': 74, 'Middle': 74, 'Ring': 72, 'Little': 70, 'Rotate': 12}),
    'Trigger':      (3,    {'Thumb': 45, 'Index':  0, 'Middle': 63, 'Ring': 63, 'Little': 63, 'Rotate': 21}),
    'Prec. Open':   (4,    {'Thumb': 23, 'Index': 23, 'Middle':  0, 'Ring':  0, 'Little':  0, 'Rotate': 47}),
    'Prec. Closed': (5,    {'Thumb': 47, 'Index': 45, 'Middle':  0, 'Ring':  0, 'Little':  0, 'Rotate': 47}),
    'Key':          (6,    {'Thumb': 52, 'Index': 59, 'Middle': 59, 'Ring': 56, 'Little': 52, 'Rotate':  3}),
    'Finger':       (7,    {'Thumb': 27, 'Index':  0, 'Middle': 45, 'Ring': 45, 'Little': 45, 'Rotate': 18}),
    'Cylinder':     (8,    {'Thumb': 59, 'Index': 68, 'Middle': 70, 'Ring': 68, 'Little': 63, 'Rotate': 11}),
    'Column':       (9,    {'Thumb': 45, 'Index': 63, 'Middle': 63, 'Ring': 63, 'Little': 63, 'Rotate': 24}),
    'Relaxed':      (10,   {'Thumb':  9, 'Index':  9, 'Middle':  9, 'Ring':  9, 'Little':  9, 'Rotate':  2}),
    'Glove':        (11,   {'Thumb':  0, 'Index':  0, 'Middle':  0, 'Ring':  0, 'Little':  0, 'Rotate':  0}),
    'Tap':          (12,   {'Thumb':  0, 'Index':  0, 'Middle': 72, 'Ring': 72, 'Little': 72, 'Rotate': 15}),
    'Grab':         (13,   {'Thumb': 74, 'Index': 79, 'Middle': 79, 'Ring': 79, 'Little': 77, 'Rotate': 14}),
    'Tripod Open':  (14,   {'Thumb': 27, 'Index': 23, 'Middle': 23, 'Ring':  0, 'Little':  0, 'Rotate': 44}),
    # Poses gestuais personalizadas (sem preset ECI de fábrica)
    # eci_id=None → só move o sim; não envia SetCurrentGrip ao real.
    'Rock':         (None, {'Thumb': 25, 'Index':  0, 'Middle': 78, 'Ring': 78, 'Little':  0, 'Rotate':  8}),
    'Phone':        (None, {'Thumb':  0, 'Index': 75, 'Middle': 75, 'Ring': 75, 'Little':  0, 'Rotate':  5}),
    'Peace':        (None, {'Thumb': 45, 'Index':  0, 'Middle':  0, 'Ring': 78, 'Little': 78, 'Rotate': 12}),
    'Count 3':      (None, {'Thumb': 55, 'Index':  0, 'Middle':  0, 'Ring':  0, 'Little': 78, 'Rotate':  8}),
    'Count 4':      (None, {'Thumb': 55, 'Index':  0, 'Middle':  0, 'Ring':  0, 'Little':  0, 'Rotate':  5}),
}

# MIMIC_LIST centralizada em kinematics.py (importada acima junto com
# urdf_to_dobot).


# Nó ROS + GUI
class PalpationGUI(FtAxesMixin, MatrixMixin, Node):
    # A classe nasceu com 234 métodos num arquivo só, e achar código nela
    # exigia ferramenta em vez de leitura. Está sendo recortada por MIXINS:
    # o corte é mecânico (os métodos continuam operando sobre `self`), então
    # não muda comportamento — só onde o código mora. `LoadCellMixin` é o
    # primeiro corte; ver gui_loadcell.py.

    def __init__(self):
        super().__init__('palpation_gui')

        # Comunicação ROS (palpation/wrench)
        self._start_pub = self.create_publisher(
            PalpationStart, '/palpation/start', QOS_COMMAND)
        self._stop_pub = self.create_publisher(
            String, '/palpation/stop', 10)
        self._pause_pub = self.create_publisher(
            Bool, '/palpation/pause', 10)
        # FREEZE (E-STOP): congela o explorer NO LUGAR, sem ir à HOME
        # (o STOP normal recua à home). Evita arrastar a ferramenta.
        self._freeze_pub = self.create_publisher(
            Empty, '/palpation/freeze', 10)
        # Setpoint de força ao vivo (modo MANUAL): o explorer segue pelos
        # micro-passos sem reiniciar o ciclo.
        self._set_force_pub = self.create_publisher(
            Float32, '/palpation/set_force', 10)
        self._set_force_after_id = None   # debounce do slider/spinbox
        self.create_subscription(
            PalpationStatus, '/palpation/status', self._cb_status, 10)
        # Leitura dos seis eixos para a aba "6 Axes" e para a planilha.
        # A GUI é só ASSINANTE aqui. Ela já publicou neste tópico o wrench
        # estimado do CR10 (bridge `read_tcp_force`), e enquanto as duas
        # coisas coexistiam o tópico carregava duas fontes intercaladas que o
        # assinante não distinguia. O bridge foi desligado em 12/08/2026 e
        # REMOVIDO em 24/08/2026: o único produtor é o ft_receiver (FA7155).
        # Se um dia o wrench do controlador voltar, ele vai para tópico
        # próprio, não para este.
        self.create_subscription(WrenchStamped, '/ft_sensor/wrench',
                                 self._cb_ft_wrench, QOS_SENSOR)
        # Tópico latched que indica se o drag teach está activo.
        self._drag_pub = self.create_publisher(Bool, '/palpation/drag_mode', QOS_COMMAND)
        # /load_cell/force_net e /load_cell/sample_net NÃO saem mais daqui:
        # quem os publica é o force_receiver, dono da porta serial. Enquanto
        # esta janela os produzia, a malha de segurança do explorer (corte de
        # 15 N, margem de 12 N, detecção de contato) dependia do Tk estar
        # responsivo — e não existia operação sem GUI. Aqui só se PEDE o tare.
        self._lc_tare_req_pub = self.create_publisher(
            Empty, '/load_cell/tare', 10)
        # Quando a GUI lê a serial do STM32 diretamente (mesmo PC), ela
        # REPUBLICA o I_final em /touch_sensor/value — assumindo o papel do
        # touch_sensor.py/touch_receiver para o explorer, o logger e o
        # force_sync.
        self._touch_value_pub = self.create_publisher(
            Float32, '/touch_sensor/value', QOS_SENSOR)
        # Tátil COMPLETO para o palpation_logger juntar no CSV unificado:
        # frame de taxels (ADC) + cada evento de spike/cuneiforme.
        # Frame tátil COM o t_us do STM32 (ver TouchFrame.msg). Substituiu o
        # Int32MultiArray de /touch_sensor/adc, que descartava o `t=` da
        # própria linha que estava parseando.
        self._touch_frame_pub = self.create_publisher(
            TouchFrame, TOUCH_FRAME_TOPIC, QOS_SENSOR)
        self._touch_event_pub = self.create_publisher(
            String, TOUCH_EVENT_TOPIC, QOS_SENSOR)

        # Publishers para comando direto (aba Controle Manual)
        # Os joint_trajectory_controllers expõem um tópico direto
        # `<controller>/joint_trajectory` (além da action).
        self._arm_pub = self.create_publisher(
            JointTrajectory,
            '/cr10_group_controller/joint_trajectory', 5)
        self._hand_pub = self.create_publisher(
            JointTrajectory,
            '/hand_position_controller/joint_trajectory', 5)
        self._suppressing = False   # evita loop ao atualizar sliders

        # Estado partilhado (Tk ↔ ROS)
        self._lock = threading.Lock()
        self._latest_phase: str = 'IDLE'
        self._latest_cycle: int = 0
        self._latest_cycles_total: int = 1
        self._paused: bool = False
        # Odômetro do TCP: posição (m, mundo) capturada no INÍCIO da fase
        # atual.
        self._phase_p0 = None
        # Histórico da força para o sparkline (t_wall, força_N) — 60 s @10 Hz.
        self._spark_data: collections.deque = collections.deque(maxlen=600)
        # Idem para o touch sensor (STM32 via touch_receiver_node).
        self._touch_spark_data: collections.deque = collections.deque(maxlen=600)
        # Cronômetro de fase: marca quando a fase atual começou (wall-clock)
        # e a duração esperada (em segundos) — usada pela progress bar para
        # SLIDING (distance/speed) e CALIBRATING; fases sem duração fixa
        # explorer).
        self._phase_t_start: float = time.time()
        self._latest_speed_mms: float = SPEED_DEFAULT

        # Mão COVVI (lazy)
        self._hand_proc: subprocess.Popen | None = None
        # Indica intenção do usuário: True entre clicar Conectar e clicar
        # Desconectar.
        self._hand_should_be_alive: bool = False
        self._hand_watchdog_thread: threading.Thread | None = None
        self._hand_watchdog_stop = threading.Event()
        self._eci_enabled = False
        self._eci_prefix = self.declare_parameter(
            'eci_prefix', '/covvi/hand').value
        self._param_robot_ip   = self.declare_parameter('robot_ip',   '').value
        self._param_robot_mode = self.declare_parameter('robot_mode', '').value
        # Efetuador final vindo do launch (hand | touch_tool) REGRA (até o
        # usuário pedir o contrário): o modo Palpação só fica disponível
        # quando a célula é aberta COM o touch_tool.
        self._end_effector = str(self.declare_parameter(
            'end_effector', 'touch_tool').value).strip().lower()
        # URDF COMPLETO que o launch entregou ao Gazebo (com os <visual>).
        self._robot_desc_path = str(self.declare_parameter(
            'robot_description_path', '').value).strip()
        self._eci_srv = None
        self._eci_msg = None
        self._cli_eci_grip = None
        self._cli_eci_posn = None
        self._cli_hand_pwr_on = None
        self._cli_hand_pwr_off = None
        self._cli_eci_realtime = None
        self._hand_powered = False
        self._eci_posn_after: str | None = None
        # Versão B: mirror real→sim da mão (telemetria DigitPosnAll)
        # A mão simulada segue a POSIÇÃO MEDIDA da mão física (escala ECI
        # 0–200), de modo que o sim acompanhe a velocidade real, em vez de
        # repetir o comando aberto do slider. Veja _on_real_hand_posn.
        self._sub_real_hand_posn = None
        self._hand_mirror_active: bool = False
        self._hand_mirror_last_rx: float | None = None
        self._hand_mirror_last_pub: float | None = None

        # CR10 real (lazy)
        self._real_driver = None    # CR10RealDriver | None
        self._real_lock = threading.Lock()
        self._robot_mode: str = 'SIM_ONLY'
        self._robot_connected: bool = False
        self._robot_connecting: bool = False
        # Heartbeat + reconexão automática do braço — detecta perda de
        # comunicação com o controlador CR10 e tenta reabrir os sockets
        # com backoff exponencial. Iniciados em `_finish_robot_connect`.
        self._robot_heartbeat_thread: threading.Thread | None = None
        self._robot_heartbeat_stop = threading.Event()
        self._robot_reconnect_thread: threading.Thread | None = None
        self._robot_reconnecting: bool = False

        # Mirror MovJ — em modo MIRROR, cada nova trajetória publicada em
        # /cr10_group_controller/joint_trajectory dispara um
        # MovJ(joint={...}) para o braço real, usando o ÚLTIMO ponto da
        # trajetória (o alvo).
        self._mirror_timer: threading.Timer | None = None
        self._mirror_timer_lock = threading.Lock()
        # Guarda de reentrância do botão Start: cobre a janela entre o clique
        # e a publicação, que o caminho do auto-tare estica em 1,8 s.
        self._starting_palpation = False
        # Instante da última publicação de /palpation/start. A guarda acima é
        # liberada NA publicação, mas o explorer leva alguns ciclos para
        # marcar-se ocupado e o status só chega a 10 Hz — nessa janela
        # `_latest_phase` ainda é IDLE e um segundo clique passava pelos dois
        # gates. Este carimbo fecha o buraco sem depender do status.
        self._start_published_t = 0.0
        # E-STOP é uma CHAVE COM TRAVA (ver _estop): True entre o primeiro e o
        # segundo toque. Enquanto estiver True, nenhum experimento inicia.
        self._estop_latched = False
        self._mirror_last_target: np.ndarray | None = None
        # Poll loop a 33 Hz: lê /joint_states (posição simulada) e espelha
        # para o braço real via MovJ.
        self._latest_joint_rad: list[float] | None = None
        self._mirror_poll_thread: threading.Thread | None = None
        # Subscrição na trajetória comandada (não na pose medida do sim):
        # captura sliders manuais e palpação autônoma com a mesma latência,
        # sem competir com /joint_states (que lagga atrás do comando).
        self.create_subscription(
            JointTrajectory,
            '/cr10_group_controller/joint_trajectory',
            self._cb_arm_trajectory, 1)  # depth=1: só o setpoint mais recente
        # /joint_states: posição real (simulada) do braço — usado pelo
        # mirror poll loop para capturar palpação via action server.
        self.create_subscription(
            JointState, '/joint_states', self._cb_joint_states, 5)
        # Força do eixo de controle: `force_net` é pós-tare (o que o explorer
        # regula), `force` é a MESMA leitura pré-tare — é a diferença entre as
        # duas que denuncia zero deslocado.
        self.create_subscription(
            Float32, '/load_cell/force_net', self._cb_lc_force_net_gui, QOS_SENSOR)
        self.create_subscription(
            Float32, '/load_cell/force', self._cb_lc_force_raw_gui, QOS_SENSOR)
        # Estado e desfecho do tare — de propriedade do ft_receiver.
        self.create_subscription(
            Bool, '/load_cell/tared', self._cb_lc_tared, 10)
        self.create_subscription(
            String, '/load_cell/tare_result', self._cb_lc_tare_result, 10)
        self.create_subscription(
            Float32, '/touch_sensor/value', self._cb_touch_value, QOS_SENSOR)

        # Home pose customizável Default (ARM_HOME_DEG) é sobrescrito se
        # ~/.config/touch_pack/ home_pose.json existir.
        self._arm_home_deg: dict[str, float] = dict(ARM_HOME_DEG)
        self._load_home_pose()
        # Parâmetros da palpação persistidos do último start — usados como
        # defaults dos vars na construção da aba (não voltam ao default de
        # fábrica a cada sessão).
        self._palp_saved: dict = self._load_palp_params()

        # IPs e modo persistidos — carregar antes da UI para os defaults
        # dos campos refletirem o último valor usado.
        self._robot_cfg: dict[str, str] = dict(ROBOT_CONFIG_DEFAULTS)
        self._load_robot_config()
        # Parâmetros ROS sobrescrevem robot.json (permitem override via launch/CLI).
        if self._param_robot_ip:
            self._robot_cfg['robot_ip'] = self._param_robot_ip
        if self._param_robot_mode in ('SIM_ONLY', 'MIRROR', 'REAL_FROM_SIM'):
            self._robot_cfg['robot_mode'] = self._param_robot_mode

        # Força do eixo de controle (N, compressão positiva). `_raw` é a
        # mesma leitura ANTES do tare.
        self._lc_force_raw: float = 0.0
        # Espelho do tare, para EXIBIÇÃO. O dono da referência e do auto-tare
        # de partida é o ft_receiver, que tem a porta.
        self._lc_tare_done: bool = False
        # Força de contato tare-compensada (N, positiva = compressão).
        self._lc_force_net: float = 0.0
        self._lc_force_net_ts: float = 0.0

        # ── Célula de 6 eixos FA7155 (/ft_sensor/wrench) ──────────────
        # Último wrench recebido, por eixo. Alimenta a aba "6 Axes" e as seis
        # colunas ft_* da planilha. Zerado (e não None) para a gravação nunca
        # precisar tratar ausência no meio da linha.
        self._ft_wrench: dict = {a: 0.0 for a in ('fx', 'fy', 'fz',
                                                  'mx', 'my', 'mz')}
        self._ft_last_ts: float = 0.0
        self._ft_frames_ok: int = 0
        self._ft_frames_bad: int = 0
        # Taxa MEDIDA no host. O FA7155 não numera nem carimba os quadros, e o
        # ft_serial gera seq/t_us do relógio do PC — então esta é a única
        # cadência observável daqui, e é ela que responde "quantos Hz chegam".
        self._ft_rate_hz: float | None = None
        self._ft_arrivals: collections.deque = collections.deque(maxlen=120)
        # Subprocesso do force_receiver_node (gerenciado pelo botão Conectar)
        # Touch sensor (STM32 → PC plotter → UDP via touch_receiver)
        # Gerenciado junto com o force_receiver pelo mesmo botão Conectar.
        self._touch_value: float = 0.0
        self._touch_last_ts: float = 0.0
        self._touch_rx_proc: subprocess.Popen | None = None
        # Fonte serial do touch sensor + figura embutida (aba Sensores)
        # Porta da serial do STM32: '' (default) → auto-detect (/dev/ttyACMx).
        self._touch_port = str(self.declare_parameter(
            'touch_serial_port', '').value).strip()
        # Tipo do sensor de toque (launch: sensor:='5' | '4')
        # '5' (DEFAULT) → grade 5×5 SEM TOTAL, que é a montada na bancada; o
        # sinal publicado em /touch_sensor/value é a ativação média por frame
        # (ver touch_source).
        # '4' → grade 4×4 com linha TOTAL/Ifinal (firmware Izhikevich clássico).
        # Qualquer outro valor cai no 5×5. O palpation_logger recebe ESTE mesmo
        # parâmetro pelo launch: se as grades divergirem ele descarta todos os
        # frames e o run sai sem tátil.
        _sensor = str(self.declare_parameter('sensor', '5').value).strip()
        if _sensor == '4':
            self._touch_rows, self._touch_cols, self._touch_has_total = 4, 4, True
        else:
            self._touch_rows, self._touch_cols, self._touch_has_total = 5, 5, False
        self._touch_taxels = self._touch_rows * self._touch_cols
        self._sensor_kind = _sensor
        # Contadores de integridade do frame ADC republicado (ver
        # _publish_tactile_line). Frame truncado é DESCARTADO, não completado:
        # o CSV precisa distinguir "não medi" de "medi errado".
        self._adc_pub_ok = 0
        self._adc_pub_bad = 0
        self._adc_bad_warn_t = 0.0
        self._touch_source = None      # TouchSensorSource | None
        self._touch_figure = None      # TouchFigure | None
        self._touch_canvas = None      # FigureCanvasTkAgg | None
        self._touch_anim = None        # FuncAnimation | None (blit)
        self._touch_anim_running = False
        self._touch_serial_ok = False
        self._sensors_tab_frame: tk.Frame | None = None
        self._sensors_after: str | None = None

        # Aba "3D Manipulation" — arrasto do TCP com IK diferencial.
        self._manip_view = None            # Manip3DView | None
        self._manip_tab_frame: tk.Frame | None = None
        # True do início do arrasto até ~200 ms depois do último publish.
        self._manip_active = False
        self._manip_mirror_on = False
        self._manip_release_after: str | None = None
        self._manip_gate_last: tuple | None = None
        self._manip_readout_q: list | None = None
        self._manip_scene_state = 'idle'   # idle | loading | ready | failed
        self._manip_scene_coarse = None    # malha reduzida usada no arrasto
        self._manip_exact = False          # True = malha do URDF sem reduzir
        # Juntas fora do braço (mão COVVI) lidas de /joint_states: é o que
        # faz os dedos da viewport 3D acompanharem a mão simulada.
        self._latest_extra_joints: dict = {}
        # Publicação de /touch_sensor/value SEM decimação: o STM32 emite a
        # ~1 kHz e queremos esse 1 kHz no ROS. period=0 → publica toda
        # amostra. (Antes limitava a 100 Hz "porque os consumidores eram
        # ≤50 Hz".) Atenção: quem consome na taxa da CÉLULA (o
        # palpation_logger, cujo sinal canônico é /load_cell/force_net) sai
        # a 10-80 Hz — limite do HX711, não deste publisher.
        self._touch_pub_period = 0.0
        self._touch_pub_last = 0.0
        # Gravação do stream sincronizado (botão na aba Palpação)
        self._rec_fh = None
        self._rec_writer = None
        self._rec_t0: float = 0.0
        self._rec_path: str | None = None
        self._rec_count: int = 0
        self._rec_after: str | None = None
        # True quando a gravação foi aberta pelo início da palpação (e não
        # pelo botão): só essa é fechada sozinha em DONE/ABORTED.
        self._rec_auto: bool = False
        # CSVs "crus" gravados em paralelo ao sensors.csv, no MESMO instante
        # de início/fim e com o MESMO timestamp no nome: adc_*, spikes_*,
        # cuneiformes_* — idênticos ao plotter de coleta standalone. Alimentados
        # pelo tap de linhas brutas da fonte (_on_raw_lines), na thread serial.
        self._ref_adc_fh = None
        self._ref_adc_writer = None
        self._ref_spike_fh = None
        self._ref_spike_writer = None
        self._ref_cn_fh = None
        self._ref_cn_writer = None
        # Cabeçalho do CSV montado a partir da GRADE escolhida (4×4 ou 5×5).
        # touch_t_stm_s = relógio do firmware (micros()/1e6 a 1 kHz): é ELE
        # que data cada amostra de 1 ms, em vez do relógio do PC (t_unix).
        self._rec_header = (
            # load_cell_raw_n é a MESMA leitura de force_net_n antes do tare:
            # a diferença entre as duas é o zero vigente na linha. A coluna de
            # tensão saiu com a célula axial — a FA7155 não tem tensão, entrega
            # newtons calibrados de fábrica.
            ['t_rel_s', 't_unix', 'touch_t_stm_s', 'force_net_n',
             'load_cell_raw_n', 'touch_i_final']
            # Seis eixos do FA7155, em N e N·m já calibrados de fábrica.
            # ft_age_ms diz há quanto tempo o wrench desta linha chegou: sem
            # ele, uma célula desconectada gravaria o último valor bom para
            # sempre, indistinguível de dado fresco.
            + ['ft_fx_n', 'ft_fy_n', 'ft_fz_n',
               'ft_mx_nm', 'ft_my_nm', 'ft_mz_nm', 'ft_age_ms']
            + [f'v{r}{c}' for r in range(self._touch_rows)
               for c in range(self._touch_cols)])
        # palpation_logger spawnado pela GUI quando ela roda standalone
        # (fora do launch) — sem ele nenhum run é gravado em ~/touch_pack_runs.
        self._logger_proc: subprocess.Popen | None = None
        # Mini-painel de leitura da célula na aba Controle Manual (modo
        # touch_tool).
        self._mlc_force_lbl = None
        self._mlc_normal_lbl = None
        self._mlc_voltage_lbl = None
        self._mlc_status_lbl = None

        # Poses & Movimentos
        self._poses: list[dict] = []        # [{id, name, q_deg:[6]}]
        self._movements: list[dict] = []    # [{id, name, pose_ids, speed_pct, dur_s}]
        self._next_pose_id: int = 1
        self._next_movement_id: int = 1
        self._drag_enabled: bool = False
        self._drag_last_valid_q: np.ndarray | None = None
        self._drag_last_t: float | None = None
        # Follow real→sim do jog em MIRROR: enquanto um MovJ está em curso,
        # o Gazebo reproduz o feedback medido do braço real (perfil de
        # velocidade físico) em vez da duração heurística do slider.
        self._mirror_follow_until: float = 0.0
        self._mirror_following: bool = False
        self._follow_last_q: np.ndarray | None = None
        self._follow_last_t: float | None = None
        self._follow_still_ticks: int = 0
        self._follow_moved: bool = False
        # Timestamp do último comando de movimento enviado ao robô real.
        self._last_robot_cmd_t: float = 0.0
        self._exec_stop = threading.Event()
        self._exec_thread: threading.Thread | None = None
        self._exec_movement_id: int | None = None
        # Refs de widgets (preenchidos em _build_poses_tab)
        self._poses_lbx: tk.Listbox | None = None
        self._movs_lbx: tk.Listbox | None = None
        self._mov_detail_outer: tk.Frame | None = None
        self._mov_detail_inner: tk.Frame | None = None
        self._drag_btn = None
        self._load_poses_data()

        # Fonte serial do touch sensor Instanciada ANTES de _build_ui porque
        # a aba Sensores constrói a TouchFigure a partir dela; o start()
        # (abre a serial) vem depois, já com a janela montada.
        if _TOUCH_PLOT_OK:
            # Transporte é a USB do STM32 e só ela — sem modo rede e sem relay
            # de frame (ver o docstring de TouchSensorSource).
            self._touch_source = TouchSensorSource(
                port=(self._touch_port or None),
                on_sample=self._on_touch_sample,
                rows=self._touch_rows, cols=self._touch_cols,
                has_total=self._touch_has_total,
                on_raw_lines=self._on_raw_lines)

        # Tkinter root
        self.root = tk.Tk()
        self.root.withdraw()
        self._build_ui()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.deiconify()

        # ROS spin em thread separada (Tk mainloop no thread principal).
        self._stop_event = threading.Event()
        self._spin_thread = threading.Thread(
            target=self._spin_ros, daemon=True)
        self._spin_thread.start()
        # Poll loop iniciado AQUI para garantir que _stop_event já existe.
        self._mirror_poll_thread = threading.Thread(
            target=self._mirror_poll_loop, daemon=True, name='mirror-poll')
        self._mirror_poll_thread.start()

        self.root.after(100, self._refresh_status_panel)
        # Abre a serial do STM32 (best-effort) e dispara o loop de redesenho
        # da figura embutida na aba Sensores.
        self._start_touch_source()
        # A célula NÃO é aberta aqui: o force_receiver é o dono da serial do
        # XIAO e a GUI só assina /load_cell/voltage (ver o topo do arquivo).
        self.root.after(200, self._refresh_sensors_tab)
        # Hot-plug serial↔rede: reconcilia a fonte do toque a cada 2 s.
        self.root.after(2000, self._retry_touch_source)

    # Touch sensor — fonte serial + publicação ROS
    def _start_touch_source(self) -> None:
        """Abre a serial do STM32. Sem ela não há dado tátil — não existe mais
        queda para o modo rede."""
        if not _TOUCH_PLOT_OK or self._touch_source is None:
            log.info('[TOUCH] matplotlib ausente — figura desabilitada')
            return
        if self._touch_source.start():
            self._touch_serial_ok = True
            log.info('[TOUCH] serial em %s — publicando /touch_sensor/value',
                     self._touch_source.port)
        else:
            self._touch_serial_ok = False
            # ERROR, não INFO: era exatamente esta linha, em INFO e seguida de
            # um fallback mudo, que deixou o run 20260807_185000 ser gravado
            # com 2,7 quadros/s sem ninguém perceber.
            log.error('[TOUCH] SEM SENSOR DE TOQUE: %s. O transporte é a USB '
                      'do STM32 e não há caminho alternativo — confira o cabo. '
                      'Qualquer coleta iniciada agora sai SEM dado tátil.',
                      self._touch_source.error)

    def _retry_touch_source(self) -> None:
        """Hot-plug: a cada 2 s reconcilia a fonte do toque com o hardware."""
        try:
            src = self._touch_source
            if src is None:
                return
            if not src.connected:
                # Nunca abriu, ou a serial caiu (replug/porta renomeada).
                src.stop()
                was_ok = self._touch_serial_ok
                self._touch_serial_ok = src.start()
                if self._touch_serial_ok:
                    log.info('[TOUCH] hot-plug: serial em %s', src.port)
                elif was_ok:
                    log.error('[TOUCH] a serial do toque CAIU (%s) — sem dado '
                              'tátil até o cabo voltar.', src.error)
        except Exception as exc:
            log.debug('retry touch source falhou: %s', exc)
        finally:
            self.root.after(2000, self._retry_touch_source)

    def _on_touch_sample(self, i_final: float) -> None:
        """Callback da thread serial: republica I_final em ROS e atualiza o
        estado interno, sem tocar em widgets Tk.
        """
        if self._touch_pub_period > 0.0:
            now = time.monotonic()
            if now - self._touch_pub_last < self._touch_pub_period:
                return
            self._touch_pub_last = now
        try:
            msg = Float32(); msg.data = float(i_final)
            self._touch_value_pub.publish(msg)
        except Exception:
            pass
        with self._lock:
            self._touch_value = float(i_final)
            self._touch_last_ts = time.time()
        # Gravação do stream força+toque a 1 kHz (se ligada) — fora do lock acima
        # porque _record_row pega self._lock por conta própria.
        if self._rec_writer is not None:
            self._record_row(i_final)

    # Aba "Sensores": todos os plots lado a lado
    def _build_sensors_tab(self, root: tk.Frame) -> None:
        """Dashboard: os quatro gráficos do touch sensor (heatmap, raster
        RA/SA, I_final, neurônio pós) embutidos via matplotlib, lado a lado
        com a leitura ao vivo da célula de carga."""
        body = tk.Frame(root, bg=BG)
        body.pack(fill='both', expand=True, padx=8, pady=8)

        # Esquerda: figura do touch sensor
        left = tk.Frame(body, bg=BG)
        left.pack(side='left', fill='both', expand=True, padx=(0, 8))

        hdr = tk.Frame(left, bg=BG); hdr.pack(fill='x')
        tk.Label(hdr, text='Touch Sensor (STM32) — Izhikevich',
                 font=FONT_HEAD, bg=BG, fg=TEXT).pack(side='left')
        self._sens_touch_status_lbl = tk.Label(
            hdr, text='', font=FONT_SMALL, bg=BG, fg=TEXT_DIM)
        self._sens_touch_status_lbl.pack(side='right')

        plot_holder = tk.Frame(left, bg=PANEL, highlightthickness=1,
                               highlightbackground=BORDER)
        plot_holder.pack(fill='both', expand=True, pady=(6, 0))
        if (_TOUCH_PLOT_OK and self._touch_source is not None
                and TouchFigure is not None):
            try:
                self._touch_figure = TouchFigure(
                    self._touch_source, facecolor=PANEL)
                self._touch_canvas = FigureCanvasTkAgg(
                    self._touch_figure.fig, master=plot_holder)
                self._touch_canvas.get_tk_widget().pack(
                    fill='both', expand=True)
                self._touch_canvas.draw()
                # blit=True: só os artistas animados são redesenhados. O
                # redraw completo custava ~80 ms por frame nesta figura (4
                # eixos + colorbar + legenda) e era pedido a cada 50 ms — o
                # laço do Tk ficava saturado e a GUI INTEIRA travava. Só é
                # válido porque os limites dos eixos são fixos: o raster usa
                # tempo relativo a agora, não absoluto (ver TouchFigure).
                #
                # interval=33 (30 fps) e não 20 (50 fps): um quadro com blit
                # custa 11,1 ms p50 / 16,4 ms p99 (medido em 10/08/2026), e a
                # 50 fps o p99 encostava no orçamento de 20 ms — na thread do
                # Tk, que também pinta todo o resto da GUI. Como a animação
                # agora roda TAMBÉM durante a coleta (ver _refresh_sensors_tab),
                # 30 fps é o que deixa folga; num heatmap a diferença não se vê.
                self._touch_anim = FuncAnimation(
                    self._touch_figure.fig,
                    self._touch_figure.update,
                    init_func=self._touch_figure.init_blit,
                    interval=33, blit=True, cache_frame_data=False)
                self._touch_anim_running = True
            except Exception as exc:
                log.warning('[TOUCH] falha ao embutir figura: %s', exc)
                self._touch_figure = None
                self._touch_canvas = None
                self._touch_anim = None
                tk.Label(plot_holder,
                         text=f'Figure unavailable: {exc}',
                         font=FONT_LBL, bg=PANEL, fg=TEXT_DIM).pack(
                    expand=True, pady=40)
        else:
            tk.Label(plot_holder,
                     text='matplotlib/pyserial missing — '
                          'install them to see the touch charts.',
                     font=FONT_LBL, bg=PANEL, fg=TEXT_DIM).pack(
                expand=True, pady=40)

        # Direita: célula de carga ao vivo
        right = tk.Frame(body, bg=BG, width=270)
        right.pack(side='right', fill='y')
        right.pack_propagate(False)

        card = self._card(right, 'Load Cell — live')
        tk.Label(card, text='Compression Force (tare)', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(anchor='w', pady=(4, 0))
        self._sens_force_lbl = tk.Label(
            card, text='—   N', font=FONT_BIG, bg=PANEL, fg=TEXT_DIM)
        self._sens_force_lbl.pack(anchor='w', pady=(2, 2))
        self._sens_status_lbl = tk.Label(
            card, text='waiting for /load_cell/force_net',
            font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM)
        self._sens_status_lbl.pack(anchor='w')

        tk.Frame(card, bg=BORDER, height=1).pack(fill='x', pady=8)
        self._sens_raw_lbl   = self._kv(card, 'LC raw',  '—  N')
        self._sens_volt_lbl  = self._kv(card, 'LC Voltage', '—  V')
        self._sens_touch_lbl = self._kv(card, 'Toque I_final', '—')

        tk.Frame(card, bg=BORDER, height=1).pack(fill='x', pady=8)
        tk.Label(card, text='Force — last 30 s', font=FONT_SMALL,
                 bg=PANEL, fg=TEXT_MUTED, anchor='w').pack(fill='x')
        self._sens_force_spark = tk.Canvas(
            card, height=80, bg=PANEL, highlightthickness=1,
            highlightbackground=BORDER)
        self._sens_force_spark.pack(fill='x', pady=(4, 2))

    def _refresh_sensors_tab(self) -> None:
        """Loop da aba Sensores. A figura do toque é desenhada pela
        FuncAnimation (blit); aqui só a pausamos/retomamos conforme a aba
        esteja visível (poupa CPU) e atualizamos os números da célula."""
        try:
            nb = getattr(self, '_nb', None)
            frame = self._sensors_tab_frame
            visible = (nb is not None and frame is not None
                       and str(nb.select()) == str(frame))
            # Anima sempre que a aba estiver à vista — INCLUSIVE durante a
            # coleta, que é justamente quando se quer olhar o sinal. O
            # `not self._experiment_active()` que havia aqui datava de quando
            # a fonte entregava 2,7 quadros/s por UDP; medido em 10/08/2026
            # sobre o stream serial real (834 quadros/s), animar não custa
            # amostra nenhuma: 800 quadros/s com e sem animação, e o
            # `snapshot()` só segura o lock da fonte 0,26 ms (p99 0,60 ms).
            self._set_touch_anim(visible)
            if visible:
                self._update_sensors_panel()
        finally:
            self._sensors_after = self.root.after(
                80, self._refresh_sensors_tab)

    def _tab_visible(self, *frames) -> bool:
        """True se alguma das abas dadas é a selecionada agora. Antes de o
        notebook existir devolve True — na dúvida, pinta."""
        nb = getattr(self, '_nb', None)
        if nb is None:
            return True
        try:
            cur = str(nb.select())
        except tk.TclError:
            return True
        return any(f is not None and str(f) == cur for f in frames)

    def _set_touch_anim(self, run: bool) -> None:
        """Liga/desliga a animação do touch sensor (idempotente)."""
        anim = getattr(self, '_touch_anim', None)
        if anim is None or run == self._touch_anim_running:
            return
        try:
            if run:
                anim.resume()
            else:
                anim.pause()
            self._touch_anim_running = run
        except Exception as exc:
            log.debug('touch anim toggle falhou: %s', exc)

    def _experiment_active(self) -> bool:
        """True se há EXPERIMENTO em andamento: palpação rodando (fase !=
        IDLE/DONE/ABORTED) ou gravação da GUI ligada (_rec_fh aberto).
        """
        return (self._rec_fh is not None
                or self._latest_phase not in ('IDLE', 'DONE', 'ABORTED'))

    def _touch_source_status(self, scalar_fresh: bool) -> tuple[str, str]:
        """Texto/cor honestos da fonte do toque, do estado AO VIVO da fonte."""
        src = self._touch_source
        if src is not None and src.connected:
            base = f'serial {src.port}'
            if src.is_fresh():
                # Frames truncados não aparecem em lugar nenhum se não forem
                # ditos aqui: o descarte é correto, mas uma coleta que perdeu
                # 19% dos frames não pode parecer verde.
                bad, ok = src.frames_bad, src.frames_ok
                total = bad + ok
                if bad and total:
                    pct = 100.0 * bad / total
                    return (f'{base} — {pct:.1f}% dos frames perdidos '
                            f'({bad}/{total})', WARN if pct < 1.0 else DANGER)
                return base, OK
            # Ligado mas mudo: porta serial errada ou STM mudo.
            return f'{base} (no data)', WARN
        if scalar_fresh:
            return 'via /touch_sensor/value', OK
        # Sem serial não há tátil nenhum. Vermelho, não cinza: em cinza isto
        # passa por "ainda não ligou" e a coleta sai vazia (ver 07/08/2026).
        return 'SEM SENSOR DE TOQUE (confira o cabo USB)', DANGER

    def _contact_indicator(self, f_net: float) -> bool:
        """Indicador "in contact" da tela, com histerese.

        Acende no MESMO limiar que o explorer usa como gatilho de halt
        (`_CONTACT_ON_N`) e apaga em `_CONTACT_OFF_FRAC` dele. Antes havia um
        0.2 cravado aqui, que abria uma zona cega de 0,10–0,20 N: o robô já
        tinha declarado contato e a tela ainda dizia "no contact".

        A histerese é SÓ do indicador. Com o limiar de acender igual ao de
        apagar, o verde pisca em ar livre a cada excursão de ruído acima de
        0,10 N — cosmético na tela, mas é o painel que se usa para julgar se a
        célula está zerada antes de um perfil de descida.
        """
        on = bool(getattr(self, '_contact_ind_on', False))
        thr = (_CONTACT_OFF_FRAC * _CONTACT_ON_N) if on else _CONTACT_ON_N
        on = f_net >= thr
        self._contact_ind_on = on
        return on

    def _update_sensors_panel(self) -> None:
        """Atualiza os números da célula de carga + sparkline na aba Sensores."""
        with self._lock:
            f_net     = self._lc_force_net
            lc_ts     = self._lc_force_net_ts
            f_raw     = self._lc_force_raw
            tare_done = self._lc_tare_done
            touch_val = self._touch_value
            touch_ts  = self._touch_last_ts

        has_data = lc_ts > 0.0 and (time.time() - lc_ts) < 3.0
        if has_data:
            if not tare_done:
                color, status = WARN, 'tare not done'
            elif f_net > _FORCE_ABORT_LIMIT_N * 0.9:
                color, status = DANGER, f'near the limit ({_FORCE_ABORT_LIMIT_N:.0f} N)'
            elif self._contact_indicator(f_net):
                color, status = OK, 'in contact'
            else:
                color, status = TEXT_MUTED, 'no contact'
            self._sens_force_lbl.config(text=f'{f_net:+6.2f}  N', fg=color)
            self._sens_status_lbl.config(text=status, fg=color)
            self._sens_raw_lbl.config(text=f'{f_raw:+6.2f} N')
            self._sens_volt_lbl.config(text=f'{f_raw - f_net:+6.3f} N')
        else:
            self._sens_force_lbl.config(text='—   N', fg=TEXT_DIM)
            self._sens_status_lbl.config(
                text='waiting for /load_cell/force_net', fg=TEXT_DIM)
            self._sens_raw_lbl.config(text='—  N')
            self._sens_volt_lbl.config(text='—  N')

        touch_fresh = touch_ts > 0.0 and (time.time() - touch_ts) < 3.0
        self._sens_touch_lbl.config(
            text=f'{touch_val:+.3f}' if touch_fresh else '—')

        label, fg = self._touch_source_status(touch_fresh)
        self._sens_touch_status_lbl.config(text=label, fg=fg)

        self._draw_force_spark(self._sens_force_spark)

    def _draw_force_spark(self, cv: tk.Canvas) -> None:
        """Desenha self._spark_data (força, 30 s) num canvas dado — usado pela
        aba Sensores. self._spark_data é alimentado em _refresh_status_panel."""
        if cv is None:
            return
        try:
            w = cv.winfo_width(); h = cv.winfo_height()
            cv.delete('all')
        except tk.TclError:
            return
        if w <= 10 or h <= 10:
            return
        now = time.time()
        window = 30.0
        pts = [(t, f) for t, f in self._spark_data if now - t <= window]
        forces = [f for _, f in pts]
        f_hi = max([1.0] + forces)
        f_lo = min([0.0] + forces)
        rng = max(f_hi - f_lo, 0.5)

        def xy(t: float, f: float) -> tuple[float, float]:
            x = w - (now - t) / window * w
            y = (h - 4) - (f - f_lo) / rng * (h - 8)
            return x, y

        y_zero = xy(now, 0.0)[1]
        cv.create_line(0, y_zero, w, y_zero, fill=BORDER)
        if len(pts) >= 2:
            coords: list[float] = []
            for t, f in pts:
                coords.extend(xy(t, f))
            cv.create_line(*coords, fill=PRIMARY, width=2)

    # Gravação do stream sincronizado força + toque (CSV) O cabeçalho
    # (self._rec_header) é montado no __init__ a partir da grade do sensor
    # (4×4 ou 5×5) — ver bloco de estado de gravação.
    _REC_STATUS_MS = 250

    def _toggle_recording(self) -> None:
        if self._rec_fh is not None:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self, run_id: str | None = None,
                         mode: str | None = None,
                         auto: bool = False) -> None:
        """Abre o sensors.csv + os CSVs crus NA PASTA DO RUN.

        `run_id`/`mode` vindos do start são os mesmos que o palpation_logger
        recebe pela mensagem, e é isso que põe estes arquivos na mesma pasta
        que o samples.csv dele. Sem eles (botão "Record data" fora de um
        run) a gravação vai para RECORDING/<carimbo novo>.

        `auto` marca que quem abriu foi o início da palpação, e não o botão —
        só a gravação automática é fechada sozinha no fim do run."""
        try:
            out_dir = run_dir(mode or '', run_id or new_run_id())
            path = os.path.join(out_dir, RUN_SENSORS_CSV)
            fh = open(path, 'w', newline='')
            writer = csv.writer(fh)
            writer.writerow(self._rec_header)
        except OSError as exc:
            self._set_rec_status(f'failed to open CSV: {exc}', DANGER)
            return
        # CSVs crus (ADC / spikes / cuneiformes), na MESMA pasta.
        # Best-effort: se algum falhar, o sensors.csv segue gravando.
        ref = self._open_reference_csvs(out_dir)
        with self._lock:
            self._rec_fh = fh
            self._rec_writer = writer
            self._rec_path = path
            self._rec_t0 = time.time()
            self._rec_count = 0
            self._rec_auto = bool(auto)
            (self._ref_adc_fh, self._ref_adc_writer,
             self._ref_spike_fh, self._ref_spike_writer,
             self._ref_cn_fh, self._ref_cn_writer) = ref
        self.rec_btn.config(text='■ Stop recording', bg=DANGER, fg='white')
        self._set_rec_status(f'recording → {_rel_run(path)}', OK)
        self._rec_after = self.root.after(
            self._REC_STATUS_MS, self._recording_status_tick)

    def _record_row(self, i_final: float) -> None:
        """Grava UMA linha do stream força+toque. Chamado por _on_touch_sample
        (thread serial, ~1 kHz) — é isto que dá ao sensors.csv a taxa de 1 kHz.
        """
        if self._rec_writer is None:
            return   # fast-path sem lock; reconferido sob o lock abaixo
        now = time.time()
        if self._touch_source is not None and self._touch_source.connected:
            # Tensões + relógio do STM32 sob o MESMO lock: o timestamp do
            # firmware (1 kHz) é o que data a amostra na planilha.
            volt, t_stm = self._touch_source.latest_voltages_and_time()
            volt_cols = [f'{volt[r, c]:.4f}'
                         for r in range(self._touch_rows)
                         for c in range(self._touch_cols)]
        else:
            t_stm = 0.0
            volt_cols = [''] * self._touch_taxels
        with self._lock:
            if self._rec_writer is None:
                return   # _stop_recording correu entre o fast-path e aqui
            f_net    = self._lc_force_net
            lc_bruto = self._lc_force_raw       # pré-tare, compressão positiva
            ftw = dict(self._ft_wrench)
            ft_age_ms = ((now - self._ft_last_ts) * 1000.0
                         if self._ft_last_ts > 0.0 else -1.0)
            try:
                self._rec_writer.writerow([
                    f'{now - self._rec_t0:.4f}', f'{now:.4f}',
                    f'{t_stm:.6f}',
                    f'{f_net:.4f}', f'{lc_bruto:.4f}',
                    f'{float(i_final):.4f}',
                    f'{ftw["fx"]:.5f}', f'{ftw["fy"]:.5f}', f'{ftw["fz"]:.5f}',
                    f'{ftw["mx"]:.6f}', f'{ftw["my"]:.6f}', f'{ftw["mz"]:.6f}',
                    f'{ft_age_ms:.1f}',
                    *volt_cols,
                ])
                self._rec_count += 1
                # Flush a cada ~1 s (1000 amostras @ 1 kHz).
                if self._rec_count % 1000 == 0 and self._rec_fh is not None:
                    self._rec_fh.flush()
            except (ValueError, OSError) as exc:
                log.warning('falha ao gravar amostra sincronizada: %s', exc)

    # CSVs "crus" (ADC / spikes / cuneiformes), iguais ao standalone
    def _open_reference_csvs(self, out_dir: str) -> tuple:
        """Abre os três CSVs crus com o cabeçalho do plotter de coleta
        standalone (adc, spikes, cuneiformes) e devolve a tupla
        (adc_fh, adc_writer, spike_fh, spike_writer, cn_fh, cn_writer).
        """
        try:
            adc_fh = open(os.path.join(out_dir, RUN_ADC_CSV), 'w', newline='')
            adc_w = csv.writer(adc_fh)
            adc_w.writerow(['tempo']
                           + [f'taxel_{i}' for i in range(self._touch_taxels)])
            spike_fh = open(os.path.join(out_dir, RUN_SPIKES_CSV),
                            'w', newline='')
            spike_w = csv.writer(spike_fh)
            spike_w.writerow(['tempo', 'tipo', 'idx', 'adc'])
            cn_fh = open(os.path.join(out_dir, RUN_CN_CSV),
                         'w', newline='')
            cn_w = csv.writer(cn_fh)
            cn_w.writerow(['tempo', 'tipo'])
        except OSError as exc:
            log.warning('falha ao abrir CSVs crus: %s', exc)
            return (None, None, None, None, None, None)
        return (adc_fh, adc_w, spike_fh, spike_w, cn_fh, cn_w)

    def _on_raw_lines(self, lines: list) -> None:
        """Tap das linhas brutas do firmware (thread serial, ~1 kHz por chunk)."""
        # Republica o tátil completo em ROS SÓ quando há
        # experimento/gravação (_experiment_active): é o que o
        # palpation_logger assina para juntar taxels+eventos no CSV do
        # experimento, e ele SÓ grava durante um run.
        if self._experiment_active():
            for line in lines:
                self._publish_tactile_line(line.strip())
        if self._ref_adc_writer is None:
            return  # fast-path: nada a gravar nos CSVs crus
        with self._lock:
            adc_w = self._ref_adc_writer
            spike_w = self._ref_spike_writer
            cn_w = self._ref_cn_writer
            if adc_w is None:
                return  # _stop_recording correu entre o fast-path e aqui
            try:
                for line in lines:
                    self._write_reference_line(line.strip(), adc_w, spike_w, cn_w)
            except (ValueError, OSError) as exc:
                log.warning('falha ao gravar CSV cru: %s', exc)

    def _parse_adc_frame(self, line: str) -> tuple[list, int] | None:
        """Extrai os N taxels + o t_us de 'ADC,v0,...,vN-1,t=micros'.

        Devolve (taxels, t_us), ou None — e CONTA — se a linha não trouxer
        exatamente
        ``self._touch_taxels`` inteiros. Um frame truncado não é um frame
        incompleto do qual se aproveita o começo: quando a serial perde bytes,
        o que sobra depois do buraco é *emendado* de outro frame, então os
        valores a partir do ponto de corte estão trocados de taxel. Medido na
        coleta de 07/08/2026: no frame parcial o erro contra o último frame
        completo passa de ~30 ADC em taxel_0..3 para >1000 ADC em taxel_16.
        Descartar o frame inteiro é a única leitura honesta.

        Note que os tokens são convertidos SEM filtro: um valor corrompido no
        meio derruba a linha (ValueError, tratado no chamador) em vez de
        encurtar silenciosamente a lista — era assim que uma linha suja virava
        um frame "curto" e depois uma linha de CSV desalinhada."""
        parts = line.split(',')
        try:
            vals = [int(v.strip()) for v in parts[1:-1]]
        except ValueError:
            vals = None
        # t_us do próprio frame. Um `t=` ilegível NÃO derruba o frame: os
        # taxels continuam válidos e o consumidor trata t_us=0 como ausente.
        try:
            t_us = int(parts[-1].replace('t=', '').strip()) & 0xFFFFFFFF
        except (ValueError, IndexError):
            t_us = 0
        if vals is not None and len(vals) == self._touch_taxels:
            self._adc_pub_ok += 1
            # Sai na numeração FÍSICA (taxel 0 = 00): é o que vai para o
            # TouchFrame e daí para as colunas taxel_* do samples.csv.
            return (taxel_frame_to_physical(
                vals, self._touch_rows, self._touch_cols), t_us)
        self._adc_pub_bad += 1
        now = time.monotonic()
        if now - self._adc_bad_warn_t > 2.0:
            self._adc_bad_warn_t = now
            total = self._adc_pub_ok + self._adc_pub_bad
            got = 'ilegível' if vals is None else f'{len(vals)}'
            self.get_logger().warn(
                f'[TOUCH] frame ADC corrompido descartado '
                f'({got}/{self._touch_taxels} taxels); '
                f'{self._adc_pub_bad} de {total} frames perdidos neste run — '
                f'bytes perdidos na serial.')
        return None

    def _publish_tactile_line(self, line: str) -> None:
        """Parseia UMA linha do firmware e republica em ROS para o logger:
        frame ADC → TouchFrame; cada spike/cuneiforme → String com o tipo
        (RA|SA|CN_MM|CN_RA|CN_SA). Best-effort: linha malformada é ignorada."""
        if not line:
            return
        try:
            if line.startswith('ADC'):
                parsed = self._parse_adc_frame(line)
                if parsed is not None:
                    vals, t_us = parsed
                    msg = TouchFrame()
                    msg.taxels = vals
                    msg.t_us = t_us
                    msg.rows = self._touch_rows
                    msg.cols = self._touch_cols
                    self._touch_frame_pub.publish(msg)
            elif (line.startswith('CN_MM') or line.startswith('CN_RA')
                  or line.startswith('CN_SA')):
                self._touch_event_pub.publish(String(data=line[:5]))
            elif line.startswith('RA') or line.startswith('SA'):
                self._touch_event_pub.publish(String(data=line[:2]))
        except (ValueError, IndexError):
            pass

    def _write_reference_line(self, line, adc_w, spike_w, cn_w) -> None:
        """Parseia UMA linha e grava no CSV cru correspondente (sob self._lock)."""
        if not line:
            return
        if line.startswith('ADC'):
            parts = line.split(',')
            try:
                tstamp = int(parts[-1].replace('t=', '').strip()) / 1e6
            except (ValueError, IndexError):
                return
            vals = [int(v.strip()) for v in parts[1:-1] if v.strip().isdigit()]
            if len(vals) != self._touch_taxels:
                return
            adc_w.writerow([tstamp, *taxel_frame_to_physical(
                vals, self._touch_rows, self._touch_cols)])
        elif line.startswith('CN_MM') or line.startswith('CN_RA') \
                or line.startswith('CN_SA'):
            m = _REF_T_RE.search(line)
            t = int(m.group(1)) / 1e6 if m else 0.0
            cn_w.writerow([t, line[:5]])
        elif line.startswith('RA') or line.startswith('SA'):
            m = _REF_SPIKE_RE.search(line)
            if m:
                spike_w.writerow([int(m.group(3)) / 1e6, line[:2],
                                  taxel_index_to_physical(
                                      int(m.group(1)),
                                      self._touch_rows, self._touch_cols),
                                  int(m.group(2))])

    def _recording_status_tick(self) -> None:
        """Só atualiza o rótulo de status (na thread Tk); as linhas são gravadas
        pelo callback do toque, não aqui."""
        if self._rec_fh is None:
            return
        self._set_rec_status(
            f'recording {self._rec_count} samples → '
            f'{os.path.basename(self._rec_path or "?")}', OK)
        self._rec_after = self.root.after(
            self._REC_STATUS_MS, self._recording_status_tick)

    def _stop_recording(self) -> None:
        if self._rec_after is not None:
            try:
                self.root.after_cancel(self._rec_after)
            except Exception:
                pass
            self._rec_after = None
        # Zera o writer SOB o lock: a thread serial (_record_row) o checa sob o
        # mesmo lock, então depois daqui ela não escreve mais e podemos fechar.
        with self._lock:
            fh = self._rec_fh
            path = self._rec_path
            n = self._rec_count
            self._rec_fh = None
            self._rec_writer = None
            self._rec_path = None
            self._rec_auto = False
            ref_fhs = [self._ref_adc_fh, self._ref_spike_fh, self._ref_cn_fh]
            self._ref_adc_fh = self._ref_adc_writer = None
            self._ref_spike_fh = self._ref_spike_writer = None
            self._ref_cn_fh = self._ref_cn_writer = None
        if fh is not None:
            try:
                fh.flush(); fh.close()
            except OSError:
                pass
        for rfh in ref_fhs:
            if rfh is not None:
                try:
                    rfh.flush(); rfh.close()
                except OSError:
                    pass
        try:
            self.rec_btn.config(
                text='●  Record data (force+touch)', bg=BTN_NEUTRAL, fg=TEXT)
            self._set_rec_status(
                f'saved: {n} samples to {_rel_run(path)}', TEXT_MUTED)
        except tk.TclError:
            pass

    def _set_rec_status(self, text: str, color: str) -> None:
        lbl = getattr(self, 'rec_status_lbl', None)
        if lbl is not None:
            try:
                lbl.config(text=text, fg=color)
            except tk.TclError:
                pass

    # UI construction
    def _build_ui(self):
        self.root.title('Tactile Palpation — touch_pack')
        self.root.configure(bg=BG)
        # Janela pode encolher bastante; o corpo das abas usa scroll vertical
        # quando o conteúdo for maior que a área visível.
        self.root.minsize(720, 460)

        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Tactile.Horizontal.TScale',
                         background=PANEL, troughcolor=BORDER)

        self._build_header()
        self._build_body()
        self._build_statusbar()

    # Header: título + barra de conexões + E-STOP
    def _build_header(self):
        """Header compacto em 2 linhas: título/E-STOP e uma barra única de
        conexões com os grupos inline (separados por divisores sutis)."""
        hdr = tk.Frame(self.root, bg=HEADER)
        hdr.pack(fill='x', side='top')

        # Linha 1: título à esquerda, E-STOP à direita ────────────────
        top = tk.Frame(hdr, bg=HEADER)
        top.pack(fill='x', padx=18, pady=(10, 4))

        tk.Label(top, text='Tactile Palpation', font=FONT_TITLE,
                 bg=HEADER, fg=HEADER_FG).pack(side='left')

        estop = _hdr_btn(top, '■', 'E-STOP', self._estop,
                          bg=DANGER, fg='white',
                          font=FONT_HEAD,
                          padx=20, pady=8)
        estop.pack(side='right')
        self._estop_btn = estop
        self._refresh_estop_button()

        # Linha 2: barra de conexões ──────────────────────────────────
        mid = tk.Frame(hdr, bg=HEADER)
        mid.pack(fill='x', padx=18, pady=(2, 10))

        def _sep():
            tk.Frame(mid, bg=_shade(HEADER, 0.25), width=1
                     ).pack(side='left', fill='y', padx=14, pady=3)

        def _group_lbl(parent, text):
            tk.Label(parent, text=text, font=FONT_SMALL,
                     bg=HEADER, fg='#cbd5e1').pack(side='left', padx=(0, 8))

        # COVVI HAND — só aparece no modo `hand` Widgets sempre criados
        # (callbacks os referenciam); o frame só é empacotado quando há mão.
        conn = tk.Frame(mid, bg=HEADER)
        if self._end_effector == 'hand':
            conn.pack(side='left')
        _group_lbl(conn, 'COVVI HAND')
        self._hand_ip_var = tk.StringVar(value=self._robot_cfg['hand_ip'])
        tk.Entry(conn, textvariable=self._hand_ip_var,
                  width=14, font=FONT_MONO_S, bg='white', fg=TEXT,
                  relief='flat', bd=0, highlightthickness=1,
                  highlightbackground=BORDER, highlightcolor=PRIMARY,
                  justify='center'
                  ).pack(side='left', padx=(0, 6), ipady=4)
        self._hand_connect_btn = _hdr_btn(
            conn, '⚡', 'Connect', self._connect_real_hand,
            bg=PRIMARY, fg='white', font=FONT_LBL, padx=12, pady=5)
        self._hand_connect_btn.pack(side='left', padx=(0, 6))
        self._eci_btn = _hdr_btn(
            conn, '◉', 'ECI OFF', self._toggle_eci,
            bg=BTN_NEUTRAL, fg=TEXT, font=FONT_SMALL, padx=10, pady=5)
        self._eci_btn.pack(side='left', padx=(0, 6))
        self._pwr_btn = _hdr_btn(
            conn, '⊙', 'PWR OFF', self._toggle_hand_power,
            bg=BTN_NEUTRAL, fg=TEXT, font=FONT_SMALL, padx=10, pady=5)
        self._pwr_btn.pack(side='left')
        if self._end_effector == 'hand':
            _sep()

        # ROBÔ CR10
        conn_rob = tk.Frame(mid, bg=HEADER)
        conn_rob.pack(side='left')
        _group_lbl(conn_rob, 'CR10 ROBOT')
        self._robot_ip_var = tk.StringVar(value=self._robot_cfg['robot_ip'])
        tk.Entry(conn_rob, textvariable=self._robot_ip_var,
                  width=13, font=FONT_MONO_S, bg='white', fg=TEXT,
                  relief='flat', bd=0, highlightthickness=1,
                  highlightbackground=BORDER, highlightcolor=PRIMARY,
                  justify='center'
                  ).pack(side='left', padx=(0, 6), ipady=4)
        self._robot_connect_btn = _hdr_btn(
            conn_rob, '⚡', 'Connect', self._connect_real_robot,
            bg=PRIMARY, fg='white', font=FONT_LBL, padx=12, pady=5)
        self._robot_connect_btn.pack(side='left', padx=(0, 6))
        self._robot_mode_var = tk.StringVar(value=self._robot_cfg['robot_mode'])
        # `_robot_mode` (estado interno) deve seguir o valor carregado.
        self._robot_mode = self._robot_cfg['robot_mode']
        mode_menu = tk.OptionMenu(
            conn_rob, self._robot_mode_var,
            'SIM_ONLY', 'MIRROR',
            command=self._set_robot_mode)
        mode_menu.config(bg=BTN_NEUTRAL, fg=TEXT, font=FONT_SMALL,
                          relief='flat', highlightthickness=0,
                          activebackground=PRIMARY,
                          activeforeground='white',
                          padx=8, pady=2)
        mode_menu['menu'].config(bg=PANEL, fg=TEXT, font=FONT_SMALL,
                                   activebackground=PRIMARY,
                                   activeforeground='white')
        mode_menu.pack(side='left')

        # Saúde do link da célula FA7155 — só no modo `touch_tool`
        if self._end_effector == 'touch_tool':
            _sep()
        conn_cell = tk.Frame(mid, bg=HEADER)
        if self._end_effector == 'touch_tool':
            conn_cell.pack(side='left')
        _group_lbl(conn_cell, 'LOAD CELL')
        self._cell_dot_lbl = tk.Label(
            conn_cell, text='●', font=FONT_LBL, bg=HEADER, fg=TEXT_DIM)
        self._cell_dot_lbl.pack(side='left')
        self._cell_status_lbl = tk.Label(
            conn_cell, text='OFFLINE', font=FONT_LBL, bg=HEADER, fg=TEXT_DIM)
        self._cell_status_lbl.pack(side='left', padx=(4, 0))

    # Corpo: Notebook com 2 abas
    def _build_body(self):
        # Estilo das abas no tema claro
        style = ttk.Style()
        style.configure('Tactile.TNotebook', background=BG, borderwidth=0)
        style.configure('Tactile.TNotebook.Tab',
                         background=BTN_NEUTRAL, foreground=TEXT,
                         padding=(18, 8), font=FONT_LBL, borderwidth=0)
        style.map('Tactile.TNotebook.Tab',
                   background=[('selected', PANEL)],
                   foreground=[('selected', PRIMARY)])

        nb = ttk.Notebook(self.root, style='Tactile.TNotebook')
        nb.pack(fill='both', expand=True, padx=18, pady=18)

        tab_palp    = tk.Frame(nb, bg=BG)
        tab_man     = tk.Frame(nb, bg=BG)
        tab_lc      = tk.Frame(nb, bg=BG)
        tab_poses   = tk.Frame(nb, bg=BG)
        tab_sensors = tk.Frame(nb, bg=BG)
        nb.add(tab_palp,    text='Palpation')
        nb.add(tab_man,     text='Manual Control')
        nb.add(tab_lc,      text='Load Cell')
        nb.add(tab_poses,   text='Poses & Motions')
        # Abas adicionadas por último → não deslocam os índices usados pelo
        # gate (Palpação=0) nem o foco em Controle Manual (1).
        nb.add(tab_sensors, text='Sensors')
        tab_manip = None
        if _MANIP3D_OK:
            tab_manip = tk.Frame(nb, bg=BG)
            nb.add(tab_manip, text='3D Manipulation')

        # ttk.Progressbar foi removida (causava segfault com Canvas embed).
        self._build_palpation_tab(self._scrollable(tab_palp))
        self._build_manual_tab(self._scrollable(tab_man))
        self._build_loadcell_tab(tab_lc)   # sub-abas são scrolláveis internamente
        self._build_poses_tab(tab_poses)   # layout próprio — sem _scrollable externo
        # Aba Sensores: layout próprio (NÃO usar _scrollable — embutir um
        # canvas matplotlib dentro de um tk.Canvas scrollável é instável).
        self._build_sensors_tab(tab_sensors)
        self._sensors_tab_frame = tab_sensors
        # Guardadas para os gates de visibilidade dos loops de refresh: sem
        # elas o painel da célula (20 Hz) e o de status (10 Hz) repintavam
        # dezenas de widgets por tick com a aba escondida.
        self._palp_tab_frame = tab_palp
        self._man_tab_frame = tab_man
        self._lc_tab_frame = tab_lc
        # Manipulação 3D: layout próprio (a viewport precisa da altura toda —
        # nada de _scrollable, que fixaria a altura do conteúdo).
        if tab_manip is not None:
            self._build_manip3d_tab(tab_manip)
            self._manip_tab_frame = tab_manip

        # Gate do modo Palpação por end_effector REGRA (até o usuário pedir
        # o contrário): a aba/modo Palpação só fica disponível quando a
        # célula é aberta COM o touch_tool.
        self._nb = nb
        # A viewport 3D só roda seu tick (33 Hz) com a aba visível.
        nb.bind('<<NotebookTabChanged>>', self._on_tab_changed, add='+')
        self._palpation_blocked = (self._end_effector != 'touch_tool')
        if self._palpation_blocked:
            nb.tab(0, text='Palpation ⊘', state='disabled')
        # Modo hand: sem célula de carga → esconde a aba dedicada (a coluna
        # da mão já ocupa o Controle Manual).
        if self._end_effector == 'hand':
            try:
                nb.hide(tab_lc)
            except Exception:
                pass
        if self._palpation_blocked:
            try:
                nb.select(1)   # foca em Controle Manual
            except Exception:
                pass

    def _scrollable(self, parent: tk.Frame) -> tk.Frame:
        """Envolve `parent` num Canvas com scrollbar vertical e retorna o
        Frame interno onde o caller deve montar o conteúdo. A largura do
        frame interno acompanha a largura do canvas (responsivo) e a
        scrollregion atualiza quando o conteúdo cresce/encolhe.
        """
        canvas = tk.Canvas(parent, bg=BG, highlightthickness=0,
                            borderwidth=0)
        vbar = ttk.Scrollbar(parent, orient='vertical',
                              command=canvas.yview)
        canvas.configure(yscrollcommand=vbar.set)
        canvas.pack(side='left', fill='both', expand=True)
        vbar.pack(side='right', fill='y')

        inner = tk.Frame(canvas, bg=BG)
        win = canvas.create_window((0, 0), window=inner, anchor='nw')

        def _on_inner(_e):
            canvas.configure(scrollregion=canvas.bbox('all'))
        inner.bind('<Configure>', _on_inner)

        def _on_canvas(e):
            canvas.itemconfigure(win, width=e.width)
        canvas.bind('<Configure>', _on_canvas)

        # Mousewheel só rola se o ponteiro estiver sobre este canvas — bind
        # local via <Enter>/<Leave> evita capturar scroll de outras abas.
        def _wheel(e):
            delta = 1 if e.num == 5 or e.delta < 0 else -1
            canvas.yview_scroll(delta, 'units')
        canvas.bind('<Enter>',
                     lambda _e: (canvas.bind_all('<MouseWheel>', _wheel),
                                  canvas.bind_all('<Button-4>', _wheel),
                                  canvas.bind_all('<Button-5>', _wheel)))
        canvas.bind('<Leave>',
                     lambda _e: (canvas.unbind_all('<MouseWheel>'),
                                  canvas.unbind_all('<Button-4>'),
                                  canvas.unbind_all('<Button-5>')))
        return inner

    def _build_palpation_tab(self, root: tk.Frame):
        body = tk.Frame(root, bg=BG)
        body.pack(fill='both', expand=True)

        col_left  = tk.Frame(body, bg=BG)
        col_right = tk.Frame(body, bg=BG)
        col_left.pack(side='left', fill='both', expand=True, padx=(0, 9))
        col_right.pack(side='left', fill='both', expand=True, padx=(9, 0))

        params_card = self._card(col_left, 'Palpation Parameters')

        # Defaults vêm do último start persistido (PALPATION_PARAMS_FILE);
        # sem arquivo, os defaults de fábrica.
        sv = self._palp_saved

        def _f(key, default):
            try:
                return float(sv.get(key, default))
            except (TypeError, ValueError):
                return default

        self.speed_var      = tk.DoubleVar(value=_f('speed', SPEED_DEFAULT))
        self.depth_var      = tk.DoubleVar(value=_f('depth', DEPTH_DEFAULT))
        # Força aceita décimos de newton (0.5, 0.6, 0.7 …) — o slider faz
        # snap para múltiplos de 0.1 via `snap=0.1` no _param_row.
        self.force_sp_var   = tk.DoubleVar(value=_f('force_sp',
                                                    FORCE_SP_DEFAULT))
        self.slide_dist_var = tk.DoubleVar(value=_f('slide_dist',
                                                    SLIDE_DIST_DEFAULT))
        self.approach_var   = tk.DoubleVar(value=_f('approach',
                                                    APPROACH_DEFAULT))
        self.slide_dir_var  = tk.StringVar(
            value=str(sv.get('slide_dir', '+Y')))
        self.repeats_var    = tk.IntVar(value=int(_f('repeats',
                                                     REPEAT_DEFAULT)))
        # Modo de palpação: 'SLIDE' (deslizamento) | 'TOUCH' (toque) |
        # 'MANUAL' (dinâmico: HOLD infinito com setpoint ao vivo).
        _mode0 = str(sv.get('mode', 'SLIDE')).upper()
        self.mode_var = tk.StringVar(
            value=_mode0 if _mode0 in ('SLIDE', 'TOUCH', 'MANUAL',
                                       'MATRIX_MAP') else 'SLIDE')
        # ── MATRIX_MAP — geometria da grade ──────────────────────────
        # Tudo em mm e relativo à ORIGEM, que o robô descobre no primeiro
        # contato.
        _shape0 = str(sv.get('matrix_shape', 'SQUARE')).upper()
        self.matrix_shape_var = tk.StringVar(
            value=_shape0 if _shape0 in MATRIX_SHAPES else 'SQUARE')
        # Dimensionamento: por passo (histórico) ou pelas dimensões do alvo.
        _sizing0 = str(sv.get('matrix_sizing', 'STEP')).upper()
        self.matrix_sizing_var = tk.StringVar(
            value=_sizing0 if _sizing0 in MATRIX_SIZING_MODES else 'STEP')
        # Ordem de visita. Default CORNERS: os 4 extremos são tocados antes
        # do resto, como conferência de registro (pedido do usuário).
        _path0 = str(sv.get('matrix_path', 'CORNERS')).upper()
        self.matrix_path_var = tk.StringVar(
            value=_path0 if _path0 in MATRIX_PATH_ORDERS else 'CORNERS')
        self.matrix_step_x_var = tk.DoubleVar(
            value=_f('matrix_step_x', MATRIX_STEP_DEFAULT))
        self.matrix_step_y_var = tk.DoubleVar(
            value=_f('matrix_step_y', MATRIX_STEP_DEFAULT))
        # Dimensões do alvo (mm), usadas só quando sizing == 'SIZE'.
        self.matrix_width_var = tk.DoubleVar(
            value=_f('matrix_width', MATRIX_SIZE_DEFAULT))
        self.matrix_height_var = tk.DoubleVar(
            value=_f('matrix_height', MATRIX_SIZE_DEFAULT))
        self.matrix_cols_var = tk.IntVar(
            value=int(_f('matrix_cols', MATRIX_N_DEFAULT)))
        self.matrix_rows_var = tk.IntVar(
            value=int(_f('matrix_rows', MATRIX_N_DEFAULT)))
        self.matrix_safe_z_var = tk.DoubleVar(
            value=_f('matrix_safe_z', MATRIX_SAFE_Z_MM_DEFAULT))
        self.matrix_transit_var = tk.DoubleVar(
            value=_f('matrix_transit', MATRIX_TRANSIT_MMS_DEFAULT))
        # Waypoint em execução (do /palpation/status) — acende no preview.
        self._matrix_live_index = 0
        # Em MANUAL, mexer no spinbox de força publica /palpation/set_force.
        self.force_sp_var.trace_add('write', self._on_force_live_change)
        # Estabilização do HOLD (defaults espelham o explorer).
        # A banda vem de constants.HOLD_TOL_N (4σ do ruído da célula), NÃO de
        # um 0,15 N cravado aqui: a PalpationStart sobrescreve o default do
        # explorer sempre que traz hold_tol_n > 0, e ela sempre traz — o
        # retune de 19/08/2026 nunca chegava a valer num run lançado da tela.
        # O default mostrado é a lei do explorer avaliada no setpoint corrente
        # (max(4σ, 5 % do alvo)), o mesmo número que ele usaria sozinho.
        self.hold_tol_var     = tk.DoubleVar(value=_f(
            'hold_tol', round(_hold_tol_n_for(
                _f('force_sp', FORCE_SP_DEFAULT)), 3)))
        self.hold_stable_var  = tk.DoubleVar(value=_f('hold_stable', 5.0))
        self.hold_timeout_var = tk.DoubleVar(value=_f('hold_timeout', 8.0))
        # Tetos do micro-passo quase-estático (defaults espelham o explorer:
        # _QS_DX_MAX_M = 10 µm, _QS_DF_HARD_N = 0.3 N).
        self.hold_dx_var      = tk.DoubleVar(value=_f('hold_dx_max', 100.0))
        self.hold_df_var      = tk.DoubleVar(value=_f('hold_df_max', 0.3))
        # Inclinação da superfície na direção do deslize (graus). Compensada
        # por geometria no SLIDING; 0 = plano horizontal (comportamento
        # anterior a 06/08/2026).
        self.slide_slope_var  = tk.DoubleVar(value=_f('slide_slope_deg', 0.0))

        # Seletor de modo (Toque / Deslizamento) — define quais parâmetros
        # ficam visíveis abaixo.
        self._build_palp_mode_selector(params_card)

        # Parâmetros essenciais — sempre visíveis (válidos em ambos os modos).
        self._param_row(params_card, label='Target Force (Setpoint)',
                         unit='N', var=self.force_sp_var,
                         vmin=FORCE_SP_MIN, vmax=FORCE_SP_MAX, step=0.1,
                         snap=0.1,
                         hint='Compression held during descent, '
                              'HOLD and sliding, in 0.1 N steps '
                              '(0.1–10 N). Measurement aborts if '
                              'it exceeds 15 N. In Manual mode, changing '
                              'this during the run retargets the hold live.')
        # ── FORÇA MODULADA (só modo TOUCH) ───────────────────────────
        # Em vez do setpoint constante, o HOLD segue
        #   F(t) = média + amplitude·trig(2πf·t),  média=(min+max)/2.
        # Os limites min/max são os editáveis abaixo; o explorer valida a
        # faixa (0,1–10 N) e a amplitude (≤ 5 N) e desliga com log [FMOD]
        # se o pedido não couber. Bloco montado como unidade e
        # mostrado/ocultado por _on_palp_mode, igual ao do deslizamento.
        _shape0 = str(sv.get('force_mod_shape', 'OFF')).upper()
        self.fmod_shape_var  = tk.StringVar(
            value=_shape0 if _shape0 in FMOD_SHAPES else 'OFF')
        # Default 2 Hz: o laço de controle roda a 33 Hz e a onda só é
        # fiel com ≥ 8 pontos por período (33/8 ≈ 4 Hz de teto).
        self.fmod_hz_var     = tk.DoubleVar(value=_f('force_mod_hz', 2.0))
        self.fmod_min_var    = tk.DoubleVar(value=_f('force_mod_min_n', 2.0))
        self.fmod_max_var    = tk.DoubleVar(value=_f('force_mod_max_n', 3.0))
        self.fmod_cycles_var = tk.IntVar(
            value=int(_f('force_mod_cycles', FMOD_CYCLES_DEFAULT)))
        self._fmod_group = tk.Frame(params_card, bg=PANEL)
        self._build_fmod_shape_selector(self._fmod_group)
        self._param_row(self._fmod_group, label='Modulation — Min Force',
                         unit='N', var=self.fmod_min_var,
                         vmin=FORCE_SP_MIN, vmax=FORCE_SP_MAX, step=0.1,
                         snap=0.1,
                         hint='Force at the trough of the wave. Together '
                              'with Max Force it sets the mean the HOLD '
                              'stabilizes at first, and the amplitude '
                              '(max−min)/2, capped at 5 N.')
        self._param_row(self._fmod_group, label='Modulation — Max Force',
                         unit='N', var=self.fmod_max_var,
                         vmin=FORCE_SP_MIN, vmax=FORCE_SP_MAX, step=0.1,
                         snap=0.1,
                         hint='Force at the peak of the wave. If it ends up '
                              'below Min Force the two are swapped; the '
                              'measurement aborts above 15 N either way.')
        self._param_row(self._fmod_group, label='Modulation — Frequency',
                         unit='Hz', var=self.fmod_hz_var,
                         vmin=FMOD_HZ_MIN, vmax=FMOD_HZ_MAX, step=0.1,
                         snap=0.1,
                         hint='Wave frequency. The ServoJ loop that drives '
                              'the real arm runs at 33 Hz by default, so the '
                              'trackable ceiling is 6.7 Hz (5 points per '
                              'cycle). Above that the explorer REFUSES the '
                              'wave instead of resampling it — raise both '
                              'mirror_node and tactile_explorer with '
                              'servoj_period_s to go faster. 10 Hz is the '
                              'hardware ceiling and needs servoj_period_s '
                              '= 0.020, the CR10 firmware minimum: at that '
                              'point a cycle IS 5 points, and the ~12% of '
                              'amplitude the interpolation eats is added '
                              'back to the command automatically.')
        self._param_row(self._fmod_group, label='Modulation — Cycles',
                         unit='×', var=self.fmod_cycles_var,
                         vmin=FMOD_CYCLES_MIN, vmax=FMOD_CYCLES_MAX, step=1,
                         integer=True,
                         hint='Periods per touch. The oscillation lasts '
                              'cycles / frequency seconds, and starts only '
                              'after the normal HOLD has settled at the '
                              'mean force.')
        # Preview da onda, mesmo papel do _step_preview_lbl da escada: mostra
        # média, amplitude, duração e pontos por período ANTES do start, para
        # os avisos não chegarem só depois de o braço já ter descido.
        self._fmod_preview_lbl = tk.Label(
            self._fmod_group, text='', font=FONT_SMALL, bg=PANEL,
            fg=TEXT_MUTED, anchor='w', justify='left', wraplength=520)
        self._fmod_preview_lbl.pack(fill='x', pady=(2, 4))
        for _v in (self.fmod_min_var, self.fmod_max_var,
                   self.fmod_hz_var, self.fmod_cycles_var,
                   self.force_sp_var):
            _v.trace_add('write', lambda *_a: self._update_fmod_preview())
        # ── DEGRAU (só modo MANUAL) ──────────────────────────────────
        # O hold infinito passa a percorrer patamares sozinho: sobe de Start
        # até Max de Step em Step, mede Dwell em cada um, e volta DESCENDO
        # pelos mesmos — a ida-e-volta é o que revela histerese/relaxação.
        # Step = 0 desliga a escada e o Manual volta ao hold infinito com
        # Target Force ao vivo.
        self.step_size_var  = tk.DoubleVar(value=_f('step_size_n', 0.0))
        self.step_start_var = tk.DoubleVar(value=_f('step_start_n', 0.5))
        self.step_max_var   = tk.DoubleVar(value=_f('step_max_n', 3.0))
        self.step_dwell_var = tk.DoubleVar(value=_f('step_dwell_s', 5.0))
        self._step_group = tk.Frame(params_card, bg=PANEL)
        self._param_row(self._step_group, label='Staircase — Step Size',
                         unit='N', var=self.step_size_var,
                         vmin=0.0, vmax=5.0, step=0.1, snap=0.1,
                         hint='Height of each force step. 0 disables the '
                              'staircase — Manual goes back to holding one '
                              'setpoint indefinitely, adjustable live.')
        self._param_row(self._step_group, label='Staircase — First Level',
                         unit='N', var=self.step_start_var,
                         vmin=FORCE_SP_MIN, vmax=FORCE_SP_MAX, step=0.1,
                         snap=0.1,
                         hint='Force of the first plateau (paper default: '
                              '0.5 N).')
        self._param_row(self._step_group, label='Staircase — Peak Level',
                         unit='N', var=self.step_max_var,
                         vmin=FORCE_SP_MIN, vmax=FORCE_SP_MAX, step=0.1,
                         snap=0.1,
                         hint='Highest plateau. Always reached exactly: if '
                              'the step does not divide the range evenly, '
                              'the last rising step is shortened to land on '
                              'it. Held once, then the run comes back down '
                              'through the same plateaus.')
        self._param_row(self._step_group, label='Staircase — Dwell per Level',
                         unit='s', var=self.step_dwell_var,
                         vmin=0.0, vmax=120.0, step=0.5, snap=0.5,
                         hint='How long each plateau is held inside the '
                              'tolerance band once settled. This is the '
                              'measurement window — the samples of a level '
                              'are the ones with that setpoint_n in the CSV.')
        self._step_preview_lbl = tk.Label(
            self._step_group, text='', font=FONT_SMALL, bg=PANEL,
            fg=TEXT_MUTED, anchor='w', justify='left', wraplength=520)
        self._step_preview_lbl.pack(fill='x', pady=(2, 4))
        for _v in (self.step_size_var, self.step_start_var,
                   self.step_max_var, self.step_dwell_var):
            _v.trace_add('write', lambda *_a: self._update_step_preview())

        self._row_repeats = self._param_row(
                         params_card, label='Experiment Repetitions',
                         unit='×', var=self.repeats_var,
                         vmin=REPEAT_MIN, vmax=REPEAT_MAX, step=1,
                         integer=True,
                         hint='How many full cycles (descent → '
                              'slide → retract) to run back-to-back '
                              'automatically. The phase shows the current cycle.')
        # Referência ao label de repetições (relabel dinâmico por modo).
        self._repeats_lbl = self._row_repeats.winfo_children()[0].winfo_children()[0]

        # Bloco de parâmetros exclusivos do deslizamento — mostrado/ocultado
        # como uma unidade conforme o modo (preserva a ordem ao reaparecer).
        self._slide_group = tk.Frame(params_card, bg=PANEL)
        self._param_row(self._slide_group, label='Sliding Speed',
                         unit='mm/s', var=self.speed_var,
                         vmin=SPEED_MIN, vmax=SPEED_MAX, step=1.0,
                         hint='Paper reference values: 5, 10, 15 mm/s')
        self._param_row(self._slide_group, label='Sliding Distance',
                         unit='mm', var=self.slide_dist_var,
                         vmin=SLIDE_DIST_MIN, vmax=SLIDE_DIST_MAX, step=5.0,
                         hint='Length of the lateral path. '
                              'Safety maximum: 300 mm.')
        self._build_slide_dir_selector(self._slide_group)

        # Configurador visual da grade (MATRIX_MAP) — mesma mecânica do bloco
        # de deslizamento: um Frame mostrado/ocultado como unidade por modo.
        self._matrix_group = tk.Frame(params_card, bg=PANEL)
        self._build_matrix_group(self._matrix_group)

        # ── CALIBRAÇÃO DO ÂNGULO DE ATAQUE ───────────────────────────
        # Desligada por padrão: ligá-la muda a aproximação de TODO run, e a
        # sondagem custa N descidas extras antes da medição. Válida em todos
        # os modos, por isso mora nos avançados e não num grupo por modo.
        self.align_on_var      = tk.BooleanVar(
            value=bool(sv.get('probe_align_on', False)))
        self.align_points_var  = tk.IntVar(
            value=int(_f('probe_align_points', PROBE_ALIGN_POINTS_DEFAULT)))
        self.align_radius_var  = tk.DoubleVar(
            value=_f('probe_align_radius_mm', PROBE_ALIGN_RADIUS_MM_DEFAULT))
        self.align_force_var   = tk.DoubleVar(
            value=_f('probe_align_force_n', PROBE_ALIGN_FORCE_N_DEFAULT))
        self.align_retract_var = tk.DoubleVar(
            value=_f('probe_align_retract_mm',
                     PROBE_ALIGN_RETRACT_MM_DEFAULT))
        self.align_tilt_var    = tk.DoubleVar(
            value=_f('probe_align_tilt_max_deg',
                     PROBE_ALIGN_TILT_MAX_DEG_DEFAULT))

        # Parâmetros avançados — recolhidos por padrão (segurança + micro-passo).
        adv = self._collapsible(params_card, 'Advanced parameters')
        # Âncora para reempacotar o bloco de deslizamento antes dos
        # avançados.
        self._adv_frame = adv.master

        # Aplica visibilidade inicial conforme o modo carregado.
        self._on_palp_mode(self.mode_var.get())
        # A velocidade de aproximação saiu da GUI em 04/07: a regulação de força
        # é por micro-passos quase-estáticos e a velocidade de descida é
        # governada pelas constantes do explorer.
        self._param_row(adv, label='Max Descent Depth',
                         unit='mm', var=self.depth_var,
                         vmin=DEPTH_MIN, vmax=DEPTH_MAX, step=0.5,
                         hint='Maximum safe travel — the descent stops '
                              'earlier, when the Target Force is reached.')
        self._param_row(adv, label='Descent Speed',
                         unit='mm/s', var=self.approach_var,
                         vmin=APPROACH_MIN, vmax=APPROACH_MAX, step=1.0,
                         hint='Free-air descent speed (PROBE phase), in mm/s. '
                              'The arm descends continuously at this rate; at '
                              'the first force reading (> 0.05 N) it HALTS '
                              'immediately, relieves the inertia spike by '
                              'backing off (RELAX), then closes on the '
                              'setpoint in 10-20 um micro-steps (FINE). '
                              'Faster = more inertia spike to relieve; the '
                              'committed course cap keeps it under the 12 N '
                              'safety margin either way.')
        self._param_row(adv, label='HOLD — Band Tolerance',
                         unit='N', var=self.hold_tol_var,
                         vmin=round(_HOLD_TOL_N, 3), vmax=2.0, step=0.01,
                         hint='Half-width of the band around the setpoint '
                              'within which the force is considered '
                              'stabilized. The floor is the load cell noise '
                              f'itself ({_HOLD_TOL_SIGMA:.0f}σ = '
                              f'{_HOLD_TOL_N:.3f} N, σ='
                              f'{_FORCE_NOISE_SIGMA_N:.3f} N): a band '
                              'narrower than the measurement uncertainty is '
                              'not a tighter criterion, it is one the cell '
                              'cannot evaluate. The explorer also floors it '
                              f'at {100*_HOLD_TOL_PCT:.0f}% of the setpoint; '
                              'the default shown is that same law. Re-measure '
                              'σ with the FA7155 to lower this floor.')
        self._param_row(adv, label='HOLD — Stable Window',
                         unit='s', var=self.hold_stable_var,
                         vmin=0.2, vmax=5.0, step=0.1,
                         hint='CONTINUOUS time inside the band required to '
                              'accept the setpoint as reached. Leaving the '
                              'band restarts the count.')
        self._param_row(adv, label='HOLD — Timeout',
                         unit='s', var=self.hold_timeout_var,
                         vmin=2.0, vmax=60.0, step=1.0,
                         hint='Maximum wait for stabilization. On expiry '
                              'the experiment proceeds with a warning.')
        self._param_row(adv, label='HOLD — Max Micro-step',
                         unit='µm', var=self.hold_dx_var,
                         vmin=1.0, vmax=500.0, step=5.0,
                         hint='Absolute cap of each quasi-static correction '
                              'step during HOLD/FINE (explorer default: '
                              '100 µm). One step is executed per ~180 ms '
                              'cycle, so 100 µm ≈ 0.5 mm/s effective. This '
                              'is only a safety ceiling — what limits the '
                              'step in practice is Max ΔF per Step divided '
                              'by the measured stiffness. Raise it for soft '
                              'tips (silicone ≈ 0.6 N/mm needs ~100 µm to '
                              'move 0.06 N); on a stiff contact the ΔF cap '
                              'bites first and this has no effect.')
        self._param_row(adv, label='HOLD — Max ΔF per Step',
                         unit='N', var=self.hold_df_var,
                         vmin=0.05, vmax=1.0, step=0.05,
                         hint='Hard cap of the projected force change per '
                              'micro-step, boost included (explorer '
                              'default: 0.3 N). This is what actually '
                              'limits the step once the stiffness is '
                              'estimated.')
        self._param_row(adv, label='Slide Slope (surface tilt)',
                         unit='°', var=self.slide_slope_var,
                         vmin=-10.0, vmax=10.0, step=0.1, snap=0.1,
                         hint='Tilt of the sample surface ALONG the sliding '
                              'direction (+ = surface rises as the probe '
                              'advances). SLIDING stops locking Z on a '
                              'horizontal plane and follows this ramp '
                              'instead. It is pure geometry — no force '
                              'feedback — so texture is untouched. Leave at '
                              '0 for a levelled sample. A drop in force may '
                              'be the texture itself (a groove or a step), so '
                              'the run NEVER aborts on lost contact: it logs '
                              'CONTATO PERDIDO once and finishes normally. '
                              'Scale is what tells them apart — a short dip '
                              'is a surface feature, a long continuous '
                              'stretch means the sample left the slide plane. '
                              'For that case the log prints the angle to '
                              'enter here, though shimming it flat is the '
                              'better fix.')

        # ── Calibração do ângulo de ataque ───────────────────────────
        self._align_chk = tk.Checkbutton(
            adv, text='Probe surface & align attack angle',
            variable=self.align_on_var, command=self._on_align_toggle,
            bg=PANEL, fg=TEXT, activebackground=PANEL, activeforeground=TEXT,
            selectcolor=PANEL, font=FONT_LBL, anchor='w', relief='flat',
            bd=0, highlightthickness=0, cursor='hand2')
        self._align_chk.pack(fill='x', pady=(10, 2))
        _Tooltip(self._align_chk,
                 'OFF (default): the probe descends straight down along world '
                 '-Z and assumes the target is perpendicular to Home. ON: '
                 'before the run, the arm makes N light touches around the '
                 'approach point, fits the surface plane and re-aims the tool '
                 'along the MEASURED normal — so the tip lands flat instead '
                 'of on an edge. Costs N extra descents, once per experiment '
                 '(not per repeat). It ABORTS the run if the fitted plane is '
                 'not trustworthy: tilt over the limit, near-collinear '
                 'contacts, or a residual too large to be a plane. In Slide '
                 'mode it also overrides Slide Slope with the measured ramp.')
        # Grupo dos ajustes — só faz sentido com a calibração ligada, então
        # aparece junto com ela (mesmo padrão dos grupos por modo).
        self._align_group = tk.Frame(adv, bg=PANEL)
        self._param_row(self._align_group, label='Align — Probe Points',
                         unit='×', var=self.align_points_var,
                         vmin=PROBE_ALIGN_POINTS_MIN,
                         vmax=PROBE_ALIGN_POINTS_MAX, step=1, integer=True,
                         hint='Number of light touches, arranged on a regular '
                              'polygon around the approach point. 3 is the '
                              'geometric minimum and gives the exact plane '
                              'through those points — with no way to tell a '
                              'bad touch from a good one. 4 or more switches '
                              'the fit to least squares, which filters the '
                              'mechanical noise of each touch and makes the '
                              'RMS residual meaningful. Each point costs one '
                              'full descent. Ignored in Matrix mode: there '
                              'the probing points are the four corners of '
                              'the grid you drew.')
        self._param_row(self._align_group, label='Align — Probe Radius',
                         unit='mm', var=self.align_radius_var,
                         vmin=PROBE_ALIGN_RADIUS_MM_MIN,
                         vmax=PROBE_ALIGN_RADIUS_MM_MAX, step=1.0,
                         hint='Radius of the probing polygon. This is the '
                              'lever arm of the fit: too small and the tilt '
                              'is buried in the noise of the touches, too '
                              'large and the probe walks off the sample. Keep '
                              'it inside the flat region you actually want to '
                              'measure. In Matrix mode it is only a fallback: '
                              'the probing ring is derived from the grid '
                              'itself (half a step outside its corners, drawn '
                              'in the preview), and the radius is used only '
                              'when that ring would be shorter than its own '
                              f'{PROBE_ALIGN_RADIUS_MM_MIN:.0f} mm floor.')
        self._param_row(self._align_group, label='Align — Probe Force',
                         unit='N', var=self.align_force_var,
                         vmin=FORCE_SP_MIN, vmax=FORCE_SETPOINT_MAX_N,
                         step=0.1, snap=0.1,
                         hint='Setpoint of each probing touch. Keep it light '
                              'so the probing does not mark the sample before '
                              'the real measurement — what makes the fitted '
                              'plane parallel to the real surface is that the '
                              'penetration is the SAME at every point, not '
                              'its value. Never exceeds Target Force: the '
                              'explorer saturates it. It also has a floor of '
                              '0.33 N in the explorer (3x the contact '
                              'threshold): below that the fit would be reading '
                              'load-cell noise, so lower values are raised '
                              'back to it.')
        self._param_row(self._align_group, label='Align — Retract Before Turn',
                         unit='mm', var=self.align_retract_var,
                         vmin=PROBE_ALIGN_RETRACT_MM_MIN,
                         vmax=PROBE_ALIGN_RETRACT_MM_MAX, step=1.0,
                         hint='Straight-line retraction before the wrist '
                              'rotates. The wrist turns about the joint, not '
                              'about the tip, so the tip sweeps an arc of '
                              'radius ~68 mm (the tool length). Without '
                              'backing off first that arc goes THROUGH the '
                              'part and shears the tip. Increase it for a '
                              'larger correction angle.')
        self._param_row(self._align_group, label='Align — Max Tilt',
                         unit='°', var=self.align_tilt_var,
                         vmin=1.0, vmax=PROBE_ALIGN_TILT_HARD_MAX_DEG,
                         step=1.0,
                         hint='Safety limit on the measured deviation from '
                              'vertical. Above it the run ABORTS instead of '
                              'turning the wrist: that much tilt is a '
                              'mounting problem (shim, fixture) and the place '
                              'to fix it is the bench, not the software. '
                              f'Hard-capped at {PROBE_ALIGN_TILT_HARD_MAX_DEG:.0f}° — '
                              'beyond that J5 runs out of useful range and '
                              'the rotation would sweep the part.')
        self._on_align_toggle()

        # Coluna direita: botão de início (fixado no fundo) + feedback FT
        # O botão é empacotado primeiro com side='bottom' para ficar visível
        # independente do tamanho da janela; o fb_card preenche o restante.
        btn_wrap = tk.Frame(col_right, bg=BG)
        btn_wrap.pack(fill='x', side='bottom', pady=(14, 0))
        self.stop_palp_btn = tk.Button(
            btn_wrap, text='■  Stop Palpation',
            command=self._on_stop_palpation, bg=WARN, fg='white',
            activebackground=_shade(WARN, -0.1), activeforeground='white',
            font=FONT_HEAD, relief='flat', bd=0, padx=18, pady=10,
            cursor='hand2')
        self.stop_palp_btn.pack(fill='x', pady=(0, 6))
        # ⏸/▶ — pausa segura: o explorer congela a posição atual e, em modo
        # MIRROR, o braço real recebe pause()/resume() do driver.
        self.pause_btn = tk.Button(
            btn_wrap, text='⏸  Pause',
            command=self._toggle_pause, bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=_shade(BTN_NEUTRAL, -0.08), activeforeground=TEXT,
            font=FONT_HEAD, relief='flat', bd=0, padx=18, pady=10,
            cursor='hand2')
        self.pause_btn.pack(fill='x', pady=(0, 6))
        self.start_btn = tk.Button(
            btn_wrap, text=('▶  Start Touch'
                            if self.mode_var.get() == 'TOUCH'
                            else ('▶  Start Manual'
                                  if self.mode_var.get() == 'MANUAL'
                                  else '▶  Start Palpation')),
            command=self._on_start, bg=PRIMARY, fg='white',
            activebackground=PRIMARY_HV, activeforeground='white',
            font=FONT_HEAD, relief='flat', bd=0, padx=18, pady=12,
            cursor='hand2')
        self.start_btn.pack(fill='x')

        # Botão pequeno: grava força + toque sincronizados em CSV
        # Independe de "Iniciar Palpação": amostra a 50 Hz um snapshot único
        # (mesmo timestamp) da célula de carga e do touch sensor.
        rec_row = tk.Frame(btn_wrap, bg=BG)
        rec_row.pack(fill='x', pady=(6, 0))
        self.rec_btn = tk.Button(
            rec_row, text='●  Record data (force+touch)',
            command=self._toggle_recording, bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=_shade(BTN_NEUTRAL, -0.08), activeforeground=TEXT,
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2')
        self.rec_btn.pack(side='left')
        self.rec_status_lbl = tk.Label(
            rec_row, text='', font=FONT_SMALL, bg=BG, fg=TEXT_DIM)
        self.rec_status_lbl.pack(side='left', padx=(8, 0))

        fb_card = self._card(col_right,
                              'Load Cell — Contact Force (N / kgf)')

        fnrow = tk.Frame(fb_card, bg=PANEL)
        fnrow.pack(fill='x', pady=(6, 4))
        tk.Label(fnrow, text='Compression Force (tare-compensated)',
                 font=FONT_LBL, bg=PANEL, fg=TEXT_MUTED).pack(anchor='w')
        self.force_value_lbl = tk.Label(
            fnrow, text='—   N', font=FONT_BIG, bg=PANEL, fg=TEXT_DIM)
        self.force_value_lbl.pack(anchor='w', pady=(2, 2))
        # Mesma leitura em kgf, lado a lado com os N (ver _N_PER_KGF).
        self.force_kgf_lbl = tk.Label(
            fnrow, text='—   kgf', font=FONT_HEAD, bg=PANEL, fg=TEXT_DIM)
        self.force_kgf_lbl.pack(anchor='w', pady=(0, 2))
        self.force_status_lbl = tk.Label(
            fnrow, text='waiting for /load_cell/force_net',
            font=FONT_LBL, bg=PANEL, fg=TEXT_DIM)
        self.force_status_lbl.pack(anchor='w')

        tk.Frame(fb_card, bg=BORDER, height=1).pack(fill='x', pady=8)
        errrow = tk.Frame(fb_card, bg=PANEL)
        errrow.pack(fill='x', pady=(2, 6))
        tk.Label(errrow, text='Target force (setpoint)', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(side='left')
        self.err_value_lbl = tk.Label(
            errrow, text='—  N', font=FONT_HEAD, bg=PANEL, fg=TEXT)
        self.err_value_lbl.pack(side='right')

        compbox = tk.Frame(fb_card, bg=PANEL)
        compbox.pack(fill='x', pady=(2, 6))
        self.fz_lbl  = self._kv(compbox, 'F net (LC)',   '0.00 N')
        self.fkgf_lbl = self._kv(compbox, 'F net (LC)',  '0.000 kgf')
        self.fx_lbl  = self._kv(compbox, 'Tare',         '—')
        self.fy_lbl  = self._kv(compbox, 'LC raw',     '0.00 N')

        # Odômetro do TCP Distância percorrida DESDE O INÍCIO DA FASE atual
        # (FK do TCP): no DESCENDING é a profundidade descida, no SLIDING é
        # o curso lateral — a leitura útil em cada uma das duas.
        tk.Frame(fb_card, bg=BORDER, height=1).pack(fill='x', pady=8)
        distrow = tk.Frame(fb_card, bg=PANEL)
        distrow.pack(fill='x', pady=(2, 6))
        tk.Label(distrow, text='Distance travelled (this phase)',
                 font=FONT_LBL, bg=PANEL, fg=TEXT_MUTED).pack(side='left')
        self.dist_value_lbl = tk.Label(
            distrow, text='—  mm', font=FONT_HEAD, bg=PANEL, fg=TEXT)
        self.dist_value_lbl.pack(side='right')
        distbox = tk.Frame(fb_card, bg=PANEL)
        distbox.pack(fill='x', pady=(0, 6))
        self.dist_z_lbl = self._kv(distbox, 'TCP Z (world)', '—  mm')

        tk.Frame(fb_card, bg=BORDER, height=1).pack(fill='x', pady=8)
        prow = tk.Frame(fb_card, bg=PANEL)
        prow.pack(fill='x')
        tk.Label(prow, text='Experiment Phase', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(side='left')
        self.phase_lbl = tk.Label(
            prow, text='IDLE', font=FONT_HEAD, bg=PANEL, fg=TEXT)
        self.phase_lbl.pack(side='right')

        # Cronômetro só com label — sem ttk.Progressbar.
        timerow = tk.Frame(fb_card, bg=PANEL)
        timerow.pack(fill='x', pady=(8, 2))
        tk.Label(timerow, text='Progress', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(side='left')
        self.timer_lbl = tk.Label(
            timerow, text='—', font=FONT_HEAD, bg=PANEL, fg=TEXT_MUTED)
        self.timer_lbl.pack(side='right')

        # Sparkline da força (últimos 30 s)
        # tk.Canvas com desenho puro (linhas) — sem fontes novas, portanto
        # imune ao bug do fontconfig descrito em ui_helpers.
        tk.Frame(fb_card, bg=BORDER, height=1).pack(fill='x', pady=8)
        tk.Label(fb_card, text='Force — last 30 s', font=FONT_SMALL,
                 bg=PANEL, fg=TEXT_MUTED, anchor='w').pack(fill='x')
        self.spark_canvas = tk.Canvas(
            fb_card, height=64, bg=PANEL, highlightthickness=1,
            highlightbackground=BORDER)
        self.spark_canvas.pack(fill='x', pady=(4, 2))

        # Sparkline do touch sensor (STM32, últimos 30 s)
        # Mesmo desenho em Canvas puro do gráfico da célula acima.
        tk.Frame(fb_card, bg=BORDER, height=1).pack(fill='x', pady=8)
        touch_hdr = tk.Frame(fb_card, bg=PANEL)
        touch_hdr.pack(fill='x')
        tk.Label(touch_hdr, text='Touch Sensor — last 30 s',
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_MUTED,
                 anchor='w').pack(side='left')
        self.touch_value_lbl = tk.Label(
            touch_hdr, text='—', font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM)
        self.touch_value_lbl.pack(side='right')
        self.touch_spark_canvas = tk.Canvas(
            fb_card, height=64, bg=PANEL, highlightthickness=1,
            highlightbackground=BORDER)
        self.touch_spark_canvas.pack(fill='x', pady=(4, 2))
        self.touch_status_lbl = tk.Label(
            fb_card, text='waiting for /touch_sensor/value',
            font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w')
        self.touch_status_lbl.pack(fill='x')

    # Aba "Controle Manual"
    def _build_manual_tab(self, root: tk.Frame):
        """Constrói a aba de jog manual: tempo de movimento + 6 sliders do
        braço + 6 sliders da mão.
        """
        body = tk.Frame(root, bg=BG)
        body.pack(fill='both', expand=True)

        # Top: controle de velocidade (SpeedFactor %)
        speed_wrap = tk.Frame(body, bg=BG)
        speed_wrap.pack(fill='x', pady=(0, 10))
        speed_inner = self._card(speed_wrap, 'Motion Speed',
                                 expand=False)

        self.speed_factor_var = tk.DoubleVar(value=SPEED_FACTOR_DEFAULT)
        self._param_row(speed_inner, label='Speed', unit='%',
                        var=self.speed_factor_var,
                        vmin=SPEED_FACTOR_MIN, vmax=SPEED_FACTOR_MAX, step=1,
                        hint='Affects manual jog (MovJ/PTP) and Gazebo duration. '
                             'Real COVVI hand has a firmware minimum of 15% — '
                             'values below are clamped to 15. '
                             'Does NOT affect streaming during palpation (CONTACT/'
                             'CALIBRATING/SLIDING/RETRACT use ServoJ with their '
                             'own speed set in the parameters above).')
        self.speed_factor_var.trace_add(
            'write', lambda *_: self._apply_speed_factor_if_active())

        cols = tk.Frame(body, bg=BG)
        cols.pack(fill='both', expand=True)
        col_arm  = tk.Frame(cols, bg=BG)
        col_hand = tk.Frame(cols, bg=BG)
        col_arm.pack(side='left', fill='both', expand=True, padx=(0, 9))
        col_hand.pack(side='left', fill='both', expand=True, padx=(9, 0))

        # BRAÇO CR10
        card_arm = self._card(col_arm, 'CR10 Arm — joints (degrees)')
        self.arm_sliders: dict[str, tk.DoubleVar] = {}
        for j in ARM_JOINTS:
            lo, hi = ARM_LIMITS_DEG[j]
            var = tk.DoubleVar(value=self._arm_home_deg[j])
            self.arm_sliders[j] = var
            self._joint_row(card_arm, label=j, unit='°',
                              var=var, vmin=lo, vmax=hi, step=1.0,
                              on_change=self._publish_arm_from_sliders)

        btns_arm = tk.Frame(col_arm, bg=BG)
        btns_arm.pack(fill='x', pady=(10, 0))
        tk.Button(btns_arm, text='⌂  Home',
                   command=self._apply_arm_home,
                   bg=PRIMARY, fg='white',
                   activebackground=PRIMARY_HV, activeforeground='white',
                   font=FONT_LBL, relief='flat', bd=0, padx=14, pady=8,
                   cursor='hand2'
                   ).pack(side='left', fill='x', expand=True, padx=(0, 4))
        # ✔ = grava os ângulos atuais como nova Home (persiste em JSON).
        tk.Button(btns_arm, text='✔  Save Home',
                   command=self._save_home_pose,
                   bg=OK, fg='white',
                   activebackground=_shade(OK, -0.08),
                   activeforeground='white',
                   font=FONT_LBL, relief='flat', bd=0, padx=14, pady=8,
                   cursor='hand2'
                   ).pack(side='left', fill='x', expand=True, padx=(4, 0))

        btns_arm2 = tk.Frame(col_arm, bg=BG)
        btns_arm2.pack(fill='x', pady=(4, 0))
        tk.Button(btns_arm2, text='⌖  Capture from Robot',
                   command=self._capture_arm_from_robot,
                   bg=_shade(PRIMARY, 0.25), fg=PRIMARY,
                   activebackground=_shade(PRIMARY, 0.15),
                   activeforeground=PRIMARY,
                   font=FONT_LBL, relief='flat', bd=0, padx=14, pady=6,
                   cursor='hand2'
                   ).pack(side='left', fill='x', expand=True, padx=(0, 4))
        # ⊥ = solver de pulso: ajusta joint4/joint5 para o TCP ficar
        # exatamente perpendicular à mesa, mantendo joint1-3 e joint6.
        tk.Button(btns_arm2, text='⊥  TCP ⊥ Table',
                   command=self._solve_tcp_perpendicular,
                   bg=_shade(OK, 0.25), fg=OK,
                   activebackground=_shade(OK, 0.15),
                   activeforeground=OK,
                   font=FONT_LBL, relief='flat', bd=0, padx=14, pady=6,
                   cursor='hand2'
                   ).pack(side='left', fill='x', expand=True, padx=(4, 0))

        # Coluna direita: adapta ao efetuador final
        #   hand       → controle da mão COVVI (sliders + presets + grips)
        #   touch_tool → leitura ao vivo da célula de carga
        if self._end_effector == 'hand':
            self._build_manual_hand_controls(col_hand)
        else:
            self._build_manual_lc_panel(col_hand)

    def _build_manual_hand_controls(self, col_hand: tk.Frame) -> None:
        """Coluna direita do Controle Manual no modo `hand`: sliders da mão
        COVVI + presets (Abrir/Apontar/Fechar) + grips de fábrica."""
        # MÃO COVVI
        card_hand = self._card(col_hand, 'COVVI Hand — primary joints (degrees)')
        self.hand_sliders: dict[str, tk.DoubleVar] = {}
        for j in HAND_JOINTS:
            lo, hi = HAND_LIMITS_DEG[j]
            var = tk.DoubleVar(value=0)
            self.hand_sliders[j] = var
            self._joint_row(card_hand, label=j, unit='°',
                              var=var, vmin=lo, vmax=hi, step=1.0,
                              on_change=self._publish_hand_from_sliders)

        btns_hand = tk.Frame(col_hand, bg=BG)
        btns_hand.pack(fill='x', pady=(10, 0))
        tk.Button(btns_hand, text='✋  Open',
                   command=lambda: self._apply_hand_preset(
                       HAND_OPEN_DEG, eci_grip_id=11),   # 11 = GLOVE
                   bg=BTN_NEUTRAL, fg=TEXT,
                   activebackground=_shade(BTN_NEUTRAL, -0.08),
                   activeforeground=TEXT,
                   font=FONT_LBL, relief='flat', bd=0, padx=12, pady=8,
                   cursor='hand2'
                   ).pack(side='left', fill='x', expand=True, padx=(0, 3))
        tk.Button(btns_hand, text='☞  Point',
                   command=lambda: self._apply_hand_preset(
                       HAND_POINT_DEG, eci_grip_id=7),    # 7 = FINGER (Index ext.)
                   bg=OK, fg='white',
                   activebackground=_shade(OK, -0.08),
                   activeforeground='white',
                   font=FONT_LBL, relief='flat', bd=0, padx=12, pady=8,
                   cursor='hand2'
                   ).pack(side='left', fill='x', expand=True, padx=3)
        tk.Button(btns_hand, text='✊  Close',
                   command=lambda: self._apply_hand_preset(
                       HAND_CLOSE_DEG, eci_grip_id=2),    # 2 = POWER
                   bg=PRIMARY, fg='white',
                   activebackground=PRIMARY_HV, activeforeground='white',
                   font=FONT_LBL, relief='flat', bd=0, padx=12, pady=8,
                   cursor='hand2'
                   ).pack(side='left', fill='x', expand=True, padx=(3, 0))

        # Grip-patterns COVVI (padrões de pega de fábrica)
        grips_card = self._card(col_hand, 'COVVI Grips — factory grip patterns')
        grow = tk.Frame(grips_card, bg=PANEL); grow.pack(fill='x')
        self._covvi_grip_var = tk.StringVar(value=next(iter(COVVI_GRIPS)))
        # tk.OptionMenu (Tk puro) em vez de ttk.Combobox: o ttk.Combobox
        # embutido neste Canvas scrollable corrompia o estado interno do Tk
        # e provocava segfault na criação de widgets (mesmo problema que
        # levou à remoção da ttk.Progressbar — ver _build_body).
        grip_menu = tk.OptionMenu(grow, self._covvi_grip_var, *COVVI_GRIPS.keys())
        grip_menu.config(bg=BTN_NEUTRAL, fg=TEXT, font=FONT_MONO,
                         relief='flat', highlightthickness=1,
                         highlightbackground=BORDER,
                         activebackground=PRIMARY, activeforeground='white')
        grip_menu['menu'].config(bg=PANEL, fg=TEXT, font=FONT_MONO,
                                 activebackground=PRIMARY, activeforeground='white')
        grip_menu.pack(side='left', fill='x', expand=True, ipady=2)
        apply_btn = tk.Button(
            grow, text='✓  Apply', command=self._apply_covvi_grip,
            bg=PRIMARY, fg='white', activebackground=PRIMARY_HV,
            activeforeground='white', font=FONT_LBL, relief='flat',
            bd=0, padx=12, pady=6, cursor='hand2')
        apply_btn.pack(side='left', padx=(6, 0))
        _Tooltip(apply_btn,
                 'Moves the sim (joints) + sends SetCurrentGrip to the real hand (ECI).')

    def _build_manual_lc_panel(self, col_hand: tk.Frame) -> None:
        """Coluna direita do Controle Manual no modo `touch_tool`: leitura ao
        vivo da célula de carga (espelha _refresh_lc_panel). É read-only — a
        conexão do receptor, a zeragem (tare) e a calibração ficam na aba
        dedicada "Célula de Carga" para manter este painel enxuto."""
        card = self._card(col_hand, 'Load Cell — live reading')

        row_f = tk.Frame(card, bg=PANEL); row_f.pack(fill='x', pady=(6, 2))
        tk.Label(row_f, text='Total Force (calibration)', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(anchor='w')
        self._mlc_force_lbl = tk.Label(
            row_f, text='—   N', font=FONT_BIG, bg=PANEL, fg=TEXT_DIM)
        self._mlc_force_lbl.pack(anchor='w', pady=(2, 0))

        tk.Frame(card, bg=BORDER, height=1).pack(fill='x', pady=6)

        row_n = tk.Frame(card, bg=PANEL); row_n.pack(fill='x', pady=(2, 2))
        tk.Label(row_n, text='Normal Force ⊥ table  (+compression / −tension)',
                 font=FONT_LBL, bg=PANEL, fg=TEXT_MUTED).pack(anchor='w')
        self._mlc_normal_lbl = tk.Label(
            row_n, text='—   N', font=FONT_BIG, bg=PANEL, fg=TEXT_DIM)
        self._mlc_normal_lbl.pack(anchor='w', pady=(2, 0))

        tk.Frame(card, bg=BORDER, height=1).pack(fill='x', pady=6)

        row_v = tk.Frame(card, bg=PANEL); row_v.pack(fill='x', pady=(2, 2))
        tk.Label(row_v, text='Sensor Voltage', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(side='left')
        self._mlc_voltage_lbl = tk.Label(
            row_v, text='—  V', font=FONT_MONO, bg=PANEL, fg=TEXT_DIM)
        self._mlc_voltage_lbl.pack(side='right')

        row_s = tk.Frame(card, bg=PANEL); row_s.pack(fill='x', pady=(2, 2))
        tk.Label(row_s, text='Board', font=FONT_LBL,
                 bg=PANEL, fg=TEXT_MUTED).pack(side='left')
        self._mlc_status_lbl = tk.Label(
            row_s, text='OFFLINE', font=FONT_LBL, bg=PANEL, fg=TEXT_DIM)
        self._mlc_status_lbl.pack(side='right')

        tk.Label(card,
                 text='Receiver connection, tare and calibration live in '
                      'the “Load Cell” tab.',
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_MUTED, anchor='w',
                 justify='left', wraplength=300
                 ).pack(fill='x', pady=(8, 0))

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

    def _joint_row(self, parent, *, label, unit, var,
                    vmin, vmax, step, on_change):
        """Linha compacta com label + spinbox + slider para uma junta.

        Conecta `var.trace` para que arrastar o slider OU digitar no
        spinbox dispare imediatamente o publish."""
        row = tk.Frame(parent, bg=PANEL); row.pack(fill='x', pady=(3, 1))
        top = tk.Frame(row, bg=PANEL); top.pack(fill='x')
        tk.Label(top, text=label, font=FONT_MONO_S, bg=PANEL, fg=TEXT,
                 width=10, anchor='w').pack(side='left')
        tk.Spinbox(top, from_=vmin, to=vmax, increment=step,
                    textvariable=var, width=7, font=FONT_MONO,
                    justify='right', relief='flat', bd=0,
                    highlightthickness=1, highlightbackground=BORDER,
                    highlightcolor=PRIMARY
                    ).pack(side='right', padx=(6, 0), ipady=2)
        tk.Label(top, text=unit, font=FONT_SMALL, bg=PANEL, fg=TEXT_MUTED
                 ).pack(side='right')
        ttk.Scale(row, from_=vmin, to=vmax, variable=var,
                   orient='horizontal',
                   style='Tactile.Horizontal.TScale'
                   ).pack(fill='x', pady=(1, 0))
        # `var.trace_add` dispara em qualquer mudança do valor.
        var.trace_add('write',
                       lambda *_a: (not self._suppressing) and on_change())

    # Clamp helpers
    def _clamp_var(self, var: tk.DoubleVar, vmin: float, vmax: float,
                    default: float | None = None) -> float | None:
        """Lê `var`, força-o ao intervalo [vmin, vmax] (re-escreve no var
        se necessário) e devolve o valor saneado. Retorna `default` (ou
        None) se a leitura falhar."""
        try:
            v = float(var.get())
        except (ValueError, tk.TclError):
            return default
        v_clamped = max(vmin, min(vmax, v))
        if v_clamped != v:
            var.set(v_clamped)
        return v_clamped

    def _move_duration_seconds(self) -> float:
        """Duração da trajetória Gazebo derivada do slider de velocidade.

        Inversamente proporcional à velocidade: 10 % → 3.0 s, 100 % → 0.3 s."""
        try:
            speed_pct = float(self.speed_factor_var.get())
            speed_pct = max(SPEED_FACTOR_MIN, min(SPEED_FACTOR_MAX, speed_pct))
        except (ValueError, tk.TclError):
            speed_pct = SPEED_FACTOR_DEFAULT
        return max(0.3, _VEL_BASE_S * (10.0 / speed_pct))

    def _apply_speed_factor_if_active(self) -> None:
        """Envia SpeedFactor(%) ao braço real sempre que o slider mudar."""
        if not self._robot_connected or self._real_driver is None:
            return
        try:
            v = int(max(SPEED_FACTOR_MIN,
                        min(SPEED_FACTOR_MAX, self.speed_factor_var.get())))
        except (ValueError, tk.TclError):
            return
        try:
            # _send_dash já serializa via _dash_lock interno — _real_lock não necessário.
            self._real_driver._send_dash(f'SpeedFactor({v})')
            self.get_logger().warning(
                f'[SPEED] SpeedFactor({v})%% enviado ao CR10 real')
        except CR10RealDriverError as exc:
            self.get_logger().warning(f'SpeedFactor falhou: {exc}')

    @staticmethod
    def _duration_msg(seconds: float) -> Duration:
        sec = int(seconds)
        nsec = int((seconds - sec) * 1e9)
        return Duration(sec=sec, nanosec=nsec)

    # Publicação direta nos controllers
    def _publish_arm_from_sliders(self):
        if self._suppressing:
            return
        # Bloqueia publish de slider durante palpação ativa: o explorer está
        # fazendo streaming no mesmo tópico JTC a 33 Hz.
        with self._lock:
            _phase = self._latest_phase
        if _phase not in ('IDLE', 'DONE', 'ABORTED'):
            return
        self._suppressing = True
        try:
            positions_deg: list[float] = []
            for j in ARM_JOINTS:
                lo, hi = ARM_LIMITS_DEG[j]
                v = self._clamp_var(self.arm_sliders[j], lo, hi)
                if v is None:
                    return
                positions_deg.append(v)
            duration_s = self._move_duration_seconds()
        finally:
            self._suppressing = False
        positions_rad = [_math.radians(d) for d in positions_deg]
        msg = JointTrajectory()
        # stamp=zero → controller starts the trajectory immediately,
        # regardless of whether the node uses sim-time or wall-time.
        msg.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions_rad]
        pt.time_from_start = self._duration_msg(duration_s)
        msg.points.append(pt)
        self._arm_pub.publish(msg)
        # MIRROR é tratado pela subscrição em /cr10_group_controller/joint_trajectory
        # — captura este publish e também o do tactile_explorer numa única rota.

    def _publish_arm_q(self, q_rad, duration_s: float) -> None:
        """Publica UMA pose articular no JTC do braço, sem passar pelos
        sliders. Usado pelo arrasto 3D, que precisa de um horizonte curto
        (~100 ms) em vez do tempo de jog do Controle Manual.
        """
        with self._lock:
            phase = self._latest_phase
        if phase not in ('IDLE', 'DONE', 'ABORTED'):
            return
        msg = JointTrajectory()
        msg.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in list(q_rad)[:6]]
        pt.time_from_start = self._duration_msg(duration_s)
        msg.points.append(pt)
        self._arm_pub.publish(msg)

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

    # Subscrição no tópico de trajetória comandada
    def _cb_arm_trajectory(self, msg: JointTrajectory) -> None:
        """Captura trajetórias publicadas em /cr10_group_controller/joint_trajectory."""
        if self._robot_mode != 'MIRROR':
            return
        # Drag teach ativo → motores liberados, não enviar comandos de posição.
        if self._drag_enabled:
            return
        # Arrasto 3D do TCP: por padrão ele é SÓ simulação. O espelhamento
        # exige o opt-in explícito na aba (checkbox "Mirror to the real
        # CR10") — sem isso, as poses do arrasto param no Gazebo.
        if self._manip_active and not self._manip_mirror_on:
            return
        # Execução de movimento via _execute_movement_worker → não interferir.
        if self._exec_movement_id is not None:
            return
        with self._lock:
            phase = self._latest_phase
        if phase not in ('IDLE', 'DONE', 'ABORTED'):
            return  # palpação ativa → ServoJ poll loop assume
        if not msg.points:
            return
        # Eco do follow real→sim: as posições MEDIDAS re-publicadas no tópico
        # (com velocities) não devem gerar MovJ de volta ao próprio feedback.
        # Sliders publicam sem velocities e continuam passando normalmente.
        if self._mirror_following and msg.points[-1].velocities:
            return
        positions_rad = list(msg.points[-1].positions)
        if len(positions_rad) < 6:
            return
        self._mirror_movj_debounced(positions_rad[:6])

    def _cb_joint_states(self, msg: JointState) -> None:
        """Armazena posições URDF das juntas do braço — alimenta o mirror poll."""
        pos = dict(zip(msg.name, msg.position))
        # Juntas que não são do braço (mão COVVI) alimentam a viewport 3D.
        extra = {k: float(v) for k, v in pos.items() if k not in ARM_JOINTS}
        if extra:
            # Rebind atômico em vez de update() in-place: a thread do Tk lê
            # este dict a 33 Hz e mutá-lo aqui dispararia "dictionary changed
            # size during iteration" no meio de um quadro.
            self._latest_extra_joints = {**self._latest_extra_joints, **extra}
        try:
            self._latest_joint_rad = [float(pos[j]) for j in ARM_JOINTS]
        except KeyError:
            pass  # msg parcial (mão ou outra cadeia) — ignorar

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
                    msg.points.append(pt)
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
        msg.points.append(pt)
        self._arm_pub.publish(msg)

    def _cb_touch_value(self, msg: Float32) -> None:
        """Recebe /touch_sensor/value de um receptor EXTERNO (touch_receiver,
        UDP 8081). Quando a GUI lê a serial diretamente, ela é a própria
        publicadora — ignoramos o eco para não processar o loopback nem
        sobrescrever o valor já atualizado (a taxa limitada) em
        _on_touch_sample."""
        if self._touch_source is not None and self._touch_source.connected:
            return
        with self._lock:
            self._touch_value = float(msg.data)
            self._touch_last_ts = time.time()

    def _cb_ft_wrench(self, msg: WrenchStamped) -> None:
        """Seis eixos do FA7155. Roda na thread do executor ROS — só guarda
        estado; quem desenha é _refresh_ft_axes, a 10 Hz."""
        now = time.time()
        f, t = msg.wrench.force, msg.wrench.torque
        vals = (f.x, f.y, f.z, t.x, t.y, t.z)
        # Quadro com NaN/inf é descartado e CONTADO: deixá-lo entrar
        # envenenaria a média do tare e a coluna da planilha em silêncio.
        if not all(math.isfinite(v) for v in vals):
            with self._lock:
                self._ft_frames_bad += 1
            return
        with self._lock:
            self._ft_wrench['fx'], self._ft_wrench['fy'] = f.x, f.y
            self._ft_wrench['fz'] = f.z
            self._ft_wrench['mx'], self._ft_wrench['my'] = t.x, t.y
            self._ft_wrench['mz'] = t.z
            self._ft_last_ts = now
            self._ft_frames_ok += 1
            self._ft_arrivals.append(now)
            # Taxa pela JANELA inteira (n-1 intervalos), não pelo último dt:
            # o dt instantâneo de um link serial oscila demais para ser lido
            # como número na tela.
            if len(self._ft_arrivals) >= 8:
                span = self._ft_arrivals[-1] - self._ft_arrivals[0]
                self._ft_rate_hz = ((len(self._ft_arrivals) - 1) / span
                                    if span > 0 else None)

    def _publish_hand_from_sliders(self):
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
        # Envia para a mão real via ECI (SetDigitPosn) se ativo
        if self._eci_enabled:
            self._schedule_eci_posn(primary_deg)

    def _publish_sim_hand(self, primary_rad: dict[str, float],
                           duration_s: float) -> None:
        """Publica a trajetória da mão no Gazebo a partir das 6 juntas
        primárias (rad), expandindo as juntas mimic do URDF. Usado tanto pelo
        comando do slider (sim-only) quanto pelo mirror real→sim (Versão B)."""
        names = list(HAND_JOINTS)
        positions = [primary_rad[j] for j in HAND_JOINTS]
        # Expande as 26 juntas mimic com as razões do URDF.
        for mimic_name, driver, mult in MIMIC_LIST:
            names.append(mimic_name)
            positions.append(primary_rad[driver] * mult)
        msg = JointTrajectory()
        # stamp=zero → controller starts immediately (sim-time-safe).
        msg.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start = self._duration_msg(duration_s)
        msg.points.append(pt)
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

        def _deg(joint: str, pos: int) -> float:
            max_deg = 60.0 if joint == 'Rotate' else 90.0
            lo, hi = ECI_POSN_OPEN[joint], ECI_POSN_CLOSED[joint]
            frac = (float(pos) - lo) / float(hi - lo)
            return max(0.0, min(max_deg, frac * max_deg))

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

    def _apply_arm_home(self):
        """Move o braço para a Home customizada do usuário."""
        self._suppressing = True
        try:
            for j, deg in self._arm_home_deg.items():
                self.arm_sliders[j].set(deg)
        finally:
            self._suppressing = False
        self._publish_arm_from_sliders()

    def _solve_tcp_perpendicular(self):
        """Solver de pulso: dado joint1-3 dos sliders, calcula joint4/joint5
        para que o eixo z do Link6 (eixo do TCP — touch tool ou mão) fique
        exatamente perpendicular à mesa, apontando para baixo (−Z mundo).
        """
        if _fk_partial is None:
            self._set_status('kinematics unavailable — solver disabled.',
                             DANGER)
            return
        try:
            q_deg = {j: float(self.arm_sliders[j].get()) for j in ARM_JOINTS}
        except (ValueError, tk.TclError):
            self._set_status('Invalid sliders.', DANGER)
            return
        q = np.array([_math.radians(q_deg[j]) for j in ARM_JOINTS])

        R03 = _fk_partial(q, 3)[:3, :3]
        v = R03.T @ np.array([0.0, 0.0, -1.0])   # −Z mundo no frame 3
        s5 = _math.hypot(float(v[0]), float(v[1]))

        if s5 < 1e-9:
            # Degenerado: −Z mundo coincide com o eixo de joint5 (z do frame
            # 3).
            if float(v[2]) > 0.0:
                sols = [(q[3], 0.0)]
            else:
                self._set_status(
                    'TCP ⊥ table unreachable with current joint1-3 '
                    '(would require joint5 = ±180°).', DANGER)
                return
        else:
            sols = []
            for sgn in (+1.0, -1.0):
                q5 = _math.atan2(sgn * s5, float(v[2]))
                q4 = _math.atan2(-sgn * float(v[1]),
                                 -sgn * float(v[0])) + _math.pi / 2
                q4 = (q4 + _math.pi) % (2 * _math.pi) - _math.pi
                sols.append((q4, q5))

        # Filtra por limites dos sliders e escolhe o ramo mais próximo
        # da pose atual do pulso (evita flip desnecessário de 180°).
        lo4, hi4 = ARM_LIMITS_DEG['joint4']
        lo5, hi5 = ARM_LIMITS_DEG['joint5']
        feasible = [
            (q4, q5) for q4, q5 in sols
            if lo4 <= _math.degrees(q4) <= hi4
            and lo5 <= _math.degrees(q5) <= hi5
        ]
        if not feasible:
            self._set_status('TCP ⊥ table outside joint4/joint5 limits.',
                             DANGER)
            return
        q4, q5 = min(feasible,
                     key=lambda s: abs(s[0] - q[3]) + abs(s[1] - q[4]))

        self._suppressing = True
        try:
            self.arm_sliders['joint4'].set(round(_math.degrees(q4), 2))
            self.arm_sliders['joint5'].set(round(_math.degrees(q5), 2))
        finally:
            self._suppressing = False
        self._publish_arm_from_sliders()
        self._set_status(
            f'TCP ⊥ table: joint4={_math.degrees(q4):+.1f}° / '
            f'joint5={_math.degrees(q5):+.1f}°.', OK)

    # Home customizada — load / save em ~/.config/touch_pack/
    def _load_home_pose(self) -> None:
        """Carrega home salvo (sobrescreve `self._arm_home_deg`)."""
        try:
            if os.path.exists(HOME_POSE_FILE):
                with open(HOME_POSE_FILE) as fh:
                    data = json.load(fh)
                if isinstance(data, dict):
                    for j in ARM_JOINTS:
                        if j in data:
                            try:
                                lo, hi = ARM_LIMITS_DEG[j]
                                v = float(data[j])
                                self._arm_home_deg[j] = max(lo, min(hi, v))
                            except (TypeError, ValueError):
                                pass
                aviso = tool_stamp_mismatch(data, what='o home salvo')
                if aviso:
                    self.get_logger().warn(f'[FERRAMENTA] {aviso}')
                self.get_logger().info(
                    f'Home carregada de {HOME_POSE_FILE}')
        except Exception as exc:    # pragma: no cover
            self.get_logger().warn(f'Falha ao ler home pose: {exc}')

    def _save_home_pose(self) -> None:
        """Captura os ângulos dos sliders do braço como nova Home e
        persiste em ~/.config/touch_pack/home_pose.json.
        O botão `⌂ Home` passa a usar esses valores."""
        try:
            new_home = {
                j: float(self.arm_sliders[j].get()) for j in ARM_JOINTS
            }
            # Com que ferramenta esta home foi ensinada (ver tool_stamp).
            new_home.update(tool_stamp())
        except (ValueError, tk.TclError):
            self._set_status('Invalid sliders.', DANGER)
            return
        try:
            os.makedirs(os.path.dirname(HOME_POSE_FILE), exist_ok=True)
            with open(HOME_POSE_FILE, 'w') as fh:
                json.dump(new_home, fh, indent=2, sort_keys=True)
        except Exception as exc:    # pragma: no cover
            self._set_status(f'Failed to save home: {exc}', DANGER)
            return
        self._arm_home_deg = new_home
        summary = ' / '.join(f'{j[-1]}={new_home[j]:+.0f}°'
                              for j in ARM_JOINTS)
        self._set_status(f'Home saved ({summary}).', OK)

    def _capture_arm_from_robot(self) -> None:
        """Lê a posição atual do robô real, atualiza os sliders e salva
        como Home. O Gazebo iniciará nessa configuração na próxima vez
        que o launch file for executado (lê o mesmo home_pose.json)."""
        if not self._robot_connected or self._real_driver is None:
            self._set_status(
                'Connect the CR10 robot before capturing the position.', WARN)
            return
        if not _REAL_DRIVER_OK:
            self._set_status('Real driver not available.', DANGER)
            return
        q_urdf_rad = None
        last_exc: Exception | None = None
        for _attempt in range(3):
            try:
                q_urdf_rad = self._real_driver.read_joints_urdf_latest()
                break
            except CR10RealDriverError as exc:
                last_exc = exc
        if q_urdf_rad is None:
            self._set_status(f'Failed to read joints: {last_exc}', DANGER)
            return
        new_home = {
            j: float(_math.degrees(q_urdf_rad[i]))
            for i, j in enumerate(ARM_JOINTS)
        }
        # Atualiza sliders (suprime o callback de publish).
        self._suppressing = True
        try:
            for j in ARM_JOINTS:
                lo, hi = ARM_LIMITS_DEG[j]
                clamped = max(lo, min(hi, new_home[j]))
                self.arm_sliders[j].set(clamped)
        finally:
            self._suppressing = False
        # Persiste em home_pose.json.
        try:
            os.makedirs(os.path.dirname(HOME_POSE_FILE), exist_ok=True)
            with open(HOME_POSE_FILE, 'w') as fh:
                json.dump(new_home, fh, indent=2, sort_keys=True)
        except Exception as exc:
            self._set_status(f'Failed to save captured home: {exc}', DANGER)
            return
        self._arm_home_deg = new_home
        self._publish_arm_from_sliders()
        summary = ' / '.join(f'{j[-1]}={new_home[j]:+.0f}°' for j in ARM_JOINTS)
        self._set_status(
            f'Home captured from the real robot and saved ({summary}).', OK)

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

    def _apply_hand_preset(self, preset_deg: dict[str, float],
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

    # Aba "3D Manipulation" — arrasto do TCP com IK diferencial
    #
    # A viewport (manip3d.Manip3DView) é Tk puro: desenha o esqueleto do CR10
    # a partir da FK e resolve a IK DIFERENCIAL (DLS) do arrasto a 33 Hz.
    # Esta seção é só a cola com o ROS/GUI:
    #   q_provider  → alimenta a viewport com a pose da cena quando ociosa
    #   on_q        → publica a pose resolvida no JTC (Gazebo) a cada tick
    #   on_state    → atualiza os números do painel lateral
    #   on_drag     → arma/desarma o gate de espelhamento no braço real



    def _on_align_toggle(self) -> None:
        """Mostra/oculta os ajustes da calibração conforme o checkbox — os
        números só interessam quando ela está ligada."""
        grp = getattr(self, '_align_group', None)
        if grp is None:
            return
        if self.align_on_var.get():
            grp.pack(fill='x')
        else:
            grp.pack_forget()
        # O preview da grade desenha o anel de sondagem, que só existe com a
        # calibração ligada.
        self._redraw_matrix_preview()
















    def _on_tab_changed(self, _event=None) -> None:
        """Liga o tick da viewport 3D só quando a aba está visível (o laço
        roda a 33 Hz e não há razão para gastá-lo numa aba escondida)."""
        view = self._manip_view
        if view is None:
            return
        nb = getattr(self, '_nb', None)
        frame = self._manip_tab_frame
        visible = (nb is not None and frame is not None
                   and str(nb.select()) == str(frame))
        if visible:
            self._manip_sync_from_scene(quiet=True)
            self._manip_load_scene()
            view.start()
        else:
            view.stop()

    # Malhas do URDF — carga preguiçosa em thread




    # Aba "Célula de Carga" — leitura + calibração
    def _build_loadcell_tab(self, root: tk.Frame):
        sub_nb = ttk.Notebook(root, style='Tactile.TNotebook')
        sub_nb.pack(fill='both', expand=True)

        tab_axes = tk.Frame(sub_nb, bg=BG)
        sub_nb.add(tab_axes, text='6 Axes')
        self._build_lc_axes_tab(self._scrollable(tab_axes))
        self.root.after(80, self._refresh_ft_axes)

    def _spawn_touch_receiver(self) -> None:
        """Inicia o touch_receiver_node (UDP 8081) junto com o force_receiver.
        Best-effort: o touch sensor é opcional — falha aqui não bloqueia a
        célula de carga; o painel apenas fica em 'aguardando'."""
        if self._touch_rx_proc is not None and self._touch_rx_proc.poll() is None:
            return
        try:
            if self.count_publishers('/touch_sensor/value') > 0:
                return      # já existe um receptor (launch) — não duplicar
        except Exception:
            pass
        try:
            self._touch_rx_proc = subprocess.Popen(
                ['ros2', 'run', 'touch_pack', 'touch_receiver'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                start_new_session=True)
        except FileNotFoundError:
            self._touch_rx_proc = None
            return

        def _pipe_log(proc=self._touch_rx_proc):
            for raw in proc.stdout:
                log.info('[TOUCH-RX] %s',
                         raw.decode('utf-8', errors='replace').rstrip())
        threading.Thread(target=_pipe_log, daemon=True,
                         name='touch-rx-log').start()

    def _ensure_palpation_logger(self) -> None:
        """Garante um palpation_logger vivo — é ele quem grava o run em
        ~/touch_pack_runs. No launch ele já sobe junto; aqui cobre a GUI
        rodando standalone. O /palpation/start é TRANSIENT_LOCAL, então o
        logger recebe o start mesmo subindo um instante depois do publish.
        Best-effort: falha não bloqueia o experimento."""
        if self._logger_proc is not None and self._logger_proc.poll() is None:
            return
        try:
            if 'palpation_logger' in self.get_node_names():
                return      # já existe (launch) — não duplicar o run
        except Exception:
            pass
        try:
            self._logger_proc = subprocess.Popen(
                ['ros2', 'run', 'touch_pack', 'palpation_logger'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                start_new_session=True)
        except FileNotFoundError:
            self._logger_proc = None
            return

        def _pipe_log(proc=self._logger_proc):
            for raw in proc.stdout:
                log.info('[LOGGER] %s',
                         raw.decode('utf-8', errors='replace').rstrip())
        threading.Thread(target=_pipe_log, daemon=True,
                         name='logger-log').start()

    def _kill_touch_receiver(self) -> None:
        proc = self._touch_rx_proc
        self._touch_rx_proc = None
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except OSError:
                    pass
            except OSError:
                pass

    def _build_poses_tab(self, root: tk.Frame) -> None:
        """Layout dois-colunas: esquerda=Poses (fixa 310px), direita=Movimentos."""
        left = tk.Frame(root, bg=BG, width=310)
        left.pack(side='left', fill='y', padx=(12, 6), pady=12)
        left.pack_propagate(False)

        right = tk.Frame(root, bg=BG)
        right.pack(side='left', fill='both', expand=True, padx=(6, 12), pady=12)

        # LEFT: Poses
        tk.Label(left, text='Poses', bg=BG, fg=TEXT, font=FONT_HEAD).pack(anchor='w')
        tk.Frame(left, bg=BORDER, height=1).pack(fill='x', pady=(4, 8))

        btn_row = tk.Frame(left, bg=BG)
        btn_row.pack(fill='x', pady=(0, 8))

        self._drag_btn = tk.Button(
            btn_row, text='✋ Drag OFF',
            command=self._toggle_drag,
            bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=_shade(BTN_NEUTRAL, -0.08),
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2')
        self._drag_btn.pack(side='left', padx=(0, 4))

        tk.Button(
            btn_row, text='◉ Robot',
            command=self._capture_pose_robot,
            bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=_shade(BTN_NEUTRAL, -0.08),
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2').pack(side='left', padx=(0, 4))

        tk.Button(
            btn_row, text='⌨ Sim',
            command=self._capture_pose_sim,
            bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=_shade(BTN_NEUTRAL, -0.08),
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2').pack(side='left')

        lbx_frame = tk.Frame(left, bg=BG)
        lbx_frame.pack(fill='both', expand=True)

        p_scroll = ttk.Scrollbar(lbx_frame, orient='vertical')
        p_scroll.pack(side='right', fill='y')

        self._poses_lbx = tk.Listbox(
            lbx_frame, yscrollcommand=p_scroll.set,
            bg=PANEL, fg=TEXT, font=FONT_MONO_S,
            selectbackground=PRIMARY, selectforeground='white',
            relief='flat', bd=0, highlightthickness=1,
            highlightbackground=BORDER, activestyle='none')
        self._poses_lbx.pack(side='left', fill='both', expand=True)
        p_scroll.config(command=self._poses_lbx.yview)

        pose_act = tk.Frame(left, bg=BG)
        pose_act.pack(fill='x', pady=(8, 0))

        tk.Button(
            pose_act, text='✏ Rename',
            command=self._rename_selected_pose,
            bg=BTN_NEUTRAL, fg=TEXT,
            activebackground=_shade(BTN_NEUTRAL, -0.08),
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2').pack(side='left', padx=(0, 4))

        tk.Button(
            pose_act, text='✖ Delete',
            command=self._delete_selected_pose,
            bg=DANGER, fg='white',
            activebackground=DANGER_HV,
            font=FONT_SMALL, relief='flat', bd=0, padx=8, pady=4,
            cursor='hand2').pack(side='left')

        # RIGHT: Movimentos
        mov_hdr = tk.Frame(right, bg=BG)
        mov_hdr.pack(fill='x')

        tk.Label(mov_hdr, text='Motions', bg=BG, fg=TEXT,
                 font=FONT_HEAD).pack(side='left', anchor='w')

        tk.Button(
            mov_hdr, text='+ New',
            command=self._new_movement,
            bg=PRIMARY, fg='white',
            activebackground=PRIMARY_HV,
            font=FONT_SMALL, relief='flat', bd=0, padx=10, pady=4,
            cursor='hand2').pack(side='right')

        tk.Frame(right, bg=BORDER, height=1).pack(fill='x', pady=(4, 8))

        mov_lbx_frame = tk.Frame(right, bg=BG, height=120)
        mov_lbx_frame.pack(fill='x')
        mov_lbx_frame.pack_propagate(False)

        m_scroll = ttk.Scrollbar(mov_lbx_frame, orient='vertical')
        m_scroll.pack(side='right', fill='y')

        self._movs_lbx = tk.Listbox(
            mov_lbx_frame, yscrollcommand=m_scroll.set,
            bg=PANEL, fg=TEXT, font=FONT_MONO_S,
            selectbackground=PRIMARY, selectforeground='white',
            relief='flat', bd=0, highlightthickness=1,
            highlightbackground=BORDER, activestyle='none')
        self._movs_lbx.pack(side='left', fill='both', expand=True)
        m_scroll.config(command=self._movs_lbx.yview)
        self._movs_lbx.bind('<<ListboxSelect>>', self._on_movement_select)

        self._mov_detail_outer = tk.Frame(right, bg=BG)
        self._mov_detail_outer.pack(fill='both', expand=True, pady=(8, 0))

        self._refresh_poses_list()
        self._refresh_movements_list()

    # Dados: load / save
    def _load_poses_data(self) -> None:
        try:
            with open(POSES_FILE) as f:
                data = json.load(f)
            self._poses = data.get('poses', [])
            self._movements = data.get('movements', [])
            self._next_pose_id = max((p['id'] for p in self._poses), default=0) + 1
            self._next_movement_id = max(
                (m['id'] for m in self._movements), default=0) + 1
            if self._poses or self._movements:
                aviso = tool_stamp_mismatch(data, what='o poses.json')
                if aviso:
                    self.get_logger().warn(f'[FERRAMENTA] {aviso}')
        except (FileNotFoundError, json.JSONDecodeError, KeyError):
            self._poses = []
            self._movements = []
            self._next_pose_id = 1
            self._next_movement_id = 1

    def _save_poses_data(self) -> None:
        os.makedirs(os.path.dirname(POSES_FILE), exist_ok=True)
        with open(POSES_FILE, 'w') as f:
            json.dump({'poses': self._poses, 'movements': self._movements,
                       **tool_stamp()}, f, indent=2)

    # Lookup helpers
    def _pose_by_id(self, pid: int) -> dict | None:
        for p in self._poses:
            if p['id'] == pid:
                return p
        return None

    def _movement_by_id(self, mid: int) -> dict | None:
        for m in self._movements:
            if m['id'] == mid:
                return m
        return None

    def _pose_label(self, p: dict) -> str:
        q = p['q_deg']
        parts = ' '.join(f'J{i + 1}={v:+.0f}°' for i, v in enumerate(q))
        hand_marker = '  [Hand]' if p.get('hand_deg') else ''
        return f"{p['name']}{hand_marker}  [{parts}]"

    # Refresh widgets
    def _refresh_poses_list(self) -> None:
        lbx = self._poses_lbx
        if lbx is None:
            return
        lbx.delete(0, 'end')
        for p in self._poses:
            lbx.insert('end', self._pose_label(p))

    def _refresh_movements_list(self, select_id: int | None = None) -> None:
        lbx = self._movs_lbx
        if lbx is None:
            return
        lbx.delete(0, 'end')
        for m in self._movements:
            lbx.insert('end', m['name'])
        if select_id is not None:
            for i, m in enumerate(self._movements):
                if m['id'] == select_id:
                    lbx.selection_set(i)
                    lbx.see(i)
                    break

    def _on_movement_select(self, _event=None) -> None:
        lbx = self._movs_lbx
        if lbx is None:
            return
        sel = lbx.curselection()
        if not sel:
            return
        self._refresh_movement_detail(self._movements[sel[0]])

    def _refresh_movement_detail(self, mov: dict) -> None:
        outer = self._mov_detail_outer
        if outer is None:
            return
        if self._mov_detail_inner is not None:
            self._mov_detail_inner.destroy()

        inner = tk.Frame(outer, bg=PANEL,
                         highlightthickness=1, highlightbackground=BORDER)
        inner.pack(fill='both', expand=True)
        self._mov_detail_inner = inner

        # Header
        hdr = tk.Frame(inner, bg=PANEL)
        hdr.pack(fill='x', padx=12, pady=(10, 4))
        tk.Label(hdr, text=mov['name'], bg=PANEL, fg=TEXT,
                 font=FONT_HEAD).pack(side='left')
        tk.Button(hdr, text='✏',
                  command=lambda: self._rename_movement(mov['id']),
                  bg=BTN_NEUTRAL, fg=TEXT,
                  font=FONT_SMALL, relief='flat', bd=0,
                  padx=6, pady=2, cursor='hand2').pack(side='left', padx=(8, 0))
        tk.Button(hdr, text='✖ Delete',
                  command=lambda: self._delete_movement(mov['id']),
                  bg=DANGER, fg='white', activebackground=DANGER_HV,
                  font=FONT_SMALL, relief='flat', bd=0,
                  padx=8, pady=2, cursor='hand2').pack(side='right')
        tk.Frame(inner, bg=BORDER, height=1).pack(fill='x')

        body = tk.Frame(inner, bg=PANEL)
        body.pack(fill='both', expand=True, padx=12, pady=8)

        # Sequência
        seq_col = tk.Frame(body, bg=PANEL)
        seq_col.pack(side='left', fill='both', expand=True, padx=(0, 12))

        tk.Label(seq_col, text='Pose Sequence', bg=PANEL, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(anchor='w')

        seq_frame = tk.Frame(seq_col, bg=PANEL)
        seq_frame.pack(fill='both', expand=True, pady=(4, 0))

        seq_sb = ttk.Scrollbar(seq_frame, orient='vertical')
        seq_sb.pack(side='right', fill='y')

        seq_lbx = tk.Listbox(
            seq_frame, yscrollcommand=seq_sb.set,
            bg=BG, fg=TEXT, font=FONT_MONO_S,
            selectbackground=PRIMARY, selectforeground='white',
            relief='flat', bd=0, highlightthickness=0,
            activestyle='none', height=6)
        seq_lbx.pack(side='left', fill='both', expand=True)
        seq_sb.config(command=seq_lbx.yview)

        def _refresh_seq():
            seq_lbx.delete(0, 'end')
            for pid in mov['pose_ids']:
                p = self._pose_by_id(pid)
                if p is None:
                    seq_lbx.insert('end', f'[deletada:{pid}]')
                else:
                    hand_tag = ' [Hand]' if p.get('hand_deg') else ''
                    seq_lbx.insert('end', f"{p['name']}{hand_tag}")

        _refresh_seq()

        def _add_pose_to_seq():
            lbx = self._poses_lbx
            if lbx is None:
                return
            sel = lbx.curselection()
            if not sel:
                self._set_status('Select a pose in the list on the left.', WARN)
                return
            mov['pose_ids'].append(self._poses[sel[0]]['id'])
            _refresh_seq()
            self._save_poses_data()

        def _remove_pose_from_seq():
            sel = seq_lbx.curselection()
            if not sel:
                return
            idx = sel[0]
            if 0 <= idx < len(mov['pose_ids']):
                del mov['pose_ids'][idx]
                _refresh_seq()
                self._save_poses_data()

        def _move_up():
            sel = seq_lbx.curselection()
            if not sel:
                return
            i = sel[0]
            if i > 0:
                mov['pose_ids'][i - 1], mov['pose_ids'][i] = \
                    mov['pose_ids'][i], mov['pose_ids'][i - 1]
                _refresh_seq()
                seq_lbx.selection_set(i - 1)
                self._save_poses_data()

        def _move_down():
            sel = seq_lbx.curselection()
            if not sel:
                return
            i = sel[0]
            if i < len(mov['pose_ids']) - 1:
                mov['pose_ids'][i], mov['pose_ids'][i + 1] = \
                    mov['pose_ids'][i + 1], mov['pose_ids'][i]
                _refresh_seq()
                seq_lbx.selection_set(i + 1)
                self._save_poses_data()

        seq_btns = tk.Frame(seq_col, bg=PANEL)
        seq_btns.pack(fill='x', pady=(6, 0))

        for txt, cmd in [('+ Adicionar', _add_pose_to_seq),
                          ('−', _remove_pose_from_seq),
                          ('↑', _move_up),
                          ('↓', _move_down)]:
            tk.Button(seq_btns, text=txt, command=cmd,
                      bg=BTN_NEUTRAL, fg=TEXT,
                      activebackground=_shade(BTN_NEUTRAL, -0.08),
                      font=FONT_SMALL, relief='flat', bd=0,
                      padx=8, pady=3, cursor='hand2').pack(side='left', padx=(0, 4))

        # Controles + Execução
        ctrl_col = tk.Frame(body, bg=PANEL, width=190)
        ctrl_col.pack(side='left', fill='y')
        ctrl_col.pack_propagate(False)

        tk.Label(ctrl_col, text='Speed (%)', bg=PANEL, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(anchor='w')
        spd_var = tk.IntVar(value=mov.get('speed_pct', 10))

        def _on_spd(*_):
            try:
                v = max(1, min(100, int(spd_var.get())))
                mov['speed_pct'] = v
                self._save_poses_data()
            except (ValueError, tk.TclError):
                pass

        tk.Spinbox(ctrl_col, from_=1, to=100, textvariable=spd_var,
                   width=7, font=FONT_MONO_S, relief='flat', bd=1,
                   command=_on_spd).pack(anchor='w', pady=(0, 10))
        spd_var.trace_add('write', _on_spd)

        tk.Label(ctrl_col, text='Duration/step (s)', bg=PANEL, fg=TEXT_MUTED,
                 font=FONT_SMALL).pack(anchor='w')
        dur_var = tk.DoubleVar(value=mov.get('dur_s', 2.0))

        def _on_dur(*_):
            try:
                v = max(0.1, float(dur_var.get()))
                mov['dur_s'] = round(v, 2)
                self._save_poses_data()
            except (ValueError, tk.TclError):
                pass

        tk.Spinbox(ctrl_col, from_=0.1, to=60.0, increment=0.5,
                   textvariable=dur_var, width=7, format='%.1f',
                   font=FONT_MONO_S, relief='flat', bd=1,
                   command=_on_dur).pack(anchor='w', pady=(0, 16))
        dur_var.trace_add('write', _on_dur)

        _mid = mov['id']
        tk.Button(ctrl_col, text='▶ Run',
                  command=lambda: self._start_movement(_mid, loop=False),
                  bg=OK, fg='white', activebackground='#15803d',
                  font=FONT_SMALL, relief='flat', bd=0,
                  padx=8, pady=4, cursor='hand2').pack(fill='x', pady=(0, 4))
        tk.Button(ctrl_col, text='↻ Loop',
                  command=lambda: self._start_movement(_mid, loop=True),
                  bg=WARN, fg='white', activebackground='#b45309',
                  font=FONT_SMALL, relief='flat', bd=0,
                  padx=8, pady=4, cursor='hand2').pack(fill='x', pady=(0, 4))

        tk.Button(ctrl_col, text='■ Stop',
                  command=self._stop_execution,
                  bg=DANGER, fg='white', activebackground=DANGER_HV,
                  font=FONT_SMALL, relief='flat', bd=0,
                  padx=8, pady=4, cursor='hand2').pack(fill='x')

    # Captura de poses
    def _capture_hand_from_sliders(self) -> dict | None:
        """Retorna {junta: graus} dos sliders da mão se disponíveis, else None."""
        sliders = getattr(self, 'hand_sliders', None)
        if not sliders:
            return None
        return {j: float(sliders[j].get()) for j in HAND_JOINTS}

    def _capture_pose_robot(self) -> None:
        drv = self._real_driver
        if drv is None or not self._robot_connected:
            self._set_status('Real robot not connected — use ⌨ Sim.', WARN)
            return
        try:
            q_urdf = drv.read_joints_urdf()
            q_deg = [math.degrees(float(v)) for v in q_urdf]
            hand_deg = self._capture_hand_from_sliders()
            self._add_pose(q_deg, prefix='Robot', hand_deg=hand_deg)
        except Exception as exc:
            self._set_status(f'Error capturing real pose: {exc}', DANGER)

    def _capture_pose_sim(self) -> None:
        positions = self._latest_joint_rad
        if positions is None:
            self._set_status('No /joint_states reading — start the simulation.', WARN)
            return
        q_deg = [math.degrees(float(v)) for v in positions]
        hand_deg = self._capture_hand_from_sliders()
        self._add_pose(q_deg, prefix='Sim', hand_deg=hand_deg)

    def _add_pose(self, q_deg: list, prefix: str = 'Pose',
                  hand_deg: dict | None = None,
                  hand_eci_id: int | None = None) -> None:
        pid = self._next_pose_id
        self._next_pose_id += 1
        name = f'{prefix} {pid}'
        pose: dict = {'id': pid, 'name': name,
                      'q_deg': [round(float(v), 2) for v in q_deg[:6]]}
        if hand_deg is not None:
            pose['hand_deg'] = {j: round(float(hand_deg.get(j, 0)), 2)
                                for j in HAND_JOINTS}
            pose['hand_eci_id'] = hand_eci_id
        self._poses.append(pose)
        self._save_poses_data()
        self._refresh_poses_list()
        hand_info = '  + COVVI Hand' if hand_deg else ''
        self._set_status(f'Pose "{name}" captured{hand_info}.', OK)

    # Ações nas poses
    def _rename_selected_pose(self) -> None:
        lbx = self._poses_lbx
        if lbx is None:
            return
        sel = lbx.curselection()
        if not sel:
            self._set_status('Select a pose to rename.', WARN)
            return
        pose = self._poses[sel[0]]
        new_name = self._ask_name_dialog('Rename Pose', pose['name'])
        if new_name:
            pose['name'] = new_name
            self._save_poses_data()
            self._refresh_poses_list()

    def _delete_selected_pose(self) -> None:
        lbx = self._poses_lbx
        if lbx is None:
            return
        sel = lbx.curselection()
        if not sel:
            self._set_status('Select a pose to delete.', WARN)
            return
        pose = self._poses[sel[0]]
        pid = pose['id']
        for m in self._movements:
            m['pose_ids'] = [x for x in m['pose_ids'] if x != pid]
        self._poses.pop(sel[0])
        self._save_poses_data()
        self._refresh_poses_list()
        self._set_status(f'Pose "{pose["name"]}" deleted.', OK)

    # Drag teach
    def _publish_drag_state(self, active: bool) -> None:
        """Publica estado do drag em /palpation/drag_mode (latched, thread-safe)."""
        try:
            msg = Bool()
            msg.data = active
            self._drag_pub.publish(msg)
        except Exception as exc:
            self.get_logger().debug(f'[DRAG] publish drag_mode falhou: {exc}')

    def _toggle_drag(self) -> None:
        """Ativa/desativa manualmente o modo drag."""
        if not self._robot_connected or self._real_driver is None:
            self._set_status('Drag teach requires the real robot connected.', WARN)
            return
        new_state = not self._drag_enabled
        if not new_state:
            # Desativando: congela sliders na posição final ANTES de zerar estado.
            self._sync_sliders_from_drag()
        self._drag_last_valid_q = None
        self._drag_last_t = None
        self._drag_enabled = new_state
        self._publish_drag_state(new_state)
        btn = self._drag_btn
        if btn is not None:
            if new_state:
                btn.config(text='✋ Drag ON', bg=WARN, fg='white',
                           activebackground='#b45309')
            else:
                btn.config(text='✋ Drag OFF', bg=BTN_NEUTRAL, fg=TEXT,
                           activebackground=_shade(BTN_NEUTRAL, -0.08))
        self._set_status(
            'Drag active — enable the physical button on the robot to move it.' if new_state
            else 'Drag desativado.', WARN if new_state else OK)

    def _update_sliders_from_q(self, q_rad) -> None:
        """Atualiza os sliders do braço com posições em rad durante o drag."""
        if not self._drag_enabled:
            return
        self._suppressing = True
        try:
            for i, j in enumerate(ARM_JOINTS):
                lo, hi = ARM_LIMITS_DEG[j]
                deg = _math.degrees(float(q_rad[i]))
                self.arm_sliders[j].set(max(lo, min(hi, deg)))
        finally:
            self._suppressing = False

    def _sync_sliders_from_drag(self) -> None:
        """Congela os sliders na posição final do drag e publica para o Gazebo."""
        q_rad = self._drag_last_valid_q
        if q_rad is None:
            q_rad = self._latest_joint_rad
        if q_rad is None:
            return
        self._suppressing = True
        try:
            for i, j in enumerate(ARM_JOINTS):
                lo, hi = ARM_LIMITS_DEG[j]
                deg = _math.degrees(float(q_rad[i]))
                self.arm_sliders[j].set(max(lo, min(hi, deg)))
        finally:
            self._suppressing = False
        self._publish_arm_from_sliders()

    # Ações nos movimentos
    def _new_movement(self) -> None:
        name = self._ask_name_dialog(
            'New Motion', f'Movimento {self._next_movement_id}')
        if name is None:
            return
        mid = self._next_movement_id
        self._next_movement_id += 1
        mov = {'id': mid, 'name': name, 'pose_ids': [],
               'speed_pct': 10, 'dur_s': 2.0}
        self._movements.append(mov)
        self._save_poses_data()
        self._refresh_movements_list(select_id=mid)
        self._refresh_movement_detail(mov)

    def _rename_movement(self, mov_id: int) -> None:
        mov = self._movement_by_id(mov_id)
        if mov is None:
            return
        new_name = self._ask_name_dialog('Rename Motion', mov['name'])
        if new_name:
            mov['name'] = new_name
            self._save_poses_data()
            self._refresh_movements_list(select_id=mov_id)
            self._refresh_movement_detail(mov)

    def _delete_movement(self, mov_id: int) -> None:
        mov = self._movement_by_id(mov_id)
        if mov is None:
            return
        name = mov['name']
        self._movements = [m for m in self._movements if m['id'] != mov_id]
        self._save_poses_data()
        self._refresh_movements_list()
        if self._mov_detail_inner is not None:
            self._mov_detail_inner.destroy()
            self._mov_detail_inner = None
        self._set_status(f'Motion "{name}" deleted.', OK)

    # Execução de movimentos
    def _start_movement(self, mov_id: int, loop: bool = False) -> None:
        if self._exec_thread is not None and self._exec_thread.is_alive():
            self._set_status('Execution in progress — stop first.', WARN)
            return
        mov = self._movement_by_id(mov_id)
        if mov is None:
            return
        if not mov['pose_ids']:
            self._set_status('Add poses to the sequence before running.', WARN)
            return
        self._exec_stop.clear()
        self._exec_movement_id = mov_id
        self._exec_thread = threading.Thread(
            target=self._execute_movement_worker,
            args=(dict(mov), loop),
            daemon=True, name='exec-movement')
        self._exec_thread.start()
        suffix = '  (loop)' if loop else ''
        self._set_status(f'Running "{mov["name"]}"{suffix}...', OK)

    def _stop_execution(self) -> None:
        self._exec_stop.set()
        # Não limpa _exec_movement_id aqui — o finally do worker faz isso.
        if (self._robot_mode == 'MIRROR' and self._robot_connected
                and self._real_driver is not None):
            try:
                self._real_driver.halt()
            except Exception:
                pass
        self._set_status('Execution stopped.', WARN)

    def _execute_movement_worker(self, mov: dict, loop: bool) -> None:
        try:
            self._run_movement_once(mov)
            while loop and not self._exec_stop.is_set():
                self._run_movement_once(mov)
        except Exception as exc:
            log.warning('Execução de movimento falhou: %s', exc)
            # `exc` é APAGADO ao sair do handler (PEP 3110), e este callback
            # roda depois — capturar por default do lambda é o que salva a
            # mensagem.
            self.root.after(
                0, lambda e=str(exc): self._set_status(
                    f'Execution failed: {e}', DANGER))
        finally:
            self._exec_movement_id = None

    def _run_movement_once(self, mov: dict) -> None:
        """Executa uma passagem completa pelo movimento."""
        dur_s = max(0.1, mov.get('dur_s', 2.0))
        speed_pct = max(1, min(100, mov.get('speed_pct', 10)))
        poses = [self._pose_by_id(pid) for pid in mov['pose_ids']]
        poses = [p for p in poses if p is not None]
        if not poses:
            return

        mode = self._robot_mode

        if mode in ('SIM_ONLY', 'MIRROR'):
            # Publica trajetória completa no Gazebo de uma vez.
            msg = JointTrajectory()
            msg.joint_names = ARM_JOINTS
            for i, pose in enumerate(poses):
                pt = JointTrajectoryPoint()
                pt.positions = [math.radians(float(v)) for v in pose['q_deg']]
                pt.velocities = [0.0] * 6
                total_s = (i + 1) * dur_s
                pt.time_from_start = Duration(
                    sec=int(total_s),
                    nanosec=int((total_s % 1.0) * 1_000_000_000))
                msg.points.append(pt)
            self._arm_pub.publish(msg)

        if mode == 'MIRROR':
            # Robô real: MovJ + mão por pose, cadenciado por dur_s — paralelo ao Gazebo.
            drv = self._real_driver
            if drv is not None and self._robot_connected:
                try:
                    drv._send_dash(f'SpeedFactor({speed_pct})')
                except Exception:
                    pass
                for pose in poses:
                    if self._exec_stop.is_set():
                        break
                    t_step_start = time.monotonic()
                    try:
                        if _urdf_to_dobot is not None:
                            q_urdf = np.array(
                                [math.radians(float(v)) for v in pose['q_deg']])
                            q_dobot_deg = np.degrees(_urdf_to_dobot(q_urdf)).tolist()
                        else:
                            q_dobot_deg = list(pose['q_deg'])
                        drv.mov_j_joint_deg(q_dobot_deg)
                    except Exception as exc:
                        log.warning('MovJ falhou: %s', exc)
                        break
                    # Aplica pose da mão COVVI se armazenada (Tk-safe via after).
                    self._apply_hand_pose_from_movement(pose)
                    # Aguarda o restante de dur_s para este passo,
                    # verificando _exec_stop a cada 100 ms.
                    deadline = t_step_start + dur_s
                    while not self._exec_stop.is_set():
                        remaining = deadline - time.monotonic()
                        if remaining <= 0.0:
                            break
                        self._exec_stop.wait(min(0.1, remaining))
        elif mode == 'SIM_ONLY':
            # Itera por pose aplicando mão a cada passo; aguarda dur_s por pose.
            for pose in poses:
                if self._exec_stop.is_set():
                    break
                self._apply_hand_pose_from_movement(pose)
                self._exec_stop.wait(dur_s)

    def _apply_hand_pose_from_movement(self, pose: dict) -> None:
        """Aplica a pose da mão COVVI armazenada na pose (thread-safe via after).

        No-op se a pose não tiver 'hand_deg' ou se estiver no modo touch_tool.
        """
        hand_deg = pose.get('hand_deg')
        if not hand_deg:
            return
        hand_eci_id = pose.get('hand_eci_id')
        self.root.after(
            0, lambda hd=dict(hand_deg), eid=hand_eci_id:
            self._apply_hand_preset(hd, eci_grip_id=eid))

    # Diálogo de nome
    def _ask_name_dialog(self, title: str, initial: str = '') -> str | None:
        result: list[str | None] = [None]
        dlg = tk.Toplevel(self.root)
        dlg.title(title)
        dlg.configure(bg=BG)
        dlg.resizable(False, False)
        dlg.grab_set()

        tk.Label(dlg, text=title, bg=BG, fg=TEXT, font=FONT_HEAD
                 ).pack(padx=24, pady=(16, 8))
        var = tk.StringVar(value=initial)
        entry = tk.Entry(dlg, textvariable=var, font=FONT_LBL, width=32)
        entry.pack(padx=24, pady=(0, 8))
        entry.select_range(0, 'end')
        entry.focus_set()

        def _ok(_=None):
            val = var.get().strip()
            if val:
                result[0] = val
            dlg.destroy()

        def _cancel(_=None):
            dlg.destroy()

        row = tk.Frame(dlg, bg=BG)
        row.pack(pady=(0, 16))
        tk.Button(row, text='OK', command=_ok,
                  bg=PRIMARY, fg='white', font=FONT_LBL,
                  relief='flat', bd=0, padx=16, pady=4,
                  cursor='hand2').pack(side='left', padx=4)
        tk.Button(row, text='Cancel', command=_cancel,
                  bg=BTN_NEUTRAL, fg=TEXT, font=FONT_LBL,
                  relief='flat', bd=0, padx=16, pady=4,
                  cursor='hand2').pack(side='left', padx=4)
        entry.bind('<Return>', _ok)
        entry.bind('<Escape>', _cancel)
        dlg.wait_window()
        return result[0]

    def _build_statusbar(self):
        self.status_var = tk.StringVar(value='Pronto.')
        bar = tk.Frame(self.root, bg=PANEL)
        bar.pack(side='bottom', fill='x')
        tk.Frame(bar, bg=BORDER, height=1).pack(fill='x', side='top')
        self._status_dot = tk.Label(bar, text='●', bg=PANEL, fg=OK,
                                     font=FONT_SMALL)
        self._status_dot.pack(side='left', padx=(18, 6), pady=3)
        self._status_lbl = tk.Label(bar, textvariable=self.status_var,
                                     bg=PANEL, fg=TEXT_MUTED,
                                     anchor='w', font=FONT_LBL)
        self._status_lbl.pack(side='left')

    # helpers UI
    def _card(self, parent, title: str, *, expand: bool = True) -> tk.Frame:
        """Card com cabeçalho de barra de acento (sem divisor pesado)."""
        card = tk.Frame(parent, bg=PANEL,
                         highlightthickness=1,
                         highlightbackground=BORDER,
                         highlightcolor=BORDER)
        card.pack(fill='both' if expand else 'x', expand=expand)
        head = tk.Frame(card, bg=PANEL)
        head.pack(fill='x', padx=14, pady=(12, 6))
        tk.Frame(head, bg=PRIMARY, width=4).pack(side='left', fill='y',
                                                  padx=(0, 8))
        tk.Label(head, text=title, bg=PANEL, fg=TEXT, font=FONT_HEAD,
                 anchor='w').pack(side='left')
        inner = tk.Frame(card, bg=PANEL)
        inner.pack(fill='both', expand=True, padx=14, pady=(2, 12))
        return inner

    def _collapsible(self, parent, title: str,
                      expanded: bool = False) -> tk.Frame:
        """Seção expansível (disclosure ▸/▾) para parâmetros avançados —
        mantém o card principal enxuto sem remover funcionalidade."""
        wrap = tk.Frame(parent, bg=PANEL)
        wrap.pack(fill='x', pady=(10, 0))
        tk.Frame(wrap, bg=BORDER, height=1).pack(fill='x', pady=(0, 6))
        btn = tk.Button(wrap, bg=PANEL, fg=TEXT_MUTED,
                        activebackground=PANEL, activeforeground=TEXT,
                        font=FONT_LBL, relief='flat', bd=0, anchor='w',
                        highlightthickness=0, cursor='hand2', padx=0)
        btn.pack(fill='x')
        inner = tk.Frame(wrap, bg=PANEL)
        state = {'open': bool(expanded)}

        def _render():
            arrow = '▾' if state['open'] else '▸'
            btn.config(text=f'{arrow}  {title}')
            if state['open']:
                inner.pack(fill='x', pady=(4, 0))
            else:
                inner.pack_forget()

        def _toggle():
            state['open'] = not state['open']
            _render()

        btn.config(command=_toggle)
        _render()
        return inner

    def _kv(self, parent, key: str, val: str) -> tk.Label:
        row = tk.Frame(parent, bg=PANEL); row.pack(fill='x', pady=1)
        tk.Label(row, text=key, font=FONT_LBL, bg=PANEL, fg=TEXT_MUTED
                 ).pack(side='left')
        lbl = tk.Label(row, text=val, font=FONT_MONO, bg=PANEL, fg=TEXT)
        lbl.pack(side='right')
        return lbl

    def _build_slide_dir_selector(self, parent) -> None:
        """Segmented control (4 botões mutex) para a direção do sliding."""
        row = tk.Frame(parent, bg=PANEL); row.pack(fill='x', pady=(8, 2))
        top = tk.Frame(row, bg=PANEL); top.pack(fill='x')
        dir_lbl = tk.Label(top, text='Sliding Direction', font=FONT_LBL,
                           bg=PANEL, fg=TEXT, anchor='w')
        dir_lbl.pack(side='left')
        info = tk.Label(top, text='ⓘ', font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM)
        info.pack(side='left', padx=(5, 0))
        _hint = ('Straight Cartesian drag in XY (world); the joints '
                 'coordinate to preserve Z and orientation.')
        _Tooltip(dir_lbl, _hint)
        _Tooltip(info, _hint)
        btns = tk.Frame(row, bg=PANEL); btns.pack(fill='x', pady=(4, 2))
        self._slide_dir_btns: dict[str, tk.Button] = {}
        for d in ('+X', '-X', '+Y', '-Y'):
            b = tk.Button(btns, text=d,
                          command=lambda dd=d: self._on_slide_dir(dd),
                          bg=BTN_NEUTRAL, fg=TEXT, font=FONT_MONO,
                          activebackground=PRIMARY_HV,
                          activeforeground='white',
                          relief='flat', bd=0, padx=14, pady=6,
                          cursor='hand2')
            b.pack(side='left', fill='x', expand=True,
                   padx=(0 if d == '+X' else 4, 0))
            self._slide_dir_btns[d] = b
        self._on_slide_dir(self.slide_dir_var.get())
        return row

    def _on_slide_dir(self, d: str) -> None:
        if d not in ('+X', '-X', '+Y', '-Y'):
            return
        self.slide_dir_var.set(d)
        for k, b in self._slide_dir_btns.items():
            if k == d:
                b.config(bg=PRIMARY, fg='white')
            else:
                b.config(bg=BTN_NEUTRAL, fg=TEXT)

    def _update_step_preview(self) -> None:
        """Resume a escada abaixo dos campos: patamares e duração mínima.

        Usa a MESMA staircase_levels do explorer (mora em constants.py), então
        o que o rótulo promete é exatamente o que o robô vai percorrer.
        """
        lbl = getattr(self, '_step_preview_lbl', None)
        if lbl is None:
            return
        try:
            size = float(self.step_size_var.get())
            start = float(self.step_start_var.get())
            mx = float(self.step_max_var.get())
            dwell = float(self.step_dwell_var.get())
        except (tk.TclError, ValueError):
            return          # campo a meio de digitação
        is_matrix = self.mode_var.get() == 'MATRIX_MAP'
        if size <= 0.0 or mx <= start:
            lbl.config(
                text=('Staircase off — each grid point holds a single '
                      'setpoint.' if is_matrix else
                      'Staircase off — Manual holds a single setpoint, '
                      'adjustable live during the run.'),
                fg=TEXT_MUTED)
            return
        lv = staircase_levels(start, size, mx)
        if not lv:
            lbl.config(
                text=(f'Step too small for this range — over '
                      f'{STEP_MAX_LEVELS} plateaus. Increase the step size.'),
                fg=DANGER)
            return
        # Duração MÍNIMA: só os dwells. A acomodação de cada patamar depende
        # do material e não dá para prever aqui — daí "pelo menos".
        seq = ' → '.join(f'{v:g}' for v in lv[:(len(lv) + 1) // 2])
        # No MATRIX_MAP a escada roda EM CADA PONTO: o custo multiplica pela
        # grade, e uma grade modesta com uma escada modesta passa de uma hora
        # sem que nada no painel avisasse. Aqui avisa, antes do Start.
        # Este preview roda a cada tecla digitada nos campos da escada, e a
        # grade pode estar num estado intermediário — nunca deixar isso
        # derrubar o callback do Tk.
        n_pts = 1
        if is_matrix:
            try:
                n_pts = len(self._matrix_waypoints()[0] or [])
            except Exception:
                n_pts = 0        # grade ainda inválida: mostra só a escada
        total = len(lv) * dwell * max(n_pts, 1)
        head = (f'{len(lv)} plateaus: {seq} N, then back down the same way.')
        if is_matrix and n_pts:
            body = (f' Runs at EACH of the {n_pts} grid points: at least '
                    f'{total / 60.0:.0f} min of dwell '
                    f'({n_pts} × {len(lv)} × {dwell:g} s), plus settling and '
                    f'travel.')
            colour = DANGER if total > 3600.0 else TEXT_MUTED
        else:
            body = (f' At least {total / 60.0:.1f} min of dwell '
                    f'({len(lv)} × {dwell:g} s), plus settling time.')
            colour = TEXT_MUTED
        lbl.config(text=head + body, fg=colour)

    def _build_fmod_shape_selector(self, parent) -> None:
        """Segmented control (3 botões mutex) para a forma da onda de força."""
        row = tk.Frame(parent, bg=PANEL); row.pack(fill='x', pady=(8, 2))
        top = tk.Frame(row, bg=PANEL); top.pack(fill='x')
        lbl = tk.Label(top, text='Modulated Force', font=FONT_LBL,
                       bg=PANEL, fg=TEXT, anchor='w')
        lbl.pack(side='left')
        info = tk.Label(top, text='ⓘ', font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM)
        info.pack(side='left', padx=(5, 0))
        _hint = ('Off: the HOLD keeps the constant Target Force. Sine / '
                 'Cosine: after settling at the mean of Min and Max, the '
                 'setpoint follows that wave. It runs as a position '
                 'feedforward (dx = dF/K, K measured during the descent) — '
                 'the load cell is read every tick for safety, not to close '
                 'the loop on the wave. The commanded wave lands in the '
                 'setpoint_n column of the samples CSV, next to the measured '
                 'force_net_n.')
        _Tooltip(lbl, _hint)
        _Tooltip(info, _hint)
        btns = tk.Frame(row, bg=PANEL); btns.pack(fill='x', pady=(4, 2))
        self._fmod_shape_btns: dict[str, tk.Button] = {}
        for key, txt in (('OFF', 'Off'), ('SINE', 'Sine'),
                         ('COSINE', 'Cosine')):
            b = tk.Button(btns, text=txt,
                          command=lambda k=key: self._on_fmod_shape(k),
                          bg=BTN_NEUTRAL, fg=TEXT, font=FONT_MONO,
                          activebackground=PRIMARY_HV,
                          activeforeground='white',
                          relief='flat', bd=0, padx=14, pady=6,
                          cursor='hand2')
            b.pack(side='left', fill='x', expand=True,
                   padx=(0 if key == 'OFF' else 4, 0))
            self._fmod_shape_btns[key] = b
        self._on_fmod_shape(self.fmod_shape_var.get())
        return row

    def _on_fmod_shape(self, s: str) -> None:
        if s not in FMOD_SHAPES:
            return
        self.fmod_shape_var.set(s)
        for k, b in self._fmod_shape_btns.items():
            if k == s:
                b.config(bg=PRIMARY, fg='white')
            else:
                b.config(bg=BTN_NEUTRAL, fg=TEXT)
        self._update_fmod_preview()

    def _update_fmod_preview(self) -> None:
        """Resume a onda abaixo dos campos: média, amplitude, duração e
        pontos por período — com os mesmos números que o explorer vai usar.

        Existe porque neste modo NÃO há um setpoint único: o alvo do HOLD
        passa a ser a média (min+max)/2 e o que define o ensaio é a excursão
        e a frequência. Sem este resumo os avisos só apareciam no log do
        explorer, ou seja, depois de o braço já ter descido ao contato.
        """
        lbl = getattr(self, '_fmod_preview_lbl', None)
        if lbl is None:
            return
        shape = str(self.fmod_shape_var.get() or 'OFF').upper()
        if shape not in FMOD_SHAPES or shape == 'OFF':
            lbl.config(text='Modulated force off — the HOLD keeps the '
                            'constant Target Force above.', fg=TEXT_MUTED)
            return
        try:
            f_min = float(self.fmod_min_var.get())
            f_max = float(self.fmod_max_var.get())
            hz = float(self.fmod_hz_var.get())
            cycles = int(self.fmod_cycles_var.get())
            force_sp = float(self.force_sp_var.get())
        except (tk.TclError, ValueError):
            return          # campo a meio de digitação
        # Mesma normalização do _start_palpation: invertidos são trocados.
        if f_min > f_max:
            f_min, f_max = f_max, f_min
        mean = 0.5 * (f_min + f_max)
        amp = 0.5 * (f_max - f_min)
        if amp <= 1e-3:
            lbl.config(text='Min and Max are the same — no wave. The run '
                            'falls back to the constant setpoint.', fg=WARN)
            return
        if hz <= 0.0 or cycles <= 0:
            lbl.config(text='Frequency and cycles must both be above zero.',
                       fg=DANGER)
            return

        dur = cycles / hz
        dt = fmod_wave_dt(hz)
        pts = 1.0 / max(hz * dt, 1e-9)
        first = mean + amp if shape == 'COSINE' else mean
        txt = (f'{shape.title()} {f_min:g}–{f_max:g} N: mean {mean:.2f} N '
               f'± {amp:.2f} N, {cycles} cycles in {dur:.1f} s. '
               f'Opens at {first:.2f} N.')
        colour = TEXT_MUTED

        # Pontos por período: o MELHOR caso, com o tick DA ONDA (derivado da
        # frequência, não o tick do QS). O tick real é sempre maior, e em MovL
        # o braço vai mais devagar ainda — por isso a frase remete ao número
        # que o explorer mede e loga.
        if pts < FMOD_MIN_PTS_PER_CYCLE:
            # Este é exatamente o critério de RECUSA do explorer: pts < 8
            # equivale a hz acima de fmod_max_freq_hz. Antes o painel dizia
            # que a onda rodaria mesmo assim — não roda mais, e prometer o
            # contrário só faria o operador descobrir no log de erro.
            txt += (f' Only {pts:.1f} points per cycle with the ServoJ loop '
                    f'at {FMOD_CTRL_DT_S*1e3:.0f} ms (needs '
                    f'{FMOD_MIN_PTS_PER_CYCLE}). The explorer will REFUSE '
                    f'this wave: the ceiling is '
                    f'{fmod_max_freq_hz():.2f} Hz. Lower the frequency, or '
                    f'raise mirror_node AND tactile_explorer with '
                    f'servoj_period_s:={1.0/(hz*FMOD_MIN_PTS_PER_CYCLE):.3f}.')
            colour = DANGER
        else:
            txt += (f' {pts:.0f} points per cycle at best '
                    f'({dt*1e3:.0f} ms tick).')

        # O setpoint constante ainda governa a DESCIDA: o braço para nele e só
        # depois o HOLD sobe/desce até a média. Se os dois diferirem, há uma
        # excursão de força que não faz parte do ensaio.
        if abs(force_sp - mean) > 0.05:
            txt += (f' Note: the descent stops at the {force_sp:.2f} N Target '
                    f'Force above, then the HOLD moves to {mean:.2f} N — set '
                    f'Target Force to the mean to avoid that extra swing.')
            colour = WARN if colour is TEXT_MUTED else colour
        lbl.config(text=txt, fg=colour)

    def _build_palp_mode_selector(self, parent) -> None:
        """Segmented control (3 botões mutex) para o modo de palpação."""
        row = tk.Frame(parent, bg=PANEL); row.pack(fill='x', pady=(2, 6))
        top = tk.Frame(row, bg=PANEL); top.pack(fill='x')
        mode_lbl = tk.Label(top, text='Palpation Mode', font=FONT_LBL,
                            bg=PANEL, fg=TEXT, anchor='w')
        mode_lbl.pack(side='left')
        info = tk.Label(top, text='ⓘ', font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM)
        info.pack(side='left', padx=(5, 0))
        _hint = ('Touch: presses the table with controlled force and returns home '
                 '(selectable count). Slide: full cycle with lateral drag. '
                 'Manual: descends to the setpoint and holds indefinitely — '
                 'adjust Target Force live during the run; Stop returns home. '
                 'Matrix: grid mapping — jog the probe above the first point, '
                 'then Start; the first contact becomes the plane origin and '
                 'every grid point is touched at the same Target Force.')
        _Tooltip(mode_lbl, _hint)
        _Tooltip(info, _hint)
        btns = tk.Frame(row, bg=PANEL); btns.pack(fill='x', pady=(4, 2))
        self._palp_mode_btns: dict[str, tk.Button] = {}
        for key, txt in (('TOUCH', 'Touch'), ('SLIDE', 'Slide'),
                         ('MANUAL', 'Manual'), ('MATRIX_MAP', 'Matrix')):
            b = tk.Button(btns, text=txt,
                          command=lambda k=key: self._on_palp_mode(k),
                          bg=BTN_NEUTRAL, fg=TEXT, font=FONT_LBL,
                          activebackground=PRIMARY_HV,
                          activeforeground='white',
                          relief='flat', bd=0, padx=14, pady=6,
                          cursor='hand2')
            b.pack(side='left', fill='x', expand=True,
                   padx=(0 if key == 'TOUCH' else 4, 0))
            self._palp_mode_btns[key] = b

    def _on_palp_mode(self, mode: str) -> None:
        """Aplica o modo: destaca o botão, mostra/esconde os parâmetros de
        deslizamento e ajusta o rótulo de repetições/toques."""
        if mode not in ('TOUCH', 'SLIDE', 'MANUAL', 'MATRIX_MAP'):
            mode = 'SLIDE'
        self.mode_var.set(mode)
        for k, b in self._palp_mode_btns.items():
            if k == mode:
                b.config(bg=PRIMARY, fg='white')
            else:
                b.config(bg=BTN_NEUTRAL, fg=TEXT)

        # Layout por modo — reempacota em ordem DETERMINÍSTICA:
        #   force → [repeats] → [slide] → [matrix] → [force-mod] → advanced.
        adv  = getattr(self, '_adv_frame', None)
        rrow = getattr(self, '_row_repeats', None)
        grp  = getattr(self, '_slide_group', None)
        mgrp = getattr(self, '_matrix_group', None)
        fgrp = getattr(self, '_fmod_group', None)
        sgrp = getattr(self, '_step_group', None)
        if rrow is not None:
            rrow.pack_forget()
        if grp is not None:
            grp.pack_forget()
        if mgrp is not None:
            mgrp.pack_forget()
        if fgrp is not None:
            fgrp.pack_forget()
        if sgrp is not None:
            sgrp.pack_forget()

        if rrow is not None and mode not in ('MANUAL', 'MATRIX_MAP'):
            if adv is not None:
                rrow.pack(fill='x', pady=(5, 3), before=adv)
            else:
                rrow.pack(fill='x', pady=(5, 3))
            lbl = getattr(self, '_repeats_lbl', None)
            if lbl is not None:
                lbl.config(text='Number of Touches' if mode == 'TOUCH'
                           else 'Experiment Repetitions')

        if grp is not None and mode == 'SLIDE':
            if adv is not None:
                grp.pack(fill='x', before=adv)
            else:
                grp.pack(fill='x')

        if mgrp is not None and mode == 'MATRIX_MAP':
            if adv is not None:
                mgrp.pack(fill='x', before=adv)
            else:
                mgrp.pack(fill='x')
            # O canvas só tem tamanho depois de mapeado — redesenha no
            # próximo idle, quando o Tk já calculou a geometria.
            self.root.after_idle(self._redraw_matrix_preview)

        # Força modulada: só TOUCH. O explorer ignora o perfil nos demais
        # modos, então esconder aqui evita prometer o que não acontece.
        if fgrp is not None and mode == 'TOUCH':
            if adv is not None:
                fgrp.pack(fill='x', before=adv)
            else:
                fgrp.pack(fill='x')

        # Escada de força: MANUAL (substitui o hold infinito) e MATRIX_MAP
        # (substitui o hold de patamar único EM CADA PONTO da grade, dando o
        # mapa de histerese da peça em vez de um único ponto da curva).
        if sgrp is not None and mode in ('MANUAL', 'MATRIX_MAP'):
            if adv is not None:
                sgrp.pack(fill='x', before=adv)
            else:
                sgrp.pack(fill='x')
            self._update_step_preview()

        # Texto do botão principal acompanha o modo.
        btn = getattr(self, 'start_btn', None)
        if btn is not None:
            btn.config(text='▶  Start Touch' if mode == 'TOUCH'
                       else ('▶  Start Manual' if mode == 'MANUAL'
                             else ('▶  Start Matrix Map'
                                   if mode == 'MATRIX_MAP'
                                   else '▶  Start Palpation')))

    # ══════════════════════════════════════════════════════════════════
    # MATRIX_MAP — configurador visual da grade
    # ══════════════════════════════════════════════════════════════════














    def _on_force_live_change(self, *_):
        """Modo MANUAL: agenda a publicação de /palpation/set_force ao alterar
        o spinbox/slider de força, para o HOLD infinito seguir o novo alvo pelos
        micro-passos (sem reiniciar). DEBOUNCE (~150 ms): arrastar o slider NÃO
        inunda o ROS — só o último valor após parar é publicado. Silencioso nos
        demais modos."""
        mv = getattr(self, 'mode_var', None)
        if getattr(self, '_set_force_pub', None) is None or mv is None \
                or mv.get() != 'MANUAL':
            return
        after_id = getattr(self, '_set_force_after_id', None)
        if after_id is not None:
            try:
                self.root.after_cancel(after_id)
            except Exception:
                pass
        self._set_force_after_id = self.root.after(150, self._publish_set_force)

    def _publish_set_force(self):
        """Publica o setpoint corrente (fim do debounce). Revalida o modo — se
        o usuário saiu de MANUAL no intervalo, não publica."""
        self._set_force_after_id = None
        mv = getattr(self, 'mode_var', None)
        if getattr(self, '_set_force_pub', None) is None or mv is None \
                or mv.get() != 'MANUAL':
            return
        try:
            val = float(self.force_sp_var.get())
        except (tk.TclError, ValueError):
            return
        val = max(FORCE_SP_MIN, min(FORCE_SP_MAX, val))
        m = Float32()
        m.data = float(val)
        self._set_force_pub.publish(m)

    def _param_row(self, parent, *, label, unit, var,
                    vmin, vmax, step, hint='', integer=False, snap=None):
        """Linha de parâmetro: label (+ⓘ tooltip) + unidade + spinbox +
        slider. O texto de ajuda vira tooltip no hover — sem ruído inline.
        """
        row = tk.Frame(parent, bg=PANEL); row.pack(fill='x', pady=(5, 3))
        if integer or snap:
            res = 1.0 if integer else float(snap)
            def _snap():
                name = str(var)
                try:
                    raw = self.root.tk.globalgetvar(name)
                    sv = round(float(raw) / res) * res
                    # round() limpa resíduo binário (2.5000…04 → 2.5).
                    sv = int(round(sv)) if integer else round(sv, 6)
                    if str(raw) != str(sv):
                        var.set(sv)
                except (ValueError, tk.TclError):
                    pass   # campo vazio/parcial ou widget destruído
            var.trace_add('write',
                          lambda *_a: self.root.after_idle(_snap))
        top = tk.Frame(row, bg=PANEL); top.pack(fill='x')
        lbl = tk.Label(top, text=label, font=FONT_LBL, bg=PANEL, fg=TEXT,
                       anchor='w')
        lbl.pack(side='left')
        if hint:
            info = tk.Label(top, text='ⓘ', font=FONT_SMALL, bg=PANEL,
                            fg=TEXT_DIM)
            info.pack(side='left', padx=(5, 0))
            _Tooltip(info, hint)
            _Tooltip(lbl, hint)
        tk.Spinbox(top, from_=vmin, to=vmax, increment=step,
                    textvariable=var, width=8, font=FONT_MONO,
                    justify='right', relief='flat', bd=0,
                    highlightthickness=1, highlightbackground=BORDER,
                    highlightcolor=PRIMARY
                    ).pack(side='right', padx=(6, 0), ipady=2)
        tk.Label(top, text=unit, font=FONT_LBL, bg=PANEL, fg=TEXT_MUTED
                 ).pack(side='right')
        ttk.Scale(row, from_=vmin, to=vmax, variable=var,
                   orient='horizontal',
                   style='Tactile.Horizontal.TScale'
                   ).pack(fill='x', pady=(2, 0))
        return row

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
            msg.points.append(pt)
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

    def _update_drag_btn_auto(self, active: bool) -> None:
        """Actualiza o botão de drag a partir do watcher (thread Tk-safe)."""
        self._publish_drag_state(active)
        if not active:
            self._sync_sliders_from_drag()
        btn = self._drag_btn
        if btn is None:
            return
        if active:
            btn.config(text='✋ Drag (auto)', bg=WARN, fg='white',
                       activebackground='#b45309')
            self._set_status(
                'Physical drag detected — simulation following the real arm.', WARN)
        else:
            btn.config(text='✋ Drag OFF', bg=BTN_NEUTRAL, fg=TEXT,
                       activebackground=_shade(BTN_NEUTRAL, -0.08))
            self._set_status('Drag desactivado.', OK)

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
        hw_ok = True
        hw_err = ''
        if self._real_driver is not None and self._robot_connected:
            try:
                self._real_driver.emergency_stop()
            except CR10RealDriverError as exc:
                hw_ok = False
                hw_err = str(exc)
                self.get_logger().error(f'E-STOP real falhou: {exc}')

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
        else:
            self._set_status(
                f'E-STOP: EmergencyStop(1) FAILED ({hw_err}). Motion is '
                'blocked in software (driver + explorer frozen), but the '
                'controller may NOT be alarmed — the arm can still be '
                'enabled. Use the physical E-Stop.', DANGER)

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

    # ROS subscriptions (rodam no executor)
    def _cb_status(self, msg: PalpationStatus):
        with self._lock:
            if msg.phase != self._latest_phase:
                self._phase_t_start = time.time()
                # Nova fase → novo zero do odômetro (fixado no refresh).
                self._phase_p0 = None
            self._latest_phase = msg.phase
            # Fim do run → fecha a gravação que o próprio run abriu. Via
            # `after` porque _stop_recording toca widgets Tk e esta callback
            # roda na thread do executor ROS.
            ended = (msg.phase in ('DONE', 'ABORTED', 'FROZEN')
                     and self._rec_fh is not None and self._rec_auto)
            self._latest_cycle = int(msg.cycle)
            self._latest_cycles_total = int(msg.cycles_total)
            self._paused = bool(msg.paused)
            if msg.speed_mms > 0.0:
                self._latest_speed_mms = float(msg.speed_mms)
            wp = int(getattr(msg, 'wp_index', 0) or 0)
        if ended:
            try:
                self.root.after(0, self._stop_recording)
            except (RuntimeError, tk.TclError):
                pass   # janela fechando
        # MATRIX_MAP: acende o ponto em execução no preview. Fora do lock e
        # via `after` — o Tk só pode ser tocado na sua própria thread, e esta
        # callback roda na thread do executor ROS.
        if wp != getattr(self, '_matrix_live_index', 0):
            self._matrix_live_index = wp
            if getattr(self, '_matrix_canvas', None) is not None:
                try:
                    self.root.after(0, self._redraw_matrix_preview)
                except (RuntimeError, tk.TclError):
                    pass   # janela fechando

    # Refresh do painel direito (Tk thread, 10 Hz)
    def _refresh_status_panel(self):
        try:
            self._status_panel_tick()
        finally:
            self.root.after(100, self._refresh_status_panel)

    def _status_panel_tick(self):
        try:
            tgt_force = float(self.force_sp_var.get())
        except (ValueError, tk.TclError):
            tgt_force = FORCE_SP_DEFAULT
        with self._lock:
            phase     = self._latest_phase
            cycle     = self._latest_cycle
            cyc_total = self._latest_cycles_total
            paused    = self._paused
            f_net     = self._lc_force_net        # positivo = compressão
            lc_ts     = self._lc_force_net_ts
            f_raw     = self._lc_force_raw        # a mesma leitura pré-tare
            lc_tared  = self._lc_tare_done
            phase_t0  = self._phase_t_start
            touch_val = self._touch_value
            touch_ts  = self._touch_last_ts
            joints    = self._latest_joint_rad
            p0        = self._phase_p0

        # ══ ESTADO — roda SEMPRE, mesmo com a aba escondida ═══════════
        # Odômetro do TCP (FK a 10 Hz, não na callback de /joint_states).
        # A FK e a captura do p0 não podem ser gateadas: p0 é a origem da
        # fase, e capturá-lo só quando a aba abre daria uma distância medida
        # a partir do meio da fase.
        p_now = None
        if _fk_tcp is not None and joints is not None and len(joints) >= 6:
            try:
                p_now = _fk_tcp(np.asarray(joints, float),
                                T_end=_T_TCP)[:3, 3]
            except Exception:
                p_now = None
        if p_now is not None and p0 is None:
            p0 = p_now.copy()
            with self._lock:
                self._phase_p0 = p0

        # Históricos das sparklines: alimentados sempre, senão o gráfico
        # abriria um buraco do tamanho do tempo passado em outra aba.
        has_data = lc_ts > 0.0 and (time.time() - lc_ts) < 3.0
        # Indicador do cabeçalho: quadros chegando é o único "online" que a
        # FA7155 tem — ela não se anuncia na USB, quem aparece é o conversor.
        self._cell_dot_lbl.config(fg=OK if has_data else TEXT_DIM)
        self._cell_status_lbl.config(
            text='ONLINE' if has_data else 'OFFLINE',
            fg=OK if has_data else TEXT_DIM)
        if has_data:
            self._spark_data.append((time.time(), f_net))
        touch_fresh = touch_ts > 0.0 and (time.time() - touch_ts) < 3.0
        if touch_fresh:
            self._touch_spark_data.append((time.time(), touch_val))

        # ══ PINTURA — 29 .config() + 2 canvas por tick ════════════════
        # Só interessa com a aba de Palpação à vista.
        if not self._tab_visible(self._palp_tab_frame):
            return

        if p_now is None:
            self.dist_value_lbl.config(text='—  mm', fg=TEXT_DIM)
            self.dist_z_lbl.config(text='—  mm')
        else:
            dist_mm = float(np.linalg.norm(p_now - p0)) * 1000.0
            self.dist_value_lbl.config(
                text=f'{dist_mm:6.2f}  mm',
                fg=TEXT if phase in _PHASE_ENDED else OK)
            self.dist_z_lbl.config(text=f'{p_now[2] * 1000.0:7.2f}  mm')

        if not has_data:
            self.force_value_lbl.config(text='—   N', fg=TEXT_DIM)
            self.force_kgf_lbl.config(text='—   kgf', fg=TEXT_DIM)
            self.force_status_lbl.config(
                text='waiting for /load_cell/force_net (start the force receiver)',
                fg=TEXT_DIM)
            self.err_value_lbl.config(text='—  N', fg=TEXT_DIM)
            self.fz_lbl.config(text='—  N')
            self.fkgf_lbl.config(text='—  kgf')
            self.fx_lbl.config(text='—')
            self.fy_lbl.config(text='—  N')
        else:
            if not lc_tared:
                color, status = WARN, 'tare not done'
            elif f_net > _FORCE_ABORT_LIMIT_N * 0.9:
                color, status = DANGER, f'near the limit ({_FORCE_ABORT_LIMIT_N:.0f} N)'
            elif self._contact_indicator(f_net):
                color, status = OK, 'in contact'
            else:
                color, status = TEXT_MUTED, 'no contact'
            self.force_value_lbl.config(text=f'{f_net:+6.2f}  N', fg=color)
            self.force_kgf_lbl.config(
                text=f'{f_net / _N_PER_KGF:+7.3f}  kgf', fg=color)
            self.force_status_lbl.config(text=status, fg=color)
            self.err_value_lbl.config(
                text=f'{tgt_force:.1f} N  ·  {tgt_force / _N_PER_KGF:.3f} kgf',
                fg=TEXT)
            self.fz_lbl.config(text=f'{f_net:+6.2f} N')
            self.fkgf_lbl.config(text=f'{f_net / _N_PER_KGF:+7.3f} kgf')
            self.fx_lbl.config(text='done' if lc_tared else 'not done')
            self.fy_lbl.config(text=f'{f_raw:+6.2f} N')   # pré-tare

        phase_color = {
            'IDLE': TEXT_MUTED, 'HOME': PRIMARY, 'DESCENDING': WARN,
            'HOLD': OK, 'SLIDING': PRIMARY, 'RETRACT': TEXT_MUTED,
            'MODULATING': OK, 'DONE': OK, 'ABORTED': DANGER,
            # Faltavam as três: CALIBRATING e TRANSIT existem desde o
            # MATRIX_MAP e caíam no cinza genérico, e FROZEN (E-STOP) pintava
            # como se fosse uma fase normal em andamento.
            'CALIBRATING': PRIMARY, 'TRANSIT': PRIMARY, 'FROZEN': DANGER,
        }.get(phase, TEXT)
        phase_txt = phase
        if (cyc_total > 1 and cycle > 0
                and phase not in _PHASE_ENDED):
            phase_txt = f'{phase} · {cycle}/{cyc_total}'
        if paused:
            phase_txt += '  ⏸ PAUSED'
            phase_color = WARN
        self.phase_lbl.config(text=phase_txt, fg=phase_color)

        # Botão Pausar/Retomar segue o estado vindo do explorer.
        if paused:
            self.pause_btn.config(
                text='▶  Resume', bg=OK, fg='white',
                activebackground=_shade(OK, -0.08),
                activeforeground='white')
        else:
            self.pause_btn.config(
                text='⏸  Pause', bg=BTN_NEUTRAL, fg=TEXT,
                activebackground=_shade(BTN_NEUTRAL, -0.08),
                activeforeground=TEXT)

        # Os históricos já foram alimentados acima (bloco de estado); aqui
        # só o desenho.
        self._draw_sparkline(tgt_force)

        if touch_fresh:
            self.touch_value_lbl.config(text=f'{touch_val:+.3f}', fg=TEXT)
            src_txt, _fg = self._touch_source_status(touch_fresh)
            self.touch_status_lbl.config(text=f'receiving ({src_txt})', fg=OK)
        else:
            self.touch_value_lbl.config(text='—', fg=TEXT_DIM)
            self.touch_status_lbl.config(
                text='waiting for touch (connect the STM32 or a UDP receiver)',
                fg=TEXT_DIM)
        self._draw_touch_spark()

        # Cronômetro só por label (sem Progressbar). SLIDING mostra só
        # tempo decorrido (sem distância fixa); CONTACT/RETRACT/CALIBRATING idem.
        elapsed = max(0.0, time.time() - phase_t0)
        if phase in _PHASE_ENDED:
            self.timer_lbl.config(text='—', fg=TEXT_DIM)
        else:
            self.timer_lbl.config(text=f'{elapsed:4.1f}s', fg=phase_color)

    # Disparo da palpação
    def _toggle_pause(self) -> None:
        """Pausa/retoma o experimento: o explorer segura a posição atual
        (_pause_gate) e, com o braço real conectado, pause()/resume() do
        driver congela/retoma a fila de movimento do controlador."""
        with self._lock:
            phase = self._latest_phase
            paused = self._paused
        if phase in ('IDLE', 'DONE', 'ABORTED'):
            self._set_status('Nothing to pause — experiment inactive.',
                             TEXT_DIM)
            return
        new_state = not paused
        msg = Bool(); msg.data = new_state
        self._pause_pub.publish(msg)
        if self._robot_connected and self._real_driver is not None:
            try:
                if new_state:
                    self._real_driver.pause()
                else:
                    self._real_driver.resume()
            except CR10RealDriverError as exc:
                self.get_logger().warning(f'pause/resume real falhou: {exc}')
        # Feedback imediato (o status do explorer confirma em seguida).
        with self._lock:
            self._paused = new_state
        self._set_status(
            'Experiment paused — position held.' if new_state
            else 'Experiment resumed.',
            WARN if new_state else OK)

    # Persistência dos parâmetros da palpação
    def _load_palp_params(self) -> dict:
        try:
            with open(PALPATION_PARAMS_FILE) as fh:
                data = json.load(fh)
            if not isinstance(data, dict):
                return {}
        except (OSError, json.JSONDecodeError):
            return {}
        return self._migrate_palp_params(data)

    # Versão do formato persistido. Sobe quando um default salvo deixa de ser
    # válido e precisa ser DESCARTADO em vez de recarregado.
    _PALP_PARAMS_VERSION = 2

    def _migrate_palp_params(self, data: dict) -> dict:
        """Poda de um arquivo antigo os campos cujo default mudou.

        `hold_tol` (v1 → v2): o arquivo guarda o valor do último start, e o
        último start de toda sessão anterior mandou o default STALE de
        0,15 N — que a partir daí passava a ser reescrito para sempre. Sem
        podá-lo, o conserto da banda (constants.HOLD_TOL_N, 4σ do ruído
        medido) não teria efeito nenhum em quem já usou a GUI uma vez.
        Um valor escolhido de propósito volta com um start.
        """
        try:
            version = int(data.get('params_version', 1))
        except (TypeError, ValueError):
            version = 1
        if version >= self._PALP_PARAMS_VERSION:
            return data
        if 'hold_tol' in data:
            velho = data.pop('hold_tol')
            self.get_logger().info(
                f'[PARAMS] hold_tol={velho} descartado do arquivo de '
                f'preferências (formato v{version}): era o default antigo de '
                f'0,15 N, que sobrescrevia a banda derivada do ruído da '
                f'célula ({_HOLD_TOL_SIGMA:.0f}σ = {_HOLD_TOL_N:.3f} N). '
                'Reajuste em Advanced se o valor era intencional.')
        return data

    def _save_palp_params(self, vals: dict) -> None:
        """Persiste os valores da aba Palpação usados no último start —
        viram os defaults da próxima sessão (como IPs/home já fazem)."""
        try:
            os.makedirs(os.path.dirname(PALPATION_PARAMS_FILE), exist_ok=True)
            with open(PALPATION_PARAMS_FILE, 'w') as fh:
                json.dump({**vals,
                           'params_version': self._PALP_PARAMS_VERSION},
                          fh, indent=2, sort_keys=True)
        except OSError as exc:
            self.get_logger().warning(f'Falha ao salvar parâmetros: {exc}')

    def _draw_sparkline(self, target: float) -> None:
        """Redesenha o gráfico de força (Canvas puro, 10 Hz)."""
        cv = getattr(self, 'spark_canvas', None)
        if cv is None:
            return
        try:
            w = cv.winfo_width()
            h = cv.winfo_height()
            cv.delete('all')
        except tk.TclError:
            return
        if w <= 10 or h <= 10:
            return
        now = time.time()
        window = 30.0
        pts = [(t, f) for t, f in self._spark_data if now - t <= window]
        forces = [f for _, f in pts]
        f_hi = max([target * 1.3, 1.0] + forces)
        f_lo = min([0.0] + forces)
        rng = max(f_hi - f_lo, 0.5)

        def xy(t: float, f: float) -> tuple[float, float]:
            x = w - (now - t) / window * w
            y = (h - 4) - (f - f_lo) / rng * (h - 8)
            return x, y

        y_zero = xy(now, 0.0)[1]
        cv.create_line(0, y_zero, w, y_zero, fill=BORDER)
        y_tgt = xy(now, target)[1]
        cv.create_line(0, y_tgt, w, y_tgt, fill=DANGER, dash=(3, 3))
        if len(pts) >= 2:
            coords: list[float] = []
            for t, f in pts:
                coords.extend(xy(t, f))
            cv.create_line(*coords, fill=PRIMARY, width=2)

    def _draw_touch_spark(self) -> None:
        """Redesenha o gráfico do touch sensor (Canvas puro, 10 Hz) —
        mesmo desenho do sparkline da célula, sem linha de setpoint e com
        autoescala plena (a unidade do STM32 é arbitrária)."""
        cv = getattr(self, 'touch_spark_canvas', None)
        if cv is None:
            return
        try:
            w = cv.winfo_width()
            h = cv.winfo_height()
            cv.delete('all')
        except tk.TclError:
            return
        if w <= 10 or h <= 10:
            return
        now = time.time()
        window = 30.0
        pts = [(t, v) for t, v in self._touch_spark_data if now - t <= window]
        vals = [v for _, v in pts]
        v_hi = max(vals) if vals else 1.0
        v_lo = min(vals + [0.0]) if vals else 0.0
        rng = max(v_hi - v_lo, 1e-3)

        def xy(t: float, v: float) -> tuple[float, float]:
            x = w - (now - t) / window * w
            y = (h - 4) - (v - v_lo) / rng * (h - 8)
            return x, y

        y_zero = xy(now, 0.0)[1]
        cv.create_line(0, y_zero, w, y_zero, fill=BORDER)
        if len(pts) >= 2:
            coords: list[float] = []
            for t, v in pts:
                coords.extend(xy(t, v))
            cv.create_line(*coords, fill=OK, width=2)

    def _on_stop_palpation(self) -> None:
        """Interrompe o experimento em curso: publica /palpation/stop e
        pausa o braço real imediatamente via Halt()."""
        msg = String()
        msg.data = 'stop'
        self._stop_pub.publish(msg)
        # Halt paralisa o movimento atual do braço real sem desabilitar.
        if self._robot_connected and self._real_driver is not None:
            try:
                self._real_driver.halt()
            except CR10RealDriverError as exc:
                self.get_logger().warning(f'Halt após stop falhou: {exc}')
        # Congela o poll loop: define last_target = posição atual para que
        # o dedup bloqueie novos ServoJ até o braço realmente se mover de novo.
        cur = self._latest_joint_rad
        if cur is not None:
            self._mirror_last_target = np.asarray(cur, dtype=np.float64)
        self._set_status('Palpation stopped by the operator.', WARN)

    def _on_start(self):
        # Gate (defesa em profundidade — ver também o bloqueio da aba em
        # _build_body): sem o touch_tool a palpação fica indisponível.
        if getattr(self, '_palpation_blocked', False):
            self._set_status(
                'Palpation mode unavailable: open the launch with '
                'end_effector:=touch_tool.', WARN)
            return
        # Reentrância: o caminho sem force_receiver agenda o start para 1,8 s
        # depois e VOLTA, deixando o botão vivo — dois cliques nessa janela
        # publicavam dois /palpation/start. O explorer tem sua própria guarda,
        # mas evitar a segunda publicação é mais barato que confiar nela.
        if self._starting_palpation:
            self._set_status('Start already in progress…', WARN)
            return
        # E-STOP travado: o driver recusaria todo movimento e o run morreria
        # na primeira fase. Barrar aqui diz o porquê, em vez de deixar o
        # operador ver um aborto sem causa aparente.
        if getattr(self, '_estop_latched', False):
            self._set_status(
                'E-STOP is engaged — press E-STOP again to release and '
                're-enable the robot before starting.', DANGER)
            return
        # Janela cega entre publicar e o explorer se marcar ocupado: o status
        # chega a 10 Hz, então por ~100-300 ms `_latest_phase` ainda diz IDLE.
        # Sem este gate, cliques repetidos nessa janela publicavam vários
        # /palpation/start — o explorer recusava o 2º pelo _busy, mas o tópico
        # é TRANSIENT_LOCAL e o latch acabava carregando a duplicata.
        since = time.time() - self._start_published_t
        if since < START_REPUBLISH_LOCKOUT_S:
            self._set_status(
                f'Start already sent {since:.1f}s ago — waiting for the '
                'explorer to pick it up.', WARN)
            return
        if self._latest_phase not in ('IDLE', 'DONE', 'ABORTED'):
            self._set_status(
                f'Experiment already running ({self._latest_phase}) — '
                'use Stop first.', WARN)
            return
        self._starting_palpation = True
        # Satura cada parâmetro ao seu intervalo válido antes de enviar,
        # tanto para a publicação quanto para o que o usuário vê nos
        # spinboxes/sliders.
        self._suppressing = True
        try:
            speed      = self._clamp_var(self.speed_var, SPEED_MIN, SPEED_MAX)
            depth      = self._clamp_var(self.depth_var, DEPTH_MIN, DEPTH_MAX)
            force_sp   = self._clamp_var(self.force_sp_var,
                                          FORCE_SP_MIN, FORCE_SP_MAX,
                                          default=FORCE_SP_DEFAULT)
            slide_dist = self._clamp_var(self.slide_dist_var,
                                          SLIDE_DIST_MIN, SLIDE_DIST_MAX)
            approach   = self._clamp_var(self.approach_var,
                                          APPROACH_MIN, APPROACH_MAX,
                                          default=APPROACH_DEFAULT)
            repeats    = self._clamp_var(self.repeats_var,
                                          REPEAT_MIN, REPEAT_MAX,
                                          default=REPEAT_DEFAULT)
            # Piso e default vêm da MESMA lei do explorer (constants):
            # nada de número próprio aqui, que era o que fazia o retune de
            # 19/08/2026 nunca valer num run lançado pela tela.
            hold_tol     = self._clamp_var(
                self.hold_tol_var, round(_HOLD_TOL_N, 3), 2.0,
                default=round(_hold_tol_n_for(
                    force_sp if force_sp is not None
                    else FORCE_SP_DEFAULT), 3))
            hold_stable  = self._clamp_var(self.hold_stable_var, 0.2, 5.0,
                                            default=1.0)
            hold_timeout = self._clamp_var(self.hold_timeout_var, 2.0, 60.0,
                                            default=12.0)
            # Faixa alargada de 50 para 500 µm: com ponteira de silicone
            # (0,62 N/mm medidos) 50 µm valem 0,03 N por passo, e o hold
            # rastejava. Ver _QS_DX_MAX_M no explorer — quem limita por
            # física é o teto por ΔF, este é só o teto absoluto.
            hold_dx      = self._clamp_var(self.hold_dx_var, 1.0, 500.0,
                                            default=100.0)
            hold_df      = self._clamp_var(self.hold_df_var, 0.05, 1.0,
                                            default=0.3)
            slide_slope  = self._clamp_var(self.slide_slope_var, -10.0, 10.0,
                                            default=0.0)
            fmod_hz      = self._clamp_var(self.fmod_hz_var,
                                            FMOD_HZ_MIN, FMOD_HZ_MAX,
                                            default=2.0)
            fmod_min     = self._clamp_var(self.fmod_min_var,
                                            FORCE_SP_MIN, FORCE_SP_MAX,
                                            default=FORCE_SP_MIN)
            fmod_max     = self._clamp_var(self.fmod_max_var,
                                            FORCE_SP_MIN, FORCE_SP_MAX,
                                            default=FORCE_SP_DEFAULT)
            fmod_cycles  = self._clamp_var(self.fmod_cycles_var,
                                            FMOD_CYCLES_MIN, FMOD_CYCLES_MAX,
                                            default=FMOD_CYCLES_DEFAULT)
            step_size    = self._clamp_var(self.step_size_var, 0.0, 5.0,
                                            default=0.0)
            step_start   = self._clamp_var(self.step_start_var,
                                            FORCE_SP_MIN, FORCE_SP_MAX,
                                            default=FORCE_SP_MIN)
            step_max     = self._clamp_var(self.step_max_var,
                                            FORCE_SP_MIN, FORCE_SP_MAX,
                                            default=FORCE_SP_DEFAULT)
            step_dwell   = self._clamp_var(self.step_dwell_var, 0.0, 120.0,
                                            default=5.0)
            align_points  = self._clamp_var(
                self.align_points_var, PROBE_ALIGN_POINTS_MIN,
                PROBE_ALIGN_POINTS_MAX, default=PROBE_ALIGN_POINTS_DEFAULT)
            align_radius  = self._clamp_var(
                self.align_radius_var, PROBE_ALIGN_RADIUS_MM_MIN,
                PROBE_ALIGN_RADIUS_MM_MAX,
                default=PROBE_ALIGN_RADIUS_MM_DEFAULT)
            align_force   = self._clamp_var(
                self.align_force_var, FORCE_SP_MIN, FORCE_SETPOINT_MAX_N,
                default=PROBE_ALIGN_FORCE_N_DEFAULT)
            align_retract = self._clamp_var(
                self.align_retract_var, PROBE_ALIGN_RETRACT_MM_MIN,
                PROBE_ALIGN_RETRACT_MM_MAX,
                default=PROBE_ALIGN_RETRACT_MM_DEFAULT)
            align_tilt    = self._clamp_var(
                self.align_tilt_var, 1.0, PROBE_ALIGN_TILT_HARD_MAX_DEG,
                default=PROBE_ALIGN_TILT_MAX_DEG_DEFAULT)
        finally:
            self._suppressing = False
        if None in (speed, depth, force_sp, slide_dist, approach):
            self._starting_palpation = False
            self._set_status('Invalid parameters.', DANGER)
            return
        sf_pct = self._clamp_var(self.speed_factor_var,
                                  SPEED_FACTOR_MIN, SPEED_FACTOR_MAX,
                                  default=SPEED_FACTOR_DEFAULT)

        # ── Força modulada: normaliza antes de publicar ───────────────
        # Campo vazio no spinbox faz _clamp_var devolver None — cai no
        # default em vez de abortar o start por causa de um parâmetro que
        # só é usado em TOUCH.
        fmod_hz     = float(fmod_hz     if fmod_hz     is not None else 2.0)
        fmod_min    = float(fmod_min    if fmod_min    is not None
                            else FORCE_SP_MIN)
        fmod_max    = float(fmod_max    if fmod_max    is not None
                            else FORCE_SP_DEFAULT)
        fmod_cycles = int(fmod_cycles if fmod_cycles is not None
                          else FMOD_CYCLES_DEFAULT)
        # min/max invertidos: ordena aqui para que o valor PERSISTIDO e o
        # log batam com o que o explorer vai executar (_ForceProfile também
        # ordena, mas silenciosamente).
        if fmod_min > fmod_max:
            fmod_min, fmod_max = fmod_max, fmod_min
        # ── Escada de força: normaliza e valida ──────────────────────
        step_size  = float(step_size  if step_size  is not None else 0.0)
        step_start = float(step_start if step_start is not None
                           else FORCE_SP_MIN)
        step_max   = float(step_max   if step_max   is not None
                           else FORCE_SP_DEFAULT)
        step_dwell = float(step_dwell if step_dwell is not None else 5.0)
        # Só MANUAL executa a escada — nos demais manda 0 explícito para o
        # `ros2 topic echo` não sugerir patamares que não vão acontecer.
        if self.mode_var.get() != 'MANUAL':
            step_size = 0.0
        elif step_size > 0.0:
            if step_max <= step_start:
                self._set_status(
                    'Staircase needs Peak > First Level — running Manual '
                    'with a single setpoint.', WARN)
                step_size = 0.0
            else:
                _lv = staircase_levels(step_start, step_size, step_max)
                if not _lv:
                    # Recusa aqui, com o braço parado: o explorer também
                    # recusaria, mas depois de já ter descido ao contato.
                    self._starting_palpation = False
                    self._set_status(
                        f'Staircase needs more than {STEP_MAX_LEVELS} '
                        f'plateaus — increase the step size.', DANGER)
                    return
                self.get_logger().info(
                    f'[DEGRAU] {len(_lv)} patamares, {step_dwell:g} s cada — '
                    f'pelo menos {len(_lv) * step_dwell / 60.0:.1f} min '
                    'de dwell.')

        fmod_shape = str(self.fmod_shape_var.get() or 'OFF').upper()
        if fmod_shape not in FMOD_SHAPES:
            fmod_shape = 'OFF'
        # Só TOUCH executa o perfil — nos outros modos manda OFF explícito
        # para que o `ros2 topic echo` não sugira uma onda que não roda.
        if self.mode_var.get() != 'TOUCH':
            fmod_shape = 'OFF'
        if fmod_shape != 'OFF':
            _amp = 0.5 * (fmod_max - fmod_min)
            if _amp <= 1e-3:
                self._set_status(
                    'Modulated force needs Min < Max — running with the '
                    'constant setpoint.', WARN)
                fmod_shape = 'OFF'
            elif fmod_hz > 4.0:
                # Não bloqueia: o explorer executa e loga. Só avisa antes.
                self._set_status(
                    f'{fmod_hz:.1f} Hz is above the 4 Hz the 33 Hz loop '
                    f'tracks — the wave will be coarse.', WARN)

        # ── Calibração do ângulo de ataque: normaliza antes de publicar ──
        # Spinbox vazio faz _clamp_var devolver None — cai no default em vez
        # de abortar o start, como nos demais avançados.
        align_on      = bool(self.align_on_var.get())
        align_points  = int(align_points if align_points is not None
                            else PROBE_ALIGN_POINTS_DEFAULT)
        align_radius  = float(align_radius if align_radius is not None
                              else PROBE_ALIGN_RADIUS_MM_DEFAULT)
        align_force   = float(align_force if align_force is not None
                              else PROBE_ALIGN_FORCE_N_DEFAULT)
        align_retract = float(align_retract if align_retract is not None
                              else PROBE_ALIGN_RETRACT_MM_DEFAULT)
        align_tilt    = float(align_tilt if align_tilt is not None
                              else PROBE_ALIGN_TILT_MAX_DEG_DEFAULT)
        if align_on:
            # O explorer satura a sonda no setpoint do ensaio; avisar aqui
            # evita a surpresa de pedir 2 N de sonda e ver 0,8 N no log.
            if align_force > float(force_sp):
                self._set_status(
                    f'Probe force capped at the {float(force_sp):.1f} N '
                    'setpoint — probing is never heavier than the run.', WARN)
            self.get_logger().info(
                f'[ALIGN] calibração LIGADA: {align_points} toques a '
                f'{min(align_force, float(force_sp)):.2f} N num círculo de '
                f'{align_radius:.1f} mm, recuo de {align_retract:.1f} mm '
                f'antes de girar o punho, desvio máximo {align_tilt:.1f}°. '
                f'Custa {align_points} descidas extras antes da medição.')

        # ── MATRIX_MAP: gera e valida a grade ANTES de publicar ──────
        # Uma grade inválida tem de parar aqui, com o braço parado: depois
        # do Start o robô já estará descendo para achar a origem.
        mode_now = self.mode_var.get()
        matrix_wps: list[tuple[float, float]] = []
        matrix_safe_z = MATRIX_SAFE_Z_MM_DEFAULT
        matrix_transit = MATRIX_TRANSIT_MMS_DEFAULT
        if mode_now == 'MATRIX_MAP':
            matrix_wps, err = self._matrix_waypoints()
            if err or not matrix_wps:
                self._starting_palpation = False
                self._set_status(f'Matrix grid invalid: {err}', DANGER)
                return
            matrix_safe_z = self._clamp_var(
                self.matrix_safe_z_var, MATRIX_SAFE_Z_MM_MIN,
                MATRIX_SAFE_Z_MM_MAX, default=MATRIX_SAFE_Z_MM_DEFAULT)
            matrix_transit = self._clamp_var(
                self.matrix_transit_var, MATRIX_TRANSIT_MMS_MIN,
                MATRIX_TRANSIT_MMS_MAX, default=MATRIX_TRANSIT_MMS_DEFAULT)
            # O Safe Z é também o curso da descida em cada ponto: se a
            # profundidade máxima de segurança for menor, a descida esgota o
            # curso antes de tocar e todo waypoint aborta por 'no_contact'.
            if depth is not None and float(depth) < float(matrix_safe_z) * 1.5:
                depth = min(DEPTH_MAX, float(matrix_safe_z) * 1.5)
                self._suppressing = True
                try:
                    self.depth_var.set(depth)
                finally:
                    self._suppressing = False
                self.get_logger().info(
                    f'[MATRIX] Max Descent Depth elevado para {depth:.1f} mm '
                    f'(1,5 × Safe Z) — o curso precisa cobrir a descida '
                    'a partir do Safe Z.')

        # Persiste os valores em unidades da GUI — defaults da próxima sessão.
        self._save_palp_params({
            'speed': float(speed), 'depth': float(depth),
            'force_sp': float(force_sp), 'repeats': int(repeats),
            'slide_dist': float(slide_dist), 'approach': float(approach),
            'slide_dir': self.slide_dir_var.get(),
            'hold_tol': float(hold_tol), 'hold_stable': float(hold_stable),
            'hold_timeout': float(hold_timeout),
            'hold_dx_max': float(hold_dx), 'hold_df_max': float(hold_df),
            'slide_slope_deg': float(slide_slope),
            'mode': self.mode_var.get(),
            'matrix_shape': self.matrix_shape_var.get(),
            'matrix_sizing': self.matrix_sizing_var.get(),
            'matrix_path': self.matrix_path_var.get(),
            'matrix_step_x': float(self.matrix_step_x_var.get()),
            'matrix_step_y': float(self.matrix_step_y_var.get()),
            'matrix_width': float(self.matrix_width_var.get()),
            'matrix_height': float(self.matrix_height_var.get()),
            'matrix_cols': int(self.matrix_cols_var.get()),
            'matrix_rows': int(self.matrix_rows_var.get()),
            'matrix_safe_z': float(matrix_safe_z),
            'matrix_transit': float(matrix_transit),
            # Chaves iguais às lidas por _f()/sv na construção dos widgets.
            'force_mod_shape': fmod_shape,
            'force_mod_hz': float(fmod_hz),
            'force_mod_min_n': float(fmod_min),
            'force_mod_max_n': float(fmod_max),
            'force_mod_cycles': int(fmod_cycles),
            'step_size_n': float(step_size),
            'step_start_n': float(step_start),
            'step_max_n': float(step_max),
            'step_dwell_s': float(step_dwell),
            'probe_align_on': bool(align_on),
            'probe_align_points': int(align_points),
            'probe_align_radius_mm': float(align_radius),
            'probe_align_force_n': float(align_force),
            'probe_align_retract_mm': float(align_retract),
            'probe_align_tilt_max_deg': float(align_tilt),
        })
        payload = {
            'speed_mms':          float(speed),
            'depth_mm':           float(depth),
            'force_n':            float(force_sp),
            'slide_dist_mm':      float(slide_dist),
            'approach_speed_mms': float(approach),
            'slide_dir':          self.slide_dir_var.get(),
            'repeats':            int(repeats if repeats is not None
                                      else REPEAT_DEFAULT),
            'speed_factor_pct':   float(sf_pct if sf_pct is not None
                                         else SPEED_FACTOR_DEFAULT),
            'hold_tol_n':         float(hold_tol),
            'hold_stable_s':      float(hold_stable),
            'hold_timeout_s':     float(hold_timeout),
            'hold_dx_max_um':     float(hold_dx),
            'hold_df_max_n':      float(hold_df),
            # Descida em dois estágios (só o caminho MovL/robô real).
            'slide_slope_deg':    float(slide_slope),
            'mode':               self.mode_var.get(),
            # Força modulada (só TOUCH; 'OFF' nos demais modos). A
            # persistência NÃO vem daqui — o _save_palp_params acima leva
            # sua própria cópia, com as chaves que o _f() relê na abertura.
            'force_mod_shape':    fmod_shape,
            'force_mod_hz':       float(fmod_hz),
            'force_mod_min_n':    float(fmod_min),
            'force_mod_max_n':    float(fmod_max),
            'force_mod_cycles':   int(fmod_cycles),
            # Escada de força (só MANUAL; 0 nos demais = escada desligada).
            'step_size_n':        float(step_size),
            'step_start_n':       float(step_start),
            'step_max_n':         float(step_max),
            'step_dwell_s':       float(step_dwell),
            # Calibração do ângulo de ataque. 'ON'/'OFF' explícitos: com o
            # campo preenchido o explorer IGNORA os parâmetros ROS
            # probe_align_* e obedece a GUI, que é a fonte visível ao usuário.
            'probe_align':            'ON' if align_on else 'OFF',
            'probe_align_points':     int(align_points),
            'probe_align_radius_mm':  float(align_radius),
            'probe_align_force_n':    float(align_force),
            'probe_align_retract_mm': float(align_retract),
            'probe_align_tilt_max_deg': float(align_tilt),
            # Home customizada: explorer leva o braço PARA CÁ antes de
            # descer.
            'home_deg': [float(self._arm_home_deg[j]) for j in ARM_JOINTS],
            # MATRIX_MAP: waypoints em mm relativos à origem, na ordem de
            # visita (serpentina). Vazio nos demais modos.
            'matrix_waypoints_mm': matrix_wps,
            'matrix_safe_z_mm':    float(matrix_safe_z),
            'matrix_transit_mms':  float(matrix_transit),
            'matrix_shape':        self.matrix_shape_var.get(),
        }
        # Garante SpeedFactor=10% no braço real durante a palpação.
        # Velocidades altas são perigosas nesse protocolo — impõe aqui
        # independente do slider de "Velocidade bruta" da aba manual.
        if self._robot_connected and self._real_driver is not None:
            try:
                self._real_driver._send_dash('SpeedFactor(10)')
                self.get_logger().info('[PALP] SpeedFactor(10) aplicado para palpação')
                # Sincroniza o slider para que a GUI reflita o valor real.
                self._suppressing = True
                try:
                    self.speed_factor_var.set(10)
                finally:
                    self._suppressing = False
            except CR10RealDriverError as exc:
                self.get_logger().warning(f'SpeedFactor(10) falhou: {exc}')

        # Pré-voo do canal TÁTIL. O start garante sozinho a célula
        # (force_receiver + tare) e o logger, mas NÃO garantia o toque: com o
        # STM32 fora do cabo o run ia até o fim e as colunas taxel_* saíam
        # vazias, sem uma linha de aviso. Avisa e deixa seguir — coletar só
        # força é um ensaio legítimo; perder o tátil sem saber, não.
        src = self._touch_source
        if src is None or not src.connected:
            self._set_status(
                'Touch sensor not connected — this run will record force '
                'only (taxel_* columns empty).', WARN)
            self.get_logger().warn(
                '[TOUCH] fonte tátil ausente no início do run: as colunas '
                'taxel_* do samples.csv sairão vazias.')
        elif not src.is_fresh():
            self._set_status(
                f'Touch sensor on {src.port} is silent — check the firmware; '
                'recording force only.', WARN)
            self.get_logger().warn(
                f'[TOUCH] fonte tátil conectada em {src.port} mas sem dados '
                'frescos — taxel_* podem sair vazios.')

        # Garante os nós auxiliares da palpação touch_receiver: o spawn
        # dedup-a contra publishers existentes.
        self._spawn_touch_receiver()
        # palpation_logger: sem ele nada é gravado em ~/touch_pack_runs
        # (caso da GUI standalone, fora do launch).
        self._ensure_palpation_logger()

        # O ft_receiver sobe pelo launch e faz auto-tare de partida sozinho.
        # Aqui só se garante que ALGUM tare aconteceu antes de começar: se o
        # auto-tare não fechou (célula ainda instável na partida), este é o
        # último ponto em que dá para zerar sem já estar em contato.
        if not self._lc_tare_done:
            self._lc_do_tare()
        self._do_palpation_start(payload)

    def _do_palpation_start(self, payload: dict) -> None:
        """Envia /palpation/start após garantir que a LC está pronta.

        Único ponto de publicação — é aqui que a guarda de reentrância do
        _on_start é liberada, inclusive no caminho que passou pelo atraso de
        1,8 s do auto-tare.
        """
        self._starting_palpation = False
        # Carimba a publicação: é o que fecha a janela cega até o explorer
        # aparecer ocupado no status (ver o gate em _on_start).
        self._start_published_t = time.time()
        # Limpa o log de movimentos da sessão anterior para que o dedup do
        # mirror poll não bloqueie os primeiros ServoJ desta sessão.
        with self._mirror_timer_lock:
            self._mirror_last_target = None

        msg = PalpationStart()
        msg.speed_mms          = float(payload['speed_mms'])
        msg.depth_mm           = float(payload['depth_mm'])
        msg.force_n            = float(payload['force_n'])
        msg.slide_dist_mm      = float(payload['slide_dist_mm'])
        msg.approach_speed_mms = float(payload['approach_speed_mms'])
        msg.slide_dir          = str(payload['slide_dir'])
        msg.repeats            = int(payload['repeats'])
        msg.speed_factor_pct   = float(payload['speed_factor_pct'])
        msg.home_deg           = [float(v) for v in payload['home_deg']]
        msg.hold_tol_n         = float(payload['hold_tol_n'])
        msg.hold_stable_s      = float(payload['hold_stable_s'])
        msg.hold_timeout_s     = float(payload['hold_timeout_s'])
        msg.hold_dx_max_um     = float(payload['hold_dx_max_um'])
        msg.hold_df_max_n      = float(payload['hold_df_max_n'])
        msg.slide_slope_deg    = float(payload.get('slide_slope_deg', 0.0))
        msg.mode               = str(payload.get('mode', 'SLIDE'))
        # Força modulada — o explorer lê no _cb_start e, com shape não
        # vazio, ignora os parâmetros ROS force_mod_* em favor daqui.
        msg.force_mod_shape    = str(payload.get('force_mod_shape', 'OFF'))
        msg.force_mod_hz       = float(payload.get('force_mod_hz', 0.0))
        msg.force_mod_min_n    = float(payload.get('force_mod_min_n', 0.0))
        msg.force_mod_max_n    = float(payload.get('force_mod_max_n', 0.0))
        msg.force_mod_cycles   = int(payload.get('force_mod_cycles', 0))
        # Escada de força — step_size_n = 0 desliga e o MANUAL volta a ser o
        # hold infinito com setpoint ao vivo.
        msg.step_size_n        = float(payload.get('step_size_n', 0.0))
        msg.step_start_n       = float(payload.get('step_start_n', 0.0))
        msg.step_max_n         = float(payload.get('step_max_n', 0.0))
        msg.step_dwell_s       = float(payload.get('step_dwell_s', 0.0))
        # Calibração do ângulo de ataque — campo não vazio faz o explorer
        # ignorar os parâmetros ROS probe_align_* em favor daqui.
        msg.probe_align            = str(payload.get('probe_align', ''))
        msg.probe_align_points     = int(payload.get('probe_align_points', 0))
        msg.probe_align_radius_mm  = float(
            payload.get('probe_align_radius_mm', 0.0))
        msg.probe_align_force_n    = float(
            payload.get('probe_align_force_n', 0.0))
        msg.probe_align_retract_mm = float(
            payload.get('probe_align_retract_mm', 0.0))
        msg.probe_align_tilt_max_deg = float(
            payload.get('probe_align_tilt_max_deg', 0.0))
        # Carimbo de emissão: o tópico é TRANSIENT_LOCAL (o logger sobe depois
        # do publish e precisa do latch), então o explorer usa esta idade para
        # não confundir o latch de uma sessão anterior com um pedido novo.
        msg.stamp              = self.get_clock().now().to_msg()
        # Identidade do run em disco. NÃO derivada de msg.stamp: com
        # use_sim_time o carimbo vem do relógio do Gazebo, que recomeça do
        # zero a cada launch — todos os runs viravam 19691231_* e o launch
        # seguinte sobrescrevia a coleta anterior. Vai na mensagem para que
        # a GUI e o palpation_logger gravem na MESMA pasta sem depender de
        # chamar strftime no mesmo segundo.
        msg.run_id             = new_run_id()
        # MATRIX_MAP: geometry_msgs/Point[] em METROS (convenção ROS),
        # RELATIVOS à origem que o explorer descobre no primeiro contato.
        # z fica em 0.0 — a descida é sempre vertical a partir do Safe Z.
        msg.waypoints = [
            Point(x=float(x) / 1000.0, y=float(y) / 1000.0, z=0.0)
            for x, y in payload.get('matrix_waypoints_mm', [])
        ]
        msg.safe_z_mm          = float(payload.get('matrix_safe_z_mm', 0.0))
        msg.transit_speed_mms  = float(payload.get('matrix_transit_mms', 0.0))
        msg.grid_shape         = str(payload.get('matrix_shape', ''))
        self._start_pub.publish(msg)
        # Gravação dos CSVs crus atrelada ao run, com o MESMO run_id que vai
        # na mensagem — antes ela dependia de alguém lembrar de clicar em
        # "Gravar", e o arquivo que carrega o relógio do STM32 (sensors.csv)
        # simplesmente não existia na maioria dos runs.
        # Uma gravação MANUAL já em curso é respeitada: não reabrimos nada.
        if self._rec_fh is None:
            self._start_recording(run_id=msg.run_id, mode=msg.mode, auto=True)
        # Quando a mão real está conectada via ECI, aciona o grip FINGER
        # (Index estendido) automaticamente, já que o tactile_explorer
        # publica a pose da mão apenas no tópico do sim (ros2_control).
        self._send_eci_grip(7, 'Finger — palpation (Index extended)')
        # Envia posição explícita com velocidade controlada pelo slider
        # (SetCurrentGrip usa velocidade interna do firmware; SetDigitPosn permite controle).
        if self._eci_enabled:
            self._schedule_eci_posn(HAND_POINT_DEG)
        mode_now = str(payload.get('mode', 'SLIDE'))
        if mode_now == 'TOUCH':
            rep_txt = (f'{payload["repeats"]} touches | '
                       if payload.get('repeats', 1) > 1 else '1 touch | ')
            self._set_status(
                f'/palpation/start — TOUCH | {rep_txt}'
                f'F={payload["force_n"]:.2f} '
                f'± {payload["hold_tol_n"]:.2f} N | '
                f'joint vel {payload["speed_factor_pct"]:.0f}%.',
                OK)
        elif mode_now == 'MANUAL':
            self._set_status(
                f'/palpation/start — MANUAL | F={payload["force_n"]:.2f} N '
                f'(live-adjustable) | infinite HOLD until Stop | '
                f'joint vel {payload["speed_factor_pct"]:.0f}%.',
                OK)
        elif mode_now == 'MATRIX_MAP':
            n_wp = len(payload.get('matrix_waypoints_mm', []))
            self._set_status(
                f'/palpation/start — MATRIX MAP | origin probe + {n_wp} '
                f'waypoints | F={payload["force_n"]:.2f} '
                f'± {payload["hold_tol_n"]:.2f} N at each | '
                f'Safe Z +{payload["matrix_safe_z_mm"]:.1f} mm | '
                f'joint vel {payload["speed_factor_pct"]:.0f}%.',
                OK)
        else:
            rep_txt = (f'{payload["repeats"]}× | '
                       if payload.get('repeats', 1) > 1 else '')
            self._set_status(
                f'/palpation/start — {rep_txt}'
                f'v={payload["speed_mms"]:.1f} mm/s, '
                f'F={payload["force_n"]:.2f} '
                f'± {payload["hold_tol_n"]:.2f} N, '
                f'dir={payload["slide_dir"]} | '
                f'joint vel {payload["speed_factor_pct"]:.0f}%.',
                OK)

    def _set_status(self, text: str, color: str = TEXT_MUTED):
        self.status_var.set(text)
        try:
            self._status_lbl.config(fg=color)
            self._status_dot.config(
                fg=color if color != TEXT_MUTED else TEXT_DIM)
        except AttributeError:
            pass  # statusbar ainda não foi construída

    # Loop ROS em thread separada
    def _spin_ros(self):
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception as exc:
                log.error('[SPIN] spin_once falhou: %s', exc)
                if not rclpy.ok():
                    break
                # Continua girando — uma exceção isolada não deve parar o executor.

    def _on_close(self):
        self._stop_event.set()
        # Fecha a gravação CSV em andamento (flush + close) e o loop da aba.
        if self._rec_fh is not None:
            try:
                self._stop_recording()
            except Exception:
                pass
        if self._sensors_after is not None:
            try:
                self.root.after_cancel(self._sensors_after)
            except Exception:
                pass
            self._sensors_after = None
        # Para o tick da viewport 3D antes do destroy() — mesmo motivo da
        # animação do touch sensor logo abaixo (timer Tk num canvas morto).
        if self._manip_view is not None:
            try:
                self._manip_view.stop()
            except Exception:
                pass
        if self._manip_release_after is not None:
            try:
                self.root.after_cancel(self._manip_release_after)
            except Exception:
                pass
            self._manip_release_after = None
        # Para a animação (blit) do touch sensor ANTES de destruir a janela —
        # senão o timer Tk dela pode disparar durante/após o destroy() e
        # tentar desenhar num canvas morto (traceback no fechamento).
        anim = getattr(self, '_touch_anim', None)
        if anim is not None:
            try:
                anim.event_source.stop()
            except Exception:
                pass
            self._touch_anim_running = False
        # Encerra a thread de leitura serial do touch sensor.
        if self._touch_source is not None:
            try:
                self._touch_source.stop()
            except Exception:
                pass
        # A serial da célula é do ft_receiver — nada a fechar aqui.
        self._stop_robot_heartbeat()
        self._robot_reconnecting = False
        # Idem para o watchdog da mão — se não desligar, o re-spawn vai
        # ser tentado durante o shutdown.
        self._hand_should_be_alive = False
        self._stop_hand_watchdog()
        # Idem para o touch_receiver_node.
        try:
            self._kill_touch_receiver()
        except Exception:
            pass
        # Idem para o palpation_logger spawnado pela GUI (SIGTERM dá ao
        # rclpy a chance de fechar o run e gerar o relatório).
        logger_proc = self._logger_proc
        self._logger_proc = None
        if logger_proc is not None and logger_proc.poll() is None:
            try:
                os.killpg(os.getpgid(logger_proc.pid), signal.SIGTERM)
                logger_proc.wait(timeout=3.0)
            except Exception:
                pass
        # Cancela callbacks Tk pendentes — disparar após `root.destroy()`
        # gera TclError ou crash.
        if self._eci_posn_after is not None:
            try:
                self.root.after_cancel(self._eci_posn_after)
            except Exception:
                pass
            self._eci_posn_after = None
        with self._mirror_timer_lock:
            if self._mirror_timer is not None:
                self._mirror_timer.cancel()
                self._mirror_timer = None
        # Apaga o LED da mão antes de matar o subprocesso (mesmo caminho de
        # _disconnect_real_hand).
        if self._eci_enabled and self._hand_powered:
            self._send_hand_poweroff_blocking(timeout_s=3.0)
            self._hand_powered = False
        self._terminate_hand_subprocess()
        # Fecha sockets do CR10.
        if self._real_driver is not None:
            try:
                self._real_driver.stop()
            except CR10RealDriverError:
                pass
            try:
                self._real_driver.close()
            except OSError:
                pass
        try:
            self.root.destroy()
        except Exception:
            pass

    # ── Manipulação 3D ───────────────────────────────────────────
    # NÃO virou mixin: depende de helpers e guardas de import
    # OPCIONAL definidos no escopo deste módulo (_fk_tcp, _rpy_deg,
    # _MANIP3D_OK, _URDF_SCENE_OK, _build_scene…). Extraí-la exige
    # mover esses helpers junto, o que deixa de ser recorte
    # mecânico — ver a nota no fim de gui_loadcell.py.

    def _manip_T_end(self) -> np.ndarray | None:
        """Transform flange→TCP do efetuador com que a célula foi aberta."""
        if self._end_effector == 'hand' and _T_HAND is not None:
            return _T_HAND
        return _T_TCP
    def _build_manip3d_tab(self, root: tk.Frame) -> None:
        body = tk.Frame(root, bg=BG)
        body.pack(fill='both', expand=True, padx=18, pady=18)

        col_view = tk.Frame(body, bg=BG)
        col_side = tk.Frame(body, bg=BG, width=330)
        col_view.pack(side='left', fill='both', expand=True, padx=(0, 12))
        col_side.pack(side='right', fill='both')
        col_side.pack_propagate(False)

        self._manip_view = Manip3DView(
            col_view,
            on_q=self._manip_on_q,
            q_provider=self._manip_q_provider,
            on_state=self._manip_on_state,
            on_drag_change=self._manip_on_drag_change,
            T_end=self._manip_T_end())
        self._manip_view.pack(fill='both', expand=True)

        # TCP ao vivo
        card_tcp = self._card(col_side, 'TCP — live pose', expand=False)
        self._manip_x_lbl = self._kv(card_tcp, 'x', '—')
        self._manip_y_lbl = self._kv(card_tcp, 'y', '—')
        self._manip_z_lbl = self._kv(card_tcp, 'z', '—')
        tk.Frame(card_tcp, bg=BORDER, height=1).pack(fill='x', pady=6)
        self._manip_roll_lbl  = self._kv(card_tcp, 'roll',  '—')
        self._manip_pitch_lbl = self._kv(card_tcp, 'pitch', '—')
        self._manip_yaw_lbl   = self._kv(card_tcp, 'yaw',   '—')
        tk.Frame(card_tcp, bg=BORDER, height=1).pack(fill='x', pady=6)
        self._manip_err_lbl = self._kv(card_tcp, 'tracking lag', '—')
        self._manip_manip_lbl = self._kv(card_tcp, 'manipulability', '—')

        # Opções do arrasto
        card_opt = self._card(col_side, 'Drag options', expand=False)
        self._manip_lock_var = tk.BooleanVar(value=True)
        self._manip_chk(
            card_opt, 'Lock tool orientation', self._manip_lock_var,
            self._manip_apply_options,
            'Keeps the current TCP orientation while you drag: the mouse '
            'moves the POINT, the wrist follows to preserve the attitude. '
            'Unchecking frees the wrist — the arm reaches the point with '
            'whatever orientation the DLS finds, which is looser but goes '
            'further before hitting a joint limit.')
        self._manip_mirror_var = tk.BooleanVar(value=False)
        self._manip_mirror_chk = self._manip_chk(
            card_opt, 'Mirror to the real CR10', self._manip_mirror_var,
            self._manip_apply_options,
            'OFF (default): dragging moves ONLY the simulated arm. ON (and '
            'in MIRROR mode): the pose reached is sent to the real CR10 as '
            'MovJ once the drag settles — the 80 ms debounce means the '
            'hardware follows the RESULT of the drag, not every frame of it.')

        tk.Label(card_opt, text='Drag axis', font=FONT_LBL, bg=PANEL,
                 fg=TEXT, anchor='w').pack(fill='x', pady=(10, 2))
        self._manip_axis_var = tk.StringVar(value='FREE')
        self._manip_axis_btns: dict[str, tk.Button] = {}
        row_axis = tk.Frame(card_opt, bg=PANEL); row_axis.pack(fill='x')
        for code, label in (('FREE', 'Free'), ('X', 'X'),
                            ('Y', 'Y'), ('Z', 'Z')):
            b = tk.Button(row_axis, text=label, font=FONT_LBL,
                          relief='flat', bd=0, padx=8, pady=5,
                          cursor='hand2', highlightthickness=0,
                          command=lambda c=code: self._on_manip_axis(c))
            b.pack(side='left', fill='x', expand=True, padx=1)
            self._manip_axis_btns[code] = b
        _Tooltip(row_axis,
                 'Free = the TCP follows the mouse on the plane parallel to '
                 'the screen. X/Y/Z = the motion is projected onto that world '
                 'axis, so you can descend in Z without drifting sideways.')

        adv = self._collapsible(card_opt, 'Advanced — IK step')
        self._manip_step_var = tk.DoubleVar(
            value=round(_MANIP_MAX_LIN_M * 1000.0, 1))
        self._param_row(adv, label='Max linear step', unit='mm',
                        var=self._manip_step_var,
                        vmin=1.0, vmax=50.0, step=1.0, snap=0.5,
                        hint='Ceiling on the TCP travel attacked per IK '
                             'iteration (6 per 30 ms tick). Lower = the arm '
                             'trails the cursor more softly; higher = it '
                             'snaps to the mouse but may lurch.')
        self._manip_dq_var = tk.DoubleVar(
            value=round(_math.degrees(_MANIP_MAX_DQ), 1))
        self._param_row(adv, label='Max joint step', unit='°',
                        var=self._manip_dq_var,
                        vmin=0.5, vmax=10.0, step=0.5, snap=0.5,
                        hint='Ceiling on each joint per IK iteration. It is '
                             'what keeps the pose continuous near a '
                             'singularity, where the DLS would otherwise ask '
                             'for a huge wrist swing.')
        self._manip_step_var.trace_add(
            'write', lambda *_: self._manip_apply_options())
        self._manip_dq_var.trace_add(
            'write', lambda *_: self._manip_apply_options())

        # Câmera + ações
        card_act = self._card(col_side, 'View & actions', expand=False)
        row_view = tk.Frame(card_act, bg=PANEL); row_view.pack(fill='x')
        for code, label in (('iso', 'Iso'), ('top', 'Top'),
                            ('front', 'Front'), ('side', 'Side')):
            tk.Button(row_view, text=label, font=FONT_LBL,
                      bg=BTN_NEUTRAL, fg=TEXT,
                      activebackground=_shade(BTN_NEUTRAL, -0.08),
                      activeforeground=TEXT,
                      relief='flat', bd=0, padx=8, pady=5, cursor='hand2',
                      highlightthickness=0,
                      command=lambda c=code: self._manip_set_view(c)
                      ).pack(side='left', fill='x', expand=True, padx=1)

        row_act = tk.Frame(card_act, bg=PANEL); row_act.pack(fill='x',
                                                             pady=(8, 0))
        tk.Button(row_act, text='⟲  Sync from scene',
                  command=self._manip_sync_from_scene,
                  bg=_shade(PRIMARY, 0.25), fg=PRIMARY,
                  activebackground=_shade(PRIMARY, 0.15),
                  activeforeground=PRIMARY,
                  font=FONT_LBL, relief='flat', bd=0, padx=10, pady=6,
                  cursor='hand2'
                  ).pack(side='left', fill='x', expand=True, padx=(0, 4))
        tk.Button(row_act, text='⌖  Capture pose',
                  command=self._manip_capture_pose,
                  bg=_shade(OK, 0.25), fg=OK,
                  activebackground=_shade(OK, 0.15), activeforeground=OK,
                  font=FONT_LBL, relief='flat', bd=0, padx=10, pady=6,
                  cursor='hand2'
                  ).pack(side='left', fill='x', expand=True, padx=(4, 0))

        tk.Button(card_act, text='⌂  Home',
                  command=self._manip_go_home,
                  bg=PRIMARY, fg='white',
                  activebackground=PRIMARY_HV, activeforeground='white',
                  font=FONT_LBL, relief='flat', bd=0, padx=10, pady=7,
                  cursor='hand2').pack(fill='x', pady=(4, 0))

        self._manip_gate_lbl = tk.Label(
            col_side, text='', font=FONT_SMALL, bg=BG, fg=TEXT_DIM,
            anchor='w', justify='left', wraplength=310)
        self._manip_gate_lbl.pack(fill='x', pady=(10, 0))

        self._on_manip_axis('FREE')
        self._manip_apply_options()
        self._manip_sync_from_scene(quiet=True)
    def _manip_chk(self, parent, text: str, var: tk.BooleanVar,
                   command, hint: str = '') -> tk.Checkbutton:
        chk = tk.Checkbutton(parent, text=text, variable=var,
                             command=command, bg=PANEL, fg=TEXT,
                             activebackground=PANEL, activeforeground=TEXT,
                             selectcolor=PANEL, font=FONT_LBL,
                             anchor='w', relief='flat', bd=0,
                             highlightthickness=0, cursor='hand2')
        chk.pack(fill='x', pady=2)
        if hint:
            _Tooltip(chk, hint)
        return chk
    def _on_manip_axis(self, code: str) -> None:
        self._manip_axis_var.set(code)
        for c, btn in self._manip_axis_btns.items():
            on = (c == code)
            btn.config(bg=PRIMARY if on else BTN_NEUTRAL,
                       fg='white' if on else TEXT,
                       activebackground=PRIMARY_HV if on
                       else _shade(BTN_NEUTRAL, -0.08),
                       activeforeground='white' if on else TEXT)
        if self._manip_view is not None:
            self._manip_view.axis_constraint = code
    def _manip_set_view(self, name: str) -> None:
        if self._manip_view is not None:
            self._manip_view.set_view(name)
    def _manip_apply_options(self) -> None:
        """Reflete os widgets do painel no estado da viewport."""
        view = self._manip_view
        if view is None:
            return
        view.lock_orientation = bool(self._manip_lock_var.get())
        self._manip_mirror_on = bool(self._manip_mirror_var.get())
        try:
            view.max_lin_m = max(0.001, float(self._manip_step_var.get())
                                 / 1000.0)
        except (tk.TclError, ValueError):
            pass
        try:
            view.max_dq = max(0.005, _math.radians(
                float(self._manip_dq_var.get())))
        except (tk.TclError, ValueError):
            pass
        self._manip_refresh_gate()
    def _manip_gate_reason(self) -> str:
        """Motivo pelo qual o arrasto está bloqueado ('' = liberado)."""
        with self._lock:
            phase = self._latest_phase
        if phase not in ('IDLE', 'DONE', 'ABORTED'):
            return f'palpation running ({phase}) — drag disabled'
        if self._drag_enabled:
            return 'drag teach active — release it first'
        if self._exec_movement_id is not None:
            return 'motion running — drag disabled'
        return ''
    def _manip_refresh_gate(self) -> None:
        view = self._manip_view
        if view is None:
            return
        reason = self._manip_gate_reason()
        view.enabled = not reason
        view.block_reason = reason
        if reason:
            txt, color = reason, WARN
        elif self._manip_mirror_on and self._robot_mode == 'MIRROR':
            txt = ('Mirroring ON — the real CR10 receives a MovJ when the '
                   'drag settles.')
            color = DANGER
        elif self._manip_mirror_on:
            txt = ('Mirroring is checked but the mode is SIM_ONLY — only '
                   'Gazebo moves.')
            color = TEXT_MUTED
        else:
            txt = 'Simulation only — the real arm is not commanded.'
            color = TEXT_DIM
        # Chamado a cada tick ocioso (33 Hz): só toca no widget quando o
        # texto muda de fato.
        if (txt, color) == getattr(self, '_manip_gate_last', None):
            return
        self._manip_gate_last = (txt, color)
        try:
            self._manip_gate_lbl.config(text=txt, fg=color)
        except (AttributeError, tk.TclError):
            pass
    def _manip_q_provider(self):
        """Pose que a viewport desenha quando ninguém está arrastando: a da
        cena (Gazebo ou, em MIRROR, o feedback do braço real que já alimenta
        /joint_states). Sem ela a viewport ficaria congelada enquanto o braço
        se move por outro caminho (palpação, movimento, drag teach)."""
        # Ponto único onde a aba respira quando ociosa: aproveita para
        # reavaliar o gate (fase da palpação, drag teach, modo do robô).
        self._manip_refresh_gate()
        q = self._latest_joint_rad
        if q is None or len(q) < 6:
            try:
                q = [_math.radians(float(self.arm_sliders[j].get()))
                     for j in ARM_JOINTS]
            except (AttributeError, ValueError, tk.TclError):
                return None
        q = [float(v) for v in q[:6]]
        if self._latest_extra_joints and self._manip_view is not None:
            self._manip_view.set_extra_joints(self._latest_extra_joints)
        # Readout ocioso: só reescreve os labels quando a cena mexeu de
        # verdade (o /joint_states do Gazebo chega a 50 Hz com ruído).
        prev = self._manip_readout_q
        if prev is None or any(abs(a - b) > 1e-4 for a, b in zip(q, prev)):
            self._manip_readout_q = q
            self._manip_update_readout(q)
        return q
    def _manip_on_q(self, q_rad) -> None:
        """Cada iteração do arrasto: publica no JTC e sincroniza os sliders."""
        reason = self._manip_gate_reason()
        if reason:
            # O gate pode FECHAR no meio do gesto (a palpação começou, o
            # drag teach foi detectado).
            self._manip_refresh_gate()
            if self._manip_view is not None:
                self._manip_view.abort_drag()
            self._set_status(f'3D drag interrupted — {reason}.', WARN)
            return
        self._publish_arm_q(q_rad, MANIP_TRAJ_DURATION_S)
        self._suppressing = True
        try:
            for i, j in enumerate(ARM_JOINTS):
                lo, hi = ARM_LIMITS_DEG[j]
                deg = _math.degrees(float(q_rad[i]))
                self.arm_sliders[j].set(round(max(lo, min(hi, deg)), 2))
        except (AttributeError, tk.TclError):
            pass
        finally:
            self._suppressing = False
    def _manip_on_state(self, q_rad, res) -> None:
        """Atualiza os números do painel a cada passo de IK."""
        self._manip_update_readout(q_rad, res)
    def _manip_update_readout(self, q_rad, res=None) -> None:
        T_end = self._manip_T_end()
        if _fk_tcp is None or _rpy_deg is None or T_end is None:
            return
        try:
            T = _fk_tcp(np.asarray(q_rad, dtype=float), T_end=T_end)
        except Exception:
            return
        p = T[:3, 3] * 1000.0
        roll, pitch, yaw = _rpy_deg(T[:3, :3])
        try:
            self._manip_x_lbl.config(text=f'{p[0]:+8.1f} mm')
            self._manip_y_lbl.config(text=f'{p[1]:+8.1f} mm')
            self._manip_z_lbl.config(text=f'{p[2]:+8.1f} mm')
            self._manip_roll_lbl.config(text=f'{roll:+7.1f} °')
            self._manip_pitch_lbl.config(text=f'{pitch:+7.1f} °')
            self._manip_yaw_lbl.config(text=f'{yaw:+7.1f} °')
            if res is None:
                self._manip_err_lbl.config(text='—', fg=TEXT)
                self._manip_manip_lbl.config(text='—')
                return
            lag_mm = res.pos_err_m * 1000.0
            if res.singular:
                lag_txt, lag_color = 'singular', DANGER
            elif res.at_limit:
                lag_txt, lag_color = f'{lag_mm:.1f} mm · limit', WARN
            else:
                lag_txt = f'{lag_mm:.1f} mm'
                lag_color = OK if lag_mm < 5.0 else WARN
            self._manip_err_lbl.config(text=lag_txt, fg=lag_color)
            self._manip_manip_lbl.config(text=f'{res.manip:.4f}')
        except (AttributeError, tk.TclError):
            pass
    def _manip_on_drag_change(self, active: bool) -> None:
        """Arma/desarma o gate do espelhamento no braço real."""
        if self._manip_release_after is not None:
            try:
                self.root.after_cancel(self._manip_release_after)
            except (tk.TclError, ValueError):
                pass
            self._manip_release_after = None
        if active:
            self._manip_active = True
            self._manip_refresh_gate()
            return
        self._manip_release_after = self.root.after(
            200, self._manip_clear_active)
    def _manip_clear_active(self) -> None:
        self._manip_release_after = None
        self._manip_active = False
    def _manip_sync_from_scene(self, quiet: bool = False) -> None:
        """Puxa a pose corrente da cena para a viewport (e para o readout)."""
        view = self._manip_view
        if view is None:
            return
        q = self._manip_q_provider()
        if q is None:
            if not quiet:
                self._set_status('No joint state available yet.', WARN)
            return
        view.set_q(q, force=True)
        self._manip_update_readout(q)
        self._manip_refresh_gate()
        if not quiet:
            self._set_status('3D view synced with the scene.', OK)
    def _manip_capture_pose(self) -> None:
        """Salva a pose atual da viewport na aba Poses & Motions."""
        view = self._manip_view
        if view is None:
            return
        q_deg = [_math.degrees(float(v)) for v in view.q]
        self._add_pose(q_deg, prefix='3D')
    def _manip_go_home(self) -> None:
        """Leva o braço à Home da GUI (mesma do Controle Manual)."""
        if self._manip_gate_reason():
            self._manip_refresh_gate()
            return
        self._apply_arm_home()
        q = [_math.radians(float(self._arm_home_deg[j])) for j in ARM_JOINTS]
        if self._manip_view is not None:
            self._manip_view.set_q(q, force=True)
        self._manip_update_readout(q)
    def _manip_load_scene(self) -> None:
        """Dispara a carga das malhas na PRIMEIRA vez que a aba é aberta."""
        if self._manip_scene_state != 'idle' or not _URDF_SCENE_OK:
            return
        self._manip_scene_state = 'loading'
        if self._manip_view is not None:
            self._manip_view.set_scene(None, 'loading robot meshes…')
        threading.Thread(target=self._manip_scene_worker,
                         daemon=True, name='manip3d-scene').start()
    def _manip_scene_worker(self) -> None:
        scene, error = None, ''
        self._manip_scene_coarse = None
        # Com VTK disponível a malha vai INTEIRA para a GPU — é a mesma
        # geometria que o Gazebo carrega, triângulo por triângulo.
        self._manip_exact = _MANIP3D_OK and _manip_vtk_available()
        try:
            scene = _build_scene(
                self._end_effector,
                description_path=self._robot_desc_path,
                triangle_budget=None if self._manip_exact else _SCENE_BUDGET)
            if not self._manip_exact:
                # Segunda malha, mais grossa, para usar durante o arrasto —
                # gerada aqui, na thread, junto com a principal.
                self._manip_scene_coarse = _coarse_scene(scene)
        except Exception as exc:
            error = str(exc)
            self.get_logger().warning(
                f'Viewport 3D: malhas indisponíveis ({error}) — '
                f'usando esqueleto.')
        # A carga da mão leva ~4 s; a janela pode ter sido fechada no meio.
        if self._stop_event.is_set():
            return
        try:
            self.root.after(0, self._manip_scene_done, scene, error)
        except (tk.TclError, RuntimeError) as exc:
            # Nunca engolir em silêncio: se o agendamento falha, a aba fica
            # eternamente em "loading" e ninguém sabe por quê.
            self._manip_scene_state = 'failed'
            self.get_logger().warning(
                f'Viewport 3D: entrega da cena falhou ({exc}).')
    def _manip_scene_done(self, scene, error: str) -> None:
        view = self._manip_view
        if scene is None:
            self._manip_scene_state = 'failed'
            if view is not None:
                view.set_scene(None, 'skeleton view — meshes unavailable')
            return
        self._manip_scene_state = 'ready'
        if view is not None:
            missing = len(getattr(scene, 'missing_meshes', ()))
            status = f'{missing} mesh(es) missing' if missing else ''
            view.set_scene(scene, status, coarse=self._manip_scene_coarse,
                           exact=self._manip_exact)
            if self._manip_exact and not view.rendering_exact:
                # A GPU recusou o contexto depois de a cena exata já estar
                # pronta.
                self._manip_exact = False
                self._manip_scene_state = 'idle'
                self.get_logger().warning(
                    'Viewport 3D: GPU indisponível — recarregando malha '
                    'reduzida para o rasterizador em software.')
                self._manip_load_scene()
                return
        self.get_logger().info(
            f'Viewport 3D: {len(scene.parts)} peças / '
            f'{scene.triangle_count} triângulos ({self._end_effector}, '
            f'{"malha exata / GPU" if self._manip_exact else "reduzida / CPU"}).')


def main(args=None):
    import faulthandler, sys
    faulthandler.enable(file=sys.stderr)
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s.%(msecs)03d [%(name)s] %(levelname)s  %(message)s',
        datefmt='%H:%M:%S')
    rclpy.init(args=args)
    gui = PalpationGUI()

    def _sighandler(sig, frame):
        # Fechar a janela Tkinter de forma limpa (roda _on_close via protocol).
        try:
            gui.root.after(0, gui._on_close)
        except Exception:
            pass

    signal.signal(signal.SIGTERM, _sighandler)
    signal.signal(signal.SIGINT, _sighandler)

    try:
        gui.root.mainloop()
    finally:
        gui._stop_event.set()
        try:
            gui.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
