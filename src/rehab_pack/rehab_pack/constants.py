"""Contrato do rehab_pack: nomes, campos e limites compartilhados pelos nós.

Dono: WS1. Os outros só importam. Precisa de algo novo aqui? Peça no relatório.
"""
from __future__ import annotations

import json
import math
import os

# ── Segurança e temporização ────────────────────────────────────────────────
FORCE_ABORT_LIMIT_N = 15.0      # mesmo limite do touch_pack
DT_S = 0.030                    # tick de controle (33 Hz)
FORCE_STALE_S = 0.5
# Torque sobre o centro da empunhadura. Nenhuma norma dá esse valor: 2 N·m fica entre o
# que um robô de reabilitação aplica em prono-supinação (1 N·m) e a força máxima média
# de adultos saudáveis (3,3 N·m, Min et al. 2025). PROVISÓRIO: confirme com a clínica.
TORQUE_ABORT_LIMIT_NM = 2.0
TARE_MAX_TILT_RAD = 0.0873      # 5°: girou mais que isso desde a tara, tare de novo
SIM_LAG_MAX_RAD = 0.15          # simulado longe do comandado: outro nó move o braço
TILT_MAX_RAD = math.radians(2.0)   # mastro girado além disso da orientação de partida: LIMITE
JS_STALE_S = 0.1                # /joint_states mais velho que isso em MIRROR: Gazebo travou
JS_STALE_SIM_S = 0.5            # idem só no simulado (sem robô real seguindo)
TABLE_HALF_XY = (0.30, 0.55)    # meia mesa (frame world), de rehab_lab.world
LINK_CLEARANCE_M = 0.10         # raio dos elos + folga
ENABLE_MAX_N = 3.0              # acima disto não habilita: mão no mastro ou tara ruim
TICK_STALE_S = 0.2              # sem tick da admitância o espelho não manda ServoJ
RAMP_S = 1.5                    # tempo para absorver a diferença sim−real ao armar
LOOP_DELAY_S = 0.12             # força → robô real. ESTIMADO, não medido: meça na bancada
ARM_STIFFNESS_MIN_N_M = 100.0   # M/B que não aguentam este braço são recusados
ARM_STIFFNESS_REF_N_M = 300.0   # abaixo disto a GUI avisa "perto do limite"

# ── Braço ───────────────────────────────────────────────────────────────────
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
# PROVISÓRIO (FK à mão a partir do xacro): tcp ≈ (0,55; 0; 0,25) m no frame
# `world`, empunhadura para cima. O WS1 confirma com tf2 e corrige.
HOME_Q_RAD = [-0.3547, -0.9220, -1.9393, -1.8510, 1.5708, 1.2161]
TABLE_Z_WORLD = 0.0             # `world` é a raiz do URDF: topo da mesa em z=0
MIN_TCP_Z = 0.10                # piso do setpoint, m, no frame world

# ── Robô real (modo mirror, como no touch_pack) ─────────────────────────────
# Só se arma o ServoJ com sim e real a menos de SYNC_TOL_START; durante o
# movimento o real fica atrás do sim (JTC + ServoJ), então a tolerância de
# seguimento é maior. Acima dela: aborta.
SYNC_TOL_START_RAD = 0.0524     # 3°
SYNC_TOL_RUN_RAD = 0.1396       # 8°
GUI_STALE_S = 1.0               # painel calado por mais que isso, com o braço solto: aborta
REAL_STATUS_STALE_S = 1.0       # status mais velho que isso = rehab_mirror fora do ar
SERVOJ_PERIOD_RANGE_S = (0.02, 0.1)   # ServoJ: o mínimo do firmware é 20 ms
SERVO_STEP_SLACK = 2.0            # passo máximo do ServoJ = JOG_MAX_DEG_S × (período × folga + atraso do laço)
FEEDBACK_TIMEOUT_MAX = 5          # leituras perdidas SEGUIDAS até dar a conexão por perdida
ROBOT_MODES_OK = (5, 7, 8)        # RobotMode V4: habilitado, rodando, movimento único
SIM_ROBOT_IP = 'sim'            # robot_ip:='sim' → robô real simulado (ensaio sem hardware)

# ── Conexão com o robô real pela tela (como o cabeçalho do touch_pack) ──────
NODE_MIRROR = 'rehab_mirror'
ARM_PUBS = {'rehab_admittance', NODE_MIRROR}   # únicos nós que podem publicar em T_ARM_CMD
REAL_MODES = ('SIM_ONLY', 'MIRROR')            # o mesmo par do seletor do touch_pack
S_REAL_CONNECT = '/rehab/real/connect'         # SetBool: True conecta ao robot_ip atual, False desconecta
S_REAL_MODE = '/rehab/real/mode'               # SetBool: True = MIRROR, False = SIM_ONLY
# /rehab/real/status (JSON) ganha: modo ('SIM_ONLY'|'MIRROR'), conectando (bool), erro (str).
# O IP é o parâmetro `robot_ip` do nó rehab_mirror (set_parameters); aplica no próximo connect.

# ── Posicionamento por juntas (aba 5 do painel) ─────────────────────────────
# Limites de SOFTWARE, mais estreitos que os do URDF (±359°): os mesmos do
# touch_pack (J2/J4 ±170°, J3 ±160° = limite de hardware do CR10).
JOINT_LIMITS_DEG = ((-180.0, 180.0), (-170.0, 170.0), (-160.0, 160.0),
                    (-170.0, 170.0), (-170.0, 170.0), (-180.0, 180.0))
JOG_MAX_DEG_S = 24.0            # pico de velocidade articular a 100 % (e teto na admitância)
JOG_SPEED_DEFAULT_PCT = 50.0
JOG_SPEED_MIN_PCT = 5.0
JOG_MIN_S = 0.5                 # duração mínima de um movimento
JOG_MARGIN_S = 0.5              # folga após o fim da trajetória antes de voltar a PARADO
JOG_MIN_DELTA_DEG = 0.5         # abaixo disto: "já está nessa posição"
MIN_CELL_Z = 0.05               # piso da célula de carga no mundo, m

# ── Mastro (conferido contra urdf/joystick_tool.urdf nos testes) ────────────
# Mastro PESADO em 24/11/2026: 285,9 g (placa + empunhadura, sem célula nem flange); placa = 9,4 g calculada.
GRIP_MASS_KG = 0.2765
PLATE_MASS_KG = 0.0094
TCP_OFFSET_M = 0.1067           # flange → centro da empunhadura

# ── Tópicos ─────────────────────────────────────────────────────────────────
T_SIM_WRENCH = '/sim/load_cell/wrench'
T_REAL_WRENCH = '/ft_sensor/wrench'
T_REAL_TARED = '/load_cell/tared'   # Bool do ft_receiver (1 Hz e ao zerar)
T_REAL_TARE_RESULT = '/load_cell/tare_result'   # ft_receiver: 'ok;...' = o zero dele mudou
T_FT = '/rehab/ft/wrench'
T_TARED = '/rehab/ft/tared'   # Float64 latched: time.monotonic() da tara; 0 = sem tara
T_FT_CONFIG = '/rehab/ft/config'   # String JSON latched {ft_sign, ft_yaw_deg}: o que o force_source usa
T_REFERENCE = '/rehab/admittance/reference'
T_TICK = '/rehab/admittance/tick'
T_TCP_POSE = '/rehab/tcp/pose'
T_STATE = '/rehab/admittance/state'
T_PARAMS = '/rehab/admittance/params'
T_ARM_CMD = '/cr10_group_controller/joint_trajectory'
T_JOINT_STATES = '/joint_states'
T_TARGET = '/rehab/task/target'
T_EVENT = '/rehab/event'
T_GUI_ALIVE = '/rehab/gui/alive'     # Empty a cada volta do laço do Tk (~30 Hz)
T_SESSION_META = '/rehab/session/meta'
T_SESSION_SUMMARY = '/rehab/session/summary'
T_SESSION_REC = '/rehab/session/recording'   # String a 2 Hz: pasta em gravação ('' = nenhuma)
T_EMG = '/rehab/emg'
T_REAL_STATUS = '/rehab/real/status'   # JSON do rehab_mirror (até 33 Hz)
T_JOG_GOTO = '/rehab/jog/goto'         # Float64MultiArray: 6 juntas em rad (URDF) + velocidade em %
T_JOG_RESULT = '/rehab/jog/result'     # String JSON {ok, motivo, duracao_s, fim?}

# ── Serviços ────────────────────────────────────────────────────────────────
S_TARE = '/rehab/ft/tare'
S_ENABLE = '/rehab/admittance/enable'
S_ESTOP = '/rehab/estop'
S_RESET = '/rehab/admittance/reset'
S_HOME = '/rehab/home'
S_SESSION_START = '/rehab/session/start'
S_SESSION_STOP = '/rehab/session/stop'

# ── Estado da admitância (o código numérico é o índice) ────────────────────
# POSICIONANDO entra no FIM: os índices dos demais não mudam.
STATES = ('PARADO', 'ATIVO', 'SEM_FORCA', 'LIMITE', 'ABORTADO', 'POSICIONANDO')

# ── Layout de /rehab/admittance/tick (Float64MultiArray, 33 Hz, sempre) ────
TICK_FIELDS = (
    't_s', 'fx_n', 'fy_n', 'fz_n', 'f_axial_n',
    'x_m', 'y_m', 'z_m',
    'sx_m', 'sy_m', 'sz_m',
    'vx_ms', 'vy_ms', 'vz_ms',
    'rx_m', 'ry_m', 'rz_m',
    'state', 'force_age_s', 'adm_scale',
    'xr_m', 'yr_m', 'zr_m',        # TCP do robô real; NaN fora de MIRROR
)

EVENT_TYPES = ('TASK_START', 'TARGET_ON', 'TARGET_HIT', 'TARGET_MISS',
               'TUNNEL_EXIT', 'TUNNEL_ENTER', 'TASK_END', 'MARK', 'ESTOP',
               'ABORT', 'RESET', 'PARAMS', 'ROBO')

# ── Colunas dos CSV de sessão ───────────────────────────────────────────────
SAMPLES_COLUMNS = (TICK_FIELDS
                   + ('target_x_m', 'target_y_m', 'target_z_m')
                   + tuple(f'q{i}_rad' for i in range(1, 7))
                   + tuple(f'q{i}_real_rad' for i in range(1, 7)))
WRENCH_COLUMNS = ('t_s', 'fx_n', 'fy_n', 'fz_n', 'tx_nm', 'ty_nm', 'tz_nm',
                  'frame')
EVENTS_COLUMNS = ('t_s', 'tipo', 'detalhe')


HOME_FILE = os.path.expanduser('~/.config/rehab_pack/home_pose.json')


def home_q() -> list:
    """HOME salva pelo operador (rad); HOME_Q_RAD se não houver ou for inválida."""
    try:
        with open(HOME_FILE) as fh:
            d = json.load(fh)
        q = [float(d[j]) for j in ARM_JOINTS]
    except (OSError, ValueError, KeyError, TypeError):
        return list(HOME_Q_RAD)
    ok = all(lo <= v <= hi for v, (lo, hi) in zip(q, JOINT_LIMITS_DEG))   # NaN reprova
    return [math.radians(v) for v in q] if ok else list(HOME_Q_RAD)


def save_home(q_rad) -> None:
    """Grava em graus, como o home_pose.json do touch_pack (outra ferramenta, outro arquivo)."""
    os.makedirs(os.path.dirname(HOME_FILE), exist_ok=True)
    with open(HOME_FILE, 'w') as fh:
        json.dump({j: round(math.degrees(v), 4) for j, v in zip(ARM_JOINTS, q_rad)},
                  fh, indent=2)


def runs_dir() -> str:
    """Pasta das sessões: $REHAB_PACK_DATA_DIR, senão $TOUCH_PACK_DATA_DIR/REHAB,
    senão <repo>/sensors/Data/REHAB
    (repo = pai de `sensors/`, subindo diretórios), senão ~/rehab_pack_runs."""
    env = os.environ.get('REHAB_PACK_DATA_DIR')
    if env:
        return os.path.expanduser(env)
    env = os.environ.get('TOUCH_PACK_DATA_DIR')     # a pasta padrão de coletas, se mudada
    if env:
        return os.path.join(os.path.abspath(os.path.expanduser(env)), 'REHAB')
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isdir(os.path.join(d, 'sensors')):
            return os.path.join(d, 'sensors', 'Data', 'REHAB')
        parent = os.path.dirname(d)
        if parent == d:
            return os.path.expanduser('~/rehab_pack_runs')
        d = parent
