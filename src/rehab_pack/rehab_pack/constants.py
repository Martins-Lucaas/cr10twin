"""Contrato do rehab_pack: nomes, campos e limites compartilhados pelos nós.

Dono: WS1. Os outros só importam. Precisa de algo novo aqui? Peça no relatório.
"""
from __future__ import annotations

import os

# ── Segurança e temporização ────────────────────────────────────────────────
FORCE_ABORT_LIMIT_N = 15.0      # mesmo limite do touch_pack
DT_S = 0.030                    # tick de controle (33 Hz)
FORCE_STALE_S = 0.5

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
REAL_STATUS_STALE_S = 1.0       # status mais velho que isso = rehab_mirror fora do ar
SIM_ROBOT_IP = 'sim'            # robot_ip:='sim' → robô real simulado (ensaio sem hardware)

# ── Conexão com o robô real pela tela (como o cabeçalho do touch_pack) ──────
NODE_MIRROR = 'rehab_mirror'
REAL_MODES = ('SIM_ONLY', 'MIRROR')            # o mesmo par do seletor do touch_pack
S_REAL_CONNECT = '/rehab/real/connect'         # SetBool: True conecta ao robot_ip atual, False desconecta
S_REAL_MODE = '/rehab/real/mode'               # SetBool: True = MIRROR, False = SIM_ONLY
# /rehab/real/status (JSON) ganha: modo ('SIM_ONLY'|'MIRROR'), conectando (bool), erro (str).
# O IP é o parâmetro `robot_ip` do nó rehab_mirror (set_parameters); aplica no próximo connect.
# T_JOG_GOTO aceita 6 juntas (rad) ou 7 valores: o 7º é a velocidade em % (JOG_SPEED_*).
JOG_SPEED_DEFAULT_PCT = 50.0
JOG_SPEED_MIN_PCT = 5.0
JOG_SPEED_MAX_PCT = 100.0

# ── Posicionamento por juntas (aba 5 do painel) ─────────────────────────────
# Limites de SOFTWARE, mais estreitos que os do URDF (±359°): os mesmos do
# touch_pack (J2/J4 ±170°, J3 ±160° = limite de hardware do CR10).
JOINT_LIMITS_DEG = ((-180.0, 180.0), (-170.0, 170.0), (-160.0, 160.0),
                    (-170.0, 170.0), (-170.0, 170.0), (-180.0, 180.0))
JOG_MAX_DEG_S = 12.0            # velocidade articular máxima do posicionamento
JOG_MIN_S = 1.5                 # duração mínima de um movimento
JOG_MARGIN_S = 0.5              # folga após o fim da trajetória antes de voltar a PARADO
JOG_MIN_DELTA_DEG = 0.5         # abaixo disto: "já está nessa posição"
MIN_CELL_Z = 0.05               # piso da célula de carga no mundo, m

# ── Mastro (conferido contra urdf/joystick_tool.urdf nos testes) ────────────
GRIP_MASS_KG = 0.0694
PLATE_MASS_KG = 0.0094
TCP_OFFSET_M = 0.1142           # flange → centro da empunhadura

# ── Tópicos ─────────────────────────────────────────────────────────────────
T_VIRTUAL_WRENCH = '/rehab/virtual_wrench'
T_SIM_WRENCH = '/sim/load_cell/wrench'
T_REAL_WRENCH = '/ft_sensor/wrench'
T_FT = '/rehab/ft/wrench'
T_REFERENCE = '/rehab/admittance/reference'
T_TICK = '/rehab/admittance/tick'
T_TCP_POSE = '/rehab/tcp/pose'
T_STATE = '/rehab/admittance/state'
T_PARAMS = '/rehab/admittance/params'
T_ARM_CMD = '/cr10_group_controller/joint_trajectory'
T_JOINT_STATES = '/joint_states'
T_TARGET = '/rehab/task/target'
T_EVENT = '/rehab/event'
T_SESSION_META = '/rehab/session/meta'
T_SESSION_SUMMARY = '/rehab/session/summary'
T_EMG = '/rehab/emg'
T_REAL_STATUS = '/rehab/real/status'   # JSON do rehab_mirror (10 Hz)
T_JOG_GOTO = '/rehab/jog/goto'         # Float64MultiArray: 6 juntas em rad (convenção URDF)
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
    'state', 'force_age_s',
)

EVENT_TYPES = ('TASK_START', 'TARGET_ON', 'TARGET_HIT', 'TARGET_MISS',
               'TUNNEL_EXIT', 'TUNNEL_ENTER', 'TASK_END', 'MARK', 'ESTOP',
               'ABORT', 'PARAMS')

# ── Colunas dos CSV de sessão ───────────────────────────────────────────────
SAMPLES_COLUMNS = (TICK_FIELDS
                   + ('target_x_m', 'target_y_m', 'target_z_m')
                   + tuple(f'q{i}_rad' for i in range(1, 7)))
WRENCH_COLUMNS = ('t_s', 'fx_n', 'fy_n', 'fz_n', 'tx_nm', 'ty_nm', 'tz_nm',
                  'frame')
EVENTS_COLUMNS = ('t_s', 'tipo', 'detalhe')


def runs_dir() -> str:
    """Pasta das sessões: $REHAB_PACK_DATA_DIR, senão <repo>/sensors/Data/REHAB
    (repo = pai de `sensors/`, subindo diretórios), senão ~/rehab_pack_runs."""
    env = os.environ.get('REHAB_PACK_DATA_DIR')
    if env:
        return os.path.expanduser(env)
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isdir(os.path.join(d, 'sensors')):
            return os.path.join(d, 'sensors', 'Data', 'REHAB')
        parent = os.path.dirname(d)
        if parent == d:
            return os.path.expanduser('~/rehab_pack_runs')
        d = parent
