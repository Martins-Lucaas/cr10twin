"""
constants.py — Constantes compartilhadas do touch_pack.

Regra: valores usados por MAIS de um módulo moram aqui; valores privados
de um único módulo ficam nele.
"""
from __future__ import annotations

import math
import os
from typing import NamedTuple

# Cadeia do braço CR10 (convenção URDF).
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# ── ServoJ: números do BRAÇO, não de cada nó ──────────────────────────
# Banda morta do espelhamento: abaixo dela o alvo conta como o mesmo ponto e
# o ServoJ não é reenviado. Era 1e-4 rad — que num CR10 de ~1,2 m de alcance
# vale ~120 µm de TCP, da ORDEM da onda trigonométrica inteira: uma amplitude
# de 700 µm sairia quantizada em ~6 degraus e qualquer onda menor seria
# suprimida em silêncio. 1e-5 rad ≈ 12 µm de TCP, ainda acima do ruído de
# encoder (~1,7e-5 rad por LSB de 0,001°) e fino o bastante para a onda.
#
# Mora AQUI porque três módulos precisam do mesmo número por motivos
# diferentes: o mirror_node e a palpation_gui para decidir o reenvio (os dois
# comandam o MESMO braço, e uma banda maior num deles engoliria a onda que o
# outro deixa passar), e o tactile_explorer para avisar quando a onda pedida é
# tão pequena que sai QUANTIZADA por ela.
SERVOJ_DEADBAND_RAD = 1.0e-5
# Alcance usado para converter a banda morta acima em micrômetros de TCP. É
# ordem de grandeza para mensagem de aviso, não cinemática — quem precisa do
# número exato chama o Jacobiano.
ARM_REACH_M = 1.2

# Pose "apontar para a mesa": home default da GUI e seed POINTING do explorer.
POINTING_SEED_DEG = {'joint1': 0.0, 'joint2': 0.0, 'joint3': -90.0,
                     'joint4': 0.0, 'joint5': 90.0, 'joint6': 0.0}

# Mão COVVI — juntas primárias.
HAND_JOINTS = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']

# Pose POINTING (palpação com o Index estendido).
HAND_POINT_DEG = {'Thumb': 30.0, 'Index': 0.0, 'Middle': 80.0,
                  'Ring': 80.0, 'Little': 80.0, 'Rotate': 0.0}
HAND_POINTING_RAD = {j: math.radians(v) for j, v in HAND_POINT_DEG.items()}

# Curso angular de cada junta primária da mão, em graus (fim de curso do URDF).
HAND_SPAN_DEG = {'Thumb': 90.0, 'Index': 90.0, 'Middle': 90.0,
                 'Ring': 90.0, 'Little': 90.0, 'Rotate': 60.0}

# Escala ECI real dos dígitos (calibrada na mão física em 06/07/2026): a
# telemetria DigitPosnAll NÃO vai de 0 a 200 — o fim de curso mecânico
# aberto lê ~47 (rotate ~67) e o fechado ~198 (rotate ~197).
ECI_POSN_OPEN = {'Thumb': 47, 'Index': 47, 'Middle': 47,
                 'Ring':  47, 'Little': 47, 'Rotate': 67}
ECI_POSN_CLOSED = {'Thumb': 198, 'Index': 198, 'Middle': 198,
                   'Ring':  198, 'Little': 198, 'Rotate': 197}


# Faixa da junta DRIVER da mão no URDF, em radianos. Não é o ângulo da ponta
# do dedo: 1,0 rad de driver produz ~163° na ponta do Index, pela cadeia de
# juntas mimic. Autoridade única — `hand_pack.urdf_helpers` importa daqui para
# aplicar o clamp no URDF, e `hand_deg_to_driver_rad` abaixo mapeia para cá.
# Elas TÊM de ser o mesmo número: quando divergiram (a conversão mirava 0–90°
# contra um teto de 57,3°), a mão simulada ceifou 30% da excursão dos dedos e
# a campanha de latência de 10/09/2026 mediu ganho 0,77 em vez de 1,0.
HAND_DRIVER_UPPER_RAD = {'Thumb': 1.00, 'Index': 1.00, 'Middle': 1.00,
                         'Ring':  1.00, 'Little': 1.00, 'Rotate': 1.00}
# `lower` calibrado — equivalente ao `open_limit` do DigitConfigMsg da mão real.
HAND_DRIVER_LOWER_RAD = {'Thumb': 0.08, 'Index': 0.12, 'Middle': 0.12,
                         'Ring':  0.12, 'Little': 0.12, 'Rotate': 0.00}


def hand_deg_to_driver_rad(joint: str, deg: float) -> float:
    """Pose em graus da mão REAL (0–90° dedos, 0–60° Rotate) → radiano da
    junta driver do URDF.

    É a fronteira entre os dois espaços angulares do sistema: o slider da GUI
    e a mão física falam em graus de ponta de dedo; o Gazebo move o driver.
    Mandar grau de dedo direto para o driver satura no teto de 1,0 rad — foi
    o bug de 10/09/2026. Ver HAND_DRIVER_UPPER_RAD acima.
    """
    span = HAND_SPAN_DEG[joint]
    lo, hi = HAND_DRIVER_LOWER_RAD[joint], HAND_DRIVER_UPPER_RAD[joint]
    frac = min(max(float(deg) / span, 0.0), 1.0)
    return lo + frac * (hi - lo)


def eci_posn_to_deg(joint: str, pos: float) -> float:
    """Contagem da telemetria DigitPosnAll → grau da junta primária.

    Fica aqui, e não na GUI, porque o mirror real→sim e o hand_latency_probe
    precisam da MESMA conversão: se as duas divergirem, a latência medida
    passa a incluir uma diferença de escala que não existe no sistema.
    """
    span = HAND_SPAN_DEG[joint]
    lo, hi = ECI_POSN_OPEN[joint], ECI_POSN_CLOSED[joint]
    frac = (float(pos) - lo) / float(hi - lo)
    return max(0.0, min(span, frac * span))

# Limite de segurança: medição CANCELADA se a compressão exceder este valor.
FORCE_ABORT_LIMIT_N = 15.0
# Setpoint máximo selecionável na GUI.
FORCE_SETPOINT_MAX_N = 10.0
CONTACT_ON_N = 0.12
# Histerese do INDICADOR — só da tela, o controle não a usa. Acende em
# CONTACT_ON_N e só apaga abaixo desta fração: com o indicador exatamente igual
# ao gatilho, o verde pisca em ar livre toda vez que o ruído cruza o limiar.
CONTACT_OFF_FRAC = 0.7

# σ em repouso da FA7155, eixo de controle (Fz), medido em 21/10/2026: 30 s
# parada e descarregada, polled a 1 Mbps, 11 731 amostras a 391 Hz.
# CRU (o que a onda e o lock-in leem): 22,4 mN.
FORCE_NOISE_SIGMA_N = 0.022
# Depois do filtro mediana+One-Euro do ft_receiver (o que a MALHA vê): 2,1 mN.
FORCE_CTRL_SIGMA_N = 0.0021


HOLD_TOL_N     = 0.02
HOLD_TOL_SIGMA = HOLD_TOL_N / FORCE_CTRL_SIGMA_N   # ≈ 9,5σ
# Fração do setpoint. Era 5 %, o que fazia a banda valer 0,10 N num alvo de
# 2 N — o termo do ruído nem chegava a mandar na faixa usada na bancada.
# A 1 % ela fica nos 0,02 N pedidos em TODO o intervalo de 0 a 2 N, e só
# volta a abrir acima disso, onde 1 % do alvo já é maior que o piso.
HOLD_TOL_PCT   = 0.01


def hold_tol_n(target_f: float) -> float:
    """Meia-banda do HOLD para um setpoint: o maior entre o piso de ruído
    (4σ) e a fração do alvo. É a LEI do explorer, exposta aqui para a GUI
    poder mostrar (e mandar) o mesmo número em vez de um default próprio."""
    return max(HOLD_TOL_N, HOLD_TOL_PCT * abs(float(target_f)))


FT_SERIAL_BAUD  = 1_000_000   # exemplar da bancada (manual §4.3 dá 115200 como
                              # default do caso geral)
FT_FRAME_HEADER = b'\x53\x54'
FT_FRAME_LEN    = 28          # 2 (cabeçalho) + 6×float32 + 2 (CRC-16/MODBUS)
# Ordem dos seis canais dentro do quadro — é ela que dá nome às colunas.
FT_AXES = ('fx', 'fy', 'fz', 'mx', 'my', 'mz')

# Taxa ENTREGUE ao host. O datasheet dá 250 Hz de fábrica e 500-1000 Hz
# configuráveis (a unidade da bancada está em FT_SENSOR_RATE_HZ), mas o
# exemplar só responde a poll, e cada transação paga o round-trip USB do CH343.
# Medido: 397-399 Hz nos 12 runs de sensors/Data/{glass,silicon} (09/2026) e
# 391 Hz ao vivo em 21/10/2026. É também a taxa assumida pelo filtro, pelo
# explorer e pelo atraso do ILC até chegar a primeira medida (/load_cell/rate_hz).
FT_NOMINAL_RATE_HZ = 400.0
# Taxa que o SENSOR produz, que é outra coisa: é o que está gravado na unidade
# (Send_Frequency) e o que a GUI pré-seleciona no combo "Set rate (Hz)" — esse
# botão escreve no DISPOSITIVO, então ele tem de oferecer um valor que o
# dispositivo aceita (FT_RATE_CHOICES_HZ), não a taxa que o USB deixa passar.
FT_SENSOR_RATE_HZ = 1000.0
# Teto ABSOLUTO do link: 28 bytes × 10 bits / 1 Mbps ≈ 0,28 ms por quadro.
# Um sensor acima disto NÃO cabe no baud em uso e vai chegar picotado.


def ft_max_rate_hz(baud: float = FT_SERIAL_BAUD) -> float:
    """Teto de quadros/s que CABE num dado baud (28 B × 10 bits por quadro).

    É função e não só constante porque o baud é PARÂMETRO do nó (`ft_baud`):
    o aviso do ft_receiver imprimia o baud pedido ao lado de um teto sempre
    calculado sobre o FT_SERIAL_BAUD do módulo, e com um `ft_baud` diferente
    (`ft_baud:=115200`, por exemplo) o número que ele mandava conferir estava
    errado na mesma proporção.
    """
    return float(baud) / (FT_FRAME_LEN * 10)


FT_MAX_RATE_HZ = ft_max_rate_hz()   # ≈ 3571 Hz a 1 Mbps
# Abaixo disto o receiver avisa: cabo ruim, baud errado ou taxa de fábrica
# diferente da configurada.
FT_MIN_RATE_HZ = 100.0

FT_RATED_FORCE_N   = 400.0
FT_RATED_TORQUE_NM = 20.0


FT_SAFE_OVERLOAD_PCT = 300.0

# Rótulo e unidade de cada eixo, na ordem do quadro (FT_AXES).
FT_AXIS_LABELS = (
    ('fx', 'Fx', 'N'),
    ('fy', 'Fy', 'N'),
    ('fz', 'Fz', 'N'),
    ('mx', 'Mx', 'N·m'),
    ('my', 'My', 'N·m'),
    ('mz', 'Mz', 'N·m'),
)


def ft_axis_rated(axis: str) -> float:
    """Fundo de escala do eixo: força para fx/fy/fz, torque para mx/my/mz."""
    return FT_RATED_FORCE_N if axis in ('fx', 'fy', 'fz') else FT_RATED_TORQUE_NM
# VIDs dos chips de ponte USB-serial usados em conversores RS485. O FA7155 não
# aparece na USB — quem aparece é o conversor. Também EXCLUÍDOS do auto-detect
# do touch sensor (ver touch_source.detect_serial_port).
FT_USB_VIDS = (
    0x1A86,   # WCH — CH340/CH341/CH343 (o ZK-U485 azul da bancada)
    0x10C4,   # Silicon Labs — CP2102/CP2104
    0x0403,   # FTDI — FT232
    0x067B,   # Prolific — PL2303
)


FT_TCP_HOST = '192.168.5.2'   # mesmo IP do CR10 (ver real_driver)
FT_TCP_PORT = 60000

FT_FORCE_AXIS_DEFAULT = 'z'
FT_FORCE_SIGN_DEFAULT = -1.0

FT_MODBUS_MAP_CONFIRMED = False

# None = endereço desconhecido. Os *_value/_on/_off são os payloads, que a
# captura também revela (o cliente mostra "reset 0 success." / "start 1
# success.", sugerindo canal 0/1 — confirme qual valor corresponde a quê).
FT_MODBUS_MAP: dict = {
    'zero':       None,   # Set_Zero        (0x06, escrita simples)
    'zero_value': 1,
    'rate':       None,   # Send_Frequency  (0x10, 2 regs = u32 Hz)
    'node_id':    None,   # Send_ModBus_ID  (0x06)
    'baud':       None,   # Send_Baud_rate  (0x10, 2 regs = u32 baud)
    'stream':     None,   # StartReading / stopReading (0x06)
    'stream_on':  1,
    'stream_off': 0,
    'device_id':  None,   # leitura do painel "Device ID"
}

FT_MODBUS_DATA_ADDR = 0x0003   # holding register inicial dos seis eixos
FT_MODBUS_DATA_REGS = 12       # 12 regs = 24 bytes = 6 x float32 LE


def ft_polled_max_rate_hz(baud: float = 115200) -> float:
    """Teto de amostras/s no modo POLLED — outra conta que a do stream.

    No stream você paga 28 B por amostra e mais nada. Aqui cada amostra custa
    a requisição (8 B), a resposta (29 B) e os DOIS silêncios de 3,5
    caracteres que o Modbus RTU exige entre quadros:

        (8 + 29) x 10 bits + 2 x 35 bits = 440 bits por amostra

    A 115200 isso dá ~262 Hz. É o teto do FIO: a latência de USB (~1 ms por
    sentido num CH340) não entra aqui e derruba a taxa medida para algo em
    torno de 150-200 Hz. Por isso a GUI mostra a taxa MEDIDA ao lado deste
    teto, em vez de prometer o número teórico.
    """
    return float(baud) / ((8 + 29) * 10 + 2 * 35)


# Modo de aquisição. Ortogonal ao MEIO (`ft_transport`: serial ou tcp) — dá
# para pollar pela 485 do flange na porta 60000 exatamente como pelo USB.
FT_MODE_STREAM = 'stream'
FT_MODE_POLLED = 'polled'
FT_MODE_CHOICES = (FT_MODE_STREAM, FT_MODE_POLLED)

FT_CHART_WINDOW_N   = 2000     # Chart_X1_Max / Chart_X2_Max — amostras
FT_CHART_FORCE_MAX  = 200.0    # Chart_Y1_Max / Min — N
FT_CHART_TORQUE_MAX = 50.0     # Chart_Y2_Max / Min — N·m

# Node ID do escravo. 1 é o default Modbus e o que o cliente de fábrica traz
# pré-preenchido; se você mudar com Send_ModBus_ID, mude aqui também.
FT_MODBUS_SLAVE_ID = 1
# Prazo de uma transação. Generoso de propósito: a resposta do comando chega
# no MEIO do stream de 1 kHz e o cliente precisa peneirar os quadros "ST"
# antes de achá-la (ver ft_modbus.find_response).
FT_MODBUS_TIMEOUT_S = 0.5
# Taxas de saída que o cliente de fábrica oferece no combo de frequência.
FT_RATE_CHOICES_HZ = (10, 50, 100, 200, 250, 500, 1000)
# Bauds oferecidos pelo mesmo cliente para a 485.
FT_BAUD_CHOICES = (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600,
                   1_000_000)
FT_PROFILE_HKVL56 = {
    'name':        'HKVL-56',
    'vendor':      'Suzhou Hangkai Microelectronics Technology Co., Ltd.',
    'style':       'hkvl56',
    # "Unless otherwise specified by the customer, the default baud rate when
    #  powering on ... is 1 Mbps" (§4.2, nota final).
    'baud_default': 1_000_000,
    # Exemplo 1: 01 03 00 03 00 18 B5 C0 — lê os seis eixos.
    'data_addr':   0x0003,
    'data_count':  0x0018,      # 24 BYTES (não registradores)
    # Exemplo 2: 01 06 00 00 00 02 08 0B — grava ID = 2.
    # "After modifying the ID, you must power on and restart the device."
    'node_id':     0x0000,
    # O manual só documenta 0x03 (ler) e 0x06 (escrever ID). Zero/tare, taxa
    # de saída e baud NÃO aparecem nele — continuam por capturar.
    'zero':        None,
    'rate':        None,
    'baud':        None,
    'stream':      None,
}

FT_SG_WINDOW_DEFAULT = 11    # ímpar
FT_SG_ORDER_DEFAULT  = 3     # < janela
FT_STATS_WINDOW_DEFAULT = 1000  # amostras de Mean_Num / MAX_Num (~1 s @1 kHz)

# Touch sensor (STM32 → PC plotter → UDP). Porta DIFERENTE da célula, senão
# os fluxos se misturam no mesmo receptor.
TOUCH_SENSOR_UDP_PORT = 8081
# Payload (little-endian, 8 bytes): uint32 seq + float valor. Espelhado no
# plotter standalone (sensors/touch_<grade>/tools).
TOUCH_PAYLOAD_FMT = '<If'
# Broadcast do I_final reemitido pelo TouchSensorSource a cada TOTAL.
TOUCH_UDP_BROADCAST_IP = '192.168.5.255'
# Relay do frame COMPLETO (linhas brutas do STM32) para PCs sem USB direto.
TOUCH_FRAME_UDP_PORT = 8082

# Idade máxima de uma amostra para entrar no par sincronizado (s).
SYNC_MAX_AGE_S = 0.25


TOUCH_FRAME_TOPIC = '/touch_sensor/frame'
TOUCH_EVENT_TOPIC = '/touch_sensor/spike_event'  # std_msgs/String: RA|SA|CN_MM|CN_RA|CN_SA


class TouchSensorSpec(NamedTuple):
    """O que distingue um sensor de toque do outro. Um sensor por vez na
    bancada, escolhido pelo parâmetro `sensor` do launch ('4' | '5')."""
    key: str
    rows: int
    cols: int
    # True: o firmware manda TOTAL/POST (I_final do neurônio pós). False: o
    # escalar publicado é a média das tensões de cada frame.
    has_total: bool
    label: str
    folder: str         # firmware e ferramentas, relativo à raiz do repo


TOUCH_SENSORS = {
    '5': TouchSensorSpec('5', 5, 5, False, '5×5 — RA+SA',
                         'sensors/touch_5x5'),
    '4': TouchSensorSpec('4', 4, 4, True, '4×4 — RA+SA+CN, TOTAL/I_final',
                         'sensors/touch_4x4'),
}
# O 5×5 é o padrão; valor desconhecido cai nele.
TOUCH_SENSOR_DEFAULT = '5'


def touch_sensor_spec(key) -> TouchSensorSpec:
    return TOUCH_SENSORS.get(str(key).strip(),
                             TOUCH_SENSORS[TOUCH_SENSOR_DEFAULT])


TOUCH_ROWS_DEFAULT = TOUCH_SENSORS[TOUCH_SENSOR_DEFAULT].rows
TOUCH_COLS_DEFAULT = TOUCH_SENSORS[TOUCH_SENSOR_DEFAULT].cols
TOUCH_TAXELS_DEFAULT = TOUCH_ROWS_DEFAULT * TOUCH_COLS_DEFAULT
TOUCH_EVENT_TYPES = ('RA', 'SA', 'CN_MM', 'CN_RA', 'CN_SA')

TOUCH_ROT180_GRIDS = frozenset({(5, 5)})


def taxel_frame_to_physical(vals: list, rows: int, cols: int) -> list:
    """Reordena um frame inteiro do firmware para a numeração FÍSICA.

    Depois disto o índice 0 é o taxel físico 00 e o índice rows*cols-1 é o
    último da última linha. Inverter a lista achatada É a rotação de 180° da
    grade, então a mesma linha serve para qualquer R×C. Grade não
    caracterizada (ou frame de tamanho inesperado) volta sem tocar."""
    if (rows, cols) in TOUCH_ROT180_GRIDS and len(vals) == rows * cols:
        return vals[::-1]
    return vals


def taxel_index_to_physical(idx: int, rows: int, cols: int) -> int:
    """O mesmo para um índice solto — as linhas RA/SA do firmware trazem
    `idx=` na convenção do frame, e o raster de spikes tem de casar com o
    heatmap. Grade não caracterizada volta sem tocar."""
    if (rows, cols) in TOUCH_ROT180_GRIDS:
        return rows * cols - 1 - idx
    return idx

def run_stamp_from_msg_time(stamp) -> str:
    """Identificador <AAAAMMDD_HHMMSS> do run a partir do campo `stamp` da
    PalpationStart.

    O logger e a GUI gravam arquivos DIFERENTES do mesmo run (__samples.csv de
    um lado, __sensors.csv e os CSVs crus do outro). Enquanto cada um chamava
    seu próprio `strftime` no instante em que começava, os nomes não batiam e
    não havia como juntar os dois no disco. Derivar o nome do MESMO carimbo da
    mensagem de início resolve isso sem inventar tópico novo.

    stamp zerado (publisher antigo ou `ros2 topic pub` sem o campo) cai na
    hora local — é o comportamento anterior, e nesse caso os nomes podem
    divergir por um segundo.
    """
    import time as _time
    try:
        sec = int(stamp.sec)
    except (AttributeError, TypeError, ValueError):
        sec = 0
    if sec <= 0:
        sec = int(_time.time())
    return _time.strftime('%Y%m%d_%H%M%S', _time.localtime(sec))


RUN_MODES = ('SLIDE', 'TOUCH', 'MANUAL', 'MATRIX_MAP')

# ── Quais modos EXECUTAM cada recurso opcional ────────────────────────
# Aqui e em nenhum outro lugar. A GUI decide com estas duas tuplas tanto o
# que MOSTRA (qual painel aparece) quanto o que ENVIA (quais campos vão
# preenchidos na PalpationStart), e o explorer despacha pelos mesmos
# conjuntos. Enquanto cada um desses três pontos carregava o seu próprio
# literal, eles podiam divergir — e divergiram: o painel da escada aparecia
# em MATRIX_MAP e o Start a zerava com um `!= 'MANUAL'`, então o operador
# configurava o mapa de histerese e recebia identações de patamar único,
# sem aviso nenhum. Um conjunto só é a única forma de isso não voltar.
STAIRCASE_MODES = ('MANUAL', 'MATRIX_MAP')   # _phase_hold_staircase
FMOD_MODES      = ('TOUCH',)                 # _phase_hold_modulated
# Gravação avulsa pelo botão "Record data", fora de qualquer run: não tem
# modo, mas também não pode cair na raiz junto das pastas de modo.
REC_DIR_NAME = 'RECORDING'

RUN_SAMPLES_CSV  = 'samples.csv'
RUN_SENSORS_CSV  = 'sensors.csv'
RUN_MATRIX_CSV   = 'matrix.csv'
RUN_ADC_CSV      = 'adc.csv'
RUN_SPIKES_CSV   = 'spikes.csv'
RUN_CN_CSV       = 'cuneiformes.csv'

# Run de APRENDIZADO: a home ainda não tem contato aprendido, a descida
# inteira rasteja só para memorizar a profundidade, e o run não é dado de
# ensaio. O explorer publica aqui o run_id dele (latched) e logger e GUI não
# gravam — ou apagam o que já tinham aberto, se o aviso chegou depois do start.
LEARNING_RUN_TOPIC = '/palpation/learning_run'


def discard_run_files(run_dir: str, names) -> None:
    """Apaga os arquivos `names` de `run_dir` e a pasta, se ficar vazia.
    Logger e GUI gravam arquivos diferentes na MESMA pasta: cada um apaga os
    seus, e quem terminar por último leva a pasta junto."""
    for name in names:
        try:
            os.remove(os.path.join(run_dir, name))
        except FileNotFoundError:
            pass
    try:
        os.rmdir(run_dir)
    except OSError:
        pass   # ainda há arquivos do outro gravador — ele remove depois
RUN_PARAMS_JSON  = 'params.json'
RUN_SUMMARY_JSON = 'summary.json'
RUN_PLOT_PNG     = 'plot.png'


def new_run_id() -> str:
    """Identidade de um run: <AAAAMMDD_HHMMSS> do relógio de PAREDE.

    Nunca do relógio ROS: sob use_sim_time ele vem do Gazebo e recomeça do
    zero a cada launch, o que fazia dois runs de sessões diferentes
    disputarem o mesmo nome de arquivo.
    """
    import time as _time
    return _time.strftime('%Y%m%d_%H%M%S', _time.localtime())


def run_id_from_msg(msg) -> str:
    """run_id da PalpationStart, ou o carimbo derivado de `stamp` quando o
    publisher é antigo e não traz o campo."""
    rid = _safe_component(str(getattr(msg, 'run_id', '') or ''))
    return rid or run_stamp_from_msg_time(getattr(msg, 'stamp', None))


def _safe_component(name: str) -> str:
    """Um componente de caminho a partir de texto que veio de MENSAGEM.

    `ros2 topic pub` pode mandar qualquer string em run_id/mode, e esses
    valores viram nome de diretório — sem filtro, um '../..' escreveria
    fora da pasta de dados.
    """
    keep = [c for c in str(name).strip() if c.isalnum() or c in '_-']
    return ''.join(keep)[:64]


def run_dir(mode: str, run_id: str, *, base: str | None = None,
            create: bool = True, sensor: str | None = None) -> str:
    """Diretório do run: <base>/[touch_RxC/]<MODO>/<run_id>, base = RUNS_DIR.

    Com `sensor` ('4' | '5') os runs de cada sensor de toque ficam separados
    (touch_4x4/ e touch_5x5/, o mesmo nome da pasta de firmware em sensors/):
    os dois têm grades e protocolos diferentes e não se agregam juntos.

    `mode` vazio (ou desconhecido) cai em REC_DIR_NAME — é o caso da
    gravação avulsa, que não pertence a modo nenhum. `run_id` vazio ou
    ilegível vira um carimbo novo, porque um run sem pasta não é gravável.
    """
    m = _safe_component(mode).upper()
    if m not in RUN_MODES:
        m = REC_DIR_NAME
    rid = _safe_component(run_id) or new_run_id()
    root = base or RUNS_DIR
    if sensor is not None:
        root = os.path.join(root,
                            os.path.basename(touch_sensor_spec(sensor).folder))
    path = os.path.join(root, m, rid)
    if create:
        os.makedirs(path, exist_ok=True)
    return path


# Códigos numéricos das fases no CSV unificado. RETRACT dobrado no HOME.
PHASE_CODES = {
    'IDLE': -1, 'HOME': 0, 'DESCENDING': 1, 'HOLD': 2, 'SLIDING': 3,
    'RETRACT': 0, 'DONE': 4, 'ABORTED': 5, 'FROZEN': 6, 'TRANSIT': 7,
    'CALIBRATING': 8, 'MODULATING': 9,
}
PHASE_NAMES = {-1: 'IDLE', 0: 'HOME', 1: 'DESCENDING', 2: 'HOLD',
               3: 'SLIDING', 4: 'DONE', 5: 'ABORTED', 6: 'FROZEN',
               7: 'TRANSIT', 8: 'CALIBRATING', 9: 'MODULATING'}

# ── MATRIX_MAP — defaults compartilhados GUI ↔ explorer ───────────────
# Safe Z: altura de trânsito acima da ORIGEM (primeiro contato).
MATRIX_SAFE_Z_MM_DEFAULT = 10.0
MATRIX_SAFE_Z_MM_MIN     = 2.0
MATRIX_SAFE_Z_MM_MAX     = 60.0
# Velocidade do trânsito XY no ar. Não há contato durante o trânsito, mas o
# teto é conservador: um erro de Safe Z vira arrasto sobre a peça.
MATRIX_TRANSIT_MMS_DEFAULT = 10.0
MATRIX_TRANSIT_MMS_MIN     = 1.0
MATRIX_TRANSIT_MMS_MAX     = 30.0
# Teto de pontos por matriz — protege contra uma grade absurda (ex.: 50×50)
# gerada por engano no configurador.
MATRIX_MAX_POINTS = 400
# Extensão máxima do plano em cada eixo, a partir da origem (mm).
MATRIX_SPAN_MAX_MM = 200.0

PROBE_ALIGN_POINTS_DEFAULT = 4
PROBE_ALIGN_POINTS_MIN     = 3
PROBE_ALIGN_POINTS_MAX     = 12
# Raio do polígono de sondagem: abaixo do mínimo o braço de alavanca do
# ajuste some no ruído dos toques; acima do máximo a sonda sai da peça.
PROBE_ALIGN_RADIUS_MM_DEFAULT = 15.0
PROBE_ALIGN_RADIUS_MM_MIN     = 5.0
PROBE_ALIGN_RADIUS_MM_MAX     = 60.0
# Setpoint dos toques de sonda. Leve para não marcar a amostra — o que faz
# o plano sair paralelo ao real é a IGUALDADE da penetração nos N pontos,
# não o valor. Nunca excede o setpoint do próprio ensaio (o explorer satura).
PROBE_ALIGN_FORCE_N_DEFAULT = 1.0

PROBE_ALIGN_RETRACT_MM_DEFAULT = 20.0
PROBE_ALIGN_RETRACT_MM_MIN     = 5.0
PROBE_ALIGN_RETRACT_MM_MAX     = 100.0
# Desvio máximo aceito. Acima disso o problema é de MONTAGEM (calço, fixação)
# e o lugar de corrigir não é o software. O teto DURO não é configurável: a
# 30° o J5 já está longe do útil e a rotação varreria a peça.
PROBE_ALIGN_TILT_MAX_DEG_DEFAULT = 20.0
PROBE_ALIGN_TILT_HARD_MAX_DEG    = 30.0

TOOL_STAMP_KEY = 'tool_tcp_mm'


def tool_tcp_mm() -> float:
    """Comprimento do TCP de palpação em mm (flange → face da ponteira)."""
    from .kinematics import T_TOUCH_TOOL_ATTACH
    return round(float(T_TOUCH_TOOL_ATTACH[2, 3]) * 1e3, 2)


def tool_stamp() -> dict:
    """Carimbo a mesclar no JSON gravado."""
    return {TOOL_STAMP_KEY: tool_tcp_mm()}


def tool_stamp_mismatch(data, *, what: str) -> str | None:
    """None se o carimbo confere; senão a frase pronta para o log.

    `data` é o JSON já carregado; `what` nomeia o arquivo na mensagem.
    Carimbo AUSENTE também devolve frase: o arquivo é anterior ao carimbo e
    portanto não há como afirmar com que ferramenta foi ensinado.
    """
    agora = tool_tcp_mm()
    if not isinstance(data, dict) or TOOL_STAMP_KEY not in data:
        return (f'{what} não traz carimbo de ferramenta — foi gravado antes '
                f'deste campo existir e pode ter sido ensinado com a pilha da '
                f'pilha anterior (TCP a 162,2 mm). A ferramenta atual '
                f'tem {agora:.1f} mm: confira a pose antes de descer.')
    try:
        antes = float(data[TOOL_STAMP_KEY])
    except (TypeError, ValueError):
        return (f'{what} tem carimbo de ferramenta ilegível '
                f'({data[TOOL_STAMP_KEY]!r}); a atual tem {agora:.1f} mm.')
    if abs(antes - agora) < 0.05:
        return None
    return (f'{what} foi ensinado com um TCP de {antes:.1f} mm e a ferramenta '
            f'montada tem {agora:.1f} mm ({agora - antes:+.1f} mm). Para os '
            f'MESMOS ângulos de junta a ponta está {abs(agora - antes):.1f} mm '
            f'{"mais alta" if agora < antes else "mais baixa"} que quando '
            f'isto foi ensinado.')


# Arquivos de configuração persistente (~/.config/touch_pack/).
CONFIG_DIR            = os.path.expanduser('~/.config/touch_pack')
HOME_POSE_FILE        = os.path.join(CONFIG_DIR, 'home_pose.json')
ROBOT_CONFIG_FILE     = os.path.join(CONFIG_DIR, 'robot.json')
POSES_FILE            = os.path.join(CONFIG_DIR, 'poses.json')
PALPATION_PARAMS_FILE = os.path.join(CONFIG_DIR, 'palpation_params.json')
# Profundidade do contato aprendida POR HOME.
LEARNED_CONTACT_FILE  = os.path.join(CONFIG_DIR, 'learned_contact.json')

# CSVs gravados em <raiz_do_repo>/sensors/Data. Override: TOUCH_PACK_DATA_DIR.
def _resolve_repo_root() -> str | None:
    """Sobe a partir deste arquivo até achar um diretório com `sensors/` —
    funciona do código-fonte (src/...) e do espaço instalado (install/...).
    None se o pacote estiver instalado fora da árvore do repo."""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        if os.path.isdir(os.path.join(d, 'sensors')):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return None


_REPO_ROOT = _resolve_repo_root()


def _resolve_runs_dir() -> str:
    env = os.environ.get('TOUCH_PACK_DATA_DIR')
    if env:
        return os.path.abspath(os.path.expanduser(env))
    if _REPO_ROOT:
        return os.path.join(_REPO_ROOT, 'sensors', 'Data')
    return os.path.expanduser('~/touch_pack_runs')


RUNS_DIR = _resolve_runs_dir()


def _resolve_latency_dir() -> str:
    """Onde os probes de latência gravam: `<repo>/data`, NUNCA sensors/Data.

    RUNS_DIR aponta para `sensors/Data`, que o .gitignore ignora inteiro (1,2
    GB de runs de palpação, com arquivos que o GitHub recusa). As medições de
    latência são pequenas (~2 MB por campanha) e sustentam o artigo, então
    moram em `data/`, versionado. Gravar direto aqui elimina o passo manual de
    copiar de sensors/ para data/ — passo que já fez uma campanha inteira
    ficar invisível ao git.
    """
    env = os.environ.get('TOUCH_PACK_LATENCY_DIR')
    if env:
        return os.path.abspath(os.path.expanduser(env))
    if _REPO_ROOT:
        return os.path.join(_REPO_ROOT, 'data')
    return os.path.join(RUNS_DIR, 'latency_runs')


LATENCY_DIR = _resolve_latency_dir()


# ── MANUAL em DEGRAU (escada de força) ────────────────────────────────
# Teto de patamares por ensaio — barra um passo minúsculo com máximo alto
# (0,01 N até 10 N = 1900 degraus) antes de o braço começar a andar.
STEP_MAX_LEVELS = 200


def staircase_levels(start_n: float, step_n: float, max_n: float,
                     *, cap: int = STEP_MAX_LEVELS) -> list[float]:
    start_n = float(start_n)
    step_n = float(step_n)
    max_n = float(max_n)
    if step_n <= 0.0 or max_n <= start_n:
        return [start_n]
    up = [start_n]
    v = start_n
    while True:
        v = round(v + step_n, 6)
        if v >= max_n - 1e-9:
            break
        up.append(v)
        if len(up) >= cap:
            # Estourou o teto ANTES de chegar ao pico. Truncar aqui seria
            # pior que recusar: o último degrau da subida saltaria do nível
            # truncado direto para max_n — vários newtons de uma vez contra
            # contato rígido. Devolve vazio e quem chamou recusa o ensaio.
            return []
    # Crava o máximo pedido, mesmo com passo que não fecha exato.
    if abs(up[-1] - max_n) > 1e-9:
        up.append(round(max_n, 6))
    # Descida pelos mesmos patamares, sem repetir o pico.
    return up + up[-2::-1]
