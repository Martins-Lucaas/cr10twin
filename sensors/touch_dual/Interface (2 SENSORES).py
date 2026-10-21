
import serial
import threading
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection


# ============================================================
# CONFIGURAÇÕES
# ============================================================

PORTA = "COM9"          # <<< ALTERE PARA SUA PORTA
BAUDRATE = 115200

ROWS = 5
COLS = 5
NUM_TAXELS = 25

ADC_MIN = 0
ADC_MAX = 4095

# Janela temporal mostrada no raster
RASTER_WINDOW = 10.0

# Atualização gráfica
# 33 ms ≈ 30 FPS
PLOT_INTERVAL_MS = 33

# Quantidade máxima de spikes guardados
MAX_SPIKES = 10000


# ============================================================
# DADOS COMPARTILHADOS
# ============================================================

adc_sensor1 = np.zeros(
    (ROWS, COLS),
    dtype=np.float32
)

adc_sensor2 = np.zeros(
    (ROWS, COLS),
    dtype=np.float32
)

# Spikes dos multimodais
spikes_mm1 = deque(
    maxlen=MAX_SPIKES
)

spikes_mm2 = deque(
    maxlen=MAX_SPIKES
)

data_lock = threading.Lock()

running = True

frames_recebidos = 0
frames_invalidos = 0

mm1_recebidos = 0
mm2_recebidos = 0

ultimo_timestamp_stm = 0
primeiro_timestamp_stm = None


# ============================================================
# SERIAL
# ============================================================

try:

    ser = serial.Serial(
        PORTA,
        BAUDRATE,
        timeout=0.01
    )

    time.sleep(2)

    ser.reset_input_buffer()

    print("=" * 60)
    print("STM32 CONECTADO")
    print(f"Porta: {PORTA}")
    print("=" * 60)

except serial.SerialException as e:

    print(
        f"Erro ao abrir {PORTA}: {e}"
    )

    raise SystemExit


# ============================================================
# TIMESTAMP
# ============================================================

def timestamp_para_segundos(timestamp):

    global primeiro_timestamp_stm

    if primeiro_timestamp_stm is None:

        primeiro_timestamp_stm = timestamp

    return (
        timestamp
        - primeiro_timestamp_stm
    ) / 1_000_000.0


# ============================================================
# PROCESSAR DATA
#
# FORMATO:
#
# DATA,S1,
# 25 ADCs,
# S2,
# 25 ADCs,
# t=89373179
#
# Índices:
#
# 0       DATA
# 1       S1
# 2:27    ADC S1
# 27      S2
# 28:53   ADC S2
# 53      timestamp
#
# ============================================================

def processar_data(linha):

    global frames_recebidos
    global frames_invalidos
    global ultimo_timestamp_stm

    try:

        partes = linha.split(",")

        # ====================================================
        # VALIDAÇÃO
        # ====================================================

        if len(partes) != 54:

            frames_invalidos += 1
            return

        if partes[0] != "DATA":

            frames_invalidos += 1
            return

        if partes[1] != "S1":

            frames_invalidos += 1
            return

        if partes[27] != "S2":

            frames_invalidos += 1
            return

        if not partes[53].startswith("t="):

            frames_invalidos += 1
            return

        # ====================================================
        # SENSOR 1
        # ====================================================

        valores_s1 = np.fromiter(
            (
                int(x)
                for x in partes[2:27]
            ),
            dtype=np.float32,
            count=25
        )

        # ====================================================
        # SENSOR 2
        # ====================================================

        valores_s2 = np.fromiter(
            (
                int(x)
                for x in partes[28:53]
            ),
            dtype=np.float32,
            count=25
        )

        # ====================================================
        # TIMESTAMP
        # ====================================================

        timestamp = int(
            partes[53][2:]
        )

        # ====================================================
        # MATRIZES 5x5
        # ====================================================

        matriz1 = valores_s1.reshape(
            ROWS,
            COLS
        )

        matriz2 = valores_s2.reshape(
            ROWS,
            COLS
        )

        # ====================================================
        # GUARDA SOMENTE O ADC MAIS RECENTE
        # ====================================================

        with data_lock:

            adc_sensor1[:, :] = matriz1
            adc_sensor2[:, :] = matriz2

            ultimo_timestamp_stm = timestamp

            frames_recebidos += 1

    except (
        ValueError,
        IndexError
    ):

        frames_invalidos += 1


# ============================================================
# PROCESSAR MULTIMODAL
#
# MM1=1,t=...
# MM2=1,t=...
#
# ============================================================

def processar_mm(linha):

    global mm1_recebidos
    global mm2_recebidos

    try:

        partes = linha.split(",")

        if len(partes) != 2:
            return

        tipo = partes[0]

        if not partes[1].startswith("t="):
            return

        timestamp = int(
            partes[1][2:]
        )

        tempo_s = timestamp_para_segundos(
            timestamp
        )

        with data_lock:

            if tipo == "MM1=1":

                spikes_mm1.append(
                    tempo_s
                )

                mm1_recebidos += 1

            elif tipo == "MM2=1":

                spikes_mm2.append(
                    tempo_s
                )

                mm2_recebidos += 1

    except (
        ValueError,
        IndexError
    ):

        pass


# ============================================================
# BUFFER DE RECEPÇÃO SERIAL
# ============================================================

rx_buffer = bytearray()


def leitura_serial():

    global running
    global rx_buffer

    while running:

        try:

            # =================================================
            # QUANTOS BYTES ESTÃO ESPERANDO?
            # =================================================

            disponiveis = ser.in_waiting

            if disponiveis <= 0:

                time.sleep(0.001)
                continue

            # =================================================
            # LÊ TUDO DE UMA VEZ
            # =================================================

            dados = ser.read(
                disponiveis
            )

            if not dados:
                continue

            rx_buffer.extend(
                dados
            )

            # =================================================
            # PROCESSA TODAS AS LINHAS COMPLETAS
            # =================================================

            while b"\n" in rx_buffer:

                linha_bytes, _, restante = (
                    rx_buffer.partition(b"\n")
                )

                rx_buffer = bytearray(
                    restante
                )

                linha = linha_bytes.decode(
                    "utf-8",
                    errors="ignore"
                ).strip()

                if not linha:
                    continue

                # =============================================
                # ADC
                # =============================================

                if linha.startswith(
                    "DATA,S1,"
                ):

                    processar_data(
                        linha
                    )

                # =============================================
                # MM1
                # =============================================

                elif linha.startswith(
                    "MM1=1,"
                ):

                    processar_mm(
                        linha
                    )

                # =============================================
                # MM2
                # =============================================

                elif linha.startswith(
                    "MM2=1,"
                ):

                    processar_mm(
                        linha
                    )

        except serial.SerialException as e:

            print(
                "Erro serial:",
                e
            )

            running = False

        except Exception as e:

            print(
                "Erro de leitura:",
                e
            )


# ============================================================
# INICIA THREAD SERIAL
# ============================================================

thread_serial = threading.Thread(
    target=leitura_serial,
    daemon=True
)

thread_serial.start()


# ============================================================
# FIGURA
# ============================================================

fig = plt.figure(
    figsize=(13, 8)
)

gs = fig.add_gridspec(
    2,
    2,
    height_ratios=[
        1.3,
        0.7
    ],
    hspace=0.40,
    wspace=0.30
)


# ============================================================
# HEATMAP 1
#
# CORES:
# cmap="jet"
#
# Azul -> Ciano -> Verde -> Amarelo -> Vermelho
# ============================================================

ax_heat1 = fig.add_subplot(
    gs[0, 0]
)

heat1 = ax_heat1.imshow(
    adc_sensor1,
    cmap="jet",
    vmin=ADC_MIN,
    vmax=ADC_MAX,
    interpolation="bicubic",
    aspect="equal"
)

ax_heat1.set_title(
    "Sensor 1 - ADC",
    fontsize=13,
    fontweight="bold"
)

ax_heat1.set_xlabel(
    "Coluna"
)

ax_heat1.set_ylabel(
    "Linha"
)

ax_heat1.set_xticks(
    range(COLS)
)

ax_heat1.set_yticks(
    range(ROWS)
)

ax_heat1.set_xticklabels(
    range(1, COLS + 1)
)

ax_heat1.set_yticklabels(
    range(1, ROWS + 1)
)

cbar1 = fig.colorbar(
    heat1,
    ax=ax_heat1,
    fraction=0.046,
    pad=0.04
)

cbar1.set_label(
    "ADC"
)


# ============================================================
# HEATMAP 2
#
# MESMA ESCALA E MESMAS CORES DO SENSOR 1
# ============================================================

ax_heat2 = fig.add_subplot(
    gs[0, 1]
)

heat2 = ax_heat2.imshow(
    adc_sensor2,
    cmap="jet",
    vmin=ADC_MIN,
    vmax=ADC_MAX,
    interpolation="bicubic",
    aspect="equal"
)

ax_heat2.set_title(
    "Sensor 2 - ADC",
    fontsize=13,
    fontweight="bold"
)

ax_heat2.set_xlabel(
    "Coluna"
)

ax_heat2.set_ylabel(
    "Linha"
)

ax_heat2.set_xticks(
    range(COLS)
)

ax_heat2.set_yticks(
    range(ROWS)
)

ax_heat2.set_xticklabels(
    range(1, COLS + 1)
)

ax_heat2.set_yticklabels(
    range(1, ROWS + 1)
)

cbar2 = fig.colorbar(
    heat2,
    ax=ax_heat2,
    fraction=0.046,
    pad=0.04
)

cbar2.set_label(
    "ADC"
)


# ============================================================
# VALORES DOS ADCs DENTRO DO HEATMAP
# ============================================================

textos1 = []
textos2 = []

for r in range(ROWS):

    linha1 = []
    linha2 = []

    for c in range(COLS):

        txt1 = ax_heat1.text(
            c,
            r,
            "0",
            ha="center",
            va="center",
            fontsize=8,
            color="white"
        )

        txt2 = ax_heat2.text(
            c,
            r,
            "0",
            ha="center",
            va="center",
            fontsize=8,
            color="white"
        )

        linha1.append(
            txt1
        )

        linha2.append(
            txt2
        )

    textos1.append(
        linha1
    )

    textos2.append(
        linha2
    )


# ============================================================
# RASTER MM1
# ============================================================

ax_raster1 = fig.add_subplot(
    gs[1, 0]
)

ax_raster1.set_title(
    "Sensor 1 - Raster MM1",
    fontsize=12,
    fontweight="bold"
)

ax_raster1.set_xlabel(
    "Tempo (s)"
)

ax_raster1.set_ylabel(
    "MM1"
)

ax_raster1.set_ylim(
    0.5,
    1.5
)

ax_raster1.set_yticks(
    [1]
)

ax_raster1.set_yticklabels(
    ["MM1"]
)

ax_raster1.grid(
    axis="x",
    alpha=0.25
)


# ============================================================
# RASTER MM2
# ============================================================

ax_raster2 = fig.add_subplot(
    gs[1, 1]
)

ax_raster2.set_title(
    "Sensor 2 - Raster MM2",
    fontsize=12,
    fontweight="bold"
)

ax_raster2.set_xlabel(
    "Tempo (s)"
)

ax_raster2.set_ylabel(
    "MM2"
)

ax_raster2.set_ylim(
    0.5,
    1.5
)

ax_raster2.set_yticks(
    [1]
)

ax_raster2.set_yticklabels(
    ["MM2"]
)

ax_raster2.grid(
    axis="x",
    alpha=0.25
)


# ============================================================
# LINECOLLECTION
# ============================================================

raster1 = LineCollection(
    [],
    linewidths=1.2
)

raster2 = LineCollection(
    [],
    linewidths=1.2
)

ax_raster1.add_collection(
    raster1
)

ax_raster2.add_collection(
    raster2
)


# ============================================================
# STATUS
# ============================================================

status1 = ax_heat1.text(
    0.0,
    1.05,
    "",
    transform=ax_heat1.transAxes,
    fontsize=9
)

status2 = ax_heat2.text(
    0.0,
    1.05,
    "",
    transform=ax_heat2.transAxes,
    fontsize=9
)


# ============================================================
# CONVERTER SPIKES EM SEGMENTOS VERTICAIS
# ============================================================

def criar_segmentos(spikes):

    if len(spikes) == 0:

        return np.empty(
            (0, 2, 2)
        )

    spikes = np.asarray(
        spikes,
        dtype=float
    )

    segmentos = np.empty(
        (
            len(spikes),
            2,
            2
        ),
        dtype=float
    )

    # X inicial
    segmentos[:, 0, 0] = spikes

    # Y inicial
    segmentos[:, 0, 1] = 0.65

    # X final
    segmentos[:, 1, 0] = spikes

    # Y final
    segmentos[:, 1, 1] = 1.35

    return segmentos


# ============================================================
# ATUALIZAÇÃO GRÁFICA
# ============================================================

def atualizar(frame):

    # ========================================================
    # COPIA RAPIDAMENTE OS DADOS
    # ========================================================

    with data_lock:

        adc1 = adc_sensor1.copy()
        adc2 = adc_sensor2.copy()

        s1 = list(
            spikes_mm1
        )

        s2 = list(
            spikes_mm2
        )

        frames_total = frames_recebidos

        total_mm1 = mm1_recebidos
        total_mm2 = mm2_recebidos

        ts_atual = ultimo_timestamp_stm

    # ========================================================
    # HEATMAP
    # ========================================================

    heat1.set_data(
        adc1
    )

    heat2.set_data(
        adc2
    )

    # ========================================================
    # NÚMEROS
    # ========================================================

    for r in range(ROWS):

        for c in range(COLS):

            textos1[r][c].set_text(
                f"{adc1[r, c]:.0f}"
            )

            textos2[r][c].set_text(
                f"{adc2[r, c]:.0f}"
            )

    # ========================================================
    # TEMPO ATUAL
    # ========================================================

    if (
        primeiro_timestamp_stm is not None
        and ts_atual >= primeiro_timestamp_stm
    ):

        tempo_atual = (
            ts_atual
            - primeiro_timestamp_stm
        ) / 1_000_000.0

    else:

        tempo_atual = 0.0

    inicio = max(
        0.0,
        tempo_atual - RASTER_WINDOW
    )

    fim = max(
        RASTER_WINDOW,
        tempo_atual
    )

    # ========================================================
    # SOMENTE SPIKES DA JANELA VISÍVEL
    # ========================================================

    if s1:

        s1_array = np.asarray(
            s1
        )

        s1_visivel = s1_array[
            s1_array >= inicio
        ]

    else:

        s1_visivel = np.array([])

    if s2:

        s2_array = np.asarray(
            s2
        )

        s2_visivel = s2_array[
            s2_array >= inicio
        ]

    else:

        s2_visivel = np.array([])

    # ========================================================
    # ATUALIZA RASTER
    # ========================================================

    raster1.set_segments(
        criar_segmentos(
            s1_visivel
        )
    )

    raster2.set_segments(
        criar_segmentos(
            s2_visivel
        )
    )

    # ========================================================
    # LIMITES
    # ========================================================

    ax_raster1.set_xlim(
        inicio,
        fim
    )

    ax_raster2.set_xlim(
        inicio,
        fim
    )

    # ========================================================
    # STATUS
    # ========================================================

    status1.set_text(
        f""
    )

    status2.set_text(
        f""
    )

    return (
        heat1,
        heat2,
        raster1,
        raster2,
        status1,
        status2
    )


# ============================================================
# ANIMAÇÃO
# ============================================================

ani = FuncAnimation(
    fig,
    atualizar,
    interval=PLOT_INTERVAL_MS,
    blit=False,
    cache_frame_data=False
)


# ============================================================
# FECHAMENTO
# ============================================================

def fechar(event):

    global running

    running = False

    try:

        ser.close()

    except Exception:

        pass

    print()
    print("=" * 60)
    print("AQUISIÇÃO ENCERRADA")
    print(f"Frames ADC: {frames_recebidos}")
    print(f"Frames inválidos: {frames_invalidos}")
    print(f"Spikes MM1: {mm1_recebidos}")
    print(f"Spikes MM2: {mm2_recebidos}")
    print("=" * 60)


fig.canvas.mpl_connect(
    "close_event",
    fechar
)


# ============================================================
# TÍTULO
# ============================================================

fig.suptitle(
    "Sistema Neuromórfico - Dois Sensores",
    fontsize=15,
    fontweight="bold"
)


plt.show()

