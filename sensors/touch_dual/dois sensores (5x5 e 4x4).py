
import serial
import threading
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ============================================================
# CONFIGURACAO
# ============================================================

PORTA = "COM9"
BAUDRATE = 115200
TIMEOUT_SERIAL = 0.005

ADC_MIN = 0
ADC_MAX = 4095

ROWS = 5
COLS = 5

JANELA_RASTER_S = 5.0

INTERVALO_PLOT_MS = 50
INTERVALO_HEATMAP_S = 0.10
INTERVALO_RASTER_S = 0.10

MAX_SPIKES = 1000

FATOR_TIMESTAMP = 1_000_000.0
TIMESTAMP_MODULO = 2**32

# Aparencia dos spikes
COR_MM1 = "#1E4B00"  # Azul
COR_MM2 = "#E63946"  # Vermelho
TAMANHO_SPIKE = 22

plt.rcParams["font.size"] = 9
plt.rcParams["path.simplify"] = True


# ============================================================
# DADOS COMPARTILHADOS
# ============================================================

lock = threading.Lock()
executando = True

adc_sensor1 = np.zeros((ROWS, COLS), dtype=np.float32)
media_s2 = np.zeros((2, 2), dtype=np.float32)

spikes_mm1 = deque(maxlen=MAX_SPIKES)
spikes_mm2 = deque(maxlen=MAX_SPIKES)

timestamp_inicial = None
ultimo_timestamp = None
tempo_relativo = 0.0

frames_recebidos = 0
frames_invalidos = 0

medias_recebidas = 0
medias_invalidas = 0

eventos_mm1 = 0
eventos_mm2 = 0

ultima_mensagem = "Aguardando STM32..."


# ============================================================
# SERIAL
# ============================================================

try:
    ser = serial.Serial(
        PORTA,
        BAUDRATE,
        timeout=TIMEOUT_SERIAL
    )

    time.sleep(1.0)
    ser.reset_input_buffer()

    print(f"Conectado em {PORTA} a {BAUDRATE} baud")

except serial.SerialException as erro:
    raise SystemExit(
        f"Nao foi possivel abrir {PORTA}: {erro}"
    )


# ============================================================
# TEMPO
# ============================================================

def extrair_timestamp(campo):
    if not campo.startswith("t="):
        raise ValueError("Timestamp ausente")

    t = int(campo[2:])

    if not 0 <= t < TIMESTAMP_MODULO:
        raise ValueError("Timestamp invalido")

    return t


def registrar_timestamp(t):
    global timestamp_inicial
    global ultimo_timestamp
    global tempo_relativo

    with lock:
        if timestamp_inicial is None:
            timestamp_inicial = t
            ultimo_timestamp = t
            tempo_relativo = 0.0
            return 0.0

        delta = (t - ultimo_timestamp) & 0xFFFFFFFF

        if delta < TIMESTAMP_MODULO // 2:
            tempo_relativo += delta / FATOR_TIMESTAMP
            ultimo_timestamp = t

        return tempo_relativo


# ============================================================
# REGISTRO DOS TIMESTAMPS DAS LINHAS
# ============================================================

def registrar_timestamp_linha(linha, tipo):
    try:
        p = linha.split(",")

        if tipo == "DATA":
            if len(p) != 54:
                return False
            campo = p[53]

        elif tipo == "MEDIA_S2":
            if len(p) != 6:
                return False
            campo = p[5]

        else:
            return False

        t = extrair_timestamp(campo)
        registrar_timestamp(t)
        return True

    except (ValueError, IndexError):
        return False


# ============================================================
# PARSER DATA
# DATA,S1,<25 ADC>,S2,<25 ADC>,t=<timestamp>
# ============================================================

def processar_data(linha, timestamp_ja_registrado=False):
    global adc_sensor1
    global frames_recebidos
    global frames_invalidos
    global ultima_mensagem

    try:
        p = linha.split(",")

        if len(p) != 54 or p[0] != "DATA":
            raise ValueError("Formato DATA incorreto")

        if p[1] != "S1" or p[27] != "S2":
            raise ValueError("Identificadores S1/S2 incorretos")

        s1 = np.asarray(
            p[2:27], dtype=np.int32
        ).reshape(ROWS, COLS)

        s2 = np.asarray(
            p[28:53], dtype=np.int32
        ).reshape(ROWS, COLS)

        t = extrair_timestamp(p[53])

        if (
            np.any(s1 < ADC_MIN)
            or np.any(s1 > ADC_MAX)
            or np.any(s2 < ADC_MIN)
            or np.any(s2 > ADC_MAX)
        ):
            raise ValueError("ADC fora da faixa")

        if not timestamp_ja_registrado:
            registrar_timestamp(t)

        with lock:
            adc_sensor1 = s1.astype(np.float32)
            frames_recebidos += 1
            ultima_mensagem = "Recebendo dados"

    except (ValueError, IndexError) as e:
        with lock:
            frames_invalidos += 1
            ultima_mensagem = f"DATA invalido: {e}"


# ============================================================
# PARSER MEDIAS SENSOR 2
# ============================================================

def processar_media(linha, timestamp_ja_registrado=False):
    global media_s2
    global medias_recebidas
    global medias_invalidas
    global ultima_mensagem

    try:
        p = linha.split(",")

        if len(p) != 6 or p[0] != "MEDIA_S2":
            raise ValueError("Formato MEDIA_S2 incorreto")

        valores = np.asarray(
            p[1:5], dtype=np.int32
        )

        if (
            np.any(valores < ADC_MIN)
            or np.any(valores > ADC_MAX)
        ):
            raise ValueError("Media ADC fora da faixa")

        t = extrair_timestamp(p[5])

        if not timestamp_ja_registrado:
            registrar_timestamp(t)

        with lock:
            media_s2 = valores.astype(
                np.float32
            ).reshape(2, 2)

            medias_recebidas += 1
            ultima_mensagem = "Recebendo medias"

    except (ValueError, IndexError) as e:
        with lock:
            medias_invalidas += 1
            ultima_mensagem = f"MEDIA invalida: {e}"


# ============================================================
# PARSER DE DISPAROS
# ============================================================

def processar_spike(linha, nome):
    global eventos_mm1
    global eventos_mm2

    try:
        p = linha.split(",")

        if len(p) != 2 or p[0] != f"{nome}=1":
            return

        t = extrair_timestamp(p[1])
        tempo = registrar_timestamp(t)

        with lock:
            if nome == "MM1":
                spikes_mm1.append(tempo)
                eventos_mm1 += 1
            else:
                spikes_mm2.append(tempo)
                eventos_mm2 += 1

    except (ValueError, IndexError):
        pass


# ============================================================
# AQUISICAO SERIAL
# Processa todos os spikes e mantem o dado mais recente
# de cada matriz por bloco recebido.
# ============================================================

def thread_serial():
    global ultima_mensagem

    buffer = bytearray()

    while executando:
        try:
            disponiveis = ser.in_waiting

            if disponiveis > 0:
                bloco = ser.read(min(disponiveis, 8192))
            else:
                bloco = ser.read(1)

            if not bloco:
                continue

            buffer.extend(bloco)

            if len(buffer) > 65536 and b"\n" not in buffer:
                buffer.clear()

                with lock:
                    ultima_mensagem = "Buffer serial reiniciado"

                continue

            linhas = []

            while True:
                pos = buffer.find(b"\n")

                if pos < 0:
                    break

                raw = bytes(buffer[:pos])
                del buffer[:pos + 1]

                linha = raw.decode(
                    "ascii", errors="ignore"
                ).strip()

                if linha:
                    linhas.append(linha)

            if not linhas:
                continue

            ultima_data = None
            ultima_media = None

            # Preserva a ordem temporal original
            for linha in linhas:

                if linha.startswith("DATA,"):
                    registrar_timestamp_linha(
                        linha, "DATA"
                    )
                    ultima_data = linha

                elif linha.startswith("MEDIA_S2,"):
                    registrar_timestamp_linha(
                        linha, "MEDIA_S2"
                    )
                    ultima_media = linha

                elif linha.startswith("MM1=1,"):
                    processar_spike(linha, "MM1")

                elif linha.startswith("MM2=1,"):
                    processar_spike(linha, "MM2")

            if ultima_data is not None:
                processar_data(
                    ultima_data,
                    timestamp_ja_registrado=True
                )

            if ultima_media is not None:
                processar_media(
                    ultima_media,
                    timestamp_ja_registrado=True
                )

        except (serial.SerialException, OSError) as e:
            if not executando:
                break

            with lock:
                ultima_mensagem = f"Erro serial: {e}"

            time.sleep(0.05)


thread = threading.Thread(
    target=thread_serial,
    daemon=True
)
thread.start()


# ============================================================
# FIGURA
# ============================================================

fig, axes = plt.subplots(
    2, 2,
    figsize=(11, 7)
)

try:
    fig.canvas.manager.set_window_title(
        "Monitoramento dos Sensores - STM32"
    )
except Exception:
    pass

ax_s1, ax_s2 = axes[0]
ax_mm1, ax_mm2 = axes[1]


# ============================================================
# HEATMAP SENSOR 1 - INTERPOLACAO BICUBICA
# ============================================================

im_s1 = ax_s1.imshow(
    adc_sensor1,
    cmap="jet",
    vmin=ADC_MIN,
    vmax=ADC_MAX,
    interpolation="bicubic",
    resample=False
)

ax_s1.set_title("Sensor 1 - Matriz 5x5")
ax_s1.set_xlabel("Coluna")
ax_s1.set_ylabel("Linha")
ax_s1.set_xticks(range(COLS))
ax_s1.set_yticks(range(ROWS))

txt_s1 = [
    ax_s1.text(
        j, i, "0",
        ha="center",
        va="center",
        color="white",
        fontsize=8
    )
    for i in range(ROWS)
    for j in range(COLS)
]

fig.colorbar(im_s1, ax=ax_s1, label="ADC")


# ============================================================
# HEATMAP SENSOR 2 - INTERPOLACAO BICUBICA
# ============================================================

im_s2 = ax_s2.imshow(
    media_s2,
    cmap="jet",
    vmin=ADC_MIN,
    vmax=ADC_MAX,
    interpolation="bicubic",
    resample=False
)

ax_s2.set_title("Sensor 2 - Medias regionais")
ax_s2.set_xticks([0, 1])
ax_s2.set_xticklabels(["Esquerda", "Direita"])
ax_s2.set_yticks([0, 1])
ax_s2.set_yticklabels(["Superior", "Inferior"])

txt_s2 = [
    ax_s2.text(
        j, i, "0",
        ha="center",
        va="center",
        color="white",
        fontsize=11
    )
    for i in range(2)
    for j in range(2)
]

fig.colorbar(im_s2, ax=ax_s2, label="Media ADC")


# ============================================================
# RASTERS - SPIKES EM FORMATO DE BOLINHAS
# ============================================================

def configurar_raster(ax, nome):
    ax.set_title(f"Disparos - {nome}")
    ax.set_xlabel("Tempo relativo (s)")
    ax.set_ylabel("Neuronio")

    ax.set_yticks([1])
    ax.set_yticklabels([nome])
    ax.set_ylim(0.5, 1.5)
    ax.set_xlim(0, JANELA_RASTER_S)
    ax.grid(True, alpha=0.2)


configurar_raster(ax_mm1, "CN_MM1")
configurar_raster(ax_mm2, "CN_MM2")

# Cada spike e desenhado como uma bolinha
spikes_plot_mm1 = ax_mm1.scatter(
    [],
    [],
    s=TAMANHO_SPIKE,
    color=COR_MM1,
    edgecolors="none",
    marker="o"
)

spikes_plot_mm2 = ax_mm2.scatter(
    [],
    [],
    s=TAMANHO_SPIKE,
    color=COR_MM2,
    edgecolors="none",
    marker="o"
)

txt_mm1 = ax_mm1.text(
    0.02, 0.90,
    "Total: 0",
    transform=ax_mm1.transAxes,
    va="top"
)

txt_mm2 = ax_mm2.text(
    0.02, 0.90,
    "Total: 0",
    transform=ax_mm2.transAxes,
    va="top"
)

status = fig.text(
    0.02, 0.015,
    "Aguardando dados...",
    fontsize=8
)

fig.tight_layout(rect=[0, 0.045, 1, 1])


# ============================================================
# CONSTRUCAO DOS PONTOS DOS SPIKES
# ============================================================

def construir_pontos(eventos, inicio, fim):
    if not eventos:
        return np.empty(0), np.empty(0)

    tempos = np.fromiter(
        eventos,
        dtype=np.float64,
        count=len(eventos)
    )

    tempos = tempos[
        (tempos >= inicio) & (tempos <= fim)
    ]

    posicoes_y = np.ones(
        tempos.size,
        dtype=np.float64
    )

    return tempos, posicoes_y


# ============================================================
# ATUALIZACAO DOS GRAFICOS
# ============================================================

ultima_atualizacao_heatmap = 0.0
ultima_atualizacao_raster = 0.0
limites_anteriores = None


def atualizar(_):
    global ultima_atualizacao_heatmap
    global ultima_atualizacao_raster
    global limites_anteriores

    agora = time.monotonic()

    atualizar_heatmap = (
        agora - ultima_atualizacao_heatmap
        >= INTERVALO_HEATMAP_S
    )

    atualizar_raster = (
        agora - ultima_atualizacao_raster
        >= INTERVALO_RASTER_S
    )

    # Copia os dados e libera o lock antes do desenho
    with lock:
        tempo = tempo_relativo

        n_frames = frames_recebidos
        n_bad = frames_invalidos
        n_medias = medias_recebidas
        n_medias_bad = medias_invalidas

        n_mm1 = eventos_mm1
        n_mm2 = eventos_mm2

        msg = ultima_mensagem

        if atualizar_heatmap:
            s1 = adc_sensor1.copy()
            s2 = media_s2.copy()
        else:
            s1 = None
            s2 = None

        if atualizar_raster:
            sp1 = tuple(spikes_mm1)
            sp2 = tuple(spikes_mm2)
        else:
            sp1 = None
            sp2 = None

    # --------------------------------------------------------
    # HEATMAPS
    # --------------------------------------------------------

    if atualizar_heatmap:
        im_s1.set_data(s1)
        im_s2.set_data(s2)

        for k, txt in enumerate(txt_s1):
            i, j = divmod(k, COLS)
            valor = s1[i, j]

            txt.set_text(str(int(valor)))
            txt.set_color(
                "black" if valor > 2800 else "white"
            )

        for k, txt in enumerate(txt_s2):
            i, j = divmod(k, 2)
            valor = s2[i, j]

            txt.set_text(str(int(valor)))
            txt.set_color(
                "black" if valor > 2800 else "white"
            )

        ultima_atualizacao_heatmap = agora

    # --------------------------------------------------------
    # RASTERS - BOLINHAS
    # --------------------------------------------------------

    if atualizar_raster:
        fim = max(
            JANELA_RASTER_S,
            tempo + 0.05
        )

        inicio = max(
            0.0,
            fim - JANELA_RASTER_S
        )

        x1, y1 = construir_pontos(
            sp1, inicio, fim
        )

        x2, y2 = construir_pontos(
            sp2, inicio, fim
        )

        if len(x1):
            spikes_plot_mm1.set_offsets(
                np.column_stack((x1, y1))
            )
        else:
            spikes_plot_mm1.set_offsets(
                np.empty((0, 2))
            )

        if len(x2):
            spikes_plot_mm2.set_offsets(
                np.column_stack((x2, y2))
            )
        else:
            spikes_plot_mm2.set_offsets(
                np.empty((0, 2))
            )

        novos_limites = (inicio, fim)

        if novos_limites != limites_anteriores:
            ax_mm1.set_xlim(inicio, fim)
            ax_mm2.set_xlim(inicio, fim)
            limites_anteriores = novos_limites

        txt_mm1.set_text(
            f"Total: {n_mm1} | Janela: {len(x1)}"
        )

        txt_mm2.set_text(
            f"Total: {n_mm2} | Janela: {len(x2)}"
        )

        ultima_atualizacao_raster = agora

    # --------------------------------------------------------
    # STATUS
    # --------------------------------------------------------



    return (
        [im_s1, im_s2,
         spikes_plot_mm1, spikes_plot_mm2,
         txt_mm1, txt_mm2, status]
        + txt_s1
        + txt_s2
    )


# ============================================================
# ENCERRAMENTO
# ============================================================

def fechar():
    global executando

    if not executando:
        return

    executando = False

    try:
        if ser.is_open:
            ser.close()
    except Exception:
        pass


fig.canvas.mpl_connect(
    "close_event",
    lambda event: fechar()
)


# ============================================================
# ANIMACAO
# ============================================================

animacao = FuncAnimation(
    fig,
    atualizar,
    interval=INTERVALO_PLOT_MS,
    blit=False,
    cache_frame_data=False
)


# ============================================================
# EXECUCAO
# ============================================================

try:
    plt.show()
finally:
    fechar()
