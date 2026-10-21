import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re


PORT = "COM9"
BAUD = 115200

ROWS = 5
COLS = 5
NUM_TAXELS = ROWS * COLS

RASTER_WINDOW = 5.0


# ==========================================================
# CORES
# ==========================================================

COR_FA = "#0B3954"
COR_SA = "#B24C3D"

COR_CNMM = "#3D4F8A"
COR_CNFA = "#8C2D5D"
COR_CNSA = "#006B6B"


# ==========================================================
# SERIAL
# ==========================================================

ser = serial.Serial(
    PORT,
    BAUD,
    timeout=0.1
)

serial_buffer = ""


# ==========================================================
# VARIÁVEIS
# ==========================================================

current_time = 0.0


spike_times_RA = [
    []
    for _ in range(NUM_TAXELS)
]


spike_times_SA = [
    []
    for _ in range(NUM_TAXELS)
]


spike_times_CN_MM = []
spike_times_CN_RA = []
spike_times_CN_SA = []


# ==========================================================
# FIGURA
# ==========================================================

fig, axs = plt.subplots(
    1,
    3,
    figsize=(16, 5)
)


ax1 = axs[0]
ax2 = axs[1]
ax3 = axs[2]


# ==========================================================
# MAPA DE ADC
# ==========================================================

mapa_adc = np.zeros(
    (ROWS, COLS)
)


im = ax1.imshow(
    mapa_adc,
    cmap="jet",
    #interpolation="bicud",
    vmin=0,
    vmax=4095
)


plt.colorbar(
    im,
    ax=ax1,
    label="ADC"
)


ax1.set_title(
    "Mapa de ADC"
)


ax1.set_xticks(
    range(COLS)
)


ax1.set_yticks(
    range(ROWS)
)


texts = []


for r in range(ROWS):

    linha_texto = []

    for c in range(COLS):

        texto = ax1.text(
            c,
            r,
            "0",
            ha="center",
            va="center",
            color="white",
            fontsize=8
        )

        linha_texto.append(
            texto
        )

    texts.append(
        linha_texto
    )


# ==========================================================
# RASTER FA / SA
# ==========================================================

ax2.set_title(
    "Raster FA / SA"
)


ax2.set_xlim(
    0,
    RASTER_WINDOW
)


ax2.set_ylim(
    -1,
    NUM_TAXELS * 2
)


scatter_RA = ax2.scatter(
    [],
    [],
    s=10,
    color=COR_FA,
    label="FA"
)


scatter_SA = ax2.scatter(
    [],
    [],
    s=10,
    color=COR_SA,
    label="SA"
)


ax2.set_xlabel(
    "Tempo (s)"
)


ax2.set_ylabel(
    "Taxel"
)


ax2.legend()


# ==========================================================
# NEURÔNIOS CUNEIFORMES
# ==========================================================

ax3.set_title(
    "Neurônios Cuneiformes"
)


ax3.set_xlim(
    0,
    RASTER_WINDOW
)


ax3.set_ylim(
    -1,
    3
)


scatter_CN_MM = ax3.scatter(
    [],
    [],
    s=30,
    color=COR_CNMM,
    label="CN_MM"
)


scatter_CN_RA = ax3.scatter(
    [],
    [],
    s=30,
    color=COR_CNFA,
    label="CN_FA"
)


scatter_CN_SA = ax3.scatter(
    [],
    [],
    s=30,
    color=COR_CNSA,
    label="CN_SA"
)


ax3.set_yticks(
    [0, 1, 2]
)


ax3.set_yticklabels(
    [
        "CN_MM",
        "CN_FA",
        "CN_SA"
    ]
)


ax3.set_xlabel(
    "Tempo (s)"
)


ax3.legend()


# ==========================================================
# LIMPAR SPIKES ANTIGOS
# ==========================================================

def limpar_spikes(
    lista,
    tempo_atual
):

    nova_lista = []

    for t in lista:

        if tempo_atual - t <= RASTER_WINDOW:

            nova_lista.append(
                t
            )

    return nova_lista


# ==========================================================
# ATUALIZAÇÃO
# ==========================================================

def update(frame):

    global serial_buffer
    global mapa_adc
    global current_time


    if ser.in_waiting > 0:

        dados = ser.read(
            ser.in_waiting
        ).decode(
            errors="ignore"
        )


        serial_buffer += dados


        linhas = serial_buffer.split(
            "\n"
        )


        serial_buffer = linhas[-1]


        for line in linhas[:-1]:

            line = line.strip()


            if line == "":

                continue


            # ==================================================
            # ADC
            # ==================================================

            if line.startswith(
                "ADC"
            ):

                try:

                    partes = line.split(
                        ","
                    )


                    if len(partes) < NUM_TAXELS + 2:

                        continue


                    valores_adc = []


                    for i in range(
                        NUM_TAXELS
                    ):

                        valores_adc.append(
                            int(
                                partes[i + 1]
                            )
                        )


                    timestamp_texto = partes[
                        NUM_TAXELS + 1
                    ]


                    timestamp = int(
                        timestamp_texto.split(
                            "="
                        )[1]
                    )


                    current_time = (
                        timestamp / 1000000.0
                    )


                    # ==========================================
                    # ADC DIRETO NO MAPA
                    # ==========================================

                    for r in range(
                        ROWS
                    ):

                        for c in range(
                            COLS
                        ):

                            idx = (
                                r * COLS
                                + c
                            )


                            mapa_adc[
                                r,
                                c
                            ] = valores_adc[
                                idx
                            ]


                except Exception:

                    pass

            # ==================================================
            # SPIKE FA
            # ==================================================

            elif line.startswith(
                "RA"
            ):

                try:

                    m = re.search(
                        r"idx=(\d+),adc=\d+,t=(\d+)",
                        line
                    )


                    if m:

                        idx = int(
                            m.group(1)
                        )


                        timestamp = int(
                            m.group(2)
                        )


                        if idx < NUM_TAXELS:

                            spike_times_RA[
                                idx
                            ].append(
                                timestamp / 1000000.0
                            )


                            current_time = (
                                timestamp / 1000000.0
                            )


                except Exception:

                    pass


            # ==================================================
            # SPIKE SA
            # ==================================================

            elif line.startswith(
                "SA"
            ):

                try:

                    m = re.search(
                        r"idx=(\d+),adc=\d+,t=(\d+)",
                        line
                    )


                    if m:

                        idx = int(
                            m.group(1)
                        )


                        timestamp = int(
                            m.group(2)
                        )


                        if idx < NUM_TAXELS:

                            spike_times_SA[
                                idx
                            ].append(
                                timestamp / 1000000.0
                            )


                            current_time = (
                                timestamp / 1000000.0
                            )


                except Exception:

                    pass


            # ==================================================
            # CN MM
            # ==================================================

            elif line.startswith(
                "CN_MM"
            ):

                try:

                    m = re.search(
                        r"t=(\d+)",
                        line
                    )


                    if m:

                        timestamp = int(
                            m.group(1)
                        )


                        spike_times_CN_MM.append(
                            timestamp / 1000000.0
                        )


                        current_time = (
                            timestamp / 1000000.0
                        )


                except Exception:

                    pass


            # ==================================================
            # CN FA
            # ==================================================

            elif line.startswith(
                "CN_RA"
            ):

                try:

                    m = re.search(
                        r"t=(\d+)",
                        line
                    )


                    if m:

                        timestamp = int(
                            m.group(1)
                        )


                        spike_times_CN_RA.append(
                            timestamp / 1000000.0
                        )


                        current_time = (
                            timestamp / 1000000.0
                        )


                except Exception:

                    pass


            # ==================================================
            # CN SA
            # ==================================================

            elif line.startswith(
                "CN_SA"
            ):

                try:

                    m = re.search(
                        r"t=(\d+)",
                        line
                    )


                    if m:

                        timestamp = int(
                            m.group(1)
                        )


                        spike_times_CN_SA.append(
                            timestamp / 1000000.0
                        )


                        current_time = (
                            timestamp / 1000000.0
                        )


                except Exception:

                    pass


    # ==========================================================
    # ATUALIZA MAPA
    # ==========================================================

    #im.set_data(np.rot90(mapa_adc, 2))
    im.set_data(mapa_adc)


    for r in range(
        ROWS
    ):

        for c in range(
            COLS
        ):

            texts[r][c].set_text(
                "{:.0f}".format(
                    mapa_adc[
                        r,
                        c
                    ]
                )
            )


    # ==========================================================
    # RASTER FA
    # ==========================================================

    x_RA = []
    y_RA = []


    for n in range(
        NUM_TAXELS
    ):

        spike_times_RA[n] = limpar_spikes(
            spike_times_RA[n],
            current_time
        )


        for t in spike_times_RA[n]:

            x_RA.append(
                t
            )


            y_RA.append(
                n
            )


    # ==========================================================
    # RASTER SA
    # ==========================================================

    x_SA = []
    y_SA = []


    for n in range(
        NUM_TAXELS
    ):

        spike_times_SA[n] = limpar_spikes(
            spike_times_SA[n],
            current_time
        )


        for t in spike_times_SA[n]:

            x_SA.append(
                t
            )


            y_SA.append(
                n + NUM_TAXELS
            )


    if len(x_RA) > 0:

        scatter_RA.set_offsets(
            np.c_[
                x_RA,
                y_RA
            ]
        )

    else:

        scatter_RA.set_offsets(
            np.empty(
                (0, 2)
            )
        )


    if len(x_SA) > 0:

        scatter_SA.set_offsets(
            np.c_[
                x_SA,
                y_SA
            ]
        )

    else:

        scatter_SA.set_offsets(
            np.empty(
                (0, 2)
            )
        )


    ax2.set_xlim(
        max(
            0,
            current_time - RASTER_WINDOW
        ),
        max(
            RASTER_WINDOW,
            current_time
        )
    )


    # ==========================================================
    # CUNEIFORMES
    # ==========================================================

    spike_times_CN_MM[:] = limpar_spikes(
        spike_times_CN_MM,
        current_time
    )


    spike_times_CN_RA[:] = limpar_spikes(
        spike_times_CN_RA,
        current_time
    )


    spike_times_CN_SA[:] = limpar_spikes(
        spike_times_CN_SA,
        current_time
    )


    if len(spike_times_CN_MM) > 0:

        scatter_CN_MM.set_offsets(
            np.c_[
                spike_times_CN_MM,
                [0] * len(spike_times_CN_MM)
            ]
        )

    else:

        scatter_CN_MM.set_offsets(
            np.empty(
                (0, 2)
            )
        )


    if len(spike_times_CN_RA) > 0:

        scatter_CN_RA.set_offsets(
            np.c_[
                spike_times_CN_RA,
                [1] * len(spike_times_CN_RA)
            ]
        )

    else:

        scatter_CN_RA.set_offsets(
            np.empty(
                (0, 2)
            )
        )


    if len(spike_times_CN_SA) > 0:

        scatter_CN_SA.set_offsets(
            np.c_[
                spike_times_CN_SA,
                [2] * len(spike_times_CN_SA)
            ]
        )

    else:

        scatter_CN_SA.set_offsets(
            np.empty(
                (0, 2)
            )
        )


    ax3.set_xlim(
        max(
            0,
            current_time - RASTER_WINDOW
        ),
        max(
            RASTER_WINDOW,
            current_time
        )
    )


    return [
        im,
        scatter_RA,
        scatter_SA,
        scatter_CN_MM,
        scatter_CN_RA,
        scatter_CN_SA
    ]


# ==========================================================
# ANIMAÇÃO
# ==========================================================

ani = FuncAnimation(
    fig,
    update,
    interval=50,
    blit=False,
    cache_frame_data=False
)


plt.tight_layout()


plt.show()