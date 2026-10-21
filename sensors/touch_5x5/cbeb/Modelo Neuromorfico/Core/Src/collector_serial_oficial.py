import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import re

# =========================================================
# CONFIGURAÇÕES
# =========================================================

PORT = 'COM7'
BAUD = 115200

ROWS = 4
COLS = 4
NUM_TAXELS = ROWS * COLS

VREF = 3.3

RASTER_WINDOW = 5.0
WINDOW_SIZE = 300

ser = serial.Serial(PORT, BAUD, timeout=0.1)

# =========================================================
# BUFFER SERIAL
# =========================================================

serial_buffer = ""

# =========================================================
# DADOS
# =========================================================

voltage_matrix = np.zeros((ROWS, COLS))

current_time = 0.0

# Corrente do neurônio cuneiforme
I_final_data = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)

# spikes do neurônio cuneiforme
spike_times_CN = []

# =========================================================
# FIGURA
# =========================================================

fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))

# =========================================================
# HEATMAP DOS TAXELS
# =========================================================

im_volt = ax1.imshow(
    voltage_matrix,
    cmap="jet",
    interpolation="bicubic",
    vmin=0,
    vmax=VREF
)

plt.colorbar(im_volt, ax=ax1)

ax1.set_title("Taxels (0 - 3.3V)")
ax1.set_xticks(range(COLS))
ax1.set_yticks(range(ROWS))

texts_volt = [
    [
        ax1.text(
            c,
            r,
            "0",
            ha="center",
            va="center",
            fontsize=8,
            color="white"
        )
        for c in range(COLS)
    ]
    for r in range(ROWS)
]

# =========================================================
# CORRENTE DO CUNEIFORME
# =========================================================

x_fixed = np.arange(WINDOW_SIZE)

line_I, = ax2.plot(
    x_fixed,
    I_final_data,
    lw=2
)

ax2.set_title("Corrente Final do Cuneiforme")
ax2.set_xlim(0, WINDOW_SIZE)

# ajuste conforme necessário
ax2.set_ylim(-300, 300)

# =========================================================
# SPIKES DO CUNEIFORME
# =========================================================

ax3.set_title("Spikes do Cuneiforme")

ax3.set_xlim(0, RASTER_WINDOW)
ax3.set_ylim(-1, 1)

scatter_CN = ax3.scatter(
    [],
    [],
    s=20,
    color="black"
)

# =========================================================
# UPDATE
# =========================================================

def update(frame):

    global serial_buffer
    global current_time
    global voltage_matrix

    # =====================================================
    # LEITURA SERIAL
    # =====================================================

    if ser.in_waiting:

        serial_buffer += ser.read(
            ser.in_waiting
        ).decode(errors='ignore')

        lines = serial_buffer.split("\n")

        serial_buffer = lines[-1]

        for line in lines[:-1]:

            line = line.strip()

            # =================================================
            # ADC DOS TAXELS
            # =================================================

            if line.startswith("ADC"):

                try:

                    values = line.split(",")

                    adc_values = []

                    for i in range(1, 17):
                        adc_values.append(int(values[i]))

                    tstamp_str = values[-1]

                    tstamp = int(
                        tstamp_str.split("=")[1]
                    ) / 1e6

                    current_time = tstamp

                    for idx in range(NUM_TAXELS):

                        row = idx // COLS
                        col = idx % COLS

                        voltage_matrix[row, col] = (
                            adc_values[idx] * VREF / 4095.0
                        )

                except:
                    pass

            # =================================================
            # CORRENTE FINAL
            # =================================================

            elif line.startswith("CN"):

                m = re.search(
                    r"t=(\d+),Ifinal=([0-9.\-eE]+)",
                    line
                )

                if m:

                    tstamp = int(m.group(1)) / 1e6

                    I_final = float(m.group(2))

                    current_time = tstamp

                    I_final_data.append(I_final)

            # =================================================
            # SPIKE DO CUNEIFORME
            # =================================================

            elif line.startswith("SPIKE_CN"):

                m = re.search(
                    r"t=(\d+)",
                    line
                )

                if m:

                    tstamp = int(m.group(1)) / 1e6

                    spike_times_CN.append(tstamp)

    # =====================================================
    # HEATMAP
    # =====================================================

    im_volt.set_data(np.rot90(voltage_matrix, 2))

    for r in range(ROWS):
        for c in range(COLS):

            texts_volt[r][c].set_text(
                f"{voltage_matrix[r,c]:.2f}"
            )

    # =====================================================
    # CORRENTE
    # =====================================================

    line_I.set_data(
        x_fixed,
        I_final_data
    )

    # =====================================================
    # SPIKES
    # =====================================================

    spike_times_CN[:] = [
        t for t in spike_times_CN
        if current_time - t <= RASTER_WINDOW
    ]

    scatter_CN.set_offsets(
        np.c_[
            spike_times_CN,
            [0]*len(spike_times_CN)
        ]
    )

    ax3.set_xlim(
        max(0, current_time - RASTER_WINDOW),
        current_time
    )

    return [
        im_volt,
        line_I,
        scatter_CN
    ]

# =========================================================
# ANIMAÇÃO
# =========================================================

ani = FuncAnimation(
    fig,
    update,
    interval=50
)

plt.tight_layout()
plt.show()