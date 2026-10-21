import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re

# ================= CONFIG ===================

PORT = 'COM12'
BAUD = 115200

ROWS, COLS = 5, 5
NUM_TAXELS = ROWS * COLS

VREF = 3.3
RASTER_WINDOW = 5.0

ser = serial.Serial(PORT, BAUD, timeout=0.1)

# ================= MATRIZES =================

spike_times_RA = [[] for _ in range(NUM_TAXELS)]
spike_times_SA = [[] for _ in range(NUM_TAXELS)]

voltage_matrix = np.zeros((ROWS, COLS))

current_time = 0

# ================= FIGURA ===================

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12,6))

# ===== HEATMAP VOLTAGEM =====

im_volt = ax1.imshow(voltage_matrix, cmap="jet", interpolation="bicubic")
cbar_volt = plt.colorbar(im_volt, ax=ax1)
cbar_volt.set_label("Voltage (V)")

ax1.set_title("Voltage (0–3.3V)")
ax1.set_xticks(range(COLS))
ax1.set_yticks(range(ROWS))

texts_volt = [[ax1.text(c, r, "0", ha="center", va="center", fontsize=8)
               for c in range(COLS)] for r in range(ROWS)]

# ===== RASTER PLOT =====

ax2.set_title("Raster Plot (Adaptação Lenta / Adaptação Rápida)")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Neuron")

ax2.set_ylim(-1, NUM_TAXELS*2)
ax2.set_xlim(0, RASTER_WINDOW)

scatter_RA = ax2.scatter([], [], marker='.', s=10, color="red", label="Adap. Rápida")
scatter_SA = ax2.scatter([], [], marker='.', s=10, color="blue", label="Adap. Lenta")

ax2.legend()

# ================= UPDATE =================

def update(frame):

    global voltage_matrix
    global current_time

    # ===== SERIAL =====

    if ser.in_waiting:

        data = ser.read(ser.in_waiting).decode(errors='ignore')
        lines = data.splitlines()

        for line in lines:

            line = line.strip()

            # -------- VOLTAGEM --------

            if line.startswith("DATA"):

                match = re.search(r"idx=(\d+),adc=(\d+),t=(\d+)", line)

                if match:

                    idx = int(match.group(1))
                    adc = int(match.group(2))
                    tstamp = int(match.group(3)) / 1e6

                    current_time = tstamp

                    row, col = divmod(idx, COLS)

                    voltage_matrix[row, col] = adc * (VREF / 4095.0)

            # -------- RAPID ADAPTATION --------

            elif line.startswith("RA"):

                match = re.search(r"idx=(\d+),adc=\d+,t=(\d+)", line)

                if match:

                    idx = int(match.group(1))
                    tstamp = int(match.group(2)) / 1e6

                    current_time = tstamp

                    spike_times_RA[idx].append(tstamp)

            # -------- SLOW ADAPTATION --------

            elif line.startswith("SA"):

                match = re.search(r"idx=(\d+),adc=\d+,t=(\d+)", line)

                if match:

                    idx = int(match.group(1))
                    tstamp = int(match.group(2)) / 1e6

                    current_time = tstamp

                    spike_times_SA[idx].append(tstamp)

    # ===== HEATMAP =====

    im_volt.set_data(voltage_matrix)
    im_volt.set_clim(0, VREF)

    for r in range(ROWS):
        for c in range(COLS):
            texts_volt[r][c].set_text(f"{voltage_matrix[r,c]:.2f}")

    # ===== RASTER RA =====

    raster_x_RA = []
    raster_y_RA = []

    for neuron in range(NUM_TAXELS):

        spike_times_RA[neuron] = [t for t in spike_times_RA[neuron]
                                 if current_time - t <= RASTER_WINDOW]

        for t in spike_times_RA[neuron]:

            raster_x_RA.append(t)
            raster_y_RA.append(neuron)

    # ===== RASTER SA =====

    raster_x_SA = []
    raster_y_SA = []

    for neuron in range(NUM_TAXELS):

        spike_times_SA[neuron] = [t for t in spike_times_SA[neuron]
                                 if current_time - t <= RASTER_WINDOW]

        for t in spike_times_SA[neuron]:

            raster_x_SA.append(t)
            raster_y_SA.append(neuron + NUM_TAXELS)

    scatter_RA.set_offsets(np.c_[raster_x_RA, raster_y_RA])
    scatter_SA.set_offsets(np.c_[raster_x_SA, raster_y_SA])

    ax2.set_xlim(max(0, current_time - RASTER_WINDOW), current_time)

    return [im_volt, scatter_RA, scatter_SA]


# ================= ANIMAÇÃO =================

ani = FuncAnimation(fig, update, interval=50, blit=False)

plt.tight_layout()
plt.show()