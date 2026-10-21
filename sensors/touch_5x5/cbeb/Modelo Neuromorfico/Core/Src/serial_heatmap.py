import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import re

# ================= CONFIG ===================

PORT = 'COM7'
BAUD = 115200

ROWS, COLS = 4, 4
NUM_TAXELS = ROWS * COLS

VREF = 3.3
RASTER_WINDOW = 5.0

WINDOW_SIZE = 200

ser = serial.Serial(PORT, BAUD, timeout=0.1)

# ================= BUFFER SERIAL ===================

serial_buffer = ""

# ================= DADOS ===================

spike_times_RA = [[] for _ in range(NUM_TAXELS)]
spike_times_SA = [[] for _ in range(NUM_TAXELS)]
spike_times_POST = []

voltage_matrix = np.zeros((ROWS, COLS))

current_time = 0.0

I_total_data = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)

# ================= FIGURA ===================

fig, (ax1, ax2, ax3, ax4) = plt.subplots(1, 4, figsize=(22, 6))

# ===== HEATMAP =====

im_volt = ax1.imshow(voltage_matrix, cmap="jet", interpolation="bicubic", vmin=0, vmax=VREF)
plt.colorbar(im_volt, ax=ax1)

ax1.set_title("Voltage (0–3.3V)")
ax1.set_xticks(range(COLS))
ax1.set_yticks(range(ROWS))

texts_volt = [[ax1.text(c, r, "0", ha="center", va="center", fontsize=8)
               for c in range(COLS)] for r in range(ROWS)]

# ===== RASTER =====

ax2.set_title("Raster RA / SA")
ax2.set_xlim(0, RASTER_WINDOW)
ax2.set_ylim(-1, NUM_TAXELS * 2)

scatter_RA = ax2.scatter([], [], s=10, color="red", label="RA")
scatter_SA = ax2.scatter([], [], s=10, color="blue", label="SA")

ax2.legend()

# ===== CORRENTE TOTAL =====

x_fixed = np.arange(WINDOW_SIZE)
line_I_total, = ax3.plot(x_fixed, I_total_data, lw=2, label="I_total")

ax3.set_title("Corrente Sináptica Total")
ax3.set_xlim(0, WINDOW_SIZE)
ax3.set_ylim(-80, 80)  # ✅ EIXO FIXO
ax3.legend()

# ===== POST =====

ax4.set_title("Neuronio 2")
ax4.set_xlim(0, RASTER_WINDOW)
ax4.set_ylim(-1, 1)

scatter_POST = ax4.scatter([], [], s=10, color="black")

# ================= UPDATE ===================

def update(frame):

    global current_time, voltage_matrix, serial_buffer

    if ser.in_waiting:
        serial_buffer += ser.read(ser.in_waiting).decode(errors='ignore')

        lines = serial_buffer.split("\n")
        serial_buffer = lines[-1]  # mantém pedaço incompleto

        for line in lines[:-1]:
            line = line.strip()

            # -------- DATA --------
            if line.startswith("DATA"):

                m = re.search(r"idx=(\d+),adc=(\d+),t=(\d+)", line)
                if m:
                    idx = int(m.group(1))
                    adc = int(m.group(2))
                    tstamp = int(m.group(3)) / 1e6

                    current_time = tstamp

                    row, col = divmod(idx, COLS)
                    voltage_matrix[row, col] = adc * (VREF / 4095.0)

            # -------- RA --------
            elif line.startswith("RA"):

                m = re.search(r"idx=(\d+),adc=\d+,t=(\d+)", line)
                if m:
                    idx = int(m.group(1))
                    tstamp = int(m.group(2)) / 1e6
                    spike_times_RA[idx].append(tstamp)

            # -------- SA --------
            elif line.startswith("SA"):

                m = re.search(r"idx=(\d+),adc=\d+,t=(\d+)", line)
                if m:
                    idx = int(m.group(1))
                    tstamp = int(m.group(2)) / 1e6
                    spike_times_SA[idx].append(tstamp)

            # -------- POST --------
            elif line.startswith("POST"):

                m = re.search(r"t=(\d+)", line)
                if m:
                    tstamp = int(m.group(1)) / 1e6
                    spike_times_POST.append(tstamp)

            # -------- CORRENTE TOTAL --------
            elif line.startswith("TOTAL"):

                m = re.search(r"I=([0-9.\-]+)", line)  # ✅ CORRIGIDO
                if m:
                    I_total = float(m.group(1))
                    I_total_data.append(I_total)

    # ===== HEATMAP =====
    im_volt.set_data(np.rot90(voltage_matrix, 2))

    for r in range(ROWS):
        for c in range(COLS):
            texts_volt[r][c].set_text(f"{voltage_matrix[r,c]:.2f}")

    # ===== RASTER RA =====
    x_RA, y_RA = [], []

    for n in range(NUM_TAXELS):
        spike_times_RA[n] = [t for t in spike_times_RA[n]
                            if current_time - t <= RASTER_WINDOW]

        for t in spike_times_RA[n]:
            x_RA.append(t)
            y_RA.append(n)

    # ===== RASTER SA =====
    x_SA, y_SA = [], []

    for n in range(NUM_TAXELS):
        spike_times_SA[n] = [t for t in spike_times_SA[n]
                            if current_time - t <= RASTER_WINDOW]

        for t in spike_times_SA[n]:
            x_SA.append(t)
            y_SA.append(n + NUM_TAXELS)

    scatter_RA.set_offsets(np.c_[x_RA, y_RA])
    scatter_SA.set_offsets(np.c_[x_SA, y_SA])

    ax2.set_xlim(max(0, current_time - RASTER_WINDOW), current_time)

    # ===== POST =====
    spike_times_POST[:] = [t for t in spike_times_POST
                          if current_time - t <= RASTER_WINDOW]

    scatter_POST.set_offsets(np.c_[spike_times_POST, [0]*len(spike_times_POST)])

    ax4.set_xlim(max(0, current_time - RASTER_WINDOW), current_time)

    # ===== CORRENTE =====
    line_I_total.set_data(x_fixed, I_total_data)

    return [im_volt, scatter_RA, scatter_SA, scatter_POST, line_I_total]


# ================= ANIMAÇÃO =================

ani = FuncAnimation(fig, update, interval=50)

plt.tight_layout()
plt.show()