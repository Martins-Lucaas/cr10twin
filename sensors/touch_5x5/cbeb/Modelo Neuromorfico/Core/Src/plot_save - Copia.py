import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re
import time

# ================= CONFIG ===================
PORT = 'COM9'           # porta serial do STM32
BAUD = 115200           # baudrate
ROWS, COLS = 5, 5
NUM_TAXELS = ROWS * COLS
VREF = 3.3              # tensão de referência ADC
RASTER_WINDOW = 5.0     # janela visível do raster (s)

ser = serial.Serial(PORT, BAUD, timeout=0.1)

# ================= MATRIZES =================
spike_times = [[] for _ in range(NUM_TAXELS)]
voltage_matrix = np.zeros((ROWS, COLS))

# ================= FIGURA =================
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
ax2.set_title("Raster Plot")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Taxel")
ax2.set_ylim(-1, NUM_TAXELS)
ax2.set_xlim(0, RASTER_WINDOW)
raster_scatter = ax2.scatter([], [], marker='|', s=100)

start_time = time.time()

# ================= UPDATE =================
def update(frame):
    global voltage_matrix
    now = time.time()
    elapsed = now - start_time

    # ===== LEITURA SERIAL =====
    if ser.in_waiting:
        data = ser.read(ser.in_waiting).decode(errors='ignore')
        lines = data.splitlines()
        for line in lines:
            line = line.strip()

            # ---- Dados contínuos ADC ----
            if line.startswith("DATA"):
                match = re.search(r"idx=(\d+),adc=(\d+)", line)
                if match:
                    idx = int(match.group(1))
                    adc = int(match.group(2))
                    row, col = divmod(idx, COLS)

                    # atualiza voltagem contínua
                    voltage_matrix[row, col] = adc * (VREF / 4095.0)

            # ---- SPIKES IZHICKEVICH ----
            elif line.startswith("SPIKE"):
                match = re.search(r"idx=(\d+),adc=(\d+),t=(\d+)", line)
                if match:
                    idx = int(match.group(1))
                    # opcional: adc = int(match.group(2))
                    spike_times[idx].append(elapsed)

    # ===== ATUALIZA HEATMAP VOLTAGEM ========
    im_volt.set_data(voltage_matrix)
    im_volt.set_clim(0, VREF)
    for r in range(ROWS):
        for c in range(COLS):
            texts_volt[r][c].set_text(f"{voltage_matrix[r,c]:.2f}")

    # ===== ATUALIZA RASTER PLOT =====
    raster_x = []
    raster_y = []
    for neuron in range(NUM_TAXELS):
        # mantém spikes dentro da janela de visualização
        spike_times[neuron] = [t for t in spike_times[neuron] if elapsed - t <= RASTER_WINDOW]
        for t in spike_times[neuron]:
            raster_x.append(t)
            raster_y.append(neuron)

    raster_scatter.set_offsets(np.c_[raster_x, raster_y])
    ax2.set_xlim(max(0, elapsed - RASTER_WINDOW), elapsed)

    return [im_volt, raster_scatter]

# ================= ANIMAÇÃO =================
ani = FuncAnimation(fig, update, interval=50, blit=False)
plt.tight_layout()
plt.show()
