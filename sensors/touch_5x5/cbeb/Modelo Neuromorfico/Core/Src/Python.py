import serial
import csv
import re
import time
import os
from datetime import datetime

# =========================================================
# CONFIGURAÇÕES
# =========================================================

PORT = 'COM7'
BAUD = 115200
RECORD_TIME = 10

SAVE_FOLDER = r"C:\Users\Gabi\OneDrive\Área de Trabalho\Dados"

ser = serial.Serial(PORT, BAUD, timeout=0.1)

os.makedirs(SAVE_FOLDER, exist_ok=True)

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

# =========================================================
# ARQUIVOS
# =========================================================

adc_file = open(os.path.join(SAVE_FOLDER, f"adc_{timestamp}.csv"), "w", newline="")
cn_file = open(os.path.join(SAVE_FOLDER, f"cn_{timestamp}.csv"), "w", newline="")
spike_file = open(os.path.join(SAVE_FOLDER, f"spike_{timestamp}.csv"), "w", newline="")

adc_writer = csv.writer(adc_file)
cn_writer = csv.writer(cn_file)
spike_writer = csv.writer(spike_file)

# =========================================================
# HEADERS
# =========================================================

adc_writer.writerow(["t"] + [f"taxel_{i}" for i in range(16)])
cn_writer.writerow(["t", "Ifinal"])
spike_writer.writerow(["t_spike"])

# =========================================================
# SERIAL BUFFER
# =========================================================

serial_buffer = ""

start_time = time.time()

print(f"Gravando por {RECORD_TIME} segundos...")
print(f"Pasta: {SAVE_FOLDER}")

# =========================================================
# LOOP
# =========================================================

while (time.time() - start_time) < RECORD_TIME:

    if ser.in_waiting:

        serial_buffer += ser.read(ser.in_waiting).decode(errors='ignore')
        lines = serial_buffer.split("\n")
        serial_buffer = lines[-1]

        for line in lines[:-1]:
            line = line.strip()

            # =========================
            # ADC
            # =========================
            if line.startswith("ADC"):
                try:
                    values = line.split(",")

                    adc_values = [int(values[i]) for i in range(1, 17)]

                    tstamp = int(values[-1].split("=")[1]) / 1e6

                    adc_writer.writerow([tstamp] + adc_values)

                except:
                    pass

            # =========================
            # CN
            # =========================
            elif line.startswith("CN"):
                m = re.search(r"t=(\d+),Ifinal=([0-9.\-eE]+)", line)
                if m:
                    t = int(m.group(1)) / 1e6
                    I = float(m.group(2))
                    cn_writer.writerow([t, I])

            # =========================
            # SPIKE
            # =========================
            elif line.startswith("SPIKE_CN"):
                m = re.search(r"t=(\d+)", line)
                if m:
                    t = int(m.group(1)) / 1e6
                    spike_writer.writerow([t])

# =========================================================
# FECHAR ARQUIVOS
# =========================================================

adc_file.close()
cn_file.close()
spike_file.close()
ser.close()

print("Gravação finalizada com sucesso.")