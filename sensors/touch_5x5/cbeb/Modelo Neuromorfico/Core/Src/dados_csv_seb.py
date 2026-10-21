import serial
import csv
import re
import time
import os
from datetime import datetime
import tkinter as tk

# =========================================================
# CONFIGURAÇÕES
# =========================================================
PORT = 'COM7'
BAUD = 115200
RECORD_TIME = 30

SAVE_FOLDER = r"C:\Users\Gabi\OneDrive\Área de Trabalho\Dados"

ser = serial.Serial(PORT, BAUD, timeout=0.1)

os.makedirs(SAVE_FOLDER, exist_ok=True)

timestamp = datetime.now().strftime("18")

# =========================================================
# JANELA VISUAL
# =========================================================
root = tk.Tk()
root.title("Experimento Tátil")
root.geometry("500x300")

label_status = tk.Label(
    root,
    text="AGUARDE",
    font=("Arial", 30, "bold"),
    bg="red",
    fg="white",
    width=20,
    height=2
)
label_status.pack(pady=20)

label_timer = tk.Label(
    root,
    text="0.0 s",
    font=("Arial", 40)
)
label_timer.pack()

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
spike_writer.writerow(["t", "type", "idx"])

# =========================================================
# BUFFER SERIAL
# =========================================================
serial_buffer = ""
start_time = time.time()

print(f"Gravando por {RECORD_TIME} segundos...")
print(f"Pasta: {SAVE_FOLDER}")

# =========================================================
# LOOP PRINCIPAL
# =========================================================
while (time.time() - start_time) < RECORD_TIME:

    tempo = time.time() - start_time

    # =====================================================
    # INTERFACE VISUAL
    # =====================================================
    label_timer.config(text=f"{tempo:.1f} s")

    if tempo < 10:

        label_status.config(
            text="AGUARDE",
            bg="red"
        )

    elif tempo < 20:

        label_status.config(
            text="APERTE O SENSOR",
            bg="green"
        )

    else:

        label_status.config(
            text="PARE DE APERTAR",
            bg="gray"
        )

    root.update()

    # =====================================================
    # LEITURA SERIAL
    # =====================================================
    if ser.in_waiting:

        serial_buffer += ser.read(ser.in_waiting).decode(errors='ignore')

        lines = serial_buffer.split("\n")
        serial_buffer = lines[-1]

        for line in lines[:-1]:

            line = line.strip()

            # =================================================
            # ADC
            # =================================================
            if line.startswith("ADC"):

                try:
                    values = line.split(",")

                    adc_values = [int(values[i]) for i in range(1, 17)]

                    tstamp = int(values[-1].split("=")[1]) / 1e6

                    adc_writer.writerow([tstamp] + adc_values)

                except:
                    pass

            # =================================================
            # CN
            # =================================================
            elif line.startswith("CN"):

                m = re.search(
                    r"t=(\d+),Ifinal=([0-9.\-eE]+)",
                    line
                )

                if m:

                    t = int(m.group(1)) / 1e6
                    I = float(m.group(2))

                    cn_writer.writerow([t, I])

            # =================================================
            # SPIKE RA
            # =================================================
            elif line.startswith("SPIKE_RA"):

                m = re.search(
                    r"idx=(\d+),t=(\d+)",
                    line
                )

                if m:

                    idx = int(m.group(1))
                    t = int(m.group(2)) / 1e6

                    spike_writer.writerow([t, "RA", idx])

            # =================================================
            # SPIKE SA
            # =================================================
            elif line.startswith("SPIKE_SA"):

                m = re.search(
                    r"idx=(\d+),t=(\d+)",
                    line
                )

                if m:

                    idx = int(m.group(1))
                    t = int(m.group(2)) / 1e6

                    spike_writer.writerow([t, "SA", idx])

            # =================================================
            # SPIKE CN
            # =================================================
            elif line.startswith("SPIKE_CN"):

                m = re.search(
                    r"t=(\d+)",
                    line
                )

                if m:

                    t = int(m.group(1)) / 1e6

                    spike_writer.writerow([t, "CN", -1])

# =========================================================
# FECHAR ARQUIVOS
# =========================================================
adc_file.close()
cn_file.close()
spike_file.close()

ser.close()

root.destroy()

print("Gravação finalizada com sucesso.")