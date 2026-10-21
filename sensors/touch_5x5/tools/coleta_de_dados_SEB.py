
import serial
import csv
import re
import time
import os
import tkinter as tk


# =========================================================
# CONFIGURAÇÃO
# =========================================================

PORT = "COM9"
BAUD = 115200

RECORD_TIME = 15.0

ROWS = 5
COLS = 5
NUM_TAXELS = ROWS * COLS


# =========================================================
# PROTOCOLO
# =========================================================
#
# 0 - 2 s:
#     NÃO TOQUE
#
# 2 - 13 s:
#     DÊ VÁRIAS BATIDAS NO SENSOR
#
# 13 - 15 s:
#     NÃO TOQUE
#
# =========================================================

INICIO_BATIDAS = 2.0
FIM_BATIDAS = 13.0


# =========================================================
# SALVAMENTO
# =========================================================

SAVE_FOLDER = (
    r"c:\Users\Gabi\OneDrive\Documentos\Artigo_SEB\DADOS_CARGA"
)

NOME_EXPERIMENTO = "23"

os.makedirs(
    SAVE_FOLDER,
    exist_ok=True
)


# =========================================================
# SERIAL
# =========================================================

ser = serial.Serial(
    PORT,
    BAUD,
    timeout=0.05
)

ser.reset_input_buffer()


# =========================================================
# INTERFACE
# =========================================================

root = tk.Tk()

root.title(
    "Experimento - Batidas no Sensor"
)

root.geometry(
    "750x430"
)


label_titulo = tk.Label(
    root,
    text="PROTOCOLO EXPERIMENTAL - FA / SA",
    font=("Arial", 20, "bold")
)

label_titulo.pack(
    pady=(20, 10)
)


label_status = tk.Label(
    root,
    text="NÃO TOQUE",
    font=("Arial", 28, "bold"),
    bg="gray",
    fg="white",
    width=28,
    height=2
)

label_status.pack(
    pady=15
)


label_timer = tk.Label(
    root,
    text="0.0 s",
    font=("Arial", 42, "bold")
)

label_timer.pack(
    pady=10
)


label_info = tk.Label(
    root,
    text=(
        "0–2 s: repouso\n"
        "2–13 s: dê várias batidas no sensor\n"
        "13–15 s: repouso"
    ),
    font=("Arial", 15)
)

label_info.pack(
    pady=10
)


root.update()


# =========================================================
# ARQUIVOS
# =========================================================

adc_file = open(
    os.path.join(
        SAVE_FOLDER,
        f"{NOME_EXPERIMENTO}_adc.csv"
    ),
    "w",
    newline=""
)


force_file = open(
    os.path.join(
        SAVE_FOLDER,
        f"{NOME_EXPERIMENTO}_forca.csv"
    ),
    "w",
    newline=""
)


spike_file = open(
    os.path.join(
        SAVE_FOLDER,
        f"{NOME_EXPERIMENTO}_spikes.csv"
    ),
    "w",
    newline=""
)


adc_writer = csv.writer(
    adc_file
)

force_writer = csv.writer(
    force_file
)

spike_writer = csv.writer(
    spike_file
)


# =========================================================
# CABEÇALHOS
# =========================================================

adc_writer.writerow(
    [
        "tempo",
        *[
            f"taxel_{i}"
            for i in range(NUM_TAXELS)
        ]
    ]
)


force_writer.writerow(
    [
        "tempo",
        "forca",
        "forca_filtrada"
    ]
)


spike_writer.writerow(
    [
        "tempo",
        "tipo",
        "idx",
        "adc"
    ]
)


# =========================================================
# INÍCIO
# =========================================================

serial_buffer = ""

start_time = time.time()


print()
print("=" * 60)
print("EXPERIMENTO - BATIDAS NO SENSOR")
print("=" * 60)

print("Tempo total: 15 s")
print("0-2 s   -> NÃO TOQUE")
print("2-13 s  -> BATIDAS")
print("13-15 s -> NÃO TOQUE")

print()
print("Salvando somente:")
print("ADC")
print("FORÇA")
print("FA")
print("SA")

print()
print("Pasta:")
print(SAVE_FOLDER)

print("=" * 60)


# =========================================================
# AQUISIÇÃO
# =========================================================

while (
    time.time() - start_time
) < RECORD_TIME:


    elapsed = (
        time.time()
        - start_time
    )


    # =====================================================
    # TIMER
    # =====================================================

    label_timer.config(
        text=f"{elapsed:.1f} s"
    )


    # =====================================================
    # PROTOCOLO
    # =====================================================

    if elapsed < INICIO_BATIDAS:

        label_status.config(
            text="NÃO TOQUE",
            bg="gray",
            fg="white"
        )


    elif elapsed < FIM_BATIDAS:

        label_status.config(
            text="DÊ BATIDAS NO SENSOR",
            bg="green",
            fg="white"
        )


    else:

        label_status.config(
            text="NÃO TOQUE",
            bg="gray",
            fg="white"
        )


    root.update()


    # =====================================================
    # SERIAL
    # =====================================================

    if ser.in_waiting > 0:


        dados_recebidos = ser.read(
            ser.in_waiting
        ).decode(
            errors="ignore"
        )


        serial_buffer += dados_recebidos


        linhas = serial_buffer.split(
            "\n"
        )


        serial_buffer = linhas[-1]


        # =================================================
        # PROCESSAR LINHAS
        # =================================================

        for line in linhas[:-1]:


            line = line.strip()


            if line == "":

                continue


            # =================================================
            # ADC
            # =================================================

            if line.startswith("ADC,"):


                try:


                    valores = line.split(",")


                    if len(valores) != NUM_TAXELS + 2:

                        continue


                    adc_values = [
                        int(valores[i])
                        for i in range(
                            1,
                            NUM_TAXELS + 1
                        )
                    ]


                    tstamp = int(
                        valores[
                            NUM_TAXELS + 1
                        ].replace(
                            "t=",
                            ""
                        )
                    ) / 1e6


                    adc_writer.writerow(
                        [
                            tstamp,
                            *adc_values
                        ]
                    )


                except Exception as e:

                    print(
                        "Erro ADC:",
                        e
                    )


            # =================================================
            # FORÇA
            # =================================================

            elif line.startswith("FORCE,"):


                try:


                    valores = line.split(",")


                    if len(valores) != 4:

                        continue


                    forca = float(
                        valores[1]
                    )


                    forca_filtrada = float(
                        valores[2]
                    )


                    tstamp = int(
                        valores[3].replace(
                            "t=",
                            ""
                        )
                    ) / 1e6


                    force_writer.writerow(
                        [
                            tstamp,
                            forca,
                            forca_filtrada
                        ]
                    )


                except Exception as e:

                    print(
                        "Erro FORCE:",
                        e
                    )


            # =================================================
            # FA
            #
            # STM32 envia RA.
            # CSV salva FA.
            # =================================================

            elif line.startswith("RA,"):


                m = re.search(
                    r"idx=(\d+),adc=(\d+),t=(\d+)",
                    line
                )


                if m:


                    idx = int(
                        m.group(1)
                    )


                    adc = int(
                        m.group(2)
                    )


                    tstamp = int(
                        m.group(3)
                    ) / 1e6


                    spike_writer.writerow(
                        [
                            tstamp,
                            "FA",
                            idx,
                            adc
                        ]
                    )


            # =================================================
            # SA
            # =================================================

            elif line.startswith("SA,"):


                m = re.search(
                    r"idx=(\d+),adc=(\d+),t=(\d+)",
                    line
                )


                if m:


                    idx = int(
                        m.group(1)
                    )


                    adc = int(
                        m.group(2)
                    )


                    tstamp = int(
                        m.group(3)
                    ) / 1e6


                    spike_writer.writerow(
                        [
                            tstamp,
                            "SA",
                            idx,
                            adc
                        ]
                    )


            # =================================================
            # CN_MM / CN_RA / CN_SA
            #
            # IGNORADOS.
            # =================================================


# =========================================================
# FINALIZAÇÃO
# =========================================================

label_status.config(
    text="FINALIZADO",
    bg="blue",
    fg="white"
)

label_timer.config(
    text="15.0 s"
)

root.update()


adc_file.close()
force_file.close()
spike_file.close()

ser.close()


print()
print("=" * 60)
print("COLETA FINALIZADA")
print("=" * 60)

print(
    f"{NOME_EXPERIMENTO}_adc.csv"
)

print(
    f"{NOME_EXPERIMENTO}_forca.csv"
)

print(
    f"{NOME_EXPERIMENTO}_spikes.csv"
)

print()

print(
    "Nenhum cuneiforme foi armazenado."
)

print("=" * 60)


root.after(
    1500,
    root.destroy
)

root.mainloop()
