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

RECORD_TIME = 15

ROWS = 5
COLS = 5
NUM_TAXELS = ROWS * COLS

SAVE_FOLDER = r"c:\Users\Gabi\OneDrive\Documentos\Artigo_CBEB\dados_carga"

NOME_EXPERIMENTO = "42"


os.makedirs(
    SAVE_FOLDER,
    exist_ok=True
)


# =========================================================
# CONEXÃO SERIAL
# =========================================================

ser = serial.Serial(
    PORT,
    BAUD,
    timeout=0.1
)


# =========================================================
# INTERFACE
# =========================================================

root = tk.Tk()

root.title(
    "Experimento Tátil"
)

root.geometry(
    "600x300"
)


label_status = tk.Label(
    root,
    text="SOLTO",
    font=("Arial", 28, "bold"),
    bg="red",
    fg="white",
    width=20
)

label_status.pack(
    pady=20
)


label_timer = tk.Label(
    root,
    text="0.0 s",
    font=("Arial", 40)
)

label_timer.pack()


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


cn_file = open(
    os.path.join(
        SAVE_FOLDER,
        f"{NOME_EXPERIMENTO}_cuneiformes.csv"
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


cn_writer = csv.writer(
    cn_file
)


# =========================================================
# CABEÇALHOS DOS ARQUIVOS
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


cn_writer.writerow(
    [
        "tempo",
        "tipo"
    ]
)


# =========================================================
# VARIÁVEIS DA SERIAL
# =========================================================

serial_buffer = ""

start_time = time.time()


print(
    "Gravando..."
)

print(
    "Pasta:",
    SAVE_FOLDER
)

print(
    "Experimento:",
    NOME_EXPERIMENTO
)


# =========================================================
# LOOP DE AQUISIÇÃO
# =========================================================

while (
    time.time() - start_time
) < RECORD_TIME:


    elapsed = (
        time.time()
        - start_time
    )


    # =====================================================
    # ATUALIZAÇÃO DO TIMER
    # =====================================================

    label_timer.config(
        text=f"{elapsed:.1f} s"
    )


    # =====================================================
    # 0 A 5 SEGUNDOS
    # SENSOR SOLTO
    # =====================================================

    if elapsed < 5:

        label_status.config(
            text="SOLTO",
            bg="red"
        )


    # =====================================================
    # 5 A 10 SEGUNDOS
    # APERTAR
    # =====================================================

    elif elapsed < 10:

        label_status.config(
            text="APERTE O SENSOR",
            bg="green"
        )


    # =====================================================
    # 10 A 15 SEGUNDOS
    # SOLTAR
    # =====================================================

    else:

        label_status.config(
            text="SOLTE O SENSOR",
            bg="gray"
        )


    root.update()


    # =====================================================
    # LEITURA DA SERIAL
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


        for line in linhas[:-1]:


            line = line.strip()


            if line == "":

                continue


            # =================================================
            # ADC
            #
            # ADC,valor1,...,valor25,t=tempo
            # =================================================

            if line.startswith(
                "ADC,"
            ):


                try:


                    valores = line.split(
                        ","
                    )


                    if len(
                        valores
                    ) != NUM_TAXELS + 2:


                        print(
                            "ADC inválido:",
                            line
                        )


                        continue


                    adc_values = []


                    for i in range(
                        1,
                        NUM_TAXELS + 1
                    ):


                        adc_values.append(
                            int(
                                valores[i]
                            )
                        )


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


                    print(
                        "Linha:",
                        line
                    )


            # =================================================
            # FORÇA
            #
            # FORCE,raw,filtered,t=tempo
            # =================================================

            elif line.startswith(
                "FORCE,"
            ):


                try:


                    valores = line.split(
                        ","
                    )


                    if len(
                        valores
                    ) != 4:


                        print(
                            "FORCE inválido:",
                            line
                        )


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


                    print(
                        "Linha:",
                        line
                    )


            # =================================================
            # SPIKE RA
            #
            # RA,idx=0,adc=1234,t=tempo
            # =================================================

            elif line.startswith(
                "RA,"
            ):


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
                            "RA",
                            idx,
                            adc
                        ]
                    )


            # =================================================
            # SPIKE SA
            #
            # SA,idx=0,adc=1234,t=tempo
            # =================================================

            elif line.startswith(
                "SA,"
            ):


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
            # CUNEIFORME MULTIMODAL
            #
            # CN_MM,t=tempo
            # =================================================

            elif line.startswith(
                "CN_MM,"
            ):


                m = re.search(
                    r"t=(\d+)",
                    line
                )


                if m:


                    tstamp = int(
                        m.group(1)
                    ) / 1e6


                    cn_writer.writerow(
                        [
                            tstamp,
                            "CN_MM"
                        ]
                    )


            # =================================================
            # CUNEIFORME RA
            #
            # CN_RA,t=tempo
            # =================================================

            elif line.startswith(
                "CN_RA,"
            ):


                m = re.search(
                    r"t=(\d+)",
                    line
                )


                if m:


                    tstamp = int(
                        m.group(1)
                    ) / 1e6


                    cn_writer.writerow(
                        [
                            tstamp,
                            "CN_RA"
                        ]
                    )


            # =================================================
            # CUNEIFORME SA
            #
            # CN_SA,t=tempo
            # =================================================

            elif line.startswith(
                "CN_SA,"
            ):


                m = re.search(
                    r"t=(\d+)",
                    line
                )


                if m:


                    tstamp = int(
                        m.group(1)
                    ) / 1e6


                    cn_writer.writerow(
                        [
                            tstamp,
                            "CN_SA"
                        ]
                    )


# =========================================================
# FECHAMENTO DOS ARQUIVOS
# =========================================================

adc_file.close()

force_file.close()

spike_file.close()

cn_file.close()

ser.close()

root.destroy()


print(
    "Gravação concluída."
)


print(
    "Arquivos salvos em:"
)


print(
    SAVE_FOLDER
)