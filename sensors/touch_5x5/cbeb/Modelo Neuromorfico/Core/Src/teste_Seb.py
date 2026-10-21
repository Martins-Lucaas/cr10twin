# =====================================================
# BIBLIOTECAS
# =====================================================

import numpy as np
import matplotlib.pyplot as plt
import glob
import os

# =====================================================
# LISTAS GLOBAIS
# =====================================================

forca_lista = []
freq_RA_lista = []
freq_SA_lista = []
labels_lista = []

# =====================================================
# DETECÇÃO DE PRESS E RELEASE
# =====================================================

def detectar_evento(signal):

    dI = np.diff(signal)

    thr = 4 * np.std(dI)

    print("\n===== DIAGNÓSTICO =====")
    print("STD derivada =", np.std(dI))
    print("Limiar =", thr)
    print("Min derivada =", np.min(dI))
    print("Max derivada =", np.max(dI))

    press = np.where(dI < -thr)[0]
    release = np.where(dI > thr)[0]

    print("PRESS encontrados =", len(press))
    print("RELEASE encontrados =", len(release))

    if len(press) == 0:
        return None, None

    i_press = press[0]

    release_after = release[release > i_press]

    if len(release_after) == 0:
        return None, None

    i_release = release_after[0]

    return i_press, i_release

# =====================================================
# ANÁLISE DE UM EXPERIMENTO
# =====================================================

def analisar_arquivo(adc_file, spike_file, label):

    data = np.loadtxt(
        adc_file,
        delimiter=",",
        skiprows=1
    )

    t = data[:, 0]
    taxels = data[:, 1:]

    signal = np.mean(taxels, axis=1)

    print("\n========================")
    print(label)
    print("========================")

    print("Min sinal =", np.min(signal))
    print("Max sinal =", np.max(signal))
    print("Número de amostras =", len(signal))

    i_press, i_release = detectar_evento(signal)

    if i_press is None:

        print(f"\n{label}: toque não encontrado")

        plt.title(label)
        plt.xlabel("Tempo (s)")
        plt.ylabel("ADC médio")
        plt.grid()
        plt.legend()
        plt.tight_layout()

        return

    baseline = np.mean(signal[:i_press])
    baseline_std = np.std(signal[:i_press])

    print("Baseline =", baseline)
    print("STD baseline =", baseline_std)

    valor_toque = np.mean(signal[i_press:i_release])

    forca = baseline - valor_toque

    tempo_contato = t[i_release] - t[i_press]

    # =================================================
    # SPIKES
    # =================================================

    try:

        spike_data = np.loadtxt(
            spike_file,
            delimiter=",",
            skiprows=1,
            dtype=str
        )

        spike_data = np.atleast_2d(spike_data)

        t_spk = spike_data[:,0].astype(float)
        tipo = spike_data[:,1]

    except:

        print("Arquivo de spikes vazio.")

        t_spk = np.array([])
        tipo = np.array([])

    mask = (
        (t_spk >= t[i_press]) &
        (t_spk <= t[i_release])
    )

    n_RA = np.sum((tipo == "RA") & mask)
    n_SA = np.sum((tipo == "SA") & mask)

    freq_RA = n_RA / tempo_contato if tempo_contato > 0 else 0
    freq_SA = n_SA / tempo_contato if tempo_contato > 0 else 0

    forca_lista.append(forca)
    freq_RA_lista.append(freq_RA)
    freq_SA_lista.append(freq_SA)
    labels_lista.append(label)

    print("PRESS =", t[i_press], "s")
    print("RELEASE =", t[i_release], "s")
    print("Tempo contato =", tempo_contato, "s")
    print("Força =", forca)

    print("RA =", freq_RA, "Hz")
    print("SA =", freq_SA, "Hz")

    print("Spikes RA =", n_RA)
    print("Spikes SA =", n_SA)

    # =================================================
    # GRÁFICO
    # =================================================

    plt.figure(figsize=(10,4))

    plt.plot(
        t,
        signal,
        label="Média dos taxels"
    )

    plt.axvline(
        t[i_press],
        color='g',
        linestyle='--',
        label='PRESS'
    )

    plt.axvline(
        t[i_release],
        color='purple',
        linestyle='--',
        label='RELEASE'
    )

    plt.axhline(
        baseline,
        color='r',
        linestyle=':',
        label='Baseline'
    )

    plt.title(label)
    plt.xlabel("Tempo (s)")
    plt.ylabel("ADC médio")
    plt.grid()
    plt.legend()
    plt.show(block=False)

    

# =====================================================
# ENCONTRA TODOS OS EXPERIMENTOS
# =====================================================

arquivos_adc = sorted(
    glob.glob(r"C:\Users\Gabi\OneDrive\Área de Trabalho\Dados\adc_*.csv")
)

experimentos = []

for adc_file in arquivos_adc:

    nome = os.path.basename(adc_file)

    numero = nome.replace("adc_", "")
    numero = numero.replace(".csv", "")

    spike_file = rf"C:\Users\Gabi\OneDrive\Área de Trabalho\Dados\spike_{numero}.csv"

    if os.path.exists(spike_file):

        experimentos.append(
            (
                adc_file,
                spike_file,
                f"{numero}"
            )
        )

print("\nExperimentos encontrados:")
for e in experimentos:
    print(e[2])

# =====================================================
# PROCESSA TODOS
# =====================================================

for adc, spk, label in experimentos:

    analisar_arquivo(
        adc,
        spk,
        label
    )

# =====================================================
# GRÁFICO FINAL
# =====================================================

if len(forca_lista) > 0:

    ordem = np.argsort(forca_lista)

    forca_lista = np.array(forca_lista)[ordem]
    freq_RA_lista = np.array(freq_RA_lista)[ordem]
    freq_SA_lista = np.array(freq_SA_lista)[ordem]
    labels_lista = np.array(labels_lista)[ordem]

    plt.figure(figsize=(8,5))

    plt.plot(
        forca_lista,
        freq_RA_lista,
        'o-',
        linewidth=2,
        label='RA'
    )

    plt.plot(
        forca_lista,
        freq_SA_lista,
        's-',
        linewidth=2,
        label='SA'
    )

    for i in range(len(labels_lista)):

        plt.text(
            forca_lista[i],
            freq_SA_lista[i],
            labels_lista[i]
        )

    plt.xlabel("Força")
    plt.ylabel("Firing Rate (Hz)")
    plt.title("Força × Taxa de Disparo")

    plt.grid()
    plt.legend()
    plt.tight_layout()

else:

    print("\nNenhum experimento válido encontrado.")

# =====================================================
# MOSTRA TODOS OS GRÁFICOS DE UMA VEZ
# =====================================================

plt.show()