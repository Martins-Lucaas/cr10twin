# =====================================================
# BIBLIOTECAS
# =====================================================

import numpy as np
import matplotlib.pyplot as plt
import glob
import os

# =====================================================
# ENCONTRA O PRIMEIRO EXPERIMENTO
# =====================================================

pasta = r"C:\Users\Gabi\OneDrive\Área de Trabalho\Dados"

arquivos_adc = sorted(
    glob.glob(
        os.path.join(pasta, "adc_*.csv")
    )
)

if len(arquivos_adc) == 0:
    raise FileNotFoundError("Nenhum arquivo adc_*.csv encontrado.")


adc_file = os.path.join(pasta, "adc_1.csv")
spike_file = os.path.join(pasta, "spike_1.csv")

nome = os.path.basename(adc_file)

numero = nome.replace("adc_", "")
numero = numero.replace(".csv", "")


print(f"ADC: {adc_file}")
print(f"SPIKES: {spike_file}")

# =====================================================
# CARREGA ADC
# =====================================================

data = np.loadtxt(
    adc_file,
    delimiter=",",
    skiprows=1
)

t = data[:, 0]

# 25 taxels (ou quantos existirem)
taxels = data[:, 1:]

n_taxels = taxels.shape[1]

print(f"Número de taxels = {n_taxels}")

# =====================================================
# SINAL MÉDIO
# =====================================================

sinal_medio = np.mean(
    taxels,
    axis=1
)

# =====================================================
# CARREGA SPIKES
# =====================================================

try:

    spike_data = np.loadtxt(
        spike_file,
        delimiter=",",
        skiprows=1,
        dtype=str
    )

    spike_data = np.atleast_2d(
        spike_data
    )

    t_spk = spike_data[:, 0].astype(float)
    tipo = spike_data[:, 1]

except Exception as e:

    print("Arquivo de spikes vazio ou inexistente.")

    t_spk = np.array([])
    tipo = np.array([])

# =====================================================
# SEPARA RA E SA
# =====================================================

ra_times = t_spk[tipo == "RA"]
sa_times = t_spk[tipo == "SA"]

# =====================================================
# FIGURA
# =====================================================

fig, axs = plt.subplots(
    3,
    1,
    figsize=(12, 10),
    sharex=True
)

# =====================================================
# GRÁFICO 1
# TODOS OS TAXELS
# =====================================================

for i in range(n_taxels):

    axs[0].plot(
        t,
        taxels[:, i],
        linewidth=1,
        alpha=0.7,
        label=f"T{i+1}"
    )

axs[0].set_title(
    f"Correntes dos {n_taxels} Taxels"
)

axs[0].set_ylabel(
    "ADC"
)

axs[0].grid(True)

# se quiser legenda dos 25 taxels, descomente:
axs[0].legend(
     ncol=5,
    fontsize=7
)

# =====================================================
# GRÁFICO 2
# SINAL MÉDIO
# =====================================================

axs[1].plot(
    t,
    sinal_medio,
    linewidth=2
)

axs[1].set_title(
    "Sinal Médio dos Taxels"
    
)

axs[1].set_ylabel(
    "ADC Médio"
)

axs[1].grid(True)

# =====================================================
# GRÁFICO 3
# RASTER PLOT
# =====================================================

axs[2].eventplot(
    ra_times,
    lineoffsets=1,
    linelengths=0.8,
    colors='red',
    label='RA'
)

axs[2].eventplot(
    sa_times,
    lineoffsets=2,
    linelengths=0.8,
    colors='blue',
    label='SA'
)

axs[2].set_yticks(
    [1, 2]
)

axs[2].set_yticklabels(
    ["RA", "SA"]
)

axs[2].set_title(
    "Raster Plot"
)

axs[2].set_xlabel(
    "Tempo (s)"
)

axs[2].grid(True)

# =====================================================
# AJUSTES FINAIS
# =====================================================

plt.suptitle(
    f"Experimento {numero}",
    fontsize=10
)

plt.tight_layout()

plt.show()