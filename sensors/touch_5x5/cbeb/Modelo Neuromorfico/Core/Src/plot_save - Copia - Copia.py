import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# =========================================================
# ARQUIVOS (ajuste o nome se necessário)
# =========================================================

adc_file = r"c:\Users\Gabi\OneDrive\Área de Trabalho\Dados\adc_20260529_090950.csv"
spike_file = r"c:\Users\Gabi\OneDrive\Área de Trabalho\Dados\spike_20260529_090950.csv"

# =========================================================
# LEITURA DOS DADOS
# =========================================================

adc = pd.read_csv(adc_file)
spike = pd.read_csv(spike_file)

# =========================================================
# FORÇA MÉDIA (16 taxels)
# =========================================================

adc["force"] = adc.iloc[:, 1:17].mean(axis=1)

t_adc = adc["t"].values
force = adc["force"].values

# =========================================================
# SPIKES
# =========================================================

t_spike = spike["t_spike"].values

# =========================================================
# PARÂMETROS DE BINNING
# =========================================================

bin_size = 0.5  # segundos (ajuste se quiser mais resolução)

t_min = min(t_adc.min(), t_spike.min())
t_max = max(t_adc.max(), t_spike.max())

bins = np.arange(t_min, t_max + bin_size, bin_size)

# =========================================================
# FIRING RATE (spikes por janela)
# =========================================================

spike_count, _ = np.histogram(t_spike, bins=bins)
firing_rate = spike_count / bin_size

time_bins = bins[:-1]

# =========================================================
# FORÇA MÉDIA POR JANELA
# =========================================================

force_binned = []

for i in range(len(bins) - 1):
    mask = (t_adc >= bins[i]) & (t_adc < bins[i+1])
    
    if np.any(mask):
        force_binned.append(np.mean(force[mask]))
    else:
        force_binned.append(np.nan)

force_binned = np.array(force_binned)

# =========================================================
# LIMPEZA (remove NaNs)
# =========================================================

valid = ~np.isnan(force_binned)

force_binned = force_binned[valid]
firing_rate = firing_rate[valid]

# =========================================================
# NORMALIZAÇÃO (opcional, deixa cara de paper)
# =========================================================

force_norm = (force_binned - np.min(force_binned)) / (np.max(force_binned) - np.min(force_binned) + 1e-9)
fr_norm = (firing_rate - np.min(firing_rate)) / (np.max(firing_rate) - np.min(firing_rate) + 1e-9)

# =========================================================
# CURVA AJUSTADA (tipo saturação neural)
# =========================================================

z = np.polyfit(force_norm, fr_norm, 3)
p = np.poly1d(z)

x = np.linspace(0, 1, 200)

# =========================================================
# PLOT FINAL (ARTIGO)
# =========================================================

plt.figure(figsize=(8,5))

plt.scatter(force_norm, fr_norm, label="Dados experimentais")
plt.plot(x, p(x), 'r', label="Ajuste (curva neural)")

plt.xlabel("Força normalizada")
plt.ylabel("Taxa de disparo normalizada (Hz)")
plt.title("Relação Força vs Firing Rate")
plt.grid()
plt.legend()

plt.show()