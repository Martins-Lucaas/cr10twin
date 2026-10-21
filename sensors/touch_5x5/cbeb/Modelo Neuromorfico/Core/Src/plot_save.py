import numpy as np
import matplotlib.pyplot as plt

# =========================================================
# 1. CARREGAR DADOS
# =========================================================
file_path = r"c:\Users\Gabi\OneDrive\Área de Trabalho\Dados\adc_20260529_101807.csv"

data = np.loadtxt(file_path, delimiter=",", skiprows=1)

t = data[:, 0]
adc = data[:, 1:]   # 16 taxels

print("Shape ADC:", adc.shape)  # (N, 16)

# =========================================================
# 2. CONVERSÃO ADC -> CORRENTE
# =========================================================
Vref = 3.3
Rshunt = 15  # ajuste se necessário

voltage = (adc / 4095) * Vref
current = voltage / Rshunt   # (N, 16)

print("Shape corrente:", current.shape)

# =========================================================
# 3. PLOT DAS 16 CORRENTES (UMA LINHA POR TAXEL)
# =========================================================
plt.figure(figsize=(12, 6))

for i in range(16):
    plt.plot(t, current[:, i], label=f"I_taxel_{i}", alpha=0.7)

plt.title("Corrente dos 16 Taxels")
plt.xlabel("Tempo")
plt.ylabel("Corrente (A)")
plt.grid()
plt.legend(ncol=4, fontsize=8)
#plt.show()

# =========================================================
# 4. MÉDIA DAS CORRENTES (SINAL GLOBAL DO SENSOR)
# =========================================================
current_mean = np.mean(current, axis=1)

plt.figure(figsize=(10, 4))

plt.plot(t, current_mean, color="black", linewidth=2, label="Corrente média")

plt.title("Corrente média dos 16 Taxels")
plt.xlabel("Tempo")
plt.ylabel("Corrente (A)")
plt.grid()
plt.legend()
plt.show()