import numpy as np
import matplotlib.pyplot as plt

forca_lista = []
freq_RA_lista = []
freq_SA_lista = []
labels_lista = []

# =========================
# FUNÇÃO
# =========================
def analisar_arquivo(adc_file, spike_file, label=""):

    # =========================
    # ADC
    # =========================
    data = np.loadtxt(adc_file, delimiter=",", skiprows=1)

    t = data[:, 0]
    signal = data[:, 1:].mean(axis=1)

    # =========================
    # DETECÇÃO PRESS / RELEASE
    # =========================
    dI = np.diff(signal)
    thr = np.std(dI) * 3

    press = np.where(dI < -thr)[0]
    release = np.where(dI > thr)[0]

    if len(press) == 0 or len(release) == 0:
        print(f"[{label}] evento inválido")
        return

    i_press = press[0]

    release_after = release[release > i_press]
    if len(release_after) == 0:
        print(f"[{label}] sem release após press")
        return

    i_release = release_after[0]

    # =========================
    # FORÇA CORRIGIDA (ADC INVERTIDO)
    # =========================
    baseline = np.mean(signal[:i_press])
    forca = baseline - np.mean(signal[i_press:i_release])

    tempo = t[i_release] - t[i_press]

    # =========================
    # SPIKES
    # =========================
    spike_data = np.loadtxt(spike_file, delimiter=",", skiprows=1, dtype=str)

    t_spk = spike_data[:, 0].astype(float)
    types = spike_data[:, 1]

    mask = (t_spk >= t[i_press]) & (t_spk <= t[i_release])

    n_RA = np.sum((types == "RA") & mask)
    n_SA = np.sum((types == "SA") & mask)

    freq_RA = n_RA / tempo if tempo > 0 else 0
    freq_SA = n_SA / tempo if tempo > 0 else 0

    # =========================
    # SALVAR
    # =========================
    forca_lista.append(forca)
    freq_RA_lista.append(freq_RA)
    freq_SA_lista.append(freq_SA)
    labels_lista.append(label)

    # =========================
    # PRINT
    # =========================
    print(f"\n===== {label} =====")
    print("Força corrigida:", forca)
    print("RA (Hz):", freq_RA)
    print("SA (Hz):", freq_SA)

    # =========================
    # PLOT SINAL
    # =========================
    plt.figure(figsize=(8,3))
    plt.plot(t, signal)
    plt.axvline(t[i_press], color='r')
    plt.axvline(t[i_release], color='purple')
    plt.title(label)
    plt.show()


# =========================
# EXPERIMENTOS
# =========================
experimentos = [
    (r"c:\dados\adc_1.csv", r"c:\dados\spike_1.csv", "Leve"),
    (r"c:\dados\adc_2.csv", r"c:\dados\spike_2.csv", "Médio"),
    (r"c:\dados\adc_3.csv", r"c:\dados\spike_3.csv", "Forte"),
]

for adc, spk, label in experimentos:
    analisar_arquivo(adc, spk, label)

# =========================
# ORDENA
# =========================
order = np.argsort(forca_lista)

forca_lista = np.array(forca_lista)[order]
freq_RA_lista = np.array(freq_RA_lista)[order]
freq_SA_lista = np.array(freq_SA_lista)[order]
labels_lista = np.array(labels_lista)[order]

# =========================
# GRÁFICO FINAL
# =========================
plt.figure(figsize=(7,5))

plt.plot(forca_lista, freq_RA_lista, 'o-', label="RA")
plt.plot(forca_lista, freq_SA_lista, 's-', label="SA")

for i, txt in enumerate(labels_lista):
    plt.text(forca_lista[i], freq_SA_lista[i], txt)

plt.xlabel("Força (corrigida)")
plt.ylabel("Firing rate (Hz)")
plt.title("Força × Taxa de disparo (RA vs SA)")
plt.grid()
plt.legend()

plt.show()