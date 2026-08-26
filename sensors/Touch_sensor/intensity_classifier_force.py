import json
import math
import os
import serial
import socket
import struct
import threading
import time
from collections import deque
import tkinter as tk
from tkinter import ttk


SERIAL_PORT = "COM7"
BAUDRATE = 115200

HZ_WINDOW_S = 0.5

GUI_UPDATE_MS = 33
PLOT_UPDATE_MS = 33

CLASSIFIER_NEURON = "CN_SA"

DEBUG_SERIAL = False


LOAD_CELL_UDP_PORT = 8080
LOAD_CELL_SAMPLE_FMT = "<IIf"          # uint32 seq, uint32 t_us, float v_sensor
LOAD_CELL_SAMPLE_SZ = struct.calcsize(LOAD_CELL_SAMPLE_FMT)   # 12 bytes
LOAD_CELL_BATCH_N = 10

LOAD_CELL_ESP_IP = "192.168.5.105"     # IP estático da ESP32
LOAD_CELL_DISCOVERY_PORT = 8090
LOAD_CELL_DISCOVERY_MAGIC = b"FRCV"

# Manda o hello p/ a ESP responder por UNICAST a ESTA máquina (perda ~0%).
# False = não manda (só recebe broadcast — e broadcast SÓ existe se NENHUM
# outro host estiver mandando hello). Ver aviso no docstring.
SEND_DISCOVERY_HELLO = True
DISCOVERY_INTERVAL_S = 2.0

# Calibração tensão→força. Se o JSON existir (cópia do repo:
# sensors/load_cell_calib.json), slope/intercept vêm de lá; senão usa os
# valores abaixo (calibração de 10/07 versionada no repo).
LOAD_CELL_CALIB_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "load_cell_calib.json")
CALIB_SLOPE_DEFAULT = 0.07629081620876085
CALIB_INTERCEPT_DEFAULT = 4.6435712874881574e-07

# Força ESPELHADA da GUI do touch_pack (aba Palpação, /load_cell/force_net):
# mesma fórmula F = (v_filtrada - v_tare) / slope, compressão = positivo.
# A tara é em TENSÃO (papel do botão "Tare Sensor" da GUI) e o auto-zero
# lento abaixo replica o da GUI: em repouso (|F| < banda) puxa a referência
# devagar p/ cancelar deriva DC sem comer força real de um toque.
AUTOZERO_BAND_N = 0.30    # igual à GUI (_lc_autozero_band_n)
AUTOZERO_RATE = 0.0001    # passo/amostra a 1 kHz ≈ τ 10 s (GUI: 0.001 @100 Hz)

# Eixo Y FIXO do gráfico de força (N). Compressão = positivo.
FORCE_MIN_PLOT = 0.0
FORCE_MAX_PLOT = 10.0

# A ESP amostra a 1 kHz; plotar 1 ponto a cada N amostras (10 → 100 Hz no
# gráfico, 500 pontos na janela de 5 s — leve para o canvas do Tk).
FORCE_PLOT_DECIMATION = 10

# Filtro pesado (idêntico ao force_receiver_node do ROS)
MEDIAN_N = 5
ONE_EURO_FREQ = 1000.0
ONE_EURO_MINCUTOFF = 4.0
ONE_EURO_BETA = 7.0
ONE_EURO_DCUTOFF = 5.0


LIM_NO_TOUCH = 1.0

LIM_VERY_LIGHT = 50.0
LIM_LIGHT = 100.0
LIM_MEDIUM = 150.0
LIM_STRONG = 200.0

ADC_NO_TOUCH = 4000

FREQ_BAR_MAX = 600

PLOT_WINDOW_S = 5.0

V_REF = 3.3


SPIKE_COLORS_CN = {
    "CN_MM": "#3D4F8A",
    "CN_FA": "#8C2D5D",
    "CN_SA": "#006B6B"
}


HZ_BAR_COLORS = {
    "CN_MM": "#3D4F8A",
    "CN_FA": "#8C2D5D",
    "CN_SA": "#006B6B",
    "FA": "#0B3954",
    "SA": "#B24C3D"
}

FORCE_LINE_COLOR = "#0B3954"


running = True
lock = threading.Lock()

start_time = time.monotonic()

spikes = {
    "FA": deque(),
    "SA": deque(),
    "CN_MM": deque(),
    "CN_FA": deque(),
    "CN_SA": deque()
}

frequencies = {
    "FA": 0.0,
    "SA": 0.0,
    "CN_MM": 0.0,
    "CN_FA": 0.0,
    "CN_SA": 0.0
}

# Série da força (célula de carga via UDP) para o gráfico
force_time = deque()
force_values = deque()

cn_spikes_plot = {
    "CN_MM": deque(),
    "CN_FA": deque(),
    "CN_SA": deque()
}

cn_spike_counter = {
    "CN_MM": 0,
    "CN_FA": 0,
    "CN_SA": 0
}

last_adc = None
last_force = None
last_force_rx_time = None
force_packet_counter = 0
serial_error = None
udp_error = None

adc_counter = 0

# Tare em TENSÃO, espelhando a GUI do touch_pack (_lc_do_tare): a referência
# v_tare é a média da tensão FILTRADA em repouso e F = (v - v_tare)/slope.
# O offset do amplificador deriva entre sessões, então na largada (e no botão
# "Tare") a média de TARE_AVG_N amostras vira a referência. Antes do primeiro
# tare, o intercept da calibração serve de referência provisória.
tare_voltage = None          # None = ainda não tarado (usa CALIB_INTERCEPT)
tare_pending = True          # auto-tare na largada
TARE_SKIP_N = 500            # amostras descartadas antes de medir (0.5 s)
TARE_AVG_N = 1000            # amostras na média da referência (1 s)


def request_tare():
    """Re-zera a força (chame com a célula EM REPOUSO)."""
    global tare_pending
    with lock:
        tare_pending = True
    print("tare solicitado: mantenha a célula sem carga por ~2 s...")


class LoadCellFilter:
    """Mediana de MEDIAN_N (mata spikes) seguida do One-Euro — passa-baixa de
    cutoff adaptativo: firme parado, rápido quando o sinal se move."""

    def __init__(self):
        self._freq = ONE_EURO_FREQ
        self._mincutoff = ONE_EURO_MINCUTOFF
        self._beta = ONE_EURO_BETA
        self._dcutoff = ONE_EURO_DCUTOFF
        self._median_n = MEDIAN_N
        self._median_buf = []
        self._mi = 0
        self._x_prev = 0.0
        self._dx_prev = 0.0
        self._seeded = False

    @staticmethod
    def _alpha(cutoff, freq):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau * freq)

    def update(self, v):
        if not self._seeded:
            self._median_buf = [v] * self._median_n
            self._x_prev = v
            self._dx_prev = 0.0
            self._seeded = True
            return v
        self._median_buf[self._mi] = v
        self._mi = (self._mi + 1) % self._median_n
        v_med = sorted(self._median_buf)[self._median_n // 2]
        dx = (v_med - self._x_prev) * self._freq
        a_d = self._alpha(self._dcutoff, self._freq)
        dx_hat = a_d * dx + (1.0 - a_d) * self._dx_prev
        cutoff = self._mincutoff + self._beta * abs(dx_hat)
        a = self._alpha(cutoff, self._freq)
        x_hat = a * v_med + (1.0 - a) * self._x_prev
        self._x_prev = x_hat
        self._dx_prev = dx_hat
        return x_hat


def load_calibration():
    """slope/intercept do JSON, na MESMA ordem de precedência da GUI do
    touch_pack (lc_calib_read_path): local ~/.config/touch_pack primeiro,
    depois a versionada no repo (sensors/load_cell_calib.json), depois a
    cópia ao lado deste script; sem nenhuma, os defaults acima."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.expanduser("~/.config/touch_pack/load_cell_calib.json"),
        os.path.join(script_dir, "..", "load_cell_calib.json"),
        LOAD_CELL_CALIB_JSON,
    ]
    for path in candidates:
        try:
            with open(path) as f:
                data = json.load(f)
            print(f"calibração carregada de {os.path.normpath(path)}")
            return float(data["slope"]), float(data["intercept"])
        except Exception:
            continue
    return CALIB_SLOPE_DEFAULT, CALIB_INTERCEPT_DEFAULT


CALIB_SLOPE, CALIB_INTERCEPT = load_calibration()


def normalize_neuron_name(name):
    name = name.strip().upper()
    name = name.replace("-", "_")

    if name in ["CN_RA", "CNRA", "CN_FA", "CNFA"]:
        return "CN_FA"

    if name in ["CN_MM", "CNMM"]:
        return "CN_MM"

    if name in ["CN_SA", "CNSA"]:
        return "CN_SA"

    # Se o STM32 ainda enviar RA, a interface trata como FA
    if name in ["RA", "FA"]:
        return "FA"

    if name == "SA":
        return "SA"

    if name == "ADC":
        return "ADC"

    return name


CLASSIFIER_NEURON_USED = normalize_neuron_name(CLASSIFIER_NEURON)


def process_line(line):
    global last_adc
    global adc_counter

    now = time.monotonic()
    relative_time = now - start_time

    line = line.strip()

    if not line:
        return

    raw_type = line.split(",")[0].strip().upper()
    raw_type = raw_type.replace("-", "_")

    neuron_type = normalize_neuron_name(raw_type)

    if DEBUG_SERIAL and "CN" in line.upper():
        print("CN LINE RECEIVED:", repr(line))

    if neuron_type == "FA":
        with lock:
            spikes["FA"].append(now)
        return

    if neuron_type == "SA":
        with lock:
            spikes["SA"].append(now)
        return

    if neuron_type in ["CN_MM", "CN_FA", "CN_SA"]:
        with lock:
            spikes[neuron_type].append(now)
            cn_spikes_plot[neuron_type].append(relative_time)
            cn_spike_counter[neuron_type] += 1
        return

    # ADC frame (ainda usado pelo classificador p/ o NO TOUCH;
    # o gráfico agora é a força da célula de carga, via UDP)
    if neuron_type == "ADC":
        try:
            parts = line.split(",")

            adc_values = []

            for p in parts[1:]:
                p = p.strip()

                if p.lower().startswith("t="):
                    break

                adc_values.append(int(p))

            if len(adc_values) == 25:
                mean_adc = sum(adc_values) / 25.0

                with lock:
                    last_adc = mean_adc
                    adc_counter += 1

        except Exception as e:
            if DEBUG_SERIAL:
                print("Error processing ADC:", e)


def read_serial():
    global running, serial_error

    try:
        ser = serial.Serial(
            SERIAL_PORT,
            BAUDRATE,
            timeout=0.001
        )

        time.sleep(1.5)
        ser.reset_input_buffer()

        print("Connected to:", SERIAL_PORT)

        buffer = b""

        while running:
            n = ser.in_waiting

            if n > 0:
                data = ser.read(n)
            else:
                data = ser.read(1)

            if not data:
                continue

            buffer += data

            if len(buffer) > 30000:
                buffer = b""

            while b"\n" in buffer:
                line_bytes, buffer = buffer.split(b"\n", 1)

                try:
                    line = line_bytes.decode(errors="ignore").strip()
                except:
                    line = ""

                if line:
                    process_line(line)

        ser.close()

    except Exception as e:
        with lock:
            serial_error = str(e)

        print("SERIAL ERROR:", e)


def read_force_udp():
    """Recebe os lotes UDP da ESP32, filtra, converte p/ força (N) e
    alimenta a série do gráfico. Mesma matemática do force_receiver_node."""
    global running, udp_error, last_force, last_force_rx_time
    global force_packet_counter, tare_voltage, tare_pending

    lc_filter = LoadCellFilter()
    sample_idx = 0
    tare_skip = 0
    tare_buf = []

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            # Não existe no Windows — só importa p/ vários receivers no mesmo PC
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except (AttributeError, OSError):
            pass
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError:
            pass
        sock.settimeout(1.0)
        # Windows: por padrão um ICMP "port unreachable" (ex.: hello enviado
        # com a ESP fora do ar) derruba o recvfrom seguinte com
        # ConnectionResetError. Este ioctl desliga esse comportamento.
        # ValueError: o socket.ioctl do Python só aceita alguns comandos e
        # rejeita SIO_UDP_CONNRESET em versões que não o expõem — nesse caso
        # (e em Linux/macOS, sem ioctl) o except ConnectionResetError do laço
        # de recepção cobre sozinho.
        try:
            SIO_UDP_CONNRESET = getattr(socket, "SIO_UDP_CONNRESET",
                                        0x9800000C)
            sock.ioctl(SIO_UDP_CONNRESET, struct.pack("I", 0))
        except (AttributeError, ValueError, OSError):
            pass
        sock.bind(("", LOAD_CELL_UDP_PORT))

        print(f"Load cell UDP bind OK on 0.0.0.0:{LOAD_CELL_UDP_PORT}")

        rcvbuf = LOAD_CELL_SAMPLE_SZ * LOAD_CELL_BATCH_N + 64
        last_hello = 0.0

        while running:
            # Hello p/ a ESP mandar unicast p/ ESTA máquina (ver docstring)
            if SEND_DISCOVERY_HELLO:
                now = time.monotonic()
                if now - last_hello >= DISCOVERY_INTERVAL_S:
                    last_hello = now
                    try:
                        sock.sendto(LOAD_CELL_DISCOVERY_MAGIC,
                                    (LOAD_CELL_ESP_IP, LOAD_CELL_DISCOVERY_PORT))
                    except OSError:
                        pass   # rede fora: firmware cai no broadcast sozinho

            try:
                raw, _ = sock.recvfrom(max(256, rcvbuf))
            except socket.timeout:
                continue
            except ConnectionResetError:
                # Windows: reset por ICMP de um hello anterior — transitório,
                # segue ouvindo (defesa extra caso o ioctl acima não pegue).
                continue
            except OSError as e:
                # Erro fatal do socket: mostra na GUI em vez de morrer mudo
                # (o label ficaria em "waiting for UDP data..." p/ sempre).
                with lock:
                    udp_error = f"socket error: {e}"
                break

            n_samples = len(raw) // LOAD_CELL_SAMPLE_SZ
            if n_samples == 0:
                continue

            now = time.monotonic()
            relative_time = now - start_time

            for k in range(n_samples):
                off = k * LOAD_CELL_SAMPLE_SZ
                (_seq, _t_us, v_raw) = struct.unpack_from(
                    LOAD_CELL_SAMPLE_FMT, raw, off)

                v_sensor = lc_filter.update(float(v_raw))

                # Tare em tensão: média do repouso (auto na largada e via
                # botão "Tare" — mesmo papel do "Tare Sensor" da GUI).
                if tare_pending:
                    tare_skip += 1
                    if tare_skip > TARE_SKIP_N:
                        tare_buf.append(v_sensor)
                        if len(tare_buf) >= TARE_AVG_N:
                            new_ref = sum(tare_buf) / len(tare_buf)
                            with lock:
                                tare_voltage = new_ref
                                tare_pending = False
                            tare_buf.clear()
                            tare_skip = 0
                            print(f"tare: referência = {new_ref:.4f} V")

                # Mesma conta do /load_cell/force_net da GUI do touch_pack:
                # F = (v - v_tare)/slope, compressão = positivo. Antes do
                # primeiro tare, o intercept é a referência provisória.
                v_ref = tare_voltage if tare_voltage is not None \
                    else CALIB_INTERCEPT
                force = (v_sensor - v_ref) / CALIB_SLOPE

                # Auto-zero lento (igual ao da GUI): em repouso, dentro da
                # banda morta, puxa a referência devagar p/ cancelar deriva
                # DC. A GUI ainda exige fase IDLE/DONE/ABORTED do robô; aqui
                # não há essa informação — a banda morta faz o papel sozinha.
                if not tare_pending and tare_voltage is not None \
                        and abs(force) < AUTOZERO_BAND_N:
                    v_ref += AUTOZERO_RATE * (v_sensor - v_ref)
                    with lock:
                        tare_voltage = v_ref
                    force = (v_sensor - v_ref) / CALIB_SLOPE

                sample_idx += 1
                if sample_idx % FORCE_PLOT_DECIMATION != 0:
                    continue

                with lock:
                    last_force = force
                    last_force_rx_time = now
                    force_time.append(relative_time)
                    force_values.append(force)

            with lock:
                force_packet_counter += 1

        sock.close()

    except Exception as e:
        with lock:
            udp_error = str(e)

        print("UDP ERROR:", e)


def update_frequencies():
    now = time.monotonic()
    relative_time = now - start_time

    with lock:
        for name, queue_spikes in spikes.items():
            while queue_spikes and now - queue_spikes[0] > HZ_WINDOW_S:
                queue_spikes.popleft()

            frequencies[name] = len(queue_spikes) / HZ_WINDOW_S

        while force_time and relative_time - force_time[0] > PLOT_WINDOW_S:
            force_time.popleft()
            force_values.popleft()

        for name in cn_spikes_plot:
            while cn_spikes_plot[name] and relative_time - cn_spikes_plot[name][0] > PLOT_WINDOW_S:
                cn_spikes_plot[name].popleft()

        frequencies_copy = frequencies.copy()
        adc_copy = last_adc
        force_copy = last_force
        force_rx_copy = last_force_rx_time
        error_copy = serial_error
        udp_error_copy = udp_error
        adc_counter_copy = adc_counter
        cn_counter_copy = cn_spike_counter.copy()

    return (
        frequencies_copy,
        adc_copy,
        force_copy,
        force_rx_copy,
        error_copy,
        udp_error_copy,
        adc_counter_copy,
        cn_counter_copy
    )


def copy_plot_data():
    now = time.monotonic()
    relative_time = now - start_time

    with lock:
        t_force = list(force_time)
        forces = list(force_values)

        t_cn_spikes = {
            "CN_MM": list(cn_spikes_plot["CN_MM"]),
            "CN_FA": list(cn_spikes_plot["CN_FA"]),
            "CN_SA": list(cn_spikes_plot["CN_SA"])
        }

        cn_counter_copy = cn_spike_counter.copy()

    return relative_time, t_force, forces, t_cn_spikes, cn_counter_copy


def classify_force(freq_hz, mean_adc):
    if mean_adc is not None:
        if mean_adc >= ADC_NO_TOUCH and freq_hz < LIM_NO_TOUCH:
            return "NO TOUCH"

    if freq_hz < LIM_NO_TOUCH:
        return "NO TOUCH"

    elif freq_hz < LIM_VERY_LIGHT:
        return "VERY LIGHT"

    elif freq_hz < LIM_LIGHT:
        return "LIGHT"

    elif freq_hz < LIM_MEDIUM:
        return "MEDIUM"

    elif freq_hz < LIM_STRONG:
        return "STRONG"

    else:
        return "VERY STRONG"


def class_color(force_class):
    if force_class == "NO TOUCH":
        return "gray"

    elif force_class == "VERY LIGHT":
        return "lightgreen"

    elif force_class == "LIGHT":
        return "green"

    elif force_class == "MEDIUM":
        return "orange"

    elif force_class == "STRONG":
        return "red"

    elif force_class == "VERY STRONG":
        return "darkred"

    else:
        return "gray"


window = tk.Tk()
window.title("Real-Time Intensity Classification - STM32 + Load Cell")
window.geometry("1050x720")


style = ttk.Style()
style.theme_use("clam")

for name, color in HZ_BAR_COLORS.items():
    style.configure(
        f"{name}.Horizontal.TProgressbar",
        troughcolor="#E6E6E6",
        background=color,
        lightcolor=color,
        darkcolor=color,
        bordercolor="#BBBBBB"
    )

title_label = tk.Label(
    window,
    text="Real-Time Intensity Classification - Cuneate Neurons",
    font=("Arial", 14, "bold")
)
title_label.pack(pady=3)


top_frame = tk.Frame(window)
top_frame.pack(fill="x", padx=10, pady=2)

class_label = tk.Label(
    top_frame,
    text="NO TOUCH",
    font=("Arial", 22, "bold"),
    width=18,
    height=1,
    bg="gray",
    fg="white"
)
class_label.pack(side="left", padx=5)

info_frame = tk.Frame(top_frame)
info_frame.pack(side="left", fill="x", expand=True, padx=10)

neuron_label = tk.Label(
    info_frame,
    text=f"Classifier neuron: {CLASSIFIER_NEURON_USED}",
    font=("Arial", 10)
)
neuron_label.pack(anchor="w")

classifier_freq_label = tk.Label(
    info_frame,
    text="Firing rate: 0.00 Hz",
    font=("Arial", 12, "bold")
)
classifier_freq_label.pack(anchor="w")

force_row = tk.Frame(info_frame)
force_row.pack(anchor="w")

force_label = tk.Label(
    force_row,
    text="Load cell: waiting for UDP data...",
    font=("Arial", 10)
)
force_label.pack(side="left")

tare_button = tk.Button(
    force_row,
    text="Tare",
    font=("Arial", 8),
    command=request_tare
)
tare_button.pack(side="left", padx=(8, 0))

debug_label = tk.Label(
    info_frame,
    text="",
    font=("Arial", 9)
)
debug_label.pack(anchor="w")


bars_frame = tk.LabelFrame(
    window,
    text="Average neuron firing frequency (Hz)",
    font=("Arial", 9, "bold")
)
bars_frame.pack(fill="x", padx=10, pady=3)

bars = {}
freq_labels = {}

for name in ["CN_MM", "CN_FA", "CN_SA", "FA", "SA"]:
    row_frame = tk.Frame(bars_frame)
    row_frame.pack(fill="x", padx=6, pady=1)

    label = tk.Label(
        row_frame,
        text=name,
        font=("Arial", 9, "bold"),
        width=7,
        anchor="w",
        fg=HZ_BAR_COLORS[name]
    )
    label.pack(side="left")

    bar = ttk.Progressbar(
        row_frame,
        orient="horizontal",
        mode="determinate",
        maximum=FREQ_BAR_MAX,
        style=f"{name}.Horizontal.TProgressbar"
    )
    bar.pack(side="left", fill="x", expand=True, padx=5)

    freq_label = tk.Label(
        row_frame,
        text="0.00 Hz",
        font=("Arial", 9),
        width=10
    )
    freq_label.pack(side="right")

    bars[name] = bar
    freq_labels[name] = freq_label


graphs_frame = tk.LabelFrame(
    window,
    text="Real-time signals",
    font=("Arial", 9, "bold")
)
graphs_frame.pack(fill="both", expand=True, padx=10, pady=3)

force_canvas = tk.Canvas(
    graphs_frame,
    height=180,
    bg="white",
    highlightthickness=1,
    highlightbackground="black"
)
force_canvas.pack(fill="both", expand=True, padx=8, pady=(5, 4))

cn_raster_canvas = tk.Canvas(
    graphs_frame,
    height=170,
    bg="white",
    highlightthickness=1,
    highlightbackground="black"
)
cn_raster_canvas.pack(fill="both", expand=True, padx=8, pady=(0, 5))


def draw_force(canvas, current_time, t_force, forces):
    canvas.delete("all")

    width = canvas.winfo_width()
    height = canvas.winfo_height()

    if width <= 10 or height <= 10:
        return

    left_margin = 50
    right_margin = 15
    top_margin = 22
    bottom_margin = 28

    plot_w = width - left_margin - right_margin
    plot_h = height - top_margin - bottom_margin

    x_min = max(0.0, current_time - PLOT_WINDOW_S)
    x_max = max(PLOT_WINDOW_S, current_time)

    # Escala Y FIXA 0–10 N (FORCE_MIN_PLOT..FORCE_MAX_PLOT), espelhando a
    # leitura da aba Palpação da GUI do touch_pack. Valores fora da faixa
    # são clampados no traço; o número exato fica no label "Load cell force".
    y_lo = FORCE_MIN_PLOT
    y_hi = FORCE_MAX_PLOT
    y_span = y_hi - y_lo
    fmt = "{:.1f}N"

    y_base = top_margin + plot_h

    canvas.create_text(
        width / 2,
        11,
        text="Load cell force (N) — via network",
        font=("Arial", 9, "bold")
    )

    canvas.create_line(left_margin, top_margin, left_margin, y_base, fill="black")
    canvas.create_line(left_margin, y_base, left_margin + plot_w, y_base, fill="black")

    canvas.create_text(23, top_margin, text=fmt.format(y_hi), font=("Arial", 7))
    canvas.create_text(23, y_base, text=fmt.format(y_lo), font=("Arial", 7))

    canvas.create_text(left_margin, height - 12, text=f"{x_min:.1f}s", font=("Arial", 7))
    canvas.create_text(left_margin + plot_w, height - 12, text=f"{x_max:.1f}s", font=("Arial", 7))
    canvas.create_text(width / 2, height - 12, text="Time (s)", font=("Arial", 7))

    for i in range(1, 4):
        y = top_margin + i * plot_h / 4
        force_grid = y_hi - i * y_span / 4
        canvas.create_line(left_margin, y, left_margin + plot_w, y, fill="#dddddd")
        canvas.create_text(23, y, text=fmt.format(force_grid), font=("Arial", 7), fill="#888888")

    if len(t_force) < 2:
        return

    points = []

    for t, force in zip(t_force, forces):
        if t < x_min or t > x_max:
            continue

        x = left_margin + ((t - x_min) / (x_max - x_min)) * plot_w

        force_limited = max(y_lo, min(y_hi, force))

        y = top_margin + (1.0 - ((force_limited - y_lo) / y_span)) * plot_h

        points.append((x, y))

    if len(points) >= 2:
        for i in range(len(points) - 1):
            canvas.create_line(
                points[i][0],
                points[i][1],
                points[i + 1][0],
                points[i + 1][1],
                fill=FORCE_LINE_COLOR,
                width=2
            )


def draw_cn_raster(canvas, current_time, t_cn_spikes, total_cn):
    canvas.delete("all")

    width = canvas.winfo_width()
    height = canvas.winfo_height()

    if width <= 10 or height <= 10:
        return

    left_margin = 60
    right_margin = 15
    top_margin = 25
    bottom_margin = 28

    plot_w = width - left_margin - right_margin
    plot_h = height - top_margin - bottom_margin

    x_min = max(0.0, current_time - PLOT_WINDOW_S)
    x_max = max(PLOT_WINDOW_S, current_time)

    y_base = top_margin + plot_h

    order = ["CN_MM", "CN_FA", "CN_SA"]

    canvas.create_text(
        width / 2,
        11,
        text="Cuneate neuron raster plot",
        font=("Arial", 9, "bold")
    )

    canvas.create_line(left_margin, top_margin, left_margin, y_base, fill="black")
    canvas.create_line(left_margin, y_base, left_margin + plot_w, y_base, fill="black")

    canvas.create_text(left_margin, height - 12, text=f"{x_min:.1f}s", font=("Arial", 7))
    canvas.create_text(left_margin + plot_w, height - 12, text=f"{x_max:.1f}s", font=("Arial", 7))
    canvas.create_text(width / 2, height - 12, text="Time (s)", font=("Arial", 7))

    for i, name in enumerate(order):
        y = top_margin + ((i + 1) / (len(order) + 1)) * plot_h

        canvas.create_line(
            left_margin,
            y,
            left_margin + plot_w,
            y,
            fill="#cccccc"
        )

        canvas.create_text(
            30,
            y,
            text=name,
            font=("Arial", 7, "bold"),
            fill=SPIKE_COLORS_CN[name]
        )

        for t in t_cn_spikes[name]:
            if x_min <= t <= x_max:
                x = left_margin + ((t - x_min) / (x_max - x_min)) * plot_w

                canvas.create_line(
                    x,
                    y - 16,
                    x,
                    y + 16,
                    fill=SPIKE_COLORS_CN[name],
                    width=2
                )


def update_interface():
    result = update_frequencies()

    (
        current_freqs,
        current_adc,
        current_force,
        current_force_rx,
        current_error,
        current_udp_error,
        adc_count,
        cn_count
    ) = result

    if current_error is not None:
        class_label.config(
            text="SERIAL ERROR",
            bg="red"
        )

        classifier_freq_label.config(
            text=f"Error: {current_error}"
        )

    else:
        classifier_freq = current_freqs[CLASSIFIER_NEURON_USED]
        force_class = classify_force(classifier_freq, current_adc)

        class_label.config(
            text=force_class,
            bg=class_color(force_class)
        )

        classifier_freq_label.config(
            text=f"{CLASSIFIER_NEURON_USED} firing rate: {classifier_freq:.2f} Hz"
        )

        now = time.monotonic()

        if current_udp_error is not None:
            force_label.config(
                text=f"Load cell UDP error: {current_udp_error}"
            )
        elif current_force is not None and current_force_rx is not None \
                and now - current_force_rx < 2.0:
            force_label.config(
                text=f"Load cell force: {current_force:.2f} N"
            )
        else:
            force_label.config(
                text="Load cell: waiting for UDP data..."
            )

        debug_label.config(text="")

        for name in bars:
            value = current_freqs[name]
            bars[name]["value"] = min(value, FREQ_BAR_MAX)
            freq_labels[name].config(text=f"{value:.2f} Hz")

    if running:
        window.after(GUI_UPDATE_MS, update_interface)


def update_graphs():
    result = copy_plot_data()
    current_time, t_force, forces, t_cn_spikes, cn_count = result

    draw_force(force_canvas, current_time, t_force, forces)
    draw_cn_raster(cn_raster_canvas, current_time, t_cn_spikes, cn_count)

    if running:
        window.after(PLOT_UPDATE_MS, update_graphs)


def close_window():
    global running
    running = False
    window.destroy()


window.protocol("WM_DELETE_WINDOW", close_window)

serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

force_thread = threading.Thread(target=read_force_udp, daemon=True)
force_thread.start()

update_interface()
update_graphs()

window.mainloop()
