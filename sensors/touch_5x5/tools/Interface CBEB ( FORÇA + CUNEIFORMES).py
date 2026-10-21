import serial
import threading
import time
from collections import deque
import tkinter as tk
from tkinter import ttk
import math


# =====================================================
# SETTINGS
# =====================================================

SERIAL_PORT = "COM9"
BAUDRATE = 115200

HZ_WINDOW_S = 0.5

GUI_UPDATE_MS = 33
PLOT_UPDATE_MS = 33

CLASSIFIER_NEURON = "CN_SA"

DEBUG_SERIAL = False


# =====================================================
# EXPONENTIAL SATURATION REGRESSION
# =====================================================

A_REG = 155.296435
B_REG = 0.957723
C_REG = 4.520999


# =====================================================
# NO TOUCH
# =====================================================

ADC_NO_TOUCH = 4000


# =====================================================
# FREQUENCY BAR
# =====================================================

FREQ_BAR_MAX = 600


# =====================================================
# PLOT
# =====================================================

PLOT_WINDOW_S = 5.0


# =====================================================
# FORCE PLOT
# =====================================================

FORCE_MIN_PLOT = 0.0
FORCE_MAX_PLOT = 15.0


# =====================================================
# ERROR PLOT
# =====================================================

ERROR_MIN_PLOT = -10.0
ERROR_MAX_PLOT = 10.0


# =====================================================
# COLORS
# =====================================================

COR_FA = "#0B3954"
COR_SA = "#B24C3D"

COR_CNMM = "#3D4F8A"
COR_CNFA = "#8C2D5D"
COR_CNSA = "#006B6B"


# =====================================================
# SPIKE COLORS
# =====================================================

SPIKE_COLORS_CN = {
    "CN_MM": COR_CNMM,
    "CN_FA": COR_CNFA,
    "CN_SA": COR_CNSA
}


# =====================================================
# HZ BAR COLORS
# =====================================================

HZ_BAR_COLORS = {
    "CN_MM": COR_CNMM,
    "CN_FA": COR_CNFA,
    "CN_SA": COR_CNSA,
    "FA": COR_FA,
    "SA": COR_SA
}


# =====================================================
# GLOBAL VARIABLES
# =====================================================

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


force_time = deque()

force_values = deque()


estimated_force_values = deque()

error_values = deque()


absolute_error_values = deque()


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


last_force = None

last_filtered_force = None

last_estimated_force = None

last_error = None

last_absolute_error = None

last_percentage_error = None

last_mae = None

last_rmse = None

last_adc = None

serial_error = None

adc_counter = 0


# =====================================================
# AUXILIARY FUNCTIONS
# =====================================================

def normalize_neuron_name(name):

    name = name.strip().upper()

    name = name.replace("-", "_")


    if name in [
        "CN_RA",
        "CNRA",
        "CN_FA",
        "CNFA"
    ]:

        return "CN_FA"


    if name in [
        "CN_MM",
        "CNMM"
    ]:

        return "CN_MM"


    if name in [
        "CN_SA",
        "CNSA"
    ]:

        return "CN_SA"


    if name in [
        "RA",
        "FA"
    ]:

        return "FA"


    if name == "SA":

        return "SA"


    if name == "ADC":

        return "ADC"


    if name == "FORCE":

        return "FORCE"


    return name


CLASSIFIER_NEURON_USED = normalize_neuron_name(
    CLASSIFIER_NEURON
)


# =====================================================
# EXPONENTIAL REGRESSION
# =====================================================

def calculate_expected_frequency(
    delta_force
):

    if delta_force is None:

        return 0.0


    if delta_force < 0.0:

        delta_force = 0.0


    expected_frequency = (

        A_REG

        *

        (

            1.0

            -

            math.exp(

                -B_REG * delta_force

            )

        )

        +

        C_REG

    )


    return expected_frequency


# =====================================================
# FORCE ESTIMATION FROM FIRING RATE
# =====================================================

def estimate_force_from_frequency(frequency):

    if frequency is None:
        return 0.0

    if frequency <= C_REG:
        return 0.0

    value = 1.0 - ((frequency - C_REG) / A_REG)

    # Evita log(0), valores negativos e instabilidade numerica
    value = max(min(value, 0.999999), 1e-9)

    estimated_force = -math.log(value) / B_REG

    if estimated_force < 0.0:
        estimated_force = 0.0

    return estimated_force


# =====================================================
# ERROR CALCULATION
# =====================================================

def calculate_error(

    real_force,

    estimated_force

):

    if real_force is None:

        return None, None, None


    if estimated_force is None:

        return None, None, None


    error = (

        estimated_force

        -

        real_force

    )


    absolute_error = abs(error)


    if real_force > 0.01:

        percentage_error = (

            absolute_error

            /

            real_force

        )

        percentage_error = (

            percentage_error

            *

            100.0

        )

    else:

        percentage_error = 0.0


    return (

        error,

        absolute_error,

        percentage_error

    )


# =====================================================
# STATISTICS
# =====================================================

def calculate_statistics():

    if len(absolute_error_values) == 0:

        return 0.0, 0.0


    total_absolute_error = 0.0

    total_squared_error = 0.0

    number_of_values = 0


    for error in absolute_error_values:

        total_absolute_error += error

        number_of_values += 1


    for error in error_values:

        total_squared_error += error * error


    if number_of_values > 0:

        mae = (

            total_absolute_error

            /

            number_of_values

        )

    else:

        mae = 0.0


    if len(error_values) > 0:

        rmse = math.sqrt(

            total_squared_error

            /

            len(error_values)

        )

    else:

        rmse = 0.0


    return mae, rmse


# =====================================================
# SERIAL LINE PROCESSING
# =====================================================

def process_line(line):

    global last_force

    global last_filtered_force

    global last_estimated_force

    global last_error

    global last_absolute_error

    global last_percentage_error

    global last_mae

    global last_rmse

    global last_adc

    global adc_counter


    now = time.monotonic()

    relative_time = now - start_time


    line = line.strip()


    if not line:

        return


    raw_type = line.split(",")[0].strip().upper()

    raw_type = raw_type.replace("-", "_")


    neuron_type = normalize_neuron_name(

        raw_type

    )


    if DEBUG_SERIAL and "CN" in line.upper():

        print(

            "CN LINE RECEIVED:",

            repr(line)

        )


    # =================================================
    # FA SPIKES
    # =================================================

    if neuron_type == "FA":

        with lock:

            spikes["FA"].append(

                now

            )


        return


    # =================================================
    # SA SPIKES
    # =================================================

    if neuron_type == "SA":

        with lock:

            spikes["SA"].append(

                now

            )


        return


    # =================================================
    # CUNEIFORM NEURON SPIKES
    # =================================================

    if neuron_type in [

        "CN_MM",

        "CN_FA",

        "CN_SA"

    ]:

        with lock:

            spikes[

                neuron_type

            ].append(

                now

            )


            cn_spikes_plot[

                neuron_type

            ].append(

                relative_time

            )


            cn_spike_counter[

                neuron_type

            ] += 1


        return


    # =================================================
    # ADC FRAME
    # =================================================

    if neuron_type == "ADC":

        try:

            parts = line.split(",")


            adc_values = []


            for p in parts[1:]:

                p = p.strip()


                if p.lower().startswith("t="):

                    break


                adc_values.append(

                    int(p)

                )


            if len(adc_values) == 25:

                mean_adc = sum(

                    adc_values

                ) / 25.0


                with lock:

                    last_adc = mean_adc

                    adc_counter += 1


        except Exception as e:

            if DEBUG_SERIAL:

                print(

                    "Error processing ADC:",

                    e

                )


        return


    # =================================================
    # FORCE DATA
    # =================================================

    if neuron_type == "FORCE":

        try:

            parts = line.split(",")


            if len(parts) < 3:

                return


            force_value = float(

                parts[1]

            )


            filtered_force_value = float(

                parts[2]

            )


            with lock:

                last_force = force_value

                last_filtered_force = filtered_force_value


                force_time.append(

                    relative_time

                )


                force_values.append(

                    filtered_force_value

                )


                if last_estimated_force is not None:

                    estimated_force_values.append(

                        last_estimated_force

                    )

                else:

                    estimated_force_values.append(

                        0.0

                    )


                if last_error is not None:

                    error_values.append(

                        last_error

                    )

                else:

                    error_values.append(

                        0.0

                    )


                if last_absolute_error is not None:

                    absolute_error_values.append(

                        last_absolute_error

                    )


                last_mae, last_rmse = calculate_statistics()


        except Exception as e:

            if DEBUG_SERIAL:

                print(

                    "Error processing FORCE:",

                    e

                )


# =====================================================
# SERIAL READING
# =====================================================

def read_serial():

    global running

    global serial_error


    try:

        ser = serial.Serial(

            SERIAL_PORT,

            BAUDRATE,

            timeout=0.001

        )


        time.sleep(

            1.5

        )


        ser.reset_input_buffer()


        print(

            "Connected to:",

            SERIAL_PORT

        )


        buffer = b""


        while running:

            n = ser.in_waiting


            if n > 0:

                data = ser.read(

                    n

                )

            else:

                data = ser.read(

                    1

                )


            if not data:

                continue


            buffer += data


            if len(buffer) > 30000:

                buffer = b""


            while b"\n" in buffer:

                line_bytes, buffer = buffer.split(

                    b"\n",

                    1

                )


                try:

                    line = line_bytes.decode(

                        errors="ignore"

                    ).strip()


                except:

                    line = ""


                if line:

                    process_line(

                        line

                    )


        ser.close()


    except Exception as e:

        with lock:

            serial_error = str(

                e

            )


        print(

            "SERIAL ERROR:",

            e

        )


# =====================================================
# FREQUENCY UPDATE
# =====================================================

def update_frequencies():

    now = time.monotonic()

    relative_time = now - start_time


    with lock:

        for name, queue_spikes in spikes.items():

            while (

                queue_spikes

                and

                now - queue_spikes[0]

                >

                HZ_WINDOW_S

            ):

                queue_spikes.popleft()


            frequencies[name] = (

                len(queue_spikes)

                /

                HZ_WINDOW_S

            )


        # =============================================
        # FORCE ESTIMATION
        # =============================================

        classifier_frequency = frequencies[

            CLASSIFIER_NEURON_USED

        ]


        estimated_force = estimate_force_from_frequency(

            classifier_frequency

        )


        global last_estimated_force

        global last_error

        global last_absolute_error

        global last_percentage_error

        global last_mae

        global last_rmse


        last_estimated_force = estimated_force


        if last_filtered_force is not None:

            (

                error,

                absolute_error,

                percentage_error

            ) = calculate_error(

                last_filtered_force,

                estimated_force

            )


            last_error = error

            last_absolute_error = absolute_error

            last_percentage_error = percentage_error


        last_mae, last_rmse = calculate_statistics()


        while (

            force_time

            and

            relative_time - force_time[0]

            >

            PLOT_WINDOW_S

        ):

            force_time.popleft()

            force_values.popleft()


            if estimated_force_values:

                estimated_force_values.popleft()


            if error_values:

                error_values.popleft()


            if absolute_error_values:

                absolute_error_values.popleft()


        for name in cn_spikes_plot:

            while (

                cn_spikes_plot[name]

                and

                relative_time

                -

                cn_spikes_plot[name][0]

                >

                PLOT_WINDOW_S

            ):

                cn_spikes_plot[name].popleft()


        frequencies_copy = frequencies.copy()


        force_copy = last_force


        filtered_force_copy = last_filtered_force


        estimated_force_copy = last_estimated_force


        error_copy = last_error


        absolute_error_copy = last_absolute_error


        percentage_error_copy = last_percentage_error


        mae_copy = last_mae


        rmse_copy = last_rmse


        adc_copy = last_adc


        error_serial_copy = serial_error


        adc_counter_copy = adc_counter


        cn_counter_copy = cn_spike_counter.copy()


    return (

        frequencies_copy,

        force_copy,

        filtered_force_copy,

        estimated_force_copy,

        error_copy,

        absolute_error_copy,

        percentage_error_copy,

        mae_copy,

        rmse_copy,

        adc_copy,

        error_serial_copy,

        adc_counter_copy,

        cn_counter_copy

    )


# =====================================================
# COPY PLOT DATA
# =====================================================

def copy_plot_data():

    now = time.monotonic()

    relative_time = now - start_time


    with lock:

        t_force = list(

            force_time

        )


        force_copy = list(

            force_values

        )


        estimated_copy = list(

            estimated_force_values

        )


        error_copy = list(

            error_values

        )


        t_cn_spikes = {

            "CN_MM": list(

                cn_spikes_plot["CN_MM"]

            ),

            "CN_FA": list(

                cn_spikes_plot["CN_FA"]

            ),

            "CN_SA": list(

                cn_spikes_plot["CN_SA"]

            )

        }


        cn_counter_copy = cn_spike_counter.copy()


    return (

        relative_time,

        t_force,

        force_copy,

        estimated_copy,

        error_copy,

        t_cn_spikes,

        cn_counter_copy

    )


# =====================================================
# CLASSIFICATION
# =====================================================

def classify_force(estimated_force):

    if estimated_force is None:
        return "NO TOUCH"

    if estimated_force <= 0.05:
        return "NO TOUCH"

    elif estimated_force <= 0.5:
        return "VERY LIGHT"

    elif estimated_force <= 1.5:
        return "LIGHT"

    elif estimated_force <= 3.0:
        return "MEDIUM"

    elif estimated_force <= 4.5:
        return "STRONG"

    else:
        return "VERY STRONG"


# =====================================================
# CLASS COLORS
# =====================================================

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


# =====================================================
# GRAPHICAL INTERFACE
# =====================================================

window = tk.Tk()


window.title(

    "Real-Time Force Estimation Error - STM32"

)


window.geometry(

    "1200x900"

)


# =====================================================
# STYLE
# =====================================================

style = ttk.Style()

style.theme_use(

    "clam"

)


for name, color in HZ_BAR_COLORS.items():

    style.configure(

        f"{name}.Horizontal.TProgressbar",

        troughcolor="#E6E6E6",

        background=color,

        lightcolor=color,

        darkcolor=color,

        bordercolor="#BBBBBB"

    )


# =====================================================
# TITLE
# =====================================================

title_label = tk.Label(

    window,

    text=(

        "Real-Time Force Estimation "

        "and Error Analysis"

    ),

    font=(

        "Arial",

        14,

        "bold"

    )

)


title_label.pack(

    pady=3

)


# =====================================================
# TOP
# =====================================================

top_frame = tk.Frame(

    window

)


top_frame.pack(

    fill="x",

    padx=10,

    pady=2

)


class_label = tk.Label(

    top_frame,

    text="NO TOUCH",

    font=(

        "Arial",

        22,

        "bold"

    ),

    width=18,

    height=1,

    bg="gray",

    fg="white"

)


class_label.pack(

    side="left",

    padx=5

)


info_frame = tk.Frame(

    top_frame

)


info_frame.pack(

    side="left",

    fill="x",

    expand=True,

    padx=10

)


neuron_label = tk.Label(

    info_frame,

    text=(

        f"Classifier neuron: "

        f"{CLASSIFIER_NEURON_USED}"

    ),

    font=(

        "Arial",

        10

    )

)


neuron_label.pack(

    anchor="w"

)


classifier_freq_label = tk.Label(

    info_frame,

    text="Firing rate: 0.00 Hz",

    font=(

        "Arial",

        12,

        "bold"

    )

)


classifier_freq_label.pack(

    anchor="w"

)


force_label = tk.Label(

    info_frame,

    text=(

        "Real force: 0.000 N | "

        "Estimated: 0.000 N"

    ),

    font=(

        "Arial",

        10

    )

)


force_label.pack(

    anchor="w"

)


error_label = tk.Label(

    info_frame,

    text=(

        "Error: 0.000 N | "

        "Absolute error: 0.000 N | "

        "Percentage: 0.00%"

    ),

    font=(

        "Arial",

        10

    )

)


error_label.pack(

    anchor="w"

)



adc_label = tk.Label(

    info_frame,

    text="",

    font=(

        "Arial",

        10

    )

)


adc_label.pack(

    anchor="w"

)


debug_label = tk.Label(

    info_frame,

    text="",

    font=(

        "Arial",

        9

    )

)


debug_label.pack(

    anchor="w"

)


# =====================================================
# FREQUENCY BARS
# =====================================================

bars_frame = tk.LabelFrame(

    window,

    text=(

        "Average neuron firing frequency (Hz)"

    ),

    font=(

        "Arial",

        9,

        "bold"

    )

)


bars_frame.pack(

    fill="x",

    padx=10,

    pady=3

)


bars = {}

freq_labels = {}


for name in [

    "CN_MM",

    "CN_FA",

    "CN_SA",

    "FA",

    "SA"

]:

    row_frame = tk.Frame(

        bars_frame

    )


    row_frame.pack(

        fill="x",

        padx=6,

        pady=1

    )


    label = tk.Label(

        row_frame,

        text=name,

        font=(

            "Arial",

            9,

            "bold"

        ),

        width=7,

        anchor="w",

        fg=HZ_BAR_COLORS[name]

    )


    label.pack(

        side="left"

    )


    bar = ttk.Progressbar(

        row_frame,

        orient="horizontal",

        mode="determinate",

        maximum=FREQ_BAR_MAX,

        style=(

            f"{name}.Horizontal.TProgressbar"

        )

    )


    bar.pack(

        side="left",

        fill="x",

        expand=True,

        padx=5

    )


    freq_label = tk.Label(

        row_frame,

        text="0.00 Hz",

        font=(

            "Arial",

            9

        ),

        width=10

    )


    freq_label.pack(

        side="right"

    )


    bars[name] = bar

    freq_labels[name] = freq_label


# =====================================================
# GRAPHICAL CANVAS
# =====================================================

graphs_frame = tk.LabelFrame(

    window,

    text="Real-time signals",

    font=(

        "Arial",

        9,

        "bold"

    )

)


graphs_frame.pack(

    fill="both",

    expand=True,

    padx=10,

    pady=3

)


force_canvas = tk.Canvas(

    graphs_frame,

    height=260,

    bg="white",

    highlightthickness=1,

    highlightbackground="black"

)


force_canvas.pack(

    fill="both",

    expand=True,

    padx=8,

    pady=(5, 4)

)


cn_raster_canvas = tk.Canvas(

    graphs_frame,

    height=240,

    bg="white",

    highlightthickness=1,

    highlightbackground="black"

)


cn_raster_canvas.pack(

    fill="both",

    expand=True,

    padx=8,

    pady=(0, 5)

)


# =====================================================
# DRAW FORCE GRAPH
# =====================================================

def draw_force_graph(

    canvas,

    current_time,

    t_force,

    force_values

):

    canvas.delete(

        "all"

    )


    width = canvas.winfo_width()

    height = canvas.winfo_height()


    if width <= 10 or height <= 10:

        return


    left_margin = 50

    right_margin = 15

    top_margin = 22

    bottom_margin = 28


    plot_w = (

        width

        -

        left_margin

        -

        right_margin

    )


    plot_h = (

        height

        -

        top_margin

        -

        bottom_margin

    )


    x_min = max(

        0.0,

        current_time - PLOT_WINDOW_S

    )


    x_max = max(

        PLOT_WINDOW_S,

        current_time

    )


    y_base = top_margin + plot_h


    canvas.create_text(

        width / 2,

        11,

        text="Real force",

        font=(

            "Arial",

            9,

            "bold"

        )

    )


    canvas.create_line(

        left_margin,

        top_margin,

        left_margin,

        y_base,

        fill="black"

    )


    canvas.create_line(

        left_margin,

        y_base,

        left_margin + plot_w,

        y_base,

        fill="black"

    )


    canvas.create_text(

        23,

        top_margin,

        text=(

            f"{FORCE_MAX_PLOT:.1f}N"

        ),

        font=(

            "Arial",

            7

        )

    )


    canvas.create_text(

        23,

        y_base,

        text=(

            f"{FORCE_MIN_PLOT:.1f}N"

        ),

        font=(

            "Arial",

            7

        )

    )


    canvas.create_text(

        left_margin,

        height - 12,

        text=(

            f"{x_min:.1f}s"

        ),

        font=(

            "Arial",

            7

        )

    )


    canvas.create_text(

        left_margin + plot_w,

        height - 12,

        text=(

            f"{x_max:.1f}s"

        ),

        font=(

            "Arial",

            7

        )

    )


    canvas.create_text(

        width / 2,

        height - 12,

        text="Time (s)",

        font=(

            "Arial",

            7

        )

    )


    for i in range(

        1,

        4

    ):

        y = (

            top_margin

            +

            i * plot_h / 4

        )


        canvas.create_line(

            left_margin,

            y,

            left_margin + plot_w,

            y,

            fill="#dddddd"

        )


    # =================================================
    # REAL FORCE
    # =================================================

    points_real = []


    for t, force in zip(

        t_force,

        force_values

    ):

        if t < x_min or t > x_max:

            continue


        x = (

            left_margin

            +

            (

                (

                    t - x_min

                )

                /

                (

                    x_max - x_min

                )

            )

            *

            plot_w

        )


        force_limited = max(

            FORCE_MIN_PLOT,

            min(

                FORCE_MAX_PLOT,

                force

            )

        )


        y = (

            top_margin

            +

            (

                1.0

                -

                (

                    (

                        force_limited

                        -

                        FORCE_MIN_PLOT

                    )

                    /

                    (

                        FORCE_MAX_PLOT

                        -

                        FORCE_MIN_PLOT

                    )

                )

            )

            *

            plot_h

        )


        points_real.append(

            (

                x,

                y

            )

        )


    if len(points_real) >= 2:

        for i in range(

            len(points_real) - 1

        ):

            canvas.create_line(

                points_real[i][0],

                points_real[i][1],

                points_real[i + 1][0],

                points_real[i + 1][1],

                fill=COR_FA,

                width=2

            )



# =====================================================
# DRAW CN RASTER
# =====================================================

def draw_cn_raster(

    canvas,

    current_time,

    t_cn_spikes,

    total_cn

):

    canvas.delete(

        "all"

    )


    width = canvas.winfo_width()

    height = canvas.winfo_height()


    if width <= 10 or height <= 10:

        return


    left_margin = 60

    right_margin = 15

    top_margin = 25

    bottom_margin = 28


    plot_w = (

        width

        -

        left_margin

        -

        right_margin

    )


    plot_h = (

        height

        -

        top_margin

        -

        bottom_margin

    )


    x_min = max(

        0.0,

        current_time - PLOT_WINDOW_S

    )


    x_max = max(

        PLOT_WINDOW_S,

        current_time

    )


    y_base = top_margin + plot_h


    order = [

        "CN_MM",

        "CN_FA",

        "CN_SA"

    ]


    canvas.create_text(

        width / 2,

        11,

        text="Cuneate neuron raster plot",

        font=(

            "Arial",

            9,

            "bold"

        )

    )


    canvas.create_line(

        left_margin,

        top_margin,

        left_margin,

        y_base,

        fill="black"

    )


    canvas.create_line(

        left_margin,

        y_base,

        left_margin + plot_w,

        y_base,

        fill="black"

    )


    for i in range(

        len(order)

    ):

        name = order[i]


        y = (

            top_margin

            +

            (

                (

                    i + 1

                )

                /

                (

                    len(order) + 1

                )

            )

            *

            plot_h

        )


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

            font=(

                "Arial",

                7,

                "bold"

            ),

            fill=SPIKE_COLORS_CN[name]

        )


        for t in t_cn_spikes[name]:

            if x_min <= t <= x_max:

                x = (

                    left_margin

                    +

                    (

                        (

                            t - x_min

                        )

                        /

                        (

                            x_max - x_min

                        )

                    )

                    *

                    plot_w

                )


                canvas.create_line(

                    x,

                    y - 16,

                    x,

                    y + 16,

                    fill=SPIKE_COLORS_CN[name],

                    width=2

                )


    # =================================================
    # TIME AXIS
    # =================================================

    canvas.create_text(

        left_margin,

        height - 12,

        text=f"{x_min:.1f}s",

        font=(

            "Arial",

            7

        )

    )


    canvas.create_text(

        left_margin + plot_w,

        height - 12,

        text=f"{x_max:.1f}s",

        font=(

            "Arial",

            7

        )

    )


    canvas.create_text(

        width / 2,

        height - 12,

        text="Time (s)",

        font=(

            "Arial",

            7

        )

    )


# =====================================================
# INTERFACE UPDATE
# =====================================================

def update_interface():

    result = update_frequencies()


    (

        current_freqs,

        current_force,

        current_filtered_force,

        current_estimated_force,

        current_error,

        current_absolute_error,

        current_percentage_error,

        current_mae,

        current_rmse,

        current_adc,

        current_error_serial,

        adc_count,

        cn_count

    ) = result


    if current_error_serial is not None:

        class_label.config(

            text="SERIAL ERROR",

            bg="red"

        )


        classifier_freq_label.config(

            text=(

                f"Error: "

                f"{current_error_serial}"

            )

        )


    else:

        classifier_freq = (

            current_freqs[

                CLASSIFIER_NEURON_USED

            ]

        )


        force_class = classify_force(

            current_estimated_force

        )


        class_label.config(

            text=force_class,

            bg=class_color(

                force_class

            )

        )


        classifier_freq_label.config(

            text=(

                f"{CLASSIFIER_NEURON_USED} "

                f"firing rate: "

                f"{classifier_freq:.2f} Hz"

            )

        )


        force_real_display = (
            current_filtered_force
            if current_filtered_force is not None
            else 0.0
        )

        force_estimated_display = (
            current_estimated_force
            if current_estimated_force is not None
            else 0.0
        )

        error_display = (
            current_error
            if current_error is not None
            else 0.0
        )

        absolute_error_display = (
            current_absolute_error
            if current_absolute_error is not None
            else 0.0
        )

        percentage_error_display = (
            current_percentage_error
            if current_percentage_error is not None
            else 0.0
        )

        mae_display = (
            current_mae
            if current_mae is not None
            else 0.0
        )

        rmse_display = (
            current_rmse
            if current_rmse is not None
            else 0.0
        )

        force_label.config(

            text=(

                f"Real force: "

                f"{force_real_display:.3f} N | "

                f"Estimated: "

                f"{force_estimated_display:.3f} N"

            )

        )


        error_label.config(

            text=(

                f"Error: "

                f"{error_display:.3f} N | "

                f"Absolute error: "

                f"{absolute_error_display:.3f} N | "

                f"Percentage: "

                f"{percentage_error_display:.2f}%"

            )

        )


        if current_adc is not None:

            adc_label.config(

                text=(

                    f"Mean ADC: "

                    f"{current_adc:.1f}"

                )

            )


        for name in bars:

            value = current_freqs[name]


            bars[name]["value"] = min(

                value,

                FREQ_BAR_MAX

            )


            freq_labels[name].config(

                text=(

                    f"{value:.2f} Hz"

                )

            )


    if running:

        window.after(

            GUI_UPDATE_MS,

            update_interface

        )


# =====================================================
# GRAPH UPDATE
# =====================================================

def update_graphs():

    result = copy_plot_data()


    (

        current_time,

        t_force,

        force_values_copy,

        estimated_values_copy,

        error_values_copy,

        t_cn_spikes,

        cn_count

    ) = result


    draw_force_graph(

        force_canvas,

        current_time,

        t_force,

        force_values_copy

    )


    draw_cn_raster(

        cn_raster_canvas,

        current_time,

        t_cn_spikes,

        cn_count

    )


    if running:

        window.after(

            PLOT_UPDATE_MS,

            update_graphs

        )


# =====================================================
# CLOSING
# =====================================================

def close_window():

    global running


    running = False


    window.destroy()


# =====================================================
# INITIALIZATION
# =====================================================

window.protocol(

    "WM_DELETE_WINDOW",

    close_window

)


serial_thread = threading.Thread(

    target=read_serial,

    daemon=True

)


serial_thread.start()


update_interface()


update_graphs()


window.mainloop()