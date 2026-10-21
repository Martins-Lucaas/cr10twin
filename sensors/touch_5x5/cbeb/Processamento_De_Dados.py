import numpy as np
import matplotlib.pyplot as plt
import glob
import os
import csv

from scipy.optimize import curve_fit
from matplotlib.ticker import AutoMinorLocator
from matplotlib.lines import Line2D


# =====================================================
# CONFIG
# =====================================================

PASTA = r"C:\Users\Gabi\OneDrive\Documentos\Artigo_CBEB\dados_carga"

NUMERO_DETALHE = "6"

ROWS = 5
COLS = 5
NUM_TAXELS = 25


# =====================================================
# PADRÃO DE FIGURA PARA ARTIGO IEEE A4
# =====================================================

FIG_COLUNA = 3.38
FIG_DUPLA = 7.01

ALTURA_GRAFICO = 2.55
ALTURA_DETALHADO = 5.40

FONTE_BASE = 8
FONTE_TITULO = 8

TAMANHO_BOLINHA = 2.5
ESPESSURA_BORDA_BOLINHA = 0.4


plt.rcParams.update({

    "font.family": "serif",
    "font.serif": [
        "Times New Roman",
        "Times",
        "DejaVu Serif"
    ],

    "font.size": FONTE_BASE,

    "axes.titlesize": FONTE_TITULO,

    "axes.labelsize": FONTE_BASE,

    "xtick.labelsize": FONTE_BASE,

    "ytick.labelsize": FONTE_BASE,

    "legend.fontsize": FONTE_BASE,

    "figure.dpi": 150,

    "savefig.dpi": 600,

    "axes.linewidth": 0.7,

    "lines.linewidth": 1.2,

    "lines.markersize": TAMANHO_BOLINHA,

    "xtick.major.width": 0.7,

    "ytick.major.width": 0.7,

    "xtick.minor.width": 0.5,

    "ytick.minor.width": 0.5,

    "xtick.direction": "in",

    "ytick.direction": "in",

    "legend.frameon": False,

    "pdf.fonttype": 42,

    "ps.fonttype": 42,

    "mathtext.fontset": "stix",

    "axes.unicode_minus": False,

    "figure.facecolor": "white",

    "axes.facecolor": "white",

    "savefig.facecolor": "white"

})

# =====================================================
# CORES
# =====================================================

COR_FA = "#0B3954"

COR_SA = "#B24C3D"


COR_CNMM = "#3D4F8A"

COR_CNFA = "#8C2D5D"

COR_CNSA = "#006B6B"


COR_FORCA = "#613c4c"

COR_FORCA_FILTRADA = "#000000"


# =====================================================
# ESTILO DOS EIXOS
# =====================================================

def configurar_eixo_artigo(ax, usar_grid=True):

    ax.spines["top"].set_visible(False)

    ax.spines["right"].set_visible(False)


    ax.spines["left"].set_linewidth(0.7)

    ax.spines["bottom"].set_linewidth(0.7)


    ax.tick_params(
        which="major",
        direction="in",
        length=3.2,
        width=0.7,
        pad=2.0,
        top=False,
        right=False
    )


    ax.tick_params(
        which="minor",
        direction="in",
        length=1.8,
        width=0.5,
        top=False,
        right=False
    )


    ax.xaxis.set_minor_locator(
        AutoMinorLocator(2)
    )

    ax.yaxis.set_minor_locator(
        AutoMinorLocator(2)
    )


    if usar_grid:

        ax.grid(
            True,
            which="major",
            linewidth=0.35,
            color="0.88",
            zorder=0
        )

        ax.grid(
            False,
            which="minor"
        )

    else:

        ax.grid(False)


    ax.set_axisbelow(True)
    
# =====================================================
# TÍTULO PADRONIZADO DOS SUBGRÁFICOS
# =====================================================

def titulo_subplot(
    ax,
    texto
):

    ax.set_title(
        texto,
        fontsize= 8,
        loc="left",
        pad=4
    )

# =====================================================
# REGRESSÃO EXPONENCIAL DE SATURAÇÃO
# =====================================================

def modelo_exponencial_saturacao(

    x,

    a,

    b,

    c

):

    return (

        a

        *

        (

            1

            -

            np.exp(

                -b

                *

                x

            )

        )

        +

        c

    )


# =====================================================
# CALCULAR REGRESSÃO E R²
# =====================================================

def calcular_regressao_e_erro(

    x,

    y

):

    x = np.asarray(

        x,

        dtype=float

    )


    y = np.asarray(

        y,

        dtype=float

    )


    mascara = (

        np.isfinite(x)

        &

        np.isfinite(y)

    )


    x_valido = x[

        mascara

    ]


    y_valido = y[

        mascara

    ]


    if len(x_valido) < 3:

        return (

            None,

            np.nan

        )


    try:

        a_inicial = (

            np.max(

                y_valido

            )

            -

            np.min(

                y_valido

            )

        )


        if a_inicial <= 0:

            a_inicial = 1.0


        b_inicial = (

            1.0

            /

            max(

                np.mean(

                    x_valido

                ),

                0.001

            )

        )


        c_inicial = np.min(

            y_valido

        )


        parametros_iniciais = [

            a_inicial,

            b_inicial,

            c_inicial

        ]


        parametros, _ = curve_fit(

            modelo_exponencial_saturacao,

            x_valido,

            y_valido,

            p0=parametros_iniciais,

            maxfev=100000

        )


        y_estimado = modelo_exponencial_saturacao(

            x_valido,

            parametros[0],

            parametros[1],

            parametros[2]

        )


        ss_res = np.sum(

            (

                y_valido

                -

                y_estimado

            )

            **

            2

        )


        ss_tot = np.sum(

            (

                y_valido

                -

                np.mean(

                    y_valido

                )

            )

            **

            2

        )


        if ss_tot == 0:

            r2 = np.nan


        else:

            r2 = (

                1

                -

                (

                    ss_res

                    /

                    ss_tot

                )

            )


        return (

            parametros,

            r2

        )


    except Exception as erro_regressao:

        print(

            "Erro na regressão:",

            erro_regressao

        )


        return (

            None,

            np.nan

        )


# =====================================================
# IMPRIMIR REGRESSÃO
# =====================================================

def imprimir_regressao(

    nome,

    parametros,

    r2

):

    print(

        "\n======================================"

    )


    print(

        "REGRESSÃO EXPONENCIAL DE SATURAÇÃO"

    )


    print(

        nome

    )


    print(

        "======================================"

    )


    if parametros is None:

        print(

            "Regressão não calculada."

        )


        return


    a = parametros[0]

    b = parametros[1]

    c = parametros[2]


    print(

        f"a = {a:.6f}"

    )


    print(

        f"b = {b:.6f}"

    )


    print(

        f"c = {c:.6f}"

    )


    print(

        "\nEquação:"

    )


    print(

        f"FR = {a:.6f} * "

        f"(1 - exp(-{b:.6f} * DeltaF)) "

        f"+ {c:.6f}"

    )


    print(

        f"\nR² = {r2:.6f}"

    )


# =====================================================
# FUNÇÕES DE LEITURA
# =====================================================

def detectar_delimitador(

    caminho

):

    with open(

        caminho,

        "r",

        encoding="utf-8-sig"

    ) as f:

        primeira_linha = f.readline()


    if (

        primeira_linha.count(";")

        >

        primeira_linha.count(",")

    ):

        return ";"


    return ","


def converter_tempo_para_segundos(

    t

):

    t = np.asarray(

        t,

        dtype=float

    )


    if len(t) == 0:

        return t


    if np.nanmax(

        np.abs(t)

    ) > 100000:

        return t / 1e6


    return t


def alinhar_tempos_eventos(

    tempos_raw,

    t0_adc_abs_s,

    duracao_adc_s

):

    tempos_s = converter_tempo_para_segundos(

        tempos_raw

    )


    if len(tempos_s) == 0:

        return tempos_s


    tempos_alinhados = (

        tempos_s

        -

        t0_adc_abs_s

    )


    dentro = (

        (

            tempos_alinhados

            >=

            -0.5

        )

        &

        (

            tempos_alinhados

            <=

            duracao_adc_s

            +

            0.5

        )

    )


    if (

        np.sum(dentro)

        >=

        0.5

        *

        len(

            tempos_alinhados

        )

    ):

        return tempos_alinhados


    return tempos_s


def encontrar_coluna(

    campos,

    opcoes

):

    if campos is None:

        return None


    campos_norm = {}


    for c in campos:

        campos_norm[

            c.strip().lower()

        ] = c


    for op in opcoes:

        op = op.strip().lower()


        if op in campos_norm:

            return campos_norm[op]


    for c in campos:

        c_norm = c.strip().lower()


        for op in opcoes:

            if op.strip().lower() in c_norm:

                return c


    return None


def normalizar_tipo(

    tipo

):

    tipo = str(

        tipo

    ).strip().upper()


    tipo = tipo.replace(

        " ",

        ""

    )


    tipo = tipo.replace(

        "-",

        "_"

    )


    if tipo in [

        "RA",

        "FA"

    ]:

        return "FA"


    if tipo == "SA":

        return "SA"


    if tipo in [

        "CN_MM",

        "CNMM",

        "MM",

        "CN_MULTIMODAL"

    ]:

        return "CN_MM"


    if tipo in [

        "CN_RA",

        "CN_FA",

        "CNRA",

        "CNFA",

        "CN_F",

        "CNF"

    ]:

        return "CN_FA"


    if tipo in [

        "CN_SA",

        "CNSA",

        "CN_S",

        "CNS"

    ]:

        return "CN_SA"


    return tipo


def ler_linhas_csv(

    caminho

):

    delimitador = detectar_delimitador(

        caminho

    )


    with open(

        caminho,

        "r",

        encoding="utf-8-sig",

        newline=""

    ) as f:

        leitor = csv.DictReader(

            f,

            delimiter=delimitador

        )


        linhas = list(

            leitor

        )


        campos = leitor.fieldnames


    return linhas, campos


def ordenar_spikes(

    tempos,

    indices=None

):

    tempos = np.asarray(

        tempos,

        dtype=float

    )


    if indices is None:

        indices = np.zeros(

            len(tempos),

            dtype=int

        )


    else:

        indices = np.asarray(

            indices,

            dtype=int

        )


    if len(tempos) == 0:

        return tempos, indices


    ordem = np.argsort(

        tempos

    )


    tempos = tempos[

        ordem

    ]


    indices = indices[

        ordem

    ]


    if len(indices) > 0:

        if (

            np.min(indices)

            >=

            1

            and

            np.max(indices)

            <=

            NUM_TAXELS

        ):

            indices = indices - 1


    return tempos, indices


# =====================================================
# CARREGAR ADC
# =====================================================

def carregar_adc(

    adc_file

):

    delimitador = detectar_delimitador(

        adc_file

    )


    data = np.loadtxt(

        adc_file,

        delimiter=delimitador,

        skiprows=1

    )


    data = np.atleast_2d(

        data

    )


    if data.shape[1] < 26:

        raise ValueError(

            "Arquivo ADC inválido."

        )


    t_raw = data[:, 0]


    adc_data = data[

        :,

        1:26

    ]


    t_abs_s = converter_tempo_para_segundos(

        t_raw

    )


    t0_abs_s = t_abs_s[0]


    t_rel_s = (

        t_abs_s

        -

        t0_abs_s

    )


    return (

        t_rel_s,

        t_abs_s,

        t0_abs_s,

        adc_data

    )


# =====================================================
# CARREGAR FORÇA
# =====================================================

def carregar_forca(

    force_file

):

    if not os.path.exists(

        force_file

    ):

        print(

            "Aviso: arquivo de força não encontrado:",

            force_file

        )


        return (

            np.array([]),

            np.array([]),

            np.array([])

        )


    linhas, campos = ler_linhas_csv(

        force_file

    )


    col_tempo = encontrar_coluna(

        campos,

        [

            "tempo",

            "time",

            "t",

            "timestamp"

        ]

    )


    col_forca = encontrar_coluna(

        campos,

        [

            "forca",

            "força",

            "force"

        ]

    )


    col_forca_filtrada = encontrar_coluna(

        campos,

        [

            "forca_filtrada",

            "força_filtrada",

            "filtered_force",

            "force_filtered"

        ]

    )


    if (

        col_tempo is None

        or

        col_forca is None

    ):

        print(

            "Colunas de força não encontradas."

        )


        return (

            np.array([]),

            np.array([]),

            np.array([])

        )


    tempos = []

    forcas = []

    forcas_filtradas = []


    for linha in linhas:

        try:

            tempo = float(

                str(

                    linha[

                        col_tempo

                    ]

                ).replace(

                    ",",

                    "."

                )

            )


            forca = float(

                str(

                    linha[

                        col_forca

                    ]

                ).replace(

                    ",",

                    "."

                )

            )


            if col_forca_filtrada is not None:

                forca_filtrada = float(

                    str(

                        linha[

                            col_forca_filtrada

                        ]

                    ).replace(

                        ",",

                        "."

                    )

                )


            else:

                forca_filtrada = forca


            tempos.append(

                tempo

            )


            forcas.append(

                forca

            )


            forcas_filtradas.append(

                forca_filtrada

            )


        except:

            continue


    tempos = converter_tempo_para_segundos(

        np.array(

            tempos

        )

    )


    forcas = np.array(

        forcas,

        dtype=float

    )


    forcas_filtradas = np.array(

        forcas_filtradas,

        dtype=float

    )


    if len(tempos) == 0:

        return (

            np.array([]),

            np.array([]),

            np.array([])

        )


    t0 = tempos[0]


    tempos = tempos - t0


    return (

        tempos,

        forcas,

        forcas_filtradas

    )


# =====================================================
# CARREGAR SPIKES AFERENTES
# =====================================================

def carregar_spikes_aferentes(

    spikes_file,

    t0_adc_abs_s,

    duracao_adc_s

):

    saida = {

        "FA": np.array([]),

        "SA": np.array([]),

        "idx_FA": np.array(

            [],

            dtype=int

        ),

        "idx_SA": np.array(

            [],

            dtype=int

        )

    }


    if not os.path.exists(

        spikes_file

    ):

        print(

            "Aviso: arquivo não encontrado:",

            spikes_file

        )


        return saida


    linhas, campos = ler_linhas_csv(

        spikes_file

    )


    col_tempo = encontrar_coluna(

        campos,

        [

            "tempo",

            "time",

            "t",

            "timestamp"

        ]

    )


    col_tipo = encontrar_coluna(

        campos,

        [

            "tipo",

            "type",

            "neuronio",

            "neuron"

        ]

    )


    col_idx = encontrar_coluna(

        campos,

        [

            "idx",

            "indice",

            "index",

            "taxel"

        ]

    )


    if (

        col_tempo is None

        or

        col_tipo is None

    ):

        return saida


    tempos_FA_raw = []

    tempos_SA_raw = []

    idx_FA = []

    idx_SA = []


    for linha in linhas:

        try:

            tempo = float(

                str(

                    linha[

                        col_tempo

                    ]

                ).replace(

                    ",",

                    "."

                )

            )


        except:

            continue


        tipo = normalizar_tipo(

            linha[

                col_tipo

            ]

        )


        if col_idx is not None:

            try:

                idx = int(

                    float(

                        str(

                            linha[

                                col_idx

                            ]

                        ).replace(

                            ",",

                            "."

                        )

                    )

                )


            except:

                idx = 0


        else:

            idx = 0


        if tipo == "FA":

            tempos_FA_raw.append(

                tempo

            )


            idx_FA.append(

                idx

            )


        elif tipo == "SA":

            tempos_SA_raw.append(

                tempo

            )


            idx_SA.append(

                idx

            )


    tempos_FA = alinhar_tempos_eventos(

        np.array(

            tempos_FA_raw

        ),

        t0_adc_abs_s,

        duracao_adc_s

    )


    tempos_SA = alinhar_tempos_eventos(

        np.array(

            tempos_SA_raw

        ),

        t0_adc_abs_s,

        duracao_adc_s

    )


    tempos_FA, idx_FA = ordenar_spikes(

        tempos_FA,

        idx_FA

    )


    tempos_SA, idx_SA = ordenar_spikes(

        tempos_SA,

        idx_SA

    )


    saida["FA"] = tempos_FA

    saida["SA"] = tempos_SA

    saida["idx_FA"] = idx_FA

    saida["idx_SA"] = idx_SA


    return saida


# =====================================================
# CARREGAR CUNEIFORMES
# =====================================================

def carregar_spikes_cuneiformes(

    cn_file,

    t0_adc_abs_s,

    duracao_adc_s

):

    saida = {

        "CN_MM": np.array([]),

        "CN_FA": np.array([]),

        "CN_SA": np.array([])

    }


    if not os.path.exists(

        cn_file

    ):

        return saida


    linhas, campos = ler_linhas_csv(

        cn_file

    )


    col_tempo = encontrar_coluna(

        campos,

        [

            "tempo",

            "time",

            "t",

            "timestamp"

        ]

    )


    col_tipo = encontrar_coluna(

        campos,

        [

            "tipo",

            "type",

            "neuronio",

            "neuron"

        ]

    )


    tempos_raw = {

        "CN_MM": [],

        "CN_FA": [],

        "CN_SA": []

    }


    for linha in linhas:

        try:

            tempo = float(

                str(

                    linha[

                        col_tempo

                    ]

                ).replace(

                    ",",

                    "."

                )

            )


        except:

            continue


        tipo = normalizar_tipo(

            linha[

                col_tipo

            ]

        )


        if tipo in tempos_raw:

            tempos_raw[

                tipo

            ].append(

                tempo

            )


    for tipo in tempos_raw:

        tempos = alinhar_tempos_eventos(

            np.array(

                tempos_raw[

                    tipo

                ]

            ),

            t0_adc_abs_s,

            duracao_adc_s

        )


        tempos, _ = ordenar_spikes(

            tempos

        )


        saida[

            tipo

        ] = tempos


    return saida


# =====================================================
# DETECTAR EVENTO USANDO FORÇA
# =====================================================

def detectar_evento_forca(

    forca

):

    forca = np.asarray(

        forca,

        dtype=float

    )


    if len(forca) < 10:

        return None, None


    janela = 15


    kernel = np.ones(

        janela

    ) / janela


    sinal_pad = np.pad(

        forca,

        (

            janela // 2,

            janela // 2

        ),

        mode="edge"

    )


    sinal_suave = np.convolve(

        sinal_pad,

        kernel,

        mode="valid"

    )


    n_base = min(

        200,

        len(

            sinal_suave

        )

        //

        4

    )


    if n_base < 5:

        return None, None


    baseline = np.mean(

        sinal_suave[

            :n_base

        ]

    )


    ruido = np.std(

        sinal_suave[

            :n_base

        ]

    )


    if ruido == 0:

        ruido = 0.001


    limiar_press = (

        baseline

        +

        5

        *

        ruido

    )


    limiar_release = (

        baseline

        +

        2

        *

        ruido

    )


    acima = np.where(

        sinal_suave

        >

        limiar_press

    )[0]


    if len(acima) == 0:

        return None, None


    i_press = acima[0]


    depois_press = np.where(

        (

            np.arange(

                len(

                    sinal_suave

                )

            )

            >

            i_press

        )

        &

        (

            sinal_suave

            <

            limiar_release

        )

    )[0]


    if len(depois_press) == 0:

        i_release = acima[-1]


    else:

        i_release = depois_press[0]


    if i_release <= i_press:

        return None, None


    return (

        i_press,

        i_release

    )


# =====================================================
# FREQUÊNCIA MÉDIA
# =====================================================

def calcular_freq_isi(

    spike_times,

    t_ini,

    t_fim

):

    spike_times = np.asarray(

        spike_times

    )


    spikes_janela = spike_times[

        (

            spike_times

            >=

            t_ini

        )

        &

        (

            spike_times

            <=

            t_fim

        )

    ]


    if len(

        spikes_janela

    ) < 2:

        return 0.0


    isi = np.diff(

        spikes_janela

    )


    isi = isi[

        isi > 0

    ]


    if len(isi) == 0:

        return 0.0


    return (

        1.0

        /

        np.mean(

            isi

        )

    )


# =====================================================
# PARTE 1
# MEAN FIRING RATE x DELTA DE FORÇA
# =====================================================

resultados = []


arquivos_adc = glob.glob(

    os.path.join(

        PASTA,

        "*_adc.csv"

    )

)


for adc_file in arquivos_adc:

    nome = os.path.basename(

        adc_file

    )


    label = nome.replace(

        "_adc.csv",

        ""

    )


    force_file = os.path.join(

        PASTA,

        f"{label}_forca.csv"

    )


    spikes_file = os.path.join(

        PASTA,

        f"{label}_spikes.csv"

    )


    cn_file = os.path.join(

        PASTA,

        f"{label}_cuneiformes.csv"

    )


    print(

        "\n=============================="

    )


    print(

        "Experimento:",

        label

    )


    try:

        t_adc, t_abs, t0_abs, adc_data = carregar_adc(

            adc_file

        )


        t_force, force, force_filtered = carregar_forca(

            force_file

        )


    except Exception as e:

        print(

            "Erro ao carregar dados:",

            e

        )


        continue


    if len(force) == 0:

        print(

            "Força não encontrada."

        )


        continue


    i_press, i_release = detectar_evento_forca(

        force_filtered

    )


    if i_press is None:

        print(

            "Toque não encontrado na força."

        )


        continue


    tempo_press = t_force[

        i_press

    ]


    tempo_release = t_force[

        i_release

    ]


    tempo_contato = (

        tempo_release

        -

        tempo_press

    )


    if tempo_contato <= 0:

        continue


    baseline_force = np.mean(

        force_filtered[

            :i_press

        ]

    )


    force_toque_medio = np.mean(

        force_filtered[

            i_press:i_release + 1

        ]

    )


    delta_forca = np.abs(

        force_toque_medio

        -

        baseline_force

    )


    spikes_af = carregar_spikes_aferentes(

        spikes_file,

        t0_abs,

        t_adc[-1]

    )


    spikes_cn = carregar_spikes_cuneiformes(

        cn_file,

        t0_abs,

        t_adc[-1]

    )


    freq_FA = calcular_freq_isi(

        spikes_af["FA"],

        tempo_press,

        tempo_release

    )


    freq_SA = calcular_freq_isi(

        spikes_af["SA"],

        tempo_press,

        tempo_release

    )


    freq_CNMM = calcular_freq_isi(

        spikes_cn["CN_MM"],

        tempo_press,

        tempo_release

    )


    freq_CNFA = calcular_freq_isi(

        spikes_cn["CN_FA"],

        tempo_press,

        tempo_release

    )


    freq_CNSA = calcular_freq_isi(

        spikes_cn["CN_SA"],

        tempo_press,

        tempo_release

    )


    resultados.append(

        [

            label,

            delta_forca,

            freq_FA,

            freq_SA,

            freq_CNMM,

            freq_CNFA,

            freq_CNSA

        ]

    )


    print(

        "PRESS =",

        tempo_press,

        "s"

    )


    print(

        "RELEASE =",

        tempo_release,

        "s"

    )


    print(

        "Tempo de contato =",

        tempo_contato,

        "s"

    )


    print(

        "Delta força =",

        delta_forca,

        "N"

    )


    print(

        "Mean Firing Rate FA =",

        freq_FA,

        "Hz"

    )


    print(

        "Mean Firing Rate SA =",

        freq_SA,

        "Hz"

    )


    print(

        "Mean Firing Rate CN_MM =",

        freq_CNMM,

        "Hz"

    )


    print(

        "Mean Firing Rate CN_FA =",

        freq_CNFA,

        "Hz"

    )


    print(

        "Mean Firing Rate CN_SA =",

        freq_CNSA,

        "Hz"

    )


# =====================================================
# GRÁFICOS GERAIS
# =====================================================

if len(

    resultados

) > 0:


    resultados = np.array(

        resultados,

        dtype=object

    )


    labels = resultados[:, 0]


    delta_forca = resultados[:, 1].astype(float)


    freq_FA = resultados[:, 2].astype(float)


    freq_SA = resultados[:, 3].astype(float)


    freq_CNMM = resultados[:, 4].astype(float)


    freq_CNFA = resultados[:, 5].astype(float)


    freq_CNSA = resultados[:, 6].astype(float)


    # =================================================
    # ORDENAR DADOS PELA FORÇA
    # =================================================

    ordem = np.argsort(

        delta_forca

    )


    labels = labels[

        ordem

    ]


    delta_forca = delta_forca[

        ordem

    ]


    freq_FA = freq_FA[

        ordem

    ]


    freq_SA = freq_SA[

        ordem

    ]


    freq_CNMM = freq_CNMM[

        ordem

    ]


    freq_CNFA = freq_CNFA[

        ordem

    ]


    freq_CNSA = freq_CNSA[

        ordem

    ]


    # =================================================
    # REGRESSÕES
    # =================================================

    parametros_FA, r2_FA = calcular_regressao_e_erro(

        delta_forca,

        freq_FA

    )


    parametros_SA, r2_SA = calcular_regressao_e_erro(

        delta_forca,

        freq_SA

    )


    parametros_CNMM, r2_CNMM = calcular_regressao_e_erro(

        delta_forca,

        freq_CNMM

    )


    parametros_CNFA, r2_CNFA = calcular_regressao_e_erro(

        delta_forca,

        freq_CNFA

    )


    parametros_CNSA, r2_CNSA = calcular_regressao_e_erro(

        delta_forca,

        freq_CNSA

    )


    # =================================================
    # IMPRIMIR REGRESSÕES NO TERMINAL
    # =================================================

    imprimir_regressao(

        "FA",

        parametros_FA,

        r2_FA

    )


    imprimir_regressao(

        "SA",

        parametros_SA,

        r2_SA

    )


    imprimir_regressao(

        "CN_MM",

        parametros_CNMM,

        r2_CNMM

    )


    imprimir_regressao(

        "CN_FA",

        parametros_CNFA,

        r2_CNFA

    )


    imprimir_regressao(

        "CN_SA",

        parametros_CNSA,

        r2_CNSA

    )


    # =================================================
    # CURVA CONTÍNUA
    # =================================================

    x_curva = np.linspace(

        0,

        np.max(delta_forca),

        500

    )


    # =================================================
    # GRÁFICO FA / SA
    # =================================================

    fig, ax = plt.subplots(
        figsize=(
            FIG_COLUNA,
            ALTURA_GRAFICO
        )
    )


    ax = plt.gca()


    configurar_eixo_artigo(

        ax,

        usar_grid=True

    )


    # =================================================
    # PONTOS FA
    # =================================================

    ax.scatter(

        delta_forca,

        freq_FA,

        s=0.4,

        color=COR_FA,

        label="FA",

        zorder=3

    )

    # =================================================
    # PONTOS SA
    # =================================================

    ax.scatter(

        delta_forca,

        freq_SA,

        s=0.4,

        color=COR_SA,

        label="SA",

        zorder=3

    )


    # =================================================
    # CURVA FA
    # =================================================

    if parametros_FA is not None:

        y_curva_FA = modelo_exponencial_saturacao(

            x_curva,

            parametros_FA[0],

            parametros_FA[1],

            parametros_FA[2]

        )


        ax.plot(

            x_curva,

            y_curva_FA,

            linewidth=1.0,

            color=COR_FA

        )


    # =================================================
    # CURVA SA
    # =================================================

    if parametros_SA is not None:

        y_curva_SA = modelo_exponencial_saturacao(

            x_curva,

            parametros_SA[0],

            parametros_SA[1],

            parametros_SA[2]

        )


        ax.plot(

            x_curva,

            y_curva_SA,

            linewidth=1.0,

            color=COR_SA

        )


    plt.xlabel(

        "Mean Force Variation During Touch (N)"

    )


    plt.ylabel(

        "Mean Firing Rate (Hz)"

    )


    titulo_subplot(

        ax,

        "Mean Firing Rate of Mechanoreceptors During Touch"

    )


    plt.xlim(
        0,
        np.max(delta_forca) * 1.05
    )


    # =================================================
    # LEGENDA CN (manual para artigo IEEE)
    # =================================================

    legenda_CN = [

        Line2D(
            [0],
            [0],
            color=COR_SA,
            linewidth=1.2,
            label="SA"
        ),

        Line2D(
            [0],
            [0],
            color=COR_FA,
            linewidth=1.2,
            label="FA"
        )

    ]


    ax.legend(
        handles=legenda_CN,
        frameon=False,
        loc="best"
    )


    plt.tight_layout(
        pad=0.55
    )


    plt.savefig(

        os.path.join(

            PASTA,

            "fig_FA_SA_delta_forca_dados_reais.png"

        ),

        dpi=300,

        bbox_inches="tight"

    )


    plt.savefig(

        os.path.join(

            PASTA,

            "fig_FA_SA_delta_forca_dados_reais.pdf"

        ),

        bbox_inches="tight"

    )


    plt.show(

        block=False

    )


    # =================================================
    # GRÁFICO CUNEIFORMES
    # =================================================

    fig, ax = plt.subplots(
        figsize=(
            FIG_COLUNA,
            ALTURA_GRAFICO
        )
    )


    ax = plt.gca()


    configurar_eixo_artigo(

        ax,

        usar_grid=True

    )


    # =================================================
    # PONTOS CN_MM
    # =================================================

    ax.scatter(
        delta_forca,
        freq_CNMM,
        s=8,
        color=COR_CNMM,
        zorder=3
    )


    # =================================================
    # PONTOS CN_FA
    # =================================================

    ax.scatter(
        delta_forca,
        freq_CNFA,
        s=8,
        color=COR_CNFA,
        zorder=3
    )


    # =================================================
    # PONTOS CN_SA
    # =================================================

    ax.scatter(
        delta_forca,
        freq_CNSA,
        s=8,
        color=COR_CNSA,
        zorder=3
    )

    # =================================================
    # CURVA CN_MM
    # =================================================

    if parametros_CNMM is not None:

        y_curva_CNMM = modelo_exponencial_saturacao(

            x_curva,

            parametros_CNMM[0],

            parametros_CNMM[1],

            parametros_CNMM[2]

        )


        ax.plot(

            x_curva,

            y_curva_CNMM,

            linewidth=1.0,

            color=COR_CNMM

        )


    # =================================================
    # CURVA CN_FA
    # =================================================

    if parametros_CNFA is not None:

        y_curva_CNFA = modelo_exponencial_saturacao(

            x_curva,

            parametros_CNFA[0],

            parametros_CNFA[1],

            parametros_CNFA[2]

        )


        ax.plot(

            x_curva,

            y_curva_CNFA,

            linewidth=1.0,

            color=COR_CNFA

        )


    # =================================================
    # CURVA CN_SA
    # =================================================

    if parametros_CNSA is not None:

        y_curva_CNSA = modelo_exponencial_saturacao(

            x_curva,

            parametros_CNSA[0],

            parametros_CNSA[1],

            parametros_CNSA[2]

        )


        ax.plot(

            x_curva,

            y_curva_CNSA,

            linewidth=1.0,

            color=COR_CNSA

        )


    plt.xlabel(

        "Mean Force Variation During Touch (N)"

    )


    plt.ylabel(

        "Mean Firing Rate (Hz)"

    )


    titulo_subplot(

        ax,

        "Mean Firing Rate of Cuneate Neurons During Touch"

    )


# =================================================
# LEGENDA DOS CUNEIFORMES
# =================================================

    legenda_CN = [

        Line2D(
            [0],
            [0],
            color=COR_CNMM,
            linewidth=1.2,
            label="CN_MM"
        ),

        Line2D(
            [0],
            [0],
            color=COR_CNFA,
            linewidth=1.2,
            label="CN_FA"
        ),

        Line2D(
            [0],
            [0],
            color=COR_CNSA,
            linewidth=1.2,
            label="CN_SA"
        )

    ]


    ax.legend(
        handles=legenda_CN,
        frameon=False,
        fontsize=8,
        loc="best"
    )


    plt.savefig(

        os.path.join(

            PASTA,

            "fig_CN_delta_forca_dados_reais.png"

        ),

        dpi=300,

        bbox_inches="tight"

    )


    plt.savefig(

        os.path.join(

            PASTA,

            "fig_CN_delta_forca_dados_reais.pdf"

        ),

        bbox_inches="tight"

    )


    plt.show(

        block=False

    )


else:

    print(

        "\nNenhum experimento válido encontrado."

    )


# =====================================================
# PARTE 2
# GRÁFICO DETALHADO
# =====================================================

adc_file = os.path.join(

    PASTA,

    f"{NUMERO_DETALHE}_adc.csv"

)


force_file = os.path.join(

    PASTA,

    f"{NUMERO_DETALHE}_forca.csv"

)


spikes_file = os.path.join(

    PASTA,

    f"{NUMERO_DETALHE}_spikes.csv"

)


cn_file = os.path.join(

    PASTA,

    f"{NUMERO_DETALHE}_cuneiformes.csv"

)


print(

    "\n==================================="

)


print(

    "GRÁFICO DETALHADO"

)


print(

    "ADC:",

    adc_file

)


print(

    "FORÇA:",

    force_file

)


if not os.path.exists(

    adc_file

):

    print(

        "Arquivo ADC não encontrado:",

        adc_file

    )


elif not os.path.exists(

    force_file

):

    print(

        "Arquivo de força não encontrado:",

        force_file

    )


else:

    (

        t_original,

        t_abs_original,

        t0_abs_original,

        taxels_original_adc

    ) = carregar_adc(

        adc_file

    )


    (

        t_force,

        force_original,

        force_filtered_original

    ) = carregar_forca(

        force_file

    )


    i_press_force, i_release_force = detectar_evento_forca(

        force_filtered_original

    )


    if i_press_force is None:

        print(

            "Toque não detectado na força."

        )


        i_press_force = 0


        i_release_force = len(

            t_force

        ) - 1


    else:

        print(

            "PRESS =",

            t_force[

                i_press_force

            ],

            "s"

        )


        print(

            "RELEASE =",

            t_force[

                i_release_force

            ],

            "s"

        )


    # =================================================
    # Janela de visualização
    # =================================================

    # Tempo mostrado antes do início do toque
    tempo_antes = 0.5      # 100 ms

    # Tempo mostrado após o término do toque
    tempo_depois = 0.01    # 150 ms

    # Início da janela
    t_ini = max(
        0,
        t_force[i_press_force] - tempo_antes
    )

    # Fim da janela (corta logo após o release)
    t_fim = t_force[i_release_force]
    

    idx_force_ini = np.searchsorted(

        t_force,

        t_ini

    )


    idx_force_fim = np.searchsorted(

        t_force,

        t_fim,

        side="right"

    )


    idx_adc_ini = np.searchsorted(

        t_original,

        t_ini

    )


    idx_adc_fim = np.searchsorted(

        t_original,

        t_fim,

        side="right"

    )


    t0 = t_ini


    t = t_original[

        idx_adc_ini:idx_adc_fim

    ] - t0


    taxels_adc = taxels_original_adc[

        idx_adc_ini:idx_adc_fim,

        :

    ]


    t_forca = t_force[

        idx_force_ini:idx_force_fim

    ] - t0


    forca = force_original[

        idx_force_ini:idx_force_fim

    ]


    forca_filtrada = force_filtered_original[

        idx_force_ini:idx_force_fim

    ]


    spikes_af = carregar_spikes_aferentes(

        spikes_file,

        t0_abs_original,

        t_original[-1]

    )


    spikes_cn = carregar_spikes_cuneiformes(

        cn_file,

        t0_abs_original,

        t_original[-1]

    )


    t_janela_ini = t_ini

    t_janela_fim = t_fim


    mask_FA = (

        (

            spikes_af["FA"]

            >=

            t_janela_ini

        )

        &

        (

            spikes_af["FA"]

            <=

            t_janela_fim

        )

    )


    mask_SA = (

        (

            spikes_af["SA"]

            >=

            t_janela_ini

        )

        &

        (

            spikes_af["SA"]

            <=

            t_janela_fim

        )

    )


    t_FA = spikes_af["FA"][

        mask_FA

    ] - t0


    idx_FA = spikes_af["idx_FA"][

        mask_FA

    ]


    t_SA = spikes_af["SA"][

        mask_SA

    ] - t0


    idx_SA = spikes_af["idx_SA"][

        mask_SA

    ]


    cn_mm_times = spikes_cn["CN_MM"][

        (

            spikes_cn["CN_MM"]

            >=

            t_janela_ini

        )

        &

        (

            spikes_cn["CN_MM"]

            <=

            t_janela_fim

        )

    ] - t0


    cn_fa_times = spikes_cn["CN_FA"][

        (

            spikes_cn["CN_FA"]

            >=

            t_janela_ini

        )

        &

        (

            spikes_cn["CN_FA"]

            <=

            t_janela_fim

        )

    ] - t0


    cn_sa_times = spikes_cn["CN_SA"][

        (

            spikes_cn["CN_SA"]

            >=

            t_janela_ini

        )

        &

        (

            spikes_cn["CN_SA"]

            <=

            t_janela_fim

        )

    ] - t0


    n_taxels = taxels_adc.shape[1]


    fig, axs = plt.subplots(
        4,
        1,
        figsize=(
            FIG_COLUNA,
            ALTURA_DETALHADO
        ),

        sharex=True,

        gridspec_kw={

            "height_ratios": [

                0.6,

                1.0,

                3.2,

                1.0

            ]

        }

    )


    for ax in axs:

        configurar_eixo_artigo(

            ax,

            usar_grid=False

        )
        
    # Letras dos subgráficos
    letras = ["a)", "b)", "c)", "d)"]

    for ax, letra in zip(axs, letras):
        ax.text(
            -0.15,
            1.05,
            letra,
            transform=ax.transAxes,
            fontsize=10,
            fontweight="bold",
            va="top",
            ha="right"
        )

    # =================================================
    # 1) FORÇA FILTRADA
    # =================================================

    axs[0].plot(

        t_forca,

        forca_filtrada,

        linewidth=1.0,

        color=COR_FORCA_FILTRADA

    )


    titulo_subplot(

        axs[0],

        "Force Applied to the Sensor"

    )


    axs[0].set_ylabel(

        "Force (N)"

    )


# =================================================
# 2) SINAIS DOS TAXELS
# =================================================

    for i in range(n_taxels):

        # Converte ADC para tensão e inverte o sinal (0 → 3.3 V)
        sinal = 3.3 - (
            taxels_adc[:, i] * 3.3 / 4095.0
        )

        axs[1].plot(
            t,
            sinal,
            linewidth=0.5,
            alpha=0.75
        )

    titulo_subplot(
        axs[1],
        f"Voltage Signal of {n_taxels} Sensors"
    )

    axs[1].set_ylabel(
        "Voltage (V)"
    )

    axs[1].set_ylim(0, 3.3)


    # =================================================
    # 3) RASTER FA E SA
    # =================================================

    for i in range(

        n_taxels

    ):

        tempos_i = t_SA[

            idx_SA == i

        ]


        if len(

            tempos_i

        ) > 0:

            axs[2].eventplot(

                tempos_i,

                lineoffsets=i + 1,

                linelengths=0.85,

                linewidths=1.1,

                colors=COR_SA

            )


    for i in range(

        n_taxels

    ):

        tempos_i = t_FA[

            idx_FA == i

        ]


        if len(

            tempos_i

        ) > 0:

            axs[2].eventplot(

                tempos_i,

                lineoffsets=n_taxels + i + 1,

                linelengths=0.85,

                linewidths=1.1,

                colors=COR_FA

            )


    titulo_subplot(

        axs[2],

        "FA and SA Spike Raster"

    )


    yticks = [

        5,

        10,

        15,

        20,

        25,

        30,

        35,

        40,

        45,

        50

    ]


    yticklabels = [

        "SA5",

        "SA10",

        "SA15",

        "SA20",

        "SA25",

        "FA5",

        "FA10",

        "FA15",

        "FA20",

        "FA25"

    ]


    axs[2].set_yticks(

        yticks

    )


    axs[2].set_yticklabels(

        yticklabels

    )


    axs[2].set_ylim(

        0.3,

        50.7

    )


    axs[2].grid(

        False

    )


    axs[2].text(

        -0.04,

        13,

        "SA",

        transform=axs[2].get_yaxis_transform(),

        fontsize=5,

        fontweight="bold",

        color=COR_SA,

        ha="right",

        va="center"

    )


    axs[2].text(

        -0.04,

        38,

        "FA",

        transform=axs[2].get_yaxis_transform(),

        fontsize=5,

        fontweight="bold",

        color=COR_FA,

        ha="right",

        va="center"

    )


    # =================================================
    # 4) RASTER CUNEIFORMES
    # =================================================

    axs[3].eventplot(

        cn_sa_times,

        lineoffsets=1,

        linelengths=0.5,

        linewidths=0.7,

        colors=COR_CNSA

    )


    axs[3].eventplot(

        cn_fa_times,

        lineoffsets=2,

        linelengths=0.5,

        linewidths=0.7,

        colors=COR_CNFA

    )


    axs[3].eventplot(

        cn_mm_times,

        lineoffsets=3,

        linelengths=0.5,

        linewidths=0.7,

        colors=COR_CNMM

    )


    titulo_subplot(

        axs[3],

        "Cuneate Neuron Spike Raster"

    )


    axs[3].set_yticks(

        [

            1,

            2,

            3

        ]

    )


    axs[3].set_yticklabels(

        [

            "CN_SA",

            "CN_FA",

            "CN_MM"

        ]

    )


    axs[3].set_ylim(

        0.3,

        3.7

    )


    axs[3].set_xlabel(

        "Time (s)"

    )


    axs[3].grid(

        False

    )


    # =================================================
    # ESPAÇAMENTO
    # =================================================
    
    fig.subplots_adjust(
        left=0.235,
        right=0.980,
        bottom=0.085,
        top=0.975,
        hspace=0.44
    )


    plt.savefig(

        os.path.join(

            PASTA,

            f"figura_experimento_{NUMERO_DETALHE}_dados_reais.png"

        ),

        dpi=300,

        bbox_inches="tight"

    )


    plt.savefig(

        os.path.join(

            PASTA,

            f"figura_experimento_{NUMERO_DETALHE}_dados_reais.pdf"

        ),

        bbox_inches="tight"

    )


    plt.show()