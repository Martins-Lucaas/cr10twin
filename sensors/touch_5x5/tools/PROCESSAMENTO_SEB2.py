import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

PASTA = r"c:\Users\Gabi\OneDrive\Documentos\Artigo_CBEB\dados_carga"

NOME_EXPERIMENTO = "6"

NUM_TAXELS = 25

TEMPO_TOTAL = 15.0


# ============================================================
# JANELA DO TOQUE
# ============================================================

INICIO_TOQUE = 5.0
FIM_TOQUE = 10.0

MARGEM_ANTES = 0.1
MARGEM_DEPOIS = 0.8

TEMPO_INICIAL_PLOT = INICIO_TOQUE - MARGEM_ANTES
TEMPO_FINAL_PLOT = FIM_TOQUE + MARGEM_DEPOIS


# ============================================================
# ADC
# ============================================================

ADC_MAX = 4095.0
V_MAX = 3.3


# ============================================================
# ARQUIVOS
# ============================================================

ARQUIVO_SPIKES = os.path.join(
    PASTA,
    f"{NOME_EXPERIMENTO}_spikes.csv"
)

ARQUIVO_FORCA = os.path.join(
    PASTA,
    f"{NOME_EXPERIMENTO}_forca.csv"
)

ARQUIVO_ADC = os.path.join(
    PASTA,
    f"{NOME_EXPERIMENTO}_adc.csv"
)


# ============================================================
# CORES
# ============================================================

COR_FORCA = "#333333"
COR_FA = "#0B3954"
COR_SA = "#B24C3D"


# ============================================================
# ESTILO
# ============================================================

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman"],
    "font.size": 9,
    "axes.labelsize": 10,
    "axes.titlesize": 10,
    "xtick.labelsize": 9,
    "ytick.labelsize": 8,
})


# ============================================================
# VERIFICAR ARQUIVOS
# ============================================================

for arquivo in [
    ARQUIVO_SPIKES,
    ARQUIVO_FORCA,
    ARQUIVO_ADC
]:

    if not os.path.exists(arquivo):

        raise FileNotFoundError(
            f"Arquivo não encontrado:\n{arquivo}"
        )


# ============================================================
# CARREGAR SPIKES
# ============================================================

df_spikes = pd.read_csv(
    ARQUIVO_SPIKES
)

df_spikes["tempo"] = pd.to_numeric(
    df_spikes["tempo"],
    errors="coerce"
)

df_spikes["idx"] = pd.to_numeric(
    df_spikes["idx"],
    errors="coerce"
)

df_spikes["tipo"] = (
    df_spikes["tipo"]
    .astype(str)
    .str.strip()
    .str.upper()
)

df_spikes = df_spikes.dropna(
    subset=[
        "tempo",
        "idx",
        "tipo"
    ]
).copy()

df_spikes["idx"] = (
    df_spikes["idx"]
    .astype(int)
)


# ============================================================
# CONVERTER RA PARA FA
# ============================================================

df_spikes["tipo"] = (
    df_spikes["tipo"]
    .replace({
        "RA": "FA"
    })
)


# ============================================================
# MANTER SOMENTE FA E SA
# ============================================================

df_spikes = df_spikes[
    df_spikes["tipo"].isin(
        ["FA", "SA"]
    )
].copy()


# ============================================================
# CARREGAR FORÇA
# ============================================================

df_forca = pd.read_csv(
    ARQUIVO_FORCA
)

df_forca["tempo"] = pd.to_numeric(
    df_forca["tempo"],
    errors="coerce"
)

df_forca["forca"] = pd.to_numeric(
    df_forca["forca"],
    errors="coerce"
)

df_forca["forca_filtrada"] = pd.to_numeric(
    df_forca["forca_filtrada"],
    errors="coerce"
)

df_forca = df_forca.dropna(
    subset=[
        "tempo",
        "forca_filtrada"
    ]
).copy()


# ============================================================
# CARREGAR ADC
# ============================================================

df_adc = pd.read_csv(
    ARQUIVO_ADC
)

df_adc["tempo"] = pd.to_numeric(
    df_adc["tempo"],
    errors="coerce"
)


# ============================================================
# COLUNAS DOS 25 TAXELS
# ============================================================

COLUNAS_ADC = [
    f"taxel_{i}"
    for i in range(NUM_TAXELS)
]


# ============================================================
# CONVERTER ADC PARA NÚMERO
# ============================================================

for coluna in COLUNAS_ADC:

    if coluna not in df_adc.columns:

        raise ValueError(
            f"Coluna {coluna} não encontrada "
            f"em {ARQUIVO_ADC}"
        )

    df_adc[coluna] = pd.to_numeric(
        df_adc[coluna],
        errors="coerce"
    )


df_adc = df_adc.dropna(
    subset=["tempo"]
).copy()


# ============================================================
# ENCONTRAR BLOCO PRINCIPAL DE 15 SEGUNDOS
# ============================================================

tempos_adc = np.sort(
    df_adc["tempo"].values
)

if len(tempos_adc) == 0:

    raise ValueError(
        "Nenhuma amostra ADC encontrada."
    )


melhor_inicio = None
melhor_fim = None
maior_quantidade = 0

j = 0


for i in range(len(tempos_adc)):

    if j < i:
        j = i

    while (
        j < len(tempos_adc)
        and
        tempos_adc[j] - tempos_adc[i] <= TEMPO_TOTAL
    ):

        j += 1

    quantidade = j - i

    if quantidade > maior_quantidade:

        maior_quantidade = quantidade

        melhor_inicio = tempos_adc[i]

        melhor_fim = tempos_adc[j - 1]


# ============================================================
# INFORMAÇÕES
# ============================================================

print()
print("=" * 60)
print("COLETA IDENTIFICADA")
print("=" * 60)

print(
    f"Início STM32: {melhor_inicio:.6f} s"
)

print(
    f"Fim STM32: {melhor_fim:.6f} s"
)

print(
    f"Duração: "
    f"{melhor_fim - melhor_inicio:.3f} s"
)

print()

print(
    f"Janela mostrada: "
    f"{TEMPO_INICIAL_PLOT:.1f} até "
    f"{TEMPO_FINAL_PLOT:.1f} s"
)

print("=" * 60)


# ============================================================
# RECORTAR ADC DA COLETA COMPLETA
# ============================================================

df_adc_recorte = df_adc[
    (df_adc["tempo"] >= melhor_inicio)
    &
    (
        df_adc["tempo"]
        <= melhor_inicio + TEMPO_TOTAL
    )
].copy()


df_adc_recorte["tempo_rel"] = (
    df_adc_recorte["tempo"]
    - melhor_inicio
)


df_adc_recorte = (
    df_adc_recorte
    .sort_values("tempo_rel")
    .reset_index(drop=True)
)


# ============================================================
# RECORTAR FORÇA
# ============================================================

df_forca_recorte = df_forca[
    (df_forca["tempo"] >= melhor_inicio)
    &
    (
        df_forca["tempo"]
        <= melhor_inicio + TEMPO_TOTAL
    )
].copy()


if len(df_forca_recorte) > 0:

    df_forca_recorte["tempo_rel"] = (
        df_forca_recorte["tempo"]
        - melhor_inicio
    )

else:

    print()
    print(
        "ATENÇÃO: força possui outra "
        "referência temporal."
    )

    t0_forca = df_forca["tempo"].min()

    df_forca_recorte = df_forca.copy()

    df_forca_recorte["tempo_rel"] = (
        df_forca_recorte["tempo"]
        - t0_forca
    )

    df_forca_recorte = df_forca_recorte[
        (df_forca_recorte["tempo_rel"] >= 0)
        &
        (
            df_forca_recorte["tempo_rel"]
            <= TEMPO_TOTAL
        )
    ].copy()


# ============================================================
# RECORTAR SPIKES
# ============================================================

df_spikes_recorte = df_spikes[
    (df_spikes["tempo"] >= melhor_inicio)
    &
    (
        df_spikes["tempo"]
        <= melhor_inicio + TEMPO_TOTAL
    )
].copy()


if len(df_spikes_recorte) > 0:

    df_spikes_recorte["tempo_rel"] = (
        df_spikes_recorte["tempo"]
        - melhor_inicio
    )

else:

    print()
    print(
        "ATENÇÃO: spikes possuem outra "
        "referência temporal."
    )

    t0_spikes = df_spikes["tempo"].min()

    df_spikes_recorte = df_spikes.copy()

    df_spikes_recorte["tempo_rel"] = (
        df_spikes_recorte["tempo"]
        - t0_spikes
    )

    df_spikes_recorte = df_spikes_recorte[
        (df_spikes_recorte["tempo_rel"] >= 0)
        &
        (
            df_spikes_recorte["tempo_rel"]
            <= TEMPO_TOTAL
        )
    ].copy()


# ============================================================
# SEPARAR FA E SA
# ============================================================

df_fa = df_spikes_recorte[
    df_spikes_recorte["tipo"] == "FA"
].copy()

df_sa = df_spikes_recorte[
    df_spikes_recorte["tipo"] == "SA"
].copy()


# ============================================================
# CONVERTER ADC PARA TENSÃO INVERTIDA
# ============================================================
#
# SENSOR SOLTO    -> tensão baixa
# SENSOR APERTADO -> tensão alta
#
# ============================================================

COLUNAS_V = []


for i, coluna in enumerate(COLUNAS_ADC):

    coluna_v = f"taxel_{i}_V"

    df_adc_recorte[coluna_v] = (
        V_MAX
        -
        (
            df_adc_recorte[coluna]
            / ADC_MAX
            * V_MAX
        )
    )

    COLUNAS_V.append(
        coluna_v
    )


# ============================================================
# REMOVER TEMPOS DUPLICADOS
# ============================================================

tempo_adc = (
    df_adc_recorte["tempo_rel"]
    .to_numpy(dtype=float)
)


if len(tempo_adc) > 1:

    mask_tempo = np.concatenate(
        (
            [True],
            np.diff(tempo_adc) > 0
        )
    )

    df_adc_recorte = (
        df_adc_recorte.loc[
            mask_tempo
        ]
        .reset_index(drop=True)
    )


tempo_adc = (
    df_adc_recorte["tempo_rel"]
    .to_numpy(dtype=float)
)


if len(tempo_adc) < 2:

    raise ValueError(
        "Poucas amostras de ADC "
        "para calcular a derivada."
    )


# ============================================================
# CALCULAR DERIVADA
# ============================================================

derivadas = {}


for coluna_v in COLUNAS_V:

    tensao = (
        df_adc_recorte[coluna_v]
        .to_numpy(dtype=float)
    )

    derivadas[coluna_v] = np.gradient(
        tensao,
        tempo_adc
    )


# ============================================================
# CRIAR FIGURA
# ============================================================

fig, (
    ax_forca,
    ax_adc,
    ax_derivada,
    ax_fa,
    ax_sa
) = plt.subplots(
    5,
    1,
    figsize=(7.01, 10),
    sharex=True,
    gridspec_kw={
        "height_ratios": [
            1.2,
            1.5,
            1.5,
            1,
            1
        ]
    }
)


# ============================================================
# 1. FORÇA
# ============================================================

ax_forca.plot(
    df_forca_recorte["tempo_rel"],
    df_forca_recorte["forca_filtrada"],
    color=COR_FORCA,
    linewidth=1.3
)

ax_forca.set_ylabel(
    "Força"
)

ax_forca.set_title(
    "Força aplicada"
)

ax_forca.grid(
    axis="x",
    alpha=0.15
)


# ============================================================
# 2. TENSÃO DOS 25 TAXELS
# ============================================================

for coluna_v in COLUNAS_V:

    ax_adc.plot(
        df_adc_recorte["tempo_rel"],
        df_adc_recorte[coluna_v],
        linewidth=0.7,
        alpha=0.65
    )


ax_adc.set_ylabel(
    "Tensão (V)"
)

ax_adc.set_title(
    "Sinais dos 25 taxels"
)

ax_adc.set_ylim(
    0,
    3.3
)

ax_adc.set_yticks(
    np.arange(
        0,
        3.31,
        0.5
    )
)

ax_adc.grid(
    axis="x",
    alpha=0.15
)


# ============================================================
# 3. DERIVADA DA TENSÃO
# ============================================================

for coluna_v in COLUNAS_V:

    ax_derivada.plot(
        tempo_adc,
        derivadas[coluna_v],
        linewidth=0.7,
        alpha=0.65
    )


ax_derivada.axhline(
    0,
    color="black",
    linewidth=0.6,
    alpha=0.5
)

ax_derivada.set_ylabel(
    "dV/dt (V/s)"
)

ax_derivada.set_title(
    "Derivada temporal da tensão"
)

ax_derivada.grid(
    axis="x",
    alpha=0.15
)


# ============================================================
# 4. RASTER PLOT FA
# ============================================================

for taxel in range(NUM_TAXELS):

    spikes = df_fa.loc[
        df_fa["idx"] == taxel,
        "tempo_rel"
    ].values

    # Mostrar somente spikes dentro da janela
    spikes = spikes[
        (spikes >= TEMPO_INICIAL_PLOT)
        &
        (spikes <= TEMPO_FINAL_PLOT)
    ]

    if len(spikes) > 0:

        ax_fa.eventplot(
            spikes,
            lineoffsets=taxel,
            linelengths=0.8,
            linewidths=0.8,
            colors=COR_FA
        )


ax_fa.set_title(
    "FA"
)

ax_fa.set_ylabel(
    "Taxel"
)

ax_fa.set_ylim(
    -1,
    NUM_TAXELS
)

ax_fa.set_yticks(
    range(
        0,
        NUM_TAXELS,
        2
    )
)

ax_fa.grid(
    axis="x",
    alpha=0.15
)


# ============================================================
# 5. RASTER PLOT SA
# ============================================================

for taxel in range(NUM_TAXELS):

    spikes = df_sa.loc[
        df_sa["idx"] == taxel,
        "tempo_rel"
    ].values

    # Mostrar somente spikes dentro da janela
    spikes = spikes[
        (spikes >= TEMPO_INICIAL_PLOT)
        &
        (spikes <= TEMPO_FINAL_PLOT)
    ]

    if len(spikes) > 0:

        ax_sa.eventplot(
            spikes,
            lineoffsets=taxel,
            linelengths=0.8,
            linewidths=0.8,
            colors=COR_SA
        )


ax_sa.set_title(
    "SA"
)

ax_sa.set_ylabel(
    "Taxel"
)

ax_sa.set_xlabel(
    "Tempo (s)"
)

ax_sa.set_ylim(
    -1,
    NUM_TAXELS
)

ax_sa.set_yticks(
    range(
        0,
        NUM_TAXELS,
        2
    )
)

ax_sa.grid(
    axis="x",
    alpha=0.15
)


# ============================================================
# JANELA DO GRÁFICO
#
# 4.5 s até 10.5 s
#
# NÃO DESENHA LINHAS VERTICAIS
# ============================================================

ax_sa.set_xlim(
    TEMPO_INICIAL_PLOT,
    TEMPO_FINAL_PLOT
)


# ============================================================
# MARCAÇÕES DO EIXO X
# ============================================================

ax_sa.set_xticks(
    np.arange(
        TEMPO_INICIAL_PLOT,
        TEMPO_FINAL_PLOT + 0.01,
        0.5
    )
)


# ============================================================
# REMOVER BORDAS SUPERIORES E DIREITAS
# ============================================================

for ax in [
    ax_forca,
    ax_adc,
    ax_derivada,
    ax_fa,
    ax_sa
]:

    ax.spines["top"].set_visible(
        False
    )

    ax.spines["right"].set_visible(
        False
    )


# ============================================================
# AJUSTE FINAL
# ============================================================

plt.tight_layout()

plt.show()
