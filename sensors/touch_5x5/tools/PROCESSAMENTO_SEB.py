import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# CONFIGURAÇÃO
# ============================================================

PASTA = r"c:\Users\Gabi\OneDrive\Documentos\Artigo_SEB\DADOS_CARGA"

NOME_EXPERIMENTO = "23"

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

NUM_TAXELS = 25
TEMPO_TOTAL = 15.0

# ============================================================
# ADC
# ============================================================

ADC_MAX = 4095.0
V_MAX = 3.3

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

df = pd.read_csv(
    ARQUIVO_SPIKES
)

df["tempo"] = pd.to_numeric(
    df["tempo"],
    errors="coerce"
)

df["idx"] = pd.to_numeric(
    df["idx"],
    errors="coerce"
)

df["tipo"] = (
    df["tipo"]
    .astype(str)
    .str.strip()
    .str.upper()
)

df = df.dropna(
    subset=[
        "tempo",
        "idx",
        "tipo"
    ]
).copy()

df["idx"] = df["idx"].astype(int)

# Manter somente FA e SA
df = df[
    df["tipo"].isin(["FA", "SA"])
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

COLUNAS_ADC = [
    f"taxel_{i}"
    for i in range(NUM_TAXELS)
]

for coluna in COLUNAS_ADC:

    df_adc[coluna] = pd.to_numeric(
        df_adc[coluna],
        errors="coerce"
    )

df_adc = df_adc.dropna(
    subset=["tempo"]
).copy()

# ============================================================
# IDENTIFICAR BLOCO PRINCIPAL DOS SPIKES
# ============================================================
#
# Procura o intervalo de 15 segundos com maior
# quantidade de spikes.
#
# Isso evita timestamps isolados ou incorretos.
#
# ============================================================

tempos = np.sort(
    df["tempo"].values
)

if len(tempos) == 0:

    raise ValueError(
        "Nenhum spike FA ou SA encontrado."
    )

melhor_inicio = None
melhor_fim = None
maior_quantidade = 0

j = 0

for i in range(len(tempos)):

    if j < i:
        j = i

    while (
        j < len(tempos)
        and
        tempos[j] - tempos[i] <= TEMPO_TOTAL
    ):

        j += 1

    quantidade = j - i

    if quantidade > maior_quantidade:

        maior_quantidade = quantidade

        melhor_inicio = tempos[i]

        melhor_fim = tempos[j - 1]

# ============================================================
# RECORTAR SPIKES
# ============================================================

df = df[
    (df["tempo"] >= melhor_inicio)
    &
    (
        df["tempo"]
        <= melhor_inicio + TEMPO_TOTAL
    )
].copy()

# ============================================================
# TEMPO RELATIVO DOS SPIKES
# ============================================================

df["tempo_rel"] = (
    df["tempo"]
    - melhor_inicio
)

# ============================================================
# SEPARAR FA E SA
# ============================================================

df_fa = df[
    df["tipo"] == "FA"
].copy()

df_sa = df[
    df["tipo"] == "SA"
].copy()

# ============================================================
# SINCRONIZAR FORÇA
# ============================================================

df_forca_recorte = df_forca[
    (df_forca["tempo"] >= melhor_inicio)
    &
    (
        df_forca["tempo"]
        <= melhor_inicio + TEMPO_TOTAL
    )
].copy()

# ============================================================
# CASO A FORÇA TENHA OUTRA REFERÊNCIA DE TEMPO
# ============================================================

if len(df_forca_recorte) == 0:

    print()
    print(
        "ATENÇÃO: força não encontrada "
        "na mesma referência dos spikes."
    )

    t0_forca = df_forca["tempo"].min()

    df_forca_recorte = df_forca.copy()

    df_forca_recorte["tempo_rel"] = (
        df_forca_recorte["tempo"]
        - t0_forca
    )

    df_forca_recorte = df_forca_recorte[
        (
            df_forca_recorte["tempo_rel"] >= 0
        )
        &
        (
            df_forca_recorte["tempo_rel"]
            <= TEMPO_TOTAL
        )
    ].copy()

else:

    df_forca_recorte["tempo_rel"] = (
        df_forca_recorte["tempo"]
        - melhor_inicio
    )

# ============================================================
# SINCRONIZAR ADC
# ============================================================

df_adc_recorte = df_adc[
    (df_adc["tempo"] >= melhor_inicio)
    &
    (
        df_adc["tempo"]
        <= melhor_inicio + TEMPO_TOTAL
    )
].copy()

# ============================================================
# CASO ADC TENHA OUTRA REFERÊNCIA DE TEMPO
# ============================================================

if len(df_adc_recorte) == 0:

    print()
    print(
        "ATENÇÃO: ADC não encontrado "
        "na mesma referência dos spikes."
    )

    t0_adc = df_adc["tempo"].min()

    df_adc_recorte = df_adc.copy()

    df_adc_recorte["tempo_rel"] = (
        df_adc_recorte["tempo"]
        - t0_adc
    )

    df_adc_recorte = df_adc_recorte[
        (
            df_adc_recorte["tempo_rel"] >= 0
        )
        &
        (
            df_adc_recorte["tempo_rel"]
            <= TEMPO_TOTAL
        )
    ].copy()

else:

    df_adc_recorte["tempo_rel"] = (
        df_adc_recorte["tempo"]
        - melhor_inicio
    )

# ============================================================
# ORDENAR ADC PELO TEMPO
# ============================================================

df_adc_recorte = (
    df_adc_recorte
    .sort_values("tempo_rel")
    .reset_index(drop=True)
)

# ============================================================
# CONVERTER ADC PARA TENSÃO INVERTIDA
# ============================================================
#
# IMPORTANTE:
#
# Queremos que:
#
# SEM PRESSÃO -> tensão baixa
# COM PRESSÃO -> tensão alta
#
#
# ADC original:
#
# ADC = 4095 -> V = 0 V
# ADC = 0    -> V = 3.3 V
#
#
# Equação:
#
# V = 3.3 - ((ADC / 4095) * 3.3)
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
# TEMPO DO ADC
# ============================================================

tempo_adc = df_adc_recorte[
    "tempo_rel"
].to_numpy()

# ============================================================
# CALCULAR DERIVADA DA TENSÃO
# ============================================================
#
# Como a tensão já foi invertida:
#
# início do aperto   -> derivada positiva
# retirada da pressão -> derivada negativa
#
# Unidade: V/s
#
# ============================================================

derivadas = {}

if len(tempo_adc) >= 2:

    for coluna_v in COLUNAS_V:

        tensao = df_adc_recorte[
            coluna_v
        ].to_numpy(
            dtype=float
        )

        derivadas[coluna_v] = np.gradient(
            tensao,
            tempo_adc
        )

else:

    raise ValueError(
        "Poucas amostras de ADC "
        "para calcular a derivada."
    )

# ============================================================
# INFORMAÇÕES
# ============================================================

print()
print("======================================")
print("COLETA IDENTIFICADA")
print("======================================")

print(
    f"Início STM32: {melhor_inicio:.6f} s"
)

print(
    f"Fim STM32: {melhor_fim:.6f} s"
)

print()

print(
    f"FA: {len(df_fa)} spikes"
)

print(
    f"SA: {len(df_sa)} spikes"
)

print(
    f"Força: {len(df_forca_recorte)} amostras"
)

print(
    f"ADC: {len(df_adc_recorte)} amostras"
)

print("======================================")

# ============================================================
# CRIAR FIGURA
#
# 1 - FORÇA
# 2 - TENSÃO INVERTIDA
# 3 - DERIVADA DA TENSÃO
# 4 - RASTER FA
# 5 - RASTER SA
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
#
# Agora o sinal SOBE quando o sensor é pressionado.
#
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

# ============================================================
# EIXO 0 - 3.3 V
# ============================================================

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

# Linha de zero
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
# 4. RASTER FA
# ============================================================

for taxel in range(NUM_TAXELS):

    spikes = df_fa.loc[
        df_fa["idx"] == taxel,
        "tempo_rel"
    ].values

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
# 5. RASTER SA
# ============================================================

for taxel in range(NUM_TAXELS):

    spikes = df_sa.loc[
        df_sa["idx"] == taxel,
        "tempo_rel"
    ].values

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
# EIXO DO TEMPO
# ============================================================

ax_sa.set_xlim(
    0,
    TEMPO_TOTAL
)

ax_sa.set_xticks(
    np.arange(
        0,
        TEMPO_TOTAL + 1,
        1
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

    ax.spines[
        "top"
    ].set_visible(False)

    ax.spines[
        "right"
    ].set_visible(False)

# ============================================================
# AJUSTE FINAL
# ============================================================

plt.tight_layout()

plt.show()