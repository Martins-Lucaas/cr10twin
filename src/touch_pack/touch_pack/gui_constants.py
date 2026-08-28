"""
gui_constants.py — constantes compartilhadas entre `palpation_gui.py` e as
fatias dele (`gui_*.py`).

Existe por causa do recorte em mixins: um punhado de constantes vivia no
corpo de `palpation_gui.py` e passou a ser usado dos dois lados. Deixá-las lá
obrigaria os mixins a importar do host, o que fecharia um ciclo de import.

Não confundir com `constants.py`: aquele guarda o que atravessa NÓS (tópicos,
limites de força, caminhos de arquivo, geometria da matriz). Aqui ficam
valores de INTERFACE — faixas de slider, opções de combo, horizonte de
trajetória do arrasto — que só a GUI conhece.
"""
from __future__ import annotations

from .constants import (
    CONTACT_ON_N, FORCE_SETPOINT_MAX_N, MATRIX_SPAN_MAX_MM,
)

# ── Setpoint de força ─────────────────────────────────────────────────
# O PISO É O LIMIAR DE CONTATO, e derivá-lo em vez de cravá-lo é o conserto
# de um defeito que custou um ensaio inteiro.
#
# Ele era 0,1 N escrito à mão aqui, e o explorer recusa qualquer setpoint (e
# qualquer mínimo de onda) abaixo de CONTACT_ON_N. Enquanto os dois números
# valeram 0,10 por coincidência ninguém percebeu; em 28/08/2026 CONTACT_ON_N
# subiu para 0,12 e a coincidência acabou. O resultado é que o slider passou
# a OFERECER 0,1 N como valor mínimo escolhível enquanto o explorer o
# rejeitava: o run TOUCH/20260828_134305 pediu uma senoide de 0,10–2,00 N,
# teve o perfil recusado por _force_profile(), e rodou como um HOLD comum —
# 20 ciclos de onda que simplesmente não aconteceram, sem nada na tela.
#
# Abaixo do limiar de contato não há setpoint possível: é uma força que o
# sistema não distingue de ar livre. Um slider que a oferece está oferecendo
# uma escolha inválida.
FORCE_SP_MIN, FORCE_SP_MAX, FORCE_SP_DEFAULT = (
    CONTACT_ON_N, FORCE_SETPOINT_MAX_N, 2.0)

# ── Configurador visual da grade (MATRIX_MAP) ─────────────────────────
# Passo entre pontos. O piso é a repetibilidade de posicionamento cartesiano
# do CR10; o teto vem do envelope da matriz.
MATRIX_STEP_MIN, MATRIX_STEP_MAX, MATRIX_STEP_DEFAULT = 0.5, 50.0, 5.0  # mm
# Linhas/colunas. O produto ainda é validado contra MATRIX_MAX_POINTS.
MATRIX_N_MIN, MATRIX_N_MAX, MATRIX_N_DEFAULT = 1, 20, 5
# Formatos do plano oferecidos pelo configurador visual.
MATRIX_SHAPES = ('SQUARE', 'RECT')
# Como a grade é dimensionada:
#   STEP — passo fixo, o tamanho do alvo é consequência (comportamento
#          histórico; preserva configs salvas antes desta opção existir).
#   SIZE — dimensões do alvo fixas, o passo é DERIVADO: step = span/(n−1).
#          A grade cobre a peça de borda a borda para qualquer contagem.
MATRIX_SIZING_MODES = ('STEP', 'SIZE')
# Dimensões do alvo (mm). O teto é o mesmo envelope da matriz — não faz
# sentido declarar uma peça maior do que o trânsito cego alcança.
MATRIX_SIZE_MIN     = MATRIX_STEP_MIN
MATRIX_SIZE_MAX     = MATRIX_SPAN_MAX_MM
MATRIX_SIZE_DEFAULT = 20.0
# Ordem de visita dos pontos:
#   CORNERS    — toca os 4 EXTREMOS da grade primeiro ((0,0), (W,0), (W,H),
#                (0,H)) e só então varre o resto em serpentina. É uma
#                verificação de registro: os cantos são os pontos onde a
#                grade sai do alvo primeiro se as dimensões ou a origem
#                estiverem erradas, e vê-los cedo custa 4 identações em vez
#                de uma matriz inteira medida fora da peça.
#   SERPENTINE — varredura linha a linha (comportamento histórico).
MATRIX_PATH_ORDERS = ('CORNERS', 'SERPENTINE')

# ── Braço ─────────────────────────────────────────────────────────────
# Faixas de slider do braço, em graus (ARM_JOINTS vem de constants.py).
ARM_LIMITS_DEG = {
    'joint1': (-180, 180), 'joint2': (-180, 180), 'joint3': (-160, 160),
    'joint4': (-180, 180), 'joint5': (-180, 180), 'joint6': (-180, 180),
}

# ── Manipulação 3D ────────────────────────────────────────────────────
# Horizonte das trajetórias publicadas pelo arrasto 3D. O tick da viewport é
# de 30 ms; pedir ~100 ms ao JTC dá margem para o controlador interpolar sem
# engasgar e mantém o braço colado no cursor.
MANIP_TRAJ_DURATION_S = 0.10
