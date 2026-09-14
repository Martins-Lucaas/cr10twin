"""explorer_constants.py — os números do protocolo de palpação, e o porquê deles.

Cada constante aqui carrega uma decisão de bancada: quantos newtons contam
como contato, quanto tempo uma leitura ainda vale, a que velocidade a ponteira
pode encostar sem dar pico de impacto. O comentário ao lado de cada uma NÃO é
decoração — é o registro do ensaio que fixou o número, e é o que impede a
próxima pessoa (ou o próximo refactor) de "arredondar" um valor que foi medido.

Elas moram num módulo próprio, e não no `tactile_explorer`, porque `force_wave`
e `stiffness` — que saíram de lá — precisam das mesmas. Deixá-las no host faria
esses dois importarem de quem os importa: um ciclo que só estoura em runtime,
no import. É a mesma razão de `gui_constants.py` existir ao lado da GUI.

O host reexporta tudo isto, então `tactile_explorer.X` continua resolvendo para
quem já importava de lá.
"""
from __future__ import annotations

import math

import numpy as np

from .constants import (
    ARM_JOINTS as _ARM_JOINTS,
    CONTACT_ON_N as _CONTACT_ON_N,
    FORCE_CTRL_SIGMA_N as _FORCE_CTRL_SIGMA_N,
    POINTING_SEED_DEG as _POINTING_SEED_DEG,
)



_POINTING_SEED_Q = np.array(
    [math.radians(_POINTING_SEED_DEG[j]) for j in _ARM_JOINTS])

# Convenção de sinal: compressão = POSITIVO, tração = NEGATIVO (/load_cell/force_net).
_SLIDING_SAFETY_M   = 0.30   # m: distância máxima de segurança no SLIDING

# Nunca exceder _FORCE_ABORT_LIMIT_N (15 N). Como a leitura tem atraso e o
# braço tem inércia, a parada dispara com MARGEM (_FORCE_SAFE_LIMIT_N) para
# o overshoot residual não ultrapassar os 15 N.
_FORCE_SAFE_LIMIT_N = 12.0   # N: margem de 3 N abaixo do teto de 15 N

# Estimador de rigidez do contato p/ a regulação quase-estática:
# Δx = relax·(alvo−fz)/K, com K_est=ΔF/Δx estimado online em REPOUSO
# (_StiffnessEstimator), sem o erro de fase da estimativa contínua.
_K_DEFAULT_NM      = 40_000.0     # N/m: rigidez default antes de estimar
# Piso do estimador. Era 8.000 N/m (8 N/mm), acima da rigidez de uma ponteira
# de SILICONE: medida em bancada em 14/08/2026 a partir do samples.csv da
# run TOUCH/20260814_102401 (regressão força × penetração do TCP), ela dá
# 0,47 N/mm no contato inteiro e 0,62 N/mm perto do setpoint — 45× mais mole
# que a ponteira rígida (28 N/mm). Com o piso antigo, TODO k_inst do silicone
# caía fora da faixa e era descartado: o estimador ficava travado no default
# de 40 N/mm, 65× rígido demais, e o regulador dava passos 65× curtos.
# Os 300 N/m que substituíram os 8.000 cobriam o silicone MEDIDO PERTO DO
# SETPOINT (0,62 N/mm), mas não o PÉ da curva, que é o trecho onde o estimador
# precisa latch: regressão sobre as 2133 amostras entre 0,06 e 0,23 N do run
# TOUCH/20260817_104248 dá 0,279 N/mm — logo ABAIXO do piso. Todo k_inst
# acumulado no pé caía fora da faixa e era descartado, `estimated` nunca
# virava True, e o teto de sonda de 8 µm de então governou a descida
# inteira: 0,60 mm de penetração em 25,9 s para sair de 0,06 N e chegar só a
# 0,23 N. No instante em que a curva enrijeceu para 1,0 N/mm o par passou no
# piso, o passo saltou para _QS_DX_MAX_M e o mesmo braço fez 1,33 mm em 2,2 s.
# O _ContactCurve já registrava 0,18 N/mm nessa mesma faixa: o piso precisa
# ficar abaixo do PÉ, não da secante perto do alvo.
#
# 50 N/m não afrouxa passo nenhum: abaixo de ~3.000 N/m quem morde é o teto
# ABSOLUTO _QS_DX_MAX_M (100 µm), não hard_cap = ΔF/K. O único efeito de
# baixar o piso é deixar o estimador ACEITAR a rigidez que ele mede. Pior caso
# inalterado — K subestimada com contato na verdade rígido: 100 µm × 28 N/mm
# = 2,8 N num passo, o mesmo que _QS_DX_MAX_M já orça.
_K_MIN_NM          =     50.0     # N/m: piso do estimador (pé da curva mole)
_K_MAX_NM          = 1_000_000.0  # N/m: teto do estimador (sensor bem fixo mede ~900 N/mm)
_K_EMA_ALPHA       = 0.25         # filtro EMA do estimador de rigidez
# ΔF mínimo para um par (Δx, ΔF) virar uma medida de rigidez. Abaixo disso o
# sinal é ruído da célula (≈ 0,037 N de pico medidos em bancada), não
# elasticidade. NÃO é mais motivo para DESCARTAR o par: ver update_pair, que
# acumula até cruzar este limiar.
_K_PAIR_MIN_DF_N   =      0.04    # N
# Δx mínimo para o par entrar no acumulador. NÃO é um limiar de ruído — quem
# cuida do ruído é _K_PAIR_MIN_DF_N acima, somando passos até o ΔF sair dele.
# Era 1,5 µm, e isso cegava o estimador justamente na aproximação final: com
# a rampa de não-ultrapassagem os passos perto do alvo são de 1 a 5 µm, e
# todo par abaixo de 1,5 µm era DESCARTADO — em contato rígido o estimador
# nunca latchava e k_push ficava no default de 40 N/mm, 20× mole demais.
# 0,2 µm fica abaixo do menor passo que a rampa comanda e acima do zero.
_K_PAIR_MIN_DX_M   =      2.0e-7  # m
# Curva F(x) MEDIDA na descida (ver _ContactCurve). Requisitos mínimos para
# ela substituir o escalar K no feedforward da onda: sem pontos suficientes
# ou sem excursão de força suficiente, a interpolação seria pior que o
# escalar e o caminho antigo continua valendo.
_FX_MIN_POINTS  = 6      # pares (x, F) distintos
_FX_MIN_SPAN_N  = 0.30   # N: excursão de força coberta pela curva
_FX_MIN_SEG_DF_N = 0.02  # N: segmento menor que isto não mede inclinação
# Teto do ganho corretivo aplicado à curva pela adaptação por ciclo. A curva
# vem da descida QUASE-ESTÁTICA e o ensaio é dinâmico: num viscoelástico as
# duas diferem, mas por um fator, não por ordens de grandeza — se a correção
# pedir mais que isto o problema é outro (contato perdido, tare errado).
_FX_GAIN_MIN, _FX_GAIN_MAX = 0.2, 3.0
_DEADBEAT_DX_MAX_M = 2.5e-4       # 0.25 mm: passo cheio do alívio de emergência (_relieve_contact)
# Afundamento máximo do TCP abaixo da reta do deslize durante o SLIDING —
# guarda GEOMÉTRICA contra mergulhar numa borda ou num degrau da amostra.
# Não há correção de força atuando aqui: a trava de Z é puramente posicional.
_SLIDE_MAX_SINK_M    = 0.010    # 10 mm
# ── SLIDING: perda de contato ─────────────────────────────────────────
# O SLIDING não regula força, então a única reserva contra a superfície
# escapar é a indentação que o HOLD deixou: alvo/K, com K vindo do
# _StiffnessEstimator daquele contato. Contra superfície rígida (K da ordem
# de centenas de N/mm) essa reserva é de POUCOS MICRONS — quem sustenta o
# contato ao longo do curso é o paralelismo da montagem, não o software.
_SLIDE_CONTACT_MIN_FRAC = 0.25
# Curso lateral CONTÍNUO tolerado sem contato antes de abortar.
_SLIDE_LOST_BUDGET_M    = 0.005
# A guarda INTERROMPE o deslize ou apenas AVISA? Fica em AVISO de propósito:
# uma queda de força PODE SER A PRÓPRIA TEXTURA (um sulco, uma depressão, um
# degrau na peça), e abortar ali descartaria medição legítima. A perda segue
# detectada e reportada uma vez por fase, com a conta de inclinação, mas o
# curso vai até o fim e o run termina 'ok'.
# O que separa os dois casos é a ESCALA, e é isso que _SLIDE_LOST_BUDGET_M
# mede: queda curta = feição da superfície; trecho longo e contínuo = a
# amostra escapou do plano, e o lugar de corrigir é o calço.
# NÃO trocar para True sem esse discernimento — ver o histórico de runs em
# que o curso inteiro saiu sem contato por montagem, não por textura.
_SLIDE_LOST_ABORTS      = False
# Teto da inclinação declarável. Acima disso a amostra está mal montada e o
# lugar de corrigir é o calço, não o software: a 10° um curso de 50 mm já
# pede 8,8 mm de Z, quase o orçamento inteiro de afundamento.
_SLIDE_SLOPE_MAX_DEG    = 10.0

# ── Plano do deslize ──────────────────────────────────────────────────
# O SLIDING percorre uma RETA CONTIDA NO PLANO DA AMOSTRA, em posição.
# A força fica livre, e a variação dela ao longo do curso é o sinal de
# textura que se quer medir — é o comportamento PRETENDIDO, não uma falta
# de controle.
#
# A reta é definida por uma base ortonormal montada em _slide_frame:
#   u — avanço, a direção pedida PROJETADA no plano (unitária). O curso
#       pedido passa a ser distância ao longo da SUPERFÍCIE, não a projeção
#       horizontal dela.
#   w — transversal, também no plano (w = n × u). Trava o desvio lateral.
#   n — normal do plano. Trava a profundidade do contato.
# Travar u/w/n é o que mantém o percurso numa reta: sem a trava em n o TCP
# mergulha ou escapa, sem a trava em w ele arqueia para fora da direção
# pedida.
#
# De onde vem o plano, nesta ordem:
#   1. a normal MEDIDA pela calibração do ângulo de ataque, quando ela
#      rodou (_slide_plane_n) — o caso em que a geometria é conhecida;
#   2. a inclinação DECLARADA na GUI ("Slide Slope" → _slide_slope_deg),
#      que descreve o plano por um único ângulo ao longo do curso;
#   3. o default 0°, que devolve o plano horizontal do mundo URDF — a
#      referência geométrica fixa e reprodutível de quando não se sabe nada.
#
# Sem (1), o plano é uma DECLARAÇÃO e continua valendo o requisito de
# montagem: a amostra precisa estar paralela à reta dentro da indentação
# disponível (alvo/K), poucos µm contra superfície rígida. Perda de contato
# no meio do curso, nesse caso, é sintoma de montagem e o lugar de corrigir
# é o calço. Com (1) a reta segue o plano real e esse requisito relaxa.
# Eixo Z do mundo URDF — referência do plano horizontal e da inclinação
# declarada.
_Z_HAT = np.array([0.0, 0.0, 1.0])
# Força mínima que caracteriza contato. Re-medido em 13/08 sobre os 2966
# quadros pré-contato do run 20260813_153811: média +0,007 N, σ = 0,0086 N,
# máximo +0,037 N em ar livre — o antigo 0,11 N estava a 12σ, custando um
# período de amostragem inteiro DENTRO do contato antes do halt. 0,06 N
# fica a ~6σ e ainda 1,6× acima do pior ruído observado. Um falso gatilho
# custa tempo, não força: o _contact_confirm exige a MEDIANA acima do
# limiar com o braço parado, e _CONTACT_FALSE_MAX aborta com diagnóstico.
#
# 17/08: de volta a 0,10 N. O σ de 0,0086 N que justificou os 0,06 N não se
# sustentou — medido sobre os 2390 quadros em repouso do run
# MANUAL/20260817_142719 (janelas 0–3 s e 6–32 s), σ = 0,023 N, 2,7x maior.
# Contra ESSE ruído os 0,06 N ficam a 2,6σ, dentro da cauda: é gatilho falso
# recorrente, e são só 8 antes de _CONTACT_FALSE_MAX abortar a descida.
# 0,10 N volta a 4,3σ. E o número que manda de verdade não é nem o ruído
# elétrico: durante o traverse rápido (3,70–4,95 s, 6 mm/s) a carga INERCIAL
# do conjunto abaixo da célula sozinha varre 0,197 N pico a pico — 3x o
# limiar antigo. Quem segura esse caso é o _contact_confirm, que exige a
# mediana acima do limiar com o braço PARADO; o limiar mais alto só evita
# gastar os halts de confirmação à toa.
#
# O valor mora em `constants.CONTACT_ON_N` porque a GUI precisa do MESMO
# número: o indicador "in contact" tinha um 0.2 cravado que nunca acompanhou
# os retunes acima.

# ── ALVO RETIRADO (colapso de rigidez) ────────────────────────────────
# O ServoJ é seguimento de POSIÇÃO puro: ele não conhece rigidez nenhuma (ver
# o guia V4.5.1 — "not affected by the global rate, but constrained by the
# speed limit"). Quem traduz força em posição é o feedforward Δx = ΔF/K deste
# nó, uma camada acima. Se a amostra sair de baixo da ponteira, K perde
# sentido: a força cai a zero, o erro vira o setpoint inteiro e o regulador
# manda avançar — e o ServoJ obedece, sem nada que o segure.
#
# No HOLD do MANUAL isso não tinha fim: ele roda com stable_s=inf e
# timeout_s=inf, então o braço desceria indefinidamente a passos livres
# procurando um contato que não existe mais.
#
# A assinatura de "alvo retirado" é rigidez nula: avanço COMANDADO sem
# NENHUMA subida de força. É diferente de um contato mole (avança e a força
# sobe pouco) e de perder o contato de raspão (recupera em poucos µm). O que
# separa os três é o CURSO livre acumulado depois de já ter havido contato.
_TARGET_LOST_FREE_M = 0.0015   # 1,5 mm de avanço livre após contato firmado
# Queda ABRUPTA: da ordem do setpoint para abaixo do limiar de contato dentro
# desta janela. Relaxação viscoelástica é lenta e não cai abaixo do contato;
# retirar a amostra é instantâneo.
_TARGET_LOST_DROP_S    = 0.7
_TARGET_LOST_DROP_FRAC = 0.5   # fração do setpoint que caracteriza "estava carregado"

# O gatilho dispara na PRIMEIRA leitura acima de _CONTACT_ON_N e para o
# braço na hora; a confirmação (N leituras) vem DEPOIS, já parado — confirmar
# antes custaria N-1 períodos de amostragem DENTRO do contato (força
# crescendo sem controle). Um falso positivo custa tempo, não força.
_CONTACT_ON_SAMPLES = 3         # leituras distintas p/ confirmar, com o braço parado
_CONTACT_CONFIRM_S  = 1.0       # s: teto de espera pelas leituras acima
_CONTACT_FALSE_MAX  = 8         # falsos gatilhos tolerados antes de abortar a descida
# Teto de velocidade de aproximação até o contato — limita o transiente de
# impacto (≈ v·latência·rigidez) para não ultrapassar a margem de força.
_DESCEND_CONTACT_V_MAX_MS = 0.0005   # 0,5 mm/s
# Descida em DOIS ESTÁGIOS: a profundidade do 1º contato é memorizada POR
# HOME (_learned_by_home); descidas seguintes daquela home vão à velocidade
# cheia até a margem antes do ponto aprendido, rastejando só no trecho
# final.
_CONTACT_ZONE_MARGIN_M = 0.0015   # 1,5 mm: piso da zona lenta antes do contato aprendido
# A zona lenta precisa ser longa o bastante para o braço LARGAR a velocidade
# do estágio rápido antes do contato aprendido, senão o toque aconteceria
# ainda em velocidade cheia.
_ZONE_REACTION_S = 0.3            # s: rampa de desaceleração + drenagem da fila
# ── Margem de INCERTEZA do contato (adaptativa) ───────────────────────
# A zona lenta soma DUAS parcelas que não têm nada a ver uma com a outra:
#
#   frenagem  = v_rápida × _ZONE_REACTION_S — física do braço, encolhe só se
#               a velocidade do estágio rápido encolher;
#   incerteza = o quanto o contato REAL pode estar longe do estimado — é o
#               trecho que o rastejo de fato precisa cobrir.
#
# Elas eram combinadas com max(), e isso não é conservador: é o contrário.
# Com approach de 20 mm/s a frenagem sozinha vale 6 mm, o max() devolve 6 mm,
# e a margem de incerteza vira ZERO — o braço termina a rampa exatamente em
# cima do contato estimado e toca ainda em movimento. Somar é o que garante
# que sempre exista rastejo de verdade antes do toque.
#
# Somar tornaria a zona maior a custo constante, e é aí que entra a
# adaptação: a incerteza deixa de ser um palpite fixo de 1,5 mm e passa a ser
# MEDIDA. Cada contato confirmado de uma mesma geometria (a grade do
# MATRIX_MAP parte sempre do mesmo Safe Z) entra numa janela; se os pontos
# concordam entre si, a peça é plana ali e não há o que rastejar.
# Auto-limitante: peça irregular ⇒ dispersão alta ⇒ a zona não encolhe.
_CONTACT_MARGIN_MIN_PTS = 3       # contatos necessários para confiar na dispersão
_CONTACT_MARGIN_K       = 4.0     # dispersões observadas cobertas pela margem
_CONTACT_MARGIN_FLOOR_M = 4.0e-4  # 0,4 mm: piso — abaixo disso o rastejo não
                                  # cobre nem a quantização de 10 µm da FK
                                  # somada ao ruído do próprio gatilho
_CONTACT_MARGIN_WINDOW  = 8       # contatos recentes considerados
# Reaproveitamento do contato aprendido de uma home VIZINHA. O offset entre os
# TCPs de duas homes se decompõe em duas parcelas com consequências diferentes:
#   • ao longo do approach — desloca o contato 1:1, e é CORRIGIDO exatamente
#     em _lookup_learned (não custa margem nenhuma);
#   • perpendicular — só importa se a peça não for plana, e é isso que
#     _LEARNED_TCP_TOL_M limita, com _LEARNED_FLATNESS_M de desconto.
_LEARNED_TCP_TOL_M  = 0.005       # 5 mm: offset LATERAL máximo aceito
_LEARNED_FLATNESS_M = 0.001       # 1 mm: desconto por não-planicidade da peça
_DESCEND_TOUCH_V_MS    = 0.0002   # 0,2 mm/s: TETO do rastejo final. Deixou de
                                  # ser o valor usado — ver _crawl_v_ms.
# Pico do toque em streaming = v · T_halt · K (curso comprometido × rigidez).
# O ORÇAMENTO desse pico é o limiar de contato — o primeiro impacto detecta,
# não mede (ver crawl_v_ms).
# T_halt é a latência da cadeia explorer→JTC→sim→mirror→ServoJ→braço.
#
# 28/08/2026: desacoplado de _ZONE_REACTION_S e fixado em 85 ms. Os dois
# valores mediam coisas diferentes e estavam colados por conveniência:
# _ZONE_REACTION_S dimensiona a RAMPA DE FRENAGEM (física do braço, continua
# 0,3 s) e T_halt é o TRANSPORTE do comando de halt. Herdar 0,3 s aqui punha
# a descida a 12 µm/s — uma palpação de 6,5 mm em 532 s, medida em
# sensors/Data/MANUAL/20260828_120028.
#
# ATENÇÃO — os 85 ms são a latência de transporte medida no executor da onda,
# NÃO nesta cadeia. É a melhor referência disponível, mas continua sendo uma
# transposição, não uma medição direta: `latency_probe.py` é o instrumento que
# fecha isso. O erro é conservador nos dois sentidos? NÃO: se a cadeia real
# for mais lenta que 85 ms, o primeiro toque bate com força PROPORCIONALMENTE
# maior que o orçamento (impact = v · T_halt · K). Medir antes de subir mais.
#
# A 12 µm/s a medição era impossível: o curso de frenagem ficava abaixo do
# quantum de 10 µm da FK. A 50 µm/s ele passa a ser observável.
_STREAM_HALT_LAT_S = 0.085
# Zona de desaceleração antes do contato aprendido.
_DESCEND_DECEL_ZONE_M = 0.003     # 3 mm
# Piso do rastejo: abaixo disto um tick de 30 ms move menos que 0,3 µm e a
# descida deixa de ser observável no feedback (quantum da FK = 10 µm).
_DESCEND_CRAWL_V_MIN_MS = 1.0e-5  # 10 µm/s

# Regulação QUASE-ESTÁTICA de força (move-then-measure): a malha contínua
# mede em MOVIMENTO com atraso do filtro, e contra contato rígido isso vira
# quique (passo comandado com leitura defasada alivia/aprofunda demais).
_QS_SETTLE_TICKS   = 5       # ticks congelado antes de medir (150 ms > lag)
_QS_MEDIAN_N       = 3       # amostras DISTINTAS da mediana settled
# Teto de ticks do _qs_measure_fz quando as _QS_MEDIAN_N leituras distintas
# não couberam nos _QS_SETTLE_TICKS.
#
# POR QUE ELE EXISTE. A mediana é de amostras DA CÉLULA, e o laço de medida
# conta em ticks de CONTROLE (33 Hz). Com a célula entregando 24 Hz, os 5
# ticks (150 ms) viam ~3,6 amostras novas: as outras 1,4 leituras de `reads`
# eram o MESMO valor lido de novo, porque _fz_corrected() devolve o último
# force_net recebido sem olhar se ele mudou. Nas últimas 3 posições havia
# tipicamente uma repetição, e a "mediana de 3" decidia com 2 amostras — no
# ponto de medida de toda a regulação quase-estática. O _contact_confirm já
# comparava `seq` para não cair nisso; aqui não se comparava.
#
# A 82 Hz o problema não aparece (150 ms = ~12 amostras), e é por isso que o
# teto quase nunca é alcançado. Ele é a rede para a taxa cair — placa com
# firmware antigo, pino RATE em GND, link engasgado.
_QS_MEASURE_MAX_TICKS = _QS_SETTLE_TICKS + 5
# ── MEDIR FORÇA QUE AINDA ESTÁ SE MOVENDO (a causa do overshoot) ──────
# O move-then-measure supõe que a força ASSENTA dentro do settle. Contra um
# contato viscoelástico ela não assenta: continua evoluindo por segundos.
#
# Medido no run TOUCH/20260828_134305 (alvo 1,0 N, banda 0,092 N): entre
# t=120,03 e t=120,76 o braço estava RECUANDO (z 50,77 → 50,79 mm) e a força
# SUBIU de 1,233 para 1,326 N. Pico de 1,326 N — 0,326 N acima do alvo, 3,5×
# a meia-banda.
#
# Por que isso vira overshoot, e por que o estimador de rigidez leva a culpa
# sem ser o culpado: o regulador lê a força 150 ms depois do passo, quando só
# parte do ΔF chegou. A secante que ele deduz (ΔF_parcial/Δx) SUBESTIMA a
# rigidez real; `k_push` sai baixo; e como todo teto de passo é ΔF/k_push, o
# passo seguinte sai grande na mesma proporção. O erro se realimenta: quanto
# mais o material dá creep, mais o regulador acredita que ele é mole, e mais
# fundo ele empurra.
#
# A correção é medir depois de a força PARAR, não depois de um relógio. E é
# cara — por isso só vale PERTO DO ALVO, onde ultrapassar custa força na
# amostra. Longe dele o passo é grande, o creep é irrelevante ao lado dele, e
# esperar só faria a descida demorar.
_QS_SETTLE_NEAR_MULT = 3.0   # dentro de N bandas do alvo, mede assentado
_QS_SETTLE_MAX_TICKS = 33    # teto da espera (~1 s a 33 Hz)
# Deriva que conta como "assentado": a mediana da 2ª metade da janela contra a
# da 1ª. Deriva e não pico-a-pico, porque o ptp cresce com o tamanho da janela
# mesmo num sinal estacionário — é o mesmo critério que o tare do
# force_receiver usa (_window_drift), e pelo mesmo motivo.
# 2σ do sinal que a MALHA vê, não do sensor cru — mesma correção do
# HOLD_TOL_N. Com o σ cru isto valia 0,046 N e ficou MAIOR que a banda de
# 0,02 N pedida em 07/09/2026: o laço declararia "assentado" ainda fora da
# banda, que é a contradição que o test_no_overshoot trava.
_QS_SETTLE_DRIFT_N = 2.0 * _FORCE_CTRL_SIGMA_N
_QS_RELAX          = 0.7     # sub-relaxação do passo (robustez a erro de K_est)
_QS_DF_MAX_N       = 0.2     # N: ΔF projetado máximo por micro-passo (contato rígido, sem silicone)
# Teto ABSOLUTO do micro-passo. Não é o limitador principal — quem limita por
# física é hard_cap = _QS_DF_HARD_N/K, que escala com a rigidez medida. Este
# aqui existe só para bound o estrago quando K_est está ERRADA (subestimada).
#
# Os 10 µm antigos eram, na prática, hard_cap para a ponteira rígida
# (0,3 N / 28 N/mm = 10,7 µm): redundante lá, e devastador no silicone, onde
# hard_cap vale 484 µm e este teto o cortava em 48×, deixando 0,006 N por
# passo — 161 passos para 1 N, a 300 ms cada.
#
# 100 µm: no silicone (0,62 N/mm) dá 0,062 N por passo, 16× mais rápido; na
# ponteira rígida nada muda, porque hard_cap (10,7 µm) continua mordendo
# primeiro. Pior caso residual — K subestimada e contato na verdade rígido
# (o silicone tocando o fundo): 100 µm × 28 N/mm = 2,8 N num passo, bem
# abaixo do teto de aborto de 15 N, e a checagem de força pega no tick
# seguinte.
_QS_DX_MAX_M       = 1.0e-4  # 100 µm: teto absoluto do micro-passo
_QS_FREE_STEP_M    = 5.0e-6  # 5 µm/ciclo: re-aproximação se perder contato
# Amarrado a _CONTACT_ON_N por INVARIANTE, não por coincidência: o alívio não
# pode projetar abaixo da força que o sistema ainda chama de contato, senão
# recua até largar a peça e o passo livre seguinte volta batendo (QUIQUE). Era
# 0,10 cravado à mão enquanto _CONTACT_ON_N também valia 0,10; quando o limiar
# subiu para 0,12 em 28/08/2026 os dois se descolaram em silêncio. Referenciar
# a constante fecha isso por construção.
_QS_RELIEF_FLOOR_N = _CONTACT_ON_N   # N: alívio nunca projeta abaixo disso
_QS_DF_DEAD_N      = 0.05    # N: ΔF mínimo p/ considerar que o passo "pegou" (abaixo, boost 1,5×)
_QS_BOOST_MAX      = 6.0     # teto do multiplicador anti-stiction
_QS_DF_HARD_N      = 0.3     # N: teto DURO de ΔF por passo (boost incluso); acima, estaciona e dá timeout
# O teto acima é ABSOLUTO, e era esse o problema em setpoint baixo: contra o
# alvo de 0,5 N do run MANUAL/20260817_142719 os 0,2 N de teto de ΔF
# autorizam UM passo a varrer 40 % da faixa inteira. Nenhuma
# estimativa de K sobrevive a isso: mesmo com K exata a descida chega em 3
# passos, e cada erro de K vira overshoot direto, porque não há passo pequeno
# o bastante para o laço se corrigir antes de cruzar o alvo.
#
# O teto passa a ser tambem uma FRAÇÃO do alvo, e vale o menor dos dois. Um
# quarto do alvo dá ~4 passos de aproximação no pior caso, que é o que torna a
# convergência monotônica (linear) em vez de oscilatória.
#
# Piso em _QS_DF_DEAD_N: abaixo disso o ΔF do passo não sai do ruído da célula
# (σ≈0,023 N em repouso neste mesmo run) e o laço não conseguiria nem medir se
# o passo pegou — é justamente o limiar que o boost anti-stiction usa.
#
# Só morde em alvo BAIXO: com df_hard de 0,2 N nada muda acima de 0,8 N de
# setpoint, e com o default de 0,3 N nada muda acima de 1,2 N.
_QS_DF_TARGET_FRAC = 0.25    # fração do alvo que UM micro-passo pode varrer
_QS_DX_PROBE_M     = 3.0e-6  # 3 µm: teto antes do 1º K_est settled (900 N/mm projeta ≤2,7 N)
_QS_DX_PROBE_MAX_M = 8.0e-6  # 8 µm: teto do passo-sonda mesmo com boost (K ainda desconhecida)
_QS_FREE_STEP_MAX_M = 8.0e-6 # 8 µm: teto da re-aproximação sem contato mesmo com boost

# ── NÃO-ULTRAPASSAGEM (por que o overshoot existia) ───────────────────
# Δx = relax·(alvo−fz)/K só não passa do alvo se K estiver CERTA. O contato
# desta bancada ENRIJECE com a penetração: a curva F(x) do run
# TOUCH/20260817_112556 dá secante de 0,18 N/mm entre 0 e 0,2 mm e 3,0 N/mm
# entre 1,8 e 2,0 mm — 17× no mesmo toque. K_est é uma EMA (α=0,25) das
# secantes JÁ PERCORRIDAS, logo mede o trecho MOLE e fica sistematicamente
# ABAIXO da inclinação que vem pela frente. O passo comandado entrega então
# várias vezes o ΔF pedido, e o último passo antes do alvo o atravessa
# inteiro: nos 5 ciclos daquele run, 0,23 a 0,59 N acima de um alvo de 1,6 N
# com banda de 0,05 N. Os tetos não seguravam porque TAMBÉM são calculados
# com a K errada (hard_cap = ΔF_max/K_est), e quem acabava mordendo era o
# teto ABSOLUTO (_QS_DX_MAX_M), que não conhece rigidez nenhuma:
# 200 µm × 2,5 N/mm = 0,5 N de quantum por passo.
#
# Três guardas, todas na direção de EMPURRAR (aliviar de menos custa um tick,
# aliviar demais perde o contato e vira quique):
#   1. k_push — cota SUPERIOR da inclinação local (_StiffnessEstimator.
#      k_upper), usada em TODO teto de passo. Curva convexa: a inclinação
#      logo à frente é maior que a já medida, então a margem cobre o trecho
#      de dentro do próximo passo.
#   2. não-ultrapassagem — o passo nunca projeta a força além do ALVO,
#      mesmo com k_push: Δx ≤ frac·(alvo−fz)/k_push. A mira era `alvo+tol`
#      até 27/08/2026, e com ela parar uma tolerância acima do setpoint era
#      o comportamento CORRETO da lei — overshoot por especificação. Agora a
#      folga tende a zero junto com o erro: a aproximação é geométrica por
#      baixo e o alvo é o teto, não a borda.
#   3. rampa — o passo de empurrar só cresce em progressão geométrica.
#      Sem ela a descida saltava do rastejo de 8 µm do pé da curva
#      direto para os 200 µm do teto absoluto num único tick, que é
#      exatamente onde o overshoot nascia.
_QS_K_PUSH_MARGIN  = 2.0     # fator de segurança da cota superior de rigidez
_QS_NO_CROSS_FRAC  = 0.9     # fração da folga até o ALVO que um passo pode gastar
_QS_STEP_GROWTH    = 3.0     # crescimento máximo do passo de empurrar por tick
_QS_DX_FLOOR_M     = 1.0e-6  # 1 µm: primeiro passo de empurrar (semente da rampa)
_QS_FREE_RESET_TICKS = 3     # leituras seguidas fora do contato p/ reiniciar a rampa
_QS_ARRIVE_S       = 0.35    # s: janela settled em banda p/ o DESCENDING declarar chegada
# Teto da convergência INICIAL (etapa A da rampa). NÃO é gate de desempenho —
# é detector de "não chega" (contato quebrado, sentido errado, curso curto).
# A rampa a v constante é mais lenta que a lei proporcional perto do alvo
# (medida assentada ~1 s/tick dentro de 3 bandas), então tem de ser generoso:
# 3 N em silício mole levam ~20-30 s de etapa A. Depois do 1º cruzamento não
# há mais timeout — a etapa B roda por `stable_s` de relógio.
_QS_TIMEOUT_S      = 45.0    # s: teto da convergência inicial no DESCENDING

# ── RAMPA A VELOCIDADE CONSTANTE (a lei que _qs_regulate usa hoje) ─────
# O bloco NÃO-ULTRAPASSAGEM acima e os _QS_* da lei proporcional
# (Δx = relax·err/K_est, tetos por k_upper, boost, rampa geométrica,
# passo de alívio) descrevem o regulador ANTERIOR. O overshoot dele era
# estrutural: K_est é uma EMA do trecho JÁ percorrido de um contato que
# enrijece, então subestima a inclinação seguinte e o passo err/K_est
# atravessa o alvo. Os _QS_* daquela lei continuam definidos porque os
# comentários documentam por que ela falhava; a lei ATIVA é a rampa abaixo.
#
# _qs_regulate agora move o TCP a VELOCIDADE CONSTANTE (passo v·dt fixo) ao
# longo do eixo de ataque até a força medida cruzar o setpoint, e então
# congela. A rigidez NÃO é ganho — só ENCURTA o passo fixo em dois clamps
# de segurança (_QS_RAMP_DF_CAP_N e a não-ultrapassagem por k_upper). O
# overshoot passa a ser ≤ 1 tick de curso, tendendo a zero perto do alvo, e
# o tempo de cada patamar vira curso/v + stable_s: as duas coisas que a lei
# proporcional não dava.
_QS_RAMP_V_MS = 1.0e-3   # 1,0 mm/s NOMINAL da rampa até o setpoint. Efetivo
                         # ≈ 1/6 disso: _qs_measure_fz congela o braço
                         # _QS_SETTLE_TICKS por leitura. Auto-ajusta: contato
                         # mole gasta o curso, rígido cruza o alvo em < 1
                         # tick. Override ROS: hold_ramp_mms.
# Clamp de SEGURANÇA por tick (NÃO é realimentação — o passo é v·dt fixo,
# isto só o LIMITA): o passo nunca projeta mais que _QS_RAMP_DF_CAP_N de
# força, dividindo por _StiffnessEstimator.value (a K já medida na descida,
# que precede todo HOLD/ESCADA; _K_DEFAULT_NM antes disso). ≈ 2× a banda de
# ruído: o passo cruza o alvo dentro de ~1 banda mesmo no contato mais
# rígido. Mesma classe de guarda que crawl_v_ms é para a descida em ar livre.
_QS_RAMP_DF_CAP_N = 0.1
# Creep viscoelástico: a força a posição constante relaxa com o tempo, então
# a defesa do patamar (etapa B de _qs_regulate) retoma a rampa quando a
# força cai abaixo de alvo−banda, SEM reiniciar o relógio de stable_s.

# O HOLD só libera o SLIDING quando a compressão fica DENTRO da tolerância
# em torno do setpoint por _HOLD_STABLE_S contínuos.
#
# A banda (σ da célula, os 4σ e os 5 % do alvo) mora em constants.py desde
# 24/08/2026: a GUI também precisa dela, e enquanto ela tinha um default
# próprio de 0,15 N a lei daqui nunca valia num run lançado pela tela — a
# PalpationStart sobrescreve este default sempre que traz hold_tol_n > 0.
# Os aliases privados ficam para não reindentar o arquivo inteiro.
_HOLD_STABLE_S  = 5.0    # s contínuos dentro da tolerância (janela estável)
# Teto da etapa A do HOLD (antes do 1º cruzamento). Subiu de 8 → 25 s com a
# rampa: normalmente o HOLD entra já no alvo (o DESCENDING trouxe a força até
# lá) e cruza no tick 1, mas se sobrar um degrau de força para fechar, a
# rampa a v constante leva mais que os 8 s calibrados para a lei proporcional
# — e expirar aqui PULA o dwell de medição e entrega o SLIDING fora do alvo.
_HOLD_TIMEOUT_S = 25.0   # s: teto de espera pela estabilização
# Após estabilizar, mantém o setpoint por mais _HOLD_DWELL_S antes de liberar
# SLIDING/recuo.
_HOLD_DWELL_S   = 5.0

# ── CALIBRAÇÃO DINÂMICA DO ÂNGULO DE ATAQUE ───────────────────────────
# A aproximação padrão desce na VERTICAL do mundo e SUPÕE o alvo
# perpendicular à home. Quando a peça está empenada, ou o calço deixou a
# face torta, a ponteira encosta de canto: a força que a célula lê deixa de
# ser a força normal à superfície (é a projeção dela), o contato vira uma
# aresta em vez da face, e o SLIDING começa a perder contato no meio do
# curso — o sintoma que _SLIDE_LOST_ABORTS documenta como "problema de
# montagem".
#
# A calibração troca a suposição por uma MEDIÇÃO: N toques leves num
# polígono regular em torno do ponto de aproximação, ajuste ortogonal do
# plano (plane_probe.fit_plane) e o eixo de ataque passa a ser a normal
# medida, com o punho reorientado em ar livre para chegar alinhado.
#
# DESLIGADA por padrão. Duas fontes de configuração, na mesma precedência
# de force_mod_* (ver _align_params): os campos probe_align_* da
# PalpationStart quando a GUI os manda, senão os PARÂMETROS ROS abaixo —
#   ros2 param set /tactile_explorer probe_align_enable true
# Os limites e defaults compartilhados com a GUI moram em constants.py
# (PROBE_ALIGN_*); só o que é privado desta fase fica aqui.
#
# Abaixo desta tolerância o desvio medido não paga uma rotação de punho: a
# própria IK converge com erro dessa ordem, e girar por menos que isso só
# adiciona movimento (e risco) sem melhorar o alinhamento.
_ALIGN_ORI_TOL_DEG   = 2.0
# Velocidade dos trânsitos em ar livre da calibração (m/s). Conservadora de
# propósito: eles correm sobre uma peça cuja altura ainda não se conhece.
_ALIGN_TRANSIT_MS    = 0.010   # 10 mm/s
# Folga somada à excursão geométrica (raio × tan(desvio máx)) ao definir onde
# o estágio rápido das descidas de sonda tem de largar a velocidade.
_ALIGN_ZONE_EXTRA_M  = 0.001   # 1 mm

# ── MANUAL em DEGRAU (escada de força) ────────────────────────────────
# O HOLD infinito do modo MANUAL passa a percorrer patamares sozinho:
# sobe de start até max de step em step, mede em cada um, e volta descendo
# pelos MESMOS patamares (a ida-e-volta é o que revela histerese/relaxação
# do material — subir só mede a curva de carga).
_STEP_START_DEFAULT_N = 0.5
_STEP_DWELL_DEFAULT_S = _HOLD_DWELL_S   # patamar de medição por degrau
# A geração dos patamares (staircase_levels) e o teto STEP_MAX_LEVELS moram
# em constants.py — a GUI usa a MESMA função para prever a escada.

# ── TOUCH: força MODULADA (perfil trigonométrico) ─────────────────────
# O setpoint deixa de ser constante e passa a oscilar entre f_min e f_max na
# frequência pedida. Segue por FEEDFORWARD de posição (Δx = ΔF/K_est), não
# pela malha quase-estática: cada passo do QS custa _QS_SETTLE_TICKS parados
# para medir (~150 ms), o que limita aquele caminho a ~1 Hz.
_FMOD_SHAPES = ('OFF', 'SINE', 'COSINE')
# Amostras por período abaixo das quais a onda comandada deixa de ser uma
# onda. O número não é estético: entre os pontos o controlador INTERPOLA, e
# interpolar linearmente uma senoide amostrada a N pontos por período é
# convoluí-la com dois boxcars — a fundamental sai com ganho sinc²(1/N) e o
# resto vira harmônico:
#
#     N     ganho da fundamental      THD
#     4            81,1 %            12,0 %
#     5            87,5 %             6,8 %
#     6            91,2 %             4,5 %
#     8            95,0 %             2,0 %
#
# Era 8, o que travava o teto em 1/(0,020·8) = 6,25 Hz e tornava 10 Hz
# "inalcançável em NENHUMA configuração". Com 5 o teto passa a ser
# exatamente 1/(0,020·5) = 10,0 Hz — o piso de 20 ms do ServoJ é do
# FIRMWARE e não se negocia, então 5 pontos por período é o preço de 10 Hz.
# Os 12,5 % de ganho que a interpolação come são DEVOLVIDOS por
# _fmod_sampling_gain (a amplitude comandada já sai dividida por ele); os
# 6,8 % de THD são irredutíveis nesta cadência e vão no log.
_FMOD_MIN_PTS_PER_CYCLE = 5
# Tick da ONDA, separado do _CTRL_DT da regulação quase-estática. Os 30 ms do
# QS existem porque ele MEDE: cada passo congela o braço para o pipeline
# One-Euro + JTC esvaziar antes da leitura. A onda não mede nada — é
# feedforward puro de posição — e o que ela precisa é de PONTOS por período.
# Amarrá-la ao tick do QS limitava a onda a 33/8 ≈ 4 Hz por um motivo que não
# se aplica a ela.
#
# O tick é derivado da frequência pedida: dt = 1/(f · pontos_por_período), com
# piso. O piso NÃO é uma escolha estética — abaixo dele o laço Python + a
# publicação da trajetória não fecham o ciclo a tempo, e o dt real passa a ser
# maior que o pedido (a cadência MEDIDA no log denuncia isso).
_FMOD_DT_MIN_S = 0.004        # 250 Hz: piso do tick da onda
# Piso do `t` do ServoJ imposto pelo FIRMWARE do CR10, não por este código:
# "Dobot TCP/IP Remote Control Interface Guide V4.5.1", comando ServoJ —
# "t (float): Running time of the point, unit: s, value range: [0.02,3600.0]".
# Abaixo disso o controlador não aceita o ponto, e a mensagem de recusa da
# onda chegava a SUGERIR valores fora da faixa (1/(f·8) vale 15,6 ms já a
# 8 Hz). O mesmo documento fixa o teto útil: "The calling frequency is
# recommended to be set to 33Hz, that is, the interval of cyclic calling is
# 30ms" — os _CTRL_DT deste arquivo.
_SERVOJ_T_MIN_S = 0.020
# Adaptação de K DURANTE a onda. A cada ciclo completo, a secante
# ΔF_medido/Δx_entregue é uma medida direta da rigidez NA amplitude e NA
# frequência do ensaio — melhor que a estimada na descida quase-estática, que
# num material viscoelástico (silicone) é outra coisa. Não é malha de força na
# onda: é adaptação LENTA de um parâmetro, um ciclo por vez.
_FMOD_K_ADAPT_ALPHA = 0.35    # EMA da correção por ciclo
# N: fundamental medida abaixo disso é ruído. O valor FICA com a FA7155, e
# só agora ele é folgado de verdade. O ruído do lock-in sobre N amostras vale
# 4σ/√(2N): com o cru da FA7155 (σ = 21,9 mN) e ~400 amostras por ciclo a
# 1 Hz dá 3,1 mN, então o piso é ~10× o chão. Com a HX711 crua (σ = 112 mN,
# ~24 amostras/ciclo) o mesmo cálculo dava 65 mN — o piso ficava ABAIXO do
# ruído e não filtrava nada. Baixá-lo agora só liberaria adaptação para ondas
# de amplitude menor que o CONTACT_ON_N, que não é ensaio que a bancada peça.
_FMOD_K_ADAPT_MIN_DF_N = 0.03
_FMOD_MAX_AMP_N = 5.0     # N: amplitude (pico) máxima aceita, por segurança
# Teto do passo por tick, em FORÇA projetada (Δx = ΔF/K). O passo da onda é
# grande perto do zero-crossing (amp·2πf·dt) e o teto do QS, de 10 µm,
# achataria a senoide; o teto aqui é o que a própria onda pede, com folga.
_FMOD_DF_STEP_MAX_N = 1.5
# Teto de ticks do arranque em fase (ver _phase_hold_modulated). A rampa vale
# amp/K de penetração em passos de _FMOD_DF_STEP_MAX_N/K, ou seja
# amp/_FMOD_DF_STEP_MAX_N ticks — no máximo 5/1,5 ≈ 4. O teto só existe para
# a rampa não virar laço infinito se o braço não responder.
_FMOD_RAMP_MAX_TICKS = 20
# Teto da VELOCIDADE do TCP na onda, independente de tudo o mais. Os outros
# tetos (_FMOD_DF_STEP_MAX_N, step_cap) limitam FORÇA por passo; com o tick
# caindo para 4 ms em alta frequência, o mesmo ΔF por passo vira uma
# velocidade 7x maior. Este teto é o que impede uma K subestimada de virar um
# movimento rápido: 150 mm/s é folgado para qualquer onda legítima
# (24 Hz x 0,4 mm de amplitude = 60 mm/s de pico) e muito abaixo do braço.
_FMOD_V_MAX_MMS = 150.0
# Desvio tolerado entre a frequência PEDIDA e a MEDIDA na onda entregue antes
# de o log virar aviso. A medida vem da contagem de cruzamentos da penetração
# por FK, que é grosseira em ondas de poucos ciclos; 20 % é folgado o
# bastante para não gritar à toa e apertado o bastante para pegar um executor
# que entrega metade da frequência.
_FMOD_FREQ_TOL_FRAC = 0.20
# Rampa de AMPLITUDE dos primeiros ciclos. A adaptação de K/curva só corrige
# um ciclo por vez (EMA de _FMOD_K_ADAPT_ALPHA), então um erro de rigidez de
# 4x leva ~5 ciclos para convergir — e até lá a onda roda com a amplitude
# cheia. Medido em 14/08/2026 no run 20260814_115804: o ciclo 1 saiu com
# 1400 µm p-p e a força foi a 3,90 N contra os 3,00 pedidos; a adaptação
# convergiu (K de 0,76 para 6,16 N/mm) mas só no ciclo 5, e o operador
# abortou antes. Abrir em fração da amplitude limita o estrago do primeiro
# ciclo à mesma fração, sem mudar a onda depois que a rampa termina.
_FMOD_AMP_RAMP_START  = 0.25   # fração da amplitude no ciclo 0
_FMOD_AMP_RAMP_CYCLES = 3.0    # ciclos até 100 %

# ── CONTROLE REPETITIVO DA ONDA (ILC) ────────────────────────────────
# O QUE A ADAPTAÇÃO POR CICLO NÃO CONSEGUE FAZER, e por que precisa de um
# vetor no lugar de um escalar.
#
# `fx_gain` é UM número, ajustado pelo módulo do lock-in na fundamental. Ele
# corrige AMPLITUDE e mais nada — a fase é descartada pelo `hypot` de
# propósito. Medido no run TOUCH/20260828_154934 (SINE 0,20–2,00 N @ 1 Hz,
# 20 ciclos), é exatamente o que se vê: a amplitude sai certa e todo o resto
# sai errado.
#
#   grandeza                pedido      medido        veredito
#   frequência              1,00 Hz     1,00 Hz       ok
#   amplitude fundamental   0,890 N     0,933 N       ok (ganho 1,047)
#   centro                  1,099 N     1,454 N       +0,355 N, e DERIVANDO
#   pico                    2,00 N      2,852 N       +43 %, 19 % do tempo
#                                                     acima do teto pedido
#   forma                   senoide     THD 32 %      resíduo 29 % da amp
#   fase                    0°          −55,5°        154 ms
#
# Um escalar não distingue essas três falhas: qualquer valor de `fx_gain` que
# acerte a amplitude deixa centro, fase e forma como estão.
#
# A CORREÇÃO É INDEXADA POR FASE. Cada bin do ciclo guarda sua própria
# correção de penetração, aprendida do erro medido NAQUELA fase no ciclo
# anterior. As três falhas caem no mesmo mecanismo sem precisar ser
# identificadas: erro de centro é a componente DC do vetor, erro de fase é a
# componente em quadratura, distorção são os harmônicos dele.
#
# POR QUE ISSO FUNCIONA COM O ATRASO que impede a malha fechada. O ILC não
# realimenta DENTRO do ciclo — ele corrige o ciclo SEGUINTE. O atraso de
# 154 ms deixa de ser um problema de estabilidade e vira o que é: um
# deslocamento conhecido entre o comando e a medida, que se desconta ao
# indexar (ver fmod_measure_lag_s).
# Bins de fase por ciclo. O número nasceu amarrado à MEDIDA: com a HX711 a
# 24 Hz, uma onda de 1 Hz dava ~24 amostras por ciclo, ou seja ~1 ponto por
# bin, e mais bins seriam bins vazios. Com a FA7155 (~400 Hz entregues, ver
# FT_NOMINAL_RATE_HZ) são ~400 amostras por ciclo a 1 Hz — ~16 por bin — e a
# medida deixou de ser o limitante. Quem limita agora é o COMANDO: no pior
# caso a onda sai com _FMOD_MIN_PTS_PER_CYCLE (5) pontos por período, e
# correção indexada em mais bins do que há pontos comandados não tem onde ser
# aplicada. Por isso 24 FICA: a célula nova deu folga na medida, não no
# comando.
_FMOD_ILC_BINS  = 24
_FMOD_ILC_ALPHA = 0.4     # ganho de aprendizado por ciclo
# Teto da correção, em frações da amplitude em posição. O ILC corrige erro de
# EXECUÇÃO; se ele pedir mais que isto o problema é outro (contato perdido,
# K absurda, tare errado) e insistir só afunda a ponteira.
_FMOD_ILC_MAX_FRAC = 0.6
# Ciclos rodados antes de o ILC começar a aprender. A rampa de amplitude
# ocupa os primeiros _FMOD_AMP_RAMP_CYCLES e durante ela a onda pedida NÃO é
# a onda final — aprender ali é aprender a corrigir a rampa.
_FMOD_ILC_WARMUP_CYCLES = _FMOD_AMP_RAMP_CYCLES + 1.0






# Ganho mínimo do pipeline de medida para uma correção por ciclo (ILC, e a
# adaptação de amplitude) poder fechar contra ele. 0,70 é o ganho a 2 Hz, que
# é onde o cutoff travado do One-Euro deixa de ser transparente: acima disso a
# medida perde mais de 30 % e o laço passa a corrigir um erro que é do FILTRO,
# não da onda. Enquanto a onda ler o Float32 filtrado, este é o teto real de
# frequência para QUALQUER coisa adaptativa.
_FMOD_ILC_MIN_MEAS_GAIN = 0.70


# Tolerância do limitador de excursão pela força MEDIDA. A leitura chega
# atrasada em relação ao comando (transporte + filtro): medido nos runs de
# 17/08/2026, a força entregue atrasa ~85 ms constantes — 14° a 0,5 Hz, 29°
# a 1 Hz, 65° a 2 Hz. Sem tolerância o pico atrasado ultrapassa f_max em
# TODO ciclo e o limitador corta, abrindo um entalhe no topo da senoide. A
# tolerância é o que separa "guarda de excursão" de "regulador por ciclo".
#
# COM A FA7155 O GUARDA MUDOU DE INIMIGO, e por isso os dois números ficam.
# Ele compara contra `fz_meas`, que na bancada nova é o CRU (a onda lê
# /load_cell/sample_net; ver _fz_raw), então o atraso do One-Euro sai da
# conta: dos 154 ms medidos a 1 Hz sobram os ~14° de transporte do executor
# e do material, ~39 ms. Em compensação o cru não tem filtro nenhum, e é o
# ruído dele que passa a dimensionar o piso: σ = 21,9 mN, logo 0,10 N são
# 4,6σ — margem sã contra um corte espúrio. Na HX711 crua (σ = 112 mN) esse
# mesmo piso valia 0,9σ e o guarda teria cortado no ruído; o piso só virou
# defensável agora.
_FMOD_BAND_TOL_FRAC = 0.15     # da amplitude pedida
_FMOD_BAND_TOL_MIN_N = 0.10    # N: piso, para amplitudes pequenas
# Velocidade de PICO da onda (2·π·f·amp). Diferente de _FMOD_V_MAX_MMS, que
# corta passo a passo DENTRO do laço: estes dois são checados ANTES de a onda
# abrir, quando ainda dá para recusar o ensaio em vez de executá-lo errado.
# Os 150 mm/s do teto por passo nunca mordem — com a amplitude comandada de
# 2,07 mm a 5 Hz o pico era 63 mm/s e o teto por tick valia 3,75 mm.
_FMOD_V_PEAK_WARN_MMS = 20.0   # acima disto avisa com os números
_FMOD_V_PEAK_MAX_MMS  = 40.0   # acima disto recusa o ensaio
_FMOD_CYCLES_DEFAULT = 20  # períodos por toque quando a GUI não disser outro
# Piso de ruído da FK do feedback real, usado onde se pergunta se a penetração
# medida ainda se move (a onda é micrométrica: ΔF/K).
_FMOD_QUIET_FLOOR_M = 3.0e-6


# Idade máxima da última leitura de /load_cell/force_net para o controle por
# força ser confiável.
_FORCE_STALE_S = 0.5

# Idade máxima de um /palpation/start para ele ser considerado um pedido NOVO.
# O tópico é TRANSIENT_LOCAL (o logger sobe depois do publish e precisa do
# latch), então um explorer que reinicia recebe na hora o último comando da
# sessão anterior. Entrega real leva milissegundos; 10 s é folgado para
# qualquer atraso legítimo e curto para qualquer reinício.
_START_MAX_AGE_S = 10.0


_CTRL_DT    = 0.030   # período de cada passo (33 Hz)
_CTRL_LOOK  = 0.10    # time_from_start do _settle (s)
_CTRL_WIN   = 10      # waypoints por batch de streaming (10 × 30 ms = 300 ms)
_SLIDE_WIN  = 3       # janela de lookahead do SLIDING (3 × 30 ms = 90 ms)
_JAC_LAM    = 0.01    # regularização DLS
_ORI_GAIN   = 0.5     # ganho de correção de orientação
_Z_CORR_GAIN = 0.5   # ganho de correção perpendicular durante sliding
_HOME_MAX_RAD_S = 0.05  # velocidade máxima do HOME (≈ 3°/s por junta);
                        # ajustável via parâmetro ROS home_speed_rad_s
_SETTLE_TICKS   = 6     # ticks de espera entre fases (6 × 30 ms = 180 ms)
# Parada VERIFICADA na entrada do DESCENDING (ver _settle_until_still). Os 180
# ms fixos do _settle são malha aberta: freiam, mas não olham se pararam. Na
# coleta 20260901_094646 o HOME entregou o braço com J4 ainda a 0,057 rad/s e
# 1,25° passado do alvo; ao fim do settle sobravam 0,015 rad/s. Como o
# DESCENDING só comanda Z, quem sobra manda no resto: os 5 primeiros ticks da
# descida saíram a 24° médios (65° de pico) fora da vertical, contra 0,0° nas
# coletas 4x3/4x4, onde o braço entrou parado.
_SETTLE_STILL_TOL_RAD_S = 0.002   # ≈0,11°/s. No braço de ~0,32 m que J4 faz
                                  # até o TCP dá < 0,6 mm/s de deriva lateral,
                                  # ~4 % de um approach de 15 mm/s. O feedback
                                  # das juntas é quantizado em 1e-5 rad, então
                                  # a tolerância está 6x acima do piso de ruído.
_SETTLE_STILL_MAX_TICKS = 40      # teto: 40 × 30 ms = 1,2 s

# Velocidade máxima de referência (rad/s) por junta — equivale ao limite
# físico do CR10 (≈ 180°/s).
_MAX_JOINT_VEL_RAD_S = math.pi  # 180°/s



















