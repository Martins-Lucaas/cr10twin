# Comando de início do experimento de palpação (/palpation/start).
# Substitui o JSON em std_msgs/String: schema tipado, introspectável
# e legível via `ros2 topic echo`.

float32 speed_mms           # velocidade de deslizamento (mm/s)
float32 depth_mm            # curso máximo de descida — segurança (mm)
float32 force_n             # setpoint de força (N, compressão) do regulador quase-estático
float32 slide_dist_mm       # distância lateral do SLIDING (mm)
float32 approach_speed_mms  # velocidade de aproximação CONTACT/RETRACT (mm/s)
string slide_dir            # '+X' | '-X' | '+Y' | '-Y'
int32 repeats               # ciclos automáticos do experimento (>= 1)
float32 speed_factor_pct    # SpeedFactor do braço real (%)
float64[6] home_deg         # home do braço em graus, ordem joint1..joint6

# Estabilização do setpoint no HOLD — 0.0 = usar o default do explorer.
float32 hold_tol_n          # tolerância absoluta da banda (N)
float32 hold_stable_s       # janela contínua dentro da banda (s)
float32 hold_timeout_s      # teto de espera pela estabilização (s)

# Modo de palpação:
#   'SLIDE'  = deslizamento (descida → hold → deslizamento lateral → recuo)
#   'TOUCH'  = toque (descida → hold com força controlada → recuo, sem deslizar)
#   'MANUAL' = dinâmico (descida → HOLD infinito; setpoint atualizável on-the-fly
#              via /palpation/set_force; encerra em STOP e retorna à HOME)
# Vazio = 'SLIDE' (compatibilidade). Em TOUCH, `repeats` = quantidade de toques.
string mode

# Inclinação da superfície ao longo da direção do deslize (graus; + = a
# superfície SOBE no sentido da marcha). O SLIDING deixa de travar Z num
# plano horizontal e passa a seguir essa rampa — compensação puramente
# geométrica, sem realimentação de força, então a textura fica intacta.
# 0.0 (ou campo ausente em mensagens antigas) = comportamento anterior.
float32 slide_slope_deg

# ── Descida em dois estágios (PROBE) — 0.0 = default do explorer ──────
# Com o contato aprendido para a home, o estágio rápido PARA por completo
# `fine_approach_mm` antes do contato conhecido e só esse último trecho é
# rastejado, a `fine_speed_mms`. Sem contato aprendido os dois campos são
# ignorados: a descida inteira roda no trecho fino.
float32 fine_approach_mm    # folga fina antes do contato aprendido (mm, default 2)
float32 fine_speed_mms      # cadência dentro da folga fina (mm/s, default 1)

# ── MATRIX_MAP — mapeamento tátil em grade/matriz ─────────────────────
# Lista de identações do plano, em METROS e RELATIVAS À ORIGEM que o próprio
# robô descobre no início do run (primeiro contato vertical). A GUI gera a
# lista a partir do formato/passo/linhas escolhidos no configurador visual;
# o explorer só a percorre — não recalcula geometria.
#
# Convenção: waypoints[i].x/.y são o deslocamento NO PLANO DA AMOSTRA a
# partir da origem; .z é IGNORADO (a descida é sempre ao longo da normal, a
# partir do Safe Z). Sem a calibração do ângulo de ataque o plano é o
# horizontal do mundo URDF e .x/.y caem exatamente no XY do mundo — o
# comportamento histórico. Com ela, a grade é traçada no plano MEDIDO, e o
# Safe Z é uma folga ao longo da normal e não uma altura do mundo: é o que
# mantém o passo da grade exato e a folga uniforme sobre uma peça inclinada.
# Lista vazia em modo MATRIX_MAP = erro → run recusado.
#
# O setpoint aplicado em CADA ponto é o `force_n` acima — MATRIX_MAP não tem
# campo de força próprio, para que exista uma única fonte de verdade.
geometry_msgs/Point[] waypoints
float32 safe_z_mm           # folga de trânsito sobre o plano, medida ao longo
                            # da normal (mm); 0 = default 10 mm
float32 transit_speed_mms   # velocidade do deslocamento no plano, no ar
                            # (mm/s); 0 = default
string grid_shape           # 'SQUARE' | 'RECT' | 'CUSTOM' — rótulo p/ log e relatório

# ── FORÇA MODULADA (modo TOUCH) ───────────────────────────────────────
# O setpoint do HOLD deixa de ser constante e passa a seguir
#     F(t) = média + amplitude·trig(2π·f·t),   média = (min+max)/2
# executada por FEEDFORWARD de posição (Δx = ΔF/K, com K medido na descida).
# Só tem efeito em mode == 'TOUCH'; nos demais modos é ignorada.
#
# Campo VAZIO (mensagem antiga, ou `ros2 topic pub` sem estes campos) faz o
# explorer cair no caminho dos PARÂMETROS ROS force_mod_* — é o que mantém
# o fluxo por `ros2 param set` funcionando sem a GUI.
string force_mod_shape      # '' = usar parâmetros ROS | 'OFF' | 'SINE' | 'COSINE'
float32 force_mod_hz        # frequência da onda (Hz); acima de ~4 Hz o laço de
                            # 33 Hz não rastreia a forma — executa com aviso
float32 force_mod_min_n     # força no vale da onda (N, compressão)
float32 force_mod_max_n     # força no pico da onda (N); amplitude = (max−min)/2,
                            # limitada a 5 N e à faixa 0,1–10 N do setpoint
int32 force_mod_cycles      # períodos por toque; duração = cycles / hz

# ── MANUAL em DEGRAU (escada de força) ────────────────────────────────
# Em vez do HOLD infinito com setpoint vindo de /palpation/set_force, o modo
# MANUAL percorre patamares sozinho: sobe de `step_start_n` até `step_max_n`
# de `step_size_n` em `step_size_n`, para `step_dwell_s` em cada um, e volta
# DESCENDO pelos mesmos patamares — a ida-e-volta é o que revela histerese e
# relaxação do material; só subir mede apenas a curva de carga.
#
# O pico entra uma única vez e é sempre `step_max_n` exato: se o passo não
# fecha certo, o último degrau da subida é encurtado para cravá-lo.
#
# `step_size_n` <= 0 (ou campo ausente em mensagem antiga) DESLIGA a escada e
# o MANUAL volta a ser o hold infinito de antes.
float32 step_start_n        # primeiro patamar (N); 0 = default 0,5
float32 step_size_n         # altura do degrau (N); 0 = escada desligada
float32 step_max_n          # patamar de pico (N), saturado em 10 N
float32 step_dwell_s        # medição em CADA patamar (s); 0 = default 5 s

# ── CALIBRAÇÃO DINÂMICA DO ÂNGULO DE ATAQUE ───────────────────────────
# A descida padrão desce na VERTICAL do mundo e supõe o alvo perpendicular à
# home. Quando não está — peça empenada, calço torto —, a ponteira encosta de
# CANTO: a célula lê a projeção da força normal, o contato vira uma aresta e
# o SLIDING sai do plano no meio do curso.
#
# Ligada, uma fase CALIBRATING roda UMA vez por experimento, logo após a
# HOME: N toques leves num polígono em torno do ponto de aproximação, ajuste
# do plano (mínimos quadrados com mais de 3 pontos) e ataque ao longo da
# normal medida — com retração linear antes de girar o punho.
#
# Campo VAZIO (mensagem antiga, ou `ros2 topic pub` sem estes campos) faz o
# explorer cair nos PARÂMETROS ROS probe_align_* — mesmo contrato de
# force_mod_shape acima, e é o que mantém o fluxo sem GUI funcionando.
# Os numéricos em 0 significam "usar o default do explorer".
string probe_align                # '' = usar parâmetros ROS | 'OFF' | 'ON'
int32 probe_align_points          # toques de sonda (mín. 3, recomendado 4+)
float32 probe_align_radius_mm     # raio do polígono de sondagem
float32 probe_align_force_n       # setpoint dos toques; nunca excede force_n
float32 probe_align_retract_mm    # retração linear antes de girar o punho
float32 probe_align_tilt_max_deg  # desvio máximo aceito (teto duro 30°);
                                  # acima disso a calibração ABORTA o run

# ── Identidade do run no disco ────────────────────────────────────────
# <AAAAMMDD_HHMMSS> do relógio de PAREDE, carimbado pela GUI. O logger e a
# GUI gravam arquivos diferentes do mesmo run e precisam concordar no nome
# da pasta — derivar isso de `stamp` NÃO serve: com use_sim_time o carimbo
# vem do relógio do Gazebo, que reinicia do zero a cada launch (todos os
# runs viravam 19691231_*, e o launch seguinte SOBRESCREVIA a coleta
# anterior). O `stamp` continua sendo o relógio do sistema ROS porque é dele
# que o explorer mede a idade do latch; a identidade em disco é este campo.
#
# Vazio (publisher antigo, ou `ros2 topic pub` sem o campo) = derivado de
# `stamp`, o comportamento anterior.
string run_id

# ── Carimbo de emissão ────────────────────────────────────────────────
# O tópico é TRANSIENT_LOCAL de propósito: o palpation_logger sobe DEPOIS do
# publish e precisa do latch para gravar o run desde o início. O efeito
# colateral é que o explorer, ao reiniciar com a GUI viva, recebia na hora o
# último comando e saía movendo o braço — a partir de uma pose que já não é
# a de quando o comando foi emitido.
#
# Com o carimbo o explorer distingue as duas coisas: entrega real leva
# milissegundos, latch de sessão anterior leva minutos. Zerado (publisher
# antigo, ou `ros2 topic pub` sem o campo) = aceita, como os demais campos
# novos.
builtin_interfaces/Time stamp
