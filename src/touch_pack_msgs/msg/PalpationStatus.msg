# Status do experimento de palpação (/palpation/status, ~10 Hz).

string phase                # IDLE|HOME|DESCENDING|HOLD|SLIDING|RETRACT|DONE|ABORTED
int32 cycle                 # ciclo atual (1-based; 0 = fora de experimento)
int32 cycles_total          # total de ciclos do experimento
float32 target_depth_mm
float32 target_force_n
float32 force_net_n         # força tare-compensada (N, + compressão)
float32 speed_mms
bool paused                 # true enquanto o experimento está pausado

# ── Contato aprendido POR HOME ───────────────────────────────────────
# true  = a home corrente já tem contato aprendido → descida em DOIS
#         ESTÁGIOS (estágio rápido no ritmo do slider até a zona lenta,
#         rastejo só no trecho final).
# false = home ainda não aprendida → a descida INTEIRA rasteja e re-aprende.
# A GUI espelha isto na checkbox "Home conhecida"; desmarcá-la publica
# /palpation/forget_contact e o explorer volta este campo a false.
bool home_known

# ── MATRIX_MAP ────────────────────────────────────────────────────────
# Preenchidos apenas no modo MATRIX_MAP; zerados nos demais.
int32 wp_index              # waypoint corrente (1-based; 0 = fora da matriz)
int32 wp_total              # total de waypoints da matriz
float32 wp_x_mm             # alvo planejado, relativo à origem (mm)
float32 wp_y_mm
bool origin_valid           # true depois que a origem foi descoberta
float64 origin_x_m          # origem = TCP no primeiro contato (mundo URDF, m)
float64 origin_y_m
float64 origin_z_m
