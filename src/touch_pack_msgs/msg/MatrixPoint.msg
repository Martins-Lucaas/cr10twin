# Um registro por identação CONCLUÍDA do modo MATRIX_MAP
# (/palpation/matrix_point, publicado pelo tactile_explorer ao fim de cada
# HOLD). É a chave que liga cada trecho da curva de força do CSV à sua
# coordenada espacial no formato do objeto: o palpation_logger grava estes
# campos em <ts>__matrix.csv e carimba wp_index em cada amostra de força.

int32 index                 # 1-based; casa com a coluna wp_index do __samples.csv
int32 total                 # total de waypoints da matriz

# Alvo PLANEJADO pela GUI, relativo à origem (mm).
float32 plan_x_mm
float32 plan_y_mm

# Origem descoberta pelo robô (TCP no primeiro contato, mundo URDF, m).
float64 origin_x_m
float64 origin_y_m
float64 origin_z_m

# TCP MEDIDO via FK no fim do HOLD (mundo URDF, m) — onde o toque de fato
# aconteceu. A diferença para o planejado é o erro de posicionamento.
float64 tcp_x_m
float64 tcp_y_m
float64 tcp_z_m

float32 setpoint_n          # setpoint pedido (force_n do start)
float32 force_n             # força medida no fim do HOLD — o que foi ATINGIDO
float32 depth_mm            # penetração relativa à origem: (origin_z − tcp_z) em mm

float64 t_start_unix        # início da descida deste ponto
float64 t_end_unix          # fim do HOLD deste ponto

# 'ok' | 'no_contact' | 'force' | 'stale' | 'timeout' | 'stop' | 'error'
string outcome
