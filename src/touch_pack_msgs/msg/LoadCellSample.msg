# Uma amostra da célula de carga COM o carimbo de aquisição (/load_cell/sample
# e /load_cell/sample_net, publicados pelo ft_receiver; o _net já com a força
# tare-compensada).
#
# Existe porque o Float32 /load_cell/force_net joga fora `seq` e `t_us` no
# primeiro salto: todo tempo que chegava ao CSV era o instante em que o
# callback rodou no processo do logger, carregando o jitter de 2 saltos ROS.
# Sem o carimbo de aquisição não há como reamostrar as duas fontes num eixo
# comum depois da coleta.

uint32 seq        # contador de quadros; salto = amostra perdida no caminho
uint32 t_us       # µs do relógio do host na leitura, uint32 COM WRAPAROUND de propósito
                  # (o dt é calculado com (t_us - anterior) & 0xFFFFFFFF)

# O nome dos dois campos abaixo é histórico: com a FA7155 eles carregam a
# força do eixo de controle em NEWTONS.
float32 voltage_raw   # SEM filtro — é ele que permite refazer a força offline
                      # sem o atraso do One-Euro
float32 voltage       # depois do filtro mediana+One-Euro

# Preenchidos só em /load_cell/sample_net (já com o tare):
float32 force_net_n   # força tare-compensada (N, compressão POSITIVA)
bool calibrated       # false = sem calibração válida; force_net_n não vale nada
