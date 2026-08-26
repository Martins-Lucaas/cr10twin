# Uma amostra da célula de carga COM o carimbo do firmware (/load_cell/sample,
# publicado pelo force_receiver; /load_cell/sample_net, republicado pela GUI já
# com a força tare-compensada).
#
# Existe porque os tópicos Float32 (/load_cell/voltage, /load_cell/force_net)
# jogam fora `seq` e `t_us` no primeiro salto: todo tempo que chegava ao CSV era
# o instante em que o callback rodou no processo do logger, carregando o jitter
# do USB e de 2 saltos ROS. Sem o relógio do firmware não há como reamostrar as
# duas fontes num eixo comum depois da coleta.

uint32 seq        # contador do firmware; salto = amostra perdida no caminho
uint32 t_us       # micros() do MCU da célula, uint32 COM WRAPAROUND de propósito
                  # (o dt é calculado com (t_us - anterior) & 0xFFFFFFFF)

float32 voltage_raw   # tensão da ponte SEM filtro (V) — é ela que permite
                      # refazer a força offline sem o atraso do One-Euro
float32 voltage       # tensão depois do filtro mediana+One-Euro (V)

# Preenchidos só em /load_cell/sample_net (a GUI é quem conhece o tare):
float32 force_net_n   # força tare-compensada (N, compressão POSITIVA)
bool calibrated       # false = sem calibração válida; force_net_n não vale nada
