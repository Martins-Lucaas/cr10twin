"""
constants.py — Constantes compartilhadas do touch_pack.

Regra: valores usados por MAIS de um módulo moram aqui; valores privados
de um único módulo ficam nele.
"""
from __future__ import annotations

import math
import os

# Cadeia do braço CR10 (convenção URDF).
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Pose "apontar para a mesa": home default da GUI e seed POINTING do explorer.
POINTING_SEED_DEG = {'joint1': 0.0, 'joint2': 0.0, 'joint3': -90.0,
                     'joint4': 0.0, 'joint5': 90.0, 'joint6': 0.0}

# Mão COVVI — juntas primárias.
HAND_JOINTS = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate']

# Pose POINTING (palpação com o Index estendido).
HAND_POINT_DEG = {'Thumb': 30.0, 'Index': 0.0, 'Middle': 80.0,
                  'Ring': 80.0, 'Little': 80.0, 'Rotate': 0.0}
HAND_POINTING_RAD = {j: math.radians(v) for j, v in HAND_POINT_DEG.items()}

# Limite de segurança: medição CANCELADA se a compressão exceder este valor.
FORCE_ABORT_LIMIT_N = 15.0
# Setpoint máximo selecionável na GUI.
FORCE_SETPOINT_MAX_N = 10.0

# Força mínima que caracteriza CONTATO. Fonte única: é o mesmo limiar que o
# explorer usa como gatilho de halt na descida (_CONTACT_ON_N) e que a GUI usa
# para acender o indicador "in contact". Mora aqui porque os dois precisam
# dele — a GUI tinha um 0.2 cravado à mão que nunca acompanhou os retunes do
# limiar (0,11 → 0,06 → 0,10) e abria uma zona cega de 0,10–0,20 N onde o robô
# já considerava que tocou e a tela ainda dizia "no contact".
# A justificativa do VALOR (σ da célula em repouso, carga inercial do traverse)
# está no bloco de _CONTACT_ON_N em tactile_explorer.py — não duplicar aqui.
CONTACT_ON_N = 0.10
# Histerese do INDICADOR — só da tela, o controle não a usa. Acende em
# CONTACT_ON_N e só apaga abaixo desta fração: com o indicador exatamente igual
# ao gatilho, o verde pisca em ar livre toda vez que o ruído cruza o limiar.
CONTACT_OFF_FRAC = 0.7

# ── Banda de força do HOLD — fonte única GUI ↔ explorer ───────────────
# Mora aqui pelo mesmo motivo que CONTACT_ON_N: os DOIS precisam dela, e
# enquanto a GUI carregava um 0,15 N cravado à mão o retune de 19/08/2026
# (que derivou a banda do ruído MEDIDO da célula) nunca chegava a valer num
# run lançado pela tela — a mensagem de PalpationStart sobrescreve o default
# do explorer sempre que traz hold_tol_n > 0, e ela sempre trazia.
#
# O piso da banda é o RUÍDO da célula: não existe banda mais estreita que a
# incerteza da medida. Era 0,15 N solto (~6,5σ do HX711), que num alvo de
# 0,1 N abria [−0,05; +0,25] N — incluía força ZERO, e overshoot deixava de
# ser mensurável.
FORCE_NOISE_SIGMA_N = 0.023  # N: σ em repouso. HX711, 17/08/2026, 2390
                             # quadros de MANUAL/20260817_142719.
                             # RE-MEDIR com a FA7155: é ELE que fixa o menor
                             # setpoint perseguível.
# 4σ e não 3σ: sair da banda RESETA a janela de estabilidade (165 leituras).
# A 3σ seriam ~0,5 excursões por janela e o hold reiniciaria sozinho.
HOLD_TOL_SIGMA = 4.0
HOLD_TOL_N     = HOLD_TOL_SIGMA * FORCE_NOISE_SIGMA_N   # 0,092 N
HOLD_TOL_PCT   = 0.05   # fração do setpoint (5 %)


def hold_tol_n(target_f: float) -> float:
    """Meia-banda do HOLD para um setpoint: o maior entre o piso de ruído
    (4σ) e a fração do alvo. É a LEI do explorer, exposta aqui para a GUI
    poder mostrar (e mandar) o mesmo número em vez de um default próprio."""
    return max(HOLD_TOL_N, HOLD_TOL_PCT * abs(float(target_f)))

# ── Célula de 6 eixos FA7155 (RS485 em modo ATIVO) ────────────────────
# Substitui a célula axial de 1 eixo. Não há placa nossa no caminho: o sensor
# fala RS485 direto com o PC por um conversor USB (ZK-U485/CH340), e o driver
# é o ft_receiver. Ver ft_serial.py para o formato do quadro.
#
# BAUD E TAXA SÃO DO EXEMPLAR, NÃO DO CASO GERAL DO MANUAL. Confirmado em
# 26/08/2026 na bancada: a unidade montada no flange fala 1 Mbps e entrega
# 1 kHz — e não os 115200/250 Hz que o manual dá como default do caso geral.
# Estes dois números são a FONTE DE VERDADE do repo: ft_serial, ft_tcp,
# ft_receiver e a aba "Load cell" da GUI leem daqui. Se a unidade for
# reconfigurada pelo canal Modbus (Send_Baud_rate / Send_Frequency), é aqui
# que o novo valor entra.
FT_SERIAL_BAUD  = 1_000_000   # exemplar da bancada (manual §4.3 dá 115200 como
                              # default do caso geral)
FT_FRAME_HEADER = b'\x53\x54'
FT_FRAME_LEN    = 28          # 2 (cabeçalho) + 6×float32 + 2 (CRC-16/MODBUS)
# Ordem dos seis canais dentro do quadro — é ela que dá nome às colunas.
FT_AXES = ('fx', 'fy', 'fz', 'mx', 'my', 'mz')
# Taxa do exemplar em uso (manual §3.1: a série aceita 500–1000 Hz sob
# encomenda, e esta unidade veio no topo da faixa).
FT_NOMINAL_RATE_HZ = 1000.0
# Teto ABSOLUTO do link: 28 bytes × 10 bits / 1 Mbps ≈ 0,28 ms por quadro.
# Um sensor acima disto NÃO cabe no baud em uso e vai chegar picotado.


def ft_max_rate_hz(baud: float = FT_SERIAL_BAUD) -> float:
    """Teto de quadros/s que CABE num dado baud (28 B × 10 bits por quadro).

    É função e não só constante porque o baud é PARÂMETRO do nó (`ft_baud`):
    o aviso do ft_receiver imprimia o baud pedido ao lado de um teto sempre
    calculado sobre o FT_SERIAL_BAUD do módulo, e com um `ft_baud` diferente
    (`ft_baud:=115200`, por exemplo) o número que ele mandava conferir estava
    errado na mesma proporção.
    """
    return float(baud) / (FT_FRAME_LEN * 10)


FT_MAX_RATE_HZ = ft_max_rate_hz()   # ≈ 3571 Hz a 1 Mbps
# Abaixo disto o receiver avisa: cabo ruim, baud errado ou taxa de fábrica
# diferente da configurada.
FT_MIN_RATE_HZ = 100.0
# Fundo de escala do FA7155 na bancada. Serve de DUAS coisas: sanidade (uma
# leitura muito além disto é ruído de sincronismo, não força) e escala das
# barras da aba "6 Axes".
#
# 19/08/2026: confirmado pela PLAQUETA da unidade montada, que diz
# "FA7155D-400N/20NM". A variante oscilou entre B e D nas notas de hoje, todas
# baseadas em relato; a plaqueta é a fonte física e prevalece. A tabela de
# variantes (datasheet da série, p. 1) confirma D = 400 N / 20 N·m.
#
# ATENÇÃO: isto é FUNDO DE ESCALA (capacidade nominal), NÃO o limite de
# deformação. A sobrecarga segura é outro número, que vem do datasheet — ver
# FT_SAFE_OVERLOAD_PCT abaixo.
FT_RATED_FORCE_N   = 400.0
FT_RATED_TORQUE_NM = 20.0

# Sobrecarga segura, em % do fundo de escala: acima disto o fabricante não
# garante retorno ao zero (deformação permanente). None = desconhecido, e a
# GUI então avisa que não pode desenhar a faixa de risco em vez de inventar
# um limite.
#
# 19/08/2026: preenchido com o datasheet da série
# (Docs/"FA7155 SeriesSix-axis force sensor.pdf", tabela da p. 1, linha
# "Overload level(%FS)"), que dá 300 % para os quatro modelos A/B/C/D. É a
# mesma linha que o manual do modo ativo traduz como "Anti-G % 300".
# Em N, para a variante D da bancada: 3 × 400 = 1200 N e 3 × 20 = 60 N·m.
FT_SAFE_OVERLOAD_PCT = 300.0

# Rótulo e unidade de cada eixo, na ordem do quadro (FT_AXES).
FT_AXIS_LABELS = (
    ('fx', 'Fx', 'N'),
    ('fy', 'Fy', 'N'),
    ('fz', 'Fz', 'N'),
    ('mx', 'Mx', 'N·m'),
    ('my', 'My', 'N·m'),
    ('mz', 'Mz', 'N·m'),
)


def ft_axis_rated(axis: str) -> float:
    """Fundo de escala do eixo: força para fx/fy/fz, torque para mx/my/mz."""
    return FT_RATED_FORCE_N if axis in ('fx', 'fy', 'fz') else FT_RATED_TORQUE_NM
# VIDs dos chips de ponte USB-serial usados em conversores RS485. O FA7155 não
# aparece na USB — quem aparece é o conversor. Também EXCLUÍDOS do auto-detect
# do touch sensor (ver touch_source.detect_serial_port).
FT_USB_VIDS = (
    0x1A86,   # WCH — CH340/CH341/CH343 (o ZK-U485 azul da bancada)
    0x10C4,   # Silicon Labs — CP2102/CP2104
    0x0403,   # FTDI — FT232
    0x067B,   # Prolific — PL2303
)

# ── Rota alternativa: RS485 do FLANGE, via porta 60000 do controlador ──
# Mesmo sensor e mesmo quadro; muda só o cano. O controlador CR expõe a 485 do
# conector aviação do punho como um socket TCP cru: o guia TCP/IP, no exemplo 3
# do ModbusCreate, diz que a 60000 é "the 485 interface at the end of the robot
# arm". Verificado na bancada — 192.168.5.2:60000 aceita conexão externa.
# Escolha entre os dois transportes pelo parâmetro `ft_transport` do nó.
FT_TCP_HOST = '192.168.5.2'   # mesmo IP do CR10 (ver real_driver)
FT_TCP_PORT = 60000
# Eixo do FA7155 que faz o papel da antiga célula axial, e o sinal que o põe na
# convenção do sistema (COMPRESSÃO POSITIVA).
#
# Fz+ aponta para FORA da face da ferramenta (figura 2 do manual). Ao empurrar
# a ponteira contra a amostra, a reação entra no sensor e o Fz medido fica
# NEGATIVO — daí o sinal −1. CONFIRA na bancada com `ft_probe.py` antes do
# primeiro ensaio: se apertar a ponteira der força negativa, inverta o
# parâmetro `ft_force_sign` do nó.
FT_FORCE_AXIS_DEFAULT = 'z'
FT_FORCE_SIGN_DEFAULT = -1.0

# ── Canal de COMANDO Modbus RTU do FA7155 ─────────────────────────────
# Até 26/08/2026 este repo tratava a célula como talker passivo. A análise do
# cliente de fábrica (Six_Axis_FT.exe, Qt5 — ver ft_modbus.py) mostrou que o
# sensor é também um ESCRAVO Modbus RTU na mesma linha 485, e que os comandos
# Set_Zero / Send_Frequency / Send_ModBus_ID / Send_Baud_rate / StartReading
# existem. O que o binário NÃO entrega são os endereços: são constantes
# numéricas no código, não strings.
#
# ┌─ COMO PREENCHER (uma sessão de bancada, ~30 min) ──────────────────┐
# │ 1. Ligue o FA7155 no conversor USB-RS485.                          │
# │ 2. Abra o cliente de fábrica e conecte na aba "Modbus".            │
# │ 3. Capture a porta serial (Portmon, ou um 2º adaptador em escuta   │
# │    na linha 485 — só RX, sem TX, para não colidir).                │
# │ 4. Acione UM comando de cada vez e anote o quadro que sai:         │
# │       <slave> <0x06|0x10> <addr_hi> <addr_lo> <valor…> <crc_lo>    │
# │       <crc_hi>                                                     │
# │ 5. addr = (addr_hi<<8)|addr_lo → é o número que entra aqui.        │
# │ 6. Ponha FT_MODBUS_MAP_CONFIRMED = True.                           │
# └────────────────────────────────────────────────────────────────────┘
#
# ENQUANTO ISTO FOR False, `FtDevice` RECUSA TODA ESCRITA. Não é excesso de
# zelo: endereço errado troca o node ID (e você perde o escravo na linha) ou
# muda o baud para um que o host não fala mais — e o sensor não tem botão de
# reset de fábrica. Leitura continua liberada: no pior caso o escravo devolve
# exceção 0x02 (endereço ilegal), que não altera nada nele.
FT_MODBUS_MAP_CONFIRMED = False

# None = endereço desconhecido. Os *_value/_on/_off são os payloads, que a
# captura também revela (o cliente mostra "reset 0 success." / "start 1
# success.", sugerindo canal 0/1 — confirme qual valor corresponde a quê).
FT_MODBUS_MAP: dict = {
    'zero':       None,   # Set_Zero        (0x06, escrita simples)
    'zero_value': 1,
    'rate':       None,   # Send_Frequency  (0x10, 2 regs = u32 Hz)
    'node_id':    None,   # Send_ModBus_ID  (0x06)
    'baud':       None,   # Send_Baud_rate  (0x10, 2 regs = u32 baud)
    'stream':     None,   # StartReading / stopReading (0x06)
    'stream_on':  1,
    'stream_off': 0,
    'device_id':  None,   # leitura do painel "Device ID"
}

# ── Leitura POLLED (o modo que o HMI/cliente de fábrica usa) ──────────
# CONFIRMADO 26/08/2026 pela tela de comando do HMI, conferida byte a byte:
#
#     requisição   01 03 00 03 00 0C B5 CF
#     resposta     01 03 18 <24 B = 6x float32 LE> 10 A3
#
# Os dois CRCs fecham com crc16_modbus() e os 24 bytes decodificam para 50,0
# nos seis eixos (o valor de exemplo da tela). test_ft_polled.py usa esses
# dois quadros como vetores dourados, no mesmo espírito do test_ft_hkvl56.
#
# Isto é um caminho DIFERENTE do stream "ST" de 28 bytes: aqui o host é
# MESTRE e pergunta, em vez de escutar um talker. Os dois convivem na mesma
# linha 485 e o parâmetro `ft_mode` do nó escolhe qual usar.
#
# Detalhe que decide o parser: a resposta traz BYTECOUNT (0x18), não o
# endereço inicial. Ou seja, o FA7155 fala o enquadramento Modbus PADRÃO, e
# NÃO o estilo HKVL-56 (que põe 2 bytes de endereço no lugar e teria 30
# bytes). Use STYLE_STANDARD com count=12 ao decodificar.
FT_MODBUS_DATA_ADDR = 0x0003   # holding register inicial dos seis eixos
FT_MODBUS_DATA_REGS = 12       # 12 regs = 24 bytes = 6 x float32 LE


def ft_polled_max_rate_hz(baud: float = 115200) -> float:
    """Teto de amostras/s no modo POLLED — outra conta que a do stream.

    No stream você paga 28 B por amostra e mais nada. Aqui cada amostra custa
    a requisição (8 B), a resposta (29 B) e os DOIS silêncios de 3,5
    caracteres que o Modbus RTU exige entre quadros:

        (8 + 29) x 10 bits + 2 x 35 bits = 440 bits por amostra

    A 115200 isso dá ~262 Hz. É o teto do FIO: a latência de USB (~1 ms por
    sentido num CH340) não entra aqui e derruba a taxa medida para algo em
    torno de 150-200 Hz. Por isso a GUI mostra a taxa MEDIDA ao lado deste
    teto, em vez de prometer o número teórico.
    """
    return float(baud) / ((8 + 29) * 10 + 2 * 35)


# Modo de aquisição. Ortogonal ao MEIO (`ft_transport`: serial ou tcp) — dá
# para pollar pela 485 do flange na porta 60000 exatamente como pelo USB.
FT_MODE_STREAM = 'stream'
FT_MODE_POLLED = 'polled'
FT_MODE_CHOICES = (FT_MODE_STREAM, FT_MODE_POLLED)

# ── Gráficos: paridade com o cliente de fábrica ───────────────────────
# Números lidos de Csv/Range.csv da instalação do Six_Axis_FT.exe — é onde o
# cliente PERSISTE as escalas, então é o que ele mostra ao abrir.
#
# O mesmo arquivo traz Column_Y1..Y6_Max = -100. Um "máximo" negativo é
# sentinela de "nunca ajustado", não escala: as barras de coluna continuam
# usando o fundo de escala real da unidade (FT_RATED_*), não isto.
#
# E os valores de força/torque abaixo são DEFAULTS GENÉRICOS da série, não
# desta unidade: ±200 N é metade do fundo de escala da variante D (400 N) e
# ±50 N·m é 2,5x o dela (20 N·m). Ficam aqui porque a paridade pedida é com o
# que o cliente MOSTRA ao abrir; a escala é ajustável na própria aba.
FT_CHART_WINDOW_N   = 2000     # Chart_X1_Max / Chart_X2_Max — amostras
FT_CHART_FORCE_MAX  = 200.0    # Chart_Y1_Max / Min — N
FT_CHART_TORQUE_MAX = 50.0     # Chart_Y2_Max / Min — N·m

# Node ID do escravo. 1 é o default Modbus e o que o cliente de fábrica traz
# pré-preenchido; se você mudar com Send_ModBus_ID, mude aqui também.
FT_MODBUS_SLAVE_ID = 1
# Prazo de uma transação. Generoso de propósito: a resposta do comando chega
# no MEIO do stream de 1 kHz e o cliente precisa peneirar os quadros "ST"
# antes de achá-la (ver ft_modbus.find_response).
FT_MODBUS_TIMEOUT_S = 0.5
# Taxas de saída que o cliente de fábrica oferece no combo de frequência.
FT_RATE_CHOICES_HZ = (10, 50, 100, 200, 250, 500, 1000)
# Bauds oferecidos pelo mesmo cliente para a 485.
FT_BAUD_CHOICES = (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600,
                   1_000_000)

# ── Perfil HKVL-56 (Suzhou Hangkai Microelectronics) ──────────────────
# CONFIRMADO por manual, 26/08/2026 — §5.1 (Communication Specifications) e
# §4.2 (Data Format). Os dois quadros de exemplo do manual foram conferidos
# byte a byte contra crc16_modbus() e FECHAM, e os floats de exemplo
# decodificam para os valores impressos (00 00 48 41 -> 12,5 N). Ver
# test_ft_hkvl56.py, que usa os quadros do manual como vetores dourados.
#
# ATENÇÃO — ESTE NÃO É O SENSOR DA BANCADA. Confirmado em 26/08/2026: a peça
# no flange é o FA7155 da Fipos/FIBOS (Changzhou), plaqueta
# "FA7155D-400N/20NM", e o exemplar fala 1 Mbps a 1 kHz. O baud coincide com o
# do HKVL-56 e por isso NÃO distingue os dois; o que distingue é o MODO: o
# FA7155 empurra quadros de 28 B com cabeçalho "ST" sozinho, enquanto o
# HKVL-56 é de OUTRO fabricante e o manual dele descreve Modbus POLLED, sem
# stream. Este perfil fica aqui descrito e testado, e NÃO ativo, por dois
# motivos que valem para o FA7155:
#
#   1. Os quadros publicados neste manual validaram o crc16_modbus() do repo
#      contra um CRC de fabricante (fecham byte a byte) — antes disso a nossa
#      conta nunca tinha sido conferida contra nada externo.
#   2. Os dois desvios do padrão descritos abaixo são um vício comum nesta
#      classe de sensor. Se a captura do FA7155 mostrar uma resposta de 30
#      bytes em vez de 29, o STYLE_HKVL56 do ft_modbus já cobre o caso.
#
# Dois desvios do Modbus padrão, ambos afirmados pela nota do §4.2 e
# coerentes com os CRCs:
#   1. a resposta de 0x03 traz o ENDEREÇO INICIAL (2 bytes) no lugar do
#      BYTECOUNT (1 byte) — 30 bytes de resposta, não 29;
#   2. o campo "number of reads" conta BYTES, não registradores (0x0018 = 24
#      bytes = 6 floats).
FT_PROFILE_HKVL56 = {
    'name':        'HKVL-56',
    'vendor':      'Suzhou Hangkai Microelectronics Technology Co., Ltd.',
    'style':       'hkvl56',
    # "Unless otherwise specified by the customer, the default baud rate when
    #  powering on ... is 1 Mbps" (§4.2, nota final).
    'baud_default': 1_000_000,
    # Exemplo 1: 01 03 00 03 00 18 B5 C0 — lê os seis eixos.
    'data_addr':   0x0003,
    'data_count':  0x0018,      # 24 BYTES (não registradores)
    # Exemplo 2: 01 06 00 00 00 02 08 0B — grava ID = 2.
    # "After modifying the ID, you must power on and restart the device."
    'node_id':     0x0000,
    # O manual só documenta 0x03 (ler) e 0x06 (escrever ID). Zero/tare, taxa
    # de saída e baud NÃO aparecem nele — continuam por capturar.
    'zero':        None,
    'rate':        None,
    'baud':        None,
    'stream':      None,
}

# ── Pós-processamento à moda do cliente de fábrica ────────────────────
# O Six_Axis_FT.exe traz um Savitzky-Golay (as mensagens "Window size must be
# odd." e "Order must be less than window size." são a assinatura dele) além
# de média e máximo por janela. Isto NÃO substitui o mediana+One-Euro do
# lc_filter, que é o filtro da MALHA de controle: é um segundo caminho, só de
# exibição e exportação, para o número da GUI bater com o do fabricante.
FT_SG_WINDOW_DEFAULT = 11    # ímpar
FT_SG_ORDER_DEFAULT  = 3     # < janela
FT_STATS_WINDOW_DEFAULT = 1000  # amostras de Mean_Num / MAX_Num (~1 s @1 kHz)

# Touch sensor (STM32 → PC plotter → UDP). Porta DIFERENTE da célula, senão
# os fluxos se misturam no mesmo receptor.
TOUCH_SENSOR_UDP_PORT = 8081
# Payload (little-endian, 8 bytes): uint32 seq + float valor. Espelhado no
# plotter standalone (sensors/Touch_sensor).
TOUCH_PAYLOAD_FMT = '<If'
# Broadcast do I_final reemitido pelo TouchSensorSource a cada TOTAL.
TOUCH_UDP_BROADCAST_IP = '192.168.5.255'
# Relay do frame COMPLETO (linhas brutas do STM32) para PCs sem USB direto.
TOUCH_FRAME_UDP_PORT = 8082

# Idade máxima de uma amostra para entrar no par sincronizado (s).
SYNC_MAX_AGE_S = 0.25

# A GUI republica o frame de taxels e cada evento de spike para o
# palpation_logger juntar tudo num único CSV.
# touch_pack_msgs/TouchFrame — taxels + o t_us do STM32. Substituiu o
# /touch_sensor/adc (Int32MultiArray), que descartava o carimbo do firmware.
TOUCH_FRAME_TOPIC = '/touch_sensor/frame'
TOUCH_EVENT_TOPIC = '/touch_sensor/spike_event'  # std_msgs/String: RA|SA|CN_MM|CN_RA|CN_SA
# Grade PADRÃO do sensor em uso. O 5×5 é o que está montado na bancada; o
# logger e a GUI derivam a grade do parâmetro `sensor` do launch, e este é o
# valor quando ninguém passa nada.
TOUCH_ROWS_DEFAULT = 5
TOUCH_COLS_DEFAULT = 5
TOUCH_TAXELS_DEFAULT = TOUCH_ROWS_DEFAULT * TOUCH_COLS_DEFAULT
TOUCH_EVENT_TYPES = ('RA', 'SA', 'CN_MM', 'CN_RA', 'CN_SA')

# ── Orientação do 5×5: o firmware entrega o frame girado 180° ───────────────
# Conferido na bancada em 18/08/2026, taxel a taxel, em DUAS ordens de varredura
# independentes (serpentina e raster; 78 mil frames a 1 kHz, sem lacunas): o
# índice que o firmware emite é o espelho do físico nos DOIS eixos —
#
#     frame_idx = (rows*cols - 1) - fisico_idx      (linha E coluna invertidas)
#
# Isso NUNCA foi embaralhamento: é bijeção perfeita, vizinho físico continua
# vizinho no frame. Só a origem caía no canto oposto. Testadas e descartadas as
# hipóteses de espelho só horizontal, só vertical, transposta e flex serpenteado
# (esta última só caiu com a varredura em raster, que quebra o empate).
#
# A causa está em `select_row()` (sensors/TouchFirmware/main.c), que roda uma
# máscara estática e IGNORA o parâmetro `row` de propósito. Mexer lá mudaria
# qual taxel físico responde por cada índice e invalidaria a calibração, então
# a correção mora aqui, do lado do PC.
#
# ATENÇÃO: vale só para as grades LISTADAS. O 4×4 legado nunca foi
# caracterizado na bancada — sem medida, ele passa intacto.
TOUCH_ROT180_GRIDS = frozenset({(5, 5)})


def taxel_frame_to_physical(vals: list, rows: int, cols: int) -> list:
    """Reordena um frame inteiro do firmware para a numeração FÍSICA.

    Depois disto o índice 0 é o taxel físico 00 e o índice rows*cols-1 é o
    último da última linha. Inverter a lista achatada É a rotação de 180° da
    grade, então a mesma linha serve para qualquer R×C. Grade não
    caracterizada (ou frame de tamanho inesperado) volta sem tocar."""
    if (rows, cols) in TOUCH_ROT180_GRIDS and len(vals) == rows * cols:
        return vals[::-1]
    return vals


def taxel_index_to_physical(idx: int, rows: int, cols: int) -> int:
    """O mesmo para um índice solto — as linhas RA/SA do firmware trazem
    `idx=` na convenção do frame, e o raster de spikes tem de casar com o
    heatmap. Grade não caracterizada volta sem tocar."""
    if (rows, cols) in TOUCH_ROT180_GRIDS:
        return rows * cols - 1 - idx
    return idx

def run_stamp_from_msg_time(stamp) -> str:
    """Identificador <AAAAMMDD_HHMMSS> do run a partir do campo `stamp` da
    PalpationStart.

    O logger e a GUI gravam arquivos DIFERENTES do mesmo run (__samples.csv de
    um lado, __sensors.csv e os CSVs crus do outro). Enquanto cada um chamava
    seu próprio `strftime` no instante em que começava, os nomes não batiam e
    não havia como juntar os dois no disco. Derivar o nome do MESMO carimbo da
    mensagem de início resolve isso sem inventar tópico novo.

    stamp zerado (publisher antigo ou `ros2 topic pub` sem o campo) cai na
    hora local — é o comportamento anterior, e nesse caso os nomes podem
    divergir por um segundo.
    """
    import time as _time
    try:
        sec = int(stamp.sec)
    except (AttributeError, TypeError, ValueError):
        sec = 0
    if sec <= 0:
        sec = int(_time.time())
    return _time.strftime('%Y%m%d_%H%M%S', _time.localtime(sec))


# ── Layout dos dados em disco ─────────────────────────────────────────
# Um diretório por MODO, e dentro dele um por RUN, com todos os arquivos
# daquela amostra juntos:
#
#   sensors/Data/MATRIX_MAP/20260812_143012/{samples,sensors,matrix,adc,
#                                            spikes,cuneiformes}.csv
#                                           {params,summary}.json  plot.png
#
# Os arquivos NÃO repetem o run_id no nome — a pasta o carrega, e
# params.json/summary.json o guardam dentro, então um arquivo copiado para
# fora ainda se identifica. Runs anteriores a este layout continuam soltos
# na raiz com o nome antigo (<ts>__samples.csv); os leitores aceitam os dois.
RUN_MODES = ('SLIDE', 'TOUCH', 'MANUAL', 'MATRIX_MAP')
# Gravação avulsa pelo botão "Record data", fora de qualquer run: não tem
# modo, mas também não pode cair na raiz junto das pastas de modo.
REC_DIR_NAME = 'RECORDING'

RUN_SAMPLES_CSV  = 'samples.csv'
RUN_SENSORS_CSV  = 'sensors.csv'
RUN_MATRIX_CSV   = 'matrix.csv'
RUN_ADC_CSV      = 'adc.csv'
RUN_SPIKES_CSV   = 'spikes.csv'
RUN_CN_CSV       = 'cuneiformes.csv'
RUN_PARAMS_JSON  = 'params.json'
RUN_SUMMARY_JSON = 'summary.json'
RUN_PLOT_PNG     = 'plot.png'


def new_run_id() -> str:
    """Identidade de um run: <AAAAMMDD_HHMMSS> do relógio de PAREDE.

    Nunca do relógio ROS: sob use_sim_time ele vem do Gazebo e recomeça do
    zero a cada launch, o que fazia dois runs de sessões diferentes
    disputarem o mesmo nome de arquivo.
    """
    import time as _time
    return _time.strftime('%Y%m%d_%H%M%S', _time.localtime())


def run_id_from_msg(msg) -> str:
    """run_id da PalpationStart, ou o carimbo derivado de `stamp` quando o
    publisher é antigo e não traz o campo."""
    rid = _safe_component(str(getattr(msg, 'run_id', '') or ''))
    return rid or run_stamp_from_msg_time(getattr(msg, 'stamp', None))


def _safe_component(name: str) -> str:
    """Um componente de caminho a partir de texto que veio de MENSAGEM.

    `ros2 topic pub` pode mandar qualquer string em run_id/mode, e esses
    valores viram nome de diretório — sem filtro, um '../..' escreveria
    fora da pasta de dados.
    """
    keep = [c for c in str(name).strip() if c.isalnum() or c in '_-']
    return ''.join(keep)[:64]


def run_dir(mode: str, run_id: str, *, base: str | None = None,
            create: bool = True) -> str:
    """Diretório do run: <base>/<MODO>/<run_id>, base = RUNS_DIR.

    `mode` vazio (ou desconhecido) cai em REC_DIR_NAME — é o caso da
    gravação avulsa, que não pertence a modo nenhum. `run_id` vazio ou
    ilegível vira um carimbo novo, porque um run sem pasta não é gravável.
    """
    m = _safe_component(mode).upper()
    if m not in RUN_MODES:
        m = REC_DIR_NAME
    rid = _safe_component(run_id) or new_run_id()
    path = os.path.join(base or RUNS_DIR, m, rid)
    if create:
        os.makedirs(path, exist_ok=True)
    return path


# Códigos numéricos das fases no CSV unificado. RETRACT dobrado no HOME.
PHASE_CODES = {
    'IDLE': -1, 'HOME': 0, 'DESCENDING': 1, 'HOLD': 2, 'SLIDING': 3,
    'RETRACT': 0, 'DONE': 4, 'ABORTED': 5, 'FROZEN': 6, 'TRANSIT': 7,
    'CALIBRATING': 8, 'MODULATING': 9,
}
PHASE_NAMES = {-1: 'IDLE', 0: 'HOME', 1: 'DESCENDING', 2: 'HOLD',
               3: 'SLIDING', 4: 'DONE', 5: 'ABORTED', 6: 'FROZEN',
               7: 'TRANSIT', 8: 'CALIBRATING', 9: 'MODULATING'}

# ── MATRIX_MAP — defaults compartilhados GUI ↔ explorer ───────────────
# Safe Z: altura de trânsito acima da ORIGEM (primeiro contato).
MATRIX_SAFE_Z_MM_DEFAULT = 10.0
MATRIX_SAFE_Z_MM_MIN     = 2.0
MATRIX_SAFE_Z_MM_MAX     = 60.0
# Velocidade do trânsito XY no ar. Não há contato durante o trânsito, mas o
# teto é conservador: um erro de Safe Z vira arrasto sobre a peça.
MATRIX_TRANSIT_MMS_DEFAULT = 10.0
MATRIX_TRANSIT_MMS_MIN     = 1.0
MATRIX_TRANSIT_MMS_MAX     = 30.0
# Teto de pontos por matriz — protege contra uma grade absurda (ex.: 50×50)
# gerada por engano no configurador.
MATRIX_MAX_POINTS = 400
# Extensão máxima do plano em cada eixo, a partir da origem (mm).
MATRIX_SPAN_MAX_MM = 200.0

# ── CALIBRAÇÃO DINÂMICA DO ÂNGULO DE ATAQUE — defaults GUI ↔ explorer ─
# A descida padrão supõe o alvo perpendicular à home. Quando não está, a
# ponteira encosta de canto e a célula lê a projeção da força normal. A
# calibração troca a suposição por uma medição: N toques leves em torno do
# ponto de aproximação, ajuste do plano e ataque ao longo da normal.
# A geometria mora em plane_probe.py; a execução, em tactile_explorer.
#
# Toques de sonda. O piso espelha plane_probe.MIN_PROBE_POINTS — três
# pontos definem um plano, e menos que isso não é opção de configuração.
# Com 4+ o ajuste vira mínimos quadrados e o resíduo passa a significar
# alguma coisa; o teto existe porque cada toque custa uma descida completa.
PROBE_ALIGN_POINTS_DEFAULT = 4
PROBE_ALIGN_POINTS_MIN     = 3
PROBE_ALIGN_POINTS_MAX     = 12
# Raio do polígono de sondagem: abaixo do mínimo o braço de alavanca do
# ajuste some no ruído dos toques; acima do máximo a sonda sai da peça.
PROBE_ALIGN_RADIUS_MM_DEFAULT = 15.0
PROBE_ALIGN_RADIUS_MM_MIN     = 5.0
PROBE_ALIGN_RADIUS_MM_MAX     = 60.0
# Setpoint dos toques de sonda. Leve para não marcar a amostra — o que faz
# o plano sair paralelo ao real é a IGUALDADE da penetração nos N pontos,
# não o valor. Nunca excede o setpoint do próprio ensaio (o explorer satura).
PROBE_ALIGN_FORCE_N_DEFAULT = 1.0
# Retração linear ANTES de girar o punho. O punho gira em torno do pulso, e
# a ponta varre um arco de raio ≈ comprimento da ferramenta (67,7 mm com a
# pilha FA7155; eram ~162 mm com a célula axial de 100 kg): sem afastar
# antes, esse arco passa dentro da peça e cisalha a ponteira. Os 20 mm de
# default foram dimensionados para a ferramenta LONGA e ficaram folgados
# para a curta — folga aqui só custa tempo de trânsito, então continuam.
PROBE_ALIGN_RETRACT_MM_DEFAULT = 20.0
PROBE_ALIGN_RETRACT_MM_MIN     = 5.0
PROBE_ALIGN_RETRACT_MM_MAX     = 100.0
# Desvio máximo aceito. Acima disso o problema é de MONTAGEM (calço, fixação)
# e o lugar de corrigir não é o software. O teto DURO não é configurável: a
# 30° o J5 já está longe do útil e a rotação varreria a peça.
PROBE_ALIGN_TILT_MAX_DEG_DEFAULT = 20.0
PROBE_ALIGN_TILT_HARD_MAX_DEG    = 30.0

# ── Carimbo da FERRAMENTA nos arquivos ENSINADOS ──────────────────────
# Home, poses e contato aprendido são ensinados COM uma ferramenta montada, e
# nenhum deles registrava qual. Quando a pilha da célula axial de 100 kg
# (TCP a 162,2 mm) deu lugar à FA7155 de 6 eixos (67,7 mm), todo arquivo em
# ~/.config/touch_pack/ continuou sendo lido em silêncio, com o TCP 94,5 mm
# mais alto para os MESMOS ângulos de junta.
#
# O carimbo não invalida nada sozinho — é diagnóstico. Erra para o lado
# seguro em todos os casos conhecidos (ferramenta mais curta = ponta mais
# LONGE da peça), e o contato aprendido já corrige o deslocamento ao longo da
# aproximação em `_lookup_learned`. O que faltava era o operador SABER que a
# pose que ele está carregando foi ensinada com outra geometria.
#
# Chave fora do `tcp_mm` que o learned_contact.json já usa DENTRO de cada
# entrada (lá é a posição do TCP que dá nome à home, não o comprimento).
TOOL_STAMP_KEY = 'tool_tcp_mm'


def tool_tcp_mm() -> float:
    """Comprimento do TCP de palpação em mm (flange → face da ponteira)."""
    from .kinematics import T_TOUCH_TOOL_ATTACH
    return round(float(T_TOUCH_TOOL_ATTACH[2, 3]) * 1e3, 2)


def tool_stamp() -> dict:
    """Carimbo a mesclar no JSON gravado."""
    return {TOOL_STAMP_KEY: tool_tcp_mm()}


def tool_stamp_mismatch(data, *, what: str) -> str | None:
    """None se o carimbo confere; senão a frase pronta para o log.

    `data` é o JSON já carregado; `what` nomeia o arquivo na mensagem.
    Carimbo AUSENTE também devolve frase: o arquivo é anterior ao carimbo e
    portanto não há como afirmar com que ferramenta foi ensinado.
    """
    agora = tool_tcp_mm()
    if not isinstance(data, dict) or TOOL_STAMP_KEY not in data:
        return (f'{what} não traz carimbo de ferramenta — foi gravado antes '
                f'deste campo existir e pode ter sido ensinado com a pilha da '
                f'célula axial de 100 kg (TCP a 162,2 mm). A ferramenta atual '
                f'tem {agora:.1f} mm: confira a pose antes de descer.')
    try:
        antes = float(data[TOOL_STAMP_KEY])
    except (TypeError, ValueError):
        return (f'{what} tem carimbo de ferramenta ilegível '
                f'({data[TOOL_STAMP_KEY]!r}); a atual tem {agora:.1f} mm.')
    if abs(antes - agora) < 0.05:
        return None
    return (f'{what} foi ensinado com um TCP de {antes:.1f} mm e a ferramenta '
            f'montada tem {agora:.1f} mm ({agora - antes:+.1f} mm). Para os '
            f'MESMOS ângulos de junta a ponta está {abs(agora - antes):.1f} mm '
            f'{"mais alta" if agora < antes else "mais baixa"} que quando '
            f'isto foi ensinado.')


# Arquivos de configuração persistente (~/.config/touch_pack/).
CONFIG_DIR            = os.path.expanduser('~/.config/touch_pack')
HOME_POSE_FILE        = os.path.join(CONFIG_DIR, 'home_pose.json')
ROBOT_CONFIG_FILE     = os.path.join(CONFIG_DIR, 'robot.json')
POSES_FILE            = os.path.join(CONFIG_DIR, 'poses.json')
PALPATION_PARAMS_FILE = os.path.join(CONFIG_DIR, 'palpation_params.json')
# Profundidade do contato aprendida POR HOME.
LEARNED_CONTACT_FILE  = os.path.join(CONFIG_DIR, 'learned_contact.json')

# CSVs gravados em <raiz_do_repo>/sensors/Data. Override: TOUCH_PACK_DATA_DIR.
def _resolve_repo_root() -> str | None:
    """Sobe a partir deste arquivo até achar um diretório com `sensors/` —
    funciona do código-fonte (src/...) e do espaço instalado (install/...).
    None se o pacote estiver instalado fora da árvore do repo."""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        if os.path.isdir(os.path.join(d, 'sensors')):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return None


_REPO_ROOT = _resolve_repo_root()


def _resolve_runs_dir() -> str:
    env = os.environ.get('TOUCH_PACK_DATA_DIR')
    if env:
        return os.path.abspath(os.path.expanduser(env))
    if _REPO_ROOT:
        return os.path.join(_REPO_ROOT, 'sensors', 'Data')
    return os.path.expanduser('~/touch_pack_runs')


RUNS_DIR = _resolve_runs_dir()


# ── MANUAL em DEGRAU (escada de força) ────────────────────────────────
# Teto de patamares por ensaio — barra um passo minúsculo com máximo alto
# (0,01 N até 10 N = 1900 degraus) antes de o braço começar a andar.
STEP_MAX_LEVELS = 200


def staircase_levels(start_n: float, step_n: float, max_n: float,
                     *, cap: int = STEP_MAX_LEVELS) -> list[float]:
    """Patamares do modo DEGRAU: sobe de `start_n` até `max_n` de `step_n` em
    `step_n`, e volta descendo pelos MESMOS patamares.

    O pico entra UMA vez (não se mede duas vezes o mesmo nível seguido), e é
    sempre `max_n` exato: se o passo não fecha certo (0,5 → 2,0 de 0,7 em
    0,7 daria 1,9), o último degrau da subida é encurtado para cravar o
    máximo pedido, senão o ensaio não chegaria à força que o usuário pediu.

    Devolve `[]` quando a escada pedida excede `cap` patamares — nesse caso o
    ensaio deve ser RECUSADO, não truncado (ver o comentário no laço).

    Função pura — testável sem ROS.
    """
    start_n = float(start_n)
    step_n = float(step_n)
    max_n = float(max_n)
    if step_n <= 0.0 or max_n <= start_n:
        return [start_n]
    up = [start_n]
    v = start_n
    while True:
        v = round(v + step_n, 6)
        if v >= max_n - 1e-9:
            break
        up.append(v)
        if len(up) >= cap:
            # Estourou o teto ANTES de chegar ao pico. Truncar aqui seria
            # pior que recusar: o último degrau da subida saltaria do nível
            # truncado direto para max_n — vários newtons de uma vez contra
            # contato rígido. Devolve vazio e quem chamou recusa o ensaio.
            return []
    # Crava o máximo pedido, mesmo com passo que não fecha exato.
    if abs(up[-1] - max_n) > 1e-9:
        up.append(round(max_n, 6))
    # Descida pelos mesmos patamares, sem repetir o pico.
    return up + up[-2::-1]
