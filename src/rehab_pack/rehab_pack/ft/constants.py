"""Constantes da FA7155 usadas pelo rehab_pack."""

FT_SERIAL_BAUD  = 1_000_000   # exemplar da bancada (manual §4.3 dá 115200 como
                              # default do caso geral)
FT_FRAME_HEADER = b'\x53\x54'
FT_FRAME_LEN    = 28          # 2 (cabeçalho) + 6×float32 + 2 (CRC-16/MODBUS)
# Ordem dos seis canais dentro do quadro — é ela que dá nome às colunas.
FT_AXES = ('fx', 'fy', 'fz', 'mx', 'my', 'mz')

# Taxa ENTREGUE ao host. O datasheet dá 250 Hz de fábrica e 500-1000 Hz
# configuráveis (a unidade da bancada está em FT_SENSOR_RATE_HZ), mas o
# exemplar só responde a poll, e cada transação paga o round-trip USB do CH343.
# Medido: 397-399 Hz nos 12 runs de sensors/Data/{glass,silicon} (09/2026) e
# 391 Hz ao vivo em 21/10/2026. É também a taxa assumida pelo filtro, pelo
# explorer e pelo atraso do ILC até chegar a primeira medida (/load_cell/rate_hz).
FT_NOMINAL_RATE_HZ = 400.0
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


# Abaixo disto o receiver avisa: cabo ruim, baud errado ou taxa de fábrica
# diferente da configurada.
FT_MIN_RATE_HZ = 100.0

FT_RATED_FORCE_N   = 400.0
FT_RATED_TORQUE_NM = 20.0

# VIDs dos chips de ponte USB-serial usados em conversores RS485. O FA7155 não
# aparece na USB — quem aparece é o conversor. Também EXCLUÍDOS do auto-detect
# do touch sensor (ver touch_source.detect_serial_port).
FT_USB_VIDS = (
    0x1A86,   # WCH — CH340/CH341/CH343 (o ZK-U485 azul da bancada)
    0x10C4,   # Silicon Labs — CP2102/CP2104
    0x0403,   # FTDI — FT232
    0x067B,   # Prolific — PL2303
)


FT_FORCE_AXIS_DEFAULT = 'z'
FT_FORCE_SIGN_DEFAULT = -1.0

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

# Node ID do escravo. 1 é o default Modbus e o que o cliente de fábrica traz
# pré-preenchido; se você mudar com Send_ModBus_ID, mude aqui também.
FT_MODBUS_SLAVE_ID = 1
# Prazo de uma transação. Generoso de propósito: a resposta do comando chega
# no MEIO do stream de 1 kHz e o cliente precisa peneirar os quadros "ST"
# antes de achá-la (ver ft_modbus.find_response).
FT_MODBUS_TIMEOUT_S = 0.5
