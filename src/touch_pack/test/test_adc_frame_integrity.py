"""Integridade do frame ADC do touch sensor.

Regressão da coleta de 07/08/2026: 19% das linhas de um `__samples.csv` saíram
com só parte dos 25 taxels preenchidos. A causa era o publicador aceitar
qualquer linha 'ADC' não vazia; o logger então completava o resto com colunas
em branco. Um frame truncado NÃO é um frame parcial aproveitável — quando a
serial perde bytes, o que vem depois do buraco é emendado de outro frame, e os
valores a partir do corte ficam em taxels trocados. Medido nessa coleta: erro
contra o último frame bom de ~30 ADC em taxel_0..3 contra >1000 ADC em
taxel_16. A regra é descartar o frame inteiro.
"""
import numpy as np
import pytest

pytest.importorskip('serial')
from touch_pack.touch_source import TouchSensorSource  # noqa: E402

N = 25
FULL = 'ADC,' + ','.join(str(1000 + i) for i in range(N)) + ',t=123456'
# Linha real da coleta: 8 taxels e o 't=' cortado junto com o resto.
TRUNC = 'ADC,3773,4081,3977,1841,3718,3818,4087,4002'


def _src():
    return TouchSensorSource(port=None, rows=5, cols=5, has_total=False,
                             udp_broadcast=False)


class _StubLog:
    """Logger mínimo: os nós são criados por __new__ (sem rclpy.init), então
    get_logger() não existe. Guarda as mensagens para os testes conferirem que
    a perda é REPORTADA, não só descartada."""

    def __init__(self):
        self.msgs = []

    def warn(self, msg):
        self.msgs.append(str(msg))

    warning = warn
    info = warn


def test_frame_completo_e_aceito():
    src = _src()
    src._parse_line(FULL, [])
    assert src.frames_ok == 1 and src.frames_bad == 0
    # O frame sai do firmware girado 180°; a fonte reordena para a numeração
    # física antes de montar a matriz (ver taxel_frame_to_physical).
    esperado = (np.arange(1000, 1000 + N)[::-1].reshape(5, 5)
                * (3.3 / 4095.0))
    assert np.allclose(src.voltage_frame, esperado)


def test_frame_truncado_e_descartado_e_contado():
    src = _src()
    src._parse_line(FULL, [])
    bom = src.voltage_frame.copy()
    src._parse_line(TRUNC, [])
    assert src.frames_bad == 1, 'frame truncado tem de ser contado'
    assert src.frames_ok == 1, 'frame truncado não pode contar como bom'
    # O estado publicado continua sendo o último frame BOM — nunca uma mistura.
    assert np.array_equal(src.voltage_frame, bom)


def test_frame_longo_demais_tambem_e_descartado():
    """Bytes emendados podem produzir uma linha com valores A MAIS."""
    src = _src()
    longa = 'ADC,' + ','.join(str(i) for i in range(N + 4)) + ',t=1'
    src._parse_line(longa, [])
    assert src.frames_bad == 1 and src.frames_ok == 0


def test_publicador_da_gui_rejeita_o_mesmo_que_a_fonte():
    """O publicador ROS (_parse_adc_frame) e a fonte têm de concordar sobre o
    que é um frame válido — foi a divergência entre eles que gerou o CSV ruim.
    """
    rclpy = pytest.importorskip('rclpy')          # noqa: F841
    from touch_pack.palpation_gui import PalpationGUI

    gui = PalpationGUI.__new__(PalpationGUI)      # sem Tk/ROS
    gui._touch_taxels = N
    gui._touch_rows = gui._touch_cols = 5
    gui._adc_pub_ok = gui._adc_pub_bad = 0
    gui._adc_bad_warn_t = 0.0
    stub = _StubLog()
    gui.get_logger = lambda: stub

    # Agora devolve (taxels, t_us): o `t=` da própria linha vai junto para o
    # CSV em vez de ser descartado (ver TouchFrame.msg).
    # Devolve na numeração FÍSICA (taxel 0 = 00), invertida em relação ao
    # que o firmware emite.
    assert gui._parse_adc_frame(FULL) == (
        list(range(1000, 1000 + N))[::-1], 123456)
    assert gui._parse_adc_frame(TRUNC) is None
    # Token corrompido no meio derruba a linha inteira em vez de encurtá-la.
    sujo = FULL.replace(',1012,', ',10x2,')
    assert gui._parse_adc_frame(sujo) is None
    assert gui._adc_pub_ok == 1 and gui._adc_pub_bad == 2
    assert stub.msgs, 'descartar em silêncio foi o bug; tem de avisar'


def test_autodeteccao_prefere_a_cdc_nativa_a_vcp_do_stlink():
    """A placa expõe duas portas. A do ST-Link é ponte UART presa ao baud:
    a 115200 dá 11,5 kB/s e o frame 5×5 já consome ~10,7 kB/s. Pegar a errada
    põe o stream contra o teto do link, então a escolha não pode depender da
    ordem de enumeração."""
    from types import SimpleNamespace as NS
    from unittest import mock
    from touch_pack import touch_source as ts

    # Conversor USB-RS485 da FA7155: é ele que disputa /dev/ttyACM* com o
    # STM32 do toque.
    pico = NS(device='/dev/ttyACM0', vid=0x1A86, pid=0x7523)
    stlink = NS(device='/dev/ttyACM1', vid=0x0483, pid=ts.STLINK_VCP_PID)
    nativa = NS(device='/dev/ttyACM2', vid=0x0483,
                pid=ts.STM32_NATIVE_CDC_PID)

    with mock.patch.object(ts.list_ports, 'comports',
                           lambda: [stlink, nativa, pico]):
        assert ts.detect_serial_port() == '/dev/ttyACM2'
    # Mesmo resultado com a ordem trocada.
    with mock.patch.object(ts.list_ports, 'comports',
                           lambda: [nativa, stlink, pico]):
        assert ts.detect_serial_port() == '/dev/ttyACM2'
    # Sem a nativa, ainda usa o que houver em vez de desistir.
    with mock.patch.object(ts.list_ports, 'comports',
                           lambda: [stlink, pico]):
        assert ts.detect_serial_port() == '/dev/ttyACM1'
    # Só a célula → nada de tátil.
    with mock.patch.object(ts.list_ports, 'comports', lambda: [pico]):
        assert ts.detect_serial_port() is None


def test_logger_nao_completa_frame_curto_com_colunas_vazias():
    pytest.importorskip('rclpy')
    from touch_pack_msgs.msg import TouchFrame
    from touch_pack.palpation_logger import PalpationLogger

    lg = PalpationLogger.__new__(PalpationLogger)
    import threading
    lg._lock = threading.Lock()
    lg._rows = lg._cols = 5
    lg._n_taxels = N
    lg._adc_cols = [''] * N
    lg._adc_ts = 0.0
    lg._adc_t_us = 0
    lg._adc_bad = 0
    stub = _StubLog()
    lg.get_logger = lambda: stub

    bom = TouchFrame()
    bom.taxels = list(range(N))
    bom.t_us = 999
    lg._cb_touch_frame(bom)
    assert lg._adc_cols == [str(i) for i in range(N)]
    assert lg._adc_t_us == 999
    ts_bom = lg._adc_ts

    curto = TouchFrame()
    curto.taxels = [3773, 4081, 3977, 1841, 3718, 3818, 4087, 4002]
    curto.t_us = 1000
    lg._cb_touch_frame(curto)
    assert lg._adc_bad == 1
    assert lg._adc_t_us == 999, 'frame ruim não pode sobrescrever o t_us bom'
    # Colunas intactas: o CSV leva o último frame BOM, não um meio-frame.
    assert lg._adc_cols == [str(i) for i in range(N)]
    assert lg._adc_ts == ts_bom, 'frame ruim não pode rejuvenescer taxel_age_ms'
    assert stub.msgs, 'o run tem de registrar que perdeu frames'


# ── 4×4: linhas DATA viram o frame ADC do 5×5 ─────────────────────────────

def _data_lines(vals, t):
    return [f'DATA,idx={i},adc={v},t={t}\r\n' for i, v in enumerate(vals)]


def test_varredura_data_completa_vira_uma_linha_adc():
    from touch_pack.touch_source import DataFrameAssembler
    asm = DataFrameAssembler(16)
    vals = list(range(100, 116))
    out = [asm.feed(ln) for ln in _data_lines(vals, 777)]
    assert out[:-1] == [None] * 15
    assert out[-1] == 'ADC,' + ','.join(map(str, vals)) + ',t=777'


def test_varredura_data_com_linha_perdida_e_descartada():
    """Linha 7 perdida: o frame não pode sair com o taxel 7 da varredura
    anterior — é descartado, e a varredura seguinte sai inteira."""
    from touch_pack.touch_source import DataFrameAssembler
    asm = DataFrameAssembler(16)
    for ln in _data_lines(range(16), 1):
        asm.feed(ln)
    falha = _data_lines(range(200, 216), 2)
    del falha[7]
    assert [asm.feed(ln) for ln in falha][-1] is None
    assert [asm.feed(ln) for ln in _data_lines(range(300, 316), 3)][-1] \
        is not None


def test_varredura_data_nao_emenda_instantes_diferentes():
    from touch_pack.touch_source import DataFrameAssembler
    asm = DataFrameAssembler(4)
    asm.feed('DATA,idx=0,adc=1,t=10')
    asm.feed('DATA,idx=1,adc=1,t=10')
    asm.feed('DATA,idx=2,adc=1,t=20')   # t trocado no meio
    assert asm.feed('DATA,idx=3,adc=1,t=20') is None


def test_run_do_4x4_publica_frame_e_grava_adc_csv():
    """O run SLIDE/20261021_113235 (sensor:=4) saiu com taxel_* e adc.csv
    vazios: o caminho de gravação só conhecia a linha ADC do 5×5."""
    pytest.importorskip('rclpy')
    import csv
    import io
    from touch_pack.palpation_gui import PalpationGUI
    from touch_pack.touch_source import DataFrameAssembler

    gui = PalpationGUI.__new__(PalpationGUI)
    gui._touch_taxels, gui._touch_rows, gui._touch_cols = 16, 4, 4
    gui._adc_pub_ok = gui._adc_pub_bad = 0
    gui._adc_bad_warn_t = 0.0
    gui.get_logger = lambda: _StubLog()
    gui._data_asm = DataFrameAssembler(16)
    gui._experiment_active = lambda: True
    pub = []
    gui._touch_frame_pub = type('P', (), {'publish': lambda s, m: pub.append(m)})()
    gui._touch_event_pub = type('P', (), {'publish': lambda s, m: None})()
    import threading
    gui._lock = threading.Lock()
    buf = io.StringIO()
    gui._ref_adc_writer = csv.writer(buf)
    gui._ref_spike_writer = gui._ref_cn_writer = csv.writer(io.StringIO())

    gui._on_raw_lines(_data_lines(range(2000, 2016), 5_000_000)
                      + ['TOTAL,t=5000000,Iexc=0,Iinh=0,Ifinal=0\r\n'])
    assert len(pub) == 1
    assert list(pub[0].taxels) == list(range(2000, 2016))
    assert pub[0].t_us == 5_000_000
    row = buf.getvalue().strip().split(',')
    assert row[0] == '5.0' and row[1:] == [str(v) for v in range(2000, 2016)]
