"""Parser do quadro da célula axial (XIAO + HX711) e a conta de força.

O que estes testes travam é o CONTRATO COM O FIRMWARE: o formato da linha e a
escala counts→volts estão escritos em dois lugares que não se compilam juntos
— `sensors/ForceDriver/src/main.cpp` e `touch_pack/constants.py`. Um deles
mudar sozinho é silencioso no fio e catastrófico na força.

Não importa rclpy: o parser é stdlib puro, de propósito.
"""
import math
import pathlib
import re
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from touch_pack.constants import (          # noqa: E402
    LC_FS_VOLTAGE_V, LC_FW_VOLTAGE_SCALE, LC_HX711_AVDD_V, LC_HX711_GAIN,
    LC_NOMINAL_V_PER_N, lc_force_n,
)
from touch_pack.lc_serial import LcLineParser   # noqa: E402

_REPO = pathlib.Path(__file__).resolve().parents[3]
_MAIN_CPP = _REPO / 'sensors' / 'ForceDriver' / 'src' / 'main.cpp'


def _feed(parser, texto: str):
    return parser.feed(texto.encode('ascii'))


def test_reads_a_whole_line():
    p = LcLineParser()
    assert _feed(p, 'F,7,123456,0.0012345\n') == [(7, 123456, 0.0012345)]


def test_a_line_split_across_two_reads_still_closes():
    """O caso NORMAL, não o excepcional: o SO entrega o buffer da USB em
    blocos que não respeitam fronteira de linha, então quase toda leitura
    termina no meio de uma."""
    p = LcLineParser()
    assert _feed(p, 'F,1,100,0.00') == []
    assert _feed(p, '5\nF,2,200,0.006\n') == [(1, 100, 0.005),
                                              (2, 200, 0.006)]


def test_boot_noise_before_the_first_frame_is_not_an_error():
    """O ESP cospe texto de boot e a porta abre no meio de uma linha. Contar
    isso como erro faria o relatório de saúde do nó acusar link sujo a cada
    replug — e aí ninguém mais olha para o relatório."""
    p = LcLineParser()
    assert _feed(p, '345,0.9\nESP-ROM:esp32s3\nF,1,10,0.001\n') == [
        (1, 10, 0.001)]
    assert p.bad_lines == 0


def test_heartbeats_are_counted_not_parsed():
    p = LcLineParser()
    assert _feed(p, '# rate 9.9 Hz\nF,1,10,0.001\n') == [(1, 10, 0.001)]
    assert p.heartbeats == 1
    assert p.bad_lines == 0


@pytest.mark.parametrize('linha', [
    'F,1,10\n',              # campo faltando
    'F,1,10,0.001,9\n',      # campo sobrando
    'F,x,10,0.001\n',        # seq ilegível
    'F,1,10,abc\n',          # tensão ilegível
])
def test_malformed_F_lines_are_counted(linha):
    p = LcLineParser()
    assert _feed(p, linha) == []
    assert p.bad_lines == 1


def test_nan_never_reaches_the_filter():
    """Um NaN envenena o x_prev do One-Euro PARA SEMPRE: toda saída seguinte
    é NaN até o nó reiniciar. Esta é a única porta de entrada, e é aqui que
    ele morre."""
    p = LcLineParser()
    assert _feed(p, 'F,1,10,nan\nF,2,20,inf\nF,3,30,0.002\n') == [
        (3, 30, 0.002)]
    assert p.bad_values == 2


def test_t_us_wraps_like_the_uint32_of_the_mcu():
    """O firmware manda `micros()`, que estoura a cada ~71 min. O parser
    entrega o número como veio; quem trata o wrap é o receiver, com a
    subtração em módulo 2³² — e para isso o valor tem de chegar truncado em
    32 bits, não como o inteiro grande do Python."""
    p = LcLineParser()
    (_seq, t_us, _v), = _feed(p, f'F,1,{2**32 + 5},0.001\n')
    assert t_us == 5


def test_the_buffer_does_not_grow_without_bound():
    """Placa gravada em modo SERIAL_TEST, ou outro programa na mesma tty:
    chegam bytes e nunca um `\\n` do nosso formato."""
    p = LcLineParser()
    _feed(p, 'x' * 20000)
    assert p.dropped_bytes >= 8192


# ── O contrato com o firmware ─────────────────────────────────────────
def test_the_voltage_scale_matches_the_firmware():
    """`COUNTS_TO_V` do main.cpp e `LC_FW_VOLTAGE_SCALE` do constants.py são
    o MESMO número escrito duas vezes, em linguagens diferentes. Divergir
    reescala toda força em silêncio."""
    src = _MAIN_CPP.read_text(encoding='utf-8', errors='replace')
    vref = float(re.search(r'HX_VREF\s*=\s*([0-9.]+)f', src).group(1))
    div = float(re.search(r'COUNTS_TO_V\s*=\s*HX_VREF\s*/\s*([0-9.]+)f',
                          src).group(1))
    gain = int(re.search(r'#define\s+HX_GAIN\s+(\d+)', src).group(1))
    assert vref == pytest.approx(LC_HX711_AVDD_V)
    assert div == pytest.approx(2 ** 24)
    assert gain == LC_HX711_GAIN
    assert vref / div == pytest.approx(LC_FW_VOLTAGE_SCALE, rel=1e-12)


def test_the_line_format_is_the_one_the_firmware_prints():
    """O formato mora num comentário do main.cpp porque é ele que o gera.
    Se alguém mexer no printf sem mexer no comentário, este teste não pega —
    mas se mexerem nos dois, ele obriga a mexer aqui também, que é onde o
    parser está."""
    src = _MAIN_CPP.read_text(encoding='utf-8', errors='replace')
    assert 'F,<seq>,<t_us>,<v_sensor>' in src


# ── A conta de força ──────────────────────────────────────────────────
def test_force_is_zero_at_the_intercept():
    assert lc_force_n(2.7e-5, 8.7e-4, 2.7e-5) == 0.0


def test_compression_is_positive_whatever_the_wiring_polarity():
    """A calibração é feita em compressão, então o slope carrega a polaridade
    da ponte. Com o slope negativo, a MESMA excursão de tensão tem de sair com
    o mesmo sinal de força — senão a parada de 15 N olha para o lado errado."""
    v0 = 1e-5
    subiu = lc_force_n(v0 + 1e-3, +8.7e-4, v0)
    desceu = lc_force_n(v0 - 1e-3, -8.7e-4, v0)
    assert subiu > 0.0 and desceu > 0.0
    assert subiu == pytest.approx(desceu)


def test_no_calibration_gives_no_force():
    """Zero e não um número inventado: sem reta não há força, e o
    force_receiver trata 0,0 como 'nada a publicar'."""
    assert lc_force_n(0.01, 0.0, 0.0) == 0.0


def test_the_measured_slope_agrees_with_the_plate():
    """A calibração de 7 pontos de 2026 mediu 8,7007e-4 V/N. O nominal da
    placa (100 kg, 2 mV/V, AVDD 3,3 V, ganho 128) dá 8,6146e-4. Estar a 1 %
    é o que se espera da tolerância de sensibilidade de uma célula destas —
    e é o que autoriza LC_NOMINAL_V_PER_N a servir de guarda no wizard."""
    medido = 0.0008700679329781172
    assert abs(medido - LC_NOMINAL_V_PER_N) / LC_NOMINAL_V_PER_N < 0.02


def test_full_scale_of_the_adc_is_below_the_cell_full_scale():
    """±12,89 mV de entrada contra ~0,85 V que a célula daria em 100 kg: o
    ADC satura MUITO antes da célula, e é por isso que o receiver descarta
    por |v| > LC_FS_VOLTAGE_V em vez de por força absurda."""
    v_100kg = LC_NOMINAL_V_PER_N * 100.0 * 9.80665
    assert LC_FS_VOLTAGE_V < v_100kg
    assert LC_FS_VOLTAGE_V == pytest.approx(
        0.5 * LC_HX711_AVDD_V / LC_HX711_GAIN)


def test_one_lsb_is_the_documented_quarter_millinewton():
    """O README anuncia ≈0,23 mN por LSB. É quantização pura (o piso real é o
    ruído do HX711), mas é o número que justifica a célula de 100 kg medir
    força de palpação de 1 N."""
    lsb_n = LC_FW_VOLTAGE_SCALE / LC_NOMINAL_V_PER_N
    assert lsb_n == pytest.approx(0.23e-3, rel=0.05)
    assert math.isfinite(lsb_n)
