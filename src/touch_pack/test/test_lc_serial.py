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
    LC_FS_COUNTS, LC_FS_VOLTAGE_V, LC_FW_VOLTAGE_SCALE, LC_HX711_AVDD_V,
    LC_HX711_GAIN, LC_NOMINAL_V_PER_N, hold_tol_n, lc_force_n,
    lc_load_calibration,
)
from touch_pack.lc_serial import LcLineParser   # noqa: E402

_REPO = pathlib.Path(__file__).resolve().parents[3]
_MAIN_CPP = _REPO / 'sensors' / 'ForceDriver' / 'src' / 'main.cpp'


def _feed(parser, texto: str):
    return parser.feed(texto.encode('ascii'))


def test_reads_a_whole_line():
    p = LcLineParser()
    assert _feed(p, 'F,7,123456,0.0012345,6300\n') == [
        (7, 123456, 0.0012345, 6300)]


def test_the_four_field_frame_of_the_old_firmware_still_parses():
    """O 5º campo (counts) entrou em 28/08/2026, e uma placa só ganha o
    firmware novo quando alguém a regrava. Recusar o quadro de 4 campos
    transformaria `pio run -t upload` em pré-requisito para o nó SUBIR — e
    quem descobre isso é quem estava tentando medir, não quem mudou o
    formato. `counts` vem None, que é honesto: não é zero, é ausente."""
    p = LcLineParser()
    assert _feed(p, 'F,7,123456,0.0012345\n') == [
        (7, 123456, 0.0012345, None)]
    assert p.bad_lines == 0


def test_a_line_split_across_two_reads_still_closes():
    """O caso NORMAL, não o excepcional: o SO entrega o buffer da USB em
    blocos que não respeitam fronteira de linha, então quase toda leitura
    termina no meio de uma."""
    p = LcLineParser()
    assert _feed(p, 'F,1,100,0.00') == []
    assert _feed(p, '5\nF,2,200,0.006\n') == [(1, 100, 0.005, None),
                                              (2, 200, 0.006, None)]


def test_boot_noise_before_the_first_frame_is_not_an_error():
    """O ESP cospe texto de boot e a porta abre no meio de uma linha. Contar
    isso como erro faria o relatório de saúde do nó acusar link sujo a cada
    replug — e aí ninguém mais olha para o relatório."""
    p = LcLineParser()
    assert _feed(p, '345,0.9\nESP-ROM:esp32c6\nF,1,10,0.001\n') == [
        (1, 10, 0.001, None)]
    assert p.bad_lines == 0


def test_heartbeats_are_never_samples():
    p = LcLineParser()
    assert _feed(p, '# rate 9.9 Hz\nF,1,10,0.001\n') == [(1, 10, 0.001, None)]
    assert p.heartbeats == 1
    assert p.bad_lines == 0


def test_the_heartbeat_key_values_become_state():
    """O heartbeat do firmware carrega o motivo de NÃO haver amostra, e até
    28/08/2026 o host o contava e jogava fora. `zeroed=0` é o firmware
    esperando um repouso que a bancada não dá — um diagnóstico completamente
    diferente de 'cabo solto', que era o que o nó mandava conferir."""
    p = LcLineParser()
    _feed(p, '# amostras=120 taxa=81.7 offset=-0.002361 zeroed=0 '
             'zero_mv=0.184 resets=2 timeouts=0 conv_us=12143 ensaio=1\n')
    assert p.heartbeat['zeroed'] == 0.0
    assert p.heartbeat['resets'] == 2.0
    assert p.heartbeat['conv_us'] == 12143.0
    assert p.heartbeat['taxa'] == pytest.approx(81.7)


def test_a_heartbeat_in_prose_is_not_an_error():
    """O firmware também manda linhas '#' sem par nenhum (o zero travado, o
    aviso de power-cycle). Elas contam como heartbeat e não sujam nem o
    dicionário nem os contadores de erro."""
    p = LcLineParser()
    _feed(p, '# HX711 travado (0/saturado): power-cycle (#3)\n')
    assert p.heartbeats == 1
    assert p.bad_lines == 0
    assert p.heartbeat == {}


@pytest.mark.parametrize('linha', [
    'F,1,10\n',                # campo faltando
    'F,1,10,0.001,9,9\n',      # campo sobrando (6 nunca foi formato)
    'F,x,10,0.001\n',          # seq ilegível
    'F,1,10,abc\n',            # tensão ilegível
    'F,1,10,0.001,abc\n',      # counts ilegível
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
        (3, 30, 0.002, None)]
    assert p.bad_values == 2


def test_t_us_wraps_like_the_uint32_of_the_mcu():
    """O firmware manda `micros()`, que estoura a cada ~71 min. O parser
    entrega o número como veio; quem trata o wrap é o receiver, com a
    subtração em módulo 2³² — e para isso o valor tem de chegar truncado em
    32 bits, não como o inteiro grande do Python."""
    p = LcLineParser()
    (_seq, t_us, _v, _c), = _feed(p, f'F,1,{2**32 + 5},0.001\n')
    assert t_us == 5


def test_the_buffer_does_not_grow_without_bound():
    """Outro firmware na placa, ou outro programa na mesma tty:
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
    """O formato mora num comentário do main.cpp porque é ele que o gera."""
    src = _MAIN_CPP.read_text(encoding='utf-8', errors='replace')
    assert 'F,<seq>,<t_us>,<v_sensor>,<counts>' in src


def test_the_parser_reads_what_the_firmware_printf_actually_emits():
    """O teste acima confere o COMENTÁRIO; este confere o `printf`.

    A fraqueza que o outro declara ("se alguém mexer no printf sem mexer no
    comentário, este teste não pega") é justamente o modo de falha caro desta
    cadeia: o formato está escrito em dois arquivos que não se compilam
    juntos, e divergir é silencioso no fio. Aqui a string de formato REAL é
    extraída do fonte, preenchida com valores plausíveis, e passada pelo
    parser — se o firmware ganhar ou perder um campo, o parser recusa a linha
    e este teste cai.
    """
    src = _MAIN_CPP.read_text(encoding='utf-8', errors='replace')
    m = re.search(r'Serial\.printf\(\s*"(F,[^"]*)\\n"', src)
    assert m, 'nenhum Serial.printf de quadro F encontrado no main.cpp'
    fmt = m.group(1)
    exemplo = fmt
    for espec, valor in (('%lu', '7'), ('%.7f', '0.0012345'), ('%ld', '6300')):
        while espec in exemplo:
            exemplo = exemplo.replace(espec, valor, 1)
    assert '%' not in exemplo, f'especificador não previsto em {fmt!r}'
    p = LcLineParser()
    amostras = _feed(p, exemplo + '\n')
    assert p.bad_lines == 0, f'o parser recusou o que o firmware imprime: {exemplo!r}'
    assert len(amostras) == 1
    assert amostras[0][:3] == (7, 7, 0.0012345)
    assert amostras[0][3] == 6300, 'o 5º campo (counts) não chegou'


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


# ── A calibração cobre a faixa onde a palpação OPERA? ─────────────────
def _calibracao_do_repo():
    cal = lc_load_calibration(str(_REPO / 'sensors' / 'load_cell_calib.json'))
    assert cal is not None
    return cal


@pytest.mark.xfail(strict=True, reason=(
    'A calibração de 7 pontos em vigor erra 0,250 N (−51 %) no ponto de 50 g. '
    'Não há massas padrão na bancada para refazê-la (28/08/2026), então o '
    'xfail REGISTRA o problema em vez de escondê-lo. Ao recalibrar com '
    'pontos entre 50 g e 2 kg, este teste passa e o strict=True obriga a '
    'remover o marcador — que é como ele avisa que o conserto chegou.'))
def test_the_calibration_error_is_smaller_than_the_band_the_loop_regulates():
    """O critério que importa não é o resíduo ABSOLUTO — é o resíduo contra a
    BANDA com que a malha regula naquele setpoint.

    `hold_tol_n` é a meia-banda do HOLD: max(4σ do ruído, 5 % do alvo). Se a
    calibração erra MAIS que ela, o explorer está perseguindo um setpoint com
    precisão melhor que a exatidão da própria medida — regula ruído de
    calibração e o número no relatório não significa o que promete.

    O `test_force_from_the_repo_line_matches_the_masses` do
    test_force_receiver.py cobre o mesmo dado com um bound ABSOLUTO
    (`pior < 0.30`), e é por isso que ele passa: 0,250 N em 0,490 N são 51 %
    de erro e cabem folgados em 0,30 N. Um bound absoluto num sistema que
    opera de 0,12 N a 10 N não diz nada sobre a ponta de baixo.
    """
    slope, v0, pontos = _calibracao_do_repo()
    piores = []
    for _m, f_real, v in pontos:
        erro = abs(lc_force_n(v, slope, v0) - f_real)
        limite = hold_tol_n(f_real)
        if erro > limite:
            piores.append(f'{f_real:.3f} N: erro {erro:.3f} N > '
                          f'banda {limite:.3f} N')
    assert not piores, 'pontos fora da banda do controle: ' + '; '.join(piores)


def test_the_calibration_is_dominated_by_its_heaviest_points():
    """Não é bug, é a natureza do ajuste — e precisa estar escrito.

    `lc_fit_slope` minimiza o erro ABSOLUTO com V₀ fixo, o que pondera cada
    ponto por F². Os dois pontos mais pesados da reta em vigor decidem 78 %
    dela, e os dois abaixo de 1 N somam 0,3 %: a faixa onde a palpação de
    fato opera (CONTACT_ON_N = 0,12 N a 10 N) não é ajustada, é EXTRAPOLADA.

    Este teste não pede que isso mude — pede que ninguém descubra por acidente.
    Recalibrar com massas concentradas abaixo de 2 kg é o conserto; enquanto
    ele não vem, o número fica registrado aqui.
    """
    _slope, _v0, pontos = _calibracao_do_repo()
    total = math.fsum(f * f for _m, f, _v in pontos)
    pesos = sorted((f * f / total for _m, f, _v in pontos), reverse=True)
    assert pesos[0] + pesos[1] > 0.75, 'os dois maiores já não dominam'
    leves = math.fsum(f * f for _m, f, _v in pontos if f < 1.0) / total
    assert leves < 0.01, f'os pontos abaixo de 1 N já pesam {leves:.1%}' 


def test_the_adc_full_scale_in_counts_mirrors_the_firmware():
    """LC_FS_COUNTS e o HX_FS_COUNTS do main.cpp são o MESMO corte escrito em
    dois arquivos que não se compilam juntos. O receiver aplica o de counts
    quando o quadro traz o 5º campo, e o firmware aplica o dele antes de
    transmitir: divergirem faz um lado aceitar o que o outro descarta."""
    src = _MAIN_CPP.read_text(encoding='utf-8', errors='replace')
    m = re.search(r'#define\s+HX_FS_COUNTS\s+(\d+)', src)
    assert m, 'HX_FS_COUNTS não encontrado no main.cpp'
    assert int(m.group(1)) == LC_FS_COUNTS
    # E o corte em counts é o mesmo corte em volts, senão o quadro de 4 campos
    # (firmware antigo) e o de 5 seriam julgados por réguas diferentes.
    assert LC_FS_COUNTS == pytest.approx(LC_FS_VOLTAGE_V / LC_FW_VOLTAGE_SCALE)
