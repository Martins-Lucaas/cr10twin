"""Orientação da grade do touch sensor 5×5.

Conferido na bancada em 18/08/2026, taxel a taxel, em duas ordens de varredura
independentes (serpentina e raster; 78 mil frames a 1 kHz, zero lacunas): o
firmware emite o frame girado 180° em relação ao sensor físico —

    frame_idx = 24 - fisico_idx

Nunca foi embaralhamento (a bijeção sempre foi perfeita), só a origem no canto
oposto. O firmware fica INALTERADO de propósito: `select_row()` roda uma
máscara estática e ignora o parâmetro `row`, e mexer lá invalidaria a
calibração. A correção mora no lado do PC, e este arquivo a prende.

O requisito, na forma em que foi pedido: o taxel físico 00 tem de cair na
coluna `taxel_0` da planilha, e o físico 44 na `taxel_24`.
"""
import pytest

from touch_pack.constants import (taxel_frame_to_physical,
                                  taxel_index_to_physical)

N = 25


def test_fisico_00_vira_taxel_0_e_fisico_44_vira_taxel_24():
    """O requisito literal, sobre um frame cujo valor denuncia a origem."""
    # Valor = índice do FIRMWARE. O físico 00 é o último que o firmware emite.
    frame = list(range(N))
    fis = taxel_frame_to_physical(frame, 5, 5)
    assert fis[0] == 24, 'taxel_0 tem de ser o físico 00'
    assert fis[24] == 0, 'taxel_24 tem de ser o físico 44'


def test_a_grade_inteira_e_espelhada_nos_dois_eixos():
    fis = taxel_frame_to_physical(list(range(N)), 5, 5)
    for r in range(5):
        for c in range(5):
            esperado = (4 - r) * 5 + (4 - c)
            assert fis[r * 5 + c] == esperado, f'linha {r}, coluna {c}'


def test_continua_bijecao():
    """A reordenação não pode perder nem duplicar taxel."""
    fis = taxel_frame_to_physical(list(range(N)), 5, 5)
    assert sorted(fis) == list(range(N))


def test_indice_solto_dos_spikes_casa_com_o_frame():
    """O raster RA/SA usa `idx=` da linha do firmware; se ele não seguir a
    mesma convenção do heatmap, spike e mapa de calor apontam taxels
    diferentes no MESMO run."""
    fis = taxel_frame_to_physical(list(range(N)), 5, 5)
    for firmware_idx in range(N):
        # fis[p] == firmware_idx  <=>  taxel_index_to_physical(firmware_idx) == p
        assert fis[taxel_index_to_physical(firmware_idx, 5, 5)] == firmware_idx


def test_grade_nao_caracterizada_passa_intacta():
    """O 4×4 legado nunca foi medido na bancada — sem medida, não se gira."""
    assert taxel_frame_to_physical(list(range(16)), 4, 4) == list(range(16))
    assert taxel_index_to_physical(0, 4, 4) == 0


def test_frame_de_tamanho_errado_nao_e_reordenado():
    """Frame curto é descartado pelo chamador; aqui só garantimos que a
    reordenação não inventa dados para um tamanho que não bate com a grade."""
    curto = [1, 2, 3]
    assert taxel_frame_to_physical(curto, 5, 5) == curto


def test_fonte_e_gui_concordam_sobre_a_orientacao():
    """Os dois caminhos (heatmap da fonte e publicador da GUI) precisam
    entregar a MESMA ordem — foi a divergência entre eles que já produziu CSV
    ruim antes (ver test_adc_frame_integrity)."""
    pytest.importorskip('serial')
    pytest.importorskip('rclpy')
    import numpy as np
    from touch_pack.touch_source import TouchSensorSource
    from touch_pack.palpation_gui import PalpationGUI

    linha = 'ADC,' + ','.join(str(1000 + i) for i in range(N)) + ',t=1'

    src = TouchSensorSource(port=None, rows=5, cols=5, has_total=False,
                            udp_broadcast=False)
    src._parse_line(linha, [])

    gui = PalpationGUI.__new__(PalpationGUI)
    gui._touch_taxels = N
    gui._touch_rows = gui._touch_cols = 5
    gui._adc_pub_ok = gui._adc_pub_bad = 0
    gui._adc_bad_warn_t = 0.0
    vals, _ = gui._parse_adc_frame(linha)

    assert np.allclose(src.voltage_frame.flatten(),
                       np.array(vals) * (3.3 / 4095.0))


# ═══════════════════════════════════════════════════════════════════════════
# DIVERGÊNCIA DELIBERADA TELA × PLANILHA (pedida em 18/08/2026)
# ═══════════════════════════════════════════════════════════════════════════
# O visualizador mostra o canto OPOSTO ao que as planilhas gravam. Não é bug:
# foi pedido depois de a medição provar que os dois estavam casados. Estes
# testes prendem a divergência para que ninguém a "conserte" sem querer.

def test_visualizador_diverge_das_planilhas_de_proposito():
    """snapshot() (tela) sai girado 180° em relação a
    latest_voltages_and_time() (sensors.csv). Se este teste falhar, alguém
    religou a tela às planilhas — confira se foi intencional."""
    pytest.importorskip('serial')
    import numpy as np
    from touch_pack.touch_source import TouchSensorSource, _VIEW_ROT180

    if not _VIEW_ROT180:
        pytest.skip('_VIEW_ROT180 desligado: tela e planilha voltaram a casar')

    src = TouchSensorSource(port=None, rows=5, cols=5, has_total=False,
                            udp_broadcast=False)
    src._parse_line(
        'ADC,' + ','.join(str(1000 + i) for i in range(N)) + ',t=1', [])

    planilha, _ = src.latest_voltages_and_time()   # caminho do sensors.csv
    tela = src.snapshot()['volt']                  # caminho do TouchFigure

    assert np.allclose(tela, planilha[::-1, ::-1]), (
        'a tela deve ser o espelho da planilha nos dois eixos')
    assert not np.allclose(tela, planilha), 'as duas não podem coincidir'


def test_planilha_continua_fisica_apesar_da_tela():
    """A inversão é SÓ do desenho: o taxel físico 00 continua caindo em v00."""
    pytest.importorskip('serial')
    from touch_pack.touch_source import TouchSensorSource

    src = TouchSensorSource(port=None, rows=5, cols=5, has_total=False,
                            udp_broadcast=False)
    # Valor = índice do firmware; o físico 00 é o último que o firmware emite.
    src._parse_line('ADC,' + ','.join(str(i) for i in range(N)) + ',t=1', [])

    planilha, _ = src.latest_voltages_and_time()
    # v00 é a posição [0,0] e tem de carregar o valor 24 (o físico 00).
    assert round(planilha[0, 0] * 4095.0 / 3.3) == 24
    assert round(planilha[4, 4] * 4095.0 / 3.3) == 0


def test_heatmap_e_raster_continuam_coerentes_entre_si():
    """A inversão da tela vale para os DOIS artistas: se o heatmap acende a
    célula k, o raster tem de marcar o mesmo k."""
    pytest.importorskip('serial')
    from touch_pack.touch_source import TouchSensorSource, _VIEW_ROT180

    if not _VIEW_ROT180:
        pytest.skip('_VIEW_ROT180 desligado')

    src = TouchSensorSource(port=None, rows=5, cols=5, has_total=False,
                            udp_broadcast=False)
    # Spike no índice 0 do FIRMWARE -> físico 24 -> na tela volta para 0.
    src._parse_line('RA,idx=0,adc=1234,t=1', [])
    snap = src.snapshot()
    marcados = [i for i, lst in enumerate(snap['ra']) if lst]
    assert marcados == [0], f'esperado o taxel 0 na tela, veio {marcados}'
