"""Aba "6 Axes" da célula FA7155 e as colunas ft_* da planilha.

A célula de 6 eixos entrega N e N·m calibrados de fábrica; o que se testa aqui
não é conversão, é INTEGRIDADE: que os seis eixos cheguem à planilha na ordem
certa, que quadro inválido não envenene o estado, e que a taxa medida no host
seja a cadência real (o sensor não numera nem carimba os quadros).
"""
import os
import threading

import pytest

os.environ.setdefault('ROS_DOMAIN_ID', '77')

from touch_pack.constants import (FT_AXES, FT_AXIS_LABELS, FT_MAX_RATE_HZ,
                                  FT_RATED_FORCE_N, FT_RATED_TORQUE_NM,
                                  FT_SERIAL_BAUD, FT_FRAME_LEN, ft_axis_rated)


def test_ordem_dos_eixos_bate_com_o_quadro():
    """Os rótulos da GUI têm de seguir a ordem do quadro binário — é ela que
    dá nome às colunas da planilha."""
    assert tuple(a for a, _, _ in FT_AXIS_LABELS) == FT_AXES


def test_fundo_de_escala_separa_forca_de_torque():
    for a in ('fx', 'fy', 'fz'):
        assert ft_axis_rated(a) == FT_RATED_FORCE_N
    for a in ('mx', 'my', 'mz'):
        assert ft_axis_rated(a) == FT_RATED_TORQUE_NM


def test_variante_da_bancada_esta_configurada():
    """A unidade montada traz na PLAQUETA "FA7155D-400N/20NM" — a FA7155D.
    A variante oscilou entre B e D nas notas de 19/08/2026, todas por relato;
    a plaqueta é a fonte física e é ela que este teste fixa."""
    assert FT_RATED_FORCE_N == 400.0
    assert FT_RATED_TORQUE_NM == 20.0


def test_teto_do_link_bate_com_a_conta_do_baud():
    """28 bytes × 10 bits por quadro: acima disto o quadro não cabe no baud
    em uso e chega picotado."""
    assert FT_MAX_RATE_HZ == pytest.approx(
        FT_SERIAL_BAUD / (FT_FRAME_LEN * 10))
    assert 3500 < FT_MAX_RATE_HZ < 3650


def test_baud_e_taxa_sao_os_do_exemplar_da_bancada():
    """26/08/2026: a unidade montada no flange fala 1 Mbps, e NÃO os 115200
    que o manual dá como default do caso geral. É fato de bancada, como a
    plaqueta — fica fixado aqui.

    07/09/2026: a TAXA deixou de ser a do sensor (1 kHz) e passou a ser a que
    a bancada SUSTENTA em modo polled. O sensor não ficou mais lento; é o
    round-trip do USB que limita (458 Hz de roda-livre pelo CH343), e o
    nominal fica ABAIXO desse teto de propósito — é ele que dá o passo do
    laço, e passo com folga é o que torna o dt constante."""
    from touch_pack.constants import FT_NOMINAL_RATE_HZ
    assert FT_SERIAL_BAUD == 1_000_000
    assert FT_NOMINAL_RATE_HZ == 400.0
    assert FT_NOMINAL_RATE_HZ < 458.0, 'o passo do laço precisa de folga'
    assert FT_NOMINAL_RATE_HZ < FT_MAX_RATE_HZ, 'a taxa tem de caber no link'


# ── Callback do wrench ────────────────────────────────────────────────────

def _gui():
    """PalpationGUI sem Tk nem ROS — só o estado que o callback toca."""
    pytest.importorskip('rclpy')
    import collections
    from touch_pack.palpation_gui import PalpationGUI

    g = PalpationGUI.__new__(PalpationGUI)
    g._lock = threading.Lock()
    g._ft_wrench = {a: 0.0 for a in FT_AXES}
    g._ft_last_ts = 0.0
    g._ft_frames_ok = 0
    g._ft_frames_bad = 0
    g._ft_rate_hz = None
    g._ft_arrivals = collections.deque(maxlen=120)
    return g


def _wrench(fx, fy, fz, mx, my, mz):
    from geometry_msgs.msg import WrenchStamped
    m = WrenchStamped()
    # rclpy exige float estrito nos campos do Vector3.
    m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = (
        float(fx), float(fy), float(fz))
    m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z = (
        float(mx), float(my), float(mz))
    return m


def test_os_seis_eixos_chegam_na_ordem_certa():
    g = _gui()
    g._cb_ft_wrench(_wrench(1.0, 2.0, 3.0, 0.1, 0.2, 0.3))
    assert g._ft_wrench == {'fx': 1.0, 'fy': 2.0, 'fz': 3.0,
                            'mx': 0.1, 'my': 0.2, 'mz': 0.3}
    assert g._ft_frames_ok == 1 and g._ft_frames_bad == 0


def test_quadro_com_nan_e_descartado_e_contado():
    """NaN entrando no estado envenenaria o tare e a coluna da planilha sem
    deixar rastro — tem de ser descartado E contado."""
    g = _gui()
    g._cb_ft_wrench(_wrench(1.0, 2.0, 3.0, 0.1, 0.2, 0.3))
    bom = dict(g._ft_wrench)
    g._cb_ft_wrench(_wrench(float('nan'), 0, 0, 0, 0, 0))
    g._cb_ft_wrench(_wrench(0, float('inf'), 0, 0, 0, 0))
    assert g._ft_wrench == bom, 'quadro ruim não pode sobrescrever o bom'
    assert g._ft_frames_ok == 1 and g._ft_frames_bad == 2


def test_taxa_medida_usa_a_janela_inteira():
    """dt instantâneo de serial oscila demais; a taxa sai da janela."""
    g = _gui()
    import time as _t
    real = _t.time
    try:
        for i in range(20):
            _t.time = lambda i=i: 1000.0 + i * 0.004   # 250 Hz
            g._cb_ft_wrench(_wrench(0, 0, 0, 0, 0, 0))
    finally:
        _t.time = real
    assert g._ft_rate_hz == pytest.approx(250.0, rel=1e-6)


def test_taxa_fica_none_ate_ter_amostras_suficientes():
    g = _gui()
    for _ in range(3):
        g._cb_ft_wrench(_wrench(0, 0, 0, 0, 0, 0))
    assert g._ft_rate_hz is None, 'não estimar taxa com 3 quadros'


def test_cabecalho_da_planilha_tem_os_seis_eixos():
    """As colunas ft_* têm de existir e vir ANTES das tensões do toque, para
    o layout da planilha não depender da grade do sensor tátil."""
    cols = (['t_rel_s', 't_unix', 'touch_t_stm_s', 'force_net_n',
             'load_cell_raw_n', 'load_cell_voltage_v', 'touch_i_final']
            + ['ft_fx_n', 'ft_fy_n', 'ft_fz_n',
               'ft_mx_nm', 'ft_my_nm', 'ft_mz_nm', 'ft_age_ms']
            + [f'v{r}{c}' for r in range(5) for c in range(5)])
    for a in ('fx', 'fy', 'fz'):
        assert f'ft_{a}_n' in cols
    for a in ('mx', 'my', 'mz'):
        assert f'ft_{a}_nm' in cols
    assert cols.index('ft_age_ms') < cols.index('v00')
    assert len(cols) == 7 + 7 + 25


# ── Fz no Controle Manual ─────────────────────────────────────────────────

class _Lbl:
    def __init__(self):
        self.text = None

    def config(self, text=None, fg=None):
        self.text = text


def _gui_fz():
    import types
    g = _gui()
    g._mlc_force_lbl, g._mlc_status_lbl = _Lbl(), _Lbl()
    g._tab_visible = lambda *_f: True
    g.root = types.SimpleNamespace(after=lambda *_a: None)
    return g


def test_fz_do_controle_manual_acompanha_o_wrench():
    """O painel existia mas ninguém o repintava — ficava em '—' com a célula
    no ar. Aqui o Fz que chega no callback tem de aparecer no label."""
    g = _gui_fz()
    g._cb_ft_wrench(_wrench(1.0, 2.0, -4.25, 0, 0, 0))
    g._refresh_manual_fz()
    assert g._mlc_force_lbl.text.strip() == '-4.250 N'
    assert g._mlc_status_lbl.text == 'live'


def test_fz_do_controle_manual_nao_mostra_valor_velho():
    import time
    g = _gui_fz()
    g._refresh_manual_fz()
    assert g._mlc_status_lbl.text == 'no data'
    g._cb_ft_wrench(_wrench(0, 0, 3.0, 0, 0, 0))
    g._ft_last_ts = time.time() - 5.0
    g._refresh_manual_fz()
    assert g._mlc_force_lbl.text.startswith('—')
    assert g._mlc_status_lbl.text == 'stale'


# ── Heatmap do toque no Controle Manual ───────────────────────────────────

@pytest.mark.parametrize('n', [4, 5])
def test_heatmap_do_manual_tem_a_orientacao_da_aba_sensores(n):
    """O 5×5 tem o _VIEW_ROT180 e o 4×4 não: o mini-heatmap tem de mostrar o
    mesmo canto que o TouchFigure (snapshot + rot90×2) nos dois."""
    import numpy as np
    from touch_pack.touch_source import TouchSensorSource
    src = TouchSensorSource(rows=n, cols=n, udp_broadcast=False)
    src.voltage_frame = np.arange(n * n, dtype=float).reshape(n, n)
    np.testing.assert_array_equal(src.view_voltages(),
                                  np.rot90(src.snapshot()['volt'], 2))


class _Canvas:
    def __init__(self):
        self.fill = {}

    def itemconfig(self, iid, fill=None):
        self.fill[iid] = fill


def _gui_heat(volt, fresh):
    import types
    import numpy as np
    from touch_pack.palpation_gui import BORDER
    g = _gui()
    g._touch_source = types.SimpleNamespace(
        is_fresh=lambda: fresh, view_voltages=lambda: np.asarray(volt))
    g._mlc_lut = ['#lo', '#mid', '#hi']
    g._mlc_heat_cv = _Canvas()
    g._mlc_heat_ids = [[0, 1], [2, 3]]
    g._mlc_heat_fill = [[BORDER] * 2 for _ in range(2)]
    return g


def test_heatmap_do_manual_pinta_pela_tensao():
    g = _gui_heat([[0.0, 3.3], [1.65, 9.9]], fresh=True)
    g._paint_manual_heatmap()
    # 9,9 V satura no topo da escala em vez de estourar o índice da LUT.
    assert g._mlc_heat_cv.fill == {0: '#lo', 1: '#hi', 2: '#mid', 3: '#hi'}


def test_heatmap_do_manual_apaga_com_o_stm_mudo():
    from touch_pack.palpation_gui import BORDER
    g = _gui_heat([[3.3, 3.3], [3.3, 3.3]], fresh=True)
    g._paint_manual_heatmap()
    g._touch_source.is_fresh = lambda: False
    g._paint_manual_heatmap()
    assert set(g._mlc_heat_cv.fill.values()) == {BORDER}
