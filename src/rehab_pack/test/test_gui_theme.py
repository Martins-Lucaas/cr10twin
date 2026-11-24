"""Tokens do tema (contraste WCAG) e marcas do eixo dos gráficos. Sem Tk."""
import pytest

pytest.importorskip('tkinter')
from rehab_pack import gui_theme as th  # noqa: E402
from rehab_pack.gui_charts import nice_ticks  # noqa: E402


def contrast(a, b):
    def lum(h):
        c = [int(h.lstrip('#')[i:i + 2], 16) / 255 for i in (0, 2, 4)]
        c = [x / 12.92 if x <= 0.03928 else ((x + 0.055) / 1.055) ** 2.4 for x in c]
        return 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2]
    la, lb = lum(a), lum(b)
    return (max(la, lb) + 0.05) / (min(la, lb) + 0.05)


def test_texto_tem_contraste_aa():
    for fg, bg in ((th.TEXT, th.BG), (th.MUTED, th.BG), (th.MUTED, th.PANEL),
                   (th.HEADER_FG, th.HEADER), (th.HEADER_MUTED, th.HEADER),
                   ('#ffffff', th.PRIMARY), ('#ffffff', th.DANGER)):
        assert contrast(fg, bg) >= 4.5, (fg, bg)
    for pill, txt, _i, ban_bg, ban_fg in th.STATE_STYLE.values():
        assert contrast(txt, pill) >= 4.5
        assert contrast(ban_fg, ban_bg) >= 4.5


def test_parar_se_destaca_do_cabecalho():
    assert contrast(th.DANGER, th.HEADER) >= 3.0      # WCAG 1.4.11
    assert contrast(th.LED_ON, th.HEADER) >= 3.0
    assert contrast(th.LED_OFF, th.HEADER) >= 3.0


def test_marcas_do_eixo_redondas_e_dentro_do_intervalo():
    t = nice_ticks(-4.6, 5.4, 4)
    assert t == [-2.5, 0.0, 2.5, 5.0]
    t = nice_ticks(-0.3, 0.3, 5)
    assert all(-0.3 <= v <= 0.3 for v in t) and 0.0 in t
    assert nice_ticks(1.0, 1.0) == [1.0]
