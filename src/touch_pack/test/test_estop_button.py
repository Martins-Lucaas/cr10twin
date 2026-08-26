"""Testes do rótulo do botão de E-STOP no header da GUI.

O botão é uma CHAVE COM TRAVA: armado ele para o robô, travado ele rearma.
O texto é a única coisa que diz ao operador em qual das duas metades ele
está, então é o texto que se testa — não a cor.
"""
import pytest

tk = pytest.importorskip('tkinter')
pytest.importorskip('rclpy')

from touch_pack.ui_helpers import _hdr_btn, DANGER, WARN


@pytest.fixture(scope='module')
def _root():
    try:
        r = tk.Tk()
    except tk.TclError as exc:            # sem display (CI headless)
        pytest.skip(f'sem display para Tk: {exc}')
    r.withdraw()
    yield r
    r.destroy()


class _FakeGUI:
    """Portador do botão; `_refresh_estop_button` só toca em `_estop_btn`."""

    def __init__(self, parent, latched=False):
        self._estop_btn = _hdr_btn(parent, '■', 'E-STOP', lambda: None,
                                   bg=DANGER, fg='white')
        self._estop_latched = latched

    def _refresh(self):
        from touch_pack.palpation_gui import PalpationGUI
        PalpationGUI._refresh_estop_button(self)

    @property
    def text(self):
        return self._estop_btn.cget('text')


def test_armed_button_keeps_the_estop_name(_root):
    """Regressão: o refresh escrevia só o ícone e apagava o nome — e como ele
    roda no fim do _build_header, o botão nunca exibia 'E-STOP'."""
    gui = _FakeGUI(_root, latched=False)
    gui._refresh()
    assert 'E-STOP' in gui.text
    assert '■' in gui.text


def test_latched_button_becomes_reconnect(_root):
    gui = _FakeGUI(_root, latched=True)
    gui._refresh()
    assert 'RECONECTAR' in gui.text
    assert 'E-STOP' not in gui.text


def test_hover_after_latching_keeps_the_latched_colour(_root):
    """O <Leave> do _hdr_btn repõe a cor GUARDADA no estado do widget. Se o
    refresh não a atualizasse, tirar o mouse do botão travado o devolveria a
    vermelho e esconderia a trava."""
    gui = _FakeGUI(_root, latched=True)
    gui._refresh()
    btn = gui._estop_btn
    btn.event_generate('<Enter>')
    btn.event_generate('<Leave>')
    _root.update()
    assert btn.cget('bg') == WARN
