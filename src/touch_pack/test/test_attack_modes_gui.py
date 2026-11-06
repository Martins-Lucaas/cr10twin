"""Probe surface e Manual attack angle são modos separados e excludentes.

O manual morava dentro do grupo do probe: só valia com o probe ligado e usava
a força e o recuo dele, escondidos. Os métodos reais são extraídos por AST
porque `palpation_gui` exige rclpy; o Tk é real, e sem display o teste pula.
"""
import ast
import pathlib
import tkinter as tk
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

_SRC = (pathlib.Path(__file__).resolve().parents[1]
        / 'touch_pack' / 'palpation_gui.py')


def _extract(name):
    tree = ast.parse(_SRC.read_text())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
              and n.name == name)
    fn.returns = None
    for arg in fn.args.args:
        arg.annotation = None
    ns = {}
    exec(compile(ast.Module([fn], []), str(_SRC), 'exec'), ns)
    return ns[name]


@pytest.mark.parametrize('saved, esperado', [
    ({}, (False, False)),
    ({'probe_align_on': True}, (True, False)),
    # Formato antigo: o manual só valia com o probe ligado.
    ({'probe_align_on': True, 'probe_align_manual': True}, (False, True)),
    ({'probe_align_on': False, 'probe_align_manual': True}, (False, False)),
    # Formato novo: o manual vale sozinho.
    ({'probe_manual_on': True}, (False, True)),
    ({'probe_align_on': True, 'probe_manual_on': False,
      'probe_align_manual': True}, (True, False)),
])
def test_preferencias_salvas(saved, esperado):
    assert _extract('_saved_attack_modes')(saved) == esperado


@pytest.fixture
def gui():
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip('sem display')
    adv = tk.Frame(root)
    adv.pack()
    align_chk = tk.Label(adv, text='probe')
    align_chk.pack()
    manual_chk = tk.Label(adv, text='manual')
    manual_chk.pack()
    g = SimpleNamespace(
        align_on_var=tk.BooleanVar(root), align_manual_var=tk.BooleanVar(root),
        _align_chk=align_chk, _manual_chk=manual_chk,
        _align_group=tk.Frame(adv), _manual_group=tk.Frame(adv),
        _redraw_matrix_preview=MagicMock(), adv=adv)
    toggle = _extract('_on_align_toggle')
    g.click = lambda var, other: (var.set(not var.get()), toggle(g, other))
    yield g
    root.destroy()


def _ordem(g):
    return [{g._align_chk: 'probe', g._manual_chk: 'manual',
             g._align_group: 'grupo-probe',
             g._manual_group: 'grupo-manual'}[w] for w in g.adv.pack_slaves()]


def test_ligar_um_desliga_o_outro_nos_dois_sentidos(gui):
    gui.click(gui.align_on_var, gui.align_manual_var)
    assert _ordem(gui) == ['probe', 'grupo-probe', 'manual']
    gui.click(gui.align_manual_var, gui.align_on_var)
    assert not gui.align_on_var.get() and gui.align_manual_var.get()
    assert _ordem(gui) == ['probe', 'manual', 'grupo-manual']
    gui.click(gui.align_on_var, gui.align_manual_var)
    assert gui.align_on_var.get() and not gui.align_manual_var.get()
    assert _ordem(gui) == ['probe', 'grupo-probe', 'manual']


def test_manual_funciona_com_o_probe_desligado_e_desliga_limpo(gui):
    gui.click(gui.align_manual_var, gui.align_on_var)
    assert gui.align_manual_var.get()
    gui.click(gui.align_manual_var, gui.align_on_var)
    assert not gui.align_manual_var.get() and not gui.align_on_var.get()
    assert _ordem(gui) == ['probe', 'manual']
