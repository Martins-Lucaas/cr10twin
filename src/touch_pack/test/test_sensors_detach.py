"""Destacar a aba Sensores: mesmo frame vira janela e volta ao mesmo lugar.

Roda `_toggle_sensors_detach` de verdade num Tk real (o módulo importa ROS,
então o método é extraído por AST). Sem display, pula.
"""
import ast
import pathlib
import tkinter as tk
from tkinter import ttk
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

_SRC = (pathlib.Path(__file__).resolve().parents[1]
        / 'touch_pack' / 'gui_sensors.py')


def _toggle():
    tree = ast.parse(_SRC.read_text())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
              and n.name == '_toggle_sensors_detach')
    fn.returns = None
    ns = {'BTN_NEUTRAL': '', 'TEXT': ''}
    exec(compile(ast.Module([fn], []), str(_SRC), 'exec'), ns)
    return ns['_toggle_sensors_detach']


@pytest.fixture(params=[True, False], ids=['aba-depois', 'ultima-aba'])
def gui(request):
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip('sem display')
    root.geometry('300x200')
    nb = ttk.Notebook(root)
    nb.pack(fill='both', expand=True)
    sensors = tk.Frame(nb)
    tk.Label(sensors, text='vivo').pack()
    nb.add(tk.Frame(nb), text='A')
    nb.add(sensors, text='Sensors')
    if request.param:
        nb.add(tk.Frame(nb), text='3D')
    nb.select(sensors)
    root.update()
    g = SimpleNamespace(root=root, _nb=nb, _sensors_tab_frame=sensors,
                        _sensors_detached_at=None,
                        _sens_detach_btn=MagicMock())
    g.toggle = lambda: (_toggle()(g), root.update())
    g._toggle_sensors_detach = g.toggle
    yield g
    root.destroy()


def test_destaca_e_devolve_na_mesma_posicao_duas_vezes(gui):
    frame, nb = gui._sensors_tab_frame, gui._nb
    n_tabs, pos = len(nb.tabs()), nb.index(frame)
    for _ in range(2):
        gui.toggle()
        assert str(frame) not in nb.tabs()
        assert str(frame.winfo_toplevel()) == str(frame)
        assert frame.winfo_viewable()
        assert frame.winfo_children()      # nada foi reconstruído
        assert gui._sensors_detached_at == pos
        gui.toggle()
        assert nb.index(frame) == pos
        assert nb.tab(frame, 'text') == 'Sensors'
        assert str(nb.select()) == str(frame)
        assert len(nb.tabs()) == n_tabs
        assert gui._sensors_detached_at is None


def test_destacar_nao_cai_na_aba_seguinte(gui):
    gui.toggle()
    assert gui._nb.index('current') == 0


def test_fechar_a_janela_devolve_a_aba(gui):
    gui.toggle()
    frame = gui._sensors_tab_frame
    handler = gui.root.tk.call('wm', 'protocol', frame, 'WM_DELETE_WINDOW')
    gui.root.tk.eval(str(handler))
    gui.root.update()
    assert gui._sensors_detached_at is None
    assert str(frame) in gui._nb.tabs()
