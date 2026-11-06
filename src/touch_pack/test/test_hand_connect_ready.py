"""A GUI só pode dizer "conectado" depois que o driver confirma a mão.

Regressão: sem rota até a caixa ECI, o driver fica vivo e travado em
`eci.start()`, e a GUI — que só olhava `proc.poll()` — ficava verde, salvava o
IP e ligava o ECI. Roda o método real sobre um `self` falso; o módulo importa
ROS, então só `HandMixin._post_connect_real_hand` é extraído por AST.
"""
import ast
import pathlib
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

_SRC = (pathlib.Path(__file__).resolve().parents[1]
        / 'touch_pack' / 'gui_hand.py')


def _post_connect():
    tree = ast.parse(_SRC.read_text())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
              and n.name == '_post_connect_real_hand')
    fn.args.args[1].annotation = fn.args.args[2].annotation = None
    fn.args.args[3].annotation = None
    fn.returns = None
    ns = {'time': time, 'HAND_READY_POLL_MS': 250, 'OK': 'ok'}
    exec(compile(ast.Module([fn], []), str(_SRC), 'exec'), ns)
    return ns['_post_connect_real_hand']


def _gui(proc):
    return MagicMock(_hand_proc=proc, _hand_should_be_alive=True,
                     _eci_enabled=False, _eci_prefix='/covvi/hand')


def _proc(alive=True):
    return SimpleNamespace(poll=lambda: None if alive else 1)


def _ready(is_set):
    ev = threading.Event()
    if is_set:
        ev.set()
    return ev


def test_driver_vivo_sem_confirmacao_nao_conecta_e_segue_esperando():
    proc = _proc()
    gui = _gui(proc)
    _post_connect()(gui, proc, _ready(False), time.monotonic() + 10)
    gui.root.after.assert_called_once()
    gui._toggle_eci.assert_not_called()
    gui._save_robot_config.assert_not_called()
    gui._disconnect_real_hand.assert_not_called()


def test_driver_travado_estoura_o_prazo_e_aborta():
    proc = _proc()
    gui = _gui(proc)
    _post_connect()(gui, proc, _ready(False), time.monotonic() - 1)
    gui._disconnect_real_hand.assert_called_once()
    gui._toggle_eci.assert_not_called()
    gui._start_hand_watchdog.assert_not_called()


def test_driver_morto_na_espera_aborta_sem_watchdog():
    proc = _proc(alive=False)
    gui = _gui(proc)
    _post_connect()(gui, proc, _ready(False), time.monotonic() + 10)
    gui._disconnect_real_hand.assert_called_once()
    gui._start_hand_watchdog.assert_not_called()


def test_confirmacao_conecta_e_liga_watchdog():
    proc = _proc()
    gui = _gui(proc)
    _post_connect()(gui, proc, _ready(True), time.monotonic() + 10)
    gui._start_hand_watchdog.assert_called_once()
    gui._save_robot_config.assert_called_once()
    gui._toggle_eci.assert_called_once()
    gui._disconnect_real_hand.assert_not_called()


def test_operador_desconectou_durante_a_espera_nao_faz_nada():
    proc = _proc()
    gui = _gui(proc)
    gui._hand_should_be_alive = False
    _post_connect()(gui, proc, _ready(True), time.monotonic() + 10)
    gui._toggle_eci.assert_not_called()
    gui._disconnect_real_hand.assert_not_called()
    gui.root.after.assert_not_called()
