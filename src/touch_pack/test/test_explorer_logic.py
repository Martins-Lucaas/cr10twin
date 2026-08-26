"""Testes da lógica do tactile_explorer — HOLD com estabilização do
setpoint, staleness da célula de carga, ciclos de repetição e pausa.
"""
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from std_msgs.msg import Float32, Bool          # noqa: E402
from touch_pack_msgs.msg import PalpationStart  # noqa: E402


@pytest.fixture(scope='module')
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def node(_ros):
    from touch_pack.tactile_explorer import TactileExplorer
    n = TactileExplorer()
    # Stubs: sem simulador nem robô — congela a cadeia de movimento.
    n._q_now = lambda: np.deg2rad([0, 0, -90, 0, 90, 0]).astype(float)
    n._stream_q = lambda *a, **k: None
    n._speed_factor_pct = 10.0
    with n._params_lock:
        n._target_force_n = 2.0
        n._target_depth_mm = 10.0
        n._kp, n._ki, n._kd = 0.001, 0.0005, 0.0
    yield n
    n.destroy_node()


def _feed_force(n, value: float):
    n._cb_lc_force_net(Float32(data=float(value)))


def _keep_force_fresh(n, value: float, duration_s: float) -> threading.Thread:
    """Renova a leitura a ~50 Hz, como a ESP32 faria. Alimenta uma vez
    de forma síncrona antes da thread, evitando a race em que a fase
    começa sem nenhuma leitura (abortaria por 'stale')."""
    _feed_force(n, value)
    def _run():
        t_end = time.time() + duration_s
        while time.time() < t_end:
            _feed_force(n, value)
            time.sleep(0.02)
    th = threading.Thread(target=_run, daemon=True)
    th.start()
    return th


def _default_start(**overrides) -> PalpationStart:
    msg = PalpationStart()
    msg.speed_mms = 10.0
    msg.depth_mm = 10.0
    msg.force_n = 2.0
    msg.slide_dist_mm = 50.0
    msg.approach_speed_mms = 50.0
    msg.slide_dir = '+Y'
    msg.repeats = 1
    msg.speed_factor_pct = 10.0
    msg.home_deg = [0.0, 0.0, -90.0, 0.0, 90.0, 0.0]
    for k, v in overrides.items():
        setattr(msg, k, v)
    return msg


# ── HOLD: estabilização do setpoint ────────────────────────────────────

def test_hold_exits_when_force_stable(node):
    # dwell_s=0: aqui testa-se o critério de estabilização, não a medição.
    th = _keep_force_fresh(node, 2.0, 3.0)
    t0 = time.time()
    out = node._phase_hold(stable_s=0.3, timeout_s=5.0, dwell_s=0.0)
    dt = time.time() - t0              # medir ANTES do join (thread dura 3 s)
    th.join()
    assert out == 'ok'
    assert dt < 1.5                    # saiu pela estabilização, não timeout


def test_hold_times_out_when_out_of_band(node):
    th = _keep_force_fresh(node, 0.5, 2.0)   # fora da banda de 2.0±0.15 N
    t0 = time.time()
    out = node._phase_hold(stable_s=0.3, timeout_s=0.8)
    dt = time.time() - t0
    th.join()
    assert out == 'ok'                 # timeout prossegue com aviso
    assert 0.7 < dt < 1.6


def test_hold_aborts_on_excess_force(node):
    th = _keep_force_fresh(node, 16.0, 1.0)
    out = node._phase_hold(stable_s=0.3, timeout_s=5.0)
    th.join()
    assert out == 'force'


# ── Staleness da célula de carga ───────────────────────────────────────

def test_descending_aborts_without_force_data(node):
    # nenhuma leitura jamais recebida
    out = node._phase_descending()
    assert out == 'stale'


def test_hold_aborts_when_force_freezes(node):
    _feed_force(node, 1.0)             # uma única leitura, depois congela
    t0 = time.time()
    out = node._phase_hold(stable_s=2.0, timeout_s=10.0)
    assert out == 'stale'
    assert time.time() - t0 < 1.5      # abortou em ~_FORCE_STALE_S


# ── Ciclos de repetição ────────────────────────────────────────────────

def _stub_protocol(node, calls, sliding_out='ok'):
    node._phase_goto_home  = lambda: calls.append('home') or True
    node._phase_descending = lambda: calls.append('desc') or 'ok'
    node._phase_hold       = lambda *a, **k: calls.append('hold') or 'ok'
    node._phase_sliding    = (lambda: calls.append('slide')
                              or (sliding_out() if callable(sliding_out)
                                  else sliding_out))
    node._phase_retract    = lambda: calls.append('retract') or True
    node._set_phase        = lambda p: calls.append(f'phase:{p}')
    # O stub tem de MARCAR A FASE como o original faz: desde bd1de90 ("Regra
    # de Ouro": STOP volta à HOME) os caminhos de interrupção passaram de
    # `_set_phase('ABORTED')` direto para `_finalize_interrupt` →
    # `_retreat_and_home`, que é quem marca a fase.
    node._retreat_and_home = (lambda fp: (calls.append(f'retreat→{fp}'),
                                          node._set_phase(fp)) and None)


def test_protocol_runs_n_cycles(node):
    calls: list = []
    _stub_protocol(node, calls)
    node._cb_start(_default_start(repeats=3))
    node._protocol_thread.join(timeout=10)
    assert calls.count('desc') == 3
    assert calls.count('slide') == 3
    assert calls.count('retract') == 0          # RETRACT removido do protocolo
    assert calls.count('home') == 5             # 3 inícios + 2 entre ciclos
    assert 'retreat→DONE' in calls
    assert node._cycle == 0 and node._cycles_total == 1   # reset no finally


def test_protocol_stop_mid_cycle_aborts_rest(node):
    calls: list = []

    def sliding():
        return 'ok' if calls.count('slide') < 2 else 'stop'
    _stub_protocol(node, calls, sliding_out=sliding)
    node._cb_start(_default_start(repeats=5))
    node._protocol_thread.join(timeout=10)
    assert calls.count('slide') == 2
    assert 'phase:ABORTED' in calls
    assert 'retreat→DONE' not in calls


def test_start_clamps_repeats(node):
    calls: list = []
    _stub_protocol(node, calls)
    node._cb_start(_default_start(repeats=999))
    node._protocol_thread.join(timeout=10)
    with node._params_lock:
        assert node._repeats == 100


# ── Pausa ──────────────────────────────────────────────────────────────

class _ForceKeeper:
    """Mantém a leitura da célula FRESCA em background.

    `_pause_gate` roda as mesmas guardas de segurança das outras fases,
    incluindo a de célula muda — segurar posição não é motivo para ficar cego
    (a amostra pode ceder, alguém pode empurrar a ferramenta, e a pausa não
    tem duração máxima). Sem alimentar leitura, o gate aborta por 'stale'
    antes de chegar a bloquear, e o teste mediria a guarda em vez da pausa.
    """

    def __init__(self, n, value: float = 1.0, period_s: float = 0.05):
        self._n, self._v, self._p = n, value, period_s
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self._stop.is_set():
            _feed_force(self._n, self._v)
            self._stop.wait(self._p)

    def __enter__(self):
        _feed_force(self._n, self._v)   # já fresca antes de qualquer leitura
        self._th.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._th.join(timeout=1.0)


def test_pause_gate_blocks_until_resume(node):
    node._busy.set()
    with _ForceKeeper(node):
        node._cb_pause(Bool(data=True))
        result: list = []
        th = threading.Thread(
            target=lambda: result.append(node._pause_gate()), daemon=True)
        th.start()
        time.sleep(0.3)
        assert not result                  # ainda bloqueado segurando posição
        node._cb_pause(Bool(data=False))   # retoma
        th.join(timeout=2.0)
        assert result == [True]
    node._busy.clear()


def test_pause_gate_stop_wins(node):
    node._busy.set()
    with _ForceKeeper(node):
        node._cb_pause(Bool(data=True))
        result: list = []
        th = threading.Thread(
            target=lambda: result.append(node._pause_gate()), daemon=True)
        th.start()
        time.sleep(0.2)
        assert not result                  # bloqueado, não abortado por stale
        node._stop_requested.set()
        th.join(timeout=2.0)
        assert result == [False]           # stop durante pausa → abortar
    node._pause_requested.clear()
    node._busy.clear()


def test_pause_gate_aborta_com_celula_muda(node):
    """A guarda que os dois testes acima neutralizam, testada de propósito:
    sem leitura fresca o gate NÃO bloqueia — ele aborta."""
    node._busy.set()
    node._cb_pause(Bool(data=True))
    with node._lc_lock:
        node._lc_force_ts = 0.0            # nunca chegou leitura
    assert node._pause_gate() is False
    node._pause_requested.clear()
    node._busy.clear()
