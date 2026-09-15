"""Os trânsitos em ar livre também vigiam força, célula e pausa.

Até 15/09/2026 `_cartesian_batch_to` publicava a trajetória inteira e depois
só dormia olhando o STOP: a checagem de força de quem chamava era
POSTERIOR ao movimento. Um relevo mais alto que a folga de trânsito — no
MATRIX_MAP, exatamente o que a grade existe para descobrir — era percorrido
inteiro contra a peça antes de alguém notar.

E o ⏸ não fazia nada nas duas primitivas batch (HOME, sondagens da
calibração, trânsitos da grade): o movimento ia até o fim e a pausa só era
percebida na fase seguinte. Pausar substitui o goal do JTC, então o que
faltava do percurso morre com ela e precisa ser REEMITIDO.
"""
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')


@pytest.fixture(scope='module')
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def node(_ros):
    from touch_pack.tactile_explorer import TactileExplorer
    n = TactileExplorer()
    n._q_now = lambda: np.deg2rad([0, 0, -90, 0, 90, 0]).astype(float)
    n._stream_q = lambda *a, **k: None
    n._settle = lambda *a, **k: None
    n._speed_factor_pct = 10.0
    n._arm_traj_pub.publish = lambda msg: None
    # Célula viva e descarregada. O carimbo é estático num teste, e o
    # trânsito dura mais que _FORCE_STALE_S de relógio — em operação quem o
    # renova é o callback da célula, a 10–80 Hz. O teste do 'stale' restaura
    # o método real.
    with n._lc_lock:
        n._lc_force_ts = time.monotonic()
        n._lc_force_net = 0.0
    n._fz_corrected = lambda: 0.0
    n._force_stale_abort = lambda fase: False
    yield n
    n.destroy_node()


def _transito(node, dist_m=0.005):
    return node._cartesian_batch_to(
        np.array([0.0, 0.0, 1.0]), dist_m, v_const_ms=0.05, lock_ori=True)


# ── Força durante o percurso ──────────────────────────────────────────

def test_transito_sem_carga_conclui(node):
    assert _transito(node) == 'done'


def test_carga_no_meio_do_transito_para_o_braco(node):
    """O braço não pode terminar a trajetória para só então acusar: o
    `_settle` do caminho de força substitui o goal do JTC e PARA no lugar."""
    from touch_pack.explorer_constants import _FORCE_SAFE_LIMIT_N
    parou = []
    node._settle = lambda *a, **k: parou.append(True)
    node._fz_corrected = lambda: _FORCE_SAFE_LIMIT_N + 1.0
    assert _transito(node) == 'force'
    assert parou, 'não substituiu o goal — o braço seguiu contra a peça'


def test_tracao_tambem_dispara(node):
    """`_force_over_limit` compara MAGNITUDE: a ponteira enganchando e
    sendo puxada é tão anormal quanto ela esmagando."""
    from touch_pack.explorer_constants import _FORCE_SAFE_LIMIT_N
    node._fz_corrected = lambda: -(_FORCE_SAFE_LIMIT_N + 1.0)
    assert _transito(node) == 'force'


def test_celula_muda_aborta_o_transito(node):
    """Sem leitura fresca não há monitor de força — e o trânsito passa a
    ser exatamente o que era antes desta guarda existir."""
    from touch_pack.tactile_explorer import TactileExplorer
    node._force_stale_abort = (
        lambda fase: TactileExplorer._force_stale_abort(node, fase))
    with node._lc_lock:
        node._lc_force_ts = 0.0
    assert _transito(node) == 'stale'


def test_stop_continua_parando(node):
    node._stop_requested.set()
    assert _transito(node) == 'stop'
    assert not node._stop_requested.is_set(), 'o STOP não foi consumido'


# ── Pausa: o resto do percurso tem de ser reemitido ───────────────────

def test_pausa_no_transito_devolve_paused(node):
    """'paused' não é falha: é "o goal se perdeu, replaneje o que faltava"."""
    node._pause_requested.set()
    node._pause_gate = lambda: True          # pausa já resolvida
    assert _transito(node) == 'paused'


def test_move_linear_replaneja_o_que_faltava(node):
    """E o replanejamento mira o ALVO, não o deslocamento original: reemitir
    `delta_m` a partir de onde o braço parou andaria o percurso duas vezes."""
    pedidos = []
    p = [np.array([0.5, 0.0, 0.400])]
    node._tcp_now = lambda: p[0].copy()

    def _fake(u, dist, **kw):
        pedidos.append(dist)
        if len(pedidos) == 1:                # parou na metade
            p[0] = p[0] + u * (dist / 2.0)
            return 'paused'
        p[0] = p[0] + u * dist
        return 'done'

    node._cartesian_batch_to = _fake
    assert node._move_linear_world(
        np.array([0.0, 0.0, 0.010]), 0.05, label='TESTE') == 'done'
    assert len(pedidos) == 2
    assert pedidos[0] == pytest.approx(0.010)
    assert pedidos[1] == pytest.approx(0.005), 'reemitiu o curso inteiro'


def test_pausa_eterna_nao_vira_movimento_eterno(node):
    node._tcp_now = lambda: np.array([0.5, 0.0, 0.4])
    node._cartesian_batch_to = lambda *a, **k: 'paused'
    assert node._move_linear_world(
        np.array([0.0, 0.0, 0.010]), 0.05, label='TESTE') == 'error'


def test_home_replaneja_depois_da_pausa(node):
    """Mesmo buraco no espaço de juntas: o ⏸ no meio de um retorno à HOME
    deixava o braço a meio caminho e a fase seguinte começava de uma pose
    que ninguém escolheu."""
    tentativas = []

    def _once(q_target):
        tentativas.append(q_target)
        return 'paused' if len(tentativas) == 1 else 'done'

    node._joint_batch_once = _once
    assert node._joint_batch_to(np.zeros(6)) is True
    assert len(tentativas) == 2
