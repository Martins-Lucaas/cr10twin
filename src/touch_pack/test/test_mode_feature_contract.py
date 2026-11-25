"""O painel que aparece na tela é o que o run executa.

Regressão de 15/09/2026: `_on_palp_mode` MOSTRAVA o painel da escada de
força em MATRIX_MAP (com o comentário certo: a escada por ponto é o mapa de
histerese da peça) e `_on_start` zerava `step_size` com um `!= 'MANUAL'`. O
operador configurava a escada, apertava Start e recebia identações de
patamar único — sem uma linha de aviso, e com toda a implementação da
escada por ponto viva no explorer e inalcançável pela tela.

A causa era três literais independentes para a mesma pergunta ("quais modos
executam este recurso?"): um no painel, um no Start e um no despacho da FSM.
Agora há um conjunto só, em constants.py. Estes testes garantem que ele
continua sendo um só e que ele descreve o que a FSM realmente faz.
"""

import pytest

pytest.importorskip('rclpy')

from touch_pack.constants import (   # noqa: E402
    FMOD_MODES,
    RUN_MODES,
    STAIRCASE_MODES,
)


# ── O conjunto descreve a FSM ─────────────────────────────────────────

def test_os_conjuntos_sao_modos_validos():
    assert set(STAIRCASE_MODES) <= set(RUN_MODES)
    assert set(FMOD_MODES) <= set(RUN_MODES)


def test_a_escada_roda_exatamente_nos_modos_declarados(braco):
    """`_phase_hold_staircase` roda no MANUAL e em cada ponto da matriz — e
    em lugar nenhum mais, mesmo configurada."""
    assert set(STAIRCASE_MODES) == {'MANUAL', 'MATRIX_MAP'}
    for modo in RUN_MODES:
        ordem = _protocolo(braco, modo, escada=True)
        assert ordem, modo
        assert ('_phase_hold_staircase' in ordem) is (
            modo in STAIRCASE_MODES), modo
        if modo in STAIRCASE_MODES:      # e só quando pedida
            assert '_phase_hold_staircase' not in _protocolo(braco, modo)


def test_a_onda_roda_so_nos_modos_declarados(braco):
    for modo in RUN_MODES:
        ordem = _protocolo(braco, modo, onda=True)
        assert ('_phase_hold_modulated' in ordem) is (modo in FMOD_MODES), modo
        if modo in FMOD_MODES:
            assert '_phase_hold_modulated' not in _protocolo(braco, modo)


# ── A GUI decide por eles, nos DOIS pontos ────────────────────────────

@pytest.mark.parametrize('modo', RUN_MODES)
def test_o_painel_e_o_payload_consultam_o_mesmo_conjunto(gui_real, modo):
    """O que o painel MOSTRA é o que o Start ENVIA, em todo modo: a GUI
    montada com escada e onda preenchidas, e o payload capturado."""
    g, saiu = gui_real, []
    g._do_palpation_start = saiu.append
    try:
        g._on_palp_mode(modo)
        g.step_size_var.set(0.5)
        g.step_start_var.set(1.0)
        g.step_max_var.set(2.0)
        g.fmod_shape_var.set('SINE')
        g.fmod_min_var.set(1.0)
        g.fmod_max_var.set(2.0)
        g.fmod_hz_var.set(1.0)
        g._collect_and_start()
    finally:
        del g._do_palpation_start
    assert len(saiu) == 1 and saiu[0]['mode'] == modo
    painel_escada = bool(g._step_group.winfo_manager())
    painel_onda = bool(g._fmod_group.winfo_manager())
    assert painel_escada is (modo in STAIRCASE_MODES)
    assert painel_onda is (modo in FMOD_MODES)
    assert (saiu[0]['step_size_n'] > 0.0) is painel_escada
    assert (saiu[0]['force_mod_shape'] != 'OFF') is painel_onda


def test_o_start_aceita_toda_fase_encerrada(gui_real):
    """FROZEN é fase ENCERRADA (o E-STOP congelou e a thread do protocolo
    morreu). Com o literal ('IDLE', 'DONE', 'ABORTED') o Start recusava por
    "já rodando" e o Stop não desfazia, e a palpação ficava travada até
    reiniciar o nó. Fase em andamento continua recusada."""
    from touch_pack.constants import PHASE_CODES
    from touch_pack import palpation_gui as gui
    assert 'FROZEN' in gui._PHASE_ENDED
    g, foi = gui_real, []
    g._collect_and_start = lambda: foi.append(1)
    try:
        for fase in PHASE_CODES:
            foi.clear()
            g._starting_palpation = False
            g._start_published_t = 0.0
            g._estop_latched = False
            g._palpation_blocked = False
            with g._lock:
                g._latest_phase = fase
            g._on_start()
            assert bool(foi) is (fase in gui._PHASE_ENDED), fase
    finally:
        del g._collect_and_start
        g._starting_palpation = False


# ── O aviso de recurso descartado ─────────────────────────────────────

def test_recurso_pedido_fora_do_modo_avisa_antes_de_mover(braco):
    """A GUI zera os campos fora dos modos que os rodam, mas ela não é o
    único publisher: por `ros2 topic pub` uma escada ou uma onda chegavam e
    eram descartadas em silêncio. O aviso sai ANTES de mover o braço."""
    for modo in RUN_MODES:
        _protocolo(braco, modo, escada=True, onda=True)
        linhas = braco.log.linhas
        move = next(i for i, (n, _) in enumerate(linhas) if n == 'fase')
        antes = ' '.join(t for n, t in linhas[:move] if n == 'warn')
        assert ('[DEGRAU]' in antes) is (modo not in STAIRCASE_MODES), modo
        assert ('[FMOD]' in antes) is (modo not in FMOD_MODES), modo


def _protocolo(braco, modo, *, escada=False, onda=False):
    """Roda `_run_protocol` de verdade com as fases trocadas por registros.
    Devolve a ordem das fases; o log fica em `braco.log`."""
    import numpy as np
    from rclpy.parameter import Parameter
    n, ordem = braco, []

    def fase(nome, ret):
        return lambda *a, **k: (ordem.append(nome), n.log.linhas.append(
            ('fase', nome)), ret)[2]
    n.log.linhas.clear()
    for nome, ret in (
            ('_phase_goto_home', True), ('_calibrate_or_abort', True),
            ('_reapply_attack_orientation', True),
            ('_phase_descending', 'ok'), ('_phase_hold', 'ok'),
            ('_phase_hold_staircase', 'ok'), ('_phase_hold_dynamic', 'ok'),
            ('_phase_hold_modulated', 'ok'), ('_phase_sliding', 'ok'),
            ('_finalize_interrupt', True), ('_abort_to_home', None),
            ('_abort_in_place', None), ('_matrix_bail', None),
            ('_lift_to_safe_z', 'done'), ('_move_to_wp_safe', 'done'),
            ('_publish_matrix_point', None), ('_retract', True)):
        if hasattr(n, nome):
            setattr(n, nome, fase(nome, ret))
    n._fz_corrected = lambda: 1.0
    with n._params_lock:
        n._mode, n._repeats, n._target_force_n = modo, 1, 2.0
        n._slide_returns = 0
        n._step_start_n, n._step_max_n = 1.0, 2.0
        n._step_size_n, n._step_dwell_s = (0.5 if escada else 0.0), 1.0
        n._matrix_wps = np.array([[0.01, 0.0]])
    n.set_parameters([
        Parameter('force_mod_shape', value='SINE' if onda else 'OFF'),
        Parameter('force_mod_min_n', value=1.0),
        Parameter('force_mod_max_n', value=2.0),
        Parameter('force_mod_hz', value=1.0)])
    n._run_protocol()
    return ordem
