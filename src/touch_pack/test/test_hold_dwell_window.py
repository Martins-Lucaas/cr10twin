"""O dwell do HOLD é o que a GUI pede, e é ele que a estatística mede.

Até 09/09/2026 o campo "HOLD — Stable Window" era publicado, gravado no
params.json e IGNORADO: `_hold_stable_s` chegava ao explorer e não era lido
por ninguém, porque as chamadas de `_phase_hold` omitem `dwell_s` e caíam no
`_HOLD_DWELL_S` embutido. O sintoma era mudo — o default da GUI e o do código
eram os mesmos 5,0 s, então os dois valores coincidiam e nada denunciava.

A janela da estatística assentada tinha o problema espelhado: era cravada em
1,0 s, então um dwell de 5 s descartava 80 % da medição.
"""
import inspect

import pytest

pytest.importorskip('rclpy')

from touch_pack.palpation_report import (      # noqa: E402
    _FINAL_WINDOW_FALLBACK_S, compute_summary)


# ── O dwell vem da GUI ─────────────────────────────────────────────────

def test_phase_hold_consulta_o_pedido_da_gui():
    from touch_pack.tactile_explorer import TactileExplorer
    src = inspect.getsource(TactileExplorer._phase_hold)
    assert '_hold_stable_s' in src, (
        'o campo hold_stable_s voltou a não ser lido — o dwell da GUI não '
        'governa nada de novo')


def test_dwell_default_so_vale_sem_pedido():
    """`dwell_s=None` = use a GUI; ausência de pedido = default do código."""
    from touch_pack.tactile_explorer import _HOLD_DWELL_S
    sig = inspect.signature(
        __import__('touch_pack.tactile_explorer', fromlist=['x'])
        .TactileExplorer._phase_hold)
    assert sig.parameters['dwell_s'].default is None, (
        'o default virou um número de novo: aí o pedido da GUI nunca é '
        'consultado')
    assert _HOLD_DWELL_S > 0.0


def test_a_escada_continua_mandando_no_seu_dwell():
    """Quem passa `dwell_s` explícito não é sobrescrito pela GUI.

    A escada mede cada patamar por conta própria (`step_dwell_s`) e chama
    `_phase_hold(dwell_s=0.0)`; se o pedido da GUI vencesse ali, cada degrau
    ganharia um dwell extra que ninguém pediu."""
    import touch_pack.tactile_explorer as te
    src = inspect.getsource(te)
    assert '_phase_hold(dwell_s=0.0)' in src


# ── A janela da estatística segue o dwell ──────────────────────────────

def _hold_sintetico(n=2400, taxa=400.0, ruidoso_ate=1600):
    """HOLD de 6 s: barulhento nos 4 s iniciais, quieto no último 1 s."""
    return [{'t': i / taxa, 'phase': 'HOLD', 'cycle': 1, 'sp': 2.0,
             'force': 2.5 if i < ruidoso_ate else 2.0,
             'tcp': None, 'taxels': None, 'touch': None}
            for i in range(n)]


def _janela(params):
    s = compute_summary(_hold_sintetico(), params)
    return list(s['cycles'].values())[0]['HOLD']


def test_janela_acompanha_o_dwell_pedido():
    h = _janela({'force_n': 2.0, 'hold_stable_s': 5.0})
    assert h['final_window_s'] == 5.0
    # 5 s a 400 Hz, e o trecho barulhento TEM de entrar: é isso que muda.
    assert h['final_window']['n_samples'] == 2001
    assert h['final_window']['std_n'] > 0.1


def test_dwell_curto_estreita_a_janela():
    h = _janela({'force_n': 2.0, 'hold_stable_s': 1.0})
    assert h['final_window_s'] == 1.0
    assert h['final_window']['n_samples'] == 401
    assert h['final_window']['std_n'] == 0.0    # só o trecho quieto


def test_run_antigo_cai_no_1s_historico():
    """Sem o campo no params.json a janela é a de antes — summaries velhos
    continuam comparáveis com os números já publicados."""
    h = _janela({'force_n': 2.0})
    assert h['final_window_s'] == _FINAL_WINDOW_FALLBACK_S == 1.0
    assert h['final_window']['n_samples'] == 401


def test_campo_zerado_tambem_cai_na_reserva():
    """`hold_stable_s = 0` é o "use o default" da mensagem, não uma janela
    de largura zero — que não teria amostra nenhuma."""
    h = _janela({'force_n': 2.0, 'hold_stable_s': 0.0})
    assert h['final_window_s'] == _FINAL_WINDOW_FALLBACK_S


# ── O teto do campo na GUI ─────────────────────────────────────────────

def test_gui_permite_dwell_longo():
    """O teto era 5,0 s — o mesmo valor do default, então não havia como
    pedir uma medição mais longa que a de fábrica."""
    import re
    from pathlib import Path
    import touch_pack.palpation_gui as gui
    src = Path(gui.__file__).read_text()
    trecho = re.search(r"label='HOLD — Stable Window'.*?\)\n", src, re.S)
    assert trecho, 'não achei o campo HOLD — Stable Window'
    vmax = re.search(r'vmax=([\d.]+)', trecho.group(0))
    assert vmax and float(vmax.group(1)) >= 30.0, (
        'o teto do dwell voltou a ser curto demais para uma medição longa')
