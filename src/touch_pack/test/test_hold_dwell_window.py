"""O dwell do HOLD é o que a GUI pede, e é ele que a estatística mede.

Até 09/09/2026 o campo "HOLD — Stable Window" era publicado, gravado no
params.json e IGNORADO: `_hold_stable_s` chegava ao explorer e não era lido
por ninguém, porque as chamadas de `_phase_hold` omitem `dwell_s` e caíam no
`_HOLD_DWELL_S` embutido. O sintoma era mudo — o default da GUI e o do código
eram os mesmos 5,0 s, então os dois valores coincidiam e nada denunciava.

A janela da estatística assentada tinha o problema espelhado: era cravada em
1,0 s, então um dwell de 5 s descartava 80 % da medição.
"""

import pytest

pytest.importorskip('rclpy')

from touch_pack.palpation_report import (      # noqa: E402
    _FINAL_WINDOW_FALLBACK_S, compute_summary)


# ── O dwell vem da GUI ─────────────────────────────────────────────────

def test_phase_hold_consulta_o_pedido_da_gui(braco):
    braco._hold_stable_s = 12.5
    assert _dwells(braco) == [12.5], (
        'o campo hold_stable_s voltou a não ser lido — o dwell da GUI não '
        'governa nada de novo')


def test_dwell_default_so_vale_sem_pedido(braco):
    """`dwell_s=None` = use a GUI; ausência de pedido = default do código."""
    from touch_pack.tactile_explorer import _HOLD_DWELL_S
    assert _HOLD_DWELL_S > 0.0
    braco._hold_stable_s = None
    assert _dwells(braco) == [_HOLD_DWELL_S]
    assert _dwells(braco, dwell_s=1.25) == [1.25]


def test_a_escada_continua_mandando_no_seu_dwell(braco, monkeypatch):
    """Quem passa `dwell_s` explícito não é sobrescrito pela GUI: a onda
    estabiliza na média sem dwell, e cada degrau cumpre o SEU."""
    from touch_pack.force_wave import _ForceProfile
    braco._hold_stable_s = 12.5
    assert _dwells(braco, dwell_s=0.0) == []
    pedidos = []
    braco._phase_hold = lambda **k: (pedidos.append(k), 'stop')[1]
    braco._phase_hold_modulated(_ForceProfile('SINE', 1.0, 2.0, 1.0, 2))
    assert pedidos == [{'dwell_s': 0.0}]
    janelas = []
    braco._qs_regulate = lambda alvo, *a, **k: (
        janelas.append((alvo, k['stable_s'])), ('ok', alvo))[1]
    braco._approach_dir = None
    assert braco._phase_hold_staircase([1.0, 2.0], 3.0) == 'ok'
    assert [s for _, s in janelas if s >= 1.0] == [3.0, 3.0]
    assert 12.5 not in [s for _, s in janelas]


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

def test_gui_permite_dwell_longo(gui_real):
    """O teto era 5,0 s — o mesmo valor do default, então não havia como
    pedir uma medição mais longa que a de fábrica. Vale no widget e no valor
    que sai no start."""
    import tkinter as tk
    nome = str(gui_real.hold_stable_var)

    def filhos(w):
        for c in w.winfo_children():
            yield c
            yield from filhos(c)
    tetos = [float(c.cget('to')) for c in filhos(gui_real.root)
             if isinstance(c, (tk.Spinbox, tk.Scale))
             and nome in (str(c.cget('textvariable'))
                          if isinstance(c, tk.Spinbox)
                          else str(c.cget('variable')))]
    assert tetos and min(tetos) >= 30.0
    saiu = []
    gui_real._do_palpation_start = saiu.append
    try:
        gui_real._on_palp_mode('TOUCH')
        gui_real.hold_stable_var.set(30.0)
        gui_real._collect_and_start()
    finally:
        del gui_real._do_palpation_start
    assert saiu[0]['hold_stable_s'] == 30.0


def _dwells(braco, **kw):
    """(janela pedida ao regulador, fase) de cada chamada de um HOLD real."""
    vistos = []
    braco._qs_regulate = lambda *a, **k: (
        vistos.append((k['stable_s'], k['phase'])), ('ok', 2.0))[1]
    braco._fz_corrected = lambda: 2.0
    assert braco._phase_hold(**kw) == 'ok'
    return [s for s, f in vistos if f.endswith('DWELL')]
