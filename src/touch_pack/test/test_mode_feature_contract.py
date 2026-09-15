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
import inspect

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


def test_a_escada_roda_exatamente_nos_modos_declarados():
    """`_phase_hold_staircase` é despachada no caminho do MANUAL e no laço
    da matriz — e em lugar nenhum mais."""
    from touch_pack import tactile_explorer as te
    assert set(STAIRCASE_MODES) == {'MANUAL', 'MATRIX_MAP'}
    manual = inspect.getsource(te.TactileExplorer._run_protocol)
    matriz = inspect.getsource(te.TactileExplorer._run_matrix_protocol)
    assert '_phase_hold_staircase' in manual
    assert '_phase_hold_staircase' in matriz


def test_a_onda_roda_so_nos_modos_declarados():
    """O despacho do HOLD modulado consulta FMOD_MODES, não um literal."""
    from touch_pack import tactile_explorer as te
    src = inspect.getsource(te.TactileExplorer._run_protocol)
    assert 'FMOD_MODES' in src, 'o despacho da onda voltou a usar literal'
    assert "== 'TOUCH' else None" not in src


# ── A GUI decide por eles, nos DOIS pontos ────────────────────────────

def _fonte(metodo):
    from touch_pack import palpation_gui as gui
    return inspect.getsource(getattr(gui.PalpationGUI, metodo))


def _codigo(metodo):
    """`_fonte` sem linhas de comentário — os literais proibidos aparecem de
    propósito nos comentários que explicam por que saíram."""
    return '\n'.join(linha for linha in _fonte(metodo).splitlines()
                     if not linha.lstrip().startswith('#'))


def test_o_painel_e_o_payload_consultam_o_mesmo_conjunto():
    """Teste de fonte porque `_on_start` exige ROS, Tk e um braço: o que
    importa é que nenhum dos dois volte a carregar o seu próprio literal."""
    painel = _fonte('_on_palp_mode')
    start = _fonte('_on_start')
    for nome in ('STAIRCASE_MODES', 'FMOD_MODES'):
        assert nome in painel, f'{nome} sumiu de _on_palp_mode'
        assert nome in start, f'{nome} sumiu de _on_start'
    # Os literais que causaram a divergência não podem voltar.
    codigo = _codigo('_on_start')
    assert "!= 'MANUAL'" not in codigo
    assert "!= 'TOUCH'" not in codigo


def test_o_start_aceita_toda_fase_encerrada():
    """FROZEN é fase ENCERRADA (o E-STOP congelou e a thread do protocolo
    morreu). Com o literal ('IDLE', 'DONE', 'ABORTED') o Start recusava por
    "já rodando" e o Stop não desfazia — o explorer não está mais busy — e
    a palpação ficava travada até reiniciar o nó."""
    from touch_pack import palpation_gui as gui
    assert 'FROZEN' in gui._PHASE_ENDED
    src = _fonte('_on_start')
    assert '_PHASE_ENDED' in src
    assert "('IDLE', 'DONE', 'ABORTED')" not in _codigo('_on_start')


# ── O aviso de recurso descartado ─────────────────────────────────────

def test_recurso_pedido_fora_do_modo_avisa_antes_de_mover():
    """A GUI zera os campos fora dos modos que os rodam, mas ela não é o
    único publisher: por `ros2 topic pub` uma escada ou uma onda chegavam e
    eram descartadas em silêncio."""
    from touch_pack import tactile_explorer as te
    src = inspect.getsource(te.TactileExplorer._run_protocol)
    i_aviso = src.find('not in STAIRCASE_MODES')
    i_home = src.find('_phase_goto_home')
    assert i_aviso != -1, 'o aviso de escada descartada sumiu'
    assert 'not in FMOD_MODES' in src, 'o aviso de onda descartada sumiu'
    assert i_aviso < i_home, 'o aviso tem de sair ANTES de mover o braço'
