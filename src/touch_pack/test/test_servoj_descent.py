"""DESCENDING em streaming (ServoJ): pico = v·T_halt·K, e a velocidade sai dele.

Contexto (19/08/2026): o DESCENDING deixou de usar MovL em todos os modos. A
troca não é de protocolo, é da FORMA do curso comprometido:

  MovL    pico = 30 µm × K       piso GEOMÉTRICO, velocidade não o move
  ServoJ  pico = v · T_halt · K  v é software

É isso que torna "desacelerar perto do contato" uma defesa real, e não era em
MovL (16,9 N medidos a 1 % de SpeedFactor).
"""
import os

os.environ.setdefault('ROS_DOMAIN_ID', '77')

import pytest

from touch_pack.tactile_explorer import (
    TactileExplorer as T, crawl_v_ms, _DESCEND_CRAWL_V_MIN_MS,
    _DESCEND_DECEL_ZONE_M, _DESCEND_TOUCH_V_MS, _HOLD_TOL_N, _HOLD_TOL_PCT,
    _STREAM_HALT_LAT_S)

K_RIGIDA = T._K_RIGID_REF_NM      # 28 N/mm
K_SILICONE = 620.0                # N/m, medido 14/08


def _tol(t):
    return max(_HOLD_TOL_N, _HOLD_TOL_PCT * t)


def _pico_previsto(v_ms, k_nm):
    """O modelo que a fase inteira assume: curso comprometido × rigidez."""
    return v_ms * _STREAM_HALT_LAT_S * k_nm


@pytest.mark.parametrize('alvo', [0.1, 0.2, 0.5, 1.0, 2.0, 5.0])
def test_pico_previsto_cabe_na_banda_do_alvo(alvo):
    """A razão de existir de crawl_v_ms: contra a ponta RÍGIDA o transiente do
    toque não pode passar da borda superior da banda. É o que MovL não
    conseguia abaixo de ~0,94 N por causa do piso de 30 µm."""
    tol = _tol(alvo)
    v = crawl_v_ms(alvo, tol, K_RIGIDA)
    assert _pico_previsto(v, K_RIGIDA) <= alvo + tol + 1e-9


# Piso geométrico que o MovL tinha: 3 quanta de 10 µm da FK. Não é mais uma
# constante do código (o caminho MovL foi removido em 19/08/2026) — fica aqui
# como a MARCA a bater, porque foi ela que motivou a troca.
_PISO_MOVL_M = 30e-6


def test_alvo_de_01n_e_o_caso_que_movl_nao_atendia():
    """0,1 N contra ponta rígida: em MovL o piso de 30 µm dava 0,84 N (8x o
    alvo) e nenhuma velocidade o reduzia. Em streaming tem de caber na banda."""
    tol = _tol(0.1)
    v = crawl_v_ms(0.1, tol, K_RIGIDA)
    pico_streaming = _pico_previsto(v, K_RIGIDA)
    assert pico_streaming <= 0.1 + tol
    assert pico_streaming < 0.25 * _PISO_MOVL_M * K_RIGIDA


def test_velocidade_escala_com_a_rigidez():
    """"Considerando a rigidez": contato mole autoriza descer mais rápido com
    o mesmo orçamento de pico."""
    tol = _tol(0.5)
    assert crawl_v_ms(0.5, tol, K_SILICONE) > crawl_v_ms(0.5, tol, K_RIGIDA)


def test_velocidade_escala_com_o_alvo():
    """Alvo maior tolera transiente maior — longe pode ir mais rápido."""
    vs = [crawl_v_ms(a, _tol(a), K_RIGIDA) for a in (0.1, 0.5, 2.0)]
    assert vs == sorted(vs)


def test_clip_entre_piso_observavel_e_teto_de_toque():
    """Piso: abaixo de 10 µm/s um tick de 30 ms não move nem 0,3 µm e a
    descida some no quantum de 10 µm da FK. Teto: o rastejo nunca passa do
    limite de toque histórico."""
    assert crawl_v_ms(0.001, 0.001, 1e9) == _DESCEND_CRAWL_V_MIN_MS
    assert crawl_v_ms(50.0, 5.0, 1.0) == _DESCEND_TOUCH_V_MS


def test_zona_de_desaceleracao_tem_piso_de_3mm():
    assert _DESCEND_DECEL_ZONE_M == 0.003


def test_zona_nao_encolhe_com_contato_raso():
    """A zona lenta é `max(3 mm, frenagem + margem)`. O clip antigo em
    `0,5 x learned_m` a encolhia justamente onde ela é mais necessária: com
    contato aprendido em 3 mm sobravam 1,5 mm, menos que a distância de parada
    do estágio rápido. Com o piso, contato raso passa a rastejar TUDO — que é
    o resultado seguro."""
    import inspect
    from touch_pack.tactile_explorer import TactileExplorer
    src = inspect.getsource(TactileExplorer._phase_descending)
    assert 'zone_m = max(_DESCEND_DECEL_ZONE_M' in src
    assert '0.5 * max(0.0, learned_m)' not in src, (
        'o clip que encolhia a zona em contato raso voltou')


def test_t_halt_menor_compra_velocidade_linearmente():
    """A latência da cadeia é o que sobra para medir: v é LINEAR em 1/T_halt.
    Travar isto documenta onde está o ganho de tempo."""
    tol = _tol(0.1)
    lento = crawl_v_ms(0.1, tol, K_RIGIDA, t_halt_s=0.30)
    rapido = crawl_v_ms(0.1, tol, K_RIGIDA, t_halt_s=0.05)
    assert rapido == pytest.approx(6.0 * lento, rel=1e-6)
