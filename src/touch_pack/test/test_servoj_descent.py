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
    TactileExplorer as T, crawl_v_ms, impact_peak_n, _CONTACT_ON_N,
    _DESCEND_CRAWL_V_MIN_MS, _DESCEND_DECEL_ZONE_M, _DESCEND_TOUCH_V_MS,
    _STREAM_HALT_LAT_S)

K_RIGIDA = T._K_RIGID_REF_NM      # 28 N/mm
K_SILICONE = 620.0                # N/m, medido 14/08


# O modelo que a fase inteira assume (curso comprometido × rigidez) mora em
# produção desde 27/08/2026: o espelho local escondia que `crawl_v_ms` devolve
# a velocidade CLIPADA, e portanto que o pico real podia não ser o orçado.
_pico_previsto = impact_peak_n


def test_pico_previsto_para_no_limiar_de_contato():
    """A razão de existir de crawl_v_ms: contra a ponta RÍGIDA o transiente do
    toque para no LIMIAR DE CONTATO. É o que MovL não conseguia abaixo de
    ~0,94 N por causa do piso de 30 µm.

    O orçamento era `alvo + tol` até 27/08/2026 — o primeiro impacto tinha
    licença para entregar o setpoint inteiro sozinho, antes de qualquer laço
    reagir. Agora ele mira em DETECTAR: quem sobe de 0,1 N até o setpoint é o
    regulador quase-estático, em micro-passos e com as três guardas de
    não-ultrapassagem."""
    v = crawl_v_ms(K_RIGIDA)
    assert _pico_previsto(v, K_RIGIDA) <= _CONTACT_ON_N + 1e-9


# Piso geométrico que o MovL tinha: 3 quanta de 10 µm da FK. Não é mais uma
# constante do código (o caminho MovL foi removido em 19/08/2026) — fica aqui
# como a MARCA a bater, porque foi ela que motivou a troca.
_PISO_MOVL_M = 30e-6


def test_alvo_de_01n_e_o_caso_que_movl_nao_atendia():
    """0,1 N contra ponta rígida: em MovL o piso de 30 µm dava 0,84 N (8x o
    alvo) e nenhuma velocidade o reduzia. Em streaming o impacto para no
    limiar."""
    pico_streaming = _pico_previsto(crawl_v_ms(K_RIGIDA), K_RIGIDA)
    assert pico_streaming <= _CONTACT_ON_N
    assert pico_streaming < 0.25 * _PISO_MOVL_M * K_RIGIDA


def test_velocidade_escala_com_a_rigidez():
    """"Considerando a rigidez": contato mole autoriza descer mais rápido com
    o mesmo orçamento de pico."""
    assert crawl_v_ms(K_SILICONE) > crawl_v_ms(K_RIGIDA)


def test_o_impacto_nao_depende_mais_do_setpoint():
    """O INVERSO do que se cobrava antes, e é a mudança inteira: a velocidade
    de rastejo escalava com o alvo, então um ensaio de 5 N tocava com 5 N. O
    toque agora é o MESMO em 0,2 N e em 5 N — ele detecta contato, não mede
    força.

    `crawl_v_ms` nem recebe mais o alvo; este teste existe para que devolver
    o parâmetro exija passar por aqui.
    """
    import inspect
    params = inspect.signature(crawl_v_ms).parameters
    assert 'target_f' not in params and 'tol_n' not in params
    # Contra a ponta rígida (onde o clip não morde) o pico previsto É o
    # limiar — e seria o mesmo com qualquer setpoint, porque o setpoint não
    # entra mais na conta.
    assert _pico_previsto(crawl_v_ms(K_RIGIDA), K_RIGIDA) == pytest.approx(
        _CONTACT_ON_N, rel=1e-9)


def test_clip_entre_piso_observavel_e_teto_de_toque():
    """Piso: abaixo de 10 µm/s um tick de 30 ms não move nem 0,3 µm e a
    descida some no quantum de 10 µm da FK. Teto: o rastejo nunca passa do
    limite de toque histórico."""
    assert crawl_v_ms(1e9) == _DESCEND_CRAWL_V_MIN_MS
    assert crawl_v_ms(1.0) == _DESCEND_TOUCH_V_MS


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


# ── o clip e o orçamento ─────────────────────────────────────────────

def test_o_piso_de_velocidade_torna_o_orcamento_inatingivel_acima_de_33knm():
    """O ACHADO que motivou `impact_peak_n`: `crawl_v_ms` clipa, e acima da
    rigidez em que o piso morde o pico real deixa de ser o orçado — sem que a
    função tenha como dizer.

    A referência em uso (28 kN/m) está a apenas 19 % do ponto de virada, então
    isto não é um canto teórico: basta alguém corrigir `_K_RIGID_REF_NM` para
    a rigidez da pilha real e o orçamento vira ficção. Quem avisa é
    `_phase_descending`, comparando `impact_peak_n` com o limiar.
    """
    k_virada = _CONTACT_ON_N / (_STREAM_HALT_LAT_S * _DESCEND_CRAWL_V_MIN_MS)
    assert crawl_v_ms(k_virada) == pytest.approx(_DESCEND_CRAWL_V_MIN_MS)
    # Abaixo da virada o orçamento é honrado…
    assert impact_peak_n(crawl_v_ms(0.9 * k_virada),
                         0.9 * k_virada) == pytest.approx(_CONTACT_ON_N)
    # …acima dela, não — e é por isso que existe um aviso.
    assert impact_peak_n(crawl_v_ms(10.0 * k_virada),
                         10.0 * k_virada) > 10.0 * _CONTACT_ON_N


def test_a_referencia_em_uso_ainda_honra_o_orcamento():
    """Guarda de regressão do valor em produção: se `_K_RIGID_REF_NM` subir
    acima da virada, este teste cai e obriga a decidir — em vez de descobrir
    na bancada com 2,7 N numa amostra."""
    assert impact_peak_n(crawl_v_ms(T._K_RIGID_REF_NM),
                         T._K_RIGID_REF_NM) == pytest.approx(_CONTACT_ON_N)


def test_a_fase_avisa_quando_o_piso_manda():
    """O aviso tem de existir no caminho da descida, não só na matemática."""
    import inspect
    src = inspect.getsource(T._phase_descending)
    assert 'impact_peak_n(' in src
    assert 'latency_probe' in src


def test_t_halt_menor_compra_velocidade_linearmente():
    """A latência da cadeia é o que sobra para medir: v é LINEAR em 1/T_halt.
    Travar isto documenta onde está o ganho de tempo."""
    lento = crawl_v_ms(K_RIGIDA, t_halt_s=0.30)
    rapido = crawl_v_ms(K_RIGIDA, t_halt_s=0.05)
    assert rapido == pytest.approx(6.0 * lento, rel=1e-6)
