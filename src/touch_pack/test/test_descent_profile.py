"""Perfil de velocidade do DESCENDING em ar livre — três estágios.

Contexto (01/09/2026): mesmo com a home aprendida o DESCENDING levava ~2 min.
Medido nas coletas do dia:

  20260901_103554  118,5 s de DESCENDING para 11,6 mm (11 amostras rápidas)
  20260901_103033  232,8 s para 11,6 mm (nenhuma rápida — home não aprendida)

A causa: a zona lenta era percorrida INTEIRA a `crawl_v_ms` (50 µm/s), e ela
é a soma de duas coisas com papéis diferentes —

  frenagem  v_fast · _ZONE_REACTION_S = 15 mm/s · 0,3 s = 4,5 mm
  margem    incerteza do contato, 1,5 mm (palpite) … 0,4 mm (medida)

A frenagem está inteira ACIMA da janela `learned ± margem`, a única onde o
contato pode estar. Rastejá-la não compra segurança nenhuma: 4,5 mm a 50 µm/s
são 90 s em ar livre sem nada para tocar. Agora ela é uma rampa v_fast→v_slow
e só a margem rasteja.

O orçamento do primeiro impacto (`v · T_halt · K = _CONTACT_ON_N`) NÃO muda —
é o que o test_invariante_do_orcamento trava.
"""
import math

import pytest


@pytest.fixture(scope='module')
def m():
    from touch_pack import tactile_explorer
    return tactile_explorer


@pytest.fixture(scope='module')
def perfil(m):
    return m.TactileExplorer._free_descent_v


# Valores default dos ensaios (params.json das coletas de 01/09).
V_FAST = 0.015                     # approach_speed_mms = 15 mm/s
LEARNED = 0.011549                 # store: contato em 11,549 mm


def _cenario(m, margin_m):
    """Reproduz o dimensionamento da zona feito em _phase_descending."""
    v_slow = m.crawl_v_ms(m.TactileExplorer._K_RIGID_REF_NM)
    brake = V_FAST * m._ZONE_REACTION_S
    zone = max(m._DESCEND_DECEL_ZONE_M, brake + margin_m)
    return v_slow, LEARNED - zone, LEARNED - margin_m


def _tempo(perfil, ramp, crawl, v_fast, v_slow, ate_m, passos=200_000):
    """∫dx/v do início até `ate_m`, por retângulos."""
    dx = ate_m / passos
    return sum(dx / perfil(i * dx, ramp, crawl, v_fast, v_slow, v_slow)
               for i in range(passos))


# ── a invariante de segurança ─────────────────────────────────────────

def test_invariante_do_orcamento(m, perfil):
    """De crawl_from em diante — a janela onde o contato PODE estar — a
    velocidade é exatamente v_slow. É ela que mantém o pico do primeiro toque
    no limiar de contato; se este teste cair, a rampa está entregando o toque
    mais rápido que o orçamento."""
    v_slow, ramp, crawl = _cenario(m, m._CONTACT_ZONE_MARGIN_M)
    for frac in (0.0, 0.001, 0.25, 0.5, 0.9, 1.0, 1.5):
        x = crawl + frac * (LEARNED - crawl)
        assert perfil(x, ramp, crawl, V_FAST, v_slow, v_slow) == v_slow
    # e o pico projetado nessa velocidade continua sendo o orçamento
    pico = m.impact_peak_n(v_slow, m.TactileExplorer._K_RIGID_REF_NM)
    assert pico <= m._CONTACT_ON_N + 1e-9


def test_nunca_acelera_ao_descer(m, perfil):
    """Monotonicamente não-crescente: a descida só desacelera conforme se
    aproxima do contato. Um degrau para cima seria um toque mais forte."""
    v_slow, ramp, crawl = _cenario(m, m._CONTACT_ZONE_MARGIN_M)
    vs = [perfil(i * LEARNED / 4000, ramp, crawl, V_FAST, v_slow, v_slow)
          for i in range(4001)]
    assert all(b <= a + 1e-15 for a, b in zip(vs, vs[1:]))
    assert min(vs) == v_slow and max(vs) == V_FAST


def test_rampa_e_continua_nas_bordas(m, perfil):
    v_slow, ramp, crawl = _cenario(m, m._CONTACT_ZONE_MARGIN_M)
    assert perfil(ramp, ramp, crawl, V_FAST, v_slow, v_slow) \
        == pytest.approx(V_FAST, rel=1e-9)
    assert perfil(crawl - 1e-9, ramp, crawl, V_FAST, v_slow, v_slow) \
        == pytest.approx(v_slow, abs=1e-7)


# ── o ganho de tempo ──────────────────────────────────────────────────

def test_margem_conservadora_cai_de_dois_minutos(m, perfil):
    """Estado de hoje: o store tem 1 contato só, então a margem é o palpite
    conservador de 1,5 mm. Antes a zona inteira (6,0 mm) rastejava = 120 s."""
    v_slow, ramp, crawl = _cenario(m, m._CONTACT_ZONE_MARGIN_M)
    antes = (LEARNED - ramp) / v_slow                 # zona inteira rastejada
    depois = _tempo(perfil, ramp, crawl, V_FAST, v_slow, LEARNED)
    assert antes > 110.0
    assert depois < 0.30 * antes
    # o que sobra é a margem no orçamento — o piso honesto desta configuração
    assert depois == pytest.approx(m._CONTACT_ZONE_MARGIN_M / v_slow, rel=0.15)


def test_margem_aprendida_fecha_em_dez_segundos(m, perfil):
    """Com >= _CONTACT_MARGIN_MIN_PTS contatos e dispersão apertada a margem
    cai ao piso de _CONTACT_MARGIN_FLOOR_M. Aí o DESCENDING inteiro dá 10,1 s,
    e a conta fica quase toda no termo IRREDUTÍVEL: 0,4 mm de janela de
    incerteza a 50 µm/s = 7,9 s, que é o orçamento de impacto, não folga.

    Os 10,1 s não são arredondados para 10,0 de propósito. O que resta para
    baixar não está neste perfil: é o `v_slow` (medir T_halt com
    latency_probe.py — v é linear em 1/T_halt) ou a própria margem."""
    v_slow, ramp, crawl = _cenario(m, m._CONTACT_MARGIN_FLOOR_M)
    total = _tempo(perfil, ramp, crawl, V_FAST, v_slow, LEARNED)
    assert total <= 11.0
    # e o resto do perfil — rápido + frenagem — cabe em ~2 s
    irredutivel = m._CONTACT_MARGIN_FLOOR_M / v_slow
    assert total - irredutivel < 2.5


def test_a_rampa_e_mais_lenta_que_a_reacao_da_frenagem(m, perfil):
    """A frenagem existe para dar curso ao braço largar v_fast em
    _ZONE_REACTION_S. A rampa tem de gastar MAIS que isso — se ela cruzasse a
    frenagem mais rápido que a reação do braço, o curso não seria suficiente."""
    v_slow, ramp, crawl = _cenario(m, m._CONTACT_ZONE_MARGIN_M)
    t_rampa = _tempo(perfil, ramp, crawl, V_FAST, v_slow, crawl) \
        - _tempo(perfil, ramp, crawl, V_FAST, v_slow, ramp)
    assert t_rampa > m._ZONE_REACTION_S
    # fórmula fechada da rampa linear em distância, como referência
    brake = crawl - ramp
    esperado = brake / (V_FAST - v_slow) * math.log(V_FAST / v_slow)
    assert t_rampa == pytest.approx(esperado, rel=0.02)


# ── home não aprendida: nada muda ─────────────────────────────────────

def test_home_nova_rasteja_inteira(perfil):
    """Sem contato aprendido o contato pode vir a qualquer momento, então a
    descida inteira segue no orçamento. A rampa não se aplica."""
    for x in (0.0, 0.005, 0.05):
        assert perfil(x, None, None, 0.015, 0.00005, 0.00005) == 0.00005


def test_zona_degenerada_nao_estoura(perfil):
    """crawl_from == ramp_from (frenagem nula): sem divisão por zero, e o
    resultado é o lado seguro."""
    assert perfil(0.01, 0.01, 0.01, 0.015, 0.00005, 0.00005) == 0.00005
