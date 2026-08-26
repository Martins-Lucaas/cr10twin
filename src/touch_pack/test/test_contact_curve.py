"""Curva F(x) medida na descida (_ContactCurve) e o que a onda faz com ela.

O contato desta bancada NÃO tem rigidez escalar: medida em 14/08/2026 sobre o
run TOUCH/20260814_115804 (ponteira de silicone), a secante local vale
0,18 N/mm entre 0,06 e 0,20 N e 2,8–6,2 N/mm dentro da onda de 0,4 a 3,9 N.
Foi colapsar isso num número só que fez a onda pedida de 0,1–3,0 N comandar
4,03 mm p-p (63 mm/s de pico) e estourar o teto de força em 30 %.
"""
import math

import pytest

pytest.importorskip('rclpy')


@pytest.fixture(scope='module')
def C():
    from touch_pack.tactile_explorer import _ContactCurve
    return _ContactCurve


# Curva REAL do run 20260814_115804: (força N, penetração mm a partir de
# F=0,10 N), lida do samples.csv. É estritamente convexa — o material
# endurece sob carga, que é a razão de o escalar não servir.
CURVA_REAL = [(0.10, 0.000), (0.20, 0.630), (0.30, 0.720), (0.50, 1.030),
              (0.75, 1.350), (1.00, 1.440), (1.25, 1.640), (1.50, 1.830)]


def _monta(C, pares=CURVA_REAL):
    c = C()
    for f_n, x_mm in pares:
        c.add(x_mm * 1e-3, f_n)
    return c


# ── invertibilidade ───────────────────────────────────────────────────

def test_curva_vazia_nao_explode(C):
    c = C()
    assert not c.usable
    assert c.x_of_f(1.0) == 0.0
    assert c.dx_between(0.1, 3.0) == 0.0


def test_poucos_pontos_nao_sao_usaveis(C):
    from touch_pack.tactile_explorer import _FX_MIN_POINTS
    c = _monta(C, CURVA_REAL[:_FX_MIN_POINTS - 1])
    assert not c.usable


def test_excursao_curta_nao_e_usavel(C):
    """Oito pontos espremidos em 0,05 N não descrevem curva nenhuma."""
    c = _monta(C, [(1.50 + i * 0.006, 1.8 + i * 0.01) for i in range(8)])
    assert not c.usable


def test_curva_real_e_usavel(C):
    assert _monta(C).usable


def test_interpola_os_pontos_medidos(C):
    c = _monta(C)
    for f_n, x_mm in CURVA_REAL:
        assert c.x_of_f(f_n) == pytest.approx(x_mm * 1e-3, abs=1e-9)
    # No meio de um segmento, linear entre os vizinhos.
    assert c.x_of_f(0.40) == pytest.approx(
        (1.030 + 0.720) / 2 * 1e-3, abs=1e-9)


def test_pares_fora_de_ordem_nao_quebram_a_inversao(C):
    """Os pares chegam na ordem em que a descida os produziu; ruído da célula
    pode inverter dois vizinhos. A curva tem de continuar monotônica, senão
    x_of_f deixa de ser função."""
    baralhado = [CURVA_REAL[i] for i in (3, 0, 7, 2, 5, 1, 6, 4)]
    c = _monta(C, baralhado)
    xs = [c.x_of_f(f) for f in (0.1, 0.3, 0.5, 1.0, 1.5)]
    assert xs == sorted(xs)


def test_inversao_de_ruido_nao_abre_buraco(C):
    """Um par com x menor que o anterior é ACHATADO, não descartado —
    descartar abriria buraco justamente no trecho mole."""
    c = _monta(C, CURVA_REAL + [(1.10, 1.000)])   # x recua contra o vizinho
    assert c.usable
    xs = [c.x_of_f(f) for f in (1.00, 1.10, 1.25)]
    assert xs == sorted(xs)


# ── a não-linearidade que o escalar não vê ────────────────────────────

def test_secante_local_varia_muito_dentro_da_faixa(C):
    """É este espalhamento que torna o escalar inadequado."""
    c = _monta(C)
    k_pe   = c.k_secant(0.10, 0.30)
    k_topo = c.k_secant(1.00, 1.50)
    assert k_topo > 4.0 * k_pe


def test_curva_pede_menos_curso_que_o_escalar_do_pe(C):
    """O caso concreto: K=0,70 N/mm (secante do curso inteiro) pedia 4,14 mm
    p-p para 0,1–3,0 N. A curva pede 2,97 mm — 28 % menos, e com a forma
    certa em vez de uma reta.

    Os 2,97 mm ainda são generosos: acima de 1,5 N (onde a descida parou) a
    curva extrapola pela secante QUASE-ESTÁTICA da ponta, 1,32 N/mm, mais
    mole que os 2,8–6,2 N/mm que a onda mediu de fato. Quem fecha essa
    diferença é o ganho por ciclo; a extrapolação só não pode ser pior que o
    escalar, e não é."""
    c = _monta(C)
    pp_curva = abs(c.dx_between(0.10, 3.00))
    pp_escalar = 2 * 1.45 / 700.0        # amp_n / K, com K = 0,70 N/mm
    assert pp_curva < 0.8 * pp_escalar
    assert pp_curva == pytest.approx(0.00297, abs=1e-4)


def test_dentro_do_medido_a_curva_diverge_do_escalar(C):
    """Sem extrapolação — a faixa que a descida realmente percorreu — a curva
    pede 0,39 mm para 1,0–1,5 N contra 0,71 mm do escalar de 0,70 N/mm: quase
    metade, porque ali o material já endureceu para 1,3 N/mm."""
    c = _monta(C)
    pp_curva = abs(c.dx_between(1.00, 1.50))
    pp_escalar = (1.50 - 1.00) / 700.0
    assert pp_curva == pytest.approx(0.00039, abs=1e-5)
    assert pp_curva < 0.6 * pp_escalar


def test_extrapola_acima_do_medido_com_a_rigidez_do_topo(C):
    """A onda pede até 3,0 N mas a descida parou em 1,5 N (o alvo). O trecho
    acima é extrapolado pela inclinação da ponta, não pela do curso inteiro."""
    c = _monta(C)
    k_topo = c.k_secant(1.25, 1.50)
    esperado = c.x_of_f(1.50) + (3.00 - 1.50) / k_topo
    assert c.x_of_f(3.00) == pytest.approx(esperado, rel=1e-6)


def test_extrapolacao_respeita_os_limites_do_estimador(C):
    """Um segmento de ponta quase plano extrapolaria para o infinito; o
    grampo em _K_MIN_NM/_K_MAX_NM é o que impede isso."""
    from touch_pack.tactile_explorer import _K_MIN_NM
    plana = [(1.0 + i * 0.05, 1.0 + i * 1.0) for i in range(8)]   # molíssima
    c = _monta(C, plana)
    dx = abs(c.dx_between(1.35, 3.00))
    assert dx <= (3.00 - 1.35) / _K_MIN_NM + 1e-9


def test_origem_dos_pares_nao_importa(C):
    """dx_between é uma DIFERENÇA: deslocar todos os x deixa tudo igual. É o
    que permite alimentar a curva com a penetração da descida e consumi-la a
    partir de onde o HOLD deixou o braço."""
    c1 = _monta(C)
    c2 = _monta(C, [(f, x + 12.5) for f, x in CURVA_REAL])
    assert c2.dx_between(0.2, 1.4) == pytest.approx(c1.dx_between(0.2, 1.4))


def test_misturar_origens_destroi_o_topo_da_curva(C):
    """Por que `_qs_regulate` só alimenta a curva quando feed_curve=True.

    `deepened_m` é relativo ao início de CADA chamada: a descida conta a
    partir do contato, e o HOLD recomeça do zero já lá em cima. Alimentar a
    curva nos dois deixa pares de força ALTA com penetração ~0; o achatamento
    monotônico os cola no topo da descida, criando um patamar plano — e é
    justamente dele que `_edge_k` tiraria a inclinação para extrapolar.

    O resultado medido: a secante do topo cai de 1,32 para 0,54 N/mm e a
    extrapolação até 3,0 N vai de 2,97 para 4,59 mm — 54 % de curso a MAIS,
    que é exatamente a falha que a curva existe para eliminar. Este teste
    guarda o motivo: se alguém ligar feed_curve no HOLD, ele quebra."""
    contaminada = _monta(C)
    for f_n, x_mm in [(1.45, 0.00), (1.50, 0.05), (1.55, 0.10)]:
        contaminada.add(x_mm * 1e-3, f_n)   # origem do HOLD, não da descida
    limpa = _monta(C)
    assert limpa.k_secant(1.50, 3.00) > 2.0 * contaminada.k_secant(1.50, 3.00)
    assert abs(contaminada.dx_between(0.10, 3.00)) > \
        1.5 * abs(limpa.dx_between(0.10, 3.00))


def test_nao_finito_e_ignorado(C):
    c = _monta(C)
    antes = c.dx_between(0.1, 1.5)
    c.add(float('nan'), 2.0)
    c.add(1e-3, float('inf'))
    assert c.dx_between(0.1, 1.5) == pytest.approx(antes)


def test_reset_esquece_o_contato_anterior(C):
    c = _monta(C)
    c.reset()
    assert not c.usable


# ── rampa de amplitude e teto de velocidade ───────────────────────────

def _amp_scale(t, hz):
    from touch_pack.tactile_explorer import (
        _FMOD_AMP_RAMP_START, _FMOD_AMP_RAMP_CYCLES)
    return min(1.0, _FMOD_AMP_RAMP_START + (1.0 - _FMOD_AMP_RAMP_START)
               * (t * hz) / _FMOD_AMP_RAMP_CYCLES)


def test_rampa_abre_reduzida_e_chega_a_cheia(C):
    """O ciclo 1 é o que roda com a estimativa ainda não corrigida — e foi ele
    que, com amplitude cheia, levou a força a 3,90 N contra 3,00 pedidos."""
    from touch_pack.tactile_explorer import (
        _FMOD_AMP_RAMP_START, _FMOD_AMP_RAMP_CYCLES)
    hz = 5.0
    assert _amp_scale(0.0, hz) == pytest.approx(_FMOD_AMP_RAMP_START)
    assert _amp_scale(_FMOD_AMP_RAMP_CYCLES / hz, hz) == pytest.approx(1.0)
    assert _amp_scale(10.0, hz) == pytest.approx(1.0)   # satura, não passa


def test_rampa_e_monotonica(C):
    vals = [_amp_scale(i / 100.0, 5.0) for i in range(200)]
    assert vals == sorted(vals)


def test_teto_de_velocidade_pega_o_ensaio_do_run_real(C):
    """0,1–3,0 N a 5 Hz nesta ponteira: mesmo com a curva certa o pico passa
    do teto de aviso. O ensaio é agressivo por si, não só por bug."""
    from touch_pack.tactile_explorer import (
        _FMOD_V_PEAK_WARN_MMS, _FMOD_V_PEAK_MAX_MMS)
    c = _monta(C)
    amp_m = 0.5 * abs(c.dx_between(0.10, 3.00))
    v_pico = 2 * math.pi * 5.0 * amp_m * 1e3
    assert v_pico > _FMOD_V_PEAK_WARN_MMS
    assert _FMOD_V_PEAK_WARN_MMS < _FMOD_V_PEAK_MAX_MMS


def test_banda_estreita_passa_sem_reclamar(C):
    """1,0–2,0 N a 5 Hz vale ~0,5 mm p-p — 8 mm/s, folgado."""
    from touch_pack.tactile_explorer import _FMOD_V_PEAK_WARN_MMS
    c = _monta(C)
    amp_m = 0.5 * abs(c.dx_between(1.00, 2.00))
    assert 2 * math.pi * 5.0 * amp_m * 1e3 < _FMOD_V_PEAK_WARN_MMS
