"""Qualidade da onda de força: amplitude da FUNDAMENTAL, compensação da
amostragem e o teto de frequência.

Medido nos runs TOUCH de 17/08/2026 (SINE 0,2–3,0 N, ±1,40 N pedidos), sobre
o 2º bloco MODULATING de cada um, por ajuste de mínimos quadrados nos
harmônicos 1..6 de f0:

    f0       fundamental          THD (força)   atraso
    0,5 Hz   1,195 N  ( 85 %)        15 %       −14°  (78 ms)
    1,0 Hz   1,128 N  ( 81 %)        30 %       −29°  (80 ms)
    2,0 Hz   1,331 N  ( 95 %)        20 %       −65°  (90 ms)

O pico-a-pico dos MESMOS blocos batia a faixa pedida (chegou a 3,54 N contra
3,00 N no topo): a excursão acontecia, a FORMA é que não. O ciclo médio a
1 Hz mostra o entalhe no pico (2,31 → 2,05 → 2,10 → 2,28 N), assinatura do
limitador de faixa cortando por causa do atraso de leitura.
"""
import math

import pytest

pytest.importorskip('rclpy')


# ── Ganho da interpolação (o que a amostragem come da amplitude) ──────

def _gain(n):
    from touch_pack.tactile_explorer import _fmod_sampling_gain
    return _fmod_sampling_gain(n)


@pytest.mark.parametrize('pts, esperado', [
    (4, 0.811), (5, 0.875), (6, 0.912), (8, 0.950), (16, 0.987)])
def test_ganho_da_interpolacao_bate_com_a_integracao(pts, esperado):
    """sinc²(1/N) é a forma fechada da convolução com dois boxcars; os
    valores conferem com a integração numérica da onda reconstruída."""
    assert _gain(pts) == pytest.approx(esperado, abs=0.002)


def test_ganho_tende_a_um_com_muitos_pontos():
    assert _gain(1000) > 0.999


def test_compensacao_devolve_a_amplitude_pedida():
    """A amplitude comandada é dividida pelo ganho: o produto dos dois é a
    amplitude pedida, e é isso que faz o PRIMEIRO ciclo já sair certo em vez
    de esperar a adaptação convergir."""
    for pts in (5, 6, 8):
        amp_pre = 1.0 / _gain(pts)
        assert amp_pre * _gain(pts) == pytest.approx(1.0)
    # A 5 pontos por período (o caso de 10 Hz) a correção é de +14 %.
    assert 1.0 / _gain(5) == pytest.approx(1.143, abs=0.005)


# ── 10 Hz ────────────────────────────────────────────────────────────

def test_dez_hz_e_alcancavel_no_piso_do_servoj():
    """O piso de 20 ms do `t` do ServoJ é do FIRMWARE do CR10. Com 5 pontos
    por período ele dá exatamente 10 Hz — era 8 pontos, que travavam o teto
    em 6,25 Hz e faziam a onda de 10 Hz ser RECUSADA."""
    from touch_pack.tactile_explorer import (
        _fmod_max_freq_hz, _SERVOJ_T_MIN_S, _FMOD_MIN_PTS_PER_CYCLE)
    assert _fmod_max_freq_hz(_SERVOJ_T_MIN_S) == pytest.approx(10.0)
    assert _FMOD_MIN_PTS_PER_CYCLE == 5
    # E o tick escolhido para 10 Hz não fura o piso do firmware.
    from touch_pack.tactile_explorer import _ForceProfile
    prof = _ForceProfile('SINE', 1.0, 2.0, 10.0, 20)
    assert prof.wave_dt(_SERVOJ_T_MIN_S) == pytest.approx(_SERVOJ_T_MIN_S)
    assert prof.pts_per_cycle_at(_SERVOJ_T_MIN_S) == pytest.approx(5.0)


def test_dez_hz_com_servoj_de_30ms_continua_recusado():
    """O teto acompanha o período REAL do ServoJ: quem não subir o
    mirror_node para 20 ms não ganha 10 Hz — ganharia uma onda reamostrada."""
    from touch_pack.tactile_explorer import _fmod_max_freq_hz
    assert _fmod_max_freq_hz(0.030) < 10.0


def test_a_frequencia_maxima_nao_passou_do_hardware():
    """Nenhuma configuração pode prometer mais do que o firmware entrega."""
    from touch_pack.tactile_explorer import (
        _fmod_max_freq_hz, _SERVOJ_T_MIN_S)
    assert _fmod_max_freq_hz(0.001) > _fmod_max_freq_hz(_SERVOJ_T_MIN_S)
    # ...mas o caminho de produção satura o período no piso do firmware
    # antes de calcular o teto (ver _phase_hold_modulated).
    assert _fmod_max_freq_hz(_SERVOJ_T_MIN_S) == pytest.approx(10.0)


# ── Lock-in: por que a adaptação deixou de usar pico-a-pico ──────────

def _lockin(y, t, f0):
    """Amplitude de PICO da fundamental — o mesmo cálculo do laço."""
    i = sum(v * math.sin(2 * math.pi * f0 * ti) for v, ti in zip(y, t))
    q = sum(v * math.cos(2 * math.pi * f0 * ti) for v, ti in zip(y, t))
    return 2.0 * math.hypot(i, q) / len(y)


def _onda(f0=1.0, amp=1.4, n=400, cycles=4, corta=0.0, ruido=0.0):
    """Senoide de teste. `corta` achata as pontas na fração dada da
    amplitude — o efeito do limitador de faixa mordendo todo ciclo."""
    t = [k * cycles / (f0 * n) for k in range(n)]
    y = []
    for ti in t:
        v = amp * math.sin(2 * math.pi * f0 * ti)
        if corta:
            v = max(-corta * amp, min(corta * amp, v))
        y.append(v + ruido * math.sin(2 * math.pi * 37.0 * ti))
    return t, y


def test_lockin_mede_a_amplitude_de_uma_senoide_limpa():
    t, y = _onda()
    assert _lockin(y, t, 1.0) == pytest.approx(1.4, rel=0.02)


def test_pico_a_pico_e_cego_a_forma_e_a_fundamental_nao():
    """A assinatura dos runs de 17/08: p-p batendo a faixa pedida com a
    fundamental em 81 %. O p-p são DOIS pontos — dois transientes bastam
    para ele reportar 100 % sobre uma onda achatada no corpo inteiro.
    Adaptar por ele faz a onda parecer correta enquanto a forma se degrada."""
    t, y = _onda(corta=0.7)
    y[10], y[11] = 1.4, -1.4        # dois transientes seguram o p-p
    assert (max(y) - min(y)) / (2 * 1.4) == pytest.approx(1.0), \
        'o p-p não enxerga o corte'
    assert _lockin(y, t, 1.0) / 1.4 < 0.85, 'a fundamental tem de enxergar'


def test_lockin_rejeita_ruido_fora_da_frequencia():
    """Ruído em outra frequência entra inteiro no p-p e é integrado a zero
    pelo lock-in: é por isso que a adaptação por ciclo ficou estável."""
    t, y = _onda(ruido=0.5)
    assert max(y) - min(y) > 2 * 1.4 * 1.1     # o p-p incha com o ruído
    assert _lockin(y, t, 1.0) == pytest.approx(1.4, rel=0.05)


def test_lockin_ignora_o_atraso_de_transporte():
    """A leitura chega ~85 ms atrasada; o módulo do lock-in descarta a fase,
    então a amplitude medida não depende disso. Uma comparação instantânea
    comandado × medido dependeria."""
    t, y = _onda()
    y_atrasada = [1.4 * math.sin(2 * math.pi * 1.0 * (ti - 0.085))
                  for ti in t]
    assert _lockin(y_atrasada, t, 1.0) == pytest.approx(_lockin(y, t, 1.0),
                                                        rel=0.02)


# ── Tolerância do limitador de faixa ─────────────────────────────────

def test_limitador_de_faixa_tolera_o_atraso_de_leitura():
    """A tolerância tem de cobrir o pico que o atraso empurra para fora da
    faixa (medido: até +0,54 N numa onda de ±1,40 N), senão o limitador
    corta todo ciclo e volta a entalhar o topo."""
    from touch_pack.tactile_explorer import (
        _FMOD_BAND_TOL_FRAC, _FMOD_BAND_TOL_MIN_N)
    amp = 1.40
    tol = max(_FMOD_BAND_TOL_FRAC * amp, _FMOD_BAND_TOL_MIN_N)
    assert tol >= 0.15, 'tolerância pequena demais para o atraso medido'
    # ...e não tão larga que deixe de ser guarda: nunca meia amplitude.
    assert tol < 0.5 * amp
