"""Perfil de força modulada do modo TOUCH (_ForceProfile).

A onda é executada por feedforward de posição (Δx = ΔF/K), então o que
precisa estar certo aqui é a forma: média, amplitude, extremos dentro da
faixa pedida e duração igual a ciclos/frequência.
"""
import math

import pytest

pytest.importorskip('rclpy')


@pytest.fixture(scope='module')
def P():
    from touch_pack.tactile_explorer import _ForceProfile
    return _ForceProfile


def test_sine_stays_inside_the_requested_range(P):
    p = P('SINE', 2.0, 3.0, 10.0, 20)
    assert p.mean_n == pytest.approx(2.5)
    assert p.amp_n == pytest.approx(0.5)
    vals = [p.setpoint_n(i / 2000.0) for i in range(2000)]
    assert min(vals) == pytest.approx(2.0, abs=1e-3)
    assert max(vals) == pytest.approx(3.0, abs=1e-3)


def test_sine_starts_at_the_mean_and_cosine_at_the_peak(P):
    assert P('SINE', 2.0, 3.0, 10.0, 1).setpoint_n(0.0) == pytest.approx(2.5)
    assert P('COSINE', 2.0, 3.0, 10.0, 1).setpoint_n(0.0) == pytest.approx(3.0)


def test_period_matches_the_requested_frequency(P):
    p = P('SINE', 2.0, 3.0, 10.0, 20)
    for t in (0.0, 0.017, 0.033):
        assert p.setpoint_n(t) == pytest.approx(p.setpoint_n(t + 0.1))
    assert p.duration_s == pytest.approx(2.0)


def test_limits_are_order_agnostic(P):
    """min/max trocados descrevem a mesma onda — quem chama não precisa saber."""
    a, b = P('SINE', 3.0, 2.0, 5.0, 4), P('SINE', 2.0, 3.0, 5.0, 4)
    assert (a.f_min_n, a.f_max_n) == (b.f_min_n, b.f_max_n)


def test_control_rate_is_the_limit_at_10hz(P):
    """A 10 Hz o laço de 33 Hz dá ~3 pontos por período — abaixo do mínimo.

    É por isso que 10 Hz exige o ServoJ no piso do firmware (20 ms): no tick
    do QS a onda não cabe, e o explorer recusa em vez de reamostrar."""
    from touch_pack.tactile_explorer import (
        _CTRL_DT, _FMOD_MIN_PTS_PER_CYCLE, _SERVOJ_T_MIN_S)
    p = P('SINE', 2.0, 3.0, 10.0, 20)
    assert p.pts_per_cycle == pytest.approx(1.0 / (10.0 * _CTRL_DT))
    assert p.pts_per_cycle < _FMOD_MIN_PTS_PER_CYCLE
    # Teto rastreável com a cadência atual, para o aviso do log ser honesto.
    f_max_hz = 1.0 / (_CTRL_DT * _FMOD_MIN_PTS_PER_CYCLE)
    assert math.isclose(f_max_hz, 6.6666, rel_tol=1e-3)
    # ...e no piso do ServoJ os mesmos 10 Hz passam a caber.
    assert p.pts_per_cycle_at(_SERVOJ_T_MIN_S) >= _FMOD_MIN_PTS_PER_CYCLE
    assert P('SINE', 2.0, 3.0, 4.0, 8).pts_per_cycle >= _FMOD_MIN_PTS_PER_CYCLE


# ── O piso do slider e o piso do explorer têm de ser o MESMO ──────────
def test_the_gui_never_offers_a_force_the_explorer_will_refuse():
    """O DEFEITO que este teste existe para impedir que volte.

    `_force_profile()` recusa qualquer onda cujo mínimo caia abaixo de
    CONTACT_ON_N, e devolve None — o que faz o run cair no HOLD comum, sem
    onda nenhuma. O piso do slider da GUI (FORCE_SP_MIN) era 0,1 N cravado à
    mão, e enquanto CONTACT_ON_N também valeu 0,10 os dois concordaram por
    coincidência.

    Em 28/08/2026 CONTACT_ON_N subiu para 0,12 e a coincidência acabou: o
    slider passou a oferecer 0,1 N como MENOR valor escolhível, e o explorer a
    rejeitá-lo. O run TOUCH/20260828_134305 pediu 0,10–2,00 N, teve o perfil
    recusado e rodou 20 ciclos de senoide que nunca existiram — o setpoint
    ficou constante em 1,0 N do começo ao fim.

    Um slider cujo valor mínimo é inválido é pior que um slider mais curto.
    """
    from touch_pack.constants import CONTACT_ON_N, FORCE_SETPOINT_MAX_N
    from touch_pack.gui_constants import FORCE_SP_MIN, FORCE_SP_MAX
    assert FORCE_SP_MIN >= CONTACT_ON_N, (
        f'a GUI oferece {FORCE_SP_MIN} N e o explorer recusa abaixo de '
        f'{CONTACT_ON_N} N — todo run no piso do slider perde a modulação')
    assert FORCE_SP_MAX <= FORCE_SETPOINT_MAX_N


def test_a_wave_at_the_gui_floor_is_accepted_by_the_explorer_rule():
    """A regra do explorer, aplicada ao pior caso que a GUI consegue montar:
    a onda mais baixa possível tem de passar. É o teste acima visto do outro
    lado — não basta o piso ser >= ao limiar, o valor no piso tem de ser
    aceito de fato."""
    from touch_pack.constants import CONTACT_ON_N, FORCE_SETPOINT_MAX_N
    from touch_pack.gui_constants import FORCE_SP_MIN, FORCE_SP_MAX
    from touch_pack.tactile_explorer import _ForceProfile
    p = _ForceProfile('SINE', FORCE_SP_MIN, FORCE_SP_MAX, 1.0, 20)
    assert not (p.f_max_n > FORCE_SETPOINT_MAX_N or p.f_min_n < CONTACT_ON_N)
