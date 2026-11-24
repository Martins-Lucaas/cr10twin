"""Núcleo de admitância cartesiana: regime, retorno elástico e segurança."""
import numpy as np
import pytest

from rehab_pack.admittance import (AdmittanceAbort, AdmittanceController,
                                   arm_stiffness_limit)


def run(c, f, n):
    for _ in range(n):
        x = c.step(f)
    return x


def test_free_guidance_reaches_terminal_velocity_F_over_B():
    c = AdmittanceController(mass_kg=2, damping_ns_m=40, stiffness_n_m=0,
                             deadband_n=0.0, v_max_ms=1.0)
    # 30 passos = 0,9 s (tau = M/B = 50 ms), ainda dentro da excursão máxima
    run(c, [4, 0, 0], 30)
    assert c.v[0] == pytest.approx(4 / 40, rel=1e-3)


def test_spring_equilibrium_is_F_over_K():
    c = AdmittanceController(mass_kg=2, damping_ns_m=40, stiffness_n_m=200,
                             deadband_n=0.0, v_max_ms=1.0)
    x = run(c, [0, 6, 0], 1000)
    assert x[1] == pytest.approx(6 / 200, rel=1e-3)


def test_returns_to_reference_when_force_is_released():
    c = AdmittanceController(stiffness_n_m=200, deadband_n=0.0)
    run(c, [5, 0, 0], 300)
    x = run(c, [0, 0, 0], 2000)
    assert np.linalg.norm(x - c.x_ref) < 1e-4


def test_deadband_ignores_sensor_noise():
    c = AdmittanceController(deadband_n=0.5)
    x = run(c, [0.3, -0.3, 0.2], 200)
    assert np.allclose(x, 0.0)


def test_velocity_and_excursion_are_clamped():
    c = AdmittanceController(damping_ns_m=0.0, v_max_ms=0.05,
                             max_excursion_m=0.02, deadband_n=0.0)
    run(c, [9, 0, 0], 300)
    assert np.linalg.norm(c.v) <= 0.05 + 1e-12
    assert np.linalg.norm(c.x - c.x_ref) <= 0.02 + 1e-12


def test_force_above_limit_aborts_without_moving():
    c = AdmittanceController(abort_force_n=15.0)
    with pytest.raises(AdmittanceAbort):
        c.step([16, 0, 0])
    assert np.allclose(c.x, 0.0)


def test_nan_force_aborts():
    with pytest.raises(AdmittanceAbort):
        AdmittanceController().step([float('nan'), 0, 0])


def test_stale_force_stops_instead_of_following_last_reading():
    c = AdmittanceController(deadband_n=0.0)
    run(c, [4, 0, 0], 50)
    x0 = c.x.copy()
    x1 = c.step([4, 0, 0], force_age_s=1.0)
    assert np.allclose(c.v, 0.0) and np.allclose(x1, x0)


def test_implicit_integration_stays_bounded_where_euler_diverged():
    c = AdmittanceController(mass_kg=0.5, damping_ns_m=100, stiffness_n_m=5000,
                             deadband_n=0.0, v_max_ms=1.0)
    assert run(c, [5, 0, 0], 500)[0] == pytest.approx(5 / 5000, rel=1e-3)


def coupled(kh, hand, delay_ticks=4, adapt=True, secs=12.0, k=0.0):
    """Braço-mola segurando o mastro, robô com atraso puro de 0,12 s."""
    dt = 0.03
    c = AdmittanceController(stiffness_n_m=k, abort_force_n=1e9)
    buf, xs, prev, peak = [0.0] * (delay_ticks + 1), [], 0.0, 1.0
    for i in range(int(secs / dt)):
        xr = buf.pop(0)
        v, prev = (xr - prev) / dt, xr
        f = np.clip(kh * (hand(i * dt) - xr) - 2.0 * v, -14, 14)
        buf.append(c.step([f, 0, 0], 0.0, [v, 0, 0] if adapt else None)[0])
        xs.append(xr)
        peak = max(peak, c.scale)
    return float(np.ptp(xs[-100:])), peak


def step_hand(t):
    return 0.03 if t > 0.5 else 0.0


@pytest.mark.parametrize('kh', [500, 800, 1200])
def test_adaptation_kills_the_limit_cycle_of_a_stiff_arm(kh):
    assert coupled(kh, step_hand, adapt=False)[0] > 0.015   # fixo: oscila
    assert coupled(kh, step_hand)[0] < 0.002


@pytest.mark.parametrize('freq', [0.3, 0.5, 0.8])
@pytest.mark.parametrize('k', [0.0, 50.0])
def test_adaptation_ignores_voluntary_movement(freq, k):
    assert coupled(150, lambda t: 0.08 * np.sin(2 * np.pi * freq * t), k=k)[1] < 1.5


def test_raising_the_scale_does_not_create_kinetic_energy():
    c = AdmittanceController()
    c.v[:] = [0.1, 0, 0]
    e0 = c.m * c.scale * c.v @ c.v
    c._adapt(-5.0)
    assert c.scale > 1 and c.m * c.scale * c.v @ c.v <= e0 * (1 + 1e-9)


def test_stiffness_limit_matches_simulation():
    k = arm_stiffness_limit(2.0, 40.0)
    assert coupled(0.7 * k, step_hand, adapt=False)[0] < 0.002
    assert coupled(1.4 * k, step_hand, adapt=False)[0] > 0.01
    assert arm_stiffness_limit(2.0, 0.0) == 0.0


@pytest.mark.parametrize('mudar', [
    lambda c: c.set_reference([0.3, 0, 0]),
    lambda c: setattr(c, 'max_excursion', 0.05)])
def test_setpoint_nunca_salta(mudar):
    c = AdmittanceController()
    run(c, [8, 0, 0], 60)
    for _ in range(80):
        x0 = c.x.copy()
        mudar(c)
        c.step([0, 0, 0])
        assert np.linalg.norm(c.x - x0) <= c.v_max * c.dt + 1e-12


def test_excursao_vale_desde_o_habilitar_mesmo_com_a_referencia_andando():
    c = AdmittanceController(stiffness_n_m=200, max_excursion_m=0.05, deadband_n=0.0)
    for i in range(600):
        c.set_reference([0.001 * i, 0, 0])
        c.step([5, 0, 0])
        assert np.linalg.norm(c.x - c.origin) <= 0.05 + 1e-9
