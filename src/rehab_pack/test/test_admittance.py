"""Núcleo de admitância cartesiana: regime, retorno elástico e segurança."""
import numpy as np
import pytest

from rehab_pack.admittance import AdmittanceAbort, AdmittanceController


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


def test_unstable_parameters_are_rejected():
    with pytest.raises(ValueError):
        AdmittanceController(mass_kg=0.5, damping_ns_m=100, dt=0.030)
    with pytest.raises(ValueError):
        AdmittanceController(mass_kg=1.0, stiffness_n_m=5000, dt=0.030)
