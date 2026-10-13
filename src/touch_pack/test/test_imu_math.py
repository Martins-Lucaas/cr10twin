"""Testes de touch_pack.imu_math — a conta do rastreio por IMU."""
import math

import numpy as np
import pytest

from touch_pack.imu_math import (
    LIMB_AXES, AccelIntegrator, cartesian_target, clamp_radius, chain_position, exp_rotvec, q_mul, quat_to_R,
    rot_z, rotvec, step_toward_pose, twist_deg, wrap180,
)


def _q_axis_angle(axis, deg):
    a = math.radians(deg) / 2
    ax = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    return np.array([math.cos(a), *(math.sin(a) * ax)])


def test_quat_to_R_matches_rot_z():
    assert np.allclose(quat_to_R(_q_axis_angle((0, 0, 1), 37.0)), rot_z(37.0))


def test_quat_to_R_is_proper_rotation():
    rng = np.random.default_rng(3)
    for _ in range(50):
        R = quat_to_R(rng.normal(size=4))
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
        assert np.isclose(np.linalg.det(R), 1.0)


@pytest.mark.parametrize('axis', [(1, 0, 0), (0, 1, 0), (0, 0, 1)])
def test_twist_recovers_rotation_about_chosen_axis(axis):
    q0 = _q_axis_angle((0.3, -0.5, 0.8), 70.0)   # pose inicial qualquer
    q = q_mul(q0, _q_axis_angle(axis, 25.0))     # gira 25° no frame do sensor
    assert twist_deg(q0, q, axis) == pytest.approx(25.0, abs=1e-9)


def test_twist_ignores_rotation_about_other_axes():
    q0 = np.array([1.0, 0, 0, 0])
    q = _q_axis_angle((1, 0, 0), 40.0)
    assert twist_deg(q0, q, (0, 0, 1)) == pytest.approx(0.0, abs=1e-9)


def test_twist_sign_independent_of_quaternion_sign():
    q0 = np.array([1.0, 0, 0, 0])
    q = _q_axis_angle((0, 0, 1), 30.0)
    assert wrap180(twist_deg(q0, -q, (0, 0, 1))) == pytest.approx(30.0)


def test_wrap180():
    assert wrap180(190.0) == pytest.approx(-170.0)
    assert wrap180(-190.0) == pytest.approx(170.0)
    assert wrap180(45.0) == pytest.approx(45.0)


def test_chain_position_sums_rotated_segments():
    R1 = np.eye(3)
    R2 = rot_z(90.0)
    p = chain_position([R1, R2], [0.30, 0.25], LIMB_AXES['+X'])
    assert np.allclose(p, [0.30, 0.25, 0.0])


@pytest.mark.parametrize('deg', [0.0, 1e-4, 30.0, 120.0, 179.9, 180.0])
def test_rotvec_exp_roundtrip(deg):
    axis = np.array([0.2, -0.7, 0.4]) / np.linalg.norm([0.2, -0.7, 0.4])
    R = exp_rotvec(axis * math.radians(deg))
    assert np.allclose(exp_rotvec(rotvec(R)), R, atol=1e-6)


def test_cartesian_target_applies_heading_and_scale():
    p0, R0 = np.array([0.4, 0.0, 0.3]), np.eye(3)
    p, R = cartesian_target(p0, R0, [0.1, 0.0, 0.0], np.eye(3),
                            heading_deg=90.0, scale=0.5, box_m=(1, 1, 1))
    assert np.allclose(p, p0 + [0.0, 0.05, 0.0])
    assert np.allclose(R, R0)


def test_cartesian_target_rotation_is_conjugated_by_heading():
    dR = exp_rotvec([math.radians(20.0), 0.0, 0.0])   # 20° em x do mundo humano
    _p, R = cartesian_target(np.zeros(3), np.eye(3), np.zeros(3), dR,
                             heading_deg=90.0)
    # com 90° de heading, o x humano vira o y do robô
    assert np.allclose(rotvec(R), [0.0, math.radians(20.0), 0.0], atol=1e-9)


def test_cartesian_target_clamps_to_box():
    p, _R = cartesian_target(np.zeros(3), np.eye(3), [1.0, -1.0, 0.05],
                             np.eye(3), box_m=(0.2, 0.2, 0.2))
    assert np.allclose(p, [0.2, -0.2, 0.05])


def test_step_toward_pose_limits_linear_and_angular_step():
    p, R = step_toward_pose(np.zeros(3), np.eye(3), [1.0, 0, 0], rot_z(90.0),
                            max_lin_m=0.01, max_ang_rad=math.radians(2.0))
    assert np.linalg.norm(p) == pytest.approx(0.01)
    assert np.linalg.norm(rotvec(R)) == pytest.approx(math.radians(2.0))


def test_step_toward_pose_reaches_close_target():
    p, R = step_toward_pose(np.zeros(3), np.eye(3), [0.001, 0, 0], rot_z(0.5),
                            max_lin_m=0.01, max_ang_rad=math.radians(2.0))
    assert np.allclose(p, [0.001, 0, 0])
    assert np.allclose(R, rot_z(0.5))


# ── AccelIntegrator ──────────────────────────────────────────────────────
G = 9.737
HZ = 100.0


def _run(integ, accel_world, seconds, t0=0.0, R=None, gyro=(0, 0, 0)):
    """Alimenta o integrador a 100 Hz com a força específica do mundo (a + g)."""
    R = np.eye(3) if R is None else R
    n = int(seconds * HZ)
    for i in range(n):
        a = accel_world(i / HZ) if callable(accel_world) else accel_world
        integ.update(t0 + i / HZ, R, R.T @ (np.asarray(a) + [0, 0, G]), gyro)
    return t0 + n / HZ


def test_accel_at_rest_learns_gravity_and_does_not_drift():
    integ = AccelIntegrator()
    assert not integ.ready
    _run(integ, (0, 0, 0), 5.0)
    assert integ.ready
    assert np.allclose(integ.displacement(), 0.0, atol=1e-9)


def test_accel_push_moves_in_the_right_direction():
    """0,5 s a +1 m/s² em x e 0,5 s a −1 m/s²: anda ~0,25 m em +x e para."""
    integ = AccelIntegrator(tau_v=1e9)
    t = _run(integ, (0, 0, 0), 1.0)
    integ.reset()
    t = _run(integ, lambda s: (1.0, 0, 0), 0.5, t0=t)
    t = _run(integ, lambda s: (-1.0, 0, 0), 0.5, t0=t)
    d = integ.displacement()
    assert d[0] == pytest.approx(0.25, abs=0.03)
    assert abs(d[1]) < 1e-6 and abs(d[2]) < 1e-6
    assert np.linalg.norm(integ.v) < 0.05


def test_accel_rotates_sensor_frame_to_world():
    """Sensor deitado a 90° em torno de y: o mesmo empurrão em x do mundo
    aparece no eixo z do sensor e ainda assim sai em +x."""
    R = exp_rotvec([0, math.pi / 2, 0])
    integ = AccelIntegrator(tau_v=1e9)
    t = _run(integ, (0, 0, 0), 1.0, R=R)
    integ.reset()
    t = _run(integ, lambda s: (1.0, 0, 0), 0.5, t0=t, R=R)
    t = _run(integ, lambda s: (-1.0, 0, 0), 0.5, t0=t, R=R)
    assert integ.displacement()[0] == pytest.approx(0.25, abs=0.03)


def test_accel_zupt_kills_velocity_error_when_hand_stops():
    """Viés persistente de 0,3 m/s² enquanto ELE mexe; ao parar, a velocidade
    espúria é zerada e a posição para de andar."""
    integ = AccelIntegrator(tau_v=1e9)
    t = _run(integ, (0, 0, 0), 1.0)
    t = _run(integ, (0.6, 0, 0), 0.3, t0=t, gyro=(0.5, 0, 0))   # movimento
    t = _run(integ, (0, 0, 0), 1.0, t0=t)                       # parou
    p = integ.displacement().copy()
    _run(integ, (0, 0, 0), 3.0, t0=t)
    assert np.allclose(integ.displacement(), p, atol=1e-9)
    assert np.allclose(integ.v, 0.0)


def test_accel_gap_in_samples_zeroes_velocity():
    integ = AccelIntegrator(tau_v=1e9)
    t = _run(integ, (0, 0, 0), 1.0)
    t = _run(integ, (2.0, 0, 0), 0.2, t0=t, gyro=(0.5, 0, 0))
    assert np.linalg.norm(integ.v) > 0.1
    integ.update(t + 0.5, np.eye(3), (0, 0, G), (0, 0, 0))
    assert np.allclose(integ.v, 0.0)


def test_accel_no_seed_while_rotating():
    integ = AccelIntegrator()
    _run(integ, (0, 0, 0), 3.0, gyro=(1.0, 0, 0))
    assert not integ.ready


def test_accel_velocity_is_capped():
    integ = AccelIntegrator(tau_v=1e9, max_v=1.0)
    t = _run(integ, (0, 0, 0), 1.0)
    _run(integ, (50.0, 0, 0), 2.0, t0=t, gyro=(0.5, 0, 0))
    assert np.linalg.norm(integ.v) <= 1.0 + 1e-9


# ── clamp_radius ─────────────────────────────────────────────────────────
def test_clamp_radius_leaves_outside_points_alone():
    p = np.array([0.3, -0.4, 0.7])
    assert np.array_equal(clamp_radius(p, 0.25), p)


def test_clamp_radius_pushes_outward_keeping_direction_and_z():
    out = clamp_radius([0.0, -0.1, 0.8], 0.25)
    assert np.allclose(out, [0.0, -0.25, 0.8])
    out = clamp_radius([0.06, 0.08, 0.5], 0.25)      # r = 0,10 → 0,25 mesma direção
    assert np.allclose(out[:2], [0.15, 0.20]) and out[2] == 0.5


def test_clamp_radius_on_the_axis_uses_fallback_direction():
    out = clamp_radius([0.0, 0.0, 0.6], 0.25, fallback_dir=(0.0, 2.0))
    assert np.allclose(out, [0.0, 0.25, 0.6])
