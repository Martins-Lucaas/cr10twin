"""Testes de touch_pack.imu_math — a conta do rastreio por IMU."""
import math

import numpy as np
import pytest

from touch_pack.imu_math import (
    LIMB_AXES, cartesian_target, chain_position, exp_rotvec, q_mul, quat_to_R,
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
