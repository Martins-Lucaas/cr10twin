"""Testes do touch_pack.manip3d — câmera, projeção e IK diferencial."""
import math

import numpy as np
import pytest

from touch_pack.kinematics import (
    JOINT_MAX, JOINT_MIN, T_TOUCH_TOOL_ATTACH, forward_kinematics,
)
from touch_pack.manip3d import (
    Camera, ik_step, rpy_deg, skeleton_points, tcp_pose,
)

_RNG = np.random.default_rng(7)
_W, _H = 800, 600

# Pose de trabalho típica da célula: braço "apontando" para a mesa, longe
# de singularidade de punho e bem dentro do envelope.
_Q_POINTING = np.deg2rad([0.0, -30.0, -90.0, 0.0, 60.0, 0.0])


# ── Câmera / projeção ──────────────────────────────────────────────────

def test_basis_is_orthonormal_right_handed():
    cam = Camera()
    r, u, f = cam.basis()
    for v in (r, u, f):
        assert np.isclose(np.linalg.norm(v), 1.0)
    assert np.isclose(r @ u, 0.0, atol=1e-12)
    assert np.isclose(r @ f, 0.0, atol=1e-12)
    assert np.isclose(u @ f, 0.0, atol=1e-12)
    # up × right = −forward numa base destra com forward "para dentro".
    assert np.allclose(np.cross(r, u), -f, atol=1e-12)


def test_target_projects_to_screen_center():
    cam = Camera()
    uv, z = cam.project(cam.target[None, :], _W, _H)
    assert np.allclose(uv[0], [_W / 2, _H / 2], atol=1e-9)
    assert np.isclose(z[0], cam.dist)


def test_point_behind_camera_has_negative_depth():
    """O caller descarta profundidade ≤ near — é o clipping da viewport."""
    cam = Camera()
    behind = cam.eye() + (cam.eye() - cam.target)
    _uv, z = cam.project(behind[None, :], _W, _H)
    assert z[0] < 0.0


def test_unproject_delta_is_inverse_of_project():
    """1 px de mouse ↔ a mesma distância em metros: é essa identidade que
    faz o TCP acompanhar o cursor sem deriva durante o arrasto."""
    cam = Camera()
    p0 = np.array([0.4, 0.1, 0.6])
    uv0, z0 = cam.project(p0[None, :], _W, _H)
    du, dv = 37.0, -21.0
    p1 = p0 + cam.unproject_delta(du, dv, float(z0[0]), _H)
    uv1, z1 = cam.project(p1[None, :], _W, _H)
    # O deslocamento está no plano da câmera → mesma profundidade …
    assert np.isclose(z1[0], z0[0], atol=1e-9)
    # … e reprojeta exatamente sobre o deslocamento de mouse pedido.
    assert np.allclose(uv1[0] - uv0[0], [du, dv], atol=1e-6)


def test_orbit_keeps_target_and_distance():
    cam = Camera()
    t0, d0 = cam.target.copy(), cam.dist
    cam.orbit(0.9, -0.4)
    assert np.allclose(cam.target, t0)
    assert np.isclose(cam.dist, d0)
    assert np.isclose(np.linalg.norm(cam.eye() - cam.target), d0)


def test_orbit_elevation_is_clamped():
    cam = Camera()
    for _ in range(50):
        cam.orbit(0.0, 0.5)
    assert cam.el <= Camera.EL_LIM + 1e-12


def test_zoom_is_clamped():
    cam = Camera()
    for _ in range(50):
        cam.zoom(0.5)
    assert cam.dist >= Camera.DIST_MIN - 1e-12
    for _ in range(80):
        cam.zoom(2.0)
    assert cam.dist <= Camera.DIST_MAX + 1e-12


def test_pan_moves_target_on_camera_plane():
    cam = Camera()
    _r, _u, f = cam.basis()
    t0 = cam.target.copy()
    cam.pan(40.0, 25.0, _H)
    delta = cam.target - t0
    assert np.linalg.norm(delta) > 1e-6
    assert np.isclose(delta @ f, 0.0, atol=1e-9)   # sem componente em profundidade


# ── Geometria desenhada ────────────────────────────────────────────────

def test_skeleton_starts_at_base_and_ends_at_tcp():
    q = _RNG.uniform(-1.2, 1.2, 6)
    pts = skeleton_points(q, T_TOUCH_TOOL_ATTACH)
    assert pts.shape == (8, 3)
    assert np.allclose(pts[0], np.zeros(3))
    assert np.allclose(
        pts[-1], forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)[:3, 3])


def test_skeleton_segment_lengths_are_pose_invariant():
    """Corpo rígido: girar as juntas não estica os elos."""
    lens = []
    for _ in range(5):
        pts = skeleton_points(_RNG.uniform(-2.0, 2.0, 6), T_TOUCH_TOOL_ATTACH)
        lens.append(np.linalg.norm(np.diff(pts, axis=0), axis=1))
    for l in lens[1:]:
        assert np.allclose(l, lens[0], atol=1e-12)


def test_rpy_deg_matches_pointing_pose():
    """Na pose apontando para a mesa o eixo z do TCP olha para −Z mundo,
    o que em RPY fixo XYZ dá pitch ≈ ±90° … aqui checamos pela matriz."""
    R = tcp_pose(_Q_POINTING, T_TOUCH_TOOL_ATTACH)[:3, :3]
    roll, pitch, yaw = rpy_deg(R)
    cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    R_back = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])
    assert np.allclose(R_back, R, atol=1e-9)


# ── IK diferencial ─────────────────────────────────────────────────────

def _tcp(q):
    return forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]


def test_ik_step_converges_to_a_nearby_target():
    """Um alvo a 30 mm é alcançado dentro de poucos ticks — é o regime
    normal do arrasto (o mouse anda poucos px por frame)."""
    q = _Q_POINTING.copy()
    target = _tcp(q) + np.array([0.03, -0.02, 0.01])
    for _ in range(10):
        res = ik_step(q, target, T_end=T_TOUCH_TOOL_ATTACH)
        q = res.q
    assert res.pos_err_m < 1e-3
    assert not res.singular


def test_ik_step_respects_max_linear_step():
    """Alvo absurdamente longe: o TCP anda no máximo max_lin_m por iteração,
    então um tick não pode teletransportar o braço."""
    q = _Q_POINTING.copy()
    p0 = _tcp(q)
    target = p0 + np.array([5.0, 0.0, 0.0])
    res = ik_step(q, target, T_end=T_TOUCH_TOOL_ATTACH,
                  max_lin_m=0.01, iters=4)
    travel = np.linalg.norm(_tcp(res.q) - p0)
    assert travel <= 4 * 0.01 + 1e-6


def test_ik_step_respects_max_joint_step():
    q = _Q_POINTING.copy()
    target = _tcp(q) + np.array([0.5, 0.5, -0.3])
    res = ik_step(q, target, T_end=T_TOUCH_TOOL_ATTACH,
                  max_dq=0.02, iters=5)
    assert np.all(np.abs(res.q - _Q_POINTING) <= 5 * 0.02 + 1e-9)


def test_ik_step_never_leaves_joint_limits():
    """Mesmo perseguindo um alvo inalcançável a pose devolvida é válida —
    a viewport publica isso direto no controlador."""
    q = _Q_POINTING.copy()
    target = np.array([3.0, 3.0, 2.5])
    for _ in range(60):
        q = ik_step(q, target, T_end=T_TOUCH_TOOL_ATTACH).q
    assert np.all(q >= JOINT_MIN - 1e-9)
    assert np.all(q <= JOINT_MAX + 1e-9)


def test_ik_step_flags_unreachable_target():
    q = _Q_POINTING.copy()
    target = np.array([3.0, 0.0, 0.5])          # muito além dos 1,375 m
    for _ in range(80):
        res = ik_step(q, target, T_end=T_TOUCH_TOOL_ATTACH)
        q = res.q
    assert res.pos_err_m > 0.5                  # a viewport mostra como "lag"


def test_ik_step_orientation_lock_preserves_attitude():
    """Com a trava ligada o arrasto move o PONTO: a atitude da ferramenta
    volta para onde estava, dentro de uma tolerância de poucos graus."""
    q = _Q_POINTING.copy()
    R_lock = tcp_pose(q, T_TOUCH_TOOL_ATTACH)[:3, :3].copy()
    target = _tcp(q) + np.array([0.06, 0.04, -0.03])
    for _ in range(25):
        res = ik_step(q, target, T_end=T_TOUCH_TOOL_ATTACH, R_lock=R_lock)
        q = res.q
    R_end = tcp_pose(q, T_TOUCH_TOOL_ATTACH)[:3, :3]
    ang = math.degrees(math.acos(
        min(1.0, max(-1.0, 0.5 * (np.trace(R_lock.T @ R_end) - 1.0)))))
    assert res.pos_err_m < 2e-3
    assert ang < 2.0


def test_ik_step_without_lock_lets_the_wrist_rotate():
    """Sem trava a IK usa os 6 graus de liberdade só para a posição —
    chega mais perto/mais rápido, sem se importar com a atitude."""
    q = _Q_POINTING.copy()
    target = _tcp(q) + np.array([0.10, 0.08, -0.05])
    for _ in range(20):
        res = ik_step(q, target, T_end=T_TOUCH_TOOL_ATTACH, R_lock=None)
        q = res.q
    assert res.pos_err_m < 1e-3


def test_ik_step_is_continuous_no_configuration_jumps():
    """O ponto do desenho: perseguindo um alvo que caminha, a pose nunca
    salta de ramo (o que faria o braço "estalar" no meio do arrasto)."""
    q = _Q_POINTING.copy()
    p = _tcp(q)
    max_jump = 0.0
    for k in range(60):
        target = p + np.array([0.004 * k, 0.002 * k, -0.001 * k])
        q_prev = q.copy()
        q = ik_step(q, target, T_end=T_TOUCH_TOOL_ATTACH).q
        max_jump = max(max_jump, float(np.max(np.abs(q - q_prev))))
    assert max_jump <= 6 * 0.06 + 1e-9        # iters × max_dq


def test_ik_step_is_a_no_op_when_already_on_target():
    q = _Q_POINTING.copy()
    res = ik_step(q, _tcp(q), T_end=T_TOUCH_TOOL_ATTACH)
    assert np.allclose(res.q, q, atol=1e-9)
    assert res.pos_err_m < 1e-9


@pytest.mark.parametrize('delta', [
    np.array([0.05, 0.0, 0.0]),
    np.array([0.0, -0.05, 0.0]),
    np.array([0.0, 0.0, 0.05]),
    np.array([-0.04, 0.03, -0.02]),
])
def test_ik_step_reaches_targets_in_every_direction(delta):
    q = _Q_POINTING.copy()
    target = _tcp(q) + delta
    for _ in range(20):
        res = ik_step(q, target, T_end=T_TOUCH_TOOL_ATTACH)
        q = res.q
    assert res.pos_err_m < 1e-3
