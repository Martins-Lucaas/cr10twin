"""Testes do touch_pack.kinematics — FK, Jacobiano, solver de pulso e IK."""
import math

import numpy as np
import pytest

from touch_pack.kinematics import (
    forward_kinematics, fk_partial, jacobian, inverse_kinematics,
    urdf_to_dobot, dobot_to_urdf,
    T_HAND_ATTACH, T_TOUCH_TOOL_ATTACH, JOINT_MIN, JOINT_MAX,
)

_RNG = np.random.default_rng(42)
_PI2 = math.pi / 2


# ── FK ─────────────────────────────────────────────────────────────────

def test_fk_pointing_pose_is_perpendicular():
    """Na pose POINTING (0,0,-90,0,90,0) o eixo z do flange aponta para
    baixo (−Z mundo) — é a pose de palpação perpendicular à mesa."""
    q = np.deg2rad([0, 0, -90, 0, 90, 0]).astype(float)
    T = forward_kinematics(q, include_hand=False)
    assert np.allclose(T[:3, 2], [0.0, 0.0, -1.0], atol=1e-12)


def test_fk_tool_offsets_along_approach():
    """T_end desloca o TCP pela translação do attach expressa no frame do
    flange (T_fl_R · att_t). Os dois attachs são AXIAIS, então isso reduz a
    att_z·z_flange. (A pilha da FA7155 é centrada no flange, e
    T_TOUCH_TOOL_ATTACH não tem componente em y.)"""
    q = _RNG.uniform(-1.5, 1.5, 6)
    T_fl = forward_kinematics(q, include_hand=False)
    for att in (T_HAND_ATTACH, T_TOUCH_TOOL_ATTACH):
        T_tcp = forward_kinematics(q, T_end=att)
        expected = T_fl[:3, 3] + T_fl[:3, :3] @ att[:3, 3]
        assert np.allclose(T_tcp[:3, 3], expected, atol=1e-12)


# ── Jacobiano ──────────────────────────────────────────────────────────

def test_jacobian_translational_matches_finite_differences():
    eps = 1e-7
    for _ in range(10):
        q = _RNG.uniform(-2.0, 2.0, 6)
        J = jacobian(q)[:3, :]
        for i in range(6):
            dq = np.zeros(6); dq[i] = eps
            p_plus  = forward_kinematics(q + dq)[:3, 3]
            p_minus = forward_kinematics(q - dq)[:3, 3]
            J_num = (p_plus - p_minus) / (2 * eps)
            assert np.allclose(J[:, i], J_num, atol=1e-5), f'coluna {i}'


# ── Solver de pulso perpendicular (botão "TCP ⊥ Mesa" da GUI) ──────────

def _solve_perpendicular_wrist(q):
    """Réplica da solução analítica usada em palpation_gui
    _solve_tcp_perpendicular: dado q1-q3, retorna os 2 ramos (q4, q5)."""
    R03 = fk_partial(q, 3)[:3, :3]
    v = R03.T @ np.array([0.0, 0.0, -1.0])
    s5 = math.hypot(float(v[0]), float(v[1]))
    if s5 < 1e-9:
        return [(float(q[3]), 0.0)] if v[2] > 0 else []
    sols = []
    for sgn in (+1.0, -1.0):
        q5 = math.atan2(sgn * s5, float(v[2]))
        q4 = math.atan2(-sgn * float(v[1]), -sgn * float(v[0])) + _PI2
        q4 = (q4 + math.pi) % (2 * math.pi) - math.pi
        sols.append((q4, q5))
    return sols


def test_perpendicular_wrist_both_branches_exact():
    for _ in range(100):
        q = _RNG.uniform(-2.5, 2.5, 6)
        for q4, q5 in _solve_perpendicular_wrist(q):
            qn = q.copy(); qn[3], qn[4] = q4, q5
            z = forward_kinematics(qn, include_hand=False)[:3, 2]
            assert np.allclose(z, [0, 0, -1], atol=1e-9)


def test_perpendicular_wrist_ur_closed_form():
    """Para cinemática tipo UR: j4 = −90° − (q2+q3), j5 = ±90°."""
    q = np.deg2rad([30, -40, -70, 10, 50, 20]).astype(float)
    sols = _solve_perpendicular_wrist(q)
    best = min(sols, key=lambda s: abs(s[0] - q[3]) + abs(s[1] - q[4]))
    assert math.degrees(best[0]) == pytest.approx(20.0, abs=1e-6)
    assert math.degrees(best[1]) == pytest.approx(90.0, abs=1e-6)


# ── IK ─────────────────────────────────────────────────────────────────

@pytest.mark.parametrize('att', [None, T_TOUCH_TOOL_ATTACH],
                         ids=['hand', 'touch_tool'])
def test_ik_roundtrip(att):
    """FK(q0) → IK → FK reproduz a posição e o eixo de abordagem."""
    q0 = np.deg2rad([15, -20, -80, 5, 80, 0]).astype(float)
    T0 = forward_kinematics(q0, T_end=att) if att is not None \
        else forward_kinematics(q0)
    p, approach = T0[:3, 3], T0[:3, 2]

    q, ok = inverse_kinematics(p, approach, q_seed=q0, T_end=att)
    assert ok
    T = forward_kinematics(q, T_end=att) if att is not None \
        else forward_kinematics(q)
    assert np.linalg.norm(T[:3, 3] - p) < 1e-2          # posição ≤ 10 mm
    assert float(T[:3, 2] @ approach) > 0.99            # eixo alinhado
    assert np.all(q >= JOINT_MIN - 1e-9)
    assert np.all(q <= JOINT_MAX + 1e-9)


def test_ik_nunca_devolve_ok_fora_dos_limites():
    """Com e sem semente: solução aceita cabe nos limites e atinge o alvo."""
    rng = np.random.default_rng(0)
    n_ok = 0
    for k in range(400):
        q0 = rng.uniform(JOINT_MIN * 0.9, JOINT_MAX * 0.9)
        T0 = forward_kinematics(q0, T_end=T_HAND_ATTACH)
        seed = None if k % 2 else q0 + rng.normal(0, 0.3, 6)
        q, ok = inverse_kinematics(T0[:3, 3], T0[:3, 2], q_seed=seed,
                                   T_end=T_HAND_ATTACH)
        if not ok:
            continue
        n_ok += 1
        T = forward_kinematics(q, T_end=T_HAND_ATTACH)
        assert np.all(q >= JOINT_MIN - 1e-9) and np.all(q <= JOINT_MAX + 1e-9), k
        assert np.linalg.norm(T[:3, 3] - T0[:3, 3]) < 1e-2, k
        assert T[:3, 2] @ T0[:3, 2] > 0.96, k      # tolerância de 0,25 rad da IK
    assert n_ok >= 370     # a IK de antes aceitava 380, 23 delas fora do limite


def test_pulso_nao_depende_de_arredondamento(monkeypatch):
    """Na pose que oscilava no CI os dois ramos do pulso empatam (~1e-13): um
    ruído dessa ordem no erro de orientação não pode trocar o ramo."""
    from touch_pack import kinematics as K
    q0 = np.deg2rad([15, -20, -80, 5, 80, 0]).astype(float)
    R = forward_kinematics(q0, T_end=T_HAND_ATTACH)[:3, :3]
    rot_error, saidas = K._rot_error, []
    for favorecido in (0, 1):
        n = iter(range(2))
        monkeypatch.setattr(K, '_rot_error', lambda a, b: rot_error(a, b) + (
            0.0 if next(n) == favorecido else 1e-12))
        saidas.append(K._set_wrist(q0.copy(), R, T_end=T_HAND_ATTACH))
    assert np.allclose(saidas[0], saidas[1])
    assert np.allclose(saidas[0], q0, atol=1e-6)     # o ramo da pose atual


# ── Conversão URDF ↔ DOBOT ─────────────────────────────────────────────

def test_urdf_dobot_roundtrip():
    q = _RNG.uniform(-3.0, 3.0, 6)
    assert np.allclose(dobot_to_urdf(urdf_to_dobot(q)), q, atol=1e-12)
    # joint1 tem sinal invertido; demais idênticas
    qd = urdf_to_dobot(q)
    assert qd[0] == pytest.approx(-q[0])
    assert np.allclose(qd[1:], q[1:])
