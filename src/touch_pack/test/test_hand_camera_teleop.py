"""Testes do touch_pack.hand_camera_teleop — o núcleo puro de mapeamento
landmarks → flexão 0–1 (curls_from_landmarks). Não toca cv2/mediapipe.

Os landmarks sintéticos vêm de uma FK 2D de dedos de 3 elos: cada dedo é
uma cadeia que dobra `t` ∈ [0,1] radianos-fração nas juntas. `t=0` é o
dedo estendido; `t=1` é o punho fechado.
"""
import math

import numpy as np
import pytest

from touch_pack.hand_camera_teleop import (
    JOINTS, ARM_KEYS, curls_from_landmarks, wrist_pronation_j6_deg,
    _J6_CENTER_DEG, _INDEX_MCP, _PINKY_MCP, _angle_deg, _lerp01,
)

_UP = -math.pi / 2          # "para cima" na imagem (y cresce para baixo)
_MAX_BEND = math.radians(95)


def _chain(base_xy, base_dir, lengths, t):
    """Cadeia planar: devolve [p0(base), p1, p2, p3]. `t` dobra as juntas."""
    pts = [np.array(base_xy, float)]
    ang = base_dir
    p = np.array(base_xy, float)
    for i, L in enumerate(lengths):
        ang = ang + t * _MAX_BEND * (0.8 if i == 0 else 1.0)
        p = p + L * np.array([math.cos(ang), math.sin(ang)])
        pts.append(p.copy())
    return pts


def _hand(t_long=0.0, t_thumb=0.0, thumb_base_dir=math.radians(-150)):
    """Monta os 21 landmarks (x, y em pixels). `t_long` fecha os 4 dedos
    longos; `t_thumb` fecha o polegar; `thumb_base_dir` gira o polegar
    (mais para −90° ≈ oposição/atravessado na palma)."""
    lm = [None] * 21
    lm[0] = np.array([320.0, 470.0])                 # wrist

    long_fingers = {
        'Index':  (5,  (292.0, 345.0)),
        'Middle': (9,  (320.0, 340.0)),
        'Ring':   (13, (348.0, 346.0)),
        'Little': (17, (374.0, 356.0)),
    }
    for _name, (mcp_idx, mcp_xy) in long_fingers.items():
        chain = _chain(mcp_xy, _UP, (46.0, 30.0, 24.0), t_long)
        for k in range(4):
            lm[mcp_idx + k] = chain[k]

    thumb = _chain((298.0, 430.0), thumb_base_dir, (34.0, 30.0, 26.0), t_thumb)
    for k in range(4):
        lm[1 + k] = thumb[k]                         # 1..4 = CMC, MCP, IP, TIP

    return np.array(lm, dtype=np.float64)


# ── helpers geométricos ───────────────────────────────────────────────

def test_angle_deg_straight_and_right():
    a = np.array([0.0, 1.0])
    b = np.array([0.0, 0.0])
    c = np.array([0.0, -1.0])
    assert _angle_deg(a, b, c) == pytest.approx(180.0, abs=1e-6)
    c2 = np.array([1.0, 0.0])
    assert _angle_deg(a, b, c2) == pytest.approx(90.0, abs=1e-6)


def test_lerp01_saturates():
    assert _lerp01(200, 178, 55) == 0.0
    assert _lerp01(20, 178, 55) == 1.0
    assert _lerp01(116.5, 178, 55) == pytest.approx(0.5, abs=0.02)


# ── mapeamento ───────────────────────────────────────────────────────

def test_returns_all_six_joints_in_unit_range():
    curls = curls_from_landmarks(_hand(0.3, 0.3))
    assert set(curls) == set(JOINTS)
    assert all(0.0 <= v <= 1.0 for v in curls.values())


def test_open_hand_curls_near_zero():
    curls = curls_from_landmarks(_hand(t_long=0.0, t_thumb=0.0))
    for j in ('Index', 'Middle', 'Ring', 'Little'):
        assert curls[j] < 0.15, (j, curls[j])


def test_fist_curls_near_one():
    curls = curls_from_landmarks(_hand(t_long=1.0, t_thumb=1.0))
    for j in ('Index', 'Middle', 'Ring', 'Little'):
        assert curls[j] > 0.8, (j, curls[j])


def test_curl_is_monotonic_in_bend():
    prev = -1.0
    for t in (0.0, 0.25, 0.5, 0.75, 1.0):
        c = curls_from_landmarks(_hand(t_long=t, t_thumb=t))['Middle']
        assert c >= prev - 1e-9, (t, c, prev)
        prev = c


def test_thumb_closes_when_bent():
    open_c = curls_from_landmarks(_hand(t_thumb=0.0))['Thumb']
    closed_c = curls_from_landmarks(_hand(t_thumb=1.0))['Thumb']
    assert closed_c > open_c + 0.2


def test_rotate_tracks_thumb_to_pinky_distance():
    """Rotate (oposição) sobe quando a ponta do polegar se aproxima do
    lado do dedo mínimo. Aqui movemos SÓ o landmark do polegar (4)."""
    base = _hand(t_thumb=0.2)
    pinky_mcp = base[17]
    palm_w = np.linalg.norm(base[5] - base[17])

    apart = base.copy()
    apart[4] = pinky_mcp + np.array([palm_w, 0.0])        # longe do mínimo
    across = base.copy()
    across[4] = pinky_mcp + np.array([0.45 * palm_w, 0.0])  # perto do mínimo

    assert (curls_from_landmarks(across)['Rotate']
            > curls_from_landmarks(apart)['Rotate'] + 0.2)


# ── joint6 do braço ← pronação/supinação do punho (palma ↔ dorso) ────

def _wpalm(theta_deg=0.0):
    """World-landmarks 3D de uma palma plana (só wrist/index_MCP/pinky_MCP),
    girada `theta_deg` em torno do eixo longo (y) = pronação. θ=0 → palma
    de frente p/ a câmera (normal em +z)."""
    base = np.zeros((21, 3))
    base[_INDEX_MCP] = (-0.04, -0.08, 0.0)
    base[_PINKY_MCP] = (0.04, -0.08, 0.0)
    t = math.radians(theta_deg)
    R = np.array([[math.cos(t), 0.0, math.sin(t)],
                  [0.0, 1.0, 0.0],
                  [-math.sin(t), 0.0, math.cos(t)]])
    return (R @ base.T).T


def test_j6_key_name():
    assert ARM_KEYS == ('ArmJ6',)


def test_j6_zero_when_palm_faces_camera():
    assert wrist_pronation_j6_deg(_wpalm(0.0)) == pytest.approx(
        _J6_CENTER_DEG, abs=2.0)


def test_j6_tracks_pronation_sign_and_monotonic():
    prev = -1e9
    seen = {}
    for ang in (-60, -30, 0, 30, 60):
        seen[ang] = wrist_pronation_j6_deg(_wpalm(ang))
        assert seen[ang] >= prev - 1e-9, (ang, seen[ang], prev)
        prev = seen[ang]
    assert abs(seen[60] - _J6_CENTER_DEG) > 20.0
    assert (seen[60] - _J6_CENTER_DEG) == pytest.approx(
        _J6_CENTER_DEG - seen[-60], abs=1e-6)


def test_j6_back_of_hand_saturates_opposite_to_palm():
    # dorso de frente (θ≈180) → satura no extremo oposto ao da palma de perfil.
    back = wrist_pronation_j6_deg(_wpalm(179.0))
    assert abs(back - _J6_CENTER_DEG) == pytest.approx(90.0, abs=1.0)
