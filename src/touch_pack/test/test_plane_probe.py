"""Testes da geometria da calibração dinâmica do ângulo de ataque.

O módulo `plane_probe` é puro de propósito: é ele que decide se o punho pode
girar com a ponteira perto da peça, e essa decisão tem de ser verificável sem
robô. Cada teste aqui corresponde a um requisito da feature ou a uma das
guardas de segurança.
"""
import math

import numpy as np
import pytest

from touch_pack.plane_probe import (
    MIN_PROBE_POINTS,
    PlaneFit,
    Z_UP,
    angle_between_deg,
    attack_dir_from_normal,
    fit_plane,
    probe_pattern,
    probe_ring_from_grid,
    slope_along_deg,
    validate_fit,
)


def _plane_points(offsets_xy, normal, z0=0.5):
    """Pontos EXATAMENTE sobre o plano de normal `normal` que passa por
    (0, 0, z0), amostrado nos offsets XY dados."""
    n = np.asarray(normal, float) / np.linalg.norm(normal)
    pts = []
    for x, y in offsets_xy:
        # z tal que n·(p − c) = 0
        z = z0 - (n[0] * x + n[1] * y) / n[2]
        pts.append([x, y, z])
    return np.asarray(pts)


def _tilted_normal(tilt_deg, azim_deg=0.0):
    """Normal unitária a `tilt_deg` da vertical, girada `azim_deg` em XY."""
    t, a = math.radians(tilt_deg), math.radians(azim_deg)
    return np.array([math.sin(t) * math.cos(a),
                     math.sin(t) * math.sin(a),
                     math.cos(t)])


# ── Padrão de sondagem ────────────────────────────────────────────────

def test_pattern_has_requested_point_count_and_radius():
    pat = probe_pattern(5, 0.015)
    assert pat.shape == (5, 2)
    assert np.allclose(np.linalg.norm(pat, axis=1), 0.015)


def test_pattern_floors_at_geometric_minimum():
    # Dois pontos não definem plano — o padrão sobe para o mínimo, não
    # devolve algo que fit_plane vai recusar depois.
    assert len(probe_pattern(2, 0.015)) == MIN_PROBE_POINTS


def test_pattern_is_well_conditioned_in_both_directions():
    # Polígono regular: as duas extensões no plano são iguais, logo a normal
    # fica igualmente determinada nas duas direções (spread ≈ 1).
    pat = probe_pattern(4, 0.02)
    _u, s, _vt = np.linalg.svd(pat - pat.mean(axis=0), full_matrices=False)
    assert s[1] / s[0] == pytest.approx(1.0, abs=1e-9)


def test_pattern_rejects_nonpositive_radius():
    with pytest.raises(ValueError):
        probe_pattern(4, 0.0)


# ── Anel derivado da grade ────────────────────────────────────────────

def _grid(n_cols, n_rows, step_x, step_y=None):
    """Nós de uma grade regular a partir da origem, em metros."""
    sy = step_x if step_y is None else step_y
    return [(ix * step_x, iy * sy)
            for iy in range(n_rows) for ix in range(n_cols)]


def test_ring_sits_half_a_step_outside_the_grid():
    # 4×4 de 5 mm: vão de 15 mm, anel de 15 + 5 = 20 mm de lado.
    ring = probe_ring_from_grid(_grid(4, 4, 0.005), min_half_extent_m=0.005)
    assert ring.shape == (4, 2)
    assert ring.min(axis=0) == pytest.approx([-0.0025, -0.0025])
    assert ring.max(axis=0) == pytest.approx([0.0175, 0.0175])


def test_ring_never_lands_on_a_grid_node():
    """O motivo de existir o meio passo: sondar um nó o pré-condiciona
    antes da identação que vai medi-lo."""
    nodes = _grid(5, 3, 0.004, 0.006)
    ring = probe_ring_from_grid(nodes, min_half_extent_m=0.005)
    for rx, ry in ring:
        for nx, ny in nodes:
            assert math.hypot(rx - nx, ry - ny) > 1e-6


def test_ring_follows_an_anisotropic_grid():
    ring = probe_ring_from_grid(_grid(4, 2, 0.010, 0.020),
                                min_half_extent_m=0.005)
    # X: vão 30 mm + passo 10 → 40; Y: vão 20 mm + passo 20 → 40.
    lo, hi = ring.min(axis=0), ring.max(axis=0)
    assert (hi - lo) == pytest.approx([0.040, 0.040])


def test_ring_is_well_conditioned_in_both_directions():
    """Mesma exigência do polígono: nenhuma direção do plano fica
    indeterminada — é o que os 4 PRIMEIROS pontos da serpentina (todos numa
    linha) não entregam."""
    ring = probe_ring_from_grid(_grid(4, 4, 0.005), min_half_extent_m=0.005)
    pts = _plane_points(ring, _tilted_normal(10.0, 25.0))
    assert fit_plane(pts).spread > 0.5


def test_ring_measures_the_tilt_of_the_grid_region():
    n_true = _tilted_normal(12.0, 200.0)
    ring = probe_ring_from_grid(_grid(4, 4, 0.005), min_half_extent_m=0.005)
    fit = fit_plane(_plane_points(ring, n_true))
    assert np.allclose(fit.normal, n_true, atol=1e-9)
    assert fit.tilt_deg == pytest.approx(12.0, abs=1e-6)


def test_short_grid_is_refused_so_the_caller_falls_back():
    """Braço de alavanca curto é pior que o polígono, não melhor: 2×2 de
    1 mm dá um anel de 2 mm de lado (braço de 1 mm) contra o piso de 5."""
    with pytest.raises(ValueError):
        probe_ring_from_grid(_grid(2, 2, 0.001), min_half_extent_m=0.005)


def test_single_column_grid_is_refused():
    """Uma coluna só: sem passo em X, o anel degenera numa linha e a normal
    ficaria indeterminada nessa direção."""
    with pytest.raises(ValueError):
        probe_ring_from_grid([(0.0, iy * 0.010) for iy in range(4)],
                             min_half_extent_m=0.005)


def test_ring_refuses_empty_or_non_finite_grid():
    with pytest.raises(ValueError):
        probe_ring_from_grid([], min_half_extent_m=0.005)
    with pytest.raises(ValueError):
        probe_ring_from_grid([(0.0, 0.0), (float('nan'), 0.01)],
                             min_half_extent_m=0.005)


# ── Ajuste do plano ───────────────────────────────────────────────────

def test_three_points_give_the_exact_plane():
    n_true = _tilted_normal(8.0, 30.0)
    pts = _plane_points(probe_pattern(3, 0.015), n_true)
    fit = fit_plane(pts)
    assert np.allclose(fit.normal, n_true, atol=1e-9)
    assert fit.rms_m == pytest.approx(0.0, abs=1e-12)
    assert fit.least_squares is False


def test_tilt_is_measured_against_the_world_vertical():
    for tilt in (0.0, 3.5, 12.0, 19.9):
        pts = _plane_points(probe_pattern(4, 0.015), _tilted_normal(tilt, 45.0))
        assert fit_plane(pts).tilt_deg == pytest.approx(tilt, abs=1e-6)


def test_least_squares_filters_noise_of_extra_points():
    # O requisito de "suavização de leitura": com mais de 3 pontos o ajuste
    # tem de recuperar a normal MELHOR do que o plano exato por 3 pontos
    # ruidosos. Ruído de 20 µm, da ordem do que um toque mecânico deixa.
    rng = np.random.default_rng(20260811)
    n_true = _tilted_normal(10.0, 0.0)
    clean = _plane_points(probe_pattern(8, 0.015), n_true)
    noisy = clean.copy()
    noisy[:, 2] += rng.normal(0.0, 20e-6, size=len(noisy))

    err_ls = angle_between_deg(fit_plane(noisy).normal, n_true)
    err_3 = angle_between_deg(fit_plane(noisy[:3]).normal, n_true)
    assert err_ls < err_3
    assert err_ls < 0.5     # sub-grau com ruído dessa escala


def test_fit_is_orthogonal_not_a_regression_of_z():
    # Regressão de z sobre (x, y) enviesa a normal na direção da própria
    # inclinação. Num plano bem inclinado o ajuste ortogonal tem de cravar
    # a normal exata; este teste falharia com o ajuste ingênuo.
    n_true = _tilted_normal(19.0, 115.0)
    fit = fit_plane(_plane_points(probe_pattern(6, 0.02), n_true))
    assert angle_between_deg(fit.normal, n_true) < 1e-6


def test_normal_is_oriented_towards_the_tool_regardless_of_svd_sign():
    # A SVD não escolhe lado; a normal tem de sair sempre para o lado de
    # onde a ferramenta chega, senão o ataque sairia invertido (para cima).
    pts = _plane_points(probe_pattern(4, 0.015), _tilted_normal(15.0, 200.0))
    for order in (pts, pts[::-1], np.roll(pts, 2, axis=0)):
        assert fit_plane(order).normal[2] > 0.0


def test_centroid_is_the_mean_of_the_contacts():
    pts = _plane_points(probe_pattern(4, 0.015), _tilted_normal(5.0))
    assert np.allclose(fit_plane(pts).centroid, pts.mean(axis=0))


def test_fewer_than_three_points_is_refused():
    with pytest.raises(ValueError):
        fit_plane(np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0]]))


def test_non_finite_contact_is_refused():
    pts = _plane_points(probe_pattern(4, 0.015), Z_UP)
    pts[2, 2] = float('nan')
    with pytest.raises(ValueError):
        fit_plane(pts)


def test_coincident_points_are_refused():
    with pytest.raises(ValueError):
        fit_plane(np.zeros((4, 3)))


def test_collinear_points_show_zero_spread():
    pts = np.array([[0.0, 0.0, 0.5], [0.01, 0.0, 0.5], [0.02, 0.0, 0.5]])
    assert fit_plane(pts).spread == pytest.approx(0.0, abs=1e-9)


# ── Eixo de ataque ────────────────────────────────────────────────────

def test_attack_axis_points_into_the_surface():
    n = _tilted_normal(12.0, 77.0)
    a = attack_dir_from_normal(n)
    assert np.allclose(a, -n)
    assert a[2] < 0.0                       # entra na peça, não sai dela
    assert np.linalg.norm(a) == pytest.approx(1.0)


def test_attack_axis_is_exactly_antiparallel_to_the_measured_normal():
    fit = fit_plane(_plane_points(probe_pattern(4, 0.015),
                                  _tilted_normal(17.0, 250.0)))
    assert angle_between_deg(attack_dir_from_normal(fit.normal),
                             fit.normal) == pytest.approx(180.0, abs=1e-6)


# ── Guardas de segurança ──────────────────────────────────────────────

def _fit(tilt_deg=0.0, rms_m=0.0, spread=1.0, n=4):
    return PlaneFit(normal=_tilted_normal(tilt_deg), centroid=np.zeros(3),
                    rms_m=rms_m, tilt_deg=tilt_deg, spread=spread, n_points=n)


def test_tilt_within_limit_is_accepted():
    ok, motivo = validate_fit(_fit(tilt_deg=19.5), tilt_max_deg=20.0)
    assert ok and motivo == ''


def test_tilt_over_the_safety_limit_is_refused():
    ok, motivo = validate_fit(_fit(tilt_deg=20.5), tilt_max_deg=20.0)
    assert not ok
    assert 'limite de segurança' in motivo


def test_near_collinear_points_are_refused():
    ok, motivo = validate_fit(_fit(spread=0.02))
    assert not ok
    assert 'colineares' in motivo


def test_non_planar_surface_is_refused():
    ok, motivo = validate_fit(_fit(rms_m=2e-3))
    assert not ok
    assert 'RMS' in motivo


def test_tilt_is_checked_before_the_other_guards():
    # Um desvio acima do limite é risco de junta/cisalhamento: tem de ser o
    # motivo reportado mesmo quando o ajuste também está ruim.
    ok, motivo = validate_fit(_fit(tilt_deg=45.0, rms_m=5e-3, spread=0.01))
    assert not ok
    assert 'limite de segurança' in motivo


# ── Rampa do deslize ──────────────────────────────────────────────────

def test_slope_along_slide_direction_matches_the_plane():
    # Plano inclinado 10° com a subida ao longo de +X.
    n = _tilted_normal(10.0, 180.0)          # normal deitada para −X
    assert slope_along_deg(n, (1.0, 0.0)) == pytest.approx(10.0, abs=1e-6)
    assert slope_along_deg(n, (-1.0, 0.0)) == pytest.approx(-10.0, abs=1e-6)
    # Perpendicular à linha de maior declive: plano de nível.
    assert slope_along_deg(n, (0.0, 1.0)) == pytest.approx(0.0, abs=1e-9)


def test_slope_is_zero_on_a_horizontal_plane():
    assert slope_along_deg(Z_UP, (0.6, 0.8)) == pytest.approx(0.0, abs=1e-12)


def test_slope_refuses_a_null_direction():
    with pytest.raises(ValueError):
        slope_along_deg(Z_UP, (0.0, 0.0))


def test_slope_refuses_a_vertical_plane():
    with pytest.raises(ValueError):
        slope_along_deg((1.0, 0.0, 0.0), (1.0, 0.0))
