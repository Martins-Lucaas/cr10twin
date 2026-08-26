"""Testes da fase CALIBRATING no tactile_explorer — saturação dos
parâmetros, roteamento do desfecho e a garantia de que, DESLIGADA, a
calibração não toca em nada.

A geometria em si mora em test_plane_probe.py; aqui só o que depende do nó.
"""
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')


@pytest.fixture(scope='module')
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def node(_ros):
    from touch_pack.tactile_explorer import TactileExplorer
    n = TactileExplorer()
    n._q_now = lambda: np.deg2rad([0, 0, -90, 0, 90, 0]).astype(float)
    n._stream_q = lambda *a, **k: None
    n._speed_factor_pct = 10.0
    with n._params_lock:
        n._target_force_n = 2.0
        n._target_depth_mm = 20.0
    yield n
    n.destroy_node()


def _set(node, **params):
    node.set_parameters([
        rclpy.parameter.Parameter(
            k,
            (rclpy.parameter.Parameter.Type.BOOL if isinstance(v, bool)
             else rclpy.parameter.Parameter.Type.INTEGER if isinstance(v, int)
             else rclpy.parameter.Parameter.Type.DOUBLE),
            v)
        for k, v in params.items()])


# ── Desligada: o caminho de sempre não pode mudar ─────────────────────

def test_disabled_by_default(node):
    assert node._align_params() is None


def test_disabled_calibration_moves_nothing(node):
    """A garantia de não-regressão: com o parâmetro em False a fase sai
    'ok' sem marcar fase, sem sondar e sem tocar no eixo de ataque."""
    touched = []
    node._set_phase = lambda p: touched.append(p)
    node._probe_plane = lambda *a, **k: pytest.fail('sondou com a calibração desligada')
    assert node._phase_calibrate_attack() == 'ok'
    assert touched == []
    assert node._attack_dir is None


def test_calibrate_or_abort_is_transparent_when_disabled(node):
    assert node._calibrate_or_abort() is True


# ── Saturação dos parâmetros ──────────────────────────────────────────

def test_point_count_floors_at_the_geometric_minimum(node):
    from touch_pack.plane_probe import MIN_PROBE_POINTS
    _set(node, probe_align_enable=True, probe_align_points=1)
    assert node._align_params()['n'] == MIN_PROBE_POINTS


def test_point_count_is_capped(node):
    from touch_pack.tactile_explorer import _ALIGN_MAX_POINTS
    _set(node, probe_align_enable=True, probe_align_points=999)
    assert node._align_params()['n'] == _ALIGN_MAX_POINTS


def test_tilt_limit_cannot_exceed_the_hard_ceiling(node):
    """Um parâmetro de 89° autorizaria uma rotação de punho que a ponteira
    não sobrevive — o teto duro não é configurável."""
    from touch_pack.tactile_explorer import _ALIGN_TILT_HARD_MAX_DEG
    _set(node, probe_align_enable=True, probe_align_tilt_max_deg=89.0)
    assert node._align_params()['tilt_max_deg'] == _ALIGN_TILT_HARD_MAX_DEG


def test_radius_and_retract_are_clamped(node):
    from touch_pack.constants import (
        PROBE_ALIGN_RADIUS_MM_MAX, PROBE_ALIGN_RETRACT_MM_MIN)
    _set(node, probe_align_enable=True,
         probe_align_radius_mm=500.0, probe_align_retract_mm=0.1)
    cfg = node._align_params()
    assert cfg['radius_m'] == pytest.approx(PROBE_ALIGN_RADIUS_MM_MAX * 1e-3)
    assert cfg['retract_m'] == pytest.approx(PROBE_ALIGN_RETRACT_MM_MIN * 1e-3)


# ── Precedência mensagem (GUI) × parâmetros ROS ───────────────────────

def test_message_overrides_ros_params(node):
    """Mesmo contrato de force_mod_shape: com o campo preenchido, a GUI
    manda e os parâmetros ROS ficam de reserva."""
    _set(node, probe_align_enable=False)
    with node._params_lock:
        node._align_from_msg = True
        node._align_on = True
        node._align_msg = {'points': 6, 'radius_mm': 20.0, 'force_n': 0.8,
                           'retract_mm': 30.0, 'tilt_max_deg': 12.0}
    cfg = node._align_params()
    assert cfg is not None and cfg['src'] == 'GUI'
    assert cfg['n'] == 6
    assert cfg['radius_m'] == pytest.approx(0.020)
    assert cfg['tilt_max_deg'] == pytest.approx(12.0)


def test_message_off_beats_enabled_ros_param(node):
    _set(node, probe_align_enable=True)
    with node._params_lock:
        node._align_from_msg = True
        node._align_on = False
    assert node._align_params() is None


def test_zero_fields_from_message_fall_back_to_defaults(node):
    """0 no campo numérico = 'usar o default', como nos demais campos da
    PalpationStart — não um raio de 0 mm."""
    from touch_pack.constants import (
        PROBE_ALIGN_POINTS_DEFAULT, PROBE_ALIGN_RADIUS_MM_DEFAULT)
    with node._params_lock:
        node._align_from_msg = True
        node._align_on = True
        node._align_msg = {'points': 0, 'radius_mm': 0.0, 'force_n': 0.0,
                           'retract_mm': 0.0, 'tilt_max_deg': 0.0}
    cfg = node._align_params()
    assert cfg['n'] == PROBE_ALIGN_POINTS_DEFAULT
    assert cfg['radius_m'] == pytest.approx(
        PROBE_ALIGN_RADIUS_MM_DEFAULT * 1e-3)


def test_probe_force_stays_above_the_contact_threshold(node):
    from touch_pack.tactile_explorer import _CONTACT_ON_N
    _set(node, probe_align_enable=True, probe_align_force_n=0.0)
    assert node._align_params()['force_n'] > _CONTACT_ON_N


# ── Onde a sonda encosta ──────────────────────────────────────────────

def _cfg(n=4, radius_mm=15.0):
    """Só o que _align_offsets consome de _align_params()."""
    return {'n': n, 'radius_m': radius_mm * 1e-3}


def _wps(n_cols, n_rows, step_m):
    """Waypoints como o explorer os recebe: grade regular SEM a origem."""
    nodes = [(ix * step_m, iy * step_m)
             for iy in range(n_rows) for ix in range(n_cols)]
    return np.asarray(nodes[1:], dtype=float)


def test_probe_pattern_is_the_polygon_outside_matrix(node):
    """Sem grade não há de onde derivar anel: os outros modos continuam
    sondando o polígono do raio, com a contagem de pontos da GUI."""
    node._mode = 'SINGLE'
    node._matrix_wps = _wps(4, 4, 0.005)
    off, label = node._align_offsets(_cfg(n=5))
    assert off.shape == (5, 2)
    assert np.allclose(np.linalg.norm(off, axis=1), 0.015)
    assert 'raio' in label


def test_matrix_probes_a_ring_derived_from_the_grid(node):
    """A grade 4×4 de 5 mm (vão 15 mm) vira um anel de 20 mm de lado, meio
    passo para fora — em vez do círculo de 15 mm de RAIO, que tocaria a
    30 mm de ponta a ponta, fora da amostra."""
    node._mode = 'MATRIX_MAP'
    node._matrix_wps = _wps(4, 4, 0.005)
    off, label = node._align_offsets(_cfg())
    assert off.shape == (4, 2)
    assert off.min(axis=0) == pytest.approx([-0.0025, -0.0025])
    assert off.max(axis=0) == pytest.approx([0.0175, 0.0175])
    assert 'grade' in label


def test_ring_never_lands_on_a_waypoint(node):
    node._mode = 'MATRIX_MAP'
    wps = _wps(4, 4, 0.005)
    node._matrix_wps = wps
    off, _label = node._align_offsets(_cfg())
    nodes = np.vstack([np.zeros((1, 2)), wps])
    for o in off:
        assert float(np.min(np.linalg.norm(nodes - o, axis=1))) > 1e-6


def test_short_grid_falls_back_to_the_polygon(node):
    """Grade de 1 mm de passo: o anel teria 1 mm de braço e mediria menos
    inclinação que o ruído dos toques — cai no raio da GUI."""
    node._mode = 'MATRIX_MAP'
    node._matrix_wps = _wps(2, 2, 0.001)
    off, label = node._align_offsets(_cfg())
    assert np.allclose(np.linalg.norm(off, axis=1), 0.015)
    assert 'raio' in label


def test_matrix_without_grid_falls_back_to_the_polygon(node):
    node._mode = 'MATRIX_MAP'
    node._matrix_wps = np.zeros((0, 2))
    off, _label = node._align_offsets(_cfg())
    assert np.allclose(np.linalg.norm(off, axis=1), 0.015)


# ── Plano do deslize entregue pela medição ────────────────────────────

def _tilted_plane_pts(tilt_deg, axis='x', z0=0.5):
    """4 pontos exatos sobre um plano que sobe `tilt_deg` ao longo de `axis`."""
    t = np.tan(np.deg2rad(tilt_deg))
    out = []
    for x, y in ((0.0, 0.0), (0.02, 0.0), (0.0, 0.02), (0.02, 0.02)):
        rise = x * t if axis == 'x' else y * t
        out.append([x, y, z0 + rise])
    return np.asarray(out)


def test_measured_normal_reaches_the_slide_plane(node):
    from touch_pack.plane_probe import fit_plane
    with node._params_lock:
        node._mode = 'SLIDE'
        node._slide_dir_vec = np.array([1.0, 0.0])
        node._slide_plane_n = None
    fit = fit_plane(_tilted_plane_pts(10.0))
    node._align_set_slide_plane(fit)
    assert np.allclose(node._slide_plane_n, fit.normal)


def test_declared_slope_is_not_overwritten(node):
    """A declaração do usuário fica de reserva — a medição entra por um
    canal próprio, não por cima dela."""
    from touch_pack.plane_probe import fit_plane
    with node._params_lock:
        node._mode = 'SLIDE'
        node._slide_dir_vec = np.array([1.0, 0.0])
        node._slide_slope_deg = 3.0
    node._align_set_slide_plane(fit_plane(_tilted_plane_pts(10.0)))
    assert node._slide_slope_deg == 3.0


def test_slide_plane_untouched_outside_slide_mode(node):
    from touch_pack.plane_probe import fit_plane
    with node._params_lock:
        node._mode = 'TOUCH'
        node._slide_dir_vec = np.array([1.0, 0.0])
        node._slide_plane_n = None
    node._align_set_slide_plane(fit_plane(_tilted_plane_pts(10.0)))
    assert node._slide_plane_n is None


# ── Base ortonormal do deslize ────────────────────────────────────────

def test_frame_is_horizontal_without_measurement_or_declaration(node):
    with node._params_lock:
        node._slide_plane_n = None
        node._slide_slope_deg = 0.0
    u, w, n, src = node._slide_frame(np.array([1.0, 0.0]))
    assert np.allclose(u, [1.0, 0.0, 0.0])
    assert np.allclose(n, [0.0, 0.0, 1.0])
    assert np.allclose(w, [0.0, 1.0, 0.0])
    assert src == 'horizontal'


def test_frame_advance_lies_in_the_measured_plane(node):
    from touch_pack.plane_probe import fit_plane
    fit = fit_plane(_tilted_plane_pts(15.0))
    with node._params_lock:
        node._slide_plane_n = np.asarray(fit.normal).copy()
    u, w, n, src = node._slide_frame(np.array([1.0, 0.0]))
    # A reta do curso e a transversal estão AMBAS no plano.
    assert float(u @ n) == pytest.approx(0.0, abs=1e-12)
    assert float(w @ n) == pytest.approx(0.0, abs=1e-12)
    assert float(u @ w) == pytest.approx(0.0, abs=1e-12)
    for v in (u, w, n):
        assert np.linalg.norm(v) == pytest.approx(1.0)
    assert src == 'medido'


def test_frame_advance_climbs_the_measured_tilt(node):
    """O avanço sobe: é isso que faz o curso ser distância sobre a
    SUPERFÍCIE em vez da projeção horizontal dela."""
    from touch_pack.plane_probe import fit_plane
    with node._params_lock:
        node._slide_plane_n = np.asarray(
            fit_plane(_tilted_plane_pts(15.0)).normal).copy()
    u, _w, _n, _src = node._slide_frame(np.array([1.0, 0.0]))
    assert np.degrees(np.arctan2(u[2], u[0])) == pytest.approx(15.0, abs=1e-6)


def test_frame_falls_back_to_the_declared_slope(node):
    with node._params_lock:
        node._slide_plane_n = None
        node._slide_slope_deg = 7.0
    u, _w, n, src = node._slide_frame(np.array([1.0, 0.0]))
    assert np.degrees(np.arctan2(u[2], u[0])) == pytest.approx(7.0, abs=1e-9)
    assert float(u @ n) == pytest.approx(0.0, abs=1e-12)
    assert 'declarado' in src


def test_frame_normal_always_points_away_from_the_surface(node):
    from touch_pack.plane_probe import fit_plane
    with node._params_lock:
        node._slide_plane_n = -np.asarray(
            fit_plane(_tilted_plane_pts(12.0)).normal)
    _u, _w, n, _src = node._slide_frame(np.array([0.0, 1.0]))
    assert n[2] > 0.0


def test_frame_refuses_a_null_direction(node):
    with node._params_lock:
        node._slide_plane_n = None
    assert node._slide_frame(np.array([0.0, 0.0])) is None


def test_frame_refuses_a_direction_normal_to_the_plane(node):
    # Plano "de pé" com normal em +X: deslizar em +X seria entrar na peça.
    with node._params_lock:
        node._slide_plane_n = np.array([1.0, 0.0, 0.0])
    assert node._slide_frame(np.array([1.0, 0.0])) is None


# ── Eixo de ataque na descida ─────────────────────────────────────────

def test_descending_uses_the_calibrated_axis(node):
    """O eixo calibrado tem de chegar ao _approach_dir, que é o que a
    regulação de força, o HOLD e o alívio de emergência consomem."""
    node._attack_dir = np.array([0.1, 0.0, -0.99498744])
    with node._params_lock:
        node._target_depth_mm = 0.0     # sai da fase antes de mover
    node._phase_descending()
    assert np.allclose(node._approach_dir, node._attack_dir)


def test_descending_falls_back_to_vertical(node):
    node._attack_dir = None
    with node._params_lock:
        node._target_depth_mm = 0.0
    node._phase_descending()
    assert np.allclose(node._approach_dir, [0.0, 0.0, -1.0])


# ── Reaplicação da orientação após a HOME (regressão) ─────────────────
# A HOME é uma pose ARTICULAR: leva a ferramenta de volta à vertical mesmo
# com _attack_dir setado. Sem reaplicar, o ciclo 2+ desceria na DIAGONAL
# com a ponteira apontando para baixo.

def test_reapply_is_a_noop_without_calibration(node):
    node._attack_dir = None
    node._rotate_to_attack = lambda *a, **k: pytest.fail('girou sem eixo')
    assert node._reapply_attack_orientation() is True


def test_reapply_rotates_to_the_calibrated_axis(node):
    seen = {}
    node._attack_dir = np.array([0.2, 0.0, -0.9797959])

    def _fake(attack, *, label='ALIGN'):
        seen['attack'] = np.asarray(attack).copy()
        return 'ok'

    node._rotate_to_attack = _fake
    assert node._reapply_attack_orientation() is True
    assert np.allclose(seen['attack'], node._attack_dir)


def test_reapply_reports_failure_so_the_cycle_aborts(node):
    node._attack_dir = np.array([0.2, 0.0, -0.9797959])
    node._rotate_to_attack = lambda *a, **k: 'error'
    assert node._reapply_attack_orientation() is False


# ── Grade do MATRIX_MAP em coordenadas do plano ───────────────────────

def test_matrix_basis_is_world_axes_without_calibration(node):
    node._attack_dir = None
    e1, e2, n = node._matrix_plane_basis()
    assert np.allclose(e1, [1.0, 0.0, 0.0])
    assert np.allclose(e2, [0.0, 1.0, 0.0])
    assert np.allclose(n, [0.0, 0.0, 1.0])


def test_matrix_basis_is_orthonormal_and_spans_the_tilted_plane(node):
    tilt = np.deg2rad(20.0)
    node._attack_dir = np.array([np.sin(tilt), 0.0, -np.cos(tilt)])
    e1, e2, n = node._matrix_plane_basis()
    for v in (e1, e2, n):
        assert np.linalg.norm(v) == pytest.approx(1.0)
    assert float(e1 @ e2) == pytest.approx(0.0, abs=1e-12)
    assert float(e1 @ n) == pytest.approx(0.0, abs=1e-12)
    assert float(e2 @ n) == pytest.approx(0.0, abs=1e-12)
    # A normal é o oposto do ataque: a grade fica no plano que ele ataca.
    assert np.allclose(n, -node._attack_dir)


def test_safe_pose_clearance_is_uniform_across_the_grid(node):
    """O bug que isto trava: com o Safe Z como ALTURA DO MUNDO, a superfície
    subia acima do plano de trânsito e o 'ar livre' virava arrasto."""
    tilt = np.deg2rad(20.0)
    node._attack_dir = np.array([np.sin(tilt), 0.0, -np.cos(tilt)])
    node._matrix_origin = np.array([0.5, 0.0, 0.3])
    node._matrix_safe_z_m = 0.010
    _e1, _e2, n = node._matrix_plane_basis()
    for wp in ([0.0, 0.0], [0.05, 0.0], [0.10, 0.03], [-0.08, -0.04]):
        pose = node._matrix_safe_pose(np.array(wp))
        gap = float((pose - node._matrix_origin) @ n)
        assert gap == pytest.approx(0.010, abs=1e-12)


def test_grid_spacing_is_exact_on_a_tilted_plane(node):
    """O outro bug: a descida ao longo do eixo inclinado comprimia o passo
    da grade por cos²(θ). Medido no plano, o passo é o pedido."""
    tilt = np.deg2rad(20.0)
    node._attack_dir = np.array([np.sin(tilt), 0.0, -np.cos(tilt)])
    node._matrix_origin = np.array([0.5, 0.0, 0.3])
    node._matrix_safe_z_m = 0.010
    a = node._matrix_safe_pose(np.array([0.000, 0.0]))
    b = node._matrix_safe_pose(np.array([0.005, 0.0]))
    assert float(np.linalg.norm(b - a)) == pytest.approx(0.005, abs=1e-12)


def test_safe_pose_matches_legacy_behaviour_when_flat(node):
    node._attack_dir = None
    node._matrix_origin = np.array([0.5, -0.2, 0.3])
    node._matrix_safe_z_m = 0.010
    pose = node._matrix_safe_pose(np.array([0.02, -0.03]))
    assert np.allclose(pose, [0.52, -0.23, 0.31])


# ── Contato aprendido não atravessa troca de eixo ─────────────────────

def test_learned_contact_key_is_dropped_when_the_axis_tilts(node):
    """O curso até o contato depende do eixo; a chave indexa a home, não o
    eixo. Mantê-la faria o estágio rápido invadir o contato aprendido."""
    t = np.tan(np.deg2rad(20.0))
    pts = np.asarray([[0.0, 0.0, 0.5], [0.02, 0.0, 0.5 + 0.02 * t],
                      [0.0, 0.02, 0.5], [0.02, 0.02, 0.5 + 0.02 * t]])
    with node._params_lock:
        node._mode = 'TOUCH'
        node._home_key_cur = (1.0, 2.0, 3.0)
        node._home_deg_cur = [0.0] * 6
        node._learned_contact_m = 0.030
        node._align_from_msg = True
        node._align_on = True
        node._align_msg = {'tilt_max_deg': 25.0}
    node._probe_plane = lambda *a, **k: ('ok', [p for p in pts])
    node._align_reorient = lambda *a, **k: 'ok'
    node._tcp_now = lambda: np.array([0.5, 0.0, 0.4])
    node._settle = lambda *a, **k: None
    node._move_linear_world = lambda *a, **k: 'done'

    assert node._phase_calibrate_attack() == 'ok'
    assert node._attack_dir is not None
    with node._params_lock:
        assert node._home_key_cur is None
        assert node._learned_contact_m is None


def test_small_tilt_keeps_the_learned_contact(node):
    """Desvio abaixo da tolerância não gira o punho, então o eixo continua
    vertical e o histórico da home segue válido."""
    pts = np.asarray([[0.0, 0.0, 0.5], [0.02, 0.0, 0.5],
                      [0.0, 0.02, 0.5], [0.02, 0.02, 0.5]])
    with node._params_lock:
        node._mode = 'TOUCH'
        node._home_key_cur = (1.0, 2.0, 3.0)
        node._learned_contact_m = 0.030
        node._align_from_msg = True
        node._align_on = True
        node._align_msg = {}
    node._probe_plane = lambda *a, **k: ('ok', [p for p in pts])
    node._tcp_now = lambda: np.array([0.5, 0.0, 0.4])
    node._settle = lambda *a, **k: None
    node._move_linear_world = lambda *a, **k: 'done'

    assert node._phase_calibrate_attack() == 'ok'
    assert node._attack_dir is None          # não girou o punho
    with node._params_lock:
        assert node._home_key_cur == (1.0, 2.0, 3.0)
        assert node._learned_contact_m == pytest.approx(0.030)


def test_small_tilt_still_hands_the_plane_to_the_slide(node):
    """A medição não pode ser descartada: 1,9° esgotam a reserva de
    indentação em menos de 2 mm de curso lateral."""
    from touch_pack.plane_probe import fit_plane

    t = np.tan(np.deg2rad(1.5))
    pts = np.asarray([[0.0, 0.0, 0.5], [0.02, 0.0, 0.5 + 0.02 * t],
                      [0.0, 0.02, 0.5], [0.02, 0.02, 0.5 + 0.02 * t]])
    with node._params_lock:
        node._mode = 'SLIDE'
        node._slide_dir_vec = np.array([1.0, 0.0])
        node._slide_plane_n = None
        node._align_from_msg = True
        node._align_on = True
        node._align_msg = {}
    node._probe_plane = lambda *a, **k: ('ok', [p for p in pts])
    node._tcp_now = lambda: np.array([0.5, 0.0, 0.4])
    node._settle = lambda *a, **k: None
    node._move_linear_world = lambda *a, **k: 'done'

    assert node._phase_calibrate_attack() == 'ok'
    assert node._attack_dir is None          # abaixo da tolerância: não gira
    assert node._slide_plane_n is not None    # mas o plano medido VALE
    assert np.allclose(node._slide_plane_n, fit_plane(pts).normal)
