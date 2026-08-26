"""Testes do modo MATRIX_MAP do tactile_explorer — validação da grade,
descoberta da origem, roteamento espacial e Regra de Ouro no aborto.
"""
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Point                # noqa: E402
from std_msgs.msg import Float32                   # noqa: E402
from touch_pack_msgs.msg import PalpationStart     # noqa: E402


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


def _start(waypoints_mm, **overrides) -> PalpationStart:
    msg = PalpationStart()
    msg.speed_mms = 10.0
    msg.depth_mm = 20.0
    msg.force_n = 2.0
    msg.slide_dist_mm = 50.0
    msg.approach_speed_mms = 8.0
    msg.slide_dir = '+Y'
    msg.repeats = 1
    msg.speed_factor_pct = 10.0
    msg.home_deg = [0.0, 0.0, -90.0, 0.0, 90.0, 0.0]
    msg.mode = 'MATRIX_MAP'
    msg.safe_z_mm = 10.0
    msg.transit_speed_mms = 10.0
    msg.grid_shape = 'SQUARE'
    msg.waypoints = [Point(x=x / 1000.0, y=y / 1000.0, z=0.0)
                     for x, y in waypoints_mm]
    for k, v in overrides.items():
        setattr(msg, k, v)
    return msg


# ── Validação da grade (nada disso pode virar movimento) ───────────────

def test_waypoints_empty_is_refused(node):
    assert node._parse_matrix_waypoints(_start([])) is None


def test_waypoints_over_point_cap_is_refused(node):
    from touch_pack.constants import MATRIX_MAX_POINTS
    too_many = [(float(i), 0.0) for i in range(MATRIX_MAX_POINTS + 1)]
    assert node._parse_matrix_waypoints(_start(too_many)) is None


def test_waypoints_outside_envelope_is_refused(node):
    from touch_pack.constants import MATRIX_SPAN_MAX_MM
    assert node._parse_matrix_waypoints(
        _start([(MATRIX_SPAN_MAX_MM + 1.0, 0.0)])) is None


def test_waypoints_with_nan_is_refused(node):
    assert node._parse_matrix_waypoints(
        _start([(0.0, 0.0), (float('nan'), 5.0)])) is None


def test_valid_waypoints_convert_to_metres(node):
    xy = node._parse_matrix_waypoints(_start([(5.0, 0.0), (5.0, 5.0)]))
    assert xy is not None
    assert xy.shape == (2, 2)
    np.testing.assert_allclose(xy, [[0.005, 0.0], [0.005, 0.005]])


def test_start_with_invalid_matrix_starts_no_thread(node):
    node._protocol_thread = None
    node._cb_start(_start([]))          # sem waypoints → run recusado
    assert node._protocol_thread is None
    assert node._phase == 'ABORTED'


# ── FSM da matriz ──────────────────────────────────────────────────────

_ORIGIN = np.array([0.40, 0.10, 0.25])


def _stub_matrix(node, calls, *, hold_out='ok', descend_out='ok'):
    """Stuba tudo que move o braço; registra a sequência em `calls`."""
    node._tcp_now = lambda: _ORIGIN.copy()
    node._send_hand_pose = lambda *a, **k: None
    node._settle = lambda *a, **k: None
    node._phase_descending = (
        lambda: calls.append('desc')
        or (descend_out() if callable(descend_out) else descend_out))
    node._phase_hold = (
        lambda *a, **k: calls.append('hold')
        or (hold_out() if callable(hold_out) else hold_out))
    node._move_linear_world = (
        lambda delta, v, **k: calls.append(f'move:{k.get("label", "?")}')
        or 'done')
    node._relieve_contact = lambda *a, **k: calls.append('relieve')
    node._retreat_and_home = lambda fp: (calls.append(f'retreat→{fp}'),
                                         node._set_phase(fp)) and None
    node._abort_to_home = lambda: (calls.append('abort→home'),
                                   node._set_phase('ABORTED')) and None


def test_matrix_visits_every_waypoint_and_homes(node):
    calls: list = []
    _stub_matrix(node, calls)
    points: list = []
    node._matrix_pub.publish = points.append

    node._cb_start(_start([(5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]))
    node._protocol_thread.join(timeout=15)

    # 1 descida de origem + 3 identações.
    assert calls.count('desc') == 4
    assert calls.count('hold') == 4
    # Um MatrixPoint por identação, com a origem em index 0.
    assert [p.index for p in points] == [0, 1, 2, 3]
    assert points[0].outcome == 'origin'
    assert all(p.outcome == 'ok' for p in points[1:])
    assert all(p.total == 3 for p in points[1:])
    # Coordenadas planejadas chegam ao log em mm, na ordem enviada.
    assert [(round(p.plan_x_mm, 3), round(p.plan_y_mm, 3))
            for p in points[1:]] == [(5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]
    # Origem gravada e propagada para todos os registros.
    for p in points:
        assert p.origin_x_m == pytest.approx(_ORIGIN[0])
        assert p.origin_z_m == pytest.approx(_ORIGIN[2])
    # Setpoint da GUI aplicado a cada ponto.
    assert all(p.setpoint_n == pytest.approx(2.0) for p in points)
    # Regra de Ouro no fim: volta à HOME articular.
    assert 'retreat→DONE' in calls
    assert node._wp_index == 0


def test_matrix_transits_at_safe_z_between_points(node):
    calls: list = []
    _stub_matrix(node, calls)
    node._matrix_pub.publish = lambda _m: None

    node._cb_start(_start([(5.0, 0.0), (5.0, 5.0)]))
    node._protocol_thread.join(timeout=15)

    moves = [c for c in calls if c.startswith('move:')]
    # Sobe após a origem, e por waypoint: TRANSIT (XY no ar) + LIFT de volta.
    assert moves[0] == 'move:LIFT-ORIGIN'
    assert moves.count('move:TRANSIT') == 2
    assert moves.count('move:LIFT-WP') == 2
    # Toda descida é precedida de um trânsito e sucedida de uma subida.
    seq = [c for c in calls if c in ('desc', 'move:TRANSIT', 'move:LIFT-WP')]
    assert seq == ['desc',
                   'move:TRANSIT', 'desc', 'move:LIFT-WP',
                   'move:TRANSIT', 'desc', 'move:LIFT-WP']


def test_matrix_origin_is_the_first_contact(node):
    calls: list = []
    _stub_matrix(node, calls)
    node._matrix_pub.publish = lambda _m: None
    node._cb_start(_start([(5.0, 0.0)]))
    node._protocol_thread.join(timeout=15)
    assert node._matrix_origin is not None
    np.testing.assert_allclose(node._matrix_origin, _ORIGIN)


def test_matrix_stop_mid_grid_relieves_lifts_and_homes(node):
    """Regra de Ouro: STOP no HOLD do 2º ponto aborta o resto da matriz,
    alivia o contato, sobe em +Z e recua à HOME."""
    calls: list = []

    def hold():
        # origem = 1ª chamada; 2ª = waypoint 1; 3ª = waypoint 2 → STOP
        return 'stop' if calls.count('hold') >= 3 else 'ok'
    _stub_matrix(node, calls, hold_out=hold)
    points: list = []
    node._matrix_pub.publish = points.append
    # Contato ativo no momento do stop, para o alívio ser exercido.
    node._cb_lc_force_net(Float32(data=2.0))

    node._cb_start(_start([(5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)]))
    node._protocol_thread.join(timeout=15)

    assert calls.count('hold') == 3          # não seguiu para o 4º ponto
    assert 'relieve' in calls                # aliviou o contato
    assert 'move:LIFT-ABORT' in calls        # subiu em +Z
    assert node._phase == 'ABORTED'
    # O ponto interrompido ainda é registrado, com o motivo.
    assert points[-1].index == 2 and points[-1].outcome == 'stop'


def test_matrix_force_abort_goes_home(node):
    calls: list = []
    _stub_matrix(node, calls, hold_out='force')
    node._matrix_pub.publish = lambda _m: None
    node._cb_start(_start([(5.0, 0.0)]))
    node._protocol_thread.join(timeout=15)
    # Estouro de força já na descida da origem → aborta com retorno à HOME.
    assert 'abort→home' in calls
    assert node._phase == 'ABORTED'


def test_matrix_no_contact_on_origin_aborts(node):
    calls: list = []
    _stub_matrix(node, calls, descend_out='no_contact')
    node._matrix_pub.publish = lambda _m: None
    node._cb_start(_start([(5.0, 0.0)]))
    node._protocol_thread.join(timeout=15)
    assert calls.count('desc') == 1          # nem tentou os waypoints
    assert 'abort→home' in calls


def test_matrix_ignores_repeats(node):
    """`repeats` não multiplica a matriz — o número de identações é o
    número de waypoints."""
    calls: list = []
    _stub_matrix(node, calls)
    node._matrix_pub.publish = lambda _m: None
    node._cb_start(_start([(5.0, 0.0)], repeats=7))
    node._protocol_thread.join(timeout=15)
    assert calls.count('desc') == 2          # origem + 1 waypoint
