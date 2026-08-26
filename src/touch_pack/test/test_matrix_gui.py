"""Testes do gerador de waypoints do configurador MATRIX_MAP da GUI."""
import pytest

tk = pytest.importorskip('tkinter')
pytest.importorskip('rclpy')


@pytest.fixture(scope='module')
def _root():
    try:
        r = tk.Tk()
    except tk.TclError as exc:            # sem display (CI headless)
        pytest.skip(f'sem display para Tk: {exc}')
    r.withdraw()
    yield r
    r.destroy()


class _FakeGUI:
    """Portador das variáveis Tk que o gerador de grade consulta."""

    def _matrix_grid_nodes(self):
        # _matrix_waypoints chama este método via self — reexpõe o real.
        from touch_pack.palpation_gui import PalpationGUI
        return PalpationGUI._matrix_grid_nodes(self)

    @classmethod
    def _matrix_index_order(cls, n_cols, n_rows, path):
        # _matrix_grid_nodes chama este via self — reexpõe o real.
        from touch_pack.palpation_gui import PalpationGUI
        return PalpationGUI._matrix_index_order(n_cols, n_rows, path)

    def __init__(self, shape='SQUARE', step_x=5.0, step_y=5.0,
                 cols=3, rows=3, sizing='STEP', width=20.0, height=20.0,
                 path='SERPENTINE', align=False):
        self.align_on_var = tk.BooleanVar(value=align)
        self.matrix_shape_var = tk.StringVar(value=shape)
        self.matrix_sizing_var = tk.StringVar(value=sizing)
        self.matrix_path_var = tk.StringVar(value=path)
        self.matrix_step_x_var = tk.DoubleVar(value=step_x)
        self.matrix_step_y_var = tk.DoubleVar(value=step_y)
        self.matrix_width_var = tk.DoubleVar(value=width)
        self.matrix_height_var = tk.DoubleVar(value=height)
        self.matrix_cols_var = tk.IntVar(value=cols)
        self.matrix_rows_var = tk.IntVar(value=rows)


def _nodes(gui):
    from touch_pack.palpation_gui import PalpationGUI
    return PalpationGUI._matrix_grid_nodes(gui)


def _waypoints(gui):
    from touch_pack.palpation_gui import PalpationGUI
    return PalpationGUI._matrix_waypoints(gui)


def _align_ring(gui):
    from touch_pack.palpation_gui import PalpationGUI
    return PalpationGUI._align_ring_nodes(gui)


# ── Anel da calibração do ângulo de ataque ────────────────────────────

def test_no_align_ring_when_the_calibration_is_off(_root):
    assert _align_ring(_FakeGUI(cols=4, rows=4, step_x=5.0)) == []


def test_align_ring_wraps_the_grid_half_a_step_out(_root):
    """O mesmo anel que o explorer vai sondar (tactile_explorer.
    _align_offsets), pela mesma função — aqui só para o preview mostrá-lo."""
    ring = _align_ring(_FakeGUI(cols=4, rows=4, step_x=5.0, align=True))
    assert len(ring) == 4
    flat = [c for p in sorted(ring) for c in p]
    assert flat == pytest.approx([-2.5, -2.5, -2.5, 17.5,
                                  17.5, -2.5, 17.5, 17.5])


def test_no_align_ring_when_the_grid_is_too_short(_root):
    """Preview honesto: nessa grade o explorer cai no polígono do raio, que
    o preview não tem como desenhar (não é referido à grade)."""
    assert _align_ring(_FakeGUI(cols=2, rows=2, step_x=1.0,
                                align=True)) == []


def test_square_grid_is_serpentine(_root):
    nodes, err = _nodes(_FakeGUI(cols=3, rows=3, step_x=5.0))
    assert err == ''
    # Linha 0 no +X, linha 1 de volta no −X, linha 2 no +X de novo.
    assert nodes == [
        (0.0, 0.0), (5.0, 0.0), (10.0, 0.0),
        (10.0, 5.0), (5.0, 5.0), (0.0, 5.0),
        (0.0, 10.0), (5.0, 10.0), (10.0, 10.0),
    ]


def test_square_mode_mirrors_x_onto_y(_root):
    # Step Y / Rows divergentes são IGNORADOS em SQUARE.
    nodes, err = _nodes(_FakeGUI(shape='SQUARE', step_x=2.0, step_y=9.0,
                                 cols=2, rows=7))
    assert err == ''
    assert nodes == [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]


def test_rect_mode_uses_independent_axes(_root):
    nodes, err = _nodes(_FakeGUI(shape='RECT', step_x=2.0, step_y=9.0,
                                 cols=2, rows=3))
    assert err == ''
    assert nodes == [(0.0, 0.0), (2.0, 0.0),
                     (2.0, 9.0), (0.0, 9.0),
                     (0.0, 18.0), (2.0, 18.0)]


def test_waypoints_drop_the_origin_node(_root):
    """O nó (0,0) é medido na descida de descoberta da origem — reenviá-lo
    faria o robô tocar duas vezes o mesmo ponto."""
    gui = _FakeGUI(cols=2, rows=2, step_x=5.0)
    nodes, _ = _nodes(gui)
    wps, err = _waypoints(gui)
    assert err == ''
    assert len(wps) == len(nodes) - 1
    assert (0.0, 0.0) not in wps
    assert wps == nodes[1:]


def test_1x1_grid_is_rejected(_root):
    wps, err = _waypoints(_FakeGUI(cols=1, rows=1))
    assert wps == []
    assert 'origin' in err


def test_point_cap_is_enforced(_root):
    from touch_pack.constants import MATRIX_MAX_POINTS
    from touch_pack.gui_constants import MATRIX_N_MAX
    gui = _FakeGUI(shape='RECT', cols=MATRIX_N_MAX, rows=MATRIX_N_MAX)
    wps, err = _waypoints(gui)
    if MATRIX_N_MAX ** 2 > MATRIX_MAX_POINTS:
        assert wps == [] and 'cap' in err
    else:
        assert err == '' and len(wps) == MATRIX_N_MAX ** 2 - 1


def test_span_envelope_is_enforced(_root):
    from touch_pack.gui_constants import MATRIX_STEP_MAX, MATRIX_N_MAX
    from touch_pack.constants import MATRIX_SPAN_MAX_MM
    gui = _FakeGUI(shape='SQUARE', step_x=MATRIX_STEP_MAX, cols=MATRIX_N_MAX)
    wps, err = _waypoints(gui)
    span = MATRIX_STEP_MAX * (MATRIX_N_MAX - 1)
    if span > MATRIX_SPAN_MAX_MM:
        assert wps == [] and ('envelope' in err or 'cap' in err)


def test_out_of_range_values_are_clamped_not_crashing(_root):
    gui = _FakeGUI(shape='RECT', step_x=999.0, step_y=-4.0, cols=99, rows=0)
    from touch_pack.gui_constants import (
        MATRIX_STEP_MIN, MATRIX_STEP_MAX, MATRIX_N_MAX)
    nodes, err = _nodes(gui)
    if err == '':
        xs = {round(x, 6) for x, _ in nodes}
        ys = {round(y, 6) for _, y in nodes}
        assert max(xs) <= MATRIX_STEP_MAX * (MATRIX_N_MAX - 1)
        assert min(ys) >= 0.0
        # step_y negativo saturou no piso, não virou grade invertida.
        assert all(y >= 0.0 for _, y in nodes)
        assert len(ys) == 1 or min(y for y in ys if y > 0) >= MATRIX_STEP_MIN


# ── Dimensionamento pelas dimensões do alvo (sizing='SIZE') ────────────

def test_size_mode_spans_exactly_the_target(_root):
    # 60 × 40 mm com 5 × 3 pontos → passo 15 × 20, bordas inclusas.
    nodes, err = _nodes(_FakeGUI(shape='RECT', sizing='SIZE',
                                 width=60.0, height=40.0, cols=5, rows=3))
    assert err == ''
    xs = [x for x, _ in nodes]
    ys = [y for _, y in nodes]
    assert (min(xs), max(xs)) == (0.0, 60.0)
    assert (min(ys), max(ys)) == (0.0, 40.0)
    assert sorted(set(round(x, 6) for x in xs)) == [0.0, 15.0, 30.0, 45.0, 60.0]
    assert sorted(set(round(y, 6) for y in ys)) == [0.0, 20.0, 40.0]


def test_size_mode_ignores_the_step_variables(_root):
    # O passo digitado é irrelevante em SIZE — quem manda é a dimensão.
    a, _ = _nodes(_FakeGUI(sizing='SIZE', width=10.0, cols=3, step_x=99.0))
    b, _ = _nodes(_FakeGUI(sizing='SIZE', width=10.0, cols=3, step_x=0.5))
    assert a == b
    assert max(x for x, _ in a) == 10.0


def test_size_mode_square_mirrors_width_onto_height(_root):
    nodes, err = _nodes(_FakeGUI(shape='SQUARE', sizing='SIZE',
                                 width=12.0, height=99.0, cols=3, rows=7))
    assert err == ''
    assert max(y for _, y in nodes) == 12.0
    assert len(nodes) == 9          # rows seguiu cols, não o 7 digitado


def test_size_mode_keeps_serpentine_order(_root):
    nodes, err = _nodes(_FakeGUI(shape='RECT', sizing='SIZE',
                                 width=10.0, height=10.0, cols=3, rows=2))
    assert err == ''
    assert nodes == [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0),
                     (10.0, 10.0), (5.0, 10.0), (0.0, 10.0)]


def test_size_mode_rejects_step_below_the_floor(_root):
    from touch_pack.gui_constants import MATRIX_STEP_MIN, MATRIX_N_MAX
    # Alvo minúsculo com muitos pontos: o passo derivado fica abaixo do piso
    # mecânico.
    gui = _FakeGUI(shape='SQUARE', sizing='SIZE',
                   width=MATRIX_STEP_MIN, cols=MATRIX_N_MAX)
    wps, err = _waypoints(gui)
    assert wps == []
    assert 'below the' in err and 'floor' in err


def test_size_mode_single_column_degenerates_to_a_line(_root):
    # 1 coluna não tem intervalo em X: passo 0, sem divisão por zero.
    nodes, err = _nodes(_FakeGUI(shape='RECT', sizing='SIZE',
                                 width=50.0, height=30.0, cols=1, rows=4))
    assert err == ''
    assert all(x == 0.0 for x, _ in nodes)
    assert [y for _, y in nodes] == [0.0, 10.0, 20.0, 30.0]


def test_step_mode_is_unchanged_by_the_new_vars(_root):
    # Configs antigas (sem matrix_sizing) caem em STEP e geram a MESMA grade.
    nodes, err = _nodes(_FakeGUI(shape='SQUARE', step_x=5.0, cols=3))
    assert err == ''
    assert nodes == [
        (0.0, 0.0), (5.0, 0.0), (10.0, 0.0),
        (10.0, 5.0), (5.0, 5.0), (0.0, 5.0),
        (0.0, 10.0), (5.0, 10.0), (10.0, 10.0),
    ]


# ── Ordem de visita: cantos primeiro (path='CORNERS') ──────────────────

def test_corners_are_the_first_four_points(_root):
    # Exemplo do usuário: numa 4×4 os extremos são (1,1) (1,4) (4,1) (4,4)
    # em contagem 1-based — aqui (0,0) (0,3) (3,0) (3,3) em passos de 1 mm.
    nodes, err = _nodes(_FakeGUI(shape='RECT', path='CORNERS',
                                 step_x=1.0, step_y=1.0, cols=4, rows=4))
    assert err == ''
    assert set(nodes[:4]) == {(0.0, 0.0), (3.0, 0.0), (0.0, 3.0), (3.0, 3.0)}
    assert nodes[0] == (0.0, 0.0)          # origem continua primeira


def test_corners_take_the_short_tour(_root):
    # Volta mais curta: origem → +X → +Y → −X, sem cruzar a diagonal.
    nodes, _ = _nodes(_FakeGUI(shape='RECT', path='CORNERS',
                               step_x=1.0, step_y=1.0, cols=4, rows=4))
    assert nodes[:4] == [(0.0, 0.0), (3.0, 0.0), (3.0, 3.0), (0.0, 3.0)]


def test_corners_are_not_re_touched_in_the_sweep(_root):
    nodes, err = _nodes(_FakeGUI(shape='RECT', path='CORNERS',
                                 step_x=1.0, step_y=1.0, cols=4, rows=4))
    assert err == ''
    assert len(nodes) == len(set(nodes)) == 16
    assert not (set(nodes[:4]) & set(nodes[4:]))


def test_corners_and_serpentine_cover_the_same_points(_root):
    kw = dict(shape='RECT', step_x=2.0, step_y=3.0, cols=5, rows=4)
    a, _ = _nodes(_FakeGUI(path='CORNERS', **kw))
    b, _ = _nodes(_FakeGUI(path='SERPENTINE', **kw))
    assert sorted(a) == sorted(b)
    assert a != b


def test_corners_come_from_the_target_dimensions(_root):
    # Em SIZE os extremos são exatamente (0,0) (W,0) (W,H) (0,H) — é assim
    # que a conferência sabe onde ficam as quinas do objeto.
    nodes, err = _nodes(_FakeGUI(shape='RECT', path='CORNERS', sizing='SIZE',
                                 width=60.0, height=40.0, cols=5, rows=3))
    assert err == ''
    assert nodes[:4] == [(0.0, 0.0), (60.0, 0.0), (60.0, 40.0), (0.0, 40.0)]
    assert len(nodes) == len(set(nodes)) == 15


def test_corners_degenerate_grids_have_no_duplicates(_root):
    # Linha/coluna única tem 2 extremos, não 4 — e nada pode repetir.
    for cols, rows in ((1, 5), (5, 1), (2, 2), (2, 6)):
        gui = _FakeGUI(shape='RECT', path='CORNERS', cols=cols, rows=rows)
        nodes, err = _nodes(gui)
        assert err == ''
        assert len(nodes) == len(set(nodes)) == cols * rows
        assert nodes[0] == (0.0, 0.0)


def test_corners_waypoints_still_drop_the_origin(_root):
    gui = _FakeGUI(shape='RECT', path='CORNERS', step_x=5.0, step_y=5.0,
                   cols=3, rows=3)
    nodes, _ = _nodes(gui)
    wps, err = _waypoints(gui)
    assert err == ''
    assert wps == nodes[1:] and (0.0, 0.0) not in wps


# ── Geometria exposta ao preview (_matrix_geom) ────────────────────────

def test_geom_carries_the_declared_target_size(_root):
    gui = _FakeGUI(shape='RECT', sizing='SIZE', path='CORNERS',
                   width=100.0, height=150.0, cols=4, rows=5)
    _nodes(gui)
    g = gui._matrix_geom
    assert (g['width'], g['height']) == (100.0, 150.0)
    assert g['by_size'] is True
    assert (g['n_cols'], g['n_rows']) == (4, 5)
    # O passo é o derivado, e é ele que o rodapé mostra.
    assert g['step_x'] == pytest.approx(100.0 / 3)
    assert g['step_y'] == pytest.approx(150.0 / 4)


def test_geom_has_no_target_size_in_step_mode(_root):
    # Sem alvo declarado o preview não tem retângulo para desenhar.
    gui = _FakeGUI(shape='RECT', sizing='STEP', cols=3, rows=3)
    _nodes(gui)
    g = gui._matrix_geom
    assert g['width'] is None and g['height'] is None
    assert g['by_size'] is False


def test_declared_height_survives_a_single_row(_root):
    """Com 1 linha o SPAN em Y é 0, mas o objeto continua tendo altura — o
    preview desenha o objeto, então precisa da dimensão declarada."""
    gui = _FakeGUI(shape='RECT', sizing='SIZE', width=100.0, height=150.0,
                   cols=4, rows=1)
    nodes, err = _nodes(gui)
    assert err == ''
    g = gui._matrix_geom
    assert g['span_y'] == 0.0
    assert g['height'] == 150.0
    assert all(y == 0.0 for _, y in nodes)


def test_corners_land_on_the_object_quinas(_root):
    # 100×150 com qualquer contagem: a conferência toca as 4 quinas reais.
    for cols, rows in ((4, 5), (2, 2), (6, 3)):
        nodes, err = _nodes(_FakeGUI(shape='RECT', sizing='SIZE',
                                     path='CORNERS', width=100.0,
                                     height=150.0, cols=cols, rows=rows))
        assert err == ''
        assert nodes[:4] == [(0.0, 0.0), (100.0, 0.0),
                             (100.0, 150.0), (0.0, 150.0)]
