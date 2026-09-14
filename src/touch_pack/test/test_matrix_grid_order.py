"""A ORDEM de visita da grade MATRIX_MAP e o anel de calibração.

`test_matrix_gui.py` já cobre a geometria (onde ficam os nós). O que fica de
fora é a ordem em que a sonda os visita — e ela não é decoração:

  * a serpentina existe para que o trânsito entre dois pontos seja sempre um
    passo curto. Trocá-la por varredura simples faz a sonda voltar ao começo
    da linha a cada fileira, multiplicando o tempo de um mapeamento e o
    desgaste da amostra entre identações;
  * CORNERS visita os quatro extremos ANTES do resto, porque é isso que
    amarra o plano cedo — se o quarto canto revelar que a peça está torta,
    descobrir na identação 4 custa três pontos, descobrir na 80 custa o
    ensaio;
  * o primeiro nó é a ORIGEM, já visitada na busca — mandá-la de novo ao
    explorer faria uma identação repetida no mesmo lugar.

Tudo aqui é estático (`@staticmethod`/`@classmethod`) ou opera sobre um
portador de variáveis: nada exige display nem ROS no ar.
"""
import ast
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

pytest.importorskip('rclpy')

from touch_pack.gui_matrix import MatrixMixin                # noqa: E402

_PKG = pathlib.Path(__file__).resolve().parents[1] / 'touch_pack'


def _ordem(n_cols, n_rows, path):
    return MatrixMixin._matrix_index_order(n_cols, n_rows, path)


# ── Serpentina ────────────────────────────────────────────────────────
def test_serpentine_alternates_direction_every_row():
    """A propriedade que dá nome à coisa. Sem a alternância, o trânsito da
    última coluna para a primeira aparece uma vez por fileira."""
    out = MatrixMixin._matrix_serpentine(0, 2, 0, 1)
    assert out == [(0, 0), (1, 0), (2, 0),
                   (2, 1), (1, 1), (0, 1)]


def test_serpentine_never_jumps_more_than_one_index():
    """O ganho real: passos consecutivos são sempre vizinhos. Este teste
    falha no instante em que alguém "simplificar" para varredura simples."""
    out = MatrixMixin._matrix_serpentine(0, 4, 0, 3)
    for (x0, y0), (x1, y1) in zip(out, out[1:]):
        assert abs(x1 - x0) + abs(y1 - y0) == 1


def test_serpentine_visits_every_cell_exactly_once():
    out = MatrixMixin._matrix_serpentine(0, 3, 0, 2)
    assert len(out) == 12
    assert len(set(out)) == 12


@pytest.mark.parametrize('args', [(0, -1, 0, 2), (0, 2, 0, -1), (1, 0, 1, 0)])
def test_empty_rectangle_gives_no_points(args):
    """Grade degenerada tem de sair vazia, não estourar: os limites vêm de
    campos que o operador digita."""
    assert MatrixMixin._matrix_serpentine(*args) == []


# ── Cantos ────────────────────────────────────────────────────────────
def test_corners_walk_the_shortest_loop():
    """(0,0) → (max,0) → (max,max) → (0,max): volta pela borda. A ordem
    "diagonal" atravessaria a peça duas vezes no ar."""
    assert MatrixMixin._matrix_corners(4, 3) == [
        (0, 0), (3, 0), (3, 2), (0, 2)]


def test_a_single_column_has_only_two_extremes():
    """Grade 1×N não tem quatro cantos. Devolver quatro repetiria dois
    pontos e faria duas identações a mais no mesmo lugar."""
    assert MatrixMixin._matrix_corners(1, 5) == [(0, 0), (0, 4)]


def test_a_single_row_has_only_two_extremes():
    assert MatrixMixin._matrix_corners(6, 1) == [(0, 0), (5, 0)]


def test_a_single_point_grid_has_one_corner():
    assert MatrixMixin._matrix_corners(1, 1) == [(0, 0)]


# ── Ordem completa ────────────────────────────────────────────────────
def test_corners_come_first_and_are_not_repeated():
    """O contrato do modo CORNERS: os extremos na frente, cada nó uma vez
    só. Um canto repetido é uma identação a mais no mesmo ponto da amostra."""
    ordem = _ordem(4, 3, 'CORNERS')
    assert ordem[:4] == [(0, 0), (3, 0), (3, 2), (0, 2)]
    assert len(ordem) == len(set(ordem)) == 12


def test_snake_order_is_the_plain_serpentine():
    assert _ordem(4, 3, 'SNAKE') == MatrixMixin._matrix_serpentine(0, 3, 0, 2)


@pytest.mark.parametrize('path', ['CORNERS', 'SNAKE'])
@pytest.mark.parametrize('n_cols,n_rows', [(1, 1), (1, 4), (5, 1), (3, 3),
                                           (2, 7), (6, 4)])
def test_every_order_covers_the_whole_grid_exactly_once(path, n_cols, n_rows):
    """Invariante que vale para qualquer modo e qualquer tamanho: o ensaio
    tem de visitar cada ponto da grade uma vez. Um nó a menos é um buraco no
    mapa; um a mais é uma identação repetida."""
    ordem = _ordem(n_cols, n_rows, path)
    assert len(ordem) == n_cols * n_rows
    assert set(ordem) == {(ix, iy)
                          for iy in range(n_rows) for ix in range(n_cols)}


@pytest.mark.parametrize('path', ['CORNERS', 'SNAKE'])
def test_the_visit_always_starts_at_the_origin(path):
    """A origem é o ponto encontrado pela busca de contato. Começar em outro
    lugar deixaria o primeiro waypoint sem referência de profundidade."""
    assert _ordem(5, 4, path)[0] == (0, 0)


def test_an_unknown_path_name_falls_back_to_the_serpentine():
    """Nome vindo de um preset antigo em disco não pode derrubar a GUI nem
    devolver lista vazia (que viraria "grade sem pontos")."""
    ordem = _ordem(3, 3, 'MODO_QUE_NAO_EXISTE')
    assert len(ordem) == 9


# ── Waypoints enviados ────────────────────────────────────────────────
class _Grade:
    """Portador que reexpõe `_matrix_waypoints` sobre uma grade fixa."""

    def __init__(self, nodes, err=''):
        self._nodes, self._err = nodes, err

    def _matrix_grid_nodes(self):
        return self._nodes, self._err

    _matrix_waypoints = MatrixMixin._matrix_waypoints


def test_the_origin_is_not_sent_as_a_waypoint():
    """Ela já foi visitada na busca de origem. Reenviá-la faria a primeira
    identação do mapa cair duas vezes no mesmo ponto."""
    nodes = [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)]
    wps, err = _Grade(nodes)._matrix_waypoints()
    assert err == ''
    assert wps == [(5.0, 0.0), (10.0, 0.0)]


def test_a_grid_error_sends_no_waypoints_at_all():
    """Meia grade é pior que nenhuma: o run pararia no meio com o braço
    sobre a amostra em vez de ser recusado antes de começar."""
    wps, err = _Grade([(0.0, 0.0)], err='passo maior que a largura')._matrix_waypoints()
    assert wps == []
    assert err != ''


def test_a_single_point_grid_sends_nothing_to_probe():
    """Grade 1×1 é só a origem — não sobra waypoint nenhum, e isso não é
    erro."""
    wps, err = _Grade([(0.0, 0.0)])._matrix_waypoints()
    assert (wps, err) == ([], '')


# ── Anel de alinhamento ───────────────────────────────────────────────
class _Anel:
    def __init__(self, ligado, nodes, err=''):
        self.align_on_var = type('_V', (), {'get': lambda _s: ligado})()
        self._nodes, self._err = nodes, err

    def _matrix_grid_nodes(self):
        return self._nodes, self._err

    _align_ring_nodes = MatrixMixin._align_ring_nodes


def _grade_mm(n=5, passo=5.0):
    return [(ix * passo, iy * passo) for iy in range(n) for ix in range(n)]


def test_no_ring_when_alignment_is_off():
    assert _Anel(False, _grade_mm())._align_ring_nodes() == []


def test_no_ring_when_the_grid_is_in_error():
    assert _Anel(True, [], err='grade inválida')._align_ring_nodes() == []


def test_ring_has_four_touch_points():
    """São os quatro toques que medem a inclinação do plano. Menos que
    quatro não define um plano com redundância."""
    anel = _Anel(True, _grade_mm())._align_ring_nodes()
    assert len(anel) == 4


def test_ring_is_returned_in_millimetres():
    """A grade está em mm e o `probe_ring_from_grid` trabalha em metros. Um
    fator 1000 perdido aqui desenharia o anel fora da tela — e, pior,
    esconderia que a sonda vai encostar longe da grade."""
    passo = 5.0
    anel = _Anel(True, _grade_mm(n=5, passo=passo))._align_ring_nodes()
    extensao = max(abs(c) for p in anel for c in p)
    assert 1.0 < extensao < 1000.0


def test_the_preview_ring_matches_the_one_the_robot_will_probe():
    """O anel desenhado e o que o explorer executa TÊM de sair da mesma
    função — duas derivações independentes divergiriam em silêncio e o
    preview mentiria sobre onde o robô encosta."""
    src = (_PKG / 'gui_matrix.py').read_text(encoding='utf-8')
    assert 'probe_ring_from_grid' in src
    explorer = (_PKG / 'tactile_explorer.py').read_text(encoding='utf-8')
    assert 'probe_ring_from_grid' in explorer


# ── Espelho SQUARE ────────────────────────────────────────────────────
class _Var:
    def __init__(self, v):
        self._v = v

    def get(self):
        return self._v

    def set(self, v):
        self._v = v


class _Shape:
    def __init__(self, shape):
        self.matrix_shape_var = _Var(shape)
        self.matrix_step_x_var, self.matrix_step_y_var = _Var(5.0), _Var(9.0)
        self.matrix_width_var, self.matrix_height_var = _Var(40.0), _Var(70.0)
        self.matrix_cols_var, self.matrix_rows_var = _Var(4), _Var(9)

    _matrix_mirror_x_onto_y = MatrixMixin._matrix_mirror_x_onto_y


def test_square_copies_x_onto_y_including_hidden_fields():
    """Os campos de Y ficam ocultos em SQUARE, mas guardam o valor de um
    RECT anterior. Sem a cópia, a grade "quadrada" sairia retangular."""
    s = _Shape('SQUARE')
    s._matrix_mirror_x_onto_y()
    assert s.matrix_step_y_var.get() == 5.0
    assert s.matrix_height_var.get() == 40.0
    assert s.matrix_rows_var.get() == 4


def test_rect_keeps_the_two_axes_independent():
    s = _Shape('RECT')
    s._matrix_mirror_x_onto_y()
    assert (s.matrix_step_y_var.get(), s.matrix_height_var.get()) == (9.0, 70.0)


def test_the_mirror_does_not_re_enter_through_the_trace():
    """`set()` dispara o trace das variáveis, que chama o espelho de novo. A
    flag `_suppressing` é o que corta a recursão — sem ela a GUI congela ao
    digitar no campo de passo."""
    s = _Shape('SQUARE')
    s._suppressing = True
    s._matrix_mirror_x_onto_y()
    assert s.matrix_step_y_var.get() == 9.0, 'espelhou durante a supressão'


def test_mixin_declares_no_import_from_the_host():
    """Guarda de ciclo: o host importa o mixin, então o mixin não pode
    importar o host — o erro só apareceria no import, em runtime."""
    tree = ast.parse((_PKG / 'gui_matrix.py').read_text(encoding='utf-8'))
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            assert node.module != 'palpation_gui'
            assert not (node.module or '').endswith('.palpation_gui')
