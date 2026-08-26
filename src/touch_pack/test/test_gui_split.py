"""Guarda do recorte de `palpation_gui.py` em mixins.

A classe tinha 234 métodos num arquivo só e está sendo fatiada. O recorte é
MECÂNICO — método move inteiro, continua operando sobre `self` —, então a
única coisa que pode dar errado em silêncio é um método sumir no caminho ou
passar a existir em dois lugares. É isso que estes testes travam.

Análise por AST: não importa `palpation_gui` (que exige rclpy + Tk), então
roda em qualquer máquina.
"""
import ast
import pathlib

import pytest

_PKG = pathlib.Path(__file__).resolve().parents[1] / 'touch_pack'

# (arquivo, classe) de cada fatia já extraída. Novas fatias entram aqui.
MIXINS = [
    ('gui_loadcell.py', 'FtAxesMixin'),
    ('gui_matrix.py', 'MatrixMixin'),
]
HOST = ('palpation_gui.py', 'PalpationGUI')

# Vem de `rclpy.node.Node`, a outra base de PalpationGUI. Um mixin pode usar
# livremente — a MRO resolve —, mas a análise estática não enxerga a classe
# do rclpy, então a lista entra à mão.
_NODE_API = {
    'get_logger', 'get_clock', 'create_publisher', 'create_subscription',
    'create_timer', 'declare_parameter', 'get_parameter', 'set_parameters',
    'count_publishers', 'count_subscribers', 'destroy_node',
    'destroy_subscription', 'destroy_publisher', 'destroy_timer',
    'get_name', 'get_namespace', 'context', 'executor',
}


def _methods(filename: str, cls_name: str) -> set[str]:
    tree = ast.parse((_PKG / filename).read_text(encoding='utf-8'), filename)
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == cls_name:
            return {m.name for m in node.body
                    if isinstance(m, (ast.FunctionDef, ast.AsyncFunctionDef))}
    raise AssertionError(f'classe {cls_name} não encontrada em {filename}')


def _bases(filename: str, cls_name: str) -> list[str]:
    tree = ast.parse((_PKG / filename).read_text(encoding='utf-8'), filename)
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == cls_name:
            return [b.id for b in node.bases if isinstance(b, ast.Name)]
    raise AssertionError(f'classe {cls_name} não encontrada')


@pytest.mark.parametrize('filename,cls_name', MIXINS)
def test_mixin_is_actually_mixed_in(filename, cls_name):
    """Extrair sem herdar deixaria os métodos órfãos — e a GUI só quebraria
    ao abrir a aba, longe de qualquer teste."""
    assert cls_name in _bases(*HOST)


@pytest.mark.parametrize('filename,cls_name', MIXINS)
def test_mixin_does_not_shadow_the_host(filename, cls_name):
    """Nenhum método pode existir nos dois lados: a MRO escolheria um deles
    silenciosamente, e a versão perdedora viraria código morto enganoso."""
    dupes = _methods(*HOST) & _methods(filename, cls_name)
    assert not dupes, f'definidos em ambos: {sorted(dupes)}'


def test_no_method_was_lost_in_the_split():
    """O conjunto host ∪ mixins tem de conter tudo que o grupo recortado
    tinha. Lista explícita: se um método sumir, o teste diz QUAL."""
    have = _methods(*HOST)
    for filename, cls_name in MIXINS:
        have |= _methods(filename, cls_name)
    # O wizard de calibração saiu com a célula axial (20/08/2026): a FA7155
    # vem calibrada de fábrica e o único ajuste do host é o TARE. O que
    # precisa continuar existindo é a aba dos seis eixos e o tare.
    esperados = {
        '_build_lc_axes_tab', '_refresh_ft_axes', '_cb_ft_wrench',
        '_lc_do_tare', '_cb_lc_tare_result', '_cb_lc_tared',
        '_cb_lc_force_net_gui', '_cb_lc_force_raw_gui',
    }
    assert esperados <= have, f'sumiram: {sorted(esperados - have)}'


def test_load_cell_group_left_the_host():
    """O ganho do recorte é o host encolher. Se os métodos continuarem lá,
    o arquivo não ficou mais navegável — só ganhou um import."""
    host = _methods(*HOST)
    assert '_build_lc_axes_tab' not in host
    assert '_build_lc_calibration_tab' not in host
    assert '_matrix_waypoints' not in host
    assert '_redraw_matrix_preview' not in host


def test_shared_gui_constants_have_a_home_of_their_own():
    """As constantes usadas pelo host E pelas fatias moram em
    `gui_constants.py`. Deixá-las no host obrigaria os mixins a importar de
    quem os importa — um ciclo que só estoura no import, em runtime."""
    host_src = (_PKG / 'palpation_gui.py').read_text(encoding='utf-8')
    for nome in ('ARM_LIMITS_DEG', 'MATRIX_SHAPES', 'FORCE_SP_DEFAULT'):
        assert f'\n{nome}' not in host_src, f'{nome} voltou a ser definido no host'
    for filename, _cls in MIXINS:
        src = (_PKG / filename).read_text(encoding='utf-8')
        assert 'from .palpation_gui import' not in src, (
            f'{filename} importa do host — ciclo de import')


def _class_node(filename: str, cls_name: str) -> ast.ClassDef:
    tree = ast.parse((_PKG / filename).read_text(encoding='utf-8'), filename)
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == cls_name:
            return node
    raise AssertionError(f'classe {cls_name} não encontrada em {filename}')


def _self_attrs(node: ast.ClassDef, store: bool):
    """Atributos `self.X` lidos (store=False) ou escritos (store=True)."""
    want = (ast.Store,) if store else (ast.Load,)
    out = set()
    for x in ast.walk(node):
        if (isinstance(x, ast.Attribute) and isinstance(x.value, ast.Name)
                and x.value.id == 'self' and isinstance(x.ctx, want)):
            out.add(x.attr)
        # setattr(self, 'nome', ...) escreve sem virar ast.Attribute
        if (store and isinstance(x, ast.Call) and isinstance(x.func, ast.Name)
                and x.func.id == 'setattr' and len(x.args) > 1
                and isinstance(x.args[1], ast.Constant)):
            out.add(x.args[1].value)
    return out


@pytest.mark.parametrize('filename,cls_name', MIXINS)
def test_every_self_reference_in_the_mixin_resolves(filename, cls_name):
    """A falha característica de um recorte: o método foi para o mixin, mas
    o atributo (ou o método irmão) que ele usa ficou para trás com outro
    nome, ou não existe mais. Sem isto, só se descobre ao abrir a aba.

    Varre TODO `self.X` lido no mixin e exige que X seja um método de
    qualquer fatia, um atributo atribuído em qualquer fatia, ou API do Node.
    """
    nodes = [_class_node(*HOST)] + [_class_node(f, c) for f, c in MIXINS]
    known = set(_NODE_API)
    for n in nodes:
        known |= {m.name for m in n.body
                  if isinstance(m, (ast.FunctionDef, ast.AsyncFunctionDef))}
        known |= {t.id for m in n.body if isinstance(m, ast.Assign)
                  for t in m.targets if isinstance(t, ast.Name)}
        known |= _self_attrs(n, store=True)

    faltando = sorted(_self_attrs(_class_node(filename, cls_name),
                                  store=False) - known)
    assert not faltando, f'{cls_name} usa self.X sem origem: {faltando}'


def test_mixin_owns_no_tare_reference():
    """A referência de tare, o auto-tare e o auto-zero são do force_receiver
    (dono da porta serial). Se voltarem para a GUI, a malha de segurança do
    explorer volta a depender do Tk estar responsivo."""
    src = (_PKG / 'gui_loadcell.py').read_text(encoding='utf-8')
    assert '_lc_autozero_rate' not in src
    assert "create_publisher(\n            Float32, '/load_cell/force_net'" not in src
