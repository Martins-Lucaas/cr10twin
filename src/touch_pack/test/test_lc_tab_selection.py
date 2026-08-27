"""A aba "Load Cell" tem de ser a da célula que está no cabo.

Duas células convivem no repo e cada uma tem a sua aba. O erro que este arquivo
trava é silencioso e caro: a tela mostrar os painéis de uma célula enquanto o
dado vem da outra. Com a viga S no cabo, a aba de seis eixos exibiria seis
zeros perfeitamente convincentes (ninguém publica `/ft_sensor/wrench`), e quem
estiver olhando vai concluir que a célula morreu.

Análise por AST, como no `test_gui_split.py`: não importa `palpation_gui` (que
exige rclpy + Tk), então roda em qualquer máquina.
"""
import ast
import pathlib

import pytest

_PKG = pathlib.Path(__file__).resolve().parents[1] / 'touch_pack'
_LAUNCH = (pathlib.Path(__file__).resolve().parents[1]
           / 'launch' / 'tactile_cell.launch.py')


def _func(filename: str, cls_name: str, fn_name: str) -> ast.FunctionDef:
    tree = ast.parse((_PKG / filename).read_text(encoding='utf-8'), filename)
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == cls_name:
            for m in node.body:
                if (isinstance(m, ast.FunctionDef) and m.name == fn_name):
                    return m
    raise AssertionError(f'{cls_name}.{fn_name} não existe em {filename}')


def _usados(node: ast.AST) -> set[str]:
    """Todo `self.X` LIDO dentro do nó — chamado ou só referenciado.

    Os dois contam: `self._build_lc_axes_tab(...)` é chamada, mas
    `self.root.after(80, self._refresh_ft_axes)` só passa a referência, e é
    assim que os loops de repintura são armados em toda a GUI.
    """
    return {a.attr for a in ast.walk(node)
            if isinstance(a, ast.Attribute) and isinstance(a.value, ast.Name)
            and a.value.id == 'self' and isinstance(a.ctx, ast.Load)}


@pytest.fixture(scope='module')
def build_tab():
    return _func('palpation_gui.py', 'PalpationGUI', '_build_loadcell_tab')


def test_a_aba_olha_para_a_celula_selecionada(build_tab):
    """Sem o teste de `_force_sensor` a função volta a montar sempre a mesma
    coisa, e o bug reaparece sem barulho nenhum."""
    assert '_force_sensor' in _usados(build_tab)


def test_monta_as_duas_abas_da_celula_axial(build_tab):
    usados = _usados(build_tab)
    assert '_build_lc_reading_tab' in usados
    assert '_build_lc_calibration_tab' in usados


def test_monta_a_aba_de_seis_eixos_da_fa7155(build_tab):
    usados = _usados(build_tab)
    assert '_build_lc_axes_tab' in usados
    # O loop de repintura dos seis eixos é armado AQUI (o da célula axial é
    # armado dentro do próprio `_build_lc_reading_tab`). Montar a aba sem
    # armá-lo dá uma tela que nasce certa e congela.
    assert '_refresh_ft_axes' in usados


def test_o_ramo_da_fa7155_nao_cai_no_da_axial(build_tab):
    """O ramo `ft6` termina em `return`. Sem ele a função seguiria e montaria
    as três sub-abas — que é exatamente o que este arquivo existe para
    impedir, e um `if` sem `return` parece correto na leitura rápida."""
    ramos = [n for n in build_tab.body if isinstance(n, ast.If)]
    assert ramos, '_build_loadcell_tab deixou de ramificar'
    assert any(isinstance(x, ast.Return) for x in ast.walk(ramos[0]))


def test_o_default_do_parametro_e_a_celula_da_bancada():
    """`palpation_gui` rodando solto (sem o launch) tem de assumir a célula
    que está montada, não a outra."""
    src = (_PKG / 'palpation_gui.py').read_text(encoding='utf-8')
    assert "declare_parameter(\n            'force_sensor', 'load_cell')" in src


def test_o_launch_manda_o_mesmo_valor_para_a_gui_e_para_o_driver():
    """O acoplamento que realmente importa: a GUI e o receiver têm de receber
    a MESMA escolha. Se o launch passar um literal para a tela e a variável
    para o nó, os dois divergem no primeiro `force_sensor:=ft6`."""
    src = _LAUNCH.read_text(encoding='utf-8')
    assert "'force_sensor': force_sensor," in src
    # E o valor vem do argumento de launch, validado num lugar só.
    assert "LaunchConfiguration(\n        'force_sensor').perform(context)" in src
    assert "if force_sensor not in ('load_cell', 'ft6'):" in src


def test_valor_desconhecido_cai_na_celula_da_bancada():
    """Erro de digitação não pode subir o driver de uma placa que não está no
    cabo — nem mostrar a aba dela. Launch e GUI caem no mesmo default."""
    launch = _LAUNCH.read_text(encoding='utf-8')
    gui = (_PKG / 'palpation_gui.py').read_text(encoding='utf-8')
    assert "force_sensor = 'load_cell'" in launch
    assert "else 'load_cell'" in gui
