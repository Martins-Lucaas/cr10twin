"""Rigidez do punho no sim — a baseline que produz descida VERTICAL.

Contexto (01/09/2026): a coleta 20260901_094646 desceu a 24,3° médios da
vertical (máx 65,7°), com J4 ultrapassando o home em 1,25° e ainda frenando
quando o DESCENDING começou. As boas (MANUAL 4x3/4x4 de 31/08, TOUCH de
29–30/08) desceram a 0,0° médios, com X/Y travados em 0,00/0,01 mm contra
16 mm de Z.

O código de controle da descida é IDÊNTICO entre as duas: o `params.json` das
boas ainda traz `hold_df_max_n`/`hold_dx_max_um`, campos que f934d4e ("ports")
apagou do PalpationStart.msg — elas rodaram o binário anterior ao commit, e a
ruim é a primeira execução depois do rebuild de 09:43. Dentro de f934d4e o
tactile_explorer só mudou no controle de força PÓS-CONTATO (_qs_regulate,
_phase_hold, escada); descida em ar livre, HOME, _settle e o passo jacobiano
não foram tocados.

O que mudou foi o ambiente do simulador, em dois pontos que atuam exatamente
onde a falha aparece — J4, em ar livre, antes de tocar em nada:

  erp global 0,25→0,20      no <constraints> do ODE o erp vale para JUNTA
                            (dWorldSetERP), não só para contato: mais baixo,
                            a articulação fica complacente e o punho
                            ultrapassa e oscila.
  disableFixedJointLumping  tira do lumping os 561,5 g da pilha da ferramenta
                            (CoM a ~100 mm do punho), que viram corpo separado
                            ligado por restrição elástica.

Estes testes travam a baseline. Não provam causalidade — isso só a bancada
diz — mas impedem que os valores voltem a afrouxar sem que alguém veja.
"""
import importlib.util
import os
import re

import pytest

from ament_index_python.packages import get_package_share_directory


@pytest.fixture(scope='module')
def share():
    return get_package_share_directory('touch_pack')


@pytest.fixture(scope='module')
def launch_mod(share):
    """O launch não é módulo do pacote (vive em share/launch), então entra
    por caminho — a forma padrão de testar um arquivo de launch."""
    path = os.path.join(share, 'launch', 'tactile_cell.launch.py')
    spec = importlib.util.spec_from_file_location('tactile_cell_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _tool_urdf(launch_mod, share, force_source):
    return launch_mod._build_touch_tool_suffix(
        '<robot name="cr10"></robot>', share, '', force_source)


# ── (a) o punho volta a ser UM corpo rígido no ensaio real ────────────

def test_force_source_real_remove_a_ft_simulada(launch_mod, share):
    """`real` é o default e é o que os ensaios usam (a força vem da célula
    física). Sem o plugin, nada precisa da junta preservada, e o lumping
    devolve os 561,5 g ao elo do punho."""
    urdf = _tool_urdf(launch_mod, share, 'real')
    assert 'disableFixedJointLumping' not in urdf
    assert 'sim_load_cell_ft' not in urdf
    assert 'load_cell_attach' in urdf      # a JUNTA continua no URDF
    assert 'tool_tip_link' in urdf         # e a ferramenta inteira também


def test_force_source_sim_mantem_a_ft_simulada(launch_mod, share):
    """Quem fecha a malha de força no sim ainda precisa dela — o custo em
    rigidez do punho é o preço, e é escolha explícita."""
    urdf = _tool_urdf(launch_mod, share, 'sim')
    assert 'disableFixedJointLumping' in urdf
    assert 'sim_load_cell_ft' in urdf


def test_os_dois_blocos_andam_juntos(launch_mod, share):
    """Nunca um sem o outro: o plugin aponta para `load_cell_attach`, e sem o
    lumping desligado essa junta não existe no SDF."""
    for src in ('real', 'sim'):
        urdf = _tool_urdf(launch_mod, share, src)
        assert ('disableFixedJointLumping' in urdf) \
            == ('sim_load_cell_ft' in urdf), src


def test_valor_desconhecido_cai_no_punho_rigido(launch_mod, share):
    """Mesma regra do launch para force_source: o que não for 'sim' é 'real'.
    Um erro de digitação não pode deixar massa solta no punho em silêncio."""
    assert 'disableFixedJointLumping' not in _tool_urdf(launch_mod, share, '')
    assert 'disableFixedJointLumping' not in _tool_urdf(launch_mod, share, 'Sim')


# ── (b) física do ODE de volta na baseline ────────────────────────────

def test_erp_global_na_baseline(share):
    """O que importa para a descida. Abaixo de 0,25 a correção de erro de
    restrição de JUNTA afrouxa e o punho passa a ultrapassar."""
    world = open(os.path.join(share, 'worlds', 'research_lab.world')).read()
    fisica = re.search(r'<constraints>(.*?)</constraints>', world, re.S).group(1)
    assert float(re.search(r'<erp>([\d.eE+-]+)</erp>', fisica).group(1)) == 0.25


def test_parametros_de_contato_na_baseline(share):
    world = open(os.path.join(share, 'worlds', 'research_lab.world')).read()
    fisica = re.search(r'<constraints>(.*?)</constraints>', world, re.S).group(1)
    cmcv = float(re.search(
        r'<contact_max_correcting_vel>([\d.eE+-]+)<', fisica).group(1))
    csl = float(re.search(
        r'<contact_surface_layer>([\d.eE+-]+)<', fisica).group(1))
    assert (cmcv, csl) == (2.0, 0.001)


def test_iters_e_step_seguem_nos_valores_originais(share):
    """Não foram mexidos pela regressão (70aabbf já tinha revertido iters);
    ficam aqui para o bloco inteiro estar coberto por um teste só."""
    world = open(os.path.join(share, 'worlds', 'research_lab.world')).read()
    assert '<iters>50</iters>' in world
    assert '<max_step_size>0.004</max_step_size>' in world


# ── (d) rigidez de contato da ponteira de volta na baseline ───────────

def test_ponteira_na_rigidez_da_baseline(share):
    """kp 1e6 / minDepth 0,05 mm entraram no mesmo commit que quebrou a
    descida. Re-endurecer é legítimo, mas isolado e remedindo o pico do
    primeiro impacto — contato mais rígido o multiplica à mesma velocidade."""
    urdf = open(os.path.join(share, 'urdf', 'touch_tool_tcp.urdf')).read()
    tip = re.search(
        r'<gazebo reference="tool_tip_link">(.*?)</gazebo>', urdf, re.S).group(1)
    assert float(re.search(r'<kp>([\d.eE+-]+)</kp>', tip).group(1)) == 1.0e5
    assert float(
        re.search(r'<minDepth>([\d.eE+-]+)</minDepth>', tip).group(1)) == 2.0e-4


# ── (c) os clamps do xacro: NADA a restaurar ──────────────────────────

def test_command_interface_do_braco_sem_clamp_placeholder():
    """f934d4e removeu `min/max ±1 rad` dos <command_interface>. Parece grande,
    mas era placeholder do fork Dobot e PROVADAMENTE não mordia: as coletas
    boas rodaram o binário que AINDA tinha os clamps e mesmo assim
    registraram joint1 em 1,0671 rad, joint3 em −2,5726 e joint5 em 1,5708 —
    todos fora de [−1,1]. Restaurá-los não desfaria regressão nenhuma e
    devolveria um limite errado ao caminho de comando; sem eles o
    gazebo_ros2_control usa os <limit> reais da junta. Este teste existe para
    que ninguém os traga de volta 'para igualar a baseline'."""
    xacro_path = os.path.join(
        get_package_share_directory('cra_description'), 'urdf',
        'cr10_robot.xacro')
    corpo = open(xacro_path).read()
    ros2_control = re.search(
        r'<ros2_control\b.*?</ros2_control>', corpo, re.S).group(0)
    for j in range(1, 7):
        bloco = re.search(
            rf'<joint name="joint{j}">(.*?)</joint>', ros2_control, re.S).group(1)
        cmd = re.search(r'<command_interface name="position".*?(?:/>|</command_interface>)',
                        bloco, re.S).group(0)
        assert 'name="min"' not in cmd and 'name="max"' not in cmd, j
