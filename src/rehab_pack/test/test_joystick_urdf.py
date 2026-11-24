"""Mastro acoplado na FA7155: geometria da pilha, meshes e montagem do URDF."""
import os
import re
import xml.etree.ElementTree as ET

import pytest

PKG = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
URDF = os.path.join(PKG, 'urdf', 'joystick_tool.urdf')


def _joints(root):
    # o bloco ros2_control também tem <joint>, sem parent/child: fora.
    return {j.get('name'): j for j in root.iter('joint')
            if j.find('child') is not None}


def _z(joint):
    return float(joint.find('origin').get('xyz').split()[2])


def test_stack_heights_match_the_cad():
    j = _joints(ET.parse(URDF).getroot())
    # flange → célula 0,4 mm; célula → placa 36,8 mm (a mesma altura em que o
    # acoplador hot-swap do touch_pack encosta); placa → empunhadura 2 mm
    # (espessura da placa); tcp no meio da empunhadura (150 / 2).
    assert _z(j['load_cell_attach']) == pytest.approx(0.0004)
    assert _z(j['joystick_plate_attach']) == pytest.approx(0.0368)
    assert _z(j['joystick_grip_attach']) == pytest.approx(0.002)
    total = sum(_z(j[n]) for n in ('ft_flange_attach', 'load_cell_attach',
                                   'joystick_plate_attach',
                                   'joystick_grip_attach', 'tcp_alias_joint'))
    assert total == pytest.approx(0.1142)


def test_every_mesh_exists():
    for m in re.findall(r'package://rehab_pack/(meshes/[^"]+)',
                        open(URDF, encoding='utf-8').read()):
        assert os.path.isfile(os.path.join(PKG, m)), m


def test_grip_is_rubber_and_heavier_per_volume_than_the_plate():
    root = ET.parse(URDF).getroot()
    links = {l.get('name'): l for l in root.iter('link')}
    mass = lambda n: float(links[n].find('inertial/mass').get('value'))
    assert mass('joystick_grip_link') > mass('joystick_plate_link')
    gz = {g.get('reference'): g for g in root.iter('gazebo')
          if g.get('reference')}
    mu = lambda n: float(gz[n].find('mu1').text)
    assert mu('joystick_grip_link') > mu('joystick_plate_link')


def test_mesh_dimensions_match_the_step():
    trimesh = pytest.importorskip('trimesh')
    plate = trimesh.load(os.path.join(PKG, 'meshes', 'joystick_placa.stl'))
    grip = trimesh.load(os.path.join(PKG, 'meshes', 'joystick_empunhadura.stl'))
    assert plate.extents == pytest.approx([110, 50, 2], abs=0.05)
    assert grip.extents == pytest.approx([35, 35, 150], abs=0.05)
    # origem no centro da face de baixo
    assert plate.bounds[0] == pytest.approx([-55, -25, 0], abs=0.05)
    assert grip.bounds[0][2] == pytest.approx(0, abs=0.05)


@pytest.fixture(scope='module')
def cra_share():
    pytest.importorskip('xacro')
    ament = pytest.importorskip('ament_index_python.packages')
    try:
        return ament.get_package_share_directory('cra_description')
    except Exception:
        pytest.skip('cra_description não está no ambiente (source install/)')


@pytest.mark.parametrize('sim_ft', [False, True])
def test_built_urdf_attaches_the_tool_to_link6(cra_share, sim_ft):
    from rehab_pack.urdf_builder import build_robot_urdf
    full, minimal = build_robot_urdf(cra_share, PKG, sim_ft=sim_ft)
    root = ET.fromstring(full)
    joints = _joints(root)
    parent = {j.find('child').get('link'): j.find('parent').get('link')
              for j in joints.values()}
    # a ferramenta pendura no Link6 do braço
    assert parent['ft_flange_link'] == 'Link6'
    assert parent['tcp_link'] == 'joystick_grip_link'
    # andaime standalone não vaza para o robô
    assert 'world_to_base' not in joints
    assert len([l for l in root.iter('link') if l.get('name') == 'Link6']) == 1
    # marcadores nunca ficam no URDF gerado
    assert 'SIM_FT' not in full
    assert ('sim_load_cell_ft' in full) == sim_ft
    if sim_ft:
        assert 'joystick_plate_attach' in joints
    # o mínimo não carrega geometria
    assert '<visual' not in minimal and '<collision' not in minimal
    assert 'tcp_link' in minimal
