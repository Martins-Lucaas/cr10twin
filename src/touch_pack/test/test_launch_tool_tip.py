"""A ponteira escolhida no launch (arg `tool`) muda o URDF da simulação, e o
tcp_link do URDF tem de cair onde a IK (constants.TOOLS) acha que está."""
import importlib.util
import os
import re
import xml.etree.ElementTree as ET

import pytest

from ament_index_python.packages import get_package_share_directory
from touch_pack.constants import TOOLS


@pytest.fixture(scope='module')
def share():
    return get_package_share_directory('touch_pack')


@pytest.fixture(scope='module')
def launch_mod(share):
    path = os.path.join(share, 'launch', 'tactile_cell.launch.py')
    spec = importlib.util.spec_from_file_location('tactile_cell_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _robot(launch_mod, share, tool):
    xml = launch_mod._build_touch_tool_suffix(
        '<robot name="cr10"><link name="Link6"/></robot>', share, '', 'real',
        tool)
    return ET.fromstring(xml)


def _flange_to_tcp_mm(robot):
    """Soma o z das juntas fixas de Link6 até tcp_link (todas em rpy 0)."""
    parent_of = {j.find('child').get('link'): j for j in robot.iter('joint')}
    z, link = 0.0, 'tcp_link'
    while link != 'Link6':
        j = parent_of[link]
        z += float(j.find('origin').get('xyz').split()[2])
        link = j.find('parent').get('link')
    return z * 1e3


@pytest.mark.parametrize('tool', sorted(TOOLS))
def test_tcp_do_urdf_bate_com_a_ik(launch_mod, share, tool):
    assert _flange_to_tcp_mm(_robot(launch_mod, share, tool)) == \
        pytest.approx(TOOLS[tool][0], abs=0.05)


@pytest.mark.parametrize('tool', sorted(TOOLS))
def test_mesh_da_ponteira_existe(launch_mod, share, tool):
    tip = next(l for l in _robot(launch_mod, share, tool).iter('link')
               if l.get('name') == 'tool_tip_link')
    mesh = tip.find('visual/geometry/mesh').get('filename')
    assert os.path.isfile(re.sub(r'^file://', '', mesh))
    if tool != 'F':
        assert 'ponteira_F' not in mesh


def test_v2_tem_a_face_de_107_por_20(launch_mod, share):
    tip = next(l for l in _robot(launch_mod, share, 'texturas_v2').iter('link')
               if l.get('name') == 'tool_tip_link')
    face = [c for c in tip.findall('collision') if c.get('name') == 'col_tip_face']
    assert face[0].find('geometry/box').get('size') == '0.107 0.020 0.014'
    assert tip.find('inertial/mass').get('value') == '0.0405'
