"""Tara e rotação da força de load_cell_link para o world."""
import numpy as np
import pytest

from rehab_pack.force_source import SOURCES, Tare


def test_tare_subtracts_window_mean():
    t = Tare()
    t.begin()
    for w in ([1, 2, 3, 0, 0, 0], [3, 2, 1, 0, 0, 0]):
        t.feed(w)
    assert t.finish()
    assert np.allclose(t.apply([2, 2, 2, 0, 0, 0]), 0)
    assert np.allclose(t.apply([5, 2, 2, 0, 0, 1])[[0, 5]], [3, 1])


def test_tare_without_samples_fails_and_keeps_offset():
    t = Tare()
    t.offset = np.ones(6)
    t.begin()
    assert not t.finish()
    assert np.allclose(t.offset, 1)


def test_sources_frames():
    assert SOURCES['virtual'][1] == 'world'
    assert SOURCES['sim'][1] == SOURCES['real'][1] == 'load_cell_link'


def test_load_cell_force_rotated_to_world():
    ament = pytest.importorskip('ament_index_python.packages')
    pytest.importorskip('xacro')
    try:
        cra = ament.get_package_share_directory('cra_description')
        rehab = ament.get_package_share_directory('rehab_pack')
    except Exception:
        pytest.skip('pacotes não instalados')
    from rehab_pack.constants import HOME_Q_RAD
    from rehab_pack.kinematics import Chain
    from rehab_pack.urdf_builder import build_robot_urdf
    ch = Chain(build_robot_urdf(cra, rehab)[1], tip='load_cell_link')
    R = ch.fk(HOME_Q_RAD)[:3, :3]
    # em HOME o eixo z da célula aponta para cima no world
    assert (R @ [0, 0, 1])[2] > 0.95
    f_w = R @ np.array([0, 0, 5.0])
    assert np.allclose(f_w, [0, 0, 5.0], atol=0.3)
    assert np.linalg.norm(f_w) == pytest.approx(5.0)
