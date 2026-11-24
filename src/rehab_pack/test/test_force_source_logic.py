"""Tara e rotação da força de load_cell_link para o world."""
import numpy as np
import pytest

from rehab_pack.force_source import SOURCES, Tare, rot_z6


def test_tare_subtracts_window_mean():
    t = Tare()
    t.begin()
    for w in 10 * ([1, 2, 3, 0, 0, 0], [3, 2, 1, 0, 0, 0]):
        t.feed(w)
    assert t.finish() == ''
    assert np.allclose(t.apply([2, 2, 2, 0, 0, 0]), 0)
    assert np.allclose(t.apply([5, 2, 2, 0, 0, 1])[[0, 5]], [3, 1])


def test_tare_without_samples_fails_and_keeps_offset():
    t = Tare()
    t.offset = np.ones(6)
    t.begin()
    assert t.finish()
    assert np.allclose(t.offset, 1)


@pytest.mark.parametrize('fx', [np.linspace(0, 3, 40), np.full(40, np.nan)])
def test_tare_recusa_mao_no_mastro_e_nan(fx):
    t = Tare()
    t.begin()
    for v in fx:
        t.feed([v, 0, 0, 0, 0, 0])
    assert t.finish() and np.allclose(t.offset, 0)


def test_sources_frames():
    assert set(SOURCES) == {'sim', 'real'}
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


def test_giro_da_montagem_gira_forca_e_torque():
    w = rot_z6(np.pi / 2) @ [1, 0, 5, 0, 2, 3]
    assert np.allclose(w, [0, 1, 5, -2, 0, 3])


@pytest.mark.parametrize('source,sinal,giro', [('real', -1.0, 30.0), ('sim', 1.0, 0.0)])
def test_publica_o_sinal_e_o_giro_que_usa(source, sinal, giro, monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    import json
    from rclpy.publisher import Publisher
    from rehab_pack.constants import T_FT_CONFIG
    from rehab_pack.force_source import ForceSource
    pubs = []
    monkeypatch.setattr(Publisher, 'publish', lambda s, m: pubs.append((s.topic_name, m)))
    rclpy.init(domain_id=78, args=['--ros-args', '-p', f'source:={source}',
                                   '-p', 'real_sign:=-1.0', '-p', 'real_yaw_deg:=30.0'])
    try:
        n = ForceSource()
        cfg, = [json.loads(m.data) for t, m in pubs if t == T_FT_CONFIG]
        assert cfg == {'ft_sign': sinal, 'ft_yaw_deg': giro}
        assert np.allclose(n._map, sinal * rot_z6(np.radians(giro)))
        n.destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('mod', ['admittance_node', 'force_source', 'session_logger'])
def test_main_toma_a_trava_antes_do_ros_e_falha_fechado(mod, monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    import importlib
    import sys
    from touch_pack import pack_lock
    m = importlib.import_module(f'rehab_pack.{mod}')
    monkeypatch.setattr(rclpy, 'init', lambda *a, **k: pytest.fail('rclpy.init sem a trava'))
    pedidos = []

    def ocupado(pack):
        pedidos.append(pack)
        raise SystemExit('touch_pack aberto')
    monkeypatch.setattr(pack_lock, 'acquire_or_exit', ocupado)
    with pytest.raises(SystemExit):
        m.main()
    assert pedidos == ['rehab_pack']
    monkeypatch.setitem(sys.modules, 'touch_pack.pack_lock', None)   # sem touch_pack
    with pytest.raises(ImportError):
        m.main()
