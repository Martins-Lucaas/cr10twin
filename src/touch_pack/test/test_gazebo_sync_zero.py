"""Conectar ao CR10 nunca manda o braço real se mover (nem para zero)."""
import threading
from types import SimpleNamespace

import numpy as np
import pytest

gui_robot = pytest.importorskip('touch_pack.gui_robot')
pg = pytest.importorskip('touch_pack.palpation_gui')

# Home padrão do touch_pack (a do rehab_pack é outra e não entra aqui).
HOME = np.radians([pg.ARM_HOME_DEG[j] for j in gui_robot.ARM_JOINTS])


class _Gui(gui_robot.RobotMixin):
    _robot_mode, _latest_phase = 'MIRROR', 'IDLE'
    _cam_arm_gate = _exec_movement_id = None
    _drag_enabled = _manip_active = _imu_tracking = _mirror_following = False

    def __init__(self):
        self._lock = threading.Lock()
        self.movj, self.pub, self.status = [], [], []
        self._arm_pub = SimpleNamespace(publish=self._publica)
        self.root = SimpleNamespace(after=lambda _t, f, *a: f(*a))
        self.arm_sliders = {j: SimpleNamespace(set=lambda v: None)
                            for j in gui_robot.ARM_JOINTS}

    def _publica(self, msg):
        self.pub.append(list(msg.points[-1].positions))
        pg.PalpationGUI._cb_arm_trajectory(self, msg)     # o eco do tópico

    def _mirror_movj_debounced(self, q):
        self.movj.append(q)

    def _set_status(self, *a):
        self.status.append(a)

    def get_logger(self):
        return SimpleNamespace(**{k: lambda *a: None for k in
                                  ('info', 'warning', 'error')})


def _sync(monkeypatch, leituras):
    monkeypatch.setattr(gui_robot.time, 'sleep', lambda s: None)
    it = iter(leituras)
    ult = [None]

    def le():
        ult[0] = next(it, ult[0])
        return ult[0]
    g = _Gui()
    g._sync_gazebo_to_real(SimpleNamespace(
        read_joints_urdf_latest=le, read_joints_urdf=le))
    return g


def test_leitura_zerada_e_descartada_e_tenta_de_novo(monkeypatch):
    g = _sync(monkeypatch, [np.zeros(6), np.zeros(6), HOME])
    assert np.allclose(g.pub, [HOME]) and g.movj == []


def test_so_zeros_nao_move_nada_e_avisa(monkeypatch):
    g = _sync(monkeypatch, [np.zeros(6)])
    assert g.pub == [] and g.movj == [] and g.status


def test_slider_e_home_depois_da_sync_continuam_movendo_o_real(monkeypatch):
    g = _sync(monkeypatch, [HOME])
    pt = SimpleNamespace(positions=[float(v) for v in HOME], velocities=[])
    pg.PalpationGUI._cb_arm_trajectory(g, SimpleNamespace(points=[pt]))
    assert np.allclose(g.movj, [HOME])
