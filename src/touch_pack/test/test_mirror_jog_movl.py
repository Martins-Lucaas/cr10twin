"""O mirror manda MovL só para o alvo do jog cartesiano; o resto vai em MovJ."""
import threading

import numpy as np
import pytest

gui_robot = pytest.importorskip('touch_pack.gui_robot')


class _Drv:
    def __init__(self):
        self.calls = []

    def _send_dash(self, _cmd):
        return '0'

    def mov_l_joint_deg(self, q):
        self.calls.append('L')

    def mov_j_joint_deg(self, q):
        self.calls.append('J')


class _Var:
    def get(self):
        return 50


class _Gui(gui_robot.RobotMixin):
    def __init__(self, jog_q):
        self._real_lock = threading.Lock()
        self._lock = threading.Lock()
        self._real_driver = _Drv()
        self._robot_connected = True
        self._robot_mode = 'MIRROR'
        self._latest_phase = 'IDLE'
        self._drag_enabled = False
        self.speed_factor_var = _Var()
        self._jog_linear_q = jog_q


@pytest.mark.parametrize('jog_q, esperado', [
    ([0.1, -0.2, -2.6, 1.4, 1.57, -0.15], ['L']),   # alvo do jog → MovL
    ([0.0, -0.2, -2.6, 1.4, 1.57, -0.15], ['J']),   # outro alvo → MovJ
    (None, ['J']),                                  # sem jog → MovJ
])
def test_mirror_escolhe_movl_so_para_o_jog(jog_q, esperado):
    g = _Gui(jog_q)
    g._mirror_movj_send([0.1, -0.2, -2.6, 1.4, 1.57, -0.15])
    assert g._real_driver.calls == esperado
