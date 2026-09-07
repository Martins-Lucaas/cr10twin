"""Regressão: o carimbo de ferramenta não pode virar uma junta do braço.

`_save_home_pose` grava em `home_pose.json` as 6 juntas MAIS o carimbo
`tool_stamp()` (`tool_tcp_mm`), que registra com que ponteira aquela home foi
ensinada. O bug: o mesmo dict carimbado era atribuído a `self._arm_home_deg`,
então o estado em memória passava a ter uma chave sem slider correspondente.
O `⌂ Home` seguinte iterava as chaves do dict e morria com
`KeyError: 'tool_tcp_mm'` — só depois de "✔ Salvar Home", nunca ao abrir a
GUI, porque `_load_home_pose` filtra por ARM_JOINTS ao ler o arquivo.

Testa os dois métodos sobre um hospedeiro de mentira: nada de Tk, nada de ROS.
"""
import json

import pytest

from touch_pack.constants import ARM_JOINTS, TOOL_STAMP_KEY

PalpationGUI = pytest.importorskip(
    'touch_pack.palpation_gui').PalpationGUI


class _FakeVar:
    """Stand-in de tk.DoubleVar: só get/set."""

    def __init__(self, v=0.0):
        self._v = float(v)

    def get(self):
        return self._v

    def set(self, v):
        self._v = float(v)


class _FakeGUI:
    """Só o que `_save_home_pose` e `_apply_arm_home` tocam."""

    def __init__(self, tmp_path):
        self.arm_sliders = {j: _FakeVar(10.0 + i)
                            for i, j in enumerate(ARM_JOINTS)}
        self._arm_home_deg = {j: 0.0 for j in ARM_JOINTS}
        self._suppressing = False
        self.status = None
        self._home_file = str(tmp_path / 'home_pose.json')
        self.published = 0

    def _set_status(self, msg, *_a, **_kw):
        self.status = msg

    def _publish_arm_from_sliders(self):
        self.published += 1


@pytest.fixture()
def gui(tmp_path, monkeypatch):
    g = _FakeGUI(tmp_path)
    monkeypatch.setattr('touch_pack.palpation_gui.HOME_POSE_FILE',
                        g._home_file, raising=False)
    return g


def test_save_home_keeps_state_joints_only(gui):
    """`_arm_home_deg` fica só com juntas, mesmo com o carimbo no arquivo."""
    PalpationGUI._save_home_pose(gui)

    assert set(gui._arm_home_deg) == set(ARM_JOINTS)
    assert TOOL_STAMP_KEY not in gui._arm_home_deg


def test_save_home_still_stamps_the_file(gui):
    """O carimbo continua indo para o JSON — o fix não pode apagá-lo."""
    PalpationGUI._save_home_pose(gui)

    with open(gui._home_file) as fh:
        data = json.load(fh)
    assert TOOL_STAMP_KEY in data
    for j in ARM_JOINTS:
        assert data[j] == pytest.approx(gui.arm_sliders[j].get())


def test_apply_home_after_save_does_not_raise(gui):
    """O caminho exato do crash: Salvar Home e depois ⌂ Home."""
    PalpationGUI._save_home_pose(gui)
    for var in gui.arm_sliders.values():
        var.set(0.0)

    PalpationGUI._apply_arm_home(gui)          # KeyError antes do fix

    for j in ARM_JOINTS:
        assert gui.arm_sliders[j].get() == pytest.approx(
            gui._arm_home_deg[j])
    assert gui.published == 1
