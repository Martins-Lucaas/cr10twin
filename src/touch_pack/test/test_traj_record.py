"""Gravação do arrasto: o que vira waypoint e o que a gravação produz.

O filtro é a peça que decide a fidelidade do caminho gravado. Frouxo demais,
some a curva que a mão fez; apertado demais, o JSON enche de poses idênticas
de braço parado e a reprodução vira uma sequência de `MovJ` de 2 mm cada.

Pula sozinho onde não há display (CI headless).
"""
import pathlib
import sys
import tkinter as tk

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from touch_pack import gui_trajectory                    # noqa: E402
from touch_pack.gui_trajectory import (                  # noqa: E402
    TrajRecMixin, should_keep, TRAJ_TOL_DEG,
)


def test_primeira_pose_sempre_entra():
    assert should_keep(None, [0.0] * 6) is True


def test_braco_parado_nao_vira_waypoint():
    q = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
    assert should_keep(q, list(q)) is False
    # Ruído do encoder abaixo da tolerância também não.
    tremido = [v + TRAJ_TOL_DEG * 0.4 for v in q]
    assert should_keep(q, tremido) is False


def test_uma_junta_que_andou_basta():
    q = [0.0] * 6
    andou = [0.0, 0.0, 0.0, 0.0, 0.0, TRAJ_TOL_DEG + 0.1]
    assert should_keep(q, andou) is True


@pytest.fixture(scope='module')
def _tk_root():
    try:
        r = tk.Tk()
    except tk.TclError as exc:                       # pragma: no cover
        pytest.skip(f'sem display: {exc}')
    r.withdraw()
    yield r
    r.destroy()


class _SemAfter:
    """Proxy do Frame que ENGOLE o `after`.

    O teste chama `_traj_tick` à mão, e cada tick reagenda o próximo. Com um
    `after` de verdade sobrariam callbacks pendentes disparando depois do
    teardown, em cima de um widget já destruído — o Tcl derruba o processo
    inteiro no fim da sessão, e a sessão inteira vira "dumped core" sem dizer
    de onde veio.
    """

    def __init__(self, frame):
        self._frame = frame

    def after(self, *_a, **_k):
        return None

    def after_cancel(self, *_a, **_k):
        return None

    def __getattr__(self, name):
        return getattr(self._frame, name)


class _Host(TrajRecMixin):
    """O mínimo que o mixin espera do PalpationGUI."""

    def __init__(self, raiz):
        self.root = _SemAfter(raiz)
        self._latest_joint_rad = [0.0] * 6
        self._poses: list[dict] = []
        self._movements: list[dict] = []
        self._next_pose_id = 1
        self._next_movement_id = 1
        self.salvou = 0
        self.status: list[str] = []
        self.nome = 'Varredura'
        self._build_traj_card(raiz)

    def _card(self, parent, _title, *, expand=True):
        f = tk.Frame(parent)
        f.pack(fill='x')
        return f

    def _set_status(self, txt, _color=''):
        self.status.append(txt)

    def _ask_name_dialog(self, _titulo, _default):
        return self.nome

    def _save_poses_data(self):
        self.salvou += 1

    def _refresh_poses_list(self):
        pass

    def _refresh_movements_list(self, select_id=None):
        pass

    def _refresh_movement_detail(self, _mov):
        pass

    # auxiliar do teste: um tick com o braço numa pose dada (graus)
    def mover_e_amostrar(self, graus):
        import math
        self._latest_joint_rad = [math.radians(v) for v in graus]
        self._traj_tick()


@pytest.fixture
def host(_tk_root):
    f = tk.Frame(_tk_root)
    f.pack()
    h = _Host(f)
    yield h
    h._traj_recording = False
    f.destroy()


def test_gravacao_vira_movimento_com_os_waypoints(host):
    host._traj_start()
    for j1 in (0.0, 5.0, 10.0, 15.0):
        host.mover_e_amostrar([j1, 0, 0, 0, 0, 0])
    host._traj_stop()

    assert len(host._movements) == 1
    mov = host._movements[0]
    assert mov['name'] == 'Varredura'
    assert len(mov['pose_ids']) == len(host._poses)
    assert [p['q_deg'][0] for p in host._poses] == [0.0, 5.0, 10.0, 15.0]
    # UM save para o caminho inteiro, não um por waypoint.
    assert host.salvou == 1


def test_braco_parado_nao_gera_movimento(host):
    host._traj_start()
    for _ in range(5):
        host.mover_e_amostrar([0, 0, 0, 0, 0, 0])
    host._traj_stop()

    assert host._movements == []
    assert 'did not move' in host.status[-1]


def test_teto_de_waypoints_interrompe_a_gravacao(host, monkeypatch):
    monkeypatch.setattr(gui_trajectory, 'TRAJ_MAX_POINTS', 3)
    host._traj_start()
    for j1 in (0.0, 5.0, 10.0, 15.0, 20.0):
        host.mover_e_amostrar([j1, 0, 0, 0, 0, 0])

    assert host._traj_recording is False
    assert len(host._movements) == 1
    assert len(host._movements[0]['pose_ids']) == 3


def test_recusa_gravar_sem_feedback_de_junta(host):
    host._latest_joint_rad = None

    host._traj_start()

    assert host._traj_recording is False
    assert 'No joint feedback' in host.status[-1]
