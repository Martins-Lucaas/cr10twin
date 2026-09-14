"""A cola cinemática do Gazebo: o objeto tem de andar RÍGIDO com a mão.

O nó existe porque Gazebo Classic não segura objeto por atrito com uma mão
underactuada. Ele captura o offset mão→objeto no instante do attach e
reescreve a pose do objeto a 100 Hz. O contrato é uma coisa só: enquanto
colado, a transformação relativa entre mão e objeto NÃO muda — nem em
translação pura, nem em rotação, nem nas duas juntas.

Errar isso não dá erro: o objeto escorrega, orbita ou entra dentro da mão ao
longo do movimento, e o vídeo do ensaio sai errado sem nenhuma linha de log.

Os métodos são exercitados sobre um dublê em vez de um `Node` real: o
construtor espera 5 s por `/gazebo/set_entity_state` e depende do Gazebo no
ar. A matemática, que é onde mora o risco, não depende de nada disso.
"""
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

pytest.importorskip('rclpy')
pytest.importorskip('gazebo_msgs')

from touch_pack.kinematic_attacher import (                  # noqa: E402
    KinematicAttacher, _q_conj, _q_mul, _q_rot,
)

_ID = np.array([0.0, 0.0, 0.0, 1.0])


def _quat_z(ang_rad: float) -> np.ndarray:
    """Rotação de `ang_rad` em torno de Z, em (x, y, z, w)."""
    return np.array([0.0, 0.0, np.sin(ang_rad / 2), np.cos(ang_rad / 2)])


# ── Quaternions (implementação própria, sem scipy) ────────────────────
def test_identity_is_neutral():
    q = _quat_z(0.7)
    assert _q_mul(q, _ID) == pytest.approx(q)
    assert _q_mul(_ID, q) == pytest.approx(q)


def test_conjugate_undoes_the_rotation():
    q = _quat_z(1.1)
    assert _q_mul(q, _q_conj(q)) == pytest.approx(_ID, abs=1e-12)


def test_rotation_matches_the_known_case():
    """90° em Z leva +X para +Y. Um sinal trocado em `_q_mul` passaria em
    todos os testes de norma e só apareceria aqui."""
    v = _q_rot(_quat_z(np.pi / 2), np.array([1.0, 0.0, 0.0]))
    assert v == pytest.approx([0.0, 1.0, 0.0], abs=1e-12)


def test_rotation_preserves_length():
    v = np.array([0.3, -0.2, 0.7])
    out = _q_rot(_quat_z(0.4), v)
    assert np.linalg.norm(out) == pytest.approx(np.linalg.norm(v))


def test_composition_order_is_not_commutative():
    """Guarda contra alguém "simplificar" `_q_mul(qh_inv, q_o)` invertendo a
    ordem: dá o mesmo resultado só quando os eixos coincidem."""
    a = _quat_z(0.5)
    b = np.array([np.sin(0.25), 0.0, 0.0, np.cos(0.25)])   # 0,5 rad em X
    assert _q_mul(a, b) != pytest.approx(_q_mul(b, a))


# ── attach / detach / tick ────────────────────────────────────────────
class _Cli:
    """Dublê do cliente de /gazebo/set_entity_state: guarda as requisições."""

    def __init__(self):
        self.reqs = []

    def call_async(self, req):
        self.reqs.append(req)


class _Resp:
    success = False
    message = ''


class _Attacher:
    """Portador de estado que reexpõe os métodos reais via `self`."""

    def __init__(self, hand=None, obj=None):
        self._obj = 'pick_object'
        self._link = 'hand_base_link'
        self._hand_pose = hand
        self._obj_pose = obj
        self._active = False
        self._p_off = np.zeros(3)
        self._q_off = _ID.copy()
        self._cli = _Cli()
        self._publicados = []
        self._pub = type('_P', (), {
            'publish': lambda _s, m: self._publicados.append(bool(m.data)),
        })()

    def get_logger(self):
        return type('_L', (), {'info': lambda _s, *a, **k: None,
                               'warn': lambda _s, *a, **k: None,
                               'error': lambda _s, *a, **k: None})()

    # `_as_pose` é estático lá; sem o wrapper o `self` viraria o primeiro
    # argumento e a pose chegaria deslocada de uma posição.
    _as_pose = staticmethod(KinematicAttacher._as_pose)
    _cb_attach = KinematicAttacher._cb_attach
    _cb_detach = KinematicAttacher._cb_detach
    _cb_links = KinematicAttacher._cb_links
    _cb_models = KinematicAttacher._cb_models
    _tick = KinematicAttacher._tick


def _pose(p, q):
    return (np.asarray(p, float), np.asarray(q, float))


def _pose_enviada(cli):
    st = cli.reqs[-1].state.pose
    return (np.array([st.position.x, st.position.y, st.position.z]),
            np.array([st.orientation.x, st.orientation.y,
                      st.orientation.z, st.orientation.w]))


def test_attach_without_poses_is_refused():
    """Sem /gazebo/*_states no ar, colar capturaria um offset de lixo e o
    objeto saltaria para a origem no primeiro tick."""
    a = _Attacher()
    resp = a._cb_attach(None, _Resp())
    assert resp.success is False
    assert a._active is False


def test_attach_without_the_gazebo_service_is_refused():
    """Mundo sem libgazebo_ros_state.so: melhor recusar com mensagem do que
    marcar colado e nunca mover nada."""
    a = _Attacher(hand=_pose([0, 0, 1], _ID), obj=_pose([0, 0, 1], _ID))
    a._cli = None
    resp = a._cb_attach(None, _Resp())
    assert resp.success is False
    assert a._active is False


def test_attached_object_does_not_move_while_the_hand_is_still():
    """O caso mais básico e o mais fácil de quebrar: recolocar o objeto na
    própria pose. Qualquer erro de sinal no offset aparece como um salto já
    no primeiro tick."""
    a = _Attacher(hand=_pose([1, 0, 0.5], _quat_z(0.3)),
                  obj=_pose([1.1, 0.05, 0.55], _quat_z(0.8)))
    assert a._cb_attach(None, _Resp()).success is True
    a._tick()
    p, q = _pose_enviada(a._cli)
    assert p == pytest.approx([1.1, 0.05, 0.55], abs=1e-12)
    assert q == pytest.approx(_quat_z(0.8), abs=1e-12)


def test_object_follows_a_pure_translation():
    a = _Attacher(hand=_pose([0, 0, 1], _ID), obj=_pose([0.1, 0, 1], _ID))
    a._cb_attach(None, _Resp())
    a._hand_pose = _pose([0.5, -0.2, 1.3], _ID)
    a._tick()
    p, _q = _pose_enviada(a._cli)
    assert p == pytest.approx([0.6, -0.2, 1.3], abs=1e-12)


def test_object_orbits_with_a_pure_rotation():
    """Girar a mão 90° em Z com o objeto 10 cm à frente em +X tem de levá-lo
    para +Y. Se o offset fosse aplicado no mundo em vez de no frame da mão, o
    objeto ficaria parado enquanto a mão gira — e atravessaria os dedos."""
    a = _Attacher(hand=_pose([0, 0, 1], _ID), obj=_pose([0.1, 0, 1], _ID))
    a._cb_attach(None, _Resp())
    a._hand_pose = _pose([0, 0, 1], _quat_z(np.pi / 2))
    a._tick()
    p, q = _pose_enviada(a._cli)
    assert p == pytest.approx([0.0, 0.1, 1.0], abs=1e-12)
    assert q == pytest.approx(_quat_z(np.pi / 2), abs=1e-12)


@pytest.mark.parametrize('ang', [0.0, 0.7, np.pi / 2, -1.3, np.pi])
def test_relative_transform_is_invariant_under_any_hand_motion(ang):
    """A propriedade que define "rígido": a distância e a orientação
    relativas medidas depois do movimento são as mesmas de antes, para
    qualquer pose da mão."""
    p_h0, q_h0 = np.array([0.2, -0.1, 0.9]), _quat_z(0.25)
    p_o0, q_o0 = np.array([0.3, 0.05, 1.0]), _quat_z(0.9)
    a = _Attacher(hand=_pose(p_h0, q_h0), obj=_pose(p_o0, q_o0))
    a._cb_attach(None, _Resp())

    p_h1, q_h1 = np.array([-0.4, 0.6, 1.4]), _q_mul(_quat_z(ang), q_h0)
    a._hand_pose = _pose(p_h1, q_h1)
    a._tick()
    p_o1, q_o1 = _pose_enviada(a._cli)

    # Offset visto do frame da mão, antes e depois.
    off0 = _q_rot(_q_conj(q_h0), p_o0 - p_h0)
    off1 = _q_rot(_q_conj(q_h1), p_o1 - p_h1)
    assert off1 == pytest.approx(off0, abs=1e-12)

    rel0 = _q_mul(_q_conj(q_h0), q_o0)
    rel1 = _q_mul(_q_conj(q_h1), q_o1)
    # ±q descrevem a mesma rotação — comparar pelo sinal do escalar.
    if np.dot(rel0, rel1) < 0:
        rel1 = -rel1
    assert rel1 == pytest.approx(rel0, abs=1e-12)


def test_detach_stops_rewriting_the_pose():
    """Soltar tem de parar de escrever de verdade: um tick a mais depois do
    detach prenderia o objeto no ar por 10 ms e mataria a queda."""
    a = _Attacher(hand=_pose([0, 0, 1], _ID), obj=_pose([0.1, 0, 1], _ID))
    a._cb_attach(None, _Resp())
    a._tick()
    n = len(a._cli.reqs)

    assert a._cb_detach(None, _Resp()).success is True
    a._hand_pose = _pose([5, 5, 5], _ID)
    a._tick()
    assert len(a._cli.reqs) == n


def test_attached_flag_is_published_every_tick():
    """O tópico é o que a GUI lê para saber se o objeto está preso — ele tem
    de sair mesmo nos ticks em que nada se move."""
    a = _Attacher()
    a._tick()
    a._tick()
    assert a._publicados == [False, False]


def test_link_is_matched_by_suffix_not_by_equality():
    """O Gazebo publica `<modelo>::<elo>`; comparar com `==` nunca casaria e
    o attach ficaria permanentemente sem pose da mão."""
    a = _Attacher()
    msg = type('_M', (), {
        'name': ['cr10::base_link', 'cr10::hand_base_link'],
        'pose': [_Pose(0, 0, 0), _Pose(1, 2, 3)],
    })()
    a._cb_links(msg)
    assert a._hand_pose is not None
    assert a._hand_pose[0] == pytest.approx([1, 2, 3])


def test_missing_object_clears_the_stale_pose():
    """Objeto removido do mundo: guardar a última pose faria o attach
    seguinte colar contra uma posição que não existe mais."""
    a = _Attacher(obj=_pose([9, 9, 9], _ID))
    msg = type('_M', (), {'name': ['outra_coisa'], 'pose': [_Pose(0, 0, 0)]})()
    a._cb_models(msg)
    assert a._obj_pose is None


class _Pose:
    def __init__(self, x, y, z):
        self.position = type('_P', (), {'x': float(x), 'y': float(y),
                                        'z': float(z)})()
        self.orientation = type('_O', (), {'x': 0.0, 'y': 0.0, 'z': 0.0,
                                           'w': 1.0})()
