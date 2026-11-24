"""FK/jacobiana/DLS sobre a cadeia world → tcp_link do URDF gerado."""
import numpy as np
import pytest

from rehab_pack.constants import HOME_Q_RAD

pytest.importorskip('xacro')
ament = pytest.importorskip('ament_index_python.packages')
try:
    CRA = ament.get_package_share_directory('cra_description')
    REHAB = ament.get_package_share_directory('rehab_pack')
except Exception:  # pragma: no cover
    pytest.skip('cra_description/rehab_pack não instalados', allow_module_level=True)

from rehab_pack.kinematics import Chain  # noqa: E402
from rehab_pack.urdf_builder import build_robot_urdf  # noqa: E402


@pytest.fixture(scope='module')
def chain():
    return Chain(build_robot_urdf(CRA, REHAB)[1])


def test_chain_has_six_joints_in_order(chain):
    assert chain.names == [f'joint{i}' for i in range(1, 7)]
    assert chain.axes[0] == [0.0, 0.0, -1.0]  # joint1 gira ao contrário


def test_zero_pose_is_rigid_and_finite(chain):
    T = chain.fk(np.zeros(6))
    assert np.all(np.isfinite(T))
    assert np.allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-9)


def test_jacobian_matches_finite_difference(chain):
    q = np.array(HOME_Q_RAD) + 0.05
    J = chain.jacobian(q)
    dq = np.array([0.3, -0.2, 0.1, 0.4, -0.3, 0.2]) * 1e-4
    dx = chain.fk(q + dq)[:3, 3] - chain.fk(q)[:3, 3]
    assert np.allclose(J[:3] @ dq, dx, atol=1e-8)


def test_dls_converges_below_1mm_with_fixed_orientation(chain):
    q0 = np.array(HOME_Q_RAD)
    T0 = chain.fk(q0)
    target = T0[:3, 3] + [0.05, -0.04, 0.03]
    q, res = chain.ik(q0, target, T0[:3, :3], iters=50)
    assert res < 1e-3
    assert np.allclose(chain.fk(q)[:3, :3], T0[:3, :3], atol=5e-3)
