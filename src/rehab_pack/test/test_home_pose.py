"""HOME_Q_RAD: empunhadura para cima, tcp no alcance da mesa; entra no URDF."""
import re

import numpy as np
import pytest

from rehab_pack.constants import ARM_JOINTS, HOME_Q_RAD

pytest.importorskip('xacro')
ament = pytest.importorskip('ament_index_python.packages')
try:
    CRA = ament.get_package_share_directory('cra_description')
    REHAB = ament.get_package_share_directory('rehab_pack')
except Exception:  # pragma: no cover
    pytest.skip('cra_description/rehab_pack não instalados', allow_module_level=True)

from rehab_pack.kinematics import Chain  # noqa: E402
from rehab_pack.urdf_builder import build_robot_urdf  # noqa: E402


@pytest.mark.parametrize('sim_ft', [False, True])
def test_urdf_writes_six_initial_values(sim_ft):
    full, _ = build_robot_urdf(CRA, REHAB, sim_ft=sim_ft)
    for j, q in zip(ARM_JOINTS, HOME_Q_RAD):
        m = re.search(rf'<joint name="{j}"\s*>.*?initial_value">([^<]*)<',
                      full, re.DOTALL)
        assert float(m.group(1)) == pytest.approx(q, abs=1e-4)


def test_home_fk_grip_points_up_in_reach():
    chain = Chain(build_robot_urdf(CRA, REHAB)[1])
    T = chain.fk(HOME_Q_RAD)
    assert T[2, 2] > 0.95               # eixo z do tcp (empunhadura) para cima
    assert 0.15 < T[2, 3] < 0.35        # altura do tcp sobre a mesa
    lo = np.array([a for a, _ in chain.limits])
    hi = np.array([b for _, b in chain.limits])
    assert np.all(np.array(HOME_Q_RAD) > lo) and np.all(np.array(HOME_Q_RAD) < hi)
    s = np.linalg.svd(chain.jacobian(HOME_Q_RAD), compute_uv=False)
    assert s[-1] / s[0] > 0.02          # longe de singularidade


def test_j5_nasce_em_mais_90_graus():
    """Pedido do pesquisador: os testes partem com J5 em +90° (não −90°)."""
    assert HOME_Q_RAD[4] == pytest.approx(np.pi / 2, abs=1e-4)
    full, _ = build_robot_urdf(CRA, REHAB)
    m = re.search(r'<joint name="joint5"\s*>.*?initial_value">([^<]*)<', full,
                  re.DOTALL)
    assert float(m.group(1)) == pytest.approx(np.pi / 2, abs=1e-4)
