"""Posicionamento por juntas: o que o nó aceita ou recusa antes de mover o robô."""
import math

import numpy as np
import pytest

from rehab_pack.constants import (HOME_Q_RAD, JOG_MAX_DEG_S, JOG_MIN_S,
                                  JOINT_LIMITS_DEG, STATES)
from rehab_pack.jog import duration_s, max_delta_deg, plan_jog

HOME = np.array(HOME_Q_RAD)


def alto(_q):                 # TCP e célula sempre bem acima do piso
    return 0.30, 0.20


def test_posicionando_entra_no_fim_e_nao_move_os_outros_indices():
    assert STATES[:5] == ('PARADO', 'ATIVO', 'SEM_FORCA', 'LIMITE', 'ABORTADO')
    assert STATES[5] == 'POSICIONANDO'


def test_limites_de_software_sao_mais_estreitos_que_os_do_urdf():
    assert all(hi <= 180 for _, hi in JOINT_LIMITS_DEG)
    assert JOINT_LIMITS_DEG[2] == (-160.0, 160.0)         # J3 = limite de hardware


def test_movimento_valido():
    goal = HOME.copy(); goal[0] += math.radians(30)
    ok, motivo, dur = plan_jog(HOME, goal, alto)
    assert ok and dur == pytest.approx(30 / (JOG_MAX_DEG_S * 0.5)) and 'J1' in motivo


def test_duracao_minima_em_movimento_curto():
    goal = HOME.copy(); goal[3] += math.radians(2)
    assert duration_s(HOME, goal) == JOG_MIN_S
    goal[3] += math.radians(40)
    assert duration_s(HOME, goal, 100) == pytest.approx(duration_s(HOME, goal, 50) / 2)


def test_maior_movimento_e_a_junta_certa():
    goal = HOME.copy(); goal[4] += math.radians(10); goal[1] -= math.radians(25)
    d, j = max_delta_deg(HOME, goal)
    assert d == pytest.approx(25) and j == 2


@pytest.mark.parametrize('junta,graus', [(0, 181), (1, -171), (2, 161), (3, 171), (4, -171)])
def test_recusa_fora_do_limite(junta, graus):
    goal = HOME.copy(); goal[junta] = math.radians(graus)
    ok, motivo, _ = plan_jog(HOME, goal, alto)
    assert not ok and f'J{junta + 1}' in motivo and 'limite' in motivo


def test_recusa_sem_movimento():
    ok, motivo, _ = plan_jog(HOME, HOME + 1e-4, alto)
    assert not ok and '0,5°' in motivo


@pytest.mark.parametrize('ruim', [[0, 0, 0], [float('nan')] * 6, [float('inf')] * 6])
def test_recusa_alvo_malformado(ruim):
    assert not plan_jog(HOME, ruim, alto)[0]


def test_recusa_tcp_abaixo_do_piso_ao_longo_do_caminho():
    # o fim está acima do piso, mas o MEIO do caminho passa por baixo
    def mergulho(q):
        t = (q[0] - HOME[0]) / math.radians(60)
        return 0.30 - 0.5 * math.sin(math.pi * t), 0.20
    goal = HOME.copy(); goal[0] += math.radians(60)
    ok, motivo, _ = plan_jog(HOME, goal, mergulho)
    assert not ok and 'TCP' in motivo and 'etapas' in motivo


def test_recusa_celula_abaixo_do_piso():
    ok, motivo, _ = plan_jog(HOME, HOME + 0.2, lambda q: (0.30, 0.01))
    assert not ok and 'célula' in motivo


def test_com_a_cinematica_real_do_urdf():
    ament = pytest.importorskip('ament_index_python.packages')
    pytest.importorskip('xacro')
    try:
        cra = ament.get_package_share_directory('cra_description')
        reh = ament.get_package_share_directory('rehab_pack')
    except Exception:
        pytest.skip('pacotes não instalados')
    from rehab_pack.kinematics import Chain
    from rehab_pack.urdf_builder import build_robot_urdf
    urdf = build_robot_urdf(cra, reh)[1]
    tcp, cell = Chain(urdf, tip='tcp_link'), Chain(urdf, tip='load_cell_link')

    def fk_z(q):
        return float(tcp.fk(q)[2, 3]), float(cell.fk(q)[2, 3])

    # girar só a base (J1) mantém a altura: aceito
    g = HOME.copy(); g[0] += math.radians(40)
    assert plan_jog(HOME, g, fk_z)[0]
    # uma pose que mergulha o TCP no tampo da mesa tem de ser recusada
    q = HOME.copy(); q[1] += math.radians(80)
    if fk_z(q)[0] < 0.10:
        assert not plan_jog(HOME, q, fk_z)[0]
    else:   # se essa pose não mergulhar, ao menos o piso é respeitado em todo o caminho
        ok, _, _ = plan_jog(HOME, q, fk_z)
        assert ok == (min(fk_z((1 - s) * HOME + s * q)[0] for s in np.linspace(0, 1, 25)) >= 0.10)


def test_nenhum_log_muda_de_severidade_na_mesma_linha():
    """Regressão: `(logger.info if ok else logger.warn)(...)` levanta ValueError no
    rclpy e derrubava o rehab_admittance dentro do timer."""
    import pathlib
    import re
    raiz = pathlib.Path(__file__).resolve().parent.parent / 'rehab_pack'
    ruins = []
    for f in raiz.glob('*.py'):
        for n, ln in enumerate(f.read_text(encoding='utf-8').splitlines(), 1):
            if re.search(r'get_logger\(\)\.(info|warn|warning|error|debug)\s+if\b', ln):
                ruins.append(f'{f.name}:{n}')
    assert not ruins, ruins


def test_piso_com_a_cinematica_real_pose_conhecida_e_recusada():
    """Pose achada por busca (4000 sorteios): a célula desce a 4 cm do tampo no
    caminho. Antes só havia um teste condicional, que podia nem exercitar o piso."""
    ament = pytest.importorskip('ament_index_python.packages')
    pytest.importorskip('xacro')
    try:
        cra = ament.get_package_share_directory('cra_description')
        reh = ament.get_package_share_directory('rehab_pack')
    except Exception:
        pytest.skip('pacotes não instalados')
    from rehab_pack.kinematics import Chain
    from rehab_pack.urdf_builder import build_robot_urdf
    urdf = build_robot_urdf(cra, reh)[1]
    tcp, cell = Chain(urdf, tip='tcp_link'), Chain(urdf, tip='load_cell_link')

    def fk_z(q):
        return float(tcp.fk(q)[2, 3]), float(cell.fk(q)[2, 3])

    ruim = [0.323, -1.866, -2.284, -2.546, 0.813, 0.808]
    ok, motivo, _ = plan_jog(HOME, ruim, fk_z)
    assert not ok and 'célula' in motivo and 'etapas' in motivo
    # a célula no HOME está baixa (13,6 cm): J2 -20° já a leva ao limite de 5 cm
    g = HOME.copy(); g[1] -= math.radians(20)
    assert not plan_jog(HOME, g, fk_z)[0]
    # movimentos laterais e de punho a partir do HOME são aceitos
    for j in (0, 3, 4, 5):
        for d in (-20, 20):
            t = HOME.copy(); t[j] += math.radians(d)
            assert plan_jog(HOME, t, fk_z)[0], (j + 1, d)
