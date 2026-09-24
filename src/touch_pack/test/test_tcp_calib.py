"""Calibração de TCP por pivô: o solver tem de devolver a ferramenta que
gerou as capturas.

As capturas sintéticas vêm da IK com uma ferramenta CONHECIDA, encostando a
ponta no mesmo ponto do mundo com vetores de ataque diferentes — que é
exatamente o procedimento de bancada. Se o solver não recuperar esse número,
toda a IK do pacote passa a andar com um TCP errado.
"""
import numpy as np
import pytest

from touch_pack import tcp_calib
from touch_pack.kinematics import forward_kinematics, inverse_kinematics
from touch_pack.manip3d import ik_step


def _tool(x_mm: float, y_mm: float, z_mm: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = [x_mm * 1e-3, y_mm * 1e-3, z_mm * 1e-3]
    return T


def _capturas(tool: np.ndarray, p_pivot: np.ndarray) -> list[np.ndarray]:
    """Ângulos que encostam a ponta de `tool` em `p_pivot` por vários ataques."""
    ataques = [(0.0, 0.0, -1.0), (0.25, 0.0, -1.0),
               (-0.2, 0.15, -1.0), (0.1, -0.3, -1.0)]
    qs = []
    for vec in ataques:
        v = np.array(vec, dtype=float)
        q, ok = inverse_kinematics(p_pivot, v / np.linalg.norm(v), T_end=tool)
        if not ok:
            continue
        # A IK global fecha a ~1,5 mm — bom para o robô, inútil como gabarito:
        # esse erro reapareceria como resíduo e o teste estaria medindo a IK,
        # não o solver. O passo diferencial leva a ponta ao ponto de verdade.
        for _ in range(60):
            q = ik_step(q, p_pivot, T_end=tool, tol_pos_m=1e-9).q
        p_real = forward_kinematics(q, T_end=tool)[:3, 3]
        if np.linalg.norm(p_real - p_pivot) < 1e-6:
            qs.append(q)
    return qs


def test_pivo_recupera_a_ferramenta():
    tool = _tool(2.0, -3.0, 80.0)
    p_pivot = np.array([0.55, 0.10, 0.20])

    qs = _capturas(tool, p_pivot)
    assert len(qs) >= tcp_calib.PIVOT_MIN_POINTS, 'IK não fechou capturas suficientes'

    fit = tcp_calib.solve_pivot(qs)

    assert fit.offset_mm == pytest.approx((2.0, -3.0, 80.0), abs=0.05)
    assert fit.pivot_mm == pytest.approx(tuple(p_pivot * 1e3), abs=0.05)
    assert fit.residual_mm < 0.05
    assert fit.length_mm == pytest.approx(80.06, abs=0.1)


def test_captura_escorregada_e_recusada():
    """Uma captura fora do ponto tem de aparecer como resíduo, não ser diluída."""
    tool = _tool(0.0, 0.0, 80.0)
    qs = _capturas(tool, np.array([0.55, 0.10, 0.20]))
    assert len(qs) >= tcp_calib.PIVOT_MIN_POINTS
    # Ponta escorregou ~8 mm num dos ataques: refaz a captura num pivô vizinho.
    qs[-1] = _capturas(tool, np.array([0.558, 0.10, 0.20]))[-1]

    fit = tcp_calib.solve_pivot(qs)

    assert tcp_calib.check_fit(fit, cad_length_mm=80.0) is not None


def test_mesma_orientacao_nao_fecha():
    """Só transladar não calibra: o sistema fica singular e tem de reclamar."""
    q = np.array([0.2, -1.2, 0.9, 0.0, 0.5, 0.0])
    with pytest.raises(ValueError):
        tcp_calib.solve_pivot([q, q, q, q])


def test_sem_arquivo_a_cinematica_fica_no_cad(monkeypatch, tmp_path):
    monkeypatch.setattr(tcp_calib, 'TCP_CALIB_FILE', str(tmp_path / 'nao_existe.json'))
    assert tcp_calib.load_offset_mm() is None


def test_ida_e_volta_do_arquivo(monkeypatch, tmp_path):
    monkeypatch.setattr(tcp_calib, 'TCP_CALIB_FILE', str(tmp_path / 'tcp_calib.json'))
    monkeypatch.setattr(tcp_calib, 'CONFIG_DIR', str(tmp_path))
    fit = tcp_calib.PivotFit(offset_mm=(1.0, 2.0, 67.5), pivot_mm=(0.0, 0.0, 0.0),
                             residual_mm=0.1, n_points=4)

    tcp_calib.save_offset_mm(fit)

    assert tcp_calib.load_offset_mm() == pytest.approx((1.0, 2.0, 67.5))
    assert tcp_calib.clear() is True
    assert tcp_calib.load_offset_mm() is None
