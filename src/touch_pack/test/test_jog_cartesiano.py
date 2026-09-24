"""Jog cartesiano: o passo tem de sair no eixo pedido e no tamanho pedido.

Um erro de sinal ou de frame aqui não trava nada — só manda a ponteira para o
lado errado com o operador olhando para a peça, que é o pior jeito de
descobrir. Por isso o teste mede o DESLOCAMENTO DO TCP resultante, não os
ângulos de junta: é o deslocamento que o operador pediu.

Pula sozinho onde não há display (CI headless), como os demais testes de GUI.

O nome começa com `j` de propósito. Pytest coleta em ordem alfabética, e um
`tk.Tk()` criado e destruído ANTES de `test_explorer_logic.py` aborta o
interpretador lá no meio (finalização do Tcl contra o logger do rclpy chamado
de uma thread, durante o GC) — a sessão inteira morre com "dumped core" sem
apontar o culpado. Todos os testes de GUI que já existiam (`ft_gui_smoke`,
`lc_axial_gui`, `matrix_gui`) vêm depois de `explorer` por acidente do nome;
este vem por escolha.
"""
import math
import pathlib
import sys
import tkinter as tk

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from touch_pack.constants import ARM_JOINTS                 # noqa: E402
from touch_pack import gui_manual_extras                     # noqa: E402
from touch_pack.gui_manual_extras import ManualExtrasMixin  # noqa: E402
from touch_pack.kinematics import (                          # noqa: E402
    forward_kinematics, T_TOUCH_TOOL_ATTACH,
)

# Pose de trabalho da palpação: braço estendido, punho para baixo.
Q_INICIAL_DEG = [-30.0, -125.0, 135.0, -100.0, 90.0, -60.0]


@pytest.fixture(scope='module')
def _tk_root():
    try:
        r = tk.Tk()
    except tk.TclError as exc:                       # pragma: no cover
        pytest.skip(f'sem display: {exc}')
    r.withdraw()
    yield r
    r.destroy()


@pytest.fixture
def raiz(_tk_root):
    f = tk.Frame(_tk_root)
    f.pack()
    yield f
    f.destroy()


class _Host(ManualExtrasMixin):
    """O mínimo que o mixin espera do PalpationGUI."""

    def __init__(self, raiz):
        self.root = raiz
        self._end_effector = 'touch_tool'
        self._robot_connected = False
        self._real_driver = None
        self._latest_joint_rad = None
        self.gate = ''
        self.status: list[tuple[str, str]] = []
        self.publicou = 0
        self.arm_sliders = {j: tk.DoubleVar(value=v)
                            for j, v in zip(ARM_JOINTS, Q_INICIAL_DEG)}
        self._build_manual_extras(raiz)

    def _card(self, parent, _title, *, expand=True):
        f = tk.Frame(parent)
        f.pack(fill='x')
        return f

    def _set_status(self, txt, color=''):
        self.status.append((txt, color))

    def _suppress(self):
        class _Nada:
            def __enter__(self_inner):
                return None

            def __exit__(self_inner, *_a):
                return False
        return _Nada()

    def _publish_arm_from_sliders(self):
        self.publicou += 1

    def _manip_gate_reason(self):
        return self.gate

    def _manip_T_end(self):
        return T_TOUCH_TOOL_ATTACH

    # auxiliares do teste
    def q_rad(self):
        return np.array([math.radians(self.arm_sliders[j].get())
                         for j in ARM_JOINTS])

    def pose_tcp(self):
        return forward_kinematics(self.q_rad(), T_end=T_TOUCH_TOOL_ATTACH)


@pytest.fixture
def host(raiz):
    return _Host(raiz)


class _DriverFalso:
    def __init__(self):
        self.freios = []

    def brake(self, axis, released):
        self.freios.append((axis, released))


def test_freio_vai_para_o_driver_com_o_braco_conectado(host):
    host._robot_connected = True
    host._real_driver = _DriverFalso()

    host._brake_set(2, True)

    assert host._real_driver.freios == [(2, True)]
    assert 'RELEASED' in host.status[-1][0]


def test_freio_sem_braco_conectado_so_avisa(host):
    host._brake_set(2, True)

    assert 'Connect the CR10' in host.status[-1][0]


def test_passo_na_base_anda_no_eixo_do_mundo(host):
    antes = host.pose_tcp()[:3, 3].copy()
    host.jog_frame_var.set('base')
    host.jog_step_mm.set(10.0)

    host._cart_jog(2, +1.0)          # Z+

    d = host.pose_tcp()[:3, 3] - antes
    assert host.publicou == 1
    assert d[2] == pytest.approx(0.010, abs=2e-4)
    assert abs(d[0]) < 2e-4 and abs(d[1]) < 2e-4


def test_passo_na_ferramenta_anda_no_eixo_da_ponteira(host):
    T0 = host.pose_tcp()
    eixo_ferramenta = T0[:3, :3] @ np.array([0.0, 0.0, 1.0])
    host.jog_frame_var.set('tool')
    host.jog_step_mm.set(1.0)

    host._cart_jog(2, -1.0)          # Z− da FERRAMENTA (recua pela ponta)

    d = host.pose_tcp()[:3, 3] - T0[:3, 3]
    assert np.linalg.norm(d) == pytest.approx(0.001, abs=5e-5)
    # Colinear com o eixo da ponteira, sentido negativo.
    assert float(d @ eixo_ferramenta) == pytest.approx(-0.001, abs=5e-5)


def test_passo_de_um_decimo_de_milimetro_chega_inteiro(host):
    """A resolução que a palpação usa perto do contato não pode ser engolida
    pelo arredondamento dos sliders."""
    antes = host.pose_tcp()[:3, 3].copy()
    host.jog_frame_var.set('base')
    host.jog_step_mm.set(0.1)

    host._cart_jog(2, -1.0)

    d = host.pose_tcp()[:3, 3] - antes
    assert d[2] == pytest.approx(-1e-4, abs=2e-5)


def test_atitude_do_tcp_e_preservada(host):
    R0 = host.pose_tcp()[:3, :3].copy()
    host.jog_frame_var.set('base')
    host.jog_step_mm.set(10.0)

    host._cart_jog(0, +1.0)          # X+

    R1 = host.pose_tcp()[:3, :3]
    # Ângulo residual entre as duas atitudes, pelo traço da rotação relativa.
    cos = (np.trace(R0.T @ R1) - 1.0) / 2.0
    assert math.degrees(math.acos(min(1.0, max(-1.0, cos)))) < 0.2


def test_gate_fechado_nao_move_nem_publica(host):
    antes = host.pose_tcp()[:3, 3].copy()
    host.gate = 'palpation running (CONTACT) — drag disabled'

    host._cart_jog(2, +1.0)

    assert host.publicou == 0
    assert np.allclose(host.pose_tcp()[:3, 3], antes)
    assert 'blocked' in host.status[-1][0]


def test_passo_que_estoura_limite_de_junta_e_recusado(host, monkeypatch):
    """Recusar inteiro, não publicar uma pose clipada: o operador pediu um
    deslocamento e precisa saber que ele NÃO aconteceu.

    Os limites são apertados em volta da pose atual em vez de procurar uma
    geometria que estoure sozinha — o que está sob teste é a recusa, e ela não
    pode depender de qual pose o braço estava quando alguém rodou o teste.
    """
    graus_iniciais = {j: host.arm_sliders[j].get() for j in ARM_JOINTS}
    monkeypatch.setattr(
        gui_manual_extras, 'ARM_LIMITS_DEG',
        {j: (v - 0.01, v + 0.01) for j, v in graus_iniciais.items()})
    antes = host.pose_tcp()[:3, 3].copy()
    host.jog_frame_var.set('base')
    host.jog_step_mm.set(10.0)

    host._cart_jog(2, +1.0)

    assert 'joint limit' in host.status[-1][0]
    assert host.publicou == 0
    assert {j: host.arm_sliders[j].get() for j in ARM_JOINTS} == graus_iniciais
    assert np.allclose(host.pose_tcp()[:3, 3], antes, atol=1e-12)
