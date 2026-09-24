"""tcp_calib.py — calibração do TCP de palpação por pivô de 4 pontos.

O `T_TOUCH_TOOL_ATTACH` do `kinematics.py` sai do CAD. A peça impressa não
sai: folga de rosca, ponteira que entortou, célula remontada meio milímetro
fora. Qualquer desvio vira erro de posição na IK e, no SLIDE, componente de
força espúria — a ponta desliza num eixo que não é o que o log diz.

O DobotStudio Pro resolve isso com o assistente de 6 pontos (TCP+ZX), que mede
posição E orientação. Aqui só a POSIÇÃO é medida, pelo método do pivô: encoste
a MESMA ponta física no MESMO ponto do mundo em 4 orientações diferentes e
capture os ângulos de junta. A orientação não é medida porque a ponteira é
simétrica em torno do z do flange — o eixo da ferramenta já é o eixo do flange,
e medir o que não muda só adiciona pontos para errar.

Matemática: para cada captura i, o mesmo ponto físico P do mundo vale

    R_i · t + o_i = P

onde (R_i, o_i) é a pose do FLANGE (FK com T_end = identidade) e `t` é o offset
flange→ponta procurado. Empilhando as capturas:

    [R_i  −I] · [t; P] = −o_i

3·N equações para 6 incógnitas: N = 3 já fecha, N = 4 dá resíduo — e o resíduo
é a única coisa que denuncia uma captura em que a ponta escorregou do ponto.

Uso pela GUI: `solve_pivot(lista_de_q_urdf_rad)` → `PivotFit`, e `save_offset_mm`
grava em `~/.config/touch_pack/tcp_calib.json`. Daí em diante o
`kinematics.T_TOUCH_TOOL_ATTACH` nasce corrigido (ver `_apply_tcp_calib` lá) e
o `real_driver.apply_tool_offset` escreve o mesmo número no Tool(1) do
controlador, para MovL/RelMovL andarem no mesmo TCP que a IK.
"""
from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .constants import CONFIG_DIR

log = logging.getLogger('touch_pack.tcp_calib')

TCP_CALIB_FILE = os.path.join(CONFIG_DIR, 'tcp_calib.json')

# Mínimo de capturas aceito. Com 3 o sistema fecha exatamente e o resíduo é
# zero por construção — ou seja, não informa nada. A quarta captura é o que
# transforma "tem solução" em "a solução confere".
PIVOT_MIN_POINTS = 4

# Resíduo RMS acima disto é captura ruim (ponta escorregou, ponto de apoio
# cedeu), não ferramenta torta: a repetibilidade do próprio CR10 é ±0,03 mm.
PIVOT_MAX_RESIDUAL_MM = 2.0

# Quanto o TCP medido pode se afastar do valor de CAD antes de a calibração
# ser recusada. 30 mm cobre folga de rosca e ponteira trocada; além disso é
# mais provável que as capturas sejam de pontos físicos DIFERENTES.
PIVOT_MAX_DEVIATION_MM = 30.0


@dataclass
class PivotFit:
    """Resultado do ajuste: offset flange→ponta e a qualidade dele."""
    offset_mm: tuple[float, float, float]
    pivot_mm: tuple[float, float, float]    # onde o ponto de apoio ficou (base)
    residual_mm: float                      # RMS entre as capturas
    n_points: int

    @property
    def length_mm(self) -> float:
        """Comprimento do TCP — o número que o carimbo `tool_tcp_mm` publica."""
        return float(np.linalg.norm(self.offset_mm))


def solve_pivot(q_list: Sequence[Sequence[float]]) -> PivotFit:
    """Resolve o offset flange→ponta a partir de N capturas articulares.

    Args:
        q_list: N vetores de 6 ângulos das juntas em rad, convenção URDF
                (a mesma dos sliders do Controle Manual), todos com a ponta
                encostada no MESMO ponto físico.

    Raises:
        ValueError: menos de PIVOT_MIN_POINTS capturas, ou capturas tão
                    parecidas que o sistema fica singular (foi preciso GIRAR
                    o pulso entre elas, não só transladar).
    """
    from .kinematics import forward_kinematics   # tardio: kinematics importa este módulo

    qs = [np.asarray(q, dtype=float).reshape(6) for q in q_list]
    if len(qs) < PIVOT_MIN_POINTS:
        raise ValueError(
            f'calibração de pivô precisa de ao menos {PIVOT_MIN_POINTS} '
            f'capturas em orientações diferentes (tem {len(qs)})')

    flanges = [forward_kinematics(q, T_end=np.eye(4)) for q in qs]   # pose do FLANGE
    A = np.zeros((3 * len(qs), 6))
    b = np.zeros(3 * len(qs))
    for i, T in enumerate(flanges):
        A[3 * i:3 * i + 3, :3] = T[:3, :3]
        A[3 * i:3 * i + 3, 3:] = -np.eye(3)
        b[3 * i:3 * i + 3] = -T[:3, 3]

    # rcond=None usa o corte padrão do numpy; o que importa é detectar o
    # posto deficiente, que é o caso "capturei 4 vezes a mesma orientação".
    sol, _res, rank, _sv = np.linalg.lstsq(A, b, rcond=None)
    if rank < 6:
        raise ValueError(
            'capturas quase idênticas — o pivô só fecha se o PULSO girar '
            'entre elas (mesmo ponto, ataques diferentes)')

    t, p = sol[:3], sol[3:]
    erros = [float(np.linalg.norm(T[:3, :3] @ t + T[:3, 3] - p)) for T in flanges]
    residual_mm = float(np.sqrt(np.mean(np.square(erros))) * 1e3)

    return PivotFit(
        offset_mm=tuple(float(v * 1e3) for v in t),      # type: ignore[arg-type]
        pivot_mm=tuple(float(v * 1e3) for v in p),       # type: ignore[arg-type]
        residual_mm=residual_mm,
        n_points=len(qs),
    )


def check_fit(fit: PivotFit, cad_length_mm: float) -> str | None:
    """None se o ajuste é aceitável; senão a frase pronta para a tela.

    Recusar é o comportamento certo: uma calibração ruim aceita em silêncio
    desloca TODA a IK, o log e as poses ensinadas de uma vez só.
    """
    if fit.residual_mm > PIVOT_MAX_RESIDUAL_MM:
        return (f'resíduo de {fit.residual_mm:.2f} mm entre as capturas '
                f'(limite {PIVOT_MAX_RESIDUAL_MM:.1f} mm) — a ponta escorregou '
                f'do ponto de apoio em alguma delas. Refaça.')
    desvio = fit.length_mm - cad_length_mm
    if abs(desvio) > PIVOT_MAX_DEVIATION_MM:
        return (f'TCP medido de {fit.length_mm:.1f} mm contra {cad_length_mm:.1f} mm '
                f'de CAD ({desvio:+.1f} mm) — fora dos {PIVOT_MAX_DEVIATION_MM:.0f} mm '
                f'admitidos. Confira se todas as capturas foram no MESMO ponto.')
    return None


def save_offset_mm(fit: PivotFit) -> str:
    """Grava a calibração e devolve o caminho do arquivo."""
    os.makedirs(CONFIG_DIR, exist_ok=True)
    with open(TCP_CALIB_FILE, 'w', encoding='utf-8') as f:
        json.dump({
            'offset_mm': list(fit.offset_mm),
            'pivot_mm': list(fit.pivot_mm),
            'residual_mm': fit.residual_mm,
            'n_points': fit.n_points,
        }, f, indent=2)
    return TCP_CALIB_FILE


def load_offset_mm() -> tuple[float, float, float] | None:
    """Offset flange→ponta salvo, em mm, ou None se não há calibração.

    Nunca levanta: este arquivo é lido no import do `kinematics`, e um JSON
    corrompido não pode derrubar o pacote inteiro — cai no valor de CAD, que
    é o comportamento de antes da calibração existir.
    """
    try:
        with open(TCP_CALIB_FILE, encoding='utf-8') as f:
            d = json.load(f)
        x, y, z = (float(v) for v in d['offset_mm'])
    except FileNotFoundError:
        return None
    except Exception as exc:
        log.warning('tcp_calib.json ilegível (%s) — usando o TCP de CAD', exc)
        return None
    return x, y, z


def clear() -> bool:
    """Apaga a calibração (volta ao CAD). True se havia arquivo."""
    try:
        os.remove(TCP_CALIB_FILE)
        return True
    except FileNotFoundError:
        return False
