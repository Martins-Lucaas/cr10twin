"""Malha leve de força do SLIDING (stiffness.slide_force_step): limites de
segurança e malha fechada simulada com os números medidos em 23/11/2026.

Função pura — não cria nó, não publica nada.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from touch_pack.stiffness import slide_force_step as step         # noqa: E402
from touch_pack.explorer_constants import (                       # noqa: E402
    _SLIDE_FC_DOWN_MAX_MS, _SLIDE_FC_UP_MAX_MS, _SLIDE_FC_DEPTH_MAX_M,
    _SLIDE_FC_LIFT_MAX_M, _SLIDE_FC_HZ_MAX)
from touch_pack.constants import CONTACT_ON_N                     # noqa: E402

DT = 0.03


def test_desligada_nao_mexe():
    assert step(1e-4, 5.0, 0.0, DT, True) == 1e-4
    assert step(1e-4, float('nan'), 1e-3, DT, True) == 1e-4


def test_descer_e_lento_e_curto():
    # Erro enorme pedindo descer: nunca mais que 0,1 mm/s nem 0,3 mm.
    d, hist = 0.0, []
    for _ in range(1000):
        d_new = step(d, 100.0, 1.0, DT, True)
        assert d_new - d <= _SLIDE_FC_DOWN_MAX_MS * DT + 1e-15
        d = d_new
        hist.append(d)
    assert max(hist) == pytest.approx(_SLIDE_FC_DEPTH_MAX_M)


def test_sem_contato_nao_desce():
    # O defeito do run 20261123_143253: sem contato o erro pede descer às
    # cegas. Agora segura d — mas ainda pode SUBIR.
    assert step(1e-4, 1.0, 1.0, DT, False) == 1e-4
    assert step(1e-4, -1.0, 1.0, DT, False) < 1e-4


def test_subir_e_limitado():
    d = 0.0
    for _ in range(1000):
        d_new = step(d, -100.0, 1.0, DT, True)
        assert d - d_new <= _SLIDE_FC_UP_MAX_MS * DT + 1e-15
        d = d_new
    assert d == pytest.approx(-_SLIDE_FC_LIFT_MAX_M)


def _cycle(K_nmm, fc_hz, v_mms, L_mm, theta_deg, n_strokes=4, drop_n=0.45,
           target=1.0, delay_s=0.04, filt_s=0.05):
    """Ciclo de ida e volta com d MANTIDO entre passadas (como no explorer).
    Planta: F = alvo + K·(d_executado − z_sup(x)) − degrau de início, com
    z_sup = tanθ·x (θ > 0: a superfície se AFASTA ao longo da ida). O
    comando chega com atraso; a força medida passa por filtro de 1ª ordem."""
    K = K_nmm * 1e3
    # Como no explorer: K do ganho = estimado (aqui, o próprio K real).
    ki = (2 * math.pi * fc_hz / K) if fc_hz else 0.0
    floor = CONTACT_ON_N
    buf = [0.0] * max(1, round(delay_s / DT))
    d, fm, a = 0.0, target, DT / (filt_s + DT)
    F, D, V = [], [], []
    n = int(L_mm / v_mms / DT)
    for s in range(n_strokes):
        for k in range(n):
            x = (k if s % 2 == 0 else n - k) * v_mms * DT * 1e-3
            z_sup = math.tan(math.radians(theta_deg)) * x
            f = max(0.0, target + K * (buf.pop(0) - z_sup)
                    - drop_n * min(k * DT / 0.3, 1.0))
            fm += a * (f - fm)
            d_prev, d = d, step(d, target - fm, ki, DT, abs(fm) >= floor)
            buf.append(d)
            F.append(f)
            D.append(d)
            V.append((d - d_prev) / DT)
    F = np.array(F)
    return float(np.mean(np.abs(F - target))), float(F.max()), max(V), max(D)


CASOS = [(K, v, L, th) for K in (5.0, 15.0, 30.0)
         for v, L in ((5.0, 10.0), (20.0, 40.0)) for th in (0.5, -0.5)]


@pytest.mark.parametrize('K,v,L,th', CASOS)
def test_malha_fechada_segura_em_ida_e_volta(K, v, L, th):
    e_ol, fmax_ol, _, _ = _cycle(K, 0.0, v, L, th)
    e_cl, fmax_cl, vdesc, dmax = _cycle(K, _SLIDE_FC_HZ_MAX, v, L, th)
    assert vdesc <= _SLIDE_FC_DOWN_MAX_MS + 1e-12
    assert dmax <= _SLIDE_FC_DEPTH_MAX_M + 1e-12   # teto vale pelo CICLO
    assert e_cl <= e_ol + 1e-9
    # Pico: nunca acima de 2× o alvo (critério de interrupção da bancada),
    # salvo onde a malha aberta já passava disso — aí nunca acima dela.
    assert fmax_cl < 2.0 or fmax_cl < fmax_ol


def test_cenario_do_run_143253_sem_pico_na_volta():
    # 1 N, 20 mm/s, 40 mm, superfície se afastando na ida (contato perdido):
    # com a lei antiga a volta bateu 2,7 N no braço. Agora fica < 2× alvo.
    _, fmax_cl, _, _ = _cycle(10.0, _SLIDE_FC_HZ_MAX, 20.0, 40.0, 0.5)
    assert fmax_cl < 2.0


@pytest.mark.parametrize('K', [5.0, 15.0, 30.0])
def test_deslize_lento_fica_perto_do_alvo(K):
    for th in (0.5, -0.5):
        e_ol, _, _, _ = _cycle(K, 0.0, 5.0, 10.0, th)
        e_cl, fmax, _, _ = _cycle(K, _SLIDE_FC_HZ_MAX, 5.0, 10.0, th)
        # Deslize lento: reduz o erro em ≥ 40 % (observado 46–82 % nos 6
        # casos; o pior é superfície subindo a 15 N/mm).
        assert e_cl < 0.6 * e_ol
        # Ganho simétrico (sem o 2× de subida): pior pico simulado 1,87 N a
        # 30 N/mm. Critério = o de interrupção da bancada, 2× o alvo.
        assert fmax < 2.0


@pytest.mark.parametrize('K', [5.0, 30.0])
@pytest.mark.parametrize('sub', [2.0, 4.0])
def test_estavel_com_K_subestimado(K, sub):
    # Pior caso de ganho: o K estimado é `sub`× MENOR que o real (malha mais
    # rápida que o projetado), banda no teto e atraso de 90 ms. Superfície
    # plana: cada passada tem que ASSENTAR no alvo, sem ciclo-limite.
    Kn = K * 1e3
    ki = 2 * math.pi * _SLIDE_FC_HZ_MAX / (Kn / sub)
    buf, d, fm, a = [0.0] * 3, 0.0, 1.0, DT / (0.05 + DT)
    n = int(4 / DT)
    for s in range(6):
        F = []
        for k in range(n):
            f = max(0.0, 1.0 + Kn * buf.pop(0) - 0.45 * min(k * DT / 0.3, 1.0))
            fm += a * (f - fm)
            d = step(d, 1.0 - fm, ki, DT, fm >= CONTACT_ON_N)
            buf.append(d)
            F.append(f)
        last = np.array(F[-int(1 / DT):])
        assert abs(last.mean() - 1.0) < 0.02 and last.std() < 0.01
