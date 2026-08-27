"""Geometria da vista 3D do vetor de força.

Só as funções puras — o resto do mixin é canvas do Tk e precisa de display.

O caso que mais importa aqui é `rot_z_to` com d = -Z: o eixo de Rodrigues é o
produto vetorial (0,0,1) x d, que ZERA nessa direção. Sem o tratamento
explícito a matriz sairia com divisão por zero e a seta sumiria exatamente na
compressão pura em Z — que na bancada é o caso comum, não o canto raro.
"""
import math
import os

import pytest

from touch_pack.gui_ft_arrow import (
    FACTORY_OBJ_DIR, FACTORY_OBJ_NAME, _mul, load_obj, normalize_along_z,
    procedural_arrow, project, rot_z_to,
)


def _quase(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def _unit(v):
    n = math.dist(v, (0, 0, 0))
    return tuple(c / n for c in v)


# ── Malha procedural ──────────────────────────────────────────────────

def test_seta_procedural_tem_indices_validos():
    verts, faces = procedural_arrow()
    assert verts and faces
    for f in faces:
        assert len(f) == 3
        for i in f:
            assert 0 <= i < len(verts)


def test_seta_procedural_aponta_para_mais_z():
    verts, _ = procedural_arrow()
    assert max(v[2] for v in verts) == pytest.approx(1.0)
    assert min(v[2] for v in verts) == pytest.approx(0.0)


# ── Normalização ──────────────────────────────────────────────────────

def test_normalizacao_poe_comprimento_1_em_z_com_base_na_origem():
    # Malha deitada em X, escala arbitrária, deslocada da origem.
    verts = [(10.0, 1.0, 2.0), (60.0, -1.0, 2.0), (35.0, 0.0, 4.0)]
    out = normalize_along_z(verts)
    zs = [v[2] for v in out]
    assert min(zs) == pytest.approx(0.0)
    assert max(zs) == pytest.approx(1.0)


def test_normalizacao_centra_os_eixos_curtos():
    verts = [(0.0, 100.0, 0.0), (0.0, 200.0, 10.0), (0.0, 150.0, -10.0)]
    out = normalize_along_z(verts)
    # O eixo longo é Y (extensão 100) e vira Z; X e o antigo Z ficam
    # centrados em zero.
    assert sum(v[0] for v in out) == pytest.approx(0.0, abs=1e-9)


# ── Rotação ───────────────────────────────────────────────────────────

@pytest.mark.parametrize('d', [
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, -1.0),
    _unit((1.0, 1.0, 1.0)),
    (0.6, -0.8, 0.0),
])
def test_rot_leva_mais_z_ate_a_direcao_pedida(d):
    assert _quase(_mul(rot_z_to(d), (0.0, 0.0, 1.0)), d, tol=1e-6)


def test_rot_em_menos_z_nao_divide_por_zero():
    """Compressão pura em Z é o caso comum da bancada, não um canto raro."""
    m = rot_z_to((0.0, 0.0, -1.0))
    assert _quase(_mul(m, (0.0, 0.0, 1.0)), (0.0, 0.0, -1.0), tol=1e-9)


@pytest.mark.parametrize('d', [
    (0.0, 0.0, 1.0), (0.0, 0.0, -1.0), (0.6, -0.8, 0.0),
    _unit((1.0, 2.0, 3.0)),
])
def test_rot_preserva_norma(d):
    """Se a matriz não fosse ortonormal, a seta esticaria conforme girasse."""
    m = rot_z_to(d)
    for v in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.3, -0.5, 0.81)):
        assert math.dist(_mul(m, v), (0, 0, 0)) == pytest.approx(
            math.dist(v, (0, 0, 0)), abs=1e-9)


# ── Projeção ──────────────────────────────────────────────────────────

def test_origem_projeta_no_centro():
    x, y, _ = project((0.0, 0.0, 0.0), 100.0, 150.0, 130.0)
    assert (x, y) == pytest.approx((150.0, 130.0))


def test_nenhum_eixo_do_sensor_degenera_na_projecao():
    """Com yaw/pitch mal escolhidos um eixo vira um ponto e some da tela."""
    vistos = [project(e, 100.0, 0.0, 0.0)[:2]
              for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    for x, y in vistos:
        assert math.hypot(x, y) > 20.0


# ── OBJ do fabricante, quando ela existe ──────────────────────────────

_OBJ = os.path.join(FACTORY_OBJ_DIR, FACTORY_OBJ_NAME)


@pytest.mark.skipif(not os.path.exists(_OBJ),
                    reason='cliente de fábrica não instalado nesta máquina')
def test_obj_do_fabricante_carrega_e_tria_ngula():
    verts, faces = load_obj(_OBJ)
    assert len(verts) == 90            # conferido com grep -c "^v "
    assert len(faces) >= 176           # >= porque quad vira 2 triângulos
    for f in faces:
        assert len(f) == 3
        for i in f:
            assert 0 <= i < len(verts)


@pytest.mark.skipif(not os.path.exists(_OBJ),
                    reason='cliente de fábrica não instalado nesta máquina')
def test_obj_do_fabricante_normaliza_para_a_mesma_forma_da_procedural():
    out = normalize_along_z(load_obj(_OBJ)[0])
    zs = [v[2] for v in out]
    assert min(zs) == pytest.approx(0.0)
    assert max(zs) == pytest.approx(1.0)


def test_obj_ausente_levanta_em_vez_de_devolver_malha_vazia():
    with pytest.raises((OSError, ValueError)):
        load_obj(os.path.join(FACTORY_OBJ_DIR, 'nao_existe.obj'))
