"""Testes do backend de GPU da viewport 3D (touch_pack.vtk_render)."""
import math

import numpy as np
import pytest

from touch_pack import urdf_scene as us
from touch_pack.manip3d import Camera
from touch_pack.vtk_render import (
    VtkRendererError, VtkRobotRenderer, vtk_available,
)

W, H = 320, 240
BG = (1.0, 1.0, 1.0)

pytestmark = pytest.mark.skipif(not vtk_available(), reason='VTK ausente')


def _marker_scene(center, half=0.02, color=(0, 0, 0)):
    """Cena mínima: um cubo pequeno num ponto conhecido do mundo."""
    tris = us.tessellate_box((2 * half, 2 * half, 2 * half)) + np.asarray(center)
    part = us.VisualPart(link='base_link', tris=tris, color=color)
    return us.RobotScene(joints=[], parts=[part],
                         link_names=['base_link'], root='base_link')


def _renderer(scene):
    try:
        return VtkRobotRenderer(scene, W, H, background=BG)
    except VtkRendererError as exc:
        pytest.skip(f'sem contexto GL: {exc}')


def _ink_centroid(frame):
    """Centroide dos pixels não-fundo, em coordenadas de tela (u, v)."""
    mask = frame.sum(axis=2) < 3 * 250
    assert mask.any(), 'nada foi desenhado'
    vs, uvs = np.nonzero(mask)
    return np.array([uvs.mean(), vs.mean()]), int(mask.sum())


@pytest.mark.parametrize('center', [
    (0.0, 0.0, 0.5),
    (0.35, 0.10, 0.70),
    (-0.20, -0.40, 0.25),
])
def test_gpu_projection_matches_camera_project(center):
    """O marcador renderizado cai no MESMO pixel que `Camera.project` prevê."""
    center = np.asarray(center, dtype=float)
    scene = _marker_scene(center)
    ren = _renderer(scene)
    try:
        cam = Camera()
        frame = ren.render({'base_link': np.eye(4)}, cam)
        assert frame.shape == (H, W, 3)
        drawn, _n = _ink_centroid(frame)
        expected, z = cam.project(center[None, :], W, H)
        assert z[0] > cam.near
        assert np.allclose(drawn, expected[0], atol=1.5), (
            f'render em {drawn} vs projeção em {expected[0]}')
    finally:
        ren.close()


@pytest.mark.parametrize('az_deg,el_deg,dist', [
    (-125.0, 20.0, 2.3),
    (30.0, 55.0, 1.4),
    (170.0, -10.0, 3.5),
])
def test_alignment_holds_across_camera_poses(az_deg, el_deg, dist):
    """Orbitar e dar zoom não pode descolar a alça do robô desenhado."""
    center = np.array([0.30, -0.15, 0.55])
    ren = _renderer(_marker_scene(center))
    try:
        cam = Camera(az=math.radians(az_deg), el=math.radians(el_deg),
                     dist=dist)
        frame = ren.render({'base_link': np.eye(4)}, cam)
        drawn, _n = _ink_centroid(frame)
        expected, _z = cam.project(center[None, :], W, H)
        assert np.allclose(drawn, expected[0], atol=1.5)
    finally:
        ren.close()


def test_alignment_holds_after_resize():
    """O canvas muda de tamanho quando a janela é redimensionada; a projeção
    depende de H (fpx = 0,5·H/tan(α/2)) e as duas têm de acompanhar juntas."""
    center = np.array([0.25, 0.0, 0.60])
    ren = _renderer(_marker_scene(center))
    try:
        w2, h2 = 500, 380
        ren.resize(w2, h2)
        cam = Camera()
        frame = ren.render({'base_link': np.eye(4)}, cam)
        assert frame.shape == (h2, w2, 3)
        drawn, _n = _ink_centroid(frame)
        expected, _z = cam.project(center[None, :], w2, h2)
        assert np.allclose(drawn, expected[0], atol=1.5)
    finally:
        ren.close()


def test_link_transform_moves_the_actor():
    """A FK entra como UserMatrix por ator — é o que anima a cena."""
    ren = _renderer(_marker_scene(np.zeros(3)))
    try:
        cam = Camera()
        T = np.eye(4)
        T[:3, 3] = [0.0, 0.0, 0.6]
        frame = ren.render({'base_link': T}, cam)
        drawn, _n = _ink_centroid(frame)
        expected, _z = cam.project(np.array([[0.0, 0.0, 0.6]]), W, H)
        assert np.allclose(drawn, expected[0], atol=1.5)
    finally:
        ren.close()


def test_each_part_gets_its_own_matrix():
    """Regressão: `SetUserMatrix` guarda a REFERÊNCIA da matriz. Com uma
    instância compartilhada entre atores, todos os elos herdam a última pose
    escrita e o robô inteiro colapsa num ponto."""
    a = us.VisualPart(link='a', tris=us.tessellate_box((0.04, 0.04, 0.04)),
                      color=(0, 0, 0))
    b = us.VisualPart(link='b', tris=us.tessellate_box((0.04, 0.04, 0.04)),
                      color=(0, 0, 0))
    scene = us.RobotScene(joints=[], parts=[a, b],
                          link_names=['a', 'b'], root='a')
    ren = _renderer(scene)
    try:
        Ta, Tb = np.eye(4), np.eye(4)
        Ta[:3, 3] = [-0.35, 0.0, 0.5]
        Tb[:3, 3] = [0.35, 0.0, 0.5]
        cam = Camera()
        frame = ren.render({'a': Ta, 'b': Tb}, cam)
        mask = frame.sum(axis=2) < 3 * 250
        cols = np.nonzero(mask.any(axis=0))[0]
        # Duas manchas separadas → dois atores em lugares distintos.
        gaps = np.diff(cols)
        assert gaps.max() > 5, 'os dois links renderizaram colados'
    finally:
        ren.close()


def test_renders_the_real_cell_at_full_resolution():
    """A cena exata (sem decimação) é justamente o motivo deste backend."""
    try:
        scene = us.build_scene('touch_tool', triangle_budget=None)
    except Exception as exc:
        pytest.skip(f'URDF da célula indisponível: {exc}')
    ren = _renderer(scene)
    try:
        assert ren.triangle_count > 40000          # malha cheia, não reduzida
        q = np.deg2rad([0.0, -30.0, -90.0, 0.0, 60.0, 0.0])
        Ts = scene.link_transforms(
            {f'joint{i + 1}': float(q[i]) for i in range(6)})
        frame = ren.render(Ts, Camera())
        _c, n_ink = _ink_centroid(frame)
        assert n_ink > 2000                        # o braço ocupa a tela
    finally:
        ren.close()
