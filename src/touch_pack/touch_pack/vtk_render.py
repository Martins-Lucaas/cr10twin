"""
vtk_render.py — Backend de render por GPU da viewport 3D.

É o caminho que mostra a malha **exata** do Gazebo: os STL do URDF inteiros,
sem redução nenhuma. O rasterizador em PIL de `manip3d` continua existindo
como plano B, mas ele precisa decimar (a mão COVVI tem 420 mil triângulos e
custaria ~1,1 s por quadro em software); aqui os mesmos 420 mil saem em
~2 ms de GPU, e o custo do quadro vira a leitura do framebuffer.

O render é OFFSCREEN: o VTK desenha num framebuffer próprio, os pixels são
lidos para numpy e a imagem é colada no mesmo `tk.Canvas` de sempre. Assim a
alça do TCP, o alvo do arrasto e o HUD continuam sendo itens vetoriais do Tk
por cima — nada de embutir uma janela GL dentro do Tk (que traz problemas de
foco, redimensionamento e empilhamento).

A projeção do vtkCamera é montada para casar EXATAMENTE com
`manip3d.Camera.project`, que é quem decide onde a alça do TCP é desenhada e
onde o clique a encontra. Com ângulo de visão VERTICAL α e aspecto W/H:

    v_px = H/2 − (H/2)·(y/z)/tan(α/2)          = H/2 − fpx·y/z
    u_px = W/2 + (W/2)·(x/z)/(tan(α/2)·(W/H))  = W/2 + fpx·x/z

que é ponto a ponto a fórmula de `Camera.project` com fpx = 0,5·H/tan(α/2).
`test_vtk_render.py` confere isso contra pixels renderizados de verdade.
"""
from __future__ import annotations

import numpy as np

try:
    import vtk
    from vtk.util import numpy_support as _ns
    _VTK_OK = True
except Exception:  # pragma: no cover
    vtk = None
    _ns = None
    _VTK_OK = False


def vtk_available() -> bool:
    """True se o VTK importa. NÃO garante contexto GL — só uma tentativa de
    render prova isso, e é por isso que `VtkRobotRenderer` valida no
    construtor em vez de confiar neste teste."""
    return _VTK_OK


class VtkRendererError(RuntimeError):
    """VTK ausente, sem contexto GL ou incapaz de ler o framebuffer."""


def _polydata(tris: np.ndarray):
    """(N,3,3) → vtkPolyData de triângulos independentes."""
    n = tris.shape[0]
    pts = np.ascontiguousarray(tris.reshape(-1, 3), dtype=np.float64)
    vpts = vtk.vtkPoints()
    vpts.SetData(_ns.numpy_to_vtk(pts, deep=1))
    cells = np.hstack([
        np.full((n, 1), 3, dtype=np.int64),
        np.arange(3 * n, dtype=np.int64).reshape(n, 3),
    ]).ravel()
    arr = vtk.vtkCellArray()
    arr.SetCells(n, _ns.numpy_to_vtkIdTypeArray(cells, deep=1))
    pd = vtk.vtkPolyData()
    pd.SetPoints(vpts)
    pd.SetPolys(arr)
    return pd


def _line_polydata(pts: np.ndarray, colors: np.ndarray | None = None):
    """(2S,3) em pares consecutivos → vtkPolyData de segmentos."""
    s = pts.shape[0] // 2
    vpts = vtk.vtkPoints()
    vpts.SetData(_ns.numpy_to_vtk(
        np.ascontiguousarray(pts, dtype=np.float64), deep=1))
    cells = np.hstack([
        np.full((s, 1), 2, dtype=np.int64),
        np.arange(2 * s, dtype=np.int64).reshape(s, 2),
    ]).ravel()
    arr = vtk.vtkCellArray()
    arr.SetCells(s, _ns.numpy_to_vtkIdTypeArray(cells, deep=1))
    pd = vtk.vtkPolyData()
    pd.SetPoints(vpts)
    pd.SetLines(arr)
    if colors is not None:
        rgb = _ns.numpy_to_vtk(
            np.ascontiguousarray(colors, dtype=np.uint8), deep=1,
            array_type=vtk.VTK_UNSIGNED_CHAR)
        rgb.SetName('Colors')
        pd.GetCellData().SetScalars(rgb)
    return pd


class VtkRobotRenderer:
    """Renderiza uma `urdf_scene.RobotScene` completa via GPU, offscreen."""

    def __init__(self, scene, width: int, height: int,
                 background: tuple = (1.0, 1.0, 1.0)):
        if not _VTK_OK:
            raise VtkRendererError('VTK indisponível')
        self._scene = scene
        self._w = max(1, int(width))
        self._h = max(1, int(height))

        self._ren = vtk.vtkRenderer()
        self._ren.SetBackground(*background)
        self._ren.SetTwoSidedLighting(True)
        self._rw = vtk.vtkRenderWindow()
        self._rw.SetOffScreenRendering(1)
        self._rw.AddRenderer(self._ren)
        self._rw.SetSize(self._w, self._h)
        # Sem multisample: o quadro é lido de volta a 33 Hz e o AA dobraria
        # o custo do readback sem mudar a leitura da pose.
        self._rw.SetMultiSamples(0)

        self._actors: list = []
        self._links: list = []
        for part in scene.parts:
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(_polydata(part.tris))
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            prop = actor.GetProperty()
            prop.SetColor(*[c / 255.0 for c in part.color])
            # Ambiente alto: o "carbon black" da luva da COVVI vira um
            # borrão preto sem isso.
            prop.SetAmbient(0.35)
            prop.SetDiffuse(0.70)
            prop.SetSpecular(0.12)
            prop.SetSpecularPower(20)
            self._ren.AddActor(actor)
            self._actors.append(actor)
            self._links.append(part.link)

        # UMA matriz por ator, criada aqui e reutilizada: `SetUserMatrix`
        # guarda a REFERÊNCIA, então compartilhar uma única instância faria
        # todos os elos herdarem a última pose escrita — o robô inteiro
        # colapsa na origem.
        self._matrices = [vtk.vtkMatrix4x4() for _ in self._actors]
        for actor, m in zip(self._actors, self._matrices):
            actor.SetUserMatrix(m)

        self._grid_actor = None
        self._buf = vtk.vtkUnsignedCharArray()

        # Prova de fogo: sem contexto GL o Render falha aqui, no construtor,
        # e o caller cai para o rasterizador em software antes de a aba
        # aparecer meio desenhada.
        try:
            self._rw.Render()
            self._read_pixels()
        except Exception as exc:  # pragma: no cover
            raise VtkRendererError(f'render offscreen indisponível: {exc}')

    @property
    def triangle_count(self) -> int:
        return int(sum(p.tris.shape[0] for p in self._scene.parts))

    def set_ground(self, pts: np.ndarray, colors: np.ndarray) -> None:
        """Instala a grade do chão como geometria da cena — assim ela passa
        pelo mesmo z-buffer do robô e some corretamente atrás dos elos."""
        if self._grid_actor is not None:
            self._ren.RemoveActor(self._grid_actor)
            self._grid_actor = None
        if pts is None or len(pts) == 0:
            return
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(_line_polydata(np.asarray(pts, dtype=float),
                                           np.asarray(colors)))
        mapper.SetScalarModeToUseCellData()
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        prop = actor.GetProperty()
        prop.SetLineWidth(1.0)
        prop.LightingOff()
        self._ren.AddActor(actor)
        self._grid_actor = actor

    def resize(self, width: int, height: int) -> None:
        w, h = max(1, int(width)), max(1, int(height))
        if (w, h) != (self._w, self._h):
            self._w, self._h = w, h
            self._rw.SetSize(w, h)

    def _apply_camera(self, cam) -> None:
        """Espelha a `manip3d.Camera` no vtkCamera (ver o docstring do
        módulo para a equivalência das projeções)."""
        eye = cam.eye()
        c = self._ren.GetActiveCamera()
        c.SetPosition(*[float(v) for v in eye])
        c.SetFocalPoint(*[float(v) for v in cam.target])
        c.SetViewUp(0.0, 0.0, 1.0)
        c.SetUseHorizontalViewAngle(False)     # ângulo VERTICAL, como o nosso
        c.SetViewAngle(float(cam.fov_deg))
        # Plano distante generoso: a grade vai a ±1,5 m do centro e o zoom
        # chega a 8 m, então amarrar no bounding box do robô cortaria o chão.
        far = float(cam.dist) + 12.0
        c.SetClippingRange(float(cam.near), far)

    def _read_pixels(self) -> np.ndarray:
        ok = self._rw.GetPixelData(0, 0, self._w - 1, self._h - 1, 1,
                                   self._buf, 0)
        if not ok:
            raise VtkRendererError('GetPixelData falhou')
        arr = _ns.vtk_to_numpy(self._buf)
        if arr.size != self._w * self._h * 3:
            raise VtkRendererError('framebuffer com tamanho inesperado')
        # O VTK entrega da base para o topo; a imagem do Tk é do topo p/ baixo.
        return np.ascontiguousarray(
            arr.reshape(self._h, self._w, 3)[::-1])

    def render(self, transforms: dict, cam) -> np.ndarray:
        """Um quadro → array (H, W, 3) uint8 pronto para virar PIL.Image."""
        eye4 = np.eye(4)
        for actor, link, m in zip(self._actors, self._links, self._matrices):
            T = transforms.get(link, eye4)
            # DeepCopy de 16 floats numa chamada: SetElement elemento a
            # elemento seriam 1680 chamadas Python por quadro na mão COVVI.
            m.DeepCopy([float(v) for v in np.asarray(T).ravel()])
            actor.Modified()
        self._apply_camera(cam)
        self._rw.Render()
        return self._read_pixels()

    def close(self) -> None:
        try:
            self._ren.RemoveAllViewProps()
            self._rw.Finalize()
        except Exception:
            pass
        self._actors = []
        self._grid_actor = None
