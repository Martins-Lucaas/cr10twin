"""
manip3d.py — Viewport 3D interativa + IK diferencial do TCP do CR10.

Segunda fatia da modularização da palpation_gui (depois de ui_helpers):
tudo aqui é Tk puro + numpy + `kinematics`, SEM ROS. A GUI injeta dois
callbacks (`on_q` para publicar a nova pose, `q_provider` para ler a pose
corrente da cena) e o widget cuida do resto.

Como funciona o arrasto:

    1. o usuário pressiona o botão esquerdo sobre a alça do TCP;
    2. congela-se a PROFUNDIDADE de câmera do TCP no instante do clique —
       o mouse passa a andar sobre o plano paralelo à tela que contém o
       TCP, então 1 px de mouse corresponde sempre à mesma distância em
       metros durante todo o arrasto (sem deriva de escala);
    3. o alvo cartesiano resultante é perseguido por IK DIFERENCIAL (DLS
       sobre o Jacobiano geométrico fechado de `kinematics.jacobian`) num
       tick fixo de 33 Hz — a mesma cadência do streaming do
       tactile_explorer.

Por que IK diferencial e não `kinematics.inverse_kinematics`: a IK completa
varre ~16 sementes × 300 iterações para achar a MELHOR solução global. Isso
custa dezenas de ms e, pior, pode saltar entre ramos de cotovelo/pulso entre
dois frames consecutivos — o braço "estalaria" no meio do arrasto. O passo
diferencial parte SEMPRE da pose atual, é contínuo por construção e roda em
dezenas de µs, que é o que dá a sensação de fluidez pedida.
"""
from __future__ import annotations

import math
import time
import tkinter as tk
from dataclasses import dataclass, field

import numpy as np

from .kinematics import (
    JOINT_MAX, JOINT_MIN, T_TOUCH_TOOL_ATTACH,
    forward_kinematics, fk_partial, jacobian, manipulability, rot_error,
)
from .ui_helpers import (
    BORDER, DANGER, FONT_SMALL, OK, PANEL, PRIMARY, TEXT_DIM, TEXT_MUTED,
    WARN, _shade,
)

# Rasterização das malhas reais do URDF. Sem PIL a viewport cai para o
# desenho em esqueleto (linhas da FK), que não precisa de nada além do Tk.
try:
    from PIL import Image, ImageDraw, ImageTk
    _RASTER_OK = True
except Exception:  # pragma: no cover
    Image = ImageDraw = ImageTk = None
    _RASTER_OK = False

# Backend de GPU: mostra a malha EXATA do URDF (sem decimação).
try:
    from .vtk_render import VtkRobotRenderer, VtkRendererError, vtk_available
    _VTK_BACKEND_OK = True
except Exception:  # pragma: no cover
    VtkRobotRenderer = None
    VtkRendererError = RuntimeError

    def vtk_available() -> bool:
        return False
    _VTK_BACKEND_OK = False

# Cadência do laço de IK/render durante o arrasto — 33 Hz, igual ao
# streaming cartesiano do explorer (_CTRL_DT = 0.030).
TICK_MS = 30

# Limites default de um passo de IK.
MAX_LIN_STEP_M = 0.015     # 15 mm por iteração
MAX_JOINT_STEP_RAD = 0.06  # ≈3,4° por iteração e por junta
IK_ITERS = 6

# Damping do DLS. λ pequeno = mais preciso; λ grande = mais estável perto
# de singularidade. 0,06 é o mesmo valor usado no explorer (_JAC_LAM).
IK_LAMBDA = 0.06
# Peso da parcela de orientação no Jacobiano quando a trava está ligada.
# < 1 dá prioridade à posição (é ela que o usuário está arrastando).
ORI_WEIGHT = 0.35

# Raio de pick da alça do TCP, em pixels.
HANDLE_PICK_PX = 16.0

# Alcance nominal do CR10 (mm no datasheet) — desenhado como círculo-guia.
CR10_REACH_M = 1.375

def _hex_rgb(color: str) -> tuple:
    """'#rrggbb' → (r, g, b) 0..255."""
    h = color.lstrip('#')
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


_WORLD_UP = np.array([0.0, 0.0, 1.0])
_AXIS_VEC = {'X': np.array([1.0, 0.0, 0.0]),
             'Y': np.array([0.0, 1.0, 0.0]),
             'Z': np.array([0.0, 0.0, 1.0])}


# Câmera orbital

@dataclass
class Camera:
    """Câmera orbital com projeção perspectiva."""
    az: float = math.radians(-125.0)
    el: float = math.radians(20.0)
    dist: float = 2.3
    target: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.45]))
    fov_deg: float = 45.0
    near: float = 0.05

    # Limites da órbita: |el| < 89° evita o gimbal com o world-up.
    EL_LIM = math.radians(89.0)
    DIST_MIN = 0.6
    DIST_MAX = 8.0

    def eye(self) -> np.ndarray:
        ce, se = math.cos(self.el), math.sin(self.el)
        return self.target + self.dist * np.array(
            [math.cos(self.az) * ce, math.sin(self.az) * ce, se])

    def basis(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Retorna (right, up, forward) unitários no frame do mundo."""
        f = self.target - self.eye()
        f = f / (np.linalg.norm(f) + 1e-12)
        r = np.cross(f, _WORLD_UP)
        n = float(np.linalg.norm(r))
        if n < 1e-9:      # olhando reto para cima/baixo — escolhe um right
            r = np.array([1.0, 0.0, 0.0])
        else:
            r = r / n
        u = np.cross(r, f)
        return r, u, f

    def focal_px(self, height_px: int) -> float:
        return 0.5 * float(height_px) / math.tan(math.radians(self.fov_deg) / 2)

    def project(self, pts: np.ndarray, w: int, h: int
                ) -> tuple[np.ndarray, np.ndarray]:
        """Projeta pontos do mundo (N,3) → (uv (N,2) px, depth (N,) m)."""
        pts = np.atleast_2d(np.asarray(pts, dtype=float))
        r, u, f = self.basis()
        rel = pts - self.eye()
        x = rel @ r
        y = rel @ u
        z = rel @ f
        fpx = self.focal_px(h)
        safe = np.where(np.abs(z) < 1e-6, 1e-6, z)
        uv = np.empty((pts.shape[0], 2), dtype=float)
        uv[:, 0] = 0.5 * w + fpx * x / safe
        uv[:, 1] = 0.5 * h - fpx * y / safe
        return uv, z

    def unproject_delta(self, du: float, dv: float, depth: float,
                        h: int) -> np.ndarray:
        """Converte um deslocamento de MOUSE (px) num deslocamento no mundo
        (m), sobre o plano paralelo à tela que está a `depth` da câmera.
        """
        r, u, _f = self.basis()
        fpx = self.focal_px(h)
        s = float(depth) / fpx
        return (du * s) * r + (-dv * s) * u

    def orbit(self, d_az: float, d_el: float) -> None:
        self.az = (self.az + d_az + math.pi) % (2 * math.pi) - math.pi
        self.el = max(-self.EL_LIM, min(self.EL_LIM, self.el + d_el))

    def zoom(self, factor: float) -> None:
        self.dist = max(self.DIST_MIN, min(self.DIST_MAX,
                                           self.dist * float(factor)))

    def pan(self, du: float, dv: float, h: int) -> None:
        self.target = self.target - self.unproject_delta(du, dv, self.dist, h)


# Geometria do braço para desenho

def skeleton_points(q: np.ndarray,
                    T_end: np.ndarray = T_TOUCH_TOOL_ATTACH) -> np.ndarray:
    """Pontos (8,3) da cadeia: base → origens das juntas 1..6 → TCP."""
    q = np.asarray(q, dtype=float)
    pts = [np.zeros(3)]
    for n in range(1, 7):
        pts.append(fk_partial(q, n)[:3, 3])
    pts.append(forward_kinematics(q, T_end=T_end)[:3, 3])
    return np.asarray(pts, dtype=float)


def tcp_pose(q: np.ndarray,
             T_end: np.ndarray = T_TOUCH_TOOL_ATTACH) -> np.ndarray:
    """Pose 4×4 do TCP (atalho legível para o caller)."""
    return forward_kinematics(np.asarray(q, dtype=float), T_end=T_end)


def rpy_deg(R: np.ndarray) -> tuple[float, float, float]:
    """Extrai roll/pitch/yaw (convenção RPY fixa XYZ, graus) de R 3×3."""
    sp = float(np.clip(-R[2, 0], -1.0, 1.0))
    pitch = math.asin(sp)
    if abs(sp) > 0.99999:            # gimbal lock — yaw absorve o roll
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


# IK diferencial (DLS) — um passo por tick

@dataclass
class IKResult:
    q: np.ndarray            # nova pose articular (rad, convenção URDF)
    pos_err_m: float         # distância TCP → alvo ao fim do passo
    singular: bool           # Jacobiano não pôde ser resolvido
    at_limit: bool           # alguma junta saturou no limite articular
    manip: float             # manipulabilidade de Yoshikawa na nova pose


def ik_step(q: np.ndarray,
            p_target: np.ndarray,
            *,
            T_end: np.ndarray = T_TOUCH_TOOL_ATTACH,
            R_lock: np.ndarray | None = None,
            max_lin_m: float = MAX_LIN_STEP_M,
            max_dq: float = MAX_JOINT_STEP_RAD,
            iters: int = IK_ITERS,
            lam: float = IK_LAMBDA,
            tol_pos_m: float = 2e-4) -> IKResult:
    """Persegue `p_target` a partir de `q` por mínimos quadrados amortecidos.

    Args:
        q:          pose articular atual (6,) rad, convenção URDF
        p_target:   posição desejada do TCP (3,) m, frame da base
        T_end:      transform flange→TCP do efetuador em uso
        R_lock:     se dado, orientação 3×3 a MANTER durante o arrasto
                    (o usuário move o ponto, não a atitude da ferramenta)
        max_lin_m:  teto do erro linear atacado por iteração (limita a
                    velocidade do TCP e evita saltos)
        max_dq:     teto do passo de cada junta por iteração
        iters:      iterações por tick
    Returns:
        IKResult — sempre com uma pose VÁLIDA (nos limites articulares),
        mesmo quando o alvo é inalcançável: nesse caso o braço estica na
        direção do alvo e `pos_err_m` fica grande, que é o que a viewport
        mostra como "fora de alcance".
    """
    q = np.clip(np.asarray(q, dtype=float).copy(), JOINT_MIN, JOINT_MAX)
    p_target = np.asarray(p_target, dtype=float).flatten()
    I3, I6 = np.eye(3), np.eye(6)
    singular = False
    at_limit = False
    pos_err = float('inf')

    for _ in range(max(1, int(iters))):
        T = forward_kinematics(q, T_end=T_end)
        dp = p_target - T[:3, 3]
        pos_err = float(np.linalg.norm(dp))

        if R_lock is None:
            if pos_err < tol_pos_m:
                break
            if pos_err > max_lin_m:
                dp = dp * (max_lin_m / pos_err)
            J = jacobian(q, T_end=T_end)[:3, :]
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + lam * lam * I3, dp)
            except np.linalg.LinAlgError:
                singular = True
                break
        else:
            dw = rot_error(T[:3, :3], R_lock)
            ang_err = float(np.linalg.norm(dw))
            if pos_err < tol_pos_m and ang_err < 1e-3:
                break
            if pos_err > max_lin_m:
                dp = dp * (max_lin_m / pos_err)
            # Mesmo teto angular do passo linear, em rad — mantém as duas
            # parcelas na mesma ordem de grandeza dentro do DLS.
            if ang_err > max_dq:
                dw = dw * (max_dq / ang_err)
            J = jacobian(q, T_end=T_end).copy()
            J[3:, :] *= ORI_WEIGHT
            tw = np.concatenate([dp, ORI_WEIGHT * dw])
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + lam * lam * I6, tw)
            except np.linalg.LinAlgError:
                singular = True
                break

        dq = np.clip(dq, -max_dq, max_dq)
        q_next = np.clip(q + dq, JOINT_MIN, JOINT_MAX)
        # Saturou: o passo pedido foi maior do que o limite deixou passar.
        if np.any(np.abs((q + dq) - q_next) > 1e-9):
            at_limit = True
        q = q_next

    T = forward_kinematics(q, T_end=T_end)
    pos_err = float(np.linalg.norm(p_target - T[:3, 3]))
    return IKResult(q=q, pos_err_m=pos_err, singular=singular,
                    at_limit=at_limit, manip=manipulability(q))


# Rasterizador da cena URDF

# Direção da luz no frame da câmera (headlight deslocado p/ dar relevo).
_LIGHT_CAM = np.array([-0.35, 0.55, -0.75])
_AMBIENT = 0.42
_DIFFUSE = 0.58


def scene_triangles(scene, transforms: dict) -> tuple:
    """Aplica a FK do quadro → (tris (M,3,3) no mundo, cores (M,3))."""
    if getattr(scene, 'flat_tris', None) is None:
        tri_blocks, col_blocks = [], []
        for part in scene.parts:
            T = transforms.get(part.link)
            if T is None:
                continue
            tri_blocks.append(part.tris @ T[:3, :3].T + T[:3, 3])
            col_blocks.append(np.repeat(
                np.array(part.color, dtype=np.float64)[None, :],
                part.tris.shape[0], axis=0))
        if not tri_blocks:
            return np.zeros((0, 3, 3)), np.zeros((0, 3))
        return np.concatenate(tri_blocks), np.concatenate(col_blocks)

    eye4 = np.eye(4)
    mats = np.stack([transforms.get(link, eye4)
                     for link in scene.part_links])          # (P, 4, 4)
    idx = scene.flat_part
    R = mats[idx, :3, :3]                                    # (M, 3, 3)
    t = mats[idx, :3, 3]                                     # (M, 3)
    world = np.einsum('mij,mkj->mki', R, scene.flat_tris) + t[:, None, :]
    return world, scene.flat_colors


def shade_and_sort(tris: np.ndarray, colors: np.ndarray, cam: Camera,
                   w: int, h: int) -> tuple:
    """Projeta, descarta o que não se vê e devolve tudo pronto para pintar.

    Returns:
        (uv (K,3,2) px, rgb (K,3) uint8) já ordenados do mais distante para
        o mais próximo — é só percorrer e preencher.
    """
    if tris.shape[0] == 0:
        return np.zeros((0, 3, 2)), np.zeros((0, 3), dtype=np.uint8)

    eye = cam.eye()
    r, u, f = cam.basis()

    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    normals = np.cross(v1 - v0, v2 - v0)
    nn = np.linalg.norm(normals, axis=1)
    ok = nn > 1e-15
    normals = np.where(ok[:, None], normals / np.where(nn[:, None] < 1e-15,
                                                       1.0, nn[:, None]), 0.0)
    centroid = tris.mean(axis=1)
    view = eye - centroid

    # Backface culling pela normal do STL (winding CCW para fora).
    facing = np.einsum('ij,ij->i', normals, view)
    keep = ok & (facing > 0.0)
    if not np.any(keep):
        return np.zeros((0, 3, 2)), np.zeros((0, 3), dtype=np.uint8)

    tris, colors, normals = tris[keep], colors[keep], normals[keep]
    centroid = centroid[keep]

    rel = tris.reshape(-1, 3) - eye
    z = rel @ f
    # Um único vértice atrás do plano de corte invalida o triângulo: sem
    # clipping real, projetá-lo geraria um polígono espelhado atravessando
    # a tela.
    zt = z.reshape(-1, 3)
    front = np.all(zt > cam.near, axis=1)
    if not np.any(front):
        return np.zeros((0, 3, 2)), np.zeros((0, 3), dtype=np.uint8)

    fpx = cam.focal_px(h)
    uv = np.empty((tris.shape[0] * 3, 2))
    uv[:, 0] = 0.5 * w + fpx * (rel @ r) / z
    uv[:, 1] = 0.5 * h - fpx * (rel @ u) / z
    uv = uv.reshape(-1, 3, 2)[front]
    colors, normals = colors[front], normals[front]
    depth = zt[front].mean(axis=1)

    # Lambert com a luz fixa NA CÂMERA: gira junto com o observador, então
    # a peça nunca fica totalmente escura ao orbitar.
    light = _LIGHT_CAM[0] * r + _LIGHT_CAM[1] * u + _LIGHT_CAM[2] * f
    light = light / (np.linalg.norm(light) + 1e-12)
    lam = np.clip(normals @ (-light), 0.0, 1.0)
    shade = (_AMBIENT + _DIFFUSE * lam)[:, None]
    rgb = np.clip(colors * shade, 0, 255).astype(np.uint8)

    order = np.argsort(-depth)          # do mais longe para o mais perto
    return uv[order], rgb[order]


# Widget da viewport

class Manip3DView(tk.Canvas):
    """Canvas 3D com o CR10 renderizado e alça arrastável no TCP."""

    # Paleta dos elos — do ombro (escuro) ao punho (claro). O degradê é o
    # que dá a leitura da cadeia num desenho sem iluminação; o punho não
    # clareia mais do que #93c5fd para continuar legível sobre o branco.
    LINK_COLORS = ('#172554', '#1e3a8a', '#1e40af',
                   '#1d4ed8', '#2563eb', '#3b82f6', '#60a5fa')
    GRID_COLOR = '#dbe3ee'
    GRID_AXIS_X = '#f0a3a3'
    GRID_AXIS_Y = '#a3d5a8'

    def __init__(self, parent, *,
                 on_q=None,
                 q_provider=None,
                 T_end: np.ndarray = T_TOUCH_TOOL_ATTACH,
                 on_state=None,
                 on_drag_change=None,
                 **kw):
        kw.setdefault('bg', PANEL)
        kw.setdefault('highlightthickness', 1)
        kw.setdefault('highlightbackground', BORDER)
        super().__init__(parent, **kw)

        self._on_q = on_q
        self._q_provider = q_provider
        self._on_state = on_state
        self._on_drag_change = on_drag_change
        self._T_end = np.asarray(
            T_TOUCH_TOOL_ATTACH if T_end is None else T_end, dtype=float)

        self.cam = Camera()
        self._q = np.zeros(6, dtype=float)
        self._pts = skeleton_points(self._q, self._T_end)

        # Cena de malhas (urdf_scene.RobotScene) — opcional
        self._scene = None
        self._scene_coarse = None
        self._gpu = None                # VtkRobotRenderer | None
        self._exact = False
        self._extra_joints: dict = {}   # juntas fora do braço (mão COVVI)
        self._photo = None              # ImageTk.PhotoImage reaproveitada
        self._img_item = None
        self._photo_size = (0, 0)
        self.scene_status = ''          # mensagem exibida no HUD

        # Estado do arrasto do TCP
        self._dragging = False
        self._drag_depth = 1.0
        self._drag_origin_px = (0.0, 0.0)
        self._drag_origin_p = np.zeros(3)
        self._target = None            # alvo cartesiano perseguido pela IK
        self._R_lock = None
        self._hover = False

        # Estado do arrasto da câmera
        self._cam_mode = None          # 'orbit' | 'pan'
        self._cam_last = (0.0, 0.0)

        # Opções expostas à GUI
        self.lock_orientation = True
        self.axis_constraint = 'FREE'  # 'FREE' | 'X' | 'Y' | 'Z'
        self.max_lin_m = MAX_LIN_STEP_M
        self.max_dq = MAX_JOINT_STEP_RAD
        self.enabled = True            # False = arrasto bloqueado (gate da GUI)
        self.block_reason = ''

        self._last_result: IKResult | None = None
        self._after_id: str | None = None
        self._dirty = True
        # Render adaptativo: a IK e o publish rodam SEMPRE a 33 Hz (é o que
        # o robô sente); só o desenho pula quadros quando as malhas ficam
        # caras — a cena da mão COVVI custa ~30 ms, mais do que o tick.
        self._render_ms = 0.0
        self._frame_i = 0
        self._ground_cache = None
        self._static_w = -1
        self._static_h = -1

        self.bind('<ButtonPress-1>', self._on_press)
        self.bind('<B1-Motion>', self._on_motion)
        self.bind('<ButtonRelease-1>', self._on_release)
        self.bind('<Motion>', self._on_hover)
        self.bind('<ButtonPress-2>', self._on_cam_press)
        self.bind('<B2-Motion>', self._on_cam_motion)
        self.bind('<ButtonRelease-2>', self._on_cam_release)
        self.bind('<ButtonPress-3>', self._on_cam_press)
        self.bind('<B3-Motion>', self._on_cam_motion)
        self.bind('<ButtonRelease-3>', self._on_cam_release)
        self.bind('<MouseWheel>', self._on_wheel)
        self.bind('<Button-4>', self._on_wheel)
        self.bind('<Button-5>', self._on_wheel)
        self.bind('<Configure>', self._on_configure)

    # API pública

    @property
    def q(self) -> np.ndarray:
        return self._q.copy()

    @property
    def dragging(self) -> bool:
        return self._dragging

    def set_q(self, q, *, force: bool = False) -> None:
        """Define a pose desenhada. Ignorado durante o arrasto (a não ser
        com force=True) — quem manda no braço enquanto o mouse está
        pressionado é a IK, não o eco da cena."""
        if self._dragging and not force:
            return
        q_new = np.asarray(q, dtype=float).flatten()[:6]
        if q_new.shape[0] < 6:
            return
        if not force and np.allclose(q_new, self._q, atol=1e-5):
            return
        self._q = q_new
        self._pts = skeleton_points(self._q, self._T_end)
        self._dirty = True

    def set_end_effector(self, T_end: np.ndarray) -> None:
        self._T_end = np.asarray(T_end, dtype=float)
        self._pts = skeleton_points(self._q, self._T_end)
        self._dirty = True

    def set_scene(self, scene, status: str = '', coarse=None,
                  exact: bool = False) -> None:
        """Instala a cena de malhas (urdf_scene.RobotScene) ou None."""
        self._scene = scene
        self._scene_coarse = coarse
        self.scene_status = status
        self._release_gpu()
        if scene is not None and exact and _VTK_BACKEND_OK:
            try:
                w = max(1, self.winfo_width())
                h = max(1, self.winfo_height())
                self._gpu = VtkRobotRenderer(scene, w, h,
                                             background=self._bg_rgb())
                pts, colors = self._ground_geometry()
                self._gpu.set_ground(pts, np.array(
                    [_hex_rgb(c) for c in colors], dtype=np.uint8))
            except Exception as exc:
                self._gpu = None
                self.scene_status = f'GPU render unavailable ({exc})'
        self._exact = bool(exact)
        self._dirty = True

    def _bg_rgb(self) -> tuple:
        return tuple(c / 255.0 for c in _hex_rgb(PANEL))

    def _release_gpu(self) -> None:
        if self._gpu is not None:
            try:
                self._gpu.close()
            except Exception:
                pass
            self._gpu = None

    @property
    def rendering_exact(self) -> bool:
        """True quando o que está na tela é a malha exata do URDF."""
        return self._gpu is not None

    def set_extra_joints(self, values: dict) -> None:
        """Ângulos das juntas que NÃO são do braço (mão COVVI), vindos de
        /joint_states. As `mimic` do URDF derivam sozinhas destas."""
        if values and values != self._extra_joints:
            self._extra_joints = dict(values)
            self._dirty = True

    @property
    def rendering_meshes(self) -> bool:
        if self._gpu is not None:
            return True
        # Cena exata sem GPU: o rasterizador em software levaria ~1 s por
        # quadro, então o esqueleto é a opção honesta.
        return self._scene is not None and _RASTER_OK and not self._exact

    def set_view(self, name: str) -> None:
        """Presets de câmera: 'iso' | 'top' | 'front' | 'side'."""
        presets = {
            'iso':   (math.radians(-125.0), math.radians(20.0), 2.3),
            'top':   (math.radians(-90.0),  math.radians(88.0), 2.6),
            'front': (math.radians(180.0),  math.radians(2.0),  2.3),
            'side':  (math.radians(-90.0),  math.radians(2.0),  2.3),
        }
        az, el, dist = presets.get(name, presets['iso'])
        self.cam.az, self.cam.el, self.cam.dist = az, el, dist
        self.cam.target = np.array([0.0, 0.0, 0.45])
        self._dirty = True

    def abort_drag(self) -> None:
        """Cancela o arrasto em curso (o caller perdeu a permissão de mover
        o braço no meio do gesto). Público: a GUI chama isto quando o gate
        fecha — palpação iniciada, drag teach ativado, movimento em execução."""
        self._end_drag()

    def start(self) -> None:
        if self._after_id is None:
            self._after_id = self.after(TICK_MS, self._tick)

    def stop(self) -> None:
        if self._after_id is not None:
            try:
                self.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None
        self._end_drag()

    def destroy(self) -> None:
        """Solta o contexto GL antes de o widget morrer."""
        self.stop()
        self._release_gpu()
        super().destroy()

    def redraw(self) -> None:
        self._dirty = True

    # Laço principal

    def _tick(self) -> None:
        self._after_id = None
        try:
            if self._dragging and self._target is not None:
                self._solve_and_emit()
            elif self._q_provider is not None:
                q_scene = self._q_provider()
                if q_scene is not None:
                    self.set_q(q_scene)
            self._frame_i += 1
            # 1 quadro por tick enquanto o desenho couber em ~22 ms; acima
            # disso desenha 1 em cada 2 (ou 3) ticks. O tick em si não muda.
            period = 1 + int(self._render_ms // 22.0)
            if self._dirty and self._frame_i % period == 0:
                t0 = time.perf_counter()
                self._draw()
                dt = (time.perf_counter() - t0) * 1000.0
                self._render_ms = 0.8 * self._render_ms + 0.2 * dt
                self._dirty = False
        except tk.TclError:
            return          # widget destruído — encerra o laço sem reagendar
        try:
            if self.winfo_exists():
                self._after_id = self.after(TICK_MS, self._tick)
        except tk.TclError:
            pass

    def _solve_and_emit(self) -> None:
        res = ik_step(self._q, self._target,
                      T_end=self._T_end,
                      R_lock=self._R_lock if self.lock_orientation else None,
                      max_lin_m=self.max_lin_m,
                      max_dq=self.max_dq)
        self._last_result = res
        if not np.allclose(res.q, self._q, atol=1e-6):
            self._q = res.q
            self._pts = skeleton_points(self._q, self._T_end)
            self._dirty = True
            if self._on_q is not None:
                self._on_q(self._q.copy())
        if self._on_state is not None:
            self._on_state(self._q.copy(), res)

    # Eventos de mouse — TCP

    def _tcp_px(self) -> tuple[np.ndarray, float]:
        w, h = max(1, self.winfo_width()), max(1, self.winfo_height())
        uv, z = self.cam.project(self._pts[-1][None, :], w, h)
        return uv[0], float(z[0])

    def _near_handle(self, x: float, y: float) -> bool:
        uv, z = self._tcp_px()
        if z <= self.cam.near:
            return False
        return math.hypot(x - uv[0], y - uv[1]) <= HANDLE_PICK_PX

    def _on_press(self, e) -> None:
        self.focus_set()
        if self.enabled and self._near_handle(e.x, e.y):
            self._begin_drag(e.x, e.y)
        else:
            self._cam_mode = 'orbit'
            self._cam_last = (e.x, e.y)

    def _begin_drag(self, x: float, y: float) -> None:
        _uv, z = self._tcp_px()
        self._dragging = True
        self._drag_depth = z
        self._drag_origin_px = (float(x), float(y))
        self._drag_origin_p = self._pts[-1].copy()
        self._target = self._pts[-1].copy()
        self._R_lock = tcp_pose(self._q, self._T_end)[:3, :3].copy()
        self._dirty = True
        if self._on_drag_change is not None:
            self._on_drag_change(True)

    def _on_motion(self, e) -> None:
        if self._dragging:
            h = max(1, self.winfo_height())
            du = float(e.x) - self._drag_origin_px[0]
            dv = float(e.y) - self._drag_origin_px[1]
            delta = self.cam.unproject_delta(du, dv, self._drag_depth, h)
            if self.axis_constraint in _AXIS_VEC:
                axis = _AXIS_VEC[self.axis_constraint]
                delta = float(delta @ axis) * axis
            self._target = self._drag_origin_p + delta
            self._dirty = True
        elif self._cam_mode == 'orbit':
            dx = e.x - self._cam_last[0]
            dy = e.y - self._cam_last[1]
            self._cam_last = (e.x, e.y)
            self.cam.orbit(-dx * 0.010, dy * 0.010)
            self._dirty = True

    def _on_release(self, _e) -> None:
        if self._dragging:
            self._end_drag()
        self._cam_mode = None

    def _end_drag(self) -> None:
        if not self._dragging:
            return
        self._dragging = False
        self._target = None
        self._R_lock = None
        self._dirty = True
        if self._on_drag_change is not None:
            self._on_drag_change(False)

    def _on_hover(self, e) -> None:
        near = self.enabled and self._near_handle(e.x, e.y)
        if near != self._hover:
            self._hover = near
            self.config(cursor='fleur' if near else '')
            self._dirty = True

    # Eventos de mouse — câmera

    def _on_cam_press(self, e) -> None:
        self._cam_mode = 'pan'
        self._cam_last = (e.x, e.y)

    def _on_cam_motion(self, e) -> None:
        if self._cam_mode != 'pan':
            return
        dx = e.x - self._cam_last[0]
        dy = e.y - self._cam_last[1]
        self._cam_last = (e.x, e.y)
        self.cam.pan(dx, dy, max(1, self.winfo_height()))
        self._dirty = True

    def _on_cam_release(self, _e) -> None:
        self._cam_mode = None

    def _on_wheel(self, e) -> None:
        up = (getattr(e, 'num', None) == 4) or (getattr(e, 'delta', 0) > 0)
        self.cam.zoom(0.88 if up else 1.0 / 0.88)
        self._dirty = True

    def _on_configure(self, _e) -> None:
        self._dirty = True

    # Render

    def _draw(self) -> None:
        w, h = max(1, self.winfo_width()), max(1, self.winfo_height())
        self.delete('ov')
        if self.rendering_meshes:
            drew = self._draw_meshes(w, h)
        else:
            drew = False
        if not drew:
            self._clear_photo()
            self._draw_ground_vector(w, h)
            self._draw_skeleton(w, h)

        uv, z = self.cam.project(self._pts, w, h)
        self._draw_target(w, h, uv, z)
        self._draw_tcp(w, h, uv, z)
        self._draw_hud(w, h)

    # Caminho de malhas (PIL)

    def _active_scene(self):
        """Malha cheia quando parado; a reduzida enquanto o mouse arrasta ou
        a câmera se move — é aí que o quadro tem de caber no tick."""
        moving = self._dragging or self._cam_mode is not None
        if moving and self._scene_coarse is not None:
            return self._scene_coarse
        return self._scene

    def _link_transforms(self, scene) -> dict:
        values = {f'joint{i + 1}': float(self._q[i]) for i in range(6)}
        values.update(self._extra_joints)
        return scene.link_transforms(values)

    def _draw_meshes(self, w: int, h: int) -> bool:
        """Desenha chão + malhas e exibe. False se falhar (o caller cai para
        o esqueleto sem deixar o quadro em branco)."""
        if self._gpu is not None:
            try:
                self._gpu.resize(w, h)
                frame = self._gpu.render(self._link_transforms(self._scene),
                                         self.cam)
                self._blit(Image.fromarray(frame), w, h)
                return True
            except Exception as exc:
                # Contexto GL perdido (tela bloqueada, driver reiniciado):
                # solta a GPU e segue em software no próximo quadro.
                self._release_gpu()
                self.scene_status = f'GPU render lost ({exc})'
        try:
            scene = self._active_scene()
            img = Image.new('RGB', (w, h), PANEL)
            draw = ImageDraw.Draw(img)
            self._raster_ground(draw, w, h)
            self._raster_shadow(draw, w, h)

            tris, colors = scene_triangles(scene,
                                           self._link_transforms(scene))
            uv, rgb = shade_and_sort(tris, colors, self.cam, w, h)
            for poly, col in zip(uv, rgb):
                draw.polygon(
                    ((poly[0][0], poly[0][1]), (poly[1][0], poly[1][1]),
                     (poly[2][0], poly[2][1])),
                    fill=(int(col[0]), int(col[1]), int(col[2])))
            self._blit(img, w, h)
            return True
        except Exception:
            # Malha corrompida, memória, PIL sem backend… o esqueleto ainda
            # permite trabalhar; a viewport nunca deve derrubar a GUI.
            return False

    def _blit(self, img, w: int, h: int) -> None:
        if self._photo is None or self._photo_size != (w, h):
            self._photo = ImageTk.PhotoImage(img)
            self._photo_size = (w, h)
            if self._img_item is not None:
                self.delete(self._img_item)
            self._img_item = self.create_image(0, 0, anchor='nw',
                                               image=self._photo)
            self.tag_lower(self._img_item)
        else:
            self._photo.paste(img)

    def _clear_photo(self) -> None:
        if self._img_item is not None:
            self.delete(self._img_item)
            self._img_item = None
        self._photo = None
        self._photo_size = (0, 0)

    def _raster_ground(self, draw, w: int, h: int) -> None:
        for x0, y0, x1, y1, color in self._project_ground(w, h):
            draw.line((x0, y0, x1, y1), fill=color,
                      width=2 if color != self.GRID_COLOR else 1)

    def _raster_shadow(self, draw, w: int, h: int) -> None:
        shadow = self._pts.copy()
        shadow[:, 2] = 0.0
        uv, z = self.cam.project(shadow, w, h)
        for i in range(len(shadow) - 1):
            if z[i] <= self.cam.near or z[i + 1] <= self.cam.near:
                continue
            draw.line((uv[i][0], uv[i][1], uv[i + 1][0], uv[i + 1][1]),
                      fill='#e2e8f0', width=5)

    # Caminho de esqueleto (Tk vetorial)

    def _seg(self, uv, z, i0: int, i1: int, **kw) -> None:
        """Desenha o segmento i0→i1 se ambos estiverem à frente do near."""
        if z[i0] <= self.cam.near or z[i1] <= self.cam.near:
            return
        self.create_line(uv[i0][0], uv[i0][1], uv[i1][0], uv[i1][1],
                         tags='ov', **kw)

    def _draw_skeleton(self, w: int, h: int) -> None:
        pts = self._pts
        # Sombra no chão: mesmos pontos com z=0. Custa 8 projeções e é o
        # que dá noção de altura numa cena sem iluminação.
        shadow = pts.copy()
        shadow[:, 2] = 0.0
        uv_s, z_s = self.cam.project(shadow, w, h)
        for i in range(len(pts) - 1):
            self._seg(uv_s, z_s, i, i + 1,
                      fill='#e2e8f0', width=3, capstyle='round')

        uv, z = self.cam.project(pts, w, h)

        # Elos — largura afunila do ombro ao punho, com atenuação por
        # profundidade para reforçar a perspectiva.
        for i in range(len(pts) - 1):
            if z[i] <= self.cam.near or z[i + 1] <= self.cam.near:
                continue
            depth = 0.5 * (z[i] + z[i + 1])
            base_w = 9.0 - 0.9 * i
            wpx = max(2.0, base_w * (self.cam.dist / max(0.3, depth)))
            self.create_line(uv[i][0], uv[i][1], uv[i + 1][0], uv[i + 1][1],
                             fill=self.LINK_COLORS[min(i, 6)],
                             width=wpx, capstyle='round', tags='ov')

        # Juntas
        for i in range(1, len(pts) - 1):
            if z[i] <= self.cam.near:
                continue
            r = max(2.0, 5.0 * (self.cam.dist / max(0.3, z[i])))
            self.create_oval(uv[i][0] - r, uv[i][1] - r,
                             uv[i][0] + r, uv[i][1] + r,
                             fill=PANEL, outline='#1e3a8a', width=1.5,
                             tags='ov')

    def _ground_geometry(self) -> tuple:
        """Grade do chão (z=0) + eixos X/Y + círculo de alcance, como
        (pontos (2S,3), cores (S,)) — pares consecutivos formam um segmento.
        """
        if self._ground_cache is not None:
            return self._ground_cache
        step, half = 0.25, 1.5
        n = int(round(half / step))
        pts, colors = [], []
        for k in range(-n, n + 1):
            t = k * step
            pts += [[-half, t, 0.0], [half, t, 0.0]]
            colors.append(self.GRID_AXIS_X if k == 0 else self.GRID_COLOR)
            pts += [[t, -half, 0.0], [t, half, 0.0]]
            colors.append(self.GRID_AXIS_Y if k == 0 else self.GRID_COLOR)
        ang = np.linspace(0, 2 * math.pi, 61)
        circle = np.stack([CR10_REACH_M * np.cos(ang),
                           CR10_REACH_M * np.sin(ang),
                           np.zeros_like(ang)], axis=1)
        for i in range(len(circle) - 1):
            pts += [circle[i].tolist(), circle[i + 1].tolist()]
            colors.append('#cbd5e1')
        self._ground_cache = (np.asarray(pts, dtype=float), colors)
        return self._ground_cache

    def _project_ground(self, w: int, h: int):
        """Gera (x0, y0, x1, y1, cor) dos segmentos do chão que estão à
        frente do plano de corte."""
        pts, colors = self._ground_geometry()
        uv, z = self.cam.project(pts, w, h)
        for i, color in enumerate(colors):
            i0, i1 = 2 * i, 2 * i + 1
            if z[i0] <= self.cam.near or z[i1] <= self.cam.near:
                continue
            yield (uv[i0][0], uv[i0][1], uv[i1][0], uv[i1][1], color)

    def _draw_ground_vector(self, w: int, h: int) -> None:
        for x0, y0, x1, y1, color in self._project_ground(w, h):
            self.create_line(x0, y0, x1, y1, fill=color,
                             width=2 if color != self.GRID_COLOR else 1,
                             tags='ov')

    # Sobreposição interativa

    def _draw_target(self, w: int, h: int, uv, z) -> None:
        """Alvo do arrasto + linha tracejada TCP→alvo (mostra o quanto a IK
        ainda está devendo — o "rastro" do cursor)."""
        if not self._dragging or self._target is None:
            return
        uv_t, z_t = self.cam.project(self._target[None, :], w, h)
        if z_t[0] <= self.cam.near:
            return
        tx, ty = float(uv_t[0][0]), float(uv_t[0][1])
        if z[-1] > self.cam.near:
            self.create_line(uv[-1][0], uv[-1][1], tx, ty,
                             fill=WARN, width=1, dash=(3, 3), tags='ov')
        r = 7
        self.create_line(tx - r, ty, tx + r, ty, fill=WARN, width=2, tags='ov')
        self.create_line(tx, ty - r, tx, ty + r, fill=WARN, width=2, tags='ov')

    def _draw_tcp(self, w: int, h: int, uv, z) -> None:
        """Alça do TCP + triedro dos eixos da ferramenta."""
        if z[-1] <= self.cam.near:
            return
        p_tcp = self._pts[-1]
        R = tcp_pose(self._q, self._T_end)[:3, :3]
        L = 0.10
        tips = np.array([p_tcp + L * R[:, 0],
                         p_tcp + L * R[:, 1],
                         p_tcp + L * R[:, 2]])
        uv_a, z_a = self.cam.project(tips, w, h)
        for i, color in enumerate(('#dc2626', '#16a34a', '#2563eb')):
            if z_a[i] <= self.cam.near:
                continue
            self.create_line(uv[-1][0], uv[-1][1], uv_a[i][0], uv_a[i][1],
                             fill=color, width=2, arrow='last',
                             arrowshape=(8, 9, 3), tags='ov')

        if self._dragging:
            fill, outline = WARN, '#92400e'
        elif not self.enabled:
            fill, outline = '#e2e8f0', TEXT_DIM
        elif self._hover:
            fill, outline = _shade(PRIMARY, 0.35), PRIMARY
        else:
            fill, outline = PANEL, PRIMARY
        r = 9.0
        cx, cy = float(uv[-1][0]), float(uv[-1][1])
        if self._hover or self._dragging:
            self.create_oval(cx - r - 5, cy - r - 5, cx + r + 5, cy + r + 5,
                             outline=_shade(PRIMARY, 0.55), width=2, tags='ov')
        self.create_oval(cx - r, cy - r, cx + r, cy + r,
                         fill=fill, outline=outline, width=2, tags='ov')

    def _draw_hud(self, w: int, h: int) -> None:
        p = self._pts[-1]
        self.create_text(
            12, h - 26, anchor='sw', font=FONT_SMALL, fill=TEXT_MUTED,
            tags='ov',
            text=f'TCP  x {p[0] * 1000:+7.1f}   y {p[1] * 1000:+7.1f}   '
                 f'z {p[2] * 1000:+7.1f}  mm')
        if not self.enabled:
            hint, color = (self.block_reason or 'drag disabled'), DANGER
        elif self._dragging:
            axis = ('camera plane' if self.axis_constraint == 'FREE'
                    else f'{self.axis_constraint} axis')
            ori = 'orientation locked' if self.lock_orientation else 'free wrist'
            hint, color = f'dragging TCP — {axis} · {ori}', WARN
        else:
            hint, color = ('drag the TCP handle · left-drag = orbit · '
                           'right-drag = pan · wheel = zoom'), TEXT_DIM
        self.create_text(12, h - 10, anchor='sw', font=FONT_SMALL,
                         fill=color, text=hint, tags='ov')
        if self.scene_status:
            self.create_text(12, 14, anchor='nw', font=FONT_SMALL,
                             fill=TEXT_DIM, text=self.scene_status, tags='ov')
        res = self._last_result
        if res is not None and self._dragging:
            msg, color = '', OK
            if res.singular:
                msg, color = 'singular Jacobian', DANGER
            elif res.at_limit:
                msg, color = 'joint limit reached', WARN
            elif res.pos_err_m > 0.02:
                msg, color = f'lag {res.pos_err_m * 1000:.0f} mm', WARN
            if msg:
                self.create_text(w - 12, h - 10, anchor='se',
                                 font=FONT_SMALL, fill=color, text=msg,
                                 tags='ov')
