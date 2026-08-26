"""
urdf_scene.py — Carrega o URDF REAL da célula e o transforma numa cena de
triângulos pronta para rasterizar.

É o que faz a viewport 3D da GUI mostrar o MESMO robô que está no Gazebo:
as mesmas malhas (`cra_description/meshes/cr10/*.STL`, a pilha impressa do
touch_tool, a mão COVVI), os mesmos offsets de junta e as mesmas cores dos
materiais — nada é redesenhado à mão aqui.

De onde vem o URDF, em ordem de preferência:

  1. `robot_description_path` — o arquivo que o `tactile_cell.launch.py`
     gerou e entregou ao Gazebo. É a fonte exata: se o launch mudar a
     geometria, a viewport muda junto.
  2. `_build_robot_urdf(end_effector)` importado do próprio launch (o
     arquivo instalado em `share/touch_pack/launch/`). Cobre a GUI rodada
     standalone (`ros2 run touch_pack palpation_gui`) sem duplicar uma
     linha da montagem.

NB: `/robot_description` (o do robot_state_publisher) NÃO serve — o launch
publica ali a versão "mínima", com `<visual>`, `<collision>` e `<inertial>`
removidos por regex.

Sem ROS: o módulo é numpy + xml puro, testável sem display.
"""
from __future__ import annotations

import glob
import importlib.util
import math
import os
import re
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

# Orçamento total de triângulos da cena.
DEFAULT_TRIANGLE_BUDGET = 5000
# Piso por link: abaixo disto uma peça deixa de ser reconhecível.
MIN_TRIS_PER_LINK = 24

_IDENTITY = np.eye(4)


class UrdfSceneError(RuntimeError):
    """URDF ausente, malformado ou sem malhas resolvíveis."""


# Primitivas de transformação

def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _origin_to_T(elem: ET.Element | None) -> np.ndarray:
    """Lê um <origin xyz rpy> do URDF (ausente = identidade)."""
    T = np.eye(4)
    if elem is None:
        return T
    xyz = [float(v) for v in (elem.get('xyz') or '0 0 0').split()]
    rpy = [float(v) for v in (elem.get('rpy') or '0 0 0').split()]
    T[:3, :3] = rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues — rotação de junta revoluta/contínua em torno do seu eixo."""
    a = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        return np.eye(3)
    a = a / n
    c, s = math.cos(angle), math.sin(angle)
    K = np.array([[0.0, -a[2], a[1]],
                  [a[2], 0.0, -a[0]],
                  [-a[1], a[0], 0.0]])
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


# Malhas: leitura de STL e tesselagem de primitivas

def load_stl(path: str) -> np.ndarray:
    """Lê um STL (binário ou ASCII) → triângulos (N, 3, 3) em float64."""
    size = os.path.getsize(path)
    with open(path, 'rb') as fh:
        head = fh.read(84)
        if len(head) >= 84:
            n = struct.unpack('<I', head[80:84])[0]
            if size == 84 + n * 50 and n > 0:
                raw = np.frombuffer(fh.read(n * 50), dtype=np.uint8)
                if raw.size == n * 50:
                    rec = raw.reshape(n, 50)
                    v = rec[:, 12:48].copy().view('<f4').reshape(n, 3, 3)
                    return v.astype(np.float64)
        fh.seek(0)
        text = fh.read().decode('utf-8', errors='ignore')
    verts = np.array(
        [[float(x) for x in m] for m in re.findall(
            r'vertex\s+(-?[\d.eE+-]+)\s+(-?[\d.eE+-]+)\s+(-?[\d.eE+-]+)',
            text)], dtype=float)
    if verts.size == 0 or verts.shape[0] % 3 != 0:
        raise UrdfSceneError(f'STL ilegível: {path}')
    return verts.reshape(-1, 3, 3)


def tessellate_box(size: tuple) -> np.ndarray:
    sx, sy, sz = (float(v) / 2.0 for v in size)
    c = np.array([
        [-sx, -sy, -sz], [sx, -sy, -sz], [sx, sy, -sz], [-sx, sy, -sz],
        [-sx, -sy, sz], [sx, -sy, sz], [sx, sy, sz], [-sx, sy, sz]])
    faces = [(0, 3, 2), (0, 2, 1), (4, 5, 6), (4, 6, 7),
             (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
             (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return np.array([[c[i], c[j], c[k]] for i, j, k in faces])


def tessellate_cylinder(radius: float, length: float,
                        segments: int = 16) -> np.ndarray:
    r, hz = float(radius), float(length) / 2.0
    a = np.linspace(0.0, 2 * math.pi, segments, endpoint=False)
    ring = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)
    tris = []
    for i in range(segments):
        j = (i + 1) % segments
        p0 = np.array([ring[i][0], ring[i][1], -hz])
        p1 = np.array([ring[j][0], ring[j][1], -hz])
        p2 = np.array([ring[j][0], ring[j][1], hz])
        p3 = np.array([ring[i][0], ring[i][1], hz])
        tris.append([p0, p1, p2])
        tris.append([p0, p2, p3])
        tris.append([np.array([0.0, 0.0, hz]), p3, p2])
        tris.append([np.array([0.0, 0.0, -hz]), p1, p0])
    return np.array(tris)


def tessellate_sphere(radius: float, rings: int = 8,
                      segments: int = 12) -> np.ndarray:
    r = float(radius)
    tris = []
    for i in range(rings):
        t0 = math.pi * i / rings
        t1 = math.pi * (i + 1) / rings
        for j in range(segments):
            p0 = 2 * math.pi * j / segments
            p1 = 2 * math.pi * (j + 1) / segments

            def _p(t, p):
                return np.array([r * math.sin(t) * math.cos(p),
                                 r * math.sin(t) * math.sin(p),
                                 r * math.cos(t)])
            a, b = _p(t0, p0), _p(t1, p0)
            c, d = _p(t1, p1), _p(t0, p1)
            tris.append([a, b, c])
            tris.append([a, c, d])
    return np.array(tris)


def decimate(tris: np.ndarray, target: int) -> np.ndarray:
    """Reduz a malha por agrupamento de vértices numa grade regular."""
    n = tris.shape[0]
    if n <= target or n == 0:
        return tris
    pts = tris.reshape(-1, 3)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    diag = float(np.linalg.norm(hi - lo))
    if diag < 1e-9:
        return tris[:target]
    def _cluster(res: int) -> np.ndarray:
        cell = diag / res
        idx = np.floor((pts - lo) / cell).astype(np.int64)
        keys, inv = np.unique(idx, axis=0, return_inverse=True)
        centro = np.zeros((keys.shape[0], 3))
        np.add.at(centro, inv, pts)
        centro /= np.bincount(inv, minlength=keys.shape[0]
                              ).astype(float)[:, None]
        tri_idx = inv.reshape(-1, 3)
        # Só os degenerados saem (dois ou três vértices no mesmo cluster).
        ia, ib, ic = tri_idx.T
        keep = (ia != ib) & (ib != ic) & (ia != ic)
        return centro[tri_idx[keep]]

    # Grade mais fina = mais triângulos. Procura a resolução cujo resultado
    # é o maior que ainda cabe no orçamento; a malha entregue é sempre uma
    # clusterização íntegra, nunca uma subamostragem furada.
    res = 6
    best = _cluster(res)
    if best.shape[0] > target:
        # Peça pequena: já na grade mais grossa do laço de subida ela
        # estoura o alvo.
        while res > 2:
            res -= 1
            cand = _cluster(res)
            if cand.shape[0] == 0:
                break
            best = cand
            if cand.shape[0] <= target:
                break
        return best
    for _ in range(10):
        res = int(res * 1.45) + 1
        cand = _cluster(res)
        if cand.shape[0] > target:
            break
        best = cand
        if cand.shape[0] >= target * 0.85:
            break
    if best.shape[0] == 0:
        best = tris[:target]
    return best


# Resolução de caminhos package:// e file://

def resolve_uri(uri: str, search_dirs: tuple = ()) -> str | None:
    """Resolve o `filename` de um <mesh> para um caminho local."""
    uri = (uri or '').strip()
    if not uri:
        return None
    if uri.startswith('file://'):
        path = uri[len('file://'):]
        return path if os.path.exists(path) else None
    if uri.startswith('package://'):
        rest = uri[len('package://'):]
        pkg, _, tail = rest.partition('/')
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory(pkg), tail)
            if os.path.exists(path):
                return path
        except Exception:
            pass
        for d in search_dirs:
            path = os.path.join(d, pkg, tail)
            if os.path.exists(path):
                return path
            path = os.path.join(d, tail)
            if os.path.exists(path):
                return path
        return None
    return uri if os.path.exists(uri) else None


# Modelo do URDF

@dataclass
class Joint:
    name: str
    jtype: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    mimic: tuple | None = None      # (joint_alvo, multiplier, offset)


@dataclass
class VisualPart:
    """Um <visual> já tesselado, em triângulos NO FRAME DO LINK."""
    link: str
    tris: np.ndarray                # (N, 3, 3)
    color: tuple                    # (r, g, b) 0..255


@dataclass
class RobotScene:
    joints: list = field(default_factory=list)
    parts: list = field(default_factory=list)
    link_names: list = field(default_factory=list)
    root: str = ''
    missing_meshes: list = field(default_factory=list)

    # Vista achatada da cena, montada uma vez em `flatten()`: TODOS os
    # triângulos de TODAS as peças num array só, com o índice da peça a que
    # cada um pertence.
    flat_tris: np.ndarray | None = None      # (M, 3, 3) no frame do link
    flat_colors: np.ndarray | None = None    # (M, 3) float
    flat_part: np.ndarray | None = None      # (M,) índice em `parts`
    part_links: list = field(default_factory=list)
    _by_parent: dict | None = None           # índice pai→filhos (cache da FK)

    @property
    def triangle_count(self) -> int:
        return int(sum(p.tris.shape[0] for p in self.parts))

    def flatten(self) -> 'RobotScene':
        if not self.parts:
            return self
        self.flat_tris = np.concatenate([p.tris for p in self.parts])
        self.flat_colors = np.concatenate(
            [np.repeat(np.asarray(p.color, dtype=float)[None, :],
                       p.tris.shape[0], axis=0) for p in self.parts])
        self.flat_part = np.concatenate(
            [np.full(p.tris.shape[0], i, dtype=np.int64)
             for i, p in enumerate(self.parts)])
        self.part_links = [p.link for p in self.parts]
        return self

    def link_transforms(self, joint_values: dict,
                        frame: str = 'base_link') -> dict:
        """FK de toda a árvore → {link: T 4×4}, expressa em `frame`."""
        # Índice pai→filhos cacheado: remontá-lo a cada quadro custava ~1 ms
        # nas 107 juntas do URDF da mão.
        by_parent = self._by_parent
        if by_parent is None:
            by_parent = {}
            for j in self.joints:
                by_parent.setdefault(j.parent, []).append(j)
            self._by_parent = by_parent

        out = {self.root: np.eye(4)}
        stack = [self.root]
        while stack:
            parent = stack.pop()
            T_parent = out[parent]
            for j in by_parent.get(parent, ()):
                if j.mimic is not None:
                    src, mult, off = j.mimic
                    q = float(joint_values.get(src, 0.0)) * mult + off
                else:
                    q = float(joint_values.get(j.name, 0.0))
                T = T_parent @ j.origin
                if j.jtype in ('revolute', 'continuous'):
                    R = np.eye(4)
                    R[:3, :3] = axis_angle_to_matrix(j.axis, q)
                    T = T @ R
                elif j.jtype == 'prismatic':
                    D = np.eye(4)
                    D[:3, 3] = j.axis * q
                    T = T @ D
                out[j.child] = T
                stack.append(j.child)

        base = out.get(frame)
        if base is not None and frame != self.root:
            R_inv = base[:3, :3].T
            T_inv = np.eye(4)
            T_inv[:3, :3] = R_inv
            T_inv[:3, 3] = -R_inv @ base[:3, 3]
            out = {k: T_inv @ v for k, v in out.items()}
        return out


def _parse_color(visual: ET.Element, materials: dict) -> tuple:
    mat = visual.find('material')
    rgba = None
    if mat is not None:
        col = mat.find('color')
        if col is not None and col.get('rgba'):
            rgba = [float(v) for v in col.get('rgba').split()]
        elif mat.get('name') in materials:
            rgba = materials[mat.get('name')]
    if rgba is None:
        rgba = [0.75, 0.75, 0.78, 1.0]
    return tuple(int(max(0.0, min(1.0, c)) * 255) for c in rgba[:3])


def parse_urdf(xml_text: str, *,
               search_dirs: tuple = (),
               triangle_budget: int | None = DEFAULT_TRIANGLE_BUDGET
               ) -> RobotScene:
    """URDF (string) → RobotScene com todas as malhas carregadas."""
    try:
        root_el = ET.fromstring(xml_text)
    except ET.ParseError as exc:
        raise UrdfSceneError(f'URDF inválido: {exc}') from exc

    materials: dict = {}
    for mat in root_el.findall('material'):
        col = mat.find('color')
        if mat.get('name') and col is not None and col.get('rgba'):
            materials[mat.get('name')] = [
                float(v) for v in col.get('rgba').split()]

    joints: list = []
    for je in root_el.findall('joint'):
        parent = je.find('parent')
        child = je.find('child')
        if parent is None or child is None:
            continue
        axis_el = je.find('axis')
        axis = np.array([float(v) for v in (axis_el.get('xyz')
                                            if axis_el is not None
                                            else '1 0 0').split()])
        mimic_el = je.find('mimic')
        mimic = None
        if mimic_el is not None and mimic_el.get('joint'):
            mimic = (mimic_el.get('joint'),
                     float(mimic_el.get('multiplier', 1.0)),
                     float(mimic_el.get('offset', 0.0)))
        joints.append(Joint(
            name=je.get('name', ''),
            jtype=je.get('type', 'fixed'),
            parent=parent.get('link', ''),
            child=child.get('link', ''),
            origin=_origin_to_T(je.find('origin')),
            axis=axis,
            mimic=mimic))

    link_names = [le.get('name', '') for le in root_el.findall('link')]
    children = {j.child for j in joints}
    roots = [n for n in link_names if n not in children]
    if not roots:
        raise UrdfSceneError('URDF sem link raiz')
    # Prefere o link com filhos (evita eleger um `world` solto como raiz).
    parents = {j.parent for j in joints}
    root_name = next((n for n in roots if n in parents), roots[0])

    # Carrega as geometrias de cada <visual>
    raw: list = []
    missing: list = []
    for le in root_el.findall('link'):
        lname = le.get('name', '')
        visuals = le.findall('visual')
        # A mão COVVI recebe do `hand_pack.urdf_helpers` uma segunda visual
        # (`covvi_glove`): a MESMA malha 4% maior, em preto, imitando a luva
        # de silicone que cobre o mecanismo.
        glove = [v for v in visuals if (v.get('name') or '') == 'covvi_glove']
        if glove:
            visuals = glove
        for vis in visuals:
            geom = vis.find('geometry')
            if geom is None:
                continue
            T_vis = _origin_to_T(vis.find('origin'))
            tris = None
            mesh = geom.find('mesh')
            if mesh is not None:
                path = resolve_uri(mesh.get('filename', ''), search_dirs)
                if path is None:
                    missing.append(mesh.get('filename', ''))
                    continue
                try:
                    tris = load_stl(path)
                except (OSError, UrdfSceneError):
                    missing.append(path)
                    continue
                scale = mesh.get('scale')
                if scale:
                    tris = tris * np.array([float(v) for v in scale.split()])
            elif geom.find('box') is not None:
                tris = tessellate_box(
                    [float(v) for v in geom.find('box').get('size').split()])
            elif geom.find('cylinder') is not None:
                cyl = geom.find('cylinder')
                tris = tessellate_cylinder(float(cyl.get('radius')),
                                           float(cyl.get('length')))
            elif geom.find('sphere') is not None:
                tris = tessellate_sphere(
                    float(geom.find('sphere').get('radius')))
            if tris is None or tris.shape[0] == 0:
                continue
            tris = tris @ T_vis[:3, :3].T + T_vis[:3, 3]
            raw.append((lname, tris, _parse_color(vis, materials)))

    if not raw:
        raise UrdfSceneError('URDF sem nenhuma geometria visual carregável')

    if triangle_budget is None:
        parts = [VisualPart(link=lname, tris=tris, color=color)
                 for lname, tris, color in raw]
    else:
        # Reparte o orçamento pelo TAMANHO físico da peça: um elo grande
        # merece mais triângulos que uma arruela, independentemente de
        # quantos o CAD tenha exportado.
        spans = np.array([float(np.linalg.norm(t.reshape(-1, 3).max(axis=0)
                                               - t.reshape(-1, 3).min(axis=0)))
                          for _l, t, _c in raw])
        weights = spans / max(1e-9, float(spans.sum()))
        parts = []
        for (lname, tris, color), wgt in zip(raw, weights):
            target = max(MIN_TRIS_PER_LINK, int(triangle_budget * wgt))
            parts.append(VisualPart(link=lname, tris=decimate(tris, target),
                                    color=color))

    scene = RobotScene(joints=joints, parts=parts, link_names=link_names,
                       root=root_name, missing_meshes=missing)
    # A vista achatada só serve ao rasterizador em software e duplica a
    # memória da malha — na cena exata (420 mil triângulos na mão) isso são
    # 30 MB jogados fora, já que a GPU nunca a consulta.
    return scene.flatten() if triangle_budget is not None else scene


# Obtenção do URDF da célula

def _launch_module():
    """Importa `tactile_cell.launch.py` como módulo (share/ ou árvore-fonte)."""
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory('touch_pack'),
            'launch', 'tactile_cell.launch.py'))
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(
        os.path.dirname(here), 'launch', 'tactile_cell.launch.py'))
    for path in candidates:
        if not os.path.exists(path):
            continue
        spec = importlib.util.spec_from_file_location(
            '_touch_pack_tactile_cell_launch', path)
        if spec is None or spec.loader is None:
            continue
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    return None


def load_cell_urdf(end_effector: str = 'touch_tool',
                   description_path: str = '') -> str:
    """URDF completo da célula, com os <visual> intactos."""
    if description_path and os.path.exists(description_path):
        with open(description_path, encoding='utf-8') as fh:
            text = fh.read()
        if '<visual' in text:
            return text
    mod = _launch_module()
    if mod is None or not hasattr(mod, '_build_robot_urdf'):
        raise UrdfSceneError(
            'tactile_cell.launch.py não encontrado — sem URDF para a cena')
    full_urdf, _minimal = mod._build_robot_urdf(end_effector)
    return full_urdf


def coarse_scene(scene: RobotScene, factor: float = 0.40,
                 min_tris: int = 10) -> RobotScene:
    """Versão de baixo detalhe da mesma cena, para usar DURANTE o arrasto."""
    # `min_tris` bem abaixo de MIN_TRIS_PER_LINK de propósito: a mão COVVI
    # tem ~100 peças pequenas que, no piso normal, sozinhas consumiriam o
    # orçamento inteiro do quadro.
    parts = [VisualPart(
        link=p.link,
        tris=decimate(p.tris, max(min_tris, int(p.tris.shape[0] * factor))),
        color=p.color) for p in scene.parts]
    return RobotScene(joints=scene.joints, parts=parts,
                      link_names=scene.link_names, root=scene.root,
                      missing_meshes=scene.missing_meshes).flatten()


def default_search_dirs() -> tuple:
    """Diretórios extra para resolver package:// fora do ament index."""
    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.dirname(os.path.dirname(here))          # …/src
    dirs = [src]
    dirs.extend(glob.glob(os.path.join(src, '*')))
    return tuple(d for d in dirs if os.path.isdir(d))


def build_scene(end_effector: str = 'touch_tool',
                description_path: str = '',
                triangle_budget: int | None = DEFAULT_TRIANGLE_BUDGET
                ) -> RobotScene:
    """Atalho: URDF da célula → RobotScene pronta para renderizar."""
    return parse_urdf(load_cell_urdf(end_effector, description_path),
                      search_dirs=default_search_dirs(),
                      triangle_budget=triangle_budget)
