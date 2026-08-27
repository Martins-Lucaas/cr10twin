"""gui_ft_arrow.py — vista 3D do vetor de força, em paridade com a FIBOS.

O `Six_Axis_FT.exe` usa Qt5OpenGL e carrega duas malhas de `OBJ/`:
`arrowhead.obj` (箭头) e `direction.obj` (箭头2). São malhas pequenas — 176 e
238 faces — e é por isso que dá para reproduzir a vista no Tk sem OpenGL: um
rasterizador de polígonos com pintor + backface culling cabe em 200 chamadas
de canvas por quadro.

O QUE ESTE MÓDULO DESENHA
-------------------------
A resultante |F| = (fx, fy, fz) como uma seta 3D partindo da origem do
sensor, mais os três eixos de referência. O comprimento da seta é |F| contra o
fundo de escala da unidade; a direção é a do vetor. É a mesma leitura que o
painel 3D do cliente de fábrica dá: para onde a força aponta, e quão perto do
limite ela está.

DEPENDÊNCIA DE ASSET, E POR QUE ELA É OPCIONAL
----------------------------------------------
As duas OBJ vivem em `C:/Program Files (x86)/费波斯六维力客户端/OBJ/`, que só
existe na máquina onde o cliente da FIBOS está instalado. A GUI deste repo
roda no Linux do ROS, onde esse caminho não existe — e vendorizar asset de
terceiro no repositório é decisão de licença, não de engenharia.

A saída: o caminho é parâmetro (`ft_arrow_obj`), e sem OBJ legível o módulo
gera uma seta equivalente por procedimento (haste + cone). A vista é a mesma
nos dois casos; só a malha muda. `mesh_source()` diz qual está em uso, para o
painel não mentir sobre o que está mostrando.
"""
from __future__ import annotations

import math
import os
import tkinter as tk

from .constants import FT_RATED_FORCE_N
from .ui_helpers import (
    PANEL, TEXT, TEXT_MUTED, TEXT_DIM, BORDER, DANGER, OK,
    FONT_LBL, FONT_SMALL, FONT_MONO_S,
)

# Caminho padrão: a instalação do cliente de fábrica no Windows da bancada.
FACTORY_OBJ_DIR = r'C:\Program Files (x86)\费波斯六维力客户端\OBJ'
FACTORY_OBJ_NAME = 'arrowhead.obj'

_VIEW_W, _VIEW_H = 300, 260
# Teto de polígonos pré-criados no canvas. As duas malhas da FIBOS cabem
# folgadamente; o corte existe para uma OBJ trocada por engano não travar a
# GUI criando milhares de itens.
_MAX_FACES = 600


# ══════════════════════════════════════════════════════════════════════
# Malha
# ══════════════════════════════════════════════════════════════════════
def load_obj(path: str):
    """Lê um Wavefront .obj e devolve (vertices, faces_trianguladas).

    Suporta as três formas de índice que a exportação da Autodesk gera
    (`f v`, `f v/vt`, `f v/vt/vn`) e faces com mais de três vértices, que
    saem em leque. Índices negativos do .obj são relativos ao fim, e são
    resolvidos aqui — ignorá-los daria uma malha embaralhada em silêncio.
    """
    verts: list[tuple[float, float, float]] = []
    faces: list[tuple[int, ...]] = []
    with open(path, 'r', encoding='utf-8', errors='replace') as fh:
        for linha in fh:
            if linha.startswith('v '):
                p = linha.split()
                verts.append((float(p[1]), float(p[2]), float(p[3])))
            elif linha.startswith('f '):
                idx = []
                for campo in linha.split()[1:]:
                    n = int(campo.split('/')[0])
                    idx.append(n - 1 if n > 0 else len(verts) + n)
                for k in range(1, len(idx) - 1):
                    faces.append((idx[0], idx[k], idx[k + 1]))
    if not verts or not faces:
        raise ValueError(f'{path}: sem vértices ou sem faces')
    return verts, faces


def procedural_arrow(n_lados: int = 16):
    """Seta equivalente à da FIBOS: haste cilíndrica + cone, ao longo de +Z.

    Usada quando a OBJ do fabricante não está acessível. As proporções
    (haste com 70 % do comprimento e cone com raio 2,5x o da haste) são as
    que fazem a seta continuar legível quando ela aponta quase para o
    observador, que é o caso ruim de qualquer vista 3D de vetor.
    """
    r_haste, r_cone, z_cone = 0.055, 0.14, 0.70
    verts: list[tuple[float, float, float]] = []
    faces: list[tuple[int, ...]] = []

    def anel(z, r):
        base = len(verts)
        for i in range(n_lados):
            a = 2 * math.pi * i / n_lados
            verts.append((r * math.cos(a), r * math.sin(a), z))
        return base

    a0 = anel(0.0, r_haste)
    a1 = anel(z_cone, r_haste)
    a2 = anel(z_cone, r_cone)
    ponta = len(verts)
    verts.append((0.0, 0.0, 1.0))
    base_c = len(verts)
    verts.append((0.0, 0.0, 0.0))

    for i in range(n_lados):
        j = (i + 1) % n_lados
        faces.append((a0 + i, a1 + i, a1 + j))       # haste
        faces.append((a0 + i, a1 + j, a0 + j))
        faces.append((a2 + i, ponta, a2 + j))        # cone
        faces.append((a2 + i, a2 + j, a1 + j))       # coroa do cone
        faces.append((a2 + i, a1 + j, a1 + i))
        faces.append((base_c, a0 + j, a0 + i))       # tampa
    return verts, faces


def normalize_along_z(verts):
    """Põe a malha na forma canônica: base na origem, comprimento 1 em +Z.

    A OBJ do fabricante pode vir em qualquer escala e apontando para
    qualquer eixo. O eixo LONGO da caixa envolvente é o da seta — é a única
    pista robusta sem ler o .mtl — e ele é rotacionado para +Z.
    """
    xs, ys, zs = zip(*verts)
    ext = [max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)]
    eixo = ext.index(max(ext))
    comp = ext[eixo] or 1.0
    cen = [(max(c) + min(c)) / 2 for c in (xs, ys, zs)]
    lo = [min(xs), min(ys), min(zs)]

    out = []
    for v in verts:
        # Centra nos dois eixos curtos e ancora o longo na base.
        c = [(v[i] - cen[i]) / comp for i in range(3)]
        c[eixo] = (v[eixo] - lo[eixo]) / comp
        # Permuta para o eixo longo virar Z.
        if eixo == 0:
            out.append((c[1], c[2], c[0]))
        elif eixo == 1:
            out.append((c[2], c[0], c[1]))
        else:
            out.append((c[0], c[1], c[2]))
    return out


# ══════════════════════════════════════════════════════════════════════
# Álgebra da vista
# ══════════════════════════════════════════════════════════════════════
def rot_z_to(d):
    """Matriz que leva +Z até a direção unitária `d` (Rodrigues).

    O caso degenerado é d == -Z, em que o eixo de rotação some (produto
    vetorial nulo). Sem tratá-lo a seta desapareceria exatamente quando a
    força fosse de compressão pura no eixo Z — que é o caso mais comum da
    bancada, não um canto raro.
    """
    dx, dy, dz = d
    if dz > 0.999999:
        return ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    if dz < -0.999999:
        return ((1, 0, 0), (0, -1, 0), (0, 0, -1))
    ax, ay = -dy, dx                      # (0,0,1) x d, com az = 0
    n = math.hypot(ax, ay)
    ax, ay = ax / n, ay / n
    c, s, t = dz, n, 1 - dz
    return ((t * ax * ax + c, t * ax * ay, s * ay),
            (t * ax * ay, t * ay * ay + c, -s * ax),
            (-s * ay, s * ax, c))


def _mul(m, v):
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


# Vista isométrica fixa: gira o mundo para o observador ver os três eixos.
# Ângulos escolhidos para nenhum dos eixos do sensor ficar degenerado na
# projeção — com 0/0 o eixo Z viraria um ponto e a leitura sumiria.
_YAW, _PITCH = math.radians(35.0), math.radians(-24.0)
_CY, _SY = math.cos(_YAW), math.sin(_YAW)
_CP, _SP = math.cos(_PITCH), math.sin(_PITCH)
VIEW = ((_CY, -_SY, 0.0),
        (_CP * _SY, _CP * _CY, -_SP),
        (_SP * _SY, _SP * _CY, _CP))


def project(v, escala: float, cx: float, cy: float):
    """Ortográfica: devolve (x_px, y_px, profundidade)."""
    x, y, z = _mul(VIEW, v)
    return cx + x * escala, cy - y * escala, z


class FtArrowMixin:
    """Painel 3D do vetor de força resultante."""

    def _ft_arrow_init(self, obj_path: str = '') -> None:
        caminho = obj_path or os.path.join(FACTORY_OBJ_DIR, FACTORY_OBJ_NAME)
        try:
            verts, faces = load_obj(caminho)
            self._ft_arrow_src = os.path.basename(caminho)
        except Exception:
            # Ausência da instalação do fabricante é o caso NORMAL fora da
            # bancada Windows, não uma falha: cair na malha procedural em
            # silêncio é o comportamento certo, e o painel diz qual está no ar.
            verts, faces = procedural_arrow()
            self._ft_arrow_src = 'procedural'
        if len(faces) > _MAX_FACES:
            faces = faces[:_MAX_FACES]
        self._ft_arrow_verts = normalize_along_z(verts)
        self._ft_arrow_faces = faces
        self._ft_arrow_items: list[int] = []

    def mesh_source(self) -> str:
        return getattr(self, '_ft_arrow_src', 'procedural')

    # ── Construção ────────────────────────────────────────────────────
    def _build_ft_arrow_card(self, root: tk.Frame) -> None:
        self._ft_arrow_init(getattr(self, '_ft_arrow_obj_path', ''))
        card = self._card(root, 'Force vector — 3D', expand=False)

        linha = tk.Frame(card, bg=PANEL)
        linha.pack(fill='x', pady=(6, 2))

        cv = tk.Canvas(linha, width=_VIEW_W, height=_VIEW_H, bg=PANEL,
                       highlightthickness=1, highlightbackground=BORDER)
        cv.pack(side='left')
        self._ft_arrow_cv = cv

        # Eixos de referência, desenhados uma vez: eles não giram com a força.
        cx, cy, esc = _VIEW_W / 2, _VIEW_H / 2 + 30, _VIEW_H * 0.30
        for vetor, rotulo, cor in (((1, 0, 0), 'X', '#2563eb'),
                                   ((0, 1, 0), 'Y', '#16a34a'),
                                   ((0, 0, 1), 'Z', '#dc2626')):
            px, py, _ = project(vetor, esc, cx, cy)
            cv.create_line(cx, cy, px, py, fill=cor, width=1, dash=(3, 3))
            cv.create_text(px, py, text=rotulo, fill=cor, font=FONT_SMALL)
        cv.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=TEXT_DIM,
                       outline='')

        # Polígonos da malha, pré-criados. Recriar 200 itens a cada quadro
        # fragmenta a display list do Tk; aqui só as coords mudam.
        for _ in self._ft_arrow_faces:
            self._ft_arrow_items.append(
                cv.create_polygon(0, 0, 0, 0, 0, 0, fill='', outline=''))

        info = tk.Frame(linha, bg=PANEL)
        info.pack(side='left', fill='both', expand=True, padx=(14, 0))
        tk.Label(info, text='|F|', font=FONT_SMALL, bg=PANEL,
                 fg=TEXT_DIM, anchor='w').pack(fill='x')
        self._ft_arrow_mag = tk.Label(info, text='—', font=FONT_LBL,
                                      bg=PANEL, fg=TEXT_DIM, anchor='w')
        self._ft_arrow_mag.pack(fill='x')
        tk.Label(info, text='direction (unit)', font=FONT_SMALL, bg=PANEL,
                 fg=TEXT_DIM, anchor='w').pack(fill='x', pady=(10, 0))
        self._ft_arrow_dir = tk.Label(info, text='—', font=FONT_MONO_S,
                                      bg=PANEL, fg=TEXT_DIM, anchor='w',
                                      justify='left')
        self._ft_arrow_dir.pack(fill='x')
        tk.Label(info,
                 text=(f'Length is |F| against the rated '
                       f'{FT_RATED_FORCE_N:.0f} N.\nMesh: '
                       f'{self.mesh_source()}'),
                 font=FONT_SMALL, bg=PANEL, fg=TEXT_DIM, anchor='w',
                 justify='left', wraplength=200).pack(fill='x', pady=(12, 0))

    # ── Repintura ─────────────────────────────────────────────────────
    def _refresh_ft_arrow(self, shown: dict, live: bool) -> None:
        if not getattr(self, '_ft_arrow_items', None):
            return
        cv = self._ft_arrow_cv
        fx = shown.get('fx')
        fy = shown.get('fy')
        fz = shown.get('fz')
        if not live or None in (fx, fy, fz):
            self._ft_arrow_mag.config(text='—', fg=TEXT_DIM)
            self._ft_arrow_dir.config(text='—', fg=TEXT_DIM)
            for it in self._ft_arrow_items:
                cv.itemconfigure(it, fill='', outline='')
            return

        mag = math.sqrt(fx * fx + fy * fy + fz * fz)
        frac = min(mag / FT_RATED_FORCE_N, 1.0) if FT_RATED_FORCE_N else 0.0
        self._ft_arrow_mag.config(
            text=f'{mag:.2f} N   ({frac * 100:.0f} % of rated)',
            fg=DANGER if frac >= 1.0 else (OK if mag > 0.5 else TEXT_MUTED))

        if mag < 1e-6:
            # Sem direção definida: esconder é honesto, desenhar uma seta
            # apontando para um lugar arbitrário não é.
            self._ft_arrow_dir.config(text='(no force)', fg=TEXT_DIM)
            for it in self._ft_arrow_items:
                cv.itemconfigure(it, fill='', outline='')
            return

        d = (fx / mag, fy / mag, fz / mag)
        self._ft_arrow_dir.config(
            text=f'x {d[0]:+.3f}\ny {d[1]:+.3f}\nz {d[2]:+.3f}', fg=TEXT)

        rot = rot_z_to(d)
        # Comprimento proporcional a |F|, com um piso: uma seta de comprimento
        # zero não mostraria a direção, que é metade da informação.
        comp = 0.25 + 0.75 * frac
        cx, cy, esc = _VIEW_W / 2, _VIEW_H / 2 + 30, _VIEW_H * 0.30

        mundo = [_mul(rot, (v[0] * 0.35, v[1] * 0.35, v[2] * comp))
                 for v in self._ft_arrow_verts]
        proj = [project(v, esc, cx, cy) for v in mundo]

        # Pintor: as faces mais fundas primeiro. Sem isto a seta fica com o
        # cone atrás da haste quando ela aponta para longe do observador.
        ordem = sorted(range(len(self._ft_arrow_faces)),
                       key=lambda i: sum(
                           proj[k][2] for k in self._ft_arrow_faces[i]))
        base = (0x25, 0x63, 0xeb) if frac < 1.0 else (0xdc, 0x26, 0x26)
        for pos, i in enumerate(ordem):
            a, b, c = self._ft_arrow_faces[i]
            (xa, ya, _), (xb, yb, _), (xc, yc, _) = proj[a], proj[b], proj[c]
            area = (xb - xa) * (yc - ya) - (xc - xa) * (yb - ya)
            item = self._ft_arrow_items[pos]
            if area <= 0:
                # Backface: some. Corta ~metade dos polígonos por quadro, que
                # é o que mantém o custo do Tk dentro do orçamento.
                cv.itemconfigure(item, fill='', outline='')
                continue
            # Lambert barato: normal projetada contra a área da face na tela.
            lum = 0.45 + 0.55 * min(area / 260.0, 1.0)
            cor = '#%02x%02x%02x' % tuple(
                min(255, int(ch * lum + 40)) for ch in base)
            cv.coords(item, xa, ya, xb, yb, xc, yc)
            cv.itemconfigure(item, fill=cor, outline=cor)
