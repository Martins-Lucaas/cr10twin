"""Helpers de pós-processamento do URDF da mão COVVI."""

from __future__ import annotations

import re
from typing import Dict


# Limites factíveis das juntas, baseados no manual técnico da COVVI Hand
# (CV-000918-TC Rev.
HAND_DRIVER_LIMITS: Dict[str, float] = {
    'Thumb':  1.00,   # ~57° driver → 102° na ponta
    'Index':  1.00,   # ~57° driver → 163° na ponta (wrap de power-grip)
    'Middle': 1.00,
    'Ring':   1.00,
    'Little': 1.00,
    'Rotate': 1.00,
}

# Com `enable_inter_finger_self_collision` ativo (skin + palm em
# self_collide=true), a falange distal NÃO atravessa a palma mesmo no cap

# `lower` calibrado — equivalente ao `open_limit` do DigitConfigMsg da mão
# real.
HAND_DRIVER_LOWER: Dict[str, float] = {
    'Thumb':  0.08,
    'Index':  0.12,
    'Middle': 0.12,
    'Ring':   0.12,
    'Little': 0.12,
    'Rotate': 0.00,   # oposição mantém neutra (palma aberta)
}


def clamp_hand_joint_limits(urdf_body: str) -> str:
    """Ajusta ``lower``/``upper`` dos drivers e propaga para os mimics."""
    joint_re = re.compile(r'<joint\b[^>]*?\bname="([^"]+)"[^>]*?>.*?</joint>',
                          re.DOTALL)
    mimic_re = re.compile(
        r'<mimic\s+joint="([^"]+)"\s+multiplier="([-\deE.+]+)"'
        r'(?:\s+offset="[-\deE.+]+")?\s*/>')
    limit_lower_re = re.compile(r'(<limit\b[^/]*?\blower=")([-\deE.+]+)(")')
    limit_upper_re = re.compile(r'(<limit\b[^/]*?\bupper=")([-\deE.+]+)(")')

    def _patch_joint(match: re.Match) -> str:
        jxml = match.group(0)
        jname = match.group(1)

        new_lower: float | None = None
        new_upper: float | None = None
        if jname in HAND_DRIVER_LIMITS:
            new_lower = HAND_DRIVER_LOWER[jname]
            new_upper = HAND_DRIVER_LIMITS[jname]
        else:
            m = mimic_re.search(jxml)
            if m is not None:
                driver = m.group(1)
                mult = float(m.group(2))
                if driver in HAND_DRIVER_LIMITS:
                    a = mult * HAND_DRIVER_LOWER[driver]
                    b = mult * HAND_DRIVER_LIMITS[driver]
                    new_lower, new_upper = (a, b) if a <= b else (b, a)

        if new_upper is None:
            return jxml

        jxml = limit_lower_re.sub(
            lambda lm: f'{lm.group(1)}{new_lower:.8f}{lm.group(3)}',
            jxml, count=1)
        jxml = limit_upper_re.sub(
            lambda lm: f'{lm.group(1)}{new_upper:.8f}{lm.group(3)}',
            jxml, count=1)
        return jxml

    return joint_re.sub(_patch_joint, urdf_body)


# 2) Inércia "fantasma" dos eixos virtuais — pequena & estável
def fix_virtual_link_inertia(urdf_body: str) -> str:
    phantom = (
        r'<inertial>\s*'
        r'<mass value="1"\s*/>\s*'
        r'<inertia ixx="1\.0" ixy="0\.0" ixz="0\.0" iyy="1\.0" iyz="0\.0" izz="1\.0"\s*/>\s*'
        r'</inertial>'
    )
    minimal = (
        '<inertial>'
        '<mass value="0.001"/>'
        '<inertia ixx="1e-9" ixy="0.0" ixz="0.0" iyy="1e-9" iyz="0.0" izz="1e-9"/>'
        '</inertial>'
    )
    return re.sub(phantom, minimal, urdf_body, flags=re.DOTALL)


# 3) Dinâmica das juntas — preensão por contato físico
def stabilize_hand_joints(urdf_body: str) -> str:
    """Patcha damping/friction/effort das revolute joints."""
    def _patch(m: re.Match) -> str:
        jxml = m.group(0)
        if 'type="revolute"' not in jxml:
            return jxml
        is_mimic = '<mimic' in jxml
        damp, fric = (30.0, 10.0) if is_mimic else (5.0, 1.0)
        dyn_tag = f'<dynamics damping="{damp}" friction="{fric}"/>'
        if '<dynamics' in jxml:
            jxml = re.sub(r'<dynamics[^/]*/>', dyn_tag, jxml)
        else:
            jxml = jxml.replace('</joint>',
                                f'      {dyn_tag}\n    </joint>')
        if not is_mimic:
            jxml = re.sub(r'effort="[\d.]+"', 'effort="8.0"', jxml)
        return jxml
    return re.sub(r'<joint\b[^>]*>.*?</joint>', _patch, urdf_body, flags=re.DOTALL)


# 4) Camada de "pele" macia — análoga à luva de silicone IP44 da COVVI
def inject_skin_layer(urdf_body: str, inflate_m: float = 0.002) -> str:
    """Adiciona <collision name="skin"> ao redor de cada falange + palma.

    Args:
        urdf_body: corpo do URDF (sem a tag <robot> externa, ou com — não
            importa: as substituições são por padrão regex local).
        inflate_m: espessura por face da "pele", em metros. Default 2 mm
            (a COVVI publica luva IP44 com perfil similar). Valores típicos:
            0.002 (pacote padrão), 0.003 (envelope mais conservador,
            agravam auto-colisão), 0.001 (pele mínima, cantos ainda
            visíveis ao objeto).
    """
    finger_link_pat = re.compile(
        r'(<link\s+name="(?:thumb|index|middle|ring|little)_(?:proximal|distal)"[^>]*>'
        r'.*?</link>)',
        re.DOTALL)
    box_coll_pat = re.compile(
        r'(<collision>\s*<geometry>\s*<box\s+size="([^"]+)"\s*/>\s*</geometry>'
        r'\s*<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"\s*/>\s*</collision>)',
        re.DOTALL)

    def _patch_finger_link(m: re.Match) -> str:
        link_xml = m.group(0)
        if '<collision name="skin"' in link_xml:
            return link_xml  # idempotência

        def _patch_collision(cm: re.Match) -> str:
            original = cm.group(1)
            sizes = [float(s) for s in cm.group(2).split()]
            xyz = cm.group(3)
            rpy = cm.group(4)
            inflated = ' '.join(
                f'{s + 2 * inflate_m:.5f}' for s in sizes)
            skin = (
                f'\n        <collision name="skin">'
                f'<geometry><box size="{inflated}"/></geometry>'
                f'<origin xyz="{xyz}" rpy="{rpy}"/>'
                f'</collision>'
            )
            return original + skin

        return box_coll_pat.sub(_patch_collision, link_xml, count=1)

    urdf_body = finger_link_pat.sub(_patch_finger_link, urdf_body)

    # Palma (link 'lisa') — colisão de malha + box-skin aproximado.
    if '<collision name="palm_skin"' not in urdf_body:
        palm_skin = (
            '\n        <collision name="palm_skin">'
            '<geometry><box size="0.085 0.092 0.045"/></geometry>'
            '<origin xyz="0 0.046 0" rpy="0 0 0"/>'
            '</collision>'
        )
        urdf_body = re.sub(
            r'(<link\s+name="lisa">.*?<collision>\s*<geometry>\s*<mesh\b[^>]*?/>'
            r'\s*</geometry>\s*<origin\b[^>]*?/>\s*</collision>)',
            lambda m: m.group(1) + palm_skin,
            urdf_body, count=1, flags=re.DOTALL)

    return urdf_body


# Habilitar colisão ENTRE dedos diferentes: por default, links que
# compartilham junta no URDF não geram contatos entre si (ODE/Gazebo
# suprimem pares "estruturalmente conectados").

FINGER_PHALANGE_LINKS: tuple = tuple(
    f'{f}_{s}'
    for f in ('thumb', 'index', 'middle', 'ring', 'little')
    for s in ('proximal', 'distal')
)
PALM_LINK = 'lisa'

INTER_FINGER_COLLISION_LINKS: tuple = FINGER_PHALANGE_LINKS + (PALM_LINK,)


def enable_inter_finger_self_collision(urdf_body: str) -> str:
    """Ativa ``<self_collide>true</self_collide>`` nas falanges e palma."""
    parts: list[str] = []
    for link in INTER_FINGER_COLLISION_LINKS:
        # Remove qualquer tag self_collide existente (true ou false)
        # para este link.
        pattern = re.compile(
            rf'<gazebo\s+reference="{re.escape(link)}"\s*>'
            rf'\s*<self_collide>\s*(?:true|false)\s*</self_collide>'
            rf'\s*</gazebo>',
            re.DOTALL)
        urdf_body = pattern.sub('', urdf_body)
        parts.append(
            f'    <gazebo reference="{link}">'
            f'<self_collide>true</self_collide>'
            f'</gazebo>')

    block = '\n' + '\n'.join(parts) + '\n'
    if '</robot>' in urdf_body:
        return urdf_body.replace('</robot>', block + '</robot>', 1)
    return urdf_body + block


# Pele VISUAL: a mão real é coberta por luva de silicone, mas o URDF do
# Onshape mostra aço/alumínio cru.

_SKIN_VISUAL_SCALE = 1.04        # 4% maior por eixo
_SKIN_VISUAL_COLOR = '0.12 0.12 0.13 1.0'   # Carbon Black opaco
_SKIN_VISUAL_NAME = 'covvi_glove'


def inject_visual_skin_layer(urdf_body: str,
                             scale_factor: float = _SKIN_VISUAL_SCALE,
                             color_rgba: str = _SKIN_VISUAL_COLOR) -> str:
    """Adiciona um <visual name="skin"> escalado sobre cada falange + palma."""
    target_links = {f'{f}_{s}' for f in
                    ('thumb', 'index', 'middle', 'ring', 'little')
                    for s in ('proximal', 'distal')}
    target_links.add('lisa')

    # Importante: excluir self-closing (<link name="x" />), senão a captura
    # casa o auto-fechado e devora conteúdo até o próximo </link>.
    link_re = re.compile(
        r'(<link\s+name="([^"]+)"[^/>]*>)(?!\s*</link>)(.*?)(</link>)',
        re.DOTALL)
    visual_mesh_re = re.compile(
        r'<visual>\s*'
        r'(<origin\b[^/]*/>)\s*'
        r'<geometry>\s*'
        r'<mesh\s+filename="([^"]+)"\s+scale="([\d.eE+\- ]+)"\s*/>\s*'
        r'</geometry>\s*'
        r'<material\b[^>]*>.*?</material>\s*'
        r'</visual>',
        re.DOTALL)

    def _patch_link(m: re.Match) -> str:
        open_tag, link_name, body, close_tag = m.groups()
        if link_name not in target_links:
            return m.group(0)
        if f'<visual name="{_SKIN_VISUAL_NAME}"' in body:
            return m.group(0)  # idempotência

        vm = visual_mesh_re.search(body)
        if vm is None:
            return m.group(0)

        origin = vm.group(1)
        mesh_file = vm.group(2)
        scale_vals = [float(s) for s in vm.group(3).split()]
        new_scale = ' '.join(f'{s * scale_factor:.8f}' for s in scale_vals)
        skin_visual = (
            f'\n        <visual name="{_SKIN_VISUAL_NAME}">\n'
            f'            {origin}\n'
            f'            <geometry>\n'
            f'                <mesh filename="{mesh_file}" scale="{new_scale}" />\n'
            f'            </geometry>\n'
            f'            <material name="{_SKIN_VISUAL_NAME}_mat">\n'
            f'                <color rgba="{color_rgba}" />\n'
            f'            </material>\n'
            f'        </visual>'
        )
        # Insere ao final do body do link, antes de </link>
        return open_tag + body + skin_visual + '\n    ' + close_tag

    return link_re.sub(_patch_link, urdf_body)


# Transmissão de força pelos dedos subatuados (preensão por física)
#
# Gazebo Classic + `<mimic>` (seja pela tag nativa do gazebo_ros2_control,
# seja pelo plugin sem `<hasPID>`) impõe a junta mimic por SetPosition —
# CINEMÁTICO, não aplica torque. As falanges distais que abraçam o objeto
# são arrastadas para um ângulo mas não empurram, então a mão não segura
# nada por atrito. Isto reintroduz o roboticsgroup_gazebo_plugins
# (`libgazebo_mimic_joint_plugin.so`, presente no `.urdf.bak`) com
# `<hasPID>` nas juntas do caminho de contato: aí a mimic é seguida por um
# PID que aplica força até `<maxEffort>`, transmitindo a preensão.
#
# `.so` ausente do GAZEBO_PLUGIN_PATH → Gazebo loga o erro e segue (a mão
# some de força mas não quebra). Ver src/grasp_ml_pack/doc/covvi_control.md.

# Juntas mimic cujo elo-filho toca o objeto: recebem <hasPID> (PID com
# força). As demais (knuckle/follower/link/chassis) seguem cinemáticas.
_MIMIC_FORCE_JOINT_RE = re.compile(
    r'^_(?:thumb|index|middle|ring|little)_(?:proximal|distal)_j01$')


def inject_mimic_joint_plugins(urdf_body: str, *,
                               max_effort: float = 1.0,
                               p: float = 40.0,
                               i: float = 0.0,
                               d: float = 0.2) -> str:
    """Emite um <plugin libgazebo_mimic_joint_plugin.so> por junta <mimic>.

    Para cada ``<joint name="X" type="revolute"> ... <mimic joint="D"
    multiplier="M" offset="O"/>``, anexa um bloco <gazebo><plugin>. As
    juntas do caminho de contato (``_MIMIC_FORCE_JOINT_RE``) ganham
    ``<hasPID>`` com ganhos ``p/i/d`` e teto ``max_effort`` (N·m) — é o
    que faz a preensão transmitir força. Idempotente por ``name=``.
    """
    joint_re = re.compile(
        r'<joint\s+name="([^"]+)"\s+type="revolute">((?:(?!</joint>).)*?)'
        r'<mimic\s+joint="([^"]+)"\s+multiplier="([-\d.eE+]+)"'
        r'(?:\s+offset="([-\d.eE+]+)")?',
        re.DOTALL)

    blocks: list[str] = []
    for m in joint_re.finditer(urdf_body):
        jname, driver = m.group(1), m.group(3)
        mult = m.group(4)
        offset = m.group(5) or '0.0'
        if f'name="mimic_{jname}"' in urdf_body:
            continue                       # idempotência
        pid = ''
        if _MIMIC_FORCE_JOINT_RE.match(jname):
            pid = (f'\n      <hasPID><p>{p}</p><i>{i}</i><d>{d}</d></hasPID>'
                   f'\n      <maxEffort>{max_effort}</maxEffort>')
        blocks.append(
            f'  <gazebo>\n'
            f'    <plugin name="mimic_{jname}" '
            f'filename="libgazebo_mimic_joint_plugin.so">\n'
            f'      <joint>{driver}</joint>\n'
            f'      <mimicJoint>{jname}</mimicJoint>\n'
            f'      <multiplier>{mult}</multiplier>\n'
            f'      <offset>{offset}</offset>\n'
            f'      <sensitiveness>0.0</sensitiveness>{pid}\n'
            f'    </plugin>\n'
            f'  </gazebo>')

    if not blocks:
        return urdf_body
    block = '\n' + '\n'.join(blocks) + '\n'
    if '</robot>' in urdf_body:
        return urdf_body.replace('</robot>', block + '</robot>', 1)
    return urdf_body + block


def strip_mimic_joints_from_ros2_control(urdf_body: str) -> str:
    """Remove as juntas mimic (``_..._j01``) do bloco <ros2_control>.

    Com o plugin mimic acima no comando, o gazebo_ros2_control NÃO pode
    também impor essas juntas (SetPosition cinemático) — as duas malhas
    brigariam. Os 6 drivers permanecem.
    """
    return re.sub(
        r'\s*<joint name="_[^"]+_j01">(?:(?!</joint>).)*?</joint>',
        '', urdf_body, flags=re.DOTALL)


def apply_all(urdf_body: str, *,
              skin_inflate_m: float = 0.002,
              visual_skin_scale: float = _SKIN_VISUAL_SCALE,
              visual_skin_color: str = _SKIN_VISUAL_COLOR) -> str:
    """Aplica a pipeline padrão de pós-processamento na ordem correta."""
    urdf_body = fix_virtual_link_inertia(urdf_body)
    urdf_body = clamp_hand_joint_limits(urdf_body)
    urdf_body = stabilize_hand_joints(urdf_body)
    urdf_body = inject_skin_layer(urdf_body, inflate_m=skin_inflate_m)
    urdf_body = inject_visual_skin_layer(urdf_body,
                                          scale_factor=visual_skin_scale,
                                          color_rgba=visual_skin_color)
    urdf_body = enable_inter_finger_self_collision(urdf_body)
    return urdf_body
