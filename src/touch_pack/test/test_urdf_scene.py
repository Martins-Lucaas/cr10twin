"""Testes do touch_pack.urdf_scene — parser de URDF, malhas e FK da árvore."""
import math
import struct

import numpy as np
import pytest

from touch_pack import urdf_scene as us


def _write_binary_stl(path, tris):
    with open(path, 'wb') as fh:
        fh.write(b'\0' * 80)
        fh.write(struct.pack('<I', len(tris)))
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0])
            fh.write(struct.pack('<3f', *n.astype('<f4')))
            for v in t:
                fh.write(struct.pack('<3f', *np.asarray(v, dtype='<f4')))
            fh.write(b'\0\0')


_CUBE = us.tessellate_box((1.0, 1.0, 1.0))


# ── Leitura de malhas ──────────────────────────────────────────────────

def test_load_binary_stl_roundtrip(tmp_path):
    p = tmp_path / 'cube.stl'
    _write_binary_stl(p, _CUBE)
    got = us.load_stl(str(p))
    assert got.shape == _CUBE.shape
    assert np.allclose(np.sort(got.reshape(-1, 3), axis=0),
                       np.sort(_CUBE.reshape(-1, 3), axis=0), atol=1e-6)


def test_load_binary_stl_with_solid_header(tmp_path):
    """Exportadores que escrevem 'solid' no header de um STL BINÁRIO são
    comuns; detectar pelo tamanho do arquivo é o que evita ler zero
    triângulo e desenhar um robô invisível."""
    p = tmp_path / 'tricky.stl'
    _write_binary_stl(p, _CUBE)
    raw = bytearray(p.read_bytes())
    raw[0:5] = b'solid'
    p.write_bytes(bytes(raw))
    assert us.load_stl(str(p)).shape[0] == _CUBE.shape[0]


def test_load_ascii_stl(tmp_path):
    p = tmp_path / 'ascii.stl'
    body = ['solid test']
    for t in _CUBE:
        body.append('facet normal 0 0 0\n outer loop')
        for v in t:
            body.append(f'  vertex {v[0]} {v[1]} {v[2]}')
        body.append(' endloop\nendfacet')
    body.append('endsolid test')
    p.write_text('\n'.join(body))
    assert us.load_stl(str(p)).shape == _CUBE.shape


def test_load_stl_rejects_garbage(tmp_path):
    p = tmp_path / 'bad.stl'
    p.write_bytes(b'not an stl at all')
    with pytest.raises(us.UrdfSceneError):
        us.load_stl(str(p))


# ── Primitivas ─────────────────────────────────────────────────────────

def test_box_is_closed_and_correctly_sized():
    tris = us.tessellate_box((0.2, 0.4, 0.6))
    pts = tris.reshape(-1, 3)
    assert np.allclose(pts.max(axis=0), [0.1, 0.2, 0.3])
    assert np.allclose(pts.min(axis=0), [-0.1, -0.2, -0.3])
    # Volume pelo teorema da divergência: uma malha fechada fecha a conta.
    vol = np.abs(np.einsum('ij,ij->i',
                           tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum()) / 6
    assert math.isclose(vol, 0.2 * 0.4 * 0.6, rel_tol=1e-9)


def test_cylinder_dimensions():
    tris = us.tessellate_cylinder(0.05, 0.2, segments=24)
    pts = tris.reshape(-1, 3)
    assert math.isclose(float(pts[:, 2].max()), 0.1, abs_tol=1e-12)
    assert math.isclose(float(pts[:, 2].min()), -0.1, abs_tol=1e-12)
    r = np.hypot(pts[:, 0], pts[:, 1])
    assert r.max() <= 0.05 + 1e-12


def test_sphere_points_lie_on_the_radius():
    tris = us.tessellate_sphere(0.03, rings=6, segments=10)
    r = np.linalg.norm(tris.reshape(-1, 3), axis=1)
    assert np.all(r <= 0.03 + 1e-12)
    assert r.max() > 0.029


# ── Decimação ──────────────────────────────────────────────────────────

def _sphere_mesh(n_rings=40, n_seg=60):
    return us.tessellate_sphere(1.0, rings=n_rings, segments=n_seg)


def test_decimate_respects_the_budget():
    mesh = _sphere_mesh()
    out = us.decimate(mesh, 400)
    assert out.shape[0] <= 400
    assert out.shape[0] > 40


def test_decimate_keeps_the_silhouette():
    """A malha reduzida tem de continuar ocupando o mesmo volume — é a
    silhueta que a viewport mostra."""
    mesh = _sphere_mesh()
    out = us.decimate(mesh, 500)
    r = np.linalg.norm(out.reshape(-1, 3), axis=1)
    assert 0.9 < float(r.max()) <= 1.0 + 1e-9
    assert float(r.mean()) > 0.85


def test_decimate_emits_no_degenerate_triangles():
    """Triângulos de área zero seriam invisíveis e só custariam tempo."""
    out = us.decimate(_sphere_mesh(), 600)
    area = 0.5 * np.linalg.norm(
        np.cross(out[:, 1] - out[:, 0], out[:, 2] - out[:, 0]), axis=1)
    assert np.all(area > 0.0)


def test_decimate_shrinks_small_meshes_too():
    """Peças pequenas (as falanges da COVVI) também têm de encolher: a busca
    de resolução precisa DESCER, não só subir. Sem isso a malha de arrasto
    ficava do mesmo tamanho da cheia e o quadro estourava o tick."""
    mesh = us.tessellate_cylinder(0.01, 0.03, segments=16)   # 64 triângulos
    out = us.decimate(mesh, 12)
    assert out.shape[0] < mesh.shape[0]
    assert out.shape[0] > 0


def test_decimate_is_a_no_op_below_budget():
    mesh = us.tessellate_box((1, 1, 1))
    out = us.decimate(mesh, 1000)
    assert out.shape == mesh.shape
    assert np.allclose(out, mesh)


# ── Parser + FK da árvore ──────────────────────────────────────────────

def _two_link_urdf(mesh_path):
    return f'''<?xml version="1.0"?>
<robot name="t">
  <material name="red"><color rgba="1 0 0 1"/></material>
  <link name="world"/>
  <joint name="fix" type="fixed">
    <parent link="world"/><child link="base_link"/>
    <origin xyz="0 0 0.03" rpy="0 0 0"/>
  </joint>
  <link name="base_link">
    <visual><origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.1 0.1 0.1"/></geometry>
      <material name="red"/>
    </visual>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base_link"/><child link="l1"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <link name="l1">
    <visual><origin xyz="0.5 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="file://{mesh_path}" scale="0.001 0.001 0.001"/></geometry>
      <material name="blue"><color rgba="0 0 1 1"/></material>
    </visual>
  </link>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <mimic joint="j1" multiplier="2.0" offset="0.1"/>
  </joint>
  <link name="l2"/>
</robot>'''


@pytest.fixture
def scene(tmp_path):
    mesh = tmp_path / 'm.stl'
    _write_binary_stl(mesh, _CUBE * 1000.0)
    return us.parse_urdf(_two_link_urdf(str(mesh)))


def test_parse_reads_links_joints_and_materials(scene):
    assert scene.root == 'world'
    assert {j.name for j in scene.joints} == {'fix', 'j1', 'j2'}
    colors = {p.link: p.color for p in scene.parts}
    assert colors['base_link'] == (255, 0, 0)     # material nomeado no topo
    assert colors['l1'] == (0, 0, 255)            # cor inline


def test_mesh_scale_and_visual_origin_are_applied(scene):
    part = next(p for p in scene.parts if p.link == 'l1')
    pts = part.tris.reshape(-1, 3)
    # cubo de 1000 mm × 0.001 = 1 m de lado, centrado em x=+0.5
    assert np.allclose(pts.min(axis=0), [0.0, -0.5, -0.5], atol=1e-5)
    assert np.allclose(pts.max(axis=0), [1.0, 0.5, 0.5], atol=1e-5)


def test_fk_is_expressed_in_the_base_link_frame(scene):
    """O URDF da célula pendura base_link 30 mm acima do `world`, mas toda a
    cinemática do pacote vive no base_link — renderizar na raiz deslocaria
    o robô inteiro em relação à alça do TCP."""
    Ts = scene.link_transforms({})
    assert np.allclose(Ts['base_link'][:3, 3], [0, 0, 0])
    assert np.allclose(Ts['world'][:3, 3], [0, 0, -0.03])
    assert np.allclose(Ts['l1'][:3, 3], [0, 0, 0.2])


def test_fk_rotates_revolute_joints(scene):
    Ts = scene.link_transforms({'j1': math.pi / 2})
    R = Ts['l1'][:3, :3]
    assert np.allclose(R @ np.array([1.0, 0, 0]), [0, 1, 0], atol=1e-12)


def test_fk_resolves_mimic_joints(scene):
    """A mão COVVI move 25 elos secundários por <mimic>; sem isso os dedos
    da viewport ficariam rígidos."""
    Ts = scene.link_transforms({'j1': 0.3})
    ang = 2.0 * 0.3 + 0.1
    expected = us.axis_angle_to_matrix(np.array([0, 0, 1.0]), 0.3 + ang)
    assert np.allclose(Ts['l2'][:3, :3], expected, atol=1e-12)


def test_fk_ignores_unknown_joint_names(scene):
    Ts = scene.link_transforms({'nao_existe': 1.0})
    assert np.allclose(Ts['l1'][:3, 3], [0, 0, 0.2])


def test_missing_mesh_is_reported_not_fatal(tmp_path):
    sc = us.parse_urdf(_two_link_urdf(str(tmp_path / 'nao_existe.stl')))
    assert sc.missing_meshes                       # registrado
    assert any(p.link == 'base_link' for p in sc.parts)   # o resto carregou


def test_urdf_without_visuals_raises():
    with pytest.raises(us.UrdfSceneError):
        us.parse_urdf('<robot name="x"><link name="a"/></robot>')


def test_glove_layer_replaces_the_inner_visual(tmp_path):
    """Duas visuais quase coincidentes no mesmo link (mecanismo + luva de
    silicone da COVVI) manchariam a peça sem z-buffer — só a luva vale."""
    mesh = tmp_path / 'm.stl'
    _write_binary_stl(mesh, _CUBE * 1000.0)
    xml = f'''<robot name="h">
      <link name="base_link">
        <visual><geometry><box size="0.1 0.1 0.1"/></geometry>
          <material name="a"><color rgba="1 0 0 1"/></material></visual>
        <visual name="covvi_glove">
          <geometry><mesh filename="file://{mesh}" scale="0.001 0.001 0.001"/></geometry>
          <material name="b"><color rgba="0.12 0.12 0.13 1"/></material></visual>
      </link>
    </robot>'''
    sc = us.parse_urdf(xml)
    assert len(sc.parts) == 1
    assert sc.parts[0].color == (30, 30, 33)


# ── Resolução de caminhos ──────────────────────────────────────────────

def test_resolve_uri_file_scheme(tmp_path):
    f = tmp_path / 'a.stl'
    f.write_bytes(b'x')
    assert us.resolve_uri(f'file://{f}') == str(f)
    assert us.resolve_uri(f'file://{tmp_path}/nope.stl') is None


def test_resolve_uri_package_scheme_via_search_dir(tmp_path):
    pkg = tmp_path / 'meu_pacote' / 'meshes'
    pkg.mkdir(parents=True)
    (pkg / 'a.stl').write_bytes(b'x')
    got = us.resolve_uri('package://meu_pacote/meshes/a.stl',
                         search_dirs=(str(tmp_path),))
    assert got == str(pkg / 'a.stl')


def test_resolve_uri_empty_is_none():
    assert us.resolve_uri('') is None
    assert us.resolve_uri(None) is None


# ── URDF real da célula (precisa do workspace construído) ──────────────

@pytest.mark.parametrize('end_effector', ['touch_tool', 'hand'])
def test_real_cell_scene_builds(end_effector):
    try:
        scene = us.build_scene(end_effector)
    except Exception as exc:                       # xacro/ament ausentes
        pytest.skip(f'URDF da célula indisponível: {exc}')
    assert scene.triangle_count > 500
    Ts = scene.link_transforms({})
    # A cadeia do braço tem de existir e nascer no base_link.
    for link in ('base_link', 'Link1', 'Link6'):
        assert link in Ts
    assert np.allclose(Ts['base_link'][:3, 3], [0, 0, 0], atol=1e-12)
    if end_effector == 'hand':
        assert 'hand_base_link' in Ts
    else:
        # A pilha da FA7155 não tem mais o link touch_tool; a ponteira é o
        # único elo que sobrevive a qualquer troca de ferramenta.
        assert 'tool_tip_link' in Ts


def test_real_cell_scene_matches_kinematics_fk():
    """O Link6 desenhado tem de cair onde a FK do pacote diz — é a garantia
    de que a alça do TCP pousa sobre a ferramenta renderizada, e não a
    30 mm dela (o offset `world → base_link` do URDF).
    """
    try:
        scene = us.build_scene('touch_tool')
    except Exception as exc:
        pytest.skip(f'URDF da célula indisponível: {exc}')
    from touch_pack.kinematics import fk_partial

    q = np.deg2rad([15.0, -30.0, -90.0, 10.0, 60.0, -20.0])
    Ts = scene.link_transforms({f'joint{i + 1}': float(q[i]) for i in range(6)})
    assert np.allclose(Ts['Link6'][:3, 3], fk_partial(q, 6)[:3, 3], atol=5e-5)
    assert np.allclose(Ts['Link6'][:3, :3], fk_partial(q, 6)[:3, :3], atol=5e-5)


# ── O TRIO DA FERRAMENTA: URDF ↔ kinematics ↔ controllers.yaml ────────
# O cabeçalho do `urdf/touch_tool_tcp.urdf` diz "Mexeu aqui, mexe nos três".
# Nada cobrava isso na primeira troca de célula (axial de 100 kg → FA7155,
# 18/08/2026): os três foram atualizados à mão e a suíte inteira teria passado
# se um deles tivesse ficado para trás. Na volta para a axial (162,2 mm,
# 0,6034 kg) este teste já existia e é ele que garante que os três batem.

_G = 9.80665


def _tool_urdf_path():
    """O URDF legível da ferramenta — o mesmo alvo de `check_urdf`."""
    import os
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('touch_pack'),
                         'urdf', 'touch_tool_tcp.urdf')
        if os.path.exists(p):
            return p
    except Exception:
        pass
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     '..', 'urdf', 'touch_tool_tcp.urdf')
    return os.path.normpath(p)


def _controllers_yaml_path():
    import os
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('touch_pack'),
                         'config', 'tactile_controllers.yaml')
        if os.path.exists(p):
            return p
    except Exception:
        pass
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     '..', 'config', 'tactile_controllers.yaml')
    return os.path.normpath(p)


def _tool_chain():
    """Percorre Link6 → tcp_link no URDF da ferramenta.

    Devolve (z_tcp, massa_total, com_z) em metros/kg, tudo no frame do Link6.
    Só translação: a pilha é axial, e que ela CONTINUE axial faz parte do
    contrato (`T_TOUCH_TOOL_ATTACH` tem rotação identidade).
    """
    import xml.etree.ElementTree as ET
    root = ET.parse(_tool_urdf_path()).getroot()

    filhos = {}      # parent -> (child, xyz)
    for j in root.findall('joint'):
        if j.get('type') != 'fixed':
            continue
        pai = j.find('parent').get('link')
        filho = j.find('child').get('link')
        o = j.find('origin')
        xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None
                                  else '0 0 0').split()]
        rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None
                                  else '0 0 0').split()]
        assert all(abs(r) < 1e-12 for r in rpy), (
            f'junta {j.get("name")} girou a pilha (rpy={rpy}) — '
            'T_TOUCH_TOOL_ATTACH pressupõe rotação identidade')
        filhos.setdefault(pai, []).append((filho, xyz))

    inerciais = {}   # link -> (massa, com_z)
    for lk in root.findall('link'):
        ine = lk.find('inertial')
        if ine is None:
            continue
        m = float(ine.find('mass').get('value'))
        o = ine.find('origin')
        com = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None
                                  else '0 0 0').split()]
        inerciais[lk.get('name')] = (m, com[2])

    z_tcp = None
    massa, momento = 0.0, 0.0

    def anda(link, z):
        nonlocal z_tcp, massa, momento
        if link == 'tcp_link':
            z_tcp = z
        # O Link6 é o flange do robô (placeholder de 1 g aqui), não peça da
        # ferramenta: entra como origem da cadeia e fica fora da massa.
        elif link != 'Link6' and link in inerciais:
            m, com_z = inerciais[link]
            massa += m
            momento += m * (z + com_z)
        for filho, xyz in filhos.get(link, []):
            assert abs(xyz[0]) < 1e-12 and abs(xyz[1]) < 1e-12, (
                f'{link} → {filho} saiu do eixo (xyz={xyz}) — a pilha deixou '
                'de ser axial e T_TOUCH_TOOL_ATTACH não a descreve mais')
            anda(filho, z + xyz[2])

    anda('Link6', 0.0)
    assert z_tcp is not None, 'a cadeia Link6 → tcp_link sumiu do URDF'
    return z_tcp, massa, momento / massa


def test_tool_urdf_bate_com_T_TOUCH_TOOL_ATTACH():
    """O +Z do `tcp_link` no URDF é o mesmo de `T_TOUCH_TOOL_ATTACH`."""
    from touch_pack.kinematics import T_TOUCH_TOOL_ATTACH as ATT

    z_tcp, _massa, _com_z = _tool_chain()
    assert np.allclose(ATT[:3, :3], np.eye(3), atol=1e-12), (
        'T_TOUCH_TOOL_ATTACH ganhou rotação e o URDF não')
    assert ATT[0, 3] == 0.0 and ATT[1, 3] == 0.0
    assert ATT[2, 3] == pytest.approx(z_tcp, abs=1e-5), (
        f'kinematics diz {ATT[2, 3]*1e3:.1f} mm e o URDF soma '
        f'{z_tcp*1e3:.1f} mm do flange até o tcp_link')


def test_gravity_compensation_bate_com_a_pilha_do_urdf():
    """CoG e força da compensação de gravidade saem da MESMA pilha.

    `pos` é o centroide expresso no frame do `tcp_link` (por isso negativo) e
    `force` é m·g do conjunto inteiro. Ambos foram calculados de
    `scripts/gen_tcp_meshes_from_step.py` e copiados à mão para o YAML.
    """
    import yaml

    z_tcp, massa, com_z = _tool_chain()
    with open(_controllers_yaml_path()) as fh:
        cfg = yaml.safe_load(fh)

    grav = None
    for chave in cfg:
        ros = cfg[chave].get('ros__parameters', {}) if isinstance(
            cfg[chave], dict) else {}
        if 'gravity_compensation' in ros:
            grav = ros['gravity_compensation']
            break
    assert grav is not None, (
        'nenhum controlador declara gravity_compensation no YAML')

    assert grav['frame']['id'] == 'tcp_link', (
        "o CoG abaixo está expresso no frame do tcp_link; mudar o frame sem "
        "recalcular o vetor põe o centroide no lugar errado")

    pos = [float(v) for v in grav['CoG']['pos']]
    esperado_z = com_z - z_tcp
    assert pos[0] == pytest.approx(0.0, abs=1e-4)
    assert pos[1] == pytest.approx(0.0, abs=1e-4)
    assert pos[2] == pytest.approx(esperado_z, abs=2e-4), (
        f'YAML diz {pos[2]*1e3:.1f} mm e a pilha do URDF dá '
        f'{esperado_z*1e3:.1f} mm (CoM em {com_z*1e3:.1f} mm do flange, '
        f'tcp_link em {z_tcp*1e3:.1f} mm)')

    assert float(grav['CoG']['force']) == pytest.approx(massa * _G, abs=0.01), (
        f'YAML diz {float(grav["CoG"]["force"]):.2f} N e as massas do URDF '
        f'somam {massa:.4f} kg = {massa * _G:.2f} N')


def test_scene_tcp_link_bate_com_a_fk_do_pacote():
    """Fecha o trio pelo lado da cena: o `tcp_link` DESENHADO cai onde a FK
    com `T_TOUCH_TOOL_ATTACH` diz. Pega o caso em que o URDF injetado pelo
    launch diverge do URDF standalone que os testes acima leem."""
    try:
        scene = us.build_scene('touch_tool')
    except Exception as exc:
        pytest.skip(f'URDF da célula indisponível: {exc}')
    from touch_pack.kinematics import (forward_kinematics,
                                       T_TOUCH_TOOL_ATTACH)

    q = np.deg2rad([15.0, -30.0, -90.0, 10.0, 60.0, -20.0])
    Ts = scene.link_transforms({f'joint{i + 1}': float(q[i]) for i in range(6)})
    assert 'tcp_link' in Ts, 'o launch não injetou o tcp_link no touch_tool'
    esperado = forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)
    assert np.allclose(Ts['tcp_link'][:3, 3], esperado[:3, 3], atol=5e-5), (
        f'tcp_link desenhado em {Ts["tcp_link"][:3, 3].round(4)} contra '
        f'{esperado[:3, 3].round(4)} da FK')
