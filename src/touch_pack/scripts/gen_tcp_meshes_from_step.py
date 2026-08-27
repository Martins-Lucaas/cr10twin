#!/usr/bin/env python3
"""Gera os meshes STL do TCP de palpação a partir do CAD em `cad/step/`.

A pilha vem de UMA montagem em vez de STEPs soltos: os sólidos já estão
posicionados uns em relação aos outros, então as alturas saem do arquivo em vez
de números digitados aqui — que era a fonte de erro toda vez que uma peça mudava
de espessura.

DUAS pilhas, uma por célula de força (`--stack`):

    load_cell  célula axial de 100 kg (viga S) — a montada na bancada.
               MONTAGEM_COMPLETA.step, TCP a 162,2 mm.
    ft6        célula FA7155 de 6 eixos. MONTAGEM_FA7155_stack.step, TCP a
               67,7 mm.

Trocar a célula troca a FERRAMENTA inteira, não só o driver: massa, CoM e
altura do TCP mudam, e com eles a compensação de gravidade e todo arquivo de
pose já ensinado. Rodar isto é o passo que produz os números para o URDF.
"""
from __future__ import annotations

import argparse
import os
import sys

from OCP.BRep import BRep_Builder
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.gp import gp_Trsf, gp_Vec

# ── Densidades (kg/m³) ─────────────────────────────────────────────────────
RHO_PRINTED = 950.0
RHO_ALU     = 2700.0
# Aço-liga da viga S de 100 kg. Aqui a geometria SABE mais que o catálogo: o
# STEP traz os rasgos em S (60,5 cm³ contra 74,0 cm³ do envelope), então
# integrar a densidade dá um número melhor que o "≈0,5 kg" da folha.
RHO_STEEL   = 7850.0

# Massa de catálogo do FA7155 (manual §3.1: ≤ 0,3 kg). O STEP do fabricante é
# um envelope MACIÇO — a 2700 kg/m³ daria 339 g para uma peça que tem cavidade
# interna, eletrônica e furação. Onde o catálogo sabe mais que a geometria, o
# catálogo manda.
FA7155_MASS_KG = 0.300

# ── As pilhas ──────────────────────────────────────────────────────────────
# Sólidos da montagem em ordem de Z crescente (a ordem que `read_solids`
# devolve) → (link, nome do STL, densidade, massa fixa em kg ou None para usar
# a densidade, quantos SÓLIDOS a entrada consome).
#
# `link=None` descarta a entrada: a montagem completa traz o punho do robô como
# referência de montagem, e ele não é da ferramenta — nem entra na massa nem
# define a base da pilha.
#
# n > 1 funde os sólidos num COMPOUND: um mesh só e uma inércia só. É o caso do
# laminado tátil, que são três camadas coladas (flex / piezorresistiva / flex)
# com 0,5 mm somados — separá-las em três links seria mentir sobre um sanduíche
# que não tem grau de liberdade nenhum.
STACKS = {
    'load_cell': ('MONTAGEM_COMPLETA.step', [
        (None,            None,                        None,        None, 1),
        ('coupler_robot', 'acoplador_robo.stl',        RHO_PRINTED, None, 1),
        ('load_cell',     'celula_carga_100kg.stl',    RHO_STEEL,   None, 1),
        ('coupler_tool',  'acoplador_tool.stl',        RHO_PRINTED, None, 1),
        ('touch_tool',    'touch_tool.stl',            RHO_PRINTED, None, 1),
        ('tool_tip',      'ponteira_D_sensor_5x5.stl', RHO_PRINTED, None, 1),
        ('tool_tip',      'sensor_5x5_pad.stl',        RHO_PRINTED, None, 3),
    ]),
    'ft6': ('MONTAGEM_FA7155_stack.step', [
        ('ft_flange',    'fa7155_flange_fixo.stl',          RHO_ALU,     None,           1),
        ('ft_sensor',    'fa7155_sensor.stl',               RHO_ALU,     FA7155_MASS_KG, 1),
        ('coupler_tool', 'acoplador_celula_hotswap.stl',    RHO_PRINTED, None,           1),
        ('tool_tip',     'ponteira_F_retangular_15x17.stl', RHO_PRINTED, None,           1),
    ]),
}

# PLANO DE MONTAGEM: a face INFERIOR do flange fixo do FA7155 é a face do
# flange do CR10 — o flange fixo parafusa direto no punho (manual §2.1, quatro
# M6 + pino Φ6) e o antigo `acoplador_robo` saiu da pilha. O STEP herdou o Z do
# assembly em que foi modelado (a base cai em Z = +95 mm), então tudo é
# normalizado pelo MENOR Z da montagem em vez de digitado: se o CAD for
# reexportado com outra origem, os números continuam certos.


def read_solids(path: str) -> list:
    reader = STEPControl_Reader()
    if reader.ReadFile(path) != 1:
        raise RuntimeError(f'não consegui ler {path}')
    reader.TransferRoots()
    shape = reader.OneShape()
    out = []
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        out.append(TopoDS.Solid_s(exp.Current()))
        exp.Next()
    # Ordem estável por Z mínimo — a tabela STACK_SOLIDS depende dela.
    def zmin(s):
        b = Bnd_Box()
        BRepBndLib.Add_s(s, b, False)
        return b.Get()[2]
    return sorted(out, key=zmin)


def compound(shapes) -> TopoDS_Compound:
    comp = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(comp)
    for s in shapes:
        builder.Add(comp, s)
    return comp


def translate(shape, dz: float):
    trsf = gp_Trsf()
    trsf.SetTranslation(gp_Vec(0.0, 0.0, dz))
    return BRepBuilderAPI_Transform(shape, trsf, True).Shape()


def bbox(shape):
    b = Bnd_Box()
    BRepBndLib.Add_s(shape, b, False)
    return b.Get()


def mass_props(shape, rho: float):
    """(massa kg, CoM m, tensor 3×3 kg·m² em torno do CoM) — mesh em mm."""
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    vol_mm3 = props.Mass()
    com = props.CentreOfMass()
    com_m = (com.X() * 1e-3, com.Y() * 1e-3, com.Z() * 1e-3)
    mass = vol_mm3 * 1e-9 * rho

    # Inércia em torno do CoM: reintegra com o CoM como ponto de referência.
    at_com = GProp_GProps(com)
    BRepGProp.VolumeProperties_s(shape, at_com)
    m = at_com.MatrixOfInertia()
    # OCC devolve o tensor da geometria (densidade 1, mm⁵ efetivos):
    # multiplicar por rho·1e-15 leva de mm²·mm³ para kg·m².
    k = rho * 1e-15
    tensor = [[m.Value(i, j) * k for j in (1, 2, 3)] for i in (1, 2, 3)]
    return mass, com_m, tensor, vol_mm3


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--stack', choices=sorted(STACKS), default='load_cell',
                    help='qual pilha gerar (default: load_cell, a montada '
                         'na bancada)')
    ap.add_argument('--deflection', type=float, default=0.15,
                    help='desvio linear da tesselagem em mm (default 0.15)')
    args = ap.parse_args()
    stack_step, stack_solids = STACKS[args.stack]

    # O console do Windows abre em cp1252 e engasga nos acentos do relatório.
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except (AttributeError, ValueError):
        pass

    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.normpath(os.path.join(here, '..', '..', '..'))
    step_dir = os.path.join(repo, 'cad', 'step')
    mesh_dir = os.path.normpath(os.path.join(here, '..', 'meshes'))

    if not os.path.isdir(step_dir):
        print(f'ERRO: {step_dir} não existe', file=sys.stderr)
        return 1
    os.makedirs(mesh_dir, exist_ok=True)

    solids = read_solids(os.path.join(step_dir, stack_step))
    esperados = sum(e[4] for e in stack_solids)
    if len(solids) != esperados:
        print(f'ERRO: {stack_step} tem {len(solids)} sólidos, mas a tabela de '
              f'{args.stack} descreve {esperados}. O CAD mudou — atualize a '
              f'tabela.', file=sys.stderr)
        return 1

    # Cada entrada consome n sólidos consecutivos; n > 1 vira um compound.
    pecas, i = [], 0
    for link, stl_name, rho, mass_fix, n in stack_solids:
        grupo = solids[i:i + n]
        i += n
        if link is None:
            continue        # peça de referência, não é da ferramenta
        shape = grupo[0] if n == 1 else compound(grupo)
        pecas.append((link, stl_name, rho, mass_fix, shape))

    # Origem da pilha = menor Z das peças DA FERRAMENTA (a face que parafusa no
    # flange do CR10) e, com ela, a altura de cada link e o TCP. Nada disto é
    # digitado à mão — descartar o punho antes desta conta é o que impede a
    # base cair 30 mm abaixo do flange.
    z_base = min(bbox(sh)[2] for *_, sh in pecas)
    z_link = {}
    for link, _stl, _rho, _mf, shape in pecas:
        z = bbox(shape)[2] - z_base
        # Um link com várias peças (ponteira + laminado) fica na MENOR delas:
        # é a base do link, não a de cada sólido.
        z_link[link] = min(z_link.get(link, z), z)
    z_tcp_mm = max(bbox(sh)[5] for *_, sh in pecas) - z_base

    # link → lista de (massa, CoM, tensor no CoM).
    by_link: dict[str, list] = {}
    print('# Meshes gerados e blocos <inertial> da geometria real '
          '(frame de cada link, metros/kg)')
    print(f'# Pilha {args.stack} — montagem: {stack_step}, base em '
          f'Z={z_base:.2f} mm no arquivo, normalizada para 0 (face do flange '
          'do CR10)')
    print()

    for link, stl_name, rho, mass_fix, solid in pecas:
        shape = translate(solid, -z_base - z_link[link])

        BRepMesh_IncrementalMesh(shape, args.deflection, False, 0.35, True)
        writer = StlAPI_Writer()
        writer.ASCIIMode = False          # STL binário (mesh ~10× menor)
        out = os.path.join(mesh_dir, stl_name)
        if not writer.Write(shape, out):
            print(f'ERRO ao gravar {out}', file=sys.stderr)
            return 1

        mass, com, tensor, vol = mass_props(shape, rho)
        if mass_fix is not None:
            # Massa de catálogo: o tensor foi integrado com a densidade errada
            # e é LINEAR na massa, então reescala junto.
            k = mass_fix / mass
            tensor = [[v * k for v in row] for row in tensor]
            mass = mass_fix
        x0, y0, z0, x1, y1, z1 = bbox(shape)
        by_link.setdefault(link, []).append((mass, com, tensor))

        origem = ('massa de catálogo' if mass_fix is not None
                  else f'rho={rho:.0f} kg/m3')
        size = os.path.getsize(out)
        print(f'# {stl_name}   (link {link}_link, '
              f'z_link={z_link[link]:.1f} mm, {origem})')
        print(f'#   bbox mm: X[{x0:.2f},{x1:.2f}] Y[{y0:.2f},{y1:.2f}] '
              f'Z[{z0:.2f},{z1:.2f}]   vol={vol/1000:.2f} cm3   '
              f'massa={mass*1000:.1f} g   STL {size/1024:.0f} KiB')

    print()
    for link in z_link:
        mass, com, tensor = _combine(by_link[link])
        print(f'  <link name="{link}_link">   <!-- z_link = '
              f'{z_link[link]:.1f} mm do flange -->')
        print('    <inertial>')
        print(f'      <origin xyz="{com[0]:.5f} {com[1]:.5f} {com[2]:.5f}" '
              f'rpy="0 0 0"/>')
        print(f'      <mass value="{mass:.4f}"/>')
        print(f'      <inertia ixx="{tensor[0][0]:.3e}" '
              f'ixy="{tensor[0][1]:.3e}" ixz="{tensor[0][2]:.3e}"')
        print(f'               iyy="{tensor[1][1]:.3e}" '
              f'iyz="{tensor[1][2]:.3e}" izz="{tensor[2][2]:.3e}"/>')
        print('    </inertial>')

    parts = [(m, (c[0], c[1], c[2] + z_link[lk] * 1e-3), t)
             for lk in z_link for (m, c, t) in by_link[lk]]
    m_tot, com_tot, _ = _combine(parts)
    print()
    print(f'# TOTAL do TCP: massa {m_tot:.4f} kg, CoM no frame do flange '
          f'= ({com_tot[0]*1000:.1f}, {com_tot[1]*1000:.1f}, '
          f'{com_tot[2]*1000:.1f}) mm')
    print(f'#   CoM visto do tcp_link (Z_TCP = {z_tcp_mm:.1f} mm): '
          f'z = {com_tot[2] - z_tcp_mm*1e-3:+.4f} m   '
          f'|  peso = {m_tot * 9.80665:.2f} N')
    print()
    print(f'# tcp_link em Z = {z_tcp_mm:.2f} mm do flange — espelhe em '
          'urdf/touch_tool_tcp.urdf e em kinematics.T_TOUCH_TOOL_ATTACH.')
    return 0


def _combine(items):
    """Soma corpos: (massa, CoM, tensor no CoM) → conjunto equivalente."""
    m_tot = sum(m for m, _, _ in items)
    com = tuple(sum(m * c[k] for m, c, _ in items) / m_tot for k in range(3))
    tensor = [[0.0] * 3 for _ in range(3)]
    for m, c, t in items:
        d = [c[k] - com[k] for k in range(3)]
        d2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2]
        for i in range(3):
            for j in range(3):
                delta = m * ((d2 if i == j else 0.0) - d[i] * d[j])
                tensor[i][j] += t[i][j] + delta
    return m_tot, com, tensor


if __name__ == '__main__':
    raise SystemExit(main())
