#!/usr/bin/env python3
"""Gera os STL e os blocos <inertial> do mastro (cad/Joystick.step).

O STEP traz DOIS sólidos, em ordem de Z crescente:

    Corpo1  placa 110 × 50 × 2 mm (face que encosta na célula FA7155)
    Corpo2  empunhadura ⌀35 × 150 mm, oca (o que o voluntário segura)

Origem dos dois links = centro da face de baixo da placa e da empunhadura,
eixos do STEP. Nada de altura ou centro digitado à mão: sai do bbox.

Materiais (densidades, kg/m³):
    placa ........ 950   impressa (PLA, convenção do projeto)
    empunhadura .. 1100  silicone/borracha — decisão do operador, 23/11/2026

ATENÇÃO: o STEP está desatualizado (150 mm; a peça tem 135). Não copie dele a massa,
o CoM e a inércia da empunhadura: no URDF são a pesagem e um cilindro uniforme.

Uso:  python3 gen_joystick_meshes.py   (precisa do OCP / cadquery-ocp)
"""
from __future__ import annotations

import os
import sys

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
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Trsf, gp_Vec

RHO_PLATE = 950.0
RHO_GRIP = 1100.0


def read_solids(path):
    reader = STEPControl_Reader()
    if reader.ReadFile(path) != 1:
        raise SystemExit(f'ERRO: não consegui ler {path}')
    reader.TransferRoots()
    ex, out = TopExp_Explorer(reader.OneShape(), TopAbs_SOLID), []
    while ex.More():
        out.append(TopoDS.Solid_s(ex.Current()))
        ex.Next()
    return out


def bbox(shape):
    b = Bnd_Box()
    BRepBndLib.Add_s(shape, b, False)
    return b.Get()          # xmin ymin zmin xmax ymax zmax (mm)


def translate(shape, dx, dy, dz):
    t = gp_Trsf()
    t.SetTranslation(gp_Vec(dx, dy, dz))
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def mass_props(shape, rho):
    """(massa kg, CoM m, tensor 3×3 kg·m² no CoM) — geometria em mm."""
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, p)
    com = p.CentreOfMass()
    at_com = GProp_GProps(com)
    BRepGProp.VolumeProperties_s(shape, at_com)
    m = at_com.MatrixOfInertia()
    k = rho * 1e-15         # mm²·mm³ → kg·m²
    tensor = [[m.Value(i, j) * k for j in (1, 2, 3)] for i in (1, 2, 3)]
    return (p.Mass() * 1e-9 * rho,
            (com.X() * 1e-3, com.Y() * 1e-3, com.Z() * 1e-3), tensor)


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.normpath(os.path.join(here, '..', '..', '..'))
    step = os.path.join(repo, 'cad', 'Joystick.step')
    mesh_dir = os.path.normpath(os.path.join(here, '..', 'meshes'))

    solids = read_solids(step)
    if len(solids) != 2:
        print(f'ERRO: {step} tem {len(solids)} sólidos; esperava 2 (placa + '
              'empunhadura). O CAD mudou — atualize este script.',
              file=sys.stderr)
        return 1
    solids.sort(key=lambda s: bbox(s)[2])
    plate, grip = solids

    px0, py0, pz0, px1, py1, pz1 = bbox(plate)
    cx, cy = (px0 + px1) / 2, (py0 + py1) / 2
    gz0 = bbox(grip)[2]
    gx0, gy0, _, gx1, gy1, gz1 = bbox(grip)
    if abs((gx0 + gx1) / 2 - cx) > 0.05 or abs((gy0 + gy1) / 2 - cy) > 0.05:
        print('AVISO: a empunhadura não está centrada na placa.',
              file=sys.stderr)

    parts = [
        ('joystick_plate_link', 'joystick_placa.stl', RHO_PLATE,
         translate(plate, -cx, -cy, -pz0)),
        ('joystick_grip_link', 'joystick_empunhadura.stl', RHO_GRIP,
         translate(grip, -cx, -cy, -gz0)),
    ]
    print(f'# placa {px1 - px0:.1f} × {py1 - py0:.1f} × {pz1 - pz0:.1f} mm; '
          f'empunhadura ⌀{gx1 - gx0:.1f} × {gz1 - gz0:.1f} mm, base a '
          f'{gz0 - pz0:.1f} mm da face da placa')
    for link, stl, rho, shape in parts:
        BRepMesh_IncrementalMesh(shape, 0.15, False, 0.35, True)
        w = StlAPI_Writer()
        w.ASCIIMode = False
        if not w.Write(shape, os.path.join(mesh_dir, stl)):
            print(f'ERRO ao gravar {stl}', file=sys.stderr)
            return 1
        mass, com, t = mass_props(shape, rho)
        print(f'\n<!-- {link}: {stl}  rho={rho:.0f} kg/m3 -->')
        print(f'<origin xyz="{com[0]:.5f} {com[1]:.5f} {com[2]:.5f}" '
              'rpy="0 0 0"/>')
        print(f'<mass value="{mass:.4f}"/>')
        print(f'<inertia ixx="{t[0][0]:.3e}" ixy="{-t[0][1]:.3e}" '
              f'ixz="{-t[0][2]:.3e}" iyy="{t[1][1]:.3e}" '
              f'iyz="{-t[1][2]:.3e}" izz="{t[2][2]:.3e}"/>')
    return 0


if __name__ == '__main__':
    sys.exit(main())
