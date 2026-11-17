#!/usr/bin/env python3
"""FK do grasp_ml_pack == cadeia do cr10_robot.xacro (origens + axis reais).

Pega a regressão do joint1 axis="0 0 -1": sem o sinal, a FK espelhava a base
e a mão chegava ~0,5 m longe do objeto no Gazebo.
    python3 src/grasp_ml_pack/scripts/test_fk_urdf.py
"""
import math
import os
import re

import numpy as np
import xacro

from grasp_ml_pack.kinematics import T_HAND_ATTACH, forward_kinematics, inverse_kinematics

_XACRO = os.path.join(os.path.dirname(__file__), '..', '..', 'cra_description',
                      'urdf', 'cr10_robot.xacro')


def _rot(axis, th):
    a = np.asarray(axis, float)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def _rpy(r, p, y):
    return _rot((0, 0, 1), y) @ _rot((0, 1, 0), p) @ _rot((1, 0, 0), r)


def urdf_fk(urdf, q):
    T = np.eye(4)
    for i, qi in enumerate(q, 1):
        j = re.search(rf'<joint name="joint{i}".*?</joint>', urdf, re.S).group(0)
        o = re.search(r'<origin[^>]*>', j).group(0)
        xyz = [float(v) for v in re.search(r'xyz="([^"]+)"', o).group(1).split()]
        rpy = [float(v) for v in re.search(r'rpy="([^"]+)"', o).group(1).split()]
        ax = [float(v) for v in re.search(r'<axis xyz="([^"]+)"', j).group(1).split()]
        A = np.eye(4); A[:3, :3] = _rpy(*rpy); A[:3, 3] = xyz
        B = np.eye(4); B[:3, :3] = _rot(ax, qi)
        T = T @ A @ B
    return T @ T_HAND_ATTACH


def main():
    urdf = xacro.process_file(_XACRO).toxml()
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.uniform(-2.5, 2.5, 6)
        err = np.abs(forward_kinematics(q) - urdf_fk(urdf, q)).max()
        assert err < 1e-3, (q, err)
    # IK de volta no pick da esteira (robot frame) fecha na URDF real.
    p = np.array([0.75, 0.0, 0.851 + 0.07 - 0.405])
    q, ok = inverse_kinematics(p, np.array([0., 0., -1.]))
    assert ok and np.linalg.norm(urdf_fk(urdf, q)[:3, 3] - p) < 5e-3, q
    print('OK: FK/IK batem com cr10_robot.xacro')


if __name__ == '__main__':
    main()
