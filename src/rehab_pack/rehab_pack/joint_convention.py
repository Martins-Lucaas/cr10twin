"""URDF <-> controlador do CR10 (cópia própria do rehab_pack).

J1 tem eixo invertido; J2-J6 têm a mesma convenção. Conferido no braço real.
"""
import numpy as np

_SIGN = np.array([-1., 1., 1., 1., 1., 1.])


def urdf_to_dobot(q_urdf):
    return _SIGN * np.asarray(q_urdf, dtype=np.float64)


def dobot_to_urdf(q_dobot):
    return _SIGN * np.asarray(q_dobot, dtype=np.float64)
