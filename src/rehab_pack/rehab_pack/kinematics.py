"""Cinemática do CR10 + mastro lida do URDF (cadeia world → tcp_link).

FK, jacobiana numérica e passo DLS 6D (posição + manter orientação).
Módulo puro: numpy e xml, sem ROS.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p), np.sin(p),
                              np.cos(y), np.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _rot(axis, q):
    """Rodrigues: rotação de q [rad] em torno de `axis`."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * k + (1 - np.cos(q)) * (k @ k)


def _log(R):
    """Vetor de rotação (eixo·ângulo) de R."""
    c = (np.trace(R) - 1.0) / 2.0
    th = np.arccos(np.clip(c, -1.0, 1.0))
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-9:
        return 0.5 * v
    if np.pi - th < 1e-6:  # perto de π: eixo pela diagonal
        d = np.sqrt(np.maximum((np.diag(R) + 1.0) / 2.0, 0.0))
        i = int(np.argmax(d))
        ax = d.copy()
        for j in range(3):
            if j != i:
                ax[j] = np.copysign(d[j], R[i, j])
        return th * ax / np.linalg.norm(ax)
    return th * v / (2.0 * np.sin(th))


class Chain:
    """Cadeia serial de juntas fixas e revolutas de `root` até `tip`."""

    def __init__(self, urdf_xml: str, root: str = 'world',
                 tip: str = 'tcp_link'):
        tree = ET.fromstring(urdf_xml)
        by_child = {}
        for j in tree.iter('joint'):
            ch = j.find('child')
            if ch is None:  # bloco ros2_control
                continue
            by_child[ch.get('link')] = j
        seq, link = [], tip
        while link != root:
            if link not in by_child:
                raise ValueError(f'{tip} não liga a {root} no URDF')
            j = by_child[link]
            seq.append(j)
            link = j.find('parent').get('link')
        self.names, self.axes, self.limits = [], [], []
        self._steps = []  # (T fixa de origem, índice da junta ou None)
        for j in reversed(seq):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None
                                      else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None
                                      else '0 0 0').split()]
            T = np.eye(4)
            T[:3, :3] = _rpy(*rpy)
            T[:3, 3] = xyz
            if j.get('type') in ('revolute', 'continuous'):
                ax = j.find('axis')
                self.axes.append([float(v) for v in ax.get('xyz').split()]
                                 if ax is not None else [1.0, 0.0, 0.0])
                self.names.append(j.get('name'))
                lim = j.find('limit')
                self.limits.append(
                    (float(lim.get('lower')), float(lim.get('upper')))
                    if lim is not None and lim.get('lower') else (-np.inf,
                                                                   np.inf))
                self._steps.append((T, len(self.names) - 1))
            else:
                self._steps.append((T, None))
        self.n = len(self.names)

    def fk(self, q) -> np.ndarray:
        """Matriz 4×4 da ponta no frame raiz."""
        T = np.eye(4)
        for Tf, i in self._steps:
            T = T @ Tf
            if i is not None:
                R = np.eye(4)
                R[:3, :3] = _rot(self.axes[i], q[i])
                T = T @ R
        return T

    def jacobian(self, q, eps: float = 1e-6) -> np.ndarray:
        """6×n numérica [v; w] no frame raiz (w = rotação infinitesimal)."""
        q = np.asarray(q, float)
        T0 = self.fk(q)
        J = np.zeros((6, self.n))
        for i in range(self.n):
            dq = q.copy()
            dq[i] += eps
            T1 = self.fk(dq)
            J[:3, i] = (T1[:3, 3] - T0[:3, 3]) / eps
            J[3:, i] = _log(T1[:3, :3] @ T0[:3, :3].T) / eps
        return J

    def dls_step(self, q, x_des, R_des, lam: float = 0.02,
                 max_dq: float = 0.2) -> np.ndarray:
        """Um passo DLS 6D rumo a (x_des, R_des); devolve o novo q."""
        q = np.asarray(q, float)
        T = self.fk(q)
        err = np.concatenate([np.asarray(x_des, float) - T[:3, 3],
                              _log(R_des @ T[:3, :3].T)])
        J = self.jacobian(q)
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), err)
        m = np.max(np.abs(dq))
        if m > max_dq:
            dq *= max_dq / m
        return q + dq

    def ik(self, q0, x_des, R_des, iters: int = 20, tol_m: float = 1e-4):
        """Itera o passo DLS a partir de q0. Devolve (q, resíduo de posição m)."""
        q = np.asarray(q0, float)
        for _ in range(iters):
            q = self.dls_step(q, x_des, R_des)
            if np.linalg.norm(self.fk(q)[:3, 3] - x_des) < tol_m:
                break
        return q, float(np.linalg.norm(self.fk(q)[:3, 3] - x_des))
