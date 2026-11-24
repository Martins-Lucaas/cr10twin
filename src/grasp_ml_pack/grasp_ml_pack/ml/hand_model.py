"""
hand_model.py — mão COVVI em LOTE (torch): caixas de colisão reais por falange.

Usa as mesmas cadeias URDF de `kinematics._FINGER_CHAINS` (já validadas) e as
caixas de colisão de `linear_covvi_hand_gazebo.urdf`. Tudo no frame hand_base_link
(+y = dedos, +z = frente da palma / sentido de aproximação).

Ordem dos drivers: Thumb, Index, Middle, Ring, Little, Rotate.
"""
from __future__ import annotations

import numpy as np
import torch

from ..kinematics import _FINGER_CHAINS, HAND_LOWER

DRIVERS = ('Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate')
FINGERS = ('Thumb', 'Index', 'Middle', 'Ring', 'Little')
LOWER = torch.tensor([HAND_LOWER[d] for d in DRIVERS], dtype=torch.float32)
UPPER = torch.ones(6)

# (nome, dedo|None, k = nº de passos da cadeia que movem o elo, centro, tamanho)
# Fonte: <collision> de linear_covvi_hand_gazebo.urdf. Palma = cubo 8 cm na
# origem de base_link (é a colisão REAL da palma no Gazebo). thumb_chassis: bbox
# do STL (right_thumb_chassis.STL).
_BOXES = [
    ('palm',          None,     0, (0, 0, 0),                    (0.08, 0.08, 0.08)),
    ('thumb_chassis', 'Thumb',  1, (0.0348, 0.0273, 0.0120),     (0.0356, 0.0472, 0.0244)),
    ('thumb_prox',    'Thumb',  2, (0.0655, 0.0440, 0.0097),     (0.0485, 0.0537, 0.0216)),
    ('thumb_dist',    'Thumb',  3, (0.0986, 0.0732, 0.0096),     (0.0324, 0.0332, 0.0210)),
    ('index_prox',    'Index',  1, (0.0225, 0.1031, -0.0188),    (0.0145, 0.0425, 0.0205)),
    ('index_dist',    'Index',  2, (0.0265, 0.1374, -0.0080),    (0.0166, 0.0380, 0.0275)),
    ('middle_prox',   'Middle', 1, (0.0028, 0.1083, -0.0194),    (0.0146, 0.0461, 0.0207)),
    ('middle_dist',   'Middle', 2, (0.0045, 0.1445, -0.0088),    (0.0172, 0.0378, 0.0289)),
    ('ring_prox',     'Ring',   1, (-0.0166, 0.1072, -0.0148),   (0.0144, 0.0444, 0.0206)),
    ('ring_dist',     'Ring',   2, (-0.0172, 0.1424, -0.0045),   (0.0158, 0.0379, 0.0278)),
    ('little_prox',   'Little', 1, (-0.0333, 0.0929, -0.0092),   (0.0143, 0.0367, 0.0179)),
    ('little_dist',   'Little', 2, (-0.0361, 0.1196, -0.0008),   (0.0161, 0.0293, 0.0254)),
]


def _T12(m) -> torch.Tensor:
    T = torch.eye(4, dtype=torch.float64)
    T[:3, :3] = torch.tensor(m[:9], dtype=torch.float64).reshape(3, 3)
    T[:3, 3] = torch.tensor(m[9:], dtype=torch.float64)
    return T


def _rot(axis: torch.Tensor, theta: torch.Tensor) -> torch.Tensor:
    """Rodrigues em lote: axis (3,), theta (B,) → (B,4,4)."""
    k = axis / (axis.norm() + 1e-12)
    K = torch.tensor([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]],
                     dtype=theta.dtype, device=theta.device)
    c = torch.cos(theta)[:, None, None]
    s = torch.sin(theta)[:, None, None]
    R = torch.eye(3, dtype=theta.dtype, device=theta.device) + s * K + (1 - c) * (K @ K)
    T = torch.eye(4, dtype=theta.dtype, device=theta.device).repeat(theta.shape[0], 1, 1)
    T[:, :3, :3] = R
    return T


class HandModel:
    def __init__(self, device: torch.device | str = 'cpu'):
        self.dev = torch.device(device)
        self.dt = torch.float32
        self.chains = {}
        for f in FINGERS:
            ch = _FINGER_CHAINS[f]
            steps = []
            for (M, axis, mult, off, drv) in ch['steps']:
                steps.append((_T12(M).to(self.dev, self.dt),
                              torch.tensor(axis, dtype=self.dt, device=self.dev),
                              float(mult), float(off), DRIVERS.index(drv)))
            self.chains[f] = steps
        # T_k(0) para cada dedo/k
        self._T0 = {}
        z = torch.zeros(1, 6, dtype=self.dt, device=self.dev)
        for f in FINGERS:
            for k in range(1, len(self.chains[f]) + 1):
                self._T0[(f, k)] = torch.linalg.inv(self._chain(f, k, z)[0])
        self.names = [b[0] for b in _BOXES]
        self.nb = len(_BOXES)
        self.c0 = torch.tensor([b[3] for b in _BOXES], dtype=self.dt, device=self.dev)
        self.half = torch.tensor([b[4] for b in _BOXES], dtype=self.dt, device=self.dev) / 2
        self.finger_of = [b[1] for b in _BOXES]
        self.finger_boxes = {f: [i for i, b in enumerate(_BOXES) if b[1] == f] for f in FINGERS}
        # 27 pontos por caixa (centro, 8 cantos, 6 faces, 12 arestas) — em coords unitárias
        u = []
        for sx in (-1, 0, 1):
            for sy in (-1, 0, 1):
                for sz in (-1, 0, 1):
                    u.append((sx, sy, sz))
        self.unit = torch.tensor(u, dtype=self.dt, device=self.dev)      # (27,3)

    def _chain(self, f: str, k: int, drv: torch.Tensor) -> torch.Tensor:
        T = torch.eye(4, dtype=self.dt, device=drv.device).repeat(drv.shape[0], 1, 1)
        for (M, axis, mult, off, di) in self.chains[f][:k]:
            q = mult * drv[:, di] + off
            T = T @ M.to(drv.device) @ _rot(axis.to(drv.device), q)
        return T

    def box_poses(self, drv: torch.Tensor):
        """drv (B,6) → centros (B,NB,3), rotações (B,NB,3,3)."""
        B = drv.shape[0]
        C = self.c0.expand(B, -1, -1).clone()
        R = torch.eye(3, dtype=self.dt, device=drv.device).expand(B, self.nb, 3, 3).clone()
        for i, (_, f, k, _, _) in enumerate(_BOXES):
            if f is None:
                continue
            L = self._chain(f, k, drv) @ self._T0[(f, k)].to(drv.device)
            R[:, i] = L[:, :3, :3]
            C[:, i] = (L[:, :3, :3] @ self.c0[i]) + L[:, :3, 3]
        return C, R

    def box_points(self, C, R):
        """Pontos de superfície/vol. de cada caixa: (B,NB,27,3)."""
        loc = self.unit[None, None] * self.half[None, :, None, :]       # (1,NB,27,3)
        return torch.einsum('bnij,bnpj->bnpi', R, loc.expand(C.shape[0], -1, -1, -1)) + C[:, :, None, :]

    def tips(self, drv: torch.Tensor) -> dict:
        """Pontas reais (mesma FK de kinematics.hand_fk) — usado nos testes."""
        out = {}
        for f in FINGERS:
            ch = _FINGER_CHAINS[f]
            T = self._chain(f, len(ch['steps']), drv) @ _T12(ch['post']).to(self.dev, self.dt)
            tip = torch.tensor(ch['tip'], dtype=self.dt, device=self.dev)
            out[f] = T[:, :3, :3] @ tip + T[:, :3, 3]
        return out
