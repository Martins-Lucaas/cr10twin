"""imu_math.py — conta pura do rastreio por IMU (sem ROS, sem Tk).

Convenções:
  * quaternion em (w, x, y, z), como o Madgwick do WBAN entrega; a rotação
    leva do referencial do SENSOR para o da Terra (x = norte magnético
    horizontal, z = cima);
  * o "mundo humano" é esse referencial da Terra. O robô tem z para cima
    também, então só falta um yaw (heading_deg) entre os dois.
"""
from __future__ import annotations

import math
import threading

import numpy as np

# Eixo local do sensor que aponta ao longo do segmento (proximal → distal).
LIMB_AXES = {
    '+X': (1.0, 0.0, 0.0), '-X': (-1.0, 0.0, 0.0),
    '+Y': (0.0, 1.0, 0.0), '-Y': (0.0, -1.0, 0.0),
    '+Z': (0.0, 0.0, 1.0), '-Z': (0.0, 0.0, -1.0),
}


def quat_to_R(q) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


def q_mul(a, b) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


def wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def twist_deg(q_ref, q, axis) -> float:
    """Rotação (graus) de q relativa a q_ref em torno de `axis`, no
    referencial do sensor em q_ref (componente twist do swing-twist)."""
    q_ref_inv = np.array([q_ref[0], -q_ref[1], -q_ref[2], -q_ref[3]])
    dq = q_mul(q_ref_inv, np.asarray(q, dtype=float))
    proj = float(np.dot(dq[1:], axis))
    return math.degrees(2.0 * math.atan2(proj, dq[0]))


def chain_position(rotations, lengths, axis_local) -> np.ndarray:
    """Ponta de uma cadeia de segmentos rígidos (m, mundo humano): cada
    segmento i tem comprimento L_i e aponta para R_i·axis_local."""
    a = np.asarray(axis_local, dtype=float)
    p = np.zeros(3)
    for R, L in zip(rotations, lengths):
        p += float(L) * (R @ a)
    return p


def rot_z(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rotvec(R: np.ndarray) -> np.ndarray:
    """Logaritmo de R: eixo·ângulo (rad)."""
    cos_a = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    ang = math.acos(cos_a)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if ang < 1e-9:
        return 0.5 * v
    if math.pi - ang < 1e-6:
        # perto de 180°: o eixo sai da coluna dominante de (R + I)/2
        M = (R + np.eye(3)) / 2.0
        axis = M[:, int(np.argmax(np.diag(M)))]
        return ang * axis / np.linalg.norm(axis)
    return ang * v / (2.0 * math.sin(ang))


def exp_rotvec(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    ang = float(np.linalg.norm(v))
    if ang < 1e-12:
        return np.eye(3)
    k = v / ang
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def cartesian_target(p_tcp0, R_tcp0, dp_world, dR_world, *,
                     heading_deg: float = 0.0, scale: float = 1.0,
                     box_m=(0.25, 0.25, 0.25)):
    """Pose alvo do TCP (frame da base): a pose do TCP no Start somada ao
    deslocamento/rotação da mão desde o Start, levados do mundo humano ao
    do robô por um yaw. Posição presa numa caixa em torno do Start."""
    Rz = rot_z(heading_deg)
    dp = scale * (Rz @ np.asarray(dp_world, dtype=float))
    box = np.asarray(box_m, dtype=float)
    dp = np.clip(dp, -box, box)
    dR = Rz @ dR_world @ Rz.T
    return np.asarray(p_tcp0, dtype=float) + dp, dR @ R_tcp0


def step_toward_pose(p, R, p_des, R_des, max_lin_m: float, max_ang_rad: float):
    """Anda de (p, R) em direção a (p_des, R_des) com teto de passo linear e
    angular — é o limite de velocidade do TCP."""
    dp = np.asarray(p_des, dtype=float) - p
    n = float(np.linalg.norm(dp))
    if n > max_lin_m:
        dp *= max_lin_m / n
    w = rotvec(R_des @ R.T)
    a = float(np.linalg.norm(w))
    if a > max_ang_rad:
        w *= max_ang_rad / a
    return p + dp, exp_rotvec(w) @ R


class AccelIntegrator:
    """Deslocamento da mão (m, mundo humano) por dead-reckoning da aceleração.

    a_mundo = R(q)·a_sensor − viés. O sensor mede força específica, então em
    repouso a_mundo = (0, 0, +g) mais o erro de inclinação do Madgwick; em vez
    de fixar g, o viés é estimado nos REPOUSOS (janela de ~0,5 s no início e,
    depois, média móvel lenta sempre que a mão para). Parada detectada
    (giro e aceleração pequenos por `still_n` amostras) zera a velocidade — o
    ZUPT que segura a deriva da dupla integração. Fora do repouso a velocidade
    vaza com `tau_v`, o que limita o erro de uma rajada ruim.

    Não é posição absoluta: o erro cresce com cada movimento; o `reset()` (re-zero
    da aba) é o que o descarta. Dado perdido (`dt > max_gap_s`) zera a velocidade.
    """

    def __init__(self, seed_s: float = 0.5, still_gyro: float = 0.08,
                 still_acc: float = 0.35, still_n: int = 6, tau_v: float = 2.0,
                 bias_tau_s: float = 1.0, max_v: float = 1.0,
                 max_gap_s: float = 0.1):
        self.seed_s, self.still_gyro, self.still_acc = seed_s, still_gyro, still_acc
        self.still_n, self.tau_v, self.bias_tau_s = still_n, tau_v, bias_tau_s
        self.max_v, self.max_gap_s = max_v, max_gap_s
        self._lock = threading.Lock()
        self.bias: np.ndarray | None = None
        self._seed: list = []
        self._seed_t = 0.0
        self._t: float | None = None
        self._still = 0
        self.v = np.zeros(3)
        self.p = np.zeros(3)

    @property
    def ready(self) -> bool:
        return self.bias is not None

    def reset(self) -> None:
        """Descarta posição e velocidade (o viés aprendido fica)."""
        with self._lock:
            self.v = np.zeros(3)
            self.p = np.zeros(3)
            self._still = 0

    def displacement(self) -> np.ndarray:
        with self._lock:
            return self.p.copy()

    def update(self, t: float, R: np.ndarray, acc, gyro_rad) -> None:
        a_w = np.asarray(R, dtype=float) @ np.asarray(acc, dtype=float)
        gyro = float(np.linalg.norm(gyro_rad))
        if not (np.all(np.isfinite(a_w)) and np.isfinite(gyro)):
            return
        with self._lock:
            dt = None if self._t is None else t - self._t
            self._t = t
            if dt is None or dt <= 0.0:
                return
            if dt > self.max_gap_s:
                self.v = np.zeros(3)
                self._seed, self._still = [], 0
                return
            if self.bias is None:
                if gyro > self.still_gyro:
                    self._seed = []
                    return
                if not self._seed:
                    self._seed_t = t
                self._seed.append(a_w)
                if t - self._seed_t >= self.seed_s:
                    self.bias = np.mean(self._seed, axis=0)
                    self._seed = []
                return
            a = a_w - self.bias
            if gyro < self.still_gyro and float(np.linalg.norm(a)) < self.still_acc:
                self._still += 1
                if self._still >= self.still_n:
                    self.v = np.zeros(3)
                    k = 1.0 - math.exp(-dt / self.bias_tau_s)
                    self.bias = self.bias + k * (a_w - self.bias)
                    return
            else:
                self._still = 0
            self.v = (self.v + a * dt) * math.exp(-dt / self.tau_v)
            n = float(np.linalg.norm(self.v))
            if n > self.max_v:
                self.v *= self.max_v / n
            self.p = self.p + self.v * dt


def clamp_radius(p, r_min: float, fallback_dir=(1.0, 0.0)) -> np.ndarray:
    """Empurra p para fora do cilindro de raio r_min em torno do eixo z da
    base (a coluna vertical x=y=0), na direção radial. Dentro dele a IK do
    braço fica quase singular: J1 e J6 giram juntos para um alvo que mal
    anda. Em cima do eixo (r≈0) não há direção radial; usa `fallback_dir`."""
    p = np.asarray(p, dtype=float).copy()
    r = math.hypot(p[0], p[1])
    if r >= r_min:
        return p
    if r > 1e-6:
        d = p[:2] / r
    else:
        d = np.asarray(fallback_dir, dtype=float)
        d = d / np.linalg.norm(d)
    p[:2] = d * r_min
    return p
