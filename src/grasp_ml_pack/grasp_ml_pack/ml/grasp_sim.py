"""
grasp_sim.py — simulador GEOMÉTRICO de grasp em lote (milhares de tentativas/passo).

Modelo (rápido, sem física de contato — a física fica por conta da validação no
Gazebo):
  * objeto = cilindro vertical (raio r, meia-altura H) — dimensões reais do SDF
  * mão = caixas de colisão reais por falange + palma (hand_model.py)
  * punho/antebraço = anel de pontos atrás da base da mão (colide com objeto/mesa)
  * fases: pré-shape aberto → varredura de aproximação (sem tocar o objeto) →
    fechamento incremental dedo a dedo, cada dedo PARA ao tocar (como o
    PerfectGrasp) → métrica de fechamento de força (cone de atrito 2D).

Parâmetros de um candidato (12), todos no frame da mão:
  p_h (3) centro do objeto · euler (ψ,e,ρ) orientação mão→mundo ·
  tgt (5) alvo de fechamento por dedo · rot (1) Rotate (oposição do polegar).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import torch

from .hand_model import HandModel, LOWER, FINGERS

# Objetos reais (SDF em conveyor_controller.py)
OBJECTS = {
    'frasco': dict(r=0.025,  H=0.045,  cz=0.851),
    'tubo':   dict(r=0.012,  H=0.060,  cz=0.866),
    'ampola': dict(r=0.005,  H=0.0375, cz=0.844),
}
BELT_TOP = 0.806
TABLE_MARGIN = 0.012      # mão ≥ 12 mm acima da esteira
TOUCH_EPS = 0.002         # gap ≤ 2 mm = contato
MU_EFF = 0.7              # atrito conservador (SDF usa 2.5)
N_CLOSE = 20              # passos do fechamento
APPROACH_D = (0.10, 0.07, 0.045, 0.025, 0.012, 0.0)   # distâncias da varredura (m)

PARAM_DIM = 12
# limites de amostragem: p_h(3), euler(3), tgt(5), rot(1)
LO = torch.tensor([-0.06, 0.00, -0.05, -math.pi, -math.pi, -math.pi,
                   0.1, 0.1, 0.1, 0.1, 0.1, 0.0])
HI = torch.tensor([0.06, 0.17, 0.12, math.pi, math.pi, math.pi,
                   1.0, 1.0, 1.0, 1.0, 1.0, 1.0])


def euler_to_R(psi, e, rho):
    """R = Rz(ψ)·Ry(e)·Rx(ρ), (B,3,3)."""
    cz, sz = torch.cos(psi), torch.sin(psi)
    cy, sy = torch.cos(e), torch.sin(e)
    cx, sx = torch.cos(rho), torch.sin(rho)
    o, z = torch.ones_like(psi), torch.zeros_like(psi)
    Rz = torch.stack([cz, -sz, z, sz, cz, z, z, z, o], -1).reshape(-1, 3, 3)
    Ry = torch.stack([cy, z, sy, z, o, z, -sy, z, cy], -1).reshape(-1, 3, 3)
    Rx = torch.stack([o, z, z, z, cx, -sx, z, sx, cx], -1).reshape(-1, 3, 3)
    return Rz @ Ry @ Rx


def _box_sdf(P, C, R, half):
    """SDF de caixas orientadas. P (B,M,3) · C (B,NB,3) · R (B,NB,3,3) · half (NB,3)
    → (B,NB,M)."""
    d = P[:, None, :, :] - C[:, :, None, :]                       # (B,NB,M,3)
    q = torch.einsum('bnji,bnmj->bnmi', R, d).abs() - half[None, :, None, :]
    outside = q.clamp(min=0).norm(dim=-1)
    inside = q.max(dim=-1).values.clamp(max=0)
    return outside + inside


def _cyl_sdf(Q, c, a, r, H):
    """SDF de cilindro. Q (...,3) · c,a (B,3) broadcast na 1ª dim."""
    sh = [-1] + [1] * (Q.dim() - 2) + [3]
    d = Q - c.reshape(sh)
    t = (d * a.reshape(sh)).sum(-1)
    rad = (d - t[..., None] * a.reshape(sh)).norm(dim=-1)
    dr, dz = rad - r, t.abs() - H
    out = torch.stack([dr.clamp(min=0), dz.clamp(min=0)], -1).norm(dim=-1)
    ins = torch.maximum(dr, dz).clamp(max=0)
    return out + ins


@dataclass
class Result:
    reward: torch.Tensor
    success: torch.Tensor
    closure: torch.Tensor        # margem normalizada (>0 = fechamento de força)
    n_contacts: torch.Tensor
    appr_pen: torch.Tensor       # maior penetração na aproximação (m)
    table_viol: torch.Tensor     # maior invasão da esteira (m)
    contact: torch.Tensor        # (B,6) 1=contato [Thumb..Little, palma]
    drv_final: torch.Tensor      # (B,6) drivers ao fim do fechamento


class GraspSim:
    def __init__(self, obj: str, device='cpu'):
        self.obj = obj
        self.dev = torch.device(device)
        self.hand = HandModel(self.dev)
        g = OBJECTS[obj]
        self.r, self.H, self.cz = g['r'], g['H'], g['cz']
        n_th, n_h = 12, 7
        th = torch.linspace(0, 2 * math.pi, n_th + 1)[:-1]
        hh = torch.linspace(-self.H, self.H, n_h)
        self.th = th.to(self.dev)
        self.hh = hh.to(self.dev)
        self.lo, self.hi = LO.to(self.dev), HI.to(self.dev)
        self.lower = LOWER.to(self.dev)
        # anel do punho no frame da mão (antebraço atrás da base)
        ys = torch.linspace(-0.04, -0.20, 5)
        ang = torch.linspace(0, 2 * math.pi, 9)[:-1]
        wr = [[0.05 * math.cos(a), float(y), 0.05 * math.sin(a)] for y in ys for a in ang]
        self.wrist = torch.tensor(wr, dtype=torch.float32, device=self.dev)

    # geometria do objeto no frame da mão
    def _cyl_points(self, c, a):
        """Pontos de superfície do cilindro (B,M,3) para eixo a (B,3) e centro c."""
        B = c.shape[0]
        helper = torch.where((a[:, 2:3].abs() < 0.9),
                             torch.tensor([[0., 0., 1.]], device=c.device).expand(B, 3),
                             torch.tensor([[1., 0., 0.]], device=c.device).expand(B, 3))
        e1 = torch.nn.functional.normalize(torch.cross(a, helper, dim=-1), dim=-1)
        e2 = torch.cross(a, e1, dim=-1)
        ct, st = torch.cos(self.th), torch.sin(self.th)
        ring = (ct[None, :, None] * e1[:, None] + st[None, :, None] * e2[:, None]) * self.r   # (B,T,3)
        pts = c[:, None, None, :] + ring[:, :, None, :] + self.hh[None, None, :, None] * a[:, None, None, :]
        return pts.reshape(B, -1, 3), e1, e2

    def _gap(self, C, R, c, a):
        """Menor distância caixa↔cilindro. Retorna gap (B,NB), ponto de contato (B,NB,3)."""
        cp, _, _ = self._cyl_points(c, a)
        sdf_a = _box_sdf(cp, C, R, self.hand.half)                    # (B,NB,M)
        ga, ia = sdf_a.min(dim=-1)
        pa = torch.gather(cp[:, None].expand(-1, C.shape[1], -1, -1), 2,
                          ia[..., None, None].expand(-1, -1, 1, 3))[:, :, 0]
        bp = self.hand.box_points(C, R)                              # (B,NB,27,3)
        sdf_b = _cyl_sdf(bp, c, a, self.r, self.H)                    # (B,NB,27)
        gb, ib = sdf_b.min(dim=-1)
        pb = torch.gather(bp, 2, ib[..., None, None].expand(-1, -1, 1, 3))[:, :, 0]
        use_a = ga <= gb
        return torch.where(use_a, ga, gb), torch.where(use_a[..., None], pa, pb)

    def evaluate(self, params: torch.Tensor, noise_xyz: torch.Tensor | None = None,
                 mu: float = MU_EFF) -> Result:
        """params (B,12). noise_xyz (B,3): erro de posição do objeto no frame MUNDO."""
        dev = self.dev
        params = params.to(dev)
        B = params.shape[0]
        p_h, eul = params[:, 0:3], params[:, 3:6]
        tgt = torch.maximum(params[:, 6:11], self.lower[None, :5] + 0.01)
        rot = params[:, 11]
        R_wh = euler_to_R(eul[:, 0], eul[:, 1], eul[:, 2])           # mão→mundo
        z_hat = torch.tensor([0., 0., 1.], device=dev)
        a_h = R_wh.transpose(1, 2) @ z_hat                           # eixo do objeto na mão
        u_w = R_wh[:, :, 2]                                          # direção de aproximação (mundo)
        if noise_xyz is not None:
            p_h = p_h + torch.einsum('bji,bj->bi', R_wh, noise_xyz.to(dev))

        # pré-shape aberto (Rotate = param)
        drv0 = torch.zeros(B, 6, device=dev)
        drv0[:, :5] = self.lower[:5]
        drv0[:, 5] = rot
        C0, Rb0 = self.hand.box_poses(drv0)
        zero_pen = torch.zeros(B, device=dev)

        # 1) varredura de aproximação: mão aberta chega por +z_h
        appr_pen = zero_pen.clone()
        table_viol = zero_pen.clone()
        bpts0 = self.hand.box_points(C0, Rb0).reshape(B, -1, 3)
        allpts = torch.cat([bpts0, self.wrist[None].expand(B, -1, -1)], 1)   # (B,P,3)
        zobj = self.cz
        for d in APPROACH_D:
            c = p_h + torch.tensor([0., 0., d], device=dev)
            gap, _ = self._gap(C0, Rb0, c, a_h)
            # no ponto final (d=0) a palma pode encostar; nas demais nada pode
            pen = (-gap).clamp(min=0)
            if d == 0.0:
                pen = pen[:, 1:]                                      # ignora a palma
            appr_pen = torch.maximum(appr_pen, pen.max(dim=-1).values)
            # antebraço/punho vs objeto
            wsdf = _cyl_sdf(self.wrist[None].expand(B, -1, -1), c, a_h, self.r, self.H)
            appr_pen = torch.maximum(appr_pen, (-wsdf).clamp(min=0).max(dim=-1).values)
            # esteira: z_world dos pontos da mão = z_obj + R·(P − c_h) − d·u? (c já inclui o recuo d)
            zw = zobj + torch.einsum('bj,bpj->bp', R_wh[:, 2, :], allpts - c[:, None, :])
            table_viol = torch.maximum(table_viol,
                                       (BELT_TOP + TABLE_MARGIN - zw).clamp(min=0).max(dim=-1).values)

        # restrições de alcance: sem vir de baixo; direção dentro de ±110° de +x
        up_viol = (u_w[:, 2] - 0.05).clamp(min=0)
        heading = torch.atan2(u_w[:, 1], u_w[:, 0]).abs()
        head_viol = (heading - math.radians(110)).clamp(min=0) * (u_w[:, 2].abs() < 0.9).float()

        # 2) fechamento incremental dedo a dedo
        drv = drv0.clone()
        stalled = torch.zeros(B, 5, dtype=torch.bool, device=dev)
        c_fin = p_h
        ang = torch.zeros(B, 6, device=dev)
        has = torch.zeros(B, 6, dtype=torch.bool, device=dev)
        # contato da palma no estado final de aproximação
        gap0, pt0 = self._gap(C0, Rb0, c_fin, a_h)
        _, e1, e2 = self._cyl_points(c_fin, a_h)
        def _angle(pt):
            v = pt - c_fin
            return torch.atan2((v * e2).sum(-1), (v * e1).sum(-1))
        pal = gap0[:, 0] < TOUCH_EPS
        has[:, 5] = pal
        ang[:, 5] = _angle(pt0[:, 0])
        fb = self.hand.finger_boxes
        for s in range(1, N_CLOSE + 1):
            frac = s / N_CLOSE
            cand = self.lower[:5] + (tgt - self.lower[:5]) * frac
            drv = drv.clone()
            drv[:, :5] = torch.where(stalled, drv[:, :5], cand)
            C, Rb = self.hand.box_poses(drv)
            gap, pt = self._gap(C, Rb, c_fin, a_h)
            for i, f in enumerate(FINGERS):
                idx = fb[f]
                # só as falanges de dedo (não o chassi do polegar, estático no fechamento)
                idx = [j for j in idx if not self.hand.names[j].endswith('chassis')]
                g = gap[:, idx]
                gmin, jm = g.min(dim=-1)
                newly = (~stalled[:, i]) & (gmin < TOUCH_EPS)
                p_sel = pt[:, idx][torch.arange(B), jm]
                ang[:, i] = torch.where(newly, _angle(p_sel), ang[:, i])
                has[:, i] = has[:, i] | newly
                stalled[:, i] = stalled[:, i] | newly

        # 3) fechamento de força 2D (normais radiais; cone de atrito ±atan μ)
        n_c = has.sum(-1)
        alpha = math.atan(mu)
        big = torch.where(has, ang, torch.full_like(ang, 1e3))
        sa, _ = torch.sort(big, dim=-1)                               # contatos primeiro
        n_max = 6
        idxs = torch.arange(n_max, device=dev)[None].expand(B, -1)
        valid = idxs < n_c[:, None]
        nxt = torch.roll(sa, -1, dims=-1)
        # lacuna entre contatos consecutivos; a última fecha com o primeiro (+2π)
        last = (idxs == (n_c[:, None] - 1))
        first = sa[:, 0:1].expand(-1, n_max)
        gaps = torch.where(last, first + 2 * math.pi - sa, nxt - sa)
        gaps = torch.where(valid, gaps, torch.zeros_like(gaps))
        maxgap = gaps.max(dim=-1).values
        margin = (math.pi + 2 * alpha - maxgap) / (2 * alpha)         # >0 = fechamento
        margin = torch.where(n_c >= 2, margin, torch.full_like(margin, -1.0)).clamp(-1, 1)

        n_f = has[:, :5].sum(-1).float()
        thumb = has[:, 0].float()
        reward = (1.5 * margin + 0.25 * n_f / 4 + 0.35 * thumb
                  - 6.0 * appr_pen / 0.005 * 0.1 - 4.0 * table_viol / 0.01 * 0.25
                  - 2.0 * up_viol - 2.0 * head_viol)
        reward = reward - 0.5 * (n_c < 3).float()
        success = ((margin > 0.1) & (n_c >= 3) & (appr_pen < 0.002) & (table_viol < 0.002)
                   & (up_viol < 1e-6) & (head_viol < 1e-6))
        return Result(reward, success, margin, n_c, appr_pen, table_viol,
                      has.float(), drv)
