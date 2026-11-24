#!/usr/bin/env python3
"""Busca de grasp (CEM) em lote na GPU, para TODOS os objetos ao mesmo tempo.

A cada iteração cada objeto avalia sua população (N candidatos × K ruídos de posição
do objeto) no GraspSim; os elites atualizam média/desvio do objeto. Ao fim, os
melhores elites de cada objeto são re-avaliados com muitas amostras de ruído e o
grasp mais robusto é salvo em JSON (parâmetros no frame da mão, ver grasp_sim.py).

Uso:  sg render -c 'python3 scripts/search_grasps.py --iters 60 --pop 20000'
"""
import argparse
import json
import time
from pathlib import Path

import torch

from grasp_ml_pack.ml.device import pick_device, describe
from grasp_ml_pack.ml.grasp_sim import GraspSim, OBJECTS, LO, HI, PARAM_DIM

NAMES = ['px', 'py', 'pz', 'psi', 'e', 'rho', 'tgt_thumb', 'tgt_index', 'tgt_middle',
         'tgt_ring', 'tgt_little', 'rotate']


def robust_eval(sim, params, k, sigma, gen):
    """Avalia cada candidato com k ruídos de posição; devolve (reward médio, taxa de sucesso)."""
    B = params.shape[0]
    p = params.repeat_interleave(k, 0)
    noise = torch.randn(B * k, 3, device=params.device, generator=gen) * sigma
    noise[::k] = 0                                   # 1ª amostra sem ruído
    r = sim.evaluate(p, noise_xyz=noise)
    return r.reward.view(B, k).mean(1), r.success.float().view(B, k).mean(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--objs', nargs='+', default=list(OBJECTS))
    ap.add_argument('--iters', type=int, default=60)
    ap.add_argument('--pop', type=int, default=20000)
    ap.add_argument('--elite', type=float, default=0.02)
    ap.add_argument('--k', type=int, default=3, help='ruídos por candidato durante a busca')
    ap.add_argument('--sigma', type=float, default=0.003, help='ruído de posição do objeto (m)')
    ap.add_argument('--final-k', type=int, default=64)
    ap.add_argument('--out', default='data/grasp_search')
    ap.add_argument('--device', default='auto')
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()

    dev = pick_device(a.device)
    print(describe(dev))
    torch.manual_seed(a.seed)
    gen = torch.Generator(device=dev).manual_seed(a.seed)
    lo, hi = LO.to(dev), HI.to(dev)
    n_el = max(8, int(a.pop * a.elite))

    sims = {o: GraspSim(o, dev) for o in a.objs}
    mean = {o: (lo + hi) / 2 for o in a.objs}
    std = {o: (hi - lo) / 2 for o in a.objs}
    elites = {}

    t0 = time.time()
    for it in range(1, a.iters + 1):
        for o in a.objs:                            # todos os objetos avançam juntos
            x = (mean[o] + std[o] * torch.randn(a.pop, PARAM_DIM, device=dev, generator=gen))
            x = torch.maximum(torch.minimum(x, hi), lo)
            rew, succ = robust_eval(sims[o], x, a.k, a.sigma, gen)
            score = rew + succ                      # favorece robustez
            top = score.topk(n_el).indices
            e = x[top]
            mean[o] = 0.3 * mean[o] + 0.7 * e.mean(0)
            std[o] = (0.3 * std[o] + 0.7 * e.std(0)).clamp(min=1e-3 * (hi - lo))
            elites[o] = e
            if it % 10 == 0 or it == a.iters:
                print(f'it {it:3d} {o:7s} succ(top)={succ[top].mean():.2f} '
                      f'succ(pop)={succ.mean():.3f} reward(top)={rew[top].mean():+.2f}')
    print(f'busca: {time.time() - t0:.1f}s')

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for o in a.objs:
        rew, succ = robust_eval(sims[o], elites[o], a.final_k, a.sigma, gen)
        best = (succ * 10 + rew).argmax()
        p = elites[o][best]
        det = sims[o].evaluate(p[None])
        res = dict(obj=o, params=dict(zip(NAMES, p.tolist())),
                   success_rate_noisy=float(succ[best]), reward_noisy=float(rew[best]),
                   success_nominal=bool(det.success[0]), closure=float(det.closure[0]),
                   n_contacts=int(det.n_contacts[0]), sigma_m=a.sigma)
        (out / f'{o}.json').write_text(json.dumps(res, indent=2))
        print(f'{o:7s} sucesso c/ ruído {res["success_rate_noisy"]:.0%} · nominal '
              f'{res["success_nominal"]} · contatos {res["n_contacts"]} · '
              f'closure {res["closure"]:+.2f} → {out / (o + ".json")}')


if __name__ == '__main__':
    main()
