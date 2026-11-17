#!/usr/bin/env python3
"""PerfectGrasp sem simulador: dedos fake que seguem o comando, exceto o
Index, travado por um "objeto" na falange proximal. Verifica:
  - Index: contato detectado pela FALANGE (não pelo driver virtual) e alvo
    final = contato + HOLD_SQUEEZE (aperto, não força zero);
  - Middle (livre, fechando): termina além do alvo pelo squeeze;
  - Ring (alvo = posição inicial): não ganha squeeze.
    python3 src/grasp_ml_pack/scripts/test_perfect_grasp.py
"""
import types

from sensor_msgs.msg import JointState

from grasp_ml_pack import perfect_grasp as pgm
from grasp_ml_pack.perfect_grasp import PerfectGrasp, _MIMIC_RATIOS

pgm.time.sleep = lambda s: None          # sem espera real
BLOCK_INDEX = 0.40                       # posição (driver) onde o Index encosta

log = types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)
node = types.SimpleNamespace(get_logger=lambda: log)
pg = None
sent = []


def send(cfg, _dur):
    sent.append(dict(cfg))
    msg = JointState()
    for j, v in cfg.items():                          # drivers seguem sempre
        msg.name.append(j); msg.position.append(v)
    for m, (prim, ratio) in _MIMIC_RATIOS.items():    # falanges: Index trava
        v = cfg.get(prim, 0.0)
        if prim == 'Index':
            v = min(v, BLOCK_INDEX)
        msg.name.append(m); msg.position.append(v * ratio)
    pg.update_from_joint_state(msg)


pg = PerfectGrasp(node, send_hand_fn=send)
send({j: 0.12 for j in ('Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate')}, 0)
target = {'Thumb': 0.12, 'Index': 0.9, 'Middle': 0.6, 'Ring': 0.12, 'Little': 0.12, 'Rotate': 0.12}
res = pg.close_until_contact(target, label='test')
final = sent[-1]
sq = PerfectGrasp.HOLD_SQUEEZE_RAD

assert res.contact_detected and res.stalled['Index'], res.summary()
assert abs(final['Index'] - (BLOCK_INDEX + sq)) < 1e-6, final['Index']
assert abs(final['Middle'] - (0.6 + sq)) < 1e-6, final['Middle']
assert abs(final['Ring'] - 0.12) < 1e-6, final['Ring']
assert not res.timed_out
print('OK:', res.summary())
