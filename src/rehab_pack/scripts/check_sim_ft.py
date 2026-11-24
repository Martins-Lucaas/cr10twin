#!/usr/bin/env python3
"""Valida a F/T simulada (sim_ft:=true) com o braço parado em HOME.

Uso (Gazebo no ar com sim_ft:=true):
    python3 scripts/check_sim_ft.py [--seg 20] [--sinal]

1. Coleta `--seg` s de /sim/load_cell/wrench e /joint_states.
   APROVADO se o pico-a-pico de joint4 < 1e-3 rad e o desvio-padrão de |F| < 0,1 N.
2. `--sinal`: aplica 2 N em +X do world na placa do mastro (/apply_link_wrench)
   e imprime o sinal do plugin (sim_sign sugerido = −sinal medido em x do world).
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from gazebo_msgs.srv import ApplyLinkWrench
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

PLACA = 'cr10_joystick::joystick_plate_link'


class Coleta(Node):
    def __init__(self):
        super().__init__('check_sim_ft')
        self.f, self.j4 = [], []
        self.create_subscription(WrenchStamped, '/sim/load_cell/wrench',
                                 lambda m: self.f.append(
                                     [m.wrench.force.x, m.wrench.force.y,
                                      m.wrench.force.z]), 100)
        self.create_subscription(JointState, '/joint_states', self._js, 50)

    def _js(self, m):
        if 'joint4' in m.name:
            self.j4.append(m.position[m.name.index('joint4')])

    def coletar(self, seg):
        self.f, self.j4 = [], []
        t0 = time.monotonic()
        while time.monotonic() - t0 < seg:
            rclpy.spin_once(self, timeout_sec=0.05)
        return np.array(self.f), np.array(self.j4)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seg', type=float, default=20.0)
    ap.add_argument('--sinal', action='store_true')
    a = ap.parse_args()
    rclpy.init()
    n = Coleta()
    f, j4 = n.coletar(a.seg)
    if len(f) < 10 or len(j4) < 10:
        print(f'REPROVADO: poucas amostras (F={len(f)}, joint4={len(j4)})')
        return 1
    mod = np.linalg.norm(f, axis=1)
    pp, sd = float(np.ptp(j4)), float(np.std(mod))
    print(f'amostras F={len(f)} joint4={len(j4)}')
    print(f'joint4 pico-a-pico = {pp:.2e} rad (limite 1e-3)')
    print(f'|F| media = {mod.mean():.3f} N, desvio-padrao = {sd:.3f} N (limite 0,1)')
    ok = pp < 1e-3 and sd < 0.1

    if a.sinal:
        base, _ = n.coletar(3.0)
        cli = n.create_client(ApplyLinkWrench, '/apply_link_wrench')
        if not cli.wait_for_service(timeout_sec=5.0):
            print('sem /apply_link_wrench: sinal NAO verificado')
        else:
            req = ApplyLinkWrench.Request()
            req.link_name = PLACA
            req.reference_frame = 'world'
            req.wrench.force.x = 2.0
            req.duration.sec = 4
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
            time.sleep(1.0)
            com, _ = n.coletar(2.0)
            d = com.mean(axis=0) - base.mean(axis=0)
            print(f'delta F no sensor com +2 N em x do world (placa): '
                  f'{np.round(d, 3)}')
            print('(o delta está no frame load_cell_link; compare com a '
                  'rotação de HOME para decidir sim_sign)')
    print('APROVADO' if ok else 'REPROVADO')
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
