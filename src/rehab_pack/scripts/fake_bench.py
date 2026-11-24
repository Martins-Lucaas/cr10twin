#!/usr/bin/env python3
"""Bancada falsa: fala os protocolos do CR10 (TCP 29999/30004) e da FA7155 (Modbus
0x03 numa pty), para rodar o `CR10RealDriver` e o `ft_receiver` de verdade sem hardware.

    python3 scripts/fake_bench.py       # imprime a porta da célula e espera

Não simula dinâmica, tempo de firmware nem colisão: prova o software, não o robô.
"""
import os
import pty
import re
import socket
import struct
import threading
import time
import tty

import numpy as np


def crc16(data: bytes) -> bytes:
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return struct.pack('<H', c)


class FakeCell:
    """Escravo Modbus 1: responde ao 0x03 com 6 float32 (Fx..Mz)."""

    def __init__(self):
        self.master, slave = pty.openpty()
        tty.setraw(self.master)
        tty.setraw(slave)
        self._slave = slave                  # segura a pty aberta entre conexões
        self.port = os.ttyname(slave)
        self.wrench = np.array([1.2, -0.8, 3.5, 0.02, 0.01, 0.0])   # viés de fábrica
        self.mute = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        buf = b''
        while True:
            buf += os.read(self.master, 256)
            while len(buf) >= 8:
                req, buf = buf[:8], buf[8:]
                if crc16(req[:6]) != req[6:] or self.mute:
                    continue
                noisy = self.wrench + np.r_[np.random.normal(0, 0.05, 3),
                                            np.random.normal(0, 0.002, 3)]
                body = bytes([1, 3, 24]) + struct.pack('<6f', *noisy)
                os.write(self.master, body + crc16(body))


class FakeCR10:
    """Controlador: dashboard 29999 e feedback 30004. Ângulos em graus DOBOT."""

    def __init__(self, q_deg, host='127.0.0.1', tau_s=0.05, vmax_deg_s=120.0):
        self.q = np.array(q_deg, float)
        self.target = self.q.copy()
        self.tau, self.vmax = tau_s, vmax_deg_s
        self.mode = 4                        # desabilitado
        self.cmds, self.servo_n, self.max_step = [], 0, 0.0
        self.zero_blip = 0
        self.conns = []
        for port, fn in ((29999, self._dash), (30004, self._feed)):
            s = socket.socket()
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((host, port))
            s.listen(4)
            threading.Thread(target=self._accept, args=(s, fn), daemon=True).start()
        threading.Thread(target=self._move, daemon=True).start()

    def _accept(self, s, fn):
        while True:
            c, _ = s.accept()
            self.conns.append(c)
            threading.Thread(target=fn, args=(c,), daemon=True).start()

    def _move(self):
        while True:
            time.sleep(0.004)
            if self.mode in (5, 7):
                d = (self.target - self.q) * min(0.004 / self.tau, 1.0)
                self.q = self.q + np.clip(d, -self.vmax * 0.004, self.vmax * 0.004)

    def _feed(self, c):
        try:
            while True:
                p = bytearray(1440)
                struct.pack_into('<H', p, 0, 1440)
                struct.pack_into('<Q', p, 24, self.mode)
                blip, self.zero_blip = self.zero_blip > 0, max(0, self.zero_blip - 1)
                struct.pack_into('<6d', p, 432, *(np.zeros(6) if blip else self.q))
                c.sendall(p)
                time.sleep(0.008)
        except OSError:
            pass

    def _dash(self, c):
        buf = b''
        try:
            while True:
                data = c.recv(4096)
                if not data:
                    return
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    cmd = line.decode().strip()
                    if cmd:
                        c.sendall((self._handle(cmd) + '\n').encode())
        except OSError:
            pass

    def _handle(self, cmd):
        name = cmd.split('(')[0]
        if name != 'ServoJ':
            self.cmds.append(cmd)
        code, val = 0, ''
        if name == 'RobotMode':
            val = str(self.mode)
        elif name == 'EnableRobot':
            if self.mode != 9:
                self.mode, self.target = 5, self.q.copy()
        elif name == 'DisableRobot':
            self.mode = 4
        elif name == 'Stop':
            self.target = self.q.copy()
        elif name == 'EmergencyStop':
            self.mode = 9 if '1' in cmd else 4
        elif name in ('ServoJ', 'MovJ'):
            if self.mode == 9:
                code = -2
            elif self.mode not in (5, 7):
                code = -1
            else:
                new = np.array([float(v) for v in re.findall(r'-?\d+\.\d+', cmd)[:6]])
                if name == 'ServoJ':
                    self.servo_n += 1
                    self.max_step = max(self.max_step, float(np.max(np.abs(new - self.target))))
                self.target = new
        return f'{code},{{{val}}},{cmd};'

    def drop(self):
        """Cabo puxado."""
        for c in self.conns:
            try:
                c.shutdown(socket.SHUT_RDWR)
                c.close()
            except OSError:
                pass
        self.conns = []


if __name__ == '__main__':
    cell = FakeCell()
    FakeCR10([20.3, -52.8, -111.1, -106.1, 90.0, 69.7])
    print(f'célula em {cell.port}; CR10 em 127.0.0.1 (29999/30004)', flush=True)
    while True:
        time.sleep(1)
