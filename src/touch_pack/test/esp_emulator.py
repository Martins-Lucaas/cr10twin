"""esp_emulator.py — ESP do WBAN-Inercial em software, para testar o imu_node.

Fala o protocolo do firmware (FirmwareSensorInercial/src/main.cpp):
  * anuncia o próprio estado por multicast em 230.0.0.1:50000 (2 Hz);
  * escuta comandos de configuração em 230.0.0.2:50000;
  * serve TCP em 50000+id e, em RUN, manda quadros AGM na taxa pedida.

O movimento é dado por `motion(t) -> (R, omega_body_rad_s)`: R é a rotação
sensor → Terra (z cima) e omega a velocidade angular no frame do sensor.
Daí saem acelerômetro (Rᵀ·g) e giroscópio consistentes, em LSB, nas mesmas
escalas que o FormatData do WBAN converte (±8 g, ±2000 °/s).

Uso manual (sem ESP real; precisa de uma interface de rede não-loopback):
    python3 esp_emulator.py --wban-dir "<.../Sensores Inerciais Nova Interface>" --ids 3 6
"""
from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import threading
import time

import numpy as np

ANNOUNCE_GROUP = '230.0.0.1'
CONFIG_GROUP = '230.0.0.2'
PORT = 50000
RATES_HZ = (500, 400, 300, 200, 100, 1)
CMD_ANNOUNCE, CMD_DATA_AGM = 0, 3
ANNOUNCE_FMT = '>BBBBBBBL'
AGM_FMT = '>BBBLLhhhhhhhhh'
ACC_LSB_PER_G = 32768 / 8.0
GYR_LSB_PER_DPS = 32768 / 2000.0
STATE_HOLD, STATE_RUN = 1, 2


def still(_t):
    return np.eye(3), np.zeros(3)


def yaw_after(t_start: float, deg_s: float):
    """Parado até t_start; depois gira em torno do z da Terra a deg_s."""
    def motion(t):
        if t < t_start:
            return np.eye(3), np.zeros(3)
        a = math.radians(deg_s * (t - t_start))
        c, s = math.cos(a), math.sin(a)
        R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        # z da Terra coincide com o z do sensor quando só há yaw
        return R, np.array([0.0, 0.0, math.radians(deg_s)])
    return motion


class EspEmulator:
    def __init__(self, sensor_id: int, iface_ip: str, motion=still):
        self.sensor_id = sensor_id
        self.iface_ip = iface_ip
        self.motion = motion
        self.state = STATE_HOLD
        self.rate_id = 4
        self.axis = 2
        self.acc_fsr, self.gyr_fsr = 2, 3
        self.configs_received = 0
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._t0 = time.monotonic()

    # ── ciclo de vida ────────────────────────────────────────────────
    def start(self) -> None:
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.iface_ip, PORT + self.sensor_id))
        self._srv.listen(1)
        self._srv.settimeout(0.2)

        self._cfg = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._cfg.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._cfg.bind((CONFIG_GROUP, PORT))   # só o tráfego do grupo de config
        self._cfg.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                             socket.inet_aton(CONFIG_GROUP) + socket.inet_aton(self.iface_ip))
        self._cfg.settimeout(0.2)

        self._ann = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._ann.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                             socket.inet_aton(self.iface_ip))
        self._ann.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)

        for fn in (self._announce_loop, self._config_loop, self._tcp_loop):
            t = threading.Thread(target=fn, daemon=True)
            t.start()
            self._threads.append(t)

    def stop(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)
        for s in (self._srv, self._cfg, self._ann):
            s.close()

    # ── laços ────────────────────────────────────────────────────────
    def _announce_loop(self) -> None:
        cnt = 0
        while not self._stop.is_set():
            pkt = struct.pack(ANNOUNCE_FMT, CMD_ANNOUNCE, self.sensor_id, self.state,
                              self.rate_id, self.axis, self.acc_fsr, self.gyr_fsr, cnt)
            self._ann.sendto(pkt, (ANNOUNCE_GROUP, PORT))
            cnt += 1
            self._stop.wait(0.5)

    def _config_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._cfg.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) != struct.calcsize(ANNOUNCE_FMT) or data[0] != CMD_ANNOUNCE:
                continue
            _c, sid, state, rate, axis, acc, gyr, _cnt = struct.unpack(ANNOUNCE_FMT, data)
            if sid not in (self.sensor_id, 0xFF):
                continue
            self.state, self.rate_id, self.axis = state, rate, axis
            self.acc_fsr, self.gyr_fsr = acc, gyr
            self.configs_received += 1

    def _tcp_loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self._stream(conn)

    def _stream(self, conn: socket.socket) -> None:
        index = 0
        next_t = time.monotonic()
        try:
            while not self._stop.is_set():
                if self.state != STATE_RUN:
                    time.sleep(0.02)
                    next_t = time.monotonic()
                    continue
                period = 1.0 / RATES_HZ[self.rate_id]
                now = time.monotonic()
                if now < next_t:
                    time.sleep(next_t - now)
                next_t += period
                t = time.monotonic() - self._t0
                R, omega = self.motion(t)
                acc_g = R.T @ np.array([0.0, 0.0, 1.0])
                gyr_dps = np.degrees(omega)
                ax, ay, az = (int(round(v * ACC_LSB_PER_G)) for v in acc_g)
                gx, gy, gz = (int(round(v * GYR_LSB_PER_DPS)) for v in gyr_dps)
                pkt = struct.pack(AGM_FMT, CMD_DATA_AGM, self.sensor_id, self.state,
                                  index, int(t * 1e6) & 0xFFFFFFFF,
                                  ax, ay, az, gx, gy, gz, 0, 0, 0)
                conn.sendall(pkt)
                index += 1
        except OSError:
            pass
        finally:
            conn.close()


def default_iface_ip(wban_dir: str) -> str:
    sys.path.insert(0, wban_dir)
    from core.utils import get_default_ip
    return get_default_ip()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--wban-dir', required=True)
    ap.add_argument('--ids', type=int, nargs='+', default=[6])
    ap.add_argument('--yaw-deg-s', type=float, default=15.0,
                    help='velocidade de yaw depois de 3 s parado (0 = parado)')
    args = ap.parse_args()
    ip = default_iface_ip(args.wban_dir)
    emus = [EspEmulator(i, ip, yaw_after(3.0, args.yaw_deg_s) if args.yaw_deg_s else still)
            for i in args.ids]
    for e in emus:
        e.start()
    print(f'emulando sensores {args.ids} em {ip} (Ctrl+C para sair)')
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    for e in emus:
        e.stop()


if __name__ == '__main__':
    main()
