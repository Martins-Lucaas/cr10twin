"""imu_recorder.py — gravação de uma sessão de rastreio por IMU (aba IMU).

Abre com o Start do rastreio e fecha com o Stop. Uma pasta por sessão:

  sensors/Data/IMU_TRACKING/<AAAAMMDD_HHMMSS>/
    imu.csv     uma linha por mensagem sensor_msgs/Imu RECEBIDA (todas as
                IDs, não só as usadas): orientação, giro, acelerômetro, o
                stamp do imu_node e o instante de chegada na GUI. Os buracos
                entre linhas da mesma ID são os congelamentos.
    robot.csv   uma linha por tick da aba (~30 Hz): pose articular comandada,
                a do sim (/joint_states), alvo e TCP cartesianos, erro da IK,
                flags e se o tick congelou (e por quais IDs).
    real.csv    só com o CR10 conectado: juntas lidas do feedback do braço
                real (~20 Hz, thread própria).
    meta.json   configuração do Start e, no fechamento, o resumo por IMU
                (taxa, maior buraco, nº de buracos > limite de stale).

Todas as linhas levam `t_rel_s` no MESMO relógio (time.monotonic desde o
Start), que é o eixo comum para cruzar os três CSVs, e `t_unix` de parede.

Nada de I/O na thread do Tk nem nos callbacks ROS: os dois só enfileiram;
uma thread escreve.
"""
from __future__ import annotations

import csv
import json
import os
import queue
import threading
import time

import numpy as np

from .constants import ARM_JOINTS, RUNS_DIR, new_run_id

REC_MODE_DIR = 'IMU_TRACKING'
REAL_POLL_S = 0.05
FLUSH_S = 1.0   # se a GUI for morta (SIGKILL), perde-se no máximo isto

IMU_COLS = ('t_rel_s', 't_unix', 'sensor_id', 'stamp_s',
            'qw', 'qx', 'qy', 'qz', 'gx', 'gy', 'gz', 'ax', 'ay', 'az')
_Q = [j[-1] for j in ARM_JOINTS]
ROBOT_COLS = (('t_rel_s', 't_unix', 'frozen', 'stale_ids', 'published')
              + tuple(f'q_cmd{i}_deg' for i in _Q)
              + tuple(f'q_sim{i}_deg' for i in _Q)
              + ('p_cmd_x_mm', 'p_cmd_y_mm', 'p_cmd_z_mm',
                 'tcp_x_mm', 'tcp_y_mm', 'tcp_z_mm',
                 'ik_err_mm', 'v_tcp_mms', 'singular', 'at_limit'))
REAL_COLS = (('t_rel_s', 't_unix') + tuple(f'q_real{i}_deg' for i in _Q)
             + ('read_ms',))


class TrackingRecorder:
    def __init__(self, meta: dict, *, real_reader=None, base_dir: str | None = None,
                 stale_s: float = 0.5):
        self.run_id = new_run_id()
        self.path = os.path.join(base_dir or RUNS_DIR, REC_MODE_DIR, self.run_id)
        os.makedirs(self.path, exist_ok=True)
        self._t0 = time.monotonic()
        self._stale_s = stale_s
        self._meta = dict(meta, run_id=self.run_id, t_start_unix=time.time())
        self._q: queue.SimpleQueue = queue.SimpleQueue()
        self._stop = threading.Event()
        self.rows = {'imu': 0, 'robot': 0, 'real': 0}
        self._imu_t: dict[int, list[float]] = {}

        self._fh, self._w = {}, {}
        for name, cols in (('imu', IMU_COLS), ('robot', ROBOT_COLS)) + (
                (('real', REAL_COLS),) if real_reader else ()):
            fh = open(os.path.join(self.path, f'{name}.csv'), 'w', newline='')
            self._fh[name], self._w[name] = fh, csv.writer(fh)
            self._w[name].writerow(cols)
        self._write_meta()

        self._writer = threading.Thread(target=self._write_loop, daemon=True,
                                        name='imu-rec-writer')
        self._writer.start()
        self._reader = None
        if real_reader is not None:
            self._reader = threading.Thread(target=self._real_loop, args=(real_reader,),
                                            daemon=True, name='imu-rec-real')
            self._reader.start()

    def rel(self, t_mono: float) -> float:
        return t_mono - self._t0

    # ── Produtores (qualquer thread) ─────────────────────────────────
    def imu(self, sensor_id: int, msg, t_mono: float) -> None:
        o, g, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        st = msg.header.stamp
        self._q.put(('imu', (round(self.rel(t_mono), 6), round(time.time(), 6),
                             sensor_id, round(st.sec + st.nanosec * 1e-9, 6),
                             o.w, o.x, o.y, o.z, g.x, g.y, g.z, a.x, a.y, a.z)))

    def robot(self, t_mono: float, *, frozen: bool, stale_ids=(), published=False,
              q_cmd_deg=None, q_sim_rad=None, p_cmd_m=None, tcp_m=None,
              ik_err_mm=None, v_tcp_mms=None, singular=False, at_limit=False) -> None:
        nan6, nan3 = [float('nan')] * 6, [float('nan')] * 3
        q_cmd = list(q_cmd_deg) if q_cmd_deg is not None else nan6
        q_sim = (np.degrees(np.asarray(q_sim_rad[:6], dtype=float)).tolist()
                 if q_sim_rad is not None and len(q_sim_rad) >= 6 else nan6)
        p_cmd = (np.asarray(p_cmd_m) * 1e3).tolist() if p_cmd_m is not None else nan3
        tcp = (np.asarray(tcp_m) * 1e3).tolist() if tcp_m is not None else nan3
        nan = float('nan')
        self._q.put(('robot', (
            round(self.rel(t_mono), 6), round(time.time(), 6), int(frozen),
            ' '.join(map(str, stale_ids)), int(published),
            *(round(v, 4) for v in q_cmd), *(round(v, 4) for v in q_sim),
            *(round(v, 3) for v in p_cmd), *(round(v, 3) for v in tcp),
            nan if ik_err_mm is None else round(ik_err_mm, 3),
            nan if v_tcp_mms is None else round(v_tcp_mms, 2),
            int(singular), int(at_limit))))

    # ── Threads ──────────────────────────────────────────────────────
    def _real_loop(self, read_q_rad) -> None:
        while not self._stop.wait(REAL_POLL_S):
            t = time.monotonic()
            try:
                q = np.degrees(np.asarray(read_q_rad(), dtype=float)[:6]).tolist()
            except Exception:
                q = [float('nan')] * 6   # leitura falhou — fica o buraco no CSV
            ms = (time.monotonic() - t) * 1e3
            self._q.put(('real', (round(self.rel(t), 6), round(time.time(), 6),
                                  *(round(v, 4) for v in q), round(ms, 2))))

    def _write_loop(self) -> None:
        t_flush = time.monotonic()
        while True:
            if time.monotonic() - t_flush >= FLUSH_S:
                for fh in self._fh.values():
                    fh.flush()
                t_flush = time.monotonic()
            try:
                kind, row = self._q.get(timeout=0.2)
            except queue.Empty:
                if self._stop.is_set():
                    return
                continue
            self._w[kind].writerow(row)
            self.rows[kind] += 1
            if kind == 'imu':
                self._imu_t.setdefault(int(row[2]), []).append(row[0])

    # ── Fechamento ───────────────────────────────────────────────────
    def close(self) -> dict:
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=1.0)
        self._writer.join(timeout=5.0)
        for fh in self._fh.values():
            fh.close()
        self._meta['t_end_unix'] = time.time()
        self._meta['duration_s'] = round(self.rel(time.monotonic()), 3)
        self._meta['rows'] = dict(self.rows)
        self._meta['imu_summary'] = self.imu_summary()
        self._write_meta()
        return self._meta

    def imu_summary(self) -> dict:
        """Por ID: amostras, taxa média, maior buraco e quantos passaram do stale."""
        out = {}
        for sid, ts in sorted(self._imu_t.items()):
            gaps = np.diff(ts) if len(ts) > 1 else np.zeros(0)
            span = ts[-1] - ts[0] if len(ts) > 1 else 0.0
            out[str(sid)] = {
                'samples': len(ts),
                'rate_hz': round((len(ts) - 1) / span, 1) if span > 0 else 0.0,
                'max_gap_s': round(float(gaps.max()), 3) if gaps.size else 0.0,
                'gaps_over_stale': int((gaps > self._stale_s).sum()),
            }
        return out

    def _write_meta(self) -> None:
        with open(os.path.join(self.path, 'meta.json'), 'w') as fh:
            json.dump(self._meta, fh, indent=2, default=str)
