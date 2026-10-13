"""TrackingRecorder: thread do CR10 real e falha de leitura viram linhas."""
import csv
import math
import os
import time

from touch_pack.imu_recorder import REAL_COLS, TrackingRecorder


def test_real_joints_are_polled_and_failures_become_nan(tmp_path):
    calls = {'n': 0}

    def reader():
        calls['n'] += 1
        if calls['n'] == 2:
            raise RuntimeError('feed caiu')
        return [math.radians(10.0)] * 6

    rec = TrackingRecorder({'x': 1}, real_reader=reader, base_dir=str(tmp_path))
    time.sleep(0.3)
    meta = rec.close()
    with open(os.path.join(rec.path, 'real.csv')) as fh:
        rows = list(csv.DictReader(fh))
    assert list(rows[0]) == list(REAL_COLS)
    assert len(rows) >= 3 and meta['rows']['real'] == len(rows)
    assert float(rows[0]['q_real1_deg']) == 10.0
    assert math.isnan(float(rows[1]['q_real1_deg']))
