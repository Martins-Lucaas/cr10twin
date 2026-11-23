"""cmd.csv (posição COMANDADA) e a resolução de q/tcp no palpation_logger.

Chama os callbacks direto: nada é publicado, em especial nada no tópico do
braço (/cr10_group_controller/joint_trajectory).
"""
import csv
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
os.environ.setdefault('ROS_DOMAIN_ID', '77')

rclpy = pytest.importorskip('rclpy')

from builtin_interfaces.msg import Duration                       # noqa: E402
from sensor_msgs.msg import JointState                            # noqa: E402
from trajectory_msgs.msg import (                                 # noqa: E402
    JointTrajectory, JointTrajectoryPoint)

from touch_pack import palpation_logger as PL                     # noqa: E402
from touch_pack.constants import ARM_JOINTS                       # noqa: E402
from test_logger_columns import _start_msg                        # noqa: E402


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_cmd_csv_e_resolucao(ros, tmp_path, monkeypatch):
    monkeypatch.setattr(PL, 'OUTPUT_DIR', str(tmp_path))
    monkeypatch.setattr(PL.PalpationLogger, '_generate_report',
                        lambda self, path: None)
    lg = PL.PalpationLogger()
    try:
        q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        # Fora de run: comando ignorado, nenhum arquivo.
        tr = JointTrajectory()
        tr.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = q
        pt.time_from_start = Duration(sec=0, nanosec=30_000_000)
        tr.points = [pt, pt]
        lg._cb_arm_cmd(tr)
        assert lg._cmd_writer is None

        lg._cb_start(_start_msg('SLIDE'))
        run_dir = lg._run_dir
        assert run_dir
        lg._cb_arm_cmd(tr)
        lg._cb_arm_cmd(tr)
        # Juntas da mão (outros nomes) não entram.
        hand = JointTrajectory()
        hand.joint_names = ['finger']
        hand.points = [pt]
        lg._cb_arm_cmd(hand)

        js = JointState()
        js.name = list(ARM_JOINTS)
        js.position = q
        lg._cb_joints(js)
        assert all(len(c.split('.')[1]) == 7 for c in lg._tcp_cols + lg._q_cols)

        with lg._lock:
            lg._close_run_locked('DONE')
        rows = list(csv.DictReader(open(os.path.join(run_dir, PL.RUN_CMD_CSV))))
        assert len(rows) == 2
        r = rows[0]
        assert r['n_points'] == '2' and float(r['cmd_tfs_s']) == pytest.approx(0.03)
        assert [float(r[f'cmd_q{i}']) for i in range(1, 7)] == pytest.approx(q)
        # A FK do comando bate com a FK do medido para o mesmo q.
        assert [float(r[c]) for c in ('cmd_tcp_x', 'cmd_tcp_y', 'cmd_tcp_z')] \
            == pytest.approx([float(c) for c in lg._tcp_cols], abs=1e-6)
    finally:
        lg.destroy_node()
