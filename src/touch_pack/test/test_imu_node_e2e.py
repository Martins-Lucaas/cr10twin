"""imu_node de ponta a ponta, contra ESPs emulados (test/esp_emulator.py).

Sobe dois ESPs em software (um parado, um girando em yaw), o imu_node num
subprocesso e confere, por /imu/sensor_<id>:
  * descoberta multicast → TCP → RUN: mensagens chegam a ~100 Hz;
  * conversões: aceleração em m/s², giro em rad/s;
  * a orientação integra o giro (yaw acompanha o emulador);
  * ao receber SIGINT o nó sai limpo e devolve os sensores a HOLD;
  * com o parâmetro iface_ip, acha ESPs numa interface que NÃO é a que o
    WBAN escolheria sozinho (o caso cabo + Wi-Fi da bancada).

Usa um ROS_DOMAIN_ID próprio, um calibration.json vazio e IDs 8/9, que
não existem na bancada (os ESPs reais são 1–6). Atenção: o nó também acha os
ESPs REAIS que estiverem ligados na mesma rede, e os põe em RUN e depois em
HOLD durante o teste — inofensivo, mas é tráfego real. Pula se não houver
o WBAN-Inercial, o PyQt6/ahrs no python do ROS, ou interface de rede
não-loopback (o protocolo é multicast).
"""
import math
import os
import random
import signal
import subprocess
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from esp_emulator import EspEmulator, still, yaw_after  # noqa: E402

from touch_pack.imu_node import DEFAULT_WBAN_DIR  # noqa: E402

pytest.importorskip('PyQt6.QtCore')
pytest.importorskip('ahrs')
if not os.path.isdir(os.path.join(DEFAULT_WBAN_DIR, 'core')):
    pytest.skip('WBAN-Inercial não encontrado', allow_module_level=True)

YAW_DEG_S = 20.0
YAW_START_S = 4.0
RUN_S = 12.0


@pytest.fixture
def domain(monkeypatch):
    d = str(random.randint(60, 99))
    monkeypatch.setenv('ROS_DOMAIN_ID', d)
    return d


def _iface_ip():
    sys.path.insert(0, DEFAULT_WBAN_DIR)
    from core.utils import get_default_ip
    ip = get_default_ip()
    if ip.startswith('127.') or ip == '0.0.0.0':
        pytest.skip('sem interface de rede para multicast')
    return ip


def _other_iface_ip(default_ip):
    """Um IPv4 local não-loopback diferente do escolhido pelo WBAN, ou None."""
    out = subprocess.run(['ip', '-4', '-o', 'addr', 'show'], capture_output=True,
                         text=True).stdout
    for line in out.splitlines():
        ip = line.split()[3].split('/')[0]
        if not ip.startswith('127.') and ip != default_ip:
            return ip
    return None


@pytest.mark.parametrize('explicit_iface', [False, True],
                         ids=['iface-automatica', 'iface_ip-explicito'])
def test_imu_node_streams_emulated_esps(domain, tmp_path, explicit_iface):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    from touch_pack.lc_filter import QOS_SENSOR

    ip = _iface_ip()
    extra = []
    if explicit_iface:
        ip = _other_iface_ip(ip)
        if ip is None:
            pytest.skip('só há uma interface de rede')
        extra = ['-p', f'iface_ip:={ip}']
    calib = tmp_path / 'calibration.json'
    calib.write_text('{}')
    emus = [EspEmulator(8, ip, still), EspEmulator(9, ip, yaw_after(YAW_START_S, YAW_DEG_S))]
    for e in emus:
        e.start()
    log = open(tmp_path / 'imu_node.log', 'w')
    proc = subprocess.Popen(
        [sys.executable, '-c', 'from touch_pack.imu_node import main; main()',
         '--ros-args', '-p', f'calibration_file:={calib}', *extra],
        stdout=log, stderr=subprocess.STDOUT)
    got = {8: [], 9: []}
    rclpy.init()
    try:
        node = Node('imu_node_e2e_check')
        for sid in got:
            node.create_subscription(
                Imu, f'/imu/sensor_{sid}',
                lambda m, s=sid: got[s].append((time.monotonic(), m)), QOS_SENSOR)
        t_end = time.monotonic() + RUN_S
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
        t_stop = time.monotonic() - emus[1]._t0
        proc.send_signal(signal.SIGINT)
        rc = proc.wait(timeout=15)
        node.destroy_node()
    finally:
        rclpy.shutdown()
        if proc.poll() is None:
            proc.kill()
        time.sleep(1.0)   # o HOLD sai no shutdown; dá tempo do emulador ler
        states = {e.sensor_id: e.state for e in emus}
        for e in emus:
            e.stop()
        log.close()

    out = (tmp_path / 'imu_node.log').read_text()
    assert rc == 0, out[-2000:]
    assert states == {8: 1, 9: 1}, 'sensores não voltaram a HOLD'
    for sid, msgs in got.items():
        assert len(msgs) > 300, f'sensor {sid}: só {len(msgs)} mensagens\n{out[-2000:]}'
        span = msgs[-1][0] - msgs[0][0]
        assert len(msgs) / span == pytest.approx(100.0, rel=0.15)
        m = msgs[-1][1]
        assert m.header.frame_id == f'imu_{sid}'
        assert m.linear_acceleration.z == pytest.approx(9.74, abs=0.1)

    assert got[8][-1][1].angular_velocity.z == pytest.approx(0.0, abs=1e-6)
    assert got[9][-1][1].angular_velocity.z == pytest.approx(math.radians(YAW_DEG_S), rel=0.02)
    o = got[9][-1][1].orientation
    yaw = math.degrees(math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y**2 + o.z**2)))
    expected = YAW_DEG_S * (t_stop - YAW_START_S)
    err = (yaw - expected + 180.0) % 360.0 - 180.0
    # a última amostra é de um pouco antes do SIGINT: tolera ~1 s de giro
    assert abs(err) < YAW_DEG_S * 1.0
