"""Gravação do palpation_logger: cabeçalho por modo + carimbos de hardware.

Cobre as regressões que motivaram a mudança de 10/08/2026:
  • grade do sensor vinda do parâmetro `sensor` (com o default 4×4 antigo, o
    logger descartava 100% dos frames de um sensor 5×5 e o run saía sem tátil);
  • seq/t_us dos DOIS firmwares chegando ao CSV (antes toda coluna de tempo era
    hora de chegada, depois de 2 saltos ROS);
  • tensão CRUA da célula na linha, para refazer a força sem o atraso do filtro;
  • pose_age_ms, simétrico ao taxel_age_ms;
  • bloco de colunas específico de cada experimento — uma senoide não tem alvo
    único, tem excursão min/max e frequência.
"""
import csv
import glob
import os
import sys
import tempfile
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Domínio ROS isolado, fixado ANTES de qualquer rclpy.init(): com a bancada
# ligada (ros2 launch tactile_cell...) o /palpation/start é TRANSIENT_LOCAL e
# o logger de teste recebia na hora o comando latched do stack real, abortando
# o run sintético com 'superseded'. Isto vale para todos os testes ROS do
# pacote, não só os deste arquivo.
os.environ.setdefault('ROS_DOMAIN_ID', '77')

rclpy = pytest.importorskip('rclpy')

from std_msgs.msg import String                                   # noqa: E402
from sensor_msgs.msg import JointState                            # noqa: E402
from touch_pack_msgs.msg import (                                 # noqa: E402
    PalpationStart, PalpationStatus, LoadCellSample, TouchFrame)

from touch_pack import palpation_logger as PL                     # noqa: E402
from touch_pack.constants import (                                # noqa: E402
    ARM_JOINTS, TOUCH_FRAME_TOPIC, RUN_SAMPLES_CSV)


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _spin(nodes, seconds=0.35):
    """Gira todos os nós por `seconds` — o logger tem callbacks em vários
    tópicos e precisa processá-los na ordem em que chegam."""
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=0.005)


def _start_msg(mode, **kw):
    m = PalpationStart()
    m.mode = mode
    m.force_n = 1.5
    m.slide_dir = '+Y'
    m.slide_dist_mm = 50.0
    m.slide_slope_deg = 0.0
    m.speed_mms = 20.0
    m.stamp.sec = int(time.time())
    for k, v in kw.items():
        setattr(m, k, v)
    return m


def _run_once(tmpdir, monkeypatch, mode, sensor='5', n_taxels=25, **start_kw):
    """Executa um run completo contra um logger real e devolve as linhas do
    __samples.csv resultante."""
    monkeypatch.setattr(PL, 'OUTPUT_DIR', tmpdir)
    # O relatório pós-run roda numa thread e não é o objeto deste teste.
    monkeypatch.setattr(PL.PalpationLogger, '_generate_report',
                        lambda self, path: None)

    logger = PL.PalpationLogger()
    # `sensor` é lido no __init__; aqui aplicamos a MESMA derivação que ele faz,
    # para exercitar o resto do caminho com a grade escolhida. O default do
    # parâmetro em si é coberto por test_default_do_parametro_sensor.
    logger._rows = 4 if sensor == '4' else 5
    logger._cols = logger._rows
    logger._n_taxels = logger._rows * logger._cols
    logger._adc_cols = [''] * logger._n_taxels

    pub = rclpy.create_node('test_pub')
    p_start = pub.create_publisher(PalpationStart, '/palpation/start',
                                   PL._QOS_COMMAND)
    p_status = pub.create_publisher(PalpationStatus, '/palpation/status', 10)
    p_lc = pub.create_publisher(LoadCellSample, '/load_cell/sample_net',
                                PL._QOS_SENSOR)
    p_touch = pub.create_publisher(TouchFrame, TOUCH_FRAME_TOPIC,
                                   PL._QOS_SENSOR)
    p_evt = pub.create_publisher(String, '/touch_sensor/spike_event',
                                 PL._QOS_SENSOR)
    p_js = pub.create_publisher(JointState, '/joint_states', 50)
    nodes = [logger, pub]

    p_start.publish(_start_msg(mode, **start_kw))
    _spin(nodes)

    js = JointState()
    js.name = list(ARM_JOINTS)
    js.position = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    p_js.publish(js)

    st = PalpationStatus()
    st.phase = 'HOLD'
    st.cycle = 1
    st.target_force_n = 2.75
    if mode == 'MATRIX_MAP':
        st.wp_index = 3
        st.wp_x_mm = 12.0
        st.wp_y_mm = -4.0
    p_status.publish(st)

    tf = TouchFrame()
    tf.taxels = list(range(n_taxels))
    tf.t_us = 123456
    tf.rows = logger._rows
    tf.cols = logger._cols
    p_touch.publish(tf)
    p_evt.publish(String(data='RA'))
    _spin(nodes)

    lc = LoadCellSample()
    lc.seq = 4242
    lc.t_us = 888777666
    lc.voltage_raw = -0.0001234
    lc.voltage = -0.0001200
    lc.force_net_n = 1.61
    lc.calibrated = True
    p_lc.publish(lc)
    _spin(nodes)

    done = PalpationStatus()
    done.phase = 'DONE'
    p_status.publish(done)
    _spin(nodes)

    logger.close()
    # Layout em disco: <tmpdir>/<MODO>/<run_id>/samples.csv.
    found = glob.glob(os.path.join(tmpdir, '*', '*', RUN_SAMPLES_CSV))
    assert found, f'nenhum {RUN_SAMPLES_CSV} foi criado em {tmpdir}'
    with open(found[0]) as fh:
        rows = list(csv.DictReader(fh))
    pub.destroy_node()
    logger.destroy_node()
    return rows


@pytest.fixture
def tmpdir_runs():
    with tempfile.TemporaryDirectory() as d:
        yield d


def test_carimbos_de_hardware_chegam_ao_csv(ros, tmpdir_runs, monkeypatch):
    """seq/t_us dos dois firmwares + tensão crua, na mesma linha."""
    rows = _run_once(tmpdir_runs, monkeypatch, 'SLIDE')
    assert rows, 'o run não gravou nenhuma linha'
    r = rows[-1]
    assert int(r['lc_seq']) == 4242
    assert int(r['lc_t_us']) == 888777666
    assert float(r['lc_voltage_raw_v']) == pytest.approx(-0.0001234, abs=1e-9)
    assert int(r['touch_t_us']) == 123456
    assert float(r['force_net_n']) == pytest.approx(1.61)
    # setpoint_n vem do status (instantâneo), não do force_n do start.
    assert float(r['setpoint_n']) == pytest.approx(2.75)


def test_idade_da_pose_e_do_toque_sao_auditaveis(ros, tmpdir_runs, monkeypatch):
    rows = _run_once(tmpdir_runs, monkeypatch, 'SLIDE')
    r = rows[-1]
    # Ambas preenchidas e plausíveis — o run inteiro dura menos de 5 s.
    assert 0.0 <= float(r['pose_age_ms']) < 5000.0
    assert 0.0 <= float(r['taxel_age_ms']) < 5000.0
    assert r['taxel_0'] == '0' and r['taxel_24'] == '24'


def test_senoide_grava_excursao_e_frequencia(ros, tmpdir_runs, monkeypatch):
    """O ensaio senoidal não tem alvo único: min/max/hz precisam estar no CSV."""
    rows = _run_once(
        tmpdir_runs, monkeypatch, 'TOUCH',
        force_mod_shape='SINE', force_mod_min_n=0.5, force_mod_max_n=3.5,
        force_mod_hz=2.0, force_mod_cycles=30)
    r = rows[-1]
    assert r['mode'] == 'TOUCH'
    assert r['mod_shape'] == 'SINE'
    assert float(r['mod_min_n']) == pytest.approx(0.5)
    assert float(r['mod_max_n']) == pytest.approx(3.5)
    assert float(r['mod_hz']) == pytest.approx(2.0)
    assert int(r['mod_cycles']) == 30
    # Sem bloco de outro modo contaminando.
    assert 'step_size_n' not in r and 'slide_dir' not in r


def test_touch_sem_modulacao_nao_ganha_bloco(ros, tmpdir_runs, monkeypatch):
    rows = _run_once(tmpdir_runs, monkeypatch, 'TOUCH', force_mod_shape='OFF')
    r = rows[-1]
    assert r['mode'] == 'TOUCH'
    assert 'mod_shape' not in r


def test_escada_grava_os_patamares(ros, tmpdir_runs, monkeypatch):
    rows = _run_once(
        tmpdir_runs, monkeypatch, 'MANUAL',
        step_start_n=1.0, step_size_n=1.0, step_max_n=5.0, step_dwell_s=3.0)
    r = rows[-1]
    assert r['mode'] == 'MANUAL'
    assert float(r['step_size_n']) == pytest.approx(1.0)
    assert float(r['step_max_n']) == pytest.approx(5.0)


def test_matrix_map_grava_o_ponto_da_grade(ros, tmpdir_runs, monkeypatch):
    rows = _run_once(tmpdir_runs, monkeypatch, 'MATRIX_MAP',
                     grid_shape='RECT')
    r = rows[-1]
    assert r['mode'] == 'MATRIX_MAP'
    assert int(r['wp_index']) == 3
    assert float(r['wp_x_mm']) == pytest.approx(12.0)
    assert r['grid_shape'] == 'RECT'


def test_slide_grava_a_geometria_do_deslize(ros, tmpdir_runs, monkeypatch):
    rows = _run_once(tmpdir_runs, monkeypatch, 'SLIDE')
    r = rows[-1]
    assert r['mode'] == 'SLIDE'
    assert r['slide_dir'] == '+Y'
    assert float(r['slide_dist_mm']) == pytest.approx(50.0)


def test_default_do_parametro_sensor(ros, tmpdir_runs, monkeypatch):
    """O default do logger é 5×5 — a grade montada na bancada. Com o antigo
    default 4×4 contra um sensor 5×5 ele descartava TODOS os frames."""
    monkeypatch.setattr(PL, 'OUTPUT_DIR', tmpdir_runs)
    logger = PL.PalpationLogger()
    try:
        assert logger.get_parameter('sensor').value == '5'
        assert (logger._rows, logger._cols, logger._n_taxels) == (5, 5, 25)
    finally:
        logger.destroy_node()


def test_grade_4x4_nao_e_mais_descartada(ros, tmpdir_runs, monkeypatch):
    """Com sensor='4' o logger deve ACEITAR 16 taxels — antes exigia 25 fixo
    e o run inteiro saía com taxel_* vazios."""
    rows = _run_once(tmpdir_runs, monkeypatch, 'SLIDE',
                     sensor='4', n_taxels=16)
    r = rows[-1]
    assert 'taxel_15' in r and 'taxel_16' not in r
    assert r['taxel_15'] == '15'
    assert r['taxel_age_ms'] != ''
