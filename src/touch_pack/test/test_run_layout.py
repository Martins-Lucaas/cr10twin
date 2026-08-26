"""Layout dos dados em disco: uma pasta por MODO, uma por RUN.

Cobre as duas regressões que motivaram a mudança de 12/08/2026:
  • os arquivos de um mesmo run ficavam espalhados numa pasta única, em duas
    convenções de nome (<ts>__samples.csv de um lado, adc_<ts>.csv de outro);
  • o run_id vinha do relógio ROS, que sob use_sim_time começa do zero a cada
    launch — todo run virava 19691231_* e o launch seguinte SOBRESCREVIA a
    coleta anterior.

O caminho de leitura tem de aceitar os DOIS layouts: há coletas antigas em
disco que continuam válidas.
"""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from touch_pack.constants import (                                # noqa: E402
    REC_DIR_NAME, RUN_PARAMS_JSON, RUN_PLOT_PNG, RUN_SAMPLES_CSV,
    RUN_SUMMARY_JSON, new_run_id, run_dir, run_id_from_msg)
from touch_pack.palpation_report import _sibling                  # noqa: E402


class _FakeStamp:
    def __init__(self, sec):
        self.sec = sec


class _FakeStart:
    def __init__(self, run_id='', stamp_sec=0):
        self.run_id = run_id
        self.stamp = _FakeStamp(stamp_sec)


# ── Pasta do run ──────────────────────────────────────────────────────

def test_run_dir_is_mode_then_run_id(tmp_path):
    d = run_dir('MATRIX_MAP', '20260812_143012', base=str(tmp_path))
    assert d == str(tmp_path / 'MATRIX_MAP' / '20260812_143012')
    assert os.path.isdir(d)


def test_every_mode_gets_its_own_folder(tmp_path):
    for mode in ('SLIDE', 'TOUCH', 'MANUAL', 'MATRIX_MAP'):
        run_dir(mode, '20260812_143012', base=str(tmp_path))
    assert sorted(os.listdir(tmp_path)) == [
        'MANUAL', 'MATRIX_MAP', 'SLIDE', 'TOUCH']


def test_modeless_recording_has_a_home_of_its_own(tmp_path):
    """O botão "Record data" grava fora de qualquer run: sem modo, mas
    também sem cair na raiz junto das pastas de modo."""
    d = run_dir('', '20260812_143012', base=str(tmp_path))
    assert os.path.basename(os.path.dirname(d)) == REC_DIR_NAME


def test_unknown_mode_does_not_create_a_folder_of_its_own(tmp_path):
    d = run_dir('LIXO', '20260812_143012', base=str(tmp_path))
    assert os.path.basename(os.path.dirname(d)) == REC_DIR_NAME


def test_run_id_from_a_message_cannot_escape_the_data_dir(tmp_path):
    """mode/run_id vêm de mensagem: `ros2 topic pub` pode mandar qualquer
    string, e ela vira nome de diretório."""
    d = run_dir('../../etc', '../../../tmp/evil', base=str(tmp_path))
    assert os.path.commonpath([str(tmp_path), os.path.abspath(d)]) == \
        str(tmp_path)


def test_run_id_is_wall_clock_not_the_ros_clock():
    """O bug de origem: com use_sim_time o carimbo ROS reinicia do zero a
    cada launch, e dois runs de sessões diferentes disputavam o nome."""
    rid = new_run_id()
    assert len(rid) == len('20260812_143012') and rid[8] == '_'
    assert int(rid[:4]) >= 2020


def test_message_run_id_wins_over_the_stamp():
    assert run_id_from_msg(_FakeStart('20260812_143012', 4496)) == \
        '20260812_143012'


def test_old_publisher_without_run_id_still_names_the_run():
    """Compatibilidade: `ros2 topic pub` sem o campo cai no carimbo."""
    rid = run_id_from_msg(_FakeStart('', 1_755_000_000))
    assert len(rid) == len('20260812_143012')


# ── Leitura: os dois layouts ──────────────────────────────────────────

def test_report_siblings_live_in_the_run_folder(tmp_path):
    csv_path = str(tmp_path / 'TOUCH' / '20260812_143012' / RUN_SAMPLES_CSV)
    assert _sibling(csv_path, RUN_SUMMARY_JSON, '__summary.json') == \
        str(tmp_path / 'TOUCH' / '20260812_143012' / RUN_SUMMARY_JSON)
    assert _sibling(csv_path, RUN_PLOT_PNG, '__plot.png').endswith(
        os.path.join('20260812_143012', RUN_PLOT_PNG))


def test_report_still_reads_the_old_flat_layout(tmp_path):
    """Um run antigo passado na linha de comando continua gerando os
    derivados com o nome dele, ao lado dele."""
    csv_path = str(tmp_path / '20260611_101010__samples.csv')
    assert _sibling(csv_path, RUN_PARAMS_JSON, '__params.json') == \
        str(tmp_path / '20260611_101010__params.json')


# ── Fim a fim: o logger grava onde os leitores procuram ───────────────

def test_logger_writes_the_whole_run_into_one_folder(tmp_path, monkeypatch):
    """samples.csv e params.json na MESMA pasta — é o ponto da mudança."""
    pytest.importorskip('rclpy')
    os.environ.setdefault('ROS_DOMAIN_ID', '77')
    import rclpy
    from touch_pack import palpation_logger as PL
    from touch_pack_msgs.msg import PalpationStart

    monkeypatch.setattr(PL, 'OUTPUT_DIR', str(tmp_path))
    rclpy.init()
    try:
        logger = PL.PalpationLogger()
        msg = PalpationStart()
        msg.mode = 'MATRIX_MAP'
        msg.run_id = '20260812_143012'
        msg.force_n = 1.5
        logger._cb_start(msg)
        logger.close()
        logger.destroy_node()
    finally:
        rclpy.shutdown()

    d = tmp_path / 'MATRIX_MAP' / '20260812_143012'
    assert (d / RUN_SAMPLES_CSV).is_file()
    assert (d / RUN_PARAMS_JSON).is_file()
    # params.json carrega o run_id: um arquivo levado para fora da pasta
    # ainda se identifica.
    with open(d / RUN_PARAMS_JSON) as fh:
        assert json.load(fh)['run_id'] == '20260812_143012'
