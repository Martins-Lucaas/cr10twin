"""Run de APRENDIZADO da home não é gravado.

Com a home sem contato aprendido a descida inteira rasteja só para memorizar a
profundidade (8 min no run SLIDE/20261021_113235) — não é dado de ensaio. O
explorer avisa o run_id em LEARNING_RUN_TOPIC; o aviso pode chegar antes ou
depois do /palpation/start, e os dois casos têm de dar em nada gravado.
"""
import os
import types

import pytest

pytest.importorskip('rclpy')
os.environ.setdefault('ROS_DOMAIN_ID', '77')
import rclpy                                            # noqa: E402
from std_msgs.msg import String                         # noqa: E402
from touch_pack_msgs.msg import PalpationStart          # noqa: E402

RID = '20261021_130000'


@pytest.fixture()
def logger(tmp_path, monkeypatch):
    from touch_pack import palpation_logger as PL
    monkeypatch.setattr(PL, 'OUTPUT_DIR', str(tmp_path))
    rclpy.init()
    lg = PL.PalpationLogger()
    yield lg
    lg.close()
    lg.destroy_node()
    rclpy.shutdown()


def _start():
    m = PalpationStart()
    m.mode, m.run_id, m.force_n = 'SLIDE', RID, 1.0
    return m


def _arquivos(tmp_path):
    return sorted(p for p in tmp_path.rglob('*') if p.is_file())


def test_aviso_antes_do_start_nao_abre_o_run(logger, tmp_path):
    logger._cb_learning_run(String(data=RID))
    logger._cb_start(_start())
    assert _arquivos(tmp_path) == []


def test_aviso_depois_do_start_apaga_o_run(logger, tmp_path):
    logger._cb_start(_start())
    assert _arquivos(tmp_path), 'o start deveria ter aberto o run'
    logger._cb_learning_run(String(data=RID))
    assert _arquivos(tmp_path) == []
    assert not (tmp_path / 'touch_5x5' / 'SLIDE' / RID).exists()


def test_aviso_de_outro_run_nao_toca_no_run_aberto(logger, tmp_path):
    logger._cb_start(_start())
    antes = _arquivos(tmp_path)
    logger._cb_learning_run(String(data='20200101_000000'))
    assert _arquivos(tmp_path) == antes


def test_explorer_avisa_so_quando_a_home_nao_e_aprendida():
    from touch_pack.tactile_explorer import TactileExplorer
    rclpy.init()
    try:
        n = TactileExplorer()
        pub = []
        n._learning_run_pub = types.SimpleNamespace(publish=pub.append)
        n._start_is_stale = lambda m: False

        def start(learned):
            def fake(msg):
                n._learned_contact_m = learned
                return True
            n._start_from_msg = fake
            n._busy.clear()
            n._cb_start(_start())
        start(None)
        start(0.0248)
        assert [m.data for m in pub] == [RID]
        n.destroy_node()
    finally:
        rclpy.shutdown()


def test_gui_descarta_so_a_gravacao_automatica_do_run(tmp_path):
    from touch_pack.palpation_gui import PalpationGUI
    import threading
    gui = PalpationGUI.__new__(PalpationGUI)
    gui._lock = threading.Lock()
    gui.root = types.SimpleNamespace(after=lambda _ms, f, *a: f(*a),
                                     after_cancel=lambda _id: None)
    gui._rec_after = None
    gui.rec_btn = types.SimpleNamespace(config=lambda **k: None)
    status = []
    gui._set_rec_status = lambda t, c: status.append(t)
    d = tmp_path / 'touch_4x4' / 'SLIDE' / RID
    d.mkdir(parents=True)
    for name in ('sensors.csv', 'adc.csv', 'spikes.csv', 'cuneiformes.csv'):
        (d / name).write_text('h\n')

    def abre(auto):
        gui._rec_fh = open(d / 'sensors.csv', 'a')
        gui._rec_writer = object()
        gui._rec_path = str(d / 'sensors.csv')
        gui._rec_count = 0
        gui._rec_auto, gui._rec_run_id = auto, (RID if auto else None)
        gui._ref_adc_fh = gui._ref_spike_fh = gui._ref_cn_fh = None
        gui._ref_adc_writer = gui._ref_spike_writer = gui._ref_cn_writer = None

    abre(auto=False)                       # gravação MANUAL: intocada
    gui._cb_learning_run(String(data=RID))
    assert (d / 'sensors.csv').exists() and gui._rec_fh is not None
    gui._rec_fh.close()

    abre(auto=True)
    gui._cb_learning_run(String(data=RID))
    assert gui._rec_fh is None
    assert not d.exists()
    assert 'not recorded' in status[-1]
