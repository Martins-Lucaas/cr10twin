"""Dados e periféricos: logger, sensors.csv, eventos, câmera, IMU, mão, IK da
viewport, célula, probes e relatórios. Só código de produção, sem hardware.
"""
import csv
import importlib.util
import math
import os
import sys
import threading
import time
import types

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
tk = pytest.importorskip('tkinter')
pytest.importorskip('serial')

sys.path.insert(0, os.path.dirname(__file__))

from std_msgs.msg import String                                # noqa: E402
from touch_pack_msgs.msg import PalpationStatus                # noqa: E402

import touch_pack.constants as C                               # noqa: E402
import touch_pack.gui_camera as gui_camera                     # noqa: E402
import touch_pack.palpation_gui as gui                         # noqa: E402
from touch_pack import palpation_logger as PL                  # noqa: E402
from touch_pack.touch_source import TouchSensorSource          # noqa: E402
from test_logger_columns import _start_msg                     # noqa: E402

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))


def _script(nome):
    spec = importlib.util.spec_from_file_location(
        '_' + nome, os.path.join(ROOT, 'scripts', nome + '.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def ros():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def lg(ros, tmp_path, monkeypatch):
    monkeypatch.setattr(PL, 'OUTPUT_DIR', str(tmp_path))
    monkeypatch.setattr(PL.PalpationLogger, '_generate_report',
                        lambda self, path: None)
    n = PL.PalpationLogger()
    yield n
    n.close()
    n.destroy_node()


# ══ logger ════════════════════════════════════════════════════════════
def _status(fase, run_id=''):
    m = PalpationStatus()
    m.phase = fase
    if hasattr(m, 'run_id'):
        m.run_id = run_id
    return m


def test_fim_atrasado_do_run_anterior_nao_fecha_o_novo(lg):
    """T28."""
    lg._cb_start(_start_msg('SLIDE'))
    lg._cb_status(_status('ABORTED', '20200101_000000'))
    assert lg._csv_fh is not None
    lg._cb_status(_status('ABORTED', lg._run_stamp))
    assert lg._csv_fh is None


def test_start_latched_repetido_nao_trunca_o_run(lg):
    """T29: 5403 -> 403 bytes."""
    msg = _start_msg('SLIDE')
    lg._cb_start(msg)
    path = os.path.join(lg._run_dir, PL.RUN_SAMPLES_CSV)
    lg._csv_fh.write('dado do run\n')
    with lg._lock:
        lg._close_run_locked('DONE')
    antes = open(path).read()
    lg._cb_start(msg)
    assert open(path).read() == antes and lg._csv_fh is None


def test_rajada_de_spikes_chega_inteira(lg):
    """T13: best-effort/depth 1 entregava 50 de 500."""
    qos = getattr(C, 'TOUCH_EVENT_QOS_DEPTH', gui.QOS_SENSOR)
    tx = rclpy.create_node('tx_spikes')
    pub = tx.create_publisher(String, C.TOUCH_EVENT_TOPIC, qos)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(lg)
    t0 = time.monotonic()
    while pub.get_subscription_count() < 1 and time.monotonic() - t0 < 5:
        time.sleep(0.02)
    tipo = PL.TOUCH_EVENT_TYPES[0]
    for _ in range(50):
        for _ in range(10):
            pub.publish(String(data=tipo))
        t1 = time.monotonic()
        while time.monotonic() - t1 < 0.01:
            ex.spin_once(timeout_sec=0.0)
    t1 = time.monotonic()
    while lg._evt_counts[tipo] < 500 and time.monotonic() - t1 < 3:
        ex.spin_once(timeout_sec=0.01)
    ex.remove_node(lg)
    tx.destroy_node()
    assert lg._evt_counts[tipo] == 500


def test_fechar_o_logger_espera_o_relatorio(ros, tmp_path, monkeypatch):
    """XT25b: a thread daemon morria com o processo e o summary se perdia."""
    feito = []

    def _rel(self, path):
        time.sleep(0.3)
        feito.append(path)
    monkeypatch.setattr(PL, 'OUTPUT_DIR', str(tmp_path))
    monkeypatch.setattr(PL.PalpationLogger, '_generate_report', _rel)
    n = PL.PalpationLogger()
    n._cb_start(_start_msg('SLIDE'))
    n._sample_count = 1
    n.close()
    n.destroy_node()
    assert feito


def test_logger_reserva_sobe_com_o_sensor_da_gui(monkeypatch):
    """T31: com o 4x4 o run caia em touch_5x5/ sem nenhum frame. Aqui a linha
    de comando que a GUI monta alimenta o logger de verdade."""
    g = gui.PalpationGUI.__new__(gui.PalpationGUI)
    cmd = []
    g._logger_proc = None
    g.get_node_names = lambda: []
    g._touch_spec = C.touch_sensor_spec('4')

    def _popen(a, **_k):
        cmd.extend(a)
        raise FileNotFoundError
    monkeypatch.setattr(gui.subprocess, 'Popen', _popen)
    g._ensure_palpation_logger()
    ctx = rclpy.Context()
    rclpy.init(args=cmd, context=ctx)
    orig = PL.Node.__init__
    monkeypatch.setattr(PL.Node, '__init__',
                        lambda self, n, **k: orig(self, n, context=ctx, **k))
    n = PL.PalpationLogger()
    try:
        assert n._sensor_key == '4' or n._rows == 4
    finally:
        n.destroy_node()
        rclpy.shutdown(context=ctx)


# ══ GUI: sensors.csv ══════════════════════════════════════════════════
def test_gui_ignora_fim_de_outro_run():
    """T28 na GUI."""
    g = gui.PalpationGUI.__new__(gui.PalpationGUI)
    g._lock = threading.Lock()
    g._latest_phase, g._phase_t_start, g._phase_p0 = 'ABORTED', 0.0, None
    g._rec_fh, g._rec_auto, g._rec_run_id = object(), True, '20260101_000001'
    pedidos = []
    g.root = types.SimpleNamespace(after=lambda *a: pedidos.append(a))
    for nome in ('_stop_recording',):
        setattr(g, nome, lambda *a, **k: None)
    for rid, n in (('20260101_000000', 0), ('20260101_000001', 1)):
        pedidos.clear()
        try:
            g._cb_status(_status('ABORTED', rid))
        except AttributeError:
            pass                      # resto: widgets
        assert len(pedidos) == n, rid


def test_sensors_csv_nao_sobrescreve(tmp_path, monkeypatch):
    """T29 na GUI."""
    import touch_pack.gui_sensors as gs
    (tmp_path / C.RUN_SENSORS_CSV).write_text('run anterior')
    g = gui.PalpationGUI.__new__(gui.PalpationGUI)
    g._touch_spec = C.touch_sensor_spec('5')
    g._rec_header = ['a']
    msgs = []
    g._set_rec_status = lambda t, *_a: msgs.append(t)
    g._touch_source = None
    monkeypatch.setattr(gs, 'run_dir', lambda *a, **k: str(tmp_path))
    try:
        g._start_recording('x', 'TOUCH')
    except AttributeError:
        pass
    assert (tmp_path / C.RUN_SENSORS_CSV).read_text() == 'run anterior'


def test_cada_linha_leva_as_tensoes_do_proprio_frame():
    """T30: um chunk com 3 frames gravava 3 linhas com os taxels do último."""
    linhas = []
    src = TouchSensorSource(port=None, rows=5, cols=5, has_total=False,
                            udp_broadcast=False)
    src._on_sample = lambda *a: linhas.append(a)
    txt = ''.join(
        'ADC,' + ','.join(str(v) for _ in range(25)) + f',t={v}\n'
        for v in (1000, 2000, 3000))
    _c, pend = src._ingest(txt)
    for p in pend:
        src._on_sample(*p) if isinstance(p, tuple) else src._on_sample(
            p, *src.latest_voltages_and_time())
    k = 3.3 / 4095.0
    assert [a[1][0, 0] for a in linhas] == pytest.approx(
        [1000 * k, 2000 * k, 3000 * k])
    assert [a[0] for a in linhas] == pytest.approx(
        [a[1].mean() for a in linhas])
    assert len({a[2] for a in linhas}) == 3


# ══ câmera ════════════════════════════════════════════════════════════
def test_camera_fora_do_ar_nao_segura_a_thread_do_tk(monkeypatch):
    """T25."""
    from touch_pack.hand_camera_teleop import HandCameraTeleop
    solta = threading.Event()

    class _Cap:
        def __init__(self, *_a):
            solta.wait(3.0)

        def isOpened(self):
            return False

        def release(self):
            pass
    monkeypatch.setitem(sys.modules, 'cv2', types.SimpleNamespace(
        VideoCapture=_Cap, CAP_V4L2=0))
    monkeypatch.setitem(sys.modules, 'mediapipe', types.SimpleNamespace())
    cam = HandCameraTeleop(on_curl=lambda c: None, camera_index=0,
                           logger=None)
    threading.Timer(0.6, solta.set).start()
    t0 = time.monotonic()
    try:
        cam.start()
    except Exception:
        pass
    dt = time.monotonic() - t0
    if cam._thread is not None:
        cam._thread.join(3)
    assert dt < 0.2
    assert cam.error and 'CameraUnavailable' in cam.error


def test_camera_presa_no_read_nao_tem_a_captura_solta_por_baixo():
    from touch_pack.hand_camera_teleop import HandCameraTeleop
    cam = HandCameraTeleop(on_curl=lambda c: None, camera_index=0, logger=None)
    livre, soltou = threading.Event(), []
    cam._release = lambda: soltou.append(1)
    cam._thread = threading.Thread(target=livre.wait, daemon=True)
    cam._thread.start()
    cam.stop()
    assert soltou == []
    livre.set()


class _Cam(gui_camera.CameraMixin):
    def __init__(self):
        self._lock = threading.Lock()
        self._latest_phase, self._robot_mode = 'IDLE', 'MIRROR'
        self._estop_latched = self._prehoming = False
        self._exec_movement_id = None
        self._latest_joint_rad = [math.radians(v)
                                  for v in (10, 20, -90, 5, 90, 45)]
        self.arm_sliders = {
            f'joint{i}': types.SimpleNamespace(set=lambda v: None,
                                               get=lambda: 0.0)
            for i in range(1, 7)}
        self.pub = []

    def _clamp_var(self, v, lo, hi, default=None):
        return float(v.get())

    def _suppress(self):
        import contextlib
        return contextlib.nullcontext()

    def _publish_arm_q(self, q, _d):
        self.pub.append([math.degrees(v) for v in q])


def test_j6_da_camera_parte_de_onde_esta_e_em_rampa(monkeypatch):
    """T26: joint6 em 45°, primeira detecção 0° -> degrau de 45° em 0,1 s."""
    t = [100.0]
    monkeypatch.setattr(time, 'monotonic', lambda: t[0])
    g = _Cam()
    g._apply_camera_j6({'ArmJ6': 0.0}, mirror_real=True)
    for _ in range(3):
        t[0] += 0.033
        g._apply_camera_j6({'ArmJ6': 80.0}, mirror_real=True)
    j6 = [45.0] + [q[5] for q in g.pub]
    assert len(j6) > 1
    assert np.max(np.abs(np.diff(j6))) <= 60 * 0.033 + 1e-6
    assert g.pub[-1][:5] == pytest.approx([10, 20, -90, 5, 90])


@pytest.mark.parametrize('campo,v', [('_estop_latched', True),
                                     ('_exec_movement_id', 1),
                                     ('_prehoming', True)])
def test_j6_da_camera_respeita_os_bloqueios(campo, v):
    g = _Cam()
    setattr(g, campo, v)
    for k in range(3):
        g._apply_camera_j6({'ArmJ6': 10.0 * k}, mirror_real=True)
    assert g.pub == []


# ══ mão ═══════════════════════════════════════════════════════════════
def test_respawn_da_mao_nao_desliga_quem_reconectou():
    """T27: _connect_real_hand é toggle."""
    g = gui.PalpationGUI.__new__(gui.PalpationGUI)
    g._hand_should_be_alive = True
    g._hand_proc = types.SimpleNamespace(poll=lambda: None)
    g._set_status = lambda *a: None
    chamou = []
    g._connect_real_hand = lambda: chamou.append(1)
    g._on_hand_respawn()
    assert chamou == []
    g._hand_proc = types.SimpleNamespace(poll=lambda: 1)
    g._on_hand_respawn()
    assert chamou == [1]


# ══ IK da viewport ════════════════════════════════════════════════════
def test_singularidade_do_punho_e_marcada():
    """T33: a coluna `singular` era sempre 0."""
    from touch_pack.kinematics import T_TOUCH_TOOL_ATTACH, forward_kinematics
    from touch_pack.manip3d import ik_step
    for j5, esperado in ((0.5, True), (90.0, False)):
        q = np.deg2rad([0.0, 0, -90, 0, j5, 0])
        T = forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)
        r = ik_step(q, T[:3, 3], T_end=T_TOUCH_TOOL_ATTACH,
                    R_lock=T[:3, :3])
        assert r.singular is esperado, j5


def test_gpu_perdida_vai_para_a_malha_reduzida_e_nao_para_o_raster():
    """XT13: caía ~1 s no rasterizador com a malha exata."""
    from touch_pack import manip3d
    v = manip3d.Manip3DView.__new__(manip3d.Manip3DView)
    v._exact, v._dragging, v._cam_mode = True, False, None
    v._scene_coarse = None
    v.cam = manip3d.Camera()
    v.close = lambda: None

    class _Gpu:
        def resize(self, *_a):
            raise RuntimeError('contexto perdido')

        def close(self):
            pass
    v._gpu = _Gpu()
    pedidos, raster = [], []
    v.on_gpu_lost = lambda: pedidos.append(1)
    v._active_scene = lambda: raster.append(1)
    assert v._draw_meshes(100, 100) is False
    assert pedidos == [1] and raster == [] and v._gpu is None


def test_gpu_recusada_nao_recarrega_em_laco(monkeypatch):
    import touch_pack.gui_manip3d_tab as tab
    g = gui.PalpationGUI.__new__(gui.PalpationGUI)
    g._manip_scene_state = 'ready'
    g._manip_exact = True
    g._manip_view = None
    g.get_logger = lambda: types.SimpleNamespace(
        warning=lambda *a: None, info=lambda *a: None)
    g._manip_load_scene = lambda: None
    g._manip_gpu_lost()
    g._end_effector, g._robot_desc_path = 'touch_tool', ''
    g._stop_event = threading.Event()
    g._stop_event.set()
    usados = []
    monkeypatch.setattr(tab, '_manip_vtk_available', lambda: True)
    monkeypatch.setattr(tab, '_build_scene',
                        lambda *a, triangle_budget, **k:
                        usados.append(triangle_budget))
    monkeypatch.setattr(tab, '_coarse_scene', lambda s: None)
    g._manip_scene_worker()
    assert usados == [tab._SCENE_BUDGET]


# ══ IMU ═══════════════════════════════════════════════════════════════
class _DiscoCheio:
    def write(self, *_a):
        raise OSError(28, 'No space left on device')

    flush = close = write


def test_gravacao_de_imu_com_disco_cheio_avisa_e_nao_acumula(tmp_path):
    """XT24a: o writer morria calado e a fila crescia."""
    from touch_pack.imu_recorder import TrackingRecorder
    rec = TrackingRecorder({}, base_dir=str(tmp_path))
    cheio = _DiscoCheio()
    for k in rec._fh:
        rec._fh[k] = cheio
        rec._w[k] = csv.writer(cheio)
    rec.robot(time.monotonic(), frozen=False)
    t0 = time.monotonic()
    while rec._writer.is_alive() and time.monotonic() - t0 < 2:
        time.sleep(0.01)
    assert rec.error
    for _ in range(100):
        rec.robot(time.monotonic(), frozen=False)
    assert rec._q.qsize() == 0
    assert rec.close()['error']


# ══ célula ════════════════════════════════════════════════════════════
def _recv():
    from test_ft_receiver_node import _Recv, _enche
    return _Recv(), _enche


def test_tara_recusa_janela_de_antes_da_queda():
    """T37."""
    r, enche = _recv()
    enche(r, fz=4.0)
    r._rate_win.clear()
    r._rate_win.append(time.perf_counter() - 5.0)
    r._on_tare_request(None)
    assert r._tare_result_pub.msgs[-1].data.startswith('err;no_data')
    assert r._tare_done is False


def test_buraco_na_linha_esquece_a_janela():
    r, enche = _recv()
    enche(r, fz=4.0)
    r._tare_done = True
    r._last_t_us = 0
    r._on_sample(1, 2_000_000, (0.0, 0, 1.0, 0, 0, 0))
    assert len(r._buf) == 1


def test_queda_do_link_zera_a_taxa_publicada():
    from touch_pack.ft_receiver_node import FtReceiverNode
    r, enche = _recv()
    enche(r)
    r._polled = None
    r._serial = types.SimpleNamespace(connected=True, last_rx=0.0, error='',
                                      port='x')
    r._last_rx_warn = True
    FtReceiverNode._check_link(r)
    assert len(r._rate_win) == 0 and len(r._buf) == 0


def test_zero_no_sensor_descarta_as_amostras_antigas():
    """T37: o auto-tare seguinte usava dados de antes do zero."""
    from touch_pack.ft_receiver_node import FtReceiverNode
    r, enche = _recv()
    enche(r, fz=4.0)
    r._tared_pub = types.SimpleNamespace(publish=lambda m: None)
    FtReceiverNode._dispatch(
        r, types.SimpleNamespace(set_zero=lambda: None), 'zero', {})
    assert len(r._buf) == 0


def test_trocar_para_polled_mantem_o_passo(monkeypatch):
    import touch_pack.ft_receiver_node as fr
    visto = {}

    class _P:
        def __init__(self, _s, **k):
            visto.update(k)

        def start(self):
            pass
    monkeypatch.setattr(fr, 'FtPolledDriver', _P)
    r = types.SimpleNamespace(
        _mode='stream', _serial=None, _modbus_slave=1, _modbus_timeout=0.1,
        _on_sample=None, _poll_interval=0.0025, _baud=1_000_000, _polled=None)
    ok, *_ = fr.FtReceiverNode._switch_mode(r, {'mode': 'polled'})
    assert ok and visto['interval_s'] == 0.0025


@pytest.mark.parametrize('ruim', [float('nan'), float('inf')])
def test_amostra_nao_finita_nao_entra_no_filtro(ruim):
    r, _ = _recv()
    r._on_sample(1, 1000, (0.0, 0, ruim, 0, 0, 0))
    assert len(r._buf) == 0 and r._absurd == 1


def test_polled_porta_fechada_nao_passa_por_linha_ocupada():
    """T37: "porta não aberta" contava como busy e connected ficava True."""
    from touch_pack.ft_polled import FtPolledDriver

    class _Fechada:
        def command_session(self):
            raise RuntimeError('porta não está aberta')
    d = FtPolledDriver(_Fechada(), slave_id=1, timeout_s=0.05,
                       on_sample=None)
    d.connected = True
    d._transact()
    assert d.connected is False and 'porta' in d.error and d.busy_skips == 0


def test_polled_linha_ocupada_continua_busy():
    from touch_pack.ft_cmd_channel import LineBusyError, LineTapMixin

    class _Linha(LineTapMixin):
        _tap_lock = threading.Lock()
        _tap_buf = bytearray()
    with pytest.raises(LineBusyError):
        with _Linha().command_session():
            pass


# ══ ft_probe ══════════════════════════════════════════════════════════
def test_probe_abre_exclusivo(monkeypatch):
    """T38."""
    probe = _script('ft_probe')
    kw = {}

    class _S:
        def __init__(self, *a, **k):
            kw.update(k)

        def close(self):
            pass
    monkeypatch.setattr(probe.serial, 'Serial', _S)
    monkeypatch.setattr(probe, 'cmd_stream', lambda *a, **k: 0)
    monkeypatch.setattr(sys, 'argv', ['ft_probe', '--port', '/dev/x'])
    assert probe.main() == 0
    assert kw.get('exclusive') is True


def test_probe_pergunta_a_celula_polled(monkeypatch, capsys):
    """T39: só escutava, e a célula sã parecia muda."""
    from touch_pack.ft_modbus import _append_crc
    import struct
    probe = _script('ft_probe')

    class _Celula:
        """Só responde a quem pergunta; fecha depois de 40 leituras."""
        eof = False

        def __init__(self):
            self.buf, self.n = bytearray(), 0

        def write(self, d):
            assert d[:2] == b'\x01\x03'
            self.n += 1
            if self.n > 40:
                self.eof = True
                return
            self.buf += _append_crc(
                b'\x01\x03\x18' + struct.pack('<6f', 0, 0, -3.5, 0, 0, 0))

        @property
        def in_waiting(self):
            return len(self.buf)

        def read(self, n=1):
            time.sleep(0.005)
            out = bytes(self.buf[:n])
            del self.buf[:n]
            return out

        def close(self):
            pass
    monkeypatch.setattr(probe.serial, 'Serial', lambda *a, **k: _Celula())
    monkeypatch.setattr(sys, 'argv', ['ft_probe', '--port', '/dev/x'])
    out = []
    t = threading.Thread(target=lambda: out.append(probe.main()), daemon=True)
    t.start()
    t.join(8)
    assert '-3.500' in capsys.readouterr().out


# ══ latência e relatórios ═════════════════════════════════════════════
@pytest.mark.parametrize('modulo,classe', [
    ('latency_probe', 'LatencyProbe'),
    ('hand_latency_probe', 'HandLatencyProbe')])
@pytest.mark.parametrize('lag', [0.1, -0.1])
def test_curva_deslocada_casa_com_o_sim(ros, modulo, classe, lag, tmp_path,
                                        monkeypatch):
    """T36: `t + shift` afastava as curvas; o número já estava certo."""
    mpl = pytest.importorskip('matplotlib')
    mpl.use('Agg')
    import matplotlib.axes as mx
    mod = importlib.import_module('touch_pack.' + modulo)
    n = getattr(mod, classe)()
    try:
        t = np.arange(0, 8, 0.01)
        N = n._N if hasattr(n, '_N') else (6 if modulo[0] == 'l' else 6)
        com = np.zeros(N)
        com[0] = 1.0
        n._sim = [(100 + x, list(0.5 * math.sin(math.pi * x) * com))
                  for x in t]
        n._real = [(100 + x, list(0.5 * math.sin(math.pi * (x - lag)) * com))
                   for x in t]
        res = n.analyze()
        assert res['lag_s'] == pytest.approx(lag, abs=0.005)
        if hasattr(mod, 'LATENCY_DIR'):
            monkeypatch.setattr(mod, 'LATENCY_DIR', str(tmp_path))
        curvas = []
        orig = mx.Axes.plot

        def _plot(self, *a, **k):
            curvas.append((np.asarray(a[0]), np.asarray(a[1]), k))
            return orig(self, *a, **k)
        monkeypatch.setattr(mx.Axes, 'plot', _plot)
        n.report_and_save(res)
        ts, sim, _ = curvas[0]
        td, des, _ = next(c for c in curvas if 'deslocado' in c[2]['label'])
        dentro = ts[(ts > 0.5) & (ts < ts[-1] - 0.5)]
        erro = np.abs(np.interp(dentro, td, des) - np.interp(dentro, ts, sim))
        assert erro.max() < 1.0
    finally:
        n.destroy_node()


def test_tabela_latex_usa_a_campanha_atual(tmp_path):
    """T35."""
    from touch_pack.latency_report import write_outputs
    base = dict(junta='joint1', lag_ms=70.0, peak_corr=1.0, mae_deg=.1,
                rmse_deg=.1, emax_deg=.2, r_pearson=.99,
                rmse_sem_compensar_deg=.5)
    rows = [dict(base, captura=c, condicao='drag teach', amp_deg=a)
            for c, a in (('a', 11.2), ('b', 12.7))]
    _csv, tex = write_outputs(rows, str(tmp_path))
    txt = open(tex).read()
    assert '11.2--12.7 deg' in txt and '2 captura' in txt
    assert '33.2' not in txt and '7 capturas' not in txt


def test_titulo_do_relatorio_usa_a_pasta_do_run(tmp_path, monkeypatch):
    """XT25a: punha "plot.png" como data."""
    mpl = pytest.importorskip('matplotlib')
    mpl.use('Agg')
    import matplotlib.axes as mx
    from touch_pack import palpation_report as PR
    tits = []
    monkeypatch.setattr(mx.Axes, 'set_title',
                        lambda self, t, **k: tits.append(t))
    rows = [{'t': 0.1 * i, 'f': 1.0, 'sp': 1.0, 'phase': 'HOLD',
             'force': 1.0, 'cycle': 1} for i in range(20)]
    d = tmp_path / '20261124_101010'
    d.mkdir()
    monkeypatch.setattr(PR, '_plot_force', lambda *a: [])
    monkeypatch.setattr(PR, '_plot_touch', lambda *a: None)
    PR._make_plot(rows, {}, str(d / 'plot.png'))
    assert '20261124_101010' in tits[0] and 'plot.png' not in tits[0]


# ══ coleta sem pandas (T42) ═══════════════════════════════════════════
def test_sem_pandas_a_coleta_dos_outros_continua(tmp_path):
    import subprocess
    (tmp_path / 'pandas.py').write_text(
        "raise ModuleNotFoundError('sem pandas', name='pandas')")
    aqui = os.path.dirname(os.path.abspath(__file__))
    env = dict(os.environ)
    env['PYTHONPATH'] = f'{tmp_path}:{env.get("PYTHONPATH", "")}'
    out = subprocess.run(
        [sys.executable, '-m', 'pytest', '-p', 'no:cacheprovider', '-v',
         'test_palp_char.py', 'test_ft_stats.py'],
        cwd=aqui, env=env, capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stdout[-800:]
    assert ' passed, 1 skipped' in out.stdout


# ══ palp_char ═════════════════════════════════════════════════════════
def _pc():
    pytest.importorskip('pandas')
    pytest.importorskip('scipy')
    from test_palp_char import PC, _run
    return PC, _run


def test_mu_com_j6_girada():
    """T34: com a ferramenta girada o atrito cai no outro eixo do sensor."""
    PC, run = _pc()
    d = run()
    mu0 = PC.sliding(d)['mu']
    w, a = d['w'], math.radians(50)
    fy = w['ft_fy_n'].values.copy()
    w['ft_fx_n'], w['ft_fy_n'] = fy * math.sin(a), fy * math.cos(a)
    assert PC.sliding(d)['mu'] == pytest.approx(mu0, rel=0.02)


def test_k_com_ataque_inclinado():
    PC, run = _pc()
    d = run(K=10.0)
    s = d['s']
    n = int((s.phase == 1).sum())
    # Aproximação livre de 3 mm e contato ao longo de um eixo a 30°.
    ax = np.array([0.5, 0.0, -math.cos(math.radians(30))])
    pen = np.linspace(-3.0, 0.3, n)
    xyz = np.outer(pen, ax) * 1e-3
    z0 = s.tcp_z.values[n:].copy()
    s.loc[s.index[:n], 'tcp_x'] = xyz[:, 0]
    s.loc[s.index[:n], 'tcp_z'] = xyz[:, 2] + 0.1
    s.loc[s.index[n:], 'tcp_z'] = z0
    s.loc[s.index[:n], 'force_net_n'] = np.maximum(0, pen * 10.0)
    assert PC.contact(d)['K_sec_Nmm'] == pytest.approx(10.0, rel=0.06)


def test_rates_com_z_constante_e_run_ruim_isolado(tmp_path, monkeypatch,
                                                  capsys):
    """XT12."""
    PC, run = _pc()
    d = run()
    d['s']['tcp_z'] = 0.1
    assert math.isnan(PC.rates(d)['z_quantum_um'])
    ok = {'contact': {}, 'sliding': {}}

    def _analyse(r, _p):
        if r == 'ruim':
            raise ValueError('min() arg is an empty sequence')
        return ok
    monkeypatch.setattr(PC, 'analyse', _analyse)
    PC.main(['ruim', 'bom'])
    assert '== bom' in capsys.readouterr().out


def test_atraso_com_comandos_fora_de_ordem():
    PC, _ = _pc()
    import pandas as pd
    t = np.arange(0, 10, 0.01)
    y = np.sin(2 * np.pi * 0.5 * t) * 0.01
    tfs = np.where(t < 1.0, 3.0, 0.0)      # lote longo na partida
    s = pd.DataFrame({'t_unix': t, 'tcp_y': np.interp(t - 0.1, t, y)})
    c = pd.DataFrame({'t_unix': t, 'cmd_tfs_s': tfs, 'cmd_tcp_y': y})
    c = c[(t + tfs >= 4.0) | (t >= 1.0)]
    o = PC.delay({'s': s, 'c': c, 'p': {'slide_dir': '+Y'}})
    assert 0.0 <= o['cmd_to_meas_s'] < 0.5
    assert o['track_err_rms_mm'] < 6.0


# ══ IMU na GUI (T21, T24) ═════════════════════════════════════════════
@pytest.fixture
def host(tmp_path, monkeypatch):
    import touch_pack.gui_imu as gi
    from touch_pack import imu_recorder
    from test_gui_imu import _Host
    monkeypatch.setattr(gi, 'IMU_CONFIG_FILE', str(tmp_path / 'c.json'))
    monkeypatch.setattr(gi, 'CONFIG_DIR', str(tmp_path))
    monkeypatch.setattr(imu_recorder, 'RUNS_DIR', str(tmp_path / 'r'))
    monkeypatch.setattr(gi, 'TK_STALL_S', 0)
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip('sem display')
    root.withdraw()
    h = _Host(tk.Frame(root))
    h._build_imu_tab(h.root)
    h.root.after_cancel(h._imu_after)
    yield h
    root.destroy()


def test_volta_do_sensor_nao_persegue_a_mao(host):
    """T21: com a referência antiga o braço ia sozinho ao alvo novo."""
    import touch_pack.gui_imu as gi
    from test_gui_imu import _ori_only, _q, _start_cartesian
    _ori_only(host)
    _start_cartesian(host, {6: _q((0, 0, 1), 0.0)})
    host.tick(5, feeds={6: _q((0, 0, 1), 0.0)})
    q0 = host._imu_cart['q'].copy()
    ts, qs = host._imu_latest[6]
    host._imu_latest[6] = (ts - 2 * gi.IMU_STALE_S, qs)
    host.tick(2)
    host.tick(30, feeds={6: _q((0, 0, 1), 40.0)})    # a mão girou 40° cega
    assert np.degrees(np.max(np.abs(host._imu_cart['q'] - q0))) < 0.5


def test_rastreio_recusado_com_o_assistente_aberto(host):
    host._imu_wizard = types.SimpleNamespace(lift=lambda: None)
    assert 'wizard' in host._imu_block_reason()
    host._imu_wizard = None


def test_mensagem_de_imu_nao_toca_variavel_tk(host):
    """T24: callback ROS lia follow_acc.get() a cada mensagem."""
    from test_gui_imu import _q
    lidos = []

    class _V:
        def get(self):
            lidos.append(threading.current_thread())
            return False
    host._imu_cart_vars = dict(host._imu_cart_vars, follow_acc=_V())
    t = threading.Thread(target=host.feed, args=(6, _q((0, 0, 1), 0.0)))
    t.start()
    t.join()
    assert lidos == []
    host._imu_cart_vars['follow_acc'] = tk.BooleanVar(host.root, value=False)


def test_campo_vazio_na_gravacao_de_imu_avisa(host):
    """XT24a: Spinbox vazio e o Start não fazia nada, calado."""
    class _Vazio:
        def get(self):
            raise tk.TclError('expected floating-point number but got ""')
    orig = host._imu_cart_vars
    host._imu_cart_vars = dict(orig, scale=_Vazio())
    host._imu_rec_open({}, [])
    host._imu_cart_vars = orig
    assert 'invalid' in host._imu_rec_lbl.cget('text')


