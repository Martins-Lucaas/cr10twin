"""Parada, espelho e exclusão de comandos (mirror_node + GUI).

Nenhum socket, nenhum nó ROS no ar: os métodos de produção rodam sobre
portadores de estado, com um driver falso que registra o que seria mandado ao
braço e falha quando o teste manda.
"""
import json
import math
import os
import threading
import time
import types

import numpy as np
import pytest

pytest.importorskip('rclpy')
tk = pytest.importorskip('tkinter')

import touch_pack.gui_poses as gui_poses                      # noqa: E402
import touch_pack.gui_robot as gui_robot                      # noqa: E402
import touch_pack.mirror_node as mn                           # noqa: E402
import touch_pack.palpation_gui as gui                        # noqa: E402
from std_msgs.msg import Empty                                # noqa: E402
from touch_pack.real_driver import CR10RealDriverError        # noqa: E402

HOME = np.deg2rad([0.0, 0, -90, 0, 90, 0])


class _Log:
    def __getattr__(self, _n):
        return lambda *a, **k: None


class _Pub:
    def __init__(self, ordem=None, nome=''):
        self.msgs, self.ordem, self.nome = [], ordem, nome

    def publish(self, m):
        self.msgs.append(m)
        if self.ordem is not None:
            self.ordem.append(self.nome)


class _Root:
    """`after` enfileira; o teste decide quando o "Tk" roda."""

    def __init__(self):
        self.fila = []

    def after(self, _ms, fn=None, *a):
        self.fila.append((fn, a))

    def roda(self):
        fila, self.fila = self.fila, []
        for fn, a in fila:
            fn(*a)


class _Drv:
    def __init__(self, q=HOME, ordem=None):
        self.q = np.array(q, float)
        self.enviados = []
        self.erro = None
        self.vivo = True
        self.estop_engaged = False
        self.ordem = [] if ordem is None else ordem

    def read_joints_urdf_latest(self):
        return self.q.copy()

    def servo_j_urdf(self, q):
        self.enviados.append(('ServoJ', tuple(q)))
        if self.erro:
            raise CR10RealDriverError(self.erro)

    def mov_j_joint_deg(self, q):
        self.enviados.append(('MovJ', tuple(q)))
        if self.erro or self.estop_engaged:
            raise CR10RealDriverError(self.erro or 'E-STOP')

    def prepare_servoj(self):
        self.enviados.append(('ClearError+Continue',))

    def emergency_stop(self):
        self.estop_engaged = True
        self.ordem.append('estop')
        return True

    def release_emergency_stop(self):
        self.estop_engaged = False

    def enable(self):
        self.enviados.append(('enable',))

    def is_connected(self):
        return self.vivo

    def stop_motion(self):
        self.ordem.append('stop_motion')

    halt = stop_motion

    def stop(self):
        self.ordem.append('stop')

    def pause(self):
        self.ordem.append('pause')

    def close(self):
        self.ordem.append('close')

    def _send_dash(self, c):
        self.enviados.append((c,))
        return '0'


# ══ mirror_node ═══════════════════════════════════════════════════════
class _Mirror(mn.MirrorNode):
    def __init__(self, drv):
        self._lock = threading.Lock()
        self._movj_lock = threading.Lock()
        self._rearm_lock = threading.Lock()
        self._movj_timer = None
        self._phase = 'HOLD'
        self._latest_q = list(HOME)
        self._driver, self._connected = drv, True
        self._stop = threading.Event()
        self._hold = ''
        self._last_servoj_q = None
        self._servoj_period_s = 0.002
        self._freeze_pub = _Pub()

    def get_logger(self):
        return _Log()

    def get_name(self):
        return 'mirror_node'

    def roda(self, s=0.08):
        self._stop.clear()
        t = threading.Thread(target=self._servoj_loop)
        t.start()
        time.sleep(s)
        self._stop.set()
        t.join(2)

    @property
    def handle(self):
        return None


def test_mirror_recusa_de_servoj_trava_sem_limpar_erro():
    """XC2: a colisão era limpa (ClearError+Continue) e o ServoJ reenviado."""
    drv = _Drv()
    drv.erro = 'ServoJ → -2'
    m = _Mirror(drv)
    m.roda()
    assert [c[0] for c in drv.enviados] == ['ServoJ']     # um só, sem reenvio
    assert drv.estop_engaged and m._hold
    assert len(m._freeze_pub.msgs) == 1                   # explorer congelado


def test_mirror_retomada_compara_com_a_pose_real():
    """T02: depois de religar, o 1º ServoJ não pode ser um salto."""
    drv = _Drv(q=HOME + np.deg2rad([0, 5.0, 0, 0, 0, 0]))
    m = _Mirror(drv)
    m.roda()
    assert drv.enviados == []
    assert 'divergem' in m._hold


def test_mirror_em_sincronia_segue_normalmente():
    drv = _Drv()
    m = _Mirror(drv)
    m.roda()
    assert [c[0] for c in drv.enviados] == ['ServoJ'] and not m._hold


def test_mirror_freeze_e_estop_sem_gui():
    """T10: sem a GUI ninguém ligava o FREEZE ao braço real."""
    drv = _Drv()
    m = _Mirror(drv)
    m._cb_freeze(Empty())
    assert drv.estop_engaged
    m.roda()
    assert drv.enviados == []


def test_mirror_frozen_nao_comanda():
    drv = _Drv()
    m = _Mirror(drv)
    m._phase = 'FROZEN'
    m.roda()
    assert drv.enviados == []


def test_mirror_rearme_so_com_sim_e_real_juntos():
    drv = _Drv(q=HOME + 0.2)
    m = _Mirror(drv)
    m._cb_freeze(Empty())
    r = m._srv_rearm(None, types.SimpleNamespace(success=None, message=''))
    assert r.success is False and m._hold
    drv.q = HOME.copy()
    r = m._srv_rearm(None, types.SimpleNamespace(success=None, message=''))
    assert r.success is True and not m._hold


def test_mirror_payload_do_launch_chega_ao_driver(monkeypatch):
    """T18: `payload_kg` do launch era ignorado (valia o default 0,5)."""
    m = _Mirror(_Drv())
    m._feedback_port, m._payload_kg, m._payload_cog_z_mm = 30004, 1.25, 80.0
    visto = {}
    monkeypatch.setattr(mn, 'CR10RealDriver',
                        lambda ip, config: visto.update(c=config))
    m._new_driver('127.0.0.1')
    assert visto['c'].payload_kg == 1.25
    assert visto['c'].payload_cog_m == pytest.approx((0, 0, 0.080))


# ══ GUI ═══════════════════════════════════════════════════════════════
class _Gui(gui.PalpationGUI):
    def __init__(self, drv=None, mode='MIRROR', phase='IDLE'):
        self.ordem = []
        self.root = _Root()
        self._lock = threading.Lock()
        self._real_lock = threading.Lock()
        self._real_driver = drv
        if drv is not None:
            drv.ordem = self.ordem
        self._robot_connected = drv is not None
        self._robot_mode = mode
        self._latest_phase = phase
        self._latest_joint_rad = list(HOME)
        self._exec_stop = threading.Event()
        self._prehome_cancel = threading.Event()
        self._stop_event = threading.Event()
        self._exec_movement_id = None
        self._exec_thread = None
        self._prehoming = self._starting_palpation = False
        self._estop_latched = False
        self._paused = self._drag_enabled = False
        self._mirror_last_target = None
        self._eci_enabled = False
        self._start_published_t = 0.0
        self._robot_reconnecting = self._robot_connecting = False
        self._robot_reconnect_thread = None
        self._last_robot_cmd_t = 0.0
        self._freeze_pub = _Pub(self.ordem, 'freeze')
        self._stop_pub = _Pub(self.ordem, '/palpation/stop')
        self._pause_pub = _Pub(self.ordem, 'pause_pub')
        self._arm_pub = _Pub()
        self._arm_home_deg = {f'joint{i + 1}': math.degrees(v)
                              for i, v in enumerate(HOME)}
        self.status = []
        self.publicou = []

    def get_logger(self):
        return _Log()

    def _set_status(self, t, _c=None):
        self.status.append(t)

    def _refresh_estop_button(self):
        pass

    def _update_drag_btn_auto(self, _v):
        pass

    def _apply_hand_pose_from_movement(self, _p):
        pass

    @property
    def handle(self):
        return None


def test_gui_recusa_de_servoj_nao_rearma_e_congela():
    """XC2 na GUI."""
    drv = _Drv()
    drv.erro = 'ServoJ → -2'
    g = _Gui(drv, phase='HOLD')
    g._servo_tick(drv, list(HOME))
    assert [c[0] for c in drv.enviados] == ['ServoJ']
    assert g._mirror_hold and g._freeze_pub.msgs
    g.root.roda()
    assert g._estop_latched and drv.estop_engaged


def test_gui_espelho_nao_comeca_dessincronizado():
    """T02 na GUI."""
    drv = _Drv(q=HOME + np.deg2rad(4.0))
    g = _Gui(drv, phase='DESCENDING')
    g._servo_tick(drv, list(HOME))
    assert drv.enviados == [] and g._mirror_hold


def test_gui_queda_da_conexao_em_palpacao_vira_estop():
    drv = _Drv()
    g = _Gui(drv, phase='HOLD')
    for n in ('_disable_drag_ui', '_spawn_robot_reconnect'):
        setattr(g, n, lambda *a, **k: None)
    g._robot_connect_btn = g._robot_led = types.SimpleNamespace(
        set_state=lambda *a, **k: None, config=lambda *a, **k: None)
    try:
        g._on_robot_connection_lost()
    except AttributeError:
        pass                # o resto do método mexe em widgets
    assert g._estop_latched and g._mirror_hold


def test_soltar_estop_em_frozen_devolve_o_explorer_a_idle():
    """T12: FROZEN não tinha saída pela GUI no MIRROR."""
    drv = _Drv()
    g = _Gui(drv, phase='FROZEN')
    g._estop_latched = g._mirror_hold = True
    g._estop_release()
    assert [m.data for m in g._stop_pub.msgs] == ['stop']
    assert not g._estop_latched and not g._mirror_hold
    assert drv.enviados == []                       # nada retomado sozinho


def test_soltar_estop_espera_a_reconexao():
    g = _Gui(_Drv())
    g._estop_latched, g._robot_reconnecting = True, True
    g._estop_release()
    assert g._estop_latched


# ── Poses ─────────────────────────────────────────────────────────────
MOV = {'id': 1, 'pose_ids': [1], 'dur_s': 0.1, 'speed_pct': 10}


def _poses(g):
    g._poses = [{'id': 1, 'q_deg': [0.0, 0, -90, 0, 90, 0]}]
    g._movements = [dict(MOV)]


def test_loop_de_poses_morre_no_estop():
    """XC6: o Loop recomeçava e, ao soltar o E-STOP, o braço voltava a andar."""
    drv = _Drv()
    g = _Gui(drv)
    _poses(g)
    t = threading.Thread(target=g._execute_movement_worker, args=(MOV, True))
    t.start()
    time.sleep(0.05)
    g._estop_engage()
    t.join(1.0)
    morto = not t.is_alive()
    n = len(drv.enviados)
    drv.estop_engaged = False            # operador solta
    time.sleep(0.3)
    g._exec_stop.set()
    t.join(2)
    assert morto and len(drv.enviados) == n


def test_movj_recusado_encerra_o_loop():
    drv = _Drv()
    drv.erro = 'MovJ → -2'
    g = _Gui(drv)
    _poses(g)
    t = threading.Thread(target=g._execute_movement_worker, args=(MOV, True))
    t.start()
    t.join(0.5)
    vivo = t.is_alive()
    g._exec_stop.set()
    t.join(2)
    assert not vivo
    assert sum(c[0] == 'MovJ' for c in drv.enviados) == 1


def test_stop_palpacao_tambem_para_as_poses():
    g = _Gui(_Drv())
    g._on_stop_palpation()
    assert g._exec_stop.is_set() and g._prehome_cancel.is_set()
    assert 'stop_motion' in g.ordem


@pytest.mark.parametrize('campo,valor', [
    ('_latest_phase', 'HOLD'), ('_prehoming', True),
    ('_starting_palpation', True), ('_estop_latched', True)])
def test_poses_recusadas_com_palpacao_ou_estop(campo, valor):
    """XC8."""
    g = _Gui(_Drv())
    _poses(g)
    setattr(g, campo, valor)
    g._start_movement(1)
    g._exec_stop.set()
    assert g._exec_thread is None


def test_start_recusado_com_poses_rodando():
    """XC8, o outro sentido."""
    g = _Gui(_Drv())
    g._exec_movement_id = 1
    g._collect_and_start = lambda: g.publicou.append(1)
    g._on_start()
    assert g.publicou == [] and g._starting_palpation is False
    assert 'Poses' in g.status[-1]


# ── pré-home ──────────────────────────────────────────────────────────
@pytest.mark.parametrize('acao', ['_on_stop_palpation', '_estop_engage'])
def test_stop_e_estop_cancelam_o_start_pendente(acao):
    """XC7: convergir depois do Stop publicava /palpation/start."""
    drv = _Drv(q=HOME + 0.3)               # ainda longe da home
    g = _Gui(drv)
    g._start_pub = _Pub()
    chamado = []
    g._do_palpation_start = lambda p, prehomed=False: chamado.append(prehomed)
    g._prehoming = g._starting_palpation = True
    t = threading.Thread(target=g._prehome_worker, args=({},))
    t.start()
    time.sleep(0.15)
    getattr(g, acao)()
    drv.q = HOME.copy()                    # operador aperta Home: converge
    t.join(3)
    g.root.roda()
    assert chamado == [] and not g._prehoming
    assert g._starting_palpation is False


def test_stop_depois_da_chegada_ainda_cancela():
    """A janela entre a chegada e o callback do Tk também conta."""
    g = _Gui(_Drv())
    g._start_pub = _Pub()
    g._starting_palpation = True
    g._prehome_cancel.set()
    g._do_palpation_start({'mode': 'TOUCH'}, prehomed=True)
    assert g._start_pub.msgs == [] and g._starting_palpation is False


# ── Start, pausa, fechar ──────────────────────────────────────────────
@pytest.mark.parametrize('erro', [tk.TclError('expected floating-point'),
                                  ValueError('could not convert')])
def test_campo_vazio_nao_trava_o_start(erro, monkeypatch):
    """T22: `_starting_palpation` ficava preso em True para sempre."""
    g = _Gui(_Drv())

    class _Ctx:
        def __enter__(self):
            raise erro

        def __exit__(self, *a):
            return False
    monkeypatch.setattr(g, '_suppress', lambda: _Ctx(), raising=False)
    try:
        g._on_start()
    except type(erro):
        pass
    assert g._starting_palpation is False


def test_fechar_a_janela_para_o_robo_primeiro():
    """T23: o stop do driver era o último passo, depois de até 10 s."""
    drv = _Drv()
    g = _Gui(drv)
    g._cam_teleop = object()

    def _cam(_m):
        g.ordem.append('camera')
        raise SystemExit            # o resto são widgets
    g._stop_camera_control = _cam
    with pytest.raises(SystemExit):
        g._on_close()
    assert 'camera' in g.ordem
    i = g.ordem.index('camera')
    assert '/palpation/stop' in g.ordem[:i] and 'stop' in g.ordem[:i]
    assert g._exec_stop.is_set()


def test_fechar_sobrevive_a_falha_na_parada():
    drv = _Drv()
    g = _Gui(drv)

    def _boom():
        raise OSError('broken pipe')
    drv.stop = _boom
    g._cam_teleop = object()

    def _cam(_m):
        raise SystemExit
    g._stop_camera_control = _cam
    with pytest.raises(SystemExit):
        g._on_close()


def test_pausa_cala_o_espelho_antes_do_pause():
    drv = _Drv()
    g = _Gui(drv, phase='SLIDING')
    visto = []
    drv.pause = lambda: visto.append(g._paused)
    g._toggle_pause()
    assert visto == [True]


# ── conexão ───────────────────────────────────────────────────────────
def test_reconexao_leva_a_mesma_porta_de_feedback(monkeypatch):
    """T16."""
    g = _Gui()
    g._feedback_port, g._servoj_period_s = 30005, 0.03
    g._robot_reconnecting = True
    portas = []

    def _novo(ip, dry_run, config):
        portas.append(config.feedback_port)
        g._robot_reconnecting = False
        raise CR10RealDriverError('fora')
    monkeypatch.setattr(gui_robot, 'CR10RealDriver', _novo)
    monkeypatch.setattr(gui_robot.time, 'sleep', lambda s: None)
    g._stop_event.wait = lambda *_a: True
    g._robot_reconnect_worker('127.0.0.1')
    assert portas == [30005]


def test_connect_durante_reconexao_nao_cria_segundo_driver(monkeypatch):
    """T20."""
    g = _Gui()
    g._robot_ip_var = types.SimpleNamespace(get=lambda: '127.0.0.1')
    g._robot_connect_btn = types.SimpleNamespace(
        set_state=lambda *a, **k: None)
    g._robot_led = types.SimpleNamespace(config=lambda *a, **k: None)
    g._robot_reconnecting = True
    threads = []
    monkeypatch.setattr(
        gui_robot.threading, 'Thread',
        lambda **k: types.SimpleNamespace(start=lambda: threads.append(k)))
    g._connect_real_robot()
    assert threads == [] and g._robot_reconnecting is False


def test_driver_da_reconexao_cancelada_e_fechado():
    """T20: o perdedor era sobrescrito sem fechar os sockets."""
    g = _Gui(_Drv())
    tarde = _Drv()
    g._robot_reconnecting = False          # cancelada
    g._finish_robot_connect = lambda *a: g.publicou.append(a)
    g._finish_robot_reconnect('127.0.0.1', tarde, '')
    assert g.publicou == [] and tarde.ordem == ['close']


# ── poses.json ────────────────────────────────────────────────────────
def test_poses_json_ilegivel_nao_e_apagado(tmp_path, monkeypatch):
    """T32: JSON cortado virava lista vazia e o próximo save apagava tudo."""
    f = tmp_path / 'poses.json'
    f.write_text('{"poses": [{"id": 1')
    monkeypatch.setattr(gui_poses, 'POSES_FILE', str(f))
    g = _Gui()
    g._load_poses_data()
    g._save_poses_data()
    assert (tmp_path / 'poses.json.bak').read_text().startswith('{"poses": [{')
    assert json.loads(f.read_text())['poses'] == []


def test_queda_no_meio_do_save_preserva_o_anterior(tmp_path, monkeypatch):
    f = tmp_path / 'poses.json'
    f.write_text('{"poses": [], "movements": []}')
    monkeypatch.setattr(gui_poses, 'POSES_FILE', str(f))
    g = _Gui()
    g._poses, g._movements = [{'id': 1}], []

    def _cai(_obj, fh, **_k):
        fh.write('{"po')
        raise KeyboardInterrupt
    monkeypatch.setattr(gui_poses.json, 'dump', _cai)
    with pytest.raises(KeyboardInterrupt):
        g._save_poses_data()
    assert f.read_text() == '{"poses": [], "movements": []}'
    assert os.path.exists(f)


# ── thread do Tk ──────────────────────────────────────────────────────
def test_status_de_outra_thread_passa_pelo_after():
    """T24."""
    g = _Gui()
    quem = []
    g.status_var = types.SimpleNamespace(
        set=lambda t: quem.append(threading.current_thread()))
    g._status_lbl = types.SimpleNamespace(config=lambda **k: None)
    g._status_dot = types.SimpleNamespace(config=lambda **k: None)
    g._set_status = types.MethodType(gui.PalpationGUI._set_status, g)
    t = threading.Thread(target=g._set_status, args=('oi',))
    t.start()
    t.join()
    assert quem == []
    g.root.roda()
    assert quem == [threading.main_thread()]


def test_movj_do_jog_nao_le_variavel_tk_fora_da_thread():
    """T24: o Timer do jog lia o slider Speed."""
    g = _Gui(_Drv())
    lidos = []

    def _get():
        lidos.append(threading.current_thread())
        return 10.0
    g.speed_factor_var = types.SimpleNamespace(get=_get)
    t = threading.Thread(target=g._mirror_movj_send, args=(list(HOME),))
    t.start()
    t.join()
    assert lidos == []
    g.speed_factor_var = types.SimpleNamespace(get=lambda: 9.0)
    g._cache_speed_factor()
    assert g._speed_factor_pct == max(gui_robot.SPEED_FACTOR_MIN, 9)


# ── E-STOP: comportamento (T40; test_estop_reconnect só lê o fonte) ───
def test_estop_vale_durante_a_janela_de_reconexao():
    drv = _Drv()
    g = _Gui(drv, phase='HOLD')
    g._robot_connected = False           # heartbeat caiu, backoff em curso
    g._estop_engage()
    assert drv.estop_engaged and g._estop_latched
    assert g.ordem.index('freeze') < g.ordem.index('estop')


@pytest.mark.parametrize('falha', ['False', 'excecao'])
def test_estop_sem_confirmacao_nao_anuncia_braco_desabilitado(falha):
    drv = _Drv()
    g = _Gui(drv, phase='HOLD')

    def _es():
        drv.estop_engaged = True
        if falha == 'excecao':
            raise CR10RealDriverError('socket caiu')
        return False
    drv.emergency_stop = _es
    g._estop_engage()
    assert g._estop_latched
    assert 'disabled and alarmed' not in g.status[-1]


@pytest.mark.parametrize('travado', [True, False])
def test_reconexao_so_habilita_sem_estop(travado, monkeypatch):
    g = _Gui()
    g._robot_reconnecting, g._estop_latched = True, travado
    drv = _Drv()
    drv.connect = lambda: None
    drv.robot_mode = lambda: ''
    monkeypatch.setattr(gui_robot, 'CR10RealDriver',
                        lambda ip, dry_run, config: drv)
    monkeypatch.setattr(gui_robot.time, 'sleep', lambda s: None)
    g._robot_reconnect_worker('127.0.0.1')
    assert (('enable',) in drv.enviados) is (not travado)


def test_fechar_congela_tambem_o_pre_home_pendente():
    g = _Gui(_Drv())
    g._cam_teleop = object()

    def _cam(_m):
        raise SystemExit
    g._stop_camera_control = _cam
    with pytest.raises(SystemExit):
        g._on_close()
    assert g._prehome_cancel.is_set() and g._mirror_hold


def test_start_com_explorer_frozen_segue_para_a_coleta():
    g = _Gui(_Drv(), phase='FROZEN')
    g._collect_and_start = lambda: g.publicou.append(1)
    g._on_start()
    assert g.publicou == [1]
