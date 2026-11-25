"""Nenhum teste lê ou grava o plano salvo da bancada (~/.config/touch_pack)."""
import pytest


@pytest.fixture(autouse=True)
def _plano_salvo_isolado(tmp_path, monkeypatch):
    try:
        import touch_pack.explorer_attack  # noqa: F401 — precisa de std_msgs
    except Exception:
        return
    monkeypatch.setattr('touch_pack.explorer_attack._SURFACE_PLANE_FILE',
                        str(tmp_path / 'surface_plane.json'))

# A trava entre packs dos testes nunca é a da bancada.
import os as _os  # noqa: E402
import tempfile as _tempfile  # noqa: E402
_os.environ['CR10_PACK_LOCK_DIR'] = _tempfile.mkdtemp(prefix='pack_lock_')


# ── Suporte dos testes de comportamento ───────────────────────────────

class Relogio:
    """Substitui o módulo `time` de UM módulo de produção: `sleep` só avança
    o ponteiro. `passo` corre a cada sleep (planta); `parede` é o salto do
    relógio de parede, que o monotônico ignora."""

    def __init__(self):
        self.t, self.parede, self.passo = 1000.0, 0.0, None

    def monotonic(self):
        return self.t

    def time(self):
        return self.t + self.parede

    def sleep(self, s):
        self.t += max(float(s), 1e-6)
        if self.passo:
            self.passo()


class LogMemoria:
    """Logger que guarda (nível, texto)."""

    def __init__(self):
        self.linhas = []

    def __getattr__(self, nivel):
        return lambda txt='', *a, **k: self.linhas.append((nivel, str(txt)))

    def tem(self, trecho, nivel=None):
        return any(trecho in t and nivel in (None, n) for n, t in self.linhas)


@pytest.fixture()
def relogio():
    return Relogio()


@pytest.fixture()
def log_memoria():
    return LogMemoria()


def _rclpy_no_ar():
    rclpy = pytest.importorskip('rclpy')
    meu = not rclpy.ok()
    if meu:
        rclpy.init()
    return rclpy, meu


@pytest.fixture()
def braco(monkeypatch, relogio, log_memoria):
    """Explorer com braço que executa na hora e relógio virtual."""
    rclpy, meu = _rclpy_no_ar()
    import numpy as np
    import touch_pack.tactile_explorer as te
    n = te.TactileExplorer()
    monkeypatch.setattr(te, 'time', relogio)
    n.q_sim = np.deg2rad([0.0, 0.0, -90.0, 0.0, 90.0, 0.0])
    n._q_now = lambda: n.q_sim.copy()
    n._stream_q = lambda q, *a, **k: setattr(n, 'q_sim', np.array(q))
    n._arm_traj_pub.publish = lambda m: setattr(
        n, 'q_sim', np.array(m.points[-1].positions))
    n._force_stale_abort = lambda fase: False
    n._settle = lambda *a, **k: None
    n._settle_until_still = lambda *a, **k: True
    n._settle_until_quiet = lambda *a, **k: True
    n._hold_current_position = lambda *a, **k: None
    n._send_hand_pose = lambda *a, **k: None
    n._speed_factor_pct = 100.0
    n.get_logger = lambda: log_memoria
    n.log = log_memoria
    yield n
    n.destroy_node()
    if meu:
        rclpy.shutdown()


class Onda:
    """Onda de força contra uma mola, com transporte puro `lag_s` na medida e
    canal cru a 400 Hz — o laço de produção inteiro, em relógio virtual."""

    def __init__(self, n, te, rel, monkeypatch):
        import numpy as np
        self.n, self.te, self.rel, self.mp, self.np = n, te, rel, monkeypatch, np
        self.k, self.k_est = 1000.0, 1000.0
        self.lag_s, self.deforma, self.bruto = 0.0, None, None
        self.com_cru, self.taxa = True, 400.0
        self.passos, self.observadas = [], []

    def roda(self, forma='SINE', f_min=1.0, f_max=2.0, hz=1.0, ciclos=5,
             servoj_s=0.020, prof=None):
        from rclpy.parameter import Parameter
        from touch_pack.force_wave import _ForceProfile
        n, np = self.n, self.np
        prof = prof or _ForceProfile(forma, f_min, f_max, hz, ciclos)
        self.prof = prof
        n.set_parameters([Parameter('servoj_period_s', value=servoj_s)])
        n._phase_hold = lambda **k: 'ok'
        n._k_est.k, n._k_est.estimated = self.k_est, True
        n._approach_dir = np.array([0.0, 0.0, -1.0])
        z0 = n._tcp_now()[2]
        pen = lambda: z0 - n._tcp_now()[2]                      # noqa: E731
        hist = [(self.rel.t, 0.0)]
        self.rel.passo = lambda: hist.append((self.rel.t, pen()))

        def forca(t):
            x = np.interp(t - self.lag_s, [h[0] for h in hist[-400:]],
                          [h[1] for h in hist[-400:]])
            f = prof.mean_n + self.k * x
            return self.deforma(self.fase(t), f) if self.deforma else f
        self.t0 = None
        fase0 = n._set_phase
        n._set_phase = lambda p: (fase0(p), setattr(self, 't0', self.rel.t))
        n._fz_corrected = lambda: forca(self.rel.t)
        n._fz_raw = lambda: (None if not self.com_cru else
                             self.bruto(self.fase(self.rel.t))
                             if self.bruto else forca(self.rel.t))
        n._lc_raw_rate_hz = n._lc_rate_hz = lambda: self.taxa

        def desde(t):
            if not self.com_cru:
                return []
            ks = range(int(np.floor(t * 400)) + 1,
                       int(np.floor(self.rel.t * 400)) + 1)
            return [(i / 400.0, forca(i / 400.0)) for i in ks if i / 400.0 > t]
        n._lc_raw_since = desde
        if not hasattr(self, '_base'):
            self._base = (n._qs_step, self.te._WaveLockIn.add)
        base, add = n._qs_step, self._base[1]
        if getattr(base, '_espia', False):
            base = self._base[0]

        def passo(d, step_m, *a, **k):
            self.passos.append((self.rel.t - (self.t0 or self.rel.t), step_m))
            return base(d, step_m, *a, **k)
        passo._espia = True
        n._qs_step = passo
        obs = self.observadas

        def espia(s, t, f):
            obs.append((t, f))
            return add(s, t, f)
        self.mp.setattr(self.te._WaveLockIn, 'add', espia)
        ini = self.rel.t
        out = n._phase_hold_modulated(prof)
        self.durou = self.rel.t - ini
        return out

    def fase(self, t):
        """Ciclos decorridos desde o início da onda."""
        return (t - (self.t0 or t)) * self.prof.freq_hz


@pytest.fixture()
def onda(braco, relogio, monkeypatch):
    import touch_pack.tactile_explorer as te
    return Onda(braco, te, relogio, monkeypatch)


@pytest.fixture(scope='module')
def gui_real(tmp_path_factory):
    """A PalpationGUI de verdade, com todos os widgets (precisa de display).
    Nenhum arquivo do usuário é lido ou gravado."""
    tk = pytest.importorskip('tkinter')
    rclpy, meu = _rclpy_no_ar()
    try:
        tk.Tk().destroy()
    except tk.TclError as exc:                           # pragma: no cover
        if meu:
            rclpy.shutdown()
        pytest.skip(f'sem display: {exc}')
    import json
    import touch_pack.gui_imu as gi
    import touch_pack.gui_poses as gp
    import touch_pack.gui_robot as gr
    import touch_pack.palpation_gui as pg
    mp = pytest.MonkeyPatch()
    d = tmp_path_factory.mktemp('gui')
    for m in (pg, gr, gp):
        for nome, v in list(vars(m).items()):
            if nome.endswith('_FILE') and isinstance(v, str):
                mp.setattr(m, nome, str(d / f'{nome}.json'))
    mp.setattr(gi, 'IMU_CONFIG_FILE', str(d / 'imu.json'))
    (d / 'imu.json').write_text(json.dumps(
        {'cartesian': {'follow_acc': True, 'ori_imu': '2'}}))
    # Nada que abra serial, suba processo, grave em disco ou peça tare.
    for nome in ('_start_touch_source', '_retry_touch_source',
                 '_spawn_touch_receiver', '_ensure_palpation_logger',
                 '_lc_do_tare'):
        mp.setattr(pg.PalpationGUI, nome, lambda s, *a, **k: None)
    g = pg.PalpationGUI()
    g.root.update()
    yield g
    g._stop_event.set()
    for t in (g._spin_thread, g._mirror_poll_thread):
        t.join(2.0)
    g.root.destroy()
    g.destroy_node()
    mp.undo()
    if meu:
        rclpy.shutdown()
