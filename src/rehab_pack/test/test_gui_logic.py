"""Lógica de segurança do painel sem abrir janela (o smoke pula sem DISPLAY)."""
import collections
import math
import time
from types import SimpleNamespace as NS

import pytest

pytest.importorskip('tkinter')

from rehab_pack import constants as C, tasks as T                # noqa: E402
from rehab_pack.rehab_gui import RehabGui                        # noqa: E402


class Var:
    def __init__(self, v): self.v = v
    def get(self): return self.v
    def set(self, v): self.v = v


class Botao:
    def __init__(self, texto=''): self.texto, self.st = texto, []
    def cget(self, _k): return self.texto
    def state(self, s): self.st = s


class Ponte:
    def __init__(self):
        self.results, self.ticks = collections.deque(), collections.deque()
        self.last, self.calls, self.refs, self.gotos = {}, [], [], []
        self.idade, self.resposta = 0.0, None

    def age(self, _k): return self.idade

    def call_trigger(self, n):
        self.calls.append(n)
        if self.resposta is not None:
            self.results.append((n, self.resposta, 'indisponível'))

    def call_enable(self, on): self.calls.append(on)
    def connect_real(self, ip): self.calls.append(ip)
    def set_params(self, n, v): self.calls.append(v)
    def publish_alive(self): self.calls.append('alive')
    def publish_event(self, t): self.calls.append(t.split(';')[0])
    def publish_target(self, xy, z): pass
    def publish_reference(self, xy, z): self.refs.append((*xy, z))
    def publish_goto(self, q, pct): self.gotos.append(q)
    def shutdown(self): self.calls.append('shutdown')


def tick(estado='ATIVO', **kw):
    d = {**dict.fromkeys(C.TICK_FIELDS, 0.0), 'x_m': 0.5, 'rx_m': 0.5, **kw}
    return {**d, 'state': kw.get('state', float(C.STATES.index(estado)))}


@pytest.fixture
def app():
    a = RehabGui.__new__(RehabGui)
    a.br, a.said, a.confirma = Ponte(), [], []
    a.root = NS(focus_get=lambda: None, after=lambda *x: 'id', destroy=lambda: a.said.append('destroy'),
                after_cancel=lambda i: a.said.append('cancelado'))
    a._say = lambda t, ok=True: a.said.append(t)
    a._jog_msg = a._say
    a._dlg = a._jog_after = a._estop_ok = a.task = a.task_name = None
    a._closing = a.moving = a.enabled = a._enable_done = a._ref_sent = a._jog_synced = False
    a._jog_t = a._last_estop = a._estop_wait = a._enable_wait = -100.0
    a._estop_wait = a._enable_wait = a._task_z = 0.0
    a.origin = None
    a._estop_alert, a.tick, a.center = '', tick(), (0.5, 0.0, 0.0)
    a.btn_enable = Botao('Habilitar')
    a.resp = False
    a._confirm = lambda *x: (a.confirma.append(x[0]), a.resp)[1]
    return a


ESPACO = NS(keysym='space')


def test_1_espaco_para_com_foco_em_widget_interno_do_tk(app):
    def popdown():
        raise KeyError('popdown')
    app.root.focus_get = popdown
    for mov in (True, False):
        app.moving, app._last_estop, app.br.calls = mov, 0.0, []
        app._on_key_press(ESPACO)
        assert C.S_ESTOP in app.br.calls


@pytest.mark.parametrize('motivo', ['moving', 'jog', 'habilitar'])
def test_2_fechar_em_movimento_exige_parar_confirmado(app, motivo):
    app.moving = motivo == 'moving'
    app._jog_t = time.monotonic() if motivo == 'jog' else 0.0
    app._enable_wait = time.monotonic() if motivo == 'habilitar' else 0.0
    app.br.resposta = False                       # PARAR não chegou
    app.close()
    assert app.confirma and not app._closing and 'destroy' not in app.said
    app._last_estop, app.resp = 0.0, True         # operador aceita fechar
    app.close()
    assert app._closing and 'destroy' in app.said


def test_2_fechar_sem_resposta_tem_limite_e_com_resposta_fecha(app):
    app.moving = True
    t0 = time.monotonic()
    app.close()
    assert 0.8 < time.monotonic() - t0 < 2.0 and not app._closing
    app._last_estop, app.br.resposta, app.confirma = 0.0, True, []
    app.close()
    assert app._closing and not app.confirma


def test_2_parado_fecha_direto(app):
    app.close()
    assert app._closing and C.S_ESTOP not in app.br.calls


def test_4_home_pede_confirmacao(app):
    app.jog_var = [Var(0.0)] * 6
    app.jog_home()
    assert app.confirma and app._jog_after is None
    app.resp = True
    app.jog_home()
    assert app._jog_after is not None


def test_5_duplo_clique_nao_pede_o_contrario(app):
    app.btn_enable.texto = 'Desabilitar'
    app.toggle_enable()
    app.btn_enable.texto, app._enable_done = 'Habilitar', True      # o estado novo já chegou
    app._enable_release()
    app.toggle_enable()
    assert app.br.calls == [False] and app.btn_enable.st == ['disabled']
    app.disable()                                  # o do banner nunca é travado
    assert app.br.calls == [False, False]
    app._enable_wait -= 1.0
    app._enable_release()
    app.toggle_enable()
    assert app.br.calls == [False, False, True]


def test_7_parar_segue_pelo_evento_mesmo_com_o_servico_quebrado(app):
    def quebrado(_n):
        raise RuntimeError('contexto caído')
    app.br.call_trigger = quebrado
    app.estop()
    app._drain_results()
    assert 'ESTOP' in app.br.calls and app._estop_ok is False
    assert 'NÃO CHEGOU' in app._estop_alert


def test_8_preset_envia_so_o_preset_e_e_recusado_em_ativo(app):
    app.pvar = {k: Var(0.0) for k in T.PRESETS['Livre']}
    app._slide = app._mark_preset = lambda *a, **k: None
    app.br.last = {'params': {'stiffness_n_m': 0.0}}
    app.apply_preset('Leve')
    assert app.br.calls == [T.PRESETS['Leve']]
    app.enabled = True
    app.apply_preset('Assistido')
    assert len(app.br.calls) == 1 and app.pvar['stiffness_n_m'].get() == 0.0


def test_10_ip_vazio_nao_conecta(app):
    app.var_ip, app.resp = Var('  '), True
    app.toggle_real(); app.toggle_mode()   # sem status: nada é pedido
    assert not app.br.calls and not app.confirma
    app.br.last['real'] = '{"conectado": false}'
    app.toggle_real()
    assert not app.br.calls and not app.confirma
    app.var_ip.set('10.0.0.7')
    app.toggle_real()
    assert app.br.calls == ['10.0.0.7']


def test_11_referencia_final_espera_a_volta_a_ativo(app):
    app._ref_sent, app.task_name = True, 'tunel_reta'
    app.tick = tick('SEM_FORCA')
    app._task_finished()
    assert not app.br.refs
    app.tick = tick('ATIVO', x_m=0.6)
    app._ref_recenter(); app._ref_recenter()
    assert app.br.refs == [(0.6, 0.0, 0.0)]
    app._ref_sent, app.tick = True, tick('PARADO')
    app._ref_recenter()
    assert len(app.br.refs) == 1 and not app._ref_sent


def test_12_recentrar_nao_muda_o_z_da_referencia(app):
    app.task, app.var_assist = T.TunnelTask('reta', (0.5, 0.0)), Var(True)
    app._task_z, app.center = 0.25, (0.5, 0.0, 0.40)
    app._task_housekeeping()
    assert app.br.refs[0][2] == 0.25


def test_13_sem_pose_conhecida_nao_manda_as_outras_juntas_a_zero(app):
    app.jog_var, app.jog_speed = [Var(10.0)] + [Var(0.0)] * 5, Var(50.0)
    app.tick = tick('PARADO')
    app._jog_send()
    app._jog_synced = True
    app._jog_send()                                # sem /joint_states recente
    assert not app.br.gotos
    app.br.last['joints'] = NS(name=list(C.ARM_JOINTS), position=[0.0] * 6)
    app._jog_send()
    assert len(app.br.gotos) == 1


def test_14_parar_cancela_jog_pendente(app):
    app._jog_after = 'id'
    app.estop()
    assert app._jog_after is None and 'cancelado' in app.said


@pytest.mark.parametrize('v', [9.0, -1.0, math.nan, math.inf])
def test_15_estado_invalido_vira_sem_dados(app, v):
    app.tick = tick(state=v)
    assert app._state_name(True) == 'SEM DADOS'


def test_16_alerta_do_parar_nao_some_com_outra_mensagem(app):
    app.br.resposta = False
    app.estop()
    app._drain_results()
    app._say('outra coisa')
    assert app._estop_alert
    app.br.results.append((C.S_ESTOP, True, ''))
    app._drain_results()
    assert not app._estop_alert


def test_17_nan_nao_chega_a_tarefa(app):
    app.task = NS(done=False, update=lambda *a: pytest.fail('NaN na tarefa'))
    app.br.ticks.append([dict(tick(), x_m=math.nan)[k] for k in C.TICK_FIELDS])
    app._drain_ticks()


def test_17_tarefa_limitada_pela_excursao_vigente(app):
    app.br.last = {'params': {'max_excursion_m': 0.08}}
    app.var_task = Var('centro_fora')
    app.tv = {k: Var(v) for k, _r, v, *_ in __import__('rehab_pack.rehab_gui', fromlist=['x']).TASK_PARAMS}
    app.recenter = lambda: None
    app.cv = NS(delete=lambda *_: None)
    app.task_start()
    assert app.task.reach_m == pytest.approx(0.08)
    app.task, app.tick = None, tick(rx_m=0.56)     # só 2 cm livres
    app.task_start()
    assert app.task is None
    app.br.last = {}
    app.task_start()
    assert app.task is None


@pytest.mark.parametrize('sinal', ['SIGINT', 'SIGTERM'])
def test_7_sigterm_fecha_como_sigint_e_tenta_de_novo_se_recusado(monkeypatch, sinal):
    import os
    import signal
    from rehab_pack import gui_bridge, rehab_gui
    from touch_pack import pack_lock
    antes = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    fila = []

    class App:
        _closing, n = False, 0

        def __init__(self, *_): App.um = self

        def close(self):
            self.n += 1
            self._closing = self.n == 2            # a 1ª vez o operador cancelou

    def mainloop():
        for _ in range(2):
            os.kill(os.getpid(), getattr(signal, sinal))
            fila.pop()()
    monkeypatch.setattr(pack_lock, 'acquire_or_exit', lambda p: None)
    monkeypatch.setattr(gui_bridge, 'GuiBridge', lambda: None)
    monkeypatch.setattr(rehab_gui, 'RehabGui', App)
    monkeypatch.setattr(rehab_gui.tk, 'Tk', lambda: NS(after=lambda ms, f: fila.append(f), mainloop=mainloop))
    try:
        assert rehab_gui.main() == 0
    finally:
        for s, h in antes.items():
            signal.signal(s, h)
    assert App.um.n == 2


def test_7_thread_do_ros_termina_se_o_contexto_caiu(monkeypatch):
    import threading
    gui_bridge = pytest.importorskip('rehab_pack.gui_bridge')

    def spin_once(timeout_sec):
        raise RuntimeError('contexto caído')
    b = gui_bridge.GuiBridge.__new__(gui_bridge.GuiBridge)
    b._stop, b.results = threading.Event(), collections.deque()
    b._executor = NS(spin_once=spin_once)
    monkeypatch.setattr(gui_bridge.rclpy, 'ok', lambda: False)
    b._spin()
    assert len(b.results) == 1


def test_aplicar_ao_no_nao_muda_a_mola_com_o_braco_ativo(app):
    app.pvar = {k: Var(v) for k, v in dict(mass_kg=2.0, damping_ns_m=40.0, stiffness_n_m=50.0,
                                           abort_force_n=15.0).items()}
    app.br.last['params'] = {'stiffness_n_m': 0.0}
    app.enabled = True
    app.apply_params()
    assert not app.br.calls
    app.enabled = False
    app.apply_params()
    assert len(app.br.calls) == 1


def test_assistencia_desmarcada_no_tunel_solta_a_referencia(app):
    app.task = T.TunnelTask.__new__(T.TunnelTask)
    app.var_assist, app._ref_sent = Var(False), True
    app._task_housekeeping()
    assert app.br.refs == [(0.5, 0.0, 0.0)]
