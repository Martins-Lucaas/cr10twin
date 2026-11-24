"""Smoke da janela do painel com ponte falsa (sem ROS no ar). Pula sem display."""
import collections
import os

import pytest

tk = pytest.importorskip('tkinter')
if not os.environ.get('DISPLAY'):
    pytest.skip('sem DISPLAY', allow_module_level=True)

from rehab_pack import constants as C  # noqa: E402
from rehab_pack.rehab_gui import RehabGui  # noqa: E402


class FakeBridge:
    """Mesma interface de GuiBridge, sem rclpy."""

    def __init__(self):
        self.ticks, self.results = collections.deque(), collections.deque()
        self.last = {'params': {'force_source': 'sim', 'max_excursion_m': 0.15}}
        self.calls, self.events = [], []
        self.targets = []
        self.gotos = []

    def age(self, key): return float('inf')
    def service_ready(self, name): return False
    def call_trigger(self, name): self.calls.append(name)
    def call_enable(self, on): self.calls.append((C.S_ENABLE, on))
    def start_session(self): self.calls.append(C.S_SESSION_START)
    def call_bool(self, name, on): self.calls.append((name, on))
    def connect_real(self, ip): self.calls.append(('connect', ip))
    def set_params(self, node, values): self.calls.append((node, values))
    def publish_target(self, xy, z): self.targets.append(xy)
    def publish_reference(self, xy, z): pass
    def publish_alive(self): pass
    def publish_event(self, text): self.events.append(text)
    def publish_goto(self, q, pct): self.gotos.append(list(q) + [pct])
    def publish_meta(self, meta): self.calls.append(('meta', meta))
    def shutdown(self): pass


def tick(t, x=0.55, y=0.0, state=1.0):
    d = dict.fromkeys(C.TICK_FIELDS, 0.0)
    d.update(t_s=t, x_m=x, y_m=y, z_m=0.25, rx_m=x, ry_m=y, rz_m=0.25, state=state)
    return [d[k] for k in C.TICK_FIELDS]


@pytest.fixture
def app():
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip('display indisponível')
    br = FakeBridge()
    a = RehabGui(root, br)
    root.update()
    yield a, br
    a._confirm = lambda *x, **k: True
    a.close()


def test_abre_todas_as_abas_sem_nos(app):
    a, br = app
    assert a.nb.index('end') == 5
    for i in range(5):
        a.nb.select(i)
        a.root.update()
        a._refresh_ui()
    assert a.lbl_state.cget('text') == 'SEM DADOS'
    assert 'não está no ar' in a.lbl_banner.cget('text')


def test_estop_e_servicos(app):
    a, br = app
    a.estop()
    assert C.S_ESTOP in br.calls
    assert any(e.startswith('ESTOP') for e in br.events)


def test_centro_fora_com_cursor_no_alvo(app):
    a, br = app
    a.nb.select(2)
    br.ticks.append(tick(0.0))
    a._poll()
    a.var_task.set('centro_fora')
    a.task_start()
    assert br.events[-1].startswith('TARGET_ON')
    alvo = a.task.current_target()
    # cursor no alvo por mais de 0,5 s de tempo de simulação
    for k in range(1, 30):
        br.ticks.append(tick(0.03 * k, x=alvo[0], y=alvo[1]))
    a._poll()
    a.root.update()
    a._refresh_ui()
    assert any(e.startswith('TARGET_HIT') for e in br.events)
    assert br.targets


def test_erro_interno_nao_mata_o_laco(app, monkeypatch):
    a, br = app
    monkeypatch.setattr(a, '_drain_ticks', lambda: 1 / 0)
    agendado = []
    monkeypatch.setattr(a.root, 'after', lambda ms, fn=None: agendado.append(fn))
    a._poll()
    assert a._poll in agendado


def test_tarefa_para_ao_sair_de_ativo_sem_mover_a_referencia(app):
    a, br = app
    refs = []
    br.publish_reference = lambda xy, z: refs.append(xy)
    br.ticks.append(tick(0.0)); a._poll(); a.task_start()
    br.ticks.append(tick(0.1, state=4.0)); a._poll()
    assert a.task.done and not refs


def test_espaco_para_dentro_de_campo_de_texto_se_o_braco_se_move(app):
    a, br = app
    e = type('E', (), {'keysym': 'space'})()
    a._typing = lambda: True
    a._on_key_press(e)
    assert C.S_ESTOP not in br.calls           # PARADO: é só um espaço na nota
    a.moving = True
    a._on_key_press(e)
    assert C.S_ESTOP in br.calls


def test_parar_sem_resposta_avisa(app):
    a, br = app
    a.estop(); a._estop_wait -= 1.0; a._poll()
    assert 'SEM RESPOSTA' in a.status.cget('text')


def test_tarefa_recusa_parametro_fora_da_faixa(app):
    a, br = app
    br.ticks.append(tick(0.0)); a._poll()
    a.var_task.set('alcance'); a.tv['duracao'].set(0); a.task_start()
    assert a.task is None


def test_sessao_exige_id_valido(app):
    a, br = app
    a.var_id.set('Maria Silva')
    a.session_start()
    assert C.S_SESSION_START not in br.calls
    a.var_id.set('S01')
    a.session_start()
    assert C.S_SESSION_START in br.calls


def test_espaco_em_botao_com_foco_e_parar(app):
    """Tab dá foco aos botões; Espaço nunca os aciona (é PARAR); Enter aciona."""
    a, br = app
    a.btn_start.focus_force()
    a.root.update()
    a.btn_start.event_generate('<KeyPress-space>')
    a.root.update()
    assert C.S_ESTOP in br.calls
    assert C.S_SESSION_START not in br.calls
    a.btn_start.event_generate('<Return>')
    a.root.update()
    assert C.S_SESSION_START in br.calls


def test_abortado_no_cabecalho(app):
    a, br = app
    br.age = lambda key: 0.0
    br.ticks.append(tick(0.0, state=float(C.STATES.index('ABORTADO'))))
    a._poll()
    a._refresh_ui()
    a.root.update()
    assert a.lbl_state.cget('text') == 'ABORTADO'
    assert 'Reset' in a.lbl_banner.cget('text')
    assert a.btn_banner.winfo_ismapped() and a.btn_banner.cget('text') == 'Reset'


def test_redimensiona_sem_erro(app):
    a, br = app
    br.ticks.append(tick(0.0))
    a._poll()
    for geo in ('1000x800', '1600x1000'):
        a.root.geometry(geo)
        for i in range(5):
            a.nb.select(i)
            a.root.update()
            a._refresh_ui()


def _real(br, **st):
    import json
    base = {'conectado': True, 'estop': False, 'sync_ok': True, 'max_dq_deg': 0.4,
            'armado': False}
    br.last['real'] = json.dumps({**base, **st})
    br.age = lambda key: 0.0 if key in ('real', 'ftraw') else float('inf')


def test_robo_real_e_celula_no_cartao_conexoes(app):
    a, br = app
    a._refresh_real(br)
    assert 'fora do ar' in a.conn['real'].cget('text')       # sem rehab_mirror
    assert 'sem dados' in a.conn['ftraw'].cget('text')           # sem ft_receiver

    _real(br, armado=True)
    a._refresh_real(br)
    t = a.conn['real'].cget('text')
    assert t.startswith('●') and 'em sincronia' in t and 'ARMADO' in t
    assert 'recebendo' in a.conn['ftraw'].cget('text')


@pytest.mark.parametrize('campo,trecho', [
    ({'conectado': False}, 'DESCONECTADO'),
    ({'estop': True}, 'E-STOP ACIONADO'),
    ({'sync_ok': False, 'max_dq_deg': 7.5}, 'FORA DE SINCRONIA 7.5'),
])
def test_robo_real_problema_aparece_em_vermelho_com_palavra(app, campo, trecho):
    a, br = app
    _real(br, **campo)
    a._refresh_real(br)
    t = a.conn['real'].cget('text')
    assert t.startswith('✖') and trecho in t          # ícone + palavra, nunca só cor


def test_conectar_e_modo_exigem_confirmacao(app, monkeypatch):
    a, br = app
    _real(br, conectado=False, modo='SIM_ONLY', ip='10.0.0.7')
    a._refresh_real(br)
    assert a.var_ip.get() == '10.0.0.7' and 'SIM_ONLY' in a.conn['real'].cget('text')
    monkeypatch.setattr(a, '_confirm', lambda *x, **k: False)
    a.toggle_real(); a.toggle_mode()
    assert not br.calls
    monkeypatch.setattr(a, '_confirm', lambda *x, **k: True)
    a.toggle_real(); a.toggle_mode()           # força simulada: MIRROR recusado
    assert br.calls == [('connect', '10.0.0.7')]
    br.last['params'] = {'force_source': 'real'}
    a.toggle_mode()
    assert br.calls == [('connect', '10.0.0.7'), (C.S_REAL_MODE, True)]
    _real(br, modo='MIRROR')
    a.toggle_real(); a.toggle_mode()
    assert br.calls[2:] == [(C.S_REAL_CONNECT, False), (C.S_REAL_MODE, False)]


class _Js:                                    # JointState de mentira, em rad
    def __init__(self, q):
        self.name, self.position = list(C.ARM_JOINTS), list(q)


def _braco_no_ar(br, q=C.HOME_Q_RAD, estado='PARADO'):
    br.last['joints'] = _Js(q)
    br.ticks.append(tick(0.0, state=float(C.STATES.index(estado))))
    br.age = lambda key: 0.0 if key in ('joints', 'tick', 'ft') else float('inf')


def _mexe(a, br, j, graus):
    """O que o ttk.Scale faz quando o operador arrasta."""
    a.jog_var[j].set(graus)
    a._jog_user()
    a.root.after_cancel(a._jog_after)
    a._jog_send()


def test_slider_move_o_braco_na_hora_sem_outro_botao(app):
    import math
    a, br = app
    _braco_no_ar(br)
    a._poll(); a._refresh_ui()
    _mexe(a, br, 0, math.degrees(C.HOME_Q_RAD[0]) + 20.0)
    assert len(br.gotos) == 1
    assert br.gotos[0][0] == pytest.approx(C.HOME_Q_RAD[0] + math.radians(20.0), abs=1e-3)
    assert br.gotos[0][1:6] == pytest.approx(list(C.HOME_Q_RAD[1:]), abs=2e-3)
    assert br.gotos[0][6] == C.JOG_SPEED_DEFAULT_PCT


def test_arrastar_agrupa_envios(app):
    import time
    a, br = app
    _braco_no_ar(br); a._poll(); a._refresh_ui()
    for g in range(10):
        a.jog_var[1].set(-50.0 - g); a._jog_user()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.3:
        a.root.update(); time.sleep(0.01)
    assert len(br.gotos) == 1                      # 10 eventos, 1 comando


def test_sliders_acompanham_o_braco_mas_nao_brigam_com_o_operador(app):
    import math
    a, br = app
    assert [v.get() for v in a.jog_var] == [0.0] * 6
    _braco_no_ar(br); a._poll(); a._refresh_ui()
    casa = [round(math.degrees(q), 1) for q in C.HOME_Q_RAD]
    assert [round(v.get(), 1) for v in a.jog_var] == casa
    a.jog_var[0].set(0.0); a._jog_t = __import__('time').monotonic()
    a._refresh_ui()
    assert a.jog_var[0].get() == 0.0               # acabou de mexer
    a._jog_t -= 5.0; a._refresh_ui()
    assert a.jog_var[0].get() == casa[0]           # alvo recusado: volta ao braço
    assert br.gotos == []                          # seguir o braço não manda comando


def test_salvar_home_ir_para_home_e_reabrir(app, tmp_path, monkeypatch):
    import math
    a, br = app
    monkeypatch.setattr(C, 'HOME_FILE', str(tmp_path / 'x' / 'home.json'))
    q = [0.1, -0.9, -1.9, -1.8, 1.5, 1.2]
    _braco_no_ar(br, q); a._poll(); a._refresh_ui()
    a.jog_save_home()
    assert C.home_q() == pytest.approx(q, abs=1e-4)
    _braco_no_ar(br); a._poll()
    monkeypatch.setattr(a, '_confirm', lambda *x, **k: True)
    a.jog_home(); a.root.after_cancel(a._jog_after); a._jog_send()
    assert br.gotos[-1][:6] == pytest.approx(q, abs=2e-3)


@pytest.mark.parametrize('conteudo', ['{ ruim', '{}', '[1]', '{"joint1": 999}',
                                      '{"joint1":"nan","joint2":0,"joint3":0,"joint4":0,"joint5":0,"joint6":0}'])
def test_home_invalida_cai_no_padrao(tmp_path, monkeypatch, conteudo):
    f = tmp_path / 'home.json'
    f.write_text(conteudo)
    monkeypatch.setattr(C, 'HOME_FILE', str(f))
    assert C.home_q() == list(C.HOME_Q_RAD)


def test_capturar_do_robo_exige_conexao_e_salva_a_pose_real(app, tmp_path, monkeypatch):
    a, br = app
    monkeypatch.setattr(C, 'HOME_FILE', str(tmp_path / 'home.json'))
    a.jog_capture()
    assert not (tmp_path / 'home.json').exists()
    _real(br, q_real=[0.2, -0.8, -1.7, -1.6, 1.4, 1.1])
    a.jog_capture()
    assert C.home_q() == pytest.approx([0.2, -0.8, -1.7, -1.6, 1.4, 1.1], abs=1e-4)


def test_juntas_sem_braco_ou_fora_de_parado_nao_publicam(app):
    a, br = app
    _mexe(a, br, 0, 10.0)
    _braco_no_ar(br, estado='ATIVO')
    a._poll(); a._refresh_ui()
    assert all('disabled' in w.state() for w in a.jog_widgets)
    _mexe(a, br, 0, 10.0)
    assert br.gotos == []


def test_aviso_no_topo_em_mirror(app):
    a, br = app
    _real(br, modo='MIRROR', ip='192.168.5.2')
    a._refresh_ui()
    assert 'ROBÔ REAL' in a.lbl_env.cget('text')


def test_estado_posicionando_tem_estilo_e_texto(app):
    a, br = app
    _braco_no_ar(br, estado='POSICIONANDO')
    a._poll(); a._refresh_ui(); a.root.update()
    assert a.lbl_state.cget('text') == 'POSICIONANDO'
    assert 'PARAR' in a.lbl_banner.cget('text')


def test_roda_do_mouse_nao_altera_spinbox_de_junta(app):
    a, br = app
    _braco_no_ar(br); a._poll(); a._refresh_ui()
    a.nb.select(4); a.root.update()
    antes = a.jog_var[0].get()
    a.jog_spin[0].event_generate('<Button-4>'); a.jog_spin[0].event_generate('<Button-5>')
    a.root.update()
    assert a.jog_var[0].get() == antes and a._jog_after is None


def test_confirmacao_nao_segura_o_grab_e_parar_a_cancela(app):
    a, br = app
    visto = []

    def durante():
        visto.append(a.root.grab_current())
        a.estop()
    a.root.after(200, durante)
    assert a._confirm('t', 'x') is False
    assert visto == [None] and C.S_ESTOP in br.calls
