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
        self.last, self.calls, self.events = {}, [], []
        self.hand_force = (0.0, 0.0, 0.0)
        self.targets = []
        self.gotos = []

    def age(self, key): return float('inf')
    def service_ready(self, name): return False
    def set_hand_force(self, f): self.hand_force = tuple(f)
    def call_trigger(self, name): self.calls.append(name)
    def call_enable(self, on): self.calls.append((C.S_ENABLE, on))
    def set_params(self, node, values): self.calls.append((node, values))
    def publish_target(self, xy, z): self.targets.append(xy)
    def publish_reference(self, xy, z): pass
    def publish_event(self, text): self.events.append(text)
    def publish_goto(self, q): self.gotos.append(list(q))
    def publish_meta(self, meta): self.calls.append(('meta', meta))
    def shutdown(self): pass


def tick(t, x=0.55, y=0.0, state=1.0):
    d = dict.fromkeys(C.TICK_FIELDS, 0.0)
    d.update(t_s=t, x_m=x, y_m=y, z_m=0.25, state=state)
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


def test_centro_fora_pela_mao_virtual(app):
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


def test_tecla_vira_forca_virtual(app):
    a, br = app
    a.hand_keys.add('w')
    a._poll()
    assert br.hand_force == (-5.0, 0.0, 0.0)
    a.br.last['params'] = {'force_source': 'real'}
    a._poll()
    assert br.hand_force == (0.0, 0.0, 0.0)      # desativada fora do virtual


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
    for geo in ('1000x680', '1600x1000'):
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
    assert 'modo simulação' in a.conn['real'].cget('text')       # sem rehab_mirror
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


class _Js:                                    # JointState de mentira, em rad
    def __init__(self, q):
        self.name, self.position = list(C.ARM_JOINTS), list(q)


def _braco_no_ar(br, q=C.HOME_Q_RAD, estado='PARADO'):
    br.last['joints'] = _Js(q)
    br.ticks.append(tick(0.0, state=float(C.STATES.index(estado))))
    br.age = lambda key: 0.0 if key in ('joints', 'tick', 'ft') else float('inf')


def test_aba5_tem_subabas_mao_e_juntas(app):
    a, br = app
    assert a.sub5.index('end') == 2
    assert len(a.jog_var) == 6 and len(a.jog_cur) == 6


def test_juntas_ler_posicao_e_home(app):
    a, br = app
    _braco_no_ar(br, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    a.jog_read()
    assert [round(v.get(), 1) for v in a.jog_var] == [5.7, 11.5, 17.2, 22.9, 28.6, 34.4]
    a.jog_home()
    import math
    assert [round(v.get(), 1) for v in a.jog_var] == [round(math.degrees(q), 1) for q in C.HOME_Q_RAD]


def test_juntas_ir_publica_o_alvo_em_radianos(app):
    import math
    a, br = app
    _braco_no_ar(br)
    a.jog_read()
    a.jog_var[0].set(math.degrees(C.HOME_Q_RAD[0]) + 20.0)
    a.jog_go()
    assert len(br.gotos) == 1
    assert br.gotos[0][0] == pytest.approx(C.HOME_Q_RAD[0] + math.radians(20.0), abs=1e-3)
    assert br.gotos[0][1:] == pytest.approx(list(C.HOME_Q_RAD[1:]), abs=2e-3)


def test_juntas_recusa_fora_do_limite_sem_publicar(app):
    a, br = app
    _braco_no_ar(br)
    a.jog_read()
    a.jog_var[2].set(175.0)                          # J3 vai só até 160°
    a.jog_go()
    assert br.gotos == [] and 'J3' in a.lbl_jog_res.cget('text')


def test_juntas_sem_braco_no_ar_nao_publica(app):
    a, br = app
    a.jog_go()
    assert br.gotos == []


def test_juntas_robo_real_exige_confirmacao(app, monkeypatch):
    import json
    import math
    from rehab_pack import rehab_gui
    a, br = app
    _braco_no_ar(br)
    a.jog_read()
    real_age = br.age
    br.age = lambda key: 0.0 if key == 'real' else real_age(key)
    br.last['real'] = json.dumps({'conectado': True, 'estop': False, 'sync_ok': True,
                                  'max_dq_deg': 0.1, 'armado': False})
    a.jog_var[0].set(math.degrees(C.HOME_Q_RAD[0]) + 15.0)

    perguntas = []
    monkeypatch.setattr(rehab_gui.messagebox, 'askokcancel',
                        lambda titulo, msg, **kw: perguntas.append((titulo, msg)) or False)
    a.jog_go()
    assert perguntas and 'REAL' in perguntas[0][0] and 'E-Stop' in perguntas[0][1]
    assert br.gotos == []                            # cancelou: nada vai ao robô

    monkeypatch.setattr(rehab_gui.messagebox, 'askokcancel', lambda *a_, **k: True)
    a.jog_go()
    assert len(br.gotos) == 1


def test_juntas_so_em_parado(app):
    a, br = app
    _braco_no_ar(br, estado='ATIVO')
    a._poll(); a._refresh_ui(); a.root.update()
    assert 'disabled' in a.btn_jog_go.state()
    assert 'PARADO' in a.lbl_jog_est.cget('text')
    _braco_no_ar(br, estado='PARADO')
    a._poll(); a._refresh_ui(); a.root.update()
    assert 'disabled' not in a.btn_jog_go.state()


def test_estado_posicionando_tem_estilo_e_texto(app):
    a, br = app
    _braco_no_ar(br, estado='POSICIONANDO')
    a._poll(); a._refresh_ui(); a.root.update()
    assert a.lbl_state.cget('text') == 'POSICIONANDO'
    assert 'PARAR' in a.lbl_banner.cget('text')


def test_alvos_nascem_na_posicao_atual_e_nao_em_zero(app):
    """Regressão: alvos em 0° mandariam o braço a uma pose esticada num clique."""
    import math
    a, br = app
    assert [v.get() for v in a.jog_var] == [0.0] * 6      # antes de o braço aparecer
    _braco_no_ar(br)
    a._poll(); a._refresh_ui(); a.root.update()
    assert [round(v.get(), 1) for v in a.jog_var] == [round(math.degrees(q), 1) for q in C.HOME_Q_RAD]
    a.jog_var[0].set(0.0)                                  # o usuário mexe: não é sobrescrito
    a._poll(); a._refresh_ui(); a.root.update()
    assert a.jog_var[0].get() == 0.0
