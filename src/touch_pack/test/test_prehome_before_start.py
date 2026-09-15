"""O Start da palpação não publica antes de o braço real estar na home.

Regressão do incidente de 09/09/2026: com a GUI em MIRROR, o simulador estava
na home e o braço real 1,2° atrás em j6. Ao apertar Start a fase deixou de ser
IDLE, o `_mirror_poll_loop` passou a mandar ServoJ com a pose do SIMULADOR e
descarregou a divergência inteira num único tick de 30 ms — um degrau, sem
rampa, em todas as juntas que diferiam.

O que se testa aqui é a ORDEM: `_do_palpation_start` sem `prehomed` não pode
chegar a publicar quando há braço real em MIRROR; ele tem de devolver o
controle ao pré-home e voltar depois. Em sim-only nada muda.
"""
import math
import threading

import pytest

pytest.importorskip('rclpy')

import touch_pack.palpation_gui as gui   # noqa: E402


class _Fake:
    """O mínimo que `_prehome_before_start` toca."""

    _PREHOME_TOL_RAD = gui.PalpationGUI._PREHOME_TOL_RAD
    _PREHOME_ARRIVE_RAD = gui.PalpationGUI._PREHOME_ARRIVE_RAD
    _PREHOME_TIMEOUT_S = gui.PalpationGUI._PREHOME_TIMEOUT_S

    def __init__(self, *, driver=None, connected=False, mode='SIM_ONLY'):
        self._real_lock = threading.Lock()
        self._real_driver = driver
        self._robot_connected = connected
        self._robot_mode = mode
        self._prehoming = False
        self.home_aplicada = False
        self.threads = []
        self.status = []

    def _apply_arm_home(self):
        self.home_aplicada = True

    def _set_status(self, texto, _cor=None):
        self.status.append(texto)

    def _prehome_worker(self, payload, sync_only=False):
        pass                               # alvo da thread; não roda aqui


def _chamar(fake, payload=None, monkeypatch=None):
    """Invoca o método real sobre o fake, sem subir thread de verdade."""
    if monkeypatch is not None:
        monkeypatch.setattr(gui.threading, 'Thread',
                            lambda **kw: _ThreadFalsa(fake, kw))
    return gui.PalpationGUI._prehome_before_start(fake, payload or {})


class _ThreadFalsa:
    def __init__(self, fake, kw):
        self._fake = fake
        fake.threads.append(kw)

    def start(self):
        pass


def test_sim_only_publica_direto(monkeypatch):
    """Sem braço real não há o que sincronizar — o Start segue como antes."""
    f = _Fake(mode='SIM_ONLY')
    assert _chamar(f, monkeypatch=monkeypatch) is False
    assert not f.home_aplicada
    assert not f.threads


def test_mirror_desconectado_publica_direto(monkeypatch):
    """MIRROR configurado mas sem conexão: idem — nada a comandar."""
    f = _Fake(driver=object(), connected=False, mode='MIRROR')
    assert _chamar(f, monkeypatch=monkeypatch) is False
    assert not f.home_aplicada


def test_mirror_conectado_segura_o_start(monkeypatch):
    """O caso do incidente: assume o start e vai para a home ANTES."""
    f = _Fake(driver=object(), connected=True, mode='MIRROR')
    assert _chamar(f, monkeypatch=monkeypatch) is True
    assert f.home_aplicada, 'não usou o caminho do botão ⌂ Home'
    assert len(f.threads) == 1
    assert f.threads[0]['daemon'] is True
    assert f._prehoming is True


def test_matrix_nao_passa_pela_home(monkeypatch):
    """MATRIX_MAP parte da pose de JOG em que o operador deixou a sonda,
    logo acima do primeiro ponto — é o único modo que NÃO passa pela home.

    Até 15/09/2026 o pré-home só olhava `_robot_mode`, nunca o modo de
    palpação: com o braço real conectado, iniciar uma matriz mandava o
    MovJ da home e destruía exatamente a pose que o modo existe para usar.
    A origem passava a ser procurada no XY da home e o run abortava por
    'no_contact' ao esgotar depth_mm em ar livre.
    """
    f = _Fake(driver=object(), connected=True, mode='MIRROR')
    assert _chamar(f, {'mode': 'MATRIX_MAP'}, monkeypatch) is True
    assert not f.home_aplicada, 'a matriz foi levada à home — perdeu o jog'


def test_matrix_ainda_verifica_o_sincronismo(monkeypatch):
    """O que o pré-home protege continua valendo: o degrau de ServoJ no
    primeiro tick do mirror depende de sim e real CONCORDAREM, não de
    concordarem NA HOME. Então a matriz assume o start do mesmo jeito — só
    que para conferir, sem mover nada."""
    f = _Fake(driver=object(), connected=True, mode='MIRROR')
    assert _chamar(f, {'mode': 'MATRIX_MAP'}, monkeypatch) is True
    assert len(f.threads) == 1
    assert f.threads[0]['args'][1] is True, 'worker não entrou em sync_only'
    assert f._prehoming is True


def test_os_demais_modos_continuam_indo_a_home(monkeypatch):
    """A exceção é só do MATRIX: TOUCH, SLIDE e MANUAL descem a partir da
    home e têm de continuar sendo pré-homeados."""
    for modo in ('TOUCH', 'SLIDE', 'MANUAL'):
        f = _Fake(driver=object(), connected=True, mode='MIRROR')
        assert _chamar(f, {'mode': modo}, monkeypatch) is True
        assert f.home_aplicada, f'{modo} deixou de passar pela home'
        assert f.threads[0]['args'][1] is False


def test_nao_reentra_enquanto_faz_home(monkeypatch):
    """Segundo clique durante o homing não dispara um segundo MovJ."""
    f = _Fake(driver=object(), connected=True, mode='MIRROR')
    _chamar(f, monkeypatch=monkeypatch)
    f.home_aplicada = False
    assert _chamar(f, monkeypatch=monkeypatch) is True
    assert not f.home_aplicada
    assert len(f.threads) == 1


def test_tolerancia_de_sincronismo_e_menor_que_a_divergencia_do_incidente():
    """A banda de aceite tem de RECUSAR o que causou o solavanco.

    Se a tolerância fosse ≥ 1,2° o pré-home aprovaria exatamente a
    divergência de 09/09/2026 e o degrau de ServoJ voltaria a acontecer."""
    assert gui.PalpationGUI._PREHOME_TOL_RAD < math.radians(1.2)
    # E a chegada à home é a folgada das duas: o resto do caminho é feito
    # pelo _phase_goto_home do explorer, que é lento e verificado.
    assert (gui.PalpationGUI._PREHOME_ARRIVE_RAD
            >= gui.PalpationGUI._PREHOME_TOL_RAD)


def test_o_gate_esta_no_unico_ponto_de_publicacao():
    """`_do_palpation_start` precisa consultar o pré-home ANTES de publicar.

    Teste de fonte porque o caminho de publicação exige ROS, Tk e um braço:
    o que importa é que o gate venha antes do `_start_pub.publish`."""
    import inspect
    src = inspect.getsource(gui.PalpationGUI._do_palpation_start)
    i_gate = src.find('_prehome_before_start')
    i_pub = src.find('_start_pub.publish')
    assert i_gate != -1, 'o gate de pré-home sumiu de _do_palpation_start'
    assert i_pub != -1
    assert i_gate < i_pub, 'o pré-home tem de vir ANTES da publicação'
