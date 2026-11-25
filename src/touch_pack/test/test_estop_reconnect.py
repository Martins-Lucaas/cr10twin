"""O E-STOP sobrevive à queda e à volta da conexão.

Duas falhas encontradas em 15/09/2026, ambas na janela de reconexão
automática — que é o momento em que o operador MAIS provavelmente aperta o
botão, porque é quando alguma coisa já deu errado:

1. `_estop_engage` só tentava `emergency_stop()` com `_robot_connected`
   True. Com o heartbeat caído e o worker tentando de novo (backoff de até
   30 s), o E-STOP não mandava nada ao controlador — e ainda assim a tela
   anunciava "robot disabled and alarmed". O que estivesse na fila de motion
   seguia executando.

2. `_robot_reconnect_worker` cria um driver NOVO e chama `enable()`. A
   reconexão rearmava o braço sozinha, com a trava da GUI ainda acesa e o
   operador lendo "RECONECTAR" no botão; como o `_estop_engaged` mora no
   objeto antigo, a trava de software se perdia junto e o braço voltava
   aceitando ServoJ.
"""

import pytest

pytest.importorskip('rclpy')

import touch_pack.gui_robot as gui_robot                     # noqa: E402
from touch_pack.real_driver import (                         # noqa: E402
    CR10RealDriver,
    CR10RealDriverError,
)


# ── A trava local é independente da rede ──────────────────────────────

def test_a_trava_arma_antes_de_qualquer_io():
    """É ela que faz `_send_motion`, `servo_j` e `drag_teach` recusarem na
    mesma hora — sem esperar socket, sem esperar resposta."""
    drv = CR10RealDriver(dry_run=True)
    assert drv.estop_engaged is False
    drv.emergency_stop()
    assert drv.estop_engaged is True
    with pytest.raises(CR10RealDriverError, match='E-STOP'):
        drv.servo_j([0.0] * 6)
    with pytest.raises(CR10RealDriverError, match='E-STOP'):
        drv._send_motion('MovJ(0,0,0,0,0,0)')


def test_estop_distingue_trava_local_de_controlador_alarmado():
    """Sem dashboard o comando NÃO chega ao controlador, e dizer que chegou
    é afirmar sobre hardware que não se tocou. `False` é o que obriga a
    tela a falar de fila de motion em vez de "disabled and alarmed"."""
    drv = CR10RealDriver(dry_run=True)
    assert drv.emergency_stop() is False      # dry-run não alarma nada

    real = CR10RealDriver(dry_run=False)
    real._dash = None
    assert real.emergency_stop() is False
    assert real.estop_engaged is True         # mas a trava vale


def test_socket_perdido_nao_impede_a_trava():
    """O E-STOP não pode levantar exceção e abortar o resto da sequência (mão
    aberta, mirror congelado, trava da GUI) por causa de um socket morto."""
    drv = CR10RealDriver(dry_run=False)

    class _Morto:
        def sendall(self, _b):
            raise OSError('broken pipe')

        def settimeout(self, _t):
            pass

        def recv(self, _n):
            return b''

        def close(self):
            pass

    drv._dash = _Morto()
    assert drv.emergency_stop() is False
    assert drv.estop_engaged is True


# ── A GUI não pode pular o hardware ───────────────────────────────────

def test_engage_nao_exige_conexao_viva():
    """A condição é `driver existe`, não `_robot_connected`: é justamente
    com ele False (reconexão em curso) que o braço pode estar executando o
    que já está na fila."""
    drv = _DrvFalso()
    g = _portador(_real_driver=drv, _robot_connected=False)
    gui_robot.RobotMixin._estop_engage(g)
    assert drv.estop_engaged and g._estop_latched
    assert g.freeze                       # explorer congelado junto


def test_engage_nasce_pessimista():
    """`hw_ok = True` como default fazia todo caminho que não falou com o
    controlador ser anunciado como sucesso."""
    for modo in ('MIRROR', 'SIM_ONLY'):
        g = _portador(_robot_mode=modo)   # sem driver: nada foi enviado
        gui_robot.RobotMixin._estop_engage(g)
        assert g._estop_latched
        assert 'disabled and alarmed' not in g.status[-1]


# ── A reconexão não desfaz o E-STOP ───────────────────────────────────

@pytest.mark.parametrize('travado', [True, False])
def test_reconexao_nao_reabilita_com_estop_travado(travado, monkeypatch):
    """`enable()` faz PowerOn + EnableRobot e espera o modo 5 — rodar isso
    com a trava acesa é a reconexão rearmando o braço por conta própria."""
    from types import SimpleNamespace
    drv = _DrvFalso()
    monkeypatch.setattr(gui_robot, 'CR10RealDriver', lambda **k: drv)
    monkeypatch.setattr(gui_robot, 'time', SimpleNamespace(
        sleep=lambda s: None, monotonic=lambda: 0.0, time=lambda: 0.0))
    g = _portador(_estop_latched=travado)
    gui_robot.RobotMixin._robot_reconnect_worker(g, '127.0.0.1')
    assert drv.ordem == (['connect'] if travado else ['connect', 'enable'])
    assert len(g.fila) == 1               # entregue à GUI nos dois casos


@pytest.mark.parametrize('travado', [True, False])
def test_driver_novo_recebe_a_trava_de_volta(travado):
    """`_estop_engaged` mora no objeto do driver. Um driver novo nasce com
    ela False, então ela tem de ser reimposta quando ele entra em serviço."""
    drv = _DrvFalso()
    g = _portador(_estop_latched=travado)
    gui_robot.RobotMixin._finish_robot_connect(g, '127.0.0.1', drv, '{5}')
    assert g._real_driver is drv
    assert drv.estop_engaged is travado
    assert g._estop_latched is travado
    if travado:
        assert any('ENGAGED' in s for s in g.status)


# ── E-Stop vindo do CONTROLADOR (botão físico) ────────────────────────

def test_alarme_do_controlador_trava_a_gui_e_ignora_leitura_velha():
    """RobotMode 9 visto pelo heartbeat trava a GUI sozinha; um 9 lido antes
    de um rearme que terminou depois NÃO pode re-travar o braço liberado."""
    from types import SimpleNamespace
    engajou = []
    gui = SimpleNamespace(_estop_latched=False, _estop_released_at=10.0,
                          _set_status=lambda *a: None)
    gui._estop_engage = lambda: (engajou.append(1),
                                 setattr(gui, '_estop_latched', True))
    alarme = gui_robot.RobotMixin._on_controller_alarm

    alarme(gui, 9.0)                    # lido antes do rearme: velho
    assert not engajou and not gui._estop_latched
    alarme(gui, 11.0)                   # lido depois: trava
    assert engajou == [1] and gui._estop_latched
    alarme(gui, 12.0)                   # já travada: não reengata
    assert engajou == [1]


class _DrvFalso:
    def __init__(self, ordem=None):
        self.ordem = [] if ordem is None else ordem
        self.estop_engaged = False

    def connect(self):
        self.ordem.append('connect')

    def enable(self):
        self.ordem.append('enable')

    def robot_mode(self):
        return '{5}'

    def emergency_stop(self):
        self.ordem.append('estop')
        self.estop_engaged = True
        return True

    def _send_dash(self, c):
        return '0'


def _portador(**kw):
    """GUI mínima para os métodos de E-STOP e reconexão de produção."""
    import threading
    from types import SimpleNamespace
    log = SimpleNamespace(**{k: lambda *a, **k2: None for k in
                             ('info', 'warn', 'error', 'warning', 'debug')})
    g = SimpleNamespace(
        status=[], freeze=[], fila=[],
        _exec_stop=threading.Event(), _prehome_cancel=threading.Event(),
        _stop_event=threading.Event(), _latest_joint_rad=None,
        _robot_mode='MIRROR', _robot_connected=False, _real_driver=None,
        _eci_enabled=False, _estop_latched=False, _robot_reconnecting=True,
        _robot_connecting=False, _drag_enabled=False,
        _servoj_period_s=0.030, _feedback_port=30004,
        _payload_cfg=lambda: {}, _refresh_estop_button=lambda: None,
        _close_driver_quietly=lambda d: None, _save_robot_config=lambda: None,
        _start_robot_heartbeat=lambda: None, _sync_gazebo_to_real=lambda d: None,
        _robot_connect_btn=SimpleNamespace(set_state=lambda *a, **k: None),
        _robot_led=SimpleNamespace(config=lambda *a, **k: None),
        speed_factor_var=SimpleNamespace(get=lambda: 10),
        get_logger=lambda: log)
    g._set_status = lambda t, c=None: g.status.append(t)
    g._freeze_pub = SimpleNamespace(publish=g.freeze.append)
    g.root = SimpleNamespace(after=lambda ms, fn=None, *a: g.fila.append(fn))
    g.__dict__.update(kw)
    return g
