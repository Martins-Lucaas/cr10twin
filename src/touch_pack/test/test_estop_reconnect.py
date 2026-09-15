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
import re
from pathlib import Path

import pytest

pytest.importorskip('rclpy')

import touch_pack.gui_robot as gui_robot                     # noqa: E402
from touch_pack.real_driver import (                         # noqa: E402
    CR10RealDriver,
    CR10RealDriverError,
)


def _corpo(nome: str) -> str:
    src = Path(gui_robot.__file__).read_text()
    m = re.search(rf'def {nome}\(.*?(?=\n    def )', src, re.S)
    assert m, f'{nome} sumiu de gui_robot'
    return m.group(0)


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
    corpo = _corpo('_estop_engage')
    # Só o CÓDIGO: os comentários falam de `_robot_connected` de propósito,
    # para explicar por que ele saiu da condição.
    codigo = '\n'.join(linha for linha in corpo.splitlines()
                       if not linha.lstrip().startswith('#'))
    trecho = codigo.split('hw_ok')[1].split('elif')[0]
    assert '_real_driver is not None' in trecho
    assert 'self._robot_connected' not in trecho, (
        'a chamada do E-STOP voltou a exigir conexão viva')


def test_engage_nasce_pessimista():
    """`hw_ok = True` como default fazia todo caminho que não falou com o
    controlador ser anunciado como sucesso."""
    assert 'hw_ok = False' in _corpo('_estop_engage')


# ── A reconexão não desfaz o E-STOP ───────────────────────────────────

def test_reconexao_nao_reabilita_com_estop_travado():
    """`enable()` faz PowerOn + EnableRobot e espera o modo 5 — rodar isso
    com a trava acesa é a reconexão rearmando o braço por conta própria."""
    corpo = _corpo('_robot_reconnect_worker')
    i_latch = corpo.find('_estop_latched')
    i_enable = corpo.find('drv.enable()')
    assert i_latch != -1, 'o worker voltou a reabilitar sem olhar a trava'
    assert i_enable != -1
    assert i_latch < i_enable, 'a checagem tem de vir ANTES do enable()'


def test_driver_novo_recebe_a_trava_de_volta():
    """`_estop_engaged` mora no objeto do driver. Um driver novo nasce com
    ela False, então o único ponto em que ele entra em serviço é o único em
    que a trava pode ser reimposta."""
    corpo = _corpo('_finish_robot_connect')
    assert '_estop_latched' in corpo
    assert 'emergency_stop()' in corpo
    i_atribui = corpo.find('self._real_driver = drv')
    i_reimpoe = corpo.find('emergency_stop()')
    assert i_atribui < i_reimpoe, (
        'a trava tem de ser reimposta depois de o driver entrar em serviço')
