"""O que o DobotStudio Pro guarda no controlador e o modo TCP/IP não herda.

Carga, recuo pós-colisão, acelerações e ferramenta calibrada: se estes
comandos deixarem de sair no enable, nada quebra na hora — o braço obedece
igual. O preço aparece depois, na detecção de colisão calibrada para uma carga
que não é a montada e no `TCPForce` do feedback estimado em cima dela.
"""
import socket

import pytest

from touch_pack import tcp_calib
from touch_pack.real_driver import (
    CR10RealDriver, CR10RealDriverConfig, CR10RealDriverError, dash_error_text,
)


class _DashFalso:
    """Socket do dashboard que responde 0 (ok) a tudo e guarda o enviado.

    A resposta só existe DEPOIS do envio. O driver drena respostas atrasadas
    antes de cada comando, e um socket que entrega bytes para sempre prende
    essa drenagem em laço infinito.
    """

    def __init__(self, resposta=b'0,{},cmd();'):
        self.resposta = resposta
        self.sent = []
        self._pendente = []

    def recv(self, _n):
        if not self._pendente:
            raise socket.timeout()
        return self._pendente.pop(0)

    def settimeout(self, _t):
        pass

    def gettimeout(self):
        return None

    def sendall(self, data):
        self.sent.append(data.decode('ascii').strip())
        self._pendente.append(self.resposta)

    def close(self):
        pass


@pytest.fixture
def drv():
    d = CR10RealDriver(config=CR10RealDriverConfig(
        payload_kg=1.25, payload_cog_m=(0.0, 0.0, 0.082),
        collision_backoff_mm=15.0, post_collision_mode=1,
        acc_j=60, acc_l=40, cp_ratio=0))
    d._dash = _DashFalso()
    return d


def _comandos(drv) -> list[str]:
    return [c.split('(')[0] for c in drv._dash.sent]


def test_enable_manda_carga_colisao_e_aceleracoes(drv, monkeypatch):
    monkeypatch.setattr(tcp_calib, 'load_offset_mm', lambda: None)

    drv.apply_runtime_config()

    assert 'SetPayload(1.250,0.0,0.0,82.0)' in drv._dash.sent
    assert 'SetBackDistance(15.0)' in drv._dash.sent
    assert 'SetPostCollisionMode(1)' in drv._dash.sent
    assert 'AccJ(60)' in drv._dash.sent
    assert 'AccL(40)' in drv._dash.sent


def test_sem_calibracao_o_tool_do_controlador_nao_e_tocado(drv, monkeypatch):
    """Trocar um TCP errado por outro é pior que manter o do fabricante."""
    monkeypatch.setattr(tcp_calib, 'load_offset_mm', lambda: None)

    drv.apply_tool_offset()

    assert not any(c.startswith(('SetTool', 'Tool')) for c in _comandos(drv))


def test_com_calibracao_escreve_e_seleciona_a_ferramenta(drv, monkeypatch):
    monkeypatch.setattr(tcp_calib, 'load_offset_mm', lambda: (1.5, -2.0, 67.9))

    drv.apply_tool_offset()

    assert 'SetTool(1,{1.500,-2.000,67.900,0,0,0},1)' in drv._dash.sent
    assert 'Tool(1)' in drv._dash.sent


def test_um_comando_recusado_nao_derruba_os_outros(drv, monkeypatch):
    """Firmware sem algum destes responde erro; o braço segue utilizável."""
    monkeypatch.setattr(tcp_calib, 'load_offset_mm', lambda: None)
    drv._dash.resposta = b'-10000,{},SetBackDistance();'

    drv.apply_runtime_config()          # não levanta

    assert 'AccL(40)' in drv._dash.sent


def test_freio_so_mexe_em_junta_valida(drv):
    with pytest.raises(ValueError):
        drv.brake(0, True)
    with pytest.raises(ValueError):
        drv.brake(7, True)


def test_freio_solta_a_junta_pedida(drv):
    drv.brake(3, True)
    drv.brake(3, False)

    assert drv._dash.sent == ['BrakeControl(3,1)', 'BrakeControl(3,0)']


def test_freio_recusado_com_o_braco_habilitado_vira_erro_legivel(drv):
    """O controlador devolve -1 com o braço habilitado; a tela tem de dizer
    POR QUE, senão o operador fica clicando."""
    drv._dash.resposta = b'-1,{},BrakeControl(1,1);'

    with pytest.raises(CR10RealDriverError) as exc:
        drv.brake(1, True)

    assert 'DESABILITADO' in str(exc.value)


@pytest.mark.parametrize('resp,trecho', [
    ('-2,{},MovJ();', 'ALARME'),
    ('-3,{},MovJ();', 'EMERG'),
    ('-4,{},MovJ();', 'POTÊNCIA'),
    ('0,{},MovJ();', 'sem erro'),
])
def test_codigo_de_erro_vira_texto(resp, trecho):
    assert trecho in dash_error_text(resp)


def test_codigo_desconhecido_nao_ganha_texto_inventado():
    assert dash_error_text('-77777,{},X();') == 'code=-77777'
    assert dash_error_text('') == 'code=?'
