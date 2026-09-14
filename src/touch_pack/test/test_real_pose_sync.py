"""Resolução do IP e política de falha do `real_pose_sync`.

Este nó roda UMA vez no launch e tem uma regra que não é óbvia: robô real
desligado **não é erro**. Ele encerra com sucesso e o Gazebo fica na pose
inicial do URDF. Inverter isso derrubaria todo launch feito sem o braço na
mesa — que é como a maior parte do trabalho em simulação acontece.

A outra metade é de onde sai o IP: parâmetro → robot.json → default de
fábrica, nessa ordem, com cada etapa tolerando ausência e lixo.
"""
import json
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

rclpy = pytest.importorskip('rclpy')

from touch_pack import real_pose_sync as rps                 # noqa: E402

_DEFAULT_IP = '192.168.5.2'


@pytest.fixture(scope='module')
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def node(_ros):
    n = rps.RealPoseSync()
    yield n
    n.destroy_node()


def _robot_json(tmp_path, payload) -> str:
    p = tmp_path / 'robot.json'
    p.write_text(json.dumps(payload), encoding='utf-8')
    return str(p)


# ── Origem do IP ──────────────────────────────────────────────────────
def test_parameter_wins_over_the_config_file(node, monkeypatch, tmp_path):
    """O parâmetro é o override do launch — tem de vencer o arquivo, senão
    não há como apontar para outro braço sem editar o disco."""
    monkeypatch.setattr(rps, 'ROBOT_CONFIG_FILE',
                        _robot_json(tmp_path, {'robot_ip': '10.0.0.1'}))
    node.set_parameters([rclpy.parameter.Parameter(
        'robot_ip', rclpy.Parameter.Type.STRING, '10.0.0.2')])
    assert node._robot_ip() == '10.0.0.2'


def test_null_parameter_does_not_crash(node, monkeypatch, tmp_path):
    """REGRESSÃO: `.value` vem None quando o launch declara o parâmetro como
    null, e o `.strip()` direto estourava AttributeError antes de qualquer
    log — o launch inteiro morria sem dizer por quê."""
    monkeypatch.setattr(rps, 'ROBOT_CONFIG_FILE', str(tmp_path / 'nao-existe'))
    monkeypatch.setattr(node, 'get_parameter',
                        lambda _n: type('_P', (), {'value': None})())
    assert node._robot_ip() == _DEFAULT_IP


def test_blank_parameter_falls_through_to_the_file(node, monkeypatch, tmp_path):
    """String vazia é 'não configurado', não 'IP vazio'."""
    monkeypatch.setattr(rps, 'ROBOT_CONFIG_FILE',
                        _robot_json(tmp_path, {'robot_ip': '10.0.0.1'}))
    assert node._robot_ip() == '10.0.0.1'


def test_missing_config_file_falls_back_to_the_default(node, monkeypatch,
                                                       tmp_path):
    monkeypatch.setattr(rps, 'ROBOT_CONFIG_FILE', str(tmp_path / 'nao-existe'))
    assert node._robot_ip() == _DEFAULT_IP


def test_corrupt_config_file_falls_back_instead_of_raising(node, monkeypatch,
                                                           tmp_path):
    """JSON quebrado não pode derrubar o launch: o default ainda é uma
    tentativa útil, e a mensagem de falha de conexão diz mais que um
    traceback de parse."""
    p = tmp_path / 'robot.json'
    p.write_text('{isto não é json', encoding='utf-8')
    monkeypatch.setattr(rps, 'ROBOT_CONFIG_FILE', str(p))
    assert node._robot_ip() == _DEFAULT_IP


def test_config_file_without_the_key_falls_back(node, monkeypatch, tmp_path):
    monkeypatch.setattr(rps, 'ROBOT_CONFIG_FILE',
                        _robot_json(tmp_path, {'outra_coisa': 1}))
    assert node._robot_ip() == _DEFAULT_IP


# ── Política de falha ─────────────────────────────────────────────────
def test_robot_offline_is_a_successful_run(node, monkeypatch):
    """Contrato central do nó: sem braço na mesa o Gazebo fica na pose do
    URDF e o launch segue. Devolver False aqui faria `main` sair com código 1
    e derrubar todo trabalho em simulação pura."""
    monkeypatch.setattr(node, '_read_robot_joints_urdf', lambda _ip: None)
    chamou = []
    monkeypatch.setattr(node, '_move_sim_to',
                        lambda q: chamou.append(q) or True)
    assert node.run() is True
    assert chamou == [], 'tentou mover o Gazebo sem ter lido o robô real'


def test_driver_unavailable_is_reported_as_offline(node, monkeypatch):
    """`real_driver` pode nem importar (sem dependência do robô instalada).
    Esse caso cai no mesmo ramo de 'robô offline', não num erro."""
    monkeypatch.setattr(rps, '_DRIVER_OK', False)
    monkeypatch.setattr(rps, 'CR10RealDriver', None)
    assert node._read_robot_joints_urdf('10.0.0.1') is None


def test_connection_failure_closes_the_driver(node, monkeypatch):
    """O driver segura sockets TCP e uma thread de keepalive. Sair sem fechar
    depois de um connect() meio-feito deixa o controlador com a sessão presa
    até o timeout dele — e o próximo launch não conecta."""
    fechou = []

    class _Drv:
        def __init__(self, ip, config=None):
            pass

        def connect(self):
            raise OSError('sem rota para o host')

        def read_joints_urdf(self):
            raise AssertionError('não deveria chegar aqui')

        def close(self):
            fechou.append(True)

    monkeypatch.setattr(rps, '_DRIVER_OK', True)
    monkeypatch.setattr(rps, 'CR10RealDriver', _Drv)
    monkeypatch.setattr(rps, 'CR10RealDriverConfig', None)
    assert node._read_robot_joints_urdf('10.0.0.1') is None
    assert fechou == [True]


def test_successful_read_returns_six_floats(node, monkeypatch):
    class _Drv:
        def __init__(self, ip, config=None):
            pass

        def connect(self):
            pass

        def read_joints_urdf(self):
            return [0, 1, 2, 3, 4, 5]      # ints de propósito

        def close(self):
            pass

    monkeypatch.setattr(rps, '_DRIVER_OK', True)
    monkeypatch.setattr(rps, 'CR10RealDriver', _Drv)
    monkeypatch.setattr(rps, 'CR10RealDriverConfig', None)
    q = node._read_robot_joints_urdf('10.0.0.1')
    assert q == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert all(isinstance(v, float) for v in q), (
        'o JointTrajectory exige float; int passaria e estouraria no serializador')


def test_move_duration_survives_the_integer_conversion():
    """`Duration(sec=int(...))` descarta a parte fracionária em silêncio. Com
    o valor atual não há perda; se alguém puser 2,5 s, a trajetória iria em
    2 s — mais rápido que o pedido, contra o braço já posicionado."""
    assert rps._MOVE_DURATION_S == int(rps._MOVE_DURATION_S)
