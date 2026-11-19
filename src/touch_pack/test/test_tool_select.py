"""Seleção da ponteira montada e calibração de TCP amarrada a ela."""
import json
import math

import pytest

from touch_pack import constants, tcp_calib


def _write(path, data):
    path.write_text(json.dumps(data))


@pytest.fixture(autouse=True)
def _sem_env(monkeypatch):
    monkeypatch.delenv(constants.TOOL_ENV, raising=False)


def test_env_do_launch_vence_o_arquivo(tmp_path, monkeypatch):
    f = tmp_path / 'tool.json'
    _write(f, {'tool': 'F'})
    monkeypatch.setattr(constants, 'TOOL_FILE', str(f))
    monkeypatch.setenv(constants.TOOL_ENV, 'texturas_v2')
    assert constants.selected_tool() == 'texturas_v2'
    monkeypatch.setenv(constants.TOOL_ENV, 'Z')
    assert constants.selected_tool() == 'F'


def test_sem_arquivo_e_a_ponteira_f(tmp_path, monkeypatch):
    monkeypatch.setattr(constants, 'TOOL_FILE', str(tmp_path / 'tool.json'))
    assert constants.selected_tool() == 'F'
    assert constants.TOOLS['F'][0] == 67.7


def test_texturas_v2_selecionada(tmp_path, monkeypatch):
    f = tmp_path / 'tool.json'
    _write(f, {'tool': 'texturas_v2'})
    monkeypatch.setattr(constants, 'TOOL_FILE', str(f))
    assert constants.selected_tool() == 'texturas_v2'
    comp, w, h = constants.TOOLS['texturas_v2']
    assert comp == 75.2
    assert math.isclose(0.5 * math.hypot(w, h), 54.43, abs_tol=0.01)


def test_nome_desconhecido_cai_na_f(tmp_path, monkeypatch):
    f = tmp_path / 'tool.json'
    _write(f, {'tool': 'Z'})
    monkeypatch.setattr(constants, 'TOOL_FILE', str(f))
    assert constants.selected_tool() == 'F'


def test_calibracao_de_outra_ponteira_e_ignorada(tmp_path, monkeypatch):
    tool = tmp_path / 'tool.json'
    calib = tmp_path / 'tcp_calib.json'
    monkeypatch.setattr(constants, 'TOOL_FILE', str(tool))
    monkeypatch.setattr(tcp_calib, 'TCP_CALIB_FILE', str(calib))
    _write(calib, {'offset_mm': [0, 0, 68.0]})          # antiga, da F
    assert tcp_calib.load_offset_mm() == (0.0, 0.0, 68.0)
    _write(tool, {'tool': 'texturas_v2'})
    assert tcp_calib.load_offset_mm() is None
    _write(calib, {'offset_mm': [0, 0, 76.0], 'tool': 'texturas_v2'})
    assert tcp_calib.load_offset_mm() == (0.0, 0.0, 76.0)
