"""Testes da classe de escrita (sem ROS). Dados só em tmp_path."""
import csv
import json
import math
import os
import re
import xml.etree.ElementTree as ET

import pytest

from rehab_pack import constants as C
from rehab_pack.session_logger import SessionWriter

META = {'subject_id': 'S01', 'lado_paretico': 'D', 'tarefa': 'centro-fora', 'notas': 'x'}
PARAMS = {'mass_kg': 2.0, 'damping_ns_m': 30.0, 'force_source': 'virtual'}


def _tick(t, state=1, fx=1.0):
    d = [0.0] * len(C.TICK_FIELDS)
    d[0] = t
    d[1] = fx
    d[C.TICK_FIELDS.index('state')] = state
    return d


def _sessao(tmp_path, emg=0):
    w = SessionWriter(str(tmp_path))
    pasta = w.abrir(META, PARAMS, {'fonte': 'virtual', 'topico_origem': '/rehab/virtual_wrench',
                                   'sim_sign': 1.0}, [0.0] * 6, emg_canais=emg)
    return w, pasta


def _cab(p):
    with open(p, newline='') as f:
        return next(csv.reader(f))


def test_cabecalhos_exatos(tmp_path):
    w, pasta = _sessao(tmp_path)
    w.fechar('concluida')
    assert _cab(os.path.join(pasta, 'samples.csv')) == list(C.SAMPLES_COLUMNS)
    assert _cab(os.path.join(pasta, 'wrench.csv')) == list(C.WRENCH_COLUMNS)
    assert _cab(os.path.join(pasta, 'events.csv')) == list(C.EVENTS_COLUMNS)
    assert not os.path.exists(os.path.join(pasta, 'emg.csv'))


def test_start_eventos_stop_gera_arquivos(tmp_path):
    w, pasta = _sessao(tmp_path)
    assert re.search(r'S01[/\\]\d{8}_\d{6}_centro-fora$', pasta)
    w.escrever_evento(0.0, 'TARGET_ON', 'id=1')
    for i in range(10):
        w.escrever_tick(_tick(i * 0.03), alvo=(0.5, 0, 0.2), q=[0.1] * 6)
        w.escrever_wrench(i * 0.01, (1, 2, 3), (0, 0, 0), 'world')
    w.escrever_tick(_tick(0.3))  # sem alvo/q -> NaN
    w.escrever_evento(0.3, 'TARGET_HIT', 'id=1')
    resumo = w.fechar('concluida')
    assert resumo['totais']['acertos'] == 1
    for n in ('samples.csv', 'wrench.csv', 'events.csv', 'meta.json', 'summary.json'):
        assert os.path.getsize(os.path.join(pasta, n)) > 0
    with open(os.path.join(pasta, 'samples.csv'), newline='') as f:
        linhas = list(csv.reader(f))
    assert len(linhas) == 12 and all(len(l) == len(C.SAMPLES_COLUMNS) for l in linhas)
    assert math.isnan(float(linhas[-1][-1]))
    json.load(open(os.path.join(pasta, 'summary.json')))
    pd = pytest.importorskip('pandas')
    for n in ('samples.csv', 'wrench.csv', 'events.csv'):
        assert len(pd.read_csv(os.path.join(pasta, n))) > 0


def test_meta_segue_esquema(tmp_path):
    w, pasta = _sessao(tmp_path)
    meta = json.load(open(os.path.join(pasta, 'meta.json')))
    assert meta['status'] == 'interrompida'  # meta de segurança enquanto aberta
    w.fechar('concluida')
    m = json.load(open(os.path.join(pasta, 'meta.json')))
    assert m['schema'] == 1 and m['status'] == 'concluida' and m['fim_iso']
    for k in ('session_id', 'subject_id', 'lado_paretico', 'tarefa', 'notas', 'inicio_iso',
              'codigo', 'admitancia', 'forca', 'mastro', 'pose_inicial_rad', 'emg_canais',
              'arquivos'):
        assert k in m, k
    assert set(m['codigo']) == {'git_commit', 'git_dirty', 'versao_pacote'}
    assert m['admitancia'] == PARAMS
    assert set(m['forca']) == {'fonte', 'topico_origem', 'sim_sign'}
    assert set(m['mastro']) == {'massa_empunhadura_kg', 'massa_placa_kg', 'tcp_offset_m'}
    assert 'summary.json' in m['arquivos']


def test_status_abortada(tmp_path):
    w, pasta = _sessao(tmp_path)
    w.escrever_evento(1.0, 'ABORT', 'estado=ABORTADO')
    w.fechar('abortada')
    assert json.load(open(os.path.join(pasta, 'meta.json')))['status'] == 'abortada'
    assert json.load(open(os.path.join(pasta, 'summary.json')))['status'] == 'abortada'


def test_params_mudam_gera_evento(tmp_path):
    w, pasta = _sessao(tmp_path)
    w.atualizar_params(2.0, dict(PARAMS))             # igual: nada
    w.atualizar_params(3.0, {**PARAMS, 'mass_kg': 5.0})
    w.fechar('concluida')
    ev = list(csv.reader(open(os.path.join(pasta, 'events.csv'))))[1:]
    assert len(ev) == 1 and ev[0][1] == 'PARAMS' and ev[0][2] == 'mass_kg=5.0'
    assert json.load(open(os.path.join(pasta, 'meta.json')))['admitancia']['mass_kg'] == 5.0


def test_emg_opcional(tmp_path):
    w, pasta = _sessao(tmp_path, emg=2)
    w.escrever_emg(0.1, [0.5, 0.6, 0.7])
    w.fechar('concluida')
    assert _cab(os.path.join(pasta, 'emg.csv')) == ['t_s', 'emg_1_mv', 'emg_2_mv']
    assert len(open(os.path.join(pasta, 'emg.csv')).read().splitlines()) == 2


def test_flush_grava_antes_de_fechar(tmp_path):
    w, pasta = _sessao(tmp_path)
    w.escrever_tick(_tick(0.0))
    w.flush_periodico(5.0)
    assert len(open(os.path.join(pasta, 'samples.csv')).read().splitlines()) == 2
    w.fechar('interrompida')


def test_rejeita_sem_subject_e_sessao_dupla(tmp_path):
    w = SessionWriter(str(tmp_path))
    with pytest.raises(ValueError):
        w.abrir({'subject_id': ' '})
    w.abrir(META)
    with pytest.raises(RuntimeError):
        w.abrir(META)
    w.fechar('concluida')
    assert not os.listdir(tmp_path / 'S01') == []


def test_tick_tamanho_errado(tmp_path):
    w, _ = _sessao(tmp_path)
    with pytest.raises(ValueError):
        w.escrever_tick([0.0, 1.0])
    w.fechar('concluida')


def test_massas_mastro_conferem_urdf():
    urdf = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'joystick_tool.urdf')
    raiz = ET.parse(urdf).getroot()
    massas = {l.get('name'): float(l.find('inertial/mass').get('value'))
              for l in raiz.iter('link') if l.find('inertial/mass') is not None}
    valores = sorted(massas.values())
    assert any(math.isclose(v, C.GRIP_MASS_KG) for v in valores)
    assert any(math.isclose(v, C.PLATE_MASS_KG) for v in valores)
