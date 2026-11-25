"""Regressões da auditoria de 24/11/2026: coleta, resumo e tarefas."""
import csv
import errno
import json
import math
import os
import random
import time

import pytest

from rehab_pack import constants as C
from rehab_pack import session_logger as sl
from rehab_pack import session_stats as ss
from rehab_pack import tasks as T

META = {'subject_id': 'S01', 'lado_paretico': 'D', 'tarefa': 'livre', 'notas': ''}


def tick(t=0.0, estado='ATIVO', **kw):
    d = {**dict.fromkeys(C.TICK_FIELDS, 0.0), 't_s': t,
         'state': float(C.STATES.index(estado)), **kw}
    return [d[k] for k in C.TICK_FIELDS]


def le(pasta, nome):
    with open(os.path.join(pasta, nome)) as f:
        return json.load(f) if nome.endswith('.json') else list(csv.reader(f))[1:]


# ── R14, R15, XR19: arquivos ────────────────────────────────────────────────
def test_r14_meta_guarda_o_inicial_e_os_eventos_as_mudancas(tmp_path):
    w = sl.SessionWriter(str(tmp_path))
    p = w.abrir(META, {'mass_kg': 2.0, 'damping_ns_m': 40.0})
    w.atualizar_params(1.0, {'mass_kg': 3.0, 'damping_ns_m': 120.0})
    w.atualizar_params(2.0, {'mass_kg': 3.0, 'damping_ns_m': 90.0})
    w.fechar('concluida')
    assert le(p, 'meta.json')['admitancia'] == {'mass_kg': 2.0, 'damping_ns_m': 40.0}
    assert [e[2] for e in le(p, 'events.csv')] == ['mass_kg=3.0;damping_ns_m=120.0',
                                                   'damping_ns_m=90.0']


def test_r15_pico_de_uma_amostra_aparece_no_resumo(tmp_path):
    w = sl.SessionWriter(str(tmp_path))
    w.abrir(META)
    for i in range(10):
        w.escrever_tick(tick(0.03 * i, fx_n=2.08))
        w.escrever_wrench(0.03 * i, (14.0 if i == 5 else 1.0, 0, 0), (0, 0, 0), 'load_cell_link')
    w.escrever_wrench(1.0, (math.nan, 0, 0), (0, 0, 0), 'load_cell_link')
    r = w.fechar('concluida')
    assert r['totais']['f_pico_n'] == pytest.approx(14.0)


def test_xr19_nan_na_forca_da_json_valido(tmp_path):
    w = sl.SessionWriter(str(tmp_path))
    p = w.abrir(META)
    w.escrever_tick(tick(0.0, fx_n=3.0))
    w.escrever_tick(tick(0.03, fx_n=math.nan))
    r = w.fechar('concluida')
    assert r['totais']['f_media_n'] == pytest.approx(3.0)
    with open(os.path.join(p, 'summary.json')) as f:
        json.load(f, parse_constant=lambda c: pytest.fail(f'{c} não é JSON'))


@pytest.mark.parametrize('sid', ['___', '_A', 'A_'])
def test_xr19_id_valido_da_a_pasta_com_o_proprio_nome(tmp_path, sid):
    w = sl.SessionWriter(str(tmp_path))
    p = w.abrir({**META, 'subject_id': sid})
    assert os.path.basename(os.path.dirname(p)) == sid
    w.fechar('concluida')


def test_id_que_escapa_da_pasta_e_recusado(tmp_path):
    with pytest.raises(ValueError):
        sl.SessionWriter(str(tmp_path / 'x')).abrir({**META, 'subject_id': '../fora'})


# ── R18: disco cheio ────────────────────────────────────────────────────────
def cheio(monkeypatch, quando):
    """open() do logger falha com ENOSPC quando `quando(caminho, modo)`."""
    import builtins
    real = builtins.open

    def falso(f, modo='r', *a, **k):
        if 'w' in modo and quando(str(f)):
            raise OSError(errno.ENOSPC, 'No space left on device')
        return real(f, modo, *a, **k)
    monkeypatch.setattr(builtins, 'open', falso)


@pytest.mark.parametrize('nome', ['samples.csv', 'wrench.csv', 'events.csv', 'meta.json'])
def test_r18_falha_ao_abrir_nao_deixa_sessao_meio_aberta(tmp_path, monkeypatch, nome):
    w = sl.SessionWriter(str(tmp_path))
    with monkeypatch.context() as m:
        cheio(m, lambda f: nome in f)
        with pytest.raises(OSError):
            w.abrir(META)
    assert not w.aberta
    assert w.abrir(META)                           # espaço liberado: abre normalmente
    w.escrever_wrench(0.0, (1, 0, 0), (0, 0, 0), 'x')
    w.fechar('concluida')


def test_r18_falha_ao_fechar_preserva_dados_e_meta_e_permite_outra(tmp_path, monkeypatch):
    w = sl.SessionWriter(str(tmp_path))
    p = w.abrir(META)
    w.escrever_tick(tick(0.0))
    with monkeypatch.context() as m:
        cheio(m, lambda f: 'summary.json' in f or 'meta.json' in f)
        r = w.fechar('concluida')
    assert not w.aberta and 'erro_gravacao' in r
    assert le(p, 'meta.json')['status'] == 'interrompida'     # o meta de segurança sobrevive
    assert len(le(p, 'samples.csv')) == 1


class Cheio:
    """Arquivo cujo flush/close falha, como num disco cheio."""

    def __init__(self, f): self.f = f
    def flush(self): raise OSError(errno.ENOSPC, 'cheio')

    def close(self):
        self.f.close()
        raise OSError(errno.ENOSPC, 'cheio')


# ── nó de verdade: R16, R17, R18, R19, R21 ──────────────────────────────────
@pytest.fixture
def logger(tmp_path, monkeypatch):
    """Roda o corpo do teste dentro do main() do registrador."""
    rclpy = pytest.importorskip('rclpy')
    from rehab_pack import pack_lock
    monkeypatch.setattr(pack_lock, 'acquire_or_exit', lambda p: None)
    monkeypatch.setenv('REHAB_PACK_DATA_DIR', str(tmp_path))
    monkeypatch.setattr(sl, 'info_codigo', lambda: {})
    init = rclpy.init

    def roda(corpo):
        def spin(no):
            no.meta_usuario = dict(META)
            no.saidas, no.recs = [], []
            no.pub_resumo.publish = lambda m: no.saidas.append(json.loads(m.data))
            if hasattr(no, 'pub_rec'):
                no.pub_rec.publish = lambda m: no.recs.append(m.data)
            corpo(no)
        monkeypatch.setattr(rclpy, 'init', lambda args=None: init(args=args, domain_id=72))
        monkeypatch.setattr(rclpy, 'spin', spin)
        sl.main([])
    yield roda
    rclpy.try_shutdown()


def inicia(no):
    from std_srvs.srv import Trigger
    r = no._srv_start(None, Trigger.Response())
    assert r.success, r.message
    return r.message


def msg(v):
    from std_msgs.msg import Float64MultiArray, String
    return String(data=v) if isinstance(v, str) else Float64MultiArray(data=[float(x) for x in v])


def test_r17_aborto_guarda_motivo_e_fim_da_tarefa(logger):
    def corpo(no):
        p = inicia(no)
        no._tick(msg(tick(1.0)))
        no._tick(msg(tick(1.03, 'ABORTADO')))
        no._evento(msg('ABORT;motivo=pico de força 16.0 N'))
        no._evento(msg('TASK_END;motivo=interrompida;area_cm2=12.5'))
        assert no.w.aberta
        no._ronda()
        assert no.w.aberta                         # ainda na espera
        time.sleep(0.6)
        no._ronda()
        assert not no.w.aberta and no.saidas[-1]['status'] == 'abortada'
        det = [e[2] for e in le(p, 'events.csv')]
        assert 'motivo=pico de força 16.0 N' in det
        assert 'motivo=interrompida;area_cm2=12.5' in det
    logger(corpo)


def test_r17_encerrar_na_espera_continua_abortada(logger):
    def corpo(no):
        from std_srvs.srv import Trigger
        inicia(no)
        no._tick(msg(tick(1.0)))
        no._tick(msg(tick(1.03, 'ABORTADO')))
        assert no._srv_stop(None, Trigger.Response()).success
        assert no.saidas[-1]['status'] == 'abortada'
        time.sleep(1.1)                            # o nome da pasta tem 1 s de resolução
        p = inicia(no)                             # a seguinte não herda o aborto
        no._tick(msg(tick(2.0)))
        no._srv_stop(None, Trigger.Response())
        assert no.saidas[-1]['status'] == 'concluida' and p
    logger(corpo)


def test_r18_disco_cheio_no_meio_fecha_a_sessao_e_o_no_segue(logger):
    def corpo(no):
        inicia(no)
        no.w._arqs['samples.csv'] = Cheio(no.w._arqs['samples.csv'])
        no._ronda()
        assert not no.w.aberta and no.saidas[-1]['status'] == 'erro_disco'
        no._tick(msg(tick(1.0)))
        time.sleep(1.1)
        assert inicia(no)
    logger(corpo)


def test_r16_registrador_anuncia_o_que_esta_gravando(logger):
    def corpo(no):
        no._ronda()
        assert no.recs[-1] == ''
        p = inicia(no)
        no.rec_t = 0.0
        no._ronda()
        assert no.recs[-1] == p
    logger(corpo)


def real(no, **kw):
    no._real(msg(json.dumps({'modo': 'MIRROR', 'conectado': True, 'ip': '10.0.0.1', **kw})))


def test_r19_status_vencido_nao_prova_robo_real(logger):
    def corpo(no):
        no.params = {'force_source': 'real'}
        real(no)
        no.real_t -= 5.0                           # o rehab_mirror morreu
        p = inicia(no)
        assert le(p, 'meta.json')['forca']['simulado'] is True
    logger(corpo)


def test_r19_robo_conectado_no_meio_gera_evento(logger):
    def corpo(no):
        no.params = {'force_source': 'real'}
        real(no, conectado=False, modo='SIM_ONLY')
        p = inicia(no)
        no._tick(msg(tick(1.0)))
        real(no)
        no._tick(msg(tick(1.03)))
        no._tick(msg(tick(1.06)))
        no._fechar('concluida')
        ev = [e for e in le(p, 'events.csv') if e[1] == 'ROBO']
        assert len(ev) == 1 and 'modo=MIRROR;conectado=True' in ev[0][2]
        assert le(p, 'meta.json')['forca']['robo_mudou'] is True
    logger(corpo)


def test_r21_sessao_nova_nao_herda_o_alvo(logger):
    def corpo(no):
        no.alvo = (1.0, 2.0, 3.0)
        p = inicia(no)
        no._tick(msg(tick(1.0)))
        no._fechar('concluida')
        linha = dict(zip(C.SAMPLES_COLUMNS, le(p, 'samples.csv')[0]))
        assert linha['target_x_m'] == 'nan'
    logger(corpo)


def test_xr16_git_fora_do_servico_de_inicio(logger, monkeypatch):
    def corpo(no):
        monkeypatch.setattr(sl, 'info_codigo', lambda: pytest.fail('git dentro do serviço'))
        inicia(no)
    logger(corpo)


# ── R22: túnel começado antes da sessão ─────────────────────────────────────
def am(n=301):
    return {'t_s': [0.1 * i for i in range(n)],
            **{k: [0.0] * n for k in ('x_m', 'y_m', 'z_m', 'vx_ms', 'vy_ms', 'vz_ms',
                                      'fx_n', 'fy_n', 'fz_n')}}


@pytest.mark.parametrize('ev,pct', [
    ([(1.0, 'TUNNEL_EXIT', ''), (5.0, 'TUNNEL_ENTER', ''), (10.0, 'TASK_END', 'dentro_pct=60')], 60.0),
    ([(4.0, 'TUNNEL_ENTER', ''), (10.0, 'TASK_END', 'dentro_pct=60')], 60.0),   # já estava fora
    ([(10.0, 'TASK_END', 'dentro_pct=100')], 100.0)])
def test_r22_so_o_tempo_da_tarefa_conta(ev, pct):
    assert ss.resumir(am(), ev)['totais']['pct_tempo_no_tunel'] == pytest.approx(pct)


# ── R23, R24, R26: túnel ────────────────────────────────────────────────────
def percorre(tt, pts, t=0.0, tremor=0.0):
    rnd, ev = random.Random(1), []
    for x, y in pts:
        t += 0.03
        ev += tt.update(t, x + rnd.gauss(0, tremor), y + rnd.gauss(0, tremor))
        if tt.done:
            break
    return ev


def reta(a, b, n=60):
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n + 1)]


def novo(forma, **kw):
    tt = T.TunnelTask(forma, (0.5, 0.0), **{'size_m': 0.2, 'width_m': 0.04, **kw})
    tt.start(0.0)
    return tt


def test_r23_circulo_pelo_centro_nao_conclui():
    tt = novo('circulo')
    centro = (0.5, 0.1)
    percorre(tt, reta((0.5, 0.0), centro) + [centro] * 300, tremor=0.0005)
    assert not tt.done and tt.progress < 0.2


def test_r23_oito_em_linha_reta_nao_conclui():
    tt = novo('oito')
    for _ in range(3):
        percorre(tt, reta((0.5, 0.0), (0.5, 0.1)) + reta((0.5, 0.1), (0.5, -0.1))
                 + reta((0.5, -0.1), (0.5, 0.0)))
    assert not tt.done


@pytest.mark.parametrize('forma', ['reta', 'circulo', 'oito'])
@pytest.mark.parametrize('tremor', [0.0, 0.001])
def test_r23_percurso_legitimo_conclui(forma, tremor):
    tt = novo(forma)
    ev = percorre(tt, list(tt.path) + [tt.path[-1]] * 10, tremor=tremor)
    assert tt.done and ev[-1][1]['motivo'] == 'concluida' and tt.exits == 0


def test_r23_curva_por_dentro_do_tunel_conclui():
    tt = novo('circulo')
    c = (0.5, 0.1)
    pts = [(c[0] + (x - c[0]) * 0.85, c[1] + (y - c[1]) * 0.85) for x, y in tt.path]
    percorre(tt, pts + [tt.path[-1]] * 10)
    assert tt.done


def test_r23_sair_e_voltar_mais_a_frente_na_reta_continua():
    tt = novo('reta')
    percorre(tt, reta((0.5, 0.0), (0.5, 0.05)) + reta((0.5, 0.05), (0.55, 0.05))
             + reta((0.55, 0.05), (0.55, 0.15)) + reta((0.55, 0.15), (0.5, 0.15))
             + reta((0.5, 0.15), (0.5, 0.2)))
    assert tt.done and tt.exits == 1


def test_r24_ruido_na_borda_nao_conta_saidas():
    tt = novo('reta')
    ev = percorre(tt, [(0.52, 0.0)] * 200, tremor=0.0003)
    assert tt.exits <= 1 and len(ev) <= 2
    ev = percorre(tt, [(0.53, 0.0)] + [(0.5, 0.0)], t=10.0)     # saída de verdade conta
    assert tt.exits >= 1 and ev[-1][0] == 'TUNNEL_ENTER'


def test_r26_inicio_da_tarefa_registra_os_parametros():
    co = T.CenterOutTask((0.5, 0.0), reach_m=0.08, target_radius_m=0.03, timeout_s=7.0, seed=9)
    d = co.start(0.0)[0][1]
    assert (d['alcance'], d['raio'], d['limite_s'], d['semente'], d['permanencia_s']) == (
        0.08, 0.03, 7.0, 9, 0.5)
    d = novo('circulo', size_m=0.15).start(0.0)[0][1]
    assert d['tamanho'] == 0.15 and d['largura'] == 0.04
    assert T.ReachTask((0.5, 0.0)).start(0.0)[0][1]['celula'] == 0.01
