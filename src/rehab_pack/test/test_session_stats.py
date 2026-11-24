"""Testes de session_stats com trajetórias sintéticas (sem ROS)."""
import math

from rehab_pack.session_stats import contar_picos, metricas_alcance, resumir


def _minjerk(s):
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


def _reta(D=0.2, T=1.0, dt=0.03):
    n = int(T / dt) + 1
    t = [i * dt for i in range(n)]
    x = [D * _minjerk(ti / T) for ti in t]
    vx = [D / T * (30 * (ti / T) ** 2 - 60 * (ti / T) ** 3 + 30 * (ti / T) ** 4) for ti in t]
    return t, x, vx


def test_reta_retilinearidade_1_um_pico():
    t, x, vx = _reta()
    m = metricas_alcance(t, [(xi, 0.0, 0.0) for xi in x], [(v, 0.0, 0.0) for v in vx])
    assert math.isclose(m['retilinearidade'], 1.0, abs_tol=1e-9)
    assert m['n_picos_velocidade'] == 1
    assert math.isclose(m['comprimento_caminho_m'], 0.2, abs_tol=1e-4)
    assert math.isclose(m['pico_velocidade_ms'], 0.2 * 1.875, rel_tol=0.02)


def test_dois_movimentos_dois_picos():
    t, x, vx = _reta()
    t2 = [ti + 1.0 + 0.03 for ti in t]
    tt = t + t2
    xx = x + [0.2 + xi for xi in x]
    vv = vx + vx
    m = metricas_alcance(tt, [(a, 0.0, 0.0) for a in xx], [(v, 0.0, 0.0) for v in vv])
    assert m['n_picos_velocidade'] == 2


def test_arco_retilinearidade_menor_que_1():
    n = 50
    pts = [(math.sin(math.pi * i / (n - 1)), 1 - math.cos(math.pi * i / (n - 1)), 0.0)
           for i in range(n)]
    m = metricas_alcance([i * 0.03 for i in range(n)], pts, [(0.1, 0, 0)] * n)
    assert m['retilinearidade'] < 0.7


def test_contar_picos_ruido_pequeno_ignorado():
    v = [0, 1, 0.98, 1.0, 0.99, 0]
    assert contar_picos(v, 0.1) == 1


def _amostras(t, x, vx, f=(1.0, 2.0, 2.0)):
    n = len(t)
    return {'t_s': t, 'x_m': x, 'y_m': [0.0] * n, 'z_m': [0.2] * n,
            'vx_ms': vx, 'vy_ms': [0.0] * n, 'vz_ms': [0.0] * n,
            'fx_n': [f[0]] * n, 'fy_n': [f[1]] * n, 'fz_n': [f[2]] * n}


def test_resumo_alcance_totais_e_tunel():
    t, x, vx = _reta()
    am = _amostras(t, x, vx)
    ev = [(0.0, 'TARGET_ON', 'id=1'), (0.3, 'TUNNEL_EXIT', ''),
          (0.6, 'TUNNEL_ENTER', ''), (1.0, 'TARGET_HIT', 'id=1'),
          (1.0, 'TARGET_ON', 'id=2')]  # alcance aberto é descartado
    r = resumir(am, ev)
    assert len(r['alcances']) == 1
    assert r['alcances'][0]['resultado'] == 'acerto'
    assert math.isclose(r['alcances'][0]['retilinearidade'], 1.0, abs_tol=1e-9)
    tot = r['totais']
    assert tot['acertos'] == 1 and tot['erros'] == 0
    assert math.isclose(tot['f_media_n'], 3.0) and math.isclose(tot['f_pico_n'], 3.0)
    assert math.isclose(tot['pct_tempo_no_tunel'], 100 * (1 - 0.3 / t[-1]), rel_tol=1e-9)


def test_sem_tunel_none_e_erro_contado():
    t, x, vx = _reta()
    r = resumir(_amostras(t, x, vx), [(0.0, 'TARGET_ON', ''), (0.5, 'TARGET_MISS', '')])
    assert r['totais']['pct_tempo_no_tunel'] is None
    assert r['totais']['erros'] == 1
    assert r['alcances'][0]['resultado'] == 'erro'


def test_sessao_vazia():
    r = resumir({}, [])
    assert r['alcances'] == [] and r['totais']['n_amostras'] == 0


def test_metricas_usam_o_robo_real_quando_gravado():
    t, x, vx = _reta()
    am = _amostras(t, x, [9.0] * len(t))             # setpoint absurdo: não pode ser usado
    am.update(xr_m=[0.5 * v for v in x], yr_m=[0.0] * len(t), zr_m=[0.2] * len(t))
    ev = [(0.0, 'TARGET_ON', ''), (1.0, 'TARGET_HIT', '')]
    a = resumir(am, ev)['alcances'][0]
    assert a['posicao'] == 'real' and a['n_picos_velocidade'] == 1
    assert math.isclose(a['comprimento_caminho_m'], 0.1, abs_tol=1e-3)
    assert math.isclose(a['pico_velocidade_ms'], 0.1 * 1.875, rel_tol=0.05)
    am['xr_m'][3] = float('nan')
    assert resumir(am, ev)['alcances'][0]['posicao'] == 'simulada'


def test_tunel_nao_conta_intervalo_entre_tarefas():
    t = [0.1 * i for i in range(301)]
    ev = [(0.0, 'TASK_START', 'tarefa=tunel_reta'), (2.0, 'TUNNEL_EXIT', ''), (4.0, 'TUNNEL_ENTER', ''),
          (10.0, 'TASK_END', ''), (20.0, 'TASK_START', 'tarefa=tunel_reta'), (30.0, 'TASK_END', '')]
    r = resumir(_amostras(t, [0.0] * 301, [0.0] * 301), ev)
    assert math.isclose(r['totais']['pct_tempo_no_tunel'], 90.0)
