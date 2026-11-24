"""Testes da lógica pura de tasks.py (sem Tk, sem ROS)."""
import math

import pytest

from rehab_pack import tasks
from rehab_pack.constants import EVENT_TYPES


def tipos(ev):
    return [e[0] for e in ev]


def test_ordem_deterministica_e_completa():
    a = tasks.shuffled_order(8, 3)
    assert a == tasks.shuffled_order(8, 3)
    assert sorted(a) == list(range(8))
    assert a != tasks.shuffled_order(8, 4)


def test_formato_evento():
    s = tasks.format_event('TARGET_HIT', alvo=3, t=1.23456789, txt='a;b=c')
    assert s == 'TARGET_HIT;alvo=3;t=1.235;txt=a b c'
    with pytest.raises(ValueError):
        tasks.format_event('INVENTADO')
    t = tasks.CenterOutTask((0.5, 0.0))
    for _ in range(3):
        for tp, kv in t.start(0.0):
            assert tp in EVENT_TYPES


def test_acerto_so_apos_permanencia():
    t = tasks.CenterOutTask((0.5, 0.0), reach_m=0.1, target_radius_m=0.02,
                            hold_s=0.5, seed=1)
    assert tipos(t.start(0.0)) == ['TASK_START', 'TARGET_ON']
    px, py = t.current_target()
    assert t.update(0.1, px, py) == []          # entrou, ainda não vale
    assert t.update(0.4, px, py) == []
    assert tipos(t.update(0.61, px, py))[0] == 'TARGET_HIT'
    assert t.hits == 1


def test_sair_do_alvo_zera_permanencia():
    t = tasks.CenterOutTask((0.5, 0.0), hold_s=0.5, seed=1)
    t.start(0.0)
    px, py = t.current_target()
    t.update(0.1, px, py)
    t.update(0.4, px + 0.1, py)                 # saiu
    assert t.update(0.7, px, py) == []          # recomeça a contar
    assert tipos(t.update(1.3, px, py))[0] == 'TARGET_HIT'


def test_timeout_gera_miss_e_avanca():
    t = tasks.CenterOutTask((0.5, 0.0), timeout_s=5.0, seed=2)
    t.start(0.0)
    primeiro = t.current_target()
    ev = t.update(5.1, 0.5, 0.0)
    assert tipos(ev) == ['TARGET_MISS', 'TARGET_ON']
    assert t.misses == 1 and t.current_target() != primeiro


def test_centro_fora_completo():
    t = tasks.CenterOutTask((0.5, 0.0), seed=5)
    t.start(0.0)
    now = 0.0
    todos = []
    while not t.done:
        now += 0.6
        px, py = t.current_target()
        todos += tipos(t.update(now, px, py))
        todos += tipos(t.update(now + 0.55, px, py))
        now += 0.55
    assert t.hits == 8 and t.misses == 0
    assert todos.count('TARGET_HIT') == 16 and todos[-1] == 'TASK_END'
    assert t.update(99.0, 0.0, 0.0) == []


def test_alvos_a_raio_correto_e_distintos():
    t = tasks.CenterOutTask((0.5, 0.1), reach_m=0.12)
    for p in t.target_positions:
        assert math.dist(p, (0.5, 0.1)) == pytest.approx(0.12)
    assert len({tuple(round(c, 6) for c in p) for p in t.target_positions}) == 8


def test_trajetorias_comecam_no_start():
    for forma in ('reta', 'circulo', 'oito'):
        p = tasks.tunnel_path(forma, 0.2, (0.5, 0.0))
        assert math.dist(p[0], (0.5, 0.0)) < 1e-9
    c = tasks.tunnel_path('circulo', 0.2, (0.5, 0.0))
    assert math.dist(c[0], c[-1]) < 1e-6        # fechado


def test_tunel_saida_e_entrada():
    tt = tasks.TunnelTask('reta', (0.5, 0.0), size_m=0.2, width_m=0.04)
    tt.start(0.0)
    assert tt.update(0.1, 0.5, 0.01) == []      # dentro (<2 cm)
    ev = tt.update(0.2, 0.55, 0.05)             # 5 cm de lado
    assert tipos(ev) == ['TUNNEL_EXIT'] and tt.exits == 1
    assert tt.update(0.3, 0.55, 0.06) == []     # continua fora: sem repetir
    assert tipos(tt.update(0.4, 0.5, 0.07)) == ['TUNNEL_ENTER']
    assert 0.0 < tt.inside_pct < 100.0


def test_tunel_conclui_e_assistencia_a_frente():
    tt = tasks.TunnelTask('reta', (0.5, 0.0), size_m=0.2, width_m=0.04)
    tt.start(0.0)
    t, y, fim = 0.0, 0.0, []
    while not tt.done and y < 0.3:
        t += 0.03
        y += 0.002
        fim += tipos(tt.update(t, 0.5, y))
    assert tt.done and fim[-1] == 'TASK_END' and tt.exits == 0
    tt2 = tasks.TunnelTask('reta', (0.5, 0.0), size_m=0.2)
    tt2.start(0.0)
    tt2.update(0.1, 0.5, 0.05)
    assert tt2.reference()[1] > 0.05            # referência à frente do cursor


def test_oito_nao_pula_o_cruzamento():
    tt = tasks.TunnelTask('oito', (0.5, 0.0), size_m=0.2, width_m=0.04)
    tt.start(0.0)
    # Cursor parado no ponto de cruzamento no início: progresso não salta p/ o fim.
    tt.update(0.1, 0.5, 0.0)
    assert tt.progress < 0.2


def test_area_explorada():
    a = tasks.ExploredArea((0.5, 0.0), half_m=0.25, cell_m=0.01)
    assert a.add(0.5, 0.0) and not a.add(0.5001, 0.0001)
    assert a.add(0.52, 0.0)
    assert a.area_m2 == pytest.approx(2e-4)
    assert not a.add(0.9, 0.0)                  # fora da grade
    r = tasks.ReachTask((0.5, 0.0), duration_s=2.0)
    r.start(0.0)
    r.update(0.5, 0.5, 0.0)
    assert tipos(r.update(2.1, 0.5, 0.1)) == ['TASK_END']


def test_estabilidade_e_presets():
    assert tasks.stability(2.0, 40.0, 0.0)['ok']
    assert not tasks.stability(0.5, 40.0, 0.0)['ok']     # B·dt/M = 2,4
    assert not tasks.stability(2.0, 0.0, 1e5)['ok']      # K altíssimo
    for nome, p in tasks.PRESETS.items():
        assert tasks.stability(p['mass_kg'], p['damping_ns_m'],
                               p['stiffness_n_m'])['ok'], nome
    assert tasks.PRESETS['Assistido']['stiffness_n_m'] > 0
    assert tasks.abort_limit_ok(15.0) and not tasks.abort_limit_ok(15.1)


def test_forca_mao_virtual():
    assert tasks.keys_to_force({'w'}, 5.0) == (-5.0, 0.0, 0.0)
    f = tasks.keys_to_force({'s', 'd', 'q'}, 10.0)
    assert math.sqrt(sum(c * c for c in f)) <= 10.0 + 1e-9
    assert tasks.keys_to_force({'a', 'd'}, 5.0) == (0.0, 0.0, 0.0)
    f = tasks.drag_to_force(500, 0, 90, 10.0)             # saturado
    assert f == pytest.approx((0.0, 10.0, 0.0))
    assert sum(c * c for c in tasks.clamp_force((30, 40, 0), 10.0)) == pytest.approx(100.0)


def test_decimacao_preserva_pico():
    vs = [0.0] * 1000
    vs[437] = 9.0
    ts = list(range(1000))
    d = tasks.decimate_minmax(ts, vs, 100)
    assert len(d) <= 200 and max(v for _, v in d) == 9.0
    assert [t for t, _ in d] == sorted(t for t, _ in d)
    assert tasks.decimate_minmax([0, 1], [1.0, 2.0], 100) == [(0, 1.0), (1, 2.0)]


def test_id_anonimo():
    assert tasks.validate_subject_id('S01')
    assert not tasks.validate_subject_id('')
    assert not tasks.validate_subject_id('João Silva')
    assert not tasks.validate_subject_id('a' * 17)
