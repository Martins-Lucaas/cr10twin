"""Testes do mundo rehab_lab.world: física e mesa intactas, decoração só visual."""
import math
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

WORLD = os.path.join(os.path.dirname(__file__), '..', 'worlds', 'rehab_lab.world')
RAIO_LIVRE = 1.6          # m, ao redor do robô
ROBO_XY = (0.30, 0.0)


@pytest.fixture(scope='module')
def world():
    return ET.parse(WORLD).getroot().find('world')


def _nums(txt):
    return [float(v) for v in txt.split()]


def test_fisica_exata(world):
    ph = world.find('physics')
    assert ph.get('type') == 'ode'
    assert float(ph.findtext('max_step_size')) == 0.004
    assert float(ph.findtext('real_time_update_rate')) == 250
    s = ph.find('ode/solver')
    assert s.findtext('type') == 'quick'
    assert int(s.findtext('iters')) == 50
    assert float(s.findtext('sor')) == 1.3
    c = ph.find('ode/constraints')
    assert float(c.findtext('cfm')) == 1e-5
    assert float(c.findtext('erp')) == 0.25


def test_mesa_exata(world):
    mesa = [m for m in world.findall('model') if m.get('name') == 'lab_table'][0]
    top = mesa.find("link[@name='top']")
    assert _nums(top.findtext('pose')) == [0.30, 0, 0.725, 0, 0, 0]
    assert _nums(top.findtext('collision/geometry/box/size')) == [0.60, 1.10, 0.05]
    assert len(mesa.findall('link')) == 5          # tampo + 4 pernas
    piso = [m for m in world.findall('model') if m.get('name') == 'floor'][0]
    assert piso.find('link/collision/geometry/plane/normal').text.split() == ['0', '0', '1']


def test_sem_model_uri_nem_http():
    txt = open(WORLD, encoding='utf-8').read()
    assert 'model://' not in txt
    assert 'http' not in txt.lower()


def test_tudo_static_e_muitos_modelos(world):
    modelos = world.findall('model')
    assert len(modelos) >= 20
    for m in modelos:
        assert m.findtext('static') == 'true', m.get('name')


def _caixa_colisao(model, col):
    """Bounding box XY (xmin, xmax, ymin, ymax) de uma colisão tipo box, sem rotação."""
    p = _nums(col.findtext('pose') or '0 0 0 0 0 0')
    lp = model.find('link').findtext('pose')
    if lp:
        p = [a + b for a, b in zip(p, _nums(lp))]
    sx, sy, _ = _nums(col.findtext('geometry/box/size'))
    return p[0] - sx / 2, p[0] + sx / 2, p[1] - sy / 2, p[1] + sy / 2


def test_decoracao_sem_colisao_perto_do_robo(world):
    ok = {'floor', 'lab_table'}
    for m in world.findall('model'):
        nome = m.get('name')
        cols = m.findall('.//collision')
        if nome in ok:
            continue
        if not nome.startswith('wall_'):
            assert not cols, f'{nome} tem colisão (decoração deve ser só visual)'
            continue
        for c in cols:
            x0, x1, y0, y1 = _caixa_colisao(m, c)
            dx = max(x0 - ROBO_XY[0], 0, ROBO_XY[0] - x1)
            dy = max(y0 - ROBO_XY[1], 0, ROBO_XY[1] - y1)
            assert math.hypot(dx, dy) > RAIO_LIVRE, nome


@pytest.mark.skipif(shutil.which('gz') is None, reason='gz ausente')
def test_gz_sdf_valido():
    r = subprocess.run(['gz', 'sdf', '-k', WORLD], capture_output=True, text=True, timeout=60)
    assert r.returncode == 0 and 'Check complete' in r.stdout, r.stdout + r.stderr


VOLUNTARIO_XY = (1.20, 0.0)
RAIO_VOLUNTARIO = 0.6     # m, cadeira + manequim (pés em x ~ 0,76)


def _rot(r, p, y):
    """Matriz de rotação RPY (eixos fixos, como no SDF)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def _raio_geom(g):
    """Raio da esfera que envolve a geometria."""
    if g.find('box') is not None:
        return math.hypot(*_nums(g.findtext('box/size'))) / 2
    if g.find('cylinder') is not None:
        return math.hypot(float(g.findtext('cylinder/radius')), float(g.findtext('cylinder/length')) / 2)
    return float(g.findtext('sphere/radius'))


def _dist_seg(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L))
    return math.hypot(a[0] + t * dx - p[0], a[1] + t * dy - p[1])


def _waypoints(actor):
    return [(float(w.findtext('time')), _nums(w.findtext('pose')))
            for w in actor.findall('script/trajectory/waypoint')]


def _folga_minima(actor, centro):
    """Menor distância XY (descontado o raio de cada visual) entre `centro` e o
    ator ao longo de toda a trajetória, inclusive o salto do fim para o início
    do laço. Se a orientação muda no trecho, usa o pivô do ator e soma o
    alcance do visual (cobre qualquer giro)."""
    wps = [p for _, p in _waypoints(actor)]
    segs = list(zip(wps, wps[1:] + wps[:1]))
    folga = math.inf
    for v in actor.findall('link/visual'):
        lp = _nums(v.findtext('pose') or '0 0 0 0 0 0')
        rg = _raio_geom(v.find('geometry'))
        for a, b in segs:
            if a[3:] == b[3:]:
                R = _rot(*a[3:])
                off = [sum(R[i][j] * lp[j] for j in range(3)) for i in range(2)]
                d = _dist_seg(centro, (a[0] + off[0], a[1] + off[1]), (b[0] + off[0], b[1] + off[1])) - rg
            else:
                # só guinada muda: alcance horizontal basta; senão, alcance 3D
                alc = lp[:2] if a[3:5] == b[3:5] == [0, 0] else lp[:3]
                d = _dist_seg(centro, a[:2], b[:2]) - math.hypot(*alc) - rg
            folga = min(folga, d)
    return folga


@pytest.fixture(scope='module')
def atores(world):
    return world.findall('actor')


def test_atores_existem_em_laco(atores):
    assert len(atores) >= 6
    for a in atores:
        nome = a.get('name')
        assert a.findtext('script/loop') == 'true', nome
        assert a.findtext('script/auto_start') == 'true', nome
        assert a.find('skin') is None, nome
        assert a.findall('link/visual'), nome
        ts = [t for t, _ in _waypoints(a)]
        assert len(ts) >= 2 and ts[0] == 0, nome
        assert all(t1 > t0 for t0, t1 in zip(ts, ts[1:])), nome


def test_atores_sem_colisao(atores):
    for a in atores:
        assert not a.findall('.//collision'), a.get('name')


def test_atores_longe_do_robo_e_do_voluntario(atores):
    for a in atores:
        nome = a.get('name')
        assert _folga_minima(a, ROBO_XY) >= RAIO_LIVRE, nome
        assert _folga_minima(a, VOLUNTARIO_XY) >= RAIO_VOLUNTARIO, nome


def test_giros_de_no_maximo_90_graus_por_waypoint(atores):
    """A interpolação de orientação pega o caminho curto: passos > 90° giram errado."""
    for a in atores:
        qs = [_quat(*p[3:]) for _, p in _waypoints(a)]
        for q0, q1 in zip(qs, qs[1:]):
            ang = 2 * math.acos(min(1.0, abs(sum(x * y for x, y in zip(q0, q1)))))
            assert ang <= math.pi / 2 + 1e-3, a.get('name')


def test_detalhes_da_mesa_so_visuais(world):
    extras = [m for m in world.findall('model') if m.get('name') == 'lab_table_extras']
    assert len(extras) == 1
    assert not extras[0].findall('.//collision')
    assert len(extras[0].findall('.//visual')) >= 20


def test_apenas_duas_paredes_fundo_e_direita_do_voluntario(world):
    """Voluntário em (1,2; 0) virado para -x: a direita dele é +y. Lado -y aberto."""
    nomes = sorted(m.get('name') for m in world.findall('model') if m.get('name').startswith('wall_')
                   and m.findall('.//collision'))
    assert nomes == ['wall_back', 'wall_right_volunteer']
    por_nome = {m.get('name'): m for m in world.findall('model')}
    x0, x1, y0, y1 = _caixa_colisao(por_nome['wall_back'], por_nome['wall_back'].find('.//collision'))
    assert (x0 + x1) / 2 == pytest.approx(-1.65)
    m = por_nome['wall_right_volunteer']
    x0, x1, y0, y1 = _caixa_colisao(m, m.find('.//collision'))
    assert (y0 + y1) / 2 == pytest.approx(2.25)
    for m in world.findall('model'):
        for c in m.findall('.//collision'):
            if c.find('geometry/box') is not None:
                _, _, ya, yb = _caixa_colisao(m, c)
                assert abs((ya + yb) / 2 + 2.25) > 0.5, m.get('name')   # nada em y = -2,25
