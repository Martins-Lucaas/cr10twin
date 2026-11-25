"""Trava touch_pack × rehab_pack, com processos de verdade."""
import subprocess
import sys

import pytest

from touch_pack import pack_lock

_HOLD = ('import sys; from touch_pack.pack_lock import acquire; acquire(sys.argv[1]);'
         'print("ok", flush=True); sys.stdin.read()')


@pytest.fixture
def hold(tmp_path, monkeypatch):
    monkeypatch.setenv('CR10_PACK_LOCK_DIR', str(tmp_path))
    monkeypatch.setattr(pack_lock, '_held', {})
    procs = []

    def _hold(pack):
        p = subprocess.Popen([sys.executable, '-c', _HOLD, pack], text=True,
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        procs.append(p)
        assert p.stdout.readline().strip() == 'ok'
        return p
    yield _hold
    for p in procs:
        p.kill()
        p.wait()


@pytest.mark.parametrize('mine,other', [pack_lock.PACKS, pack_lock.PACKS[::-1]])
def test_other_pack_open_blocks_until_it_dies(hold, mine, other):
    p = hold(other)
    with pytest.raises(pack_lock.PackLockError, match=other):
        pack_lock.acquire(mine)
    with pytest.raises(SystemExit):
        pack_lock.acquire_or_exit(mine)
    p.kill()          # morte abrupta: sem trava órfã
    p.wait()
    pack_lock.acquire(mine)


def test_nodes_of_same_pack_coexist(hold):
    hold('touch_pack')
    hold('touch_pack')
    pack_lock.acquire('touch_pack')
    pack_lock.acquire('touch_pack')


def test_unknown_pack_is_refused():
    with pytest.raises(ValueError):
        pack_lock.acquire('outro')


# ── R01: todo produtor de comando do braço toma a trava ─────────────────────
@pytest.mark.parametrize('mod', ['hand_pack.combined_gui', 'grasp_ml_pack.manual_control_node',
                                 'grasp_ml_pack.grasp_executor'])
def test_r01_produtores_recusam_com_o_rehab_aberto(mod, monkeypatch, tmp_path):
    import fcntl
    from touch_pack import pack_lock
    rclpy = pytest.importorskip('rclpy')
    m = pytest.importorskip(mod)
    monkeypatch.setenv('CR10_PACK_LOCK_DIR', str(tmp_path))
    monkeypatch.setattr(pack_lock, '_held', {})

    def subiu(*a, **k):
        raise AssertionError('subiu com o rehab aberto')
    monkeypatch.setattr(rclpy, 'init', subiu)
    if hasattr(m, 'tk'):
        monkeypatch.setattr(m.tk, 'Tk', subiu)
    with open(tmp_path / 'rehab_pack.lock', 'a') as f:
        fcntl.flock(f, fcntl.LOCK_SH)
        with pytest.raises(SystemExit):
            m.main()
    with open(tmp_path / 'touch_pack.lock', 'a') as f:       # touch aberto: convive
        fcntl.flock(f, fcntl.LOCK_SH)
        with pytest.raises(AssertionError):
            m.main()


@pytest.mark.parametrize('rel', ['hand_pack/launch/cr10_covvi_gazebo.launch.py',
                                 'grasp_ml_pack/launch/conveyor_cell.launch.py'])
def test_r01_launches_com_controlador_do_braco_recusam(rel, monkeypatch, tmp_path):
    import fcntl
    import importlib.util
    import pathlib
    pytest.importorskip('launch')
    from touch_pack import pack_lock
    f = pathlib.Path(__file__).parents[2] / rel
    if not f.exists():
        pytest.skip(f'{rel} fora desta árvore')
    spec = importlib.util.spec_from_file_location('l', f)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except ImportError as e:
        pytest.skip(str(e))
    monkeypatch.setenv('CR10_PACK_LOCK_DIR', str(tmp_path))
    monkeypatch.setattr(pack_lock, '_held', {})
    with open(tmp_path / 'rehab_pack.lock', 'a') as h:
        fcntl.flock(h, fcntl.LOCK_SH)
        with pytest.raises(pack_lock.PackLockError):
            mod.generate_launch_description()
