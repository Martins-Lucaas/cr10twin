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
