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


def test_relatorio_do_touch_ignora_sessoes_do_rehab(tmp_path, monkeypatch):
    from touch_pack import palpation_report as pr
    vistos = []
    monkeypatch.setattr(pr, 'OUTPUT_DIR', str(tmp_path))
    monkeypatch.setattr(pr, 'generate_report',
                        lambda p, **k: vistos.append(p) or {})
    touch = tmp_path / 'SLIDE' / 'r1' / pr.RUN_SAMPLES_CSV
    rehab = tmp_path / 'REHAB' / 'S01' / 'r2' / pr.RUN_SAMPLES_CSV
    for p in (touch, rehab):          # o do rehab é o mais recente
        p.parent.mkdir(parents=True)
        p.write_text('')
    import os
    os.utime(touch, (1, 1))
    try:
        pr.main(['--latest', '--no-plot'])
    except (KeyError, SystemExit):
        pass                          # o resumo falso não tem campos
    assert vistos == [str(touch)]
