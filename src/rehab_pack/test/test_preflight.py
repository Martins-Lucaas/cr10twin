"""Pré-voo do hardware: só leitura; testado com servidor TCP local e portas fechadas."""
import socket
import threading

import pytest

from rehab_pack import preflight as pf


def _servidor():
    s = socket.socket(); s.bind(('127.0.0.1', 0)); s.listen(1)
    threading.Thread(target=lambda: s.accept()[0].close(), daemon=True).start()
    return s, s.getsockname()[1]


def test_check_tcp_aceita_e_nao_envia_nada():
    s, porta = _servidor()
    try:
        assert pf.check_tcp('127.0.0.1', porta) == (True, 'aceitou a conexão')
    finally:
        s.close()


def test_check_tcp_porta_fechada():
    s = socket.socket(); s.bind(('127.0.0.1', 0)); porta = s.getsockname()[1]; s.close()
    ok, det = pf.check_tcp('127.0.0.1', porta, timeout=0.5)
    assert not ok and det


def test_same_subnet():
    assert pf.same_subnet('192.168.5.2', ['10.0.0.4', '192.168.5.100']) == '192.168.5.100'
    assert pf.same_subnet('192.168.5.2', ['192.168.1.20']) is None
    assert pf.same_subnet('lixo', ['192.168.5.100']) is None


def test_ft_vids_sao_os_do_ft_receiver():
    assert set(pf.FT_VIDS) == {0x1A86, 0x10C4, 0x0403, 0x067B}


def test_candidatos_seriais_filtram_por_vid(monkeypatch):
    class P:
        def __init__(self, d, v): self.device, self.vid, self.description = d, v, 'x'
    lp = pytest.importorskip('serial.tools.list_ports')
    monkeypatch.setattr(lp, 'comports', lambda: [P('/dev/ttyUSB0', 0x1A86), P('/dev/ttyS0', None),
                                                  P('/dev/ttyACM0', 0x2341)])
    assert [d for d, _ in pf.serial_candidates()] == ['/dev/ttyUSB0']


def test_run_checks_tem_nivel_nome_e_detalhe_e_nao_move_nada(monkeypatch):
    # nenhum robô: as portas falham, mas NADA além de abrir/fechar TCP é tentado
    monkeypatch.setattr(pf, 'check_tcp', lambda ip, p, timeout=2.0: (False, 'sem rota'))
    res = pf.run_checks('192.0.2.1', '/dev/nao_existe')
    assert res and all(n in ('OK', 'AVISO', 'FALHA') for n, _, _ in res)
    por = {nome: n for n, nome, _ in res}
    assert por['192.0.2.1:29999 (dashboard/ServoJ)'] == 'FALHA'
    assert por['porta da célula /dev/nao_existe'] == 'FALHA'
    assert any(n == 'AVISO' and 'E-Stop' in nome for n, nome, _ in res)


def test_main_retorna_1_com_falha(monkeypatch, capsys):
    monkeypatch.setattr(pf, 'check_tcp', lambda ip, p, timeout=2.0: (False, 'sem rota'))
    assert pf.main(['--robot-ip', '192.0.2.1', '--ft-port', '/dev/nao_existe']) == 1
    assert 'FALHA' in capsys.readouterr().out


def test_install_is_stale_detecta_copia(monkeypatch):
    import rehab_pack
    monkeypatch.setattr(rehab_pack, '__file__', '/ws/install/rehab_pack/lib/python3.10/site-packages/rehab_pack/__init__.py')
    monkeypatch.setattr(pf.os.path, 'realpath', lambda p: p)
    assert 'cópia instalada' in pf.install_is_stale()
    monkeypatch.setattr(rehab_pack, '__file__', '/ws/src/rehab_pack/rehab_pack/__init__.py')
    assert pf.install_is_stale() is None


def test_trava_do_touch_pack_vira_falha_e_nao_fica_segurada(monkeypatch, tmp_path):
    import fcntl
    import sys
    from touch_pack import pack_lock
    monkeypatch.setenv('CR10_PACK_LOCK_DIR', str(tmp_path))
    monkeypatch.setattr(pf, 'check_tcp', lambda ip, p, timeout=2.0: (False, 'sem rota'))

    def nivel():
        return {nome: n for n, nome, _ in pf.run_checks('192.0.2.1')}['trava do touch_pack']
    assert nivel() == 'OK'
    with open(tmp_path / 'touch_pack.lock', 'a') as f:
        fcntl.flock(f, fcntl.LOCK_SH)
        assert nivel() == 'FALHA'
    assert nivel() == 'OK'
    monkeypatch.setattr(pack_lock, '_held', {})
    pack_lock.acquire('touch_pack')         # o pré-voo não segurou nenhuma das duas
    for f in pack_lock._held.values():
        f.close()
    monkeypatch.setitem(sys.modules, 'touch_pack.pack_lock', None)
    assert nivel() == 'FALHA'
