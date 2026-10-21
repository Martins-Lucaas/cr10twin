"""Texto que a aba "6 Axes" mostra para cada desfecho de /load_cell/tare_result."""
from std_msgs.msg import String

from touch_pack.gui_loadcell import FtAxesMixin


class _Host:
    def __init__(self):
        self.status = []

    def _set_status(self, texto, _cor=None):
        self.status.append(texto)


def _tare_result(texto):
    h = _Host()
    FtAxesMixin._cb_lc_tare_result(h, String(data=texto))
    return ' '.join(h.status)


def test_tare_ok_mostra_a_referencia():
    assert '+1.250 N' in _tare_result('ok;1.25;0.01')


def test_sem_dado_aponta_para_a_485_e_os_24V():
    assert '24 V' in _tare_result('err;no_data;0')


def test_deriva_recusa_o_tare():
    assert 'drifting' in _tare_result('err;drifting;0.8')


def test_mensagem_desconhecida_nao_diz_nada():
    assert _tare_result('lixo') == ''
