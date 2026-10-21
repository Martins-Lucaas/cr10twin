"""Tabela dos sensores de toque: o que separa o 4×4 do 5×5."""
import os

from touch_pack.constants import TOUCH_SENSORS, touch_sensor_spec

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))


def test_cada_sensor_tem_a_sua_grade_e_o_seu_escalar():
    s4, s5 = touch_sensor_spec('4'), touch_sensor_spec('5')
    assert (s4.rows, s4.cols, s4.has_total) == (4, 4, True)
    assert (s5.rows, s5.cols, s5.has_total) == (5, 5, False)


def test_valor_desconhecido_cai_no_5x5():
    for v in ('', '3', 'x', None, ' 5 '):
        assert touch_sensor_spec(v).key == '5'
    assert touch_sensor_spec(4).key == '4'


def test_a_pasta_de_cada_sensor_existe_no_repo():
    for spec in TOUCH_SENSORS.values():
        assert os.path.isdir(os.path.join(_REPO, spec.folder)), spec.folder
