"""Vigia de perda de contato do SLIDING (_ContactWatch)."""
import pytest

pytest.importorskip('rclpy')


@pytest.fixture(scope='module')
def W():
    from touch_pack.tactile_explorer import TactileExplorer
    return TactileExplorer._ContactWatch


def _floor(W, target=1.5):
    from touch_pack.tactile_explorer import (
        _SLIDE_LOST_BUDGET_M, _SLIDE_CONTACT_MIN_FRAC)
    return W(target, _SLIDE_LOST_BUDGET_M, _SLIDE_CONTACT_MIN_FRAC)


def test_floor_scales_with_the_setpoint(W):
    from touch_pack.tactile_explorer import (
        _CONTACT_ON_N, _SLIDE_CONTACT_MIN_FRAC, _SLIDE_LOST_BUDGET_M)
    w = W(4.0, _SLIDE_LOST_BUDGET_M, _SLIDE_CONTACT_MIN_FRAC)
    assert w.floor_n == pytest.approx(4.0 * _SLIDE_CONTACT_MIN_FRAC)
    # Setpoint baixo não pode empurrar o piso para dentro do ruído.
    w_low = W(0.2, _SLIDE_LOST_BUDGET_M, _SLIDE_CONTACT_MIN_FRAC)
    assert w_low.floor_n == _CONTACT_ON_N


def test_contact_in_range_never_trips(W):
    w = _floor(W)
    for i in range(500):
        assert not w.exceeded(i * 1e-4, 1.5)
    assert w.worst_m == 0.0 and w.lost_at is None


def test_short_valley_is_data_not_failure(W):
    """Vale de 2 mm com o orçamento de 5 mm: não aborta e o contador zera."""
    w = _floor(W)
    for i in range(20):                       # 0 → 2 mm sem contato
        assert not w.exceeded(i * 1e-4, 0.02)
    assert w.update(20e-4, 1.5) == 0.0        # voltou o contato
    for i in range(20, 40):                   # outros 2 mm sem contato
        assert not w.exceeded(i * 1e-4, 0.02)
    assert w.worst_m < 5e-3


def test_sustained_loss_trips_after_the_budget(W):
    from touch_pack.tactile_explorer import _SLIDE_LOST_BUDGET_M
    w = _floor(W)
    tripped_at = None
    for i in range(200):
        d = i * 1e-4
        if w.exceeded(d, 0.05):
            tripped_at = d
            break
    assert tripped_at is not None
    assert tripped_at > _SLIDE_LOST_BUDGET_M
    assert tripped_at <= _SLIDE_LOST_BUDGET_M + 2e-4


def test_replays_the_run_that_was_reported_ok(W):
    """20260806_170115: contato bom até 0,74 mm, depois ~0,05 N até 50 mm.
    A guarda tem de abortar, e perto de onde a força caiu."""
    w = _floor(W, target=1.5)
    tripped_at = None
    d = 0.0
    while d < 0.050:
        fz = 1.50 if d < 0.00074 else 0.05
        if w.exceeded(d, fz):
            tripped_at = d
            break
        d += 1e-4
    assert tripped_at is not None, 'a guarda deixou passar o run inteiro'
    assert w.lost_at == pytest.approx(0.0008, abs=1e-4)
    assert tripped_at < 0.007, 'abortou tarde demais — mediria ar por 6 mm+'


def test_tension_counts_as_contact(W):
    """A guarda olha |fz|: tração é contato (e é informação), não ausência."""
    w = _floor(W)
    for i in range(200):
        assert not w.exceeded(i * 1e-4, -1.5)
