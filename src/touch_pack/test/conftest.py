"""Nenhum teste lê ou grava o plano salvo da bancada (~/.config/touch_pack)."""
import pytest


@pytest.fixture(autouse=True)
def _plano_salvo_isolado(tmp_path, monkeypatch):
    try:
        import touch_pack.explorer_attack  # noqa: F401 — precisa de std_msgs
    except Exception:
        return
    monkeypatch.setattr('touch_pack.explorer_attack._SURFACE_PLANE_FILE',
                        str(tmp_path / 'surface_plane.json'))
