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

# A trava entre packs dos testes nunca é a da bancada.
import os as _os  # noqa: E402
import tempfile as _tempfile  # noqa: E402
_os.environ['CR10_PACK_LOCK_DIR'] = _tempfile.mkdtemp(prefix='pack_lock_')
