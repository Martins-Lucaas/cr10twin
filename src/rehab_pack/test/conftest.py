"""Os testes nunca tocam a trava de verdade (~/.cache/cr10twin)."""
import os
import tempfile

import pytest

from rehab_pack import constants

os.environ['CR10_PACK_LOCK_DIR'] = tempfile.mkdtemp(prefix='cr10_lock_test_')
constants.ROBOT_FILE = os.path.join(os.environ['CR10_PACK_LOCK_DIR'], 'cfg', 'robot.json')


@pytest.hookimpl(hookwrapper=True)
def pytest_pycollect_makemodule(path, parent):
    """O launch_testing importa o módulo aqui, e um skip de módulo zerava a coleta
    da sessão INTEIRA. Devolve o módulo normal, que pula sozinho."""
    res = yield
    if res.excinfo and issubclass(res.excinfo[0], pytest.skip.Exception):
        res.force_result(pytest.Module.from_parent(parent, fspath=path))
