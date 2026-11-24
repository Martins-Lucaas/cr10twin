"""Os testes nunca tocam a trava de verdade (~/.cache/cr10twin)."""
import os
import tempfile

os.environ['CR10_PACK_LOCK_DIR'] = tempfile.mkdtemp(prefix='cr10_lock_test_')
