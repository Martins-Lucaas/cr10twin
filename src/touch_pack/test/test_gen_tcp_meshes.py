"""XT-14: o TOTAL/TCP impresso só vale para a pilha completa (ft6).

Roda o script de produção numa árvore temporária: nenhum STL do pacote é
regravado. Precisa do OCP (`cadquery-ocp`), que só o gerador usa.
"""
import importlib.util
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(importlib.util.find_spec('OCP') is None,
                                reason='sem OCP (cadquery-ocp)')
_PKG = Path(__file__).resolve().parents[1]
_STEP = 'MONTAGEM_FA7155_stack.step'


@pytest.fixture()
def gera(tmp_path):
    scripts = tmp_path / 'src' / 'touch_pack' / 'scripts'
    scripts.mkdir(parents=True)
    script = shutil.copy(_PKG / 'scripts' / 'gen_tcp_meshes_from_step.py',
                         scripts)
    for d in ('montagens', 'ponteiras'):
        (tmp_path / 'cad' / d).mkdir(parents=True)

    def roda(stack):
        r = subprocess.run([sys.executable, script, '--stack', stack],
                           capture_output=True, text=True, timeout=120)
        assert r.returncode == 0, r.stderr
        return r.stdout, list((scripts.parent / 'meshes').glob('*.stl'))
    return tmp_path / 'cad', roda


def test_peca_avulsa_nao_imprime_o_tcp(gera):
    """A base 0 da ponteira avulsa é a da peça: copiar o 'TCP' dela para o
    URDF poria a ponta dentro do flange."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
    cad, roda = gera
    w = STEPControl_Writer()
    w.Transfer(BRepPrimAPI_MakeBox(107.0, 20.0, 12.0).Shape(),
               STEPControl_AsIs)
    w.Write(str(cad / 'ponteiras' / 'ponteira_texturas_v2.step'))
    saida, stl = roda('texturas_v2')
    assert 'TOTAL' not in saida and 'tcp_link' not in saida
    assert '<link name="tool_tip_link">' in saida     # o resto continua
    # 107 x 20 x 12 mm a 950 kg/m3 = 24,4 g
    assert '<mass value="0.0244"/>' in saida
    assert len(stl) == 1 and stl[0].stat().st_size > 84


def test_pilha_completa_imprime_o_tcp(gera):
    cad, roda = gera
    origem = _PKG.parents[1] / 'cad' / 'montagens' / _STEP
    if not origem.exists():
        pytest.skip(f'{_STEP} fora desta árvore')
    shutil.copy(origem, cad / 'montagens')
    saida, stl = roda('ft6')
    assert '# TOTAL do TCP' in saida
    assert 'tcp_link em Z = 67.7' in saida
    assert len(stl) == 4
