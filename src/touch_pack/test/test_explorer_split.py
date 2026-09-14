"""Guarda do recorte de `tactile_explorer.py`.

O arquivo tinha 7052 linhas e misturava quatro coisas: os números do protocolo,
a matemática da onda de força, a estimativa de rigidez da amostra e a máquina
de estados do ensaio. As três primeiras saíram para `explorer_constants.py`,
`force_wave.py` e `stiffness.py`.

O recorte foi MECÂNICO — bloco move inteiro, ninguém reescreveu conta nenhuma.
Então só três coisas podiam dar errado em silêncio, e são as que ficam travadas
aqui:

  * um nome sumir no caminho, ou passar a existir em dois lugares (a segunda
    definição venceria e a perdedora viraria código morto enganoso);
  * uma fatia importar de volta do host, fechando um ciclo que só estoura no
    import, em runtime, com o launch já subindo;
  * uma constante ser redefinida no host, passando a existir com dois valores
    — o do host valendo para a FSM e o da fatia valendo para a onda, sem nada
    no log dizendo isso.

Análise por AST: não importa os módulos (que exigem rclpy), então roda em
qualquer máquina.
"""
import ast
import pathlib

import pytest

_PKG = pathlib.Path(__file__).resolve().parents[1] / 'touch_pack'

HOST = 'tactile_explorer.py'
FATIAS = ['explorer_constants.py', 'force_wave.py', 'stiffness.py']

# O que cada fatia tem de levar consigo. Lista explícita: se um nome sumir,
# o teste diz QUAL.
DA_ONDA = {
    'fmod_measure_lag_s', 'fmod_measure_gain', '_WaveILC',
    '_fmod_max_freq_hz', '_fmod_sampling_gain', '_ForceProfile',
}
DA_RIGIDEZ = {
    'crawl_v_ms', 'impact_peak_n', 'setpoint_resolvable',
    '_StiffnessEstimator', '_ContactCurve',
}


def _arvore(nome):
    return ast.parse((_PKG / nome).read_text(encoding='utf-8'), nome)


def _definidos(nome):
    """Nomes que o módulo DEFINE (não os que importa)."""
    out = set()
    for n in _arvore(nome).body:
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            out.add(n.name)
        elif isinstance(n, ast.Assign):
            # `A, B = 1, 2` define DOIS nomes num alvo `ast.Tuple`. Olhar só
            # `ast.Name` era o ponto cego que deixou `_FX_GAIN_MIN/_MAX` fora
            # da reexportação: NameError no meio da onda de força, com o braço
            # já sobre a amostra.
            for alvo in n.targets:
                if isinstance(alvo, ast.Name):
                    out.add(alvo.id)
                elif isinstance(alvo, ast.Tuple):
                    out |= {e.id for e in alvo.elts if isinstance(e, ast.Name)}
    return out


def _importados(nome):
    out = set()
    for n in ast.walk(_arvore(nome)):
        if isinstance(n, (ast.Import, ast.ImportFrom)):
            out |= {a.asname or a.name for a in n.names}
    return out


def _de_onde_importa(nome):
    return {n.module for n in ast.walk(_arvore(nome))
            if isinstance(n, ast.ImportFrom) and n.module}


# ── Nada se perdeu ────────────────────────────────────────────────────
@pytest.mark.parametrize('esperados,arquivo', [
    (DA_ONDA, 'force_wave.py'),
    (DA_RIGIDEZ, 'stiffness.py'),
])
def test_the_slice_actually_carries_its_block(esperados, arquivo):
    faltando = sorted(esperados - _definidos(arquivo))
    assert not faltando, f'não chegaram em {arquivo}: {faltando}'


@pytest.mark.parametrize('nomes', [DA_ONDA, DA_RIGIDEZ])
def test_the_host_no_longer_defines_what_it_exported(nomes):
    """O ganho do recorte é o host encolher. Se os blocos continuarem lá, o
    arquivo não ficou mais navegável — só ganhou um import. E pior: haveria
    duas definições, e a do host venceria."""
    dupes = sorted(nomes & _definidos(HOST))
    assert not dupes, f'ainda definidos em {HOST}: {dupes}'


@pytest.mark.parametrize('nomes', [DA_ONDA, DA_RIGIDEZ])
def test_the_host_re_exports_everything_it_moved_out(nomes):
    """Meia dúzia de testes e o `palpation_logger` importam esses nomes de
    `tactile_explorer`. A reexportação é o que mantém esses imports válidos —
    tirá-la quebraria quem nunca soube do recorte."""
    faltando = sorted(nomes - _importados(HOST))
    assert not faltando, f'{HOST} deixou de reexportar: {faltando}'


def test_no_constant_is_defined_in_two_places():
    """Uma constante com dois valores é o pior resultado possível deste
    recorte: a FSM usaria um e a onda o outro, os dois plausíveis, e nada no
    log diria qual valeu em qual trecho do ensaio."""
    const = _definidos('explorer_constants.py')
    host = _definidos(HOST)
    dupes = sorted(const & host)
    assert not dupes, f'definidas nos dois lados: {dupes}'


def test_the_host_still_reaches_every_constant():
    """111 constantes saíram. Esquecer uma na lista de import daria
    NameError — mas só na fase do ensaio que a usa, possivelmente com o braço
    já sobre a amostra."""
    faltando = sorted(_definidos('explorer_constants.py') - _importados(HOST))
    assert not faltando, f'{HOST} não importa de volta: {faltando}'


# ── Sem ciclos ────────────────────────────────────────────────────────
@pytest.mark.parametrize('fatia', FATIAS)
def test_a_slice_never_imports_the_host(fatia):
    """O host importa as fatias. Uma fatia importando o host fecha o ciclo, e
    o erro aparece no import do nó — com o launch já subindo, longe daqui."""
    for mod in _de_onde_importa(fatia):
        assert not mod.endswith('tactile_explorer'), (
            f'{fatia} importa o host: ciclo de import')


def test_the_constants_module_depends_on_nothing_of_the_protocol():
    """Ele é a base: as duas outras fatias importam dele. Se ele passar a
    importar de `force_wave` ou `stiffness`, o ciclo volta pelo outro lado."""
    mods = _de_onde_importa('explorer_constants.py')
    assert not {m for m in mods
                if m.endswith(('force_wave', 'stiffness', 'tactile_explorer'))}


def test_the_constants_module_carries_no_ros_message_types():
    """As mensagens são do nó, não dos números. Deixá-las aqui faria um
    módulo de constantes exigir touch_pack_msgs compilado só para ler um
    limiar — e foi exatamente por onde o primeiro recorte passou do ponto."""
    mods = _de_onde_importa('explorer_constants.py')
    assert not {m for m in mods if m.endswith('_msgs.msg') or m.endswith(
        ('std_msgs.msg', 'sensor_msgs.msg', 'trajectory_msgs.msg'))}


# ── O host encolheu de verdade ────────────────────────────────────────
def test_the_host_is_smaller_than_it_was():
    """O recorte existe para isto. 7052 linhas era o ponto de partida; o teto
    aqui é folgado de propósito, mas volta a falhar se alguém devolver um dos
    blocos para o host."""
    n = len((_PKG / HOST).read_text(encoding='utf-8').splitlines())
    assert n < 6200, f'{HOST} voltou a ter {n} linhas'


@pytest.mark.parametrize('fatia', FATIAS)
def test_each_slice_is_navigable_on_its_own(fatia):
    """Uma fatia que cresce até o tamanho do host não resolveu nada."""
    n = len((_PKG / fatia).read_text(encoding='utf-8').splitlines())
    assert n < 1200, f'{fatia} tem {n} linhas'
