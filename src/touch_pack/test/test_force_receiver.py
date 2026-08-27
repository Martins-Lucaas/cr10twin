"""Lógica do `force_receiver` que decide se a força vale alguma coisa.

O nó em si abre porta serial e publica em tópicos; o que se testa aqui é o
miolo que NÃO depende de nada disso e que é onde mora o risco:

  * a leitura da calibração — é ela que decide se `/load_cell/force_net`
    existe, e ela lê um JSON escrito por outro programa (o wizard da GUI);
  * o critério de estabilidade do tare, que é o que separa um zero honesto de
    um zero tirado com a ponteira já encostada.

Precisa de rclpy só para o import (o módulo declara um Node). Sem ambiente ROS
o arquivo é pulado inteiro em vez de quebrar a suíte.
"""
import json
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('touch_pack_msgs')

from touch_pack.constants import (                     # noqa: E402
    G_N_PER_KG, lc_fit_slope, lc_force_n, lc_load_calibration,
)
from touch_pack.force_receiver_node import ForceReceiverNode   # noqa: E402


def load_calibration(path):
    """Só a reta — o formato que o receiver consome. O leitor único devolve
    também os pontos, que interessam ao wizard e não a ele."""
    cal = lc_load_calibration(path)
    return None if cal is None else (cal[0], cal[1])


# ── Calibração ────────────────────────────────────────────────────────
def _grava(tmp_path, payload) -> str:
    p = tmp_path / 'load_cell_calib.json'
    p.write_text(json.dumps(payload), encoding='utf-8')
    return str(p)


def test_reads_slope_and_intercept(tmp_path):
    path = _grava(tmp_path, {'slope': 8.7e-4, 'intercept': 2.7e-5})
    assert load_calibration(path) == (8.7e-4, 2.7e-5)


def test_zero_voltage_is_accepted_as_the_intercept(tmp_path):
    """Arquivos anteriores ao campo `intercept` só trazem `zero_voltage`. Os
    dois nomes são o mesmo número, e recusar o antigo invalidaria calibrações
    boas que ninguém tem como refazer sem as massas padrão na mão."""
    path = _grava(tmp_path, {'slope': 8.7e-4, 'zero_voltage': 3.1e-5})
    assert load_calibration(path) == (8.7e-4, 3.1e-5)


def test_intercept_wins_over_the_alias(tmp_path):
    path = _grava(tmp_path, {'slope': 8.7e-4, 'intercept': 1.0e-5,
                             'zero_voltage': 9.9e-5})
    assert load_calibration(path) == (8.7e-4, 1.0e-5)


@pytest.mark.parametrize('payload', [
    {'intercept': 2.7e-5},          # sem slope
    {'slope': 0.0},                 # slope nulo: divisão por zero adiante
    {'slope': 'oito'},              # slope ilegível
])
def test_unusable_calibration_is_none_not_a_guess(tmp_path, payload):
    """None e não um default: o nó trata ausência de calibração como
    'não publicar força', que é o único comportamento seguro. Um slope
    inventado faria o explorer regular contra um número sem origem."""
    assert load_calibration(_grava(tmp_path, payload)) is None


def test_a_missing_file_is_not_an_exception(tmp_path):
    assert load_calibration(str(tmp_path / 'nao_existe.json')) is None


def test_a_corrupt_file_is_not_an_exception(tmp_path):
    p = tmp_path / 'calib.json'
    p.write_text('{isto não é json', encoding='utf-8')
    assert load_calibration(str(p)) is None


def _calib_do_repo():
    repo = pathlib.Path(__file__).resolve().parents[3]
    cal = lc_load_calibration(str(repo / 'sensors' / 'load_cell_calib.json'))
    assert cal is not None, 'sensors/load_cell_calib.json sumiu ou quebrou'
    return cal


def test_the_real_calibration_of_the_repo_loads():
    """`sensors/load_cell_calib.json` é a calibração de 7 pontos que veio com
    a célula. Se o formato do wizard divergir dela, é aqui que aparece."""
    slope, intercept, pontos = _calib_do_repo()
    assert slope == pytest.approx(8.7007e-4, rel=1e-3)
    assert intercept == pytest.approx(2.774e-5, rel=1e-3)
    assert len(pontos) == 7


def test_the_derived_force_agrees_with_the_one_stored_in_the_file():
    """A força de cada ponto é DERIVADA da massa pelo leitor, não lida do
    arquivo — e este teste confere que as duas dizem a mesma coisa NO ARQUIVO
    QUE VEIO COM A CÉLULA.

    Comparar a força derivada com `massa × g` seria tautologia: é exatamente o
    que `lc_load_calibration` calcula. O que tem conteúdo é comparar com o
    `force_n` GRAVADO, que veio de outro programa e de outra época — se um dia
    divergirem, é sinal de arquivo escrito com outro valor de g, e o leitor
    protege o reajuste justamente por ignorá-lo.

    A tolerância é de 4 casas porque é assim que o campo está gravado
    (0,4903 contra os 0,4903325 exatos).
    """
    repo = pathlib.Path(__file__).resolve().parents[3]
    bruto = json.loads((repo / 'sensors' / 'load_cell_calib.json')
                       .read_text(encoding='utf-8'))
    _slope, _v0, pontos = _calib_do_repo()
    assert len(bruto['points']) == len(pontos)
    for item, (massa, forca, _v) in zip(bruto['points'], pontos):
        assert item['mass_kg'] == pytest.approx(massa)
        assert forca == pytest.approx(massa * G_N_PER_KG, rel=1e-12)
        assert item['force_n'] == pytest.approx(forca, abs=5e-5)


def test_refitting_the_stored_points_reproduces_the_stored_line():
    """O TESTE QUE IMPORTA nesta calibração.

    O wizard reajusta os pontos que carrega do arquivo. Se o estimador dele
    não for o que produziu a reta em vigor, abrir a aba e apertar "Fit"
    mudaria a escala de força do sistema inteiro sem uma linha de aviso —
    e a reta continuaria parecendo perfeitamente razoável.

    A reta do repo foi ajustada com o V₀ FIXO no zero medido. Um ajuste livre
    de dois parâmetros sobre EXATAMENTE os mesmos pontos devolve 8,7709e-4:
    0,8 % de diferença, que é muito mais que qualquer resíduo do teste e
    invisível em qualquer gráfico.
    """
    slope, v0, pontos = _calib_do_repo()
    refit = lc_fit_slope([(f, v) for _m, f, v in pontos], v0)
    assert refit is not None
    assert refit[0] == pytest.approx(slope, rel=1e-9)


def test_the_free_two_parameter_fit_would_have_moved_the_scale():
    """Guarda do teste acima: se um dia os dois métodos derem o mesmo número,
    o teste anterior deixa de provar qualquer coisa e este avisa."""
    _slope, v0, pontos = _calib_do_repo()
    n = float(len(pontos))
    sf = sum(f for _m, f, _v in pontos)
    sv = sum(v for _m, _f, v in pontos)
    sff = sum(f * f for _m, f, _v in pontos)
    sfv = sum(f * v for _m, f, v in pontos)
    livre = (n * sfv - sf * sv) / (n * sff - sf * sf)
    fixo = lc_fit_slope([(f, v) for _m, f, v in pontos], v0)[0]
    assert abs(livre - fixo) / fixo > 0.005


def test_the_fixed_zero_fit_needs_a_force_span():
    """Só o zero, ou tudo na mesma força: não há reta, e devolver uma seria
    devolver ruído com cara de calibração."""
    assert lc_fit_slope([], 0.0) is None
    assert lc_fit_slope([(0.0, 1e-5)], 0.0) is None


def test_the_fit_is_exact_on_a_perfect_line():
    """Sanidade do estimador: pontos exatamente sobre `v = s·F + v0` têm de
    devolver `s` e resíduo nulo, com o V₀ entrando como veio."""
    v0, s = 3.1e-5, 8.6e-4
    pts = [(f, s * f + v0) for f in (1.0, 5.0, 9.0)]
    slope, pior = lc_fit_slope(pts, v0)
    assert slope == pytest.approx(s, rel=1e-12)
    assert pior == pytest.approx(0.0, abs=1e-9)


def test_force_from_the_repo_line_matches_the_masses():
    """Ponta a ponta: tensão do arquivo → newton, contra a massa padrão que a
    produziu. O pior desvio é o da própria bancada (~0,25 N em 15 N)."""
    slope, v0, pontos = _calib_do_repo()
    pior = max(abs(lc_force_n(v, slope, v0) - f) for _m, f, v in pontos)
    assert pior < 0.30


# ── Estabilidade do tare ──────────────────────────────────────────────
_drift = ForceReceiverNode._window_drift


def test_a_flat_window_has_no_drift():
    assert _drift([1.0] * 40) == pytest.approx(0.0)


def test_symmetric_noise_does_not_count_as_drift():
    """O critério é MEDIANA da 2ª metade contra a 1ª, e não pico-a-pico, por
    isto: ruído simétrico tem ptp grande e deriva nula. Com o ptp, uma janela
    mais longa recusaria tares perfeitamente bons só por ser mais longa."""
    win = [1.0 + (0.5 if i % 2 else -0.5) for i in range(40)]
    assert _drift(win) < 0.01


def test_a_ramp_shows_up_as_drift():
    """O caso que o tare tem de recusar: a célula ainda assentando (deriva
    térmica, ou a ponteira encostando devagar)."""
    win = [i * 0.05 for i in range(40)]
    assert _drift(win) > ForceReceiverNode._TARE_STABLE_N


def test_an_odd_window_still_works():
    """A janela vem de um deque fatiado, então o tamanho não é garantido par
    — e a conta usa índices calculados à mão."""
    assert _drift([0.0] * 39) == pytest.approx(0.0)


def test_the_autotare_guard_is_tighter_than_the_abort():
    """O auto-tare de partida só aceita repouso perto do V₀ da calibração. O
    teto tem de ficar MUITO abaixo do aborto de 15 N: se ele chegasse perto,
    ligar o nó com a ponteira apoiada zeraria uma força de apoio real e o
    explorer desceria contra um contato que já existia."""
    from touch_pack.constants import FORCE_ABORT_LIMIT_N
    assert ForceReceiverNode._AUTOTARE_MAX_N < FORCE_ABORT_LIMIT_N / 5.0


def test_the_autozero_band_alone_does_not_protect_a_hold():
    """A banda do auto-zero (0,30 N) é MAIOR que o limiar de contato (0,10 N):
    sozinha ela deixaria o auto-zero comer devagar um contato leve durante um
    HOLD. É por isso que a guarda de FASE não é redundante — as duas condições
    (`_AUTOZERO_PHASES` e a banda) são exigidas juntas no `_publish_net`, e
    este teste existe para que ninguém remova a de fase achando que a banda
    já resolve.

    Herdado do ft_receiver, que usa a mesma banda pelo mesmo motivo.
    """
    from touch_pack.constants import CONTACT_ON_N
    assert ForceReceiverNode._AUTOZERO_BAND_N > CONTACT_ON_N
    assert 'IDLE' in ForceReceiverNode._AUTOZERO_PHASES
    assert 'DESCENDING' not in ForceReceiverNode._AUTOZERO_PHASES
    assert 'HOLD' not in ForceReceiverNode._AUTOZERO_PHASES


# ── O heartbeat separa dois silêncios ─────────────────────────────────
def test_o_contador_de_heartbeat_tem_consumidor():
    """`LcLineParser.heartbeats` foi escrito e nunca lido durante toda a
    primeira versão deste driver — um contador que não chega a ninguém.

    Ele existe para separar dois defeitos que de fora são o mesmo silêncio:
    heartbeat chegando SEM amostra significa MCU vivo na USB e HX711 mudo
    (fiação da ponte), enquanto silêncio total significa placa fora do cabo.
    Este teste guarda o consumo, não a redação da mensagem.
    """
    import inspect
    from touch_pack import force_receiver_node as FR
    src = inspect.getsource(FR.ForceReceiverNode._report_link_health)
    assert 'heartbeats' in src
    # A condição É a informação: heartbeat COM amostra não é anomalia nenhuma.
    assert 'if d_hb and not rx:' in src


def test_a_ausencia_de_calibracao_diz_a_consequencia():
    """`AUSENTE (<path>)` sozinho não diz que o ensaio será recusado — e o
    sintoma (nenhuma força) parece placa morta."""
    import inspect
    from touch_pack import force_receiver_node as FR
    src = inspect.getsource(FR.ForceReceiverNode._calib_desc)
    assert 'LC_CALIB_SOURCE' in src, (
        'a origem do caminho não é dita: fora da árvore do repo o caminho é '
        'o do ~/.config e o arquivo provavelmente nunca existiu')
    assert 'force_net' in src


# ── A MESMA calibração em qualquer computador ─────────────────────────
# O mecanismo é o git (`sensors/load_cell_calib.json` é versionado) mais a
# cópia instalada com o pacote, para o caso do deploy que não leva a árvore
# do repo. O que estes testes travam é o resolvedor e a verificabilidade.

def test_a_calibracao_e_instalada_com_o_pacote():
    """Sem isto, um deploy que leve só o `install/` chega sem reta — e sem
    reta o force_receiver não publica força nenhuma."""
    setup_py = (pathlib.Path(__file__).resolve().parents[1] / 'setup.py'
                ).read_text(encoding='utf-8')
    assert "'sensors'" in setup_py and 'load_cell_calib.json' in setup_py


def test_a_origem_resolvida_e_uma_das_tres_conhecidas():
    from touch_pack.constants import LC_CALIB_SOURCE
    assert LC_CALIB_SOURCE in ('repo', 'share', 'config')


def test_config_nao_conta_como_origem_que_se_propaga():
    """`~/.config` é a única das três que fica na máquina. Marcá-la como
    compartilhada faria o wizard calar justamente no caso em que precisa
    avisar."""
    from touch_pack.constants import LC_CALIB_SHARED_SOURCES
    assert 'config' not in LC_CALIB_SHARED_SOURCES
    assert set(LC_CALIB_SHARED_SOURCES) == {'repo', 'share'}


def test_a_impressao_digital_e_dos_numeros_nao_do_arquivo(tmp_path):
    """Dois arquivos que MEDEM igual têm de dar a mesma impressão, senão ela
    não serve para comparar duas máquinas: metadado, indentação e ordem de
    chaves mudam entre versões do wizard."""
    from touch_pack.constants import lc_calib_fingerprint
    base = {'slope': 8.7e-4, 'intercept': 2.7e-5,
            'points': [{'mass_kg': 0.5, 'force_n': 4.9, 'v_sensor': 4.5e-3},
                       {'mass_kg': 1.0, 'force_n': 9.8, 'v_sensor': 8.9e-3}]}
    a = tmp_path / 'a.json'
    a.write_text(json.dumps(base), encoding='utf-8')
    b = tmp_path / 'b.json'
    b.write_text(json.dumps({**base, 'n_points': 99, 'lixo': 'x'},
                            indent=4, sort_keys=True), encoding='utf-8')
    assert lc_calib_fingerprint(str(a)) == lc_calib_fingerprint(str(b))


def test_a_impressao_digital_muda_com_a_reta(tmp_path):
    """O contrário do teste acima, e é ele que dá valor ao outro: uma reta
    diferente NÃO pode passar por igual."""
    from touch_pack.constants import lc_calib_fingerprint
    base = {'slope': 8.7e-4, 'intercept': 2.7e-5,
            'points': [{'mass_kg': 0.5, 'v_sensor': 4.5e-3}]}
    a = tmp_path / 'a.json'
    a.write_text(json.dumps(base), encoding='utf-8')
    for campo, valor in (('slope', 8.7e-4 * 1.001), ('intercept', 3.0e-5)):
        b = tmp_path / f'{campo}.json'
        b.write_text(json.dumps({**base, campo: valor}), encoding='utf-8')
        assert lc_calib_fingerprint(str(a)) != lc_calib_fingerprint(str(b))
    # E um ponto a mais também é outra calibração.
    c = tmp_path / 'c.json'
    c.write_text(json.dumps({**base, 'points': base['points'] + [
        {'mass_kg': 1.0, 'v_sensor': 8.9e-3}]}), encoding='utf-8')
    assert lc_calib_fingerprint(str(a)) != lc_calib_fingerprint(str(c))


def test_sem_calibracao_a_impressao_e_vazia(tmp_path):
    from touch_pack.constants import lc_calib_fingerprint
    assert lc_calib_fingerprint(str(tmp_path / 'nao_existe.json')) == ''


def test_a_calibracao_do_repo_tem_impressao_estavel():
    """Guarda do valor: se `sensors/load_cell_calib.json` mudar de números,
    este teste cai — que é o aviso de que as outras máquinas ficaram para
    trás e o arquivo precisa ser commitado."""
    from touch_pack.constants import lc_calib_fingerprint
    repo = pathlib.Path(__file__).resolve().parents[3]
    fp = lc_calib_fingerprint(str(repo / 'sensors' / 'load_cell_calib.json'))
    assert fp == 'bd6813e9', (
        f'a calibração da bancada mudou (impressão {fp}). Se foi de '
        'propósito, atualize este valor E commite o JSON — senão as outras '
        'máquinas continuam medindo com a reta antiga.')
