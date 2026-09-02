"""Canal LENTO da célula (`/load_cell/force_net_slow`).

O canal existe porque um filtro só atendia dois trabalhos incompatíveis: o
rápido precisa de latência (segurança, contato, textura do SLIDING) e o do
regulador quase-estático precisa de σ, com o braço já congelado. O que se
testa aqui é o que a bancada offline mediu sobre o sinal REAL e o que a
separação promete:

  * a janela é em SEGUNDOS e não em amostras — o mesmo número tem de valer
    com o HX711 a 10 e a 80 Hz;
  * a média só está limpa depois da janela INTEIRA (é o que obriga o
    consumidor a congelar por `win_s` antes de ler);
  * σ cai com 1/√N, que é a propriedade que justifica alargar a janela e que
    a nota de sintonia antiga negava ao chamar o ruído de 1/f;
  * os dois canais saem do MESMO v_raw em paralelo, nunca em cascata.
"""
import math
import pathlib
import random
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
pytest.importorskip('rclpy')

from touch_pack.lc_filter import (                     # noqa: E402
    LC_SLOW_WIN_S, _BoxcarFilter, _LoadCellFilter,
)

FS = 24.39          # Hz — a taxa real medida nos runs de 01/09/2026
DT = 1.0 / FS


def _noise(n, sigma=0.1148, seed=0):
    """σ = 114,8 mN: o ruído CRU medido em 32,6 s de ar livre do run
    MANUAL/20260901_162614."""
    r = random.Random(seed)
    return [r.gauss(0.0, sigma) for _ in range(n)]


def _sigma(xs):
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / len(xs))


def test_a_janela_e_em_segundos_e_nao_em_amostras():
    """A mesma janela em taxas diferentes tem de guardar o mesmo TEMPO."""
    for fs in (10.0, 24.39, 80.0):
        f = _BoxcarFilter(2.0)
        for _ in range(int(6 * fs)):
            f.update(1.0, 1.0 / fs)
        assert f.span_s == pytest.approx(2.0, abs=2.0 / fs)


def test_media_de_constante_e_a_constante():
    f = _BoxcarFilter(2.0)
    for _ in range(200):
        out = f.update(5.0, DT)
    assert out == pytest.approx(5.0, abs=1e-9)


def test_a_media_so_esta_limpa_depois_da_janela_inteira():
    """É a razão de `_qs_measure_settled` congelar por `win_s` antes de ler:
    metade da janela ainda devolve metade da força ANTERIOR."""
    f = _BoxcarFilter(2.0)
    for _ in range(300):
        f.update(0.0, DT)
    meio = None
    for k in range(1, int(2.0 * FS) + 2):
        out = f.update(1.0, DT)
        if meio is None and k >= int(1.0 * FS):
            meio = out
    assert meio == pytest.approx(0.5, abs=0.05)   # metade da janela, metade
    assert out == pytest.approx(1.0, abs=0.02)    # janela cheia, tudo


def test_sigma_cai_com_raiz_de_n():
    """A propriedade que justifica o canal. Se o ruído fosse 1/f isto
    saturaria — e era o que a nota de sintonia antiga supunha."""
    x = _noise(int(60 * FS))
    ref = None
    for win_s in (0.5, 2.0, 4.0):
        f = _BoxcarFilter(win_s)
        y = [f.update(v, DT) for v in x][int(8 * FS):]
        s = _sigma(y)
        if ref is None:
            ref, ref_win = s, win_s
        else:
            assert s == pytest.approx(ref * math.sqrt(ref_win / win_s), rel=0.25)


def test_o_canal_lento_e_mais_quieto_que_o_rapido():
    """19 mN contra 33 mN foi o medido no sinal real; aqui basta a ordem."""
    x = _noise(int(60 * FS))
    fast = _LoadCellFilter()
    fast.set_sensitivity(1.0)
    slow = _BoxcarFilter(LC_SLOW_WIN_S)
    yf = [fast.update(v, DT) for v in x][int(8 * FS):]
    ys = [slow.update(v, DT) for v in x][int(8 * FS):]
    assert _sigma(ys) < _sigma(yf)


def test_uma_janela_maior_que_o_dt_nunca_esvazia():
    """dt gigante (link mudo e religando) não pode zerar a janela e devolver
    divisão por zero."""
    f = _BoxcarFilter(2.0)
    f.update(1.0, DT)
    assert f.update(3.0, 30.0) == pytest.approx(3.0)


def test_os_dois_canais_saem_do_mesmo_v_raw_em_paralelo():
    """Encadear somaria a latência do One-Euro à da janela sem ganho de σ.
    O nó tem de alimentar os dois com a MESMA amostra crua."""
    src = pathlib.Path(__file__).resolve().parents[1] / 'touch_pack'
    txt = (src / 'force_receiver_node.py').read_text()
    assert 'v_filt = self._filter.update(float(v_raw), dt)' in txt
    assert 'v_slow = self._slow.update(float(v_raw), dt)' in txt
    assert 'self._slow.update(v_filt' not in txt


def test_o_canal_lento_usa_o_mesmo_tare_do_rapido():
    """Zeros diferentes fariam o regulador trocar de referência ao passar de
    um canal para o outro."""
    src = pathlib.Path(__file__).resolve().parents[1] / 'touch_pack'
    txt = (src / 'force_receiver_node.py').read_text()
    i = txt.index('_force_net_slow_pub.publish')
    trecho = txt[i - 400:i]
    assert 'self._force_of(v_slow) - tare' in trecho


def test_o_explorer_cai_no_canal_rapido_sem_o_lento():
    """O ft_receiver não publica o par; sem fallback o HOLD travaria."""
    src = pathlib.Path(__file__).resolve().parents[1] / 'touch_pack'
    txt = (src / 'tactile_explorer.py').read_text()
    assert 'def _fz_slow' in txt
    i = txt.index('def _qs_measure_fz')
    corpo = txt[i:i + 2000]
    assert 'if settle:' in corpo and '_qs_measure_settled' in corpo
    assert 'if fz is not None:' in corpo
