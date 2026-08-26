"""Savitzky-Golay e estatísticas de janela do painel de 6 eixos.

Reproduz o pós-processamento do cliente de fábrica da FIBOS. O que importa
aqui é a PROPRIEDADE que define o SG — preservar polinômio de grau <= ordem —
e não bater com uma tabela de coeficientes copiada de algum lugar.
"""
import numpy as np
import pytest

from touch_pack import ft_stats as st


# ── Validação (mesmo texto do cliente de fábrica) ─────────────────────

def test_janela_par_e_recusada_com_a_frase_do_fabricante():
    with pytest.raises(ValueError, match='Window size must be odd.'):
        st.validate_savgol(10, 3)


def test_ordem_maior_ou_igual_a_janela_e_recusada():
    with pytest.raises(ValueError, match='Order must be less than window'):
        st.validate_savgol(5, 5)


# ── A propriedade que define o SG ─────────────────────────────────────

def test_coeficientes_somam_um():
    """Ganho DC unitário: um sinal constante tem de sair intacto."""
    for w, o in ((5, 2), (11, 3), (21, 4)):
        assert st.savgol_coeffs(w, o).sum() == pytest.approx(1.0)


def test_preserva_polinomio_ate_a_ordem():
    """Um cúbico puro passa por um SG de ordem 3 sem alteração alguma."""
    x = np.arange(60, dtype=float)
    y = 3.0 - 0.5 * x + 0.02 * x ** 2 - 0.0003 * x ** 3
    out = st.savgol_filter(y, window=11, order=3)
    assert np.allclose(out, y, atol=1e-8)


def test_bordas_nao_inventam_pico():
    """Espelhar o sinal criaria um pico no degrau de contato — por isso as
    bordas usam o polinômio na posição real, e ficam dentro da faixa."""
    y = np.concatenate([np.zeros(20), np.ones(20)])
    out = st.savgol_filter(y, window=9, order=2)
    assert out.min() > -0.35 and out.max() < 1.35


def test_serie_menor_que_a_janela_volta_intacta():
    y = np.array([1.0, 2.0, 3.0])
    assert np.array_equal(st.savgol_filter(y, window=11, order=3), y)


def test_suaviza_ruido_branco():
    rng = np.random.default_rng(0)
    y = rng.normal(0.0, 1.0, 4000)
    out = st.savgol_filter(y, window=21, order=2)
    assert out.std() < 0.6 * y.std()


# ── Versão de stream (sem atraso) ─────────────────────────────────────

def test_stream_devolve_a_amostra_ate_encher_a_janela():
    f = st.StreamingSavGol(window=7, order=2)
    for i in range(6):
        assert f.update(float(i)) == float(i)


def test_stream_nao_atrasa_uma_rampa():
    """Avaliado na PONTA: sobre uma reta o SG devolve o valor corrente, não o
    do centro da janela — é essa a diferença para o SG canônico."""
    f = st.StreamingSavGol(window=11, order=2)
    out = [f.update(2.0 * i) for i in range(40)]
    assert out[-1] == pytest.approx(2.0 * 39, abs=1e-6)


def test_stream_ignora_nan_sem_envenenar():
    """Mesmo motivo do FtFrameParser: um NaN no buffer contamina toda saída
    seguinte."""
    f = st.StreamingSavGol(window=5, order=2)
    for i in range(10):
        f.update(1.0)
    assert np.isnan(f.update(float('nan')))
    assert f.update(1.0) == pytest.approx(1.0)


def test_reconfigurar_limpa_o_buffer():
    f = st.StreamingSavGol(window=5, order=2)
    for _ in range(5):
        f.update(9.0)
    f.configure(7, 3)
    assert f.update(0.0) == 0.0        # janela vazia -> passa direto


# ── Mean_Num / MAX_Num ────────────────────────────────────────────────

def test_estatisticas_da_janela():
    s = st.RollingStats(window=4)
    for v in (1.0, -3.0, 2.0, 0.0):
        s.update(v)
    snap = s.snapshot()
    assert snap['n'] == 4
    assert snap['mean'] == pytest.approx(0.0)
    assert snap['min'] == -3.0 and snap['max'] == 2.0
    assert snap['pp'] == 5.0


def test_max_e_do_valor_absoluto():
    """Num eixo bipolar o pico que interessa é o de magnitude; um max ingênuo
    sobre um sinal só negativo devolveria o ponto mais perto de zero."""
    s = st.RollingStats(window=3)
    for v in (-5.0, -2.0, -1.0):
        s.update(v)
    assert s.snapshot()['max_abs'] == 5.0


def test_janela_desliza():
    s = st.RollingStats(window=3)
    for v in (100.0, 1.0, 1.0, 1.0):
        s.update(v)
    assert s.snapshot()['max_abs'] == 1.0


def test_snapshot_vazio_nao_explode():
    assert st.RollingStats().snapshot()['n'] == 0


def test_nan_nao_entra_na_janela():
    s = st.RollingStats(window=5)
    s.update(1.0)
    s.update(float('nan'))
    assert s.snapshot()['n'] == 1


def test_resize_preserva_o_que_couber():
    s = st.RollingStats(window=5)
    for v in range(5):
        s.update(float(v))
    s.resize(2)
    assert s.snapshot()['n'] == 2
