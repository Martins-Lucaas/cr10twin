"""Decimação dos gráficos da aba "6 Axes".

O que estes testes protegem é a única promessa não-óbvia do desenho: que
reduzir 2000 amostras a 360 colunas de pixel NÃO esconde pico. Num sensor de
força o transiente curto é o dado, não o ruído — subamostrar por passo
(pegar 1 a cada k) perderia exatamente isso, e o bug seria invisível porque o
gráfico continuaria bonito.

Só a função pura é testada: o resto do mixin é canvas do Tk e precisa de
display.
"""
import pytest

from touch_pack.gui_ft_charts import decimate_minmax


def test_vazio_nao_explode():
    assert decimate_minmax([]) == []


def test_amostra_unica_vira_um_ponto_em_x_zero():
    assert decimate_minmax([3.0]) == [(0.0, 3.0)]


def test_serie_menor_que_as_colunas_sai_intacta():
    vals = [1.0, 2.0, 3.0, 4.0]
    out = decimate_minmax(vals, cols=360)
    assert [v for _, v in out] == vals
    assert [x for x, _ in out] == [0.0, 1 / 3, 2 / 3, 1.0]


def test_x_cobre_zero_a_um():
    out = decimate_minmax(list(range(2000)), cols=360)
    assert out[0][0] == 0.0
    assert out[-1][0] == pytest.approx(1.0)


def test_pico_de_uma_amostra_sobrevive_a_decimacao():
    """1 pico em 2000 amostras, reduzido a 360 colunas: tem de aparecer."""
    vals = [0.0] * 2000
    vals[1234] = 97.5
    out = decimate_minmax(vals, cols=360)
    assert max(v for _, v in out) == 97.5


def test_vale_de_uma_amostra_tambem_sobrevive():
    vals = [0.0] * 2000
    vals[77] = -42.0
    out = decimate_minmax(vals, cols=360)
    assert min(v for _, v in out) == -42.0


def test_subamostragem_por_passo_perderia_o_pico():
    """Justifica o min/max: a alternativa ingênua falha neste mesmo dado."""
    vals = [0.0] * 2000
    vals[1234] = 97.5
    passo = vals[::len(vals) // 360]          # o que NÃO fazemos
    assert max(passo) == 0.0                  # pico sumiu
    assert max(v for _, v in decimate_minmax(vals, cols=360)) == 97.5


def test_custo_depende_das_colunas_e_nao_das_amostras():
    curto = decimate_minmax([0.0] * 2000, cols=360)
    longo = decimate_minmax([0.0] * 200000, cols=360)
    # Sinal constante: uma coluna gera 1 ponto (lo == hi), não 2.
    assert len(curto) == len(longo) == 360


def test_coluna_com_variacao_gera_dois_pontos():
    vals = [(-1.0 if i % 2 else 1.0) for i in range(2000)]
    out = decimate_minmax(vals, cols=360)
    assert len(out) == 720
    assert min(v for _, v in out) == -1.0
    assert max(v for _, v in out) == 1.0
