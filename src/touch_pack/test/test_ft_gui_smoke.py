"""Smoke test dos painéis novos da aba "6 Axes" (gráficos e vista 3D).

Por que existe: pyflakes não pega erro de widget. Um `tk.Label` com opção que
não existe, um `coords()` com número errado de argumentos ou um item do canvas
referenciado antes de criado só aparecem quando alguém abre a aba — e na
bancada, com o sensor ligado, é o pior lugar para descobrir.

Aqui os dois mixins são montados sobre um hospedeiro de mentira (só o `_card`,
o `_lock` e a taxa que eles consomem do resto da GUI), alimentados com
amostras e mandados repintar. Não valida aparência; valida que o caminho de
desenho inteiro roda sem estourar.

Pula sozinho onde não há display (CI headless).
"""
import threading
import tkinter as tk

import pytest

from touch_pack.constants import FT_AXES
from touch_pack.gui_ft_arrow import FtArrowMixin
from touch_pack.gui_ft_charts import FtChartsMixin
from touch_pack.ui_helpers import PANEL


@pytest.fixture(scope='module')
def _tk_root():
    """UM Tk por módulo.

    Criar e destruir um root por teste falha de forma intermitente no Python
    da Microsoft Store ("Can't find a usable tk.tcl") — e o teste que caía era
    sorteado, o que faria a suíte parecer instável sem motivo. Cada teste
    ganha um Frame próprio, destruído no teardown.
    """
    try:
        r = tk.Tk()
    except tk.TclError as exc:                       # pragma: no cover
        pytest.skip(f'sem display: {exc}')
    r.withdraw()
    yield r
    r.destroy()


@pytest.fixture
def raiz(_tk_root):
    f = tk.Frame(_tk_root)
    f.pack()
    yield f
    f.destroy()


class _Host(FtChartsMixin, FtArrowMixin):
    """O mínimo que os mixins esperam do PalpationGUI."""

    def __init__(self, raiz):
        self.root = raiz
        self._lock = threading.Lock()
        self._ft_rate_hz = 1000.0

    def _card(self, root, titulo, expand=False):
        f = tk.Frame(root, bg=PANEL)
        f.pack(fill='both', expand=expand)
        return f


@pytest.fixture
def host(raiz):
    h = _Host(raiz)
    h._ft_charts_init()
    h._build_ft_chart_card(raiz)
    h._build_ft_columns_card(raiz)
    h._build_ft_arrow_card(raiz)
    raiz.update_idletasks()
    return h


def _alimenta(h, n=2500, f=(1.0, -2.0, 3.0, 0.1, -0.2, 0.3)):
    for _ in range(n):
        h._ft_charts_feed(f)


def _mostrado(f=(1.0, -2.0, 3.0, 0.1, -0.2, 0.3)):
    return dict(zip(FT_AXES, f))


# ── Construção ────────────────────────────────────────────────────────

def test_paineis_constroem_sem_estourar(host):
    assert host._ft_chart_lines and host._ft_col_widgets
    assert len(host._ft_arrow_items) > 0


def test_buffer_respeita_a_janela_de_fabrica(host):
    _alimenta(host, n=5000)
    assert len(host._ft_hist['fx']) == host._ft_chart_win == 2000


# ── Repintura ─────────────────────────────────────────────────────────

def test_repinta_com_dados_vivos(host, raiz):
    _alimenta(host)
    host._refresh_ft_charts(_mostrado(), live=True)
    host._refresh_ft_arrow(_mostrado(), live=True)
    raiz.update_idletasks()


def test_repinta_com_buffer_vazio(host, raiz):
    """Primeiro quadro depois de abrir a aba: ainda não chegou amostra."""
    host._refresh_ft_charts({}, live=False)
    host._refresh_ft_arrow({}, live=False)
    raiz.update_idletasks()


def test_repinta_com_forca_exatamente_zero(host, raiz):
    """|F| = 0 não tem direção: a seta some em vez de apontar para um lugar
    arbitrário. É o estado da célula tarada e sem carga."""
    zero = dict.fromkeys(FT_AXES, 0.0)
    _alimenta(host, n=100, f=(0.0,) * 6)
    host._refresh_ft_charts(zero, live=True)
    host._refresh_ft_arrow(zero, live=True)
    raiz.update_idletasks()
    assert host._ft_arrow_dir.cget('text') == '(no force)'


def test_repinta_em_compressao_pura_z(host, raiz):
    """O caso degenerado da rotação, e o mais comum da bancada."""
    v = (0.0, 0.0, -120.0, 0.0, 0.0, 0.0)
    _alimenta(host, n=100, f=v)
    host._refresh_ft_arrow(_mostrado(v), live=True)
    raiz.update_idletasks()
    assert '120' in host._ft_arrow_mag.cget('text')


def test_sobrecarga_nao_desenha_fora_do_canvas(host, raiz):
    """Valor muito além da escala satura no gráfico; deixar o Tk desenhar em
    coordenadas absurdas engasga o widget inteiro."""
    v = (9999.0, -9999.0, 9999.0, 999.0, -999.0, 999.0)
    _alimenta(host, n=300, f=v)
    host._refresh_ft_charts(_mostrado(v), live=True)
    raiz.update_idletasks()
    cv = host._ft_chart_cv['force']
    ys = cv.coords(host._ft_chart_lines['fx'])[1::2]
    assert ys and all(0.0 <= y <= 200.0 for y in ys)


# ── Controles ─────────────────────────────────────────────────────────

def test_trocar_a_janela_preserva_as_amostras_que_cabem(host):
    _alimenta(host, n=2000)
    host._ft_chart_win_var.set('500')
    host._ft_apply_chart_scale()
    assert len(host._ft_hist['fx']) == 500


def test_escala_invalida_avisa_em_vez_de_quebrar(host):
    host._ft_chart_fmax_var.set('nonsense')
    host._ft_apply_chart_scale()
    assert 'number' in host._ft_chart_lbl.cget('text').lower()


def test_factory_defaults_restaura_o_range_csv(host):
    host._ft_chart_win_var.set('50')
    host._ft_apply_chart_scale()
    host._ft_reset_chart_scale()
    assert host._ft_chart_win == 2000
    assert host._ft_chart_fmax == 200.0
    assert host._ft_chart_mmax == 50.0


def test_freeze_para_de_acumular(host):
    _alimenta(host, n=10)
    antes = len(host._ft_hist['fx'])
    host._ft_chart_pause_var.set(True)
    host._ft_toggle_chart_pause()
    _alimenta(host, n=100)
    assert len(host._ft_hist['fx']) == antes
