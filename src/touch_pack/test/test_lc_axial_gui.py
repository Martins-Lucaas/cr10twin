"""Smoke test da aba da célula axial de 100 kg (Reading + Calibration).

Mesmo motivo do `test_ft_gui_smoke.py`: pyflakes não pega erro de widget, e o
lugar errado de descobrir que um `tk.Entry` recebeu opção inexistente é a
bancada, com as massas padrão já empilhadas na ponteira.

Mas aqui há mais que desenho. O wizard é o que produz a reta slope/intercept
que TODA força do sistema atravessa, e ele tem três guardas que existem por
erro já cometido — mínimo de pontos, massa repetida e slope fora da placa.
Guarda que ninguém testa é guarda que alguém remove.

O hospedeiro é de mentira (só `_card`, `_lock`, `_set_status` e `_tab_visible`),
e o `root` é um proxy que ENGOLE o `after`: o refresh se reagenda sozinho a
120 ms e um timer pendente disparando depois do teardown estouraria no widget
já destruído.

Pula sozinho onde não há display (CI headless).
"""
import collections
import json
import pathlib
import sys
import threading
import time
import tkinter as tk

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from touch_pack.constants import (          # noqa: E402
    CONTACT_ON_N, FORCE_ABORT_LIMIT_N, LC_CALIB_MIN_POINTS,
    LC_NOMINAL_V_PER_N, G_N_PER_KG,
)
from touch_pack import gui_lc_axial               # noqa: E402
from touch_pack.gui_lc_axial import LcAxialMixin   # noqa: E402
from touch_pack.ui_helpers import PANEL            # noqa: E402


@pytest.fixture(scope='module')
def _tk_root():
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


class _SemAfter:
    """Proxy do Frame que engole `after` — ver o cabeçalho."""

    def __init__(self, frame):
        self._frame = frame

    def after(self, *_a, **_k):
        return None

    def __getattr__(self, name):
        return getattr(self._frame, name)


class _PubFalso:
    def __init__(self):
        self.n = 0

    def publish(self, _msg):
        self.n += 1


class _Host(LcAxialMixin):
    """O mínimo que o mixin espera do PalpationGUI."""

    def __init__(self, raiz):
        self.root = _SemAfter(raiz)
        self._lock = threading.Lock()
        self._lc_voltage = 0.0
        self._lc_voltage_ts = 0.0
        self._lc_arrivals = collections.deque(maxlen=200)
        self._lc_capture = None
        self._lc_force_net = 0.0
        self._lc_force_net_ts = 0.0
        self._lc_force_raw = 0.0
        self._lc_tare_done = False
        self._lc_calibrated = False
        self._lc_tare_req_pub = _PubFalso()
        self._lc_rezero_pub = _PubFalso()
        self._lc_tab_frame = raiz
        # 'real' = o default do launch; o caso 'sim' tem teste próprio.
        self._force_source = 'real'
        self.titulos = []
        self.status = []

    # Vem do FtAxesMixin no PalpationGUI real; aqui só o efeito que importa.
    def _lc_do_tare(self):
        self._lc_tare_req_pub.publish(None)

    def _card(self, root, titulo, expand=False):
        # O título fica registrado: no `_card` real ele vira um tk.Label
        # dentro do cabeçalho, e sem guardá-lo aqui nenhum teste consegue
        # afirmar o que o card anuncia.
        self.titulos.append(titulo)
        f = tk.Frame(root, bg=PANEL)
        f.pack(fill='both', expand=expand)
        return f

    def _set_status(self, texto, _cor=None):
        self.status.append(texto)

    def _tab_visible(self, *_frames):
        return True


@pytest.fixture
def host(raiz, tmp_path, monkeypatch):
    """Aba de calibração sobre um arquivo VAZIO.

    O caminho é trocado ANTES de montar: a aba carrega a calibração em vigor
    na montagem, e apontada para o repo ela traria os 7 pontos reais para
    dentro de cada teste — que então passariam (ou falhariam) por causa de
    dados que o teste não escolheu.
    """
    alvo = str(tmp_path / 'load_cell_calib.json')
    monkeypatch.setattr(gui_lc_axial, 'LC_CALIB_FILE', alvo)
    h = _Host(raiz)
    h._build_lc_calibration_tab(raiz)
    raiz.update_idletasks()
    return h


def _zero(h, v=3e-5):
    """O V₀ medido. O ajuste o segura FIXO, então sem ele não há reta."""
    h._lc_calib_zero = v


@pytest.fixture
def leitura(raiz):
    """Hospedeiro com a aba READING montada (a outra metade da aba)."""
    h = _Host(raiz)
    h._build_lc_reading_tab(raiz)
    raiz.update_idletasks()
    return h


def _vivo(h, *, v=1.0e-3, f_net=0.0, f_raw=0.0, tared=True, cal=True):
    agora = time.time()
    with h._lock:
        h._lc_calibrated = cal
        h._lc_voltage, h._lc_voltage_ts = v, agora
        h._lc_force_net, h._lc_force_net_ts = f_net, agora
        h._lc_force_raw = f_raw
        h._lc_tare_done = tared
        h._lc_arrivals.extend([agora] * 20)


def _ponto(h, massa_kg: float, v: float):
    """Injeta um ponto direto na tabela, sem a coleta temporizada."""
    h._lc_calib_points.append((massa_kg, massa_kg * G_N_PER_KG, v))
    h._lc_calib_points.sort()


def _reta_boa(h, n=5):
    """V₀ + n pontos sobre a reta NOMINAL da placa — o caso que tem de passar."""
    _zero(h)
    for i in range(n):
        m = 0.2 * (i + 1)
        _ponto(h, m, LC_NOMINAL_V_PER_N * m * G_N_PER_KG + 3e-5)


# ── Aba Reading ───────────────────────────────────────────────────────
def test_a_aba_reading_constroi_sem_estourar(leitura):
    assert leitura._lc_live_lbls and leitura._lc_link_lbls
    assert leitura._lc_bar_items['nivel']


def test_reading_sem_dado_nenhum(leitura, raiz):
    """Como a aba abre antes de o receiver subir. Tem de dizer o que falta em
    vez de mostrar zero, que é um número plausível e errado."""
    leitura._refresh_lc_reading()
    raiz.update_idletasks()
    assert leitura._lc_net_lbl.cget('text') == '—   N'
    assert 'OFFLINE' in leitura._lc_link_lbls['board'].cget('text')


def test_reading_com_ponte_viva_mas_sem_forca(leitura, raiz):
    """O estado que o gate do force_receiver produz: chegam amostras, mas sem
    calibração ou sem tare não sai força. A aba tem de nomear a causa — senão
    parece placa morta, e o operador vai mexer no cabo."""
    with leitura._lock:
        leitura._lc_voltage, leitura._lc_voltage_ts = 1e-3, time.time()
        leitura._lc_arrivals.extend([time.time()] * 20)
    leitura._refresh_lc_reading()
    raiz.update_idletasks()
    assert 'ONLINE' in leitura._lc_link_lbls['board'].cget('text')
    assert 'no calibration loaded' in leitura._lc_net_status.cget('text')
    assert leitura._lc_link_lbls['calib'].cget('text') == 'MISSING'


def test_reading_mostra_os_tres_estagios(leitura, raiz):
    _vivo(leitura, v=2.5e-3, f_net=1.25, f_raw=1.40)
    leitura._refresh_lc_reading()
    raiz.update_idletasks()
    assert '+1.250' in leitura._lc_net_lbl.cget('text')
    assert '+2.5000 mV' in leitura._lc_live_lbls['v'].cget('text')
    assert '+1.400 N' in leitura._lc_live_lbls['raw'].cget('text')
    # kgf é a unidade em que as massas padrão estão escritas.
    assert leitura._lc_live_lbls['kgf'].cget('text').startswith('+0.127')


def test_reading_acusa_contato_no_limiar(leitura, raiz):
    _vivo(leitura, f_net=CONTACT_ON_N * 1.5)
    leitura._refresh_lc_reading()
    raiz.update_idletasks()
    assert leitura._lc_net_status.cget('text') == 'in contact'
    _vivo(leitura, f_net=CONTACT_ON_N * 0.5)
    leitura._refresh_lc_reading()
    raiz.update_idletasks()
    assert leitura._lc_net_status.cget('text') == 'no contact'


def test_reading_avisa_perto_do_aborto(leitura, raiz):
    _vivo(leitura, f_net=FORCE_ABORT_LIMIT_N * 0.95)
    leitura._refresh_lc_reading()
    raiz.update_idletasks()
    assert 'limit' in leitura._lc_net_status.cget('text')


def test_a_barra_nao_sai_do_canvas(leitura, raiz):
    """Força além do aborto (ou negativa, em tração) não pode desenhar fora do
    canvas — é o erro clássico de barra sem clamp."""
    for f in (-50.0, 0.0, FORCE_ABORT_LIMIT_N * 10):
        _vivo(leitura, f_net=f)
        leitura._refresh_lc_reading()
        raiz.update_idletasks()
        x0, _y0, x1, _y1 = leitura._lc_bar.coords(
            leitura._lc_bar_items['nivel'])
        assert 0.0 <= x0 <= x1 <= leitura._lc_bar.winfo_reqwidth()


def test_reading_sem_tare_avisa(leitura, raiz):
    _vivo(leitura, f_net=0.5, tared=False)
    leitura._refresh_lc_reading()
    raiz.update_idletasks()
    assert leitura._lc_net_status.cget('text') == 'tare not done'
    assert 'not done' in leitura._lc_tare_state_lbl.cget('text')


def test_reading_marca_quando_a_forca_e_simulada(raiz):
    """Com force_source:=sim quem publica /load_cell/force_net é o
    sim_force_bridge, e o número é o wrench do plugin FT do Gazebo: ~5,5 N
    do peso da pilha abaixo da célula, com a célula física DESLIGADA. Sem
    marca no card isso tem a mesma cara de uma leitura de bancada."""
    f_real, f_sim = tk.Frame(raiz), tk.Frame(raiz)
    h_real = _Host(f_real)
    h_real._build_lc_reading_tab(f_real)
    h_sim = _Host(f_sim)
    h_sim._force_source = 'sim'
    h_sim._build_lc_reading_tab(f_sim)
    raiz.update_idletasks()

    assert not any('SIMULADA' in t for t in h_real.titulos)
    assert any('SIMULADA' in t for t in h_sim.titulos)


def test_os_dois_zeros_sao_botoes_diferentes(leitura):
    """Tare (host) e re-zero (firmware) não são a mesma operação, e o painel
    tem de mandar cada um para o seu tópico."""
    leitura._lc_do_tare()
    assert (leitura._lc_tare_req_pub.n, leitura._lc_rezero_pub.n) == (1, 0)
    leitura._lc_do_rezero()
    assert (leitura._lc_tare_req_pub.n, leitura._lc_rezero_pub.n) == (1, 1)
    assert any('Re-zero' in m for m in leitura.status)


def test_a_taxa_medida_sai_das_chegadas(leitura, raiz):
    """A taxa é contada no host: o firmware carimba o tempo mas não diz
    quantas amostras chegaram, e é a cadência de chegada que responde se o
    pino RATE está em GND (10 Hz) ou em DVDD (80 Hz)."""
    from std_msgs.msg import Float32
    for _ in range(20):
        leitura._cb_lc_voltage(Float32(data=1e-3))
    leitura._refresh_lc_reading()
    raiz.update_idletasks()
    assert leitura._lc_link_lbls['rate'].cget('text') == '10.0 Hz'


# ── Aba Calibration: construção e repintura ───────────────────────────
def test_a_aba_constroi_sem_estourar(host):
    assert host._lc_calib_lbls and host._lc_points_txt
    assert host._lc_calib_points == []


def test_repinta_sem_nenhuma_amostra(host, raiz):
    """O estado em que a aba passa a maior parte do tempo: com a FA7155 no ar
    ninguém publica /load_cell/voltage, e a aba tem de dizer isso em vez de
    quebrar."""
    host._refresh_lc_calib()
    raiz.update_idletasks()
    assert 'Waiting' in host._lc_calib_note.cget('text')


def test_repinta_com_tensao_viva_e_sem_ajuste(host, raiz):
    import time
    with host._lock:
        host._lc_voltage = 1.2e-3
        host._lc_voltage_ts = time.time()
    host._refresh_lc_calib()
    raiz.update_idletasks()
    assert 'mV' in host._lc_calib_lbls['v'].cget('text')
    assert host._lc_calib_lbls['f'].cget('text') == 'fit first'


def test_a_tabela_de_pontos_repinta(host, raiz):
    _reta_boa(host)
    host._refresh_lc_points()
    raiz.update_idletasks()
    assert 'bridge mV' in host._lc_points_txt.get('1.0', 'end')


# ── As três guardas ───────────────────────────────────────────────────
def test_recusa_ajuste_sem_o_zero(host):
    """O V₀ é segurado FIXO pelo ajuste, então ele não é opcional: sem o zero
    medido não há o que segurar, e inventá-lo seria voltar ao ajuste de dois
    parâmetros que este wizard deixou de fazer."""
    for i in range(LC_CALIB_MIN_POINTS + 1):
        _ponto(host, 0.5 * (i + 1), 1e-3 * (i + 1))
    assert host._lc_do_fit() is None
    assert 'No zero captured' in host._lc_fit_lbl.cget('text')


def test_recusa_ajuste_com_menos_pontos_que_o_minimo(host):
    """Dois pontos SEMPRE dão uma reta perfeita. É o terceiro que produz
    resíduo, e o resíduo é a única coisa que denuncia massa digitada errada."""
    _zero(host)
    for i in range(LC_CALIB_MIN_POINTS - 1):
        _ponto(host, 0.5 * (i + 1), 1e-3 * (i + 1))
    assert host._lc_do_fit() is None
    assert str(LC_CALIB_MIN_POINTS) in host._lc_fit_lbl.cget('text')


def test_recusa_slope_fora_da_placa(host):
    """O erro clássico: massa em GRAMA onde se pede quilo. A reta continua
    lindíssima — só a escala do mundo muda por 1000 — então nenhum resíduo
    pega. Quem pega é a comparação com a placa."""
    _zero(host, 0.0)
    for i in range(5):
        g = 200.0 * (i + 1)                     # "gramas" digitadas como kg
        _ponto(host, g, LC_NOMINAL_V_PER_N * (g / 1000.0) * G_N_PER_KG)
    assert host._lc_do_fit() is None
    assert 'REFUSED' in host._lc_fit_lbl.cget('text')


def test_massa_repetida_e_recusada_na_coleta(host):
    _ponto(host, 1.0, 1e-3)
    host._lc_mass_var.set('1.0')
    with host._lock:
        host._lc_voltage_ts = time.time()
    host._lc_start_capture()
    assert host._lc_capture is None
    assert any('already in the table' in m for m in host.status)


def test_massa_ilegivel_nao_arma_coleta(host):
    host._lc_mass_var.set('meio quilo')
    host._lc_start_capture()
    assert host._lc_capture is None
    assert any('must be a number' in m for m in host.status)


def test_massa_acima_do_fundo_de_escala_e_recusada(host):
    host._lc_mass_var.set('250')
    host._lc_start_capture()
    assert host._lc_capture is None
    assert any('out of range' in m for m in host.status)


def test_sem_tensao_nao_arma_coleta(host):
    """Sem o force_receiver no ar a coleta mediria o silêncio e gravaria um
    ponto em zero — que é pior que não gravar nada, porque entra na reta."""
    host._lc_mass_var.set('0.5')
    host._lc_start_capture()
    assert host._lc_capture is None
    assert any('No voltage' in m for m in host.status)


# ── O ajuste ──────────────────────────────────────────────────────────
def test_ajusta_a_reta_nominal(host):
    _reta_boa(host)
    fit = host._lc_do_fit()
    assert fit is not None
    slope, intercept, pior = fit
    assert slope == pytest.approx(LC_NOMINAL_V_PER_N, rel=1e-9)
    # O V₀ sai do ajuste EXATAMENTE como entrou: ele é medido, não estimado.
    assert intercept == 3e-5
    assert pior < 1e-9


def test_a_regressao_e_de_V_em_funcao_de_F(host):
    """O erro está na TENSÃO (a massa padrão é conhecida). Regredir F sobre V
    enviesaria o slope para baixo na presença de ruído — este teste fixa a
    direção do ajuste com um outlier que só afeta um dos dois sentidos."""
    _reta_boa(host, n=6)
    m, f, v = host._lc_calib_points[-1]
    host._lc_calib_points[-1] = (m, f, v * 1.02)
    slope, _i, _p = host._lc_do_fit()
    assert slope > LC_NOMINAL_V_PER_N       # o outlier PUXA a tensão, não a força


def test_grava_o_json_que_o_receiver_le(host):
    from touch_pack.constants import lc_load_calibration
    _reta_boa(host)
    host._lc_save_calibration()
    d = json.loads(pathlib.Path(host._lc_calib_path).read_text(encoding='utf-8'))
    assert d['load_direction'] == 'compression'
    assert d['n_points'] == len(host._lc_calib_points)
    # O alias histórico vai junto: um JSON novo continua legível por versões
    # que só conhecem `zero_voltage`.
    assert d['zero_voltage'] == d['intercept']
    # E o LEITOR ÚNICO — o mesmo que o receiver usa — lê de volta a MESMA
    # reta e os MESMOS pontos. É este ida-e-volta que fecha o ciclo do
    # wizard: gravar num formato que só o wizard entende seria calibrar
    # para ninguém.
    slope, intercept, pontos = lc_load_calibration(host._lc_calib_path)
    assert (slope, intercept) == (d['slope'], d['intercept'])
    assert pontos == host._lc_calib_points


def test_ajuste_recusado_nao_grava_nada(host):
    _zero(host)
    _ponto(host, 1.0, 1e-3)
    host._lc_save_calibration()
    assert not pathlib.Path(host._lc_calib_path).exists()
    assert any('nothing was written' in m for m in host.status)


def test_limpar_pontos_zera_o_ajuste_e_o_zero(host):
    """O V₀ vai junto: mantê-lo sob pontos novos misturaria duas sessões de
    bancada, e o zero é justamente o que deriva entre elas."""
    _reta_boa(host)
    assert host._lc_do_fit() is not None
    host._lc_clear_points()
    assert host._lc_calib_points == []
    assert host._lc_calib_zero is None
    assert host._lc_calib_fit is None


# ── Backup da calibração antes de sobrescrever ────────────────────────
def test_salvar_guarda_a_calibracao_anterior(host, tmp_path, monkeypatch):
    """O alvo do Save é `sensors/load_cell_calib.json`, que é VERSIONADO —
    então há dois jeitos de perder uma calibração: um Save por cima apaga a
    antiga, e um `git checkout` distraído apaga a nova. A cópia cobre o
    primeiro, e vai para fora do git para não agravar o segundo.
    """
    cfg = tmp_path / 'cfg'
    monkeypatch.setattr(gui_lc_axial, 'CONFIG_DIR', str(cfg))
    pathlib.Path(host._lc_calib_path).write_text(
        json.dumps({'slope': 1.0e-3, 'intercept': 0.0}), encoding='utf-8')

    _reta_boa(host)
    host._lc_save_calibration()

    copias = sorted(cfg.glob('load_cell_calib.*.json'))
    assert len(copias) == 1, 'a calibração anterior não foi copiada'
    antiga = json.loads(copias[0].read_text(encoding='utf-8'))
    assert antiga['slope'] == 1.0e-3, 'a cópia não é a reta ANTIGA'
    nova = json.loads(pathlib.Path(host._lc_calib_path)
                      .read_text(encoding='utf-8'))
    assert nova['slope'] != antiga['slope']


def test_sem_arquivo_anterior_nao_ha_o_que_copiar(host, tmp_path, monkeypatch):
    """Primeira calibração de uma máquina nova: não existe nada a preservar, e
    o backup não pode inventar um arquivo nem atrapalhar a gravação."""
    cfg = tmp_path / 'cfg'
    monkeypatch.setattr(gui_lc_axial, 'CONFIG_DIR', str(cfg))
    assert not pathlib.Path(host._lc_calib_path).exists()
    _reta_boa(host)
    host._lc_save_calibration()
    assert pathlib.Path(host._lc_calib_path).exists()
    assert list(cfg.glob('load_cell_calib.*.json')) == []


def test_falha_no_backup_nao_impede_de_gravar(host, tmp_path, monkeypatch):
    """Backup é rede de segurança, não pré-requisito: um CONFIG_DIR sem
    permissão não pode custar uma calibração boa."""
    monkeypatch.setattr(gui_lc_axial, 'CONFIG_DIR', '/proc/nao/da/para/criar')
    pathlib.Path(host._lc_calib_path).write_text('{"slope": 1.0e-3}',
                                                 encoding='utf-8')
    _reta_boa(host)
    host._lc_save_calibration()
    d = json.loads(pathlib.Path(host._lc_calib_path).read_text(encoding='utf-8'))
    assert d['slope'] == pytest.approx(LC_NOMINAL_V_PER_N, rel=1e-9)


# ── Re-zero do firmware: a GUI não pode prometer o que não sabe ───────
def test_rezero_apenas_PEDE(leitura):
    """Publicar no tópico não põe o 'Z' no fio — quem escreve é o receiver, e
    ele pode falhar com a porta fechada."""
    leitura._lc_do_rezero()
    assert leitura._lc_rezero_pub.n == 1
    dito = ' '.join(leitura.status).lower()
    assert 'requested' in dito
    assert 'sent to the firmware' not in dito


# ── A resposta do receiver ao re-zero chega pelo canal do tare ────────
class _HostTare(gui_lc_axial.LcAxialMixin):
    """Só o que `_cb_lc_tare_result` (que mora no FtAxesMixin) precisa."""

    def __init__(self, cell='load_cell'):
        self._force_sensor = cell
        self._lock = threading.Lock()
        self._lc_tare_done = False
        self.status = []

    def _set_status(self, texto, _cor=None):
        self.status.append(texto)


def _tare_result(texto, cell='load_cell'):
    from std_msgs.msg import String
    from touch_pack.gui_loadcell import FtAxesMixin
    h = _HostTare(cell)
    FtAxesMixin._cb_lc_tare_result(h, String(data=texto))
    return ' '.join(h.status)


def test_rezero_confirmado_pelo_receiver():
    assert 'sent to the firmware' in _tare_result('rezero;ok;0')


def test_rezero_que_nao_chegou_ao_fio_e_denunciado():
    """O caso que a GUI ANTES escondia: a porta fechada fazia o receiver
    publicar err;no_link, ninguém tratava, e a mensagem otimista do botão
    ficava na tela."""
    dito = _tare_result('err;no_link;0')
    assert 'could NOT be sent' in dito
    assert 'did not happen' in dito


def test_o_texto_de_sem_dado_segue_a_celula():
    """Mandar conferir os 24 V e o par RS485 com a viga S no cabo é conselho
    para o hardware errado."""
    assert 'USB' in _tare_result('err;no_data;0', cell='load_cell')
    assert '24 V' in _tare_result('err;no_data;0', cell='ft6')
