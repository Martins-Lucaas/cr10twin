"""Controle repetitivo (ILC) da onda de força: o que ele exige para funcionar.

O `_WaveILC` existia completo desde o commit da modulação e NUNCA era
chamado — nem instanciado. A onda rodava com `fx_gain`, UM escalar ajustado
pelo módulo do lock-in, que corrige amplitude e mais nada. Medido no run
TOUCH/20260828_154934 (SINE 0,20–2,00 N @ 1 Hz), é exatamente o que se via:
amplitude certa (0,933 N contra 0,890 pedidos), centro +0,355 N e derivando,
THD de 32 %, fase −55,5°.

Ligar o vetor exige DUAS coisas que não são detalhe de sintonia; errar
qualquer uma faz a onda ficar PIOR do que sem ILC nenhum, e por isso cada uma
tem teste próprio aqui:

  1. uma medida que exista na frequência da onda. O One-Euro está travado em
     2 Hz, então a 10 Hz ele entrega 20 % da amplitude. Um laço que se adapta
     por essa leitura conclui que a onda está curta e manda cinco vezes mais
     curso: simulado, 149 % de amplitude e pico de 2,62 N numa onda pedida
     até 2,00 N.

  2. a FASE do plano, medida e não estimada. O ILC indexa a correção por
     fase; recebendo a fase errada ele não corrige menos, ele realimenta
     positivamente. Simulado a 10 Hz com o resto perfeito:

         erro de fase   fundamental entregue   THD
              0°              100 %             2 %
             50°              144 %            32 %
            194°              169 %            35 %   (diverge)
"""
import math

import pytest

pytest.importorskip('rclpy')


# ── 1. A medida existe nesta frequência? ─────────────────────────────

@pytest.mark.parametrize('f_hz, esperado', [
    (0.5, 0.970), (1.0, 0.894), (2.0, 0.707), (4.0, 0.447), (10.0, 0.196)])
def test_ganho_da_medida_segue_o_passa_baixa_de_2hz(f_hz, esperado):
    """O cutoff do One-Euro está TRAVADO em ONE_EURO_MAXCUTOFF_HZ; um
    passa-baixa de 1ª ordem ali vale 1/√(1+(f/fc)²)."""
    from touch_pack.tactile_explorer import fmod_measure_gain
    assert fmod_measure_gain(f_hz) == pytest.approx(esperado, abs=0.002)


def test_o_portao_do_ilc_bate_com_o_cutoff_do_filtro():
    """O teto não é um número escolhido: é onde o filtro deixa de ser
    transparente. Abrir acima disso é autorizar a sobre-excitação."""
    from touch_pack.tactile_explorer import (
        fmod_measure_gain, _FMOD_ILC_MIN_MEAS_GAIN, _ONE_EURO_MAXCUTOFF_HZ)
    assert fmod_measure_gain(_ONE_EURO_MAXCUTOFF_HZ) >= _FMOD_ILC_MIN_MEAS_GAIN
    assert fmod_measure_gain(2.5) < _FMOD_ILC_MIN_MEAS_GAIN
    assert fmod_measure_gain(10.0) < _FMOD_ILC_MIN_MEAS_GAIN


def test_ganho_da_medida_e_um_em_dc():
    """Em DC o filtro é transparente, e é por isso que a trava de segurança
    de 12 N pode continuar lendo o sinal FILTRADO: uma sobrecarga sustentada
    aparece nele inteira."""
    from touch_pack.tactile_explorer import fmod_measure_gain
    assert fmod_measure_gain(0.0) == pytest.approx(1.0)


# ── 2. Anti-windup: ciclo cortado não é ciclo aprendido ──────────────

def test_discard_nao_move_a_correcao_e_limpa_o_ciclo():
    """No ciclo em que o limitador cortou, o comando não foi o que o laço
    pediu — parte do erro é obra do corte. Aprender ali ensina o vetor a
    empurrar mais contra o limitador, que corta mais."""
    from touch_pack.tactile_explorer import _WaveILC
    ilc = _WaveILC(n_bins=8, alpha=0.5, clip_m=1e-3)
    for i in range(8):
        ilc.observe(i / 8.0, 1.0, 1000.0)
    antes = ilc.corr.copy()
    ilc.discard()
    assert (ilc.corr == antes).all(), 'discard não pode mover a correção'
    # e o ciclo seguinte começa limpo: um commit sem observações é no-op
    ilc.commit()
    assert (ilc.corr == antes).all()


def test_commit_move_a_correcao_no_sentido_do_erro():
    """Guarda de SINAL. Uma correção com o sinal trocado é a única forma de
    o ILC afundar a ponteira em vez de corrigir a onda."""
    from touch_pack.tactile_explorer import _WaveILC
    ilc = _WaveILC(n_bins=8, alpha=0.5, clip_m=1e-3)
    # erro POSITIVO (alvo acima do medido) => precisa aprofundar MAIS
    for i in range(8):
        ilc.observe(i / 8.0, +0.5, 1000.0)
    ilc.commit()
    assert ilc.corr.mean() > 0.0


def test_a_correcao_respeita_o_teto():
    from touch_pack.tactile_explorer import _WaveILC
    ilc = _WaveILC(n_bins=8, alpha=1.0, clip_m=1e-4)
    for _ in range(30):
        for i in range(8):
            ilc.observe(i / 8.0, 100.0, 1000.0)
        ilc.commit()
    assert abs(ilc.corr).max() <= 1e-4 + 1e-12


# ── 3. A fase do plano, medida pelo lock-in ──────────────────────────

def _lockin_lag(f_hz, dt, lag_real_s, n_cycles=4):
    """Reproduz o cálculo do laço: fase entre o fasor da FORÇA e o da
    penetração COMANDADA, convertida em atraso e embrulhada num período."""
    fi = fq = ci = cq = 0.0
    n = int(n_cycles / f_hz / dt)
    cmd = {}
    for i in range(n):
        t = i * dt
        c = math.sin(2 * math.pi * f_hz * t)
        cmd[i] = c
        f = cmd.get(round((t - lag_real_s) / dt), 0.0)
        w = 2 * math.pi * f_hz * t
        fi += f * math.sin(w); fq += f * math.cos(w)
        ci += c * math.sin(w); cq += c * math.cos(w)
    ph = math.atan2(fq, fi) - math.atan2(cq, ci)
    ph = (ph + math.pi) % (2 * math.pi) - math.pi
    return (-ph / (2 * math.pi * f_hz)) % (1.0 / f_hz)


@pytest.mark.parametrize('f_hz, dt, lag', [
    (1.0, 0.030, 0.050), (1.0, 0.030, 0.154),
    (5.0, 0.020, 0.050), (10.0, 0.020, 0.050)])
def test_lockin_recupera_o_atraso_do_plano(f_hz, dt, lag):
    """O atraso sai da fase entre dois lock-ins que o laço já acumula — não
    custa movimento nenhum e mede nas condições do próprio ensaio."""
    medido = _lockin_lag(f_hz, dt, lag)
    erro_graus = 360.0 * (medido - lag % (1.0 / f_hz)) * f_hz
    erro_graus = (erro_graus + 180.0) % 360.0 - 180.0
    assert abs(erro_graus) < 25.0, f'erro de fase {erro_graus:.0f}°'


def test_o_atraso_e_medido_modulo_um_periodo():
    """A 10 Hz o atraso de 154 ms vale 1,54 ciclos. O ILC indexa por FASE,
    então o que ele precisa é o RESTO (54 ms) — distinguir 54 de 154 não
    mudaria em que bin o erro cai. É por isso que medir a fase basta e não é
    preciso desenrolar o número de ciclos."""
    medido = _lockin_lag(10.0, 0.020, 0.154)
    assert medido == pytest.approx(0.054, abs=0.010)
    assert medido < 0.100, 'não pode passar de um período'


def test_a_formula_analitica_so_vale_onde_o_filtro_domina():
    """fmod_measure_lag_s responde o atraso do FILTRO. Com o sinal cru o
    filtro sai do caminho e sobra o transporte, que a fórmula não conhece —
    daí o atraso ser medido e não calculado."""
    from touch_pack.tactile_explorer import fmod_measure_lag_s
    # a 1 Hz o filtro domina e a fórmula é uma boa semente
    assert 0.060 < fmod_measure_lag_s(1.0) < 0.120
    # a fórmula CAI com a frequência; o transporte não. Confiar nela a 10 Hz
    # é o erro de fase que faz o ILC divergir.
    assert fmod_measure_lag_s(10.0) < fmod_measure_lag_s(1.0)


# ── 3. A fase depende da CÉLULA que está no fio ──────────────────────
# A bancada trocou a HX711 (24 Hz) pela FA7155 (~400 Hz entregues em polled).
# O termo da mediana do atraso é meia janela DA TAXA DA FONTE, então trocar a
# célula sem trocar a taxa gira a correção do ILC em fase — o modo de falha
# do bloco 2 acima, só que silencioso.

def test_o_atraso_da_mediana_segue_a_taxa_da_fonte():
    """Só o termo da mediana muda com a célula; o do One-Euro não, porque o
    cutoff está travado em 2 Hz nas duas taxas (min(rate/3, 2) = 2)."""
    from touch_pack.tactile_explorer import fmod_measure_lag_s, _FMOD_ILC_BINS
    from touch_pack.constants import FT_NOMINAL_RATE_HZ, LC_NOMINAL_RATE_HZ
    from touch_pack.lc_filter import MEDIAN_N
    f_hz = 1.0
    lag_hx = fmod_measure_lag_s(f_hz, LC_NOMINAL_RATE_HZ)
    lag_ft = fmod_measure_lag_s(f_hz, FT_NOMINAL_RATE_HZ)
    esperado = 0.5 * (MEDIAN_N - 1) * (1.0 / LC_NOMINAL_RATE_HZ
                                       - 1.0 / FT_NOMINAL_RATE_HZ)
    assert lag_hx - lag_ft == pytest.approx(esperado, abs=1e-9)
    assert lag_ft == pytest.approx(0.0763, abs=0.001)
    # E não é diferença cosmética: a 1 Hz ela vale quase um bin INTEIRO do
    # ILC, ou seja o erro cairia no bin vizinho o ensaio todo.
    bin_s = 1.0 / (f_hz * _FMOD_ILC_BINS)
    assert (lag_hx - lag_ft) > 0.75 * bin_s


def test_a_onda_passa_a_taxa_medida_e_nao_a_constante_da_celula():
    """Os call sites do explorer não podem cair no default da função: ele é
    o nominal da HX711, e a célula no fio é outra."""
    import inspect
    from touch_pack import tactile_explorer as te
    chamadas = [ln for ln in inspect.getsource(te).splitlines()
                if 'fmod_measure_lag_s(' in ln and 'def ' not in ln]
    usos = [ln for ln in chamadas if 'prof.freq_hz' in ln]
    assert usos, 'nenhum call site de fmod_measure_lag_s encontrado'
    for ln in usos:
        assert '_lc_rate_hz()' in ln, f'call site sem a taxa medida: {ln.strip()}'


def test_a_taxa_cai_no_nominal_enquanto_nenhuma_leitura_chegou():
    """Sem amostra não há taxa medida. O fallback é o pior caso (maior
    atraso) e, na prática, onda nenhuma roda nesse estado — _FORCE_STALE_S
    barra antes."""
    import threading
    import types
    from touch_pack.tactile_explorer import TactileExplorer
    from touch_pack.constants import LC_NOMINAL_RATE_HZ
    stub = types.SimpleNamespace(_lc_lock=threading.Lock(),
                                 _lc_rate_ema_hz=0.0)
    assert TactileExplorer._lc_rate_hz(stub) == LC_NOMINAL_RATE_HZ
    stub._lc_rate_ema_hz = 399.7
    assert TactileExplorer._lc_rate_hz(stub) == pytest.approx(399.7)


def test_a_taxa_medida_acompanha_a_cadencia_que_chega(monkeypatch):
    """A taxa sai da CHEGADA das leituras, então trocar a fonte no fio troca
    o atraso usado pelo ILC sem ninguém reconfigurar nada."""
    import threading
    import types
    import touch_pack.tactile_explorer as te

    relogio = {'t': 100.0}
    monkeypatch.setattr(te.time, 'monotonic', lambda: relogio['t'])
    stub = types.SimpleNamespace(_lc_lock=threading.Lock(),
                                 _lc_rate_ema_hz=0.0, _lc_force_net=0.0,
                                 _lc_force_ts=0.0, _lc_force_seq=0,
                                 _LC_MAX_PLAUSIBLE_N=100.0)

    def alimenta(rate_hz, n):
        for _ in range(n):
            relogio['t'] += 1.0 / rate_hz
            te.TactileExplorer._cb_lc_force_net(
                stub, types.SimpleNamespace(data=1.0))

    alimenta(24.0, 200)      # HX711
    assert stub._lc_rate_ema_hz == pytest.approx(24.0, abs=0.5)
    alimenta(400.0, 1000)    # FA7155
    assert stub._lc_rate_ema_hz == pytest.approx(400.0, abs=5.0)
    assert stub._lc_force_seq == 1200
