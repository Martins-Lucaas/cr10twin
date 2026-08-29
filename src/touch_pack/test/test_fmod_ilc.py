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
