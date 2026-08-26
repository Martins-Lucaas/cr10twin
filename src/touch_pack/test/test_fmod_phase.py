"""Fase MODULATING e o arranque em fase da onda trigonométrica.

Antes, a onda rodava carimbada como HOLD — a mesma fase do assentamento
inicial na força média. No CSV não havia como recortar o trecho da onda para
analisá-la, que é justamente o que caracteriza o ensaio.

Adicionar uma fase é uma mudança com raio: três consumidores decidem
comportamento a partir do NOME dela, e um deles (o mirror_node) passaria a
disputar o controle do braço com o explorer se não conhecesse a nova.
"""
import pytest

pytest.importorskip('rclpy')


def test_codigo_de_fase_existe_e_faz_round_trip():
    from touch_pack.constants import PHASE_CODES, PHASE_NAMES
    assert 'MODULATING' in PHASE_CODES
    code = PHASE_CODES['MODULATING']
    assert PHASE_NAMES[code] == 'MODULATING'
    # Não pode colidir com nenhuma fase pré-existente.
    outros = [v for k, v in PHASE_CODES.items()
              if k not in ('MODULATING', 'RETRACT', 'HOME')]
    assert code not in outros


def test_mirror_node_nao_disputa_o_braco_durante_a_onda():
    """_ACTIVE_PHASES é o que impede o mirror de mandar MovJ enquanto o
    explorer controla. Sem MODULATING na lista, os dois comandariam o CR10 ao
    mesmo tempo durante toda a onda."""
    from touch_pack.mirror_node import _ACTIVE_PHASES
    assert 'MODULATING' in _ACTIVE_PHASES


def test_relatorio_inclui_a_onda_no_resumo_de_forca():
    from touch_pack.palpation_report import _FORCE_PHASES, _PHASE_COLORS
    assert 'MODULATING' in _FORCE_PHASES, (
        'a onda é o trecho com controle de força que MAIS importa no resumo')
    assert 'MODULATING' in _PHASE_COLORS, 'fase sem cor cai no default do plot'


def test_gui_tem_cor_para_a_fase():
    import re
    from pathlib import Path
    import touch_pack.palpation_gui as g
    src = Path(g.__file__).read_text()
    bloco = re.search(r'phase_color = \{(.+?)\}\.get', src, re.S)
    assert bloco and 'MODULATING' in bloco.group(1)


# ── arranque em fase ──────────────────────────────────────────────────

@pytest.fixture(scope='module')
def P():
    from touch_pack.tactile_explorer import _ForceProfile
    return _ForceProfile


def test_seno_abre_na_media_e_nao_precisa_de_rampa(P):
    """SINE vale a média em t=0 — a rampa é no-op e nada muda."""
    p = P('SINE', 2.0, 3.0, 1.0, 10)
    dx0 = (p.setpoint_n(0.0) - p.mean_n)
    assert dx0 == pytest.approx(0.0, abs=1e-9)


def test_cosseno_abre_no_pico_e_exige_rampa(P):
    """COSINE vale mean+amp em t=0, mas o HOLD deixou o braço na MÉDIA.

    Sem levar a penetração até esse valor ANTES de o relógio começar, o
    primeiro tick pede a amplitude inteira e o teto por tick espalha isso
    como um degrau de força que não faz parte da onda.
    """
    p = P('COSINE', 2.0, 3.0, 1.0, 10)
    assert p.setpoint_n(0.0) == pytest.approx(p.mean_n + p.amp_n)
    assert p.setpoint_n(0.0) - p.mean_n == pytest.approx(p.amp_n)


def test_rampa_cabe_no_teto_de_ticks(P):
    """A rampa vale amp/K em passos de _FMOD_DF_STEP_MAX_N/K — o K se cancela,
    então o número de ticks é amp/_FMOD_DF_STEP_MAX_N, independente da
    ponteira. O teto existe só para não virar laço infinito."""
    import math
    from touch_pack.tactile_explorer import (
        _FMOD_DF_STEP_MAX_N, _FMOD_RAMP_MAX_TICKS, _FMOD_MAX_AMP_N)
    ticks = math.ceil(_FMOD_MAX_AMP_N / _FMOD_DF_STEP_MAX_N)
    assert ticks <= _FMOD_RAMP_MAX_TICKS, (
        f'amplitude máxima ({_FMOD_MAX_AMP_N} N) precisa de {ticks} ticks, '
        f'acima do teto {_FMOD_RAMP_MAX_TICKS} — a rampa sairia truncada')


def test_tolerancia_de_frequencia_e_util(P):
    """A tolerância tem de pegar um executor entregando metade da frequência
    e não gritar com o erro de ±1 meio-período da contagem."""
    from touch_pack.tactile_explorer import _FMOD_FREQ_TOL_FRAC
    assert 0.5 > _FMOD_FREQ_TOL_FRAC > 0.025, (
        'fora dessa faixa a checagem ou é cega para metade da frequência, '
        'ou avisa por causa do erro de borda da contagem')


def test_onda_no_silicone_pede_curso_muito_maior(P):
    """O curso da onda é amp/K: com a ponteira de silicone medida em bancada
    ele é ~45x maior que com a rígida. É o que torna a onda observável pela
    FK — e o que exige velocidade do braço."""
    p = P('SINE', 0.75, 1.25, 2.0, 10)      # amplitude 0,25 N
    curso_rigida_m = p.amp_n / 28_000.0
    curso_silicone_m = p.amp_n / 620.0
    assert curso_rigida_m < 1e-5            # < 10 µm: no piso de ruído da FK
    assert curso_silicone_m > 1e-4          # > 100 µm: visível
    assert curso_silicone_m / curso_rigida_m == pytest.approx(45.2, rel=0.05)


# ── tick próprio da onda (24 Hz) ──────────────────────────────────────

def test_tick_da_onda_e_derivado_da_frequencia(P):
    """O tick do QS (30 ms) existe porque ele MEDE — congela o braço para o
    pipeline esvaziar antes de ler. A onda não mede nada, é feedforward puro,
    e amarrá-la àquele tick a limitava a 33/8 ≈ 4 Hz por um motivo que não se
    aplica a ela."""
    from touch_pack.tactile_explorer import (
        _CTRL_DT, _FMOD_DT_MIN_S, _SERVOJ_T_MIN_S)
    # Frequência baixa: não faz sentido ir mais rápido que o tick do QS.
    assert P('SINE', 0.5, 1.5, 1.0, 5).wave_dt() == pytest.approx(_CTRL_DT)
    # Frequência alta: desce até o piso, nunca abaixo. O piso REAL é o do
    # ServoJ (20 ms), não o do laço Python (4 ms) — pedir um período menor não
    # acelera o braço, o firmware recusa o ponto (guia V4.5.1: "t ... value
    # range: [0.02,3600.0]"). Antes este teste pedia 4 ms e recebia 4 ms, o
    # que descrevia um comando que o CR10 nunca aceitaria.
    assert P('SINE', 0.5, 1.5, 100.0, 5).wave_dt(_FMOD_DT_MIN_S) == \
        pytest.approx(_SERVOJ_T_MIN_S)
    # O piso do laço Python continua documentado, mas é DOMINADO pelo do
    # ServoJ — se um dia o hardware permitir menos, ele volta a morder.
    assert _FMOD_DT_MIN_S >= 0.002
    assert _SERVOJ_T_MIN_S > _FMOD_DT_MIN_S


def test_tick_nunca_fica_abaixo_do_periodo_do_servoj(P):
    """Publicar mais rápido que o laço ServoJ não entrega mais onda — o
    mirror amostra o ÚLTIMO alvo e o excedente é DESCARTADO. Foi o que houve
    no run 20260814_115804: tick de 25 ms contra 30 ms do mirror."""
    p = P('SINE', 0.1, 3.0, 8.0, 10)
    assert p.wave_dt(0.030) == pytest.approx(0.030)
    # Subindo o mirror, o tick acompanha e a onda passa a caber.
    assert p.wave_dt(0.025) == pytest.approx(0.025)


def test_teto_de_frequencia_segue_o_periodo_do_servoj():
    """6,7 Hz com os 30 ms padrão; 10 Hz exige o mirror no piso do firmware
    (20 ms). É este teto que o explorer usa para RECUSAR a onda em vez de
    reamostrá-la."""
    from touch_pack.tactile_explorer import _fmod_max_freq_hz
    assert _fmod_max_freq_hz(0.030) == pytest.approx(6.6666, rel=1e-3)
    assert _fmod_max_freq_hz(0.025) == pytest.approx(8.0)
    assert _fmod_max_freq_hz(0.020) == pytest.approx(10.0)


@pytest.mark.parametrize('hz', [1.0, 2.0, 4.0, 6.25, 10.0])
def test_pontos_por_periodo_suficientes_ate_o_teto_do_servoj(P, hz):
    """Até o teto do FIRMWARE a onda COMANDADA tem os pontos por período que
    _FMOD_MIN_PTS_PER_CYCLE exige — inclusive nos 10 Hz, que são exatamente
    o piso de 20 ms com 5 pontos.

    O teto era 24 Hz neste teste, mas os 24 Hz nunca foram executáveis: eles
    exigiriam ServoJ com t = 1/(24*8) = 5,2 ms, e o guia V4.5.1 dá a faixa
    [0.02, 3600] s. O período que o teste calculava como "o que o operador
    tem de configurar" era justamente um valor que o controlador recusa.
    """
    from touch_pack.tactile_explorer import (
        _FMOD_MIN_PTS_PER_CYCLE, _SERVOJ_T_MIN_S, _fmod_max_freq_hz)
    p = P('SINE', 0.5, 1.5, hz, 10)
    # O período de ServoJ que este ensaio exige — e que precisa ser LEGAL.
    period = 1.0 / (hz * _FMOD_MIN_PTS_PER_CYCLE)
    assert period >= _SERVOJ_T_MIN_S - 1e-9, (
        f'{hz} Hz pediria t={period*1e3:.1f} ms, abaixo do mínimo do ServoJ')
    assert _fmod_max_freq_hz(period) >= hz - 1e-9
    assert p.pts_per_cycle_at(p.wave_dt(period)) >= \
        _FMOD_MIN_PTS_PER_CYCLE - 1e-9


def test_gui_nao_deixa_pedir_o_que_o_firmware_recusa():
    """O painel não pode oferecer frequência que o CR10 não executa.

    Com t mínimo de 20 ms e 5 pontos por período, o teto FÍSICO é 10,0 Hz.
    O painel oferecia 30 Hz."""
    from touch_pack.palpation_gui import FMOD_HZ_MAX, fmod_wave_dt
    from touch_pack.tactile_explorer import (
        _ForceProfile, _SERVOJ_T_MIN_S, _fmod_max_freq_hz)
    teto_hw = _fmod_max_freq_hz(_SERVOJ_T_MIN_S)
    assert teto_hw == pytest.approx(10.0)
    assert FMOD_HZ_MAX <= teto_hw * 1.001, (
        f'GUI deixa pedir {FMOD_HZ_MAX} Hz mas o firmware só entrega '
        f'{teto_hw:.2f} Hz')
    # E o preview da GUI tem de prever o MESMO tick que o explorer executa.
    for hz in (0.5, 1.0, 4.0, 6.25, 10.0):
        assert fmod_wave_dt(hz) == pytest.approx(
            _ForceProfile('SINE', 0.5, 1.5, hz, 5).wave_dt())


def test_piso_do_servoj_vale_em_toda_a_cadeia():
    """explorer, GUI e driver têm de concordar no mínimo do `t` do ServoJ —
    é um número do firmware, não uma preferência de cada módulo."""
    from touch_pack.tactile_explorer import _SERVOJ_T_MIN_S as EXP
    from touch_pack.palpation_gui import SERVOJ_T_MIN_S as GUI
    from touch_pack.real_driver import SERVOJ_T_MIN_S as DRV
    assert EXP == GUI == DRV == 0.020


def test_adaptacao_de_k_tem_parametros_sensatos():
    from touch_pack.tactile_explorer import (
        _FMOD_K_ADAPT_ALPHA, _FMOD_K_ADAPT_MIN_DF_N)
    assert 0.0 < _FMOD_K_ADAPT_ALPHA < 1.0
    # Limiar acima do ruído de pico da célula (0,037 N), senão o "K medido"
    # seria ruído dividido por deslocamento.
    assert _FMOD_K_ADAPT_MIN_DF_N > 0.02


# ── setpoint_n tem de SER um setpoint ─────────────────────────────────

def test_onda_comandada_nunca_sai_da_faixa(P):
    """A coluna setpoint_n do CSV carrega prof.setpoint_n(t). Ela tem de ficar
    dentro de [f_min, f_max] em qualquer instante — foi por não ficar que a
    reconstrução por FK (média + K·Δx) foi abandonada: media −1,085 a 4,199 N
    numa onda pedida de 0,1 a 3,0 N (bancada, 14/08/2026)."""
    p = P('SINE', 0.1, 3.0, 1.0, 10)
    vals = [p.setpoint_n(i / 500.0) for i in range(5000)]
    assert min(vals) >= p.f_min_n - 1e-6
    assert max(vals) <= p.f_max_n + 1e-6
    # E o valor de fim da drenagem também.
    fin = p.setpoint_n(p.duration_s)
    assert p.f_min_n - 1e-6 <= fin <= p.f_max_n + 1e-6


def test_banda_morta_do_servoj_nao_engole_a_onda():
    """O mirror amostra o último q e descarta alvos que mudaram menos que a
    banda morta. Ela precisa ser MUITO menor que a onda, senão a suprime.

    Uma amplitude típica no silicone (±0,5 N / 0,62 N/mm ≈ 800 µm) vira ~7e-4
    rad num braço de ~1,2 m; a banda morta antiga, de 1e-4 rad (~120 µm de
    TCP), quantizaria isso em ~6 degraus."""
    from touch_pack.mirror_node import _SERVOJ_DEADBAND_RAD
    ALCANCE_M = 1.2
    tcp_equiv_m = _SERVOJ_DEADBAND_RAD * ALCANCE_M
    assert tcp_equiv_m < 20e-6, (
        f'banda morta vale {tcp_equiv_m*1e6:.0f} µm de TCP — grande demais '
        'para uma onda micrométrica')


def test_os_dois_caminhos_de_servoj_usam_a_mesma_banda_morta():
    """A GUI também espelha para o braço real (palpation_gui). Uma banda
    maior lá engoliria a onda que o mirror deixa passar, e o sintoma
    dependeria de qual dos dois estava no ar."""
    from touch_pack.mirror_node import _SERVOJ_DEADBAND_RAD
    from touch_pack.palpation_gui import SERVOJ_DEADBAND_RAD
    assert SERVOJ_DEADBAND_RAD == pytest.approx(_SERVOJ_DEADBAND_RAD)


# ── caminho de execução e segurança da onda ───────────────────────────

def test_perfil_configurado_e_detectado_sem_efeito_colateral():
    """_fmod_configured não pode validar nem logar — ela roda antes da 1a fase
    só para escolher o caminho, e _force_profile já loga por conta própria."""
    import inspect
    from touch_pack.tactile_explorer import TactileExplorer
    src = inspect.getsource(TactileExplorer._fmod_configured)
    assert 'get_logger' not in src, 'checagem barata não deve logar'
    for forma in ('SINE', 'COSINE'):
        assert forma in src


def test_teto_de_velocidade_cobre_as_ondas_legitimas():
    """O teto tem de deixar passar o ensaio pedido e cortar o disparado.

    Pior caso legítimo: 24 Hz com a amplitude máxima que o silicone permite
    dentro da faixa de força — acima disso é erro de K, não experimento.
    """
    import math
    from touch_pack.tactile_explorer import _FMOD_V_MAX_MMS
    K_SIL = 620.0
    # 24 Hz, amplitude 0,25 N no silicone: 0,40 mm de curso.
    x_m = 0.25 / K_SIL
    v_pico_mms = 2 * math.pi * 24.0 * x_m * 1e3
    assert v_pico_mms < _FMOD_V_MAX_MMS, (
        f'onda legítima de 24 Hz pede {v_pico_mms:.0f} mm/s, acima do teto')
    # E o teto não pode ser tão alto que deixe de proteger.
    assert _FMOD_V_MAX_MMS <= 300.0


def test_limites_de_seguranca_da_onda_seguem_ativos():
    """A onda roda fora do _qs_regulate, então as travas dela são próprias.
    Se alguma sumir do laço, o ensaio deixa de ser seguro."""
    import inspect
    from touch_pack.tactile_explorer import TactileExplorer
    src = inspect.getsource(TactileExplorer._phase_hold_modulated)
    for trava, o_que in [
            ('_force_over_limit', 'teto de força'),
            ('_force_stale_abort', 'célula sem dados frescos'),
            ('_stop_requested', 'STOP do usuário'),
            ('_pause_gate', 'PAUSE'),
            ('_relieve_contact', 'alívio ao estourar a força'),
            ('_FMOD_V_MAX_MMS', 'teto de velocidade'),
            ('step_cap_m', 'teto de ΔF por passo')]:
        assert trava in src, f'trava ausente no laço da onda: {o_que}'
