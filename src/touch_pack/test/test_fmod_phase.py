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


def test_gui_tem_cor_para_a_fase(gui_real):
    from touch_pack.ui_helpers import OK
    with gui_real._lock:
        gui_real._latest_phase = 'MODULATING'
        gui_real._paused = False
    gui_real._status_panel_tick()
    assert gui_real.phase_lbl.cget('text').startswith('MODULATING')
    assert gui_real.phase_lbl.cget('fg') == OK       # não a cor genérica


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

def test_perfil_configurado_e_detectado_sem_efeito_colateral(braco):
    """_fmod_configured não pode validar nem logar — ela roda antes da 1a fase
    só para escolher o caminho, e _force_profile já loga por conta própria."""
    from rclpy.parameter import Parameter
    for forma, quer in (('SINE', True), ('COSINE', True), ('OFF', False)):
        braco.set_parameters([Parameter('force_mod_shape', value=forma),
                              # faixa INVÁLIDA: validar aqui logaria
                              Parameter('force_mod_min_n', value=3.0),
                              Parameter('force_mod_max_n', value=1.0)])
        braco.log.linhas.clear()
        assert bool(braco._fmod_configured()) is quer, forma
        assert braco.log.linhas == []


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


def test_limites_de_seguranca_da_onda_seguem_ativos(onda):
    """A onda roda fora do _qs_regulate, então as travas dela são próprias.
    Cada falha é INJETADA no meio da onda e tem de pará-la ali."""
    n = onda.n
    alivio = []
    n._relieve_contact = lambda *a, **k: alivio.append(1)

    def meio():
        return bool(onda.t0) and onda.fase(onda.rel.t) > 1.2

    n._force_stale_abort = lambda fase: meio()
    assert onda.roda() == 'stale' and onda.durou < 1.4
    n._force_stale_abort = lambda fase: False

    base = n._qs_step
    n._qs_step = lambda *a, **k: (
        n._stop_requested.set() if meio() else None, base(*a, **k))[1]
    assert onda.roda() == 'stop' and onda.durou < 1.4
    assert not n._stop_requested.is_set()
    n._qs_step = base

    gate = n._pause_gate
    n._pause_gate = lambda: not meio()        # STOP durante a pausa
    assert onda.roda() == 'stop' and onda.durou < 1.4
    n._pause_gate = gate

    assert not alivio
    onda.deforma = lambda c, f: 13.0 if c > 1.2 else f
    assert onda.roda() == 'force' and onda.durou < 1.4
    assert alivio, 'não aliviou o contato ao estourar a força'


# ── 10 Hz: a cadeia de comando inteira ────────────────────────────────

def test_launch_expoe_servoj_period_s(monkeypatch):
    """O teto de frequência da onda é 1/(t·5), e `t` estava escrito à mão em
    TRÊS lugares sem nenhum argumento de launch por cima. Roda o launch real
    com `servoj_period_s:=0.020` e lê o que chega aos nós."""
    import importlib.util
    from pathlib import Path
    launch = pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    import touch_pack
    src = (Path(touch_pack.__file__).parent.parent
           / 'launch' / 'tactile_cell.launch.py')
    if not src.exists():          # instalado sem os launch/ ao lado
        pytest.skip('launch não disponível neste layout')
    spec = importlib.util.spec_from_file_location('tc_launch', src)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    vistos = {}
    real = m.Node

    def _no(*a, **kw):
        for p in kw.get('parameters') or []:
            if isinstance(p, dict) and 'servoj_period_s' in p:
                vistos[kw.get('executable')] = p['servoj_period_s']
        return real(*a, **kw)
    monkeypatch.setattr(m, 'Node', _no)
    ld = m.generate_launch_description()
    ctx = launch.LaunchContext()
    for d in ld.entities:
        if isinstance(d, DeclareLaunchArgument):
            d.execute(ctx)
    assert 'servoj_period_s' in ctx.launch_configurations
    # no_gui + mirror: sobem explorer, GUI (nó condicionado) e mirror_node.
    ctx.launch_configurations.update(
        servoj_period_s='0.020', control_mode='mirror', no_gui='true',
        end_effector='touch_tool', robot_ip='127.0.0.1')
    for e in ld.entities:
        if isinstance(e, OpaqueFunction):
            e.execute(ctx)
    assert len(vistos) == 3, vistos
    assert set(vistos.values()) == {0.020}, vistos


@pytest.mark.parametrize('periodo', [0.020, 0.030])
def test_gui_nao_prende_o_servoj_em_30ms(periodo, monkeypatch, relogio):
    """O poll loop da GUI é quem comanda o braço real quando ela está aberta.
    Com o período preso em 30 ms, 10 Hz era impossível pela GUI. Roda o laço
    real em relógio virtual e mede a cadência."""
    import threading
    import types
    import numpy as np
    import touch_pack.gui_robot as gr
    monkeypatch.setattr(gr, 'time', relogio)
    monkeypatch.setattr(gr, '_urdf_to_dobot', lambda q: q)
    ticks = []
    ev = types.SimpleNamespace(is_set=lambda: len(ticks) >= 20,
                               wait=relogio.sleep)
    g = types.SimpleNamespace(
        _stop_event=ev, _servoj_period_s=periodo, _robot_mode='MIRROR',
        _robot_connected=True, _real_driver=object(), _drag_enabled=False,
        _exec_movement_id=None, _lock=threading.Lock(),
        _latest_phase='MODULATING', _latest_joint_rad=[0.0] * 6,
        _mirror_hold=False, _paused=False, _estop_latched=False,
        _servo_tick=lambda d, p: ticks.append(relogio.t))
    gr.RobotMixin._mirror_poll_loop(g)
    assert np.diff(ticks) == pytest.approx(periodo)


def test_banda_morta_tem_fonte_unica():
    """Eram dois literais iguais, em módulos que comandam o MESMO braço,
    mantidos em sincronia por um teste em vez de por uma definição — e o
    explorer não tinha como consultá-la para avisar sobre quantização."""
    from touch_pack.constants import SERVOJ_DEADBAND_RAD as BASE
    from touch_pack.mirror_node import _SERVOJ_DEADBAND_RAD
    from touch_pack.palpation_gui import SERVOJ_DEADBAND_RAD as GUI
    assert _SERVOJ_DEADBAND_RAD is BASE
    assert GUI is BASE


def test_teto_de_10hz_com_o_piso_do_firmware(P):
    """O contrato do ensaio de 10 Hz: t=20 ms dá exatamente 5 pontos por
    período, e o tick da onda desce até lá."""
    from touch_pack.tactile_explorer import (
        _fmod_max_freq_hz, _SERVOJ_T_MIN_S, _FMOD_MIN_PTS_PER_CYCLE)
    assert _fmod_max_freq_hz(_SERVOJ_T_MIN_S) == pytest.approx(10.0)
    p = P('SINE', 0.5, 1.5, 10.0, 20)
    assert p.wave_dt(_SERVOJ_T_MIN_S) == pytest.approx(_SERVOJ_T_MIN_S)
    assert p.pts_per_cycle_at(p.wave_dt(_SERVOJ_T_MIN_S)) == \
        pytest.approx(_FMOD_MIN_PTS_PER_CYCLE)


# ── 10 Hz: o que a precisão exige ─────────────────────────────────────

def test_medida_e_drenada_por_amostra_e_nao_por_tick(onda):
    """A 10 Hz o laço roda a 50 Hz e a célula a 400. Ler a célula NO TICK
    amostra 400 Hz a 50, e os harmônicos da interpolação do comando dobram
    sobre a fundamental. Cada amostra entra UMA vez, com o carimbo DELA."""
    import numpy as np
    onda.k_est = 1300.0                   # 30 % acima da planta (1000 N/m)
    assert onda.roda(hz=10.0, f_min=1.4, f_max=1.6, ciclos=20) == 'ok'
    t = np.array([o[0] for o in onda.observadas])
    assert len(t) == pytest.approx(400 * onda.prof.duration_s, rel=0.05)
    assert np.diff(t) == pytest.approx(1 / 400.0, abs=1e-6)
    # Força e posição têm contadores próprios: trocá-los erra ΔF/Δx por 8× e
    # a rigidez adaptada sairia da planta para outra ordem de grandeza.
    assert onda.n._k_est.k == pytest.approx(1000.0, rel=0.28)
    assert onda.n._k_est.k < 1300.0


def test_lock_in_por_amostra_recupera_a_amplitude_aliasada():
    """O motivo do buffer, em números: uma senoide reconstruída por
    interpolação linear de 5 pontos por período, medida NO TICK (5 amostras
    por período), tem a fundamental contaminada; medida a 40 amostras por
    período, não."""
    import math
    f, amp, cycles = 10.0, 1.0, 20

    def wave(t):
        """Onda como o braço a executa: linear entre os pontos comandados."""
        n = 5
        dt = 1.0 / (f * n)
        i = math.floor(t / dt)
        a, b = math.sin(2*math.pi*f*i*dt), math.sin(2*math.pi*f*(i+1)*dt)
        return amp * (a + (b - a) * (t / dt - i))

    def fundamental(rate):
        n = int(cycles * rate / f)
        si = sum(wave(k/rate) * math.sin(2*math.pi*f*k/rate) for k in range(n))
        sq = sum(wave(k/rate) * math.cos(2*math.pi*f*k/rate) for k in range(n))
        return 2.0 * math.hypot(si, sq) / n

    # No tick do comando (5 amostras/período) a leitura é MUITO maior que a
    # fundamental verdadeira — é o alias de 4f e 6f caindo sobre f.
    no_tick = fundamental(f * 5)
    # Na taxa da célula (40 amostras/período) ela converge para o sinc².
    na_celula = fundamental(f * 40)
    esperado = amp * (math.sin(math.pi/5) / (math.pi/5)) ** 2
    assert na_celula == pytest.approx(esperado, rel=0.02)
    assert no_tick > na_celula * 1.10, (
        f'amostrar no tick deveria inflar a fundamental: {no_tick:.3f} vs '
        f'{na_celula:.3f}')


@pytest.mark.parametrize('hz,quer', [(10.0, 5), (1.0, None)])
def test_bins_do_ilc_acompanham_os_pontos_comandados(onda, monkeypatch, hz,
                                                     quer):
    """Correção indexada numa grade mais fina que a grade em que o comando
    existe não dá resolução — dá graus de liberdade que só o ruído preenche.
    A 10 Hz o período tem 5 pontos, então são 5 bins e não 24."""
    te = onda.te
    criados = []
    base = te._WaveILC

    class _Espia(base):
        def __init__(self, *a, **k):
            criados.append(k.get('n_bins'))
            super().__init__(*a, **k)
    monkeypatch.setattr(te, '_WaveILC', _Espia)
    onda.roda(hz=hz, f_min=1.4, f_max=1.6, ciclos=2)
    assert criados == [quer or te._FMOD_ILC_BINS]


@pytest.mark.parametrize('hz,corta', [(1.0, True), (10.0, False)])
def test_corte_por_passo_se_cala_quando_a_medida_sai_de_fase(onda, hz, corta):
    """O corte por passo pergunta 'a força AGORA está fora da faixa?'. Com
    60 ms de transporte isso vale 22° a 1 Hz e 216° a 10 Hz: lá ele corta em
    fase aleatória e abre entalhes. A leitura instantânea fica fora da faixa
    do 3º ciclo em diante, nas duas frequências."""
    onda.lag_s = 0.060
    onda.bruto = lambda c: 1.75 if c > 3.0 else 1.5
    assert onda.roda(hz=hz, f_min=1.4, f_max=1.6, ciclos=8) == 'ok'
    depois = [s for t, s in onda.passos if 4.0 < t * hz < 7.9]
    zerados = sum(s == 0.0 for s in depois) / len(depois)
    if corta:
        assert zerados > 0.3 and onda.n.log.tem('passos cortados')
    else:
        assert zerados == 0.0


def test_guarda_de_excursao_sobrevive_ao_corte_calado(onda):
    """Com o corte por passo calado em alta frequência, o recuo por CICLO
    passa a ser o único guarda de excursão — e por isso não pode depender de
    ter havido corte. O gatilho é o extremo MEDIDO."""
    onda.lag_s = 0.060
    onda.bruto = lambda c: 1.5            # o corte por passo nunca dispara
    onda.deforma = lambda c, f: 1.5 + (f - 1.5) * (3.0 if c > 6 else 1.0)
    assert onda.roda(hz=10.0, f_min=1.35, f_max=1.65, ciclos=20) == 'ok'
    assert not onda.n.log.tem('passos cortados')
    assert onda.n.log.tem('Amplitude comandada recuada')
    amp = _amplitude_por_ciclo(onda)
    assert min(amp[8:]) < 0.85 * max(amp[4:7])


def test_recuo_de_amplitude_volta_a_subir(onda, monkeypatch):
    """`limit_scale` só descia: um ciclo ruim encolhia o ensaio INTEIRO de
    forma irreversível. E um ciclo em que a amplitude mudou não é aprendido."""
    from touch_pack.tactile_explorer import (
        _FMOD_LIMIT_RECOVER_CYCLES, _FMOD_LIMIT_RECOVER_STEP)
    te = onda.te
    monkeypatch.setattr(te._WaveILC, 'value', lambda s, p: 0.0)
    descartes, aprendidos = [], []
    d0, c0 = te._WaveILC.discard, te._WaveILC.commit

    def ciclo():
        return int(onda.fase(onda.rel.t))
    monkeypatch.setattr(te._WaveILC, 'discard', lambda s: (
        descartes.append(ciclo()), d0(s))[1])
    monkeypatch.setattr(te._WaveILC, 'commit', lambda s: (
        aprendidos.append(ciclo()), c0(s))[1])
    onda.bruto = lambda c: 1.5
    onda.deforma = lambda c, f: 1.5 + (f - 1.5) * (
        1.6 if 8 < c < 9 else 1.0)
    assert onda.roda(f_min=1.2, f_max=1.8, ciclos=40, hz=2.0) == 'ok'
    amp = _amplitude_por_ciclo(onda)
    cheia, vale = amp[7], min(amp[9:14])
    assert vale < 0.9 * cheia                         # recuou
    assert amp[-2] > vale * 1.10                      # e voltou a subir
    assert amp[-2] <= cheia * 1.02
    # Subir é MAIS LENTO que descer: no máximo um passo por N ciclos limpos.
    i = amp.index(vale)
    assert amp[i + _FMOD_LIMIT_RECOVER_CYCLES] <= (
        vale + 1.5 * _FMOD_LIMIT_RECOVER_STEP * cheia)
    assert 9 in descartes and 9 not in aprendidos     # o ciclo que recuou
    assert len(descartes) >= 2                        # e os que subiram
    assert not set(descartes) & set(aprendidos)


def _pre(hz=10.0, amp_n=0.30, k_nm=620.0, servoj_s=0.020,
         meas_rate_hz=400.0, has_raw=True):
    """Preflight de uma onda no silicone, com os valores da bancada."""
    from touch_pack.force_wave import _ForceProfile, fmod_preflight
    mean = 1.0
    prof = _ForceProfile('SINE', mean - amp_n, mean + amp_n, hz, 20)
    return fmod_preflight(
        prof, servoj_period_s=servoj_s, amp_m=amp_n / k_nm, k_nm=k_nm,
        use_curve=False, meas_rate_hz=meas_rate_hz, has_raw=has_raw,
        deadband_tcp_m=12e-6)


def _errs(findings):
    return [t for lvl, t in findings if lvl == 'error']


def test_10hz_passa_no_preflight_na_bancada_montada():
    """O contrato do ensaio: 10 Hz com t=20 ms, FA7155 a 400 Hz e o canal cru
    no ar é um ensaio VÁLIDO. Se este teste ficar vermelho, 10 Hz voltou a ser
    inalcançável — que era o estado de partida."""
    _amp_pre, findings = _pre()
    assert _errs(findings) == [], _errs(findings)
    assert _amp_pre == pytest.approx(1.143, rel=0.01)   # sinc² de 5 pontos


def test_10hz_e_recusado_com_o_servoj_no_padrao():
    """Com t=30 ms o teto é 6,67 Hz. Era o estado de partida da bancada, e a
    onda saía reamostrada em vez de recusada."""
    errs = _errs(_pre(servoj_s=0.030)[1])
    assert any('6.67 Hz' in e for e in errs), errs
    assert any('servoj_period_s:=' in e for e in errs), (
        'a recusa tem de dizer COMO configurar, não só que não dá')


def test_10hz_e_recusado_com_a_celula_lenta():
    """Uma onda de 10 Hz medida a 24 Hz não sai imprecisa, sai ALIASADA:
    são 2,4 amostras por período para medir fundamental e 3 harmônicos."""
    errs = _errs(_pre(meas_rate_hz=24.0)[1])
    assert any('24 Hz' in e and '80 Hz' in e for e in errs), errs
    # A FA7155 passa com folga.
    assert _errs(_pre(meas_rate_hz=400.0)[1]) == []


def test_10hz_e_recusado_sem_o_canal_cru():
    """Sem /load_cell/sample_net a força passa pelo One-Euro travado em 2 Hz,
    que a 10 Hz entrega 20 % da amplitude. Sem ILC não há correção de centro,
    fase nem forma — a onda sairia em malha aberta, e isso não é 'impreciso',
    é outro ensaio."""
    errs = _errs(_pre(has_raw=False)[1])
    assert any('sample_net' in e for e in errs), errs
    # Abaixo do portão do One-Euro o filtrado ainda serve.
    assert _errs(_pre(hz=1.0, has_raw=False, servoj_s=0.030)[1]) == []


def test_conselho_de_velocidade_e_executavel():
    """A mensagem calculava a frequência sugerida sobre o teto de AVISO
    (20 mm/s) e não sobre o de RECUSA (40): mandava baixar para metade do que
    já teria passado. O conselho tem de ser o MENOR recuo que funciona."""
    import re
    errs = _errs(_pre(amp_n=0.5)[1])       # ±0,5 N a 10 Hz no silicone
    vel = [e for e in errs if 'mm/s de pico' in e]
    assert vel, errs
    hz_ok = float(re.search(r'frequência para ≤([\d.]+) Hz', vel[0]).group(1))
    # Seguir o conselho tem de FUNCIONAR — e não sobrar recuo de graça.
    assert _errs(_pre(hz=hz_ok, amp_n=0.5)[1]) == []
    assert _errs(_pre(hz=hz_ok * 1.1, amp_n=0.5)[1]) != []
    # A outra saída também: manter 10 Hz estreitando a faixa.
    amp_ok = float(re.search(r'faixa para ±([\d.]+) N', vel[0]).group(1))
    assert _errs(_pre(amp_n=amp_ok)[1]) == []


def test_preflight_reporta_tudo_antes_de_recusar():
    """Um ensaio pode esbarrar em dois limites ao mesmo tempo. Descobrir um
    por run é exatamente o que o preflight existe para evitar."""
    errs = _errs(_pre(servoj_s=0.030, meas_rate_hz=24.0, has_raw=False)[1])
    assert len(errs) >= 3, errs


def test_teto_de_velocidade_e_o_que_limita_a_amplitude_a_10hz():
    """O limite REAL de amplitude a 10 Hz não é força nem firmware: é o teto
    de velocidade de pico. Vale a pena estar no teste porque é o número que o
    operador precisa saber antes de montar o ensaio."""
    import math
    from touch_pack.tactile_explorer import (
        _FMOD_V_PEAK_MAX_MMS, _FMOD_MIN_PTS_PER_CYCLE, _fmod_sampling_gain)
    amp_pre = 1.0 / _fmod_sampling_gain(_FMOD_MIN_PTS_PER_CYCLE)
    amp_m = _FMOD_V_PEAK_MAX_MMS * 1e-3 / (2 * math.pi * 10.0 * amp_pre)
    assert amp_m == pytest.approx(0.557e-3, rel=0.02)   # 0,56 mm de pico
    # No silicone medido em bancada isso são ±0,35 N de amplitude máxima.
    assert amp_m * 620.0 == pytest.approx(0.345, rel=0.02)
    # E é por isso que ±0,5 N a 10 Hz no silicone é RECUSADO: o curso que a
    # faixa pede (0,81 mm) passa do que o teto de velocidade autoriza.
    assert 0.5 / 620.0 > amp_m


def test_onda_micrometrica_avisa_sobre_a_banda_morta():
    """Na ponteira rígida ±0,5 N valem 18 µm de pico contra ~12 µm de banda
    morta de ServoJ: ~1,5 degrau por semiciclo. A onda sai — quadrada."""
    from touch_pack.constants import SERVOJ_DEADBAND_RAD, ARM_REACH_M
    from touch_pack.tactile_explorer import _FMOD_DEADBAND_MIN_RATIO
    deadband_m = SERVOJ_DEADBAND_RAD * ARM_REACH_M
    assert deadband_m == pytest.approx(12e-6, rel=0.05)
    amp_rigida_m = 0.5 / 28_000.0
    assert amp_rigida_m < _FMOD_DEADBAND_MIN_RATIO * deadband_m   # avisa
    amp_silicone_m = 0.35 / 620.0
    assert amp_silicone_m > _FMOD_DEADBAND_MIN_RATIO * deadband_m  # não avisa


def test_onda_usa_um_relogio_so_e_monotonico(onda):
    """Eram dois: um de parede para a fase e um monotônico para a grade de
    ticks. Um salto do relógio de parede (NTP) no meio da onda não muda
    nada."""
    assert onda.roda(ciclos=4) == 'ok'
    limpo, durou = list(onda.passos), onda.durou
    onda.passos.clear()
    gate = onda.n._pause_gate

    def _gate():
        onda.rel.parede = 3600.0 if onda.fase(onda.rel.t) > 1.5 else -50.0
        return gate()
    onda.n._pause_gate = _gate
    assert onda.roda(ciclos=4) == 'ok'
    assert onda.durou == pytest.approx(durou, abs=0.05)
    assert len(onda.passos) == pytest.approx(len(limpo), abs=2)


@pytest.mark.parametrize('v_max', [1.0, 150.0])
def test_degrau_na_onda_e_fatiado_pelos_tetos(onda, monkeypatch, v_max):
    """Um salto no setpoint não vira um passo único: o teto de velocidade
    (1 mm/s aqui) ou o de ΔF por passo (150 mm/s) o espalham."""
    from touch_pack.force_wave import _ForceProfile
    te = onda.te

    class _Salto(_ForceProfile):
        def setpoint_n(self, t):
            return self.mean_n + (self.amp_n * 8.0 if 0.5 < t < 0.6 else 0.0)
    monkeypatch.setattr(te, '_FMOD_V_MAX_MMS', v_max)
    monkeypatch.setattr(te._WaveILC, 'value', lambda s, p: 0.0)
    onda.roda(prof=_Salto('SINE', 1.0, 3.0, 1.0, 1))
    na_onda = [abs(s) for t, s in onda.passos if t < 1.0]
    dt = onda.prof.wave_dt(0.020)
    teto = min(v_max * 1e-3 * dt, te._FMOD_DF_STEP_MAX_N / onda.k_est)
    assert max(na_onda) == pytest.approx(teto, rel=1e-6)


@pytest.mark.parametrize('worker', ['_connect_robot_worker',
                                    '_robot_reconnect_worker'])
def test_driver_recebe_o_periodo_do_laco(worker, monkeypatch):
    """O `t=` do ServoJ tem de acompanhar o período do laço, na conexão E na
    reconexão: pontos a cada 20 ms com t=30 ms fazem o controlador interpolar
    noutra grade."""
    import threading
    import types
    import touch_pack.gui_robot as gr
    visto = []
    ev = threading.Event()

    def _drv(ip, dry_run, config):
        visto.append(config.servoj_period_s)
        ev.set()
        raise gr.CR10RealDriverError('basta')
    monkeypatch.setattr(gr, 'CR10RealDriver', _drv)
    log = types.SimpleNamespace(**{k: lambda *a: None for k in
                                   ('info', 'warn', 'error', 'warning')})
    g = types.SimpleNamespace(
        _servoj_period_s=0.020, _feedback_port=30004,
        _payload_cfg=lambda: {}, _robot_reconnecting=True, _stop_event=ev,
        _close_driver_quietly=lambda d: None,
        root=types.SimpleNamespace(after=lambda *a: None),
        get_logger=lambda: log)
    getattr(gr.RobotMixin, worker)(g, '127.0.0.1')
    assert visto == [0.020]


def _amplitude_por_ciclo(onda):
    """Pico a pico da penetração COMANDADA em cada ciclo da onda."""
    import numpy as np
    acc, por = 0.0, {}
    for t, s in onda.passos:
        acc += s
        por.setdefault(int(t * onda.prof.freq_hz), []).append(acc)
    return [float(np.ptp(por[c])) for c in range(onda.prof.cycles)]
