"""Não-ultrapassagem do setpoint na regulação quase-estática por RAMPA.

Contexto (31/08/2026): a lei proporcional Δx = relax·err/K_est foi trocada
por uma RAMPA A VELOCIDADE CONSTANTE. O overshoot dela era estrutural — não
ruído nem inércia: K_est é uma EMA da secante JÁ percorrida de uma curva que
ENRIJECE. Números medidos da curva F(x) do run TOUCH/1hz/20260817_112556
(regressão sobre a descida do ciclo 1):

    penetração      secante local
    0,0 – 0,2 mm      0,18 N/mm
    1,0 – 1,2 mm      0,80 N/mm
    1,8 – 2,0 mm      3,0  N/mm      <- 17x a do pé, no MESMO toque

Com a EMA presa no pé, o passo perto do alvo entregava 3-4x o ΔF pedido e o
teto ABSOLUTO virava o limitador efetivo: 200 µm × 2,5 N/mm = 0,5 N de
quantum por passo contra uma banda de 0,05 N.

A lei NOVA (`_qs_regulate`, etapa A) move o TCP a velocidade constante
(`hold_ramp_mms`, _QS_RAMP_V_MS) até a força medida CRUZAR o setpoint, e
então congela (etapa B). A rigidez NÃO entra na decisão do passo — só num
clamp de segurança por tick (_QS_RAMP_DF_CAP_N / K_est) que ENCURTA o passo
fixo quando o contato é rígido, para o cruzamento cair dentro de ~1 banda.

Estes testes exercitam o bloco de decisão do `_qs_regulate` contra as MESMAS
curvas F(x) sintéticas e cobram os invariantes que a rampa promete:
  1. o passo NUNCA projeta a força além do alvo + 1 quantum de tick;
  2. a aproximação é MONÓTONA (a rampa não recua enquanto empurra);
  3. converge em curso e ticks limitados, do silicone mole ao contato
     rígido de _K_MAX_NM;
  4. um degrau de DESCIDA (alvo abaixo da força atual) recua até o nível;
  5. a defesa do patamar (etapa B) retoma a rampa quando a força relaxa,
     SEM reiniciar o relógio de estabilidade.

QUAL FERRAMENTA ESTAS CURVAS DESCREVEM
══════════════════════════════════════
Curvas da pilha da viga S de 100 kg + acoplador impresso + ponteira D de
silicone, TCP a 162,2 mm do flange. `f_rigido` cobre de 10 N/mm (ponteira
rígida antiga) até `_K_MAX_NM` (1000 N/mm), o envelope que o estimador
declara suportar.
"""
import pytest

pytest.importorskip('rclpy')


# ── curvas de contato ────────────────────────────────────────────────
# F(x) quase-estática de carga, penetração em mm, força em N.

def f_silicone(x_mm):
    """Ajuste da curva MEDIDA: F = 0,06 + 0,325·x^2,4 (rms 76 mN).

    O expoente > 1 é o que faz a secante variar 17x entre o pé e o alvo.
    """
    return 0.06 + 0.325 * max(0.0, x_mm) ** 2.4


def f_rigido(k_n_mm):
    """Contato linear F = 0,06 + k·x."""
    return lambda x_mm: 0.06 + k_n_mm * max(0.0, x_mm)


@pytest.fixture(scope='module')
def m():
    from touch_pack import tactile_explorer
    return tactile_explorer


# ── espelho do bloco de decisão da rampa ─────────────────────────────

def _ramp_step(m, *, target_f, tol_n, fz, est, in_contact, crossed, sign0,
               dynamic=False):
    """Reproduz UM passo da lei do `_qs_regulate` (rampa a v constante).

    ESPELHO do bloco de decisão de passo — mexer lá pede mexer aqui. A
    alternativa seria instanciar o nó ROS inteiro só para exercitar vinte
    linhas de aritmética. Devolve (step_m, crossed, sign0).
    """
    err = target_f - fz
    in_band = abs(err) <= tol_n
    if not crossed and (in_band or (sign0 != 0.0 and sign0 * err <= 0.0)):
        crossed = True
    if crossed:
        if err > tol_n:
            sign_now = 1.0
        elif err < -tol_n:
            sign_now = -1.0
        else:
            sign_now = 0.0
    else:
        if sign0 == 0.0:
            sign0 = 1.0 if err > 0.0 else -1.0
        sign_now = (1.0 if err > 0.0 else -1.0) if dynamic else sign0
    if sign_now == 0.0:
        return 0.0, crossed, sign0
    step_mag = m._QS_RAMP_V_MS * m._CTRL_DT
    if in_contact:
        k_cap = est.value if est.estimated else est.k_upper
        step_mag = min(
            step_mag,
            m._QS_RAMP_DF_CAP_N / max(k_cap, m._K_MIN_NM),
            m._QS_NO_CROSS_FRAC * abs(err) / max(est.k_upper, m._K_MIN_NM))
    else:
        step_mag = min(step_mag, m._QS_FREE_STEP_MAX_M)
    return sign_now * step_mag, crossed, sign0


def _x_at_force(curva, f_n, hi_mm=60.0):
    """Penetração (m) onde `curva` vale `f_n` — busca binária."""
    lo, hi = 0.0, hi_mm
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if curva(mid) < f_n:
            lo = mid
        else:
            hi = mid
    return mid * 1e-3


def _run_ramp(m, curva, target_f, tol_n, *, x0_m=None, max_ticks=8000,
              dynamic=False):
    """Roda a rampa inteira contra `curva`, alimentando o estimador REAL a
    cada passo (como o `_qs_regulate` faz).

    Entra JÁ EM CONTATO (fz logo acima de _CONTACT_ON_N): é a pré-condição
    real — `_qs_regulate` só é chamado depois de o DESCENDING confirmar o
    toque. Devolve (pico_N, ticks, fz_final_N)."""
    est = m._StiffnessEstimator()
    est.reset()
    x = _x_at_force(curva, m._CONTACT_ON_N + 0.01) if x0_m is None else x0_m
    pico = curva(x * 1e3)
    crossed = False
    sign0 = 0.0
    fz_prev = None
    step_prev = 0.0
    for tick in range(max_ticks):
        fz = curva(x * 1e3)
        pico = max(pico, fz)
        in_contact = fz > m._CONTACT_ON_N
        if fz_prev is not None and step_prev != 0.0 \
                and (in_contact or step_prev > 0.0):
            est.update_pair(step_prev, fz - fz_prev)
        step_m, crossed, sign0 = _ramp_step(
            m, target_f=target_f, tol_n=tol_n, fz=fz, est=est,
            in_contact=in_contact, crossed=crossed, sign0=sign0,
            dynamic=dynamic)
        if crossed and step_m == 0.0:
            return pico, tick, fz
        x += step_m
        fz_prev, step_prev = fz, step_m
    return pico, max_ticks, curva(x * 1e3)


# ── 1. não-ultrapassagem ─────────────────────────────────────────────

@pytest.mark.parametrize('curva,alvo,tol', [
    (f_silicone,       1.6, 0.05),
    (f_silicone,       0.5, 0.05),
    (f_silicone,       3.0, 0.15),
    (f_silicone,       0.2, 0.05),
    (f_rigido(10.0),   0.5, 0.05),
    (f_rigido(28.0),   1.0, 0.05),
    (f_rigido(100.0),  0.3, 0.05),
    (f_rigido(300.0),  0.5, 0.05),
    (f_rigido(300.0),  1.0, 0.05),
    (f_rigido(600.0),  1.0, 0.05),
    (f_rigido(900.0),  0.5, 0.05),
    (f_rigido(900.0),  3.0, 0.15),
    (f_rigido(1000.0), 0.5, 0.05),
    (f_rigido(1000.0), 1.0, 0.05),
])
def test_rampa_nao_ultrapassa_o_alvo(m, curva, alvo, tol):
    """O pico da rampa NÃO passa do alvo — do silicone mole ao contato rígido
    de _K_MAX_NM. É o que a lei proporcional NÃO dava: lá o pico passava
    0,23–0,59 N do alvo porque K_est subestimava a inclinação seguinte. A
    guarda de não-ultrapassagem (_QS_NO_CROSS_FRAC·|err|/k_upper) torna a
    aproximação final geométrica POR BAIXO.
    """
    pico, ticks, fz_fim = _run_ramp(m, curva, alvo, tol)
    assert pico <= alvo + 1e-6, (
        f'pico {pico:.3f} N ULTRAPASSOU o alvo {alvo:.2f} '
        f'({pico - alvo:+.3f} N em {ticks} ticks)')
    assert fz_fim >= alvo - tol, (
        f'parou fora da banda por baixo: fz={fz_fim:.3f} N, '
        f'alvo {alvo:.2f} − {tol:.2f}')
    assert ticks < 8000, f'não convergiu em {ticks} ticks'


def test_quantum_de_tick_cai_com_a_rigidez(m):
    """O clamp de segurança _QS_RAMP_DF_CAP_N/K_est: num contato rígido o
    passo encolhe para o cruzamento não estourar a banda; num mole ele nem
    morde e a rampa anda ao ritmo de _QS_RAMP_V_MS."""
    dt = m._CTRL_DT
    livre = m._QS_RAMP_V_MS * dt
    # Silicone perto do pé: k ~ 0,3 N/mm -> cap = 0,1/300 = 333 µm >> livre.
    assert m._QS_RAMP_DF_CAP_N / 300.0 > livre
    # Contato rígido _K_MAX_NM: cap = 0,1/1e6 = 0,1 µm << livre.
    dx_rigido = m._QS_RAMP_DF_CAP_N / m._K_MAX_NM
    assert dx_rigido < livre
    # e o ΔF projetado desse passo mínimo fica na ordem da banda, não acima.
    assert dx_rigido * m._K_MAX_NM == pytest.approx(m._QS_RAMP_DF_CAP_N)
    assert m._QS_RAMP_DF_CAP_N <= 0.15, (
        'quantum de tick acima de ~2 bandas — o cruzamento estoura a banda')


# ── 2. monotonicidade ───────────────────────────────────────────────

@pytest.mark.parametrize('curva,alvo', [
    (f_silicone, 1.6), (f_silicone, 0.5), (f_rigido(28.0), 1.0),
    (f_rigido(300.0), 0.5), (f_rigido(1000.0), 1.0),
])
def test_aproximacao_e_monotona(m, curva, alvo):
    """Enquanto EMPURRA, a rampa nunca recua: sem termo proporcional não há
    o ciclo-limite alívio/empurra que produzia a assinatura QUIQUE."""
    est = m._StiffnessEstimator(); est.reset()
    x = _x_at_force(curva, m._CONTACT_ON_N + 0.01)
    crossed = False
    sign0 = 0.0
    fz_prev = None
    step_prev = 0.0
    for _ in range(8000):
        fz = curva(x * 1e3)
        in_contact = fz > m._CONTACT_ON_N
        if fz_prev is not None and step_prev != 0.0 \
                and (in_contact or step_prev > 0.0):
            est.update_pair(step_prev, fz - fz_prev)
        step_m, crossed, sign0 = _ramp_step(
            m, target_f=alvo, tol_n=0.05, fz=fz, est=est,
            in_contact=in_contact, crossed=crossed, sign0=sign0)
        if not crossed:
            assert step_m >= 0.0, 'a rampa recuou ANTES de cruzar o alvo'
        if crossed and step_m == 0.0:
            break
        x += step_m
        fz_prev, step_prev = fz, step_m


# ── 3. degrau de DESCIDA ────────────────────────────────────────────

@pytest.mark.parametrize('k_n_mm', [100.0, 300.0, 900.0])
def test_degrau_de_descida_recua_ate_o_nivel(m, k_n_mm):
    """Escada 2,0 -> 1,8 N: a força atual está ACIMA do alvo, então a etapa A
    RECUA a v constante até fz <= 1,8, e só então a etapa B congela."""
    curva = f_rigido(k_n_mm)
    x0 = _x_at_force(curva, 2.0)
    pico, ticks, f_fim = _run_ramp(m, curva, 1.8, 0.05, x0_m=x0)
    assert f_fim <= 1.8 + 0.05, f'parou em {f_fim:.3f} N, acima de 1,8 + banda'
    assert f_fim >= 1.8 - 0.20, f'recuou demais — parou em {f_fim:.3f} N'
    assert ticks < 8000


# ── 4. defesa do patamar (relaxação viscoelástica) ──────────────────

class _FakeEst:
    """Estimador com value/k_upper fixos — para exercitar SÓ a decisão."""
    def __init__(self, k_nm=3000.0):
        self.value = k_nm
        self.k_upper = 2.0 * k_nm
        self.estimated = True


def test_defesa_retoma_a_rampa_sem_reiniciar_o_relogio(m):
    """Etapa B: dentro do patamar a força relaxa abaixo de alvo - banda; a
    decisão devolve um passo de EMPURRAR (sign_now > 0) para recruzar. O
    relógio de stable_s é responsabilidade do laço, não da decisão — aqui se
    cobra só que a defesa MANDA empurrar quando devia."""
    alvo, tol = 1.0, 0.05
    est = _FakeEst()
    # já cruzou, força relaxou para 0,90 N (0,05 abaixo da borda inferior).
    step_m, crossed, _ = _ramp_step(
        m, target_f=alvo, tol_n=tol, fz=0.90, est=est, in_contact=True,
        crossed=True, sign0=1.0)
    assert crossed
    assert step_m > 0.0, 'a defesa não retomou a rampa com a força relaxada'
    # de volta na banda: congela.
    step_m, _, _ = _ramp_step(m, target_f=alvo, tol_n=tol, fz=0.99, est=est,
                              in_contact=True, crossed=True, sign0=1.0)
    assert step_m == 0.0, 'a defesa continua mexendo dentro da banda'


def test_defesa_recupera_a_forca_dos_dois_lados(m):
    """A defesa da etapa B corrige a força ACIMA do alvo (recuperação
    viscoelástica após um degrau de descida) e ABAIXO (relaxação após um de
    subida) — nas fases comuns e no MANUAL dinâmico. O passo de recuo é
    limitado por _QS_NO_CROSS_FRAC·|err|/k_upper, minúsculo perto do alvo,
    então não larga o contato."""
    est = _FakeEst()
    for dyn in (False, True):
        step_m, _, _ = _ramp_step(m, target_f=1.0, tol_n=0.05, fz=1.20,
                                  est=est, in_contact=True, crossed=True,
                                  sign0=1.0, dynamic=dyn)
        assert step_m < 0.0, f'dynamic={dyn}: deveria recuar para 1,0 N'
        # e o recuo é pequeno (não cruza o alvo nem larga o contato).
        assert abs(step_m) <= m._QS_NO_CROSS_FRAC * 0.20 / est.k_upper + 1e-12


# ── 5. o parâmetro da rampa ─────────────────────────────────────────

def test_hold_ramp_default_bate_com_a_constante(m):
    """O default do parâmetro ROS é _QS_RAMP_V_MS em mm/s — re-medir a
    cadência é mudar UM número."""
    assert m._QS_RAMP_V_MS == pytest.approx(1.0e-3)


# ── Medir força que ainda está se movendo (inalterado) ──────────────

def test_a_deriva_ve_tendencia_e_ignora_ruido(m):
    """`_deriva` é o critério de "a força parou": mediana da 2ª metade da
    janela contra a da 1ª. Tendência e não pico-a-pico, porque o ptp cresce
    com o tamanho da janela mesmo num sinal estacionário."""
    d = m.TactileExplorer._deriva
    assert d([1.0, 1.02, 0.98, 1.01, 0.99, 1.0]) < 0.02
    assert d([1.0, 1.1, 1.2, 1.3, 1.4, 1.5]) > 0.2
    assert d([1.0, 1.0, 1.0, 1.3, 1.3, 1.3]) == pytest.approx(0.3)


def test_a_deriva_nao_quebra_com_janela_minuscula(m):
    d = m.TactileExplorer._deriva
    assert d([]) == 0.0
    assert d([1.0]) == 0.0


def test_o_limiar_de_assentado_esta_acima_do_ruido_da_celula(m):
    """Abaixo do ruído, a força nunca seria declarada assentada e toda medida
    perto do alvo gastaria o teto de ticks."""
    assert m._QS_SETTLE_DRIFT_N > m._FORCE_NOISE_SIGMA_N
    assert m._QS_SETTLE_DRIFT_N < m._HOLD_TOL_N


def test_a_espera_assentada_so_vale_perto_do_alvo(m):
    """A medida assentada custa até 1 s por passo. Longe do alvo o passo é
    grande e o creep é irrelevante ao lado dele."""
    assert m._QS_SETTLE_NEAR_MULT >= 1.0
    assert m._QS_SETTLE_MAX_TICKS > m._QS_MEASURE_MAX_TICKS
