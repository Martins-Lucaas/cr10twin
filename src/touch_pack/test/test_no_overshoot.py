"""Não-ultrapassagem do setpoint na regulação quase-estática.

O overshoot dos runs de 17/08/2026 não era ruído nem inércia: era o passo
Δx = relax·err/K_est dimensionado com uma K_est que mede o trecho JÁ
percorrido de uma curva que ENRIJECE. Números medidos da curva F(x) do run
TOUCH/1hz/20260817_112556 (regressão sobre a descida do ciclo 1):

    penetração      secante local
    0,0 – 0,2 mm      0,18 N/mm
    1,0 – 1,2 mm      0,80 N/mm
    1,8 – 2,0 mm      3,0  N/mm      <- 17x a do pé, no MESMO toque

Com a EMA presa no pé, o passo perto do alvo entregava 3-4x o ΔF pedido e o
teto ABSOLUTO (hold_dx_max_um = 200 µm na GUI daqueles runs) virava o
limitador efetivo: 200 µm × 2,5 N/mm = 0,5 N de quantum por passo contra uma
banda de 0,05 N. Os 5 ciclos daquele run fecharam com +0,23 a +0,59 N.

Estes testes exercitam a lei de passo do `_qs_regulate` contra curvas F(x)
sintéticas com a MESMA não-linearidade medida, e cobram o invariante que
resolve o problema: **o passo de empurrar nunca projeta a força além da borda
superior da banda, nem no pior caso de rigidez.**

QUAL FERRAMENTA ESTAS CURVAS DESCREVEM
══════════════════════════════════════
As curvas medidas acima são da pilha da viga S de 100 kg + acoplador impresso
+ ponteira D de silicone, TCP a 162,2 mm do flange — que voltou a ser a
bancada em 27/08/2026 (entre 18/08 e 26/08 o padrão foi a FA7155 de 6 eixos,
TCP a 67,7 mm, curta e maciça em alumínio e portanto muito mais rígida).

As duas convivem no repo, então a parametrização cobre as duas faixas: o
silicone pelo lado mole e `f_rigido` até `_K_MAX_NM` (1000 N/mm), o envelope
que o próprio estimador declara suportar. A faixa de 300 N/mm para cima
falhava INTEIRA antes de duas correções (cota do resultado nulo em `k_upper`
e aceitação do par que atravessa a fronteira do contato): a 900 N/mm um alvo
de 0,5 N fechava com pico de 5,0 N.
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
    """Contato linear.

    10 a 28 N/mm era a ponteira rígida da ferramenta ANTIGA (viga S de
    100 kg + acoplador impresso + ponteira D, TCP a 162,2 mm). A célula
    padrão de hoje — `end_effector:=touch_tool`, pilha FA7155 de 6 eixos com
    TCP a 67,7 mm — é 94,5 mm mais curta e maciça em alumínio: a mesma força
    trabalha um braço de alavanca muito menor, e a rigidez do conjunto sobe.
    Por isso a faixa exercitada aqui vai até `_K_MAX_NM` (1000 N/mm), que é o
    envelope que o próprio estimador declara suportar.
    """
    return lambda x_mm: 0.06 + k_n_mm * max(0.0, x_mm)


def _passo(explorer_mod, est, *, target_f, fz, boost=1.0,
           step_up_prev=None, free_ticks=0, dx_max_m=200e-6,
           df_hard_n=1.0):
    """Reproduz UM passo da lei do `_qs_regulate` com o estimador dado.

    ESPELHO do bloco de decisão de passo do `_qs_regulate` — mexer lá pede
    mexer aqui. A alternativa seria instanciar o nó ROS inteiro só para
    exercitar dez linhas de aritmética.
    """
    m = explorer_mod
    err = target_f - fz
    in_contact = fz > m._CONTACT_ON_N
    k = est.value
    k_push = est.k_upper
    head_n = target_f - fz          # mira no ALVO, não na borda da banda
    ramp_m = (m._QS_DX_FLOOR_M if step_up_prev is None
              else m._QS_STEP_GROWTH * step_up_prev)
    if not in_contact:
        step_m = min(m._QS_FREE_STEP_M * boost, m._QS_FREE_STEP_MAX_M)
        free_ticks += 1
        if free_ticks == m._QS_FREE_RESET_TICKS:
            step_up_prev, ramp_m = None, m._QS_DX_FLOOR_M
        if head_n > 0.0:
            step_m = min(step_m, m._QS_NO_CROSS_FRAC * head_n / k_push, ramp_m)
        step_up_prev = max(step_m, m._QS_DX_FLOOR_M)
    elif err > 0.0:
        free_ticks = 0
        step_m = m._QS_RELAX * err * boost / k
        df_step_n = m.TactileExplorer._qs_df_step_n(df_hard_n, target_f)
        step_m = min(step_m, m._QS_NO_CROSS_FRAC * head_n / k_push)
        step_m = min(step_m, df_step_n / k_push)
        step_m = min(step_m, ramp_m)
        step_m = max(step_m, 0.0)
        if not est.estimated:
            step_m = min(step_m, min(m._QS_DX_PROBE_M * boost,
                                     m._QS_DX_PROBE_MAX_M))
        step_up_prev = max(step_m, m._QS_DX_FLOOR_M)
    else:
        free_ticks = 0
        hard_cap = m.TactileExplorer._qs_df_step_n(df_hard_n, target_f) / k_push
        step_m = m._QS_RELAX * err * boost / k_push
        step_m = max(-hard_cap, min(hard_cap, step_m))
        step_m = max(step_m, -(fz - m._QS_RELIEF_FLOOR_N) / k_push)
    step_m = max(-dx_max_m, min(dx_max_m, step_m))
    return step_m, step_up_prev, free_ticks


def _descer(explorer_mod, curva, target_f, tol_n, *,
            dx_max_m=200e-6, df_hard_n=1.0, max_ticks=600):
    """Roda a descida quase-estática inteira. Devolve (pico_N, ticks)."""
    m = explorer_mod
    est = m._StiffnessEstimator()
    est.reset()
    x = 0.0
    pico = curva(0.0)
    boost, step_up_prev, free_ticks = 1.0, None, 0
    fz_prev, step_prev = None, 0.0
    for tick in range(max_ticks):
        fz = curva(x * 1e3)
        pico = max(pico, fz)
        if fz_prev is not None and step_prev != 0.0:
            # ESPELHO do gate do `_qs_regulate`: contato nas duas leituras,
            # ou passo que atravessou a fronteira empurrando.
            if fz > m._CONTACT_ON_N or step_prev > 0.0:
                est.update_pair(step_prev, fz - fz_prev)
            boost = (min(boost * 1.5, m._QS_BOOST_MAX)
                     if abs(fz - fz_prev) < m._QS_DF_DEAD_N else 1.0)
        if abs(target_f - fz) <= tol_n:
            return pico, tick
        step_m, step_up_prev, free_ticks = _passo(
            m, est, target_f=target_f, fz=fz, boost=boost,
            step_up_prev=step_up_prev, free_ticks=free_ticks,
            dx_max_m=dx_max_m, df_hard_n=df_hard_n)
        x += step_m
        fz_prev, step_prev = fz, step_m
    return pico, max_ticks


@pytest.fixture(scope='module')
def m():
    from touch_pack import tactile_explorer
    return tactile_explorer


# ── a cota superior de rigidez ───────────────────────────────────────

def test_k_upper_fica_acima_da_ema(m):
    """k_push tem de ser CONSERVADOR: numa curva convexa a inclinação de
    frente é maior que a média já percorrida."""
    est = m._StiffnessEstimator()
    est.reset()
    # Cada trecho tem de cruzar _K_PAIR_MIN_DF_N sozinho, senão o
    # acumulador os funde numa secante única e a distinção se perde.
    est.update_pair(4.0e-4, 180.0 * 4.0e-4)        # 0,18 N/mm (pé)
    est.update_pair(2.0e-5, 3000.0 * 2.0e-5)       # 3,0 N/mm (perto do alvo)
    assert est.k_last == pytest.approx(3000.0, rel=0.01)
    # SUBIDA: o `_absorb` é assimétrico e sobe NA HORA (α=1), então aqui a
    # própria EMA já vale a secante do último trecho — o atraso de ~4 pares
    # que a EMA simétrica tinha era metade do overshoot.
    assert est.value == pytest.approx(3000.0, rel=0.01), (
        'premissa: numa SUBIDA o _absorb acompanha a secante imediatamente'
    )
    assert est.k_upper > est.value, (
        'k_upper perdeu a margem — o passo volta a ser dimensionado pela '
        'inclinação já percorrida e o overshoot volta com ele')
    assert est.k_upper >= 3000.0, (
        f'k_upper={est.k_upper:.0f} N/m ignora a secante de 3,0 N/mm '
        'medida no último trecho')
    # DESCIDA: é aqui que `k_last` ainda manda sozinho. O _absorb desce por
    # EMA (robustez a um par ruidoso), então a EMA fica ACIMA da secante nova
    # e é o k_last que continua fixando a cota — sem ele, k_upper seguiria a
    # EMA para baixo e o teto de passo afrouxaria cedo demais.
    est.update_pair(4.0e-4, 180.0 * 4.0e-4)        # volta a 0,18 N/mm
    assert est.k_last == pytest.approx(180.0, rel=0.01)
    assert est.value > 180.0, (
        'premissa: numa DESCIDA o _absorb amortece por EMA (α=0,25)')
    assert est.k_upper == pytest.approx(est.value * 2.0, rel=0.01)


def test_k_upper_sem_medida_nenhuma_nao_explode(m):
    """Antes do 1º par aceito só existe o default; quem protege é a rampa."""
    est = m._StiffnessEstimator()
    est.reset()
    assert est.k_last is None
    assert m._K_MIN_NM <= est.k_upper <= m._K_MAX_NM


def test_par_de_1um_ainda_ensina_o_estimador(m):
    """A rampa comanda passos de 1 a 5 µm perto do alvo. Se o estimador
    descartar esses pares (o gate antigo era 1,5 µm) ele nunca latcha em
    contato rígido e k_push fica no default, 20x mole demais."""
    est = m._StiffnessEstimator()
    est.reset()
    for _ in range(6):
        est.update_pair(1.0e-6, 28_000.0 * 1.0e-6)   # 28 N/mm, 0,028 N/passo
    assert est.estimated, 'passos de 1 µm continuam sendo descartados'
    assert est.value == pytest.approx(28_000.0, rel=0.10)


# ── o invariante ─────────────────────────────────────────────────────

def test_passo_de_empurrar_nunca_projeta_alem_do_alvo(m):
    """Invariante central: ΔF do passo, calculado com a cota SUPERIOR de
    rigidez, cabe dentro da folga até o ALVO.

    A mira era a BORDA DE CIMA DA BANDA até 27/08/2026, e com ela parar uma
    tolerância inteira acima do setpoint era o comportamento CORRETO da lei —
    overshoot por especificação, não por falha. Contra um alvo de 0,5 N com a
    banda de 4σ (0,092 N) isso valia +18 % de força na amostra.

    É este teste que trava a mira: reverter `head_n` para `(alvo+tol) − fz`
    faz o `fz=1.59` abaixo passar, porque lá a folga até a borda é 12x a
    folga até o alvo.
    """
    est = m._StiffnessEstimator()
    est.reset()
    est.update_pair(5.0e-5, 2.5 * 5e-5 * 1e3)        # 2,5 N/mm medidos
    alvo = 1.6
    for fz in (0.10, 0.50, 1.00, 1.40, 1.55, 1.59):
        step_m, _, _ = _passo(m, est, target_f=alvo, fz=fz,
                              step_up_prev=1.0e-3)   # rampa já larga
        df_pior = step_m * est.k_upper
        assert fz + df_pior <= alvo + 1e-9, (
            f'fz={fz:.2f} N: passo de {step_m*1e6:.1f} µm projeta '
            f'{fz + df_pior:.3f} N, acima do alvo {alvo:.2f} N')


def test_rampa_impede_o_salto_do_pe_da_curva_para_o_teto(m):
    """O salto de 8 µm para 200 µm num tick é onde o overshoot nascia:
    200 µm × 2,5 N/mm = 0,5 N contra uma banda de 0,05 N."""
    est = m._StiffnessEstimator()
    est.reset()
    step_m, _, _ = _passo(m, est, target_f=1.6, fz=0.10,
                          step_up_prev=8.0e-6)
    assert step_m <= m._QS_STEP_GROWTH * 8.0e-6 + 1e-12, (
        f'passo de {step_m*1e6:.1f} µm cresceu mais que '
        f'{m._QS_STEP_GROWTH:.0f}x o anterior (8 µm)')


def test_alivio_usa_a_cota_superior_e_nao_larga_o_contato(m):
    """Recuar com a EMA (mole) sobre um contato rígido tirava a ponteira do
    contato; o passo livre seguinte voltava batendo — assinatura QUIQUE."""
    est = m._StiffnessEstimator()
    est.reset()
    est.update_pair(1.0e-5, 28_000.0 * 1.0e-5)       # 28 N/mm
    step_m, _, _ = _passo(m, est, target_f=1.0, fz=1.6)
    assert step_m < 0.0, 'acima da banda o passo tem de recuar'
    recuo_pedido = 0.6 / 28_000.0                     # ΔF/K real
    assert abs(step_m) <= recuo_pedido * 1.01, (
        f'recuo de {abs(step_m)*1e6:.1f} µm contra {recuo_pedido*1e6:.1f} µm '
        'necessários — recuo desse tamanho larga o contato')


# ── a descida inteira, contra as curvas medidas ──────────────────────

# `teto` é o EXCESSO máximo tolerado sobre o alvo, e desde a mira no alvo
# (27/08/2026) ele é ZERO em toda linha menos uma: o pico da descida fica
# ABAIXO do setpoint, não dentro de uma banda acima dele. Antes o teto era a
# própria tolerância em toda linha, porque a lei mirava na borda.
@pytest.mark.parametrize('curva,alvo,tol,teto', [
    (f_silicone,        1.6, 0.05, 0.0),    # o caso dos runs TOUCH
    (f_silicone,        0.5, 0.05, 0.0),
    (f_silicone,        3.0, 0.15, 0.0),
    # ÚNICA linha com excesso, e ele NÃO vem da não-ultrapassagem: vem da
    # RAMPA. No pé mole do silicone o estimador só latcha em x≈0,42 mm, e
    # três ticks de rampa depois (24→72→216 µm) o passo de 216 µm cai num
    # trecho onde k_push (193 N/m) subestima a inclinação real (~390 N/m) —
    # o ×2 de _QS_K_PUSH_MARGIN não cobre o quanto a curva enrijece DENTRO
    # de um passo desse tamanho. É a hipótese do ×2 falhando, e o remédio é
    # trocar a cota heurística pela curva F(x) medida (_ContactCurve), não
    # mexer na mira. Os 6 mN residuais ficam em 1/4 do σ da célula
    # (FORCE_NOISE_SIGMA_N = 23 mN), então não há passo que os resolva sem
    # o sensor enxergá-los primeiro. Era +6,2 mN com a mira antiga.
    (f_silicone,        0.2, 0.05, 0.007),
    (f_rigido(10.0),    0.5, 0.05, 0.0),    # o caso dos runs MANUAL
    (f_rigido(10.0),    0.2, 0.05, 0.0),
    (f_rigido(28.0),    1.0, 0.05, 0.0),
    # ── A CÉLULA PADRÃO (end_effector:=touch_tool) ────────────────────
    # Tudo abaixo é a pilha FA7155 + ponteira F. Antes da cota do resultado
    # nulo em `k_upper` e do par de cruzamento, TODA linha de 300 N/mm para
    # cima falhava: o alívio dimensionado com o default de 40 N/mm largava o
    # contato, o estimador nunca latchava e a regulação entrava em ciclo
    # QUIQUE — a 900 N/mm um alvo de 0,5 N fechava com pico de 5,0 N.
    (f_rigido(100.0),   0.3, 0.05, 0.0),
    (f_rigido(300.0),   0.5, 0.05, 0.0),
    (f_rigido(300.0),   1.0, 0.05, 0.0),
    (f_rigido(600.0),   0.3, 0.05, 0.0),
    (f_rigido(600.0),   1.0, 0.05, 0.0),
    (f_rigido(900.0),   0.5, 0.05, 0.0),
    (f_rigido(900.0),   3.0, 0.15, 0.0),
    (f_rigido(1000.0),  0.5, 0.05, 0.0),    # _K_MAX_NM: o teto declarado
    (f_rigido(1000.0),  1.0, 0.05, 0.0),
    # Alvo baixo CONTRA contato muito rígido era o canto que mais passava da
    # banda (+0,381 N no início desta série, +21 mN depois da cota do
    # resultado nulo). Com a mira no alvo ele fecha 15 mN ABAIXO do
    # setpoint: aqui quem mordia era a não-ultrapassagem, e mirar 50 mN mais
    # alto era exatamente o que sobrava de folga para o passo de sonda gastar.
    (f_rigido(300.0),   0.2, 0.05, 0.0),
])
def test_descida_nao_ultrapassa_o_alvo(m, curva, alvo, tol, teto):
    """Com os MESMOS parâmetros da GUI daqueles runs (hold_dx_max_um=200,
    hold_df_max_n=1,0) o pico da descida fica ABAIXO do alvo.

    `teto=0.0` é a forma forte do invariante e a razão de a mira ter mudado:
    não basta o pico caber na banda, ele não pode passar do setpoint. Uma
    única linha (o pé mole do silicone) tem teto não-nulo, e o comentário
    dela nomeia a causa — que não é a mira.
    """
    pico, ticks = _descer(m, curva, alvo, tol)
    assert pico - alvo <= teto, (
        f'pico {pico:.3f} N contra alvo {alvo:.2f} ± {tol:.2f} '
        f'({pico - alvo:+.3f} N em {ticks} passos)')
    assert ticks < 400, f'convergiu em {ticks} passos — descida lenta demais'


@pytest.mark.parametrize('curva,alvo,tol', [
    (f_silicone,       1.6, 0.05),
    (f_silicone,       0.5, 0.05),
    (f_rigido(900.0),  0.5, 0.05),
    (f_rigido(1000.0), 1.0, 0.05),
])
def test_teto_absoluto_da_gui_nao_manda_mais_sozinho(m, curva, alvo, tol):
    """hold_dx_max_um vinha da GUI em 200 µm e era o ÚNICO limitador ativo
    perto do alvo. O que se cobra é que ele não seja mais quem segura o pico:
    para QUALQUER teto — 20× frouxo ou 4× apertado — o pico fica ABAIXO do
    alvo, porque quem morde antes é a não-ultrapassagem por k_push.

    NÃO se cobra que o pico seja o mesmo nos três tetos. Com a cota do
    resultado nulo em `k_upper`, um contato mole passa a ser reconhecido como
    mole em poucos ticks e o teto frouxo deixa a descida convergir mais
    rápido. O pico sobe junto e continua abaixo do setpoint; essa diferença é
    velocidade de convergência, não risco. A versão antiga do teste lia o
    spread como falha e escondia que o ganho estava do lado certo.
    """
    picos = [_descer(m, curva, alvo, tol, dx_max_m=dx)[0]
             for dx in (1.0e-3, 200e-6, 50e-6)]
    for dx, pico in zip((1.0e-3, 200e-6, 50e-6), picos):
        assert pico <= alvo, (
            f'com teto absoluto de {dx*1e6:.0f} µm o pico foi {pico:.3f} N, '
            f'acima do alvo {alvo:.2f} N — é o teto que está segurando, não '
            'a guarda de não-ultrapassagem')


# ── Medir força que ainda está se movendo ────────────────────────────
def test_a_deriva_ve_tendencia_e_ignora_ruido(m):
    """`_deriva` é o critério de "a força parou": mediana da 2ª metade da
    janela contra a da 1ª.

    Tendência e não pico-a-pico, porque o ptp cresce com o tamanho da janela
    mesmo num sinal perfeitamente estacionário — uma janela longa e quieta
    seria recusada por ser longa. É o mesmo critério do tare do
    force_receiver (_window_drift) e pelo mesmo motivo.
    """
    d = m.TactileExplorer._deriva
    # Ruído simétrico em torno de um valor fixo: sem tendência.
    assert d([1.0, 1.02, 0.98, 1.01, 0.99, 1.0]) < 0.02
    # Rampa: as duas metades discordam, e a deriva é da ordem da rampa.
    assert d([1.0, 1.1, 1.2, 1.3, 1.4, 1.5]) > 0.2
    # Degrau no meio: é exatamente o caso que o creep produz.
    assert d([1.0, 1.0, 1.0, 1.3, 1.3, 1.3]) == pytest.approx(0.3)


def test_a_deriva_nao_quebra_com_janela_minuscula(m):
    """O laço de medida chama isto antes de a janela encher."""
    d = m.TactileExplorer._deriva
    assert d([]) == 0.0
    assert d([1.0]) == 0.0


def test_o_limiar_de_assentado_esta_acima_do_ruido_da_celula(m):
    """Se o limiar ficasse ABAIXO do ruído, a força nunca seria declarada
    assentada e toda medida perto do alvo gastaria o teto de ticks — a
    descida ficaria 6x mais lenta sem ganhar exatidão nenhuma."""
    assert m._QS_SETTLE_DRIFT_N > m._FORCE_NOISE_SIGMA_N
    # E abaixo da banda do HOLD: um limiar maior que a própria tolerância
    # declararia assentado um sinal que ainda pode cruzar a banda inteira.
    assert m._QS_SETTLE_DRIFT_N < m._HOLD_TOL_N


def test_a_espera_assentada_so_vale_perto_do_alvo(m):
    """A medida assentada custa até 1 s por passo. Longe do alvo o passo é
    grande e o creep é irrelevante ao lado dele, então esperar só faria a
    descida demorar — o gatilho é o erro cair dentro de algumas bandas."""
    assert m._QS_SETTLE_NEAR_MULT >= 1.0
    assert m._QS_SETTLE_MAX_TICKS > m._QS_MEASURE_MAX_TICKS
