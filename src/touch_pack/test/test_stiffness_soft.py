"""Estimador de rigidez com contato MOLE (ponteira de silicone).

Números de bancada (14/08/2026, run TOUCH/20260814_102401, regressão
força × penetração do TCP no samples.csv):

    ponteira rígida   28    N/mm
    silicone          0,62  N/mm   <- 45x mais mole

Com esses valores o caminho antigo travava num impasse circular: antes do
primeiro K_est o regulador limitava o passo a 8 µm, e 8 µm
no silicone produzem 0,005 N — abaixo do limiar de 0,1 N que o update_pair
usava para DESCARTAR o par. Sem par aceito não havia K_est; sem K_est o teto
de sonda continuava valendo. A descida rastejou 25 s e parou em 0,41 N com
1 N pedido.
"""
import pytest

pytest.importorskip('rclpy')

K_SILICONE_NM = 620.0      # N/m  (0,62 N/mm, medido)
K_RIGIDA_NM   = 28_000.0   # N/m  (28 N/mm, medido)
# Secante do PÉ da curva — o trecho logo acima de _CONTACT_ON_N, que é onde o
# estimador tem de latch para o regulador largar o teto de sonda. Regressão
# força × penetração sobre as 2133 amostras entre 0,06 e 0,23 N do run
# TOUCH/20260817_104248: 0,279 N/mm, 2,2x mais mole que a secante perto do
# setpoint que fixou o piso anterior.
K_PE_DA_CURVA_NM = 279.0   # N/m  (0,279 N/mm, medido)


@pytest.fixture()
def est():
    from touch_pack.tactile_explorer import _StiffnessEstimator
    e = _StiffnessEstimator()
    e.reset()
    return e


def _passos(est, k_real_nm, dx_m, n):
    """Aplica `n` micro-passos de `dx_m` contra um contato de rigidez
    `k_real_nm`, como o _qs_regulate faz (par Δx executado / ΔF medido)."""
    for _ in range(n):
        est.update_pair(dx_m, k_real_nm * dx_m)


def test_silicone_cabe_no_piso_do_estimador():
    """O piso não pode excluir a ponteira que está na bancada."""
    from touch_pack.tactile_explorer import _K_MIN_NM
    assert _K_MIN_NM < K_SILICONE_NM, (
        f'piso {_K_MIN_NM} N/m exclui o silicone medido ({K_SILICONE_NM} N/m)')


def test_pe_da_curva_cabe_no_piso_do_estimador():
    """O piso tem de cobrir o PÉ da curva, não só a secante perto do alvo.

    É onde o latch precisa acontecer: enquanto `estimated` for False o passo
    fica no teto de sonda de 8 µm, e no run 20260817_104248 isso custou 25,9 s
    para percorrer 0,60 mm sem sair de 0,23 N.
    """
    from touch_pack.tactile_explorer import _K_MIN_NM
    assert _K_MIN_NM < K_PE_DA_CURVA_NM, (
        f'piso {_K_MIN_NM} N/m exclui o pé da curva medido '
        f'({K_PE_DA_CURVA_NM} N/m) — a descida volta a rastejar no teto de sonda')


def test_aprende_o_pe_da_curva_com_o_passo_de_sonda(est):
    """O caso do run 20260817_104248: 8 µm produzem 0,0022 N contra o pé."""
    dx = 8.0e-6
    _passos(est, K_PE_DA_CURVA_NM, dx, 25)
    assert est.estimated, 'K do pé nunca foi estimado — a descida rasteja'
    assert est.value == pytest.approx(K_PE_DA_CURVA_NM, rel=0.10)


def test_piso_menor_nao_afrouxa_o_passo():
    """Baixar o piso não pode aumentar passo nenhum: abaixo de ~3.000 N/m quem
    morde é o teto ABSOLUTO, não o teto por ΔF."""
    from touch_pack.tactile_explorer import (
        _K_MIN_NM, _QS_DX_MAX_M, _QS_DF_HARD_N)
    assert _QS_DF_HARD_N / _K_MIN_NM > _QS_DX_MAX_M, (
        'no piso do estimador o teto por ΔF passou a morder antes do teto '
        'absoluto — baixar o piso agora afrouxa o passo')


def test_aprende_silicone_com_o_passo_de_sonda(est):
    """O caso que travava: passos de 8 µm, ΔF de 0,005 N cada.

    Nenhum par isolado cruza o limiar de ruído; a soma deles cruza.
    """
    dx = 8.0e-6                              # 8 µm — o teto de sonda de então
    df_por_passo = K_SILICONE_NM * dx
    assert df_por_passo < 0.05, 'premissa: um passo isolado fica no ruído'

    _passos(est, K_SILICONE_NM, dx, 12)

    assert est.estimated, 'K nunca foi estimado — o impasse voltou'
    assert est.value == pytest.approx(K_SILICONE_NM, rel=0.10)


def test_ainda_aprende_contato_rigido(est):
    """A acumulação não pode estragar o caso que já funcionava."""
    _passos(est, K_RIGIDA_NM, 8.0e-6, 4)
    assert est.estimated
    assert est.value == pytest.approx(K_RIGIDA_NM, rel=0.10)


def test_um_passo_grande_sozinho_ainda_vale(est):
    """Par que já cruza o limiar sozinho não precisa de acumulação."""
    est.update_pair(1.0e-5, K_RIGIDA_NM * 1.0e-5)   # 0,28 N num passo
    assert est.estimated
    assert est.value == pytest.approx(K_RIGIDA_NM, rel=0.05)


def test_acumulador_zera_no_reset(est):
    """Trecho de um contato não pode vazar para o próximo."""
    _passos(est, K_SILICONE_NM, 8.0e-6, 3)     # acumula sem cruzar o limiar
    assert not est.estimated
    est.reset()
    # Agora um contato RÍGIDO: se o acumulado anterior tivesse sobrevivido, o
    # Δx extra achataria k_inst para baixo.
    est.update_pair(1.0e-5, K_RIGIDA_NM * 1.0e-5)
    assert est.value == pytest.approx(K_RIGIDA_NM, rel=0.05)


def test_ruido_puro_nao_vira_rigidez(est):
    """ΔF alternando em torno de zero não pode virar uma estimativa."""
    for i in range(40):
        est.update_pair(8.0e-6, 0.01 if i % 2 == 0 else -0.01)
    assert not est.estimated, 'ruído simétrico não deveria estimar nada'


def test_passo_do_regulador_cobre_o_silicone():
    """Com K do silicone, o teto absoluto tem de deixar passar um ΔF útil.

    É a checagem que amarra _QS_DX_MAX_M ao K real: 0,03 N por passo é o piso
    do que faz o hold convergir em tempo razoável (1 N em ~30 passos).
    """
    from touch_pack.tactile_explorer import _QS_DX_MAX_M, _QS_DF_HARD_N
    df_por_passo = K_SILICONE_NM * _QS_DX_MAX_M
    assert df_por_passo >= 0.03, (
        f'teto de {_QS_DX_MAX_M*1e6:.0f} µm só move {df_por_passo:.4f} N '
        'no silicone — o hold não converge')
    # E na ponteira rígida quem manda continua sendo o teto por ΔF.
    hard_cap_m = _QS_DF_HARD_N / K_RIGIDA_NM
    assert hard_cap_m < _QS_DX_MAX_M, (
        'no contato rígido o teto absoluto passou a morder antes do teto por '
        'ΔF — o comportamento antigo mudou onde não devia')
