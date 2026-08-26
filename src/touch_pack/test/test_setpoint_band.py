"""Banda de chegada derivada do RUÍDO da célula, e setpoints degenerados.

Contexto (19/08/2026): o piso absoluto da banda era 0,15 N, um número solto.
Num alvo de 0,1 N ele abria a banda [-0,05; +0,25] N — inclui força ZERO e
2,5x o alvo. Nesse regime o laço declarava chegada em qualquer lugar e
"overshoot" deixava de ser mensurável, porque a banda já o continha.
"""
import os

os.environ.setdefault('ROS_DOMAIN_ID', '77')

from touch_pack.tactile_explorer import (
    _CONTACT_ON_N, _FORCE_NOISE_SIGMA_N, _HOLD_TOL_N, _HOLD_TOL_PCT,
    _HOLD_TOL_SIGMA, _HOLD_STABLE_S, _CTRL_DT, setpoint_resolvable)


def _tol(target_f: float) -> float:
    """Espelho da conta que os cinco call sites do explorer fazem."""
    return max(_HOLD_TOL_N, _HOLD_TOL_PCT * target_f)


def test_piso_da_banda_e_o_ruido_da_celula():
    """O piso não é arbitrário: é múltiplo de σ. Re-medir a célula tem de ser
    mudar UM número e a banda seguir junto."""
    assert _HOLD_TOL_N == _HOLD_TOL_SIGMA * _FORCE_NOISE_SIGMA_N


def test_banda_nunca_inclui_forca_zero():
    """Borda inferior <= 0 significa que "cheguei ao setpoint" é satisfeito
    sem tocar em nada. É o bug que 0,15 N tinha no alvo de 0,1 N."""
    for target in (0.1, 0.15, 0.2, 0.5, 1.0, 2.0, 5.0):
        assert target - _tol(target) > 0.0, f'alvo {target} N: banda inclui zero'


def test_sigma_alto_demais_e_pego_pelo_aviso():
    """Contrapositivo do teste acima: com a banda larga o bastante para cruzar
    o limiar de contato, `setpoint_resolvable` tem de reprovar — é o que põe o
    aviso no log em vez de deixar o ensaio passar calado."""
    ok, porque = setpoint_resolvable(0.10, _tol(0.10))
    assert not ok and 'limiar de contato' in porque
    # Alvo folgado acima do limiar + banda: aprovado, sem ruído no log.
    ok, porque = setpoint_resolvable(1.0, _tol(1.0))
    assert ok and porque == ''


def test_banda_a_4_sigma_sobrevive_a_janela_de_estabilidade():
    """Sair da banda RESETA a janela de _HOLD_STABLE_S. A 3σ esperam-se ~0,5
    excursões por janela (o hold reiniciaria sozinho); a 4σ, ~0,01."""
    amostras = _HOLD_STABLE_S / _CTRL_DT
    p_fora_4s = 6.3e-5          # duas caudas, normal
    assert _HOLD_TOL_SIGMA >= 4.0
    assert amostras * p_fora_4s < 0.05


def test_menor_alvo_com_sentido_esta_documentado():
    """O piso prático do setpoint é limiar de contato + banda. Se algum dia
    ele cair abaixo do próprio limiar, a conta do aviso ficou incoerente."""
    menor = _CONTACT_ON_N + _tol(_CONTACT_ON_N)
    assert menor > _CONTACT_ON_N
    ok, _ = setpoint_resolvable(menor * 1.001, _tol(menor))
    assert ok
