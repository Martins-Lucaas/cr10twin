"""O ângulo de ataque é CONFERIDO no braço, não só comandado.

Até 15/09/2026 a fase CALIBRATING media o plano, resolvia a IK, conferia a
FK da SOLUÇÃO e publicava o comando — e parava aí. Nada media o que o braço
fez com ele. Entre o comando e a pose real cabem um limite de junta
alcançado na execução, um JTC que não fecha o último grau, o lock de
orientação (proporcional) cedendo no trânsito de volta, e o braço real
atrasado em relação ao simulado.

O desfecho de falhar aqui é silencioso e é o pior possível: a descida, a
regulação de força e o alívio de emergência passam a correr sobre um eixo
que a ferramenta não tem, a célula lê a PROJEÇÃO da força normal — mede
menos do que aplica — e nenhum log denuncia.

Aqui se testa que a conferência existe, que ela usa a pose MEDIDA, e que a
barra de erro da medição do plano é honesta.
"""
import math

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from touch_pack.plane_probe import PlaneFit, fit_plane   # noqa: E402


@pytest.fixture(scope='module')
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def node(_ros):
    from touch_pack.tactile_explorer import TactileExplorer
    n = TactileExplorer()
    # Pose de HOME do banco: TCP exatamente na vertical para baixo.
    n._q_now = lambda: np.deg2rad([0, 0, -90, 0, 90, 0]).astype(float)
    n._stream_q = lambda *a, **k: None
    n._settle = lambda *a, **k: None
    n._speed_factor_pct = 10.0
    yield n
    n.destroy_node()


_DOWN = np.array([0.0, 0.0, -1.0])


# ── A conferência em si ───────────────────────────────────────────────

def test_eixo_entregue_confere_quando_a_pose_e_a_pedida(node):
    assert node._attack_axis_err_deg(_DOWN) == pytest.approx(0.0, abs=1e-6)
    assert node._verify_attack_axis(_DOWN, 'TESTE') is True


def test_eixo_entregue_recusa_pose_fora_da_tolerancia(node):
    """A pose medida aponta para baixo; pedir um eixo 10° torto tem de ser
    recusado, e não aceito porque 'a IK tinha resolvido'."""
    t = math.radians(10.0)
    pedido = np.array([math.sin(t), 0.0, -math.cos(t)])
    assert node._attack_axis_err_deg(pedido) == pytest.approx(10.0, abs=1e-3)
    assert node._verify_attack_axis(pedido, 'TESTE') is False


def test_a_banda_de_aceite_e_a_mesma_que_decide_girar(node):
    """Tolerâncias diferentes abririam uma faixa em que o punho gira mas a
    conferência aceita qualquer coisa — ou o contrário."""
    from touch_pack.explorer_constants import _ALIGN_ORI_TOL_DEG
    t = math.radians(_ALIGN_ORI_TOL_DEG * 0.9)
    quase = np.array([math.sin(t), 0.0, -math.cos(t)])
    assert node._verify_attack_axis(quase, 'TESTE') is True
    t = math.radians(_ALIGN_ORI_TOL_DEG * 1.1)
    passou = np.array([math.sin(t), 0.0, -math.cos(t)])
    assert node._verify_attack_axis(passou, 'TESTE') is False


def test_rotacao_aceita_pela_ik_mas_nao_entregue_pelo_braco_aborta(node):
    """O caso que o teste existe para pegar: a IK resolve, o comando é
    enviado, e o braço fica onde estava. Antes isto devolvia 'ok'."""
    t = math.radians(12.0)
    attack = np.array([math.sin(t), 0.0, -math.cos(t)])
    # _joint_stream_to "executa" sem que _q_now mude: braço parado.
    node._joint_stream_to = lambda q: True
    assert node._rotate_to_attack(attack, label='TESTE') == 'error'


def test_reaplicacao_apos_a_home_tambem_confere(node):
    """Os ciclos 2..N reaplicam a rotação em vez de re-sondar — mesmo
    buraco, mesma conferência (é o MESMO _rotate_to_attack)."""
    t = math.radians(12.0)
    node._attack_dir = np.array([math.sin(t), 0.0, -math.cos(t)])
    node._joint_stream_to = lambda q: True
    assert node._reapply_attack_orientation() is False


# ── A barra de erro da medição ────────────────────────────────────────

def test_tres_pontos_nao_certificam_nada():
    """Com N == 3 o plano passa EXATO pelos pontos: resíduo zero por
    construção. Anunciar incerteza zero a partir disso seria mentira."""
    pts = np.array([[0.0, 0.0, 0.5], [0.02, 0.0, 0.5], [0.0, 0.02, 0.5]])
    fit = fit_plane(pts)
    assert fit.rms_m == pytest.approx(0.0, abs=1e-12)
    assert fit.tilt_unc_deg == math.inf


def test_incerteza_cai_com_o_braco_de_alavanca():
    """σ/braço: dobrar o raio de sondagem com o mesmo ruído corta a
    incerteza pela metade. É o que se diz ao operador que quer garantir
    perpendicularidade."""
    def _unc(raio_m, ruido_m=50e-6):
        ang = np.arange(4) * (np.pi / 2.0)
        xy = raio_m * np.column_stack([np.cos(ang), np.sin(ang)])
        z = 0.5 + ruido_m * np.array([1.0, -1.0, 1.0, -1.0])
        return fit_plane(np.column_stack([xy, z])).tilt_unc_deg

    u1, u2 = _unc(0.010), _unc(0.020)
    assert u2 == pytest.approx(u1 / 2.0, rel=0.02)


def test_plano_inclinado_tem_desvio_muito_maior_que_a_incerteza():
    """Uma inclinação real de 5° medida num anel de 20 mm com ruído de
    50 µm sai muito acima da própria barra: o desvio é CONCLUSIVO."""
    t = math.tan(math.radians(5.0))
    ang = np.arange(4) * (np.pi / 2.0)
    xy = 0.020 * np.column_stack([np.cos(ang), np.sin(ang)])
    z = 0.5 + xy[:, 0] * t + 50e-6 * np.array([1.0, -1.0, 1.0, -1.0])
    fit = fit_plane(np.column_stack([xy, z]))
    assert fit.tilt_deg == pytest.approx(5.0, abs=0.3)
    assert fit.tilt_unc_deg < fit.tilt_deg / 10.0


def test_braco_de_alavanca_e_medido_no_plano():
    """O braço é a distância NO PLANO ao centroide — a componente normal é
    o próprio resíduo, e contá-la inflaria o braço justamente quando o
    ajuste está ruim, mascarando a incerteza."""
    ang = np.arange(4) * (np.pi / 2.0)
    xy = 0.015 * np.column_stack([np.cos(ang), np.sin(ang)])
    z = np.full(4, 0.5)
    assert fit_plane(np.column_stack([xy, z])).lever_m == pytest.approx(
        0.015, rel=1e-6)


def test_planefit_antigo_continua_construivel():
    """`lever_m` entrou com default para não quebrar quem constrói o
    PlaneFit por keyword (validate_fit não usa o campo)."""
    f = PlaneFit(normal=np.array([0.0, 0.0, 1.0]), centroid=np.zeros(3),
                 rms_m=0.0, tilt_deg=0.0, spread=1.0, n_points=4)
    assert f.lever_m == 0.0
    assert f.tilt_unc_deg == math.inf


# ── Conferência de ponta a ponta (probe_align_verify) ─────────────────
#
# Os testes acima conferem ELOS: a solução da IK, a pose entregue, a barra
# de erro do ajuste. Nenhum deles vê o PRODUTO — o ângulo entre ferramenta e
# peça — porque todos partem da mesma primeira medição e herdam o erro dela.
# A conferência re-sonda o plano com o ataque já corrigido: aí o número
# medido É o desvio residual.

def _ligar_verify(node, ligado=True):
    node.set_parameters([rclpy.parameter.Parameter(
        'probe_align_verify', rclpy.parameter.Parameter.Type.BOOL, ligado)])


def _plano(tilt_deg, z0=0.5, raio=0.02):
    """Quatro contatos num anel, sobre um plano inclinado de `tilt_deg`."""
    t = math.tan(math.radians(tilt_deg))
    ang = np.arange(4) * (np.pi / 2.0)
    xy = raio * np.column_stack([np.cos(ang), np.sin(ang)])
    return np.column_stack([xy, z0 + xy[:, 0] * t])


_CFG = {'force_n': 0.5, 'n': 4, 'radius_m': 0.02}


def test_desligada_nao_sonda_nada(node):
    """O default não pode custar uma rodada de toques a mais nem mudar o
    contrato de quem chama."""
    node._attack_dir = np.array([0.0, 0.0, -1.0])
    node._probe_plane = lambda *a, **k: pytest.fail('sondou com verify OFF')
    assert node._verify_attack_plane(
        np.zeros(3), np.zeros((4, 2)), _CFG) == 'ok'


def test_sem_rotacao_nao_ha_o_que_conferir(node):
    """Desvio abaixo da tolerância não gira o punho e não define
    `_attack_dir` — não há eixo corrigido para conferir."""
    _ligar_verify(node)
    node._attack_dir = None
    node._probe_plane = lambda *a, **k: pytest.fail('sondou sem eixo')
    assert node._verify_attack_plane(
        np.zeros(3), np.zeros((4, 2)), _CFG) == 'ok'


def test_ataque_perpendicular_a_face_e_aprovado(node):
    """O caso bom: o eixo corrigido é exatamente −normal do plano remedido."""
    from touch_pack.plane_probe import attack_dir_from_normal, fit_plane
    _ligar_verify(node)
    pts = _plano(8.0)
    node._attack_dir = attack_dir_from_normal(fit_plane(pts).normal)
    node._probe_plane = lambda *a, **k: ('ok', list(pts))
    assert node._verify_attack_plane(
        np.zeros(3), np.zeros((4, 2)), _CFG) == 'ok'


def test_peca_que_se_moveu_entre_as_duas_medicoes_reprova(node):
    """O que só a conferência enxerga: a primeira sondagem fechou, a
    rotação entregou o eixo pedido, e mesmo assim a ferramenta não está
    perpendicular — porque a face não é mais a que foi medida."""
    from touch_pack.plane_probe import attack_dir_from_normal, fit_plane
    _ligar_verify(node)
    node._attack_dir = attack_dir_from_normal(fit_plane(_plano(8.0)).normal)
    node._probe_plane = lambda *a, **k: ('ok', list(_plano(16.0)))
    assert node._verify_attack_plane(
        np.zeros(3), np.zeros((4, 2)), _CFG) == 'error'


def test_conferencia_interrompida_aborta(node):
    """Sem conferência não há garantia, e o ensaio foi pedido COM ela:
    seguir em frente entregaria exatamente a incerteza que se quis fechar."""
    _ligar_verify(node)
    node._attack_dir = np.array([0.0, 0.0, -1.0])
    node._probe_plane = lambda *a, **k: ('no_contact', [])
    assert node._verify_attack_plane(
        np.zeros(3), np.zeros((4, 2)), _CFG) == 'no_contact'


def test_conferencia_nao_contamina_o_aprendizado(node):
    """Os toques da conferência partem de outros XY, como os da primeira
    sondagem: não podem entrar no histórico de contato de home nenhuma."""
    from touch_pack.plane_probe import attack_dir_from_normal, fit_plane
    _ligar_verify(node)
    pts = _plano(8.0)
    node._attack_dir = attack_dir_from_normal(fit_plane(pts).normal)
    with node._params_lock:
        node._home_key_cur = (1.0, 2.0, 3.0)
        node._learned_contact_m = 0.030
        node._target_force_n = 3.0

    def _sonda(*a, **k):
        with node._params_lock:
            assert node._home_key_cur is None
            assert node._learned_contact_m is None
            # Sonda leve: nunca mais pesada que o próprio ensaio.
            assert node._target_force_n <= _CFG['force_n']
        return 'ok', list(pts)

    node._probe_plane = _sonda
    assert node._verify_attack_plane(
        np.zeros(3), np.zeros((4, 2)), _CFG) == 'ok'
    with node._params_lock:
        assert node._home_key_cur == (1.0, 2.0, 3.0)
        assert node._learned_contact_m == pytest.approx(0.030)
        assert node._target_force_n == pytest.approx(3.0)


# ── Curvatura: o modo cego do anel ────────────────────────────────────
#
# Numa calota simétrica TODOS os pontos do anel caem na mesma altura. O
# ajuste devolve um plano perfeito — desvio 0,000°, resíduo 0,000 mm,
# espalhamento 1,00 — sobre uma superfície que não é plana. Pior: com a
# barra de erro que este arquivo testa acima, `tilt_unc_deg` sai 0,0°, isto
# é, o sistema anuncia CERTEZA ABSOLUTA justamente no caso em que está cego.
#
# O toque central não acrescenta nada à normal (está no centroide), mas é a
# única medição que vê a curvatura.

def _calota(raio_curv_m, raio_anel_m=0.015, z0=0.5, com_centro=True,
            concava=False):
    """Contatos sobre uma calota de raio `raio_curv_m`. `concava=True`
    inverte a superfície (depressão no centro)."""
    from touch_pack.plane_probe import probe_pattern
    off = probe_pattern(4, raio_anel_m, with_center=com_centro)
    r2 = (off ** 2).sum(axis=1)
    z = np.sqrt(raio_curv_m ** 2 - r2) - raio_curv_m
    return np.column_stack([off, z0 + (-z if concava else z)])


def test_o_anel_sozinho_da_nota_perfeita_a_uma_calota():
    """A regressão que o toque central existe para fechar."""
    from touch_pack.plane_probe import validate_fit
    fit = fit_plane(_calota(0.200, com_centro=False))
    assert fit.tilt_deg == pytest.approx(0.0, abs=1e-6)
    assert fit.rms_m == pytest.approx(0.0, abs=1e-9)
    assert validate_fit(fit)[0] is True
    assert fit.tilt_unc_deg == pytest.approx(0.0, abs=1e-9)   # "certeza"


def test_a_sagitta_mede_a_calota_que_o_anel_nao_ve():
    """R = 200 mm com anel de 15 mm põe o centro 0,56 mm ACIMA do plano —
    muito além da reserva de indentação (alvo/K, dezenas de µm)."""
    from touch_pack.plane_probe import sagitta_m
    pts = _calota(0.200)
    fit = fit_plane(pts[1:])                 # normal SÓ do anel
    sag = sagitta_m(fit, pts[0])
    assert sag == pytest.approx(0.563e-3, rel=0.02)
    assert sag > 0, 'o sinal tem de dizer que o centro está ACIMA'


def test_depressao_no_centro_tem_sinal_negativo():
    """Uma concavidade não faz a ponteira encostar no ápice — o sinal
    distingue os dois casos, e o log diz qual é."""
    from touch_pack.plane_probe import sagitta_m
    pts = _calota(0.200, concava=True)
    fit = fit_plane(pts[1:])
    assert sagitta_m(fit, pts[0]) < 0


def test_superficie_plana_tem_sagitta_nula():
    from touch_pack.plane_probe import probe_pattern, sagitta_m
    off = probe_pattern(4, 0.015, with_center=True)
    pts = np.column_stack([off, np.full(len(off), 0.5)])
    fit = fit_plane(pts[1:])
    assert sagitta_m(fit, pts[0]) == pytest.approx(0.0, abs=1e-12)


def test_o_centro_nao_dilui_o_desvio_no_ajuste():
    """Por que a sagitta é medida contra o plano do ANEL e não deixada para
    o resíduo: incluir o centro entre os pontos ajustados transforma 1,13 mm
    de calota num RMS de 0,45 mm, que ainda passa no teto de 0,5 mm e ainda
    parece ruído de medição."""
    from touch_pack.plane_probe import sagitta_m, validate_fit
    pts = _calota(0.100)
    diluido = fit_plane(pts)                 # centro DENTRO do ajuste
    assert diluido.rms_m * 1e3 == pytest.approx(0.45, abs=0.02)
    assert validate_fit(diluido)[0] is True, 'o teto de RMS deixaria passar'
    direto = sagitta_m(fit_plane(pts[1:]), pts[0])
    assert direto * 1e3 == pytest.approx(1.13, rel=0.02)


def test_calibracao_recusa_amostra_curva(node):
    """Ponta a ponta: a fase aborta em vez de girar o punho para uma normal
    que não descreve a superfície."""
    from touch_pack.explorer_constants import _ALIGN_ORI_TOL_DEG   # noqa: F401
    pts = _calota(0.100)
    with node._params_lock:
        node._mode = 'TOUCH'
        node._align_from_msg = True
        node._align_on = True
        node._align_msg = {}
    node._probe_plane = lambda *a, **k: ('ok', [p for p in pts])
    node._tcp_now = lambda: np.array([0.0, 0.0, 0.55])
    node._move_linear_world = lambda *a, **k: 'done'
    assert node._phase_calibrate_attack() == 'error'
    assert node._attack_dir is None, 'girou o punho para uma calota'


def test_calibracao_aceita_plano_inclinado_de_verdade(node):
    """A guarda nova não pode recusar o caso que a fase existe para tratar:
    uma face PLANA e inclinada segue sendo calibrada."""
    t = np.tan(np.deg2rad(6.0))
    from touch_pack.plane_probe import probe_pattern
    off = probe_pattern(4, 0.015, with_center=True)
    pts = np.column_stack([off, 0.5 + off[:, 0] * t])
    with node._params_lock:
        node._mode = 'TOUCH'
        node._align_from_msg = True
        node._align_on = True
        node._align_msg = {}
    node._probe_plane = lambda *a, **k: ('ok', [p for p in pts])
    node._align_reorient = lambda *a, **k: 'ok'
    node._tcp_now = lambda: np.array([0.0, 0.0, 0.55])
    node._move_linear_world = lambda *a, **k: 'done'
    assert node._phase_calibrate_attack() == 'ok'
    assert node._attack_dir is not None
