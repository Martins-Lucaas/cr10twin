"""Regressões da auditoria de 24/08/2026 (controle de força, movimentação, GUI).

Um teste por defeito encontrado. Todos falham no código anterior à
auditoria — é isso que os torna testes e não descrição.
"""
import os

import pytest

os.environ.setdefault('ROS_DOMAIN_ID', '77')


@pytest.fixture(scope='module')
def _ros():
    """Contexto rclpy DESTE módulo. Init/shutdown emparelhados: cada arquivo
    de teste sobe o seu, e um `rclpy.init()` solto aqui derrubaria a fixture
    de todos os módulos seguintes com "Context.init() must only be called
    once"."""
    rclpy = pytest.importorskip('rclpy')
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def explorer(_ros):
    """TactileExplorer com a thread do protocolo neutralizada — o que se
    testa aqui é só o parsing do start."""
    from touch_pack.tactile_explorer import TactileExplorer
    node = TactileExplorer()
    node._run_protocol = lambda: None
    yield node
    node.destroy_node()


# ══════════════════════════════════════════════════════════════════════
# 1. mirror_node: fase nova nasce ATIVA, não invisível
# ══════════════════════════════════════════════════════════════════════

def test_mirror_espelha_calibrating_e_transit():
    """As duas percorrem retas cartesianas publicadas como UMA
    JointTrajectory. Fora da lista de ativas, o ServoJ calava e quem assumia
    era o MovJ debounced: o braço real fazia um PTP ARTICULAR até o ponto
    final em vez da reta (podendo mergulhar abaixo do plano no MATRIX_MAP)."""
    pytest.importorskip('rclpy')
    from touch_pack.mirror_node import _is_active
    for fase in ('CALIBRATING', 'TRANSIT'):
        assert _is_active(fase), f'{fase} precisa ser espelhada'


def test_mirror_conta_fase_desconhecida_como_ativa():
    """A regra é o COMPLEMENTO de propósito. Com allowlist, toda fase nova
    nascia invisível para o espelho — foi assim que CALIBRATING e TRANSIT
    passaram despercebidas."""
    pytest.importorskip('rclpy')
    from touch_pack.mirror_node import _is_active
    assert _is_active('UMA_FASE_QUE_AINDA_NAO_EXISTE')


def test_mirror_so_solta_o_braco_nas_tres_fases_ociosas():
    pytest.importorskip('rclpy')
    from touch_pack.mirror_node import _is_active
    for fase in ('IDLE', 'DONE', 'ABORTED'):
        assert not _is_active(fase)


def test_mirror_cobre_todas_as_fases_do_explorer():
    """Nenhuma fase do PHASE_CODES pode ficar sem decisão explícita."""
    pytest.importorskip('rclpy')
    from touch_pack.constants import PHASE_CODES
    from touch_pack.mirror_node import _ACTIVE_PHASES, _IDLE_PHASES
    for fase in PHASE_CODES:
        assert (fase in _ACTIVE_PHASES) != (fase in _IDLE_PHASES), fase


def test_mirror_e_gui_usam_a_mesma_regra():
    """Os dois espelhos divergiam: a GUI usava o complemento e o
    mirror_node uma allowlist. Divergência aqui é o braço real seguindo o sim
    com a GUI aberta e NÃO seguindo com no_gui:=true."""
    pytest.importorskip('rclpy')
    from touch_pack.mirror_node import _IDLE_PHASES
    assert set(_IDLE_PHASES) == {'IDLE', 'DONE', 'ABORTED'}


# ══════════════════════════════════════════════════════════════════════
# 2. Banda do HOLD: fonte única, e a GUI não tem número próprio
# ══════════════════════════════════════════════════════════════════════

def test_banda_e_a_mesma_constante_nos_tres_consumidores():
    pytest.importorskip('rclpy')
    from touch_pack import constants
    from touch_pack.tactile_explorer import _HOLD_TOL_N, _HOLD_TOL_PCT
    import touch_pack.palpation_gui as gui
    assert _HOLD_TOL_N is constants.HOLD_TOL_N
    assert _HOLD_TOL_PCT is constants.HOLD_TOL_PCT
    assert gui._HOLD_TOL_N is constants.HOLD_TOL_N


def test_gui_nao_carrega_default_de_banda_proprio():
    """O 0,15 N cravado na GUI sobrescrevia a banda derivada do ruído da
    célula em TODO run lançado pela tela, porque a PalpationStart vence o
    default do explorer sempre que traz hold_tol_n > 0."""
    import re
    from pathlib import Path
    import touch_pack.palpation_gui as gui
    src = Path(gui.__file__).read_text()
    trecho = re.search(r'self\.hold_tol_var\s*=.*?\n\n', src, re.S)
    assert trecho, 'não achei a criação de hold_tol_var'
    assert '0.15' not in trecho.group(0), (
        'default de banda cravado na GUI: use constants.hold_tol_n()')


def test_gui_default_de_banda_e_a_lei_do_explorer():
    from touch_pack.constants import HOLD_TOL_N, HOLD_TOL_PCT, hold_tol_n
    assert hold_tol_n(0.5) == HOLD_TOL_N          # alvo baixo: manda o ruído
    assert hold_tol_n(5.0) == HOLD_TOL_PCT * 5.0  # alvo alto: manda a fração
    assert hold_tol_n(-2.0) == hold_tol_n(2.0)    # simétrica no sinal


def test_explorer_eleva_banda_abaixo_do_ruido_ao_piso(explorer):
    """Uma banda mais estreita que σ não é um critério mais apertado: é um
    que a célula não consegue avaliar, e o hold reiniciaria a janela de
    estabilidade em cima do próprio ruído para sempre."""
    from touch_pack.constants import HOLD_TOL_N
    _start(explorer, hold_tol_n=0.001)
    assert explorer._hold_tol_n == HOLD_TOL_N
    _start(explorer, hold_tol_n=0.30)      # acima do piso: respeitado
    assert explorer._hold_tol_n == pytest.approx(0.30)
    _start(explorer, hold_tol_n=0.0)       # 0 = "use o default do explorer"
    assert explorer._hold_tol_n is None


def test_migracao_descarta_hold_tol_persistido_da_v1():
    """Sem a poda, o conserto não teria efeito nenhum em quem já usou a GUI
    uma vez: o arquivo guarda o valor do último start, e todo start anterior
    mandou o default stale de 0,15 N."""
    pytest.importorskip('rclpy')
    import touch_pack.palpation_gui as gui
    migra = gui.PalpationGUI._migrate_palp_params
    fake = type('F', (), {'get_logger': lambda self: type(
        'L', (), {'info': lambda self, *a: None})()})()
    fake._PALP_PARAMS_VERSION = gui.PalpationGUI._PALP_PARAMS_VERSION
    v1 = migra(fake, {'hold_tol': 0.15, 'force_sp': 2.0})
    assert 'hold_tol' not in v1
    assert v1['force_sp'] == 2.0            # o resto do arquivo sobrevive
    v2 = migra(fake, {'hold_tol': 0.42, 'params_version':
                      gui.PalpationGUI._PALP_PARAMS_VERSION})
    assert v2['hold_tol'] == 0.42           # escolha deliberada permanece


# ══════════════════════════════════════════════════════════════════════
# 3. E-STOP não pode anunciar mais do que verificou
# ══════════════════════════════════════════════════════════════════════

def test_drag_teach_recusado_com_estop_pressionado():
    """Soltar os freios do arrasto com a chave pressionada é exatamente o
    movimento inesperado que o E-Stop existe para impedir."""
    from touch_pack.real_driver import CR10RealDriver, CR10RealDriverError
    drv = CR10RealDriver(dry_run=True)
    drv.emergency_stop()
    with pytest.raises(CR10RealDriverError, match='E-STOP'):
        drv.drag_teach(True)
    drv.drag_teach(False)       # desligar é o sentido seguro: passa sempre


def test_gui_separa_falha_do_estop_real_do_sucesso():
    """A trava local vale sempre (ela bloqueia o Start e o driver), mas o
    texto não pode afirmar 'robot disabled and alarmed' quando o
    EmergencyStop(1) nem chegou ao controlador."""
    import re
    from pathlib import Path
    import touch_pack.palpation_gui as gui
    src = Path(gui.__file__).read_text()
    corpo = re.search(r'def _estop_engage.*?(?=\n    def )', src, re.S).group(0)
    assert 'hw_ok' in corpo, 'sucesso e falha do hardware indistinguíveis'
    assert 'FAILED' in corpo


# ══════════════════════════════════════════════════════════════════════
# 4. STOP/FREEZE não vazam para o run seguinte
# ══════════════════════════════════════════════════════════════════════

def test_start_limpa_stop_e_freeze_pendentes(explorer):
    """Um STOP no fim do run (quando `_joint_batch_to` sai pela porta rápida
    e não consome o Event) matava a PRIMEIRA fase do run seguinte, sem causa
    visível para o operador."""
    explorer._stop_requested.set()
    explorer._freeze_requested.set()
    _start(explorer)
    assert not explorer._stop_requested.is_set()
    assert not explorer._freeze_requested.is_set()


# ══════════════════════════════════════════════════════════════════════
# 5. ft_probe: socket fechado não vira laço ocupado
# ══════════════════════════════════════════════════════════════════════

def test_sockreader_sinaliza_eof_em_vez_de_girar():
    """recv() devolvendo b'' fazia read() voltar vazio para sempre, e os dois
    laços tratavam isso como timeout: 100 % de CPU acusando 24 V/A-B quando o
    que caiu foi o socket."""
    probe = _ft_probe()

    class _Fechado:
        def settimeout(self, _t): pass
        def recv(self, _n): return b''

    r = probe._SockReader(_Fechado())
    assert r.read(1) == b''
    assert r.eof is True


def test_sockreader_nao_marca_eof_em_timeout():
    import socket
    probe = _ft_probe()

    class _Mudo:
        def settimeout(self, _t): pass
        def recv(self, _n): raise socket.timeout()

    r = probe._SockReader(_Mudo())
    assert r.read(1) == b''
    assert r.eof is False       # linha muda != conexão caída


def test_sockreader_guarda_o_resto_do_bloco():
    """in_waiting é o que faz os laços drenarem o recv inteiro; sem ele a
    leitura sairia byte a byte."""
    probe = _ft_probe()

    class _Jorro:
        def __init__(self): self._n = 0
        def settimeout(self, _t): pass
        def recv(self, _n):
            self._n += 1
            return b'ABCDEF' if self._n == 1 else b''

    r = probe._SockReader(_Jorro())
    assert r.read(1) == b'A'
    assert r.in_waiting == 5
    assert r.read(5) == b'BCDEF'


# ══════════════════════════════════════════════════════════════════════
# 6. Teto do link segue o baud EM USO
# ══════════════════════════════════════════════════════════════════════

def test_teto_do_link_acompanha_o_baud():
    from touch_pack.constants import (
        FT_FRAME_LEN, FT_MAX_RATE_HZ, FT_SERIAL_BAUD, ft_max_rate_hz)
    assert ft_max_rate_hz() == FT_MAX_RATE_HZ
    assert ft_max_rate_hz(FT_SERIAL_BAUD) == FT_MAX_RATE_HZ
    assert ft_max_rate_hz(460800) == 460800 / (FT_FRAME_LEN * 10)
    assert ft_max_rate_hz(460800) == pytest.approx(4 * FT_MAX_RATE_HZ)


# ══════════════════════════════════════════════════════════════════════
# 7. Código morto que descrevia máquinas inexistentes
# ══════════════════════════════════════════════════════════════════════

def test_gui_nao_publica_mais_em_ft_sensor_wrench():
    """O bridge do CR10 estava desligado desde 12/08 mas a máquina inteira
    continuava no arquivo, e a aba "6 Axes" ainda avisava de um conflito de
    publicadores que não podia mais acontecer."""
    from pathlib import Path
    import touch_pack.palpation_gui as gui
    import touch_pack.gui_loadcell as glc
    src = Path(gui.__file__).read_text()
    assert '_force_bridge' not in src
    assert '_wrench_pub' not in src
    assert '_force_bridge_active' not in Path(glc.__file__).read_text()


def test_gui_tem_cor_para_toda_fase_do_explorer():
    import re
    from pathlib import Path
    import touch_pack.palpation_gui as gui
    from touch_pack.constants import PHASE_CODES
    src = Path(gui.__file__).read_text()
    mapa = re.search(r'phase_color = \{(.*?)\}\.get', src, re.S).group(1)
    for fase in PHASE_CODES:
        assert f"'{fase}'" in mapa, f'{fase} cai na cor genérica'


def test_cronometro_para_em_frozen():
    """FROZEN é um fim de run (E-STOP congela no lugar). Enquanto ficava de
    fora da lista, o cronômetro da GUI contava para sempre depois dele."""
    import touch_pack.palpation_gui as gui
    assert 'FROZEN' in gui._PHASE_ENDED


# ══════════════════════════════════════════════════════════════════════
# 8. O alívio nunca empurra
# ══════════════════════════════════════════════════════════════════════

def test_alivio_nunca_empurra():
    """Com fz ABAIXO de _QS_RELIEF_FLOOR_N a expressão do piso fica positiva
    e o max() transformava um passo de alívio num de empurrar. Hoje isso só
    não acontece por coincidência de três limites independentes, então a
    invariante tem de ser do CÓDIGO, não do contexto."""
    pytest.importorskip('rclpy')
    from touch_pack.tactile_explorer import (
        TactileExplorer, _QS_RELIEF_FLOOR_N)
    passo = TactileExplorer._qs_relief_step
    for fz in (0.0, 0.05, _QS_RELIEF_FLOOR_N, 0.5, 3.0):
        for bruto in (-1e-4, -1e-6, 0.0):
            out = passo(bruto, fz, 5_000.0, 0.3)
            assert out <= 0.0, f'fz={fz} bruto={bruto} virou {out}'


def test_alivio_ainda_respeita_o_piso_quando_ha_folga():
    """A guarda de sinal não pode ter comido a função do piso: com força bem
    acima dele o recuo continua limitado a (fz - piso)/k."""
    pytest.importorskip('rclpy')
    from touch_pack.tactile_explorer import (
        TactileExplorer, _QS_RELIEF_FLOOR_N)
    k_push, fz = 5_000.0, 3.0
    limite = -(fz - _QS_RELIEF_FLOOR_N) / k_push
    # Pedido enorme (-1 m) e teto por ΔF folgado: quem morde é o piso.
    assert TactileExplorer._qs_relief_step(
        -1.0, fz, k_push, 1e6) == pytest.approx(limite)


def test_alivio_respeita_o_teto_por_df():
    """O teto por ΔF é o mesmo do empurrar e continua sendo o primeiro a
    morder quando o piso está longe."""
    pytest.importorskip('rclpy')
    from touch_pack.tactile_explorer import TactileExplorer
    k_push = 5_000.0
    assert TactileExplorer._qs_relief_step(
        -1.0, 9.0, k_push, 0.3) == pytest.approx(-0.3 / k_push)


# ── auxiliares ────────────────────────────────────────────────────────

def _ft_probe():
    """Importa scripts/ft_probe.py, que não é módulo do pacote."""
    import importlib.util
    from pathlib import Path
    caminho = (Path(__file__).resolve().parent.parent
               / 'scripts' / 'ft_probe.py')
    spec = importlib.util.spec_from_file_location('_ft_probe', caminho)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _start(node, **campos):
    """Aplica uma PalpationStart mínima e devolve o resultado do parsing."""
    from touch_pack_msgs.msg import PalpationStart
    msg = PalpationStart()
    msg.depth_mm = 10.0
    msg.force_n = 2.0
    msg.slide_dist_mm = 10.0
    msg.speed_mms = 10.0
    msg.mode = 'TOUCH'
    for k, v in campos.items():
        setattr(msg, k, v)
    node._busy.clear()
    ok = node._start_from_msg(msg)
    if node._protocol_thread is not None:
        node._protocol_thread.join(timeout=2.0)
    node._busy.clear()
    return ok
