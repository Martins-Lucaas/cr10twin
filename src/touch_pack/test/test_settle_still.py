"""Parada VERIFICADA na entrada do DESCENDING (`_settle_until_still`).

Contexto (01/09/2026, coleta 20260901_094646): a descida saiu na diagonal em
vez de em Z puro. A causa não estava no controle cartesiano — assim que ele
assume, o vetor de deslocamento é (≈0, ≈0, −0,98). Estava na ENTRADA da fase:

  4x3 / 4x4 (boas)   q4 na 1ª amostra = 53,00547°, erro vs home −0,00009°,
                     Δq dos 3 ticks anteriores = 0,0000 → braço PARADO.
                     Descida a 0,0° médios da vertical (máx 1,0°).
  20260901_094646    q4 na 1ª amostra = 52,098°, erro vs home −0,907°, ainda
                     frenando a 0,057 rad/s (J4 tinha passado 1,25° do alvo).
                     Descida a 24,3° médios da vertical (máx 65,7°).

O `_settle` é malha aberta: publica a pose travada por `_SETTLE_TICKS` ticks
fixos (180 ms) e volta, sem verificar nada. Ele FREIA — o Δq4 medido decai
0,0571 → 0,0398 → 0,0354 → 0,0146 rad/s dentro da janela — mas devolve o
controle no meio da frenagem. Como o DESCENDING só comanda Z, o que sobra de
velocidade vira movimento lateral do TCP: J4 sozinha varre um arco de raio
~0,32 m cuja tangente é (−0,44, +0,79, −0,43), 65° fora da vertical.

Estes testes cobrem o critério de saída, não a cinemática.
"""
import numpy as np
import pytest


@pytest.fixture(scope='module')
def m():
    from touch_pack import tactile_explorer
    return tactile_explorer


@pytest.fixture()
def node(m, monkeypatch):
    """Instância NUA: só os dois colaboradores que `_settle_until_still` usa.

    Instanciar o nó ROS inteiro pediria rclpy, publishers e um /joint_states
    vivo para exercitar um laço de vinte linhas — mesma escolha do
    test_no_overshoot."""
    n = object.__new__(m.TactileExplorer)
    n.enviados = []
    n._stream_q = lambda q, dt, velocities=None: n.enviados.append(np.asarray(q).copy())
    monkeypatch.setattr(m.time, 'sleep', lambda _s: None)
    return n


def _feed(node, velocidades_rad_s, dt=0.041, q0=None):
    """Programa `_q_sample` com uma trajetória de J4 dada pelas velocidades
    (uma leitura de /joint_states por entrada). dt = 0,041 s é o intervalo
    real entre amostras da coleta."""
    q = np.zeros(6) if q0 is None else q0.copy()
    ts = 100.0
    leituras = [(q.copy(), ts)]
    for v in velocidades_rad_s:
        q = q.copy()
        q[3] += v * dt
        ts += dt
        leituras.append((q.copy(), ts))
    it = iter(leituras)
    ultimo = [leituras[0]]

    def _sample():
        try:
            ultimo[0] = next(it)
        except StopIteration:
            pass                      # feed esgotado: repete a última leitura
        return ultimo[0][0].copy(), ultimo[0][1]

    node._q_sample = _sample


# ── o caso normal: braço já parado ────────────────────────────────────

def test_braco_parado_sai_em_quiet_ticks(m, node):
    """4x3/4x4: o braço chega no DESCENDING parado. O critério tem de sair
    imediatamente — se custasse ticks, seria um atraso novo no caminho que
    hoje funciona."""
    _feed(node, [0.0] * 10)
    assert node._settle_until_still(quiet_ticks=3) is True
    assert len(node.enviados) == 3


# ── o caso da coleta 20260901_094646 ──────────────────────────────────

def test_espera_a_frenagem_do_home_terminar(m, node):
    """As 4 primeiras velocidades são as MEDIDAS na janela do settle; a cauda
    é a continuação da frenagem que o `_settle` não esperou. O critério tem de
    segurar até ela cair abaixo da tolerância."""
    medidas = [0.0571, 0.0398, 0.0354, 0.0146]
    cauda = [0.0080, 0.0040, 0.0015, 0.0008, 0.0004, 0.0002, 0.0]
    _feed(node, medidas + cauda)
    assert node._settle_until_still(quiet_ticks=3) is True
    # Segurou além dos 6 ticks fixos do _settle: é exatamente o que faltava.
    assert len(node.enviados) > m._SETTLE_TICKS


def test_settle_fixo_teria_liberado_com_velocidade_residual(m):
    """Ancora o diagnóstico: 180 ms de `_settle` não cobrem a frenagem
    medida. A 4ª leitura (fim da janela) ainda estava 7x acima da tolerância."""
    assert m._SETTLE_TICKS * m._CTRL_DT == pytest.approx(0.180)
    assert 0.0146 > m._SETTLE_STILL_TOL_RAD_S * 7


def test_tolerancia_limita_a_deriva_lateral(m):
    """A tolerância existe para limitar o ERRO DE TCP, não a velocidade de
    junta: J4 move o TCP num braço de ~0,32 m, e a tangente do arco tem 0,90
    de componente XY. Abaixo de 4% de um approach de 15 mm/s."""
    deriva_mm_s = 0.90 * 0.320 * m._SETTLE_STILL_TOL_RAD_S * 1e3
    assert deriva_mm_s < 0.04 * 15.0
    # e acima do piso de ruído: o feedback é quantizado em 1e-5 rad
    assert m._SETTLE_STILL_TOL_RAD_S > 5 * (1e-5 / 0.041)


# ── guardas ───────────────────────────────────────────────────────────

def test_leitura_repetida_nao_conta_como_parado(m, node):
    """/joint_states chega a ~24 Hz contra os 33 Hz do laço: um tick em cada
    quatro relê a MESMA pose. Medir entre ticks daria Δq = 0 e declararia
    parado um braço em movimento — a falha que o critério mais precisa não
    ter. Aqui o timestamp nunca avança: nenhuma leitura é nova."""
    q = np.zeros(6)
    def _sample():
        q[3] += 0.05 * 0.041          # o braço SE MOVE...
        return q.copy(), 100.0        # ...mas nada novo chegou
    node._q_sample = _sample
    assert node._settle_until_still(max_ticks=10, quiet_ticks=3) is False


def test_teto_devolve_false_sem_travar(m, node):
    """Braço que não para: o laço não pode ser infinito. Devolve False e quem
    chamou avisa e desce assim mesmo — comportamento de hoje, agora audível."""
    _feed(node, [0.05] * 50)
    assert node._settle_until_still(max_ticks=8, quiet_ticks=3) is False
    assert len(node.enviados) == 8


def test_segura_a_pose_de_entrada_e_nao_persegue_a_deriva(m, node):
    """A pose publicada é a da ENTRADA, sempre a mesma. Re-travar na pose
    corrente a cada tick perseguiria a deriva em vez de freá-la."""
    _feed(node, [0.05] * 20)
    node._settle_until_still(max_ticks=5, quiet_ticks=3)
    assert all(np.array_equal(p, node.enviados[0]) for p in node.enviados)
