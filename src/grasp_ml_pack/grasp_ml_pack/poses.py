"""
poses.py — fonte única de poses, grips e fases para a célula CR10+COVVI.

Conforme `docs/SDD_grasp_perfection.md` v1.0.0 §4–§7. Reutilizado por
`manual_control_node.py` (GUI step-by-step) e por `grasp_executor.py`
(ciclo autônomo). Não depende de ROS — apenas de `math` e `numpy` (via
`kinematics`).

Para qualquer mudança em pose/grasp, atualize antes o SDD e referencie
a seção `§4` no commit.
"""

from __future__ import annotations

import math


# ── Conjunto de juntas ────────────────────────────────────────────────
ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')
HAND_JOINTS = ('Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate')

ARM_LIMITS_DEG = {
    'joint1': (-180, 180), 'joint2': (-260,  80),
    'joint3': (-135, 135), 'joint4': (-260,  80),
    'joint5': (-135, 135), 'joint6': (-360, 360),
}

# Limites factíveis — espelham hand_pack.urdf_helpers.HAND_DRIVER_LIMITS.
MAX_RAD = {'Thumb': 1.0, 'Index': 1.0, 'Middle': 1.0,
           'Ring':  1.0, 'Little': 1.0, 'Rotate': 1.0}


# ── Presets de braço (poses-base) ─────────────────────────────────────
ARM_PRESETS_BASE = {
    'Home':      {'joint1':   0, 'joint2':   0, 'joint3':  90,
                  'joint4': -90, 'joint5': -90, 'joint6':   0},
    'Vertical':  {'joint1':   0, 'joint2':   0, 'joint3':   0,
                  'joint4': -90, 'joint5': -90, 'joint6':   0},
    'Estendido': {'joint1':   0, 'joint2':   0, 'joint3': -90,
                  'joint4':   0, 'joint5':   0, 'joint6':   0},
}


# Orientações canônicas de TCP (R_tcp 3x3, world frame). Convenção da
# cadeia: TCP_z = direção dos dedos (hand_y), TCP_y = -palm front,
# TCP_x = largura da palma (hand_x).
#   Palm-Down: palma normal para −Z (encara o objeto por cima), dedos em
#     +Y — usado pelo frasco/ampola (palma sobrepõe o topo do objeto).
#   Lateral-Claw: palma normal para +Y, dedos para −Z, de modo que ao
#     fechar cruzam o eixo do cilindro vertical e o abraçam pelo lado —
#     usado pelo tubo (ver images/claw.png).
def _Rtcp_palm_down(finger_dir: tuple = (0.0, 1.0, 0.0)):
    """R_tcp com palma normal = −Z mundo e dedos em `finger_dir`.
    Retorna numpy.array 3x3.
    """
    import numpy as np
    z = np.asarray(finger_dir, float); z /= np.linalg.norm(z) + 1e-12
    # palm-front = -TCP_y → TCP_y = -palm_dir; palm_dir = (0,0,-1)
    y = np.array([0.0, 0.0, 1.0])
    # Ortogonaliza y em relação a z
    y = y - z * float(np.dot(y, z)); y /= np.linalg.norm(y) + 1e-12
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


# CONVENÇÃO TCP POR TIPO DE GRIP: T_HAND_ATTACH define TCP = ponto de
# convergência dos fingertips = hand_base_link + 0.115·finger_dir. A palma
# física fica ENTRE o flange e o TCP (hand_base +45mm = palm_center, +70mm
# = TCP). Cada tipo de grip precisa de um alvo diferente para a IK:
#   PALM grip (frasco): a PALMA é a área de contato. Target = posição da
#     palma; convertida para TCP via TCP = palm + 0.070·finger_dir.
#   FINGERTIP grip (ampola): os fingertips são a área de contato — o
#     target IK é a posição exata do objeto.
#   CLAW grip (tubo): fingertips abraçam o objeto pela lateral — target IK
#     é a posição do objeto, com a palma 115mm atrás na direção do approach.
_FINGER_DIR_FRASCO = (1.0, 0.0, 0.0)   # top-down palm grip, dedos em +X
_FINGER_DIR_AMPOLA = (1.0, 0.0, 0.0)   # fingertip pinch top-down

# Offset físico palma→fingertip convergence (m). hand_base→TCP = 0.115
# inclui ~45mm de palma + ~70mm de fingertip extension; o centro útil
# da palma está em hand_base + 0.045·hand_y, então palm→TCP = 0.070.
_PALM_TO_TCP_OFFSET: float = 0.070

# Tipo de cada grip (informativo + usado em compute_targets)
GRIP_TYPE = {
    'frasco': 'palm',       # palma toca o objeto
    'ampola': 'fingertip',  # fingertips (pinça) tocam o objeto
    'tubo':   'claw',       # fingertips (claw) tocam o objeto
}


def _build_rtcp_by_obj():
    """R_tcp por objeto. Lateral para frasco/tubo (wrap horizontal em
    torno do cilindro vertical); top-down pinch para ampola."""
    try:
        import numpy as np
        from .kinematics import approach_to_Rtcp
    except Exception:
        return {}
    return {
        # Frasco — TOP-DOWN palm grip: palma horizontal sobre o frasco,
        # dedos extendem em +X e curvam para baixo abraçando a parte
        # superior do cilindro.
        'frasco': _Rtcp_palm_down(finger_dir=_FINGER_DIR_FRASCO),
        'ampola': _Rtcp_palm_down(finger_dir=_FINGER_DIR_AMPOLA),
        # Tubo — mantido lateral (cilindro vertical alto 120mm — top-down
        # exigiria descida muito grande). Lateral claw aborda em +Y.
        'tubo':   approach_to_Rtcp(np.array([0.0, 1.0, 0.0])),
        '_finger_dir': {
            'frasco': _FINGER_DIR_FRASCO,
            'ampola': _FINGER_DIR_AMPOLA,
        },
    }


try:
    RTCP_BY_OBJ = _build_rtcp_by_obj()
except Exception:
    RTCP_BY_OBJ = {}


# Legado: vetores de approach (motion direction, NÃO orientação de TCP)
# Mantido para compatibilidade com qualquer chamador que ainda use
# `solve_pose(p, q_seed, approach_vec)` com a convenção antiga
# (`approach_to_Rtcp`).
APPROACH_VEC_BY_OBJ: dict[str, tuple] = {
    'frasco': (0.0, 0.0, -1.0),
    'ampola': (0.0, 0.0, -1.0),
    'tubo':   (0.0, 1.0,  0.0),
}

# ── Alvos de TCP (world frame, metros) ───────────────────────────────
# Frasco: bbox cz=0.851 sz=0.090 → topo z=0.896. Palma encosta no topo com
# pequeno offset (palm thickness 15 mm → TCP 5 mm acima do topo).
_OBJ_TOP_Z: dict[str, float] = {
    'frasco': 0.851 + 0.090 / 2,  # 0.896
    'ampola': 0.844 + 0.075 / 2,  # 0.881
    'tubo':   0.866 + 0.120 / 2,  # 0.926 (lateral grasp — não é descida)
}
_PALM_CLEARANCE: float = 0.020   # 20 mm — palma 2 cm acima do topo
_PINCH_CLEARANCE: float = 0.000  # 0 mm — pinça TOCA o topo (objetivo)

# Alvos da PALMA / TCP por objeto. Para palm-grip (frasco) usamos a posição
# da PALMA física (não do flange/joint6 nem do TCP).
_FRASCO_PALM_Z_DESIRED: float = max(0.920,
                                     _OBJ_TOP_Z['frasco'] + _PALM_CLEARANCE)

PRE_APPROACH_TCP_WORLD: dict[str, tuple] = {
    'frasco': (0.0, 0.0, 0.0),  # preenchidos por _refresh_tcp_from_grip_type()
    'ampola': (0.0, 0.0, 0.0),
    'tubo':   (0.0, 0.0, 0.0),
}

APPROACH_TCP_WORLD_BY_OBJ: dict[str, tuple] = {
    'frasco': (0.0, 0.0, 0.0),
    'ampola': (0.0, 0.0, 0.0),
    'tubo':   (0.0, 0.0, 0.0),
}

PICK_TCP_WORLD: dict[str, tuple] = {
    'frasco': (0.0, 0.0, 0.0),
    'ampola': (0.0, 0.0, 0.0),
    'tubo':   (0.0, 0.0, 0.0),
}


def _refresh_tcp_from_grip_type():
    """Recalcula TCP world por objeto a partir do GRASP_CENTER atual."""
    try:
        import numpy as np
        from .kinematics import HAND_CONFIGS, grasp_center_in_hand
        from .collision import PICK_OBJ_BBOX
    except Exception:
        return

    if not isinstance(RTCP_BY_OBJ, dict) or not RTCP_BY_OBJ:
        return

    # Rx(π/2) — converte TCP frame → hand_base_link frame (T_HAND_ATTACH
    # mais a rotação fixa do URDF Link6 → hand_base_link).
    R_HAND_FROM_TCP = np.array([[1.0, 0.0,  0.0],
                                 [0.0, 0.0, -1.0],
                                 [0.0, 1.0,  0.0]], dtype=float)

    _GRIP_KIND   = {'frasco': 'palm', 'ampola': 'fingertip', 'tubo': 'claw'}
    _HAND_CFGKEY = {'frasco': 'palm_grip',
                    'ampola': 'fingertip_grip',
                    'tubo':   'claw_grip'}

    # Offsets de approach: top-down sobe em +Z, claw lateral recua em
    _APPR_DZ = 0.060
    _PRE_DZ  = 0.180
    _APPR_DLAT = 0.120
    _PRE_DLAT  = 0.200

    for obj in ('frasco', 'ampola', 'tubo'):
        if obj not in PICK_OBJ_BBOX or obj not in RTCP_BY_OBJ:
            continue
        cx, cy, cz, *_ = PICK_OBJ_BBOX[obj]
        obj_pos = np.array([cx, cy, cz], dtype=float)
        gc_hand = grasp_center_in_hand(HAND_CONFIGS[_HAND_CFGKEY[obj]],
                                        _GRIP_KIND[obj])
        R_tcp  = np.asarray(RTCP_BY_OBJ[obj], dtype=float)
        R_hand = R_tcp @ R_HAND_FROM_TCP
        hand_base = obj_pos - R_hand @ gc_hand
        finger_dir_w = R_hand[:, 1]      # hand_y no mundo
        tcp_pick = hand_base + 0.115 * finger_dir_w

        if obj == 'tubo':
            tcp_appr = tcp_pick - _APPR_DLAT * finger_dir_w
            tcp_pre  = tcp_pick - _PRE_DLAT  * finger_dir_w
        else:
            up = np.array([0.0, 0.0, 1.0])
            tcp_appr = tcp_pick + _APPR_DZ * up
            tcp_pre  = tcp_pick + _PRE_DZ  * up

        PICK_TCP_WORLD[obj]            = tuple(float(round(v, 4)) for v in tcp_pick)
        APPROACH_TCP_WORLD_BY_OBJ[obj] = tuple(float(round(v, 4)) for v in tcp_appr)
        PRE_APPROACH_TCP_WORLD[obj]    = tuple(float(round(v, 4)) for v in tcp_pre)


_refresh_tcp_from_grip_type()

# Caixas de entrega
DELIVERY_XY_WORLD = {
    'frasco': (-0.05, 0.65),
    'tubo':   ( 0.25, 0.65),
    'ampola': ( 0.55, 0.65),
}
DELIVERY_Z_WORLD = 0.75


# ── Fallbacks (graus) — usados se o IK numérico não estiver disponível ─
_FALLBACK_TOP_DOWN_SEED = {
    'joint1':   0.0, 'joint2': -30.0, 'joint3': -90.0,
    'joint4':   0.0, 'joint5': -90.0, 'joint6':   0.0,
}
_FALLBACK_LATERAL_SEED = {
    'joint1':  +2.5, 'joint2': -21.0, 'joint3':  -84.9,
    'joint4':  -74.1, 'joint5': -177.5, 'joint6': +90.0,
}
FALLBACK_PRE_APPROACH_DEG: dict[str, dict] = {
    'frasco': dict(_FALLBACK_TOP_DOWN_SEED),
    'ampola': dict(_FALLBACK_TOP_DOWN_SEED),
    'tubo':   dict(_FALLBACK_LATERAL_SEED),
}
FALLBACK_APPROACH_DEG_BY_OBJ: dict[str, dict] = {
    'frasco': dict(_FALLBACK_TOP_DOWN_SEED),
    'ampola': dict(_FALLBACK_TOP_DOWN_SEED),
    'tubo':   dict(_FALLBACK_LATERAL_SEED),
}
FALLBACK_PICK_DEG: dict[str, dict] = {
    'frasco': dict(_FALLBACK_TOP_DOWN_SEED),
    'ampola': dict(_FALLBACK_TOP_DOWN_SEED),
    'tubo':   dict(_FALLBACK_LATERAL_SEED),
}
FALLBACK_DELIVERY_DEG = {
    'frasco': {'joint1': +108.4, 'joint2': -19.6, 'joint3': -91.3,
               'joint4':  -69.1, 'joint5': +108.4, 'joint6':  +0.0},
    'tubo':   {'joint1':  +85.6, 'joint2': -23.2, 'joint3': -86.3,
               'joint4':  -70.5, 'joint5':  +85.6, 'joint6':  +0.0},
    'ampola': {'joint1':  +65.8, 'joint2': -36.5, 'joint3': -65.6,
               'joint4':  -77.9, 'joint5':  +65.8, 'joint6':  +0.0},
}


# ── Obstáculos tolerados no instante do PICK, por objeto ─────────────
#   • belt_surface: laje de 6 mm onde o objeto repousa — o flange chega
#     forçosamente a esse nível para pegá-lo.
#   • belt_frame (só tubo): no conveyor_cell.world o corpo da esteira é
#     APENAS VISUAL (sem <collision>; a colisão real está nos pés
#     belt_leg_*). O grasp lateral do tubo aproxima o punho a ~1 mm do
#     AABB desse visual; tratá-lo como sólido bloqueava o único ramo IK
#     que coloca o grasp_center no centro do tubo (a pose antiga errava
#     o centro por ~73 mm e os dedos nunca tocavam o cilindro).
PICK_SKIP_OBSTACLES: dict[str, set] = {
    'frasco': {'belt_surface', 'belt_frame'},
    'ampola': {'belt_surface', 'belt_frame'},
    'tubo':   {'belt_surface', 'belt_frame'},
}


# ── Configurações de preensão (SDD §4) ────────────────────────────────
HAND_OPEN = {j: 0 for j in HAND_JOINTS}

# HAND_GRIPS derivados de kinematics.HAND_CONFIGS (fonte de verdade do
# projeto). Conversão: slider = rad / MAX_RAD * 200, com MAX_RAD=1.6
# (dedos) e 1.0 (Rotate). Cada grip tem assinatura geométrica distinta:
#
#   palm_grip      — envoltura palmar — TODOS os dedos altos (138-156)
#                    + Rotate baixo (50) — polegar no lado oposto.
#   claw_grip      — preensão em garra — dedos médios (113-138) +
#                    Rotate médio (90) — semi-fechamento conformante.
#   fingertip_grip — pinça precisa — APENAS polegar+indicador (~100) +
#                    Rotate alto (164) — médio/anelar/mínimo NÃO usados.
HAND_GRIPS = {
    'Open': HAND_OPEN,
    # Slider 200 = curl máximo (1.6 rad). 138 / 156 / 138 (anteriores)
    # deixavam fingertips ~57mm do eixo do cilindro — fora do frasco
    # (r=45mm). Subimos a 185-195 para envolverem a geometria.
    'Palm Grip (frasco)':      {'Thumb': 185, 'Index': 195, 'Middle': 195,
                                 'Ring':  190, 'Little': 185, 'Rotate':  50},
    # Claw e Fingertip: posições MÁXIMAS (slider 0..200) — o
    # PerfectGrasp interrompe o fechamento antes via lag-detection
    # quando o dedo encontra o objeto, então estes valores definem o
    # envelope de contato (não o aperto final).
    'Claw Grip (tubo)':        {'Thumb':  75, 'Index':  75, 'Middle':  80,
                                 'Ring':   82, 'Little':  87, 'Rotate': 200},
    'Fingertip Grip (ampola)': {'Thumb': 104, 'Index':  62, 'Middle':   0,
                                 'Ring':    0, 'Little':   0, 'Rotate': 146},
}

# Pré-shape = aprox. 50% de cada grip final. Mantém a assinatura
# distintiva (palm/claw/fingertip) mas com dedos pré-curvados o
# suficiente para não varrer o objeto durante a descida.
# Cup-shape para descida: fingertips ficam ABAIXO do topo do objeto MAS
# fora da projeção horizontal. Validado numericamente para canonical (0.75, 0, z).
# Notas:
#   - Thumb em palm/frasco fica menos curlado (slider 30 vs 70) para
#     não penetrar a parede +Y do frasco durante a descida.
#   - Rotate alto em fingertip (=35 final, =20 preshape) abre a mão para
#     que polegar e indicador convirjam SOBRE a ampola na fase final.
#   - Palm/frasco: Index/Middle/Ring/Little reduzidos (80/80/75/70 →
#     60/60/55/55) — com o curl antigo os fingertips desciam a r≈40-42mm
#     do eixo do frasco (raio real 42mm + pele 3mm) e varriam a borda
#     superior do cilindro durante TODA a descida APPROACH→PICK,
#     empurrando o objeto antes do fechamento. Com 60/60/55/55 o pior
#     dedo passa a r≥47mm (folga ≥2mm além da pele) e continua ≥68mm
#     ABAIXO do topo do frasco no PICK (cage preservado).
HAND_PRESHAPE = {
    'Palm Grip (frasco)':      {'Thumb':  30, 'Index':  60, 'Middle':  60,
                                 'Ring':   55, 'Little':  55, 'Rotate':  25},
    #   - Claw/tubo: dedos ABERTOS + Rotate já na oposição final (200).
    #     O engajamento do tubo é LATERAL (translação horizontal até o
    #     PICK): com qualquer curl os fingertips varriam a faixa de
    #     altura do tubo e o empurravam (pior caso −13 mm de
    #     interferência). Com a mão aberta o conjunto passa a +16 mm da
    #     superfície e, na pose final, polegar (r=24 mm) e dedos
    #     (r=38-56 mm) engaiolam o cilindro; o PerfectGrasp fecha a
    #     partir do aberto com contato em 4/5 dedos.
    'Claw Grip (tubo)':        {'Thumb':   0, 'Index':   0, 'Middle':   0,
                                 'Ring':    0, 'Little':   0, 'Rotate': 200},
    'Fingertip Grip (ampola)': {'Thumb':  15, 'Index':  50, 'Middle':   0,
                                 'Ring':    0, 'Little':   0, 'Rotate':  20},
}

# ── Poses extras (gestos / formas úteis) ──────────────────────────────
# Valores 0–200 por dedo (0=aberto/relaxado, 200=curl máximo); Rotate segue
# a mesma escala (0=polegar fora, 200=polegar atravessando a palma).
HAND_EXTRA_POSES = {
    'Punho':         {'Thumb': 200, 'Index': 200, 'Middle': 200,
                      'Ring':  200, 'Little': 200, 'Rotate':  60},
    'Apontar':       {'Thumb': 170, 'Index':   0, 'Middle': 195,
                      'Ring':  195, 'Little': 185, 'Rotate':  55},
    'Paz (V)':       {'Thumb': 180, 'Index':   0, 'Middle':   0,
                      'Ring':  195, 'Little': 195, 'Rotate':  60},
    'Joinha':        {'Thumb':   0, 'Index': 195, 'Middle': 195,
                      'Ring':  195, 'Little': 195, 'Rotate':   0},
    'Rock':          {'Thumb':  40, 'Index':   0, 'Middle': 200,
                      'Ring':  200, 'Little':   0, 'Rotate':  30},
    'Pistola':       {'Thumb':   0, 'Index':   0, 'Middle': 195,
                      'Ring':  195, 'Little': 185, 'Rotate':  95},
    'Gancho':        {'Thumb':   0, 'Index': 140, 'Middle': 140,
                      'Ring':  140, 'Little': 130, 'Rotate':   0},
    'Concha':        {'Thumb': 110, 'Index': 120, 'Middle': 120,
                      'Ring':  115, 'Little': 110, 'Rotate':  80},
    'Garra Aberta':  {'Thumb':  70, 'Index':  80, 'Middle':  80,
                      'Ring':   75, 'Little':  70, 'Rotate':  90},
    'Contar 3':      {'Thumb': 180, 'Index':   0, 'Middle':   0,
                      'Ring':    0, 'Little': 195, 'Rotate':  50},
}


OBJ_GRIP = {
    'frasco': 'Palm Grip (frasco)',
    'tubo':   'Claw Grip (tubo)',
    'ampola': 'Fingertip Grip (ampola)',
}

OBJ_APPROACH_DESC = {
    'frasco': 'Top-down — palma horizontal envolve o topo do frasco',
    'ampola': 'Top-down — pinça polegar+indicador sobre a ampola',
    'tubo':   'Lateral — claw frontal, mão virada (ver claw.png)',
}

# Nomes das 6 fases do ciclo de pick (SDD §6)
PHASE_NAMES = ('F1 pre-appr', 'F2 approach', 'F3 preshape',
               'F4 descend',  'F5 close',    'F6 lift')


# ── IK runtime ────────────────────────────────────────────────────────
def solve_pose(p_world, q_seed_deg, approach_vec=(0.0, 0.0, -1.0)):
    """Resolve IK para TCP em world frame com a abordagem dada. Retorna
    dict de graus por junta, ou None se IK não disponível / não convergir.
    """
    try:
        import numpy as np
        from .kinematics import (
            inverse_kinematics, forward_kinematics, approach_to_Rtcp,
            _ik_refine,
        )
    except Exception:
        return None

    base_z = 0.405
    p_base = np.array([p_world[0], p_world[1], p_world[2] - base_z])
    approach = np.array(approach_vec, dtype=float)
    q_seed = np.array([math.radians(q_seed_deg[j]) for j in ARM_JOINTS])

    q_global, ok = inverse_kinematics(
        p_base, approach, q_seed=q_seed, elbow_up=True)
    branch_ok = (ok
                 and abs(math.degrees(q_global[1])
                          - math.degrees(q_seed[1])) < 60.0
                 and abs(math.degrees(q_global[0])
                          - math.degrees(q_seed[0])) < 60.0)
    if branch_ok:
        q_final = q_global
    else:
        R_tcp = approach_to_Rtcp(approach)
        q_final, _ = _ik_refine(p_base, R_tcp, q_seed)

    T = forward_kinematics(q_final)
    err_mm = float(np.linalg.norm(T[:3, 3] - p_base)) * 1000.0
    if err_mm > 5.0:
        return None
    return {j: float(math.degrees(q_final[i])) for i, j in enumerate(ARM_JOINTS)}


def solve_pose_R(p_world, R_tcp, q_seed_deg, *,
                  check_collision: bool = True,
                  allow_object: str | None = None,
                  collision_margin: float = 0.005,
                  skip_obstacles: set | None = None):
    """Resolve IK com R_tcp 3x3 explícito (orientação completa do TCP).

    Args:
        p_world: TCP em world frame (3,)
        R_tcp:   matriz 3x3 da orientação do TCP em world frame
        q_seed_deg: dict {jointN: graus} usado como semente
        check_collision: se True, filtra ramos IK que colidem com o
            mundo (esteira, paredes, caixas, câmera, prateleira).
        allow_object: nome do objeto picável tolerado no toque.
        collision_margin: folga (m) entre cada elo e os obstáculos.
    Retorna dict graus por junta ou None se nenhum ramo IK satisfizer
    posição + orientação + colisão.
    """
    try:
        import numpy as np
        from .kinematics import (
            _ik_candidates, _ik_refine, forward_kinematics, _rot_error,
        )
        from .collision import pose_is_safe
    except Exception:
        return None

    base_z = 0.405
    p_base = np.array([p_world[0], p_world[1], p_world[2] - base_z])
    R = np.asarray(R_tcp, float)
    q_seed = np.array([math.radians(q_seed_deg[j]) for j in ARM_JOINTS])

    # Coletamos TODOS os candidatos viáveis (geom + IK) e só depois
    # escolhemos o melhor — assim podemos rejeitar ramos colidentes.
    viable: list[tuple[float, "np.ndarray", float, float, bool]] = []
    for cand in _ik_candidates(p_base, R, q_seed=q_seed, elbow_up=True):
        q, _ok = _ik_refine(p_base, R, cand)
        T = forward_kinematics(q)
        err_p = float(np.linalg.norm(T[:3, 3] - p_base))
        err_o = float(np.linalg.norm(_rot_error(T[:3, :3], R)))
        if err_p > 0.010 or err_o > 0.25:
            continue
        # Penalidades adicionais:
        #   • seed_dev: cascata pre→appr→pick consistente (mesmo ramo)
        #   • joint2 extremo: punição para soluções tipo "elbow swings
        #     for trás" (q2 < -100° ou > 60°)
        seed_dev = float(np.linalg.norm(q - q_seed))
        q2_deg = math.degrees(q[1])
        q2_pen = 0.0
        if q2_deg < -100.0:
            q2_pen = (abs(q2_deg) - 100.0) / 30.0
        elif q2_deg > 60.0:
            q2_pen = (q2_deg - 60.0) / 30.0

        # Penalidade extra para joint1 longe de zero quando o TCP está
        # centrado em Y=0 (frasco/ampola).
        q1_deg = math.degrees(q[0])
        q1_pen = 0.0
        # Detectar TCP centrado (target_y do chamador) — usamos p_world
        target_y = float(p_world[1])
        if abs(target_y) < 0.05:  # alvo essencialmente em Y=0
            q1_pen = max(0.0, (abs(q1_deg) - 5.0) / 15.0)

        safe = True
        if check_collision:
            safe, _ = pose_is_safe(q, margin_arm=collision_margin + 0.005,
                                     margin_wrist=collision_margin,
                                     skip=skip_obstacles)

        score = (err_p + 0.10 * err_o
                 + 0.02 * seed_dev
                 + 0.50 * q2_pen
                 + 1.00 * q1_pen
                 + (0.0 if safe else 100.0))
        viable.append((score, q, err_p, err_o, safe))

    if not viable:
        return None

    # 1ª opção: candidato com menor score E livre de colisão.
    viable.sort(key=lambda t: t[0])
    chosen = next((t for t in viable if t[4]), None)
    if chosen is None:
        # Nenhum ramo livre. Se não exigíamos colisão, devolve o melhor;
        # se exigíamos, devolve None (chamador deve relaxar margem ou
        # mover o objetivo).
        if not check_collision:
            chosen = viable[0]
        else:
            return None

    _, q, _, _, _ = chosen
    return {j: float(math.degrees(q[i])) for i, j in enumerate(ARM_JOINTS)}


def recenter_tcp_targets(obj_class: str, obj_world):
    """TCPs de PRE/APPR/PICK recentrados na posição REAL do objeto.

    Retorna (tcp_pre, tcp_appr, tcp_pick) como tuplas, ou None.
    """
    try:
        import numpy as np
        from .collision import PICK_OBJ_BBOX
    except Exception:
        return None
    if obj_class not in PICK_OBJ_BBOX:
        return None
    ref = np.array(PICK_OBJ_BBOX[obj_class][:3], dtype=float)
    obj = np.asarray(obj_world, dtype=float)
    out = []
    for src in (PRE_APPROACH_TCP_WORLD, APPROACH_TCP_WORLD_BY_OBJ,
                PICK_TCP_WORLD):
        out.append(tuple(float(v) for v in obj + (np.array(src[obj_class]) - ref)))
    return tuple(out)


def _harmonize_branch_deg(sol: dict, ref: dict) -> dict:
    """Aproxima `sol` de `ref` entre ramos EQUIVALENTES de punho."""
    import numpy as np
    from .kinematics import forward_kinematics, _rot_error

    def _wrap360(d: dict) -> dict:
        out = {}
        for j, v in d.items():
            best = v
            for cand in (v - 360.0, v + 360.0):
                if abs(cand - ref[j]) < abs(best - ref[j]) and abs(cand) < 355.0:
                    best = cand
            out[j] = best
        return out

    def _dist(d: dict) -> float:
        return max(abs(d[j] - ref[j]) for j in ARM_JOINTS)

    def _q(d: dict):
        return np.array([math.radians(d[j]) for j in ARM_JOINTS])

    candidates = [_wrap360(sol)]
    # Espelho de punho: q4±180, −q5, q6±180 (mantendo |ângulo|<355°)
    for s4 in (+180.0, -180.0):
        for s6 in (+180.0, -180.0):
            m = dict(sol)
            m['joint4'] = sol['joint4'] + s4
            m['joint5'] = -sol['joint5']
            m['joint6'] = sol['joint6'] + s6
            if any(abs(m[j]) > 355.0 for j in ('joint4', 'joint5', 'joint6')):
                continue
            candidates.append(_wrap360(m))

    T_ref = forward_kinematics(_q(sol))
    best = candidates[0]
    for cand in candidates[1:]:
        if _dist(cand) >= _dist(best):
            continue
        T = forward_kinematics(_q(cand))
        if (float(np.linalg.norm(T[:3, 3] - T_ref[:3, 3])) < 1e-3 and
                float(np.linalg.norm(_rot_error(T[:3, :3], T_ref[:3, :3]))) < 0.02):
            best = cand
    return best


def solve_grasp_poses_at_world(obj_class: str, obj_world, *,
                                relax: bool = True):
    """Resolve PRE/APPR/PICK (dicts em graus) na posição REAL do objeto.

    Args:
        obj_world: (x, y, z) do CENTRO do objeto em world frame.
        relax: se o PICK não tiver ramo collision-free, aceita o melhor
            ramo SEM checagem — errar o objeto é pior que raspar a
            margem de um obstáculo já tolerado.
    Retorna (pre, appr, pick) ou (None, None, None) se IK indisponível.
    """
    tcps = recenter_tcp_targets(obj_class, obj_world)
    if tcps is None or obj_class not in RTCP_BY_OBJ:
        return (None, None, None)
    tcp_pre, tcp_appr, tcp_pick = tcps
    R = RTCP_BY_OBJ[obj_class]
    skip = PICK_SKIP_OBSTACLES.get(obj_class, {'belt_surface'})

    pick = solve_pose_R(tcp_pick, R, PICK_POSES_DEG[obj_class],
                        check_collision=True, allow_object=obj_class,
                        collision_margin=0.003, skip_obstacles=skip)
    if pick is None and relax:
        pick = solve_pose_R(tcp_pick, R, PICK_POSES_DEG[obj_class],
                            check_collision=False)
    if pick is None:
        return (None, None, None)

    appr = solve_pose_R(tcp_appr, R, pick, check_collision=False)
    if appr is None:
        return (None, None, None)
    appr = _harmonize_branch_deg(appr, pick)

    pre = solve_pose_R(tcp_pre, R, appr, check_collision=False)
    if pre is None:
        return (None, None, None)
    pre = _harmonize_branch_deg(pre, appr)

    return (pre, appr, pick)


def compute_targets():
    """Resolve via IK (com fallbacks) as poses por objeto."""
    pre, appr, pick = {}, {}, {}
    for obj in ('frasco', 'ampola', 'tubo'):
        R = RTCP_BY_OBJ.get(obj)
        seed_pre  = FALLBACK_PRE_APPROACH_DEG[obj]
        seed_appr = FALLBACK_APPROACH_DEG_BY_OBJ[obj]
        seed_pick = FALLBACK_PICK_DEG[obj]
        skip_pick = PICK_SKIP_OBSTACLES[obj]

        if R is not None and obj == 'tubo':
            # TUBO (claw lateral): cascata PICK-primeiro.
            pick[obj] = (solve_pose_R(PICK_TCP_WORLD[obj], R, seed_pick,
                                        check_collision=True,
                                        allow_object=obj,
                                        collision_margin=0.003,
                                        skip_obstacles=skip_pick)
                         or solve_pose_R(PICK_TCP_WORLD[obj], R, seed_pick,
                                          check_collision=False)
                         or seed_pick)
            appr[obj] = (solve_pose_R(APPROACH_TCP_WORLD_BY_OBJ[obj], R,
                                        pick[obj], check_collision=True,
                                        skip_obstacles=skip_pick)
                         or pick[obj])
            pre[obj]  = (solve_pose_R(PRE_APPROACH_TCP_WORLD[obj], R,
                                        appr[obj], check_collision=True,
                                        skip_obstacles=skip_pick)
                         or appr[obj])
        elif R is not None:
            # Estratégia: resolve APPROACH primeiro (TCP mid-height,
            # ramo IK mais estável), depois usa appr[obj] como semente
            # de PRE e PICK — garante ramo consistente nas 3 fases.
            appr[obj] = (solve_pose_R(APPROACH_TCP_WORLD_BY_OBJ[obj], R,
                                        seed_appr, check_collision=True)
                         or solve_pose_R(APPROACH_TCP_WORLD_BY_OBJ[obj], R,
                                          seed_appr, check_collision=False)
                         or seed_appr)
            pre[obj] = (solve_pose_R(PRE_APPROACH_TCP_WORLD[obj], R,
                                       appr[obj], check_collision=True)
                        or solve_pose_R(PRE_APPROACH_TCP_WORLD[obj], R,
                                         appr[obj], check_collision=False)
                        or appr[obj])
            # Pick: a mão TOCA o objeto picável (frasco/ampola/tubo), mas
            # nenhum link deve tocar a esteira/caixas/parede.
            pick[obj] = (solve_pose_R(PICK_TCP_WORLD[obj], R, appr[obj],
                                        check_collision=True,
                                        allow_object=obj,
                                        collision_margin=0.003,
                                        skip_obstacles=skip_pick)
                         or solve_pose_R(PICK_TCP_WORLD[obj], R, appr[obj],
                                          check_collision=False)
                         or appr[obj])
        else:
            # Fallback: numpy/kinematics ausentes — usa seeds-base
            pre[obj]  = seed_pre
            appr[obj] = seed_appr
            pick[obj] = seed_pick

    delivery = {}
    for obj in ('frasco', 'tubo', 'ampola'):
        dx, dy = DELIVERY_XY_WORLD[obj]
        delivery[obj] = (solve_pose((dx, dy, DELIVERY_Z_WORLD),
                                     q_seed_deg=FALLBACK_DELIVERY_DEG[obj],
                                     approach_vec=(0.0, 0.0, -1.0))
                         or FALLBACK_DELIVERY_DEG[obj])
    return pre, appr, pick, delivery


# ── Cache hardcoded de poses (resolvido off-line por compute_targets) ─
# Para evitar 10s de IK em cada import. Re-gerar em runtime definindo
# GRASP_RECOMPUTE_POSES=1 no ambiente (ver bloco de resolução abaixo).
_CACHED_PRE_APPROACH_POSES_DEG = {
    'frasco': {'joint1':  21.2071, 'joint2':   8.3155, 'joint3': -100.0195,
               'joint4': -88.2957, 'joint5': -68.7932, 'joint6':  -0.0001},
    'ampola': {'joint1':  19.8765, 'joint2':  12.6962, 'joint3': -101.1162,
               'joint4': -91.5796, 'joint5': -70.1238, 'joint6':  -0.0001},
    'tubo':   {'joint1': -18.7403, 'joint2': -34.8821, 'joint3':  -66.0045,
               'joint4': -79.1134, 'joint5': 161.2597, 'joint6':  90.0000},
}
_CACHED_APPROACH_POSES_DEG = {
    'frasco': {'joint1':  21.2090, 'joint2':   6.7135, 'joint3': -110.9968,
               'joint4': -75.7163, 'joint5': -68.7912, 'joint6':  -0.0002},
    'ampola': {'joint1':  19.8808, 'joint2':  11.7286, 'joint3': -112.4787,
               'joint4': -79.2495, 'joint5': -70.1196, 'joint6':  -0.0001},
    'tubo':   {'joint1': -13.8222, 'joint2': -31.2261, 'joint3':  -72.1125,
               'joint4': -76.6613, 'joint5': 166.1778, 'joint6':  90.0000},
}
_CACHED_PICK_POSES_DEG = {
    'frasco': {'joint1':  21.2084, 'joint2':   4.8444, 'joint3': -115.6019,
               'joint4': -69.2421, 'joint5': -68.7920, 'joint6':  -0.0001},
    'ampola': {'joint1':  19.8810, 'joint2':  10.2150, 'joint3': -117.5355,
               'joint4': -72.6792, 'joint5': -70.1194, 'joint6':  -0.0001},
    'tubo':   {'joint1':  -5.7597, 'joint2': -27.0708, 'joint3':  -78.2997,
               'joint4': -74.6294, 'joint5': 174.2403, 'joint6':  90.0000},
}
# DELIVERY regenerado 2026-07-05: as poses antigas eram pré-acoplador
# (T_HAND_ATTACH 0.115 m) — com o acoplador (+0.05546 m) o TCP real caía 171
# mm AQUÉM da caixa em Y e 75 mm alto: o objeto era solto na borda da
# prateleira, não dentro da caixa.
_CACHED_DELIVERY_POSES_DEG = {
    'frasco': {'joint1': 111.43, 'joint2': -18.54, 'joint3':  -84.44,
               'joint4': -167.02, 'joint5': -90.00, 'joint6': -158.57},
    'tubo':   {'joint1':  84.88, 'joint2':  -0.12, 'joint3': -106.13,
               'joint4':  16.25, 'joint5':  90.00, 'joint6':   -5.12},
    'ampola': {'joint1':  62.73, 'joint2': -14.90, 'joint3':  -89.21,
               'joint4':  14.11, 'joint5':  90.00, 'joint6':  -27.27},
}


# Resolução em tempo de import — usa cache hardcoded para evitar IK pesada.
import os as _os
if _os.environ.get('GRASP_RECOMPUTE_POSES'):
    (PRE_APPROACH_POSES_DEG, APPROACH_POSES_DEG,
     PICK_POSES_DEG, DELIVERY_POSES_DEG) = compute_targets()
else:
    PRE_APPROACH_POSES_DEG = dict(_CACHED_PRE_APPROACH_POSES_DEG)
    APPROACH_POSES_DEG     = dict(_CACHED_APPROACH_POSES_DEG)
    PICK_POSES_DEG         = dict(_CACHED_PICK_POSES_DEG)
    DELIVERY_POSES_DEG     = dict(_CACHED_DELIVERY_POSES_DEG)


# Compatibilidade com código legacy que esperava um único APPROACH_POSE_DEG
APPROACH_POSE_DEG = APPROACH_POSES_DEG['frasco']
