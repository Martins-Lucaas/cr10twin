"""Teleoperação da mão COVVI por visão computacional.

Captura a webcam USB local (OpenCV), rastreia UMA mão do usuário
(MediaPipe Hands) e traduz a pose dos dedos em **flexão normalizada
0–1 por junta** — as 6 juntas primárias que a GUI já conhece:

    Thumb  Index  Middle  Ring  Little  Rotate

`0.0` = dedo estendido (mão aberta), `1.0` = dedo totalmente fechado.
`Rotate` é a oposição do polegar (rotador COVVI): `0.0` polegar ao lado
do indicador, `1.0` polegar cruzado sobre a palma.

O consumidor (palpation_gui) converte 0–1 → graus com o seu próprio
`HAND_LIMITS_DEG` e injeta nos `hand_sliders`, reaproveitando todo o
caminho existente (escala ECI, speed_factor, debounce, mirror do sim).

O mesmo callback traz `ArmJ6` (chave em `ARM_KEYS`): alvo ABSOLUTO em
GRAUS para `joint6` do CR10, derivado da PRONAÇÃO/SUPINAÇÃO do punho
(palma ↔ dorso), a partir do normal da palma nos world-landmarks 3D.
Ver `wrist_pronation_j6_deg`. É a ÚNICA junta do braço que este módulo
comanda, e a GUI só usa quando a checkbox "→ real" está marcada.

Projeto: sem ROS, sem Tk. `cv2` e `mediapipe` são importados só em
`start()` — o módulo importa limpo em máquina sem eles, e a GUI degrada
para uma mensagem de erro. Ver `HandCameraTeleop.start`.

O loop de captura roda numa thread daemon própria. Ela NÃO abre janela
OpenCV (`imshow` fora da main thread trava no Linux): anota cada quadro e
o publica em `get_latest_frame()`, que a GUI Tk desenha no seu loop `after`.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable

import numpy as np

# Nomes das juntas na MESMA ordem/rotulagem que touch_pack.constants.HAND_JOINTS.
JOINTS = ('Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Rotate')

# ── Parâmetros de mapeamento (calibráveis) ────────────────────────────
# Ângulos em graus. "Estendido" ≈ ângulo grande na articulação; "fechado"
# ≈ ângulo pequeno. Os pares (aberto, fechado) definem a reta de
# normalização — foram ajustados em bancada com webcam frontal e
# provavelmente precisam de retoque no seu setup (iluminação/lente).
_PIP_OPEN_DEG, _PIP_CLOSED_DEG = 178.0, 55.0     # dobra média do dedo (PIP)
_MCP_OPEN_DEG, _MCP_CLOSED_DEG = 165.0, 95.0     # nó do dedo (MCP)
_THUMB_OPEN_DEG, _THUMB_CLOSED_DEG = 172.0, 108.0  # IP do polegar

# Pesos na combinação PIP+MCP para os 4 dedos longos.
_W_PIP, _W_MCP = 0.65, 0.35
# Polegar: metade ângulo do IP, metade "distância do polegar à palma".
_W_THUMB_ANGLE, _W_THUMB_DIST = 0.5, 0.5
_THUMB_DIST_OPEN, _THUMB_DIST_CLOSED = 0.95, 0.30   # |tip-indexMCP| / palma
# Rotate = oposição: distância polegar→MCP do mínimo, normalizada pela palma.
_ROTATE_DIST_OPEN, _ROTATE_DIST_CLOSED = 0.95, 0.40

# ── Filtro One-Euro da saída ─────────────────────────────────────────
# Ajuste "latência mínima": `min_cutoff` alto deixa o filtro quase
# transparente (só corta o ruído mais rápido); `beta` alto faz a saída
# colar no movimento. Baixe `min_cutoff`/`beta` se quiser mais firmeza.
_OE_MIN_CUTOFF = 3.5
_OE_BETA = 0.07
_OE_D_CUTOFF = 1.0
# Taxa de callbacks para a GUI (a GUI ainda tem debounce de 60 ms).
_EMIT_HZ = 30.0
# Só reporta se ALGUM dedo mudou mais que isto — evita puro desperdício
# com a mão totalmente parada; pequeno o bastante para não somar latência.
_EMIT_DEADBAND = 0.004
# Idem para joint6 do braço (graus).
_ARM_DEADBAND_DEG = 0.3
# Quadros consecutivos COM mão antes de começar a comandar — barra o
# "chute" de uma detecção espúria de 1 quadro.
_MIN_TRACK_FRAMES = 2
# Quadros seguidos sem leitura antes de abortar o loop com erro.
_MAX_READ_FAILS = 30

# ── joint6 do braço ← PRONAÇÃO/SUPINAÇÃO do punho (palma ↔ dorso) ────
# Do normal da palma nos world-landmarks 3D: palma de frente p/ a câmera
# → 0°, mão de perfil → ±90°, dorso de frente → ±180° (satura).
# `_J6_PRON_RANGE_DEG` é a pronação (±) que satura o meio-span de joint6.
# Inverta `_J6_SIGN` se girar num sentido mover a junta no outro.
_J6_CENTER_DEG, _J6_SPAN_DEG, _J6_SIGN = 0.0, 180.0, +1.0
_J6_PRON_RANGE_DEG = 90.0
# Filtro de joint6: um pouco mais firme (o braço amplifica ruído).
_ARM_OE_MIN_CUTOFF, _ARM_OE_BETA = 2.5, 0.05
# Chave da junta de braço no dict passado a on_curl.
ARM_KEYS = ('ArmJ6',)

# Índices dos 21 landmarks do MediaPipe Hands.
_WRIST = 0
_THUMB_CMC, _THUMB_MCP, _THUMB_IP, _THUMB_TIP = 1, 2, 3, 4
_FINGER_LM = {
    #        MCP  PIP  DIP  TIP
    'Index':  (5,  6,  7,  8),
    'Middle': (9, 10, 11, 12),
    'Ring':  (13, 14, 15, 16),
    'Little': (17, 18, 19, 20),
}
_INDEX_MCP, _PINKY_MCP = 5, 17


class CameraUnavailable(RuntimeError):
    """A câmera USB não pôde ser aberta / não entregou o primeiro quadro."""


def _angle_deg(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    """Ângulo ABC (no vértice B), em graus, no plano da imagem."""
    v1, v2 = a - b, c - b
    n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 180.0
    cosv = float(np.dot(v1, v2) / (n1 * n2))
    return float(np.degrees(np.arccos(max(-1.0, min(1.0, cosv)))))


def _lerp01(x: float, x_open: float, x_closed: float) -> float:
    """Normaliza x na reta [x_open → 0, x_closed → 1], saturando em 0..1."""
    if x_open == x_closed:
        return 0.0
    return max(0.0, min(1.0, (x_open - x) / (x_open - x_closed)))


class _OneEuro:
    """Filtro One-Euro escalar (Casiez et al. 2012): pouco lag no movimento,
    pouco jitter parado. `dt` real por amostra — o loop passa o intervalo
    medido entre quadros."""

    def __init__(self, x0: float, *, min_cutoff: float, beta: float,
                 d_cutoff: float) -> None:
        self._min_cutoff = min_cutoff
        self._beta = beta
        self._d_cutoff = d_cutoff
        self._x_prev = x0
        self._dx_prev = 0.0

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x: float, dt: float) -> float:
        if dt <= 0.0:
            dt = 1e-2
        dx = (x - self._x_prev) / dt
        a_d = self._alpha(self._d_cutoff, dt)
        dx_hat = a_d * dx + (1.0 - a_d) * self._dx_prev
        cutoff = self._min_cutoff + self._beta * abs(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1.0 - a) * self._x_prev
        self._x_prev, self._dx_prev = x_hat, dx_hat
        return x_hat


def curls_from_landmarks(pts: np.ndarray) -> dict[str, float]:
    """`pts`: array (21, 2) com os landmarks já em pixels (x, y).
    Devolve a flexão 0–1 de cada uma das 6 juntas primárias.

    Função PURA — testada sem cv2/mediapipe (ver test_hand_camera_teleop).
    """
    palm_w = float(np.linalg.norm(pts[_INDEX_MCP] - pts[_PINKY_MCP])) or 1.0

    out: dict[str, float] = {}

    for name, (mcp, pip, dip, tip) in _FINGER_LM.items():
        pip_ang = _angle_deg(pts[mcp], pts[pip], pts[tip])
        mcp_ang = _angle_deg(pts[_WRIST], pts[mcp], pts[pip])
        curl = (_W_PIP * _lerp01(pip_ang, _PIP_OPEN_DEG, _PIP_CLOSED_DEG)
                + _W_MCP * _lerp01(mcp_ang, _MCP_OPEN_DEG, _MCP_CLOSED_DEG))
        out[name] = max(0.0, min(1.0, curl))

    # Polegar: ângulo do IP + quão perto a ponta está do MCP do indicador.
    ip_ang = _angle_deg(pts[_THUMB_MCP], pts[_THUMB_IP], pts[_THUMB_TIP])
    thumb_ang01 = _lerp01(ip_ang, _THUMB_OPEN_DEG, _THUMB_CLOSED_DEG)
    d_tip_index = np.linalg.norm(pts[_THUMB_TIP] - pts[_INDEX_MCP]) / palm_w
    thumb_dist01 = _lerp01(float(d_tip_index),
                           _THUMB_DIST_OPEN, _THUMB_DIST_CLOSED)
    out['Thumb'] = max(0.0, min(
        1.0, _W_THUMB_ANGLE * thumb_ang01 + _W_THUMB_DIST * thumb_dist01))

    # Rotate = oposição: ponta do polegar aproximando-se do lado do mínimo.
    d_thumb_pinky = np.linalg.norm(pts[_THUMB_TIP] - pts[_PINKY_MCP]) / palm_w
    out['Rotate'] = _lerp01(float(d_thumb_pinky),
                            _ROTATE_DIST_OPEN, _ROTATE_DIST_CLOSED)

    return out


def wrist_pronation_j6_deg(wpts: np.ndarray) -> float:
    """Pronação/supinação do punho (palma ↔ dorso) → alvo ABSOLUTO em
    GRAUS para joint6.

    `wpts`: world-landmarks 3D (21, 3) em metros — eixos alinhados à
    imagem: x→direita, y→baixo, z→câmera (`multi_hand_world_landmarks`).

    palma de frente p/ a câmera → 0°; mão de perfil → ±90°; dorso de
    frente → ±180° (saturado por `_J6_PRON_RANGE_DEG`). Assume UMA mão
    consistente — troque `_J6_SIGN` se usar a outra. Função PURA.
    """
    n = np.cross(wpts[_INDEX_MCP] - wpts[_WRIST],
                 wpts[_PINKY_MCP] - wpts[_WRIST])
    nn = float(np.linalg.norm(n))
    if nn < 1e-9:
        return _J6_CENTER_DEG
    n = n / nn                                   # normal da palma (→câmera se palma)
    phi = float(np.degrees(np.arctan2(n[0], n[2])))
    phi = max(-_J6_PRON_RANGE_DEG, min(_J6_PRON_RANGE_DEG, phi))
    return _J6_CENTER_DEG + _J6_SIGN * (phi / _J6_PRON_RANGE_DEG) * (_J6_SPAN_DEG * 0.5)


class HandCameraTeleop:
    """Gerencia câmera + MediaPipe numa thread e emite flexão 0–1.

    Uso:
        cam = HandCameraTeleop(on_curl=cb, camera_index=0, logger=node.get_logger())
        cam.start()          # levanta CameraUnavailable se a câmera falhar
        ...
        cam.stop()           # idempotente; libera cap + fecha janela

    `on_curl` recebe `dict[str, float]` (chaves de JOINTS, mais `ArmJ6`)
    e é chamado da thread da câmera — o consumidor marshaliza para a GUI.
    Se o loop morrer sozinho (câmera desconectada), `self.error` vira uma
    string e o consumidor deve chamar `stop()` no seu próximo poll.
    """

    def __init__(self, on_curl: Callable[[dict[str, float]], None], *,
                 camera_index: int = 0, logger=None,
                 show_window: bool = True) -> None:
        self._on_curl = on_curl
        self._camera_index = int(camera_index)
        self._log = logger
        self._show_window = show_window

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._cap = None
        self._hands = None
        self._cv2 = None
        self._mp = None
        self._oe: dict[str, _OneEuro] | None = None
        self._oe_arm: dict[str, _OneEuro] | None = None
        self._last_emit = 0.0
        self._last_emitted: dict[str, float] | None = None
        self._track_frames = 0
        self._last_ts: float | None = None
        self._latest_frame: np.ndarray | None = None
        self._frame_lock = threading.Lock()
        self.error: str | None = None

    # ── ciclo de vida ────────────────────────────────────────────────
    def start(self) -> None:
        try:
            import cv2
            import mediapipe as mp
        except ImportError as exc:                       # pragma: no cover
            raise ImportError(
                'camera hand control needs opencv-python + mediapipe '
                f'({exc})') from exc
        self._cv2, self._mp = cv2, mp

        # CAP_V4L2 é o backend estável para webcam USB no Linux.
        cap = cv2.VideoCapture(self._camera_index, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            raise CameraUnavailable(
                f'cv2.VideoCapture({self._camera_index}) não abriu')
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ok, _ = cap.read()
        if not ok:
            cap.release()
            raise CameraUnavailable(
                f'câmera {self._camera_index} abriu mas não entregou quadro')
        self._cap = cap

        self._hands = mp.solutions.hands.Hands(
            static_image_mode=False, max_num_hands=1, model_complexity=0,
            min_detection_confidence=0.6, min_tracking_confidence=0.5)

        self._stop.clear()
        self.error = None
        self._oe = None
        self._oe_arm = None
        self._last_emitted = None
        self._track_frames = 0
        self._last_ts = None
        self._thread = threading.Thread(
            target=self._loop, name='hand-camera-teleop', daemon=True)
        self._thread.start()
        self._info(f'camera hand control ON (device {self._camera_index})')

    def stop(self) -> None:
        self._stop.set()
        t = self._thread
        self._thread = None
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=2.0)
        # Fallback: se a thread travou no join, ainda liberamos os recursos.
        self._release()
        self._info('camera hand control OFF')

    # ── loop da thread ──────────────────────────────────────────────
    def _loop(self) -> None:
        cv2 = self._cv2
        fails = 0
        try:
            while not self._stop.is_set():
                ok, frame = self._cap.read()
                if not ok:
                    fails += 1
                    if fails >= _MAX_READ_FAILS:
                        self.error = 'câmera parou de entregar quadros'
                        break
                    time.sleep(0.03)
                    continue
                fails = 0

                now = time.monotonic()
                dt = (now - self._last_ts) if self._last_ts is not None else (1.0 / 30.0)
                self._last_ts = now

                frame = cv2.flip(frame, 1)               # espelho (natural)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                rgb.flags.writeable = False
                res = self._hands.process(rgb)

                curls: dict[str, float] | None = None
                if res.multi_hand_landmarks:
                    lm = res.multi_hand_landmarks[0]
                    h, w = frame.shape[:2]
                    pts = np.array([[p.x * w, p.y * h] for p in lm.landmark],
                                   dtype=np.float64)
                    curls = self._smooth(curls_from_landmarks(pts), dt)

                    if res.multi_hand_world_landmarks:
                        wpts = np.array(
                            [[p.x, p.y, p.z] for p in
                             res.multi_hand_world_landmarks[0].landmark],
                            dtype=np.float64)
                        curls.update(self._smooth_arm(
                            {'ArmJ6': wrist_pronation_j6_deg(wpts)}, dt))

                    self._track_frames += 1
                    if self._track_frames >= _MIN_TRACK_FRAMES:
                        self._maybe_emit(curls)
                    if self._show_window:
                        self._mp.solutions.drawing_utils.draw_landmarks(
                            frame, lm, self._mp.solutions.hands.HAND_CONNECTIONS)
                else:
                    # Mão perdida: NÃO comanda (segura a última pose) e zera os
                    # filtros para a re-aquisição entrar suave, sem tranco.
                    self._track_frames = 0
                    self._oe = None
                    self._oe_arm = None

                if self._show_window:
                    self._draw_overlay(frame, curls)
                    with self._frame_lock:
                        self._latest_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        except Exception as exc:                          # pragma: no cover
            self.error = f'{type(exc).__name__}: {exc}'
            self._warn(f'loop da câmera abortou: {self.error}')
        finally:
            self._release()

    # ── helpers ────────────────────────────────────────────────────
    def _smooth(self, curls: dict[str, float], dt: float) -> dict[str, float]:
        if self._oe is None:
            self._oe = {k: _OneEuro(v, min_cutoff=_OE_MIN_CUTOFF,
                                    beta=_OE_BETA, d_cutoff=_OE_D_CUTOFF)
                        for k, v in curls.items()}
            return dict(curls)
        return {k: max(0.0, min(1.0, self._oe[k](v, dt)))
                for k, v in curls.items()}

    def _smooth_arm(self, axes: dict[str, float], dt: float) -> dict[str, float]:
        """Filtro leve de joint6 (graus, sem clamp aqui)."""
        if self._oe_arm is None:
            self._oe_arm = {
                k: _OneEuro(v, min_cutoff=_ARM_OE_MIN_CUTOFF,
                            beta=_ARM_OE_BETA, d_cutoff=_OE_D_CUTOFF)
                for k, v in axes.items()}
            return dict(axes)
        return {k: self._oe_arm[k](v, dt) for k, v in axes.items()}

    def _maybe_emit(self, payload: dict[str, float]) -> None:
        now = time.monotonic()
        if (now - self._last_emit) < (1.0 / _EMIT_HZ):
            return
        prev = self._last_emitted
        if prev is not None:
            still_fingers = max(abs(payload[k] - prev[k])
                                for k in JOINTS) < _EMIT_DEADBAND
            still_arm = all(abs(payload.get(k, 0.0) - prev.get(k, 0.0))
                            < _ARM_DEADBAND_DEG for k in ARM_KEYS)
            if still_fingers and still_arm:
                return
        self._last_emit = now
        self._last_emitted = dict(payload)
        try:
            self._on_curl(payload)
        except Exception as exc:                          # pragma: no cover
            self._warn(f'callback on_curl falhou: {exc}')

    def get_latest_frame(self) -> np.ndarray | None:
        """Último quadro anotado (RGB, HxWx3 uint8) ou None. Thread-safe —
        a GUI Tk chama isto no seu loop `after` para desenhar o vídeo."""
        with self._frame_lock:
            return None if self._latest_frame is None else self._latest_frame.copy()

    def _draw_overlay(self, frame, curls: dict[str, float] | None) -> None:
        cv2 = self._cv2
        y = 24
        cv2.putText(frame, 'no hand' if curls is None else 'tracking',
                    (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 255) if curls is None else (0, 200, 0), 2)
        if curls is None:
            return
        for name in JOINTS:
            y += 26
            v = curls.get(name, 0.0)
            cv2.putText(frame, f'{name:<7}', (12, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.rectangle(frame, (90, y - 12), (290, y), (60, 60, 60), 1)
            cv2.rectangle(frame, (90, y - 12), (90 + int(200 * v), y),
                          (0, 180, 255), -1)
        y += 26
        j6 = curls.get('ArmJ6')
        cv2.putText(frame,
                    'joint6  (no 3d)' if j6 is None
                    else f'joint6  {j6:+6.1f}deg',
                    (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 220), 1)

    def _release(self) -> None:
        if self._hands is not None:
            try:
                self._hands.close()
            except Exception:
                pass
            self._hands = None
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        with self._frame_lock:
            self._latest_frame = None

    def _info(self, msg: str) -> None:
        if self._log is not None:
            self._log.info(f'[CAM-TELEOP] {msg}')

    def _warn(self, msg: str) -> None:
        if self._log is not None:
            self._log.warning(f'[CAM-TELEOP] {msg}')
