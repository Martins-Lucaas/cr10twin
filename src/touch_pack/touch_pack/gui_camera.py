"""gui_camera.py — teleoperação da mão pela webcam (MediaPipe).

A câmera é um modo de ENTRADA alternativo: em vez dos sliders, a curvatura dos
dedos do operador vira ângulo de junta da COVVI, e a inclinação da mão vira J6.

Tudo aqui vive sob duas regras que não são óbvias olhando o código:

  * o teleop só comanda a mão e J6 — nunca o resto do braço. Um falso positivo
    do MediaPipe com autoridade sobre as seis juntas moveria o braço inteiro
    por causa de uma sombra;
  * a janela de preview e o laço de polling são objetos Tk, e desligar o modo
    tem de derrubar os dois. Um `after` sobrevivente continua consumindo quadro
    depois que o operador achou que desligou a câmera.

Recortado de `palpation_gui.py` — os métodos operam sobre `self` como antes.
"""
from __future__ import annotations

import math as _math
import tkinter as tk

from .gui_constants import HAND_LIMITS_DEG
from .constants import ARM_JOINTS, HAND_JOINTS
from .gui_constants import ARM_LIMITS_DEG, MANIP_TRAJ_DURATION_S
from .ui_helpers import BG, BTN_NEUTRAL, DANGER, OK, TEXT, TEXT_DIM, WARN


class CameraMixin:
    """gui_camera.py — teleoperação da mão pela webcam (MediaPipe)."""

    # ── Teleoperação da mão por câmera ───────────────────────────────
    # A webcam + MediaPipe (hand_camera_teleop) devolve flexão 0–1 por
    # junta; aqui isso vira grau (HAND_LIMITS_DEG) e entra nos
    # `hand_sliders` — daí em diante é o MESMO caminho do slider manual.
    # Ligar a câmera já move o SIM: dedos da mão + joint6 (pronação do
    # punho, palma↔dorso, chave 'ArmJ6'). A checkbox "→ real" só libera o
    # hardware: mão COVVI real (SetDigitPosn, precisa do ECI) e joint6 no
    # braço real quando a checkbox está marcada E o robô está em MIRROR.
    def _toggle_camera_control(self) -> None:
        """Liga/desliga o controle da mão COVVI pela câmera."""
        if self._cam_teleop is not None:
            self._stop_camera_control('Camera hand control disabled.')
            return
        if not getattr(self, 'hand_sliders', None):
            self._set_status(
                'Camera control needs the hand end-effector.', WARN)
            return
        try:
            from touch_pack.hand_camera_teleop import (
                HandCameraTeleop, CameraUnavailable)
        except ImportError as exc:
            self._set_status(
                f'Camera control unavailable — pip install mediapipe ({exc}).',
                DANGER)
            return
        cam_idx = self._camera_index
        try:
            self._cam_teleop = HandCameraTeleop(
                on_curl=self._on_camera_curl, camera_index=cam_idx,
                logger=self.get_logger())
            self._cam_teleop.start()
        except (CameraUnavailable, ImportError) as exc:
            self._cam_teleop = None
            self._cam_btn.set_state('📷', 'CAM OFF', BTN_NEUTRAL, TEXT)
            self._set_status(
                f'Camera not available (index {cam_idx}): {exc}', DANGER)
            return
        self._cam_btn.set_state('📷', 'CAM ON', OK, 'white')
        tail = '' if self._eci_enabled else '  (sim only — ECI is OFF)'
        self._set_status(f'Camera hand control active.{tail}', OK)
        self._cam_teleop_after = self.root.after(500, self._poll_camera_teleop)
        self._open_camera_preview()
    def _poll_camera_teleop(self) -> None:
        """Vigia o loop da câmera; se ele morreu sozinho, encerra limpo."""
        self._cam_teleop_after = None
        cam = self._cam_teleop
        if cam is None:
            return
        if cam.error:
            self._stop_camera_control(
                f'Camera hand control stopped: {cam.error}', color=DANGER)
            return
        self._cam_teleop_after = self.root.after(500, self._poll_camera_teleop)
    def _on_camera_curl(self, curl: dict) -> None:
        """Callback vindo da thread da câmera → volta ao thread Tk."""
        self.root.after(0, self._apply_camera_curl, curl)
    def _apply_camera_curl(self, curl: dict) -> None:
        if self._cam_teleop is None or not getattr(self, 'hand_sliders', None):
            return
        with self._suppress():
            for j in HAND_JOINTS:
                c = curl.get(j)
                if c is None:
                    continue
                lo, hi = HAND_LIMITS_DEG[j]
                deg = lo + max(0.0, min(1.0, float(c))) * (hi - lo)
                self.hand_sliders[j].set(round(deg, 1))
        # Ligar a câmera já move o SIM: dedos da mão + joint6. A checkbox
        # "→ real" só libera o hardware — mão COVVI real (precisa do ECI) e,
        # se o robô estiver em MIRROR, joint6 no braço real.
        mirror_real = bool(self._cam_mirror_var and self._cam_mirror_var.get())
        self._publish_hand_from_sliders(allow_real=mirror_real)
        self._apply_camera_j6(curl, mirror_real=mirror_real)
    def _apply_camera_j6(self, curl: dict, *, mirror_real: bool) -> None:
        """joint6 ← pronação do punho, palma↔dorso (chave 'ArmJ6'). Sempre
        move o sim; só espelha no CR10 real quando `mirror_real` E o robô
        está em MIRROR. joint1-5 seguram o valor atual dos sliders."""
        j6 = curl.get('ArmJ6')
        if j6 is None or not getattr(self, 'arm_sliders', None):
            return
        with self._lock:
            phase = self._latest_phase
        if phase not in ('IDLE', 'DONE', 'ABORTED'):
            return
        q_deg: list[float] = []
        with self._suppress():
            for j in ARM_JOINTS:
                lo, hi = ARM_LIMITS_DEG[j]
                if j == 'joint6':
                    v = max(lo, min(hi, float(j6)))
                    self.arm_sliders[j].set(round(v, 1))
                else:
                    v = self._clamp_var(self.arm_sliders[j], lo, hi)
                    if v is None:
                        return
                q_deg.append(v)
        q_rad = [_math.radians(d) for d in q_deg]
        mirror = mirror_real and self._robot_mode == 'MIRROR'
        # `_cb_arm_trajectory` lê isto para decidir se espelha no braço real.
        self._cam_arm_gate = (mirror, tuple(round(float(x), 4) for x in q_rad))
        self._publish_arm_q(q_rad, MANIP_TRAJ_DURATION_S)
    def _open_camera_preview(self) -> None:
        """Janela Tk com o vídeo anotado da câmera. Quem desenha é a GUI
        (main thread) — `cv2.imshow` fora da main thread não funciona no Linux."""
        if self._cam_win is not None:
            return
        try:
            from PIL import Image, ImageTk  # noqa: F401
        except ImportError:
            self._set_status(
                'Camera active (no video preview — pip install pillow).', WARN)
            return
        win = tk.Toplevel(self.root)
        win.title('Camera Hand Control')
        win.configure(bg=BG)
        win.resizable(False, False)
        lbl = tk.Label(win, bg='black', bd=0)
        lbl.pack(padx=6, pady=6)
        win.protocol(
            'WM_DELETE_WINDOW',
            lambda: self._stop_camera_control('Camera hand control disabled.'))
        self._cam_win = win
        self._cam_win_lbl = lbl
        self._cam_preview_imgtk = None
        self._cam_preview_after = self.root.after(60, self._pump_camera_preview)
    def _pump_camera_preview(self) -> None:
        """Puxa o último quadro do teleop e mostra na janela (loop Tk `after`)."""
        self._cam_preview_after = None
        cam = self._cam_teleop
        if cam is None or self._cam_win is None:
            return
        frame = cam.get_latest_frame()
        if frame is not None:
            from PIL import Image, ImageTk
            imgtk = ImageTk.PhotoImage(Image.fromarray(frame))
            self._cam_win_lbl.configure(image=imgtk)
            self._cam_preview_imgtk = imgtk        # segura a referência
        self._cam_preview_after = self.root.after(60, self._pump_camera_preview)
    def _stop_camera_control(self, msg: str, *, color: str = TEXT_DIM) -> None:
        """Encerra o teleop por câmera: cancela o poll, para a thread,
        libera a câmera e destrói a janela de vídeo. Idempotente."""
        if self._cam_teleop_after is not None:
            try:
                self.root.after_cancel(self._cam_teleop_after)
            except Exception:
                pass
            self._cam_teleop_after = None
        if self._cam_preview_after is not None:
            try:
                self.root.after_cancel(self._cam_preview_after)
            except Exception:
                pass
            self._cam_preview_after = None
        if self._cam_win is not None:
            try:
                self._cam_win.destroy()
            except Exception:
                pass
            self._cam_win = None
            self._cam_preview_imgtk = None
        cam = self._cam_teleop
        self._cam_teleop = None
        if cam is not None:
            try:
                cam.stop()
            except Exception:
                pass
        if getattr(self, '_cam_btn', None) is not None:
            self._cam_btn.set_state('📷', 'CAM OFF', BTN_NEUTRAL, TEXT)
        self._set_status(msg, color)
