"""Guardas de movimento em ar livre e de entrada do `tactile_explorer`.

Mixin: usa o estado e as primitivas do nó. Mora fora do host pelo limite de
tamanho travado em test_explorer_split.py.
"""
from __future__ import annotations

import math
import time

import numpy as np
from touch_pack_msgs.msg import PalpationStart

from .explorer_constants import (
    _ARRIVE_MOVE_RAD, _ARRIVE_RTF_MAX,
    _ARRIVE_TIMEOUT_S, _ARRIVE_TOL_M, _ARRIVE_TOL_RAD, _CTRL_DT,
    _FORCE_SAFE_LIMIT_N, _FORCE_STALE_S,
)
from .kinematics import (
    JOINT_MAX, JOINT_MIN, T_TOUCH_TOOL_ATTACH, forward_kinematics,
)


class GuardsMixin:
    @staticmethod
    def _start_refusal(msg: PalpationStart) -> str | None:
        """Motivo para recusar o start antes de qualquer movimento, ou None."""
        for name, typ in PalpationStart.get_fields_and_field_types().items():
            if not typ.startswith(('float', 'double')):
                continue
            v = np.asarray(getattr(msg, name, 0.0), dtype=float)
            if not np.all(np.isfinite(v)):
                return f'campo {name} não-finito ({v})'
        home = np.radians(np.asarray(msg.home_deg, dtype=float))
        if home.shape != (6,) or not np.any(home):
            return ('home_deg ausente ou toda zero: é o braço ESTICADO, não '
                    'uma home (inclua o campo no ros2 topic pub)')
        if (np.any(home < JOINT_MIN - 1e-3) or np.any(home > JOINT_MAX + 1e-3)):
            return f'home_deg fora dos limites articulares ({list(msg.home_deg)})'
        return None

    def _motion_guard(self, label: str,
                      require_fresh: bool = True) -> str | None:
        """Guardas de um movimento em ar livre, uma vez por tick. Quando
        disparam, já substituíram o goal do JTC. None = pode seguir.

        `require_fresh=False` (o recuo à HOME): célula muda não impede de sair
        do contato, mas força alta com a célula viva impede de arrastar.
        """
        if self._stop_requested.is_set():
            self._stop_requested.clear()
            self._settle()
            return 'stop'
        if self._pause_requested.is_set():
            # _pause_gate segura a posição, o que descarta o resto da
            # trajetória: 'paused' é para quem chama replanejar.
            return 'paused' if self._pause_gate() else 'stop'
        if require_fresh:
            if self._force_stale_abort(label):
                self._settle()
                return 'stale'
            fresca = True
        else:
            with self._lc_lock:
                fresca = time.monotonic() - self._lc_force_ts <= _FORCE_STALE_S
        if fresca and self._force_over_limit():
            self._settle()          # substitui o goal: PARA no lugar
            self.get_logger().error(
                f'SEGURANÇA [{label}]: força {self._fz_corrected():+.1f} N '
                f'(margem de {_FORCE_SAFE_LIMIT_N:.0f} N) durante um movimento '
                'em ar livre — algo está no caminho. Trajetória interrompida '
                'no lugar.')
            return 'force'
        return None

    def _wait_motion(self, q_target: np.ndarray, duration_s: float,
                     label: str, require_fresh: bool = True) -> str:
        """Espera a trajetória já publicada, sob guarda, e confirma a pose
        MEDIDA. 'done' | 'stop' | 'paused' | 'stale' | 'force' | 'error'."""
        p_alvo = forward_kinematics(q_target, T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]
        t_ini = time.monotonic()
        t_end = t_ini + duration_s
        # O JTC anda em tempo de simulação: com RTF < 1 a trajetória dura mais
        # que o relógio de parede diz. Enquanto o braço ANDA, espera-se — até
        # um teto de parede.
        t_prog, q_prog = t_end, None
        t_teto = t_end + _ARRIVE_RTF_MAX * duration_s + _ARRIVE_TIMEOUT_S
        while True:
            out = self._motion_guard(label, require_fresh)
            if out is not None:
                return out
            now = time.monotonic()
            if now >= t_end:
                q = self._q_now()
                err_m = float(np.linalg.norm(forward_kinematics(
                    q, T_end=T_TOUCH_TOOL_ATTACH)[:3, 3] - p_alvo))
                err_rad = float(np.max(np.abs(q - q_target)))
                if err_m <= _ARRIVE_TOL_M and err_rad <= _ARRIVE_TOL_RAD:
                    return 'done'
                if (q_prog is None
                        or float(np.max(np.abs(q - q_prog))) > _ARRIVE_MOVE_RAD):
                    t_prog, q_prog = now, q
                if now >= min(t_prog + _ARRIVE_TIMEOUT_S, t_teto):
                    self._settle()
                    self.get_logger().error(
                        f'[{label}] o braço não chegou: TCP a '
                        f'{err_m*1e3:.1f} mm do alvo, junta a '
                        f'{math.degrees(err_rad):.2f}° — trajetória descartada '
                        'ou braço parado.')
                    return 'error'
            time.sleep(_CTRL_DT)

    def _abort_in_place(self) -> None:
        """A própria HOME falhou (STOP, força, braço que não chegou): repetir
        o mesmo caminho é o que o operador, ou a guarda, acabou de interromper."""
        if not self._freeze_in_place():
            self._settle()
            self._set_phase('ABORTED')
