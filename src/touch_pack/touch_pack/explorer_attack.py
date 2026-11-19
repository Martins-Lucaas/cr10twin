"""Ângulo de ataque manual e tare automático do `tactile_explorer`.

Mixin: os métodos usam o estado e as primitivas de movimento do nó. Mora fora
do host pelo limite de tamanho travado em test_explorer_split.py.
"""
from __future__ import annotations

import json
import math
import time

import numpy as np
from std_msgs.msg import Empty, String

from .constants import (
    PROBE_ALIGN_TILT_HARD_MAX_DEG as _ALIGN_TILT_HARD_MAX_DEG,
    SURFACE_PLANE_FILE as _SURFACE_PLANE_FILE,
    selected_tool as _selected_tool,
)
from .explorer_constants import (
    _ALIGN_ORI_TOL_DEG, _ALIGN_TRANSIT_MS, _TARE_FREE_AIR_MAX_N,
    _TARE_SETTLE_S, _TARE_TIMEOUT_S, _TIP_HALF_DIAG_M, _Z_HAT,
)
from .kinematics import (
    T_TOUCH_TOOL_ATTACH, forward_kinematics, inverse_kinematics,
)
from .plane_probe import (
    angle_between_deg as _angle_between_deg,
    approach_start as _approach_start,
    attack_dir_from_normal as _attack_dir_from_normal,
    attack_dir_from_tilt_azimuth as _attack_dir_from_tilt_azimuth,
    load_plane as _load_plane,
    save_plane as _save_plane,
    slope_along_deg as _slope_along_deg,
)


class AttackMixin:
    def _cb_tare_result(self, msg: String) -> None:
        self._tare_reply = str(msg.data)
        self._tare_done.set()

    def _tare_here(self, label: str) -> bool:
        """Pede o tare com o braço parado e espera uma resposta NOVA.

        Quem chama garante que a ferramenta está em ar livre: nenhuma leitura
        distingue um toque leve do peso projetado.
        """
        if not self._settle_until_still():
            self.get_logger().error(
                f'[TARE {label}] o braço não parou — tare recusado.')
            return False
        time.sleep(_TARE_SETTLE_S)
        fz = self._fz_corrected()
        if abs(fz) > _TARE_FREE_AIR_MAX_N:
            self.get_logger().error(
                f'[TARE {label}] fz={fz:+.2f} N acima de '
                f'{_TARE_FREE_AIR_MAX_N:.2f} N: isso é contato, e zerar aqui '
                'esconderia a carga. Tare recusado.')
            return False
        self._tare_done.clear()
        self._tare_pub.publish(Empty())
        if not self._tare_done.wait(_TARE_TIMEOUT_S):
            self.get_logger().error(
                f'[TARE {label}] sem resposta em {_TARE_TIMEOUT_S:.0f} s.')
            return False
        if not self._tare_reply.startswith('ok;'):
            self.get_logger().error(
                f'[TARE {label}] recusado pela célula: {self._tare_reply}.')
            return False
        self.get_logger().info(f'[TARE {label}] zero refeito nesta orientação.')
        return True

    def _tare_tilted_pose(self, label: str) -> bool:
        """Tare numa pose inclinada; a próxima HOME desfaz (_phase_goto_home)."""
        ok = self._tare_here(label)
        self._tare_tilted = self._tare_tilted or ok
        return ok

    def _manual_attack_refusal(self, cfg: dict) -> str | None:
        """Motivo para recusar o ângulo manual, ou None. Roda antes do toque."""
        tilt, az = cfg['manual_tilt_deg'], cfg['manual_azimuth_deg']
        if not 0.0 <= tilt <= _ALIGN_TILT_HARD_MAX_DEG:
            return (f'inclinação {tilt} fora de 0 a '
                    f'{_ALIGN_TILT_HARD_MAX_DEG:.0f}°')
        if not math.isfinite(az):
            return f'azimute inválido ({az})'
        with self._params_lock:
            mode = self._mode
            depth_m = float(self._target_depth_mm) * 1e-3
        if mode not in ('TOUCH', 'MANUAL'):
            return (f'o modo {mode} toma o eixo de ataque como normal da peça '
                    'e inclinaria o plano do ensaio; use TOUCH ou MANUAL')
        attack = _attack_dir_from_tilt_azimuth(tilt, az)
        _start, travel_m = _approach_start(
            np.zeros(3), attack, cfg['retract_m'], _TIP_HALF_DIAG_M)
        if travel_m >= depth_m:
            return (f'o curso até o alvo é de {travel_m * 1e3:.1f} mm e o '
                    f'limite de descida é {depth_m * 1e3:.1f} mm; aumente o '
                    'Max Descent Depth ou reduza o recuo')
        return None

    def _manual_attack(self, target: np.ndarray, cfg: dict) -> str:
        """Prepara a aproximação oblíqua ao `target` localizado pelo toque.

        O alvo é a posição do TCP, o centro da face. Com a ponteira plana a
        aresta toca antes, a cerca de meia largura / cos(θ) do alvo.
        """
        tilt, az = cfg['manual_tilt_deg'], cfg['manual_azimuth_deg']
        attack = _attack_dir_from_tilt_azimuth(tilt, az)
        start, travel_m = _approach_start(
            target, attack, cfg['retract_m'], _TIP_HALF_DIAG_M)
        for nome, p in (('partida', start), ('alvo', target)):
            q, ik_ok = inverse_kinematics(
                p, attack, q_seed=self._q_now(), T_end=T_TOUCH_TOOL_ATTACH)
            T = forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)
            # 3 mm: a IK para com até ~2 mm de erro, e saturar num limite de
            # junta erra bem mais. Aqui só se decide se a pose é alcançável.
            if (not ik_ok or float(np.linalg.norm(T[:3, 3] - p)) > 3e-3
                    or _angle_between_deg(T[:3, 2], attack)
                    > _ALIGN_ORI_TOL_DEG):
                self.get_logger().error(
                    f'[ATTACK] pose de {nome} inalcançável a {tilt:.1f}° / '
                    f'azimute {az:.1f}°. Abortando sem girar o punho.')
                return 'error'

        self._attack_dir = attack
        self._attack_start = start
        # O curso aprendido na vertical não vale neste eixo.
        with self._params_lock:
            self._home_key_cur = None
            self._home_deg_cur = None
            self._learned_contact_m = None
            self._contact_depths.clear()

        self._attack_ref_pub.publish(String(data=json.dumps({
            'tilt_deg': tilt, 'azimuth_deg': az,
            'attack_dir': attack.tolist(),
            'target_m': np.asarray(target, dtype=float).tolist(),
            'start_m': start.tolist(), 'travel_m': travel_m,
            'tip_half_diag_m': _TIP_HALF_DIAG_M})))
        self.get_logger().info(
            f'[ATTACK] alvo em {np.round(target * 1e3, 2)} mm, ataque a '
            f'{tilt:.1f}° / azimute {az:.1f}°, curso de '
            f'{travel_m * 1e3:.1f} mm.')
        return self._goto_attack_start()

    def _goto_attack_start(self) -> str:
        """Sobe até a folga, gira, vai à partida da reta oblíqua e refaz o tare.

        A altura da partida já desconta a aresta inclinada, então girar nela
        ou acima dela não encosta a ponteira na peça.
        """
        attack, start = self._attack_dir, self._attack_start
        self._set_phase('CALIBRATING')
        lift_m = max(0.0, float(start[2] - self._tcp_now()[2]))
        out = self._move_linear_world(
            np.array([0.0, 0.0, lift_m]), _ALIGN_TRANSIT_MS,
            label='ATTACK-LIFT')
        if out != 'done':
            return out
        out = self._rotate_to_attack(attack, label='ATTACK')
        if out != 'ok':
            return out
        out = self._move_linear_world(
            start - self._tcp_now(), _ALIGN_TRANSIT_MS, label='ATTACK-START')
        if out != 'done':
            return out
        self._attack_engaged = True
        if not self._verify_attack_axis(attack, 'ATTACK-START'):
            return 'error'
        return 'ok' if self._tare_tilted_pose('ATTACK') else 'error'

    def _withdraw_along_attack(self) -> None:
        """Sai pela reta oblíqua antes da HOME: o caminho articular arrastaria
        a ponteira. Como o _unload_before_home, nunca impede o recuo."""
        if not self._attack_engaged:
            return
        self._attack_engaged = False
        if self._attack_start is None:
            return
        self._unload_before_home()
        try:
            out = self._move_linear_world(
                self._attack_start - self._tcp_now(), _ALIGN_TRANSIT_MS,
                label='ATTACK-WITHDRAW')
        except Exception as exc:
            out = repr(exc)
        if out != 'done':
            self.get_logger().warn(
                f'[ATTACK] recuo pelo eixo não concluído ({out}); retorno à '
                'home segue mesmo assim.')

    def _align_set_slide_plane(self, fit) -> None:
        """Entrega ao SLIDING o plano MEDIDO da amostra.

        O deslize percorre uma reta contida no plano (ver o bloco "Plano do
        deslize" nas constantes). Sem esta entrega ele cairia na inclinação
        DECLARADA na GUI, e o alinhamento conquistado na descida se perderia
        no primeiro milímetro de curso.

        A declaração do usuário NÃO é sobrescrita: ela continua em
        `_slide_slope_deg` e volta a valer se a calibração não rodar. Só o
        modo SLIDE consome isto.
        """
        with self._params_lock:
            mode = self._mode
        # Grava em SLIDE, TOUCH e MANUAL — medir o vidro num TOUCH é o caso
        # natural. MATRIX_MAP não: o anel dele cai sobre a amostra da grade, e
        # sobrescreveria o plano do vidro pelo de uma peça qualquer.
        if mode != 'MATRIX_MAP':
            try:
                _save_plane(_SURFACE_PLANE_FILE, fit, tool=_selected_tool(),
                            mode=mode)
                self.get_logger().info(
                    f'[ALIGN] plano salvo em {_SURFACE_PLANE_FILE} — runs '
                    'SLIDE com o probe align DESLIGADO passam a usá-lo.')
            except OSError as exc:
                self.get_logger().warn(f'[ALIGN] plano não salvo: {exc}')
        with self._params_lock:
            if self._mode != 'SLIDE':
                return
            dir_xy = self._slide_dir_vec.copy()
            declarado = float(self._slide_slope_deg)
            self._slide_plane_n = np.asarray(fit.normal, dtype=float).copy()
        try:
            medido = _slope_along_deg(fit.normal, dir_xy)
        except ValueError:
            # O plano existe e já foi guardado; só a leitura em GRAUS ao
            # longo do curso é que não faz sentido (plano de pé). O deslize
            # usa o vetor, não este número — que é só para o log.
            self.get_logger().info(
                '[ALIGN] plano do deslize definido pela normal medida '
                '(inclinação ao longo do curso indefinida).')
            return
        self.get_logger().info(
            f'[ALIGN] plano do deslize definido pela normal medida: o curso '
            f'segue uma reta a {medido:+.2f}° na direção pedida (o valor '
            f'declarado na GUI, {declarado:+.2f}°, fica de reserva). O curso '
            'pedido passa a ser distância sobre a SUPERFÍCIE.')

    def _apply_saved_plane(self) -> str:
        """Probe align DESLIGADO no SLIDE: usa o plano salvo pela última
        calibração aceita (`surface_plane.json`), se houver.

        Faz o que a calibração faria sem sondar: entrega a normal ao SLIDING
        e, se o desvio pagar a rotação, gira o punho NA HOME (ar livre, como
        _reapply_attack_orientation) e refaz o tare. Sem arquivo = vertical,
        o comportamento de antes. Retorna 'ok' | 'error' | outcome da rotação.
        """
        with self._params_lock:
            mode = self._mode
        if mode != 'SLIDE':
            return 'ok'
        try:
            saved = _load_plane(_SURFACE_PLANE_FILE,
                                tilt_max_deg=_ALIGN_TILT_HARD_MAX_DEG)
        except ValueError as exc:
            self.get_logger().error(
                f'[PLANE] plano salvo recusado: {exc}. Apague o arquivo ou '
                'remeça com o probe align ligado.')
            return 'error'
        if saved is None:
            return 'ok'
        n, info = saved
        tilt = _angle_between_deg(n, _Z_HAT)
        self._set_phase('CALIBRATING')
        with self._params_lock:
            self._slide_plane_n = n.copy()
        self.get_logger().info(
            f'[PLANE] usando o plano SALVO em {info.get("saved_at", "?")} '
            f'(ponteira {info.get("tool", "?")}, {info.get("n_points", "?")} '
            f'pontos, resíduo {info.get("rms_mm", float("nan")):.3f} mm): '
            f'normal=({n[0]:+.4f}, {n[1]:+.4f}, {n[2]:+.4f}), '
            f'{tilt:.2f}° da horizontal. Se o vidro mudou, ligue o probe '
            'align para remedir.')
        if tilt <= _ALIGN_ORI_TOL_DEG:
            self.get_logger().info(
                f'[PLANE] {tilt:.2f}° dentro da tolerância de '
                f'{_ALIGN_ORI_TOL_DEG:.1f}° — ataque na vertical, deslize no '
                'plano salvo.')
            return 'ok'
        attack = _attack_dir_from_normal(n)
        out = self._rotate_to_attack(attack, label='PLANE-SAVED')
        if out != 'ok':
            return out
        self._attack_dir = attack
        self._attack_start = None
        # Mesmo motivo da calibração: o curso aprendido foi medido na
        # vertical e o novo eixo o alonga.
        with self._params_lock:
            self._home_key_cur = None
            self._home_deg_cur = None
            self._learned_contact_m = None
            self._contact_depths.clear()
        return 'ok' if self._tare_tilted_pose('PLANE-SAVED') else 'error'
