"""
hand_latency_probe.py — Mede a latência dos fluxos Sim↔Real da MÃO COVVI.

É o análogo do `latency_probe.py` (que mede o braço CR10) para o outro elo do
gêmeo digital. Mesmo método, mesma convenção de sinal, mesmos artefatos — o
que muda é de onde vem cada timeline:

  Braço                                Mão
  ─────                                ───
  SIM  = /joint_states (juntas do      SIM  = /joint_states (juntas primárias
         braço, publicadas pelo               da mão no Gazebo)
         Gazebo)
  REAL = feedback @125 Hz do CR10      REAL = telemetria DigitPosnAll do ECI
         lido em poll readonly                (stream realtime da mão física)

Método (idêntico ao do braço):
  • As duas séries são carimbadas com time.monotonic() DENTRO deste processo
    → relógio comum, comparação justa.
  • São reamostradas numa grade uniforme e alinhadas por correlação cruzada;
    o deslocamento de pico (com refino parabólico sub-amostra) é a latência.

Diferença de instrumentação (medida em 10/09/2026, vai no JSON):
  a posição real do dedo chega quantizada em contagens ECI — o curso de 90°
  cabe em ~151 contagens, ou ~0,6°/contagem — e a 10 Hz (período de 99,8 ms,
  aferido no `DigitPosnAllMsg` da mão física), não num poll que este nó
  controle. Os 10 Hz PARECEM pouco contra um atraso de dezenas de ms, mas não
  são o limite da estimativa: a correlação cruzada sobre uma janela de 20 s
  promedia ~200 amostras, e num teste sintético com esta taxa e esta
  quantização o estimador recupera um atraso de 70 ms com ±0,9 ms (pior erro
  2,7 ms), contra ±0,15 ms do braço a 125 Hz. O que degrada a medida é janela
  curta ou dedo parado, não a cadência do stream.

Convenção de sinal do deslocamento estimado `d` (a MESMA do braço):
  d > 0  → o REAL atrasa em relação ao SIM  → latência **Sim-to-Real**
           (comanda-se pelos sliders/GUI; a mão física replica com atraso `d`).
  d < 0  → o SIM atrasa em relação ao REAL  → latência **Real-to-Sim**
           (mirror Versão B: a mão física é a mestra e o gêmeo replica).

Uso:
  # Sim-to-Real: comande a mão pela GUI (sliders/grips) durante a captura
  ros2 run touch_pack hand_latency_probe --ros-args \
      -p direction:=sim_to_real -p duration_s:=20.0

  # Real-to-Sim: com o mirror da mão ativo na GUI, mova a mão física
  ros2 run touch_pack hand_latency_probe --ros-args \
      -p direction:=real_to_sim -p duration_s:=20.0

  # duration_s:=0  → captura até Ctrl-C.

Saída (em data/latency_hand/, versionado no git como o do braço):
  hand_latency_<sentido>_<ts>_raw.csv      as DUAS séries brutas (6 juntas,
                                           taxas nativas, relógio comum)
  hand_latency_<sentido>_<ts>_aligned.csv  par reamostrado da junta usada
  hand_latency_<sentido>_<ts>_result.json  resultado + metadados completos
  hand_latency_<sentido>_<ts>.png          gráfico sobreposto

Pós-processamento (tabelas): o MESMO `latency_report`, apontado para o
diretório da mão:
  ros2 run touch_pack latency_report -- data/latency_hand

Parâmetros ROS:
  eci_prefix   '/covvi/hand'   namespace do driver COVVI (mesmo da GUI)
  direction    'auto'          'sim_to_real' | 'real_to_sim' | 'auto'
  duration_s   20.0            janela de captura (0 = até Ctrl-C)
  joint_index  -1              dedo usado na correlação (-1 = o que mais move)
  grid_dt_s    0.004           passo da grade de reamostragem
  max_lag_s    0.6             busca de atraso em ±max_lag_s
  enable_realtime  True        chama SetRealtimeCfg(digit_posn=True) ao subir
"""
from __future__ import annotations

import csv
import json
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import JointState

from .constants import (
    HAND_JOINTS, LATENCY_DIR, eci_posn_to_deg, hand_deg_to_driver_rad)
from .latency_report import xcorr_lag

# Campos do DigitPosnAllMsg, na ordem de HAND_JOINTS.
_POSN_FIELDS = ['thumb_pos', 'index_pos', 'middle_pos',
                'ring_pos', 'little_pos', 'rotate_pos']

# Abaixo desta amplitude o dedo está parado e a correlação é ruído. Maior que
# o limiar do braço porque a contagem ECI quantiza em ~0,6°: um dedo imóvel
# ainda balança meio degrau.
MIN_AMP_DEG = 0.5


class HandLatencyProbe(Node):

    def __init__(self):
        super().__init__('hand_latency_probe')
        # Os parâmetros numéricos são declarados com tipagem dinâmica para
        # `-p duration_s:=20` valer tanto quanto `:=20.0`. Sem isso o rclpy
        # recusa o INTEGER contra um default DOUBLE e o nó nem sobe — erro
        # que só aparece na bancada, com a mão já ligada esperando.
        # Toda leitura abaixo já faz float()/int() no valor.
        num = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('eci_prefix', '/covvi/hand')
        self.declare_parameter('direction', 'auto')
        self.declare_parameter('duration_s', 20.0, num)
        self.declare_parameter('joint_index', -1, num)
        self.declare_parameter('grid_dt_s', 0.004, num)
        self.declare_parameter('max_lag_s', 0.6, num)
        self.declare_parameter('enable_realtime', True)

        self._lock = threading.Lock()
        self._sim: list[tuple[float, list[float]]] = []
        self._real: list[tuple[float, list[float]]] = []
        self._stop = threading.Event()

        self.create_subscription(
            JointState, '/joint_states', self._cb_joints, 100)

    def _prefix(self) -> str:
        return str(self.get_parameter('eci_prefix').value or '/covvi/hand')

    # ── coleta SIM (assinatura /joint_states) ─────────────────────────
    def _cb_joints(self, msg: JointState) -> None:
        pos = dict(zip(msg.name, msg.position))
        try:
            q = [float(pos[j]) for j in HAND_JOINTS]
        except KeyError:
            return   # mensagem só do braço — ignora
        with self._lock:
            self._sim.append((time.monotonic(), q))

    # ── coleta REAL (telemetria DigitPosnAll) ─────────────────────────
    def start_real_stream(self) -> bool:
        """Liga o stream digit_posn no driver COVVI e assina o tópico."""
        try:
            import covvi_interfaces.msg as eci_msg
            import covvi_interfaces.srv as eci_srv
        except ImportError as exc:
            self.get_logger().error(
                f'covvi_interfaces indisponível ({exc}) — o driver da mão '
                'não está no ambiente. Faça source do overlay do eci_ros.')
            return False

        prefix = self._prefix()
        if bool(self.get_parameter('enable_realtime').value):
            cli = self.create_client(
                eci_srv.SetRealtimeCfg, f'{prefix}/SetRealtimeCfg')
            if not cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(
                    f'{prefix}/SetRealtimeCfg não apareceu em 10 s — o driver '
                    'da mão está rodando? (a mão real precisa estar ligada)')
                return False
            # Preserva os streams que o driver já liga no startup — desligar
            # digit_touch/env/orient aqui quebraria a GUI rodando em paralelo.
            req = eci_srv.SetRealtimeCfg.Request()
            req.digit_posn = True
            req.digit_touch = True
            req.environmental = True
            req.orientation = True
            cli.call_async(req)
            self.get_logger().info(
                'SetRealtimeCfg(digit_posn=True) enviado ao driver.')

        self.create_subscription(
            eci_msg.DigitPosnAllMsg, f'{prefix}/DigitPosnAllMsg',
            self._cb_real_posn, 10)
        self.get_logger().info(
            f'Assinando {prefix}/DigitPosnAllMsg (posição medida dos dedos).')
        return True

    def _cb_real_posn(self, msg) -> None:
        # Contagem ECI → grau de ponta de dedo → radiano da junta DRIVER: é o
        # espaço em que o /joint_states do sim reporta, e comparar as duas
        # séries em espaços diferentes mediria um erro de escala como se fosse
        # infidelidade do gêmeo.
        q = [hand_deg_to_driver_rad(j, eci_posn_to_deg(j, getattr(msg, f)))
             for j, f in zip(HAND_JOINTS, _POSN_FIELDS)]
        with self._lock:
            self._real.append((time.monotonic(), q))

    # ── análise ───────────────────────────────────────────────────────
    def analyze(self):
        with self._lock:
            sim = list(self._sim)
            real = list(self._real)
        if len(sim) < 50 or len(real) < 50:
            self.get_logger().error(
                f'Amostras insuficientes (sim={len(sim)}, real={len(real)}). '
                'A mão se moveu durante a captura? A telemetria DigitPosnAll '
                'está chegando?')
            return None

        t_sim = np.array([s[0] for s in sim])
        q_sim = np.array([s[1] for s in sim])
        t_real = np.array([r[0] for r in real])
        q_real = np.array([r[1] for r in real])

        dt = float(self.get_parameter('grid_dt_s').value)
        max_lag_s = float(self.get_parameter('max_lag_s').value)
        j_req = int(self.get_parameter('joint_index').value)

        t0 = max(t_sim[0], t_real[0])
        t1 = min(t_sim[-1], t_real[-1])
        if t1 - t0 < 2.0:
            self.get_logger().error(
                'Sobreposição temporal insuficiente entre as séries.')
            return None
        grid = np.arange(t0, t1, dt)
        n = len(HAND_JOINTS)
        S = np.column_stack([np.interp(grid, t_sim, q_sim[:, j]) for j in range(n)])
        R = np.column_stack([np.interp(grid, t_real, q_real[:, j]) for j in range(n)])

        if j_req < 0 or j_req >= n:
            var = np.var(S, axis=0) + np.var(R, axis=0)
            j = int(np.argmax(var))
        else:
            j = j_req

        max_lag = int(round(max_lag_s / dt))
        amp_deg = float(np.degrees(np.std(R[:, j])))
        lag_s, peak = xcorr_lag(S[:, j], R[:, j], dt, max_lag)

        # Lag por dedo (todos com movimento mensurável) — redundância para a
        # análise posterior validar o número principal.
        per_joint: dict[str, dict] = {}
        for jj in range(n):
            amp_jj = float(np.degrees(np.std(R[:, jj])))
            if amp_jj < MIN_AMP_DEG:
                continue
            l_jj, c_jj = xcorr_lag(S[:, jj], R[:, jj], dt, max_lag)
            per_joint[HAND_JOINTS[jj]] = {
                'lag_ms': round(l_jj * 1e3, 2),
                'peak_corr': round(c_jj, 4),
                'amp_deg': round(amp_jj, 3),
            }

        direction = str(self.get_parameter('direction').value).strip().lower()
        if lag_s >= 0:
            leads, latency_s, detected = 'SIM', lag_s, 'sim_to_real'
        else:
            leads, latency_s, detected = 'REAL', -lag_s, 'real_to_sim'

        # Taxa efetiva das duas fontes: na mão a do REAL é a que limita a
        # resolução do atraso, e ela não é escolhida por este nó.
        real_hz = (len(real) - 1) / (t_real[-1] - t_real[0])
        sim_hz = (len(sim) - 1) / (t_sim[-1] - t_sim[0])

        return {
            'grid': grid, 'S': S, 'R': R, 'joint': j, 'joint_name': HAND_JOINTS[j],
            'lag_s': lag_s, 'latency_ms': latency_s * 1e3, 'leads': leads,
            'peak_corr': peak, 'amp_deg': amp_deg, 'detected': detected,
            'requested': direction, 'n_sim': len(sim), 'n_real': len(real),
            'dur_s': float(grid[-1] - grid[0]), 'per_joint': per_joint,
            'real_hz': float(real_hz), 'sim_hz': float(sim_hz),
        }

    def report_and_save(self, res: dict) -> None:
        jn = res['joint_name']
        self.get_logger().info('─' * 60)
        self.get_logger().info(
            f"LATÊNCIA da MÃO: {res['latency_ms']:.1f} ms "
            f"({res['leads']} adianta → fluxo {res['detected'].replace('_', '-')})")
        self.get_logger().info(
            f"  dedo usado: {jn} | amplitude do movimento: {res['amp_deg']:.2f}° | "
            f"correlação de pico: {res['peak_corr']:.3f}")
        self.get_logger().info(
            f"  janela: {res['dur_s']:.1f} s | amostras sim={res['n_sim']} "
            f"({res['sim_hz']:.0f} Hz) real={res['n_real']} "
            f"({res['real_hz']:.0f} Hz)")
        if res['requested'] in ('sim_to_real', 'real_to_sim') \
                and res['requested'] != res['detected']:
            self.get_logger().warning(
                f"  ATENÇÃO: você pediu '{res['requested']}' mas o sinal indica "
                f"'{res['detected']}'. Confira o modo/direção da captura.")
        if res['amp_deg'] < 2.0:
            self.get_logger().warning(
                '  Movimento muito pequeno (<2°) — a contagem ECI quantiza em '
                '~0,6°, mova mais os dedos para um número confiável.')
        if res['peak_corr'] < 0.9:
            self.get_logger().warning(
                '  Correlação de pico baixa (<0.9) — sinal ruidoso; considere um '
                'movimento mais amplo/lento e repetir.')
        # A 10 Hz (cadência normal do ECI) a estimativa fecha em ±0,9 ms
        # DESDE QUE a janela promedie amostras suficientes — é a contagem que
        # importa, não a taxa. 150 amostras ≈ 15 s de telemetria.
        if res['n_real'] < 150:
            self.get_logger().warning(
                f"  Só {res['n_real']} amostras de telemetria em "
                f"{res['dur_s']:.0f} s ({res['real_hz']:.0f} Hz) — janela curta "
                'para a cadência do ECI. Capture 20 s ou mais.')

        out_dir = os.path.join(LATENCY_DIR, 'latency_hand')
        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        base = os.path.join(out_dir, f"hand_latency_{res['detected']}_{ts}")

        # 1) Séries BRUTAS — mesmo formato do braço, é o que permite refazer a
        #    análise depois sem repetir a bancada.
        with self._lock:
            sim_raw = list(self._sim)
            real_raw = list(self._real)
        raw_path = base + '_raw.csv'
        with open(raw_path, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['t_mono_s', 'source'] + [f'{j}_rad' for j in HAND_JOINTS])
            for t, q in sim_raw:
                w.writerow([f'{t:.6f}', 'sim'] + [f'{v:.6f}' for v in q])
            for t, q in real_raw:
                w.writerow([f'{t:.6f}', 'real'] + [f'{v:.6f}' for v in q])
        self.get_logger().info(f'  Séries brutas: {raw_path}')

        # 2) Par alinhado do dedo usado (pronto para plotar).
        csv_path = base + '_aligned.csv'
        grid, S, R = res['grid'], res['S'], res['R']
        t0 = grid[0]
        with open(csv_path, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['t_rel_s', f'sim_{jn}_deg', f'real_{jn}_deg'])
            for k in range(len(grid)):
                w.writerow([f'{grid[k] - t0:.4f}',
                            f'{np.degrees(S[k, res["joint"]]):.4f}',
                            f'{np.degrees(R[k, res["joint"]]):.4f}'])
        self.get_logger().info(f'  Par alinhado: {csv_path}')

        # 3) Resultado + metadados completos (JSON).
        meta = {
            'timestamp': ts,
            'subsystem': 'hand',
            'direction_requested': res['requested'],
            'direction_detected': res['detected'],
            'latency_ms': round(res['latency_ms'], 2),
            'lag_s_signed': round(res['lag_s'], 5),
            'sign_convention': ('lag>0: REAL atrasa (Sim-to-Real); '
                                'lag<0: SIM atrasa (Real-to-Sim)'),
            'peak_corr': round(res['peak_corr'], 4),
            'joint_used': jn,
            'movement_amp_deg': round(res['amp_deg'], 3),
            'per_joint': res['per_joint'],
            'n_samples_sim': res['n_sim'],
            'n_samples_real': res['n_real'],
            'sim_rate_hz': round(res['sim_hz'], 2),
            'real_telemetry_rate_hz': round(res['real_hz'], 2),
            'real_quantization_deg': round(90.0 / 151.0, 3),
            'overlap_duration_s': round(res['dur_s'], 2),
            'grid_dt_s': float(self.get_parameter('grid_dt_s').value),
            'max_lag_s': float(self.get_parameter('max_lag_s').value),
            'eci_prefix': self._prefix(),
        }
        with open(base + '_result.json', 'w') as fh:
            json.dump(meta, fh, indent=2, ensure_ascii=False)
        self.get_logger().info(f'  Resultado: {base}_result.json')

        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            t = grid - t0
            sim_deg = np.degrees(S[:, res['joint']])
            real_deg = np.degrees(R[:, res['joint']])
            shift = res['lag_s']
            fig, ax = plt.subplots(figsize=(9, 4.2))
            ax.plot(t, sim_deg, label='Gêmeo digital (sim)', lw=1.6)
            ax.plot(t, real_deg, label='Mão física (real)', lw=1.6, alpha=0.85)
            ax.plot(t + shift, real_deg, '--', lw=1.0, alpha=0.6,
                    label=f'Real deslocado ({res["latency_ms"]:.0f} ms)')
            ax.set_xlabel('Tempo (s)')
            ax.set_ylabel(f'{jn} (°)')
            ax.set_title(
                f'Latência da mão {res["detected"].replace("_", "-")}: '
                f'{res["latency_ms"]:.1f} ms  (corr. de pico {res["peak_corr"]:.3f})')
            ax.legend(loc='best', fontsize=8)
            ax.grid(True, alpha=0.3)
            fig.tight_layout()
            png_path = base + '.png'
            fig.savefig(png_path, dpi=140)
            plt.close(fig)
            self.get_logger().info(f'  Gráfico salvo: {png_path}')
        except Exception as exc:
            self.get_logger().warning(
                f'  matplotlib indisponível — gráfico não gerado ({exc}). '
                'Use o CSV.')
        self.get_logger().info(
            f'  Para publicar: git add "{out_dir}" && git commit')
        self.get_logger().info('─' * 60)

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandLatencyProbe()
    if not node.start_real_stream():
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    duration = float(node.get_parameter('duration_s').value)
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True, name='hand-latency-spin')
    spin_thread.start()

    node.get_logger().info(
        f'Capturando por {duration:.0f} s — MOVIMENTE a mão agora '
        '(Ctrl-C encerra antes).' if duration > 0 else
        'Capturando até Ctrl-C — MOVIMENTE a mão agora.')
    try:
        if duration > 0:
            node._stop.wait(duration)
        else:
            while not node._stop.is_set():
                node._stop.wait(1.0)
    except KeyboardInterrupt:
        pass

    node._stop.set()
    res = node.analyze()
    if res is not None:
        node.report_and_save(res)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
