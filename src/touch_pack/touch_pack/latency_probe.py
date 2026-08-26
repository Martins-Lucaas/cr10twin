"""
latency_probe.py — Mede a latência dos fluxos Sim-to-Real e Real-to-Sim.

Motivação: a validação preliminar em bancada (Seção 4.6 do TCC) afirma
operação "em tempo real", mas sem número. Este nó quantifica o atraso
ponta-a-ponta entre a postura do gêmeo digital e a do braço físico, para
sustentar a afirmação com dado medido.

Método (correlação cruzada de duas timelines de juntas):
  • Timeline SIM  — assina /joint_states (estado publicado pelo Gazebo).
  • Timeline REAL — abre um CR10RealDriver em modo READONLY (não pede o
    token de controle, não move o robô; só lê o feedback @125 Hz da porta
    30004) e amostra as juntas a ~100 Hz.
  • Ambas as amostras são carimbadas com time.monotonic() DENTRO deste
    processo → relógio comum, comparação justa.
  • As duas séries são reamostradas numa grade uniforme e alinhadas por
    correlação cruzada; o deslocamento de pico (com refino parabólico
    sub-amostra) é a latência.

Convenção de sinal do deslocamento estimado `d`:
  d > 0  → o REAL atrasa em relação ao SIM  → latência **Sim-to-Real**
           (o operador comanda no sim; o físico replica com atraso `d`).
  d < 0  → o SIM atrasa em relação ao REAL  → latência **Real-to-Sim**
           (drag teach: o físico é o mestre; o gêmeo replica com atraso).

Uso:
  # Sim-to-Real: rode em MIRROR e movimente pela GUI/palpação durante a captura
  ros2 run touch_pack latency_probe --ros-args \
      -p direction:=sim_to_real -p robot_ip:=192.168.5.2 -p duration_s:=20.0

  # Real-to-Sim: rode com drag teach ativo e conduza o braço à mão
  ros2 run touch_pack latency_probe --ros-args \
      -p direction:=real_to_sim -p duration_s:=20.0

  # duration_s:=0  → captura até Ctrl-C.

Saída (tudo em sensors/Data/latency/ — diretório VERSIONADO no git, para a
análise posterior ser reproduzível a partir do repositório):
  latency_<sentido>_<ts>_raw.csv      as DUAS séries brutas (6 juntas, taxas
                                      nativas, relógio monotônico comum)
  latency_<sentido>_<ts>_aligned.csv  par reamostrado da junta usada
  latency_<sentido>_<ts>_result.json  resultado + metadados completos
  latency_<sentido>_<ts>.png          gráfico sobreposto (direto p/ o slide)
Após as capturas: git add sensors/Data/latency && git commit.

Parâmetros ROS:
  robot_ip     ''              IP do CR10; vazio → ~/.config/touch_pack/robot.json
  direction    'auto'          'sim_to_real' | 'real_to_sim' | 'auto'
  duration_s   20.0            janela de captura (0 = até Ctrl-C)
  poll_hz      100.0           taxa de amostragem do feedback real
  joint_index  -1              junta usada na correlação (-1 = a que mais se move)
  grid_dt_s    0.004           passo da grade de reamostragem (resolução do lag)
  max_lag_s    0.6             busca de atraso em ±max_lag_s
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

from sensor_msgs.msg import JointState

from .constants import ARM_JOINTS, ROBOT_CONFIG_FILE, RUNS_DIR

try:
    from .real_driver import CR10RealDriver, CR10RealDriverConfig
    _DRIVER_OK = True
except Exception:  # pragma: no cover
    CR10RealDriver = None
    CR10RealDriverConfig = None
    _DRIVER_OK = False


class LatencyProbe(Node):

    def __init__(self):
        super().__init__('latency_probe')
        self.declare_parameter('robot_ip', '')
        self.declare_parameter('direction', 'auto')
        self.declare_parameter('duration_s', 20.0)
        self.declare_parameter('poll_hz', 100.0)
        self.declare_parameter('joint_index', -1)
        self.declare_parameter('grid_dt_s', 0.004)
        self.declare_parameter('max_lag_s', 0.6)

        self._lock = threading.Lock()
        self._sim: list[tuple[float, list[float]]] = []
        self._real: list[tuple[float, list[float]]] = []
        self._stop = threading.Event()
        self._driver: CR10RealDriver | None = None

        self.create_subscription(
            JointState, '/joint_states', self._cb_joints, 100)

    # ── parâmetros ────────────────────────────────────────────────────
    def _robot_ip(self) -> str:
        ip = str(self.get_parameter('robot_ip').value or '').strip()
        if ip:
            return ip
        try:
            with open(ROBOT_CONFIG_FILE) as fh:
                ip = str(json.load(fh).get('robot_ip', '')).strip()
        except (OSError, json.JSONDecodeError, AttributeError):
            ip = ''
        return ip or '192.168.5.2'

    # ── coleta SIM (assinatura /joint_states) ─────────────────────────
    def _cb_joints(self, msg: JointState) -> None:
        pos = dict(zip(msg.name, msg.position))
        try:
            q = [float(pos[j]) for j in ARM_JOINTS]
        except KeyError:
            return   # mensagem parcial (mão) — ignora
        with self._lock:
            self._sim.append((time.monotonic(), q))

    # ── coleta REAL (poll do feedback readonly) ───────────────────────
    def start_real_poll(self) -> bool:
        if not _DRIVER_OK or CR10RealDriver is None:
            self.get_logger().error(
                'real_driver indisponível — impossível ler o robô real.')
            return False
        ip = self._robot_ip()
        cfg = CR10RealDriverConfig(readonly=True)
        drv = CR10RealDriver(ip=ip, config=cfg)
        try:
            drv.connect()
            drv.read_joints_urdf_latest()   # sanity read
        except Exception as exc:
            self.get_logger().error(
                f'CR10 real em {ip} indisponível ({exc}) — '
                'confira IP/rede e se o robô está ligado.')
            try:
                drv.close()
            except Exception:
                pass
            return False
        self._driver = drv
        self.get_logger().info(
            f'CR10 real conectado em {ip} (readonly) — lendo feedback.')
        threading.Thread(target=self._real_poll_loop, daemon=True,
                         name='latency-real-poll').start()
        return True

    def _real_poll_loop(self) -> None:
        period = 1.0 / max(1.0, float(self.get_parameter('poll_hz').value))
        drv = self._driver
        assert drv is not None
        t_next = time.monotonic()
        fails = 0
        while not self._stop.is_set():
            t_next += period
            try:
                q = drv.read_joints_urdf_latest()
                with self._lock:
                    self._real.append((time.monotonic(), [float(v) for v in q]))
                fails = 0
            except Exception as exc:
                fails += 1
                if fails >= 20:
                    self.get_logger().error(
                        f'Feedback real perdido ({exc}) — encerrando poll.')
                    break
            sleep = t_next - time.monotonic()
            if sleep > 0:
                self._stop.wait(sleep)
            else:
                t_next = time.monotonic()

    # ── análise ───────────────────────────────────────────────────────
    @staticmethod
    def _xcorr_lag(sig_s: np.ndarray, sig_r: np.ndarray, dt: float,
                   max_lag: int) -> tuple[float, float]:
        """Atraso (s) de sig_r em relação a sig_s por correlação cruzada.

        Retorna (lag_s, corr_pico). lag_s > 0 → REAL atrasado (Sim→Real);
        lag_s < 0 → SIM atrasado (Real→Sim). Refino parabólico sub-amostra.
        """
        s = sig_s - sig_s.mean()
        r = sig_r - sig_r.mean()
        n = len(s)
        lags = np.arange(-max_lag, max_lag + 1)
        scores = np.full(len(lags), -2.0)
        for i, d in enumerate(lags):
            # r[k] ≈ s[k-d]  →  d>0 significa REAL atrasado em relação ao SIM
            if d >= 0:
                a, b = (s[:n - d] if d else s), r[d:]
            else:
                a, b = s[-d:], r[:n + d]
            m = min(len(a), len(b))
            if m < 20:
                continue
            a, b = a[:m], b[:m]
            if a.std() < 1e-9 or b.std() < 1e-9:
                continue
            scores[i] = float(np.corrcoef(a, b)[0, 1])
        ki = int(np.argmax(scores))
        best_k = float(lags[ki])
        peak = float(scores[ki])
        if 0 < ki < len(lags) - 1:
            y0, y1, y2 = scores[ki - 1], scores[ki], scores[ki + 1]
            denom = (y0 - 2 * y1 + y2)
            if abs(denom) > 1e-12:
                best_k += 0.5 * (y0 - y2) / denom
        return best_k * dt, peak

    def analyze(self):
        with self._lock:
            sim = list(self._sim)
            real = list(self._real)
        if len(sim) < 50 or len(real) < 50:
            self.get_logger().error(
                f'Amostras insuficientes (sim={len(sim)}, real={len(real)}). '
                'O robô se moveu durante a captura?')
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
        S = np.column_stack([np.interp(grid, t_sim, q_sim[:, j]) for j in range(6)])
        R = np.column_stack([np.interp(grid, t_real, q_real[:, j]) for j in range(6)])

        if j_req < 0 or j_req > 5:
            var = np.var(S, axis=0) + np.var(R, axis=0)
            j = int(np.argmax(var))
        else:
            j = j_req

        max_lag = int(round(max_lag_s / dt))
        amp_deg = float(np.degrees(np.std(R[:, j])))
        lag_s, peak = self._xcorr_lag(S[:, j], R[:, j], dt, max_lag)

        # Lag por junta (todas com movimento mensurável) — vai para o JSON:
        # redundância para a análise posterior validar o número principal.
        per_joint: dict[str, dict] = {}
        for jj in range(6):
            amp_jj = float(np.degrees(np.std(R[:, jj])))
            if amp_jj < 0.05:
                continue   # junta parada — correlação sem significado
            l_jj, c_jj = self._xcorr_lag(S[:, jj], R[:, jj], dt, max_lag)
            per_joint[ARM_JOINTS[jj]] = {
                'lag_ms': round(l_jj * 1e3, 2),
                'peak_corr': round(c_jj, 4),
                'amp_deg': round(amp_jj, 3),
            }

        direction = str(self.get_parameter('direction').value).strip().lower()
        if lag_s >= 0:
            leads, latency_s, detected = 'SIM', lag_s, 'sim_to_real'
        else:
            leads, latency_s, detected = 'REAL', -lag_s, 'real_to_sim'

        return {
            'grid': grid, 'S': S, 'R': R, 'joint': j, 'joint_name': ARM_JOINTS[j],
            'lag_s': lag_s, 'latency_ms': latency_s * 1e3, 'leads': leads,
            'peak_corr': peak, 'amp_deg': amp_deg, 'detected': detected,
            'requested': direction, 'n_sim': len(sim), 'n_real': len(real),
            'dur_s': float(grid[-1] - grid[0]), 'per_joint': per_joint,
        }

    def report_and_save(self, res: dict) -> None:
        jn = res['joint_name']
        self.get_logger().info('─' * 60)
        self.get_logger().info(
            f"LATÊNCIA medida: {res['latency_ms']:.1f} ms "
            f"({res['leads']} adianta → fluxo {res['detected'].replace('_', '-')})")
        self.get_logger().info(
            f"  junta usada: {jn} | amplitude do movimento: {res['amp_deg']:.2f}° | "
            f"correlação de pico: {res['peak_corr']:.3f}")
        self.get_logger().info(
            f"  janela: {res['dur_s']:.1f} s | amostras sim={res['n_sim']} "
            f"real={res['n_real']}")
        if res['requested'] in ('sim_to_real', 'real_to_sim') \
                and res['requested'] != res['detected']:
            self.get_logger().warning(
                f"  ATENÇÃO: você pediu '{res['requested']}' mas o sinal indica "
                f"'{res['detected']}'. Confira o modo/direção da captura.")
        if res['amp_deg'] < 0.5:
            self.get_logger().warning(
                '  Movimento muito pequeno (<0.5°) — mova mais o braço para um '
                'número confiável.')
        if res['peak_corr'] < 0.9:
            self.get_logger().warning(
                '  Correlação de pico baixa (<0.9) — sinal ruidoso; considere um '
                'movimento mais amplo/lento e repetir.')

        # ── Artefatos publicáveis (sensors/Data/latency/ é versionado) ───
        out_dir = os.path.join(RUNS_DIR, 'latency')
        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        tag = res['detected']
        base = os.path.join(out_dir, f'latency_{tag}_{ts}')

        # 1) Séries BRUTAS (6 juntas, taxas nativas, relógio comum) — é o que
        #    permite refazer qualquer análise depois, sem repetir a bancada.
        with self._lock:
            sim_raw = list(self._sim)
            real_raw = list(self._real)
        raw_path = base + '_raw.csv'
        with open(raw_path, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['t_mono_s', 'source'] + [f'{jn}_rad' for jn in ARM_JOINTS])
            for t, q in sim_raw:
                w.writerow([f'{t:.6f}', 'sim'] + [f'{v:.6f}' for v in q])
            for t, q in real_raw:
                w.writerow([f'{t:.6f}', 'real'] + [f'{v:.6f}' for v in q])
        self.get_logger().info(f'  Séries brutas: {raw_path}')

        # 2) Par alinhado da junta usada (pronto para plotar).
        csv_path = base + '_aligned.csv'
        grid, S, R = res['grid'], res['S'], res['R']
        t0 = grid[0]
        with open(csv_path, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['t_rel_s', f'sim_{res["joint_name"]}_deg',
                        f'real_{res["joint_name"]}_deg'])
            for k in range(len(grid)):
                w.writerow([f'{grid[k] - t0:.4f}',
                            f'{np.degrees(S[k, res["joint"]]):.4f}',
                            f'{np.degrees(R[k, res["joint"]]):.4f}'])
        self.get_logger().info(f'  Par alinhado: {csv_path}')

        # 3) Resultado + metadados completos (JSON) — autossuficiente para a
        #    análise posterior preencher os "___ ms" do TCC/slides.
        meta = {
            'timestamp': ts,
            'direction_requested': res['requested'],
            'direction_detected': res['detected'],
            'latency_ms': round(res['latency_ms'], 2),
            'lag_s_signed': round(res['lag_s'], 5),
            'sign_convention': ('lag>0: REAL atrasa (Sim-to-Real); '
                                'lag<0: SIM atrasa (Real-to-Sim)'),
            'peak_corr': round(res['peak_corr'], 4),
            'joint_used': res['joint_name'],
            'movement_amp_deg': round(res['amp_deg'], 3),
            'per_joint': res['per_joint'],
            'n_samples_sim': res['n_sim'],
            'n_samples_real': res['n_real'],
            'overlap_duration_s': round(res['dur_s'], 2),
            'poll_hz': float(self.get_parameter('poll_hz').value),
            'grid_dt_s': float(self.get_parameter('grid_dt_s').value),
            'max_lag_s': float(self.get_parameter('max_lag_s').value),
            'robot_ip': self._robot_ip(),
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
            shift = res['lag_s']   # s>0: real atrasa → adianta o real p/ visualizar o casamento
            fig, ax = plt.subplots(figsize=(9, 4.2))
            ax.plot(t, sim_deg, label='Gêmeo digital (sim)', lw=1.6)
            ax.plot(t, real_deg, label='Braço físico (real)', lw=1.6, alpha=0.85)
            ax.plot(t + shift, real_deg, '--', lw=1.0, alpha=0.6,
                    label=f'Real deslocado ({res["latency_ms"]:.0f} ms)')
            ax.set_xlabel('Tempo (s)')
            ax.set_ylabel(f'{res["joint_name"]} (°)')
            ax.set_title(
                f'Latência {res["detected"].replace("_", "-")}: '
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
        drv = self._driver
        self._driver = None
        if drv is not None:
            try:
                drv.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LatencyProbe()
    if not node.start_real_poll():
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    duration = float(node.get_parameter('duration_s').value)
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True, name='latency-spin')
    spin_thread.start()

    node.get_logger().info(
        f'Capturando por {duration:.0f} s — MOVIMENTE o braço agora '
        '(Ctrl-C encerra antes).' if duration > 0 else
        'Capturando até Ctrl-C — MOVIMENTE o braço agora.')
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
