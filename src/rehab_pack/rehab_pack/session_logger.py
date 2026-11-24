"""Coleta de dados do rehab_pack: grava cada sessão em disco.

SessionWriter (sem ROS) escreve os arquivos; SessionLogger (nó) liga os tópicos.
Pasta: runs_dir()/<subject_id>/<AAAAMMDD_HHMMSS>_<tarefa>/ com samples.csv,
wrench.csv, events.csv, (emg.csv), meta.json e summary.json.
"""
from __future__ import annotations

import csv
import datetime
import json
import math
import os
import time
import re
import subprocess

from .constants import (
    ARM_JOINTS, EVENTS_COLUMNS, GRIP_MASS_KG, PLATE_MASS_KG, SAMPLES_COLUMNS,
    STATES, T_EMG, T_EVENT, T_FT, T_FT_CONFIG, T_JOINT_STATES, T_PARAMS, T_SESSION_META,
    T_SESSION_SUMMARY, T_TARGET, T_TICK, TCP_OFFSET_M, S_SESSION_START,
    S_SESSION_STOP, SIM_ROBOT_IP, TICK_FIELDS, T_REAL_STATUS, WRENCH_COLUMNS, runs_dir)
from .tasks import validate_subject_id
from .session_stats import resumir_arquivos

NAN = float('nan')
FLUSH_S = 1.0
_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _git(*args):
    try:
        r = subprocess.run(['git', *args], cwd=_PKG_DIR, capture_output=True,
                           text=True, timeout=3)
        return r.stdout.strip() if r.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        return None


def info_codigo():
    commit = _git('rev-parse', 'HEAD')
    sujo = _git('status', '--porcelain')
    try:
        from importlib.metadata import version
        versao = version('rehab_pack')
    except Exception:
        versao = 'desconhecido'
    return {'git_commit': commit or 'desconhecido',
            'git_dirty': (bool(sujo) if sujo is not None else 'desconhecido'),
            'versao_pacote': versao}


def _seguro(texto):
    s = re.sub(r'[^A-Za-z0-9_-]+', '_', str(texto)).strip('_')
    return s or 'livre'


def _diff_params(antigo, novo):
    return {k: v for k, v in novo.items() if antigo.get(k) != v}


class SessionWriter:
    """Escreve uma sessão. Uso: abrir -> escrever_* -> fechar(status)."""

    def __init__(self, base_dir=None):
        self.base_dir = base_dir or runs_dir()
        self.pasta = None
        self.meta = None
        self._arqs = {}
        self._wr = {}
        self._ultimo_flush = 0.0
        self.params = {}
        self.emg_canais = 0

    @property
    def aberta(self):
        return self.pasta is not None

    def abrir(self, meta_usuario, params=None, forca=None, pose_inicial=None,
              emg_canais=0, agora=None):
        """Cria a pasta e os arquivos. meta_usuario: {subject_id, lado_paretico, tarefa, notas}."""
        if self.aberta:
            raise RuntimeError('sessão já aberta')
        sujeito = str(meta_usuario.get('subject_id') or '').strip()
        if not sujeito:
            raise ValueError('subject_id vazio')
        agora = agora or datetime.datetime.now()
        tarefa = str(meta_usuario.get('tarefa') or 'livre')
        sid = f"{agora:%Y%m%d_%H%M%S}_{_seguro(tarefa)}"
        pasta = os.path.join(self.base_dir, _seguro(sujeito), sid)
        os.makedirs(pasta, exist_ok=False)
        self.pasta = pasta
        self.params = dict(params or {})
        self.emg_canais = int(emg_canais)
        self.meta = {
            'schema': 2, 'pack': 'rehab_pack', 'relogio': 'monotonic', 'session_id': sid, 'subject_id': sujeito,
            'lado_paretico': meta_usuario.get('lado_paretico', ''),
            'tarefa': tarefa, 'notas': meta_usuario.get('notas', ''),
            'inicio_iso': agora.isoformat(timespec='seconds'), 'fim_iso': None,
            'status': 'interrompida',
            'codigo': info_codigo(),
            'admitancia': self.params,
            'forca': forca or {},
            'mastro': {'massa_empunhadura_kg': GRIP_MASS_KG,
                       'massa_placa_kg': PLATE_MASS_KG,
                       'tcp_offset_m': TCP_OFFSET_M},
            'pose_inicial_rad': list(pose_inicial) if pose_inicial else None,
            'emg_canais': self.emg_canais, 'arquivos': [],
        }
        cab = {'samples.csv': SAMPLES_COLUMNS, 'wrench.csv': WRENCH_COLUMNS,
               'events.csv': EVENTS_COLUMNS}
        if self.emg_canais > 0:
            cab['emg.csv'] = ('t_s',) + tuple(f'emg_{i}_mv' for i in range(1, self.emg_canais + 1))
        for nome, colunas in cab.items():
            f = open(os.path.join(pasta, nome), 'w', newline='', encoding='utf-8')
            w = csv.writer(f)
            w.writerow(colunas)
            self._arqs[nome], self._wr[nome] = f, w
        self.meta['arquivos'] = list(cab) + ['meta.json', 'summary.json']
        self._gravar_meta()   # já deixa um meta válido ('interrompida') caso o processo morra
        return pasta

    def escrever_tick(self, tick, alvo=None, q=None, q_real=None):
        """tick: len(TICK_FIELDS) números; alvo: (x,y,z) ou None; q: 6 rad ou None."""
        if len(tick) != len(TICK_FIELDS):
            raise ValueError(f'tick com {len(tick)} campos, esperado {len(TICK_FIELDS)}')
        self._wr['samples.csv'].writerow(
            list(tick) + list(alvo if alvo is not None else (NAN,) * 3)
            + list(q if q is not None else (NAN,) * 6)
            + list(q_real if q_real is not None else (NAN,) * 6))

    def escrever_wrench(self, t_s, forca, torque, frame):
        self._wr['wrench.csv'].writerow([t_s, *forca, *torque, frame])

    def escrever_evento(self, t_s, tipo, detalhe=''):
        self._wr['events.csv'].writerow([t_s, tipo, detalhe])

    def escrever_emg(self, t_s, valores):
        if 'emg.csv' in self._wr:
            self._wr['emg.csv'].writerow([t_s, *valores[:self.emg_canais]])

    def atualizar_params(self, t_s, novos):
        """Grava evento PARAMS se algum parâmetro mudou no meio da sessão."""
        mudou = _diff_params(self.params, novos)
        if mudou:
            self.escrever_evento(t_s, 'PARAMS', ';'.join(f'{k}={v}' for k, v in mudou.items()))
            self.params = dict(novos)

    def flush(self):
        for f in self._arqs.values():
            f.flush()

    def flush_periodico(self, agora_s):
        if agora_s - self._ultimo_flush >= FLUSH_S:
            self._ultimo_flush = agora_s
            self.flush()

    def _gravar_meta(self):
        self.meta['admitancia'] = self.params
        with open(os.path.join(self.pasta, 'meta.json'), 'w', encoding='utf-8') as f:
            json.dump(self.meta, f, ensure_ascii=False, indent=2)

    def fechar(self, status):
        """Fecha arquivos, grava meta.json final e summary.json; devolve o resumo."""
        if not self.aberta:
            raise RuntimeError('nenhuma sessão aberta')
        for f in self._arqs.values():
            f.close()
        try:
            resumo = resumir_arquivos(os.path.join(self.pasta, 'samples.csv'),
                                      os.path.join(self.pasta, 'events.csv'))
        except Exception as e:  # resumo falhar não pode perder os dados
            resumo = {'erro': str(e)}
        resumo['session_id'] = self.meta['session_id']
        resumo['status'] = status
        resumo['pasta'] = self.pasta
        with open(os.path.join(self.pasta, 'summary.json'), 'w', encoding='utf-8') as f:
            json.dump(resumo, f, ensure_ascii=False, indent=2)
        self.meta['status'] = status
        self.meta['fim_iso'] = datetime.datetime.now().isoformat(timespec='seconds')
        self._gravar_meta()
        self._arqs, self._wr, self.pasta = {}, {}, None
        return resumo


# ── Nó ROS ───────────────────────────────────────────────────────────────────
def _parse_evento(texto):
    tipo, _, detalhe = texto.partition(';')
    return tipo.strip(), detalhe.strip()


def ler_ft_config(texto):
    """{ft_sign, ft_yaw_deg} do JSON do force_source; None no que não se sabe."""
    try:
        d = json.loads(texto)
    except (TypeError, ValueError):
        d = None
    d = d if isinstance(d, dict) else {}
    return {k: d.get(k) for k in ('ft_sign', 'ft_yaw_deg')}


def main(args=None):
    # trava touch_pack × rehab_pack; sem touch_pack o import falha e o nó não sobe
    from touch_pack.pack_lock import acquire_or_exit
    acquire_or_exit('rehab_pack')
    import rclpy
    from geometry_msgs.msg import PointStamped, WrenchStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32MultiArray, Float64MultiArray, String
    from std_srvs.srv import Trigger

    latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
    idx_abortado = STATES.index('ABORTADO')

    class SessionLogger(Node):
        def __init__(self):
            super().__init__('rehab_logger')
            self.declare_parameter('emg_channels', 0)
            self.declare_parameter('sim_sign', 1.0)
            self.w = SessionWriter()
            self.meta_usuario = {}
            self.params = {}
            self.q = None
            self.alvo = None
            self.ultimo_estado = None
            self.create_subscription(Float64MultiArray, T_TICK, self._tick, 50)
            self.create_subscription(WrenchStamped, T_FT, self._wrench, 200)
            self.create_subscription(String, T_EVENT, self._evento, 50)
            self.create_subscription(String, T_SESSION_META, self._meta, latch)
            self.create_subscription(String, T_PARAMS, self._params, latch)
            self.ft_config = None
            self.create_subscription(
                String, T_FT_CONFIG, lambda m: setattr(self, 'ft_config', m.data), latch)
            self.create_subscription(PointStamped, T_TARGET, self._alvo, 10)
            self.create_subscription(JointState, T_JOINT_STATES, self._js, 20)
            self.create_subscription(Float32MultiArray, T_EMG, self._emg, 50)
            self.pub_resumo = self.create_publisher(String, T_SESSION_SUMMARY, latch)
            self.real, self.real_t = {}, 0.0
            self.create_subscription(String, T_REAL_STATUS, self._real, 5)
            self.create_service(Trigger, S_SESSION_START, self._srv_start)
            self.create_service(Trigger, S_SESSION_STOP, self._srv_stop)
            self.create_timer(FLUSH_S, lambda: self.w.aberta and self.w.flush())
            self.get_logger().info(f'rehab_logger pronto; pasta base: {self.w.base_dir}')

        def _agora(self):
            return time.monotonic()     # o mesmo relógio do t_s da admitância

        # callbacks de estado
        def _meta(self, m):
            try:
                self.meta_usuario = json.loads(m.data)
            except json.JSONDecodeError:
                self.get_logger().warning('session/meta inválido (JSON)')

        def _params(self, m):
            try:
                novos = json.loads(m.data)
            except json.JSONDecodeError:
                return
            if self.w.aberta:
                self.w.atualizar_params(self._agora(), novos)
            self.params = novos

        def _real(self, m):
            try:
                self.real, self.real_t = json.loads(m.data), time.monotonic()
            except json.JSONDecodeError:
                return

        def _alvo(self, m):
            p = m.point
            self.alvo = (p.x, p.y, p.z)

        def _js(self, m):
            pos = dict(zip(m.name, m.position))
            if all(j in pos for j in ARM_JOINTS):
                self.q = [pos[j] for j in ARM_JOINTS]

        # callbacks de dados
        def _tick(self, m):
            d = list(m.data)
            if len(d) != len(TICK_FIELDS):
                return
            estado = int(d[TICK_FIELDS.index('state')])
            anterior, self.ultimo_estado = self.ultimo_estado, estado
            if not self.w.aberta:
                return
            vivo = time.monotonic() - self.real_t < 0.5
            self.w.escrever_tick(d, self.alvo, self.q,
                                 self.real.get('q_real') if vivo else None)
            self.w.flush_periodico(d[0])
            if estado == idx_abortado and anterior != idx_abortado:
                self.w.escrever_evento(d[0], 'ABORT', 'estado=ABORTADO')
                self._fechar('abortada')

        def _wrench(self, m):
            if self.w.aberta:
                self.w.escrever_wrench(
                    self._agora(),
                    (m.wrench.force.x, m.wrench.force.y, m.wrench.force.z),
                    (m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z),
                    m.header.frame_id)

        def _evento(self, m):
            if not self.w.aberta:
                return
            tipo, detalhe = _parse_evento(m.data)
            self.w.escrever_evento(self._agora(), tipo, detalhe)

        def _emg(self, m):
            if self.w.aberta and self.w.emg_canais > 0:
                self.w.escrever_emg(self._agora(), list(m.data))

        # sessão
        def _fechar(self, status):
            resumo = self.w.fechar(status)
            self.pub_resumo.publish(String(data=json.dumps(resumo, ensure_ascii=False)))
            self.get_logger().info(f'sessão fechada ({status}): {resumo.get("pasta")}')
            return resumo

        def _srv_start(self, _req, resp):
            if self.w.aberta:
                resp.success, resp.message = False, 'já existe uma sessão aberta'
                return resp
            if not validate_subject_id(str(self.meta_usuario.get('subject_id', ''))):
                resp.success, resp.message = False, 'subject_id ausente ou inválido em /rehab/session/meta'
                return resp
            fonte = self.params.get('force_source', 'desconhecida')
            topicos = {'sim': '/sim/load_cell/wrench',
                       'real': '/ft_sensor/wrench'}
            forca = {'fonte': fonte, 'topico_origem': topicos.get(fonte, 'desconhecido'),
                     'sim_sign': self.get_parameter('sim_sign').value,
                     # sem force_source no ar (ou com dois) não se sabe qual valeu: null
                     **ler_ft_config(self.ft_config
                                     if self.count_publishers(T_FT_CONFIG) == 1 else None),
                     'robo': {k: self.real.get(k) for k in ('modo', 'conectado', 'ip')}}
            # ensaio com robô e célula reais x qualquer coisa simulada
            forca['simulado'] = not (fonte == 'real' and self.real.get('conectado')
                                     and self.real.get('modo') == 'MIRROR'
                                     and self.real.get('ip') != SIM_ROBOT_IP)
            try:
                pasta = self.w.abrir(
                    self.meta_usuario, self.params, forca, self.q,
                    int(self.get_parameter('emg_channels').value))
            except (OSError, ValueError) as e:
                resp.success, resp.message = False, f'falha ao abrir sessão: {e}'
                return resp
            resp.success, resp.message = True, pasta
            return resp

        def _srv_stop(self, _req, resp):
            if not self.w.aberta:
                resp.success, resp.message = False, 'nenhuma sessão aberta'
                return resp
            resumo = self._fechar('concluida')
            resp.success, resp.message = True, resumo['pasta']
            return resp

        def fechar_ao_sair(self):
            if self.w.aberta:
                self._fechar('interrompida')

    rclpy.init(args=args)
    no = SessionLogger()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():               # no Ctrl-C o contexto já caiu: sai sem traceback
            raise
    finally:
        no.fechar_ao_sair()
        no.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
