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
import shutil
import subprocess

from .constants import (
    ARM_JOINTS, EVENTS_COLUMNS, GRIP_MASS_KG, PLATE_MASS_KG, REAL_STATUS_STALE_S, SAMPLES_COLUMNS,
    STATES, T_EMG, T_EVENT, T_FT, T_FT_CONFIG, T_JOINT_STATES, T_PARAMS, T_SESSION_META,
    T_SESSION_REC, T_SESSION_SUMMARY, T_TARGET, T_TICK, TCP_OFFSET_M, S_SESSION_START,
    S_SESSION_STOP, SIM_ROBOT_IP, TICK_FIELDS, T_REAL_STATUS, WRENCH_COLUMNS, runs_dir)
from .tasks import validate_subject_id
from .session_stats import resumir_arquivos

NAN = float('nan')
FLUSH_S = 1.0
ABORT_GRACE_S = 0.5     # espera o motivo do aborto e o TASK_END antes de fechar
REC_S = 0.5             # período do aviso de gravação
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


def _sem_nan(o):
    """NaN e infinito não são JSON: viram null."""
    if isinstance(o, dict):
        return {k: _sem_nan(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_sem_nan(v) for v in o]
    return None if isinstance(o, float) and not math.isfinite(o) else o


def _gravar_json(caminho, obj):
    """Grava ao lado e troca: disco cheio no meio não destrói o arquivo anterior."""
    tmp = caminho + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(_sem_nan(obj), f, ensure_ascii=False, indent=2)
    os.replace(tmp, caminho)


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
        self.codigo = info_codigo()     # git leva até 6 s: fora do serviço de início

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
        if not validate_subject_id(sujeito):
            raise ValueError(f'subject_id inválido: {sujeito!r}')
        agora = agora or datetime.datetime.now()
        tarefa = str(meta_usuario.get('tarefa') or 'livre')
        sid = f"{agora:%Y%m%d_%H%M%S}_{_seguro(tarefa)}"
        pasta = os.path.join(self.base_dir, sujeito, sid)
        self.params = dict(params or {})
        self.emg_canais = int(emg_canais)
        self.meta = {
            'schema': 2, 'pack': 'rehab_pack', 'relogio': 'monotonic', 'session_id': sid, 'subject_id': sujeito,
            'lado_paretico': meta_usuario.get('lado_paretico', ''),
            'tarefa': tarefa, 'notas': meta_usuario.get('notas', ''),
            'inicio_iso': agora.isoformat(timespec='seconds'), 'fim_iso': None,
            'status': 'interrompida',
            'codigo': self.codigo,
            'admitancia': dict(self.params),   # os INICIAIS; mudanças viram eventos PARAMS
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
        self.meta['arquivos'] = list(cab) + ['meta.json', 'summary.json']
        os.makedirs(pasta, exist_ok=False)
        try:
            for nome, colunas in cab.items():
                f = self._arqs[nome] = open(os.path.join(pasta, nome), 'w', newline='',
                                            encoding='utf-8')
                self._wr[nome] = csv.writer(f)
                self._wr[nome].writerow(colunas)
                f.flush()          # disco cheio tem de aparecer aqui, não na 1ª amostra
            # já deixa um meta válido ('interrompida') caso o processo morra
            _gravar_json(os.path.join(pasta, 'meta.json'), self.meta)
        except BaseException:
            self._fechar_arquivos()
            shutil.rmtree(pasta, ignore_errors=True)
            raise
        self.pasta = pasta         # só agora está aberta
        return pasta

    def _fechar_arquivos(self):
        """Fecha todos e esquece; devolve os erros (o close também grava)."""
        erros = []
        for f in self._arqs.values():
            try:
                f.close()
            except OSError as e:
                erros.append(str(e))
        self._arqs, self._wr = {}, {}
        return erros

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

    def fechar(self, status):
        """Fecha arquivos, grava meta.json final e summary.json; devolve o resumo."""
        if not self.aberta:
            raise RuntimeError('nenhuma sessão aberta')
        pasta, self.pasta = self.pasta, None    # falhe o que falhar, não fica meio aberta
        erros = self._fechar_arquivos()
        try:
            resumo = resumir_arquivos(*(os.path.join(pasta, n) for n in (
                'samples.csv', 'events.csv', 'wrench.csv')))
        except Exception as e:  # resumo falhar não pode perder os dados
            resumo = {'erro': str(e)}
        resumo['session_id'] = self.meta['session_id']
        resumo['status'] = status
        resumo['pasta'] = pasta
        self.meta['status'] = status
        self.meta['fim_iso'] = datetime.datetime.now().isoformat(timespec='seconds')
        for nome, obj in (('summary.json', resumo), ('meta.json', self.meta)):
            try:
                _gravar_json(os.path.join(pasta, nome), obj)
            except OSError as e:
                erros.append(f'{nome}: {e}')
        if erros:
            resumo['erro_gravacao'] = '; '.join(erros)
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
    # trava touch_pack × rehab_pack
    from .pack_lock import acquire_or_exit
    acquire_or_exit('rehab_pack')
    import rclpy
    from rclpy.clock import Clock, ClockType
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
            self.w = SessionWriter()
            self.meta_usuario = {}
            self.params = {}
            self.q = None
            self.alvo = None
            self.ultimo_estado = None
            self.abortar_em = None      # sessão abortada: quando fechar
            self.robo = {}
            self.rec_t = 0.0
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
            self.pub_rec = self.create_publisher(String, T_SESSION_REC, 5)
            # parede: com o /clock parado o flush e o fechamento parariam junto
            self.create_timer(0.1, self._ronda, clock=Clock(clock_type=ClockType.STEADY_TIME))
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
                self._grava(self.w.atualizar_params, self._agora(), novos)
            self.params = novos

        def _real(self, m):
            try:
                novo = json.loads(m.data)
            except json.JSONDecodeError:
                return
            if isinstance(novo, dict):
                self.real, self.real_t = novo, time.monotonic()

        def _robo(self):
            """Robô real agora; status vencido não prova nada."""
            vivo = time.monotonic() - self.real_t < REAL_STATUS_STALE_S
            return {k: self.real.get(k) if vivo else None for k in ('modo', 'conectado', 'ip')}

        def _grava(self, fn, *a):
            """Escrita na sessão: disco cheio fecha a sessão, não derruba o nó."""
            try:
                fn(*a)
            except OSError as e:
                self.get_logger().error(f'falha de disco, sessão encerrada: {e}')
                self._fechar('erro_disco')

        def _ronda(self):
            agora = time.monotonic()
            if self.w.aberta:
                if self.abortar_em is not None and agora >= self.abortar_em:
                    self._fechar('abortada')
                else:
                    self._grava(self.w.flush_periodico, agora)
            if agora - self.rec_t >= REC_S:
                self.rec_t = agora
                self.pub_rec.publish(String(data=self.w.pasta or ''))

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
            self._grava(self.w.escrever_tick, d, self.alvo, self.q,
                        self.real.get('q_real') if vivo else None)
            robo = self._robo()
            if self.w.aberta and robo != self.robo:
                self.robo = robo
                self.w.meta['forca']['robo_mudou'] = True
                self._grava(self.w.escrever_evento, d[0], 'ROBO',
                            ';'.join(f'{k}={v}' for k, v in robo.items()))
            if self.w.aberta and estado == idx_abortado and anterior != idx_abortado:
                if self.abortar_em is None:
                    self.abortar_em = time.monotonic() + ABORT_GRACE_S
                self._grava(self.w.escrever_evento, d[0], 'ABORT', 'estado=ABORTADO')

        def _wrench(self, m):
            if self.w.aberta:
                self._grava(
                    self.w.escrever_wrench, self._agora(),
                    (m.wrench.force.x, m.wrench.force.y, m.wrench.force.z),
                    (m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z),
                    m.header.frame_id)

        def _evento(self, m):
            if not self.w.aberta:
                return
            tipo, detalhe = _parse_evento(m.data)
            self._grava(self.w.escrever_evento, self._agora(), tipo, detalhe)

        def _emg(self, m):
            if self.w.aberta and self.w.emg_canais > 0:
                self._grava(self.w.escrever_emg, self._agora(), list(m.data))

        # sessão
        def _fechar(self, status):
            resumo = self.w.fechar('abortada' if self.abortar_em is not None else status)
            status, self.abortar_em = resumo['status'], None
            if 'erro_gravacao' in resumo:
                self.get_logger().error(f"sessão com falha de gravação: {resumo['erro_gravacao']}")
            self.pub_resumo.publish(String(data=json.dumps(_sem_nan(resumo), ensure_ascii=False)))
            self.get_logger().info(f'sessão fechada ({status}): {resumo.get("pasta")}')
            return resumo

        def _srv_start(self, _req, resp):
            if self.w.aberta:
                resp.success, resp.message = False, 'já existe uma sessão aberta'
                return resp
            if not validate_subject_id(str(self.meta_usuario.get('subject_id', ''))):
                resp.success, resp.message = False, 'subject_id ausente ou inválido em /rehab/session/meta'
                return resp
            robo, self.alvo, self.abortar_em = self._robo(), None, None
            fonte = self.params.get('force_source', 'desconhecida')
            topicos = {'sim': '/sim/load_cell/wrench',
                       'real': '/ft_sensor/wrench'}
            forca = {'fonte': fonte, 'topico_origem': topicos.get(fonte, 'desconhecido'),
                     # sem force_source no ar (ou com dois) não se sabe qual valeu: null
                     **ler_ft_config(self.ft_config
                                     if self.count_publishers(T_FT_CONFIG) == 1 else None),
                     'robo': robo}
            # ensaio com robô e célula reais x qualquer coisa simulada
            forca['simulado'] = not (fonte == 'real' and robo['conectado']
                                     and robo['modo'] == 'MIRROR'
                                     and robo['ip'] != SIM_ROBOT_IP)
            self.robo = robo
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
