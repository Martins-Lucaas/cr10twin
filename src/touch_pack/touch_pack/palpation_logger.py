"""
palpation_logger.py — Nó ROS 2 que grava cada execução de palpação em disco.

Lê seis tópicos:
  sub /palpation/start     touch_pack_msgs/PalpationStart — parâmetros do
                                               experimento; marca o início de
                                               um "run".
  sub /palpation/status    touch_pack_msgs/PalpationStatus — fase, ciclo e
                                               SETPOINT INSTANTÂNEO atuais
                                               (target_force_n → coluna
                                               setpoint_n); encerra o run em
                                               DONE/ABORTED. É a fonte
                                               autoritativa do setpoint: cobre
                                               modulação, degrau e MANUAL.
  sub /palpation/set_force std_msgs/Float32  setpoint COMANDADO pela GUI. O
                                               explorer republica o valor
                                               vigente no status, então isto é
                                               reserva — vale entre dois
                                               status, ou sem explorer no ar.
  sub /load_cell/sample_net touch_pack_msgs/LoadCellSample — força
                                               tare-compensada (N, compressão
                                               positiva) + seq/t_us do firmware
                                               + tensão CRUA. É o sinal CANÔNICO
                                               e o gatilho da linha (~82 Hz).
  sub /touch_sensor/frame  touch_pack_msgs/TouchFrame — frame de taxels + o
                                               t_us do STM32, republicado pela GUI;
                                               o ÚLTIMO frame é copiado em cada
                                               amostra (colunas taxel_*).
  sub /touch_sensor/spike_event std_msgs/String — um evento por mensagem
                                               (RA|SA|CN_MM|CN_RA|CN_SA); contados
                                               POR amostra (colunas n_RA/n_SA/
                                               cn_mm/cn_ra/cn_sa).
  sub /joint_states        sensor_msgs/JointState — juntas do braço; a pose do
                                               TCP é calculada via FK
                                               (kinematics + T_TOUCH_TOOL_ATTACH).
  sub /palpation/matrix_point touch_pack_msgs/MatrixPoint — um registro por
                                               identação concluída no modo
                                               MATRIX_MAP (vira matrix.csv).

Saída — uma pasta por RUN, dentro da pasta do MODO (ver constants.run_dir):

  sensors/Data/<MODO>/<run_id>/    MODO ∈ SLIDE|TOUCH|MANUAL|MATRIX_MAP
                                   run_id = <AAAAMMDD_HHMMSS> de PAREDE,
                                   carimbado pela GUI no campo run_id da
                                   PalpationStart (a GUI grava sensors.csv,
                                   adc.csv, spikes.csv e cuneiformes.csv na
                                   MESMA pasta).

  samples.csv   uma linha por amostra da célula (~82 Hz). O
      cabeçalho é DINÂMICO em duas dimensões — a grade do sensor (parâmetro
      `sensor`) e o MODO do experimento (ver _mode_columns/_build_header):
        núcleo   t_rel_s, t_unix, mode, cycle, phase (CÓDIGO — ver PHASE_CODES),
                 setpoint_n, force_net_n, lc_seq, lc_t_us, lc_voltage_raw_v,
                 q1..q6, tcp_x/y/z, pose_age_ms, taxel_0..taxel_N,
                 touch_t_us, taxel_age_ms, n_RA, n_SA, cn_mm, cn_ra, cn_sa
        SLIDE    slide_dir, slide_dist_mm, slide_slope_deg, speed_mms
        TOUCH    mod_shape, mod_min_n, mod_max_n, mod_hz, mod_cycles
                 (só com modulação vinda da GUI; sem ela o bloco não existe)
        MANUAL   step_start_n, step_size_n, step_max_n, step_dwell_s
                 (só com a escada ligada)
        MATRIX_MAP  wp_index, wp_x_mm, wp_y_mm, grid_shape
      lc_t_us e touch_t_us são os relógios dos DOIS firmwares — os únicos
      tempos da linha que não passaram pela pilha ROS, e a base de qualquer
      reamostragem posterior num eixo comum.
  params.json   parâmetros do start (inclui o run_id — é o que identifica
      um arquivo levado para fora da pasta)
  matrix.csv    MATRIX_MAP: uma linha por identação — coordenada
      planejada, coordenada MEDIDA do TCP, origem do plano, setpoint pedido e
      força atingida. É a chave que liga cada trecho da curva de força do
      samples.csv (filtrando por wp_index) à posição espacial onde ela foi
      medida. Só é criado se o run publicar pontos.
  summary.json  métricas pós-run (gerado pelo palpation_report)
  plot.png      força×tempo por fase (se matplotlib disponível)

Runs anteriores a este layout continuam soltos na raiz de sensors/Data com o
nome antigo (<ts>__samples.csv); palpation_report e analyze_force_runs leem
os dois.

Encerramento: o run fecha quando recebe status DONE ou ABORTED, ou após
5 min sem amostras de força (timeout de segurança caso o explorer caia).
Ao fechar um run com amostras, o relatório é gerado automaticamente em
background (ver palpation_report.generate_report).
"""
from __future__ import annotations

import csv
import json
import logging
import os
import threading
import time
from typing import IO

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)

from std_msgs.msg import Float32, String
from sensor_msgs.msg import JointState
from rosidl_runtime_py.convert import message_to_ordereddict
from touch_pack_msgs.msg import (
    PalpationStart, PalpationStatus, MatrixPoint, LoadCellSample, TouchFrame)

from .kinematics import forward_kinematics, T_TOUCH_TOOL_ATTACH
from .constants import (
    ARM_JOINTS as _ARM_JOINTS,
    RUNS_DIR as OUTPUT_DIR,
    TOUCH_FRAME_TOPIC, TOUCH_EVENT_TOPIC,
    TOUCH_ROWS_DEFAULT, TOUCH_COLS_DEFAULT, TOUCH_EVENT_TYPES,
    PHASE_CODES, run_dir, run_id_from_msg,
    RUN_SAMPLES_CSV, RUN_MATRIX_CSV, RUN_PARAMS_JSON,
)


log = logging.getLogger('touch_pack.palpation_logger')

RUN_IDLE_TIMEOUT_S = 300.0   # 5 min sem força → fecha run "perdido"


def _rel(path: str) -> str:
    """Caminho do arquivo relativo à pasta de dados, para o log. Agora que
    todo run tem os mesmos nomes de arquivo, só o basename ('samples.csv')
    não diz de qual run se está falando."""
    try:
        return os.path.relpath(path, OUTPUT_DIR)
    except (ValueError, TypeError):
        return str(path)

# CSV unificado do experimento (1 linha por amostra de força, ~1 kHz):
#   tempo + ciclo + fase NUMÉRICA + setpoint + força + juntas + TCP +
#   25 taxels (último frame ADC) + contagem de eventos por amostra.
_EVENT_COLS = ['n_RA', 'n_SA', 'cn_mm', 'cn_ra', 'cn_sa']


def _mode_columns(mode: str, msg) -> tuple[list[str], list[str]]:
    """Colunas EXTRA e seus valores (constantes no run) para este modo.

    Cada experimento de palpação mede uma coisa diferente, e um único
    `setpoint_n` não descreve todos: numa senoide não existe alvo único — o
    que caracteriza o ensaio é a excursão entre f_min e f_max e a frequência
    com que ela é percorrida (o próprio explorer diz isso em
    _phase_hold_modulated). Sem estas colunas o CSV de um run senoidal é
    indistinguível de um run de força constante que oscilou.

    São constantes por run e portanto redundantes com o params.json — de
    propósito, pela mesma razão que wp_x_mm/wp_y_mm já se repetiam: o
    samples.csv precisa ser legível sozinho.
    """
    def f(name, default=0.0):
        try:
            return float(getattr(msg, name))
        except (AttributeError, TypeError, ValueError):
            return default

    if mode == 'MATRIX_MAP':
        # wp_index casa com a coluna `index` do __matrix.csv — filtrar por ele
        # recorta a curva de força de UMA identação. Estas três são as únicas
        # do bloco que variam ao longo do run (vêm do /palpation/status).
        return (['wp_index', 'wp_x_mm', 'wp_y_mm', 'grid_shape'],
                [])          # preenchidas por amostra, ver _wp_cols

    if mode == 'TOUCH':
        shape = str(getattr(msg, 'force_mod_shape', '') or '').upper().strip()
        if shape in ('SINE', 'COSINE'):
            # Modulação vinda da GUI. shape == '' significa "usar os
            # parâmetros ROS force_mod_*", que o logger não enxerga — nesse
            # caso não há bloco, e setpoint_n (a onda ENTREGUE) segue valendo.
            return (['mod_shape', 'mod_min_n', 'mod_max_n', 'mod_hz',
                     'mod_cycles'],
                    [shape, f'{f("force_mod_min_n"):.4f}',
                     f'{f("force_mod_max_n"):.4f}', f'{f("force_mod_hz"):.4f}',
                     str(int(f('force_mod_cycles')))])
        return ([], [])

    if mode == 'MANUAL':
        if f('step_size_n') > 0.0:
            # Escada de força: sobe e DESCE pelos mesmos patamares. Sem estes
            # campos não dá para separar o ramo de carga do de descarga, que é
            # justamente o que a ida-e-volta mede (histerese/relaxação).
            return (['step_start_n', 'step_size_n', 'step_max_n',
                     'step_dwell_s'],
                    [f'{f("step_start_n"):.4f}', f'{f("step_size_n"):.4f}',
                     f'{f("step_max_n"):.4f}', f'{f("step_dwell_s"):.4f}'])
        return ([], [])

    # SLIDE (default): a geometria do deslize é o que define o ensaio.
    return (['slide_dir', 'slide_dist_mm', 'slide_slope_deg', 'speed_mms'],
            [str(getattr(msg, 'slide_dir', '') or ''),
             f'{f("slide_dist_mm"):.3f}', f'{f("slide_slope_deg"):.3f}',
             f'{f("speed_mms"):.3f}'])


def _build_header(n_taxels: int, mode_cols: list[str]) -> list[str]:
    """Cabeçalho do samples.csv. Dinâmico em duas dimensões: a grade do
    sensor (4×4 ou 5×5, do parâmetro `sensor`) e o modo do experimento."""
    return (['t_rel_s', 't_unix', 'mode', 'cycle', 'phase',
             'setpoint_n', 'force_net_n',
             # Carimbo do firmware da célula: seq detecta amostra perdida,
             # t_us é o ÚNICO tempo desta linha que não passou por 2 saltos
             # ROS. lc_voltage_raw_v é a tensão SEM o One-Euro, que permite
             # refazer a força offline sem o atraso do filtro (~49 ms de t90).
             'lc_seq', 'lc_t_us', 'lc_voltage_raw_v']
            + [f'q{i}' for i in range(1, 7)]
            + ['tcp_x', 'tcp_y', 'tcp_z', 'pose_age_ms']
            + [f'taxel_{i}' for i in range(n_taxels)]
            + ['touch_t_us', 'taxel_age_ms']
            + _EVENT_COLS
            + mode_cols)

# __matrix.csv — uma linha por identação concluída (msg MatrixPoint).
# plan_* é o que a GUI pediu; tcp_* é onde o toque realmente aconteceu (FK do
# feedback). rel_x/rel_y são o MEDIDO relativo à origem — comparados a plan_*
# dão o erro de posicionamento de cada ponto do mapa.
MATRIX_CSV_HEADER = [
    'index', 'total',
    'plan_x_mm', 'plan_y_mm',
    'rel_x_mm', 'rel_y_mm',
    'depth_mm',
    'setpoint_n', 'force_n', 'force_err_n',
    'tcp_x', 'tcp_y', 'tcp_z',
    'origin_x', 'origin_y', 'origin_z',
    't_start_unix', 't_end_unix', 't_rel_start_s', 't_rel_end_s',
    'outcome',
]

_QOS_COMMAND = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
_QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class PalpationLogger(Node):

    def __init__(self):
        super().__init__('palpation_logger')

        os.makedirs(OUTPUT_DIR, exist_ok=True)

        # Grade do sensor vinda do MESMO parâmetro `sensor` do launch que a
        # GUI usa. Antes isto era a constante TOUCH_TAXELS_DEFAULT: com o
        # launch em 4×4 a GUI publicava 16 taxels, o logger exigia 25 e
        # descartava 100% dos frames — o run inteiro saía com taxel_* vazios e
        # só um warning estrangulado dizia por quê.
        _sensor = str(self.declare_parameter('sensor', '5').value).strip()
        if _sensor == '4':
            self._rows, self._cols = 4, 4
        else:
            self._rows, self._cols = TOUCH_ROWS_DEFAULT, TOUCH_COLS_DEFAULT
        self._n_taxels = self._rows * self._cols

        self._lock = threading.Lock()
        self._csv_fh: IO | None = None
        self._csv_writer: csv.writer | None = None
        self._run_path: str | None = None
        self._run_t0: float | None = None
        self._phase: str = 'IDLE'
        self._cycle: int = 0
        self._last_sample_t: float = 0.0
        self._sample_count: int = 0
        # Últimas juntas do braço (rad, ordem _ARM_JOINTS); None até o
        # primeiro /joint_states.
        self._q: np.ndarray | None = None
        self._q_cols: list[str] = [''] * 6
        self._tcp_cols: list[str] = [''] * 3
        # Chegada do último /joint_states → pose_age_ms. Simétrico ao
        # taxel_age_ms: sem ele não há como auditar depois quão velha estava a
        # pose gravada nesta linha, e medido em 07/08/2026 a pose chegava a
        # 3,3 Hz num DESCENDING enquanto a força saía a 82 Hz.
        self._q_ts: float = 0.0
        # Setpoint de força do run (force_n do start) — vai numa coluna do CSV.
        self._setpoint: float | None = None
        # MATRIX_MAP: carimbo espacial corrente, vindo do /palpation/status.
        self._wp_cols: list[str] = ['', '', '']
        # __matrix.csv — aberto sob demanda, na PRIMEIRA identação do run.
        self._matrix_fh: IO | None = None
        self._matrix_writer: csv.writer | None = None
        self._matrix_path: str | None = None
        self._matrix_rows: int = 0
        # Identidade do run corrente e a pasta onde TODOS os seus arquivos
        # moram — o matrix.csv é escrito depois do start e precisa dela.
        self._run_stamp: str | None = None
        self._run_dir: str | None = None
        # Tátil completo (republicado pela GUI). _adc_cols = último frame ADC já
        # formatado (25 colunas); _evt_counts conta eventos POR amostra (zerado
        # a cada linha escrita no _cb_lc_sample).
        self._adc_cols: list[str] = [''] * self._n_taxels
        # Chegada (relógio do PC) do último frame tátil — vira taxel_age_ms.
        self._adc_ts: float = 0.0
        # t_us do STM32 do último frame — o carimbo de HARDWARE do toque.
        self._adc_t_us: int = 0
        # Frames descartados por tamanho errado (ver _cb_touch_frame).
        self._adc_bad: int = 0
        self._evt_counts: dict[str, int] = {t: 0 for t in TOUCH_EVENT_TYPES}
        # Colunas extra do modo corrente (constantes no run) e o próprio modo.
        self._mode: str = ''
        self._mode_vals: list[str] = []
        self._mode_cols: list[str] = []

        self.create_subscription(
            PalpationStart, '/palpation/start', self._cb_start, _QOS_COMMAND)
        self.create_subscription(
            PalpationStatus, '/palpation/status', self._cb_status, 10)
        # Setpoint atualizado on-the-fly (modo MANUAL): atualiza a coluna
        # setpoint_n das amostras seguintes → o CSV segue a série real.
        self.create_subscription(
            Float32, '/palpation/set_force', self._cb_set_force, 10)
        # Gatilho da linha: a amostra CARIMBADA da célula. Era
        # /load_cell/force_net (Float32), que chegava sem seq nem t_us — a
        # linha ficava datada só pela hora de chegada no logger, depois de
        # 2 saltos ROS. O force_net continua publicado para o explorer e o
        # force_sync; aqui usamos a versão que traz o carimbo junto.
        self.create_subscription(
            LoadCellSample, '/load_cell/sample_net', self._cb_lc_sample,
            _QOS_SENSOR)
        self.create_subscription(
            TouchFrame, TOUCH_FRAME_TOPIC, self._cb_touch_frame, _QOS_SENSOR)
        self.create_subscription(
            String, TOUCH_EVENT_TOPIC, self._cb_event, _QOS_SENSOR)
        self.create_subscription(
            JointState, '/joint_states', self._cb_joints, 50)
        # Uma mensagem por identação do MATRIX_MAP. Fila funda: perder um
        # ponto significa perder o vínculo entre uma curva de força e a
        # coordenada onde ela foi medida.
        self.create_subscription(
            MatrixPoint, '/palpation/matrix_point', self._cb_matrix_point, 50)

        # Watchdog @1 Hz para fechar runs órfãos.
        self.create_timer(1.0, self._watchdog)

        self.get_logger().info(
            f'palpation_logger ativo — gravando em {OUTPUT_DIR}/')

    # Callbacks
    def _cb_start(self, msg: PalpationStart) -> None:
        """Início de um novo run: cria CSV e dump dos parâmetros em JSON.
        O msg tipado vira dict (mesmas chaves dos campos) para o
        __params.json — o palpation_report lê 'force_n' etc. de lá."""
        try:
            params = dict(message_to_ordereddict(msg))
            params['home_deg'] = list(params.get('home_deg', []))
        except Exception:
            params = {}
        try:
            setpoint = float(getattr(msg, 'force_n'))
        except (AttributeError, TypeError, ValueError):
            setpoint = None

        # Modo do experimento — vazio = SLIDE (compatibilidade, ver
        # PalpationStart.msg). Ele escolhe o bloco de colunas do run.
        mode = str(getattr(msg, 'mode', '') or '').upper().strip() or 'SLIDE'
        if mode not in ('SLIDE', 'TOUCH', 'MANUAL', 'MATRIX_MAP'):
            mode = 'SLIDE'
        mode_cols, mode_vals = _mode_columns(mode, msg)
        grid_shape = str(getattr(msg, 'grid_shape', '') or '')

        with self._lock:
            self._close_run_locked('superseded')
            # Mesma pasta que a GUI usa para os CSVs crus do run — é o que
            # faz samples.csv e sensors.csv caírem juntos. O run_id vem da
            # mensagem justamente para os dois não dependerem de chamar
            # strftime no mesmo segundo.
            ts = run_id_from_msg(msg)
            try:
                # base=OUTPUT_DIR e não o default: é o alias de módulo que o
                # teste redireciona para um tmpdir.
                out_dir = run_dir(mode, ts, base=OUTPUT_DIR)
            except OSError as exc:
                self.get_logger().error(
                    f'Falha ao criar a pasta do run: {exc}')
                return
            csv_path = os.path.join(out_dir, RUN_SAMPLES_CSV)
            json_path = os.path.join(out_dir, RUN_PARAMS_JSON)
            fh = None
            try:
                fh = open(csv_path, 'w', newline='')
                writer = csv.writer(fh)
                writer.writerow(_build_header(self._n_taxels, mode_cols))
                with open(json_path, 'w') as pf:
                    json.dump(params, pf, indent=2, sort_keys=True)
            except OSError as exc:
                self.get_logger().error(
                    f'Falha ao criar arquivos de run: {exc}')
                if fh is not None:
                    fh.close()
                return
            self._csv_fh = fh
            self._csv_writer = writer
            self._run_path = csv_path
            self._run_stamp = ts
            self._run_dir = out_dir
            self._run_t0 = time.time()
            self._last_sample_t = self._run_t0
            self._sample_count = 0
            self._phase = 'IDLE'
            self._cycle = 0
            self._setpoint = setpoint
            self._mode = mode
            self._mode_cols = mode_cols
            self._mode_vals = mode_vals
            # MATRIX_MAP é o único bloco que VARIA por amostra: wp_* vem do
            # /palpation/status a cada ponto da grade. grid_shape é constante.
            self._wp_cols = (['', '', '', grid_shape]
                             if mode == 'MATRIX_MAP' else [])
            self._adc_cols = [''] * self._n_taxels
            self._adc_ts = 0.0
            self._adc_t_us = 0
            self._adc_bad = 0
            self._q_ts = 0.0
            self._evt_counts = {t: 0 for t in TOUCH_EVENT_TYPES}
            extra = (f' | {mode} + [{", ".join(mode_cols)}]'
                     if mode_cols else f' | {mode}')
            self.get_logger().info(
                f'Run iniciado → {_rel(csv_path)} '
                f'(setpoint={setpoint if setpoint is not None else "?"} N'
                f'{extra}, grade {self._rows}×{self._cols})')

    def _cb_set_force(self, msg: Float32) -> None:
        """Novo setpoint de força on-the-fly (modo MANUAL). Atualiza o valor
        gravado na coluna setpoint_n das amostras seguintes — assim o CSV (e o
        gráfico/summary derivados) refletem a série real de alvos, não só o
        valor do start. Só tem efeito com run ativo."""
        with self._lock:
            if self._csv_fh is None:
                return
            try:
                self._setpoint = float(msg.data)
            except (TypeError, ValueError):
                return

    def _cb_status(self, msg: PalpationStatus) -> None:
        with self._lock:
            self._phase = msg.phase
            self._cycle = int(msg.cycle)
            # Setpoint INSTANTÂNEO do experimento. É esta a fonte autoritativa
            # da coluna setpoint_n: o explorer publica aqui o alvo corrente —
            # a onda entregue sob modulação, o patamar corrente no DEGRAU, o
            # valor vigente no MANUAL. Antes o logger só ouvia
            # /palpation/set_force, que a GUI usa para COMANDAR e que o
            # explorer nunca publica: por isso a coluna saía constante no
            # valor do start, mesmo em runs senoidais.
            sp = getattr(msg, 'target_force_n', None)
            if sp is not None:
                try:
                    self._setpoint = float(sp)
                except (TypeError, ValueError):
                    pass
            # MATRIX_MAP: wp_index > 0 identifica o ponto da grade que está
            # sendo executado AGORA. Fora deste modo o bloco não existe no
            # cabeçalho, então não há nada a atualizar.
            if self._mode == 'MATRIX_MAP' and len(self._wp_cols) == 4:
                grid_shape = self._wp_cols[3]
                wp = int(getattr(msg, 'wp_index', 0) or 0)
                if wp > 0:
                    self._wp_cols = [
                        str(wp),
                        f'{float(getattr(msg, "wp_x_mm", 0.0)):.3f}',
                        f'{float(getattr(msg, "wp_y_mm", 0.0)):.3f}',
                        grid_shape,
                    ]
                else:
                    self._wp_cols = ['', '', '', grid_shape]
            if msg.phase in ('DONE', 'ABORTED', 'FROZEN') \
                    and self._csv_fh is not None:
                self._close_run_locked(msg.phase)

    def _cb_matrix_point(self, msg: MatrixPoint) -> None:
        """Uma identação do MATRIX_MAP terminou: grava a linha do
        __matrix.csv. O arquivo é criado na primeira mensagem do run.
        """
        with self._lock:
            if self._csv_fh is None or self._run_t0 is None:
                return   # sem run ativo: ponto órfão, descartado
            if self._matrix_writer is None and not self._open_matrix_locked():
                return
            origin = (float(msg.origin_x_m), float(msg.origin_y_m),
                      float(msg.origin_z_m))
            tcp = (float(msg.tcp_x_m), float(msg.tcp_y_m), float(msg.tcp_z_m))
            # Medido relativo à origem — é a coordenada REAL do toque no
            # sistema do plano, contra a qual plan_* é o pedido.
            rel_x_mm = (tcp[0] - origin[0]) * 1e3
            rel_y_mm = (tcp[1] - origin[1]) * 1e3
            setpoint = float(msg.setpoint_n)
            force = float(msg.force_n)
            try:
                self._matrix_writer.writerow([
                    int(msg.index), int(msg.total),
                    f'{float(msg.plan_x_mm):.3f}', f'{float(msg.plan_y_mm):.3f}',
                    f'{rel_x_mm:.3f}', f'{rel_y_mm:.3f}',
                    f'{float(msg.depth_mm):.4f}',
                    f'{setpoint:.4f}', f'{force:.4f}',
                    f'{force - setpoint:.4f}',
                    f'{tcp[0]:.6f}', f'{tcp[1]:.6f}', f'{tcp[2]:.6f}',
                    f'{origin[0]:.6f}', f'{origin[1]:.6f}', f'{origin[2]:.6f}',
                    f'{float(msg.t_start_unix):.4f}',
                    f'{float(msg.t_end_unix):.4f}',
                    f'{float(msg.t_start_unix) - self._run_t0:.4f}',
                    f'{float(msg.t_end_unix) - self._run_t0:.4f}',
                    str(msg.outcome),
                ])
                self._matrix_rows += 1
                # Uma identação leva segundos: o flush por linha é barato e
                # garante o mapa em disco mesmo se o nó morrer no meio da grade.
                if self._matrix_fh is not None:
                    self._matrix_fh.flush()
            except (ValueError, OSError) as exc:
                self.get_logger().warn(
                    f'Falha ao gravar ponto da matriz: {exc}')

    def _open_matrix_locked(self) -> bool:
        """Cria o matrix.csv do run. Chamar com `self._lock`. False em falha."""
        if self._run_dir is None:
            return False
        path = os.path.join(self._run_dir, RUN_MATRIX_CSV)
        try:
            fh = open(path, 'w', newline='')
            writer = csv.writer(fh)
            writer.writerow(MATRIX_CSV_HEADER)
        except OSError as exc:
            self.get_logger().error(f'Falha ao criar {path}: {exc}')
            return False
        self._matrix_fh = fh
        self._matrix_writer = writer
        self._matrix_path = path
        self._matrix_rows = 0
        self.get_logger().info(
            f'MATRIX_MAP detectado → {_rel(path)}')
        return True

    def _cb_touch_frame(self, msg: TouchFrame) -> None:
        """Frame de taxels COM o t_us do STM32, republicado pela GUI. Guarda as
        colunas já formatadas — cada amostra da célula copia o ÚLTIMO frame.

        Frame com tamanho != self._n_taxels é DESCARTADO. Antes ele era
        completado com colunas vazias, o que gravava um frame corrompido como
        se fosse um frame parcial legítimo (07/08/2026: 19% das linhas de um
        run saíram assim). O publicador já filtra na origem; isto é a segunda
        barreira, e a que garante que nenhum `taxel_*` vazio no CSV signifique
        outra coisa além de "nenhum frame chegou ainda"."""
        vals = list(msg.taxels)
        if len(vals) != self._n_taxels:
            with self._lock:
                self._adc_bad += 1
                n = self._adc_bad
            if n == 1 or n % 100 == 0:
                self.get_logger().warn(
                    f'frame tátil com {len(vals)} taxels (esperado '
                    f'{self._n_taxels}, grade {self._rows}×{self._cols}) '
                    f'descartado — {n} até agora neste nó. Confira se o '
                    f"parâmetro `sensor` do launch bate com o sensor montado.")
            return
        cols = [str(int(v)) for v in vals]
        now = time.time()
        with self._lock:
            self._adc_cols = cols
            self._adc_ts = now
            self._adc_t_us = int(msg.t_us)

    def _cb_event(self, msg: String) -> None:
        """Um spike/cuneiforme (tipo em msg.data). Conta por tipo; o contador é
        zerado a cada linha do CSV (contagem POR amostra de força)."""
        t = str(msg.data)
        with self._lock:
            if t in self._evt_counts:
                self._evt_counts[t] += 1

    def _cb_joints(self, msg: JointState) -> None:
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in _ARM_JOINTS):
            return   # mensagem só com juntas da mão
        q = np.array([float(msg.position[idx[j]]) for j in _ARM_JOINTS])
        # FK aqui (~50 Hz), não no _cb_lc_sample: as colunas ficam prontas
        # e cada amostra de força só as copia.
        q_cols = [f'{v:.5f}' for v in q]
        try:
            tcp = forward_kinematics(q, T_end=T_TOUCH_TOOL_ATTACH)[:3, 3]
            tcp_cols = [f'{v:.5f}' for v in tcp]
        except Exception:
            tcp_cols = ['', '', '']
        now = time.time()
        with self._lock:
            self._q = q
            self._q_cols = q_cols
            self._tcp_cols = tcp_cols
            self._q_ts = now

    def _cb_lc_sample(self, msg: LoadCellSample) -> None:
        """Uma linha do CSV unificado por amostra da célula — sinal canônico.

        O arquivo sai na taxa da CÉLULA, que é o HX711: 10 Hz com o pino RATE
        em GND (como vêm as placas vermelhas de fábrica) ou 80 Hz com ele em
        DVDD. Não há caminho de 1 kHz aqui — o conversor não passa de 80 SPS.
        O toque, que chega a ~834 Hz, é portanto DECIMADO neste arquivo: cada
        linha leva o último frame recebido, e `taxel_age_ms` diz quão velho
        ele era. Para o toque em taxa cheia use os CSVs crus da GUI
        (`__adc.csv`), que gravam toda linha do firmware.

        `lc_t_us`/`touch_t_us` são os relógios dos DOIS firmwares, os únicos
        tempos desta linha que não passaram pela pilha ROS — é com eles que se
        reamostra as duas fontes num eixo comum depois da coleta. `t_unix`
        continua sendo a hora de CHEGADA, útil só para ordenar."""
        now = time.time()
        with self._lock:
            self._last_sample_t = now
            if self._csv_writer is None or self._run_t0 is None:
                return
            # Colunas de junta/TCP já calculadas no _cb_joints (FK fora daqui).
            q_cols = self._q_cols
            tcp_cols = self._tcp_cols
            # Tátil: último frame ADC + contagem de eventos DESDE a amostra
            # anterior (zerada após escrever). Fase como código numérico.
            adc_cols = self._adc_cols
            # Idade do frame tátil NESTA amostra de força. As duas fontes são
            # seriais independentes: a linha do CSV parea a força que acabou
            # de chegar com o ÚLTIMO frame recebido, e sem esta coluna não há
            # como auditar depois quão perto no tempo os dois realmente
            # estavam. Vazio = nenhum frame recebido ainda.
            adc_age = ('' if self._adc_ts <= 0.0
                       else f'{(now - self._adc_ts) * 1e3:.1f}')
            adc_t_us = '' if self._adc_ts <= 0.0 else str(self._adc_t_us)
            # Simétrico ao taxel_age_ms — vazio = nenhum /joint_states ainda.
            pose_age = ('' if self._q_ts <= 0.0
                        else f'{(now - self._q_ts) * 1e3:.1f}')
            wp_cols = self._wp_cols
            mode_vals = self._mode_vals
            mode = self._mode
            evt_cols = [self._evt_counts[t] for t in TOUCH_EVENT_TYPES]
            self._evt_counts = {t: 0 for t in TOUCH_EVENT_TYPES}
            phase_code = PHASE_CODES.get(self._phase, -1)
            setpoint = ('' if self._setpoint is None
                        else f'{self._setpoint:.4f}')
            try:
                self._csv_writer.writerow([
                    f'{now - self._run_t0:.4f}',
                    f'{now:.4f}',
                    mode,
                    self._cycle,
                    phase_code,
                    setpoint,
                    f'{float(msg.force_net_n):.4f}',
                    int(msg.seq),
                    int(msg.t_us),
                    f'{float(msg.voltage_raw):.7f}',
                    *q_cols,
                    *tcp_cols,
                    pose_age,
                    *adc_cols,
                    adc_t_us,
                    adc_age,
                    *evt_cols,
                    # MATRIX_MAP varia por amostra (wp_*); os demais modos
                    # carregam constantes do run.
                    *(wp_cols if mode == 'MATRIX_MAP' else mode_vals),
                ])
                self._sample_count += 1
                # Flush a cada ~1 s (1000 amostras @ 1 kHz) para não perder
                # dados se o nó for morto sem encerrar limpo, sem martelar o
                # disco a cada amostra.
                if self._sample_count % 1000 == 0 and self._csv_fh is not None:
                    self._csv_fh.flush()
            except (ValueError, OSError) as exc:
                self.get_logger().warn(f'Falha ao gravar amostra: {exc}')

    def _watchdog(self) -> None:
        with self._lock:
            if self._csv_fh is None or self._run_t0 is None:
                return
            if time.time() - self._last_sample_t > RUN_IDLE_TIMEOUT_S:
                self.get_logger().warn(
                    f'Run sem amostras de força há {RUN_IDLE_TIMEOUT_S:.0f}s '
                    '— encerrando por timeout.')
                self._close_run_locked('timeout')

    # Encerramento
    def _close_run_locked(self, reason: str) -> None:
        """Fecha o run atual. Deve ser chamado com `self._lock`."""
        if self._csv_fh is None:
            return
        try:
            self._csv_fh.flush()
            self._csv_fh.close()
        except OSError:
            pass
        # __matrix.csv (se houve MATRIX_MAP) fecha junto com o run.
        matrix_path = self._matrix_path
        n_points = self._matrix_rows
        if self._matrix_fh is not None:
            try:
                self._matrix_fh.flush()
                self._matrix_fh.close()
            except OSError:
                pass
        self._matrix_fh = None
        self._matrix_writer = None
        self._matrix_path = None
        self._matrix_rows = 0

        duration = (time.time() - self._run_t0
                     if self._run_t0 else 0.0)
        run_path = self._run_path
        n_samples = self._sample_count
        self.get_logger().info(
            f'Run encerrado ({reason}): {n_samples} amostras '
            f'em {duration:.1f}s → {_rel(run_path) if run_path else "?"}')
        if matrix_path:
            self.get_logger().info(
                f'Mapa da matriz: {n_points} identações → '
                f'{_rel(matrix_path)}')
        # Integridade do canal tátil: um run que perdeu frames continua válido
        # para a força, mas quem for analisar os taxels precisa saber disso
        # ANTES de olhar o CSV — silêncio aqui foi o que deixou a coleta de
        # 07/08/2026 passar com 19% dos frames corrompidos.
        if self._adc_bad:
            self.get_logger().warn(
                f'ATENÇÃO: {self._adc_bad} frames ADC corrompidos foram '
                f'descartados neste run — a serial do STM32 perdeu bytes. As '
                f'linhas afetadas carregam o último frame BOM (veja '
                f'taxel_age_ms).')
        self._csv_fh = None
        self._csv_writer = None
        self._run_path = None
        self._run_stamp = None
        self._run_dir = None
        self._run_t0 = None
        self._sample_count = 0
        self._wp_cols = []
        self._mode_cols = []
        self._mode_vals = []
        # Relatório pós-run (summary JSON + gráfico) em background — só
        # para runs concluídos com dados; 'superseded' é um run substituído.
        if run_path and n_samples > 0 and reason != 'superseded':
            threading.Thread(
                target=self._generate_report, args=(run_path,),
                daemon=True, name='palpation-report').start()

    def _generate_report(self, csv_path: str) -> None:
        try:
            from .palpation_report import generate_report
            summary = generate_report(csv_path)
            self.get_logger().info(
                'Relatório gerado: '
                f'{_rel(summary.get("summary_path", "?"))}')
        except Exception as exc:   # nunca derruba o logger
            self.get_logger().warn(f'Falha ao gerar relatório: {exc}')

    def close(self) -> None:
        with self._lock:
            self._close_run_locked('shutdown')


def main(args=None):
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s.%(msecs)03d [%(name)s] %(levelname)s  %(message)s',
        datefmt='%H:%M:%S')
    rclpy.init(args=args)
    node = PalpationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
