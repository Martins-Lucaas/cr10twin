"""
lc_health_probe.py — Mede a saúde do link da célula axial (XIAO + HX711).

Para que serve: DECIDIR, com número, se uma mudança no firmware melhorou ou
piorou o stream. A tabela de guardas do main.cpp foi levantada assim, e a
lição que ela deixou é a razão de este nó existir com um default de 10
minutos: `60 s não provam estabilidade`. A guarda de 20 ms passou num teste
de 60 s (2526/2526 amostras boas) e dessincronizou depois de alguns minutos de
bancada, porque o que a quebrou foi TEMPERATURA — e temperatura precisa de
tempo para aparecer.

Mede pelos tópicos, não pela tty: o `force_receiver` é dono exclusivo da porta
(uma tty admite um leitor só), então abrir a serial aqui exigiria derrubá-lo —
e aí não se estaria medindo o caminho que a palpação usa. As duas fontes:

  /load_cell/sample     LoadCellSample — seq, t_us, voltage_raw (SEM o
                        One-Euro: é o ruído da FONTE que interessa aqui, não o
                        do filtro), voltage (filtrado, para comparação)
  /load_cell/fw_health  String chave=valor — o heartbeat do firmware
                        (resets, timeouts, conv_us, zeroed) republicado pelo
                        receiver, mais os contadores do host

O que sai, e por que cada número:

  taxa_hz         entrega efetiva. Contra LC_NOMINAL_RATE_HZ diz se a placa
                  está entregando o que o conversor produz.
  sigma_v/_n      ruído em repouso do sinal CRU. É o número que a sincronia
                  por borda ataca: descartar conversões custava ruído.
  dt_*            estatística do intervalo entre CONVERSÕES, tirada do t_us
                  (que carimba a borda de descida do DOUT). É a prova direta
                  de que a sincronia está sã: um dt limpo cola no período do
                  HX711; um dt com cauda longa é leitura caindo dentro da
                  conversão.
  perdidas        lacunas no seq — amostra transmitida que não chegou.
  resets          power-cycles do HX711 no período. Alguns por hora é o chip
                  travando e se recuperando; alguns por MINUTO é sincronia
                  perdida, e é o sintoma que a mudança de 28/08/2026 ataca.

REPOUSO. `sigma_*` só significa alguma coisa com a célula PARADA E
DESCARREGADA. O nó não tem como saber se ela está — quem garante isso é quem
roda. Com o braço se movendo, `sigma` mede o movimento, não o ruído.

Uso:
  # linha de base ANTES de gravar o firmware novo, e a mesma medida depois
  ros2 run touch_pack lc_health_probe --ros-args -p duration_s:=600.0
  ros2 run touch_pack lc_health_probe --ros-args -p duration_s:=600.0 \
      -p rotulo:=borda_dout

  # duration_s:=0 → captura até Ctrl-C.

Saída (em sensors/Data/lc_health/):
  lc_health_<rotulo>_<ts>_samples.csv   uma linha por amostra
  lc_health_<rotulo>_<ts>.json          resultado + metadados
"""
from __future__ import annotations

import json
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from touch_pack_msgs.msg import LoadCellSample

from .constants import (
    LC_CALIB_FILE, LC_NOMINAL_RATE_HZ, RUNS_DIR, lc_calib_fingerprint,
    lc_load_calibration,
)
from .lc_filter import QOS_SENSOR

# Acima disto o intervalo não é o período do conversor: é amostra perdida, ou
# o firmware recolhendo o zero. Entra na contagem de outliers em vez de
# deformar a média e o desvio do dt.
_DT_OUTLIER_FACTOR = 3.0


def _stats(xs: list[float]) -> dict:
    """Média, desvio, extremos e p99 — o suficiente para comparar duas
    capturas sem trazer numpy para um nó que só soma números."""
    if not xs:
        return {}
    n = len(xs)
    media = math.fsum(xs) / n
    var = math.fsum((x - media) ** 2 for x in xs) / n
    ordenados = sorted(xs)
    return {
        'n': n,
        'media': media,
        'sigma': math.sqrt(var),
        'min': ordenados[0],
        'max': ordenados[-1],
        'p50': ordenados[n // 2],
        'p99': ordenados[min(n - 1, int(0.99 * n))],
    }


class LcHealthProbe(Node):

    def __init__(self):
        super().__init__('lc_health_probe')
        self._dur = float(self.declare_parameter('duration_s', 600.0).value)
        self._rotulo = str(self.declare_parameter('rotulo', '').value).strip()

        self._t0 = time.monotonic()
        self._seq_prev: int | None = None
        self._t_us_prev: int | None = None
        self._perdidas = 0
        self._resyncs = 0
        self._linhas: list[tuple] = []
        self._dts_us: list[float] = []
        self._v_raw: list[float] = []
        self._fw_primeiro: dict[str, float] = {}
        self._fw_ultimo: dict[str, float] = {}

        cal = lc_load_calibration(LC_CALIB_FILE)
        self._slope = abs(cal[0]) if cal else 0.0

        self.create_subscription(LoadCellSample, '/load_cell/sample',
                                 self._on_sample, QOS_SENSOR)
        self.create_subscription(String, '/load_cell/fw_health',
                                 self._on_fw_health, 10)
        self.create_timer(1.0, self._tick)

        alvo = (f'{self._dur:.0f} s' if self._dur > 0 else 'até Ctrl-C')
        self.get_logger().info(
            f'lc_health_probe: capturando {alvo}. MANTENHA A CÉLULA PARADA E '
            'DESCARREGADA — com o braço se movendo, o sigma mede o movimento '
            'e não o ruído.')

    def _on_fw_health(self, msg: String) -> None:
        campos = {}
        for token in msg.data.split():
            chave, sep, valor = token.partition('=')
            if not sep:
                continue
            try:
                campos[chave] = float(valor)
            except ValueError:
                continue
        if not campos:
            return
        if not self._fw_primeiro:
            self._fw_primeiro = campos
        self._fw_ultimo = campos

    def _on_sample(self, msg: LoadCellSample) -> None:
        seq, t_us = int(msg.seq), int(msg.t_us)
        dt_us = None
        if self._seq_prev is not None:
            d = (seq - self._seq_prev) & 0xFFFFFFFF
            if d == 1 and self._t_us_prev is not None:
                # Só um salto de EXATAMENTE 1 mede um período de conversão. Com
                # amostra perdida no meio, o dt cobriria duas conversões e
                # entraria na estatística como se fosse jitter.
                dt_us = float((t_us - self._t_us_prev) & 0xFFFFFFFF)
            elif 1 < d <= 1000:
                self._perdidas += d - 1
            elif d != 1:
                self._resyncs += 1
        self._seq_prev, self._t_us_prev = seq, t_us

        v_raw = float(msg.voltage_raw)
        self._v_raw.append(v_raw)
        if dt_us is not None:
            self._dts_us.append(dt_us)
        self._linhas.append((time.monotonic() - self._t0, seq, t_us,
                             dt_us if dt_us is not None else '',
                             v_raw, float(msg.voltage)))

    def _tick(self) -> None:
        se = time.monotonic() - self._t0
        if self._dur > 0 and se >= self._dur:
            self._finalizar()
            raise SystemExit(0)
        if int(se) % 60 == 0 and se >= 60:
            self.get_logger().info(
                f'{se / 60:.0f} min: {len(self._v_raw)} amostras, '
                f'{self._perdidas} perdidas, '
                f'{int(self._diff_fw("resets"))} resets do HX711.')

    def _diff_fw(self, chave: str) -> float:
        """Delta de um contador do firmware DENTRO da captura. Subtrair as
        pontas é o que permite entrar no meio de uma sessão já em curso — os
        contadores da placa são acumulados desde o boot dela, não desde aqui.
        """
        if not self._fw_ultimo:
            return 0.0
        return (self._fw_ultimo.get(chave, 0.0)
                - self._fw_primeiro.get(chave, 0.0))

    def _resultado(self) -> dict:
        se = time.monotonic() - self._t0
        n = len(self._v_raw)
        # Outliers do dt fora da estatística: um intervalo de várias vezes o
        # período não é jitter da conversão, é buraco no stream.
        dt = _stats(self._dts_us)
        limpos = self._dts_us
        if dt:
            corte = _DT_OUTLIER_FACTOR * dt['p50']
            limpos = [x for x in self._dts_us if x <= corte]
        v = _stats(self._v_raw)
        sigma_v = v.get('sigma', 0.0)
        return {
            'duracao_s': se,
            'amostras': n,
            'taxa_hz': (n / se) if se > 0 else 0.0,
            'taxa_nominal_hz': LC_NOMINAL_RATE_HZ,
            'perdidas': self._perdidas,
            'perdidas_frac': (self._perdidas / max(n + self._perdidas, 1)),
            'resyncs': self._resyncs,
            'sigma_v': sigma_v,
            # Sem calibração não há newton, e inventar um seria pior que
            # devolver None: o número seguiria para uma comparação.
            'sigma_n': (sigma_v / self._slope) if self._slope else None,
            'tres_sigma_n': (3.0 * sigma_v / self._slope
                             if self._slope else None),
            'dt_us': dt,
            'dt_us_sem_outliers': _stats(limpos),
            'dt_outliers': len(self._dts_us) - len(limpos),
            'fw_resets': self._diff_fw('resets'),
            'fw_timeouts': self._diff_fw('timeouts'),
            'fw_saturated': self._diff_fw('saturated'),
            'fw_bad_lines': self._diff_fw('bad_lines'),
            'fw_conv_us': self._fw_ultimo.get('conv_us'),
            'fw_zeroed': self._fw_ultimo.get('zeroed'),
            'fw_zero_mv': self._fw_ultimo.get('zero_mv'),
            'calib_fingerprint': lc_calib_fingerprint(),
            'calib_slope_v_por_n': self._slope or None,
        }

    def _finalizar(self) -> None:
        r = self._resultado()
        d = os.path.join(RUNS_DIR, 'lc_health')
        os.makedirs(d, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        base = f'lc_health_{self._rotulo + "_" if self._rotulo else ""}{ts}'
        with open(os.path.join(d, base + '_samples.csv'), 'w',
                  encoding='utf-8') as f:
            f.write('t_s,seq,t_us,dt_us,voltage_raw_v,voltage_v\n')
            for linha in self._linhas:
                f.write(','.join(str(c) for c in linha) + '\n')
        with open(os.path.join(d, base + '.json'), 'w', encoding='utf-8') as f:
            json.dump(r, f, indent=2)

        dt = r['dt_us_sem_outliers'] or {}
        log = self.get_logger()
        log.info('─' * 62)
        log.info(f'lc_health: {r["amostras"]} amostras em '
                 f'{r["duracao_s"]:.0f} s')
        log.info(f'  taxa           {r["taxa_hz"]:.2f} Hz '
                 f'(nominal {r["taxa_nominal_hz"]:.0f})')
        if r['sigma_n'] is not None:
            log.info(f'  ruído CRU      σ = {r["sigma_n"] * 1e3:.1f} mN '
                     f'(3σ = {r["tres_sigma_n"] * 1e3:.1f} mN)')
        else:
            log.info(f'  ruído CRU      σ = {r["sigma_v"] * 1e6:.2f} µV '
                     '(sem calibração: não dá para dar em newton)')
        if dt:
            log.info(f'  dt conversão   {dt["media"]:.0f} ± '
                     f'{dt["sigma"]:.0f} µs '
                     f'(p99 {dt["p99"]:.0f}, máx {dt["max"]:.0f})')
        log.info(f'  perdidas       {r["perdidas"]} '
                 f'({r["perdidas_frac"] * 100:.3f} %), '
                 f'{r["dt_outliers"]} buracos no dt')
        log.info(f'  HX711          {r["fw_resets"]:.0f} power-cycles, '
                 f'{r["fw_timeouts"]:.0f} timeouts de borda')
        log.info(f'  arquivos       {os.path.join(d, base)}.json / _samples.csv')
        log.info('─' * 62)
        log.info('COMPARE com a captura da outra variante de firmware. Uma '
                 'captura sozinha não diz se melhorou.')


def main(args=None):
    rclpy.init(args=args)
    node = LcHealthProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node._linhas and node._dur <= 0:
            node._finalizar()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
