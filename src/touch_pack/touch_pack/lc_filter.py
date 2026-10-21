"""Filtro da célula de carga e QoS dos tópicos de sensor.

Usado pelo `ft_receiver` (FA7155 de 6 eixos) para o eixo de controle que sai
em `/load_cell/force_net`; os ganhos do explorer foram ajustados contra esta
dinâmica.
"""
import math

from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy)

from .constants import FT_NOMINAL_RATE_HZ

MEDIAN_N = 3                  # rejeita glitch isolado de 1 amostra
                              # (5 custa 24 ms de lag e rende só 0,15 mN)
ONE_EURO_FREQ      = FT_NOMINAL_RATE_HZ   # chute até o 1º dt medido
ONE_EURO_MINCUTOFF = 0.3      # Hz — repouso (↓ = zero mais firme, +lag parado)
ONE_EURO_BETA_N    = 0.1      # Hz por (N/s) — responsividade ao contato
ONE_EURO_DCUTOFF   = 1.0      # Hz — cutoff do estimador de derivada

ONE_EURO_MAXCUTOFF_FRAC = 1.0 / 3.0
ONE_EURO_MAXCUTOFF_HZ   = 2.0


NOMINAL_V_PER_N = 1.0


class _LoadCellFilter:
    """Mediana de MEDIAN_N seguida do One-Euro (passa-baixa adaptativo)."""

    def __init__(self, freq: float = ONE_EURO_FREQ,
                 mincutoff: float = ONE_EURO_MINCUTOFF,
                 beta_n: float = ONE_EURO_BETA_N,
                 dcutoff: float = ONE_EURO_DCUTOFF,
                 median_n: int = MEDIAN_N):
        self._freq = freq
        self._mincutoff = mincutoff
        self._beta_n = beta_n
        self._dcutoff = dcutoff
        self._median_n = median_n
        # V por N usados para converter dV/dt → dF/dt (beta em Hz/(N/s)).
        self._v_per_n = NOMINAL_V_PER_N
        self._median_buf: list[float] = []
        self._mi = 0
        self._x_prev = 0.0
        self._dx_prev = 0.0
        self._seeded = False

    def set_sensitivity(self, v_per_n: float) -> None:
        """Ajusta a escala V/N do termo adaptativo ao slope real da
        calibração. Slope absurdo (ou ausente) mantém o nominal.

        O teto INCLUI 1.0 porque o ft_receiver reusa este filtro sobre um
        sinal que já vem em newtons (FA7155), e ali a escala é 1 N/N."""
        s = abs(float(v_per_n))
        if 1e-6 < s <= 1.0:
            self._v_per_n = s

    @staticmethod
    def _alpha(cutoff: float, freq: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau * freq)

    def update(self, v: float, dt: float | None = None) -> float:
        """``dt`` = intervalo real desde a amostra anterior (s); None usa a
        taxa nominal."""
        freq = (1.0 / dt) if dt else self._freq
        if not self._seeded:
            self._median_buf = [v] * self._median_n
            self._x_prev = v
            self._dx_prev = 0.0
            self._seeded = True
            return v
        self._median_buf[self._mi] = v
        self._mi = (self._mi + 1) % self._median_n
        v_med = sorted(self._median_buf)[self._median_n // 2]
        dx = (v_med - self._x_prev) * freq
        a_d = self._alpha(self._dcutoff, freq)
        dx_hat = a_d * dx + (1.0 - a_d) * self._dx_prev
        # |dx_hat| em V/s → N/s antes de entrar no beta.
        cutoff = self._mincutoff + self._beta_n * abs(dx_hat) / self._v_per_n
        cutoff = min(cutoff, ONE_EURO_MAXCUTOFF_HZ,
                     max(freq * ONE_EURO_MAXCUTOFF_FRAC, self._mincutoff))
        a = self._alpha(cutoff, freq)
        x_hat = a * v_med + (1.0 - a) * self._x_prev
        self._x_prev = x_hat
        self._dx_prev = dx_hat
        return x_hat
    
QOS_LATCHED = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)

QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def window_drift(win: list[float]) -> float:
    """Deriva da janela de tare: |mediana da 2ª metade − mediana da 1ª|.

    Critério de estabilidade dos DOIS receivers — é ele que separa um zero
    honesto de um zero tirado com a ponteira já encostada. Mediana e não
    pico-a-pico porque o ptp cresce com o tamanho da janela mesmo em sinal
    estacionário, e recusaria tares bons por um único glitch.

    Mora aqui, e não em cada receiver, porque estava copiado byte a byte nos
    dois: afrouxar o critério num sensor e não no outro daria dois padrões de
    "zero aceitável" na mesma bancada, sem nada no log dizendo isso.
    """
    half = len(win) // 2
    m1 = sorted(win[:half])[half // 2]
    tail = win[half:]
    m2 = sorted(tail)[len(tail) // 2]
    return abs(m2 - m1)
