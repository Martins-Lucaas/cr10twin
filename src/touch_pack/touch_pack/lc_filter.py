"""Filtro da célula de carga e QoS dos tópicos de sensor.

Morava em `force_receiver_node.py` (célula axial XIAO+HX711, removida em
20/08/2026). Ficou aqui porque o `ft_receiver` da FA7155 usa o MESMO filtro:
a cadeia a jusante (`/load_cell/force_net`) tem de ter a mesma dinâmica
qualquer que seja a célula, senão os ganhos do explorer mudam de sentido.
"""
import collections
import math

from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy)

from .constants import FT_NOMINAL_RATE_HZ

# Filtro pesado: mediana + One-Euro (Casiez et al. 2012), cutoff ADAPTATIVO:
# parado cai a ONE_EURO_MINCUTOFF (zero firme); em movimento sobe e a
# latência despenca. O dt REAL vem do t_us do firmware, então a taxa do
# HX711 (10 vs 80 Hz) não precisa ser conhecida.
#
# beta é expresso em Hz por (N/s), não V/s: o sinal desta célula é minúsculo
# em volts, então a derivada é convertida para N/s pela sensibilidade da
# célula (slope da calibração quando existir, nominal antes disso) — senão o
# termo adaptativo degenera num passa-baixa fixo.
#
# ATENÇÃO: a sintonia abaixo foi medida no HX711 a 82 Hz. A FA7155 entrega
# 250 Hz com ruído diferente — RE-MEDIR antes de confiar nos números. O
# cutoff é adaptativo e o dt vem do carimbo do sensor, então nada quebra com
# a taxa nova; o que muda é se 0,3/0,5 continua sendo o ótimo.
#
# Sintonia de 05/08/2026, medida sobre 60 s do stream REAL a 82 Hz (ruído
# cru σ=19,3 mN, branco só em parte: a média de 8 amostras rende 1,43× mais
# que o ideal √n, e há um piso de ~5 mN que nenhum filtro razoável passa —
# não adianta pedir mais suavização, tem que atacar a fonte).
# mincutoff 1,2→0,3 Hz e beta 0,25→0,5 Hz/(N/s) melhoram TUDO ao mesmo tempo,
# porque num transitório quem manda é o beta e não o mincutoff — 1,2 Hz
# pagava ruído em repouso sem comprar velocidade:
#                       σ repouso   t90 (degrau 1 N)   assentamento 0,11 N
#   1,2 / 0,25 (antigo)   6,85 mN         61 ms              115 ms
#   0,3 / 0,50 (atual)    5,97 mN         49 ms              106 ms
MEDIAN_N = 3                  # rejeita glitch isolado de 1 amostra
                              # (5 custa 24 ms de lag e rende só 0,15 mN)
ONE_EURO_FREQ      = FT_NOMINAL_RATE_HZ   # chute até o 1º dt medido
ONE_EURO_MINCUTOFF = 0.3      # Hz — repouso (↓ = zero mais firme, +lag parado)
ONE_EURO_BETA_N    = 0.5      # Hz por (N/s) — responsividade ao contato
ONE_EURO_DCUTOFF   = 5.0      # Hz — cutoff do estimador de derivada
# Cutoff nunca passa de freq/3, senão o passa-baixa de 1ª ordem deixa de
# filtrar e só propaga ruído de banda alta.
ONE_EURO_MAXCUTOFF_FRAC = 1.0 / 3.0
# Escala do termo adaptativo: quantas unidades do sinal de entrada valem 1 N.
# A FA7155 entrega NEWTONS, então é 1 N/N — o ft_receiver confirma chamando
# set_sensitivity(1.0) logo após construir o filtro. O default anterior era
# LC_NOMINAL_V_PER_N, derivado da ponte do HX711 (mV/V × AVDD × ganho) da
# célula axial removida em 20/08/2026; aquelas constantes saíram de
# constants.py e o nome ficou pendurado aqui, quebrando o nó na partida.
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
        cutoff = min(cutoff, max(freq * ONE_EURO_MAXCUTOFF_FRAC,
                                 self._mincutoff))
        a = self._alpha(cutoff, freq)
        x_hat = a * v_med + (1.0 - a) * self._x_prev
        self._x_prev = x_hat
        self._dx_prev = dx_hat
        return x_hat

QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
