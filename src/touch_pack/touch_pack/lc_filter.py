"""Filtro da célula de carga e QoS dos tópicos de sensor.

Usado pelos DOIS receivers — `force_receiver` (célula axial XIAO ESP32C6 +
HX711, em uso na bancada) e `ft_receiver` (FA7155 de 6 eixos) — porque a
cadeia a jusante (`/load_cell/force_net`) tem de ter a mesma dinâmica
qualquer que seja a célula, senão os ganhos do explorer mudam de sentido.
"""
import collections
import math

from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy)

from .constants import LC_NOMINAL_RATE_HZ

# Filtro pesado: mediana + One-Euro (Casiez et al. 2012), cutoff ADAPTATIVO:
# parado cai a ONE_EURO_MINCUTOFF (zero firme); em movimento sobe e a
# latência despenca. O dt REAL vem do t_us do firmware, então a taxa do
# HX711 não precisa ser conhecida.
#
# beta é expresso em Hz por (N/s), não V/s: o sinal desta célula é minúsculo
# em volts, então a derivada é convertida para N/s pela sensibilidade da
# célula (slope da calibração quando existir, nominal antes disso) — senão o
# termo adaptativo degenera num passa-baixa fixo.
#
# RE-SINTONIA DE 28/08/2026, medida sobre 63 s do stream REAL a 47,6 Hz com
# um peso padrão de 0,5 kg (4,903 N) na célula. A sintonia anterior
# (0,3 / beta 0,5 / dcutoff 5 / teto freq÷3) foi levantada quando o ruído cru
# era 19,3 mN; o ruído medido AGORA é 114 mN — 5,9× pior — e nesse regime ela
# falhava de um jeito específico: o termo adaptativo lia o próprio ruído como
# movimento, o cutoff saturava no teto (15,9 Hz a 47,6 Hz) e o filtro virava
# quase transparente. Resultado medido: 3σ = 147 mN contra um CONTACT_ON_N de
# 100 mN, e o sinal cruzava o limiar sozinho em 100 % das janelas de repouso
# — foi o que abortou os runs com "8 falsos gatilhos".
#
# As três mudanças atacam isso: beta 0,5→0,1 (o ruído deixa de abrir o
# cutoff), dcutoff 5→1 Hz (a derivada é estimada numa banda onde há sinal e
# não ruído) e um TETO ABSOLUTO de 2 Hz no lugar de freq÷3.
#
#   config                        3σ       falsos+   t_det 0,5 N
#   0,3/0,5/dc5/teto freq÷3     147 mN      100 %        21 ms
#   0,3/0,1/dc1/teto 2 Hz  ←     86 mN        0 %        63 ms
#   0,25/0,05/dc1/teto 2 Hz      76 mN        0 %        84 ms
#
# RE-VALIDADA a 24,5 Hz (guarda de 40 ms) sobre 100 s de stream real: 3σ =
# 91 mN, 0 % de falsos positivos, 0,5 N detectado em 82 ms. Praticamente
# igual ao medido a 47,6 Hz — o filtro é limitado por FREQUÊNCIA (teto de
# 2 Hz, mincutoff 0,3 Hz) e não por número de amostras, então mudar a taxa
# de entrega quase não mexe no σ. Não precisa re-sintonizar ao mudar a
# guarda do firmware.
#
# ESTA CONCLUSÃO É A QUE MANDA, e ela contraria a conta de livro: para ruído
# BRANCO, σ cairia com √(taxa), e de 24,5 para 47,6 Hz seriam 1,39× — que não
# apareceram. Ruído que não melhora com mais amostras é ruído 1/f, e ele
# atravessa um passa-baixa de 2 Hz qualquer que seja a taxa de entrada.
# Em 28/08/2026 o firmware passou de 24 para ~82 Hz (sincronia por borda de
# DOUT, ver o main.cpp). 82 Hz está FORA da faixa em que a conclusão acima foi
# medida (24,5–47,6 Hz), então ela é a hipótese de trabalho e não um fato
# medido ali: se o σ a 82 Hz cair perto de 1,7×, o ruído era mais branco do
# que estas duas capturas sugeriram. Medir com o lc_health_probe antes de
# mexer em qualquer um dos números acima.
#
# ATENÇÃO: medido no HX711. A FA7155 entrega 1 kHz com ruído diferente —
# RE-MEDIR antes de confiar nestes números para ela.
MEDIAN_N = 3                  # rejeita glitch isolado de 1 amostra
                              # (5 custa 24 ms de lag e rende só 0,15 mN)
ONE_EURO_FREQ      = LC_NOMINAL_RATE_HZ   # chute até o 1º dt medido
ONE_EURO_MINCUTOFF = 0.3      # Hz — repouso (↓ = zero mais firme, +lag parado)
ONE_EURO_BETA_N    = 0.1      # Hz por (N/s) — responsividade ao contato
ONE_EURO_DCUTOFF   = 1.0      # Hz — cutoff do estimador de derivada
# Teto do cutoff. O freq/3 sozinho não protege: a 47,6 Hz ele vale 15,9 Hz, e
# com o ruído atual o termo adaptativo encostava lá e desligava o filtro. O
# teto ABSOLUTO de 2 Hz é quem segura — o freq/3 continua como guarda para
# taxas baixas, onde 2 Hz seria alto demais para um passa-baixa de 1ª ordem.
ONE_EURO_MAXCUTOFF_FRAC = 1.0 / 3.0
ONE_EURO_MAXCUTOFF_HZ   = 2.0
# ── CANAL LENTO (medida quase-estática) ───────────────────────────────
# MEDIDO em 01/09/2026 sobre 32,6 s de ar livre do run MANUAL/20260901_162614
# (797 amostras a 24,39 Hz), reprocessando `lc_voltage_raw_v` do CSV — a
# reprodução offline do filtro rápido bateu com o log em σ (33,0 vs 33,2 mN),
# então a bancada abaixo vale sobre o sinal real e não sobre um modelo.
#
#   σ do sinal CRU      = 115 mN  (densidade 23,6 mN/√Hz na banda 0–12,2 Hz)
#   1 LSB do HX711      = 0,226 mN
#   ruído do conversor  ≈ 0,057 mN  →  o ruído medido é ~2000× o piso do ADC.
#
# Ou seja: NÃO é o conversor, e nenhum filtro conserta a fonte. O σ cru é o
# mesmo (110–135 mN) com o braço parado, em movimento, e sob 0,5 a 3,5 N de
# carga — piso aditivo, não vibração nem acoplamento mecânico.
#
# O ruído é BRANCO na banda, e isto CORRIGE a nota de sintonia acima ("ruído
# que não melhora com mais amostras é ruído 1/f"). Média móvel sobre o cru:
#
#   janela   0,12 s   0,49 s   2,01 s   4,02 s   8,00 s
#   σ        69,8     38,8     19,0     12,1      9,4  mN
#   1/√N     69,8     34,9     17,3     12,2      8,7  mN
#
# O escalonamento 1/√N vale até 8 s, e a inclinação log-log da PSD entre 0,05
# e 10 Hz é −0,04 (branco = 0, 1/f = −1). A observação que gerou a conclusão
# antiga (σ igual a 24,5 e 47,6 Hz) continua correta, mas a causa é outra: o
# ruído do HX711 por amostra cresce com √taxa, então a DENSIDADE é constante.
# A diferença importa — 1/f diz que alargar a média é inútil, e os dados
# dizem que ela paga integralmente. Subir a taxa do firmware não ajuda;
# alargar a janela ajuda na raiz quadrada.
#
# Daí os DOIS canais. Um filtro só atendia dois trabalhos incompatíveis:
#   RÁPIDO (/load_cell/force_net) — segurança, detecção de contato e a
#     textura do SLIDING, onde a variação de força É o sinal medido. Precisa
#     de latência baixa e 3σ abaixo de CONTACT_ON_N. Fica como está.
#   LENTO (/load_cell/force_net_slow) — a medida assentada do regulador
#     quase-estático, que já congela o braço para medir. Ali a latência é de
#     graça e o que falta é σ.
#
# Média móvel e não IIR: para ruído branco com janela de observação fixa a
# média é o estimador ótimo, e ela DOMINA o passa-baixa de 1ª ordem nos dois
# eixos ao mesmo tempo (σ 28,6 mN contra 33,0 no MESMO t95 de 0,9 s).
#
#   janela   σ      3σ      t95
#   1,0 s    28,6   85,7    0,90 s
#   2,0 s    19,0   56,9    1,89 s     ← default
#   4,0 s    12,1   36,3    3,81 s
#
# 2 s é o default porque o assentamento MECÂNICO do contato é da ordem de
# 4,5 s (mediana medida nos 6 runs de escada de 01/09): abaixo disso a
# janela não custa tempo nenhum de ensaio. Ajustável em `lc_slow_win_s`.
LC_SLOW_WIN_S = 2.0


class _BoxcarFilter:
    """Média móvel de janela fixa em SEGUNDOS (não em amostras).

    Em segundos pelo mesmo motivo do `_TARE_WIN_S` do receiver: o mesmo
    número tem de valer com o pino RATE do HX711 em GND (10 Hz) e em DVDD
    (80 Hz), e o σ de saída depende da JANELA, não da contagem.

    Enquanto a janela não encheu devolve a média do que há — o mesmo
    aquecimento da bancada offline. Quem precisa saber se ela já está cheia
    pergunta a `span_s`.
    """

    def __init__(self, win_s: float = LC_SLOW_WIN_S):
        self._win_s = max(1e-3, float(win_s))
        self._buf: collections.deque = collections.deque()   # (t_cum, v)
        self._sum = 0.0
        self._t = 0.0

    @property
    def win_s(self) -> float:
        return self._win_s

    @property
    def span_s(self) -> float:
        """Extensão temporal do que está na janela agora (s)."""
        if len(self._buf) < 2:
            return 0.0
        return self._buf[-1][0] - self._buf[0][0]

    def update(self, v: float, dt: float | None = None) -> float:
        """``dt`` = intervalo real desde a amostra anterior (s); None usa a
        taxa nominal."""
        self._t += dt if dt else (1.0 / LC_NOMINAL_RATE_HZ)
        self._buf.append((self._t, v))
        self._sum += v
        # Descarta pela ESQUERDA mantendo ao menos 1 amostra: a janela cobre
        # `win_s` de sinal, e nunca esvazia num dt maior que ela.
        while len(self._buf) > 1 and self._t - self._buf[0][0] > self._win_s:
            self._sum -= self._buf.popleft()[1]
        return self._sum / len(self._buf)


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
        cutoff = min(cutoff, ONE_EURO_MAXCUTOFF_HZ,
                     max(freq * ONE_EURO_MAXCUTOFF_FRAC, self._mincutoff))
        a = self._alpha(cutoff, freq)
        x_hat = a * v_med + (1.0 - a) * self._x_prev
        self._x_prev = x_hat
        self._dx_prev = dx_hat
        return x_hat

# Anúncio da janela do canal lento: quem consome precisa saber quanto tempo
# congelar antes de a média estar limpa de amostras pré-movimento, e um
# parâmetro duplicado nos dois nós silenciaria a divergência. TRANSIENT_LOCAL
# porque o explorer sobe depois do receiver.
QOS_LATCHED = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)

QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
