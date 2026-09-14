"""force_wave.py — a onda de força modulada: medida, ganho e correção por ciclo.

O modo FMOD pede ao braço uma força que OSCILA, e tudo aqui existe porque o
caminho da medida não é transparente: o One-Euro atrasa e atenua conforme a
frequência, a interpolação do controlador corta amplitude, e o resultado é uma
onda que nasce curta e fora de fase sem que nada no log diga isso.

  * `fmod_measure_lag_s` / `fmod_measure_gain` — o que o pipeline de medida faz
    com a onda ANTES de ela ser comparada ao setpoint. É o que permite corrigir
    em malha aberta, em vez de deixar a adaptação descobrir sozinha.
  * `_WaveILC` — aprende o erro de execução ciclo a ciclo, por fase.
  * `_ForceProfile` — a onda pedida (forma, amplitude, duração) e os limites que
    a tornam executável dentro do período do servoj.

Saiu do `tactile_explorer` porque não depende da FSM nem do nó: é matemática da
onda, roda sem ROS, e estava enterrada no meio de sete mil linhas de máquina de
estados.
"""
from __future__ import annotations

import math

import numpy as np

from .constants import LC_NOMINAL_RATE_HZ as _LC_NOMINAL_RATE_HZ
from .lc_filter import (
    MEDIAN_N as _MEDIAN_N,
    ONE_EURO_MAXCUTOFF_HZ as _ONE_EURO_MAXCUTOFF_HZ,
)
from .explorer_constants import (
    _CTRL_DT, _FMOD_DT_MIN_S, _FMOD_ILC_ALPHA, _FMOD_ILC_BINS,
    _FMOD_MIN_PTS_PER_CYCLE, _SERVOJ_T_MIN_S
)


def fmod_measure_lag_s(freq_hz: float,
                       rate_hz: float = _LC_NOMINAL_RATE_HZ) -> float:
    """Atraso do PIPELINE DE MEDIDA (s) na frequência da onda.

    O ILC compara a força medida com o setpoint que a causou, e os dois estão
    separados por este atraso. Ele é conhecido de antemão, não precisa ser
    estimado: sai do filtro que o `lc_filter` aplica e da taxa da célula.

    Medido no run TOUCH/20260828_154934 a 1 Hz, o atraso TOTAL foi 55,5°
    (154 ms). Esta função responde 41,3° dele — One-Euro (26,6°) mais mediana
    (14,8°). Os ~14° restantes são transporte do executor e o próprio
    material, que o ILC aprende como qualquer outro erro; o que ele NÃO pode
    aprender sozinho é um atraso grande o bastante para o erro entrar no bin
    errado, e é isso que descontar a parte conhecida evita.

    `rate_hz` É A TAXA DA FONTE QUE ESTÁ NO FIO, e ela não é uma constante
    deste arquivo: a mediana de _MEDIAN_N amostras atrasa meia janela, o que
    vale 41,7 ms na HX711 (24 Hz) e 2,5 ms na FA7155 (~400 Hz entregues em
    polled — ver FT_NOMINAL_RATE_HZ). Os 39 ms de diferença são 14° a 1 Hz,
    mais de um bin dos _FMOD_ILC_BINS: assumir a célula errada aqui gira a
    correção do ILC em fase, que é o erro que esta função existe para não
    cometer. Por isso o chamador passa a taxa MEDIDA (`_lc_rate_hz`) e o
    default é só semente para quem chamar sem célula no ar.

    O termo do filtro NÃO muda com a célula: o One-Euro trava em
    min(rate/3, ONE_EURO_MAXCUTOFF_HZ) e a 24 Hz ou a 400 Hz quem manda é o
    teto de 2 Hz nos dois casos (ver lc_filter).

    Função pura — testável sem ROS e sem bancada.
    """
    f = max(float(freq_hz), 1e-6)
    # Passa-baixa de 1ª ordem no cutoff em que o One-Euro está TRAVADO em
    # repouso e perto dele (o teto absoluto de 2 Hz manda; ver lc_filter).
    tau = 1.0 / (2.0 * math.pi * _ONE_EURO_MAXCUTOFF_HZ)
    lag_filtro = math.atan(2.0 * math.pi * f * tau) / (2.0 * math.pi * f)
    # Mediana de N: o valor devolvido é o do meio da janela.
    lag_mediana = 0.5 * (_MEDIAN_N - 1) / max(float(rate_hz), 1.0)
    return lag_filtro + lag_mediana


def fmod_measure_gain(freq_hz: float) -> float:
    """Fração da amplitude que SOBREVIVE ao pipeline de medida em `freq_hz`.

    Companheira de fmod_measure_lag_s: aquela dá a fase, esta dá o módulo. O
    One-Euro está travado em ONE_EURO_MAXCUTOFF_HZ (2 Hz) em repouso e perto
    dele — e continua travado lá com a FA7155, porque o outro teto do filtro
    é rate/3 (133 Hz a 400 Hz, 8 Hz a 24 Hz) e nunca é ele quem manda. Um
    passa-baixa de 1ª ordem nesse cutoff vale 1/√(1+(f/fc)²):

        0,5 Hz → 97 %      2 Hz → 71 %      6,67 Hz → 29 %
        1,0 Hz → 89 %      4 Hz → 45 %     10,0 Hz → 20 %

    POR QUE ISTO EXISTE. Qualquer laço que se adapte pela amplitude MEDIDA
    (o `fx_gain` por lock-in, e o ILC) fecha contra este ganho sem saber. A
    10 Hz ele lê 20 % da onda, conclui que a onda está curta e manda cinco
    vezes mais curso: simulado, o ILC sobre-excita para 149 % da amplitude
    pedida e o pico vai a 2,62 N numa onda pedida até 2,00 N. O erro não é
    de sintonia, é de premissa — a medida não existe naquela frequência.

    Função pura — testável sem ROS e sem bancada.
    """
    f = max(float(freq_hz), 0.0)
    return 1.0 / math.hypot(1.0, f / _ONE_EURO_MAXCUTOFF_HZ)


class _WaveILC:
    """Correção de penetração aprendida POR FASE, um ciclo por vez.

    `observe()` acumula, dentro do ciclo, o erro de força de cada bin;
    `commit()` fecha o ciclo e move a correção; `value()` devolve a correção
    daquela fase para o feedforward somar.
    """

    def __init__(self, n_bins: int = _FMOD_ILC_BINS,
                 alpha: float = _FMOD_ILC_ALPHA, clip_m: float = 1e-3):
        self.n = int(max(4, n_bins))
        self.alpha = float(alpha)
        self.clip_m = float(abs(clip_m))
        self.corr = np.zeros(self.n)
        self.cycles = 0
        self._acc = np.zeros(self.n)
        self._cnt = np.zeros(self.n)

    def observe(self, phase01: float, err_n: float, k_nm: float) -> None:
        """Erro de força `err_n` (alvo − medido) atribuído à fase que o
        causou. O chamador já descontou o atraso da medida."""
        i = int((phase01 % 1.0) * self.n) % self.n
        self._acc[i] += float(err_n) / max(float(k_nm), 1.0)
        self._cnt[i] += 1.0

    def discard(self) -> None:
        """Joga fora o ciclo observado SEM mover a correção.

        Para o ciclo em que o limitador de excursão cortou: ali o comando não
        foi o que o laço pediu, então o erro medido é em parte obra do corte.
        Aprender com ele ensina o vetor a empurrar mais contra o limitador,
        que corta mais — o windup clássico do par integrador+saturação.
        """
        self._acc[:] = 0.0
        self._cnt[:] = 0.0

    def commit(self) -> float:
        """Fecha o ciclo. Devolve a norma da correção aplicada (m)."""
        vis = self._cnt > 0
        upd = np.zeros(self.n)
        upd[vis] = self._acc[vis] / self._cnt[vis]
        # FILTRO Q (suavização circular). Sem ele o ILC realimenta o ruído da
        # célula nos harmônicos altos do vetor, onde a planta não responde, e
        # a correção diverge em poucos ciclos — é o modo de falha clássico do
        # controle repetitivo. Continua não sendo precaução teórica com a
        # FA7155, só menos violenta: o cru dela tem σ = 21,9 mN (contra os
        # 112 mN do cru da HX711) e cada bin agora promedia ~16 amostras em
        # vez de ~1 a 1 Hz, o que já divide o ruído por ~4. Sobra ~5 mN por
        # bin realimentados a cada ciclo, e é isso que o Q derruba.
        upd = (np.roll(upd, 1) + 2.0 * upd + np.roll(upd, -1)) / 4.0
        self.corr = np.clip(self.corr + self.alpha * upd,
                            -self.clip_m, self.clip_m)
        self._acc[:] = 0.0
        self._cnt[:] = 0.0
        self.cycles += 1
        return float(np.sqrt(np.mean(self.corr ** 2)))

    def value(self, phase01: float) -> float:
        """Correção (m) nesta fase, interpolada entre bins — o vetor é
        circular, então o último bin faz fronteira com o primeiro."""
        x = (phase01 % 1.0) * self.n
        i0 = int(x) % self.n
        i1 = (i0 + 1) % self.n
        f = x - math.floor(x)
        return float((1.0 - f) * self.corr[i0] + f * self.corr[i1])


def _fmod_max_freq_hz(servoj_period_s: float) -> float:
    """Frequência máxima RASTREÁVEL com um dado período de ServoJ.

    Quem governa o braço real é o laço ServoJ do mirror_node, que amostra o
    ÚLTIMO alvo publicado: publicar mais rápido que ele não acelera nada,
    apenas descarta pontos. O teto é o período dele vezes o mínimo de pontos
    por período — 6,7 Hz com os 30 ms padrão, 8,0 Hz com 25 ms, e 10,0 Hz
    com os 20 ms do piso do firmware.
    """
    return 1.0 / max(servoj_period_s * _FMOD_MIN_PTS_PER_CYCLE, 1e-9)


def _fmod_sampling_gain(pts_per_cycle: float) -> float:
    """Fração da amplitude que sobrevive à AMOSTRAGEM da onda.

    A onda é comandada em `pts_per_cycle` pontos por período e o controlador
    interpola entre eles. Interpolação linear = convolução com dois boxcars
    de um período de amostragem, então a fundamental sai atenuada por
    sinc²(1/N) — 87,5 % a 5 pontos, 95 % a 8. Confere com a integração
    numérica da onda reconstruída (ver a tabela em _FMOD_MIN_PTS_PER_CYCLE).

    Isto é ganho de MALHA ABERTA, conhecido antes de a onda abrir: dividir a
    amplitude comandada por ele entrega a amplitude pedida já no primeiro
    ciclo, em vez de deixar a adaptação por ciclo descobrir sozinha. Sem
    isto, uma onda a 10 Hz nasce 12,5 % curta por construção.
    """
    n = max(float(pts_per_cycle), 2.0)
    return float(np.sinc(1.0 / n) ** 2)   # np.sinc(x) = sin(pi x)/(pi x)


class _ForceProfile:
    """Setpoint de força variável no tempo: F(t) = média + amp·trig(2πf·t).

    Só descreve a onda; quem a executa é _phase_hold_modulated. Construir
    por from_params() (parâmetros ROS) — o construtor não valida nada.
    """

    def __init__(self, shape: str, f_min_n: float, f_max_n: float,
                 freq_hz: float, cycles: int):
        self.shape = shape
        self.f_min_n = float(min(f_min_n, f_max_n))
        self.f_max_n = float(max(f_min_n, f_max_n))
        self.freq_hz = float(freq_hz)
        self.cycles = int(cycles)

    @property
    def mean_n(self) -> float:
        return 0.5 * (self.f_min_n + self.f_max_n)

    @property
    def amp_n(self) -> float:
        return 0.5 * (self.f_max_n - self.f_min_n)

    @property
    def duration_s(self) -> float:
        return self.cycles / max(self.freq_hz, 1e-6)

    @property
    def pts_per_cycle(self) -> float:
        """Amostras por período NO TICK DO QS (_CTRL_DT). Mantido porque é o
        número que descreve o caminho quase-estático; a onda usa o seu próprio
        tick — ver pts_per_cycle_at() e wave_dt()."""
        return 1.0 / max(self.freq_hz * _CTRL_DT, 1e-9)

    def pts_per_cycle_at(self, dt: float) -> float:
        """Amostras por período com um tick de `dt` segundos."""
        return 1.0 / max(self.freq_hz * dt, 1e-9)

    def wave_dt(self, servoj_period_s: float = _CTRL_DT) -> float:
        """Tick que dá _FMOD_MIN_PTS_PER_CYCLE pontos por período nesta
        frequência, limitado pelo piso do laço, pelo tick do QS (não faz
        sentido ir mais DEVAGAR que ele) e pelo período do ServoJ.

        O piso por `servoj_period_s` é o que impede o descarte silencioso:
        publicar a 40 Hz para um mirror que amostra a 33 Hz não entrega uma
        onda de 5 Hz, entrega uma reamostrada — medido em 14/08/2026, o run
        20260814_115804 pediu 5 Hz com tick de 25 ms contra os 30 ms do
        mirror. Agora o tick nunca fica abaixo do período REAL do ServoJ, e
        quem quiser 5 Hz sobe o mirror com servoj_period_s:=0.025.
        """
        want = 1.0 / max(self.freq_hz * _FMOD_MIN_PTS_PER_CYCLE, 1e-9)
        # _SERVOJ_T_MIN_S entra no piso porque é o limite do FIRMWARE, não uma
        # escolha: o `t` do ServoJ tem faixa [0.02, 3600] s, e um tick menor
        # não vira comando nenhum — o controlador recusa o ponto. Ele DOMINA
        # _FMOD_DT_MIN_S (4 ms), que continua documentando onde o laço Python
        # deixaria de fechar o ciclo caso o hardware um dia permitisse.
        floor_s = max(_FMOD_DT_MIN_S, _SERVOJ_T_MIN_S, float(servoj_period_s))
        return float(min(max(want, floor_s), _CTRL_DT))

    def setpoint_n(self, t_s: float) -> float:
        """Força pedida em t segundos do início da modulação."""
        w = 2.0 * math.pi * self.freq_hz * t_s
        trig = math.cos(w) if self.shape == 'COSINE' else math.sin(w)
        return self.mean_n + self.amp_n * trig

    def describe(self) -> str:
        return (f'{self.shape} {self.f_min_n:.2f}–{self.f_max_n:.2f} N '
                f'@ {self.freq_hz:.2f} Hz × {self.cycles} ciclos '
                f'({self.duration_s:.1f} s)')
