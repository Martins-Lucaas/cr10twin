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
    _FMOD_MIN_PTS_PER_CYCLE, _SERVOJ_T_MIN_S,
    _FMOD_DEADBAND_MIN_RATIO, _FMOD_ILC_MIN_MEAS_GAIN,
    _FMOD_MIN_MEAS_RATE_MULT, _FMOD_V_PEAK_MAX_MMS, _FMOD_V_PEAK_WARN_MMS,
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


def fmod_preflight(prof: '_ForceProfile', *, servoj_period_s: float,
                   amp_m: float, k_nm: float, use_curve: bool,
                   meas_rate_hz: float, has_raw: bool,
                   deadband_tcp_m: float) -> tuple[float, list[tuple[str, str]]]:
    """Tudo o que dá para saber sobre a onda ANTES de o braço se mexer.

    Devolve `(amp_pre, achados)`, onde `amp_pre` é o fator que devolve à
    amplitude comandada o que a interpolação come, e `achados` é uma lista de
    `(nível, texto)` com nível em 'error' | 'warn' | 'info'. Um único 'error'
    cancela o ensaio — quem loga e decide é o chamador.

    POR QUE TUDO JUNTO, E AQUI. Estas checagens são a mesma pergunta feita de
    seis ângulos: "a onda pedida existe nesta bancada?". Todas são função de
    números que já estão na mão (frequência, amplitude em posição, período do
    ServoJ, taxa da célula), nenhuma precisa de ROS, e juntas elas respondem
    de uma vez em vez de o operador descobrir uma por run. Estavam embutidas
    na FSM, onde não davam para testar sem subir um nó.

    A ORDEM É A DA CAUSA, não a da gravidade: frequência (o que o laço
    consegue comandar), velocidade (o que o material custa), medida (o que a
    célula consegue ver), correção (o que o ILC consegue corrigir) e
    quantização (o que a banda morta deixa passar). Quem lê o log de cima para
    baixo lê a cadeia na ordem em que ela quebra.

    Função pura — testável sem ROS e sem bancada.
    """
    out: list[tuple[str, str]] = []
    wave_dt = prof.wave_dt(servoj_period_s)
    pts = prof.pts_per_cycle_at(wave_dt)
    f_max_hz = _fmod_max_freq_hz(servoj_period_s)

    # ── 1. FREQUÊNCIA rastreável pelo laço ServoJ ────────────────────
    # Antes isto era um aviso e a onda rodava assim mesmo, reamostrada pelo
    # mirror: o CSV saía com uma frequência que não era a pedida nem a
    # entregue. Um ensaio que não pode ser rastreado é melhor recusado do que
    # gravado errado.
    if prof.freq_hz > f_max_hz * (1.0 + 1e-6):
        # O período que daria os pontos pedidos — SATURADO no mínimo que o
        # firmware aceita. Sugerir 1/(f·5) cru mandava o operador configurar
        # um `t` fora da faixa [0.02, 3600] s do ServoJ, que o controlador
        # recusa: o conselho não tinha como funcionar.
        want_s = 1.0 / (prof.freq_hz * _FMOD_MIN_PTS_PER_CYCLE)
        hw_max_hz = _fmod_max_freq_hz(_SERVOJ_T_MIN_S)
        if want_s < _SERVOJ_T_MIN_S:
            out.append(('error',
                f'[FMOD] {prof.freq_hz:.2f} Hz não é alcançável em NENHUMA '
                f'configuração: exigiria ServoJ com t={want_s*1e3:.1f} ms, '
                f'abaixo do mínimo de {_SERVOJ_T_MIN_S*1e3:.0f} ms do '
                f'firmware do CR10 (faixa [0.02, 3600] s). O teto absoluto da '
                f'bancada é {hw_max_hz:.2f} Hz com '
                f'{_FMOD_MIN_PTS_PER_CYCLE} pontos por período. Baixe a '
                f'frequência. Modulação cancelada.'))
        else:
            out.append(('error',
                f'[FMOD] {prof.freq_hz:.2f} Hz é mais do que o laço ServoJ '
                f'consegue rastrear: com '
                f'servoj_period_s={servoj_period_s*1e3:.0f} ms o teto é '
                f'{f_max_hz:.2f} Hz ({_FMOD_MIN_PTS_PER_CYCLE} pontos por '
                f'período). Baixe a frequência, ou relance com '
                f'servoj_period_s:={want_s:.3f} — um argumento só, que ajusta '
                f'explorer, GUI e mirror_node juntos: publicar a onda mais '
                f'rápido do que o braço é comandado não entrega mais onda, '
                f'entrega uma reamostrada. Modulação cancelada.'))

    # ── 2. o que a AMOSTRAGEM come, devolvido na amplitude ───────────
    # Malha aberta e conhecido de antemão — não faz sentido deixar a adaptação
    # por ciclo redescobri-lo às cegas. Entra ANTES da checagem de velocidade
    # porque é curso a MAIS: medir a velocidade de pico sobre a amplitude não
    # compensada deixaria passar um ensaio 14 % mais rápido que o teto.
    samp_gain = _fmod_sampling_gain(pts)
    amp_pre = 1.0 / max(samp_gain, 0.5)
    if amp_pre > 1.01:
        out.append(('info',
            f'[FMOD] {pts:.1f} pontos por período entregam '
            f'{100*samp_gain:.1f} % da fundamental (sinc² da interpolação) — '
            f'a amplitude comandada sai multiplicada por {amp_pre:.3f} para '
            f'compensar. A DISTORÇÃO que a mesma interpolação gera (~7 % de '
            f'THD a 5 pontos, ~2 % a 8) não tem como ser compensada em '
            f'amplitude; ela vai medida no log de fim.'))

    # ── 3. VELOCIDADE de pico ────────────────────────────────────────
    # A amplitude em POSIÇÃO é imposta pelo material: a faixa de força pedida
    # vale tantos mm de penetração, e percorrê-los na frequência pedida custa
    # 2·π·f·amp de velocidade de pico. Os tetos de dentro do laço cortam passo
    # a passo e não veem isto; aqui dá para dizer NÃO antes de o braço se
    # mexer. É este o limite REAL de amplitude em alta frequência.
    v_peak_mms = 2.0 * math.pi * prof.freq_hz * amp_m * amp_pre * 1e3
    if v_peak_mms > _FMOD_V_PEAK_MAX_MMS:
        # As DUAS saídas, porque as duas são decisões do operador: baixar a
        # frequência mantendo a faixa de força, ou manter a frequência
        # estreitando a faixa. Antes só havia a primeira, e ela saía calculada
        # sobre o teto de AVISO (20 mm/s) e não sobre o de RECUSA (40): o
        # conselho mandava baixar para metade do que já teria passado.
        # ARREDONDADOS PARA BAIXO, na casa em que são impressos. O valor
        # exato cai EM CIMA do teto, e o teste é `>`: quem seguisse o conselho
        # ao pé da letra levava a mesma recusa de volta. Um conselho que não é
        # executável é pior que nenhum — manda o operador repetir o ensaio
        # para descobrir que não mudou nada.
        f_ok = math.floor(100.0 * _FMOD_V_PEAK_MAX_MMS * 1e-3 / (
            2.0 * math.pi * max(amp_m, 1e-12) * amp_pre)) / 100.0
        amp_ok_m = _FMOD_V_PEAK_MAX_MMS * 1e-3 / (
            2.0 * math.pi * prof.freq_hz * amp_pre)
        amp_ok_n = math.floor(100.0 * amp_ok_m * (
            prof.amp_n / amp_m if amp_m > 1e-12 else k_nm)) / 100.0
        out.append(('error',
            f'[FMOD] a faixa {prof.f_min_n:.2f}–{prof.f_max_n:.2f} N vale '
            f'{2*amp_m*1e3:.2f} mm de curso NESTE material; percorrê-la a '
            f'{prof.freq_hz:.2f} Hz pede {v_peak_mms:.1f} mm/s de pico, acima '
            f'do teto de {_FMOD_V_PEAK_MAX_MMS:.0f} mm/s. Duas saídas: baixar '
            f'a frequência para ≤{f_ok:.2f} Hz com esta faixa, ou manter '
            f'{prof.freq_hz:.2f} Hz estreitando a faixa para ±{amp_ok_n:.2f} '
            f'N em torno de {prof.mean_n:.2f} N '
            f'({prof.mean_n - amp_ok_n:.2f}–{prof.mean_n + amp_ok_n:.2f} N). '
            f'Modulação cancelada.'))
    elif v_peak_mms > _FMOD_V_PEAK_WARN_MMS:
        out.append(('warn',
            f'[FMOD] velocidade de pico {v_peak_mms:.1f} mm/s '
            f'({2*amp_m*1e3:.2f} mm p-p a {prof.freq_hz:.2f} Hz) acima de '
            f'{_FMOD_V_PEAK_WARN_MMS:.0f} mm/s — a onda é rápida para uma '
            f'ponteira de palpação. É o que a faixa de força pedida custa '
            f'neste material; estreite a faixa ou baixe a frequência se não '
            f'for intencional.'))

    # ── 4. a onda é MEDÍVEL nesta frequência? ────────────────────────
    # Tudo o que audita o ensaio — amplitude da fundamental, THD, bins de fase
    # do ILC — sai da mesma sequência de amostras da célula. Sem amostras por
    # período suficientes o ensaio não sai impreciso: sai ALIASADO, e o log de
    # fim imprime números que não descrevem onda nenhuma.
    need_rate_hz = prof.freq_hz * _FMOD_MIN_MEAS_RATE_MULT
    if 0.0 < meas_rate_hz < need_rate_hz:
        out.append(('error',
            f'[FMOD] a célula entrega {meas_rate_hz:.0f} Hz e uma onda de '
            f'{prof.freq_hz:.2f} Hz precisa de {need_rate_hz:.0f} Hz '
            f'({_FMOD_MIN_MEAS_RATE_MULT:.0f} amostras por período) para ser '
            f'medida: com {meas_rate_hz/max(prof.freq_hz, 1e-9):.1f} amostras '
            f'por período a fundamental e os harmônicos do relatório de FORMA '
            f'dobram uns sobre os outros. O ensaio rodaria, mas o log de fim '
            f'não descreveria a onda. Baixe a frequência para '
            f'≤{meas_rate_hz/_FMOD_MIN_MEAS_RATE_MULT:.2f} Hz, ou use a '
            f'FA7155 (~400 Hz) no lugar da HX711 (24 Hz). Modulação '
            f'cancelada.'))

    # ── 5. existe correção de FORMA nesta frequência? ────────────────
    # O ILC é a única coisa que corrige centro, fase e forma; `fx_gain` é um
    # escalar e só mexe em amplitude. E o ILC fecha contra a força lida: no
    # Float32 ela passa pelo One-Euro travado em 2 Hz, então acima de ~2 Hz
    # ele "corrigiria" o filtro — simulado a 10 Hz, leva a amplitude a 149 % e
    # o pico a 2,62 N numa onda pedida até 2,00 N. O portão que o desliga ali
    # sempre existiu; o que faltava era dizer que, desligado, o ensaio acima
    # de 2 Hz não tem como sair preciso. A saída não é um parâmetro novo, é
    # ligar o canal cru.
    meas_gain = 1.0 if has_raw else fmod_measure_gain(prof.freq_hz)
    if meas_gain < _FMOD_ILC_MIN_MEAS_GAIN:
        out.append(('error',
            f'[FMOD] a {prof.freq_hz:.2f} Hz o pipeline de medida entrega '
            f'{100*meas_gain:.0f} % da amplitude (One-Euro travado em '
            f'{_ONE_EURO_MAXCUTOFF_HZ:.0f} Hz), abaixo dos '
            f'{100*_FMOD_ILC_MIN_MEAS_GAIN:.0f} % que uma correção por ciclo '
            f'exige — sem ILC não há correção de centro, fase nem forma, e a '
            f'onda sairia em malha aberta. Publique /load_cell/sample_net (o '
            f'ft_receiver já publica; confira lc_raw_scale_n_per_unit) e o '
            f'ganho da medida vira 1,00 em qualquer frequência. Sem ele o '
            f'teto útil é {_ONE_EURO_MAXCUTOFF_HZ:.0f} Hz. Modulação '
            f'cancelada.'))

    # ── 6. a onda cabe ACIMA da banda morta do ServoJ? ───────────────
    # O espelho só reenvia ServoJ quando o alvo mudou mais que a banda morta,
    # que vale ~12 µm de TCP. Numa ponteira rígida a onda é micrométrica por
    # construção (Δx = ΔF/K), e ±0,5 N em 28 N/mm são 18 µm de pico: cerca de
    # um degrau e meio por semiciclo. A onda sai — quadrada. Avisa e não
    # recusa: é escolha de ponteira, não erro de configuração.
    if amp_m * amp_pre < _FMOD_DEADBAND_MIN_RATIO * deadband_tcp_m:
        out.append(('warn',
            f'[FMOD] amplitude de {amp_m*amp_pre*1e6:.0f} µm contra uma banda '
            f'morta de ServoJ de {deadband_tcp_m*1e6:.0f} µm de TCP — a onda '
            f'comandada sai quantizada em '
            f'~{2*amp_m*amp_pre/max(deadband_tcp_m, 1e-12):.0f} degraus por '
            f'período e a forma vai sofrer. É a ponteira: nesta rigidez a '
            f'faixa de força pedida vale pouco curso. Use uma ponteira mais '
            f'mole ou uma faixa mais larga se a FORMA importa neste ensaio.'))

    # Estimativa a priori: assume tick EXATO de wave_dt. O tick real é sempre
    # maior (o sleep vem depois do Jacobiano/publish), então isto é o MELHOR
    # CASO — a contagem medida sai no fim, e o aviso de verdade vem dela.
    if pts < _FMOD_MIN_PTS_PER_CYCLE:
        out.append(('warn',
            f'[FMOD] {prof.freq_hz:.1f} Hz dá {pts:.1f} pontos por período NO '
            f'MELHOR CASO, com o tick já no piso de '
            f'{max(_FMOD_DT_MIN_S, servoj_period_s)*1e3:.0f} ms. Confira a '
            f'frequência ENTREGUE no log de fim: é ela que vale.'))

    out.append(('info',
        f'[FMOD] {prof.describe()} — média {prof.mean_n:.2f} N, amplitude '
        f'±{prof.amp_n:.2f} N = ±{amp_m*1e6:.0f} µm de penetração '
        f'({"curva F(x)" if use_curve else f"K={k_nm/1e3:.2f} N/mm"}), pico '
        f'{v_peak_mms:.1f} mm/s, tick {wave_dt*1e3:.1f} ms → {pts:.1f} '
        f'pts/período (ServoJ {servoj_period_s*1e3:.0f} ms, teto '
        f'{f_max_hz:.2f} Hz).'))
    return amp_pre, out


class _WaveLockIn:
    """Lock-in da onda, alimentado AMOSTRA A AMOSTRA da célula.

    POR QUE POR AMOSTRA, e não por tick do laço. O laço da onda roda no tick
    do COMANDO — _FMOD_MIN_PTS_PER_CYCLE pontos por período, 50 Hz a 10 Hz. A
    célula entrega ~400 Hz. Ler a célula no tick amostrava 400 Hz de sinal a
    50, e o que a interpolação linear do comando gera de harmônico cai
    justamente em 4f e 6f: com 5 amostras por período os dois DOBRAM sobre a
    própria fundamental. O lock-in passava a medir a amplitude contaminada
    pela distorção que ele existe para denunciar — e o `fx_gain` e o ILC
    fechavam contra esse número.

    Com cada amostra entrando com o SEU instante, a medida roda na taxa da
    célula, independente da taxa do comando. É o que torna 10 Hz medível.

    Guarda dois conjuntos. O do CICLO corrente (fasor da fundamental, contagem
    e extremos) alimenta a adaptação por ciclo e é zerado por `reset_cycle()`;
    o do ENSAIO inteiro (harmônicos 1..N e extremos) vai para o log de fim e
    nunca é zerado.

    Classe pura — testável sem ROS e sem bancada.
    """

    def __init__(self, freq_hz: float, n_harmonics: int = 3):
        self.f = float(freq_hz)
        self.cyc_i = self.cyc_q = 0.0
        self.cyc_n = 0
        self.cyc_min: float | None = None
        self.cyc_max: float | None = None
        self.tot_h = [[0.0, 0.0] for _ in range(int(n_harmonics))]
        self.tot_n = 0
        self.f_min: float | None = None
        self.f_max: float | None = None

    def add(self, t_s: float, fz_n: float) -> None:
        """Uma amostra da célula. `t_s` é o instante DA AMOSTRA contado do
        início da onda, não o do tick que a drenou."""
        w = 2.0 * math.pi * self.f * t_s
        self.cyc_i += fz_n * math.sin(w)
        self.cyc_q += fz_n * math.cos(w)
        self.cyc_n += 1
        self.f_min = fz_n if self.f_min is None else min(self.f_min, fz_n)
        self.f_max = fz_n if self.f_max is None else max(self.f_max, fz_n)
        self.cyc_min = (fz_n if self.cyc_min is None
                        else min(self.cyc_min, fz_n))
        self.cyc_max = (fz_n if self.cyc_max is None
                        else max(self.cyc_max, fz_n))
        for h in range(len(self.tot_h)):
            wh = (h + 1) * w
            self.tot_h[h][0] += fz_n * math.sin(wh)
            self.tot_h[h][1] += fz_n * math.cos(wh)
        self.tot_n += 1

    def cycle_pp_n(self) -> float:
        """Pico-a-pico da FUNDAMENTAL do ciclo = 4·|Σ x·e^{-jωt}|/N.

        A fase não entra — o módulo a descarta —, então o atraso de transporte
        do executor (~85 ms medidos, que a 2 Hz já valem 60°) não contamina a
        medida como contaminaria uma comparação instantânea comandado ×
        medido."""
        return 4.0 * math.hypot(self.cyc_i, self.cyc_q) / max(self.cyc_n, 1)

    def cycle_phase(self) -> float:
        """Fase do fasor do ciclo (rad). Comparada com a do COMANDO, dá
        ∠G(jω) — a fase do plano comandado→entregue, que é o que o ILC precisa
        para indexar a correção no bin certo."""
        return math.atan2(self.cyc_q, self.cyc_i)

    def reset_cycle(self) -> None:
        self.cyc_i = self.cyc_q = 0.0
        self.cyc_n = 0
        self.cyc_min = self.cyc_max = None

    def harmonic_amps(self) -> list[float]:
        """Amplitude de pico de cada harmônico no ensaio INTEIRO. O 1º é a
        amplitude que caracteriza a senoide; a raiz da soma dos outros sobre
        ele é a distorção."""
        return [2.0 * math.hypot(a, b) / max(self.tot_n, 1)
                for a, b in self.tot_h]


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
