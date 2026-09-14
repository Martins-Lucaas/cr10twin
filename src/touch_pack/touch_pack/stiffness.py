"""stiffness.py — quão dura é a amostra, e a que velocidade dá para encostar nela.

A rigidez do que está sob a ponteira não é dado de entrada: ela se descobre
enquanto se desce. E decide as duas coisas que mais estragam um ensaio — com
que velocidade rastejar (rápido demais num material duro vira pico de impacto;
devagar demais num mole custa minutos por ponto) e quanto empurrar para chegar
a um setpoint.

  * `_StiffnessEstimator` — K (N/m) a partir dos pares (deslocamento, força) da
    própria descida, com teto e piso para que um par ruim não vire comando.
  * `_ContactCurve` — a curva força×penetração acumulada, que dá a distância
    entre dois níveis de força sem supor K constante.
  * `crawl_v_ms` / `impact_peak_n` / `setpoint_resolvable` — as três contas que
    traduzem K em velocidade segura, pico previsto e "este setpoint se
    distingue do ruído da célula?".

Saiu do `tactile_explorer` pelo mesmo motivo da onda: é física da amostra, não
máquina de estados, e roda inteira sem ROS.
"""
from __future__ import annotations

import math

import numpy as np

from .constants import (
    CONTACT_ON_N as _CONTACT_ON_N,
    FORCE_NOISE_SIGMA_N as _FORCE_NOISE_SIGMA_N,
    HOLD_TOL_SIGMA as _HOLD_TOL_SIGMA,
)
from .explorer_constants import (
    _DESCEND_CRAWL_V_MIN_MS, _DESCEND_TOUCH_V_MS, _FX_MIN_POINTS,
    _FX_MIN_SEG_DF_N, _FX_MIN_SPAN_N, _K_DEFAULT_NM, _K_EMA_ALPHA,
    _K_MAX_NM, _K_MIN_NM, _K_PAIR_MIN_DF_N, _K_PAIR_MIN_DX_M,
    _QS_K_PUSH_MARGIN, _STREAM_HALT_LAT_S
)


def crawl_v_ms(k_nm: float,
               t_halt_s: float = _STREAM_HALT_LAT_S) -> float:
    """Velocidade de rastejo (m/s) que faz o PRIMEIRO IMPACTO parar no limiar
    de contato: `v = _CONTACT_ON_N / (T_halt · K)`.

    O orçamento era `alvo + tol` até 27/08/2026, e com ele o primeiro toque
    tinha licença para chegar ao setpoint SOZINHO — o transiente de impacto
    entregava a força inteira do ensaio antes de qualquer laço reagir, e a
    regulação quase-estática só arrumava o que sobrasse. Contra um alvo de
    5 N isso é um golpe de 5 N numa amostra que pode ser biológica.

    Agora o impacto mira em DETECTAR, não em medir: o transiente para no
    limiar de contato e quem sobe de lá até o setpoint é o regulador, em
    micro-passos, com as três guardas de não-ultrapassagem. O pico do toque
    deixa de depender do setpoint — 0,2 N e 5 N tocam com a mesma força.

    O QUE ISSO CUSTA. `v` é linear no orçamento, então cortá-lo de `alvo+tol`
    para 0,1 N divide a velocidade de rastejo na mesma razão: contra a ponta
    rígida de referência, um alvo de 1,6 N descia nos 200 µm/s do teto e
    passa a descer a ~12 µm/s. Sem contato aprendido a descida INTEIRA roda
    nessa velocidade (ver o perfil de dois estágios em `_phase_descending`),
    então o primeiro toque de uma home nova fica caro; do segundo em diante o
    estágio rápido cobre tudo menos a zona de incerteza.

    E o grosso desse custo NÃO é o orçamento, é `T_halt`: os 0,3 s são
    emprestados de `_ZONE_REACTION_S` e nunca foram MEDIDOS. `v` é linear em
    1/T_halt, e a latência de transporte medida no executor da onda é de
    ~85 ms — se ela valer aqui, o rastejo volta para ~42 µm/s só com a
    medição. `latency_probe.py` é o instrumento.

    `k_nm` deve ser a ponta RÍGIDA de referência, não a K estimada: antes do
    contato não existe estimativa, e errar para o lado mole custa FORÇA.
    Contra silicone a conta estoura o teto e o clip resolve — custa tempo.
    """
    k = max(1.0, float(k_nm))
    t = max(1e-3, float(t_halt_s))
    v = _CONTACT_ON_N / (t * k)
    return float(np.clip(v, _DESCEND_CRAWL_V_MIN_MS, _DESCEND_TOUCH_V_MS))


def impact_peak_n(v_ms: float, k_nm: float,
                  t_halt_s: float = _STREAM_HALT_LAT_S) -> float:
    """Pico do toque (N) para uma velocidade JÁ comandada: `v · T_halt · K`.

    É o mesmo modelo de `crawl_v_ms`, invertido. Existe separado porque
    `crawl_v_ms` devolve a velocidade CLIPADA, e o clip pode tornar o
    orçamento inalcançável sem que nada acuse: em `_DESCEND_CRAWL_V_MIN_MS`
    (10 µm/s) o piso passa a mandar acima de 33 kN/m, e a 900 kN/m — a rigidez
    que o próprio `_StiffnessEstimator` cita para um sensor bem fixo — o pico
    real vira 2,7 N contra um orçamento de 0,1 N.

    Quem chama compara o resultado com `_CONTACT_ON_N` e AVISA. Baixar o piso
    não é opção: abaixo de 10 µm/s um tick de 30 ms não move nem 0,3 µm e a
    descida some no quantum de 10 µm da FK. O que resta é dizer a verdade.
    """
    return float(v_ms) * max(1e-3, float(t_halt_s)) * max(1.0, float(k_nm))


def setpoint_resolvable(target_f: float, tol_n: float,
                        contact_on_n: float = _CONTACT_ON_N) -> tuple[bool, str]:
    """O setpoint pedido é distinguível do próprio limiar de contato?

    Se a borda INFERIOR da banda cai em `_CONTACT_ON_N` ou abaixo, o laço pode
    declarar "cheguei" numa força que o sistema nem considera contato, e
    "overshoot" deixa de ser mensurável. Só AVISA — quem decide é o operador,
    e quem move esse piso é σ da célula, não o controle.
    """
    if float(target_f) - float(tol_n) > float(contact_on_n):
        return True, ''
    return False, (
        f'setpoint {target_f:.2f} N com banda +-{tol_n:.2f} N desce ate '
        f'{target_f - tol_n:.2f} N, em/abaixo do limiar de contato '
        f'({contact_on_n:.2f} N): a chegada nao se distingue de "sem contato". '
        f'O piso da banda e o ruido da celula (sigma={_FORCE_NOISE_SIGMA_N:.3f} '
        f'N x {_HOLD_TOL_SIGMA:.0f}); o menor alvo com sentido hoje e '
        f'~{contact_on_n + tol_n:.2f} N. Re-medir sigma com a FA7155 baixa '
        'esse piso.')


class _StiffnessEstimator:
    """Estima a rigidez de contato K = ΔF/Δx (N/m) online, por EMA."""

    def __init__(self):
        self.reset()

    def reset(self, k0: float = _K_DEFAULT_NM):
        self.k = float(k0)
        self._f_prev: float | None = None
        # Acumuladores do update_pair — zerados junto com o resto, senão o
        # trecho de um contato vazaria para o próximo.
        self._acc_dx = 0.0
        self._acc_df = 0.0
        # Secante do ÚLTIMO trecho aceito, sem a EMA. Num contato que
        # enrijece ela é a medida mais próxima da inclinação que vem pela
        # frente, e é o que sustenta a cota superior `k_upper`.
        self.k_last: float | None = None
        self.estimated = False

    def _absorb(self, k_inst: float) -> None:
        """Incorpora um k_inst medido, com adaptação ASSIMÉTRICA: sobe na hora,
        desce por EMA.

        As duas direções do erro de K não custam a mesma coisa. O regulador
        dimensiona o passo por Δx = relax·err/K e o limita por hard_cap = ΔF/K:
        superestimar K só encurta o passo (custa tempo), mas SUBESTIMAR o
        alonga na razão K_real/K_est — e o excesso vira força.

        A curva F(x) deste contato enrijece muito ao longo da penetração
        (0,279 N/mm no pé, ~1 N/mm perto do alvo), então o EMA, que aprende no
        pé e demora ~4 pares para acompanhar, chega SEMPRE atrasado na subida —
        e cada um desses 4 pares é um passo grande demais. Foi o que aconteceu
        no run MANUAL/20260817_142719 (alvo 0,5 N): o estimador vinha com
        K_est≈5 N/mm, o hard_cap de 0,2 N virou 40 µm, e 40 µm contra os
        18 N/mm que o contato realmente tinha ali entregaram 0,72 N por passo —
        3,6x o teto que o laço acreditava estar aplicando. O resultado foi um
        ciclo-limite de 2,5 s entre 0,1 e 1,11 N (overshoot de 0,61 N) com o
        braço praticamente parado (40 µm de curso no evento inteiro).

        Subir na hora torna o teto por ΔF verdadeiro já no passo SEGUINTE ao
        primeiro contato mais rígido; descer por EMA preserva a robustez a um
        par isolado ruidoso, que é o que o filtro existe para dar.
        """
        a = (1.0 if (not self.estimated or k_inst > self.k)
             else _K_EMA_ALPHA)
        self.k = (1.0 - a) * self.k + a * k_inst
        # A secante do ÚLTIMO trecho é registrada mesmo quando a EMA a
        # amortece: é ela que sustenta `k_upper`/`k_push` (ver a property
        # k_push), e sem ela o passo volta a ser dimensionado pelo trecho
        # mole já percorrido — o overshoot que o k_upper existe para evitar.
        self.k_last = k_inst
        self.estimated = True

    def update(self, dx_cmd_m: float, f_now: float, in_contact: bool):
        f_prev = self._f_prev
        self._f_prev = f_now
        # Limiar de Δx: abaixo disso k_inst = ΔF/Δx vira ruído puro.
        if not in_contact or f_prev is None or abs(dx_cmd_m) < 8e-6:
            return
        k_inst = (f_now - f_prev) / dx_cmd_m   # N/m (assinado: dx e dF mesmo sinal)
        if _K_MIN_NM <= k_inst <= _K_MAX_NM:
            self._absorb(k_inst)

    def update_pair(self, dx_m: float, df_n: float):
        """Par (Δx executado, ΔF medido) com AMBAS as forças lidas em REPOUSO
        (modo quase-estático) — sem o erro de fase do update() contínuo, o
        k_inst é a rigidez real do trecho percorrido.

        ΔF abaixo do ruído da célula é ACUMULADO, não descartado. A versão
        antiga jogava fora todo par com |ΔF| < 0,1 N, o que criava um impasse
        circular em contato mole: antes do primeiro K_est o regulador limita o
        passo a 8 µm (o teto de sonda de então), e 8 µm numa ponteira de silicone
        (0,62 N/mm) produzem 0,005 N — sempre abaixo do limiar. O par era
        descartado, K nunca era estimado, o teto de sonda continuava valendo, e
        a descida rastejava (25 s sem chegar ao setpoint, run 20260814_102401).

        Somar passos consecutivos resolve sem baixar o limiar de ruído: a razão
        ΣΔF/ΣΔx é a secante do trecho percorrido — a mesma grandeza física do
        par individual, com relação sinal-ruído proporcional ao número de
        passos somados.

        O Δx entra no acumulador SEMPRE, e é o Δx SOMADO que precisa cruzar
        _K_PAIR_MIN_DX_M. Descartar o passo individual pequeno (o que se fazia
        antes) trancava o estimador no canto contato-mole + alvo-baixo: com
        `k_upper` na cota do resultado nulo abaixo, a folga até a borda da
        banda comanda passos de ~0,17 µm, abaixo do mínimo do par — nada
        acumulava, a cota não afrouxava, e os passos ficavam nos 0,17 µm para
        sempre. Somar o Δx é a mesma cura que o ΔF já recebia."""
        self._acc_dx += dx_m
        self._acc_df += df_n
        if abs(self._acc_df) < _K_PAIR_MIN_DF_N:
            return
        if abs(self._acc_dx) < _K_PAIR_MIN_DX_M:
            # Passos de sinais opostos que se cancelaram. NÃO é hipotético: o
            # regulador satura em ±hard_cap e ±dx_max_m, que são simétricos,
            # então dois passos saturados em sentidos contrários somam
            # EXATAMENTE 0.0 em ponto flutuante — e o ΔF entre eles não
            # cancela junto. Dividir aqui matava a thread do protocolo com a
            # ponteira dentro da amostra. O trecho percorrido é nulo: não
            # mede rigidez nenhuma, e o acumulador recomeça.
            self._acc_dx = 0.0
            self._acc_df = 0.0
            return
        k_inst = self._acc_df / self._acc_dx
        self._acc_dx = 0.0
        self._acc_df = 0.0
        if _K_MIN_NM <= k_inst <= _K_MAX_NM:
            self._absorb(k_inst)

    @property
    def value(self) -> float:
        return float(min(max(self.k, _K_MIN_NM), _K_MAX_NM))

    @property
    def k_upper(self) -> float:
        """Cota SUPERIOR da rigidez local (N/m) — o K que TODO teto de passo
        de EMPURRAR usa, no lugar da EMA.

        A EMA responde pelo trecho JÁ percorrido. Num contato que enrijece
        (silicone medido: 0,18 N/mm no pé, 3,0 N/mm perto de 2 mm) o trecho
        que vem pela frente é mais duro que ela, e dimensionar o passo pela
        EMA entrega várias vezes o ΔF pedido — foi assim que o overshoot
        nasceu. A secante do ÚLTIMO trecho (`k_last`) é a medida mais
        próxima da inclinação seguinte; a margem cobre o quanto a curva
        ainda enrijece DENTRO do próximo passo.

        Antes de qualquer par aceito NÃO se usa o default: ele é otimista
        (40 N/mm) e a rampa sozinha não segurava. A pilha FA7155 + ponteira F
        é curta e maciça, e num contato rígido de verdade o primeiro passo
        dimensionado com 40 N/mm entrega vários newtons — a 900 N/mm o alívio
        seguinte, também dimensionado com o default, recuava tanto que largava
        o contato, e o par nunca se formava: o regulador entrava em ciclo
        QUIQUE indefinido (medido em simulação: alvo de 0,5 N fechando a
        5,0 N).

        O que substitui o default é uma cota deduzida do RESULTADO NULO: se o
        acumulador ainda não cruzou _K_PAIR_MIN_DF_N depois de ΣΔx de curso,
        então |ΔF| < _K_PAIR_MIN_DF_N nesse trecho e portanto
        K < _K_PAIR_MIN_DF_N / ΣΔx. É um limite SUPERIOR rigoroso, que é
        exatamente a grandeza que os tetos de passo precisam, e afrouxa
        sozinho conforme o curso sem resposta cresce — contato mole volta ao
        ritmo normal em poucos ticks em vez de rastejar. Sem nenhum curso
        acumulado não há informação alguma: vale o teto do estimador.
        """
        if not self.estimated:
            if abs(self._acc_dx) > 0.0:
                k_null = _K_PAIR_MIN_DF_N / abs(self._acc_dx)
                return float(min(max(k_null, _K_MIN_NM), _K_MAX_NM))
            return float(_K_MAX_NM)
        k = max(self.value, self.k_last or 0.0) * _QS_K_PUSH_MARGIN
        return float(min(max(k, _K_MIN_NM), _K_MAX_NM))


class _ContactCurve:
    """Curva F(x) MEDIDA: pares (penetração comandada, força em REPOUSO)
    coletados pela regulação quase-estática enquanto ela desce até o alvo.

    Existe porque a rigidez deste contato NÃO é um escalar. Medida em
    14/08/2026 sobre o run TOUCH/20260814_115804 (ponteira de silicone), a
    secante local vale 0,18 N/mm entre 0,06 e 0,20 N, 1,41 N/mm entre 1,0 e
    1,5 N e 2,8–6,2 N/mm dentro da onda de 0,4 a 3,9 N — 34× de variação
    DENTRO da faixa que o ensaio percorre. Um K único calibrado no pé da
    curva comanda várias vezes a amplitude necessária no topo: a onda pedida
    de 0,1–3,0 N a 5 Hz recebeu K=0,70 N/mm da descida, comandou 2,07 mm de
    amplitude (4,03 mm p-p, 63 mm/s de pico) e estourou o teto de força em
    30 %, sem nunca chegar ao piso de 0,1 N.

    A descida já percorre a curva ponto a ponto e a jogava fora ao colapsá-la
    em `_StiffnessEstimator.value`. Aqui ela é guardada e INVERTIDA:
    `dx_between(f_a, f_b)` devolve quanta penetração separa duas forças, que
    é exatamente o que o feedforward da onda precisa — e a não-linearidade
    fica embutida, sem escalar nenhum.
    """

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self._pts: list[tuple[float, float]] = []   # (força N, penetração m)

    def add(self, x_m: float, f_n: float) -> None:
        """Registra um par. `x_m` é a penetração COMANDADA acumulada e `f_n`
        a força lida em REPOUSO — as duas grandezas que o move-then-measure
        já produz a cada micro-passo."""
        if math.isfinite(x_m) and math.isfinite(f_n):
            self._pts.append((float(f_n), float(x_m)))

    def _clean(self) -> list[tuple[float, float]]:
        """Pares ordenados por força, com penetração monotônica.

        Ordenar por força é o que torna a curva INVERTÍVEL. O teto corrente
        em x remove as inversões de ruído sem DESCARTAR o par: descartar
        abriria buracos justamente no trecho mole, que é onde a curva mais
        difere de uma reta.
        """
        out: list[tuple[float, float]] = []
        x_run = -math.inf
        for f_n, x_m in sorted(self._pts):
            x_run = max(x_run, x_m)
            if out and f_n - out[-1][0] < 1e-9:
                out[-1] = (out[-1][0], x_run)   # mesma força: fica o mais fundo
            else:
                out.append((f_n, x_run))
        return out

    @property
    def usable(self) -> bool:
        pts = self._clean()
        return (len(pts) >= _FX_MIN_POINTS
                and pts[-1][0] - pts[0][0] >= _FX_MIN_SPAN_N
                and pts[-1][1] - pts[0][1] > 1e-6)

    @property
    def f_range(self) -> tuple[float, float]:
        pts = self._clean()
        return (pts[0][0], pts[-1][0]) if pts else (0.0, 0.0)

    def _edge_k(self, pts: list[tuple[float, float]], top: bool) -> float:
        """Rigidez do segmento da ponta — usada para EXTRAPOLAR fora do
        medido. A onda pode pedir força acima do alvo da descida (0,1–3,0 N
        contra um alvo de 1,5 N), e é este trecho que responde por lá."""
        seq = list(reversed(pts)) if top else pts
        f_ref, x_ref = seq[0]
        for f_n, x_m in seq[1:]:
            if abs(f_ref - f_n) >= _FX_MIN_SEG_DF_N and abs(x_ref - x_m) > 1e-9:
                return abs(f_ref - f_n) / abs(x_ref - x_m)
        return _K_DEFAULT_NM

    def x_of_f(self, f_n: float) -> float:
        """Penetração (m) que corresponde a esta força, na mesma origem em
        que os pares foram registrados."""
        pts = self._clean()
        if not pts:
            return 0.0
        if f_n <= pts[0][0]:
            k = min(max(self._edge_k(pts, False), _K_MIN_NM), _K_MAX_NM)
            return pts[0][1] + (f_n - pts[0][0]) / k
        if f_n >= pts[-1][0]:
            k = min(max(self._edge_k(pts, True), _K_MIN_NM), _K_MAX_NM)
            return pts[-1][1] + (f_n - pts[-1][0]) / k
        for (f0, x0), (f1, x1) in zip(pts, pts[1:]):
            if f0 <= f_n <= f1:
                if f1 - f0 < 1e-9:
                    return x1
                return x0 + (x1 - x0) * (f_n - f0) / (f1 - f0)
        return pts[-1][1]

    def dx_between(self, f_a: float, f_b: float) -> float:
        """Penetração que separa duas forças (m). A origem se cancela, então
        não importa onde os pares foram zerados."""
        return self.x_of_f(f_b) - self.x_of_f(f_a)

    def k_secant(self, f_a: float, f_b: float) -> float:
        """Rigidez secante (N/m) entre duas forças — só para log."""
        dx = abs(self.dx_between(f_a, f_b))
        return abs(f_b - f_a) / dx if dx > 1e-9 else _K_MAX_NM
