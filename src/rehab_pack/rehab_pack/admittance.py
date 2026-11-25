"""Núcleo de controle de ADMITÂNCIA cartesiana (guiar membro em reabilitação).

O CR10 é controlado por posição (ServoJ a 33 Hz), então impedância "pura"
(posição entra, força sai) não existe aqui. A lei é a admitância:

    M·a + B·v + K·(x − x_ref) = F_ext          (por eixo, 3 DOF)

A força medida entra, o deslocamento de setpoint sai. K=0 é guiagem livre
(o robô acompanha a mão); K>0 puxa o membro de volta à trajetória de
referência com rigidez virtual K [N/m]; B dá o amortecimento.

Integração por Euler IMPLÍCITO: não injeta energia numérica, para qualquer M, B, K.
O que limita M e B é o atraso da malha (força → sim → ServoJ → robô) contra a
rigidez do braço da pessoa: ver `arm_stiffness_limit`.

Admitância variável: com a velocidade MEDIDA, observa a potência na mão. Se a
fração devolvida pelo robô passa de IDX_LO, M e B sobem juntos (M/B constante,
então a resposta não fica mais lenta de fase) e voltam devagar. Observador de
passividade inspirado em Dimeas & Aspragathos (2016), Landi et al. (2017) e Ferraguti
et al. (2019); é heurística, não prova de passividade.

Módulo puro (numpy, sem ROS e sem driver): recebe a força JÁ no frame do
setpoint, já tarada (a tara só vale na orientação em que foi feita), e devolve a posição
desejada. Quem chama converte para juntas e envia o ServoJ.

Segurança — por convenção do projeto compressão é positiva:
  * |F| acima de `abort_force_n` levanta AdmittanceAbort (sem setpoint novo);
  * força velha (> stale_s) zera a velocidade em vez de seguir a última lida;
  * velocidade e afastamento (da referência e do ponto onde se habilitou) são saturados.
"""
from __future__ import annotations

import math

import numpy as np

from .constants import FORCE_ABORT_LIMIT_N, LOOP_DELAY_S

# Limiares ajustados em simulação (test_admittance.py); reveja com o robô real.
IDX_LO, IDX_HI = 0.30, 0.50   # fração de energia devolvida: começa a agir / máximo
SCALE_MAX = 8.0
OBS_TAU_S = 1.0               # memória do observador
RELAX_TAU_S = 3.0             # volta de M e B ao nominal
POWER_FLOOR_J = 0.02          # abaixo disto o índice é só ruído


def arm_stiffness_limit(mass_kg, damping_ns_m, delay_s=LOOP_DELAY_S) -> float:
    """Maior rigidez do braço [N/m] com a malha estável (K=0, atraso puro):
    cruzamento de fase de k·e^(−sT)/(Ms²+Bs). K > 0 só aumenta a folga."""
    if damping_ns_m <= 0.0:
        return 0.0
    lo, hi = 0.0, math.pi / (2.0 * delay_s)
    for _ in range(60):
        w = (lo + hi) / 2.0
        if math.atan(w * mass_kg / damping_ns_m) + w * delay_s < math.pi / 2.0:
            lo = w
        else:
            hi = w
    return lo * math.hypot(damping_ns_m, lo * mass_kg)


class AdmittanceAbort(RuntimeError):
    """Força acima do limite de segurança: o controlador para de emitir setpoints."""


class AdmittanceController:
    def __init__(self, mass_kg: float = 2.0, damping_ns_m: float = 40.0,
                 stiffness_n_m: float = 0.0, dt: float = 0.030,
                 deadband_n: float = 0.5, v_max_ms: float = 0.10,
                 max_excursion_m: float = 0.15,
                 abort_force_n: float = FORCE_ABORT_LIMIT_N,
                 stale_s: float = 0.5):
        if mass_kg <= 0.0 or damping_ns_m < 0.0 or stiffness_n_m < 0.0:
            raise ValueError('M deve ser > 0 e B, K >= 0')
        if dt <= 0.0:
            raise ValueError('dt deve ser > 0')
        self.m, self.b, self.k = mass_kg, damping_ns_m, stiffness_n_m
        self.dt = dt
        self.deadband_n = deadband_n
        self.v_max = v_max_ms
        self.max_excursion = max_excursion_m
        self.abort_force_n = abort_force_n
        self.stale_s = stale_s
        self.reset(np.zeros(3))

    def reset(self, x0) -> None:
        """Reinicia no ponto x0 [m]; a referência passa a ser x0."""
        self.x = np.array(x0, dtype=float).reshape(3)
        self.x_ref = self.x.copy()
        self.origin = self.x.copy()   # onde se habilitou: x_ref anda, ela não
        self.v = np.zeros(3)
        self.scale = 1.0              # multiplica M e B
        self._p = self._n = 0.0       # energia trocada / devolvida (J, com esquecimento)

    def _adapt(self, power_w: float) -> None:
        decay = math.exp(-self.dt / OBS_TAU_S)
        self._p = decay * self._p + abs(power_w) * self.dt
        self._n = decay * self._n + max(0.0, -power_w) * self.dt
        idx = self._n / (self._p + POWER_FLOOR_J)
        target = 1.0 + (SCALE_MAX - 1.0) * min(
            max((idx - IDX_LO) / (IDX_HI - IDX_LO), 0.0), 1.0)
        old = self.scale
        self.scale = max(target, old + (1.0 - old) * self.dt / RELAX_TAU_S)
        if self.scale > old:          # subir M não pode criar energia cinética
            self.v *= math.sqrt(old / self.scale)

    def set_reference(self, x_ref) -> None:
        self.x_ref = np.array(x_ref, dtype=float).reshape(3)

    def step(self, force_n, force_age_s: float = 0.0, v_meas=None) -> np.ndarray:
        """Avança um tick e devolve o setpoint de posição [m].

        `v_meas`: velocidade medida do efetuador [m/s]; None = sem adaptação.
        `force_n`: força do membro sobre o efetuador (3,), frame do setpoint.
        """
        f = np.asarray(force_n, dtype=float).reshape(3)
        if not np.all(np.isfinite(f)):
            raise AdmittanceAbort(f'força não-finita: {f}')
        if np.linalg.norm(f) > self.abort_force_n:
            raise AdmittanceAbort(
                f'|F|={np.linalg.norm(f):.1f} N > {self.abort_force_n:.1f} N')
        if force_age_s > self.stale_s:
            # Sem medida confiável: para, não segue a última leitura.
            self.v[:] = 0.0
            return self.x.copy()

        n = np.linalg.norm(f)
        if n <= self.deadband_n:
            f = np.zeros(3)
        else:  # deadband contínua: sem degrau na borda
            f = f * (n - self.deadband_n) / n

        f = f - self.k * (self.x - self.x_ref)
        if v_meas is not None:
            self._adapt(float(f @ np.asarray(v_meas, float)))  # sem a energia da mola K
        m, b = self.m * self.scale, self.b * self.scale
        self.v = (m * self.v + self.dt * f) / (
            m + b * self.dt + self.k * self.dt ** 2)
        sp = np.linalg.norm(self.v)
        if sp > self.v_max:
            self.v *= self.v_max / sp
        x0 = self.x
        self.x = self.x + self.v * self.dt

        # sem mola x_ref não limita nada; origin por último: é o limite absoluto
        for c in ([self.x_ref] if self.k > 0.0 else []) + [self.origin]:
            d = self.x - c
            dn = np.linalg.norm(d)
            # já fora (referência ou excursão mudou): só não se afasta mais, nunca é arrastado
            lim = max(self.max_excursion, float(np.linalg.norm(x0 - c)))
            if dn > lim:
                self.x = c + d * (lim / dn)
                # mata a componente de velocidade que empurra para fora
                out = d / dn
                self.v -= max(0.0, float(self.v @ out)) * out
        # x_ref ou excursão podem mudar em ATIVO: o setpoint não salta
        step = self.x - x0
        sn, lim = float(np.linalg.norm(step)), self.v_max * self.dt
        if sn > lim:
            self.x = x0 + step * (lim / sn)
        return self.x.copy()
