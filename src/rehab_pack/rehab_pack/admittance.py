"""Núcleo de controle de ADMITÂNCIA cartesiana (guiar membro em reabilitação).

O CR10 é controlado por posição (ServoJ a 33 Hz), então impedância "pura"
(posição entra, força sai) não existe aqui. A lei é a admitância:

    M·a + B·v + K·(x − x_ref) = F_ext          (por eixo, 3 DOF)

A força medida entra, o deslocamento de setpoint sai. K=0 é guiagem livre
(o robô acompanha a mão); K>0 puxa o membro de volta à trajetória de
referência com rigidez virtual K [N/m]; B dá o amortecimento.

Módulo puro (numpy, sem ROS e sem driver): recebe a força JÁ no frame do
setpoint, com tara e compensação de peso aplicadas, e devolve a posição
desejada. Quem chama converte para juntas e envia o ServoJ.

Segurança — por convenção do projeto compressão é positiva:
  * |F| acima de `abort_force_n` levanta AdmittanceAbort (sem setpoint novo);
  * força velha (> stale_s) zera a velocidade em vez de seguir a última lida;
  * velocidade e afastamento da referência são saturados.
"""
from __future__ import annotations

import math

import numpy as np

from .constants import FORCE_ABORT_LIMIT_N


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
        # Euler semi-implícito: estável com B·dt/M < 2 e dt < 2·sqrt(M/K).
        if damping_ns_m * dt / mass_kg >= 2.0:
            raise ValueError(
                f'instável: B·dt/M = {damping_ns_m * dt / mass_kg:.2f} >= 2 '
                f'(reduza B, aumente M ou diminua dt)')
        if stiffness_n_m > 0.0 and dt >= 2.0 * math.sqrt(mass_kg / stiffness_n_m):
            raise ValueError(
                f'instável: dt={dt} s >= 2·sqrt(M/K)='
                f'{2.0 * math.sqrt(mass_kg / stiffness_n_m):.3f} s')
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
        self.v = np.zeros(3)

    def set_reference(self, x_ref) -> None:
        self.x_ref = np.array(x_ref, dtype=float).reshape(3)

    def step(self, force_n, force_age_s: float = 0.0) -> np.ndarray:
        """Avança um tick e devolve o setpoint de posição [m].

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

        a = (f - self.b * self.v - self.k * (self.x - self.x_ref)) / self.m
        self.v = self.v + a * self.dt
        sp = np.linalg.norm(self.v)
        if sp > self.v_max:
            self.v *= self.v_max / sp
        self.x = self.x + self.v * self.dt

        d = self.x - self.x_ref
        dn = np.linalg.norm(d)
        if dn > self.max_excursion:
            self.x = self.x_ref + d * (self.max_excursion / dn)
            # mata a componente de velocidade que empurra para fora
            out = d / dn
            self.v -= max(0.0, float(self.v @ out)) * out
        return self.x.copy()
