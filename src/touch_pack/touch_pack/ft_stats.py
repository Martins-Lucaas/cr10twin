"""ft_stats.py — pós-processamento de EXIBIÇÃO, à moda do cliente de fábrica.

O `Six_Axis_FT.exe` da FIBOS aplica um Savitzky-Golay sobre a série antes de
plotar e mostra média e máximo por janela (`Mean_Num`, `MAX_Num`). As duas
mensagens de erro dele — "Window size must be odd." e "Order must be less
than window size." — são a assinatura clássica do SG e estão reproduzidas
aqui palavra por palavra, para quem comparar as duas telas achar o mesmo
texto.

FRONTEIRA IMPORTANTE: isto NÃO entra na malha de controle. Quem filtra o sinal
que o `tactile_explorer` regula é o mediana + One-Euro do `lc_filter.py`, e
mexer nele muda o sentido dos ganhos do controlador (está dito lá). O que este
módulo faz é um segundo caminho, paralelo e só de leitura, para que o número
mostrado e exportado pela nossa GUI possa bater com o do fabricante.

Sem scipy de propósito: `savgol_coeffs` sai de um mínimos-quadrados de 6
linhas em numpy, e scipy não é dependência deste workspace (ver o README).
"""
from __future__ import annotations

import collections
import math
from typing import Deque, Iterable, Optional

import numpy as np

from .constants import (
    FT_SG_ORDER_DEFAULT,
    FT_SG_WINDOW_DEFAULT,
    FT_STATS_WINDOW_DEFAULT,
)


def validate_savgol(window: int, order: int) -> None:
    """Levanta ValueError com o MESMO texto do cliente de fábrica."""
    if window % 2 == 0:
        raise ValueError('Window size must be odd.')
    if order >= window:
        raise ValueError('Order must be less than window size.')
    if window < 3:
        raise ValueError('Window size must be odd.')
    if order < 0:
        raise ValueError('Order must be less than window size.')


def savgol_coeffs(window: int, order: int,
                  pos: Optional[int] = None) -> np.ndarray:
    """Coeficientes FIR do Savitzky-Golay.

    `pos` é a amostra da janela onde o polinômio é avaliado, em 0..window-1.
    None = centro (window//2), que é o SG canônico e o que faz sentido para
    suavizar uma série JÁ GRAVADA.

    Para stream ao vivo o centro custa (window-1)/2 amostras de atraso, então
    o filtro em tempo real abaixo usa pos=window-1 (a ponta): mesma
    suavização, sem defasagem, ao preço de mais variância na borda.
    """
    validate_savgol(window, order)
    if pos is None:
        pos = window // 2
    if not 0 <= pos < window:
        raise ValueError(f'pos fora de 0..{window - 1}: {pos}')
    # Vandermonde dos deslocamentos em torno de `pos`; a linha 0 da
    # pseudo-inversa é o filtro que devolve o próprio valor ajustado.
    x = np.arange(window, dtype=float) - pos
    A = np.vander(x, order + 1, increasing=True)
    return np.linalg.pinv(A)[0]


def savgol_filter(values: Iterable[float], window: int = FT_SG_WINDOW_DEFAULT,
                  order: int = FT_SG_ORDER_DEFAULT) -> np.ndarray:
    """SG centrado sobre uma série completa (uso offline: CSV, relatório).

    As bordas usam coeficientes avaliados na posição real da amostra dentro da
    primeira/última janela, em vez de espelhar o sinal: espelhar inventa dados
    que o sensor não mediu, e num sinal de força com degrau de contato isso
    produz um pico que não existe.
    """
    y = np.asarray(list(values), dtype=float)
    validate_savgol(window, order)
    n = y.size
    if n < window:
        return y.copy()
    half = window // 2
    out = np.empty(n, dtype=float)
    c_mid = savgol_coeffs(window, order)
    # Miolo: uma correlação com os coeficientes centrais.
    out[half:n - half] = np.convolve(y, c_mid[::-1], mode='valid')
    for i in range(half):
        out[i] = float(savgol_coeffs(window, order, pos=i) @ y[:window])
        j = n - 1 - i
        out[j] = float(
            savgol_coeffs(window, order, pos=window - 1 - i) @ y[-window:])
    return out


class StreamingSavGol:
    """SG em tempo real, avaliado na PONTA da janela (sem atraso).

    Enquanto a janela não encheu devolve a própria amostra: um SG de ordem 3
    sobre 4 pontos interpola exatamente e não filtra nada, então fingir que
    filtra só esconderia o transiente de partida.
    """

    def __init__(self, window: int = FT_SG_WINDOW_DEFAULT,
                 order: int = FT_SG_ORDER_DEFAULT):
        self.configure(window, order)

    def configure(self, window: int, order: int) -> None:
        validate_savgol(window, order)
        self.window = int(window)
        self.order = int(order)
        self._c = savgol_coeffs(self.window, self.order, pos=self.window - 1)
        self._buf: Deque[float] = collections.deque(maxlen=self.window)

    def reset(self) -> None:
        self._buf.clear()

    def update(self, v: float) -> float:
        if not math.isfinite(v):
            return v
        self._buf.append(float(v))
        if len(self._buf) < self.window:
            return float(v)
        return float(self._c @ np.fromiter(self._buf, dtype=float,
                                           count=self.window))


class RollingStats:
    """`Mean_Num` / `MAX_Num` do painel de fábrica, mais mínimo e RMS.

    O máximo é do VALOR ABSOLUTO, como no cliente: num eixo bipolar o pico que
    interessa é o de magnitude, e um max ingênuo sobre um sinal negativo
    devolveria o ponto mais perto de zero.
    """

    def __init__(self, window: int = FT_STATS_WINDOW_DEFAULT):
        self.window = int(window)
        self._buf: Deque[float] = collections.deque(maxlen=self.window)

    def resize(self, window: int) -> None:
        if int(window) < 1:
            raise ValueError('janela de estatística tem de ser >= 1')
        self.window = int(window)
        self._buf = collections.deque(self._buf, maxlen=self.window)

    def reset(self) -> None:
        self._buf.clear()

    def update(self, v: float) -> None:
        if math.isfinite(v):
            self._buf.append(float(v))

    @property
    def n(self) -> int:
        return len(self._buf)

    def snapshot(self) -> dict:
        """Tudo de uma vez: a GUI repinta a 10 Hz e não deve varrer a janela
        uma vez por métrica."""
        if not self._buf:
            return {'n': 0, 'mean': None, 'max_abs': None,
                    'min': None, 'max': None, 'rms': None, 'pp': None}
        a = np.fromiter(self._buf, dtype=float, count=len(self._buf))
        return {
            'n': int(a.size),
            'mean': float(a.mean()),
            'max_abs': float(np.abs(a).max()),
            'min': float(a.min()),
            'max': float(a.max()),
            'rms': float(np.sqrt(np.mean(a * a))),
            'pp': float(a.max() - a.min()),
        }
