"""code.py — ForceDriver para Seeed XIAO RP2040 (CircuitPython).

Porte 1:1 do firmware Arduino em sensors/ForceDriver/src/main.cpp, que rodava
no XIAO ESP32S3. O protocolo na USB é IDÊNTICO, então nada muda do lado do
host (lc_serial.py / force_receiver_node.py):

    F,<seq>,<t_us>,<v_sensor>\\n     amostra
    # ...                            heartbeat/diagnóstico (host ignora)

`t_us` é uint32 COM WRAPAROUND de propósito: o force_receiver_node calcula o
dt com `(t_us - last) & 0xFFFFFFFF` e um contador que crescesse sem limite
quebraria essa conta.

FIAÇÃO (XIAO RP2040):
    HX711 DT  → D1 = GP27
    HX711 SCK → D3 = GP29
    HX711 VCC → 3V3        GND → GND
São os MESMOS pinos de silk (D1/D3) da montagem do ESP32S3; só o número do
GPIO mudou (era GPIO2/GPIO4 lá).

INSTALAÇÃO: copie este arquivo como `code.py` na raiz do drive CIRCUITPY.
O projeto de joystick/mouse HID que estava aqui não tem relação com a célula
e foi substituído (cópia preservada em code_joystick_antigo.py.bak).
"""

import gc
import sys
import time

import board
import digitalio
import supervisor

# ── Configuração ──────────────────────────────────────────────────────────
# SERIAL_TEST=True: modo de diagnóstico de fiação (counts crus, sem zero de
# boot). Equivale ao env:serial_test do platformio.ini antigo.
SERIAL_TEST = False

HX_DOUT_NAMES = ("GP27",)          # D1
HX_SCK_NAMES = ("GP29", "A3")      # D3 (o build raspberry_pi_pico pode expor
                                   # o GP29 só como A3 — ver _resolve_pin)

# v_sensor = counts·AVDD/2²⁴ (tensão da ponte já ×PGA). MANTER SINCRONIZADO
# com LC_FW_VOLTAGE_SCALE / LC_FW_VOLTAGE_OFFSET do constants.py.
HX_VREF = 3.3
COUNTS_TO_V = HX_VREF / 16777216.0

# Auto-zero: trava o repouso como offset ao ligar (célula é bidirecional).
# REQUISITO: ligar/re-zerar com a célula EM REPOUSO. Comando 'Z' via USB
# refaz o zero sob demanda. Nenhuma amostra é transmitida sem zero travado.
ZERO_SAMPLES = 40           # ~4 s @ 10 Hz (~0,5 s @ 80 Hz)
ZERO_STABLE_V = 0.0002      # faixa máx aceita durante a coleta (~0,23 N)
# Se o repouso não vier, a tolerância dobra a cada ZERO_RELAX_MS (até
# ZERO_STABLE_MAX_V) para o zero nunca ficar travado indefinidamente.
ZERO_RELAX_MS = 3000
ZERO_STABLE_MAX_V = 0.002

HEARTBEAT_MS = 2000

# Recuperação do handshake do DOUT (ver o bloco "Handshake real" no loop).
# A 80 Hz a janela em que o DOUT fica alto é de ~11 ms; perdê-la uma vez
# travaria o loop para sempre. Passado este tempo com o DOUT em baixo, lê
# assim mesmo — 4 períodos de conversão, longe de um atraso normal.
STUCK_LOW_RECOVER_MS = 50


def _resolve_pin(names):
    """Primeiro nome existente em `board`. O drive veio com o build
    `raspberry_pi_pico` (ver boot_out.txt), não o do XIAO, e nesse build o
    GP29 às vezes só aparece como A3 — daí a lista de alternativas."""
    for n in names:
        pin = getattr(board, n, None)
        if pin is not None:
            return pin, n
    disponiveis = " ".join(
        sorted(d for d in dir(board) if d.startswith(("GP", "A"))))
    raise RuntimeError(
        "nenhum de %s existe neste build de CircuitPython. Pinos: %s"
        % (" / ".join(names), disponiveis))


class HX711:
    """Bit-bang do HX711 em digitalio.

    CUIDADO DE TIMING: o HX711 entra em power-down se o SCK ficar ALTO por
    mais de 60 µs. Em CircuitPython o risco não é a velocidade do toggle (uns
    poucos µs) e sim o coletor de lixo disparar no meio do pulso. Por isso
    (a) `gc.collect()` roda ANTES da leitura, com a janela de 100 ms entre
    amostras sobrando, e (b) o SCK volta a LOW antes de qualquer alocação —
    o deslocamento de bits acontece com o clock já baixo.
    """

    def __init__(self, dout_pin, sck_pin):
        self._dout = digitalio.DigitalInOut(dout_pin)
        self._dout.direction = digitalio.Direction.INPUT
        # Pull-up: sem HX711 o pino flutua em LOW ("amostra pronta"), gerando
        # zeros falsos. Mesma proteção do pinMode(INPUT_PULLUP) do main.cpp.
        self._dout.pull = digitalio.Pull.UP
        self._sck = digitalio.DigitalInOut(sck_pin)
        self._sck.direction = digitalio.Direction.OUTPUT
        self._sck.value = False

    def is_ready(self):
        """DOUT em LOW = conversão pronta (mesma semântica do bogde/HX711)."""
        return not self._dout.value

    def read(self):
        """Relojoa os 24 bits + 1 pulso de ganho (128, canal A). Não espera
        conversão — o chamador já confirmou is_ready()."""
        gc.collect()
        sck = self._sck
        dout = self._dout
        v = 0
        for _ in range(24):
            sck.value = True
            bit = dout.value          # amostra com o clock alto
            sck.value = False         # desce ANTES de alocar
            v = (v << 1) | bit
        # 1 pulso extra = ganho 128 / canal A na próxima conversão.
        sck.value = True
        sck.value = False
        # 24 bits em complemento de dois.
        if v & 0x800000:
            v -= 0x1000000
        return v


def _now_ms():
    return time.monotonic_ns() // 1_000_000


def _now_us_u32():
    """Espelha o micros() do Arduino: uint32 que dá a volta. O host depende
    disso (force_receiver_node.py: `(t_us - last) & 0xFFFFFFFF`)."""
    return (time.monotonic_ns() // 1000) & 0xFFFFFFFF


def main():
    # Auto-reload reinicia o board a cada escrita no drive — no meio de uma
    # coleta isso apareceria como reset do MCU (seq reancorado) no receiver.
    supervisor.runtime.autoreload = False

    dout_pin, dout_name = _resolve_pin(HX_DOUT_NAMES)
    sck_pin, sck_name = _resolve_pin(HX_SCK_NAMES)
    hx = HX711(dout_pin, sck_pin)

    print("# ForceDriver RP2040 | DT=%s SCK=%s | %s"
          % (dout_name, sck_name, "SERIAL_TEST" if SERIAL_TEST else "normal"))

    tx_seq = 0
    v_offset = 0.0
    zeroed = False
    zero_acc = 0.0
    zero_min = 0.0
    zero_max = 0.0
    zero_cnt = 0
    zero_tol = ZERO_STABLE_V
    zero_t0_ms = _now_ms()

    last_hb_ms = _now_ms()
    last_hb_seq = 0
    last_counts = 0
    dout_seen_high = False
    stuck_low_t0 = 0        # 0 = não estamos esperando recuperação
    recoveries = 0

    while True:
        # ── Heartbeat de diagnóstico (0,5 Hz) ─────────────────────────────
        hb_ms = _now_ms()
        if hb_ms - last_hb_ms >= HEARTBEAT_MS:
            dt_ms = hb_ms - last_hb_ms
            rate = (tx_seq - last_hb_seq) * 1000.0 / dt_ms if dt_ms else 0.0
            last_hb_ms = hb_ms
            last_hb_seq = tx_seq
            # `counts` entra no heartbeat porque é o que diagnostica a fiação
            # sem recompilar em SERIAL_TEST: 0 imóvel = ponte sem excitação
            # (AVDD do HX711 não partiu) ou SCK não chega; -1 = DT sem
            # contato; ±8388607 = saturado (A+/A− trocados ou ponte solta).
            # `recup` > 0 = o loop está perdendo a janela do DOUT (ver o
            # handshake). Ainda funciona, mas é o aviso de que a folga de
            # ~11 ms dos 80 Hz acabou.
            print("# amostras=%d taxa=%.1fHz offset=%.6f zeroed=%d "
                  "counts=%d recup=%d"
                  % (tx_seq, rate, v_offset, 1 if zeroed else 0,
                     last_counts, recoveries))

        # ── Comando de re-zero pela USB ('Z') ─────────────────────────────
        # Non-blocking: sys.stdin.read() sozinho bloquearia. A GUI (via
        # force_receiver) pede um novo zero com a célula em repouso.
        while supervisor.runtime.serial_bytes_available:
            if sys.stdin.read(1) == "Z":
                zeroed = False
                zero_cnt = 0
                zero_tol = ZERO_STABLE_V
                zero_t0_ms = _now_ms()

        # ── Handshake real do HX711 ───────────────────────────────────────
        # Só lê com DOUT baixo APÓS tê-lo visto alto desde a última leitura —
        # um DOUT preso em LOW (curto/fiação errada) daria stream de zeros, e
        # zero constante num controle por força é pior que silêncio: parece
        # "sem contato" e o robô continua descendo.
        #
        # A janela em que o DOUT fica alto é o intervalo entre a leitura e a
        # próxima conversão: ~90 ms a 10 Hz, mas só ~11 ms a 80 Hz. Se ela for
        # perdida (gc, engasgo do USB), o DOUT já está baixo na volta e NÃO
        # sobe mais sozinho — só a leitura o libera. Sem a recuperação abaixo
        # o `continue` rodaria indefinidamente: mudo até resetar a placa.
        # Por isso, após STUCK_LOW_RECOVER_MS lemos assim mesmo; a proteção
        # original vira o teste dos counts logo abaixo, que é o que de fato
        # distingue um curto (0 / -1 imóveis) de uma janela perdida.
        # No SERIAL_TEST o handshake é dispensado de propósito.
        if SERIAL_TEST:
            if not hx.is_ready():
                continue
        else:
            if not hx.is_ready():
                dout_seen_high = True
                stuck_low_t0 = 0
                continue
            if not dout_seen_high:
                now_ms = _now_ms()
                if stuck_low_t0 == 0:
                    stuck_low_t0 = now_ms
                    continue
                if now_ms - stuck_low_t0 < STUCK_LOW_RECOVER_MS:
                    continue
                recoveries += 1
            stuck_low_t0 = 0
            dout_seen_high = False

        counts = hx.read()
        last_counts = counts

        # Assinaturas de fiação quebrada: DT sem contato (-1) ou SCK/excitação
        # ausentes (0), ambos imóveis. Não transmitir é proposital — ver o
        # comentário do handshake. O heartbeat continua reportando counts.
        if not SERIAL_TEST and counts in (0, -1):
            dout_seen_high = False
            continue
        now_us = _now_us_u32()
        v_raw = counts * COUNTS_TO_V

        if SERIAL_TEST:
            seq = tx_seq
            tx_seq += 1
            print("seq=%d  counts=%d  v_sensor=%.6f V" % (seq, counts, v_raw))
            continue

        # ── Auto-zero ─────────────────────────────────────────────────────
        if not zeroed:
            # Afrouxa a tolerância se o repouso não vier — nunca ficar mudo.
            if (zero_tol < ZERO_STABLE_MAX_V
                    and _now_ms() - zero_t0_ms >= ZERO_RELAX_MS):
                zero_tol = min(zero_tol * 2.0, ZERO_STABLE_MAX_V)
                zero_t0_ms = _now_ms()
                print("# zero sem travar: tolerancia afrouxada p/ %.3f mV "
                      "(bancada vibrando? celula carregada?)"
                      % (zero_tol * 1e3))
            if zero_cnt == 0:
                zero_acc = 0.0
                zero_min = v_raw
                zero_max = v_raw
            zero_acc += v_raw
            if v_raw < zero_min:
                zero_min = v_raw
            if v_raw > zero_max:
                zero_max = v_raw
            zero_cnt += 1
            if zero_max - zero_min > zero_tol:
                zero_cnt = 0          # sinal derivando — recomeça a coleta
            elif zero_cnt >= ZERO_SAMPLES:
                v_offset = zero_acc / zero_cnt
                zeroed = True
                print("# zero travado: offset=%.6f V (faixa %.3f mV, "
                      "tolerancia %.3f mV)"
                      % (v_offset, (zero_max - zero_min) * 1e3, zero_tol * 1e3))
            continue   # sem zero travado, nenhuma amostra é transmitida

        v_sensor = v_raw - v_offset
        seq = tx_seq
        tx_seq = (tx_seq + 1) & 0xFFFFFFFF
        print("F,%d,%d,%.7f" % (seq, now_us, v_sensor))


main()
