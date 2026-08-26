#include <Arduino.h>
#include "HX711.h"

// Seeed XIAO ESP32S3 + HX711 (célula de carga).
// SERIAL_TEST=1: modo de diagnóstico de fiação (counts crus, sem zero de boot).
#ifndef SERIAL_TEST
#define SERIAL_TEST 0
#endif

// DT → D1 (GPIO2), SCK → D3 (GPIO4)
#define HX_DOUT_PIN  6
#define HX_SCK_PIN   7
#define HX_GAIN      128     // canal A

static HX711 hx;

// v_sensor = counts·AVDD/2²⁴ (tensão da ponte já ×PGA). MANTER SINCRONIZADO
// com LC_FW_VOLTAGE_SCALE / LC_FW_VOLTAGE_OFFSET do constants.py.
const float HX_VREF     = 3.3f;
const float COUNTS_TO_V = HX_VREF / 16777216.0f;

// Auto-zero: trava o repouso como offset ao ligar (célula é bidirecional).
// REQUISITO: ligar/re-zerar com a célula EM REPOUSO. Comando 'Z' via USB
// refaz o zero sob demanda. Nenhuma amostra é transmitida sem zero travado.
#define ZERO_SAMPLES   40       // ~0,5 s @ 80 Hz
#define ZERO_STABLE_V  0.0002f  // faixa máx aceita durante a coleta (~0,23 N)
// Se o repouso não vier, a tolerância dobra a cada ZERO_RELAX_MS (até
// ZERO_STABLE_MAX_V) para o zero nunca ficar travado indefinidamente.
#define ZERO_RELAX_MS      3000
#define ZERO_STABLE_MAX_V  0.002f
static float g_v_offset = 0.0f;
static bool  g_zeroed   = false;
static float g_zero_acc = 0.0f, g_zero_min = 0.0f, g_zero_max = 0.0f;
static int   g_zero_cnt = 0;
static float g_zero_tol = ZERO_STABLE_V;
static uint32_t g_zero_t0_ms = 0;

static void zero_restart()
{
    g_zeroed     = false;
    g_zero_cnt   = 0;
    g_zero_tol   = ZERO_STABLE_V;
    g_zero_t0_ms = millis();
}

// Formato na serial: "F,<seq>,<t_us>,<v_sensor>\n" — espelhado em
// touch_pack/lc_serial.py. Taxa ditada pelo pino RATE do HX711 (GND=10Hz, VDD=80Hz).
static uint32_t tx_seq = 0;

void setup()
{
    hx.begin(HX_DOUT_PIN, HX_SCK_PIN, HX_GAIN);
    // Pull-up: sem HX711 o pino flutua em LOW ("amostra pronta"), gerando zeros falsos.
    pinMode(HX_DOUT_PIN, INPUT_PULLUP);
    Serial.begin(115200);
    // TxTimeout curto: nunca segura o loop além de um período de amostragem
    // por causa de um engasgo do host USB.
    Serial.setTxTimeoutMs(2);
#if SERIAL_TEST
    // LED (GPIO21, aceso em LOW) alterna a cada amostra; 1 Hz = sem amostra.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);               // tempo do monitor USB CDC enumerar/abrir
    Serial.println("[serial_test] HX711 DT=GPIO2(D1) SCK=GPIO4(D3)");
    Serial.println("[serial_test] esperando amostras (DOUT precisa pulsar)...");
#else
    zero_restart();
#endif
}

void loop()
{
#if !SERIAL_TEST
    // Heartbeat de diagnóstico (0,5 Hz). Linhas '#' são ignoradas pelo lc_serial.
    static uint32_t last_hb_ms  = 0;
    static uint32_t last_hb_seq = 0;
    uint32_t hb_ms = millis();
    if (hb_ms - last_hb_ms >= 2000) {
        uint32_t dt_ms = hb_ms - last_hb_ms;
        float rate = (dt_ms && last_hb_ms)
                     ? (tx_seq - last_hb_seq) * 1000.0f / (float)dt_ms : 0.0f;
        last_hb_ms  = hb_ms;
        last_hb_seq = tx_seq;
        Serial.printf("# amostras=%lu taxa=%.1fHz offset=%.6f zeroed=%d\n",
                      (unsigned long)tx_seq, rate, g_v_offset, (int)g_zeroed);
    }
#else
    // Heartbeat 1 Hz sem amostra: DOUT preso em HIGH = HX711 ausente/DT
    // errado; preso em LOW = SCK errado ou DT em curto.
    static uint32_t last_beat_ms = 0;
    static uint32_t last_sample_seq = 0;
    uint32_t beat_ms = millis();
    if (beat_ms - last_beat_ms >= 1000) {
        last_beat_ms = beat_ms;
        if (tx_seq == last_sample_seq) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            Serial.printf("[serial_test] sem amostra ha 1 s; DOUT=%s\n",
                          digitalRead(HX_DOUT_PIN) ? "HIGH (HX711 ausente/DT errado?)"
                                                   : "LOW (SCK errado/DT em curto?)");
        }
        last_sample_seq = tx_seq;
    }
#endif

    // Handshake real do HX711: só lê com DOUT baixo APÓS tê-lo visto alto
    // desde a última leitura — um DOUT preso em LOW (curto/fiação errada)
    // trava aqui em vez de virar stream de zeros. read() só relojoa os
    // 24 bits (~60 µs); nunca espera conversão. No SERIAL_TEST o handshake
    // é dispensado de propósito (queremos ler continuamente p/ diagnóstico).
#if SERIAL_TEST
    if (!hx.is_ready()) return;
#else
    static bool dout_seen_high = false;
    if (!hx.is_ready()) { dout_seen_high = true; return; }
    if (!dout_seen_high) return;
    dout_seen_high = false;
#endif
    long counts = hx.read();
    uint32_t now_us = micros();

    float v_raw = (float)counts * COUNTS_TO_V;

#if !SERIAL_TEST
    // Comando de re-zero pela USB ('Z'): a GUI (via force_receiver) pede um
    // novo zero com a célula em repouso. Lido aqui, no caminho da amostra.
    while (Serial.available() > 0) {
        if (Serial.read() == 'Z') zero_restart();
    }

    if (!g_zeroed) {
        if (g_zero_t0_ms == 0) g_zero_t0_ms = millis();
        // Afrouxa a tolerância se o repouso não vier — nunca ficar mudo.
        if (g_zero_tol < ZERO_STABLE_MAX_V &&
            millis() - g_zero_t0_ms >= ZERO_RELAX_MS) {
            g_zero_tol   = min(g_zero_tol * 2.0f, ZERO_STABLE_MAX_V);
            g_zero_t0_ms = millis();
            Serial.printf("# zero sem travar: tolerancia afrouxada p/ %.3f mV "
                          "(bancada vibrando? celula carregada?)\n",
                          g_zero_tol * 1e3f);
        }
        if (g_zero_cnt == 0) {
            g_zero_acc = 0.0f;
            g_zero_min = g_zero_max = v_raw;
        }
        g_zero_acc += v_raw;
        if (v_raw < g_zero_min) g_zero_min = v_raw;
        if (v_raw > g_zero_max) g_zero_max = v_raw;
        g_zero_cnt++;
        if (g_zero_max - g_zero_min > g_zero_tol) {
            g_zero_cnt = 0;            // sinal derivando — recomeça a coleta
        } else if (g_zero_cnt >= ZERO_SAMPLES) {
            g_v_offset = g_zero_acc / (float)g_zero_cnt;
            g_zeroed   = true;
            Serial.printf("# zero travado: offset=%.6f V (faixa %.3f mV, "
                          "tolerancia %.3f mV)\n",
                          g_v_offset, (g_zero_max - g_zero_min) * 1e3f,
                          g_zero_tol * 1e3f);
        }
        return;   // sem zero travado, nenhuma amostra é transmitida
    }
#endif

    float v_sensor = v_raw - g_v_offset;
    uint32_t seq = tx_seq++;

#if SERIAL_TEST
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // counts crus diagnosticam a fiação: -1 = DT com mau contato/sem GND
    // comum; 0 = SCK não chega; ±8388607 = saturado (A+/A− trocados ou
    // ponte solta); variando com o toque = OK. Prints a ~20 linhas/s.
    static uint32_t last_print_ms   = 0;
    static uint32_t win_samples     = 0;
    win_samples++;
    uint32_t print_ms = millis();
    uint32_t dt_ms = print_ms - last_print_ms;
    if (dt_ms >= 50) {
        Serial.printf("seq=%lu  counts=%ld  v_sensor=%.6f V  (~%.0f amostras/s)\n",
                      (unsigned long)seq, counts, v_sensor,
                      win_samples * 1000.0f / (float)dt_ms);
        last_print_ms = print_ms;
        win_samples   = 0;
    }
#else
    Serial.printf("F,%lu,%lu,%.7f\n",
                  (unsigned long)seq, (unsigned long)now_us,
                  (double)v_sensor);
#endif
}
