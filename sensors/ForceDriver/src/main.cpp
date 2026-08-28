#include <Arduino.h>
#include "HX711.h"

// ─────────────────────────────────────────────────────────────────────
// Seeed XIAO ESP32C6 + HX711 — driver da célula axial de 100 kg.
// Firmware SERIAL-ONLY: a única saída é a linha ASCII pela USB.
//
// REESCRITO EM 28/08/2026, e o critério da reescrita foi ESTABILIDADE e não
// taxa. A versão anterior perseguia as 82 Hz do conversor sincronizando pela
// borda de descida do DOUT, e a bancada respondeu com o oposto do prometido:
// medido no mesmo binário, a entrega caiu de 58,8 Hz para 11,8 Hz ao longo de
// dez minutos, com os travamentos do HX711 subindo de 112 para 160 e a placa
// entrando em ciclo de reboot. Ler mais rápido não é ler melhor quando o
// caminho até o dado não sustenta a taxa.
//
// O que ficou de pé destas medidas, e vale registrar porque custou bancada:
//
//   pull-up no DOUT depois do begin()   OBRIGATÓRIO. A bogde/HX711 o desliga
//                                       de propósito em Espressif (HX711.cpp:65,
//                                       nota do ESP8266). Sem ele: σ de 6527
//                                       counts (1,5 N) contra ~200 com ele.
//   guarda de tempo entre leituras      OBRIGATÓRIA. Sem ela o loop relojoa o
//                                       chip antes da conversão terminar e os
//                                       counts viram lixo (σ 21127).
//   DOUT que não sobe após o 25º pulso  é chip TRAVADO, não USB engasgando.
//                                       Só o power-cycle o recupera.
//
// A guarda de 40 ms (24 Hz) é a que sustentou uso prolongado. 20 ms passou em
// 60 s e dessincronizou depois de alguns minutos — o oscilador RC interno do
// HX711 deriva com temperatura, e uma guarda perto do período de conversão
// deixa de ter margem quando o chip esquenta. O que compra robustez é a
// MARGEM sobre o período medido (12135–12154 µs), não a taxa nominal.
// ─────────────────────────────────────────────────────────────────────

#define HX_DOUT_PIN  D1
#define HX_SCK_PIN   D3
#define HX_GAIN      128        // canal A

// Piso de tempo entre leituras. NÃO reduzir sem validar por VÁRIOS MINUTOS:
// 60 s não provam estabilidade, foi essa a lição do 20 ms.
#define HX_PERIOD_US         40000L
// Prazo para o dado ficar pronto depois da guarda. Generoso de propósito:
// cobre até o modo lento do pino RATE (GND = 10 Hz, 100 ms por conversão),
// então trocar o jumper não vira falha.
#define HX_READY_TIMEOUT_MS  200
// Fundo de escala do canal A com ganho 128: ±0,5·AVDD/128 = ±2²⁴/256 counts.
// A célula em repouso lê ~-29000 e a 100 kg nominais ~-45000, então nada
// fisicamente medível passa daqui — o que vier além é bit deslocado, não
// força. Espelha LC_FS_COUNTS do constants.py.
#define HX_FS_COUNTS         65536L

// ── Detecção de célula DESCONECTADA ──────────────────────────────────
// Três defeitos que de fora parecem o mesmo silêncio, e que o firmware
// precisa saber separar porque a ação é diferente em cada um:
//
//   DOUT preso em HIGH      não há HX711 respondendo — placa sem alimentação,
//                           DT solto, ou o chip morreu. Power-cycle não
//                           resolve; é fiação.
//   counts fora de escala   há chip, mas a leitura não é força: ponte de 4
//                           fios solta, ou o chip dessincronizado.
//   counts CONGELADOS       o valor não muda em N leituras seguidas. O HX711
//                           travado devolve o mesmo número indefinidamente, e
//                           esse é o modo que atravessava todos os filtros:
//                           um valor plausível, constante, que o host tratava
//                           como uma célula muito quieta.
#define HX_STUCK_REPEATS     40     // leituras idênticas = congelado
#define HX_BAD_BEFORE_RESET  10     // leituras ruins antes do power-cycle
#define HX_RESET_COOLDOWN_MS 500    // intervalo mínimo entre power-cycles

// Auto-zero de boot: trava o repouso como offset ao ligar (a célula é
// bidirecional). REQUISITO: ligar/re-zerar com a célula EM REPOUSO. O comando
// 'Z' pela USB refaz o zero sob demanda. Nada é transmitido sem zero travado.
#define ZERO_SAMPLES         40
#define ZERO_STABLE_V        0.0002f   // faixa máx durante a coleta (~0,23 N)
#define ZERO_RELAX_MS        3000
// Teto do afrouxamento. 0,002 V ÷ 8,7e-4 V/N = 2,3 N de faixa aceita: um zero
// travado no pior caso carrega ±1,15 N de dispersão. Afrouxar é preferível a
// ficar mudo, mas não é de graça — a faixa ALCANÇADA vai no heartbeat
// (zero_mv=) e o force_receiver avisa quando ela é grande.
#define ZERO_STABLE_MAX_V    0.002f

// Sem leitura válida por este tempo o firmware se considera parado.
#define STALL_MS             5000UL
// Validade do aviso de ENSAIO EM CURSO ('B'). O host reenvia a cada segundo;
// se ele morrer o aviso expira e o reinício volta a ser permitido, que é o
// fail-safe certo — host morto é ensaio nenhum.
#define RUN_FLAG_TTL_MS      3000UL

// v_sensor = counts·AVDD/2²⁴ (tensão da ponte, já ×PGA). MANTER SINCRONIZADO
// com LC_FW_VOLTAGE_SCALE do constants.py. O quadro leva TAMBÉM os counts
// crus (5º campo): o inteiro é a medida, e o volt é uma interpretação dele
// com uma constante que os dois lados do cabo precisam jurar manter igual.
// HX_VREF fica como constante NOMEADA de propósito: o
// test_the_voltage_scale_matches_the_firmware extrai os dois números daqui e
// os confere contra LC_FW_VOLTAGE_SCALE. Inline o 3.3 e o contrato deixa de
// ser verificável — os dois lados do cabo voltam a se acertar por promessa.
static const float HX_VREF     = 3.3f;
static const float COUNTS_TO_V = HX_VREF / 16777216.0f;

static HX711 hx;

// ── Estado ───────────────────────────────────────────────────────────
static uint32_t tx_seq      = 0;
static float    g_v_offset  = 0.0f;
static bool     g_zeroed    = false;
static float    g_zero_acc  = 0.0f, g_zero_min = 0.0f, g_zero_max = 0.0f;
static int      g_zero_cnt  = 0;
static float    g_zero_tol  = ZERO_STABLE_V;
static float    g_zero_span = 0.0f;      // faixa do último zero TRAVADO
static uint32_t g_zero_t0   = 0;

static uint32_t g_resets    = 0;         // power-cycles do HX711
static uint32_t g_ready_to  = 0;         // conversão não terminou
static uint32_t g_bad       = 0;         // leituras fora de escala
static uint32_t g_last_ok   = 0;         // millis da última leitura válida
static uint32_t g_run_flag  = 0;         // millis do último 'B'
static uint32_t g_conv_us   = 0;         // período entre as duas últimas leituras
static bool     g_link_down = false;     // sem HX711 respondendo

static bool em_ensaio()
{
    return g_run_flag != 0 && (millis() - g_run_flag) < RUN_FLAG_TTL_MS;
}

static void zero_restart()
{
    g_zeroed   = false;
    g_zero_cnt = 0;
    g_zero_tol = ZERO_STABLE_V;
    g_zero_t0  = millis();
}

// SCK alto por >60 µs derruba o HX711; a descida o reinicia. hx.begin() NÃO
// faz isso, e um chip que perdeu a sincronia no meio dos 25 pulsos devolve
// zeros para sempre até ser resetado assim.
//
// `assentar_ms`: tempo dado ao chip depois de acordar. 400 ms no boot (pior
// caso, RATE em GND a 10 Hz); em runtime o chip já está aquecido e um ciclo
// de conversão basta — com 400 ms na recuperação, 68 travamentos em 40 s
// comiam 27 s parados.
static void hx_reset(uint32_t assentar_ms)
{
    digitalWrite(HX_SCK_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(HX_SCK_PIN, LOW);
    delay(assentar_ms);
}

static void power_cycle(const char *motivo)
{
    static uint32_t ultimo = 0;
    if (millis() - ultimo < HX_RESET_COOLDOWN_MS) return;
    ultimo = millis();
    g_resets++;
    Serial.printf("# HX711 %s: power-cycle (#%lu)\n",
                  motivo, (unsigned long)g_resets);
    hx_reset(400);
    // NÃO re-zerar: o chip travou, a célula não se moveu, e o offset continua
    // valendo. Descartá-lo obrigava a recolher 40 amostras estáveis a cada
    // travamento — e como sem zero travado nada é transmitido, o stream não
    // nascia.
}

void setup()
{
    // A serial vem PRIMEIRO. Qualquer coisa que trave antes dela deixa a
    // placa muda, e mudo é o pior modo de falha para um driver cujo único
    // canal é a serial.
    Serial.begin(115200);
    // NÃO reduzir o TxTimeout. Com prazo curto o CDC aborta a escrita no meio
    // quando o host engasga, e o abort cai por cima da leitura do HX711: as
    // amostras saem com bits deslocados e o chip dessincroniza.

    pinMode(HX_DOUT_PIN, INPUT_PULLUP);
    pinMode(HX_SCK_PIN, OUTPUT);
    digitalWrite(HX_SCK_PIN, LOW);

    // Durante o reset do ESP os GPIOs flutuam, e um SCK que fique alto derruba
    // o HX711. Esta pausa deixa a alimentação e o chip assentarem antes de
    // qualquer conversa.
    delay(2000);
    hx_reset(400);

    hx.begin(HX_DOUT_PIN, HX_SCK_PIN, HX_GAIN);
    // DE NOVO, e depois do begin(): a bogde/HX711 faz pinMode(DOUT, INPUT) lá
    // dentro e APAGA o pull-up. Medido em 28/08/2026 — sem esta linha o σ vai
    // de ~200 para 6527 counts (1,5 N). A nota da lib que justifica desligá-lo
    // é sobre o ESP8266 e não vale para esta placa. NÃO reordenar.
    pinMode(HX_DOUT_PIN, INPUT_PULLUP);

    g_last_ok = millis();
    zero_restart();
    Serial.println("# ForceDriver pronto (XIAO ESP32C6 + HX711)");
}

// Uma leitura completa, ou LONG_MIN se não deu. Concentra aqui TODO o
// diálogo com o chip, para o loop() ficar só com a política.
static long le_counts()
{
    // Guarda de tempo: o piso entre leituras. Relojoar o HX711 no meio de uma
    // conversão o dessincroniza de forma permanente.
    static uint32_t ultima_us = 0;
    while ((uint32_t)(micros() - ultima_us) < HX_PERIOD_US)
        delay(1);                     // cede a CPU; não relojoa nada

    // NÃO existe espera pelo DOUT SUBIR aqui, e a ausência é deliberada.
    // Ela faz sentido num firmware sem guarda de tempo, onde é ela que
    // garante que o dado é de uma conversão NOVA. Com a guarda, o dado já é
    // novo por construção — passaram-se 40 ms sobre um período de 12,1 ms —
    // e nesse ponto o DOUT já desceu há ~28 ms. Esperar uma SUBIDA que já
    // aconteceu é esperar para sempre: medido em 28/08/2026, 113 falsos
    // "DOUT preso em LOW" e 12 power-cycles em 40 s, com ZERO amostras
    // transmitidas. Quem denuncia chip travado aqui é `recusa()`, que olha
    // para o dado.
    if (!hx.wait_ready_timeout(HX_READY_TIMEOUT_MS)) {
        g_ready_to++;
        // DOUT preso em HIGH: não há conversão terminando. Se persistir, não
        // é o chip travado — é não haver chip. Power-cycle não resolve fiação,
        // e é por isso que este caminho AVISA em vez de martelar o reset.
        if (!g_link_down && g_ready_to % 10 == 1)
            Serial.println("# HX711 sem resposta (DOUT preso em HIGH): "
                           "DT solto, placa sem alimentacao ou chip morto. "
                           "Isto e fiacao, nao trava — o power-cycle nao "
                           "resolve.");
        return LONG_MIN;
    }
    long c = hx.read();
    uint32_t agora = micros();
    if (ultima_us) g_conv_us = (uint32_t)(agora - ultima_us);
    ultima_us = agora;
    return c;
}

// Valida a leitura. Devolve o motivo da recusa, ou nullptr se ela presta.
static const char *recusa(long c)
{
    if (c == 0 || c == 1 || c == -1) return "travado (0/saturado)";
    if (c > HX_FS_COUNTS || c < -HX_FS_COUNTS) return "fora de escala";
    // CONGELADO: o mesmo valor repetido é o modo de falha que atravessava
    // todos os outros filtros — um número plausível e constante, que o host
    // lia como uma célula muito quieta. Ruído de ~200 counts torna a
    // repetição EXATA impossível num sensor vivo.
    static long ultimo = LONG_MIN;
    static int  iguais = 0;
    iguais = (c == ultimo) ? iguais + 1 : 0;
    ultimo = c;
    if (iguais >= HX_STUCK_REPEATS) { iguais = 0; return "congelado (mesmo valor)"; }
    return nullptr;
}

// Acumula o zero de boot. Devolve true quando ele TRAVOU.
static bool acumula_zero(float v)
{
    // Afrouxa a tolerância se o repouso não vier — nunca ficar mudo.
    if (g_zero_tol < ZERO_STABLE_MAX_V && millis() - g_zero_t0 >= ZERO_RELAX_MS) {
        g_zero_tol = min(g_zero_tol * 2.0f, ZERO_STABLE_MAX_V);
        g_zero_t0  = millis();
        Serial.printf("# zero sem travar: tolerancia p/ %.3f mV "
                      "(bancada vibrando? celula carregada?)\n",
                      g_zero_tol * 1e3f);
    }
    if (g_zero_cnt == 0) { g_zero_acc = 0.0f; g_zero_min = g_zero_max = v; }
    g_zero_acc += v;
    if (v < g_zero_min) g_zero_min = v;
    if (v > g_zero_max) g_zero_max = v;
    g_zero_cnt++;

    if (g_zero_max - g_zero_min > g_zero_tol) {
        g_zero_cnt = 0;                 // derivando — recomeça a coleta
        return false;
    }
    if (g_zero_cnt < ZERO_SAMPLES) return false;
    g_v_offset  = g_zero_acc / (float)g_zero_cnt;
    g_zero_span = g_zero_max - g_zero_min;
    g_zeroed    = true;
    Serial.printf("# zero travado: offset=%.6f V (faixa %.3f mV, tol %.3f mV)\n",
                  g_v_offset, g_zero_span * 1e3f, g_zero_tol * 1e3f);
    return true;
}

void loop()
{
    // ── Heartbeat (0,5 Hz), em chave=valor ───────────────────────────
    // O host PARSEIA estes campos (LcLineParser.heartbeat) e é por eles que
    // ele descobre POR QUE não há amostra: zeroed=0 é bancada vibrando, e não
    // cabo solto. Sem este canal o diagnóstico ficava preso no log da placa.
    static uint32_t hb_ms = 0, hb_seq = 0;
    uint32_t agora_ms = millis();
    if (agora_ms - hb_ms >= 2000) {
        uint32_t dt = agora_ms - hb_ms;
        float taxa = (dt && hb_ms) ? (tx_seq - hb_seq) * 1000.0f / (float)dt : 0.0f;
        hb_ms = agora_ms; hb_seq = tx_seq;
        Serial.printf("# amostras=%lu taxa=%.1f offset=%.6f zeroed=%d "
                      "zero_mv=%.3f resets=%lu ready_to=%lu bad=%lu "
                      "conv_us=%lu link=%d ensaio=%d\n",
                      (unsigned long)tx_seq, taxa, g_v_offset, (int)g_zeroed,
                      g_zero_span * 1e3f, (unsigned long)g_resets,
                      (unsigned long)g_ready_to, (unsigned long)g_bad,
                      (unsigned long)g_conv_us,
                      (int)!g_link_down, (int)em_ensaio());
    }

    // ── Comandos do host ─────────────────────────────────────────────
    //   'Z'  refaz o zero de boot (/load_cell/rezero)
    //   'B'  ensaio em curso — inibe o reinício automático
    while (Serial.available() > 0) {
        int c = Serial.read();
        if      (c == 'Z') zero_restart();
        else if (c == 'B') g_run_flag = millis();
    }

    // ── Watchdog de software, CONDICIONAL ────────────────────────────
    // Não há esp_task_wdt armado: o WDT de hardware não sabe distinguir
    // bancada parada de ensaio em curso, e reiniciar no meio de uma palpação
    // joga o ensaio fora sem salvar nada. Durante um ensaio isto só avisa —
    // o explorer já aborta sozinho por leitura velha e volta à HOME.
    if ((uint32_t)(millis() - g_last_ok) > STALL_MS) {
        if (em_ensaio()) {
            Serial.println("# PARADO ha >5 s durante ENSAIO: nao vou "
                           "reiniciar. O explorer aborta por leitura velha.");
            g_last_ok = millis();          // não repetir a cada volta
        } else {
            Serial.println("# PARADO ha >5 s e fora de ensaio: reiniciando.");
            Serial.flush();
            delay(50);
            ESP.restart();
        }
    }

    long counts = le_counts();
    if (counts == LONG_MIN) {
        // Sem leitura. O link cai depois de muitas tentativas seguidas — é o
        // que transforma "não chega amostra" em "a célula está desconectada".
        if (g_ready_to > 20 && !g_link_down) {
            g_link_down = true;
            Serial.println("# CELULA DESCONECTADA: sem resposta do HX711.");
        }
        return;
    }

    const char *motivo = recusa(counts);
    if (motivo) {
        if (++g_bad % HX_BAD_BEFORE_RESET == 0) power_cycle(motivo);
        return;                              // lixo não entra no stream
    }

    if (g_link_down) {
        g_link_down = false;
        g_ready_to  = 0;
        Serial.println("# celula de volta.");
    }
    g_last_ok = millis();

    float v = (float)counts * COUNTS_TO_V;
    if (!g_zeroed) { acumula_zero(v); return; }  // sem zero, nada é transmitido

    // Formato (espelhado em touch_pack/lc_serial.py):
    //   F,<seq>,<t_us>,<v_sensor>,<counts>\n
    Serial.printf("F,%lu,%lu,%.7f,%ld\n",
                  (unsigned long)tx_seq++, (unsigned long)micros(),
                  (double)(v - g_v_offset), counts);
}
