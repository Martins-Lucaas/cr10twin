#include "stm32f7xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

// DEFINIÇÕES
#define DEBUG_ADC_PORT GPIOB
#define DEBUG_ADC_PIN  GPIO_PIN_8 //(GPIO DE PULSO PRA ADC)
#define DEBUG_IZH_PORT GPIOB
#define DEBUG_IZH_PIN  GPIO_PIN_9//(GPIO DE PULSO PRA Izhikevich)

//////////////////////////////////////////////
#define ROWS 5
#define COLS 5
#define NUM_TAXELS (ROWS*COLS)
#define DIFF_BUFFER 25
static uint8_t row_mask = 0x1E;

// IZH PARAMETERS

/* Adap Rapida */
#define A_RA 0.1f
#define B_RA 0.2f
#define C_RA -65.0f
#define D_RA 2.0f
#define G_RA 1000.0f

/* Adap lenta */
#define A_SA 0.02f
#define B_SA 0.2f
#define C_SA -65.0f
#define D_SA 8.0f
#define G_SA 40.0f

//* Cuneate Neuron MM
#define A_CN 0.02f
#define B_CN 0.2f
#define C_CN -65.0f
#define D_CN 8.0f
#define G_CN 100000.0f

//* Cuneate Neuron FA
#define A_CNF 0.02f
#define B_CNF 0.2f
#define C_CNF -65.0f
#define D_CNF 8.0f
#define G_CNF 100000.0f

//* Cuneate Neuron SA
#define A_CNS 0.02f
#define B_CNS 0.2f
#define C_CNS -65.0f
#define D_CNS 8.0f
#define G_CNS 100000.0f

//* Cuneate Neuron - NEURONIO DO ARTIGO DA ANA
//#define A_CN 0.211f
//#define B_CN 0.228f
//#define C_CN -27.668f
//#define D_CN 31.761f
//#define G_CN 6.310e+09f

#define DT 0.10f

#define VTH 30.0f
#define V_MIN 0.0f
#define V_MAX 3.2f

// ── USB ───────────────────────────────────────────────────────────────
// USB_PACKET_SIZE é o wMaxPacketSize do endpoint bulk em Full Speed: 64 B.
// Ele NÃO é o teto de uma transferência — CDC_Transmit_FS aceita centenas de
// bytes e a própria pilha os fatia em pacotes de 64. Usar 64 como tamanho de
// transferência limitava a vazão a ~1 transferência por polling do host,
// muito abaixo dos ~143 KB/s que os frames ADC produzem a 1 kHz. [4]
#define USB_PACKET_SIZE     64
#define USB_TX_CHUNK        512     // bytes por transferência (múltiplo de 64)
#define USB_TX_BUFFER_SIZE  8192    // ring; >= 2x o chunk, potência de 2

// Linha de diagnóstico "STAT,drop=<n>,t=<us>". Período em ms; 0 desliga. [5]
#define USB_STAT_PERIOD_MS  1000

#define TS_64BIT 0

//#define SEND_INTERVAL_MS 100  // envio ADC a cada 10 ms

// Para neuronio de segunda ordem
#define TAU_SYN 4.0f
#define G_MAX 1.0f

#define W_INB 0.0f /////0.19450982f // peso inibitorio, é o mesmo para todos
/////////////

#define TEMPLATE_SIZE 20


#define HX711_DOUT_PORT GPIOC
#define HX711_DOUT_PIN  GPIO_PIN_6

#define HX711_SCK_PORT  GPIOC
#define HX711_SCK_PIN   GPIO_PIN_7

#define HX711_SAMPLE_FREQ 80.0f
#define HX711_CUTOFF_FREQ 30.0f

// Datasheet do HX711: PD_SCK em nível alto tem mínimo de 0,2 us (T2) e
// MÁXIMO de 60 us — acima disso o chip entra em power-down. Dois
// HAL_GPIO_WritePin consecutivos a 168 MHz davam ~40 ns, abaixo do mínimo.
// 60 ciclos a 168 MHz = 0,36 us, com folga sobre os 0,2 us. [6]
#define HX711_SCK_DELAY_CYC 60

// Teto da espera por DOUT em nível baixo, em MILISSEGUNDOS de relógio real. A
// 80 SPS uma amostra nova chega a cada 12,5 ms; 100 ms cobre isso com folga e
// devolve o controle rápido quando o chip não responde.
#define HX711_READY_TIMEOUT_MS 100

// Tara: amostras somadas e janela máxima para consegui-las. Se o HX711 não
// entregar as amostras dentro da janela, o firmware desiste, segue com o
// offset que tiver e AVISA — nunca fica preso esperando a célula.
#define HX711_TARE_SAMPLES   200
#define HX711_TARE_WINDOW_MS 8000
// Tempo após o boot antes de começar a tara, para o HX711 estabilizar.
#define HX711_TARE_DELAY_MS  1000

#define HX711_CALIBRATION_FACTOR 260.6f

#define GRAMS_TO_NEWTONS 0.00980665f


float g_template[TEMPLATE_SIZE];

float gain_RA[NUM_TAXELS];
float gain_SA[NUM_TAXELS];

float gain_SA_init[NUM_TAXELS] = {

        1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0

    };

float gain_RA_init[NUM_TAXELS] = {

        1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0
    };


float I_inh = 0.0f;

//  ESTRUTURAS
typedef struct {
    float v_RA;
    float u_RA;

    float v_SA;
    float u_SA;

    float I;
} Taxel;

typedef struct {
      float v_CN;
      float u_CN;
} CuneateNeuron;

typedef struct {
      float v_CNF;
      float u_CNF;
} CuneateNeuronFast;

typedef struct {
      float v_CNS;
      float u_CNS;
} CuneateNeuronSlow;


// HANDLES
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim2;
extern USBD_HandleTypeDef hUsbDeviceFS;

// Onde a pilha guarda o contexto da classe CDC. Nas versões mais novas da
// USB Device Library isto virou `pClassDataCmsit[0]`; se o projeto usar uma
// dessas, redefina este macro no build em vez de editar o código abaixo.
#ifndef USBD_CDC_CTX
#define USBD_CDC_CTX (hUsbDeviceFS.pClassData)
#endif

// VARIÁVEIS
Taxel taxels[NUM_TAXELS];
uint16_t adc_buffer[COLS];
uint8_t current_row = 0;

volatile uint8_t spike_flags_RA[NUM_TAXELS] = {0};
volatile uint8_t spike_flags_SA[NUM_TAXELS] = {0};

volatile uint16_t last_adc[NUM_TAXELS] = {0};

float I_buffer[NUM_TAXELS][DIFF_BUFFER] = {0}; // corrente excitatória
uint8_t I_index[NUM_TAXELS] = {0};

float I_inh_buffer[NUM_TAXELS][DIFF_BUFFER] = {0}; // corrente inibitória
uint8_t I_inh_index[NUM_TAXELS] = {0};

uint8_t usb_tx_buffer[USB_TX_BUFFER_SIZE];
volatile uint16_t usb_head = 0;
volatile uint16_t usb_tail = 0;
volatile uint32_t usb_dropped = 0;   // linhas descartadas por buffer cheio [5]

float g_syn_RA[NUM_TAXELS] = {0};
float g_syn_SA[NUM_TAXELS] = {0};

uint8_t template_idx_RA[NUM_TAXELS] = {0};
uint8_t template_idx_SA[NUM_TAXELS] = {0}; // em qual posição do template cada neurônio está

uint8_t template_idx_INH_RA[NUM_TAXELS] = {0};
uint8_t template_idx_INH_SA[NUM_TAXELS] = {0};

bool izhikevich_step(float *v, float *u, float I,
                     float a, float b, float c, float d);
// =====================================================
// VARIÁVEIS HX711
// =====================================================

volatile int32_t hx711_offset = 0;

volatile float force_raw = 0.0f;
volatile float force_filtered = 0.0f;

float hx711_alpha = 0.0f;

volatile uint8_t hx711_ready = 0;
volatile float grams = 0.0f;
uint32_t last_force_time = 0;

CuneateNeuron CN;
CuneateNeuronFast CNF;
CuneateNeuronSlow CNS;

// PROTÓTIPOS
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_TIM6_Init(void);
void MX_TIM2_Init(void);
void select_row(uint8_t row);
void update_taxels(Taxel *t, uint16_t *adc, uint8_t row_idx);
bool process_spikes(void);
bool usb_buffer_write(const char *data, uint16_t len);
void usb_buffer_process(void);

void send_adc_frame(uint64_t tstamp);

void HX711_Init(void);
bool HX711_IsReady(void);
int32_t HX711_Read(void);
float HX711_ReadForce(void);
void HX711_Update(void);

float update_synapse(float *g_syn, bool spike);
float compute_Isyn(float g_syn);


// =====================================================
// SEÇÃO CRÍTICA
// =====================================================
//
// Salva e restaura o PRIMASK em vez de chamar __enable_irq() cego. Sem isto,
// uma seção crítica aninhada dentro de outra reabilitaria as interrupções ao
// sair da interna — exatamente o que aconteceria entre o ring buffer e a
// leitura do HX711.

static inline uint32_t irq_save(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void irq_restore(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}


// =====================================================
// ATRASO POR CICLOS (DWT) — COM VERIFICAÇÃO E RESERVA
// =====================================================
//
// O contador de ciclos do DWT é a forma barata de temporizar sub-microssegundo
// (o SysTick tem resolução de 1 ms), mas ligá-lo NÃO é garantido:
//
//  • no Cortex-M7 o bloco DWT é CoreSight e tem Lock Access Register. Sem
//    escrever a chave em LAR, as escritas em DWT->CTRL são simplesmente
//    IGNORADAS e CYCCNT nunca sai de zero;
//  • um depurador anexado (ou que esteve anexado) pode deixar o TRCENA em
//    estado diferente do esperado.
//
// Se CYCCNT não anda, um laço `while (CYCCNT - start < n)` NUNCA TERMINA. Como
// esse atraso é usado dentro de HX711_Read, que roda no boot durante a tara, o
// firmware travaria ANTES do laço principal — o USB continuaria enumerando
// (é interrupção) e o dispositivo ficaria mudo para sempre. É uma falha que se
// parece exatamente com "a placa não envia nada".
//
// Duas defesas: a chave do LAR + verificação de que o contador anda, e um laço
// de reserva por NOP quando ele não anda. Nenhum dos dois pode ser infinito.

#ifndef DWT_LAR_KEY
#define DWT_LAR_KEY 0xC5ACCE55U
#endif
// LAR fica em 0xE0001FB0 (base do DWT + 0xFB0). Nem toda versão do CMSIS o
// expõe na struct DWT_Type, então o acesso é pelo endereço. O harness de host
// redefine este macro para uma variável — no PC o endereço absoluto seria um
// acesso inválido.
#ifndef DWT_LAR_ADDR
#define DWT_LAR_ADDR (*(volatile uint32_t *)0xE0001FB0U)
#endif

static bool dwt_ok = false;
// Ciclos de CPU por iteração do laço de reserva. Medido de forma conservadora
// (o laço é NOP + comparação + salto); errar para MENOS só deixa o pulso mais
// longo, e o teto do HX711 é 60 us — folga de sobra sobre os 0,36 us pedidos.
#define FALLBACK_CYCLES_PER_ITER 3U

static void dwt_delay_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT_LAR_ADDR = DWT_LAR_KEY;      // destrava as escritas (Cortex-M7)
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Verificação: o contador precisa ANDAR. Sem isto, um DWT que ignorou a
    // habilitação viraria travamento no primeiro atraso.
    uint32_t c0 = DWT->CYCCNT;
    for (volatile int i = 0; i < 64; i++)
    {
        __NOP();
    }
    dwt_ok = (DWT->CYCCNT != c0);
}

/* Espera `cycles` ciclos de CPU. Sempre TERMINA: com o DWT vivo, o laço tem
 * teto de iterações; sem ele, cai no laço de reserva, que é contado. */
static inline void dwt_delay_cycles(uint32_t cycles)
{
    if (dwt_ok)
    {
        uint32_t start = DWT->CYCCNT;
        // Teto de iterações: mesmo que CYCCNT congele no meio do caminho (por
        // um depurador, por exemplo), este laço sai.
        uint32_t guard = cycles * 4U + 64U;
        while (((DWT->CYCCNT - start) < cycles) && guard--)
        {
            __NOP();
        }
        return;
    }

    for (uint32_t i = 0; i < (cycles / FALLBACK_CYCLES_PER_ITER) + 1U; i++)
    {
        __NOP();
    }
}


// =====================================================
// RELÓGIO DE 64 BITS (TIM2 a 1 MHz + extensão por software)
// =====================================================

#if TS_64BIT
static volatile uint32_t tim2_hi   = 0;
static volatile uint32_t tim2_last = 0;
#endif

/* Microssegundos desde o boot. Precisa ser chamado pelo menos uma vez por
 * volta do TIM2 (71,6 min) para não perder um incremento — é chamado a cada
 * frame (1 ms), então há margem de sobra. Seção crítica porque a ISR do ADC
 * e o laço principal chamam os dois. */
static uint64_t micros64(void)
{
#if TS_64BIT
    uint32_t primask = irq_save();

    uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);

    if (now < tim2_last)
    {
        tim2_hi++;
    }

    tim2_last = now;

    uint64_t t = ((uint64_t)tim2_hi << 32) | (uint64_t)now;

    irq_restore(primask);

    return t;
#else
    return (uint64_t)__HAL_TIM_GET_COUNTER(&htim2);
#endif
}


// =====================================================
// FORMATADORES DECIMAIS
// =====================================================
//
// snprintf com 25 conversões %d rodava DENTRO da ISR do ADC, a 1 kHz. Estes
// formatadores fazem o mesmo trabalho em uma fração do tempo e produzem
// exatamente os mesmos caracteres, então o protocolo não muda. [9]

/* Escreve `v` em decimal em `out`. Devolve quantos caracteres escreveu. */
static uint16_t u32_to_dec(uint32_t v, char *out)
{
    char tmp[10];
    uint16_t n = 0;

    if (v == 0U)
    {
        out[0] = '0';
        return 1;
    }

    while (v > 0U)
    {
        tmp[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }

    for (uint16_t i = 0; i < n; i++)
    {
        out[i] = tmp[n - 1U - i];
    }

    return n;
}

static uint16_t u64_to_dec(uint64_t v, char *out)
{
    char tmp[20];
    uint16_t n = 0;

    if (v <= 0xFFFFFFFFULL)
    {
        // Caminho rápido: evita __aeabi_uldivmod enquanto o tempo couber em
        // 32 bits, que é o caso nas primeiras 71 min de qualquer ensaio.
        return u32_to_dec((uint32_t)v, out);
    }

    while (v > 0ULL)
    {
        tmp[n++] = (char)('0' + (uint32_t)(v % 10ULL));
        v /= 10ULL;
    }

    for (uint16_t i = 0; i < n; i++)
    {
        out[i] = tmp[n - 1U - i];
    }

    return n;
}

/* Fecha uma linha com ",t=<micros>\r\n" (ou "t=..." se `comma` for false). */
static uint16_t append_ts(char *buf, uint16_t off, uint64_t t)
{
    buf[off++] = 't';
    buf[off++] = '=';
    off += u64_to_dec(t, buf + off);
    buf[off++] = '\r';
    buf[off++] = '\n';
    return off;
}

/* snprintf devolve o tamanho que a string TERIA — pode passar do buffer.
 * Usar esse valor num memcpy/usb_buffer_write lê além do array. [10] */
static uint16_t snprintf_len(int n, size_t cap)
{
    if (n < 0)
    {
        return 0;
    }

    if ((size_t)n >= cap)
    {
        return (uint16_t)(cap - 1U);   // snprintf terminou em cap-1 + '\0'
    }

    return (uint16_t)n;
}


// =====================================================
// HX711 - INICIALIZAÇÃO apagar
// =====================================================

void HX711_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    // DOUT - entrada
    g.Pin = HX711_DOUT_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(HX711_DOUT_PORT, &g);

    // SCK - saída
    g.Pin = HX711_SCK_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(HX711_SCK_PORT, &g);

    HAL_GPIO_WritePin(
        HX711_SCK_PORT,
        HX711_SCK_PIN,
        GPIO_PIN_RESET
    );
}


// =====================================================
// VERIFICA SE HX711 ESTÁ PRONTO
// =====================================================

bool HX711_IsReady(void)
{
    return HAL_GPIO_ReadPin(
        HX711_DOUT_PORT,
        HX711_DOUT_PIN
    ) == GPIO_PIN_RESET;
}


// =====================================================
// UM PULSO DE PD_SCK
// =====================================================
//
// A janela sem interrupção cobre SÓ o pulso, não a leitura inteira. Manter
// os 25 pulsos dentro de um único __disable_irq() bloqueava a ISR do ADC de
// 5 kHz; deixá-los completamente livres arriscava uma ISR longa (o
// update_taxels chega a formatar linhas) segurar o SCK em nível alto por
// mais de 60 us, o que coloca o HX711 em power-down. [6]

static inline bool hx711_pulse(void)
{
    uint32_t primask = irq_save();

    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
    dwt_delay_cycles(HX711_SCK_DELAY_CYC);

    bool bit = (HAL_GPIO_ReadPin(HX711_DOUT_PORT,
                                 HX711_DOUT_PIN) == GPIO_PIN_SET);

    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);

    irq_restore(primask);

    dwt_delay_cycles(HX711_SCK_DELAY_CYC);   // T3: nível baixo >= 0,2 us

    return bit;
}


// =====================================================
// LEITURA DE 24 BITS DO HX711
// GANHO 128 - CANAL A
// =====================================================

int32_t HX711_Read(void)
{
    uint32_t data = 0;

    // Timeout em TEMPO, não em iterações. O contador de 1.000.000 antigo
    // valia um número de voltas, não uma duração: quanto mais rápido o
    // núcleo, mais curta a espera, e ninguém sabia dizer quanto ela durava.
    // Aqui o teto é explícito e independente do clock.
    uint32_t t_start = HAL_GetTick();

    while (!HX711_IsReady())
    {
        if ((HAL_GetTick() - t_start) >= HX711_READY_TIMEOUT_MS)
        {
            return 0;
        }
    }

    for (int i = 0; i < 24; i++)
    {
        data = data << 1;

        if (hx711_pulse())
        {
            data |= 1;
        }
    }

    // Pulso adicional:
    // 1 pulso = ganho 128, canal A
    (void)hx711_pulse();

    // Conversão de complemento de dois de 24 bits
    if (data & 0x800000)
    {
        data |= 0xFF000000;
    }

    return (int32_t)data;
}


// =====================================================
// MÉDIA DE LEITURAS
// =====================================================

// =====================================================
// TARA — NÃO-BLOQUEANTE
// =====================================================
//
// A tara antiga rodava ANTES do laço principal, somando 200 leituras de
// uma vez. Enquanto ela durava, usb_buffer_process() não era chamado: o
// dispositivo enumerava (isso é interrupção) mas não transmitia UM BYTE —
// nem o banner de boot. Se a célula não respondesse, o mudo se estendia por
// toda a janela de espera; se qualquer coisa lá dentro travasse, o mudo era
// permanente. Do lado do PC os dois casos são indistinguíveis de "a placa
// está morta", e foi exatamente esse o sintoma que custou uma sessão de
// diagnóstico.
//
// Agora a tara acontece DENTRO do laço, uma amostra por volta e só quando o
// chip avisa que tem dado. O USB fala desde o primeiro milissegundo, aconteça
// o que acontecer com o HX711 — inclusive ele não estar conectado.

static bool     tare_done   = false;
static uint16_t tare_taken  = 0;
static int64_t  tare_sum    = 0;
static uint32_t tare_t0_ms  = 0;


/*Errado retirar depois*/
/* Um passo da tara. Retorna imediatamente se não há amostra pronta. */
static void HX711_TareStep(void)
{
    if (tare_done)
    {
        return;
    }

    uint32_t now = HAL_GetTick();

    if ((now - tare_t0_ms) < HX711_TARE_DELAY_MS)
    {
        return;                       // janela de estabilização do chip
    }

    if (HX711_IsReady())
    {
        tare_sum += HX711_Read();
        tare_taken++;
    }

    if (tare_taken >= HX711_TARE_SAMPLES)
    {
        hx711_offset = (int32_t)(tare_sum / tare_taken);
        tare_done = true;
        usb_buffer_write("HX711 TARE OK\r\n",
                         (uint16_t)strlen("HX711 TARE OK\r\n"));
        return;
    }

    // Desistência por tempo: sem célula (ou com ela muda) a tara nunca
    // completaria, e o resto do firmware ficaria esperando por ela.
    if ((now - tare_t0_ms) >= (HX711_TARE_DELAY_MS + HX711_TARE_WINDOW_MS))
    {
        hx711_offset = (tare_taken > 0)
                       ? (int32_t)(tare_sum / tare_taken)
                       : 0;
        tare_done = true;

        char msg[80];
        int raw = snprintf(msg, sizeof(msg),
                           "HX711 TARE TIMEOUT,%u,%u,",
                           (unsigned)tare_taken,
                           (unsigned)HX711_TARE_SAMPLES);
        uint16_t n = snprintf_len(raw, sizeof(msg));
        n = append_ts(msg, n, micros64());
        usb_buffer_write(msg, n);
    }
}


// =====================================================
// LEITURA DA FORÇA EM NEWTONS
// =====================================================

/*Errado retirar depois*/
float HX711_ReadForce(void)
{
    int32_t raw_value;

    raw_value = HX711_Read();

    /*
     * Conversão para gramas
     *
     * Se o peso aplicado produzir aumento no valor bruto:
     *
     * raw_value - hx711_offset
     *
     * Se a força aparecer negativa, troque a ordem.
     */

    float grams_local =
        ((float)(raw_value - hx711_offset))
        /
        HX711_CALIBRATION_FACTOR;

    float force =
        grams_local * GRAMS_TO_NEWTONS;

    return force;
}


// =====================================================
// ATUALIZAÇÃO DO HX711
// =====================================================
//
// Sem gate por milissegundos. O antigo calculava
// (uint32_t)(1000.0f / 80.0f) = 12, truncando 12,5 ms: a amostragem real
// ficava em 83,3 Hz enquanto o alpha do IIR era calculado para 80 Hz. Quem
// dita a cadência é o próprio HX711 (~80 SPS pelo pino RATE); basta ler
// quando ele avisa que tem amostra. HX711_IsReady() é não-bloqueante, então
// isto não segura o laço principal. [7]


/*Errado*/
void HX711_Update(void)
{
    if (!tare_done)
    {
        return;          // o offset ainda não vale; ver HX711_TareStep
    }

    if (!HX711_IsReady())
    {
        return;
    }

    // Leitura bruta
    float current_force =
        HX711_ReadForce();

    force_raw =
        current_force;

    // Filtro passa-baixa IIR
    force_filtered =
        hx711_alpha * force_raw
        +
        (1.0f - hx711_alpha)
        * force_filtered;
}

// VER VALOR POSIÇÃO DA PRIMEIRA
void init_template(void)
{
    g_template[0] = 0.0f;

    for(int i = 1; i < TEMPLATE_SIZE; i++)
    {
        g_template[i] =
            G_MAX * expf(-(float)(i-1) / 4.0f);

    }
}


// =====================================================
// USB BUFFER — ESCRITA
// =====================================================
//
// Duas correções num só lugar:
//
// [2] Seção crítica. `usb_buffer_write` tem DOIS produtores concorrentes: a
//     ISR do ADC (send_adc_frame e as linhas CN_*) e o laço principal
//     (process_spikes, FORCE, boot). O par
//         usb_tx_buffer[usb_head] = c;  usb_head = next;
//     não é atômico: uma ISR caindo entre as duas instruções escrevia no
//     mesmo índice e tinha o seu byte sobrescrito — caracteres de uma linha
//     apareciam no meio de outra.
//
// [3] All-or-nothing. A versão antiga copiava byte a byte e dava `break` ao
//     encher, cortando a linha no meio: o PC recebia um frame truncado e o
//     contava em `frames_bad`. Agora a mensagem só entra se couber inteira;
//     se não couber, é descartada por completo e contabilizada. Frame que
//     chega, chega íntegro.
//
// Devolve true se a mensagem entrou no buffer.

bool usb_buffer_write(const char *data, uint16_t len)
{
    if (len == 0U)
    {
        return true;
    }

    uint32_t primask = irq_save();

    uint16_t head = usb_head;
    uint16_t tail = usb_tail;

    // Espaço livre: uma posição fica sempre vaga para distinguir cheio de
    // vazio, daí o -1.
    uint16_t free_space =
        (uint16_t)((tail - head - 1U) % USB_TX_BUFFER_SIZE);

    if (len > free_space)
    {
        usb_dropped++;
        irq_restore(primask);
        return false;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        usb_tx_buffer[head] = (uint8_t)data[i];
        head = (uint16_t)((head + 1U) % USB_TX_BUFFER_SIZE);
    }

    usb_head = head;

    irq_restore(primask);

    return true;
}


// =====================================================
// USB BUFFER — DRENAGEM
// =====================================================

/* True se a pilha CDC pode aceitar uma nova transferência AGORA. */
static bool usb_tx_ready(void)
{
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return false;
    }

    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef *)USBD_CDC_CTX;

    return (hcdc != NULL) && (hcdc->TxState == 0U);
}

//
// [1] A correção central. CDC_Transmit_FS apenas GUARDA o ponteiro
//     (USBD_CDC_SetTxBuffer) e devolve; no OTG_FS do F7 os bytes só são
//     empurrados para a FIFO depois, dentro da interrupção. O buffer tem de
//     ficar intacto até TxState voltar a zero.
//
//     A versão antiga enchia `packet` ANTES de saber se podia transmitir.
//     Como `usb_tail` já havia avançado na chamada bem-sucedida anterior, a
//     iteração seguinte do laço principal sobrescrevia `packet` com os bytes
//     SEGUINTES enquanto a transferência anterior ainda o estava lendo — o
//     host recebia as duas metades misturadas. Depois CDC_Transmit_FS
//     devolvia USBD_BUSY, `usb_tail` não avançava, e os mesmos bytes eram
//     reenviados: corrompidos E duplicados.
//
//     Agora o teste de TxState vem PRIMEIRO, antes de tocar em `packet`.
//
// [4] `packet` cresceu para USB_TX_CHUNK. A pilha fatia sozinha em pacotes
//     de 64 B, então uma transferência de 512 B vale 8 pacotes por polling
//     do host em vez de 1.

void usb_buffer_process(void)
{
    static uint8_t packet[USB_TX_CHUNK];

    // ANTES de encher o buffer. Ver [1].
    if (!usb_tx_ready())
    {
        return;
    }

    uint16_t len = 0;
    uint16_t temp_tail = usb_tail;
    uint16_t head = usb_head;      // snapshot: a ISR pode avançá-lo aqui

    while (temp_tail != head && len < USB_TX_CHUNK)
    {
        packet[len++] = usb_tx_buffer[temp_tail];
        temp_tail = (uint16_t)((temp_tail + 1U) % USB_TX_BUFFER_SIZE);
    }

    if (len == 0U)
    {
        return;
    }

    // Uma transferência de tamanho múltiplo do wMaxPacketSize precisa de um
    // pacote de comprimento zero para o host saber que acabou. Em vez de
    // emitir o ZLP, encurta em um byte: o pacote final fica curto e fecha a
    // transferência sozinho. O byte que sobrou vai na próxima rodada.
    if ((len % USB_PACKET_SIZE) == 0U)
    {
        len--;
        temp_tail =
            (uint16_t)((usb_tail + len) % USB_TX_BUFFER_SIZE);
    }

    if (CDC_Transmit_FS(packet, len) == USBD_OK)
    {
        usb_tail = temp_tail;   // confirma envio
    }
}

// SELECT ROW
static inline uint8_t row5(uint8_t v)
{
    return ((v << 1) | (v >> 4)) & 0x1F;
}

const uint8_t row_masks[ROWS] = {
    0b11110,
    0b11101,
    0b11011,
    0b10111,
    0b01111
};

/* ATENÇÃO — o parâmetro `row` NÃO é usado: a função roda uma máscara
 * estática em vez de selecionar a linha pedida. Isso está PRESERVADO de
 * propósito. Amarrar a máscara a `row` mudaria qual taxel físico responde
 * por cada índice do frame 5x5, invalidando a calibração e as gravações já
 * existentes. É uma correção de bancada, com o sensor na mão, não de
 * escrivaninha — está anotada no CHANGELOG.md. */
void select_row(uint8_t row)
{
    (void)row;

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, (row_mask & (1<<0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5,  (row_mask & (1<<1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3,  (row_mask & (1<<2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,  (row_mask & (1<<3)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,  (row_mask & (1<<4)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    row_mask = row5(row_mask);
}

// Modelo de Izhikevith

bool izhikevich_step(float *v, float *u, float I,
                     float a, float b, float c, float d)
{
    *v += DT * (0.04f * (*v) * (*v) + 5.0f * (*v) + 140.0f - (*u) + I);
    *u += DT * (a * (b * (*v) - (*u)));

    if (*v >= VTH)
    {
        *v = c;
        *u += d;
        return true;
    }

    return false;
}

float update_synapse(float *g_syn, bool spike)
{
    // decaimento
    *g_syn -= (DT / TAU_SYN) * (*g_syn);

    // incremento por spike
    if (spike)
        *g_syn += G_MAX;


    return *g_syn;
}

float compute_Isyn(float g_syn)
{
    return g_syn;
}

void update_taxels(Taxel *t, uint16_t *adc, uint8_t row_idx)
{
    HAL_GPIO_WritePin(DEBUG_IZH_PORT,
                      DEBUG_IZH_PIN,
                      GPIO_PIN_SET);

    static float I_total_MM = 0.0f;
    static float I_total_RA = 0.0f;
    static float I_total_SA = 0.0f;

    for (int i = 0; i < COLS; i++)
    {
        int global_idx = row_idx * COLS + i;

        // =====================================================
        // ADC -> CORRENTE
        // =====================================================

        float V = adc[i] * (V_MAX / 4095.0f);

        float Vn =
            (V_MAX - V) / (V_MAX - V_MIN);

        float I_raw = Vn;

        uint8_t idx = I_index[global_idx];

        float I_old =
            I_buffer[global_idx][idx];

        I_buffer[global_idx][idx] = I_raw;

        I_index[global_idx] =
            (idx + 1) % DIFF_BUFFER;

        float dI =
            fabsf(I_raw - I_old) /
            (DIFF_BUFFER * DT);

        if (fabsf(dI) < 0.01f)
            dI = 0.0f;

        float I_RA =
            (G_RA * fabsf(dI));

        float I_SA =
            (G_SA * I_raw);

        // =====================================================
        // NEURÔNIO RA
        // =====================================================

        bool spike_ra = izhikevich_step(
            &t[i].v_RA,
            &t[i].u_RA,
            I_RA,
            A_RA,
            B_RA,
            C_RA,
            D_RA
        );

        if (spike_ra)
        {
            spike_flags_RA[global_idx] = 1;

            // excitatória
            template_idx_RA[global_idx] = 1;

            // inibitória
            template_idx_INH_RA[global_idx] = 1;
        }

        // =====================================================
        // NEURÔNIO SA
        // =====================================================

        bool spike_sa = izhikevich_step(
            &t[i].v_SA,
            &t[i].u_SA,
            I_SA,
            A_SA,
            B_SA,
            C_SA,
            D_SA
        );

        if (spike_sa)
        {
            spike_flags_SA[global_idx] = 1;

            // excitatória
            template_idx_SA[global_idx] = 1;

            // inibitória
            template_idx_INH_SA[global_idx] = 1;
        }

        // =====================================================
        // SINAPSE EXCITATÓRIA RA
        // =====================================================

        float I_ra = 0.0f;

        uint8_t idx_ra =
            template_idx_RA[global_idx];

        if(idx_ra > 0)
        {
            I_ra =
                gain_RA[global_idx] *
                g_template[idx_ra];

            idx_ra++;

            if(idx_ra >= TEMPLATE_SIZE)
            {
                idx_ra = 0;
            }

            template_idx_RA[global_idx] = idx_ra;
        }

        // =====================================================
        // SINAPSE EXCITATÓRIA SA
        // =====================================================

        float I_sa = 0.0f;

        uint8_t idx_sa =
            template_idx_SA[global_idx];

        if(idx_sa > 0)
        {
            I_sa =
                gain_SA[global_idx] *
                g_template[idx_sa];

            idx_sa++;

            if(idx_sa >= TEMPLATE_SIZE)
            {
                idx_sa = 0;
            }

            template_idx_SA[global_idx] = idx_sa;
        }

        // =====================================================
        // SOMA EXCITATÓRIA
        // =====================================================

        I_total_RA += I_ra;

        I_total_SA += I_sa;

        I_total_MM += I_ra;
        I_total_MM += I_sa;

        last_adc[global_idx] = adc[i];
    }

    // =========================================================
    // PROCESSA PÓS-SINÁPTICO APÓS TODAS AS LINHAS
    // =========================================================

    if (row_idx == (ROWS - 1))
    {
        uint64_t tstamp = micros64();

        // =====================================================
        // CORRENTE INIBITÓRIA GLOBAL
        // =====================================================

        float I_inh = 0.0f;

        // ---------- INIBIÇÃO RA ----------

        for(int k = 0; k < NUM_TAXELS; k++)
        {
            uint8_t idx_inh_ra =
                template_idx_INH_RA[k];

            if(idx_inh_ra > 0)
            {
                I_inh -=
                    W_INB *
                    g_template[idx_inh_ra];

                idx_inh_ra++;

                if(idx_inh_ra >= TEMPLATE_SIZE)
                {
                    idx_inh_ra = 0;
                }

                template_idx_INH_RA[k] =
                    idx_inh_ra;
            }
        }

        // ---------- INIBIÇÃO SA ----------

        for(int k = 0; k < NUM_TAXELS; k++)
        {
            uint8_t idx_inh_sa =
                template_idx_INH_SA[k];

            if(idx_inh_sa > 0)
            {
                I_inh -=
                    W_INB *
                    g_template[idx_inh_sa];

                idx_inh_sa++;

                if(idx_inh_sa >= TEMPLATE_SIZE)
                {
                    idx_inh_sa = 0;
                }

                template_idx_INH_SA[k] =
                    idx_inh_sa;
            }
        }

        // =====================================================
        // NORMALIZAÇÃO
        // =====================================================

        //I_total *= ganho;
        //I_inh   *= ganho;

        // =====================================================
        // CORRENTE FINAL
        // =====================================================

        //float I_final =
            //(I_total - I_inh)*ganho;

        //I_total *= correction;
        //I_inh   *= correction;

        //A normalizaçõa vem depois do inibitorio
        //inibitorio um pra cada neuronio


        I_total_RA /= NUM_TAXELS;
        I_total_SA /= NUM_TAXELS;
        I_total_MM /= (NUM_TAXELS*2);

        float I_final_MM =
            (I_total_MM + I_inh) * G_CN;

        float I_final_RA =
            (I_total_RA + I_inh) * G_CNF;

        float I_final_SA =
            (I_total_SA + I_inh) * G_CNS;

        // =====================================================
        // CUNEIFORME MULTIMODAL
        // =====================================================

        bool spike_mm = izhikevich_step(
            &CN.v_CN,
            &CN.u_CN,
            I_final_MM,
            A_CN,
            B_CN,
            C_CN,
            D_CN
        );


        // =====================================================
        // CUNEIFORME RA
        // =====================================================

        bool spike_fast = izhikevich_step(
            &CNF.v_CNF,
            &CNF.u_CNF,
            I_final_RA,
            A_CNF,
            B_CNF,
            C_CNF,
            D_CNF
        );


        // =====================================================
        // CUNEIFORME SA
        // =====================================================

        bool spike_slow = izhikevich_step(
            &CNS.v_CNS,
            &CNS.u_CNS,
            I_final_SA,
            A_CNS,
            B_CNS,
            C_CNS,
            D_CNS
        );

        // =====================================================
        // DEBUG USB
        // =====================================================
        //
        // Linhas montadas sem snprintf: estamos dentro da ISR do ADC. O texto
        // gerado é idêntico ao de antes ("CN_MM,t=<us>\r\n").

        if (spike_mm || spike_fast || spike_slow)
        {
            char msg[48];
            uint16_t n;

            if (spike_mm)
            {
                memcpy(msg, "CN_MM,", 6);
                n = append_ts(msg, 6, tstamp);
                usb_buffer_write(msg, n);
            }

            if (spike_fast)
            {
                memcpy(msg, "CN_RA,", 6);
                n = append_ts(msg, 6, tstamp);
                usb_buffer_write(msg, n);
            }

            if (spike_slow)
            {
                memcpy(msg, "CN_SA,", 6);
                n = append_ts(msg, 6, tstamp);
                usb_buffer_write(msg, n);
            }
        }


        // ENVIA ADC COMPLETO 5x5 A CADA FRAME
        send_adc_frame(tstamp);

        // =====================================================
        // RESET
        // =====================================================

        I_total_MM = 0.0f;
        I_total_RA = 0.0f;
        I_total_SA = 0.0f;
    }

    HAL_GPIO_WritePin(DEBUG_IZH_PORT,
                      DEBUG_IZH_PIN,
                      GPIO_PIN_RESET);
}

// PROCESS SPIKES
bool process_spikes(void)
{
    // 25 taxels x 2 (RA+SA) x ~40 B = ~2 KB no pior caso. O buffer tem folga,
    // mas o limite passou a ser verificado: a versão antiga fazia memcpy no
    // batch sem nunca comparar batch_count com o tamanho do array.
    static char batch_msg[2560];
    static uint16_t batch_count = 0;

    char msg[80];
    uint64_t tstamp = micros64();
    bool has_spike = false;

    for (int i = 0; i < NUM_TAXELS; i++)
    {
        if (spike_flags_RA[i])
        {
            spike_flags_RA[i] = 0;
            has_spike = true;

            int raw = snprintf(msg, sizeof(msg),
                               "RA,idx=%d,adc=%d,", i, last_adc[i]);
            uint16_t n = snprintf_len(raw, sizeof(msg));
            n = append_ts(msg, n, tstamp);

            if ((size_t)(batch_count + n) <= sizeof(batch_msg))
            {
                memcpy(batch_msg + batch_count, msg, n);
                batch_count += n;
            }
        }

        if (spike_flags_SA[i])
        {
            spike_flags_SA[i] = 0;
            has_spike = true;

            int raw = snprintf(msg, sizeof(msg),
                               "SA,idx=%d,adc=%d,", i, last_adc[i]);
            uint16_t n = snprintf_len(raw, sizeof(msg));
            n = append_ts(msg, n, tstamp);

            if ((size_t)(batch_count + n) <= sizeof(batch_msg))
            {
                memcpy(batch_msg + batch_count, msg, n);
                batch_count += n;
            }
        }
    }

    if (batch_count > 0)
    {
        usb_buffer_write(batch_msg, batch_count);
        batch_count = 0;
    }

    return has_spike;
}

/* "ADC,v0,...,v24,t=<us>\r\n" — mesmos caracteres de antes, montados sem as
 * 25 conversões %d do snprintf, que rodavam dentro da ISR a 1 kHz. [9]
 *
 * Pior caso: 4 ("ADC,") + 25*5 (4 dígitos + vírgula) + 2 ("t=") + 20
 * (microssegundos de 64 bits) + 2 (CRLF) = 153 B. */
void send_adc_frame(uint64_t tstamp)
{
    char msg[192];
    uint16_t off = 0;

    memcpy(msg, "ADC,", 4);
    off = 4;

    for (int i = 0; i < NUM_TAXELS; i++)
    {
        off += u32_to_dec((uint32_t)last_adc[i], msg + off);
        msg[off++] = ',';
    }

    off = append_ts(msg, off, tstamp);

    usb_buffer_write(msg, off);
}

/* "STAT,drop=<n>,t=<us>\r\n" — quantas linhas o ring buffer descartou desde
 * o boot. O descarte antes era mudo: o PC só via o frame sumir e somava em
 * frames_bad, sem saber se a perda foi no firmware ou no link. [5] */
static void send_stat_line(void)
{
    char msg[64];
    uint16_t off = 0;

    memcpy(msg, "STAT,drop=", 10);
    off = 10;

    uint32_t primask = irq_save();
    uint32_t dropped = usb_dropped;
    irq_restore(primask);

    off += u32_to_dec(dropped, msg + off);
    msg[off++] = ',';

    off = append_ts(msg, off, micros64());

    usb_buffer_write(msg, off);
}

// CALLBACK ADC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        HAL_GPIO_WritePin(DEBUG_ADC_PORT, DEBUG_ADC_PIN, GPIO_PIN_SET);///// PULSO NO GPIO ADC NO INICIO DA DIGITALIZAÇÃO

        ///////////////////////////

        select_row(current_row); //Seleciona linha antes de processar ADC
        update_taxels(&taxels[current_row * COLS], adc_buffer, current_row);
        current_row = (current_row + 1) % ROWS;

        /////////////////////////////////
        HAL_GPIO_WritePin(DEBUG_ADC_PORT, DEBUG_ADC_PIN, GPIO_PIN_RESET);///// PULSO NO GPIO ADC NO FIM DIGITALIZAÇÃO
    }
}

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    dwt_delay_init();     // contador de ciclos p/ o temporizador do HX711

    __HAL_RCC_GPIOA_CLK_ENABLE();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM6_Init();
    MX_TIM2_Init();
    MX_USB_DEVICE_Init();

    CN.v_CN = -30.0f;
    CN.u_CN = B_CN * CN.v_CN;

    CNF.v_CNF = -30.0f;
    CNF.u_CNF = B_CNF * CNF.v_CNF;

    CNS.v_CNS = -30.0f;
    CNS.u_CNS = B_CNS * CNS.v_CNS;

    init_template();

    hx711_alpha =
        (2.0f * 3.14159265359f * HX711_CUTOFF_FREQ)
        /
        (
            2.0f * 3.14159265359f * HX711_CUTOFF_FREQ
            +
            HX711_SAMPLE_FREQ
        );

    // O TIM2 precisa estar rodando antes de qualquer micros64().
    HAL_TIM_Base_Start(&htim2);

    // Marca o início da janela da tara. Ela acontece no LAÇO, não aqui — ver
    // HX711_TareStep. Nada entre este ponto e o while(1) pode bloquear, senão
    // o dispositivo volta a enumerar mudo.
    tare_t0_ms = HAL_GetTick();

    usb_buffer_write(
        "HX711 TARE START\r\n",
        strlen("HX711 TARE START\r\n")
    );

    for (int i = 0; i < NUM_TAXELS; i++)
    {
        taxels[i].v_RA = -30.0f;
        taxels[i].u_RA =
            B_RA * taxels[i].v_RA;

        taxels[i].v_SA = -30.0f;
        taxels[i].u_SA =
            B_SA * taxels[i].v_SA;

        taxels[i].I = 0.0f;

        gain_RA[i] =
            gain_RA_init[i];

        gain_SA[i] =
            gain_SA_init[i];
    }

    select_row(0);

    HAL_TIM_Base_Start(&htim6);

    HAL_ADC_Start_DMA(
        &hadc1,
        (uint32_t*)adc_buffer,
        COLS
    );

    usb_buffer_write(
        "BOOT OK\r\n",
        9
    );

    static uint32_t last_force_send = 0;
    static uint32_t last_stat_send  = 0;

    while (1)
    {
        process_spikes();

        HX711_TareStep();   // no-op depois que a tara termina
        HX711_Update();

        uint32_t now = HAL_GetTick();

        if ((now - last_force_send) >= 10U)
        {
            last_force_send = now;

            char msg[100];

            // %.6f depende do printf de ponto flutuante e é caro, mas o
            // formato da linha FORCE é contrato com o PC — fica como está.
            int raw = snprintf(
                msg,
                sizeof(msg),
                "FORCE,%.6f,%.6f,",
                force_raw,
                force_filtered
            );

            uint16_t n = snprintf_len(raw, sizeof(msg));
            n = append_ts(msg, n, micros64());

            usb_buffer_write(msg, n);
        }

#if USB_STAT_PERIOD_MS > 0
        if ((now - last_stat_send) >= (uint32_t)USB_STAT_PERIOD_MS)
        {
            last_stat_send = now;
            send_stat_line();
        }
#endif

        // Drenar mais de uma vez por volta do laço: cada chamada só consegue
        // enfileirar uma transferência, e o resto da volta (HX711, snprintf
        // de float) é lento o bastante para deixar o endpoint ocioso.
        usb_buffer_process();
        usb_buffer_process();
    }
}
// GPIO
void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    /* LINHAS */
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = GPIO_PIN_0 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOC, &g);

    g.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOF, &g);

    /* PINOS ADC */
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    g.Pin = GPIO_PIN_0 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_6; // ADC0, ADC3, ADC4, ADC6
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin = GPIO_PIN_1; // ADC9
    HAL_GPIO_Init(GPIOB, &g);

    ///////////////////////////////////////////////////////////

    __HAL_RCC_GPIOB_CLK_ENABLE(); //// DEFINIÇÃO DE GPIO COMO OUTPUT


    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    g.Pin = DEBUG_ADC_PIN | DEBUG_IZH_PIN;
    HAL_GPIO_Init(GPIOB, &g);

    // =====================================================
    // HX711
    // PC6 = DOUT
    // PC7 = SCK
    // =====================================================

    g.Pin = HX711_DOUT_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(GPIOC, &g);


    g.Pin = HX711_SCK_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOC, &g);

    HAL_GPIO_WritePin(
        HX711_SCK_PORT,
        HX711_SCK_PIN,
        GPIO_PIN_RESET
    );


    /////////////////////////////////////////////////////////////////
}

// DMA
void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    HAL_DMA_Init(&hdma_adc1);
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc1);
}

// ADC
void MX_ADC1_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    ADC_ChannelConfTypeDef c = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = COLS;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T6_TRGO;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    HAL_ADC_Init(&hadc1);

    uint32_t ch[COLS] = {
        ADC_CHANNEL_0,
        ADC_CHANNEL_3,
        ADC_CHANNEL_4,
        ADC_CHANNEL_6,
        ADC_CHANNEL_9,
    };

    for (int i = 0; i < COLS; i++)
    {
        c.Channel = ch[i];
        c.Rank = i + 1;
        c.SamplingTime = ADC_SAMPLETIME_15CYCLES;
        HAL_ADC_ConfigChannel(&hadc1, &c);
    }
}

// TIM6
void MX_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 83;
    htim6.Init.Period = 199; //microsegundos -- cada taxel atualizado em 1 ms

    HAL_TIM_Base_Init(&htim6);

    TIM_MasterConfigTypeDef s = {0};
    s.MasterOutputTrigger = TIM_TRGO_UPDATE;
    s.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim6, &s);
}

// ========================== TIM2
void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 83;
    htim2.Init.Period = 0xFFFFFFFF;
    HAL_TIM_Base_Init(&htim2);
}

// ========================== CLOCK ==========================
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef o = {0};
    RCC_ClkInitTypeDef c = {0};

    o.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    o.HSEState = RCC_HSE_BYPASS;
    o.PLL.PLLState = RCC_PLL_ON;
    o.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    o.PLL.PLLM = 8;
    o.PLL.PLLN = 336;
    o.PLL.PLLP = RCC_PLLP_DIV2;
    o.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&o);

    c.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV4;
    c.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&c, FLASH_LATENCY_5);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1);
}
