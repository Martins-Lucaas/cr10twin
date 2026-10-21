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

// Passo de integração dos Izhikevich, em MILISSEGUNDOS. Tem de ser o período
// real da ISR: TIM6 a 1 MHz com período 199 dispara a cada 200 us, e cada
// disparo roda um izhikevich_step por taxel. O valor antigo (0,10f) fazia o
// modelo andar a METADE do tempo real — todas as taxas de disparo saíam com
// fator 2. As constantes de ganho (G_RA, G_SA, G_CN*) foram ajustadas na
// bancada CONTRA o passo errado: precisam de nova calibração.
#define DT 0.20f

// Período de um frame 5x5 completo, em milissegundos: 5 linhas x 200 us.
// É o intervalo real entre duas visitas à MESMA linha, e é isso que separa
// duas amostras consecutivas de I_buffer.
#define FRAME_PERIOD_MS 1.0f

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

#define TS_64BIT 1

//#define SEND_INTERVAL_MS 100  // envio ADC a cada 10 ms

#define G_MAX 1.0f

#define TEMPLATE_SIZE 20



float g_template[TEMPLATE_SIZE];

//  ESTRUTURAS
typedef struct {
    float v_RA;
    float u_RA;

    float v_SA;
    float u_SA;
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

uint8_t usb_tx_buffer[USB_TX_BUFFER_SIZE];
volatile uint16_t usb_head = 0;
volatile uint16_t usb_tail = 0;
volatile uint32_t usb_dropped = 0;   // linhas descartadas por buffer cheio [5]

uint8_t template_idx_RA[NUM_TAXELS] = {0};
uint8_t template_idx_SA[NUM_TAXELS] = {0}; // em qual posição do template cada neurônio está

bool izhikevich_step(float *v, float *u, float I,
                     float a, float b, float c, float d);
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
void process_spikes(void);
bool usb_buffer_write(const char *data, uint16_t len);
void usb_buffer_process(void);

void send_adc_frame(uint64_t tstamp);


// =====================================================
// SEÇÃO CRÍTICA
// =====================================================
//
// Salva e restaura o PRIMASK em vez de chamar __enable_irq() cego. Sem isto,
// uma seção crítica aninhada dentro de outra reabilitaria as interrupções ao
// sair da interna — e aqui há aninhamento real: process_spikes() fecha as
// interrupções para ler os flags e chama usb_buffer_write(), que as fecha de
// novo.

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

/* Fecha uma linha com "t=<micros>\r\n".
 *
 * `cap` é o tamanho do buffer e NÃO é decorativo: snprintf_len() devolve
 * cap-1 quando o snprintf trunca, e escrever o timestamp a partir dali
 * passaria 24 bytes do fim do array. Quando o timestamp não cabe no que
 * sobrou, o OFFSET é recuado para que ele caiba: a linha perde caracteres do
 * payload mas continua terminando em "t=<n>\r\n", que é o que o parser do PC
 * procura (`parts[-1]` em touch_source.py). Linha curta o host conta como
 * frame ruim; linha sem timestamp ele não conta de jeito nenhum. */
#define APPEND_TS_MAX 24U   /* "t=" + 20 digitos + CRLF */

static uint16_t append_ts(char *buf, uint16_t off, uint16_t cap, uint64_t t)
{
    char ts[APPEND_TS_MAX];
    uint16_t k = 0;

    ts[k++] = 't';
    ts[k++] = '=';
    k += u64_to_dec(t, ts + k);
    ts[k++] = '\r';
    ts[k++] = '\n';

    if (cap < k)
    {
        return off;                 // buffer menor que o timestamp: impossível
    }

    if ((uint32_t)off + k > cap)
    {
        off = (uint16_t)(cap - k);  // recua o payload para o timestamp caber
    }

    memcpy(buf + off, ts, k);

    return (uint16_t)(off + k);
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

/* ATENÇÃO — o parâmetro `row` NÃO seleciona a linha: a função roda uma
 * máscara estática. Isso está PRESERVADO de propósito. Amarrar a máscara a
 * `row` mudaria qual taxel físico responde por cada índice do frame 5x5,
 * invalidando a calibração e as gravações já existentes. É uma correção de
 * bancada, com o sensor na mão, não de escrivaninha — está no CHANGELOG.md.
 *
 * O que `row` passou a fazer é VIGIAR essa escolha. `row_mask` roda por conta
 * própria e `current_row` roda por conta própria; um callback de ADC perdido
 * ou repetido desfasa os dois PERMANENTEMENTE, e o frame 5x5 continua saindo
 * bem formado — só que com os taxels rotacionados. Era a única falha aqui que
 * não tinha sintoma. Agora tem: uma linha "ROWSYNC ERR", uma vez.
 *
 * A máscara escrita é sempre a da PRÓXIMA linha (a escrita de agora vale para
 * a conversão seguinte), então em fase vale
 *     row_mask == row_masks[(row + 1) % ROWS].
 * A chamada de priming do main() entra fora de fase por construção, daí o
 * `armed`. */
void select_row(uint8_t row)
{
    static bool armed = false;
    static bool reported = false;

    if (armed && !reported &&
        (row >= ROWS || row_mask != row_masks[(row + 1U) % ROWS]))
    {
        reported = true;
        usb_buffer_write("ROWSYNC ERR\r\n", 13);
    }

    armed = true;

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

        // I_old é a amostra de DIFF_BUFFER FRAMES atrás, e um frame dura
        // FRAME_PERIOD_MS (1 ms) — a linha só é revisitada a cada 5 disparos
        // de 200 us. Dividir por (DIFF_BUFFER * DT) supunha 2,5 ms em vez de
        // 25 ms e inflava dI — e portanto I_RA — em 10x.
        float dI =
            fabsf(I_raw - I_old) /
            (DIFF_BUFFER * FRAME_PERIOD_MS);

        // Dead-band de ruído. O limiar acompanhou a correção da escala acima
        // (0,01 -> 0,001) para continuar cortando exatamente a mesma variação
        // física de I_raw que cortava antes.
        if (dI < 0.001f)
            dI = 0.0f;

        float I_RA =
            (G_RA * dI);

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
            template_idx_RA[global_idx] = 1;
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
            template_idx_SA[global_idx] = 1;
        }

        // =====================================================
        // SINAPSE EXCITATÓRIA RA
        // =====================================================

        float I_ra = 0.0f;

        uint8_t idx_ra =
            template_idx_RA[global_idx];

        if(idx_ra > 0)
        {
            I_ra = g_template[idx_ra];

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
            I_sa = g_template[idx_sa];

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

        float I_final_MM = I_total_MM * G_CN;

        float I_final_RA = I_total_RA * G_CNF;

        float I_final_SA = I_total_SA * G_CNS;

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
                n = append_ts(msg, 6, sizeof(msg), tstamp);
                usb_buffer_write(msg, n);
            }

            if (spike_fast)
            {
                memcpy(msg, "CN_RA,", 6);
                n = append_ts(msg, 6, sizeof(msg), tstamp);
                usb_buffer_write(msg, n);
            }

            if (spike_slow)
            {
                memcpy(msg, "CN_SA,", 6);
                n = append_ts(msg, 6, sizeof(msg), tstamp);
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
void process_spikes(void)
{
    // 25 taxels x 2 (RA+SA) x ~40 B = ~2 KB no pior caso. O buffer tem folga,
    // mas o limite passou a ser verificado: a versão antiga fazia memcpy no
    // batch sem nunca comparar batch_count com o tamanho do array.
    static char batch_msg[2560];
    static uint16_t batch_count = 0;

    char msg[80];
    uint64_t tstamp = micros64();

    for (int i = 0; i < NUM_TAXELS; i++)
    {
        // Ler e zerar tem de ser ATÔMICO: a ISR do ADC marca estes flags. Um
        // spike que chegasse entre o teste e o zeramento era perdido sem
        // deixar rastro — `volatile` ordena o acesso, não o torna indivisível.
        uint32_t primask = irq_save();
        bool had_ra = spike_flags_RA[i];
        spike_flags_RA[i] = 0;
        bool had_sa = spike_flags_SA[i];
        spike_flags_SA[i] = 0;
        irq_restore(primask);

        if (had_ra)
        {
            int raw = snprintf(msg, sizeof(msg),
                               "RA,idx=%d,adc=%d,", i, last_adc[i]);
            uint16_t n = snprintf_len(raw, sizeof(msg));
            n = append_ts(msg, n, sizeof(msg), tstamp);

            if ((size_t)(batch_count + n) <= sizeof(batch_msg))
            {
                memcpy(batch_msg + batch_count, msg, n);
                batch_count += n;
            }
        }

        if (had_sa)
        {
            int raw = snprintf(msg, sizeof(msg),
                               "SA,idx=%d,adc=%d,", i, last_adc[i]);
            uint16_t n = snprintf_len(raw, sizeof(msg));
            n = append_ts(msg, n, sizeof(msg), tstamp);

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

    off = append_ts(msg, off, sizeof(msg), tstamp);

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

    // Leitura de 32 bits alinhada já é indivisível no Cortex-M7.
    uint32_t dropped = usb_dropped;

    off += u32_to_dec(dropped, msg + off);
    msg[off++] = ',';

    off = append_ts(msg, off, sizeof(msg), micros64());

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

    // O TIM2 precisa estar rodando antes de qualquer micros64().
    HAL_TIM_Base_Start(&htim2);

    for (int i = 0; i < NUM_TAXELS; i++)
    {
        taxels[i].v_RA = -30.0f;
        taxels[i].u_RA =
            B_RA * taxels[i].v_RA;

        taxels[i].v_SA = -30.0f;
        taxels[i].u_SA =
            B_SA * taxels[i].v_SA;
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

    static uint32_t last_stat_send = 0;

    while (1)
    {
        process_spikes();

        uint32_t now = HAL_GetTick();

        if ((now - last_stat_send) >= USB_STAT_PERIOD_MS)
        {
            last_stat_send = now;
            send_stat_line();
        }

        // Uma chamada por volta. A segunda que existia aqui era placebo: ela
        // caía no `if (!usb_tx_ready()) return`, porque TxState só volta a
        // zero quando a transferência anterior completa, microssegundos
        // depois, dentro da interrupção do OTG.
        usb_buffer_process();
    }
}
// GPIO
void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // GPIOB estava sendo ligado LÁ EMBAIXO, depois do HAL_GPIO_Init de PB1.
    // Escrita em periférico com o clock fechado é descartada: PB1 (ADC9, a
    // quinta coluna) ficava no modo de reset — entrada digital, buffer
    // Schmitt ligado — em vez de analógica, em todos os frames desde sempre.
    __HAL_RCC_GPIOB_CLK_ENABLE();
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

    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    g.Pin = DEBUG_ADC_PIN | DEBUG_IZH_PIN;
    HAL_GPIO_Init(GPIOB, &g);
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

    // Prioridade 5, não 0. Na 0 esta ISR era a mais alta do sistema, acima
    // do OTG_FS: a cada 1 ms ela roda o passo de 25 neurônios e monta os 153
    // bytes do frame ADC, e enquanto isso o USB não era atendido. Agora o
    // OTG_FS (que fica na 0 por padrão do CubeMX) a preempta; a ISR do ADC
    // continua com 200 us de folga para terminar.
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
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
