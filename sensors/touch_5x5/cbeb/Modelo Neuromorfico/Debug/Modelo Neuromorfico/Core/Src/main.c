GABRIELA/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <string.h>

/* Definições ---------------------------------------------------------------*/
#define ROWS 4
#define COLS 4
#define NUM_SENSORS (ROWS * COLS)

/* Parâmetros do modelo Izhikevich */
static float a = 0.02f;
static float b = 0.2f;
static float c = -65.0f;
static float d = 8.0f;

/* Estados dos neurônios */
static float v[NUM_SENSORS];
static float u[NUM_SENSORS];

/* Buffer ADC DMA: 4 linhas por coluna */
static uint16_t adc_buffer[ROWS];

/* Handles HAL */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
UART_HandleTypeDef huart3;
TIM_HandleTypeDef htim6;

/* Coluna ativa */
volatile uint8_t current_col = 0;

/* =========================
   Mapeamentos de Hardware
   ========================= */

/* Linhas (entradas analógicas) -> ADC1 IN0..IN3 (PA0..PA3).
   Ajuste se usar outros pinos/canais. */
static const uint32_t adc_channels[ROWS] = {
    ADC_CHANNEL_0,   // PA0
    ADC_CHANNEL_1,   // PA1
    ADC_CHANNEL_2,   // PA2
    ADC_CHANNEL_3    // PA3
};

/* Colunas (saídas digitais para multiplexar) -> ajuste conforme seu hardware:
   Ex.: PC0, PC3, PF3, PF5 */
typedef struct { GPIO_TypeDef* port; uint16_t pin; } gpio_pin_t;
static const gpio_pin_t column_pins[COLS] = {
    {GPIOC, GPIO_PIN_0},  // Coluna 0
    {GPIOC, GPIO_PIN_3},  // Coluna 1
    {GPIOF, GPIO_PIN_3},  // Coluna 2
    {GPIOF, GPIO_PIN_5}   // Coluna 3
};

/* Pinos de saída para pulsos de spike por taxel (exemplo; ajuste conforme seu hardware) */
static GPIO_TypeDef* spike_ports[NUM_SENSORS] = {
    GPIOD, GPIOD, GPIOD, GPIOE,
    GPIOE, GPIOE, GPIOF, GPIOF,
    GPIOF, GPIOF, GPIOF, GPIOG,
    GPIOG, GPIOG, GPIOG, GPIOG
};
static uint16_t spike_pins[NUM_SENSORS] = {
    GPIO_PIN_13, GPIO_PIN_12, GPIO_PIN_11, GPIO_PIN_2,
    GPIO_PIN_3,  GPIO_PIN_4,  GPIO_PIN_5,  GPIO_PIN_6,
    GPIO_PIN_7,  GPIO_PIN_8,  GPIO_PIN_9,  GPIO_PIN_10,
    GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14
};

/* Protótipos ---------------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
void Error_Handler(void);

static void process_neurons_column(uint8_t col);
static void select_column(uint8_t col);

/* MAIN ---------------------------------------------------------------------*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_USART3_UART_Init();
    MX_TIM6_Init();

    /* Inicializa neurônios */
    for (int i = 0; i < NUM_SENSORS; i++) {
        v[i] = -65.0f;
        u[i] = b * v[i];
    }

    /* Seleciona coluna inicial */
    select_column(0);

    /* Inicia ADC (DMA circular) e TIM6 (4 kHz) */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ROWS) != HAL_OK) Error_Handler();
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();

    while (1) {
        /* Tudo acontece nas interrupções */
    }
}

/* =========================
   Lógica de processamento
   ========================= */
static void process_neurons_column(uint8_t col)
{
    int spike_flags[ROWS] = {0};

    for (int r = 0; r < ROWS; r++) {
        int idx = col * ROWS + r;

        /* Converte ADC para tensão (0..3.3V) e aplica ganho I=G*V */
        float I = ((float)adc_buffer[r] / 4095.0f) * 3.3f;
        I *= 100.0f; /* G = 100 */

        /* Izhikevich (dt = 1 ms = 0.001 s) */
        v[idx] += 0.001f * (0.04f * v[idx] * v[idx] + 5.0f * v[idx] + 140.0f - u[idx] + I);
        u[idx] += 0.001f * (a * (b * v[idx] - u[idx]));

        if (v[idx] >= 30.0f) {
            v[idx] = c;
            u[idx] += d;
            spike_flags[r] = 1;

            /* Pulso rápido no GPIO (~10–15 us) */
            HAL_GPIO_WritePin(spike_ports[idx], spike_pins[idx], GPIO_PIN_SET);
            for (volatile int dly = 0; dly < 200; dly++) { __NOP(); }
            HAL_GPIO_WritePin(spike_ports[idx], spike_pins[idx], GPIO_PIN_RESET);
        }
    }

    /* Debug via UART */
    char msg[64];
    int len = snprintf(msg, sizeof(msg), "Col %d:", col);
    for (int r = 0; r < ROWS; r++) {
        len += snprintf(msg + len, sizeof(msg) - len, " %d", spike_flags[r]);
    }
    len += snprintf(msg + len, sizeof(msg) - len, "\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, HAL_MAX_DELAY);
}

static void select_column(uint8_t col)
{
    /* Desativa todas */
    for (int c = 0; c < COLS; c++) {
        HAL_GPIO_WritePin(column_pins[c].port, column_pins[c].pin, GPIO_PIN_RESET);
    }
    /* Ativa a desejada */
    HAL_GPIO_WritePin(column_pins[col].port, column_pins[col].pin, GPIO_PIN_SET);
}

/* Callback do Timer --------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        /* Processa neurônios da coluna atual */
        process_neurons_column(current_col);

        /* Próxima coluna (multiplex) */
        current_col = (current_col + 1) % COLS;
        select_column(current_col);
    }
}

/* =========================
   Inicializações de Perif.
   ========================= */

/* TIM6 - 4 kHz -------------------------------------------------------------*/
static void MX_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim6.Instance = TIM6;
    /* APB1 timer clock = 108 MHz (com overdrive 216 MHz e APB1 /4, timers multiplicam por 2) */
    htim6.Init.Prescaler = 108 - 1;  /* 108 MHz / 108 = 1 MHz */
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 250 - 1;     /* 1 MHz / 250 = 4 kHz */
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) Error_Handler();

    /* NVIC TIM6 */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* ADC1 - leitura de 4 canais por DMA --------------------------------------*/
static void MX_ADC1_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ENABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfDiscConversion   = 0;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T6_TRGO; /* F7: use esta constante */
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = ROWS; /* 4 canais */
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    for (int i = 0; i < ROWS; i++) {
        sConfig.Channel      = adc_channels[i];
        sConfig.Rank         = i + 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
    }
}

/* DMA2 Stream0 para ADC1 ---------------------------------------------------*/
static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_adc1.Instance                 = DMA2_Stream0;
    hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    /* NVIC DMA */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* UART3 (115200 8N1) -------------------------------------------------------*/
static void MX_USART3_UART_Init(void)
{
    __HAL_RCC_USART3_CLK_ENABLE();

    huart3.Instance        = USART3;
    huart3.Init.BaudRate   = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits   = UART_STOPBITS_1;
    huart3.Init.Parity     = UART_PARITY_NONE;
    huart3.Init.Mode       = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

/* GPIO: colunas como saída, linhas analógicas e pinos de spike -------------*/
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Linhas analógicas: PA0..PA3 (ADC1 IN0..IN3) */
    GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Colunas: PC0, PC3, PF3, PF5 como saída push-pull */
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_3;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GPIO_PIN_3 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* Pinos de spike: saída rápida */
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    for (int i = 0; i < NUM_SENSORS; i++) {
        GPIO_InitStruct.Pin = spike_pins[i];
        HAL_GPIO_Init(spike_ports[i], &GPIO_InitStruct);
        HAL_GPIO_WritePin(spike_ports[i], spike_pins[i], GPIO_PIN_RESET);
    }
}

/* Clock do sistema (F7, 216 MHz) ------------------------------------------*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Habilita clock do PWR e escala de voltagem */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Osciladores: HSE + PLL a 216 MHz */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM            = 4;
    RCC_OscInitStruct.PLL.PLLN            = 216;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    /* OverDrive para 216 MHz */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) Error_Handler();

    /* Clocks AHB/APB */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                                     | RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* 216 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;     /* 54 MHz (timers x2 = 108 MHz) */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;     /* 108 MHz */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) Error_Handler();
}

/* Error Handler ------------------------------------------------------------*/
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}


