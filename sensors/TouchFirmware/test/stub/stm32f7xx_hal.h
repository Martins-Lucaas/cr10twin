#ifndef STUB_HAL_H
#define STUB_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t dummy; } GPIO_TypeDef;
extern GPIO_TypeDef *GPIOA, *GPIOB, *GPIOC, *GPIOF;
#define GPIO_PIN_0 (1u<<0)
#define GPIO_PIN_1 (1u<<1)
#define GPIO_PIN_3 (1u<<3)
#define GPIO_PIN_4 (1u<<4)
#define GPIO_PIN_5 (1u<<5)
#define GPIO_PIN_6 (1u<<6)
#define GPIO_PIN_7 (1u<<7)
#define GPIO_PIN_8 (1u<<8)
#define GPIO_PIN_9 (1u<<9)
#define GPIO_PIN_10 (1u<<10)
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
#define GPIO_MODE_INPUT 0u
#define GPIO_MODE_OUTPUT_PP 1u
#define GPIO_MODE_ANALOG 2u
#define GPIO_NOPULL 0u
#define GPIO_SPEED_FREQ_LOW 0u
#define GPIO_SPEED_FREQ_HIGH 2u
#define GPIO_SPEED_FREQ_VERY_HIGH 3u
void HAL_GPIO_Init(GPIO_TypeDef*, GPIO_InitTypeDef*);
void HAL_GPIO_WritePin(GPIO_TypeDef*, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*, uint16_t);
typedef struct { uint32_t Channel, Direction, PeriphInc, MemInc, PeriphDataAlignment,
  MemDataAlignment, Mode, Priority, FIFOMode; } DMA_InitTypeDef;
typedef struct { void *Instance; DMA_InitTypeDef Init; } DMA_HandleTypeDef;
#define DMA2_Stream0 ((void*)0)
#define DMA_CHANNEL_0 0u
#define DMA_PERIPH_TO_MEMORY 0u
#define DMA_PINC_DISABLE 0u
#define DMA_MINC_ENABLE 0u
#define DMA_PDATAALIGN_HALFWORD 0u
#define DMA_MDATAALIGN_HALFWORD 0u
#define DMA_CIRCULAR 0u
#define DMA_PRIORITY_HIGH 0u
#define DMA_FIFOMODE_DISABLE 0u
void HAL_DMA_Init(DMA_HandleTypeDef*);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef*);
typedef struct { uint32_t Resolution, ScanConvMode, ContinuousConvMode, NbrOfConversion,
  ExternalTrigConv, ExternalTrigConvEdge, DMAContinuousRequests; } ADC_InitTypeDef;
typedef struct { void *Instance; ADC_InitTypeDef Init; DMA_HandleTypeDef *DMA_Handle; } ADC_HandleTypeDef;
typedef struct { uint32_t Channel, Rank, SamplingTime; } ADC_ChannelConfTypeDef;
#define ADC1 ((void*)0)
#define ADC_RESOLUTION_12B 0u
#define ENABLE 1u
#define DISABLE 0u
#define ADC_EXTERNALTRIGCONV_T6_TRGO 0u
#define ADC_EXTERNALTRIGCONVEDGE_RISING 0u
#define ADC_CHANNEL_0 0u
#define ADC_CHANNEL_3 3u
#define ADC_CHANNEL_4 4u
#define ADC_CHANNEL_6 6u
#define ADC_CHANNEL_9 9u
#define ADC_SAMPLETIME_15CYCLES 0u
void HAL_ADC_Init(ADC_HandleTypeDef*);
void HAL_ADC_ConfigChannel(ADC_HandleTypeDef*, ADC_ChannelConfTypeDef*);
void HAL_ADC_Start_DMA(ADC_HandleTypeDef*, uint32_t*, uint32_t);
typedef struct { uint32_t Prescaler, Period; } TIM_InitTypeDef;
typedef struct { void *Instance; TIM_InitTypeDef Init; } TIM_HandleTypeDef;
typedef struct { uint32_t MasterOutputTrigger, MasterSlaveMode; } TIM_MasterConfigTypeDef;
#define TIM2 ((void*)0)
#define TIM6 ((void*)0)
#define TIM_TRGO_UPDATE 0u
#define TIM_MASTERSLAVEMODE_DISABLE 0u
void HAL_TIM_Base_Init(TIM_HandleTypeDef*);
void HAL_TIM_Base_Start(TIM_HandleTypeDef*);
void HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef*, TIM_MasterConfigTypeDef*);
extern uint32_t stub_tim_counter;
#define __HAL_TIM_GET_COUNTER(h) (stub_tim_counter)
typedef struct { uint32_t OscillatorType, HSEState; struct { uint32_t PLLState, PLLSource, PLLM, PLLN, PLLP, PLLQ; } PLL; } RCC_OscInitTypeDef;
typedef struct { uint32_t ClockType, SYSCLKSource, AHBCLKDivider, APB1CLKDivider, APB2CLKDivider; } RCC_ClkInitTypeDef;
#define RCC_OSCILLATORTYPE_HSE 0u
#define RCC_HSE_BYPASS 0u
#define RCC_PLL_ON 0u
#define RCC_PLLSOURCE_HSE 0u
#define RCC_PLLP_DIV2 0u
#define RCC_CLOCKTYPE_SYSCLK 1u
#define RCC_CLOCKTYPE_HCLK 2u
#define RCC_CLOCKTYPE_PCLK1 4u
#define RCC_CLOCKTYPE_PCLK2 8u
#define RCC_SYSCLKSOURCE_PLLCLK 0u
#define RCC_SYSCLK_DIV1 0u
#define RCC_HCLK_DIV4 0u
#define RCC_HCLK_DIV2 0u
#define FLASH_LATENCY_5 5u
void HAL_RCC_OscConfig(RCC_OscInitTypeDef*);
void HAL_RCC_ClockConfig(RCC_ClkInitTypeDef*, uint32_t);
#define __HAL_RCC_GPIOA_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_GPIOF_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_DMA2_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_ADC1_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_TIM2_CLK_ENABLE() do{}while(0)
#define __HAL_RCC_TIM6_CLK_ENABLE() do{}while(0)
typedef enum { DMA2_Stream0_IRQn = 56 } IRQn_Type;
void HAL_NVIC_SetPriority(IRQn_Type, uint32_t, uint32_t);
void HAL_NVIC_EnableIRQ(IRQn_Type);
void HAL_Init(void);
void HAL_Delay(uint32_t);
uint32_t HAL_GetTick(void);
/* CMSIS core bits */
typedef struct { uint32_t DEMCR; } CoreDebug_Type;
typedef struct { uint32_t CTRL; uint32_t CYCCNT; } DWT_Type;
extern CoreDebug_Type *CoreDebug;
extern DWT_Type *DWT;
#define CoreDebug_DEMCR_TRCENA_Msk (1u<<24)
#define DWT_CTRL_CYCCNTENA_Msk (1u<<0)
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
#define __NOP() do{}while(0)
#define __HAL_LINKDMA(h,f,d) do{ (h)->f = &(d); }while(0)
#endif
