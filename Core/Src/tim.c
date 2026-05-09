#include "tim.h"

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_set;
DMA_HandleTypeDef hdma_data;
DMA_HandleTypeDef hdma_reset;

void MX_TIM1_WS2815_Init(void)
{
    /* GPIO PD9/PD11/PD13/PD15 en output PP */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin   = WS_ALL_PINS,
        .Mode  = GPIO_MODE_OUTPUT_PP,
        .Pull  = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    };
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    GPIOD->BSRR = BSRR_RESET_ALL;

    /* TIM1 : APB2=168MHz, PSC=0, ARR=69 -> 2.4MHz
     * 3 slots x 0.417us = 1.25us (800kHz WS2815)
     * CCR1=29 -> T0H ~0.36us
     * CCR2=58 -> T1H ~0.71us */
    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 69;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim1);

    TIM_OC_InitTypeDef sOC = {
        .OCMode     = TIM_OCMODE_TIMING,
        .Pulse      = 29,
        .OCPolarity = TIM_OCPOLARITY_HIGH,
        .OCFastMode = TIM_OCFAST_DISABLE,
    };
    HAL_TIM_OC_ConfigChannel(&htim1, &sOC, TIM_CHANNEL_1);
    sOC.Pulse = 58;
    HAL_TIM_OC_ConfigChannel(&htim1, &sOC, TIM_CHANNEL_2);

    /* DMA2 */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* hdma_set : TIM1_UP -> DMA2_Stream5 Ch6 */
    hdma_set.Instance                 = DMA2_Stream5;
    hdma_set.Init.Channel             = DMA_CHANNEL_6;
    hdma_set.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_set.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_set.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_set.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_set.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_set.Init.Mode                = DMA_NORMAL;
    hdma_set.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_set.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_set);
    __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_UPDATE], hdma_set);

    /* hdma_data : TIM1_CH1 -> DMA2_Stream1 Ch6 */
    hdma_data.Instance                 = DMA2_Stream1;
    hdma_data.Init.Channel             = DMA_CHANNEL_6;
    hdma_data.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_data.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_data.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_data.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_data.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_data.Init.Mode                = DMA_NORMAL;
    hdma_data.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_data.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_data);
    __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC1], hdma_data);

    /* hdma_reset : TIM1_CH2 -> DMA2_Stream2 Ch6 */
    hdma_reset.Instance                 = DMA2_Stream2;
    hdma_reset.Init.Channel             = DMA_CHANNEL_6;
    hdma_reset.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_reset.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_reset.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_reset.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_reset.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_reset.Init.Mode                = DMA_NORMAL;
    hdma_reset.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_reset.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_reset);
    __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC2], hdma_reset);

    /* NVIC */
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}