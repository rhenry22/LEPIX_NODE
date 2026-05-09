#ifndef __TIM_H__
#define __TIM_H__

#include "main.h"

/* Handles TIM1 + 3 streams DMA2 */
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_set;
extern DMA_HandleTypeDef hdma_data;
extern DMA_HandleTypeDef hdma_reset;

/* Pins WS2815 sur GPIOD */
#define WS_PIN_D9    GPIO_PIN_9
#define WS_PIN_D11   GPIO_PIN_11
#define WS_PIN_D13   GPIO_PIN_13
#define WS_PIN_D15   GPIO_PIN_15
#define WS_ALL_PINS  (WS_PIN_D9 | WS_PIN_D11 | WS_PIN_D13 | WS_PIN_D15)

/* BSRR reset : bits 16-31 abaissent les pins */
#define BSRR_RESET_ALL  (WS_ALL_PINS << 16)

void MX_TIM1_WS2815_Init(void);

#endif /* __TIM_H__ */