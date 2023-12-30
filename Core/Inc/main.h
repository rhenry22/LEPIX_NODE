/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void emergency_stop(void);
void trigger_json_update(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EVSE_PP_Pin GPIO_PIN_0
#define EVSE_PP_GPIO_Port GPIOA
#define EVSE_CP_Pin GPIO_PIN_1
#define EVSE_CP_GPIO_Port GPIOA
#define ADC1_VAC_Pin GPIO_PIN_4
#define ADC1_VAC_GPIO_Port GPIOA
#define SPI1_BATT_CS__Pin GPIO_PIN_4
#define SPI1_BATT_CS__GPIO_Port GPIOC
#define LEAK_TEST_EN_Pin GPIO_PIN_5
#define LEAK_TEST_EN_GPIO_Port GPIOC
#define CHADEMO_LOCK_Pin GPIO_PIN_0
#define CHADEMO_LOCK_GPIO_Port GPIOB
#define CHADEMO_SEQ1_Pin GPIO_PIN_1
#define CHADEMO_SEQ1_GPIO_Port GPIOB
#define CHADEMO_SEQ2_Pin GPIO_PIN_2
#define CHADEMO_SEQ2_GPIO_Port GPIOB
#define CHADEMO_CHARGE_ALLOWED__Pin GPIO_PIN_7
#define CHADEMO_CHARGE_ALLOWED__GPIO_Port GPIOE
#define EVSE_CHARGE_EN_Pin GPIO_PIN_8
#define EVSE_CHARGE_EN_GPIO_Port GPIOE
#define ESP_FLASH__Pin GPIO_PIN_9
#define ESP_FLASH__GPIO_Port GPIOE
#define ESP_EN_Pin GPIO_PIN_10
#define ESP_EN_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOE
#define TEST_HV_EN_Pin GPIO_PIN_13
#define TEST_HV_EN_GPIO_Port GPIOA
#define RS485_TX_RX__Pin GPIO_PIN_7
#define RS485_TX_RX__GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

#define EVSE_Pin     LED1_Pin
#define CHADEMO_Pin  LED2_Pin
#define INVERTER_Pin LED3_Pin

#define ADC_EVSE_PP     (0)
#define ADC_BATT_CURR   (1)

#define ERROR_LEN       (128)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
