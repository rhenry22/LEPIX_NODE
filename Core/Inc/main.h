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
#include <stdbool.h>
#include "util.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern int32_t power_offset;


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* #define ESP_FLASH_MODE */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void JumpToBootloader(void);
bool cmd_init(void);
void stdio_parser(uint8_t *ptr, uint16_t len);
void trigger_json_update(void);
void dump_packet(uint8_t *data, uint8_t len);

int app_get_batt_voltage(int32_t *val);
int app_get_batt_current(int32_t *val);

int app_process_cmd_ctrl(char **args, int argc);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EVSE_PP_Pin GPIO_PIN_0
#define EVSE_PP_GPIO_Port GPIOA
#define EVSE_CP_Pin GPIO_PIN_1
#define EVSE_CP_GPIO_Port GPIOA
#define OD1_EN_Pin GPIO_PIN_6
#define OD1_EN_GPIO_Port GPIOA
#define OD2_EN_Pin GPIO_PIN_7
#define OD2_EN_GPIO_Port GPIOA
#define OD3_EN_Pin GPIO_PIN_0
#define OD3_EN_GPIO_Port GPIOB
#define IGN_EN_Pin GPIO_PIN_7
#define IGN_EN_GPIO_Port GPIOE
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
#define GPIO1_Pin GPIO_PIN_6
#define GPIO1_GPIO_Port GPIOC
#define GPIO2_Pin GPIO_PIN_7
#define GPIO2_GPIO_Port GPIOC
#define RS485_TX_RX__Pin GPIO_PIN_4
#define RS485_TX_RX__GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

#define LED_GPIO_Port GPIOE

#define ADC_EVSE_PP     (0)
#define ADC_BATT_CURR   (1)

#define ADC2_CCS2_CP    (0)

#define ERROR_LEN       (128)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
