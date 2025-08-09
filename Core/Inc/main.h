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
/* Upper / Red (bits 8-15) and Lower / Green (bits 0-7) Debug Leds */
enum debug_leds
{
  DBG_LED_HV_TEST = 0,
  DBG_LED_ISO_TEST,
  DBG_LED_CT_PRE,
  DBG_LED_CT_MAIN,
  DBG_LED_STAT_RED_INV,
  DBG_LED_STAT_RED_EV,
  DBG_LED_HV_INV,
  DBG_LED_HV_BATT,

  DBG_LED_PP_INSERTED = 8,
  DBG_LED_CP_READY,
  DBG_LED_CP_CHARGE,
};

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* Debug LED variables */
extern uint16_t debug_leds;
extern uint16_t flash_debug_leds;
extern uint8_t flash_user_mask;
extern uint8_t user_led_base;
extern bool flash_state;

/* HV Generator variables */
extern uint32_t hv_time;
extern uint32_t hv_target;
extern uint32_t hv_iso_resistance;

extern int32_t power_offset;


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* #define ESP_FLASH_MODE */

#define HV_GEN_MAX_VOLTAGE    (500) /* Max HV voltage in V */
#define HV_GEN_MIN_VOLTAGE    (0)   /* Min HV voltage in V */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void JumpToBootloader(void);
bool cmd_init(void);
void stdio_parser(uint8_t *ptr, uint16_t len);
void trigger_json_update(void);
void dump_packet(uint8_t *data, uint8_t len);

int app_process_cmd_power(char **args, int argc);
int app_process_cmd_hv(char **args, int argc);
int app_process_cmd_leds(char **args, int argc);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CHADEMO_CP_Pin GPIO_PIN_6
#define CHADEMO_CP_GPIO_Port GPIOE
#define EVSE_PP_Pin GPIO_PIN_0
#define EVSE_PP_GPIO_Port GPIOA
#define EVSE_CP_Pin GPIO_PIN_1
#define EVSE_CP_GPIO_Port GPIOA
#define CCS2_CP_Pin GPIO_PIN_2
#define CCS2_CP_GPIO_Port GPIOA
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
#define HV_FREQ_Pin GPIO_PIN_11
#define HV_FREQ_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOE
#define GPIO_INT__Pin GPIO_PIN_14
#define GPIO_INT__GPIO_Port GPIOE
#define CCS2_PWM_Pin GPIO_PIN_10
#define CCS2_PWM_GPIO_Port GPIOB
#define CTPRE_EN_Pin GPIO_PIN_11
#define CTPRE_EN_GPIO_Port GPIOB
#define CTMAIN_EN_Pin GPIO_PIN_12
#define CTMAIN_EN_GPIO_Port GPIOB
#define GPIO1_Pin GPIO_PIN_13
#define GPIO1_GPIO_Port GPIOB
#define GPIO2_Pin GPIO_PIN_14
#define GPIO2_GPIO_Port GPIOB
#define GPIO3_Pin GPIO_PIN_15
#define GPIO3_GPIO_Port GPIOB
#define ISO_TEST_EN_Pin GPIO_PIN_6
#define ISO_TEST_EN_GPIO_Port GPIOC
#define TEST_HV_EN_Pin GPIO_PIN_13
#define TEST_HV_EN_GPIO_Port GPIOA
#define HV_EN_Pin GPIO_PIN_14
#define HV_EN_GPIO_Port GPIOA
#define RS485_TX_RX__Pin GPIO_PIN_7
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
