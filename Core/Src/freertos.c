/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "semphr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "iwdg.h"
#include "usart.h"

#include "evse.h"
#include "modbus.h"
#include "sensor.h"
#include "solax.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define DEBUG_CONTROLLER

#define JSON_UPDATE_TIME         (5000)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int32_t power_offset = 0;      /* Offset from actual power (i.e. charge / discharge) */

static bool error = false;            /* Whether we are in the error state */

/* USER CODE END Variables */
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for jsonTask */
osThreadId_t jsonTaskHandle;
const osThreadAttr_t jsonTask_attributes = {
  .name = "jsonTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for jsonMutex */
osSemaphoreId_t jsonMutexHandle;
const osSemaphoreAttr_t jsonMutex_attributes = {
  .name = "jsonMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void pp_changed_cb(EVSE_PP pp, uint8_t current);

/* USER CODE END FunctionPrototypes */

void mainTaskEntry(void *argument);
void jsonTaskEntry(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
    __WFI();
}
/* USER CODE END 2 */

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t ulExpectedIdleTime)
{
/* place for user code */
}

__weak void PostSleepProcessing(uint32_t ulExpectedIdleTime)
{
/* place for user code */
}
/* USER CODE END PREPOSTSLEEP */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of jsonMutex */
  jsonMutexHandle = osSemaphoreNew(1, 0, &jsonMutex_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(mainTaskEntry, NULL, &mainTask_attributes);

  /* creation of jsonTask */
  jsonTaskHandle = osThreadNew(jsonTaskEntry, NULL, &jsonTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_mainTaskEntry */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_mainTaskEntry */
void mainTaskEntry(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN mainTaskEntry */

#ifndef ESP_FLASH_MODE
  /* Request supply from the EVSE */
  HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_SET);

  /* Power Up ESP8266 */
  HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);
#else
  osDelay(2000);

  HAL_GPIO_WritePin(ESP_FLASH__GPIO_Port, ESP_FLASH__Pin, GPIO_PIN_SET); // Not in Flash mode
  HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET); // Power up chip

  while(1)
    osDelay(100);
#endif

  /* Give USB and ESP a chance to start up */
  osDelay(3000);

  /* Initialise Sensors and Peripherals */

  if (!sensor_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise sensors.\"}]\n");
    error = true;
  }

#ifdef ENABLE_EVSE
  if (!evse_init(&pp_changed_cb))
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise EVSE interface.\"}]\n");
    error = true;
  }
#endif

#ifdef ENABLE_SOLAX
  if (!solax_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise Solax interface.\"}]\n");
    error = true;
  }
#endif

  if (!modbus_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise Modbus interface.\"}]\n");
    error = true;
  }

  if (!error)
    printf("{\"controller\":[{\"status\":0,\"message\":\"Initialized OK\"}]}\n");

  if (!cmd_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise command parser.\"}]\n");
    error = true;
  }

  MX_IWDG_Init();

  /* Infinite loop */
  for(;;)
  {
    /* Catch any errors and EStop */
    if (error)
    {
      emergency_stop();
    }

    /* Update the inverter power */
    solax_set_output_power(power_offset);

    /* Kick the Watchdog */
    HAL_IWDG_Refresh(&hiwdg);

	  osDelay(100);
  }
  /* USER CODE END mainTaskEntry */
}

/* USER CODE BEGIN Header_jsonTaskEntry */
/**
* @brief Function implementing the jsonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_jsonTaskEntry */
void jsonTaskEntry(void *argument)
{
  /* USER CODE BEGIN jsonTaskEntry */
  /* Infinite loop */
  for(;;)
  {
    /* Send regular JSON messages */
    xSemaphoreTake(jsonMutexHandle, JSON_UPDATE_TIME);

    printf("{\"controller\":{");
    printf("\"power_offset\":%ld", power_offset);
    printf(",\"timestamp\":%ld", HAL_GetTick());

    printf(",");
    evse_json_update();
    printf(",");
    solax_json_update();
    printf("}}\n");
  }
  /* USER CODE END jsonTaskEntry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  Trigger an update of the JSON output.
  * @retval None
  */
void trigger_json_update(void)
{
  if (xPortIsInsideInterrupt())
  {
    BaseType_t pxHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(jsonMutexHandle, &pxHigherPriorityTaskWoken);
  }
  else
  {
    xSemaphoreGive(jsonMutexHandle);
  }
}

/**
  * @brief  EVSE PP and Current callback.
  * @param  pp Current connector state
  * @param  current Maximum AC current (in / out)
  * @retval None
  */
static void pp_changed_cb(EVSE_PP pp, uint8_t current)
{
  switch (pp)
  {
    case EVSE_PP_INSERTED:
      /* Enable the CP Line */
      /* This tells the EVSE to start charging (supply power) */
      HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_SET);
    break;

    default:

    case EVSE_PP_NONE:
      /* Disable the CP line */
      HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_RESET);
    /* break; */  /* Deliberate fall through */

    case EVSE_PP_PRESSED:
      /* Update the inverter max */
      power_offset = 0;
      solax_set_output_power(0);
    break;
  }

#ifdef ENABLE_SOLAX
  /* Update the inverter max (BMS / DC handled by ChaDeMo). */
  solax_set_max_ac_current(current);
#endif

  trigger_json_update();
}

/* USER CODE END Application */

