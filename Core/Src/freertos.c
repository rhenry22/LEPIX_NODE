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

#define BUTTON_DEBOUNCE_TIME     (150)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int32_t power_offset = 0;      /* Offset from actual power (i.e. charge / discharge) */

static bool error = false;            /* Whether we are in the error state */

/* Key used to enter ESP special bridge mode on reboot (disable WDT) */
uint32_t esp_prog_key __attribute__((section(".noinit")));
bool esp_flash_mode = false;

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

void bridgeTaskEntry(void *argument);

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
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  __WFI();
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
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
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
  {
    if (esp_prog_key == ESP_MODE_KEY)
    {
      esp_prog_key = 0;
      esp_flash_mode = true;

      /* Create ESP Serial Bridge Tasks */
      mainTaskHandle = osThreadNew(bridgeTaskEntry, NULL, &mainTask_attributes);

      return;
    }
  }
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

  /* Request supply from the EVSE */
  HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_SET);

  /* Power Up ESP8266 */
  HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);

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

  if (!solax_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise Solax interface.\"}]\n");
    error = true;
  }

  if (!modbus_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise Modbus interface.\"}]\n");
    error = true;
  }
  HAL_UART_Setup_UART1();

  if (!cmd_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise command parser.\"}]\n");
    error = true;
  }

  if (!error)
  {
    printf("{\"controller\":[{\"status\":0,\"message\":\"Initialized OK\"}]}\n");
  }
  else
  {
    printf("{\"controller\":[{\"info\":\"Entering FLASH mode\"}]}\n");
    JumpToBootloader();
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

    /* User Buttons and LEDs */
    {
      static uint32_t button_time = 0;            /* Timer for button LED(s) */

      if (HAL_GPIO_ReadPin(GPIO1_GPIO_Port, GPIO1_Pin) == GPIO_PIN_SET)
        button_time = 0;

      /* Button Press */
      if (HAL_GPIO_ReadPin(GPIO1_GPIO_Port, GPIO1_Pin) == GPIO_PIN_RESET)
      {
        if ((button_time == 0 || (HAL_GetTick() - button_time) > BUTTON_DEBOUNCE_TIME))
        {
          printf("{\"controller\":[{\"button\":1}]}\n");
        }
        button_time = HAL_GetTick();
      }
    }

    /* Update the inverter power */
    solax_set_output_power(power_offset);

    /* Kick the Watchdog */
    // ToDo: Update Watchdog logic to receive regular updates from
    //       critical tasks, not just main loop.
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

    int32_t batt_voltage = 0;
    int32_t batt_current = 0;
    uint32_t err = 0;

    if (0 != app_get_batt_voltage(&batt_voltage))
      err |= 1 << 0;
    if (0 != app_get_batt_current(&batt_current))
      err |= 1 << 1;
    printf("{\"controller\":{");
    printf("\"power_offset\":%ld", power_offset);
    printf(",\"timestamp\":%ld", HAL_GetTick());

    printf(",\"sensors\":{");
    printf("\"status\":%ld", err);
    printf(",\"battery\":{\"v\":%ld, \"i\":%ld}", batt_voltage / 10, batt_current / 10);

    printf("}\n{");
    evse_json_update();

    printf("}\n{");
    solax_json_update();
    printf("}\n");
  }
  /* USER CODE END jsonTaskEntry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  Function implementing the bridgeTask thread.
  * @param  argument: Not used
  * @retval None
  */
void bridgeTaskEntry(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  osDelay(3000);

  /* Power Up ESP8266 */
  HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);

  while (1)
  {
    osDelay(100);
  }
}

/**
  * @brief  Used by modules to get the battery current (x10 V)
  * @param val Pointer to int32_t to receive the value
  * @retval 0: Success, otherwise Error value
  */
int app_get_batt_voltage(int32_t *val)
{
  // ToDo: Hook this up to the Leaf Battery module
  return -1;
}

/**
  * @brief  Used by modules to get the battery current (x10 A)
  * @param val Pointer to int32_t to receive the value
  * @retval 0: Success, otherwise Error value
  */
int app_get_batt_current(int32_t *val)
{
  // ToDo: Hook this up to the Leaf Battery module
  return -1;
}

/**
  * @brief  Used by modules to get the delta between battery and inverter voltages (x10 V)
  * @param val Pointer to int32_t to receive the value
  * @retval 0: Success, otherwise Error value
  */
int app_get_precharge_delta(uint32_t *val)
{
  int ret = 0;
  int32_t batt_current;

  /* We don't have access to the inverter voltage
     so calculate delta based on current. */
  ret = app_get_batt_current(&batt_current);
  if (ret == 0)
  {
    *val = batt_current * 33;
  }

  return ret;
}


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

  /* Update the inverter max (BMS / DC handled by ChaDeMo). */
  solax_set_max_ac_current(current);

  trigger_json_update();
}


/* USER CODE END Application */

