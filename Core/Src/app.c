/** @file app.c
 *  @brief Application Implementation
 *
 *  This contains the application logic connecting the modules together.
 *
 *  Copyright (c) 2026 ARTaylor.co.uk.
 *  All rights reserved.
 *
 *  @author Richard Taylor <richard@artaylor.co.uk>
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "semphr.h"
#include <stdio.h>
#include <stdlib.h>

#include "iwdg.h"
#include "usart.h"

#include "evse.h"
#include "modbus.h"
#include "sensor.h"
#include "solax.h"

#define JSON_UPDATE_TIME         (5000)

#define BUTTON_DEBOUNCE_TIME     (150)

int32_t power_offset = 0;      /* Offset from actual power (i.e. charge / discharge) */
static bool error = false;            /* Whether we are in the error state */

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

static void jsonTaskEntry(void *argument);
static void pp_changed_cb(EVSE_PP pp, uint8_t current);
/**
  * @brief  Application Init
  * @retval true: Success, false: Failure
  */
bool app_init(void)
{
  /* creation of jsonMutex */
  jsonMutexHandle = osSemaphoreNew(1, 0, &jsonMutex_attributes);

  /* creation of jsonTask */
  jsonTaskHandle = osThreadNew(jsonTaskEntry, NULL, &jsonTask_attributes);

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

  return !error;
}

/**
  * @brief  Main Application Loop.
  * @retval None
  */
void app_main(void)
{
  /* Infinite loop */
  for(;;)
  {
    /* Catch any errors and EStop */
    if (error)
    {
      emergency_stop();
    }

    /* User Button */
    {
      static uint32_t button_time = 0;

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
void app_trigger_json_update(void)
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

  app_trigger_json_update();
}

/**
* @brief Function implementing the jsonTask thread.
* @param argument: Not used
* @retval None
*/
void jsonTaskEntry(void *argument)
{
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
}
