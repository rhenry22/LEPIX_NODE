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

#include "chademo.h"
#include "evse.h"
#include "hvgen.h"
#include "ioexp.h"
#include "leds.h"
#include "modbus.h"
#include "sensor.h"
#include "solax.h"

#define JSON_UPDATE_TIME         (5000)
#define HIGH_VOLTAGE_THRESHOLD   (500) /* 50V */
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
#ifdef TARGET_CCS2
static void cp_changed_cb(EVSE_CP cp);
#endif

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

  /* Debug and Status LEDs */
  if (!leds_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise LED Controller.\"}]\n");
    error = true;
  }

  /* Give USB and ESP a chance to start up */
  osDelay(3000);

  /* Initialise Sensors and Peripherals */

  if (!sensor_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise sensors.\"}]\n");
    error = true;
  }

#ifdef ENABLE_EVSE
#ifdef TARGET_CCS2
  if (!evse_init(&pp_changed_cb, &cp_changed_cb))
#else
  if (!evse_init(&pp_changed_cb))
#endif
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise EVSE interface.\"}]\n");
    error = true;
  }
#ifdef TARGET_CCS2
  else
  {
    evse_set_cp(100);
  }
#endif
#endif

#ifdef TARGET_CHADEMO
  if (!chademo_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise ChaDeMo interface.\"}]\n");
    error = true;
  }
#endif

  if (!hvgen_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise HV Generator interface.\"}]\n");
    error = true;
  }

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
#ifdef TARGET_CHADEMO
  /* Set Maximum DC power. Import / Export will be controlled separately. */
  chademo_set_max_power(SOLAX_MINIMUM_SUPPORTED_VOLTAGE * SOLAX_MAXIMUM_SUPPORTED_CURRENT);
#endif
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

#ifdef TARGET_CHADEMO
    /* Check shutdown */
    if (power_offset == 0 &&
        chademo_get_state() == CHADEMO_STATE_ON)
    {
      chademo_stop();
    }

    /* Check startup */
    if (power_offset != 0 &&
        chademo_get_state() == CHADEMO_STATE_OFF)
    {
      chademo_start();
    }

    /* Prevent continuous loop if we shut down */
    // ToDo: CAN timeout case?
    if (chademo_get_state() >= CHADEMO_STATE_STOP)
    {
      power_offset = 0;
    }

    /* Set Solax state based on ChaDeMo */
    if (chademo_get_state() == CHADEMO_STATE_ON)
    {
      solax_enable();
    }
    if (chademo_get_state() > CHADEMO_STATE_STOP)
    {
      solax_disable();
    }
#endif

    /* User Button */
    {
      static uint32_t button_time = 0;

      if (HAL_GPIO_ReadPin(GPIO3_GPIO_Port, GPIO3_Pin) == GPIO_PIN_SET)
        button_time = 0;

      /* Button Press */
      if (HAL_GPIO_ReadPin(GPIO3_GPIO_Port, GPIO3_Pin) == GPIO_PIN_RESET)
      {
        if ((button_time == 0 || (HAL_GetTick() - button_time) > BUTTON_DEBOUNCE_TIME))
        {
          printf("{\"controller\":[{\"button\":1}]}\n");

#ifdef TARGET_CHADEMO
        chademo_stop();
#endif
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
  if (HAL_OK == sensor_get_value(SENSOR_BATT_VOLTAGE, val))
    return 0;

  return -1;
}

/**
  * @brief  Used by modules to get the battery current (x10 A)
  * @param val Pointer to int32_t to receive the value
  * @retval 0: Success, otherwise Error value
  */
int app_get_batt_current(int32_t *val)
{
  if (HAL_OK == sensor_get_value(SENSOR_BATT_CURRENT, val))
    return 0;

  return -1;
}

/**
  * @brief  Used by modules to get the delta between battery and inverter voltages (x10 V)
  * @param val Pointer to int32_t to receive the value
  * @retval 0: Success, otherwise Error value
  */
int app_get_precharge_delta(uint32_t *val)
{
  int32_t batt_voltage;
  int32_t inv_voltage;

  if (HAL_OK != sensor_get_value(SENSOR_BATT_VOLTAGE, &batt_voltage))
    return -1;
  if (HAL_OK != sensor_get_value(SENSOR_INV_VOLTAGE, &inv_voltage))
    return -1;

  *val = labs(batt_voltage - inv_voltage);
  return 0;
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
      leds_set(1 << DBG_LED_PP_INSERTED);
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
#ifdef TARGET_CHADEMO
      chademo_stop(); /* Ensure ChaDeMo is stopped */
#endif
      leds_clear(1 << DBG_LED_PP_INSERTED);
    break;
  }

  /* Update the inverter max (BMS / DC handled by ChaDeMo). */
  solax_set_max_ac_current(current);

  app_trigger_json_update();
}

#ifdef TARGET_CCS2
/**
  * @brief  EVSE CP callback.
  * @param  cp Current vehicle state
  * @retval None
  */
static void cp_changed_cb(EVSE_CP cp)
{
  switch (cp)
  {
    case EVSE_CP_A:
      solax_disable();
      leds_clear(1 << DBG_LED_CP_READY);
      leds_clear(1 << DBG_LED_CP_CHARGE);
    break;

    case EVSE_CP_B:
      solax_set_output_power(0);
      leds_set(1 << DBG_LED_CP_READY);
      leds_clear(1 << DBG_LED_CP_CHARGE);
    break;

    case EVSE_CP_C:
    case EVSE_CP_D:
      leds_set(1 << DBG_LED_CP_READY);
      leds_set(1 << DBG_LED_CP_CHARGE);
    break;

    case EVSE_CP_ERROR:
      solax_set_output_power(0);
      solax_disable();
      leds_flash(1 << DBG_LED_CP_READY);
      leds_flash(1 << DBG_LED_CP_CHARGE);
    break;
  }

  app_trigger_json_update();
}
#endif

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

    int32_t acc_current;
    int32_t acc_voltage;
    int32_t hv_current;
    int32_t batt_voltage;
    int32_t batt_current;
    int32_t inv_voltage;
    uint32_t hv_iso_resistance = 0;
    uint32_t err = 0;

    if (HAL_OK != sensor_get_value(SENSOR_ACC_CURRENT, &acc_current))
      err |= 1 << SENSOR_ACC_CURRENT;

    if (HAL_OK != sensor_get_value(SENSOR_ACC_VOLTAGE, &acc_voltage))
      err |= 1 << SENSOR_ACC_VOLTAGE;

    if (HAL_OK != sensor_get_value(SENSOR_HV_TEST_CURRENT, &hv_current))
      err |= 1 << SENSOR_HV_TEST_CURRENT;

    if (HAL_OK != sensor_get_value(SENSOR_BATT_VOLTAGE, &batt_voltage))
      err |= 1 << SENSOR_BATT_VOLTAGE;
    if (HAL_OK != sensor_get_value(SENSOR_BATT_CURRENT, &batt_current))
      err |= 1 << SENSOR_BATT_CURRENT;

    if (HAL_OK != sensor_get_value(SENSOR_INV_VOLTAGE, &inv_voltage))
      err |= 1 << SENSOR_INV_VOLTAGE;

    if (!hvgen_get_isolation_r(&hv_iso_resistance))
      err |= 1 << SENSOR_MAX;

    // ToDo: Gather metrics and set thresholds from a working system
#if 0
    /* Check that we're outputting a sensible voltage */
    if (labs(batt_voltage - inv_voltage) > precharge_delta)
    {
      snprintf(last_error, ERROR_LEN,
          "Fire risk: Check HV fuses and connections. (%ldv, %ldv, %ldv)",
          batt_voltage, inv_voltage, precharge_delta);
      max_discharge_current = 0;
      max_charge_current = 0;
    }
#endif

    /* Display Batt and Inverter Voltages on LEDs (200-500v) */
    {
      int32_t b;
      int32_t i;
      b = (batt_voltage - 2000) * 8 / 3000;
      if (b <= 0) b = 0;
      if (b > 7) b = 7;
      if (b > 0)
        b = ((1 << b) - 1) & 0xff;

      i = (inv_voltage - 2000) * 8 / 3000;
      if (i <= 0) i = 0;
      if (i > 7) i = 7;
      if (i > 0)
        i = ((1 << i) - 1) & 0xff;
      ioexp_set_direction(IOEXP_BOT_LEDS, ~(b << 8 | i));
    }

    /* Update Debug LEDs */
    if (batt_voltage > HIGH_VOLTAGE_THRESHOLD)
      leds_set((1 << DBG_LED_HV_BATT));
    else
      leds_clear(1 << DBG_LED_HV_BATT);

    if (inv_voltage > HIGH_VOLTAGE_THRESHOLD)
      leds_set(1 << DBG_LED_HV_INV);
    else
      leds_clear(1 << DBG_LED_HV_INV);

    if (HAL_GPIO_ReadPin(CTPRE_EN_GPIO_Port, CTPRE_EN_Pin) == GPIO_PIN_SET)
      leds_set(1 << DBG_LED_CT_PRE);
    else
      leds_clear(1 << DBG_LED_CT_PRE);

    if (HAL_GPIO_ReadPin(CTMAIN_EN_GPIO_Port, CTMAIN_EN_Pin) == GPIO_PIN_SET)
      leds_set(1 << DBG_LED_CT_MAIN);
    else
      leds_clear(1 << DBG_LED_CT_MAIN);

    printf("{\"controller\":{");
    printf("\"power_offset\":%ld", power_offset);
    printf(",\"timestamp\":%ld", HAL_GetTick());

    printf(",\"sensors\":{");
    printf("\"status\":%ld", err);
    printf(",\"acc\":{\"v\":%ld, \"i\":%ld}", acc_voltage, acc_current);
    printf(",\"hv_iso\":{\"i\":%ld, \"r\":%ld}", hv_current, hv_iso_resistance);
    printf(",\"battery\":{\"v\":%ld, \"i\":%ld}", batt_voltage / 10, batt_current / 10);
    printf(",\"inverter\":{\"v\":%ld, \"dv\":%ld}", inv_voltage / 10, labs(batt_voltage - inv_voltage));

    printf("}\n{");
    evse_json_update();

    printf("}\n{");
    solax_json_update();
#ifdef TARGET_CHADEMO
    printf("}\n{");
    chademo_json_update();
#endif
    printf("}\n");
  }
}
