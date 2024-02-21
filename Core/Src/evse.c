/** @file evse.c
 *  @brief Functions to interact with EVSE
 *
 *  This contains functions and logic to determine
 *  the state of an EVSE interface, request it to
 *  turn on, and report the maximum current capability.
 *
 *  Inspired by the description of Type 2 connectors here:
 *  https://www.elso.sk/en/blog/technologies/evse-charging-of-electric-vehicles
 *
 *  @author Richard Taylor <richard@artaylor.co.uk>
 *  @bug No known bugs.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "tim.h"

#include "sensor.h"
#include "main.h"
#include "evse.h"

#define PP_UNPLUGGED_MIN      (2800)
#define PP_PRESSED_MIN        (2400)
#define PP_INSERTED_MIN       (1400)
#define PP_CHECK_INTERVAL     (100)

#define EVSE_DEFAULT_CURRENT  (6)

static uint32_t last_pp_check = 0;
static EVSE_PP pp = EVSE_PP_NONE;

static uint32_t cp_first_rise = 0;
static uint32_t cp_active = 0;
static uint32_t cp_pwm = 0;
static uint32_t max_current = EVSE_DEFAULT_CURRENT; /* Maximum Current (A x1) */

static char last_error[ERROR_LEN+1] = {0};  /* Last error string */

static evse_current_changed_cb *evse_cb = NULL;

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
    cp_pwm = 0;
    cp_active = 0;
    max_current = EVSE_DEFAULT_CURRENT;
  }
}

/**
  * @brief  Input Capture callback in non-blocking mode
  * @param  htim TIM IC handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
    uint32_t time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    if (GPIO_PIN_SET == HAL_GPIO_ReadPin(EVSE_CP_GPIO_Port, EVSE_CP_Pin))
    {
      if (cp_active != 0)
      {
        htim->Instance->CNT = 0;
        cp_pwm = 100 * (cp_active - cp_first_rise) / (time - cp_first_rise);
        cp_active = 0;
        cp_first_rise = 0;

        // Only apply if we have a valid length PWM cycle (1kHz)
        if (time > 900 && time < 1100)
        {
          if (cp_pwm >= 10)
          {
            /*
            * 6A = 10%
            * 48A = 80%
            *
            * (x + denom / 2) for integer rounding
            *
            */
            if (cp_pwm <= 80)
              max_current = 6 + ((48 - 6) * (cp_pwm - 10) + (80 - 10) / 2) / (80 - 10);
            else
              max_current = 48 + ((80 - 48) * (cp_pwm - 10) + (96 - 80) / 2) / (96 - 80);
          }
          else
          {
            max_current = EVSE_DEFAULT_CURRENT;
          }
        }
      }
      else
      {
        cp_first_rise = time;
      }
    }
    else
    {
      cp_active = time;
    }
  }
}


/**
  * @brief  Perform initialisation of evse interface
  * @param  None
  * @retval bool true: Success, false: Failure
  */
bool evse_init(evse_current_changed_cb *cb)
{
  // Start the CP PWM timer
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  evse_cb = cb;

  return true;
}

/**
  * @brief  Get the PP (connector) signal state
  * @param  None
  * @retval EVSE_PP enum with the state
  */
EVSE_PP evse_get_pp(void)
{
  int32_t val;
  HAL_StatusTypeDef ret;

  if (last_pp_check + PP_CHECK_INTERVAL < HAL_GetTick())
  {
    last_pp_check = HAL_GetTick();

    ret = sensor_get_value(SENSOR_EVSE_PP, &val);
    if (HAL_OK != ret)
    {
      pp = EVSE_PP_ERROR;
      snprintf(last_error, ERROR_LEN, "Error: %d, PP val: %ld", ret, val);
    }
    else if (val > PP_UNPLUGGED_MIN)
    {
      pp = EVSE_PP_NONE;
    }
    else if (val > PP_PRESSED_MIN)
    {
      pp = EVSE_PP_PRESSED;
    }
    else if (val > PP_INSERTED_MIN)
    {
      pp = EVSE_PP_INSERTED;
    }
  }

  return pp;
}

/**
  * @brief  Check EVSE state
  * @retval None
  */
void evse_process(void)
{
  static EVSE_PP pp_prev = EVSE_PP_NONE;
  static uint8_t cur_prev = UINT8_MAX;
  bool update = false;

  /* Check the EVSE PP Line Status */
  pp = evse_get_pp();
  if (pp != pp_prev)
  {
    pp_prev = pp;
    update = true;
  }

  if (max_current != cur_prev)
  {
    cur_prev = max_current;
    update = true;
  }

  if (update && evse_cb)
  {
    evse_cb(pp, max_current);
  }
}

/**
  * @brief  Send JSON message with EVSE Data
  * @retval None
  */
void evse_json_update(void)
{
  printf("\"evse\":{\"max_current\":%ld", max_current);

  if (strnlen(last_error, ERROR_LEN))
  {
    printf(",\"pp\":%d, \"last_error\":\"%s\"",
          pp,
          last_error);
  }

  printf("}");
}
