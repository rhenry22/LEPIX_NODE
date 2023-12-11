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

static uint32_t last_pp_check = 0;
static EVSE_PP pp = EVSE_PP_NONE;

static uint32_t cp_first_rise = 0;
static uint32_t cp_active = 0;
static uint32_t cp_pwm = 0;
static uint32_t max_current = 0;      /* Maximum Current (A x1) */

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
    max_current = 0;
    //HAL_GPIO_TogglePin(LED3_GPIO_Port, EVSE_Pin);
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
            */
            if (cp_pwm <= 80)
              max_current = 6 + (48 - 6) * (cp_pwm - 9) / (80 - 10);
            else
              max_current = 48 + (80 - 48) * (cp_pwm - 9) / (96 - 80);
          }
          else
          {
            max_current = 0;
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
bool evse_init(void)
{
  // Start the CP PWM timer
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  return true;
}

/**
  * @brief  Get the maximum current allowed by the EVSE
  * @param  current Pointer to variable to receive current (A x1)
  * @retval None
  */
void evse_get_max_current(uint8_t *current)
{
  if (current)
    *current = max_current;
}

/**
  * @brief  Get the PP (connector) signal state
  * @param  None
  * @retval EVSE_PP enum with the state
  */
EVSE_PP evse_get_pp(void)
{
  int32_t val;
  
  if (last_pp_check + PP_CHECK_INTERVAL < HAL_GetTick())
  {
    last_pp_check = HAL_GetTick();

    if (HAL_OK != sensor_get_value(SENSOR_EVSE_PP, &val))
    {
      pp = EVSE_PP_ERROR;
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
  * @brief  Send JSON message with EVSE Data
  * @retval None
  */
void evse_json_update(void)
{
  printf("{\"evse\":[");

  printf("{\"pp\":%d, \"max_current\":%ld}",
         pp,
         max_current);

  printf("]}\n");
  }