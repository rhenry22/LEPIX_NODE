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
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "sensor.h"
#include "evse.h"

#define PP_UNPLUGGED_MIN      (2800)
#define PP_PRESSED_MIN        (2400)
#define PP_INSERTED_MIN       (1400)
#define PP_CHECK_INTERVAL     (100)

#define EVSE_DEFAULT_CURRENT  (0)

#define TIMER_LOOPS_TIMEOUT   (500) /* 0.5s Timeout on loss of CP PWM signal */

static uint32_t last_pp_check = 0;
static EVSE_PP pp = EVSE_PP_NONE;
static EVSE_CP cp = EVSE_CP_ERROR;

static uint16_t cp_loops = 0;
static uint32_t cp_rise = 0;
static uint32_t cp_fall = 0;

static uint32_t max_current = EVSE_DEFAULT_CURRENT; /* Maximum Current (A x1) */


static char last_error[ERROR_LEN+1] = {0};  /* Last error string */

static evse_pp_changed_cb *evse_pp_cb = NULL;
static evse_cp_changed_cb *evse_cp_cb = NULL;

static osThreadId_t taskHandle;
static const osThreadAttr_t taskAttributes = {
  .name = "evseTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


static void evseTask(void *argument);

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void evse_tim_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim == &htim2)
  {
    /* We're using the same timer as a 1kHz PWM channel 
       and Capture / Compare for the CP Input */

    if (cp_loops++ > TIMER_LOOPS_TIMEOUT)
    {
      cp_loops = 0;
      max_current = 0;
    }
  }
}

/**
  * @brief  Input Capture callback in non-blocking mode
  * @param  htim TIM IC handle
  * @retval None
  */
void evse_tim_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
    uint32_t time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    if (GPIO_PIN_SET == HAL_GPIO_ReadPin(EVSE_CP_GPIO_Port, EVSE_CP_Pin))
    {
      /* CP has gone high, calculate total and reset */
      uint32_t total = time + cp_loops * 1000 - cp_rise;

      /* Only apply if we have a valid length PWM cycle (1kHz) */
      if (total > 900 && total <= 1100)
      {
        uint32_t cp_pwm = 100 * (cp_fall - cp_rise) / total;

        if (cp_pwm <= 100)
        {
          if (cp_pwm < 3)                       /* No charging Allowed */
            max_current = 0;
          else if (3 <= cp_pwm && cp_pwm <= 7)  /* ISO 15118 */
            max_current = 1;
          else if (7 < cp_pwm && cp_pwm < 8)    /* No charging Allowed */
            max_current = 0;
          else if (8 <= cp_pwm && cp_pwm < 10)  /* 6A Max */
            max_current = EVSE_DEFAULT_CURRENT;
          else if (10 <= cp_pwm && cp_pwm <= 85)/* Duty Cycle * 0.6A */
            max_current = cp_pwm * 6 / 10;
          else if (85 < cp_pwm && cp_pwm <= 96) /* (Duty Cycle - 64) * 2.5A */
            max_current = (cp_pwm - 64) * 25 / 10;
          else if (96 < cp_pwm && cp_pwm <= 97) /* 80A Max */
            max_current = 80;
          else if (97 < cp_pwm)                 /* No charging Allowed */
            max_current = 0;
        }
      }

      /* Reset for next cycle */
      cp_loops = 0;
      cp_rise = time;
      cp_fall = 0;
    }
    else
    {
      /* CP Has gone low save the time */
      cp_fall = cp_loops * 1000 + time;
    }
  }
}


/**
  * @brief  Perform initialisation of evse interface
  * @param  None
  * @retval bool true: Success, false: Failure
  */
bool evse_init(evse_pp_changed_cb *pp_cb, evse_cp_changed_cb *cp_cb)
{
  /* Start the CP PWM timer */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  evse_pp_cb = pp_cb;
  evse_cp_cb = cp_cb;

  taskHandle = osThreadNew(evseTask, NULL, &taskAttributes);

  return (taskHandle != NULL);
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
  * @brief  Get the CP (connector) signal state
  * @param  None
  * @retval EVSE_CP enum with the state
  */
EVSE_CP evse_get_cp(void)
{
  int32_t val;
  uint16_t adch, adcl;

  if (HAL_OK == sensor_get_value(SENSOR_CP, &val))
  {
    adcl = val & 0xffff;
    adch = (val >> 16) & 0xffff;

    int32_t cp_h = ((int32_t)adch - 2040) * 130 / 2056 + 5; // 11.9, 9.1, 5.9
    cp_h /= 10;
    int32_t cp_l = ((int32_t)adcl - 1960) * 135 / 1960 + 5;
    cp_l /= 10;

    if (cp_l > -10)
      cp = EVSE_CP_ERROR;
    else if (cp_h >= 10)
      cp = EVSE_CP_A;
    else if (cp_h >= 8)
      cp = EVSE_CP_B;
    else if (cp_h >= 5)
      cp = EVSE_CP_C;
    else if (cp_h >= 2)
      cp = EVSE_CP_D;
    else
      cp = EVSE_CP_A;
  }
  else
  {
    cp = EVSE_CP_ERROR;
  }

  return cp;
}

/**
  * @brief  Set the CP (connector) PWM Width
  * @param  pwm: 0-100% PWM value
  * @retval None
  */
void evse_set_cp(uint8_t pwm)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  if (pwm > 100)
    return;

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pwm * 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Thread monitoring the EVSE state.
  * @param  argument: Not used
  * @retval None
  */
static void evseTask(void *argument)
{
  for (;;)
  {
    evse_process();
    osDelay(100);
  }
}

/**
  * @brief  Check EVSE state
  * @retval None
  */
void evse_process(void)
{
  static EVSE_PP pp_prev = EVSE_PP_NONE;
  static EVSE_CP cp_prev = EVSE_CP_ERROR;
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

  if (update && evse_pp_cb)
  {
    evse_pp_cb(pp, max_current);
  }

  /* Check the CP ADC Values */
  update = false;
  cp = evse_get_cp();
  if (cp != cp_prev)
  {
    cp_prev = cp;
    update = true;
  }


  if (update && evse_cp_cb)
  {
    evse_cp_cb(cp);
  }

}

/**
  * @brief  Process EVSE related commands
  * @param  args Command arguments
  * @param  argc Number of arguments
  * @retval Status (0 = OK, -1 = Error / Unknown Command)
  */
int evse_process_cmd(char **args, int argc)
{
  int ret = 0;
  if (argc >= 2)
  {
    if (0 == strcmp(args[0], "pp"))
    {
      switch (strtol(args[1], NULL, 10))
      {
        case 0:
          /* Disable the CP line */
          HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_RESET);
        break;

        case 1:
          /* Enable the CP line */
          HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_SET);
        break;

        default:
          ret = -1;
        break;
      }
    }
    else if (0 == strcmp(args[0], "cp"))
    {
      uint32_t pwm = strtol(args[1], NULL, 10);
      if (pwm <= 100)
        evse_set_cp(pwm);
      else
        ret = -1;
    }
  }
  return ret;
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
    printf(",\"pp\":%d, \"cp\":%d, \"last_error\":\"%s\"",
          pp, cp,
          last_error);
  }

  printf("}");
}
