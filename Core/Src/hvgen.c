/** @file hvgen.c
 *  @brief Functions to control the HV generator
 *
 *  This contains a PID controller to control the HV generator with PWM
 *  and keep it at the set target.
 *  It also contains isolation estimation based on the current drawn by
 *  the generator.
 *
 *  Copyright (c) 2026 ARTaylor.co.uk.
 *  All rights reserved.
 *
 *  @author Richard Taylor <richard@artaylor.co.uk>
 */

#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "tim.h"

#include "hvgen.h"
#include "leds.h"
#include "sensor.h"

#define HIGH_VOLTAGE_TIMEOUT     (60000) /* Allow the HV source to be left on for this duration (max) */

#define HV_PWM_MIN               (200)
#define HV_PWM_MAX               (5600)
#define HV_PWM_DEFAULT           (HV_PWM_MAX)
#define HV_HYST_VOLT             (5)  /* Hysteresis for HV voltage control (V) */

#define HV_DCDC_C                (9800) /* Minimum current draw from HV DCDC in uA */
#define HV_DCDC_M                (110) /* DC-DC Efficiency */
#define HV_DCDC_N                (11) /* Voltage dependent loss */
#define HV_DCDC_O                (-300) /* R Offset */

/* PID tuning for HV generator (integer fixed-point) */
#define HV_PID_SCALE            (1000)
#define HV_PID_SAMPLE_MS        (100)

#define HV_PID_KP_SCALED        (-6000)
#define HV_PID_KI_SCALED        (-5)
#define HV_PID_KD_SCALED        (-800)

#define HV_PID_INTEGRAL_MAX     (100)  /* Anti-windup clamp */
#define HV_PID_MAX_DELTA        (1500)   /* Max change per sample period */

static uint32_t hv_time = 0;          /* When the HV source was enabled */
static uint32_t hv_target = 0;        /* Target HV voltage in V */
static uint32_t hv_iso_resistance = -1; /* Measured HV isolation resistance in kOhms */

static uint32_t hv_pwm = HV_PWM_DEFAULT;  /* Current value for HV PWM */

/* PID controller state for HV generator */
static int32_t hv_pid_integral = 0; /* accumulated error (samples * volts) */
static int32_t hv_pid_prev_error = 0; /* previous error (volts) */
static uint32_t hv_pid_last_time = 0;

static osThreadId_t hvGenTaskHandle;
const osThreadAttr_t hvGenTask_attributes = {
  .name = "hvGenTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static void hvGenTaskEntry(void *argument);

/**
  * @brief  HV Gen initialization
  * @param  None
  * @retval None
  */
bool hvgen_init(void)
{
  hvGenTaskHandle = osThreadNew(hvGenTaskEntry, NULL, &hvGenTask_attributes);
  return (hvGenTaskHandle != NULL);
}

/**
  * @brief  Control the HV Isolation Test
  * @param  enable: true to enable, false to disable
  * @param  voltage: Target voltage for HV source (only used when enabling)
  * @retval None
  */
void hvgen_iso_test_enable(bool enable, uint32_t voltage)
{
  if (enable)
  {
      /* Enable ISO Test */
      HAL_GPIO_WritePin(ISO_TEST_EN_GPIO_Port, ISO_TEST_EN_Pin, GPIO_PIN_SET);
      leds_set(1 << DBG_LED_ISO_TEST);

      /* Enable HV Test Source */
      HAL_GPIO_WritePin(TEST_HV_EN_GPIO_Port, TEST_HV_EN_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(HV_EN_GPIO_Port, HV_EN_Pin, GPIO_PIN_SET);

      if (hv_target == 0)
      {
        /* Set an initial PWM value to help the PID loop */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, htim1.Init.Period + 1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        leds_set(1 << DBG_LED_HV_TEST);
      }

      hv_time = HAL_GetTick();
      hv_target = voltage;
  }
  else
  {
      hv_target = 0;
      hv_time = 0;

      /* Disable HV Test Source */
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
      HAL_GPIO_WritePin(TEST_HV_EN_GPIO_Port, TEST_HV_EN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(HV_EN_GPIO_Port, HV_EN_Pin, GPIO_PIN_RESET);
      leds_clear(1 << DBG_LED_HV_TEST);

      /* Disable ISO Test */
      HAL_GPIO_WritePin(ISO_TEST_EN_GPIO_Port, ISO_TEST_EN_Pin, GPIO_PIN_RESET);
      leds_clear(1 << DBG_LED_ISO_TEST);
  }
}

/**
  * @brief  Get the isolation resistance estimate
  * @param  iso_r: Variable to receive the result
  * @retval true: success, false: failure
  */
bool hvgen_get_isolation_r(uint32_t *iso_r)
{
  if (hv_iso_resistance > 0)
  {
    *iso_r = hv_iso_resistance;
    return true;
  }

  return false;
}

/**
  * @brief  HV Gen Task
  * @param  argument Unused
  * @retval None
  */
void hvGenTaskEntry(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    /* Check HV Source Timeout */
    if (hv_time != 0 && HAL_GetTick() - hv_time > HIGH_VOLTAGE_TIMEOUT)
    {
      hvgen_process_cmd((char*[]){"iso", "0"}, 2);
      hv_time = 0;
      printf("{\"controller\":[{\"hv_timeout\":1}]}\n");
    }

    /* HV Generator PWM - We only have feedback in ISO test mode */
    if (hv_target != 0 && hv_time != 0)
    {
      int32_t batt_voltage = -1;
      int32_t hv_current = -1;

      sensor_get_value(SENSOR_BATT_VOLTAGE, &batt_voltage);
      sensor_get_value(SENSOR_HV_TEST_CURRENT, &hv_current);

      /* Calcuations based on dV, not V */
      if (hv_current <= HV_DCDC_C || batt_voltage < 1000)
        hv_iso_resistance = -1;
      else
      {
        uint32_t p1 = 5 * (hv_current - HV_DCDC_C);
        uint32_t p2 = HV_DCDC_M * p1 / 100 - HV_DCDC_N * p1 / 100000 * batt_voltage;
        hv_iso_resistance = batt_voltage * batt_voltage * 10 / p2 - HV_DCDC_O;
      }

      /* Convert sensor value to volts (same as previous code) */
      batt_voltage = batt_voltage / 10;


      /* PID controller (integer fixed-point)
         We assume a nominal sample time of HV_PID_SAMPLE_MS (100 ms). The scaled gains
         are defined above as HV_PID_K*_SCALED. Calculation uses 64-bit intermediates.
      */
      uint32_t now = HAL_GetTick();
      int32_t dt_ms = (hv_pid_last_time == 0) ? HV_PID_SAMPLE_MS : (int32_t)(now - hv_pid_last_time);
      if (dt_ms < 1) dt_ms = 1;
      if (dt_ms > HV_PID_SAMPLE_MS * 2) dt_ms = HV_PID_SAMPLE_MS; /* clamp unreasonable dt */

      /* Error = target - measured (volts) */
      int32_t error = (int32_t)hv_target - (int32_t)batt_voltage;

      /* Integrate (accumulate error scaled by dt) and clamp to avoid windup */
      hv_pid_integral += (error * dt_ms) / HV_PID_SAMPLE_MS;

      /* Anti-windup: clamp integral to reasonable bounds */
      if (hv_pid_integral > HV_PID_INTEGRAL_MAX) hv_pid_integral = HV_PID_INTEGRAL_MAX;
      if (hv_pid_integral < -HV_PID_INTEGRAL_MAX) hv_pid_integral = -HV_PID_INTEGRAL_MAX;

      /* Adjust Ki and Kd for actual dt (integer math) */
      int32_t ki_adj = (int32_t)(((int64_t)HV_PID_KI_SCALED * dt_ms) / HV_PID_SAMPLE_MS);
      int32_t kd_adj = (int32_t)(((int64_t)-HV_PID_KD_SCALED * HV_PID_SAMPLE_MS) / dt_ms);

      /* Compute P, I, D terms using 64-bit intermediates then scale down */
      int64_t p_term = (int64_t)HV_PID_KP_SCALED * (int64_t)error;
      int64_t i_term = (int64_t)ki_adj * (int64_t)hv_pid_integral;
      int64_t d_term = (int64_t)kd_adj * (int64_t)(error - hv_pid_prev_error);

      int64_t pid_sum = p_term + i_term + d_term;
      int32_t pid_out = (int32_t)(pid_sum / HV_PID_SCALE);

      /* Limit change size */
      if (pid_out > HV_PID_MAX_DELTA) pid_out = HV_PID_MAX_DELTA;
      if (pid_out < -HV_PID_MAX_DELTA) pid_out = -HV_PID_MAX_DELTA;

      /* Apply to hv_pwm and clamp */
      int32_t new_pwm = (int32_t)((int32_t)hv_pwm + pid_out);
      if (new_pwm > HV_PWM_MAX) new_pwm = HV_PWM_MAX;
      if (new_pwm < HV_PWM_MIN) new_pwm = HV_PWM_MIN;
      hv_pwm = (uint32_t)new_pwm;

      /* Save state */
      hv_pid_prev_error = error;
      hv_pid_last_time = now;

      /* Update timer compare (timer uses 0..HV_PWM_MAX scale) */
      uint32_t ccr = hv_pwm * (htim1.Init.Period + 1) / HV_PWM_MAX;
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);

      /* Telemetry: print PID state */
      //printf("{\"hv_pid\":{\"error\":%ld,\"p_term\":%ld,\"i_term\":%ld,\"d_term\":%ld,\"pid_out\":%ld,\"hv_pwm\":%lu,\"batt_voltage\":%ld,\"hv_target\":%lu}}\n",
      //        (long)error, (long)p_term, (long)i_term, (long)d_term, (long)pid_out, (unsigned long)hv_pwm, (long)batt_voltage, (unsigned long)hv_target);
    }
    else
    {
      hv_iso_resistance = -1;
      hv_pwm = HV_PWM_DEFAULT;
      /* Clear PID state when generator is not active so integrator doesn't accumulate */
      hv_pid_integral = 0;
      hv_pid_prev_error = 0;
      hv_pid_last_time = 0;
    }

    osDelay(HV_PID_SAMPLE_MS);
  }
}

/**
  * @brief  Process HV Test commands
  * @param  args Command arguments
  * @param  argc Number of arguments
  * @retval Status (0 = OK, -1 = Error / Unknown Command)
  */
int hvgen_process_cmd(char **args, int argc)
{
  int ret = -1;
  if (argc >= 2 && 0 == strcmp(args[0], "iso"))
  {
    int32_t tgt = strtol(args[1], NULL, 10);
    if (tgt < 0) tgt = 0;

    if (tgt == 0)
    {
      hvgen_iso_test_enable(false, 0);
      ret = 0;
    }
    else if (tgt >= HV_GEN_MIN_VOLTAGE && tgt <= HV_GEN_MAX_VOLTAGE)
    {
      hvgen_iso_test_enable(true, tgt);
      ret = 0;
    }
    else
    {
      /* Out of range */
      printf("{\"controller\":[{\"status\":-1,\"message\":\"HV target out of range (%d-%dV)\"}]}\n",
             HV_GEN_MIN_VOLTAGE, HV_GEN_MAX_VOLTAGE);
    }
  }
  else if (argc >= 1 && 0 == strcmp(args[0], "get"))
  {
    app_trigger_json_update();
    ret = 0;
  }
  return ret;
}
