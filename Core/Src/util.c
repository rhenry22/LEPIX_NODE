/** @file util.c
 *  @brief General Utility functions
 *
 *  Help tidy up freertos.c by moving common utility functions here.
 *
 *  Copyright (c) 2025 ARTaylor.co.uk.
 *  All rights reserved.
 *
 *  @author Richard Taylor <richard@artaylor.co.uk>
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "tim.h"

static uint8_t comm_count = 0;        /* Number of active comm sessions (I2C / SPI / UART) */

/**
  * @brief  Indicate the state of a comms session.
  * @param start_stop: Whether we are starting or stopping a session
  * @retval None
  */
void comm_session(bool start_stop)
{
  if (start_stop)
  {
    if (comm_count == 0)
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    comm_count++;
  }
  else
  {
    if (comm_count > 0)
      comm_count--;

    if (comm_count == 0)
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  }
}

/**
  * @brief  Forcibly shut everything down
  * @retval None
  */
void emergency_stop(void)
{
  volatile uint32_t i;

  /* Stop Everything in the system */
  __disable_irq();

  /* Turn Off EVSE */
  HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_RESET);

  /* Force DC contactors off */
  HAL_GPIO_WritePin(CHADEMO_SEQ2_GPIO_Port, CHADEMO_SEQ2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CHADEMO_SEQ1_GPIO_Port, CHADEMO_SEQ1_Pin, GPIO_PIN_RESET);

  /* Force Leak Test Off */
  TIM1_EStop_HiZ();
  HAL_GPIO_WritePin(LEAK_TEST_EN_GPIO_Port, LEAK_TEST_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ISO_TEST_EN_GPIO_Port, ISO_TEST_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TEST_HV_EN_GPIO_Port, TEST_HV_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HV_EN_GPIO_Port, HV_EN_Pin, GPIO_PIN_RESET);

  while(1)
  {
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    for (i=0; i<1000000; ++i);

    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    for (i=0; i<1000000; ++i);
  }
}

/**
  * @brief  Control the HV Isolation Test
  * @param  enable: true to enable, false to disable
  * @param  voltage: Target voltage for HV source (only used when enabling)
  * @retval None
  */
void hv_iso_test_enable(bool enable, uint32_t voltage)
{
  if (enable)
  {
      /* Enable ISO Test */
      HAL_GPIO_WritePin(ISO_TEST_EN_GPIO_Port, ISO_TEST_EN_Pin, GPIO_PIN_SET);
      debug_leds |= (1 << DBG_LED_ISO_TEST);

      /* Enable HV Test Source */
      HAL_GPIO_WritePin(TEST_HV_EN_GPIO_Port, TEST_HV_EN_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(HV_EN_GPIO_Port, HV_EN_Pin, GPIO_PIN_SET);

      if (hv_target == 0)
      {
        /* Set an initial PWM value to help the PID loop */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, htim1.Init.Period + 1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        debug_leds |= (1 << DBG_LED_HV_TEST);
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
      debug_leds &= ~(1 << DBG_LED_HV_TEST);

      /* Disable ISO Test */
      HAL_GPIO_WritePin(ISO_TEST_EN_GPIO_Port, ISO_TEST_EN_Pin, GPIO_PIN_RESET);
      debug_leds &= ~(1 << DBG_LED_ISO_TEST);
  }
}
