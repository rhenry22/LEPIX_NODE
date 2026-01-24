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
  HAL_GPIO_WritePin(OD1_EN_GPIO_Port, OD1_EN_Pin, GPIO_PIN_RESET);  // Negative
  HAL_GPIO_WritePin(OD2_EN_GPIO_Port, OD2_EN_Pin, GPIO_PIN_RESET);  // Positive
  HAL_GPIO_WritePin(OD3_EN_GPIO_Port, OD3_EN_Pin, GPIO_PIN_RESET);  // Precharge

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
