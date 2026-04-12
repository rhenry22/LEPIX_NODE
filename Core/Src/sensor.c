/** @file sensor.c
 *  @brief Module to interface with various sensors
 *
 *  This module provides a layer to collect sensor data and convert it
 *  to a usable unit value.
 *  Includes INA219 on I2C, MAX22530 ADC on SPI and built in ADC of the STM32.
 *
 *  Copyright (c) 2023 ARTaylor.co.uk.
 *  All rights reserved.
 *
 *  @author Richard Taylor <richard@artaylor.co.uk>
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "semphr.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "sensor.h"
#include "adc.h"

#define SENSOR_TIMEOUT_MS       (10)        /* Timeout for sensor reads */

static bool init = false;

static struct _last_value
{
  int32_t val;
  uint32_t time;
} last_values[SENSOR_MAX] = {{0, 0}};

static osSemaphoreId_t mutexHandle;
static const osSemaphoreAttr_t mutexAttributes = {
  .name = "sensorMutex"
};


/**
  * @brief  Perform initialisation of all sensors
  * @param  None
  * @retval bool true: Success, false: Failure
  */
bool sensor_init(void)
{
  bool ret = true;
  uint16_t val;

  if (HAL_OK != MX_ADC1_Get_Sample(ADC_EVSE_PP, &val))
  {
    ret = false;
  }

  mutexHandle = osSemaphoreNew(1, 1, &mutexAttributes);

  init = ret;

  return ret;
}

/**
  * @brief  Get value of specified sensor
  * @param  src Sensor to read
  * @param  val Pointer to read value
  * @retval HAL_StatusTypeDef HAL_OK on success
  */
HAL_StatusTypeDef sensor_get_value(SENSOR_SOURCE src, int32_t *val)
{
  HAL_StatusTypeDef ret = HAL_ERROR;

  assert_param(val);

  if (!init)
    return HAL_ERROR;

  /* Return cached value if within timeout */
  if (HAL_GetTick() - last_values[src].time < SENSOR_TIMEOUT_MS)
  {
    *val = last_values[src].val;
    return HAL_OK;
  }

  if (pdFALSE == xSemaphoreTake(mutexHandle, 2*SENSOR_TIMEOUT_MS))
  {
    /* Timeout obtaining mutex */
    return HAL_TIMEOUT;
  }

  switch (src)
  {
    case SENSOR_EVSE_PP: /* mV */
    {
      uint16_t tmp;
      ret = MX_ADC1_Get_Sample_Avg(ADC_EVSE_PP, &tmp);
      if (ret == HAL_OK)
        *val = ((int32_t)tmp * 3300) / 4096;
    }
    break;

    case SENSOR_MAX:
#ifdef  USE_FULL_ASSERT
      assert_failed((uint8_t*)__FILE__, __LINE__);
#endif
    break;
  }

  if (ret == HAL_OK)
  {
    last_values[src].val = *val;
    last_values[src].time = HAL_GetTick();
  }

  xSemaphoreGive(mutexHandle);

  return ret;
}