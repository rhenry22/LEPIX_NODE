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
#include "max22530.h"
#include "ina219.h"
#include "adc.h"

#define INA219_ACC_ADDR         (0x41)
#ifdef TARGET_CCS2
#define INA219_ACC_SHUNT        (0.010)     /* 10mR Shunt resistor */
#else
#define INA219_ACC_SHUNT        (0.005)     /* 5mR Shunt resistor */
#endif

#define INA219_ACC_CURRENT_LSB  (0.001)     /* 1mA per LSB */

#define INA219_HV_ADDR          (0x44)      /* Main board: 0x40, Daughter board: 0x44 */
#define INA219_HV_SHUNT         (0.1)       /* 100mR Shunt resistor */
#define INA219_HV_CURRENT_LSB   (0.000050)  /* 50uA per LSB */

#define SENSOR_TIMEOUT_MS       (10)        /* Timeout for sensor reads */

static uint16_t ibatt_zero = 2048;
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
#ifdef ENABLE_MAX22530
  if (!MAX22530_Init())
  {
    ret = false;
  }
#endif

#ifdef ENABLE_INA219
  if (!ina219_init(INA219_ACC_ADDR, 0.04096 / (INA219_ACC_CURRENT_LSB * INA219_ACC_SHUNT)))
  {
    ret = false;
  }

  if (!ina219_init(INA219_HV_ADDR, 0.04096 / (INA219_HV_CURRENT_LSB * INA219_HV_SHUNT)))
  {
    ret = false;
  }
#endif

  if (HAL_OK != MX_ADC1_Get_Sample_Avg(ADC_BATT_CURR, &ibatt_zero))
  {
    ret = false;
  }

  if (HAL_OK != MX_ADC1_Get_Sample(ADC_EVSE_PP, &val))
  {
    ret = false;
  }

  mutexHandle = osSemaphoreNew(1, 1, &mutexAttributes);

  init = ret;

  return ret;
}

/**
  * @brief  Zero the battery current sensor
  * @retval HAL_StatusTypeDef HAL_OK on success
  */
HAL_StatusTypeDef sensor_zero_ibatt(void)
{
  HAL_StatusTypeDef ret;

  if (!init)
    return HAL_ERROR;

  ret = MX_ADC1_Get_Sample_Avg(ADC_BATT_CURR, &ibatt_zero);

  if (ret == HAL_OK)
  {
    if (ibatt_zero < 100)
      ret = HAL_ERROR;
  }

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
#ifdef ENABLE_MAX22530
    case SENSOR_BATT_VOLTAGE: /* V x10 */
    {
      uint16_t tmp;
      ret = MAX22530_read_register(MAX22530_ADC1, &tmp);
      if (ret == HAL_OK)
        *val = (int32_t)tmp * MAX22530_VREF / 4096 * (4.7 + 1500) * 10 / 4.7 / 1000;
    }
    break;

    case SENSOR_INV_VOLTAGE: /* V x10 */
    {
      uint16_t tmp;
      ret = MAX22530_read_register(MAX22530_ADC2, &tmp);
      if (ret == HAL_OK)
        *val = (int32_t)tmp * MAX22530_VREF / 4096 * (4.7 + 1500) * 10 / 4.7 / 1000;
    }
    break;
#endif

#ifdef ENABLE_INA219
    case SENSOR_ACC_VOLTAGE: /* mV */
    {
      int16_t reg;
      ret = ina219_read_reg(INA219_ACC_ADDR, 0x02, &reg);
      if (ret == HAL_OK)
        *val = (int32_t)((reg & 0xFFF8) >> 1);
    }
    break;

    case SENSOR_ACC_CURRENT: /* uA */
    {
      int16_t reg;
      ret = ina219_read_reg(INA219_ACC_ADDR, 0x04, &reg);
      if (ret == HAL_OK)
        *val = (int32_t)reg * (1000000 * INA219_ACC_CURRENT_LSB);
    }
    break;

    case SENSOR_HV_TEST_CURRENT: /* uA */
    {
      int16_t reg;
      ret = ina219_read_reg(INA219_HV_ADDR, 0x04, &reg);
      if (ret == HAL_OK)
        *val = (int32_t)reg * (1000000 * INA219_HV_CURRENT_LSB);
    }
    break;
#endif

    case SENSOR_BATT_CURRENT: /* A x10 */
    {
      uint16_t tmp;
      ret = MX_ADC1_Get_Sample_Avg(ADC_BATT_CURR, &tmp);
#ifdef TARGET_CCS2
      if (ret == HAL_OK)
        *val = ((int32_t)tmp - ibatt_zero) * 1000 / 3423;
#else
      if (ret == HAL_OK)
        *val = -((int32_t)tmp - ibatt_zero) * 1000 / 2500;
#endif
      //printf("\nBatt Curr ADC: %d (Zero: %d) %ld mA\n", tmp, ibatt_zero, *val * 100);
    }
    break;

    case SENSOR_EVSE_PP: /* mV */
    {
      uint16_t tmp;
      ret = MX_ADC1_Get_Sample_Avg(ADC_EVSE_PP, &tmp);
      if (ret == HAL_OK)
        *val = ((int32_t)tmp * 3300) / 4096;
    }
    break;

    case SENSOR_CP: /* RAWh << 16 | RAWl */
    {
#ifdef TARGET_CCS2
      uint16_t tmph, tmpl;
      ret = MX_ADC2_Get_Sample_Avgs(ADC2_CCS2_CP, &tmph, &tmpl);
      //printf("MX_ADC2_Get_Sample_Avgs(%d): %ld, %ld\n", ret, tmph, tmpl);
      if (ret == HAL_OK)
      {
        *val = tmph << 16 | tmpl;
      }
#endif
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