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
 *  @bug No known bugs.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "sensor.h"
#include "max22530.h"
#include "ina219.h"
#include "adc.h"

#define INA219_ACC_ADDR         (0x41)
#define INA219_ACC_SHUNT        (0.005)  // 5mR Shunt resistor
#define INA219_ACC_CURRENT_LSB  (0.001)  // 1mA per LSB

#define INA219_HV_ADDR          (0x40)
#define INA219_HV_SHUNT         (0.1)  // 100mR Shunt resistor
#define INA219_HV_CURRENT_LSB   (0.000050)  // 50uA per LSB

static uint16_t ibatt_zero = 2048;

/**
  * @brief  Perform initialisation of all sensors
  * @param  None
  * @retval bool true: Success, false: Failure
  */
bool sensor_init(void)
{
  bool ret = true;
  uint16_t val;

  if (!MAX22530_Init())
  {
    ret = false;
  }

  if (!ina219_init(INA219_ACC_ADDR, 0.04096 / (INA219_ACC_CURRENT_LSB * INA219_ACC_SHUNT)))
  {
    ret = false;
  }

  if (!ina219_init(INA219_HV_ADDR, 0.04096 / (INA219_HV_CURRENT_LSB * INA219_HV_SHUNT)))
  {
    ret = false;
  }

  if (HAL_OK != MX_ADC1_Get_Sample_Avg(ADC_BATT_CURR, &ibatt_zero) || ibatt_zero < 100)
  {
    ret = false;
  }

  if (HAL_OK != MX_ADC1_Get_Sample(ADC_EVSE_PP, &val))
  {
    ret = false;
  }

  return ret;
}

/**
  * @brief  Zero the battery current sensor
  * @retval HAL_StatusTypeDef HAL_OK on success
  */
HAL_StatusTypeDef sensor_zero_ibatt(void)
{
  return MX_ADC1_Get_Sample_Avg(ADC_BATT_CURR, &ibatt_zero);
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

  switch (src)
  {
    case SENSOR_BATT_VOLTAGE: // V x10
    {
      uint16_t tmp;
      ret = MAX22530_read_register(MAX22530_ADC1, &tmp);
      if (ret == HAL_OK)
        *val = (int32_t)tmp * MAX22530_VREF / 4096 * (4.7 + 1500) * 10 / 4.7 / 1000;
    }
    break;

    case SENSOR_ACC_CURRENT: // uA
    {
      int16_t reg;
      ret = ina219_read_reg(INA219_ACC_ADDR, 0x04, &reg);
      if (ret == HAL_OK)
        *val = (int32_t)reg * (1000000 * INA219_ACC_CURRENT_LSB);
    }
    break;

    case SENSOR_HV_TEST_CURRENT: // uA
    {
      int16_t reg;
      ret = ina219_read_reg(INA219_HV_ADDR, 0x04, &reg);
      if (ret == HAL_OK)
        *val = (int32_t)reg * (1000000 * INA219_HV_CURRENT_LSB);
    }
    break;

    case SENSOR_BATT_CURRENT: // A x10
    {
      uint16_t tmp;
      ret = MX_ADC1_Get_Sample_Avg(ADC_BATT_CURR, &tmp);
      if (ret == HAL_OK)
        *val = ((int32_t)tmp - ibatt_zero) * 1000 / 2095;
    }
    break;

    case SENSOR_EVSE_PP: // mV
    {
      uint16_t tmp;
      ret = MX_ADC1_Get_Sample(ADC_EVSE_PP, &tmp);
      if (ret == HAL_OK)
        *val = ((int32_t)tmp * 3300) / 4096;
    }
    break;

    default:
      assert_failed((uint8_t*)__FILE__, __LINE__);
    break;
  }

  return ret;
}