/** @file sensor.c
 *  @brief Module to interface with various sensors
 *
 *  This module provides a layer to collect sensor data and convert it
 *  to a usable unit value.
 *  Includes INA219 on I2C, MAX22530 ADC on SPI and built in ADC of the STM32.
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

static uint16_t midpoint = 2048;

/**
  * @brief  Perform initialisation of all sensors
  * @param  None
  * @retval bool true: Success, false: Failure
  */
bool sensor_init(void)
{
  bool ret = true;

  if (!MAX22530_Init())
  {
    printf("MAX22530:   Init Failed\n");
    ret = false;
  }

  if (!ina219_init(INA219_ACC_ADDR, 0.04096 / (INA219_ACC_CURRENT_LSB * INA219_ACC_SHUNT)))
  {
    printf("INA219 ACC: Init Failed\n");
    ret = false;
  }

  if (!ina219_init(INA219_HV_ADDR, 0.04096 / (INA219_HV_CURRENT_LSB * INA219_HV_SHUNT)))
  {
    printf("INA219 HV:  Init Failed\n");
    ret = false;
  }

  if (UINT32_MAX == MX_ADC1_Get_Sample(ADC_BATT_CURR))
  {
    printf("BATT CURR:  Init Failed\n");
    ret = false;
  }

  if (UINT32_MAX == MX_ADC1_Get_Sample(ADC_EVSE_PP))
  {
    printf("EVSE PP:    Init Failed\n");
    ret = false;
  }

  midpoint = MX_ADC1_Get_Sample(ADC_MIDPOINT);
  if (UINT32_MAX == midpoint)
  {
    printf("Midpoint:   Init Failed\n");
    ret = false;
  }

  return ret;
}

/**
  * @brief  Get value of specified sensor
  * @param  src Sensor to read
  * @retval uint32_t UINT32_MAX: Failure, Otherwise sensor value
  */
uint32_t sensor_get_value(SENSOR_SOURCE src)
{
  uint32_t ret = UINT32_MAX;

  switch (src)
  {
    case SENSOR_BATT_VOLTAGE:
    {
      uint16_t val;
      if (HAL_OK == MAX22530_read_register(MAX22530_ADC1, &val))
        ret = (uint32_t)val * MAX22530_VREF / 4096 * (5 + 1500) / 5 / 1000;
    }
    break;

    case SENSOR_ACC_CURRENT:
    {
      uint16_t reg;      
      ina219_read_reg(INA219_ACC_ADDR, 0x04, &reg);
      ret = reg * (1000000 * INA219_ACC_CURRENT_LSB);
    }
    break;

    case SENSOR_HV_TEST_CURRENT:
    {
      uint32_t i;
      uint32_t val;

      // Average 1000 current samples
      val = 0;
      for (i=0; i<1000; ++i)
      {
        uint16_t reg;
        ina219_read_reg(INA219_HV_ADDR, 0x04, &reg);  
        val += reg;
        HAL_Delay(1);
      }
      ret = val * (1000 * INA219_HV_CURRENT_LSB);
    }
    break;

    case SENSOR_BATT_CURRENT:
      ret = MX_ADC1_Get_Sample(ADC_BATT_CURR);
      ret = (ret * 3300) / 4096;
    break;

    case SENSOR_EVSE_PP:
      ret = MX_ADC1_Get_Sample(ADC_EVSE_PP);
      ret = (ret * 3300) / 4096;
    break;

    case SENSOR_MIDPOINT:
      ret = MX_ADC1_Get_Sample(ADC_MIDPOINT);
    break;

    default:
      printf("Sensor %d not implemnented yet\n", src);
    break;
  }

  return ret;
}