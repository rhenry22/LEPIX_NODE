/** @file sensor.h
 *  @brief Function prototypes for sensor interface
 *
 *  @author Richard Taylor <richard@artaylor.co.uk>
 *  @bug No known bugs.
 */

#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef enum
{
#ifdef ENABLE_INA219
  /* INA219 I2C Sensors */
  SENSOR_ACC_CURRENT,       /* Current from 12V supply (uA) */
  SENSOR_HV_TEST_CURRENT,   /* Current into HV DCDC Test module (uA) */
#endif

#ifdef ENABLE_MAX22530
  /* MAX22530AWE+ Inputs */
  SENSOR_BATT_VOLTAGE,      /* HV Battery Voltage (V x10) */
  SENSOR_INV_VOLTAGE,       /* HV Inverter Voltage (V x10) */
#endif

  /* On Chip ADC */
  SENSOR_BATT_CURRENT,      /* HV Battery Current (A x10) */
  SENSOR_EVSE_PP,           /* EVSE PP Voltage (mV) */
  SENSOR_CP                 /* CCS2 CP Voltage (mV) */
} SENSOR_SOURCE;

bool sensor_init(void);
HAL_StatusTypeDef sensor_zero_ibatt(void);
HAL_StatusTypeDef sensor_get_value(SENSOR_SOURCE src, int32_t *val);

#endif /* __SENSOR_H__ */
