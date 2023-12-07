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

typedef enum
{
  SENSOR_ACC_CURRENT,       // Current from 12V supply (uA)
  SENSOR_HV_TEST_CURRENT,   // Current into HV DCDC Test module (uA)

  SENSOR_BATT_VOLTAGE,      // HV Battery Voltage (V)
  SENSOR_BATT_CURRENT,      // HV Battery Current (mA)

  SENSOR_EVSE_PP,           // EVSE PP Voltage (mV)

  SENSOR_MIDPOINT,          // Midpoint for AC signals (mV)
  SENSOR_IAC,               // AC Current probe (Not used)

} SENSOR_SOURCE;

bool sensor_init(void);
uint32_t sensor_get_value(SENSOR_SOURCE src);

#endif /* __SENSOR_H__ */
