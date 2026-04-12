#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef enum
{
  SENSOR_EVSE_PP,           /* EVSE PP Voltage (mV) */
  SENSOR_MAX
} SENSOR_SOURCE;

bool sensor_init(void);
HAL_StatusTypeDef sensor_get_value(SENSOR_SOURCE src, int32_t *val);

#endif /* __SENSOR_H__ */
