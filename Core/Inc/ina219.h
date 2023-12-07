#ifndef _INA219_H_
#define _INA219_H_

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

bool ina219_init(uint8_t addr, uint16_t cal);
HAL_StatusTypeDef ina219_read_reg(uint8_t addr, uint8_t reg, int16_t *data);
HAL_StatusTypeDef ina219_write_reg(uint8_t addr, uint8_t reg, uint16_t data);

#endif /* _INA219_H_ */