/** @file ioexp.c
 *  @brief Functions to control a TCA9535 I2C GPIO Expander
 *
 *  @author Richard Taylor <richard@artaylor.co.uk>
 */

#include <stdint.h>

#include "i2c.h"
#include "ioexp.h"

#define I2C_TIMEOUT (100)

enum
{
  IOEXP_INPUT_PORT0 = 0,
  IOEXP_INPUT_PORT1,
  IOEXP_OUTPUT_PORT0,
  IOEXP_OUTPUT_PORT1,
  IOEXP_POLARITY_PORT0,
  IOEXP_POLARITY_PORT1,
  IOEXP_CONFIG_PORT0,
  IOEXP_CONFIG_PORT1
};


/**
  * @brief  Initialise the GPIO Expander
  * @param  addr Chip I2C address
  * @retval bool true: Success, false: Failure
  */
bool ioexp_init(uint8_t addr)
{
  uint16_t inputs;
  return ioexp_get_input(addr, &inputs);
}

/**
  * @brief  Set port direction bits
  * @param  addr Chip I2C address
  * @param  dir Direction bits (1: input, 0: output)
  * @retval bool true: Success, false: Failure
  */
bool ioexp_set_direction(uint8_t addr, uint16_t dir)
{
  if (HAL_OK == HAL_I2C_Mem_Write(&hi2c1, addr << 1 | 0x00, IOEXP_CONFIG_PORT0,
                                  I2C_MEMADD_SIZE_8BIT, (uint8_t*)&dir, 2,
                                  I2C_TIMEOUT))
  {
    return true;
  }
  return false;
}

/**
  * @brief  Set port output bits
  * @param  addr Chip I2C address
  * @param  out Output bits (1: high, 0: low)
  * @retval bool true: Success, false: Failure
  */
bool ioexp_set_output(uint8_t addr, uint16_t out)
{
  if (HAL_OK == HAL_I2C_Mem_Write(&hi2c1, addr << 1 | 0x00, IOEXP_OUTPUT_PORT0,
                                  I2C_MEMADD_SIZE_8BIT, (uint8_t*)&out, 2,
                                  I2C_TIMEOUT))
  {
    return true;
  }
  return false;
}

/**
  * @brief  Get port input bits
  * @param  addr Chip I2C address
  * @param  in Input bits (1: high, 0: low)
  * @retval bool true: Success, false: Failure
  */
bool ioexp_get_input(uint8_t addr, uint16_t *in)
{
  if (HAL_OK == HAL_I2C_Mem_Read(&hi2c1, addr << 1 | 0x01, IOEXP_INPUT_PORT0,
                                  I2C_MEMADD_SIZE_8BIT, (uint8_t*)in, 2,
                                  I2C_TIMEOUT))
  {
    return true;
  }
  return false;
}
