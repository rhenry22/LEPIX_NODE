/******************************************************************************/
/* MAX22530      Programming Guide Functions                     */
/******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "i2c.h"
#include "main.h"
#include "ina219.h"

#define I2C_TIMEOUT (100)

#define CONFIG_RST    (1 << 15)
#define CONFIG_BRNG   (1 << 13)
#define CONFIG_PG1    (1 << 12)
#define CONFIG_PG0    (1 << 11)
#define CONFIG_BADC4  (1 << 10)
#define CONFIG_BADC3  (1 << 9)
#define CONFIG_BADC2  (1 << 8)
#define CONFIG_BADC1  (1 << 7)
#define CONFIG_SADC4  (1 << 6)
#define CONFIG_SADC3  (1 << 5)
#define CONFIG_SADC2  (1 << 4)
#define CONFIG_SADC1  (1 << 3)
#define CONFIG_MODE3  (1 << 2)
#define CONFIG_MODE2  (1 << 1)
#define CONFIG_MODE1  (1 << 0)


/**
  * @brief  Write to INA219 register
  * @param  addr Chip I2C address
  * @param  reg Register Address
  * @param  data Data to write
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef ina219_write_reg(uint8_t addr, uint8_t reg, uint16_t data)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t buf[2];

  buf[0] = (data >> 8) & 0xff;
  buf[1] = data & 0xff;
  if (HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, I2C_TIMEOUT) != HAL_OK)
  {
    ret = HAL_I2C_GetError(&hi2c1);
  }

  return ret;
}

/**
  * @brief  Read from INA219 register
  * @param  addr Chip I2C address
  * @param  reg Register Address
  * @param  data pointer to data storage for read
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef ina219_read_reg(uint8_t addr, uint8_t reg, uint16_t *data)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t buf[2];

  if (HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, I2C_TIMEOUT) != HAL_OK)
  {
    ret = HAL_I2C_GetError(&hi2c1);
  }
  else
  {
    *data = buf[0] << 8;
    *data |= buf[1];
  }

  return ret;
}


/**
  * @brief  Initialise an INA219
  * @param  addr Chip I2C address
  * @param  cal Calibration value
  * @retval bool true: Success, false: Failure
  */
bool ina219_init(uint8_t addr, uint16_t cal)
{
  uint16_t data;
  bool ret = false;
  HAL_StatusTypeDef halRet;

  halRet = ina219_read_reg(addr, 0x00, &data);
  if (halRet == HAL_OK)
  {
    // Configure PGA to +/- 40mV and 16V bus voltage range
    data &= ~(CONFIG_BRNG | CONFIG_PG1 | CONFIG_PG0);
    halRet = ina219_write_reg(addr, 0x00, data);
  }

  if (halRet == HAL_OK)
  {
    halRet = ina219_write_reg(addr, 0x05, cal);
  }

  if (halRet == HAL_OK)
    ret = true;

  return ret;
}