/******************************************************************************/
/* MAX22530      Programming Guide Functions                     */
/******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "spi.h"
#include "main.h"
#include "max22530.h"

/**
  * @brief  Read from a MAX22530 Register
  * @param  reg Register address
  * @param  cal Calibration value
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef MAX22530_read_register(uint8_t reg, uint16_t *data)
{
  HAL_StatusTypeDef ret;

  uint8_t header = (reg << 2);
  uint8_t buf[2];

  HAL_GPIO_WritePin(SPI1_BATT_CS__GPIO_Port, SPI1_BATT_CS__Pin, GPIO_PIN_RESET);
  
  buf[0] = header;
  ret = HAL_SPI_TransmitReceive(&hspi1, buf, buf, 1, 100);
  if (ret == HAL_OK)
  {
    ret = HAL_SPI_TransmitReceive(&hspi1, buf, buf, 2, 100);
    if (data)
      *data = buf[0] << 8 | buf[1];
  }

  HAL_GPIO_WritePin(SPI1_BATT_CS__GPIO_Port, SPI1_BATT_CS__Pin, GPIO_PIN_SET);

  return ret;
}


/**
  * @brief  Write to a MAX22530 Register
  * @param  reg Register address
  * @param  data Value to write
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef MAX22530_write_register(uint8_t reg, uint16_t data)
{
  HAL_StatusTypeDef ret;
  uint8_t buf[2];

  HAL_GPIO_WritePin(SPI1_BATT_CS__GPIO_Port, SPI1_BATT_CS__Pin, GPIO_PIN_RESET);

  buf[0] = ((reg << 2) + (1 << 1));
  ret = HAL_SPI_TransmitReceive(&hspi1, buf, buf, 1, 100);
  if (ret == HAL_OK)
  {
    buf[0] = (data >> 8) & 0xff;
    buf[1] = data & 0xff;
    ret = HAL_SPI_TransmitReceive(&hspi1, buf, buf, 2, 100);
  }
  HAL_GPIO_WritePin(SPI1_BATT_CS__GPIO_Port, SPI1_BATT_CS__Pin, GPIO_PIN_SET);
  
  if (ret != HAL_OK)
  {
    printf("%s: Error (%d)\n", __func__, ret);
  }

  return ret;
}


/**
  * @brief  Initialise MAX22530
  * @param  None
  * @retval bool true: Success, false: Failure
  */
bool MAX22530_Init(void)
{
  bool ret = true;
  uint16_t val;
  HAL_StatusTypeDef s;

  // Clear any CS glitches
  HAL_GPIO_WritePin(SPI1_BATT_CS__GPIO_Port, SPI1_BATT_CS__Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI1_BATT_CS__GPIO_Port, SPI1_BATT_CS__Pin, GPIO_PIN_SET);
  HAL_Delay(1);

  s = MAX22530_read_register(MAX22530_PROD_ID, &val);
  if (s != HAL_OK || val != MAX22530_ID)
  {
    ret = false;
  }

  s = MAX22530_read_register(MAX22530_CONTROL, &val);
  if (s != HAL_OK || val > 32767)
  {
    ret = false;
  }

  return ret;
}
