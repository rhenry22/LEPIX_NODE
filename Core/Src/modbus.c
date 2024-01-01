/** @file modbus.c
 *  @brief Simple Modbus slave implementation
 *
 *  This is the minimum needed to emulate one of the two
 *  Power meters supported by FoxESS inverters.
 *
 *  Copyright (c) 2023 ARTaylor.co.uk.
 *  All rights reserved.
 *
 *  @author Richard Taylor <richard@artaylor.co.uk>
 */
#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "modbus.h"

#define BUFFER_LEN  (16)

static uint8_t tx_index = 0;
static uint8_t tx_buffer[BUFFER_LEN];

static modbus_read_fn read_cb = NULL;
static uint8_t mb_addr = 0;

static const uint16_t modbus_crc_table[] = {
0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
0XC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
0XCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
0X0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
0XD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
0X1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
0X1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
0XD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
0XF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
0X3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
0X3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
0XFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
0X2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
0XEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
0XE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
0X2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
0XA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
0X6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
0X6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
0XAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
0X7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
0XBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
0XB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
0X7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
0X5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
0X9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
0X9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
0X5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
0X8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
0X4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
0X4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
0X8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

/**
  * @brief  Adds the byte to the tx buffer ready for CRC calculation
  * @param  data Byte of data to append.
  * @retval None
  */
static void modbus_tx_add_byte(uint8_t data)
{
  if (tx_index < BUFFER_LEN)
    tx_buffer[tx_index++] = data;
}

/**
  * @brief  Adds the byte to the tx buffer ready for CRC calculation
  * @param  data Buffer containing data
  * @param  len Length of data in bytes
  * @retval uint8_t resulting CRC
  */
static uint16_t modbus_calculate_crc(uint8_t *data, uint16_t len)
{
  uint8_t byte;
  uint16_t crc = 0xFFFF;

  while (len--)
  {
    byte = (*data++) ^ crc;
    crc >>= 8;
    crc ^= modbus_crc_table[byte];
  }
  return crc;
}

/**
  * @brief  Initialise Modbus module
  * @param  addr Modbus address to respond to
  * @param  read_fn Function callback to get reg data
  * @retval bool true: Success, false: Failure
  */
bool modbus_init(uint8_t addr, modbus_read_fn read_fn)
{
  read_cb = read_fn;
  mb_addr = addr;

  tx_index = 0;
  memset(tx_buffer, 0, BUFFER_LEN);

  HAL_UART_Setup_UART2();

  return true;
}

/**
  * @brief  Non ISR processing function.
  * @retval None
  */
void modbus_process(uint8_t *data, uint16_t len)
{
  // Rely on the timeout to signal the packet end
  if (data[0] == mb_addr)
  {
    // Check the CRC
    uint16_t crc = modbus_calculate_crc(&data[0], len);

    if (crc == 0)
    {
      // Good CRC, let our app know
      if (read_cb)
        read_cb(data[1], data[2] << 8 | data[3], data[4] << 8 | data[5]);
    }
  }
}

void modbus_resp_begin(uint8_t func, uint8_t len)
{
  tx_index = 0;
  memset(tx_buffer, 0, BUFFER_LEN);

  // Send our device address, function and data length
  modbus_tx_add_byte(mb_addr);
  modbus_tx_add_byte(func);
  modbus_tx_add_byte(len);
}

void modbus_resp_byte(uint8_t data)
{
  modbus_tx_add_byte(data);
}

void modbus_resp_float(float data)
{
  uint8_t *ptr = (uint8_t*)&data;

  modbus_tx_add_byte(ptr[3]);
  modbus_tx_add_byte(ptr[2]);
  modbus_tx_add_byte(ptr[1]);
  modbus_tx_add_byte(ptr[0]);
}

void modbus_resp_end(void)
{
  uint16_t crc = modbus_calculate_crc(&tx_buffer[0], tx_index);

  modbus_tx_add_byte(crc & 0xff);
  modbus_tx_add_byte((crc >> 8) & 0xff);

  HAL_UART_Transmit_DMA(&huart2, &tx_buffer[0], tx_index);
}
