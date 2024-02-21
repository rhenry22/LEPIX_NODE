#ifndef _MODBUS_H_
#define _MODBUS_H_

#include <stdint.h>
#include <stdbool.h>

#define MB_RX_TIMEOUT_MS    (5)

enum
{
    MB_ERR_ILLEGAL_FUNCTION = 0x01,
    MB_ERR_ILLEGAL_ADDRESS = 0x02
};

typedef enum
{
    MB_READ_COIL            = 0x01,
    MB_READ_DISCRETE        = 0x02,
    MB_READ_HOLDING         = 0x03,
    MB_READ_INPUT           = 0x04,

    MB_WRITE_COIL           = 0x05,
    MB_WRITE_HOLDING        = 0x06
} MB_FUNC;

typedef void (*modbus_m_rx_cb)(uint8_t addr, MB_FUNC type, uint16_t reg, uint8_t *data, uint16_t len);
typedef void (*modbus_s_rx_cb)(uint8_t addr, MB_FUNC type, uint8_t *data, uint16_t len);
typedef void (*modbus_tx_cb)(void);

bool modbus_init(modbus_m_rx_cb m_rx_fn, modbus_s_rx_cb s_rx_fn, modbus_tx_cb tx_fn);
void modbus_process(uint8_t *data, uint16_t len);
void modbus_tx_complete(void);

void modbus_tx_begin(uint8_t addr, uint8_t func, uint8_t len);
void modbus_tx_uint8(uint8_t data);
void modbus_tx_uint16(uint16_t data);
void modbus_tx_float(float data);
void modbus_tx_end(void);

HAL_StatusTypeDef modbus_read(uint8_t addr, uint8_t fn, uint16_t reg);

#endif // _MODBUS_H_