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

typedef void (*modbus_read_fn)(MB_FUNC type, uint16_t reg, uint16_t len);

bool modbus_init(uint8_t addr, modbus_read_fn read_fn);
void modbus_process(uint8_t *data, uint16_t len);

void modbus_resp_begin(uint8_t func, uint8_t len);
void modbus_resp_byte(uint8_t data);
void modbus_resp_float(float data);
void modbus_resp_end(void);

#endif // _MODBUS_H_