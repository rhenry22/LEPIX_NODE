#ifndef _MAX22530_H_
#define _MAX22530_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/*MAX22530 Registers*/
#define MAX22530_PROD_ID           0x00
#define MAX22530_ADC1              0x01
#define MAX22530_ADC2              0x02
#define MAX22530_ADC3              0x03
#define MAX22530_ADC4              0x04
#define MAX22530_FADC1             0x05
#define MAX22530_FADC2             0x06
#define MAX22530_FADC3             0x07
#define MAX22530_FADC4             0x08
#define MAX22530_COUTHI1           0x09
#define MAX22530_COUTHI2           0x0a
#define MAX22530_COUTHI3           0x0b
#define MAX22530_COUTHI4           0x0c
#define MAX22530_COUTLO1           0x0d
#define MAX22530_COUTLO2           0x0e
#define MAX22530_COUTLO3           0x0f
#define MAX22530_COUTLO4           0x10
#define MAX22530_COUT_STATUS       0x11
#define MAX22530_INTERRUPT_STATUS  0x12
#define MAX22530_INTERRUPT_ENABLE  0x13
#define MAX22530_CONTROL           0x14


#define MAX22530_ID                0x81
#define MAX22530_VREF              1800


bool MAX22530_Init(void);
HAL_StatusTypeDef MAX22530_read_register(uint8_t reg, uint16_t *data);
HAL_StatusTypeDef MAX22530_write_register(uint8_t reg, uint16_t data);

#endif // _MAX22530_H_