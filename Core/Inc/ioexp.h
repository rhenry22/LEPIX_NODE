#ifndef _IOEXP_H_
#define _IOEXP_H_

#include <stdint.h>
#include <stdbool.h>

#define IOEXP_RG_LED    (0x21)
#define IOEXP_BY_LED    (0x20)
#define IOEXP_IO        (0x22)

bool ioexp_init(uint8_t addr);
bool ioexp_set_direction(uint8_t addr, uint16_t dir);
bool ioexp_set_output(uint8_t addr, uint16_t out);
bool ioexp_get_input(uint8_t addr, uint16_t *in);

#endif /* _IOEXP_H_ */