/* artnet.h */
#ifndef ARTNET_H
#define ARTNET_H

#include "main.h"   /* UART_HandleTypeDef, HAL, etc. */
#include <stdarg.h>

#define ARTNET_PORT_STR "6454"

void artnet_init(void);

#endif /* ARTNET_H */