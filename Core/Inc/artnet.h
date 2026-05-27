/* artnet.h */
#ifndef ARTNET_H
#define ARTNET_H

#include "main.h"   /* UART_HandleTypeDef, HAL, etc. */
#include <stdarg.h>
#include <stdint.h>

#define ARTNET_PORT_STR "6454"

/**
 * @brief Callback appelé à chaque trame DMX reçue.
 * @param universe  Numéro d'univers Art-Net (0..32767)
 * @param data      Pointeur vers les données DMX (512 octets max)
 * @param len       Nombre de canaux valides dans data
 */
typedef void (*artnet_dmx_cb_t)(uint16_t universe, uint8_t *data, uint16_t len);

void artnet_init(void);
void artnet_set_callback(artnet_dmx_cb_t cb);

#endif /* ARTNET_H */