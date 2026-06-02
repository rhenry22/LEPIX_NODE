#ifndef ICON_LOADER_H
#define ICON_LOADER_H

#include <stdint.h>
#include <stdbool.h>
#include "st7789.h"

#define ICON_SIZE       32
#define ICON_PIXELS     (ICON_SIZE * ICON_SIZE)
#define ICON_BYTES      (ICON_PIXELS * 2)
#define ICON_COUNT      8
#define ICON_DIR        "icons/"

/* Icon index — matches filename on SD */
typedef enum {
    ICON_NETWORK  = 0,
    ICON_OUTPUTS  = 1,
    ICON_PROTOCOL = 2,
    ICON_SYSTEM   = 3,
    ICON_DHCP     = 4,
    ICON_STATIC   = 5,
    ICON_ARTNET   = 6,
    ICON_SACN     = 7,
} IconID_t;

/* BMP RGB565 header is 66 bytes for 16-bit BMP */
#define BMP_HEADER_SIZE 66

bool Icon_LoadAll(void);
void Icon_Draw(IconID_t id, uint16_t x, uint16_t y);
bool Icon_IsLoaded(IconID_t id);

#endif