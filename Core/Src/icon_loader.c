#include "icon_loader.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>

/* Cache RAM : 8 icônes × 2048 bytes = 16 KB */
static uint16_t icon_cache[ICON_COUNT][ICON_PIXELS];
static bool     icon_loaded[ICON_COUNT] = {false};

static const char *icon_filenames[ICON_COUNT] = {
    "icons/network.bmp",
    "icons/outputs.bmp",
    "icons/protocol.bmp",
    "icons/system.bmp",
    "icons/dhcp.bmp",
    "icons/static.bmp",
    "icons/artnet.bmp",
    "icons/sacn.bmp",
};

/* Load a single BMP RGB565 icon from SD into cache */
static bool load_bmp(IconID_t id)
{
    FIL  fil;
    UINT br;
    uint8_t header[BMP_HEADER_SIZE];

    if (f_open(&fil, icon_filenames[id], FA_READ) != FR_OK) {
        printf("[Icon] Cannot open %s\r\n", icon_filenames[id]);
        return false;
    }

    /* Read and skip BMP header */
    if (f_read(&fil, header, BMP_HEADER_SIZE, &br) != FR_OK || br < BMP_HEADER_SIZE) {
        printf("[Icon] Header read failed: %s\r\n", icon_filenames[id]);
        f_close(&fil);
        return false;
    }

    /* Validate BMP signature */
    if (header[0] != 'B' || header[1] != 'M') {
        printf("[Icon] Not a BMP: %s\r\n", icon_filenames[id]);
        f_close(&fil);
        return false;
    }

    /* Data offset from header bytes 10-13 */
    uint32_t data_offset = (uint32_t)header[10]
                         | ((uint32_t)header[11] << 8)
                         | ((uint32_t)header[12] << 16)
                         | ((uint32_t)header[13] << 24);

    /* Seek to pixel data if offset differs from expected */
    if (data_offset > BMP_HEADER_SIZE) {
        f_lseek(&fil, data_offset);
    }

    /* Read pixel data directly into cache */
    if (f_read(&fil, icon_cache[id], ICON_BYTES, &br) != FR_OK || br < ICON_BYTES) {
        printf("[Icon] Pixel read failed: %s (%u/%u bytes)\r\n",
               icon_filenames[id], br, ICON_BYTES);
        f_close(&fil);
        return false;
    }

    f_close(&fil);

    /* BMP stores rows bottom-up — flip vertically */
    uint16_t tmp[ICON_SIZE];
    for (int row = 0; row < ICON_SIZE / 2; row++) {
        memcpy(tmp,
               &icon_cache[id][row * ICON_SIZE],
               ICON_SIZE * 2);
        memcpy(&icon_cache[id][row * ICON_SIZE],
               &icon_cache[id][(ICON_SIZE - 1 - row) * ICON_SIZE],
               ICON_SIZE * 2);
        memcpy(&icon_cache[id][(ICON_SIZE - 1 - row) * ICON_SIZE],
               tmp,
               ICON_SIZE * 2);
    }

    icon_loaded[id] = true;
    printf("[Icon] Loaded %s\r\n", icon_filenames[id]);
    return true;
}

bool Icon_LoadAll(void)
{
    bool all_ok = true;
    for (int i = 0; i < ICON_COUNT; i++) {
        if (!load_bmp((IconID_t)i)) all_ok = false;
    }
    return all_ok;
}

bool Icon_IsLoaded(IconID_t id)
{
    if (id >= ICON_COUNT) return false;
    return icon_loaded[id];
}

void Icon_Draw(IconID_t id, uint16_t x, uint16_t y)
{
    if (id >= ICON_COUNT || !icon_loaded[id]) {
        /* Fallback: draw a gray placeholder square */
        ST7789_Fill(x, y, x + ICON_SIZE - 1, y + ICON_SIZE - 1, 0x8410);
        return;
    }
    ST7789_DrawImage(x, y, ICON_SIZE, ICON_SIZE, icon_cache[id]);
}