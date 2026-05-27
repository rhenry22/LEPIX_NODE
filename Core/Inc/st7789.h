#ifndef ST7789_H
#define ST7789_H

#include "main.h"
#include <stdint.h>

// ── Dimensions écran ───────────────────────────────
#define ST7789_WIDTH   240
#define ST7789_HEIGHT  320

// ── Macros GPIO BSRR ──────────────────────────────
#define LCD_SCK_HIGH()  (GPIOE->BSRR = LCD_SCK_Pin)
#define LCD_SCK_LOW()   (GPIOE->BSRR = (uint32_t)LCD_SCK_Pin << 16)

#define LCD_MOSI_HIGH() (GPIOE->BSRR = LCD_MOSI_Pin)
#define LCD_MOSI_LOW()  (GPIOE->BSRR = (uint32_t)LCD_MOSI_Pin << 16)

#define LCD_CS_HIGH()   (GPIOE->BSRR = LCD_CS_Pin)
#define LCD_CS_LOW()    (GPIOE->BSRR = (uint32_t)LCD_CS_Pin << 16)

#define LCD_DC_HIGH()   (GPIOE->BSRR = LCD_DC_Pin)
#define LCD_DC_LOW()    (GPIOE->BSRR = (uint32_t)LCD_DC_Pin << 16)

#define LCD_RST_HIGH()  (GPIOC->BSRR = LCD_RST_Pin)
#define LCD_RST_LOW()   (GPIOC->BSRR = (uint32_t)LCD_RST_Pin << 16)

#define LCD_BLK_HIGH()  (GPIOC->BSRR = LCD_BLK_Pin)
#define LCD_BLK_LOW()   (GPIOC->BSRR = (uint32_t)LCD_BLK_Pin << 16)

// ── Couleurs RGB565 ───────────────────────────────
#define ST7789_BLACK    0x0000
#define ST7789_WHITE    0xFFFF
#define ST7789_RED      0xF800
#define ST7789_GREEN    0x07E0
#define ST7789_BLUE     0x001F
#define ST7789_YELLOW   0xFFE0
#define ST7789_CYAN     0x07FF
#define ST7789_MAGENTA  0xF81F

// ── Prototypes ────────────────────────────────────
void ST7789_Init(void);
void ST7789_FillScreen(uint16_t color);
void ST7789_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7789_SetCursor(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

#endif