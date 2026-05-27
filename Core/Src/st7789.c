#include "st7789.h"

// ── SPI bit-bang ──────────────────────────────────────────────
static inline void SPI_WriteByte(uint8_t byte)
{
    for (int i = 7; i >= 0; i--)
    {
        if (byte & (1 << i)) LCD_MOSI_HIGH();
        else                  LCD_MOSI_LOW();
        LCD_SCK_HIGH();
        LCD_SCK_LOW();
    }
}

// ── Primitives ST7789 ─────────────────────────────────────────
static void WriteCmd(uint8_t cmd)
{
    LCD_DC_LOW();   // Command mode
    LCD_CS_LOW();
    SPI_WriteByte(cmd);
    LCD_CS_HIGH();
}

static void WriteData(uint8_t data)
{
    LCD_DC_HIGH();  // Data mode
    LCD_CS_LOW();
    SPI_WriteByte(data);
    LCD_CS_HIGH();
}

static void WriteData16(uint16_t data)
{
    LCD_DC_HIGH();
    LCD_CS_LOW();
    SPI_WriteByte(data >> 8);
    SPI_WriteByte(data & 0xFF);
    LCD_CS_HIGH();
}

// ── Initialisation ────────────────────────────────────────────
void ST7789_Init(void)
{
    LCD_RST_LOW();
    HAL_Delay(15);
    LCD_RST_HIGH();
    HAL_Delay(120);

    WriteCmd(0x11); // Sleep Out
    HAL_Delay(120);

    WriteCmd(0x36); // MADCTL
    WriteData(0x00);

    WriteCmd(0x3A); // Pixel format RGB565
    WriteData(0x55);

    // CASET : colonnes 0 → 239
    WriteCmd(0x2A);
    WriteData(0x00); WriteData(0x00);
    WriteData(0x00); WriteData(0xEF); // 239

    // RASET : lignes 0 → 319
    WriteCmd(0x2B);
    WriteData(0x00); WriteData(0x00);
    WriteData(0x01); WriteData(0x3F); // 319

    WriteCmd(0xB2); // Porch Setting
    WriteData(0x0C); WriteData(0x0C);
    WriteData(0x00); WriteData(0x33); WriteData(0x33);

    WriteCmd(0xB7); // Gate Control
    WriteData(0x35);

    WriteCmd(0xBB); // VCOM
    WriteData(0x19);

    WriteCmd(0xC0); // LCM Control
    WriteData(0x2C);

    WriteCmd(0xC2);
    WriteData(0x01);

    WriteCmd(0xC3); // VRH
    WriteData(0x12);

    WriteCmd(0xC4); // VDV
    WriteData(0x20);

    WriteCmd(0xC6); // 60Hz
    WriteData(0x0F);

    WriteCmd(0xD0); // Power Control
    WriteData(0xA4); WriteData(0xA1);

    WriteCmd(0xE0); // Gamma +
    WriteData(0xD0); WriteData(0x04); WriteData(0x0D);
    WriteData(0x11); WriteData(0x13); WriteData(0x2B);
    WriteData(0x3F); WriteData(0x54); WriteData(0x4C);
    WriteData(0x18); WriteData(0x0D); WriteData(0x0B);
    WriteData(0x1F); WriteData(0x23);

    WriteCmd(0xE1); // Gamma -
    WriteData(0xD0); WriteData(0x04); WriteData(0x0C);
    WriteData(0x11); WriteData(0x13); WriteData(0x2C);
    WriteData(0x3F); WriteData(0x44); WriteData(0x51);
    WriteData(0x2F); WriteData(0x1F); WriteData(0x1F);
    WriteData(0x20); WriteData(0x23);

    WriteCmd(0x21); // Inversion ON
    WriteCmd(0x29); // Display ON
    HAL_Delay(10);

    LCD_BLK_HIGH();
}
// ── API publique ──────────────────────────────────────────────
void ST7789_SetCursor(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    WriteCmd(0x2A); // Column Address Set
    WriteData(x0 >> 8); WriteData(x0 & 0xFF);
    WriteData(x1 >> 8); WriteData(x1 & 0xFF);

    WriteCmd(0x2B); // Row Address Set
    WriteData(y0 >> 8); WriteData(y0 & 0xFF);
    WriteData(y1 >> 8); WriteData(y1 & 0xFF);

    WriteCmd(0x2C); // Memory Write
}

void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    ST7789_SetCursor(x, y, x, y);
    WriteData16(color);
}

void ST7789_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    ST7789_SetCursor(x, y, x + w - 1, y + h - 1);
    LCD_DC_HIGH();
    LCD_CS_LOW();
    for (uint32_t i = 0; i < (uint32_t)w * h; i++)
    {
        SPI_WriteByte(color >> 8);
        SPI_WriteByte(color & 0xFF);
    }
    LCD_CS_HIGH();
}

void ST7789_FillScreen(uint16_t color)
{
    ST7789_FillRect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, color);
}