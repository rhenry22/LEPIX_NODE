#include "menu_screens.h"
#include "fonts.h"
#include <stdio.h>
#include <string.h>

/* ── Primitives ────────────────────────────────────────── */

static void draw_header(const char *title)
{
    ST7789_Fill(0, 0, MENU_W - 1, HEADER_H - 1, COL_HEADER);
    ST7789_WriteString(ITEM_PAD, (HEADER_H - 18) / 2,
                       title, Font_11x18, COL_ACCENT, COL_HEADER);
    /* separator line */
    ST7789_Fill(0, HEADER_H - 1, MENU_W - 1, HEADER_H - 1, COL_SEPARATOR);
}

static void draw_footer(const char *hint)
{
    int y = LCD_H - FOOTER_H;
    ST7789_Fill(0, y, MENU_W - 1, LCD_H - 1, COL_HEADER);
    ST7789_Fill(0, y, MENU_W - 1, y, COL_SEPARATOR);
    ST7789_WriteString(ITEM_PAD, y + 4, hint, Font_7x10, COL_TEXT_DIM, COL_HEADER);
}

static void draw_item(int8_t row, const char *label, IconID_t icon,
                       bool selected)
{
    int y   = HEADER_H + row * ITEM_H;
    uint16_t bg  = selected ? COL_SELECTED : COL_BG;
    uint16_t fg  = selected ? COL_TEXT     : COL_TEXT_DIM;

    ST7789_Fill(0, y, MENU_W - 1, y + ITEM_H - 1, bg);

    /* Icon */
    Icon_Draw(icon, ITEM_PAD, y + (ITEM_H - ICON_SIZE) / 2);

    /* Label */
    ST7789_WriteString(ITEM_PAD + ITEM_ICON_W + ITEM_PAD,
                       y + (ITEM_H - 18) / 2,
                       label, Font_11x18, fg, bg);

    /* Selection indicator */
    if (selected) {
        ST7789_Fill(MENU_W - 4, y, MENU_W - 1, y + ITEM_H - 1, COL_ACCENT);
    }

    /* Row separator */
    ST7789_Fill(0, y + ITEM_H - 1, MENU_W - 1, y + ITEM_H - 1, COL_SEPARATOR);
}

static void draw_value_row(int y, const char *label, const char *value,
                            bool selected, bool editing)
{
    uint16_t bg = selected ? COL_SELECTED : COL_BG;
    uint16_t fg = selected ? COL_TEXT     : COL_TEXT_DIM;
    uint16_t vg = editing  ? COL_ACCENT   : fg;

    ST7789_Fill(0, y, MENU_W - 1, y + ITEM_H - 1, bg);
    ST7789_WriteString(ITEM_PAD, y + 4,  label, Font_7x10, fg, bg);
    ST7789_WriteString(ITEM_PAD, y + 20, value, Font_11x18, vg, bg);
    ST7789_Fill(0, y + ITEM_H - 1, MENU_W - 1, y + ITEM_H - 1, COL_SEPARATOR);
}

/* ── Screens ───────────────────────────────────────────── */

void MenuScreen_Main(int8_t cursor, const MenuItem_t *items,
                     uint8_t count, int8_t scroll)
{
    ST7789_Fill_Color(COL_BG);
    draw_header("LEPIX NODE");

    uint8_t visible = (uint8_t)((LCD_H - HEADER_H - FOOTER_H) / ITEM_H);
    for (uint8_t i = 0; i < visible && (i + scroll) < count; i++) {
        draw_item(i, items[i + scroll].label,
                  items[i + scroll].icon,
                  (i + scroll) == cursor);
    }
    draw_footer("Rotate:nav  Press:enter  2xPress:save");
}

void MenuScreen_List(const char *title, const MenuItem_t *items,
                     uint8_t count, int8_t cursor)
{
    ST7789_Fill_Color(COL_BG);
    draw_header(title);
    for (uint8_t i = 0; i < count; i++) {
        draw_item(i, items[i].label, items[i].icon, i == cursor);
    }
    draw_footer("Press:select  LongPress:back");
}

void MenuScreen_DHCP(void)
{
    DeviceConfig_t *cfg = Config_Get();
    ST7789_Fill_Color(COL_BG);
    draw_header("DHCP mode");

    int y = HEADER_H + ITEM_PAD;
    ST7789_WriteString(ITEM_PAD, y,
        "DHCP will assign an IP", Font_7x10, COL_TEXT, COL_BG);
    y += 16;
    ST7789_WriteString(ITEM_PAD, y,
        "address automatically.", Font_7x10, COL_TEXT, COL_BG);
    y += 30;

    uint16_t bg = COL_SELECTED;
    ST7789_Fill(ITEM_PAD, y, MENU_W - ITEM_PAD - 1, y + 32, bg);
    ST7789_WriteString(ITEM_PAD + 8, y + 6,
        (cfg->net_mode == NET_DHCP) ? "[x] DHCP enabled" : "[ ] DHCP disabled",
        Font_11x18, COL_TEXT, bg);

    draw_footer("Press:enable  LongPress:back");
}

void MenuScreen_StaticIP(int8_t cursor)
{
    DeviceConfig_t *cfg = Config_Get();
    ST7789_Fill_Color(COL_BG);
    draw_header("Static IP");

    char buf[24];
    int y = HEADER_H;

    snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
             cfg->ip[0], cfg->ip[1], cfg->ip[2], cfg->ip[3]);
    draw_value_row(y, "IP address", buf, cursor == 0, false);
    y += ITEM_H;

    snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
             cfg->netmask[0], cfg->netmask[1],
             cfg->netmask[2], cfg->netmask[3]);
    draw_value_row(y, "Subnet mask", buf, cursor == 1, false);
    y += ITEM_H;

    snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
             cfg->gateway[0], cfg->gateway[1],
             cfg->gateway[2], cfg->gateway[3]);
    draw_value_row(y, "Gateway", buf, cursor == 2, false);
    y += ITEM_H;

    snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
             cfg->dns[0], cfg->dns[1], cfg->dns[2], cfg->dns[3]);
    draw_value_row(y, "DNS", buf, cursor == 3, false);

    draw_footer("Press:edit  LongPress:back");
}

/* IP field editor state */
static uint8_t ip_octet_cursor = 0;
static bool    ip_editing_octet = false;

void MenuScreen_IPEditor_Draw(ScreenID_t field)
{
    DeviceConfig_t *cfg = Config_Get();
    uint8_t *ip_field;
    const char *title;

    switch (field) {
    case SCR_NET_EDIT_IP:   ip_field = cfg->ip;      title = "Edit IP address"; break;
    case SCR_NET_EDIT_MASK: ip_field = cfg->netmask; title = "Edit Subnet mask"; break;
    case SCR_NET_EDIT_GW:   ip_field = cfg->gateway; title = "Edit Gateway"; break;
    case SCR_NET_EDIT_DNS:  ip_field = cfg->dns;     title = "Edit DNS"; break;
    default: return;
    }

    ST7789_Fill_Color(COL_BG);
    draw_header(title);

    int y = HEADER_H + 20;
    char parts[4][8];
    for (int i = 0; i < 4; i++)
        snprintf(parts[i], sizeof(parts[i]), "%3u", ip_field[i]);

    /* Draw each octet */
    for (int i = 0; i < 4; i++) {
        int x   = ITEM_PAD + i * 56;
        bool sel = (i == ip_octet_cursor);
        bool ed  = sel && ip_editing_octet;
        uint16_t bg = ed  ? COL_ACCENT   :
                      sel ? COL_SELECTED : COL_BG;
        uint16_t fg = ed  ? COL_BG       : COL_TEXT;

        ST7789_Fill(x, y, x + 50, y + 36, bg);
        ST7789_WriteString(x + 4, y + 8, parts[i], Font_16x26, fg, bg);

        if (i < 3) {
            ST7789_WriteString(x + 52, y + 12, ".", Font_11x18,
                               COL_TEXT_DIM, COL_BG);
        }
    }

    y += 60;
    ST7789_WriteString(ITEM_PAD, y,
        ip_editing_octet ? "Rotate:change value" : "Rotate:move  Press:edit",
        Font_7x10, COL_TEXT_DIM, COL_BG);

    draw_footer("LongPress:back & save");
}

void MenuScreen_IPEditor_Handle(ScreenID_t field, EncoderEvent_t evt)
{
    DeviceConfig_t *cfg = Config_Get();
    uint8_t *ip_field;
    switch (field) {
    case SCR_NET_EDIT_IP:   ip_field = cfg->ip;      break;
    case SCR_NET_EDIT_MASK: ip_field = cfg->netmask; break;
    case SCR_NET_EDIT_GW:   ip_field = cfg->gateway; break;
    case SCR_NET_EDIT_DNS:  ip_field = cfg->dns;     break;
    default: return;
    }

    if (!ip_editing_octet) {
        if (evt == ENC_EVENT_CW)  ip_octet_cursor = (ip_octet_cursor + 1) % 4;
        if (evt == ENC_EVENT_CCW) ip_octet_cursor = (ip_octet_cursor + 3) % 4;
        if (evt == ENC_EVENT_SHORT_PRESS) ip_editing_octet = true;
    } else {
        if (evt == ENC_EVENT_CW)
            ip_field[ip_octet_cursor] = (ip_field[ip_octet_cursor] + 1) % 256;
        if (evt == ENC_EVENT_CCW)
            ip_field[ip_octet_cursor] = (ip_field[ip_octet_cursor] + 255) % 256;
        if (evt == ENC_EVENT_SHORT_PRESS) ip_editing_octet = false;
    }
}

void MenuScreen_Outputs(int8_t cursor)
{
    DeviceConfig_t *cfg = Config_Get();
    ST7789_Fill_Color(COL_BG);
    draw_header("Outputs");

    for (int i = 0; i < MAX_OUTPUTS; i++) {
        int y = HEADER_H + i * ITEM_H;
        bool sel = (i == cursor);
        uint16_t bg = sel ? COL_SELECTED : COL_BG;
        uint16_t fg = sel ? COL_TEXT     : COL_TEXT_DIM;

        ST7789_Fill(0, y, MENU_W - 1, y + ITEM_H - 1, bg);

        char label[32];
        snprintf(label, sizeof(label), "Output %d", i + 1);
        ST7789_WriteString(ITEM_PAD, y + 4, label, Font_11x18, fg, bg);

        char info[32];
        snprintf(info, sizeof(info), "Univ:%u  %uA  %uLED",
                 cfg->outputs[i].universe,
                 cfg->outputs[i].max_current_A,
                 cfg->outputs[i].led_count);
        ST7789_WriteString(ITEM_PAD, y + 24, info, Font_7x10, COL_TEXT_DIM, bg);

        /* Enabled indicator */
        uint16_t dot = cfg->outputs[i].enabled ? COL_ACCENT : 0xF800;
        ST7789_Fill(MENU_W - 12, y + 16, MENU_W - 4, y + 24, dot);

        ST7789_Fill(0, y + ITEM_H - 1, MENU_W - 1, y + ITEM_H - 1, COL_SEPARATOR);
    }
    draw_footer("Press:configure  LongPress:back");
}

/* Output detail fields */
#define OUT_FIELD_ENABLED   0
#define OUT_FIELD_UNIVERSE  1
#define OUT_FIELD_LEDS      2
#define OUT_FIELD_CURRENT   3
#define OUT_FIELD_COUNT     4

void MenuScreen_OutputDetail(uint8_t idx, int8_t cursor, bool edit_mode)
{
    DeviceConfig_t *cfg = Config_Get();
    OutputConfig_t *out = &cfg->outputs[idx];

    ST7789_Fill_Color(COL_BG);
    char title[16];
    snprintf(title, sizeof(title), "Output %u", idx + 1);
    draw_header(title);

    int y = HEADER_H;
    char val[16];

    /* Enabled */
    draw_value_row(y, "Status",
        out->enabled ? "Enabled" : "Disabled",
        cursor == OUT_FIELD_ENABLED,
        edit_mode && cursor == OUT_FIELD_ENABLED);
    y += ITEM_H;

    /* Universe */
    snprintf(val, sizeof(val), "%u", out->universe);
    draw_value_row(y, "Universe (Art-Net/sACN)", val,
        cursor == OUT_FIELD_UNIVERSE,
        edit_mode && cursor == OUT_FIELD_UNIVERSE);
    y += ITEM_H;

    /* LED count */
    snprintf(val, sizeof(val), "%u pixels", out->led_count);
    draw_value_row(y, "LED count", val,
        cursor == OUT_FIELD_LEDS,
        edit_mode && cursor == OUT_FIELD_LEDS);
    y += ITEM_H;

    /* Max current */
    snprintf(val, sizeof(val), "%u A  (max 5A)", out->max_current_A);
    draw_value_row(y, "Max current", val,
        cursor == OUT_FIELD_CURRENT,
        edit_mode && cursor == OUT_FIELD_CURRENT);

    /* Current bar */
    y += ITEM_H + 4;
    int bar_w = (out->max_current_A * (MENU_W - ITEM_PAD * 2)) / 5;
    ST7789_Fill(ITEM_PAD, y, MENU_W - ITEM_PAD - 1, y + 8, COL_SEPARATOR);
    uint16_t bar_col = (out->max_current_A >= 4) ? 0xF800 :
                       (out->max_current_A >= 3) ? 0xFFE0 : COL_ACCENT;
    if (bar_w > 0)
        ST7789_Fill(ITEM_PAD, y, ITEM_PAD + bar_w, y + 8, bar_col);

    draw_footer(edit_mode ? "Rotate:change  Press:confirm" :
                            "Press:edit  LongPress:back");
}

void MenuScreen_OutputDetail_Handle(uint8_t idx, EncoderEvent_t evt,
                                    int8_t *cursor, bool *edit_mode)
{
    DeviceConfig_t *cfg = Config_Get();
    OutputConfig_t *out = &cfg->outputs[idx];

    if (!(*edit_mode)) {
        if (evt == ENC_EVENT_CW)
            *cursor = (*cursor + 1) % OUT_FIELD_COUNT;
        if (evt == ENC_EVENT_CCW)
            *cursor = (*cursor - 1 + OUT_FIELD_COUNT) % OUT_FIELD_COUNT;
        if (evt == ENC_EVENT_SHORT_PRESS) *edit_mode = true;
    } else {
        switch (*cursor) {
        case OUT_FIELD_ENABLED:
            if (evt == ENC_EVENT_CW || evt == ENC_EVENT_CCW)
                out->enabled = !out->enabled;
            if (evt == ENC_EVENT_SHORT_PRESS) *edit_mode = false;
            break;
        case OUT_FIELD_UNIVERSE:
            if (evt == ENC_EVENT_CW)
                out->universe = (out->universe + 1) % 32768;
            if (evt == ENC_EVENT_CCW)
                out->universe = (out->universe + 32767) % 32768;
            if (evt == ENC_EVENT_SHORT_PRESS) *edit_mode = false;
            break;
        case OUT_FIELD_LEDS:
            if (evt == ENC_EVENT_CW && out->led_count < 512)  out->led_count++;
            if (evt == ENC_EVENT_CCW && out->led_count > 1)   out->led_count--;
            if (evt == ENC_EVENT_SHORT_PRESS) *edit_mode = false;
            break;
        case OUT_FIELD_CURRENT:
            if (evt == ENC_EVENT_CW && out->max_current_A < 5) out->max_current_A++;
            if (evt == ENC_EVENT_CCW && out->max_current_A > 0) out->max_current_A--;
            if (evt == ENC_EVENT_SHORT_PRESS) *edit_mode = false;
            break;
        }
    }
}

void MenuScreen_Protocol(int8_t cursor)
{
    DeviceConfig_t *cfg = Config_Get();
    ST7789_Fill_Color(COL_BG);
    draw_header("Input protocol");

    static const char *protos[] = {"Art-Net", "sACN / E1.31", "DMX512 UART"};

    for (int i = 0; i < 3; i++) {
        int y = HEADER_H + i * ITEM_H;
        bool sel     = (i == cursor);
        bool active  = ((int)cfg->protocol == i);
        uint16_t bg  = sel ? COL_SELECTED : COL_BG;
        uint16_t fg  = sel ? COL_TEXT     : COL_TEXT_DIM;

        ST7789_Fill(0, y, MENU_W - 1, y + ITEM_H - 1, bg);

        char label[32];
        snprintf(label, sizeof(label), "%s %s",
                 active ? "[*]" : "[ ]", protos[i]);
        ST7789_WriteString(ITEM_PAD, y + (ITEM_H - 18) / 2,
                           label, Font_11x18, fg, bg);

        if (active) {
            ST7789_Fill(MENU_W - 4, y, MENU_W - 1, y + ITEM_H - 1, COL_ACCENT);
        }
        ST7789_Fill(0, y + ITEM_H - 1, MENU_W - 1, y + ITEM_H - 1, COL_SEPARATOR);
    }
    draw_footer("Press:select  LongPress:back");
}

void MenuScreen_System(void)
{
    ST7789_Fill_Color(COL_BG);
    draw_header("System");

    int y = HEADER_H + ITEM_PAD;
    ST7789_WriteString(ITEM_PAD, y,
        "LEPIX NODE v1.0", Font_11x18, COL_TEXT, COL_BG);
    y += 30;
    ST7789_WriteString(ITEM_PAD, y,
        "STM32F407VET6", Font_7x10, COL_TEXT_DIM, COL_BG);
    y += 20;
    ST7789_WriteString(ITEM_PAD, y,
        "4x WS2815 outputs", Font_7x10, COL_TEXT_DIM, COL_BG);
    y += 20;
    ST7789_WriteString(ITEM_PAD, y,
        "Art-Net / sACN / DMX", Font_7x10, COL_TEXT_DIM, COL_BG);

    y += 40;
    ST7789_Fill(ITEM_PAD, y, MENU_W - ITEM_PAD - 1, y + 36, COL_SELECTED);
    ST7789_WriteString(ITEM_PAD + 8, y + 8,
        "[ Save config to SD ]", Font_11x18, COL_ACCENT, COL_SELECTED);

    draw_footer("Press:save  LongPress:back");
}

void MenuScreen_SaveOK(void)
{
    ST7789_Fill_Color(COL_BG);
    draw_header("Saved");

    int y = LCD_H / 2 - 30;
    ST7789_WriteString(ITEM_PAD, y,
        "Configuration saved", Font_11x18, COL_ACCENT, COL_BG);
    y += 30;
    ST7789_WriteString(ITEM_PAD, y,
        "to SD card.", Font_11x18, COL_TEXT, COL_BG);

    draw_footer("Press:back to menu");
}