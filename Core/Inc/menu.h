#ifndef LEPIX_MENU_H
#define LEPIX_MENU_H

#include <stdint.h>
#include <stdbool.h>
#include "encoder.h"
#include "icon_loader.h"
#include "st7789.h"

typedef enum {
    SCR_MAIN = 0,
    SCR_NETWORK,
    SCR_NET_DHCP,
    SCR_NET_STATIC,
    SCR_NET_EDIT_IP,
    SCR_NET_EDIT_MASK,
    SCR_NET_EDIT_GW,
    SCR_NET_EDIT_DNS,
    SCR_OUTPUTS,
    SCR_OUTPUT_DETAIL,
    SCR_PROTOCOL,
    SCR_SYSTEM,
    SCR_SAVE_OK,
    SCR_COUNT
} ScreenID_t;

typedef struct {
    const char *label;
    IconID_t    icon;
    ScreenID_t  target;
} MenuItem_t;

#define MENU_W        260
#define LCD_H         240
#define HEADER_H      44
#define ITEM_H        44
#define ITEM_ICON_W   40
#define ITEM_PAD      8
#define FOOTER_H      24
#define VISIBLE_ITEMS ((LCD_H - HEADER_H - FOOTER_H) / ITEM_H)

#define COL_BG        0x0841
#define COL_HEADER    0x1082
#define COL_SELECTED  0x2945
#define COL_TEXT      0xFFFF
#define COL_TEXT_DIM  0x8C71
#define COL_ACCENT    0x05F4
#define COL_SEPARATOR 0x2945

void Menu_Init(void);
void Menu_Task(void);
void Menu_Navigate(EncoderEvent_t evt);
ScreenID_t Menu_GetCurrentScreen(void);

#endif
