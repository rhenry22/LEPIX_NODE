#include "menu.h"
#include "menu_screens.h"
#include "config.h"
#include "icon_loader.h"
#include <string.h>
#include <stdio.h>

static ScreenID_t current_screen  = SCR_MAIN;
static ScreenID_t previous_screen = SCR_MAIN;
static int8_t     cursor          = 0;
static int8_t     scroll_offset   = 0;
static uint8_t    selected_output = 0;
static bool       edit_mode       = false;
static bool       needs_redraw    = true;

static const MenuItem_t main_items[] = {
    {"Network",        ICON_NETWORK,  SCR_NETWORK},
    {"Outputs",        ICON_OUTPUTS,  SCR_OUTPUTS},
    {"Input protocol", ICON_PROTOCOL, SCR_PROTOCOL},
    {"System",         ICON_SYSTEM,   SCR_SYSTEM},
};
#define MAIN_ITEM_COUNT 4

static const MenuItem_t net_items[] = {
    {"DHCP",      ICON_DHCP,   SCR_NET_DHCP},
    {"Static IP", ICON_STATIC, SCR_NET_STATIC},
};
#define NET_ITEM_COUNT 2

static void set_screen(ScreenID_t s)
{
    previous_screen = current_screen;
    current_screen  = s;
    cursor          = 0;
    scroll_offset   = 0;
    edit_mode       = false;
    needs_redraw    = true;
}

static void go_back(void) { set_screen(previous_screen); }

void Menu_Navigate(EncoderEvent_t evt)
{
    DeviceConfig_t *cfg = Config_Get();

    switch (current_screen) {
    case SCR_MAIN:
        if (evt == ENC_EVENT_CW)  cursor = (cursor + 1) % MAIN_ITEM_COUNT;
        if (evt == ENC_EVENT_CCW) cursor = (cursor - 1 + MAIN_ITEM_COUNT) % MAIN_ITEM_COUNT;
        if (evt == ENC_EVENT_SHORT_PRESS) set_screen(main_items[cursor].target);
        if (evt == ENC_EVENT_DOUBLE_PRESS) { Config_Save(); set_screen(SCR_SAVE_OK); }
        needs_redraw = true;
        break;
    case SCR_NETWORK:
        if (evt == ENC_EVENT_CW)  cursor = (cursor + 1) % NET_ITEM_COUNT;
        if (evt == ENC_EVENT_CCW) cursor = (cursor - 1 + NET_ITEM_COUNT) % NET_ITEM_COUNT;
        if (evt == ENC_EVENT_SHORT_PRESS) set_screen(net_items[cursor].target);
        if (evt == ENC_EVENT_DOUBLE_PRESS)  go_back();
        needs_redraw = true;
        break;
    case SCR_NET_DHCP:
        if (evt == ENC_EVENT_SHORT_PRESS) { cfg->net_mode = NET_DHCP; needs_redraw = true; }
        if (evt == ENC_EVENT_DOUBLE_PRESS)  go_back();
        break;
    case SCR_NET_STATIC: {
        static const ScreenID_t st[] = {SCR_NET_EDIT_IP,SCR_NET_EDIT_MASK,SCR_NET_EDIT_GW,SCR_NET_EDIT_DNS};
        if (evt == ENC_EVENT_CW)  cursor = (cursor + 1) % 4;
        if (evt == ENC_EVENT_CCW) cursor = (cursor - 1 + 4) % 4;
        if (evt == ENC_EVENT_SHORT_PRESS) { cfg->net_mode = NET_STATIC; set_screen(st[cursor]); }
        if (evt == ENC_EVENT_DOUBLE_PRESS)  go_back();
        needs_redraw = true;
        break;
    }
    case SCR_NET_EDIT_IP:
    case SCR_NET_EDIT_MASK:
    case SCR_NET_EDIT_GW:
    case SCR_NET_EDIT_DNS:
        MenuScreen_IPEditor_Handle(current_screen, evt);
        needs_redraw = true;
        if (evt == ENC_EVENT_DOUBLE_PRESS) go_back();
        break;
    case SCR_OUTPUTS:
        if (evt == ENC_EVENT_CW)  cursor = (cursor + 1) % MAX_OUTPUTS;
        if (evt == ENC_EVENT_CCW) cursor = (cursor - 1 + MAX_OUTPUTS) % MAX_OUTPUTS;
        if (evt == ENC_EVENT_SHORT_PRESS) { selected_output = cursor; set_screen(SCR_OUTPUT_DETAIL); }
        if (evt == ENC_EVENT_DOUBLE_PRESS)  go_back();
        needs_redraw = true;
        break;
    case SCR_OUTPUT_DETAIL:
        MenuScreen_OutputDetail_Handle(selected_output, evt, &cursor, &edit_mode);
        needs_redraw = true;
        if (evt == ENC_EVENT_DOUBLE_PRESS && !edit_mode) go_back();
        break;
    case SCR_PROTOCOL:
        if (evt == ENC_EVENT_CW)  cursor = (cursor + 1) % 3;
        if (evt == ENC_EVENT_CCW) cursor = (cursor - 1 + 3) % 3;
        if (evt == ENC_EVENT_SHORT_PRESS) { cfg->protocol = (InputProtocol_t)cursor; needs_redraw = true; }
        if (evt == ENC_EVENT_DOUBLE_PRESS)  go_back();
        needs_redraw = true;
        break;
    case SCR_SYSTEM:
        if (evt == ENC_EVENT_SHORT_PRESS) { Config_Save(); set_screen(SCR_SAVE_OK); }
        if (evt == ENC_EVENT_DOUBLE_PRESS)  go_back();
        break;
    case SCR_SAVE_OK:
        if (evt == ENC_EVENT_SHORT_PRESS || evt == ENC_EVENT_DOUBLE_PRESS) set_screen(SCR_MAIN);
        break;
    default: break;
    }
}

static void render(void)
{
    switch (current_screen) {
    case SCR_MAIN:       MenuScreen_Main(cursor, main_items, MAIN_ITEM_COUNT, scroll_offset); break;
    case SCR_NETWORK:    MenuScreen_List("Network", net_items, NET_ITEM_COUNT, cursor); break;
    case SCR_NET_DHCP:   MenuScreen_DHCP(); break;
    case SCR_NET_STATIC: MenuScreen_StaticIP(cursor); break;
    case SCR_NET_EDIT_IP:   MenuScreen_IPEditor_Draw(SCR_NET_EDIT_IP); break;
    case SCR_NET_EDIT_MASK: MenuScreen_IPEditor_Draw(SCR_NET_EDIT_MASK); break;
    case SCR_NET_EDIT_GW:   MenuScreen_IPEditor_Draw(SCR_NET_EDIT_GW); break;
    case SCR_NET_EDIT_DNS:  MenuScreen_IPEditor_Draw(SCR_NET_EDIT_DNS); break;
    case SCR_OUTPUTS:       MenuScreen_Outputs(cursor); break;
    case SCR_OUTPUT_DETAIL: MenuScreen_OutputDetail(selected_output, cursor, edit_mode); break;
    case SCR_PROTOCOL:      MenuScreen_Protocol(cursor); break;
    case SCR_SYSTEM:        MenuScreen_System(); break;
    case SCR_SAVE_OK:       MenuScreen_SaveOK(); break;
    default: break;
    }
}

void Menu_Init(void)
{
    current_screen = SCR_MAIN;
    cursor         = 0;
    needs_redraw   = true;
    ST7789_Fill_Color(COL_BG);
}

void Menu_Task(void)
{
    EncoderEvent_t evt = Encoder_GetEvent();
    if (evt != ENC_EVENT_NONE) Menu_Navigate(evt);
    if (needs_redraw) { render(); needs_redraw = false; }
}

ScreenID_t Menu_GetCurrentScreen(void) { return current_screen; }
