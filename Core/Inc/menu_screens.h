#ifndef MENU_SCREENS_H
#define MENU_SCREENS_H

#include "menu.h"
#include "config.h"
#include "encoder.h"

void MenuScreen_Main(int8_t cursor, const MenuItem_t *items,
                     uint8_t count, int8_t scroll);
void MenuScreen_List(const char *title, const MenuItem_t *items,
                     uint8_t count, int8_t cursor);
void MenuScreen_DHCP(void);
void MenuScreen_StaticIP(int8_t cursor);
void MenuScreen_IPEditor_Draw(ScreenID_t field);
void MenuScreen_IPEditor_Handle(ScreenID_t field, EncoderEvent_t evt);
void MenuScreen_Outputs(int8_t cursor);
void MenuScreen_OutputDetail(uint8_t idx, int8_t cursor, bool edit_mode);
void MenuScreen_OutputDetail_Handle(uint8_t idx, EncoderEvent_t evt,
                                    int8_t *cursor, bool *edit_mode);
void MenuScreen_Protocol(int8_t cursor);
void MenuScreen_System(void);
void MenuScreen_SaveOK(void);

#endif
