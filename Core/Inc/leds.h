#ifndef _LEDS_H_
#define _LEDS_H_

#include <stdint.h>
#include <stdbool.h>

/* Upper / Red (bits 8-15) and Lower / Green (bits 0-7) Debug Leds */
enum debug_leds
{
  DBG_LED_HV_TEST = 0,
  DBG_LED_ISO_TEST,
  DBG_LED_CT_PRE,
  DBG_LED_CT_MAIN,
  DBG_LED_STAT_RED_INV,
  DBG_LED_STAT_RED_EV,
  DBG_LED_HV_INV,
  DBG_LED_HV_BATT,

  DBG_LED_PP_INSERTED = 8,
  DBG_LED_CP_READY,
  DBG_LED_CP_CHARGE,
  DBG_LED_STAT_GREEN_INV = 12,
  DBG_LED_STAT_GREEN_EV,
};

bool leds_init(void);
bool leds_set(uint16_t leds);
bool leds_flash(uint16_t leds);
bool leds_clear(uint16_t leds);
int leds_process_cmd(char **args, int argc);

#endif /* _LEDS_H_ */
