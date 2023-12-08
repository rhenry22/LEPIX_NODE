#ifndef __EVSE_H__
#define __EVSE_H__

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
  EVSE_PP_NONE,     // Plug not inserted
  EVSE_PP_PRESSED,  // Plug inserted, button pressed
  EVSE_PP_INSERTED, // Plug inserted, not pressed
  EVSE_PP_ERROR     // Invalid reading
} EVSE_PP;


bool evse_init(void);
void evse_get_max_current(uint8_t *current);
EVSE_PP evse_get_pp(void);

#endif /* __SENSOR_H__ */
