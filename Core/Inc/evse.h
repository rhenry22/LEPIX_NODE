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


typedef void (evse_current_changed_cb)(EVSE_PP pp, uint8_t current);
void evse_tim_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void evse_tim_CaptureCallback(TIM_HandleTypeDef *htim);
bool evse_init(evse_current_changed_cb *cb);
EVSE_PP evse_get_pp(void);
void evse_json_update(void);
void evse_process(void);

#endif /* __SENSOR_H__ */
