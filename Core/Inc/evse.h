#ifndef __EVSE_H__
#define __EVSE_H__

#include <stdint.h>
#include <stdbool.h>
#include "tim.h"

typedef enum
{
  EVSE_PP_NONE,     /* Plug not inserted */
  EVSE_PP_PRESSED,  /* Plug inserted, button pressed */
  EVSE_PP_INSERTED, /* Plug inserted, not pressed */
  EVSE_PP_ERROR     /* Invalid reading */
} EVSE_PP;

typedef void (evse_pp_changed_cb)(EVSE_PP pp, uint8_t current);
void evse_tim_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void evse_tim_CaptureCallback(TIM_HandleTypeDef *htim);
bool evse_init(evse_pp_changed_cb *pp_cb);
EVSE_PP evse_get_pp(void);

int evse_process_cmd(char **args, int argc);
void evse_json_update(void);
void evse_process(void);

#endif /* __SENSOR_H__ */
