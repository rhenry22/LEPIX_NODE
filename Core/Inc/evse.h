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

typedef enum
{
  EVSE_CP_A,        /* No vehicle connected */
  EVSE_CP_B,        /* Vehicle Connected, not ready */
  EVSE_CP_C,        /* Vehicle Connected, Charge */
  EVSE_CP_D,        /* Vehicle Connected, Charge (with ventilation) */
  EVSE_CP_ERROR     /* Invalid reading */
} EVSE_CP;

typedef void (evse_pp_changed_cb)(EVSE_PP pp, uint8_t current);
typedef void (evse_cp_changed_cb)(EVSE_CP cp);
void evse_tim_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void evse_tim_CaptureCallback(TIM_HandleTypeDef *htim);
bool evse_init(evse_pp_changed_cb *pp_cb, evse_cp_changed_cb *cp_cb);
EVSE_PP evse_get_pp(void);
EVSE_CP evse_get_cp(void);
void evse_set_cp(uint8_t pwm);

int evse_process_cmd(char **args, int argc);
void evse_json_update(void);
void evse_process(void);

#endif /* __SENSOR_H__ */
