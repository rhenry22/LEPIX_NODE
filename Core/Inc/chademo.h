#ifndef _CHADEMO_H_
#define _CHADEMO_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum _chademo_state
{
  CHADEMO_STATE_OFF,
  CHADEMO_STATE_START,
  CHADEMO_STATE_PARAM_CHK,
  CHADEMO_STATE_PERM_OK,
  CHADEMO_STATE_INS_TEST_BASE,
  CHADEMO_STATE_INS_TEST,
  CHADEMO_STATE_BATT_CHECK,
  CHADEMO_STATE_ON,
  CHADEMO_STATE_STOP,
  CHADEMO_STATE_WELD_CHECK,
  CHADEMO_STATE_WAIT_K_OFF,
  CHADEMO_STATE_WAIT_VEHICLE_OFF,
  CHADEMO_STATE_ERROR
} CHADEMO_STATE;

bool chademo_init(void);

void chademo_process(void);
void chademo_stop(void);
void chademo_start(void);

void chademo_set_max_power(uint32_t power);
CHADEMO_STATE chademo_get_state(void);
int32_t chademo_get_power(void);

int chademo_process_cmd(char **args, int argc);
void chademo_json_update(void);

#endif /* _CHADEMO_H_ */

