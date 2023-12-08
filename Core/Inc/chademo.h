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
  CHADEMO_STATE_INS_TEST,
  CHADEMO_STATE_ON
} CHADEMO_STATE;

bool chademo_init(void);

void chademo_process(void);
void chademo_stop(void);
void chademo_start(void);

void chademo_set_max_power(uint16_t power);
CHADEMO_STATE chademo_get_state(void);

#endif /* _CHADEMO_H_ */

