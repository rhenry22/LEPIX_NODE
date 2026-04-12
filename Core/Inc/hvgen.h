#ifndef _HVGEN_H_
#define _HVGEN_H_

#include <stdint.h>
#include <stdbool.h>

#define HV_GEN_MAX_VOLTAGE    (500) /* Max HV voltage in V */
#define HV_GEN_MIN_VOLTAGE    (0)   /* Min HV voltage in V */

bool hvgen_init(void);
void hvgen_iso_test_enable(bool enable, uint32_t voltage);
bool hvgen_get_isolation_r(uint32_t *iso_r);
int hvgen_process_cmd(char **args, int argc);

#endif /* _HVGEN_H_ */
