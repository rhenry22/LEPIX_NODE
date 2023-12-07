#ifndef _SOLAX_H_
#define _SOLAX_H_

void solax_process(void);
void solax_set_max_ac_current(uint8_t current);
void solax_set_max_dc_chg_power(uint32_t power);
void solax_set_max_dc_dis_power(uint32_t power);

#endif /* _SOLAX_H_ */

