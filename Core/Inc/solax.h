#ifndef _SOLAX_H_
#define _SOLAX_H_

void solax_process(void);
void solax_set_max_ac_current(uint8_t current);

void solax_set_max_dc_chg_current(uint16_t current);
void solax_set_max_dc_dis_current(uint16_t current);

void solax_set_battery_voltage_max(uint16_t voltage);
void solax_set_battery_voltage_tgt(uint16_t voltage);
void solax_set_battery_voltage_min(uint16_t voltage);
void solax_set_battery_capacity_max(uint32_t energy);
void solax_set_battery_capacity(uint32_t energy);
void solax_set_battery_soc(uint16_t soc);

#endif /* _SOLAX_H_ */

