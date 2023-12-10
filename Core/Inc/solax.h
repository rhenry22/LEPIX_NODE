#ifndef _SOLAX_H_
#define _SOLAX_H_

#define SOLAX_MAXIMUM_SUPPORTED_VOLTAGE (450)
#define SOLAX_MAXIMUM_SUPPORTED_CURRENT (16)    // 40, but only have a 16A fuse!

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

void solax_json_update(void);

#endif /* _SOLAX_H_ */

