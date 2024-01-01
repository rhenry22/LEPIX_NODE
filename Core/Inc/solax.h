#ifndef _SOLAX_H_
#define _SOLAX_H_

#define SOLAX_MAXIMUM_SUPPORTED_VOLTAGE (450)
#define SOLAX_MINIMUM_SUPPORTED_VOLTAGE (288)
#define SOLAX_MAXIMUM_SUPPORTED_CURRENT (40)

#define SOLAX_MAXIMUM_SOC               (90)
#define SOLAX_MINIMUM_SOC               (20)

HAL_StatusTypeDef solax_init(void);
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

bool solax_contactor_enabled(void);

void solax_json_update(void);

#endif /* _SOLAX_H_ */

