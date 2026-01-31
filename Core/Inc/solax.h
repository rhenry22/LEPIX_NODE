#ifndef _SOLAX_H_
#define _SOLAX_H_

#include <stdbool.h>
#include <stdint.h>

#define SOLAX_MAXIMUM_SUPPORTED_VOLTAGE (450)
#define SOLAX_MINIMUM_SUPPORTED_VOLTAGE (288)
#define SOLAX_MAXIMUM_SUPPORTED_CURRENT (40)

#define SOLAX_MAXIMUM_SOC               (100)
#define SOLAX_MINIMUM_SOC               (15)

#define MB_SLAVE_INVERTER   (247)

enum
{
  FOX_GRID_V = 11009,     /* Grid Voltage (V x10) */
  FOX_GRID_I = 11010,     /* Grid Current (A x10) */
  FOX_GRID_P1 = 11011,    /* Grid Phase R Power (W) */
  FOX_GRID_P2 = 11012,    /* Grid Phase Q Power (W) */
  FOX_GRID_P3 = 11013,    /* Grid Phase S Power (W) */

  FOX_TEMP_INV = 11024,    /* Inverter Temp. (degC x10) */
  FOX_TEMP_ENV = 11025,    /* Environment Temp. (degC x10) */

  FOX_INV_STATE = 11056,   /* Inverter Status */
  FOX_BATT_STATE = 11057,  /* Battery Status */

  FOX_FAULT_1 = 11061,
  FOX_FAULT_2 = 11062,
  FOX_FAULT_3 = 11063,
  FOX_FAULT_4 = 11064,
  FOX_FAULT_5 = 11065,
  FOX_FAULT_6 = 11066,
  FOX_FAULT_7 = 11067,
  FOX_FAULT_8 = 11068,

  FOX_REM_EN = 44000,
  FOX_REM_TIMER = 44001,
  FOX_REM_POWER = 44002,

  FOX_SYS_EN = 41013,

  FOX_CLEAR_EVTS = 45000
};

/* Fault bits */
enum
{
  FAULT1_GRID_LOST = 0,
  FAULT1_GRID_VOLT,
  FAULT1_GRID_FREQ,
  FAULT1_GRID_10M_VOLT,
  FAULT1_EPS_VOLT,
  FAULT1_SW_INV_CUR,
  FAULT1_DCI,
  FAULT1_TBD1,
  FAULT1_HW_INV_CUR,
  FAULT1_SW_BUS_VOLT,
  FAULT1_BAT_VOLT,
  FAULT1_SW_BAT_CUR,
  FAULT1_ISO,
  FAULT1_RES_CUR,
  FAULT1_PV_VOLT,
  FAULT1_SW_PV_CUR
};

enum
{
  FAULT2_TEMP = 0,
  FAULT2_GROUND_CONN,
  FAULT2_TBD1,
  FAULT2_EPS_OVER_LOAD,
  FAULT2_BATT_POWER_LOW,
  FAULT2_HW_BUS_VOLT,
  FAULT2_HW_PV_CUR,
  FAULT2_HW_BAT_CUR,
  FAULT2_SCI,
  FAULT2_MASTER_SPI
};
bool solax_init(void);
void solax_kick(void);

void solax_enable(void);
void solax_disable(void);

bool solax_check_faults(uint32_t *faults);

void solax_set_output_power(int16_t power);

void solax_set_max_ac_current(uint8_t current);

void solax_set_max_dc_chg_current(uint16_t current);
void solax_set_max_dc_dis_current(uint16_t current);

void solax_set_battery_voltage_max(uint16_t voltage);
void solax_set_battery_voltage_tgt(uint16_t voltage);
void solax_set_battery_voltage_min(uint16_t voltage);
void solax_set_battery_capacity_max(uint32_t energy);
void solax_set_battery_capacity(uint32_t energy);
void solax_set_battery_soc(uint16_t soc);

int solax_process_cmd(char **args, int argc);
void solax_json_update(void);

#endif /* _SOLAX_H_ */

