/** @file solax.c
 *  @brief Module to interface with Solax / FoxESS Inverters
 *
 *  This module provides a layer to run a statemachine emulating a BMS to use
 *  with the Solax / FoxESS inverters.
 *
 *  Inspired by:
 *    https://github.com/rand12345/solax_can_bus
 *  and
 *    https://github.com/dalathegreat/BYD-Battery-Emulator-For-Gen24
 *
 *  @author Richard Taylor <richard@artaylor.co.uk>
 *  @bug No known bugs.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "can.h"
#include "sensor.h"
#include "chademo.h"
#include "solax.h"

#define DEBUG_SOLAX

/* Battery size in Wh (Maximum value for most inverters is 60000 [60kWh],
 * you can use larger batteries but do not set value over 60000!
 */
#define BATTERY_WH_MAX    (24000)
#define CELL_MAX_VOLTAGE  (4135)
#define CELL_MIN_VOLTAGE  (3135)
#define NUM_CELLS         (96)

#define ABSOLUTE_MAX_VOLTAGE (NUM_CELLS * CELL_MAX_VOLTAGE / 1000)
#define ABSOLUTE_MIN_VOLTAGE (NUM_CELLS * CELL_MIN_VOLTAGE / 1000)

#define SOLAX_TIMEOUT     (5000)
#define SOLAX_UPDATE_RATE (1000)
#define SOLAX_N_PACKS     (7)

#define MSG_1871_STATUS     (1)
#define MSG_1871_CONTACTOR  (3)

typedef enum _solax_state
{
  SOLAX_BATTERY_ANNOUNCE,
  SOLAX_REQUEST_CONTACTOR_CLOSE,
  SOLAX_CONTACTOR_CLOSING,
  SOLAX_CONTACTOR_CLOSED,
  SOLAX_FAULT,
  SOLAX_UPDATING_FW
} SOLAX_STATE;

struct _solax_data
{
  struct
  {
    struct
    {
      uint8_t frame_id;
      uint8_t data[7];
    } msg_1871;

    struct
    {
      uint8_t frame_id;
      uint8_t data[7];
    } msg_1871_3; /* Time Heartbeat */
  } inverter;

  struct
  {
    struct _bms_answer
    {
      uint16_t data[4];
    } msg_1801;

    struct _bms_limits
    {
      uint16_t slave_voltage_max; /* Voltage x10 V */
      uint16_t slave_voltage_min; /* Voltage x10 V */
      uint16_t charge_max;        /* Current x10 A */
      uint16_t discharge_max;     /* Current x10 A */
    } msg_1872;

    struct _bms_pack_data
    {
      uint16_t voltage; /* Voltage x10 V */
      int16_t current;  /* Current x10 A */
      uint16_t soc;     /* % */
      uint16_t energy;  /* Energy x100 kWh (12.34kWh)*/
    } msg_1873;

    struct _bms_cell_data
    {
      uint16_t cell_temp_max; /* Temp x10 C */
      uint16_t cell_temp_min; /* Temp x10 C */
      uint16_t cell_mv_max;   /* mV / 100 */
      uint16_t cell_mv_min;   /* mV / 100 */
    } msg_1874;

    struct _bms_status
    {
      int16_t pack_temp; /* Temp x10 C */
      uint16_t num_batts; /* Number of batteries */
      uint16_t contactor; /* Contactor on / off */
      uint16_t reserved;
    } msg_1875;

    struct _bms_pack_temps
    {
      uint16_t reserved;
      uint16_t cell_mv_max;   /* mV */
      uint16_t reserved2;
      uint16_t cell_mv_min;   /* mV */
    } msg_1876;

    struct __attribute__((packed)) _bms_pack_type
    {
      uint16_t reserved1;
      uint16_t reserved2;
      uint8_t type;  /* Battery Type */
      uint8_t version1;
      uint8_t version2;
      uint8_t pack;
    } msg_1877;

    struct _bms_pack_stats
    {
      uint16_t pack_voltage_max;  /* Voltage x10 V */
      uint16_t reserved;
      uint32_t wh_total;
    } msg_1878;

    struct _bms_pack_version1
    {
      uint16_t data[4];
    } msg_187A;

    struct _bms_pack_version2
    {
      uint16_t pack;
      uint16_t ver1;
      uint16_t ver2;
      uint16_t ver3;
    } msg_187B;

    struct
    {
      char serial[8];
    } msg_1881;

    struct
    {
      char serial[8];
    } msg_1882;

    struct _bms_announce
    {
      /* Empty (DLC = 0) */
    } msg_100A001;
  } bms;
};

struct _solax_data solax_data = {
  .bms = {

    /* BMS_Answer
     * Unknown contents.
     * Response announcing that battery will be connected.
     */
    .msg_1801 = {
      .data = {0x0002, 0x0001, 0x0001, 0x0000}
    },

    /* BMS_Limits */
    .msg_1872 = {
      .slave_voltage_max = ABSOLUTE_MAX_VOLTAGE / 100,
      .slave_voltage_min = ABSOLUTE_MIN_VOLTAGE / 100
    },

    /* BMS_CellData */
    .msg_1874 = {
      .cell_mv_max = CELL_MAX_VOLTAGE / 100,
      .cell_mv_min = CELL_MIN_VOLTAGE / 100,
      .cell_temp_max = 200,
      .cell_temp_min = 160
    },

    /* BMS_Status */
    .msg_1875 = {
      .pack_temp = 180,
      .num_batts = 0,//SOLAX_N_PACKS,
      .contactor = 0
    },

    /* BMS_PackTemps (Cell voltages) */
    .msg_1876 = {
      .reserved = 1
    },

    /* BMS_Version */
    .msg_1877 = {0x0000, 0x0000, 0x50, 0x02, 0x22, 0x02},

    /* BMS_PackStats */
    .msg_1878 = {
      .pack_voltage_max = ABSOLUTE_MAX_VOLTAGE * 10,
      .wh_total = BATTERY_WH_MAX
    },

    .msg_187A = { {0x4001, 0x0000, 0x0000, 0x0000} },
    .msg_187B = { 0x0000, 0x0000, 0x0000, 0x0032 },

    /* BMS_Serial */
    .msg_1881 = { .serial = { 0x00, 0x35, 0x53, 0x42, 0x4D, 0x53, 0x46, 0x41 }},
    .msg_1882 = { .serial = { 0x00, 0x31, 0x33, 0x41, 0x42, 0x30, 0x35, 0x30 }},

    /* The following will be updated once ChaDeMo starts up */

    /* BMS_Limits */
    .msg_1872 = {
      .charge_max = 0,
      .discharge_max = 0
    },

    /* BMS_PackData */
    .msg_1873 = {
      .voltage = 3800,
      .current = 0,
      .soc = 25,
      .energy = BATTERY_WH_MAX / 10 / 2
    },

    /* BMS_PackTemps (Cell voltages) */
    .msg_1876 = {
      .cell_mv_max = 3700,
      .cell_mv_min = 3700
    }
  }
};

static SOLAX_STATE state = SOLAX_BATTERY_ANNOUNCE;  /* BMS state machine */
static uint16_t max_ac_power = 0;                /* Maximum current limit advertised by EVSE (W x1) */
static uint32_t t_zero_set = 0;                     /* Time at which current request set to zero (debug / check inverter response) */
static uint32_t last_update = 0;                    /* Last time we saw a CAN message */
static uint32_t bms_update = 0;                     /* Last time we sent our CAN messages */
static uint16_t max_charge_current = 0;             /* Max DC charge current (A x10) */
static uint16_t max_discharge_current = 0;          /* Max DC discharge current (A x10) */
static bool contactor_close = false;                /* Has the inverter requested contactor close? */

static char last_error[ERROR_LEN+1] = {0};          /* Last error string */

static HAL_StatusTypeDef solax_send_message(uint32_t id, uint8_t *data, uint8_t len)
{
  CAN_TxHeaderTypeDef TxHeader;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Make sure we clear the header to default */
  memset(&TxHeader, 0, sizeof(TxHeader));

  TxHeader.DLC = len;
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.ExtId = id;
  TxHeader.RTR = CAN_RTR_DATA;

  ret = MX_CAN_Transmit(&hcan2, &TxHeader, data);

#ifdef DEBUG_SOLAX
      //printf("< 0x%04lX (%d): ", id, len);
      //dump_packet(data, TxHeader.DLC);
#endif

  if (ret != HAL_OK)
  {
    snprintf(last_error, ERROR_LEN, "CAN Send Failed: %d", ret);
  }

  return ret;
}

static HAL_StatusTypeDef solax_send_standard_response(void)
{
  static uint8_t pack = 0;
  HAL_StatusTypeDef ret = HAL_OK;


  /* Set pack specific messages */
  if (pack == 0)
  {
    solax_data.bms.msg_1877.pack = 0x02;

    solax_data.bms.msg_187B.pack = pack;
    solax_data.bms.msg_187B.ver1 = 0x0801;
  }
  else
  {
    solax_data.bms.msg_1877.pack = 0x10 * pack;

    solax_data.bms.msg_187B.pack = pack;
    solax_data.bms.msg_187B.ver1 = 0x0602;
  }

  /* Rotate between all of the packs */
  pack++;
  if (pack > solax_data.bms.msg_1875.num_batts)
    pack = 0;

  ret = solax_send_message(0x1872, (uint8_t*)&solax_data.bms.msg_1872, 8);
  if (HAL_OK == ret)
    ret = solax_send_message(0x1873, (uint8_t*)&solax_data.bms.msg_1873, 8);
  if (HAL_OK == ret)
    ret = solax_send_message(0x1874, (uint8_t*)&solax_data.bms.msg_1874, 8);
  if (HAL_OK == ret)
    ret = solax_send_message(0x1875, (uint8_t*)&solax_data.bms.msg_1875, 8);
  if (HAL_OK == ret)
    ret = solax_send_message(0x1876, (uint8_t*)&solax_data.bms.msg_1876, 8);
  if (HAL_OK == ret)
    ret = solax_send_message(0x1877, (uint8_t*)&solax_data.bms.msg_1877, 8);
  if (HAL_OK == ret)
    ret = solax_send_message(0x1878, (uint8_t*)&solax_data.bms.msg_1878, 8);
/*
  if (HAL_OK == ret)
    ret = solax_send_message(0x187A, (uint8_t*)&solax_data.bms.msg_187A, 8);
  if (HAL_OK == ret)
    ret = solax_send_message(0x187B, (uint8_t*)&solax_data.bms.msg_187B, 8);
*/
  return ret;
}

static void solax_update_values(void)
{
  int32_t voltage; /* V x10 */
  int32_t current; /* A x10 */

  /* Update Measured Values */
  sensor_get_value(SENSOR_BATT_VOLTAGE, &voltage);
  sensor_get_value(SENSOR_BATT_CURRENT, &current);

  /* BMS_PackData */
  solax_data.bms.msg_1873.voltage = voltage;
  solax_data.bms.msg_1873.current = current;

  /* BMS_PackTemps (Cell voltages) */
  solax_data.bms.msg_1876.cell_mv_max = (voltage / NUM_CELLS) + 40;
  solax_data.bms.msg_1876.cell_mv_min = (voltage / NUM_CELLS) - 40;

  // ToDo: Add temperature monitoring
#if 0
  //BMS_Status
  SOLAX_1875.data.u8[0] = (uint8_t)temperature_average;
  SOLAX_1875.data.u8[1] = (temperature_average >> 8);
#endif

  /* Ensure we're within EVSE limits */

  if (voltage > 0)
  {
    uint32_t req_current; /* A x10 */
    uint32_t req_power;   /* W x1 */

    /* Voltage and Curent are both x10 */
    /* AC power is in W x1 */

    /* Charge */
    req_current = max_charge_current;
    req_power = voltage * req_current / 100;
    if (req_power > max_ac_power)
      req_current = (100 * max_ac_power) / voltage;

    solax_data.bms.msg_1872.charge_max = req_current;

    /* Discharge */
    req_current = max_discharge_current;
    req_power = voltage * req_current / 100;
    if (req_power > max_ac_power)
      req_current = (100 * max_ac_power) / voltage;

    solax_data.bms.msg_1872.discharge_max = req_current;
  }
  else
  {
    solax_data.bms.msg_1872.charge_max = 0;
    solax_data.bms.msg_1872.discharge_max = 0;
  }

  /* Check SoC and adjust charge rate if needed */
  if (solax_data.bms.msg_1873.soc >= SOLAX_MAXIMUM_SOC)
    solax_data.bms.msg_1872.charge_max = 0;

  if (solax_data.bms.msg_1873.soc <= SOLAX_MINIMUM_SOC)
    solax_data.bms.msg_1872.discharge_max = 0;
}

static HAL_StatusTypeDef solax_update_state(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  SOLAX_STATE s = state;

  /* Update the contactor state */
  if (chademo_get_state() == CHADEMO_STATE_ON)
    solax_data.bms.msg_1875.contactor = 2;
  else
    solax_data.bms.msg_1875.contactor = 0;

  if (solax_data.inverter.msg_1871.data[MSG_1871_STATUS] != 0x0001)
  {
    state = SOLAX_FAULT;
    snprintf(last_error, ERROR_LEN,
             "Unhandled Inverter Status: %d",
             solax_data.inverter.msg_1871.data[MSG_1871_STATUS]);
    contactor_close = false;
    ret = HAL_ERROR;
  }

  if (ret == HAL_OK)
  {
    switch (state) {
      case SOLAX_BATTERY_ANNOUNCE:
        /* BMS Announce */
        solax_send_message(0x100A001, (uint8_t*)&solax_data.bms.msg_100A001, 0);

        if (contactor_close)
        {
          /* Message from the inverter to proceed to contactor closing */
          state = SOLAX_REQUEST_CONTACTOR_CLOSE;
        }
      break;

      case SOLAX_REQUEST_CONTACTOR_CLOSE:
        /* Stay in this state until ChaDeMo starts up */
        if (chademo_get_state() >= CHADEMO_STATE_START &&
            chademo_get_state() <= CHADEMO_STATE_ON)
        {
          
        }

        if (contactor_close)
        {
          /* Respond that battery is connecting */
          solax_send_message(0x1801, (uint8_t*)&solax_data.bms.msg_1801, 8);

          state = SOLAX_CONTACTOR_CLOSING;
        }
      break;

      case SOLAX_CONTACTOR_CLOSING:
        /* Stay in this state until ChaDeMo completes connection */
        if (solax_data.bms.msg_1875.contactor != 0)
        {
          state = SOLAX_CONTACTOR_CLOSED;
        }
      break;

      case SOLAX_CONTACTOR_CLOSED:
      {
        if (chademo_get_state() >= CHADEMO_STATE_STOP)
        {
          snprintf(last_error, ERROR_LEN, "Vehicle stopping");
          state = SOLAX_BATTERY_ANNOUNCE;
        }

        if (!contactor_close)
        {
          /* Message from the inverter to open contactor */
          snprintf(last_error, ERROR_LEN, "Inverter Requests Open Contactor");
          state = SOLAX_BATTERY_ANNOUNCE;
        }
      }
      break;

      case SOLAX_FAULT:
      case SOLAX_UPDATING_FW:
        contactor_close = false;
      break;
    }
  }

  if (s != state)
    trigger_json_update();

  return ret;
}

static void solax_process_frame(void)
{
  /* Process the frame */
  switch (solax_data.inverter.msg_1871.frame_id)
  {
    case 0x01: /* Status update */
      /* These are the frames that count */
      last_update = HAL_GetTick();
    break;

    case 0x02: /* Command */
      /* Set inverter contactor close request state */
      if (solax_data.inverter.msg_1871.data[MSG_1871_CONTACTOR] == 0x01)
        contactor_close = true;
      else
        contactor_close = false;
    break;

    case 0x03: /* Time Update (heartbeat) */
      memcpy(&solax_data.inverter.msg_1871_3,
             &solax_data.inverter.msg_1871,
             sizeof(solax_data.inverter.msg_1871_3));
    break;

    case 0x05: /* Send BMS IDs */
    {
      int i;

      for (i=0; i<=solax_data.bms.msg_1875.num_batts; ++i)
      {
        solax_data.bms.msg_1881.serial[0] = i;
        solax_data.bms.msg_1882.serial[0] = i;
        solax_data.bms.msg_1882.serial[7] = 0x30 + i;
        solax_send_message(0x1881, (uint8_t*)&solax_data.bms.msg_1881, 8);
        solax_send_message(0x1882, (uint8_t*)&solax_data.bms.msg_1882, 8);
      }
    }
    break;

    default:
      snprintf(last_error, ERROR_LEN,
                "1871 frame 0x%02X received from inverter.",
                solax_data.inverter.msg_1871.frame_id);
    break;
  }
}

/**
  * @brief  Process CAN2 Solax data
  * @param  None
  * @retval None
  */
void solax_process(void)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t data[8];
  uint8_t msg_limit = 10;

  while (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO1) > 0)
  {
    /* Read the message */
    if (HAL_OK == HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &RxHeader, data))
    {
#ifdef DEBUG_SOLAX
      //printf("> 0x%04lX (%ld): ", RxHeader.ExtId, RxHeader.DLC);
      //dump_packet(data, RxHeader.DLC);
#endif

      if (RxHeader.IDE != CAN_ID_EXT)
      {
        snprintf(last_error, ERROR_LEN,
                 "Unexpected CAN message: 0x%04lX, len %ld",
                 RxHeader.StdId, RxHeader.DLC);
        continue;
      }

      switch (RxHeader.ExtId)
      {
        case 0x1871:
          memcpy(&solax_data.inverter.msg_1871, data, 8);
          solax_process_frame();
        break;

        default:
          snprintf(last_error, ERROR_LEN,
                   "Unhandled CAN message from Inverter: 0x%08lX, len %ld",
                   RxHeader.ExtId, RxHeader.DLC);
        break;
      }
    }

    if (msg_limit-- == 0)
      break;
  }

  /* Update our data and send BMS messages. */
  if (HAL_GetTick() > bms_update + SOLAX_UPDATE_RATE && 
      HAL_GetTick() < last_update + SOLAX_UPDATE_RATE) 
  {
    bms_update = HAL_GetTick();

    /* Update voltage / current values */
    solax_update_values();

    /* Update the state machine */
    solax_update_state();

    /* Send CAN messages */
    solax_send_standard_response();
  }

  /* Shut down if we timeout receiving messages */
  if (HAL_GetTick() > last_update + SOLAX_TIMEOUT && (state > SOLAX_BATTERY_ANNOUNCE))
  {
    snprintf(last_error, ERROR_LEN,
              "No CAN messages received in %lds", (HAL_GetTick() - last_update) / 1000);
    state = SOLAX_BATTERY_ANNOUNCE;
  }
}

bool solax_init(void)
{
  return true;
}

/**
  * @brief  Set the maximum current to be drawn from the EVSE
  * @param  current Max current (A x1)
  * @retval None
  */
void solax_set_max_ac_current(uint8_t current)
{
  max_ac_power = current * 240;

  if (current == 0)
    t_zero_set = HAL_GetTick();
}


/**
  * @brief  Set the maximum current to be put into the battery
  * @param  current Max current (A x10)
  * @retval None
  */
void solax_set_max_dc_chg_current(uint16_t current)
{
  max_charge_current = current;

  if (current == 0)
    t_zero_set = HAL_GetTick();
}

/**
  * @brief  Set the maximum current to be taken from the battery
  * @param  current Max current (A x10)
  * @retval None
  */
void solax_set_max_dc_dis_current(uint16_t current)
{
  max_discharge_current = current;

  if (current == 0)
    t_zero_set = HAL_GetTick();
}

/**
  * @brief  Set the Battery voltage Target
  * @param  voltage Battery voltage in V x10
  * @retval None
  */
void solax_set_battery_voltage_tgt(uint16_t voltage)
{
  solax_data.bms.msg_1872.slave_voltage_max = voltage;
}

/**
  * @brief  Set the maximum Battery voltage
  * @param  voltage Battery voltage in V x10
  * @retval None
  */
void solax_set_battery_voltage_max(uint16_t voltage)
{
  solax_data.bms.msg_1878.pack_voltage_max = voltage;
}

/**
  * @brief  Set the minimum Battery voltage
  * @param  voltage Battery voltage in V x10
  * @retval None
  */
void solax_set_battery_voltage_min(uint16_t voltage)
{
  solax_data.bms.msg_1872.slave_voltage_min = voltage;
}

/**
  * @brief  Set the maximum (full) battery capacity
  * @param  energy  Battery full capacity in Wh
  * @retval None
  */
void solax_set_battery_capacity_max(uint32_t energy)
{
  solax_data.bms.msg_1878.wh_total = energy;
}

/**
  * @brief  Set the remaining battery capacity
  * @param  energy  Battery energy remaining in Wh
  * @retval None
  */
void solax_set_battery_capacity(uint32_t energy)
{
  solax_data.bms.msg_1873.energy = energy / 10;
}

/**
  * @brief  Set the Battery SoC
  * @param  soc     Battery SoC in % x1
  * @retval None
  */
void solax_set_battery_soc(uint16_t soc)
{
  solax_data.bms.msg_1873.soc = soc;
}

/**
  * @brief  Send JSON message with Inverter Data
  * @retval None
  */
void solax_json_update(void)
{
  printf("\"solax\":{");

  printf("\"state\":%d", state);

  if (strnlen(last_error, ERROR_LEN))
  {
    printf(",\"last_error\":\"%s\"", last_error);
  }

#ifdef DEBUG_SOLAX
  printf(",\"last_update\":%ld", HAL_GetTick() - last_update);
  printf(",\"contactor_req\":%d", contactor_close);
  printf(", \"max_chg_current\":%d, \"max_dis_current\":%d",
         solax_data.bms.msg_1872.charge_max,
         solax_data.bms.msg_1872.discharge_max);

  printf(", \"voltage\":%d, \"current\":%d",
          solax_data.bms.msg_1873.voltage,
          solax_data.bms.msg_1873.current);


  /* Time information from Inverter */
  printf(", \"date\":\"%04d/%02d/%02d\", \"time\":\"%02d:%02d:%02d\"",
    solax_data.inverter.msg_1871_3.data[1] + 2000,
    solax_data.inverter.msg_1871_3.data[2],
    solax_data.inverter.msg_1871_3.data[3],
    solax_data.inverter.msg_1871_3.data[4],
    solax_data.inverter.msg_1871_3.data[5],
    solax_data.inverter.msg_1871_3.data[6]);
#endif
  printf("}");
}

/**
  * @brief  Does the inverter allow us to close the concactor?
  * @retval bool true: Yes, false: No
  */
bool solax_contactor_enabled(void)
{
  return contactor_close;
}
