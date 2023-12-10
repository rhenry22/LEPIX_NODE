#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "can.h"
#include "sensor.h"
#include "chademo.h"
#include "solax.h"

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
#define SOLAX_UPDATE_RATE (50)

#define MSG_1871_STATUS     (1)
#define MSG_1871_CONTACTOR  (3)

typedef enum _solax_state
{
  SOLAX_BATTERY_ANNOUNCE,
  SOLAX_WAITING_FOR_CONTACTOR,
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
      uint16_t pack_temp; /* Temp x10 C */
      uint16_t num_batts; /* Number of batteries (7) */
      uint16_t contactor; /* Contactor on / off (0x1 / 0x0) */
      uint16_t reserved;
    } msg_1875;

    struct _bms_pack_temps
    {
      uint16_t reserved;
      uint16_t cell_mv_max;   /* mV */
      uint16_t reserved2;
      uint16_t cell_mv_min;   /* mV */
    } msg_1876;

    struct _bms_version
    {
      uint16_t reserved;
      uint16_t reserved2;
      uint16_t id;  /* Battery Type? (0x50) */
      uint16_t data;  /* FW Version? (0x0222) */
    } msg_1877;

    struct _bms_pack_stats
    {
      uint16_t pack_voltage_max;  /* Voltage x10 V */
      uint16_t reserved;
      uint32_t wh_total;
    } msg_1878;

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
      .data = {0x0002, 0x0001, 0x0001}
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
      .num_batts = 0, // 7?
      .contactor = 0
    },

    /* BMS_PackTemps (Cell voltages) */
    .msg_1876 = {
      .reserved = 1
    },

    /* BMS_Version */
    .msg_1877 = {
      .id = 0x50,
      .data = 0x0222
    },

    /* BMS_PackStats */
    .msg_1878 = {
      .pack_voltage_max = ABSOLUTE_MAX_VOLTAGE / 100,
      .wh_total = BATTERY_WH_MAX
    },

    /* BMS_Serial */
    .msg_1881 = { .serial = { 0x00, 0x35, 0x53, 0x42, 0x4D, 0x53, 0x46, 0x41 }},
    .msg_1882 = { .serial = { 0x00, 0x31, 0x33, 0x41, 0x42, 0x30, 0x35, 0x32 }},

    /* The following will be updated once ChaDeMo starts up */

    /* BMS_Limits */
    .msg_1872 = {
      .charge_max = 1,
      .discharge_max = 1
    },

    /* BMS_PackData */ 
    .msg_1873 = {
      .voltage = 3800,
      .current = 0,
      .soc = 50,
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
static uint16_t max_ac_power = 1000;                /* Maximum current limit advertised by EVSE */
static uint32_t t_zero_set = 0;                     /* Time at which current request set to zero (debug / check inverter response) */
static uint32_t last_update = 0;                    /* Last time we saw a CAN message */
static uint32_t last_time_update = 0;               /* Last time we saw frame 0x03 */
static uint16_t max_charge_current = 0;             /* Max DC charge current (A x10) */
static uint16_t max_discharge_current = 0;          /* Max DC discharge current (A x10) */

static HAL_StatusTypeDef solax_send_message(uint32_t id, uint8_t *data, uint8_t len)
{
  CAN_TxHeaderTypeDef TxHeader;
  HAL_StatusTypeDef ret;

  TxHeader.DLC = len;
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.ExtId = id;

  ret = MX_CAN_Transmit(&hcan2, &TxHeader, data);

  return ret;
}

static HAL_StatusTypeDef solax_send_standard_response(void)
{
  HAL_StatusTypeDef ret;
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

  return ret;
}

static void solax_update_values(void)
{
  uint32_t voltage;
  uint32_t current;

  /* Update Measured Values */
  voltage = sensor_get_value(SENSOR_BATT_VOLTAGE);
  current = sensor_get_value(SENSOR_BATT_CURRENT);

  /* BMS_PackData */
  solax_data.bms.msg_1873.voltage = voltage;
  solax_data.bms.msg_1873.current = current;

  /* BMS_PackTemps (Cell voltages) */
  solax_data.bms.msg_1876.cell_mv_max = (voltage / NUM_CELLS) + 40;
  solax_data.bms.msg_1876.cell_mv_min = (voltage / NUM_CELLS) - 40;
  
  // ToDo: Add some form of temperature monitoring / reporting
#if 0
  //BMS_Status
  SOLAX_1875.data.u8[0] = (uint8_t)temperature_average;
  SOLAX_1875.data.u8[1] = (temperature_average >> 8);
#endif

  /* Ensure we're within EVSE limits */
  {
    uint32_t req_current;
    uint32_t req_power;

    /* Charge */
    req_current = max_charge_current;
    req_power = voltage * req_current;
    if (req_power > max_ac_power)
      req_current = max_ac_power / voltage;

    solax_data.bms.msg_1872.charge_max = req_current;

    /* Discharge */
    req_current = max_discharge_current;
    req_power = voltage * req_current;
    if (req_power > max_ac_power)
      req_current = max_ac_power / voltage;

    solax_data.bms.msg_1872.discharge_max = req_current;
  }

  /* Check SoC and adjust charge rate if needed */
  if (solax_data.bms.msg_1873.soc >= 95)
    solax_data.bms.msg_1872.charge_max = 0;

  if (solax_data.bms.msg_1873.soc <= 20)
    solax_data.bms.msg_1872.discharge_max = 0;
}

static HAL_StatusTypeDef solax_update_state(void)
{
  HAL_StatusTypeDef ret;

  /* Update the contactor state */
  if (chademo_is_contactor_closed())
    solax_data.bms.msg_1875.contactor = 1;
  else
    solax_data.bms.msg_1875.contactor = 0;

  if (solax_data.inverter.msg_1871.data[MSG_1871_STATUS] != 0x0001)
  {
    state = SOLAX_FAULT;
    printf("Solax: Unhandled Inverter Status: %d\n", 
      solax_data.inverter.msg_1871.data[MSG_1871_STATUS]);
    chademo_stop();
    ret = HAL_ERROR;
  }

  ret = solax_send_standard_response();

  switch (state) {
    case SOLAX_BATTERY_ANNOUNCE:
      printf("Solax: Battery State: Announce\n");

      //for (int i = 0; i < solax_data.bms.msg_1875.num_batts; i++) {
      //  solax_send_standard_response();      
      //}

      /* BMS Announce */
      solax_send_message(0x100A001, (uint8_t*)&solax_data.bms.msg_100A001, 0);

      if (solax_data.inverter.msg_1871.data[MSG_1871_CONTACTOR] == 0x0001)
      {
        /* Message from the inverter to proceed to contactor closing */
        chademo_start();
        state = SOLAX_WAITING_FOR_CONTACTOR;
      }
    break;

    case SOLAX_WAITING_FOR_CONTACTOR:
      printf("Solax: Battery State: Waiting for Contactor\n");

      /* Announce that the battery will be connected */
      solax_send_message(0x1801, (uint8_t*)&solax_data.bms.msg_1801, 8);
      state = SOLAX_CONTACTOR_CLOSING;
      break;

    case SOLAX_CONTACTOR_CLOSING:
      /* Stay in this state until ChaDeMo completes connection */
      if (solax_data.bms.msg_1875.contactor == 1)
      {
        printf("Solax: Battery State: Contactor Closed\n");
        state = SOLAX_CONTACTOR_CLOSED;
      }
    break;

    case SOLAX_CONTACTOR_CLOSED:
      if (solax_data.inverter.msg_1871.data[MSG_1871_CONTACTOR] == 0)
      {
        /* Message from the inverter to open contactor */
        printf("Solax: Battery State: Inverter Requests Open Contactor\n");
        state = SOLAX_BATTERY_ANNOUNCE;
        chademo_stop();
      }
    break;

    case SOLAX_FAULT:
    case SOLAX_UPDATING_FW:
    break;
  }

  return ret;
}

static void solax_process_frame(void)
{
  /* Process the frame */
  switch (solax_data.inverter.msg_1871.frame_id)
  {
    case 0x01:
    case 0x02:
      /* These are the frames that count */
      last_update = HAL_GetTick();

      /* Make sure our response messages are up to date. */
      if (HAL_GetTick() > last_update + SOLAX_UPDATE_RATE)
        solax_update_values();

      /* Update the state machine */
      solax_update_state();
    break;

    case 0x03:
      if (solax_data.inverter.msg_1871.data[0] == 0x06)
      {
        if (HAL_GetTick() > last_time_update + 1000)
        {
          last_time_update = HAL_GetTick();
          /* Time information from Inverter */
          printf("Solax: Time: %04d/%02d/%02d %02d:%02d:%02d\n", 
            solax_data.inverter.msg_1871.data[1] + 2000,
            solax_data.inverter.msg_1871.data[2],
            solax_data.inverter.msg_1871.data[3],
            solax_data.inverter.msg_1871.data[4],
            solax_data.inverter.msg_1871.data[5],
            solax_data.inverter.msg_1871.data[6]);
        }
      }
    break;

    case 0x05:
      /* Send BMS IDs */
      solax_send_message(0x1881, (uint8_t*)&solax_data.bms.msg_1881, 8);
      solax_send_message(0x1882, (uint8_t*)&solax_data.bms.msg_1882, 8);
    break;

    default:
      printf("Solax: 1871 frame 0x%02X received from inverter.\n",
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
    HAL_GPIO_WritePin(GPIOE, INVERTER_Pin, GPIO_PIN_RESET);

    /* Read the message */
    if (HAL_OK == HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &RxHeader, data))
    {
      if (RxHeader.IDE != CAN_ID_EXT)
      {
        printf("ERROR: Unexpected CAN message: 0x%04lX, len %ld\n", RxHeader.StdId, RxHeader.DLC);
        continue;
      }

#ifdef DEBUG_SOLAX
      {
        int i;
        printf("Solax Packet: ID: 0x%02lX\nData: ", RxHeader.ExtId);
        for (i=0; i<RxHeader.DLC; ++i)
        {
          printf("0x%02X ", data[i]);
        }
        printf("\n");
      }
#endif

      switch (RxHeader.ExtId)
      {
        case 0x1871:
          memcpy(&solax_data.inverter.msg_1871, data, 8);
          solax_process_frame();
        break;

        default:
          printf("Unhandled CAN message from Inverter: 0x%08lX, len %ld\n", RxHeader.ExtId, RxHeader.DLC);
        break;
      }
    }

    if (msg_limit-- == 0)
      break;
  }

  /* Shut down if we timeout receiving messages */
  if (HAL_GetTick() > last_update + SOLAX_TIMEOUT)
  {
    state = SOLAX_BATTERY_ANNOUNCE;
    chademo_stop();
  }

  HAL_GPIO_WritePin(GPIOE, INVERTER_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Set the maximum current to be drawn from the EVSE
  * @param  current Max current in Amps
  * @retval None
  */
void solax_set_max_ac_current(uint8_t current)
{
  uint32_t power_dc;

  max_ac_power = current * 240;

  /* Make sure we limit DC charge power to the EVSE limit */
  power_dc = solax_data.bms.msg_1872.charge_max * solax_data.bms.msg_1873.voltage;
  if (power_dc > max_ac_power)
    solax_data.bms.msg_1872.charge_max = (max_ac_power / solax_data.bms.msg_1873.voltage) * 10;

  /* Make sure we limit DC discharge power to the EVSE limit */
  power_dc = solax_data.bms.msg_1872.discharge_max * solax_data.bms.msg_1873.voltage;
  if (power_dc > max_ac_power)
    solax_data.bms.msg_1872.discharge_max = (max_ac_power / solax_data.bms.msg_1873.voltage) * 10;

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
}

/**
  * @brief  Set the maximum current to be taken from the battery
  * @param  current Max current (A x10)
  * @retval None
  */
void solax_set_max_dc_dis_current(uint16_t current)
{
  max_discharge_current = current;
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
