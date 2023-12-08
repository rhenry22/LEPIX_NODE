/** @file chademo.c
 *  @brief Functions to interact with ChaDeMo connection
 *
 *  This contains logic and CAN bus message data to communicate with a 
 *  Chademo 1.0 Vehicle.
 *  It also includes a state machine to control the Analog and Digital handshake.
 * 
 *  Inspired by the description of Type 2 connectors here:
 *  https://www.elso.sk/en/blog/technologies/evse-charging-of-electric-vehicles
 *
 *  @author Richard Taylor <richard@artaylor.co.uk>
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "can.h"
#include "chademo.h"
#include "solax.h"
#include "sensor.h"

#define DEBOUNCE_TIME   (50)
#define CHADEMO_CAN_TIMEOUT (10000)

#define LEAKAGE_CURRENT_MAX  (500)

/* MSG ID 0x102 Bits */
#define FAULT_OVER_VOLT       (1 << 0)
#define FAULT_UNDER_VOLT      (1 << 1)
#define FAULT_CURRENT         (1 << 2)
#define FAULT_OVER_TEMP       (1 << 3)
#define FAULT_VOLTAGE         (1 << 4)

#define STATUS_CHARGE         (1 << 0)
#define STATUS_NOT_PARKED     (1 << 1)
#define STATUS_MALFUNCTION    (1 << 2)
#define STATUS_CONTACTOR_OPEN (1 << 3)
#define STATUS_CHG_STOP       (1 << 4)

/* MSG ID 0x109 Bits */
#define MSG109_CHARGE         (1 << 0)
#define MSG109_FAULT          (1 << 1)
#define MSG109_CONN_LOCK      (1 << 2)
#define MSG109_BATT_INCOMPAT  (1 << 3)
#define MSG109_CHG_MALFUNC    (1 << 4)
#define MSG109_CHG_STOPPED    (1 << 5)

struct can_data
{
  bool std_data;
  bool v2x_data;

  struct _vehicle
  {
    struct
    {
        uint8_t reserved[4];
        uint16_t max_battery_voltage;   /* Maximum voltage (V x1) */
        uint8_t charge_rate_indication; /* Const 100 (100%) */
        uint8_t reserved2;
    } msgid_100;

    struct
    {
        uint8_t reserved;
        uint8_t max_charging_time_10s;
        uint8_t max_charging_time_1min;
        uint8_t estimated_charge_time_1min;
        uint8_t reserved2;
        uint16_t rated_battery_capacity;  /* Capacity when full (0.1kWh) */
        uint8_t reserved3;
    } msgid_101;

    struct
    {
        uint8_t chademo_version;
        uint16_t target_battery_voltage;  /* Target Voltage (V x1) */
        uint8_t charge_current_requested; /* Requested Current (A x1) */
        uint8_t faults;
        uint8_t status;
        uint8_t charge_rate;              /* Battery SoC (% x1) */
        uint8_t reserved;
    } msgid_102;

    /* Chademo v2.0 only */
    struct
    {
        uint8_t status;
        uint8_t pad[7];
    } msgid_110;

    struct
    {
        uint8_t max_discharge_current;    /* Max Discharge (A x1) */
        uint16_t min_discharge_voltage;   /* Min Voltage (V x1) */
        uint8_t min_discharge_level;      /* Min SoC (% x1) */
        uint8_t max_remaining_capacity;   /* Max Available SoC? */
        uint8_t pad[3];
    } msgid_200;

    struct
    {
        uint8_t v2x_sequence_num;
        uint16_t estimated_discharge_time;
        uint16_t available_energy;
        uint8_t pad[3];
    } msgid_201;

    struct
    {
        uint8_t manufacturer_code;
        uint8_t pad[7];
    } msgid_700;
  } vehicle;

  struct _charger
  {
    struct
    {
        uint8_t welding_detection;            /* 0x00: Not Supported, else supported */
        uint16_t available_charger_voltage;   /* V x1 */
        uint8_t available_charger_current;    /* A x1 */
        uint16_t threshold_voltage;           /* Fault Threshold Voltage (Max) V x1 */
        uint8_t pad[2];
    } msgid_108;

    struct
    {
        uint8_t chademo_version;
        uint16_t charger_voltage;             /* Battery voltage (V x1) */
        uint8_t charger_current;              /* Battery voltage (A x1) */
        uint8_t reserved;
        uint8_t fault_status;
        uint8_t time_remaining_10s;
        uint8_t time_remaining_1min;
    } msgid_109;

    /* Chademo v2.0 only */
    struct
    {
        uint8_t pad[8];
    } msgid_118;

    struct
    {
        uint8_t pad[8];
    } msgid_208;

    struct
    {
        uint8_t pad[8];
    } msgid_209;
  } charger;
};

struct chademo_message
{
  uint16_t id;
  uint8_t data[8];
};

static bool initialised = false;
static uint32_t last_update;
static CHADEMO_STATE chademo_state;
static uint32_t leak_base;
static struct can_data can_data;

static uint32_t chg_perm_tick = 0;
static bool chg_perm = false;

/* For chademo v2.0 only */
static struct chademo_message chademo_118 = {0x118, {0x10, 0x64, 0x00, 0xB0, 0x00, 0x1E, 0x00, 0x8F}};
/* For V2X */
static struct chademo_message chademo_208 = {0x208, {0xFF, 0xF4, 0x01, 0xF0, 0x00, 0x00, 0xFA, 0x00}};
static struct chademo_message chademo_209 = {0x209, {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

static HAL_StatusTypeDef chademo_send_message(uint32_t id, uint8_t* data)
{
  CAN_TxHeaderTypeDef TxHeader;

  TxHeader.DLC = 8;
  TxHeader.StdId = id;

  return MX_CAN_Transmit(&hcan1, &TxHeader, data);
}

/**
  * @brief  Update the ChaDeMo State Machine
  * @param  state The new state to transition to
  * @retval None
  */
static void chademo_transition_state(CHADEMO_STATE new_state)
{
  bool update_state = true;

  switch (new_state)
  {
    case CHADEMO_STATE_OFF:
      if (chademo_state >= CHADEMO_STATE_ON)
      {
        printf("ChaDeMo: Stop Inverter.\n");
        solax_set_max_dc_chg_current(0);
        solax_set_max_dc_dis_current(0);

        /* Wait for current to drop below 5A (Sensor is A x10) */
        while (sensor_get_value(SENSOR_BATT_CURRENT) > 50);
        printf("ChaDeMo: Current dropped to < 5A.\n");
        // ToDo: Add a timeout!
      }

      can_data.charger.msgid_109.fault_status &= ~MSG109_CHARGE;

      /* This forcibly opens the contactors, so if current is not zero there is a welding risk. */
      HAL_GPIO_WritePin(CHADEMO_SEQ2_GPIO_Port, CHADEMO_SEQ2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(CHADEMO_SEQ1_GPIO_Port, CHADEMO_SEQ1_Pin, GPIO_PIN_RESET);
      printf("ChaDeMo: SEQx Off.\n");

      HAL_GPIO_WritePin(GPIOE, CHADEMO_Pin, GPIO_PIN_SET);

      /* Give it time for the voltage to drop */
      HAL_Delay(5000);

      /*  Check for contact welding */
      // ToDo: Check in a timeout loop rather than a fixed delay
      if (sensor_get_value(SENSOR_BATT_VOLTAGE) <= 100)
      {
        /* Unlock connector */
        HAL_GPIO_WritePin(CHADEMO_LOCK_GPIO_Port, CHADEMO_LOCK_Pin, GPIO_PIN_RESET);
        can_data.charger.msgid_109.fault_status &= ~MSG109_CONN_LOCK;
        printf("ChaDeMo: Connector Unlocked.\n");
      }
      else
      {
        printf("ChaDeMo: ERROR: Fault Detected, not Unlocking!\n");
        can_data.charger.msgid_109.fault_status |= MSG109_FAULT;
      }
    break;

    case CHADEMO_STATE_START:
      if (chademo_state == CHADEMO_STATE_OFF)
      {
        can_data.charger.msgid_109.fault_status = MSG109_CHG_STOPPED;

        /* Signal to the vehicle that we're ready to start */
        printf("ChaDeMo: CP Ready.\n");
        HAL_GPIO_WritePin(CHADEMO_SEQ1_GPIO_Port, CHADEMO_SEQ1_Pin, GPIO_PIN_SET);
        HAL_Delay(200);
        chademo_transition_state(CHADEMO_STATE_PARAM_CHK);
        update_state = false;
      }
      else
      {
        printf("ChaDeMo: User Stopped charge.\n");
        chademo_transition_state(CHADEMO_STATE_OFF);
        update_state = false;
      }
    break;

    case CHADEMO_STATE_PARAM_CHK:
      printf("ChaDeMo: Checking parameters.\n");
      // ToDo: Wait for CAN data to become valid, with a timeout.
      if (can_data.std_data)
      {
        printf("ChaDeMo: Received Standard packets\n");
        // ToDo: Dump Std Data
      }
      if (can_data.v2x_data)
      {
        printf("ChaDeMo: Received V2X packets\n");
        // ToDo: Dump V2X Data
      }
    break;

    case CHADEMO_STATE_PERM_OK:
      if (chademo_state == CHADEMO_STATE_PARAM_CHK)
      {
        printf("ChaDeMo: Vehicle Permission OK.\n");
        printf("ChaDeMo: Start insulation test.\n");
        
        /* Enable HV DCDC Test source */
        HAL_GPIO_WritePin(TEST_HV_EN_GPIO_Port, TEST_HV_EN_Pin, GPIO_PIN_SET);

        HAL_Delay(1000);

        /* Store the HV DCDC current before applying to connector */
        leak_base = sensor_get_value(SENSOR_HV_TEST_CURRENT);

        /* Start Test (run for at least 2s) */
        HAL_GPIO_WritePin(LEAK_TEST_EN_GPIO_Port, LEAK_TEST_EN_Pin, GPIO_PIN_SET);
        HAL_Delay(2000);

        chademo_transition_state(CHADEMO_STATE_INS_TEST);
        update_state = false;
      }
      else
      {
        printf("ChaDeMo: ERROR: Invalid state for Vehicle Permission.\n");
        printf("ChaDeMo: Aborting.\n");
        /* Invalid state for OK signal, abort. */
        chademo_transition_state(CHADEMO_STATE_OFF);
        update_state = false;
      }
    break;

    case CHADEMO_STATE_INS_TEST:
    {
      uint32_t leak_current = sensor_get_value(SENSOR_HV_TEST_CURRENT);

      /* Disable HV Test */
      HAL_GPIO_WritePin(LEAK_TEST_EN_GPIO_Port, LEAK_TEST_EN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(TEST_HV_EN_GPIO_Port, TEST_HV_EN_Pin, GPIO_PIN_RESET);

      /* Check that HV Test current is below threshold */
      if (leak_current > leak_base + LEAKAGE_CURRENT_MAX)
      {
        printf("ChaDeMo: ERROR: Earth leakage test failed. Aborting.\n");
        chademo_transition_state(CHADEMO_STATE_OFF);  
        update_state = false;
      }
      else
      {
        printf("ChaDeMo: Insulation test pass, lock plug and enable HV contactors.\n");
        HAL_GPIO_WritePin(CHADEMO_LOCK_GPIO_Port, CHADEMO_LOCK_Pin, GPIO_PIN_SET);      
        HAL_GPIO_WritePin(CHADEMO_SEQ2_GPIO_Port, CHADEMO_SEQ2_Pin, GPIO_PIN_SET);

        can_data.charger.msgid_109.fault_status |= MSG109_CONN_LOCK;

        chademo_transition_state(CHADEMO_STATE_ON);
        update_state = false;
      }
    }
    break;

    case CHADEMO_STATE_ON:
      can_data.charger.msgid_109.fault_status |= MSG109_CHARGE;
      printf("ChaDeMo: Charging!\n");
    break;
  }

  if (update_state)
  {
    chademo_state = new_state;
  }
}

/**
  * @brief  Initialise ChaDeMo Interface
  * @param  None
  * @retval bool true: Success, false: Failure
  */
bool chademo_init(void)
{
  /* Set up our CAN response messages */

  memset(&can_data, 0, sizeof(can_data));

  can_data.charger.msgid_108.welding_detection = 0x01;
  can_data.charger.msgid_108.available_charger_voltage = 500;
  can_data.charger.msgid_108.available_charger_current = 0; // 15?
  can_data.charger.msgid_108.threshold_voltage = 435;

  can_data.charger.msgid_109.chademo_version = 0x02;
  can_data.charger.msgid_109.charger_voltage = 0;
  can_data.charger.msgid_109.charger_current = 0;
  can_data.charger.msgid_109.fault_status = MSG109_CHG_STOPPED;
  can_data.charger.msgid_109.time_remaining_10s = 0xff;
  can_data.charger.msgid_109.time_remaining_1min = 0xff;

  memcpy(&can_data.charger.msgid_118, chademo_118.data, 8);
  memcpy(&can_data.charger.msgid_208, chademo_208.data, 8);
  memcpy(&can_data.charger.msgid_209, chademo_209.data, 8);

  initialised = true;

  return initialised;
}

/**
  * @brief  Receive ChaDeMo data
  * @retval None
  */
void chademo_process(void)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t data[8];
  uint32_t cur_tick = HAL_GetTick();

  /* Check that we are receiving regular CAN messages from ChaDeMo and Inverter */
  if (chademo_state == CHADEMO_STATE_ON && 
      cur_tick > (last_update + CHADEMO_CAN_TIMEOUT))
  {
      printf("ChaDeMo: ERROR: CAN message timeout. Aborting.\n");
      chademo_transition_state(CHADEMO_STATE_OFF);
  }

  /* Check Vehicle Charge Permission input */
  if (HAL_GPIO_ReadPin(CHADEMO_CHARGE_ALLOWED__GPIO_Port, CHADEMO_CHARGE_ALLOWED__Pin) == GPIO_PIN_RESET)
  {
    if (!chg_perm)
    {
      printf("ChaDeMo: Vehicle Ready.\n");
      chademo_transition_state(CHADEMO_STATE_PERM_OK);
    }
    chg_perm = true;
    chg_perm_tick = cur_tick + DEBOUNCE_TIME;
  }
  else if (cur_tick > chg_perm_tick)
  {
    if (chg_perm)
    {
      printf("ChaDeMo: Vehicle NOT Ready.\n");
      chademo_transition_state(CHADEMO_STATE_OFF);
    }
    chg_perm = false;
  }


  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
  {
    int i;

    /* Read the message */
    if (HAL_OK == HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, data))
    {
      if (RxHeader.IDE != CAN_ID_STD)
      {
        printf("ChaDeMo: ERROR: Unexpected CAN message\n");
        continue;
      }

      printf("ChaDeMo: ChaDeMo Packet: ID: 0x%02lX\nData: ", RxHeader.StdId);
      for (i=0; i<RxHeader.DLC; ++i)
      {
        printf("0x%02X ", data[i]);
      }
      printf("\n");

      switch (RxHeader.StdId) 
      {
        case 0x100:
          if (sizeof(can_data.vehicle.msgid_100) >= RxHeader.DLC)
          {
            memcpy(&can_data.vehicle.msgid_100, data, RxHeader.DLC);

            solax_set_battery_voltage_max(can_data.vehicle.msgid_100.max_battery_voltage * 10);
          }
          break;

        case 0x101:
          if (sizeof(can_data.vehicle.msgid_101) >= RxHeader.DLC)
          {
            memcpy(&can_data.vehicle.msgid_101, data, RxHeader.DLC);

            solax_set_battery_capacity_max(can_data.vehicle.msgid_101.rated_battery_capacity * 100);
          }
          break;

        case 0x102:
          if (sizeof(can_data.vehicle.msgid_102) >= RxHeader.DLC)
          {
            memcpy(&can_data.vehicle.msgid_102, data, RxHeader.DLC);

            solax_set_max_dc_chg_current(can_data.vehicle.msgid_102.charge_current_requested * 10);
            solax_set_battery_soc(can_data.vehicle.msgid_102.charge_rate);
            solax_set_battery_voltage_tgt(can_data.vehicle.msgid_102.target_battery_voltage * 10);

            /* Let logic know we've received our v1.0 data */
            can_data.std_data = true;
          }
          break;

        case 0x200:  /* For V2X */
          if (sizeof(can_data.vehicle.msgid_200) >= RxHeader.DLC)
          {
            memcpy(&can_data.vehicle.msgid_200, data, RxHeader.DLC);

            solax_set_battery_voltage_min(can_data.vehicle.msgid_200.min_discharge_voltage * 10);
          }
          break;

        case 0x201:  /* For V2X */
          if (sizeof(can_data.vehicle.msgid_201) >= RxHeader.DLC)
          {
            memcpy(&can_data.vehicle.msgid_201, data, RxHeader.DLC);

            solax_set_battery_capacity(can_data.vehicle.msgid_201.available_energy * 100);

            /* Let logic know we've received our v1.0.1+ data */
            can_data.v2x_data = true;
          }
          break;

        case 0x700:
          if (sizeof(can_data.vehicle.msgid_700) >= RxHeader.DLC)
          {
            memcpy(&can_data.vehicle.msgid_700, data, RxHeader.DLC);
          }
          break;

        case 0x110:  /* Only present on Chademo v2.0 */
          if (sizeof(can_data.vehicle.msgid_110) >= RxHeader.DLC)
          {
            memcpy(&can_data.vehicle.msgid_110, data, RxHeader.DLC);
          }
          break;

        default:
          printf("ChaDeMo: Unknown Message: 0x%02lX\n", RxHeader.StdId);
          break;
      }
    }

    /* Check for Faults */
    if (can_data.std_data || can_data.v2x_data)
    {
      if (can_data.vehicle.msgid_102.faults || 
          (can_data.vehicle.msgid_102.status & ~STATUS_CHARGE))
      {
        printf("ChaDeMo: Aborting Charge. Vehicle Faults: 0x%02X, Status: 0x%02X\n", 
              can_data.vehicle.msgid_102.faults, 
              can_data.vehicle.msgid_102.status);

        chademo_transition_state(CHADEMO_STATE_OFF);
        can_data.std_data = false;
        can_data.v2x_data = false;
      }
    }

    /* Respond to standard messages */
    if (can_data.std_data)
    {
      last_update = HAL_GetTick();

      if (chademo_state >= CHADEMO_STATE_PARAM_CHK)
      {
        HAL_GPIO_WritePin(GPIOE, CHADEMO_Pin, GPIO_PIN_RESET);
      }

      /* Reset the remaining charge time, unless we're charging. */
      if (chademo_state < CHADEMO_STATE_ON)
      {
        // ToDo: Consider decrementing the minute counter!
        can_data.charger.msgid_109.time_remaining_10s = 0xff;
        can_data.charger.msgid_109.time_remaining_1min = 0xff;
      } 
      else
      {
        /* Normal Operation */
        can_data.charger.msgid_109.charger_voltage = sensor_get_value(SENSOR_BATT_VOLTAGE) / 10;
        can_data.charger.msgid_109.charger_current = sensor_get_value(SENSOR_BATT_CURRENT) / 10;

        // ToDo: Consider decrementing the minute counter!
        //can_data.charger.msgid_109.time_remaining_10s = 0xff;
        //can_data.charger.msgid_109.time_remaining_1min = 0xff;
      }

      chademo_send_message(0x108, (uint8_t*)&can_data.charger.msgid_108);
      chademo_send_message(0x109, (uint8_t*)&can_data.charger.msgid_109);

      if (can_data.vehicle.msgid_102.chademo_version >= 0x03) 
      {
        /* Only send the following on Chademo 2.0 vehicles? */
        chademo_send_message(0x118, (uint8_t*)&can_data.charger.msgid_118);
      }
    }

    /* Respond to V2X messages */
    if (can_data.v2x_data)
    {
      solax_set_max_dc_dis_current(can_data.vehicle.msgid_200.max_discharge_current);

      chademo_send_message(0x208, (uint8_t*)&can_data.charger.msgid_208);
      chademo_send_message(0x209, (uint8_t*)&can_data.charger.msgid_209);
    }
  }
}

/**
  * @brief  Start the ChaDeMo Session
  * @param  None
  * @retval None
  */
void chademo_start(void)
{
  if (initialised)
    chademo_transition_state(CHADEMO_STATE_START);
}

/**
  * @brief  Stop the ChaDeMo Session
  * @param  None
  * @retval None
  */
void chademo_stop(void)
{
  if (initialised)
    chademo_transition_state(CHADEMO_STATE_OFF);
}

/**
  * @brief  Max power to/from EVSE
  * @param  None
  * @retval None
  */
void chademo_set_max_power(uint16_t power)
{
  uint16_t current = 0;

  if (initialised)
  {
    if (can_data.charger.msgid_109.charger_voltage > 0)
      current = power / can_data.charger.msgid_109.charger_voltage;

    can_data.charger.msgid_108.available_charger_current = current;
  }
}

/**
  * @brief  Get the current state of the ChaDeMo interface
  * @param  None
  * @retval CHADEMO_STATE Current State
  */
CHADEMO_STATE chademo_get_state(void)
{
  return chademo_state;
}
