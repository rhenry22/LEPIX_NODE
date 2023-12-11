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
 *  ToDo: 
 *  Requires a user interface to show charging status and start / stop charging
 *  Requires Short Circuit and Isolation detection (not just earth leakage)
 *  Emergency Stop button (Red, latching)
 *  Start / Stop buttons (Blue, Green, illuminated)
 *  Separate CPU for Inverter Control and Interface (technically OK?)
 *  Analog handshake should go via connector lock detection and inverter shutoff in HW 
 *  (i.e. separate from CPU)
 *  
 *  Calibrate Earth Leakage to >50kOhm threshold value
 *  Monitor lock soleniod current and set CONN Lock flag to zero if it fails.
 *  
 *  Copyright (c) 2023 ARTaylor.co.uk.
 *  All rights reserved.
 * 
 *  @author Richard Taylor <richard@artaylor.co.uk>
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "usbd_cdc_if.h"  // for MIN
#include "can.h"
#include "chademo.h"
#include "solax.h"
#include "sensor.h"

#define DEBUG_CAN

#define MEASUREMENT_INTERVAL  (50)

#define DEBOUNCE_TIME         (50)
#define CHADEMO_CAN_TIMEOUT   (10000)
#define LEAKAGE_CURRENT_MAX   (500)
#define STOP_CURRENT          (50)    /* 5A (x10) */
#define BATT_CHECK_TIMEOUT    (1000)
#define STOP_TIMEOUT          (5000)

#define CONTACTOR_CLOSED_V    (500)
#define CONTACTOR_OPEN_V      (100)

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

#define ERROR_LEN             (128)

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

static uint32_t last_update;                /* Last time we saw a CAN message */
static CHADEMO_STATE chademo_state;         /* State Machine State */
static uint32_t leak_base;                  /* Baseline (Off) current of leakage HV module */
static struct can_data can_data;            /* Structure holding all CAN message data */

static bool chg_perm = false;               /* Vehicle Charge permission state */
static bool k_perm = false;                 /* Vehicle Charge permission state (K line only) */

static uint32_t state_time = 0;             /* Time that the last state transition happened */

static bool contactor_closed = false;       /* Whether we have potentially live DC */

static bool errored = false;                /* If we hit any errors, prevent starting again */
static char last_error[ERROR_LEN+1] = {0};  /* Last error string */

uint16_t max_evse_power = 0;                /* Maximum Power (W x1) from the EVSE */
int32_t measured_voltage = 0;               /* Voltage (V x10) */
int32_t measured_current = 0;               /* Current (A x10) */
static int32_t measured_power = 0;          /* Power (W x1) in (+'ve) or out (-'ve) of the battery */

/* For chademo v2.0 only */
static uint8_t chademo_118[8] = {0x10, 0x64, 0x00, 0xB0, 0x00, 0x1E, 0x00, 0x8F};
/* For V2X */
static uint8_t chademo_208[8] = {0xFF, 0xF4, 0x01, 0xF0, 0x00, 0x00, 0xFA, 0x00};
static uint8_t chademo_209[8] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static HAL_StatusTypeDef chademo_send_message(uint32_t id, uint8_t* data)
{
  CAN_TxHeaderTypeDef TxHeader;

  /* Make sure we clear the header to default */
  memset(&TxHeader, 0, sizeof(TxHeader));

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

  /* Used for Sensor reads */
  int32_t val;
  HAL_StatusTypeDef ret;

  state_time = HAL_GetTick();

  switch (new_state)
  {
    case CHADEMO_STATE_OFF:
      /* This forcibly opens the contactors, so if current is not zero there is a welding risk. */
      HAL_GPIO_WritePin(CHADEMO_SEQ2_GPIO_Port, CHADEMO_SEQ2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(CHADEMO_SEQ1_GPIO_Port, CHADEMO_SEQ1_Pin, GPIO_PIN_RESET);
      contactor_closed = false;


      /* These should already be off, but can be used as an emergency stop */
      HAL_GPIO_WritePin(LEAK_TEST_EN_GPIO_Port, LEAK_TEST_EN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(TEST_HV_EN_GPIO_Port, TEST_HV_EN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(CHADEMO_SEQ1_GPIO_Port, CHADEMO_SEQ1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(CHADEMO_SEQ2_GPIO_Port, CHADEMO_SEQ2_Pin, GPIO_PIN_RESET);

      /* Unlock connector */
      HAL_GPIO_WritePin(CHADEMO_LOCK_GPIO_Port, CHADEMO_LOCK_Pin, GPIO_PIN_RESET);

      /* Let the vehicle know we're unlocked */
      can_data.charger.msgid_109.fault_status &= ~MSG109_CONN_LOCK;
    break;

    case CHADEMO_STATE_START:
      if (!errored)
      {
        if (chademo_state == CHADEMO_STATE_OFF)
        {
          /* Signal to the vehicle that we're ready to start */
          HAL_GPIO_WritePin(CHADEMO_SEQ1_GPIO_Port, CHADEMO_SEQ1_Pin, GPIO_PIN_SET);
          chademo_transition_state(CHADEMO_STATE_PARAM_CHK);
          update_state = false;
        }
        else
        {
          chademo_transition_state(CHADEMO_STATE_STOP);
          update_state = false;
        }
      }
    break;

    case CHADEMO_STATE_PARAM_CHK:
      // ToDo: CAN CHG before Physical charge == fault
      // ToDo: Physical Charge before sending first CAN data == fault
    break;

    case CHADEMO_STATE_PERM_OK:
    {
      uint32_t max_current = 0;

      /* These values are on only allowed to change during Parameter checking */

      /* Set the Threshold Voltage */
      can_data.charger.msgid_108.threshold_voltage = 
        MIN(SOLAX_MAXIMUM_SUPPORTED_VOLTAGE, can_data.vehicle.msgid_100.max_battery_voltage);

      /*
       * Use the max battery voltage reported by the vehicle and max power
       * reported by the EVSE to calculate our maximum supported current.
       */
      if (can_data.vehicle.msgid_100.max_battery_voltage > 0)
        max_current = max_evse_power / can_data.vehicle.msgid_100.max_battery_voltage;
      can_data.charger.msgid_108.available_charger_current = max_current;

      /* Lock the connector */
      HAL_GPIO_WritePin(CHADEMO_LOCK_GPIO_Port, CHADEMO_LOCK_Pin, GPIO_PIN_SET);      
      can_data.charger.msgid_109.fault_status |= MSG109_CONN_LOCK;

      /*  Check for contact welding */
      if (measured_voltage > CONTACTOR_OPEN_V ||
          !(can_data.vehicle.msgid_102.status & (STATUS_CONTACTOR_OPEN)))
      {
        snprintf(last_error, ERROR_LEN,
                 "Pre-contactor close battery check failed. More than 10V present.");
        chademo_transition_state(CHADEMO_STATE_ERROR);
        update_state = false;
      }
      else
      {
        /* Enable HV DCDC Test source */
        HAL_GPIO_WritePin(TEST_HV_EN_GPIO_Port, TEST_HV_EN_Pin, GPIO_PIN_SET);
      }
    }
    break;

    case CHADEMO_STATE_INS_TEST_BASE:
    {
        /* Store the HV DCDC current before applying to connector */
        ret = sensor_get_value(SENSOR_HV_TEST_CURRENT, &val);

        if (ret == HAL_OK)
        {
          leak_base = val;

          /* Start Test */
          HAL_GPIO_WritePin(LEAK_TEST_EN_GPIO_Port, LEAK_TEST_EN_Pin, GPIO_PIN_SET);
        }
        else
        {
          snprintf(last_error, ERROR_LEN,
                  "Failed to get baseline leakage current.");
          chademo_transition_state(CHADEMO_STATE_ERROR);
          update_state = false;
        }
    }
    break;

    case CHADEMO_STATE_INS_TEST:
    {
      int32_t leak_current;
      
      ret = sensor_get_value(SENSOR_HV_TEST_CURRENT, &leak_current);

      /* Disable HV Test */
      HAL_GPIO_WritePin(LEAK_TEST_EN_GPIO_Port, LEAK_TEST_EN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(TEST_HV_EN_GPIO_Port, TEST_HV_EN_Pin, GPIO_PIN_RESET);

      /* Check that HV Test current is below threshold */
      if (ret != HAL_OK || leak_current > leak_base + LEAKAGE_CURRENT_MAX)
      {
        snprintf(last_error, ERROR_LEN,
                 "Earth leakage test failed.");
        chademo_transition_state(CHADEMO_STATE_ERROR);
        update_state = false;
      }
      else
      {
        contactor_closed = true;
        HAL_GPIO_WritePin(CHADEMO_SEQ2_GPIO_Port, CHADEMO_SEQ2_Pin, GPIO_PIN_SET);

        chademo_transition_state(CHADEMO_STATE_BATT_CHECK);
        update_state = false;
      }
    }
    break;

    case CHADEMO_STATE_BATT_CHECK:
    break;

    case CHADEMO_STATE_ON:
      can_data.charger.msgid_109.fault_status |= MSG109_CHARGE;
      can_data.charger.msgid_109.fault_status &= ~MSG109_CHG_STOPPED;
    break;

    case CHADEMO_STATE_STOP:
      /* Tell the inverter to stop */
      solax_set_max_dc_chg_current(0);
      solax_set_max_dc_dis_current(0);

      /* Let the vehicle know to stop charging (current ramp down) */
      can_data.charger.msgid_109.fault_status |= MSG109_CHG_STOPPED;
    break;

    case CHADEMO_STATE_WELD_CHECK:
    break;

    case CHADEMO_STATE_WAIT_K:
    case CHADEMO_STATE_WAIT_CONTACTOR:
    break;

    case CHADEMO_STATE_ERROR:
      errored = true;
      chademo_transition_state(CHADEMO_STATE_STOP);
      update_state = false;
    break;
  }

  if (update_state)
  {
    chademo_state = new_state;
  }
}

/**
  * @brief  Check Vehicle Charge Permission input (and CAN)
  * @param  state The new state to transition to
  * @retval None
  */
static void chademo_check_vehicle_permission(void)
{
  bool phy_ok = false;
  bool can_ok = false;
  
  /* Check physical Vehicle Charge Permission */
  if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(CHADEMO_CHARGE_ALLOWED__GPIO_Port, 
                                         CHADEMO_CHARGE_ALLOWED__Pin))
  {
    phy_ok = true;
  }
  /* Check CAN Vehicle Charge Permission */
  if ((can_data.vehicle.msgid_102.status & STATUS_CHARGE) == STATUS_CHARGE)
  {
    can_ok = true;
  }

  k_perm = phy_ok;

  /* Check for inconsistencies */
  if (chademo_state == CHADEMO_STATE_ON)
  {
    /* Vehicle has requested termination */
    if ((can_data.vehicle.msgid_102.status & STATUS_CHG_STOP) == STATUS_CHG_STOP)
    {
      chademo_transition_state(CHADEMO_STATE_STOP);
    }

    if (phy_ok != can_ok)
    {
      snprintf(last_error, ERROR_LEN,
               "Inconsistency seen between Phys and CAN permission signals");
      can_data.charger.msgid_109.fault_status |= MSG109_CHG_MALFUNC;
      chademo_transition_state(CHADEMO_STATE_STOP);
    }
  }

  if (phy_ok && can_ok)
  {
    chg_perm = true;
  }
  else
  {
    chg_perm = false;
  }
}

/**
  * @brief  Process any buffered CAN frames received
  * @retval None
  */
static void chademo_process_can(void)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t data[8];
  uint8_t msg_limit = 10;

  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
  {
    int i;

    /* Read the message */
    if (HAL_OK == HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, data))
    {
      if (RxHeader.IDE != CAN_ID_STD)
      {
        snprintf(last_error, ERROR_LEN, "Unexpected Ext CAN message");
        continue;
      }

#ifdef DEBUG_CAN
      printf("ChaDeMo: ChaDeMo Packet: ID: 0x%02lX\nData: ", RxHeader.StdId);
      for (i=0; i<RxHeader.DLC; ++i)
      {
        printf("0x%02X ", data[i]);
      }
      printf("\n");
#endif

      switch (RxHeader.StdId) 
      {
        case 0x100:
          assert_param(sizeof(can_data.vehicle.msgid_100) >= RxHeader.DLC);
          memcpy(&can_data.vehicle.msgid_100, data, RxHeader.DLC);
          break;

        case 0x101:
          assert_param(sizeof(can_data.vehicle.msgid_101) >= RxHeader.DLC);
          memcpy(&can_data.vehicle.msgid_101, data, RxHeader.DLC);
          break;

        case 0x102:
          assert_param(sizeof(can_data.vehicle.msgid_102) >= RxHeader.DLC);
          memcpy(&can_data.vehicle.msgid_102, data, RxHeader.DLC);

          /* Let logic know we've received our v1.0 data */
          can_data.std_data = true;
          break;

        case 0x200:  /* For V2X */
          assert_param(sizeof(can_data.vehicle.msgid_200) >= RxHeader.DLC);
          memcpy(&can_data.vehicle.msgid_200, data, RxHeader.DLC);
          break;

        case 0x201:  /* For V2X */
          assert_param(sizeof(can_data.vehicle.msgid_201) >= RxHeader.DLC);
          memcpy(&can_data.vehicle.msgid_201, data, RxHeader.DLC);

          /* Let logic know we've received our v1.0.1+ data */
          can_data.v2x_data = true;
          break;

        case 0x700:
          assert_param(sizeof(can_data.vehicle.msgid_700) >= RxHeader.DLC);
          memcpy(&can_data.vehicle.msgid_700, data, RxHeader.DLC);
          break;

        case 0x110:  /* Only present on Chademo v2.0 */
          assert_param(sizeof(can_data.vehicle.msgid_110) >= RxHeader.DLC);
          memcpy(&can_data.vehicle.msgid_110, data, RxHeader.DLC);
          break;

        default:
          snprintf(last_error, ERROR_LEN,
                   "Unknown Message: 0x%02lX", RxHeader.StdId);
          break;
      }
    }

    if (msg_limit-- == 0)
      break;
  }
}

/**
  * @brief  Send messages in response to ChaDeMo frames
  * @retval None
  */
HAL_StatusTypeDef chademo_send_responses(void)
{
  HAL_StatusTypeDef ret = HAL_OK;

  /* If we received any valid messages */
  if (can_data.std_data || can_data.v2x_data)
  {
    /* Check for Faults */
    if (ret != HAL_OK || can_data.vehicle.msgid_102.faults || 
        (can_data.vehicle.msgid_102.status & (STATUS_NOT_PARKED | STATUS_MALFUNCTION)) )
    {
      snprintf(last_error, ERROR_LEN,
            "Aborting Charge. Vehicle Faults: 0x%02X, Status: 0x%02X",
            can_data.vehicle.msgid_102.faults,
            can_data.vehicle.msgid_102.status);

      chademo_transition_state(CHADEMO_STATE_ERROR);
      can_data.v2x_data = false;
    }
  }

  /* Respond to standard messages */
  if (ret == HAL_OK && can_data.std_data)
  {
    can_data.std_data = false;

    last_update = HAL_GetTick();

    HAL_GPIO_WritePin(GPIOE, CHADEMO_Pin, GPIO_PIN_RESET);

    if (chademo_state == CHADEMO_STATE_ON)
    {
      /* Normal Operation */
      can_data.charger.msgid_109.charger_voltage = measured_voltage / 10;
      can_data.charger.msgid_109.charger_current = abs(measured_current / 10);

      // ToDo: Consider decrementing the minute counter!
      //can_data.charger.msgid_109.time_remaining_10s = 0xff;
      //can_data.charger.msgid_109.time_remaining_1min = 0xff;

      /* Update Solax Data (Standard) */
      solax_set_battery_voltage_max(can_data.vehicle.msgid_100.max_battery_voltage * 10);
      solax_set_battery_capacity_max(can_data.vehicle.msgid_101.rated_battery_capacity * 100);
      solax_set_max_dc_chg_current(can_data.vehicle.msgid_102.charge_current_requested * 10);
      solax_set_battery_soc(can_data.vehicle.msgid_102.charge_rate);
      solax_set_battery_voltage_tgt(can_data.vehicle.msgid_102.target_battery_voltage * 10);
    }

    ret = chademo_send_message(0x108, (uint8_t*)&can_data.charger.msgid_108);
    if (ret == HAL_OK)
      ret = chademo_send_message(0x109, (uint8_t*)&can_data.charger.msgid_109);

    if (ret == HAL_OK && can_data.vehicle.msgid_102.chademo_version >= 0x03) 
    {
      /* Only send the following on Chademo 2.0 vehicles? */
      ret = chademo_send_message(0x118, (uint8_t*)&can_data.charger.msgid_118);
    }

    HAL_GPIO_WritePin(GPIOE, CHADEMO_Pin, GPIO_PIN_SET);
  }

  /* Respond to V2X messages */
  if (ret == HAL_OK && can_data.v2x_data)
  {
    can_data.v2x_data = false;

    HAL_GPIO_WritePin(GPIOE, CHADEMO_Pin, GPIO_PIN_RESET);

    ret = chademo_send_message(0x208, (uint8_t*)&can_data.charger.msgid_208);
    if (ret == HAL_OK)
      ret = chademo_send_message(0x209, (uint8_t*)&can_data.charger.msgid_209);

    if (ret == HAL_OK && chademo_state == CHADEMO_STATE_ON)
    {
      /* Update Solax Data (V2X) */
      solax_set_battery_voltage_min(can_data.vehicle.msgid_200.min_discharge_voltage * 10);
      solax_set_max_dc_dis_current(can_data.vehicle.msgid_200.max_discharge_current);
      solax_set_battery_capacity(can_data.vehicle.msgid_201.available_energy * 100);
    }

    HAL_GPIO_WritePin(GPIOE, CHADEMO_Pin, GPIO_PIN_SET);
  }

  return ret;
}

/**
  * @brief  Initialise ChaDeMo Interface
  * @retval bool true: Success, false: Failure
  */
bool chademo_init(void)
{
  /* Set up our CAN response messages */

  memset(&can_data, 0, sizeof(can_data));

  can_data.charger.msgid_108.welding_detection = 0x01;
  can_data.charger.msgid_108.available_charger_voltage = SOLAX_MAXIMUM_SUPPORTED_VOLTAGE;
  can_data.charger.msgid_108.available_charger_current = 0;   /* This will be updated by EVSE */
  can_data.charger.msgid_108.threshold_voltage = SOLAX_MAXIMUM_SUPPORTED_VOLTAGE;

  can_data.charger.msgid_109.chademo_version = 0x02;
  can_data.charger.msgid_109.charger_voltage = 0;
  can_data.charger.msgid_109.charger_current = 0;
  can_data.charger.msgid_109.fault_status = MSG109_CHG_STOPPED;
  can_data.charger.msgid_109.time_remaining_10s = 0xff;
  can_data.charger.msgid_109.time_remaining_1min = 0xff;

  memcpy(&can_data.charger.msgid_118, chademo_118, 8);
  memcpy(&can_data.charger.msgid_208, chademo_208, 8);
  memcpy(&can_data.charger.msgid_209, chademo_209, 8);

  return true;
}

/**
  * @brief  Receive ChaDeMo data
  * @retval None
  */
void chademo_process(void)
{
  HAL_StatusTypeDef ret;
  static uint32_t last_measurement = 0;

  /* Process any CAN messages */
  chademo_process_can();
  
  /* Update our Voltage, Current and Power measurements */
  if (HAL_GetTick() > last_measurement + MEASUREMENT_INTERVAL)
  {
    ret = sensor_get_value(SENSOR_BATT_VOLTAGE, &measured_voltage);
    if (ret != HAL_OK)
      return;
    ret = sensor_get_value(SENSOR_BATT_CURRENT, &measured_current);
    if (ret != HAL_OK)
      return;
    measured_power = measured_voltage * measured_current / 100;
  }

  /* If we've started, respond to CAN messages */
  if (chademo_state >= CHADEMO_STATE_START)
  {
    /* Send any response messages */
    chademo_send_responses();
  }

  /* Check the Vehicle Permission GPIO */
  chademo_check_vehicle_permission();

  /* Time based state machine transitions */
  switch (chademo_state)
  {
    /* Wait for vehicle to grant permission */
    case CHADEMO_STATE_PARAM_CHK:
      if (HAL_GetTick() > state_time + CHADEMO_CAN_TIMEOUT)
      {
        snprintf(last_error, ERROR_LEN,
                 "Timed out waiting for CAN messages from Vehicle");
        /* No need for full shut down sequence, no HV involved yet */
        chademo_transition_state(CHADEMO_STATE_OFF);
      }

      if (chg_perm)
      {
        /* Check we're within limits */
        if (can_data.vehicle.msgid_100.max_battery_voltage < SOLAX_MAXIMUM_SUPPORTED_VOLTAGE &&
            can_data.vehicle.msgid_102.target_battery_voltage < SOLAX_MAXIMUM_SUPPORTED_VOLTAGE)
        {
          chademo_transition_state(CHADEMO_STATE_PERM_OK);
        }
        else
        {
          snprintf(last_error, ERROR_LEN, "Battery incompatible");
          can_data.charger.msgid_109.fault_status |= MSG109_BATT_INCOMPAT;
          chademo_transition_state(CHADEMO_STATE_STOP);
        }
      }
    break;

    /* Vehicle Permission granted, HV turned on, waiting 1000ms to settle */
    case CHADEMO_STATE_PERM_OK:
      if (HAL_GetTick() > state_time + 1000)
        chademo_transition_state(CHADEMO_STATE_INS_TEST_BASE);
    break;

    /* Leak Test Started waiting 1000ms for result */
    case CHADEMO_STATE_INS_TEST_BASE:
      if (HAL_GetTick() > state_time + 1000)
        chademo_transition_state(CHADEMO_STATE_INS_TEST);
    break;

    case CHADEMO_STATE_BATT_CHECK:
      if (measured_voltage > CONTACTOR_CLOSED_V)
      {
        chademo_transition_state(CHADEMO_STATE_ON);
      }
      else if (HAL_GetTick() > state_time + BATT_CHECK_TIMEOUT)
      {
        snprintf(last_error, ERROR_LEN,
                 "Timeout waiting for battery voltage to appear.");
        can_data.charger.msgid_109.fault_status |= MSG109_CHG_MALFUNC | MSG109_FAULT;
        chademo_transition_state(CHADEMO_STATE_ERROR);
      }
    break;

    case CHADEMO_STATE_ON:
    {
      uint32_t voltage = measured_voltage / 10;
      uint16_t soc = can_data.vehicle.msgid_102.charge_rate;

      if (!chg_perm)
      {
        snprintf(last_error, ERROR_LEN,
                 "Charge Permission Revoked");
        chademo_transition_state(CHADEMO_STATE_STOP);
      }

      if (voltage > can_data.charger.msgid_108.threshold_voltage)
      {
        snprintf(last_error, ERROR_LEN,
                 "Maximum Voltage (%ldV) Reached.", voltage);
        chademo_transition_state(CHADEMO_STATE_STOP);
      }

      if (voltage < can_data.vehicle.msgid_200.min_discharge_voltage)
      {
        snprintf(last_error, ERROR_LEN,
                 "Minimum Voltage (%ldV) Reached.", voltage);
        chademo_transition_state(CHADEMO_STATE_STOP);
      }

      if (soc < can_data.vehicle.msgid_200.min_discharge_level)
      {
        snprintf(last_error, ERROR_LEN,
                 "Minimum SoC (%d%%) Reached.", soc);
        chademo_transition_state(CHADEMO_STATE_STOP);
      }
    }
    break;

    /* Stop Requested. Waiting for current to drop below 5A */
    case CHADEMO_STATE_STOP:
      /* Wait for current to drop below 5A (Sensor is A x10) */
      if (measured_current <= STOP_CURRENT)
      {
        chademo_transition_state(CHADEMO_STATE_WELD_CHECK);
      }
      else if (HAL_GetTick() > state_time + STOP_TIMEOUT)
      {
        snprintf(last_error, ERROR_LEN,
                "Timeout waiting for current to drop to < 5A.");
        can_data.charger.msgid_109.fault_status |= MSG109_CHG_MALFUNC | MSG109_FAULT;
        chademo_transition_state(CHADEMO_STATE_ERROR);
      }
    break;

    /* Contactors Opened, waiting for voltage to drop below 10V */
    case CHADEMO_STATE_WELD_CHECK:
      /*  Check for contact welding */
      if ((measured_voltage <= CONTACTOR_OPEN_V) && 
          (can_data.vehicle.msgid_102.status & (STATUS_CONTACTOR_OPEN)))
      {
        chademo_transition_state(CHADEMO_STATE_STOP);
      }
      else if (HAL_GetTick() > state_time + BATT_CHECK_TIMEOUT)
      {
        snprintf(last_error, ERROR_LEN,
                 "Welding Fault Detected, not Unlocking!");
        can_data.charger.msgid_109.fault_status |= MSG109_FAULT;
        chademo_transition_state(CHADEMO_STATE_ERROR);
      }
    break;

    case CHADEMO_STATE_WAIT_K:
      if (!k_perm || (HAL_GetTick() > state_time + STOP_TIMEOUT))
      {
        /* Let the vehicle know that we've stopped charging */
        can_data.charger.msgid_109.fault_status &= ~MSG109_CHARGE;
        chademo_transition_state(CHADEMO_STATE_WAIT_CONTACTOR);
      }
    break;

    case CHADEMO_STATE_WAIT_CONTACTOR:
      if ((can_data.vehicle.msgid_102.status & STATUS_CONTACTOR_OPEN) == STATUS_CONTACTOR_OPEN)
      {
        /* Finally we can power off 12V! */
        chademo_transition_state(CHADEMO_STATE_OFF);
      }
      if (HAL_GetTick() > state_time + STOP_TIMEOUT)
      {
        snprintf(last_error, ERROR_LEN,
                 "Gave up waiting for Vehicle");
        chademo_transition_state(CHADEMO_STATE_OFF);
      }
    break;
    
    default:
    break;
  }

  /* We're errored. Flash the ChaDeMo LED */
  if (errored)
  {
    if (HAL_GetTick() > state_time + 500)
    {
      state_time = HAL_GetTick();
      HAL_GPIO_TogglePin(GPIOE, CHADEMO_Pin);
    }
  }

  /* Check that we are receiving regular CAN messages from ChaDeMo */
  if (chademo_state == CHADEMO_STATE_ON && 
      HAL_GetTick() > (last_update + CHADEMO_CAN_TIMEOUT))
  {
      snprintf(last_error, ERROR_LEN,
               "CAN message timeout. Aborting.");
      chademo_transition_state(CHADEMO_STATE_ERROR);
  }
}

/**
  * @brief  Start the ChaDeMo Session
  * @retval None
  */
void chademo_start(void)
{
  if (chademo_state == CHADEMO_STATE_OFF)
    chademo_transition_state(CHADEMO_STATE_START);
}

/**
  * @brief  Stop the ChaDeMo Session
  * @retval None
  */
void chademo_stop(void)
{
  if (chademo_state >= CHADEMO_STATE_PERM_OK)
    chademo_transition_state(CHADEMO_STATE_STOP);
  else
    chademo_transition_state(CHADEMO_STATE_OFF);
}

/**
  * @brief  Max power to/from EVSE
  * @retval None
  */
void chademo_set_max_power(uint32_t power)
{
  max_evse_power = power;
}

/**
  * @brief  Returns whether the DC is potentially live (contactor closed)
  * @retval bool true: Live, false: Not Live
  */
bool chademo_is_contactor_closed(void)
{
  return contactor_closed;
}

/**
  * @brief  Returns the current measured power
  * @retval int32_t power in (+'ve) or out (-'ve) of the battery.
  */
int32_t chademo_get_power(void)
{
  return measured_power;  
}

/**
  * @brief  Send JSON message with ChaDeMo Data
  * @retval None
  */
void chademo_json_update(void)
{
  printf("{\"chademo\":[{\"charger\":{[");
  printf("{\"charger\":{[");
  printf("{\"0x108\":{\"threshold\":%d, \"voltage\":%d, \"current\":%d}},",
         can_data.charger.msgid_108.threshold_voltage,
         can_data.charger.msgid_108.available_charger_voltage,
         can_data.charger.msgid_108.available_charger_current);
  printf("{\"0x109\":{\"voltage\":%d, \"current\":%d, \"power\":%ld, \"fault_status\":%d}}",
         can_data.charger.msgid_109.charger_voltage,
         can_data.charger.msgid_109.charger_current,
         measured_power,
         can_data.charger.msgid_109.fault_status);
  printf("]},");

  printf("{\"vehicle\":{[");
  printf("{\"0x100\":{\"max_voltage\":%d}},",
         can_data.vehicle.msgid_100.max_battery_voltage);
  printf("{\"0x101\":{\"rated_capacity\":%d}},",
         can_data.vehicle.msgid_101.rated_battery_capacity);
  printf("{\"0x102\":{\"target_voltage\":%d, \"charge_current\":%d, \"faults\":0x%02X, \"status\":0x%02X, \"soc\":%d}},",
         can_data.vehicle.msgid_102.target_battery_voltage,
         can_data.vehicle.msgid_102.charge_current_requested,
         can_data.vehicle.msgid_102.faults,
         can_data.vehicle.msgid_102.status,
         can_data.vehicle.msgid_102.charge_rate);
  printf("{\"0x200\":{\"min_voltage\":%d, \"max_current\":%d, \"min_soc\":%d, \"capacity\":%d}}",
         can_data.vehicle.msgid_200.min_discharge_voltage,
         can_data.vehicle.msgid_200.max_discharge_current,
         can_data.vehicle.msgid_200.min_discharge_level,
         can_data.vehicle.msgid_200.max_remaining_capacity);
  printf("{\"0x201\":{\"available_energy\":%d}}",
         can_data.vehicle.msgid_201.available_energy);
  printf("]},");

  printf("{\"state\":%d, \"last_error\":\"%s\"}", chademo_state, last_error);

  printf("]}\n");
}
