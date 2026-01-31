/** @file chademo.c
 *  @brief Functions to interact with ChaDeMo connection
 *
 *  This contains logic and CAN bus message data to communicate with a
 *  Chademo 1.0.1 V2X enabled vehicle (Tested on a 2015 Nissan Leaf).
 *  It also includes a state machine to control the Analog and Digital handshake.
 *
 *  Inspired by:
 *    https://github.com/dalathegreat/BYD-Battery-Emulator-For-Gen24
 *    https://github.com/jsphuebner/stm32-car
 *    https://openinverter.org/wiki/Tesla_Model_S/X_GEN2_Charger#Functionality_of_external_CAN_bus
 *
 *  ToDo:
 *  Analog handshake should go via connector lock detection and inverter shutoff in HW
 *  (i.e. separate from CPU)
 *
 *  Copyright (c) 2023 ARTaylor.co.uk.
 *  All rights reserved.
 *
 *  @author Richard Taylor <richard@artaylor.co.uk>
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "usbd_cdc_if.h"  /* for MIN */
#include "can.h"
#include "ioexp.h"
#include "chademo.h"
#include "solax.h"
#include "sensor.h"
#include "tim.h"

/* #define DEBUG_CHADEMO */

/*
 * Leakage is monitored by the Inverter and we will cause problems if done in
 * multiple places
 */
/* #define LEAK_TEST */

#define MESSAGE_INTERVAL      (100)

#define DEBOUNCE_TIME         (50)
#define CHADEMO_CAN_TIMEOUT   (10000)
#define BATT_CHECK_TIMEOUT    (5000 * 2)
#define STOP_TIMEOUT          (10000) /* How long to wait for current to drop */

#define STOP_CURRENT          (5)    /* Spec is 5A, but using 0.5A (x10) */
#define ISOLATION_MIN_RES     (500)  /* 500kOhm */
#define LEAK_TEST_TIME        (1000)  /* Between 200ms and 1000ms */
#define ISOLATION_MIN_VOLTAGE (3500)
#define DEFAULT_MIN_SOC       (SOLAX_MINIMUM_SOC)
#define ACC_CURRENT_MAX       (800 * 1000)  /* 800mA */

#define CONTACTOR_CLOSED_V    (1000) // 50v (x10)
#define CONTACTOR_OPEN_V      (900)  // 10v (x10)

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

#define STATUS_CHG_V2X_1      (1 << 5)
#define STATUS_CHG_V2X_2      (1 << 6)
#define STATUS_CHG_V2X_COMPAT (1 << 7)


/* MSG ID 0x109 Bits */
#define MSG109_CHARGE         (1 << 0)
#define MSG109_FAULT          (1 << 1)
#define MSG109_CONN_LOCK      (1 << 2)
#define MSG109_BATT_INCOMPAT  (1 << 3)
#define MSG109_CHG_MALFUNC    (1 << 4)
#define MSG109_CHG_STOPPED    (1 << 5)

struct can_data
{
  struct _vehicle
  {
    struct __attribute__((packed))
    {
        uint8_t min_charge_current;     /* Minimum charge current (A x1) 0: unused */
        uint8_t reserved[1];
        uint16_t min_battery_voltage;   /* Minimum voltage for V2X (V x1?) 0: unused */
        uint16_t max_battery_voltage;   /* Maximum voltage (V x1) */
        uint8_t charge_rate_indication; /* Const 100 (100%) */
        uint8_t reserved2;
    } msgid_100;

    struct __attribute__((packed))
    {
        uint8_t reserved;
        uint8_t max_charging_time_10s;
        uint8_t max_charging_time_1min;
        uint8_t estimated_charge_time_1min;
        uint8_t reserved2;
        uint16_t rated_battery_capacity;  /* Capacity when full (0.1kWh) includes SoH factor? */
        uint8_t reserved3;
    } msgid_101;

    struct __attribute__((packed))
    {
        uint8_t chademo_version;
        uint16_t target_battery_voltage;  /* Target Voltage (V x1) */
        uint8_t charge_current_requested; /* Requested Current (A x1) */
        uint8_t faults;
        uint8_t status;
        uint8_t charge_rate;              /* Battery SoC (% x1) */
        uint8_t pad[1];
    } msgid_102;

    /* Chademo v2.0 only */
    struct __attribute__((packed))
    {
        uint8_t status;
        uint8_t pad[7];
    } msgid_110;

    struct __attribute__((packed))
    {
        uint8_t max_discharge_current;    /* Max Discharge (A x1) */
        uint16_t min_discharge_voltage;   /* Min Voltage (V x1?) 0: unused */
        uint8_t min_discharge_level;      /* Min SoC or kWh (% x1) 0: unused */
        uint8_t max_remaining_capacity;   /* Max Available capacity with which the vehicle permits (dis?)charging (0.1%?)*/
        uint8_t pad[3];                   /* Data here, need more info (0x00, 0x45, 0xb9) */
    } msgid_200;

    struct __attribute__((packed))
    {
        uint8_t v2x_sequence_num;
        uint16_t estimated_discharge_time;
        uint16_t available_energy;
        uint8_t pad[3];
    } msgid_201;

    struct __attribute__((packed))
    {
        uint8_t manufacturer_code;
        uint8_t pad[7];
    } msgid_700;
  } vehicle;

  struct _charger
  {
    struct __attribute__((packed))
    {
        uint8_t welding_detection;            /* 0x00: Not Supported, 0x01: Supported */
        uint16_t available_charger_voltage;   /* V x1 */
        uint8_t available_charger_current;    /* A x1 */
        uint16_t threshold_voltage;           /* Fault Threshold Voltage Max (V x1) */
        uint8_t pad[2];
    } msgid_108;

    struct __attribute__((packed))
    {
        uint8_t chademo_version;              /* ChaDeMo Version supported (0x02) */
        uint16_t charger_voltage;             /* Battery voltage (V x1) */
        uint8_t charger_current;              /* Battery current (A x1) */
        uint8_t reserved;                     /* 0x01 when V2X? */
        uint8_t fault_status;
        uint8_t time_remaining_10s;           /* 0xff if using 1min field */
        uint8_t time_remaining_1min;          /* Remaining time in mins */
    } msgid_109;

    /* Chademo v2.0 only */
    struct __attribute__((packed))
    {
        uint8_t status;
        uint8_t pad[7];
    } msgid_118;

    struct __attribute__((packed))
    {
        uint8_t discharge_current;          /* Battery current: 0xff - current (A x1) */
        uint16_t min_voltage;               /* Minimum voltage for inverter (V x1?) */
        uint8_t discharge_current_max;      /* Max current for protection (A x1) */
        uint8_t reserved;
        uint16_t low_threshold_voltage;     /* Inverter batt protection cutoff voltage (V x1?) */
        uint8_t pad[1];
    } msgid_208;

    struct __attribute__((packed))
    {
        uint8_t v2x_sequence_num;           /* Always 0x02? */
        uint16_t remaining_discharge_time;  /* Hours? */
        uint8_t pad[5];
    } msgid_209;
  } charger;
};

static uint32_t last_update = 0;            /* Last time we saw a CAN message */
static CHADEMO_STATE chademo_state = 0;     /* State Machine State */
static struct can_data can_data;            /* Structure holding all CAN message data */

static bool chg_perm = false;               /* Vehicle Charge permission state */
static bool k_perm = false;                 /* Vehicle Charge permission state (K line only) */
static bool cp_ready = false;               /* Is the car plugged in? */

static uint32_t state_time = 0;             /* Time that the last state transition happened */
static uint32_t error_time = 0;             /* Timer to flash LED on error */

static bool errored = false;                /* If we hit any errors, prevent starting again */
static char last_error[ERROR_LEN+1] = {0};  /* Last error string */

static uint16_t max_evse_power = 0;         /* Maximum Power (W x1) from the EVSE */
static int32_t measured_voltage = 0;        /* Voltage (V x10) */
static int32_t measured_current = 0;        /* Current (A x10) */
static int32_t measured_power = 0;          /* Power (W x1) in (+'ve) or out (-'ve) of the battery */

static uint8_t min_discharge_level = DEFAULT_MIN_SOC; /* Minimum SoC during discharge */
static uint32_t rated_capacity = 180000;     /* Rated capacity (0.1 kWh) */
static uint32_t available_energy = 0;       /* Available energy (0.1 kWh) */

static bool start_pending = false;          /* A start request is pending */
static bool stop_pending = false;           /* A stop request is pending */

/* For chademo v2.0 only */
static uint8_t chademo_118[8] = {0x10, 0x64, 0x00, 0xB0, 0x00, 0x1E, 0x00, 0x8F};
/* For V2X */
static uint8_t chademo_208[8] = {0xFF, 0xF4, 0x01, 0xF0, 0x00, 0x00, 0xFA, 0x00};
static uint8_t chademo_209[8] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static osThreadId_t taskHandle;
static const osThreadAttr_t taskAttributes = {
  .name = "chademoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

void chademoTask(void *argument);

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
  * @brief  Perform immediate actions associated with a state transition.
  *         Any delayed transitions must be handled in chademo_process() instead.
  *         Must not call itself!
  * @param  state The new state
  * @retval None
  */
static void chademo_transition_state(CHADEMO_STATE new_state)
{
  /* Used for Sensor reads */
  HAL_StatusTypeDef ret = HAL_ERROR;

  state_time = HAL_GetTick();

  /* Make sure we notify watchers ASAP */
  trigger_json_update();

  switch (new_state)
  {
    case CHADEMO_STATE_OFF:
      /* These should already be off, but can be used as an emergency stop */
      hv_iso_test_enable(false, 0);
      HAL_GPIO_WritePin(LEAK_TEST_EN_GPIO_Port, LEAK_TEST_EN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(CHADEMO_SEQ2_GPIO_Port, CHADEMO_SEQ2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(CHADEMO_SEQ1_GPIO_Port, CHADEMO_SEQ1_Pin, GPIO_PIN_RESET);

      /* Unlock connector */
      HAL_GPIO_WritePin(CHADEMO_LOCK_GPIO_Port, CHADEMO_LOCK_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED3_Pin, GPIO_PIN_SET);

      /* Let the vehicle know we're unlocked */
      can_data.charger.msgid_109.fault_status &= ~MSG109_CONN_LOCK;
    break;

    case CHADEMO_STATE_START:
      /* Ensure that our current measurement is zeroed */
      ret = sensor_zero_ibatt();
      if (ret != HAL_OK)
      {
        snprintf(last_error, ERROR_LEN, "Failed to zero battery current.");
        new_state = CHADEMO_STATE_ERROR;
      }
    break;

    case CHADEMO_STATE_PARAM_CHK:
      if (k_perm)
      {
        /* Vehicle permission line stuck */
        snprintf(last_error, ERROR_LEN,
                 "Vehicle permission (k) line state invalid.");
        new_state = CHADEMO_STATE_ERROR;
      }

      if ((can_data.vehicle.msgid_102.status & STATUS_CHARGE) == STATUS_CHARGE)
      {
        /* Vehicle permission message invalid */
        snprintf(last_error, ERROR_LEN,
                 "Vehicle permission CHARGE bit state invalid.");
        new_state = CHADEMO_STATE_ERROR;
      }
    break;

    case CHADEMO_STATE_PERM_OK:
    {
      /*  Check for contact welding */
      if (measured_voltage > CONTACTOR_OPEN_V ||
          !(can_data.vehicle.msgid_102.status & (STATUS_CONTACTOR_OPEN)))
      {
        snprintf(last_error, ERROR_LEN,
                 "Pre-contactor close battery check failed. Voltage: %ldv, Status: 0x%02X.",
                 measured_voltage / 10, can_data.vehicle.msgid_102.status);
        new_state = CHADEMO_STATE_ERROR;
      }
      else
      {
        /* Lock the connector */
        HAL_GPIO_WritePin(CHADEMO_LOCK_GPIO_Port, CHADEMO_LOCK_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

        can_data.charger.msgid_109.fault_status |= MSG109_CONN_LOCK;

        /* Enable HV DCDC Test source(s) */
        hv_iso_test_enable(true, HV_GEN_MAX_VOLTAGE);
      }
    }
    break;

    case CHADEMO_STATE_INS_TEST_BASE:
    {
#ifdef LEAK_TEST
        /* Start Earth Leakage Test */
        HAL_GPIO_WritePin(LEAK_TEST_EN_GPIO_Port, LEAK_TEST_EN_Pin, GPIO_PIN_SET);

        // ToDo: Work out how to do this
#endif
        snprintf(last_error, ERROR_LEN,
                "Failed to get baseline HV current.");
        new_state = CHADEMO_STATE_ERROR;
    }
    break;

    case CHADEMO_STATE_INS_TEST:
    {
      int32_t acc_current;  /* 12V ACC Current in uA */

      ret = sensor_get_value(SENSOR_ACC_CURRENT, &acc_current);

      /* Check that HV Test current is below threshold */
      if (ret != HAL_OK || hv_iso_resistance < ISOLATION_MIN_RES)
      {
#ifdef LEAK_TEST
        snprintf(last_error, ERROR_LEN,
                 "Earth leakage test failed (%ld uA).", hv_current - leak_base);
#else
        snprintf(last_error, ERROR_LEN,
                 "Isolation test failed (%ld kOhm at %ld V).", hv_iso_resistance, measured_voltage / 10);
#endif
        new_state = CHADEMO_STATE_ERROR;
      }
      /* Check that we are able to bring up the HV Test voltage */
      else if (measured_voltage < ISOLATION_MIN_VOLTAGE)
      {
        snprintf(last_error, ERROR_LEN,
                 "Isolation test failed (%ld V).", measured_voltage / 10);
        new_state = CHADEMO_STATE_ERROR;
      }
      /* Check that the connector is properly locked */
      else if (acc_current > ACC_CURRENT_MAX)
      {
        can_data.charger.msgid_109.fault_status &= ~MSG109_CONN_LOCK;

        snprintf(last_error, ERROR_LEN,
                 "Connector Lock failed. High ACC current (%ld mA).", acc_current / 1000);
        new_state = CHADEMO_STATE_ERROR;
      }
      else
      {
        /* Enable the Battery Contactors to close */
        HAL_GPIO_WritePin(CHADEMO_SEQ2_GPIO_Port, CHADEMO_SEQ2_Pin, GPIO_PIN_SET);
        new_state = CHADEMO_STATE_BATT_CHECK;
      }

      /* Disable HV Test */
      HAL_GPIO_WritePin(LEAK_TEST_EN_GPIO_Port, LEAK_TEST_EN_Pin, GPIO_PIN_RESET);
      hv_iso_test_enable(false, 0);
    }
    break;

    case CHADEMO_STATE_ON:
      // ToDo: Spec says this should be set when charging (> 5A)
      can_data.charger.msgid_109.fault_status |= MSG109_CHARGE;
      can_data.charger.msgid_109.fault_status &= ~MSG109_CHG_STOPPED;
    break;

    case CHADEMO_STATE_STOP:
      /* Tell the inverter to stop */
      solax_set_max_dc_chg_current(0);
      solax_set_max_dc_dis_current(0);

      /* Let the vehicle know we're stopping (current ramp down) */
      can_data.charger.msgid_109.fault_status |= MSG109_CHG_STOPPED;
    break;

    case CHADEMO_STATE_BATT_CHECK:
    case CHADEMO_STATE_WELD_CHECK:
    case CHADEMO_STATE_WAIT_K_OFF:
    case CHADEMO_STATE_WAIT_VEHICLE_OFF:
    case CHADEMO_STATE_ERROR:
      /* Nothing to do here, handled in chademo_process() */
    break;
  }

  chademo_state = new_state;

  /* Let Solax know we've updated state */
  solax_kick();

  {
    uint16_t leds = 0;

    if (errored)
      leds |= (1 << CHADEMO_STATE_ERROR);

    if (chademo_state > 0)
      leds |= (1 << chademo_state);

    ioexp_set_direction(IOEXP_BOT_LEDS, ~leds);
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

  k_perm = phy_ok;
  chg_perm = (phy_ok && can_ok);

  /* Check for Vehicle Pilot signal */
  if (HAL_GPIO_ReadPin(CHADEMO_CP_GPIO_Port, CHADEMO_CP_Pin) == GPIO_PIN_SET)
    cp_ready = true;
  else
    cp_ready = false;
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
    /* Read the message */
    if (HAL_OK == HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, data))
    {
      if (RxHeader.IDE != CAN_ID_STD)
      {
        snprintf(last_error, ERROR_LEN, "Unexpected Ext CAN message");
        continue;
      }

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

          /* Use this as our CAN active indicator */
          last_update = HAL_GetTick();
          break;

        case 0x200:  /* For V2X */
          assert_param(sizeof(can_data.vehicle.msgid_200) >= RxHeader.DLC);
          memcpy(&can_data.vehicle.msgid_200, data, RxHeader.DLC);
          break;

        case 0x201:  /* For V2X */
          assert_param(sizeof(can_data.vehicle.msgid_201) >= RxHeader.DLC);
          memcpy(&can_data.vehicle.msgid_201, data, RxHeader.DLC);
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
  * @brief  Send Charger messages
  * @retval None
  */
HAL_StatusTypeDef chademo_send_messages(void)
{
  static uint32_t last_send = 0;
  HAL_StatusTypeDef ret = HAL_ERROR;

  /* Send Charger messages every 100ms */
  if (HAL_GetTick() < last_send + MESSAGE_INTERVAL)
    return HAL_BUSY;

  last_send = HAL_GetTick();

  /* Update our Voltage, Current and Power measurements */
  ret = sensor_get_value(SENSOR_BATT_VOLTAGE, &measured_voltage);
  if (ret != HAL_OK)
    return ret;

  ret = sensor_get_value(SENSOR_INV_CURRENT, &measured_current);
  if (ret != HAL_OK)
    return ret;

  measured_power = measured_voltage * measured_current / 100;

  /* Check for Faults */
  if (ret != HAL_OK || can_data.vehicle.msgid_102.faults ||
      (can_data.vehicle.msgid_102.status & (STATUS_NOT_PARKED | STATUS_MALFUNCTION)) )
  {
    if (chademo_state == CHADEMO_STATE_ON)
    {
      snprintf(last_error, ERROR_LEN,
            "Aborting Charge. Vehicle Faults: 0x%02X, Status: 0x%02X",
            can_data.vehicle.msgid_102.faults,
            can_data.vehicle.msgid_102.status);

      chademo_transition_state(CHADEMO_STATE_ERROR);
    }
  }

  /* Send standard messages */
  if (ret == HAL_OK)
  {
    can_data.charger.msgid_109.charger_voltage = measured_voltage / 10;
    if (measured_current >= 0)
    {
      can_data.charger.msgid_109.charger_current = measured_current / 10;
      can_data.charger.msgid_208.discharge_current = 255;
    }
    else
    {
      can_data.charger.msgid_109.charger_current = 0;
      can_data.charger.msgid_208.discharge_current = 255 + (measured_current / 10);
    }

    ret = chademo_send_message(0x108, (uint8_t*)&can_data.charger.msgid_108);
    if (ret == HAL_OK)
      ret = chademo_send_message(0x109, (uint8_t*)&can_data.charger.msgid_109);

    if (ret == HAL_OK && can_data.vehicle.msgid_102.chademo_version >= 0x03)
    {
      /* Only send the following on Chademo 2.0 vehicles? */
      ret = chademo_send_message(0x118, (uint8_t*)&can_data.charger.msgid_118);
    }
  }

  /* Send V2X messages */
  if (ret == HAL_OK)
  {
    ret = chademo_send_message(0x208, (uint8_t*)&can_data.charger.msgid_208);
    if (ret == HAL_OK)
      ret = chademo_send_message(0x209, (uint8_t*)&can_data.charger.msgid_209);
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

  can_data.charger.msgid_208.low_threshold_voltage = SOLAX_MINIMUM_SUPPORTED_VOLTAGE;
  can_data.charger.msgid_208.discharge_current_max = SOLAX_MAXIMUM_SUPPORTED_CURRENT;

  can_data.vehicle.msgid_102.status = STATUS_CONTACTOR_OPEN;

  taskHandle = osThreadNew(chademoTask, NULL, &taskAttributes);

  return (taskHandle != NULL);
}

/**
  * @brief  Process ChaDeMo CAN data and drive time based
  *         state machine transitions.
  * @retval None
  */
void chademo_process(void)
{
  /* Process any CAN messages */
  chademo_process_can();

  /* Check the Vehicle Permission GPIO */
  chademo_check_vehicle_permission();

  /* Send Charger messages every 100ms */
  chademo_send_messages();

  /* Time based state machine transitions */
  switch (chademo_state)
  {
    case CHADEMO_STATE_OFF:
      last_update = 0;
    break;

    case CHADEMO_STATE_START:
      if (!errored)
      {
        /* Signal to the vehicle that we're ready to start */
        HAL_GPIO_WritePin(CHADEMO_SEQ1_GPIO_Port, CHADEMO_SEQ1_Pin, GPIO_PIN_SET);
      }

      if (HAL_GetTick() > state_time + CHADEMO_CAN_TIMEOUT)
      {
        if (last_update == 0)
        {
          snprintf(last_error, ERROR_LEN,
                  "Timed out waiting for CAN messages from Vehicle");
        }

        /* No need for full shut down sequence, no HV involved yet */
        chademo_transition_state(CHADEMO_STATE_OFF);
      }

      /* Check that we're allowed to enable and are receiving CAN messages */
      if (last_update > 0)
      {
        chademo_transition_state(CHADEMO_STATE_PARAM_CHK);
      }
    break;

    /* Wait for vehicle to grant permission */
    case CHADEMO_STATE_PARAM_CHK:
    {
      /* These values are only allowed to change during Parameter checking */

      /* Set the Threshold Voltage */
      can_data.charger.msgid_108.threshold_voltage =
        MIN(SOLAX_MAXIMUM_SUPPORTED_VOLTAGE, can_data.vehicle.msgid_100.max_battery_voltage);

      /* Set Maximum possible current (will be limited by EVSE and Inverter setting)*/
      can_data.charger.msgid_108.available_charger_current = SOLAX_MAXIMUM_SUPPORTED_CURRENT;

      /* Set the Charge time remaining */
      can_data.charger.msgid_109.time_remaining_10s = can_data.vehicle.msgid_101.max_charging_time_10s;
      can_data.charger.msgid_109.time_remaining_1min = can_data.vehicle.msgid_101.max_charging_time_1min;

      /* Set rated capacity if available otherwise use default */
      if (can_data.vehicle.msgid_101.rated_battery_capacity > 0)
      {
        rated_capacity = can_data.vehicle.msgid_101.rated_battery_capacity;
      }

      /* Set up V2X Parameters */

      /* Set maximum discharge current based on vehicle and inverter */
      can_data.charger.msgid_208.discharge_current_max = SOLAX_MAXIMUM_SUPPORTED_CURRENT;
      if (can_data.vehicle.msgid_200.max_discharge_current != 0)
      {
        can_data.charger.msgid_208.discharge_current_max =
          MIN(can_data.vehicle.msgid_200.max_discharge_current, SOLAX_MAXIMUM_SUPPORTED_CURRENT);
      }

      /* Set minimum and threshold voltage based on vehicle and inverter */
      can_data.charger.msgid_208.low_threshold_voltage = SOLAX_MINIMUM_SUPPORTED_VOLTAGE;
      can_data.charger.msgid_208.min_voltage = SOLAX_MINIMUM_SUPPORTED_VOLTAGE;
      if (can_data.vehicle.msgid_200.min_discharge_voltage != 0)
      {
        can_data.charger.msgid_208.low_threshold_voltage =
          MAX(SOLAX_MINIMUM_SUPPORTED_VOLTAGE, can_data.vehicle.msgid_200.min_discharge_voltage);
      }

      /* Set the minimum SoC */
      min_discharge_level = DEFAULT_MIN_SOC;
      if (can_data.vehicle.msgid_200.min_discharge_level != 0)
      {
        min_discharge_level = can_data.vehicle.msgid_200.min_discharge_level;
      }

      /* Update Solax Data (Standard) */
      solax_set_battery_voltage_max(can_data.vehicle.msgid_100.max_battery_voltage * 10);
      solax_set_battery_capacity_max(rated_capacity * 100);
      solax_set_max_dc_chg_current(can_data.vehicle.msgid_102.charge_current_requested * 10);
      solax_set_battery_voltage_tgt(can_data.vehicle.msgid_102.target_battery_voltage * 10);

      /* Update Solax Data (V2X) */
      solax_set_battery_voltage_min(can_data.charger.msgid_208.low_threshold_voltage * 10);
      solax_set_max_dc_dis_current(can_data.charger.msgid_208.discharge_current_max * 10);

      if (chg_perm)
      {
        /* Check we're within limits */
        if (can_data.vehicle.msgid_100.max_battery_voltage < SOLAX_MAXIMUM_SUPPORTED_VOLTAGE &&
            can_data.vehicle.msgid_102.target_battery_voltage < SOLAX_MAXIMUM_SUPPORTED_VOLTAGE &&
            (can_data.vehicle.msgid_102.status & STATUS_CHG_V2X_COMPAT) == STATUS_CHG_V2X_COMPAT)
        {
          chademo_transition_state(CHADEMO_STATE_PERM_OK);
        }
        else
        {
          snprintf(last_error, ERROR_LEN, "Battery / Vehicle incompatible (%d)",
                   can_data.vehicle.msgid_102.status);
          can_data.charger.msgid_109.fault_status |= MSG109_BATT_INCOMPAT;
          chademo_transition_state(CHADEMO_STATE_OFF);
        }
      }

      if (HAL_GetTick() > state_time + CHADEMO_CAN_TIMEOUT)
      {
        snprintf(last_error, ERROR_LEN,
                 "Timed out waiting for Vehicle Permission");
        /* No need for full shut down sequence, no HV involved yet */
        chademo_transition_state(CHADEMO_STATE_OFF);
      }
    }
    break;

    /* Vehicle Permission granted, HV turned on, waiting to settle */
    case CHADEMO_STATE_PERM_OK:
      if (HAL_GetTick() > state_time + LEAK_TEST_TIME)
        chademo_transition_state(CHADEMO_STATE_INS_TEST_BASE);
    break;

    /* Leak Test Started, waiting for result */
    case CHADEMO_STATE_INS_TEST_BASE:
      if (HAL_GetTick() > state_time + LEAK_TEST_TIME)
        chademo_transition_state(CHADEMO_STATE_INS_TEST);
    break;

    case CHADEMO_STATE_INS_TEST:
    break;

    case CHADEMO_STATE_BATT_CHECK:
      if ((measured_voltage > CONTACTOR_CLOSED_V) &&
          (can_data.vehicle.msgid_102.status & (STATUS_CONTACTOR_OPEN)) != STATUS_CONTACTOR_OPEN)
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
      int32_t voltage = measured_voltage / 10;
      int32_t current = measured_current / 10;
      uint8_t max_dc_chg_current = can_data.vehicle.msgid_102.charge_current_requested * 10;
      uint8_t max_dc_dis_current = can_data.charger.msgid_208.discharge_current_max * 10;
      uint16_t soc = can_data.vehicle.msgid_102.charge_rate;

      available_energy = soc * rated_capacity / 100;

      if (!chg_perm)// || !cp_ready)
      {
        snprintf(last_error, ERROR_LEN,
                 "Charge Permission Revoked (%d, %d, %d)", chg_perm, k_perm, cp_ready);
        chademo_transition_state(CHADEMO_STATE_STOP);
      }

      /* Charge Limits */
      if (voltage >= can_data.charger.msgid_108.threshold_voltage)
      {
        snprintf(last_error, ERROR_LEN,
                 "Maximum Voltage (%ldV) Reached.", voltage);
        max_dc_chg_current = 0;
      }

      /* V2X Limits */
      if (voltage <= can_data.charger.msgid_208.low_threshold_voltage)
      {
        snprintf(last_error, ERROR_LEN,
                 "Minimum Voltage (%ldV) Reached.", voltage);
        max_dc_dis_current = 0;
      }

      if (current > can_data.charger.msgid_208.discharge_current_max)
      {
        snprintf(last_error, ERROR_LEN,
                 "Maximum Current %ldA) Exceeded.", current);
        chademo_transition_state(CHADEMO_STATE_STOP);
      }

      if (soc <= min_discharge_level)
      {
        snprintf(last_error, ERROR_LEN,
                "Minimum SoC (%d%%) Reached.", soc);
        max_dc_dis_current = 0;
      }

      // ToDo: Consider decrementing the minute counter!
      //can_data.charger.msgid_109.time_remaining_10s = 0xff;
      //can_data.charger.msgid_109.time_remaining_1min = 0xff;

      if (can_data.charger.msgid_109.time_remaining_10s == 0 ||
          can_data.charger.msgid_109.time_remaining_1min == 0)
      {
        snprintf(last_error, ERROR_LEN,
                 "Max Charge Time Reached");
        //chademo_transition_state(CHADEMO_STATE_STOP);
      }

      /* Update Solax Data */
      solax_set_battery_capacity(available_energy * 100);
      solax_set_battery_soc(can_data.vehicle.msgid_102.charge_rate);

      /* Update allowed currents */
      solax_set_max_dc_chg_current(max_dc_chg_current);
      solax_set_max_dc_dis_current(max_dc_dis_current);
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
                "Timeout waiting for current to drop below threshold.");
        can_data.charger.msgid_109.fault_status |= MSG109_CHG_MALFUNC | MSG109_FAULT;
        chademo_transition_state(CHADEMO_STATE_WELD_CHECK);
      }
    break;

    /* Contactors Opened, waiting for voltage to drop below 10V */
    case CHADEMO_STATE_WELD_CHECK:
      /* Let the vehicle know that we've stopped charging (< 5A) */
      can_data.charger.msgid_109.fault_status &= ~MSG109_CHARGE;

      /* Charge time remaining is now zero */
      can_data.charger.msgid_109.time_remaining_10s = 0x00;
      can_data.charger.msgid_109.time_remaining_1min = 0x00;

      /*  Check for contact welding */
      if ((measured_voltage <= CONTACTOR_OPEN_V) &&
          (can_data.vehicle.msgid_102.status & (STATUS_CONTACTOR_OPEN)))
      {
        chademo_transition_state(CHADEMO_STATE_WAIT_K_OFF);
      }
      else if (HAL_GetTick() > state_time + BATT_CHECK_TIMEOUT)
      {
        snprintf(last_error, ERROR_LEN,
                 "Welding Fault Detected, not Unlocking!");
        can_data.charger.msgid_109.fault_status |= MSG109_FAULT;

        /* Try forcing it open */
        HAL_GPIO_WritePin(CHADEMO_SEQ2_GPIO_Port, CHADEMO_SEQ2_Pin, GPIO_PIN_RESET);

        chademo_transition_state(CHADEMO_STATE_ERROR);
      }
    break;

    case CHADEMO_STATE_WAIT_K_OFF:
      if (!k_perm || (HAL_GetTick() > state_time + STOP_TIMEOUT))
      {
        chademo_transition_state(CHADEMO_STATE_WAIT_VEHICLE_OFF);
      }
    break;

    case CHADEMO_STATE_WAIT_VEHICLE_OFF:
      if ((can_data.vehicle.msgid_102.status & STATUS_CONTACTOR_OPEN) == STATUS_CONTACTOR_OPEN)
      {
        /* Finally we can power off 12V! */
        HAL_GPIO_WritePin(CHADEMO_SEQ2_GPIO_Port, CHADEMO_SEQ2_Pin, GPIO_PIN_RESET);

        chademo_transition_state(CHADEMO_STATE_OFF);
      }
      if (HAL_GetTick() > state_time + STOP_TIMEOUT)
      {
        snprintf(last_error, ERROR_LEN,
                 "Gave up waiting for Vehicle");
        chademo_transition_state(CHADEMO_STATE_OFF);
      }
    break;

    case CHADEMO_STATE_ERROR:
      errored = true;
      chademo_transition_state(CHADEMO_STATE_STOP);
    break;
  }

  /* Check for Start Request */
  if (start_pending && !stop_pending)
  {
    start_pending = false;
    if (!errored && cp_ready)
    {
      if (chademo_state == CHADEMO_STATE_OFF)
      {
        chademo_transition_state(CHADEMO_STATE_START);
      }
    }
  }

  /* Check for Stop Request */
  if (stop_pending)
  {
    stop_pending = false;

    if (!errored)
    {
      if (chademo_state >= CHADEMO_STATE_STOP)
      {
        /* Already stopping, ignore */
      }
      else if (chademo_state >= CHADEMO_STATE_PERM_OK)
      {
        /* Need to go through full stop sequence */
        chademo_transition_state(CHADEMO_STATE_STOP);
      }
      else
      {
        chademo_transition_state(CHADEMO_STATE_OFF);
      }
    }
  }

  /* We're errored. Flash the ChaDeMo LED */
  if (errored)
  {
    if (HAL_GetTick() > error_time + 500)
    {
      error_time = HAL_GetTick();
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED3_Pin);
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
  start_pending = true;
}

/**
  * @brief  Stop the ChaDeMo Session
  * @retval None
  */
void chademo_stop(void)
{
  stop_pending = true;
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
  * @brief  Returns the current state
  * @retval CHADEMO_STATE
  */
CHADEMO_STATE chademo_get_state(void)
{
  return chademo_state;
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
  * @brief  Process command line input for the chademo module
  * @param  args Argument list
  * @param  argc Number of arguments
  * @retval Status (0 = OK, -1 = Error / Unknown Command)
  */
int chademo_process_cmd(char **args, int argc)
{
  if (argc >= 1)
  {
    if (0 == strcmp(args[0], "start"))
    {
      chademo_start();
    }
    else if (0 == strcmp(args[0], "stop"))
    {
      chademo_stop();
    }
  }
  return 0;
}

/**
  * @brief  Send JSON message with ChaDeMo Data
  * @retval None
  */
void chademo_json_update(void)
{
  printf("\"chademo\":{");
    printf("\"state\":%d", chademo_state);
    printf(",\"cp_ready\":%d", cp_ready);
    printf(",\"voltage\":%ld,\"current\":%ld,\"power\":%ld",
          measured_voltage/10,
          measured_current/10,
          measured_power);
    if (strnlen(last_error, ERROR_LEN))
      printf(",\"last_error\":\"%s\"", last_error);
#ifdef DEBUG_CHADEMO
    printf(",\"last_update\":%ld", HAL_GetTick() - last_update);
    printf(",\"k_perm\":%d", k_perm);
    printf(",\"charger\":{");
      printf("\"0x108\":{\"threshold_voltage\":%d, \"available_voltage\":%d, \"available_current\":%d},",
            can_data.charger.msgid_108.threshold_voltage,
            can_data.charger.msgid_108.available_charger_voltage,
            can_data.charger.msgid_108.available_charger_current);
      printf("\"0x109\":{\"voltage\":%d, \"current\":%d, \"power\":%ld, \"fault_status\":%d},",
            can_data.charger.msgid_109.charger_voltage,
            can_data.charger.msgid_109.charger_current,
            measured_power,
            can_data.charger.msgid_109.fault_status);
      printf("\"0x208\":{\"discharge_current\":%d, \"min_voltage\":%d, \"discharge_current_max\":%d, \"low_threshold_voltage\":%d}",
            can_data.charger.msgid_208.discharge_current,
            can_data.charger.msgid_208.min_voltage,
            can_data.charger.msgid_208.discharge_current_max,
            can_data.charger.msgid_208.low_threshold_voltage);
    printf("}");

    printf(",\"vehicle\":{");
      printf("\"0x100\":{\"max_voltage\":%d, \"min_voltage\":%d},",
            can_data.vehicle.msgid_100.max_battery_voltage,
            can_data.vehicle.msgid_100.min_battery_voltage);
      printf("\"0x101\":{\"rated_capacity\":%d, \"max_charge_time\":%d},",
            can_data.vehicle.msgid_101.rated_battery_capacity,
            (can_data.vehicle.msgid_101.max_charging_time_10s == 0xff)?
            can_data.vehicle.msgid_101.max_charging_time_1min * 60:
            can_data.vehicle.msgid_101.max_charging_time_10s);
      printf("\"0x102\":{\"target_voltage\":%d, \"charge_current\":%d, \"faults\":%d, \"status\":%d, \"soc\":%d},",
            can_data.vehicle.msgid_102.target_battery_voltage,
            can_data.vehicle.msgid_102.charge_current_requested,
            can_data.vehicle.msgid_102.faults,
            can_data.vehicle.msgid_102.status,
            can_data.vehicle.msgid_102.charge_rate);
      printf("\"0x200\":{\"min_voltage\":%d, \"max_current\":%d, \"min_soc\":%d, \"capacity\":%d},",
            can_data.vehicle.msgid_200.min_discharge_voltage,
            can_data.vehicle.msgid_200.max_discharge_current,
            can_data.vehicle.msgid_200.min_discharge_level,
            can_data.vehicle.msgid_200.max_remaining_capacity);
      printf("\"0x201\":{\"available_energy\":%d}",
            can_data.vehicle.msgid_201.available_energy);
    printf("}");
#endif
  printf("}");
}

/**
  * @brief  Function implementing the chademo thread.
  * @param  argument: Not used
  * @retval None
  */
void chademoTask(void *argument)
{
  for (;;)
  {
    chademo_process();
    osDelay(10);
  }
}
