#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "can.h"
#include "solax.h"

#define BATTERY_WH_MAX \
  24000  //Battery size in Wh (Maximum value for most inverters is 60000 [60kWh], you can use larger batteries but do not set value over 60000!
#define ABSOLUTE_MAX_VOLTAGE (96 * 4.135)
#define ABSOLUTE_MIN_VOLTAGE (96 * 3.135)

typedef enum _inverter_state
{
    INVERTER_STANDBY,
    INVERTER_INACTIVE,
    INVERTER_DARKSTART,
    INVERTER_ACTIVE,
    INVERTER_FAULT,
    INVERTER_UPDATING
} INVERTER_STATE;

// Common inverter parameters
uint16_t capacity_Wh_startup = BATTERY_WH_MAX;
uint16_t max_power = 40960;                   // 41kW
uint16_t max_voltage = ABSOLUTE_MAX_VOLTAGE;  // If higher charging is not possible (goes into forced discharge)
uint16_t min_voltage = ABSOLUTE_MIN_VOLTAGE;  // If lower Gen24 disables battery
uint16_t battery_voltage = 3700;
uint16_t battery_current = 0;
uint16_t SOC = 5000;                              // SOC 0-100.00% // Updates later on from CAN
uint16_t StateOfHealth = 9900;                    // SOH 0-100.00% // Updates later on from CAN
uint16_t capacity_Wh = BATTERY_WH_MAX;            // Updates later on from CAN
uint16_t remaining_capacity_Wh = BATTERY_WH_MAX;  // Updates later on from CAN
uint16_t max_target_discharge_power = 0;          // 0W (0W > restricts to no discharge) // Updates later on from CAN
uint16_t max_target_charge_power = 4312;  // 4.3kW (during charge), both 307&308 can be set (>0) at the same time 
                                          // Updates later on from CAN. Max value is 30000W
uint16_t temperature_max = 50;     // Reads from battery later
uint16_t temperature_min = 60;     // Reads from battery later
uint16_t bms_char_dis_status;      // 0 idle, 1 discharging, 2, charging
uint16_t bms_status = INVERTER_ACTIVE;      // ACTIVE - [0..5]<>[STANDBY,INACTIVE,DARKSTART,ACTIVE,FAULT,UPDATING]
uint16_t stat_batt_power = 0;      // Power going in/out of battery
uint16_t cell_max_voltage = 3700;  // Stores the highest cell voltage value in the system
uint16_t cell_min_voltage = 3700;  // Stores the minimum cell voltage value in the system

typedef enum _solax_state
{
  SOLAX_BATTERY_ANNOUNCE,
  SOLAX_WAITING_FOR_CONTACTOR,
  SOLAX_CONTACTOR_CLOSED,
  SOLAX_FAULT_SOLAX,
  SOLAX_UPDATING_FW
} SOLAX_STATE;

struct solax_data
{
  SOLAX_STATE state;
  uint16_t max_charge_rate_amp;
  uint16_t max_discharge_rate_amp;
  uint16_t temperature_average;
  unsigned long LastFrameTime;
  int number_of_batteries;
};

struct solax_message
{
  uint32_t id;
  uint8_t data[8];
};

static struct solax_message solax_1801 = {0x1801, {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};
static struct solax_message solax_1872 = {0x1872, {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};  //BMS_Limits
static struct solax_message solax_1873 = {0x1873, {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};  //BMS_PackData
static struct solax_message solax_1874 = {0x1874, {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};  //BMS_CellData
static struct solax_message solax_1875 = {0x1875, {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};  //BMS_Status
static struct solax_message solax_1876 = {0x1876, {0x0, 0x0, 0xE2, 0x0C, 0x0, 0x0, 0xD7, 0x0C}};  //BMS_PackTemps
static struct solax_message solax_1877 = {0x1877, {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};
static struct solax_message solax_1878 = {0x1878, {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};  //BMS_PackStats
static struct solax_message solax_1879 = {0x1879, {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};
static struct solax_message solax_1881 = {0x1881, {0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};  // E.g.: 0 6 S B M S F A
static struct solax_message solax_1882 = {0x1882, {0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}};  // E.g.: 0 2 3 A B 0 5 2
static struct solax_message solax_100A001 = {0x100A001, {}};

static struct solax_data solax_data;

static HAL_StatusTypeDef solax_send_message(struct solax_message *msg)
{
  CAN_TxHeaderTypeDef TxHeader;

  TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.ExtId = msg->id;

  return MX_CAN_Transmit(&hcan1, &TxHeader, msg->data);
}

/**
  * @brief  Process CAN2 Solax data
  * @retval true if TX work is pending
  */
static bool Solax_Process_RX(void)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t data[8];
  bool ret = false;

  HAL_GPIO_WritePin(GPIOE, INVERTER_Pin, GPIO_PIN_RESET);

  while (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO1) > 0)
  {
    /* Read the message */
    if (HAL_OK == HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &RxHeader, data))
    {
      // ToDo: Process the data
    }
  }

  HAL_GPIO_WritePin(GPIOE, INVERTER_Pin, GPIO_PIN_SET);

  return ret;
}

void solax_process(void)
{

}

void solax_set_max_ac_current(uint8_t current)
{
  // ToDo: We need to apply this ASAP!
  // Tell the inverter to stop any output when 0
  // Need to decide whether to keep EPS powered up?

  // ToDo: Block this function until we're within the limit
  // This will be used by the EVSE and ChaDeMo logic to clear locks where appropriate.

  // Timeout ~50ms max?

}


void solax_set_max_dc_chg_power(uint32_t power)
{

}

void solax_set_max_dc_dis_power(uint32_t power)
{
  
}


#if 0
void update_values_can_solax() {  //This function maps all the values fetched from battery CAN to the correct CAN messages
  // If not receiveing any communication from the inverter, open contactors and return to battery announce state
  if (millis() - LastFrameTime >= SolaxTimeout) {
    inverterAllowsContactorClosing = false;
    STATE = BATTERY_ANNOUNCE;
  }
  //Calculate the required values
  temperature_average = ((temperature_max + temperature_min) / 2);

  //max_target_charge_power (30000W max)
  if (SOC > 9999)  //99.99%
  {                //Additional safety incase SOC% is 100, then do not charge battery further
    max_charge_rate_amp = 0;
  } else {  //We can pass on the battery charge rate (in W) to the inverter (that takes A)
    if (max_target_charge_power >= 30000) {
      max_charge_rate_amp = 75;  //Incase battery can take over 30kW, cap value to 75A
    } else {                     //Calculate the W value into A
      max_charge_rate_amp = (max_target_charge_power / (battery_voltage * 0.1));  // P/U = I
    }
  }

  //max_target_discharge_power (30000W max)
  if (SOC < 100)  //1.00%
  {               //Additional safety incase SOC% is below 1, then do not charge battery further
    max_discharge_rate_amp = 0;
  } else {  //We can pass on the battery discharge rate to the inverter
    if (max_target_discharge_power >= 30000) {
      max_discharge_rate_amp = 75;  //Incase battery can be charged with over 30kW, cap value to 75A
    } else {                        //Calculate the W value into A
      max_discharge_rate_amp = (max_target_discharge_power / (battery_voltage * 0.1));  // P/U = I
    }
  }

  //Put the values into the CAN messages
  //BMS_Limits
  SOLAX_1872.data.u8[0] = (uint8_t)max_voltage;  //TODO: scaling OK?
  SOLAX_1872.data.u8[1] = (max_voltage >> 8);
  SOLAX_1872.data.u8[2] = (uint8_t)min_voltage;  //TODO: scaling OK?
  SOLAX_1872.data.u8[3] = (min_voltage >> 8);
  SOLAX_1872.data.u8[4] = (uint8_t)(max_charge_rate_amp * 10);  //TODO: scaling OK?
  SOLAX_1872.data.u8[5] = ((max_charge_rate_amp * 10) >> 8);
  SOLAX_1872.data.u8[6] = (uint8_t)(max_discharge_rate_amp * 10);  //TODO: scaling OK?
  SOLAX_1872.data.u8[7] = ((max_discharge_rate_amp * 10) >> 8);

  //BMS_PackData
  SOLAX_1873.data.u8[0] = (uint8_t)battery_voltage;  // OK
  SOLAX_1873.data.u8[1] = (battery_voltage >> 8);
  SOLAX_1873.data.u8[2] = (int8_t)battery_current;  // OK, Signed (Active current in Amps x 10)
  SOLAX_1873.data.u8[3] = (battery_current >> 8);
  SOLAX_1873.data.u8[4] = (uint8_t)(SOC / 100);  //SOC (100.00%)
  //SOLAX_1873.data.u8[5] = //Seems like this is not required? Or shall we put SOC decimals here?
  SOLAX_1873.data.u8[6] = (uint8_t)(remaining_capacity_Wh / 100);  //TODO: scaling OK?
  SOLAX_1873.data.u8[7] = ((remaining_capacity_Wh / 100) >> 8);

  //BMS_CellData
  SOLAX_1874.data.u8[0] = (uint8_t)temperature_max;
  SOLAX_1874.data.u8[1] = (temperature_max >> 8);
  SOLAX_1874.data.u8[2] = (uint8_t)temperature_min;
  SOLAX_1874.data.u8[3] = (temperature_min >> 8);
  SOLAX_1874.data.u8[4] =
      (uint8_t)(cell_max_voltage);  //TODO: scaling OK? Supposed to be alarm trigger absolute cell max?
  SOLAX_1874.data.u8[5] = (cell_max_voltage >> 8);
  SOLAX_1874.data.u8[6] =
      (uint8_t)(cell_min_voltage);  //TODO: scaling OK? Supposed to be alarm trigger absolute cell min?
  SOLAX_1874.data.u8[7] = (cell_min_voltage >> 8);

  //BMS_Status
  SOLAX_1875.data.u8[0] = (uint8_t)temperature_average;
  SOLAX_1875.data.u8[1] = (temperature_average >> 8);
  SOLAX_1875.data.u8[2] = (uint8_t)0;  // Number of slave batteries
  SOLAX_1875.data.u8[4] = (uint8_t)0;  // Contactor Status 0=off, 1=on.

  //BMS_PackTemps (strange name, since it has voltages?)
  SOLAX_1876.data.u8[2] = (uint8_t)cell_max_voltage;  //TODO: scaling OK?
  SOLAX_1876.data.u8[3] = (cell_max_voltage >> 8);

  SOLAX_1876.data.u8[6] = (uint8_t)cell_min_voltage;  //TODO: scaling OK?
  SOLAX_1876.data.u8[7] = (cell_min_voltage >> 8);

  //Unknown
  SOLAX_1877.data.u8[4] = (uint8_t)0x50;  // Battery type
  SOLAX_1877.data.u8[6] = (uint8_t)0x22;  // Firmware version?
  SOLAX_1877.data.u8[7] =
      (uint8_t)0x02;  // The above firmware version applies to:02 = Master BMS, 10 = S1, 20 = S2, 30 = S3, 40 = S4

  //BMS_PackStats
  SOLAX_1878.data.u8[0] = (uint8_t)(battery_voltage);  //TODO: should this be max or current voltage?
  SOLAX_1878.data.u8[1] = ((battery_voltage) >> 8);

  SOLAX_1878.data.u8[4] = (uint8_t)capacity_Wh;  //TODO: scaling OK?
  SOLAX_1878.data.u8[5] = (capacity_Wh >> 8);

  // BMS_Answer
  SOLAX_1801.data.u8[0] = 2;
  SOLAX_1801.data.u8[2] = 1;
  SOLAX_1801.data.u8[4] = 1;
}

void receive_can_solax(CAN_frame_t rx_frame) {
  if (rx_frame.MsgID == 0x1871 && rx_frame.data.u8[0] == (0x01) ||
      rx_frame.MsgID == 0x1871 && rx_frame.data.u8[0] == (0x02)) {
    LastFrameTime = millis();
    switch (STATE) {
      case (BATTERY_ANNOUNCE):
        Serial.println("Solax Battery State: Announce");
        inverterAllowsContactorClosing = false;
        SOLAX_1875.data.u8[4] = (0x00);  // Inform Inverter: Contactor 0=off, 1=on.
        for (int i = 0; i <= number_of_batteries; i++) {
          CAN_WriteFrame(&SOLAX_1872);
          CAN_WriteFrame(&SOLAX_1873);
          CAN_WriteFrame(&SOLAX_1874);
          CAN_WriteFrame(&SOLAX_1875);
          CAN_WriteFrame(&SOLAX_1876);
          CAN_WriteFrame(&SOLAX_1877);
          CAN_WriteFrame(&SOLAX_1878);
        }
        CAN_WriteFrame(&SOLAX_100A001);  //BMS Announce
        // Message from the inverter to proceed to contactor closing
        // Byte 4 changes from 0 to 1
        if (rx_frame.data.u64 == Contactor_Close_Payload)
          STATE = WAITING_FOR_CONTACTOR;
        break;

      case (WAITING_FOR_CONTACTOR):
        SOLAX_1875.data.u8[4] = (0x00);  // Inform Inverter: Contactor 0=off, 1=on.
        CAN_WriteFrame(&SOLAX_1872);
        CAN_WriteFrame(&SOLAX_1873);
        CAN_WriteFrame(&SOLAX_1874);
        CAN_WriteFrame(&SOLAX_1875);
        CAN_WriteFrame(&SOLAX_1876);
        CAN_WriteFrame(&SOLAX_1877);
        CAN_WriteFrame(&SOLAX_1878);
        CAN_WriteFrame(&SOLAX_1801);  // Announce that the battery will be connected
        STATE = CONTACTOR_CLOSED;     // Jump to Contactor Closed State
        Serial.println("Solax Battery State: Contactor Closed");
        break;

      case (CONTACTOR_CLOSED):
        inverterAllowsContactorClosing = true;
        SOLAX_1875.data.u8[4] = (0x01);  // Inform Inverter: Contactor 0=off, 1=on.
        CAN_WriteFrame(&SOLAX_1872);
        CAN_WriteFrame(&SOLAX_1873);
        CAN_WriteFrame(&SOLAX_1874);
        CAN_WriteFrame(&SOLAX_1875);
        CAN_WriteFrame(&SOLAX_1876);
        CAN_WriteFrame(&SOLAX_1877);
        CAN_WriteFrame(&SOLAX_1878);
        // Message from the inverter to open contactor
        // Byte 4 changes from 1 to 0
        if (rx_frame.data.u64 == Contactor_Open_Payload)
          STATE = BATTERY_ANNOUNCE;
        break;
    }
  }

  if (rx_frame.MsgID == 0x1871 && rx_frame.data.u64 == __builtin_bswap64(0x0500010000000000)) {
    CAN_WriteFrame(&SOLAX_1881);
    CAN_WriteFrame(&SOLAX_1882);
    Serial.println("1871 05-frame received from inverter");
  }
  if (rx_frame.MsgID == 0x1871 && rx_frame.data.u8[0] == (0x03)) {
    Serial.println("1871 03-frame received from inverter");
  }
}
#endif