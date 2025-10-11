/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "iwdg.h"
#include "usart.h"

#include "chademo.h"
#include "evse.h"
#include "ioexp.h"
#include "modbus.h"
#include "sensor.h"
#include "solax.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define DEBUG_CONTROLLER

#define JSON_UPDATE_TIME    (5000)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
enum ccs_state
{
  CCS_OFF = 0,                      /* CP Pilot Signal Off */
  CCS_PILOT,                        /* CP Pilot Signal at 5% */
  CCS_ERROR = 8
};

/* Upper / Red (bits 8-15) and Lower / Green (bits 0-7) Debug Leds */
enum debug_leds
{
  DBG_LED_HV_TEST = 0,
  DBG_LED_ISO_TEST,
  DBG_LED_PRECHARGE,
  DBG_LED_HV_ENABLED,

  DBG_LED_PP_INSERTED = 8,
  DBG_LED_CP_READY,
  DBG_LED_MAX_CURRENT,
};

static bool error = false;            /* Whether we are in the error state */
static int32_t power_offset = 0;      /* Offset from actual power (i.e. charge / discharge) */

static uint16_t debug_leds = 0;       /* Combined state of debug leds */

static uint8_t comm_count = 0;        /* Number of active comm sessions (I2C / SPI / UART) */

/* CCS Related Variables */
static enum ccs_state ccs_state = CCS_OFF;

/* USER CODE END Variables */
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for jsonTask */
osThreadId_t jsonTaskHandle;
const osThreadAttr_t jsonTask_attributes = {
  .name = "jsonTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for jsonMutex */
osSemaphoreId_t jsonMutexHandle;
const osSemaphoreAttr_t jsonMutex_attributes = {
  .name = "jsonMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void pp_changed_cb(EVSE_PP pp, uint8_t current);
static void cp_changed_cb(EVSE_CP cp);

/* USER CODE END FunctionPrototypes */

void mainTaskEntry(void *argument);
void jsonTaskEntry(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
    /* Re-purpose the EVSE LED to show when we're busy */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    __WFI();
    HAL_GPIO_WritePin(LED_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
}
/* USER CODE END 2 */

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t ulExpectedIdleTime)
{
/* place for user code */
}

__weak void PostSleepProcessing(uint32_t ulExpectedIdleTime)
{
/* place for user code */
}
/* USER CODE END PREPOSTSLEEP */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of jsonMutex */
  jsonMutexHandle = osSemaphoreNew(1, 0, &jsonMutex_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(mainTaskEntry, NULL, &mainTask_attributes);

  /* creation of jsonTask */
  jsonTaskHandle = osThreadNew(jsonTaskEntry, NULL, &jsonTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_mainTaskEntry */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_mainTaskEntry */
void mainTaskEntry(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN mainTaskEntry */

  /* Turn on the EVSE */
  HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_SET);

  /* Power Up ESP8266 */
  HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);

  /* Initialise Top IO expander and clear LEDs */
  if (ioexp_init(IOEXP_TOP_LEDS))
  {
    ioexp_set_direction(IOEXP_TOP_LEDS, 0xFFFF);
    ioexp_set_output(IOEXP_TOP_LEDS, 0x0000);
  }
  else
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise Top LED IO Expander.\"}]\n");
    error = true;
  }

  /* Initialise Bottom LED IO expander and clear LEDs */
  if (ioexp_init(IOEXP_BOT_LEDS))
  {
    ioexp_set_direction(IOEXP_BOT_LEDS, 0xFFFF);
    ioexp_set_output(IOEXP_BOT_LEDS, 0x0000);
  }
  else
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise Bottom LED IO Expander.\"}]\n");
    error = true;
  }

  /* Give USB and ESP a chance to start up */
  osDelay(3000);

  /* Initialise Sensors and Peripherals */

  if (!sensor_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise sensors.\"}]\n");
    error = true;
  }

#ifdef ENABLE_EVSE
  if (!evse_init(&pp_changed_cb, &cp_changed_cb))
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise EVSE interface.\"}]\n");
    error = true;
  }
#endif

#ifdef ENABLE_CHADEMO
  if (!chademo_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise ChaDeMo interface.\"}]\n");
    error = true;
  }
#endif

#ifdef ENABLE_SOLAX
  if (!solax_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise Solax interface.\"}]\n");
    error = true;
  }
#endif

  if (!modbus_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise Modbus interface.\"}]\n");
    error = true;
  }
  HAL_UART_Setup_UART1();

  if (!error)
    printf("{\"controller\":[{\"status\":0,\"message\":\"Initialized OK\"}]}\n");

#ifdef ENABLE_CHADEMO
  /* Set Maximum DC power. Import / Export will be controlled separately. */
  chademo_set_max_power(SOLAX_MINIMUM_SUPPORTED_VOLTAGE * SOLAX_MAXIMUM_SUPPORTED_CURRENT);
#endif

  MX_IWDG_Init();

  /* Infinite loop */
  for(;;)
  {
    /* Catch any errors and EStop */
    if (error)
    {
      emergency_stop();
    }

    /* Leds (Top 2 rows) */
    {
      static uint16_t old_leds = 0;
      if (debug_leds != old_leds)
      {
        old_leds = debug_leds;
        ioexp_set_direction(IOEXP_TOP_LEDS, ~debug_leds);
      }
    }

#ifdef ENABLE_CHADEMO
    /* Debug LEDs */
    {
      static uint16_t old_leds = 0;
      uint16_t leds = 0;

      if (GPIO_PIN_SET == HAL_GPIO_ReadPin(CHADEMO_CP_GPIO_Port, CHADEMO_CP_Pin))
        leds |= 1 << 8;

      if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(CHADEMO_CHARGE_ALLOWED__GPIO_Port, CHADEMO_CHARGE_ALLOWED__Pin))
        leds |= 1 << 9;

      if (leds != old_leds)
      {
        old_leds = leds;
        ioexp_set_direction(IOEXP_BOT_LEDS, ~leds);
      }
    }

    /* Check shutdown */
    if (power_offset == 0 &&
        chademo_get_state() == CHADEMO_STATE_ON)
    {
      chademo_stop();
    }

    /* Check startup */
    if (power_offset != 0 &&
        chademo_get_state() == CHADEMO_STATE_OFF)
    {
      chademo_start();
    }

    /* Prevent continuous loop if we shut down */
    // ToDo: CAN timeout case?
    if (chademo_get_state() >= CHADEMO_STATE_STOP)
    {
      power_offset = 0;
    }

    /* Set Solax state based on ChaDeMo */
    if (chademo_get_state() == CHADEMO_STATE_ON)
    {
      solax_enable();
      solax_kick();
    }
    if (chademo_get_state() > CHADEMO_STATE_STOP)
    {
      solax_disable();
      solax_kick();
    }
#endif

    /* Update the inverter power */
    solax_set_output_power(power_offset);

    /* Kick the Watchdog */
    HAL_IWDG_Refresh(&hiwdg);

	  osDelay(100);
  }
  /* USER CODE END mainTaskEntry */
}

/* USER CODE BEGIN Header_jsonTaskEntry */
/**
* @brief Function implementing the jsonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_jsonTaskEntry */
void jsonTaskEntry(void *argument)
{
  /* USER CODE BEGIN jsonTaskEntry */
  /* Infinite loop */
  for(;;)
  {
    /* Send regular JSON messages */
    xSemaphoreTake(jsonMutexHandle, JSON_UPDATE_TIME);

#ifdef DEBUG_CONTROLLER
    int32_t acc_current = -1;
    int32_t hv_current = -1;
    int32_t batt_voltage = -1;
    int32_t inv_voltage = -1;

#ifdef ENABLE_INA219
    sensor_get_value(SENSOR_ACC_CURRENT, &acc_current);
    sensor_get_value(SENSOR_HV_TEST_CURRENT, &hv_current);
#endif
#ifdef ENABLE_MAX22530
    sensor_get_value(SENSOR_BATT_VOLTAGE, &batt_voltage);
    sensor_get_value(SENSOR_INV_VOLTAGE, &inv_voltage);
#endif

#endif

    printf("{\"controller\":{");
    printf("\"power_offset\":%ld", power_offset);
    printf(",\"timestamp\":%ld", HAL_GetTick());
#ifdef DEBUG_CONTROLLER
    printf(",\"acc_current\":%ld", acc_current / 1000);
    printf(",\"hv_current\":%ld", hv_current);
    printf(",\"batt_voltage\":%ld", batt_voltage);
    printf(",\"inv_voltage\":%ld", inv_voltage);
#endif
    printf(",");
    evse_json_update();
    printf(",");
    solax_json_update();
#ifdef ENABLE_CHADEMO
    printf(",");
    chademo_json_update();
#endif
    printf("}}\n");
  }
  /* USER CODE END jsonTaskEntry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  Forcibly shut everything down
  * @retval None
  */
void emergency_stop(void)
{
  volatile uint32_t i;

  /* Stop Everything in the system */
  __disable_irq();

  /* Turn Off EVSE */
  HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_RESET);

  /* Force DC contactors off */
  HAL_GPIO_WritePin(CHADEMO_SEQ2_GPIO_Port, CHADEMO_SEQ2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CHADEMO_SEQ1_GPIO_Port, CHADEMO_SEQ1_Pin, GPIO_PIN_RESET);

  /* Force Leak Test Off */
  HAL_GPIO_WritePin(LEAK_TEST_EN_GPIO_Port, LEAK_TEST_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ISO_TEST_EN_GPIO_Port, ISO_TEST_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TEST_HV_EN_GPIO_Port, TEST_HV_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HV_EN_GPIO_Port, HV_EN_Pin, GPIO_PIN_RESET);

  while(1)
  {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    for (i=0; i<1000000; ++i);

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    for (i=0; i<1000000; ++i);
  }
}

/**
  * @brief  Indicate the state of a comms session.
  * @param start_stop: Whether we are starting or stopping a session
  * @retval None
  */
void comm_session(bool start_stop)
{
  if (start_stop)
  {
    if (comm_count == 0)
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    comm_count++;
  }
  else
  {
    if (comm_count > 0)
      comm_count--;

    if (comm_count == 0)
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  }
}

/**
  * @brief  Trigger an update of the JSON output.
  * @retval None
  */
void trigger_json_update(void)
{
  xSemaphoreGive(jsonMutexHandle);
}

/**
  * @brief  EVSE PP and Current callback.
  * @param  pp Current connector state
  * @param  current Maximum AC current (in / out)
  * @retval None
  */
static void pp_changed_cb(EVSE_PP pp, uint8_t current)
{
  switch (pp)
  {
    case EVSE_PP_INSERTED:
      /* Enable the CP Line */
      /* This tells the EVSE to start charging (supply power) */
      HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_SET);
      debug_leds |= (1 << DBG_LED_PP_INSERTED);
    break;

    default:

    case EVSE_PP_NONE:
      /* Disable the CP line */
      HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_RESET);
    /* break; */  /* Deliberate fall through */

    case EVSE_PP_PRESSED:
      /* Update the inverter max */
      power_offset = 0;
      chademo_stop();
      debug_leds &= ~(1 << DBG_LED_PP_INSERTED);
    break;
  }

#ifdef ENABLE_SOLAX
  /* Update the inverter max (BMS / DC handled by ChaDeMo). */
  solax_set_max_ac_current(current);
#endif
}

/**
  * @brief  EVSE CP callback.
  * @param  cp Current vehicle state
  * @retval None
  */
static void cp_changed_cb(EVSE_CP cp)
{
  if (cp > EVSE_CP_B)
    debug_leds |= (1 << DBG_LED_CP_READY);
  else
    debug_leds &= ~(1 << DBG_LED_CP_READY);
}

/* Command Handlers (Application Level) */

/**
  * @brief  Process Power Setting command
  * @param  args Command arguments
  * @param  argc Number of arguments
  * @retval Status (0 = OK, -1 = Error / Unknown Command)
  */
int app_process_cmd_power(char **args, int argc)
{
  if (argc >= 1)
  {
    power_offset = strtol(args[0], NULL, 10);
  }
  return 0;
}


/**
  * @brief  Process HV Test commands
  * @param  args Command arguments
  * @param  argc Number of arguments
  * @retval Status (0 = OK, -1 = Error / Unknown Command)
  */
int app_process_cmd_hv(char **args, int argc)
{
  if (argc >= 2)
  {
    if (0 == strcmp(args[0], "src"))
    {
      switch (strtol(args[1], NULL, 10))
      {
        case 0:
          /* Disable HV Test Source */
          HAL_GPIO_WritePin(TEST_HV_EN_GPIO_Port, TEST_HV_EN_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(HV_EN_GPIO_Port, HV_EN_Pin, GPIO_PIN_RESET);
          debug_leds &= ~(1 << DBG_LED_HV_TEST);
        break;

        case 1:
          /* Enable HV Test Source */
          HAL_GPIO_WritePin(TEST_HV_EN_GPIO_Port, TEST_HV_EN_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(HV_EN_GPIO_Port, HV_EN_Pin, GPIO_PIN_SET);
          debug_leds |= (1 << DBG_LED_HV_TEST);
        break;

        default:
        break;
      }
    }
    else if (0 == strcmp(args[0], "iso"))
    {
      switch (strtol(args[1], NULL, 10))
      {
        case 0:
          /* Disable ISO Test */
          HAL_GPIO_WritePin(ISO_TEST_EN_GPIO_Port, ISO_TEST_EN_Pin, GPIO_PIN_RESET);
          debug_leds &= ~(1 << DBG_LED_ISO_TEST);
        break;

        case 1:
          /* Enable ISO Test */
          HAL_GPIO_WritePin(ISO_TEST_EN_GPIO_Port, ISO_TEST_EN_Pin, GPIO_PIN_SET);
          debug_leds |= (1 << DBG_LED_ISO_TEST);
        break;

        default:
        break;
      }
    }
  }
  return 0;
}

/* USER CODE END Application */

