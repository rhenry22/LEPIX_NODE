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
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "semphr.h"
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

#define JSON_UPDATE_TIME         (5000)

#define HIGH_VOLTAGE_THRESHOLD   (500) /* 50V */
#define HIGH_VOLTAGE_TIMEOUT     (10000) /* Allow the HV source to be left on for this duration (max) */

#define BUTTON_DEBOUNCE_TIME     (150)
/* How often to toggle flashing LEDs (ms) */
#define FLASH_TOGGLE_TIME        (500)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* Upper / Red (bits 8-15) and Lower / Green (bits 0-7) Debug Leds */
enum debug_leds
{
  DBG_LED_HV_TEST = 0,
  DBG_LED_ISO_TEST,
  DBG_LED_CT_PRE,
  DBG_LED_CT_MAIN,
  DBG_LED_STAT_RED_INV,
  DBG_LED_STAT_RED_EV,
  DBG_LED_HV_INV,
  DBG_LED_HV_BATT,

  DBG_LED_PP_INSERTED = 8,
  DBG_LED_CP_READY,
  DBG_LED_CP_CHARGE,
};

static bool error = false;            /* Whether we are in the error state */
static int32_t power_offset = 0;      /* Offset from actual power (i.e. charge / discharge) */
static uint16_t debug_leds = 0;       /* Combined state of debug leds */

static uint8_t comm_count = 0;        /* Number of active comm sessions (I2C / SPI / UART) */
static uint32_t hv_time = 0;          /* When the HV source was enabled */

/* Flashing LED state: which debug LEDs should flash and which user GPIOs */
static uint16_t flash_debug_leds = 0; /* Bits for debug LEDs that should flash */
static uint8_t flash_user_mask = 0;   /* Bits (0x01,0x02) for user GPIOs that should flash */
static uint8_t user_led_base = 0;     /* Base values for user LEDs (bit0 -> GPIO1, bit1 -> GPIO2) */
static bool flash_state = false;      /* Current on/off state for flashed LEDs */
static uint32_t last_flash_toggle = 0;/* Last tick when flash_state toggled */

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

  /* Request supply from the EVSE */
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

  if (!cmd_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise command parser.\"}]\n");
    error = true;
  }

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
  else
  {
    evse_set_cp(100);
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

    /* Leds (Top 2 rows) + flashing support */
    {
      static uint16_t old_display_leds = 0;

      /* Handle flash toggle timing */
      if (flash_debug_leds != 0 || flash_user_mask != 0)
      {
        uint32_t now = HAL_GetTick();
        if (last_flash_toggle == 0 || (now - last_flash_toggle) >= FLASH_TOGGLE_TIME)
        {
          flash_state = !flash_state;
          last_flash_toggle = now;
        }
      }

      /* Compute the LEDs to display on the top IO expander, applying flashing */
      uint16_t display_leds = debug_leds;
      if (flash_debug_leds != 0 && !flash_state)
      {
        /* When flash_state is false, clear the flashing bits so they appear off */
        display_leds &= ~flash_debug_leds;
      }

      if (display_leds != old_display_leds)
      {
        old_display_leds = display_leds;
        ioexp_set_direction(IOEXP_TOP_LEDS, ~display_leds);
      }

      /* Update user GPIO LEDs (GPIO1 / GPIO2) according to flash state */
      /* If a user LED is marked for flashing, show flash_state, otherwise show base value */
      if (flash_user_mask & 0x01)
      {
        HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, (flash_state ? GPIO_PIN_SET : GPIO_PIN_RESET));
      }
      else
      {
        HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, (user_led_base & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }

      if (flash_user_mask & 0x02)
      {
        HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, (flash_state ? GPIO_PIN_SET : GPIO_PIN_RESET));
      }
      else
      {
        HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, (user_led_base & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }
    }

#ifdef ENABLE_CHADEMO
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
    }
    if (chademo_get_state() > CHADEMO_STATE_STOP)
    {
      solax_disable();
    }
#endif

    /* User Buttons and LEDs */
    {
      static uint32_t button_time = 0;            /* Timer for button LED(s) */

      if (HAL_GPIO_ReadPin(GPIO3_GPIO_Port, GPIO3_Pin) == GPIO_PIN_SET)
        button_time = 0;

      /* Button Press */
      if ((button_time == 0 || (HAL_GetTick() - button_time) > BUTTON_DEBOUNCE_TIME) &&
          HAL_GPIO_ReadPin(GPIO3_GPIO_Port, GPIO3_Pin) == GPIO_PIN_RESET)
      {
        button_time = HAL_GetTick();
        printf("{\"controller\":[{\"button\":1}]}\n");
      }
    }

    /* Check HV Source */
    if (hv_time != 0 && HAL_GetTick() - hv_time > HIGH_VOLTAGE_TIMEOUT)
    {
      app_process_cmd_hv((char*[]){"iso", "0"}, 2);
      app_process_cmd_hv((char*[]){"src", "0"}, 2);
    }

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

    int32_t acc_current = -1;
    int32_t hv_current = -1;
    int32_t batt_current = -1;
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

    sensor_get_value(SENSOR_BATT_CURRENT, &batt_current);

    /* Display Batt and Inverter Voltages on LEDs (300-500v) */
    {
      int32_t b;
      int32_t i;
      b = (batt_voltage - 3000) * 8 / 2000;
      if (b < 0) b = 0;
      b = ((1 >> b) - 1) & 0xff;
      i = (inv_voltage - 3000) * 8 / 2000;
      if (i < 0) i = 0;
      i = ((1 >> i) - 1) & 0xff;
      ioexp_set_direction(IOEXP_BOT_LEDS, ~(b << 8 | i));
    }

    /* Update Debug LEDs */
    if (batt_voltage > HIGH_VOLTAGE_THRESHOLD)
      debug_leds |= (1 << DBG_LED_HV_BATT);
    else
      debug_leds &= ~(1 << DBG_LED_HV_BATT);

    if (inv_voltage > HIGH_VOLTAGE_THRESHOLD)
      debug_leds |= (1 << DBG_LED_HV_INV);
    else
      debug_leds &= ~(1 << DBG_LED_HV_INV);

    if (HAL_GPIO_ReadPin(CTPRE_EN_GPIO_Port, CTPRE_EN_Pin) == GPIO_PIN_SET)
      debug_leds |= (1 << DBG_LED_CT_PRE);
    else
      debug_leds &= ~(1 << DBG_LED_CT_PRE);

    if (HAL_GPIO_ReadPin(CTMAIN_EN_GPIO_Port, CTMAIN_EN_Pin) == GPIO_PIN_SET)
      debug_leds |= (1 << DBG_LED_CT_MAIN);
    else
      debug_leds &= ~(1 << DBG_LED_CT_MAIN);

    printf("{\"controller\":{");
    printf("\"power_offset\":%ld", power_offset);
    printf(",\"timestamp\":%ld", HAL_GetTick());

    printf(",\"acc_current\":%ld", acc_current);
    printf(",\"hv_current\":%ld", hv_current);
    printf(",\"batt_voltage\":%ld", batt_voltage / 10);
    printf(",\"batt_current\":%ld", batt_current / 10);
    printf(",\"inv_voltage\":%ld", inv_voltage / 10);

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
  if (xPortIsInsideInterrupt())
  {
    BaseType_t pxHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(jsonMutexHandle, &pxHigherPriorityTaskWoken);
  }
  else
  {
    xSemaphoreGive(jsonMutexHandle);
  }
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
      solax_set_output_power(0);
#ifdef ENABLE_CHADEMO
      chademo_stop(); /* Ensure ChaDeMo is stopped */
#endif
      debug_leds &= ~(1 << DBG_LED_PP_INSERTED);
    break;
  }

#ifdef ENABLE_SOLAX
  /* Update the inverter max (BMS / DC handled by ChaDeMo). */
  solax_set_max_ac_current(current);
#endif

  trigger_json_update();
}

/**
  * @brief  EVSE CP callback.
  * @param  cp Current vehicle state
  * @retval None
  */
static void cp_changed_cb(EVSE_CP cp)
{
  switch (cp)
  {
    case EVSE_CP_A:
      solax_disable();
      debug_leds &= ~(1 << DBG_LED_CP_READY);
      debug_leds &= ~(1 << DBG_LED_CP_CHARGE);
    break;

    case EVSE_CP_B:
      solax_set_output_power(0);
      debug_leds |= (1 << DBG_LED_CP_READY);
      debug_leds &= ~(1 << DBG_LED_CP_CHARGE);
    break;

    case EVSE_CP_C:
    case EVSE_CP_D:
      debug_leds |= (1 << DBG_LED_CP_READY);
      debug_leds |= (1 << DBG_LED_CP_CHARGE);
    break;
  }

  trigger_json_update();
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
  if (argc == 1)
  {
    power_offset = strtol(args[0], NULL, 10);
    solax_set_output_power(power_offset);
    return 0;
  }
  return -1;
}

int app_process_cmd_leds(char **args, int argc)
{
  int ret = 0;
  if (argc >= 3 && 0 == strcmp(args[0], "debug"))
  {
    int16_t mask = strtol(args[1], NULL, 16);
    int16_t val = strtol(args[2], NULL, 16);

    /* Only modify bits covered by mask */
    debug_leds &= ~mask;
    debug_leds |= (val & mask);
    /* Optional 4th arg: "flash" -> add any bits set to 1 (mask & val) to flash list. */
    if (argc >= 4 && 0 == strcmp(args[3], "flash"))
    {
      /* Add bits where mask says and val is 1 */
      flash_debug_leds |= (mask & val);
      /* Remove any bits from flash list where mask requested clearing (mask & ~val) */
      flash_debug_leds &= ~(mask & ~val);
    }
    else
    {
      /* No explicit "flash" arg -> clear flashing for the bits covered by mask */
      flash_debug_leds &= ~((uint16_t)mask);
    }
  }
  else if (argc >= 3 && 0 == strcmp(args[0], "user"))
  {
    int16_t mask = strtol(args[1], NULL, 16);
    int16_t val = strtol(args[2], NULL, 16);

    /* Update base user LED values */
    user_led_base &= ~mask;
    user_led_base |= (val & mask);

    /* Optional flash parameter: add/remove flashing for the bits being set */
    if (argc >= 4 && 0 == strcmp(args[3], "flash"))
    {
      /* Add bits where mask says and val is 1 */
      flash_user_mask |= (mask & val);
      /* Remove any bits from flash list where mask requested clearing (mask & ~val) */
      flash_user_mask &= ~(mask & ~val);
    }
    else
    {
      /* No explicit "flash" arg -> clear flashing for the bits covered by mask */
      flash_user_mask &= ~mask;
    }

    /* Immediately apply the current visible state for user LEDs (honour flash_state)
       If a user LED is flashing, show flash_state, otherwise show base value */
    if (flash_user_mask & 0x01)
      HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, (flash_state ? GPIO_PIN_SET : GPIO_PIN_RESET));
    else
      HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, (user_led_base & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (flash_user_mask & 0x02)
      HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, (flash_state ? GPIO_PIN_SET : GPIO_PIN_RESET));
    else
      HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, (user_led_base & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else
  {
    ret = -1;
  }

  return ret;
}


/**
  * @brief  Process HV Test commands
  * @param  args Command arguments
  * @param  argc Number of arguments
  * @retval Status (0 = OK, -1 = Error / Unknown Command)
  */
int app_process_cmd_hv(char **args, int argc)
{
  int ret = -1;
  if (argc >= 2 && 0 == strcmp(args[0], "src"))
  {
    switch (strtol(args[1], NULL, 10))
    {
      case 0:
        /* Disable HV Test Source */
        HAL_GPIO_WritePin(TEST_HV_EN_GPIO_Port, TEST_HV_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HV_EN_GPIO_Port, HV_EN_Pin, GPIO_PIN_RESET);
        debug_leds &= ~(1 << DBG_LED_HV_TEST);
        hv_time = 0;
        ret = 0;
      break;

      case 1:
        /* Enable HV Test Source */
        HAL_GPIO_WritePin(TEST_HV_EN_GPIO_Port, TEST_HV_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(HV_EN_GPIO_Port, HV_EN_Pin, GPIO_PIN_SET);
        debug_leds |= (1 << DBG_LED_HV_TEST);
        hv_time = HAL_GetTick();
        ret = 0;
      break;

      default:
      break;
    }
  }
  else if (argc >= 2 && 0 == strcmp(args[0], "iso"))
  {
    switch (strtol(args[1], NULL, 10))
    {
      case 0:
        /* Disable ISO Test */
        HAL_GPIO_WritePin(ISO_TEST_EN_GPIO_Port, ISO_TEST_EN_Pin, GPIO_PIN_RESET);
        debug_leds &= ~(1 << DBG_LED_ISO_TEST);
        ret = 0;
      break;

      case 1:
        /* Enable ISO Test */
        HAL_GPIO_WritePin(ISO_TEST_EN_GPIO_Port, ISO_TEST_EN_Pin, GPIO_PIN_SET);
        debug_leds |= (1 << DBG_LED_ISO_TEST);
        ret = 0;
      break;

      default:
      break;
    }
  }
  else if (argc >= 1 && 0 == strcmp(args[0], "get"))
  {
    trigger_json_update();
    ret = 0;
  }
  return ret;
}

/* USER CODE END Application */

