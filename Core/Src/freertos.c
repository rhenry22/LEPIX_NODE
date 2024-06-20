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
#include "semphr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "iwdg.h"
#include "usart.h"

#include "usbd_cdc_if.h"

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

#define ENABLE_EVSE
#define ENABLE_CHADEMO
#define ENABLE_SOLAX

#define DEBUG_CONTROLLER

#define JSON_UPDATE_TIME    (5000)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static bool error = false;            /* Whether we are in the error state */
static int32_t power_offset = 0;      /* Offset from actual power (i.e. charge / discharge) */
static uint8_t cmd_buf[APP_RX_DATA_SIZE];  /* Buffer for stdin commands */
static uint16_t cmd_buf_len = 0;      /* Length of stdin buffer */

static uint8_t comm_count = 0;        /* Number of active comm sessions (I2C / SPI / UART) */

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

static void evse_changed_cb(EVSE_PP pp, uint8_t current);

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

  /* Initialise Red/Green IO expander and clear LEDs */
  if (ioexp_init(IOEXP_RG_LED))
  {
    ioexp_set_output(IOEXP_RG_LED, 0x0000);
    ioexp_set_direction(IOEXP_RG_LED, 0xffff);
  }

  /* Initialise Blue/Yellow IO expander and clear LEDs */
  if (ioexp_init(IOEXP_BY_LED))
  {
    ioexp_set_output(IOEXP_BY_LED, 0x0000);
    ioexp_set_direction(IOEXP_BY_LED, 0xffff);
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
  if (!evse_init(&evse_changed_cb))
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

  if (!error)
    printf("{\"controller\":[{\"status\":0,\"message\":\"Initialized OK\"}]}\n");

  /* Set Maximum DC power. Import / Export will be controlled separately. */
  chademo_set_max_power(SOLAX_MINIMUM_SUPPORTED_VOLTAGE * SOLAX_MAXIMUM_SUPPORTED_CURRENT);

  MX_IWDG_Init();

  /* Infinite loop */
  for(;;)
  {
    /* Catch any errors and EStop */
    if (error)
    {
      emergency_stop();
    }

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
        ioexp_set_direction(IOEXP_RG_LED, ~leds);
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
    int32_t acc_current;
    int32_t hv_current;

    sensor_get_value(SENSOR_ACC_CURRENT, &acc_current);
    sensor_get_value(SENSOR_HV_TEST_CURRENT, &hv_current);
#endif

    printf("{\"controller\":{");
    printf("\"power_offset\":%ld", power_offset);
    printf(",\"timestamp\":%ld", HAL_GetTick());
#ifdef DEBUG_CONTROLLER
    printf(",\"acc_current\":%ld", acc_current / 1000);
    printf(",\"hv_current\":%ld", hv_current);
#endif
    printf(",");
    evse_json_update();
    printf(",");
    solax_json_update();
    printf(",");
    chademo_json_update();
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

  /* Force ChaDeMo contactors off */
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
static void evse_changed_cb(EVSE_PP pp, uint8_t current)
{
  switch (pp)
  {
    case EVSE_PP_INSERTED:
      /* Enable the CP Line */
      /* This tells the EVSE to start charging (supply power) */
      HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_SET);
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
    break;
  }

#ifdef ENABLE_SOLAX
  /* Update the inverter max (BMS / DC handled by ChaDeMo). */
  solax_set_max_ac_current(current);
#endif
}

/**
  * @brief  Process a line of stdin data
  * @param  ptr Pointer to received data
  * @param  len length of data
  * @retval None
  */
static void process_stdin_line(uint8_t *ptr, uint16_t len)
{
  char *tok;

  tok = strtok((char*)ptr, " \r\n");
  if (tok)
  {
    if (0 == strcmp(tok, "chademo"))
    {
      tok = strtok(NULL, " ");
      if (tok)
      {
        if (0 == strcmp(tok, "start"))
        {
          chademo_start();
        }
        else if (0 == strcmp(tok, "stop"))
        {
          power_offset = 0;
          chademo_stop();
        }
      }
    }
    else if (0 == strcmp(tok, "power"))
    {
      tok = strtok(NULL, " ");
      if (tok)
      {
        power_offset = strtol(tok, NULL, 10);
      }
    }
    else if (0 == strcmp(tok, "evse"))
    {
      tok = strtok(NULL, " ");
      if (tok)
      {
        switch (strtol(tok, NULL, 10))
        {
          case 0:
            /* Disable the CP line */
            HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_RESET);
          break;

          case 1:
            /* Enable the CP line */
            HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_SET);
          break;

          default:
          break;
        }
      }
    }
    else if (0 == strcmp(tok, "reset"))
    {
      HAL_NVIC_SystemReset();
    }
  }
}

/**
  * @brief  Process data received on stdin
  * @param  ptr Pointer to received data
  * @param  len length of data
  * @retval None
  */
void stdio_parser(uint8_t *ptr, uint16_t len)
{
  uint32_t i;

  if (len < (APP_RX_DATA_SIZE - cmd_buf_len))
  {
    uint8_t *c = &cmd_buf[cmd_buf_len];

    memcpy(&cmd_buf[cmd_buf_len], ptr, len);
    cmd_buf_len += len;

    for (i=0; i<len; ++i, ++c)
    {
      if (*c == '\n' || *c == '\r')
      {
        uint32_t offset = cmd_buf_len - len + i;

        /* Ensure we're null terminated */
        cmd_buf[offset] = 0;

        /* Process this line */
        process_stdin_line(&cmd_buf[0], offset);
        cmd_buf_len = 0;
        memset(cmd_buf, 0, APP_RX_DATA_SIZE);
      }
    }
  }
  else
  {
    /* Buffer overflow */
    cmd_buf_len = 0;
  }

  /* Fast response for when hacking around */
  if (len == 1)
  {
    switch (ptr[0])
    {
      case '1':
        chademo_start();
      break;

      case '2':
        power_offset = 0;
        chademo_stop();
      break;

      case '3':
        HAL_NVIC_SystemReset();
      break;

      case '4':
        JumpToBootloader();
      break;

      case '+':
        power_offset += 100;
      break;

      case '-':
        power_offset -= 100;
      break;

      default:
      break;
    }
  }
}

/* USER CODE END Application */

