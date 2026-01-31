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
#define HIGH_VOLTAGE_TIMEOUT     (60000) /* Allow the HV source to be left on for this duration (max) */

#define BUTTON_DEBOUNCE_TIME     (150)
/* How often to toggle flashing LEDs (ms) */
#define FLASH_TOGGLE_TIME        (500)

#define HV_PWM_MIN               (200)
#define HV_PWM_MAX               (5600)
#define HV_PWM_DEFAULT           (HV_PWM_MAX)
#define HV_HYST_VOLT             (5)  /* Hysteresis for HV voltage control (V) */

#define HV_DCDC_C                (9800) /* Minimum current draw from HV DCDC in uA */
#define HV_DCDC_M                (110) /* DC-DC Efficiency */
#define HV_DCDC_N                (11) /* Voltage dependent loss */
#define HV_DCDC_O                (-300) /* R Offset */

/* PID tuning for HV generator (integer fixed-point) */
#define HV_PID_SCALE            (1000)
#define HV_PID_SAMPLE_MS        (100)

#define HV_PID_KP_SCALED        (-6000)
#define HV_PID_KI_SCALED        (-5)
#define HV_PID_KD_SCALED        (-800)

#define HV_PID_INTEGRAL_MAX     (100)  /* Anti-windup clamp */
#define HV_PID_MAX_DELTA        (1500)   /* Max change per sample period */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int32_t power_offset = 0;      /* Offset from actual power (i.e. charge / discharge) */

uint32_t hv_time = 0;          /* When the HV source was enabled */
uint32_t hv_target = 0;        /* Target HV voltage in V */
uint32_t hv_iso_resistance = -1; /* Measured HV isolation resistance in kOhms */

uint16_t debug_leds = 0;       /* Combined state of debug leds */
uint16_t flash_debug_leds = 0; /* Bits for debug LEDs that should flash */
uint8_t flash_user_mask = 0;   /* Bits (0x01,0x02) for user GPIOs that should flash */
uint8_t user_led_base = 0;     /* Base values for user LEDs (bit0 -> GPIO1, bit1 -> GPIO2) */
bool flash_state = false;      /* Current on/off state for flashed LEDs */


static uint32_t last_flash_toggle = 0;/* Last tick when flash_state toggled */

static bool error = false;            /* Whether we are in the error state */

static uint32_t hv_pwm = HV_PWM_DEFAULT;  /* Current value for HV PWM */
/* PID controller state for HV generator */
static int32_t hv_pid_integral = 0; /* accumulated error (samples * volts) */
static int32_t hv_pid_prev_error = 0; /* previous error (volts) */
static uint32_t hv_pid_last_time = 0;

/* Key used to enter ESP special bridge mode on reboot (disable WDT) */
uint32_t esp_prog_key __attribute__((section(".noinit")));
bool esp_flash_mode = false;

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
/* Definitions for jsonTask */
osThreadId_t hvGenTaskHandle;
const osThreadAttr_t hvGenTask_attributes = {
  .name = "hvGenTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void pp_changed_cb(EVSE_PP pp, uint8_t current);
#ifdef TARGET_CCS2
static void cp_changed_cb(EVSE_CP cp);
#endif

/* USER CODE END FunctionPrototypes */

void mainTaskEntry(void *argument);
void jsonTaskEntry(void *argument);
void hvGenTaskEntry(void *argument);
void bridgeTaskEntry(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
    __WFI();
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

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
  {
    if (esp_prog_key == ESP_MODE_KEY)
    {
      esp_prog_key = 0;
      esp_flash_mode = true;

      /* Create ESP Serial Bridge Tasks */
      mainTaskHandle = osThreadNew(bridgeTaskEntry, NULL, &mainTask_attributes);

      return;
    }
  }

  /* creation of mainTask */
  mainTaskHandle = osThreadNew(mainTaskEntry, NULL, &mainTask_attributes);

  /* creation of jsonTask */
  jsonTaskHandle = osThreadNew(jsonTaskEntry, NULL, &jsonTask_attributes);

  hvGenTaskHandle = osThreadNew(hvGenTaskEntry, NULL, &hvGenTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_mainTaskEntry */
/**
  * @brief  Function implementing the bridgeTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_mainTaskEntry */
void bridgeTaskEntry(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN mainTaskEntry */

  osDelay(3000);

  /* Power Up ESP8266 */
  HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);

  /* Initialise Top IO expander and clear LEDs */
  if (ioexp_init(IOEXP_TOP_LEDS))
  {
    ioexp_set_direction(IOEXP_TOP_LEDS, 0xAAAA);
    ioexp_set_output(IOEXP_TOP_LEDS, 0x0000);
  }

  while (1)
  {
    osDelay(100);
  }
}

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

  /* Initialise Sensors and Peripherals */

  if (!sensor_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise sensors.\"}]\n");
    error = true;
  }

#ifdef ENABLE_EVSE
#ifdef TARGET_CCS2
  if (!evse_init(&pp_changed_cb, &cp_changed_cb))
#else
  if (!evse_init(&pp_changed_cb))
#endif
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise EVSE interface.\"}]\n");
    error = true;
  }
#ifdef TARGET_CCS2
  else
  {
    evse_set_cp(100);
  }
#endif
#endif

#ifdef TARGET_CHADEMO
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

  if (!cmd_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise command parser.\"}]\n");
    error = true;
  }

  if (!error)
  {
    printf("{\"controller\":[{\"status\":0,\"message\":\"Initialized OK\"}]}\n");
#ifdef TARGET_CHADEMO
  /* Set Maximum DC power. Import / Export will be controlled separately. */
  chademo_set_max_power(SOLAX_MINIMUM_SUPPORTED_VOLTAGE * SOLAX_MAXIMUM_SUPPORTED_CURRENT);
#endif
  }
  else
  {
    printf("{\"controller\":[{\"info\":\"Entering FLASH mode\"}]}\n");
    JumpToBootloader();
  }

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

#ifdef TARGET_CHADEMO
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
      if (HAL_GPIO_ReadPin(GPIO3_GPIO_Port, GPIO3_Pin) == GPIO_PIN_RESET)
      {
        if ((button_time == 0 || (HAL_GetTick() - button_time) > BUTTON_DEBOUNCE_TIME))
        {
          printf("{\"controller\":[{\"button\":1}]}\n");
        }
        button_time = HAL_GetTick();

#ifdef TARGET_CHADEMO
        chademo_stop();
#endif
      }
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

    int32_t acc_current;
    int32_t acc_voltage;
    int32_t hv_current;
    int32_t batt_voltage;
    int32_t inv_voltage;
    int32_t inv_current;
    uint32_t err = 0;

    if (HAL_OK != sensor_get_value(SENSOR_ACC_CURRENT, &acc_current))
      err |= 1 << SENSOR_ACC_CURRENT;

    if (HAL_OK != sensor_get_value(SENSOR_ACC_VOLTAGE, &acc_voltage))
      err |= 1 << SENSOR_ACC_VOLTAGE;

    if (HAL_OK != sensor_get_value(SENSOR_HV_TEST_CURRENT, &hv_current))
      err |= 1 << SENSOR_HV_TEST_CURRENT;

    if (HAL_OK != sensor_get_value(SENSOR_BATT_VOLTAGE, &batt_voltage))
      err |= 1 << SENSOR_BATT_VOLTAGE;
    if (HAL_OK != sensor_get_value(SENSOR_INV_VOLTAGE, &inv_voltage))
      err |= 1 << SENSOR_INV_VOLTAGE;
    if (HAL_OK != sensor_get_value(SENSOR_INV_CURRENT, &inv_current))
      err |= 1 << SENSOR_INV_CURRENT;

    /* Display Batt and Inverter Voltages on LEDs (200-500v) */
    {
      int32_t b;
      int32_t i;
      b = (batt_voltage - 2000) * 8 / 3000;
      if (b <= 0) b = 0;
      if (b > 7) b = 7;
      if (b > 0)
        b = ((1 << b) - 1) & 0xff;

      i = (inv_voltage - 2000) * 8 / 3000;
      if (i <= 0) i = 0;
      if (i > 7) i = 7;
      if (i > 0)
        i = ((1 << i) - 1) & 0xff;
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

    printf(",\"sensors\":{");
    printf("\"status\":%ld", err);
    printf(",\"acc\":{\"v\":%ld, \"i\":%ld}", acc_voltage, acc_current);
    printf(",\"hv_iso\":{\"i\":%ld, \"r\":%ld}", hv_current, hv_iso_resistance);
    printf(",\"battery\":{\"v\":%ld}", batt_voltage / 10);
    printf(",\"inverter\":{\"v\":%ld, \"i\":%ld}", inv_voltage / 10, inv_current / 10);

    printf("}\n{");
    evse_json_update();

    printf("}\n{");
    solax_json_update();
#ifdef TARGET_CHADEMO

    printf("}\n{");
    chademo_json_update();
#endif
    printf("}\n");
  }
  /* USER CODE END jsonTaskEntry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void hvGenTaskEntry(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    /* Check HV Source Timeout */
    if (hv_time != 0 && HAL_GetTick() - hv_time > HIGH_VOLTAGE_TIMEOUT)
    {
      app_process_cmd_hv((char*[]){"iso", "0"}, 2);
      hv_time = 0;
      printf("{\"controller\":[{\"hv_timeout\":1}]}\n");
    }

    /* HV Generator PWM - We only have feedback in ISO test mode */
    if (hv_target != 0 && hv_time != 0)
    {
      int32_t batt_voltage = -1;
      int32_t hv_current = -1;

      sensor_get_value(SENSOR_BATT_VOLTAGE, &batt_voltage);
      sensor_get_value(SENSOR_HV_TEST_CURRENT, &hv_current);

      /* Calcuations based on dV, not V */
      if (hv_current <= HV_DCDC_C || batt_voltage < 1000)
        hv_iso_resistance = -1;
      else
      {
        uint32_t p1 = 5 * (hv_current - HV_DCDC_C);
        uint32_t p2 = HV_DCDC_M * p1 / 100 - HV_DCDC_N * p1 / 100000 * batt_voltage;
        hv_iso_resistance = batt_voltage * batt_voltage * 10 / p2 - HV_DCDC_O;
      }

      /* Convert sensor value to volts (same as previous code) */
      batt_voltage = batt_voltage / 10;


      /* PID controller (integer fixed-point)
         We assume a nominal sample time of HV_PID_SAMPLE_MS (100 ms). The scaled gains
         are defined above as HV_PID_K*_SCALED. Calculation uses 64-bit intermediates.
      */
      uint32_t now = HAL_GetTick();
      int32_t dt_ms = (hv_pid_last_time == 0) ? HV_PID_SAMPLE_MS : (int32_t)(now - hv_pid_last_time);
      if (dt_ms < 1) dt_ms = 1;
      if (dt_ms > HV_PID_SAMPLE_MS * 2) dt_ms = HV_PID_SAMPLE_MS; /* clamp unreasonable dt */

      /* Error = target - measured (volts) */
      int32_t error = (int32_t)hv_target - (int32_t)batt_voltage;

      /* Integrate (accumulate error scaled by dt) and clamp to avoid windup */
      hv_pid_integral += (error * dt_ms) / HV_PID_SAMPLE_MS;

      /* Anti-windup: clamp integral to reasonable bounds */
      if (hv_pid_integral > HV_PID_INTEGRAL_MAX) hv_pid_integral = HV_PID_INTEGRAL_MAX;
      if (hv_pid_integral < -HV_PID_INTEGRAL_MAX) hv_pid_integral = -HV_PID_INTEGRAL_MAX;

      /* Adjust Ki and Kd for actual dt (integer math) */
      int32_t ki_adj = (int32_t)(((int64_t)HV_PID_KI_SCALED * dt_ms) / HV_PID_SAMPLE_MS);
      int32_t kd_adj = (int32_t)(((int64_t)-HV_PID_KD_SCALED * HV_PID_SAMPLE_MS) / dt_ms);

      /* Compute P, I, D terms using 64-bit intermediates then scale down */
      int64_t p_term = (int64_t)HV_PID_KP_SCALED * (int64_t)error;
      int64_t i_term = (int64_t)ki_adj * (int64_t)hv_pid_integral;
      int64_t d_term = (int64_t)kd_adj * (int64_t)(error - hv_pid_prev_error);

      int64_t pid_sum = p_term + i_term + d_term;
      int32_t pid_out = (int32_t)(pid_sum / HV_PID_SCALE);

      /* Limit change size */
      if (pid_out > HV_PID_MAX_DELTA) pid_out = HV_PID_MAX_DELTA;
      if (pid_out < -HV_PID_MAX_DELTA) pid_out = -HV_PID_MAX_DELTA;

      /* Apply to hv_pwm and clamp */
      int32_t new_pwm = (int32_t)((int32_t)hv_pwm + pid_out);
      if (new_pwm > HV_PWM_MAX) new_pwm = HV_PWM_MAX;
      if (new_pwm < HV_PWM_MIN) new_pwm = HV_PWM_MIN;
      hv_pwm = (uint32_t)new_pwm;

      /* Save state */
      hv_pid_prev_error = error;
      hv_pid_last_time = now;

      /* Update timer compare (timer uses 0..HV_PWM_MAX scale) */
      uint32_t ccr = hv_pwm * (htim1.Init.Period + 1) / HV_PWM_MAX;
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);

      /* Telemetry: print PID state */
      //printf("{\"hv_pid\":{\"error\":%ld,\"p_term\":%ld,\"i_term\":%ld,\"d_term\":%ld,\"pid_out\":%ld,\"hv_pwm\":%lu,\"batt_voltage\":%ld,\"hv_target\":%lu}}\n",
      //        (long)error, (long)p_term, (long)i_term, (long)d_term, (long)pid_out, (unsigned long)hv_pwm, (long)batt_voltage, (unsigned long)hv_target);
    }
    else
    {
      hv_iso_resistance = -1;
      hv_pwm = HV_PWM_DEFAULT;
      /* Clear PID state when generator is not active so integrator doesn't accumulate */
      hv_pid_integral = 0;
      hv_pid_prev_error = 0;
      hv_pid_last_time = 0;
    }

    osDelay(HV_PID_SAMPLE_MS);
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
#ifdef TARGET_CHADEMO
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

#ifdef TARGET_CCS2
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

    case EVSE_CP_ERROR:
      solax_set_output_power(0);
      solax_disable();
      debug_leds |= (1 << DBG_LED_CP_READY);
      debug_leds |= (1 << DBG_LED_CP_CHARGE);
      flash_debug_leds |= (1 << DBG_LED_CP_READY);
      flash_debug_leds |= (1 << DBG_LED_CP_CHARGE);
    break;
  }

  trigger_json_update();
}
#endif

/* USER CODE END Application */

