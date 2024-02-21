/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdbool.h>

#include "sensor.h"

#include "evse.h"
#include "chademo.h"
#include "solax.h"

#include "modbus.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ENABLE_EVSE
#define ENABLE_CHADEMO
#define ENABLE_SOLAX

#define DEBUG_CONTROLLER

#define JSON_UPDATE_TIME    (1000)
#define MB_SLAVE_METER      (1)

#define MB_SLAVE_INVERTER   (247)

enum
{
  FOX_BATT_V = 11006,     // (V x10)
  FOX_BATT_I = 11007,     // (A x10)
  FOX_BATT_P = 11008,     // (W)

  FOX_GRID_V = 11009,     // Grid Voltage (V x10)
  FOX_GRID_I = 11010,     // Grid Current (A x10)
  FOX_GRID_P1 = 11011,    // Grid Phase R Power (W)
  FOX_GRID_P2 = 11012,    // Grid Phase Q Power (W)
  FOX_GRID_P3 = 11013,    // Grid Phase S Power (W)

  FOX_INV_STATE = 11056,  // Inverter Status
  FOX_BATT_STATE = 11057, // Battery Status

  FOX_FAULT_1 = 11061,
  FOX_FAULT_2 = 11062,
  FOX_FAULT_3 = 11063,
  FOX_FAULT_4 = 11064,
  FOX_FAULT_5 = 11065,
  FOX_FAULT_6 = 11066,
  FOX_FAULT_7 = 11067,
  FOX_FAULT_8 = 11068,
  
  FOX_REM_EN = 44000,
  FOX_REM_TIMER = 44001,
  FOX_REM_POWER = 44002
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static bool error = false;            /* Whether we are in the error state */
static uint32_t last_json_update = 0; /* Last time we saw frame 0x03 */
static uint32_t req_inv_power = 0;    /* Time read inverter power requested */
static int32_t power_offset = 0;      /* Offset from actual power (i.e. charge / discharge) */
static int32_t inv_power = 0;         /* Power reported by inverter */
static uint32_t loop_time_max = 0;    /* Maximum loop time observed */
static uint8_t cmd_buf[APP_RX_DATA_SIZE];  /* Buffer for stdin commands */
static uint16_t cmd_buf_len = 0;      /* Length of stdin buffer */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Forcibly shut everything down
  * @retval None
  */
void emergency_stop(void)
{
  volatile uint32_t i;

  /* Stop Everything in the system */
  //__disable_irq();

  /* Turn Off EVSE */
  HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_RESET);

  /* Force ChaDeMo contactors off */
  HAL_GPIO_WritePin(CHADEMO_SEQ2_GPIO_Port, CHADEMO_SEQ2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CHADEMO_SEQ1_GPIO_Port, CHADEMO_SEQ1_Pin, GPIO_PIN_RESET);

  /* Force Leak Test Off */
  HAL_GPIO_WritePin(LEAK_TEST_EN_GPIO_Port, LEAK_TEST_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TEST_HV_EN_GPIO_Port, TEST_HV_EN_Pin, GPIO_PIN_RESET);

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
  * @brief  Trigger an update of the JSON output.
  * @retval None
  */
void trigger_json_update(void)
{
  last_json_update = 0;
}

#ifdef ENABLE_EVSE
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
#endif

/**
  * @brief Modbus Slave Read Callback
  * @param addr Modbus slave address
  * @param type Modbus function type
  * @param reg Register to read
  * @param len Number of registers to read
  * @retval None
  */
void mb_slave_read_cb(uint8_t addr, MB_FUNC type, uint8_t *data, uint16_t len)
{
  if (addr == MB_SLAVE_METER && len == 4)
  {
    uint16_t reg = data[0] << 8 | data[1];

    switch (type)
    {
      case MB_READ_HOLDING:
      case MB_READ_INPUT:
        switch (reg)
        {
          case 12:  // Current Power (W)
            modbus_tx_begin(MB_SLAVE_METER, type, sizeof(int32_t));
            modbus_tx_float((float)chademo_get_power() + power_offset);
            modbus_tx_end();
          break;

          case 0x2004:  // Current Power (KW)
            modbus_tx_begin(MB_SLAVE_METER, type, sizeof(int32_t));
            modbus_tx_float(((float)chademo_get_power() + power_offset) / 1000.0f);
            modbus_tx_end();
          break;

          case 70:   // Frequency (Hz)
          case 0x200E:   // Frequency (Hz)
            modbus_tx_begin(MB_SLAVE_METER, type, sizeof(int32_t));
            modbus_tx_float(50.0f);
            modbus_tx_end();
          break;

          default:
            modbus_tx_begin(MB_SLAVE_METER, 0x80 | type, MB_ERR_ILLEGAL_ADDRESS);
            modbus_tx_end();
          break;
        }
      break;

      default:
        modbus_tx_begin(MB_SLAVE_METER, 0x80 | type, MB_ERR_ILLEGAL_FUNCTION);
        modbus_tx_end();
      break;
    }
  }
}

/**
  * @brief Modbus Master Read Callback (Data back from Slave)
  * @param addr Modbus slave address
  * @param type Modbus function type
  * @param reg Register that was resuested
  * @param data Pointer to raw data
  * @param len Length (in bytes) of raw data
  * @retval None
  */
void mb_master_read_cb(uint8_t addr, MB_FUNC type, uint16_t reg, uint8_t *data, uint16_t len)
{
  if (addr == MB_SLAVE_INVERTER)
  {
    /* Process Inverter's response to our request */
    switch (reg)
    {
      case FOX_GRID_P1:
        if (data[0] == 2)
        {
          inv_power = data[1] << 8 | data[2];
        }
      break;
    }
  }
}

/**
  * @brief Modbus Write Complete Callback
  * @retval None
  */
void mb_write_cb(void)
{
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
    else if (0 == strcmp(tok, "reset"))
    {
      HAL_NVIC_SystemReset();
    }
    else if (0 == strcmp(tok, "modbus"))
    {
      tok = strtok(NULL, " ");
      if (tok)
      {
        if (0 == strcmp(tok, "read"))
        {
          tok = strtok(NULL, " ");
          if (tok)
          {
          }
        }
      }
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* Turn on the EVSE */
  HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_SET);

  /* Power Up ESP8266 */
  HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(3000);

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

  if (!modbus_init(&mb_master_read_cb, &mb_slave_read_cb, &mb_write_cb))
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise Modbus interface.\"}]\n");
    error = true;
  }

  if (!error)
    printf("{\"controller\":[{\"status\":0,\"message\":\"Initialized OK\"}]}\n");

  /* Set Maximum DC power. Import / Export will be controlled separately. */
  chademo_set_max_power(SOLAX_MINIMUM_SUPPORTED_VOLTAGE * SOLAX_MAXIMUM_SUPPORTED_CURRENT);

  /* USER CODE END 2 */

  MX_IWDG_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t loop_time = HAL_GetTick();

    /* Catch any errors and EStop */
    if (error)
    {
      emergency_stop();
    }

#ifdef ENABLE_EVSE
    evse_process();
#endif

#ifdef ENABLE_CHADEMO
    /* Process any ChaDeMo work */
    chademo_process();
#endif

#ifdef ENABLE_SOLAX
    /* Process any Inverter work */
    solax_process();
#endif

    /* Process Serial Data */
    HAL_UART_Process();

    if (HAL_GetTick() > req_inv_power + 100)
    {
      req_inv_power = HAL_GetTick();
      modbus_read(MB_SLAVE_INVERTER, MB_READ_INPUT, FOX_GRID_P1);
    }

    /* Send regular JSON messages */
    if ((last_json_update == 0) ||
        (HAL_GetTick() > last_json_update + JSON_UPDATE_TIME))
    {
      int32_t acc_current;

      sensor_get_value(SENSOR_ACC_CURRENT, &acc_current);

      printf("{\"controller\":{");
      printf("\"power_offset\":%ld", power_offset);
      printf("\"inverter_power\":%ld", inv_power);
      printf(",\"timestamp\":%ld", HAL_GetTick());
#ifdef DEBUG_CONTROLLER
      printf(",\"loop_time_max\":%ld", loop_time_max);
      printf(",\"acc_current\":%ld", acc_current / 1000);
#endif
      printf(",");
      evse_json_update();
      printf(",");
      solax_json_update();
      printf(",");
      chademo_json_update();
      printf("}}\n");

      last_json_update = HAL_GetTick();
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

    /* Kick the Watchdog */
    HAL_IWDG_Refresh(&hiwdg);

    loop_time = HAL_GetTick() - loop_time;
    if (loop_time > loop_time_max) loop_time_max = loop_time;

    /* Re-purpose the EVSE LED to show when we're busy */
    HAL_GPIO_WritePin(LED_GPIO_Port, EVSE_Pin, GPIO_PIN_SET);
    __WFI();
    HAL_GPIO_WritePin(LED_GPIO_Port, EVSE_Pin, GPIO_PIN_RESET);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  /* Something went wrong */
  NVIC_SystemReset();

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief Print the contents of a packet in HEX
  * @param  data: pointer to data
  * @param  len: length of data
  * @retval None
  */
void dump_packet(uint8_t *data, uint8_t len)
{
  int i;
  for (i=0; i<len; ++i)
    printf("0x%02X, ", data[i]);
  printf("\n");
}

/**
  * @brief Override for printf output
  * @param  file: pointer to the source file name
  * @param  ptr: pointer to data to write
  * @param  len: length data to write in bytes
  * @retval Number of bytes written
  */
int _write(int file, char *ptr, int len)
{
#ifndef ESP_FLASH_MODE
  /* If USB is connected, send to USB */
  if (CDC_Is_Connected())
  {
    /* Send the data */
    CDC_Transmit_FS((uint8_t*)ptr, len, 10);
  }

  /* Send Data to Serial */
  HAL_UART_Write_UART1((uint8_t *)ptr, len, 100);
#endif

  return len;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();

  emergency_stop();
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  printf("{\"controller\":[{\"timestamp\":%ld,\"status\":-1,\"message\":\"Assert Failed Failed file %s on line %ld.\"}]\n", HAL_GetTick(), file, line);

  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
