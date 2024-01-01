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

#define JSON_UPDATE_TIME    (10000)

#define MB_SLAVE_ADDRESS	  (1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static bool error = false;
static uint32_t last_json_update = 0;  /* Last time we saw frame 0x03 */
static float power_offset = -1000;

static uint32_t loop_time_max = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void mb_read_cb(MB_FUNC type, uint16_t reg, uint16_t len);

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
  __disable_irq();

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
      chademo_stop();
    break;
  }

#ifdef ENABLE_SOLAX
    /* Update the inverter max. */
    solax_set_max_ac_current(current);
#endif
}
#endif

/**
  * @brief Modbus Read Callback
  * @param type Modbus function type
  * @param reg Register to read
  * @param len Number of registers to read
  * @retval None
  */
void mb_read_cb(MB_FUNC type, uint16_t reg, uint16_t len)
{
  //HAL_GPIO_WritePin(LED_GPIO_Port, INVERTER_Pin, GPIO_PIN_RESET);
	switch (type)
	{
		case MB_READ_HOLDING:
    case MB_READ_INPUT:
			switch (reg)
			{
				case 12:	// Current Power (W)
					modbus_resp_begin(type, sizeof(int32_t));
					modbus_resp_float((float)chademo_get_power() + power_offset);
                    modbus_resp_end();
				break;


				case 0x2004:	// Current Power (KW)
					modbus_resp_begin(type, sizeof(int32_t));
					modbus_resp_float(((float)chademo_get_power() + power_offset) / 1000.0f);
                    modbus_resp_end();
				break;

				case 70: 	// Frequency (Hz)
				case 0x200E: 	// Frequency (Hz)
					modbus_resp_begin(type, sizeof(int32_t));
					modbus_resp_float(50.0f);
                    modbus_resp_end();
				break;

				default:
					modbus_resp_begin(0x80 | type, MB_ERR_ILLEGAL_ADDRESS);
                    modbus_resp_end();
				break;
			}
		break;

		default:
			modbus_resp_begin(0x80 | type, MB_ERR_ILLEGAL_FUNCTION);
            modbus_resp_end();
		break;
	}

  //HAL_GPIO_WritePin(LED_GPIO_Port, INVERTER_Pin, GPIO_PIN_SET);
}

void stdio_parser(uint8_t *ptr, uint32_t len)
{
 if (len == 1)
  {
    switch (ptr[0])
    {
      case '1':
        chademo_start();
      break;

      case '2':
        chademo_stop();
      break;

      case '3':
        HAL_NVIC_SystemReset();
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

  HAL_CAN_DeInit(&hcan1);
  HAL_CAN_DeInit(&hcan2);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* Power Up ESP8266 */
  HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);
  
  /* Turn on the EVSE */
  HAL_GPIO_WritePin(EVSE_CHARGE_EN_GPIO_Port, EVSE_CHARGE_EN_Pin, GPIO_PIN_SET);
  
  HAL_Delay(2000);

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
  MX_CAN_Setup_Receive(&hcan1, CAN_FILTER_FIFO0);
  if (!chademo_init())
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise ChaDeMo interface.\"}]\n");
    error = true;
  }
#endif

#ifdef ENABLE_SOLAX
  MX_CAN_Setup_Receive(&hcan2, CAN_FILTER_FIFO1);
  solax_init();
#endif

  if (!modbus_init(MB_SLAVE_ADDRESS, &mb_read_cb))
  {
    printf("{\"controller\":[{\"status\":-1,\"message\":\"Failed to initialise Modbus interface.\"}]\n");
    error = true;
  }

  if (!error)
    printf("{\"controller\":[{\"status\":0,\"message\":\"Initialized OK\"}]}\n");

  /* Set Maximum DC power. Import / Export will be controlled separately. */
  chademo_set_max_power(SOLAX_MINIMUM_SUPPORTED_VOLTAGE * SOLAX_MAXIMUM_SUPPORTED_CURRENT);
  
  // ToDo: Drive this from the ESP8266
  //chademo_start();

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

    /* Send regular JSON messages */
    if ((last_json_update == 0) || 
        (HAL_GetTick() > last_json_update + JSON_UPDATE_TIME))
    {
      int32_t acc_current;

      sensor_get_value(SENSOR_ACC_CURRENT, &acc_current);

      printf("{\"controller\":{");
      printf("\"timestamp\":%ld", HAL_GetTick());
      printf(",\"loop_time_max\":%ld", loop_time_max);
      printf(",\"acc_current\":%ld", acc_current / 1000);
      printf(",");
      evse_json_update();
      printf(",");
      solax_json_update();
      printf(",");
      chademo_json_update();
      printf("}}\n");

      last_json_update = HAL_GetTick();
    }

    /* Kick the Watchdog */
    HAL_IWDG_Refresh(&hiwdg);

    loop_time = HAL_GetTick() - loop_time;
    if (loop_time > loop_time_max) loop_time_max = loop_time;

    /* Re-purpose the EVSE LED to show when we're busy */
    HAL_GPIO_WritePin(LED3_GPIO_Port, EVSE_Pin, GPIO_PIN_SET);
    __WFI();
    HAL_GPIO_WritePin(LED3_GPIO_Port, EVSE_Pin, GPIO_PIN_RESET);

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
