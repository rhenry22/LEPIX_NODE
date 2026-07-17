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
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "fatfs.h"
#include "lwip.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include "eeprma2_m24.h"

#include <stdio.h>

#include "artnet.h"
#include "ws2815.h"
#include "tim.h"
#include "st7789.h"

#include "st7789_test.h"

/* ← Ajouter ces trois lignes */
#include "config.h"
#include "icon_loader.h"
#include "menu.h"
#include "web_ui.h"
#include "mode_select.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Use USB Device CDC instead of USART1 for debug output */
/* #define USB_DEBUG */

/* Enable the USB Host stack (USB Disk) */
/* #define ENABLE_USBHOST */

/* Enable the LWIP Ethernet Stack */
/* #define ENABLE_ETHERNET */

/* Test GPIO : génère un toggle continu sur PD15 dans le while(1)
 * pour vérifier la sortie à l'oscilloscope. Désactiver en production. */
/* #define WS2815_GPIO_TEST */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* Enable the ENABLE_SPI_SCREEN*/
#define ENABLE_SPI_SCREEN 
#define ENABLE_ROTARY_ENCODER

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

WS2815_Chain_t all_chains[MAX_OUTPUTS];

/* Sortie i (config) → pin GPIOD (voir tim.h : PD15/PD13/PD11/PD9) */
static const uint16_t ws_output_pins[MAX_OUTPUTS] = {
    GPIO_PIN_15, GPIO_PIN_13, GPIO_PIN_11, GPIO_PIN_9
};

/* Levé par dmx_to_ws2815, consommé dans la boucle principale */
static volatile bool ws_frame_dirty = false;

/* Perte de signal Art-Net : après ARTNET_TIMEOUT_MS sans trame, on émet
 * un flash blanc bref sur toutes les LEDs toutes les ARTNET_FLASH_PERIOD_MS. */
#define ARTNET_TIMEOUT_MS       60000u   /* 1 minute sans trame        */
#define ARTNET_FLASH_PERIOD_MS  20000u   /* un flash toutes les 20 s   */
#define ARTNET_FLASH_ON_MS        120u   /* durée d'allumage du flash  */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

void MX_EEPRMA2_Check_24C02(void);
void WS2815_Startup_Sequence(void);
static void dmx_to_ws2815(uint16_t universe, uint8_t *data, uint16_t len);
static void artnet_signal_lost_task(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  Mode_Init();   /* jumper PA5/PA6 : mode LED (WS2815) ou DMX/RDM */
  MX_DMA_Init();
  //MX_CAN1_Init();
  //MX_CAN2_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  printf("MX_USART_UART_Init : Done\r\n");
  printf("Mode (jumper PA5/PA6) : %s\r\n", Mode_Name(Mode_Get()));
  /* FATFS + montage SD + config AVANT LwIP :
   * MX_LWIP_Init() lit la config réseau (DHCP / IP statique).
   * opt=0 : montage paresseux (n'accède pas à la carte ici) — le premier
   * accès disque a lieu dans Config_Init(), qui retombe sur les valeurs
   * par défaut si la carte est absente. Ne jamais bloquer le boot ici. */
  MX_FATFS_Init();
  FRESULT sd_res = f_mount(&SDFatFS, SDPath, 0);
  printf("SD mount : %s\r\n", (sd_res == FR_OK) ? "OK" : "FAILED - config par defaut");
  Config_Init();
  printf("Config : Done\r\n");
  MX_LWIP_Init();
  MX_USB_DEVICE_Init();
  printf("MX_USB_DEVICE_Init : Done\r\n");
  MX_CRC_Init();
#ifdef ENABLE_USBHOST
  MX_USB_HOST_Init();
  printf("MX_USB_HOST_Init : Done\r\n");
#endif
  MX_SPI2_Init();
  printf("MX_SPI2_Init : Done\r\n");
/* USER CODE BEGIN 2 */
#ifdef ENABLE_SPI_SCREEN

    // 1 — CS Flash et NRF inactifs AVANT tout accès SPI
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);  // Flash CS HIGH
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);  // NRF CS HIGH
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // LCD CS HIGH
    HAL_Delay(10);

    // 2 — Init écran
    ST7789_Init();
    printf("Screen Init: Done\r\n");

    // 3 — Backlight ON (ne jamais l'éteindre)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    printf("BLK: ON\r\n");
    HAL_Delay(10);

    #ifdef ENABLE_SCREEN_TEST
    ST7789_RunAllTests();
    #endif

#endif

#ifdef ENABLE_ROTARY_ENCODER
    Encoder_Init();
    printf("ROTARY_ENCODER: ON\r\n");

#endif
  Icon_LoadAll();         // loads all 8 icons into RAM cache (~16KB)
  Menu_Init();            // clears screen, shows main menu

  // Clignotement backlight au démarrage = preuve que GPIO fonctionne
  HAL_Delay(500);

  /* WS2815 : 4 sorties sur GPIOD — PD15 / PD13 / PD11 / PD09
   * Envoi par TIM1 + DMA2 (non bloquant), nombre de LEDs depuis la config */
  MX_TIM1_WS2815_Init();
  {
    DeviceConfig_t *cfg = Config_Get();
    for (uint8_t i = 0; i < MAX_OUTPUTS; i++) {
      uint16_t n = cfg->outputs[i].enabled ? cfg->outputs[i].led_count : 0;
      WS2815_Init(&all_chains[i], GPIOD, ws_output_pins[i], n);
    }
  }
  printf("WS2815 Init: Done\r\n");

  printf("\r\nInit preripherals and IO Complete.\r\n");
  printf("Checking Storage Devices:\r\n");
      
  /* Séquence de démarrage visuelle */
  #ifdef ENABLE_STARTUP_SEQUENCE
    printf("ws2815:  NO  WS2815_Startup_Sequence\r\n");
    WS2815_Startup_Sequence();
    printf("WS2815 Startup Sequence: Done\r\n");
  #endif
  artnet_init();
  artnet_set_callback(dmx_to_ws2815);
  printf("Art-Net Initialized\r\n");

  WebUI_Init();   /* serveur HTTP : http://<ip>/ (config + monitoring) */
  printf("Web UI Initialized\r\n");

  uint32_t last_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  
  while (1)
  {
    /* Pompe LwIP (mode raw, pas de FreeRTOS) */
    MX_LWIP_Process();
    /* USER CODE BEGIN WHILE */
    /* Votre init Art-Net se fait UNE FOIS avant le while(1) : */
    /* 2. Heartbeat visuel (clignote toutes les 500ms) */
    if (HAL_GetTick() - last_tick > 500) {
        HAL_GPIO_TogglePin(GPIOE, LED1_Pin); // Utilise ta pin LED définie
        last_tick = HAL_GetTick();
    }
    Menu_Task();            // handles encoder events + redraws when needed
    MX_LWIP_Process();

    /* Envoi WS2815 : dès que des données DMX sont arrivées et que le
     * DMA est libre (latch >280µs incluse dans WS2815_Busy) */
    if (ws_frame_dirty && !WS2815_Busy()) {
        ws_frame_dirty = false;
        WS2815_Show(all_chains, MAX_OUTPUTS);
    }

    /* Fixture "perte de signal" : flash blanc si pas d'Art-Net depuis 1 min */
    artnet_signal_lost_task();
    /* USER CODE END WHILE */

#ifdef ENABLE_USBHOST
    MX_USB_HOST_Process();
#endif
    
  }

  /* Something went wrong */
  NVIC_SystemReset();
  /* USER CODE BEGIN 3 */
  printf("DEBUG : System_reset\r\n");
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

void WS2815_Startup_Sequence(void)
{
    static const WS2815Pixel_t seq[] = {
        WS2815_WHITE, WS2815_GREEN, WS2815_RED, WS2815_BLUE, WS2815_BLACK
    };

    for (uint8_t s = 0; s < sizeof(seq) / sizeof(seq[0]); s++) {
        for (uint8_t i = 0; i < 60; i++) {
            for (uint8_t c = 0; c < MAX_OUTPUTS; c++)
                WS2815_SetLed(&all_chains[c], i, seq[s]);
            while (WS2815_Busy());
            WS2815_Show(all_chains, MAX_OUTPUTS);
            HAL_Delay(25);
        }
        HAL_Delay(500);
    }
    while (WS2815_Busy());
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
    HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_RESET);

#ifdef USB_DEBUG
    static uint8_t rc = USBD_OK;
    bool wait = false;

    /* Wait for terminal to be opened */
    while (!CDC_Is_Connected())
    {
      wait = true;
    }

    /* If the terminal has just opened, give it some extra time */
    if (wait)
    {
      HAL_Delay(250);
    }

    /* Send the data, retrying if busy */
    do {
        rc = CDC_Transmit_FS((uint8_t*)ptr, len);
    } while (USBD_BUSY == rc);

    if (USBD_FAIL == rc) {
        /// NOTE: Should never reach here.
        /// TODO: Handle this error.
        return 0;
    }
#else
    HAL_StatusTypeDef rc;
    do {
      /* Send the data, retrying if busy */
      rc = HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 100);
    } while (rc == HAL_BUSY);
#endif

    HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_SET);

    return len;
}

/* USER CODE BEGIN 4 */

/**
 * @brief Mappe les données DMX Art-Net sur les chaînes WS2815.
 *        Le routage vient de la config : cfg->outputs[i].universe → all_chains[i].
 *        3 canaux DMX consécutifs par LED : R, G, B.
 *        L'envoi réel est fait dans la boucle principale (ws_frame_dirty),
 *        le DMA étant non bloquant il n'y a plus besoin d'attendre le
 *        dernier univers.
 */
static void dmx_to_ws2815(uint16_t universe, uint8_t *data, uint16_t len)
{
    DeviceConfig_t *cfg = Config_Get();

    for (uint8_t i = 0; i < MAX_OUTPUTS; i++) {
        if (!cfg->outputs[i].enabled || cfg->outputs[i].universe != universe)
            continue;

        WS2815_Chain_t *ch = &all_chains[i];
        uint16_t num_leds = len / 3;
        if (num_leds > ch->num_leds)
            num_leds = ch->num_leds;

        for (uint16_t l = 0; l < num_leds; l++) {
            WS2815Pixel_t px = { data[l * 3], data[l * 3 + 1], data[l * 3 + 2] };
            WS2815_SetLed(ch, l, px);
        }
        ws_frame_dirty = true;
    }
}

/**
 * @brief Signale la perte de signal Art-Net.
 *        Si aucune trame n'a été reçue depuis ARTNET_TIMEOUT_MS, émet un
 *        flash blanc bref sur toutes les LEDs toutes les ARTNET_FLASH_PERIOD_MS.
 *        S'arrête dès qu'une trame arrive (le DMX reprend alors la main).
 *        À appeler depuis la boucle principale (non bloquant).
 */
static void artnet_signal_lost_task(void)
{
    static uint32_t last_flash_ms = 0;   /* début du dernier cycle de flash */
    static bool     flash_on      = false;

    DeviceConfig_t *cfg = Config_Get();
    /* La fixture ne concerne que la réception Art-Net */
    if (cfg->protocol != PROTO_ARTNET)
        return;

    WebUI_Stats_t st;
    WebUI_GetStats(&st);
    uint32_t now = HAL_GetTick();

    /* Temps écoulé depuis la dernière trame. Si aucune trame n'a jamais été
     * reçue (packets==0), on mesure depuis le boot (last_ms == 0). */
    uint32_t since = now - st.artnet_last_ms;
    bool signal_lost = (since >= ARTNET_TIMEOUT_MS);

    if (!signal_lost) {
        /* Signal présent : on s'assure de ne pas laisser un flash allumé.
         * (le DMX écrasera de toute façon les LEDs au prochain ws_frame_dirty) */
        flash_on = false;
        return;
    }

    if (!flash_on) {
        /* Attente entre deux flashs */
        if (now - last_flash_ms < ARTNET_FLASH_PERIOD_MS)
            return;
        if (WS2815_Busy())
            return;
        /* Allumage : blanc sur toutes les LEDs des sorties actives */
        for (uint8_t i = 0; i < MAX_OUTPUTS; i++) {
            for (uint16_t l = 0; l < all_chains[i].num_leds; l++)
                WS2815_SetLed(&all_chains[i], l, WS2815_WHITE);
        }
        WS2815_Show(all_chains, MAX_OUTPUTS);
        last_flash_ms = now;
        flash_on      = true;
    } else {
        /* Extinction après ARTNET_FLASH_ON_MS */
        if (now - last_flash_ms < ARTNET_FLASH_ON_MS)
            return;
        if (WS2815_Busy())
            return;
        for (uint8_t i = 0; i < MAX_OUTPUTS; i++) {
            for (uint16_t l = 0; l < all_chains[i].num_leds; l++)
                WS2815_SetLed(&all_chains[i], l, WS2815_BLACK);
        }
        WS2815_Show(all_chains, MAX_OUTPUTS);
        flash_on = false;
    }
}

/* USER CODE END 4 */

/**
  * @brief Check for presence of I2C Flash
  * @retval None
  */
void MX_EEPRMA2_Check_24C02(void)
{
  int32_t ret = EEPRMA2_M24_Init(EEPRMA2_M24C02_0);
  if (ret == BSP_ERROR_NONE)
  {
    ret = EEPRMA2_M24_IsDeviceReady(EEPRMA2_M24C02_0, 10);
    if (ret == BSP_ERROR_NONE)
    {
      printf("I2C 24C02:    256B\r\n");
    }
    else
    {
      printf("I2C 24C02:    Error\r\n");
    }
  }
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
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
