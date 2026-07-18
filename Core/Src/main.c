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
#include "sacn_rx.h"
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
#include "sd_selftest.h"
#include "dmx.h"
#include "gpio_test.h"

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

/* Ecran ST7789 + encodeur rotatif : desactives tant que seule la
 * motherboard est cablee (pas de front panel). Reactiver les deux
 * lignes ci-dessous quand l'ecran/encodeur seront rebranches. */
/* #define ENABLE_SPI_SCREEN */
/* #define ENABLE_ROTARY_ENCODER */
#define ENABLE_STARTUP_SEQUENCE

/* Test carte SD : cree test.txt + log au boot, le supprime 2 min apres.
 * Commenter cette ligne pour desactiver le test. */
#define SD_SELFTEST

/* Serveur web (httpd LwIP) : compile ici, pilote au runtime par le
 * jumper PA6 (Mode_WebEnabled) + presence carte SD. Desactive par defaut
 * cote materiel (aucun jumper PA6). */
#define ENABLE_WEB_UI

/* Test GPIO : lit et affiche au boot l'etat du jumper de mode (PA5/PA6)
 * et des broches DMX (direction PD7/PD10, TX/RX). Diagnostic passif. */
#define GPIO_TEST

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

/* LED3 (PE15) = indicateur "trame recue" (Art-Net/sACN, tous modes).
 * Allumee a chaque trame, eteinte apres RX_LED_ON_MS dans la boucle.
 * Convention carte : niveau bas (RESET) = LED allumee. */
#define RX_LED_ON_MS   40u
static volatile uint32_t rx_led_last_ms = 0;
static volatile bool     rx_led_on      = false;
#define RX_LED_SET_ON()   do { HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); rx_led_on = true; } while (0)
#define RX_LED_SET_OFF()  do { HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);   rx_led_on = false; } while (0)

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
  printf("Mode (jumper PA5) : %s\r\n", Mode_Name(Mode_Get()));
  printf("Web  (jumper PA6) : %s\r\n", Mode_WebEnabled() ? "active" : "desactive (defaut)");
  /* FATFS + montage SD + config AVANT LwIP :
   * MX_LWIP_Init() lit la config réseau (DHCP / IP statique).
   * opt=0 : montage paresseux (n'accède pas à la carte ici) — le premier
   * accès disque a lieu dans Config_Init(), qui retombe sur les valeurs
   * par défaut si la carte est absente. Ne jamais bloquer le boot ici. */
  MX_FATFS_Init();
  FRESULT sd_res = f_mount(&SDFatFS, SDPath, 0);
  printf("SD mount : %s\r\n", (sd_res == FR_OK) ? "OK" : "FAILED - config par defaut");
  Config_Init();
  /* IP par défaut selon le mode (jumper), sauf si un config.json valide
   * l'a déjà fixée : node DMX -> 2.0.0.3, node LED -> 2.0.0.4. */
  if (!Config_IsFromSD()) {
    DeviceConfig_t *cfg = Config_Get();
    cfg->ip[3] = (Mode_Get() == MODE_DMX) ? 3 : 4;
  }
  printf("Config : Done (IP %u.%u.%u.%u)\r\n",
         Config_Get()->ip[0], Config_Get()->ip[1],
         Config_Get()->ip[2], Config_Get()->ip[3]);
#ifdef SD_SELFTEST
  SDTest_Begin();   /* cree test.txt + log ; suppression 2 min plus tard */
#endif
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
#ifdef ENABLE_SPI_SCREEN
  Icon_LoadAll();         // loads all 8 icons into RAM cache (~16KB)
  Menu_Init();            // clears screen, shows main menu
#endif

  /* Aiguillage selon le jumper PA5/PA6 (lu par Mode_Init) :
   *  - MODE_DMX : sortie DMX512/RDM sur XLR (USART2 PD5/6, dir PD7)
   *  - MODE_LED : 4 chaines WS2815 sur GPIOD (PD15/13/11/9), TIM1+DMA2 */
  if (Mode_Get() == MODE_DMX) {
    DMX_Init();
    printf("DMX Init: Done (sortie XLR)\r\n");
  } else {
    printf("WS2815 Init: Start\r\n");
    MX_TIM1_WS2815_Init();
    DeviceConfig_t *cfg = Config_Get();
    for (uint8_t i = 0; i < MAX_OUTPUTS; i++) {
      uint16_t n = cfg->outputs[i].enabled ? cfg->outputs[i].led_count : 0;
      WS2815_Init(&all_chains[i], GPIOD, ws_output_pins[i], n);
    printf("WS2815 Init: OUTPUT %d\r\n", i);
    }
    printf("WS2815 Init: Done\r\n");
  }

#ifdef GPIO_TEST
  GpioTest_Run();   /* log etat jumper + GPIO DMX */
#endif

  printf("\r\nInit preripherals and IO Complete.\r\n");
  printf("Checking Storage Devices:\r\n");
      
  /* Séquence de démarrage visuelle */
  #ifdef ENABLE_STARTUP_SEQUENCE
    printf("ws2815:  NO  WS2815_Startup_Sequence\r\n");
    WS2815_Startup_Sequence();
    printf("WS2815 Startup Sequence: Done\r\n");
  #endif
  printf("Art-Net Configuration ...\r\n");
  artnet_init();
  artnet_set_callback(dmx_to_ws2815);
  printf("Art-Net Initialized\r\n");

  /* sACN (E1.31) : même callback que l'Art-Net (routage par univers).
   * On rejoint les groupes multicast des univers configurés. */
  printf("sACN (E1.31) Configuration ...\r\n");
  sacn_rx_init();
  sacn_rx_set_callback(dmx_to_ws2815);
  {
    DeviceConfig_t *cfg = Config_Get();
    for (uint8_t i = 0; i < MAX_OUTPUTS; i++)
      if (cfg->outputs[i].enabled)
        sacn_rx_join_universe(cfg->outputs[i].universe);
  }
  printf("sACN Initialized\r\n");

  /* Serveur web chargé si : compile (ENABLE_WEB_UI) + jumper PA6 pose
   * + carte SD presente (montee). */
#ifndef ENABLE_WEB_UI
  printf("Web UI : desactive (ENABLE_WEB_UI commente)\r\n");
#else
  if (!Mode_WebEnabled()) {
    printf("Web UI : desactive (jumper PA6 ouvert)\r\n");
  } else if (sd_res != FR_OK) {
    printf("Web UI : desactive (carte SD absente)\r\n");
  } else {
    WebUI_Init();   /* serveur HTTP : http://<ip>/ (config + monitoring) */
    printf("Web UI Initialized (jumper PA6)\r\n");
  }
#endif /* ENABLE_WEB_UI */

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
#ifdef ENABLE_SPI_SCREEN
    Menu_Task();            // handles encoder events + redraws when needed
#endif
    MX_LWIP_Process();

    if (Mode_Get() == MODE_DMX) {
        /* Rafraîchissement DMX512 à 40 Hz (flux continu vers le XLR) */
        DMX_Task();
    } else {
        /* Envoi WS2815 : dès que des données DMX sont arrivées et que le
         * DMA est libre (latch >280µs incluse dans WS2815_Busy) */
        if (ws_frame_dirty && !WS2815_Busy()) {
            ws_frame_dirty = false;
            WS2815_Show(all_chains, MAX_OUTPUTS);
        }

        /* Fixture "perte de signal" : flash blanc si pas d'Art-Net depuis 1 min */
        artnet_signal_lost_task();
    }

    /* Extinction de LED3 (indicateur RX) apres l'impulsion */
    if (rx_led_on && (HAL_GetTick() - rx_led_last_ms) >= RX_LED_ON_MS) {
        RX_LED_SET_OFF();
    }

#ifdef SD_SELFTEST
    SDTest_Task();   /* supprime test.txt une fois les 2 min ecoulees */
#endif
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
        printf("WS2815 Startup Sequence: Step %d\r\n", s);
        for (uint8_t i = 0; i < 60; i++) {
            for (uint8_t c = 0; c < MAX_OUTPUTS; c++) {
                WS2815_SetLed(&all_chains[c], i, seq[s]);
            }
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
 * @brief Route une trame Art-Net reçue selon le mode courant.
 *        - MODE_DMX : l'univers configuré (outputs[0].universe) sort sur
 *          le port XLR via la couche DMX512.
 *        - MODE_LED : mappe les données sur les chaînes WS2815 (config
 *          par sortie), 3 canaux DMX par LED (R,G,B). Envoi réel dans la
 *          boucle principale (ws_frame_dirty).
 */
static void dmx_to_ws2815(uint16_t universe, uint8_t *data, uint16_t len)
{
    DeviceConfig_t *cfg = Config_Get();

    /* Indicateur visuel : LED3 s'allume a chaque trame recue (tous modes,
     * Art-Net ou sACN). Extinction geree dans la boucle principale. */
    RX_LED_SET_ON();
    rx_led_last_ms = HAL_GetTick();

    /* Instantané pour la matrice de canaux de l'interface web (/dmx) */
    WebUI_NotifyDmxData(universe, data, len);

    if (Mode_Get() == MODE_DMX) {
        /* 2 ports DMX : port i suit l'univers de outputs[i] (i=0,1). */
        for (uint8_t port = 0; port < DMX_NUM_PORTS; port++) {
            if (cfg->outputs[port].enabled &&
                cfg->outputs[port].universe == universe) {
                uint16_t n = (len > DMX_SLOTS) ? DMX_SLOTS : len;
                DMX_SetSlots(port, 0, data, n);
                DMX_Commit(port);
            }
        }
        return;
    }

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
