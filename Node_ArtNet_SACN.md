Intégration écran TFT ST7789 240×280 — LEPIX_NODE STM32F407VET6
Vue d'ensemble
Ce document décrit toutes les modifications apportées au projet LEPIX_NODE (basé sur la carte industrielle STM32F407VET6) pour intégrer un écran TFT SPI 240×280 pixels piloté par le contrôleur ST7789, en utilisant le bus SPI2 hardware via la librairie Floyd-Fish portée sous STM32 HAL.

Matériel utilisé
ComposantRéférenceMicrocontrôleurSTM32F407VET6 (LQFP100, 168 MHz)CarteJZ-STM32F407VET6 Industrial BoardÉcranTFT SPI ST7789 240×280 px 3.3VBus SPISPI2 hardware (21 Mbits/s)

Câblage physique
Broche module ST7789GPIO STM32Label projetRemarqueGNDGND—Masse commune obligatoireVCC3.3V—Alimentation 3.3VSCL (SCK)PB10SPI2_SCKPartagé avec Flash W25Q128 + NRF24L01SDA (MOSI)PC3SPI2_MOSIPartagé avec Flash W25Q128 + NRF24L01RES (RST)PC0LCD_RSTGPIO Output dédiéDCPE6LCD_DCGPIO Output dédiéCSPA4LCD_CSGPIO Output dédié — isole l'écran sur le bus SPI2BLKPC13LCD_BLKGPIO Output dédié — backlight

Important : SCK et MOSI sont partagés avec la Flash SPI et le NRF24L01. L'isolation est assurée par les CS respectifs :

Flash W25Q128 → PE3 (SPI2_FLASH_CS)
NRF24L01 → PE8 (SPI2_NRF_CS)
Écran ST7789 → PA4 (LCD_CS)

Un seul CS doit être actif (LOW) à la fois.


Modifications du fichier .ioc / CubeMX
Les GPIO suivants ont été ajoutés ou modifiés dans Industrial_Board.ioc :
ini# Ajouts LCD
PC0.GPIO_Label=LCD_RST
PC0.Signal=GPIO_Output

PC13.GPIO_Label=LCD_BLK
PC13.Signal=GPIO_Output

PE6.GPIO_Label=LCD_DC
PE6.Signal=GPIO_Output

PA4.GPIO_Label=LCD_CS
PA4.Signal=GPIO_Output

# Anciens pins bit-bang (désormais libres)
# PE1 → LCD_SCK  (bit-bang) → maintenant libre
# PE4 → LCD_MOSI (bit-bang) → maintenant libre

Note : PE1 et PE4, utilisés initialement pour le SPI bit-bang, sont maintenant libres pour d'autres usages.


Modifications Core/Inc/main.h
Ajouter dans la section USER CODE BEGIN Private defines :
c/* USER CODE BEGIN Private defines */

/* LCD ST7789 — CS, DC, RST, BLK */
#define ST7789_CS_Pin        GPIO_PIN_4
#define ST7789_CS_GPIO_Port  GPIOA

#define ST7789_DC_Pin        GPIO_PIN_6
#define ST7789_DC_GPIO_Port  GPIOE

#define ST7789_RST_Pin       GPIO_PIN_0
#define ST7789_RST_GPIO_Port GPIOC

/* LCD_BLK et LCD_RST générés automatiquement par CubeMX */

/* USER CODE END Private defines */

Modifications Core/Src/gpio.c
Ajouter à l'intérieur de MX_GPIO_Init(), avant l'accolade fermante, dans la section USER CODE BEGIN 2 :
c/* USER CODE BEGIN 2 */

/* ── LCD ST7789 : PE6 DC, PA4 CS → OUTPUT ── */
HAL_GPIO_WritePin(GPIOA, LCD_CS_Pin, GPIO_PIN_SET);   // CS inactif au démarrage
HAL_GPIO_WritePin(GPIOE, ST7789_DC_Pin, GPIO_PIN_SET);

GPIO_InitStruct.Pin   = ST7789_DC_Pin;
GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull  = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

GPIO_InitStruct.Pin   = LCD_CS_Pin;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* ── LCD ST7789 : PC0 RST, PC13 BLK → OUTPUT ── */
HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOC, LCD_BLK_Pin, GPIO_PIN_RESET);

GPIO_InitStruct.Pin   = LCD_RST_Pin | LCD_BLK_Pin;
GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull  = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE END 2 */

Attention : retirer PC0 et PC13 des blocs GPIO_MODE_INPUT existants pour éviter qu'ils soient réinitialisés en entrée.


Fichiers ajoutés au projet
Core/Inc/st7789.h
Librairie Floyd-Fish adaptée. Modifications apportées :
ParamètreValeur originaleValeur modifiéeST7789_SPI_PORThspi1hspi2Résolution activeUSING_240X240USING_240X280 (ajouté)ST7789_CS_PORTST7789_CS_GPIO_PortGPIOAST7789_CS_PINST7789_CS_PinGPIO_PIN_4
Bloc résolution ajouté :
c#ifdef USING_240X280
    #define ST7789_WIDTH  240
    #define ST7789_HEIGHT 280
    #define X_SHIFT 0
    #define Y_SHIFT 20   // offset typique des modules 240x280
#endif
Core/Src/st7789.c
Correction du cast de type ligne 188 :
c// Avant
ST7789_WriteData(disp_buf, sizeof(disp_buf));

// Après
ST7789_WriteData((uint8_t*)disp_buf, sizeof(disp_buf));
Core/Inc/st7789_test.h
c#ifndef ST7789_TEST_H
#define ST7789_TEST_H

#include "st7789.h"

void ST7789_RunAllTests(void);
void Test1_Backlight(void);
void Test2_FillRed(void);
void Test3_ColorCycle(void);
void Test4_Checkerboard(void);
void Test5_Gradient(void);
void Test6_Cross(void);
void Test7_Text(void);

#endif
Core/Src/st7789_test.c
Programme de validation avec 7 tests progressifs :
TestFonction Floyd-Fish utiliséeRésultat attenduTest 1HAL_GPIO_WritePin BLKBacklight clignote 3×Test 2ST7789_Fill_Color(RED)Écran rouge uniformeTest 3ST7789_Fill_Color(...)Cycle 8 couleursTest 4ST7789_Fill(x0,y0,x1,y1,c)Damier rouge/bleu 8×10Test 5ST7789_Fill(...) ligne par ligneDégradé vertical vertTest 6ST7789_Fill(...)Croix blanche + 4 coins colorésTest 7ST7789_WriteString(...)Texte "ST7789 OK / SPI2 HAL / 240x280"

Correspondance API — ancien driver vs Floyd-Fish
Ancien driver bit-bangFloyd-Fish HAL SPIST7789_Init()ST7789_Init() (inchangé)ST7789_FillScreen(color)ST7789_Fill_Color(color)ST7789_FillRect(x, y, w, h, c)ST7789_Fill(x0, y0, x1, y1, c)ST7789_DrawPixel(x, y, c)ST7789_DrawPixel(x, y, c) (inchangé)ST7789_DrawPixel_4px(x, y, c)ST7789_DrawPixel_4px(x, y, c) (inchangé)LCD_BLK_HIGH()HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_SET)LCD_BLK_LOW()HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_RESET)

Intégration dans Core/Src/main.c
c/* USER CODE BEGIN Includes */
#include "st7789.h"
#include "st7789_test.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
ST7789_RunAllTests();
/* USER CODE END 2 */

/* USER CODE BEGIN WHILE */
while (1)
{
    // application
}

GPIO libres après intégration
Les GPIO suivants restent disponibles pour de futurs usages :
GPIOFonction AF possiblePE1Libre (ex LCD_SCK bit-bang)PE4Libre (ex LCD_MOSI bit-bang)PE5GPIO / TIM9_CH1PE0GPIO génériquePB7GPIO / UART1_RXPA3GPIO / UART2_RX / TIM2_CH4PA5GPIO / SPI1_SCKPA6GPIO / SPI1_MISO / TIM3_CH1PB0GPIO / TIM3_CH3PB1GPIO / TIM3_CH4PD4GPIO génériquePD8GPIO / USART3_TXPD10GPIO génériquePD12GPIO / TIM4_CH1PD14GPIO / TIM4_CH3PC6GPIO / USART6_TX / TIM3_CH1PC7GPIO / USART6_RX / TIM3_CH2PA0GPIO / UART4_TX / TIM2_CH1PA8GPIO / TIM1_CH1

Diagnostic rapide
SymptômeCause probableSolutionÉcran blanc, rien affichéCS non actif ou DC mal câbléVérifier PA4 et PE6 en OutputÉcran noir, backlight alluméST7789_Init() non reçuVérifier SPI2 avec analyseur logiqueCouleurs inverséesInversion ON/OFFSwapper 0x21 ↔ 0x20 dans InitImage décaléeY_SHIFT incorrectAjuster Y_SHIFT (essayer 0, 20, 40)Conflit SPI2CS Flash/NRF non HIGHVérifier PE3 et PE8 HIGH avant toute comm LCDÉcran vert à la fin des tests—✅ Tous les tests passés

Construction et flash
bash# Compiler
make clean && make all

# Flasher via DFU (BOOT0=1, mini USB connecté)
make flash

# Remettre BOOT0=0 et reset