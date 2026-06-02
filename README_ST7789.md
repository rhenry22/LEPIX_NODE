# Intégration écran TFT ST7789 240×280 — LEPIX_NODE STM32F407VET6

## Vue d'ensemble

Ce document décrit toutes les modifications apportées au projet **LEPIX_NODE** (basé sur la carte industrielle STM32F407VET6) pour intégrer un écran TFT SPI 240×280 pixels piloté par le contrôleur **ST7789**, en utilisant le bus **SPI2 hardware** via la librairie Floyd-Fish portée sous STM32 HAL.

---

## Matériel utilisé

| Composant | Référence |
|---|---|
| Microcontrôleur | STM32F407VET6 (LQFP100, 168 MHz) |
| Carte | JZ-STM32F407VET6 Industrial Board |
| Écran | TFT SPI ST7789 240×280 px 3.3V |
| Bus SPI | SPI2 hardware (21 Mbits/s) |

---

## Câblage physique

| Broche module ST7789 | GPIO STM32 | Label projet | Remarque |
|---|---|---|---|
| GND | GND | — | Masse commune obligatoire |
| VCC | 3.3V | — | Alimentation 3.3V |
| SCL (SCK) | PB10 | SPI2_SCK | Partagé avec Flash W25Q128 + NRF24L01 |
| SDA (MOSI) | PC3 | SPI2_MOSI | Partagé avec Flash W25Q128 + NRF24L01 |
| RES (RST) | PC0 | LCD_RST | GPIO Output dédié |
| DC | PE6 | LCD_DC | GPIO Output dédié |
| CS | PA4 | LCD_CS | GPIO Output dédié — isole l'écran sur le bus SPI2 |
| BLK | PC13 | LCD_BLK | GPIO Output dédié — backlight |

> **Important** : SCK et MOSI sont partagés avec la Flash SPI et le NRF24L01. L'isolation est assurée par les CS respectifs :
> - Flash W25Q128 → PE3 (SPI2_FLASH_CS)
> - NRF24L01 → PE8 (SPI2_NRF_CS)
> - Écran ST7789 → PA4 (LCD_CS)
>
> Un seul CS doit être actif (LOW) à la fois.

---

## Modifications du fichier `.ioc` / CubeMX

Les GPIO suivants ont été ajoutés ou modifiés dans `Industrial_Board.ioc` :

```ini
# Ajouts LCD
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
```

> **Note** : PE1 et PE4, utilisés initialement pour le SPI bit-bang, sont maintenant libres pour d'autres usages.

---

## Modifications `Core/Inc/main.h`

Ajouter dans la section `USER CODE BEGIN Private defines` :

```c
/* USER CODE BEGIN Private defines */

/* LCD ST7789 — CS, DC, RST, BLK */
#define ST7789_CS_Pin        GPIO_PIN_4
#define ST7789_CS_GPIO_Port  GPIOA

#define ST7789_DC_Pin        GPIO_PIN_6
#define ST7789_DC_GPIO_Port  GPIOE

#define ST7789_RST_Pin       GPIO_PIN_0
#define ST7789_RST_GPIO_Port GPIOC

/* LCD_BLK et LCD_RST générés automatiquement par CubeMX */

/* USER CODE END Private defines */
```

---

## Modifications `Core/Src/gpio.c`

Ajouter à l'intérieur de `MX_GPIO_Init()`, avant l'accolade fermante, dans la section `USER CODE BEGIN 2` :

```c
/* USER CODE BEGIN 2 */

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
```

> **Attention** : retirer `PC0` et `PC13` des blocs `GPIO_MODE_INPUT` existants pour éviter qu'ils soient réinitialisés en entrée.

---

## Fichiers ajoutés au projet

### `Core/Inc/st7789.h`

Librairie Floyd-Fish adaptée. Modifications apportées :

| Paramètre | Valeur originale | Valeur modifiée |
|---|---|---|
| `ST7789_SPI_PORT` | `hspi1` | `hspi2` |
| Résolution active | `USING_240X240` | `USING_260x240` (ajouté) |
| `ST7789_CS_PORT` | `ST7789_CS_GPIO_Port` | `GPIOA` |
| `ST7789_CS_PIN` | `ST7789_CS_Pin` | `GPIO_PIN_4` |

Bloc résolution ajouté :

```c
#ifdef USING_260x240
    #define ST7789_WIDTH  240
    #define ST7789_HEIGHT 280
    #define X_SHIFT 0
    #define Y_SHIFT 20   // offset typique des modules 260x240
#endif
```

### `Core/Src/st7789.c`

Correction du cast de type ligne 188 :

```c
// Avant
ST7789_WriteData(disp_buf, sizeof(disp_buf));

// Après
ST7789_WriteData((uint8_t*)disp_buf, sizeof(disp_buf));
```

### `Core/Inc/st7789_test.h`

```c
#ifndef ST7789_TEST_H
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
```

### `Core/Src/st7789_test.c`

Programme de validation avec 7 tests progressifs :

| Test | Fonction Floyd-Fish utilisée | Résultat attendu |
|---|---|---|
| Test 1 | `HAL_GPIO_WritePin` BLK | Backlight clignote 3× |
| Test 2 | `ST7789_Fill_Color(RED)` | Écran rouge uniforme |
| Test 3 | `ST7789_Fill_Color(...)` | Cycle 8 couleurs |
| Test 4 | `ST7789_Fill(x0,y0,x1,y1,c)` | Damier rouge/bleu 8×10 |
| Test 5 | `ST7789_Fill(...)` ligne par ligne | Dégradé vertical vert |
| Test 6 | `ST7789_Fill(...)` | Croix blanche + 4 coins colorés |
| Test 7 | `ST7789_WriteString(...)` | Texte "ST7789 OK / SPI2 HAL / 260x240" |

---

## Correspondance API — ancien driver vs Floyd-Fish

| Ancien driver bit-bang | Floyd-Fish HAL SPI |
|---|---|
| `ST7789_Init()` | `ST7789_Init()` (inchangé) |
| `ST7789_FillScreen(color)` | `ST7789_Fill_Color(color)` |
| `ST7789_FillRect(x, y, w, h, c)` | `ST7789_Fill(x0, y0, x1, y1, c)` |
| `ST7789_DrawPixel(x, y, c)` | `ST7789_DrawPixel(x, y, c)` (inchangé) |
| `ST7789_DrawPixel_4px(x, y, c)` | `ST7789_DrawPixel_4px(x, y, c)` (inchangé) |
| `LCD_BLK_HIGH()` | `HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_SET)` |
| `LCD_BLK_LOW()` | `HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_RESET)` |

---

## Intégration dans `Core/Src/main.c`

```c
/* USER CODE BEGIN Includes */
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
```

---

## GPIO libres après intégration

Les GPIO suivants restent disponibles pour de futurs usages :

| GPIO | Fonction AF possible |
|---|---|
| PE1 | Libre (ex LCD_SCK bit-bang) |
| PE4 | Libre (ex LCD_MOSI bit-bang) |
| PE5 | GPIO / TIM9_CH1 |
| PE0 | GPIO générique |
| PB7 | GPIO / UART1_RX |
| PA3 | GPIO / UART2_RX / TIM2_CH4 |
| PA5 | GPIO / SPI1_SCK |
| PA6 | GPIO / SPI1_MISO / TIM3_CH1 |
| PB0 | GPIO / TIM3_CH3 |
| PB1 | GPIO / TIM3_CH4 |
| PD4 | GPIO générique |
| PD8 | GPIO / USART3_TX |
| PD10 | GPIO générique |
| PD12 | GPIO / TIM4_CH1 |
| PD14 | GPIO / TIM4_CH3 |
| PC6 | GPIO / USART6_TX / TIM3_CH1 |
| PC7 | GPIO / USART6_RX / TIM3_CH2 |
| PA0 | GPIO / UART4_TX / TIM2_CH1 |
| PA8 | GPIO / TIM1_CH1 |

---

## Diagnostic rapide

| Symptôme | Cause probable | Solution |
|---|---|---|
| Écran blanc, rien affiché | CS non actif ou DC mal câblé | Vérifier PA4 et PE6 en Output |
| Écran noir, backlight allumé | `ST7789_Init()` non reçu | Vérifier SPI2 avec analyseur logique |
| Couleurs inversées | Inversion ON/OFF | Swapper `0x21` ↔ `0x20` dans Init |
| Image décalée | Y_SHIFT incorrect | Ajuster `Y_SHIFT` (essayer 0, 20, 40) |
| Conflit SPI2 | CS Flash/NRF non HIGH | Vérifier PE3 et PE8 HIGH avant toute comm LCD |
| Écran vert à la fin des tests | — | ✅ Tous les tests passés |

---

## Construction et flash

```bash
# Compiler
make clean && make all

# Flasher via DFU (BOOT0=1, mini USB connecté)
make flash

# Remettre BOOT0=0 et reset
```

---

*Document généré en association avec le projet LEPIX_NODE — rhenry22/LEPIX_NODE*
