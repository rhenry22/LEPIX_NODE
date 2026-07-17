#ifndef WS2815_H
#define WS2815_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>

/* Nombre maximum de LEDs par chaîne */
#define WS2815_MAX_LEDS  120

/* Pixel RGB */
typedef struct {
    uint8_t r, g, b;
} WS2815Pixel_t;

/* Chaîne WS2815 (buffer intégré) */
typedef struct {
    GPIO_TypeDef   *port;
    uint16_t        pin;
    WS2815Pixel_t   pixels[WS2815_MAX_LEDS];
    uint16_t        num_leds;
} WS2815_Chain_t;

/* Couleurs prédéfinies */
#define WS2815_BLACK  ((WS2815Pixel_t){  0,   0,   0})
#define WS2815_WHITE  ((WS2815Pixel_t){255, 255, 255})
#define WS2815_RED    ((WS2815Pixel_t){255,   0,   0})
#define WS2815_GREEN  ((WS2815Pixel_t){  0, 255,   0})
#define WS2815_BLUE   ((WS2815Pixel_t){  0,   0, 255})

/**
 * @brief Initialise une chaîne WS2815 et efface le buffer.
 *        Le driver TIM1+DMA2 pilote GPIOD uniquement : pin doit être
 *        l'une de PD15/PD13/PD11/PD9 (voir tim.h) et MX_TIM1_WS2815_Init()
 *        doit avoir été appelé avant le premier WS2815_Show().
 * @param ch       Pointeur vers la structure chaîne
 * @param port     Ignoré (GPIOD imposé par le driver DMA)
 * @param pin      Numéro de pin GPIOD (ex: GPIO_PIN_15)
 * @param num_leds Nombre de LEDs (max WS2815_MAX_LEDS)
 */
void WS2815_Init(WS2815_Chain_t *ch, GPIO_TypeDef *port, uint16_t pin, uint16_t num_leds);

/**
 * @brief Définit la couleur d'une LED dans le buffer.
 */
void WS2815_SetLed(WS2815_Chain_t *ch, uint16_t idx, WS2815Pixel_t color);

/**
 * @brief 1 si une trame DMA est en cours ou si le temps de latch
 *        (>280 µs de niveau bas) n'est pas encore écoulé.
 */
int WS2815_Busy(void);

/**
 * @brief Lance l'envoi de toutes les chaînes en parallèle par DMA
 *        (non bloquant, ~3.6 ms pour 120 LEDs). Les données des chaînes
 *        sont recopiées dans le buffer DMA avant le retour : les buffers
 *        pixels peuvent être modifiés immédiatement après.
 * @param chains     Tableau de WS2815_Chain_t
 * @param num_chains Nombre de chaînes
 */
void WS2815_Show(WS2815_Chain_t *chains, uint8_t num_chains);

#endif /* WS2815_H */
