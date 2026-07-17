#include "mode_select.h"
#include "main.h"   /* HAL, GPIOA, P4_GPIOA5_Pin, P4_GPIOA6_Pin */

/* Jumper de sélection de mode : PA5 = MODE_DMX, PA6 réservé (futur). */
#define MODE_JUMPER_DMX_Pin   P4_GPIOA5_Pin   /* GPIO_PIN_5 */
#define MODE_JUMPER_ALT_Pin   P4_GPIOA6_Pin   /* GPIO_PIN_6 (réservé)     */
#define MODE_JUMPER_Port      GPIOA

static OperatingMode_t s_mode = MODE_LED;

void Mode_Init(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Entrées avec pull-up : un jumper vers la masse lit 0, sinon 1. */
    gi.Pin  = MODE_JUMPER_DMX_Pin | MODE_JUMPER_ALT_Pin;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MODE_JUMPER_Port, &gi);

    /* Laisser le temps au pull-up d'établir le niveau avant lecture. */
    HAL_Delay(1);

    /* PA5 ponté à la masse -> mode DMX ; sinon LED. */
    if (HAL_GPIO_ReadPin(MODE_JUMPER_Port, MODE_JUMPER_DMX_Pin) == GPIO_PIN_RESET)
        s_mode = MODE_DMX;
    else
        s_mode = MODE_LED;
}

OperatingMode_t Mode_Get(void)
{
    return s_mode;
}

const char *Mode_Name(OperatingMode_t m)
{
    switch (m) {
        case MODE_LED: return "LED (WS2815)";
        case MODE_DMX: return "DMX/RDM";
        default:       return "?";
    }
}
