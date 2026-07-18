#include "mode_select.h"
#include "main.h"   /* HAL, GPIOA, P4_GPIOA5_Pin, P4_GPIOA6_Pin */

/* Jumpers de configuration :
 *   PA5 = mode de sortie (masse -> DMX, ouvert -> LED)
 *   PA6 = interface web    (masse -> activee, ouvert -> desactivee) */
#define MODE_JUMPER_DMX_Pin   P4_GPIOA5_Pin   /* GPIO_PIN_5 */
#define MODE_JUMPER_WEB_Pin   P4_GPIOA6_Pin   /* GPIO_PIN_6 */
#define MODE_JUMPER_Port      GPIOA

static OperatingMode_t s_mode        = MODE_LED;
static bool            s_web_enabled = false;

void Mode_Init(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Entrées avec pull-up : un jumper vers la masse lit 0, sinon 1. */
    gi.Pin  = MODE_JUMPER_DMX_Pin | MODE_JUMPER_WEB_Pin;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MODE_JUMPER_Port, &gi);

    /* Laisser le temps au pull-up d'établir le niveau avant lecture. */
    HAL_Delay(1);

    /* PA5 ponté à la masse -> mode DMX ; sinon LED. */
    s_mode = (HAL_GPIO_ReadPin(MODE_JUMPER_Port, MODE_JUMPER_DMX_Pin) == GPIO_PIN_RESET)
                 ? MODE_DMX : MODE_LED;

    /* PA6 ponté à la masse -> interface web activée ; sinon désactivée. */
    s_web_enabled =
        (HAL_GPIO_ReadPin(MODE_JUMPER_Port, MODE_JUMPER_WEB_Pin) == GPIO_PIN_RESET);
}

OperatingMode_t Mode_Get(void)
{
    return s_mode;
}

bool Mode_WebEnabled(void)
{
    return s_web_enabled;
}

const char *Mode_Name(OperatingMode_t m)
{
    switch (m) {
        case MODE_LED: return "LED (WS2815)";
        case MODE_DMX: return "DMX/RDM";
        default:       return "?";
    }
}
