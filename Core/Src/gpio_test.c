#include "gpio_test.h"
#include "main.h"          /* HAL, defines de broches */
#include "mode_select.h"   /* Mode_Get / Mode_Name    */
#include <stdio.h>

/* Lecture d'une broche -> "1 (haut)" / "0 (bas)". */
static const char *lvl(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? "1 (haut)" : "0 (bas)";
}

void GpioTest_Run(void)
{
    printf("\r\n===== TEST GPIO (lecture au boot) =====\r\n");

    /* ---- Jumpers de config PA5 (mode) / PA6 (web), entrées pull-up ---- */
    printf("[Jumper] PA5 = %s | PA6 = %s\r\n",
           lvl(P4_GPIOA5_GPIO_Port, P4_GPIOA5_Pin),
           lvl(P4_GPIOA6_GPIO_Port, P4_GPIOA6_Pin));
    printf("[Jumper] PA5 -> mode : %s (%s)\r\n",
           Mode_Name(Mode_Get()),
           (Mode_Get() == MODE_DMX) ? "PA5 pontee a la masse"
                                    : "PA5 ouverte (pas de jumper)");
    printf("[Jumper] PA6 -> web  : %s (%s)\r\n",
           Mode_WebEnabled() ? "active" : "desactive",
           Mode_WebEnabled() ? "PA6 pontee a la masse"
                             : "PA6 ouverte (pas de jumper)");

    /* ---- GPIO DMX ---- */
    /* Direction MAX485 : PD7 (port 0) et PD10 (port 1).
     * DIR_TX = niveau haut (emission), DIR_RX = niveau bas (ecoute). */
    printf("[DMX] Direction port0 PD7  = %s (%s)\r\n",
           lvl(RS485_TX_RX__GPIO_Port, RS485_TX_RX__Pin),
           (HAL_GPIO_ReadPin(RS485_TX_RX__GPIO_Port, RS485_TX_RX__Pin) == GPIO_PIN_SET)
               ? "TX/emission" : "RX/ecoute");
    printf("[DMX] Direction port1 PD10 = %s (%s)\r\n",
           lvl(P5_GPIOD10_GPIO_Port, P5_GPIOD10_Pin),
           (HAL_GPIO_ReadPin(P5_GPIOD10_GPIO_Port, P5_GPIOD10_Pin) == GPIO_PIN_SET)
               ? "TX/emission" : "RX/ecoute");

    /* Lignes de donnees (en mode DMX elles sont en AF USART : la lecture
     * GPIO reflete l'etat courant de la ligne, indicatif). */
    printf("[DMX] Port0 TX PD5  = %s | RX PD6  = %s\r\n",
           lvl(GPIOD, GPIO_PIN_5), lvl(GPIOD, GPIO_PIN_6));
    printf("[DMX] Port1 TX PC10 = %s | RX PC11 = %s\r\n",
           lvl(GPIOC, GPIO_PIN_10), lvl(GPIOC, GPIO_PIN_11));

    printf("=======================================\r\n\r\n");
}
