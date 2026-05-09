#include "ws2815.h"
#include "main.h"

/* -----------------------------------------------------------------------
 * Timings bit-bang à 168 MHz (1 cycle ≈ 5.95 ns)
 *   T0H ≈ 300 ns  → 50 NOP
 *   T1H ≈ 600 ns  → 100 NOP
 *   Reset > 280 µs → HAL_Delay(1)
 * ----------------------------------------------------------------------- */
#define NOP() __asm volatile ("nop")

#define T0H()  do { \
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); \
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); \
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); \
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); \
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); \
} while(0)

#define T1H()  do { \
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); \
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); \
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); \
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); \
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); \
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); \
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); \
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); \
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); \
    NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP(); \
} while(0)

/* Envoie un octet MSB en premier (format GRB attendu par le protocole) */
static void send_byte(GPIO_TypeDef *port, uint16_t pin, uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
        if (byte & (1 << i)) {
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
            T1H();
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
            T0H();
        } else {
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
            T0H();
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
            T1H();
        }
    }
}

/* Envoie une chaîne complète (IRQ désactivées pendant l'envoi) */
static void send_chain(WS2815_Chain_t *ch)
{
    __disable_irq();
    for (uint16_t i = 0; i < ch->num_leds; i++) {
        send_byte(ch->port, ch->pin, ch->pixels[i].g); /* GRB ! */
        send_byte(ch->port, ch->pin, ch->pixels[i].r);
        send_byte(ch->port, ch->pin, ch->pixels[i].b);
    }
    __enable_irq();
    HAL_Delay(1); /* Reset > 280 µs pour WS2815 */
}

/* -----------------------------------------------------------------------
 * API publique
 * ----------------------------------------------------------------------- */

void WS2815_Init(WS2815_Chain_t *ch, GPIO_TypeDef *port, uint16_t pin, uint16_t num_leds)
{
    ch->port     = port;
    ch->pin      = pin;
    ch->num_leds = (num_leds > WS2815_MAX_LEDS) ? WS2815_MAX_LEDS : num_leds;
    memset(ch->pixels, 0, sizeof(ch->pixels));
}

void WS2815_SetLed(WS2815_Chain_t *ch, uint16_t idx, WS2815Pixel_t color)
{
    if (idx < ch->num_leds)
        ch->pixels[idx] = color;
}

int WS2815_Busy(void)
{
    return 0; /* bit-bang synchrone : jamais occupé après Show */
}

void WS2815_Show(WS2815_Chain_t *chains, uint8_t num_chains)
{
    for (uint8_t i = 0; i < num_chains; i++)
        send_chain(&chains[i]);
}
