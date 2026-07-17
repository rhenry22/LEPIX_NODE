#include "ws2815.h"
#include "tim.h"
#include "main.h"

/* -----------------------------------------------------------------------
 * Driver WS2815 par TIM1 + DMA2 sur GPIOD (PD15/PD13/PD11/PD9 en parallèle)
 *
 * Technique « 3 streams » : à chaque bit (1.25 µs) le timer déclenche
 * trois écritures DMA dans GPIOD->BSRR :
 *   UP  (t=0)       : toutes les pins montent à 1
 *   CC1 (t=0.30 µs) : les pins dont le bit courant vaut '0' redescendent
 *   CC2 (t=0.70 µs) : toutes les pins redescendent
 * Les 4 chaînes sont donc émises simultanément, sans bloquer le CPU
 * (~3.6 ms pour 120 LEDs, entièrement en DMA).
 *
 * Contrainte : toutes les chaînes doivent être sur GPIOD (voir tim.h).
 * ----------------------------------------------------------------------- */

/* 1 mot par bit : masque BSRR des pins à abaisser à T0H (bits '0') */
static uint32_t ws_dma_buf[WS2815_MAX_LEDS * 24];

static const uint32_t ws_set_word   = WS_ALL_PINS;      /* BSRR : set   */
static const uint32_t ws_reset_word = BSRR_RESET_ALL;   /* BSRR : reset */

static volatile uint8_t  ws_dma_running   = 0;
static volatile uint32_t ws_frame_end_ms  = 0;

/* Fin de trame : le dernier transfert du stream CC2 a eu lieu,
 * toutes les pins sont à 0. On arrête le timer et on libère les streams. */
static void ws_dma_complete(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    __HAL_TIM_DISABLE(&htim1);
    __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE | TIM_DMA_CC1 | TIM_DMA_CC2);
    htim1.Instance->CNT = 0;

    /* Les streams set/data sont déjà terminés (même NDTR) ; Abort remet
     * juste leur état HAL à READY pour le prochain HAL_DMA_Start. */
    HAL_DMA_Abort(&hdma_set);
    HAL_DMA_Abort(&hdma_data);

    GPIOD->BSRR = BSRR_RESET_ALL;   /* latch : lignes maintenues à 0 */

    ws_frame_end_ms = HAL_GetTick();
    ws_dma_running  = 0;
}

/* -----------------------------------------------------------------------
 * API publique (inchangée)
 * ----------------------------------------------------------------------- */

void WS2815_Init(WS2815_Chain_t *ch, GPIO_TypeDef *port, uint16_t pin, uint16_t num_leds)
{
    (void)port; /* le driver DMA pilote GPIOD uniquement (cf. tim.h) */
    ch->port     = GPIOD;
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
    if (ws_dma_running)
        return 1;
    /* Latch WS2815 : >280 µs de niveau bas entre deux trames.
     * HAL_GetTick a une résolution de 1 ms -> on impose 2 ticks. */
    if ((HAL_GetTick() - ws_frame_end_ms) < 2)
        return 1;
    return 0;
}

void WS2815_Show(WS2815_Chain_t *chains, uint8_t num_chains)
{
    /* Sécurité : attendre la fin d'une trame en cours (timeout 10 ms) */
    uint32_t t0 = HAL_GetTick();
    while (WS2815_Busy()) {
        if (HAL_GetTick() - t0 > 10)
            return;
    }

    /* Longueur de trame = chaîne la plus longue ; les chaînes plus
     * courtes reçoivent des bits '0' au-delà de leur longueur (sans effet) */
    uint16_t max_leds = 0;
    for (uint8_t c = 0; c < num_chains; c++)
        if (chains[c].num_leds > max_leds)
            max_leds = chains[c].num_leds;
    if (max_leds == 0)
        return;

    /* Construction du buffer : ordre WS2815 = G puis R puis B, MSB first */
    uint32_t idx = 0;
    for (uint16_t l = 0; l < max_leds; l++) {
        for (uint8_t bit = 0; bit < 24; bit++) {
            uint16_t low_mask = 0;
            for (uint8_t c = 0; c < num_chains; c++) {
                const WS2815_Chain_t *ch = &chains[c];
                if (l >= ch->num_leds) {
                    low_mask |= ch->pin;
                    continue;
                }
                uint8_t byte;
                if      (bit < 8)  byte = ch->pixels[l].g;
                else if (bit < 16) byte = ch->pixels[l].r;
                else               byte = ch->pixels[l].b;
                if (!(byte & (0x80u >> (bit & 7u))))
                    low_mask |= ch->pin;
            }
            ws_dma_buf[idx++] = ((uint32_t)low_mask) << 16;
        }
    }

    uint32_t slots = idx;   /* = 24 * max_leds transferts par stream */

    ws_dma_running = 1;

    hdma_reset.XferCpltCallback     = ws_dma_complete;
    hdma_reset.XferHalfCpltCallback = NULL;
    hdma_reset.XferErrorCallback    = NULL;

    HAL_DMA_Start(&hdma_set,      (uint32_t)&ws_set_word,   (uint32_t)&GPIOD->BSRR, slots);
    HAL_DMA_Start(&hdma_data,     (uint32_t)ws_dma_buf,     (uint32_t)&GPIOD->BSRR, slots);
    HAL_DMA_Start_IT(&hdma_reset, (uint32_t)&ws_reset_word, (uint32_t)&GPIOD->BSRR, slots);

    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE | TIM_FLAG_CC1 | TIM_FLAG_CC2);
    htim1.Instance->CNT = 0;
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE | TIM_DMA_CC1 | TIM_DMA_CC2);

    /* UG force un événement Update immédiat : première écriture SET
     * (toutes pins à 1) au moment où le compteur démarre. */
    htim1.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_ENABLE(&htim1);
}
