/* test_seq.c
 * Séquences de test pour valider un déploiement sans signal réseau.
 * Voir test_seq.h pour le contrat. Un seul test actif à la fois ;
 * TestSeq_Task() est appelée dans la boucle principale, avant l'aiguillage
 * réseau habituel — tant qu'un test est actif elle "consomme" le tour et
 * l'appelant ne doit pas laisser le flux réseau écraser la sortie.
 */
#include "test_seq.h"
#include "config.h"
#include "mode_select.h"
#include "ws2815.h"
#include "dmx.h"
#include <string.h>

extern uint32_t HAL_GetTick(void);
extern WS2815_Chain_t all_chains[MAX_OUTPUTS];

#define STEP_MS 40u   /* ~25 fps pour chase/rainbow */

static struct {
    bool          active;
    TestPattern_t pattern;
    uint8_t       output;      /* 0..MAX_OUTPUTS-1 ou TEST_SEQ_ALL_OUTPUTS */
    uint32_t      solid_rgb;
    uint32_t      started_ms;
    uint32_t      last_step_ms;
    uint16_t      phase;        /* position d'animation (chase/rainbow) */
} s_test;

void TestSeq_Start(TestPattern_t pattern, uint8_t output, uint32_t solid_rgb)
{
    if (pattern == TEST_PATTERN_OFF) {
        TestSeq_Stop();
        return;
    }
    s_test.active       = true;
    s_test.pattern      = pattern;
    s_test.output       = output;
    s_test.solid_rgb    = solid_rgb;
    s_test.started_ms   = HAL_GetTick();
    s_test.last_step_ms = 0;
    s_test.phase        = 0;
}

void TestSeq_Stop(void)
{
    if (!s_test.active)
        return;
    s_test.active = false;

    /* Restaure un etat neutre pour ne pas laisser le dernier pattern
     * fige a l'ecran ; le prochain paquet reseau ecrasera de toute facon. */
    if (Mode_Get() != MODE_DMX) {
        for (uint8_t i = 0; i < MAX_OUTPUTS; i++)
            for (uint16_t l = 0; l < all_chains[i].num_leds; l++)
                WS2815_SetLed(&all_chains[i], l, WS2815_BLACK);
        if (!WS2815_Busy())
            WS2815_Show(all_chains, MAX_OUTPUTS);
    }
}

bool TestSeq_IsActive(void)
{
    return s_test.active;
}

void TestSeq_GetStatus(TestPattern_t *pattern, uint8_t *output, uint32_t *remaining_ms)
{
    if (pattern) *pattern = s_test.active ? s_test.pattern : TEST_PATTERN_OFF;
    if (output)  *output  = s_test.output;
    if (remaining_ms) {
        uint32_t elapsed = HAL_GetTick() - s_test.started_ms;
        *remaining_ms = (s_test.active && elapsed < TEST_SEQ_TIMEOUT_MS)
                       ? (TEST_SEQ_TIMEOUT_MS - elapsed) : 0;
    }
}

/* ─────────────────────────────────────────────────────────────────────────
 *  Mode LED : ecrit dans all_chains[], memes primitives que ws2815.c
 * ───────────────────────────────────────────────────────────────────────── */

/* HSV -> RGB simplifie (h: 0-359, s=v=255) pour le degrade arc-en-ciel. */
static WS2815Pixel_t hue_to_rgb(uint16_t h)
{
    uint8_t region = h / 60;
    uint8_t rem    = (h % 60) * 255 / 60;
    uint8_t rising  = rem;
    uint8_t falling = 255 - rem;
    switch (region) {
        case 0:  return (WS2815Pixel_t){255, rising, 0};
        case 1:  return (WS2815Pixel_t){falling, 255, 0};
        case 2:  return (WS2815Pixel_t){0, 255, rising};
        case 3:  return (WS2815Pixel_t){0, falling, 255};
        case 4:  return (WS2815Pixel_t){rising, 0, 255};
        default: return (WS2815Pixel_t){255, 0, falling};
    }
}

static void led_apply_pattern(void)
{
    WS2815Pixel_t solid = {
        (uint8_t)(s_test.solid_rgb >> 16), (uint8_t)(s_test.solid_rgb >> 8),
        (uint8_t)(s_test.solid_rgb)
    };

    for (uint8_t i = 0; i < MAX_OUTPUTS; i++) {
        if (s_test.output != TEST_SEQ_ALL_OUTPUTS && s_test.output != i)
            continue;
        WS2815_Chain_t *ch = &all_chains[i];
        for (uint16_t l = 0; l < ch->num_leds; l++) {
            WS2815Pixel_t px;
            switch (s_test.pattern) {
            case TEST_PATTERN_SOLID:
                px = solid;
                break;
            case TEST_PATTERN_FLASH:
                px = ((s_test.phase / 12) & 1) ? WS2815_WHITE : WS2815_BLACK;
                break;
            case TEST_PATTERN_CHASE:
                px = (l == (s_test.phase % (ch->num_leds ? ch->num_leds : 1)))
                     ? WS2815_WHITE : WS2815_BLACK;
                break;
            case TEST_PATTERN_RAINBOW:
                px = hue_to_rgb((uint16_t)((l * 8 + s_test.phase) % 360));
                break;
            default:
                px = WS2815_BLACK;
                break;
            }
            WS2815_SetLed(ch, l, px);
        }
    }
    if (!WS2815_Busy())
        WS2815_Show(all_chains, MAX_OUTPUTS);
}

/* ─────────────────────────────────────────────────────────────────────────
 *  Mode DMX : ecrit directement les slots des 2 ports filaires.
 *  Pas de notion d'univers ici : output 0 -> port 0, output 1 -> port 1.
 * ───────────────────────────────────────────────────────────────────────── */

static void dmx_apply_pattern(void)
{
    uint8_t frame[DMX_SLOTS];

    for (uint8_t port = 0; port < DMX_NUM_PORTS; port++) {
        if (s_test.output != TEST_SEQ_ALL_OUTPUTS && s_test.output != port)
            continue;

        switch (s_test.pattern) {
        case TEST_PATTERN_SOLID:
            memset(frame, (uint8_t)s_test.solid_rgb, sizeof(frame));
            break;
        case TEST_PATTERN_FLASH:
            memset(frame, ((s_test.phase / 12) & 1) ? 255 : 0, sizeof(frame));
            break;
        case TEST_PATTERN_CHASE: {
            memset(frame, 0, sizeof(frame));
            uint16_t idx = s_test.phase % DMX_SLOTS;
            frame[idx] = 255;
            break;
        }
        case TEST_PATTERN_RAINBOW:
            /* Pas de notion de couleur en DMX brut : degrade de valeur
             * canal par canal, a titre de motif visible sur un dimmer. */
            for (uint16_t c = 0; c < DMX_SLOTS; c++)
                frame[c] = (uint8_t)(((c + s_test.phase) * 255) / DMX_SLOTS);
            break;
        default:
            memset(frame, 0, sizeof(frame));
            break;
        }

        DMX_SetSlots(port, 0, frame, DMX_SLOTS);
        DMX_Commit(port);
    }
}

bool TestSeq_Task(void)
{
    if (!s_test.active)
        return false;

    uint32_t now = HAL_GetTick();

    /* Coupure de securite : jamais de test oublie en prod. */
    if (now - s_test.started_ms >= TEST_SEQ_TIMEOUT_MS) {
        TestSeq_Stop();
        return false;
    }

    if (now - s_test.last_step_ms < STEP_MS)
        return true;   /* test actif, rien a rafraichir ce tour */
    s_test.last_step_ms = now;
    s_test.phase++;

    if (Mode_Get() == MODE_DMX)
        dmx_apply_pattern();
    else
        led_apply_pattern();

    return true;
}
