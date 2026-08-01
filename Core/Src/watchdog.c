#include "watchdog.h"
#include "main.h"   /* HAL, IWDG */

/* IWDG : horloge LSI (~32 kHz). Prescaler /32 -> ~1 kHz -> 1 tick ≈ 1 ms.
 * Reload = 2000 -> timeout ~2 s. Marge large : les operations lentes
 * legitimes (WS2815_Show ~3,6 ms, acces SD) restent tres en dessous, et
 * la boucle rafraichit a chaque iteration. */
#define IWDG_RELOAD   2000u   /* ~2 s */

static IWDG_HandleTypeDef s_hiwdg;

void Watchdog_Init(void)
{
    s_hiwdg.Instance       = IWDG;
    s_hiwdg.Init.Prescaler = IWDG_PRESCALER_32;   /* 32 kHz / 32 = 1 kHz */
    s_hiwdg.Init.Reload    = IWDG_RELOAD;
    /* Ce derive HAL n'expose pas le mode fenetre (pas de champ Window). */

    /* HAL_IWDG_Init demarre le watchdog (registre KR). Irreversible. */
    if (HAL_IWDG_Init(&s_hiwdg) != HAL_OK) {
        /* En cas d'echec, ne pas bloquer le boot : on continue sans WDG. */
        return;
    }
}

void Watchdog_Refresh(void)
{
    HAL_IWDG_Refresh(&s_hiwdg);
}
