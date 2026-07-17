/* mode_select.h
 * Sélection du mode de fonctionnement par jumper matériel sur PA5/PA6.
 *
 * Le node peut piloter soit des LEDs WS2815 (sortie sur GPIOD), soit une
 * ligne DMX512/RDM (MAX485 sur USART2/PD5-PD6, direction PD7). Un jumper
 * lu au démarrage détermine le mode.
 *
 * Câblage (entrées à pull-up interne, jumper = pont vers la masse) :
 *   PA5 ouvert / PA6 ouvert  -> MODE_LED   (défaut, aucun jumper)
 *   PA5 à la masse           -> MODE_DMX
 *   PA6 à la masse (réservé) -> usage futur (ex. entrée DMX)
 */
#ifndef MODE_SELECT_H
#define MODE_SELECT_H

typedef enum {
    MODE_LED = 0,   /* sorties WS2815 (comportement historique) */
    MODE_DMX,       /* sortie DMX512 / RDM sur XLR (MAX485)      */
} OperatingMode_t;

/* Configure PA5/PA6 en entrée pull-up et lit le jumper. À appeler une fois
 * au démarrage, après MX_GPIO_Init(). Mémorise le mode pour Mode_Get(). */
void            Mode_Init(void);

/* Retourne le mode déterminé par Mode_Init(). */
OperatingMode_t Mode_Get(void);

/* Libellé lisible du mode courant (pour logs / UI). */
const char     *Mode_Name(OperatingMode_t m);

#endif /* MODE_SELECT_H */
