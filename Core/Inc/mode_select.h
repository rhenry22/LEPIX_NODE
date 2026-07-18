/* mode_select.h
 * Configuration matérielle par 2 jumpers indépendants sur PA5/PA6.
 *
 * Deux réglages lus au démarrage :
 *   PA5 -> mode de sortie : LED (WS2815) ou DMX512/RDM (MAX485)
 *   PA6 -> interface web (serveur httpd) activée ou non
 *
 * Câblage (entrées à pull-up interne, jumper = pont vers la masse) :
 *   PA5 ouvert  -> MODE_LED (défaut)   | PA5 à la masse -> MODE_DMX
 *   PA6 ouvert  -> web désactivé (déf.) | PA6 à la masse -> web activé
 */
#ifndef MODE_SELECT_H
#define MODE_SELECT_H

#include <stdbool.h>

typedef enum {
    MODE_LED = 0,   /* sorties WS2815 (comportement historique) */
    MODE_DMX,       /* sortie DMX512 / RDM sur XLR (MAX485)      */
} OperatingMode_t;

/* Configure PA5/PA6 en entrée pull-up et lit les 2 jumpers. À appeler une
 * fois au démarrage, après MX_GPIO_Init(). */
void            Mode_Init(void);

/* Mode de sortie déterminé par le jumper PA5. */
OperatingMode_t Mode_Get(void);

/* true si le jumper PA6 demande l'activation de l'interface web. */
bool            Mode_WebEnabled(void);

/* Libellé lisible du mode courant (pour logs / UI). */
const char     *Mode_Name(OperatingMode_t m);

#endif /* MODE_SELECT_H */
