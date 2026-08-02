#ifndef TEST_SEQ_H
#define TEST_SEQ_H

#include <stdint.h>
#include <stdbool.h>

/* Séquences de test pour valider un déploiement sans signal réseau.
 * Fonctionne dans les deux modes (jumper PA5) :
 *   - MODE_LED : injecte des trames RGB via le même chemin que le réseau
 *     (dmx_to_ws2815), sortie par WS2815_Show() dans la boucle principale.
 *   - MODE_DMX : écrit directement les 2 ports DMX (DMX_SetSlots/Commit).
 * Un seul test actif à la fois, sur une sortie/port ou toutes.
 * Coupure automatique après TEST_SEQ_TIMEOUT_MS pour ne jamais laisser un
 * node en mode test après un déploiement (voir TestSeq_Task). */

typedef enum {
    TEST_PATTERN_OFF = 0,
    TEST_PATTERN_SOLID,     /* couleur unie / valeur de canal fixe   */
    TEST_PATTERN_CHASE,     /* chenillard                            */
    TEST_PATTERN_RAINBOW,   /* degrade arc-en-ciel (mode LED only)   */
    TEST_PATTERN_FLASH,     /* flash blanc / pleins feux              */
} TestPattern_t;

#define TEST_SEQ_ALL_OUTPUTS  0xFFu
#define TEST_SEQ_TIMEOUT_MS   (10u * 60u * 1000u)  /* 10 min, securite */

/* Démarre un test sur une sortie/port (0..MAX_OUTPUTS-1, ou
 * TEST_SEQ_ALL_OUTPUTS). 'solid_rgb' utilisé seulement pour SOLID en mode
 * LED (packed 0xRRGGBB) ; en mode DMX, SOLID pousse solid_rgb & 0xFF sur
 * tous les canaux. */
void TestSeq_Start(TestPattern_t pattern, uint8_t output, uint32_t solid_rgb);

/* Arrête le test en cours et rend la main au réseau. */
void TestSeq_Stop(void);

/* true si un test est actif (pour bandeau d'alerte UI + statut). */
bool TestSeq_IsActive(void);

/* Info pour l'UI : pattern courant, sortie ciblée, temps restant en ms. */
void TestSeq_GetStatus(TestPattern_t *pattern, uint8_t *output, uint32_t *remaining_ms);

/* À appeler dans la boucle principale (avant l'aiguillage LED/DMX habituel) :
 * fait avancer l'animation et pousse les trames si un test est actif.
 * Retourne true si un test a consommé la trame de ce tour (l'appelant ne
 * doit alors pas laisser le flux réseau normal écraser la sortie). */
bool TestSeq_Task(void);

#endif /* TEST_SEQ_H */
