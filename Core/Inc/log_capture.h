#ifndef LOG_CAPTURE_H
#define LOG_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>

/* Capture ponctuelle d'une trame sACN et d'une trame Art-Net, pour la
 * commande CLI "log" : identifier ce qui est reellement envoye par les
 * sources reseau (univers, longueur, premiers octets).
 *
 * Usage :
 *   1. LogCapture_Arm() (depuis la commande CLI "log")
 *   2. A la prochaine trame de chaque protocole, LogCapture_OnFrame() est
 *      appelee par artnet.c / sacn_rx.c (avant Merge_Submit, donc sur les
 *      donnees brutes recues, pas la version fusionnee HTP).
 *   3. LogCapture_Task() (boucle principale) imprime la trame capturee et
 *      desarme automatiquement — impression hors contexte reseau/IRQ.
 */

typedef enum {
    LOG_SRC_SACN = 0,
    LOG_SRC_ARTNET,
    LOG_SRC_COUNT,
} LogSource_t;

#define LOG_CAPTURE_MAX_BYTES 32u   /* premiers octets affiches, pas la trame entiere */

/* Arme la capture des deux sources (sACN + Art-Net). Idempotent : un
 * nouvel appel reinitialise l'armement (utile si rien n'a ete recu). */
void LogCapture_Arm(void);

/* true si la capture d'au moins une source est encore armee (en attente). */
bool LogCapture_IsArmed(void);

/* Appelee depuis artnet.c / sacn_rx.c avec les donnees brutes recues,
 * avant fusion HTP. No-op si cette source n'est pas armee. Contexte
 * reseau (IRQ lwIP) : ne fait que copier, aucun printf ici. */
void LogCapture_OnFrame(LogSource_t src, uint16_t universe,
                        const uint8_t *data, uint16_t len);

/* A appeler dans la boucle principale : imprime (printf, donc sur la
 * console CLI USART1) toute capture fraichement arrivee, puis la
 * consomme. Non bloquant. */
void LogCapture_Task(void);

#endif /* LOG_CAPTURE_H */
