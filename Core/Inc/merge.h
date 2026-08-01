/* merge.h
 * Merge HTP (Highest Takes Priority) multi-sources par univers.
 *
 * En live, deux consoles peuvent emettre le meme univers (redondance,
 * multi-operateurs). Le merge combine leurs trames canal par canal en
 * gardant la valeur la plus elevee, et delivre le resultat au routage.
 *
 * Chaque source est identifiee par un identifiant de 16 octets :
 *   - sACN : le CID (E1.31, 16 octets)
 *   - Art-Net : l'IP source (4 octets) completee de zeros
 *
 * Une source silencieuse au-dela de MERGE_SOURCE_TIMEOUT_MS est retiree.
 */
#ifndef MERGE_H
#define MERGE_H

#include <stdint.h>

/* ─── Paramètres configurables ─────────────────────────────────────────── */
#ifndef MERGE_MAX_UNIVERSES
#  define MERGE_MAX_UNIVERSES        4   /* univers mergés simultanément   */
#endif
#ifndef MERGE_MAX_SOURCES
#  define MERGE_MAX_SOURCES          4   /* sources max par univers        */
#endif
#ifndef MERGE_SOURCE_TIMEOUT_MS
#  define MERGE_SOURCE_TIMEOUT_MS 2500u  /* expiration d'une source        */
#endif

#define MERGE_SLOTS        512
#define MERGE_SRCID_LEN     16

/* Callback recevant la trame MERGÉE (HTP) d'un univers. */
typedef void (*merge_out_cb_t)(uint16_t universe, uint8_t *data, uint16_t len);

/* Arme le callback de sortie (le routage réel : dmx_to_ws2815). */
void Merge_SetCallback(merge_out_cb_t cb);

/* Soumet une trame reçue d'une source. Met à jour le merge de l'univers
 * et invoque le callback de sortie avec le résultat HTP.
 *   src_id : 16 octets identifiant la source (CID ou IP+zeros)
 * Appelé depuis artnet.c / sacn_rx.c (contexte lwIP). */
void Merge_Submit(uint16_t universe, const uint8_t src_id[MERGE_SRCID_LEN],
                  const uint8_t *data, uint16_t len);

#endif /* MERGE_H */
