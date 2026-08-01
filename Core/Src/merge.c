/* merge.c — merge HTP multi-sources par univers. */

#include "merge.h"
#include "main.h"   /* HAL_GetTick */
#include <string.h>

typedef struct {
    uint8_t  id[MERGE_SRCID_LEN];
    uint8_t  data[MERGE_SLOTS];
    uint16_t len;
    uint32_t last_ms;
    uint8_t  active;
} merge_source_t;

typedef struct {
    uint16_t       universe;      /* 0 = slot libre                     */
    uint8_t        active;
    merge_source_t sources[MERGE_MAX_SOURCES];
    uint8_t        merged[MERGE_SLOTS];
} merge_universe_t;

static merge_universe_t s_univ[MERGE_MAX_UNIVERSES];
static merge_out_cb_t   s_out = NULL;

void Merge_SetCallback(merge_out_cb_t cb) { s_out = cb; }

/* Trouve (ou alloue) l'entrée d'un univers. NULL si plus de place. */
static merge_universe_t *univ_get(uint16_t universe)
{
    merge_universe_t *libre = NULL;
    for (int i = 0; i < MERGE_MAX_UNIVERSES; i++) {
        if (s_univ[i].active && s_univ[i].universe == universe)
            return &s_univ[i];
        if (!s_univ[i].active && libre == NULL)
            libre = &s_univ[i];
    }
    if (libre) {
        memset(libre, 0, sizeof(*libre));
        libre->universe = universe;
        libre->active   = 1;
    }
    return libre;
}

/* Trouve (ou alloue) la source dans un univers. Retire au passage les
 * sources expirées. NULL si plus de place. */
static merge_source_t *source_get(merge_universe_t *u,
                                   const uint8_t id[MERGE_SRCID_LEN],
                                   uint32_t now)
{
    merge_source_t *libre = NULL;
    for (int i = 0; i < MERGE_MAX_SOURCES; i++) {
        merge_source_t *s = &u->sources[i];
        if (s->active && (now - s->last_ms) >= MERGE_SOURCE_TIMEOUT_MS)
            s->active = 0;   /* expiration */
        if (s->active && memcmp(s->id, id, MERGE_SRCID_LEN) == 0)
            return s;
        if (!s->active && libre == NULL)
            libre = s;
    }
    if (libre) {
        memset(libre, 0, sizeof(*libre));
        memcpy(libre->id, id, MERGE_SRCID_LEN);
        libre->active = 1;
    }
    return libre;
}

/* Compte les sources actives (non expirées) d'un univers. */
static int active_source_count(merge_universe_t *u, uint32_t now)
{
    int c = 0;
    for (int i = 0; i < MERGE_MAX_SOURCES; i++) {
        merge_source_t *s = &u->sources[i];
        if (s->active && (now - s->last_ms) < MERGE_SOURCE_TIMEOUT_MS)
            c++;
    }
    return c;
}

void Merge_Submit(uint16_t universe, const uint8_t src_id[MERGE_SRCID_LEN],
                  const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return;
    if (len > MERGE_SLOTS) len = MERGE_SLOTS;

    uint32_t now = HAL_GetTick();

    merge_universe_t *u = univ_get(universe);
    if (u == NULL) {
        /* Table pleine : on route la trame telle quelle (pas de merge). */
        if (s_out) s_out(universe, (uint8_t *)data, len);
        return;
    }

    merge_source_t *s = source_get(u, src_id, now);
    if (s == NULL) {
        if (s_out) s_out(universe, (uint8_t *)data, len);
        return;
    }

    /* Met à jour la source. */
    memcpy(s->data, data, len);
    if (len < s->len)                    /* efface la queue si trame plus courte */
        memset(&s->data[len], 0, s->len - len);
    s->len     = len;
    s->last_ms = now;

    /* Cas fréquent : une seule source -> pas de merge, on route direct
     * (evite le cout du HTP quand il n'y a pas de redondance). */
    if (active_source_count(u, now) <= 1) {
        if (s_out) s_out(universe, s->data, s->len);
        return;
    }

    /* Merge HTP : pour chaque canal, max sur toutes les sources actives. */
    uint16_t merged_len = 0;
    memset(u->merged, 0, sizeof(u->merged));
    for (int i = 0; i < MERGE_MAX_SOURCES; i++) {
        merge_source_t *src = &u->sources[i];
        if (!src->active || (now - src->last_ms) >= MERGE_SOURCE_TIMEOUT_MS)
            continue;
        if (src->len > merged_len) merged_len = src->len;
        for (uint16_t c = 0; c < src->len; c++)
            if (src->data[c] > u->merged[c])
                u->merged[c] = src->data[c];
    }

    if (s_out) s_out(universe, u->merged, merged_len);
}
