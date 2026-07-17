#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ─── Configuration compile-time ──────────────────────────────── */
#ifndef SACN_MAX_UNIVERSES
#  define SACN_MAX_UNIVERSES   8      /* streams simultanés max    */
#endif
#ifndef SACN_DMX_SLOTS
#  define SACN_DMX_SLOTS       512
#endif
#ifndef SACN_DEFAULT_PRIORITY
#  define SACN_DEFAULT_PRIORITY 100
#endif

/* ─── Types opaques ───────────────────────────────────────────── */
typedef struct sacn_ctx   sacn_ctx_t;
typedef struct sacn_univ  sacn_univ_t;

/* ─── HAL : 3 fonctions à implémenter sur votre cible ─────────── */
typedef struct {
    /* Envoie un paquet UDP multicast sur l'univers indiqué.
       Retourne 0 si succès, -1 sinon. */
    int  (*udp_send)(uint16_t universe,
                     const uint8_t *data, uint16_t len,
                     void *user);

    /* Timestamp monotone en millisecondes (wrapping OK). */
    uint32_t (*get_time_ms)(void *user);

    /* Optionnel : mutex (NULL = non thread-safe). */
    void (*lock)(void *user);
    void (*unlock)(void *user);

    void *user;   /* contexte libre (handle socket, etc.) */
} sacn_hal_t;

/* ─── Configuration d'un univers ─────────────────────────────── */
typedef enum {
    SACN_MERGE_HTP = 0,   /* Highest Takes Priority (défaut) */
    SACN_MERGE_LTP,       /* Latest Takes Priority            */
    SACN_MERGE_NONE,      /* Pas de merge, source unique      */
} sacn_merge_mode_t;

typedef struct {
    uint16_t          universe;       /* 1 – 63999              */
    uint8_t           priority;       /* 0 – 200                */
    sacn_merge_mode_t merge_mode;
    bool              preview_bit;    /* données de prévisualisation */
    uint8_t           cid[16];        /* UUID E1.31 (généré si NULL) */
    const char       *source_name;    /* 64 chars max, tronqué  */
} sacn_universe_cfg_t;

/* ─── Lifecycle ───────────────────────────────────────────────── */

/* Initialise le contexte sur un buffer statique fourni.
   buffer_size = sizeof(sacn_ctx_t) fourni par SACN_CTX_SIZE.    */
#define SACN_CTX_SIZE  (sizeof(sacn_ctx_t))

sacn_ctx_t *sacn_init(void *buffer, size_t buffer_size,
                       const sacn_hal_t *hal);
void        sacn_destroy(sacn_ctx_t *ctx);

/* ─── Gestion des univers ─────────────────────────────────────── */
sacn_univ_t *sacn_add_universe(sacn_ctx_t *ctx,
                                const sacn_universe_cfg_t *cfg);
void         sacn_remove_universe(sacn_ctx_t *ctx,
                                  sacn_univ_t *univ);

/* ─── Écriture des données DMX ───────────────────────────────── */

/* Écrit dans le back-buffer (sans envoi immédiat). */
int  sacn_set_slots(sacn_univ_t *univ,
                    uint16_t start_slot,   /* 0-based          */
                    const uint8_t *data,
                    uint16_t count);

int  sacn_set_slot(sacn_univ_t *univ,
                   uint16_t slot, uint8_t value);

/* Prépare l'envoi (swap des buffers). */
void sacn_commit(sacn_univ_t *univ);

/* ─── Scheduler ───────────────────────────────────────────────── */

/* À appeler depuis votre boucle principale ou une tâche RTOS.
   Envoie les paquets en attente + keep-alive E1.31 (~25 ms).
   Retourne le délai en ms avant le prochain appel recommandé.   */
uint32_t sacn_tick(sacn_ctx_t *ctx);

/* Envoi immédiat d'un univers (bypass scheduler). */
int sacn_flush(sacn_ctx_t *ctx, sacn_univ_t *univ);

/* ─── Statistiques (debug) ────────────────────────────────────── */
typedef struct {
    uint32_t packets_sent;
    uint32_t send_errors;
    uint32_t seq_overflows;
    uint32_t universes_active;
} sacn_stats_t;

void sacn_get_stats(const sacn_ctx_t *ctx, sacn_stats_t *out);