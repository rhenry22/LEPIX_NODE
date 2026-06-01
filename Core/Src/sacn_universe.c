/* Technique : index de buffer actif dans un uint8_t atomique.
   Convient pour bare-metal (pas de preemption entre set et commit)
   ou FreeRTOS avec section critique. */

typedef struct sacn_univ {
    sacn_universe_cfg_t cfg;
    uint8_t  buf[2][SACN_DMX_SLOTS]; /* double buffer           */
    uint8_t  active;                 /* index du back-buffer    */
    uint8_t  seq_num;                /* wrapping 0-255 E1.31    */
    uint32_t last_send_ms;
    bool     dirty;                  /* données modifiées       */
    uint8_t  pkt[SACN_PKT_SIZE(SACN_DMX_SLOTS)]; /* buffer TX  */
} sacn_univ_t;

/* Lecture toujours sur front-buffer (1 - active),
   écriture toujours sur back-buffer (active).
   sacn_commit() swape atomiquement les deux. */

void sacn_commit(sacn_univ_t *univ) {
    univ->active ^= 1;
    univ->dirty   = true;
}