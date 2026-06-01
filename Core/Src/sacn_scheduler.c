uint32_t sacn_tick(sacn_ctx_t *ctx) {
    uint32_t now = ctx->hal.get_time_ms(ctx->hal.user);
    uint32_t next_ms = 25;  /* cible 40 Hz */

    for (int i = 0; i < SACN_MAX_UNIVERSES; i++) {
        sacn_univ_t *u = &ctx->universes[i];
        if (!u->active_slot) continue;

        bool keepalive = (now - u->last_send_ms) >= 800;
        if (u->dirty || keepalive) {
            _sacn_build_packet(u);
            ctx->hal.udp_send(u->cfg.universe,
                              u->pkt,
                              SACN_PKT_SIZE(SACN_DMX_SLOTS),
                              ctx->hal.user);
            u->last_send_ms = now;
            u->dirty = false;
            u->seq_num++;
        }
    }
    return next_ms;
}