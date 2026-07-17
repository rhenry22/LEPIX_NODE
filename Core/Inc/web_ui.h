/* web_ui.h
 * Interface web de configuration et monitoring du node (httpd LwIP, raw).
 * Pages générées dynamiquement en RAM (fs_open_custom).
 */
#ifndef WEB_UI_H
#define WEB_UI_H

#include <stdint.h>

/* À appeler une fois après MX_LWIP_Init() (httpd_init + handlers CGI). */
void WebUI_Init(void);

/* ─── Compteurs de monitoring (alimentés par artnet.c) ─── */
typedef struct {
    uint32_t artnet_packets;      /* paquets ArtDmx acceptés            */
    uint32_t artnet_last_ms;      /* HAL_GetTick() de la dernière trame */
    uint16_t artnet_last_universe;/* dernier univers reçu               */
} WebUI_Stats_t;

/* Incrémenté depuis le callback Art-Net (contexte IRQ lwIP). */
void WebUI_NotifyArtnet(uint16_t universe);

/* Copie atomique-ish des compteurs pour affichage. */
void WebUI_GetStats(WebUI_Stats_t *out);

#endif /* WEB_UI_H */
