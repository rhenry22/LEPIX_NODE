/* web_ui.c
 * Interface web de configuration/monitoring — httpd LwIP en mode raw (no-RTOS).
 *
 * Les pages sont générées à la volée dans un buffer RAM par fs_open_custom() :
 *   GET /            -> page de statut (monitoring)
 *   GET /config      -> formulaire de configuration
 *   GET /save?...    -> CGI : applique la config puis redirige vers /config
 *
 * La config est appliquée en RAM (Config_Get) et sauvée via Config_Save().
 * Les changements réseau ne prennent effet qu'au redémarrage (LwIP n'est pas
 * réinitialisé à chaud dans cette version).
 */

#include "web_ui.h"
#include "config.h"
#include "ws2815.h"   /* WS2815_MAX_LEDS */
#include "lwip/apps/httpd.h"
#include "lwip/apps/fs.h"
#include "lwip/netif.h"
#include "lwip/init.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern struct netif gnetif;

/* ─────────────────────────────────────────────────────────────────────────
 *  Monitoring
 * ───────────────────────────────────────────────────────────────────────── */

static volatile WebUI_Stats_t s_stats = {0};

/* HAL_GetTick() est fourni par le HAL ; déclaré ici pour éviter d'inclure
 * tout main.h dans un contexte lwIP. */
extern uint32_t HAL_GetTick(void);

void WebUI_NotifyArtnet(uint16_t universe)
{
    s_stats.artnet_packets++;
    s_stats.artnet_last_universe = universe;
    s_stats.artnet_last_ms = HAL_GetTick();
}

void WebUI_GetStats(WebUI_Stats_t *out)
{
    if (!out) return;
    out->artnet_packets       = s_stats.artnet_packets;
    out->artnet_last_ms       = s_stats.artnet_last_ms;
    out->artnet_last_universe = s_stats.artnet_last_universe;
}

/* ─────────────────────────────────────────────────────────────────────────
 *  Génération des pages
 *  Un buffer par fichier ouvert. Le httpd raw no-RTOS ne sert qu'une requête
 *  à la fois dans MX_LWIP_Process() ; on garde néanmoins 2 buffers pour tolérer
 *  deux connexions TCP concurrentes.
 * ───────────────────────────────────────────────────────────────────────── */

#define WEBUI_BUF_SIZE   3072
#define WEBUI_NUM_BUFS   2

typedef struct {
    int    used;
    char   buf[WEBUI_BUF_SIZE];
} webui_page_t;

static webui_page_t s_pages[WEBUI_NUM_BUFS];

static webui_page_t *page_alloc(void)
{
    for (int i = 0; i < WEBUI_NUM_BUFS; i++) {
        if (!s_pages[i].used) {
            s_pages[i].used = 1;
            return &s_pages[i];
        }
    }
    return NULL;
}

static void page_free(webui_page_t *p)
{
    if (p) p->used = 0;
}

static const char *proto_name(InputProtocol_t p)
{
    switch (p) {
        case PROTO_ARTNET:   return "Art-Net";
        case PROTO_SACN:     return "sACN";
        case PROTO_DMX_UART: return "DMX (UART)";
        default:             return "?";
    }
}

/* En-tête HTTP + <head> commun. Retourne le nombre d'octets écrits. */
static int emit_header(char *b, int cap, const char *title)
{
    return snprintf(b, cap,
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Connection: close\r\n"
        "\r\n"
        "<!doctype html><html><head><meta charset=utf-8>"
        "<meta name=viewport content=\"width=device-width,initial-scale=1\">"
        "<title>LEPIX Node - %s</title><style>"
        "body{font-family:system-ui,sans-serif;margin:0;background:#111;color:#eee}"
        "header{background:#1e88e5;padding:14px 20px;font-size:20px;font-weight:600}"
        "nav a{color:#90caf9;margin-right:16px;text-decoration:none}"
        "main{padding:20px;max-width:760px}"
        "table{border-collapse:collapse;width:100%%;margin:10px 0}"
        "td,th{border:1px solid #333;padding:6px 10px;text-align:left}"
        "th{background:#1a1a1a}"
        "input,select{background:#222;color:#eee;border:1px solid #444;padding:4px;border-radius:4px}"
        "button{background:#1e88e5;color:#fff;border:0;padding:8px 18px;border-radius:4px;cursor:pointer;font-size:15px}"
        ".ok{color:#66bb6a}.off{color:#888}"
        "</style></head><body>"
        "<header>LEPIX Node</header>"
        "<nav style=padding:10px20px><a href=/>Statut</a><a href=/config>Configuration</a></nav>"
        "<main>", title);
}

/* Page de statut / monitoring. */
static void build_status(webui_page_t *p)
{
    DeviceConfig_t *cfg = Config_Get();
    WebUI_Stats_t st;
    WebUI_GetStats(&st);

    const ip4_addr_t *ip = netif_ip4_addr(&gnetif);
    const ip4_addr_t *nm = netif_ip4_netmask(&gnetif);
    const ip4_addr_t *gw = netif_ip4_gw(&gnetif);
    int link = netif_is_link_up(&gnetif);

    uint32_t ago = HAL_GetTick() - st.artnet_last_ms;

    int n = emit_header(p->buf, WEBUI_BUF_SIZE, "Statut");

    n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n,
        "<h2>Reseau</h2><table>"
        "<tr><th>Lien</th><td class=%s>%s</td></tr>"
        "<tr><th>Mode</th><td>%s</td></tr>"
        "<tr><th>IP</th><td>%s</td></tr>"
        "<tr><th>Masque</th><td>%s</td></tr>"
        "<tr><th>Passerelle</th><td>%s</td></tr>"
        "</table>",
        link ? "ok" : "off", link ? "UP" : "DOWN",
        cfg->net_mode == NET_DHCP ? "DHCP" : "Statique",
        ip4addr_ntoa(ip),
        ip4addr_ntoa(nm),
        ip4addr_ntoa(gw));

    n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n,
        "<h2>Entree</h2><table>"
        "<tr><th>Protocole</th><td>%s</td></tr>"
        "<tr><th>Paquets Art-Net</th><td>%lu</td></tr>"
        "<tr><th>Dernier univers</th><td>%u</td></tr>"
        "<tr><th>Derniere trame</th><td>%s</td></tr>"
        "</table>",
        proto_name(cfg->protocol),
        (unsigned long)st.artnet_packets,
        st.artnet_last_universe,
        st.artnet_packets ? "" : "aucune");
    /* Affichage du délai depuis la dernière trame (si au moins une reçue) */
    if (st.artnet_packets) {
        n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n,
            "<p>Il y a %lu.%lu s</p>",
            (unsigned long)(ago / 1000), (unsigned long)((ago % 1000) / 100));
    }

    n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n, "<h2>Sorties</h2><table>"
        "<tr><th>#</th><th>Etat</th><th>Univers</th><th>LEDs</th></tr>");
    for (int i = 0; i < MAX_OUTPUTS; i++) {
        OutputConfig_t *o = &cfg->outputs[i];
        n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n,
            "<tr><td>%d</td><td class=%s>%s</td><td>%u</td><td>%u</td></tr>",
            i + 1,
            o->enabled ? "ok" : "off",
            o->enabled ? "ON" : "off",
            o->universe, o->led_count);
    }
    n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n, "</table></main></body></html>");

    p->buf[WEBUI_BUF_SIZE - 1] = '\0';
}

/* Formulaire de configuration. Un seul GET /save reprend tous les champs. */
static void build_config(webui_page_t *p)
{
    DeviceConfig_t *cfg = Config_Get();
    int n = emit_header(p->buf, WEBUI_BUF_SIZE, "Configuration");

    n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n,
        "<form action=/save method=get>"
        "<h2>Reseau</h2><table>"
        "<tr><th>Mode</th><td><select name=nm>"
        "<option value=0 %s>DHCP</option>"
        "<option value=1 %s>Statique</option></select></td></tr>"
        "<tr><th>IP</th><td><input name=ip value=%u.%u.%u.%u></td></tr>"
        "<tr><th>Masque</th><td><input name=mk value=%u.%u.%u.%u></td></tr>"
        "<tr><th>Passerelle</th><td><input name=gw value=%u.%u.%u.%u></td></tr>"
        "</table>",
        cfg->net_mode == NET_DHCP   ? "selected" : "",
        cfg->net_mode == NET_STATIC ? "selected" : "",
        cfg->ip[0], cfg->ip[1], cfg->ip[2], cfg->ip[3],
        cfg->netmask[0], cfg->netmask[1], cfg->netmask[2], cfg->netmask[3],
        cfg->gateway[0], cfg->gateway[1], cfg->gateway[2], cfg->gateway[3]);

    n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n,
        "<h2>Protocole</h2><select name=pr>"
        "<option value=0 %s>Art-Net</option>"
        "<option value=1 %s>sACN</option>"
        "<option value=2 %s>DMX (UART)</option></select>",
        cfg->protocol == PROTO_ARTNET   ? "selected" : "",
        cfg->protocol == PROTO_SACN     ? "selected" : "",
        cfg->protocol == PROTO_DMX_UART ? "selected" : "");

    n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n, "<h2>Sorties</h2><table>"
        "<tr><th>#</th><th>Active</th><th>Univers</th><th>LEDs</th></tr>");
    for (int i = 0; i < MAX_OUTPUTS; i++) {
        OutputConfig_t *o = &cfg->outputs[i];
        n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n,
            "<tr><td>%d</td>"
            "<td><input type=checkbox name=e%d %s></td>"
            "<td><input name=u%d value=%u size=6></td>"
            "<td><input name=l%d value=%u size=6></td></tr>",
            i + 1,
            i, o->enabled ? "checked" : "",
            i, o->universe,
            i, o->led_count);
    }
    n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n,
        "</table><p><button type=submit>Enregistrer</button></p>"
        "</form></main></body></html>");

    p->buf[WEBUI_BUF_SIZE - 1] = '\0';
}

/* ─────────────────────────────────────────────────────────────────────────
 *  CGI : GET /save?nm=..&ip=..&...
 * ───────────────────────────────────────────────────────────────────────── */

static const char *find_param(int n, char *keys[], char *vals[], const char *k)
{
    for (int i = 0; i < n; i++)
        if (keys[i] && strcmp(keys[i], k) == 0)
            return vals[i];
    return NULL;
}

static void parse_ip4(const char *s, uint8_t out[4])
{
    unsigned a = 0, b = 0, c = 0, d = 0;
    if (s && sscanf(s, "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
        out[0] = (uint8_t)a; out[1] = (uint8_t)b;
        out[2] = (uint8_t)c; out[3] = (uint8_t)d;
    }
}

static const char *cgi_save(int index, int n, char *keys[], char *vals[])
{
    (void)index;
    DeviceConfig_t *cfg = Config_Get();
    const char *v;
    char key[4];

    if ((v = find_param(n, keys, vals, "nm")) != NULL)
        cfg->net_mode = (atoi(v) == 1) ? NET_STATIC : NET_DHCP;
    if ((v = find_param(n, keys, vals, "ip")) != NULL) parse_ip4(v, cfg->ip);
    if ((v = find_param(n, keys, vals, "mk")) != NULL) parse_ip4(v, cfg->netmask);
    if ((v = find_param(n, keys, vals, "gw")) != NULL) parse_ip4(v, cfg->gateway);
    if ((v = find_param(n, keys, vals, "pr")) != NULL)
        cfg->protocol = (InputProtocol_t)atoi(v);

    for (int i = 0; i < MAX_OUTPUTS; i++) {
        /* Case à cocher : présente => cochée, absente => décochée */
        snprintf(key, sizeof(key), "e%d", i);
        cfg->outputs[i].enabled = (find_param(n, keys, vals, key) != NULL);

        snprintf(key, sizeof(key), "u%d", i);
        if ((v = find_param(n, keys, vals, key)) != NULL)
            cfg->outputs[i].universe = (uint16_t)atoi(v);

        snprintf(key, sizeof(key), "l%d", i);
        if ((v = find_param(n, keys, vals, key)) != NULL) {
            int leds = atoi(v);
            if (leds < 0) leds = 0;
            if (leds > WS2815_MAX_LEDS) leds = WS2815_MAX_LEDS;
            cfg->outputs[i].led_count = (uint16_t)leds;
        }
    }

    Config_Save();  /* persiste sur SD (no-op si SD absente) */

    /* Redirection vers la page de config (rechargée avec les nouvelles valeurs) */
    return "/config";
}

static const tCGI s_cgis[] = {
    { "/save", cgi_save },
};

/* ─────────────────────────────────────────────────────────────────────────
 *  Hooks « custom files » du httpd
 * ───────────────────────────────────────────────────────────────────────── */

int fs_open_custom(struct fs_file *file, const char *name)
{
    webui_page_t *p = NULL;

    /* Le httpd passe le chemin sans query-string pour l'ouverture de fichier.
     * /save est traité par le CGI ; ici on ne gère que les pages affichables. */
    if (strcmp(name, "/") == 0 || strcmp(name, "/index.html") == 0) {
        p = page_alloc();
        if (!p) return 0;
        build_status(p);
    } else if (strcmp(name, "/config") == 0 || strcmp(name, "/config.html") == 0) {
        p = page_alloc();
        if (!p) return 0;
        build_config(p);
    } else {
        return 0;   /* non géré -> 404 */
    }

    file->data   = p->buf;
    file->len    = (int)strlen(p->buf);
    file->index  = file->len;   /* tout est déjà en mémoire */
    file->flags  = FS_FILE_FLAGS_HEADER_INCLUDED; /* on fournit l'en-tête HTTP */
    file->state  = p;           /* pour libérer le buffer au close */
    file->pextension = NULL;
    return 1;
}

void fs_close_custom(struct fs_file *file)
{
    if (file && file->state)
        page_free((webui_page_t *)file->state);
}

int fs_read_custom(struct fs_file *file, char *buffer, int count)
{
    (void)file; (void)buffer; (void)count;
    /* Toute la donnée est déjà pointée par file->data (index == len),
     * donc le httpd n'appelle jamais read : EOF. */
    return FS_READ_EOF;
}

/* fs_state_init/free : requis par LWIP_HTTPD_FILE_STATE, mais notre state
 * est posé directement dans fs_open_custom. On fournit des stubs neutres. */
void *fs_state_init(struct fs_file *file, const char *name)
{
    (void)name;
    return file ? file->state : NULL;
}

void fs_state_free(struct fs_file *file, void *state)
{
    (void)file; (void)state;
    /* La libération réelle a lieu dans fs_close_custom. */
}

/* ─────────────────────────────────────────────────────────────────────────
 *  Init
 * ───────────────────────────────────────────────────────────────────────── */

void WebUI_Init(void)
{
    httpd_init();
    http_set_cgi_handlers(s_cgis, LWIP_ARRAYSIZE(s_cgis));
}
