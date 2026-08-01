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
#include "sacn_rx.h"  /* sacn_rx_set_universes (application a chaud) */
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

void WebUI_NotifySacn(uint16_t universe)
{
    s_stats.sacn_packets++;
    s_stats.sacn_last_universe = universe;
    s_stats.sacn_last_ms = HAL_GetTick();
}

void WebUI_GetStats(WebUI_Stats_t *out)
{
    if (!out) return;
    out->artnet_packets       = s_stats.artnet_packets;
    out->artnet_last_ms       = s_stats.artnet_last_ms;
    out->artnet_last_universe = s_stats.artnet_last_universe;
    out->sacn_packets         = s_stats.sacn_packets;
    out->sacn_last_ms         = s_stats.sacn_last_ms;
    out->sacn_last_universe   = s_stats.sacn_last_universe;
}

/* ─────────────────────────────────────────────────────────────────────────
 *  Instantané des canaux DMX (matrice /dmx)
 *  Un slot de 512 canaux par sortie configurée. 2 Ko de RAM.
 * ───────────────────────────────────────────────────────────────────────── */

#define DMX_SNAP_SLOTS 512u

static uint8_t  s_dmx_val[MAX_OUTPUTS][DMX_SNAP_SLOTS];
static uint16_t s_dmx_len[MAX_OUTPUTS];
static uint32_t s_dmx_ms[MAX_OUTPUTS];

void WebUI_NotifyDmxData(uint16_t universe, const uint8_t *data, uint16_t len)
{
    DeviceConfig_t *cfg = Config_Get();
    if (len > DMX_SNAP_SLOTS)
        len = DMX_SNAP_SLOTS;
    for (int i = 0; i < MAX_OUTPUTS; i++) {
        if (!cfg->outputs[i].enabled || cfg->outputs[i].universe != universe)
            continue;
        memcpy(s_dmx_val[i], data, len);
        s_dmx_len[i] = len;
        s_dmx_ms[i]  = HAL_GetTick();
    }
}

/* ─────────────────────────────────────────────────────────────────────────
 *  Génération des pages
 *  Un buffer par fichier ouvert. Le httpd raw no-RTOS ne sert qu'une requête
 *  à la fois dans MX_LWIP_Process() ; on garde néanmoins 2 buffers pour tolérer
 *  deux connexions TCP concurrentes.
 * ───────────────────────────────────────────────────────────────────────── */

#define WEBUI_BUF_SIZE   6144   /* la page /dmx (HTML+JS) approche 4 Ko */
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

/* En-tête HTTP + <head> commun. 'refresh_s' > 0 ajoute un auto-refresh.
 * 'active' surligne l'onglet courant (0=statut,1=flux,2=config). */
static int emit_header(char *b, int cap, const char *title,
                       int refresh_s, int active)
{
    int n = snprintf(b, cap,
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Connection: close\r\n"
        "\r\n"
        "<!doctype html><html><head><meta charset=utf-8>"
        "<meta name=viewport content=\"width=device-width,initial-scale=1\">");
    if (refresh_s > 0)
        n += snprintf(b + n, cap - n,
            "<meta http-equiv=refresh content=%d>", refresh_s);
    n += snprintf(b + n, cap - n,
        "<title>LEPIX Node - %s</title><style>"
        "body{font-family:system-ui,sans-serif;margin:0;background:#0f1115;color:#e8e8e8}"
        "header{background:#1e88e5;padding:14px 20px;font-size:20px;font-weight:600}"
        "nav{background:#161a20;padding:0 12px}"
        "nav a{display:inline-block;color:#9fb4c8;padding:12px 16px;text-decoration:none;border-bottom:3px solid transparent}"
        "nav a.on{color:#fff;border-bottom-color:#1e88e5}"
        "nav a:hover{color:#fff}"
        "main{padding:20px;max-width:820px}"
        "h2{font-size:16px;color:#9fb4c8;margin:22px 0 8px;text-transform:uppercase;letter-spacing:.5px}"
        "table{border-collapse:collapse;width:100%%;margin:6px 0;background:#161a20;border-radius:8px;overflow:hidden}"
        "td,th{border-bottom:1px solid #232a33;padding:9px 12px;text-align:left}"
        "th{background:#1b2028;color:#9fb4c8;font-weight:600}"
        "tr:last-child td{border-bottom:0}"
        "input,select{background:#0f1115;color:#e8e8e8;border:1px solid #2a323d;padding:6px;border-radius:5px}"
        "button{background:#1e88e5;color:#fff;border:0;padding:10px 22px;border-radius:6px;cursor:pointer;font-size:15px;font-weight:600}"
        "button:hover{background:#1976d2}"
        ".ok{color:#4caf50;font-weight:600}.off{color:#78828c}"
        ".pill{display:inline-block;padding:2px 10px;border-radius:20px;font-size:13px;font-weight:600}"
        ".pill.on{background:#14331c;color:#4caf50}.pill.no{background:#2a2020;color:#c86}"
        "</style></head><body>"
        "<header>LEPIX Node</header><nav>"
        "<a href=/ class=%s>Statut</a>"
        "<a href=/flux class=%s>Reception</a>"
        "<a href=/dmx class=%s>Canaux</a>"
        "<a href=/config class=%s>Configuration</a>"
        "</nav><main>",
        title,
        active == 0 ? "on" : "",
        active == 1 ? "on" : "",
        active == 2 ? "on" : "",
        active == 3 ? "on" : "");
    return n;
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

    int n = emit_header(p->buf, WEBUI_BUF_SIZE, "Statut", 0, 0);

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
        "<tr><th>Art-Net</th><td>%lu paquets</td></tr>"
        "<tr><th>sACN</th><td>%lu paquets</td></tr>"
        "</table><p><a href=/flux style=color:#90caf9>Voir la reception en detail &rarr;</a></p>",
        proto_name(cfg->protocol),
        (unsigned long)st.artnet_packets,
        (unsigned long)st.sacn_packets);

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

/* Rend une ligne "flux" : nom, actif/inactif (timeout 5 s), stats. */
static int emit_flux_row(char *b, int cap, const char *name,
                         uint32_t packets, uint16_t last_uni, uint32_t last_ms)
{
    uint32_t ago = HAL_GetTick() - last_ms;
    bool active = (packets > 0) && (ago < 5000);   /* trame < 5 s */
    if (packets == 0) {
        return snprintf(b, cap,
            "<tr><th>%s</th><td><span class='pill no'>INACTIF</span></td>"
            "<td>0</td><td>-</td><td>-</td></tr>", name);
    }
    return snprintf(b, cap,
        "<tr><th>%s</th><td><span class='pill %s'>%s</span></td>"
        "<td>%lu</td><td>%u</td><td>%lu.%lu s</td></tr>",
        name,
        active ? "on" : "no", active ? "ACTIF" : "silence",
        (unsigned long)packets, last_uni,
        (unsigned long)(ago / 1000), (unsigned long)((ago % 1000) / 100));
}

/* Onglet "Reception" : monitoring des flux Art-Net et sACN, auto-refresh 2 s. */
static void build_flux(webui_page_t *p)
{
    WebUI_Stats_t st;
    WebUI_GetStats(&st);

    int n = emit_header(p->buf, WEBUI_BUF_SIZE, "Reception", 2, 1);

    n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n,
        "<h2>Flux entrants</h2><table>"
        "<tr><th>Protocole</th><th>Etat</th><th>Paquets</th>"
        "<th>Dernier univers</th><th>Derniere trame</th></tr>");
    n += emit_flux_row(p->buf + n, WEBUI_BUF_SIZE - n, "Art-Net",
                       st.artnet_packets, st.artnet_last_universe, st.artnet_last_ms);
    n += emit_flux_row(p->buf + n, WEBUI_BUF_SIZE - n, "sACN (E1.31)",
                       st.sacn_packets, st.sacn_last_universe, st.sacn_last_ms);
    n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n,
        "</table><p style=color:#78828c;font-size:13px>"
        "Rafraichissement automatique toutes les 2 s. "
        "Un flux est ACTIF si une trame a ete recue depuis moins de 5 s.</p>"
        "</main></body></html>");

    p->buf[WEBUI_BUF_SIZE - 1] = '\0';
}

/* Onglet "Canaux" : matrice 32x16 des 512 canaux de la sortie choisie.
 * Rendu cote client (canvas) : chaque canal est une cellule en degrade
 * dont l'intensite suit la valeur DMX (0-255). Le JS interroge
 * /dmxdata<i> toutes les 500 ms — la representation pourra changer
 * sans toucher au firmware, seul ce JS est a modifier. */
static void build_dmx(webui_page_t *p)
{
    DeviceConfig_t *cfg = Config_Get();
    int n = emit_header(p->buf, WEBUI_BUF_SIZE, "Canaux DMX", 0, 2);

    n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n,
        "<h2>Matrice des canaux</h2>"
        "<p>Sortie <select id=out>");
    for (int i = 0; i < MAX_OUTPUTS; i++)
        n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n,
            "<option value=%d>%d — univers %u%s</option>",
            i, i + 1, cfg->outputs[i].universe,
            cfg->outputs[i].enabled ? "" : " (off)");
    n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n,
        "</select> <span id=inf class=off></span></p>"
        "<canvas id=cv width=704 height=352 "
        "style=\"width:100%%;background:#0a0c10;border-radius:8px\"></canvas>"
        "<p id=tip class=off>Survoler une cellule pour lire canal et valeur.</p>"
        "<script>\n"
        "const cv=document.getElementById('cv'),cx=cv.getContext('2d');\n"
        "const sel=document.getElementById('out');let last=null;\n"
        "function draw(j){last=j;const W=32,H=16,cw=cv.width/W,ch=cv.height/H;\n"
        " cx.clearRect(0,0,cv.width,cv.height);\n"
        " for(let i=0;i<512;i++){\n"
        "  const v=i<j.len?parseInt(j.hex.substr(i*2,2),16):0;\n"
        "  const x=(i%%W)*cw,y=Math.floor(i/W)*ch;\n"
        "  const g=cx.createLinearGradient(x,y,x,y+ch);\n"
        "  g.addColorStop(0,'rgb('+v+','+Math.round(v*.62)+','+Math.round(v*.18)+')');\n"
        "  g.addColorStop(1,'rgb('+Math.round(v*.35)+','+Math.round(v*.22)+',0)');\n"
        "  cx.fillStyle=g;cx.fillRect(x+1,y+1,cw-2,ch-2);}}\n"
        "async function poll(){try{\n"
        " const r=await fetch('/dmxdata'+sel.value);const j=await r.json();draw(j);\n"
        " document.getElementById('inf').textContent=j.len?\n"
        "  (j.len+' canaux — trame il y a '+(j.age/1000).toFixed(1)+' s'):\n"
        "  'aucune trame recue pour cette sortie';\n"
        "}catch(e){}setTimeout(poll,500);}\n"
        "cv.onmousemove=e=>{if(!last)return;const r=cv.getBoundingClientRect();\n"
        " const cx2=Math.floor((e.clientX-r.left)/r.width*32);\n"
        " const cy=Math.floor((e.clientY-r.top)/r.height*16);\n"
        " const i=cy*32+cx2;if(i<0||i>511)return;\n"
        " const v=i<last.len?parseInt(last.hex.substr(i*2,2),16):0;\n"
        " document.getElementById('tip').textContent='canal '+(i+1)+' = '+v;};\n"
        "poll();\n"
        "</script></main></body></html>");

    p->buf[WEBUI_BUF_SIZE - 1] = '\0';
}

/* /dmxdata<i> : instantane JSON des canaux de la sortie i (hex, 2 c/canal). */
static void build_dmxdata(webui_page_t *p, int idx)
{
    static const char hexd[] = "0123456789abcdef";
    uint16_t len = s_dmx_len[idx];
    uint32_t age = len ? (HAL_GetTick() - s_dmx_ms[idx]) : 0;

    int n = snprintf(p->buf, WEBUI_BUF_SIZE,
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n"
        "{\"out\":%d,\"len\":%u,\"age\":%lu,\"hex\":\"",
        idx + 1, len, (unsigned long)age);
    for (uint16_t i = 0; i < len && n < WEBUI_BUF_SIZE - 8; i++) {
        p->buf[n++] = hexd[s_dmx_val[idx][i] >> 4];
        p->buf[n++] = hexd[s_dmx_val[idx][i] & 0x0F];
    }
    n += snprintf(p->buf + n, WEBUI_BUF_SIZE - n, "\"}");
    p->buf[WEBUI_BUF_SIZE - 1] = '\0';
}

/* Formulaire de configuration. Un seul GET /save reprend tous les champs. */
static void build_config(webui_page_t *p)
{
    DeviceConfig_t *cfg = Config_Get();
    int n = emit_header(p->buf, WEBUI_BUF_SIZE, "Configuration", 0, 3);

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

    /* Application a chaud (sans reboot) : le routage des sorties (enabled,
     * universe, led_count) est relu a chaque trame, donc deja effectif.
     * Seul le sACN doit re-souscrire aux groupes multicast des univers. */
    {
        uint16_t univ[MAX_OUTPUTS];
        uint8_t  nu = 0;
        for (int i = 0; i < MAX_OUTPUTS; i++)
            if (cfg->outputs[i].enabled)
                univ[nu++] = cfg->outputs[i].universe;
        sacn_rx_set_universes(univ, nu);
    }

    /* NB : les changements reseau (IP/masque/passerelle) ne sont PAS
     * appliques a chaud ici — ils necessitent toujours un redemarrage. */

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
    } else if (strcmp(name, "/flux") == 0 || strcmp(name, "/flux.html") == 0) {
        p = page_alloc();
        if (!p) return 0;
        build_flux(p);
    } else if (strcmp(name, "/config") == 0 || strcmp(name, "/config.html") == 0) {
        p = page_alloc();
        if (!p) return 0;
        build_config(p);
    } else if (strcmp(name, "/dmx") == 0 || strcmp(name, "/dmx.html") == 0) {
        p = page_alloc();
        if (!p) return 0;
        build_dmx(p);
    } else if (strncmp(name, "/dmxdata", 8) == 0 &&
               name[8] >= '0' && name[8] < '0' + MAX_OUTPUTS && name[9] == '\0') {
        p = page_alloc();
        if (!p) return 0;
        build_dmxdata(p, name[8] - '0');
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
