/* artnet.c
 * Réception des univers Art-Net via LwIP (mode RAW, sans FreeRTOS)
 * IP configurable (défaut statique 2.0.0.2/24, voir config.h)
 * Debug sur USART1 (PA9=TX, PA10=RX) à 115200 bauds
 *
 * À inclure dans Core/Src/ et appeler depuis main.c
 */

#include "artnet.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include <string.h>
#include <stdio.h>
#include "ws2815.h"
#include "web_ui.h"
#include "merge.h"
#include "log_capture.h"

extern struct netif gnetif;

/* ------------------------------------------------------------------ */
/*  Defines Art-Net                                                     */
/* ------------------------------------------------------------------ */
#define ARTNET_PORT         6454
#define ARTNET_ID           "Art-Net"
#define ARTNET_OPCODE_DMX   0x5000   /* OpDmx (little-endian : 0x00 0x50) */
#define ARTNET_DMX_CHANNELS 512

/* ------------------------------------------------------------------ */
/*  Handle UART debug (déclaré extern dans main.c par CubeMX)          */
/* ------------------------------------------------------------------ */
extern UART_HandleTypeDef huart1;

/* ------------------------------------------------------------------ */
/*  Callback utilisateur DMX                                            */
/* ------------------------------------------------------------------ */
static artnet_dmx_cb_t s_dmx_cb = NULL;

void artnet_set_callback(artnet_dmx_cb_t cb)
{
    s_dmx_cb = cb;
}

/* ------------------------------------------------------------------ */
/*  Tampon de debug UART                                               */
/* ------------------------------------------------------------------ */
static char dbg_buf[128];

static void debug_print(const char *msg)
{
    HAL_UART_Transmit(&huart1,
                      (uint8_t *)msg,
                      (uint16_t)strlen(msg),
                      100);
}

static void debug_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(dbg_buf, sizeof(dbg_buf), fmt, args);
    va_end(args);
    debug_print(dbg_buf);
}

/* ------------------------------------------------------------------ */
/*  Structure paquet Art-Net OpDmx (simplifié)                         */
/* ------------------------------------------------------------------ */
typedef struct __attribute__((packed)) {
    char     id[8];          /* "Art-Net\0"                */
    uint16_t opcode;         /* 0x0050 little-endian       */
    uint8_t  prot_hi;        /* Protocol version Hi = 0    */
    uint8_t  prot_lo;        /* Protocol version Lo = 14   */
    uint8_t  sequence;       /* 0 = désactivé              */
    uint8_t  physical;       /* Port physique source        */
    uint8_t  sub_uni;        /* Universe (bits 0-3)        */
    uint8_t  net;            /* Net (bits 8-14)            */
    uint16_t length;         /* Longueur DMX (big-endian)  */
    uint8_t  data[ARTNET_DMX_CHANNELS];
} ArtDmx_t;

/* ------------------------------------------------------------------ */
/*  Callback LwIP UDP                                                   */
/* ------------------------------------------------------------------ */
static void artnet_recv_cb(void *arg,
                           struct udp_pcb *pcb,
                           struct pbuf *p,
                           const ip_addr_t *addr,
                           u16_t port)
{
    (void)arg;
    (void)pcb;
    (void)port;
    if (p == NULL)
        return;

    /* Vérification taille minimale (header = 18 octets) */
    if (p->tot_len < 18)
        goto done;

    /* Copie dans un buffer local pour analyse */
    uint8_t buf[sizeof(ArtDmx_t)];
    uint16_t copy_len = p->tot_len < sizeof(buf) ? p->tot_len : sizeof(buf);
    pbuf_copy_partial(p, buf, copy_len, 0);

    ArtDmx_t *pkt = (ArtDmx_t *)buf;

    /* Vérification de l'ID "Art-Net" */
    if (memcmp(pkt->id, ARTNET_ID, 7) != 0)
        goto done;

    /* Vérification OpCode OpDmx (0x5000 little-endian) */
    if (pkt->opcode != ARTNET_OPCODE_DMX)
        goto done;

    /* Calcul de l'univers Art-Net complet (0..32767) — utilisé pour le routage */
    uint16_t universe = (uint16_t)(pkt->net << 8) | pkt->sub_uni;

    /* Longueur DMX (big-endian dans le paquet), bornée par ce qui a
     * réellement été reçu (18 octets de header avant les données) */
    uint16_t dmx_len = (uint16_t)((pkt->length >> 8) | (pkt->length << 8));
    if (dmx_len > (uint16_t)(copy_len - 18))
        dmx_len = (uint16_t)(copy_len - 18);

    /* ---- Monitoring (interface web) ---- */
    WebUI_NotifyArtnet(universe);

    /* Commande CLI "log" : capture sur donnees brutes, avant fusion HTP. */
    LogCapture_OnFrame(LOG_SRC_ARTNET, universe, pkt->data, dmx_len);

    /* ---- Merge HTP : source identifiee par l'IP emettrice (4 octets +
     *      zeros pour completer les 16 octets d'identifiant). ---- */
    uint8_t src_id[MERGE_SRCID_LEN] = {0};
    if (addr != NULL) {
        uint32_t ip = ip4_addr_get_u32(ip_2_ip4(addr));  /* network order */
        memcpy(src_id, &ip, 4);
    }
    Merge_Submit(universe, src_id, pkt->data, dmx_len);

done:
    pbuf_free(p);
}

/* ------------------------------------------------------------------ */
/*  Initialisation                                                      */
/* ------------------------------------------------------------------ */
void artnet_init(void)
{
    struct udp_pcb *pcb = udp_new();
    if (pcb == NULL) {
        debug_print("[ArtNet] ERREUR: impossible de créer le PCB UDP\r\n");
        return;
    }

    err_t err = udp_bind(pcb, IP_ADDR_ANY, ARTNET_PORT);
    if (err != ERR_OK) {
        debug_printf("[ArtNet] ERREUR: bind port %u (err=%d)\r\n",
                     ARTNET_PORT, err);
        udp_remove(pcb);
        return;
    }

    udp_recv(pcb, artnet_recv_cb, NULL);

    debug_printf("[ArtNet] Ecoute sur %s:" ARTNET_PORT_STR "\r\n",
                 ip4addr_ntoa(netif_ip4_addr(&gnetif)));
}