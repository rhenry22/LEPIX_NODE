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
#include "config.h"

extern struct netif gnetif;

/* ------------------------------------------------------------------ */
/*  Defines Art-Net                                                     */
/* ------------------------------------------------------------------ */
#define ARTNET_PORT         6454
#define ARTNET_ID           "Art-Net"
#define ARTNET_OPCODE_DMX   0x5000   /* OpDmx (little-endian : 0x00 0x50)   */
#define ARTNET_OPCODE_POLL  0x2000   /* OpPoll (little-endian : 0x00 0x20)  */
#define ARTNET_DMX_CHANNELS 512

/* ArtPollReply : taille et identite du node annoncees aux controleurs. */
#define ARTNET_POLLREPLY_SIZE  239
#define ARTNET_SHORT_NAME      "LEPIX_NODE"
#define ARTNET_LONG_NAME       "LEPIX Art-Net/sACN Node (STM32F407)"

/* PCB conserve pour pouvoir emettre les ArtPollReply. */
static struct udp_pcb *s_pcb = NULL;

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
/*  ArtPollReply : rend le node visible dans les controleurs Art-Net   */
/*  (grandMA, QLC+, Resolume...). Format E1.20 / Art-Net 4, 239 octets. */
/*  Offsets repris de la spec Art-Net (reference : projet pixfrog).     */
/* ------------------------------------------------------------------ */
static void artnet_copy_bounded(uint8_t *dst, int cap, const char *src)
{
    int i = 0;
    if (src) {
        for (; i < cap - 1 && src[i]; i++)
            dst[i] = (uint8_t)src[i];
    }
    for (; i < cap; i++)
        dst[i] = 0;   /* padding + terminaison */
}

static void artnet_send_poll_reply(const ip_addr_t *dst_addr, u16_t dst_port)
{
    if (s_pcb == NULL)
        return;

    uint8_t pkt[ARTNET_POLLREPLY_SIZE];
    memset(pkt, 0, sizeof(pkt));

    /* IP locale + MAC depuis le netif. */
    const ip4_addr_t *ip4 = netif_ip4_addr(&gnetif);
    uint32_t ip_be = ip4_addr_get_u32(ip4);   /* deja en network order */
    const uint8_t *mac = gnetif.hwaddr;

    /* Univers annonces : ceux des 4 sorties (nibble bas de sub_uni). */
    DeviceConfig_t *cfg = Config_Get();

    memcpy(pkt, ARTNET_ID, 8);          /* "Art-Net\0"                       */
    pkt[8]  = 0x00; pkt[9] = 0x21;      /* OpPollReply (0x2100 LE)           */
    memcpy(pkt + 10, &ip_be, 4);        /* IP du node                        */
    pkt[14] = 0x36; pkt[15] = 0x19;     /* port 6454 (LE)                    */
    pkt[16] = 0x00; pkt[17] = 0x01;     /* VersInfo H/L                      */
    pkt[18] = 0x00;                     /* NetSwitch                         */
    pkt[19] = 0x00;                     /* SubSwitch                         */
    pkt[20] = 0x00; pkt[21] = 0x00;     /* Oem (inconnu)                     */
    pkt[22] = 0x00;                     /* UbeaVersion                       */
    pkt[23] = 0xD0;                     /* Status1 : indicateurs normaux     */
    pkt[24] = 0xFF; pkt[25] = 0xFF;     /* EstaMan = 0xFFFF (inconnu)        */

    artnet_copy_bounded(pkt + 26,  18, ARTNET_SHORT_NAME);
    artnet_copy_bounded(pkt + 44,  64, ARTNET_LONG_NAME);
    artnet_copy_bounded(pkt + 108, 64, "OK");   /* NodeReport                */

    pkt[172] = 0x00; pkt[173] = 0x04;   /* NumPorts = 4                      */
    for (uint8_t pnum = 0; pnum < 4; pnum++) {
        if (cfg->outputs[pnum].enabled) {
            pkt[174 + pnum] = 0x80;     /* PortType : sortie DMX512          */
            pkt[182 + pnum] = 0x80;     /* GoodOutputA : emission active     */
        }
        pkt[190 + pnum] = (uint8_t)(cfg->outputs[pnum].universe & 0x0F); /* SwOut */
    }
    pkt[200] = 0x00;                    /* Style : StNode                    */

    memcpy(pkt + 201, mac, 6);          /* MAC                               */
    memcpy(pkt + 207, &ip_be, 4);       /* BindIp                            */
    pkt[211] = 0x01;                    /* BindIndex (primaire)              */
    pkt[212] = 0x08;                    /* Status2 : DHCP capable            */

    struct pbuf *pb = pbuf_alloc(PBUF_TRANSPORT, sizeof(pkt), PBUF_RAM);
    if (pb == NULL)
        return;
    memcpy(pb->payload, pkt, sizeof(pkt));
    /* Reponse a l'emetteur (unicast) ; les controleurs l'acceptent aussi
     * en broadcast, mais l'unicast vers le poller est le plus sur. */
    udp_sendto(s_pcb, pb, dst_addr, dst_port);
    pbuf_free(pb);
}

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

    /* ArtPoll : un controleur cherche les nodes -> repondre par ArtPollReply */
    if (pkt->opcode == ARTNET_OPCODE_POLL) {
        artnet_send_poll_reply(addr, ARTNET_PORT);
        goto done;
    }

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

    s_pcb = pcb;   /* conserve pour emettre les ArtPollReply */
    udp_recv(pcb, artnet_recv_cb, NULL);

    debug_printf("[ArtNet] Ecoute sur %s:" ARTNET_PORT_STR "\r\n",
                 ip4addr_ntoa(netif_ip4_addr(&gnetif)));
}