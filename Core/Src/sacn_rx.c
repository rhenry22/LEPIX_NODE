/* sacn_rx.c — réception sACN (ANSI E1.31) en UDP multicast, LwIP raw. */

#include "sacn_rx.h"
#include "web_ui.h"
#include "merge.h"
#include "log_capture.h"
#include "lwip/udp.h"
#include "lwip/igmp.h"
#include "lwip/ip_addr.h"
#include <string.h>
#include <stdbool.h>

/* ─── Offsets E1.31 (repris de la biblio sACN) ───────────────────────────── */
#define OFF_IDENT           4    /* "ASC-E1.17\0..." (12 octets)     */
#define OFF_CID            22    /* CID source (16 octets)           */
#define OFF_VECTOR_ROOT     18   /* uint32 BE = 0x00000004 (E1.31)   */
#define OFF_VECTOR_FRAME    40   /* uint32 BE = 0x00000002 (DATA)    */
#define OFF_SOURCE_NAME     44   /* 64 octets                        */
#define OFF_SEQ_NUM        111
#define OFF_OPTIONS        112
#define OFF_UNIVERSE       113   /* uint16 BE                        */
#define OFF_VECTOR_DMP     117   /* 0x02                             */
#define OFF_PROP_COUNT     123   /* uint16 BE = slots + 1 (startcode)*/
#define OFF_START_CODE     125   /* 0x00 = DMX                       */
#define OFF_DMX_DATA       126

#define E131_ROOT_VECTOR    0x00000004u
#define E131_FRAME_VECTOR   0x00000002u
#define E131_DMP_VECTOR     0x02u
#define E131_MIN_LEN        126

static const char ACN_ID[12] = { 'A','S','C','-','E','1','.','1','7',0,0,0 };

static struct udp_pcb *s_pcb = NULL;
static sacn_dmx_cb_t   s_cb  = NULL;

/* Suivi de séquence par univers (détection de paquets périmés/désordonnés). */
static uint8_t s_last_seq[512];   /* indexé par (universe & 0x1FF) */

static uint32_t rd_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}
static uint16_t rd_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

void sacn_rx_set_callback(sacn_dmx_cb_t cb) { s_cb = cb; }

/* Adresse multicast E1.31 d'un univers : 239.255.<hi>.<lo>. */
static void sacn_universe_group(uint16_t universe, ip_addr_t *out)
{
    IP4_ADDR(out, 239, 255, (universe >> 8) & 0xFF, universe & 0xFF);
}

/* Univers actuellement rejoints (pour pouvoir les quitter au changement).
 * Taille = nombre max de sorties routables ; ajuster si besoin. */
#define SACN_MAX_JOINED  8
static uint16_t s_joined[SACN_MAX_JOINED];
static uint8_t  s_joined_count = 0;

static bool joined_contains(uint16_t u)
{
    for (uint8_t i = 0; i < s_joined_count; i++)
        if (s_joined[i] == u) return true;
    return false;
}

void sacn_rx_join_universe(uint16_t universe)
{
    if (universe == 0 || universe > 63999) return;
    if (joined_contains(universe)) return;              /* deja rejoint */
    ip_addr_t grp;
    sacn_universe_group(universe, &grp);
    if (igmp_joingroup(IP_ADDR_ANY, ip_2_ip4(&grp)) == ERR_OK &&
        s_joined_count < SACN_MAX_JOINED) {
        s_joined[s_joined_count++] = universe;
    }
}

void sacn_rx_leave_universe(uint16_t universe)
{
    if (universe == 0 || universe > 63999) return;
    ip_addr_t grp;
    sacn_universe_group(universe, &grp);
    igmp_leavegroup(IP_ADDR_ANY, ip_2_ip4(&grp));
    /* Retire de la table (compactage). */
    for (uint8_t i = 0; i < s_joined_count; i++) {
        if (s_joined[i] == universe) {
            s_joined[i] = s_joined[--s_joined_count];
            break;
        }
    }
}

void sacn_rx_set_universes(const uint16_t *universes, uint8_t count)
{
    if (universes == NULL) return;

    /* 1) Quitter les univers rejoints qui ne sont plus demandes.
     *    On itere sur une copie car sacn_rx_leave_universe modifie s_joined. */
    uint16_t old[SACN_MAX_JOINED];
    uint8_t  old_count = s_joined_count;
    for (uint8_t i = 0; i < old_count; i++) old[i] = s_joined[i];

    for (uint8_t i = 0; i < old_count; i++) {
        bool still = false;
        for (uint8_t j = 0; j < count; j++)
            if (universes[j] == old[i]) { still = true; break; }
        if (!still)
            sacn_rx_leave_universe(old[i]);
    }

    /* 2) Rejoindre les nouveaux (join est idempotent). */
    for (uint8_t j = 0; j < count; j++)
        sacn_rx_join_universe(universes[j]);
}

static void sacn_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                         const ip_addr_t *addr, u16_t port)
{
    (void)arg; (void)pcb; (void)addr; (void)port;
    if (p == NULL) return;
    if (p->tot_len < E131_MIN_LEN) goto done;

    uint8_t buf[638];   /* 126 header + 512 slots */
    uint16_t copy_len = (p->tot_len < sizeof(buf)) ? p->tot_len : sizeof(buf);
    pbuf_copy_partial(p, buf, copy_len, 0);

    /* Validation ACN packet identifier + vecteurs de couche. */
    if (memcmp(&buf[OFF_IDENT], ACN_ID, sizeof(ACN_ID)) != 0) goto done;
    if (rd_be32(&buf[OFF_VECTOR_ROOT])  != E131_ROOT_VECTOR)  goto done;
    if (rd_be32(&buf[OFF_VECTOR_FRAME]) != E131_FRAME_VECTOR) goto done;
    if (buf[OFF_VECTOR_DMP] != E131_DMP_VECTOR)               goto done;
    if (buf[OFF_START_CODE] != 0x00)                          goto done; /* DMX only */

    uint16_t universe = rd_be16(&buf[OFF_UNIVERSE]);
    if (universe == 0 || universe > 63999) goto done;

    /* property count = slots + 1 (le start code). */
    uint16_t prop = rd_be16(&buf[OFF_PROP_COUNT]);
    uint16_t slots = (prop > 0) ? (prop - 1) : 0;
    if (slots > 512) slots = 512;
    if ((uint16_t)(OFF_DMX_DATA + slots) > copy_len)
        slots = (copy_len > OFF_DMX_DATA) ? (copy_len - OFF_DMX_DATA) : 0;

    /* Détection de séquence (E1.31 §6.7.2) : ignore les paquets trop anciens. */
    uint8_t  seq = buf[OFF_SEQ_NUM];
    uint16_t si  = universe & 0x1FF;
    int8_t   diff = (int8_t)(seq - s_last_seq[si]);
    if (diff <= 0 && diff > -20) goto done;   /* périmé / doublon */
    s_last_seq[si] = seq;

    WebUI_NotifySacn(universe);

    /* Commande CLI "log" : capture sur donnees brutes, avant fusion HTP. */
    LogCapture_OnFrame(LOG_SRC_SACN, universe, &buf[OFF_DMX_DATA], slots);

    /* Merge HTP : source identifiee par le CID E1.31 (16 octets, offset 22). */
    Merge_Submit(universe, &buf[OFF_CID], &buf[OFF_DMX_DATA], slots);

done:
    pbuf_free(p);
}

void sacn_rx_init(void)
{
    s_pcb = udp_new();
    if (s_pcb == NULL) return;

    /* Écoute sur le port E1.31, toutes interfaces. */
    udp_bind(s_pcb, IP_ADDR_ANY, SACN_PORT);
    udp_recv(s_pcb, sacn_recv_cb, NULL);

    memset(s_last_seq, 0, sizeof(s_last_seq));
}
