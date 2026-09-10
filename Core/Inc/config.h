#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#define CONFIG_PATH      "config.json"
#define CONFIG_VERSION   1
#define MAX_OUTPUTS      4

/* Input protocol */
typedef enum {
    PROTO_ARTNET = 0,
    PROTO_SACN,
    PROTO_DMX_UART,
} InputProtocol_t;

/* Network mode */
typedef enum {
    NET_DHCP = 0,
    NET_STATIC,
} NetMode_t;

/* Format pixel par sortie — nombre d'octets et ordre des canaux dans le
 * flux DMX/Art-Net/sACN reçu pour cette sortie. Affiché/éditable dans la
 * Web UI (onglet Configuration) et dans /groupes ; PAS ENCORE branché au
 * driver DMA (Core/Src/ws2815.c émet toujours en GRB 3 octets fixe — voir
 * commentaire dans ws2815.h). Valeur purement informative pour l'instant,
 * pensée pour préparer un futur driver multi-format. */
typedef enum {
    PIXEL_FMT_RGB = 0,
    PIXEL_FMT_GRB,      /* défaut WS2812/WS2815 */
    PIXEL_FMT_BRG,
    PIXEL_FMT_RGBW,
    PIXEL_FMT_GRBW,
    PIXEL_FMT_RGBWW,    /* RGB + blanc chaud + blanc froid (5 octets) */
} PixelFormat_t;

#define PIXEL_FMT_COUNT 6

/* Nombre d'octets par pixel pour un format donné. */
static inline uint8_t PixelFormat_BytesPerPixel(PixelFormat_t f)
{
    switch (f) {
        case PIXEL_FMT_RGBW:
        case PIXEL_FMT_GRBW:  return 4;
        case PIXEL_FMT_RGBWW: return 5;
        default:              return 3;   /* RGB / GRB / BRG */
    }
}

/* Nom court pour affichage (Web UI, CLI). */
static inline const char *PixelFormat_Name(PixelFormat_t f)
{
    switch (f) {
        case PIXEL_FMT_RGB:   return "RGB";
        case PIXEL_FMT_GRB:   return "GRB";
        case PIXEL_FMT_BRG:   return "BRG";
        case PIXEL_FMT_RGBW:  return "RGBW";
        case PIXEL_FMT_GRBW:  return "GRBW";
        case PIXEL_FMT_RGBWW: return "RGBWW";
        default:              return "?";
    }
}

/* Per-output configuration */
typedef struct {
    bool          enabled;
    uint16_t      universe;      /* Art-Net / sACN universe 0-32767 */
    uint16_t      led_count;     /* 1-512 */
    uint8_t       max_current_A; /* 0-5 amps */
    uint8_t       dmx_channel;   /* start channel for DMX mode */
    PixelFormat_t pixel_format;  /* RGB/GRB/RGBW/... — voir note ci-dessus */
} OutputConfig_t;

/* Full device configuration */
typedef struct {
    uint8_t  version;

    /* Network */
    NetMode_t   net_mode;
    uint8_t     ip[4];
    uint8_t     netmask[4];
    uint8_t     gateway[4];
    uint8_t     dns[4];

    /* Input protocol */
    InputProtocol_t protocol;
    uint8_t         dmx_uart; /* USART index: 1 or 2 */

    /* Outputs */
    OutputConfig_t  outputs[MAX_OUTPUTS];
} DeviceConfig_t;

/* Default values */
/* Défauts = DHCP pour le moment (adresse fournie par le serveur du réseau).
 * Les champs IP/netmask/gateway ci-dessous servent de repli si l'on
 * repasse en statique. En DHCP, la surcharge IP-par-mode de main.c
 * (dernier octet .3/.4) reste sans effet. */
#define CONFIG_DEFAULT { \
    .version    = CONFIG_VERSION, \
    .net_mode   = NET_DHCP, \
    .ip         = {2, 0, 0, 3}, \
    .netmask    = {255, 255, 255, 0}, \
    .gateway    = {2, 0, 0, 1}, \
    .dns        = {8, 8, 8, 8}, \
    .protocol   = PROTO_SACN, \
    .dmx_uart   = 2, \
    .outputs    = { \
        {true,  0, 120, 5, 1, PIXEL_FMT_GRB}, \
        {true,  1, 120, 5, 1, PIXEL_FMT_GRB}, \
        {true,  2, 120, 5, 1, PIXEL_FMT_GRB}, \
        {true,  3, 120, 5, 1, PIXEL_FMT_GRB}, \
    }, \
}

void Config_Init(void);
void Config_Load(void);
void Config_Save(void);
void Config_SetDefaults(void);
DeviceConfig_t *Config_Get(void);

/* true si la config vient d'un config.json valide (SD), false si défauts. */
bool Config_IsFromSD(void);

#endif