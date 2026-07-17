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

/* Per-output configuration */
typedef struct {
    bool     enabled;
    uint16_t universe;      /* Art-Net / sACN universe 0-32767 */
    uint16_t led_count;     /* 1-512 */
    uint8_t  max_current_A; /* 0-5 amps */
    uint8_t  dmx_channel;   /* start channel for DMX mode */
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
/* Défauts = comportement historique du node : IP statique 2.2.2.2/8
 * (convention Art-Net réseau 2.x.x.x) même sans carte SD. */
#define CONFIG_DEFAULT { \
    .version    = CONFIG_VERSION, \
    .net_mode   = NET_STATIC, \
    .ip         = {2, 2, 2, 2}, \
    .netmask    = {255, 0, 0, 0}, \
    .gateway    = {2, 2, 2, 1}, \
    .dns        = {8, 8, 8, 8}, \
    .protocol   = PROTO_ARTNET, \
    .dmx_uart   = 2, \
    .outputs    = { \
        {true,  0, 120, 5, 1}, \
        {true,  1, 120, 5, 1}, \
        {true,  2, 120, 5, 1}, \
        {true,  3, 120, 5, 1}, \
    }, \
}

void Config_Init(void);
void Config_Load(void);
void Config_Save(void);
void Config_SetDefaults(void);
DeviceConfig_t *Config_Get(void);

#endif