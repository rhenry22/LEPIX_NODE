#include "config.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static DeviceConfig_t g_config = CONFIG_DEFAULT;

/* true si la config a été chargée depuis un config.json valide sur SD.
 * false = valeurs par défaut (permet la surcharge IP-par-mode au boot). */
static bool g_config_from_sd = false;

DeviceConfig_t *Config_Get(void) { return &g_config; }

bool Config_IsFromSD(void) { return g_config_from_sd; }

void Config_SetDefaults(void)
{
    DeviceConfig_t def = CONFIG_DEFAULT;
    memcpy(&g_config, &def, sizeof(DeviceConfig_t));
}

/* Minimal JSON parser — no external lib required */
static uint32_t parse_uint(const char *json, const char *key)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return 0;
    p += strlen(search);
    while (*p == ' ') p++;
    return (uint32_t)strtoul(p, NULL, 10);
}

static void parse_ip(const char *json, const char *key, uint8_t out[4])
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return;
    p += strlen(search);
    while (*p == ' ' || *p == '"') p++;
    sscanf(p, "%hhu.%hhu.%hhu.%hhu", &out[0], &out[1], &out[2], &out[3]);
}

void Config_Load(void)
{
    FIL    fil;
    UINT   br;
    char   buf[1024] = {0};

    if (f_open(&fil, CONFIG_PATH, FA_READ) != FR_OK) {
        printf("[Config] No config.json — using defaults\r\n");
        Config_SetDefaults();
        return;
    }

    f_read(&fil, buf, sizeof(buf) - 1, &br);
    f_close(&fil);

    if (br == 0) { Config_SetDefaults(); return; }

    /* Version check */
    uint32_t ver = parse_uint(buf, "version");
    if (ver != CONFIG_VERSION) {
        printf("[Config] Version mismatch — resetting to defaults\r\n");
        Config_SetDefaults();
        return;
    }

    /* Network */
    g_config.net_mode = (NetMode_t)parse_uint(buf, "net_mode");
    parse_ip(buf, "ip",      g_config.ip);
    parse_ip(buf, "netmask", g_config.netmask);
    parse_ip(buf, "gateway", g_config.gateway);
    parse_ip(buf, "dns",     g_config.dns);

    /* Protocol */
    g_config.protocol  = (InputProtocol_t)parse_uint(buf, "protocol");
    g_config.dmx_uart  = (uint8_t)parse_uint(buf, "dmx_uart");

    /* Outputs — parse output0..output3 */
    for (int i = 0; i < MAX_OUTPUTS; i++) {
        char key[32];
        char section[256] = {0};
        snprintf(key, sizeof(key), "\"output%d\":", i);
        const char *p = strstr(buf, key);
        if (!p) continue;
        p += strlen(key);
        /* Extract the {...} block */
        const char *start = strchr(p, '{');
        const char *end   = strchr(p, '}');
        if (!start || !end) continue;
        size_t len = (size_t)(end - start + 1);
        if (len > sizeof(section) - 1) len = sizeof(section) - 1;
        memcpy(section, start, len);

        g_config.outputs[i].enabled      = (bool)parse_uint(section, "enabled");
        g_config.outputs[i].universe     = (uint16_t)parse_uint(section, "universe");
        g_config.outputs[i].led_count    = (uint16_t)parse_uint(section, "led_count");
        g_config.outputs[i].max_current_A= (uint8_t)parse_uint(section, "max_current_A");
        g_config.outputs[i].dmx_channel  = (uint8_t)parse_uint(section, "dmx_channel");

        /* pixel_format absent (config.json d'avant cette version) -> GRB */
        if (strstr(section, "\"pixel_format\":")) {
            uint32_t pf = parse_uint(section, "pixel_format");
            g_config.outputs[i].pixel_format =
                (pf < PIXEL_FMT_COUNT) ? (PixelFormat_t)pf : PIXEL_FMT_GRB;
        } else {
            g_config.outputs[i].pixel_format = PIXEL_FMT_GRB;
        }
    }

    g_config_from_sd = true;
    printf("[Config] Loaded from SD\r\n");
}

void Config_Save(void)
{
    FIL  fil;
    UINT bw;
    char buf[1024];
    int  n = 0;

    n += snprintf(buf + n, sizeof(buf) - n,
        "{\r\n"
        "  \"version\": %u,\r\n"
        "  \"net_mode\": %u,\r\n"
        "  \"ip\": \"%u.%u.%u.%u\",\r\n"
        "  \"netmask\": \"%u.%u.%u.%u\",\r\n"
        "  \"gateway\": \"%u.%u.%u.%u\",\r\n"
        "  \"dns\": \"%u.%u.%u.%u\",\r\n"
        "  \"protocol\": %u,\r\n"
        "  \"dmx_uart\": %u,\r\n",
        g_config.version,
        g_config.net_mode,
        g_config.ip[0], g_config.ip[1], g_config.ip[2], g_config.ip[3],
        g_config.netmask[0], g_config.netmask[1], g_config.netmask[2], g_config.netmask[3],
        g_config.gateway[0], g_config.gateway[1], g_config.gateway[2], g_config.gateway[3],
        g_config.dns[0], g_config.dns[1], g_config.dns[2], g_config.dns[3],
        g_config.protocol,
        g_config.dmx_uart
    );

    for (int i = 0; i < MAX_OUTPUTS; i++) {
        n += snprintf(buf + n, sizeof(buf) - n,
            "  \"output%d\": {"
            "\"enabled\":%u, \"universe\":%u, \"led_count\":%u, "
            "\"max_current_A\":%u, \"dmx_channel\":%u, \"pixel_format\":%u}%s\r\n",
            i,
            g_config.outputs[i].enabled,
            g_config.outputs[i].universe,
            g_config.outputs[i].led_count,
            g_config.outputs[i].max_current_A,
            g_config.outputs[i].dmx_channel,
            g_config.outputs[i].pixel_format,
            (i < MAX_OUTPUTS - 1) ? "," : ""
        );
    }
    n += snprintf(buf + n, sizeof(buf) - n, "}\r\n");

    if (f_open(&fil, CONFIG_PATH, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        printf("[Config] Save failed — cannot open file\r\n");
        return;
    }
    f_write(&fil, buf, n, &bw);
    f_sync(&fil);
    f_close(&fil);
    printf("[Config] Saved to SD (%u bytes)\r\n", bw);
}

void Config_Init(void)
{
    Config_Load();
}