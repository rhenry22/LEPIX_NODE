#include "cli.h"
#include "usart.h"    /* huart1 */
#include "config.h"
#include "log_capture.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define CLI_RING_SIZE 256u   /* puissance de 2 non requise (modulo) */
#define CLI_LINE_MAX  96u

static uint8_t           s_rx_byte;
static volatile uint8_t  s_ring[CLI_RING_SIZE];
static volatile uint16_t s_head, s_tail;
static char              s_line[CLI_LINE_MAX];
static uint16_t          s_len;

/* ------------------------------------------------------------------ RX IT */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;
    uint16_t next = (uint16_t)((s_head + 1u) % CLI_RING_SIZE);
    if (next != s_tail) {           /* buffer plein : octet perdu */
        s_ring[s_head] = s_rx_byte;
        s_head = next;
    }
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

/* --------------------------------------------------------------- helpers */

static int parse_ip(const char *s, uint8_t out[4])
{
    unsigned a, b, c, d;
    if (sscanf(s, "%u.%u.%u.%u", &a, &b, &c, &d) != 4 ||
        a > 255 || b > 255 || c > 255 || d > 255)
        return 0;
    out[0] = a; out[1] = b; out[2] = c; out[3] = d;
    return 1;
}

static const char *proto_name(InputProtocol_t p)
{
    switch (p) {
    case PROTO_ARTNET:   return "artnet";
    case PROTO_SACN:     return "sacn";
    case PROTO_DMX_UART: return "dmx";
    default:             return "?";
    }
}

static void cmd_show(void)
{
    DeviceConfig_t *c = Config_Get();
    printf("source   : %s\r\n", Config_IsFromSD() ? "config.json (SD)"
                                                  : "defauts");
    printf("reseau   : %s\r\n", c->net_mode == NET_DHCP ? "dhcp" : "statique");
    printf("ip       : %u.%u.%u.%u\r\n", c->ip[0], c->ip[1], c->ip[2], c->ip[3]);
    printf("mask     : %u.%u.%u.%u\r\n", c->netmask[0], c->netmask[1],
                                         c->netmask[2], c->netmask[3]);
    printf("gw       : %u.%u.%u.%u\r\n", c->gateway[0], c->gateway[1],
                                         c->gateway[2], c->gateway[3]);
    printf("proto    : %s\r\n", proto_name(c->protocol));
    for (int i = 0; i < MAX_OUTPUTS; i++) {
        OutputConfig_t *o = &c->outputs[i];
        printf("out %d    : %s univers=%u leds=%u imax=%uA dmx_ch=%u\r\n",
               i + 1, o->enabled ? "on " : "off", o->universe,
               o->led_count, o->max_current_A, o->dmx_channel);
    }
}

static void cmd_help(void)
{
    printf("Commandes :\r\n"
           "  show                        etat de la configuration\r\n"
           "  log                         capture et affiche la prochaine\r\n"
           "                              trame sACN et Art-Net recue\r\n"
           "  ip A.B.C.D                  adresse IP statique\r\n"
           "  mask A.B.C.D                masque reseau\r\n"
           "  gw A.B.C.D                  passerelle\r\n"
           "  dhcp on|off                 mode reseau\r\n"
           "  proto artnet|sacn|dmx       protocole d'entree\r\n"
           "  out N on|off                active/desactive la sortie N (1-4)\r\n"
           "  out N universe U            univers de la sortie N\r\n"
           "  out N leds L                nombre de LEDs (1-512)\r\n"
           "  save                        ecrit config.json sur la SD\r\n"
           "  defaults                    recharge les valeurs par defaut\r\n"
           "  reboot                      redemarre le node\r\n"
           "Les changements reseau/proto s'appliquent apres save + reboot.\r\n");
}

static void cmd_log(void)
{
    LogCapture_Arm();
    printf("log arme : en attente de la prochaine trame sACN et Art-Net...\r\n");
}

static void cmd_out(char *arg1, char *arg2, char *arg3)
{
    DeviceConfig_t *c = Config_Get();
    int n = arg1 ? atoi(arg1) : 0;
    if (n < 1 || n > MAX_OUTPUTS || !arg2) {
        printf("usage : out <1-%d> on|off|universe U|leds L\r\n", MAX_OUTPUTS);
        return;
    }
    OutputConfig_t *o = &c->outputs[n - 1];
    if (!strcmp(arg2, "on") || !strcmp(arg2, "off")) {
        o->enabled = !strcmp(arg2, "on");
        printf("out %d : %s\r\n", n, o->enabled ? "on" : "off");
    } else if (!strcmp(arg2, "universe") && arg3) {
        o->universe = (uint16_t)atoi(arg3);
        printf("out %d : univers %u\r\n", n, o->universe);
    } else if (!strcmp(arg2, "leds") && arg3) {
        int l = atoi(arg3);
        if (l < 1 || l > 512) { printf("leds : 1-512\r\n"); return; }
        o->led_count = (uint16_t)l;
        printf("out %d : %u leds\r\n", n, o->led_count);
    } else {
        printf("usage : out <1-%d> on|off|universe U|leds L\r\n", MAX_OUTPUTS);
    }
}

static void exec_line(char *line)
{
    DeviceConfig_t *c = Config_Get();
    char *cmd  = strtok(line, " \t");
    char *a1   = strtok(NULL, " \t");
    char *a2   = strtok(NULL, " \t");
    char *a3   = strtok(NULL, " \t");
    if (!cmd)
        return;

    if      (!strcmp(cmd, "help"))     cmd_help();
    else if (!strcmp(cmd, "show"))     cmd_show();
    else if (!strcmp(cmd, "log"))      cmd_log();
    else if (!strcmp(cmd, "save"))   { Config_Save();
                                       printf("config.json ecrit sur SD\r\n"); }
    else if (!strcmp(cmd, "defaults")){ Config_SetDefaults();
                                       printf("valeurs par defaut rechargees "
                                              "(save pour conserver)\r\n"); }
    else if (!strcmp(cmd, "reboot"))  { printf("reboot...\r\n");
                                       HAL_Delay(50);
                                       NVIC_SystemReset(); }
    else if (!strcmp(cmd, "ip")   && a1 && parse_ip(a1, c->ip))
        printf("ip %s (save + reboot pour appliquer)\r\n", a1);
    else if (!strcmp(cmd, "mask") && a1 && parse_ip(a1, c->netmask))
        printf("mask %s (save + reboot pour appliquer)\r\n", a1);
    else if (!strcmp(cmd, "gw")   && a1 && parse_ip(a1, c->gateway))
        printf("gw %s (save + reboot pour appliquer)\r\n", a1);
    else if (!strcmp(cmd, "dhcp") && a1) {
        c->net_mode = !strcmp(a1, "on") ? NET_DHCP : NET_STATIC;
        printf("reseau : %s (save + reboot pour appliquer)\r\n",
               c->net_mode == NET_DHCP ? "dhcp" : "statique");
    }
    else if (!strcmp(cmd, "proto") && a1) {
        if      (!strcmp(a1, "artnet")) c->protocol = PROTO_ARTNET;
        else if (!strcmp(a1, "sacn"))   c->protocol = PROTO_SACN;
        else if (!strcmp(a1, "dmx"))    c->protocol = PROTO_DMX_UART;
        else { printf("proto : artnet|sacn|dmx\r\n"); return; }
        printf("proto : %s (save + reboot pour appliquer)\r\n",
               proto_name(c->protocol));
    }
    else if (!strcmp(cmd, "out"))
        cmd_out(a1, a2, a3);
    else
        printf("commande inconnue : %s (help pour la liste)\r\n", cmd);
}

/* ------------------------------------------------------------------- API */

void CLI_Init(void)
{
    /* stdout sans buffer : l'echo caractere par caractere doit partir
     * immediatement (sinon newlib retient jusqu'au \n ou buffer plein) */
    setvbuf(stdout, NULL, _IONBF, 0);
    s_head = s_tail = 0;
    s_len = 0;
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
    printf("CLI prete sur USART1 (help pour la liste des commandes)\r\n> ");
}

void CLI_Task(void)
{
    /* Auto-reparation : une erreur UART (overrun...) stoppe Receive_IT */
    if (huart1.RxState == HAL_UART_STATE_READY)
        HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);

    while (s_tail != s_head) {
        char ch = (char)s_ring[s_tail];
        s_tail = (uint16_t)((s_tail + 1u) % CLI_RING_SIZE);

        if (ch == '\r' || ch == '\n') {
            printf("\r\n");
            if (s_len) {
                s_line[s_len] = '\0';
                exec_line(s_line);
                s_len = 0;
            }
            printf("> ");
        } else if (ch == 0x7F || ch == '\b') {      /* backspace */
            if (s_len) { s_len--; printf("\b \b"); }
        } else if (s_len < CLI_LINE_MAX - 1u && ch >= 0x20) {
            s_line[s_len++] = ch;
            printf("%c", ch);                        /* echo */
        }
    }
}
