#include "log_capture.h"
#include <string.h>
#include <stdio.h>

typedef struct {
    volatile bool     armed;
    volatile bool     ready;     /* trame capturee, en attente d'impression */
    uint16_t          universe;
    uint16_t          len;       /* longueur reelle recue (peut depasser le buffer) */
    uint8_t           data[LOG_CAPTURE_MAX_BYTES];
} capture_slot_t;

static capture_slot_t s_slot[LOG_SRC_COUNT];

static const char *src_name(LogSource_t s)
{
    return (s == LOG_SRC_SACN) ? "sACN" : "Art-Net";
}

void LogCapture_Arm(void)
{
    for (int i = 0; i < LOG_SRC_COUNT; i++) {
        s_slot[i].armed = true;
        s_slot[i].ready = false;
    }
}

bool LogCapture_IsArmed(void)
{
    for (int i = 0; i < LOG_SRC_COUNT; i++)
        if (s_slot[i].armed)
            return true;
    return false;
}

void LogCapture_OnFrame(LogSource_t src, uint16_t universe,
                        const uint8_t *data, uint16_t len)
{
    if (src >= LOG_SRC_COUNT)
        return;
    capture_slot_t *s = &s_slot[src];
    if (!s->armed)
        return;

    uint16_t copy_len = (len > LOG_CAPTURE_MAX_BYTES) ? LOG_CAPTURE_MAX_BYTES : len;
    memcpy(s->data, data, copy_len);
    s->universe = universe;
    s->len      = len;
    s->armed    = false;
    s->ready    = true;   /* consommee/imprimee par LogCapture_Task() */
}

void LogCapture_Task(void)
{
    for (int i = 0; i < LOG_SRC_COUNT; i++) {
        capture_slot_t *s = &s_slot[i];
        if (!s->ready)
            continue;
        s->ready = false;

        printf("[log] %s : univers=%u len=%u octets\r\n",
               src_name((LogSource_t)i), s->universe, s->len);
        printf("[log]   data:");
        uint16_t shown = (s->len > LOG_CAPTURE_MAX_BYTES) ? LOG_CAPTURE_MAX_BYTES : s->len;
        for (uint16_t j = 0; j < shown; j++)
            printf(" %02X", s->data[j]);
        if (s->len > shown)
            printf(" ... (%u octets non affiches)", (unsigned)(s->len - shown));
        printf("\r\n");
    }
}
