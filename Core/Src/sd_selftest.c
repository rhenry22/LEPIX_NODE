#include "sd_selftest.h"
#include "fatfs.h"
#include "main.h"     /* HAL_GetTick */
#include <stdio.h>
#include <string.h>

#define SDTEST_FILENAME      "test.txt"
#define SDTEST_DELETE_MS     (2u * 60u * 1000u)   /* 2 minutes */

/* États de la routine de test. */
typedef enum {
    SDT_IDLE = 0,   /* rien à faire (fichier non créé ou déjà supprimé) */
    SDT_ARMED,      /* fichier créé, en attente de suppression          */
    SDT_DONE,       /* suppression effectuée                            */
} sdt_state_t;

static sdt_state_t s_state    = SDT_IDLE;
static uint32_t    s_create_ms = 0;

void SDTest_Begin(void)
{
    FIL  fil;
    UINT bw;
    FRESULT res;

    /* Création (écrase si déjà présent) */
    res = f_open(&fil, SDTEST_FILENAME, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        printf("[SDTest] Creation %s ECHEC (res=%d) — carte absente/non montee ?\r\n",
               SDTEST_FILENAME, res);
        s_state = SDT_IDLE;
        return;
    }

    /* Message de log */
    static const char msg[] =
        "LEPIX Node - test carte SD\r\n"
        "Ce fichier a ete cree au demarrage et sera supprime dans 2 minutes.\r\n";

    res = f_write(&fil, msg, (UINT)strlen(msg), &bw);
    f_sync(&fil);
    f_close(&fil);

    if (res != FR_OK || bw != strlen(msg)) {
        printf("[SDTest] Ecriture %s ECHEC (res=%d, %u/%u octets)\r\n",
               SDTEST_FILENAME, res, bw, (unsigned)strlen(msg));
        s_state = SDT_IDLE;
        return;
    }

    s_create_ms = HAL_GetTick();
    s_state     = SDT_ARMED;
    printf("[SDTest] %s cree (%u octets) — suppression dans 2 min\r\n",
           SDTEST_FILENAME, bw);
}

void SDTest_Task(void)
{
    if (s_state != SDT_ARMED)
        return;

    if ((HAL_GetTick() - s_create_ms) < SDTEST_DELETE_MS)
        return;

    FRESULT res = f_unlink(SDTEST_FILENAME);
    if (res == FR_OK)
        printf("[SDTest] %s supprime (2 min ecoulees)\r\n", SDTEST_FILENAME);
    else
        printf("[SDTest] Suppression %s ECHEC (res=%d)\r\n", SDTEST_FILENAME, res);

    s_state = SDT_DONE;
}
