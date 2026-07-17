/* sd_selftest.h
 * Test de la carte SD : crée test.txt, y écrit un message de log, puis le
 * supprime 2 minutes plus tard. Non bloquant : piloté depuis la boucle
 * principale. Activé par le define SD_SELFTEST dans main.c.
 */
#ifndef SD_SELFTEST_H
#define SD_SELFTEST_H

/* Crée test.txt et y écrit le message de log. À appeler une fois au boot,
 * après le montage de la carte SD. Arme le timer de suppression. */
void SDTest_Begin(void);

/* À appeler périodiquement depuis la boucle principale. Supprime test.txt
 * une fois le délai (2 min) écoulé. Non bloquant. */
void SDTest_Task(void);

#endif /* SD_SELFTEST_H */
