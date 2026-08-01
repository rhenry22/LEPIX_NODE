/* watchdog.h
 * Watchdog matériel indépendant (IWDG) : redémarre le MCU si la boucle
 * principale se fige (crash, blocage réseau/DMA). Indispensable en live.
 */
#ifndef WATCHDOG_H
#define WATCHDOG_H

/* Démarre l'IWDG (~2 s de timeout). Une fois lancé, il ne peut plus être
 * arrêté : la boucle DOIT appeler Watchdog_Refresh() au moins tous les 2 s,
 * sinon le MCU redémarre. À appeler juste avant la boucle principale. */
void Watchdog_Init(void);

/* Recharge le compteur du watchdog (à appeler régulièrement dans la boucle). */
void Watchdog_Refresh(void);

#endif /* WATCHDOG_H */
