/* gpio_test.h
 * Test/diagnostic passif des GPIO au démarrage : lit et affiche sur l'UART
 * l'état du jumper de mode (PA5/PA6) et des broches du DMX (direction
 * MAX485 PD7/PD10, TX/RX des 2 USART). Ne pilote rien (non destructif).
 * Activé par le define GPIO_TEST dans main.c.
 */
#ifndef GPIO_TEST_H
#define GPIO_TEST_H

/* Lit et journalise l'état des GPIO jumper + DMX. À appeler au boot,
 * après MX_GPIO_Init() / Mode_Init() (et DMX_Init si mode DMX). */
void GpioTest_Run(void);

#endif /* GPIO_TEST_H */
