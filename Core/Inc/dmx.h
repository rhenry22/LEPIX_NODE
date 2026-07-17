/* dmx.h
 * Couche DMX512 sortie (contrôleur) sur USART2 + MAX485.
 *
 * Câblage : PD5 = USART2_TX -> DI, PD6 = USART2_RX -> RO (RDM, étape 4),
 *           PD7 = RS485_TX_RX_ -> DE+/RE (HIGH = émission).
 *
 * Trame DMX512 : BREAK (>=88 µs) + MAB (>=8 µs) + start code (0x00) +
 *                512 slots, à 250000 baud 8N2. Rafraîchie ~40 Hz.
 *
 * Le BREAK est généré par abaissement temporaire du baudrate (envoi d'un
 * octet 0x00 à ~90000 baud ≈ 100 µs de niveau bas), puis les 513 octets
 * sont envoyés par DMA1_Stream6 à 250000 baud.
 */
#ifndef DMX_H
#define DMX_H

#include <stdint.h>
#include <stdbool.h>

#define DMX_SLOTS   512

/* Initialise USART2 (250k 8N2), le DMA TX et la broche de direction PD7.
 * À appeler une fois au démarrage lorsque le mode DMX est actif. */
void DMX_Init(void);

/* Écrit un slot (0-based) dans le buffer de travail. Sans effet si idx>=512. */
void DMX_SetSlot(uint16_t idx, uint8_t value);

/* Copie 'count' slots à partir de start_slot (0-based) dans le buffer. */
void DMX_SetSlots(uint16_t start_slot, const uint8_t *data, uint16_t count);

/* Marque le buffer comme à émettre au prochain rafraîchissement. */
void DMX_Commit(void);

/* À appeler périodiquement depuis la boucle principale : émet une trame
 * DMX tous les ~25 ms (40 Hz) si le DMA est libre. Non bloquant. */
void DMX_Task(void);

/* 1 si une trame DMX est en cours d'émission (DMA occupé). */
bool DMX_Busy(void);

/* À câbler dans DMA1_Stream6_IRQHandler (USART2_TX). */
void DMX_DMA_TX_IRQHandler(void);

#endif /* DMX_H */
