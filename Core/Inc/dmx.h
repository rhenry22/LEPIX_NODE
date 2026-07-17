/* dmx.h
 * Couche DMX512 sortie (contrôleur) — 2 ports sur MAX485.
 *
 * Port 0 : USART2 (PD5=TX, PD6=RX), direction PD7 (RS485_TX_RX_),
 *          DMA1_Stream6 Ch4.
 * Port 1 : USART3 (PC10=TX, PC11=RX), direction PD10 (P5_GPIOD10),
 *          DMA1_Stream3 Ch4.
 *
 * Trame DMX512 : BREAK (>=88 µs) + start code (0x00) + 512 slots, à
 * 250000 baud 8N2, rafraîchie ~40 Hz. Le BREAK est généré par abaissement
 * temporaire du baudrate (octet 0x00 à ~90000 baud ≈ 100 µs de niveau bas).
 */
#ifndef DMX_H
#define DMX_H

#include <stdint.h>
#include <stdbool.h>

#define DMX_SLOTS       512
#define DMX_NUM_PORTS   2

/* Initialise les 2 ports (USART + DMA + direction). À appeler une fois au
 * démarrage lorsque le mode DMX est actif. */
void DMX_Init(void);

/* Écrit un slot (0-based) dans le buffer d'un port. Sans effet si hors borne. */
void DMX_SetSlot(uint8_t port, uint16_t idx, uint8_t value);

/* Copie 'count' slots à partir de start_slot (0-based) dans le buffer du port. */
void DMX_SetSlots(uint8_t port, uint16_t start_slot, const uint8_t *data, uint16_t count);

/* Informe qu'un port a de nouvelles données (informatif : émission cadencée). */
void DMX_Commit(uint8_t port);

/* À appeler périodiquement depuis la boucle principale : émet une trame sur
 * chaque port tous les ~25 ms (40 Hz) si son DMA est libre. Non bloquant. */
void DMX_Task(void);

/* 1 si une trame est en cours d'émission sur le port. */
bool DMX_Busy(uint8_t port);

/* À câbler dans les IRQ DMA : port 0 -> DMA1_Stream6, port 1 -> DMA1_Stream3. */
void DMX_DMA_TX_IRQHandler(uint8_t port);

#endif /* DMX_H */
