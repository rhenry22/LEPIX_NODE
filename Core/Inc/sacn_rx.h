/* sacn_rx.h
 * Réception sACN (ANSI E1.31) via LwIP raw UDP multicast.
 *
 * sACN = DMX over IP en multicast : port UDP 5568, groupe multicast
 * 239.255.<univers_hi>.<univers_lo>. Le node rejoint les groupes des
 * univers qu'il écoute et délivre les données DMX via un callback,
 * identique dans l'esprit à artnet.c.
 */
#ifndef SACN_RX_H
#define SACN_RX_H

#include <stdint.h>

#define SACN_PORT   5568

/* Callback appelé à chaque trame DMX sACN valide reçue.
 * universe = 1..63999, data = slots DMX (hors start code), len = nb slots. */
typedef void (*sacn_dmx_cb_t)(uint16_t universe, uint8_t *data, uint16_t len);

/* Crée le PCB UDP, rejoint les groupes multicast des univers 1..count et
 * arme le callback de réception. À appeler après MX_LWIP_Init(). */
void sacn_rx_init(void);

/* Enregistre le callback DMX utilisateur. */
void sacn_rx_set_callback(sacn_dmx_cb_t cb);

/* Rejoint le groupe multicast d'un univers (1..63999). */
void sacn_rx_join_universe(uint16_t universe);

#endif /* SACN_RX_H */
