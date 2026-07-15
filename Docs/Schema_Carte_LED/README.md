# Carte d'envoi des données LED — LEPIX NODE

Node **Art-Net / sACN** sur carte industrielle **JZ-STM32F407VET6** (STM32F407VET6, LQFP100, 168 MHz).
Réception DMX sur IP via Ethernet, pilotage de **4 chaînes WS2815 en parallèle** par TIM1 + DMA2 sur le port GPIOD.
Schéma reconstitué depuis le code du dépôt (branche `artnet_receiver_HRP`).

> 💡 Version interactive : ouvrir [`index.html`](index.html) dans un navigateur (page autonome, thèmes clair/sombre).

---

## Fig. 1 — Synoptique : du réseau aux rubans

La trame Art-Net (UDP 6454) ou sACN E1.31 (UDP 5568) arrive par le PHY DP83848 en RMII, est décodée
par LwIP puis routée univers → sortie selon `config.json` (carte SD). Les 4 sorties sont émises
simultanément par écriture DMA dans `GPIOD->BSRR`.

![Synoptique de la carte](fig1_synoptique.svg)

*Univers indiqués = configuration par défaut ([`Core/Inc/config.h`](../../Core/Inc/config.h)).*

---

## Fig. 2 — Moteur d'émission TIM1 + DMA2

Trois événements du timer par bit, chacun déclenchant un flux DMA qui écrit dans `GPIOD->BSRR`.
Le port entier bascule d'un coup : les 4 sorties sont strictement synchrones.

![Moteur TIM1 + DMA2](fig2_moteur_tim1_dma2.svg)

*Source : `MX_TIM1_WS2815_Init()` dans [`Core/Src/tim.c`](../../Core/Src/tim.c).*

---

## Fig. 3 — Chronogramme d'un bit (1,25 µs)

La valeur du bit est décidée uniquement par l'événement CC1 : si le masque DMA contient la pin,
elle retombe à 0,30 µs (bit « 0 »), sinon elle reste haute jusqu'à CC2 à 0,70 µs (bit « 1 »).

![Chronogramme d'un bit WS2815](fig3_chronogramme.svg)

| Paramètre       | Valeur        | Origine                |
|-----------------|---------------|------------------------|
| Période bit     | 1,25 µs       | `ARR = 209` @ 168 MHz  |
| T0H             | 0,30 µs       | TIM1_CH1, `Pulse = 50` |
| T1H             | 0,70 µs       | TIM1_CH2, `Pulse = 118`|
| Trame 120 LED   | ≈ 3,6 ms      | 120 × 24 bits          |
| Latch (reset)   | > 280 µs bas  | `WS2815_Busy()`        |

---

## Brochage

### Sorties LED — GPIOD / connecteur P5

| Canal | Pin MCU | Label CubeMX | Univers (défaut) | LEDs max | Courant max (config) |
|-------|---------|--------------|------------------|----------|----------------------|
| CH1   | PD15    | `WS2815_CH1` | 0                | 120      | 5 A                  |
| CH2   | PD13    | `WS2815_CH2` | 1                | 120      | 5 A                  |
| CH3   | PD11    | `WS2815_CH3` | 2                | 120      | 5 A                  |
| CH4   | PD9     | `WS2815_CH4` | 3                | 120      | 5 A                  |

### Ethernet RMII (PHY DP83848)

| Signal                | Pin MCU             |
|-----------------------|---------------------|
| `ETH_REF_CLK` (50 MHz)| PA1                 |
| `ETH_MDIO` / `ETH_MDC`| PA2 / PC1           |
| `ETH_CRS_DV`          | PA7                 |
| `ETH_RXD0` / `RXD1`   | PC4 / PC5           |
| `ETH_TX_EN`           | PB11                |
| `ETH_TXD0` / `TXD1`   | PB12 / PB13         |
| Adresse MAC           | `00:80:E1:00:00:01` |

### Interface locale (configuration du node)

| Périphérique                 | Bus / pins                                          |
|------------------------------|-----------------------------------------------------|
| Écran ST7789 280×240         | SPI2 : SCK PB10 · MOSI PC3                          |
| — CS / DC / RST / BLK        | PA4 / PE6 / PC0 / PC13                              |
| Encodeur rotatif CLK/DT/SW   | PE0 / PE5 / PB7                                     |
| Carte SD (`config.json`)     | SDIO : PC8-PC12, PD2 · CD PD3                       |
| DMX filaire (option)         | USART1 PA9/PA10 · USART2 PD5/PD6 (RS485, DE PD7)    |

---

## Points d'attention matériels

- **Niveau logique** : le signal data sort du GPIO en 3,3 V push-pull, sans level-shifter visible dans
  le projet. Les WS2815 (alim 12 V) l'acceptent généralement, mais sur câble long un buffer 5 V
  (74HCT245) et une résistance série ~100 Ω en début de ligne sécurisent le montage.
- **Masse commune obligatoire** : le GND de l'alimentation 12 V des rubans doit être relié au GND de
  la carte, sinon le signal data n'a pas de référence.
- **Alimentation des rubans** : le 12 V des WS2815 est externe — la carte ne fournit que le signal
  data. La limite 5 A par sortie est une valeur de configuration (`max_current_A`), pas une protection
  matérielle.
- **Broche BI des WS2815** : ligne de secours du ruban ; la carte ne la pilote pas, la laisser suivre
  le câblage recommandé du fabricant (généralement reliée à GND en entrée de ruban ou au DO précédent).
- **Synchronisation** : les 4 canaux partagent le même port GPIOD et les mêmes événements TIM1 — les
  trames partent rigoureusement en même temps sur les 4 sorties (~3,6 ms pour 120 LED).

---

*Sources : [`Core/Src/tim.c`](../../Core/Src/tim.c) · [`Core/Inc/tim.h`](../../Core/Inc/tim.h) ·
[`Core/Src/ws2815.c`](../../Core/Src/ws2815.c) · [`Core/Inc/config.h`](../../Core/Inc/config.h) ·
[`Industrial_Board.ioc`](../../Industrial_Board.ioc) · [`ReadMe.md`](../../ReadMe.md) — juillet 2026.*
