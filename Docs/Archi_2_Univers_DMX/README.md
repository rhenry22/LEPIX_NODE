# Architecture 2 univers DMX512 filaires — LEPIX NODE

Le même matériel que le node LED, basculé en **mode DMX** par le jumper PA5/PA6 : il reçoit deux
univers en sACN (ou Art-Net) sur Ethernet et les ressort en continu sur deux lignes DMX512 RS-485
à 40 Hz. Aucune sortie WS2815 dans cette configuration.

> 💡 Version interactive : ouvrir [`index.html`](index.html) dans un navigateur.
> Schéma matériel du node : [`Docs/Schema_Carte_LED`](../Schema_Carte_LED/README.md).

---

## Fig. 1 — Synoptique : 2 univers, de la console aux projecteurs

La console émet les univers 1 et 2 en sACN (multicast UDP 5568). Le node (IP par défaut
`2.0.0.3` en mode DMX) décode chaque univers et alimente son port série dédié : chaque port
émet en boucle BREAK + 512 canaux, que la console envoie ou non.

![Synoptique 2 univers DMX](fig1_synoptique_dmx.svg)

*Source : [`Core/Src/dmx.c`](../../Core/Src/dmx.c).*

---

## Les 2 ports DMX

| Port | UART   | TX / direction | DMA         | Transceiver              |
|------|--------|----------------|-------------|--------------------------|
| A    | USART2 | PD5 / PD7 (TX/#RX)      | DMA1 St.6   | port **RS485 de la carte** (bornier A/B/GND) |
| B    | USART3 | PD8 / PD10    | DMA1 St.3   | MAX485 **externe** (5 V)  |

| Paramètre DMX512     | Valeur                              |
|----------------------|-------------------------------------|
| Débit                | 250 kbauds · 8N2                    |
| Trame                | BREAK + start code + 512 canaux     |
| Rafraîchissement     | 40 Hz (25 ms), flux continu         |
| Appareils par ligne  | 32 max (norme EIA-485)              |
| Terminaison          | 120 Ω entre Data+ et Data−          |

## Configuration pour 2 univers

Générée avec [`tools/make_sd_config.py`](../../tools/make_sd_config.py) :

```bash
./tools/make_sd_config.py --protocol sacn --universes 1,2,0,0 --disable 3,4
```

| Clé                  | Valeur      | Effet         |
|----------------------|-------------|---------------|
| `protocol`           | 1           | sACN E1.31    |
| `net_mode` / `ip`    | 1 · 2.0.0.3 | IP statique   |
| `output0.universe`   | 1           | port DMX A    |
| `output1.universe`   | 2           | port DMX B    |
| `output2/3.enabled`  | 0           | inutilisés    |

## Points d'attention

- **Jumper de mode** : PA5/PA6 sélectionne DMX au boot — sans lui le node reste en mode LED et
  les ports DMX sont muets.
- **Port B** : PD8 et PD10 (adjacents sur P5) sortent en logique 3,3 V sur les connecteurs — le transceiver RS-485
  externe (MAX485/SN75176 alimenté en 5 V) est indispensable pour attaquer le bus.
- **Câblage XLR** : broche 1 = GND, 2 = Data−, 3 = Data+. Toujours une terminaison 120 Ω sur le
  dernier appareil de chaque ligne.
- **Flux continu** : les ports émettent à 40 Hz même sans trafic réseau — les projecteurs gardent
  leurs dernières valeurs si la console se tait.
- **sACN multicast** : univers 1 → groupe 239.255.0.1, univers 2 → 239.255.0.2 ; le switch doit
  laisser passer le multicast (éviter l'IGMP snooping trop agressif).

---

*Sources : [`Core/Src/dmx.c`](../../Core/Src/dmx.c) · [`Core/Inc/dmx.h`](../../Core/Inc/dmx.h) ·
[`Core/Src/mode_select.c`](../../Core/Src/mode_select.c) ·
[`Core/Src/config.c`](../../Core/Src/config.c) — juillet 2026.*
