#!/usr/bin/env python3
"""Genere le config.json du LEPIX_NODE et l'ecrit sur la carte SD.

Le format produit est exactement celui du parseur maison du firmware
(Core/Src/config.c) : cles a plat, valeurs numeriques, CRLF.
Rappels : protocole 0=artnet 1=sacn 2=dmx ; net_mode 0=dhcp 1=statique ;
pixel_format 0=RGB 1=GRB 2=BRG 3=RGBW 4=GRBW 5=RGBWW (voir config.h ;
purement informatif pour l'instant, le driver reel emet toujours en GRB).
La carte doit etre en FAT16/FAT32 (pas d'exFAT).

Exemples :
    ./make_sd_config.py                          # sACN, defauts firmware, SD auto-detectee
    ./make_sd_config.py --ip 2.0.0.10 --leds 60
    ./make_sd_config.py --protocol artnet --universes 4,5,6,7
    ./make_sd_config.py --out /tmp/config.json   # sans carte SD (apercu)
"""
import argparse
import glob
import os
import sys

PROTOS = {"artnet": 0, "sacn": 1, "dmx": 2}
PIXEL_FORMATS = {"rgb": 0, "grb": 1, "brg": 2, "rgbw": 3, "grbw": 4, "rgbww": 5}
CONFIG_VERSION = 1
MAX_OUTPUTS = 4


def find_sd():
    user = os.environ.get("USER", "")
    mounts = [m for pat in (f"/media/{user}/*", "/run/media/*/*", "/mnt/sd*")
              for m in glob.glob(pat) if os.path.ismount(m)]
    if not mounts:
        sys.exit("Aucune carte SD montee trouvee — monter la carte ou passer "
                 "--out /chemin/config.json")
    if len(mounts) > 1:
        sys.exit("Plusieurs volumes montes : " + ", ".join(mounts) +
                 " — preciser avec --out <volume>/config.json")
    return os.path.join(mounts[0], "config.json")


def check_ip(s):
    parts = s.split(".")
    if len(parts) != 4 or not all(p.isdigit() and int(p) <= 255 for p in parts):
        raise argparse.ArgumentTypeError(f"IP invalide : {s}")
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", help="fichier de sortie (defaut : SD auto-detectee)")
    ap.add_argument("--protocol", choices=PROTOS, default="sacn")
    ap.add_argument("--dhcp", action="store_true", help="DHCP au lieu d'IP statique")
    ap.add_argument("--ip",      type=check_ip, default="2.0.0.4")
    ap.add_argument("--netmask", type=check_ip, default="255.255.255.0")
    ap.add_argument("--gateway", type=check_ip, default="2.0.0.1")
    ap.add_argument("--dns",     type=check_ip, default="8.8.8.8")
    ap.add_argument("--dmx-uart", type=int, choices=(1, 2), default=2)
    ap.add_argument("--universes", default="0,1,2,3",
                    help="univers des 4 sorties, ex : 0,1,2,3")
    ap.add_argument("--leds", type=int, default=120,
                    help="LEDs par sortie (1-512, defaut 120)")
    ap.add_argument("--max-current", type=int, default=5, help="limite en A (0-5)")
    ap.add_argument("--disable", default="",
                    help="sorties a desactiver, ex : 3,4")
    ap.add_argument("--pixel-format", choices=PIXEL_FORMATS, default="grb",
                    help="format pixel des 4 sorties (defaut : grb — WS2815 standard)")
    args = ap.parse_args()

    universes = [int(u) for u in args.universes.split(",")]
    if len(universes) != MAX_OUTPUTS:
        sys.exit(f"--universes attend {MAX_OUTPUTS} valeurs")
    if not 1 <= args.leds <= 512:
        sys.exit("--leds : 1-512")
    disabled = {int(d) for d in args.disable.split(",") if d}

    lines = [
        "{",
        f'  "version": {CONFIG_VERSION},',
        f'  "net_mode": {0 if args.dhcp else 1},',
        f'  "ip": "{args.ip}",',
        f'  "netmask": "{args.netmask}",',
        f'  "gateway": "{args.gateway}",',
        f'  "dns": "{args.dns}",',
        f'  "protocol": {PROTOS[args.protocol]},',
        f'  "dmx_uart": {args.dmx_uart},',
    ]
    for i in range(MAX_OUTPUTS):
        en = 0 if (i + 1) in disabled else 1
        sep = "," if i < MAX_OUTPUTS - 1 else ""
        lines.append(f'  "output{i}": {{"enabled":{en}, "universe":{universes[i]}, '
                     f'"led_count":{args.leds}, "max_current_A":{args.max_current}, '
                     f'"dmx_channel":1, "pixel_format":{PIXEL_FORMATS[args.pixel_format]}}}{sep}')
    lines.append("}")
    content = "\r\n".join(lines) + "\r\n"

    out = args.out or find_sd()
    with open(out, "w", newline="") as f:
        f.write(content)
    os.sync()

    print(f"config.json ecrit : {out}\n")
    print(content.replace("\r\n", "\n"), end="")
    print(f"\nprotocole={args.protocol} reseau={'dhcp' if args.dhcp else args.ip} "
          f"univers={universes} leds={args.leds}/sortie")
    if not args.out:
        print("Ejecter proprement la carte (umount) avant de la retirer.")


if __name__ == "__main__":
    main()
