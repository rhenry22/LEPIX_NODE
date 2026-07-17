#!/usr/bin/env python3
"""Client UART pour la console de configuration du LEPIX_NODE
(USART1 / port DB9, 115200 8N1 — module Core/Src/cli.c).

Sans dependance : utilise termios/select de la bibliotheque standard
(Linux). Deux modes :

  Interactif (mini-terminal, quitter avec Ctrl-]) :
      ./uart_client.py [-p /dev/ttyUSB0]

  Une commande puis sortie (scriptable) :
      ./uart_client.py -c "show"
      ./uart_client.py -c "ip 2.0.0.10" -c "save" -c "reboot"

Commandes du node : help, show, ip/mask/gw A.B.C.D, dhcp on|off,
proto artnet|sacn|dmx, out N ..., save, defaults, reboot.
"""
import argparse
import glob
import os
import select
import sys
import termios
import time
import tty


def find_port():
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyS0"):
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    sys.exit("Aucun port serie trouve (essayer --port /dev/ttyUSBx)")


def open_port(path, baud):
    speed = getattr(termios, f"B{baud}", None)
    if speed is None:
        sys.exit(f"Baudrate non supporte : {baud}")
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    # raw 8N1, pas de controle de flux
    attrs[0] = 0                                    # iflag
    attrs[1] = 0                                    # oflag
    attrs[2] = (termios.CS8 | termios.CREAD | termios.CLOCAL)  # cflag
    attrs[3] = 0                                    # lflag
    attrs[4] = attrs[5] = speed
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def read_until_prompt(fd, timeout=2.0):
    """Lit jusqu'au prompt '> ' ou expiration du delai."""
    buf = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            chunk = os.read(fd, 4096)
            if chunk:
                buf += chunk
                if buf.endswith(b"> "):
                    break
    return buf.decode("utf-8", errors="replace")


def one_shot(fd, commands):
    read_until_prompt(fd, timeout=0.3)          # purge banniere/prompt
    for cmd in commands:
        os.write(fd, cmd.encode() + b"\r")
        out = read_until_prompt(fd)
        # retire l'echo de la commande et le prompt final
        for line in out.replace("\r", "").split("\n"):
            line = line.strip()
            if line and line != cmd and line != ">":
                print(line.removesuffix("> ").rstrip() or line)


def interactive(fd, port):
    print(f"Connecte a {port} — 115200 8N1. Quitter : Ctrl-]")
    os.write(fd, b"\r")                          # provoque l'affichage du prompt
    saved = termios.tcgetattr(sys.stdin.fileno())
    try:
        tty.setraw(sys.stdin.fileno())
        while True:
            r, _, _ = select.select([fd, sys.stdin], [], [])
            if fd in r:
                data = os.read(fd, 4096)
                if data:
                    sys.stdout.buffer.write(data)
                    sys.stdout.buffer.flush()
            if sys.stdin in r:
                key = os.read(sys.stdin.fileno(), 1)
                if key == b"\x1d":               # Ctrl-]
                    break
                os.write(fd, key)
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, saved)
        print("\nDeconnecte.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="port serie (defaut : auto-detection)")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-c", "--command", action="append", default=[],
                    help="commande a envoyer puis quitter (repetable)")
    args = ap.parse_args()

    port = args.port or find_port()
    fd = open_port(port, args.baud)
    try:
        if args.command:
            one_shot(fd, args.command)
        else:
            interactive(fd, port)
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
