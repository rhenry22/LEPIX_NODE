#!/usr/bin/env python3
"""Validation des sorties WS2815 du LEPIX_NODE à partir d'une capture
d'analyseur logique 8 canaux (sigrok-cli -O binary : 1 octet par
échantillon, bit N = canal DN).

Câblage attendu (voir tools/ws2815_capture.sh) :
    D0 = CH1 = PD15   D1 = CH2 = PD13   D2 = CH3 = PD11   D3 = CH4 = PD9

Pour chaque canal, le script vérifie :
  - présence d'un signal (ligne ni bloquée à 0, ni à 1) ;
  - largeurs T0H / T1H et période bit (nominal 0,30 / 0,70 / 1,25 µs) ;
  - trames complètes de leds x 24 bits ;
  - temps de latch entre trames (> 280 µs à l'état bas) ;
et décode les premiers pixels (ordre GRB, MSB en premier — cf. ws2815.c).

Usage :
    ws2815_validate.py capture.bin [--samplerate 24000000] [--leds 120]
    ws2815_validate.py --selftest
"""
import argparse
import re
import statistics
import sys

# Tolérances en ns (datasheet WS2815 + marge d'échantillonnage 24 MHz = 42 ns)
T0H_RANGE    = (150, 500)
T1H_RANGE    = (550, 1000)
BIT_SPLIT_NS = 525          # sous ce seuil de largeur haute : bit '0'
PERIOD_RANGE = (1000, 1600)
GAP_SPLIT_NS = 50_000       # état bas plus long : fin de trame
LATCH_MIN_NS = 280_000

DEFAULT_MAP = "0=CH1_PD15,1=CH2_PD13,2=CH3_PD11,3=CH4_PD9"


def high_pulses(data, ch):
    """Retourne les impulsions hautes [(start, end)) du canal ch, plus
    le niveau constant si la ligne ne bouge pas ('0', '1' ou None)."""
    table = bytes(((b >> ch) & 1) for b in range(256))
    chdata = data.translate(table)
    pulses = [(m.start(), m.end()) for m in re.finditer(rb"\x01+", chdata)]
    if not pulses:
        return [], "0"
    if len(pulses) == 1 and pulses[0] == (0, len(chdata)):
        return [], "1"
    return pulses, None


def split_frames(pulses, ns_per_sample):
    """Groupe les impulsions en trames, séparées par un état bas > GAP_SPLIT_NS.
    Retourne [(pulses_de_la_trame, gap_apres_ns | None)]."""
    frames, current = [], []
    for i, (s, e) in enumerate(pulses):
        current.append((s, e))
        gap_ns = None
        if i + 1 < len(pulses):
            gap_ns = (pulses[i + 1][0] - e) * ns_per_sample
        if gap_ns is None or gap_ns > GAP_SPLIT_NS:
            frames.append((current, gap_ns))
            current = []
    return frames


def decode_bits(bits):
    """Bits GRB MSB-first -> liste de pixels (r, g, b)."""
    px = []
    for i in range(0, len(bits) - 23, 24):
        g = int("".join(bits[i:i + 8]), 2)
        r = int("".join(bits[i + 8:i + 16]), 2)
        b = int("".join(bits[i + 16:i + 24]), 2)
        px.append((r, g, b))
    return px


def analyze_channel(data, ch, name, fs, leds):
    ns = 1e9 / fs
    expected_bits = leds * 24
    pulses, stuck = high_pulses(data, ch)
    report = {"name": name, "pass": False, "lines": []}
    say = report["lines"].append

    if stuck is not None:
        say(f"ECHEC : ligne bloquée à l'état {'haut' if stuck == '1' else 'bas'}, "
            f"aucune impulsion détectée")
        return report

    frames = split_frames(pulses, ns)
    t0h, t1h, periods, latches = [], [], [], []
    complete, partial, bad = 0, 0, 0
    first_pixels = None

    for fi, (fp, gap_after) in enumerate(frames):
        bits = []
        for i, (s, e) in enumerate(fp):
            w = (e - s) * ns
            bits.append("0" if w < BIT_SPLIT_NS else "1")
            (t0h if w < BIT_SPLIT_NS else t1h).append(w)
            if i + 1 < len(fp):
                periods.append((fp[i + 1][0] - s) * ns)
        edge = fi == 0 or fi == len(frames) - 1  # tronquée par la capture ?
        if len(bits) == expected_bits:
            complete += 1
            if first_pixels is None:
                first_pixels = decode_bits(bits)
        elif edge:
            partial += 1
        else:
            bad += 1
            say(f"ECHEC : trame {fi} incomplète ({len(bits)} bits au lieu de "
                f"{expected_bits})")
        if gap_after is not None:
            latches.append(gap_after)

    if complete == 0:
        say(f"ECHEC : aucune trame complète de {expected_bits} bits "
            f"({len(frames)} trame(s) vue(s))")
        return report

    def stats(vals):
        return (f"min {min(vals):.0f} / moy {statistics.fmean(vals):.0f} / "
                f"max {max(vals):.0f} ns")

    bad_t0h = [w for w in t0h if not T0H_RANGE[0] <= w <= T0H_RANGE[1]]
    bad_t1h = [w for w in t1h if not T1H_RANGE[0] <= w <= T1H_RANGE[1]]
    bad_per = [p for p in periods if not PERIOD_RANGE[0] <= p <= PERIOD_RANGE[1]]
    bad_latch = [g for g in latches if g < LATCH_MIN_NS]

    say(f"trames : {complete} complète(s), {partial} tronquée(s) en bord de "
        f"capture, {bad} en erreur")
    if t0h:
        say(f"T0H ({len(t0h)} bits '0') : {stats(t0h)} — attendu ~300 ns")
    if t1h:
        say(f"T1H ({len(t1h)} bits '1') : {stats(t1h)} — attendu ~700 ns")
    if periods:
        say(f"période bit : {stats(periods)} — attendu ~1250 ns")
    if latches:
        say(f"latch inter-trames : min {min(latches) / 1000:.0f} µs — "
            f"attendu > {LATCH_MIN_NS // 1000} µs")
    for label, badv, total in (("T0H", bad_t0h, len(t0h)),
                               ("T1H", bad_t1h, len(t1h)),
                               ("période", bad_per, len(periods)),
                               ("latch", bad_latch, len(latches))):
        if badv:
            say(f"ECHEC : {len(badv)}/{total} valeurs {label} hors tolérance "
                f"(ex : {badv[0]:.0f} ns)")

    if first_pixels:
        shown = ", ".join(f"#{r:02X}{g:02X}{b:02X}" for r, g, b in first_pixels[:4])
        lit = sum(1 for p in first_pixels if p != (0, 0, 0))
        say(f"pixels (1ère trame) : {shown} … — {lit}/{leds} LED non éteintes")

    report["pass"] = not (bad or bad_t0h or bad_t1h or bad_per or bad_latch)
    return report


def parse_map(spec):
    out = []
    for item in spec.split(","):
        ch, name = item.split("=")
        out.append((int(ch), name))
    return out


# ---------------------------------------------------------------- selftest --

def synth_capture(fs, leds, frames, pixel_of):
    """Génère une capture synthétique : bit0 = 7 éch. haut, bit1 = 17 éch.
    haut, période 30 éch. (24 MHz), latch 300 µs. pixel_of(ch, i) -> (r,g,b)."""
    n_bit = 30
    latch = int(300e-6 * fs)
    frame_bits = leds * 24
    total = frames * (frame_bits * n_bit + latch) + 100
    buf = bytearray(total)
    for ch in range(4):
        pos = 50
        bits = []
        for i in range(leds):
            r, g, b = pixel_of(ch, i)
            for byte in (g, r, b):
                bits += [(byte >> (7 - k)) & 1 for k in range(8)]
        for _ in range(frames):
            p = pos
            for bit in bits:
                hi = 17 if bit else 7
                for k in range(hi):
                    buf[p + k] |= 1 << ch
                p += n_bit
            pos = p + latch
    return bytes(buf)


def selftest():
    fs, leds = 24_000_000, 8
    palette = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (18, 52, 86)]

    def pixel_of(ch, i):
        return palette[ch] if i == 0 else (i, 2 * i, 3 * i)

    data = synth_capture(fs, leds, 3, pixel_of)
    ok = True
    for ch, name in parse_map(DEFAULT_MAP):
        rep = analyze_channel(data, ch, name, fs, leds)
        expected = "#{:02X}{:02X}{:02X}".format(*palette[ch])
        decoded_ok = any(expected in ln for ln in rep["lines"])
        status = rep["pass"] and decoded_ok
        ok &= status
        print(f"[{'OK ' if status else 'KO '}] {name} — 1er pixel attendu {expected}")
        for ln in rep["lines"]:
            print(f"      {ln}")
    print("SELFTEST", "OK" if ok else "ECHEC")
    return 0 if ok else 1


# -------------------------------------------------------------------- main --

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", nargs="?", help="fichier binaire sigrok (-O binary)")
    ap.add_argument("--samplerate", type=float, default=24e6)
    ap.add_argument("--leds", type=int, default=120,
                    help="LEDs par sortie (défaut : 120)")
    ap.add_argument("--map", default=DEFAULT_MAP,
                    help="canaux à analyser, ex : 0=CH1_PD15,1=CH2_PD13")
    ap.add_argument("--selftest", action="store_true",
                    help="valide le décodeur sur une capture synthétique")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest())
    if not args.capture:
        ap.error("fichier de capture requis (ou --selftest)")

    with open(args.capture, "rb") as f:
        data = f.read()
    dur_ms = len(data) / args.samplerate * 1000
    print(f"{args.capture} : {len(data)} échantillons @ {args.samplerate / 1e6:g} MHz "
          f"({dur_ms:.0f} ms)\n")

    all_pass = True
    for ch, name in parse_map(args.map):
        rep = analyze_channel(data, ch, name, args.samplerate, args.leds)
        all_pass &= rep["pass"]
        print(f"[{'PASS' if rep['pass'] else 'FAIL'}] D{ch} — {name}")
        for ln in rep["lines"]:
            print(f"      {ln}")
        print()
    print("Bilan :", "TOUT OK" if all_pass else "AU MOINS UN CANAL EN ECHEC")
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
