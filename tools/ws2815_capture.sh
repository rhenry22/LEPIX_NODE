#!/usr/bin/env bash
# Capture des 4 sorties WS2815 du LEPIX_NODE avec un analyseur logique
# 24 MHz 8 canaux compatible fx2lafw (clones Saleae Logic, USB 0925:3881),
# puis validation automatique des timings et des trames.
#
# Câblage analyseur -> carte (masse commune obligatoire) :
#   CH1 / D0 -> PD15    CH2 / D1 -> PD13
#   CH3 / D2 -> PD11    CH4 / D3 -> PD9
#   GND      -> GND carte
#
# Usage : ./ws2815_capture.sh [duree_ms] [leds_par_sortie]
#         (défauts : 500 ms, 120 LEDs)
set -euo pipefail

DUR_MS="${1:-500}"
LEDS="${2:-120}"
OUT="${OUT:-/tmp/ws2815_capture.bin}"
HERE="$(cd "$(dirname "$0")" && pwd)"

command -v sigrok-cli >/dev/null || {
    echo "sigrok-cli manquant : sudo apt install sigrok-cli" >&2
    exit 1
}

echo "Capture ${DUR_MS} ms @ 24 MHz sur D0-D3 -> ${OUT}"
sigrok-cli -d fx2lafw --config samplerate=24m -C D0,D1,D2,D3 \
    --time "${DUR_MS}" -O binary -o "${OUT}"

python3 "${HERE}/ws2815_validate.py" "${OUT}" \
    --samplerate 24000000 --leds "${LEDS}"
