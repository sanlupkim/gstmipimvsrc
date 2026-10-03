#!/bin/bash
# Is the camera's ISP in the RAW output path?
#
# If it is, the user LUT can carry the tone curve that gstmipimvsrc currently applies on the host, and
# the camera can be asked for RAW8 directly: no unpacking, no table lookup, and 1 byte per pixel on the
# MIPI link instead of 1.5. If it is not, the LUT only touches processed output and the host has to keep
# doing the work.
#
# The test uploads a gamma 0.5 curve, which is unmistakable - a pixel at 1024 becomes 2048, one at 256
# becomes 1024 - captures before and after, and compares. Everything is restored at the end.
#
# Run it in daylight on a scene that does not move. Nothing here writes flash.
#
# Usage: ./test_lut_path.sh [i2c_bus] [video_node] [width] [height]

set -u

BUS=${1:-4}
VIDEO=${2:-/dev/video8}
W=${3:-2400}
H=${4:-2064}
MV=${MV_TOOLS:-$HOME/raspberrypi_v4l2/mv_tools_rpi}
OUT=$(mktemp -d /tmp/lutpath.XXXXXX)

cd "$MV" || { echo "Set MV_TOOLS to the mv_tools_rpi directory"; exit 1; }

i2c() { ./mv_mipi_i2c.sh "$@" -b "$BUS" 2>&1 | grep -v "^Executing:"; }

echo "=== state before ==="
i2c -r -f lutenable
i2c -r -f gammaenable
i2c -r -f pixelformat
i2c -r -f blacklevel

WAS_LUT=$(i2c -r -f lutenable | grep -oE '[0-9]+$')
WAS_FMT=$(i2c -r -f pixelformat | grep -oE '[0-9]+$')
echo "(will restore lutenable=${WAS_LUT:-0} pixelformat=${WAS_FMT:-2})"

# --- Phase 1: what domain does the LUT live in? One I2C read per format, no capture needed.
#
# The documentation says the sensor is always 12 bit and "the logic unit performs data rounding to
# achieve a different pixel format", so the question is whether the LUT sits before that rounding or
# after it. LUT Count (0xC74) is read-only and the camera reports what it wants:
#
#   stays 4096 in Mono8  -> the LUT works on 12 bit values, before the rounding. This is the useful
#                           case: upload our gamma, ask for Mono8, and the camera hands back exactly
#                           what convertRows would have produced.
#   becomes 256 in Mono8 -> the LUT is applied after the reduction, on values that have already lost
#                           the faint end. Useless for us.
#   stays 0 or errors    -> no LUT in that mode at all.
#
# Run this with RMS stopped: it changes pixelformat briefly behind the driver's back.
echo
echo "=== phase 1: LUT size per pixel format (0xC74, read only) ==="
printf '%-10s %-14s %s\n' "format" "LUT entries" "meaning"
for FMT in 2 1 0; do
    NAME=$(case $FMT in 0) echo Mono8;; 1) echo Mono10;; 2) echo Mono12;; esac)
    i2c -w -f pixelformat -p1 $FMT >/dev/null
    sleep 0.3
    CNT=$(./i2c_4read "$BUS" 0x3b 0xC74 2>/dev/null | tr -d ' ')
    case "$CNT" in
        4096) M="12 bit domain" ;;
        1024) M="10 bit domain" ;;
        256)  M="8 bit domain - after the rounding, no use to us" ;;
        0|"") M="no LUT reported" ;;
        *)    M="unexpected" ;;
    esac
    printf '%-10s %-14s %s\n' "$NAME" "${CNT:-?}" "$M"
done

i2c -w -f pixelformat -p1 "${WAS_FMT:-2}" >/dev/null
echo "(pixelformat restored to ${WAS_FMT:-2})"
echo
echo "If Mono8 reported 4096, the curve can move into the camera and the element becomes a memcpy."
echo "Phase 2 below confirms the LUT actually reaches the RAW data at all."

echo
echo "=== phase 2: does the LUT reach RAW data? capture with it off ==="
i2c -w -f lutenable -p1 0 >/dev/null
grab "$OUT/before.raw"

echo
echo "=== upload gamma 0.5 and enable ==="
python3 - "$OUT/g05.txt" <<'PY'
import sys
# 12 bit in, 12 bit out, exactly the shape of the vendor's own samples
vals = [round(4095*(i/4095)**0.5) for i in range(4096)]
open(sys.argv[1], 'w').write(','.join(str(v) for v in vals))
PY
./lut_rw "$BUS" 0x3b w "$OUT/g05.txt" && echo "    LUT uploaded"
i2c -w -f lutenable -p1 1 >/dev/null
i2c -r -f lutenable

echo
echo "=== capture with the LUT on ==="
grab "$OUT/after.raw"

echo
echo "=== verdict ==="
python3 - "$OUT/before.raw" "$OUT/after.raw" "$W" "$H" <<'PY'
import sys
import numpy as np

before, after, W, H = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])

def frame(path):
    data = np.fromfile(path, np.uint8)
    # Y12P: 3 bytes per 2 pixels. Take the last whole frame, stride from the file size.
    fb = len(data)//max(len(data)//(W*3//2*H), 1)
    d = data[-(W*3//2*H):].reshape(H, -1)[:, :W*3//2].reshape(H, -1, 3).astype(np.uint16)
    lo = (d[:, :, 0] << 4) | (d[:, :, 2] >> 4)
    hi = (d[:, :, 1] << 4) | (d[:, :, 2] & 0x0F)
    return np.stack([lo, hi], 2).reshape(H, W)

try:
    a, b = frame(before), frame(after)
except Exception as e:
    print("could not unpack:", e); sys.exit(1)

print("  median before %.0f   after %.0f" % (np.median(a), np.median(b)))

# Sample where the before-image has a given level and see what the after-image has there
hit = False
for lvl in (128, 256, 512, 1024, 2048):
    m = (a > lvl - 8) & (a < lvl + 8)
    if m.sum() < 200:
        continue
    got = np.median(b[m])
    want = round(4095*(lvl/4095)**0.5)
    print("  level %4d -> measured %4.0f, gamma 0.5 predicts %4d, unchanged would be %4d"
          % (lvl, got, want, lvl))
    hit = True

if not hit:
    print("  not enough pixels at the sampled levels - point the camera at something with more range")
else:
    d = float(np.median(np.abs(b.astype(int) - a.astype(int))))
    print()
    if d < 4:
        print("  ISP IS NOT in the RAW path: the LUT changed nothing (median |diff| %.1f)." % d)
        print("  The host has to keep applying the curve.")
    else:
        print("  ISP IS in the RAW path (median |diff| %.1f)." % d)
        print("  The curve can move into the camera. Next question is whether pixelformat=RAW8 then")
        print("  gives the LUT output's top 8 bits - rerun with RAW8 configured in media-ctl.")
PY

echo
echo "=== restoring ==="
./lut_rw "$BUS" 0x3b w lut_samples/lut_linear.txt >/dev/null 2>&1 && echo "    LUT reset to linear"
i2c -w -f lutenable -p1 "${WAS_LUT:-0}" >/dev/null
i2c -r -f lutenable
echo "    nothing was written to flash, so a power cycle also clears this"
echo "    captures left in $OUT"
