#!/bin/bash
# Is the camera's internal gamma stage in the RAW output path?
#
# probe reports bit 5 of 0xC00 set and the Gamma register (0xC60) holding 0.80 on a camera nobody asked
# to do that. If that stage acts on the 12 bit data, it composes with whatever curve gstmipimvsrc or the
# user LUT applies: 0.80 under a 0.50 curve gives a net exponent of 0.40, and photometry drifts about
# 1 mag across a 4 mag range - silently, because the image still looks right.
#
# Measurement on real frames says the stage was NOT acting: linearising the aperture sums of five bright
# stars comes out linear at an exponent of 0.51, not 0.40. So either the bit was off on those nights and
# is on now, or it has always been on and is inert in the RAW path. Nothing found so far writes that bit
# - the kernel driver never touches 0xC00, set_params.sh does not, and both the vendor's dpcenable and
# this element's read-modify-write preserve it - which points at inert, but points is not knows.
#
# This settles it the same way the LUT question was settled: change the gamma VALUE by a large factor
# with the bit left alone, capture before and after, and look at whether RAW12 moved.
#
# Run it in daylight on a scene that does not move, with RMS stopped. Nothing here writes flash.
#
# Usage: ./test_gamma_path.sh [i2c_bus] [video_node] [width] [height]

set -u

BUS=${1:-4}
VIDEO=${2:-/dev/video8}
W=${3:-2400}
H=${4:-2064}
MV=${MV_TOOLS:-$HOME/raspberrypi_v4l2/mv_tools_rpi}
OUT=$(mktemp -d /tmp/gammapath.XXXXXX)

cd "$MV" || { echo "Set MV_TOOLS to the mv_tools_rpi directory"; exit 1; }

i2c() { ./mv_mipi_i2c.sh "$@" -b "$BUS" 2>&1 | grep -v "^Executing:"; }

grab() {
    v4l2-ctl -d "$VIDEO" --set-fmt-video=width=$W,height=$H \
             --stream-mmap --stream-count=3 --stream-to="$1" >/dev/null 2>&1
    if [ -s "$1" ]; then
        ls -l "$1" | awk '{print "    captured", $5, "bytes"}'
    else
        echo "    capture FAILED - is $VIDEO right, and is RMS stopped?"
        return 1
    fi
}

echo "=== state before ==="
i2c -r -f gammaenable
i2c -r -f gamma
i2c -r -f lutenable
i2c -r -f pixelformat

WAS_EN=$(i2c -r -f gammaenable | grep -oE '[0-9]+$')
WAS_G=$(i2c -r -f gamma | grep -oE '[0-9]+\.[0-9]+')
echo "(will restore gammaenable=${WAS_EN:-0} gamma=${WAS_G:-1.00})"

restore() {
    echo
    echo "=== restoring ==="
    i2c -w -f gamma -p1 "${WAS_G:-1.00}" >/dev/null
    i2c -w -f gammaenable -p1 "${WAS_EN:-0}" >/dev/null
    i2c -r -f gamma
    i2c -r -f gammaenable
    echo "    nothing was written to flash, so a power cycle also clears this"
}
trap restore EXIT INT TERM

# The bit stays exactly as found. Only the value changes, from 0.80 to 2.50 - which is on the other side
# of 1.0, so if the stage acts at all the frame gets DARKER rather than brighter, and no clipping at the
# top can hide it. A stage that is enabled but not wired to this data will not move a single pixel.
echo
echo "=== capture with gamma as found (value ${WAS_G:-?}, enable ${WAS_EN:-?}) ==="
grab "$OUT/before.raw"

echo
echo "=== same enable bit, value changed to 2.50 ==="
i2c -w -f gamma -p1 2.50 >/dev/null
i2c -r -f gamma
sleep 0.5
grab "$OUT/after.raw"

echo
echo "=== verdict ==="
python3 - "$OUT/before.raw" "$OUT/after.raw" "$W" "$H" "${WAS_G:-0.80}" <<'PY'
import sys
import numpy as np

before, after, W, H, g0 = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), float(sys.argv[5])

def frame(path):
    data = np.fromfile(path, np.uint8)
    d = data[-(W*3//2*H):].reshape(H, -1)[:, :W*3//2].reshape(H, -1, 3).astype(np.uint16)
    lo = (d[:, :, 0] << 4) | (d[:, :, 2] >> 4)
    hi = (d[:, :, 1] << 4) | (d[:, :, 2] & 0x0F)
    return np.stack([lo, hi], 2).reshape(H, W)

try:
    a, b = frame(before), frame(after)
except Exception as e:
    print("could not unpack:", e); sys.exit(1)

print("  median before %.0f   after %.0f" % (np.median(a), np.median(b)))

# If the stage is live, a pixel that read L under exponent g0 should read 4095*(L/4095)**(2.50/g0).
for lvl in (256, 512, 1024, 2048):
    m = (a > lvl - 8) & (a < lvl + 8)
    if m.sum() < 200:
        continue
    want = round(4095*(lvl/4095.0)**(2.50/g0))
    print("  level %4d -> measured %4.0f, a live stage predicts %4d, inert would be %4d"
          % (lvl, np.median(b[m]), want, lvl))

d = float(np.median(np.abs(b.astype(int) - a.astype(int))))
print()
if d < 4:
    print("  GAMMA IS INERT in the RAW path (median |diff| %.1f)." % d)
    print("  The bit reading 1 is harmless and the photometry in the report is unaffected.")
    print("  cam-gamma=0 is then optional - it costs nothing, but it fixes nothing either.")
else:
    print("  GAMMA IS LIVE in the RAW path (median |diff| %.1f)." % d)
    print("  Then it composes with any curve set here, and cam-gamma=0 is REQUIRED before")
    print("  any night that matters. It also means the bit was off during the nights already")
    print("  analysed, since those measure a net exponent of 0.51 rather than 0.40 - so")
    print("  something switched it on, and that is worth finding.")

print()
print("  Caveat: this compares two frames of a live scene. If the light or the scene moved")
print("  between them, |diff| is large for a reason that has nothing to do with gamma. The")
print("  per-level lines above are the ones to read - they say WHICH way it moved.")
PY

echo
echo "    captures left in $OUT"
