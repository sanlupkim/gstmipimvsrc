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
# Run it in DAYLIGHT, on a scene that does not move, with RMS stopped. Nothing here writes flash.
#
# Daylight is not a preference, it is the requirement: the test compares what a given input level
# becomes, so the frame has to contain a range of mid levels. A night frame at 40 ms and 30 dB is sky
# background and stars - nothing between - and the comparison would have nothing to measure while
# still printing a verdict. For the same reason the script sets its own exposure and gain rather than
# using whatever was left configured: at night settings a daylit scene is saturated end to end, and a
# curve applied to 4095 returns 4095, which looks exactly like an inert stage.
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
WAS_EXP=$(i2c -r -f metime | grep -oE '[0-9]+' | tail -1)
WAS_GAIN=$(i2c -r -f mgain | grep -oE '[0-9]+\.[0-9]+' | tail -1)
echo "(will restore gammaenable=${WAS_EN:-0} gamma=${WAS_G:-1.00} metime=${WAS_EXP:-40000} mgain=${WAS_GAIN:-0.0})"

restore() {
    echo
    echo "=== restoring ==="
    i2c -w -f gamma -p1 "${WAS_G:-1.00}" >/dev/null
    i2c -w -f gammaenable -p1 "${WAS_EN:-0}" >/dev/null
    i2c -w -f metime -p1 "${WAS_EXP:-40000}" >/dev/null
    i2c -w -f mgain -p1 "${WAS_GAIN:-0.0}" >/dev/null
    i2c -r -f gamma
    i2c -r -f gammaenable
    i2c -r -f metime
    i2c -r -f mgain
    echo "    nothing was written to flash, so a power cycle also clears this"
}
trap restore EXIT INT TERM

# The bit stays exactly as found. Only the value changes, from 0.80 to 2.50 - which is on the other side
# of 1.0, so if the stage acts at all the frame gets DARKER rather than brighter, and no clipping at the
# top can hide it. A stage that is enabled but not wired to this data will not move a single pixel.
# Find an exposure that puts the scene in the middle of the range. Stepping rather than guessing,
# because how bright a daylit sky is depends on the sky, and a saturated or black frame makes the
# whole test vacuous. Gain goes to zero: it costs headroom at the top and buys nothing here.
i2c -w -f mgain -p1 0 >/dev/null

echo
echo "=== finding a usable exposure (gain 0) ==="
GOOD_EXP=""
for EXP in 20 50 100 200 500 1000 2000 5000 10000; do
    i2c -w -f metime -p1 $EXP >/dev/null
    sleep 0.4
    grab "$OUT/probe.raw" >/dev/null 2>&1 || continue
    MED=$(python3 - "$OUT/probe.raw" "$W" "$H" <<'PY'
import sys
import numpy as np
W, H = int(sys.argv[2]), int(sys.argv[3])
d = np.fromfile(sys.argv[1], np.uint8)
d = d[-(W*3//2*H):].reshape(H, -1)[:, :W*3//2].reshape(H, -1, 3).astype(np.uint16)
lo = (d[:, :, 0] << 4) | (d[:, :, 2] >> 4)
hi = (d[:, :, 1] << 4) | (d[:, :, 2] & 0x0F)
print(int(np.median(np.stack([lo, hi], 2))))
PY
)
    printf '    %6s us -> median %s\n' "$EXP" "${MED:-?}"
    if [ -n "${MED:-}" ] && [ "$MED" -ge 800 ] && [ "$MED" -le 2500 ]; then
        GOOD_EXP=$EXP
        break
    fi
done

if [ -z "$GOOD_EXP" ]; then
    echo
    echo "    No exposure put the scene between 800 and 2500 of 4095. Either it is too dark to run"
    echo "    this now, or the lens is capped. The test needs mid range levels to mean anything, so"
    echo "    stopping rather than printing a verdict it cannot support."
    exit 1
fi

echo "    using ${GOOD_EXP} us"

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
sampled = 0
for lvl in (256, 512, 1024, 2048):
    m = (a > lvl - 8) & (a < lvl + 8)
    if m.sum() < 200:
        continue
    want = round(4095*(lvl/4095.0)**(2.50/g0))
    print("  level %4d -> measured %4.0f, a live stage predicts %4d, inert would be %4d"
          % (lvl, np.median(b[m]), want, lvl))
    sampled += 1

d = float(np.median(np.abs(b.astype(int) - a.astype(int))))
print()
if not sampled:
    print("  NO VERDICT: the frame had no mid range levels to compare, so neither answer is")
    print("  supported. Point the camera at something with a range of brightness and rerun.")
    sys.exit(1)
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
