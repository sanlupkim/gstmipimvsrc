#!/bin/bash
# Does the camera's internal gamma stage act on the 12 bit RAW output?
#
# It matters because if it does, it composes with whatever curve gstmipimvsrc or the user LUT applies:
# 0.80 under a 0.50 curve is a net 0.40, and photometry then drifts about 1 mag across 4 mag while the
# image still looks right. Measurement on real star fields says the stage was NOT acting on the nights
# analysed - linearised aperture sums come out linear at an exponent of 0.51, not 0.40 - but that is an
# inference from photometry, and this measures the path directly.
#
# WHAT THE FIRST VERSION GOT WRONG, because it is the reason this one is shaped as it is:
#
#   It left the enable bit exactly as found, on the assumption the bit was on. With cam-curve in use it
#   is not: uploading a LUT turns the gamma bit off on purpose, so two curves are not applied in
#   series. Changing the gamma VALUE while the stage is disabled measures nothing.
#
#   It then wrote a value without checking it took. The camera kept 0.80 when asked for 2.50 - clamped
#   or refused, there is no documented range - so the two captures were taken under identical
#   settings.
#
#   And it decided from the median absolute difference between two frames, which at gain 0 is sensor
#   noise. It read 16, called that "live", and printed per level lines showing every level mapping to
#   itself, i.e. its own data contradicting its verdict. The per level lines are the measurement; the
#   difference between two frames of a live scene is not.
#
# So this version enables the stage, disables the LUT for the duration, finds two gamma values the
# camera actually accepts, and decides from how the levels moved. Everything is restored on exit,
# including on Ctrl+C. Nothing is written to flash.
#
# Run it in DAYLIGHT, on a scene that does not move, with RMS stopped. Daylight is required, not
# preferred: the test asks what a given input level becomes, so the frame has to contain a range of mid
# levels. A night frame at 40 ms and 30 dB is background and stars with nothing in between.
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
    [ -s "$1" ]
}

median() {
    python3 - "$1" "$W" "$H" <<'PY'
import sys
import numpy as np
W, H = int(sys.argv[2]), int(sys.argv[3])
d = np.fromfile(sys.argv[1], np.uint8)
d = d[-(W*3//2*H):].reshape(H, -1)[:, :W*3//2].reshape(H, -1, 3).astype(np.uint16)
lo = (d[:, :, 0] << 4) | (d[:, :, 2] >> 4)
hi = (d[:, :, 1] << 4) | (d[:, :, 2] & 0x0F)
print(int(np.median(np.stack([lo, hi], 2))))
PY
}

# Read a gamma value back as the camera holds it
read_gamma() { i2c -r -f gamma | grep -oE '[0-9]+\.[0-9]+' | tail -1; }

# Write a gamma and report what stuck. Returns non-zero if the camera did not take it.
set_gamma() {
    i2c -w -f gamma -p1 "$1" >/dev/null
    sleep 0.3
    local got
    got=$(read_gamma)
    [ "$(awk -v a="$got" -v b="$1" 'BEGIN{print (a-b<0.02 && b-a<0.02) ? 1 : 0}')" = "1" ]
}

echo "=== state before ==="
i2c -r -f gammaenable
i2c -r -f gamma
i2c -r -f lutenable
i2c -r -f pixelformat

WAS_EN=$(i2c -r -f gammaenable | grep -oE '[0-9]+' | tail -1)
WAS_G=$(read_gamma)
WAS_LUT=$(i2c -r -f lutenable | grep -oE '[0-9]+' | tail -1)
WAS_EXP=$(i2c -r -f metime | grep -oE '[0-9]+' | tail -1)
WAS_GAIN=$(i2c -r -f mgain | grep -oE '[0-9]+\.[0-9]+' | tail -1)
echo "(will restore gammaenable=${WAS_EN:-0} gamma=${WAS_G:-1.00} lutenable=${WAS_LUT:-0}"
echo "              metime=${WAS_EXP:-40000} mgain=${WAS_GAIN:-0.0})"

# Writes back only what was actually read. Substituting a default for a value that failed to parse is
# how the previous version turned the user LUT off: the read said 1, the parse gave nothing, and the
# default said 0. A setting we cannot name is a setting we must not write.
put_back() {
    local what="$1" value="$2"

    if [ -z "$value" ]; then
        echo "    NOT restoring $what: its original value was never read. Check it by hand."
        return
    fi

    i2c -w -f "$what" -p1 "$value" >/dev/null
    # The ISP enables share one register, so two of these in a row can race - the second read can still
    # see the value from before the first write, and writing that back undoes it. A pause between them,
    # and a check afterwards.
    sleep 0.3
}

if [ -z "${WAS_EN:-}" ] || [ -z "${WAS_LUT:-}" ] || [ -z "${WAS_G:-}" ]; then
    echo
    echo "    Could not read the current gamma/LUT state, so there would be nothing to restore."
    echo "    Refusing to change anything."
    exit 1
fi

restore() {
    echo
    echo "=== restoring ==="
    put_back gamma       "${WAS_G:-}"
    put_back gammaenable "${WAS_EN:-}"
    put_back lutenable   "${WAS_LUT:-}"
    put_back metime      "${WAS_EXP:-}"
    put_back mgain       "${WAS_GAIN:-}"

    i2c -r -f gamma
    local en lut
    en=$(i2c -r -f gammaenable | grep -oE '[0-9]+' | tail -1)
    lut=$(i2c -r -f lutenable | grep -oE '[0-9]+' | tail -1)
    echo "r gamma enable is $en"
    echo "r lut enable is $lut"

    if [ -n "${WAS_EN:-}" ] && [ "$en" != "$WAS_EN" ]; then
        echo "    WARNING: gammaenable came back as $en, not $WAS_EN. Retrying once..."
        i2c -w -f gammaenable -p1 "$WAS_EN" >/dev/null
        sleep 0.3
        i2c -r -f gammaenable
    fi

    if [ -n "${WAS_LUT:-}" ] && [ "$lut" != "$WAS_LUT" ]; then
        echo "    WARNING: lutenable came back as $lut, not $WAS_LUT. Retrying once..."
        i2c -w -f lutenable -p1 "$WAS_LUT" >/dev/null
        sleep 0.3
        i2c -r -f lutenable
    fi

    echo "    nothing was written to flash, so a power cycle also clears this"
}
trap restore EXIT INT TERM

# The LUT occupies the same tone mapping job. Leaving it on would mean measuring which of the two wins
# rather than whether gamma reaches the RAW data, so it goes off for the duration.
echo
echo "=== preparing: LUT off, gamma stage on ==="
i2c -w -f lutenable -p1 0 >/dev/null
sleep 0.3
i2c -w -f gammaenable -p1 1 >/dev/null
sleep 0.3
i2c -r -f lutenable
i2c -r -f gammaenable

if [ "$(i2c -r -f gammaenable | grep -oE '[0-9]+' | tail -1)" != "1" ]; then
    echo
    echo "    The camera refused to enable the gamma stage, so there is nothing to measure."
    exit 1
fi

# Gain to zero: it costs headroom at the top and buys nothing here. Then step the exposure until the
# scene sits in the middle of the range, because a saturated or black frame makes the test vacuous.
i2c -w -f mgain -p1 0 >/dev/null

echo
echo "=== finding a usable exposure (gain 0) ==="
GOOD_EXP=""
for EXP in 20 50 100 200 500 1000 2000 5000 10000; do
    i2c -w -f metime -p1 $EXP >/dev/null
    sleep 0.4
    grab "$OUT/probe.raw" || continue
    MED=$(median "$OUT/probe.raw")
    printf '    %6s us -> median %s\n' "$EXP" "${MED:-?}"
    if [ -n "${MED:-}" ] && [ "$MED" -ge 800 ] && [ "$MED" -le 2500 ]; then
        GOOD_EXP=$EXP
        break
    fi
done

if [ -z "$GOOD_EXP" ]; then
    echo
    echo "    No exposure put the scene between 800 and 2500 of 4095. Too dark to run this now, or"
    echo "    the lens is capped. Stopping rather than printing a verdict it cannot support."
    exit 1
fi
echo "    using ${GOOD_EXP} us"

# Two values the camera actually accepts, as far apart as it allows. No documented range, so this is
# found by trying: the first version assumed 2.50 would be taken and it was not.
echo
echo "=== finding two gamma values the camera accepts ==="
G_LOW=""
G_HIGH=""
for G in 0.30 0.40 0.50 0.60 0.70 0.80 0.90 1.00 1.20 1.50 2.00 2.50 3.00; do
    if set_gamma "$G"; then
        printf '    %.2f accepted\n' "$G"
        [ -z "$G_LOW" ] && G_LOW=$G
        G_HIGH=$G
    else
        printf '    %.2f refused (camera kept %s)\n' "$G" "$(read_gamma)"
    fi
done

if [ -z "$G_LOW" ] || [ "$G_LOW" = "$G_HIGH" ]; then
    echo
    echo "    The camera accepts at most one gamma value, so the curve cannot be varied and this"
    echo "    test cannot run. Nothing is concluded."
    exit 1
fi

RATIO=$(awk -v a="$G_LOW" -v b="$G_HIGH" 'BEGIN{printf "%.3f", b/a}')
echo "    using ${G_LOW} and ${G_HIGH} (exponent ratio ${RATIO})"

echo
echo "=== capture at gamma ${G_LOW} ==="
set_gamma "$G_LOW" || { echo "    could not set it back"; exit 1; }
sleep 0.5
grab "$OUT/low.raw" || { echo "    capture FAILED - is $VIDEO right, and is RMS stopped?"; exit 1; }
echo "    median $(median "$OUT/low.raw")"

echo
echo "=== capture at gamma ${G_HIGH} ==="
set_gamma "$G_HIGH" || { echo "    could not set it"; exit 1; }
sleep 0.5
grab "$OUT/high.raw" || { echo "    capture FAILED"; exit 1; }
echo "    median $(median "$OUT/high.raw")"

echo
echo "=== verdict ==="
python3 - "$OUT/low.raw" "$OUT/high.raw" "$W" "$H" "$G_LOW" "$G_HIGH" <<'PY'
import sys
import numpy as np

low, high, W, H = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
g_low, g_high = float(sys.argv[5]), float(sys.argv[6])

def frame(path):
    d = np.fromfile(path, np.uint8)
    d = d[-(W*3//2*H):].reshape(H, -1)[:, :W*3//2].reshape(H, -1, 3).astype(np.uint16)
    lo = (d[:, :, 0] << 4) | (d[:, :, 2] >> 4)
    hi = (d[:, :, 1] << 4) | (d[:, :, 2] & 0x0F)
    return np.stack([lo, hi], 2).reshape(H, W)

a, b = frame(low), frame(high)

# A level L seen under g_low came from a linear input x = (L/4095)**(1/g_low), so under g_high it
# should read 4095*x**g_high = 4095*(L/4095)**(g_high/g_low). An inert stage leaves it at L.
k = g_high/g_low
print("  a live stage maps level L to 4095*(L/4095)^%.3f, an inert one leaves it at L\n" % k)
print("  %8s %10s %10s %10s   %s" % ("level", "measured", "live", "inert", "closer to"))

votes_live = votes_inert = 0
for lvl in (256, 512, 1024, 2048, 3072):
    m = (a > lvl - 8) & (a < lvl + 8)
    if m.sum() < 200:
        continue
    got = float(np.median(b[m]))
    live = 4095.0*(lvl/4095.0)**k
    which = "live" if abs(got - live) < abs(got - lvl) else "inert"
    votes_live += which == "live"
    votes_inert += which == "inert"
    print("  %8d %10.0f %10.0f %10d   %s" % (lvl, got, live, lvl, which))

print()
if votes_live + votes_inert == 0:
    print("  NO VERDICT: no mid level had enough pixels to compare. Point the camera at something")
    print("  with a range of brightness and rerun.")
    sys.exit(1)

if votes_live > votes_inert:
    print("  GAMMA IS LIVE in the RAW path (%d of %d levels moved as the curve predicts)."
          % (votes_live, votes_live + votes_inert))
    print("  It therefore composes with any curve set in the element or the LUT, and cam-gamma=0")
    print("  is required before any night where photometry matters.")
else:
    print("  GAMMA IS INERT in the RAW path (%d of %d levels did not move)."
          % (votes_inert, votes_live + votes_inert))
    print("  The stage does not reach 12 bit data even when enabled, so the enable bit is harmless")
    print("  here and the photometry in the report is unaffected. This agrees with section 17 of")
    print("  the IMX178 analysis, which found the stage acting on the 8 bit path only.")

print()
print("  Read the per level lines, not any single difference figure: two frames of a live scene")
print("  differ by sensor noise alone, and at gain 0 that is a median of some 16 ADU.")
PY

echo
echo "    captures left in $OUT"
