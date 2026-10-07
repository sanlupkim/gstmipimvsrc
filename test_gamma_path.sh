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

# Unpacking depends on the pixel format, and getting that wrong is how the first working run of this
# test drew a conclusion about "the RAW path" from a RAW8 frame read as packed 12 bit. The byte count
# does not disambiguate on its own: three RAW8 frames and two Y12P frames are the same size.
unpack_py() {
    cat <<'PY'
import sys
import numpy as np

path, W, H, fmt = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
d = np.fromfile(path, np.uint8)

if fmt == 0:                                  # RAW8, one byte per pixel, scaled to 12 bit terms
    f = d[-(W*H):].reshape(H, W).astype(np.uint16)*16
elif fmt == 1:                                # Y10P, 5 bytes per 4 pixels
    b = d[-(W*5//4*H):].reshape(H, -1)[:, :W*5//4].reshape(H, -1, 5).astype(np.uint16)
    p = [((b[:, :, i] << 2) | ((b[:, :, 4] >> (6 - 2*i)) & 0x03)) << 2 for i in range(4)]
    f = np.stack(p, 2).reshape(H, W)
else:                                         # Y12P, 3 bytes per 2 pixels
    b = d[-(W*3//2*H):].reshape(H, -1)[:, :W*3//2].reshape(H, -1, 3).astype(np.uint16)
    lo = (b[:, :, 0] << 4) | (b[:, :, 2] >> 4)
    hi = (b[:, :, 1] << 4) | (b[:, :, 2] & 0x0F)
    f = np.stack([lo, hi], 2).reshape(H, W)
PY
}

# The format defaults to whatever the camera is in, because the exposure ladder runs before the format
# loop has picked one. Defaulted rather than required: leaving it to the caller is how this went out
# with one call passing a single argument, which under set -u killed the function and made the whole
# ladder report "?".
median() {
    local fmt="${2:-${WAS_FMT:-2}}"
    { unpack_py; echo 'print(int(np.median(f)))'; } | python3 - "$1" "$W" "$H" "$fmt"
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

# The analysis goes in its own file rather than a heredoc, so the format loop below can call it per
# format without the quoting getting out of hand.
cat > "$OUT/analyse.py" <<'PY'
import sys

import numpy as np

low, high, W, H, fmt = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])
name, g_low, g_high = sys.argv[6], float(sys.argv[7]), float(sys.argv[8])


def frame(path):
    """ Unpack according to the pixel format, scaled to 12 bit terms so one set of levels fits all.

        Getting this wrong is how the first working run drew a conclusion about "the RAW path" from a
        RAW8 frame read as packed 12 bit. The byte count does not disambiguate: three RAW8 frames and
        two Y12P frames of this size are the same number of bytes.
    """
    d = np.fromfile(path, np.uint8)

    if fmt == 0:
        return d[-(W*H):].reshape(H, W).astype(np.uint16)*16

    if fmt == 1:
        b = d[-(W*5//4*H):].reshape(H, -1)[:, :W*5//4].reshape(H, -1, 5).astype(np.uint16)
        p = [((b[:, :, i] << 2) | ((b[:, :, 4] >> (6 - 2*i)) & 0x03)) << 2 for i in range(4)]
        return np.stack(p, 2).reshape(H, W)

    b = d[-(W*3//2*H):].reshape(H, -1)[:, :W*3//2].reshape(H, -1, 3).astype(np.uint16)
    lo = (b[:, :, 0] << 4) | (b[:, :, 2] >> 4)
    hi = (b[:, :, 1] << 4) | (b[:, :, 2] & 0x0F)
    return np.stack([lo, hi], 2).reshape(H, W)


a, b = frame(low), frame(high)
k = g_high/g_low

print("  %s: a live stage maps level L to 4095*(L/4095)^%.3f, an inert one leaves it at L"
      % (name, k))
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
    print("  %s: NO VERDICT - no mid level had enough pixels to compare." % name)
    sys.exit(2)

if votes_live > votes_inert:
    print("  %s: GAMMA IS LIVE (%d of %d levels moved as the curve predicts)."
          % (name, votes_live, votes_live + votes_inert))
else:
    print("  %s: GAMMA IS INERT (%d of %d levels did not move)."
          % (name, votes_inert, votes_live + votes_inert))
PY

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
WAS_FMT=$(i2c -r -f pixelformat | grep -oE '[0-9]+' | tail -1)
ROI=$(i2c -r -f roi | grep -oE '[0-9]+,[0-9]+,[0-9]+,[0-9]+' | tail -1)
RX=${ROI%%,*}; R=${ROI#*,}; RY=${R%%,*}; R=${R#*,}; RW=${R%%,*}; RH=${R##*,}

# Changing the pixel format is the driver's business, through media-ctl, because it configures the
# CSI-2 receiver to match what the camera will send. Writing Pixel_Format over I2C changes only one
# side of that and the frames come back unusable.
MEDIA=${MEDIA_SCRIPT:-$MV/../rpi5_scripts/media_setting_rpi5.sh}
CAM=${CAM:-1}

fmt_name() { case $1 in 0) echo RAW8;; 1) echo RAW10;; 2) echo RAW12;; *) echo "fmt$1";; esac; }

set_format() {
    local want=$1 name
    name=$(fmt_name "$want")

    if [ ! -x "$MEDIA" ]; then
        return 1
    fi

    "$MEDIA" mvcam -fmt "$name" -x "$RX" -y "$RY" -w "$RW" -h "$RH" -c "$CAM" >/dev/null 2>&1
    sleep 1
    [ "$(i2c -r -f pixelformat | grep -oE '[0-9]+' | tail -1)" = "$want" ]
}
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

    # The format goes back the way it was changed, through the driver
    if [ -n "${WAS_FMT:-}" ] \
       && [ "$(i2c -r -f pixelformat | grep -oE '[0-9]+' | tail -1)" != "$WAS_FMT" ]; then
        if set_format "$WAS_FMT"; then
            echo "    pixel format restored to $(fmt_name "$WAS_FMT")"
        else
            echo "    WARNING: could not restore the pixel format to $(fmt_name "$WAS_FMT")."
            echo "             Run media_setting_rpi5.sh yourself before starting RMS."
        fi
    fi

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

# The answer is expected to differ per format - section 17 of the IMX178 analysis measured the stage
# acting on the 8 bit reduction and not on 10 or 12 bit - so all three are worth covering. Each is
# reconfigured through media_setting_rpi5.sh so the driver and the camera agree; the register is never
# written directly, which is what made the previous version report a median of 0 for RAW12 and RAW10
# while claiming to have tested them. FORMATS can narrow it, e.g. FORMATS=2 for 12 bit only.
if [ ! -x "$MEDIA" ]; then
    echo
    echo "    $MEDIA not found or not executable, so the format cannot be changed safely."
    echo "    Testing only $(fmt_name "$WAS_FMT"), the one the driver has. Point MEDIA_SCRIPT at it to"
    echo "    cover the others, and set CAM if this camera is not -c $CAM."
    FORMATS=${FORMATS:-$WAS_FMT}
fi

for FMT in ${FORMATS:-2 1 0}; do

    NAME=$(fmt_name "$FMT")

    echo
    echo "================= $NAME ================="

    if [ "$FMT" != "$WAS_FMT" ] || [ -x "$MEDIA" ]; then
        if ! set_format "$FMT"; then
            echo "    could not reconfigure the driver for $NAME, skipping"
            continue
        fi
        echo "    driver reconfigured: $NAME at ${RW}x${RH}+${RX}+${RY}"
    fi

    set_gamma "$G_LOW" || { echo "    could not set gamma $G_LOW"; continue; }
    sleep 0.5
    grab "$OUT/low_$FMT.raw" || { echo "    capture FAILED - is $VIDEO right, and is RMS stopped?"; continue; }
    echo "    gamma $G_LOW  median $(median "$OUT/low_$FMT.raw" "$FMT")"

    set_gamma "$G_HIGH" || { echo "    could not set gamma $G_HIGH"; continue; }
    sleep 0.5
    grab "$OUT/high_$FMT.raw" || { echo "    capture FAILED"; continue; }
    echo "    gamma $G_HIGH median $(median "$OUT/high_$FMT.raw" "$FMT")"

    echo
    if ! python3 "$OUT/analyse.py" "$OUT/low_$FMT.raw" "$OUT/high_$FMT.raw" \
                 "$W" "$H" "$FMT" "$NAME" "$G_LOW" "$G_HIGH"; then
        echo
        echo "    Nothing to compare usually means the frames are not in the format assumed here."
        echo "    Check that media_setting_rpi5.sh configured $NAME, and that nothing has written the"
        echo "    Pixel_Format register since: the driver sets up the CSI-2 receiver from its own"
        echo "    configuration, so the two disagreeing gives frames that unpack to nothing."
    fi
done

echo
echo "=== what this means ==="
cat <<'EOT'
  A format where the stage is LIVE composes its gamma with any curve the element or the LUT applies.
  There, cam-gamma=0 is required before a night where photometry matters - the net exponent is the
  product of the two, and RMS has one gamma field in the platepar, so it cannot undo a product.

  A format where it is INERT can leave the enable bit alone: it changes nothing.

  This is expected to split by format rather than answer yes or no. Section 17 of the IMX178 analysis
  measured the stage acting on the 8 bit reduction - exponent 0.820, and a pedestal of 21.8 levels
  where plain truncation would give 12.4 - and not on 10 or 12 bit, where the measurement is a clean
  truncation and the star photometry comes out linear at the curve the element applied. If this run
  agrees, that is two independent methods on the same conclusion.
EOT

echo "    captures left in $OUT"
