#!/bin/bash
# Runs test_gamma_path.sh against a stubbed camera, here, with no hardware.
#
# Written after that script went to the station four times with a fault each time: an exposure it never
# set, a verdict decided by sensor noise, a value it never checked took, a restore that invented a
# setting it had failed to parse, and a function call one argument short that set -u turned into a
# ladder of question marks. Every one of those would have shown up in ten seconds here.
#
# The stub applies gamma only when reducing to 8 bits, which is what section 17 of the IMX178 analysis
# measured on real sunset frames. So the expected result is live in RAW8 and inert in RAW10 and RAW12.
#
# Each format is a separate run with the stub set to it, because that is how it works on the camera: the
# driver owns the pixel format and the test may not change it. One run tests the one format the camera
# is in. The previous version of both files ran a loop writing the register, which changed what the
# camera sent without changing what the CSI receiver expected, and got frames that unpacked to nothing.
#
# Usage: tests/run_test_gamma_path.sh

set -u

HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d /tmp/gammastub.XXXXXX)
trap 'rm -rf "$WORK"' EXIT

cp "$HERE/fake_mv_mipi_i2c.sh" "$WORK/mv_mipi_i2c.sh"
mkdir -p "$WORK/bin"
cp "$HERE/bin/v4l2-ctl" "$WORK/bin/v4l2-ctl"
chmod +x "$WORK/mv_mipi_i2c.sh" "$WORK/bin/v4l2-ctl"

fail=0

for case in "0 RAW8 LIVE" "1 RAW10 INERT" "2 RAW12 INERT"; do

    set -- $case
    fmt=$1 name=$2 want=$3

    cat > "$WORK/state" <<EOF
gammaenable 0
gamma 0.80
lutenable 1
pixelformat $fmt
metime 39906
mgain 30.0
EOF
    cp "$WORK/state" "$WORK/state.before"

    LC_ALL=C PATH="$WORK/bin:$PATH" MV_TOOLS="$WORK" \
        bash "$HERE/../test_gamma_path.sh" 4 /dev/video8 240 206 > "$WORK/out.txt" 2>&1
    rc=$?

    echo "--- camera configured for $name (exit $rc) ---"
    [ $rc -eq 0 ] || { echo "  FAIL  non zero exit"; fail=1; }

    if grep -qF "$name: GAMMA IS $want" "$WORK/out.txt"; then
        echo "  ok    $name: GAMMA IS $want"
    else
        echo "  FAIL  expected $name: GAMMA IS $want"
        grep -E "GAMMA IS|NO VERDICT" "$WORK/out.txt" | sed 's/^/        got: /'
        fail=1
    fi

    # The format may not be touched: the driver owns it
    if [ "$(awk '$1=="pixelformat"{print $2}' "$WORK/state")" = "$fmt" ]; then
        echo "  ok    pixel format left alone"
    else
        echo "  FAIL  the script changed the pixel format"
        fail=1
    fi

    # Nothing may be left changed, and nothing may be invented
    if diff -q <(sort "$WORK/state.before") <(sort "$WORK/state") >/dev/null; then
        echo "  ok    camera restored exactly"
    else
        echo "  FAIL  camera not restored:"
        diff <(sort "$WORK/state.before") <(sort "$WORK/state") | sed 's/^/        /'
        fail=1
    fi

    if grep -q "NOT restoring" "$WORK/out.txt"; then
        echo "  FAIL  a setting could not be parsed, so it was left alone"
        fail=1
    fi

    [ $fail -eq 0 ] || sed 's/^/        /' "$WORK/out.txt"
done

exit $fail
