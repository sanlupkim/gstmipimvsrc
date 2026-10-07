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
# One run covers all three, because the script reconfigures through media_setting_rpi5.sh, which on the
# Pi changes the driver and the camera together. The stub stands in for it. What the script must never
# do is write Pixel_Format itself: that moves one side only and the frames unpack to nothing, which is
# what an earlier version did while reporting it had tested them. The stub's fake v4l2-ctl honoured the
# register back then, so it shared the mistake and could not catch it; now only the media stub moves
# the format, and the harness asserts the script did not.
#
# Usage: tests/run_test_gamma_path.sh

set -u

HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d /tmp/gammastub.XXXXXX)
trap 'rm -rf "$WORK"' EXIT

cp "$HERE/fake_mv_mipi_i2c.sh" "$WORK/mv_mipi_i2c.sh"
mkdir -p "$WORK/bin"
cp "$HERE/bin/v4l2-ctl" "$WORK/bin/v4l2-ctl"
cp "$HERE/fake_media_setting.sh" "$WORK/media_setting.sh"
chmod +x "$WORK/mv_mipi_i2c.sh" "$WORK/bin/v4l2-ctl" "$WORK/media_setting.sh"

cat > "$WORK/state" <<'EOF'
gammaenable 0
gamma 0.80
lutenable 1
pixelformat 0
metime 39906
mgain 30.0
roi 344,0,2400,2064
EOF
cp "$WORK/state" "$WORK/state.before"

LC_ALL=C PATH="$WORK/bin:$PATH" MV_TOOLS="$WORK" MEDIA_SCRIPT="$WORK/media_setting.sh" \
    bash "$HERE/../test_gamma_path.sh" 4 /dev/video8 240 206 > "$WORK/out.txt" 2>&1
rc=$?

fail=0
echo "exit code $rc"
[ $rc -eq 0 ] || fail=1

for want in "RAW12: GAMMA IS INERT" "RAW10: GAMMA IS INERT" "RAW8: GAMMA IS LIVE"; do
    if grep -qF "$want" "$WORK/out.txt"; then
        echo "  ok    $want"
    else
        echo "  FAIL  expected: $want"
        fail=1
    fi
done

# Only the media stub may move the format. If the script wrote the register itself the stub's
# v4l2-ctl would not follow, so this also catches that.
if grep -q "GAMMA IS\|NO VERDICT" "$WORK/out.txt" \
   && ! grep -q "could not reconfigure" "$WORK/out.txt"; then
    echo "  ok    every format reconfigured through the driver"
else
    echo "  FAIL  a format could not be reconfigured"
    fail=1
fi

# The media script must be called with the camera's own ROI and only the format varying. A broken ROI
# read would show here rather than passing silently, which is what happened when the fake tool had no
# roi field at all and the fake media script ignored its arguments.
want_calls="mvcam -fmt RAW12 -x 344 -y 0 -w 2400 -h 2064 -c 1
mvcam -fmt RAW10 -x 344 -y 0 -w 2400 -h 2064 -c 1
mvcam -fmt RAW8 -x 344 -y 0 -w 2400 -h 2064 -c 1"

if [ "$(grep -v 'RAW8 -x 344' "$WORK/media_calls" | head -2; grep -m1 'RAW8 -x 344' "$WORK/media_calls")" \
     = "$want_calls" ]; then
    echo "  ok    media script called with the camera's ROI, only -fmt varying"
else
    echo "  FAIL  unexpected media script calls:"
    sed 's/^/        /' "$WORK/media_calls"
    fail=1
fi

if diff -q <(sort "$WORK/state.before") <(sort "$WORK/state") >/dev/null; then
    echo "  ok    camera restored exactly, pixel format included"
else
    echo "  FAIL  camera not restored:"
    diff <(sort "$WORK/state.before") <(sort "$WORK/state") | sed 's/^/        /'
    fail=1
fi

if grep -q "NOT restoring" "$WORK/out.txt"; then
    echo "  FAIL  a setting could not be parsed, so it was left alone"
    fail=1
fi

[ $fail -eq 0 ] || { echo; sed 's/^/    /' "$WORK/out.txt"; }

exit $fail
