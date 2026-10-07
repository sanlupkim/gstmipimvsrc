#!/bin/bash
# Runs test_gamma_path.sh against a stubbed camera, here, with no hardware.
#
# Written after that script went to the station four times with a fault each time: an exposure it never
# set, a verdict decided by sensor noise, a value it never checked took, a restore that invented a
# setting it had failed to parse, and a function call one argument short that set -u turned into a
# ladder of question marks. Every one of those would have shown up in ten seconds here.
#
# The stub applies gamma only when reducing to 8 bits, which is what section 17 of the IMX178 analysis
# measured on real sunset frames. So the expected result is RAW12 inert, RAW10 inert, RAW8 live, and
# the camera left exactly as it was found.
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

cat > "$WORK/state" <<'EOF'
gammaenable 0
gamma 0.80
lutenable 1
pixelformat 0
metime 39906
mgain 30.0
EOF
cp "$WORK/state" "$WORK/state.before"

LC_ALL=C PATH="$WORK/bin:$PATH" MV_TOOLS="$WORK" \
    bash "$HERE/../test_gamma_path.sh" 4 /dev/video8 240 206 > "$WORK/out.txt" 2>&1
rc=$?

fail=0
check() {
    if grep -qF "$1" "$WORK/out.txt"; then
        echo "  ok    $1"
    else
        echo "  FAIL  expected: $1"
        fail=1
    fi
}

echo "exit code $rc"
[ $rc -eq 0 ] || fail=1

check "RAW12: GAMMA IS INERT"
check "RAW10: GAMMA IS INERT"
check "RAW8: GAMMA IS LIVE"

# Nothing may be left changed, and nothing may be invented: the restore has to be exact
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
else
    echo "  ok    every setting was read back and restored"
fi

if [ $fail -ne 0 ]; then
    echo
    echo "full output:"
    sed 's/^/    /' "$WORK/out.txt"
fi

exit $fail
