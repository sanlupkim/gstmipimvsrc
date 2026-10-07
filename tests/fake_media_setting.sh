#!/bin/bash
# Atrapa media_setting_rpi5.sh: tylko przestawia pixelformat w stanie atrapy,
# czyli robi to, co na Pi robi sterownik - obie strony naraz.
S="$(dirname "$0")/state"
FMT=""; X=""; Y=""; WI=""; HE=""; C=""
while [ $# -gt 0 ]; do
  case "$1" in
    -fmt) FMT="$2"; shift;; -x) X="$2"; shift;; -y) Y="$2"; shift;;
    -w) WI="$2"; shift;; -h) HE="$2"; shift;; -c) C="$2"; shift;;
  esac; shift
done
# Zapisz cale wywolanie, zeby harness mogl sprawdzic, co dostal
echo "mvcam -fmt $FMT -x $X -y $Y -w $WI -h $HE -c $C" >> "$(dirname "$0")/media_calls"

# Odrzuc niepelne albo bezsensowne ROI - na Pi media-ctl tez by sie wywrocilo, a bez tego
# zepsuty odczyt ROI przechodzilby niezauwazony
for v in "$X" "$Y" "$WI" "$HE" "$C"; do
  case "$v" in ''|*[!0-9]*) echo "fake media: bad or missing argument ($v)" >&2; exit 1;; esac
done
[ "$WI" -ge 64 ] && [ "$HE" -ge 64 ] || { echo "fake media: implausible size ${WI}x${HE}" >&2; exit 1; }

case "$FMT" in RAW8) V=0;; RAW10) V=1;; RAW12) V=2;; *) exit 1;; esac
awk -v v="$V" '{ if ($1=="pixelformat") print "pixelformat", v; else print }' "$S" > "$S.t" && mv "$S.t" "$S"
echo "media configured for $FMT"
