#!/bin/bash
# Atrapa media_setting_rpi5.sh: tylko przestawia pixelformat w stanie atrapy,
# czyli robi to, co na Pi robi sterownik - obie strony naraz.
S="$(dirname "$0")/state"
FMT=""
while [ $# -gt 0 ]; do
  case "$1" in -fmt) FMT="$2"; shift;; esac; shift
done
case "$FMT" in RAW8) V=0;; RAW10) V=1;; RAW12) V=2;; *) exit 1;; esac
awk -v v="$V" '{ if ($1=="pixelformat") print "pixelformat", v; else print }' "$S" > "$S.t" && mv "$S.t" "$S"
echo "media configured for $FMT"
