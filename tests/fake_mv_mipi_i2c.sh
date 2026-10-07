#!/bin/bash
# Atrapa narzedzia producenta: te same formaty wyjscia, ze spacja przed koncem linii.
S="$(dirname "$0")/state"
MODE=""; FIELD=""; P1=""
while [ $# -gt 0 ]; do
  case "$1" in
    -r) MODE=r;; -w) MODE=w;; -f) FIELD="$2"; shift;;
    -p1) P1="$2"; shift;; -b) shift;;
  esac; shift
done
get() { awk -v k="$1" '$1==k{print $2}' "$S"; }
put() { awk -v k="$1" -v v="$2" '{ if ($1==k) print k, v; else print }' "$S" > "$S.t" && mv "$S.t" "$S"; }
case "$FIELD" in
  gammaenable)  [ "$MODE" = w ] && put gammaenable "$P1";  printf "r gamma enable is %d \n" "$(get gammaenable)";;
  lutenable)    [ "$MODE" = w ] && put lutenable "$P1";    printf "r lut enable is %d \n" "$(get lutenable)";;
  gamma)        [ "$MODE" = w ] && put gamma "$P1";        printf "r gamma is %.2f \n" "$(get gamma)";;
  pixelformat)  [ "$MODE" = w ] && put pixelformat "$P1";  printf "r pixelformat is %d \n" "$(get pixelformat)";;
  metime)       [ "$MODE" = w ] && put metime "$P1";       printf "r manual exp time %d us \n" "$(get metime)";;
  mgain)        [ "$MODE" = w ] && put mgain "$P1";        printf "r manual gain %.1f dB \n" "$(get mgain)";;
  roi)          printf "r roi is %s \n" "$(get roi)";;
  *) echo "unknown field $FIELD" >&2; exit 1;;
esac
