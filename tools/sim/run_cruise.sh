#!/bin/bash
# Build and run the Cruise module simulator natively (host g++). With `--build-only` it only compiles (used by sweep_acc.py).
set -euo pipefail
here=$(realpath "$(dirname "$0")"); suite=$(realpath "$here/../../plugin/dazdrive")
# (-O3 + LTO: the same results bit for bit, ~10-20 % faster; -ffp-contract=off keeps the float math as written. SIM_CFLAGS overrides)
SIM_CFLAGS=${SIM_CFLAGS:--O3 -march=native -flto=auto -ffp-contract=off}
g++ $SIM_CFLAGS -std=c++17 -w -I"$here/shim" -I"$suite" -I"$suite/../sdk/include" -o "$here/sim_cruise" "$here/sim_cruise.cpp" "$suite/core/gearreq.cpp"
[ "${1:-}" = "--build-only" ] && exit 0
"$here/sim_cruise" "$@"
