#!/bin/bash
# Build and run the TrueShift simulator natively (host g++). With `--build-only` it only compiles (sweeps, the sim worker).
set -euo pipefail
here=$(realpath "$(dirname "$0")"); suite=$(realpath "$here/../../plugin/dazdrive")
# (-O3 + LTO: the same results bit for bit, ~10-20 % faster; -ffp-contract=off keeps the float math as written. SIM_CFLAGS overrides)
SIM_CFLAGS=${SIM_CFLAGS:--O3 -march=native -flto=auto -ffp-contract=off}
g++ $SIM_CFLAGS -std=c++17 -w -I"$here/shim" -I"$suite" -I"$suite/../sdk/include" -o "$here/sim" \
    "$here/sim.cpp" "$suite/modules/trueshift.cpp" "$suite/modules/carshift.cpp" "$suite/core/shifthook.cpp" "$suite/core/gearreq.cpp" "$suite/core/shiftmodel.cpp" "$suite/core/shiftcommon.cpp" "$suite/core/vehicle.cpp" "$suite/core/bands.cpp" "$suite/core/ahead.cpp" "$suite/core/wish.cpp" "$suite/core/demand.cpp" "$suite/core/driver.cpp" "$suite/core/gearchoice.cpp" "$here/../../plugin/dazdrive/core/intent.cpp" "$suite/core/display.cpp"
[ "${1:-}" = "--build-only" ] && exit 0
"$here/sim" "$@"
