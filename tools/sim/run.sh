#!/bin/bash
# Build and run the TrueShift simulator natively (host g++).
set -euo pipefail
here=$(realpath "$(dirname "$0")"); suite=$(realpath "$here/../../plugin/dazdrive")
g++ -O1 -std=c++17 -w -I"$here/shim" -I"$suite" -I"$suite/../sdk/include" -o "$here/sim" \
    "$here/sim.cpp" "$suite/modules/trueshift.cpp" "$suite/core/vehicle.cpp" "$suite/core/display.cpp"
"$here/sim" "$@"
