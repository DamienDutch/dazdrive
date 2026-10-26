#!/bin/bash
# Build and run the deceleration assist simulator natively (host g++). Exit 1 if an assert failed.
# modules/decel.cpp, core/rsc.cpp and core/actuate.cpp are included in the sim (the real brake writer sits between the assist and the plant).
set -euo pipefail
here=$(realpath "$(dirname "$0")"); suite=$(realpath "$here/../../plugin/dazdrive")
g++ -O1 -std=c++17 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -I"$here/shim" -I"$suite" -I"$suite/../sdk/include" -o "$here/sim_decel" "$here/sim_decel.cpp"
"$here/sim_decel" "$@"
