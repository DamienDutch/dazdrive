#!/bin/bash
# Build and run the deceleration assist simulator natively (host g++). Exit 1 if an assert failed.
# modules/decel.cpp, core/rsc.cpp and core/actuate.cpp are included in the sim (the real brake writer sits between the assist and the plant);
# TrueShift and the vehicle model are linked for real (the gearbox the assist shifts down with, and its prediction of a shift).
set -euo pipefail
here=$(realpath "$(dirname "$0")"); suite=$(realpath "$here/../../plugin/dazdrive")
obj=$(mktemp -d); trap 'rm -rf "$obj"' EXIT
inc=(-I"$here/shim" -I"$suite" -I"$suite/../sdk/include")
g++ -O1 -std=c++17 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers "${inc[@]}" -c -o "$obj/sim_decel.o" "$here/sim_decel.cpp"
g++ -O1 -std=c++17 -w "${inc[@]}" -c -o "$obj/ts.o" "$suite/modules/trueshift.cpp"
g++ -O1 -std=c++17 -w "${inc[@]}" -c -o "$obj/veh.o" "$suite/core/vehicle.cpp"
g++ -O1 -std=c++17 -w "${inc[@]}" -c -o "$obj/disp.o" "$suite/core/display.cpp"
g++ -o "$here/sim_decel" "$obj/sim_decel.o" "$obj/ts.o" "$obj/veh.o" "$obj/disp.o"
"$here/sim_decel" "$@"
