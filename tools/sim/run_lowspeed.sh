#!/bin/bash
# Build and run the low-speed owner simulator natively (host g++). Exit 1 if an assert failed.
# TrueShift and the vehicle model are linked for real (the start gear is TrueShift's physics pick); lowspeed.cpp is included in the sim.
set -euo pipefail
here=$(realpath "$(dirname "$0")"); suite=$(realpath "$here/../../plugin/dazdrive")
obj=$(mktemp -d); trap 'rm -rf "$obj"' EXIT
inc=(-I"$here/shim" -I"$suite" -I"$suite/../sdk/include")
g++ -O1 -std=c++17 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers "${inc[@]}" -c -o "$obj/sim_lowspeed.o" "$here/sim_lowspeed.cpp"
g++ -O1 -std=c++17 -w "${inc[@]}" -c -o "$obj/ts.o" "$suite/modules/trueshift.cpp"
g++ -O1 -std=c++17 -w "${inc[@]}" -c -o "$obj/veh.o" "$suite/core/vehicle.cpp"
g++ -o "$here/sim_lowspeed" "$obj/sim_lowspeed.o" "$obj/ts.o" "$obj/veh.o"
"$here/sim_lowspeed" "$@"
