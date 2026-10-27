#!/bin/bash
# Build and run the learned brake-gain simulator natively (host g++). Exit 1 if an assert failed.
# Only the vehicle model is linked for real (core/vehicle.cpp: the estimator lives there); the game is stubbed in the sim.
set -euo pipefail
here=$(realpath "$(dirname "$0")"); suite=$(realpath "$here/../../plugin/dazdrive")
obj=$(mktemp -d); trap 'rm -rf "$obj"' EXIT
inc=(-I"$here/shim" -I"$suite" -I"$suite/../sdk/include")
g++ -O1 -std=c++17 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers "${inc[@]}" -c -o "$obj/sim_brake.o" "$here/sim_brake.cpp"
g++ -O1 -std=c++17 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers "${inc[@]}" -c -o "$obj/veh.o" "$suite/core/vehicle.cpp"
g++ -o "$here/sim_brake" "$obj/sim_brake.o" "$obj/veh.o"
"$here/sim_brake" "$@"
