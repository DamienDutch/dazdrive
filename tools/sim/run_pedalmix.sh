#!/bin/bash
# Build and run the torque_share simulator natively (host g++): the wish pedal (modules/pedal.cpp, the real one) on the TrueShift simulator's plant.
# A table of torque_share 0 / 0.5 / 0.75 / 1 over two trucks, flat and 2 %, three steady pedals, and a wavering pedal. Exit 1 if a check failed.
set -euo pipefail
here=$(realpath "$(dirname "$0")"); suite=$(realpath "$here/../../plugin/dazdrive")
obj=$(mktemp -d); trap 'rm -rf "$obj"' EXIT
inc=(-I"$here/shim" -I"$suite" -I"$suite/../sdk/include")
g++ -O1 -std=c++17 -w "${inc[@]}" -c -o "$obj/sim_pedalmix.o" "$here/sim_pedalmix.cpp"
g++ -O1 -std=c++17 -w "${inc[@]}" -c -o "$obj/ts.o" "$suite/modules/trueshift.cpp"
g++ -O1 -std=c++17 -w "${inc[@]}" -c -o "$obj/veh.o" "$suite/core/vehicle.cpp"
g++ -O1 -std=c++17 -w "${inc[@]}" -c -o "$obj/disp.o" "$suite/core/display.cpp"
g++ -o "$here/sim_pedalmix" "$obj/sim_pedalmix.o" "$obj/ts.o" "$obj/veh.o" "$obj/disp.o"
"$here/sim_pedalmix" "$@"
