#!/bin/bash
# Build and run the Lane module simulator natively (host g++). Args: [wheel spring] [tau_w]
set -euo pipefail
here=$(realpath "$(dirname "$0")"); suite=$(realpath "$here/../../plugin/dazdrive")
g++ -O1 -std=c++17 -w -I"$here/shim" -I"$suite" -I"$suite/../sdk/include" -o "$here/sim_lane" "$here/sim_lane.cpp"
"$here/sim_lane" "$@"
