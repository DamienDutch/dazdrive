#!/bin/bash
# Build and run the Cruise module simulator natively (host g++).
set -euo pipefail
here=$(realpath "$(dirname "$0")"); suite=$(realpath "$here/../../plugin/dazdrive")
g++ -O1 -std=c++17 -w -I"$here/shim" -I"$suite" -I"$suite/../sdk/include" -o "$here/sim_cruise" "$here/sim_cruise.cpp"
"$here/sim_cruise" "$@"
