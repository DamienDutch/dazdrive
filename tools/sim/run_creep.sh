#!/bin/bash
# Build and run the Creep module simulator natively (host g++).
set -euo pipefail
here=$(realpath "$(dirname "$0")"); suite=$(realpath "$here/../../plugin/dazdrive")
g++ -O1 -std=c++17 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -I"$here/shim" -I"$suite" -I"$suite/../sdk/include" -o "$here/sim_creep" "$here/sim_creep.cpp"
"$here/sim_creep" "$@"
