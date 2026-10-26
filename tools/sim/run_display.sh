#!/bin/bash
# Build and run the dashboard letter simulator natively (host g++): the priority of the wishes, the letters outside the table.
set -euo pipefail
here=$(realpath "$(dirname "$0")"); suite=$(realpath "$here/../../plugin/dazdrive")
g++ -O1 -std=c++17 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -I"$here/shim" -I"$suite" -I"$suite/../sdk/include" -o "$here/sim_display" "$here/sim_display.cpp"
"$here/sim_display" "$@"
