#!/bin/bash
# Build and run the traffic probe's geometry test natively (host g++). Exit 1 if a check failed.
set -euo pipefail
here=$(realpath "$(dirname "$0")"); suite=$(realpath "$here/../../plugin/dazdrive")
g++ -O1 -std=c++17 -Wall -Wextra -I"$suite/.." -I"$suite" -o "$here/sim_traffic" "$here/sim_traffic.cpp"
"$here/sim_traffic" "$@"
