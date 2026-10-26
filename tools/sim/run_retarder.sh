#!/bin/bash
# Build and run the retarder hook simulator natively (host g++): the factor merge, the hook pattern, the stub's bytes executed.
set -euo pipefail
here=$(realpath "$(dirname "$0")"); suite=$(realpath "$here/../../plugin/dazdrive")
g++ -O1 -std=c++17 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -I"$here/shim" -I"$suite" -I"$suite/../sdk/include" -o "$here/sim_retarder" "$here/sim_retarder.cpp"
"$here/sim_retarder" "$@"
