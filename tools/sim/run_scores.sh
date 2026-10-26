#!/bin/bash
# The score-based gear choice ([trueshift] choice = 1) in the TrueShift simulator: the score function on the vehicle model, and
# choice 0 against choice 1 in the simulator (a table). Exit 1 if an assert failed. Same build as run.sh (sim.cpp, SCORES=1).
here=$(realpath "$(dirname "$0")")
SCORES=1 exec "$here/run.sh" "$@"
