#!/bin/bash
# Build and run every native simulator (TrueShift, Cruise, Lane, Creep, LowSpeed, the wish pedal, the score-based gear choice, the retarder hook, the deceleration assist, the dashboard letter, the learned brake gain) and say which passed. Exit 1 if any failed.
# The incident recorder test (run_incident.sh) needs podman + wine and is not included.
here=$(realpath "$(dirname "$0")")
fail=0
for s in run.sh run_cruise.sh run_lane.sh run_creep.sh run_lowspeed.sh run_pedal.sh run_scores.sh run_retarder.sh run_decel.sh run_display.sh run_brake.sh; do
  if out=$("$here/$s" 2>&1); then echo "ok    $s"; else echo "FAIL  $s"; echo "$out" | tail -15; fail=1; fi
done
exit $fail
