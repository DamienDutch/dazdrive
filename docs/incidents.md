# Incident files

DazDrive does not keep a broad log. `dazdrive.log` holds the start-up lines, errors, mode changes and one line per incident. When
something goes wrong, the plugin saves that moment as its own file in `plugins\dazdrive\incidents\`.

## What is an incident

| Name | Trigger |
|---|---|
| `hunt` | TrueShift's gearbox reverses direction three times within 10 s (up-down-up or down-up-down), none of them yours; not below the takeover speed |
| `override` | you give a shift hint against a shift TrueShift made less than 5 s ago |
| `limiter` | the rpm sits at the limiter for more than 0.6 s while TrueShift drives (not in M, not on the engine brake) |
| `lug` | the rpm sits under 90 % of idle for more than 0.8 s while moving above 8 km/h with the throttle open |
| `neutral` | stuck in neutral for 1.5 s (also after an eco-roll) and TrueShift had to engage a gear; not right after the pause menu or a load |
| `air` | the brake air is so low (while moving) that DazDrive stops its own braking |
| `aeb` | automatic emergency braking acted |
| `input` | the ACC distance key reads as held for 6 s (its state does not return to 0): holding it for ISA is switched off for the session |
| `lane` | the lane assist sways or rings: the truck swings to and fro around the center and the gains are cut |

Triggers are deliberately strict: a normal ride (hills, eco-roll, stops, cruise) must not make one. A further trigger within 15 s
of the last one joins the open file, so one bad stretch is one file.

## What is in a file

`<name>_<local date>_<local time>.csv`: a header with the version, the truck and the trigger, then the log lines of the 20 s before
(also the detail lines that `log_level` 0 keeps out of `dazdrive.log`), then the rows. Rows are 10 per second, from 20 s before the
trigger until 15 s after the last one; `t` is in seconds from the trigger. Log lines that appear while the file is open are in it
too, as `#` lines. Columns: `kmh, rpm, gear, target` (the gear TrueShift wants), `thr, brake, accel, grade_pct, set_kmh` (cruise),
`gap_m` (radar), `mode` (1 Eco, 2 Normal, 3 Power, 4 M), `flags` (1 neutral roll, 2 crawl, 4 AEB, 8 speed-limit following,
16 hint hold). AEB files have their own, richer columns (20 per second).

## Size

`incident_mb` in `[core]` (standard 4, 0 = off): the folder is trimmed oldest first. Per game session at most 40 files and 12 per
name, so one hunting loop cannot push the rest out. Together with `log_max_mb` (0.6; log, `.1` and the previous session's
`dazdrive.prev.log`) the standard preset stays under about 6 MB.

## Test

`tools/sim/run_incident.sh` builds the recorder as a Windows exe and runs it under wine (30 s): quiet log, level 1 lines in the
file, joining, closing, the folder cap. `tools/sim/run.sh` prints every incident the gearbox simulator would raise (`INCIDENT ...`).
