# For an AI assistant helping someone with DazDrive

Short map for a person who hands this repo (or a log) to an assistant. The player README in the zip (`README-player.txt`) has
the install steps, controls and known limits; the main [README](../README.md) has the feature list.

## What it is

One plugin DLL for Euro Truck Simulator 2 (Steam beta `temporary_1_60`, ETS2 only). It patches the game's code in memory.
Modules: **TrueShift** (gearbox), **Cruise** (SteadyCruise, ACC, AEB, PCC, Stop & Go, Auto Hold), **Lane** (lane centering),
**Creep** (crawl mode). Singleplayer only; never TruckersMP (their rule 2.1). SCS Convoy is not tested.

## Where things are (in the game folder)

- `bin\win_x64\plugins\dazdrive.dll`: the plugin.
- `bin\win_x64\plugins\dazdrive\dazdrive.ini`: all settings, one section per module (`[core]`, `[trueshift]`, `[cruise]`,
  `[lane]`, `[creep]`). Each key has a help line above it. An unknown key is reported in `dazdrive.log`; out-of-range values are clamped.
- `dazdrive.log` (and `dazdrive.prev.log`): start-up lines, errors, mode changes. `log_level` in `[core]` sets how much.
- `incidents\`: one csv per incident.
- `plugins\dazdrive\dazdrive-setup.html` (in the zip): the setup page. Opens, edits and saves the ini in a browser. It offers six "truck years"
  (~1995, ~2000, ~2010, ~2015, ~2020, 2024) that switch on the features a real European truck had by then, and marks every
  feature stable, beta or experimental. The shipped ini equals ~2010.

## Switching things off

`enabled=0` in `[trueshift]`, `[cruise]` or `[creep]` turns that module off; `mode=0` in `[lane]` turns lane centering off. Most
features also have their own key (`eco_roll`, `lookahead`, `predictive`, `curves`, `junctions`, `own_acc`, `aeb`). When a problem is unclear, turn off
the newest or experimental feature first and see if it goes away.

## Incidents (what the files mean)

DazDrive saves a moment as `incidents\<name>_<date>_<time>.csv` when something goes wrong: 20 s before and 15 s after the trigger,
10 rows per second, with the log lines in it. Triggers are strict, so a normal ride makes none.

- `hunt`: the gearbox changed direction three times in 10 s. `override`: you gave a shift hint against a shift it just made.
- `limiter`: rpm at the limiter over 0.6 s. `lug`: rpm under 90 % of idle over 0.8 s while moving.
- `neutral`: stuck in neutral for 1.5 s. `air`: brake air so low that DazDrive stopped its own braking.
- `aeb`: emergency braking acted. `input`: the ACC distance key read as held for 6 s. `lane`: the lane assist swayed or rang. `rsc`: the rollover guard saw a growing sideways swing (tone, throttle off, brake).

Columns include `kmh, rpm, gear, target, thr, brake, accel, grade_pct, set_kmh, gap_m, mode, flags`. The budget is `incident_mb`
(standard 4) and `log_max_mb`.

## What to ask the person for

The folder `plugins\dazdrive\incidents\` (zipped), `dazdrive.log`, the DazDrive version (first log lines), the truck and trailer
weight, and what they were doing. Problems go to the issue tracker of the repo.

## About the source

`plugin/dazdrive/core/` is the shared layer (hooks, telemetry, route reading, config, log). `modules/*.cpp` are the four modules,
each with a parameter table at the top that is the single source for the ini keys and their help text. `tools/gen_ini.py` builds
the ini presets and the setup page from those tables. `tools/sim/` holds simulators that run the real module code offline.
