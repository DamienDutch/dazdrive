# TrueShift: the gearbox module

The suite DLL (`dazdrive`, `plugin/dazdrive/`) with its first module TrueShift. Own code, MIT; hook layer
derived from car_automatic_transmission (MIT, Deppelover). ETS2 1.60 only (layout table in `core/game.cpp`; ATS 1.61
moved the target gear and throttle, not mapped). Unknown build → nothing patched, log says why.

## Files

- `core/suite.h` shared API (SUITE_NAME, log, ini, telemetry struct, module struct). Rename the suite = change
  SUITE_NAME (DLL, ini and folder names follow).
- `core/main.cpp` SDK entry, module registry (`g_modules[]`).
- `core/game.{h,cpp}` layout table, discovery, call-site trampolines, object access (gear, target, force flag,
  throttle, engine data, live mass, grade, adaptive cvar).
- `core/telemetry.cpp` SDK channels + truck config (ratios, diff, powered wheel radius), filtered accel.
- `core/config.cpp` `dazdrive.ini` (sections = modules; missing file → written with defaults + help).
- `modules/trueshift.cpp` the shift logic.
- Build `plugin/dazdrive/build.sh [install]`. Offline discovery check `tools/check_suite.py <exe>`.
  Logic simulator `tools/sim/run.sh` (host g++, truck model, 27 scenarios). Drive analysis
  `tools/analyze_trueshift.py <plugins/dazdrive dir>`.
- Game output: `bin/win_x64/plugins/dazdrive/` = `dazdrive.ini`, `dazdrive.log` (events), `trueshift.csv` (10 Hz).

## How it hooks (ETS2 1.60.1.7 addresses, all found by pattern at runtime)

Automatic update `0x14098c640` (per frame): if shift state `self+0x18 == 0`, gear != 0, target `self+0x68` != 0 it
reads the current gear's window (call `0x14098cb3a` = **decide site**), then calls the chooser `0x14098d0d0` when
rpm is outside the window (with throttle/accel conditions), or at once when `self+0x119` (kick/force flag) is set.
The chooser returns the gear; the update stores it in `self+0x68`; the generic gearbox `0x140601a10` shifts.

- decide site → `stub_decide`: original window, then our decision; want != gear → set `self+0x119 = 1`.
- chooser call sites `0x14098cc82`, `0x14098ccc6` → `stub_choose`: original (clears 0x119), return our gear
  (below `low_speed_kmh`, reverse, no data: the game's gear).
- start-gear fn `0x14098dbf0`, callers `0x14098c73c`, `0x14098c7c7`, `0x14098df6d` → `stub_start`: game pick,
  lowered while the launch slip time > `launch_slip_s`.
- No gate patch, no window changes, no bound clamps (car_automatic_transmission did those).

Stock chooser, for reference: candidates [lo, hi] from the bound fns, picks the gear whose predicted rpm is nearest
to down + (up-down)·k (k: up 0.3, down 0.9, kick 0.7), speed after the shift predicted with shift time
(`trans+0x1c0`); on upshift requires accel ≥ 9.81·sin(grade)+0.049 using ratio·diff·torque·`drive+0x334`/mass.
Upshift trigger in the update only if net accel at current throttle > grade accel.

## Game data used (1.60)

- Live mass: `float mass(game_actor, bool trailers=1)` = `0x1405c25d0`, game_actor = `[[0x143550398]+0x31a8]`
  (same call the stock update makes). Solves "where is the live weight". Inside it: physics vehicle
  `game_actor+0x18` → `+0x1f8` body vfunc `+0xb0`, trailers via `+0x1518`, next `+0x1010`.
- Grade: `float grade_deg(inner)` = `0x14062bea0` (first call in the lower-bound fn), positive uphill.
- Engine (`resolve(inner+0xff0)`): torque `+0x1c0`, curve ptr `+0x1d0` (float2 pairs, starts with (0,0)),
  count `+0x1d8`, rpm limit `+0x22c`, rpm ranges `+0x23c` (brake, low, high, power; (down, up)).
- Transmission (`resolve(inner+0xfe8)`): shift time `+0x1c0`, fwd ratios dyn array `+0x1f0`.
- inner: gear `+0x1448`, throttle in/eff `+0x1424/+0x1428`, clutch `+0x142c` (see `docs/clutch-mode.md`).
- `g_adaptive_shift` cvar `0x142bf4b70`, read via `0x1401d6ad0`; logged on change (letter test).

## Decision (per update, forward gear, ≥ low_speed_kmh)

rpm is speed-based (`v/r·ratio·diff`) × a learned factor vs real rpm (clutch closed). Demand = max(pedal, cruise
throttle), up fast (0.3 s) down slow (2 s). Shift points per gear = the engine's own ranges interpolated low→high
gear (= stock), moved toward the power range as demand goes `power_from`→`power_full`.
Order: rev-limit upshift (0.96·limit, 0.99 with engine brake) → stall downshift (< idle+120) → kickdown (pedal past
0.92 at ≥ 1.5/s, armed 3 s, lands ~power mid+100) → hint hold → engine brake (keep rpm in brake range, no upshift)
→ service brake (no upshift; downshift late, land high ≈ eco_up(1)−200 → fewer shifts) → min gap → downshift
(below down point or lugging: pedal > 0.8, decelerating, no force left) with anti-hunt after an upshift → upshift
(above up point; new gear above its down point + hysteresis after the speed lost in the shift, and still
≥ 0.02-0.05 m/s² pull; skip up to `max_skip` only when pedal < 0.6, grade < 2 %, ≥ `skip_accel` left).
Hint = a gear change nobody here commanded at ≥ low speed → hold `hint_hold_s` (protections still apply).

## Simulator results, base logic (tools/sim)

40 t full throttle 10→93 km/h: 10 shifts, up at 1645 rpm; 10 t at 0.4 pedal: up at 1260-1330; 40 t cruise 85
on 1 %: one downshift 12→11, no hunting; 6 % climb full: 11→10→9→8 (lugging), one up/down pair at a step grade
change; braking 80→7 km/h: 6 shifts (8→6→4→2 skips); hint down held 10 s. Fixed via the sim: kickdown latched too
long, min gap 1 s too slow in low gears, upshift pull requirement too strict at 85 km/h, load raising shift points.

## Modes
- The game's own keybind for the adaptive transmission cycles the cvar `g_adaptive_shift`: Eco (10) -> Normal (3) -> Power (1.6667) -> off (0) -> Eco, one step per press, each step visible in the log
  within 1 s. DazDrive uses that keybind as the mode source and turns the game's "off" slot into the fourth mode **M** (assisted manual), so the cycle is Eco -> Normal -> Power -> M.
- With cruise control on the game sets `game_actor+0x199` = 1 (it ignores the adaptive mode while cruising); the cvar itself stays as it was. TrueShift ignores that flag, so the mode survives cruise control.
- **Eco / Power** move the shift points and the cruise floor by `eco_rpm` -75 / `power_rpm` +150, with a force reserve for an early upshift of 1.2 / 1.5 (Normal 1.3); the demand-driven downshift starts at
  pedal 0.9 / 0.75 (Normal 0.85).
- **M (assisted manual)**: the gear stays where the driver's shift hints put it, but kickdown still works, the rev limiter gets one gear up, stall protection, downshifts when braking to a stop (below the low-speed
  handover the game picks as usual) and one gear down when lugging (below `m_lug_rpm` 1000 rpm with a lot of pedal and no pull; `m_assist` 0 turns kickdown and lugging help off). `m_full_manual` 1 = rev
  limiter and stall protection only.
- **Dashboard letter**: the game does not show the mode by itself (the dashboard stays on "A" in every mode, it is not driven by the cvar), so TrueShift writes it. The transmission's `auto_mode_name`
  string_dyn at `tx+0x258` (tx = resolve(inner+0xfe8); fallback `[inner+0x150]+0x358`) is checked (size 1, manual name "M" at +0x278) and its first character overwritten 4x/s from the SDK frame via
  `game_actor+0x1d68` (only when that equals the gearbox the hooks saw). The letter is D/E/P/M, also stopped and in N (a steady letter rather than "A" at every stop), and a plain A in reverse. The HUD reads it
  live via vfunc `0x14098db70` (`docs/engine-brake-re.md` section 3), so it shows at once.

## Design rules
- Normal is adaptive enough to never need switching; no mode may hurt a player who forgets that it is on. Eco and Power nudge the shift points down and up, never to an unusable value, and the differences
  between the modes stay small. Eco/Power = a firmer setting for heavy transport or convoys without hurry.
- **No hunting loop.** Never shift up into an rpm that is "allowed" but has no pull: more pedal gives nothing, then even more, then a downshift, speed picks up, and the box upshifts into no pull again. After an
  upshift there must be a real force reserve, and failures are remembered (back off per situation for a while). Eco must never keep checking whether it can now. Without navigation the logic is careful (only when
  sure: light, downhill); with route elevation it may be less careful. Failure detection is preventive, the response is to raise the rpm floor and let it fade (self-learning rather than fixed numbers). Nothing
  is learned across drives and nothing is written to disk for it.
- **Skips**: a slow box (long shift time) should shift less and skip more; a fast dual-clutch box may shift every gear or skip.
- **Cruise control** is bang-bang in the game (full throttle below the target, zero above): its throttle is smoothed before it counts as demand; the pedal stays direct.
- **Engine brake / retarder**: the harder the level, the higher the rpm (light = a bit higher, full = high); if the game's own "engine brake downshift" toggle is on, do a full downshift regardless of level.
- **Low-speed handover with hysteresis** (take over above 10 km/h, hand back below 6); no skip right above the boundary.
- **Reverse** behaves like the stock one: no separate smart reverse mode, just the same general behavior as D.
- Idea, not built: a passenger-car profile (ATS pickups or car mods) with higher rpm and a torque-converter feel, picked by mass and rpm limit.

## Current logic: additions to the decision above
- **Low rpm (early upshift)**: at light demand (< `power_from`) the next gear is taken when it lands at or above the start of the torque-curve plateau (+ mode offset, + learned) and its full force is at least
  reserve x the force asked (smoothed pedal force, at least the resistance). Without route data: no early upshift while a climb starts (grade +0.5 % in 3 s) and +0.1 reserve unless downhill. Learned extra
  resistance `f_bias` (force - model - m·a while pulling, 20 s, +-2 % grade). An early upshift also needs throttle >= 0.15 and >= 0.6 x demand, accel > -0.15, 3 s after takeover and >= takeover + 5 km/h.
- **Upshift hunting guard (learning, no timers)**: after each upshift (not rev limit) the throttle the new gear should need is predicted; throttle above max(pred + 0.15, 0.8) while not gaining speed
  integrates into `strain`; > 1 means "no pull" and a downshift. A no-pull, down, lugging or hint-down within 25 s is a lesson for that grade class (< -1, -1..1, 1..2.5, 2.5..4, >= 4 %; also counts for steeper):
  landing floor + `learn_rpm` 100, reserve + `learn_reserve` 0.1 (max 4x). It fades by distance (half after `learn_km` 3 km), -25 % per upshift that holds 30 s, and is cleared on a 25 % load change. A plain
  downshift only counts as a failure when throttle >= 0.5, and never when the grade rose >= 1 % since the upshift (terrain).
- **Upshift on a climb with a dip**: the rpm upshift reckons with the steepest of the grade now, a fading peak of the last seconds (0.15 %/s, only while > 1.5 %) and the route ahead, also for the speed lost
  in the shift. On a climb > 2 % the margin is 0.05-0.12 m/s2 (from 2 to 5 %); a comeback within 5 s always counts as a failed upshift. (A 9 -> 8 -> 9 -> 8 sequence in a 4 % dip of a 5-6 % climb was the case.)
  Sim H1/H1b (climbs 5.8-8 % with a 60 m dip): failed upshifts 4 + 1 hunting pair -> 0 (one left: an early upshift before a climb without a route), +50-70 rpm on those climbs, all other scenarios unchanged.
- **Skips** are allowed whenever best (`max_skip` 2); the shift time only weighs a little: the pull a skipped-to gear must keep is `skip_accel` x 1.25 (fast box) ... x 0.85 (>= 1.5 s). No skips at full
  throttle was a bug: the demand-raised downshift point blocked any landing below ~1200 rpm. Now the demand downshift (and that landing limit) only applies when the gear lacks pull (< `a_power` 0.3 m/s2,
  Eco 0.2, Power 0.45), and full-demand skips need 2.5 x `skip_accel`. Sim: 40 t full throttle skips 5 -> 7 (dual clutch) or 5 -> 7 -> 9 (1.2 s box), solo 7 -> 9 -> 11; high gears stay sequential.
- **Pedal and cruise control**: `inner+0x1424` is the throttle *after* the cruise control, not the pedal; the pedal is the SDK `input_throttle`. Otherwise every cruise burst would look like a kickdown and the
  damping would be bypassed. The cruise throttle is smoothed over `cruise_damp_s` 4 s before it counts; the damping falls in ~1 s once the cruise control is off the throttle for > 1.5 s (ACC slowing).
- **Engine brake / retarder**: strength = max(retarder level/steps, `inner+0x143c` demand, `game_actor+0x378` fraction) (also the automatic engine brake the SDK misses). Keeps rpm >= lerp(`ebrake_light_rpm`
  1100, brake-range low, strength); the full brake range when the game itself would (`inner+0x1440` set and engine def `engine_brake_downshift` = `engine+0x260`). `ebrake_downshift` -1 follow / 0 level /
  1 always full.
- **Handover** with hysteresis: take over >= `takeover_kmh` 10, hand back < `handback_kmh` 6; no skips for 3 s after takeover or below takeover + 5 km/h.
- Trace columns (`trueshift.csv`): `mode,thr,f_need_kn,floor,ours,eb_level,f_bias_kn,strain,learn_rpm` among others.

## Simulator results, current logic
`tools/sim/run.sh`, 27 scenarios: solo 80 km/h flat -> 12 at 932 rpm (the base logic: 11 at ~1200); light acceleration lands at Eco 971 / Normal 1002 / Power 1190 rpm mean; 40 t at 85 on 1 % stays in 11 (12
lacks reserve); wrong-mass Eco trap: one no-pull lesson, then waits for flat (before the guard: 9 hunting pairs); cruise control bang-bang: 0 shifts; M: limiter upshifts, no lugging help, stop downshifts;
M with a hint the game does not carry out (M5): three presses give 7-8 (the limiter came first), 8 -> 9, 9 -> 10, and with the game doing it (M4) no double shifts.
