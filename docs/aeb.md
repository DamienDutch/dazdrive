# Own AEB (emergency braking)

Implemented in `plugin/dazdrive/modules/cruise.cpp` (section AEB, `aeb()`); the sim is `tools/sim/run_cruise.sh` (`aeb_run`).
Research and design background: `docs/aeb-research.md`.

## What the game does (why DazDrive replaces it)
- With the game's AEB option off (`g_emergency_brake 0`; related options `g_emergency_brake_mode`, `g_blind_spot`, `g_acc`) the stock AEB never fires.
  The sudden air-pressure dumps seen with ACC come from **the ACC's emergency part** (`0x1409937ea`: cruise on, driver throttle <= 0.1,
  `0 < gap(+0x1a8) < 0.5 D(v)`): it sends the service brake `(D/2 - gap) / (D/2)` straight to the brakes every tick
  (no slew), so every gap wobble is a new application.
- The stock AEB proper (`g_emergency_brake` 1/2, block `0x140995c62`): `+0x17c = 1.0` at `0x140995e52` (mode 2: obstacle inside braking
  distance > 0.1 s) and `0x140995ee5` (radar lead closing). Sets the collision warning bit `+0x114 |= 0x10`.
- While DazDrive's AEB runs, both game parts are skipped (patched out, see below); if any patch cannot be applied the game's own behavior stays.

## Patches (1.60 only, together with the radar hook, the cruise function hook and the 1.60 layout; flag `s_aeb_owns`)
| what | pattern (unique in 1.60 .text) | patch |
|---|---|---|
| ACC emergency part: gap load `movss xmm7,[rsi+0x1a8]` | `80 BE 99 01 00 00 00 0F 84 ?? ?? ?? ?? 48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? 0F 2F C6 0F 86 ?? ?? ?? ?? 0F 2F 3D ?? ?? ?? ?? 0F 87 ?? ?? ?? ?? 48 8B 46 18 F3 0F 10 BE A8 01 00 00` (+0x33, 8 bytes) | new `game_patch_zero_xmm_if`: xmm7 = 0 while ours runs -> `comiss xmm7,xmm6(0); jbe` skips the part |
| AEB store 1 | `41 0F 2F F8 73 14 44 0F 2F 8B 0C 01 00 00 73 0A C7 83 7C 01 00 00 00 00 80 3F` (+16, 10 bytes) | `game_patch_skip_if` |
| AEB store 2 | `84 C0 75 0D 83 8B 14 01 00 00 10 EB 1A 84 C0 74 16 C7 83 7C 01 00 00 00 00 80 3F` (+17, 10 bytes) | `game_patch_skip_if` |
If any patch fails the flag goes to 0 and every patch is inert (the game's stays). The radar runs with `g_acc > 0` or blind spot / AEB option / auto high beams
(`docs/cruise-re.md` § 3); the log warns when none is on. Our requests go through `brake_request()` (`+0x17c`; with the AEB option off nothing in the game
clears it, we write 0 back).

## Behavior
- **Input:** the radar hook (gap, lead speed), fresh < 0.3 s, gap 0.3-100 m. A gap jump > 3 m + 15 % is a new target. Lead speed is low-passed over 0.25 s,
  lead acceleration over 0.5 s, the measured gap rate over 0.4 s. Active with cruise on or off (`aeb_without_cruise` 1), in forward gears, engine on, no parking brake.
- **Threat measure:** `ttc = gap / closing`, and `a_need` = the deceleration needed to stop `3 m + 0.3 s x lead speed` behind the lead (a standing lead: 3 m).
  Behind a *moving* lead at low speed the stop margin is smaller, 1.5 m + 0.1 s x lead speed, and fades to the full `3 m + 0.3 s x lead speed` as the lead's
  speed goes from 18 to 32 km/h (the lead's speed, not the truck's, because the truck's drops while braking and the margin must not shrink then). A lead that is
  pulling away (acceleration > 0.3 m/s2) reduces `a_need` and gives an acceleration-aware TTC. A braking lead (accel < -0.8) counts as one that brakes to a stop.
  A heavier truck (mass / 30 t, 1..2) acts earlier: TTC thresholds x sqrt, a_need thresholds / sqrt.
- **Stages:**
  - *warning*: TTC < 3.0 s or a_need > 1.5 m/s2 (two tones per second, the cruise throttle is cut). With cruise/ACC on the warning needs TTC < 2.0 s or
    a_need > 2.8 m/s2 (a take-over warning; the ACC is already braking).
  - *partial*: TTC < 2.0 s or a_need > 2.5 m/s2; about 3 m/s2, 0.7-1.15 x `aeb_partial_decel`.
  - *full*: TTC < 1.2 s or a_need > 0.6 x 6.5; 1.4 x a_need, `aeb_max_decel` 6.5 below TTC 0.8 s.
  - Up at once, down one stage after 1 s (warning 1.5 s) below the thresholds with margin (TTC x1.3, a_need x0.75). It enters only above `aeb_min_kmh` (15),
    then brakes to a stop and holds (request 0.4) until the driver gives gas, brakes (Auto Hold takes over), switches cruise on (ACC holds) or the lead drives off.
- **Request:** `(a_cmd - 0.4) / 10 x heavy` (fitted: request 0.1 is about 1.3 m/s2, 0.3 about 3.5) plus a slow loop on the measured deceleration; up 3/s (full 6/s),
  down 1.5/s; no floor once the truck is no longer closing. Air below 90 psi: at most 0.6 outside the full stage; air low during the event: no release while closing
  (no pulsing). Not paused by `air_min_psi` (this is a safety brake). Requests go through `brake_request()` (`+0x17c`); with the game's AEB option off nothing in the
  game clears the field, so DazDrive writes 0 back itself.
- **Only the throttle cancels:** pedal > 0.8, or > 0.5 for 0.3 s, or 0.25 more than when the braking began (a pedal that was already held does not cancel by
  itself). Then no braking for 3 s (re-arms at < 5 m and closing). Crossing traffic (the gap closes much faster than the lead speed explains) also ends braking
  (not a threat). The indicator, the driver braking enough, a target younger than 0.3 s (unless < 12 m and a_need > 3) and a moving lead near a junction
  (< 60 m: partial < 1.5 s, full < 1.0 s) only keep the AEB from starting.
- **Evasive lane change** (`aeb_evade_steer` 0.01): one indicator (game_actor `+0x1d0`, bit 4 left, 8 right) plus steering input (SDK `truck.input.steering`, + = left)
  moved >= 0.01 that way, or the yaw rate (SDK `truck.local.velocity.angular` y, + = left) >= 0.004 rot/s (1.4 deg/s) that way, both relative to a 1 s average from
  before the warning (a bend is no evasion), for 0.3 s while in the warning or partial stage: the AEB stands down for 3 s (ends at once, like the gas pedal), unless the
  lead is still < 10 m ahead and TTC < 1 s. Full braking is not interrupted by it. Sim: slow truck + indicator + steering gives only a warning and no braking;
  indicator without steering brakes as before.
- **Tailgating:** no warning for a merely short gap (like a real car, no nagging). Following closer than 1 s uses faster lead filters (speed 0.1 s, accel 0.2 s
  instead of 0.25 / 0.5) so a braking lead is seen sooner. Really close (< 0.4 s, > 40 km/h, for 2 s): one warning, again only after the gap has been > 1 s
  (log line `AEB: very close behind the lead`).
- **One application:** while braking and closing in, dips of the target below 0.08 are ignored; the stage floors fade continuously with the closing speed, so a steady gap
  behind a braking lead does not flip between two requests (sim before the change: 56-78 reversals, -155 psi at 7 m).
- **Hazards:** the AEB's request mutes the game's hazard rule; `emergency_hazards` (> 4 m/s2 above 50 km/h) switches them on as before.
- **Radar:** it runs when `g_acc` > 0 or blind spot / the AEB option / auto high beams are on (`docs/cruise-re.md` section 3); the log warns when none is on.
  Open point: the radar's lead is not guaranteed to be in the truck's path in bends (parked or oncoming cars).

## Logging
- `dazdrive.log`: `AEB: warning / partial braking / FULL braking: lead ... TTC ... needs ...`, `AEB: over after ...: highest ..., km/h, min gap, min TTC, peak decel,
  air psi, held back by ...`, `AEB: overruled by the gas pedal`.
- `cruise.csv` has the columns `aeb,ttc,a_need,a_cmd,decel` (also written while the AEB acts with cruise off).
- **Incident files** (`aeb_logs_mb` 10): every event gets its own file `plugins\dazdrive\aeb\aeb_<yyyy-mm-dd_hh-mm-ss local>.csv` at 20 Hz: the 10 s before the
  event, the event, and 10 s after it is over. Columns: t, kmh, gap, lead_kmh, closing, ttc, a_need, stage (0-4), request, a_cmd, thr, brake, decel, steer,
  yaw_deg_s, indicator (1 L, -1 R, 2 hazards), air. The folder is trimmed oldest-first to 90 % of the cap before a new file is created; `aeb_logs_mb` 0 = no incident files.

## Sim (26 t unless said; request -> 0.4 + 10 r / (m / 30 t) m/s2, game slew 5/s; the driver holds 30 % throttle and does nothing)
| scenario | result |
|---|---|
| stopped queue at 80 (first seen at 100 m) | partial only, peak 2.9 m/s2, stops 3.9 m behind, air -15 psi |
| same 60 t / air 80 psi | 4.5 m / 4.3 m behind, partial only |
| slow truck 40 at 80 | partial, min gap 5.8 m, never full |
| cut-in 20 m at 30 (we 60), brakes to 0 | full, peak 6.6, min gap 2.1 m, no hit |
| car crossing 25 m ahead (we 40) | nothing |
| kick-down during partial, then lane change | request 0 within 0.4 s, stays off |
| driver brakes himself 4 m/s2 | nothing |
| following 25 m at 80, lead brakes 5 m/s2 to 0 | full, peak 5.5, min gap 2.8 m |
| tailgating 12 m / 7 m at 80, lead brakes 4 m/s2 to 0 | full, min gap 3.5 / 3.3 m, 4 reversals, -28 psi |
| own ACC + AEB, lead brakes 3 m/s2 80 -> 30 | min gap 16.3 m (game's emergency part: 7.5 m), partial briefly |
The sim's brake model is a fit, not the game: compare the real deceleration per request (`decel` against `req_ga` in `cruise.csv`).
