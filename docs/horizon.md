# What DazDrive could still do

Ideas and open questions, based on the reverse-engineering notes in this folder and the SDK headers (`plugin/sdk/include/`). Nothing listed here is built or tested; the point is an honest
inventory of what is still possible, what it would cost and what the game does not provide. The data sources and their status per candidate are in `docs/horizon-data.md`.
Candidate numbers are stable references (A1 = lead speed and A6 = Auto Hold are already implemented, see `docs/autohold-acc.md`, so they are not listed).

## Where it stands

- **DazDrive covers the feel of a modern truck.** Compare the list of what a 2024 Volvo / Scania / Mercedes does on the road (`docs/smart.md` section 1) with what is in there now: predictive shifting and
  eco-roll, drive modes, smooth ACC with Stop & Go and time gaps, ISA, bends/roundabouts/junctions, descent mode, hill start and Auto Hold, crawl/maneuver mode, lane centering, an own AEB, and
  cruise keys that behave like a real steering-column stalk. The game already has blind spot warning and auto headlights/high beam itself. What a real truck does beyond that (a speed sign ahead,
  turn assist with radar) the game cannot do, or only with a big RE job.
- **The value now lies in polish and robustness**, not in new modules: lane centering is experimental, crawl mode has only recently become smooth, roundabouts are a heuristic without a flag, and ISA does
  not look ahead.
- **Worth it** (small): audio signals for what the HUD knows but a HUD-less driver does not see (blind spot, collision warning), hot-brake awareness in descent mode.
- **Leave alone:** everything that needs a learned database, a second HUD or nagging, and everything whose data is not in memory (traffic lights, signs ahead, distance to the loading dock).
- Short version of "should there be more?": the game has no lane-level data, no sign data ahead, no proximity sensors, GPS distances at 19x scale and a simple tire model. What is still possible is small
  and mostly audio.

## 1. Candidates

Per candidate: what, the real-world counterpart, value, feasibility (data/hook, what is already known, what is not), effort S/M/L, risk. Ordered by preference within each group.

### A. Longitudinal and traffic

**A2. Traffic-aware ACC: cut-in prediction and "lead leaves the lane"**
- What: besides the radar lead, read the AI list: a car moving from the overtaking lane into the own lane means building up the gap early; a lead taking the exit means not braking along with it.
- Real: multi-target radar + camera (Mercedes ADA, Scania ACC) does exactly this.
- Value: the remaining part of "the game's ACC is dumb". Noticeable on the motorway, especially with a short gap.
- Feasible: `game_traffic_u` global via pattern (`docs/research.md` section 6): lists `spawned_vehicles_1/2` with placement, `physics->speed`, `acceleration` (ets2la_plugin layout, loads on 1.60).
  **Not done:** verifying the struct offsets ourselves, lateral assignment (the route polyline as lane axis; there are only node positions, no lane width, so the lane offset o1 of the game's lane
  keeping may help), what exactly the radar function picks as lead (not decoded).
- Effort M-L. Risk medium: a wrongly read list means a crash; log only at first.

**A3. Full speed planning over the profile (beyond the current band and dip)**
- What: the current predictive cruise adds a band before a climb and eases off before a crest (`docs/predictive-cruise.md`). A planner would optimize the speed over the 1.5 km ahead (mass, torque
  curve, drag, grade) with adjustable under/overrun and pick where eco-roll gains the most.
- Real: Mercedes PPC, Volvo I-See, DAF PCC 3 (`docs/smart.md` #1).
- Value: less throttle flapping on rolling terrain, longer rolls, "it knows what is coming".
- Feasible: everything is already there: heights every 25 m up to 1575 m (`core/route.cpp`), live mass, torque curve, `f_bias` (learned resistance), the cruise sim (`tools/sim/run_cruise.sh`). Only
  calculation plus tuning.
- Effort M. Risk low (sim first; without a route = current behavior).

**A4. Downhill speed control with an own stage choice and predictive release**
- What: on a descent pick the engine-brake / retarder stage from grade and mass with an overspeed band (+2..+5), and release before the end of the slope (route). Now: descent mode sets
  `g_cruise_control_smart` and lets the game brake (retarder full at 1 km/h over, `docs/cruise-re.md` section 2).
- Real: Volvo Downhill Cruise, Iveco GAP, Scania downhill speed control.
- Value: realism and calm (no retarder on-off around the set speed).
- Feasible: setters known (engine brake `0x1405c7680`, retarder `0x1405c74c0`, `docs/engine-brake-re.md`, `docs/cruise-keys-re.md`).
  **Catch:** the cancel hub (`0x140993321`) turns cruise off as soon as `+0x180` (retarder) > 0 or the manual-engine-brake flag is set, so via the setters an own stage would cancel cruise. An own stage
  has to go through the demand floats (`inner+0x143c/+0x1444`), which are rewritten every tick, so the stores in the pedal function (`0x140993c16/bed`) would have to be patched. "Retarder up" is
  used as a cruise cancel by players; that has to stay.
- Effort M. Risk medium. A "retarder + cruise like Volvo" variant is not planned; this is the milder one. Only worth it if descent mode hunts in practice.

**A5. Hot-brake awareness**
- What: also read SDK `truck.brake.temperature`; above a threshold in descent mode ask for retarder/engine brake earlier and for less service brake; one tone as a warning. TrueShift then keeps the rpm
  higher on a long descent.
- Real: brake temperature warning, "brake fade" on the dashboard.
- Value: small but real; ETS2 has brake fade. Fits a cockpit without HUD.
- Feasible: the channel exists (`scssdk_telemetry_truck_common_channels.h`), not registered yet. Otherwise only logic.
- Effort S. Risk low.

**A7. Traffic-light anticipation (release early for red)**
- What: a red light on the route ahead means coasting and downshift planning as for a junction. Real: only passenger cars.
- Value: limited (lights are in cities, where speeds are low anyway).
- Feasible: **unknown.** Semaphores are actors in the traffic list (`docs/research.md`), but the state (red/green, timing) and the link to the direction of travel are not decoded. Effort L. Risk high.
  Leave alone.

**A8. ISA before the sign (limit ahead)**
- What: let the set speed drop before the sign is passed.
- Real: Volvo I-See speed limits, Scania Speed Sign Information.
- Feasible: no static source (`docs/speedlimit-ahead-re.md`); a probe found no predictor in the route nodes. What remains is walking from a node's pointee chain to the world sign items, or finding
  a "next limit" field next to `game_actor+0x194`. Only if the game has it in memory itself; a learned database is out of scope.
- Effort M-L. Risk: a good chance it yields nothing.

### B. Drivetrain

**B1. Grip-aware driving (wheel slip + substance)**
- What: wheel speed x radius against truck speed gives slip; `wheel_substance` (id from the `substances` config). On slip: a higher start gear or less throttle in crawl, no eco-roll on slippery
  surfaces, less engine-brake downshift. Real: ASR, I-Shift winter mode.
- Value: moderate; without winter mods there is little grip difference, and wet via the wipers covers most of it. Feasible: SDK channels `wheel_velocity`, `wheel_substance`, `wheel_on_ground` (not
  registered). Effort S-M. Risk low.

**B2. Overtake hint: left indicator + throttle = temporary Power**
- What: left blinker above 60 km/h with pedal > 0.5 gives the Power shift points for 10 s (no mode change, no letter).
- Real: kickdown/"overtaking" logic in Scania Opticruise; MAN EfficientCruise has an overtaking setting.
- Value: charm, small. Fits "Normal is adaptive enough".
- Feasible: SDK `lblinker` is already there; the `demand` path in TrueShift. Effort S. Risk: false triggers when turning (hence only above 60).

**B3. Torque-converter and multi-torque gearboxes (mod robustness)**
- What: TrueShift assumes a clutch AMT with one torque curve. Mod trucks have `secondary_torque` (multi-torque below a ratio), `stall_torque_ratio` + `torque_converter_lock_gear` (TC automatics,
  ATS pickups), electric trucks (regen, 1-2 gears).
- Value: "works on every truck" is a requirement for a broad release.
- Feasible: def fields known (`docs/game-defs.md`); runtime offsets for secondary torque not searched. Electric: signature in the pedal function (`engine+0x1b8 == 0x16829f66307`,
  `docs/engine-brake-re.md`) means the module switches off cleanly.
- Effort M. Risk medium (cannot be tested without such a truck in the garage). Belongs to polish, listed here because it needs RE.

### C. Lateral, warning, assistance

**C1. Audio for what the HUD knows and a HUD-less driver does not see (blind spot, collision warning)**
- What: the game sets bits in `ga+0x114` (0x10 collision warning, bits 1/2 blind spot, `docs/cruise-re.md` section 3). Blinker on + spot occupied gives a short tone; a collision warning gives a
  separate tone. No HUD needed.
- Real: Mercedes Sideguard, Scania Lane Change Collision Prevention (the warning part).
- Value: high for the price, since the stock warnings do not exist for a driver without HUD.
- Feasible: reading known bits + `core/sound.cpp`. `g_blind_spot` has to be on (stock option).
- Effort S. Risk low. Nagging danger: only with blinker and at most once per 3 s.

**C2. Turn assist (turning right with something beside the truck)**
- What: right blinker, < 30 km/h, vehicle in the right-hand box gives a tone.
- Real: MAN/DAF/Mercedes Turn Assist (EU-mandatory on new trucks since 2024).
- Feasible: needs the traffic list (A2); the game's blind-spot bit may already cover low speed (unknown whether it works below 30).
- Effort S after A2, otherwise M. Risk as A2.

**C3. Lane-change prevention / steer back**
- What: lane keeping steers back if the lane is changed without a free lane.
- Real: Scania LCP, Mercedes ADA3.
- Feasible: only once lane centering is stable and A2 exists. Effort L. Risk high (an earlier near-crash showed that a wrong steering signal is dangerous). Leave alone.

**C4. Docking/coupling aid on articulation angle**
- What: while reversing in crawl: a tone that rises with the truck-trailer articulation angle, a warning at the jackknife limit; a calm tick when straight behind. No distance to the loading dock (that
  data is not there).
- Real: jackknife warning when reversing (Volvo), trailer reversing assist (vans).
- Value: charm, belongs with crawl mode; without HUD or mirrors on a button, an audible angle is really handy.
- Feasible: `trailer.0.world.placement` against `truck.world.placement` gives the angle; `trailer.0.connected`. Distance to objects: **not available** (no proximity data, loading dock not in the
  traffic list).
- Effort S. Risk low; only in R and crawl, otherwise it nags.

**C5. Trailer stability (ESP/RSC feel)**
- What: trailer `local_angular_velocity` + articulation angle at speed: on a swing-out no engine brake/downshift/eco-roll, plus a tone. A trailer-only brake (Volvo Stretch Brake) is not possible: the
  "trailer brake" input is not hooked, `+0x17c` brakes the whole combination.
- Value: low (ETS2 rarely jackknifes below the limit); for ice mods. Effort S-M. Risk low. Only together with B1.

### D. Charm, outside the truck

**D1. Trip report after delivery (eco coaching without nagging)**
- What: on `job_delivered` one summary to `dazdrive.log`/CSV: consumption (`fuel_average_consumption`), coasting share, eco-roll seconds, shifts per km, hints, hard braking. Optionally one tone "report
  ready". Post-drive analysis via `tools/analyze_trueshift.py`.
- Real: DAF Driver Performance Assistant, Scania Driver Support (the scoring side).
- Value: nice for someone who drives economically; nothing in-game (no HUD, no nag).
- Feasible: gameplay events + fuel channels in the SDK; the rest is script. Effort S-M. Risk none.

**D2. Phone dash (second screen like an infotainment)**
- What: the DLL serves a page on the LAN (letter, gear, mode, ACC setting, ISA, route profile, lead gap). A phone next to the wheel is the display a real truck has and a HUD-less cockpit does not. No
  overlay, so no HUD.
- Feasible: winsock HTTP in a thread of its own (works under Proton), or a JSON file every second that any script can serve. Effort M. Risk low (isolated thread, never in the game tick).

**D3. Drive viewer (replay analytics)**
- What: from the CSVs, one HTML per drive: speed, gear, rpm, mode, events, why-lines. Replaces reading logs by hand after every test. Value: high while tuning, low afterwards. Effort S-M. Risk none.

**D4. ATS 1.61 live (road trip with pickups/cars)**
- Layout verified offline (`docs/ats-161.md`); the tail/stop/cruise/lane patterns are 1.60-only, so in ATS only TrueShift + route work. Effort S (start once and log) + M (1.61 patterns for more).
  Risk low: everything fails safe.

## 2. Polish and open questions: what should be better in what exists

1. **Lane centering** (highest priority, safety). The model-based law is close to the game's law plus a safety rule. Open: a measurement drive with deliberate hands-off stretches, damping based only
   on real data (the sim is too pessimistic: do not tune on the sim alone), the release of the assist after switching off, and behavior when the lane is lost. Goal: roughly 10 s hands-off on the
   motorway, VW-like: further from the center = more assist, steering yourself = less.
2. **Roundabouts and junctions**: the detection is a heuristic without a flag: >= 3 curves, or 2 with ratio >= 1.2, or ratio >= 1.4 with chord >= 15 m. Known gaps: straight through a large
   roundabout (ratio < 1.4), city scale (now local), prefab words send no flag. Todo: the brake request in the trace, one drive with 5+ roundabouts, then fix the thresholds. Possible: the prefab
   descriptor via item* -> heap (RE, M).
3. **Crawl mode**: heavy load + grade (governor torque unknown, fallback = merged throttle), reverse pedal semantics, the interaction with Auto Hold and with TrueShift's handback at 6-10 km/h.
4. **ISA**: the ease-down rate is a setting; limit ahead needs a source (A8); the mph unit is not handled (ATS).
5. **Eco-roll**: `eco_roll_max_decel` 0.04 is strict; with a full planner (A3) this becomes the place where the gain is.
6. **Robustness on other trucks/gearboxes**: B3 (TC/multi-torque/electric); `transmission_class standard`; gearboxes with crawler gears (`crawls`); 6-speed cars; idle learning after engine start
   also on a truck change in the garage.
7. **Edge cases that go wrong once per session**:
   - pause menu / Steam overlay: dt jumps (filters, hold timers, rate limits), so clamp dt and reset holds at dt > 0.5 s; an earlier "stuck after the pause menu" case was the `+0x17c` latch, but dt
     robustness is untested;
   - save/load, truck or trailer change, teleport (quick travel/ferry/train: gameplay events exist in the SDK): reset the route cache, mass, `f_bias`, learned idle, hunting lessons; re-validate the
     game_actor/gearbox pointer (hooks compare `ga+0x1d68`);
   - engine off/on, electric truck, profile without an automatic (shifter type from the SDK: modules off);
   - `cruise_state.txt` and the ini: write errors (read-only folder) must never stop a module.
8. **Input without FFB / without a wheel**: the lane module assumes FFB (assist = wheel target); with `c_ff_enabled` 0 or mouse/keyboard the assist adds up in `veh+0x640`
   (`docs/lka-roll-re.md` A1), which is a different behavior and is handled by falling back to the game's own law. Mode/crawl toggle: the game keybind works (the cvar as source), so keyboard users need nothing extra.
9. **ETS2 1.61 and ATS 1.61**: `tools/check_suite.py` on a new exe; everything that is 1.60-only fails safe but silently, so the status line has to say so out loud (D4).
10. **Small, known**: a second - tap during easing counts from the eased value; the `g_acc` +0.1 trick and the accmode wrapper are fragile on a game update; `curve_lat` 3.25 may need tuning; the
    `junction ahead` upshift block on ProMods (a prefab every 130 m).

## 3. Out of scope (and why)

- **Learned maps/databases** (height, limits, roundabouts per map): route memory makes it unnecessary for height; for limits there is no source.
- **Traffic-light anticipation** (A7): data not decoded, low value, big job.
- **Auto headlights / auto high beam / auto dip for oncoming traffic**: stock (`g_automatic_headlights`, `g_auto_traffic_headlights`, `g_automatic_high_beams`). Tunnels: no data.
- **Rain-sensing wipers**: rain intensity is not in the SDK (`docs/smart.md` section 2 #17); the wet flag is read from the wipers instead.
- **Tire temperature / tire wear**: not in the SDK (only `wear_wheels` as a sleet number); no physical effect in the game.
- **Fatigue / hands-off nag / SafeStop**: nagging is not wanted; the game already has a sleep mechanism (`next_rest_stop`).
- **Multiplayer convoy mode**: see `docs/multiplayer.md`. "Following an AI convoy" is ACC.
- **Voice control**: out of scope.
- **Retarder + cruise like Volvo** (- on the stalk = retarder): descent mode covers it, and "retarder up" is the usual cruise cancel.
- **One-pedal driving**: feasible, but not wanted by default (maybe for keyboard users).
- **Smart reverse mode**: R behaves like D.
- **Lane-change steer back (C3)** as long as lane centering itself is experimental.
- **In-game overlay/HUD from DazDrive**: the dashboard letter + tones are the whole UI concept. A phone dash (D2) is the only exception.
- **Distance to the loading dock / parking sensors**: no proximity data in the SDK or known structs.
- **Lift axle automation, differential lock, rocking mode, engine stop-start**: inputs not hooked (no generic key injector, action objects are RE work per key), stop-start cancels cruise via the
  hub; low value.
