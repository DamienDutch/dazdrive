# What we found out about the game

Findings about ETS2 1.60.1.7 (the layout DazDrive is built and verified on); ATS 1.61 is in `docs/ats-161.md`. They come from static analysis of the executable, the SCS
telemetry SDK headers, the MIT-licensed `ets2la_plugin` and `car_automatic_transmission` sources, and two runs of a private probe plugin ("probe run 1" and "probe run 2": it logged
gearbox, engine, mass and route objects while driving). [V] = verified in the game or in the code, [I] = inference. Related notes: clutch and adaptive mode `docs/clutch-mode.md`,
design and hooks of the gearbox `docs/trueshift.md`, cruise `docs/cruise-re.md`, game definition files `docs/game-defs.md`.

## 1. Ways to influence the automatic gearbox
1. **Def mod**: transmission ratios and shift time plus the engine's `rpm_range_low_gear`, `rpm_range_high_gear`, `rpm_range_power_boost` (pairs = (downshift, upshift), interpolated from
   the low to the high gear; the power pair is mainly used with the adaptive automatic and kickdown). Not load or grade aware. Sources:
   https://modding.scssoft.com/wiki/Games/ETS2/Modding_guides/1.24 , https://forum.scssoft.com/viewtopic.php?t=316038 , https://forum.scssoft.com/viewtopic.php?t=275748
2. **Stock automatic plus an external tool** sending `gearup` / `geardown` through the controller interface: untested whether the automatic accepts and keeps that.
3. **Sequential mode plus an external tool** (one press per gear step).
4. **A native plugin hooking the stock automatic** (no external program): prior art exists (section 3). This is what DazDrive does; the game stays in automatic.

The "Manual Transmission" plugin by theGodspeed in the ETS2LA catalog (telemetry-driven sequential shifting) and MonoCruise (MIT, Python, https://github.com/luuukske/MonoCruise;
`core/sending_thread/scscontroller.py`, `checker/ets2_checker.py`) are prior art for the external routes under Proton; their sources were not studied in depth.

## 2. What the SDK gives and does not give
- **Telemetry** (SCS SDK, `Local\SCSTelemetry` for external readers): engine rpm, `engine.gear`, `displayed.gear`, input and effective throttle and brake, motor brake, retarder level,
  local linear/angular acceleration, speed, cruise control, orientation (pitch gives the slope), cargo mass. Configs: `forward.ratio[]`, `reverse.ratio[]`, `differential.ratio`,
  `gears.forward/reverse`, `shifter.type`, rpm max.
- **Missing**: engine torque/power curve (it is in `def/vehicle/truck/<brand>/engine/*.sii`, inside the .scs archives, or in memory, section 4), total truck mass (section 5),
  wheel slip (wheel rotation against speed gives it), rain intensity, tire temperature and wear (only `wear_wheels`), distance to objects, lane width, speed limit ahead.
- **Controller output** (`scs_sdk_controller`, `Local\SCSControls`, layout from `inputs.h` / `scscontroller.py` in https://github.com/ETS2LA/scs-sdk-controller): bools `gearup`, `geardown`,
  `gear0` (neutral), `geardrive`, `gearreverse`, `gear1`..`gear16`, `gearsel1on/off/tgl`, `gearsel2on/off/tgl`, `transemi`, `gearuphint`, `geardownhint`. They are levels read every
  frame: hold True for at least one frame (~33 ms), then False = one press. Quirk: wipers can cycle until `wipers0` is sent.
- The game has separate `physics_gearbox_sequential_u` / `_automatic_u` / `_direct_u` classes (ets2la_plugin `src/prism/game_actor.hpp`). In sequential mode up/down is expected to step through
  every gear (split/range). [I]
- `gearuphint` / `geardownhint` work in Automatic [V] (down tested); a hint just writes the target gear (`self+0x68`) with no flag of its own, so a hint is detected as a target change that
  our chooser stubs did not produce. A hint downshift came with an effective throttle of 0.2 (a rev-matching blip).

## 3. Hooking the stock automatic (upstream `car_automatic_transmission` and the 1.60 port)
- **Upstream** (`deppelover/car_automatic_transmission`, MIT, C, ~700 lines, commit 254a2f6; the copy lives in `plugin/upstream/`): one DLL in `bin/win_x64/plugins`, loaded via the
  telemetry SDK entry point (`scs_telemetry_init`), then patches the game's automatic in memory. Single player only. It finds code by byte patterns (`PAT_WINDOW`, `PAT_BOUND`,
  `PAT_LOBOUND`, `PAT_CHOOSER`, `PAT_GATE`) and redirects call sites to stubs:
  - `window(self, out, gear)` returns in `out[0..1]` the (downshift rpm, upshift rpm) window of a gear: the main lever.
  - bound functions = the highest gear allowed on upshift/kickdown, the lowest on downshift: step limits.
  - the gate = the "no downshift while accelerating" check (patched to a jump).
  - struct offsets used: gear `inner+0x14b0`, throttle `self+0x68`, rpm limit `engine+0x234`, ratio table `self+0x48/0x50`.
  - any pattern missing means no patch at all (the status line names which).
- **Pattern check on disk**: `tools/patscan.py <exe>`; offline emulation of upstream's `discover()`: `tools/discover.py <exe> 1.61 1.60`. ATS 1.61: the upstream set passes (7 window call sites).
  ETS2 1.60.1.7: the upstream set fails; the **1.60 layout passes** (same call structure: the chooser calls ED0, E70, E60 in the same order; 7 window call sites; gate `jb` at +12).
- **The 1.60 layout** (found by diffing ETS2 1.60 against ATS 1.61; the code keeps a `layout` table, the first layout whose patterns all match wins, and the status line names it):
  - inner-object fields -0x68: gear `0x1448`, flag `0x1440`, gate field `0x143c`.
  - engine handle `inner+0xff0` (1.61 has two handles, `0x1038`/`0x1040`); engine fields -8: rpm limit `0x22c`.
  - throttle: 1.61 reads gearbox `self+0x68`; in 1.60 that is an int state field (set to -1), the throttle is `inner+0x1424` (same compare spots in the window and chooser functions).
  - step table `self+0x40` (+0x48 data, +0x50 count) and `self+0x20` unchanged.
- **Live in the game** [V]: "applied (1.60 layout), 13 patches"; upshifts go one gear at a time (unlike stock), so the hooks are active.

## 4. Objects and offsets (1.60)
- **Chain**: `game_actor` = `[[0x143550398]+0x31a8]`; `game_actor+0x18` = the physics vehicle ("inner", the same object as `self+0x10` of the gearbox); `game_actor+0x1d68` = the gearbox object
  (a hook-free path to it). Engine object `inner+0xff0` (resolver), transmission def `inner+0xfe8`.
- **Verified live** [V]: `inner+0x1448` = SDK gear, `inner+0x1424` = input throttle (`+0x1428` effective), engine `+0x22c` = rpm limit (= SDK `rpm.limit`).
- **Engine** (test truck: Volvo FH6, 420 hp, 2400 N·m): `+0x1c0` torque 2400 N·m; `+0x1d0` pointer to the torque curve, float2 pairs starting at `ptr+8`: (600,0.62) (900,1) (1230,1)
  (1600,0.77) (1700,0.69) (1800,0.44) (2000,0). Power = torque x factor x rpm (2400 N·m x 1230 rpm = 309 kW = the 420 hp in the UI). `+0x22c`/`+0x234` 1900 (limit, limit in neutral).
  The rpm_range float2s: `+0x23c` (1500,1900) engine brake (the window function returns it when inner flag `0x1440` is set), `+0x244` (750,1450) low gear, `+0x24c` (900,1250) high gear,
  `+0x254` (1200,1650) power. `+0x298` 1150 (unknown).
- **Stock window** = interpolation low to high over the gears: gear 1 ~ 759-1454 rpm, gear 12 ~ 910-1263 rpm (light throttle). According to a forum report the adaptive modes barely change anything: with them on, the game
  shifts at exactly the `rpm_range_*` values (default 1000/1500).
- **Shift mechanics**: `self+0x68` = target gear (set ~0.38 s before the gear changes), `self+0x18` shift state (0 idle, 2, 1, 3), `self+0x34` bit 0 = request pulse, `inner+0x1430` = 1.0 while
  shifting, `inner+0x1434` 6 to 12. A shift takes ~0.4 s to the gear change and ~0.6 s to done (with a dual-clutch mod).
- `inner+0x2a0` bit 0x80 toggles every 0.34 s = the blinker.
- **Throttle/brake override**: the game_actor input-mix offsets come from the ets2la_plugin patterns `48 B8 42 7C FD CC 88 01 00 00 48 8D 95 ?? ?? ?? ?? 48 89 85 ?? ?? ?? ?? 48 8D 8F` (+27, throttle)
  and `48 B8 C7 91 B7 6C 4E 3A 00 00 48 8D 8F` (+13, brake); writing `input->value` overrides the pedal. They match ETS2 1.60 (1 hit). Basis for the crawl function.

## 5. Mass
- Fields `self+0x78` (31716 kg with 17.2 t cargo: cargo + truck + trailer), `self+0x88..+0xf8` step 0x10 (~31790, per gear, maybe equivalent mass with inertia) and `inner+0x1098` (32105 in one
  truck, garbage in another) do **not** change when the trailer is dropped; `self+0x78` looks like the train mass at job start. No mass-like float in self/inner/engine changed at all.
- The stock automatic picks its start gear by a **live** mass: with a trailer (~30 t) always gear 1 (7 starts, flat), solo (~10 t) gear 4 (3 starts, also at full throttle). So some live
  mass exists; it is read through the game function `mass(game_actor, 1)` (the same one the stock automatic uses), found later and used by TrueShift.
- A physics estimate from full-throttle pulls (torque x curve x ratio x diff x 0.9 / wheel radius against acceleration, grade and rolling resistance) gives a median **30 t with trailer, 10.3 t
  solo**, plausible; `trailer.0.connected` is a reset trigger for the estimator.

## 6. Route and traffic in memory (probe run 2; ets2la_plugin, commit c925bd9, MIT, built for 1.60)
All of these patterns hit exactly once in ETS2 1.60.1.7 (offline scan), and `ets2la_plugin.dll` loads fine on it.
- **Entry**: `base_ctrl` pattern `48 8B 15 ?? ?? ?? ?? 48 8B ?? 48 8B 41 ?? 48 8B 92` (rip-relative global at +3; disp32 at +17 = the `game_actor` offset in base_ctrl). ETS2 1.60.1.7: slot RVA
  `0x3550398`, game_actor `+0x31a8`. While driving, base_ctrl is a `game_ctrl` (vtable slot 5 = get_unit_descriptor, `*desc->p_class_name == "game_ctrl"`).
- **GPS**: `gps_manager_t` = game_ctrl + disp32 from `48 8D 88 ?? ?? ?? ?? 48 85 C9 74 ?? 48 8B 01 48 8D 54` (+3); 1.60 = `0x40f8` (2 hits, same value). Windows layout: route_task* `+0x28`,
  trip_distance `+0x274`, trip_time `+0x278`, next_checkpoint_distance `+0x27c`. route_task is NULL without a route (and on arrival).
- **`route_task_t`**: `physical_route_items` array_dyn (data `+0x58`, size `+0x60`), `route_items` (data `+0x80`, size `+0x88`). Physical item (0x20): node_item_t* `+0`, dist/time from the
  previous `+0xc/+0x10`, dist/time till the end `+0x14/+0x18`. Logical item (0x30): type (0 node, 1 prefab) `+0`, node_id `+4` (junk `0xffffffff` for roads), item* `+8`, dist/time prev
  `+0x18/+0x1c`, till end `+0x24/+0x28`. Type 1 = prefab = a junction.
- **`node_item_t`** (0x38, no vtable): int3 coords `+0`, flags `+0xc`, quaternion (w,x,y,z) `+0x10`, uid `+0x30`. Node coords are meters x 256 (x, **y = height**, z) and match the SDK placement; the
  quaternion gives the road direction including pitch.
- **What the route offers** [V]: the physical items cover the **whole route to the destination** (14.5 km for the probed route), node spacing median ~60 m, max ~310 m. Linear interpolation of
  height between nodes against the driven SDK y: median error 0.0 m, p90 0.3 m (max 23 m: bridges, off-route). Good enough for look-ahead as it is. GPS distances (`dist_prev`, `dist_end`,
  `trip_dist`) are in **game-scaled meters, about 19x real** outside cities: use node geometry for real distances. Analysis helper: `tools/analyze_route.py`.
- ETS2LA itself only exports uid + distance + time per node (max 6000) in `Local\ETS2LARoute`; its app looks the positions up in its own map data. DazDrive reads the node directly.
- **Traffic**: `game_traffic_u` global via `48 8B D9 48 8B 0D ?? ?? ?? ?? 48 85 C9 74 ?? 48 8B 83 ?? ?? ?? ?? 48 85 C0` (+6); lists `spawned_vehicles_1/2` and `traffic_objects_1` give AI vehicles with
  placement, `physics->speed` and `physics->acceleration`. Traffic lights are semaphore actors in the same list (their state is not decoded).

## 7. The dashboard letter ("A", "E", "D", "P", "M")
- The stock dashboard of the test truck showed **"E"** (Eco) during a whole drive with the stock automatic; it seems to show the game's adaptive mode letter (E/N/P). A def mod can change
  the HUD's "A" to "D".
- The letter text is not a cvar value: candidates are the transmission def's `auto_mode_name` / `manual_mode_name` strings. The best candidate is a `string_dyn` pair ("A" at `+0x258`, "M" at
  `+0x278`, size 1, 0x20 apart) in the object at `inner+0x150` then `+0x358`; the same layout also appears in two other objects, `+0x530` variants and read-only copies in the exe. The
  transmission ratios are near it (`inner+0xb28` block). A search over the heap found 10 distinct "A" strings, so a blind write is not safe. Details and the unresolved mapping:
  `docs/clutch-mode.md` and `docs/engine-brake-re.md`.

## 8. Settings (cvars) seen
- `g_adaptive_shift` (the Adaptive Automatic Transmission setting; E/N/P map to 10 / 3 / 1.67 in the game's mapping; "0.0" = off), `g_upshift_coef` 0.3, `g_downshift_coef` 0.8,
  `g_intelligent_transmission` 0 in a typical profile. The adaptive mode has Eco/Normal/Power and switches itself off when cruise control is on [V].
- Cruise and ACC cvars: `docs/acc.md`, `docs/cruise-re.md`.

## 9. Stock behavior when braking to a stop [V]
- Braking gently (pedal 0.1) the box downshifts 10, 9, 8, 6, 4, 2, 1 in 9 s with a clutch dip each time.
- At ~2 km/h the clutch stays closed in gear 1 and the engine rpm falls to ~410-450 (idle 500): the idle governor pushes, the truck holds ~1.8 km/h against the brake for ~1.3 s, and only then the
  clutch opens. Real AMTs open the clutch near idle rpm in the start gear, and downshift less, skipping gears. (DazDrive's `smooth_stop` addresses this: `docs/trueshift.md`.)

## 10. What does not exist or is not available
- No live mass field and no engine torque in the SDK (sections 2 and 5). No wheel slip channel (derive it from wheel velocities).
- No lane geometry for the lane keeping assist other than what the game's own lane code reads (`docs/lka.md`), and no lane width in the route.
- No speed limit ahead (`docs/speedlimit-ahead-re.md`), no rain intensity, no proximity data, no traffic-light state decoded. The stock automatic does not keep a hinted gear for long (it shifts back
  once the throttle and rpm leave its window), so DazDrive holds a hint itself (`hint_hold_s`).
- The roundabout flag is not in the route nodes (junction and roundabout detection is a geometry heuristic in `core/route.cpp`).

## 11. Reference: how real AMTs decide (rules of thumb, no OEM publishes thresholds)
- **Inputs**: rpm, torque demand / pedal position and its rate (kickdown), speed, estimated gross combination weight, grade, brake/retarder; look-ahead topography (I-See learned hills, PPC,
  Active Prediction, PreVision).
- **Mass/grade estimation**: recursive least squares with forgetting on `m*a = F_traction - F_drag - F_roll - m*g*sin(theta)`, only in clean-torque windows, frozen during shifts and braking
  (patent US7499784, Vahidi/Stefanopoulou/Peng).
- **Strategy**: skip shifts when light or flat (Eaton skips up to about 9th from a standstill); heavy uphill: fewer shifts because a torque gap costs speed, allow early or multi-gear downshifts;
  crest: hold the gear, coast over the top, shift after it; downhill: eco-roll (neutral) when gentle, downshift for engine brake when steep, no upshift while braking; kickdown (pedal >= 90 % or
  fast) switches to a performance map and multi-gear downshifts; hysteresis (a few hundred rpm) plus a dwell of a few seconds; manual +/- is a temporary manual mode; modes eco vs power.
- **Numbers**: torque plateau from about 900 rpm (DAF MX-13 runs 85 km/h at about 1000 rpm); cruise 1000-1400 rpm; upshift 1300-1500 rpm (eco), higher in power; downshift 900-1100 rpm;
  shift time 0.3-1 s; 12/14 gears.
- **Per-tick sketch**: update inputs and estimators; hold while shifting or within the dwell (unless kickdown); for each candidate gear compute rpm = v x ratio x diff / wheel radius, the available
  wheel torque and the predicted speed after a 0.5-1 s torque gap; choose (eco: the highest gear above the rpm floor that still gives the demanded acceleration; heavy/uphill: require margin after
  the gap, otherwise downshift before lugging; kickdown: rpm near peak torque or power; descent with brake: keep rpm in the engine-brake band); send one press per step, then start the dwell timer.
