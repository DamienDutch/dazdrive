# Creep / maneuver mode: RE findings and design (ETS2 1.60.1.7, static)

Exe `eurotrucks2.exe`, base 0x140000000. Static disassembly only, nothing run in the game. [V] = seen in code, [I] = inference.
Names as in `clutch-mode.md` / `neutral-exit-re.md`: `self` = automatic gearbox, `inner` = vehicle (`self+0x10`), `drive` = `inner+0x150`,
`engine` = `drive+0x360`, `ga` = game_actor. Design decisions: `lowspeed.md`. Code: `modules/creep.cpp`,
`game_patch_clutch_floor` in `core/game.cpp`, sim `tools/sim/sim_creep.cpp` (`run_creep.sh`).

## Answers to the open questions

1. **Does the engine give torque at idle without the driver's throttle? Yes, via the idle governor.** [V]
   `0x1408cc4f0` (called first in the throttle fn `0x1408cc1e0`, result used as a floor for the engine throttle: `max(inner+0x1428, governor)`, `0x1408cc315`):
   `thr = gain * (1 + 0.1*(idle - rpm))`, `gain = drive+0x2f8` (neutral) / `drive+0x2fc` (in gear), cap `min(0.5, 10*gain)`, `>= 0`.
   So at rpm <= idle the governor opens the throttle (up to 50 %), above idle+10 rpm it is 0. `idle = engine+0x228 + drive+0x2dc` (rpm).
   The gains are filled in at init (`0x1408c6926..0x1408c6999`, lambdas `0x1408c70b0` / `0x1408c7160`, a torque-curve lookup `0x140b26730` at idle
   speed; the in-gear one also uses vehicle mass `0x140643ad0` and the gear ratio). [I] calibrated to hold idle against the driveline.
   This is also why the stock truck "pushes at 2 km/h": closed clutch + governor.
2. **Can the clutch slip at idle without a pedal? Yes, with a floor on the drivetrain's clutch copy.** [V code, I effect]
   `0x1408c82cf` is the only writer of `drive+0x330` (`mov eax,[rbx+0x142c]; mov [rsi+0x330],eax`, 12 bytes, pattern
   `8B 83 2C 14 00 00 89 86 30 03 00 00`, unique in .text, no jump targets into it, rbx = vehicle). Everything that couples engine and wheels reads `drive+0x330`:
   - engine speed `0x1408c8988..0x1408c8aa4`: with `e = 1 - drive+0x330` the new engine speed is the implicit step
     `w' = (w + dt/J*(T + a*J*w_wheel)) / (1 + dt*a)` (`0x1408c8ea0`), `a = e * g(clutch)`, `g` = `0x1408ca960` (interpolation between two
     global constants `[[0x143550398]+0x4d0]+0xb8/0xbc` by `(1-clutch)^2` [I]). So torque and slip are continuous in the clutch value: a partial value is a slipping clutch;
   - closed-clutch engine drag `0x1408c8f20` (only while `drive+0x330 <= 0`), stall detection `0x1408ca79e..0x1408ca83d` (uses `1 - drive+0x330`; stall when engine speed < 0.4 * idle, `0x1423d77b4`).
   Raising only that copy leaves the gearbox sequencer, the integrator `0x140622900` and the automatic's rules with the clutch they computed (stored `inner+0x142c` stays 0).
   Chosen over writing `inner+0x1430/+0x1434` (target/rate): in gearbox state 3 (closing) the sequencer rewrites those fields every tick (`0x140601a77`, after any tail hook),
   and in state 0 a partially open stored clutch would be closed by the integrator's own slip rule (clutch-mode.md) with different semantics.
   [I] the drive copy runs per physics tick for every vehicle: the thunk compares rbx with the player's vehicle and leaves all others alone.
3. **The automatic vs. creep (what has to be overridden):** [V]
   - rest rule `0x14098c6c9..0x14098c6fb`: A mode, `|inner+0x2f0| < 0.1 m/s`, throttle and brake args < 0.01 -> `self+0x68 = 0`, latch `self+0x101 = 1`, drive state 4->3 / 0->1.
     This is what makes the truck neutral at a standstill; after that the gap state waits for throttle (neutral-exit-re.md § 1.3).
   - at rest with the latch set (states 1..3): brake > 0.01 -> R (`0x14098c7a8..0x14098c83c`), throttle > 0.01 -> start gear, state 4 (`0x14098c7ba..`).
     In states 0/1/2 with gear 0 the brake also gives R (`0x14098c77a`). So at a standstill the brake pedal = "reverse", the throttle = "forward". Creep must not break that.
   - `rpm < 0.8 idle` (or `T` with brake) -> neutral (lka-roll-re.md C): not touched, and it is the stall guard we want to keep. TrueShift's `smooth_stop` raises it to idle+150 under braking, so braking in a crawl goes to neutral by itself (= brake takes the crawl away).
   - hill hold `ga+0x1d00 > 0` makes the update pick neutral itself: creep stays off while it runs.
   Hold/engage is done in the tail hook (below): at rest with free pedals the target is put back to the gear, drive state 4 (R: 0), latch 0; from neutral the gap state is left with `gb_engage` (no throttle needed).
4. **Torque is enough?** [I] The governor can lift the throttle to 50 % to hold idle against the slip; the slipping clutch then transmits roughly `J*a*(w_engine - w_wheel)`. Whether that moves 13-40 t up a grade cannot be settled statically; the controller saturates at floor 0 (closed) and the rpm guard opens it again before the engine is dragged under 0.88 idle. No merged-throttle or input-mix fallback is built (not needed as far as the code shows; neutral-exit-re.md § 2 (a) remains the fallback if the test ride shows too little torque).
5. **Speed at lock-up** (closed clutch, engine at idle) is `idle * ratio`-bound: about 3 km/h in a typical gear 1 (600 rpm, total ratio ~39, r 0.5 m), ~6 in gear 2. `creep_kmh` above that just means "closed".

## Design (modules/creep.cpp)

- Mode on: A-mode automatic, gear 1 / 2 / R (R only when already moving backward), speed < `on_kmh` (8), throttle < 0.8 `max_pedal` (0.30), brake < `brake`, not blocked
  (parking brake, cruise on, engine off, paused, eco-roll `g_rolling`, hill hold). Off: leaving gear set, speed > `off_kmh` (16), throttle >= `max_pedal`, blocked.
- Controller (per frame): target speed `creep_kmh` (3) with free pedals, smoothly up to `manoeuvre_kmh` (8) at `max_pedal`. Floor `f` (0 closed .. 1 open): integrator, below target close by
  `min(close_rate, err)` per second, above target open (or close when the engine rpm > 1.04 idle = the wheels drive the engine), brake > `brake`: open at 4/s, engine rpm < 0.95 idle: stop closing, < 0.88 idle: open at 2/s.
  Starts at 0.9 (nearly open) when the truck is (almost) standing. Handing back (mode off): `f` -> 0 in `release_s` (0.8), i.e. the game's closed clutch returns gradually.
- From rest (`from_rest`, default 1): armed after the truck was driven above 8 km/h with a gear (reset when the engine stops, so no crawl-off at game start), speed < 5, last direction forward, both pedals free for `rest_delay_s` (0.6) -> target gear 1
  (also out of the game's brake-at-rest R). Reverse is never started from rest; it only continues. Brake at rest while we hold gear 1 -> neutral + latch, so the game's own R-on-brake works as before.
- Tail hook: `game_patch_tail(G.tail_site, tail_pre)` is **chained** behind TrueShift's (the helper reads the current jump target, so a second patch wraps the first). Creep's `tail_pre` runs first, TrueShift's second. Needs nothing from TrueShift; TrueShift's stop rule, start gear and shifts keep working (a gear change simply makes the mode ask for the gear again or end).
- Only the 1.60 layout (offsets self+0x100/0x101/0x110, inner+0x142c/0x150). 1.61: off with a log line.

## Patterns and addresses (all unique in .text, checked)

| what | VA | pattern |
|---|---|---|
| drivetrain clutch copy (patched, 12 bytes) | 0x1408c82cf | `8B 83 2C 14 00 00 89 86 30 03 00 00` (with the preceding `48 8B CE`, 0x1408c82cc, and following `E8`, also unique) |
| idle governor | 0x1408cc4f0 | (fn start; callers 0x1408cc213 and 0x1406229f5) |
| engine throttle fn (governor floor, rev limit) | 0x1408cc1e0 | |
| engine-speed update, implicit coupling step | 0x1408c8ea0 | |
| coupling stiffness g(clutch) | 0x1408ca960 | |
| stall/launch flags (`drive+0x33c`, `0x33d`) | 0x1408ca690 | |
| governor gain init (`drive+0x2f8/0x2fc`) | 0x1408c6926.. | |
| automatic rest rule / R-on-brake | 0x14098c6de / 0x14098c7a8 | |
| tail jump (chained hook) | 0x14098cd48 | `49 8B E3 41 5E 5F 5E 5B E9 ?? ?? ?? ??` |

Fields (1.60): `drive+0x2e4` engine rad/s (x 9.5493 = rpm), `+0x2c8` wheel-side speed, `+0x2dc`/`engine+0x228` idle, `+0x330` clutch copy, `+0x33c` launch-zone flag, `+0x33d` stall flag.
Drive state `self+0x110`: 4 forward, 3 neutral after forward, 2 neutral, 1 neutral after reverse, 0 reverse.

## Open points

- That the floor really gives a smooth crawl (torque scale of the clutch constants unknown) and that the controller does not hunt: `creep.csv` (10 Hz: kmh, gear, pedals, floor, target, rpm, idle, game clutch, hold, gearbox state, drive state) and `dazdrive.log` lines `Creep: on/off/holding/engaging/engaged`.
- Engine rpm near idle while crawling (the guard opens at < 0.95 idle); if the rpm column sits at 0.9 idle with floor < 0.2 the governor is too weak for that gear: lower `creep_kmh` or look at the merged-throttle fallback.
- From rest: gear 1 engages without throttle (`Creep: engaged gear 1 without throttle`), no repeated neutral/engage flicker, the truck does not creep off at a light when you only lift the brake (that is intended; use the parking brake or `from_rest=0`).
- Brake at rest returns to the game's R-on-brake; reversing after a stop works as before.
- Launch with throttle > 0.3 from a crawl: no jerk when the floor hands back in 0.8 s (`release_s`).
- Reverse crawl: only continues; the pedal semantics in R are unknown (accelerator drives backward, brake brakes is assumed).
- Sim limits (`tools/sim/sim_creep.cpp`): crude plant, no engine drag, so the downhill case just runs away until the mode ends at 16 km/h; it tests only the logic and the controller's stability on flat/slope.

## For Cruise (nothing required)

Creep does not patch `stub_cruise` or any site another module patches. The only shared point is the tail jump (chained). If the governor torque turns out to be too small, the fallback needs a small merged throttle in `stub_cruise`'s return value while `g_hold`/creep is on (neutral-exit-re.md § 2 (a)); it would need a global in suite.h (e.g. `g_creep_thr`) that Cruise adds before its early returns.

## v2: crawl up to ~7 km/h, brake modulation, gear by weight, added throttle

- Brake: `in.brake = T.in_brake` (the effective brake carries the game's own braking). `target_speed(thr, brake)` lowers the crawl speed linearly with the pedal; `brake` (0.30) removes the crawl.
- Gear by weight (`pick_gear`): the lock-up speed at idle is `idle*0.10472*r/(ratio*diff)` (m/s); crawl gear = lowest gear whose lock-up speed is >= 1.05 x `creep_kmh`, so the clutch slips a little and the engine stays at idle.
  Light (mass < `heavy_t`, default 20 t, `game_mass()` as TrueShift, refreshed 1/s): gears up to `max_gear` (5), clutch only. Heavy or unknown mass: up to `heavy_gear` (2) plus added throttle.
  Mode counts any gear 1..cap (and R) as crawling. At rest the tail hook holds/engages the crawl gear (gear 1 -> 3 is a normal sequencer shift); while rolling it only asks for a higher crawl gear when the current gear is lower, never a downshift (stall rules of the game / TrueShift keep that).
- Added throttle (heavy only): `g_creep_thr` (suite.h, main.cpp) is maxed into `stub_cruise`'s result in cruise.cpp (the wrapper, so every early return incl. cruise off; only values 0 < c < 0.3). The value is stored in `inner+0x1424` (merged throttle, store gates as in neutral-exit-re.md § 1.1) and so reaches the engine; the idle governor `max()`es with it. Controller: +0.1/s while the clutch is closed (floor < 0.1) and 0.15 km/h-ish too slow, -0.3/s otherwise, capped at `heavy_throttle` (0.15), 0 at once with brake >= `brake`, driver throttle >= 0.02, mode off, light load, reverse. Never with cruise on (mode is blocked then). Needs the Cruise module active (its call-site hook).
  Side effect to watch: `inner+0x1424 > 0` also lets the gearbox gap state engage a gear from neutral (harmless, wanted).
- Coexistence: TrueShift hands back below 6 km/h and takes over above 10; between 6-10 km/h Creep may still be on (off at 16 km/h or gear above the cap). TrueShift's stop threshold, start gear and shifts still run; if it shifts out of the crawl range the mode ends ("gear") and re-enters when the speed/gear fit again.
- Sim (`run_creep.sh`): light 9 t flat settles at ~7 km/h in gear 3, brake modulation lowers it, heavy 28 t flat/3 % uphill reaches ~7 km/h in gear 2 with up to 0.15 added throttle; the sim's light 5 % uphill case saturates (no added throttle by design). `run.sh`, `run_cruise.sh` unchanged and running.
- Diagnostics: log `Creep: gear g (crawl gear c, light|heavy)`, csv columns `add_thr, crawl_gear, heavy`.
