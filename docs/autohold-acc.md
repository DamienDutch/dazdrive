# Auto Hold, lead speed and own ACC

Implemented in `plugin/dazdrive/modules/cruise.cpp`, tested in `tools/sim/sim_cruise.cpp` (`tools/sim/run_cruise.sh`).
Background: `docs/cruise-re.md` § 3, `docs/neutral-exit-re.md` § 3 (brake request `ga+0x17c`).

## 1. Auto Hold

Ini (cruise section): `auto_hold` 1, `auto_hold_request` 0.6, `auto_hold_gentle_s` 0.5, `auto_hold_hard_s` 0.15, `auto_hold_max_s` 2.5.
`hill_start` stays as before but only runs with `auto_hold 0` (Auto Hold supersedes it).

**Behavior**
- Stopped with the brake (speed < 0.5 km/h, driver brake > 0.1): the hold comes on (request ramps to 0.6 in ~0.2 s, while the pedal is still down),
  any grade, no timer. Dashboard letter `B` (`g_dash_status`, the same as hill start).
- Off / never on: crawl switched on (`g_crawl_on`: crawl owns the standstill), parking brake, reverse (`T.gear` or `T.dgear` < 0), engine off, Stop&Go holding
  (`g_sg_hold`). A hold that is on when one of these appears fades out in ~0.2 s.
- Driver brake again while holding: stays held. Gas (> 0.05): launch (below). Pedal lifted again before the truck moves: holds again.
- Everything goes through the one writer `brake_request()` (the lesson of the `+0x17c` latch: a value written there stays until the writer clears it): what we wrote goes back to 0, hazards stay muted, the request
  never ends non-zero (sim: `request at end` = 0 in every case, also with the game's AEB option off).

**The release (what "do it better" means here)**
Real cars (Crafter, Lynk & Co) drop the hold at a fixed moment; pulling away hard then lurches or rolls. Two things make ours different:
1. **The gas reaches the engine while we hold.** The game zeroes the merged throttle while `ga+0x17c > 0` (neutral-exit-re § 3.1). One 7-byte compare is
   patched (`comiss xmm7,[rbx+0x17c]` at `0x1405c8583`, found with the unique pattern of the throttle store gates, `game_patch_cf0_if` with the flag
   `s_thr_pass`): the compare is skipped while **only Auto Hold** asks (not for curve brake, hill start, creep brake or the game's own AEB, which are bigger/other
   requests). So the box engages as for a normal launch (the neutral-exit gate needs `inner+0x1424 > 0`), the clutch closes, torque builds against the brake.
2. **The request follows the drive force**: `r = hold x (1 - F_drive / F_need)`.
   `F_drive` = engine torque (`gb_engine` curve at the real rpm x effective throttle) x gear ratio x diff x 0.9 x (1 - clutch open) / wheel radius;
   `F_need` = 1.1 x grade force + 0.25 m/s2 x mass (mass from the game, grade from the SDK pitch). On an uphill the brake therefore stays until the engine can
   hold the truck. The fall is also limited by the pedal: gentle (pedal ~0.25) `auto_hold_gentle_s` 0.5 s for the full hold, hard (0.8+) `auto_hold_hard_s` 0.15 s
   (the game slews its brake at 5/s anyway, so 0.12 s is the floor); in between linear. `r` only ever goes down, except:
   - **roll back** (speed < -0.08 m/s on an uphill): grab again (+4/s), and the drive estimate is trusted less (`s_ah_k` x0.7, min 0.3; +0.1 after each clean launch).
   - after `auto_hold_max_s` (2.5 s) of launch the hold lets go at least at the hard rate (it never fights the pedal for long).
   - the launch ends (state off) only when the truck really goes (> 2 km/h), so a roll back right after the request hit 0 is still caught.
- Without the patch (pattern not found, log says so) the throttle is dead while we hold; the release is then paced by the pedal only (still ramped).
- Bug found by the sim: when the hold ended, the soft-release filter of `brake_request()` dragged the last 0.02 out for 2 ticks and the gate closed again (a
  dead throttle for 30 ms). Fixed: a hold ends directly (`hold_was`).

**Sim results** (26 t, 2400 N·m, gear 1; stop with 0.5 brake, release, 2.5 s later gas; `launch jerk` = max d(accel)/dt, 0.1 s smoothing)

| case | roll back | launch jerk | old hill start (roll back / jerk) |
|---|---|---|---|
| 0 % gentle / hard | 0 / 0 m | 3.8 / 13.9 | (no hold; reference without any hold: 3.4 / 12.3) |
| 5 % gentle / hard | 0 / 0 m | 3.1 / 12.8 | 0.021 m / 4.1; 0.004 m / 12.0 |
| 10 % gentle / hard | 0 / 0 m | 2.3 / 11.9 | 0.119 m / 6.4; 0.018 m / 11.8 |

Also tested: no throttle gate patch; mass or drive estimate 1.5x wrong; engine really 75 % of the estimate (no roll back); engine 45 % (pedal too light: the guard grabs
and learns, 3.6 m -> 0.3 m roll); the four blockers (request stays 0). The jerk of a hard launch is the clutch bite of the sim; the hold adds < 1.7 m/s3.

**Things to check when driving** (`cruise.csv` and the log): "Auto Hold: holding at x%", "launch", "released after ... rolled back ... m"; does the box engage while
we hold (gear and `eff_clutch` in the csv)? Does `eff_thr` follow the pedal during the hold? How hard does the 0.6 request really brake (a measured 5.9 m/s2 at
request 0.3 is not proven)? If the truck lurches on release, lower `auto_hold_hard_s` or raise it for a gentler fall.

## 2. Lead speed (A1) and own ACC

**Hook.** The ACC fn `0x1409958b0` calls the radar `0x140998ce0(game_actor, out)` at `0x1409959ea` (`E8` rel32; right after it `movups xmm0,[rax]; mov eax,[rax+0x10]`).
Pattern (unique, checked against .text): `48 8D 54 24 50 48 8B CB C6 83 21 01 00 00 01 E8 ?? ?? ?? ?? 0F 10 00 8B 40 10 89 44 24 48` (call at +15).
The call is redirected to `radar_stub` (via `game_patch_call`), which calls the original and copies the result: **`out[0]` gap (m), `out[1]` lead speed (m/s, absolute),
`out[2]` FLT_MAX, `out[3]` 0, `out[4]` 0** (the fn initializes `out` that way at `0x140998d4c..d5b`; only `out[0..1]` are used by the ACC: `[rsp+0x38]` gap,
`[rsp+0x3c]` lead speed, `T = vlead*3.6 + ...`). The radar fn takes only (rcx, rdx), no xmm arguments (prologue read). The log says "lead speed hook in at ...".
The data counts as valid while it is < 0.3 s old, the game itself accepts the lead (`+0x1a8 > 0.5`: gap < 100 m, no indicator) and the gaps agree within 3 m + 10 %.
Otherwise everything falls back to the old behavior (d(gap)/dt filter, the game's target).

**(a) Calm cut-in** (`acc_calm_cut_in`): the opening speed is now `lead speed - own speed` (instant, no 0.5 s of filter history needed); d(gap)/dt is the fallback.
The calm rule (opening > 0.3 m/s, gap > 8 m) is also written back into `+0x1a0` when own ACC is on, so the game's overspeed braking (retarder, engine brake,
service brake read `+0x1a0`) does not brake for a cut-in either; before, the calm rule only held the throttle. Sim: cut-in at 86 km/h, 30 m ahead, we 80:
the game's target brakes 80 -> 71 km/h (peak 1.45 m/s2), own: no braking at all.

**(b) Own ACC target** (`own_acc`, default **1**):
`target = lead speed + own_acc_gain x (gap - wanted gap)`, wanted gap = `own_acc_stop_m` (6 m) + time gap x speed (the same time gaps as before: the player's
short/middle/long distance, mode factors, max `acc_max_m`), clamped to [0, set speed]; a lead faster than the set speed gives the set speed (like the game).
The target is slewed: a lead that appears starts from our speed; it falls at `own_acc_decel` (1.2 m/s2), faster when closing in needs more than 0.5 m/s2
(`a_req = closing^2 / (2 (gap - 6 m))`, up to 8x); it rises at `own_acc_accel` (0.8 m/s2). The result is written into `ga+0x1a0` every tick **before**
the game's controller reads it (in `stub_cruise`), so the stock PI, our smooth PI and the game's overspeed braking all follow it. The game's emergency braking
(gap < half of its D(v), `+0x1a8`) is untouched: it stays the safety net, and `cr_frame` still sets `g_acc` so D(v) stays our time gap.
Not used when the player set ACC off, when the radar data is missing or inconsistent, or with `own_acc 0` (the game's target as before).

*Why default on:* the game's target is `vlead + (1..10 km/h per meter) x gap error` with an exponential factor, so any gap error of a few meters swings the target
by tens of km/h (the earlier "calm" patches only treated the symptom), and it does not know the lead's acceleration. Ours is linear with a time constant of 5 s
(gain 0.2), uses the lead speed (a braking lead lowers the target at once), and is rate limited. Sim (26 t, 80 km/h, scripted lead):

| scenario | game target | own ACC |
|---|---|---|
| far lead 70 m, 70 km/h | min gap 38.6 m, peak decel 1.45, jerk 4.4 | min gap 44.6 m, peak decel 1.20, jerk 1.75 |
| cut-in slower (60 km/h, 28 m) | min gap 12.2 m, peak 3.20, min speed 43 | min gap 12.2 m, peak 3.21, min speed 53 (no over-braking) |
| cut-in faster (86 km/h, 30 m) | brakes to 71 km/h | no braking |
| lead brakes 3 m/s2 (80 -> 30, 50 m) | min gap 7.8 m, peak 2.23 | min gap 7.5 m, peak 2.48 (the lead out-brakes the truck; the net does the rest) |

The sim's braking model is rough (retarder 0.7 + service 0.6 m/s2, emergency up to 6), so these are about direction, not numbers. Risk: the lead speed is read from a
static analysis ([V] in the code: `T = vlead*3.6 + ...`). It was checked on a logged drive against d(gap)/dt over 0.5 s: slope 0.96, correlation 0.99, rms difference
0.65 m/s, bias +0.3 m/s (the derivative lags), so the hook delivers good lead speeds. If a wrong value ever did occur, the consistency check would usually reject it
(speed outside 0..70 m/s), but a plausible wrong value would steer the ACC: then set `own_acc 0`.

## 3. cruise.csv (ini `trace` 1)

10 Hz while cruise is on, or Auto Hold / any brake request of ours is active. Columns: `t` (wall s), `kmh` (signed), `set`, `gap` (game `+0x1a8`), `radar_gap`
(hook), `lead_kmh` (nan = none/invalid), `opening_ms` (lead speed - ours), `acc_game` (the game's target before ours), `acc_own` (nan = not used), `acc_used`
(`+0x1a0` after), `wanted_gap`, `thr` (cruise output), `req_ours` (what `brake_request` wrote), `req_ga` (`+0x17c` as read), `ah_state` (0 off, 1 hold, 2 launch),
`ah_r` (Auto Hold request), `in_thr`, `in_brake`, `gear`, `eff_clutch`, `rpm`, `grade` (%), `air_psi`; further columns for the AEB, bend speed and lateral acceleration. Analysis ideas: lead_kmh against the speed of a lead you follow at a steady
speed; `acc_game` vs `acc_own` at cut-ins; the Auto Hold stops: `ah_r`, `eff_clutch`, `kmh` around the launch.

## 4. Code map

`auto_hold()`, `ah_drive_force()`, `ah_need()` (cruise.cpp, after `hill_start`); `brake_request()` takes `s_want_hold` (direct, no filters) and sets `s_thr_pass`;
`radar_stub`, `lead_speed`, `lead_rel_speed`, `desired_gap`, `own_acc`, `cruise_trace` (before `cr_start`); patches in `cr_start` (throttle gate, hazard mute now also for
`auto_hold`, radar call). Sim: `ah_run` (signed speed, static brake, clutch bite after the throttle gate) and `acc_run` in `sim_cruise.cpp`.
