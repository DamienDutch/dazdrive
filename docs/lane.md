# Lane module: calm lane centering (experimental)

**Status: experimental.** Off = the game's own lane assist. Everything is in `plugin/dazdrive/modules/lane.cpp`; tests are in `tools/sim/sim_lane.cpp`
(`tools/sim/run_lane.sh`). Related notes: the game's mechanism in `docs/lka.md` and `docs/lka-roll-re.md` (part A), the fitted model and the law's design
in `docs/lka-sysid.md`, the self-learning steering profile and anti-sway in `docs/lka-learn.md`.

## Why a module
With a force-feedback wheel the game's lane keeping does not add the assist to the steering angle: it becomes a position target for the wheel motor
(wheel + A, where A is a leaky integrator of the control law's output u inside the vehicle). A chain of integrators under a P law oscillates, and the game's
gains are fixed floats (2.78x higher with FFB). The module hooks the call to the control law (`stub_law`, 1.60 only) and replaces its output.

## Modes (`[lane] mode`)
- **0** = the game's own law; the plugin only measures it and writes `lane.csv` (o1 lateral offset, o2 heading error, o3 bend term, u, A, steering, wheel at 20 Hz
  while the assist is active).
- **2** (default; 1 is treated as 2) = the model-based law below, fitted on logged drives (`docs/lka-sysid.md`).

## The mode 2 law
Expressed in units of the assist offset A, then converted into the request u the game's integrator needs (`A' = -al A + be u`, `al` and `be` depend on speed):
```
a_cmd = -offset_gain * stiff(o1) * o1 + heading_gain * o2 - bend_assist * o3 + I
u     = (al/be + KA) * a_cmd - KA * A
```
- **Offset** (`offset_gain` 0.009 A per m): stiffer towards the line (`edge_boost`, from `edge_from` 0.3 m to `edge_full` 0.9 m; the strength follows where the truck
  will be `edge_look_s` 1.5 s ahead). A slow integral `I` (`i_gain`, `i_max`) removes a steady offset; it is bled off while the driver steers.
- **Heading** (`heading_gain` 0.55): the damping term. Since `d(o1)/dt = -0.88 v o2`, heading is the lead that the ~0.5 s actuator lag needs. Above 60 km/h the gain
  shrinks with 1/speed (at most to half).
- **Bend feed-forward** (`bend_assist` 2.9): a bend needs a wheel angle of about 4.4-4.6 x o3; the game's law only supplies about a quarter of that, so
  hands off the truck drifts out in bends. The feed-forward looks `bend_lead_s` 0.35 s further ahead than the game's lane term (the game sees ~0.25 s ahead, the wheel
  follows ~0.45 s late) and is scaled by the wheel's measured response.
- **Feedback on A** (`assist_feedback` KA = 40): the game's integrator (time constant 0.26 s) is closed into a fast loop (about 0.1 s), and the request may change
  at most `rate2` 12 per second.
- **Live wheel gain.** The wheel turns by more than the tuning assumes in a sharp bend with a large A (ratio wheel/A up to 3x, a centering band in the FFB wheel). The
  module measures the ratio (for the feed-forward) and the slope (for the feedback) live and scales every A term down (never up) against `wheel_gain_ref` 1.3.
- **Bend hold** (`bend_hold` 0.5, `unwind_s` 0.8): in a bend, while the wheel is heading for less than half of what the bend needs, A is not unwound and is kept at least at
  the level that puts the wheel there; unwinding is rate-limited unless the wheel is past the need, and it fades out when the truck is already more than 0.5-1 m on the inside.
- **Saturation.** A request saturated at +-1 still counts as "the law ran" (otherwise the filter on A would take the assist for off and let the wheel go in the bend).
- **Driver detection.** Wheel motion in the direction A moved 0.33 s ago, up to 3.5x as fast, is the assist's own; motion against it or faster is the driver's. With the
  driver steering the assist backs off (`driver_backoff`, default 1 = no back-off, the assist does not care whether a hand is on the wheel) and leaks A away
  (`driver_leak`).
- **Safety rule.** When the game asks for a firm correction (|u| > 0.3), the output goes the same way and at least half as strong (not in a bend where the game says
  "unwind" while the wheel is under `bend_hold` of the need). The game's indicator clamp (`ga+0x1d0` bits 4/8) is applied to the output, since this law is firmer than the game's.
  Lost lanes are handled as in the game. The assist offset never grows past `assist_max` 0.12.
- **Soft start** (`soft_start` 0.35): after switch-on, or when the lane is found again, only 35 % of the law for the first second, then full strength once the truck has been
  near the center (|o1| < 0.25, |o2| < 0.012) for 0.5 s (or after 20 s), ramped over 2 s. A truck heading for the line (1.5 s ahead beyond 0.8 m) gets full strength at once.
- **Learning and anti-sway.** The gains follow a per-wheel profile (`steer_profile.txt`) and a sway detector cuts them temporarily: `docs/lka-learn.md`.

## Minimum speed (`min_kmh` 20, `keep_on` 1)
- The game's lane keeping stops below 55 km/h. With `keep_on` the patched 55 km/h float (`ln_start`) is 0, so the game never clears lane keeping for low speed: it
  can be switched on standing still. Below `min_kmh` (+1 km/h hysteresis) mode 2 gives no request and A is released; above it the request ramps in over 1 s.
  `keep_on` 0 = the game's behavior (off below `min_kmh`, back on from `on_kmh`). Mode 0 keeps the game's own speed limit.
- Reversing counts as slow (no request).
- In the sim (plant fitted at 30-100 km/h, extrapolated) the law is stable at 5-40 km/h over 18 plant variants, but slow to settle (a 0.5 m step takes 23-42 s).
- Lanes vanish at junctions (the lane-lost path handles it). With `keep_on` the assist also works at junction speeds, so turning without an indicator means steering against it;
  the indicator pause covers turns with the indicator on.
- The era presets from ~2020 set `min_kmh` 20, `keep_on` 1 and `indicator_pause` 1; otherwise the game's own lane assist (55 km/h) stays.

## Indicator
- `indicator_pause` 1: one indicator (game_actor `+0x1d0` bit 4 = left, 8 = right; both = hazards, no pause) pauses the assist. The request decays to 0 (x0.85 per law call),
  the filter on A releases it (x0.94 per step), the short-term law state is reset. The assist comes back after the indicator has been off for 1 s and the truck is near
  the center of a lane (|o1| < 0.9) for 0.5 s, or after 4 s at most, with the request ramping in over 1 s.
- `indicator_off` 1: the indicator also switches the game's own lane keeping off silently (`game_actor+0x1d1` bit `0x20` cleared, atomic; the on/off tone skips states set by
  the module), and the bit is set again 1 s after the indicator went off. If the driver switches lane keeping on during the indicator, that is left alone. A press of the LKA key
  during the indicator means OFF: the bit is cleared again, the falling tone plays and it is not re-enabled afterwards.
- `indicator_resume` 0 (default): lane keeping that went off during a lane change (above `min_kmh` + 2) is switched on again by setting the bit. Off by default because the game
  does not switch lane keeping off for an indicator by itself, so re-setting the flag would mostly undo the driver's own key press.
- A forgotten indicator: after 20 s on (a blinking lamp with < 1 s off phases counts as one period) the assist is back, with the old rule (no steering against the indicator).

## No force feedback (`non_ffb_mode` 0)
- The module recognizes the wheel type from the data, no offsets needed: with an FFB wheel the effective steering (`veh+0x640`) equals the wheel position (`veh+0x1408`); without
  one the game adds A: steering = wheel + A. It takes 60 samples with |A| > 0.005 (about 1 s) and re-checks every 60 samples. A wheel with a learned profile counts as FFB
  from the load on. Until the type is known the game's own law runs (A builds anyway, so detection works).
- Without FFB the game's own law runs (mode 2 and the filter on A are off; the lane-lost fix, the indicator pause and the low-speed gate stay). `non_ffb_mode` 2 forces mode 2 (not tuned).
- Not built: a separate fit for the additive plant (A would be a plain steering-angle offset without wheel lag; a PD on o1/o2 with gains x v would do). The game's own gains
  for that profile are 2.78x lower than with FFB.
- The `ga+0x3f0` / `0x1cb5` reading is not used: `game_actor+0x3f0` is the engine-brake fraction elsewhere (`docs/ats-161.md`).

## Results in the sim
- Crash-bend scenario (a sharp bend at 63-73 km/h, o3 up to 0.018, plant refitted from a drive where the assist saturated): max |o1| 0.40 m (0.31 m with the nominal profile)
  against 0.50 / 1.00 m for the earlier law; the wheel never drops under 50 % of the need; |u| = 1 for 0.1-0.3 s instead of 0.6-1.9 s. Plants with gA <= 2 cannot make this bend at all
  (it needs A above its ceiling of 0.047), the driver must steer there.
- Soft start (`run_lane.sh engage`): switched on calmly at 0.4 m the wheel speed is 0.003 instead of 0.027 rad/s; switched on at -0.9 m and drifting out it is firm as before.
- The suites `run_lane.sh` (`score`, `step`, `stress`, `road`, `low`, `crash`, `release`, `gsweep`, `engage`, `learn`, `sway`, `replay`) cover the plant sweeps; see `docs/lka-sysid.md` and `docs/lka-learn.md`.

## Known limits
- Supported on ETS2 1.60, the layout the hook patterns were verified on. ATS and ETS2 1.61 are not supported.
- Whether clearing or setting the lane-keeping bit alone is enough for the game (the key handler and the arming timer `ga+0x11c` are not touched) is not verified in all cases: the
  dashboard LKA icon and the log line "lane keeping back on" show it.
- Why the learned profile could drift low in the past (gated to >= 50 km/h) is only partly understood; ring and sway events now only count in the small-signal regime (see `docs/lka-learn.md`).
- The strength of the FFB per unit of A is unknown; requests up to |A| 0.03-0.04 occur in bends.

## Logging
- Experimental modules (lane, crawl) show their detail lines one log level lower (`LOG_BOOST`). `lane.csv` columns: `t,kmh,o1,o2,o3,u_game,u_out,A,steer,wheel,activity,scale,ci,damp,ff,
  l_s,l_dfac,l_gA,l_delay,l_c,sway_fac,l_usable,w_ratio,w_slope,lf,li,bend_hold` (`ff` = bend feed-forward in A units, `damp` = the A feedback part, `bend_hold` 1 = wheel under 50 % of
  the need, 2 = A held). Log and csv sizes are capped (`[core] log_max_mb`, `csv_max_mb`; rotated to `.1`).
