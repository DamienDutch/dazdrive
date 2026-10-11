# Lane module: self-learning steering profile and anti-sway

Reference for the learning part of the lane module (mode 2 only). The law itself is in `docs/lane.md` and `docs/lka-sysid.md`.

Code: `plugin/dazdrive/modules/lane.cpp` (learner block above `lane_reset`, `sway_step`, `prof_load` / `prof_save`); the wheel name comes from `input_wheel_name()`
in `core/input.cpp`. Sim: `tools/sim/run_lane.sh learn [kmh] [seconds]`, `run_lane.sh sway [kmh]`, and `sim_lane replay <lane.csv> t0 t1` (open loop on a recorded drive,
with `LEARN=1 SWAY=1 QUIET=1`). With `learn 0` and `anti_sway 0` the law runs on its fixed gains exactly.

## What is learned
The mode 2 law is tuned for a reference wheel. Wheels differ in strength (how far the wheel turns per unit of assist offset A), in delay, and in how strongly the truck turns per
wheel angle. The learner measures these on the road and scales the gains so the loop gain stays at the tuned value:

```
wheel = gA * A(t - delay)          A -> wheel       (gA: strength, delay)
d(o2)/dt + 0.78 v o3 = -c v wheel  wheel -> heading rate   (c: truck response, depends on the wheelbase)
```
Outcome is a profile `(s, dfac, cap)` per wheel: `s` scales all gains (target `gA_ref c_ref / (gA c)`), `dfac` calms the offset and heading terms for a slower wheel, `cap` is a ceiling
set by the ring guard.

## How
1. **Identification** only on usable stretches: lane keeping on, mode 2, speed >= 50 km/h, driver activity < 0.08 (and quiet for 3 s), |o3| < 0.0025 and steady, |o1| < 1 m,
   |u_game| < 0.3, |A| < 0.045, no indicator, no sway episode, 4 s warm-up.
   - A -> wheel: a bank of 19 scalar regressions over delays 0 to 0.9 s in 0.05 s steps (wheel and A high-passed with a 3 s mean). The best residual gives the delay (parabolic
     refinement), its slope gives gA.
   - wheel -> heading rate: `d(o2)/dt + 0.78 v o3`, filtered over 0.3 s, against `-v wheel` (means removed over 10 s) gives c.
   - The sums forget with a time constant of 60 s of usable time. An estimate is only used after 20 s of data with enough excitation and fit quality.
   - **Block gate (anti-sabotage).** Data joins the sums in blocks of 5 s of contiguous usable driving, and only if the block fits well on its own (R2 of the A -> wheel fit >= 0.5,
     c fit >= 0.05, A rms >= 0.0012, gain within 0.4-2.5 x the reference) and agrees with the long-term fit (gain within 35 %, delay within 0.15 s). Ten disagreeing blocks in a
     row (50 s) mean the wheel really changed: the fit starts from scratch.
2. **Profile update.** Every `learn_update_s` (30 s) of usable driving the target `s = gA_ref c_ref / (gA c)` (bounded by `learn_min` 0.7 and `learn_max` 2) and
   `dfac = 1.25 delay_ref / delay` (0.5 to 1; no change up to 25 % more delay than the reference wheel; delay readings count as at most 0.55 s, because a centering band in the
   wheel makes small A read as a long delay) are approached by `learn_rate` (0.4 of the log distance, at most -20 % / +25 % per update, nothing under 3 %).
   Applied in the law: offset gain x s x dfac, heading gain x s x sqrt(dfac), slow integral x s. The bend feed-forward follows the live wheel ratio instead (`docs/lane.md`), and
   towards the line (offset from 0.5 m to `edge_full`) the offset term gets the learned scale back. The A feedback and `rate2` stay fixed: the A actuator is the game's own
   integrator, the same on every wheel; only the wheel behind it differs.
3. **Reference** = what this estimator reads on a calm, recorded, hands-free drive with a direct-drive wheel: gA 0.98, delay 0.35 s, c 0.168. A hand fit of the large-signal behavior
   gives 1.34 / 0.2 / 0.155; the FFB wheel has a dead band, so small signals read weaker and slower. On a wheel like the reference `s` stays close to 1.
4. **Self-check.** Score = offset rms and zero crossings of o1 per minute over the usable driving of an update period. A new profile is a trial: if the next period scores worse than
   the baseline (rms > 1.25 x + 0.02 m, or crossings > 1.5 x + 3/min) it is rolled back and proposals pause for 4 updates.
5. **Ring guard** (A based): the high-passed A with rms >= 0.006 and >= 6 sign changes in 8 s (a ringing of the assist, period around 1.3 s) gives `s` x 0.85 at once, `cap` = the new `s`
   (the learner may not climb above it; the cap relaxes 2 % per update), 10 s cool-down. It only counts in the small-signal regime (|o3| < 0.003 and the live wheel slope not above 1.15 x
   `wheel_gain_ref`) and not with hands on the wheel: a sway in a sharp bend belongs to the live part of the law, and must not push the profile down for good.
6. **Anti-sway** (`anti_sway` 1, `sway_cut` 0.65, `sway_recover_s` 25, `sway_amp` 0.15): a zig-zag detector on o1 (an extremum is confirmed after a reversal of 0.075 m). Three consecutive
   extrema, each swing >= 0.225 m peak to peak, half periods 0.4-1.6 s, the second swing >= 85 % of the first (not dying out). Then all mode 2 gains are multiplied by `sway_cut`
   (offset x f, heading x (0.5 + 0.5 f), at most down to 0.4), held for 5 s and given back linearly over `sway_recover_s`. Repeated hits compound (0.65, 0.42, 0.4). It is skipped while an
   indicator is set. Slow wander with half periods over 1.6 s is too little gain, not a sway, and does not count.
   The learner reads a hit as "too much gain" (s x 0.9, never below 0.8, `cap` = s, proposals pause for 3 updates) only if the driver was not steering in the last 8 s and the regime
   is small-signal; it never learns or proposes while a sway episode is active.
7. **Safety.** The safety rule of the law is untouched. All learned values are bounded; a profile file with a value out of bounds or garbage means fixed gains.

## Where it is stored, how to reset
- **File:** `plugins\dazdrive\steer_profile.txt`, one line per wheel (`wheel=<winmm name of the device with the most buttons> v=1 s= dfac= cap= ga= c= delay= n=`), under 1 KB
  (at most 16 lines). It is loaded when the wheel is found (else as "default" after 45 s), written at most every `learn_save_s` (240 s) when something changed and at shutdown.
  A write error is logged once and ignored. A wheel with a stored profile counts as an FFB wheel from the load on (still re-checked from the data).
- **Reset:** delete `steer_profile.txt` (or the wheel's line); learning starts again from the fixed gains. `learn 0` switches the learner off, `anti_sway 0` the sway detector.
- **ini keys:** `learn` 1, `learn_rate` 0.4, `learn_min` 0.7, `learn_max` 2, `learn_update_s` 30, `learn_save_s` 240, and the sway keys above.

## Log and csv
- Log level 1: profile loaded / new / saved / rolled back, ring guard, sway detected, "the wheel loop has changed for good". Level 2: the estimates and the score of every update.
  Look for the lines starting `Lane learn:` and `Lane: sway detected`.
- `lane.csv` end columns: `l_s, l_dfac, l_gA, l_delay, l_c, sway_fac, l_usable`. `l_s` should stay near 0.9-1.1 on a wheel like the reference; `sway_fac` < 1 only in real sways.

## What the sim shows
- Hands off at 90 km/h on a wandering road (900 s): the estimates converge (gA within 1 % of the plant, delay +0.03 s, c about 8 % low because of a closed-loop bias that the reference
  contains too). A plant that reads like the reference wheel keeps s 1.00. Weaker or slower plants get a higher or calmer profile: gA x0.6 rms 0.39 -> 0.23 m, a slow wheel
  (delay 0.35 + lag 0.15) goes from a limit cycle (31 crossings/min) to 0.26 m.
- A stronger wheel (gA x1.8) costs tracking accuracy on purpose (rms 0.07 -> 0.16 m, s 0.59): the fixed law tracks tighter there but sits at 1.9x its stability limit.
- A driver weaving for 120 s with a hand on the wheel barely moves the profile (the block gate throws the blocks away). A broken profile file means the fixed gains.
- Anti-sway: a strong plant (gA x3.5) rings 0.91 m, 0.35 m after the first hit (2.8 s); a delay-0.5 s plant that diverges (4 m) ends at 0.4 m. Normal bend entries and exits at 60 and 90 km/h
  give no hits. A replay of a recorded drive detects both of its ringing episodes and no hit in the other ~950 s.
- The sway hit needs three extrema (2-5 s after the sway builds up); the A-based ring guard is the faster one on a hard ring.
- The reference comes from one wheel and one truck. A different truck changes c (wheelbase), and `s` follows it.
