# Lane assist: system identification and the mode 2 law

Code: `plugin/dazdrive/modules/lane.cpp` (mode 2, the default), sim: `tools/sim/sim_lane.cpp` (`run_lane.sh`). Data: `lane.csv` logs of real drives (20 Hz; o1 lateral offset,
o2 heading error, o3 bend term, u request, A assist offset, wheel). Everything below is fitted on those logs and checked in the sim; the fit scripts were throw-away
(numpy), the numbers are here. The module overview is `docs/lane.md`, the learning on top of this law `docs/lka-learn.md`.

## Model (fitted)

```
u -> A:      A'  = -al A + be u          al 2.6+0.016 v (3.1 at 45 km/h, 4.0 at 88), be 0.235-0.001 v   (v in km/h)
A -> wheel:  wheel = gA A(t - delay) + hands     hands-free: gA 1.34, delay 0.2 s, first-order lag ~0  (hands-on lumped fit: gA 0.8-1.0, tau 0.23-0.45 s)
wheel -> o2: o2' = -c v wheel - d v o3          c 0.155 +- 0.01 (hands-free; 0.15-0.2 overall), d 0.8-0.85  (v in m/s);  d/c = 4.4-4.6 = wheel the bend needs per o3
o2 -> o1:    o1' = -kk v o2                 kk 0.85-0.92, no measurable delay (best shift -0.03 s)
```
- The game's law is exactly `u = clamp(v(0.075 o2 - 0.00275 o1 - 0.5 o3))` (v in km/h): 100 % of the samples in the clean drives (a drive with indicator clamps and lane changes
  differs in ~15 %; those were left out of the plant fits).
- **Fit quality.** (a) u -> A: R2 0.92-0.998 per run; held-out R2 0.96 (rms 0.0014 vs A std 0.007). (b) A -> wheel (+ bend term): R2 0.90-0.97 per run, held-out 0.84;
  hands-free output-error fits per block R2 0.75-0.98. (c) o2': R2 0.77 held-out (the hands add unmodeled input); o1' from o2: R2 0.94 held-out. Lumped A -> o2' (no wheel): gain
  c*gA = 0.17-0.27 with 0.3 s total lag, consistent with the chain. Actuator at 88 km/h: al 3.84, be 0.144 (model 4.0 / 0.146).
- **Plant is amplitude dependent.** The FFB wheel has a dead band / friction: the static wheel is not exactly 0 when A = 0 (offsets up to +-0.012 rad held for seconds), and the wheel
  answers weakly to small A and about 1.5x stronger to large swings (gA 1.8-2.5 at delay 0.15-0.2, c 0.21 in a ringing episode). In a sharp bend the ratio wheel/A rises to 2-3x.
- **Bend**: `wheel = -4.4 ... -4.6 o3` is the steering a bend needs (speed independent: -4.8 at 35-55 km/h ... -4.0 at 85+). Regressing wheel on A and o3 for hands-on bends: A coefficient
  0.6-1.2, o3 coefficient -3 (the hands) + the game's own feed-forward through A (-0.5 v o3 gives ~-1.2 o3 of wheel at 80 km/h): hands-on bend steering is ~75 % driver, ~25 % game
  feed-forward. **Hands off, the game's law supplies only a quarter of the bend steering**; the rest has to come from an offset (0.9 m at o3 0.003): that is the "drifts out in bends".
- Actuator authority: |u| = 1 gives A = be/al = 0.036-0.04 (a ceiling around 0.047 in practice), i.e. wheel ~0.04 and a bend of o3 ~ 0.008 at most. Typical A: p99 0.02-0.027, max 0.049.
- Sampling: `lane.csv` is gated at 0.05 s of wall time, so rows are 0.05-0.067 s apart; the fits resample to 20 Hz.

## Where the oscillation comes from (the game's law)
- A logged hands-on drive: o1 rms 0.29 m, 12.6 half-cycles/min (+-0.1 m), with a clear sway at 0.30 Hz (3.1-3.3 s period, +-0.5 m) in one stretch.
- Model poles of the game's law plus a small center term (0.15), nominal plant: 40 km/h 12 s period zeta 0.15; **80 km/h 5.0 s, zeta 0.08; 90 km/h 4.5 s, zeta 0.06**. With plant gain x1.5 the
  period is 4.3 s, x2-x3 gives 3.6-2.8 s, so the observed 3.3 s fits a plant 2x stronger than nominal (a loosely held wheel).
- Phase budget at ~2 rad/s: game integrator 1/al = 0.26 s (-27 deg) + wheel lag 0.3-0.45 s (-30...-42 deg) = 60-70 deg lost, and the game's PD zero (K1 kk v / K2 = 1.7 rad/s) leaves
  only ~50 deg of lead there. Phase margin about 0. **The heading term of the game's law is too small (0.075 v vs the offset 0.00275 v) and the actuator lags too much.** Gain margin of the
  game's law: x2.4 at 80 km/h, x1.9 at 90 (a stiffer wheel hand already tips it).
- A damper acting on the wheel "works" in a sim with the wrong lag structure; in reality the wheel contains the driver and the bend. The law therefore uses only A (our own state, safe)
  and o2 (clean: noise 0.0003-0.0009).

## Mode 2 law
In units of the assist offset A, then converted to a request:
```
a_cmd = -offset_gain * stiff(o1) * o1 + heading_gain * o2 - bend_assist * o3 + I          (I: i_gain per m*s, |I| <= i_max, bled off while you steer)
u     = (al/be + KA) * a_cmd - KA * A                                                         (A' = -al A + be u  ->  A follows a_cmd with pole al + be KA ~ 10 /s)
```
- Defaults: `offset_gain` 0.009 A/m (game 0.015 at 80), `heading_gain` 0.55 (game 0.24; x clamp(60/km/h, 0.5, 1), so 0.37 at 90), `assist_feedback` KA 40, `bend_assist` 2.9 (the bend needs
  4.4-4.6: 65 %), `edge_boost` 1 (twice as stiff at 0.9 m, from 0.3), `i_gain` 0.001 / `i_max` 0.004, `rate2` 12/s. Safety rule and lane-lost handling as in the game; the game's indicator clamp
  (`ga+0x1d0` bits 4/8) is applied to the output, since this law is much firmer than the game's.
- Zero of the PD: 0.47 rad/s (game 1.7) -> ~70 deg lead at crossover (1.3 rad/s) against ~30 deg lag: phase margin ~40-45 deg nominal.
- Gain margin (sim, 90 km/h, hands off, first parameter set): stable up to plant gain x3 at tau 0.6 s, x4 at 0.3 s, x2 at 0.9 s (the game's law is unstable at x3 / tau 0.6 nominal).
  Also fine at a law frame time of 0.017 / 0.033 / 0.05 s (`DT=0.05 sim_lane ...`).
- `lane.csv` has the column `ff` (bend feed-forward in A units); `damp` holds the A feedback part (-KA*A).

## Why the gains are what they are
The first parameter set (offset 0.012, heading 0.5, rate2 6) was designed on the hands-on fit. Hands-free drives at 90 km/h on a motorway showed a ringing the hands-on data could not:
two episodes with a period of **1.33-1.4 s** (4.6 rad/s): A +-0.015, u +-0.9, o2 +-0.045, o1 +-0.4 m, triggered when a bend ends (o3 0.0051 -> 0 in 1 s while the wheel still holds -0.02), and a smaller one
sustained inside a steady bend. With a delay plant the loop is a heading integrator behind a 0.2 s delay and the A pole: phase at 4.6 rad/s = -90 (integrator) -53 (delay) -25 (A pole), so the
margin is the gain, and the plant is stronger at large swings (gA 1.8-2.5).

The sim reproduces it with the hands-free plant (plant c 0.19, tau 0.08, delay 0.2; env vars `PL_C PL_GA PL_TAU PL_DELAY` in `sim_lane.cpp`; `road` replay, hands off; rms o1 in m / zero
crossings per min):

| gA | set 1 (offset .012, head .5, rate2 6) | set 2 (head x60/kmh, rate2 4) | set 3 = current (offset .009, head .55, rate2 12) |
|---|---|---|---|
| 1.0 | 0.14 / 2.0 | 0.15 / 3.5 | 0.17 / 2.0 |
| 1.34 | 0.05 / 0.7 | 0.06 / 3.4 | 0.07 / 1.4 |
| 1.8 | 0.05 / 1.0 | 0.05 / 2.0 | 0.06 / 1.4 |
| 2.5 | **0.6 / 44-50 ring** | 0.11 and **5.9 diverging** (2 roads) | 0.12 / 1.0 |
| 3.5 | 1.9 / 42 | 9.1 / 22 | 0.55 / 67 (bounded, saturated) |

- A slower `rate2` (4/s) makes u slew-limited when it swings +-0.9 and adds lag, so it did **not** help. What helped: a lower offset gain (a less lag-heavy outer loop) with the heading gain kept;
  lowering the heading gain alone destroys the damping (0.2: limit cycle everywhere, 0.3: ring at gA 2.5). Stability margin of the current law: up to about x1.9 of the hands-free fit (gA 2.5), against x1.4-1.7 before.
- Side effects of the current set on the nominal plant: a step from 0.5 m settles in 9.5 s at 80 km/h (3.9 s before), hands-off bend steady offset 0.29 m (0.25), straight rms 0.12 (0.11); the worst-case grid
  still has 0 diverged cases at 40/60 km/h, 3/18 at 80/90 (the weakest plant, an authority limit). If it feels lazy to come back to the middle: `offset_gain` 0.011.
- If it still rings at 90: a lower `rate2` is no remedy; the next steps would be a phase lead from the delayed wheel (feed A a predicted o2 = o2 + 0.2 s * o2') or a describing-function gain limit on |A| > 0.01.
- Hands-free bends o3 0.003-0.005 stayed within o1 +-0.4 m with `bend_assist` 2.9, so the 65 % feed-forward share stays; the plant says 4.6 o3 is needed.
- Hands-free performance of the first law: o1 sd 0.05-0.14 m per block (sim promised rms 0.08-0.10), A p99 0.027; straight roads and gentle bends were good.

## Sim (tools/sim/sim_lane.cpp, plant = the fitted model; `run_lane.sh [score|step|stress|road|replay ...]`)
First parameter set against the game's law, nominal hands-on plant, hands off (1 min, wandering road o3 sd 0.0012, start 0.5 m): rms o1 game 0.24-0.29 m / mode 2 **0.08-0.10**; max 0.6-0.7 / **0.15-0.24**;
half-cycles/min 9-13 / **1.9**. Bend o3 0.0035 (R~300) at 60-80 km/h hands off: max offset 1.0 / **0.37 m**, steady 0.43 / **0.25 m** (hands on, the driver steering the bend too: game -0.04, mode 2 -0.24 m = cuts the corner
a bit, the hands add to the feed-forward). Step 0.5 m: the game's law rings (overshoot 0.34 m, never settles, diverges at 80-90 km/h for c x1.5 tau 0.45); mode 2 overshoot <= 0.08 m, within 5 cm after 2.5-4 s (80-90 km/h),
4-5 s (60), 14 s (40, slow by design). Worst case over c x0.6-1.5, tau 0.2-0.45, gA 0.7-1: game's law rms up to 4.3 m (80 km/h), mode 2 0.37 m; only the weakest plant (c x0.6 and gA 0.7) cannot follow a 0.0035 bend at
80-90 km/h (authority limit), neither can the game's law.
**Closed loop on recorded roads** (`road`, recorded o3(t), v(t)): hands off rms 0.44 -> 0.16 m, max 1.18 -> 0.36; another drive 0.52 -> 0.16 (max 2.7 -> 1.3); another 0.40 -> 0.17; hands on 0.23-0.34 -> 0.15-0.22.
Plausibility of the plant: the game's law with a hands-on driver gives rms 0.28 / 21 half-cycles/min on one recorded road; the real drive had 0.29 / 12.6.
Evasive 1.2 m: the same assist opposing the driver as the game's law (0.0047 vs 0.0048 mean A at 80 km/h), overshoot after letting go 0.11 vs 0.09 m.
**Near-crash replay** (`replay <lane.csv>`, open loop, recorded inputs): of 152 rows with |u_game| > 0.3 none has an output of the opposite sign or under half the size (also 0 in six other drives); in the
drift to 1.3 m the new output is -0.8...-1.0, the game's -0.3...-1.0. Closed loop on that bend: hands off max offset 4.1 -> 1.9 m (the bend o3 0.0165 is at the actuator's limit), hands on 0.37 -> 0.42 m.

## Uncertainty
- The hands-on fits have the hands inside the loop; the hands-free fits (about 445 s of hands-free driving, 14 quiet blocks) fixed gA, delay and c, but only on one wheel and one truck. The sim sweeps plant gain x0.6-1.5 (stress to x3) and
  tau 0.2-0.9 s; the law holds there. The learner (`docs/lka-learn.md`) adapts the gains to other wheels.
- The 3.3 s sway of the game's law is only reproduced with a plant ~2x stronger than the fit; period and damping of the game's law are reliable, the exact frequency is not.
- The frame rate of the law calls in the game is not logged (the sim is robust at 17-50 ms). The A feedback needs `A` readable (`veh+0x1420`); without it the law falls back to leak compensation only.
- Force feel: |A| up to 0.03-0.04 is requested in bends; the FFB strength per unit of A is unknown.

## Tuning knobs (ini, no rebuild)
Sways slowly -> lower `heading_gain` (0.35) or `offset_gain`; sluggish towards the center -> `offset_gain` 0.011-0.016; drifts outward in bends hands off -> `bend_assist` 3.5-4.4; pulls inward in bends with the hands on -> 2.0.
