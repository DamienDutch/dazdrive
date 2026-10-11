# PCC: predictive cruise speed band, dip and crest

PCC = predictive cruise control, the industry name (DAF PCC, Mercedes PPC; Volvo calls theirs I-See).

Design: a band around the set speed that is asymmetric, at most -5 km/h under the set speed and rather above it. The truck carries momentum into a
climb, may run over the set speed in a dip, and does not downshift just before a top it can reach. Margins depend on mass. There is no learned
topography: everything comes from the route in memory (heights every 25 m, `core/route.cpp`).

## Cruise: band over the set speed (`[cruise] band_up_kmh` = 6, needs predictive = 1 + navigation)
- `band_up_kmh` is a cap, not a fixed offset. TrueShift works out `g_pcc_need_kmh` (`climb_need_kmh`, 2 Hz): the extra speed at the foot of the next climb
  (within 1200 m, grade > 1 %) so that the truck gets over its top at full throttle in the highest plateau gear without dropping more than 1 km/h under the set speed
  (energy over 25 m steps, 0.5 km/h search up to 8). Cruise uses min(`band_up_kmh`, need) for the momentum target and for the dip; a dip only when need >= 0.5.
- Without a TrueShift answer (NAN) the band scales with mass instead: band_up_kmh x (0.4 + 0.6 k), k = (mass - 12 t) / 28 t clamped 0..1 (2.4 km/h solo, 6 at 40 t).
- **Momentum into a climb:** the predictive target is set + band.
- **Dip** (at most `band_max_s` = 40 s over the set speed, like Mercedes PPC): on a descent (descent logic active, >= 2 %) with a valley ahead
  (`route_valley_m(max(300, v*30))`: lowest point, road rises >= 3 m within 300 m after it) the game's smart-overspeed cvar is set to the band instead of 0, so
  the engine brake / retarder only hold above set + band. Latched to the bottom; dropped when the descent goes on with no climb in sight, or when the descent is over.
  Log: "Cruise: dip ahead (bottom in N m): up to +X km/h"; after the cap: "dip margin used for N s: back to the set speed".
- **Traffic:** a vehicle within about 4 s ahead (radar lead < max(80 m, 4 v)) switches momentum and dip off: they would only be braked away by the ACC
  (real PCC: ACC distance first). This is not a safety issue, the ACC target always wins.

## TrueShift: band under the set speed (`[trueshift] crest_band_kmh` = 5, needs lookahead + navigation for the top part)
- Band under the set speed = crest_band_kmh x (0.6 + 0.4 k), x 0.6 in Power (3 km/h solo, 5 at 40 t).
- `top_reachable`: top within max(400, v*25) m (no false flat: no > 1 % climb right after it); energy
  v_end^2 = v^2 + 2 d (F_full - R) / m with F at the lower of v and the floor; floor = max(ref - band,
  speed at plateau - 150 rpm (min idle + 200)). ref = set speed (cruise) or the speed when the top came in view.
  Reachable -> no downshift ("top reachable", which also blocks the "climb ahead" pre-downshift). Log once per top:
  "top in N m: holding G at X km/h (gets over above Y km/h)".
- "over the top": 4 s grace after the last reachable estimate (the last ~50 m are not seen as a climb).
- "cruise band" (also without a route): cruise on, own pedal < 0.1, no kickdown, v >= set - band, rpm > floor and
  the gear still pulls > 0.06 m/s2 on this grade -> no demand downshift; the speed comes back slowly.
- Diagnostics: the reasons "cruise band" / "over the top" appear in `trueshift.csv`.

## Sim (tools/sim/run.sh, T-block, band 0 vs 5; other scenarios unchanged)
| scenario (40 t, cruise 80, route) | shifts 0 -> 5 | min km/h |
|---|---|---|
| T0 150 m at 3 % | 4 -> 0 | 75 -> 76 |
| T0b 400 m at 1.5 % | 2 -> 0 | 78 -> 78 |
| T6 rolling hills +-1.5 % | 9 -> 0 | 78 -> 78 |
| T6c hills +-2.5 % per km | 28 -> 3 | 76 -> 75 |
| T1-T3 400 m 3 %, 300 m 5 %, 2 km 4 % | unchanged (still downshift) | unchanged |

A 40 t truck loses ~12 km/h on 400 m of 3 % in 12th, so the band only covers short humps; longer climbs downshift as before.

Needed extra speed (`need`, set 80, foot of the climb 500 m ahead): 8 t and 12 t need 0 on every climb (400 m 1.5 %, 400 m 3 %, 300 m 5 %, 800 m 3 %, 2 km 4 %);
26 t: 0 / 2.5 / 8 / 6 / 8; 40 t: 0 / 8 / 8 / 8 / 8 (capped at 6 by `band_up_kmh`). The sim runs 40 s before measuring because the force bias (`S.f_bias`)
needs about 20 s to settle.

Dip detection was tested offline with `route_valley_m` on 8 height profiles. Not simulated: the game's smart-overspeed braking (`sim_cruise` has no cvars).
