// TrueShift: makes the game's automatic shift like a real truck AMT.
//
// How it hooks in (ETS2 1.60, see trueshift notes):
//  - decide: the window call in the automatic's update, right before the game
//    decides whether to look for another gear. We run our own decision there;
//    if we want another gear we set the update's force flag, so the game calls
//    its chooser now.
//  - choose: both calls of the game's gear chooser. We call the original (it
//    clears the force flag) and return our gear instead.
//  - start: every call of the start-gear fn. We keep the game's pick (it already
//    depends on load) and only lower it when the launch would take too long.
// Below the takeover speed, in reverse and in neutral the game decides as before.
//
// Modes follow the game's adaptive-shift keybind (cvar g_adaptive_shift):
// Eco (10), Normal (3), Power (1.67) and the game's "off" slot (0) = M, assisted
// manual. The dashboard's "A" becomes D/E/S/M.
#include <algorithm>
#include "../core/suite.h"
#include "../core/game.h"
#include "../core/sense.h"
#include "../core/act.h"
#include "../core/route.h"
#include "../core/vehicle.h"
#include "../core/speedtarget.h"
#include "../core/vclass.h"
#include "../core/shiftmodel.h"
#include "../core/shiftcommon.h"
#include "../core/shifthook.h"
#include "../core/gearscore.h"
#include "../core/rpmspan.h"
#include "../core/bands.h"
#include "../core/powertrain.h"
#include "../core/gearchoice.h"
#include "../core/wish.h"
#include "../core/driver.h"
#include "../core/gearreq.h"
#include "../core/demand.h"
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------- settings

float g_rpm_by_engine;   // [trueshift] rpm_by_engine (core/rpmspan.h; LowSpeed reads it too)

static float p_eco_ceil, p_mode_char, p_enabled, p_takeover, p_own_low, p_on_kmh, p_off_kmh, p_on_rpm, p_hint_hold, p_min_gap, p_dwell_ud, p_dwell_du,
    p_max_skip, p_skip_acc, p_hyst, p_kick_pedal, p_kick_rate, p_power_from, p_power_full,
    p_cruise_rpm, p_reserve, p_learn_rpm, p_learn_res, p_learn_km, p_eco_rpm, p_power_rpm, p_eco_reserve, p_power_reserve, p_lug_res, p_lug_res_eco, p_lug_res_pw, p_cc_damp,
    p_eb_down, p_eb_light, p_eb_skip, p_m_style, p_m_lug, p_letter, p_start_fix, p_launch_max, p_slip_s, p_launch_torque, p_launch_rpm,
    p_idle, p_trace, p_lookahead, p_roll, p_roll_cruise, p_roll_under, p_roll_under_eco, p_roll_under_pw, p_roll_boost, p_load_from, p_launch_fix, p_ret_down, p_air_psi, p_air_rpm, p_no_creep, p_stop, p_tones, p_crest_band, p_ch_flip, p_pw_frac, p_pw_frac_eco, p_pw_frac_power, p_pw_calm_thr, p_kw, p_kr_eco, p_kr_norm, p_kr_pw, p_kl_eco, p_kl_norm, p_kl_pw,
    p_tc, p_tc_stall, p_tc_lock, p_band_weight, p_resp_eco, p_resp, p_resp_pw, p_fuel_eco, p_fuel, p_fuel_pw, p_tf_eco, p_tf, p_tf_pw;

static param g_params[] = {
    { "enabled", &p_enabled, 1, "1 = TrueShift on, 0 = the game's own automatic" },
    { "takeover", &p_takeover, 1, "1 = TrueShift picks the gears, 0 = only log what it would do" },
    { "own_low_speed", &p_own_low, 1, "1 = TrueShift owns every shift of the automatic modes from the standstill on (there is no takeover speed); 0 = the game's own automatic picks the gear in the low-speed range as before, TrueShift takes over from takeover_kmh / takeover_rpm and hands back below handback_kmh (a way back, and what the sims call 'today')" },
    { "takeover_kmh", &p_on_kmh, 7, "hand-over (M mode, and with own_low_speed = 0): the speed from which TrueShift takes the gearbox from the game" },
    { "takeover_rpm", &p_on_rpm, 1000, "hand-over (M mode, and with own_low_speed = 0): TrueShift takes the gearbox from the game as soon as the engine has reached this rpm with the clutch closed in any gear (0 = by speed only, takeover_kmh)" },
    { "handback_kmh", &p_off_kmh, 4, "hand-over (M mode, and with own_low_speed = 0): and hands it back below this speed (the game picks the start gear and the stop's downshifts there)" },
    { "eco_rev_ceiling_rpm", &p_eco_ceil, 200, "Eco: an upshift never waits for revs above the top of the efficient range plus this (rpm), also with the pedal down; a kickdown lifts it. 0 = off. Saves fuel on long pulls" },
    { "mode_character", &p_mode_char, 1, "1 = Eco and Power differ from Normal in the small drivetrain details too, with their own fixed values: kickdown only at a floored pedal in Eco (0.97) and easier in Power (0.85) instead of kickdown_pedal; Power holds a gear 4 s after a downshift and 8 s after a kickdown, Eco 3 s, instead of dwell_down_up_s and kick_dwell_s; a roll must last Eco 12 / Normal 15 / Power 20 s; hint hold, engine brake and pedal filter per mode. 0 = only the main differences, the ini values for all" },
    { "hint_hold_s", &p_hint_hold, 10, "seconds a gear chosen with the shift-hint buttons is held (0 = off)" },
    { "min_gap_s", &p_min_gap, 0.5f, "minimum time between two shifts" },
    { "dwell_up_down_s", &p_dwell_ud, 3.0f, "after an upshift, no downshift for this long (unless kickdown or lugging)" },
    { "dwell_down_up_s", &p_dwell_du, 2.0f, "after a downshift, no upshift for this long (unless clearly past the upshift point)" },
    { "max_skip", &p_max_skip, 2, "most gears an upshift may skip (0 = never skip)" },
    { "skip_accel", &p_skip_acc, 0.15f, "acceleration (m/s2) the skipped-to gear must still give" },
    { "choice_flip_s", &p_ch_flip, 5.6f, "kickdown_by_wish: within this many seconds after an upshift only a shortage of kick_ratio + 0.3 may trigger the kickdown (0 = any shortage)" },
    { "rpm_by_engine", &g_rpm_by_engine, 0, "1 = the fixed rpm margins of the shift rules (the stall margin, the lowest landing rpm, the comfort floors, the stop and hand-off margins, the engine-brake light stage, the air-charge and M-lug revs) follow the engine's own span from idle to the torque plateau and its brake range, instead of numbers fitted on engines with idle 550-600 and a plateau at 900-1000 rpm. On those engines nothing changes. 0 = the fixed numbers" },
    { "power_target_frac", &p_pw_frac, 1.0f, "At a firm pedal the revs aim for the lowest rpm where the engine still gives this share of its peak power (torque times rpm, from its torque curve), not for the top of the power range: a flat power curve is not chased up to its peak for a few percent. The upshift point at a firm pedal is the rpm that lands the next gear on that rpm, the kickdown target follows it, and so does the top of each gear's window (the window only while the truck is free to accelerate; a truck that struggles up a climb keeps its gear). Never above the engine's own power range. Lower = fewer revs and a little slower acceleration. 1 = at the peak power, 0 = off (the power range of the engine's definition)" },
    { "power_target_frac_eco", &p_pw_frac_eco, 0.90f, "Eco: power_target_frac for this mode. 0 = the same as power_target_frac" },
    { "power_target_frac_power", &p_pw_frac_power, 1.0f, "Power: power_target_frac for this mode. 0 = the same as power_target_frac" },
    { "power_calm_throttle", &p_pw_calm_thr, 0.6f, "Power: the calm stretch (Power's own shift points sinking to Normal's) needs the throttle under this, steady (acceleration under 0.2 m/s2), on the flat (grade under 1.5 %) for 8 s. A heavy truck holding its cruise needs more throttle than a light one" },
    { "launch_correct", &p_launch_fix, 1, "1 = when the game picked a lower start gear than the physics of launch_max_kmh / launch_slip_s allow (after reversing, after the engine started, or its own downshift at a stop), the gear is changed at the standstill before the pull-away: no climbing through the low gears (0 = off)" },
    { "load_down_from", &p_load_from, 0.75f, "on a climb: a gear that needs this share of its full-throttle force to hold the speed (and the speed falls under real throttle) gets its downshift point raised towards the torque plateau (1 = off; a gear that cannot hold it at all downshifts at once anyway)" },
    { "kickdown_by_wish", &p_kw, 1, "With the torque pedal ([pedal] wish): the kickdown is the driver's dissatisfaction, not a flat pedal. The ratio of the force the pedal wishes to the force this gear gives at full throttle now is held (the gear stays as long as it is 1 or less: the wish pedal just opens the throttle further); over kick_ratio the excess adds up, and past kick_level the gear that gets the speed soonest is taken, whatever the dwell times say. A fast press halves what is left; right after an upshift only a large shortage counts; a flat pedal with the ratio over kick_ratio kicks down at once. 0 = the kickdown rule of kickdown_pedal and kickdown_rate, as without the wish pedal" },
    { "kick_ratio_eco", &p_kr_eco, 1.05f, "kickdown_by_wish, Eco: the wished force over the force this gear gives at full throttle, above which the dissatisfaction adds up (1 = the gear just gives the wish; the margin is for the error of the vehicle model)" },
    { "kick_ratio", &p_kr_norm, 1.05f, "kickdown_by_wish, Normal: the same" },
    { "kick_ratio_power", &p_kr_pw, 1.05f, "kickdown_by_wish, Power: the same" },
    { "kick_level_eco", &p_kl_eco, 0.15f, "kickdown_by_wish, Eco: the dissatisfaction (excess over kick_ratio, summed over seconds) at which the kickdown comes. The default is 3 s at a 10 % shortage and 0.4 s at 40 %; the pedal pressed fast halves what is left" },
    { "kick_level", &p_kl_norm, 0.05f, "kickdown_by_wish, Normal: the same (default: 1 s at a 10 % shortage, 0.14 s at 40 %)" },
    { "kick_level_power", &p_kl_pw, 0.015f, "kickdown_by_wish, Power: the same (default: 0.3 s at a 10 % shortage, 0.04 s at 40 %)" },
    { "hysteresis_rpm", &p_hyst, 50, "an upshift must land at least this far above the next downshift point" },
    { "kickdown_pedal", &p_kick_pedal, 0.92f, "pedal position for kickdown" },
    { "kickdown_rate", &p_kick_rate, 1.5f, "pedal speed (per second) that makes a full press a kickdown" },
    { "power_from", &p_power_from, 0.65f, "pedal (0-1) where the shift points start moving to the power range" },
    { "power_full", &p_power_full, 1.0f, "pedal where the shift points reach the power range" },
    { "cruise_rpm", &p_cruise_rpm, 0, "Normal: an early upshift lands at least this far above the start of the torque plateau" },
    { "reserve", &p_reserve, 1.3f, "Normal: the force reserve. With the torque pedal on, an upshift may land low (down to the smooth-running limit, under the torque plateau) while the new gear gives the wished force times this at full throttle (1.3 = up to ~75 % of its force). With choice 0 also the reserve an early upshift needs" },
    { "learn_rpm", &p_learn_rpm, 100, "after an upshift without pull: lowest landing rpm +this on that kind of grade (fades with distance)" },
    { "learn_reserve", &p_learn_res, 0.1f, "and the force reserve +this" },
    { "learn_km", &p_learn_km, 3, "half of what was learned is forgotten after this many km (faster when upshifts hold)" },
    { "eco_rpm", &p_eco_rpm, -75, "Eco: shift points and cruise rpm moved by this much" },
    { "power_rpm", &p_power_rpm, 150, "Power: shift points and cruise rpm moved by this much" },
    { "eco_reserve", &p_eco_reserve, 1.2f, "Eco: the same force reserve (lower = lands lower)" },
    { "lug_max_pct_eco", &p_lug_res_eco, 125, "Eco with the torque pedal, pulling away: the wish may be up to this % of the force the landing gear gives at full throttle (125 = the pedal is at full and the truck pulls a little softer than wished, at lower revs). It holds while the wish is mostly acceleration (the resistance under 1 - 100/this of the wish) and is spent where holding the speed takes more of it; what a lugging gear costs on top is the expected time it loses to the demands still to come (the road ahead, a driver who lifts). 100 = off" },
    { "lug_max_pct", &p_lug_res, 112.5f, "Normal: the same" },
    { "lug_max_pct_power", &p_lug_res_pw, 107.5f, "Power: the same (a hard launch: one more gear skipped with a moment of lugging beats one more shift, which drives nothing)" },
    { "power_reserve", &p_power_reserve, 1.5f, "Power: the same force reserve (higher = keeps more pull in the new gear)" },
    { "cruise_damp_s", &p_cc_damp, 4, "cruise control throttle is smoothed over this many seconds before it counts as demand" },
    { "ebrake_downshift", &p_eb_down, -1, "engine brake downshifts of the engine brake the driver does NOT hold as a lever (the pedal's automatic engine brake, a stage the brake channel holds): -1 = full when the engine's def asks for it (engine_brake_downshift), else by level; 0 = always by level; 1 = always full. The driver's own engine brake lever and retarder lever follow the step's rpm band instead (owner decision 70)" },
    { "ebrake_skip", &p_eb_skip, 1, "1 = engine brake / retarder downshifts go two gears at a time when that still fits the brake range (half the shifts, half the game's throttle blips)" },
    { "ebrake_light_rpm", &p_eb_light, 1100, "rpm the lightest engine brake stage keeps where it is not the driver's lever (the pedal's automatic engine brake, a stage the brake channel holds; the full stage keeps the engine's brake range). The driver's lever follows the step's rpm band instead (owner decision 70)" },
    { "m_style", &p_m_style, 0, "M mode: 0 = European (like Volvo, Mercedes): every shift is yours, also at the rev limiter; a warning tone when the engine goes over its limit (the wheels push it, e.g. downhill). 1 = protected (like Eaton): also one gear up at the rev limiter and one down before the engine stalls. 2 = assisted (like a VW DSG): also kickdown, downshifts when braking to a stop and one gear down when lugging" },
    { "m_lug_rpm", &p_m_lug, 1000, "M mode with m_style 2: one gear down below this rpm with a lot of pedal and no pull" },
    { "dash_letter", &p_letter, 1, "1 = the dashboard shows the mode (D, E, S, M) instead of A" },
    { "start_gear_fix", &p_start_fix, 1, "1 = lower the game's start gear when the launch would be too slow" },
    { "launch_max_kmh", &p_launch_max, 14, "a start gear above the game's pick is allowed when the clutch would be closed by this speed (km/h at launch_rpm) and the slip time stays within launch_slip_s: on a light or downhill start the truck no longer revs a low gear to the limiter. Not in Power or M. 0 = never above the game's pick (the game picks by a number in the truck definition: 4 solo, 1 with a trailer, whatever the gear ratios are). With this on, a full pedal does not demand a quicker clutch than a normal one" },
    { "launch_slip_s", &p_slip_s, 1.0f, "longest clutch-slip time accepted for a start gear" },
    { "launch_torque", &p_launch_torque, 0.6f, "share of max torque the clutch passes while starting" },
    { "launch_rpm", &p_launch_rpm, 1000, "rpm at which the clutch closes when starting" },
    { "torque_converter", &p_tc, 0, "0 = detect (a box whose rpm sits above the driveline's below the lockup is a converter), 1 = this truck is a torque-converter automatic (creep and launch are the converter's crawl, the clutch is never driven, the shift points and the force model account for the slip and the multiplication), 2 = never a converter" },
    { "tc_stall", &p_tc_stall, 0, "torque converter stall torque ratio (0 = read from the game, else learned from the launches; a figure over 1 overrides both)" },
    { "tc_lock_gear", &p_tc_lock, 0, "first gear the converter locks in (0 = learn it from the ride, or never locks)" },
    // the vehicle model's resistance (core/vehicle.cpp); the keys stay here so every ini keeps working
    { "rolling_resistance", &V.crr, 0.006f, "rolling resistance coefficient" },
    { "drag_area", &V.cda, 6.0f, "air drag Cd*A in m2 for trucks (vehicles under 4.5 t use 1.0: cars, pickups)" },
    { "efficiency", &V.eta, 0.9f, "driveline efficiency" },
    { "idle_rpm", &p_idle, 550, "engine idle rpm to start with (learned while standing still)" },
    { "lookahead", &p_lookahead, 0, "1 = use the navigation route ahead: the grade of the road in front of the truck (climbs, tops, descents) in the gear choice, the descent control and the eco-roll's forecast, the top of a climb for the cruise, and the planner's neutral requests. 0 = the box knows only what the truck itself tells (pedal, revs, load, the grade under the truck, its own acceleration), as without a route; predictive cruise (PCC) comes back as a layer on top of that (the default). (No junction, bend or limit is read either way: the gearbox acts on demands, not on route geometry.)" },
    { "crest_band_kmh", &p_crest_band, 5, "with the route (predictive cruise, PCC): no downshift for a top ahead that this gear gets over at full throttle while losing at most this much below the set speed (without cruise: below the speed when the top came into view) and staying on the torque plateau. Full at 40 t, 60 % solo, 60 % in Power; 0 = off" },
    { "eco_roll_under_kmh", &p_roll_under, 3.0f, "Normal mode, with cruise control: the roll ends when the speed is about to drop this far below the set speed (km/h; Volvo I-Roll: 1-3) (Written for a set speed of 70 km/h; it scales with the set speed.)" },
    { "eco_roll_under_eco_kmh", &p_roll_under_eco, 4.0f, "the same in Eco: a little more room, the most relaxed (Written for a set speed of 70 km/h; it scales with the set speed.)" },
    { "eco_roll_under_power_kmh", &p_roll_under_pw, 1.5f, "the same in Power: the strictest (keep it between 1 and 2 km/h; values around 0.5 give very short rolls that hunt) (Written for a set speed of 70 km/h; it scales with the set speed.)" },
    { "eco_roll_boost_kmh", &p_roll_boost, 1.0f, "with cruise control, Eco and Normal: just before an eco-roll starts, the cruise control lets the speed come this far over the set speed (km/h), so the roll takes the speed down to a little under it and the average stays near the set speed (0 = roll from the set speed) (Written for a set speed of 70 km/h; it scales with the set speed.)" },
    { "eco_roll", &p_roll, 1, "1 = roll in neutral with the pedal released on gentle stretches (Eco, Normal and Power; not in M; never with any brake)" },
    { "eco_roll_cruise", &p_roll_cruise, 1, "1 = roll in neutral also while the cruise control is on, even with eco_roll 0. Needs SteadyCruise, predictive cruise (PCC), lookahead and a route; 0 = only with the cruise off" },
    { "tones", &p_tones, 1, "1 = a short tone when the mode changes (Eco low, Normal middle, Power rising, M two short)" },
    { "retarder_downshift", &p_ret_down, 1, "1 (the default since K3 round 2) = the retarder lever also gets its step's rpm band like the engine brake (owner decision 70: every step gives an allowed range, the box moves up in it while the speed keeps rising and down when it is under control; step 1 never shifts for a tap); 0 = only the engine brake does" },
    { "air_charge_psi", &p_air_psi, 100, "brake air below this (psi): keep the rpm up so the engine-driven compressor refills it ([core] air_from_truck: 35 psi over the truck's own warning pressure instead); 0 = off" },
    { "air_charge_rpm", &p_air_rpm, 1200, "the rpm kept while charging (no upshift below it)" },
    { "no_creep", &p_no_creep, 1, "1 = with crawl switched off the truck never creeps on the clutch at idle: pedals free below 8 km/h opens the clutch" },
    { "smooth_stop", &p_stop, 1, "1 = when braking to a stop the clutch opens around idle +150 rpm (more when braking harder) instead of pushing to ~2 km/h" },
    { "trace", &p_trace, 1, "1 = write trueshift.csv (10 per second) for analysis" },
    { "band_weight", &p_band_weight, 0.5f, "how strongly the mode's band wish counts in the gear choice (layer 3's band cost is the intent's per-band cost x this, in seconds; 0.5 = the band rows (0..3) weigh 0..1.5 s, in line with a shift (~0.3-1 s) and the acceleration time lost (~0-3 s); 0 = revs do not matter, only time and lugging)" },
    // The mode's response weight (WP-E): the one knob of a mode's character besides the band taste. 0.6 / 1.0 / 1.6 are the plan's start values (G tunes them).
    { "response_weight_eco", &p_resp_eco, 0.6f, "Eco: how many seconds of expected delay a lower rpm is worth (the weight of the expected-time term: 1 = a second is a second; under 1 the mode accepts a slower response for lower revs, over 1 it buys response with revs). It also sets where the band wish moves to the power band: at a demand of 1 / this times what the eco band can deliver" },
    { "response_weight", &p_resp, 1.0f, "Normal: the same" },
    { "response_weight_power", &p_resp_pw, 1.6f, "Power: the same" },
    // The fuel term (WP-E, owner 02-11): seconds one litre is worth to the mode. E started them at 7 / 2 / 0.5 from the round-2 Pareto points of the PULLS (Eco gave 1.55 % time for
    // 0.29 % fuel, Normal 0.36 % for 0.18 %, Power about nothing; dearer than that the term costs time on the 70 km/h pulls). G1 tuned them on the fleet against refactor 53ac7f8 and powerband
    // f342c12 (g1_tune.py, 16 pairs spanning the fleet; the bar: time within +0.5 % per scenario row, fuel not worse, fewer shifts and hunts, Eco not higher in rpm). The pedal-driven fleet rows
    // (steady, ramp, cruise) pay litres for revs and have time to spare, and fuel and rpm keep falling to 100 s/L and beyond (Normal 100: fuel +1.5 -> +0.1 %, rpm +4.3 -> -1.6 %, the cruise climb
    // and the known climb of part 18 close; Power 100: +1.3 -> +0.1 %; Eco 100: +2.3 -> +0.6 %, rpm +8 -> +1.8 %, the overtake row +0.5 -> +2.5 %). But the acceptance rows stop the weights early:
    // run_pcc's desc5 (a 60 t descent, traced: at Normal 50 the flat approach holds 50 km/h in 11 where 2 holds it in 10, and the descent ends in 9, pegged at 57.7 km/h, where it ends in 8 at 52 .. 54:
    // 435 units over the allowed speed against 3; Power 60 the same; Eco fails this row at the shipped 7 already and run_pcc checks Normal only), and part 18i: a shift in a 0.3 s blip (Power 10 .. 40 and Eco 50, until
    // the row compared the blip with the same pull without a lift: that shift was the pull's own, decided as the pedal came back), none after a held lift (Eco 100), a shift in a 0.8 s lift (Normal 30), a shift inside a roundabout (Normal 40), the demand-fed calm lift opens (Normal 10, Eco 40 and 50), a second
    // crawl-landing line (Normal 20); run_pcc's time over its 90 runs is not worse only up to Eco 15 (20: +0.003 %, 30: +0.016 %). The largest values that pass every row as P6b leaves them:
    // Eco 15, Normal 5 (2 .. 5), Power 5 (the blip rows count a shift only if the same pull makes it without the lift: Power then passes to 40; 5 is the largest where run_pcc is not
    // worse in time and no part 18i row opens; Power 40 closes the Power cruise climb and takes run_pcc fuel -0.7 % for +0.007 % time, one crest row +2.9 s). The weights are pinned by the
    // pedal-off pricing and the descent, not by the bar (docs/powerband-change.md, "G: the fleet").
    // WP-G1b: the weights by the owner's currencies (Eco diesel, Power time, Normal the mix; nothing wasted), searched on the 16-pair subset of the merged tree and checked on the fleet
    // (docs/powerband-change.md 'G1b'). Normal stays at 5: from 6 on run_pcc's hard-braking lead (flat, 40 t, 85 km/h) is run into (min gap 2.1 .. 2.5 m, the brake channel of BR),
    // and 20 opens two 18i foot rows of the lug lift. Eco 15 -> 18 (3 x Normal's exchange rate, fuel_weight / response_weight: Eco 30, Normal 5, Power 3.1 s per litre per second of
    // response; 30 opens the 18i calm-blip row of Eco, 18 is the largest tested that passes every row): fuel -1.3 % against Normal for time +0.26 % (the overtake row +1.8 %).
    // Power = Normal's: a lower Power weight buys no time on this fleet (time over Normal +0.03 % at 5 and at 2.67) and costs fuel (+0.19 % over Normal at 5, +0.26 % at 2.67), so
    // Power spends no more fuel weight than Normal and no less.
    { "fuel_weight_eco", &p_fuel_eco, 18.0f, "Eco: how many seconds one litre of fuel is worth in the gear choice (the gear's litres over the time it is expected to be held, less the cheapest gear's, times this; the game cuts the fuel in gear with the pedal off). 0 = the fuel is not priced, only the band wish stands for it" },
    { "fuel_weight", &p_fuel, 5.0f, "Normal: the same" },
    { "fuel_weight_power", &p_fuel_pw, 5.0f, "Power: the same" },
    { "target_full_eco", &p_tf_eco, 0.5f, "Eco: the pedal is an rpm target (calm revs at a light pedal up to this mode's full-pedal point, then P3 at the end of the driver's pedal). The full-pedal point is this share of the way from the end of the plateau (P2) to the power peak (P3): 0.5 = the middle, ~1600 rpm on a typical truck" },
    { "target_full", &p_tf, 0.75f, "Normal: the same, three quarters of the way" },
    { "target_full_power", &p_tf_pw, 1.0f, "Power: the same, the power peak" },
};

// ---------------------------------------------------------------- state

static const char *const mode_name[] = { "Normal", "Eco", "Power", "M" };
static const char mode_letter[] = { 'D', 'E', 'P', 'M' };  // P = Power (Damien 15-11; was S for Sport since 25-10)
static const char *mode_label(int m) { return mode_name[m]; }
static char mode_dash(int m) { return mode_letter[m]; }

static int g_active;    // TrueShift has the gearbox: on in the ini and the shared hooks (core/shifthook.cpp) are in
static int s_tail_ok;   // the update's tail hook (eco-roll, the neutral watchdog) is in place
// a neutral request of the longitudinal planner (ts_neutral_request; see "neutral on request" below)
static const double NQ_TTL = 0.3;   // a request lives this long without a refresh (the planner calls every frame)
static struct {
    double at;          // sim time of the last call (-1e9: none)
    float hold_s;       // the planner's "about this much more" at that call
    int want;           // 1 = coast in neutral, 0 = stay in gear
    int roll;           // the roll of now is the request's (it ends when the request runs out)
    const char *no;     // the reason the request of now is refused; NULL = granted, nothing asked, or nothing to say
} s_nq = { -1e9, 0, 0, 0, NULL };

// Upshift hunting guard, learned per grade class: after an upshift without
// pull the lowest landing rpm and the force reserve go up; both fade with the
// distance driven, faster while upshifts hold. Classes: < -1 %, -1..1, 1..2.5,
// 2.5..4, >= 4 %. A lesson on a grade also counts for the steeper classes.
enum { N_CLASS = 5 };
struct lesson { float rpm, res; };
static lesson g_learn[N_CLASS];
static float g_learn_mass;

static struct : shift_common_state {
    void *self;
    int out_of_gear;        // the telemetry showed no gear (neutral at a stop, the gap of a shift) since note_gear_change last saw the box in a gear: the gear it comes in with is the box's own launch, no hint (R0611)
    int gear_before;        // the gear the last shift left (note_gear_change): a kickdown inside the dwell pays the shift back to it twice
    int sh_braked;          // the last shift was made for braking (the engine brake, the braking rules, the deceleration assist): its situation is the braking, and ends with it (sc_reversal_weight)
    float tgt_kd;                         // and how far into the kickdown the foot was
    float tgt_rpm;                        // the operating point of the last layers decision (the rev-limit rule's upshift lands near it)
    float foot_prev;                      // the raw foot at the last decision (is it still rising?)
    float foot_f; double foot_t;          // the foot the target follows: up at once, down over the dwell (gear_layers)
    int was_kick; double kick_at;         // the driver's foot was at the start of the kickdown (his pedal's end less a step) the last time the layers asked, and since when (the rising edge of a press)
    float wk_d;             // kickdown_by_wish: the dissatisfaction (excess of the wish over the gear, summed over time)
    int lug_back; double lug_back_at;   // the gear the lugging rule left for a priced sag (0 = none) and when: the way back (decide_gear_rules) takes the box back to it when the road eases
    float lug_ratio;        // the lugging the landing reserve allows now (wish over a gear's full-throttle force; 1 = none): landing_reserve
    float wk_r;             // and the ratio of now (wished force over the force this gear gives at full throttle); wk_on: the wish pedal gives the throttle this frame
    int wk_on, wk_fast;     // wk_fast: a fast press has already halved the level
    double wk_until;        // the kickdown is due (set by the trigger, ends with the shift, 3 s, or the pedal lifting)
    double wk_retry;        // no new trigger before this time (no gear could give the wish)
    double wk_pt; float wk_pp;   // the pedal 0.05 s or more ago, for its speed
    const char *last_why;   // reason of the last shift we made
    int up_gear;            // last upshift we made, to judge whether it held
    int up_tainted;         // that upshift landed in braking or a lift-off: whatever goes wrong with it is no lesson
    double up_at;
    float up_grade, up_kmh;
    float up_pred;          // throttle the new gear was expected to need
    float up_ped;           // the driver's pedal at that upshift
    float strain;           // throttle above that prediction while not gaining speed, integrated
    float crest_ref;        // speed when the top ahead came into view (0 = no top in view)
    int crest_said;         // logged "top reachable" for this top
    double crest_seen_at;   // last time a top was in view
    double top_at;          // last time a top ahead was reachable in this gear (the last ~50 m to it are not seen as a climb)
    float top_floor;        // and the lowest speed (m/s) that was allowed for it
    float up_pt, dn_pt, floor_rpm, eb_level;
    int eb_full;            // the game itself would shift into the engine-brake range now
    // WP-K3 (decision 70): the DRIVER's lever as a fraction of its steps, kept apart from eb_level (which the other rules read as the one strength of every kind of engine brake / retarder)
    float lv_ret;           // a retarder step held, step / steps (0 = none, or retarder_downshift off)
    float lv_eb;            // the engine brake held by the driver (the SDK flag), stage / stages (0.5 where the game does not say)
    int lb_on, lb_k, lb_gear, lb_left_g;   // the band rule ran at the last decision; its step; the gear it held; the gear a downshift of its own left last (0 = none)
    double lb_at;                          // when it last ran (it is not asked while a shift's gap is open: the lever is the same one if it comes back within a shift and the dwell)
    float lb_vmin, lb_left_e;              // the lowest speed since the last shift or the lever's change (m/s); the model's optimism where that gear was left (m/s2)
    float lb_cal;                          // what the truck measures of the lever against the model's force (1 = as modelled; 0 = not measured yet)
    double lb_pull;                        // when the lever was pulled or changed step
    float lb_aus, lb_afr, lb_beta; int lb_dec, lb_beta_n;   // the deceleration of a lever decision (the lever's, the free one), what the truck did across a gap between them (0 = the free one, 1 = the lever's; learned), and whether the decision in flight was the band rule's
    int launch_want;        // a start gear the game did not pick (launch_correct); the chooser hands it back until it is in or the truck rolls
    double launch_at;
    int boost_want;         // eco-roll would start but wants the cruise control to build the roll_boost speed first
    float roll_kmh;         // speed when the roll started
    // what the roll started with (WP-ER1, owner decision 67): who starts a roll ends it. A roll the box started without cruise (the driver's) lasts until the driver does something: the cruise
    // was off then (roll_cc), the mode was this one (roll_mode). (A +/- press in neutral is not read yet: tail_pre.)
    int roll_cc, roll_mode;
    int roll_prot;          // the roll was ended by the runaway protection: the exit gear is one for the engine brake range (roll_exit_pick)
    // the braking downshift rules as decide_gear last looked at them (ts_brake_down_eta reads them): the rule's rpm threshold, the
    // seconds of speed the shift loses before the rule judges (rule 1: the engine brake keep point, 2: braking, 3: the deceleration assist's cap)
    double bd_at, bd_after, ad_at, ad_after;   // when the rule looked, and not before
    int bd_gear, ad_gear;
    float bd_floor, bd_lead, ad_cap, ad_lead;
    // the dashboard's view (ts_view, the dev display): read-only, nothing here steers
    int adv_dir, advice;    // M: the direction of the gear the automatic would take now (-1, 0, 1), and the gear shown (once that direction held 1 s)
    double adv_since;
    int tk_why, tk_from, tk_to;   // the ticker: the reason shown (why_e) and its shift (tk_from 0 = a hold in tk_to)
    double tk_at;
    int hold_cand;          // a hold reason waiting to be shown (it must last 0.5 s)
    double hold_since;
    double hold_age_t0;     // when the hold of the speed began: the last time the wish was mostly a pull (hold_share under a half) with the foot on the pedal (the age of the situation, not of the gear)
    // what a shift really costs (the layers' landing estimate): the decision (when, speed, gears, the resistance then) and what was learned from the
    // shifts that followed: seconds from the decision to the gear being in, and the deceleration beyond the resistance
    double dec_at; float dec_v, dec_fres; int dec_from, dec_for;
    float sl_dur, sl_dec; int sl_n;
    double ws_t; float ws;   // gear_layers: the wish filtered over a few shift times (for the need), and when it was last updated
    // gear_layers, the pedal off: the demand still expected (ex_pre: the largest wish of the pedal on, fading over the plan horizon), when it was last asked for
    // (wish_on_t: the lift clock) and the lateral load's steering gain (a regression of the measured lateral acceleration on v^2 x steering: sums with the demand
    // memory's forgetting, and when they were last updated)
    float ex_pre, ex_v; double ex_t, wish_on_t; int lift_done, lift_pick;   // (ex_v: the speed of that moment; lift_pick: the gear the layers asked for in this lift; lift_done: that shift was made, this lift has had its decision)
    float lat_sxx, lat_sxy, lat_syy, lat_w; double lat_t;
} S;

static const float CC_FALL_S = 0.5f;   // our own cruise throttle: the filters follow it this fast (s) when it falls
enum { ROLL = GR_NEUTRAL };   // decide_gear: "put it in neutral" (eco-roll); the arbiter's value for it (core/gearreq.h)

// Not an automatic (manual, H-shifter): the game never calls the automatic's code, we stay quiet.
static int manual_box()
{
    return T.shifter[0] && strcmp(T.shifter, "automatic") && strcmp(T.shifter, "arcade");
}
// Unsupported truck (electric, 1-2 gears, rpm that does not follow the speed): when unsure, stay off and say so once.
static void unsupported(const char *why)
{
    if (S.unsupported) return;
    S.unsupported = why;
    g_truck_unsupported = why;
    S.rolling = 0;
    S.roll_exit = 0;
    g_rolling = 0;
    logline("TrueShift: this truck is left to the game's own automatic (%s)", why);
}
// Everything we change in the game's decisions goes through this.
static int takeover() { return p_takeover > 0 && !S.unsupported && !manual_box(); }

static FILE *g_trace;
static double g_trace_at;

static float grade_pct() { return V.grade_pct; }
// There is no takeover speed any more (owner decision 60, 04-11: "take over everything from the game instead of 7 km/h / 1000 rpm, since we already do the clutch at launch and pick the start
// gear"): the three ini keys of it are ignored. Two places used the same numbers for something else and keep them as what they are, not as a hand-over:
//  - K_MOVING_KMH: the speed above which a gap state or a roll's exit is a truck in motion and not a standstill (the open clutch and the preselected gear at rest are LowSpeed's, the
//    "stuck in neutral" engage must not fight them): the value the handback had, 4 km/h (a walking pace; under it the wheel speed sensor of a truck is noise);
//  - K_ADVICE_MIN_KMH: the speed under which the M advice and the skip stay in the start gears (advice only, nothing shifts on it).
static const float K_MOVING_KMH = 4.0f, K_ADVICE_MIN_KMH = 7.0f;
// The idle as the tach shows it standing still (learned below, the ini's until then): right for our own rpm rules (stall,
// floors, lug). What the game's code compares with is its engine figure (+ its idle add): Creep reads that, and the stop
// threshold hook gets it as its argument. The two agree once learned on a warm engine, not before, so they stay apart.
static float idle_rpm() { return S.idle > 300 ? S.idle : p_idle; }
static float ioff(float n) { return rs_off(idle_rpm(), n); }   // a fixed offset from idle / the plateau, by the engine's span when rpm_by_engine is on (core/rpmspan.h)

static int n_gears()
{
    int n = T.fwd_gears;
    return n > 0 && n < 32 ? n : 0;
}

// The vehicle model's forces (core/vehicle.cpp), the short names the decisions below are written in: the engine rpm in gear g
// at speed v (m/s), tractive force (N) in g at v with a share of the torque, the resistance (N) on the grade of now or
// on another grade (%). rpm_at is the ENGINE's rpm (the shift points, floors and lug checks compare it): on a torque converter
// it is the stall speed while the converter slips and the driveline's once it is coupled; on a locked box the two are the same,
// so nothing changes there.
static float rpm_at(int g, float v) { return vehicle_engine_rpm_at(g, v); }
// The two lines of the protections (decide_gear_rules, decide_manual) and of the driver's press (hint_refusal): a gear above rev_line is one the rev-limit rule shifts up
// out of at once, a locked gear under stall_line one the stall rule shifts down out of. One definition, so a press is refused exactly where a protection would take it back.
static float rev_line(int ebrake) { return V.eng.rpm_limit * (ebrake ? 0.99f : 0.96f); }
static float stall_line() { return idle_rpm() + ioff(120); }
// ... and the converter's: below its lockup a gear that the converter's multiplication lets carry the wish against the road is no stall, whatever the rpm line says (WP-TC1, the owner
// 04-11: "as long as the lockup is off, you may take a gear that in theory could not"). ONE test (vehicle_tc_carries, core/vehicle.cpp), asked by the hint's refusal, the stall rule,
// M's stall rule, the allowed mask of the layers and the layers' landing window (core/gearchoice.cpp): they cannot disagree, so a press that is taken is not taken back in the next tick.
// The wish is what the pedal asks of the gear now (S.f_need, a force), the speed the one the gear would run at.
// (05-11: and the one step out of the converter's launch gear, down to halfway between the idle and the floor, as the layers' landing window allows it: else the stall
// line, which sits above that half, takes the 2nd back at once.)
static int tc_relief(int g, float v, float wish_n)
{
    if (vehicle_tc_carries(g, v, wish_n)) return 1;
    const eng_bands *b = bands_now();
    const int lg = vehicle_tc_lock_gear(), top = lg > 1 ? lg : 2;
    return V.tc && b->ok && g >= 2 && g <= top + 1 && vehicle_rpm_at(g, v) >= 0.75f * b->idle + 0.25f * b->floor;   // (a quarter: the gear holds; the half is the landing's, core/gearchoice.cpp)
}
// The DRIVELINE's rpm in gear g at v (m/s), the gear's effective ratio calibrated on a converter (core/vehicle.cpp): what the gear turns at once the
// converter is out of the way. The landing checks of an upshift (a floor against lugging, the choice floor, the early upshift's floor) read
// this: the engine's rpm of rpm_at is held at a throttle-scaled stall speed over it while the converter slips, which let every floor through
// (ride 31-10, the Crown Victoria: 1->2->3->4 at 21-33 km/h, the 4th at 576 rpm of driveline). On every other box it is rpm_at.
static float drv_at(int g, float v) { return vehicle_rpm_at(g, v); }
static float force_at(int g, float v, float share) { return vehicle_force_at(g, v, share); }
static float resist(float v) { return vehicle_resist(v); }
static float resist_grade(float v, float pct) { return vehicle_resist_grade(v, pct); }

// The learning into the vehicle model (the force correction, the converter, the effective ratios) is core/shiftmodel.cpp
static int thr_capped() { return sm_thr_capped(); }
static ratio_feeder g_rf;                                        // the calibration of a converter box's effective ratios
static sm_tc_cfg tc_cfg() { sm_tc_cfg c = { p_tc, p_tc_stall, p_tc_lock, idle_rpm() }; return c; }
// The demand the driver (or the cruise/planner) asks now, as a share 0..1 of what the truck can give at this speed: the wish pedal's share,
// else the cruise's wish share (force_mode 1), else the throttle. A share, not a force, so a held pedal (or a held cruise ask) stays the same
// demand while the speed changes (the force the same share asks falls as the speed rises) and in every gear (the shaped share, before phi). This is the
// "asked for" demand the flip lock and the hunt metric compare across a shift.
static float dm_share(int g)
{
    float f = wish_share_eff();
    if (!(f == f)) f = cruise_wish_share();
    if (!(f == f)) f = g > 0 && V.eng_ok ? S.thr : 0.0f;
    return f;
}

// ---------------------------------------------------------------- shift points

// A value that differs per mode (Normal and M take the middle one); mode_character = 0 gives every mode the middle value
static float by_mode(float eco, float normal, float power)
{
    if (p_mode_char <= 0) return normal;
    return S.mode == MODE_ECO ? eco : S.mode == MODE_POWER ? power : normal;
}
static float mode_rpm() { return S.mode == MODE_ECO ? p_eco_rpm : S.mode == MODE_POWER ? p_power_rpm * (1 - S.calm_k) : 0; }
static float mode_reserve()
{
    return S.mode == MODE_ECO ? p_eco_reserve : S.mode == MODE_POWER ? lerpf(p_power_reserve, p_reserve, S.calm_k) : p_reserve;
}

// Stock shift points interpolated from low to high gear, like the game does.
static float eco_dn(int g)
{
    int n = n_gears();
    float k = n > 1 ? (float)(g - 1) / (n - 1) : 1;
    return lerpf(V.eng.range[R_LOW][0], V.eng.range[R_HIGH][0], k);
}
static float eco_up(int g)
{
    int n = n_gears();
    float k = n > 1 ? (float)(g - 1) / (n - 1) : 1;
    return lerpf(V.eng.range[R_LOW][1], V.eng.range[R_HIGH][1], k);
}
// How far the shift window has moved from the calm point to the power range: the demand's movement toward a full pedal.
static float pw_k() { return smooth(p_power_from, p_power_full, S.demand); }
// power_target_frac: the lowest rpm that still gives that share of the peak power; 0 = off
static float pw_target()
{
    float fr = S.mode == MODE_ECO && p_pw_frac_eco > 0 ? p_pw_frac_eco : S.mode == MODE_POWER && p_pw_frac_power > 0 ? p_pw_frac_power : p_pw_frac;
    if (!(fr > 0) || !(V.peak_pw > 0)) return 0;
    // Cached once per engine, share and idle (review 30-10: an explicit key of the inputs, compared field by field, in place of a
    // float "hash" of a few engine values, so two engines can never share a cache line and a reset recomputes).
    struct pw_key { float fr, idle, pk, lim; int n; float curve[24][2]; };
    static pw_key key; static int have; static float val;
    pw_key k = { fr, idle_rpm(), V.peak_pw, V.eng.rpm_limit, V.eng.n_curve, {} };
    for (int i = 0; i < 24; i++) { k.curve[i][0] = V.eng.curve[i][0]; k.curve[i][1] = V.eng.curve[i][1]; }
    int same = have && k.fr == key.fr && k.idle == key.idle && k.pk == key.pk && k.lim == key.lim && k.n == key.n;
    for (int i = 0; same && i < k.n; i++) same = k.curve[i][0] == key.curve[i][0] && k.curve[i][1] == key.curve[i][1];
    if (!same) { key = k; have = 1; val = gs_power_rpm(fr, idle_rpm()); }
    return val;
}
// The power range the shift logic aims at: the engine's own (R_POWER). Its bottom stays where the engine says (the downshift point and the lug check at
// a firm pedal: a climb keeps its pull); its top comes down with power_target_frac, to the rpm that lands one gear step up on the lowest rpm
// giving the share of the peak power (never above the engine's own top)
static float pw_lo() { return V.eng.range[R_POWER][0]; }
// the top for the gear g: where the next gear lands on the target rpm
static float pw_hi(int g)
{
    float hi = V.eng.range[R_POWER][1], t = pw_target();
    if (!(t > 0)) return hi;
    int n = n_gears();
    float step = 1.28f;   // typical gear step (no next gear, or no ratio: the usual high-gear step)
    if (g >= 1 && g < n) { float r0 = vehicle_rpm_at(g, 10.0f), r1 = vehicle_rpm_at(g + 1, 10.0f); if (r0 > 1 && r1 > 1) step = r0 / r1; }
    else if (g < 1 && n > 2) { float r0 = vehicle_rpm_at(n / 2, 10.0f), r1 = vehicle_rpm_at(n / 2 + 1, 10.0f); if (r0 > 1 && r1 > 1) step = r0 / r1; }
    return fminf(hi, fmaxf(t * step, pw_lo() + 150));
}
static float pw_mid() { return 0.5f * (pw_lo() + pw_hi(0)); }
// (k = how far the demand has moved it from Eco's point to the power range's top: pw_k(); 1 = a full demand)
static float up_pt_k(int g, float k)
{
    float mo = mode_rpm();
    float u = fminf(lerpf(eco_up(g) + mo, pw_hi(g) + 0.5f * mo, k), V.eng.rpm_limit * 0.94f);
    // Eco: a soft ceiling (fuel); not during a kickdown (the pedal's or the wish pedal's: review 30-10)
    if (p_mode_char > 0 && p_eco_ceil > 0 && S.mode == MODE_ECO && S.kick_until <= sim_s() && S.wk_until <= sim_s()) {
        float capped = fminf(u, fmaxf(eco_up(g) + mo, V.eng.range[R_HIGH][1] + p_eco_ceil));
        u = capped;
    }
    return u;
}
static float up_pt(int g) { return up_pt_k(g, pw_k()); }
// Light-load downshift point: below the plateau, so that an early upshift to
// the plateau has room before it has to come back down.
static float base_dn(int g)
{
    return fmaxf(idle_rpm() + ioff(220), fminf(eco_dn(g), V.plateau - ioff(100)) + 0.5f * mode_rpm());
}
// The downshift point only rises with demand near full pedal: pressing on at
// cruise in a low gear is not a request for power (kickdown and lugging are).
static float dn_k()
{
    float from = S.mode == MODE_ECO ? 0.9f : S.mode == MODE_POWER ? 0.75f : 0.85f;
    return smooth(from, p_power_full, S.demand);
}
static float dn_pt(int g) { return lerpf(base_dn(g), pw_lo() + 0.5f * mode_rpm(), dn_k()); }
static lesson learned();

// Lowest landing rpm of an early upshift.
// A skip (2 steps or more) may land up to 100 rpm under the plateau (Damien 12-10: fewer shifts when pulling lightly); the
// downshift point plus hysteresis stays the lower bound, so it does not hunt back.
static float cruise_floor(int g, float under = 0)
{
    return fmaxf(V.plateau + p_cruise_rpm + mode_rpm() - under, base_dn(g) + p_hyst + 50) + learned().rpm;
}

static float skip_accel();

static int skip_allowed()
{
    int n = (int)p_max_skip;
    return n;
}
// The anti-hunt dwells, scaled to the box: a torque-converter automatic swaps gears almost instantly (no clutch to open, no pause), so
// its dwells are a moment, not a truck's slow AMT seconds. The anti-hunt intent stays: the values never vanish, they just shrink. Trucks
// (no converter) keep the full ini seconds, bit for bit.
static float dwell_ud() { return V.tc ? 1.0f : p_dwell_ud; }
static float dwell_du() { return V.tc ? 0.7f : p_dwell_du; }
static float min_gap() { return V.tc ? 0.15f : p_min_gap; }
static float flip_s() { return V.tc ? 2.0f : p_ch_flip; }
// Pull a skipped-to gear must keep: less with a slow box (every shift is a
// long gap without power). Damien (ts3b): skip also with a fast (DCT) box.
static float skip_accel() { return p_skip_acc * (1.0f - 0.25f * smooth(0.4f, 1.5f, V.eng.shift_time)); }

// Highest gear above cur+1 (up to max_skip) an upshift at high demand may jump
// to: lands on the plateau (the bands' P1, where the torque is within 2 % of the peak: the layers keep a gear there, so it does not come back down)
// and still pulls hard and keeps most of the pull of cur+1 (a car must not jump 2 -> 5). Used at the rev limit, where a light
// truck arrives in the low gears before the regular upshift (ts3b: 3-4-5). The landing is measured at the speed of now, the one a skip lands at.
static int skip_target(int cur, float v, float f_res, float grade)
{
    int best = cur + 1, n = n_gears();
    if (grade > 0.02f || v * 3.6f < K_ADVICE_MIN_KMH) return best;
    const eng_bands *b = bands_now();
    const float land_min = fmaxf(b->ok ? b->p1 : 0.0f, idle_rpm() + ioff(350));   // (the stall line and a margin stay the floor under a plateau that starts low)
    float a_skip = skip_accel() * 2.0f;
    for (int h = cur + 2; h <= n && h <= cur + 1 + skip_allowed(); h++) {
        if (drv_at(h, v) < land_min) break;
        if ((force_at(h, v, 0.85f) - f_res) / V.mass < a_skip) break;
        if (force_at(h, v, 1.0f) - f_res < 0.7f * (force_at(cur + 1, v, 1.0f) - f_res)) break;
        best = h;
    }
    return best;
}

// Gear below cur whose rpm at speed v lands nearest to target, without
// landing above its own upshift point or near the limiter. max_drop > 0 caps
// how many gears a downshift may skip.
static int pick_down(int cur, float v, float target, bool free = false, int max_drop = 0)
{
    int best = cur;
    float best_err = 1e9f;
    int lo = max_drop > 0 ? cur - max_drop : 1;
    if (lo < 1) lo = 1;
    for (int l = cur - 1; l >= lo; l--) {
        float rpm = rpm_at(l, v);
        if (rpm > V.eng.rpm_limit * 0.93f) break;
        // while braking nobody shifts up, so the upshift point means nothing there: only the limiter and the eco upshift point + 150 count
        if (l < cur - 1 && rpm > (free ? fminf(V.eng.rpm_limit * 0.85f, eco_up(l) + 150) : up_pt(l) - p_hyst)) break;
        float err = fabsf(rpm - target);
        if (err < best_err) { best_err = err; best = l; }
    }
    return best;
}

// ---------------------------------------------------------------- upshift hunting guard

static int grade_class(float pct)
{
    return pct < -1 ? 0 : pct < 1 ? 1 : pct < 2.5f ? 2 : pct < 4 ? 3 : 4;
}

// What was learned for the current grade: the most of this class and the flatter ones.
static lesson learned()
{
    lesson l = { 0, 0 };
    for (int c = 0; c <= grade_class(grade_pct()); c++) {
        l.rpm = fmaxf(l.rpm, g_learn[c].rpm);
        l.res = fmaxf(l.res, g_learn[c].res);
    }
    return l;
}

static void up_failed(const char *how)
{
    lesson *l = &g_learn[grade_class(S.up_grade)];
    l->rpm = fminf(l->rpm + p_learn_rpm, 4 * p_learn_rpm);
    l->res = fminf(l->res + p_learn_res, 4 * p_learn_res);
    logline("upshift to %d had no pull after %.0f s (%s) at %.0f km/h, grade %.1f%%: on this grade now +%.0f rpm, "
            "reserve +%.2f", S.up_gear, sim_s() - S.up_at, how, S.up_kmh, S.up_grade, l->rpm, l->res);
    S.up_gear = 0;
    DEV_NOTE(no_pull());
}

static void up_held()
{
    lesson *l = &g_learn[grade_class(S.up_grade)];
    l->rpm = fmaxf(0, l->rpm - 0.25f * p_learn_rpm);
    l->res = fmaxf(0, l->res - 0.25f * p_learn_res);
    S.up_gear = 0;
}

// Fade with distance; start over when the load changes a lot (trailer).
static void learn_fade(float dist_m)
{
    if (!(g_learn_mass > 0) || fabsf(V.mass - g_learn_mass) > 0.25f * g_learn_mass) {
        if (g_learn_mass > 0 && (g_learn[0].rpm + g_learn[1].rpm + g_learn[2].rpm + g_learn[3].rpm + g_learn[4].rpm) > 0)
            logline("load changed (%.1f -> %.1f t): learned upshift limits cleared", g_learn_mass / 1000, V.mass / 1000);
        memset(g_learn, 0, sizeof g_learn);
        g_learn_mass = V.mass;
    }
    if (p_learn_km <= 0) return;
    float k = expf(-dist_m * 0.6931f / (p_learn_km * 1000));
    for (int c = 0; c < N_CLASS; c++) { g_learn[c].rpm *= k; g_learn[c].res *= k; }
}

// ---------------------------------------------------------------- inputs

// The mode from the game's adaptive-shift cvar; log and a tone on a change.
// Called from the gearbox hook and from the frame (also while standing).
static void mode_update()
{
    float a = sense_adaptive();
    int m = mode_of(a);
    static int logged = -1;
    if (m != logged) {
        if (logged >= 0 && p_tones > 0) {
            // Eco low bell, Normal middle, Power rising pair, M two equal
            play_cue(m == MODE_ECO ? CUE_MODE_ECO : m == MODE_POWER ? CUE_MODE_POWER : m == MODE_M ? CUE_MODE_M : CUE_MODE_NORMAL);
        }
        logged = m;
        logline("mode %s (g_adaptive_shift = %g)", mode_label(m), a);
    }
    S.mode = m;
}

// Continuity broke (core/telemetry.cpp session_reset): pause menu, hitch, engine start (SOFT: timers, filters, holds);
// trailer (de)coupled (LOAD: mass-dependent values); other truck, save/load, teleport (HARD: everything learned).
static double g_reset_at = -1e9;  // last continuity break (the neutral watchdog acts faster right after one)
// The standing decision of the layers (K1 round 14): the whole score costs hundreds of microseconds, and a choice made again every frame (60 or 144 a second) buys nothing the frame before did not have.
// It is made again when GC_DECIDE_S game seconds have passed, and at once when something happened that the old choice did not see: a press (the foot rising, or a pedal step since the choice), the kickdown,
// the brake, the pedal on or off, the mode or the controller, a shift made or in the gap, the gears the caller allows, the revs at the limiter wall (the margin GC_LATENCY_S), and the speed moved by a whole cell of the
// path's grid (the choice was made around a cell of the speed; the cell is the pull of a time step, so this bounds how stale a launch's choice gets however far apart the decisions are).
struct gc_stand {
    bool valid; double at; int g; gc_out out; char terms[300];
    int cur, state; unsigned long long allowed; double shift_at; int shift_dir, gear_before; float foot, v;
};
static gc_stand s_gs;
// What a decision costs on this PC and so how far apart the decisions are (K1 round 14). The cost of the score is measured, not assumed: the median of the last decisions, read from the performance counter
// around gc_choose (a median, because a page fault or a preempted thread makes one sample many times the rest). The interval is GC_DECIDE_S, or cost / GC_CPU_BUDGET where that is longer: the decisions
// take at most the share GC_CPU_BUDGET of the game thread. It is logged when it moves to another octave of GC_DECIDE_S (the line is for the reader of the log, the interval itself is not rounded).
struct gc_cost_clock { float us[15]; int n, k; };
static gc_cost_clock s_gcc;
static float s_gs_interval = GC_DECIDE_S;
static int s_gs_logged = 0;
static void gc_cost_note(float us)
{
    s_gcc.us[s_gcc.k] = us; s_gcc.k = (s_gcc.k + 1) % 15; if (s_gcc.n < 15) s_gcc.n++;
    if (s_gcc.n < 8) return;   // (the first few decisions of a session are no measure: cold caches)
    const int m = s_gcc.n < 15 ? s_gcc.n : 15;
    float sorted[15]; memcpy(sorted, s_gcc.us, sizeof sorted);
    std::sort(sorted, sorted + m);
    const float med = sorted[m / 2];
    s_gs_interval = fmaxf(GC_DECIDE_S, med * 1e-6f / GC_CPU_BUDGET);
    const int octave = s_gs_interval > GC_DECIDE_S ? 1 + (int)floorf(log2f(s_gs_interval / GC_DECIDE_S)) : 0;
    if (octave != s_gs_logged) {
        s_gs_logged = octave;
        logline("layers: decisions %.0f ms apart (a decision costs %.0f us, the median of the last %d; at most %.0f %% of the game thread)", s_gs_interval * 1000.0f, med, s_gcc.n, GC_CPU_BUDGET * 100.0f);
    }
}
static void ts_reset(int level, const char *why)
{
    double none = -1e9;
    g_reset_at = sim_s();
    ahead_reset();                          // the road view (its 0.25 s cache and its certainty fade) belongs to the road we left
    S.hold_until = S.kick_until = 0;
    S.out_of_gear = 0;
    S.lug_back = 0; S.lug_back_at = 0;
    S.lug_ratio = 1; S.wk_d = 0; S.wk_on = S.wk_fast = 0; S.wk_until = S.wk_retry = 0; S.wk_pt = 0;
    S.demand_at = S.pedal_at = 0;           // the next dt is "unknown": filters step, no kickdown from a stale pedal
    S.ours_at = S.shift_at = none;          // dwell and take-over timers must not hold a shift back after a pause
    S.was_kick = 0; S.kick_at = none;       // a press before the break is no press now
    S.foot_f = 0; S.foot_prev = 0; S.foot_t = none; S.tgt_rpm = 0; S.tgt_kd = 0;
    s_gs.valid = false;
    S.sh_braked = 0;                        // no shift of before the break is a braking one
    S.top_at = none; S.crest_ref = 0; S.crest_said = 0;  // no "over the top" hold across a teleport
    S.bd_at = S.ad_at = none;               // no braking downshift is expected across a break
    S.svc = 0;                              // no brake of before the break brakes now
    S.gap_since = 0;                        // a pause in the gap state is not a stuck shift (the timer starts again)
    S.pedal_prev = clampf(T.in_thr, 0, 1);
    S.ws = 0; S.ws_t = 0;                        // the filtered wish of the layers starts over
    S.ex_pre = S.ex_v = 0; S.ex_t = S.wish_on_t = none; S.lift_done = S.lift_pick = 0;   // so does the demand expected after a lift (a pause is no lift)
    S.lat_t = none;
    S.adv_dir = S.advice = 0; S.hold_cand = 0;   // the advice and the ticker's hold start over
    if (level >= RESET_LOAD) {
        S.f_need = 0;
        S.up_gear = 0;
        S.strain = 0;
        fl_reset(&S.fl);      // a load (de)couples: the resistance learned against the old mass is void (f_bias reset in core; f_scale kept)
    }
    if (level >= RESET_HARD) {
        S.calm_k = 0; S.calm_since = 0;     // Power starts with its own shift points again
        S.pedal_on_at = none; S.kd_at = none;
        memset(g_learn, 0, sizeof g_learn);
        g_learn_mass = 0;
        S.self = NULL;                      // refresh() starts over from the gearbox it sees next
        S.eng_seen = 0;                     // the engine line is logged again
        S.idle = 0;                         // idle is learned again (engine on 5 s, standing still)
        S.rpm_odd_s = 0;
        S.sl_n = 0; S.dec_for = 0;          // the shift losses are learned again (another truck shifts differently)
        S.lb_beta_n = 0; S.lb_dec = 0;
        S.lat_sxx = S.lat_sxy = S.lat_syy = S.lat_w = 0;   // the steering gain belongs to the truck
        S.tc_slip_s = 0;
        S.unsupported = NULL;
        for (float &h : S.grade_hist) h = grade_pct();   // (no trend yet: the grade of now all along)
        S.pk = -99; S.pk_at = 0;            // the grade peak belongs to the road we left
        S.letter_state = -1;
        S.cc_thr = 0;
        S.demand = 0;
        S.seed_demand = S.seed_force = 1;   // (sim X4: from 0 a full pedal on a climb looked like a light one and skipped to a gear without pull)
        S.rolling = 0;
        S.roll_exit = 0;
        g_rolling = 0;
        s_nq.at = -1e9; s_nq.want = 0; s_nq.roll = 0; s_nq.no = NULL;   // a neutral request belongs to the session it was made in
    }
}

// kickdown_by_wish (the wish pedal giving the throttle): the kickdown as the driver's dissatisfaction. Once per frame.
// r = the force the pedal wishes / the force this gear gives at full throttle now. r <= 1: the gear holds (the wish pedal just opens the throttle
// further). D adds up max(0, r - kick_ratio) x dt, falls when r is under kick_ratio, and a kickdown is due at D >= kick_level. A fast press
// (over kickdown_rate, per mode) halves what is left of the level, once per press. Right after an upshift (choice_flip_s) only r >= kick_ratio + 0.3
// may trigger. A flat pedal (kickdown_pedal, per mode) with r over kick_ratio triggers at once: the old rule's last resort. The gear itself is the layers' kickdown (GC_KICKDOWN, by the drive time); this only raises the demand and
// lifts the Eco ceiling (up_pt_k) and the eco-roll's veto.
static void wish_kick_update(int g, float v, float pedal, double dt, double now)
{
    if (!(dt > 0)) return;   // (refresh runs more than once per frame: a call without time passed changes nothing)
    S.wk_on = 0;
    double pdt = now - S.wk_pt;
    float rate = 0;
    if (pdt > 0.05 && pdt < 0.5) rate = (float)((pedal - S.wk_pp) / pdt);
    if (pdt > 0.05) { S.wk_pp = pedal; S.wk_pt = now; }
    // (what this gear is asked: F_d(g, v) of the torque pedal, held under a cap or a limiter; -1 while the pedal gives no demand)
    float fw = p_kw > 0 && S.mode != MODE_M && S.ours && g > 0 && V.eng_ok && !V.electric ? wish_in(g, v) : -1.0f;
    if (fw >= 0) fw *= wish_limit_k();
    // (the own cruise with [cruise] force_mode 1 asks a force too: the same kickdown when its force is what this gear cannot give, cruise on and the pedal off)
    int cc_wish = 0;
    if (!(fw >= 0) && p_kw > 0 && S.mode != MODE_M && S.ours && g > 0 && V.eng_ok && !V.electric) {
        // (the overtake assist's pull toward the set speed is shaped, not a request for power: its force wish must not
        // kick down on its own; the lugging downshift still works if the speed falls)
        fw = cruise_overtake_on() ? -1.0f : cruise_wish_force(v);
        if (fw >= 0) cc_wish = 1;
    }
    float fg = g > 0 && V.eng_ok ? vehicle_force_at(g, v, 1.0f) : 0.0f;
    if (!(fw >= 0) || !(fg > 1)) { S.wk_d = 0; S.wk_fast = 0; S.wk_until = 0; return; }
    const float thr = fmaxf(by_mode(p_kr_eco, p_kr_norm, p_kr_pw), S.lug_ratio), level = fmaxf(0.0f, by_mode(p_kl_eco, p_kl_norm, p_kl_pw));
    const float r = fw / fg;
    S.wk_on = 1;
    S.wk_r = r;
    if (r > thr) S.wk_d = fminf(2 * level, S.wk_d + (r - thr) * (float)dt);
    else S.wk_d = fmaxf(0.0f, S.wk_d - level / 1.5f * (float)dt);
    if (!cc_wish) {   // (the pedal-only rules: the cruise has no press, no flat pedal to measure, and its throttle is not a lift)
        // a fast press (rate over kickdown_rate) that the gear does not give: half of what is left of the level, once per press
        if (rate < 0.8f * by_mode(2.5f, p_kick_rate, 1.0f)) S.wk_fast = 0;
        else if (rate > by_mode(2.5f, p_kick_rate, 1.0f) && r > thr && !S.wk_fast) { S.wk_fast = 1; S.wk_d += 0.5f * fmaxf(0.0f, level - S.wk_d); }
        if (r > thr && pedal >= by_mode(0.97f, p_kick_pedal, 0.85f)) S.wk_d = fmaxf(S.wk_d, level);
        if (pedal < 0.3f) S.wk_until = 0;
    }
    bool after_up = S.shift_dir > 0 && now - S.shift_at < flip_s();
    if (S.wk_d >= level && r > thr && now >= S.wk_retry && !(after_up && r < thr + 0.3f) && S.wk_until <= now) S.wk_until = now + 3;
}

// How long the deceleration a driver's service brake made outlasts his pedal: the lag of that deceleration as the gearbox sees it. Two lags in series: the telemetry's low-pass on T.accel
// (ACCEL_LAG_S) and the game's brake, which lets go over a moment. Measured on the six clean releases of the ride of 08-11 evening (pedal > 0.15 to 0.00 for 1 s, no retarder, no engine
// brake, 39.5 t): the deceleration falls by a factor e in 0.55 .. 0.76 s (median 0.66); the filter alone gives 0.5, so the brake's own release is up to 0.26 s on top: 0.5 + 0.3.
static const float SVC_TAIL_LAG_S = ACCEL_LAG_S + 0.3f;

static void refresh(void *self)
{
    double now = sim_s();
    bands_set_idle(idle_rpm());   // the bands (core/bands.cpp) follow the truck's idle (learned at a standstill, else the ini's)
    bands_set_floor_off(ioff(120) + p_hyst);   // and the lug floor: the stall line (idle + 120, by the engine's span) + the hysteresis
    // The vehicle model (core/vehicle.cpp) first: inside the game's gearbox update the grade is this frame's and the
    // mass is fresh right after a reset; another gearbox object makes the model start over as well.
    vehicle_update(self);
    if (V.eng_ok && bands_stale()) bands_refresh(&V.eng);   // (the idle was learned, or the plateau moved the span: the floor with it)
    if (self != S.self) {
        // another truck (or the first update): start from its current gear
        if (S.self) ts_reset(RESET_HARD, "gearbox object changed");
        S.self = self;
        S.last_gear = S.commanded = sense_gb_gear(self);
        S.hold_until = S.kick_until = 0;
        S.letter_state = -1;
    }
    if (V.electric) unsupported("electric motor: no gearbox to shift");
    if (V.eng_ok && (!S.eng_seen || V.eng.torque != S.eng_torque || V.eng.rpm_limit != S.eng_limit)) {
        const engine_info *e = &V.eng;
        logline("engine: %.0f Nm, limit %.0f rpm, %d curve points, plateau from %.0f rpm, ranges brake %.0f-%.0f "
                "low %.0f-%.0f high %.0f-%.0f power %.0f-%.0f, shift time %.2f s",
                e->torque, e->rpm_limit, e->n_curve, V.plateau, e->range[0][0], e->range[0][1], e->range[1][0],
                e->range[1][1], e->range[2][0], e->range[2][1], e->range[3][0], e->range[3][1], e->shift_time);
        const eng_bands *b = bands_now();
        if (b->ok) {
            logline("Bands: no-go < %.0f, lug %.0f-%.0f, eco %.0f-%.0f, power %.0f-crossover, temporary to %.0f, no-go from there to the limit "
                    "(power peak P3 %.0f; game low range from %.0f)", b->floor, b->floor, b->p1, b->p1, b->p2, b->p2, b->cap, b->p3, b->game_low_lo);
        }
        // the whole torque curve (rpm: share of the peak), so a ride log shows where the torque falls off
        char cv[24 * 16] = "";
        for (int i = 0, k = 0; i < e->n_curve && i < 24; i++) {
            int w = snprintf(cv + k, sizeof cv - k, "%s%.0f: %.2f", i ? ", " : "", e->curve[i][0], e->curve[i][1]);
            if (w < 0 || w >= (int)sizeof cv - k) break;
            k += w;
        }
        logv(1, "engine curve: %s", cv);
        S.letter_state = -1;
        if (S.eng_seen && (V.eng.torque != S.eng_torque || V.eng.rpm_limit != S.eng_limit)) fl_reset(&S.fl);  // another engine: the learned force correction starts over
        S.eng_seen = 1; S.eng_torque = e->torque; S.eng_limit = e->rpm_limit;
    }
    if (!V.eng_ok && S.eng_seen) { logline("engine data unreadable, TrueShift pauses"); S.eng_seen = 0; }

    // Mode from the game's adaptive-shift keybind, once a second. The game
    // ignores this cvar while cruise control is on (game_actor+0x199); we don't.
    static double mode_at = -1e9;
    if (now - mode_at > 1 || now < mode_at) {
        mode_at = now;
        mode_update();
    }

    if (now - S.grade_at >= 1 || now < S.grade_at) {
        S.grade_at = now;
        memmove(S.grade_hist + 1, S.grade_hist, sizeof S.grade_hist - sizeof S.grade_hist[0]);
        S.grade_hist[0] = grade_pct();
    }
    vehicle_rpm_k_sane();

    float v = fabsf(T.speed), kmh = v * 3.6f;
    // TrueShift owns every forward shift (owner decision 60, 04-11): no takeover speed, no takeover rpm, no handback. Creep and LowSpeed own the clutch and the launch as they did; the start
    // gear was ours already; from the standstill on the decider below picks the gear, the game's chooser only fills in where it has no opinion (neutral, reverse). The parking brake is the one
    // state that is not driving. [lowspeed] owner=1: at the owner's hand-off (clutch locked + 0.5 s) the dwell timers start, as they did.
    int was = S.ours;
    // The hand-over rule of before (the three keys) is kept for two jobs only, as it was: M mode, the driver's own mode, where the game still picks the gear in the low-speed range (the
    // start gear after a stop, the stop's downshifts), and the start of the dwell timers at the owner's hand-off -- exactly where it started them (a hand-off that the rpm rule had not
    // already handed over: a launch that reaches the hand-off under 1000 rpm; one that reached it over gets no new dwell). Nothing else reads it.
    const int old_was = S.old_ours;
    if (T.parking_brake || kmh < p_off_kmh) S.old_ours = 0;
    else if (low_handed()) S.old_ours = 1;
    else if (p_on_rpm > 0 ? ((T.rpm >= p_on_rpm && T.gear > 0 && clutch_seen() < 0.1f) || kmh >= fmaxf(p_on_kmh, 20.0f)) : kmh >= p_on_kmh) S.old_ours = 1;
    if (S.old_ours && !old_was && low_handed()) S.shift_at = now;
    S.ours = S.mode == MODE_M || p_own_low < 0.5f ? S.old_ours : T.parking_brake ? 0 : 1;
    if (S.ours && !was) { S.ours_at = now; S.ours_launch = kmh < 25; }

    // A converter the last drive of this vehicle found is known from the start (the detection needs a second of slip, and the first upshifts of
    // a launch were a manual box's: ride 31-10, the Crown Victoria 1->2 at 26 km/h before the detection, back at 38 km/h)
    sm_tc_known(tc_cfg());
    // Learn the vehicle model's rpm correction while the clutch is closed and the speed steady. On a torque-converter box the
    // engine runs faster than the driveline below the lockup (kr > 1), so nothing is learned from a slipping converter: the tire
    // correction, the force correction and the shift points all assume a locked driveline.
    int g = sense_gb_gear(self);
    if (sm_learn_rpm(tc_cfg(), g, v, clampf((float)(now - S.demand_at), 0, 0.5f), &S.tc_slip_s, &S.rpm_odd_s))
        unsupported("engine rpm does not follow the road speed: unusual gearbox?");
    ratio_feed(&g_rf, g, v, now, S.shift_at, idle_rpm());

    double dt = now - S.demand_at;
    if (dt < 0 || dt > 1) dt = 0;  // a pause or hitch is no time step (filters and integrators skip it; ts_reset also zeroes demand_at)
    S.demand_at = now;

    // Throttle that counts: the pedal directly; cruise control (bang-bang in
    // the game: full below the set speed, zero above) only smoothed.
    // The player's pedal comes from the SDK: inner+0x1424 already holds the
    // cruise control's throttle (ride ts2: every cruise burst looked like a kickdown).
    float pedal = clampf(T.in_thr, 0, 1), eff = sense_gb_thr_eff(self);
    int cc = T.cruise > 0.5f;
    static double cc_zero_since;
    if (eff > 0.02f || !cc) cc_zero_since = now;
    // Bursts are averaged over cruise_damp_s; when it stays off the throttle
    // (slowing for traffic, adaptive cruise) the damped value follows in ~1 s.
    float cc_tau = now - cc_zero_since > 1.5 ? 1.0f : p_cc_damp;
    // With our own cruise throttle (smooth, no bursts): its throttle is already smooth, so it is followed as it falls (0.5 s) instead of
    // through the three filters below in a row (cruise_damp_s, demand, force), which kept 10 kN 'needed' for 20 s where 3 kN held the speed
    // (ride 7, 396-416 s). Not while the driver's own pedal is on
    const bool cc_own = cc && g_cruise_smooth && pedal < 0.05f;
    if (cc_own && eff < S.cc_thr) cc_tau = fminf(cc_tau, CC_FALL_S);
    S.cc_thr += (float)(dt / (cc_tau + dt)) * (eff - S.cc_thr);
    if (!cc) S.cc_thr = eff;
    // with the Eco / Power pedal curve the engine's own throttle (eff) is what counts, not the raw pedal
    S.thr = cc ? fmaxf(pedal, S.cc_thr) : g_pedal_mapped ? eff : fmaxf(pedal, eff);
    // a cap holds the throttle below the pedal (the own speed limiter, the rollover guard, launch control): the capped throttle counts
    if (!cc && S.thr > eff && thr_capped()) S.thr = eff;

    // Demand rises fast (0.3 s) and falls slowly (2 s). Load itself does not
    // move the shift points: the force checks handle it.
    float tau = S.thr > S.demand ? by_mode(0.5f, 0.3f, 0.2f) : by_mode(1.5f, 2.0f, 3.0f);
    if (cc_own && S.thr <= S.demand) tau = fminf(tau, CC_FALL_S);
    if (S.seed_demand) { S.demand = S.thr; S.seed_demand = 0; }   // right after a hard reset: the pedal of now, not 0
    else S.demand += (float)(dt / (tau + dt)) * (S.thr - S.demand);

    // Tractive force asked for (clutch closed only): up in 0.3 s, down in 3 s.
    if (g > 0 && clutch_seen() < 0.1f && V.eng_ok) {
        float f = force_at(g, v, S.thr);
        if (S.seed_force) { S.f_need = f; S.seed_force = 0; }   // right after a hard reset: what this pedal asks now, not 0
        else { float tf = f > S.f_need ? 0.3f : cc_own ? CC_FALL_S : 3.0f; S.f_need += (float)(dt / (tf + dt)) * (f - S.f_need); }
    }

    // Extra resistance / force factor the vehicle model misses, learned into it: V.f_bias is the additive resistance
    // error as before, V.f_scale a multiplier on the modeled engine force (a vehicle the model under- or over-rates). It only learns in steady
    // moments (core/shiftmodel.cpp); the force and the resistance are lagged 0.5 s like T.accel.
    if (g > 0 && V.eng_ok && dt > 0)
        fl_step(&S.fl, g, v, eff, dt, now, fmax(S.brake_at, S.own_brake_at), S.shift_at, T.motor_brake || T.retarder || S.eb_level >= 0.01f);
    if (T.in_thr > 0.02f) { S.pedal_on_at = now; S.drv_v = fabsf(T.speed); }
    if (act_request() > 0) S.own_brake_at = now;
    // The driver's service brake, as the deceleration it makes still sees it: the pedal reads at once, the deceleration it made comes with a lag (SVC_TAIL_LAG_S), so a pedal that feathers
    // (the ride of 08-11: 0.21 0.01 0.00 0.00 0.06 ...) reads 0.00 in its gaps while the truck is still braking, and the braking rule (decide_gear_rules) took the deceleration it saw for a
    // brake nobody pressed (the unseen kind: AEB, the plugin's own) and landed high. The memory follows the pedal up at once and down by that lag, every frame: it is above the pedal's noise
    // floor (0.05, the brake test of the rules) for as long as the deceleration it made is above its own entry there (-0.8 m/s2) whenever the brake gives less than 0.8 / 0.05 = 16 m/s2 per
    // unit, the top of what a truck's service brake was ever measured at (core/brakemodel.h BM_SVC_GAIN_HI): the tail of his brake is never taken for a brake of nobody's.
    // (Only his foot is remembered: while the suite brakes itself (act_request: the AEB, the cruise, the assist) the pedal read is not fed. In the game the request is not in T.in_brake;
    // the sims that make the plant's pedal of it (sim_pcc) get the old rule untouched, the momentary read below.)
    S.svc = fmaxf(act_request() > 0 ? 0.0f : T.in_brake, S.svc * expf(-(float)dt / SVC_TAIL_LAG_S));
    // Power on a flat, light, steady stretch for a while: the shift points sink to Normal's, so a long calm cruise ends in the top gear
    // and Power does not cost fuel for nothing (Damien 25-10: it stayed in the 11th at ~1480 rpm); a firm pedal brings them back at once
    {
        bool calm_any = p_mode_char > 0 && S.thr < p_pw_calm_thr && fabsf(T.accel) < 0.2f && fabsf(grade_pct()) < 1.5f && v > 5.5f;
        bool calm = calm_any && S.mode == MODE_POWER;
        if (!calm_any) S.calm_since = now;
        float tgt = calm && now - S.calm_since > 8 ? 1.0f : 0.0f;
        float rate = tgt > S.calm_k ? 1.0f / 6 : 1.0f;
        S.calm_k += clampf(tgt - S.calm_k, -rate * (float)dt, rate * (float)dt);
    }

    // Kickdown: the pedal pressed past kickdown_pedal quickly. Armed for 3 s
    // (cleared by the kickdown shift itself or by lifting below 0.8). With kickdown_by_wish and the wish pedal on the wish decides instead
    wish_kick_update(g, v, pedal, dt, now);
    double pdt = now - S.pedal_at;
    if (pdt > 0.05) {
        if (pdt < 0.5 && !S.wk_on) {
            float rate = (float)((pedal - S.pedal_prev) / pdt);
            float kp = by_mode(0.97f, p_kick_pedal, 0.85f), kr = by_mode(2.5f, p_kick_rate, 1.0f);
            if (pedal >= kp && S.pedal_prev < kp && rate >= kr) S.kick_until = now + 3;
        }
        S.pedal_prev = pedal;
        S.pedal_at = now;
    }
    if (pedal < 0.8f) S.kick_until = 0;
    if (S.kick_until > now || S.wk_until > now) S.demand = 1;

    learn_fade(v * (float)dt);
    // A stop ends the test of the last upshift: a hint after it is about the new launch (Damien 11-10, 1302-1311 s:
    // 4->6, stop, launch, hint 6->1 at 6 km/h counted as "no pull" and later blocked 8->9 on the flat at 1613 rpm)
    if (S.up_gear && v * 3.6f < 3) S.up_gear = 0;

    // After an upshift: throttle well above what the new gear was expected to
    // need, while not gaining speed, adds up; a little over for long or a lot
    // over for short both end in a "no pull" downshift (see decide_gear).
    if (S.up_gear && g == S.up_gear && clutch_seen() < 0.1f) {
        float over = S.thr - fmaxf(S.up_pred + 0.15f, 0.8f);
        if (over > 0 && T.accel < 0.05f) S.strain += over * (float)dt;
        else S.strain = fmaxf(0, S.strain - 0.05f * (float)dt);
        if (now - S.up_at > 30 && S.strain < 0.2f) up_held();
    } else S.strain = 0;
    // An upshift that landed in braking or a lift-off (Damien 24-10, 1831 s and 3543 s) fails for that, not for lack of pull:
    // no lesson from it (one such lesson held back 5 -> 6 at full throttle on a 3 % climb two minutes later).
    if (S.up_gear && now - S.up_at < 6 && (fmaxf(T.in_brake, T.eff_brake) > 0.05f || (T.accel < -1.0f && pedal < 0.1f))) S.up_tainted = 1;

    // Engine brake / retarder strength 0..1.
    float e = 0;
    // a retarder brakes the driveline and needs no engine rpm: only with retarder_downshift (Damien 11-10: first
    // retarder click sent the rpm up)
    if (T.retarder > 0 && p_ret_down > 0) e = T.retarder_steps > 0 ? clampf((float)T.retarder / T.retarder_steps, 0, 1) : 1;
    float lv = sense_ebrake_level(self);  // also the automatic engine brake, which the SDK flag misses
    // ... but not the engine brake the game adds by itself to the retarder lever: that counts as the retarder (Damien 11-10,
    // first 0.6 ride 1405/1467 s: retarder 1-2 in town, engine brake level 0.4-0.7 with the SDK flag off, "engine brake (level)"
    // downshifts 5-4-3-2, each with the game's throttle blip: "gas and engine brake and retarder at once")
    if (!T.motor_brake && T.retarder > 0 && p_ret_down <= 0) lv = 0;
    if (lv > 0.01f) e = fmaxf(e, lv);
    else if (T.motor_brake && lv < 0) e = fmaxf(e, 0.5f);
    S.eb_level = e;
    // (the driver's lever, for the band rule: the retarder step as a share of its steps, the engine brake the SDK flags as held and its stage as the game reports it; the automatic engine brake the
    // pedal asks for is not a lever and stays on the held-stage rule)
    S.lv_ret = T.retarder > 0 && p_ret_down > 0 && T.retarder_steps > 0 ? clampf((float)T.retarder / T.retarder_steps, 0, 1) : 0.0f;
    S.lv_eb = T.motor_brake ? (lv > 0.01f ? lv : 0.5f) : 0.0f;
    S.eb_full = sense_ebrake_downshift(self) > 0;
}

// ---------------------------------------------------------------- the decision

// Gear whose rpm at speed v lands nearest to target, staying between its own
// light-load downshift point and the limiter (re-engaging after a roll).
static int pick_for_speed(float v, float target)
{
    int best = 0;
    float best_err = 1e9f;
    for (int g = 1; g <= n_gears(); g++) {
        float r = drv_at(g, v);
        if (r > V.eng.rpm_limit * 0.9f || r < base_dn(g) + p_hyst) continue;
        float err = fabsf(r - target);
        if (err < best_err) { best_err = err; best = g; }
    }
    // nothing fits: too slow for every gear (crawling at 4 km/h after a pause: 25-10, "engage at 4 km/h: gear 12") -> the lowest, not the top
    if (!best) return drv_at(1, v) < base_dn(1) + p_hyst ? 1 : n_gears();
    return best;
}

// ---------------------------------------------------------------- eco-roll

// Back into drive after a roll: a gear for what is asked now.
static int roll_exit_gear(float v, int ebrake, float brake)
{
    // with cruise back into the gear it rolled from while that still fits: the cruise's own gear choice, in every mode (Power's reserve is for
    // a driver's pedal; PCC sim 29-10, Power after a roll in a dip: one gear down for the whole climb and the flat after it, +1-1.7 % fuel)
    int g0 = S.roll_from;
    if (T.cruise > 0.5f && !ebrake && !(brake > 0.02f) && g0 >= 1 && g0 <= n_gears() && rpm_at(g0, v) >= base_dn(g0) + p_hyst && rpm_at(g0, v) <= V.eng.rpm_limit * 0.9f)
        return g0;
    float target = ebrake ? V.eng.range[R_BRAKE][0] + 100
                   : brake > 0.02f ? eco_up(1) - 200
                   : lerpf(cruise_floor(n_gears()) + 100, pw_mid(), pw_k());
    return pick_for_speed(v, target);
}

static float roll_limit()   // the road's limit (+ the cap) under the truck's top; 0 = neither known
{
    float lim = road_limit_ms() > 1 ? road_limit_ms() + overspeed_cap_now() / 3.6f : 0.0f, top = speed_top_kmh() / 3.6f;
    return lim > 0 ? (top > 1 ? fminf(lim, top) : lim) : top;
}
// The speed a roll may reach (m/s): the road's limit (plus overspeed_cap_kmh, where a fine starts), never over the truck's own top (its
// limiter); with cruise and no limit known, 3 km/h over the set speed (the roll's old band); without cruise and nothing known, the speed of now.
// With cruise never past where the game's cruise brakes (cruise_hold_kmh: the smart overspeed, less on a descent): there its engine brake /
// retarder end the roll (PCC sim 29-10, 40 t on rolling hills: rolls of 1 s, each followed by three shifts, fuel +1..+4 %)
static float roll_allowed(int cc, float v)
{
    float a = roll_limit(), h = cc ? cruise_hold_kmh(NAN) : NAN;
    if (a > 1) a = cc ? fmaxf(a, T.cruise) : fmaxf(a, v);
    else a = cc ? T.cruise + 3 / 3.6f : v;
    return h == h && h >= 0 ? fminf(a, T.cruise + h / 3.6f) : a;
}
// The speed worth keeping (m/s): the cruise's set speed; without cruise the speed the driver last held with the pedal, never over the
// limit (speed the truck gathered on a descent with the pedal off is not wanted: sim 29-10, 40 t down 3 %, 70 -> 105 km/h in gear, then
// a roll on the flat 'kept' 105 that the driver lets go of anyway)
static float roll_use(int cc)
{
    if (cc) return T.cruise;
    float a = roll_limit(), d = S.drv_v > 0.5f ? S.drv_v : fabsf(T.speed);
    return a > 1 ? fminf(d, a) : d;
}
// The one protection of a roll nobody asked for (owner decision 67, 04-11): really running away. A roll the box started without cruise lasts until the driver acts (roll_wanted);
// the truck that gathers speed on a descent in neutral is the driver's to hold ("als trucker gebruik je retarder of rem"), but not past the speed it may be held to. That speed is
// roll_limit(): the road's limit plus the cap (where a fine starts), under the truck's own limiter. With none of them known it is the speed at which the exit gear could no longer go in
// without the rev limiter: the top gear at 90 % of the rev limit (pick_for_speed's own line for a gear that fits). (Manager 05-11: the driver's held speed plus the cap was tried as the
// stand-in and ended every roll of run.sh B3d at 85 km/h on its descents, 7 hunting pairs: on rolling hills the speed a descent gives over the held speed IS what the roll is for, and
// it is no running away. In the game a road limit or the limiter is nearly always known.)
static float roll_runaway_ms()
{
    const float lim = roll_limit();
    if (lim > 1) return lim;
    const int n = n_gears();
    const float r10 = n >= 1 ? drv_at(n, 10.0f) : 0.0f;
    return r10 > 0 && V.eng.rpm_limit > 0 ? 10.0f * V.eng.rpm_limit * 0.9f / r10 : 0.0f;
}
// The trigger is derived, not a number: the speed carried on at the acceleration the truck has now (the larger of the telemetry's, which lags the road by half a second, and what the
// road gives the coasting truck by the model, which does not) for the time the exit gear needs to go in (a shift time, the converter's 0.3 s, and the layers' latency) would pass it.
// So the gear is in when the speed gets there, and not before: a truck that gathers speed slowly is let go to the limit, one that gathers it fast is taken in earlier by exactly the
// speed it gains on the way. A truck that is over it and falling is not running away (the pedal, the brake, the lever and the hint end the roll above). One line says why.
static int roll_runaway(float v)
{
    const float lim = roll_runaway_ms();
    if (!(lim > 1)) return 0;
    const float t_in = (V.tc ? 0.3f : V.eng.shift_time) + GC_LATENCY_S;
    const float a = fmaxf(T.accel, V.mass > 0 ? -resist_grade(v, grade_pct()) / V.mass : 0.0f);
    if (!(a > 0) || !(v + a * t_in >= lim)) return 0;
    S.roll_prot = 1;
    logline("eco-roll runaway protection: %.1f km/h gaining %.2f m/s2, in %.2f s that is %.1f km/h = the limit %.1f km/h (%s): back into gear", v * 3.6f, a, t_in, (v + a * t_in) * 3.6f, lim * 3.6f,
            roll_limit() > 1 ? "the road's limit plus the cap, or the truck's limiter" : "the top gear at 90 % of the rev limit");
    return 1;
}
// The roll that fades to a crawl (WP-ER1 item 5, a proposal for the owner, droppable): nothing but the driver ends a roll without cruise, so on a rise the speed falls through every gear to
// a standstill in neutral (on a slope the truck would roll back). It ends where the exit gear would be the launch gear: the physics pick for the speed of now (roll_exit_gear, which is
// pick_for_speed) is a gear a pull-away starts in or lower, so only those fit, and the speed is the crawl. The gear goes in there and the stop path takes the truck: the stop threshold
// of the gear in (LowSpeed's open point, the game's), the hold at a standstill. No number: the launch gear is the one bs_start_gear gives for this mass and grade.
static int bs_start_gear(float thr);
static int roll_fades(float v)
{
    const int launch = bs_start_gear(0.0f), g = roll_exit_gear(v, 0, 0);
    return launch > 0 && g > 0 && g <= launch;
}
// How long the neutral way has to stay above the bottom of the driver's band before the box starts a roll that nobody but the driver ends (WP-ER1, owner decision 67: "strenger erin,
// ... Eco 5 / Normal 10 / Power 15", his example and no figure). The roll stays now, so it starts only where it is a sure thing for a while, and what a while is depends on what the mode
// buys with the time: Power's time is dear (the roll costs the shifts out and in, and a coast that gives out early costs them for nothing), Eco's diesel is. The unit is the plan horizon
// of every term of the score (GC_HORIZON, 20 s: the drive the layers judge a gear or a shift over, and over which the demand memory fades), the share is the mode's share of time in its
// currency. The currency is the exchange rate of the gear choice, fuel_weight / response_weight (seconds of delay one litre is worth); Normal's rate is the owner's 50:50, a mode whose
// rate is r times Normal's weighs time 1 / (1 + r): his 25:75 for Eco (r = 3) and 75:25 for Power (r = 1/3, keuze 37). The shipped weights are Eco 18 / 0.6, Normal 5 / 1, Power 5 / 1.6
// (G1b: the largest that pass the acceptance rows, not the design rates): r = 6 and 0.625, so T = 2.9 s in Eco, 10 s in Normal, 12.3 s in Power (the design rates would give 5 / 10 / 15).
// A weight of 0 for Normal (the fuel is not priced) leaves no currency: every mode weighs time 1/2.
// The mode's one fact (core/powertrain.h pt_mode_q): its worth of a second against Normal's, from the two weights of every mode. The roll's entry, the lateness and everything new read this one place.
static float mode_q_for(int mode)   // 0 Normal, 1 Eco, 2 Power (mode_of's order)
{
    const float fw = mode == MODE_ECO ? p_fuel_eco : mode == MODE_POWER ? p_fuel_pw : p_fuel, rw = mode == MODE_ECO ? p_resp_eco : mode == MODE_POWER ? p_resp_pw : p_resp;
    return pt_mode_q(fw, rw, p_fuel, p_resp);
}
float ts_mode_q(int mode) { return mode_q_for(mode); }
static float roll_entry_s_for(int mode)   // 0 Normal, 1 Eco, 2 Power (mode_of's order)
{
    return GC_HORIZON * pt_mode_tau(mode_q_for(mode));   // (the mode's share of time: 1 / (1 + rate_m / rate_n) as it always was)
}
static float roll_entry_s() { return roll_entry_s_for(S.mode); }
float ts_roll_entry_s(int mode) { return roll_entry_s_for(mode); }
// The operating point the foot asked of the engine at the last layers decision (rpm, 0 = none) and how far into the kickdown it was: read-only, for the sims' rows (they judge the box against what the code
// asked, not against a mirror of the formula).
float ts_target_rpm(float *kd) { if (kd) *kd = S.tgt_kd; return S.tgt_rpm; }
// The one place the box takes the navigation route (the point queries: route_grade, route_crest_m, route_max_grade): the key [trueshift] lookahead AND a valid route. The box is good on its
// own (owner 09-11: pedal, revs, load, the grade under the truck, its own acceleration and lateral load); the route is a layer on top that comes back with PCC, and with the key at 0 (the default)
// none of it is read: no crest, no top, no route-confirmed roll, no climb forecast for the cruise, no planned neutral (ts_neutral_request), and the road view of the layers is that of a truck with
// no route (road_ahead). The grade of now carried on (grade_ahead_pct) is the box's own and stays.
static int route_on() { return p_lookahead > 0 && route_ok(); }

// Is a roll in neutral cheaper than staying in gear over the stretch ahead? Both ways are followed for 30 s in half seconds along the route's
// grade (else the grade of now) with the fuel model the truck has learned (core/vehicle.cpp: the game burns a x omega x (torque + T_f),
// T_f = fuel_k x peak torque; in gear with the pedal off nothing while the engine drags at fuel_kd x peak torque):
//   in gear   the cruise holds its set speed (a throttle just open on a gentle descent: no drag, the friction paid), or the truck coasts on the
//             engine's drag where the descent gives more than that (and where there is no cruise, the pedal being off);
//   neutral   idle (omega_idle x T_f) and no drag; the roll ends at the band's bottom (set - under) and the gear comes back in (a shift at idle,
//             the speed lost in it), then the speed is held. Without cruise the neutral way has that end too: the band's bottom is under the speed the driver
//             last held (v_use), because a driver whose speed has fallen that far is back on the pedal, and any throttle in neutral engages (WP-G6c: the
//             roll that the model saw lasting 30 s, 79.8 km/h on -1.0 % under the 80.3 held, ended at the first touch of the pedal half a second later:
//             B3d 11>0>12 in 0.5 s, three times, and B3's 1 s roll; the gain was +0.21, with the band it is -0.21). The gear way stays the foot off (a coast on
//             the drag, the debt paid at the end): a one-sided driver model. The cruise-consistent one, the gear way back on the pedal at the floor too, measured
//             worse (B3d 8 pairs in 31 shifts, run.sh 1063 / 92 against 0 in 20 and 1039 / 76), so the asymmetry is the measured choice, not a derivation.
// The speed each way ends with counts: the one that ends slower pays to make the difference back at 1 / efficiency per joule (the torque
// part of the fuel), up to v_use, the speed the driver wants (roll_use: the cruise's set speed, else what the driver last held):
// speed over it is worth nothing (it goes back to the air on the way down to it, or to the brakes over the allowed speed). On the way the
// speed may build up to the allowed speed (Damien 29-10: a moderate descent may carry speed into what follows, within the 30 s). A roll
// that would need the brakes to stay under the allowed speed is no roll (a descent wants the engine's drag). The game's fuel scale (a) drops out, so one decision fits any map and
// any truck; the learned k / kd move it (a high friction: idle is dear, roll only where the gear's drag costs more).
// Returns the saving of the roll as a share of the dearer way (> 0: roll); NAN when it cannot tell (no gear, no engine data).
// The grade `d` m on when no route says (none, or it ends): the grade of now, carried on at the rate it has had over the last two seconds for the time a roll needs to pay for its two
// shifts (out and back in, each a shift time and the latency), and the same after that. A descent that steepens or a hill that levels is seen for as long as it matters to the roll: the
// roll that started where the coast held the speed by a hair and ended half a second later, when the grade had moved a tenth of a percent (B3d: 11>0>10 in 1.5 s, six times), is not one.
static float grade_ahead_pct(float d, float v)
{
    float rate = (grade_pct() - S.grade_hist[2]) / (float)(2.0 + fmax(0.0, sim_s() - S.grade_at));
    if (!(fabsf(rate) < 100.0f)) rate = 0;
    return grade_pct() + rate * fminf(d / fmaxf(v, 1.0f), 2.0f * (V.eng.shift_time + GC_LATENCY_S));
}
static float roll_gain(int g, float v, int cc, float set, float under, float allowed, float v_use, int rolling, float *band_s = NULL)
{
    if (band_s) *band_s = 0;   // (the seconds the neutral way stays above the band's bottom; 0 when the model cannot tell or the roll would need the brakes)
    if (g < 1 || !V.eng_ok || !(V.mass > 0) || !(V.eta > 0)) return NAN;
    const float m = V.mass, tmax = V.eng.torque, tf = fuel_k() * tmax, kd = fuel_kd(), w_i = idle_rpm() * 0.10472f, ts = V.eng.shift_time;
    const float dt = 0.5f, H = 30;
    const int rk = route_on();
    auto grade_at = [&](float d) { float gp = rk ? route_grade(d, d + fmaxf(25.0f, v * dt)) : NAN; return gp == gp ? gp : grade_ahead_pct(d, v); };
    // with cruise the speed allowed on the way: where the cruise will brake there (it looks ~10 s ahead for a descent); else the allowed speed
    auto allowed_at = [&](float d, float vv) {
        if (!cc) return allowed;
        float h = cruise_hold_kmh(fminf(grade_at(d), grade_at(d + vv * 10)));
        return h == h && h >= 0 ? fminf(allowed, set + h / 3.6f) : allowed;
    };
    // in gear
    float vg = v, sg = 0, fg = 0;
    for (float t = 0; t < H; t += dt) {
        float R = resist_grade(vg, grade_at(sg)), full = force_at(g, vg, 1.0f), rpm = rpm_at(g, vg);
        if (!(full > 1)) return NAN;
        float drag = kd * (rpm / fuel_kd_rpm()) * full / fmaxf(0.05f, curve_factor(&V.eng, rpm) * V.f_scale), a;   // (the drag grows with the rpm)
        if (cc && R > -drag && vg <= set + 0.1f) {   // the cruise holds (a throttle just open stops the drag)
            float share = fmaxf(0.0f, R) / full;
            fg += rpm * 0.10472f * (fminf(1.0f, share) * tmax * curve_factor(&V.eng, rpm) + tf) * dt;
            a = share > 1 ? (full - R) / m : 0;
        } else a = (-R - drag) / m;                  // coasting in gear: no fuel, the engine drags
        vg = fminf(fmaxf(0.5f, vg + a * dt), allowed_at(sg, vg));
        sg += vg * dt;
    }
    // in neutral (rolling = already in neutral: no shift out; ending it now is what the gear way above costs, plus the shift back in)
    float vn = v, sn = 0, fn = 0, e_road = 0, v_end = NAN;
    int in_gear = 0;
    for (float t = 0; t < H; t += dt) {
        float gp = grade_at(sn), R = resist_grade(vn, gp), a;
        if (!in_gear) {
            fn += w_i * tf * dt;
            e_road -= m * 9.81f * gp / 100 * vn * dt;   // what the road gives the roll (down) or takes from it (up)
            a = -R / m;
            if (a > 0 && vn + a * dt > allowed_at(sn, vn)) return -1;   // it would need the brakes: no roll (a roll that starts over it and slows is not braked)
            // (the band is the cruise's own, eco_roll_under_*: the speed deficit at which a controller gives the gear back. The driver is that controller with the speed he last
            // held for the set speed, and the same speed is already what the make-up energy below is counted up to; no new number)
            if (vn + a * dt < (cc ? set : v_use) - under) { in_gear = 1; v_end = vn; if (band_s) *band_s = t + dt; fn += w_i * tf * ts; vn = fmaxf(0.5f, vn - R / m * ts); }
        } else {
            float full = force_at(g, vn, 1.0f), rpm = rpm_at(g, vn), share = full > 1 ? fmaxf(0.0f, R) / full : 1;
            fn += rpm * 0.10472f * (fminf(1.0f, share) * tmax * curve_factor(&V.eng, rpm) + tf) * dt;
            a = share > 1 ? (full - R) / m : 0;
        }
        vn = fmaxf(0.5f, vn + a * dt);
        sn += vn * dt;
    }
    if (!in_gear) { fn += w_i * tf * ts; v_end = vn; if (band_s) *band_s = H; }   // (the shift back in at the end of the roll; the band held for the whole look)
    // With cruise a new roll only where the road pays for it: the energy the grade gives over the roll at least the speed it costs. On the
    // flat the roll lives on speed the cruise has just put in (and puts back after it): gas, roll, gas, roll, a few % cheaper in the game's
    // fuel model but restless (Damien 29-10: "gas/rol/gas/rol is best irritant"; PCC sim: a roll every 11-25 s on a flat road)
    if (cc && !rolling && e_road < 0.5f * m * (v * v - v_end * v_end)) return -1;
    if (rolling) fg += w_i * tf * ts;                // (already rolling: the gear way starts with that shift)
    // the speed each way ends with: the slower one pays to make the difference back (never over the allowed speed)
    float ve = fminf(fmaxf(vg, vn), fminf(allowed, v_use));
    float cg = fg + 0.5f * m * fmaxf(0.0f, ve * ve - vg * vg) / V.eta, cn = fn + 0.5f * m * fmaxf(0.0f, ve * ve - vn * vn) / V.eta;
    return (cg - cn) / fmaxf(1.0f, fmaxf(cg, cn));   // (a gear way that costs nothing, coasting down a descent: any roll loses)
}

// ---------------------------------------------------------------- neutral on request (the longitudinal planner)
// With the planner driving (modules/planner.cpp) the planner decides WHEN to coast in neutral (it plans over 2 km with the fitted
// fuel model, the lead, the bends and the limits); TrueShift carries it out and keeps its safety veto. While a request is fresh
// (refreshed within NQ_TTL) it replaces roll_wanted's economy (the set-relative band, roll_gain, the boost, the pause between rolls,
// Power's route rule); want 0 means "no neutral" and ends a roll of TrueShift's own as well. A roll that a request started or kept
// ends when the request expires. Without requests nothing here runs: roll_wanted is today's, bit for bit.

static int nq_fresh() { double age = sim_s() - s_nq.at; return age >= 0 && age <= NQ_TTL; }

// roll_gain's neutral way alone, followed for `horizon` s: 1 when the coast passes the allowed speed (the same grade along the route,
// the same allowed speed with the cruise's own braking point there). Such a descent wants the engine's drag: neutral would need the brakes.
static int roll_needs_brakes(float v, int cc, float set, float allowed, float horizon)
{
    if (!(V.mass > 0)) return 0;
    const float m = V.mass, dt = 0.5f;
    const int rk = route_on();
    auto grade_at = [&](float d) { float gp = rk ? route_grade(d, d + fmaxf(25.0f, v * dt)) : NAN; return gp == gp ? gp : grade_ahead_pct(d, v); };
    auto allowed_at = [&](float d, float vv) {
        if (!cc) return allowed;
        float h = cruise_hold_kmh(fminf(grade_at(d), grade_at(d + vv * 10)));
        return h == h && h >= 0 ? fminf(allowed, set + h / 3.6f) : allowed;
    };
    float vn = v, sn = 0;
    for (float t = 0; t < horizon; t += dt) {
        float a = -resist_grade(vn, grade_at(sn)) / m;
        if (a > 0 && vn + a * dt > allowed_at(sn, vn)) return 1;   // (a roll that starts over it and slows is not braked)
        vn = fmaxf(0.5f, vn + a * dt);
        sn += vn * dt;
    }
    return 0;
}

// Why TrueShift must not or cannot be in neutral now, whoever asks; NULL = it may. The safety rules of roll_wanted and decide_gear,
// none of the economy: what the brakes, the driver, the road ahead, the air and the gearbox say.
static const char *roll_veto(float v, int ebrake, float brake, double pedal_free_s)
{
    const int cc = T.cruise > 0.5f;
    const double now = sim_s();
    if (!s_tail_ok) return "no neutral hook";
    if (!V.eng_ok || V.electric) return "no engine data";
    if (cc ? p_roll_cruise <= 0 : p_roll <= 0) return cc ? "eco_roll_cruise off" : "eco_roll off";
    if (S.mode == MODE_M) return "M mode";
    if (S.mode != MODE_ECO && S.mode != MODE_NORMAL && S.mode != MODE_POWER) return "mode";
    // under the speed where TrueShift surely has the truck: the exit gear must go in before the game's automatic takes over again
    if (!S.ours || g_lv.crawl_toggle || low_launch_hold() || v * 3.6f < 20.0f) return "low speed";   // (the eco-roll is a cruising thing: not under 20 km/h)
    if (cc && !g_cruise_smooth) return "game cruise";   // the game's cruise would rev the engine in neutral
    if (brake > 0.02f) return "brake";
    if (ebrake) return "engine brake or retarder";
    if (S.rolling ? T.in_thr > 0.1f : (T.in_thr > 0.02f || (!cc && T.eff_thr > 0.02f))) return "driver throttle";
    if (!S.rolling && pedal_free_s < 1.5) return "pedal just released";
    if (S.kick_until > now || S.wk_until > now) return "kickdown";
    if (S.hold_until > now) return "hint gear held";
    // the engine-driven compressor charges at idle only slowly: below the charge edge the rpm stays up (decide_gear's air rule)
    float air = air_edge_psi(p_air_psi, AIR_MARGIN_CHARGE);
    if (T.air_warning || (air > 0 && T.air_psi > 1 && T.air_psi < air)) return "brake air low";
    const float allowed = roll_allowed(cc, v);
    if (S.rolling && v > allowed) return "over the allowed speed";
    // the descent over the time the planner still wants neutral, plus the way back into gear
    float left = s_nq.hold_s - (float)(now - s_nq.at), horizon = fminf(30.0f, fmaxf(3.0f, fmaxf(0.0f, left) + V.eng.shift_time + 1.5f));
    if (roll_needs_brakes(v, cc, T.cruise, allowed, horizon)) return "steep descent";
    if (!S.rolling) {
        if (S.roll_exit > 0) return "shift in progress";   // the last roll's gear is still going in
        if (now - S.shift_at < 1) return "shift in progress";
        int g = T.gear;
        if (g < 1) return "not in gear";
        float r = rpm_at(g, v);
        if (r > V.eng.rpm_limit * 0.96f || (r < idle_rpm() + ioff(120) && g > 1)) return "gear protection";   // decide_gear's rev limit / stall rules act first
    }
    return NULL;
}

// roll_wanted while a request is fresh, or while a roll the request started or kept is on.
static int roll_requested(float v, int ebrake, float brake, double pedal_free_s)
{
    if (!nq_fresh()) {   // the planner stopped asking: back into gear
        if (S.rolling) logline("neutral request expired (no refresh for %.1f s): back into gear at %.0f km/h", sim_s() - s_nq.at, v * 3.6f);
        s_nq.roll = 0;
        s_nq.no = NULL;
        return 0;
    }
    const char *no = s_nq.want ? roll_veto(v, ebrake, brake, pedal_free_s) : NULL;
    if (no != s_nq.no && no) logv(1, "neutral request %s: %s (%.0f km/h, gear %d)", S.rolling ? "ended" : "refused", no, v * 3.6f, T.gear);
    s_nq.no = no;
    if (!s_nq.want || no) return 0;
    s_nq.roll = 1;
    return 1;
}

// Neutral on gentle stretches with the pedal released (deliberately: 1.5 s),
// Eco, Normal and Power (Power only with a route that confirms the roll), never with the brake, engine brake or retarder. Ends
// with real pedal (> 10 %), any brake, or when the speed leaves its band:
// without cruise the start speed +6/-12 km/h, with cruise the set speed +3/-5.
// The route (if any) vetoes rolling into a steep descent (the grade ahead; not a junction or a bend: those are the speed choosers'). With cruise only
// when the Cruise module drives the throttle (else the game's cruise would rev
// the engine in neutral); the cruise then gives no throttle while rolling.
static int roll_wanted(float v, float f_res, int ebrake, float brake, double pedal_free_s)
{
    S.boost_want = 0;
    if (nq_fresh() || (s_nq.roll && S.rolling)) return roll_requested(v, ebrake, brake, pedal_free_s);   // the planner decides (no boost)
    float kmh = v * 3.6f, set = T.cruise * 3.6f;
    int cc = T.cruise > 0.5f;
    // with cruise only under predictive cruise (Damien 25-10): a roll needs the route to know how long it lasts; the plain
    // eco_roll switch is not needed for it (eco_roll_cruise is its own option)
    int cc_ok = cc && p_roll_cruise > 0 && g_cruise_smooth && g_pcc_on && route_on();
    if ((p_roll <= 0 && !cc_ok) || (S.mode != MODE_ECO && S.mode != MODE_NORMAL && S.mode != MODE_POWER) || !S.ours) return 0;
    // a little gas before the roll (Damien 24-10): with cruise the roll may start up to roll_boost over the set speed, and the
    // cruise control builds that speed first; not in Power, not when the set speed is a speed limit being eased down
    float boost = cc && g_cruise_smooth && S.mode != MODE_POWER && !S.rolling ? fminf(p_roll_boost * band_scale(T.cruise), overspeed_room(T.cruise)) : 0;
    if ((cc && !cc_ok) || brake > 0.02f || ebrake) return 0;
    if (S.rolling ? T.in_thr > 0.1f : (T.in_thr > 0.02f || (!cc && T.eff_thr > 0.02f) || pedal_free_s < 1.5)) return 0;
    // Damien: at most 1-2 km/h under the set speed, then back on the gas; leave a little early (re-engaging
    // takes ~1 s) using the current deceleration
    float kmh_soon = kmh + fminf(0, T.accel) * 3.6f * 1.0f;
    // how far under the set speed it may roll: Eco most, Power least (Damien 11-10: between 1 and 2 km/h)
    // (the band scales with the speed it is under: the set speed, or without cruise the speed the driver held, as roll_gain reads it)
    float under = (S.mode == MODE_ECO ? p_roll_under_eco : S.mode == MODE_POWER ? p_roll_under_pw : p_roll_under) * band_scale(cc ? T.cruise : roll_use(cc));
    const float allowed = roll_allowed(cc, v);
    if (cc && (S.rolling ? (v > allowed || kmh_soon < set - under) : (kmh > set + 2 + boost || kmh < set - 0.5f))) return 0;
    int rk = route_on();
    if (!S.rolling) {
        bool eco = S.mode == MODE_ECO;  // (a shorter pause between rolls in Eco)
        if (sim_s() - S.roll_at < (eco ? 5 : 10) || sim_s() - S.shift_at < 1) return 0;
        // Power: only with a route that confirms the roll (Damien 25-10: Power is for pulling, not for rolling)
        if (S.mode == MODE_POWER && p_mode_char > 0 && !rk) return 0;
        // A roll where it saves fuel (roll_gain: the learned fuel model against the gear's drag, the speed made back, the allowed speed),
        // from the speed it would start at (with the boost: once the cruise has built it). This replaced eco_roll_min_kmh, eco_roll_max_decel,
        // a fixed 0.075 m/s2 ceiling, the 20 s minimum and the -3.5 % veto (Damien 29-10: no fixed limits; sim: the best minimum speed ran
        // from 40 km/h at a low engine friction to 80 at a high one, which the model gives by itself)
        float v0 = boost > 0 ? fmaxf(v, (set + boost) / 3.6f) : v;
        if (!cc && roll_fades(v)) return 0;   // (no roll that would end where it starts)
        float band_s;
        float gain = roll_gain(T.gear, v0, cc, T.cruise, under / 3.6f, allowed, roll_use(cc), 0, &band_s);
        if (!(gain > 0)) return 0;
        // Without cruise nobody but the driver ends the roll (decision 67), so it starts only where it is a sure thing for a while: the neutral way stays above the bottom of the driver's
        // band for the time the mode asks (roll_entry_s). With cruise the roll is PCC's, ends with its route, and the entry is as it was.
        if (!cc) {
            static double said = -1e9;
            const float need = roll_entry_s();
            const int ok = band_s >= need;
            if ((ok || band_s > 0) && (sim_s() - said >= 2.0 || sim_s() < said)) {
                said = sim_s();
                logv(2, "eco-roll %s: gain %.2f, the coast stays above the band's bottom (%.1f km/h) for %.1f s, this mode asks %.1f s", ok ? "may start" : "not started", gain, (roll_use(cc) - under / 3.6f) * 3.6f, band_s, need);
            }
            if (!ok) return 0;
        }
        // everything fits but the speed: build the boost first, the roll starts when it is there
        if (boost > 0 && kmh < set + boost - 0.5f) { S.boost_want = 1; return 0; }
        return 1;
    }
    // rolling: on while it still saves against going back into gear now (that shift counted), and under the allowed speed
    if (v > allowed) return 0;
    // Without cruise the roll is the driver's and lasts until the DRIVER does something (owner decision 67, 04-11: "als de bak zelf voor eco-roll kiest zonder navi-data, dan moet de
    // chauffeur hem er weer uit halen"; before: "zonder CC gewoon tot je zelf weer gas geeft", 29-10). The pedal (> 0.1), the brake, the engine brake or retarder end it above; here the
    // rest of what the driver does: the mode knob, a +/- press, the cruise switched off (a roll that PCC started is PCC's: with the cruise gone nobody asked for it). The cruise switched
    // ON is the cruise path (cc below). No model ends it any more.
    if (!cc) {
        const char *why = S.roll_cc ? "the cruise was switched off" : S.mode != S.roll_mode ? "the mode was changed" : NULL;   // (a +/- press: not yet, see tail_pre)
        if (why) { logv(2, "eco-roll ended by the driver: %s (%.0f km/h)", why, v * 3.6f); return 0; }
        // ... a roll that has faded to a crawl (a rise, nobody acting): the exit gear is a launch gear, the stop path takes the truck (roll_fades; a proposal, WP-ER1 item 5)
        if (roll_fades(v)) { logv(1, "eco-roll ended: the speed has faded to %.0f km/h, the exit gear is a launch gear (the stop path takes the truck)", v * 3.6f); return 0; }
        // ... and the one protection: really running away (a descent the truck gathers speed on in neutral up to the limit). No forecast of 30 s ends a roll any more, no gain
        return !roll_runaway(v);
    }
    float gain = roll_gain(S.roll_exit > 0 ? S.roll_exit : roll_exit_gear(v, 0, 0), v, cc, T.cruise, under / 3.6f, allowed, roll_use(cc), 1);
    return gain > 0;   // (the model cannot tell: back into gear)
}

// A rule's request for the gear (core/gearreq.h): the gear it wants, why, and the priority of its kind
static void ask(int gear, int why, int prio) { const gear_req r = { gear, why, prio }; gear_req_post(&r); }

// M: the gear stays where the driver's shift hints put it. m_style (Damien 17-10, after the manuals of Volvo I-Shift,
// Mercedes PowerShift and Eaton UltraShift): 0 European = no shift of our own (overspeed_tone warns instead),
// 1 protected = rev limiter and stall only (Eaton), 2 assisted = also kickdown, stopping and lugging (like a VW DSG).
// Below the low-speed handover the game picks the gear in every style (the start gear after a stop).
// The gear is the driver's (GP_OWNER); the rev limiter and the stall rule of the protected styles are protections (GP_PROTECT).
static void decide_manual(int cur, float rpm, float v, double since)
{
    int n = n_gears(), ebrake = T.motor_brake || T.retarder > 0 || S.eb_level > 0.01f;
    double now = sim_s();
    float brake = fmaxf(T.in_brake, T.eff_brake);
    if (p_m_style < 1) { ask(cur, W_M, GP_OWNER); return; }
    if (rpm > rev_line(ebrake) && cur < n) { ask(cur + 1, W_M_REV_LIMIT, GP_PROTECT); return; }
    if (rpm < stall_line() && cur > 1 && !tc_relief(cur, v, S.f_need)) { ask(pick_down(cur, v, eco_dn(1) + 300), W_M_STALL, GP_PROTECT); return; }
    if (p_m_style < 2) { ask(cur, W_M, GP_OWNER); return; }
    if (S.kick_until > now && rpm < pw_lo() + 100 && cur > 1 && since > 0.3) {
        int l = pick_down(cur, v, pw_mid() + 100);
        if (l < cur) { ask(l, W_M_KICKDOWN, GP_OWNER); return; }
    }
    if (brake > 0.05f && rpm < idle_rpm() + ioff(250) && cur > 1 && since > 0.5) {
        int l = pick_down(cur, v, eco_up(1) - 200);
        if (l < cur) { ask(l, W_M_STOPPING, GP_OWNER); return; }
    }
    float fmax = force_at(cur, v, 1.0f);
    if (S.thr > 0.8f && rpm < rs_abs(idle_rpm(), p_m_lug) && cur > 1 && since > 2 &&
        (T.accel < 0.05f || fmax - resist(v) < V.mass * 0.03f)) {
        ask(cur - 1, W_M_LUGGING, GP_OWNER);
        return;
    }
    ask(cur, W_M, GP_OWNER);
}

// M: the gear the automatic would take now, for the shift advice on the dashboard (Damien 28-10: "like VW, 4 -> 6", no sound). Read-only:
// the shift points and the vehicle model of now, no timers and no state, a simplified copy of the paths below (the engine brake and braking
// downshift, the downshift and upshift on rpm, the early upshift under a light steady pull). 0 = none (the gear fits, or nothing to say).
static int m_advice(int cur, float v)
{
    int n = n_gears();
    if (S.mode != MODE_M || !S.ours || S.rolling || cur < 1 || cur > n || !V.eng_ok || V.mass < 1000) return 0;
    float rpm = rpm_at(cur, v), f_res = resist(v), thr = S.thr, brake = fmaxf(T.in_brake, T.eff_brake);
    float grade = sinf(V.grade_deg * 0.0174533f);
    if ((T.motor_brake || T.retarder > 0) && thr < 0.05f) {   // engine brake: keep the revs in its range
        float lo = V.eng.range[R_BRAKE][0], hi = V.eng.range[R_BRAKE][1];
        if (!T.motor_brake || rpm >= lo - 50 || cur < 2) return 0;
        int l = pick_down(cur, v, 0.5f * (lo + hi), true);
        return l < cur && rpm_at(l, v) < hi ? l : 0;
    }
    if (brake > 0.05f) {   // braking: stay above the floor, land high
        if (cur < 2 || rpm >= fmaxf(idle_rpm() + ioff(250), eco_dn(cur) - 100)) return 0;
        int l = pick_down(cur, v, eco_up(1) - 200, true);
        return l < cur ? l : 0;
    }
    if (thr > 0.05f && cur < n && rpm > S.up_pt) return skip_target(cur, v, f_res, grade);
    if (thr > 0.3f && cur > 1 && rpm < S.dn_pt) {
        int l = pick_down(cur, v, lerpf(0.5f * (eco_dn(cur - 1) + eco_up(cur - 1)) + 100, pw_mid(), pw_k()));
        return l < cur ? l : 0;
    }
    // light steady pull: the highest gear the early upshift would take (lands above its floor, keeps the reserve)
    if (thr >= 0.15f && cur < n && S.demand < p_power_from && T.accel > -0.15f && v * 3.6f >= K_ADVICE_MIN_KMH + 5) {
        float v2 = fmaxf(0.5f, v - f_res / V.mass * V.eng.shift_time);
        float need = fmaxf(S.f_need, f_res), k = mode_reserve() + learned().res + (grade_pct() > -0.5f ? 0.1f : 0.0f);
        int best = cur;
        for (int h = cur + 1; h <= n && h <= cur + 1 + skip_allowed(); h++) {
            if (drv_at(h, v2) < cruise_floor(h, h > cur + 1 ? 100.0f : 0.0f)) break;
            if (force_at(h, v2, 1.0f) < k * need) break;
            if (p_load_from < 1 && grade > 0.01f && need > p_load_from * force_at(h, v2, 1.0f)) break;
            if (h > cur + 1 && (grade > 0.02f || (force_at(h, v2, 0.85f) - f_res) / V.mass < 0)) break;
            best = h;
        }
        if (best > cur) return best;
    }
    return 0;
}

// Returns the gear TrueShift wants; *why says why. 0 = leave it to the game.
// PCC-like top (Damien 11-10): a top within reach that this gear gets over at full throttle while losing at most
// crest_band_kmh below the reference speed (the set speed with cruise, else the speed when the top came into view)
// and without leaving the torque plateau: no downshift for it. Energy over the climb: v_end^2 = v^2 + 2 d (F - R) / m.
// The band under the set speed (m/s): full at 40 t, 60 % solo, 60 % in Power.
static float crest_band()
{
    float k = fminf(1.0f, fmaxf(0.0f, (V.mass - 12000) / 28000));
    return p_crest_band > 0 ? p_crest_band * (0.6f + 0.4f * k) * (S.mode == MODE_POWER ? 0.6f : 1.0f) / 3.6f : 0;
}
// Lowest rpm the band may pull the gear down to: just under the torque plateau.
static float band_rpm() { return fmaxf(V.plateau - 150, idle_rpm() + 200); }

static int top_reachable(int cur, float v, float *top_m, float *floor_kmh)
{
    *top_m = -1;
    float c = p_crest_band > 0 && route_on() && V.mass > 500 ? route_crest_m(fmaxf(400, v * 25)) : -1;
    // a false flat is no top: the road must not climb again right after it
    // (the reference speed is forgotten only after 3 s without a top: one tick without one must not lower it)
    double now = sim_s();
    if (c < 0 || route_max_grade(c, c + fmaxf(150, v * 8)) > 1.0f) {
        if (now - S.crest_seen_at > 3 || now < S.crest_seen_at) { S.crest_ref = 0; S.crest_said = 0; }
        return 0;
    }
    S.crest_seen_at = now;
    if (!(S.crest_ref > 0)) S.crest_ref = v;
    *top_m = c;
    float ref = T.cruise > 0.5f ? T.cruise : S.crest_ref, per = fmaxf(1.0f, rpm_at(cur, 1.0f));  // rpm per m/s
    float v_floor = fmaxf(ref - crest_band(), band_rpm() / per);
    *floor_kmh = v_floor * 3.6f;
    float g = route_grade(0, c);
    if (v <= v_floor || !(g == g)) return 0;
    float f = fminf(force_at(cur, v, 1.0f), force_at(cur, v_floor, 1.0f)), r = resist_grade(v, g);
    return v * v + 2 * c * (f - r) / V.mass >= v_floor * v_floor;
}

// PCC momentum (Damien 11-10: only what is needed; solo differs from loaded): the extra speed over the set speed at
// the foot of the next climb on the route so that the truck gets over its top without dropping more than 1 km/h under
// the set speed, at full throttle in the highest gear that is still on the plateau at the set speed. 0 = no climb ahead
// or it holds the speed anyway; capped at 8 (cruise caps at band_up_kmh); NAN = unknown (no route, no cruise, no engine).
static float climb_need_kmh()
{
    float set = T.cruise;
    if (!(set > 5) || !route_on() || !V.eng_ok || V.mass < 500) return NAN;
    int g = 0;
    for (int i = n_gears(); i >= 1; i--) if (rpm_at(i, set) >= band_rpm()) { g = i; break; }
    if (!g) return NAN;
    float foot = -1;
    for (float d = 0; d <= 1200; d += 25) { float gr = route_grade(d, d + 50); if (gr == gr && gr > 1.0f) { foot = d; break; } }
    if (foot < 0) return 0;
    for (float dv = 0; dv <= 8; dv += 0.5f) {
        float v = set + dv / 3.6f, top = v;
        int ok = 1;
        for (float x = foot; x < foot + 1500; x += 25) {
            float gr = route_grade(x, x + 25);
            if (!(gr == gr)) break;
            if (x > foot + 50 && route_grade(x, x + 50) < 0.3f) break;  // over the top
            float a = (force_at(g, v, 1.0f) - resist_grade(v, gr)) / V.mass;
            v = sqrtf(fmaxf(1.0f, v * v + 2 * 25 * a));
            if (v > top) v = top;  // the cruise does not run away above the speed it started with
            if (v < set - 1 / 3.6f || rpm_at(g, v) < band_rpm()) { ok = 0; break; }
        }
        if (ok) return dv;
    }
    return 8;
}


// The deceleration assist's downshift (see decide_gear): the lowest gear (one or two below cur) that lands, at the speed the shift ends at, inside the
// engine's brake range (rpm from its lower end up to the cap: a fifth of the range's width under the top, and 88 % of the rpm limit);
// cur if none does. *cap = that cap (rpm).
// The landing must also survive the road (WP-FX1, ride 04-11, Normal on -9 .. -14 %: the assist went down to 1800 - 2000 rpm at lever steps 1 - 4, the driver let the lever go 1 - 2 s later, the
// truck ran away from the retarder and the rev-limit rule shifted it up again 2 - 3 s after the downshift: 5>4>5>4 at 825.3 s, 4>5>4>5 at 827.6 s, 5>6>5>6 at 845 s). The cap above is
// judged at the speed of the shift's end with the deceleration of NOW carried through it (v_e), which is the lever's: nothing says the lever stays. The layers' own landing test is the road
// alone: the speed the shift ends at is v less (f_res / m + the shift's measured loss) over the shift (`v2` in gear_layers), and the gear must sit under the rev-limit rule's line there. Here the
// same, carried on over the dwell: after a downshift the layers hold no upshift for dwell_down_up_s, and a rev-limit upshift inside it is the downshift taken back, so the road is left to push for
// the dwell as well (`push` = the free acceleration of a net descent, 0 where the road does not push) before the line is read. A gear whose revs would cross the rev line in that time is not
// taken (the lever's retarder and the service brake still carry the truck in the gear it is in; the next lower gear lands higher and fails sooner, so none below it is tried).
static float sl_dur_est();
static float sl_dec_est();
static int assist_down(int cur, float v, float *cap)
{
    float lo = V.eng.range[R_BRAKE][0], hi = V.eng.range[R_BRAKE][1];
    *cap = fminf(hi - 0.2f * (hi - lo), V.eng.rpm_limit * 0.88f);   // (under the 0.9 of the limit where an upshift is vetoed while slowing: no flip back up)
    if (!(hi > lo) || !(*cap > lo)) return cur;
    // (the speed at the end of a shift of the length the engine says, not the longer one the braking rule takes to be on the low side:
    // here the top is what matters, and the clutch is open as long as that)
    float v_e = fmaxf(0.5f, v - fmaxf(0.0f, -T.accel) * V.eng.shift_time);
    const float m = V.mass > 0 ? V.mass : VEHICLE_MASS_FALLBACK, f_res = resist(v);
    const float dur = V.tc ? 0.3f : fmaxf(sl_dur_est(), V.eng.shift_time);
    const float v_shift = fmaxf(0.5f, v - (f_res / m + (V.tc ? 0.0f : sl_dec_est())) * dur);   // (the road alone through the shift: the layers' v2)
    const float v_h = v_shift + fmaxf(0.0f, -f_res / m) * dwell_du();                           // ... and the dwell after it
    int best = cur;
    for (int l = cur - 1; l >= 1 && l >= cur - (p_eb_skip > 0 ? 2 : 1); l--) {
        float r = rpm_at(l, v_e);
        if (r > *cap) break;
        if (rpm_at(l, v_h) > rev_line(0)) {
            static double said; const double now = sim_s();
            if (now - said > 5.0 || now < said) { said = now; logv(2, "braking assist %d->%d not taken: the road alone has it at %.0f km/h after the shift and the %.1f s dwell (%.0f rpm against the rev limit's line %.0f), lands %.0f rpm now", cur, l, v_h * 3.6f, dwell_du(), rpm_at(l, v_h), rev_line(0), r); }
            break;
        }
        if (r >= lo) best = l;
    }
    return best;
}

// The most force (N) any gear gives with its rpm inside the eco bands (P1 up to P2) at the speed v: what the eco
// band can deliver. No gear inside (the speed is too low or too high for the box): the gear nearest to the band.
static float eco_force_at(int n, float v, const eng_bands *b)
{
    float best = 0, near_f = 0, near_d = 1e30f;
    for (int g = 1; g <= n; g++) {
        float r = vehicle_engine_rpm_at(g, v);
        float f = vehicle_force_at(g, v, 1.0f);
        if (r >= b->p1 && r <= b->p2) best = fmaxf(best, f);
        float d = r < b->p1 ? b->p1 - r : r > b->p2 ? r - b->p2 : 0.0f;
        if (d < near_d) { near_d = d; near_f = f; }
    }
    return best > 0 ? best : near_f;
}

// The lateral load (core/gearchoice.h gc_in.lat_load), measured live from the telemetry, never from route data: with and without navigation the box
// behaves the same. The share of the truck's own bend allowance (g_bend_lat: Cruise's curve_lat, which follows what the roll learning measured of this
// truck and trailer, core/roll.h) the truck uses now or, by its steering, is about to use. The steering leads: a bend is entered with the wheel before
// the lateral acceleration builds, so the load is read as the larger of the measured lateral acceleration (v x yaw rate, the confirmation) and the one
// the steering predicts (gain x v^2 x steering). The gain is a regression through the origin of the measured on v^2 x steering, summed with the
// forgetting of the demand memory (DEMAND_MEM_S), and what is used is its lower bound (the slope less its standard error, as roll_fit_eval judges a
// bend), so with little evidence the steering predicts nothing and the measured acceleration decides alone. Only an analog wheel counts for the
// steering (a keyboard or pad tap sits at full lock, as Cruise's hold-the-gas rule has it).
static float lateral_load(float v, double now)
{
    const float st = fabsf(T.in_steer), a_meas = fabsf(v * T.yaw * 6.2831853f);
    const float budget = g_bend_lat > 0.3f ? (float)g_bend_lat : 3.125f;
    const float x = v * v * st;
    const double dt = now - S.lat_t;
    if (dt > 0 && dt < 0.5) {
        const float k = expf(-(float)dt / (float)DEMAND_MEM_S);
        S.lat_sxx *= k; S.lat_sxy *= k; S.lat_syy *= k; S.lat_w *= k;
        if (st < 0.95f) {
            const float w = (float)dt;
            S.lat_sxx += w * x * x; S.lat_sxy += w * x * a_meas; S.lat_syy += w * a_meas * a_meas; S.lat_w += w;
        }
    }
    S.lat_t = now;
    float lead = 0;
    if (st < 0.95f && S.lat_sxx > 1e-3f) {
        const float n_eff = S.lat_w / 0.25f;                   // one independent sample per 0.25 s (the body's response, core/rolllearn.cpp RL_TAU_A)
        const float slope = S.lat_sxy / S.lat_sxx;
        if (n_eff > 2.0f) {
            const float s2 = fmaxf(0.0f, S.lat_syy - slope * S.lat_sxy) / S.lat_w * (n_eff / (n_eff - 1.0f));
            const float se = sqrtf(s2 / (n_eff * S.lat_sxx / S.lat_w));
            lead = fmaxf(0.0f, slope - se) * x;
        }
    }
    return clampf(fmaxf(a_meas, lead) / budget, 0.0f, 2.0f);
}

// Is the eco-roll about to take over the lift? It owns lifting then: the gearbox does not shift on the way into neutral. roll_wanted has one side
// effect (the boost request), which is put back.
static int roll_soon(float v, float f_res, double now)
{
    const int bw = S.boost_want;
    const int ebrake = T.motor_brake || T.retarder > 0 || S.eb_level > 0.01f;
    const float brake = fmaxf(T.in_brake, T.eff_brake);
    const int r = S.rolling || roll_wanted(v, f_res, ebrake, brake, now - S.pedal_on_at) || S.boost_want;
    S.boost_want = bw;
    return r;
}

// The physics rule of decide_gear_rules below, for gear g at the speed of now: the gear cannot even hold the speed at full throttle, the speed falls and the
// rpm is in the lug zone (the rpm clause stays: without it the rule also fires at 1600 rpm on a 6 % climb where no gear holds the speed). The rev-limit rule asks
// it of the gear it would go up to: a gear the next tick takes off again is no protection.
static int lug_rule_fires(int g, float v, float f_res, float thr)
{
    const float fmax = force_at(g, v, 1.0f);
    const int cannot_hold = thr > 0.25f && fmax < 1.05f * f_res && T.accel < -0.1f;
    return cannot_hold && T.accel < 0 && fmax - f_res < V.mass * 0.03f && rpm_at(g, v) < pw_lo() + 100;
}

// The lugging downshift as a price (WP-1011-T6C). lug_rule_fires is a fact: at full throttle the gear gives less than the road takes. What that costs is a number of seconds. The truck is played
// forward on the road of now with the gear's own force at the speed it has by then (the force falls with the revs under the plateau, the resistance with the speed), and the drive lost is the
// speed under the one it holds now, over that speed, summed. How long the road holds on is the one assumption of the price and it is made twice, consistently: it holds on for the plan
// horizon (GC_HORIZON, the horizon every term of the layers is measured over) and for one more, as long again as the horizon the layers can see (the same prior as the hold's age in
// gear_layers); the shift back up, which a downshift to ride a climb is half of, is paid when it eases, and what is left of the deficit then is won back on the flat in the gear we are
// in, at the pull it has to spare. Staying is played the same way, so what the downshift buys is the difference. The shift costs what a shift costs in the score: its duration, the gap
// itself (a stretch without drive, played with the road's deceleration and the learned loss) and the duration of the shift back. A gear 330 N short on a 2.2 % grade sags 0.1 s, the
// shift and the way back cost 1.1 s; a climb no gear holds buys seconds. No threshold: the two meet where the sag costs more than the shift and the way back. The fuel of the lower revs
// is left out on purpose: this is the rule that protects the speed, the layers price the revs.
static float lug_sag_s(int g, int rec, float v0, float gap_s, float decel)
{
    const float m = V.mass > 0 ? V.mass : VEHICLE_MASS_FALLBACK, h = 0.25f;
    float v = v0, lost = 0;
    for (float t = 0; t < 2.0f * GC_HORIZON; t += h) {
        const float a = t < gap_s ? -(resist(v) / m + decel) : (force_at(g, v, 1.0f) - resist(v)) / m;
        v = fmaxf(0.5f, fminf(v + a * h, v0));   // (the cruise does not run away above the speed it holds)
        lost += (v0 - v) / v0 * h;
    }
    for (float t = 0; t < 60.0f && v < v0; t += h) {   // the road eases: the deficit left is won back on the flat in gear `rec`
        const float a = fmaxf(0.05f, (force_at(rec, v, 1.0f) - resist_grade(v, 0.0f)) / m);
        v = fminf(v + a * h, v0);
        lost += (v0 - v) / v0 * h;
    }
    return lost;
}
// the seconds the downshift cur > l gains (< 0: the sag costs less than the shift and the shift back)
static float lug_gain(int cur, int l, float v)
{
    const float dur = V.tc ? 0.3f : fmaxf(sl_dur_est(), V.eng.shift_time), decel = V.tc ? 0.0f : sl_dec_est();
    return lug_sag_s(cur, cur, v, 0.0f, 0.0f) - lug_sag_s(l, cur, v, dur, decel) - 2.0f * dur;
}
// the best lower gear's gain (a gear whose revs are over the limiter's line is no candidate)
static float lug_best_gain(int cur, float v)
{
    float best = -1e30f;
    for (int l = cur - 1; l >= 1 && rpm_at(l, v) <= V.eng.rpm_limit * 0.93f; l--) best = fmaxf(best, lug_gain(cur, l, v));
    return best;
}
// The cruise whose sag is a price: the game's own with the foot off, and without the route the plugin's cruise as well (with the route the box knows the top, and the descent after it: the fact stands)
static bool lug_priced(float v) { return T.cruise > 0.5f && T.in_thr <= 0.02f && (!route_on() || !(cruise_wish_force(v) >= 0.0f)); }

// The layered powertrain (core/powertrain.h). Every hard rule above already ran (rev limit, stall, braking,
// engine brake, kickdown, hints, dwell, crawl): what is left is the soft choice, now layers 2 and 3. The caller's hard limits
// become the allowed mask (the rev-limit / stall caps and the anti-hunt direction; gc_choose's own rpm window covers them too),
// the intent (layer 2) reads the wish, and gc_choose (layer 3) scores the allowed gears.
// The pedal off is not "no intent": the driver is still expected to ask what he asked a moment ago. The wish the layers get is that demand, decaying
// with the time since the lift with the shift's own duration as the time constant (a lift shorter than a shift cannot be used for one), so a shift
// right after the lift still costs the drive it would lose and a shift late in a held lift costs nothing: the cheap moment to shift, both ways. The
// engine brake, the brakes and the descent control returned earlier; the eco-roll is the other exception (it owns the lift).
static float sl_dur_est();
static float sl_dec_est();
static int descent_gear(int cur, int n, float v, float f_res, int engaged, char *terms = nullptr, size_t tn = 0);
static int drag_gear(int cur, int n, float v, float need_n, int engaged, int max_down, float v_land, const char *head, char *terms, size_t tn);
// The last decision of the layers, read-only for the hint votes (core/driver.h (e), owner point 20.2): the gear we were in, the gear chosen, when, and the terms of the
// candidates (the same text as the 'layers:' log line, cut to what fits a log line). A floored pedal or a hold has no scores: it says so.
static gc_out s_last_gc; static double s_last_gc_at = -1;

const gc_out *ts_last_choice(double *at) { if (at) *at = s_last_gc_at; return s_last_gc_at >= 0 ? &s_last_gc : nullptr; }
static char s_last_terms[208];
static double s_last_terms_at = -1;
static int s_last_terms_cur, s_last_terms_pick;
static int s_pick_idle;   // roll_exit_pick: the cruise's idle wish is the force that holds the speed (a roll's exit has to choose a gear)
// The longest age of a pedal hold the layers count (s). The one number of the pedal hold's band_scale that is measured and not derived (WP-G1b): the cruise's hold is capped at the
// driver model's window, 2 horizons = 40 s, and the pedal's was first; with 40 s the rolling-hill trap B3d (run.sh: 25 t seen as 14 t, 80 km/h on +-3 % hills with a swinging pedal)
// starts eco-rolls that end within a shift or two (6 hunting pairs) at Normal fuel weights 6 and 9 .. 12, with 30 s it is hunt-free at every weight from 5 to 12, and with 20 s the
// calm pedal's ramp is left at +23 rpm and +0.2 % fuel over the old box (Eco rampd, 16-pair subset) where 30 s gives -13 rpm.
static const float HOLD_AGE_MAX_S = 30.0f;
// The road the box reads (core/ahead.h ahead_now()), the same in the gear choice (the intent's hold_ahead_n, the floored plan's worst stretch, gc_expect's timed samples) and in the descent
// control. [trueshift] lookahead 0 (the default) makes it the view of a truck with no route: only the grade under the truck is known (no samples, certainty 0, the worst and the mean are the
// resistance of now). With route_on() the other place the route enters the box: the two are the one key.
static const ahead_view *road_ahead()
{
    ahead_use_route(p_lookahead > 0);   // the key goes in where the view is formed: off, the route is not read at all (core/ahead.cpp), not read and then hidden
    return ahead_now();
}

// The raw pedal whose shaped share is k times the share of `foot` in this mode (gc_in::foot_for).
static float ts_foot_for(float foot, float k) { return wish_unmap(fminf(1.0f, wish_map(foot, S.mode) * k), S.mode); }

static int gear_layers(int cur, int n, float v, float rpm, float thr, float f_res, int coast_p, double since, double now, int *why)
{
    const eng_bands *b = bands_now();
    if (!b->ok) { *why = W_HOLD; return cur; }
    const float ts = V.tc ? 0.3f : V.eng.shift_time;
    const float sl_dur = sl_dur_est(), sl_dec = sl_dec_est();
    const float dur = V.tc ? ts : fmaxf(sl_dur, ts);   // (as gc_choose reads it: a converter has no gap, its 0.3 s stays the estimate)
    // the wish (core/wish.h): the torque pedal's power wish, else the cruise's force wish, else the pedal's own demand in this gear (a plain
    // pedal, or a cruise whose force_mode is 0: the force that holds the speed / asks the pull), 0 while the pedal is off; with the wish filtered
    // over a few shift times (the need reads it) and the pedal's rate
    wish_t wsh;
    {
        const wish_args wi = { v, coast_p, 1, fmaxf(S.f_need, force_at(cur, v, thr)), now };
        wish_get(&wi, &S.ws, &S.ws_t, &wsh);
    }
    float wish = wsh.force_n, ws = wsh.slow_n;
    // A cruise that is below its set speed asks at least what holds the speed: its force wish is the controller's output, and that climbs from 0 for the seconds after it
    // is engaged (or after a bend, a lower limit, a descent) while the truck already needs the holding force. The gears were chosen for the ramp, a taller one that lugged
    // when the throttle came up and was left again (Renault 36 t on 4 %, 13 -> 14 -> 13 -> 12 in 3 s). At or above the set speed it may ease off, and then it asks less.
    if (wsh.source == WISH_CRUISE && T.cruise > 0.5f && v < T.cruise && wish > 0 && wish < f_res) { wish = f_res; ws = fmaxf(ws, f_res); }
    const float rate = driver_press_rate_now();   // (the press of now, in the unit the driver profile learns his presses in: core/driver.h (b))
    // the demand still expected: the largest wish of the pedal on, fading over the plan horizon (what was asked a while ago is worth less as a reserve)
    {
        const double dt = now - S.ex_t;
        if (dt < 0 || dt > 60) S.ex_pre = fmaxf(wish, 0.0f);
        else if (dt > 0) S.ex_pre = fmaxf(wish, S.ex_pre * expf(-(float)dt / GC_HORIZON));
        S.ex_t = now;
        if (wish > 0) { S.wish_on_t = now; S.ex_v = v; S.lift_done = S.lift_pick = 0; }
    }
    const int zero = !(wish > 0);
    // The pedal-off model is the driver's foot: a lift has habits (how long, how often it comes back) and a price. A controller has none: while the cruise drives,
    // its own force wish is the demand, also when it eases the throttle on a descent (wish 0), and that is the pedal-on path with that wish.
    // A roll's exit (s_pick_idle: the engage out of neutral, or TrueShift's own end of a roll) is no lift either: nothing is in gear, the coast that makes a lift cheap to leave is over and the
    // engine takes the push of the road, as the cruise's idle wish below has it. The lift model would price the gears for a demand that may come back, from a speed gathered in neutral
    // (WP-G6c round 2, B4: 12 at 917 rpm on 2.6 %, lugging 12 > 11 > 10 in two seconds, three shifts at the foot of every climb); the gear that holds the push is the one to take.
    const int drv_off = zero && !(T.cruise > 0.5f);
    const int off = drv_off && !s_pick_idle;
    if (zero) {
        if (roll_soon(v, f_res, now)) { *why = W_HOLD; return cur; }
        // On a net descent (the grade pushes the truck) the engine's drag is what holds the speed, and a shift up takes it away: the gear stays, the engine
        // brake rules above own the downshift. A DEVIATION from the owner's exception list (engine braking wanted, eco-roll): measured needed, run_pcc's desc5 60 t
        // fails without it (over the allowed speed 435 against 3 units: the descent control starts after the box has already gone up) even with the descent
        // control, the deceleration assist and the brake channel as exceptions. (Choosing the gear whose drag about holds the speed, and the least drag on the
        // flat, is package E's.)
        {   // (a held engine-brake stage or a wish for more engine drag stands in the arbiter: core/gearreq.h; the brakes' standing requests)
            gear_stand st;
            if (gear_stand_get(&st) && (st.eb_held || st.drag_n > 0)) { *why = W_HOLD; return cur; }
        }
        if (f_res <= 0) {
            // (E, owner point 19.4: the gear whose drag about holds the speed; B's hold of whatever gear was in is replaced)
            // (a lift is no surprise, P2: not before the lift has lasted a shift and its latency, as the layers' lift price has it; a short lift on a rolling road shifts nothing)
            // ... and only while the truck gains speed: the model says the drag does not hold the push, the truck says whether it does not (a gear that slows it needs no help)
            char dg_terms[200] = "";
            const int dg = drv_off && T.accel > 0 && now - fmax(S.wish_on_t, S.pedal_on_at) >= dur + GC_LATENCY_S && !(T.motor_brake || T.retarder > 0 || S.eb_level > 0.01f) ? descent_gear(cur, n, v, f_res, !s_pick_idle, dg_terms, sizeof dg_terms) : 0;
            if (dg) {
                // (P7: the pick names its terms as the layers' lines do, through the same prefix: promise P7 reads "layers: a->b ... terms: ...";
                // the first five arguments are the layers' line's (int, int, band, action, rpm), sim.cpp's log reads them by position)
                logv(1, "layers: %d->%d (%s band, %s, %.0f rpm; pedal off on a net descent, the gear whose drag holds the push) terms: %s", cur, dg,
                     gc_band_name(pt_band_of(b, rpm_at(dg, v), gc_cross(b, dg, n, v))), "drag", rpm_at(dg, v), dg_terms);
                *why = W_LAYERS; S.lift_pick = dg; return dg;
            }
            *why = W_HOLD; return cur;
        }
        // The cruise eases its throttle (wish 0): a controller, not a driver's foot, so no lift (no price for a shift now, no habit): it asks to hold the speed and the
        // gear that holds it stays until the cruise asks for something (run_pcc: 458 shifts with the lift model for this case, 528 when the layers chose on the holding
        // force, 459 holding the gear)
        if (!off) {
            if (!s_pick_idle) { *why = W_HOLD; return cur; }
            wish = fmaxf(f_res, 1.5f); ws = wish;   // (a roll's exit has to choose a gear: on the force that holds the speed, the cruise will ask at least that)
        }
    }
    if (off) {
        // One lift, one decision: what the layers price at a lift is the demand that may come back and its reserve; the coast that follows changes the speed, not
        // that demand, and following it with free shifts (up, down, up as the revs sag) is what the protections (the stall and the rev-limit rules) are for. The
        // decision is open again when the pedal returns.
        // A lift is a pedal that was on and went off: before any pedal of this session (or of the pause) nothing is expected back and nothing is lifted.
        if (S.wish_on_t < -1e8 && S.pedal_on_at < -1e8) { *why = W_HOLD; return cur; }
        if (S.lift_done) { *why = W_HOLD; return cur; }
        wish = fmaxf(S.ex_pre, 1.5f);   // (layer 2 takes a wish over 1 N as an intent)
        ws = wish;                        // the band wish follows the demand that is expected back
    }
    // A speed limiter holds the truck at its top speed whatever the pedal asks: the wish cannot accelerate it past the limit, so the demand that is left
    // is the force that closes the gap to the limit within one shift (the time scale at which the gear matters: farther from the limit the full pull is
    // still all of it, at the limit nothing but the holding force is left) and the pedal that counts is that share of the one pressed.
    // (A flat pedal against the 100 km/h limiter kept the gear that revs for power instead of the most economical one that holds the limit.)
    const float ve = off && S.ex_v > 0.5f ? S.ex_v : v;   // the speed the lift's reserve is a force at (the levels of the history are shares of a full wish at that speed)
    // The pedal that counts is the shaped share the demand has (the torque pedal's, or the cruise's own ask), not the raw pedal: the same pedal reads the same
    // everywhere (the floored path and the pedal blend of the band wish), and a cruise that asks its whole pull is floored like a flat pedal. No share (the plain pedal):
    // the raw pedal ... and under the game's own cruise (T.cruise on, no share: the demand is the throttle times this gear's force) the throttle in use, because the foot is off
    // (T.in_thr 0) and the cruise's throttle is the pedal there. A throttle against its stop asks everything, like a flat pedal; read as the pedal 0 it asked "what this gear gives", a
    // demand every gear delivers, however far under the set speed the truck sagged (WP-G1: 44 t on 3 % at 85 km/h in the fleet's cr85 rows stayed in the overdrive at full throttle,
    // 85 -> 47 km/h in 160 s, +28 % time in the worst pair; the old builds took the gear that holds 77).
    float pedal = clampf(wsh.share == wsh.share ? wsh.share : (T.cruise > 0.5f ? fmaxf(T.in_thr, thr) : T.in_thr), 0, 1);
    float f_lim = 1e30f;   // the most force the limiter lets the truck use (none: no limit)
    int capped = wsh.source == WISH_PEDAL && wish_limit_k() < 0.999f;   // the wish is held under a limiter or a cap (core/wish.cpp wish_limit_k: gc_in::capped)
    if (!off) {
        f_lim = wish_limit_force(v, dur);   // (core/wish: the one formula, the force that closes the gap to the limit within one shift)
        if (f_lim < 1e29f) {
            if (wish > f_lim) { ws = fminf(ws, f_lim); wish = f_lim; capped = 1; }
            // the pedal that counts is the share of a full wish the limiter lets through (the torque pedal has already taken some of its wish off near the
            // limit: the pedal itself still reads 0.9 at the limit)
            const float full0 = wish_full(v);
            if (full0 > 0) pedal = fminf(pedal, f_lim / full0);
        }
    }
    // The part of the plan horizon the truck spends AT the limit: 1 - (the time the pedal's free pull needs to take the speed to the limit) / (the horizon). A pedal against the limiter pulls the truck up to it and
    // then only holds it: there nothing is gained by pulling, the pedal says nothing about revs (a flat pedal against the limiter on +2 % asked the operating point of a full pedal, 1790 rpm in gear 10, where
    // gear 11 at 1440 and 12 at 1150 hold the limit), and the box holds the limit in the most economical gear that holds it. The part fades the foot of the pedal's operating point (cfg.foot below): far from the
    // limit the horizon is a pull (0, the foot is a foot), at the limit it is all hold (1). The time and not the force the limiter leaves (f_lim - f_res = m (top - v) / dur): that moves by a third of the pull
    // for a tenth of a km/h of sag, and the foot would swing with the limiter's own band and take the box with it. Continuous in the speed and in the pedal (a pedal that does not pull never reaches the limit).
    float at_limit = 0.0f;
    if (!off && f_lim < 1e29f && wsh.source == WISH_PEDAL) {
        // the pedal's free pull is the one the strongest gear gives at its share (wish_share x wish_full: gear-independent), not the demand in the gear we are in: that one falls with the revs (phi), and a tall gear at
        // low revs would read a pedal that cannot take the truck to the limit on a hill the strongest gear climbs
        const float sh = wish_share(), fd = sh == sh ? sh * wish_full(v) : wish_demand(), top = speed_top_kmh() / 3.6f;
        const float a_free = (fd - f_res) / V.mass;
        if (a_free > 1e-3f) at_limit = clampf(1.0f - fmaxf(top - v, 0.0f) / a_free / GC_HORIZON, 0.0f, 1.0f);
    }
    if (off) pedal = 0.0f;
    // A pedal that is on is an intent, however little it asks. The limiter on a road that asks nothing of the engine (a mild net descent at the limit: f_lim = max(f_res, 0) = 0) holds the wish at 0, and the
    // intent reads a wish of 1 N or less as no wish at all (all zeros: no band shares, so no gear qualifies and the chooser took the strongest one, a kickdown with nothing to deliver: 12 -> 10 on -1.3 %,
    // the ride 22:31:34, where 15 N later it chose the economical 12). The same 1.5 N the cruise's and the lift's wish have (the idiom of this function: layer 2 takes a wish over 1 N as an intent), so the
    // choice is continuous across the 1 N: a pedal on a road that asks nothing is a calm hold.
    if (!off && wish < 1.5f) { wish = 1.5f; ws = fmaxf(ws, 1.5f); }
    // the road ahead (layer 1's read-only view, core/ahead.h; its certainty already faded): the resistance that holds the speed now and the worst over
    // the next seconds
    const ahead_view road = *road_ahead();
    // the intent (layer 2): the wish, the hold share, the lugging shares and the band wish
    int mode = S.mode == MODE_ECO ? 0 : S.mode == MODE_POWER ? 2 : 1;
    pt_intent_cfg cfg;
    cfg.lug_max_pct = by_mode(p_lug_res_eco, p_lug_res, p_lug_res_pw);
    cfg.band_weight = p_band_weight;
    cfg.expect_weight = by_mode(p_resp_eco, p_resp, p_resp_pw);
    cfg.fuel_weight = by_mode(p_fuel_eco, p_fuel, p_fuel_pw);
    cfg.q = mode_q_for(S.mode);
    // The driver's demand history (core/demand.h): the largest demand to come in the horizon is a light, a half, a firm or a hard one (a share of a full wish at this
    // speed), with the probabilities the last ten minutes give (a fresh session: the uniform prior, which asks a reserve like the old cruise stand's, relaxing as
    // the driver proves calm). The band taste reads its 0.7-quantile as a force (the need of layer 2); the reserve of gc_expect reads the levels themselves, below.
    // Where a controller holds the speed with the foot off, the history is the CONTROLLER's (core/demand.h demand_ctl_*): what the cruise asked in the last minutes, session only. A cruise on rolling hills
    // has asked for the climb at every hill, and the gear that sagged on the last one does not get the crest's upshift for free; the driver's history there is the foot's, which is off. The chance the
    // foot is back (p_back below) does not scale it: the controller is there.
    const bool use_ctl = !off && T.cruise > 0.5f && clampf(T.in_thr, 0, 1) <= 0.02f && wsh.source == WISH_NONE && demand_ctl_seconds() >= 3 * (int)DEMAND_HORIZON_S;   // (the game's cruise, whose throttle is all there is, once it has three horizons of it: a cruise that has just started has asked for its own acceleration and nothing else; the plugin's own controllers keep the driver's history as before)
    const demand_view *dv = use_ctl ? demand_ctl_now() : demand_now();
    // (With the pedal off the levels are forces at the speed the lift began: a coast slows, a full wish asks more force at a lower speed, and judged at the
    // speed of now the reserve would ask for a lower gear every few seconds of the same lift. One lift, one decision.)
    const float full = wish_full(ve);   // (the history's shares are of the strongest force the gears give at that speed, as B tuned the reserve; the gear's phi rides on top inside gc_expect)
    // The cruise holds and the driver's foot is off: the hold goes on as long as such a hold does (the scale-free survival: about as long again as it has lasted), so the
    // band taste of the gear that holds it is paid for that long and not only for the plan horizon, and a shift is paid once. tau: since the foot last left the pedal
    // (never this session: the length the lift learner closes a lift at). The history is what the DRIVER may ask next, and the foot is off: it counts as far as the
    // foot is expected back, the chance that a foot that left the pedal tau seconds ago is back within the plan horizon (core/driver.h driver_lift_return_p: the
    // scale-free prior, learned from his lifts): a cruise that has held for a minute is a long lift of the foot, and the reserve for a big pull the cruise would not
    // ask for is worth that chance of it.
    const int cc_off = !off && T.cruise > 0.5f && clampf(T.in_thr, 0, 1) <= 0.02f;
    const float cc_tau = cc_off ? fminf(S.pedal_on_at < -1e8 ? DRV_LIFT_TIMEOUT_S : (float)fmax(0.0, now - S.pedal_on_at), DRV_LIFT_TIMEOUT_S) : 0.0f;
    const float p_back = cc_off && !use_ctl ? clampf(driver_lift_return_p(cc_tau, GC_HORIZON, dur), 0.0f, 1.0f) : 1.0f;
    // The band wish asks for the power band when the minutes AND the seconds agree that the eco band is short: the 0.7-quantile of the history (a stab at the pedal moves it by
    // a window in thirty) and the wish filtered over a few shift times (a hill, a pull now; the history alone is the uniform prior in a fresh session and in the rows with no pedal,
    // which asks for the power band whatever the truck is doing: the cruise on rolling hills shifted 19 -> 38 times).
    const float demand_n = full > 0 ? fminf(fminf(demand_level_at(dv, p_back, 0.7f) * full, f_lim), ws >= 0 ? ws : 1e30f) : -1.0f;   // (a demand past what the limiter lets the truck use is no demand)
    // The game's own cruise is a controller, not the driver's foot. Three places read "the pedal" and must not take the cruise's throttle for the driver's demand:
    // - the band wish's "a nearly full pedal wants the power band" (intent.cpp): a cruise at 0.88 on a rolling 1.5 % hill flipped the band wish ~0.90 toward the power
    //   band (the eco band's run cost 0 -> 1.79 s, the power band's land cost 3.0 -> 0.31 s) and the layers paid the band saving (2.76 s, band_scale x2) with a
    //   12->10->11->12 that bought no time (min speed 78 km/h either way). The band wish reads the foot (0).
    // - the floored / full-pedal path (intent.floored, layer 3's full-throttle plan): it is the driver's launch and kickdown; a controller holding a speed with the foot
    //   off does not launch, but a cruise pinned at full throttle on a real climb must still downshift before the speed is gone (cr85: 44 t on 3 % held 77 km/h in 14 where
    //   the box without the path sagged to 47). The plan is armed by the throttle, but only where this gear cannot hold the easiest stretch of the road ahead even with the
    //   room the intent already gives it (RUN_SLACK, intent.cpp 1.1 = 10 %): a rolling 1.5 % hill or a hump has a descent or a flat ahead the gear holds, so the truck
    //   recovers the 2 km/h sag on its own (min speed 78 km/h either way, the shift buys nothing); a sustained climb never drops under the gear's force, and there the
    //   downshift pays. Without a route the "easiest stretch" is the road of now, and the same 10 % tells the marginal hill (a few % short) from the climb (tens of % short).
    const int game_cruise_hold = wsh.source == WISH_NONE && T.cruise > 0.5f && clampf(T.in_thr, 0, 1) <= 0.02f;
    const float band_pedal = game_cruise_hold ? 0.0f : pedal;
    pt_intent it;
    cfg.controller = wsh.source == WISH_CRUISE || wsh.source == WISH_PLANNER || game_cruise_hold;   // (gc_in::controller below, and the game's cruise with the foot off)
    // The pedal as an rpm target (WP-0911-K1, core/intent.cpp): the foot is the raw pedal the share stands for (core/wish.h wish_unmap; a plain pedal or the game's throttle is already raw),
    // the end of his pedal is the profile's (core/driver.h driver_press_end), the mode's full-pedal point its share of the way from P2 to P3. A controller's throttle is no press: no kickdown,
    // except the game's own cruise pinned against its stop on a road the gear cannot hold (cr85: 44 t on 3 % at the throttle's stop must go down before the speed is gone): there the
    // throttle is the press, and the road says that it is one.
    const float foot_end = driver_press_end();
    float foot_raw = wsh.share == wsh.share ? wish_unmap(pedal, S.mode) : pedal;
    cfg.no_kick = cfg.controller;
    // A controller has no foot, so there is no operating point asked for: the plugin's cruise and the planner (cruise_follow: a hardbrake with the AEB active twice as long, 126 flips of the brake request in
    // a jam, whatever target is read off them: their momentary throttle or the 0.7-quantile of their own memory) keep the box they had. The target is the driver's foot's. (The game's own cruise, no foot
    // either, has none unless it is pinned against its stop on a road the gear cannot hold: below.)
    if (cfg.controller) cfg.target = 0;
    // A controller on a net descent is easing its throttle as the grade takes over: the engine's part is the drag, no operating point (the same reason intent.cpp gives the taste no weight there).
    if (cfg.controller && f_res <= 0.0f) cfg.target = 0;
    if (game_cruise_hold) {
        // The game's own cruise has no foot: its throttle swings with every hill and is the loop's output, not an operating point asked for, so the box is the band taste's, as it was (the band wish
        // reads the foot as 0, band_pedal above). Pinned against its stop on a road the gear cannot hold it is a press, and the throttle is the foot (see above).
        float min_ahead = f_res;
        for (int i = 0; i < road.n_samples; i++) if (road.sample_s[i] <= GC_HORIZON) min_ahead = fminf(min_ahead, road.sample_resist_n[i]);
        // (pinned at its stop in a gear that cannot hold: the press arms the target and the kickdown; the downshift that answers it has to pay as the lugging rule's does, WP-1011-T6C)
        const bool pinned = foot_raw >= foot_end - DRV_END_STEP && force_at(cur, v, 1.0f) < min_ahead / 1.1f && lug_best_gain(cur, v) > 0.0f;
        cfg.no_kick = !pinned;
        cfg.target = pinned;   // (no foot, no target: the box is the band taste's, as it was)
    }
    // The foot the target follows rises at once and comes down over the time a hold takes to tell from a hunt (the anti-hunt dwell, the caller's hold_s): a pedal that has come down is believed
    // once it has stayed down that long. Without it a driver who holds a speed with a pedal that swings by a fifth (the sim's hold, a foot on a rolling road) moves the target by 150 rpm each
    // way, and a box whose shifts cost a third of a second (a converter) follows it, one gear each swing.
    {
        const double dt = now - S.foot_t;
        const bool fresh = !(dt >= 0) || dt > 10.0 || S.foot_t == 0.0;   // (no history of the foot: the first sample of a session (the state starts at 0, a reset at none), or after a long time with the pedal off)
        if (fresh || foot_raw >= S.foot_f) S.foot_f = foot_raw;
        else S.foot_f += (foot_raw - S.foot_f) * (1.0f - expf(-(float)dt / fmaxf(dwell_ud(), 0.5f)));
        // A foot with no past is not pressing: the first sample is the pedal where it stands, not a press from nothing to there (a ride that starts on a pedal of 0.62 was judged as a press to the end,
        // kd = 1, and shifted 11 > 10 > 9 on the strength of one sample).
        if (fresh) S.foot_prev = foot_raw;
        S.foot_t = now;
    }
    // A press in progress is going somewhere: the foot is where it will be when the shift is in (the press rate over the shift's duration), so a stomp to the end is judged as the end and not
    // as the intermediate pedal it passes through on the way (an overtake from a calm hold downshifted at a half pedal and lost the seconds the shift costs).
    const bool rising = foot_raw > S.foot_prev + 0.005f && foot_raw >= S.foot_f - 1e-4f;
    cfg.foot = fminf(1.0f, S.foot_f + (rising ? rate * dur : 0.0f)) * (1.0f - at_limit);   // (at the limit the foot says nothing about revs: at_limit above)
    S.foot_prev = foot_raw;
    cfg.foot_end = foot_end;
    cfg.full_frac = by_mode(p_tf_eco, p_tf, p_tf_pw);
    {   // the power the road takes (K2 v3): the force that holds the speed times the speed, over the engine's peak power at the wheels (T x curve x omega x eta, with the learned force scale the model's forces carry); a descent takes none
        const eng_bands *bt = bands_now();
        const float pk = bt->ok && V.eta > 0.0f ? V.eng.torque * curve_factor(&V.eng, bt->p3) * bt->p3 * 0.10471976f * V.eta * V.f_scale : 0.0f;
        cfg.road_pw = pk > 0.0f && f_res > 0.0f ? clampf(f_res * v / pk, 0.0f, 1.0f) * (1.0f - at_limit) : 0.0f;   // (at the limit the road says nothing about revs either, as the foot does not: the limiter holds the speed and the gear that holds it is the box's, at_limit above)
    }
    pt_intent_compute(&it, &cfg, mode, v, wish, &road, band_pedal, rate, b, eco_force_at(n, v, b), demand_n);
    if (!off && !(it.hold_share >= 0.5f)) S.hold_age_t0 = now;   // (the hold of the speed begins when the wish is mostly a hold again: a pull ends the one before; a lift does not, as it did not end the age of the gear)
    S.tgt_rpm = off ? 0.0f : it.tgt_rpm; S.tgt_kd = off ? 0.0f : it.tgt_kd;
    // the caller's hard limits as the allowed mask: every gear, minus the rev-limit / stall caps and the anti-hunt direction
    unsigned long long allowed = 0;
    for (int g = 1; g <= n; g++) allowed |= (1ull << g);
    float lim = V.eng.rpm_limit, v2 = fmaxf(0.5f, v - (f_res / V.mass + sl_dec) * fmaxf(sl_dur, V.eng.shift_time));
    for (int g = 1; g <= n; g++) {
        float rg = vehicle_engine_rpm_at(g, g == cur ? v : v2);
        if (rg > lim * 0.96f) allowed &= ~(1ull << g);                        // the rev limit would shift up at once
        if (g < cur && rg < idle_rpm() + ioff(120) && !tc_relief(g, g == cur ? v : v2, wish)) allowed &= ~(1ull << g);  // the stall rule would take it off (not on a converter that carries the wish)
        if (g > cur && force_at(cur, v, 1.0f) < 1.05f * f_res && lug_rule_fires(g, v, f_res, thr)) allowed &= ~(1ull << g);   // the lug rule would take it off (no gear above holds the speed that is falling)
        // ... and a gear that cannot hold the road even at full throttle at the speed the gap leaves the truck at falls through the same rule the moment it is in (the speed that falls is its own, whatever the gear we
        // leave did): the rule's physics (full throttle under the road, the lug zone) at the landing, without the speed that is falling now. Without it the layers' 7 > 9 on a 6 % climb at a quarter pedal (41.9 t, Eco)
        // landed in a gear the lugging rule took off again 1.4 s later, every 20 s (g7 R3 .36 Eco, once the dip decided more of the choices)
        if (g > cur && thr > 0.25f) { const float fm = force_at(g, v2, 1.0f); if (fm < 1.05f * f_res && fm - f_res < V.mass * 0.03f && rpm_at(g, v2) < pw_lo() + 100) allowed &= ~(1ull << g); }
    }
    // A converter truck that is still slipping in the gear it is in takes the next gear, not a skip: the model credits the converter's multiplication to any gear at a standstill (a pull-away
    // on a 0.5 pedal chose 1 > 3 at 5 km/h and 1 > 4 at 15), the heat of such a launch is not in the score, and a skip needs a locked driveline to be a saving (WP-TC1, item 4).
    if (V.tc && vehicle_tc_slip(cur, v) < 1.0f) for (int g = cur + 2; g <= n; g++) allowed &= ~(1ull << g);
    // (The dwells guard a pull against hunting. With the pedal off nothing is pulled: the price of the shift now and of the reserve, and the double shift back to
    // the gear just left (in.prev below), are what keep one lift to one decision; the dwell would hold the lift's shift until it ran out, 3 s after the pull's last.)
    // (A press to the floor is the driver's kickdown: for the dwell after it the dwell does not hold it back -- the key's own help says "unless kickdown" -- and the score charges the shift back
    // to the gear just left twice, below. A pedal that stays on the floor is a pull, and its upshift followed at once by its downshift is the limit cycle the dwell is for: it holds.)
    // The press is the DRIVER's foot reaching the start of the kickdown (his pedal's end less a step), not the limiter's cap (the share the layers read is held under it: against the limiter it
    // swings up and down with the speed's sag, and every swing was a new press that skipped the dwell, ride 08-11 1631-1660 s).
    const float s_drv = wish_share();
    const float foot_drv = s_drv == s_drv ? wish_unmap(s_drv, S.mode) : foot_raw;
    const bool kick_now = !off && !cfg.no_kick && foot_drv >= foot_end - DRV_END_STEP;
    if (kick_now && !S.was_kick) S.kick_at = now;
    S.was_kick = kick_now;
    const bool kick_press = kick_now && now - S.kick_at < dwell_ud();
    // The price of undoing the last shift, and the dwell, are the price of reversing a decision IN THE SAME SITUATION, a hunt (WP-G8, the owner's decisions 63 and 64: "a price that fades").
    // How much of the situation of that shift is still there: the demand then (S.dm_last) against the pedal now (0 with it off), and for a shift made for braking, the braking still on or not
    // (the brake, the retarder lever, the engine brake). core/shiftcommon.cpp sc_reversal_weight has the derivation; 1 = the same situation (the whole price and the whole dwell), the pedal
    // off after a pull or the braking over with a calm pedal = nearly none, a full pedal = the whole.
    float rev_w = 1.0f;
    if (S.shift_dir != 0 && since < fmaxf(dwell_ud(), dwell_du())) {
        const float brk = fmaxf(T.in_brake, T.eff_brake);
        const bool ebrk = T.motor_brake || T.retarder > 0 || S.eb_level > 0.01f;
        rev_w = sc_reversal_weight(S.dm_last, off ? 0.0f : dm_share(cur), S.sh_braked, ebrk ? 1.0f : clampf(brk / 0.05f, 0, 1));   // (0.05: the brake over it is braking, decide_gear_rules)
    }
    if (!off && !kick_press && S.shift_dir > 0 && since < dwell_ud()) for (int g = 1; g < cur; g++) allowed &= ~(1ull << g);   // no downshift right after an upshift
    // K2 (owner 09-11): a lighter foot lowers the target and the box goes up to the gear that still delivers the lighter wish; the one guard is a descent. On a net descent with the speed rising the
    // engine's drag holds the truck and a taller gear drags less: the gear we are in stays. Only the truck's own signals: the push of the grade now (the resistance below nothing), the measured
    // acceleration, and that the foot asks no more than the push already gives (a pull-away down a slope is a pull: it goes up the gears).
    if (!off && f_res <= 0.0f && wish <= -f_res && T.accel > 0.0f) for (int g = cur + 1; g <= n; g++) allowed &= ~(1ull << g);
    // ... and after a downshift made for braking the dwell is as long as the braking is still the situation: out of the braking onto a calm pedal the layers' upshift is not a hunt (a lug or a
    // kickdown downshift keeps its whole dwell: nothing changed in their situation that the dwell was not made for)
    if (!off && S.shift_dir < 0 && since < dwell_du() * (S.sh_braked ? rev_w : 1.0f)) for (int g = cur + 1; g <= n; g++) allowed &= ~(1ull << g);   // no upshift right after a downshift
    float rpm_rate = v > 0.5f ? rpm * T.accel / v : 0.0f;
    gc_in in = { cur, 0, (int)p_max_skip, n, v, allowed, it, *b, ts, rpm_rate, GC_LATENCY_S };
    if (!V.tc) { in.shift_dur = sl_dur; in.shift_decel = sl_dec; }   // (a converter has no gap: its 0.3 s stays the estimate)
    in.road = &road;
    in.foot_for = wsh.source == WISH_PEDAL ? ts_foot_for : nullptr;   // (the foot a gear would have for the same wish: gc_in::foot_for)
    in.phi = wsh.source == WISH_PEDAL || off ? wish_phi : nullptr;   // the torque pedal asks each gear level x phi(its rpm); a controller's wish is a force (core/wish.h)
    in.controller = wsh.source == WISH_CRUISE || wsh.source == WISH_PLANNER;   // (a controller's excess over the holding force is its throttle of the moment, not a demand for the road ahead)
    in.capped = capped;
    in.game_cruise = cc_off && wsh.source == WISH_NONE;   // (the game's own cruise with the foot off: its throttle is the wish, gc_in::game_cruise)
    in.lat_load = lateral_load(v, now);
    in.hold_s = dwell_ud();   // (WP-G7: a landing the pull takes out of its range sooner than the anti-hunt dwell pays its shift time for the rest, gc_choose)
    // the driver's history for gc_expect: the probability that the LARGEST demand is a level and not the next one up is the difference of the two "at least" probabilities
    // The foot on the pedal is judged as it has been seen (p_seen: a calm foot has proved it within the minute); a foot that has just left the pedal, or a controller, keeps the whole prior.
    const float *pv = !off && !cc_off ? dv->p_seen : dv->p;
    if (full > 0 && (int)DEMAND_LEVELS <= (int)DEMAND_LEVELS_MAX) {
        in.hist_n = DEMAND_LEVELS;
        for (int q = 0; q < DEMAND_LEVELS; q++) {
            in.hist_f[q] = fminf(dv->level[q] * full, f_lim);   // (a demand past what the limiter lets the truck use is no demand)
            in.hist_p[q] = fmaxf(0.0f, pv[q] - (q + 1 < DEMAND_LEVELS ? pv[q + 1] : 0.0f));
        }
        // ... and not past a net descent ahead: where the road turns into one the hold ends (the engine brake and the descent control take the speed)
        float t_end = 1e9f;
        for (int i = 0; i < road.n_samples; i++) if (road.sample_resist_n[i] <= 0) { t_end = road.sample_s[i]; break; }
        if (cc_off) {
            // (a hold that is not happening goes on for nothing: where even the strongest gear gives less than the holding force the truck slows whatever the gear, the speed is not
            // held and its taste is not charged for the minute the cruise has held before: T3, the 2 km climb at 4 %, 9 -> 10 at 12.4 s of taste for the temporary band against 4.4,
            // the gear that could not pull it, and the lesson that sent it back)
            in.band_scale = full >= f_res ? 1.0f + fminf(cc_tau, t_end) / GC_HORIZON : 1.0f;
            // (Tried alone, before the cruise's wish above was fixed, this made the Renault 36 t on 4 % reverse at the cruise's start: 13 -> 14 -> 13 -> 12.)
            for (int q = 0; q < DEMAND_LEVELS; q++) in.hist_p[q] *= p_back;
            in.hist_full = 1;
        } else if (!off && !s_pick_idle) {
            // The pedal holds the speed: the same survival, by the age of the gear's hold. A gear that has held for tau seconds is expected to hold about tau more, and the band
            // taste and the fuel are rates, paid for as long as it holds, where the shift into a better band is paid once. (WP-G1b: charged for the plan horizon only, a calm pedal
            // that held a speed or crept up a ramp for minutes (the sweep's rampd, a pull of 0.04 m/s2) stayed in the power band for a saving of 1.2 s of taste against the 1.3 s of
            // the shift: Eco rampd 1205 -> 1439 rpm on the 16-pair subset, fuel +3.0 %.) Only a hold goes on: the wish is a hold where more than half of it holds the speed
            // (hold_share over 1/2: the holding force is more than the pull on top of it), counted from there to the whole wish (2 hold_share - 1); a wish that mostly accelerates the
            // truck is a pull and is not extended (the weights of both terms are scaled by hold_share already, so a pull pays no taste and its shifts are the plan's; extended by the
            // whole age a pull from a standstill took its upshifts 2 .. 4 s earlier (the 18i-5 roundabout rows) and a near tie at the end of a pull flipped into the lug band, 9 -> 12
            // at 716 rpm and back 12 -> 11 a second later (18i-4, DC16 40 t Eco)). The age is counted from the start of the hold (S.hold_age_t0, the last time the wish was a pull: the
            // hold ends the pull before it, as before), not from the last shift: the age is a fact of the situation and a gear change inside one hold does not start it again. Counted from the shift, a
            // change of gear halved the weight of the taste for the gear it landed in (Power on the flat, k1 41.9 t, 80 km/h: g12 -> g11 at 1.0 s of path advantage, the age back to 0, and 25 s later the
            // same hold chose g12 again), as far as a net descent ahead (it ends the hold, as the cruise's) and at most HOLD_AGE_MAX_S. A bend breaks the hold:
            // lat_load is the share of the truck's own bend allowance in use (core/gearchoice.h), the hold counts for what is left of it, and at the whole allowance the horizon is the
            // plan's again, where the stay bonus of the bend keeps the gear until the exit (18i-5: with the full hold the shift due in the roundabout came inside it, 3 GAP).
            in.band_scale = full >= f_res ? 1.0f + clampf(2.0f * it.hold_share - 1.0f, 0, 1) * (1.0f - clampf(in.lat_load, 0, 1)) * fminf(fminf((float)(now - S.hold_age_t0), HOLD_AGE_MAX_S), t_end) / GC_HORIZON : 1.0f;
            // The reserve is paid for the same stretch: the history's chances are those of ONE horizon window (the largest demand in a window of 20 s), a hold of band_scale windows meets
            // a demand of the level in at least one of them with 1 - (1 - p)^band_scale. (Neutral on the sweep: Eco subset fuel +0.50 % against +0.48 %; without it the taste and the
            // fuel grow with the hold and the reserve does not.)
            if (in.band_scale > 1.0f) {
                float P[DEMAND_LEVELS_MAX + 1];
                for (int q = 0; q < DEMAND_LEVELS; q++) P[q] = 1.0f - powf(1.0f - clampf(pv[q], 0, 1), in.band_scale);
                P[DEMAND_LEVELS] = 0.0f;
                for (int q = 0; q < DEMAND_LEVELS; q++) in.hist_p[q] = fmaxf(0.0f, P[q] - P[q + 1]);
            }
        }
    }
    if (off) {
        // the pedal is off: lift_n is the demand that is expected back (it sets the taste and the band wish), the reserve is the history's, and lift_p the chance
        // that the driver asks for anything at all within the horizon (the lightest level of the history)
        in.lift_n = wish;
        in.lift_p = dv->p[0];
        // the expected drive a shift now loses: the chance that the pedal returns during it. A lift shorter than a shift cannot be used for one (the owner's
        // rule, 02-11), so until the lift is as long as a shift the pedal is as good as back; after it the chance falls with the shift duration as the time constant.
        in.lift_scale = gc_lift_return((float)(now - fmax(S.wish_on_t, S.pedal_on_at)), dur + GC_LATENCY_S, dur);   // (the lift clock: the pedal itself ticks every frame, the layers are not asked in a shift gap)
    }
    // anti-hunting inside the score: a shift back to the gear just left within the dwell costs its shift time twice (gc_in::prev, the design of 02-11). The gear just left is gear_before:
    // last_gear is the gear we are in since the shift was noted (note_gear_change), so passing it named cur, a gear no candidate is, and the price never applied except through the
    // kickdown's own line (WP-G6c round 2).
    if (S.shift_dir > 0 && since < dwell_ud()) in.prev = S.gear_before;   // (the kickdown inside the dwell too: the gear just left costs its shift twice)
    if (S.shift_dir < 0 && since < dwell_du()) in.prev = S.gear_before;
    in.prev_w = rev_w;   // (twice, while the situation of that shift is still there; WP-G8)
    gc_out dbg;
    int g;
    char terms[300];
    {
        // what the old choice did not see (see s_gs): one number, compared, so a change of any of them decides again
        const bool brake_on = fmaxf(T.in_brake, T.eff_brake) > 0.05f || T.motor_brake || T.retarder > 0 || S.eb_level > 0.01f;   // (the same touch decide_gear_rules and the reversal weight read as braking)
        // the limiter wall: the revs the gear will have within GC_LATENCY_S reach the cap (the expression gc_choose's near_wall has): a stale choice must never wait for the next interval at the wall
        const bool near_wall = vehicle_engine_rpm_at(cur, v) >= b->cap - rpm_rate * GC_LATENCY_S;
        const int state = (near_wall ? 1024 : 0) | (off ? 1 : 0) | (s_pick_idle ? 2 : 0) | (kick_now ? 4 : 0) | (kick_press ? 8 : 0) | (brake_on ? 16 : 0) | (in.controller ? 32 : 0) | (in.capped ? 64 : 0) | (in.game_cruise ? 128 : 0)
                        | (rising ? 256 : 0) | (S.out_of_gear ? 512 : 0) | (mode << 11) | ((int)wsh.source << 13);
        const bool same = s_gs.valid && s_gs.cur == cur && s_gs.state == state && s_gs.allowed == allowed && s_gs.shift_at == S.shift_at && s_gs.shift_dir == S.shift_dir && s_gs.gear_before == S.gear_before
                          && fabsf(foot_raw - s_gs.foot) < DRV_END_STEP && !(s_gs.out.cell_v > 0.0f && fabsf(v - s_gs.v) >= s_gs.out.cell_v) && now >= s_gs.at && now - s_gs.at < s_gs_interval;
        if (same) {
            dbg = s_gs.out; g = s_gs.g; memcpy(terms, s_gs.terms, sizeof terms);   // (the choice stands: its terms are the ones of the moment it was made)
            s_last_gc = dbg; s_last_gc_at = now;
        } else {
            LARGE_INTEGER q0, q1, qf; QueryPerformanceCounter(&q0);
            g = gc_choose(&in, &dbg);
            QueryPerformanceCounter(&q1); QueryPerformanceFrequency(&qf);
            if (qf.QuadPart > 0) gc_cost_note((float)((double)(q1.QuadPart - q0.QuadPart) * 1e6 / (double)qf.QuadPart));
            s_last_gc = dbg; s_last_gc_at = now;   // (observation only: ts_last_choice, the promise checks of tools/sim/sim_promises.cpp)
            // Every decision with its terms, per candidate (seconds: shift, accel, band, expect, stay; = the score, lowest wins): the log shows WHICH term
            // chose the gear. The candidates within three gears of the one we are in, the gear chosen first.
            size_t tk = 0; terms[0] = 0;
            for (int pass = 0; pass < 2; pass++)
                for (int c = (pass ? 1 : g); c <= (pass ? n : g); c++) {
                    const gc_term *t = &dbg.t[c];
                    if (!t->candidate || (pass && (c == g || c < cur - 3 || c > cur + 3))) continue;
                    int w = snprintf(terms + tk, sizeof terms - tk, "%s%d%s sh %.2f ac %.2f bd %.2f ex %.2f fu %.2f st %.2f = %.2f", tk ? "; " : "", c, c == cur ? "*" : "", t->shiftc, t->accc, t->bandc, t->expc, t->fuelc, t->stay, t->score);
                    if (w > 0) tk = tk + (size_t)w < sizeof terms ? tk + (size_t)w : sizeof terms - 1;
                }
            {   // (for the hint votes: the text is cut at the last whole candidate that fits)
                if (!terms[0]) snprintf(terms, sizeof terms, "no scores (%s)", dbg.why == GC_KICKDOWN ? "kickdown" : dbg.why == GC_NEAREST ? "no gear in the window" : "the full-throttle plan");
                const size_t tl = strlen(terms);
                snprintf(s_last_terms, sizeof s_last_terms, "%s", terms);
                if (tl >= sizeof s_last_terms) { char *cut = strrchr(s_last_terms, ';'); if (cut) *cut = 0; }
                s_last_terms_at = now; s_last_terms_cur = cur; s_last_terms_pick = g;
            }
            s_gs.valid = true; s_gs.at = now; s_gs.g = g; s_gs.out = dbg; memcpy(s_gs.terms, terms, sizeof terms);
            s_gs.cur = cur; s_gs.state = state; s_gs.allowed = allowed; s_gs.shift_at = S.shift_at; s_gs.shift_dir = S.shift_dir; s_gs.gear_before = S.gear_before; s_gs.foot = foot_raw; s_gs.v = v;
        }
    }
    if (g != cur) {
        const gc_term *t = &dbg.t[g];
        char act[48];
        if (dbg.why == GC_WALL) snprintf(act, sizeof act, "wall");
        else if (dbg.why == GC_KICKDOWN) snprintf(act, sizeof act, "kickdown");
        else if (g > cur) snprintf(act, sizeof act, dbg.skip ? "skip %d" : "up", dbg.skip);
        else snprintf(act, sizeof act, "down");
        logv(1, "layers: %d->%d (%s band, %s, %.0f rpm; wish %.0f N, hold %.0f N, share %.2f%s, pedal %.2f, urgency %.2f, need %.2f, lat %.2f; expected %.0f N back with p %.2f, shift cost x%.2f) terms: %s", cur, g,
             gc_band_name(t->band), act, t->rpm, wish, road.resist_now_n, it.hold_share, off ? ", pedal off" : "", pedal, it.urgency, it.need, in.lat_load, off ? in.lift_n : 0.0f, off ? in.lift_p : 1.0f, off ? in.lift_scale : 1.0f, terms);
        *why = W_LAYERS;
        if (off) S.lift_pick = g;   // (the lift has had its decision when the shift is made: shift_note below)
        return g;
    }
    // The standing decision, once a second at level 2 (WP-G6: the 03-11 ride could not say which term kept a gear, a decision is logged only when it shifts): the weights the terms carried
    // (the share of the wish that holds the speed scales the band taste, the fuel and the history's reserve; the age of the hold the same) and the candidates' terms. Level 2 is silent at
    // the shipped log level; "layers hold:" is not a "layers: a->b" decision line. The dev flavor only (the owner: "log it all, as long as it is not in production"): the branch is compiled
    // in every build (so a sim checks the arguments) and optimized away in the public one, whose log is byte for byte what it was. Not for the candidate gear a roll's exit asks about.
    {
#ifdef DAZDRIVE_DEV
        static const bool k_hold_line = true;
#else
        static const bool k_hold_line = false;
#endif
        static double said = -1e9;
        if (k_hold_line && !off && !s_pick_idle && (now - said >= 1.0 || now < said)) {
            said = now;
            const demand_view *dvh = dv;
            logv(2, "layers hold: %d at %.0f rpm (%s band; wish %.0f N, hold %.0f N, share %.2f, band scale %.2f, pedal %.2f, need %.2f; the history's chances of 0.3 / 0.5 / 0.7 / 0.9 of a full wish %.2f %.2f %.2f %.2f) terms: %s",
                 cur, rpm, gc_band_name(dbg.t[cur].band), wish, road.resist_now_n, it.hold_share, in.band_scale, pedal, it.need, dvh->p[0], dvh->p[1], dvh->p[2], dvh->p[3], terms);
        }
    }
    *why = W_HOLD;
    return cur;
}

// The gear the layers would pick for the speed and the demand of now when the truck comes out of neutral, from the gear g a physics pick gave: one decision for every end of a roll, so the
// exit is one shift and not an exit into a gear the layers leave in the same tick (the exit gear of roll_exit_gear is a physics pick for the speed: with the pedal off after a long roll the
// shift costs nothing and the layers moved 10 -> 12 the moment 10 was in). The braking exits keep the physics gear (the brake range, the braking floor).
static int layers_pick_from(int g, float v, int ebrake, float brake)
{
    const int n = n_gears();
    if (ebrake || brake > 0.02f || S.mode == MODE_M || g < 1 || g > n || !V.eng_ok) return g;
    const int coast_p = (T.cruise > 0.5f ? S.thr : clampf(T.in_thr, 0, 1)) < 0.05f;
    int why = W_NONE;
    s_pick_idle = 1;
    const int p = gear_layers(g, n, v, rpm_at(g, v), S.thr, resist(v), coast_p, 1e9, sim_s(), &why);
    s_pick_idle = 0;
    return p >= 1 && p <= n ? p : g;
}
// TrueShift's own end of a roll (the gain, the speed band): the exit gear of the physics, then the layers' pick.
static int roll_exit_pick(float v, int ebrake, float brake)
{
    ebrake = ebrake || S.roll_prot;   // (the runaway protection ended it: the gear whose revs sit in the engine brake's range, as for a lever)
    return layers_pick_from(roll_exit_gear(v, ebrake, brake), v, ebrake, brake);
}

static void decide_gear_rules(void *self);

// The engine's drag in gear g at the speed v (N at the wheels, throttle shut): the learned share of the peak torque, at the rpm of the gear (the drag grows with the rpm: kd
// was measured at kdr), in the force the gear gives at full throttle (the same expression roll_gain follows a coast with).
static float drag_at(int g, float v) { return vehicle_drag_force(g, v); }   // (the vehicle model's: the brake channel's natural deceleration reads the same drag)
// Pedal off on a net descent (the grade pushes the truck, f_res <= 0): the engine's drag is what holds the speed. The drag that holds it is the drag the truck has now plus what
// it is short by: the truck gains speed at a (T.accel), so its drag falls short by m x a, whatever the model's push says (a mass the model has wrong, 25 t seen as 14 t, made the
// pick judge the push of the model: a gear that held it by 3 N, and the next one 1.6 s later when the descent steepened), and the road ahead adds what the push still grows by
// within the plan horizon (the timed samples of ahead_now(), as gc_expect reads the road; blind, nothing). One descent, one decision: the gear to take is the highest whose drag
// is at least that (the least drag that holds: the least noise and the farthest the truck coasts when the road flattens), inside the bands from the floor to the gear's crossover
// (never the temporary band: that is the revs the owner hears, 02-11); none holds -> the one that drags the most inside them (the descent control and the brakes take the rest).
// A gear that already holds stays: nothing up (the drag that is left over slows the truck, the road flattens), a downshift only where the gear we are in does not hold. 0 = no change.
// WP-G6b (c), the owner (03-11): "I no longer think of high revs when I lift off, I have the retarder for that". A lift is not a request to slow down. The need is the drag that is ASKED
// of the gearbox, and the pedal-off path of a gear in (engaged) is reached only when nobody asks: no brake (decide_gear_rules), no retarder lever, no engine brake stage, and no
// standing request of the brake channel or of the lever assist (gear_layers returns W_HOLD for those before it asks; stand_drag_pick answers them with the same drag_gear query and its pay
// test). What the grade adds to the speed under a plain lift ('speeding up' and 'road ahead', the two parts this pick used to hold) is the driver's lever's to take, and the brake
// channel's the moment he brakes; bought with the gear, it was paid in revs the layers price at a landing (the pow band at 1700 rpm in the ride's A2: 10 -> 9 for 1400 N of drag
// worth 0.04 m/s2) and in the shift's gap, for a push the pick had no ask for. So with a gear in, the need is the drag the truck has: a gear that holds that stays, and a gear whose
// revs are outside the bands (below the floor, over the crossover) is the one case left that moves, down to the nearest that is inside. The roll's exit (nothing in gear: the engine
// has to hold it) asks the whole push, as it did.
static int descent_gear(int cur, int n, float v, float f_res, int engaged, char *terms, size_t tn)
{
    if (terms && tn) terms[0] = 0;
    const eng_bands *b = bands_now();
    if (!b->ok || cur < 1 || cur > n) return 0;
    const ahead_view *road = road_ahead();
    float low = f_res;   // the steepest push within the horizon (the lowest resistance)
    for (int i = 0; i < road->n_samples; i++) if (road->sample_s[i] <= GC_HORIZON) low = fminf(low, road->sample_resist_n[i]);
    // (a roll ending on a descent asks which gear to engage: the truck has been in neutral, its gain is the whole push and the gear we are in has no drag yet)
    const float drag_now = engaged ? drag_at(cur, v) : 0.0f;
    const float accel_n = engaged ? 0.0f : V.mass * fmaxf(0.0f, T.accel), ahead_n = engaged ? 0.0f : fmaxf(0.0f, f_res - low);   // (asked: nothing under a plain lift, the whole push at a roll's exit)
    char head[120];
    snprintf(head, sizeof head, "need %.0f = drag now %.0f + asked: speeding up %.0f + road ahead %.0f", drag_now + accel_n + ahead_n, drag_now, accel_n, ahead_n);
    return drag_gear(cur, n, v, drag_now + accel_n + ahead_n, engaged, n, v, head, terms, tn);
}

// THE drag-gear query (package BR): the gear whose engine drag holds `need_n` (N at the wheels, the whole drag wanted, the drag of the gear we are in included), the one
// question the gearbox answers for every asker: the pedal-off descent above (the push of the grade), the brake channel's wish for more drag and the lever assist's (the stand,
// stand_drag_pick). The gearbox owns which gear gives the drag: only a gear whose revs, at the speed v_land the shift ends at, sit inside the bands from the floor to its
// own crossover (the owner's noise rule, 02-11: never the temporary band), the highest of them whose drag holds it (the least drag that holds: the least noise and the
// farthest the truck coasts when the road flattens); none holds it: the one that drags the most inside them (the retarder and the service brake take the rest). Down only
// once in gear (an upshift would take the drag away), at most max_down gears. 0 = no change. head is the first words of the terms text.
static int drag_gear(int cur, int n, float v, float need_n, int engaged, int max_down, float v_land, const char *head, char *terms, size_t tn)
{
    const eng_bands *b = bands_now();
    if (!b->ok || cur < 1 || cur > n) return 0;
    auto usable = [&](int g) { const float r = rpm_at(g, v_land); return r >= b->floor && r <= gc_cross(b, g, n, v_land); };
    if (engaged && usable(cur) && drag_at(cur, v_land) >= need_n) return 0;
    int best = 0; float best_drag = 0.0f;
    for (int g = 1; g <= n; g++) {
        if (g < cur - max_down || !usable(g)) continue;
        const float d = drag_at(g, v_land);
        if (d >= need_n) { if (!best || d < best_drag || best_drag < need_n) { best = g; best_drag = d; } }
        else if (!best || (best_drag < need_n && d > best_drag)) { best = g; best_drag = d; }
    }
    const int pick = best && best != cur && (best < cur || !engaged) ? best : 0;   // (down only, once in gear: an upshift would take the drag away; the gear we are in is not the gear to leave upward)
    if (pick && terms && tn) {   // the pick's terms (N at the wheels): what the drag has to hold, and what each gear in the bands to its crossover gives
        size_t k = (size_t)snprintf(terms, tn, "%s; holds from %d (%.0f N at %.0f rpm)", head ? head : "", pick, best_drag, rpm_at(pick, v_land));
        for (int g = 1; g <= n && k + 1 < tn; g++)
            if (g >= cur - max_down && usable(g)) { int w = snprintf(terms + k, tn - k, "; %d%s %.0f N", g, g == cur ? "*" : "", drag_at(g, v_land)); if (w > 0) k = k + (size_t)w < tn ? k + (size_t)w : tn - 1; }
    }
    return pick;
}

// The brake channel's standing wish for more engine drag (core/gearreq.h, kind GSK_SUSTAINED: its need has lasted and the retarder, the stages and the service brake
// are carrying it), turned into a gear by the drag-gear query above. Two things the gearbox weighs that the asker does not know:
//  - a lift is no surprise (promise P2): not before the pedal has been off for a shift and its latency, as the pedal-off descent's pick has it, so a blip of the pedal that
//    ended the asker's request and brought it back never makes a shift;
//  - the shift has to pay, with the road ahead: the shift's gap takes the drag away for a shift duration (`lost` m/s2 for dur seconds, the brakes carry it), the lower gear gives
//    `gain` m/s2 more for as long as the push lasts (t_push: the ahead samples' stretch to where the road stops pushing; no route: about as long again as the wish has stood,
//    the scale-free survival the layers' hold uses). It pays when gain x t_push > lost x dur: the cost of what the brakes carry (the same stage before and after, so its price
//    cancels) against the saving; a push that ends before the shift is done never pays.
// s_drag_terms is the text for the log line the caller writes when it acts.
static char s_drag_terms[240];
static int stand_drag_pick(int cur, int n, float v, const gear_stand &st, double now)
{
    static double t0 = -1e9, last = -1e9;
    if (now - last > 0.5) t0 = now;   // the wish was gone for more than a moment: its age starts over
    last = now;
    s_drag_terms[0] = 0;
    const float dur = V.tc ? 0.3f : fmaxf(sl_dur_est(), V.eng.shift_time);
    if (now - fmax(S.wish_on_t, S.pedal_on_at) < dur + GC_LATENCY_S) return cur;
    const float v_e = fmaxf(0.5f, v - fmaxf(0.0f, -T.accel) * V.eng.shift_time);   // the speed at the end of the shift, the braking carried through it
    const float drag0 = drag_at(cur, v_e);
    char head[120], terms[200];
    snprintf(head, sizeof head, "the brakes want %.0f N more drag than the %.0f N of %d", st.drag_n, drag0, cur);
    const int l = drag_gear(cur, n, v, drag0 + st.drag_n, 1, p_eb_skip > 0 ? 2 : 1, v_e, head, terms, sizeof terms);
    if (!l || l >= cur) return cur;
    const float m = V.mass > 0 ? V.mass : VEHICLE_MASS_FALLBACK;
    const float gain = (drag_at(l, v_e) - drag0) / m, lost = drag0 / m;
    const ahead_view *road = road_ahead();
    // How long the need lasts. On a descent that pushes the truck now and a route that is known, the road says: to the first stretch that does not push. Anywhere else (the
    // flat, a climb: a lead, a junction, a bend, a queue asks for the braking, nothing in the road ends it) it is the scale-free survival the layers' hold uses: about as long
    // again as the wish has stood. (The road's first stretch that does not push, on a road that does not push now, is the very next sample -- under a second -- and would never pay.)
    float t_push = fmaxf(1.0f, (float)(now - t0));
    const char *src = "as long again as it has stood";
    if (road->n_samples > 0 && road->conf >= 0.5f && road->resist_now_n < 0) {
        t_push = road->horizon_s; src = "the road ahead";
        for (int i = 0; i < road->n_samples; i++) if (road->sample_resist_n[i] > 0) { t_push = road->sample_s[i]; break; }
    }
    if (!(gain > 0.03f && gain * t_push > lost * dur)) return cur;   // (a gear that adds almost nothing, or a push that ends before the shift pays for itself)
    snprintf(s_drag_terms, sizeof s_drag_terms, "%d->%d: +%.2f m/s2 of drag for ~%.0f s (%s) against %.2f m/s2 lost for the %.1f s shift; %s", cur, l, gain, t_push, src, lost, dur, terms);
    return l;
}

// The highest rpm a downshift may land on at the speed of now: the wall of the layers (core/gearchoice.cpp gc_choose: the bands' cap, 95 % of the
// limit, less the rpm the revs gain during the latency). The rev-limit rule shifts a gear off at 96 % of the limit (99 % with the engine brake), so a
// gear that lands above the cap is one the next tick takes off again. These were the two terms that flipped in the owner's ride (02-11: braking
// downshifts landed at 1840-2142 rpm of a 2200 limit, then the rev-limit rule shifted up: 8>5>2>3>1, 9>6>9): the landing a braking rule plans for (the
// speed at the end of the shift, with the deceleration of now carried through it) and the speed the next tick sees if the braking lets go or does not
// fall as planned. The rate the revs gain is the pull of now and never negative: while braking the wall is the cap, not above it (a falling speed is no
// reason to land higher). The latency is the one gear_layers hands gc_choose (GC_LATENCY_S, core/gearchoice.h).
static float landing_wall(int cur, float v)
{
    const eng_bands *b = bands_now();
    if (!b->ok) return V.eng.rpm_limit;
    const float rate = v > 0.5f ? rpm_at(cur, v) * fmaxf(0.0f, T.accel) / v : 0.0f;
    return b->cap - rate * GC_LATENCY_S;
}

// Every rule's downshift while pulling: never into a gear above the wall at the speed of now (it was the rev limit itself, which the rev-limit rule
// at 96 % of it took off again). A rule that plans for the speed at the end of the shift (braking: the deceleration of now carried through the shift)
// lands too low when the braking stops (ETS2 ride: the planner braked at 5 m/s2, 7->1 at 18 km/h, then the pedal went down: 2548 rpm on a 2200 limit).
// (While the truck keeps slowing with the pedal off the end-of-shift speed is the right one to plan for, but the braking rule keeps its own landing
// under the wall too: decide_gear_rules.)
static int decide_gear(void *self, int *why)
{
    gear_req_begin();
    decide_gear_rules(self);
    int want = gear_req_resolve(why);
    int cur = sense_gb_gear(self);
    if (want == GR_NONE) { want = cur; *why = W_HOLD; }   // (every path of the rules posts; this is only the safe answer: keep the gear)
    float v = fabsf(T.speed);
    int pulls = clampf(T.in_thr, 0, 1) >= 0.05f || T.accel >= 0;
    if (pulls && want > 0 && want != ROLL && cur > 0 && want < cur && V.eng.rpm_limit > 0) {
        int w = want;
        const float wall = landing_wall(cur, v);
        while (w < cur && rpm_at(w, v) > wall) w++;
        if (w != want) logv(2, "downshift %d->%d raised to %d: %d runs %.0f rpm at %.0f km/h (wall %.0f, limit %.0f)", cur, want, w, want,
                            rpm_at(want, v), v * 3.6f, wall, V.eng.rpm_limit);
        want = w;
    }
    return want;
}

// ---------------------------------------------------------------- the revs under the lever (WP-K3, owner decision 70)
// The owner (05-11): "every step gives an allowed range, and it may go up or down in it on time or rising speed ... down when under control", for the retarder lever and the engine brake, never for
// the brake pedal. It replaces the held-stage rule (every step all the way down the box: K2 measured 11>9>7>5>3>1 at every braking strength) for the DRIVER's lever. The held-stage rule stays for
// what is not the driver's lever: the pedal's automatic engine brake and a stage the brake channel holds (the descent control and the braking assist are not this package).
//
// The band of step k of n. The engine's engine-brake range [lo, hi] (the game's rpm_range_engine_brake, 1500-2500 by default) is cut into n slices of s = (hi - lo) / n and step k is given the
// slice it reaches (U = lo + k s: the top step reaches hi) and the one below it (L = lo + (k - 2) s), so that the band of the next step up overlaps this one by a whole slice (a change of step
// forces no shift unless the rpm is outside the new band). Step 1 has no lower edge: the gear in may stay, a short tap never shifts. Default engine, before the two limits below: n = 3: 1: up to
// 1833, 2: 1500 - 2167, 3: 1833 - 2500; n = 5: 1: up to 1700, 2: 1500 - 1900, 3: 1700 - 2100, 4: 1900 - 2300, 5: 2100 - 2500. Two things the engine and the box add, not tuned:
//  - U is never over 0.93 of the rev limit, the ceiling pick_down already lands under (the rev-limit rule shifts a gear off at 0.99 of it with a lever held);
//  - a band is never narrower than the box's largest step between adjacent gears (L <= U / rho): where it is, some speeds have no gear inside it at all (n = 5 on the defaults: 400 rpm
//    against a step of 1.25 - 1.3 at 1700 rpm). The narrow bands of the top steps widen downward to one gear step, the wide ones stay as they are.
// n is the retarder's step count; the engine brake alone has the game's own stage count, 3 by default (engine_brake_positions, notes/game-defs.md), which TrueShift is not told: the stage fraction
// the game reports (stage / stages) is read as step ceil(fraction x 3). The retarder needs retarder_downshift (its key: the retarder lever follows its band; 0 = only the engine brake does).
//
// Inside the band, three terms and no new number; the time scales are the shift (the speed its gap gives the road) and the hunt guard's dwell:
//  - OUTSIDE it (the lever just pulled or changed, the speed moved the rpm out): the fewest shifts that put the rpm in it (the nearest gear whose rpm is inside, at the speed the shift ends at;
//    none inside: the nearest to the band, if nearer than the gear in), up to two gears at once as the engine brake's rule always did. A landing must stay over the band's bottom for the dwell at
//    the deceleration of now, or the upshift is taken straight back (a truck braking hard comes into the band by itself); a downshift never lands over the rev ceiling.
//  - UP in the band (a downshift) when the speed keeps rising against the lever: a downshift gives the speed a gap, `lost x dur` (the deceleration the gap gives up, the lever's and the engine's
//    drag in this gear by the model's force, over the shift's length). If the speed has gained more than that since it was last at its lowest in this gear (counted from a shift and its latency
//    after the pull: the retarder has built up, and a tap is never answered) it has been rising for as long as a gap takes to pay, and "as long again" is the survival the brake channel's wish
//    uses: the shift pays. The same quantity is the time scale (the speed gained is the rate of the rise times how long it has lasted): a small rise waits long, a large one not at all, continuously.
//    The landing must stay under U for the speed after the gap (the road alone moves it: a bridge of the service brake is not counted on) and for the dwell after it, at the rise the model gives in
//    the lower gear or the truck shows now, whichever is more; after an upshift the hunt guard's dwell (G8's fading) holds it.
//  - DOWN in the band (an upshift) as soon as the speed is under control: it is not rising, and the gear above holds the speed by the model's force in that gear with this step against the road
//    (the lever's force through the ratio, the engine's drag, the road's push at the speed the shift ends at), lands inside the band over the stall rule's own landing, and stays there for the dwell.
//    The shift's price: the gap gives the road's push a free `dv = push / m x dur`; the gear above must win it back, |shortfall| x (how long the lever has been held) >= dv, again "as long as it has
//    stood". The price of undoing the shift just made is G8's (sc_reversal_weight: the same situation, the whole price): the gear just left costs that price twice (decision 63), and where it was
//    left because the speed rose in spite of the model, the model's optimism there (the measured rise less its shortfall) is added to its shortfall for as long as the lever stays: the same
//    model cannot flip back.
//  - The model's lever force is multiplied by what the truck measures of it (lb_cal): in a steady stretch the lever's part of the deceleration against the model's, one number for the truck, taken with
//    the dwell as its time scale. The model's start values are an average truck's (the owner's retarder gave 1.83 x its step 1 in the ride of 04-11).
//  - The brake pedal changes nothing here (the rule does not read it). Not met: where the band's top leaves no room under it for the gear below (n = 5, step 1 at 40 t on -6 %), no downshift is taken
//    (it would be undone by the band's top within seconds); that step then moves up in its band only as the speed raises the rpm. docs/powerband-change.md, K3.
// The lever's hunt guard is the clutch box's dwells for every box: a converter box shifts fast and cheaply, so the layers shorten their dwells there (1.0 / 0.7 s), but a reversal of the lever's shift is a
// hunting pair for the driver whatever the shift costs (run.sh's metric counts one within 5 s), and the converter truck is where a lever rule that undid its own shift 0.7 s later hunted (er1 E4e).
static float lv_dwell_ud() { return p_dwell_ud; }
static float lv_dwell_du() { return p_dwell_du; }
struct lever_band { int on, k, n; float L, U, X, rho; };   // (X: the gear step's crossover, the rpm the band's top is held under while the speed is not rising; 1e9 = not known)
static lever_band lever_band_get(const gear_stand *st, int eb_hold, int n_gears_)
{
    lever_band b = { 0, 0, 0, 0.0f, 0.0f, 1e9f, 1.0f };
    const float lo = V.eng.range[R_BRAKE][0], hi = V.eng.range[R_BRAKE][1];
    if (!V.eng_ok || !(hi > lo) || V.eng.rpm_limit <= 0 || n_gears_ < 2) return b;
    const bool channel = eb_hold || (st && (st->eb_held || st->eb_alloc));   // (a stage the brake channel holds is the channel's, not the driver's lever: the held-stage rule keeps it)
    const float x_ret = S.lv_ret, x_eb = channel ? 0.0f : S.lv_eb, x = fmaxf(x_ret, x_eb);
    if (!(x > 0.01f)) return b;
    b.n = x_ret >= x_eb ? (int)T.retarder_steps : 3;
    b.k = (int)clampf(ceilf(x * (float)b.n - 0.001f), 1.0f, (float)b.n);
    float rho = 1.0f;
    for (int g = 2; g <= n_gears_; g++) { const float r = rpm_at(g - 1, 10.0f), q = rpm_at(g, 10.0f); if (q > 1.0f) rho = fmaxf(rho, r / q); }
    const float s = (hi - lo) / (float)b.n;
    b.U = fminf(lo + (float)b.k * s, 0.93f * V.eng.rpm_limit);
    // The lowest band is at least one downshift wide above the bottom of the brake range: a truck that sits at lo must be able to take one downshift and land INSIDE the band, with the room to rise a little
    // before the top (lo x the box's largest step). Without it a band of one slice (n = 5: 1500 - 1700 against a step of 1.3) holds no gear below the one we are in, the speed rising against the lever finds
    // no downshift that lands inside, and the box leaves the band by its top instead (n = 5 step 1 at 40 t on -6 %: 144.8 km/h, rows C and D). It never goes over the top of the next step's band.
    if (b.k == 1 && b.n >= 2) b.U = fmaxf(b.U, fminf(fminf(lo + 2.0f * s, 0.93f * V.eng.rpm_limit), lo * rho));
    b.L = b.k >= 2 ? fminf(lo + (float)(b.k - 2) * s, b.U / rho) : 0.0f;
    // The crossover (owner's choice a, 12-11: "revs for revs' sake are annoying"): the rpm of a gear above which the next taller gear, at the rpm it lands at, gives at least the power of this one (bands_cross, from
    // the engine's curve and the box's largest step: one number for the truck, no gear in it, so that a shift does not move the band). Above it the lower gear pays for its revs; the high steps (their top over the
    // crossover) get no more than that while the speed is held, and the top moves up with the speed's rise (lever_top).
    const eng_bands *bb = bands_now();
    if (bb && bb->ok && rho > 1.001f) b.X = bands_cross(&V.eng, bb, rho);
    b.rho = rho;
    b.on = 1;
    return b;
}

// The gear the lever's band wants (cur = stay); *why says why where it shifts.
// The top of the band: the crossover X while the speed is held, up to the step's own top U as the speed rises under the lever. rise = the speed's rise as a share of the lever's own deceleration in
// the gear (0 = held or falling, 1 = the road overpowers the lever's whole deceleration): a cost that grows with the rise, not a wall; never under the bottom of the band.
static void lever_top(lever_band &b, float rise)
{
    if (!(b.U > b.X)) return;
    b.U = b.X + clampf(rise, 0.0f, 1.0f) * (b.U - b.X);
    b.L = fminf(b.L, b.U / b.rho);   // (a band is never narrower than the box's largest step, or some speeds have no gear inside it: it widens downward, as lever_band_get does)
}
static int lever_band_pick(int cur, int n, float v, const lever_band &b0, int lb_was, int *why)
{
    lever_band b = b0;
    const double now = sim_s(), since = now - S.shift_at;
    const float m = V.mass > 0 ? V.mass : VEHICLE_MASS_FALLBACK;
    const float dur = V.tc ? 0.3f : fmaxf(sl_dur_est(), V.eng.shift_time);
    // the lever's event: a new pull, a new step, or a new gear starts the speed's bookkeeping over (the lowest speed since, and since when the lever was held). The rule is not asked while a shift's gap
    // is open, so the lever is the same one if the rule comes back within a shift and the hunt guard's dwell.
    const bool same = lb_was || (S.lb_at > 0 && now - S.lb_at <= dur + lv_dwell_ud());
    const double dt_ = S.lb_at > 0 && now - S.lb_at < 0.5 ? now - S.lb_at : 0.0;
    if (!same || S.lb_k != b.k) { S.lb_pull = now; S.lb_left_g = 0; S.lb_left_e = 0; S.lb_vmin = v; }
    // (the speed counts as gained only once the lever has been held for a shift and its latency: its retarder has built up, and a tap shorter than that is never answered, promise P2)
    else if (S.lb_gear != cur || now - S.lb_pull < dur + GC_LATENCY_S) S.lb_vmin = v;
    else S.lb_vmin = fminf(S.lb_vmin, v);
    S.lb_k = b.k; S.lb_gear = cur; S.lb_on = 1; S.lb_at = now;
    // R0611-RET (owner 06-11: "the retarder fades too much at low speed"): step 1 has no lower edge (a tap never shifts), so a step 1 HELD while the speed falls sat in its gear down to idle (MAN Allison gear 3,
    // 955 -> 650 rpm at 35 -> 27 km/h; Scania gear 8-10 at 920-1050 rpm) and the game's retarder, which follows the engine side (F rises with the rpm, FX1b), gave 0.2-0.4 m/s2 where it gave 0.7-0.9 at 70 km/h.
    // Once the lever has been held for a shift, its latency and the hunt guard's dwell (so a tap is still never answered) step 1 gets the lower edge that continues the series of the other steps (L_k = lo + (k - 2) s, k = 1:
    // lo - s: its own slice and the one below, so that the band of step 2 overlaps it by a slice), never above U / rho (one gear step wide, as every band); a lower gear then lands inside [L, U].
    // The edge is also the floor of an UPSHIFT at any time (the box would take it straight back once the lever has been held: 11>12 landing at 946 rpm 1 s after the pull, 12>10 3.5 s later, the hunting pair of the rows).
    float L1 = 0.0f;
    if (b.k == 1 && b.n >= 2) {
        float rho1 = 1.0f;
        for (int g = 2; g <= n; g++) { const float r = rpm_at(g - 1, 10.0f), q = rpm_at(g, 10.0f); if (q > 1.0f) rho1 = fmaxf(rho1, r / q); }
        const float lo1 = V.eng.range[R_BRAKE][0], s1 = (V.eng.range[R_BRAKE][1] - lo1) / (float)b.n;
        L1 = fmaxf(fminf(lo1 - s1, b.U / rho1), 0.0f);
        if (now - S.lb_pull > dur + GC_LATENCY_S + lv_dwell_ud()) b.L = L1;
    }
    const int maxd = p_eb_skip > 0 ? 2 : 1;
    *why = W_ENGINE_BRAKE_HOLD;
    // The model's brake in a gear (N at the wheels) against the road's push. The lever's part: the retarder step's force through the ratio (the retarder acts on the engine side, FX1b), the engine brake
    // stage's by the vehicle model (it knows the gear and the rpm); the engine's drag. The lever's part is then multiplied by what the truck measures against the model (lb_cal, below): the model's
    // start values are for an average truck (the owner's retarder gave 1.83 x the model's step 1 in the ride of 04-11), and the gear the force was learned in is not the gear it is asked for.
    const bool eb_on = S.lv_eb > 0.0f && T.motor_brake;
    const int eb_cap = 3, eb_stage = (int)clampf(ceilf(S.lv_eb * (float)eb_cap - 0.001f), 1.0f, (float)eb_cap);
    const float f_ret = V.mass_ok && S.lv_ret > 0.0f ? vehicle_ret_force((int)T.retarder) : 0.0f;
    const float cal = S.lb_cal > 0.0f ? S.lb_cal : 1.0f;
    auto lever_at = [&](int g, float vv, int from) {   // the model's lever force in gear g at speed vv, taken from the gear `from` the step's force is known in
        const float rg = rpm_at(g, vv), rf = rpm_at(from, vv);
        float f = f_ret * rg / fmaxf(rf, 1.0f);
        if (eb_on && V.mass_ok) f += m * vehicle_eb_decel_at(eb_stage, eb_cap, g, rg);
        return f;
    };
    auto brk_at = [&](int g, float vv) { return cal * lever_at(g, vv, cur) + (V.mass_ok ? drag_at(g, vv) : 0.0f); };
    auto brk_max_at = [&](int g, float vv) {   // the same with the retarder at its last step (the driver may step the lever up)
        const float rg = rpm_at(g, vv), rf = rpm_at(cur, vv);
        float f = V.mass_ok && S.lv_ret > 0.0f ? vehicle_ret_force((int)T.retarder_steps) * rg / fmaxf(rf, 1.0f) : f_ret * rg / fmaxf(rf, 1.0f);
        if (eb_on && V.mass_ok) f += m * vehicle_eb_decel_at(eb_stage, eb_cap, g, rg);
        return cal * f + (V.mass_ok ? drag_at(g, vv) : 0.0f);
    };
    const float push_now = -resist(v);   // N along the truck's way: > 0 where the road pushes it on, < 0 where it slows by itself
    // The calibration: in a steady stretch (a shift and its latency since the last shift and since the pull, no service brake) the lever's part of what the truck measures, against the model's. One
    // number for the truck (the model's error is about the truck and its retarder, not about the step), taken with the time scale of the hunt guard's dwell.
    // (the gate, each part from the learner of the lever's stage in the brake channel, core/brake.cpp bc_learn_lever: over 15 km/h, a shift and the dwell since the last shift and since the pull so that the retarder has built
    // up and the telemetry's half-second lag has settled, no service brake of ours or the driver's, and a lever force the measurement can see: over its noise, 0.1 m/s2 of the mass (BC_LV_SPREAD's own figure))
    if (V.mass_ok && v * 3.6f > 15.0f && f_ret + (eb_on ? 1.0f : 0.0f) > 0.0f && since > dur + lv_dwell_du() && now - S.lb_pull > dur + lv_dwell_du() && now - S.brake_at > dur && now - S.own_brake_at > dur && dt_ > 0.0) {
        const float fm = lever_at(cur, v, cur);
        if (fm > 0.1f * m) {
            const float meas = push_now - m * T.accel - drag_at(cur, v);   // (what brakes the truck in this gear beyond the road and the engine's drag: Newton along the road)
            S.lb_cal = cal + (clampf(meas / fm, 0.4f, 2.5f) - cal)   // (0.4 - 2.5: the band vehicle_ret_note takes a measured figure of a stage in, a figure outside it is not a retarder's)
                       * (1.0f - expf(-(float)dt_ / lv_dwell_ud()));
        }
    }
    // The speed the shift ends at: the acceleration of now carried through it. Where the model knows what the lever gives, the deceleration is the model's if that is the larger (the measured one
    // lags by half a second and is the first to be small when the lever has just come on or a gap has just closed). The deceleration assist bridges a clutch box's gap with the service brake, to
    // what the lever gave just before it, so the lever's deceleration does carry through a shift (the old engine-brake rule carried it the same way).
    float a_use = T.accel;
    // The service brake is not the lever's: the driver may let go of it during the shift (22:24:30 on 12-11: 8 -> 6 predicted for -3.2 m/s2 WITH the brake, the brake went off in the gap and the gear landed at 2384 rpm
    // instead of ~1850). While a service brake brakes (his foot, or just off it: S.svc, the same reading the braking rule below uses) the landing is predicted from the lever's own modelled deceleration alone; the
    // measured excess over the model (the brake's part) counts for what is left of it, 0.05 of the pedal being the braking rule's own threshold, so there is no switch (core values 3, 5).
    if (V.mass_ok && f_ret + (eb_on ? 1.0f : 0.0f) > 0.0f) {
        const float a_lev = (push_now - brk_at(cur, v)) / m;
        const float svc_w = clampf(fmaxf(T.in_brake, S.svc) / 0.05f, 0.0f, 1.0f);
        a_use = a_lev + (1.0f - svc_w) * fminf(a_use - a_lev, 0.0f);
    }
    // ... and the bridge fills a gap only as fast as its build-up goes (the shift's gap is shorter than the hand-over it would need): through the gap the deceleration is between the free one (the road
    // and the engine's drag) and the lever's, a triangular rise to the lever's share over the gap on average half of it.
    const float a_free = push_now / m - (V.tc ? 0.0f : sl_dec_est());
    const float a_gap = a_use < a_free ? a_free + (S.lb_beta_n > 0 ? S.lb_beta : 0.5f) * (a_use - a_free) : a_use;
    S.lb_aus = a_use; S.lb_afr = a_free;
    const float v_e = fmaxf(0.5f, v + (V.tc ? a_use : a_gap) * dur);
    auto r_at = [&](int g) { return rpm_at(g, v_e); };
    // (the rise: the speed's gain now, or since it was last at its lowest, against the deceleration the lever gives in this gear over the shift: the UP rule's own measure, continuous)
    {
        const float lever_a = V.mass_ok ? brk_at(cur, v_e) / m : 0.0f;
        const float rise = lever_a > 0.05f ? fmaxf(T.accel / lever_a, (v - S.lb_vmin) / (lever_a * dur)) : 1.0f;
        lever_top(b, rise);
    }
    if (b.k >= 2) { S.bd_at = now; S.bd_gear = cur; S.bd_floor = b.L - 50.0f; S.bd_lead = 0.0f; S.bd_after = S.shift_at + min_gap(); }   // (the downshift the deceleration assist's hand-over waits for, stated as the held-stage rule stated its own: the band's bottom less its 50 rpm of margin and no lead. K3 measured: with the shift's lead (the speed at the shift's end) the hand-over began a shift too early and cost ~1 request x s more service brake at the top step, 3.86 against 2.82 on -6 % at 40 t)
    // OUTSIDE the band: the fewest shifts that put the rpm in it (the nearest gear that is inside; no gear inside: the nearest to it, if that is nearer than the gear we are in), at most maxd gears
    // at once. A landing inside the band must also stay over its bottom for the dwell at the deceleration of now (a truck braking hard is under it again within a second: the rpm comes into the
    // band by itself there, and an upshift into it would be taken straight back), and a downshift must not land over the rev ceiling.
    auto dist = [&](int g) { const float r = r_at(g); return fmaxf(fmaxf(b.L - r, r - b.U), 0.0f); };
    // (the gear in is outside the band when it is outside it NOW or at the speed the shift would end at: the speed of the landing is what a shift to another gear meets, but the gear we hold does not wait for a
    // shift to take place, and a speed that the road pushes up across the gap must not hide a rpm that is under the band's bottom this very second: rows A, K3 round 2)
    const float r_now = rpm_at(cur, v);
    // (and before a shift decided now can begin: the decision's own latency, GC_LATENCY_S, at the deceleration (the rise) of now)
    const float r_lead_lo = rpm_at(cur, fmaxf(0.5f, v + fminf(a_use, 0.0f) * GC_LATENCY_S)), r_lead_hi = rpm_at(cur, v + fmaxf(T.accel, 0.0f) * GC_LATENCY_S);
    const float r_mid = rpm_at(cur, 0.5f * (v + v_e));   // (the gear in is judged at the speed half way through the shift: the gear we shift to then lands inside the band with room on its top side for a landing error)
    const float d_cur = fmaxf(fmaxf(fmaxf(b.L - r_mid, r_mid - b.U), 0.0f), fmaxf(fmaxf(fmaxf(b.L - r_now, r_now - b.U), fmaxf(b.L - r_lead_lo, r_lead_hi - b.U)), 0.0f));
    if (d_cur > 0.0f) {
        const float v_f = fmaxf(0.5f, v + fminf(a_use, 0.0f) * (dur + lv_dwell_ud()));
        int best = cur; float bd = d_cur;
        // Of the gears that land inside the band (by the prediction of the speed the shift ends at) the one that SITS in the middle of it for the time it has to hold it is taken: the landing speed is a
        // prediction (how much of the lever's deceleration the gap lets through is the truck's own), and the gear then has to stay inside until the next shift could be made, the minimum gap and the
        // decision's latency later, at the deceleration (the rise) of now. The middle of the band (geometric: the band is a ratio, and the gears are) at the middle of that time is the landing that is
        // safest against an error of either side; the fewest gears no longer decides when two land inside (rows A: a landing 13 rpm over the bottom, predicted, was 13 under it).
        const float v_c = fmaxf(0.5f, v_e + (a_use < 0.0f ? a_use : T.accel > 0.0f ? T.accel : 0.0f) * 0.5f * (min_gap() + GC_LATENCY_S));
        const float mid = sqrtf((b.k == 1 && b.L > 0.0f ? b.L : fmaxf(b.L, b.U / 1.3f)) * b.U);   // (step 1 has no lower edge: the band is one gear step wide, 1.3 being the largest step of an ordinary box; a step 1 held long enough has the edge of R0611-RET above, and its middle is the middle of [L, U]: a landing near U would be undone by the gear above as soon as it holds)
        float best_c = 1e9f;
        // Rising speed wins (the owner's coordinator, K3 round 2): while the speed rises under the lever there is never an upshift, and the band's top is no limit then. An upshift lowers the engine brake's force
        // (the retarder's through the ratio, the engine's drag): it can never help a speed that is already rising. "Rising" is the UP rule's own measure: the speed is rising now, or has gained more since it was last
        // at its lowest than the gap of a downshift costs. The rpm may go above the band's top up to the brake range's top or the rev limit's safe line (the rev-limit rule's own 0.99 of the limit).
        const bool rising = T.accel > 0.0f || v - S.lb_vmin >= brk_at(cur, v_e) / m * dur;
        for (int k = 1; k <= maxd; k++) for (int dir = -1; dir <= 1; dir += 2) {
            const int g = cur + dir * k;
            if (g < 1 || g > n) continue;
            if (dir < 0 && r_at(g) > 0.93f * V.eng.rpm_limit) continue;
            if (dir > 0 && (rpm_at(g, v_f) < b.L || rising)) continue;
            // (and an upshift out of the band's top only where the lever, stepped up to its last step, holds the speed in the gear above by the model: with the top at the crossover a gear over it is common, and a gear
            // above that cannot hold even then lets the speed rise, the downshift follows, and the box cycles)
            if (dir > 0 && V.mass_ok && (push_now - brk_max_at(g, v_e)) / m >= 0.0f) continue;
            const float dg = dist(g);
            if (dg > 0.0f) { if (dir < 0 && best_c == 1e9f && dg < bd - 1.0f) { best = g; bd = dg; } continue; }   // (an upshift only where it lands INSIDE: out of the band at its top a gear that is still over it takes brake away for nothing)   // (inside by the prediction: counted below; none inside: the nearest to it)
            const float c = fabsf(logf(fmaxf(rpm_at(g, v_c), 1.0f) / mid));
            if (c < best_c) { best_c = c; best = g; bd = 0.0f; }
        }
        if (best != cur && since > min_gap()) { S.lb_left_g = cur; S.lb_left_e = 0.0f; *why = best > cur ? W_LEVER_UP : W_ENGINE_BRAKE_LEVEL; return best; }   // (the gear left is remembered for the price of undoing the shift, whichever branch made it)
        return cur;
    }
    if (!V.mass_ok) return cur;   // (the model's forces need the mass)
    // INSIDE the band
    const float rev_w = sc_reversal_weight(S.dm_last, 0.0f, S.sh_braked, 1.0f);   // (G8: the pedal is off and the lever on: the situation of the last shift is the same one)
    // UP in the band: the speed keeps rising and a downshift pays
    if (cur > 1 && T.accel > 0.0f && since > min_gap()) {
        const int l = cur - 1;
        // the landing must stay under the top of the band through the dwell after the shift: the speed at its end plus what the speed still gains in the lower gear, by the model's force there
        // (through the gap itself, with the lever gone, the road alone moves the speed: a bridge of the service brake is not counted on)
        const float v_gap = fmaxf(v_e, v + (push_now / m - (V.tc ? 0.0f : sl_dec_est())) * dur);
        const float v_h = v_gap + fmaxf(fmaxf(0.0f, (push_now - brk_at(l, v_e)) / m), T.accel) * lv_dwell_du();   // (the model's shortfall in the lower gear, or the rise the truck has now, whichever is more: the model may be wrong)
        const bool room = rpm_at(l, v_h) <= b.U && r_at(l) <= b.U;
        const bool guard = S.shift_dir > 0 && since < lv_dwell_ud() * rev_w;   // (after an upshift: the hunt guard, which fades as the situation changes)
        const float lost = brk_at(cur, v_e) / m, gained = v - S.lb_vmin;
        // (the last chance: the speed keeps rising, and the landing in the lower gear is inside the band now but would not be after waiting another dwell at the rise of now: the shift that pays later has no room
        // by then, and the box would leave the band by its top instead (a band that is one downshift wide only: n = 5 step 1 at 40 t on -6 %). The lever has been held for a shift and a dwell: a tap is never answered)
        const bool last = room && rpm_at(l, v_h + T.accel * lv_dwell_du()) > b.U && now - S.lb_pull > dur + lv_dwell_du();
        const bool undo = l == S.lb_left_g;   // (the gear our own upshift just left: undoing it costs G8's reversal price, as the upshift's own rule charges for undoing a downshift)
        if (room && !guard && (gained >= lost * dur * (undo ? 1.0f + rev_w : 1.0f) || last)) {
            S.lb_left_g = cur; S.lb_left_e = fmaxf(0.0f, T.accel - (push_now - brk_at(cur, v_e)) / m);
            *why = W_ENGINE_BRAKE_LEVEL;
            return l;
        }
    }
    // DOWN in the band: it is not rising, and the gear above holds the speed
    // (it is not rising now AND has not gained more since it was last at its lowest than the gap of a downshift costs: the UP rule's own measure, so that the two cannot both be true. A truck that gains
    // 30 km/h in half a minute is not under control because the acceleration of this very tick is under zero: C, K3 round 2)
    if (cur < n && T.accel <= 0.0f && v - S.lb_vmin < brk_at(cur, v_e) / m * dur && since > min_gap()) {
        const int h = cur + 1;
        const bool guard = S.shift_dir < 0 && since < lv_dwell_du() * rev_w;
        const float v_f = fmaxf(0.5f, v + fminf(a_use, 0.0f) * (dur + lv_dwell_ud()));   // where the speed is when the hunt guard lets the box come back down
        const float floor_h = fmaxf(fmaxf(b.L, L1), idle_rpm() + ioff(250));   // (the stall rule's own landing: a gear that lands lower is one it takes straight off again)
        const bool fits = r_at(h) >= floor_h && rpm_at(h, v_f) >= floor_h && r_at(h) <= b.U;
        const bool back = h == S.lb_left_g;   // the gear just left: the price twice (decision 63), and the model was optimistic there
        const float short_h = (push_now - brk_at(h, v_e)) / m + (back ? S.lb_left_e : 0.0f);   // m/s2, < 0 = it holds
        const float dv_gap = fmaxf(push_now, 0.0f) / m * dur;                                   // what the gap gives the road
        // (after a downshift the evidence that the gear above holds counts from that shift, not from the pull: the shift changed the situation, and an upshift that reverses it must win back its price in the time since)
        const float held = (float)(S.shift_dir < 0 ? fmin(now - S.lb_pull, since) : now - S.lb_pull);
        if (fits && !guard && short_h < 0.0f && -short_h * held >= dv_gap * (back ? 1.0f + rev_w : 1.0f)) { S.lb_left_g = cur; S.lb_left_e = 0.0f; *why = W_LEVER_UP; return h; }
    }
    return cur;
}

// The rules of the gear: each one that has an opinion posts it (ask) with the priority of its kind, in the order the rules ran when they were a chain
// of returns; the first that fires ends the chain.
static void decide_gear_rules(void *self)
{
    int cur = sense_gb_gear(self), n = n_gears();
    float v = fabsf(T.speed);
    double now = sim_s();
    const int lb_was = S.lb_on;   // (the band rule ran at the last decision: it sets this again where it runs)
    S.lb_on = 0;
    if (!V.eng_ok || n < 2 || cur < 0 || cur > n || wheel_radius() <= 0.1f) { ask(0, W_NO_DATA, GP_PROTECT); return; }

    float f_res = resist(v);
    float brake = fmaxf(T.in_brake, T.eff_brake);
    if (brake > 0.05f) S.brake_at = sim_s();
    int ebrake = T.motor_brake || T.retarder > 0 || S.eb_level > 0.01f;
    int roll = roll_wanted(v, f_res, ebrake, brake, now - S.pedal_on_at);
    if (S.rolling) {
        if (roll) { ask(ROLL, W_ECO_ROLL, GP_PLANNER); return; }
        S.rolling = 0;
        S.roll_at = now;
        if (cur == 0) {
            S.roll_exit = roll_exit_pick(v, ebrake, brake);
            S.roll_exit_at = now;
            ask(S.roll_exit, W_ROLL_END, GP_PLANNER);
            return;
        }
    }
    if (cur < 1) { ask(0, W_NEUTRAL, GP_OWNER); return; }
    // [lowspeed] owner=1: a pull-away keeps the start gear until the clutch is locked (+ 0.5 s), then TrueShift has the truck. Only the
    // protections shift: a locked engine under idle + 100 (it would stall), or at the limiter (the gear is too low for the speed). While the
    // clutch slips the engine speed is not the driveline's, so a low rpm is no stall there (the slip law opens the clutch for that).
    if (low_launch_hold()) {
        float er = T.rpm, lm = V.eng.rpm_limit;
        int slipping = clutch_seen() >= 0.02f;
        if ((slipping || er > idle_rpm() + ioff(100)) && !(lm > 0 && er > 0.97f * lm)) { ask(cur, W_LOW_SPEED_HOLD, GP_OWNER); return; }
        // ... and where the engine is locked under idle + 100 the protections would shift, but the owner's slip law opens the clutch for that (L1: no gear change before the lock): until the
        // pull-away has been handed over by the old rule (S.old_ours) the launch is the owner's and the game's chooser stays out of it, as it has been.
        if (!S.old_ours) { ask(0, W_LOW_SPEED, GP_OWNER); return; }
    }
    if (!S.ours) { ask(0, W_LOW_SPEED, GP_OWNER); return; }

    float rpm = rpm_at(cur, v);   // the engine's rpm: the stall speed while the converter slips, else the driveline's
    float lim = V.eng.rpm_limit;
    float thr = S.thr;
    // lift-off by the pedal itself: the game's throttle reads 0.08 at pedal 0 (24-10, 1831 s), which hid the coast
    int coast_p = (T.cruise > 0.5f ? thr : clampf(T.in_thr, 0, 1)) < 0.05f;
    double since = now - S.shift_at;
    int kick = S.kick_until > now || S.wk_until > now;
    S.up_pt = up_pt(cur);
    S.dn_pt = dn_pt(cur);
    S.floor_rpm = 0;

    if (S.mode == MODE_M) { decide_manual(cur, rpm, v, since); return; }

    // Protections, also while a hint is held. Far above the limiter (a wrong
    // gear after neutral): straight to a gear that fits the speed. On a converter the engine rpm is the stall speed while it slips,
    // so the limiter and the stall rule read the engine's rpm, not the driveline's.
    if (rpm > lim * 1.05f) { ask(pick_for_speed(v, pw_mid()), W_WRONG_GEAR, GP_PROTECT); return; }
    if (rpm > rev_line(ebrake) && cur < n) {
        const float ts_ = V.tc ? 0.3f : V.eng.shift_time;
        int h = ebrake || S.thr < 0.75f ? cur + 1
                                        : gc_full_upshift(cur, n, v, ts_, V.tc ? ts_ : fmaxf(sl_dur_est(), ts_), V.tc ? 0.0f : sl_dec_est(), 1 + (int)p_max_skip, S.tgt_rpm, 1.0f - S.tgt_kd);
        S.hold_until = 0;  // a held hint gear ends here, the next gear is TrueShift's again (friend 11-10: 2-3-4-5-6 all at the limiter)
        // The gear it goes up to must be one that holds the speed: where the lug rule below would take it off at once (the speed falls at full throttle in cur too: no gear holds it,
        // 35.6 t on 6 % with the cruise at 80, 8 -> 9 at 65 km/h and 9 -> 8 at 62: 9>8>9>8 in 2.5 s) the gear between is the answer, and with none the speed falls and takes the revs with it.
        while (h > cur && lug_rule_fires(h, v, f_res, thr)) h--;
        if (h == cur) { ask(cur, W_REV_LIMIT_HOLD, GP_PROTECT); return; }
        ask(h, h > cur + 1 ? W_REV_LIMIT_SKIP : W_REV_LIMIT, GP_PROTECT);
        return;
    }
    // A locked engine under idle + a margin (it would stall): one or two gears down, the highest that sits clear of the stall point,
    // taken before the rpm sags under idle. While the clutch slips (a pull-away the smooth-start law owns) the driveline rpm is not the
    // engine's, so a low rpm there is no stall: the slip law opens the clutch for it (ride 30-10: gear 6 at 8 km/h, clutch 0.71, rpm 640).
    // A converter is the same: while it slips the engine sits at the stall speed, never at the driveline's low rpm.
    // While the clutch slips (the engine held at the slip rpm, core/vehicle.h) the layers' premise, the engine turning with the wheels, is false: the clutch is making the launch and the gear it
    // started in is the one that was picked for it (sc_launch_pick). It ends at the lock, which the telemetry shows (the engine's rpm falls to the driveline's).
    if (vehicle_slipping()) { ask(cur, W_LAUNCH_SLIP, GP_OWNER); return; }
    // ... and a locked engine under the line that the pull is taking out of it (the truck speeds up) is leaving the line, not stalling on it: the lock of a launch sits at the slip rpm, a hair from the line.
    if (rpm < stall_line() && cur > 1 && clutch_seen() < 0.02f && !(T.accel > 0.0f) && !tc_relief(cur, v, S.f_need)) { ask(pick_down(cur, v, idle_rpm() + ioff(250), false, 2), W_STALL, GP_PROTECT); return; }

    // A hint gear is held, but not up to the limiter: pulling past the top of the power range hands it back
    // (friend 11-10: hint down to 2 at 8 km/h, then 2->3->4->5->6 each at ~1900 rpm with 30% throttle).
    if (S.hold_until > now) {
        if (!(rpm > pw_hi(cur) && S.thr > 0.05f)) { ask(cur, W_HINT_HOLD, GP_OWNER); return; }
        S.hold_until = 0;
        logv(1, "hint hold ended at %.0f rpm (top of the power range)", rpm);
    }

    if (roll && since > 1) {
        S.rolling = 1;
        S.roll_from = cur;
        g_rolling = 1;  // right away: Cruise keeps the game from canceling cruise when the box goes to neutral
        S.roll_at = now;
        S.roll_kmh = v * 3.6f;
        S.roll_cc = T.cruise > 0.5f; S.roll_mode = S.mode; S.roll_prot = 0;
        ask(ROLL, W_ECO_ROLL, GP_PLANNER);
        return;
    }

    // Route look-ahead (navigation on): what comes in the next seconds.
    int rk = route_on();
    float crest = rk ? route_crest_m(v * 10) : -1;
    int top_near = crest >= 0 && crest < v * 8;
    float top_m, top_floor = 0;
    int top_ok = top_reachable(cur, v, &top_m, &top_floor);
    if (top_ok) { S.top_at = now; S.top_floor = top_floor / 3.6f; }
    // ... and over the top: the profile no longer sees the last meters of the climb, the filtered pull lags behind
    int top_over = !top_ok && now - S.top_at < 4 && now >= S.top_at && v > S.top_floor && !(rk && route_max_grade(0, v * 4) > 1.0f);

    // Deceleration assist ([decel] downshift_assist): while its service brake is needed to reach the lever's target, the order is the
    // retarder, then a lower gear, then the service brake. A lower gear puts the engine into its own brake range, where the engine
    // brake and the retarder (through the lower gear's ratio) do more than at the rpm of now. The lowest gear whose rpm after the shift
    // is inside the range, short of its top by a fifth of its width and of the rpm limit (88 %), is taken at once (two gears when that
    // lands inside the range too and ebrake_skip allows it).
    gear_stand stand;
    const bool have_stand = gear_stand_get(&stand);
    const int eb_hold = have_stand && stand.hold_until > 0 && sim_s() < stand.hold_until;   // the channel took the stages off a moment ago: keep the gear, they come back
    const lever_band lb = lever_band_get(have_stand ? &stand : nullptr, eb_hold, n);        // the driver's lever as a band (WP-K3); on = 0 where it is not the driver's lever
    if (have_stand && stand.drag_n > 0 && coast_p && cur > 1) {   // (the brake channel's retarder is fed full and the service brake still helps, or Decel's lever needs the service brake: a standing request, core/gearreq.h)
        float cap = 0;
        S.ad_at = now; S.ad_gear = cur; S.ad_lead = V.eng.shift_time; S.ad_after = S.shift_at + min_gap();
        int l = cur;
        if (stand.kind == GSK_LEVER) {   // the driver holds the lever: the lowest gear that lands in the engine's brake range, as asked ...
            l = assist_down(cur, v, &cap);
            while (lb.on && l < cur && rpm_at(l, v) > lb.U) l++;   // ... but not over the top of the step's band: the owner chooses the step (decision 70)
        }
        else l = stand_drag_pick(cur, n, v, stand, now);            // the channel's wish: the drag gear, if the shift pays
        S.ad_cap = cap;
        if (l < cur && since > min_gap()) {
            if (stand.kind != GSK_LEVER) logv(1, "brake drag: %s", s_drag_terms);
            ask(l, W_ASSIST_DOWN, GP_BRAKING);
            return;
        }
    }

    // Engine brake / retarder: keep the rpm up, more for a stronger stage; the
    // full brake range when the game's "downshift with engine brake" is on.
    if ((T.motor_brake || S.eb_level > 0.01f || eb_hold) && thr < 0.05f) {
        if (lb.on) {   // the driver's lever (a retarder step, the engine brake held): the step's band, WP-K3 (decision 70), replaces the rule below for it
            int why_b = W_ENGINE_BRAKE_HOLD;
            const int g = lever_band_pick(cur, n, v, lb, lb_was, &why_b);
            ask(g, g != cur ? why_b : W_ENGINE_BRAKE_HOLD, GP_BRAKING);
            return;
        }
        float lo = V.eng.range[R_BRAKE][0], hi = V.eng.range[R_BRAKE][1];
        int full = p_eb_down > 0 || (p_eb_down < 0 && S.eb_full) || (have_stand && stand.eb_held) || eb_hold;   // (the channel holds the speed with the stages: the gear where the brake works, whatever the stage)
        float keep = full ? lo : lerpf(fminf(rs_brake(by_mode(p_eb_light - 100, p_eb_light, p_eb_light + 150)), lo), lo, S.eb_level);
        S.bd_at = now; S.bd_gear = cur; S.bd_floor = keep - 50; S.bd_lead = 0; S.bd_after = S.shift_at + min_gap();
        if (rpm < keep - 50 && cur > 1 && since > min_gap()) {
            // The stage the brake channel holds is the channel's second stage (core/brake.cpp): its gear stays inside the bands up to the gear's crossover, the owner's noise rule
            // (02-11: engine braking never in the temporary band) that the pedal-off descent's drag gear keeps, and not in the engine's brake range above it. The driver's own lever
            // asks the brake range, as it always did.
            const eng_bands *bb = bands_now();
            const bool in_bands = have_stand && (stand.eb_held || eb_hold) && bb->ok;
            auto cap_ok = [&](int g) { return !in_bands || rpm_at(g, v) <= gc_cross(bb, g, n, v); };
            int l = pick_down(cur, v, full ? 0.5f * (lo + hi) : fminf(keep + 150, hi - 100), true);
            while (l < cur && !cap_ok(l)) l++;
            // two at a time when the second still fits under the brake range: every downshift costs a throttle blip
            // from the game (Damien 11-10: "gas and engine brake at once", 9->8->...->2 one by one)
            // (05-11: and only when the second lands under the stage's ceiling: stage 1 the top of the eco band, stage 2 the power peak, the full range for the last stage,
            // the descent or the game's downshift setting. Ride: stage 1 at 50 km/h took 4>2 at 2186 rpm.)
            float skip_lid = hi - 100;
            if (!full && bb->ok) skip_lid = fminf(skip_lid, S.eb_level < 0.5f ? bb->p2 : S.eb_level < 0.9f ? bb->p3 : skip_lid);
            if (p_eb_skip > 0 && l == cur - 1 && cur > 2) {
                float r2 = rpm_at(cur - 2, v);
                if (r2 < skip_lid && r2 < V.eng.rpm_limit * 0.93f && cap_ok(cur - 2)) l = cur - 2;
            }
            if (l < cur && rpm_at(l, v) < hi) { ask(l, full ? W_ENGINE_BRAKE : W_ENGINE_BRAKE_LEVEL, GP_BRAKING); return; }
        }
        ask(cur, W_ENGINE_BRAKE_HOLD, GP_BRAKING);
        return;
    }

    // Also slowing hard with the pedal off and no brake seen (a brake the telemetry misses, AEB), and the
    // gear is judged at the speed the shift will end at (ride: braking from 70, 10->8 only at 20 km/h and 638 rpm, then the stall rule twice)
    if (brake > 0.05f || (coast_p && T.accel < -0.8f)) {
        // stay above the game's neutral threshold, which rises with brake^2
        float b = fminf(brake, 1.0f), opens = 0.8f * idle_rpm() + b * b * idle_rpm();
        // (the service brake alone, 05-11: no revs for the brake, the gear goes down only where it must: just over the stall line and the game's neutral threshold)
        const bool svc_only = (T.in_brake > 0.05f || S.svc > 0.05f) && !(T.retarder > 0);   // (his foot on the pedal, or just off it while the brake still brakes: S.svc; the brake channel's own service brake keeps the old landing)
        float floor = svc_only ? fmaxf(stall_line() + ioff(30), opens + ioff(100))
                               : fmaxf(fmaxf(idle_rpm() + ioff(250), eco_dn(cur) - 100), opens + ioff(250));
        // (the speed at the end of a shift started now, at the deceleration of now: under the brake the revs fall through a slow
        // box's shift, and a gear chosen for the speed of now lands under the floor or at the stall rule)
        float v_l = fmaxf(0.5f, v - fmaxf(0.0f, -T.accel) * (V.eng.shift_time + 0.25f));
        S.bd_at = now; S.bd_gear = cur; S.bd_floor = floor; S.bd_lead = V.eng.shift_time + 0.25f; S.bd_after = S.shift_at + 0.5;
        if (rpm_at(cur, v_l) < floor && cur > 1 && since > 0.5) {
            // land high, so that fewer downshifts are needed on the way down -- with the retarder working. On the service brake alone the owner wants no revs (05-11:
            // "when I touch the brake I do not want it higher in the revs; only the retarder or the engine brake"): the tallest gear that clears the floor, one step at a time.
            int l = cur;
            if (svc_only) { for (int g = cur - 1; g >= 1; g--) { l = g; if (rpm_at(g, v_l) >= floor) break; } }
            else l = pick_down(cur, v_l, eco_up(1) + by_mode(-300, -200, -100), true);
            // (land where the rev-limit rule will not fire at the speed of now either, if the braking lets go: a gear higher up, or none; the stall rule has the rest)
            const float wall = landing_wall(cur, v);
            // (ride 05-11: a 0.06 brake took 2>1 at 21 km/h, 1981 rpm, twice; the old target was eco_up(1) - 300 for every brake)
            while (l < cur && rpm_at(l, v) > wall) l++;
            if (l < cur) { ask(l, W_BRAKING, GP_BRAKING); return; }
        }
        ask(cur, W_BRAKING_HOLD, GP_BRAKING);
        return;
    }

    if (since < min_gap()) { ask(cur, W_MIN_GAP, GP_SOFT); return; }

    // The last upshift has no pull (strain built up): back down, and learn.
    // The layers may land a gear in the lug band on purpose while pulling away (the torque pedal then holds the
    // throttle high); the gear pulls when the speed has passed the upshift's own speed again, so that is no "no pull"
    if (S.up_gear == cur && S.strain > 1.0f && cur > 1 && !(v * 3.6f > S.up_kmh)) { ask(cur - 1, W_NO_PULL, GP_SOFT); return; }

    // A gear that cannot even hold the speed at full throttle any more (a steep climb): no waiting for the throttle to come up (cruise and
    // ACC raise it slowly) or for the rpm to sag (Damien 24-10, 545-553 s: 10-11 % at 43 km/h in 12, held to 759 rpm and 35 km/h, the lead
    // pulled away 13 -> 58 m). It needs real throttle (> 0.25) and a speed that is falling, so a burst of gas on the flat does nothing. This is
    // physics, the one downshift rule left before the layers: a full pedal that merely asks more than the gear gives (thr > 0.8, the speed
    // not yet falling) is the layers' judgement, they price a gear that cannot deliver the wish. Only in the lug zone (the rpm clause stays: without
    // it the rule also fires at 1600 rpm on a 6 % climb where no gear holds the speed, lands over the limiter and hunts; measured in the commit).
    float fmax = force_at(cur, v, 1.0f);
    int lug = lug_rule_fires(cur, v, f_res, thr);
    if (cur > 1 && lug) {
        // Almost over the top: hold on, it gets easier in a moment.
        if (top_near && rpm > idle_rpm() + ioff(300)) { ask(cur, W_TOP_AHEAD, GP_SOFT); return; }
        // A top this gear still gets over within the band: let the speed sag instead (PCC).
        if (top_ok || top_over) {
            if (top_ok && !S.crest_said) {
                S.crest_said = 1;
                DEV_NOTE(pcc_top());
                logv(1, "top in %.0f m: holding %d at %.0f km/h (gets over above %.0f km/h)", top_m, cur, v * 3.6f, top_floor);
            }
            ask(cur, top_ok ? W_TOP_REACHABLE : W_OVER_TOP, GP_SOFT);
            return;
        }
        // In the cruise band (PCC): a few km/h under the set speed and this gear still pulls on this grade (not
        // just lugging along): let the speed come back slowly instead of a downshift for pull. Not with the driver's
        // own pedal (overtaking with cruise on wants the pull).
        if (T.cruise > 0.5f && !kick && T.in_thr < 0.1f && v >= T.cruise - crest_band() && rpm > band_rpm() && (fmax - f_res) / V.mass > 0.06f) {
            ask(cur, W_CRUISE_BAND, GP_SOFT);
            return;
        }
        int l = pick_down(cur, v, pw_mid());
        if (l == cur) l = cur - 1;
        // A cruise that defends a set speed with a proportional throttle takes a sag as a price (lug_gain). The driver's foot asks for the pull he presses: for him the fact stands.
        const bool priced = lug_priced(v);
        if (!priced || lug_best_gain(cur, v) > 0.0f) {
            if (priced) { S.lug_back = cur; S.lug_back_at = now; }   // (the way back is part of the price: below)
            ask(l, W_LUGGING, GP_SOFT);
            return;
        }
        // (the sag costs less than the shift and the shift back: the gear stays, the layers judge it)
    }

    // The way back, which the price of the lugging downshift counted: the gear it left holds the road again with the margin of the rule that took us out of it (a gear 5 % over the resistance), and
    // lands above the lug floor. Only for the cruise whose sag is a price, only after that shift, and not before the dwell after a downshift: the road eased, the revs fall again. (Without it the
    // layers, which carry no taste on a descent under a controller and decide nothing in its coast, left the box in the lower gear for good: T6c, 12 > 11 at the top of the first hill, 11 to the end.)
    if (S.lug_back > cur && cur >= 1) {
        const float v_up = fmaxf(0.5f, v - (f_res / (V.mass > 0 ? V.mass : VEHICLE_MASS_FALLBACK) + sl_dec_est()) * fmaxf(sl_dur_est(), V.eng.shift_time));
        if (!lug_priced(v) || now - S.lug_back_at > 120.0) S.lug_back = 0;
        else if (coast_p && since > dwell_du() && force_at(S.lug_back, v, 1.0f) >= 1.05f * f_res && rpm_at(S.lug_back, v_up) >= bands_now()->floor && rpm_at(S.lug_back, v_up) >= bands_now()->p1) {
            ask(S.lug_back, W_UP, GP_SOFT);
            return;
        }
    } else S.lug_back = 0;

    // Brake air low (Damien: the tank ran empty twice): the compressor runs on the engine, and at ~900 rpm it barely
    // keeps up. Below air_charge_psi keep the rpm up: no upshift below air_charge_rpm, a downshift when well under it.
    float air_charge = air_edge_psi(p_air_psi, AIR_MARGIN_CHARGE);   // the truck's warning + 35 with [core] air_from_truck
    if (air_charge > 0 && T.air_psi > 1 && T.air_psi < air_charge && thr > 0.02f) {
        static double said;
        if (sim_s() - said > 10) { said = sim_s(); logline("air %.0f psi: keeping the rpm up to charge it", T.air_psi); }
        if (rpm < rs_abs(idle_rpm(), p_air_rpm) - 250 && cur > 1) {
            int l = pick_down(cur, v, rs_abs(idle_rpm(), p_air_rpm) + 100);
            if (l < cur && rpm_at(l, v) < V.eng.rpm_limit * 0.9f) { ask(l, W_AIR_CHARGE, GP_BRAKING); return; }
        }
        if (cur < n && drv_at(cur + 1, v) < rs_abs(idle_rpm(), p_air_rpm)) { ask(cur, W_AIR_CHARGE, GP_BRAKING); return; }
    }

    // the layered powertrain (core/powertrain.h) is the soft choice: the hard rules above stand, the layers get the gears they may take and pick one
    // with a single score (the shift points, the early upshift and the scores of gearscore.h were the old choice: tag old-gear-logic)
    int why = W_NONE;
    const int g = gear_layers(cur, n, v, rpm, thr, f_res, coast_p, since, now, &why);
    ask(g, why, GP_SOFT);
}

// ---------------------------------------------------------------- hook stubs

// ---------------------------------------------------------------- what a shift costs (the layers' landing estimate)
// The layers decide a gear for the rpm it lands at after the shift: the speed at the decision less what the truck loses until the gear is in.
// That is not f_res/m x the shift time: the gear is in a good while after the decision (the game's hook, the clutch) and at crawl speeds the
// truck loses more than its resistance says (41 t, ETS2 ride: decided 1>4 for a landing at 737 rpm, landed at 651, under the no-go). So the
// decision is timed (when, speed, the resistance then) and the gear coming in is measured against it: the seconds in between and the
// deceleration beyond the resistance, each a small running average per truck (reset with the truck). Until a few shifts are in, a prior:
// a duration of 1.25 shift times + 0.15 s and 0.1 m/s2 extra deceleration.
static float sl_prior_dur() { return 1.25f * V.eng.shift_time + 0.15f; }
static float sl_dur_est()
{
    float d = S.sl_n > 0 ? S.sl_dur : sl_prior_dur();
    return fmaxf(d, V.eng.shift_time);
}
static float sl_prior_dec() { return 0.1f; }   // m/s2 beyond the resistance until a shift has been measured: at crawl speeds (a 41 t truck) a shift loses well over f_res/m
static float sl_dec_est() { return S.sl_n > 0 ? S.sl_dec : sl_prior_dec(); }
// The deceleration through the gap of a shift the lever's band rule made. The rule plans the speed the shift ends at (the landing it chooses its gear for), and the deceleration through the gap lies
// between the free one (the road and the engine's drag) and the lever's (the deceleration assist bridges the clutch box's gap with the service brake, to what the lever gave just before, as fast as
// its build-up goes): where between, depends on the truck and the assist, so it is measured, one number for the truck, as the shift losses are (the same running average, 2 + n shifts, from a prior of
// halfway: the build-up's triangle).
static void lever_gap_learn(int prev, int g, double now, float v)
{
    const bool ok = S.lb_dec && S.dec_for == g && S.dec_from == prev && prev > 0 && g > 0 && !V.tc && !S.rolling;
    S.lb_dec = 0;
    const double dt = now - S.dec_at;
    const float span = S.lb_afr - S.lb_aus;   // m/s2 between the free and the lever's deceleration: a gap whose two ends are the same one tells nothing
    // (a shift decided before the lever had built up says nothing about the bridge: the retarder takes a shift and the decision's latency to come on, and the first shift after a pull loses far less than the
    // steady lever's deceleration says)
    if (!ok || !(dt > 0.05) || dt > 6.0 || span < 0.3f || T.in_brake > 0.05f || S.dec_at - S.lb_pull < sl_dur_est() + GC_LATENCY_S) return;
    const float meas = clampf(((v - S.dec_v) / (float)dt - S.lb_afr) / (S.lb_aus - S.lb_afr), 0.0f, 1.0f);
    if (S.lb_beta_n == 0) S.lb_beta = 0.5f;
    S.lb_beta += (meas - S.lb_beta) / (2.0f + (float)(S.lb_beta_n < 8 ? S.lb_beta_n : 8));
    S.lb_beta_n++;
}
static void shift_loss_learn(int prev, int g, double now, float v)
{
    int ok = S.dec_for == g && S.dec_from == prev && prev > 0 && g > 0 && !V.tc && !S.rolling;
    double dt = now - S.dec_at;
    S.dec_for = 0;
    if (!ok || !(dt > 0.05) || dt > 6.0) return;
    if (now - S.brake_at < dt + 0.3 || now - S.own_brake_at < dt + 0.3 || T.motor_brake || T.retarder > 0) return;   // braking is no shift loss
    float dv = S.dec_v - v;
    float ex = fminf(1.5f, fmaxf(0.0f, dv / (float)dt - S.dec_fres / fmaxf(V.mass, 1.0f)));
    if (S.sl_n == 0) { S.sl_dur = sl_prior_dur(); S.sl_dec = sl_prior_dec(); }
    float w = 1.0f / (2.0f + (float)(S.sl_n < 8 ? S.sl_n : 8));
    S.sl_dur += ((float)dt - S.sl_dur) * w;
    S.sl_dec += (ex - S.sl_dec) * w;
    S.sl_n++;
}

static void note_gear_change(void *self)
{
    int g = sense_gb_gear(self);
    if (g == S.last_gear) { if (g > 0 && T.gear > 0) S.out_of_gear = 0; return; }
    double now = sim_s();
    float v = fabsf(T.speed);
    int prev = S.last_gear;
    // A gear the box comes in with after it was out of gear (R0611, the Scania of 06-11: 5 > 2 at 7 km/h after the stop, 7 > 2 at 6 km/h after a slow-down, each read as the driver's hint, 'hold 6 s':
    // the pull-away revved 1st / 2nd to 1700-2100 rpm at a calm pedal, the hold ended and the layers skipped 2 > 5 and 1 > 5 from there, and the 'upshift had no pull' lesson was learned from a
    // gear the driver never chose). The gear noted last is still the one from before the stop (both hints logged 'from' that gear, 5 and 7: the box reports no neutral to this function while at rest), the telemetry saw the neutral (out_of_gear).
    // A press never goes this way: it is taken at once in bs_choose (hint_pending, why = W_HINT) and stays a hint below. What is left here is the change nobody here asked for: the box's launch.
    const int launch = S.out_of_gear && prev > 0 && g > 0 && g != S.commanded && S.ours;
    if (launch) { prev = 0; S.commanded = g; S.cmd_why = W_GAME; }
    lever_gap_learn(prev, g, now, v);
    shift_loss_learn(prev, g, now, v);
    S.shift_dir = g > prev ? 1 : -1;
    S.shift_at = now;
    S.gear_before = prev;
    S.last_gear = g;
    if (T.gear > 0) S.out_of_gear = 0;
    S.dm_last = dm_share(g);   // the demand at this shift, for the flip lock and the hunt metric
    if (S.cmd_why == W_KICKDOWN) { S.kick_until = 0; S.wk_until = 0; S.wk_d = 0; }
    if (g < prev && g > 0 && (S.cmd_why == W_KICKDOWN || S.cmd_why == W_LUGGING)) S.kd_at = now;
    int hint = prev > 0 && g > 0 && g != S.commanded && S.ours;   // (prev is 0 for a launch: not a hint)
    int why = hint ? W_HINT : g == S.commanded ? S.cmd_why : W_GAME;
    if (why == W_LAYERS && !hint && S.lift_pick == g) S.lift_done = 1;   // the layers' shift of a lift is made: one lift, one decision
    if (why == W_HINT) hint = 1;   // a shift the driver's hint asked for is a hint even when we carried it out (M mode): it is not our shift to hunt-metric
    // The situation the shift was made in, for the price of undoing it (sc_reversal_weight): the demand above (S.dm_last) and whether the reason was braking.
    S.sh_braked = why == W_BRAKING || why == W_BRAKING_JUNCTION || why == W_ASSIST_DOWN || why == W_ENGINE_BRAKE || why == W_ENGINE_BRAKE_LEVEL || why == W_LEVER_UP;
    DEV_NOTE(shift(prev, g, why_text[why], hint, now));
    if (hint) {   // a +/- press is a vote on the gearbox: this round only LOGGED, like MARK (core/driver.h (e), owner points 19.2 / 20.2); nothing learns from it
        const bool tk = S.tk_why > W_NONE && S.tk_why < W_COUNT && S.tk_at > 0;   // the box's last visible decision (the ticker's)
        // the layers' last decision (gear_layers: the gear we were in, the gear it chose, how long ago, and the terms of its candidates): what the box thought when he pressed
        char tbuf[260] = "";
        const int have = s_last_terms_at >= 0 && now - s_last_terms_at <= 5.0;
        if (have) snprintf(tbuf, sizeof tbuf, "cur %d pick %d age %.1f | %s", s_last_terms_cur, s_last_terms_pick, now - s_last_terms_at, s_last_terms);
        const driver_hint vote = { prev, g, v * 3.6f, T.rpm, clampf(T.in_thr, 0, 1), S.demand, grade_pct(), V.mass / 1000.0f, mode_label(S.mode),
                                   tk ? why_text[S.tk_why] : "", tk ? (float)(now - S.tk_at) : -1.0f, have ? tbuf : NULL };
        char line[500];
        driver_hint_line(line, sizeof line, vote);
        logline("%s", line);
    }
    // the ticker (ts_view): a shift of our own, or an eco-roll going in or out; not the game's own shifts, not the driver's hints
    if (!hint && why != W_GAME && why != W_HINT && why != W_M && why != W_NONE && ((prev > 0 && g > 0) || why == W_ECO_ROLL || why == W_ROLL_END)) {
        S.tk_why = why; S.tk_from = prev; S.tk_to = g; S.tk_at = now; S.hold_cand = 0;
    }
    sc_track_shifts(S.ours, prev, g, hint, now, S.dm_last, mode_label(S.mode));
    
    // Did the last upshift hold? A downshift out of it for lack of force (or a
    // hint down) within 25 s means it did not.
    if (S.up_gear && prev == S.up_gear && g < prev) {
        // A plain downshift only counts when power was asked for (not when
        // the driver or the cruise control lets the truck slow down).
        // A climb that started after the upshift is terrain, not a bad upshift.
        // (within 5 s it counts anyway: a climb that went on steepening right after it is what the upshift should have seen)
        int asked = S.thr >= 0.5f, same_road = grade_pct() - S.up_grade < 1.0f || now - S.up_at < 5;
        // A pedal pressed well past where it was at the upshift is a new request (an overtake after a calm stretch), not the upshift
        // failing what it was asked (sim 29-10, the V8 at 18 t: calm in 12 at 60 km/h for 22 s, a full pedal, and the lesson raised the
        // upshift rpm and the reserve on the flat for the next 25 km)
        int new_ask = clampf(T.in_thr, 0, 1) > S.up_ped + 0.3f;
        if (now - S.up_at < 25 && why && same_road && !S.up_tainted && !new_ask &&
            (hint || why == W_NO_PULL || why == W_LUGGING || (asked && (why == W_DOWN || why == W_DOWN_SKIP))))
            up_failed(why_text[why]);
        else S.up_gear = 0;
    }
    // an upshift of ours (the hold reasons 'up veto' and 'up blocked' keep the gear, so they never become the command's reason)
    if (!hint && (why == W_UP || why == W_UP_SKIP || why == W_UP_EARLY || why == W_UP_EARLY_SKIP || why == W_LAYERS) && g > prev) {
        S.up_gear = g;
        S.up_tainted = 0;
        S.up_at = now;
        S.up_grade = grade_pct();
        S.up_kmh = v * 3.6f;
        S.up_ped = clampf(T.in_thr, 0, 1);
        float fmax = force_at(g, v, 1.0f);
        S.up_pred = fmax > 1 ? clampf(fmaxf(S.f_need, resist(v)) / fmax, 0, 1) : 1;
        S.strain = 0;
    }

    if (hint && p_hint_hold > 0 && S.mode != MODE_M) {
        float hh = by_mode(fminf(p_hint_hold, 6.0f), p_hint_hold, p_hint_hold * 2);
        S.hold_until = now + hh;
        logv(1, "shift %d->%d by hint at %.0f km/h: hold %.0f s", prev, g, v * 3.6f, hh);
    } else {
        logv(1, "shift %d->%d (%s) at %.0f km/h, %.0f rpm, pedal %.2f, demand %.2f, grade %.1f%%, %.1f t, %s", prev,
                g, why ? why_text[why] : "?", v * 3.6f, T.rpm, T.in_thr, S.demand, grade_pct(), V.mass / 1000,
                mode_label(S.mode));
    }
}

// A launch the start-gear function never saw (24-10: after reversing in the yard, and right after the engine started, the game picked
// gear 2 itself with 36.7 t, and the box climbed 2>3>4>5 in 7 s; the start function says 4): at a standstill, in a low gear, the clutch
// still slipping or even before the pedal, take the physics' gear straight away: a shift now costs next to nothing. Once per standstill.
static int launch_pick(int stock, float thr, bool force = false);
static bool takeover_launch_ok(float) { return p_takeover > 0; }
static void launch_correct(void *self)
{
    if (low_owner_on()) return;   // [lowspeed] owner=1: the start gear at a standstill is the owner's (it changes a gear the game picked itself)
    static int done, prev_lcx;
    double now = sim_s();
    float kmh = fabsf(T.speed) * 3.6f;
    bool lcx = g_lv.lcx == 1;
    if (lcx && !prev_lcx) done = 0;  // launch control just armed: pick the start gear again, with the brake held
    prev_lcx = lcx;
    if (kmh > 6) { done = 0; return; }
    if (done || kmh > 2.5f || !g_active || p_launch_fix <= 0 || p_start_fix <= 0 || p_launch_max <= 0 || T.parking_brake || (T.in_brake > 0.3f && !lcx) || !V.eng_ok || (!lcx && (S.mode == MODE_M || S.mode == MODE_POWER))) return;
    int cur = sense_gb_gear(self);
    if (cur < 1 || sense_gb_state(self) != 0 || now - S.shift_at < 1.0 || S.rolling || g_lv.crawl_toggle || V.mass < 3000) return;
    // at a standstill with the pedal still up: the gear for a gentle pull-away; with the pedal down: for that pedal (the actual
    // pedal, not a 0.3 floor: a light pedal launches in a low gear, else the start gear is too tall for the closing speed)
    int pick = launch_pick(cur, clampf(T.in_thr, 0, 1));
    done = 1;
    // Both ways while the pedal is up: too low (24-10, 4->1) and too high (28-10: with a stronger engine the game's chooser started a 17 t solo truck in 6 for a 3). With the pedal down the
    // launch is on and a change UP is not made (K1 round 15): a gear change at a standstill costs the drive of its gap (0.65 s without drive on the ride of 08-11, the stall of 15:32:25: "start
    // gear 2 (game 1)" swapped 1 > 2 with the pedal at .63; the sweep's launch at a firm pedal 1 > 2 at 1.8 km/h, 2 > 1 at 5.3 and 1 > 2 again, 5.9 s to 20 km/h against 3.1 without), and the layers
    // climb from the gear in with one shift, a skip where it pays. A change DOWN stays (a start gear too tall to launch in stalls or slips for seconds).
    const bool pedal_down = clampf(T.in_thr, 0, 1) >= DRV_OFF;
    if (pick != cur && !(pedal_down && pick > cur) && takeover_launch_ok(kmh)) {
        logline("launch: the game started in %d, the physics say %d (%.1f t, grade %.1f%%): shifting at %.1f km/h", cur, pick, V.mass / 1000, grade_pct(), kmh);
        S.launch_want = pick; S.launch_at = now;
        act_gb_set_target(self, pick); act_gb_force(self);
        S.commanded = pick; S.cmd_why = W_START_CORRECTED;
    }
}

// The holds the ticker shows: a reason that kept the gear while another one was on the table (not the plain 'hold', the minimum gap, braking).
static int hold_shown(int why)
{
    switch (why) {
    case W_TOP_AHEAD: case W_TOP_REACHABLE: case W_OVER_TOP: case W_CRUISE_BAND:
    case W_SKIP_WAIT: case W_POWER_HOLDS: case W_UP_VETO: case W_ANTI_HUNT_UP: case W_ANTI_HUNT_DOWN: case W_UP_WAIT: case W_WISH_UP:
    case W_UP_BLOCKED: case W_HINT_HOLD: case W_ENGINE_BRAKE_HOLD:
        return 1;
    default:
        return 0;
    }
}

// The dashboard's view (ts_view), once per update: the gearbox of now, the M advice (shown once an advice in the same direction held 1 s,
// so a gear that moves on while the speed changes still shows; gone 0.5 s after it went) and the ticker's holds (shown once they lasted
// 0.5 s, not over a shift of ours younger than 3 s). Only notes, nothing steers.
static void view_note(void *self, int want, int why)
{
    double now = sim_s();
    int cur = sense_gb_gear(self);
    S.view_at = wall_s(); S.view_gear = cur; S.view_state = sense_gb_state(self);
    int a = m_advice(cur, fabsf(T.speed));
    if (a == cur) a = 0;
    int dir = a > cur ? 1 : a ? -1 : 0;
    if (dir != S.adv_dir) { S.adv_dir = dir; S.adv_since = now; }
    if (now - S.adv_since >= (dir ? 1.0 : 0.5) || now < S.adv_since) S.advice = a;
    else if (S.advice && dir && (S.advice > cur ? 1 : -1) == dir) S.advice = a;   // (shown: follows the gear within its direction)
    if (S.advice == cur) S.advice = 0;   // the driver took it
    int h = want == cur && cur > 0 && hold_shown(why) ? why : 0;
    if (h != S.hold_cand) { S.hold_cand = h; S.hold_since = now; }
    else if (h && now - S.hold_since >= 0.5) {
        if (S.tk_from == 0 && S.tk_why == h && S.tk_to == cur) S.tk_at = now;   // still holding
        else if (S.tk_from == 0 || now - S.tk_at >= 3 || now < S.tk_at) { S.tk_why = h; S.tk_from = 0; S.tk_to = cur; S.tk_at = now; }
    }
}

// The driver's +/- press (P6, "what the driver does wins, inside the protections"). The game's handler moves the target one gear before the update runs, and the chooser's
// answer is written over the target: a press lives only in a tick where the chooser does not run (the rpm inside the game's window and nobody forcing it). So the press is
// read here, in every mode: at rest in a gear (no shift going), a target that is neither the gear we are in nor the one we last handed back is the driver's. In an automatic
// mode only while this brain has the truck (below the hand-over speed the game's own automatic has it and shifts as asked). The press is the driver's; only the two
// protections that would take the gear back in the very next tick refuse it (hint_refusal), and a refusal is made in the tick of the press, with a line that names it.
static int hint_pending(void *self)
{
    if (sense_gb_state(self) != 0) return 0;
    const int cur = sense_gb_gear(self), tgt = sense_gb_target(self);
    if (S.mode != MODE_M && (!takeover() || !S.ours)) return 0;
    return cur > 0 && tgt > 0 && tgt != cur && tgt != S.commanded && abs(tgt - cur) <= 3 && tgt <= n_gears() ? tgt : 0;
}
// The protection that refuses a press to gear `tgt` at the speed of now, or NULL: the rev-limit rule for a gear that lands above its line (a press down), the stall rule for a
// locked gear that lands under its line (a press up). The same two lines the rules use (rev_line, stall_line). *rpm = where the gear would run.
static const char *hint_refusal(int cur, int tgt, float *rpm)
{
    const int ebrake = T.motor_brake || T.retarder > 0 || S.eb_level > 0.01f;
    *rpm = rpm_at(tgt, fabsf(T.speed));
    if (tgt < cur && *rpm > rev_line(ebrake)) return why_text[W_REV_LIMIT];
    if (tgt > cur && tgt > 1 && *rpm < stall_line() && clutch_seen() < 0.02f && !tc_relief(tgt, fabsf(T.speed), S.f_need)) return why_text[W_STALL];
    return NULL;
}
// The press and the verdict, once per tick that has a press: returns the refusing protection (NULL = taken). A refusal is a line at level 1 (the driver pressed, so he sees it).
static const char *hint_verdict(void *self, int tgt)
{
    const int cur = sense_gb_gear(self);
    float rpm;
    const char *by = hint_refusal(cur, tgt, &rpm);
    static double said;
    if (by) logv(1, "hint %d->%d refused by %s: %d would run %.0f rpm at %.0f km/h (the line is %.0f rpm)", cur, tgt, by, tgt, rpm, fabsf(T.speed) * 3.6f,
                 tgt < cur ? rev_line(T.motor_brake || T.retarder > 0 || S.eb_level > 0.01f) : stall_line());
    else if (sim_s() - said > 0.5 || sim_s() < said) { said = sim_s(); logv(2, "hint target %d seen in %d (rpm %.0f): honored", tgt, cur, rpm); }
    return by;
}

// The hook bodies (core/shifthook.cpp calls them once the game's own call has run, and only while this brain has the gearbox)
static void bs_window(void *self)
{
    refresh(self);
    note_gear_change(self);
    // At rest in a gear with the target on it nothing of ours is on its way, so the gear we last handed back IS the gear we are in. (A shift nobody here asked for -- the driver's press, the
    // game's own automatic below the hand-over speed -- left the old one standing, and a press back to that gear was read as our own command.)
    if (sense_gb_state(self) == 0 && sense_gb_gear(self) > 0 && sense_gb_target(self) == sense_gb_gear(self)) S.commanded = sense_gb_gear(self);
    // a press a protection refuses is taken back here, in its own tick, before the game compares the rpm with its window and starts the shift
    if (const int pt = hint_pending(self)) if (hint_verdict(self, pt)) act_gb_set_target(self, sense_gb_gear(self));
    int why;
    int want = decide_gear(self, &why);
    S.why = why_text[why];
    S.want = want;
    S.target = sense_gb_target(self);
    // Neutral goes in through the tail hook (the chooser path clamps to +-1).
    if (want > 0 && want != ROLL && want != sense_gb_gear(self) && takeover()) act_gb_force(self);
    launch_correct(self);
    view_note(self, want, why);
}

static int bs_choose(void *self, int stock)
{
    int ret = stock;
    int why = W_GAME;
    // A shift hint of the driver in M (the game's handler moves the target one gear, before this chooser runs). Out of its rpm window
    // (the rev limiter, 24-10 ride 124-130 s and 133-138 s) the chooser runs every tick and the call site writes its result into the
    // target: our "stay in this gear" erased the press until the pedal came off. So a target nobody here asked for is honored.
    // the physics' start gear the game did not pick (launch_correct): keep handing it back until it is in
    if (S.launch_want > 0 && (fabsf(T.speed) * 3.6f > 6 || sim_s() - S.launch_at > 2.5 || sense_gb_gear(self) == S.launch_want)) S.launch_want = 0;
    const int hint_tgt = hint_pending(self);
    if (takeover()) {
        int w;
        refresh(self);
        int want = decide_gear(self, &w);
        if (want == ROLL) { ret = sense_gb_gear(self); why = w; }
        else if (want > 0) { ret = want; why = w; }
    }
    if (S.launch_want > 0 && sense_gb_gear(self) != S.launch_want) { ret = S.launch_want; why = W_START_CORRECTED; }
    if (hint_tgt && !hint_verdict(self, hint_tgt)) { ret = hint_tgt; why = W_HINT; }   // (the driver's press is taken at once, in every mode, and never reverted by us; a refused one is back at the gear in bs_window)
    // Every gear we hand back counts as ours, so that only the hint buttons
    // show up as shifts nobody here asked for.
    if (ret != sense_gb_gear(self)) {
        if (ret > 0 && sense_gb_gear(self) > 0 && (S.dec_for != ret || S.dec_from != sense_gb_gear(self))) {   // a new decision: what the shift will cost is measured from here
            S.dec_at = sim_s(); S.dec_v = fabsf(T.speed); S.dec_fres = resist(fabsf(T.speed)); S.dec_for = ret; S.dec_from = sense_gb_gear(self);
            S.lb_dec = S.lb_on && (why == W_LEVER_UP || why == W_ENGINE_BRAKE_LEVEL);
        }
        S.commanded = ret; S.cmd_why = why;
    }
    return ret;
}

// The automatic's stop threshold: below it the clutch opens / neutral. Stock
// 0.8 x idle (+ a brake term the game adds), which pushes until ~2 km/h.
// While braking to a stop without throttle (below 15 km/h): idle + 150 rpm;
// the game's own brake term comes on top, so harder braking opens earlier.
// Not at speed: ride ts3a went to neutral at 74 km/h in 12 when braking hard.
static float stop_threshold(float idle, float thr, float brake)
{
    // [lowspeed] owner=1: the open point is the owner's (the start gear's idle point, lowspeed.cpp); no fixed km/h edge, one definition of the stop
    if (low_owner_on() && !S.unsupported && !manual_box()) return low_stop_rpm(idle, thr, brake, T.gear);
    return sc_stop_threshold(p_stop, p_no_creep, g_lv.crawl_toggle, S.unsupported || manual_box(), idle, thr, brake);
}
// Runs at the very end of every automatic update (also in neutral), right
// before the generic gearbox acts on the target gear: eco-roll goes in and
// out here (lka-roll-re notes part B).
static void tail_pre(void *self)
{
    if (!g_active || !takeover() || self != S.self) return;
    int g = sense_gb_gear(self), st = sense_gb_state(self);
    // After a roll: our exit gear as the target; the generic gearbox then
    // parks in its gap state (clutch open, gear 0) until there is throttle
    // (ts3a-c, neutral-exit-re notes § 1). There we do its own engage step.
    if (S.roll_exit > 0) {
        if (g != 0 || sim_s() - S.roll_exit_at > 3 || fabsf(T.speed) * 3.6f < K_MOVING_KMH) {
            if (g == 0) {
                logline("roll end: gear %d did not engage in 3 s", S.roll_exit);
                incident("neutral", "after the eco-roll gear %d did not engage in 3 s (%.0f km/h)", S.roll_exit, fabsf(T.speed) * 3.6f);
            }
            S.roll_exit = 0;
            g_rolling = 0;
        } else {
            if (sense_gb_target(self) != S.roll_exit) act_gb_set_target(self, S.roll_exit);
            if (st == 1 && act_gb_engage(self, S.roll_exit)) logline("roll end: engaged gear %d at %.0f km/h", S.roll_exit, fabsf(T.speed) * 3.6f);
            return;
        }
    }
    // Parked in the gap state while moving, with a gear as target (ts4q 604-616 s: a pause in the middle of an upshift
    // left gear 0 and the clutch open for 11 s at 38 -> 25 km/h: our cruise throttle waits for the shift to end, the
    // gearbox waits for throttle). After 1.5 s we do its engage step, as at the end of a roll.
    {
        int tgt = sense_gb_target(self);
        if (st == 1 && g == 0 && tgt > 0 && !S.rolling && !g_lv.crawl_toggle && fabsf(T.speed) * 3.6f > K_MOVING_KMH) {
            if (!S.gap_since) S.gap_since = sim_s();
            // right after a pause the gap state is the pause's, not a shift's (pre.6 ride 609/1716 s: 1.5 s in neutral at 45-50 km/h)
            else if (sim_s() - S.gap_since > (sim_s() - g_reset_at < 5 ? 0.3 : 1.5) && act_gb_engage(self, tgt)) {
                logline("stuck in neutral for %.1f s at %.0f km/h: engaged gear %d", sim_s() - S.gap_since, fabsf(T.speed) * 3.6f, tgt);
                if (sim_s() - g_reset_at >= 5)  // right after a pause the gap state is normal: no incident
                    incident("neutral", "stuck in neutral for %.1f s at %.0f km/h, engaged gear %d", sim_s() - S.gap_since, fabsf(T.speed) * 3.6f, tgt);
                S.gap_since = 0;
            }
        } else S.gap_since = 0;
    }
    if (st != 0) return;
    if (S.rolling && g > 0) { act_gb_set_target(self, 0); S.commanded = 0; S.cmd_why = W_ECO_ROLL; return; }
    if (!S.rolling || g != 0) return;
    refresh(self);
    // A +/- press in neutral would end a roll the box started without cruise (decision 67), but it cannot be told apart yet: the game's target is not quiet during a roll (the rides of
    // 02-11 read 6 .. 12 in it in every rolling row of trueshift.csv: the game's own automatic writes the gear for the speed, this hook puts 0 back each tick), so "a target nothing of
    // ours asked for" fired on every roll. Until a ride shows what a press does to it, the target is only logged when it changes.
    { static int seen = -99; const int tg = sense_gb_target(self); if (tg != seen) { seen = tg; logv(2, "eco-roll: the target in neutral reads %d at %.0f km/h", tg, fabsf(T.speed) * 3.6f); } }
    float v = fabsf(T.speed), brake = fmaxf(T.in_brake, T.eff_brake);
    int ebrake = T.motor_brake || T.retarder > 0 || S.eb_level > 0.01f;
    if (roll_wanted(v, resist(v), ebrake, brake, sim_s() - S.pedal_on_at)) { act_gb_set_target(self, 0); return; }
    S.rolling = 0;
    S.roll_at = sim_s();
    int pick = roll_exit_pick(v, ebrake, brake);
    act_gb_set_target(self, pick);
    S.roll_exit = pick;
    S.roll_exit_at = sim_s();
    S.commanded = pick;
    S.cmd_why = W_ROLL_END;
}

// The start gear by physics (see stub_start): lock speed and slip time at this mass and grade, never by a gear count. `stock` = the game's
// pick; the result may be above it (launch_max_kmh) or below it.
static int launch_pick(int stock, float thr, bool force)
{
    sc_launch c = { takeover(), p_start_fix, p_launch_max, p_slip_s, p_launch_torque, p_launch_rpm, p_kick_pedal, S.mode, g_lv.lcx != 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    // The game's clutch of 1.62 (vehicle_slip_rpm): the start gear is picked for the foot of the launch, and the pick is made before the pedal (the game asks at the brake release, the pedal
    // 0.00): the driver's usual launch at this load (core/driver.h driver_launch_share, 0.3 until his launches say otherwise) stands for it until the pedal is down and harder than that.
    if (!force && !g_lv.lcx && vehicle_slip_rpm() > 0.0f && V.eng_ok) {
        const float usual = wish_unmap(driver_launch_share(V.mass_ok ? V.mass : 0.0f), S.mode);
        c.slip_rpm = vehicle_slip_rpm(); c.foot = fmaxf(clampf(thr, 0, 1), usual); c.share = wish_map(c.foot, S.mode);
        c.gap_s = fmaxf(sl_dur_est(), V.eng.shift_time); c.decel = sl_dec_est();
    }
    return sc_launch_pick(c, stock, thr, force);
}
static int bs_start(void *self, int stock)
{
    if (stock <= 0) return stock;
    vehicle_mass_stale();  // fresh mass for the start
    refresh(self);
    // [lowspeed] owner=1: the gear the truck starts in is the owner's (the physics pick, preselected at the stop); this function only carries it out
    if (low_owner_on()) {
        int og = low_start_gear();
        if (og > 0 && og <= n_gears()) {
            logv(1, "start gear %d (game %d, the low-speed owner's), %.1f t, grade %.1f%%, pedal %.2f", og, stock, V.mass / 1000, grade_pct(), clampf(T.in_thr, 0, 1));
            S.commanded = og;
            S.cmd_why = og == stock ? W_START : W_START_LOWERED;
            return og;
        }
    }
    // Engaging while moving (after an eco-roll or neutral): never the start
    // gear, a gear that fits the speed.
    float v = fabsf(T.speed);
    if (takeover() && V.eng_ok && v * 3.6f > K_MOVING_KMH && wheel_radius() > 0.1f) {
        S.rolling = 0;
        S.roll_at = sim_s();
        // the demand filter lags a pedal that goes down hard right at the engage (25-10, 2397 s: full pedal at a junction, demand 0.02, gear 10
        // at 877 rpm on a 2.3% climb, then 10->8 two seconds later): the pedal itself counts
        // Engaging while rolling is a gear for the speed, not a start gear: launch_pick assumes a standstill and would
        // pick a gear far too tall for the speed (ride 30-10: "engage at 5 km/h: gear 5", then a stall at 574 rpm). Only a
        // standstill gets the physics start gear.
        int g = pick_for_speed(v, lerpf(cruise_floor(n_gears()) + 100, pw_mid(), smooth(p_power_from, p_power_full, fmaxf(S.demand, clampf(T.in_thr, 0, 1)))));
        // ... and then the layers' pick, as for the roll's other end (roll_exit_pick): the pedal that ended the roll is only just on (any throttle in neutral engages), so the gear is the one that holds the
        // push of the road now, not the one for the light pedal that the lugging rule takes down again one second later
        g = layers_pick_from(g, v, T.motor_brake || T.retarder > 0 || S.eb_level > 0.01f, fmaxf(T.in_brake, T.eff_brake));
        logv(1, "engage at %.0f km/h: gear %d (game %d)", v * 3.6f, g, stock);
        S.commanded = g;
        S.cmd_why = W_ENGAGE;
        return g;
    }
    float thr = clampf(T.in_thr, 0, 1);
    int pick = launch_pick(stock, thr);
    static double logged_at;
    if (sim_s() - logged_at > 2) {
        logged_at = sim_s();
        logv(1, "start gear %d (game %d), %.1f t, grade %.1f%%, pedal %.2f", pick, stock, V.mass / 1000, grade_pct(),
                thr);
    }
    S.commanded = pick;
    S.cmd_why = pick == stock ? W_START : W_START_LOWERED;
    return pick;
}

// For the low-speed owner (suite.h): the start gear the physics would pick, also where TrueShift does not have the truck (below its
// takeover speed, which is where a stop and a launch happen). The game's own pick is its default: 4 alone, 1 with a trailer.
static int bs_start_gear(float thr)
{
    int n = n_gears();
    if (n < 1 || !V.eng_ok || !(V.mass > 0) || !(wheel_radius() > 0.1f)) return 0;
    return launch_pick(T.trailer ? 1 : (n < 4 ? n : 4), clampf(thr, 0, 1), true);
}
static float bs_idle_rpm() { return idle_rpm(); }
float ts_up_pt() { return S.up_pt; }
// The floor (rpm) the braking rule of the last look judged the gear against (the service-brake-only floor, the engine brake's keep point ...), 0 when it did not look a moment ago.
// For the sims (G1: a downshift on the service brake lands on the tallest gear that holds it); the same state ts_brake_down_eta reads.
float ts_brake_floor() { return S.bd_at > 0 && sim_s() - S.bd_at < 0.3 && sim_s() >= S.bd_at ? S.bd_floor : 0.0f; }

// Seconds until TrueShift's next downshift in braking, at this deceleration: the speed falls to the point where the rule that
// looked last (the engine brake's keep point, the braking rule's floor, the deceleration assist's window) judges the gear, less
// the speed the shift itself loses; 0 = due now (or waiting only for the dwell), NAN = no such rule is looking.
float ts_brake_down_eta(float decel)
{
    float v = fabsf(T.speed);
    int cur = T.gear;
    if (!(decel > 0.05f) || !(v > 0.5f) || cur < 2 || !V.eng_ok) return NAN;
    float per = rpm_at(cur, 1.0f), eta = NAN;   // rpm per m/s
    if (!(per > 0)) return NAN;
    double now = sim_s();
    if (S.bd_gear == cur && now - S.bd_at < 0.3 && now >= S.bd_at) {
        float t = (v - (S.bd_floor / per + decel * S.bd_lead)) / decel;
        eta = fmaxf(fmaxf(t, 0.0f), (float)(S.bd_after - now));
    }
    if (S.ad_gear == cur && now - S.ad_at < 0.3 && now >= S.ad_at && S.ad_cap > 0) {
        for (int l = cur - 1; l >= 1 && l >= cur - (p_eb_skip > 0 ? 2 : 1); l--) {
            float pl = rpm_at(l, 1.0f);
            if (!(pl > 0)) continue;
            float t = (v - (S.ad_cap / pl + decel * S.ad_lead)) / decel;
            t = fmaxf(fmaxf(t, 0.0f), (float)(S.ad_after - now));
            if (!(eta <= t)) eta = t;
        }
    }
    return eta;
}

// The same for a deceleration the truck is about to have but does not have yet (the lever's retarder coming back after a shift's close, through the lower gear): the braking rule only looks when the truck
// slows over 0.8 m/s2, so right after a close it has not looked and the answer above is NAN. Here the rule's floor is evaluated for the gear in, as it would be at that deceleration with no brake pedal.
float ts_brake_down_eta_hard(float decel)
{
    const float e = ts_brake_down_eta(decel);
    if (e == e) return e;
    const float v = fabsf(T.speed);
    const int cur = T.gear;
    if (!(decel > 0.8f) || !(v > 0.5f) || cur < 2 || !V.eng_ok) return NAN;
    const float per = rpm_at(cur, 1.0f);   // rpm per m/s
    if (!(per > 0)) return NAN;
    const float floor = fmaxf(fmaxf(idle_rpm() + ioff(250), eco_dn(cur) - 100), 0.8f * idle_rpm() + ioff(250));
    const float t = (v - (floor / per + decel * (V.eng.shift_time + 0.25f))) / decel;
    return fmaxf(fmaxf(t, 0.0f), (float)(S.shift_at + 0.5 - sim_s()));
}
// For the wish pedal (suite.h): the mode from the game's cvar, as the module reads it, so it is also right where our hooks are not in.
static int bs_mode()
{
    if (S.unsupported || manual_box()) return -1;
    int m = sense_mode();
    return m == MODE_M ? -1 : m;
}
static int bs_view(ts_view_t *o)
{
    if (!g_active || S.unsupported || manual_box() || T.paused || !S.self || !(wall_s() - S.view_at < 0.5)) return 0;
    int cur = S.view_gear, tgt = S.target;
    o->shifting = cur >= 0 && ((tgt > 0 && tgt != cur) || S.view_state != 0) && !S.rolling && fabsf(T.speed) * 3.6f > 3;   // (at a standstill the gap state waits for the pedal: no shift)
    if (tgt > 0 && tgt != cur) o->next = tgt;
    else if (S.mode != MODE_M && S.want > 0 && S.want != ROLL && S.want != cur && takeover()) o->next = S.want;   // forced this update
    o->advice = S.mode == MODE_M && S.advice != cur ? S.advice : 0;
    o->up_rpm = S.up_pt > 0 ? S.up_pt : 0;
    o->dn_rpm = S.dn_pt > 0 ? S.dn_pt : 0;
    o->demand = S.demand;
    if (S.tk_why > W_NONE && S.tk_why < W_COUNT) {
        o->why = why_text[S.tk_why];
        o->why_from = S.tk_from;
        o->why_to = S.tk_to;
        o->why_age = (float)fmax(0.0, sim_s() - S.tk_at);
    }
    return 1;
}
static void bs_commanded(int gear) { S.commanded = gear; S.cmd_why = W_START_CORRECTED; }
// The longitudinal planner's neutral (see "neutral on request"): called every frame while it drives; want 1 = coast in neutral now,
// for about hold_s more; want 0 = stay in gear. A request lives NQ_TTL without a refresh.
void ts_neutral_request(int want, float hold_s)
{
    // The planner plans over the route (2 km of it, with the lead, the bends and the limits): it is the route layer's, so with [trueshift] lookahead at 0 the box takes no request of it (the box's own
    // eco-roll, which reads the grade of now, stays). Nothing is kept of a request that was refused, so the box is not in the planner's state when the key goes on.
    if (p_lookahead <= 0) { s_nq.at = -1e9; s_nq.want = 0; s_nq.hold_s = 0; s_nq.roll = 0; s_nq.no = want ? "route layer off (lookahead 0)" : NULL; return; }
    s_nq.at = sim_s();
    s_nq.want = want ? 1 : 0;
    s_nq.hold_s = hold_s == hold_s ? hold_s : 0.0f;
}
int ts_neutral_state() { return S.rolling ? 1 : 0; }
const char *ts_neutral_refusal()
{
    if (p_lookahead <= 0) return s_nq.no;   // (a request refused at the door: ts_neutral_request keeps only the reason)
    if (!nq_fresh() || !s_nq.want) return NULL;
    if (!g_active || !takeover()) return "TrueShift off";
    return s_nq.no;
}

// ---------------------------------------------------------------- module

static char g_status[200] = "off";
static const char *ts_status() { return g_status; }

static void ts_start()
{
    if (p_enabled <= 0) { logline("TrueShift disabled in the ini"); snprintf(g_status, sizeof g_status, "off (ini)"); return; }
    S.pk = -99;
    int done, want;
    g_active = shift_hooks_install(&done, &want);   // the game's hook sites are shared with the car automatic (core/shifthook.cpp)
    if (g_active && (p_stop > 0 || low_owner_wanted())) {
        if (shift_hook_stop()) logline("smooth stop hook in place");
        else { p_stop = 0; logline("smooth stop off: threshold code not found"); }
    }
    // The update's tail hook carries eco-roll AND the neutral watchdog (a shift interrupted by a pause leaves gear 0 with
    // the clutch open): both need the engage code, and the watchdog is wanted with eco_roll = 0 too.
    if (g_active) {
        int tail = shift_hook_tail();
        s_tail_ok = tail;
        if (!tail) {
            logline("%s: %s", p_roll > 0 ? "eco-roll and neutral watchdog off" : "neutral watchdog off",
                    shift_tail_engage_ok() ? "update tail not found" : "gearbox engage code not found");
            p_roll = 0; p_roll_cruise = 0;   // neutral needs this hook: no roll of any kind without it
        } else logline("%s hook in place", p_roll > 0 ? "eco-roll and neutral watchdog" : "neutral watchdog (eco-roll off)");
    }
    logline("TrueShift %s: %d/%d call sites hooked%s", g_active ? "active" : "NOT active", done, want,
            p_takeover > 0 ? "" : " (observe only)");
    snprintf(g_status, sizeof g_status, "%s, %d/%d hooks%s; smooth stop %s, eco-roll %s, lookahead %s, dashboard letter %s",
             g_active ? (p_takeover > 0 ? "on" : "on (observe only)") : "NOT active", done, want,
             "", p_stop > 0 ? "on" : "off", p_roll > 0 ? "on" : "off", p_lookahead > 0 ? "on" : "off", p_letter > 0 ? "on" : "off");
    if (p_trace > 0) {
        g_trace = open_out("trueshift.csv", "w");
        if (g_trace)
            fputs("t,kmh,rpm,gear,target,want,why,pedal,thr_eff,brake,mbrake,retarder,cruise_kmh,grade_pct,mass_t,"
                  "demand,up_pt,dn_pt,hold,kick,accel,rpm_k,clutch,mode,thr,f_need_kn,floor,ours,eb_level,f_bias_kn,strain,learn_rpm,roll,route,crest_m,junction_m,idle,f_scale\n", g_trace);
    }
}

static char cur_letter() { return mode_dash(S.mode); }
static void ts_frame()
{
    if (g_active && T.gear < 1) S.out_of_gear = 1;   // (neutral at a stop, or the gap of a shift: note_gear_change reads it when the box comes in with a gear nobody here chose)
    g_rolling = S.rolling || S.roll_exit > 0;  // neutral by our choice, also until the exit gear is in
    if (!S.rolling) s_nq.roll = 0;               // (the requested roll is over, or never started)
    g_roll_boost_kmh = S.boost_want && !S.rolling ? fminf(p_roll_boost * band_scale(T.cruise), overspeed_room(T.cruise)) : 0;
    {
        static const int to_inc[] = { 2, 1, 3, 4 };  // Normal, Eco, Power, M -> incident file numbering
        int on = g_active && !S.unsupported && !manual_box();
        g_inc.mode = on ? to_inc[S.mode] : 0;
        g_inc.target = on && S.commanded > 0 ? S.commanded : 0;
        g_inc.grade_pct = on ? grade_pct() : NAN;
        g_inc.flags = (g_inc.flags & ~(INC_ROLL | INC_HOLD)) | (g_rolling ? INC_ROLL : 0) | (S.hold_until > sim_s() ? INC_HOLD : 0);
        if (on) {
            sc_watch_engine(g_active && !S.unsupported && takeover() && S.mode != MODE_M, mode_label(S.mode), idle_rpm());
            sc_overspeed_tone(g_active && takeover() && S.mode == MODE_M);
        }
    }
    DEV_NOTE(gearbox(g_active && !S.unsupported && !manual_box() ? mode_dash(S.mode) : 0, V.mass, grade_pct()));
    {
        static double at;
        if (wall_s() - at > 0.5 || wall_s() < at) { at = wall_s(); g_pcc_need_kmh = g_active && !S.unsupported ? climb_need_kmh() : NAN; }
    }
    sc_dash_letter(p_letter > 0, g_active && !S.unsupported && !manual_box(), S.self, &S.letter_state, mode_update, cur_letter);
    sc_idle_learn(&S.idle, p_idle, 0);
    if (!g_trace || !S.self || T.paused) return;
    double now = wall_s();
    if (now - g_trace_at < 0.1) return;
    g_trace_at = now;
    double sim = sim_s();
    fprintf(g_trace, "%.2f,%.1f,%.0f,%d,%d,%d,%s,%.2f,%.2f,%.2f,%d,%u,%.0f,%.2f,%.2f,%.2f,%.0f,%.0f,%d,%d,%.2f,%.3f,%.2f,"
            "%c,%.2f,%.1f,%.0f,%d,%.2f,%.2f,%.2f,%.0f,%d,%d,%.0f,%.0f,%.0f,%.3f\n",
            now, fabsf(T.speed) * 3.6f, T.rpm, T.gear, S.target, S.want, S.why ? S.why : "", T.in_thr,
            T.eff_thr, T.in_brake, T.motor_brake, T.retarder, T.cruise * 3.6f, grade_pct(), V.mass / 1000, S.demand,
            S.up_pt, S.dn_pt, S.hold_until > sim, S.kick_until > sim, T.accel, V.rpm_k, T.eff_clutch,
            mode_dash(S.mode), S.thr, S.f_need / 1000, S.floor_rpm, S.ours, S.eb_level, V.f_bias / 1000, S.strain, learned().rpm, S.rolling, route_on(),
            route_on() ? route_crest_m(2000) : -1.0f, route_on() ? route_junction_m() : -1.0f, idle_rpm(), V.f_scale);   // (the columns show what the box sees of the route: nothing with the key at 0)
    static double flushed;
    if (now - flushed > 2) { flushed = now; fflush(g_trace); csv_cap(&g_trace, "trueshift.csv"); }
}

static void ts_stop()
{
    if (g_trace) fflush(g_trace);
}

// Truck configuration (SDK) arrived: not a gearbox we manage -> left to the game, quietly and with one log line.
static void ts_config(const scs_telemetry_configuration_t *c)
{
    if (!c || strcmp(c->id, "truck")) return;
    if (T.fwd_gears >= 1 && T.fwd_gears <= 2) unsupported("1-2 forward gears (electric or single-speed drive)");
    if (manual_box() && !S.manual_logged) {
        S.manual_logged = 1;
        logline("TrueShift: gearbox type '%s' is not an automatic, staying idle", T.shifter);
    }
    if (!manual_box()) S.manual_logged = 0;
}

// A live setting changed (config_live_set). eco_roll lives in the update's tail hook: without it no roll, whatever the switch says.
static const char *ts_applied(const char *key, float old)
{
    (void)old;
    if (!strcmp(key, "eco_roll") && p_roll > 0 && !s_tail_ok) { p_roll = 0; return "not available (hook missing)"; }
    return NULL;
}

// This brain drives the trucks; the car automatic is modules/carshift.cpp. core/shifthook.cpp hands the game's hooks to the brain of the class.
static int bs_is_on() { return g_active; }
static const shift_brain k_brain = { "TrueShift", VCM_TRUCK, bs_is_on, bs_window, bs_choose, bs_start, tail_pre, stop_threshold,
                                     bs_start_gear, bs_idle_rpm, bs_mode, bs_commanded, bs_view };
static struct brain_reg { brain_reg() { shift_register(&k_brain); } } k_brain_reg;

module m_trueshift = { "trueshift", "TrueShift", g_params, sizeof g_params / sizeof g_params[0],
                       ts_start, ts_frame, ts_config, ts_stop, ts_reset, ts_status, ts_applied, VCM_TRUCK };
