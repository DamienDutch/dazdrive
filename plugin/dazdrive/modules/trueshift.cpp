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
#include "../core/suite.h"
#include "../core/game.h"
#include "../core/route.h"
#include "../core/vehicle.h"
#include "../core/gearscore.h"
#include "../core/rpmspan.h"
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------- settings

float g_rpm_by_engine;   // [trueshift] rpm_by_engine (core/rpmspan.h; LowSpeed reads it too)

static float p_eco_ceil, p_mode_char, p_enabled, p_takeover, p_on_kmh, p_off_kmh, p_on_rpm, p_hint_hold, p_min_gap, p_dwell_ud, p_dwell_du,
    p_max_skip, p_skip_acc, p_skip_wait, p_skip_land_frac, p_ch_land, p_ch_over, p_hyst, p_kick_pedal, p_kick_rate, p_power_from, p_power_full,
    p_cruise_rpm, p_reserve, p_learn_rpm, p_learn_res, p_learn_km, p_eco_rpm, p_power_rpm, p_eco_reserve, p_power_reserve, p_cc_damp,
    p_eb_down, p_eb_light, p_eb_skip, p_m_style, p_m_lug, p_letter, p_start_fix, p_launch_max, p_slip_s, p_launch_torque, p_launch_rpm,
    p_idle, p_trace, p_lookahead, p_roll, p_roll_cruise, p_roll_cruise_min, p_roll_dec, p_roll_dec_eco, p_roll_min_eco, p_roll_under, p_roll_under_eco, p_roll_under_pw, p_roll_boost, p_calm, p_skip_over, p_kick_dwell, p_load_from, p_launch_fix, p_ret_down, p_air_psi, p_air_rpm, p_no_creep, p_stop, p_tones, p_crest_band, p_relax,
    p_choice, p_ch_eco, p_ch_norm, p_ch_power, p_ch_rpm, p_ch_shift, p_ch_stay, p_ch_stay_pw, p_ch_floor,
    p_ch_obj, p_ch_amin, p_ch_ts_eco, p_ch_ts_norm, p_ch_ts_pw, p_ch_upwait, p_ch_flip, p_ch_pw_eco, p_ch_pw_norm, p_ch_pw_pw, p_ch_pw_var, p_ch_fl_idle, p_pw_cross,
    p_ch_own_brake, p_ch_route_cap, p_lift_up, p_lift_delay,
    p_kw, p_kr_eco, p_kr_norm, p_kr_pw, p_kl_eco, p_kl_norm, p_kl_pw;

static param g_params[] = {
    { "enabled", &p_enabled, 1, "1 = TrueShift on, 0 = the game's own automatic" },
    { "takeover", &p_takeover, 1, "1 = TrueShift picks the gears, 0 = only log what it would do" },
    { "takeover_kmh", &p_on_kmh, 7, "TrueShift takes over from the game's automatic above this speed" },
    { "takeover_rpm", &p_on_rpm, 1000, "TrueShift takes over as soon as the engine has reached this rpm with the clutch closed in any gear (the clutch is locked then, so the rpm is real); 0 = by speed only (takeover_kmh)" },
    { "handback_kmh", &p_off_kmh, 4, "and hands back below this speed (starting, stopping, maneuvering)" },
    { "eco_rev_ceiling_rpm", &p_eco_ceil, 200, "Eco: an upshift never waits for revs above the top of the efficient range plus this (rpm), also with the pedal down; a kickdown lifts it. 0 = off. Saves fuel on long pulls" },
    { "mode_character", &p_mode_char, 1, "1 = Eco and Power differ from Normal in the small drivetrain details too, with their own fixed values: kickdown only at a floored pedal in Eco (0.97) and easier in Power (0.85) instead of kickdown_pedal; Power holds a gear 4 s after a downshift and 8 s after a kickdown, Eco 3 s, instead of dwell_down_up_s and kick_dwell_s; a roll must last Eco 12 / Normal 15 / Power 20 s; hint hold, engine brake and pedal filter per mode. 0 = only the main differences, the ini values for all" },
    { "hint_hold_s", &p_hint_hold, 10, "seconds a gear chosen with the shift-hint buttons is held (0 = off)" },
    { "min_gap_s", &p_min_gap, 0.5f, "minimum time between two shifts" },
    { "dwell_up_down_s", &p_dwell_ud, 3.0f, "after an upshift, no downshift for this long (unless kickdown or lugging)" },
    { "dwell_down_up_s", &p_dwell_du, 2.0f, "after a downshift, no upshift for this long (unless clearly past the upshift point)" },
    { "max_skip", &p_max_skip, 2, "most gears an upshift may skip (0 = never skip)" },
    { "skip_accel", &p_skip_acc, 0.15f, "acceleration (m/s2) the skipped-to gear must still give" },
    { "skip_land_frac", &p_skip_land_frac, 0.0f, "a HARD extra rule on top of the scores (off by default; the scores' landing cost does this job): a skip upshift (more than one gear up) is only taken when the gear it lands in sits at least this share of the way up from the choice floor to the upshift point (0.3 = the skip lands in the top 70 % of the window; 0 = off). Never at a pedal of 0.9 or more: a hard launch still skips" },
    { "choice_land_weight", &p_ch_land, 6.0f, "choice 1: weight of the landing cost: under load a gear that would sit just above the choice floor costs this much (times the share of the power asked), so the gear one up that lands higher wins. 0 = off" },
    { "choice_over_weight", &p_ch_over, 1.0f, "choice 1: weight of the cost of revs over the mode's upshift point (it grows with the square of the distance; one quarter of the way from the point to the top of the window costs this much). 0 = off" },
    { "choice", &p_choice, 0, "0 = the gear is chosen by the shift points and rules (upshift and downshift points, reserve, skip checks). 1 = the gear is chosen by scores: of the gears the hard rules allow, the one with the lowest cost of throttle, revs and shift time wins (the hard rules still run first: rev limit, stall, braking, engine brake, kickdown, hints, dwell times). Aims at fewer shifts and more drive time" },
    { "choice_comfort_eco", &p_ch_eco, 0.75f, "choice 1, Eco: the share of its full-throttle force a gear may need for the acceleration asked before it starts to cost a lot (higher = skips as far as the engine can pull)" },
    { "choice_comfort", &p_ch_norm, 0.45f, "choice 1, Normal: the same, a little more careful" },
    { "choice_comfort_power", &p_ch_power, 0.35f, "choice 1, Power: the same, the most careful (skips the least)" },
    { "choice_rpm_weight", &p_ch_rpm, 0.5f, "choice 1: weight of the cost of the revs (far above the torque plateau, and just above idle under load). A guess, there is no efficiency map: 0 = revs do not count" },
    { "choice_shift_weight", &p_ch_shift, 8.0f, "choice 1: weight of the cost of a shift (the shift time of the gearbox times the power asked: the drive time it loses). Higher = fewer, longer shifts, more skipping, mostly with a slow gearbox" },
    { "choice_stay", &p_ch_stay, 0.03f, "choice 1, Eco and Normal: bonus for the gear the truck is in, against hunting between two gears that score alike" },
    { "choice_stay_power", &p_ch_stay_pw, 0.2f, "choice 1, Power: the same, higher" },
    { "choice_floor_rpm", &p_ch_floor, 60, "choice 1: under load the revs stay at least this far over the start of the torque plateau (rpm). Coasting and the engine brake are not held to it. 0 = the window reaches below the plateau. Only where choice_floor_power of the mode is 0" },
    { "choice_objective", &p_ch_obj, 1, "choice 1: 1 = when the scores say up, the gear to shift up to is the one that gains speed soonest (the shift time without pull counted against the pull each gear gives), so a skip can win under load; an upshift waits a little after the last one, and a shift back the other way soon after a shift needs a hard reason; braking downshifts look at the speed the shift ends at. 0 = the gear with the best score, as before" },
    { "choice_min_accel", &p_ch_amin, 0.10f, "choice_objective 1: a gear that cannot give all the acceleration asked is still a choice if at full throttle it gives at least this (m/s2)" },
    { "choice_time_shift_eco", &p_ch_ts_eco, 3.0f, "choice_objective 1, Eco: a second in a shift counts as this many seconds of driving when the gear up is chosen (higher = fewer shifts, more skips)" },
    { "choice_time_shift", &p_ch_ts_norm, 1.3f, "choice_objective 1, Normal: the same" },
    { "choice_time_shift_power", &p_ch_ts_pw, 1.0f, "choice_objective 1, Power: the same (1 = only the time counts)" },
    { "choice_up_wait", &p_ch_upwait, 2.0f, "choice_objective 1: after an upshift the next one waits this many shift times (never less than 1.5), unless the revs near the top of the window. 0 = off" },
    { "choice_flip_s", &p_ch_flip, 4.0f, "choice_objective 1: within this many seconds after a shift, a shift back the other way only for kickdown, the rev limit, a stall, braking or revs at the edge of the window (0 = the dwell times alone)" },
    { "choice_floor_power_eco", &p_ch_pw_eco, 0.64f, "choice 1, Eco: under load the revs stay at or above the lowest rpm where the engine still gives this share of its peak power (torque times rpm, read from its torque curve), also right after an upshift: the power in hand for the next push. Replaces choice_floor_rpm. 0 = choice_floor_rpm" },
    { "choice_floor_power", &p_ch_pw_norm, 0.69f, "choice 1, Normal: the same" },
    { "choice_floor_power_power", &p_ch_pw_pw, 0.74f, "choice 1, Power: the same" },
    { "power_shift_crossover", &p_pw_cross, 1, "choice 1: 1 = at a firm pedal in Power (0.8 or more) and at a full pedal in Normal (0.95 or more) the upshift point is the power crossover of the engine and the gearbox: the rpm where the power in this gear equals the power in the next gear at the rpm it lands at, from the engine's torque curve and the ratios, never below the top of the power range and never above 95 % of the rev limit. Eco never. 0 = the top of the power range, as before" },
    { "rpm_by_engine", &g_rpm_by_engine, 0, "1 = the fixed rpm margins of the shift rules (the stall margin, the lowest landing rpm, the comfort floors, the stop and hand-off margins, the engine-brake light stage, the air-charge and M-lug revs) follow the engine's own span from idle to the torque plateau and its brake range, instead of numbers fitted on engines with idle 550-600 and a plateau at 900-1000 rpm. On those engines nothing changes. 0 = the fixed numbers" },
    { "choice_floor_idle_frac", &p_ch_fl_idle, 0.5f, "choice 1: the floor of choice_floor_power* is at least this share of the way from idle up to the start of the torque plateau, so an engine with a high plateau keeps a floor of its own span and not one near idle. 0 = the power share alone" },
    { "choice_floor_power_varied", &p_ch_pw_var, 0.025f, "choice 1: added to the share above while the driver's pedal has been varying over the last 10 s (standard deviation of the pressed pedal over 0.05; lifts do not count): more power in hand for the next push. 0 = off" },
    { "choice_own_brake", &p_ch_own_brake, 1, "choice 1: 1 = the pause of upshifts after a brake (6 s below 60 km/h) also follows the brake requests of the suite itself (deceleration assist, ACC), not only the brake the game reports. 0 = only the game's brake" },
    { "choice_route_cap", &p_ch_route_cap, 1, "choice 1 with the route look-ahead: 1 = the speeds the scores look ahead to stop at what the route allows ahead (bends, junctions), so no upshift is chosen for a speed the truck will not reach before it slows. 0 = off" },
    { "lift_upshift", &p_lift_up, 1, "choice 1: 1 = with the pedal off for lift_up_delay_s, a higher gear the scores prefer is taken at once instead of waiting for the pedal, if this driver's lifts usually last longer than the shift (the median of the lifts of this session, at least 5, over the shift time + 0.5 s). Never on a descent (under -1 %), while braking, with a bend or junction within about 6 s, or into a gear under the floor of choice_floor_power. 0 = off" },
    { "lift_up_delay_s", &p_lift_delay, 0.6f, "lift_upshift: seconds the pedal must be off first" },
    { "launch_correct", &p_launch_fix, 1, "1 = when the game picked a lower start gear than the physics of launch_max_kmh / launch_slip_s allow (after reversing, after the engine started, or its own downshift at a stop), the gear is changed at the standstill before the pull-away: no climbing through the low gears (0 = off)" },
    { "load_down_from", &p_load_from, 0.75f, "on a climb: a gear that needs this share of its full-throttle force to hold the speed (and the speed falls under real throttle) gets its downshift point raised towards the torque plateau (1 = off; a gear that cannot hold it at all downshifts at once anyway)" },
    { "skip_wait_over_rpm", &p_skip_over, 400, "skip_wait_s: how far over the upshift point the revs may build up while waiting for a skip (rpm; never above 85 % of the limit)" },
    { "kick_dwell_s", &p_kick_dwell, 5, "after a kickdown or lugging downshift no upshift for this long, unless clearly past the upshift point (the rpm path, not the rev limit)" },
    { "kickdown_by_wish", &p_kw, 1, "choice 1 with the wish pedal ([pedal] wish) and choice_objective 1: the kickdown is the driver's dissatisfaction, not a flat pedal. The ratio of the force the pedal wishes to the force this gear gives at full throttle now is held (the gear stays as long as it is 1 or less: the wish pedal just opens the throttle further); over kick_ratio the excess adds up, and past kick_level the gear that can give the wish and scores best is taken, whatever the dwell times say. A fast press halves what is left; right after an upshift only a large shortage counts; a flat pedal with the ratio over kick_ratio kicks down at once. 0 = the kickdown rule of kickdown_pedal and kickdown_rate, as without the wish pedal" },
    { "kick_ratio_eco", &p_kr_eco, 1.05f, "kickdown_by_wish, Eco: the wished force over the force this gear gives at full throttle, above which the dissatisfaction adds up (1 = the gear just gives the wish; the margin is for the error of the vehicle model)" },
    { "kick_ratio", &p_kr_norm, 1.05f, "kickdown_by_wish, Normal: the same" },
    { "kick_ratio_power", &p_kr_pw, 1.05f, "kickdown_by_wish, Power: the same" },
    { "kick_level_eco", &p_kl_eco, 0.15f, "kickdown_by_wish, Eco: the dissatisfaction (excess over kick_ratio, summed over seconds) at which the kickdown comes. The default is 3 s at a 10 % shortage and 0.4 s at 40 %; the pedal pressed fast halves what is left" },
    { "kick_level", &p_kl_norm, 0.05f, "kickdown_by_wish, Normal: the same (default: 1 s at a 10 % shortage, 0.14 s at 40 %)" },
    { "kick_level_power", &p_kl_pw, 0.015f, "kickdown_by_wish, Power: the same (default: 0.3 s at a 10 % shortage, 0.04 s at 40 %)" },
    { "skip_wait_s", &p_skip_wait, 6, "pulling lightly: wait up to this long for the revs to build up so a skip can land (0 = shift one gear at once)" },
    { "skip_relax_rpm", &p_relax, 80, "pulling lightly at low speed: a skip may land up to this much lower (rpm), never under the downshift point plus hysteresis_rpm. Full below pedal 0.40 or under 20 km/h at any pedal, fading out by pedal 0.65 and from 45 to 75 km/h (0 = off)" },
    { "hysteresis_rpm", &p_hyst, 50, "an upshift must land at least this far above the next downshift point" },
    { "kickdown_pedal", &p_kick_pedal, 0.92f, "pedal position for kickdown" },
    { "kickdown_rate", &p_kick_rate, 1.5f, "pedal speed (per second) that makes a full press a kickdown" },
    { "power_from", &p_power_from, 0.65f, "pedal (0-1) where the shift points start moving to the power range" },
    { "power_full", &p_power_full, 1.0f, "pedal where the shift points reach the power range" },
    { "cruise_rpm", &p_cruise_rpm, 0, "Normal: an early upshift lands at least this far above the start of the torque plateau" },
    { "reserve", &p_reserve, 1.3f, "Normal: an early upshift needs this much more force in the new gear than now (1.3 = ~75% pedal)" },
    { "learn_rpm", &p_learn_rpm, 100, "after an upshift without pull: lowest landing rpm +this on that kind of grade (fades with distance)" },
    { "learn_reserve", &p_learn_res, 0.1f, "and the force reserve +this" },
    { "learn_km", &p_learn_km, 3, "half of what was learned is forgotten after this many km (faster when upshifts hold)" },
    { "eco_rpm", &p_eco_rpm, -75, "Eco: shift points and cruise rpm moved by this much" },
    { "power_rpm", &p_power_rpm, 150, "Power: shift points and cruise rpm moved by this much" },
    { "eco_reserve", &p_eco_reserve, 1.2f, "Eco: force reserve for an early upshift" },
    { "power_reserve", &p_power_reserve, 1.5f, "Power: force reserve for an early upshift" },
    { "cruise_damp_s", &p_cc_damp, 4, "cruise control throttle is smoothed over this many seconds before it counts as demand" },
    { "ebrake_downshift", &p_eb_down, -1, "engine brake downshifts: -1 = full when the engine's def asks for it (engine_brake_downshift), else by level; 0 = always by level; 1 = always full" },
    { "ebrake_skip", &p_eb_skip, 1, "1 = engine brake / retarder downshifts go two gears at a time when that still fits the brake range (half the shifts, half the game's throttle blips)" },
    { "ebrake_light_rpm", &p_eb_light, 1100, "rpm the lightest engine brake/retarder stage keeps (the full stage keeps the engine's brake range)" },
    { "m_style", &p_m_style, 0, "M mode: 0 = European (like Volvo, Mercedes): every shift is yours, also at the rev limiter; a warning tone when the engine goes over its limit (the wheels push it, e.g. downhill). 1 = protected (like Eaton): also one gear up at the rev limiter and one down before the engine stalls. 2 = assisted (like a VW DSG): also kickdown, downshifts when braking to a stop and one gear down when lugging" },
    { "m_lug_rpm", &p_m_lug, 1000, "M mode with m_style 2: one gear down below this rpm with a lot of pedal and no pull" },
    { "dash_letter", &p_letter, 1, "1 = the dashboard shows the mode (D, E, S, M) instead of A" },
    { "start_gear_fix", &p_start_fix, 1, "1 = lower the game's start gear when the launch would be too slow" },
    { "launch_max_kmh", &p_launch_max, 14, "a start gear above the game's pick is allowed when the clutch would be closed by this speed (km/h at launch_rpm) and the slip time stays within launch_slip_s: on a light or downhill start the truck no longer revs a low gear to the limiter. Not in Power or M. 0 = never above the game's pick (the game picks by a number in the truck definition: 4 solo, 1 with a trailer, whatever the gear ratios are). With this on, a full pedal does not demand a quicker clutch than a normal one" },
    { "launch_slip_s", &p_slip_s, 1.0f, "longest clutch-slip time accepted for a start gear" },
    { "launch_torque", &p_launch_torque, 0.6f, "share of max torque the clutch passes while starting" },
    { "launch_rpm", &p_launch_rpm, 1000, "rpm at which the clutch closes when starting" },
    // the vehicle model's resistance (core/vehicle.cpp); the keys stay here so every ini keeps working
    { "rolling_resistance", &V.crr, 0.006f, "rolling resistance coefficient" },
    { "drag_area", &V.cda, 6.0f, "air drag Cd*A in m2 for trucks (vehicles under 4.5 t use 1.0: cars, pickups)" },
    { "efficiency", &V.eta, 0.9f, "driveline efficiency" },
    { "idle_rpm", &p_idle, 550, "engine idle rpm to start with (learned while standing still)" },
    { "lookahead", &p_lookahead, 1, "1 = use the navigation route ahead (climbs, tops, junctions); without a route TrueShift stays careful" },
    { "crest_band_kmh", &p_crest_band, 5, "with the route (predictive cruise, PCC): no downshift for a top ahead that this gear gets over at full throttle while losing at most this much below the set speed (without cruise: below the speed when the top came into view) and staying on the torque plateau. Full at 40 t, 60 % solo, 60 % in Power; 0 = off" },
    { "eco_roll_under_kmh", &p_roll_under, 1.5f, "Normal mode, with cruise control: the roll ends when the speed is about to drop this far below the set speed (km/h; Volvo I-Roll: 1-3) (Written for a set speed of 70 km/h; it scales with the set speed.)" },
    { "eco_roll_under_eco_kmh", &p_roll_under_eco, 2.0f, "the same in Eco: a little more room, the most relaxed (Written for a set speed of 70 km/h; it scales with the set speed.)" },
    { "eco_roll_under_power_kmh", &p_roll_under_pw, 1.0f, "the same in Power: the strictest (keep it between 1 and 2 km/h; values around 0.5 give very short rolls that hunt) (Written for a set speed of 70 km/h; it scales with the set speed.)" },
    { "eco_roll_boost_kmh", &p_roll_boost, 2.0f, "with cruise control, Eco and Normal: just before an eco-roll starts, the cruise control lets the speed come this far over the set speed (km/h), so the roll takes the speed down to a little under it and the average stays near the set speed (0 = roll from the set speed) (Written for a set speed of 70 km/h; it scales with the set speed.)" },
    { "low_calm_kmh", &p_calm, 12, "below this speed (km/h) with a gentle pedal the gear is only changed down for a stall or lugging: no hunting through the gears while maneuvering (0 = off)" },
    { "eco_roll_max_decel", &p_roll_dec, 0.04f, "eco-roll only where the truck would lose at most this much speed in neutral (m/s2; 0.04 = ~3 km/h in 20 s: a slight descent, not the flat)" },
    { "eco_roll_max_decel_eco", &p_roll_dec_eco, 0.08f, "the same in Eco: a slightly steeper stretch may still be rolled (0.08 = ~6 km/h in 20 s)" },
    { "eco_roll_min_kmh_eco", &p_roll_min_eco, 40, "Eco: the lowest speed that starts an eco-roll (km/h; Normal and Power: 50)" },
    { "eco_roll", &p_roll, 1, "1 = roll in neutral with the pedal released on gentle stretches (Eco, Normal and Power; not in M; never with any brake)" },
    { "eco_roll_cruise", &p_roll_cruise, 1, "1 = roll in neutral also while the cruise control is on, even with eco_roll 0. Needs SteadyCruise, predictive cruise (PCC), lookahead and a route; 0 = only with the cruise off" },
    { "eco_roll_cruise_min_s", &p_roll_cruise_min, 15, "with cruise: when the cruise first has to build the roll boost speed, a roll must be expected to last this long (s); without the boost 10 s" },
    { "tones", &p_tones, 1, "1 = a short tone when the mode changes (Eco low, Normal middle, Power rising, M two short)" },
    { "retarder_downshift", &p_ret_down, 0, "1 = the retarder also makes TrueShift keep the rpm up like the engine brake (a retarder does not need it); 0 = only the engine brake does" },
    { "air_charge_psi", &p_air_psi, 100, "brake air below this (psi): keep the rpm up so the engine-driven compressor refills it ([core] air_from_truck: 35 psi over the truck's own warning pressure instead); 0 = off" },
    { "air_charge_rpm", &p_air_rpm, 1200, "the rpm kept while charging (no upshift below it)" },
    { "no_creep", &p_no_creep, 1, "1 = with crawl switched off the truck never creeps on the clutch at idle: pedals free below 8 km/h opens the clutch" },
    { "smooth_stop", &p_stop, 1, "1 = when braking to a stop the clutch opens around idle +150 rpm (more when braking harder) instead of pushing to ~2 km/h" },
    { "trace", &p_trace, 1, "1 = write trueshift.csv (10 per second) for analysis" },
};

// ---------------------------------------------------------------- state

static const char *const mode_name[] = { "Normal", "Eco", "Power", "M" };
static const char mode_letter[] = { 'D', 'E', 'S', 'M' };  // S = Sport (Power); P felt like Park (Damien 25-10)

static window_fn o_window;
static gear_fn o_choose, o_start;
static int g_active;

// Upshift hunting guard, learned per grade class: after an upshift without
// pull the lowest landing rpm and the force reserve go up; both fade with the
// distance driven, faster while upshifts hold. Classes: < -1 %, -1..1, 1..2.5,
// 2.5..4, >= 4 %. A lesson on a grade also counts for the steeper classes.
enum { N_CLASS = 5 };
struct lesson { float rpm, res; };
static lesson g_learn[N_CLASS];
static float g_learn_mass;

// Why a gear was chosen (decide_gear and the stubs): one value the logic compares (it used to strcmp the text), the text only for
// the log, the trace and the dev display. One list keeps the two together.
#define WHY_LIST(X) \
    X(W_NONE, "") \
    X(W_GAME, "game") \
    X(W_HINT, "hint") \
    X(W_START, "start") \
    X(W_START_LOWERED, "start, lowered") \
    X(W_START_CORRECTED, "start, corrected") \
    X(W_ENGAGE, "engage") \
    X(W_ECO_ROLL, "eco-roll") \
    X(W_ROLL_END, "roll end") \
    X(W_NO_DATA, "no data") \
    X(W_NEUTRAL, "neutral") \
    X(W_LOW_SPEED, "low speed") \
    X(W_WRONG_GEAR, "wrong gear") \
    X(W_REV_LIMIT, "rev limit") \
    X(W_REV_LIMIT_SKIP, "rev limit skip") \
    X(W_STALL, "stall") \
    X(W_KICKDOWN, "kickdown") \
    X(W_HINT_HOLD, "hint hold") \
    X(W_ENGINE_BRAKE, "engine brake") \
    X(W_ENGINE_BRAKE_LEVEL, "engine brake (level)") \
    X(W_ENGINE_BRAKE_HOLD, "engine brake hold") \
    X(W_BRAKING, "braking") \
    X(W_BRAKING_JUNCTION, "braking (junction ahead)") \
    X(W_BRAKING_HOLD, "braking hold") \
    X(W_MIN_GAP, "min gap") \
    X(W_NO_PULL, "no pull") \
    X(W_CLIMB_AHEAD, "climb ahead") \
    X(W_LOW_SPEED_HOLD, "low-speed hold") \
    X(W_ANTI_HUNT_DOWN, "anti-hunt (down)") \
    X(W_TOP_AHEAD, "top ahead") \
    X(W_TOP_REACHABLE, "top reachable") \
    X(W_OVER_TOP, "over the top") \
    X(W_CRUISE_BAND, "cruise band") \
    X(W_LUGGING, "lugging") \
    X(W_DOWN_SKIP, "down skip") \
    X(W_DOWN, "down") \
    X(W_AIR_CHARGE, "air charge") \
    X(W_JUNCTION_AHEAD, "junction ahead") \
    X(W_SKIP_WAIT, "skip wait") \
    X(W_UP_EARLY_SKIP, "up early skip") \
    X(W_UP_EARLY, "up early") \
    X(W_POWER_HOLDS, "power holds") \
    X(W_UP_VETO, "up veto (slowing)") \
    X(W_ANTI_HUNT_UP, "anti-hunt (up)") \
    X(W_UP_SKIP, "up skip") \
    X(W_UP, "up") \
    X(W_UP_BLOCKED, "up blocked") \
    X(W_HOLD, "hold") \
    X(W_M, "M") \
    X(W_M_REV_LIMIT, "M: rev limit") \
    X(W_M_STALL, "M: stall") \
    X(W_M_KICKDOWN, "M: kickdown") \
    X(W_M_STOPPING, "M: stopping") \
    X(W_M_LUGGING, "M: lugging") \
    X(W_UP_WAIT, "up wait") \
    X(W_LIFT_UP, "up on lift") \
    X(W_WISH_UP, "wish unmet above") \
    X(W_ASSIST_DOWN, "braking assist") \
    /* end */
enum why_e {
#define X(id, text) id,
    WHY_LIST(X)
#undef X
    W_COUNT
};
static const char *const why_text[W_COUNT] = {
#define X(id, text) text,
    WHY_LIST(X)
#undef X
};

static struct {
    void *self;
    int eng_seen;           // the engine line is logged for the engine the vehicle model (V.eng) has now ...
    float eng_torque, eng_limit;   // ... which had these values
    float thr;              // throttle that counts: pedal, or damped cruise control
    float cc_thr;           // cruise control throttle, smoothed
    float demand;           // smoothed demand 0..1
    double demand_at;
    float f_need;           // tractive force asked for (N), up fast, down slow
    float pedal_prev;
    double pedal_at;
    double kick_until;
    float wk_d;             // kickdown_by_wish: the dissatisfaction (excess of the wish over the gear, summed over time)
    float wk_r;             // and the ratio of now (wished force over the force this gear gives at full throttle); wk_on: the wish pedal gives the throttle this frame
    int wk_on, wk_fast;     // wk_fast: a fast press has already halved the level
    double wk_until;        // the kickdown is due (set by the trigger, ends with the shift, 3 s, or the pedal lifting)
    double wk_retry;        // no new trigger before this time (no gear could give the wish)
    double wk_pt; float wk_pp;   // the pedal 0.05 s or more ago, for its speed
    int mode;
    int ours;               // above the takeover speed (with hysteresis)
    double ours_at;
    int ours_launch;        // taken over while pulling away (below 25 km/h), not after a pause or a teleport at speed
    int last_gear;
    double shift_at;
    int shift_dir;
    int commanded;          // last gear we returned to the game
    int cmd_why;            // why_e: why that gear (the logic compares it)
    const char *last_why;   // reason of the last shift we made
    double hold_until;
    int up_gear;            // last upshift we made, to judge whether it held
    int up_tainted;         // that upshift landed in braking or a lift-off: whatever goes wrong with it is no lesson
    double up_at;
    float up_grade, up_kmh;
    float up_pred;          // throttle the new gear was expected to need
    float strain;           // throttle above that prediction while not gaining speed, integrated
    float grade_hist[8];    // grade (%) once a second, for "a climb starts"
    float crest_ref;        // speed when the top ahead came into view (0 = no top in view)
    int crest_said;         // logged "top reachable" for this top
    double crest_seen_at;   // last time a top was in view
    double top_at;          // last time a top ahead was reachable in this gear (the last ~50 m to it are not seen as a climb)
    float top_floor;        // and the lowest speed (m/s) that was allowed for it
    double grade_at;
    const char *why;        // reason of the last decision, the text, for the trace
    int want, target;
    double blocked_log_at;
    double brake_at;        // the last time the brake was on (service brake over 0.05)
    double own_brake_at;    // the last time the suite itself held a brake request (act_request; choice_own_brake)
    double kd_at;           // the last kickdown or lugging downshift of ours
    double skip_wait_at, skip_wait_last;    // when the early upshift started waiting for a skip (0 = not waiting), and the last frame it waited
    float up_pt, dn_pt, floor_rpm, eb_level;
    int eb_full;            // the game itself would shift into the engine-brake range now
    int letter_state;       // -1 unknown, 0 not found, 1 written
    float idle;             // learned idle rpm (0 = not yet)
    int rolling;            // eco-roll: in neutral by our choice
    double roll_at;
    int launch_want;        // a start gear the game did not pick (launch_correct); the chooser hands it back until it is in or the truck rolls
    double launch_at;
    int boost_want;         // eco-roll would start but wants the cruise control to build the roll_boost speed first
    float roll_kmh;         // speed when the roll started
    int roll_exit;          // gear to engage after a roll (kept as target until it is in)
    double roll_exit_at;
    double pedal_on_at;     // last time the pedal was pressed
    double skip_log_at;     // last 'single up, no skip' log line
    double score_log_at;    // last 'score' log line (choice = 1)
    double veto_log_at;     // last 'score veto' log line (choice = 1)
    double hold_log_at;     // last 'score hold' log line (choice = 1)
    double calm_since;      // Power: since when the driving is flat, light and steady
    float calm_k;           // Power: 0 = full Power shift points, 1 = like Normal (after a calm stretch)
    const char *unsupported; // why this truck is left to the game's own automatic (NULL = fine)
    float rpm_odd_s;        // seconds the rpm did not follow the speed model at all (torque converter?)
    int manual_logged;
    float pk;               // decide_gear: the steepest grade (%) of the last seconds, fading 0.15 %/s (the climb the upshift reckons with)
    double pk_at;           // when it was last updated
    double gap_since;       // tail_pre: parked in the gearbox's gap state (gear 0, a target, clutch open) while moving since; 0 = not
    int seed_demand, seed_force;   // after a hard reset: demand / f_need start from the pedal of now, not from 0 (one update each)
    float pv[10][3];        // the driver's pressed pedal per second of the last 10 (sum, sum of squares, samples): is it varying (choice_floor_power_varied)
    long pv_sec;            // the second pv was last filled for
    int varied;             // 1 = the pedal has been varying (standard deviation over PV_STD)
    double lift_at;         // the driver's pedal went off (under 0.05) from a pull above 15 km/h; 0 = not lifted
    float lift_prev;        // the pedal of the last update (for that)
    double lift_used;       // lift_at of the lift that already had its upshift (one per lift)
    // the braking downshift rules as decide_gear last looked at them (ts_brake_down_eta reads them): the rule's rpm threshold, the
    // seconds of speed the shift loses before the rule judges (rule 1: the engine brake keep point, 2: braking, 3: the deceleration assist's cap)
    double bd_at, bd_after, ad_at, ad_after;   // when the rule looked, and not before
    int bd_gear, ad_gear;
    float bd_floor, bd_lead, ad_cap, ad_lead;
} S;

// The driver's lifts (pedal off from a pull until the pedal again, a brake or a standstill), for lift_upshift: the last LIFT_N of the
// session, whatever truck. Measured on one driver: median 2.8 s, 31 % shorter than 1 s.
enum { LIFT_N = 64, LIFT_MIN = 5 };
static float g_lift[LIFT_N];
static int g_lift_n;
static const float CC_FALL_S = 0.5f;   // choice 1, our own cruise throttle: the filters follow it this fast (s) when it falls
static const float PV_STD = 0.05f;   // standard deviation of the pedal over 10 s above which it counts as varying (a steady foot ~0.01, a foot wavering 0.30-0.54: 0.085)

// The median of the lifts measured so far (s), < 0 = fewer than LIFT_MIN
static float lift_median()
{
    int n = g_lift_n < LIFT_N ? g_lift_n : LIFT_N;
    if (n < LIFT_MIN) return -1;
    float a[LIFT_N];
    memcpy(a, g_lift, n * sizeof a[0]);
    for (int i = 1; i < n; i++) for (int j = i; j > 0 && a[j] < a[j - 1]; j--) { float t = a[j]; a[j] = a[j - 1]; a[j - 1] = t; }
    return n & 1 ? a[n / 2] : 0.5f * (a[n / 2 - 1] + a[n / 2]);
}

enum { ROLL = 100 };        // decide_gear: "put it in neutral" (eco-roll)

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

static float lerpf(float a, float b, float k) { return a + (b - a) * k; }
static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static float smooth(float e0, float e1, float x)
{
    if (e1 <= e0) return x >= e1 ? 1.0f : 0.0f;
    float t = clampf((x - e0) / (e1 - e0), 0, 1);
    return t * t * (3 - 2 * t);
}
static float grade_pct() { return V.grade_pct; }
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

// The vehicle model's forces (core/vehicle.cpp), the short names the decisions below are written in: engine rpm in gear g
// at speed v (m/s), tractive force (N) in g at v with a share of the torque, the resistance (N) on the grade of now or
// on another grade (%).
static float rpm_at(int g, float v) { return vehicle_rpm_at(g, v); }
static float force_at(int g, float v, float share) { return vehicle_force_at(g, v, share); }
static float resist(float v) { return vehicle_resist(v); }
static float resist_grade(float v, float pct) { return vehicle_resist_grade(v, pct); }

// A throttle cap or scale of ours (the own speed limiter, the rollover guard, launch control: core/actuate.cpp) holds the engine's
// throttle below what the pedal asks. Then the pedal is not a request for pull: what counts is the throttle the engine actually gets.
static int thr_capped()
{
    const act_wish *w = act_wishes();
    for (int o = 0; o < ACT_N; o++)
        if (w[o].cap < 1.0f || w[o].scale < 1.0f) return 1;
    return 0;
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
static float pw_k() { return smooth(p_power_from, p_power_full, S.demand); }
static float pw_mid() { return 0.5f * (V.eng.range[R_POWER][0] + V.eng.range[R_POWER][1]); }
static float up_pt(int g)
{
    float mo = mode_rpm();
    float u = fminf(lerpf(eco_up(g) + mo, V.eng.range[R_POWER][1] + 0.5f * mo, pw_k()), V.eng.rpm_limit * 0.94f);
    // Eco: a soft ceiling (fuel); not during a kickdown
    if (p_mode_char > 0 && p_eco_ceil > 0 && S.mode == MODE_ECO && S.kick_until <= sim_s())
        u = fminf(u, fmaxf(eco_up(g) + mo, V.eng.range[R_HIGH][1] + p_eco_ceil));
    return u;
}
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
static float dn_pt(int g) { return lerpf(base_dn(g), V.eng.range[R_POWER][0] + 0.5f * mode_rpm(), dn_k()); }
// Pull (m/s2) below which a demand downshift makes sense; a light truck with
// plenty of pull left keeps its gear (and may skip on the way up).
static float a_power() { return S.mode == MODE_ECO ? 0.2f : S.mode == MODE_POWER ? 0.45f : 0.3f; }
static lesson learned();

// Lowest landing rpm of an early upshift.
// A skip (2 steps or more) may land up to 100 rpm under the plateau (Damien 12-10: fewer shifts when pulling lightly); the
// downshift point plus hysteresis stays the lower bound, so it does not hunt back.
static float cruise_floor(int g, float under = 0)
{
    return fmaxf(V.plateau + p_cruise_rpm + mode_rpm() - under, base_dn(g) + p_hyst + 50) + learned().rpm;
}

static float skip_accel();
// Lowest landing rpm of an upshift on the rpm path: above the downshift point that applies plus the hysteresis, and at least idle + 350.
// A skip (2 gears or more) pulling lightly at low speed may go up to skip_relax_rpm under the idle + 350 part (the engine has
// plenty of torque there), never under the downshift point plus the hysteresis, so it cannot shift straight back down.
// Gradual, not a switch: full below pedal (demand) 0.40, or under 20 km/h at any pedal; gone at 0.65 and from 45 to 75 km/h, and only while the skipped-to gear
// still pulls at least skip_accel at the pedal of now (half of it: no relaxation). Firm pedal and higher speeds keep the strict floor.
static float skip_relax(float v, float a_now)
{
    if (p_relax <= 0) return 0;
    // (25-10, Damien: skips more, also gently under 50 km/h and even at full pedal under 20 km/h; fewer shifts is calmer)
    float dk = fmaxf(1 - smooth(0.40f, 0.65f, S.demand), 1 - smooth(20 / 3.6f, 28 / 3.6f, v));
    return p_relax * dk * (1 - smooth(45 / 3.6f, 75 / 3.6f, v)) * smooth(0.5f * skip_accel(), skip_accel(), a_now);
}
static float land_floor(int h, float a_h, float relax)
{
    return fmaxf((a_h < a_power() ? dn_pt(h) : base_dn(h)) + p_hyst, idle_rpm() + ioff(350) - relax);
}

static int skip_allowed() { return (int)p_max_skip; }
// Pull a skipped-to gear must keep: less with a slow box (every shift is a
// long gap without power). Damien (ts3b): skip also with a fast (DCT) box.
static float skip_accel() { return p_skip_acc * (1.0f - 0.25f * smooth(0.4f, 1.5f, V.eng.shift_time)); }

// Highest gear above cur+1 (up to max_skip) an upshift at high demand may jump
// to: lands on the plateau and still pulls hard and keeps most of the pull of
// cur+1 (a car must not jump 2 -> 5). Used at the rev limit, where a light
// truck arrives in the low gears before the regular upshift (ts3b: 3-4-5).
static int skip_target(int cur, float v, float f_res, float grade)
{
    int best = cur + 1, n = n_gears();
    if (grade > 0.02f || v * 3.6f < p_on_kmh) return best;
    float a_skip = skip_accel() * 2.0f;
    for (int h = cur + 2; h <= n && h <= cur + 1 + skip_allowed(); h++) {
        float a_h = (force_at(h, v, 0.95f) - f_res) / V.mass;  // still accelerating hard: the plateau is enough
        if (rpm_at(h, v) < fmaxf((a_h < a_power() ? dn_pt(h) : base_dn(h)) + p_hyst, idle_rpm() + ioff(350))) break;
        if ((force_at(h, v, 0.85f) - f_res) / V.mass < a_skip) break;
        if (force_at(h, v, 1.0f) - f_res < 0.7f * (force_at(cur + 1, v, 1.0f) - f_res)) break;
        best = h;
    }
    return best;
}

// Gear below cur whose rpm at speed v lands nearest to target, without
// landing above its own upshift point or near the limiter.
static int pick_down(int cur, float v, float target, bool free = false)
{
    int best = cur;
    float best_err = 1e9f;
    for (int l = cur - 1; l >= 1; l--) {
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

static int mode_from_cvar(float a) { return mode_of(a); }

// The mode from the game's adaptive-shift cvar; log and a tone on a change.
// Called from the gearbox hook and from the frame (also while standing).
static void mode_update()
{
    float a = game_adaptive();
    int m = mode_from_cvar(a);
    static int logged = -1;
    if (m != logged) {
        if (logged >= 0 && p_tones > 0) {
            // Eco low bell, Normal middle, Power rising pair, M two equal
            play_cue(m == MODE_ECO ? CUE_MODE_ECO : m == MODE_POWER ? CUE_MODE_POWER : m == MODE_M ? CUE_MODE_M : CUE_MODE_NORMAL);
        }
        logged = m;
        logline("mode %s (g_adaptive_shift = %g)", mode_name[m], a);
    }
    S.mode = m;
}

// Continuity broke (core/telemetry.cpp session_reset): pause menu, hitch, engine start (SOFT: timers, filters, holds);
// trailer (de)coupled (LOAD: mass-dependent values); other truck, save/load, teleport (HARD: everything learned).
static double g_reset_at = -1e9;  // last continuity break (the neutral watchdog acts faster right after one)
static void ts_reset(int level, const char *why)
{
    double none = -1e9;
    g_reset_at = sim_s();
    S.hold_until = S.kick_until = 0;
    S.wk_d = 0; S.wk_on = S.wk_fast = 0; S.wk_until = S.wk_retry = 0; S.wk_pt = 0;
    S.demand_at = S.pedal_at = 0;           // the next dt is "unknown": filters step, no kickdown from a stale pedal
    S.ours_at = S.shift_at = none;          // dwell and take-over timers must not hold a shift back after a pause
    S.top_at = none; S.crest_ref = 0; S.crest_said = 0;  // no "over the top" hold across a teleport
    S.bd_at = S.ad_at = none;               // no braking downshift is expected across a break
    S.blocked_log_at = 0;
    S.gap_since = 0;                        // a pause in the gap state is not a stuck shift (the timer starts again)
    S.pedal_prev = clampf(T.in_thr, 0, 1);
    memset(S.pv, 0, sizeof S.pv); S.varied = 0;   // the pedal's variation starts over (a pause is no steady foot), so does a lift
    S.lift_at = 0;
    if (level >= RESET_LOAD) {
        S.f_need = 0;
        S.up_gear = 0;
        S.strain = 0;
    }
    if (level >= RESET_HARD) {
        S.calm_k = 0; S.calm_since = 0;     // Power starts with its own shift points again
        S.pedal_on_at = none; S.skip_log_at = S.veto_log_at = S.hold_log_at = 0; S.kd_at = none;
        memset(g_learn, 0, sizeof g_learn);
        g_learn_mass = 0;
        S.self = NULL;                      // refresh() starts over from the gearbox it sees next
        S.eng_seen = 0;                     // the engine line is logged again
        S.idle = 0;                         // idle is learned again (engine on 5 s, standing still)
        S.rpm_odd_s = 0;
        S.unsupported = NULL;
        memset(S.grade_hist, 0, sizeof S.grade_hist);
        S.pk = -99; S.pk_at = 0;            // the grade peak belongs to the road we left
        S.letter_state = -1;
        S.cc_thr = 0;
        S.demand = 0;
        S.seed_demand = S.seed_force = 1;   // (sim X4: from 0 a full pedal on a climb looked like a light one and skipped to a gear without pull)
        S.rolling = 0;
        S.roll_exit = 0;
        g_rolling = 0;
    }
}

// kickdown_by_wish (choice 1, choice_objective 1, the wish pedal giving the throttle): the kickdown as the driver's dissatisfaction. Once per frame.
// r = the force the pedal wishes / the force this gear gives at full throttle now. r <= 1: the gear holds (the wish pedal just opens the throttle
// further). D adds up max(0, r - kick_ratio) x dt, falls when r is under kick_ratio, and a kickdown is due at D >= kick_level. A fast press
// (over kickdown_rate, per mode) halves what is left of the level, once per press. Right after an upshift (choice_flip_s) only r >= kick_ratio + 0.3
// may trigger. A flat pedal (kickdown_pedal, per mode) with r over kick_ratio triggers at once: the old rule's last resort. decide_gear takes the gear.
static void wish_kick_update(int g, float v, float pedal, double dt, double now)
{
    if (!(dt > 0)) return;   // (refresh runs more than once per frame: a call without time passed changes nothing)
    S.wk_on = 0;
    double pdt = now - S.wk_pt;
    float rate = 0;
    if (pdt > 0.05 && pdt < 0.5) rate = (float)((pedal - S.wk_pp) / pdt);
    if (pdt > 0.05) { S.wk_pp = pedal; S.wk_pt = now; }
    float fw = p_kw > 0 && p_choice > 0 && p_ch_obj > 0 && S.mode != MODE_M && S.ours && g > 0 && V.eng_ok && !V.electric ? pedal_wish_force(v) : -1.0f;
    // (the own cruise with [cruise] force_mode 1 asks a force too: the same kickdown when its force is what this gear cannot give, cruise on and the pedal off)
    int cc_wish = 0;
    if (!(fw >= 0) && p_kw > 0 && p_choice > 0 && p_ch_obj > 0 && S.mode != MODE_M && S.ours && g > 0 && V.eng_ok && !V.electric) {
        fw = cruise_wish_force(v);
        if (fw >= 0) cc_wish = 1;
    }
    float fg = g > 0 && V.eng_ok ? vehicle_force_at(g, v, 1.0f) : 0.0f;
    if (!(fw >= 0) || !(fg > 1)) { S.wk_d = 0; S.wk_fast = 0; S.wk_until = 0; return; }
    const float thr = by_mode(p_kr_eco, p_kr_norm, p_kr_pw), level = fmaxf(0.0f, by_mode(p_kl_eco, p_kl_norm, p_kl_pw));
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
    bool after_up = S.shift_dir > 0 && now - S.shift_at < p_ch_flip;
    if (S.wk_d >= level && r > thr && now >= S.wk_retry && !(after_up && r < thr + 0.3f) && S.wk_until <= now) S.wk_until = now + 3;
}

static void refresh(void *self)
{
    double now = sim_s();
    // The vehicle model (core/vehicle.cpp) first: inside the game's gearbox update the grade is this frame's and the
    // mass is fresh right after a reset; another gearbox object makes the model start over as well.
    vehicle_update(self);
    if (self != S.self) {
        // another truck (or the first update): start from its current gear
        if (S.self) ts_reset(RESET_HARD, "gearbox object changed");
        S.self = self;
        S.last_gear = S.commanded = gb_gear(self);
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
        // the whole torque curve (rpm: share of the peak), so a ride log shows where the torque falls off
        char cv[24 * 16] = "";
        for (int i = 0, k = 0; i < e->n_curve && i < 24; i++) {
            int w = snprintf(cv + k, sizeof cv - k, "%s%.0f: %.2f", i ? ", " : "", e->curve[i][0], e->curve[i][1]);
            if (w < 0 || w >= (int)sizeof cv - k) break;
            k += w;
        }
        logv(1, "engine curve: %s", cv);
        S.letter_state = -1;
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
    if (!(V.rpm_k > 0.5f)) V.rpm_k = 1;

    float v = fabsf(T.speed), kmh = v * 3.6f;
    int was = S.ours;
    if (T.parking_brake || kmh < p_off_kmh) S.ours = 0;
    // [lowspeed] owner=1: the owner hands the truck over at clutch lock + 0.5 s (whatever the rpm or the speed), not at takeover_rpm / takeover_kmh
    else if (low_handed()) S.ours = 1;
    // By rpm: the engine has reached takeover_rpm with the clutch closed in a gear. Also above 20 km/h whatever the rpm: after a
    // pause, a load or a teleport the truck may already be cruising at 900 rpm in a high gear and never reach the rpm.
    else if (p_on_rpm > 0 ? ((T.rpm >= p_on_rpm && T.gear > 0 && clutch_seen() < 0.1f) || kmh >= fmaxf(p_on_kmh, 20.0f)) : kmh >= p_on_kmh) S.ours = 1;
    if (S.ours && !was) { S.ours_at = now; S.ours_launch = kmh < 25; if (low_handed()) S.shift_at = now; }   // (the owner's hand-off: the dwell timers start at the lock)

    // Learn the vehicle model's rpm correction while the clutch is closed and the speed steady.
    int g = gb_gear(self);
    if (g > 0 && v > 5 && clutch_seen() < 0.02f && T.rpm > 700) {
        float raw = rpm_at(g, v) / V.rpm_k;
        if (raw > 300) {
            float kr = T.rpm / raw;
            float k = clampf(kr, 0.7f, 1.4f);
            V.rpm_k += 0.02f * (k - V.rpm_k);
            // The rpm of a manual-style box follows the speed within a few percent. If it is far off for a long time
            // with the clutch closed, this is not a box we model (torque converter, hybrid, unusual mod): stay off.
            float step = (float)clampf((float)(now - S.demand_at), 0, 0.5f);
            if (kr < 0.7f || kr > 1.4f) S.rpm_odd_s += step;
            else S.rpm_odd_s = fmaxf(0, S.rpm_odd_s - 2 * step);
            if (S.rpm_odd_s > 20) unsupported("engine rpm does not follow the road speed: torque converter or unusual gearbox?");
        }
    }

    double dt = now - S.demand_at;
    if (dt < 0 || dt > 1) dt = 0;  // a pause or hitch is no time step (filters and integrators skip it; ts_reset also zeroes demand_at)
    S.demand_at = now;

    // Throttle that counts: the pedal directly; cruise control (bang-bang in
    // the game: full below the set speed, zero above) only smoothed.
    // The player's pedal comes from the SDK: inner+0x1424 already holds the
    // cruise control's throttle (ride ts2: every cruise burst looked like a kickdown).
    float pedal = clampf(T.in_thr, 0, 1), eff = gb_thr_eff(self);
    int cc = T.cruise > 0.5f;
    static double cc_zero_since;
    if (eff > 0.02f || !cc) cc_zero_since = now;
    // Bursts are averaged over cruise_damp_s; when it stays off the throttle
    // (slowing for traffic, adaptive cruise) the damped value follows in ~1 s.
    float cc_tau = now - cc_zero_since > 1.5 ? 1.0f : p_cc_damp;
    // choice 1 with our own cruise throttle (smooth, no bursts): its throttle is already smooth, so it is followed as it falls (0.5 s) instead of
    // through the three filters below in a row (cruise_damp_s, demand, force), which kept 10 kN 'needed' for 20 s where 3 kN held the speed
    // (ride 7, 396-416 s). Not while the driver's own pedal is on
    const bool cc_own = p_choice > 0 && cc && g_cruise_smooth && pedal < 0.05f;
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

    // Extra resistance the vehicle model misses (tires, wind, a wrong mass), learned into it: force the
    // engine gives minus modeled resistance minus mass x acceleration, while
    // pulling steadily with the clutch closed. Slow (20 s), limited to +-2 % grade.
    if (g > 0 && clutch_seen() < 0.02f && V.eng_ok && v > 6 && eff > 0.1f && T.in_brake < 0.02f && !T.motor_brake &&
        !T.retarder && S.eb_level < 0.01f && now - S.shift_at > 2) {
        float r = force_at(g, v, eff) - (resist(v) - V.f_bias) - V.mass * T.accel;
        float lim = V.mass * 9.81f * 0.02f;
        V.f_bias = clampf(V.f_bias + (float)(dt / (20 + dt)) * (r - V.f_bias), -lim, lim);
    }

    if (T.in_thr > 0.02f) S.pedal_on_at = now;
    if (act_request() > 0) S.own_brake_at = now;
    // choice 1: is the driver's pedal varying (choice_floor_power_varied)? Its standard deviation over the last 10 s, from one sum per second,
    // over the moments the pedal is pressed (a lift is no varying foot: it would raise the floor of the gear the lift may shift up to)
    {
        long sec = (long)floor(now);
        if (sec != S.pv_sec) {
            for (long k = S.pv_sec + 1; k <= sec && k <= S.pv_sec + 10; k++) memset(S.pv[(unsigned long)k % 10], 0, sizeof S.pv[0]);
            if (sec < S.pv_sec) memset(S.pv, 0, sizeof S.pv);   // (the clock went back)
            S.pv_sec = sec;
        }
        float *b = S.pv[(unsigned long)sec % 10];
        if (pedal >= 0.05f && !cc) { b[0] += pedal; b[1] += pedal * pedal; b[2] += 1; }
        float n = 0, sm = 0, sq = 0;
        for (int i = 0; i < 10; i++) { sm += S.pv[i][0]; sq += S.pv[i][1]; n += S.pv[i][2]; }
        float sd = n > 1 ? sqrtf(fmaxf(0.0f, sq / n - (sm / n) * (sm / n))) : 0;
        int varied = sd > PV_STD;
        if (varied != S.varied && p_choice > 0 && p_ch_pw_var > 0) {
            float base = by_mode(p_ch_pw_eco, p_ch_pw_norm, p_ch_pw_pw), share = fminf(1.0f, base + (varied ? p_ch_pw_var : 0.0f));
            if (base > 0 && V.eng_ok)
                logv(2, "choice floor: the pedal %s (sd %.3f over 10 s): %.0f rpm, %.3f of the peak power", varied ? "varies" : "is steady", sd, gs_floor_rpm(share, idle_rpm(), p_ch_fl_idle), share);
        }
        S.varied = varied;
    }
    // lift_upshift: the driver's lifts, from a pull above 15 km/h (pedal under 0.05) until the pedal again, a brake (the game's or our own),
    // a crawl under 5 km/h or the cruise control (not counted); over 30 s counts 30. A lift that ends in a brake counts that short
    {
        int brk = fmaxf(T.in_brake, T.eff_brake) > 0.05f || act_request() > 0;
        if (S.lift_at > 0) {
            if (cc) S.lift_at = 0;
            else if (pedal >= 0.05f || brk || v * 3.6f < 5) {
                g_lift[g_lift_n % LIFT_N] = fminf(30.0f, (float)(now - S.lift_at));
                g_lift_n++;
                S.lift_at = 0;
                if (p_choice > 0 && p_lift_up > 0 && g_lift_n >= LIFT_MIN && g_lift_n % 10 == LIFT_MIN % 10)
                    logv(2, "lifts: median %.1f s over the last %d", lift_median(), g_lift_n < LIFT_N ? g_lift_n : (int)LIFT_N);
            }
        } else if (pedal < 0.05f && S.lift_prev >= 0.05f && v * 3.6f > 15 && !cc && !brk) S.lift_at = now > 0 ? now : 1e-6;
        S.lift_prev = pedal;
    }
    // Power on a flat, light, steady stretch for a while: the shift points sink to Normal's, so a long calm cruise ends in the top gear
    // and Power does not cost fuel for nothing (Damien 25-10: it stayed in the 11th at ~1480 rpm); a firm pedal brings them back at once
    {
        bool calm = S.mode == MODE_POWER && p_mode_char > 0 && S.thr < 0.4f && fabsf(T.accel) < 0.2f && fabsf(grade_pct()) < 1.5f && v > 5.5f;
        if (!calm) S.calm_since = now;
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
    float lv = game_ebrake_level(self);  // also the automatic engine brake, which the SDK flag misses
    // ... but not the engine brake the game adds by itself to the retarder lever: that counts as the retarder (Damien 11-10,
    // first 0.6 ride 1405/1467 s: retarder 1-2 in town, engine brake level 0.4-0.7 with the SDK flag off, "engine brake (level)"
    // downshifts 5-4-3-2, each with the game's throttle blip: "gas and engine brake and retarder at once")
    if (!T.motor_brake && T.retarder > 0 && p_ret_down <= 0) lv = 0;
    if (lv > 0.01f) e = fmaxf(e, lv);
    else if (T.motor_brake && lv < 0) e = fmaxf(e, 0.5f);
    S.eb_level = e;
    S.eb_full = game_ebrake_downshift(self) > 0;
}

// ---------------------------------------------------------------- the decision

// Gear whose rpm at speed v lands nearest to target, staying between its own
// light-load downshift point and the limiter (re-engaging after a roll).
static int pick_for_speed(float v, float target)
{
    int best = 0;
    float best_err = 1e9f;
    for (int g = 1; g <= n_gears(); g++) {
        float r = rpm_at(g, v);
        if (r > V.eng.rpm_limit * 0.9f || r < base_dn(g) + p_hyst) continue;
        float err = fabsf(r - target);
        if (err < best_err) { best_err = err; best = g; }
    }
    // nothing fits: too slow for every gear (crawling at 4 km/h after a pause: 25-10, "engage at 4 km/h: gear 12") -> the lowest, not the top
    if (!best) return rpm_at(1, v) < base_dn(1) + p_hyst ? 1 : n_gears();
    return best;
}

// ---------------------------------------------------------------- eco-roll

// Back into drive after a roll: a gear for what is asked now.
static int roll_exit_gear(float v, int ebrake, float brake)
{
    float target = ebrake ? V.eng.range[R_BRAKE][0] + 100
                   : brake > 0.02f ? eco_up(1) - 200
                   : lerpf(cruise_floor(n_gears()) + 100, pw_mid(), pw_k());
    return pick_for_speed(v, target);
}

// Neutral on gentle stretches with the pedal released (deliberately: 1.5 s),
// Eco, Normal and Power (Power only with a route that confirms the roll), never with the brake, engine brake or retarder. Ends
// with real pedal (> 10 %), any brake, or when the speed leaves its band:
// without cruise the start speed +6/-12 km/h, with cruise the set speed +3/-5.
// The route (if any) vetoes rolling into a steep descent. With cruise only
// when the Cruise module drives the throttle (else the game's cruise would rev
// the engine in neutral); the cruise then gives no throttle while rolling.
static int roll_wanted(float v, float f_res, int ebrake, float brake, double pedal_free_s)
{
    S.boost_want = 0;
    float kmh = v * 3.6f, set = T.cruise * 3.6f;
    int cc = T.cruise > 0.5f;
    // with cruise only under predictive cruise (Damien 25-10): a roll needs the route to know how long it lasts; the plain
    // eco_roll switch is not needed for it (eco_roll_cruise is its own option)
    int cc_ok = cc && p_roll_cruise > 0 && g_cruise_smooth && g_pcc_on && p_lookahead > 0 && route_ok();
    if ((p_roll <= 0 && !cc_ok) || (S.mode != MODE_ECO && S.mode != MODE_NORMAL && S.mode != MODE_POWER) || !S.ours) return 0;
    // a little gas before the roll (Damien 24-10): with cruise the roll may start up to roll_boost over the set speed, and the
    // cruise control builds that speed first; not in Power, not when the set speed is a speed limit being eased down
    float boost = cc && g_cruise_smooth && S.mode != MODE_POWER && !S.rolling ? fminf(p_roll_boost * band_scale(T.cruise), overspeed_room(T.cruise)) : 0;
    // no gas for a roll when a bend, junction or roundabout ahead already asks for less than the boosted speed: it only ends in gas, then
    // brakes (24-10 evening, 575 s and 743 s: gas 0.5 at 83 km/h, a bend plan of 76-80 km/h, braking 5 s later)
    if (boost > 0 && p_lookahead > 0 && route_ok()) {
        float vs = route_curve_speed(2.5f, 0.3f);
        if (vs == vs && vs * 3.6f < set + boost + 6) boost = 0;
    }
    if ((cc && !cc_ok) || brake > 0.02f || ebrake) return 0;
    if (S.rolling ? T.in_thr > 0.1f : (T.in_thr > 0.02f || (!cc && T.eff_thr > 0.02f) || pedal_free_s < 1.5)) return 0;
    // Damien: at most 1-2 km/h under the set speed, then back on the gas; leave a little early (re-engaging
    // takes ~1 s) using the current deceleration
    float kmh_soon = kmh + fminf(0, T.accel) * 3.6f * 1.0f;
    // how far under the set speed it may roll: Eco most, Power least (Damien 11-10: between 1 and 2 km/h)
    float under = (S.mode == MODE_ECO ? p_roll_under_eco : S.mode == MODE_POWER ? p_roll_under_pw : p_roll_under) * band_scale(T.cruise);
    if (cc && (S.rolling ? (kmh > set + 3 || kmh_soon < set - under) : (kmh > set + 2 + boost || kmh < set - 0.5f))) return 0;
    float a_n = -f_res / V.mass;  // acceleration in neutral
    int rk = p_lookahead > 0 && route_ok();
    if (rk) {
        float steep = route_min_grade(0, fmaxf(200, v * 15));
        if (steep == steep && steep < -3.5f) return 0;
        // not into a junction, roundabout or bend we have to slow down for: the box would have to come back from neutral right
        // there (the exit takes ~1.5 s; 24-10, 227 s: rolled up to a junction, braked, "stuck in neutral 1.5 s at 39 km/h")
        float jn = route_junction_m(), vs = jn >= 0 ? route_curve_speed(2.5f, 0.3f) : NAN;
        if (jn >= 0 && vs == vs && vs < v - 2.5f && jn < (S.rolling ? v * 6 + 20 : fmaxf(150.0f, v * 10))) return 0;
    }
    if (!S.rolling) {
        bool eco = S.mode == MODE_ECO;  // Eco rolls more often: from a lower speed, on a slightly steeper stretch, with a shorter pause between rolls
        if (kmh < (eco ? p_roll_min_eco : 50) || sim_s() - S.roll_at < (eco ? 5 : 10) || sim_s() - S.shift_at < 1) return 0;
        // Only when the speed would stay in the band for ~20 s (route: the
        // grade of the next 20 s; else the live grade).
        float a_pred = a_n;
        if (rk) {
            float g = route_grade(0, v * 20);
            if (g == g) a_pred = -(resist_grade(v, g)) / V.mass;
        }
        // at most ~3 km/h lost in 20 s (Damien 11-10: it rolled at 90 on the flat down to 85, pointless;
        // -0.15 allowed a flat road at speed): in practice a slight descent
        // with cruise: only a roll that lasts >= 10 s before it reaches the band's bottom (DAF/Scania; Power's 0.5 km/h
        // band made 0.5 s rolls that hunted, sim R5b)
        // (the worse of now and the route's 20 s mean: sim R5b started at -0.03 predicted but -0.14 now and rolled 1.5 s)
        float a_chk = fminf(a_pred, a_n);
        // 10 s; 15 s (eco_roll_cruise_min_s) when the cruise first has to spend gas on the boost speed
        // a roll that is too short is no gain (Damien 25-10: 4 of the 7 Normal rolls were under 10 s): Eco 12 s, Normal 15, Power 20; with the boost gas 3 s more
        float need0 = by_mode(12.0f, 15.0f, 20.0f);
        float need_s = boost > 0 && kmh < set + boost - 0.5f ? fmaxf(p_roll_cruise_min, need0 + 3) : need0;
        if (cc && a_chk < 0 && (fmaxf(kmh, set + boost) - (set - under)) / 3.6f / -a_chk < need_s) return 0;
        if (!cc && a_chk < 0 && 12.0f / 3.6f / -a_chk < need0) return 0;  // without cruise the roll ends 12 km/h under where it began
        // Power: only with a route that confirms the roll (Damien 25-10: Power is for pulling, not for rolling)
        if (S.mode == MODE_POWER && p_mode_char > 0 && !rk) return 0;
        if (a_pred > 0.075f || a_pred < -(eco ? p_roll_dec_eco * (rk && p_mode_char > 0 ? 1.4f : 1.0f) : p_roll_dec)) {
            // over a top: a short climb left and a gentle way down after it
            float crest = rk ? route_crest_m(v * 5) : -1;
            if (!(crest >= 0 && a_n > -0.5f && route_min_grade(crest, crest + v * 10) >= -3.5f)) return 0;
            // with cruise it has to reach the top before it drops out of the band (a narrow band ended rolls after 1.5 s)
            if (cc && a_n < 0 && (kmh - (set - under)) / 3.6f / -a_n < crest / fmaxf(v, 1.0f) + 1) return 0;
        }
        // everything fits but the speed: build the boost first, the roll starts when it is there
        if (boost > 0 && kmh < set + boost - 0.5f) { S.boost_want = 1; return 0; }
        return 1;
    }
    return kmh <= S.roll_kmh + 6 && kmh >= S.roll_kmh - 12 && kmh >= (S.mode == MODE_ECO ? p_roll_min_eco - 5 : 45) && grade_pct() > -4;
}

// M: the gear stays where the driver's shift hints put it. m_style (Damien 17-10, after the manuals of Volvo I-Shift,
// Mercedes PowerShift and Eaton UltraShift): 0 European = no shift of our own (overspeed_tone warns instead),
// 1 protected = rev limiter and stall only (Eaton), 2 assisted = also kickdown, stopping and lugging (like a VW DSG).
// Below the low-speed handover the game picks the gear in every style (the start gear after a stop).
static int decide_manual(int cur, float rpm, float v, double since, int *why)
{
    int n = n_gears(), ebrake = T.motor_brake || T.retarder > 0 || S.eb_level > 0.01f;
    double now = sim_s();
    float brake = fmaxf(T.in_brake, T.eff_brake);
    *why = W_M;
    if (p_m_style < 1) return cur;
    if (rpm > V.eng.rpm_limit * (ebrake ? 0.99f : 0.96f) && cur < n) { *why = W_M_REV_LIMIT; return cur + 1; }
    if (rpm < idle_rpm() + ioff(120) && cur > 1) { *why = W_M_STALL; return pick_down(cur, v, eco_dn(1) + 300); }
    if (p_m_style < 2) return cur;
    if (S.kick_until > now && rpm < V.eng.range[R_POWER][0] + 100 && cur > 1 && since > 0.3) {
        int l = pick_down(cur, v, pw_mid() + 100);
        if (l < cur) { *why = W_M_KICKDOWN; return l; }
    }
    if (brake > 0.05f && rpm < idle_rpm() + ioff(250) && cur > 1 && since > 0.5) {
        int l = pick_down(cur, v, eco_up(1) - 200);
        if (l < cur) { *why = W_M_STOPPING; return l; }
    }
    float fmax = force_at(cur, v, 1.0f);
    if (S.thr > 0.8f && rpm < rs_abs(idle_rpm(), p_m_lug) && cur > 1 && since > 2 &&
        (T.accel < 0.05f || fmax - resist(v) < V.mass * 0.03f)) {
        *why = W_M_LUGGING;
        return cur - 1;
    }
    *why = W_M;
    return cur;
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
    float c = p_crest_band > 0 && p_lookahead > 0 && route_ok() && V.mass > 500 ? route_crest_m(fmaxf(400, v * 25)) : -1;
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
    if (!(set > 5) || !route_ok() || !V.eng_ok || V.mass < 500 || p_lookahead <= 0) return NAN;
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

// choice = 1: the settings of the score function (core/gearscore.h) for the mode of now
static float floor_share()   // choice_floor_power of the mode, + choice_floor_power_varied while the pedal varies; 0 = choice_floor_rpm
{
    float b = by_mode(p_ch_pw_eco, p_ch_pw_norm, p_ch_pw_pw);
    return b > 0 ? fminf(1.0f, b + (S.varied && p_ch_pw_var > 0 ? p_ch_pw_var : 0.0f)) : 0;
}
// The choice floor of the mode (the score's own floor, core/gearscore.h): the lowest rpm under load. choice_floor_power* (a share of the
// peak power), else choice_floor_rpm over the torque plateau.
static float choice_floor_rpm()
{
    float fs = floor_share();
    return fs > 0 ? gs_floor_rpm(fs, idle_rpm(), p_ch_fl_idle) : V.plateau + p_ch_floor;
}

static gs_cfg score_cfg(int skips, int skips_up, int skips_later, int low_edge, int coast, float pedal, float v_cap, int capped)
{
    gs_cfg c = {};
    c.comfort = by_mode(p_ch_eco, p_ch_norm, p_ch_power);
    c.w_rpm = p_ch_rpm;
    c.w_shift = p_ch_shift;
    c.stay = by_mode(p_ch_stay, p_ch_stay, p_ch_stay_pw);
    c.idle = idle_rpm();
    c.skips = skips;
    c.skips_up = skips_up;
    c.skips_later = skips_later;
    c.floor = low_edge ? 0 : p_ch_floor;   // (coasting for a while and the engine brake keep the window's low edge)
    c.floor_cur = !coast;                  // (a lift-off does not make the gear it just skipped to a wrong one)
    c.hyst = p_hyst;
    c.horizon = 8;                 // s: the speed the demanded acceleration reaches in that time decides how soon a gear needs the next shift
    c.learn_res = learned().res;   // what the no-pull lessons of this grade ask of an upshift
    c.pull = pedal >= 0.8f && !capped ? pedal : 0;   // the driver's pedal near the floor asks for the truck's pull, not for what this gear gives (a cap holds the throttle below it: no pull then)
    c.time_obj = p_ch_obj > 0;
    c.a_min = p_ch_amin;
    c.k_shift = by_mode(p_ch_ts_eco, p_ch_ts_norm, p_ch_ts_pw);
    c.floor_pw = low_edge ? 0 : floor_share();
    c.floor_idle_frac = p_ch_fl_idle;
    c.cross = p_pw_cross > 0 && pedal >= by_mode(2.0f, 0.95f, 0.8f);   // (Eco: no pedal is that far)
    c.v_cap = v_cap;
    c.up_pt = S.up_pt;
    c.w_over = p_ch_over;
    c.w_land = p_ch_land;
    return c;
}

// The rpm under which decide_gear's downshift rule (choice 0) takes a gear off: the demand-raised point only counts when this gear
// lacks pull; a gear that can still just hold the speed on a climb (needs 75-100 % of its full-throttle force) while the speed falls
// under real throttle: the point rises smoothly to the torque plateau, so the revs do not sag first (Damien 24-10: "subtieler, zonder bog
// down"). No kickdown behavior: it needs the load, not a burst of throttle; over 100 % the lugging rule takes over. *base = the point
// before the load raised it.
static float dn_point(int cur, float v, float thr, float f_res, float *base)
{
    float fmax = force_at(cur, v, 1.0f);
    float dn_eff = (fmax - f_res) / V.mass < a_power() ? S.dn_pt : base_dn(cur);
    float load = fmax > 1 ? f_res / fmax : 0;
    *base = dn_eff;
    if (p_load_from < 1 && thr > 0.25f && T.accel < -0.1f && load > p_load_from) dn_eff = fmaxf(dn_eff, lerpf(dn_eff, V.plateau + 50, smooth(p_load_from, 1.0f, load)));
    return dn_eff;
}

// The deceleration assist's downshift (see decide_gear): the lowest gear (one or two below cur) that lands, at the speed the shift ends at, inside the
// engine's brake range (rpm from its lower end up to the cap: a fifth of the range's width under the top, and 88 % of the rpm limit);
// cur if none does. *cap = that cap (rpm).
static int assist_down(int cur, float v, float *cap)
{
    float lo = V.eng.range[R_BRAKE][0], hi = V.eng.range[R_BRAKE][1];
    *cap = fminf(hi - 0.2f * (hi - lo), V.eng.rpm_limit * 0.88f);   // (under the 0.9 of the limit where an upshift is vetoed while slowing: no flip back up)
    if (!(hi > lo) || !(*cap > lo)) return cur;
    // (the speed at the end of a shift of the length the engine says, not the longer one the braking rule takes to be on the low side:
    // here the top is what matters, and the clutch is open as long as that)
    float v_e = fmaxf(0.5f, v - fmaxf(0.0f, -T.accel) * V.eng.shift_time);
    int best = cur;
    for (int l = cur - 1; l >= 1 && l >= cur - (p_eb_skip > 0 ? 2 : 1); l--) {
        float r = rpm_at(l, v_e);
        if (r > *cap) break;
        if (r >= lo) best = l;
    }
    return best;
}

static int decide_gear(void *self, int *why)
{
    *why = W_NONE;
    int cur = gb_gear(self), n = n_gears();
    float v = fabsf(T.speed);
    double now = sim_s();
    if (!V.eng_ok || n < 2 || cur < 0 || cur > n || wheel_radius() <= 0.1f) { *why = W_NO_DATA; return 0; }

    float f_res = resist(v);
    float brake = fmaxf(T.in_brake, T.eff_brake);
    if (brake > 0.05f) S.brake_at = sim_s();
    int ebrake = T.motor_brake || T.retarder > 0 || S.eb_level > 0.01f;
    int roll = roll_wanted(v, f_res, ebrake, brake, now - S.pedal_on_at);
    if (S.rolling) {
        if (roll) { *why = W_ECO_ROLL; return ROLL; }
        S.rolling = 0;
        S.roll_at = now;
        if (cur == 0) {
            *why = W_ROLL_END;
            S.roll_exit = roll_exit_gear(v, ebrake, brake);
            S.roll_exit_at = now;
            return S.roll_exit;
        }
    }
    if (cur < 1) { *why = W_NEUTRAL; return 0; }
    // [lowspeed] owner=1: a pull-away keeps the start gear until the clutch is locked (+ 0.5 s), then TrueShift has the truck. Only the
    // protections shift: a locked engine under idle + 100 (it would stall), or at the limiter (the gear is too low for the speed). While the
    // clutch slips the engine speed is not the driveline's, so a low rpm is no stall there (the slip law opens the clutch for that).
    if (low_launch_hold()) {
        float er = T.rpm, lm = V.eng.rpm_limit;
        int slipping = clutch_seen() >= 0.02f;
        if ((slipping || er > idle_rpm() + ioff(100)) && !(lm > 0 && er > 0.97f * lm)) { *why = W_LOW_SPEED_HOLD; return cur; }
    }
    if (!S.ours) { *why = W_LOW_SPEED; return 0; }

    float rpm = rpm_at(cur, v);
    float lim = V.eng.rpm_limit;
    float thr = S.thr;
    // lift-off by the pedal itself: the game's throttle reads 0.08 at pedal 0 (24-10, 1831 s), which hid the coast and the junction check
    int coast_p = (T.cruise > 0.5f ? thr : clampf(T.in_thr, 0, 1)) < 0.05f;
    double since = now - S.shift_at;
    int kick = S.kick_until > now || S.wk_until > now;
    S.up_pt = up_pt(cur);
    S.dn_pt = dn_pt(cur);
    S.floor_rpm = 0;

    if (S.mode == MODE_M) return decide_manual(cur, rpm, v, since, why);

    // Protections, also while a hint is held. Far above the limiter (a wrong
    // gear after neutral): straight to a gear that fits the speed.
    if (rpm > lim * 1.05f) { *why = W_WRONG_GEAR; return pick_for_speed(v, pw_mid()); }
    if (rpm > lim * (ebrake ? 0.99f : 0.96f) && cur < n) {
        int h = ebrake || S.thr < 0.75f ? cur + 1 : skip_target(cur, v, f_res, sinf(V.grade_deg * 0.0174533f));
        *why = h > cur + 1 ? W_REV_LIMIT_SKIP : W_REV_LIMIT;
        S.hold_until = 0;  // a held hint gear ends here, the next gear is TrueShift's again (friend 11-10: 2-3-4-5-6 all at the limiter)
        return h;
    }
    if (rpm < idle_rpm() + ioff(120) && cur > 1) { *why = W_STALL; return pick_down(cur, v, eco_dn(1) + 300); }

    // not right after a launch shift (ts4o: 1->3 by the game at 10 km/h with the pedal floored, then 3->2->3)
    if (kick && !S.wk_on && rpm < V.eng.range[R_POWER][0] + 100 && cur > 1 && since > 2.0 && clutch_seen() < 0.1f && v * 3.6f > 15) {
        int l = pick_down(cur, v, pw_mid() + 100);
        // never into a gear that is at its upshift point a moment later (ts3a: 3->2->3 at 13 km/h)
        while (l < cur && rpm_at(l, v) > V.eng.range[R_POWER][1] - 100) l++;
        if (l < cur) { *why = W_KICKDOWN; return l; }
    }

    // A hint gear is held, but not up to the limiter: pulling past the top of the power range hands it back
    // (friend 11-10: hint down to 2 at 8 km/h, then 2->3->4->5->6 each at ~1900 rpm with 30% throttle).
    if (S.hold_until > now) {
        if (!(rpm > V.eng.range[R_POWER][1] && S.thr > 0.05f)) { *why = W_HINT_HOLD; return cur; }
        S.hold_until = 0;
        logv(1, "hint hold ended at %.0f rpm (top of the power range)", rpm);
    }

    if (roll && since > 1) {
        S.rolling = 1;
        g_rolling = 1;  // right away: Cruise keeps the game from canceling cruise when the box goes to neutral
        S.roll_at = now;
        S.roll_kmh = v * 3.6f;
        *why = W_ECO_ROLL;
        return ROLL;
    }

    // Route look-ahead (navigation on): what comes in the next seconds.
    int rk = p_lookahead > 0 && route_ok();
    float crest = rk ? route_crest_m(v * 10) : -1;
    int top_near = crest >= 0 && crest < v * 8;
    float jn = rk ? route_junction_m() : -1;
    // Only a junction we have to slow down for (a bend in the route there, or
    // the pedal released towards it): ts3d held 8 at 1500 rpm for a minute on a
    // road with a prefab every ~130 m while Damien pulled away gently.
    float vb = rk ? route_curve_speed(2.5f, 0.8f) : NAN;
    int junction_near = jn >= 0 && jn < fmaxf(150, v * 8) && v * 3.6f > 25 && ((vb == vb && vb < v - 2.5f) || coast_p);
    // Slowing for a junction / roundabout / bend on the route: braking
    // downshifts come sooner, so the gear fits pulling out of it (Damien 11-10).
    float vs = rk && jn >= 0 && jn < fmaxf(150, v * 10) ? route_curve_speed(2.5f, 0.3f) : NAN;  // gentle 0.3 m/s2: will we have to slow down?
    int slow_ahead = vs == vs && vs < v - 2.5f;
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
    if (g_decel_need > 0 && coast_p && (T.retarder > 0 || T.motor_brake) && cur > 1) {
        float cap = 0;
        S.ad_at = now; S.ad_gear = cur; S.ad_lead = V.eng.shift_time; S.ad_after = S.shift_at + p_min_gap;
        int l = assist_down(cur, v, &cap);
        S.ad_cap = cap;
        if (l < cur && since > p_min_gap) { *why = W_ASSIST_DOWN; return l; }
    }

    // Engine brake / retarder: keep the rpm up, more for a stronger stage; the
    // full brake range when the game's "downshift with engine brake" is on.
    if ((T.motor_brake || S.eb_level > 0.01f) && thr < 0.05f) {
        float lo = V.eng.range[R_BRAKE][0], hi = V.eng.range[R_BRAKE][1];
        int full = p_eb_down > 0 || (p_eb_down < 0 && S.eb_full);
        float keep = full ? lo : lerpf(fminf(rs_brake(by_mode(p_eb_light - 100, p_eb_light, p_eb_light + 150)), lo), lo, S.eb_level);
        S.bd_at = now; S.bd_gear = cur; S.bd_floor = keep - 50; S.bd_lead = 0; S.bd_after = S.shift_at + p_min_gap;
        if (rpm < keep - 50 && cur > 1 && since > p_min_gap) {
            int l = pick_down(cur, v, full ? 0.5f * (lo + hi) : fminf(keep + 150, hi - 100), true);
            // two at a time when the second still fits under the brake range: every downshift costs a throttle blip
            // from the game (Damien 11-10: "gas and engine brake at once", 9->8->...->2 one by one)
            if (p_eb_skip > 0 && l == cur - 1 && cur > 2) {
                float r2 = rpm_at(cur - 2, v);
                if (r2 < hi - 100 && r2 < V.eng.rpm_limit * 0.93f) l = cur - 2;
            }
            if (l < cur && rpm_at(l, v) < hi) { *why = full ? W_ENGINE_BRAKE : W_ENGINE_BRAKE_LEVEL; return l; }
        }
        *why = W_ENGINE_BRAKE_HOLD;
        return cur;
    }

    // choice 1 with choice_objective 1: also slowing hard with the pedal off and no brake seen (a brake the telemetry misses, AEB), and the
    // gear is judged at the speed the shift will end at (ride: braking from 70, 10->8 only at 20 km/h and 638 rpm, then the stall rule twice)
    int stop_c = p_choice > 0 && p_ch_obj > 0;
    if (brake > 0.05f || (stop_c && coast_p && T.accel < -0.8f)) {
        // stay above the game's neutral threshold, which rises with brake^2
        float b = fminf(brake, 1.0f), opens = 0.8f * idle_rpm() + b * b * idle_rpm();
        float floor = fmaxf(fmaxf(idle_rpm() + ioff(250), eco_dn(cur) - 100), opens + ioff(250));
        if (slow_ahead) floor = fmaxf(floor, V.plateau + 200);
        // (choice 1: the speed at the end of a shift started now, at the deceleration of now: under the brake the revs fall through a slow
        // box's shift, and a gear chosen for the speed of now lands under the floor or at the stall rule)
        float v_l = stop_c ? fmaxf(0.5f, v - fmaxf(0.0f, -T.accel) * (V.eng.shift_time + 0.25f)) : v;
        S.bd_at = now; S.bd_gear = cur; S.bd_floor = floor; S.bd_lead = stop_c ? V.eng.shift_time + 0.25f : 0; S.bd_after = S.shift_at + 0.5;
        if (rpm_at(cur, v_l) < floor && cur > 1 && since > 0.5) {
            // land high, so that fewer downshifts are needed on the way down
            int l = pick_down(cur, v_l, eco_up(1) + by_mode(-300, -200, -100), true);
            if (l < cur) { *why = slow_ahead ? W_BRAKING_JUNCTION : W_BRAKING; return l; }
        }
        *why = W_BRAKING_HOLD;
        return cur;
    }

    // (ts_brake_down_eta: the downshift point of choice 0 is a downshift to expect in braking too; it is noted before the dwell, which it counts)
    if (p_choice < 1 && cur > 1) { float b; S.bd_at = now; S.bd_gear = cur; S.bd_floor = dn_point(cur, v, thr, f_res, &b); S.bd_lead = 0; S.bd_after = S.shift_at + p_min_gap; }
    if (since < p_min_gap) { *why = W_MIN_GAP; return cur; }

    // The last upshift has no pull (strain built up): back down, and learn.
    if (S.up_gear == cur && S.strain > 1.0f && cur > 1) { *why = W_NO_PULL; return cur - 1; }

    // A climb starts within ~3 s that this gear won't hold: one down now, at
    // the foot, instead of lugging into it.
    if (rk && thr >= by_mode(0.6f, 0.5f, 0.35f) && cur > 1 && since > 2 && !top_near && !top_ok) {
        float g_now = grade_pct(), g_climb = route_grade(v, v + fmaxf(200, v * 10));
        float g_start = route_max_grade(0, v * 3);
        if (g_climb == g_climb && g_start == g_start && g_climb > g_now + 1.5f && g_start > g_now + 1.0f &&
            force_at(cur, v * 0.95f, 1.0f) < resist_grade(v * 0.95f, g_climb) && rpm_at(cur - 1, v) < lim * 0.9f) {
            // the highest gear that holds the climb (else the one nearest the power band)
            int l = cur - 1;
            while (l > 1 && force_at(l, v * 0.95f, 0.95f) < resist_grade(v * 0.95f, g_climb) &&
                   rpm_at(l - 1, v) < lim * 0.9f)
                l--;
            *why = W_CLIMB_AHEAD;
            return l;
        }
    }

    // Downshift: below the downshift point, or lugging (full pedal, losing
    // speed and no force left in this gear).
    float fmax = force_at(cur, v, 1.0f);
    // A gear that cannot even hold the speed at full throttle any more (a steep climb): no waiting for the throttle to come up (cruise and
    // ACC raise it slowly) or for the rpm to sag (Damien 24-10, 545-553 s: 10-11 % at 43 km/h in 12, held to 759 rpm and 35 km/h, the lead
    // pulled away 13 -> 58 m). It needs real throttle (> 0.25) and a speed that is falling, so a burst of gas on the flat does nothing.
    int cannot_hold = thr > 0.25f && fmax < 1.05f * f_res && T.accel < -0.1f;
    int lug = (thr > 0.8f || cannot_hold) && T.accel < 0 && fmax - f_res < V.mass * 0.03f && rpm < V.eng.range[R_POWER][0] + 100;
    // choice 1: full throttle under the torque plateau is lugging too, even while the truck still gains a little speed (ride: 34.6 t in 10
    // at 809 rpm, throttle 1.00, 0.3-0.5 m/s2, held for 5 s). Not within the dwell after an upshift (the scores chose that landing), and only
    // when the gear below still fits under the top of the power range
    if (p_choice > 0 && !lug && thr > 0.8f && rpm < V.plateau && cur > 1 && !(S.shift_dir > 0 && since < p_dwell_ud) &&
        rpm_at(cur - 1, v) < V.eng.range[R_POWER][1] - p_hyst)
        lug = 1;
    // The demand-raised point, and a climb's raise of it (the gear that can just hold the speed): see dn_point.
    float dn_base, dn_eff = dn_point(cur, v, thr, f_res, &dn_base);
    if (cur > 1 && ((p_choice < 1 && rpm < dn_eff) || lug)) {   // choice 1: only lugging; the scores decide the rest below
        // maneuvering (Damien 24-10, 3856-3877 s: 6>4>5>3>4>5>6>7>8>9 in 21 s between 6 and 25 km/h): at a gentle pedal and a low speed only
        // a near-stall or lugging takes a gear off, not the demand-based downshift point
        if (p_calm > 0 && V.mass > 6000 && v * 3.6f < p_calm && !lug && !kick && thr < 0.6f && rpm > idle_rpm() + ioff(230)) { *why = W_LOW_SPEED_HOLD; return cur; }
        int deep = rpm < dn_base - 100 || rpm < idle_rpm() + ioff(250);   // the load-raised point does not skip the anti-hunt dwell
        if (S.shift_dir > 0 && since < p_dwell_ud && !deep && !lug) { *why = W_ANTI_HUNT_DOWN; return cur; }
        // Almost over the top: hold on, it gets easier in a moment.
        if (top_near && rpm > idle_rpm() + ioff(300)) { *why = W_TOP_AHEAD; return cur; }
        // A top this gear still gets over within the band: let the speed sag instead (PCC).
        if (top_ok || top_over) {
            if (top_ok && !S.crest_said) {
                S.crest_said = 1;
                DEV_NOTE(pcc_top());
                logv(1, "top in %.0f m: holding %d at %.0f km/h (gets over above %.0f km/h)", top_m, cur, v * 3.6f, top_floor);
            }
            *why = top_ok ? W_TOP_REACHABLE : W_OVER_TOP;
            return cur;
        }
        // In the cruise band (PCC): a few km/h under the set speed and this gear still pulls on this grade (not
        // just lugging along): let the speed come back slowly instead of a downshift for pull. Not with the driver's
        // own pedal (overtaking with cruise on wants the pull).
        if (T.cruise > 0.5f && !kick && T.in_thr < 0.1f && v >= T.cruise - crest_band() && rpm > band_rpm() && (fmax - f_res) / V.mass > 0.06f) {
            *why = W_CRUISE_BAND;
            return cur;
        }
        float target = lug ? pw_mid() : lerpf(0.5f * (eco_dn(cur - 1) + eco_up(cur - 1)) + 100, pw_mid(), pw_k());
        int l = pick_down(cur, v, target);
        if (l == cur) l = cur - 1;
        *why = lug ? W_LUGGING : (cur - l > 1 ? W_DOWN_SKIP : W_DOWN);
        return l;
    }

    // Brake air low (Damien: the tank ran empty twice): the compressor runs on the engine, and at ~900 rpm it barely
    // keeps up. Below air_charge_psi keep the rpm up: no upshift below air_charge_rpm, a downshift when well under it.
    float air_charge = air_edge_psi(p_air_psi, AIR_MARGIN_CHARGE);   // the truck's warning + 35 with [core] air_from_truck
    if (air_charge > 0 && T.air_psi > 1 && T.air_psi < air_charge && thr > 0.02f) {
        static double said;
        if (sim_s() - said > 10) { said = sim_s(); logline("air %.0f psi: keeping the rpm up to charge it", T.air_psi); }
        if (rpm < rs_abs(idle_rpm(), p_air_rpm) - 250 && cur > 1) {
            int l = pick_down(cur, v, rs_abs(idle_rpm(), p_air_rpm) + 100);
            if (l < cur && rpm_at(l, v) < V.eng.rpm_limit * 0.9f) { *why = W_AIR_CHARGE; return l; }
        }
        if (cur < n && rpm_at(cur + 1, v) < rs_abs(idle_rpm(), p_air_rpm)) { *why = W_AIR_CHARGE; return cur; }
    }

    if (junction_near) { *why = W_JUNCTION_AHEAD; return cur; }

    int coast = coast_p;
    int skips = skip_allowed(), skips_free = skips;   // (skips_free: what is allowed once the takeover is a few seconds old)
    float grade = sinf(V.grade_deg * 0.0174533f);
    // No skipping right after taking over from the game (it may have started
    // in a high gear) or close above the takeover speed.
    // The launch is the exception (Damien 12-10: 1 -> 2 -> 3 -> 4 -> 5 one by one between 12 and 20 km/h, "skip one, also in Eco"): from
    // the takeover speed the first gears may be skipped; the rpm and pull checks below still decide.
    // (while pulling away only 1 s after the takeover, else 3: the game starts a light truck in 4 and the next two shifts come within
    // seconds; after a pause or a teleport the gear may be wrong; the rpm and pull checks below still stop a skip that lands too low)
    if (!(cur <= 2 && v * 3.6f >= p_on_kmh) && (now - S.ours_at < (S.ours_launch ? 1.0 : 3.0) || v * 3.6f < p_on_kmh + 5)) skips = 0;
    float v2 = fmaxf(0.5f, v - f_res / V.mass * V.eng.shift_time);

    // Early upshift: light load, the new gear lands on the torque plateau and
    // gives the force asked for now with reserve to spare. Failed attempts
    // (had to come back down) block that situation for a while.
    // Without route data be careful: no early upshift when a climb starts
    // (grade up 0.5 % in 3 s), and more reserve unless downhill.
    int climbing = grade_pct() - S.grade_hist[3] > 0.5f;
    // Only while pulling steadily: not lifting off (throttle well below the
    // recent demand), not slowing down, not right after taking over.
    // not within 6 s after braking below 60 km/h (stop and go: a brake, a shift up, the next brake; 24-10, 231-241 s: 12>10>11>9)
    int steady = thr >= 0.15f && thr >= 0.6f * S.demand && T.accel > -0.15f && now - S.ours_at > 3 && (now - S.brake_at > 6 || v * 3.6f >= 60) &&
                 v * 3.6f >= p_on_kmh + 5;
    // With the route: the grade of the next seconds counts, no extra caution,
    // and a little less reserve when nothing climbs ahead.
    float g_ahead = NAN;
    if (rk) {
        float a = route_grade(0, v * 12), b = route_max_grade(0, v * 5);
        if (a == a && b == b) g_ahead = fmaxf(a, b - 0.5f);
        climbing = 0;
    }
    // choice = 1: of the gears the rules above left open, the best score. The rules that came before (limiter, stall, braking, engine
    // brake, kickdown, hints, min gap, no pull, climb ahead, lugging, junction) stand; the vetoes and timers below are the same ones the
    // upshift and downshift paths use. No candidate at all (a wrong gear far from the window) falls through to the paths below.
    if (p_choice > 0) {
        float gp = grade_pct();
        S.pk = fmaxf(gp, S.pk - 0.15f * (float)clampf((float)(now - S.pk_at), 0, 60)); S.pk_at = now;
        float g_pk = gp > 1.5f ? fmaxf(gp, S.pk) : gp;
        if (g_ahead == g_ahead) g_pk = fmaxf(g_pk, g_ahead);
        float f_hold = fmaxf(f_res, resist_grade(v, g_pk));      // what holding the speed on the worst grade of the next seconds asks
        // what is asked: the force of the last seconds (up fast, down slow), but no more than the pedal gives in this gear now: right after
        // a shift the old gear's force would still be in it and make every higher gear look out of reach. Never less than the pedal of now
        // asks in this gear: the filter lags 0.3 s behind a pedal going down, and the floor would take a steady load for it (after a takeover
        // or a reset it starts from 0). A pedal near the floor asks more than this gear gives: gs_cfg.pull
        float need = fmaxf(fminf(fmaxf(S.f_need, force_at(cur, v, S.thr)), force_at(cur, v, fmaxf(S.thr, S.demand))), f_hold);
        // a cap (the own speed limiter, the rollover guard, launch control) holds the throttle below the pedal: the pedal's wish is not a
        // demand then, what counts is the force the capped throttle gives (what it takes to hold the speed)
        int capped = thr_capped() && clampf(T.in_thr, 0, 1) > 0.1f;
        // (choice_objective 1 with the wish pedal on: what the driver wishes, not the throttle the wish gives in this gear, which is the wish
        // only where the gear can deliver it; the same for the cruise with [cruise] force_mode 1: the force it asks, not its throttle)
        float f_wish = p_ch_obj > 0 && !coast_p ? pedal_wish_force(v) : -1;
        if (f_wish < 0 && p_ch_obj > 0 && !coast_p) f_wish = cruise_wish_force(v);
        if (f_wish >= 0 && !capped) need = fmaxf(f_wish, f_hold);
        // The cruise holds its set speed (own throttle, within 1.5 km/h of it and hardly accelerating, the driver's pedal off): what is asked is
        // the force to hold the speed, not the filtered need that is still running down after the approach. That keeps the score steady, like a
        // real steady drive: no pull in hand to pay for, so the shift to a gear that sits lower is cheap (ride 7: 22 s in 11 at 83 km/h before the 12th)
        const bool cc_hold = p_choice > 0 && g_cruise_smooth && T.cruise > 0.5f && T.in_thr < 0.05f && fabsf(v - T.cruise) * 3.6f < 1.5f && fabsf(T.accel) < 0.08f;
        if (cc_hold) need = f_hold;
        gs_out so;
        // choice_route_cap: the speeds the scores look ahead to stop at what the route allows: the speed allowed now for the bends and
        // junctions ahead at a gentle 0.3 m/s2, less what the horizon's 8 s at this speed use of the way there (ride: 7->9 at 34 km/h,
        // a junction ahead, the engine brake took 9 back to 7 three seconds later)
        float v_cap = 0;
        if (rk && p_ch_route_cap > 0) {
            float vc = route_curve_speed(2.5f, 0.3f);
            if (vc == vc) v_cap = sqrtf(fmaxf(0.25f, vc * vc - 2 * 0.3f * v * 8));
        }
        // (no skipping with the pedal off, and no skip up on a climb of 1.5 % and more (the upshift path: 2 %): the gear skipped to had no pull later)
        int sup = coast_p || grade > 0.015f ? 0 : skips;
        // skip_land_frac: a skip must land a share of the way up from the choice floor to the upshift point (not at a near-flat pedal, where a
        // hard launch may still skip). Otherwise only +1 is offered, and the score still decides between +1 and staying.
        if (sup > 0 && p_skip_land_frac > 0 && clampf(T.in_thr, 0, 1) < 0.9f && cur + 2 <= n) {
            float fl = choice_floor_rpm(), lm = fl + p_skip_land_frac * (S.up_pt - fl);
            if (rpm_at(cur + 2, v2) < lm) sup = 0;
        }
        gs_eval(&so, cur, n, v, need, f_hold, coast_p, score_cfg(coast_p ? 0 : skips, sup, grade > 0.015f ? 0 : skips_free, ebrake || (coast_p && S.demand < 0.05f), coast_p, clampf(T.in_thr, 0, 1), v_cap, capped));
        // kickdown_by_wish: the kickdown is due (wish_kick_update). Of the lower gears the window and the floor let in that can give more than
        // this one, the best score, whatever the dwell times say (the stall rule and the limiter came before, and the window keeps the revs under
        // the limit). Nothing to take: no new trigger for a second
        if (S.wk_on && S.wk_until > now && since > 0.3 && clutch_seen() < 0.1f) {
            int kg = 0;
            float fc = vehicle_force_at(cur, v, 1.0f), bs = -1e30f;
            for (int g2 = cur - 1; g2 >= 1; g2--) {
                const gs_term &t = so.t[g2];
                if (!t.g || !t.ok || !(vehicle_force_at(g2, v, 1.0f) > fc * 1.02f) || t.rpm > so.hi_g[g2] - 200) continue;   // (not into a gear that is out of the window's top in a second or two)
                if (t.score > bs) { bs = t.score; kg = g2; }
            }
            if (kg) {
                *why = W_KICKDOWN;
                static double said;
                if (now - said > 0.5 || now < said) {
                    said = now;
                    logv(2, "kickdown by wish %d->%d: the pedal wishes %.0f N, gear %d gives %.0f N at full throttle (ratio %.2f, dissatisfaction %.3f, level %.3f); gear %d gives %.0f N (%.0f km/h, pedal %.2f)",
                         cur, kg, f_wish, cur, fc, S.wk_r, S.wk_d, by_mode(p_kl_eco, p_kl_norm, p_kl_pw), kg, vehicle_force_at(kg, v, 1.0f), v * 3.6f, T.in_thr);
                }
                return kg;
            }
            S.wk_until = 0; S.wk_d = 0; S.wk_retry = now + 1;
        }
        int w = so.best;
        // lift_upshift: the pedal off for lift_up_delay_s, and this driver's lifts usually outlast the shift: the gear up the scores prefer
        // now, not when the pedal comes back (one per lift). Also a gear up that only its shift cost keeps out. Not on a descent, not while
        // braking (the game's brake or our own request), not with a bend or junction within ~6 s (a stop at v/12 m/s2 over 6 s, at least
        // 0.3 m/s2) or the engine brake / retarder on, and never into a gear under the floor (at the speed of now and after the shift)
        int lift_g = 0;
        if (p_lift_up > 0 && coast_p && T.cruise < 0.5f && S.lift_at > 0 && S.lift_used != S.lift_at && now - S.lift_at >= p_lift_delay && cur < n &&
            lift_median() > V.eng.shift_time + 0.5f && grade_pct() > -1.0f && act_request() <= 0 && !ebrake && T.accel > -0.5f && !(jn >= 0 && jn < v * 6)) {
            float vb6 = rk ? route_curve_speed(2.5f, fmaxf(0.3f, v / 12)) : NAN;
            float fs = floor_share(), fl = fs > 0 ? gs_floor_rpm(fs, idle_rpm(), p_ch_fl_idle) : V.plateau + p_ch_floor;
            if (!(vb6 == vb6 && vb6 < v - 0.7f)) {
                for (int g = cur + 1; g <= n && g <= GS_MAX; g++) {
                    const gs_term &t = so.t[g];
                    if (!t.ok || t.rpm < fl || rpm_at(g, v) < fl || !so.cur_ok) continue;
                    if (g == w || (w == cur && t.score + t.shiftc > so.t[cur].score)) { lift_g = g; break; }
                }
            }
        }
        if (lift_g && w == cur) w = lift_g;
        if (w == cur) {   // (choice_objective 1: waiting for a skip that lands a little later)
            // the gear stays because it scores best: when the runner-up is a higher gear, say by how much and why (one line per 5 s), so a ride
            // log shows what kept the top gear out (ride 7: 22 s in 11 at 83 km/h, and nothing in the log said why)
            if (so.second > cur && so.cur_ok && (now - S.hold_log_at > 5 || now < S.hold_log_at)) {
                S.hold_log_at = now;
                const gs_term &c0 = so.t[cur], &c2 = so.t[so.second];
                logv(2, "score hold %d: gear %d %.3f (share %.2f, %.0f rpm; gas %.3f revs %.3f shift %.3f stay %.3f), gear %d %.3f (share %.2f, %.0f rpm; gas %.3f revs %.3f shift %.3f, %d shifts) (%.0f km/h, pedal %.2f, throttle %.2f, demand %.2f, asks %.0f N, to hold %.0f N (model bias %.0f N)%s, grade %.1f%%)",
                     cur, cur, c0.score, c0.share, c0.rpm, c0.gas, c0.rpmc, c0.shiftc, c0.stay, so.second, c2.score, c2.share, c2.rpm, c2.gas, c2.rpmc, c2.shiftc, c2.shifts,
                     v * 3.6f, T.in_thr, thr, S.demand, need, f_hold, V.f_bias, cc_hold ? ", cruise holding" : "", grade_pct());
            }
            *why = so.wait ? W_SKIP_WAIT : W_HOLD;
            return cur;
        }
        if (w) {
            bool forced = !so.cur_ok;   // this gear is outside the window or cannot make the demanded pull: a change is due, whatever the timers say
            int ret = -1, why_v = W_NONE;
            const char *not_steady = NULL;
            if (w < cur) {
                bool deep = forced && (rpm < idle_rpm() + ioff(250) || so.t[cur].share > 1.0f || kick);
                if (!forced && !kick && p_calm > 0 && V.mass > 6000 && v * 3.6f < p_calm && thr < 0.6f && rpm > idle_rpm() + ioff(230)) ret = cur, why_v = W_LOW_SPEED_HOLD;
                else if (S.shift_dir > 0 && since < p_dwell_ud && !deep && !kick) ret = cur, why_v = W_ANTI_HUNT_DOWN;
                // (choice_objective 1: no flip back down soon after an upshift, also not for a gear the floor or the demand has put out; kickdown,
                // the stall rule and braking come before this, revs near idle or under the window and a gear without a_min at full throttle go through)
                else if (p_ch_obj > 0 && S.shift_dir > 0 && since < p_ch_flip && !kick && rpm >= fmaxf(idle_rpm() + ioff(250), so.lo) && (fmax - f_res) / V.mass >= p_ch_amin)
                    ret = cur, why_v = W_ANTI_HUNT_DOWN;
                else if (top_near && rpm > idle_rpm() + ioff(300)) ret = cur, why_v = W_TOP_AHEAD;
                else if (top_ok || top_over) ret = cur, why_v = top_ok ? W_TOP_REACHABLE : W_OVER_TOP;
                else if (T.cruise > 0.5f && !kick && T.in_thr < 0.1f && v >= T.cruise - crest_band() && rpm > band_rpm() && (fmax - f_res) / V.mass > 0.06f)
                    ret = cur, why_v = W_CRUISE_BAND;
                else { ret = w; why_v = cur - w > 1 ? W_DOWN_SKIP : W_DOWN; }
            } else {
                bool wants_power = clampf(T.in_thr, 0, 1) >= 0.5f || thr >= 0.5f || thr > S.demand + 0.05f;
                bool jn_soon = jn >= 0 && jn < fmaxf(50.0f, v * 3);
                // our own cruise cut its throttle (a bend, a junction or a limit ahead) while the driver's pedal is off: that is no driver's lift,
                // so neither 'lifting' nor the gentle slowing it causes (not a brake) holds the gear (ride 7, 621-640 s: 10 held 10 s at 72 km/h)
                const bool cc_cut = p_choice > 0 && g_cruise_smooth && T.cruise > 0.5f && T.in_thr < 0.05f && thr < 0.05f && act_request() <= 0 && T.accel > -0.5f;
                double dwell = now - S.kd_at < by_mode(3, p_kick_dwell, 8) + 1 ? by_mode(3, p_kick_dwell, 8) : by_mode(p_dwell_du, p_dwell_du, 4.0f);
                // a change for a better score only while pulling steadily (the early upshift's conditions): not right after taking over, not
                // right after a brake at low speed, not onto a rising grade, not within the kickdown dwell, not while the truck is slowing, not
                // while the throttle in effect is far above the filtered demand (cruise control pulling up to its speed: the demand is still
                // on its way up, and the gear chosen for it has no pull a second later). The engine's throttle, not the raw pedal: under the Eco
                // pedal curve the pedal stands well above the throttle it gives, and would veto every upshift at a middle pedal
                // (choice_own_brake: our own brake request counts as a brake: the deceleration assist's does not show in the telemetry)
                double brake_at = p_ch_own_brake > 0 && S.own_brake_at > S.brake_at ? S.own_brake_at : S.brake_at;
                bool steady_up = now - S.ours_at > (S.ours_launch ? 1.0 : 3.0) && (now - brake_at > 6 || v * 3.6f >= 60) && !climbing &&
                                 !(now - S.kd_at < by_mode(3, p_kick_dwell, 8)) && (cc_cut || (T.accel > -0.15f && thr >= 0.6f * S.demand)) &&
                                 T.eff_thr <= fmaxf(thr, S.demand) + 0.15f;
                if (coast_p && S.mode == MODE_POWER && p_mode_char > 0 && now - S.pedal_on_at < 3 && rpm < lim * 0.9f) ret = cur, why_v = W_POWER_HOLDS;
                // (lift_upshift: the slowing of a coasting truck is not a reason; its own checks stand)
                else if (!wants_power && ((T.accel < -0.2f && w != lift_g && !cc_cut) || jn_soon) && rpm < lim * 0.9f) ret = cur, why_v = W_UP_VETO;
                else if (S.shift_dir < 0 && since < dwell && rpm < S.up_pt + 100) ret = cur, why_v = W_ANTI_HUNT_UP;
                // (choice_objective 1: no flip back up soon after a downshift unless the revs near the top of the window; and no upshift ladder:
                // after an upshift the next one waits until this gear has pulled for choice_up_wait shift times, the shift it cost)
                else if (p_ch_obj > 0 && S.shift_dir < 0 && since < p_ch_flip && rpm < so.hi - 100) ret = cur, why_v = W_ANTI_HUNT_UP;
                else if (p_ch_obj > 0 && p_ch_upwait > 0 && S.shift_dir > 0 && !forced && since < fmaxf(1.5f, p_ch_upwait) * V.eng.shift_time &&
                         rpm < so.hi - 100) ret = cur, why_v = W_UP_WAIT;
                // (kickdown_by_wish: no upshift into a gear that would not give the wish: the kickdown would take it back a moment later)
                else if (S.wk_on && !forced && f_wish >= 0 && f_wish > by_mode(p_kr_eco, p_kr_norm, p_kr_pw) * vehicle_force_at(w, v, 1.0f)) ret = cur, why_v = W_WISH_UP;
                else if (!forced && !steady_up && w != lift_g) {
                    ret = cur, why_v = W_HOLD;
                    // which of the steady conditions failed (for the veto line below)
                    not_steady = now - S.ours_at <= (S.ours_launch ? 1.0 : 3.0) ? "just taken over" : !(now - brake_at > 6 || v * 3.6f >= 60) ? "after a brake" :
                                 climbing ? "grade rising" : now - S.kd_at < by_mode(3, p_kick_dwell, 8) ? "kickdown dwell" : T.accel <= -0.15f ? "slowing" :
                                 thr < 0.6f * S.demand ? "lifting" : "throttle over the demand";
                }
                else if (w == lift_g) { ret = w; why_v = W_LIFT_UP; }
                else { ret = w; why_v = w - cur > 1 ? W_UP_SKIP : W_UP; }
            }
            *why = why_v;
            // the best gear is another one, but a veto keeps this one: one line per 5 s, the winner, its score, this gear's and the veto
            if (ret == cur && (now - S.veto_log_at > 5 || now < S.veto_log_at)) {
                S.veto_log_at = now;
                char cs[24] = "out";
                if (so.cur_ok) snprintf(cs, sizeof cs, "%.3f", so.t[cur].score);
                logv(2, "score veto %d->%d (%s%s%s): gear %d %.3f (share %.2f, %.0f rpm), gear %d %s (share %.2f, %.0f rpm) (%.0f km/h, pedal %.2f, throttle %.2f, demand %.2f, asks %.0f N, to hold %.0f N%s, grade %.1f%%)",
                     cur, w, why_text[why_v], not_steady ? ": " : "", not_steady ? not_steady : "", w, so.t[w].score, so.t[w].share, so.t[w].rpm, cur, cs,
                     so.t[cur].share, so.t[cur].rpm, v * 3.6f, T.in_thr, thr, S.demand, need, f_hold, cc_hold ? ", cruise holding" : "", grade_pct());
            }
            if (ret != cur && now - S.score_log_at > 0.5) {
                S.score_log_at = now;
                int g2 = so.second;
                char nx[48] = "no other candidate";
                if (g2) snprintf(nx, sizeof nx, "gear %d %.3f", g2, so.t[g2].score);
                logv(2, "score %s %d->%d: gear %d %.3f (share %.2f, %.0f rpm), then %s (%.0f km/h, pedal %.2f, demand %.2f, asks %.0f N, window %.0f-%.0f rpm, grade %.1f%%, %.1f t)",
                     ret > cur ? "up" : "down", cur, ret, w, so.t[w].score, so.t[w].share, so.t[w].rpm, nx, v * 3.6f, T.in_thr, S.demand, need, so.lo, so.hi, grade_pct(), V.mass / 1000);
            }
            return ret;
        }
    }

    // (not within the kickdown/lugging dwell either: the early path took 9 -> 10 two seconds after a lugging 10 -> 9 on a 2.3 % climb)
    if (steady && cur < n && S.demand < p_power_from && !(S.shift_dir < 0 && since < p_dwell_du) && !(now - S.kd_at < by_mode(3, p_kick_dwell, 8)) && !climbing) {
        float need = fmaxf(S.f_need, f_res), k = mode_reserve() + learned().res;
        if (g_ahead == g_ahead) {
            need = fmaxf(need, resist_grade(v, g_ahead));
            if (g_ahead <= fmaxf(grade_pct(), 0) + 0.3f) k -= 0.05f;
        } else if (grade_pct() > -0.5f) k += 0.1f;
        int best = cur;
        for (int h = cur + 1; h <= n && h <= cur + 1 + skips; h++) {
            float fl = cruise_floor(h, h > cur + 1 ? 100.0f : 0.0f);
            if (h == cur + 1) S.floor_rpm = fl;
            if (rpm_at(h, v2) < fl) break;
            if (force_at(h, v2, 1.0f) < k * need) break;
            // on a climb not into a gear the load rule (load_down_from) would take off again as soon as the speed dips
            if (p_load_from < 1 && grade > 0.01f && need > p_load_from * force_at(h, v2, 1.0f)) break;
            if (h > cur + 1 && (grade > 0.02f || (force_at(h, v2, 0.85f) - f_res) / V.mass < 0)) break;
            best = h;
        }
        // Light pull and no skip fits yet: do not take the single early upshift but keep the gear and let the revs build up past the
        // upshift point, until a skip lands above the downshift point (Damien 24-10: wait a bit longer when pulling away lightly,
        // then skip). The rpm path below makes the shift. At most skip_wait_s, at most 250 rpm over the upshift point, never near the limiter.
        bool wait = false;
        float skip_rpm = 0;
        if (best == cur + 1 && skips > 0 && p_skip_wait > 0 && cur + 2 <= n && grade <= 0.02f && T.accel > 0.05f && v > 1) {
            int h = cur + 2;
            float rc = rpm_at(cur, v2), rh = rpm_at(h, v2);
            if (S.skip_wait_at != 0 && now - S.skip_wait_last > 1.0) S.skip_wait_at = 0;   // an old wait, not this one
            float low = fmaxf(fmaxf(dn_pt(h), base_dn(h)) + p_hyst, idle_rpm() + ioff(350));
            // the rpm in this gear from which the skip lands on `low` AFTER the speed lost during the shift (v2 < v: 25-10, the wait aimed at
            // the landing at today's speed, so the skip then failed on 'rpm too low after shift' and became a single step)
            skip_rpm = rh > 1 ? fmaxf(low * (rc / rh) * (v / fmaxf(v2, 0.5f)), S.up_pt + 10) : 0;
            float t_to = (skip_rpm - rpm) / fmaxf(1.0f, rpm * T.accel / v);
            if (skip_rpm > 0 && skip_rpm <= fminf(S.up_pt + p_skip_over, 0.85f * V.eng.rpm_limit) && t_to <= p_skip_wait &&
                (S.skip_wait_at == 0 || now - S.skip_wait_at <= p_skip_wait)) {
                if (S.skip_wait_at == 0) S.skip_wait_at = now;
                S.skip_wait_last = now;
                wait = true;
            }
        }
        if (wait && rpm < skip_rpm) { *why = W_SKIP_WAIT; return cur; }
        if (best > cur && !wait) { S.skip_wait_at = 0; *why = best - cur > 1 ? W_UP_EARLY_SKIP : W_UP_EARLY; return best; }
    }

    // Upshift on rpm: above the upshift point, the new gear must stay above its
    // own downshift point and still pull after the speed lost during the shift.
    // On a climb the grade that counts is the steepest of now, the last seconds (a peak that fades 0.15 %/s) and the route
    // ahead (Damien 11-10, ride 3 at 400 s: 8 -> 9 in a 4.1 % dip of a 5-6 % climb at 54 km/h, back to 8 after 1.6 s, lugging).
    // The speed lost in the shift is reckoned with that grade too, and a climb asks a real margin, not 0.02-0.05 m/s2.
    float gp = grade_pct();
    S.pk = fmaxf(gp, S.pk - 0.15f * (float)clampf((float)(now - S.pk_at), 0, 60)); S.pk_at = now;
    float g_up = gp > 1.5f ? fmaxf(gp, S.pk) : gp;   // the peak only while still climbing (over the crest it is the route's job)
    if (g_ahead == g_ahead) g_up = fmaxf(g_up, g_ahead);
    float f_up = fmaxf(f_res, resist_grade(v, g_up));
    float v2u = fmaxf(0.5f, v - f_up / V.mass * V.eng.shift_time);
    if (cur < n && rpm > S.up_pt + (coast ? by_mode(50.0f, 200.0f, 200.0f) : 0)) {
        // Power: just off the pedal the gear stays (3 s), unless the revs are nearly at the limit
        if (coast && S.mode == MODE_POWER && p_mode_char > 0 && now - S.pedal_on_at < 3 && rpm < lim * 0.9f) { *why = W_POWER_HOLDS; return cur; }
        // Rolling out or lifting towards a junction: no upshift, it only ends in a shift back (Damien 24-10, 1831 s and 3421 s). Not with
        // the pedal at 0.5 or going up: whoever wants power at a junction gets it.
        bool wants_power = clampf(T.in_thr, 0, 1) >= 0.5f || thr >= 0.5f || thr > S.demand + 0.05f;
        bool jn_soon = jn >= 0 && jn < fmaxf(50.0f, v * 3);
        if (!wants_power && (T.accel < -0.2f || jn_soon) && rpm < lim * 0.9f) { *why = W_UP_VETO; return cur; }
        if (S.shift_dir < 0 && since < (now - S.kd_at < by_mode(3, p_kick_dwell, 8) + 1 ? by_mode(3, p_kick_dwell, 8) : by_mode(p_dwell_du, p_dwell_du, 4.0f)) && rpm < S.up_pt + 100) { *why = W_ANTI_HUNT_UP; return cur; }
        float a_req = lerpf(0.02f, 0.05f, S.demand);
        if (g_up > 2 && !coast) a_req = fmaxf(a_req, lerpf(0.05f, 0.12f, clampf((g_up - 2) / 3, 0, 1)));
        int best = cur, maxg = cur + 1 + (coast_p ? 0 : skips);   // no skipping with the pedal off: it only ends in the next brake (sim Y1: 6>8 at the lift-off, 8>6 a second later)
        const char *block = "";
        for (int h = cur + 1; h <= n && h <= maxg; h++) {
            float rh = rpm_at(h, v2u);
            float a_h = (force_at(h, v2u, 0.95f) - f_up) / V.mass;
            if (rh < land_floor(h, a_h, h > cur + 1 ? skip_relax(v2u, (force_at(h, v2u, clampf(S.demand, 0.1f, 1.0f)) - f_up) / V.mass) : 0)) {
                block = "rpm too low after shift";
                break;
            }
            if (!coast && (force_at(h, v2u, 0.95f) - f_up) / V.mass < a_req) { block = "not enough pull"; break; }
            // on a climb the new gear must carry the load under load_down_from, or the load rule shifts straight back (10 > 9 > 10 > 8, 25-10)
            if (!coast && p_load_from < 1 && g_up > 1.0f && f_up > p_load_from * force_at(h, v2u, 1.0f)) { block = "too much load after shift"; break; }
            if (!coast && learned().res > 0 && rh < fmaxf(dn_pt(h) + p_hyst, idle_rpm() + ioff(350)) + learned().rpm &&
                rpm < S.up_pt + 150) { block = "no pull here before"; break; }
            if (h > cur + 1) {
                // skip_land_frac: a skip must land a share of the way up from the choice floor to the upshift point (a hard launch may still skip)
                if (p_skip_land_frac > 0 && clampf(T.in_thr, 0, 1) < 0.9f) {
                    float fl = choice_floor_rpm();
                    if (rh < fl + p_skip_land_frac * (S.up_pt - fl)) { block = "skip: lands too low for skip_land_frac"; break; }
                }
                // At high demand only when the skipped-to gear still pulls hard (light truck).
                bool pull_away = v2u < 20 / 3.6f;   // (25-10, Damien: under 20 km/h a skip is fine even at full pedal)
                float a_skip = skip_accel() * (S.demand > 0.75f && !pull_away ? 2.0f : 1.0f);
                if (grade > 0.02f || (force_at(h, v2u, 0.85f) - f_up) / V.mass < a_skip) {   // (coasting never gets here: maxg)
                    block = grade > 0.02f ? "skip: climbing" : "skip: too little pull in the skipped-to gear";
                    break;
                }
                // and keeps most of the pull of the next gear (a car must not jump 2 -> 5 at full throttle)
                if (S.demand > 0.75f && !pull_away &&
                    force_at(h, v2u, 1.0f) - f_up < 0.7f * (force_at(cur + 1, v2u, 1.0f) - f_up)) { block = "skip: loses too much pull at high demand"; break; }
            }
            best = h;
        }
        if (best > cur) {
            // why was this a single step when a skip was allowed (Damien 25-10: it skips too little when driving gently)? one line per 5 s
            if (best == cur + 1 && maxg > cur + 1 && cur + 2 <= n && (T.in_thr < 0.3f || v < 25 / 3.6f) && now - S.skip_log_at > 5) {
                S.skip_log_at = now;
                logv(2, "single up %d->%d, no skip: %s (%.0f km/h, %.0f rpm, pedal %.2f, demand %.2f, grade %.1f%%, %.1f t)", cur, best,
                     block[0] ? block : "the next gear above was not tried", v * 3.6f, rpm, T.in_thr, S.demand, grade_pct(), V.mass / 1000);
            }
            S.skip_wait_at = 0; *why = best - cur > 1 ? W_UP_SKIP : W_UP; return best;
        }
        if (now - S.blocked_log_at > 10) {
            S.blocked_log_at = now;
            logline("up blocked in %d: %s (%.0f km/h, %.0f rpm, demand %.2f, grade %.1f%%, %.1f t)", cur, block,
                    v * 3.6f, rpm, S.demand, grade_pct(), V.mass / 1000);
        }
        *why = W_UP_BLOCKED;
        return cur;
    }
    *why = W_HOLD;
    return cur;
}

// ---------------------------------------------------------------- hook stubs

// Incidents from the shifts (core/incident.cpp): the gearbox hunting (three shifts in a row that each reverse the direction
// within 10 s, none of them the driver's) and the driver fighting a shift (a hint against our last shift within 5 s).
static void track_shifts(int prev, int g, int hint, double now)
{
    static struct { double t; int from, to; } sh[3];
    if (!S.ours) { memset(sh, 0, sizeof sh); return; }  // below the takeover speed the game's own automatic shifts
    if (prev <= 0 || g <= 0 || fabsf(T.speed) * 3.6f < 5) return;  // neutral (eco-roll) is not a shift, nor is picking the launch gear at a stop
    if (hint) {
        if (sh[2].t && now - sh[2].t < 5 && (g - prev) * (sh[2].to - sh[2].from) < 0)
            incident("override", "hint %d->%d against our shift %d->%d, %.1f s later, %.0f km/h, %.0f rpm, mode %s", prev, g, sh[2].from, sh[2].to,
                     now - sh[2].t, fabsf(T.speed) * 3.6f, T.rpm, mode_name[S.mode]);
        return;
    }
    sh[0] = sh[1]; sh[1] = sh[2]; sh[2].t = now; sh[2].from = prev; sh[2].to = g;
    if (sh[0].t && now - sh[0].t < 10 && (sh[0].to - sh[0].from) * (sh[1].to - sh[1].from) < 0 && (sh[1].to - sh[1].from) * (g - prev) < 0)
        incident("hunt", "gears %d->%d->%d->%d in %.1f s at %.0f km/h, %.0f rpm, grade %.1f%%, mode %s", sh[0].from, sh[0].to, sh[1].to, g,
                 now - sh[0].t, fabsf(T.speed) * 3.6f, T.rpm, grade_pct(), mode_name[S.mode]);
}

static void note_gear_change(void *self)
{
    int g = gb_gear(self);
    if (g == S.last_gear) return;
    double now = sim_s();
    float v = fabsf(T.speed);
    int prev = S.last_gear;
    S.shift_dir = g > prev ? 1 : -1;
    S.shift_at = now;
    S.last_gear = g;
    if (S.cmd_why == W_KICKDOWN) { S.kick_until = 0; S.wk_until = 0; S.wk_d = 0; }
    if (g < prev && g > 0 && (S.cmd_why == W_KICKDOWN || S.cmd_why == W_LUGGING)) S.kd_at = now;
    int hint = prev > 0 && g > 0 && g != S.commanded && S.ours;
    int why = hint ? W_HINT : g == S.commanded ? S.cmd_why : W_GAME;
    DEV_NOTE(shift(prev, g, why_text[why], hint, now));
    track_shifts(prev, g, hint, now);
    if (why == W_LIFT_UP) S.lift_used = S.lift_at;   // one upshift per lift (set when it is in: decide_gear runs twice per update)

    // Did the last upshift hold? A downshift out of it for lack of force (or a
    // hint down) within 25 s means it did not.
    if (S.up_gear && prev == S.up_gear && g < prev) {
        // A plain downshift only counts when power was asked for (not when
        // the driver or the cruise control lets the truck slow down).
        // A climb that started after the upshift is terrain, not a bad upshift.
        // (within 5 s it counts anyway: a climb that went on steepening right after it is what the upshift should have seen)
        int asked = S.thr >= 0.5f, same_road = grade_pct() - S.up_grade < 1.0f || now - S.up_at < 5;
        if (now - S.up_at < 25 && why && same_road && !S.up_tainted &&
            (hint || why == W_NO_PULL || why == W_LUGGING || (asked && (why == W_DOWN || why == W_DOWN_SKIP))))
            up_failed(why_text[why]);
        else S.up_gear = 0;
    }
    // an upshift of ours (the hold reasons 'up veto' and 'up blocked' keep the gear, so they never become the command's reason)
    if (!hint && (why == W_UP || why == W_UP_SKIP || why == W_UP_EARLY || why == W_UP_EARLY_SKIP) && g > prev) {
        S.up_gear = g;
        S.up_tainted = 0;
        S.up_at = now;
        S.up_grade = grade_pct();
        S.up_kmh = v * 3.6f;
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
                mode_name[S.mode]);
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
    bool lcx = g_lcx == 1;
    if (lcx && !prev_lcx) done = 0;  // launch control just armed: pick the start gear again, with the brake held
    prev_lcx = lcx;
    if (kmh > 6) { done = 0; return; }
    if (done || kmh > 2.5f || !g_active || p_launch_fix <= 0 || p_start_fix <= 0 || p_launch_max <= 0 || T.parking_brake || (T.in_brake > 0.3f && !lcx) || !V.eng_ok || (!lcx && (S.mode == MODE_M || S.mode == MODE_POWER))) return;
    int cur = gb_gear(self);
    if (cur < 1 || gb_state(self) != 0 || now - S.shift_at < 1.0 || S.rolling || g_crawl_on || V.mass < 3000) return;
    // at a standstill with the pedal still up: the gear for a gentle pull-away (0.3); with the pedal down: for that pedal
    int pick = launch_pick(cur, fmaxf(clampf(T.in_thr, 0, 1), 0.3f));
    done = 1;
    if (pick > cur && takeover_launch_ok(kmh)) {
        logline("launch: the game started in %d, the physics say %d (%.1f t, grade %.1f%%): shifting at %.1f km/h", cur, pick, V.mass / 1000, grade_pct(), kmh);
        S.launch_want = pick; S.launch_at = now;
        gb_set_target(self, pick); gb_force(self);
        S.commanded = pick; S.cmd_why = W_START_CORRECTED;
    }
}

static float *stub_decide(void *self, float *out, unsigned gear)
{
    float *r = o_window(self, out, gear);
    if (!g_active) return r;
    refresh(self);
    note_gear_change(self);
    int why;
    int want = decide_gear(self, &why);
    S.why = why_text[why];
    S.want = want;
    S.target = gb_target(self);
    // Neutral goes in through the tail hook (the chooser path clamps to +-1).
    if (want > 0 && want != ROLL && want != gb_gear(self) && takeover()) gb_force(self);
    launch_correct(self);
    return r;
}

static int stub_choose(void *self)
{
    int stock = o_choose(self);
    if (!g_active) return stock;
    int ret = stock;
    int why = W_GAME;
    // A shift hint of the driver in M (the game's handler moves the target one gear, before this chooser runs). Out of its rpm window
    // (the rev limiter, 24-10 ride 124-130 s and 133-138 s) the chooser runs every tick and the call site writes its result into the
    // target: our "stay in this gear" erased the press until the pedal came off. So a target nobody here asked for is honored.
    // the physics' start gear the game did not pick (launch_correct): keep handing it back until it is in
    if (S.launch_want > 0 && (fabsf(T.speed) * 3.6f > 6 || sim_s() - S.launch_at > 2.5 || gb_gear(self) >= S.launch_want)) S.launch_want = 0;
    int hint_tgt = 0;
    if (S.mode == MODE_M && gb_state(self) == 0) {
        int cur = gb_gear(self), tgt = gb_target(self);
        if (cur > 0 && tgt > 0 && tgt != cur && tgt != S.commanded && abs(tgt - cur) <= 3 && tgt <= n_gears()) hint_tgt = tgt;
    }
    if (takeover()) {
        int w;
        refresh(self);
        int want = decide_gear(self, &w);
        if (want == ROLL) { ret = gb_gear(self); why = w; }
        else if (want > 0) { ret = want; why = w; }
    }
    if (S.launch_want > 0 && gb_gear(self) < S.launch_want) { ret = S.launch_want; why = W_START_CORRECTED; }
    if (hint_tgt && (ret == gb_gear(self) || (ret > gb_gear(self)) == (hint_tgt > gb_gear(self)))) {
        static double said;
        if (sim_s() - said > 0.5) { said = sim_s(); logv(2, "hint target %d seen in %d (rpm %.0f): honored", hint_tgt, gb_gear(self), T.rpm); }
        ret = hint_tgt; why = W_HINT;
    }
    // Every gear we hand back counts as ours, so that only the hint buttons
    // show up as shifts nobody here asked for.
    if (ret != gb_gear(self)) { S.commanded = ret; S.cmd_why = why; }
    return ret;
}

// The automatic's stop threshold: below it the clutch opens / neutral. Stock
// 0.8 x idle (+ a brake term the game adds), which pushes until ~2 km/h.
// While braking to a stop without throttle (below 15 km/h): idle + 150 rpm;
// the game's own brake term comes on top, so harder braking opens earlier.
// Not at speed: ride ts3a went to neutral at 74 km/h in 12 when braking hard.
static int g_stop_in;   // the stop threshold hook is in
static float stop_threshold(float idle, float thr, float brake)
{
    float stock = 0.8f * idle;
    // [lowspeed] owner=1: the open point is the owner's (the start gear's idle point, lowspeed.cpp); no fixed km/h edge, one definition of the stop
    if (low_owner_on() && !S.unsupported && !manual_box()) return low_stop_rpm(idle, thr, brake, T.gear);
    if (p_stop <= 0 || S.unsupported || manual_box() || thr > 0.01f || !(idle > 200 && idle < 2000)) return stock;
    // Crawl off (Damien): never creep on the clutch at idle; pedals free below 8 km/h opens it like a stop.
    // not with cruise on: in a jam the ACC rolls at walking pace, and neutral would cancel cruise (ts4p 534 s)
    if (brake < 0.05f && !(p_no_creep > 0 && !g_crawl_on && T.cruise < 0.5f && fabsf(T.speed) * 3.6f < 8)) return stock;
    if (fabsf(T.speed) * 3.6f > 15) return stock;
    return fmaxf(stock, idle + rs_off(idle, 150));
}

// Runs at the very end of every automatic update (also in neutral), right
// before the generic gearbox acts on the target gear: eco-roll goes in and
// out here (lka-roll-re notes part B).
static void tail_pre(void *self)
{
    if (!g_active || !takeover() || self != S.self) return;
    int g = gb_gear(self), st = gb_state(self);
    // After a roll: our exit gear as the target; the generic gearbox then
    // parks in its gap state (clutch open, gear 0) until there is throttle
    // (ts3a-c, neutral-exit-re notes § 1). There we do its own engage step.
    if (S.roll_exit > 0) {
        if (g != 0 || sim_s() - S.roll_exit_at > 3 || fabsf(T.speed) * 3.6f < p_off_kmh) {
            if (g == 0) {
                logline("roll end: gear %d did not engage in 3 s", S.roll_exit);
                incident("neutral", "after the eco-roll gear %d did not engage in 3 s (%.0f km/h)", S.roll_exit, fabsf(T.speed) * 3.6f);
            }
            S.roll_exit = 0;
            g_rolling = 0;
        } else {
            if (gb_target(self) != S.roll_exit) gb_set_target(self, S.roll_exit);
            if (st == 1 && gb_engage(self, S.roll_exit)) logline("roll end: engaged gear %d at %.0f km/h", S.roll_exit, fabsf(T.speed) * 3.6f);
            return;
        }
    }
    // Parked in the gap state while moving, with a gear as target (ts4q 604-616 s: a pause in the middle of an upshift
    // left gear 0 and the clutch open for 11 s at 38 -> 25 km/h: our cruise throttle waits for the shift to end, the
    // gearbox waits for throttle). After 1.5 s we do its engage step, as at the end of a roll.
    {
        int tgt = gb_target(self);
        if (st == 1 && g == 0 && tgt > 0 && !S.rolling && !g_crawl_on && fabsf(T.speed) * 3.6f > p_off_kmh) {
            if (!S.gap_since) S.gap_since = sim_s();
            // right after a pause the gap state is the pause's, not a shift's (pre.6 ride 609/1716 s: 1.5 s in neutral at 45-50 km/h)
            else if (sim_s() - S.gap_since > (sim_s() - g_reset_at < 5 ? 0.3 : 1.5) && gb_engage(self, tgt)) {
                logline("stuck in neutral for %.1f s at %.0f km/h: engaged gear %d", sim_s() - S.gap_since, fabsf(T.speed) * 3.6f, tgt);
                if (sim_s() - g_reset_at >= 5)  // right after a pause the gap state is normal: no incident
                    incident("neutral", "stuck in neutral for %.1f s at %.0f km/h, engaged gear %d", sim_s() - S.gap_since, fabsf(T.speed) * 3.6f, tgt);
                S.gap_since = 0;
            }
        } else S.gap_since = 0;
    }
    if (st != 0) return;
    if (S.rolling && g > 0) { gb_set_target(self, 0); S.commanded = 0; S.cmd_why = W_ECO_ROLL; return; }
    if (!S.rolling || g != 0) return;
    refresh(self);
    float v = fabsf(T.speed), brake = fmaxf(T.in_brake, T.eff_brake);
    int ebrake = T.motor_brake || T.retarder > 0 || S.eb_level > 0.01f;
    if (roll_wanted(v, resist(v), ebrake, brake, sim_s() - S.pedal_on_at)) { gb_set_target(self, 0); return; }
    S.rolling = 0;
    S.roll_at = sim_s();
    int pick = roll_exit_gear(v, ebrake, brake);
    gb_set_target(self, pick);
    S.roll_exit = pick;
    S.roll_exit_at = sim_s();
    S.commanded = pick;
    S.cmd_why = W_ROLL_END;
}

// The start gear by physics (see stub_start): lock speed and slip time at this mass and grade, never by a gear count. `stock` = the game's
// pick; the result may be above it (launch_max_kmh) or below it.
static int launch_pick(int stock, float thr, bool force)
{
    int pick = stock;
    if (p_start_fix > 0 && (force || takeover()) && V.eng_ok && wheel_radius() > 0.1f && stock <= n_gears()) {
        float r = wheel_radius(), f_res = resist(0);
        // Above the game's pick: only by physics (lock speed and slip time at this mass and grade), never by a gear count,
        // because every gearbox numbers its gears differently (Damien 13-10: "look at the gear ratio, not the number").
        bool lcx = g_lcx != 0;  // launch control (Creep): a higher start gear on the held revs, a longer slip
        bool raise = lcx || (p_launch_max > 0 && S.mode != MODE_POWER && S.mode != MODE_M);
        float limit = p_slip_s * (lcx ? 2.5f : thr >= p_kick_pedal && !raise ? 0.6f : 1.0f);
        float lrpm = lcx ? fminf(fmaxf(T.rpm_limit, 1900.0f) * 0.82f, 1700.0f) : p_launch_rpm;
        pick = 1;
        for (int g = raise ? n_gears() : stock; g >= 1; g--) {
            float ratio = T.ratio[g - 1] * T.diff;
            float v_lock = lrpm / (ratio * 9.5493f) * r;
            float a = (V.eng.torque * p_launch_torque * ratio * V.eta / r - f_res) / V.mass;
            if (g > stock && v_lock * 3.6f > (lcx ? 30.0f : p_launch_max)) continue;
            if (a > 0 && v_lock / a <= limit) { pick = g; break; }
        }
    }
    return pick;
}

static int stub_start(void *self)
{
    int stock = o_start(self);
    if (!g_active || stock <= 0) return stock;
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
    if (takeover() && V.eng_ok && v * 3.6f > p_off_kmh && wheel_radius() > 0.1f) {
        S.rolling = 0;
        S.roll_at = sim_s();
        // the demand filter lags a pedal that goes down hard right at the engage (25-10, 2397 s: full pedal at a junction, demand 0.02, gear 10
        // at 877 rpm on a 2.3% climb, then 10->8 two seconds later): the pedal itself counts
        int g = v * 3.6f <= p_on_kmh ? launch_pick(stock, clampf(T.in_thr, 0, 1))   // still at walking pace: a start gear by physics, not a gear for the speed
              : pick_for_speed(v, lerpf(cruise_floor(n_gears()) + 100, pw_mid(), smooth(p_power_from, p_power_full, fmaxf(S.demand, clampf(T.in_thr, 0, 1)))));
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
int ts_start_gear(float thr)
{
    int n = n_gears();
    if (n < 1 || !V.eng_ok || !(V.mass > 0) || !(wheel_radius() > 0.1f)) return 0;
    return launch_pick(T.trailer ? 1 : (n < 4 ? n : 4), clampf(thr, 0, 1), true);
}
float ts_idle_rpm() { return idle_rpm(); }
float ts_up_pt() { return S.up_pt; }
float ts_choice_floor() { return choice_floor_rpm(); }   // (the simulator reads the mode's upshift point)

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
// For the wish pedal (suite.h): the mode from the game's cvar, as the module reads it, so it is also right where our hooks are not in.
int ts_mode()
{
    if (S.unsupported || manual_box()) return -1;
    int m = mode_from_cvar(game_adaptive());
    return m == MODE_M ? -1 : m;
}
int ts_stop_hooked() { return g_stop_in; }
void ts_commanded(int gear) { S.commanded = gear; S.cmd_why = W_START_CORRECTED; }

// ---------------------------------------------------------------- module

static char g_status[200] = "off";
static const char *ts_status() { return g_status; }

static void ts_start()
{
    if (p_enabled <= 0) { logline("TrueShift disabled in the ini"); snprintf(g_status, sizeof g_status, "off (ini)"); return; }
    S.pk = -99;
    o_window = (window_fn)G.window_fn;
    o_choose = (gear_fn)G.chooser_fn;
    o_start = (gear_fn)G.start_fn;
    int done = game_patch_call(G.decide_site, (void *)stub_decide), want = 1 + G.n_choose + G.n_start;
    for (int i = 0; i < G.n_choose; i++) done += game_patch_call(G.choose_sites[i], (void *)stub_choose);
    for (int i = 0; i < G.n_start; i++) done += game_patch_call(G.start_sites[i], (void *)stub_start);
    g_active = done == want;
    if (g_active && (p_stop > 0 || low_owner_wanted())) {
        if (G.stop_site && game_patch_stop(G.stop_site, stop_threshold)) { g_stop_in = 1; logline("smooth stop hook in place"); }
        else { p_stop = 0; logline("smooth stop off: threshold code not found"); }
    }
    // The update's tail hook carries eco-roll AND the neutral watchdog (a shift interrupted by a pause leaves gear 0 with
    // the clutch open): both need the engage code, and the watchdog is wanted with eco_roll = 0 too.
    if (g_active) {
        int engage = game_engage_init(), tail = engage && G.tail_site && game_patch_tail(G.tail_site, (void *)tail_pre);
        if (!tail) {
            logline("%s: %s", p_roll > 0 ? "eco-roll and neutral watchdog off" : "neutral watchdog off",
                    engage ? "update tail not found" : "gearbox engage code not found");
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
                  "demand,up_pt,dn_pt,hold,kick,accel,rpm_k,clutch,mode,thr,f_need_kn,floor,ours,eb_level,f_bias_kn,strain,learn_rpm,roll,route,crest_m,junction_m,idle\n", g_trace);
    }
}

// Dashboard: the mode letter, also stopped and in N (Damien 11-10 prefers a
// steady letter over "A" flashing in at every stop); A in reverse. Runs outside
// the gearbox update, so the gearbox comes from game_actor, not a cached pointer.
// The mode letter is the lowest wish of the dashboard letter (core/display.cpp); the writer runs here, at this rate, and shows
// the modules' status (B hill-start brake, C crawl) and flashes (e.g. the new ACC distance for 1.5 s) over it.
static void dash_letter()
{
    static double at;
    double now = wall_s();
    if (p_letter <= 0 || !g_active || S.unsupported || manual_box()) { disp_post(DISP_TS, DISP_MODE, 0, 0); return; }
    if (now - at < 0.25) return;
    at = now;
    // Only the gearbox the hooks have seen (never a stale or menu object).
    void *gb = game_gearbox();
    if (!gb || gb != S.self) return;
    mode_update();  // 4x/s, also while stopped
    disp_post(DISP_TS, DISP_MODE, T.gear < 0 ? 'A' : mode_letter[S.mode], 0);  // reverse: plain A (Damien 11-10)
    char c;
    int r = disp_write(gb, &c);
    if (r != S.letter_state) {
        if (r > 0) logline("dashboard letter works (%c)", c);
        else logline("dashboard letter: auto_mode_name string not found");
        S.letter_state = r;
    }
}

// Incident watch (core/incident.cpp): the rpm limiter hit while TrueShift drives, or the engine dragged below idle while moving.
static void watch_engine()
{
    static double hi_at, lo_at;
    static int armed = 1;
    double now = sim_s();
    int driving = g_active && !S.unsupported && takeover() && S.mode != MODE_M && T.engine_on && !T.paused && T.gear > 0;
    float lim = V.eng.rpm_limit > 500 ? V.eng.rpm_limit : T.rpm_limit;
    if (driving && lim > 500 && T.rpm > 0.985f * lim && T.retarder == 0 && !T.motor_brake) {
        if (!hi_at) hi_at = now;
        if (armed && now - hi_at > 0.6) {
            armed = 0;
            incident("limiter", "rpm %.0f of %.0f for %.1f s in gear %d at %.0f km/h, throttle %.0f%%, grade %.1f%%, mode %s", T.rpm, lim, now - hi_at, T.gear,
                     fabsf(T.speed) * 3.6f, T.in_thr * 100, grade_pct(), mode_name[S.mode]);
        }
    } else {
        hi_at = 0;
        if (T.rpm < 0.9f * lim) armed = 1;
    }
    if (driving && fabsf(T.speed) * 3.6f > 8 && T.rpm > 100 && T.rpm < 0.9f * idle_rpm() && T.in_thr > 0.2f) {
        if (!lo_at) lo_at = now;
        if (now - lo_at > 0.8) {
            incident("lug", "rpm %.0f under idle %.0f for %.1f s in gear %d at %.0f km/h, throttle %.0f%%, grade %.1f%%", T.rpm, idle_rpm(), now - lo_at, T.gear,
                     fabsf(T.speed) * 3.6f, T.in_thr * 100, grade_pct());
            lo_at = now + 30;  // not again for a while
        }
    } else if (!lo_at || now > lo_at) lo_at = 0;
}

// M: the engine over its limit (the wheels push it: downhill, or a downshift that was too low). Like the Mercedes and
// Volvo manuals: a warning tone, no shift. Under power the governor keeps the rpm at the limit, so this stays quiet.
static void overspeed_tone()
{
    static double over_at, tone_at;
    static int said;
    double now = sim_s();
    if (now < tone_at) tone_at = 0;  // the sim clock started again (new session)
    float lim = V.eng.rpm_limit > 500 ? V.eng.rpm_limit : T.rpm_limit;
    int on = g_active && takeover() && S.mode == MODE_M && T.engine_on && !T.paused && T.gear > 0 && lim > 500;
    if (!on || T.rpm < 0.98f * lim) { over_at = 0; said = 0; return; }
    if (T.rpm < 1.03f * lim) return;
    if (!over_at) over_at = now;
    if (now - over_at < 0.5 || now - tone_at < 1.5) return;
    tone_at = now;
    play_cue(CUE_OVERSPEED);  // a warning, not feedback: also with tones 0
    if (!said) {
        said = 1;
        logline("M: engine over its limit, %.0f of %.0f rpm in gear %d at %.0f km/h, grade %.1f%%: warning tone", T.rpm, lim, T.gear,
                fabsf(T.speed) * 3.6f, grade_pct());
    }
}

static void ts_frame()
{
    g_rolling = S.rolling || S.roll_exit > 0;  // neutral by our choice, also until the exit gear is in
    g_roll_boost_kmh = S.boost_want && !S.rolling ? fminf(p_roll_boost * band_scale(T.cruise), overspeed_room(T.cruise)) : 0;
    {
        static const int to_inc[] = { 2, 1, 3, 4 };  // Normal, Eco, Power, M -> incident file numbering
        int on = g_active && !S.unsupported && !manual_box();
        g_inc.mode = on ? to_inc[S.mode] : 0;
        g_inc.target = on && S.commanded > 0 ? S.commanded : 0;
        g_inc.grade_pct = on ? grade_pct() : NAN;
        g_inc.flags = (g_inc.flags & ~(INC_ROLL | INC_HOLD)) | (g_rolling ? INC_ROLL : 0) | (S.hold_until > sim_s() ? INC_HOLD : 0);
        if (on) { watch_engine(); overspeed_tone(); }
    }
    DEV_NOTE(gearbox(g_active && !S.unsupported && !manual_box() ? mode_letter[S.mode] : 0, V.mass, grade_pct()));
    {
        static double at;
        if (wall_s() - at > 0.5 || wall_s() < at) { at = wall_s(); g_pcc_need_kmh = g_active && !S.unsupported ? climb_need_kmh() : NAN; }
    }
    dash_letter();
    // Idle rpm: standing, engine on for 5 s, pedal and throttle released, rpm
    // steady for 1 s. Starts from idle_rpm in the ini, never from one sample:
    // ride ts3b took the start-up flare (1294 rpm) as idle, and every low-speed
    // shift after it was a "stall" downshift.
    {
        static double on_at = -1, win_at;
        static float lo = 1e9f, hi = 0;
        double now = wall_s();
        if (!T.engine_on) on_at = -1;
        else if (on_at < 0) on_at = now;
        int ok = T.engine_on && now - on_at > 5 && fabsf(T.speed) < 0.3f && T.in_thr < 0.02f && T.eff_thr < 0.02f
                 && T.rpm > 300 && T.rpm < 1000;
        if (!ok) { lo = 1e9f; hi = 0; win_at = now; }
        else {
            lo = fminf(lo, T.rpm);
            hi = fmaxf(hi, T.rpm);
            if (now - win_at > 1) {
                if (hi - lo < 40) {
                    float r = 0.5f * (lo + hi);
                    if (S.idle < 300) S.idle = p_idle;
                    S.idle += 0.3f * (r - S.idle);
                }
                lo = 1e9f; hi = 0; win_at = now;
            }
        }
    }
    if (!g_trace || !S.self || T.paused) return;
    double now = wall_s();
    if (now - g_trace_at < 0.1) return;
    g_trace_at = now;
    double sim = sim_s();
    fprintf(g_trace, "%.2f,%.1f,%.0f,%d,%d,%d,%s,%.2f,%.2f,%.2f,%d,%u,%.0f,%.2f,%.2f,%.2f,%.0f,%.0f,%d,%d,%.2f,%.3f,%.2f,"
            "%c,%.2f,%.1f,%.0f,%d,%.2f,%.2f,%.2f,%.0f,%d,%d,%.0f,%.0f,%.0f\n",
            now, fabsf(T.speed) * 3.6f, T.rpm, T.gear, S.target, S.want, S.why ? S.why : "", T.in_thr,
            T.eff_thr, T.in_brake, T.motor_brake, T.retarder, T.cruise * 3.6f, grade_pct(), V.mass / 1000, S.demand,
            S.up_pt, S.dn_pt, S.hold_until > sim, S.kick_until > sim, T.accel, V.rpm_k, T.eff_clutch,
            mode_letter[S.mode], S.thr, S.f_need / 1000, S.floor_rpm, S.ours, S.eb_level, V.f_bias / 1000, S.strain, learned().rpm, S.rolling, route_ok(),
            route_crest_m(2000), route_junction_m(), idle_rpm());
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

module m_trueshift = { "trueshift", "TrueShift", g_params, sizeof g_params / sizeof g_params[0],
                       ts_start, ts_frame, ts_config, ts_stop, ts_reset, ts_status };
