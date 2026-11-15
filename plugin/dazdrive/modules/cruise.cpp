// Cruise: the cruise control management module (0.3 concept).
//
// Step 1 (built): the adaptive cruise gap as a time. The game's ACC keeps
// D(v) = max(4, g_acc * (0.25 + v/30)) meters (v in m/s, cruise-re notes),
// so g_acc is the gap at ~81 km/h and the gap in seconds shrinks at speed
// (17.5 -> 0.8 s at 80). This module sets g_acc so that D = time gap x speed.
// The options menu's three distances (20 / 30 / 40) stay the player's choice:
// short / middle / long time gap (idea Damien 11-10). We never write exactly
// 20, 30 or 40, so a round value means the player just picked a distance. The
// mode scales the gap (Eco a bit more room, Power a bit less). The player's
// own value is put back when the game closes.
//
// Step 2 (built, untested in game): our own smooth speed controller. The
// game's cruise throttle fn 0x140991c90 (one caller, cruise-re notes) is an
// incremental PI that keeps integrating while the clutch is open during a
// shift, so it comes out of every shift with a full-throttle burst. We still
// call it (limiter and its own state), then replace its output: PI on the
// target speed (set speed, or the ACC target when lower), integrator frozen
// while shifting, output rate-limited, gains and rates per mode, 0 while
// TrueShift rolls in neutral (eco-roll with cruise).
//
// Next: Stop&Go below the 30 km/h minimum.
#include "../core/suite.h"
#include "../core/game.h"
#include "../core/sense.h"
#include "../core/act.h"
#include "../core/route.h"
#include "../core/vehicle.h"
#include "../core/speedtarget.h"
#include "../core/vclass.h"
#include "../core/wish.h"
#include "../core/roll.h"      // the bends' allowance follows the learned lean
#include "../core/brake.h"
#include "../core/planner.h"   // the PL_* action classes the planner drive publishes in g_planner_drive
#include "../core/traffic.h"   // the blind-spot detector (overtake assist)
#include "../core/aeblat.h"    // the AEB's sideways reading of the radar target (is it in our path?)
#include "../core/speedlayer.h" // [cruise] engine = layers: the speed arbiter, the cruise state, CC and LIM (docs/cruise-layers.md)
#include "../core/req_legal.h"  // the road's requesters of the arbiter (WP-SP2): the limit line and the descent hold, the bends, the junctions, the PCC
#include "../core/req_bend.h"
#include "../core/req_junction.h"
#include "../core/req_pcc.h"
#include "../core/req_acc.h"    // WP-SP3: the lead model, the ACC as a moving target of the arbiter, Stop&Go and the overtake assist for the layers (docs/cruise-layers.md)
#include "../core/stopgo.h"
#include <math.h>
#include <string.h>

static float p_herm, p_bind_a;
static float p_os_room_max;
static float p_acc_us, p_acc_ft, p_sg_rearm, p_ah_park, p_jn_signal, p_jn_coast, p_jn_over, p_jn_hand_kmh, p_jn_hand_m, p_own_lim, p_own_bias, p_eco_cap, p_crest_heavy, p_os_cap, p_os_pct, p_acc_fast_from, p_acc_fast_full, p_acc_fast_k, p_acc_calm, p_limiter, p_acc_up, p_acc_letter, p_ess, p_ess_dec, p_ess_off, p_ess_kmh, p_curve_lat, p_curve_lat_ats, p_curve_lat_max, p_curve_lat_max_ats, p_bend_now, p_hill, p_hill_pct, p_hill_s, p_sg, p_sg_auto, p_descent, p_curve, p_curve_brake, p_min_set, p_isa, p_isa_off, p_isa_pct, p_isa_hold, p_predict, p_min_kmh, p_enabled, p_gap_s, p_gap_m, p_gap_l, p_min_m, p_max_m, p_smooth, p_step_tap, p_step_hold, p_hold_s, p_hold_rate, p_ret_off, p_res_lim, p_isa_ahead, p_isa_probe, p_junc, p_junc_lat, p_rb_kmh, p_junc_deg, p_rb_ratio, p_jn_dec, p_cv_coast, p_cv_brake, p_ah, p_ah_req, p_ah_lever, p_ah_gentle, p_ah_hard, p_ah_max, p_own_acc, p_own_brake, p_own_plan, p_own_gain, p_own_stop_m, p_own_dec, p_own_up, p_acc_eco_k, p_acc_pw_k, p_plan_to, p_acc_band, p_acc_cross, p_os_cruise, p_desc_kmh, p_band_up, p_band_s, p_trace, p_aeb, p_aeb_manual, p_aeb_warn_s, p_aeb_part_s, p_aeb_full_s, p_aeb_part_dec, p_aeb_max_dec, p_aeb_kmh, p_aeb_evade, p_aeb_mode, p_aeb_margin, p_aeb_warn_dec, p_aeb_full_frac, p_aeb_rise, p_aeb_jerk, p_overtake;
static float p_rb_curves;  // roundabout_curves
static float p_jn_margin, p_jn_cancel_s, p_approach_s;  // junctions: brake only to the junction speed + margin; cancel the cruise this many seconds before it
static float p_app_mode, p_app_sh_eco, p_app_sh, p_app_sh_pw;  // approach_by_mode and the share of the pull to spare each mode asks
static float p_force_mode;  // own cruise throttle: the controller asks a force at the wheels, not a throttle
static float p_engine;      // [cruise] engine: 0 = classic (the loop below), 1 = layers (docs/cruise-layers.md). Default classic
static float p_ly_tau_e, p_ly_tau, p_ly_tau_p, p_ly_sp_e, p_ly_sp, p_ly_sp_p, p_ly_rise_e, p_ly_rise, p_ly_rise_p, p_ly_trim_s, p_ly_trim_max;   // layers: the arbiter's comfort knobs
static float p_ly_coast, p_ly_bcomf, p_ly_bfirm, p_ly_bmax, p_ly_btol, p_ly_bk_e, p_ly_bk_p;
static float p_ly_rb;   // layers: the roundabout's entry speed follows the learned lean
static float p_lim_presses, p_lim_window, p_lim_tau, p_lim_kick0, p_lim_kick1, p_lim_show;   // layers: LIM and the toggle gesture
static float p_car_trail_lim;  // car_with_trailer: the cap a light vehicle with a trailer is held to (km/h)

// ---------------------------------------------------------------- the light-vehicle (car) tuning
// What differs for a car lives in its class profile (core/vclass.h, vprof()): ACC gaps and standstill gap, emergency-hazard thresholds, AEB
// times and decelerations, the gain scaling, no truck speed limiter (a car with a trailer keeps a lower cap, [cruise] car_trailer_limit_kmh), and
// hydraulic brakes (no air gating). A truck's profile says NAN where the ini decides, so the truck paths are untouched.
static param g_params[] = {
    { "enabled", &p_enabled, 1, "1 = cruise module on" },
    { "acc_short_s", &p_gap_s, 1.0f, "adaptive cruise gap in seconds when the game option is the short distance (20). Real trucks: 2.4-3.6 s (Detroit default 3.6), but the game is scaled: 1 s feels normal" },
    { "acc_middle_s", &p_gap_m, 1.25f, "... the middle distance (30)" },
    { "acc_long_s", &p_gap_l, 1.5f, "... the long distance (40)" },
    { "acc_us_rule", &p_acc_us, 0, "true = the ACC gap follows the rule of thumb for US trucks instead of the three times above: one second for every 10 feet of vehicle length below 40 mph, one second more above (a tractor and trailer, taken as 70 feet, gets 7 seconds, and 8 at highway speed; a truck alone, 25 feet, gets 2.5 and 3.5). The short and long distance steps are 0.8 and 1.2 times that. The game's radar sees a lead only up to about 100 meters, so the gap stops at 95 meters (max_gap_meters is raised to that while this is on): about 3.5 seconds at 100 km/h. false = the times above" },
    { "acc_length_ft", &p_acc_ft, 0, "with us_truck_rule_enabled: the vehicle length in feet (0 = 70 with a trailer attached, 25 without)" },
    { "crest_heavy_factor", &p_crest_heavy, 1.6f, "predictive cruise: the ease-off before a top with a descent after it is this many times larger at a gross weight of 40 t or more (linear from 20 t; Scania allows -8 %). 1 = same at every weight" },
    { "eco_cruise_throttle_cap", &p_eco_cap, 1.0f, "Eco with the cruise on: the cruise never asks for more than this throttle (0..1; 1 = no cap) while the speed is within 4 km/h of the target; further under it the cap is lifted (a climb, a resume)" },
    { "rollover_guard", &g_rsc_mode, 3, "rollover guard (a growing sideways swing, two opposite lateral peaks within 2.5 s, also when you drive by hand): 0 = off, 1 = a tone only, 2 = also no throttle, 3 = also a soft or firm brake" },
    { "rollover_bend", &g_rsc_bend, 0, "rollover guard also for one bend taken too fast without a swing (lateral 3.5 m/s2 = tone, 4.5 = no throttle, 6 = soft brake, 8.5 = firm): 0 = off, 1 = on. Off by default: the limits depend on the physics (mods), so they fire in ordinary bends on some trucks" },
    { "overspeed_room_max_kmh", &p_os_room_max, 0, "the overspeed number shown and written with the cruise off (the distance from the set speed to the fine line, limit plus overspeed_cap_kmh) is at most this (km/h; 0 = no clamp). With the cruise on overspeed_cruise_kmh applies" },
    { "overspeed_cap_kmh", &p_os_cap, 5, "the extra speed we grant over the set speed (hills, descents, coasting boost, smart overspeed) stops this far over the speed limit (km/h; 0 = no cap). Only while the limit is known" },
    { "overspeed_cap_pct", &p_os_pct, 10, "ATS only: the cap over the limit is this share of the limit instead (%; 10 = 66 at 60, 88 at 80, like the usual US tolerance). 0 = overspeed_cap_kmh in ATS too. ETS2 always uses overspeed_cap_kmh" },
    { "acc_fast_from_kmh", &p_acc_fast_from, 75, "ACC: from this speed (km/h) the time gap shrinks toward acc_fast_factor, so it follows closer on a motorway (country roads keep the full gap)" },
    { "acc_fast_full_kmh", &p_acc_fast_full, 95, "ACC: from this speed (km/h) the time gap is acc_fast_factor times the set gap; in between it changes smoothly" },
    { "acc_fast_factor", &p_acc_fast_k, 0.8f, "ACC: the time gap times this at motorway speed (1 = same gap at every speed). Never below 0.7 s" },
    { "acc_min_m", &p_min_m, 15, "the gap never gets shorter than this (meters)" },
    { "acc_max_m", &p_max_m, 90, "and never longer than this" },
    { "speed_limit", &p_isa, 1, "speed-limit following at the first start (before any on/off choice is remembered): 1 = on (at every new limit the cruise set speed becomes limit + offset; +/- still work until the next sign); after that your last on/off choice is remembered (cruise_state.txt)" },
    { "speed_limit_hold_s", &p_isa_hold, 0.8f, "hold the ACC distance key of the game this long (s) to switch speed-limit following on/off; the distance stays as it was. No button number to set" },
    { "speed_limit_offset_kmh", &p_isa_off, 3, "set speed = limit plus this (3: 80 -> 83; in miles per hour when the game shows mph, g_mph: 65 -> 68 mph)" },
    { "speed_limit_offset_pct", &p_isa_pct, 5, "ATS only: set speed = limit plus this share of it instead, in whole units (%; 5: 55 -> 58 mph, 65 -> 68, 80 -> 84; the cap over the limit is overspeed_cap_pct). 0 = speed_limit_offset_kmh in ATS too" },
    { "auto_hold", &p_ah, 1, "1 = Auto Hold: after stopping with the brake the truck stays held when you release the pedal (any slope, no timer) until you give gas; it lets go smoothly as the engine pulls. Not with crawl on, the parking brake, reverse or Stop&Go. With auto_hold on, hill_start is not used" },
    { "auto_hold_request", &p_ah_req, 0.6f, "how hard the hold brakes (brake request 0..1)" },
    { "auto_hold_lever", &p_ah_lever, 1, "1 = Auto Hold also holds after a stop with only the retarder or engine-brake lever on, or with the deceleration assist braking; it takes over from the assist's brake without a gap. 0 = only after a stop with the brake pedal" },
    { "auto_hold_gentle_s", &p_ah_gentle, 0.25f, "letting go after a gentle launch (light pedal) takes about this long (s)" },
    { "auto_hold_hard_s", &p_ah_hard, 0.15f, "after a hard launch (pedal 0.8+) about this long (s), still ramped" },
    { "auto_hold_park_s", &p_ah_park, 60, "Auto Hold: after standing held this long (s) the parking brake goes on and the service brake lets go (no air spent, like a real truck); gas or Resume releases it, with the hold taken over for the pull-away. The plugin checks that the game really shows the parking brake and switches this off for the session if not. 0 = off" },
    { "auto_hold_max_s", &p_ah_max, 2.5f, "the hold never fights the pedal longer than this (s); then it is released at the hard rate" },
    { "hill_start", &p_hill, 1, "1 = (only with auto_hold 0) after stopping on a slope the brake holds for a moment when you release it, until you give gas" },
    { "hill_start_pct", &p_hill_pct, 3, "from this grade (%)" },
    { "hill_start_s", &p_hill_s, 3, "at most this long (s)" },
    { "stop_go", &p_sg, 1, "1 = adaptive cruise keeps going in a jam: stays on at a standstill behind a vehicle and drives off with it" },
    { "stop_go_rearm_s", &p_sg_rearm, 60, "Stop&Go: when the cruise drops by itself at a standstill behind a vehicle, the first tap on the gas pedal within this many seconds (also Resume) puts it back on with the old set speed, and B stays on the dashboard until then. 0 = off" },
    { "stop_go_auto_s", &p_sg_auto, 0, "after standing longer than this (s), driving off needs resume or the pedal; 0 = always drives off by itself (real trucks, MAN/Detroit: 2)" },
    { "descent", &p_descent, 1, "1 = on a descent of 2 % or more (a navigation route lets it start earlier) brake gently ahead (engine brake / retarder) so the set speed holds instead of overshooting" },
    { "curves", &p_curve, 1, "1 = with navigation: slow down gently before sharp bends and exits" },
    { "acc_cycle_up", &p_acc_up, 1, "1 = the ACC distance key steps further away: off, close, middle, far, off (the game: far, middle, close, off)" },
    { "acc_calm_cut_in", &p_acc_calm, 1, "1 = a vehicle cutting in or pulling away (the gap opening) does not make the cruise slow down, it only stops speeding up" },
    { "acc_letter", &p_acc_letter, 0, "dashboard after an ACC distance change: 0 = 1 / 2 / 3 (close, medium, far; C = plain cruise, ACC off), 1 = A (the tones tell the distance)" },
    { "emergency_hazards", &p_ess, 1, "1 = hazards on by themselves when braking very hard at speed (like an emergency stop signal); off again when you drive on" },
    { "emergency_decel", &p_ess_dec, 4.0f, "deceleration (m/s2) that counts as an emergency stop" },
    { "emergency_off_decel", &p_ess_off, 2.5f, "below this deceleration (m/s2) before the truck stops, the hazards go off again (real trucks: 2.5)" },
    { "emergency_kmh", &p_ess_kmh, 50, "only above this speed (km/h)" },
    { "air_min_psi", &g_air_min_psi, 75, "below this brake air pressure (psi) the module stops its own brake requests (bends, crawl descent) and leaves descents to the engine brake ([core] air_from_truck: 10 psi over the truck's own warning pressure instead)" },
    { "curve_lat", &p_curve_lat, 3.125f, "sideways acceleration (m/s2) allowed in bends at about 92 km/h, the same in every mode (higher = faster through bends). Slower bends get more, up to 1.23x (4 m/s2), like the allowance in a Mercedes-Benz patent. With roll_learn 1 this is where a truck starts, and what it keeps until its lean has been measured" },
    { "curve_lat_ats", &p_curve_lat_ats, 3.4f, "ATS only: curve_lat for American Truck Simulator, where highway bends are tighter for their speed limits (a radius of 100-180 m is common) and the trucks hold more than a real one would. 3.4 m/s2 is about 0.35 g, the rollover level of a loaded real tractor-semitrailer; the game's trucks stay upright far beyond that. 0 = curve_lat in ATS too. ETS2 always uses curve_lat" },
    { "curve_lat_max", &p_curve_lat_max, 3.5f, "roll_learn: the most sideways acceleration (m/s2) the bends are planned with, however little the vehicle leans (a stiff, low truck). The comfort limit, ETS2" },
    { "curve_lat_max_ats", &p_curve_lat_max_ats, 4.0f, "roll_learn: the same for American Truck Simulator. 0 = curve_lat_max in ATS too" },
    { "roll_learn", &g_roll_learn, 1, "1 = the bends' sideways allowance follows how much the truck and its trailer lean (measured in the bends you drive: lean per m/s2, kept per truck and load in roll_gain.txt, the road's bank taken out): a tall or soft load gets less, a stiff low one more, never above curve_lat_max. Until a lean has been measured, and with 0, curve_lat is used as set. The game's physics settings and physics mods are taken into account, because the lean is measured, not assumed" },
    { "roll_edge_share", &g_roll_edge, 0.7f, "how close to the edge bends are driven, as a share of the sideways acceleration at which the vehicle's lean reaches the edge (roll_budget_deg). 0.7 = bends take 70 % of what would be the edge. Lower is calmer. The same in every drive mode" },
    { "roll_budget_deg", &g_roll_budget, 6.0f, "the lean (degrees) at which a vehicle is at the edge: about where the game's trucks start to lift a wheel and go over" },
    { "bend_now", &p_bend_now, 1, "1 = the speed is also capped by the bend the truck is IN (measured from its own yaw rate, no navigation needed): no acceleration above the speed curve_lat allows for that radius, and the usual gentle bend braking when over it. Motorway curves never trigger it (only radius under about 250 m). 0 = off, only the bends ahead on the route count" },
    { "route_hermite", &p_herm, 1, "1 = with navigation: the road between two route points is read as a curve through the directions of the points, so tight bends are seen earlier and sharper (0 = straight lines between the points, the older way)" },
    { "junctions", &p_junc, 1, "1 = with navigation: also slow down before junctions where the route turns, and before roundabouts (needs curves = 1; always on with own_acc = 1)" },
    { "junction_lat", &p_junc_lat, 2.5f, "sideways acceleration (m/s2) allowed in a junction turn (v = sqrt(this x turn radius), at least 15 km/h)" },
    { "junction_margin_kmh", &p_jn_margin, 10, "junctions and roundabouts: brake only down to the junction's speed plus this (km/h); the rest is yours. 0 = down to the junction speed itself" },
    { "junction_coast_decel", &p_jn_coast, 0.2f, "approaching a junction or roundabout the cruise may still pick up speed while the truck could coast down to the junction speed at this deceleration (m/s2; a loaded truck coasts at about 0.1 to 0.2); no gas above that speed" },
    { "junction_approach_over_kmh", &p_jn_over, 20, "and never faster than the junction speed plus junction_margin_kmh plus this (km/h) on the approach" },
    { "junction_handover_kmh", &p_jn_hand_kmh, 6, "the cruise is only cancelled for a junction once the speed is down to the junction speed plus the margin plus this (km/h) ..." },
    { "junction_handover_m", &p_jn_hand_m, 30, "... or when the junction is this close (m), whatever the speed" },
    { "speed_limiter_kmh", &p_own_lim, 0, "our own speed limiter (km/h, 0 = off): the throttle is held to what brings the speed onto the limit smoothly (from the vehicle model: a strong engine pulls fully until a few km/h before it, a climb at 92 keeps its full throttle, the speed settles on the limit); in every mode and with the pedal curves, and the cruise set speed never goes above it. Switch the game's own limiter off" },
    { "speed_limiter_bias_kmh", &p_own_bias, 0.5f, "adds this much to the limit in the throttle fade so the speedometer shows the configured value instead of a bit under it (km/h, 0 to 3; only with speed_limiter_kmh; the cruise set speed stops at the limit itself). Only where the vehicle model cannot tell (the clutch open, no engine data): elsewhere the limiter settles on the limit by itself" },
    { "junction_signal_deg", &p_jn_signal, 50, "a junction on the route that turns less than this (degrees, not a roundabout) is only slowed for when you use an indicator: the route is not where you are going, and a crossing you drive straight over needs no braking. 0 = always slow for it" },
    { "junction_cancel_s", &p_jn_cancel_s, 5, "this many seconds before a junction or roundabout we slowed for, the cruise is cancelled with a sound (the game's Resume brings the set speed back). 0 = never" },
    { "junction_min_deg", &p_junc_deg, 25, "turns smaller than this (degrees) are not slowed for" },
    { "roundabout_kmh", &p_rb_kmh, 25, "speed at the entry of a roundabout, also when going straight through" },
    { "roundabout_ratio", &p_rb_ratio, 1.4f, "a prefab counts as a roundabout when the path through it is this many times its entry-exit chord (lower = more prefabs count)" },
    { "roundabout_curves", &p_rb_curves, 2, "a prefab the route crosses in this many navigation curves or more counts as a roundabout, also straight through (2 = every roundabout seen so far; 3 = only those where the route uses the ring; 9 = off, the path ratio alone decides)" },
    { "descent_overspeed_kmh", &p_desc_kmh, 8, "on a descent (descent = 1) the engine brake / retarder hold the set speed plus this (km/h; 0 = exactly the set speed) (Written for a set speed of 70 km/h; it scales with the set speed.)" },
    { "overspeed_cruise_kmh", &p_os_cruise, 5, "cruise on: how far over the set speed the truck may run (downhill, coasting) before the game's engine brake / retarder hold it (the game's 'smart' cruise option, km/h); -1 = keep the game's setting. Never over the fine line (the limit plus overspeed_cap_kmh), never less than the predictive cruise (PCC) momentum asks for; a bend or junction ahead, a steep descent (descent_overspeed_kmh) and a heavy trailer lower it" },
    { "acc_ignore_crossing", &p_acc_cross, 1, "own ACC: a vehicle crossing in front (on a roundabout, at a junction) is passed, not followed: no braking or stopping for it, only no speeding up. Seen as a lead that drives (> 11 km/h) while the gap closes as if it stood still" },
    { "junction_plan_decel", &p_jn_dec, 1.15f, "junctions and roundabouts: the BRAKES are planned with this deceleration (m/s2); higher = later and firmer braking" },
    { "curve_brake_decel", &p_cv_brake, 1.05f, "bends (on/off-ramps): the brakes are planned with this deceleration (m/s2): only when it takes this much" },
    { "curve_coast_decel", &p_cv_coast, 0.4f, "bends and junctions: lifting off the throttle is planned with this gentle deceleration (m/s2), so the truck coasts first and brakes late" },
    { "curve_brake", &p_curve_brake, 1, "1 = and brake gently (engine brake / retarder first) when releasing the throttle is not enough" },
    { "predictive", &p_predict, 1, "1 = with navigation: carry a few km/h extra into a climb, ease off before a top with a descent after it (Eco most, Power half)" },
    { "band_up_kmh", &p_band_up, 6, "with predictive = 1 and navigation (predictive cruise, PCC): carry up to this much over the set speed into a climb, and in a dip (a descent with a climb right after it) let the truck run up to it before the engine brake / retarder hold it. At most this: only as much as the next climb needs (TrueShift: mass, engine, route; solo often 0); 0 = off (Written for a set speed of 70 km/h; it scales with the set speed.)" },
    { "band_max_s", &p_band_s, 40, "a dip may run over the set speed for at most this long, then the engine brake / retarder hold the set speed again (Mercedes PPC: 40 s)" },
    { "min_kmh", &p_min_kmh, 5, "lowest speed at which cruise control can be switched on or resumed (game: 30), e.g. in a jam" },
    { "limiter_kmh", &p_limiter, 0, "with the game's speed limiter on, the cruise set speed never goes above the game's limiter value (it knows the vehicle: a truck 90), 100 where the game gives none (a bus, EU law); a value here (km/h) caps lower still, 0 = follow the game" },
    { "car_trailer_limit_kmh", &p_car_trail_lim, 0, "a light vehicle (car mode) with a trailer: the set speed never goes above this (km/h). 0 = no cap (default: the weight is already in every choice; local trailer limits vary)" },
    { "min_set_kmh", &p_min_set, 20, "the set speed never goes below this: switched on slower, it sets this (with ACC it follows the vehicle ahead)" },
    { "step_tap_kmh", &p_step_tap, 5, "a short tap on cruise + / - changes the set speed by this (rounded to a multiple of it: 83 -> 85; in mph when the game shows mph, g_mph: 5 mph steps); 0 = the game's own step" },
    { "step_hold_kmh", &p_step_hold, 1, "holding + / - changes it in steps of this (not rounded to the tap step; mph when the game shows mph)" },
    { "hold_s", &p_hold_s, 0.5f, "how long + / - must be held before it counts as holding (s)" },
    { "hold_rate", &p_hold_rate, 2, "while holding: steps per second" },
    { "retarder_off_on_set", &p_ret_off, 1, "1 = pressing cruise on / set / resume while the retarder lever is on first switches the retarder off (the game refuses or cancels cruise otherwise)" },
    { "resume_to_limit", &p_res_lim, 1, "1 = pressing resume while cruise is already on sets the speed limit + speed_limit_offset_kmh (only while limit following is on; lower eases down, higher at once)" },
    { "speed_limit_ahead", &p_isa_ahead, 1, "not working yet: the route data carries no speed limits, so this does nothing; kept for a later build (meant to start easing early for a lower limit ahead on the route)" },
    { "speed_limit_probe", &p_isa_probe, 0, "not working yet, debugging only: the route data carries no speed limits; 1 = at every speed-limit change write the route nodes around the truck to the log (max 30 dumps). Leave at 0" },
    { "own_acc", &p_own_acc, 1, "1 = own ACC speed target from the lead vehicle's gap and speed (read from the game's radar), smooth; the game's emergency braking stays. 0 = the game's ACC target" },
    { "own_acc_gain", &p_own_gain, 0.3f, "own ACC: m/s of speed offset per meter of gap error (target = lead speed + gain x (gap - wanted gap))" },
    { "own_acc_stop_m", &p_own_stop_m, 0, "own ACC: gap kept at a standstill (m); 0 = follow the ACC distance step (close 3, middle 4, far 5, interpolated for any step count), a positive value = this gap instead" },
    { "own_acc_plan_decel", &p_own_plan, 0.3f, "own ACC: approaching a slower lead is planned with this deceleration of the speed difference (m/s2; 0.3 = about what lifting off gives: it lifts at the last moment and rolls in), so the game's emergency braking (which empties the brake air) is not needed; 0 = off" },
    { "own_acc_brake", &p_own_brake, 0.35f, "own ACC: brake request (0..1) it may use itself when lifting off is not enough, so the game's emergency braking is not needed; 0 = never" },
    { "own_acc_decel", &p_own_dec, 3.5f, "own ACC: how fast the target may fall (m/s2; faster, up to 8x, when closing in needs more than 0.5 m/s2)" },
    { "own_acc_accel", &p_own_up, 1.2f, "own ACC: how fast the target may rise (m/s2)" },
    { "own_acc_eco_k", &p_acc_eco_k, 0.75f, "own ACC in Eco: own_acc_accel and own_acc_plan_decel times this (under 1 = lifts off earlier and pulls up calmer behind a lead: saves fuel)" },
    { "own_acc_power_k", &p_acc_pw_k, 1.25f, "own ACC in Power and M: own_acc_accel and own_acc_plan_decel times this (over 1 = follows more directly)" },
    { "own_acc_plan_to", &p_plan_to, 1.0f, "own ACC: the approach to a slower lead is planned to end at this share of the wanted gap (the gap law then closes the rest)" },
    { "own_acc_band", &p_acc_band, 0.03f, "own ACC: uphill the gap may grow by this share of the wanted gap before the cruise pulls up for it (no chasing the lead up a climb in a low gear); it narrows to nothing when it gets tense (a braking lead, a short time to collision); 0 = off" },
    { "aeb", &p_aeb, 1, "1 = own emergency brake assist (AEB): warning tones, then partial braking, then full braking, all ramped; it replaces the game's ACC emergency braking and the game's AEB (both empty the brake air). 0 = the game's" },
    { "aeb_evade_steer", &p_aeb_evade, 0.01f, "AEB: the indicator to one side plus steering that way by this much (share of the full steering range, 0.01 = ~4.5 degrees on a 900-degree wheel; or the truck turning that way at 1.4 deg/s) for 0.3 s = an evasive lane change: a warning or partial braking stands down for 3 s. 0 = off" },
    { "aeb_without_cruise", &p_aeb_manual, 1, "1 = the AEB also watches when you drive yourself (cruise off); needs the game's radar, which runs with an ACC distance, blind spot or emergency brake switched on in the options" },
    { "aeb_warn_s", &p_aeb_warn_s, 3.0f, "AEB: warning tones when the time to collision drops below this (s), or earlier when more than ~1.5 m/s2 of braking would be needed (with cruise on: 2 s / 2.8 m/s2)" },
    { "aeb_partial_s", &p_aeb_part_s, 2.0f, "AEB: partial braking below this time to collision (s)" },
    { "aeb_full_s", &p_aeb_full_s, 1.2f, "AEB: full braking below this time to collision (s)" },
    { "aeb_partial_decel", &p_aeb_part_dec, 3.0f, "AEB: deceleration of the partial stage (m/s2; Mercedes ABA: ~3)" },
    { "aeb_max_decel", &p_aeb_max_dec, 6.5f, "AEB: the most it asks for in the full stage (m/s2)" },
    { "aeb_min_kmh", &p_aeb_kmh, 15, "AEB: only steps in above this speed (km/h); once braking it brakes to a stop. Slower is the ACC's job, except a stopped lead under 2 m ahead that the truck cannot stop short of comfortably (needs 0.6 x aeb_warn_decel or more)" },
    { "aeb_mode", &p_aeb_mode, 0, "AEB: 0 = stages by time to collision (aeb_warn_s ... aeb_full_s). 1 = by the deceleration it would take to stop short of the lead: leaves the driver in charge until late (a US-style system), then brakes firmly to stop about aeb_margin_m behind the lead" },
    { "aeb_margin_m", &p_aeb_margin, 0.5f, "AEB, mode 1: the gap it aims to stop at behind the lead (m)" },
    { "aeb_warn_decel", &p_aeb_warn_dec, 2.5f, "AEB, mode 1: warning tones when stopping behind the lead takes at least this deceleration (m/s2)" },
    { "aeb_rising", &p_aeb_rise, 1, "AEB, mode 1: 1 = full braking rises during the event instead of settling: it starts at the level the room to spare allows (softer when there is room, at once at full when the stop takes nearly all the brakes can do) and gets harder at aeb_rise_rate, so the truck is put down firmly just before the lead. Only with [core] brake_learn 1 and brakes measured at least as good as the built-in estimate (otherwise as with 0). Planned again every tick from the gap and the speed, the stop still lands aeb_margin_m behind the lead. 0 = the deceleration the stop takes now (falls off as the truck slows down)" },
    { "aeb_rise_rate", &p_aeb_jerk, 2.0f, "AEB, mode 1, aeb_rising: how fast the planned deceleration rises (m/s2 per second). Higher = a shorter, harder rise" },
    { "aeb_full_frac", &p_aeb_full_frac, 0.85f, "AEB, mode 1: full braking when stopping takes this share of aeb_max_decel; it then brakes with exactly the deceleration the stop takes (never above aeb_max_decel)" },
    { "trace", &p_trace, 1, "1 = write cruise.csv (10 per second while cruise is on or Auto Hold holds) for analysis" },
    { "approach_s", &p_approach_s, 2.5f, "own cruise throttle: the throttle eases off when the truck would reach the set speed within about this many seconds at the speed it is gaining now (also on a descent), so it arrives without overshooting. Eco 1.2x, Power 0.6x. 0 = off" },
    { "smooth", &p_smooth, 1, "1 = own smooth cruise throttle (no bursts after shifts; Eco gentle, Power keen), 0 = the game's" },
    { "ahead_accel", &p_bind_a, 0.088f, "a bend, junction or roundabout ahead that the truck will have to slow down for soon: the cruise does not chase the set speed hard up to it and then cut. It gains speed at about this acceleration (m/s2) at most, only up to the speed from which it can still coast down to the bend or junction (the grade of the road up to it counts), then follows the slowing plan. Lower = a calmer approach, 0 = off (the target is the speed the plan allows now, as before)" },
    { "approach_by_mode", &p_app_mode, 1, "own cruise throttle: 1 = the approach to the set speed (more than 2.5 km/h under it) asks only a share of the pull the truck has to spare at this speed (what a full wish could give over what holding the speed takes), the share the mode has (approach_share_eco, approach_share, approach_share_power): about the same pull in every mode (0.9 / 0.95 / 1.0), the modes differ in rpm and shifting. A truck that does not gain speed is given more, and a climb is held at whatever it takes. 0 = the throttle follows the deficit up to full, as before. Not with a vehicle ahead (ACC)" },
    { "approach_share_eco", &p_app_sh_eco, 0.9f, "approach_by_mode, Eco: the share (0-1) of the pull to spare that the approach to the set speed asks. Lower = a slower, softer approach. Default close to the other modes: they pull up about equally hard, the difference lives in rpm and shifting" },
    { "approach_share", &p_app_sh, 0.95f, "approach_by_mode, Normal: the share (0-1) of the pull to spare that the approach to the set speed asks. Lower = a slower, softer approach" },
    { "approach_share_power", &p_app_sh_pw, 1.0f, "approach_by_mode, Power: the share (0-1) of the pull to spare that the approach to the set speed asks. 1 = all of it" },
    { "force_mode", &p_force_mode, 1, "own cruise throttle: 1 = the controller asks a force at the wheels (a share of what the truck can give at this speed, the wish pedal's reference) and the throttle is what gives that force in the gear the truck is in, so a shift does not change the pull it asks; TrueShift's score-based gear choice reads that force as the demand. The throttle caps stay throttle caps. 0 = the controller asks a throttle, as before" },
    { "overtake_assist", &p_overtake, 1, "1 = overtake assist: with the own ACC following a slower vehicle, the left turn signal tells the cruise to prepare a lane change like a real overtake assist. It closes in a little (a shorter time gap, never under about 1 s) and pulls toward the set speed, only in Normal and Power, and only while the blind-spot data shows the left lane clear (needs [traffic] probe and blind_spot). It ends on the signal, a lane change, the brake or after about 8 s, and hands the gap back smoothly. 0 = off" },
    { "engine", &p_engine, 0, "which speed loop drives the cruise: 0 = classic (the cruise as it has been), 1 = layers (experimental: the cruise set through a speed arbiter, and the adjustable limiter LIM, switched on with three quick presses of SET; no bends, junctions, PCC or own ACC in it yet: with a vehicle ahead the classic loop drives). Needs smooth = 1. Not the same as enabled = 0 under [cruise.general] (that also switches off the torque pedal, Auto Hold, the AEB, the hazards and the limiter)" },
    { "layers_approach_eco_s", &p_ly_tau_e, 3.0f, "layers: the approach to the set speed takes about this long (s; the speed error over this is the acceleration asked) in Eco: the old approach_seconds 2.5 x its Eco factor 1.2" },
    { "layers_approach_s", &p_ly_tau, 2.5f, "layers: the same in Normal (the old approach_seconds)" },
    { "layers_approach_power_s", &p_ly_tau_p, 1.5f, "layers: the same in Power: the old approach_seconds x its Power factor 0.6" },
    { "layers_spare_eco", &p_ly_sp_e, 0.9f, "layers: the most of the pull the truck has to spare (full wish less what holding takes) an approach asks, Eco (the old approach_share_eco)" },
    { "layers_spare", &p_ly_sp, 0.95f, "layers: the same in Normal (the old approach_share)" },
    { "layers_spare_power", &p_ly_sp_p, 1.0f, "layers: the same in Power (the old approach_share_power)" },
    { "layers_rise_eco", &p_ly_rise_e, 0.4f, "layers: the force asked may rise this much of a full wish per second in Eco (it falls twice as fast); the old PI's rate" },
    { "layers_rise", &p_ly_rise, 0.6f, "layers: the same in Normal (the old PI's rate)" },
    { "layers_rise_power", &p_ly_rise_p, 1.0f, "layers: the same in Power (the old PI's rate)" },
    { "layers_trim_s", &p_ly_trim_s, 3.0f, "layers: the slow trim that takes the vehicle model's steady offset out has this integral time (s); the old ki / kp = 0.33 per second" },
    { "layers_trim_max", &p_ly_trim_max, 0.5f, "layers: the trim never exceeds this share of a full wish (the old PI's integrator limit; the model's error, such as a grade it does not know, is its job)" },
    { "layers_coast_decel", &p_ly_coast, 0.4f, "layers: lifting off for a speed ahead is planned with this deceleration (m/s2): how early the throttle goes (the old curve_coast_decel)" },
    { "layers_brake_comfort", &p_ly_bcomf, 1.05f, "layers: a request at a distance brakes only when it needs more than this (m/s2, Normal; the plan: coast first, then brake late); the cruise set's own brake never asks more (the old curve_brake_decel)" },
    { "layers_brake_firm", &p_ly_bfirm, 2.2f, "layers: the firmest a request of the legal and flow classes may brake (m/s2; the level the bend brake used)" },
    { "layers_brake_max", &p_ly_bmax, 3.5f, "layers: the firmest a protection may brake (m/s2; the bend brake's far-too-fast level)" },
    { "layers_brake_tolerance", &p_ly_btol, 0.15f, "layers: the brake asks only above what the truck slows by itself plus this (m/s2); between the channel's firm (0.1) and comfort (0.3) tolerances" },
    { "layers_brake_eco_k", &p_ly_bk_e, 0.75f, "layers: the comfort deceleration times this in Eco (the own ACC's eco factor)" },
    { "layers_brake_power_k", &p_ly_bk_p, 1.25f, "layers: the same in Power (the own ACC's power factor)" },
    { "layers_roundabout_from_lean", &p_ly_rb, 1, "layers: the roundabout's entry speed (roundabout_kmh) follows how far this truck at this load leans, as learned in your bends (roll_learn): a stiff, empty truck takes it a little faster, a heavy one that leans a lot a little softer, never outside about 22-30 km/h from 25; nothing learned yet = roundabout_kmh exactly. 0 = always roundabout_kmh" },
    { "lim_presses", &p_lim_presses, 3, "layers: this many quick presses of SET (the cruise on / off key) switch between the cruise (CC) and the adjustable limiter (LIM)" },
    { "lim_window_s", &p_lim_window, 1.5f, "layers: ... within this long, first to last press (s)" },
    { "lim_tau_s", &p_lim_tau, 0.75f, "layers, LIM: the ceiling closes the gap to the limit in this long (s): the own limiter's g_lim_tau, tuned in the sims" },
    { "lim_kickdown_from", &p_lim_kick0, 0.92f, "layers, LIM: from this pedal a kickdown starts to pass the ceiling (0.92 = TrueShift's kickdown_pedal)" },
    { "lim_kickdown_full", &p_lim_kick1, 1.0f, "layers, LIM: from this pedal it passes it fully (1.0 = the floor); in between a smooth blend, never a switch" },
    { "lim_show_set", &p_lim_show, 0, "layers, LIM: 1 = keep the game's cruise on with the limit as its set speed, so the HUD shows it (the live experiment; it makes the torque pedal, TrueShift, Creep and Decel see a cruise: 0 keeps the game's cruise off in LIM and the drivetrain drives as without one)" },
};
// [cruise] engine = layers and the throttle is ours (smooth): the speed arbiter drives the CC, the cruise state decides LIM and the toggle
static int layers_on() { return p_engine > 0 && p_smooth > 0; }
// Junction / roundabout slowing: always with the own ACC (friend 11-10: own ACC without it entered roundabouts at
// ~40 km/h and stopped there 4 times, probably for traffic on the roundabout; Damien: build it into the ACC)
static int junc_on() { return p_curve > 0 && (p_junc > 0 || p_own_acc > 0); }


// ---------------------------------------------------------------- smooth controller
// The module works inside the actuator's tick (core/actuate.cpp, the game's cruise fn call): cr_pre before the game's own
// controller, cr_post after it (our controller, Auto Hold, the AEB, the pedal: posted as wishes), cr_done for the trace.

static double s_cb_on_since, s_jn_cancel_at;  // cruise_body: when the cruise came on, when we last cancelled it for a junction
static int s_cb_was_on;
static struct { float i, u; int on; } C;
// force_mode 1: the controller's output (C.u, C.i) is a share of the force a full wish asks at this speed (wish_full). fm = the output
// is in that unit now, k = the throttle per unit of it in the gear of now, u = the output and at = the time of the last tick it gave the
// throttle (cruise_wish_force; -1 = none)
static struct { int fm; float k, u, to; double at; } CF = { 0, 1, 0, 0, -1 };

// State that cr_reset() must reach (was function-local).
static int s_hs_armed;                                  // hill_start: stopped with the brake pressed on a slope
static double s_hs_until;                               // hill_start: hold until
static float s_lead_prev = -1, s_gap_rate;              // lead_rel_speed: gap filter
// The sideways acceleration the bends are planned with (at about 92 km/h): curve_lat, in ATS curve_lat_ats when it is set
// With roll_learn the value moves with the lean measured for this truck and load (core/roll.h): curve_lat is where it starts, curve_lat_max its ceiling
static float bend_lat()
{
    bool ats = sense_is_ats();
    float start = p_curve_lat_ats > 0 && ats ? p_curve_lat_ats : p_curve_lat;
    float cap = ats && p_curve_lat_max_ats > 0 ? p_curve_lat_max_ats : p_curve_lat_max;
    return roll_bend_lat(start, cap);
}
static float s_bend_coast_kmh = NAN;   // the route's bend speed (coast plan) this frame, for the trace (Damien 11-10: a sharp bend at full set speed)
static float s_bn_f, s_bn_pk; static int s_bn_init, s_bn_on, s_bn_cap; static double s_bn_said, s_bn_cap_at;   // bend_now: filtered path curvature (1/m), its held peak, state, last log
static int s_curve_braking;                             // bend braking hysteresis
static int s_jn_braking;                                // junction braking hysteresis
static int s_isa_last = -1;                             // the limit (km/h) we last set the cruise to
static double s_btn_down_at, s_btn_up_at;               // speed-limit button hold
static int s_btn_fired, s_acc_armed, s_acc_key_bad;
static float s_hold_before = NAN;  // the ACC distance when the hold of its key began (a held key may step it again)
static float s_ahead_ours = NAN;                        // limit ahead: the set speed we wrote
static int s_ahead_ignore = -1, s_ahead_logged = -1;
static int s_down_now, s_dip, s_dip_spent;              // descent hysteresis; in a dip (climb after it); its time is up
static double s_dip_at;                                 // when the dip margin started
static double s_flat_since, s_desc_at;
static int s_haz_ours;                                  // emergency hazards we switched on
static double s_haz_hard_at, s_haz_go_at;
static int s_haz_stopped; static double s_haz_soft_at;  // emergency_hazards: the stop and the ease-off timer (reset in cr_reset, review 30-10)
static float s_haz_peak_kmh; static double s_haz_peak_at;
static double s_gacc_at;                                // g_acc poll clock
static float s_gacc_seen = NAN;
static double s_steer_until;                            // cruise_body: no gas while the wheel is turned, until
static double s_jn_ignore_until;                        // cruise_body: junctions left alone after the gas pedal, until
static float s_acc_f; static int s_acc_init;            // cruise_body: filtered acceleration (km/h/s) for the approach look-ahead
static int s_desc_braking;                              // cruise_body: descent service-brake hysteresis
static double s_stuck_since;                            // cruise_body: full throttle without moving since (0 = not)
static float s_cr_prev = NAN, s_cr_rate = NAN; static double s_cr_since = -1, s_cr_said;   // own_acc: crossing-traffic gap filter

// Fields of game_actor (per-layout, cruise-re notes § 4): read in core/sense.h (sense_cruise_on, sense_cruise_set, sense_acc_target, sense_lead_m), written in core/act.h.
// The gap to the game's lead (m; <= 0.5 = none). ETS2: the game_actor field. ATS 1.61: that field is something else (it reads ~1.0 m
// with no vehicle ahead: a phantom lead at 0.6-1 m braked a truck from 65 to 15 mph, 01-11), so the gap comes from the radar call the
// ACC makes (the lead speed hook), the same source the AEB and the dashboard use.
static float lead_gap_m(char *g);

// Stop&Go (cruise-re notes § 4): at a standstill the automatic goes to
// neutral (it is fed the driver's pedals, not the cruise throttle) and that
// cancels cruise. While we hold a stop, two cancel stores are skipped
// (neutral idle, auto box not in drive), and to drive off the automatic gets
// our cruise throttle as its pedal (g_feed), so it engages and goes.
static int g_low_ok;  // the minimum-speed patch is in (1.60 layout: game_actor cruise fields valid)
typedef float (*cvar_read_fn)(void *cvar);
static cvar_read_fn o_acc_read;
static float acc_read_stub(void *cvar);
static unsigned char g_sg_hold;
static float g_feed;
static int g_sg_ok;
static double g_stopped_at;

// Grade (%) from the SDK placement's pitch (the vehicle model computes it; TrueShift decides on the game's own gearbox grade, V.grade_pct)
static float pitch_grade() { return V.pitch_pct; }
// The band over the set speed (km/h): at most band_up_kmh (Damien 11-10: above may always, +6), but only as much as the
// next climb needs (TrueShift works it out from mass, engine and route: solo often 0, loaded more). Without TrueShift's
// answer: scaled by mass as before (full at 40 t, 40 % at 12 t).
static float s_mom_up;   // km/h over the set speed the PCC momentum aims for now (the game's overspeed braking must allow it)
static int s_lead_near;  // a vehicle within ~4 s ahead (cruise_body): no PCC momentum, it would only be braked away
// gross weight (kg) from the vehicle model; its fallback until the game tells (no extra ease-off before a top, the mass-scaled
// PCC band small, ACC and AEB plan as for 30 t and less); the descent allowance (smart_dynamic) treats unknown as heavy
static float load_mass() { return V.mass_ok ? V.mass : VEHICLE_MASS_FALLBACK; }
// The own ACC's brake law asks 1 / ACC_GAIN_SHARE (1.43) times the deceleration the kinematics need: the brake's lag and the lead's braking still to come
// are not in them. It was written as a divisor of 7 m/s2 per request where the fit says 10 (7 / 10): the law is tuned on that over-ask, so it stays, but the
// 10 now comes from the vehicle model (the cautious gain), where it was a constant of its own.
static const float ACC_GAIN_SHARE = 0.7f;
// 1 in Normal, eco_k in Eco, power_k in Power (the adaptive mode of the game)
// The cruise's reading of the mode: M drives like Power (Damien 30-10: "Manual, the same as Power")
static int cruise_mode() { int md = sense_mode(); return md == MODE_M ? MODE_POWER : md; }
static float mode_k(float eco_k, float power_k)
{
    int md = cruise_mode();
    return md == MODE_ECO ? eco_k : md == MODE_POWER ? power_k : 1.0f;
}
// the ease-off before a top grows with the weight (Damien 25-10: a heavy load may drop a bit more)
static float crest_mass_k()
{
    float f = fminf(1.0f, fmaxf(0.0f, (load_mass() - 20000) / 20000));
    return 1 + (fmaxf(p_crest_heavy, 1.0f) - 1) * f;
}
static float band_up()
{
    if (!(p_predict > 0 && p_band_up > 0) || s_lead_near) return 0;
    float need = g_pcc_need_kmh;
    float k = band_scale(T.cruise), room = overspeed_room(T.cruise);
    float pk = mode_k(1.0f, 0.5f);  // Power: half the carry into a climb
    if (need == need) return fminf(fminf(p_band_up * k, need) * pk, room);
    float m = load_mass();
    return fminf(p_band_up * k * pk * (0.4f + 0.6f * fminf(1.0f, fmaxf(0.0f, (m - 12000) / 28000))), room);
}

// Hill-start: stopped on a slope, the driver lets go of the brake: hold it
// through the game's brake request (game_actor+0x17c; the game slews to it
// and zeroes the throttle while it is > 0) until the pedal, a timeout or
// moving. Runs every tick (the cruise fn is called with cruise off too).
// The wishes below (s_want_*) are posted to the actuator (core/actuate.cpp) once per tick in the stub; that is the one
// writer of the brake request, with the slew, the air guard and the hazard mute.
static int s_sg_wait;  // Stop&Go: standing longer than stop_go_auto_s, the driver has to press Resume or the pedal: the dash shows B
static float s_want_hill, s_want_hold, s_want_curve, s_want_acc, s_want_aeb;
// [brake] channel 1 (core/brake.cpp): the same decisions ask a deceleration (m/s2: the total, the own ACC's on top) instead of posting a request. Reset every
// tick like s_want_curve; asked in cr_post. All 0 with the channel off (then nothing below changes what is posted).
static float s_ch[BRQ_N]; static int s_chu[BRQ_N];
static int s_lim_braking;   // channel: the line at limit + cap holds until 1 km/h under it (the ask fades to 0 there)
static float cr_game_brake_from(char *g, float pedal);   // where the game's own overspeed braking starts now (below, with the cvars)
static float cr_game_target(char *g);   // the speed the game's cruise holds now: the ACC target with a lead, else the set speed
static void cr_view_update(char *g, float pedal);
static float s_view_from = NAN, s_view_target = NAN;   // the two, as of this tick's cr_post (cruise_overspeed_view)
static float s_target_dbg;   // the cruise target of the last tick (km/h), for the sims
static int s_layers_ran;        // layers: the last tick of the CC was the arbiter's (the classic loop takes over while a vehicle is followed that the layers cannot follow)
// WP-SP3: the vehicle ahead in the layers. ONE lead model (core/leadmodel.h), updated in cr_pre with engine = layers and own_acc on; the classic ACC (own_acc) and the AEB keep their own reads
static lm_state g_lm; static lm_out g_lmo;   // the lead model and its output of this tick
static ra_state g_ra; static ra_out g_rao;   // the ACC requester's state and what it posted
static ot_state g_ot;                        // the overtake assist (core/req_overtake.h)
static sg_state g_sgs;                       // Stop&Go's go latch (the stopped-since time is g_stopped_at, as it has been)
static int s_acc_lay;                        // this tick the vehicle ahead is followed by the layers' ACC (cr_pre decides; cruise_body and cr_layers_body read it)
static float s_lv_prev = NAN, s_a_lead;                 // own ACC: lead speed history (reset without a lead)

// Planner drive ([planner] drive=1): the cruise follows the plan's target. s_pl_tgt = the target the cruise last drove (km/h), for a
// smooth hand-back to the blocks; s_pl_was = the cruise followed the plan last tick. planner_follow() = a fresh, ok plan to follow now.
static float s_pl_tgt;
static int s_pl_was;
static int planner_follow(planner_drive *pd) { *pd = g_planner_drive; return pd->on && pd->fresh; }

// Standing, for Auto Hold's arming and its parking brake: with the low-speed owner the owner's one definition (< 0.3 km/h, measurement
// noise), else Auto Hold's own.
static int ah_stopped(float kmh) { return low_owner_on() ? low_stopped() : kmh < 0.5f; }

static void hill_start(char *g)
{
    double now = wall_s();
    float kmh = fabsf(T.speed) * 3.6f, gr = fabsf(pitch_grade());
    int on_slope = gr >= p_hill_pct;
    s_want_hill = 0;
    if (p_hill <= 0 || !g_low_ok) return;
    if (kmh < 0.5f && T.in_brake > 0.1f && on_slope && !T.parking_brake) s_hs_armed = 1;
    if (s_hs_armed && T.in_brake < 0.05f && !s_hs_until) s_hs_until = now + p_hill_s;
    int release = T.in_thr > 0.05f || kmh > 2 || T.parking_brake || !on_slope || (s_hs_until && now > s_hs_until);
    if (s_hs_until && release) { s_hs_until = 0; s_hs_armed = 0; disp_post(DISP_CRUISE, DISP_STATUS, 0, 0); return; }
    if (s_hs_until) s_want_hill = 0.6f;
    // B on the dashboard while the hill-start brake holds
    disp_post(DISP_CRUISE, DISP_STATUS, s_hs_until ? 'B' : 0, 0);
}


// ---------------------------------------------------------------- Auto Hold (autohold-acc notes)
// After a stop with the brake (speed < 0.5 km/h, pedal > 0.1) the truck stays held through the same brake
// request as hill start (game_actor+0x17c, one writer: the actuator, core/actuate.cpp), at any grade and without a timer,
// until the driver gives gas. Real cars (Damien: VW Crafter, Lynk & Co 01) struggle to let go when you pull
// away hard: the hold drops at a fixed moment and the car lurches or rolls. Here the release follows the drive:
//  - the game zeroes the merged throttle while 0x17c > 0 (neutral-exit-re notes § 3.1); a patch of that
//    one compare (the actuator's throttle gate) lets the driver's gas through while only we hold, so the engine builds torque
//    against the brake and the box engages as usual;
//  - the request falls with the drive force: r = hold x (1 - F_drive / F_needed), F_needed = the grade force plus
//    what it takes to start moving; never faster than the pedal allows (gentle ~1.2 s, hard ~0.25 s: still a ramp)
//    and never up again, except when the truck starts to roll back: then it grabs again;
//  - moving off (speed > 0.3 km/h): target 0 at the pedal's own pace; after 2.5 s of launch: the hard rate at least.
// With auto_hold_lever a stop with only the retarder / engine-brake lever on, or with the deceleration assist (modules/decel.cpp)
// braking, arms it the same way. The assist keeps its request at the standstill until the hold has risen to it (s_ah_take, read
// through cruise_info_get): the actuator merges by the biggest wish, so the hand-over has no gap.
// Without the patch (pattern not found) the throttle is dead while we hold, so the request just ramps with
// the pedal (the old hill-start behavior, but ramped).
enum { AH_OFF, AH_HOLD, AH_LAUNCH };
static struct { int st, released; float r; double launch_t, grab_until, roll_m; } AH;
static int s_gate_ok;
static float s_ah_take;   // the request Auto Hold would hold after a stop by the lever now (0 = it would not take one)
static float s_ah_k = 1;  // learned: how much of the estimated drive force really arrives (the clutch and the engine model are rough)

// Force (N) at the wheels the engine and clutch deliver right now; NAN if the data is not there. The engine at its
// real rpm with the clutch slipping, so not the model's force_at (that is a closed clutch at the driveline rpm); the
// driveline efficiency is the model's (V.eta, [trueshift] efficiency).
static float ah_drive_force()
{
    int g = T.gear;
    float r = wheel_radius();
    if (!V.eng_ok || g < 1 || g > 31 || !(r > 0.1f) || !(T.diff > 0) || !(T.ratio[g - 1] > 0)) return NAN;
    float couple = 1 - clutch_seen();
    couple = couple < 0 ? 0 : couple > 1 ? 1 : couple;
    float te = V.eng.torque * curve_factor(&V.eng, T.rpm) * T.eff_thr;
    return s_ah_k * te * T.ratio[g - 1] * T.diff * V.eta * couple / r;
}

// Force (N) that gets the truck going: the model's whole resistance at a standstill on the pitch grade (grade, rolling, the
// learned extra; holding back alone would be less, but the release aims at moving off) with 10 % margin, plus a small push (0.25 m/s2).
static float ah_need()
{
    float m = V.mass;
    float need = 1.1f * vehicle_resist_grade(0, pitch_grade()) + m * 0.25f;
    return need < m * 0.12f ? m * 0.12f : need;
}

// Parking brake after a long Auto Hold (Damien 25-10): the game_actor's parking brake flag and value (cruise-re.md, the cancel hub, inferred)
// are written and the SDK's parking_brake channel has to confirm it within 0.8 s, else everything is put back and it stays off this session.
static double s_res_press_at = -10;      // last resume press (on or off): cruise coming on by resume follows ISA, by set not
static double s_rearm_at; static float s_rearm_set; static int s_rearm_active;
static float s_dstar_sm = NAN;   // own ACC: the slewed wanted gap (reset whenever following stops)
static double s_ah_hold_since, s_park_at; static int s_park_on, s_park_ok, s_park_bad; static unsigned char s_park_sv; static float s_park_svf;
int ah_park_owned() { return s_park_on; }
int ah_holding() { return AH.st == AH_HOLD; }
static void park_release(char *g, const char *why)
{
    if (!s_park_on) return;
    act_park_flag(g, s_park_ok ? 0 : s_park_sv);
    act_park_val(g, s_park_ok ? 0.0f : s_park_svf);
    logv(1, "Auto Hold: parking brake released (%s)", why);
    s_park_on = s_park_ok = 0;
}
static void auto_hold(char *g, float dt)
{
    s_want_hill = 0;
    s_want_hold = 0;
    float h0 = p_ah_req < 0.1f ? 0.1f : p_ah_req > 1 ? 1 : p_ah_req;
    // [core] brake_learn: the hold sized from the learned brake gain for the grade (twice the slope's pull plus 0.8 m/s2, 0.2-1), in place of the ini's
    // request, once the gain is confident; a roll back still grabs with at least the ini's request (h0). Not confident: the ini's.
    float h = vehicle_brake_hold_req(pitch_grade(), h0);
    double now = wall_s();
    if (!(dt > 0 && dt < 0.5f)) { s_want_hold = AH.r; return; }  // paused / hitch: keep what is set
    float kmh = fabsf(T.speed) * 3.6f;
    int blocked = !g_low_ok || (!T.engine_on && !cab_ss_engine_off()) || (T.parking_brake && !s_park_on) || T.gear < 0 || T.dgear < 0 || g_lv.crawl_toggle || g_sg_hold;
    if (blocked) {                         // crawl owns the standstill, parking brake / reverse / Stop&Go: hand over quickly
        if (AH.st != AH_OFF) logv(1, "Auto Hold: off (%s)", !T.engine_on ? "engine off" : T.parking_brake ? "parking brake" : T.gear < 0 || T.dgear < 0 ? "reverse" : g_lv.crawl_toggle ? "crawl on" : "Stop&Go holds");
        AH.st = AH_OFF;
    }
    s_ah_take = p_ah_lever > 0 && !blocked ? h : 0;
    float grade = pitch_grade();
    if (AH.st == AH_HOLD) { if (!s_ah_hold_since) s_ah_hold_since = now; } else s_ah_hold_since = 0;
    if (s_park_on) {
        if (!s_park_ok && now - s_park_at > 0.8) {
            if (T.parking_brake) { s_park_ok = 1; logv(1, "Auto Hold: parking brake on (the game confirms it)"); }
            else { park_release(g, "the game does not show it: off for this session"); s_park_bad = 1; }
        }
        // A start-stop engine off is not a handover to the auto parking brake, and not a reason to drop the parking brake: our
        // start-stop will start the engine again on gas (or a real need) and only then is the brake released on drive-off. The
        // driver's own engine off still hands over (cab_ss_engine_off() is false then).
        if (s_park_on && s_park_ok && !T.engine_on && !cab_ss_engine_off() && cab_park_auto_on()) {
            logline("Auto Hold: engine off, parking brake stays on");
            s_park_on = s_park_ok = 0;
        } else if (s_park_on && (T.in_thr > 0.05f || now - s_res_press_at < 0.4 || kmh > 1.0f || AH.st != AH_HOLD || T.gear < 0 || T.dgear < 0 || (!T.engine_on && !cab_ss_engine_off()))) {
            park_release(g, T.in_thr > 0.05f ? "gas" : now - s_res_press_at < 0.4 ? "resume" : "no longer held");
            if (T.in_thr > 0.05f || now - s_res_press_at < 0.4) { AH.st = AH_LAUNCH; AH.launch_t = 0; AH.grab_until = 0; AH.r = h; AH.released = 0; }
        }
    }
    switch (AH.st) {
    case AH_OFF:
        AH.r = AH.r > 0 ? fmaxf(0, AH.r - 3 * dt) : 0;   // fade out, ~0.2 s
        // (or by the lever: the retarder / engine brake on, or the assist's brake still posted; at 0.5 km/h, not at the low-speed
        // owner's standstill: no pedal holds the truck meanwhile, and on a climb the assist has let go before it stands;
        // not behind the stop assist that lets go at the standstill, [decel] stop_assist 1)
        if (!blocked && ((ah_stopped(kmh) && T.in_brake > 0.1f) ||
                         (p_ah_lever > 0 && kmh < 0.5f && (T.retarder > 0 || T.motor_brake || (act_wishes()[ACT_DECEL].brake > 0 && !g_sa_nohold))))) {
            AH.st = AH_HOLD; AH.roll_m = 0;
            logv(2, "Auto Hold: holding at %.1f%% (mass %.0f kg, request %.2f)", grade, V.mass, h);
        }
        break;
    case AH_HOLD:
        // on within ~0.2 s, while the driver still has the pedal; once the parking brake is confirmed the service brake lets go slowly
        // (it was decayed above and then pulled back to h here every frame, so both brakes stayed on)
        AH.r = s_park_on && s_park_ok ? fmaxf(0.0f, AH.r - 0.5f * dt) : fminf(h, AH.r + 3 * dt);
        if (T.in_thr > 0.05f) { AH.st = AH_LAUNCH; AH.launch_t = 0; AH.grab_until = 0; logv(2, "Auto Hold: launch, pedal %.2f, grade %.1f%%, hold %.2f", T.in_thr, grade, AH.r); }
        else if (kmh > 1.5f) { AH.st = AH_OFF; logv(2, "Auto Hold: truck moves without gas (%.1f km/h), off", kmh); }
        else if (p_ah_park > 0 && !s_park_on && !s_park_bad && g_low_ok && now - s_ah_hold_since > p_ah_park && ah_stopped(kmh) && T.in_thr < 0.03f && !T.parking_brake) {
            s_park_sv = sense_park_flag(g); s_park_svf = sense_park_val(g);
            act_park_flag(g, 1); act_park_val(g, 1.0f);
            s_park_on = 1; s_park_ok = 0; s_park_at = now;
            logv(1, "Auto Hold: held %.0f s, parking brake requested", p_ah_park);
        }
        break;
    case AH_LAUNCH: {
        AH.launch_t += dt;
        if (T.in_thr < 0.03f && kmh < 1.0f && T.speed > -0.05f) { AH.st = AH_HOLD; logv(2, "Auto Hold: pedal released again, holding"); break; }
        if (T.in_brake > 0.1f && kmh < 1.0f) { AH.st = AH_HOLD; break; }
        if (T.in_brake > 0.1f) { AH.st = AH_OFF; break; }
        float need = ah_need(), drive = s_gate_ok && V.mass_ok ? ah_drive_force() : NAN;
        // [lowspeed] owner=1 under the smooth-start law: the clutch is the owner's command, so the drive is the force that command carries (the
        // engine's torque and the clutch's capacity, lowspeed.cpp), not an estimate from the clutch the game shows; the two timers below that wait
        // for a clutch that never closes against the brake are not needed then (the law closes it whatever the brake does; the 2.5 s limit stays)
        float lf = low_drive_force();
        int owner_f = lf == lf;
        if (owner_f) drive = s_gate_ok && V.mass_ok ? s_ah_k * lf : NAN;
        float target = drive == drive ? h * fmaxf(0, 1 - drive / need) : 0;   // no drive data / no throttle gate / mass unknown: only the pedal rate paces it
        int moving = T.speed > 0.3f / 3.6f;
        if (moving || AH.launch_t > p_ah_max) target = 0;
        // Flat or downhill: nothing to hold against, let go at the pedal's pace (ts4n: on 0 % it held ~2.2 s because
        // the drive estimate stays low while the clutch is still open and the engine at idle; Damien: "too slow").
        if (grade < 1.5f) target = 0;
        // Uphill but the clutch never bites while we hold (engine kept at idle): do not wait for it longer than 0.6 s.
        if (!owner_f && AH.launch_t > 0.6 && clutch_seen() > 0.9f) target = fminf(target, h * 0.5f);
        // ... and with the pedal (almost) flat for 1.2 s and the clutch still open the hold is what keeps the clutch from closing (the
        // automatic does not close it against the service brake): let go (Damien 12-10: 4 % up, gear 2, pedal 1.0, clutch 0.97 for 2 s)
        if (!owner_f && AH.launch_t > 1.2 && clutch_seen() > 0.9f && T.in_thr > 0.7f) target = 0;
        float ped = fminf(1, fmaxf(0, (T.in_thr - 0.25f) / 0.55f));          // 0 gentle .. 1 hard
        float t_rel = p_ah_gentle + (p_ah_hard - p_ah_gentle) * ped;          // seconds for the full hold
        float rate = h / fmaxf(t_rel, 0.05f);
        if (AH.launch_t > p_ah_max) rate = fmaxf(rate * 2, h / p_ah_hard);      // never fight the pedal for long (moving off keeps the pedal's pace: a quick drop is a lurch)
        if (T.speed < -0.08f && grade > 0.5f) {                               // rolling back uphill: grab again, trust the estimate less
            if (now > AH.grab_until) { s_ah_k = fmaxf(0.3f, s_ah_k * 0.7f); logv(2, "Auto Hold: rolling back (%.2f km/h): grabbing again, drive estimate x%.2f", -T.speed * 3.6f, s_ah_k); }
            AH.r = fminf(fmaxf(h, h0), AH.r + 4 * dt);
            AH.grab_until = now + 0.5;
        } else if (now > AH.grab_until) AH.r = fminf(AH.r, fmaxf(target, AH.r - rate * dt));
        if (T.speed < 0) AH.roll_m += -T.speed * dt;
        if (AH.r <= 0.002f) {
            AH.r = 0;
            if (!AH.released) {
                AH.released = 1;
                if (AH.roll_m < 0.02) s_ah_k = fminf(1, s_ah_k + 0.1f);          // a clean launch: trust the estimate again
                logv(2, "Auto Hold: released after %.2f s (%.1f km/h, rolled back %.2f m)", AH.launch_t, kmh, AH.roll_m);
            }
            // stay in the launch (request 0) until the truck is really going: a roll back still gets caught above
            if (kmh >= 2 || AH.launch_t > 12) AH.st = AH_OFF;
        } else AH.released = 0;
        break; }
    }
    s_want_hold = AH.r;
    disp_post(DISP_CRUISE, DISP_STATUS, AH.r > 0 || s_sg_wait || s_rearm_active ? 'B' : layers_on() ? sl_letter() : 0, 0);   // (layers: LIM has no letter, decision 66)
}



// ---------------------------------------------------------------- ISA state (limit following, resume to limit)
static int s_res_ev;                     // resume pressed while cruise was on (handled in keys_tick)
static float s_res_before;

// The cap the set speed may not exceed because of a speed limiter (km/h, 0 = none): for a truck the game's own limiter (the option
// g_use_speed_limiter on), at the value the game holds it to: the game knows the vehicle (a truck 90, a bus 100, ATS its 65 mph), so
// the cap is that one fact, not a number of ours (ride 14-11: a bus capped at our fixed 90). limiter_kmh > 0 caps lower still
// (the owner's choice); 0 (the default) follows the game. A heavy vehicle the game gives no limiter value (the ETS2 bus, Damien 14-11)
// gets LIMITER_NO_VALUE_KMH, the EU law for a bus (a truck's 90 always comes from the game). The car trailer cap for a light vehicle
// with a trailer, nothing for a car alone.
static const float LIMITER_NO_VALUE_KMH = 100.0f;
static float limiter_cap()
{
    if (vprof()->trailer_cap) return T.trailer ? p_car_trail_lim : 0;
    if (!vprof()->truck_limiter || !speed_limiter_on()) return 0;
    char *ga = sense_actor();
    float g = (ga && sense_lim_on(ga) && sense_lim(ga) > 0) ? sense_lim(ga) : LIMITER_NO_VALUE_KMH;
    return (p_limiter > 0 && p_limiter < g) ? p_limiter : g;
}

// The speed limit the ISA follows (m/s): the one the game reports (car limits for a light vehicle in Road Trip mode or when the
// game is set to car limits).
static float isa_limit_ms()
{
    return road_limit_ms();
}

// The game's units: ATS and ETS2 keep the set speed in km/h internally, but with g_mph on (the ATS profile's "uset g_mph 1") the dash, the
// cruise grid and the driver's whole numbers are miles per hour. In mph mode the steps, the offset of the limit following and the rounding of
// the set speed work in whole mph (the ini keys keep their names: step_tap_kmh 5 is 5 mph then, speed_limit_offset_kmh 3 is 3 mph). Read from
// the cvar once a second; ETS2 has none (km/h).
static const float MPH_KMH = 1.609344f;
static int s_primed_mph = -2;   // sims: -2 = read the game, else 0 / 1
int cruise_units_mph()
{
    if (s_primed_mph != -2) return s_primed_mph;
    static void *cv; static int tried; static double at; static int mph;
    if (!cv && !tried) { tried = 1; cv = sense_cvar("g_mph"); logline(cv ? "Cruise: the game's units: g_mph found (mph when on)" : "Cruise: g_mph not found (km/h)"); }
    if (cv && wall_s() - at > 1) {
        at = wall_s();
        float v = sense_cvar_get(cv);
        int now = v == v && v > 0.5f;
        if (now != mph) logline("Cruise: the game's units are %s", now ? "mph" : "km/h");
        mph = now;
    }
    return cv ? mph : 0;
}
void cruise_units_prime(int mph) { s_primed_mph = mph; }
static float unit_kmh() { return cruise_units_mph() ? MPH_KMH : 1.0f; }
// The set speed (km/h) that follows a limit (m/s): the limit in whole units plus the offset, at least min_set; lim_u = the limit in the unit
// The offset over a limit of lim units: speed_limit_offset_kmh, or in ATS a share of the limit in whole units (Damien 01-11: aim at +5 %,
// the fine starts at +10 %)
static float isa_offset_units(int lim)
{
    if (p_isa_pct > 0 && sense_is_ats()) return roundf(lim * p_isa_pct / 100.0f);
    return p_isa_off;
}
static float isa_set_kmh(float limit_ms, int *lim_u)
{
    float u = unit_kmh();
    int lim = (int)roundf(limit_ms * 3.6f / u);
    if (lim_u) *lim_u = lim;
    return fmaxf((lim + isa_offset_units(lim)) * u, p_min_set);
}

// Resume while on: the set speed becomes limit + offset (>= min_set, <= limiter cap).
static void resume_to_limit(char *ga)
{
    act_cruise_set(ga, s_res_before);  // undo whatever the game's resume did to the set speed
    float limit_ms = isa_limit_ms();   // the car limit for a light vehicle where the game has both (ATS), else the truck limit
    if (limit_ms <= 1) return;
    int lim;
    float v = isa_set_kmh(limit_ms, &lim);
    float cap = g_low_ok ? limiter_cap() : 0;
    if (cap > 0 && v > cap) v = cap;
    act_cruise_set(ga, v);
    logv(1, "Cruise: resume while on: set speed %.0f km/h (limit %d %s)", v, lim, cruise_units_mph() ? "mph" : "km/h");
}

// ---------------------------------------------------------------- cruise keys (cruise-keys-re notes)
// The key handler 0x14099b660 consumes one action object per key (toggle,
// inc, dec, resume at game_actor+0x1e710/20/30/40; +0xf8 == 2 = pressed) with
// `call 0x1401ed460`. We wrap those four calls: one hook sees every press.
//  - Retarder: the activate fn refuses (and the cancel hub cancels) while
//    game_actor+0x180 > 0. On a press we set the level to 0 with the game's
//    own setter 0x1405c74c0, before the handler goes on.
//  - Steps: the handler steps by the grid cvar on every event. We let it
//    happen, then put the set speed back (the controller reads it after the
//    handler, keys_tick runs in front of it) and do the stepping ourselves:
//    release before hold_s = one tap step to the next multiple; held longer =
//    hold steps at hold_rate. The tap step is thus applied on release.
enum { ACT_STATE = 0xf8 };
// The cruise keys' action objects: game_actor + 0x1e710 (toggle), 0x1e720 (inc), 0x1e730 (dec), 0x1e740 (resume),
// 0x1e750 (accmode) on 1.60; they sit +0x100 higher on 1.61 (ETS2 and ATS, sense_is_161()). key 0..4 in that order.
typedef void (*consume_fn)(void *action);
typedef void (*retarder_fn)(void *ga, unsigned level, unsigned char snap);
static consume_fn o_consume;
static retarder_fn o_retarder;
static int g_keys_ok, g_ret_ok;
struct keydir { int ev, was_on; float before; double last_ev, t0, next; float s0; int seq, fired, block; };
static keydir K[2];  // [0] = +, [1] = -
static int s_prev_on;
static double s_key_at;  // last press of a cruise key (on/off, +, -, resume): a drop right after it is the driver's own

// One press seen at the wrapped consume call. Plain C so the sim can feed it.
static void key_event(char *ga, void *act)
{
    int on = layers_on() ? sl_eff_on(ga) : sense_cruise_on(ga);   // (layers: in LIM the state is ours, not the game's byte)
    int is_tog = act == sense_action_at(ga, sense_key_off(0)), is_inc = act == sense_action_at(ga, sense_key_off(1)),
        is_dec = act == sense_action_at(ga, sense_key_off(2)), is_res = act == sense_action_at(ga, sense_key_off(3));
    if (!act || !(is_tog || is_inc || is_dec || is_res)) return;
    s_key_at = wall_s();
    if (layers_on()) sl_key(is_tog ? CSK_TOG : is_inc ? CSK_INC : is_dec ? CSK_DEC : CSK_RES, s_key_at);
    if (g_ret_ok && p_ret_off > 0 && !(is_tog && on) && !(layers_on() && g_cs.mode == CSM_LIM && !(p_lim_show > 0)) && sense_retarder_level(ga) > 0) {   // (layers: not in LIM, where the game's cruise is not engaged: the retarder is the driver's)
        unsigned was = sense_retarder_level(ga);
        act_retarder_level(o_retarder, ga, 0, 1);
        logv(1, "Cruise: %s pressed with the retarder on (level %u): retarder off (now %u)",
                is_tog ? "on/off" : is_inc ? "+" : is_dec ? "-" : "resume", was, sense_retarder_level(ga));
    }
    if (is_res) s_res_press_at = wall_s();
    if (g_keys_ok && is_res && on && p_res_lim > 0) { s_res_ev = 1; s_res_before = sense_cruise_set(ga); }
    if (g_keys_ok && (is_inc || is_dec)) {
        keydir &k = K[is_dec];
        k.ev = 1; k.was_on = on; k.before = sense_cruise_set(ga);
    }
}
static void consume_stub(void *act)
{
    o_consume(act);
    char *ga = sense_actor();
    if (ga) key_event(ga, act);
}

// New set speed for one step in direction dir (+1 / -1) on the grid `step`: the game's own
// formula (cruise-re notes § 4) with unit 1: inc = (floor((s+0.5)/g)+1)*g, dec = floor((s-0.5)/g)*g.
static float key_step(float base, int dir, float step)
{
    if (!(step >= 1)) step = 1;  // 0 or garbage from the ini would give a NaN set speed (audit 11-10)
    return dir > 0 ? (floorf((base + 0.5f) / step) + 1) * step : floorf((base - 0.5f) / step) * step;
}
static void key_apply(char *ga, int dir, float base, float step, const char *why)
{
    // (in mph mode the step grid is whole mph: 5 mph taps, 1 mph holds; the set speed itself stays in km/h)
    const float u = unit_kmh();
    float r = key_step(base / u, dir, step) * u;
    // The truck's speed limiter caps the set speed, as in the game. Not a light vehicle's: ride 31-10 (ATS pickup, 8.3 t, rpm limit 3900) 'cannot go higher than
    // 65 mph': the game's limiter byte (65 mph = 104.6 km/h, the truck governor) stood on for the pickup too, and every + landed on it
    if (dir > 0 && vprof()->truck_limiter) { float cap = limiter_cap(); if (cap > 0 && r > cap) r = cap; }   // the same cap as the frame's (one fact)
    float minv = g_low_ok ? p_min_kmh : 30;
    if (dir < 0 && g_low_ok && p_min_set > 0) { if (r < p_min_set) r = p_min_set; }
    else if (dir < 0 && r < minv) {  // below the minimum: cruise off, like the game's dec
        act_cruise_on(ga, 0); act_cruise_set(ga, 0);
        s_key_at = wall_s();
        logv(1, "Cruise: %s - from %.0f: below the minimum, cruise off", why, base);
        return;
    }
    // the driver's own +/- is taken at once, no slide on the dash (Damien 25-10)
    act_cruise_set(ga, r);
    logv(2, "Cruise: %s %c from %.0f to %.0f km/h (%.0f to %.0f %s)", why, dir > 0 ? '+' : '-', base, r, base / u, r / u, u > 1 ? "mph" : "km/h");
}

// The lane-keeping key with needs_cruise and the cruise off (modules/lane.cpp): switch the cruise on, as Resume does with ISA. The set speed is the limit
// + offset (ISA on and a limit known), else the last set speed, else the speed of now in whole units; never under min_set, never over the limiter's cap.
// 1 = the cruise was switched on (or already is); 0 = refused, *why says why. The state the game's own keys would leave is left too: the retarder goes
// off like on any cruise key, the key time is stamped so the drop detectors do not take it for a driver's cancel, and s_res_press_at marks it as a
// resume, so ISA follows the limit (a "set" keeps the speed until the next sign).
int cruise_lane_engage(const char **why, float *set_kmh)
{
    char *ga = sense_actor();
    static char buf[80];
    *why = "";
    if (!ga || !g_keys_ok || !g_low_ok) { *why = "the cruise hook is not in place"; return 0; }
    if (layers_on() && g_cs.mode == CSM_LIM) { *why = "the speed limiter (LIM) is the cruise state"; return 0; }
    if (sense_cruise_on(ga)) { if (set_kmh) *set_kmh = sense_cruise_set(ga); return 1; }
    const float kmh = T.speed * 3.6f;
    if (!T.engine_on && !cab_ss_engine_off()) { *why = "the engine is off"; return 0; }
    if (T.gear < 0 || T.dgear < 0 || kmh < 0) { *why = "reversing"; return 0; }
    if (kmh < p_min_kmh) { snprintf(buf, sizeof buf, "%.0f km/h is under the cruise's minimum switch-on speed (%.0f)", kmh, p_min_kmh); *why = buf; return 0; }
    const float u = unit_kmh();
    float v = 0; const char *src = "";
    int lim = 0; float limit_ms = isa_limit_ms();
    if (p_isa > 0 && limit_ms > 1) { v = isa_set_kmh(limit_ms, &lim); src = "speed limit + offset"; }
    else {
        float last = layers_on() && g_cs.mem_set[CSM_CC] > 0 ? g_cs.mem_set[CSM_CC] : sense_cruise_set(ga);
        if (last > 0) { v = last; src = "last set speed"; }
        else { v = roundf(kmh / u) * u; src = "current speed"; }
    }
    v = fmaxf(v, p_min_set);
    float cap = limiter_cap();
    if (cap > 0 && v > cap) v = cap;
    if (g_ret_ok && p_ret_off > 0 && sense_retarder_level(ga) > 0) act_retarder_level(o_retarder, ga, 0, 1);   // (as any cruise key: the game's activation refuses with the retarder on)
    act_cruise_set(ga, v);
    act_cruise_on(ga, 1);
    s_key_at = s_res_press_at = wall_s();
    if (set_kmh) *set_kmh = v;
    logv(1, "Cruise: switched on by the lane-keeping key: set speed %.0f km/h (%s)", v, src);
    return 1;
}

// Runs every tick before the controller reads the set speed (and from the frame).
static void keys_tick(char *ga)
{
    if (!g_keys_ok || !ga) return;
    double now = wall_s();
    int on = layers_on() ? sl_eff_on(ga) : sense_cruise_on(ga);
    if (s_res_ev) { s_res_ev = 0; if (on && p_isa > 0) resume_to_limit(ga); }  // Damien: ISA off = hands off
    if (on && !s_prev_on) {
        K[0].block = K[1].block = 1;  // switched on: a key still down from that press is not a step
        // mph: the set speed the game took from the speed is a whole number of mph, as the dash shows it
        if (cruise_units_mph() && p_step_tap > 0) {
            if (sense_cruise_set(ga) > 1) act_cruise_set(ga, roundf(sense_cruise_set(ga) / MPH_KMH) * MPH_KMH);
        }
    }
    for (int d = 0; d < 2; d++) {
        keydir &k = K[d];
        int dir = d ? -1 : 1;
        if (k.ev) {
            k.ev = 0;
            k.last_ev = now;
            if (!k.was_on) { k.block = 1; k.seq = 0; }          // this press switched cruise on
            else if (!on) k.seq = 0;                            // the handler switched it off
            else {
                act_cruise_set(ga, k.before);            // undo the game's step
                if (!k.block && !k.seq) { k.seq = 1; k.t0 = now; k.s0 = k.before; k.fired = 0; }
            }
        }
        void *act = sense_action_at(ga, sense_key_off(d ? 2 : 1));
        int held = now - k.last_ev < 0.08 || (act && sense_action_state_u(act) != 0);
        if (!held) {
            if (k.seq && !k.fired && on) key_apply(ga, dir, k.s0, p_step_tap, "tap");
            k.seq = 0; k.fired = 0; k.block = 0;
        } else if (k.seq && on && now - k.t0 >= p_hold_s) {
            if (!k.fired) { k.fired = 1; k.next = now; }
            if (now >= k.next) {
                key_apply(ga, dir, sense_cruise_set(ga), p_step_hold, "hold");
                double per = 1.0 / (p_hold_rate > 0.5f ? p_hold_rate : 0.5f);
                k.next = k.next + per > now ? k.next + per : now + per;
            }
        }
    }
    s_prev_on = on;
}

// bend_now: the sideways acceleration allowed at a speed, as route.cpp's lat_scale (the Mercedes-Benz patent shape: 5 m/s2 at 25 km/h ... 3.2 at 111,
// as a factor on curve_lat = the value at 92 km/h, at most 4/3.25). A copy: that one is static in route.cpp.
static float bn_lat_scale(float v_ms)
{
    static const float kmh[] = { 25, 51, 67, 92, 111 }, a[] = { 5.0f, 4.0f, 3.5f, 3.25f, 3.2f };
    float k = v_ms * 3.6f, r;
    if (k <= kmh[0]) r = a[0];
    else if (k >= kmh[4]) r = a[4];
    else {
        int i = 0;
        while (k > kmh[i + 1]) i++;
        r = a[i] + (a[i + 1] - a[i]) * (k - kmh[i]) / (kmh[i + 1] - kmh[i]);
    }
    r /= 3.25f;
    return r > 4.0f / 3.25f ? 4.0f / 3.25f : r;
}

static float cruise_body(void *ga, float dt, float pedal, float stock);
static void own_acc(char *g, float dt);
static void acc_layers_tick(char *g, float dt);
static void cr_acc_post(char *g, float dt, float set_kmh, int mi);
static float lead_rel_speed(char *g, float lead, float dt);
static void cruise_trace(char *g, float u);
static void aeb(char *g, float dt);
enum { AEB_OFF, AEB_WARN, AEB_PARTIAL, AEB_FULL, AEB_HOLD };
static struct {
    int st;                                     // AEB_*
    float vl, gap_prev, rate, a_l, vl_prev, req, i, a_cmd, ttc, a_need, thr_at_brake;
    double age_t0, calm_since, lock_until, beep_at, lost_at, thr_since, st_at, close_since; int close_warned, no_soft;
    // the event (for the summary line)
    double ev_t0; float ev_kmh, ev_min_gap, ev_min_ttc, ev_psi0, ev_psi_min, ev_max_dec; int ev_max, air_low; const char *ev_cap;
    // evasive lane change: steering / yaw before the warning (a bend is no evasion), since when it looks like one, until when
    float steer0, yaw0; double evade_since, evade_until;
    // the sideways reading of the radar target (core/aeblat.h): found in the traffic list, its lateral clearance to our path and the weight the stop is asked with
    int lat_ok; float lat_w, lat_clr, lat_d;
} AE;
static unsigned char s_aeb_owns;   // our AEB is hooked in (patches in place)
static int aeb_enabled() { return vprof()->aeb_on > 0 || p_aeb > 0; }   // on for the class of now: a car always, a truck by its ini
static unsigned char s_aeb_watch;  // read by those patches every frame: the game's ACC emergency braking and its AEB stores are
                                   // skipped only while our AEB is watching right now (cruise on, or aeb_without_cruise = 1)
// Our own speed limiter: how much of the throttle may stay. Only the speed in 1 s counts (Damien 25-10: no fade on a climb at 92 km/h):
// full while that is more than 1.5 km/h under the limit, none once it reaches the limit (a direct cut-off: the first version was slow and soft).
static float own_limit_k(float kmh, float accel)
{
    if (!(p_own_lim > 0) || !vprof()->truck_limiter) return 1.0f;   // no truck speed limiter on a light vehicle
    float lim = p_own_lim + p_own_bias;   // the speedometer then shows the configured value, not a bit under it
    if (kmh >= lim) return 0.0f;
    float proj = kmh + accel * 3.6f * 1.0f;
    return fmaxf(0.0f, fminf(1.0f, (lim - proj) / 1.5f));
}
// The same from the vehicle model (Damien 29-10: with the engine at x5 the fade above began at 93 km/h and pumped, it took far too
// much pull): a cap on the throttle at the share of the gear's full force that lets the truck close the gap to the limit at
// gap / g_lim_tau (m/s2) against the road's resistance of now, so the speed meets the limit in an exponential of g_lim_tau seconds
// whatever the engine, the mass and the grade; a strong engine runs at full throttle until a few km/h before it, a weak one on a
// climb is never capped below the limit. No measured acceleration in it (the telemetry's lags 0.5 s, and a strong engine turned
// that lag into a pump), only the speed. Over the limit the cap falls under what holding takes, so the speed comes back by itself.
// NAN = the model cannot tell (no gear, the clutch open, no engine data, over the rev limit): the fade above takes over.
float g_lim_tau = 0.75f;   // s; sims: a 0.6 s torque lag stays under 0.25 km/h over (the sims sweep it; 0 = the old fade alone)
static float s_lim_last = NAN; static double s_lim_last_at = -1;   // own_limit_cap: the cap held through a shift (reset in cr_reset, review 30-10)
static float own_limit_cap(float v)
{
    if (!(p_own_lim > 0) || !vprof()->truck_limiter) return 1.0f;   // no truck speed limiter on a light vehicle
    if (!(g_lim_tau > 0)) return NAN;
    if (!V.eng_ok || V.electric || !(V.mass > 0)) return NAN;
    float full = T.gear >= 1 && clutch_seen() <= 0.05f ? vehicle_force_at(T.gear, v, 1.0f) : 0;
    if (!(full > 0)) return sim_s() - s_lim_last_at < 2 ? s_lim_last : NAN;   // a shift: the cap of before it (the fade would cut in and out)
    float lim = (p_own_lim + 0.25f) / 3.6f;   // it settles on its target, so no bias: a quarter over it, and the speedometer shows the limit, not one under
    float f = V.mass * (lim - v) / g_lim_tau + vehicle_resist(v);
    s_lim_last = fmaxf(0.0f, fminf(1.0f, f / full)); s_lim_last_at = sim_s();
    return s_lim_last;
}
// And the cruise set speed never above it: the limit itself (the bias is for the throttle fade above, else the dash shows 96 at 95).
static void own_limit_set(char *ga)
{
    if (!vprof()->truck_limiter || !(p_own_lim > 0)) return;   // no truck speed limiter on a light vehicle
    if (ga && sense_cruise_on(ga) && sense_cruise_set(ga) > p_own_lim) act_cruise_set(ga, p_own_lim);
}
// [cruise] engine = layers: the knobs (the param table) into the layers, and what the state needs of the module's params and the road
static void sl_configure()
{
    sa_cfg &c = g_slc.arb;
    c.tau_s[0] = fmaxf(0.3f, p_ly_tau_e); c.tau_s[1] = fmaxf(0.3f, p_ly_tau); c.tau_s[2] = fmaxf(0.3f, p_ly_tau_p);
    c.spare_share[0] = p_ly_sp_e; c.spare_share[1] = p_ly_sp; c.spare_share[2] = p_ly_sp_p;
    c.force_rise[0] = fmaxf(0.01f, p_ly_rise_e); c.force_rise[1] = fmaxf(0.01f, p_ly_rise); c.force_rise[2] = fmaxf(0.01f, p_ly_rise_p);
    c.trim_s = fmaxf(0.5f, p_ly_trim_s); c.trim_max = fmaxf(0.0f, p_ly_trim_max);
    c.coast_decel = fmaxf(0.05f, p_ly_coast); c.brake_comfort = fmaxf(0.1f, p_ly_bcomf); c.brake_firm = fmaxf(c.brake_comfort, p_ly_bfirm); c.brake_max = fmaxf(c.brake_firm, p_ly_bmax);
    c.brake_tol = fmaxf(0.0f, p_ly_btol); c.brake_k[0] = p_ly_bk_e; c.brake_k[1] = 1.0f; c.brake_k[2] = p_ly_bk_p;
    c.bind_accel = p_bind_a > 0 ? p_bind_a : -1.0f;   // the calm approach: ahead_accel, 0 = off (as the old ahead_cap read it; a request's own 0 means "no gain at all")
    g_slc.presses = p_lim_presses; g_slc.window_s = p_lim_window; g_slc.show_set = p_lim_show; g_slc.lim_tau_s = p_lim_tau; g_slc.kick_from = p_lim_kick0; g_slc.kick_full = p_lim_kick1;
}
static sl_env layers_env()
{
    sl_env e;
    e.min_on_kmh = g_low_ok ? p_min_kmh : 30; e.min_set_kmh = g_low_ok ? p_min_set : 0;
    float cap = g_low_ok ? limiter_cap() : 0;   // the set never above the game's limiter / the trailer cap, nor above our own limiter
    if (p_own_lim > 0 && vprof()->truck_limiter && (cap <= 0 || p_own_lim < cap)) cap = p_own_lim;
    e.max_set_kmh = cap; e.isa_off_kmh = p_isa_off; e.isa_pct = p_isa_pct; e.unit_kmh = unit_kmh(); e.ats = sense_is_ats(); e.isa = g_low_ok && p_isa > 0;
    e.limit_ms = isa_limit_ms(); e.since_res_s = wall_s() - s_res_press_at;
    return e;
}
// Before the game's own cruise controller runs: the keys, and our ACC target into +0x1a0 (the game's controller and its
// overspeed braking read it right after).
static void cr_pre(act_tick *a)
{
    char *ga = a->ga;
    if (ga) keys_tick(ga);
    if (ga && layers_on()) { sl_configure(); if (sl_pre(ga, layers_env())) s_key_at = wall_s(); }   // the cruise state: the 3x SET, LIM, ISA; what it writes into the game is no drop
    if (layers_on()) { int wr; float top = sl_top_kmh(a->pedal, p_own_lim, &wr); if (wr) g_speed_limiter_kmh = top; }   // LIM's ceiling is the top speed the model and the gearbox see (as the own limiter's is)
    s_want_curve = 0;
    for (int i = 0; i < BRQ_N; i++) { s_ch[i] = 0; s_chu[i] = BRU_COMFORT; }
    s_acc_lay = 0;
    if (ga && layers_on() && p_own_acc > 0) acc_layers_tick(ga, a->dt);   // WP-SP3: the lead model; decides whether the layers follow the vehicle ahead this tick
    if (ga && !s_acc_lay) { if (layers_on()) ot_reset(&g_ot); own_acc(ga, a->dt); }   // the classic ACC: engine = classic, own_acc off, or a vehicle the layers cannot follow
}
// After it: our controller and the holds (cruise_body), the AEB, the pedal; the brake wishes and the base throttle go to
// the actuator, which merges them with the other modules' (Creep's crawl throttle, the rollover guard's and launch control's
// caps) and writes once. Our own speed limiter is a scale on the merged throttle, applied last.
static void cr_post(act_tick *a)
{
    char *ga = a->ga; float pedal = a->pedal;
    float r = cruise_body(ga, a->dt, pedal, a->stock);
    cr_view_update(ga, pedal);
    brake_descent_view(s_view_from, s_view_target);   // (the engine-brake stage the channel holds on a truck without a retarder reads it, core/brake.cpp)
    if (ga) aeb(ga, a->dt);
    if (AE.st > 0 && r > pedal) r = pedal;  // AEB warns or brakes: no cruise throttle
    // The driver's gas always wins over our own braking (Damien, ts4w: full gas near a stop did nothing until he
    // switched cruise off; the game zeroes the throttle while a brake request is set). Auto Hold lets go by itself.
    if (pedal > 0.1f) { s_want_curve = 0; s_want_acc = 0; for (int i = 0; i < BRQ_N; i++) s_ch[i] = 0; }  // (the AEB has its own override: kick-down)
    // [planner] drive: the plan follows the lead and brakes through BRQ_PLANNER; the own ACC's comfort brake (s_ch[BRQ_ACC]) stays as the
    // fast, per-tick reaction to a cut-in or a hard close the 4 Hz plan has not seen yet. The brake channel takes the largest ask.
    planner_drive pd = g_planner_drive;
    int pl_follow = pd.on && pd.fresh;
    float ch_max = 0;
    if (brake_channel_on()) {
        for (int i = 0; i < BRQ_N; i++) { if (i == BRQ_ACC) brake_ask_add(i, s_ch[i], s_chu[i]); else if (i != BRQ_PLANNER) brake_ask(i, s_ch[i], s_chu[i]); ch_max = fmaxf(ch_max, s_ch[i]); }
        brake_game_overspeed(ga ? cr_game_brake_from(ga, pedal) : NAN);   // (the channel counts the game's own overspeed retarder as built)
    }
    if ((s_want_acc > 0.04f || s_want_curve > 0.04f || ch_max > 0 || (pl_follow && pd.act == PL_BRAKE && pd.decel > 0.05f)) && r > pedal) r = pedal;  // we brake: no throttle of ours against it (ts4r 213 s, ts4w 532 s)
    if (!(r > pedal)) CF.at = -1;   // (force_mode: TrueShift reads the cruise's force only while its throttle goes through)
    act_brake(ACT_HILL, s_want_hill); act_brake(ACT_HOLD, s_want_hold); act_brake(ACT_BEND, s_want_curve);
    act_brake(ACT_ACC, s_want_acc); act_brake(ACT_AEB, s_want_aeb);
    // The pedal (modules/pedal.cpp, [pedal] wish, the torque pedal) takes this stage over where it applies: ONE pedal definition everywhere, its map per mode (owner 03-11, P6.4). The
    // cruise had curves of its own here (Eco p^1.35, Power 1 - (1 - p)^1.6, blended after a mode change): a second map that ran when the torque pedal passed the throttle through (a shift,
    // a slipping clutch), and a third was Normal's linear. They are gone; with [pedal] wish 0, and wherever the torque pedal passes through, the throttle is the game's own pedal.
    g_pedal_mapped = 0;
    float wish;
    if (ga && pedal_wish(r, a->dt, sense_cruise_on(ga) || g_lv.crawl || AE.st > 0 || fabsf(r - pedal) > 1e-4f, &wish)) {
        if (wish != r) { r = wish; g_pedal_mapped = 1; }
    }
    act_thr_base(r);
    // our own speed limiter (the game's is switched off): a cap from the vehicle model, the old fade where the model cannot tell. Posted
    // only when it holds the throttle below what it would be (TrueShift reads a cap as "the pedal is not a request for pull")
    float cap = own_limit_cap(fabsf(T.speed));
    float lim_fade = 1.0f;   // layers: where the model cannot tell LIM's ceiling, the old fade on its limit
    if (layers_on()) {   // LIM's ceiling with its kickdown pass (the own limiter stays hard)
        float lc = sl_lim_cap(pedal, fabsf(T.speed));
        if (lc == lc && lc < 1.0f) cap = cap == cap ? fminf(cap, lc) : lc;
        else if (lc != lc) lim_fade = sl_lim_fade(fabsf(T.speed) * 3.6f, T.accel);
    }
    act_thr_cap(ACT_CRUISE, cap == cap && cap < r ? cap : 1.0f);
    act_thr_scale(ACT_CRUISE, (cap == cap ? 1.0f : own_limit_k(fabsf(T.speed) * 3.6f, T.accel)) * lim_fade);
}
// After the merge: the trace sees the throttle the engine gets.
static void cr_done(act_tick *a)
{
    if (a->ga) cruise_trace(a->ga, a->thr);
}
static void cr_attach()
{
    act_hook(ACT_PRE, cr_pre);
    act_hook(ACT_POST, cr_post);
    act_hook(ACT_DONE, cr_done);
}

// force_mode: the throttle per unit of the controller's output in the gear of now, the force a full wish asks at v over what this gear gives
// at full throttle (the wish pedal's steps 4 and 5); 0 = the vehicle model cannot tell (no gear, over the rev limit, no engine data)
static float force_k(float v)
{
    int g = T.gear;
    if (g < 1 || !V.eng_ok || V.electric) return 0;
    float full = wish_full(v), fg = vehicle_force_at(g, v, 1.0f);
    // Below the idle rpm the locked-driveline force is ~0 (the clutch slips and the engine stays at idle): the gear's full
    // throttle then really gives the launch pull at idle, so the throttle per force unit stays finite and the controller can
    // build the throttle to pull away on a grade (hill-report 31-10).
    float fl = vehicle_force_at(g, vehicle_kmh_at(g, ts_idle_rpm()) / 3.6f, 1.0f);
    if (fg < fl) fg = fl;
    return full > 0 && fg > 0 ? full / fg : 0;
}

// approach_by_mode: the highest throttle the approach to the set speed may use in this gear (1 = no limit). The force it may ask is the force that
// holds the speed plus the mode's share of the pull to spare (a full wish at this speed, the wish pedal's reference, less what holding takes).
// The share rises (0.5 per second) while the truck gains less than 0.05 m/s2 although it is more than 1 km/h under the speed (a climb, a
// wrong mass), and falls back when it gains again. Kept as it was across a shift (the torque gap shows no gear).
static float s_app_ex, s_app_ceil = 1;
static float approach_ceiling(int md, float e, float acc_kmhs, float dt, int shifting)
{
    bool slow = acc_kmhs < 0.18f && e > 1.0f;
    s_app_ex = slow ? fminf(1.0f, s_app_ex + 0.5f * dt) : fmaxf(0.0f, s_app_ex - 0.5f * dt);
    if (shifting) return s_app_ceil;
    float v = fabsf(T.speed), full = wish_full(v), fg = T.gear > 0 && V.eng_ok && !V.electric ? vehicle_force_at(T.gear, v, 1.0f) : 0;
    if (!(full > 0) || !(fg > 0)) return s_app_ceil = 1;
    float fh = fmaxf(0.0f, vehicle_resist(v)), share = fminf(1.0f, fmaxf(0.0f, md == MODE_ECO ? p_app_sh_eco : md == MODE_POWER ? p_app_sh_pw : p_app_sh));
    if (full - fh < 0.25f * full) return s_app_ceil = 1;   // little to spare (a climb that takes nearly all the pull): the whole throttle, as a driver would
    share += (1 - share) * s_app_ex;
    return s_app_ceil = fminf(1.0f, fmaxf(0.0f, (fh + share * fmaxf(0.0f, full - fh)) / fg));
}

// 0 below a, 1 from b, linear between (a weight in place of a switch)
static float ramp01(float x, float a, float b) { return x <= a ? 0.0f : x >= b ? 1.0f : (x - a) / (b - a); }

// The coast deceleration over the next d meters: gravity of the mean grade the route has there on top of base (the grade of the truck now
// when the route does not reach that far). Ride 8 (27-10): the grade of now at the foot of a climb planned a junction 290 m ahead as if the
// whole road climbed, and at the top of it as if the climb went on.
static float coast_over(float base, float d, float lo, float hi)
{
    float gm = d >= 10 ? route_grade(0, d) : NAN;
    if (gm != gm) gm = pitch_grade();
    return fminf(hi, fmaxf(lo, base + 0.0981f * gm));
}
// The speed (m/s) the cruise may chase while a slower speed lies ahead: env is the speed the coast plan allows now (along the road it falls as
// env(s)^2 = env^2 - 2 a_c s) and the truck gains at a_g. The two meet where v^2 + 2 a_g s = env^2 - 2 a_c s: that speed is the
// highest the truck can reach and still follow the plan down by coasting. Faster would be a burst followed by a cut. No fixed distance: far
// from the bend env is large and the result is above the set speed. env at or under the speed of now (already slowing): env itself.
static float ahead_cap(float v, float env, float a_c)
{
    if (!(env == env) || p_bind_a <= 0 || env <= v || !(a_c > 0)) return env;
    return sqrtf((a_c * v * v + p_bind_a * env * env) / (p_bind_a + a_c));
}

// aeb_rising: the deceleration to ask now. The plan rises as a(t) = min(a_top, a0 + jerk t) and stops the truck (speed v) exactly after `room` meters;
// a0 is found by bisection (the stopping distance falls as a0 rises). Where even a_top held from now does not stop in time: a_top at once.
// With room to spare a0 is low and the rise does the work; with little room the plan runs into a_top and holds it.
static float aeb_rise_decel(float v, float room, float a_top, float jerk)
{
    auto dist = [&](float a0) {
        float x = 0, u = v, t = 0; const float h = 0.02f;
        while (u > 0 && t < 20) { float a = fminf(a_top, a0 + jerk * t); x += u * h - 0.5f * a * h * h; u -= a * h; t += h; }
        return x;
    };
    if (!(v > 0.1f) || !(room > 0.05f) || dist(a_top) > room) return a_top;
    float lo = 0, hi = a_top;
    for (int i = 0; i < 14; i++) { float m = 0.5f * (lo + hi); if (dist(m) > room) lo = m; else hi = m; }
    return hi;
}

// ---------------------------------------------------------------- [cruise] engine = layers: what the road asks (WP-SP2, docs/cruise-layers.md)
// The road's requesters (core/req_*.h) are pure: they take what the module senses and post "v at distance d, because ..." to the arbiter. Here is the sensing and the
// two things that write the game: the junction cancel (card 22) and the roundabout speed from the learned lean. The lessons of the classic code are carried, the code is not.
static rb_now_state g_rb_now;      // the bend you are in: the curvature filter
static rp_state g_rp;              // the PCC's crest ease-off in force
static float s_rj_hush;            // the junction request faded out by your pedal or indicator (0 = there, 1 = gone), slewed
static float g_rb_factor = 1.0f;   // the roundabout's entry speed over the ini's, from how far this truck at this load leans (core/roll.h roll_rb_factor); 1 = nothing learned
static int cr_jn_cancel(char *g, const route_junction_t &ji, float jn_orig_kmh, float kmh)
{
    // A few seconds before a junction or roundabout we slowed for, the cruise drops with a sound: the rest is the driver's. Like the game's own cancels only the on flag
    // drops; the set speed stays and Resume brings it back (Damien 13-10). Card 22: 7 s with the owner's ini (5 by default), the conditions as they were.
    if (!(ji.limit_ms > 0) || !(p_jn_cancel_s > 0) || T.paused) return 0;
    double now_c = wall_s();
    float t_to = fmaxf(ji.dist_m - (ji.roundabout ? 15 : 8), 0.0f) / fmaxf(kmh / 3.6f, 1.0f);
    // A roundabout is a hard rule (Damien 24-10: the cruise stayed on at one): no 5 s after switching on / 20 s between two cancels, only 1.5 s and 3 s.
    bool free_to_cancel = ji.roundabout ? (now_c - s_cb_on_since > 1.5 && now_c - s_jn_cancel_at > 3) : (now_c - s_cb_on_since > 5 && now_c - s_jn_cancel_at > 20);
    bool slowed = kmh <= jn_orig_kmh + p_jn_margin + p_jn_hand_kmh || ji.dist_m <= p_jn_hand_m;   // handed over when slowed down or close (Damien 25-10: later, slower)
    // A junction is only cancelled while the truck is still faster than its speed + 3 (slower = you are at it already); a roundabout also behind a vehicle, but not under
    // 5 km/h and not while Stop&Go holds the standstill in a queue before the entrance.
    bool driving = ji.roundabout ? (kmh >= 5 && !g_sg_hold) : kmh > jn_orig_kmh + 3;
    if (t_to <= p_jn_cancel_s && driving && free_to_cancel && slowed) {
        s_jn_cancel_at = now_c;
        if (ji.roundabout) g_rb_cancel_at = now_c;   // (Lane reads it)
        act_cruise_on(g, 0);
        play_cue(CUE_CRUISE_DROP);
        logline("Cruise: %s in %.0f m (%.1f s) at %.0f km/h, its speed %.0f (+%.0f margin): cruise cancelled for you, Resume brings the set speed back",
                ji.roundabout ? "roundabout" : "junction", ji.dist_m, t_to, kmh, jn_orig_kmh, p_jn_margin);
        return 1;
    }
    return 0;
}
// Posts everything the road asks of the speed (the set is posted already). rolling: in neutral (the eco-roll) the bends and junctions still ask their brake (the old
// code asked before it returned for the roll), the descent and the limit line do not (a roll ends on a brake, not on a line). Returns 1 when the cruise was cancelled
// for a junction or roundabout (the caller returns the game's throttle: the cruise is off).
static int cr_road_post(char *g, float dt, float set_kmh, float kmh, int rolling)
{
    const sa_cfg &c = g_slc.arb;
    const float v = fabsf(T.speed), vm = fmaxf(v, 5.0f);
    const int rt = route_ok();
    s_bend_coast_kmh = NAN; s_mom_up = 0;
    // ---- the bends of the road ahead (17) and the junctions and roundabouts (20, 21, 23, 24): they need the route and [cruise] curves, as they always did
    if (p_curve > 0 && rt) {
        const float reach = route_ahead_m();
        {
            route_bend_t bp[8]; rb_pt pts[8];
            const int nb = route_bend_points(bend_lat(), bp, 8);
            for (int i = 0; i < nb; i++) {
                float gm = bp[i].d_m >= 10 ? route_grade(0, bp[i].d_m) : NAN;   // the road the truck coasts on is the road up to the bend, not the road under it now
                if (gm != gm) gm = pitch_grade();
                pts[i].v_ms = bp[i].v_ms; pts[i].d_m = bp[i].d_m; pts[i].grade_pct = gm;
            }
            rb_in bi; bi.pt = pts; bi.n = nb; bi.reach_m = reach; bi.v = v; bi.brake = p_curve_brake > 0;
            s_bend_coast_kmh = rb_post_ahead(&g_sa, c, bi);
        }
        if (junc_on()) {
            route_junction_t ji = {};
            float vj = route_junction_speed(fminf(1.0f, fmaxf(0.1f, c.coast_decel + 0.0981f * pitch_grade())), &ji);
            if (vj == vj) vj = route_junction_speed(coast_over(c.coast_decel, ji.dist_m, 0.1f, 1.0f), &ji);   // again with the grade over the road to that junction
            // You step on the gas for a junction we are slowing for: you go another way than the route (Damien 12-10: a petrol station with 3-4 side entrances).
            // Junctions and roundabouts are left alone for 6 s; the bends of the road itself still count. Your pedal wins (23).
            if (vj == vj && T.in_thr > 0.3f && kmh < 60 && ji.dist_m < 250 && (kmh > ji.limit_ms * 3.6f - 5)) {
                if (wall_s() >= s_jn_ignore_until) logv(1, "Cruise: gas pedal at a junction ahead (%.0f m, %.0f\xc2\xb0 \xe2\x86\x92 %.0f km/h): junctions are left alone for 6 s", ji.dist_m, ji.turn_deg, ji.limit_ms * 3.6f);
                s_jn_ignore_until = wall_s() + 6;
            }
            const int there = vj == vj && ji.limit_ms > 0;   // a junction is in view
            int counts = there;                                // ... and it counts: your pedal and your indicator say whether the route is where you are going
            if (wall_s() < s_jn_ignore_until) counts = 0;
            // a gentle route turn without an indicator: you are going straight on (Damien 25-10: crossings) (24)
            if (counts && p_jn_signal > 0 && !ji.roundabout && ji.turn_deg < p_jn_signal) {
                unsigned char fl = sense_lights(g);   // bit 4 left, bit 8 right (both = hazards)
                if (!((fl & 12) == 4 || (fl & 12) == 8)) counts = 0;
            }
            // what the pedal or the indicator decides comes and goes over seconds, not in a frame (no step in a request): 2 s in, 0.5 s out. With no junction in view the
            // weight stands at "there", so the next one is not held back by this one.
            if (!there) s_rj_hush = 0; else s_rj_hush = counts ? fmaxf(0.0f, s_rj_hush - dt / 2.0f) : fminf(1.0f, s_rj_hush + dt / 0.5f);
            if (counts) {
                const float jn_orig_kmh = ji.limit_ms * 3.6f;
                if (cr_jn_cancel(g, ji, jn_orig_kmh, kmh)) return 1;
            }
            if (there && s_rj_hush < 0.999f) {
                rj_in in; memset(&in, 0, sizeof in);
                in.valid = 1; in.dist_m = ji.dist_m; in.base_ms = ji.limit_ms; in.margin_ms = fmaxf(0.0f, p_jn_margin) / 3.6f; in.roundabout = ji.roundabout;
                in.grade_pct = ji.dist_m >= 10 ? route_grade(0, ji.dist_m) : NAN; if (in.grade_pct != in.grade_pct) in.grade_pct = pitch_grade();
                in.plan_decel = p_jn_dec; in.reach_m = reach; in.v = v; in.brake = p_curve_brake > 0; in.hush = s_rj_hush;
                rj_post(&g_sa, c, in);
            }
        }
    }
    // ---- the bend the truck is IN (18): no route needed
    if (p_bend_now > 0) rb_post_now(&g_sa, &g_rb_now, dt, T.yaw, T.speed, bend_lat(), bn_lat_scale, p_curve_brake > 0, NULL);
    else rb_now_reset(&g_rb_now);
    if (rolling) return 0;
    // ---- PCC (25) and the eco-roll's boost (9)
    {
        rp_in in; memset(&in, 0, sizeof in);
        const int eco = cruise_mode() == MODE_ECO, power = cruise_mode() == MODE_POWER;
        in.on = p_predict > 0 && rt; in.lead_near = s_lead_near; in.boost_kmh = g_roll_boost_kmh > 0 ? g_roll_boost_kmh : 0;
        in.g_now = in.g_up = in.after = NAN;
        if (in.on) {
            in.up_kmh = band_up(); in.down_kmh = (eco ? 5 : 3) * band_scale(T.cruise) * crest_mass_k() * (power ? 0.5f : 1.0f);
            in.g_now = route_grade(0, 50); in.g_up = route_max_grade(vm * 3, vm * 15);
            const float crest = route_crest_m(vm * 12);
            if (crest >= 0) in.after = route_min_grade(crest, crest + vm * 15);
        }
        const rp_out po = rp_post(&g_sa, &g_rp, in, set_kmh / 3.6f, dt, c.raise_up, c.raise_dn);
        s_mom_up = po.mom_up_kmh;   // (the game's own overspeed braking must allow the momentum: cr_frame and cruise_hold_kmh read it)
    }
    // ---- the legal class: the descent hold (8) and the limit + cap line (13)
    if (T.cruise > 0.5f) {
        const float ga = rt ? route_grade(0, 50) : NAN;
        if (p_descent > 0) rl_post_descent(&g_sa, set_kmh / 3.6f, fmaxf(0.0f, p_desc_kmh) * band_scale(T.cruise), s_down_now ? cruise_hold_kmh(NAN) : NAN);
        const float room = overspeed_room(T.cruise);
        if (room < 1e8f) {
            rl_line_in li; li.line_kmh = T.cruise * 3.6f + room; li.accel = T.accel; li.grade_pct = fminf(pitch_grade(), ga == ga ? ga : 0.0f);
            rl_post_line(&g_sa, li);
        }
    }
    return 0;
}

// What the arbiter asks the brake channel (one total deceleration, BRQ_SPEED), with the urgency of the request that asks. Shared by the force path and the model-less one below.
static void cr_layers_brake(const sa_out &o)
{
    if (!(o.ok && o.brake_decel > 0)) return;
    if (brake_channel_on()) {
        s_ch[BRQ_SPEED] = o.brake_decel;
        // the urgency of the request that asks: the bends and junctions brake firm (the old bend brake), the descent comfortably (the retarder first), the limit line hard
        const int road = o.brake_owner == SAO_BEND || o.brake_owner == SAO_BEND_NOW || o.brake_owner == SAO_JUNCTION;
        // (WP-SP3) the vehicle ahead: a protection is urgent (no jerk limit, the service brake fills at once), a firm ask is firm, a gentle one comfortable (the retarder first)
        const int lead = o.brake_owner == SAO_LEAD;
        s_chu[BRQ_SPEED] = o.brake_urgent ? BRU_URGENT : lead && o.brake_cls <= SAC_PROTECT ? BRU_URGENT : o.brake_owner == SAO_DESCENT ? BRU_COMFORT : road || (!lead && o.brake_cls <= SAC_LEGAL) || o.brake_decel > g_slc.arb.brake_comfort * 1.25f ? BRU_FIRM : BRU_COMFORT;
        // (WP-SP3 round 3) The plan waits for the gear: behind a vehicle the truck may owe the braking of a shift gap (core/brake.cpp) as long as the time to collision stays above the lead model's
        // tension threshold (LM_TTC_FROM_S, 6 s): the speed it may owe is the one that keeps the gap over 6 s of closing, D = gap / 6 s - the closing speed (nothing left to owe when the gap is
        // tense: a lead that brakes hard and the first sight of a queue bridge as before), at most the arbiter's firm level in m/s (one second of a firm ask: the shift gaps last 0.5-1 s);
        // and nothing at all while the gap is under the wanted one (a cut-in that sits closer than the time gap, ticks 21-22 s of the cut-in script: the owner's gap is already short), the share
        // coming in between the wanted gap and 1.2 x it
        brake_patience(BRQ_SPEED, lead && s_chu[BRQ_SPEED] != BRU_URGENT && g_lmo.valid && g_rao.following ? fminf(g_slc.arb.brake_firm, fmaxf(0.0f, g_lmo.gap / LM_TTC_FROM_S - fmaxf(0.0f, g_lmo.closing))) * lm_ramp(g_lmo.gap / fmaxf(1.0f, g_rao.wanted_m), 1.0f, 1.2f) : 0.0f);
    }
    else s_want_curve = fmaxf(s_want_curve, fminf(0.6f, o.brake_decel / vehicle_brake_cautious() + 0.02f));
}

// WP-SP2b: [cruise] engine = layers on a truck whose vehicle model cannot tell the force (V.eng_ok = 0 or no gear: force_k() is 0). Ride 04-11, the Spectrans Scania with the Allison
// box: its engine read invalid all ride (TrueShift 'no data', Pedal [no_model], the engine brake 'worth 0.00') and this body returned the game's own throttle BEFORE the road was asked, for
// every tick of the CC: no bend, no junction, no descent hold, no limit line, no reason in the log (P9) - the CC held 63 km/h into a loop ramp with a 37 km/h bend and the driver braked
// late. The road and the law need the speed, the route and the mass, not the engine; only the FORCE does. So the same requests are posted and decided (the winner, its reason, the
// brake ask: all as with a model), and the game's own cruise holds the speed with its own throttle, which the decision shapes with one continuous weight (core/speedarb.h
// sa_nf_weight): whole while the set binds or the truck is more than an approach's coasting under the allowed speed, off at or over it. Never gas with the brake: the weight is 0 at once
// while the brake is asked (cr_post's gate too). The weight moves at the arbiter's own force rates (up force_rise of the throttle per second, down twice as fast: the old PI's rate), so
// whatever the game's own cruise has wound up while the road held it back (its integrator) comes back as a ramp and not as a step, and a request that appears or goes
// in a frame does not move the throttle in a frame. The arbiter starts over when this begins, so a model that comes back starts bumpless (cr_layers_body's reset).
static int s_nm_ran;   // the last tick of the CC was the model-less one (the arbiter starts over when it begins; the line below is said once per start)
static float s_nm_w = 1.0f;   // the weight on the game's throttle in force (1 = the game's own)
static double s_nm_brk_at = -1e9;   // the last time the speed layer asked for a brake: no throttle until the request it left in the game has gone (the channel lets go over a few tenths of a second)
static double s_nm_since = -1;      // when this stretch of model-less ticks began
static float cr_layers_nomodel(char *g, float dt, float pedal, float stock, float kmh, int mi, float set)
{
    if (!s_nm_ran) { sa_reset(&g_sa, 0); rb_now_reset(&g_rb_now); rp_reset(&g_rp); s_nm_ran = 1; s_nm_w = 1.0f; s_nm_brk_at = -1e9; s_nm_since = wall_s(); }
    {   // said once the stretch has lasted a second (a debounce: a tick of it in a shift or a gear change is no news), and not again within a minute (P9: the reason is visible)
        static double said_at = -1e9;
        if (wall_s() - s_nm_since > 1.0 && wall_s() - said_at > 60) {
            said_at = wall_s();
            logv(1, "Cruise (layers): the vehicle model cannot tell the force of this truck (%s): the game's own cruise holds the speed, the bends, junctions and the law still lift the throttle and brake", V.eng_ok ? (V.electric ? "electric" : "no force in gear now") : "its engine data is unreadable");
        }
    }
    sl_cc_begin(set, g_cs.isa_last >= 0);
    if (cr_road_post(g, dt, set, kmh, 0)) { sa_reset(&g_sa, 0); s_nm_ran = 0; return stock; }   // the cruise dropped for a junction or roundabout: the road is the driver's
    cr_acc_post(g, dt, set, mi);
    sl_cc_run(dt, mi, 0, 1);
    const sa_out &o = g_sl_out;
    s_target_dbg = o.ok ? o.v_target_ms * 3.6f : set;
    sl_log_decision(o);
    cr_layers_brake(o);
    if (!o.ok) { s_nm_w = 1.0f; return stock; }
    float w = sa_nf_weight(fabsf(T.speed), o.v_target_ms, set / 3.6f + o.raise_ms, sa_nf_x(g_slc.arb, mi));
    const float rise = g_slc.arb.force_rise[mi] * dt;
    if (o.brake_decel > 0) s_nm_brk_at = wall_s();
    // asked now, or asked and the request is still standing (never gas with the brake). The window is a safety net for a request some other owner keeps up: the longest the channel
    // can take to let go of the largest ask the arbiter makes (brake_max at the channel's comfort jerk, [brake.channel] jerk_comfort 1.5 m/s3: 3.5 / 1.5 = 2.3 s)
    const bool braking = o.brake_decel > 0 || (wall_s() - s_nm_brk_at < g_slc.arb.brake_max / 1.5f && act_request() > 0.02f);
    if (braking) w = 0.0f;                                                        // the throttle goes at once, as the arbiter's force does
    else w = fminf(s_nm_w + rise, fmaxf(s_nm_w - 2.0f * rise, w));                // ... and otherwise at the force's rates
    s_nm_w = w;
    return w >= 1.0f ? stock : fmaxf(pedal, stock * w);
}

// [cruise] engine = layers: the CC through the speed arbiter (core/speedarb.h, docs/cruise-layers.md). It replaces the target composition (PCC, bends,
// junctions, the lead, the planner), the PI and the descent / limit-line brakes of cruise_body below; Auto Hold and Hill start ran before it, the
// AEB, the pedal and the limiter run after it (cr_post). While a vehicle ahead is followed (the ACC target below the set) the gate hands the tick to the
// classic loop: the ACC has no layers yet. The force goes to the throttle the way the PI's did (force_k, CF: TrueShift reads cruise_wish_force), the
// deceleration to the brake channel (its own requester, BRQ_SPEED). WP-SP2: the road asks the arbiter too (cr_road_post above).
static float cr_layers_body(char *g, float dt, float pedal, float stock)
{
    if (!s_cb_was_on) s_cb_on_since = wall_s();
    s_cb_was_on = 1;
    const float kmh = fabsf(T.speed) * 3.6f;
    const int md = cruise_mode(), mi = md == MODE_ECO ? 0 : md == MODE_POWER ? 2 : 1;
    const float set = sense_cruise_set(g);
    const float lead = lead_gap_m(g);
    s_lead_near = lead > 0.5f && lead < fmaxf(80.0f, 4 * fabsf(T.speed));   // (band_up() reads it: no momentum with a vehicle close ahead)
    const int shifting = T.eff_clutch > 0.05f && kmh > 2;
    const int rolling = g_rolling;   // in neutral (the eco-roll): no throttle, the engine at idle; the game must not cancel the cruise (ts3b)
    {   // the roundabout's entry speed follows how far this truck at this load leans; the route layer reads it (cr_frame)
        const float f0 = g_rb_factor;
        g_rb_factor = p_ly_rb > 0 ? roll_rb_factor(dt) : 1.0f;
        static double said_rb; if (fabsf(g_rb_factor - f0) > 0.01f && wall_s() - said_rb > 20) { said_rb = wall_s(); logv(1, "Cruise (layers): roundabout entry speed %.1f km/h (the ini's %.0f x %.2f from the lean learned for this truck and load)", p_rb_kmh * g_rb_factor, p_rb_kmh, g_rb_factor); }
    }
    float k = 1;
    if (!rolling) {
        const float kn = shifting ? 0.0f : force_k(kmh / 3.6f);   // (in a shift: the gear before it)
        if (!(kn > 0 || (CF.fm && C.on))) { C.on = 0; s_layers_ran = 0; return cr_layers_nomodel(g, dt, pedal, stock, kmh, mi, set); }   // the model cannot tell the force: the game's own throttle, shaped by what the road and the law ask (WP-SP2b; it was the game's own cruise and nothing else)
        k = kn > 0 ? kn : CF.k;
        s_nm_ran = 0;
        if (!s_layers_ran || !C.on) { sa_reset(&g_sa, !C.on ? (k > 0 ? stock / k : 0.0f) : C.u); C.on = 1; s_layers_ran = 1; rb_now_reset(&g_rb_now); rp_reset(&g_rp); }   // bumpless: the force share the throttle had (the game's, or the classic loop's)
        CF.fm = 1; CF.k = k;
    }
    sl_cc_begin(set, g_cs.isa_last >= 0);
    if (cr_road_post(g, dt, set, kmh, rolling)) { sa_reset(&g_sa, 0); return stock; }   // the cruise dropped for a junction or roundabout: the road is the driver's
    cr_acc_post(g, dt, set, mi);   // WP-SP3: the vehicle ahead asks like the road does (also while rolling in neutral: a lead that brakes does not wait for the roll to end)
    sl_cc_run(dt, mi, rolling ? 0 : shifting);
    const sa_out &o = g_sl_out;
    s_target_dbg = o.ok ? o.v_target_ms * 3.6f : set;
    sl_log_decision(o);
    cr_layers_brake(o);
    if (rolling) {   // in neutral: no throttle of ours
        sa_reset(&g_sa, 0); C.on = 1; C.u = C.i = 0;
        if (g_sg_ok) { g_sg_hold = 1; g_feed = 0; }
        return pedal;
    }
    float share = o.ok ? o.share : 0.0f, thr = fminf(1.0f, share * k);
    if (g_sg_ok) {   // Stop&Go (core/stopgo.h: the classic's bookkeeping as a core, rows hold it to the loop below tick for tick; g_stopped_at stays the global it was)
        const int own = low_owner_on();
        sg_cfg sc; sc.on = p_sg; sc.auto_s = p_sg_auto;
        sg_in si; memset(&si, 0, sizeof si);
        si.now = wall_s(); si.kmh = kmh; si.owner = own; si.zone = own ? low_sg_zone() : 0; si.stopped = own ? low_stopped() : 0; si.lead = lead > 0.5f; si.parking = T.parking_brake; si.thr = thr;
        si.go = wall_s() - s_res_press_at < 1.0 || T.in_thr > 0.05f;   // Resume, or the pedal
        g_sgs.stopped_at = g_stopped_at;
        const sg_out so = sg_step(&g_sgs, sc, si);
        g_stopped_at = g_sgs.stopped_at;
        g_sg_hold = so.hold; s_sg_wait = so.wait; g_feed = so.feed;
        {   // after the wait the truck does not drive off by itself, whatever the vehicle ahead does: Resume or the pedal (the 'B' on the dash)
            static int was_stay;
            if (so.stay && !was_stay) logv(1, "Cruise (layers): Stop&Go: stood %.0f s, the truck waits for Resume or the pedal (stop_go_auto_s %.0f)", so.stood_s, p_sg_auto);
            else if (!so.stay && was_stay) logv(1, "Cruise (layers): Stop&Go: the wait is over (%s)", kmh >= 3 ? "the truck moves" : "Resume or the pedal");
            was_stay = so.stay;
        }
        if (so.stay) share = thr = 0;
    } else { g_sg_hold = 0; g_feed = 0; s_sg_wait = 0; }
    {   // held back by something else (ts4n): low air, or full throttle without moving for 4 s: our throttle 0 until the truck moves again
        int air_bad = T.air_warning || (T.air_psi > 1 && T.air_psi < air_pause_psi());
        int stuck = thr > 0.5f && kmh < 2 && T.in_thr < 0.02f;
        if (!stuck) s_stuck_since = 0;
        else if (!s_stuck_since) s_stuck_since = wall_s();
        if (air_bad || (s_stuck_since && wall_s() - s_stuck_since > 4)) { sa_reset(&g_sa, 0); share = thr = 0; }
    }
    {   // (WP-SP3) never gas with the brake: the arbiter's force is 0 while it asks for a brake, but the channel lets go of what it was asked over a few tenths of a second (the retarder's build-up,
        // the service brake's release), and a force that comes back the moment the ask ends is gas and retarder together. The throttle waits until the channel is quiet: its window is the
        // longest the channel takes to let go of the largest ask at its comfort jerk (1.5 m/s3)
        static double brk_at = -1e9;
        if (o.ok && o.brake_decel > 0 && o.brake_owner == SAO_LEAD) brk_at = wall_s();   // (only the vehicle ahead's brake: the road's keeps the SP2 behavior to the digit; its rows measured 0.07-0.33 s of overlap on a stub bend that this guard would also have removed, and moved a bend's sideways peak by 0.3 m/s2 in the sim)
        brake_view bv; brake_view_get(&bv);
        const int busy = bv.on && (bv.active || bv.ret_built > 0.05f || bv.request > 0.02f);
        if (!rolling && share > 0 && wall_s() - brk_at < fmaxf(g_slc.arb.brake_max, ra_protect_max(vp_or(vprof()->aeb_max_dec, p_aeb_max_dec))) / 1.5f && busy) { share = thr = 0; g_sa.share_prev = 0; }
    }
    C.u = share;   // (the unit the PI kept: a share of a full wish)
    if (thr > pedal) { CF.u = share; CF.at = sim_s(); }
    return thr > pedal ? thr : pedal;
}

static float cruise_body(void *ga, float dt, float pedal, float stock)
{
    char *g = (char *)ga;
    CF.at = -1;
    if (g) { if (p_ah > 0 && g_low_ok) auto_hold(g, dt); else { s_want_hold = 0; s_ah_take = 0; hill_start(g); if (layers_on()) disp_post(DISP_CRUISE, DISP_STATUS, sl_letter(), 0); } }
    if (p_smooth <= 0 || !g || !sense_cruise_on(g) || !(dt > 0 && dt < 0.5f)) {
        C.on = 0;
        s_cb_was_on = 0; s_layers_ran = 0; s_nm_ran = 0;
        s_bn_init = 0; s_bn_on = 0;
        g_sg_hold = 0;
        g_feed = 0;
        s_sg_wait = 0;   // no 'B' on the dash from a wait that the cruise drop ended
        return stock;
    }
    if (layers_on()) {   // the narrow gate: only the speed loop below steps aside (Auto Hold and Hill start ran above, the AEB, the pedal and the limiter run in cr_post)
        if (sl_lim_active()) { C.on = 0; s_cb_was_on = 0; g_sg_hold = 0; g_feed = 0; s_sg_wait = 0; return pedal; }   // LIM with the game's cruise on for the display: the throttle is the driver's
        {   // a vehicle ahead below the set (the game's ACC target, or ours), or the planner driving: neither has layers yet, so the classic loop drives this tick; it starts
            // bumpless from the throttle the layers had (the game's own PI output, `stock`, is not what the truck was given)
            // WP-SP3: the layers follow a vehicle ahead themselves (the lead model and the ACC's requests, cr_pre decided s_acc_lay): the classic loop drives only where they cannot, i.e. the vehicle
            // ahead has no known speed, the vehicle model cannot tell the force, own_acc is off (the game's ACC target is then the ACC), or the planner drives
            const float lead0 = lead_gap_m(g), acc0 = sense_acc_target(g), set0 = sense_cruise_set(g);
            planner_drive pd0;
            if (!(lead0 > 0.5f && acc0 >= 0 && acc0 < set0 && !s_acc_lay) && !planner_follow(&pd0)) return cr_layers_body(g, dt, pedal, stock);
            s_nm_ran = 0;   // (the classic loop drives: the model-less layers start over when they come back)
            if (s_layers_ran) {
                stock = fminf(1.0f, g_sa.share_prev * (CF.k > 0 ? CF.k : 1.0f)); C.on = 0; s_layers_ran = 0; s_nm_ran = 0;
                logv(1, "Cruise (layers): %s, the classic loop drives (%s)", lead0 > 0.5f && acc0 >= 0 && acc0 < set0 && !s_acc_lay ? "a vehicle ahead" : "the planner drives", lead0 > 0.5f && acc0 >= 0 && acc0 < set0 && !s_acc_lay ? "the layers cannot follow it: its speed is not known, the vehicle model cannot tell the force, or own_acc is off" : "the planner has no layers yet");
            }
        }
    }
    float set = sense_cruise_set(g), acc = sense_acc_target(g);
    if (!s_cb_was_on) s_cb_on_since = wall_s();
    s_cb_was_on = 1;
    float target = set;
    float kmh = fabsf(T.speed) * 3.6f;
    int md = cruise_mode();
    planner_drive pd; int pl_follow = planner_follow(&pd);   // [planner] drive=1: the plan's target drives the cruise (fresh, ok plan)
    // Predictive (route): momentum into a climb, ease off before a top.
    int eco = md == MODE_ECO, power = md == MODE_POWER;
    s_mom_up = 0;
    if (p_predict > 0 && route_ok()) {
        float v = fmaxf(fabsf(T.speed), 5.0f), up = band_up(), down = (eco ? 5 : 3) * band_scale(T.cruise) * crest_mass_k() * (power ? 0.5f : 1.0f);
        float g_now = route_grade(0, 50), g_up = route_max_grade(v * 3, v * 15);
        float crest = route_crest_m(v * 12);
        float after = crest >= 0 ? route_min_grade(crest, crest + v * 15) : NAN;
        if (after == after && after < -2) target = set - down;
        else if (g_up == g_up && g_now == g_now && g_up > 2 && g_up > g_now + 1) { target = set + up; s_mom_up = up; }
    }
    // A little over the set speed just before an eco-roll (TrueShift asks for it): the roll then takes the speed down to a little under it
    if (g_roll_boost_kmh > 0 && !s_lead_near && target <= set + 0.1f) target = set + g_roll_boost_kmh;
    // Speed limit following: the sign's limit (+offset), never above the set speed.
    // Bends ahead (route): the speed that takes every bend comfortably.
    float bend_kmh = NAN, jn_need = 0, jn_kmh = 0;
    s_bend_coast_kmh = NAN;
    if (p_curve > 0 && route_ok()) {
        float a_lat = bend_lat();  // same in every mode (Damien 11-10)
        // Like a human (Damien 11-10, first 0.6 ride: "braked oddly before a junction and very early before a ramp; later and
        // harder"): the throttle comes off early on a gentle plan (coast), the brakes only on a firm plan (bend_kmh below)
        // the coast deceleration the plan counts on depends on the slope (Damien 25-10: 83 -> 38 km/h without gas up a 2-4 % hill, 1217-1243 s):
        // gravity adds 0.098 m/s2 per % uphill and takes it away downhill
        float v_ms = fmaxf(fabsf(T.speed), 5.0f);
        float coast_a = fminf(1.0f, fmaxf(0.1f, p_cv_coast + 0.0981f * pitch_grade())), coast_b = coast_a;
        // bends only: junctions reach the throttle and the brake through vj below, after the gas-pedal, turn-signal and margin filters
        // (25-10: "braked without reason" at 2437 s was a 90-degree junction 600 m ahead planned here as a "bend", unfiltered)
        // the grade the truck coasts on is the road up to the bend, not the road under it now: plan with the mean grade over 8, 16 and 32 s of
        // road and take the lowest speed (a climb that ends before the bend does not count, a descent that goes on does)
        float vb = NAN;
        static const float k_horizons[] = { 8.0f, 16.0f, 32.0f };
        for (float sec : k_horizons) {
            float ca = coast_over(p_cv_coast, v_ms * sec, 0.1f, 1.0f), vi = route_bend_speed(a_lat, ca);
            if (vi == vi && (vb != vb || vi < vb)) { vb = vi; coast_b = ca; }
        }
        s_bend_coast_kmh = vb * 3.6f;
        float vb_brake = route_bend_speed(a_lat, p_cv_brake);
        static double said;
        if (vb == vb && vb * 3.6f < target - 5 && wall_s() - said > 10) {
            said = wall_s();
            logv(2, "Cruise: bend ahead: %.0f km/h (target %.0f, now %.0f)", vb * 3.6f, target, kmh);
        }
        route_junction_t ji = {};
        float vj = junc_on() ? route_junction_speed(coast_a, &ji) : NAN;
        float coast_j = coast_a;
        if (vj == vj) {   // again with the grade over the road to that junction
            coast_j = coast_over(p_cv_coast, ji.dist_m, 0.1f, 1.0f);
            vj = route_junction_speed(coast_j, &ji);
        }
        // You step on the gas for a junction we are slowing for: you go another way than the route (Damien 12-10: a petrol station with
        // 3-4 side entrances, the route turned there, he drove straight on and we braked to 20 for each). Leave junctions and roundabouts alone
        // for 6 s (Damien 12-10: first 12); the bends of the road itself still count.
        {
            if (vj == vj && T.in_thr > 0.3f && kmh < 60 && ji.dist_m < 250 && (kmh > ji.limit_ms * 3.6f - 5)) {
                if (wall_s() >= s_jn_ignore_until) logv(1, "Cruise: gas pedal at a junction ahead (%.0f m, %.0f\xc2\xb0 \xe2\x86\x92 %.0f km/h): junctions are left alone for 6 s", ji.dist_m, ji.turn_deg, ji.limit_ms * 3.6f);
                s_jn_ignore_until = wall_s() + 6;
            }
            if (wall_s() < s_jn_ignore_until) vj = NAN;
            // a gentle route turn without an indicator: you are going straight on (Damien 25-10: crossings, 548 s and 1455 s)
            if (vj == vj && p_jn_signal > 0 && !ji.roundabout && ji.turn_deg < p_jn_signal) {
                unsigned char fl = sense_lights(g);   // game_actor lights: bit 4 left, bit 8 right (both = hazards)
                bool signalling = (fl & 12) == 4 || (fl & 12) == 8;
                if (!signalling) vj = NAN;
            }
        }
        // Damien 13-10: we cannot know a good junction speed, so brake only down to it plus a margin, and a few seconds before the
        // junction cancel the cruise (sound) so the rest is the driver's. Like the game's own cancels only the on flag drops; the
        // set speed stays and Resume brings it back.
        float jn_orig_kmh = ji.limit_ms * 3.6f;
        // A prefab (junction, crossing, roundabout) close ahead that the route reading does not know (Damien 25-10, 1007 s: 49 km/h at 46 m of
        // a prefab, while the plugin only knew a roundabout 190 m further): say so, so the next ride shows what kind of prefab it is.
        // route_junction_m is the distance to the prefab's EXIT (25-10, 78 lines in two logs: all were the exit of a junction already read,
        // or a prefab we were already inside), so ask route.cpp whether the reading covers it instead of comparing it with an entry.
        {
            float jm = route_junction_m();
            static double said_u;
            if (jm >= 0 && jm < 150 && kmh > 35 && route_junction_known(jm) == 0 && wall_s() - said_u > 10) {
                said_u = wall_s();
                char jd[400]; route_junction_dump(jd, sizeof jd);
                logv(1, "Cruise: the prefab ending %.0f m ahead is not a known junction or roundabout (known: %s)", jm, jd);
            }
        }
        if (vj == vj && ji.limit_ms > 0 && p_jn_margin > 0) {
            float lo = ji.limit_ms, m = p_jn_margin / 3.6f;
            vj = sqrtf(fmaxf(0.0f, vj * vj - lo * lo) + (lo + m) * (lo + m));
            ji.limit_ms = lo + m;
        }
        if (vj == vj && ji.limit_ms > 0 && p_jn_cancel_s > 0 && !T.paused) {
            double now_c = wall_s();
            float t_to = fmaxf(ji.dist_m - (ji.roundabout ? 15 : 8), 0.0f) / fmaxf(kmh / 3.6f, 1.0f);
            // A roundabout is a hard rule (Damien 24-10: the cruise stayed on at one): no 5 s after switching on / 20 s between two cancels,
            // only 1.5 s and 3 s (a second roundabout right after the first, or a Resume still on the approach).
            bool free_to_cancel = ji.roundabout ? (now_c - s_cb_on_since > 1.5 && now_c - s_jn_cancel_at > 3)
                                                : (now_c - s_cb_on_since > 5 && now_c - s_jn_cancel_at > 20);
            bool slowed = kmh <= jn_orig_kmh + p_jn_margin + p_jn_hand_kmh || ji.dist_m <= p_jn_hand_m;  // handed over when slowed down or close (Damien 25-10: later, slower)
            // A junction is only cancelled while the truck is still faster than its speed + 3 (slower = you are at it already). A roundabout
            // also behind a vehicle the ACC has slowed us for (Damien 26-10: the cruise stayed on behind a lead): no speed floor there, but
            // not under 5 km/h and not while Stop&Go holds the standstill in a queue before the entrance; once the truck drives again it cancels.
            bool driving = ji.roundabout ? (kmh >= 5 && !g_sg_hold) : kmh > jn_orig_kmh + 3;
            if (t_to <= p_jn_cancel_s && driving && free_to_cancel && slowed) {
                s_jn_cancel_at = now_c;
                if (ji.roundabout) g_rb_cancel_at = now_c;
                act_cruise_on(g, 0);
                play_cue(CUE_CRUISE_DROP);
                logline("Cruise: %s in %.0f m (%.1f s) at %.0f km/h, its speed %.0f (+%.0f margin): cruise cancelled for you, Resume brings the set speed back",
                        ji.roundabout ? "roundabout" : "junction", ji.dist_m, t_to, kmh, jn_orig_kmh, p_jn_margin);
                return stock;
            }
        }
        static double said_j;
        // Log as soon as a junction/roundabout limit is well under our speed and close (<= 5 s apart).
        if (vj == vj && ji.limit_ms * 3.6f < fminf(target, kmh) - 5 && ji.dist_m < 400 && wall_s() - said_j > 5) {
            said_j = wall_s();
            if (ji.roundabout)
                logv(2, "Cruise: roundabout ahead \xe2\x86\x92 %.0f km/h (in %.0f m, %d curves, path x%.1f; target %.0f, now %.0f)",
                        ji.limit_ms * 3.6f, ji.dist_m, ji.curves, ji.ratio, target, kmh);
            else
                logv(2, "Cruise: junction ahead: %.0f\xc2\xb0 \xe2\x86\x92 %.0f km/h (in %.0f m; target %.0f, now %.0f)",
                        ji.turn_deg, ji.limit_ms * 3.6f, ji.dist_m, target, kmh);
        }
        // Deceleration still needed to be at the junction speed at the entry: coasting gives only ~0.05-0.2 m/s2 on a
        // loaded truck (ts4i: 0.03), so the brake request has to supply the rest.
        jn_need = 0;
        if (vj == vj && ji.limit_ms > 0 && kmh / 3.6f > ji.limit_ms + 0.8f) {
            float v = kmh / 3.6f, dd = ji.dist_m - (ji.roundabout ? 15 : 8);
            jn_kmh = ji.limit_ms * 3.6f;
            jn_need = (v * v - ji.limit_ms * ji.limit_ms) / (2 * fmaxf(dd, 6.0f));
        }
        if (vb == vb) target = fminf(target, ahead_cap(v_ms, vb, coast_b) * 3.6f);
        if (vb_brake == vb_brake) bend_kmh = vb_brake * 3.6f;  // the brakes (below) follow the firm plan
        // the junction profile is a target too (ts4w 532-553 s: no lead, set 83, braking 0.1-0.15 for a 95° turn while
        // our throttle sat at 1.0, because only the brake knew about the junction)
        if (vj == vj && vj > 0) target = fminf(target, ahead_cap(v_ms, vj, coast_j) * 3.6f);
        // and no gas toward a junction or roundabout we have to slow down for anyway: the plan above allows speeds above ours far
        // from the entry, so the throttle came on and the brakes right after (Damien 12-10: "gas, then brake")
        // from 400 m (30 s) out, not 150 m: entering a town at 42 km/h the throttle came on towards a roundabout 400 m ahead (35 km/h plan) up to 51 km/h,
        // the brakes and the cancel only 75 m before it (24-10, 1688-1713 s: "veel te hard")
        float jn_gate = fmaxf(400.0f, 30 * kmh / 3.6f);
        if (vj == vj && ji.limit_ms > 0 && kmh > ji.limit_ms * 3.6f + 1 && ji.dist_m < jn_gate) {
            // Gas only while the truck can still coast down to the junction speed (not a blanket no-gas from 400 m: 25-10, two roundabouts
            // close together on a 70 road: 30 km/h all the way) and never faster than the junction speed + junction_approach_over_kmh
            float jn_coast = coast_over(p_jn_coast, ji.dist_m, 0.05f, 0.8f);
            float v_env = sqrtf(ji.limit_ms * ji.limit_ms + 2 * jn_coast * fmaxf(ji.dist_m - (ji.roundabout ? 15 : 8), 0.0f));
            float v_ok = fminf(ahead_cap(v_ms, v_env, jn_coast) * 3.6f, ji.limit_ms * 3.6f + p_jn_over);
            // (the last 2 km/h under v_ok are not chased: the target is v_ok - 2 there and the truck's own speed from it up. It was
            // `kmh >= v_ok - 2 ? kmh : v_ok`: a jump of 2 km/h in the error (0.5 throttle) at the edge, which tipped a climb run from 3 to 4 s at
            // full throttle on a 0.001 change of ahead_accel)
            // (faded in over the first 40 m inside the gate: at the gate itself the target stepped by ~3 km/h, 27-10)
            float jn_cap = fmaxf(kmh, v_ok - 2), jn_w = fminf(1.0f, (jn_gate - ji.dist_m) / 40.0f);
            if (jn_cap < target) target += jn_w * (jn_cap - target);
        }
    }
    // The ACC's own target (vehicle ahead) always wins, also 0 = stop behind it. But someone cutting in or a
    // car that pulls away (the gap is opening) is no reason to slow down (Damien): then only hold the speed.
    float lead = lead_gap_m(g);
    s_lead_near = lead > 0.5f && lead < fmaxf(80.0f, 4 * fabsf(T.speed));
    float gap_rate = lead_rel_speed(g, lead, dt);  // m/s, + = the gap opens: the lead's speed from the radar, else d(gap)/dt
    if (!pl_follow) {
        // opening: coast at most. A weight, not a switch (27-10: at 0.3 m/s the target jumped from the ACC's to our speed, 15 km/h in a probe):
        // in from 0.3 to 0.6 m/s, so the coasting comes a little later than before, never earlier
        if (lead > 0.5f && acc >= 0 && acc < target) {
            float w_coast = p_acc_calm > 0 ? ramp01(gap_rate, 0.3f, 0.6f) * ramp01(lead, 8.0f, 9.0f) : 0.0f;
            target = acc + w_coast * (fmaxf(acc, fminf(target, kmh)) - acc);
        }
    }
    // [planner] drive=1: the plan's target (speed ~2 s ahead) replaces the composed target and the ACC's comfort override (the
    // planner follows the lead itself); a stale or failed plan falls back to the blocks smoothly, no jump in the target.
    if (pl_follow) {
        // (review 30-10: the plan refreshes at 4 Hz and its target steps as caps enter the horizon: blended over 0.5 s, every refresh,
        // so the PI does not hunt at 4 Hz; on taking over it starts from the blocks' target of this tick)
        if (!s_pl_was) s_pl_tgt = target;
        if (dt > 0) s_pl_tgt += (pd.v_target_ms * 3.6f - s_pl_tgt) * fminf(1.0f, dt / 0.5f);
        target = s_pl_tgt;
    } else if (s_pl_was && dt > 0) {
        target = s_pl_tgt + (target - s_pl_tgt) * fminf(1.0f, dt / 0.5f);
        s_pl_tgt = target;
        static double said_fb;
        if (wall_s() - said_fb > 5) { said_fb = wall_s(); logv(1, "Cruise: planner handed back (%s), back to the cruise's own blocks at %.0f km/h", pd.on ? "stale or failed plan" : "planner off", target); }
    }
    s_pl_was = pl_follow;
    // No gas while the wheel is turned (Damien 12-10: on a roundabout it gave gas with the wheel still round): hold the speed
    // until it is straight again (+ 0.8 s)
    {
        float st_abs = fabsf(T.in_steer);  // a keyboard or gamepad tap sits at full lock: only an analog wheel counts
        if (st_abs > (kmh > 60 ? 0.25f : 0.15f) && st_abs < 0.95f) s_steer_until = wall_s() + 0.8;
        if (wall_s() < s_steer_until && kmh > 8 && target > kmh) target = kmh;
    }
    // The bend the truck is IN (ride 8, 27-10, 1176-1187 s: bend ahead 45, then 47 -> 58 km/h inside a radius 40-65 m bend, lateral peaks 5.3): the route
    // plan above only looks ahead, and the ACC target ignores the road. Path curvature = yaw rate / speed (lane o3 is not needed: the yaw is there
    // with the lane assist off too), filtered over 0.5 s so a lane change does not count, the peak held with a 1.5 s decay (an S-bend, the bend's
    // end: the cap lets go gently). The cap is the speed curve_lat allows for that radius, the same allowance the bend plan uses. Only a clear bend
    // (curvature over 0.004 = radius under 250 m; a motorway curve at 90 km/h stays under 1.5 m/s2 at 0.0024), with hysteresis (off at 0.7x).
    if (p_bend_now > 0 && T.speed > 0 && kmh >= 20) {
        float k_in = fabsf(T.yaw) * 6.2831853f / fmaxf(T.speed, 5.0f);
        if (!s_bn_init) { s_bn_f = s_bn_pk = k_in; s_bn_init = 1; }
        else { s_bn_f += (k_in - s_bn_f) * fminf(1.0f, dt / 0.5f); s_bn_pk = fmaxf(s_bn_f, s_bn_pk * expf(-dt / 1.5f)); }
        int was = s_bn_on;
        s_bn_on = s_bn_pk > (was ? 0.0028f : 0.004f);
        if (s_bn_on) {
            float vc = sqrtf(bend_lat() / s_bn_pk);
            for (int it = 0; it < 3; it++) vc = sqrtf(bend_lat() * bn_lat_scale(vc) / s_bn_pk);
            float cap_kmh = fmaxf(vc * 3.6f, 15.0f);
            int capping = cap_kmh < fmaxf(target, kmh) + (s_bn_cap ? 2 : -1);   // (it only says so once it really holds the truck back)
            if (capping && !s_bn_cap) s_bn_cap_at = wall_s();
            // said 0.6 s after it starts holding (the curvature filter has settled by then: the radius is the bend's)
            if (capping && s_bn_cap_at > 0 && wall_s() - s_bn_cap_at > 0.6 && wall_s() - s_bn_said > 5) {
                s_bn_said = wall_s(); s_bn_cap_at = 0;
                logv(2, "Cruise: in a bend: speed capped at %.0f km/h (radius %.0f m; target %.0f, now %.0f)", cap_kmh, 1.0f / s_bn_pk, target, kmh);
            }
            s_bn_cap = capping;
            target = fminf(target, cap_kmh);
            bend_kmh = bend_kmh == bend_kmh ? fminf(bend_kmh, cap_kmh) : cap_kmh;   // over it: the same gentle bend braking as for a bend ahead
        } else s_bn_cap = 0;
    } else { s_bn_init = 0; s_bn_on = 0; s_bn_cap = 0; }
    s_target_dbg = target;   // (the sims read the target before the controller turns it into a throttle)
    float e = target - kmh;
    // Overshoot (Damien 13-10: a resume from 68 to 90 gave full throttle for 12 s into a descent, 92.5 km/h): the P term looks at the
    // error we will have in approach_s seconds at the speed we gain now, so the throttle eases off before the set speed is reached.
    float ep = e;
    {
        float acc_kmhs = T.accel * 3.6f;
        if (!s_acc_init || !C.on) { s_acc_f = acc_kmhs; s_acc_init = 1; }
        else s_acc_f += (acc_kmhs - s_acc_f) * fminf(1.0f, dt / 0.5f);
        float tau = p_approach_s * (md == MODE_ECO ? 1.2f : (md == MODE_POWER ? 0.6f : 1.0f));
        if (tau > 0 && s_acc_f > 0) ep = e - tau * s_acc_f;
    }
    // Too fast for a bend and the throttle is already off: a gentle brake request
    // (game_actor+0x17c, 0..1; the game slews it and uses engine brake/retarder first).
    // with hysteresis (start 3 km/h over, keep until 0.5 under): no on/off pumping of the brakes.
    // (planner drive: the plan's decel goes through the brake channel instead; the in-bend cap above still clamps the target)
    s_curve_braking = !pl_follow && p_curve_brake > 0 && bend_kmh == bend_kmh && kmh > bend_kmh + (s_curve_braking ? -0.5f : 3.0f);
    if (s_curve_braking) {
        // gentle, but firmer when far too fast (Damien: roundabouts still too fast, it could not slow down enough)
        if (brake_channel_on()) {   // channel: the firm plan's own deceleration plus 0.3 m/s2 per km/h over it, back onto the plan
            s_ch[BRQ_BEND] = fminf(kmh > bend_kmh + 8 ? 3.5f : 2.2f, fmaxf(0.6f, p_cv_brake + 0.3f * (kmh - bend_kmh + 0.5f)));
            s_chu[BRQ_BEND] = BRU_FIRM;
        } else
        s_want_curve = fminf(kmh > bend_kmh + 8 ? 0.4f : 0.2f, fmaxf(0.04f, (kmh - bend_kmh + 0.5f) / 25));
    }
    // Junction / roundabout ahead and the profile cannot be kept by coasting: brake at least as hard as the
    // distance left needs (0.2 at 0.35 m/s2, 0.4 at 2 m/s2), also under the 3 km/h hysteresis above.
    // brake once coasting along the planned profile is not enough (threshold follows the plan: 0.35 at the old 0.4)
    float jn_thr = p_jn_dec - 0.05f;
    // With a hysteresis (27-10, a plant braking like the game's: 15 on / off cycles of the request on one approach): the 0.2 request is far more
    // than the 0.35 m/s2 it started for, the need fell under the threshold within a second, the request let go, the need rose again. On at the
    // threshold, off only when the need is 0.35 m/s2 under it (the request then still follows the need, 0.16 there, so it does not step to 0 from a high value).
    s_jn_braking = !pl_follow && p_curve_brake > 0 && jn_need > jn_thr - (s_jn_braking ? 0.35f : 0.0f) && kmh > jn_kmh + (s_jn_braking ? 0.0f : 3.0f);
    if (s_jn_braking) {
        if (brake_channel_on()) {   // channel: the deceleration the distance left needs (it is recomputed every tick: it closes the loop) and a margin
            s_ch[BRQ_JUNCTION] = fminf(3.5f, fmaxf(0.3f, jn_need + 0.1f));
            s_chu[BRQ_JUNCTION] = jn_need > p_jn_dec + 0.5f ? BRU_FIRM : BRU_COMFORT;
        } else
        s_want_curve = fmaxf(s_want_curve, fminf(0.45f, 0.2f + 0.12f * (jn_need - jn_thr)));
    }
    // a descent the retarder and the engine brake cannot hold (1678-1692 s, -9 %: set 63, 70 km/h; Damien 25-10: the service brake may help):
    // over the set speed plus the descent allowance and not slowing: a gentle brake request that grows with the overspeed
    // over what the cruise itself grants (set, or a higher target from a dip, momentum or a roll boost), with hysteresis: it starts when
    // the truck is not slowing and holds until it is clearly slowing or back inside the allowance, so its own braking does not switch
    // it off every second (pulsing). Not while rolling in neutral (the roll ends on a brake). (planner drive: the plan's decel instead)
    // (review 30-10: also under the planner: the descent hold and the limit line below are the 'never above limit + 5' backstops the
    // plan, recomputed at 4 Hz with a modelled grade and mass, does not guarantee by itself)
    if (s_down_now && T.cruise > 0.5f && !g_rolling) {
        // (the allowance at least what the game's own overspeed braking grants now: a dip lets it run up to the band, and the service
        // brake came in 2 km/h over the descent margin, under the retarder's point: PCC sim 29-10, up to 7 MJ of brake per 10 km)
        float hold = cruise_hold_kmh(NAN);
        float allow = fmaxf(fmaxf(0.0f, p_desc_kmh) * band_scale(T.cruise), hold == hold ? hold : 0.0f) + 2.0f, over = kmh - (fmaxf(set, target) + allow);
        if (brake_channel_on()) {   // channel: asks a deceleration that brings it back into the allowance (0 at 1 km/h under it, where it lets go)
            s_desc_braking = s_desc_braking ? over > -1.0f : (over > 0 && T.accel > -0.05f);
            if (s_desc_braking) { s_ch[BRQ_DESCENT] = fminf(1.5f, fmaxf(0.05f, 0.1f * (over + 1.0f))); s_chu[BRQ_DESCENT] = BRU_COMFORT; }
        } else {
        s_desc_braking = s_desc_braking ? (over > -1.0f && T.accel > -0.15f) : (over > 0 && T.accel > -0.05f);
        if (s_desc_braking) s_want_curve = fmaxf(s_want_curve, fminf(0.4f, 0.06f * fmaxf(over, 0.5f)));
        }
    } else s_desc_braking = 0;
    // The fine line (Damien: never more than overspeed_cap_kmh over the limit, whatever the margins): the game's retarder builds up
    // over ~1 s and may not hold a heavy truck on a steep descent at all, so the service brake keeps the speed under limit + cap,
    // starting early enough for its own build-up (the game slews the request at 5/s): from the speed 1.5 s ahead at the acceleration
    // of now, asking what stops the acceleration plus a share of the overspeed (sim 30-10 with the game's real braking: up to
    // +10.8 km/h over the cap on descents, 337 of 486 runs over it).
    {
        // Only when gravity is taking the truck over the line, never while the cruise is pulling (review 30-10): on the level under
        // full throttle the throttle side (overspeed_room on the target) already keeps it under the line, and this brake fired then.
        // The road counts as going down from the placement's pitch or the route's grade just ahead.
        float ga = route_grade(0, 50);
        int pushing = T.eff_thr < 0.05f || pitch_grade() < -0.5f || (ga == ga && ga < -0.5f);
        float room = overspeed_room(T.cruise);
        if (pushing && room < 1e8f && T.cruise > 0.5f && !g_rolling) {
            float cap = set + room, acc_up = fmaxf(0.0f, T.accel), soon = kmh + acc_up * 3.6f * 1.5f, over_c = soon - (cap - 0.5f);
            if (brake_channel_on()) {   // channel: a deceleration that brings the speed back under the line, held until 1 km/h under it
                // (the projection with no more acceleration than the slope can give: a spike of the measured acceleration -- a teleport, a
                // hitch -- asked an urgent 2.5 m/s2 for a moment on the level in sim_pcc's start)
                float gd = fmaxf(0.0f, -fminf(pitch_grade(), ga == ga ? ga : 0.0f));
                float acc_c = fminf(acc_up, 0.3f + 9.81f * gd / 100.0f);
                over_c = kmh + acc_c * 3.6f * 1.5f - (cap - 0.5f);
                s_lim_braking = s_lim_braking ? over_c > -1.0f : over_c > 0;
                if (s_lim_braking) { s_ch[BRQ_LIMIT] = fminf(2.5f, fmaxf(0.05f, 0.15f * (over_c + 1.0f))); s_chu[BRQ_LIMIT] = BRU_URGENT; }   // (urgent: a hard line, no tolerance for the retarder)
            } else
            if (over_c > 0) {
                float need = acc_up + 0.15f * over_c;   // m/s2
                s_want_curve = fmaxf(s_want_curve, fminf(0.6f, need / vehicle_brake_cautious() + 0.02f));   // (the request for the deceleration through the model's cautious gain: the fit up to 30 t, the same force above, for weak brakes what they measured)
            }
        } else s_lim_braking = 0;
    }
    float kp = 0.25f, ki = 0.08f, rate = 0.6f;               // Normal / M (per km/h, per km/h·s, per s)
    if (md == MODE_ECO) { kp = 0.18f; ki = 0.05f; rate = 0.4f; }        // Eco: gentle, lets it sag a bit
    else if (md == MODE_POWER) { kp = 0.35f; ki = 0.12f; rate = 1.0f; }  // Power: keen
    // force_mode 1: the output in force units wherever the model can convert it; a moment it cannot (a shift, the rev limit) keeps the unit
    // and the gear of a moment ago. The gains and the rate are the same per throttle in the gear of now, so between two shifts it drives as
    // before; across a shift it keeps the force, not the throttle. A change of unit under a running controller keeps its throttle
    int fm = 0; float k = 1;
    if (p_force_mode > 0) {
        float kn = T.eff_clutch > 0.05f && kmh > 2 ? 0 : force_k(kmh / 3.6f);   // (in a shift: the gear before it)
        fm = kn > 0 || (CF.fm && C.on);
        if (fm) k = kn > 0 ? kn : CF.k;
    }
    if (fm != CF.fm && C.on) { float c = fm ? 1 / k : CF.k; C.u *= c; C.i *= c; }
    CF.fm = fm; CF.k = k;
    if (fm) { kp /= k; ki /= k; rate /= k; }
    // The gains were fitted on trucks, whose full throttle gives them well under 1 m/s2. A car's gives 2-5: the same kp per km/h is several times the
    // loop gain, and with the controller's 3 s look-ahead and the converter's lag it limit-cycled (ride 31-10 21:20, the Mustang set 80: the speed
    // 71-78 and the throttle 0-0.9 every 6 s for 35 s; the Crown Victoria's 0-0.8 on the climb). On a light vehicle the gains are scaled by the
    // acceleration a full throttle gives in the gear of now against 1.2 m/s2 (1 at most; held through a shift)
    {
        static float s_car_sc = 1;
        if (!vprof()->gain_by_accel) s_car_sc = 1;
        else if (T.eff_clutch <= 0.05f) {
            int gg = T.gear;
            float fg = gg >= 1 && gg <= T.fwd_gears && V.eng_ok && !V.electric && V.mass > 0 ? vehicle_force_at(gg, fmaxf(0.5f, fabsf(T.speed)), 1.0f) : 0;
            if (fg > 0) s_car_sc = fminf(1.0f, fmaxf(0.35f, 1.2f / (fg / V.mass)));
        }
        kp *= s_car_sc; ki *= s_car_sc;
    }
    if (!C.on) { C.on = 1; C.u = fm ? stock / k : stock; C.i = C.u - kp * ep; }   // bumpless start
    // at a standstill the open clutch is no shift: the throttle has to rise for Stop&Go to drive off (ts4r 189-193 and
    // 227-231 s: the lead pulled away, our throttle stayed frozen at 0.03, Damien had to give gas)
    int shifting = T.eff_clutch > 0.05f && kmh > 2;
    if (g_rolling) {   // in neutral: no throttle, keep the engine at idle; the game must not cancel cruise (ts3b)
        C.u = 0; C.i = -kp * e;
        if (g_sg_ok) { g_sg_hold = 1; g_feed = 0; }
        return pedal;
    }
    float u_raw = kp * ep + C.i;
    // [planner] drive: the plan's traction is a feed-forward (a share of the full wish), the PI only trims. Force mode only: in force
    // units the plan's wheel force is gear-independent, so a mismatch of the planner's gear stand-in against TrueShift's real gear does not
    // change the pull asked (the throttle in the gear of now follows through k).
    {   // (review 30-10: the feed-forward low-passed over 0.5 s as the target, so a refresh does not step the pull)
        static float s_pl_ff;
        float ff_in = pl_follow && fm && pd.act == PL_GAS && pd.force > 0 ? pd.force : 0;
        s_pl_ff = pl_follow ? s_pl_ff + (ff_in - s_pl_ff) * fminf(1.0f, dt > 0 ? dt / 0.5f : 1.0f) : 0;
        float full = s_pl_ff > 0 ? wish_full(kmh / 3.6f) : 0;
        if (full > 0) u_raw += fminf(1.0f, s_pl_ff / full);
    }
    // Following in a jam: gentle throttle at walking pace (ts4r 296-313 s, toll queue: full throttle / off every
    // second at 4-5 km/h against our own brake pulses; Damien: "full gas, gas off, full gas"). 0.4 from rest up to 1.0
    // at 18 km/h; and no wind-up while our ACC brakes.
    float u_cap = lead > 0.5f && acc >= 0 && acc < set ? fminf(1.0f, 0.4f + kmh / 30) : 1.0f;
    if (u_cap < 1.0f) {   // a climb: raise the gentle floor so a heavy truck stopped behind a lead on a grade still pulls away after it
                          // (hill-report 31-10): the grade alone may need most of the launch pull, then it gets the throttle
        int g = T.gear;
        float fg = g >= 1 && g <= T.fwd_gears && V.eng_ok && !V.electric ? vehicle_force_at(g, vehicle_kmh_at(g, ts_idle_rpm()) / 3.6f, 1.0f) : 0;
        if (fg > 0) {
            float hold = fmaxf(0.0f, vehicle_resist(fabsf(T.speed))) / fg;   // the throttle share that just holds the grade (idle: the launch pull)
            if (hold > 0.35f) u_cap = 1.0f;
        }
    }
    if (md == MODE_ECO && p_eco_cap < 1 && kmh > target - 4) u_cap = fminf(u_cap, p_eco_cap);  // Eco: a calmer cruise throttle (fuel), lifted when well under the speed
    if (p_app_mode > 0 && !(lead > 0.5f && acc >= 0 && acc < set) && e > 2.5f) {
        float uc = approach_ceiling(md, e, s_acc_f, dt, shifting);
        u_cap = fminf(u_cap, uc);   // (the integrator waits while the output is held down, as for any cap)
    } else s_app_ex = 0;
    if (s_want_acc > 0.04f || s_ch[BRQ_ACC] > 0 || (pl_follow && pd.act != PL_GAS)) { u_cap = 0; if (C.i > 0) C.i = 0; }   // (planner drive: only PL_GAS pulls; coast/brake/neutral are the plan's fuel cut or brake)
    // (force_mode 1: a cap is the same throttle cap in this gear; without one the force may ask more than this gear gives at full throttle, up
    // to a full wish: TrueShift reads that demand, the throttle stays at 1)
    if (fm) u_cap = u_cap < 1 ? u_cap / k : fmaxf(1.0f, 1 / k);
    float u = u_raw < 0 ? 0 : (u_raw > u_cap ? u_cap : u_raw);
    float step = rate * dt;
    if (shifting) step = 0;                                  // hold during the shift
    if (u > C.u + step) u = C.u + step;
    if (u < C.u - 2 * step) u = C.u - 2 * step;              // back off twice as fast
    // Integrate only while the output can follow (not shifting, not held by the
    // limits or the rate limit in the same direction): no wind-up.
    int held_up = (u < u_raw - 1e-4f || (fm && u * k > 1)) && e > 0, held_down = u > u_raw + 1e-4f && e < 0;
    // integrate the same look-ahead error as P: with e the integrator kept filling during the approach and held the throttle up
    // after P had eased it (toy model 13-10: +1.5 instead of +0.6 on the flat)
    if (!shifting && !held_up && !held_down) C.i += ki * ep * dt;
    float i_hi = fm ? fmaxf(1.0f, 1 / k) : 1, i_lo = fm ? -0.5f / k : -0.5f;   // (force_mode: the same throttle limits, or a full wish)
    if (C.i > i_hi) C.i = i_hi;
    if (C.i < i_lo) C.i = i_lo;
    C.u = u;
    float thr = fm ? fminf(1.0f, u * k) : u;   // the throttle that gives it

    // Stop&Go bookkeeping (1.60 fields; only when its patches are in).
    if (g_sg_ok) {
        float lead = lead_gap_m(g);
        // with the low-speed owner: the hold is due from the owner's zone (the engine near the stop threshold, before the box can go to
        // neutral and the game cancels the cruise), the wait for stop_go_auto_s counts from the owner's standstill
        int own = low_owner_on();
        int slow = own ? low_sg_zone() : kmh < 3, stopped = own ? low_stopped() : slow;
        double now = wall_s();
        if (!stopped) g_stopped_at = 0;
        else if (!g_stopped_at) g_stopped_at = now;
        int may_wait = p_sg_auto <= 0 || (g_stopped_at ? now - g_stopped_at < p_sg_auto : own);
        g_sg_hold = p_sg > 0 && slow && lead > 0.5f && may_wait && !T.parking_brake;
        s_sg_wait = p_sg > 0 && p_sg_auto > 0 && slow && g_stopped_at && !may_wait && !T.parking_brake;
        g_feed = (p_sg > 0 && kmh < 10 && (g_sg_hold || g_stopped_at) && thr > 0.02f) ? thr : 0;
    }
    // Held back by something else (ts4n 433-448 s: brake air low → the game's spring brakes stopped the truck while our
    // cruise gave full throttle for 14 s, engine racing in neutral): low air, or full throttle without moving for 2 s
    // → our throttle 0 and the integrator reset, until the truck moves again.
    {
        int air_bad = T.air_warning || (T.air_psi > 1 && T.air_psi < air_pause_psi());
        int stuck = thr > 0.5f && kmh < 2 && T.in_thr < 0.02f;
        if (!stuck) s_stuck_since = 0;
        else if (!s_stuck_since) s_stuck_since = wall_s();
        if (air_bad || (s_stuck_since && wall_s() - s_stuck_since > 4)) { C.u = 0; C.i = 0; g_feed = 0; u = thr = 0; }
    }
    // (CF.to: where the controller is heading, its output before the rate limit: the gearbox chooses the gear for the pull the cruise is making, not for the first tenth of it)
    if (fm && thr > pedal) { CF.u = u; CF.to = (lead > 0.5f && acc >= 0 && acc < set) ? u : (u_raw < 0 ? 0 : (u_raw > u_cap ? u_cap : u_raw)); CF.at = sim_s(); }   // (behind a lead the throttle is the ACC's own, a following: no pull to look ahead to)
    return thr > pedal ? thr : pedal;
}

static void *g_acc, *g_smart;
// Where the game's own overspeed braking starts now (km/h; cruise-re notes § 2): A = max(T + smart - 0.25, T + 0.25), T = the ACC target
// +0x1a0 while ACC is on (g_acc > 0; the sims without the cvar: a lead is seen), else the set speed; smart off (< 0) = none, but 1.0 with
// ACC on. NAN = it does not act (cruise off, the driver's throttle pressed, smart unknown).
static float cr_game_brake_from(char *g, float pedal)
{
    if (!g || !sense_cruise_on(g) || pedal > 0) return NAN;
    float sm = g_smart ? sense_cvar_get(g_smart) : NAN, acc = g_acc ? sense_cvar_get(g_acc) : NAN;
    int acc_on = acc == acc ? acc > 0 : lead_gap_m(g) > 0.5f;
    if (sm != sm) return NAN;
    if (sm < 0 && acc_on) sm = 1.0f;
    if (sm < 0) return NAN;
    float t = acc_on ? sense_acc_target(g) : sense_cruise_set(g);
    return fmaxf(t + sm - 0.25f, t + 0.25f);
}
static float cr_game_target(char *g)
{
    if (!g || !sense_cruise_on(g)) return NAN;
    float acc = g_acc ? sense_cvar_get(g_acc) : NAN;
    int acc_on = acc == acc ? acc > 0 : sense_lead_m(g) > 0.5f;
    return acc_on ? sense_acc_target(g) : sense_cruise_set(g);
}
// (the engine brake's descent control reads these later in this tick, modules/ebrake.cpp. Only with the overspeed setting on: the game cancels
// the cruise when the engine brake comes on with it off, and its own rules then differ from what the control assumes)
static void cr_view_update(char *g, float pedal)
{
    float sm_raw = g && g_smart ? sense_cvar_get(g_smart) : NAN;
    s_view_from = g && sm_raw >= 0 ? cr_game_brake_from(g, pedal) : NAN;
    s_view_target = s_view_from == s_view_from ? cr_game_target(g) : NAN;
}
// Where the game's own overspeed braking starts and the speed it holds (km/h), as of this tick's Cruise hook (ACT_POST); 0 = none now
// (cruise off, the driver's throttle pressed, the overspeed setting off).
int cruise_overspeed_view(float *from_kmh, float *target_kmh)
{
    if (!(s_view_from == s_view_from) || !(s_view_target == s_view_target)) return 0;
    *from_kmh = s_view_from; *target_kmh = s_view_target;
    return 1;
}
static float g_smart_player = NAN, g_smart_ours = NAN;
// The game's overspeed number (the 'smart' cruise option, which the dashboard shows): one fact, the distance to the fine line.
//  - cruise OFF: how far the set speed is from the fine line (the limit plus overspeed_cap_kmh, core/speedtarget.h fine_room_kmh), at most overspeed_room_max_kmh when that is set; no limit known: overspeed_cruise_kmh.
//  - cruise ON: overspeed_cruise_kmh (a flat distance: on a 40 km/h set speed on an 80 road the retarder holds near 40, not 85), and the same clamp; the fine line caps it where it is written (overspeed_room).
// The old base (smart_overspeed_kmh 8 x band_scale x a mode factor) is gone: a distance to a line is not a share of 70 km/h, and the mode has no reason to move a fine line.
// The player's own game setting (overspeed_cruise_kmh = -1) is left alone.
static float smart_base()
{
    if (p_os_cruise < 0) return g_smart_player;
    float b = p_os_cruise;
    if (!(T.cruise > 0.5f)) { const float room = fine_room_kmh(T.cruise); if (room == room) b = fmaxf(room, 0.0f); }
    if (p_os_room_max > 0) b = fminf(b, p_os_room_max);
    return b;
}
// The smart overspeed on the flat and gentle descents is not a fixed number (Damien 25-10):
// A grade: the full value down to -1 %, gliding to the steep-descent value at -2 % (no jump);
// B route: a bend or junction within ~300 m that asks for less than we run: no extra speed at all, so the engine brake holds the set speed;
// C weight: a heavy trailer rolls on, so up to 0.5 km/h less at 40 t (linear from 20 t).
// The player's own game setting (overspeed_cruise_kmh = -1) is left alone.
static float smart_dynamic(float gp)   // (gp: the grade it is for, %)
{
    float base = smart_base();
    if (p_os_cruise < 0 || !(T.cruise > 0.5f)) return base;   // (no cruise: the number is only shown, no refinement of the road applies)
    float desc = fmaxf(0.0f, p_desc_kmh) * band_scale(T.cruise);
    float f = fminf(1.0f, fmaxf(0.0f, -gp - 1.0f));
    float w = base + (fminf(desc, base) - base) * f;
    float jn = route_ok() && junc_on() ? route_junction_m() : -1;
    if (route_ok() && ((s_bend_coast_kmh == s_bend_coast_kmh && s_bend_coast_kmh < T.cruise * 3.6f + 3) || (jn >= 0 && jn < 300))) w = 0;
    w -= 0.5f * (V.mass_ok ? fminf(1.0f, fmaxf(0.0f, (load_mass() - 20000) / 20000)) : 1.0f);
    return fmaxf(0.0f, w);
}
static float g_acc_player = NAN;  // the player's own setting, restored on exit
static float g_acc_ours = NAN;
static float s_press_before = NAN;   // ACC distance before the last key press (a long press undoes it)
static double s_press_at;
static int s_btn_held;               // the speed-limit button is down: the distance is announced on release

// Runs only when the ACC distance key is pressed (the handler reads g_acc to
// pick the next distance): put the player's distance back first, so the key
// steps 40 -> 30 -> 20 -> off from there and not from our computed value.
static float acc_read_stub(void *cvar)
{
    if (cvar == g_acc) {
        s_press_before = g_acc_player == g_acc_player ? g_acc_player : sense_cvar_get(g_acc);
        s_press_at = wall_s();
    }
    if (cvar == g_acc && g_acc_ours == g_acc_ours && g_acc_player == g_acc_player) {
        act_cvar_set(NULL, g_acc, g_acc_player);
        g_acc_ours = NAN;
    }
    float g = o_acc_read(cvar);
    // Damien: a press goes further away: off -> close -> middle -> far -> off (the game goes the other way:
    // far -> middle -> close -> off). The handler maps what it reads: > 30 -> 30, > 20 -> 20, > 0 -> 0, else 40,
    // so we hand it the value that makes it step the way we want.
    if (cvar == g_acc && p_acc_up > 0 && g == g) {
        if (g < 1) return 25;        // off   -> 20 close
        if (g < 25) return 35;       // close -> 30 middle
        if (g < 35) return 0;        // middle -> 40 far
        return 10;                   // far   -> 0 off
    }
    return g;
}

static int g_level = 1;  // -1 off, 0 short, 1 middle, 2 long (from the player's g_acc)
static const char *const level_name[] = { "off", "short", "middle", "long" };

static int level_of(float g) { return g < 1 ? -1 : g < 25 ? 0 : g < 35 ? 1 : 2; }

// cruise_state.txt: what the player last chose, kept over restarts (Damien 11-10: ISA, and the ACC distance like the
// gearbox mode). The profile's g_acc cannot carry it: we write our own non-round values there while driving.
static int s_isa_ini, s_acc_saved = -9;  // speed_limit as the ini had it; the last picked distance level (-1 off .. 2)
static void state_write()
{
    if (FILE *f = open_out("cruise_state.txt", "w")) {
        fprintf(f, "speed_limit=%d ini=%d\nacc=%d\n", p_isa > 0, s_isa_ini, s_acc_saved);
        fclose(f);
    }
}
static int is_round(float g)
{
    return fabsf(g) < 0.01f || fabsf(g - 20) < 0.01f || fabsf(g - 30) < 0.01f || fabsf(g - 40) < 0.01f;
}

// One beep per step of distance (short 1, middle 2, long 3), a low one for off.
static void level_tone(int lv)
{
    if (lv < 0) play_cue(CUE_ACC_OFF);
    else play_cue(lv <= 0 ? CUE_ACC_1 : lv == 1 ? CUE_ACC_2 : CUE_ACC_3);
}

// ---------------------------------------------------------------- overtake assist (idea 30-10)
// With the own ACC following a slower lead, the driver's LEFT turn signal (right-hand traffic; the code does not know the
// traffic side, so it is the left lane only) tells the cruise to prepare a lane change, as a real overtake assist does: it
// closes in a little (a shorter time gap, never under ~1 s) and pulls toward the set speed at the mode's own_acc_accel.
// Only Normal and Power (never Eco, never M). Only while the blind-spot detector ([traffic] probe + blind_spot) runs and
// shows the left lane clear; without trusted data it withholds and logs why. It ends on the signal, a lane change (the lead
// is gone), a brake, or a ~8 s timeout, and hands the normal gap back smoothly (s_dstar_sm). TrueShift reads
// cruise_overtake_on() to skip the cruise's force wish while it runs, so the pull does not trigger a kickdown; its lugging
// downshift still works.
static const float OT_TIMEOUT_S = 8.0f;      // the assist gives up this long after it started without a lane change
static const float OT_GAP_MIN_S = 1.0f;      // the overtake time gap never goes below this (s)
static const float OT_GAP_PULL_S = 0.25f;    // ... and is this much shorter than the player's own gap
static const float OT_IND_CONFIRM_S = 0.2f;  // the signal must be on this long before it counts (a flicker does not)
static const float OT_IND_OFF_S = 0.3f;      // and off this long before it ends (a blink is not the lever)
static const float OT_LOG_S = 2.0f;          // at most one on/off line this often (a flapping start cannot spam the log)
static struct {
    int on;             // the assist is active now (shorter gap + pull toward the set speed)
    double t0;          // when it started (the timeout)
    double ind_on_at;   // when the left signal came on (its confirmation)
    double ind_off_at;  // when it went off (the grace before ending)
    double said_at;     // throttle for the "withheld" lines
    double log_at;      // throttle for the on/off lines
    int ind_was;        // the signal was on last tick: only a fresh pull starts a new confirmation
} OT;

int cruise_overtake_on() { return OT.on || g_ot.on; }

// The US rule of thumb for the following distance of a truck: one second per 10 feet of vehicle length below 40 mph, one second more above
// (the extra second comes in between 36 and 44 mph, not as a step: the ACC would brake for it). The length: acc_length_ft, else 70 feet for
// a combination (a trailer is attached) and 25 for a truck alone; the vehicle model has no length.
static float us_gap_s(float v_ms)
{
    float ft = p_acc_ft > 0 ? p_acc_ft : (T.trailer ? 70.0f : 25.0f);
    float more = fminf(1.0f, fmaxf(0.0f, (v_ms * 2.23694f - 36.0f) / 8.0f));
    return (ft / 10.0f + more) * (g_level == 0 ? 0.8f : g_level == 2 ? 1.2f : 1.0f);
}
// The longest gap: acc_max_m, and under the US rule up to the radar's reach (about 100 m) so the rule is not cut at 90
static float acc_max_gap() { return p_acc_us > 0 ? fmaxf(p_max_m, 95.0f) : p_max_m; }

// (WP-SP3) with the layers' ACC (engine = layers, own_acc on) the three gaps are the owner's numbers at every speed: no motorway shrink, and the overtake assist's shorter gap is the requester's (core/req_acc.h)
static float layers_gap_s(float v_ms)
{
    const float t[3] = { vp_or(vprof()->acc_gap_s, p_gap_s), vp_or(vprof()->acc_gap_m, p_gap_m), vp_or(vprof()->acc_gap_l, p_gap_l) };
    return ra_gap_s(t, p_acc_us > 0, p_acc_ft, T.trailer, g_level, v_ms);
}
static float gap_s(float v_ms)
{
    if (layers_on() && p_own_acc > 0) return layers_gap_s(v_ms);
    float t = p_acc_us > 0 ? us_gap_s(v_ms) : g_level == 0 ? vp_or(vprof()->acc_gap_s, p_gap_s) : g_level == 2 ? vp_or(vprof()->acc_gap_l, p_gap_l) : vp_or(vprof()->acc_gap_m, p_gap_m);
    if (g_level != 0 && !(p_acc_us > 0)) {   // level 1 is really acc_short_s, in every mode (Damien 11-10: it came out at ~1.5 s)
        // closer at motorway speed (Damien 25-10): the full gap up to acc_fast_from_kmh, acc_fast_factor times it from acc_fast_full_kmh,
        // never below 0.7 s (or the set gap if that is already lower)
        float k = p_acc_fast_k;
        if (k < 1 && p_acc_fast_full > p_acc_fast_from) {
            float f = fminf(1.0f, fmaxf(0.0f, (v_ms * 3.6f - p_acc_fast_from) / (p_acc_fast_full - p_acc_fast_from)));
            t = fmaxf(t * (1 - f * (1 - k)), fminf(t, 0.7f));
        }
    }
    if (OT.on) {   // overtake: a little closer, never under ~1 s, and never wider than the normal gap
        float floor = fminf(t, OT_GAP_MIN_S);
        t = fminf(t, fmaxf(floor, t - OT_GAP_PULL_S));
    }
    return t;
}

// ---------------------------------------------------------------- lead speed and own ACC (autohold-acc notes)
// The game's ACC fn 0x1409958b0 asks the radar fn 0x140998ce0(game_actor, out) every tick; out = {gap m, lead speed m/s,
// FLT_MAX, 0, 0}. The call site is wrapped: we copy the first two floats. The lead is only valid while the game itself
// accepts it (+0x1a8 > 0.5, gap < 100 m, no indicator) and its gap matches ours.
typedef float *(*radar_fn)(void *ga, float *out);
static radar_fn o_radar;
static int g_radar_ok;
static struct { float gap, v; double at; int n; } s_radar;
// The lead is what the game's radar says (ATS, and ETS2 since 1.61): the raw field (L->a_lead_m) is the gap only on ETS2 1.60. On 1.61 it is not: the owner's ride of 04-11 (ETS2 1.61, 217-256 s)
// has it at 0.1 - 1.0 in 363 rows and never over 1.05, with no vehicle ahead (the radar's gap FLT_MAX, the ACC target 0, no car ahead in the traffic list); on 1.60 the same field was a gap in
// metres (six rides: over 1.05 it coincides with a radar gap in most rows, and it never read 0.3 - 1.05 with the radar silent). A lead then exists only when the radar confirms it, as on ATS:
// a stale or absent radar is no lead. Where the hook could not be installed (g_radar_ok 0) there is nothing to confirm with and 1.61 reads no lead (the raw read was the phantom).
static float lead_gap_m(char *g)
{
    if (sense_is_ats() || sense_is_161())
        return g_radar_ok && wall_s() - s_radar.at < 0.3 && s_radar.gap > 0.3f && s_radar.gap < 1e5f ? s_radar.gap : 0.0f;
    return sense_lead_m(g);
}
static float *radar_stub(void *ga, float *out)
{
    float *r = o_radar(ga, out);
    if (r && sense_readable(r, 20)) { s_radar.gap = r[0]; s_radar.v = r[1]; s_radar.at = wall_s(); s_radar.n++; }
    return r;
}

static float s_lead_v = NAN, s_acc_game = NAN, s_acc_own = NAN, s_dstar = NAN;
static struct { float t; int on; } OA;  // own ACC target (km/h) and whether it is following a lead

// Speed (m/s) of the game's current lead from the radar hook; NAN when there is none or the data is stale/inconsistent.
static float lead_speed(char *g)
{
    float lead = lead_gap_m(g);
    if (!g_radar_ok || !(lead > 0.5f) || !(wall_s() - s_radar.at < 0.3)) return NAN;
    if (!(fabsf(s_radar.gap - lead) <= 3 + 0.1f * lead)) return NAN;
    float v = s_radar.v;
    return v >= 0 && v < 70 ? v : NAN;
}

// The cab's start-stop: is there a vehicle ahead now? 1 = yes, *m its distance (m), *kmh its speed (km/h; 0 = unknown).
int cruise_lead(float *m, float *kmh)
{
    if (m) *m = 0;
    if (kmh) *kmh = 0;
    char *g = sense_actor();
    if (!g) return 0;
    float lead = lead_gap_m(g);
    if (!(lead > 0.5f)) return 0;
    if (m) *m = lead;
    float lv = lead_speed(g);
    if (kmh) *kmh = lv == lv ? lv * 3.6f : 0;
    return 1;
}

// Opening speed of the gap (m/s, + = the lead pulls away / someone cut in faster): lead speed minus ours, else d(gap)/dt.
static float lead_rel_speed(char *g, float lead, float dt)
{
    if (lead > 0.5f && s_lead_prev > 0.5f && fabsf(lead - s_lead_prev) < 15 && dt > 0)
        s_gap_rate += ((lead - s_lead_prev) / dt - s_gap_rate) * fminf(1, dt / 0.5f);
    else s_gap_rate = 0;
    s_lead_prev = lead;
    float lv = lead_speed(g);
    return lv == lv ? lv - fabsf(T.speed) : s_gap_rate;
}

// The own ACC's standstill gap follows the player's distance step: 3 m at the closest, 5 m at the farthest, the steps in
// between interpolated (three steps: close 3 / middle 4 / far 5). The player's own_acc_stop_m, when set (> 0), overrides it.
static const int g_acc_steps = 3;   // the game's ACC distance steps (g_acc 20 / 30 / 40)
static const float g_stop_near = 3.0f, g_stop_far = 5.0f, g_stop_grow = 2.0f;   // standstill gap at the closest / farthest step, and how much it opens up by 30 km/h
static float stop_base()   // the gap at a standstill (m)
{
    if (p_own_stop_m > 0) return p_own_stop_m;
    if (vprof()->stop_m == vprof()->stop_m) return vprof()->stop_m;   // a car stops ~2.5 m behind the lead (a truck: 3-5 m, by the distance step)
    float f = g_acc_steps > 1 ? (float)fmaxf(0, g_level) / (g_acc_steps - 1) : 0;
    return g_stop_near + (g_stop_far - g_stop_near) * f;
}
static float stop_full()   // the low-speed gap at 30 km/h (m)
{
    return stop_base() + vp_or(vprof()->stop_grow, g_stop_grow);
}

// The gap we want to keep (m): a standstill gap plus the time gap of the player's distance level (and mode).
// (Damien 12-10: in a queue it may close up, whatever the distance level, not onto the bumper: from walking pace the gap grows
// from the standstill gap to its full value and the time gap fades in, both full at 30 km/h)
static float stop_gap(float v_ms)
{
    return stop_base() + vp_or(vprof()->stop_grow, g_stop_grow) * fminf(1.0f, v_ms / 8.33f);
}
static float desired_gap(float v_ms)
{
    float mx = fmaxf(acc_max_gap(), stop_full());
    float fade = fminf(1.0f, v_ms / 8.33f);
    // level 1: the standstill gap only matters at low speed (max, not sum); 2 and 3 keep the margin on top
    float m = g_level == 0 ? fmaxf(stop_gap(v_ms), gap_s(v_ms) * v_ms) : stop_gap(v_ms) + gap_s(v_ms) * v_ms * fade;
    return m > mx ? mx : m;
}

// The closing speed (m/s) above the lead's speed the own ACC plan allows: sqrt(2 a room), scaled by the room still ahead over
// the wanted gap and by the speed, so at low speed and short gaps the truck matches the lead earlier and settles at the gap
// (Damien 31-10: on a slowing queue it stayed 2-6 km/h over the lead down to 5 m, closing 1-1.9 m/s, then it crept). Motorway
// following (from 30 km/h) keeps the plain sqrt(2 a room), so it is not made sluggish.
static float plan_close(float room, float dstar, float kmh, float heavy)
{
    float close = sqrtf(2 * (p_own_plan * mode_k(p_acc_eco_k, p_acc_pw_k) / heavy) * room);
    float room_share = dstar > 0.5f ? fminf(1.0f, room / dstar) : 0.0f;
    float slow = 1.0f - fminf(1.0f, kmh / 30.0f);   // 1 at a crawl, 0 from 30 km/h up
    return close * (1.0f - slow * (1.0f - room_share));
}

// Overtake assist state (see the comment block before gap_s). Runs once per own-ACC tick (cr_pre), also on the tick that
// ends a following run, so a lane change (the lead going away) still ends it with a log line. Read by own_acc (shorter gap,
// pull toward the set speed), by gap_s/desired_gap (the planner's lead constraint), and by TrueShift (cruise_overtake_on).
// The brake request that holds the truck at the stop gap behind a vehicle that stands (the classic own ACC's last clause, the layers' ACC uses it too: who holds at a standstill is the same
// code): sized for the grade from the learned gain ([core] brake_learn), never under 0.2; the low-speed owner holds while Stop&Go does. 0 = none.
static float acc_hold_wish(float v, float lv, float lead)
{
    if (p_own_brake > 0 && v <= 0.05f && lv < 0.5f && lead < stop_base() + 0.6f && !(low_owner_on() && g_sg_hold)) return fmaxf(0.2f, vehicle_brake_hold_req(pitch_grade(), 0.2f));
    return 0.0f;
}

// The last metres of a stop behind a vehicle that stands (the layers' ACC; the classic own ACC's clause in own_acc, which keeps it as it is): under the speed where the brake channel hands the
// service brake over and fades out (10 km/h, BC_HAND_HI in core/brake.cpp: the channel keeps no authority at a crawl), the stop is the ACC's own brake request above the channel, as it has
// always been (the header of core/brake.cpp: "the ACC's stop behind a standing lead keeps its own owner"): when the stop at the stop gap needs more than 0.4 m/s2, or the truck is at the gap,
// a request sized by the brake model, firm at the gap so the truck does not creep past it. 0 = none.
static float acc_stop_wish(float v, float lv, float lead, float gmin)
{
    if (!(p_own_brake > 0 && lv < 1.5f && lead < gmin + 15 && v > 0.05f && v < 10.0f / 3.6f)) return 0.0f;
    const float stop = fmaxf(2.5f, stop_base()), s = lead - stop;   // the room still ahead of the stop gap (negative = past it)
    const float a_t = v * v / (2 * fmaxf(0.3f, s));
    // with a hysteresis (on above 0.4 m/s2, off under 0.25): a stop that needs about 0.4 would otherwise switch the request on and off with every wobble of the speed (the sim's flips)
    static int on; on = a_t > (on ? 0.25f : 0.4f) || s < 0.15f;
    if (!on) return 0.0f;
    float bs = fminf(fmaxf(p_own_brake, 0.45f), 0.08f + a_t / (ACC_GAIN_SHARE * vehicle_brake_cautious()));
    if (s < 0.15f) bs = fmaxf(bs, 0.25f);   // at the gap: firm, no creep past it
    return bs;
}

static void overtake_tick(char *g, float lead, float lv)
{
    double now = sim_s();   // simulation time: a pause does not run the ~8 s timeout down
    int on_cruise = g && sense_cruise_on(g);
    int have_lead = lead > 0.5f && lv == lv;
    unsigned char fl = sense_lights_guarded(g);
    int ind_l = T.blink_l || (fl & 4);
    int md = sense_mode();
    float set = g ? sense_cruise_set(g) : 0;
    float kmh = fabsf(T.speed) * 3.6f;

    // the signal's confirmation / off-grace bookkeeping. Only a fresh pull of the lever starts a confirmation: an end while
    // the signal is still held (timeout, brake) must not re-arm on the stale on-time.
    if (ind_l) { if (!OT.ind_was) OT.ind_on_at = now; OT.ind_off_at = 0; }
    else { OT.ind_on_at = 0; if (OT.on && !OT.ind_off_at) OT.ind_off_at = now; }
    OT.ind_was = ind_l;

    if (OT.on) {
        const char *why = NULL;
        if (!(p_overtake > 0) || p_own_acc <= 0) why = p_own_acc <= 0 ? "the own ACC is off" : "overtake assist off in the ini";
        else if (!on_cruise) why = "cruise off";
        else if (!have_lead) why = "the lead is gone (lane changed)";
        else if (T.in_brake > 0.05f || s_want_acc > 0.04f || AE.st != AEB_OFF) why = T.in_brake > 0.05f ? "brake" : "the own ACC is braking";
        else if (!ind_l && now - OT.ind_off_at >= OT_IND_OFF_S) why = "the indicator went off";
        else if (now - OT.t0 >= OT_TIMEOUT_S) why = "no lane change within the timeout";
        if (why) {
            OT.on = 0; OT.ind_off_at = 0; OT.ind_on_at = 0;   // (ind_was keeps a held lever from re-arming it)
            if (now - OT.log_at >= OT_LOG_S) {
                OT.log_at = now;
                logline("Cruise: overtake assist off (%s) after %.1f s at %.0f km/h", why, now - OT.t0, kmh);
            }
        }
        return;
    }

    // not active: consider starting. Only the own ACC, Normal and Power, a clearly slower lead, and a signal held a moment.
    if (!(p_overtake > 0) || p_own_acc <= 0 || !on_cruise || !have_lead || !ind_l || !OT.ind_on_at) return;
    if (md == MODE_ECO || md == MODE_M) return;
    if (!(lv * 3.6f < set - 1.0f)) return;
    if (T.in_brake > 0.05f || s_want_acc > 0.04f || AE.st != AEB_OFF) return;   // the same ends: no tick-rate flapping
    if (now - OT.ind_on_at < OT_IND_CONFIRM_S) return;
    traffic_around ta;
    traffic_around_get(&ta);
    if (!ta.ok || !ta.bs_on) {
        if (now - OT.said_at > 5.0) {
            OT.said_at = now;
            logv(1, "Cruise: overtake assist withheld: no trusted blind-spot data (probe %s, detector %s)", traffic_probe_on() ? "on" : "off", ta.bs_on ? "on" : "off");
        }
        return;
    }
    if (ta.bs_left) {
        if (now - OT.said_at > 5.0) { OT.said_at = now; logv(1, "Cruise: overtake assist withheld: the left lane is occupied"); }
        return;
    }
    OT.on = 1; OT.t0 = now;
    if (now - OT.log_at >= OT_LOG_S) {
        OT.log_at = now;
        logline("Cruise: overtake assist on (lead %.0f m at %.0f km/h, we %.0f, set %.0f, %s)", lead, lv * 3.6f, kmh, set, md == MODE_POWER ? "Power" : "Normal");
    }
}

// Own ACC: target speed = lead speed + gain x (gap - wanted gap), slewed (a lead that appears starts from our speed),
// written into +0x1a0 so the game's controller and its overspeed braking (retarder / engine brake / service brake)
// follow it. The game's emergency braking (gap < half its D(v)) is untouched. Cut-in or a lead pulling away faster than
// we drive: never slower (acc_calm_cut_in), the target only stops rising.
static void own_acc(char *g, float dt)
{
    s_acc_game = sense_acc_target(g);
    s_acc_own = NAN;
    s_dstar = NAN;
    float lead = lead_gap_m(g), lv = lead_speed(g);
    s_lead_v = lv;
    overtake_tick(g, lead, lv);   // the overtake assist state (before the early return: it must end on a lane change too)
    if (p_own_acc <= 0 || !sense_cruise_on(g) || g_level < 0 || !(lv == lv) || !(dt > 0 && dt < 0.5f)) { OA.on = 0; s_want_acc = 0; s_lv_prev = NAN; s_a_lead = 0; s_dstar_sm = NAN; return; }
    float set = sense_cruise_set(g), v = fabsf(T.speed), kmh = v * 3.6f;
    // the wanted gap moves by at most 1.5 m/s when it grows and 3 m/s when it shrinks: a change of the distance stand (1 -> 2) is no reason to brake
    // at once (Damien 25-10), the gap grows into it. Starts fresh whenever following starts (a stale small gap made a late approach).
    // The wanted gap is time gap x the LEAD's speed (Damien 30-10): it aims at once for the gap it will settle at, instead of one that
    // shrinks while we slow down to the lead. When it gets tense it slides smoothly to our own speed (the margin the ACC standard
    // defines), no tipping point: a lead braking from 0.3 m/s2 (radar noise under that) to 1.5, or a time to collision from 6 s to 3 s
    float w = fminf(1, fmaxf(0, (-s_a_lead - 0.3f) / 1.2f)), cl = v - fmaxf(0, lv);
    if (cl > 0.3f) w = fmaxf(w, fminf(1, fmaxf(0, (6 - lead / cl) / 3)));
    float k_lead = 1 - w;   // 1 = the lead's speed, 0 = ours
    float w_tense = w;      // (also narrows the gap band below)
    float dstar_raw = desired_gap(v + k_lead * (fmaxf(0, lv) - v));
    if (!(s_dstar_sm == s_dstar_sm) || fabsf(dstar_raw - s_dstar_sm) > 60) s_dstar_sm = dstar_raw;
    else s_dstar_sm += fmaxf(-3.0f * dt, fminf(1.5f * dt, dstar_raw - s_dstar_sm));
    float dstar = s_dstar_sm;
    float stopm = stop_gap(v);
    s_dstar = dstar;
    // Heavier = slower to stop (Damien: "start braking earlier at 60 t"): from 30 t up the planned deceleration shrinks
    // and the same need asks for more brake, up to 2.5x at 75 t.
    float heavy = fminf(2.5f, fmaxf(1.0f, load_mass() / 30000));
    float want = set;
    if (lv * 3.6f < set) {
        want = (lv + p_own_gain * (lead - dstar)) * 3.6f;
        want = want < 0 ? 0 : want > set ? set : want;
    }
    if (OT.on) want = set;   // overtake: pull toward the set speed; the approach plan below still caps it for a safe close-in
    // Plan the approach (ts4o 365-377 s: 66 km/h onto a ~25 km/h lead, gap 49 → 10 m in 8 s: the game's emergency
    // braking emptied the brake air, 135 → 25 psi in 4 s): never faster than the speed from which a comfortable
    // deceleration still ends at the lead's speed at the wanted gap.
    if (p_own_plan > 0) {
        // relative to the lead (Damien 30-10: it reacted at 100 m to a lead 20 km/h slower): only the speed difference has to go, so
        // the closing speed allowed is sqrt(2 a room) and the target the lead's speed plus that (a stopped lead: the same as before)
        float room = fmaxf(0, lead - fmaxf(stopm, p_plan_to * dstar));
        float v_ok = (fmaxf(0, lv) + plan_close(room, dstar, kmh, heavy)) * 3.6f;
        if (v_ok < want) want = v_ok;
    }
    float opening = lv - v, closing = -opening;
    // Crossing traffic (friend 11-10: cruise at ~40 km/h into roundabouts, down to a stop 3-4 times): the radar's lead
    // speed is absolute, so a car crossing in front drives at ~25 km/h while the gap closes at our own speed. Following
    // it is wrong; it is gone in a moment. Measured gap rate (0.4 s filter) against what the lead's speed explains.
    if (s_cr_prev == s_cr_prev && fabsf(lead - s_cr_prev) < 10) {
        float r = (lead - s_cr_prev) / dt;
        s_cr_rate = s_cr_rate == s_cr_rate ? s_cr_rate + (r - s_cr_rate) * fminf(1, dt / 0.4f) : r;
    } else s_cr_rate = NAN;
    s_cr_prev = lead;
    int crossing = 0;
    if (p_acc_cross > 0 && s_cr_rate == s_cr_rate && lv > 3.0f && v > 3.0f && opening - s_cr_rate > fmaxf(2.5f, 0.5f * lv)) {
        if (s_cr_since < 0) s_cr_since = wall_s();
        crossing = wall_s() - s_cr_since > 0.4;
        if (crossing && wall_s() - s_cr_said > 10) {
            s_cr_said = wall_s();
            logv(1, "Cruise: own ACC passes crossing traffic at %.0f m (it drives %.0f km/h, the gap closes at %.0f km/h; we %.0f), junction ahead %.0f m",
                    lead, lv * 3.6f, -s_cr_rate * 3.6f, kmh, route_junction_m());
        }
    } else s_cr_since = -1;
    if (crossing) want = fmaxf(want, fminf(kmh, set));
    // level 1: the own ACC slowing hard for a lead (friend 11-10: cruise at ~40 km/h into roundabouts, 3-4 times down
    // to a stop with no pedal; with log_level 1 nothing showed why)
    {
        static double said_slow;
        double now_s = wall_s();
        if (want < fmaxf(10, 0.4f * kmh) && kmh > 15 && now_s - said_slow > 10) {
            said_slow = now_s;
            logv(1, "Cruise: own ACC slows for a lead at %.0f m, %.0f km/h (we %.0f, set %.0f, gap %+.1f m/s, from its speed %+.1f): "
                    "target %.0f km/h, junction ahead %.0f m", lead, lv * 3.6f, kmh, set, s_cr_rate, lv - v, want, route_junction_m());
        }
    }
    if (!OA.on) {
        static double said;
        OA.on = 1;
        OA.t = fminf(kmh, set);
        if (wall_s() - said > 5) {
            said = wall_s();
            logv(2, "Cruise: own ACC follows a lead at %.0f m, %.0f km/h (we %.0f), wanted gap %.0f m, target %.0f km/h (game %.0f)", lead, lv * 3.6f, kmh, dstar, want, s_acc_game);
        }
    }
    float dn = p_own_dec * 3.6f, up = p_own_up * 3.6f * mode_k(p_acc_eco_k, p_acc_pw_k);  // Eco calmer, Power quicker after a lead pulls away
    if (closing > 0.3f) {  // deceleration it takes to be at the lead's speed before the standstill gap: above 0.5 m/s2 the target falls faster (up to 8x)
        float a_req = heavy * closing * closing / (2 * fmaxf(lead - stopm, 2.0f));
        if (a_req > 0.5f) dn *= 1 + fminf(7.0f, (a_req - 0.5f) * 4);
    }
    // Brake ourselves, early and gently, when easing off is not enough (Damien, ts4p 308-314 s: a cut-in at 27 m that
    // was slowing; we only lifted, the game's overspeed braking did nothing at 3 km/h over, and at 7 m its emergency
    // braking emptied the brake air 138 -> 5 psi in 1.3 s). Needed deceleration to be at the lead's speed at the
    // minimum gap, plus the lead's own braking; coasting gives ~0.3 m/s2; the rest is our brake request.
    {
        // (lead speed history is reset whenever there is no lead: a new lead must not look like a lead braking at
        // (lv - old lv) / dt, audit 11-10: a 0.35 brake request for 1.5 s)
        if (s_lv_prev == s_lv_prev && dt > 0) s_a_lead += ((lv - s_lv_prev) / dt - s_a_lead) * fminf(1, dt / 0.5f);
        s_lv_prev = lv;
        float gmin = fmaxf(stopm, 0.5f * dstar), room = fmaxf(1.0f, lead - gmin);
        float need = closing > 0 ? closing * closing / (2 * room) : 0;
        // the lead's own braking only counts when it is near: ts4t 444-458 s a car 99 m ahead (radar range edge, 31 m
        // wanted) braked to a junction and we mirrored it 78 -> 4 km/h with 0.30 requests; far away the closing term
        // above is enough. Full weight at the wanted gap, none from 1.5x it.
        float near_w = fminf(1, fmaxf(0, (1.5f * dstar - lead) / (0.5f * dstar)));
        // ... and also from afar when the time to collision is short (Damien 12-10: 54 m behind a lead braking from 49 to 33 km/h
        // while we closed at 10 m/s: our brake request was 0.14, the AEB had to add partial braking on top)
        if (closing > 0.3f) near_w = fmaxf(near_w, fminf(1, fmaxf(0, (7.0f - lead / closing) / 3.0f)));
        if (closing > -1) need += near_w * fmaxf(0, -s_a_lead - 0.3f);  // 0.3: radar speed noise (ts4t 505-516 s: pulses at 40 m)
        float b = p_own_brake > 0 && need > 0.35f / heavy ? fminf(p_own_brake * fminf(heavy, 1.5f), (need - 0.3f / heavy) / (ACC_GAIN_SHARE * vehicle_brake_cautious())) : 0;
        if (b > 0 && b < 0.05f) b = 0.05f;
        // channel: the same request as a brake deceleration (brake_ask_add: on top of what the truck does by itself), retarder first;
        // urgent with a short time to collision or a hard need, else firm. The stop behind a standing lead and the standing hold below
        // stay this module's own requests (low speed, above the channel).
        if (brake_channel_on()) {
            if (b > 0) {
                float ttc = closing > 0.3f ? lead / closing : 99.0f;
                // (its law was tuned as an extra request on top of coasting and the game's own overspeed braking, which the need's
                // kinematics leave out with the brake's lag and the lead's braking still to come: so it stays an extra brake
                // deceleration, the old request through the brake's gain, the learned one or the fit)
                s_ch[BRQ_ACC] = crossing ? 0 : b * brake_gain(NULL);
                s_chu[BRQ_ACC] = ttc < 4.0f || need > 2.5f ? BRU_URGENT : BRU_FIRM;
            }
            b = 0;
        }
        // Stopping behind a lead that stops (Damien, ts4q 292-300 s: the lead stopped at 8.7 m, we crept on at 3 km/h
        // in gear 1 at idle with a 0.015 request for 7 s, at 2.6 m the game's emergency braking dumped the air 120 -> 5
        // psi). Plan and execute the stop at the standstill gap stop_base() (floor 2.5 m): cap the target to the walking
        // pace that dies at the gap, so a truck that stopped short creeps up the last meters (like a real ACC Stop&Go)
        // and cannot overshoot; brake only when the stop needs it or it is at the gap. A lead creeping away at walking
        // pace (the gap opening) is no threat: follow it.
        if (p_own_brake > 0 && lv < 1.5f && lead < gmin + 15) {
            float stop = fmaxf(2.5f, stop_base());
            float s = lead - stop;   // the room still ahead of the stop gap (negative = past it)
            if (v > 0.05f) {
                float v_ok = fminf(1.4f, sqrtf(2 * 0.4f * fmaxf(0.0f, s)));   // walking pace (5 km/h) dying at the gap
                if (v_ok * 3.6f < want) want = v_ok * 3.6f;
                // a truck that stopped short creeps up the last meters at walking pace: keep the target at the creep
                // profile while it is already slow, so the plan law's short-gap scaling does not stall it short of the gap.
                if (s > 0.05f && v < 1.4f) want = fmaxf(want, v_ok * 3.6f);
                float a_t = v * v / (2 * fmaxf(0.3f, s));
                // only when the stop really needs it: ts4r 213-215 s stopped at 16.6 m behind a stopped car (a_t 0.2) with
                // our throttle fighting it; with room to spare the creep closes in by itself (Damien liked that).
                if (a_t > 0.4f || s < 0.15f) {
                    float bs = fminf(fmaxf(p_own_brake, 0.45f), 0.08f + a_t / (ACC_GAIN_SHARE * vehicle_brake_cautious()));
                    if (s < 0.15f) bs = fmaxf(bs, 0.25f);   // at the gap: firm, no creep past it
                    b = fmaxf(b, bs);
                }
            }
        }
        b = fmaxf(b, acc_hold_wish(v, lv, lead));  // standing at the gap: hold (the low-speed owner's while Stop&Go holds); [core] brake_learn: sized for the grade from the learned gain, never under 0.2
        s_want_acc = crossing ? 0 : b;
    }
    if (want < OA.t) OA.t = fmaxf(want, OA.t - dn * dt);
    else OA.t = fminf(want, OA.t + up * dt);
    // calm only when the gap is about right: ts4n followed at 8.8 m with 27 m wanted, and "the lead pulls away 1 km/h
    // faster" kept the target at our speed, so it never opened the gap (Damien: "it seems to do nothing")
    // Damien: a lead that is clearly faster (≥ ~3.5 km/h) opens the gap by itself, even when you are on its bumper:
    // no braking then, only no speeding up. Just a little faster (0.3-1 m/s): only calm when the gap is already reasonable.
    // (Weights, not switches: the target is what the game's own overspeed braking reads at once, and it jumped by the whole gap between
    // "follow" and "hold the speed" at an opening of 0.3 m/s. Full calm from 0.6 m/s, so that the calm comes a little later than before,
    // never earlier; the state is lifted only at full weight.)
    float w_open = ramp01(opening, 0.3f, 0.6f) * ramp01(lead, 0.5f * dstar, 0.6f * dstar);
    float w_pull = fmaxf(ramp01(opening, 1.0f, 1.3f), w_open);   // 1 = pulling away (the old flag)
    float w_close = 0;
    if (lead < 0.4f * dstar && s_acc_game == s_acc_game && s_acc_game >= 0) w_close = (1 - w_pull) * ramp01(0.45f * dstar - lead, 0.0f, 0.05f * dstar);   // (full from 0.4 x the wanted gap as before, in from 0.45)
    float t_out = OA.t, w_calm = p_acc_calm > 0 ? w_pull * ramp01(lead, 5.0f, 6.0f) : 0.0f;
    if (w_calm > 0) {
        float hold = fmaxf(OA.t, fminf(kmh, set));
        t_out += w_calm * (hold - t_out);
        if (w_calm >= 1) OA.t = hold;
    }
    // far too close: never more than the game's own ACC target (it knows how to open a gap)
    if (w_close > 0) {
        float cap = fminf(t_out, fmaxf(s_acc_game, want));
        t_out += w_close * (cap - t_out);
        if (w_close >= 1) OA.t = fminf(OA.t, cap);
    }
    // The gap band (own_acc_band, Damien 30-10: following a lead cost fuel, uphill the throttle pulled us up to the lead again and
    // downhill the retarder took it back): uphill the gap may grow to hi before the cruise pulls up for it, so it does not chase the
    // lead up a climb in a low gear; a narrow band (3 %) also ends the pumping between a little throttle and a little retarder. Anchored to the lead: at hi the target is the gap law's with hi (never slower than that), so the
    // speed cannot sag away by itself. It narrows to nothing when it gets tense. (Only above the wanted gap: letting the gap shrink
    // downhill before braking only moved the braking deeper and then had to pull up again, +13..+28 % fuel in the sweep. Wider than 3 % gained no more on average and grew the outliers.)
    if (p_acc_band > 0 && !crossing && w_close <= 0) {
        float hi = dstar * (1 + p_acc_band), here = fminf(kmh, set);
        float t_hi = fminf(set, fmaxf(0, (lv + p_own_gain * (lead - hi)) * 3.6f));
        if (t_out > here) t_out += (1 - w_tense) * (fmaxf(here, t_hi) - t_out);
    }
    if (crossing) { OA.t = fmaxf(OA.t, fminf(kmh, set)); t_out = fmaxf(t_out, fminf(kmh, set)); }  // hold the speed past it (the game's emergency braking stays as the last resort)
    if (OA.t > set) OA.t = set;
    if (t_out > set) t_out = set;
    act_acc_target(g, t_out);
    s_acc_own = t_out;
}

// ---------------------------------------------------------------- (WP-SP3) the vehicle ahead in the layers
// cr_pre, engine = layers and own_acc on: ONE reading of the radar per tick into the lead model (the same gap and speed the classic ACC and the AEB read: lead_gap_m, lead_speed), the overtake
// assist's state, the standstill hold, and the decision whether the layers follow the vehicle ahead this tick (s_acc_lay). They do when its speed is known (or there is none), the vehicle model
// can tell the force and the planner does not drive; otherwise the classic loop and own_acc keep the tick, as in SP1 (the game's ACC target then is the ACC). The layers' ACC does not write
// the game except to take the game's own ACC out of the way: the game's overspeed braking (retarder, engine brake, service brake) works on its ACC target, and the brakes belong to the arbiter's
// channel, so that target is put on the set speed, where it is with no vehicle ahead.
// Who holds the truck at a standstill behind a vehicle, with layers (SP5 unifies it): acc_hold_wish's ACT_ACC brake at the stop gap (the module); the low-speed owner while Stop&Go holds the cruise
// (g_sg_hold: it owns the clutch and the drive, and acc_hold_wish stands down); Auto Hold after a stop on the pedal or the lever (blocked while Stop&Go holds). The arbiter holds nothing at
// 0 km/h: its force at a standstill is the rolling resistance, and stopgo.h's `stay` zeroes the cruise's throttle while the truck waits for Resume.
static void acc_layers_tick(char *g, float dt)
{
    const float v = fabsf(T.speed);
    const int on = sense_cruise_on(g) && dt > 0 && dt < 0.5f;
    lm_in in; memset(&in, 0, sizeof in);
    in.dt = dt; in.v = v; in.gap = lead_gap_m(g); in.lv = lead_speed(g);
    in.src = (sense_is_ats() || sense_is_161()) ? (!g_radar_ok ? LMS_NO_HOOK : wall_s() - s_radar.at >= 0.3 ? LMS_STALE : LMS_OK) : LMS_OK;   // (1.60 reads the raw field, as the classic ACC does)
    in.merger_n = g_cutin_pred.n; in.merger_id = g_cutin_pred.id; in.merger_s = g_cutin_pred.s; in.merger_d = g_cutin_pred.d; in.merger_closing = g_cutin_pred.closing;
    lm_step(&g_lm, in, &g_lmo);
    s_acc_game = sense_acc_target(g); s_lead_v = g_lmo.valid ? g_lmo.lv : NAN; s_acc_own = NAN; s_dstar = NAN;
    // The log (dev, level 2): a lead comes and goes, with its reason; the vehicle Cutin predicts to merge (LOG ONLY: nothing acts on it; Cutin's own follow-up line, same id, says if the radar took it)
    if (on && g_level >= 0) {
        static int was_valid, was_reason = -1; static double said_at;
        if ((g_lmo.valid != was_valid || (!g_lmo.valid && g_lmo.reason != was_reason)) && wall_s() - said_at > 1.0) {
            said_at = wall_s(); was_valid = g_lmo.valid; was_reason = g_lmo.reason;
            if (g_lmo.valid) logv(2, "Cruise (layers): lead %.0f m ahead, %.0f km/h (we %.0f), it %s %.2f m/s2", g_lmo.gap, g_lmo.lv * 3.6f, v * 3.6f, g_lmo.a_lead < 0 ? "brakes at" : "accelerates at", fabsf(g_lmo.a_lead));
            else logv(2, "Cruise (layers): no lead: %s", lm_reason_name(g_lmo.reason));
        }
    }
    if (g_lmo.merger_new && on)
        logv(2, "Cruise (layers): predicted merger id %06x, %.0f m ahead and %+.1f m off our path, closing at %.1f m/s (we %.0f km/h, %s): log only, nothing acts on it; Cutin's follow-up line of the same id says if the radar took it",
             g_lmo.merger_id, g_lmo.merger_s, g_lmo.merger_d, g_lmo.merger_closing, v * 3.6f, g_lmo.valid ? "a lead is followed" : "no lead");
    planner_drive pd0;
    s_acc_lay = V.eng_ok && !V.electric && V.mass > 0 && !planner_follow(&pd0) && (g_lmo.valid || !g_lmo.present);
    if (!s_acc_lay) return;
    OT.on = 0;   // the classic assist's state: the layers' own is in force
    // the overtake assist (core/req_overtake.h): the lead model's lead, the blind-spot detector, the indicator
    {
        const double now = sim_s();
        const int md = sense_mode();
        unsigned char fl = sense_lights_guarded(g);
        traffic_around ta; traffic_around_get(&ta);
        ot_in oi; memset(&oi, 0, sizeof oi);
        oi.now = now; oi.enabled = p_overtake > 0 && p_own_acc > 0; oi.cruise_on = on && g_level >= 0; oi.lead = g_lmo.valid; oi.lv_ms = g_lmo.lv; oi.set_kmh = sense_cruise_set(g);
        oi.mode = md == MODE_ECO ? 0 : md == MODE_POWER ? 2 : md == MODE_M ? 3 : 1;
        oi.ind_left = T.blink_l || (fl & 4); oi.brake = T.in_brake > 0.05f;
        oi.acc_braking = g_sl_out.ok && g_sl_out.owner == SAO_LEAD && g_sl_out.brake_decel > 0.04f;   // the ACC itself asks for a brake (the last decision)
        oi.aeb = AE.st != AEB_OFF; oi.bs_trusted = ta.ok && ta.bs_on; oi.bs_left = ta.bs_left;
        const ot_out oo = ot_step(&g_ot, oi);
        static double log_at, said_at;
        if (oo.started && now - log_at >= 2.0) { log_at = now; logline("Cruise (layers): overtake assist on (lead %.0f m at %.0f km/h, we %.0f, set %.0f, %s)", g_lmo.gap, g_lmo.lv * 3.6f, v * 3.6f, oi.set_kmh, oi.mode == 2 ? "Power" : "Normal"); }
        if (oo.ended && now - log_at >= 2.0) { log_at = now; logline("Cruise (layers): overtake assist off (%s) after %.1f s at %.0f km/h", oo.why, oo.t_on, v * 3.6f); }
        if (oo.withheld && now - said_at > 5.0) { said_at = now; logv(1, "Cruise (layers): overtake assist withheld: %s", oo.withheld); }
    }
    // the standstill hold at the stop gap (the module's ACT_ACC brake, the same function the classic ACC calls)
    s_want_acc = 0.0f;
    if (on && g_level >= 0 && g_lmo.valid) {
        const float gmin = fmaxf(ra_stop_gap(stop_base(), vp_or(vprof()->stop_grow, g_stop_grow), v), 0.5f * g_ra.dstar);
        s_want_acc = fmaxf(acc_hold_wish(v, g_lmo.lv, g_lmo.gap), acc_stop_wish(v, g_lmo.lv, g_lmo.gap, gmin));   // (not behind a crossing car: it is passed)
        if (g_lmo.crossing && p_acc_cross > 0) s_want_acc = 0.0f;
    }
    // the game's own ACC out of the way: its target (which its overspeed braking works on) is the set speed, as with no vehicle ahead
    if (on && g_level >= 0 && g_lmo.present) { const float set = sense_cruise_set(g); if (s_acc_game >= 0 && s_acc_game < set) act_acc_target(g, set); }
}

// The ACC's requests of this tick (cr_layers_body / cr_layers_nomodel, after the road's): the vehicle ahead as a moving target, its gap law, the braking lead, the protections; or nothing
static void cr_acc_post(char *g, float dt, float set_kmh, int mi)
{
    (void)g;
    const float v = fabsf(T.speed);
    ra_in ri; memset(&ri, 0, sizeof ri);
    ri.on = s_acc_lay && g_level >= 0; ri.lead = &g_lmo; ri.dt = dt; ri.mode = mi; ri.v = v; ri.set = set_kmh / 3.6f; ri.level = g_level < 0 ? 1 : g_level;
    ri.t_gap_s = layers_gap_s(v); ri.stop_base_m = stop_base(); ri.stop_grow_m = vp_or(vprof()->stop_grow, g_stop_grow); ri.max_m = acc_max_gap();
    ri.gain = p_own_gain; ri.plan_to = p_plan_to; ri.grade_pct = pitch_grade(); ri.nat = vehicle_natural_decel(T.gear, v); ri.band = p_acc_band;
    { brake_view bv; brake_view_get(&bv); ri.a_dec = bv.on ? bv.measured : 0.0f; }
    ri.calm = p_acc_calm > 0; ri.cross = p_acc_cross > 0; ri.overtake = g_ot.on; ri.aeb_max = vp_or(vprof()->aeb_max_dec, p_aeb_max_dec);
    g_rao = ra_post(&g_sa, &g_ra, g_slc.arb, ri);
    s_dstar = g_rao.following ? g_rao.wanted_m : NAN;
}

static FILE *g_csv;
static void cruise_trace(char *g, float u)
{
    static double at;
    if (!g_csv) return;
    int on = sense_cruise_on(g);
    if (!on && AH.st == AH_OFF && AH.r <= 0 && act_request() <= 0 && AE.st == AEB_OFF) return;
    double now = wall_s();
    if (now - at < 0.1) return;
    at = now;
    fprintf(g_csv, "%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.2f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%d,%.3f,%.2f,%.2f,%d,%.2f,%.0f,%.2f,%.1f,%d,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f\n", now, T.speed * 3.6f,
            on ? sense_cruise_set(g) : 0.0f, sense_lead_m(g) /* raw field, for the trace */, s_radar.gap < 1e6f ? s_radar.gap : 0.0f, s_lead_v * 3.6f,
            s_lead_v == s_lead_v ? s_lead_v - fabsf(T.speed) : NAN, s_acc_game, s_acc_own, sense_acc_target(g), s_dstar, u, act_request(),
            sense_brake_req(g), AH.st, AH.r, T.in_thr, T.in_brake, T.gear, T.eff_clutch, T.rpm, pitch_grade(), T.air_psi,
            AE.st, AE.ttc < 99 ? AE.ttc : 99.0f, AE.a_need, AE.a_cmd, -T.accel,
            s_bend_coast_kmh, fabsf(T.speed) * T.yaw * 6.2832f);   // sideways acceleration now (m/s2, + = left): v x yaw rate
}

static void cr_start()
{
    // the last speed-limit following on/off (Damien: remember it)
    s_isa_ini = p_isa > 0;
    if (FILE *f = open_out("cruise_state.txt", "r")) {
        int v, ini = -1;
        // the ini value at the time of the toggle is stored too: a changed ini (setup page) wins over the memory
        if (fscanf(f, "speed_limit=%d ini=%d", &v, &ini) >= 1) {
            if (ini >= 0 && ini != (p_isa > 0)) logline("Cruise: speed limit following %s (from the ini, changed since the last toggle)", p_isa > 0 ? "on" : "off");
            else { p_isa = v ? 1 : 0; logline("Cruise: speed limit following %s (remembered)", v ? "on" : "off"); }
        }
        int a;
        if (fscanf(f, " acc=%d", &a) == 1 && a >= -1 && a <= 2) s_acc_saved = a;
        fclose(f);
    }
    if (p_enabled <= 0) { logline("Cruise disabled in the ini"); return; }
    // Cruise from min_kmh instead of 30: the minimum is only checked in the key
    // handlers (cruise-re notes § 4), against a pooled 30.0 constant; point
    // those two loads (activate compare, handler's xmm7 for dec/resume) at our own.
    // The same two code sites also prove the 1.60 game_actor layout that g_low_ok stands for (hazards, hill start, Auto
    // Hold, ISA, easing, limiter): found → g_low_ok, whatever min_kmh is (audit 11-10: with the player preset's 30 all of
    // those were silently off, in 0.4 too).
    {
        uintptr_t act = game_scan(sense_is_161() ? "0F 54 05 ?? ?? ?? ?? 0F 2F 05 ?? ?? ?? ?? F3 0F 11 83 08 02 00 00 73"
                                                : "0F 54 05 ?? ?? ?? ?? 0F 2F 05 ?? ?? ?? ?? F3 0F 11 83 9C 01 00 00 73");
        uintptr_t head = game_scan(sense_is_162() ? "F3 0F 10 35 ?? ?? ?? ?? 45 33 FF 84 C9 0F 84 ?? ?? ?? ?? 44 38 BF 04 02 00 00"   // 1.62: xmm6
                                 : sense_is_161() ? "F3 0F 10 3D ?? ?? ?? ?? 45 33 ED 84 C9 0F 84 ?? ?? ?? ?? 44 38 AF 04 02 00 00"
                                                 : "F3 0F 10 3D ?? ?? ?? ?? 45 33 FF 84 C9 0F 84 ?? ?? ?? ?? 44 38 BF 99 01 00 00");
        int layout = act && head && *(const float *)(act + 14 + *(const int32_t *)(act + 10)) == 30.0f &&
                     *(const float *)(head + 8 + *(const int32_t *)(head + 4)) == 30.0f;
        if (!layout) logline("Cruise: cruise code not found: minimum speed, hazards, hill start, Auto Hold and ISA stay off");
        else {
            g_low_ok = 1;
            if (p_min_kmh > 0 && p_min_kmh < 30) {
                if (game_patch_rip_float(act + 7, 3, 7, p_min_kmh) && game_patch_rip_float(head, 4, 8, p_min_kmh))
                    logline("Cruise: on/resume from %.0f km/h, set speed at least %.0f km/h", p_min_kmh, p_min_set);
                else logline("Cruise: minimum speed patch failed, stays 30 km/h");
            }
            // The handler's xmm7 is also the ACC distance key's first threshold
            // (neutral-exit-re notes § 4): give that compare its own 30.0, and
            // let a press step from the player's distance, not from our value.
            uintptr_t ak = game_scan(sense_is_162() ? "48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? 0F 2F C6 76 ?? 48 8D 0D ?? ?? ?? ?? E8"   // 1.62: xmm6
                                                   : "48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? 0F 2F C7 76 ?? 48 8D 0D ?? ?? ?? ?? E8");
            if (ak) o_acc_read = (cvar_read_fn)(ak + 12 + *(const int32_t *)(ak + 8));
            if (ak && game_patch_comiss_xmm0(ak + 12, 30.0f) && game_patch_call(ak + 7, (void *)acc_read_stub))
                logline("Cruise: ACC distance key kept working");
            else logline("Cruise: ACC distance key code not found: it may only switch to 30 now");
        }
    }

    // Cruise keys: retarder off on a press, tap / hold steps (cruise-keys-re notes). Four wrapped consume calls.
    if (p_step_tap > 0 || p_ret_off > 0) {
        static const char *pat_tog = sense_is_161() ? "48 85 C9 74 12 83 B9 F8 00 00 00 02 75 09 E8 ?? ?? ?? ?? B1 01 EB 02 32 C9 84 C9 74 1A 80 BF 04 02 00 00 00 75 0A"
                                                   : "48 85 C9 74 12 83 B9 F8 00 00 00 02 75 09 E8 ?? ?? ?? ?? B1 01 EB 02 32 C9 84 C9 74 1A 80 BF 99 01 00 00 00 75 0A";
        static const char *pat_k[3] = {
            sense_is_162() ? "48 8B 8F 28 E8 01 00 48 85 C9 74 ?? 83 B9 F8 00 00 00 02 75 ?? E8 ?? ?? ?? ?? B1 01"   // 1.62: +8
          : sense_is_161() ? "48 8B 8F 20 E8 01 00 48 85 C9 74 ?? 83 B9 F8 00 00 00 02 75 ?? E8 ?? ?? ?? ?? B1 01"
                          : "48 8B 8F 20 E7 01 00 48 85 C9 74 ?? 83 B9 F8 00 00 00 02 75 ?? E8 ?? ?? ?? ?? B1 01",
            sense_is_162() ? "48 8B 8F 38 E8 01 00 48 85 C9 74 ?? 83 B9 F8 00 00 00 02 75 ?? E8 ?? ?? ?? ?? B1 01"   // 1.62: +8
          : sense_is_161() ? "48 8B 8F 30 E8 01 00 48 85 C9 74 ?? 83 B9 F8 00 00 00 02 75 ?? E8 ?? ?? ?? ?? B1 01"
                          : "48 8B 8F 30 E7 01 00 48 85 C9 74 ?? 83 B9 F8 00 00 00 02 75 ?? E8 ?? ?? ?? ?? B1 01",
            sense_is_162() ? "48 8B 8F 48 E8 01 00 48 85 C9 74 ?? 83 B9 F8 00 00 00 02 75 ?? E8 ?? ?? ?? ?? B1 01"   // 1.62: +8
          : sense_is_161() ? "48 8B 8F 40 E8 01 00 48 85 C9 74 ?? 83 B9 F8 00 00 00 02 75 ?? E8 ?? ?? ?? ?? B1 01"
                          : "48 8B 8F 40 E7 01 00 48 85 C9 74 ?? 83 B9 F8 00 00 00 02 75 ?? E8 ?? ?? ?? ?? B1 01" };
        uintptr_t site[4] = { 0, 0, 0, 0 };
        uintptr_t t = game_scan(pat_tog);
        if (t) site[0] = t + 14;
        for (int i = 0; i < 3; i++) { uintptr_t h = game_scan(pat_k[i]); if (h) site[i + 1] = h + 21; }
        uintptr_t rt = game_scan("48 89 5C 24 08 48 89 74 24 18 89 54 24 10 57 48 83 EC 50 48 8B 41 18 48 8B F9 41 0F B6 F0 8B DA");
        int ok = 1;
        for (int i = 0; i < 4; i++) ok = ok && site[i] && *(const unsigned char *)site[i] == 0xE8;
        if (ok) {
            uintptr_t tg = site[0] + 5 + (intptr_t) * (const int32_t *)(site[0] + 1);
            for (int i = 1; i < 4; i++) ok = ok && tg == site[i] + 5 + (intptr_t) * (const int32_t *)(site[i] + 1);
            o_consume = (consume_fn)tg;
        }
        o_retarder = (retarder_fn)rt;
        for (int i = 0; ok && i < 4; i++) ok = game_patch_call(site[i], (void *)consume_stub);
        if (ok) {
            g_keys_ok = p_step_tap > 0;
            g_ret_ok = rt != 0 && p_ret_off > 0;
            logline("Cruise: keys in place (tap %.0f / hold %.0f km/h after %.1f s at %.0f/s%s; retarder off on set/resume: %s)", p_step_tap,
                    p_step_hold, p_hold_s, p_hold_rate, g_keys_ok ? "" : ", steps off", g_ret_ok ? "yes" : "no");
        } else logline("Cruise: key handler code not found: + / - steps and retarder-off stay the game's");
    }
    g_pcc_on = p_predict > 0;
    g_overspeed_cap = p_os_cap;
    g_overspeed_pct = sense_is_ats() ? p_os_pct : 0;
    g_speed_limiter_kmh = p_own_lim;       // the model's speed chooser reads these (core/speedtarget.cpp)
    g_car_trailer_lim_kmh = p_car_trail_lim;
    // The module runs inside the actuator's tick (core hooks the game's cruise fn call whatever this section says); our own
    // smooth controller replaces the game's throttle only with smooth = 1.
    cr_attach();
    g_cruise_smooth = p_smooth > 0 && act_hooked();
    if (p_engine > 0) logline(g_cruise_smooth ? "Cruise: engine = layers (the speed arbiter drives the CC; LIM on %d presses of SET%s)" : "Cruise: engine = layers needs the throttle hook: the classic loop stays", (int)(p_lim_presses + 0.5f), g_keys_ok ? "" : ", but the key hook is missing: no LIM");
    if (p_smooth > 0) logline(g_cruise_smooth ? "Cruise: own smooth cruise throttle on" : "Cruise: no throttle hook (see the Actuator lines): the game's cruise stays");
    // Auto Hold: the actuator patches the game's throttle gate (the merged throttle is zeroed while the brake request is > 0)
    // so the pedal builds torque against the brake while only the hold asks, and the release can follow it (autohold-acc notes).
    if (p_ah > 0 && act_hooked() && g_low_ok) {
        s_gate_ok = act_gate_ok();
        if (s_gate_ok) logline("Cruise: Auto Hold in place (throttle passes while it holds, hold %.2f, release %.2f-%.2f s)", p_ah_req, p_ah_hard, p_ah_gentle);
        else logline("Cruise: Auto Hold: throttle gate not found, the release is paced by the pedal only (no drive feedback)");
    } else if (p_ah > 0) logline("Cruise: Auto Hold needs the cruise code and the actuator's throttle hook: off (hill start stays)");
    // Lead speed: wrap the radar call in the ACC fn (the game keeps the lead's speed in a local only).
    if (L && L->a_cruise_on && L->a_lead_m) {
        uintptr_t rh = game_scan(sense_is_161() ? "48 8D 54 24 50 48 8B CB C6 83 89 01 00 00 01 E8 ?? ?? ?? ?? 0F 10 00 8B 40 10 89 44 24 48"
                                                : "48 8D 54 24 50 48 8B CB C6 83 21 01 00 00 01 E8 ?? ?? ?? ?? 0F 10 00 8B 40 10 89 44 24 48");
        uintptr_t rs = rh ? rh + 15 : 0;
        if (rs && *(const unsigned char *)rs == 0xE8) {
            o_radar = (radar_fn)(rs + 5 + (intptr_t) * (const int32_t *)(rs + 1));
            if (game_patch_call(rs, (void *)radar_stub)) {
                g_radar_ok = 1;
                logline("Cruise: lead speed hook in at %p (radar fn %p): own ACC %s", (void *)rs, (void *)o_radar, p_own_acc > 0 ? "on" : "off (hook only feeds the trace and the cut-in test)");
            } else logline("Cruise: lead speed hook failed: ACC target and cut-in test stay the game's / d(gap)/dt");
        } else logline("Cruise: radar call in the ACC fn not found: ACC target and cut-in test stay the game's / d(gap)/dt");
        // Own AEB (aeb notes): the ACC's emergency part sees no lead (its gap load becomes 0) and the game's AEB stores of
        // 1.0 into +0x17c are skipped while ours runs. Needs the actuator's hook in the cruise fn (that is where the AEB runs) and the radar.
        // (the hooks go in for a car whatever the ini says: the class profile decides its AEB; while it is off for the class of now they pass through)
        if ((p_aeb > 0 || k_vprof_car.aeb_on > 0) && g_radar_ok && act_hooked() && g_low_ok) {
            uintptr_t ae = game_scan(sense_is_161() ? "80 BE 04 02 00 00 00 0F 84 ?? ?? ?? ?? 48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? 0F 2F C6 0F 86 ?? ?? ?? ?? 44 0F 2F 05 ?? ?? ?? ?? 0F 87 ?? ?? ?? ?? 48 8B 46 18 F3 44 0F 10 86 18 02 00 00"
                                                    : "80 BE 99 01 00 00 00 0F 84 ?? ?? ?? ?? 48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? 0F 2F C6 0F 86 ?? ?? ?? ?? 0F 2F 3D ?? ?? ?? ?? 0F 87 ?? ?? ?? ?? 48 8B 46 18 F3 0F 10 BE A8 01 00 00");
            uintptr_t s1 = game_scan(sense_is_161() ? "41 0F 2F F8 73 14 44 0F 2F 8B 74 01 00 00 73 0A C7 83 E4 01 00 00 00 00 80 3F"
                                                    : "41 0F 2F F8 73 14 44 0F 2F 8B 0C 01 00 00 73 0A C7 83 7C 01 00 00 00 00 80 3F");
            uintptr_t s2 = game_scan(sense_is_161() ? "84 C0 75 0D 83 8B 7C 01 00 00 10 EB 1A 84 C0 74 16 C7 83 E4 01 00 00 00 00 80 3F"
                                                    : "84 C0 75 0D 83 8B 14 01 00 00 10 EB 1A 84 C0 74 16 C7 83 7C 01 00 00 00 00 80 3F");
            s_aeb_owns = 1;  // cleared again if any of the patches fails; s_aeb_watch is set per frame in aeb()
            if (ae && s1 && s2 && game_patch_zero_xmm_if(ae + (sense_is_161() ? 0x34 : 0x33), sense_is_161() ? 9 : 8, sense_is_161() ? 8 : 7, &s_aeb_watch) &&
                game_patch_skip_if(s1 + 16, 10, &s_aeb_watch) && game_patch_skip_if(s2 + 17, 10, &s_aeb_watch)) {
                void *eb = sense_cvar("g_emergency_brake"), *bs = sense_cvar("g_blind_spot");
                float ebv = eb ? sense_cvar_get(eb) : NAN, bsv = bs ? sense_cvar_get(bs) : NAN;
                logline("Cruise: own AEB in place (%s; warning %.1f s, partial %.1f s / %.1f m/s2, full %.1f s); the game's emergency braking is replaced only while ours watches (its AEB option: %.0f)",
                        p_aeb_manual > 0 ? "also without cruise" : "with cruise only", p_aeb_warn_s, p_aeb_part_s, p_aeb_part_dec, p_aeb_full_s, ebv);
                if (p_aeb_mode > 0)
                    logline("Cruise: AEB by distance (aeb_mode 1): warning from %.1f m/s2 needed, full braking from %.1f m/s2 (%.2f x %.1f), stop %.1f m behind the lead", p_aeb_warn_dec,
                            p_aeb_full_frac * p_aeb_max_dec, p_aeb_full_frac, p_aeb_max_dec, p_aeb_margin);
                if (p_aeb_manual > 0 && !(ebv > 0) && !(bsv > 0))
                    logline("Cruise: AEB: the game's radar only runs with an ACC distance, the blind spot or the emergency brake option on; with ACC off and both options off the AEB sees nothing without cruise");
            } else {
                s_aeb_owns = 0;
                logline("Cruise: AEB code not found%s: the game's emergency braking stays", ae && s1 && s2 ? " (patch failed)" : "");
            }
        }
    }
    if (p_trace > 0) {
        g_csv = open_out("cruise.csv", "w");
        if (g_csv) fputs("t,kmh,set,gap,radar_gap,lead_kmh,opening_ms,acc_game,acc_own,acc_used,wanted_gap,thr,req_ours,req_ga,ah_state,ah_r,in_thr,in_brake,gear,eff_clutch,rpm,grade,air_psi,aeb,ttc,a_need,a_cmd,decel,bend_kmh,a_lat\n", g_csv);
    }
    if (p_descent > 0) {
        g_smart = sense_cvar("g_cruise_control_smart");
        if (g_smart) g_smart_player = sense_cvar_get(g_smart);
        logline("Cruise: descent braking %s (player's smart overspeed %.1f km/h, used %.1f%s)", g_smart ? "on" : "off: cvar not found",
                g_smart_player, smart_base(), p_os_cruise >= 0 ? " from the ini" : "");
    }
    if (p_sg > 0 && g_cruise_smooth) {
        uintptr_t ped = game_scan(sense_is_162() ? "F3 41 0F 10 96 98 1D 00 00 F3 41 0F 10 8E 94 1D 00 00"   // 1.62: +8
                                : sense_is_161() ? "F3 41 0F 10 96 90 1D 00 00 F3 41 0F 10 8E 8C 1D 00 00"
                                                : "F3 41 0F 10 96 C4 1C 00 00 F3 41 0F 10 8E C0 1C 00 00");
        uintptr_t c1 = game_scan(sense_is_161() ? "C6 86 04 02 00 00 00 48 8B 0F 48 8B 01 FF 90 A0 00 00 00 84"
                                               : "C6 86 99 01 00 00 00 48 8B 0B 48 8B 01 FF 90 A0 00 00 00 84");
        uintptr_t c2 = game_scan(sense_is_162() ? "C6 86 04 02 00 00 00 48 8B 46 18 48 8B 88 50 01 00 00 80 B9 45 03"   // 1.62: drive +8
                               : sense_is_161() ? "C6 86 04 02 00 00 00 48 8B 46 18 48 8B 88 50 01 00 00 80 B9 3D 03"
                                               : "C6 86 99 01 00 00 00 48 8B 46 18 48 8B 88 50 01 00 00 80 B9 3D 03");
        if (ped && c1 && c2 && game_patch_feed_xmm1(ped, 18, &g_feed) && game_patch_skip_if(c1, 7, &g_sg_hold) &&
            game_patch_skip_if(c2, 7, &g_sg_hold)) {
            g_sg_ok = 1;
            logline("Cruise: Stop&Go in place (drives off %s)", p_sg_auto > 0 ? "within the set time" : "always");
        } else logline("Cruise: Stop&Go code not found");
    }
    g_acc = sense_cvar("g_acc");
    if (!g_acc) { logline("Cruise: g_acc not found, ACC gap stays the game's"); return; }
    g_acc_player = sense_cvar_get(g_acc);
    // A value we left in the profile (not 0/20/30/40): back to the distance the player last picked (remembered), else
    // the nearest (that turned short into middle: our 25.5 rounded to 30).
    if (g_acc_player == g_acc_player && !is_round(g_acc_player)) {
        static const float lv[4] = { 0, 20, 30, 40 };
        float r = s_acc_saved >= -1 ? lv[s_acc_saved + 1] : g_acc_player <= 0 ? 0 : g_acc_player < 25 ? 20 : g_acc_player < 35 ? 30 : 40;
        logline("Cruise: g_acc %.1f was ours, back to %.0f", g_acc_player, r);
        act_cvar_set("cruise", g_acc, r);
        g_acc_player = r;
    }
    if (g_acc_player == g_acc_player) g_level = level_of(g_acc_player);
    logline("Cruise: ACC gap by time (short %.1f / middle %.1f / long %.1f s), player's g_acc %.1f = %s", p_gap_s,
            p_gap_m, p_gap_l, g_acc_player, level_name[g_level + 1]);
}

// Hazards on/off the way the game's hazard key does it (0x14099d633, neutral-exit-re notes
// § 3): bit 0x10 of game_actor+0x1d0, flasher +0x340,
// blink phase +0x1e0 when no indicator runs. 1.60 offsets (g_low_ok).
// (the writer is act_hazards in core/act.h: bit 0x10 of the lights byte, the flasher, the blink phase; per layout, g_low_ok)
static int hazards_on(char *ga) { return (sense_lights(ga) & 0x10) != 0; }
// Emergency stop signal (Damien 11-10: ABS braking): braking harder than
// emergency_decel above emergency_kmh switches the hazards on; they go off
// when the truck pulls away again (throttle and speeding up for 1.5 s).
// Hazards the driver switched on are never touched.
static void emergency_hazards()
{
    char *ga = g_low_ok && (p_ess > 0 || vprof()->ess_on > 0) ? sense_actor() : NULL;
    if (!ga || T.paused) return;
    double now = wall_s();
    float kmh = fabsf(T.speed) * 3.6f;
    // A light vehicle needs a real emergency stop (a car brakes ~9 m/s2; 6 m/s2 is firm but not an emergency, and a truck's 4 m/s2
    // would fire on every hard stop) and also for an AEB partial/full braking. A truck keeps the ini thresholds.
    float dec = vp_or(vprof()->ess_dec, p_ess_dec), off = vp_or(vprof()->ess_off, p_ess_off);
    if (s_haz_ours && !hazards_on(ga)) s_haz_ours = 0;  // the driver switched them off
    // The speed test uses the highest speed of the last 2 s: the measured decel needs ~0.6 s to build up, and a stop
    // from 65 is already below 50 by then (ts4q 84.5 s: 4.3 m/s2 first seen at 52 km/h, hazards stayed off).
    if (kmh >= s_haz_peak_kmh || now - s_haz_peak_at > 2.0) { s_haz_peak_kmh = kmh; s_haz_peak_at = now; }
    if (!s_haz_ours) {
        s_haz_stopped = 0;
        s_haz_soft_at = 0;
        int hard = s_haz_peak_kmh > p_ess_kmh && (-T.accel > dec || (vprof()->ess_aeb_stage && AE.st >= AEB_PARTIAL));
        if (!hard) s_haz_hard_at = 0;
        else if (!s_haz_hard_at) s_haz_hard_at = now;
        else if (now - s_haz_hard_at > 0.3 && !hazards_on(ga)) {
            act_hazards(ga, 1);
            s_haz_ours = 1;
            s_haz_go_at = 0;
            logline("Cruise: emergency braking at %.0f km/h (%.1f m/s2): hazards on", kmh, -T.accel);
        }
        return;
    }
    // Like a real truck (UNECE: on from 4 m/s2, off below 2.5): not stopped and the braking eases off → off again;
    // brought to a full stop → they stay on until you drive on (Damien).
    if (kmh < 1) s_haz_stopped = 1;
    if (!s_haz_stopped) {
        if (-T.accel < off) { if (!s_haz_soft_at) s_haz_soft_at = now; } else s_haz_soft_at = 0;
        if (s_haz_soft_at && now - s_haz_soft_at > 0.3) {
            act_hazards(ga, 0);
            s_haz_ours = 0;
            s_haz_soft_at = 0;
            logline("Cruise: braking eased off before a stop: hazards off");
        }
        return;
    }
    int going = T.in_thr > 0.1f && T.accel > 0.3f;
    if (!going) s_haz_go_at = 0;
    else if (!s_haz_go_at) s_haz_go_at = now;
    else if (now - s_haz_go_at > 1.5) {
        act_hazards(ga, 0);
        s_haz_ours = 0;
        s_haz_stopped = 0;
        logline("Cruise: driving on: hazards off");
    }
}

// ---------------------------------------------------------------- AEB (aeb notes)
// The game's emergency braking comes in two kinds, both a step to (near) full brake that empties the air:
//  - the ACC's emergency part (cruise on, gap < half its D(v)): service brake (D/2 - gap) / (D/2), straight to the brakes
//    every tick (0x1409937ea); with the game's AEB option off (Damien's profile) this is what dumped the air in ts4o / ts4p /
//    ts4q (135 -> 25 psi in 4 s, 138 -> 5 psi in 1.3 s; no request 1.0 in any cruise.csv);
//  - the AEB option (g_emergency_brake 1/2): brake request +0x17c = 1.0 (0x140995e52, 0x140995ee5).
// Ours, the ladder of real truck systems (UN R131, Mercedes ABA 5): warning tones, then partial braking (~3 m/s2), then
// full braking, from the radar lead (gap, lead speed), with or without cruise. The request follows the deceleration it
// takes (feed-forward from the ts4* logs plus a slow loop on the measured deceleration) and ramps, so it brakes once and
// steadily instead of pulsing. Crossing traffic, a fresh target, the indicator, the driver's own braking and kick-down
// keep it at the warning. Both game parts are skipped only while ours is watching (s_aeb_watch).
static const char *const aeb_name[] = { "off", "warning", "partial braking", "FULL braking", "holding" };

// One alarm that climbs: each step sounds at once when it is reached and repeats while it lasts. A step back down
// (the danger eased, the gas pedal, a new target) stays silent, so the warning never sounds again after the braking.
static void aeb_beep(int st)
{
    static int top = AEB_OFF;  // highest step sounded in this event
    double now = wall_s();
    if (st == AEB_OFF) { top = AEB_OFF; return; }
    if (st > AEB_FULL || st < top) return;
    bool up = st > top;
    top = st;
    double every = st == AEB_WARN ? 1.5 : 0.8;  // the braking sounds last 0.4-0.7 s
    if (up || now - AE.beep_at > every) {
        play_cue(st == AEB_WARN ? CUE_AEB_WARN : st == AEB_PARTIAL ? CUE_AEB_PARTIAL : CUE_AEB_FULL);
        AE.beep_at = now;
    }
}

// The sideways reading for the log line of a stage: where the radar's lead sits against our path and the weight the stop was asked with ("" = not found in the list)
static const char *aeb_lat_text()
{
    static char b[120];
    if (!AE.lat_ok) return "";
    snprintf(b, sizeof b, "; to the %s of our path %.1f m, clearance %+.1f m, asked with %.0f %%", AE.lat_d >= 0 ? "left" : "right", fabsf(AE.lat_d), AE.lat_clr, AE.lat_w * 100);
    return b;
}

static void aeb_stage(int st, float gap, float kmh)
{
    double now = wall_s();
    if (AE.st == AEB_OFF && st != AEB_OFF) {
        AE.ev_t0 = now; AE.ev_kmh = kmh; AE.ev_min_gap = 1e9f; AE.ev_min_ttc = 1e9f; AE.ev_psi0 = T.air_psi;
        AE.ev_psi_min = T.air_psi; AE.ev_max = st; AE.ev_max_dec = 0; AE.ev_cap = NULL; AE.beep_at = 0;
    }
    if (st != AEB_OFF && st <= AEB_FULL && st > AE.ev_max) AE.ev_max = st;
    if (AE.st == AEB_OFF) AE.air_low = 0;
    if (st > AE.st && st <= AEB_FULL && p_aeb_mode > 0)
        logline("AEB: %s: lead %.1f m ahead at %.0f km/h (we %.0f), needs %.1f m/s2 to stop %.1f m behind it (braking at most %.1f), time to collision %.1f s (mass %.0f t)%s",
                aeb_name[st], gap, AE.vl * 3.6f, kmh, AE.a_need, p_aeb_margin, p_aeb_max_dec, AE.ttc, load_mass() / 1000, aeb_lat_text());
    else if (st > AE.st && st <= AEB_FULL)
        logline("AEB: %s: lead %.1f m ahead at %.0f km/h (we %.0f), time to collision %.1f s, needs %.1f m/s2 (mass %.0f t)%s",
                aeb_name[st], gap, AE.vl * 3.6f, kmh, AE.ttc, AE.a_need, load_mass() / 1000, aeb_lat_text());
    if (st == AEB_HOLD && AE.st != AEB_HOLD) {
        if (gap > 0) logline("AEB: stopped %.1f m behind the lead: holding until gas or brake", gap);
        else logline("AEB: stopped (the radar lost the lead on the way): holding until gas or brake");
    }
    if (st == AEB_OFF && AE.st != AEB_OFF) DEV_NOTE(aeb_over(AE.ev_max >= AEB_FULL ? 2 : AE.ev_max >= AEB_PARTIAL ? 1 : 0));
    if (st == AEB_OFF && AE.st != AEB_OFF)
        logline("AEB: over after %.1f s: highest %s, %.0f -> %.0f km/h, min gap %.1f m, min time to collision %.1f s, peak decel %.1f m/s2, air %.0f -> %.0f psi%s%s",
                now - AE.ev_t0, aeb_name[AE.ev_max], AE.ev_kmh, kmh, AE.ev_min_gap < 1e8f ? AE.ev_min_gap : 0.0f,
                AE.ev_min_ttc < 1e8f ? AE.ev_min_ttc : 0.0f, AE.ev_max_dec, AE.ev_psi0, AE.ev_psi_min, AE.ev_cap ? ", held back by: " : "", AE.ev_cap ? AE.ev_cap : "");
    if (st != AE.st) { AE.st_at = now; if (st != AEB_FULL) AE.no_soft = 0; }
    AE.st = st;
}

// AEB incident files (Damien 11-10: light logs normally, a full trace per incident with the local time, so a player can
// say "it was about that time"): 20 rows per second in a 10 s ring all the time; at an event the file starts with the
// ring and runs until 10 s after the event is over.
enum { AEB_RING = 200, AEB_COLS = 19 };
static float s_ring[AEB_RING][AEB_COLS];
static int s_ring_n, s_ring_i;
static FILE *s_inc;
static double s_inc_off_at, s_rec_at;
static void aeb_row(FILE *f, const float *r)
{
    fprintf(f, "%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%d,%.3f,%.2f,%.2f,%.2f,%.2f,%.3f,%.1f,%d,%.0f,%.1f,%.2f\n", r[0], r[1], r[2], r[3], r[4], r[5],
            r[6], (int)r[7], r[8], r[9], r[10], r[11], r[12], r[13], r[14], (int)r[15], r[16], r[17], r[18]);
}
static void aeb_record(float gap, float kmh, float vc, float ttc, float a_need)
{
    double now = wall_s();
    if (g_incident_mb <= 0 || now - s_rec_at < 0.05) return;
    s_rec_at = now;
    char *ga = sense_actor();
    unsigned char fl = ga ? sense_lights(ga) & 0x0c : 0;
    float r[AEB_COLS] = { (float)now, kmh, gap, AE.vl * 3.6f, vc, ttc < 99 ? ttc : 99.0f, a_need, (float)AE.st, AE.req, AE.a_cmd,
                          T.in_thr, T.in_brake, T.accel, T.in_steer, T.yaw * 360, (float)(fl == 4 ? 1 : fl == 8 ? -1 : fl ? 2 : 0), T.air_psi,
                          AE.lat_ok ? AE.lat_clr : 99.0f, AE.lat_ok ? AE.lat_w : 1.0f };
    memcpy(s_ring[s_ring_i], r, sizeof r);
    s_ring_i = (s_ring_i + 1) % AEB_RING;
    if (s_ring_n < AEB_RING) s_ring_n++;
    if (AE.st != AEB_OFF && !s_inc) {
        char stamp[32];
        s_inc = open_incident("incidents", "aeb", g_incident_mb, stamp, sizeof stamp);
        if (s_inc) {
            fprintf(s_inc, "# DazDrive %s AEB incident at %s (local time), %.0f t, log clock %.0f s (dazdrive.log); stage 0 off 1 warning 2 partial 3 full 4 hold%s; indicator 1 left -1 right 2 hazards\n", SUITE_VERSION, stamp,
                    load_mass() / 1000, now, p_aeb_mode > 0 ? " (aeb_mode 1: no partial; a_need is the deceleration it takes to stop aeb_margin_m behind the lead)" : "");
            fputs("t,kmh,gap_m,lead_kmh,closing_ms,ttc_s,a_need,stage,request,a_cmd,thr,brake,decel,steer,yaw_deg_s,indicator,air_psi,lat_clr_m,lat_w\n", s_inc);
            for (int k = 0; k < s_ring_n; k++) aeb_row(s_inc, s_ring[(s_ring_i - s_ring_n + k + AEB_RING) % AEB_RING]);
            logline("incident: aeb -> incidents\\aeb_%s.csv", stamp);
            DEV_NOTE(aeb_file(stamp));
        }
    } else if (s_inc) aeb_row(s_inc, r);
    if (AE.st != AEB_OFF) s_inc_off_at = 0;
    else if (s_inc) {
        if (!s_inc_off_at) s_inc_off_at = now;
        if (now - s_inc_off_at > 10) { fclose(s_inc); s_inc = NULL; s_inc_off_at = 0; }
    }
}

static void aeb(char *g, float dt)
{
    s_want_aeb = 0;
    if (!s_aeb_owns) { s_aeb_watch = 0; return; }
    if (!aeb_enabled()) {   // off for this class (a truck's ini, the year): the game's own emergency braking is left alone; a running event ends cleanly
        if (AE.st != AEB_OFF) aeb_stage(AEB_OFF, 0.0f, fabsf(T.speed) * 3.6f);
        AE.req = 0;
        s_aeb_watch = 0;
        return;
    }
    if (!(dt > 0 && dt < 0.5f)) { s_want_aeb = AE.req; return; }  // paused / hitch: keep what is set
    double now = wall_s();
    float v = T.speed, kmh = fabsf(v) * 3.6f;
    int on = sense_cruise_on(g);
    int active = (on || p_aeb_manual > 0) && T.engine_on && !T.paused && T.gear >= 0 && T.dgear >= 0 && v > -0.5f && !T.parking_brake;
    // Power (Sport) on a light vehicle: the AEB is off, the driver is in charge (trucks keep their AEB unchanged). The game's own
    // emergency braking is left alone too, exactly as when our AEB is off entirely; a running event ends cleanly.
    if (vprof()->aeb_off_in_sport && sense_mode() == MODE_POWER) {
        if (AE.st != AEB_OFF) aeb_stage(AEB_OFF, 0.0f, kmh);
        AE.req = 0;
        s_want_aeb = 0;
        s_aeb_watch = 0;
        return;
    }
    s_aeb_watch = active ? 1 : 0;  // not watching (cruise off with aeb_without_cruise = 0, reversing, ...): the game's own AEB is left alone
    // A light vehicle brakes harder and later than a truck: car figures from the class profile, the truck ini thresholds otherwise.
    float max_dec = vp_or(vprof()->aeb_max_dec, p_aeb_max_dec), part_dec = vp_or(vprof()->aeb_part_dec, p_aeb_part_dec);
    float warn_s = vp_or(vprof()->aeb_warn_s, p_aeb_warn_s), part_s = vp_or(vprof()->aeb_part_s, p_aeb_part_s), full_s = vp_or(vprof()->aeb_full_s, p_aeb_full_s);
    // A car brakes far harder, so its deceleration-based stage bars scale up with its braking capability (the share of full braking that
    // counts as "enough to warn / brake" is the truck's): a fast close while passing a slower car needs no warning at ~1.5 m/s2.
    float kb = vprof()->aeb_max_dec == vprof()->aeb_max_dec ? max_dec / p_aeb_max_dec : 1.0f;
    float heavy = fminf(2.0f, fmaxf(1.0f, load_mass() / 30000)), hs = sqrtf(heavy);  // heavier: slower to stop, so earlier
    // The gain the AEB sizes by: the model's start value (the fit: 10 m/s2 per request up to 30 t, the same force above); in aeb_mode 1 with the brakes
    // measured (a confident slot) the estimate, never above the start value and never under 2.6 m/s2 per request (a_top 3.0: an estimate that low is a
    // bad measurement more likely than brakes that weak). So the measurement can only make the AEB earlier and the request larger, for weak brakes (a
    // heavy trailer with poor brakes, a physics mod): a wrong capability is the dangerous error, the stages start by it. The time stages of aeb_mode 0
    // are tuned on their own (in the sim a larger request there made one 60 t case worse): they keep the start value.
    float g_aeb = vehicle_brake_prior();
    int brakes_good = 0;   // aeb_rising's soft start only with brakes MEASURED at least as good as the start value (Damien 27-10: less careful only when the brakes are known to do it)
    if (p_aeb_mode > 0 && vehicle_brake_gain() == vehicle_brake_gain()) {
        float gl = vehicle_brake_est();
        brakes_good = gl >= 0.98f * g_aeb;   // (0.98: the start value itself counts as good)
        g_aeb = fminf(fmaxf(gl, 2.6f), g_aeb);
    }
    float a_top = fminf(max_dec, 0.4f + g_aeb);  // aeb_mode 1: the most it can ask for: what the brakes deliver at full request
    // aeb_mode 1: a warning from aeb_warn_decel, full braking from aeb_full_frac of a_top (a heavy truck cannot brake as hard: it starts in proportion earlier)
    float f_a1 = p_aeb_full_frac * a_top, w_a1 = p_aeb_warn_dec * a_top / max_dec;
    // Evasive lane change (see below): one indicator and steering or turning to that side, compared with what the truck did before the warning
    unsigned char ev_fl = sense_lights(g) & 0x0c;
    int ev_side = ev_fl == 4 ? 1 : ev_fl == 8 ? -1 : 0;  // + = left, like the steering and the yaw rate
    // (aeb_mode 1: not while an evasion stands it down, else a held steering becomes the new straight ahead within a second and
    // the evasion "ends" mid lane change)
    if (AE.st == AEB_OFF && !(p_aeb_mode > 0 && now < AE.evade_until)) {
        float k = fminf(1, dt / 1.0f);
        AE.steer0 += (T.in_steer - AE.steer0) * k;
        AE.yaw0 += (T.yaw - AE.yaw0) * k;
    }
    int ev = p_aeb_evade > 0 && ev_side && ((T.in_steer - AE.steer0) * ev_side > p_aeb_evade || (T.yaw - AE.yaw0) * ev_side > 0.004f);
    // the radar lead (fresh, plausible); a jump in the gap is a new target
    int fresh = g_radar_ok && now - s_radar.at < 0.3 && s_radar.gap > 0.3f && s_radar.gap < 100 && s_radar.v >= 0 && s_radar.v < 70;
    float gap = fresh ? s_radar.gap : NAN;
    if (fresh) {
        if (!(AE.gap_prev == AE.gap_prev) || fabsf(gap - AE.gap_prev) > 3 + 0.15f * AE.gap_prev) {
            AE.age_t0 = now; AE.vl = s_radar.v; AE.rate = NAN; AE.a_l = 0; AE.vl_prev = NAN;
        } else {
            float r = (gap - AE.gap_prev) / dt;
            AE.rate = AE.rate == AE.rate ? AE.rate + (r - AE.rate) * fminf(1, dt / 0.4f) : r;
        }
        // following closely (< 1 s): faster filters, so a lead that brakes is seen ~0.4 s sooner (more radar noise, but at
        // that distance every tenth counts)
        int close = v > 5 && gap / v < 1.0f;
        AE.vl += (s_radar.v - AE.vl) * fminf(1, dt / (close ? 0.1f : 0.25f));
        if (AE.vl_prev == AE.vl_prev) AE.a_l += ((AE.vl - AE.vl_prev) / dt - AE.a_l) * fminf(1, dt / (close ? 0.2f : 0.5f));
        AE.vl_prev = AE.vl;
        AE.gap_prev = gap;
        AE.lost_at = 0;
    } else {
        AE.gap_prev = NAN; AE.rate = NAN; AE.vl_prev = NAN; AE.a_l = 0; AE.lat_ok = 0;
        if (!AE.lost_at) AE.lost_at = now;
    }
    // the threat: time to collision and the deceleration it takes to stop 3 m behind the lead (also if the lead brakes to a stop)
    float vc = 0, ttc = INFINITY, a_need = 0, pl_v = 0, pl_room = 0, a_raw = 0, ttc_raw = INFINITY;   // (pl_*: the stop aeb_rising plans: the closing speed and the room to the margin, or our speed and the room plus the braking lead's own stop)
    int cap_st = AEB_FULL, crossing = 0;
    const char *cap = NULL;
    float p_s = part_s * hs, f_s = full_s * hs;
    if (fresh) {
        vc = v - AE.vl;
        // Stop point: the stand-off distance d(vl) behind the lead that the braking plan ends at. It is one formula, a fixed
        // cushion plus a time gap that grows with the lead's speed, interpolated between the two ends the rides anchored:
        //   d(vl) = 3.0 m                                                        for a standing lead (vl < 0.5 m/s)
        //   d(vl) = 1.5 + 0.1 vl + kv (1.5 + 0.2 vl) m                          for a moving one, kv = ramp(vl, 5, 9 m/s)
        // Derivation of the moving-lead form: at town speed (kv = 0) it is 1.5 m + 0.1 s x vl -- the close-quarters gap of the
        // 11-10 ride (tailgating at 25 km/h, 5 m behind a car pulling away: a stop there must NOT be an emergency, the gap is
        // the ACC's job). At highway speed (kv = 1, vl >= 9 m/s ~ 32 km/h) it is (1.5 + 1.5) + (0.1 + 0.2) vl = 3.0 m + 0.3 s x
        // vl -- the old 3 m + 0.3 s margin, kept so a stop beside a 40 km/h truck at 80 hands the driver room, not the next
        // emergency. kv fades the extra 1.5 m + 0.2 s in over the lead's 18 .. 32 km/h (5 .. 9 m/s), so the stand-off does not
        // shrink while our own speed drops under braking (sim "slow truck" ended 1.6 m behind). The 0.3 m floor keeps the
        // braking distance below (a_need = vc^2 / (2 room)) finite when the gap has already been eaten.
        float kv = fminf(1, fmaxf(0, (AE.vl - 5.0f) / 4.0f));
        float room = fmaxf(0.3f, gap - (AE.vl < 0.5f ? 3.0f : 1.5f + 0.1f * AE.vl + kv * (1.5f + 0.2f * AE.vl)));
        a_need = vc > 0 ? vc * vc / (2 * room) : 0;
        // a lead that pulls away closes the gap less and less: its acceleration counts (time to collision too)
        float la = AE.vl >= 0.5f && AE.a_l > 0.3f ? AE.a_l - 0.3f : 0;  // 0.3: radar speed noise
        if (la > 0) a_need = fmaxf(0, a_need - la);
        float b = -AE.a_l - 0.5f;  // the lead brakes (0.5: radar speed noise)
        if (AE.vl < 0.5f) a_need = fmaxf(a_need, v > 0 ? v * v / (2 * room) : 0);
        else if (b > 0.3f) a_need = fmaxf(a_need, v > 0 ? v * v / (2 * (room + AE.vl * AE.vl / (2 * b))) : 0);
        // aeb_mode 1: the deceleration that stops us aeb_margin_m behind the lead (a standing lead, or one that holds its speed;
        // a lead that brakes is added as its own stopping distance); no fixed margin, no credit for a lead that pulls away
        if (p_aeb_mode > 0) {
            float room1 = fmaxf(0.2f, gap - p_aeb_margin);
            a_need = vc > 0 ? vc * vc / (2 * room1) : 0;
            pl_v = vc > 0 ? vc : 0; pl_room = fmaxf(0.2f, room1 - 0.3f);   // (0.3 m: the soft start spends a little room, the reserve gives it back)
            if (AE.vl >= 0.5f && b > 0.3f && v > 0) {
                float an2 = v * v / (2 * (room1 + AE.vl * AE.vl / (2 * b)));
                if (an2 > a_need) { a_need = an2; pl_v = 0; }   // (a lead that brakes: its own stop is an estimate, no soft start)
            }
        }
        ttc = vc > 0.3f ? gap / vc : INFINITY;
        if (la > 0 && vc > 0.3f) { float d = vc * vc - 2 * la * gap; ttc = d < 0 ? INFINITY : (vc - sqrtf(d)) / la; }  // gap = vc t - la t^2 / 2
        // Is the radar's lead in our path? The radar says how far and how fast, never how far to the side: on 14-11 it offered the trailer of the truck we had just
        // passed (2.4 - 4.3 m beside us, 0.4 m "ahead") and the AEB braked fully at 67 - 90 km/h with the driver on the gas and the indicator on. The traffic list
        // knows where the vehicle is (core/aeblat.h): the weight w is 1 for a box in our path and falls to 0 at a free lane's width. The stop is asked with w times
        // the authority the brakes have: a_need saturates at the most the brakes give (a lead 0.3 m away would ask 100 m/s2 of a stop that no brake makes) and is
        // scaled, the time to collision grows by 1/w, so the stages move with the evidence and not at a wall. Only looked at when it matters (closing, a stop
        // that asks something); not found in the list = w 1 = as before.
        a_raw = a_need; ttc_raw = ttc;
        AE.lat_ok = 0;
        if (vc > 0.3f && (a_need > 0.5f || ttc < 4.0f)) {
            traffic_around ta;
            traffic_around_get(&ta);
            const al_in li = { now, s_radar.at, gap, vc, fabsf(v), T.yaw, ev_side, AE.vl };
            const al_out lo = al_read(ta, li);
            if (lo.matched) {
                AE.lat_ok = 1; AE.lat_w = lo.w; AE.lat_clr = lo.clr; AE.lat_d = lo.d;
                a_need = lo.w * fminf(a_need, max_dec);
                ttc = lo.w > 0.02f ? ttc / lo.w : INFINITY;
            }
        }
        // crossing traffic (roundabouts, junctions): the gap closes as if the lead stood still while its speed says it drives;
        // it is gone in a moment. Only when it is really in front and close: brake.
        crossing = AE.rate == AE.rate && AE.vl > 3 && v > 3 && -AE.rate - vc > fmaxf(2.5f, 0.5f * AE.vl);
        crossing = crossing && !(gap < 8 && -AE.rate > 0.3f && gap / -AE.rate < 1.0f);
        if (crossing) { cap_st = AEB_WARN; cap = "crossing traffic"; }
        // a new target must be seen a moment first (a jump in the gap, a car swerving past), unless it is close and urgent
        // (a target that appears under 1 m ahead asking more than twice what the brakes can give is not a lead: it is the vehicle we are passing, seen at its corner)
        if (now - AE.age_t0 < 0.3 && (!(gap < 12 && a_need > 3) || (gap < 1.0f && a_raw > 2 * max_dec))) { cap_st = AEB_WARN; if (!cap) cap = "new target"; }
        // (aeb_mode 1: "< 10 m, < 1 s" is a crash at speed for a late system; there an indicator holds braking back only while the
        // driver also steers that way, else full braking comes as without it)
        if ((sense_lights(g) & 0x0c) && !(p_aeb_mode > 0 ? !ev && vc > 0.3f && a_need >= f_a1 : gap < 10 && ttc < 1.0f)) { cap_st = AEB_WARN; cap = "indicator"; }
        // a lead that drives near a junction (turning, merging): partial and full braking later
        float jm = route_junction_m();
        if (jm >= 0 && jm < 60 && AE.vl > 2) { p_s = fminf(p_s, 1.5f); f_s = fminf(f_s, 1.0f); }
        // the driver brakes enough by himself
        // a firm brake pedal ends the warning (Damien 25-10), but only while it does at least half of what is needed: a moderate press
        // against a stopped queue must not switch the AEB off until ttc 1.5 s
        if ((T.in_brake > 0.05f && -T.accel_fast >= 0.9f * a_need) || (T.in_brake > 0.3f && ttc > 1.5f && -T.accel_fast >= 0.5f * a_need)) { cap_st = AEB_OFF; cap = "driver brakes"; }
        // the ACC (cruise on) already brakes enough and there is time: leave it to the ACC (Damien 12-10: the AEB went off with
        // the ACC on and braking, and he thought the ACC did fine). The AEB is back as soon as that stops being true.
        else if (on && T.in_brake < 0.05f && ttc > 3.0f && -T.accel_fast >= 0.65f * a_need) { cap_st = AEB_OFF; cap = "ACC brakes"; }
    }
    // The only cancel (Damien 11-10): the driver gives gas. Kick-down, a firm pedal held 0.3 s, or the pedal pressed 0.25 further
    // than it was when the braking started (a pedal that was already held does not cancel by itself). Then 3 s no braking.
    if (AE.st < AEB_PARTIAL) AE.thr_at_brake = T.in_thr;
    if (T.in_thr > 0.5f) { if (!AE.thr_since) AE.thr_since = now; } else AE.thr_since = 0;
    if (T.in_thr > 0.8f || (AE.thr_since && now - AE.thr_since > 0.3) || (AE.st >= AEB_PARTIAL && T.in_thr > AE.thr_at_brake + 0.25f)) {
        if (AE.st >= AEB_PARTIAL && AE.st <= AEB_FULL && now >= AE.lock_until) logline("AEB: overruled by the gas pedal (%.0f%%)", T.in_thr * 100);   // (once per overrule: the lock below runs on for as long as the pedal is down)
        AE.lock_until = now + 3;
    }
    // (under 5 m the pedal does not cancel: a lead that close and closing is a crash; unless the sideways reading says it is not in our path, then it is a vehicle we pass)
    int gas = now < AE.lock_until && !(fresh && gap < 5 && vc > 0 && !(AE.lat_ok && AE.lat_w < 0.5f));
    if (gas) { cap_st = AEB_WARN; cap = "gas pedal"; }
    // Evasive lane change (Damien 11-10: "if it warns me and I indicate left and steer left, it must understand"): one
    // indicator and steering or turning to that side, compared with what the truck did before the warning, for 0.3 s ends
    // a warning or partial braking; 3 s no AEB for it, unless the lead is still dead ahead and very close.
    // (ev: computed at the top)
    if (!ev) AE.evade_since = 0;
    else if (!AE.evade_since) AE.evade_since = now;
    if (ev && now - AE.evade_since > 0.3 && AE.st >= AEB_WARN && AE.st <= AEB_PARTIAL && now >= AE.evade_until) {
        AE.evade_until = now + 3;
        logline("AEB: evasive lane change to the %s (steering %+.3f, turning %+.1f deg/s): stands down", ev_side > 0 ? "left" : "right",
                T.in_steer - AE.steer0, (T.yaw - AE.yaw0) * 360);
    }
    // aeb_mode 1 brakes late: its full braking starts where "dead ahead and very close" (< 10 m, < 1 s) already holds at any
    // closing speed under ~40 km/h, so that exception would brake into every evasion it warned for (sim: slow truck at 80,
    // full braking 6.9 m behind it, mid lane change). There the evasion holds while the driver keeps steering away; once he
    // stops (gives up, the lane is taken) and the stop takes full braking, it brakes, and keeps braking until he steers away again.
    int evade = now < AE.evade_until && !(fresh && (p_aeb_mode > 0 ? !ev && (AE.st == AEB_FULL || (vc > 0.3f && a_need >= f_a1)) : gap < 10 && ttc < 1.0f));
    if (evade) { cap_st = AEB_OFF; cap = "evasive lane change"; }
    if (cap && AE.st != AEB_OFF && !AE.ev_cap) AE.ev_cap = cap;
    // the stage it asks for, and the stage that may stay (hysteresis)
    int want = AEB_OFF, keep = AEB_OFF;
    // With the ACC on it brakes for the lead itself (up to ~3.5 m/s2): warn only when that is not going to be enough, like
    // a real ACC's take-over warning (Damien 11-10, ride 20:08:37: a jam on ACC, the ACC already braking 1.8 m/s2, and a
    // warning at 1.5 m/s2 needed). Partial and full braking stay the same.
    float w_ttc = on ? fminf(warn_s, 2.0f) : warn_s, w_a = on ? vprof()->aeb_warn_dec_on : vprof()->aeb_warn_dec_off;
    if (fresh && vc > 0.3f) {
        if (ttc < w_ttc * hs || a_need > w_a / hs) want = AEB_WARN;
        if (ttc < p_s || a_need > 2.5f * kb / hs) want = AEB_PARTIAL;
        if (ttc < f_s || a_need > 0.6f * max_dec / hs) want = AEB_FULL;
    }
    if (fresh && vc > -0.5f) {
        float k = 1.3f, ka = 0.75f;
        if (ttc < w_ttc * hs * k || a_need > w_a / hs * ka) keep = AEB_WARN;
        if (ttc < p_s * k || a_need > 2.5f * kb / hs * ka) keep = AEB_PARTIAL;
        if (ttc < f_s * k || a_need > 0.6f * max_dec / hs * ka) keep = AEB_FULL;
    }
    if (p_aeb_mode > 0) {
        // by the required deceleration only (instead of the above): a warning from w_a1, full braking from f_a1 (no partial stage);
        // braking goes on while the gap to the lead it braked for is still closing, so it brakes to a stop. A new target (a jump
        // in the gap: the lead changed lanes, another car) is judged by its own need: no braking carried over to a far one
        want = keep = AEB_OFF;
        if (fresh && vc > 0.3f) {
            if (a_need >= w_a1) want = AEB_WARN;
            if (a_need >= f_a1) want = AEB_FULL;
        }
        if (fresh && vc > -0.5f) {
            float ka = 0.75f;
            if (a_need >= w_a1 * ka) keep = AEB_WARN;
            // (braking goes on while the lead it braked for still closes -- unless the sideways reading says it has left the path)
            if (a_need >= f_a1 * ka || (AE.st == AEB_FULL && vc > 0.3f && AE.age_t0 < AE.st_at && !(AE.lat_ok && AE.lat_w < 0.5f))) keep = AEB_FULL;
        }
    }
    // Below aeb_min_kmh the AEB normally leaves it to the ACC (the ACC stops behind a standing lead). But a crawl toward a
    // standing lead that is too close must still be covered: at a crawl the time to collision is huge (the speed is a crawl),
    // so the distance (the gap) is what threatens. A stationary lead with a closing gap under ~2 m is a stop the ACC failed
    // to make, so the AEB brakes to a stop. "Failed to make" means the stop really is in danger: it takes at least 0.6 of the
    // warning deceleration to stop aeb_margin_m short of the lead (1.5 m/s2 at the defaults, scaled with the brakes). Without
    // that, pulling away at 4-5 km/h from a standstill 2 m behind a car (a stop takes 0.7 m/s2) braked and warned (log
    // 01-11 23:17: "needs 0.7 m/s2", 0.5 m/s2).
    if (AE.st <= AEB_WARN && kmh < p_aeb_kmh) {
        float a_low = vc > 0 && fresh ? vc * vc / (2 * fmaxf(0.2f, gap - p_aeb_margin)) : 0;
        if (AE.lat_ok) a_low *= AE.lat_w;   // (a standing vehicle beside the lane at a crawl is no stop to make)
        if (fresh && vc > 0.3f && AE.vl < 0.5f && gap < 2.0f && a_low >= 0.6f * w_a1) want = AEB_FULL;
        else want = AEB_OFF;
    }
    if (!fresh && AE.st >= AEB_PARTIAL && AE.st <= AEB_FULL && now - AE.lost_at < 0.7) keep = AE.st;  // lost it close in: keep braking a moment
    if (want > cap_st) want = cap_st;
    // gas (the driver's cancel) and crossing traffic (not a threat) end braking; the indicator, the driver's own braking and a
    // fresh target only keep it from starting (Damien 11-10: only gas cancels)
    int hard = gas || crossing || evade;
    if (keep > cap_st && (hard || AE.st < AEB_PARTIAL)) keep = cap_st;
    int st = AE.st;
    if (!active) st = AEB_OFF;
    else if (st == AEB_HOLD) {
        int moved_off = fresh && AE.vl > 1.5f && gap > 8;
        if (T.in_thr > 0.1f || T.in_brake > 0.1f || on || moved_off || v > 1) st = AEB_OFF;  // Auto Hold / the ACC / the driver take over
    } else if (st >= AEB_PARTIAL && v < 0.3f) st = AEB_HOLD;
    else if (want > st) { st = want; AE.calm_since = 0; }
    else if (st > cap_st && hard) { st = cap_st; AE.calm_since = 0; }  // at once
    else if (keep >= st) AE.calm_since = 0;
    else {
        if (!AE.calm_since) AE.calm_since = now;
        int lower = p_aeb_mode > 0 && st == AEB_FULL ? AEB_WARN : st - 1;  // no partial stage in aeb_mode 1
        if (now - AE.calm_since > (st >= AEB_PARTIAL ? 1.0 : 1.5)) { st = lower > keep ? lower : keep; AE.calm_since = now; }
    }
    AE.ttc = ttc_raw; AE.a_need = a_raw;
    if (st != AE.st) aeb_stage(st, fresh ? gap : 0.0f, kmh);
    if (st != AEB_OFF) {
        if (fresh) { AE.ev_min_gap = fminf(AE.ev_min_gap, gap); AE.ev_min_ttc = fminf(AE.ev_min_ttc, ttc_raw); }
        if (T.air_psi > 1) AE.ev_psi_min = fminf(AE.ev_psi_min, T.air_psi);
        AE.ev_max_dec = fmaxf(AE.ev_max_dec, -T.accel_fast);
    }
    aeb_beep(st);
    // Tailgating is the driver's business (Damien 11-10: like real cars, no nagging): no warning for a short gap, only when
    // it is really close (< 0.4 s, ~9 m at 80) for 2 s at speed, once; again after a normal gap (> 1 s).
    {
        float tg = fresh && v > 1 ? gap / v : 99;
        if (tg > 1.0f) AE.close_warned = 0;
        if (fresh && kmh > 40 && tg < 0.4f && vc > -1 && st == AEB_OFF) { if (!AE.close_since) AE.close_since = now; }
        else AE.close_since = 0;
        if (AE.close_since && now - AE.close_since > 2 && !AE.close_warned) {
            AE.close_warned = 1;
            play_cue(CUE_AEB_WARN);
            logline("AEB: very close behind the lead: %.1f m at %.0f km/h (%.2f s)", gap, kmh, tg);
        }
    }
    // the request: deceleration wanted -> brake request (ts4* logs, coasting ~0.4 m/s2: request 0.1 ~ 1.3 m/s2, 0.3 ~ 3.5),
    // heavier asks more; a slow loop on the measured deceleration trims it once the brakes have built up
    float a_cmd = 0, target = 0;
    // the floors fade out as the closing speed goes to 0 (else the partial stage brakes below the lead's speed); continuous in
    // the closing speed, so following a braking lead at a steady gap does not flip between two requests
    float fl = fminf(1, fmaxf(0, vc));
    if (st == AEB_PARTIAL) a_cmd = fminf(fmaxf(1.15f * a_need, fl * 0.7f * part_dec), 1.15f * part_dec);
    else if (st == AEB_FULL && p_aeb_mode > 0) {
        a_cmd = fminf(fmaxf(a_need, fl * 1.0f), a_top);  // what the stop takes, nothing more
        if (p_aeb_rise > 0 && brakes_good && fresh && pl_v > 0.3f) {
            // rising (aeb_rising): the plan that stops at the margin with a deceleration that grows by aeb_rise_rate per second; the level of now
            // is lower than the constant one while there is room and the end is harder. Close to what the brakes can do (a_need 0.6 to 0.9 of a_top, fully
            // from there) the soft start fades: at least that share of the constant deceleration the stop takes, a_top at the limit
            float ar = aeb_rise_decel(pl_v, pl_room, a_top, p_aeb_jerk), k = fminf(1.0f, fmaxf(0.0f, (a_need / a_top - 0.6f) / 0.3f));
            // brakes that do not deliver what was asked (a lighter or worn plant than the estimate, 0.45 s after the start: the build-up): no soft part for the rest of the event
            if (now - AE.st_at > 0.45 && -T.accel_fast < 0.85f * AE.a_cmd && !AE.no_soft) { AE.no_soft = 1; logv(1, "AEB: the brakes lag (%.2f of %.2f m/s2 after %.2f s): the constant profile", -T.accel_fast, AE.a_cmd, now - AE.st_at); }
            if (AE.no_soft) k = 1;
            a_cmd = fminf(fmaxf(fmaxf(ar, k * a_need), fl * 1.0f), a_top);
        }
    }
    else if (st == AEB_FULL) a_cmd = ttc < 0.8f * hs ? max_dec : fminf(fmaxf(1.4f * a_need, fl * part_dec), max_dec);
    if ((st == AEB_PARTIAL || st == AEB_FULL) && !fresh) a_cmd = fmaxf(a_cmd, AE.a_cmd);
    AE.a_cmd = a_cmd;
    if (st == AEB_HOLD) target = 0.4f;
    else if (a_cmd > 0) {
        float ff = (a_cmd - 0.4f) / g_aeb;
        if (now - AE.st_at > 0.6) AE.i = fminf(0.5f, fmaxf(-0.15f, AE.i + 0.08f * (a_cmd + T.accel_fast) * dt));
        target = fminf(1, fmaxf(0.05f, ff + AE.i));
        if (vprof()->air_brakes && T.air_psi > 1 && T.air_psi < air_edge_psi(90, AIR_MARGIN_SAVE) && st < AEB_FULL) target = fminf(target, 0.6f);  // save the air outside full braking (a car has hydraulic brakes, no air to save)
    } else AE.i = 0;
    float up = st == AEB_FULL ? 6.0f : 3.0f, dn = 1.5f;
    // low air: no pulsing while braking (each new application costs air), hold it steady (a car has no air tank)
    if (vprof()->air_brakes && (T.air_warning || (T.air_psi > 1 && T.air_psi < air_pause_psi()))) AE.air_low = 1;  // for the rest of this event
    if (AE.air_low && (st == AEB_PARTIAL || st == AEB_FULL) && vc > 0) dn = 0;
    // one application, not a pumping one: while braking and still closing in, small dips of the target are ignored (each
    // release and re-apply costs air; the sim pumped 56 times following at 7 m with the fast filters)
    if ((st == AEB_PARTIAL || st == AEB_FULL) && vc > 0 && target < AE.req && target > AE.req - 0.08f) target = AE.req;
    AE.req = target > AE.req ? fminf(target, AE.req + up * dt) : fmaxf(target, AE.req - dn * dt);
    if (AE.req < 0.005f && target <= 0) AE.req = 0;
    s_want_aeb = AE.req;
    aeb_record(fresh ? gap : -1.0f, kmh, vc, ttc_raw, a_raw);
}

static void cr_frame()
{
    // Trace for a bug report (Damien 25-10: "gas and retarder at once", then "log every kind of braking"): the throttle the engine gets
    // while anything brakes: the retarder, the engine brake, the service brake (pedal or our own request), Auto Hold, the AEB, the parking brake
    {
        static double gr_since, gr_log;
        char *gga = sense_actor();
        float brk = fmaxf(T.in_brake, T.eff_brake);
        int src = (T.retarder > 0 ? 1 : 0) | (T.motor_brake ? 2 : 0) | (brk > 0.05f ? 4 : 0) | (act_request() > 0.05f ? 8 : 0) |
                  (AH.r > 0.05f ? 16 : 0) | (AE.st > 0 ? 32 : 0) | (T.parking_brake ? 64 : 0);
        if (!T.paused && T.eff_thr > 0.1f && src && fabsf(T.speed) * 3.6f > 1.0f) {
            if (!gr_since) gr_since = wall_s();
            if (wall_s() - gr_since > 0.8 && wall_s() - gr_log > 5) {
                gr_log = wall_s();
                logv(2, "gas together with braking [%s%s%s%s%s%s%s]: %.0f km/h, engine throttle %.2f (pedal %.2f), brake %.2f, own request %.2f, retarder %d, cruise %s set %.0f, smart %.1f, grade %.1f%%, mode %d",
                     src & 1 ? "retarder " : "", src & 2 ? "engine-brake " : "", src & 4 ? "service-brake " : "", src & 8 ? "our-brake-request " : "",
                     src & 16 ? "auto-hold " : "", src & 32 ? "aeb " : "", src & 64 ? "parking-brake" : "", fabsf(T.speed) * 3.6f, T.eff_thr, T.in_thr, brk,
                     act_request(), (int)T.retarder, T.cruise > 0.5f ? "on" : "off", T.cruise * 3.6f, g_smart_ours == g_smart_ours ? g_smart_ours : -1.0f,
                     pitch_grade(), sense_mode());
            }
        } else gr_since = 0;
        (void)gga;
    }
    g_inc.gap_m = g_radar_ok && wall_s() - s_radar.at < 0.3 && s_radar.gap > 0.3f ? s_radar.gap : NAN;
    g_inc.flags = (g_inc.flags & ~(INC_AEB | INC_ISA)) | (AE.st != AEB_OFF ? INC_AEB : 0) | (p_isa > 0 ? INC_ISA : 0);
    DEV_NOTE(cruise(T.cruise > 0.5f, T.cruise * 3.6f, g_acc ? g_level : -1, g_radar_ok ? s_radar.gap : NAN, s_radar.v,
                    s_radar.at, p_isa > 0, AE.st, AH.st, s_dip));
    if (!T.paused) {  // say when cruise drops and why it might have, and when it is back (a driver who pressed nothing wants to know)
        static int was_on; static double off_at, brake_at, lead_seen_at; static float set_kmh;
        {
            char *lga = sense_actor();
            float ld = lga ? lead_gap_m(lga) : 0;
            if (ld > 0.5f && ld < 60) lead_seen_at = wall_s();
            if (s_rearm_active) {
                float kmh_r = fabsf(T.speed) * 3.6f;
                if (T.cruise > 0.5f || kmh_r > 5 || T.parking_brake || T.in_brake > 0.3f || wall_s() - s_rearm_at > p_sg_rearm || T.gear < 0) s_rearm_active = 0;
                else if (lga && T.in_thr > 0.05f && s_rearm_set > 5 && ld > 0.5f && ld < 60) {   // the first tap on the gas with the vehicle still ahead: the cruise is back on with the old set speed (no lead: you drive off yourself)
                    act_cruise_set(lga, s_rearm_set);
                    act_cruise_on(lga, 1);
                    s_rearm_active = 0;
                    logline("Cruise: back on by a tap on the gas at a standstill (set %.0f km/h)", s_rearm_set);
                }
            }
        }
        int is_on = T.cruise > 0.5f;
        if (is_on) set_kmh = T.cruise * 3.6f;
        if (T.in_brake > 0.05f || T.parking_brake) brake_at = wall_s();
        if (was_on && !is_on) {
            off_at = wall_s();
            DEV_NOTE(cc_off(!g_keys_ok ? "unknown" : off_at - s_key_at <= 0.5 ? "key" : off_at - brake_at <= 0.5 ? "brake" :
                            off_at - s_jn_cancel_at <= 0.5 ? (off_at - g_rb_cancel_at <= 1.0 ? "roundabout" : "junction") : AE.st != AEB_OFF ? "aeb" : "other"));
            // Damien 13-10: a sound when the cruise drops by itself; not for his brake or his keys (he did it himself), not for
            // our junction cancel (it has its own) or the AEB (its alarm says enough). Without the key hook we cannot tell: silent.
            if (g_keys_ok && off_at - s_key_at > 0.5 && off_at - brake_at > 0.5 && off_at - s_jn_cancel_at > 0.5 && AE.st == AEB_OFF) {
                play_cue(CUE_CRUISE_DROP);
                logv(1, "Cruise: dropped without brake or key: sound");
            }
            // dropped at a standstill behind a vehicle with nobody having touched a key or the brake (the game cancels the cruise in neutral): keep
            // the old set speed for the first gas tap (Damien 25-10)
            if (p_sg_rearm > 0 && p_sg > 0 && fabsf(T.speed) * 3.6f < 3 && lead_seen_at > 0 && off_at - lead_seen_at < 3 && off_at - s_key_at > 0.5 && off_at - brake_at > 0.5 &&
                !T.parking_brake && AE.st == AEB_OFF) {
                s_rearm_at = off_at; s_rearm_set = set_kmh; s_rearm_active = 1;
                logline("Cruise: dropped at a standstill behind a vehicle: a tap on the gas or Resume puts it back on at %.0f km/h", set_kmh);
            }
            logline("Cruise: off at %.0f km/h (was set to %.0f): brake %.2f, throttle %.2f, gear %d, eco-roll %s, AEB %s, parking brake %s, air %.0f psi",
                    fabsf(T.speed) * 3.6f, set_kmh, T.in_brake, T.in_thr, T.gear, g_rolling ? "yes" : "no", aeb_name[AE.st], T.parking_brake ? "on" : "off", T.air_psi);
        } else if (!was_on && is_on && off_at > 0)
            logline("Cruise: on again after %.0f s (%.0f km/h, %.0f rpm)", wall_s() - off_at, fabsf(T.speed) * 3.6f, T.rpm);
        was_on = is_on;
    }
    {
        static double flushed;
        if (g_csv && wall_s() - flushed > 2) { flushed = wall_s(); fflush(g_csv); csv_cap(&g_csv, "cruise.csv"); }
    }
    route_junction_cfg(junc_on(), p_junc_lat, layers_on() ? p_rb_kmh * g_rb_factor : p_rb_kmh, p_junc_deg, p_rb_ratio, (int)(p_rb_curves + 0.5f));   // (layers: the roundabout's speed from the learned lean; classic: the ini's)
    route_hermite(p_herm > 0);
    if (g_keys_ok) keys_tick(sense_actor());
    emergency_hazards();
    // Speed-limit following on/off: hold the game's own ACC distance key (no wheel button number: the key is read from the
    // game's action object, so a wheel or a keyboard both work). The game already stepped the distance on the press, a long
    // press puts it back. Held = the action state is not 0; it must have been seen at 0 first (else a state that never
    // reads 0 would fire at start). A held key may flicker: 0.3 s up ends a hold.
    if (g_low_ok && p_isa_hold > 0 && !s_acc_key_bad) {
        double now = wall_s();
        char *ga = sense_actor();
        void *act = ga ? sense_action_at(ga, sense_key_off(4)) : NULL;
        int st = act ? sense_action_state(act) : 0;
        static int last_st = -1, n_st;
        if (act && st != last_st && n_st < 16) { n_st++; logv(1, "Cruise: ACC distance key state %d", st); }
        last_st = act ? st : -1;
        if (act && st == 0) s_acc_armed = 1;
        int held = act && s_acc_armed && st != 0;
        if (held && s_btn_down_at && now - s_btn_down_at > 6.0) {
            // nobody holds a key that long: the state does not go back to 0 on release. Stop reading it for this session and
            // undo the toggle it made, so the ACC gap code (which waits while the key is held) is not blocked.
            s_acc_key_bad = 1;
            if (s_btn_fired) { p_isa = p_isa > 0 ? 0 : 1; s_isa_last = -1; state_write(); }
            s_btn_fired = 0; s_btn_up_at = 0; held = 0;
            logline("Cruise: the ACC distance key state does not return to 0 (%d for %.0f s): holding it to switch speed-limit following is off for this session", st, now - s_btn_down_at);
            incident("input", "ACC distance key state stuck at %d for %.0f s", st, now - s_btn_down_at);
        } else if (!held) {
            if (!s_btn_up_at) s_btn_up_at = now;
            if (now - s_btn_up_at > 0.3) { s_btn_down_at = 0; s_btn_fired = 0; s_hold_before = NAN; }
        } else {
            s_btn_up_at = 0;
            if (!s_btn_down_at) {
                s_btn_down_at = now;
                s_hold_before = now - s_press_at < 1.0 ? s_press_before : NAN;  // else the key press is read a moment later
            }
            if (!s_btn_fired && now - s_btn_down_at >= p_isa_hold) {
                s_btn_fired = 1;
                float back = s_hold_before == s_hold_before ? s_hold_before : now - s_press_at < p_isa_hold + 1 ? s_press_before : NAN;
                if (g_acc && back == back) {
                    act_cvar_set("cruise", g_acc, back);
                    g_acc_player = back;
                    g_level = level_of(back);
                    g_acc_ours = NAN;
                }
                s_press_before = NAN;
                p_isa = p_isa > 0 ? 0 : 1;
                s_isa_last = -1;
                state_write();  // remembered
                play_cue(p_isa > 0 ? CUE_ON : CUE_OFF);
                disp_post(DISP_CRUISE, DISP_FLASH, p_isa > 0 ? 'I' : 'X', 1.5);  // I = ISA, following the speed signs, X = off (1.5 s); S is Power now
                logline("Cruise: speed limit following %s", p_isa > 0 ? "on" : "off");
            }
        }
        s_btn_held = held || (s_btn_up_at && now - s_btn_up_at <= 0.3);
    }
    // Following: at every new limit (and when cruise comes on) the set speed
    // becomes limit + offset; in between +/- are the driver's.
    if (g_low_ok) {
        char *ga = sense_actor();
        int on = ga && sense_cruise_on(ga);
        float limit_ms = isa_limit_ms();   // the car limit for a light vehicle where the game has both (ATS), else the truck limit
        if (!on || p_isa <= 0) s_isa_last = -1;
        else if (limit_ms > 1 && !layers_on()) {   // (layers: the cruise state follows the sign, in CC and in LIM)
            int lim = (int)roundf(limit_ms * 3.6f / unit_kmh());   // (the limit in whole units: km/h, or mph when the game shows mph)
            if (s_isa_last < 0 && wall_s() - s_res_press_at > 1.0) {
                // Switched on with set (not resume): the current speed stays, like the game; ISA takes over at the
                // next sign (Damien 11-10: "set goes straight to the ISA speed, set and resume do the same").
                s_isa_last = lim;
                logv(1, "Cruise: set at %.0f km/h with ISA on: kept, limit %d follows from the next sign", sense_cruise_set(ga), lim);
            } else if (lim != s_isa_last) {
                s_isa_last = lim;
                float set = isa_set_kmh(limit_ms, NULL);
                act_cruise_set(ga, set);  // lower or higher, at once (the game brakes on the set speed)
                logv(1, "Cruise: speed limit %d %s: set speed %.0f km/h", lim, cruise_units_mph() ? "mph" : "km/h", set);
            }
        }
        // Probe for the test ride: every change of the SDK limit dumps the route nodes (speedlimit-ahead-re notes).
        {
            static int probe_last = -1;
            int pl = limit_ms > 1 ? (int)roundf(limit_ms * 3.6f) : 0;
            if (p_isa_probe > 0 && pl != probe_last) { probe_last = pl; route_probe_limit((float)pl); }
        }
        // A lower limit AHEAD on the route: ease so that the set speed is limit + offset at the sign
        // (comfortable braking ahead_dec, so v(d) = sqrt(v_t^2 + 2 a d)). Needs route_limit_ahead (unknown source: NAN).
        {
            float dist = NAN, v_t = NAN;
            if (on && p_isa > 0 && p_isa_ahead > 0 && ga) {
                float v = fmaxf(fabsf(T.speed), 5.0f);
                float lim = route_limit_ahead(v * v / (2 * 0.5f) + 150, &dist);
                if (lim == lim && dist == dist && dist >= 0) {
                    int li = (int)roundf(lim / unit_kmh());
                    v_t = fmaxf(li * unit_kmh() + isa_offset_units(li) * unit_kmh(), p_min_set) / 3.6f;
                    if (li != s_ahead_logged) { s_ahead_logged = li; logv(2, "Cruise: limit %d ahead in %.0f m", li, dist); }
                    if (s_ahead_ours == s_ahead_ours && fabsf(sense_cruise_set(ga) - s_ahead_ours) > 0.5f) s_ahead_ignore = li;  // the driver changed it
                    float v_ideal = sqrtf(v_t * v_t + 2 * 0.5f * dist) * 3.6f;
                    if (s_ahead_ignore != li && v_ideal < sense_cruise_set(ga) - 0.1f && sense_cruise_set(ga) > v_t * 3.6f) {
                        act_cruise_set(ga, fmaxf(v_t * 3.6f, v_ideal));
                        s_ahead_ours = sense_cruise_set(ga);
                    } else s_ahead_ours = NAN;
                } else { s_ahead_ours = NAN; s_ahead_logged = -1; s_ahead_ignore = -1; }
            } else { s_ahead_ours = NAN; s_ahead_logged = -1; s_ahead_ignore = -1; }
        }
    }
    // Switched on below min_set_kmh (or set lower with -): set speed up to min_set_kmh.
    if (g_low_ok && p_min_set > 0) {
        char *ga = sense_actor();
        if (ga && sense_cruise_on(ga) && sense_cruise_set(ga) < p_min_set) act_cruise_set(ga, p_min_set);
    }
    // Our own speed limiter: the set speed never above it
    if (p_own_lim > 0) own_limit_set(sense_actor());
    // The game's speed limiter on (g_use_speed_limiter): the set speed never above it (Damien: + / hold went to 110). A light
    // vehicle has no truck limiter; a light vehicle with a trailer keeps the car trailer cap instead (car_trailer_limit_kmh).
    if (g_low_ok) {
        float cap = limiter_cap();
        static float s_cap_logged = -1;
        if (fabsf(cap - s_cap_logged) >= 0.5f) { s_cap_logged = cap; logv(1, "Cruise: set speed cap %.0f km/h (%s)", cap, cap <= 0 ? "none" : vprof()->trailer_cap ? "car trailer cap" : p_limiter > 0 && cap == p_limiter ? "limiter_kmh" : "the game's speed limiter"); }
        if (cap > 0) {
            char *ga = sense_actor();
            if (ga && sense_cruise_on(ga) && sense_cruise_set(ga) > cap) act_cruise_set(ga, cap);
        }
    }
    // Descent: on or just before a descent of 2 % or more, the game's own
    // overspeed braking (retarder / engine brake) starts right at the set
    // speed (smart = 0) instead of at the player's margin; back afterwards.
    if (g_smart && g_smart_player == g_smart_player && !T.paused) {
        if (wall_s() - s_desc_at > 0.5) {
            s_desc_at = wall_s();
            float v = fmaxf(fabsf(T.speed), 5.0f), ahead = route_ok() ? route_min_grade(0, v * 10) : NAN;
            // hysteresis (ts4e: on/over every second on a rolling descent, each time a brake application):
            // on at -2 %, over only after 5 s above -1 %; and not while the brake air is low
            int steep = pitch_grade() < -2 || (ahead == ahead && ahead < -2);
            int flat = pitch_grade() > -1 && !(ahead == ahead && ahead < -1);
            if (steep) { s_down_now = 1; s_flat_since = 0; }
            else if (s_down_now && flat) { if (!s_flat_since) s_flat_since = wall_s(); else if (wall_s() - s_flat_since > 5) s_down_now = 0; }
            else s_flat_since = 0;
            int air_ok = !T.air_warning && !(T.air_psi > 1 && T.air_psi < air_pause_psi());
            int down = T.cruise > 0.5f && s_down_now && air_ok;
            // A dip (PCC, Damien 11-10): a climb follows the descent, so let the truck run up to the band over the set
            // speed and carry that into the climb instead of braking it away. Latched to the bottom (there the road
            // ahead climbs and no valley is left); dropped when the descent goes on with no climb in sight.
            float up = band_up(), vy = up > 0 && route_ok() ? route_valley_m(fmaxf(300, v * 30)) : -1;
            if (vy >= 0 && up >= 0.5f) s_dip = 1;  // a dip only when the climb after it needs the momentum
            else if (ahead == ahead && (ahead < -2 || (ahead < -1 && pitch_grade() < -1))) s_dip = 0;  // no bottom in sight and still going down
            if (!s_down_now || !route_ok()) s_dip = 0;  // no route: no dip (review 11-10)
            if (!s_down_now) s_dip_spent = 0;
            if (!s_dip || !down) s_dip_at = 0;
            else if (!s_dip_at) s_dip_at = wall_s();
            else if (wall_s() - s_dip_at > p_band_s && !s_dip_spent) { s_dip_spent = 1; logv(2, "Cruise: dip margin used for %.0f s: back to the set speed", p_band_s); }
            if (s_dip_spent) s_dip = 0;
            // never brake below what our own momentum target asks (Damien 11-10, second 0.6 ride: "gas and retarder at once":
            // the player's smart overspeed 0 braked at the set speed while the PCC target was set + up)
            float want = down ? (s_dip ? up : fmaxf(0.0f, p_desc_kmh) * band_scale(T.cruise)) : smart_dynamic(pitch_grade());
            want = fminf(fmaxf(want, s_mom_up + 0.5f * (s_mom_up > 0)), overspeed_room(T.cruise));
            // following a lead with our own ACC: the game's overspeed braking reads our ACC target (+0x1a0) plus this margin, so with a
            // margin it let us run up to ~8 km/h over the ACC target before braking (sim 30-10 with the game's real braking: down to
            // 0.5 m behind a lead slowing on a climb). While following, the game brakes at the ACC target itself.
            if (OA.on) want = 0;
            static int said = 0;  // 0 flat, 1 descent, 2 dip
            int st = down ? 1 + s_dip : 0;
            if (st != said) {
                if (st == 2) logv(2, "Cruise: dip ahead (bottom in %.0f m): up to +%.1f km/h into the climb", vy, up);
                else logv(2, "Cruise: descent %s", st ? "ahead: braking from the set speed" : "over");
                said = st;
            }
            if (!(g_smart_ours == g_smart_ours) || fabsf(want - g_smart_ours) > 0.01f) {
                act_cvar_set("cruise", g_smart, want);
                g_smart_ours = want;
            }
        }
    }
    // [planner] drive: the game's own overspeed braking must not fight the plan. Its threshold is max(T + smart - 0.25, T + 0.25) with
    // T = the ACC target (+0x1a0) while a lead is seen, else the set speed; keep smart so that threshold is at or above the plan's max
    // speed (v_overspeed_ok). Written every frame while the plan is fresh (it updates at rate_hz, faster than the 0.5 s block above).
    g_bend_lat = bend_lat();   // (route_profile's bends use the cruise's allowance)
    if (g_smart && g_smart_player == g_smart_player && !T.paused) {
        planner_drive pd = g_planner_drive;
        static int pl_raised;
        if (!(pd.on && pd.fresh && pd.ok)) { if (pl_raised) { pl_raised = 0; s_desc_at = 0; } }   // (review 30-10: stale or failed: the block above restores its own margin on the next frame, not 0.5 s later)
        else {
            pl_raised = 1;
            char *ga = sense_actor();
            float t_ms = (OA.on && ga) ? sense_acc_target(ga) : T.cruise;
            float need = fmaxf(0.0f, (pd.v_overspeed_ok_ms - fmaxf(t_ms, 0.0f)) * 3.6f + 0.5f);
            float want = fminf(need, overspeed_room(T.cruise));
            if (!(g_smart_ours == g_smart_ours) || fabsf(want - g_smart_ours) > 0.01f) {
                act_cvar_set("cruise", g_smart, want);
                g_smart_ours = want;
            }
        }
    }
    if (!g_acc || T.paused || s_btn_held) return;  // a distance press is announced when the button is released
    double now = wall_s();
    if (now - s_gacc_at < 0.5) return;
    s_gacc_at = now;
    // A round value we did not write: the player picked a distance in the options.
    float cur = sense_cvar_get(g_acc);
    if (cur == cur && !(fabsf(cur - s_gacc_seen) < 0.01f) && !(g_acc_ours == g_acc_ours && fabsf(cur - g_acc_ours) < 0.01f))
        logv(2, "Cruise: g_acc is now %g (was %g, s_ahead_ours %g)", cur, s_gacc_seen, g_acc_ours);
    s_gacc_seen = cur;
    if (cur == cur && is_round(cur) && !(g_acc_ours == g_acc_ours && fabsf(cur - g_acc_ours) < 0.01f)) {
        if (!(g_acc_player == g_acc_player) || fabsf(cur - g_acc_player) > 0.01f || g_acc_ours == g_acc_ours) {
            g_level = level_of(cur);
            logv(1, "Cruise: player picked ACC distance %.0f = %s", cur, level_name[g_level + 1]);
            if (s_acc_saved != g_level) { s_acc_saved = g_level; state_write(); }
            level_tone(g_level);
            // Damien 11-10: 1 close, 2 medium, 3 far, C = plain cruise (ACC off); or A (ACC) with the tones only
            disp_post(DISP_CRUISE, DISP_FLASH, p_acc_letter > 0 && g_level >= 0 ? 'A' : "C123"[g_level + 1], 1.5);
        }
        g_acc_player = cur;
        g_acc_ours = NAN;
    }
    if (!(T.cruise > 0.5f) || g_level < 0) {  // only while cruising, and never when the player set ACC off
        // not cruising: the player's own value back (ts3c started with our 45 saved in the profile)
        if (g_acc_ours == g_acc_ours && g_acc_player == g_acc_player) { act_cvar_set("cruise", g_acc, g_acc_player); g_acc_ours = NAN; }
        return;
    }
    float v = fmaxf(fabsf(T.speed), 8.0f);
    float gap_m = fminf(fmaxf(gap_s(v) * v, p_min_m), acc_max_gap());
    float want = roundf(gap_m / (0.25f + v / 30) * 2) / 2;  // g_acc for that gap at this speed
    if (is_round(want)) want += 0.1f;                         // never a value the menu could have set
    if (g_acc_ours == g_acc_ours && fabsf(want - g_acc_ours) < 2) return;
    act_cvar_set("cruise", g_acc, want);
    if (!(g_acc_ours == g_acc_ours) || fabsf(want - g_acc_ours) >= 10)
        logv(2, "Cruise: ACC gap %.0f m (%.1f s at %.0f km/h, g_acc %.1f)", gap_m, gap_s(v), fabsf(T.speed) * 3.6f, want);
    g_acc_ours = want;
}

static void cr_stop()
{
    if (s_inc) { fclose(s_inc); s_inc = NULL; }
    AH.r = 0; s_want_hold = 0; s_ah_take = 0; act_brake(ACT_HOLD, 0);   // (the actuator's own stop puts the request back to 0)
    if (g_csv) fflush(g_csv);
    if (g_smart && g_smart_player == g_smart_player) act_cvar_set("cruise", g_smart, g_smart_player);
    if (g_acc && g_acc_player == g_acc_player && g_acc_ours == g_acc_ours) {
        act_cvar_set("cruise", g_acc, g_acc_player);
        g_acc_ours = NAN;
    }
}

// Continuity broke (see RESET_*): nothing timed or filtered may carry over. SOFT clears the key timers, Auto Hold,
// easing anchors, filters, the PI start and the hysteresis states; HARD also the own-ACC state and puts the
// player's g_acc / smart-overspeed values back (they are re-derived from the live game afterwards).
static void cr_reset(int level, const char *why)
{
    (void)why;
    char *ga = g_low_ok ? sense_actor() : NULL;
    // keys
    for (int d = 0; d < 2; d++) K[d] = keydir();
    s_res_ev = 0;
    s_lv_prev = NAN; s_a_lead = 0;
    s_prev_on = ga ? sense_cruise_on(ga) : 0;
    s_btn_down_at = s_btn_up_at = 0;
    s_btn_fired = 0;
    s_acc_armed = 0; s_hold_before = NAN;
    s_press_before = NAN;
    s_btn_held = 0;
    // Auto Hold / hill start / our brake wishes back to 0 (the actuator's own reset puts the request back to 0)
    AH.st = AH_OFF; AH.r = 0; AH.released = 0; AH.launch_t = 0; AH.grab_until = 0; AH.roll_m = 0;
    s_want_hill = s_want_hold = s_want_curve = s_want_aeb = 0; s_ah_take = 0;
    for (int i = 0; i < BRQ_N; i++) s_ch[i] = 0;
    s_lim_braking = 0;
    act_brake(ACT_HILL, 0); act_brake(ACT_HOLD, 0); act_brake(ACT_BEND, 0); act_brake(ACT_AEB, 0);
    memset(&AE, 0, sizeof AE); AE.gap_prev = AE.rate = AE.vl_prev = NAN;  // AEB: no event spans a reset
    s_hs_armed = 0; s_hs_until = 0;
    s_sg_wait = 0;
    s_rearm_active = 0; s_dstar_sm = NAN;
    if (s_park_on && ga) park_release(ga, "reset");   // a parking brake we set does not outlive a teleport, truck change or load
    s_park_on = s_park_ok = 0; s_ah_hold_since = 0;
    disp_post(DISP_CRUISE, DISP_STATUS, 0, 0);
    s_ahead_ours = NAN; s_ahead_ignore = s_ahead_logged = -1;
    // calm cut-in / own-ACC filters
    s_lead_prev = -1; s_gap_rate = 0;
    s_lead_v = s_acc_game = s_acc_own = s_dstar = NAN;
    s_radar.at = 0;
    OA.on = 0;
    // PI: bumpless restart; stop&go and descent/curve hysteresis; the body's timers and filters
    C.on = 0; C.u = C.i = 0;
    CF.fm = 0; CF.k = 1; CF.u = 0; CF.to = 0; CF.at = -1;
    s_pl_was = 0; s_pl_tgt = 0;   // planner drive: a reset starts over from the blocks (or a fresh plan)
    sa_reset(&g_sa, 0); s_layers_ran = 0; s_nm_ran = 0; g_sl_nev = 0; g_cs.n_tog = 0; g_lim_last = NAN;   // layers: the arbiter and the gesture start over; the mode, on and the set survive a pause
    rb_now_reset(&g_rb_now); rp_reset(&g_rp); s_rj_hush = 0; g_rb_factor = 1.0f; RL.rb_out = 1.0f;          // layers: the road's requesters start over (the learned lean itself is the roll learning's)
    if (level >= RESET_HARD) { if (layers_on() && sl_lim_active()) { logline("Cruise (layers): the limiter is off (hard reset: %s)", why ? why : "?"); sl_cue(SLE_LIM_OFF); } sl_reset_all(); }
    g_sg_hold = 0; g_feed = 0; g_stopped_at = 0;
    s_curve_braking = 0; s_jn_braking = 0; s_desc_braking = 0; s_bn_init = 0; s_bn_on = 0; s_bn_cap = 0;
    s_acc_f = 0; s_acc_init = 0;
    s_steer_until = s_jn_ignore_until = 0; s_stuck_since = 0;
    s_cr_prev = s_cr_rate = NAN; s_cr_since = -1; s_cr_said = 0;
    memset(&OT, 0, sizeof OT);   // overtake assist: nothing timed or held spans a reset
    lm_reset(&g_lm); memset(&g_lmo, 0, sizeof g_lmo); ra_reset(&g_ra); memset(&g_rao, 0, sizeof g_rao); ot_reset(&g_ot); sg_reset(&g_sgs); s_acc_lay = 0;   // WP-SP3: the lead model, the ACC's requests, the assist and the wait start over
    s_down_now = 0; s_dip = 0; s_dip_spent = 0; s_dip_at = 0; s_flat_since = 0; s_desc_at = 0;
    s_gacc_at = 0;
    // emergency hazards: ours go off
    if (s_haz_ours && ga && hazards_on(ga)) act_hazards(ga, 0);
    s_haz_ours = 0; s_haz_hard_at = s_haz_go_at = 0;
    s_haz_stopped = 0; s_haz_soft_at = 0; s_haz_peak_kmh = 0; s_haz_peak_at = 0;   // (review 30-10: these were function-local and survived)
    s_lim_last = NAN; s_lim_last_at = -1;   // own_limit_cap's shift hold starts over
    if (level >= RESET_HARD) {
        s_isa_last = -1;
        OA.t = 0;
        s_ah_k = 1;
        s_gacc_seen = NAN;
        if (g_smart && g_smart_player == g_smart_player) act_cvar_set("cruise", g_smart, smart_base());
        g_smart_ours = NAN;
        if (g_acc && g_acc_player == g_acc_player && g_acc_ours == g_acc_ours) act_cvar_set("cruise", g_acc, g_acc_player);
        g_acc_ours = NAN;
    }
}

static const char *cr_status()
{
    static char b[320];
    if (p_enabled <= 0) return "off (ini)";
    const char *acc = g_acc ? (g_radar_ok && p_own_acc > 0 ? "own" : "game") : "off";
    snprintf(b, sizeof b, "smooth %s, ACC %s (gap %s), ISA %s (+%.0f), Auto Hold %s, Stop&Go %s, curves %s, junctions %s, descent %s, hazards %s, AEB %s; hooks: min-speed %s, keys %s, radar %s",
             g_cruise_smooth ? "on" : "off", acc, g_acc ? level_name[g_level + 1] : "-", p_isa > 0 ? "on" : "off", p_isa_off,
             p_ah > 0 && g_low_ok && act_hooked() ? "on" : "off", g_sg_ok ? "on" : "off",
             p_curve > 0 && g_cruise_smooth ? "on" : "off", junc_on() ? (p_junc > 0 ? "on" : "on (own ACC)") : "off", g_smart ? "on" : "off",
             (p_ess > 0 || vprof()->ess_on > 0) && g_low_ok ? "on" : "off", s_aeb_owns ? (p_aeb_manual > 0 ? "own (also without cruise)" : "own (with cruise)") : "game's", g_low_ok ? "yes" : "no", g_keys_ok ? "yes" : "no", g_radar_ok ? "yes" : "no");
    if (layers_on()) { size_t n = strlen(b); snprintf(b + n, sizeof b - n, "; engine layers (%s)", g_keys_ok ? "CC and LIM" : "CC only, no key hook"); }
    return b;
}

// How far over the set speed the game's cruise lets the truck run before its engine brake / retarder come in (km/h; NAN = not known):
// gp NAN = now (the smart overspeed we wrote, else the player's own setting); else what we would write on a grade of gp % (a descent of
// 2 % or more: descent_overspeed_kmh; a dip's band is not foreseen, so it errs low). An eco-roll that runs past it ends in those brakes
// (TrueShift's roll_gain looks 30 s ahead along the route)
float cruise_hold_kmh(float gp)
{
    if (gp != gp || !(g_smart_player == g_smart_player) || p_os_cruise < 0) return g_smart_ours == g_smart_ours ? g_smart_ours : g_smart_player;
    float w = gp < -2 ? fmaxf(0.0f, p_desc_kmh) * band_scale(T.cruise) : smart_dynamic(gp);
    return fminf(fmaxf(w, s_mom_up + 0.5f * (s_mom_up > 0)), overspeed_room(T.cruise));
}
float cruise_wish_share()
{
    double now = sim_s();
    if (!(p_force_mode > 0) || CF.at < 0 || now < CF.at || now - CF.at > 0.3) return NAN;
    return fminf(1.0f, fmaxf(0.0f, CF.u));
}
float cruise_wish_force(float v)
{
    double now = sim_s();
    if (!(p_force_mode > 0) || CF.at < 0 || now < CF.at || now - CF.at > 0.3) return -1;
    return CF.u * wish_full(v);
}

// Where the controller is heading (cruise_wish_ahead, core/suite.h): the larger of its output and the output it is rate-limited towards. The gear for a pull is chosen for the pull, not for the first
// tenth of it (the engage at 33 km/h with the set speed 90: the force wish starts near 0 and takes seconds to climb, and Eco skipped into the lug for it and kicked down four seconds later).
float cruise_wish_ahead(float v)
{
    const float f = cruise_wish_force(v);
    return f < 0 ? f : fmaxf(CF.u, CF.to) * wish_full(v);
}

void cruise_info_get(cruise_info *o)
{
    o->hold = AH.st;
    o->hold_req = AH.r;
    o->lever_take = s_ah_take;
    o->sg_hold = g_sg_hold;
    o->feed = g_feed;
}

// The planner shadow's read of the cruise (modules/planner.cpp): nothing here changes the cruise.
void cruise_plan_info_get(cruise_plan_info *o)
{
    float v = fabsf(T.speed);
    o->target_kmh = C.on ? s_target_dbg : NAN;
    o->level = g_level;
    o->time_gap_s = g_level >= 0 ? gap_s(v) : NAN;
    o->d0_m = stop_gap(v);
    char *g = sense_actor();
    float lead = g ? lead_gap_m(g) : 0;
    o->lead_m = lead > 0.5f ? lead : 0;
    o->lead_ms = g && o->lead_m > 0 ? lead_speed(g) : NAN;
    o->want_m = g_level >= 0 ? desired_gap(o->lead_ms == o->lead_ms ? o->lead_ms : v) : NAN;
}

// The phone's ACC distance buttons (dev.cpp {do:'acc_dist',dir}): one step closer (-1) or further (+1), no wrap. The player's pick is
// written to g_acc like the game's own key does; the frame code below notices it (cue, display letter, cruise_state.txt) as it does for the key.
int cruise_acc_step(int dir, const char **err)
{
    static const float lv[3] = { 20, 30, 40 };
    if (!g_acc) { *err = "ACC distance not available"; return 0; }
    int to = (g_level < 0 ? -1 : g_level) + (dir < 0 ? -1 : 1);
    if (to < 0) to = 0;
    if (to > 2) to = 2;
    if (to == g_level) return 1;   // at the end already
    act_cvar_set("cruise", g_acc, lv[to]);
    return 1;
}

// A live setting changed (config_live_set): what start() derived from the param is derived again. The rest is read every frame.
static const char *cr_applied(const char *key, float old)
{
    if (!strcmp(key, "engine")) { sl_reset_all(); return NULL; }
    if (!strcmp(key, "predictive")) g_pcc_on = p_predict > 0;
    else if (!strcmp(key, "overspeed_cap_kmh")) g_overspeed_cap = p_os_cap;
    else if (!strcmp(key, "overspeed_cap_pct")) g_overspeed_pct = sense_is_ats() ? p_os_pct : 0;
    else if (!strcmp(key, "speed_limit")) { s_isa_last = -1; state_write(); }   // as the key does: a new limit is taken at once, the choice is remembered
    else if (!strcmp(key, "auto_hold") && p_ah > 0) {
        if (g_low_ok && act_hooked()) s_gate_ok = act_gate_ok();
        else { p_ah = old; return "needs the throttle hook"; }   // review 30-10: put the old value back (a hold without the hook runs half-installed)
    }
    return NULL;
}

module m_cruise = { "cruise", "Cruise", g_params, sizeof g_params / sizeof g_params[0], cr_start, cr_frame, NULL,
                    cr_stop, cr_reset, cr_status, cr_applied, VCM_BOTH };
