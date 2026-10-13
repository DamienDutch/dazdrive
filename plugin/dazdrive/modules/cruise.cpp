// Cruise: the cruise control management module (0.3 concept).
//
// Step 1 (built): the adaptive cruise gap as a time. The game's ACC keeps
// D(v) = max(4, g_acc * (0.25 + v/30)) meters (v in m/s, docs/cruise-re.md),
// so g_acc is the gap at ~81 km/h and the gap in seconds shrinks at speed
// (17.5 -> 0.8 s at 80). This module sets g_acc so that D = time gap x speed.
// The options menu's three distances (20 / 30 / 40) stay the player's choice:
// short / middle / long time gap (idea Damien 11-10). We never write exactly
// 20, 30 or 40, so a round value means the player just picked a distance. The
// mode scales the gap (Eco a bit more room, Power a bit less). The player's
// own value is put back when the game closes.
//
// Step 2 (built, untested in game): our own smooth speed controller. The
// game's cruise throttle fn 0x140991c90 (one caller, docs/cruise-re.md) is an
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
#include "../core/route.h"
#include <math.h>
#include <string.h>

static float p_key_slide, p_acc_calm, p_set_ease, p_limiter, p_air_min, p_acc_up, p_acc_letter, p_ess, p_ess_dec, p_ess_off, p_ess_kmh, p_curve_lat, p_hill, p_hill_pct, p_hill_s, p_sg, p_sg_auto, p_descent, p_curve, p_curve_brake, p_min_set, p_isa, p_isa_off, p_isa_ramp, p_isa_hold, p_predict, p_min_kmh, p_enabled, p_gap_s, p_gap_m, p_gap_l, p_eco_k, p_power_k, p_min_m, p_max_m, p_smooth, p_step_tap, p_step_hold, p_hold_s, p_hold_rate, p_ret_off, p_res_lim, p_isa_ahead, p_isa_probe, p_junc, p_junc_lat, p_rb_kmh, p_junc_deg, p_rb_ratio, p_jn_dec, p_cv_coast, p_cv_brake, p_ah, p_ah_req, p_ah_gentle, p_ah_hard, p_ah_max, p_own_acc, p_own_brake, p_own_plan, p_own_gain, p_own_stop_m, p_own_dec, p_own_up, p_acc_cross, p_smart_kmh, p_desc_kmh, p_band_up, p_band_s, p_trace, p_aeb, p_aeb_manual, p_aeb_warn_s, p_aeb_part_s, p_aeb_full_s, p_aeb_part_dec, p_aeb_max_dec, p_aeb_kmh, p_aeb_evade;

static param g_params[] = {
    { "enabled", &p_enabled, 1, "1 = cruise module on" },
    { "acc_short_s", &p_gap_s, 1.0f, "adaptive cruise gap in seconds when the game option is the short distance (20). Real trucks: 2.4-3.6 s (Detroit default 3.6), but the game is scaled: 1 s feels normal" },
    { "acc_middle_s", &p_gap_m, 1.25f, "... the middle distance (30)" },
    { "acc_long_s", &p_gap_l, 1.5f, "... the long distance (40)" },
    { "acc_eco_factor", &p_eco_k, 1.0f, "Eco: the gap times this (more room, less brake-and-go)" },
    { "acc_power_factor", &p_power_k, 1.0f, "Power: the gap times this" },
    { "acc_min_m", &p_min_m, 15, "the gap never gets shorter than this (meters)" },
    { "acc_max_m", &p_max_m, 90, "and never longer than this" },
    { "speed_limit", &p_isa, 1, "speed-limit following at the first start (before any on/off choice is remembered): 1 = on (at every new limit the cruise set speed becomes limit + offset; +/- still work until the next sign); after that your last on/off choice is remembered (cruise_state.txt)" },
    { "speed_limit_hold_s", &p_isa_hold, 0.8f, "hold the ACC distance key of the game this long (s) to switch speed-limit following on/off; the distance stays as it was. No button number to set" },
    { "speed_limit_ease_kmh_s", &p_isa_ramp, 2.5f, "a lower limit: the set speed goes down this fast (km/h per second; 0 = at once, the game then brakes hard at the sign)" },
    { "speed_limit_offset_kmh", &p_isa_off, 3, "set speed = limit plus this (3: 80 -> 83)" },
    { "auto_hold", &p_ah, 1, "1 = Auto Hold: after stopping with the brake the truck stays held when you release the pedal (any slope, no timer) until you give gas; it lets go smoothly as the engine pulls. Not with crawl on, the parking brake, reverse or Stop&Go. With auto_hold on, hill_start is not used" },
    { "auto_hold_request", &p_ah_req, 0.6f, "how hard the hold brakes (brake request 0..1)" },
    { "auto_hold_gentle_s", &p_ah_gentle, 0.5f, "letting go after a gentle launch (light pedal) takes about this long (s)" },
    { "auto_hold_hard_s", &p_ah_hard, 0.15f, "after a hard launch (pedal 0.8+) about this long (s), still ramped" },
    { "auto_hold_max_s", &p_ah_max, 2.5f, "the hold never fights the pedal longer than this (s); then it is released at the hard rate" },
    { "hill_start", &p_hill, 1, "1 = (only with auto_hold 0) after stopping on a slope the brake holds for a moment when you release it, until you give gas" },
    { "hill_start_pct", &p_hill_pct, 3, "from this grade (%)" },
    { "hill_start_s", &p_hill_s, 3, "at most this long (s)" },
    { "stop_go", &p_sg, 1, "1 = adaptive cruise keeps going in a jam: stays on at a standstill behind a vehicle and drives off with it" },
    { "stop_go_auto_s", &p_sg_auto, 0, "after standing longer than this (s), driving off needs resume or the pedal; 0 = always drives off by itself (real trucks, MAN/Detroit: 2)" },
    { "descent", &p_descent, 1, "1 = with navigation: on a descent brake gently ahead (engine brake / retarder) so the set speed holds instead of overshooting" },
    { "curves", &p_curve, 1, "1 = with navigation: slow down gently before sharp bends and exits" },
    { "acc_cycle_up", &p_acc_up, 1, "1 = the ACC distance key steps further away: off, close, middle, far, off (the game: far, middle, close, off)" },
    { "acc_calm_cut_in", &p_acc_calm, 1, "1 = a vehicle cutting in or pulling away (the gap opening) does not make the cruise slow down, it only stops speeding up" },
    { "acc_letter", &p_acc_letter, 0, "dashboard after an ACC distance change: 0 = 1 / 2 / 3 (close, medium, far; C = plain cruise, ACC off), 1 = A (the tones tell the distance)" },
    { "emergency_hazards", &p_ess, 1, "1 = hazards on by themselves when braking very hard at speed (like an emergency stop signal); off again when you drive on" },
    { "emergency_decel", &p_ess_dec, 4.0f, "deceleration (m/s2) that counts as an emergency stop" },
    { "emergency_off_decel", &p_ess_off, 2.5f, "below this deceleration (m/s2) before the truck stops, the hazards go off again (real trucks: 2.5)" },
    { "emergency_kmh", &p_ess_kmh, 50, "only above this speed (km/h)" },
    { "air_min_psi", &p_air_min, 75, "below this brake air pressure (psi) the module stops its own brake requests (bends, crawl descent) and leaves descents to the engine brake" },
    { "curve_lat", &p_curve_lat, 3.125f, "sideways acceleration (m/s2) allowed in bends at about 92 km/h, the same in every mode (higher = faster through bends). Slower bends get more, up to 1.23x (4 m/s2), like the allowance in a Mercedes-Benz patent" },
    { "junctions", &p_junc, 1, "1 = with navigation: also slow down before junctions where the route turns, and before roundabouts (needs curves = 1; always on with own_acc = 1)" },
    { "junction_lat", &p_junc_lat, 2.5f, "sideways acceleration (m/s2) allowed in a junction turn (v = sqrt(this x turn radius), at least 15 km/h)" },
    { "junction_min_deg", &p_junc_deg, 25, "turns smaller than this (degrees) are not slowed for" },
    { "roundabout_kmh", &p_rb_kmh, 25, "speed at the entry of a roundabout, also when going straight through" },
    { "roundabout_ratio", &p_rb_ratio, 1.4f, "a prefab with 2+ route curves counts as a roundabout when the path through it is this many times its entry-exit chord (lower = more prefabs count)" },
    { "descent_overspeed_kmh", &p_desc_kmh, 2, "on a descent (descent = 1) the engine brake / retarder hold the set speed plus this (km/h; 0 = exactly the set speed)" },
    { "smart_overspeed_kmh", &p_smart_kmh, 3, "cruise downhill: how far over the set speed the truck may run before the engine brake / retarder hold it (the game's 'smart' cruise option, km/h); -1 = keep the game's setting (at 0 the retarder comes in the moment the truck is over the set speed). On a descent of 2 % or more descent_overspeed_kmh applies instead; never less than the predictive cruise (PCC) momentum asks for" },
    { "acc_ignore_crossing", &p_acc_cross, 1, "own ACC: a vehicle crossing in front (on a roundabout, at a junction) is passed, not followed: no braking or stopping for it, only no speeding up. Seen as a lead that drives (> 11 km/h) while the gap closes as if it stood still" },
    { "junction_plan_decel", &p_jn_dec, 1.15f, "junctions and roundabouts: the BRAKES are planned with this deceleration (m/s2); higher = later and firmer braking" },
    { "curve_brake_decel", &p_cv_brake, 1.05f, "bends (on/off-ramps): the brakes are planned with this deceleration (m/s2): only when it takes this much" },
    { "curve_coast_decel", &p_cv_coast, 0.4f, "bends and junctions: lifting off the throttle is planned with this gentle deceleration (m/s2), so the truck coasts first and brakes late" },
    { "curve_brake", &p_curve_brake, 1, "1 = and brake gently (engine brake / retarder first) when releasing the throttle is not enough" },
    { "predictive", &p_predict, 1, "1 = with navigation: carry a few km/h extra into a climb, ease off before a top with a descent after it (Eco most, Power not)" },
    { "band_up_kmh", &p_band_up, 6, "with predictive = 1 and navigation (predictive cruise, PCC): carry up to this much over the set speed into a climb, and in a dip (a descent with a climb right after it) let the truck run up to it before the engine brake / retarder hold it. At most this: only as much as the next climb needs (TrueShift: mass, engine, route; solo often 0); 0 = off" },
    { "band_max_s", &p_band_s, 40, "a dip may run over the set speed for at most this long, then the engine brake / retarder hold the set speed again (Mercedes PPC: 40 s)" },
    { "min_kmh", &p_min_kmh, 5, "lowest speed at which cruise control can be switched on or resumed (game: 30), e.g. in a jam" },
    { "key_slide_s", &p_key_slide, 0.15f, "your own +/- slides the set speed on the dash over this long (s), up and down alike; 0 = a jump" },
    { "set_ease_kmh_s", &p_set_ease, 2.5f, "a lower cruise set speed (-, resume, speed limit) is reached by easing down this fast (km/h per second; 0 = at once: the game brakes)" },
    { "limiter_kmh", &p_limiter, 90, "with the game's speed limiter on, the cruise set speed never goes above this (km/h; 0 = no cap)" },
    { "min_set_kmh", &p_min_set, 20, "the set speed never goes below this: switched on slower, it sets this (with ACC it follows the vehicle ahead)" },
    { "step_tap_kmh", &p_step_tap, 5, "a short tap on cruise + / - changes the set speed by this (rounded to a multiple of it: 83 -> 85); 0 = the game's own step" },
    { "step_hold_kmh", &p_step_hold, 1, "holding + / - changes it in steps of this (not rounded to the tap step)" },
    { "hold_s", &p_hold_s, 0.5f, "how long + / - must be held before it counts as holding (s)" },
    { "hold_rate", &p_hold_rate, 2, "while holding: steps per second" },
    { "retarder_off_on_set", &p_ret_off, 1, "1 = pressing cruise on / set / resume while the retarder lever is on first switches the retarder off (the game refuses or cancels cruise otherwise)" },
    { "resume_to_limit", &p_res_lim, 1, "1 = pressing resume while cruise is already on sets the speed limit + speed_limit_offset_kmh (only while limit following is on; lower eases down, higher at once)" },
    { "speed_limit_ahead", &p_isa_ahead, 1, "not working yet: the route data carries no speed limits, so this does nothing; kept for a later build (meant to start easing early for a lower limit ahead on the route)" },
    { "speed_limit_probe", &p_isa_probe, 0, "not working yet, debugging only: the route data carries no speed limits; 1 = at every speed-limit change write the route nodes around the truck to the log (max 30 dumps). Leave at 0" },
    { "own_acc", &p_own_acc, 1, "1 = own ACC speed target from the lead vehicle's gap and speed (read from the game's radar), smooth; the game's emergency braking stays. 0 = the game's ACC target" },
    { "own_acc_gain", &p_own_gain, 0.3f, "own ACC: m/s of speed offset per meter of gap error (target = lead speed + gain x (gap - wanted gap))" },
    { "own_acc_stop_m", &p_own_stop_m, 6, "own ACC: gap kept at a standstill (m); the wanted gap is this plus time gap x speed" },
    { "own_acc_plan_decel", &p_own_plan, 1.2f, "own ACC: approaching a slower lead is planned with this deceleration (m/s2), so the game's emergency braking (which empties the brake air) is not needed; 0 = off" },
    { "own_acc_brake", &p_own_brake, 0.35f, "own ACC: brake request (0..1) it may use itself when lifting off is not enough, so the game's emergency braking is not needed; 0 = never" },
    { "own_acc_decel", &p_own_dec, 1.2f, "own ACC: how fast the target may fall (m/s2; faster, up to 8x, when closing in needs more than 0.5 m/s2)" },
    { "own_acc_accel", &p_own_up, 0.8f, "own ACC: how fast the target may rise (m/s2)" },
    { "aeb", &p_aeb, 1, "1 = own emergency brake assist (AEB): warning tones, then partial braking, then full braking, all ramped; it replaces the game's ACC emergency braking and the game's AEB (both empty the brake air). 0 = the game's" },
    { "aeb_evade_steer", &p_aeb_evade, 0.01f, "AEB: the indicator to one side plus steering that way by this much (share of the full steering range, 0.01 = ~4.5 degrees on a 900-degree wheel; or the truck turning that way at 1.4 deg/s) for 0.3 s = an evasive lane change: a warning or partial braking stands down for 3 s. 0 = off" },
    { "aeb_without_cruise", &p_aeb_manual, 1, "1 = the AEB also watches when you drive yourself (cruise off); needs the game's radar, which runs with an ACC distance, blind spot or emergency brake switched on in the options" },
    { "aeb_warn_s", &p_aeb_warn_s, 3.0f, "AEB: warning tones when the time to collision drops below this (s), or earlier when more than ~1.5 m/s2 of braking would be needed (with cruise on: 2 s / 2.8 m/s2)" },
    { "aeb_partial_s", &p_aeb_part_s, 2.0f, "AEB: partial braking below this time to collision (s)" },
    { "aeb_full_s", &p_aeb_full_s, 1.2f, "AEB: full braking below this time to collision (s)" },
    { "aeb_partial_decel", &p_aeb_part_dec, 3.0f, "AEB: deceleration of the partial stage (m/s2; Mercedes ABA: ~3)" },
    { "aeb_max_decel", &p_aeb_max_dec, 6.5f, "AEB: the most it asks for in the full stage (m/s2)" },
    { "aeb_min_kmh", &p_aeb_kmh, 15, "AEB: only steps in above this speed (km/h); once braking it brakes to a stop. Slower is the ACC's job" },
    { "trace", &p_trace, 1, "1 = write cruise.csv (10 per second while cruise is on or Auto Hold holds) for analysis" },
    { "smooth", &p_smooth, 1, "1 = own smooth cruise throttle (no bursts after shifts; Eco gentle, Power keen), 0 = the game's" },
};
// Junction / roundabout slowing: always with the own ACC (friend 11-10: own ACC without it entered roundabouts at
// ~40 km/h and stopped there 4 times, probably for traffic on the roundabout; Damien: build it into the ACC)
static int junc_on() { return p_curve > 0 && (p_junc > 0 || p_own_acc > 0); }


// ---------------------------------------------------------------- smooth controller

typedef float (*cruise_fn)(void *ga, float dt, float pedal);
static cruise_fn o_cruise;
static struct { float i, u; int on; } C;

// State that cr_reset() must reach (was function-local).
static int s_air_low;                                   // brake air low: our requests paused
static double s_br_at;                                  // brake_request: time of the last call (slew)
static int s_hill_was, s_hold_was;                      // brake_request: hill-start / Auto Hold was asking last call
static int s_hs_armed;                                  // hill_start: stopped with the brake pressed on a slope
static double s_hs_until;                               // hill_start: hold until
static float s_lead_prev = -1, s_gap_rate;              // lead_rel_speed: gap filter
static float s_bend_coast_kmh = NAN;   // the route's bend speed (coast plan) this frame, for the trace (Damien 11-10: a sharp bend at full set speed)
static int s_curve_braking;                             // bend braking hysteresis
static int s_isa_last = -1;                             // the limit (km/h) we last set the cruise to
static double s_btn_down_at, s_btn_up_at;               // speed-limit button hold
static int s_btn_fired, s_acc_armed, s_acc_key_bad;
static float s_hold_before = NAN;  // the ACC distance when the hold of its key began (a held key may step it again)
static float s_ahead_ours = NAN;                        // limit ahead: the set speed we wrote
static int s_ahead_ignore = -1, s_ahead_logged = -1;
static double s_isa_at;                                 // ISA easing clock
static float s_ease_prev = NAN, s_ease_goal = NAN;      // set-speed easing anchor
static float s_anim_from = NAN, s_anim_to = NAN;        // the driver's +/- shown as a short slide on the dash
static double s_anim_t0;
static double s_ease_at;
static int s_down_now, s_dip, s_dip_spent;              // descent hysteresis; in a dip (climb after it); its time is up
static double s_dip_at;                                 // when the dip margin started
static double s_flat_since, s_desc_at;
static int s_haz_ours;                                  // emergency hazards we switched on
static double s_haz_hard_at, s_haz_go_at;
static double s_gacc_at;                                // g_acc poll clock
static float s_gacc_seen = NAN;

// Fields of game_actor (1.60, docs/cruise-re.md § 4).
enum { GA_ON = 0x199, GA_SET = 0x19c, GA_ACC_TARGET = 0x1a0, GA_LEAD_M = 0x1a8, GA_BRAKE_REQ = 0x17c };

// Stop&Go (docs/cruise-re.md § 4): at a standstill the automatic goes to
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

// Grade (%) from the SDK placement: pitch in turns, positive = nose up.
static float pitch_grade() { return tanf((float)T.place.orientation.pitch * 6.2831853f) * 100; }
// The band over the set speed (km/h): at most band_up_kmh (Damien 11-10: above may always, +6), but only as much as the
// next climb needs (TrueShift works it out from mass, engine and route: solo often 0, loaded more). Without TrueShift's
// answer: scaled by mass as before (full at 40 t, 40 % at 12 t).
static float s_mom_up;   // km/h over the set speed the PCC momentum aims for now (the game's overspeed braking must allow it)
static int s_lead_near;  // a vehicle within ~4 s ahead (cruise_body): no PCC momentum, it would only be braked away
static float band_up()
{
    if (!(p_predict > 0 && p_band_up > 0) || s_lead_near) return 0;
    float need = g_pcc_need_kmh;
    if (need == need) return fminf(p_band_up, need);
    static float m = 30000; static double at = -10;
    if (wall_s() - at > 2 || wall_s() < at) { at = wall_s(); float g = game_mass(); if (g == g && g > 500) m = g; }
    return p_band_up * (0.4f + 0.6f * fminf(1.0f, fmaxf(0.0f, (m - 12000) / 28000)));
}

// Hill-start: stopped on a slope, the driver lets go of the brake: hold it
// through the game's brake request (game_actor+0x17c; the game slews to it
// and zeroes the throttle while it is > 0) until the pedal, a timeout or
// moving. Runs every tick (the cruise fn is called with cruise off too).
// The brake request is never cleared by the game when its emergency brake
// option is off: what we write stays (ts3c: brakes on, hazards forced on,
// throttle dead, docs/neutral-exit-re.md § 3). So all our wishes meet here,
// once per tick, and what we wrote goes back to 0 when we no longer want it.
static float s_want_hill, s_want_hold, s_want_curve, s_want_acc, s_want_aeb, s_req_ours;
static int s_aeb_was;                                  // brake_request: the AEB was asking last call (it ramps by itself)
static float s_lv_prev = NAN, s_a_lead;                 // own ACC: lead speed history (reset without a lead)
static unsigned char s_thr_pass;     // Auto Hold alone holds the brake: the driver's throttle still reaches the engine (patch of the game's 0x17c throttle gate)
static unsigned char s_hazard_mute;  // our request must not switch the hazards on (game rule for 0x17c > 0)
static void brake_request(char *g, float want)
{
    float *req = (float *)(g + GA_BRAKE_REQ);
    // Save the air (Damien 11-10: the tank ran empty): no brake requests of ours while the pressure is low; a
    // request that holds still (hill start) is kept, it uses no new air.
    if (T.air_warning || (T.air_psi > 1 && T.air_psi < p_air_min)) {
        if (!s_air_low) {
            logline("Cruise: brake air low (%.0f psi): our braking paused", T.air_psi);
            if (fabsf(T.speed) * 3.6f > 10)  // standing after a start or a teleport the tank is simply still filling
                incident("air", "brake air low: %.0f psi at %.0f km/h, our braking paused", T.air_psi, fabsf(T.speed) * 3.6f);
        }
        s_air_low = 1;
    } else if (s_air_low && T.air_psi > p_air_min + 15) { s_air_low = 0; logline("Cruise: brake air back (%.0f psi)", T.air_psi); }
    // the AEB is never paused: it brakes with what is left (and holds its request steady instead of pulsing)
    if (s_air_low && s_want_hill <= 0 && s_want_hold <= 0 && s_want_aeb <= 0) want = 0;
    // a small change is not worth a new brake application (each one costs air): hold it. Auto Hold shapes its own ramp.
    if (s_want_hold <= 0 && s_want_aeb <= 0 && want > 0 && s_req_ours > 0 && fabsf(want - s_req_ours) < 0.05f) want = s_req_ours;
    // soft onset and release (Damien: subtler, smoother); the hill-start hold comes at once
    {
        double now = wall_s(), dt = now - s_br_at;  // the hill-start hold lets go at once too (else it fights the driver's gas)
        s_br_at = now;
        if (s_want_hill > 0) s_hill_was = 1;
        else if (s_want_aeb > 0 && s_want_aeb >= want - 1e-6f) { s_aeb_was = 1; s_hill_was = 0; }  // the AEB ramps by itself (up fast, down at its own pace)
        else if (s_aeb_was && want <= 0) s_aeb_was = 0;                  // ... its last step to 0 is its own
        else if (s_want_hold > 0) { s_hold_was = 1; s_hill_was = 0; }  // Auto Hold ramps by itself: no extra filter
        else if (s_hold_was) s_hold_was = 0;                           // ... and its last step to 0 is not dragged out either
        else if (s_aeb_was) s_aeb_was = 0;                             // a bigger wish took over from the AEB
        else if (s_hill_was) { s_hill_was = 0; want = 0; }
        else if (dt > 0 && dt < 0.5) {
            float up = s_want_acc > 0 ? 1.0f : 0.25f;  // following a lead: firm enough, still ramped (no valve dump)
            float lo = s_req_ours - 0.6f * (float)dt, hi = s_req_ours + up * (float)dt, asked = want;
            want = want < lo ? fmaxf(lo, 0) : (want > hi ? hi : want);
            // only a fading request is cut off: a rising one starts below 0.01 (0.25/s × 1/60 s = 0.004), and
            // cutting that kept bend / roundabout braking at 0 for good (ts4q 643-648 s: 63 km/h held to 50 m before
            // a roundabout, then a hard stop to 16)
            if (want < 0.01f && asked < 0.01f) want = 0;
        }
    }
    if (want > 0) {
        if (*req < want || *req == s_req_ours) *req = want;
        s_req_ours = want;
    } else if (s_req_ours > 0) {
        if (*req == s_req_ours) *req = 0;
        s_req_ours = 0;
    }
    s_hazard_mute = s_req_ours > 0 && *req <= s_req_ours;  // a stronger request (the game's AEB) keeps its hazards
    // the driver's gas reaches the engine while only the Auto Hold asks (it lets go as the torque builds)
    s_thr_pass = s_hazard_mute && s_want_hold > 0 && s_want_hill <= 0 && s_want_curve <= 0 && g_creep_brake <= 0;
}

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
    if (s_hs_until && release) { s_hs_until = 0; s_hs_armed = 0; if (g_dash_status == 'B') g_dash_status = 0; return; }
    if (s_hs_until) s_want_hill = 0.6f;
    // B on the dashboard while the hill-start brake holds
    if (s_hs_until) g_dash_status = 'B';
    else if (g_dash_status == 'B') g_dash_status = 0;
}


// ---------------------------------------------------------------- Auto Hold (docs/autohold-acc.md)
// After a stop with the brake (speed < 0.5 km/h, pedal > 0.1) the truck stays held through the same brake
// request as hill start (game_actor+0x17c, one writer: brake_request()), at any grade and without a timer,
// until the driver gives gas. Real cars (Damien: VW Crafter, Lynk & Co 01) struggle to let go when you pull
// away hard: the hold drops at a fixed moment and the car lurches or rolls. Here the release follows the drive:
//  - the game zeroes the merged throttle while 0x17c > 0 (docs/neutral-exit-re.md § 3.1); a patch of that
//    one compare (s_thr_pass) lets the driver's gas through while only we hold, so the engine builds torque
//    against the brake and the box engages as usual;
//  - the request falls with the drive force: r = hold x (1 - F_drive / F_needed), F_needed = the grade force plus
//    what it takes to start moving; never faster than the pedal allows (gentle ~1.2 s, hard ~0.25 s: still a ramp)
//    and never up again, except when the truck starts to roll back: then it grabs again;
//  - moving off (speed > 0.3 km/h): target 0 at the pedal's own pace; after 2.5 s of launch: the hard rate at least.
// Without the patch (pattern not found) the throttle is dead while we hold, so the request just ramps with
// the pedal (the old hill-start behavior, but ramped).
enum { AH_OFF, AH_HOLD, AH_LAUNCH };
static struct { int st, released; float r, mass; double launch_t, grab_until, mass_at, roll_m; } AH;
static engine_info s_eng;
static int s_eng_ok, s_gate_ok;
static float s_ah_k = 1;  // learned: how much of the estimated drive force really arrives (the clutch and the engine model are rough)

// Total mass (kg) and engine data, refreshed while holding.
static void ah_refresh()
{
    double now = wall_s();
    if (now - AH.mass_at < 1) return;
    AH.mass_at = now;
    float m = game_mass();
    if (m == m && m > 500) AH.mass = m;
    else if (!(AH.mass > 500)) AH.mass = 26000;
    void *self = game_gearbox();
    s_eng_ok = self && gb_engine(self, &s_eng);
}

// Force (N) at the wheels the engine and clutch deliver right now; NAN if the data is not there.
static float ah_drive_force()
{
    int g = T.gear;
    float r = wheel_radius();
    if (!s_eng_ok || g < 1 || g > 31 || !(r > 0.1f) || !(T.diff > 0) || !(T.ratio[g - 1] > 0)) return NAN;
    float couple = 1 - clutch_seen();
    couple = couple < 0 ? 0 : couple > 1 ? 1 : couple;
    float te = s_eng.torque * curve_factor(&s_eng, T.rpm) * T.eff_thr;
    return s_ah_k * te * T.ratio[g - 1] * T.diff * 0.9f * couple / r;
}

// Force (N) that gets the truck going: the grade force plus a small push (0.25 m/s2 with rolling resistance).
static float ah_need()
{
    float gr = pitch_grade() / 100, m = AH.mass;
    float f_grade = m * 9.81f * gr / sqrtf(1 + gr * gr);
    float need = 1.1f * f_grade + m * 0.25f;
    return need < m * 0.12f ? m * 0.12f : need;
}

static void auto_hold(char *g, float dt)
{
    s_want_hill = 0;
    s_want_hold = 0;
    float h = p_ah_req < 0.1f ? 0.1f : p_ah_req > 1 ? 1 : p_ah_req;
    double now = wall_s();
    if (!(dt > 0 && dt < 0.5f)) { s_want_hold = AH.r; return; }  // paused / hitch: keep what is set
    float kmh = fabsf(T.speed) * 3.6f;
    int blocked = !g_low_ok || !T.engine_on || T.parking_brake || T.gear < 0 || T.dgear < 0 || g_crawl_on || g_sg_hold;
    if (blocked) {                         // crawl owns the standstill, parking brake / reverse / Stop&Go: hand over quickly
        if (AH.st != AH_OFF) logv(1, "Auto Hold: off (%s)", !T.engine_on ? "engine off" : T.parking_brake ? "parking brake" : T.gear < 0 || T.dgear < 0 ? "reverse" : g_crawl_on ? "crawl on" : "Stop&Go holds");
        AH.st = AH_OFF;
    }
    float grade = pitch_grade();
    switch (AH.st) {
    case AH_OFF:
        AH.r = AH.r > 0 ? fmaxf(0, AH.r - 3 * dt) : 0;   // fade out, ~0.2 s
        if (!blocked && kmh < 0.5f && T.in_brake > 0.1f) {
            AH.st = AH_HOLD; AH.mass_at = -10; AH.roll_m = 0;
            ah_refresh();
            logv(2, "Auto Hold: holding at %.1f%% (mass %.0f kg, request %.2f)", grade, AH.mass, h);
        }
        break;
    case AH_HOLD:
        ah_refresh();
        AH.r = fminf(h, AH.r + 3 * dt);                  // on within ~0.2 s, while the driver still has the pedal
        if (T.in_thr > 0.05f) { AH.st = AH_LAUNCH; AH.launch_t = 0; AH.grab_until = 0; logv(2, "Auto Hold: launch, pedal %.2f, grade %.1f%%, hold %.2f", T.in_thr, grade, AH.r); }
        else if (kmh > 1.5f) { AH.st = AH_OFF; logv(2, "Auto Hold: truck moves without gas (%.1f km/h), off", kmh); }
        break;
    case AH_LAUNCH: {
        AH.launch_t += dt;
        if (T.in_thr < 0.03f && kmh < 1.0f && T.speed > -0.05f) { AH.st = AH_HOLD; logv(2, "Auto Hold: pedal released again, holding"); break; }
        if (T.in_brake > 0.1f && kmh < 1.0f) { AH.st = AH_HOLD; break; }
        if (T.in_brake > 0.1f) { AH.st = AH_OFF; break; }
        float need = ah_need(), drive = s_gate_ok ? ah_drive_force() : NAN;
        float target = drive == drive ? h * fmaxf(0, 1 - drive / need) : 0;   // no drive data / no throttle gate: only the pedal rate paces it
        int moving = T.speed > 0.3f / 3.6f;
        if (moving || AH.launch_t > p_ah_max) target = 0;
        // Flat or downhill: nothing to hold against, let go at the pedal's pace (ts4n: on 0 % it held ~2.2 s because
        // the drive estimate stays low while the clutch is still open and the engine at idle; Damien: "too slow").
        if (grade < 1.5f) target = 0;
        // Uphill but the clutch never bites while we hold (engine kept at idle): do not wait for it longer than 0.6 s.
        if (AH.launch_t > 0.6 && clutch_seen() > 0.9f) target = fminf(target, h * 0.5f);
        // ... and with the pedal (almost) flat for 1.2 s and the clutch still open the hold is what keeps the clutch from closing (the
        // automatic does not close it against the service brake): let go (Damien 12-10: 4 % up, gear 2, pedal 1.0, clutch 0.97 for 2 s)
        if (AH.launch_t > 1.2 && clutch_seen() > 0.9f && T.in_thr > 0.7f) target = 0;
        float ped = fminf(1, fmaxf(0, (T.in_thr - 0.25f) / 0.55f));          // 0 gentle .. 1 hard
        float t_rel = p_ah_gentle + (p_ah_hard - p_ah_gentle) * ped;          // seconds for the full hold
        float rate = h / fmaxf(t_rel, 0.05f);
        if (AH.launch_t > p_ah_max) rate = fmaxf(rate * 2, h / p_ah_hard);      // never fight the pedal for long (moving off keeps the pedal's pace: a quick drop is a lurch)
        if (T.speed < -0.08f && grade > 0.5f) {                               // rolling back uphill: grab again, trust the estimate less
            if (now > AH.grab_until) { s_ah_k = fmaxf(0.3f, s_ah_k * 0.7f); logv(2, "Auto Hold: rolling back (%.2f km/h): grabbing again, drive estimate x%.2f", -T.speed * 3.6f, s_ah_k); }
            AH.r = fminf(h, AH.r + 4 * dt);
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
    if (AH.r > 0) g_dash_status = 'B';
    else if (g_dash_status == 'B') g_dash_status = 0;
}



// ---------------------------------------------------------------- ISA state (limit following, resume to limit)
static float isa_goal = NAN, isa_ours;  // easing the set speed down to a lower limit
static int s_res_ev;                     // resume pressed while cruise was on (handled in keys_tick)
static double s_res_press_at = -10;      // last resume press (on or off): cruise coming on by resume follows ISA, by set not
static float s_res_before;

// The game's speed limiter (g_use_speed_limiter) on? Cached, read once a second.
static int limiter_on()
{
    static void *cv;
    static double at;
    static float use = NAN;
    if (!cv) cv = game_cvar("g_use_speed_limiter");
    if (cv && wall_s() - at > 1) { at = wall_s(); use = game_cvar_get(cv); }
    return use == use && use > 0.5f;
}

// Resume while on: the set speed becomes limit + offset (>= min_set, <= limiter cap).
static void resume_to_limit(char *ga)
{
    float *set = (float *)(ga + GA_SET);
    *set = s_res_before;  // undo whatever the game's resume did to the set speed
    if (T.speed_limit <= 1) return;
    int lim = (int)roundf(T.speed_limit * 3.6f);
    float v = fmaxf(lim + p_isa_off, p_min_set);
    if (g_low_ok && p_limiter > 0 && limiter_on() && v > p_limiter) v = p_limiter;
    float cur = *set;
    if (p_isa_ramp > 0 && v < cur - 1) { isa_goal = v; isa_ours = cur; }
    else { isa_goal = NAN; *set = v; }
    logv(1, "Cruise: resume while on: set speed %.0f (limit %d)%s", v, lim, isa_goal == isa_goal ? " (easing down)" : "");
}

// ---------------------------------------------------------------- cruise keys (docs/cruise-keys-re.md)
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
enum { GA_RETARDER = 0x180, GA_LIM = 0x190, GA_LIM_ON = 0x198, ACT_TOGGLE = 0x1e710, ACT_INC = 0x1e720, ACT_DEC = 0x1e730, ACT_RESUME = 0x1e740, ACT_ACCMODE = 0x1e750, ACT_STATE = 0xf8 };
typedef void (*consume_fn)(void *action);
typedef void (*retarder_fn)(void *ga, unsigned level, unsigned char snap);
static consume_fn o_consume;
static retarder_fn o_retarder;
static int g_keys_ok, g_ret_ok;
struct keydir { int ev, was_on; float before; double last_ev, t0, next; float s0; int seq, fired, block; };
static keydir K[2];  // [0] = +, [1] = -
static int s_prev_on;

static void *action_of(char *ga, unsigned off)
{
    void *a = *(void **)(ga + off);
    return a && mem_readable(a, ACT_STATE + 4) ? a : NULL;
}

// One press seen at the wrapped consume call. Plain C so the sim can feed it.
static void key_event(char *ga, void *act)
{
    int on = *(unsigned char *)(ga + GA_ON);
    int is_tog = act == action_of(ga, ACT_TOGGLE), is_inc = act == action_of(ga, ACT_INC),
        is_dec = act == action_of(ga, ACT_DEC), is_res = act == action_of(ga, ACT_RESUME);
    if (!act || !(is_tog || is_inc || is_dec || is_res)) return;
    if (g_ret_ok && p_ret_off > 0 && !(is_tog && on) && *(unsigned *)(ga + GA_RETARDER) > 0) {
        unsigned was = *(unsigned *)(ga + GA_RETARDER);
        o_retarder(ga, 0, 1);
        logv(1, "Cruise: %s pressed with the retarder on (level %u): retarder off (now %u)",
                is_tog ? "on/off" : is_inc ? "+" : is_dec ? "-" : "resume", was, *(unsigned *)(ga + GA_RETARDER));
    }
    if (is_res) s_res_press_at = wall_s();
    if (g_keys_ok && is_res && on && p_res_lim > 0) { s_res_ev = 1; s_res_before = *(float *)(ga + GA_SET); }
    if (g_keys_ok && (is_inc || is_dec)) {
        keydir &k = K[is_dec];
        if (s_anim_to == s_anim_to) { *(float *)(ga + GA_SET) = s_anim_to; s_anim_to = NAN; }  // finish a slide first
        k.ev = 1; k.was_on = on; k.before = *(float *)(ga + GA_SET);
    }
}
static void consume_stub(void *act)
{
    o_consume(act);
    char *ga = game_actor_safe();
    if (ga) key_event(ga, act);
}

// New set speed for one step in direction dir (+1 / -1) on the grid `step`: the game's own
// formula (docs/cruise-re.md § 4) with unit 1: inc = (floor((s+0.5)/g)+1)*g, dec = floor((s-0.5)/g)*g.
static float key_step(float base, int dir, float step)
{
    if (!(step >= 1)) step = 1;  // 0 or garbage from the ini would give a NaN set speed (audit 11-10)
    return dir > 0 ? (floorf((base + 0.5f) / step) + 1) * step : floorf((base - 0.5f) / step) * step;
}
static void key_apply(char *ga, int dir, float base, float step, const char *why)
{
    float r = key_step(base, dir, step);
    if (dir > 0 && *(unsigned char *)(ga + GA_LIM_ON) && *(float *)(ga + GA_LIM) > 0 && r > *(float *)(ga + GA_LIM))
        r = *(float *)(ga + GA_LIM);  // speed limiter caps the set speed, as in the game
    float minv = g_low_ok ? p_min_kmh : 30;
    if (dir < 0 && g_low_ok && p_min_set > 0) { if (r < p_min_set) r = p_min_set; }
    else if (dir < 0 && r < minv) {  // below the minimum: cruise off, like the game's dec
        *(unsigned char *)(ga + GA_ON) = 0; *(float *)(ga + GA_SET) = 0;
        logv(1, "Cruise: %s - from %.0f: below the minimum, cruise off", why, base);
        return;
    }
    // the driver's own +/- is taken at once (ts4o: five taps 40 -> 35 while the easing still walked down: "I could
    // hardly do anything by hand"), shown as a short slide on the dash (Damien: ~0.1 s, a few in-between steps)
    float from = *(float *)(ga + GA_SET);
    *(float *)(ga + GA_SET) = r;
    s_ease_goal = NAN;
    s_ease_prev = r;
    if (p_key_slide > 0 && from == from && fabsf(from - r) > 0.5f) { s_anim_from = from; s_anim_to = r; s_anim_t0 = wall_s(); *(float *)(ga + GA_SET) = from; }
    logv(2, "Cruise: %s %c from %.0f to %.0f km/h", why, dir > 0 ? '+' : '-', base, r);
}

// Runs every tick before the controller reads the set speed (and from the frame).
static void keys_tick(char *ga)
{
    if (!g_keys_ok || !ga) return;
    double now = wall_s();
    int on = *(unsigned char *)(ga + GA_ON);
    if (s_res_ev) { s_res_ev = 0; if (on && p_isa > 0) resume_to_limit(ga); }  // Damien: ISA off = hands off
    if (on && !s_prev_on) K[0].block = K[1].block = 1;  // switched on: a key still down from that press is not a step
    for (int d = 0; d < 2; d++) {
        keydir &k = K[d];
        int dir = d ? -1 : 1;
        if (k.ev) {
            k.ev = 0;
            k.last_ev = now;
            if (!k.was_on) { k.block = 1; k.seq = 0; }          // this press switched cruise on
            else if (!on) k.seq = 0;                            // the handler switched it off
            else {
                *(float *)(ga + GA_SET) = k.before;            // undo the game's step
                if (!k.block && !k.seq) { k.seq = 1; k.t0 = now; k.s0 = k.before; k.fired = 0; }
            }
        }
        void *act = action_of(ga, d ? ACT_DEC : ACT_INC);
        int held = now - k.last_ev < 0.08 || (act && *(unsigned *)((char *)act + ACT_STATE) != 0);
        if (!held) {
            if (k.seq && !k.fired && on) key_apply(ga, dir, k.s0, p_step_tap, "tap");
            k.seq = 0; k.fired = 0; k.block = 0;
        } else if (k.seq && on && now - k.t0 >= p_hold_s) {
            if (!k.fired) { k.fired = 1; k.next = now; }
            if (now >= k.next) {
                key_apply(ga, dir, *(float *)(ga + GA_SET), p_step_hold, "hold");
                double per = 1.0 / (p_hold_rate > 0.5f ? p_hold_rate : 0.5f);
                k.next = k.next + per > now ? k.next + per : now + per;
            }
        }
    }
    s_prev_on = on;
}

static float cruise_body(void *ga, float dt, float pedal);
static void own_acc(char *g, float dt);
static float lead_rel_speed(char *g, float lead, float dt);
static void cruise_trace(char *g, float u);
static void aeb(char *g, float dt);
enum { AEB_OFF, AEB_WARN, AEB_PARTIAL, AEB_FULL, AEB_HOLD };
static struct {
    int st;                                     // AEB_*
    float vl, gap_prev, rate, a_l, vl_prev, req, i, a_cmd, ttc, a_need, mass, thr_at_brake;
    double age_t0, calm_since, lock_until, beep_at, lost_at, thr_since, st_at, mass_at, close_since; int close_warned;
    // the event (for the summary line)
    double ev_t0; float ev_kmh, ev_min_gap, ev_min_ttc, ev_psi0, ev_psi_min, ev_max_dec; int ev_max, air_low; const char *ev_cap;
    // evasive lane change: steering / yaw before the warning (a bend is no evasion), since when it looks like one, until when
    float steer0, yaw0; double evade_since, evade_until;
} AE;
static unsigned char s_aeb_owns;   // our AEB is hooked in (patches in place)
static unsigned char s_aeb_watch;  // read by those patches every frame: the game's ACC emergency braking and its AEB stores are
                                   // skipped only while our AEB is watching right now (cruise on, or aeb_without_cruise = 1)
static float stub_cruise(void *ga, float dt, float pedal)
{
    if (ga) keys_tick((char *)ga);
    s_want_curve = 0;
    if (ga) own_acc((char *)ga, dt);  // our ACC target goes into +0x1a0 before the game's controller and its overspeed braking read it
    float r = cruise_body(ga, dt, pedal);
    if (ga) aeb((char *)ga, dt);
    if (AE.st > 0 && r > pedal) r = pedal;  // AEB warns or brakes: no cruise throttle
    // The driver's gas always wins over our own braking (Damien, ts4w: full gas near a stop did nothing until he
    // switched cruise off; the game zeroes the throttle while a brake request is set). Auto Hold lets go by itself.
    if (pedal > 0.1f) { s_want_curve = 0; s_want_acc = 0; }  // (the AEB has its own override: kick-down)
    if ((s_want_acc > 0.04f || s_want_curve > 0.04f) && r > pedal) r = pedal;  // we brake: no throttle of ours against it (ts4r 213 s, ts4w 532 s)
    if (ga) brake_request((char *)ga, fmaxf(fmaxf(fmaxf(fmaxf(fmaxf(s_want_hill, s_want_hold), s_want_curve), g_creep_brake), s_want_acc), s_want_aeb));
    float c = g_creep_thr;  // Creep (heavy loads): a little throttle on top, also with cruise off (stored in inner+0x1424)
    if (c > 0 && c < 0.3f && c > r) r = c;
    if (ga) cruise_trace((char *)ga, r);
    return r;
}

static float cruise_body(void *ga, float dt, float pedal)
{
    float stock = o_cruise(ga, dt, pedal);
    char *g = (char *)ga;
    if (g) { if (p_ah > 0 && g_low_ok) auto_hold(g, dt); else { s_want_hold = 0; hill_start(g); } }
    if (p_smooth <= 0 || !g || !*(unsigned char *)(g + GA_ON) || !(dt > 0 && dt < 0.5f)) {
        C.on = 0;
        g_sg_hold = 0;
        g_feed = 0;
        return stock;
    }
    float set = *(float *)(g + GA_SET), acc = *(float *)(g + GA_ACC_TARGET);
    float target = set;
    float kmh = fabsf(T.speed) * 3.6f;
    float a = game_adaptive();
    // Predictive (route): momentum into a climb, ease off before a top.
    int eco = a == a && a > 3.01f, power = a == a && a > 0 && a < 2.0f;
    s_mom_up = 0;
    if (p_predict > 0 && !power && route_ok()) {
        float v = fmaxf(fabsf(T.speed), 5.0f), up = band_up(), down = eco ? 5 : 3;
        float g_now = route_grade(0, 50), g_up = route_max_grade(v * 3, v * 15);
        float crest = route_crest_m(v * 12);
        float after = crest >= 0 ? route_min_grade(crest, crest + v * 15) : NAN;
        if (after == after && after < -2) target = set - down;
        else if (g_up == g_up && g_now == g_now && g_up > 2 && g_up > g_now + 1) { target = set + up; s_mom_up = up; }
    }
    // Speed limit following: the sign's limit (+offset), never above the set speed.
    // Bends ahead (route): the speed that takes every bend comfortably.
    float bend_kmh = NAN, jn_need = 0, jn_kmh = 0;
    s_bend_coast_kmh = NAN;
    if (p_curve > 0 && route_ok()) {
        float a_lat = p_curve_lat;  // same in every mode (Damien 11-10)
        // Like a human (Damien 11-10, first 0.6 ride: "braked oddly before a junction and very early before a ramp; later and
        // harder"): the throttle comes off early on a gentle plan (coast), the brakes only on a firm plan (bend_kmh below)
        float vb = route_curve_speed(a_lat, p_cv_coast, p_cv_coast);
        s_bend_coast_kmh = vb * 3.6f;
        float vb_brake = route_curve_speed(a_lat, p_cv_brake, p_jn_dec);
        static double said;
        if (vb == vb && vb * 3.6f < target - 5 && wall_s() - said > 10) {
            said = wall_s();
            logv(2, "Cruise: bend ahead: %.0f km/h (target %.0f, now %.0f)", vb * 3.6f, target, kmh);
        }
        route_junction_t ji = {};
        float vj = junc_on() ? route_junction_speed(p_cv_coast, &ji) : NAN;
        // You step on the gas for a junction we are slowing for: you go another way than the route (Damien 12-10: a petrol station with
        // 3-4 side entrances, the route turned there, he drove straight on and we braked to 20 for each). Leave junctions and roundabouts alone
        // for 6 s (Damien 12-10: first 12); the bends of the road itself still count.
        {
            static double ignore_until;
            if (vj == vj && T.in_thr > 0.3f && kmh < 60 && ji.dist_m < 250 && (kmh > ji.limit_ms * 3.6f - 5)) {
                if (wall_s() >= ignore_until) logv(1, "Cruise: gas pedal at a junction ahead (%.0f m, %.0f\xc2\xb0 \xe2\x86\x92 %.0f km/h): junctions are left alone for 6 s", ji.dist_m, ji.turn_deg, ji.limit_ms * 3.6f);
                ignore_until = wall_s() + 6;
            }
            if (wall_s() < ignore_until) vj = NAN;
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
        if (vb == vb) target = fminf(target, vb * 3.6f);
        if (vb_brake == vb_brake) bend_kmh = vb_brake * 3.6f;  // the brakes (below) follow the firm plan
        // the junction profile is a target too (ts4w 532-553 s: no lead, set 83, braking 0.1-0.15 for a 95° turn while
        // our throttle sat at 1.0, because only the brake knew about the junction)
        if (vj == vj && vj > 0) target = fminf(target, vj * 3.6f);
        // and no gas toward a junction or roundabout we have to slow down for anyway: the plan above allows speeds above ours far
        // from the entry, so the throttle came on and the brakes right after (Damien 12-10: "gas, then brake")
        if (vj == vj && ji.limit_ms > 0 && kmh > ji.limit_ms * 3.6f + 1 && ji.dist_m < fmaxf(150.0f, 12 * kmh / 3.6f)) target = fminf(target, kmh);
    }
    // The ACC's own target (vehicle ahead) always wins, also 0 = stop behind it. But someone cutting in or a
    // car that pulls away (the gap is opening) is no reason to slow down (Damien): then only hold the speed.
    float lead = *(float *)(g + GA_LEAD_M);
    s_lead_near = lead > 0.5f && lead < fmaxf(80.0f, 4 * fabsf(T.speed));
    float gap_rate = lead_rel_speed(g, lead, dt);  // m/s, + = the gap opens: the lead's speed from the radar, else d(gap)/dt
    if (lead > 0.5f && acc >= 0 && acc < target) {
        if (p_acc_calm > 0 && gap_rate > 0.3f && lead > 8) target = fmaxf(acc, fminf(target, kmh));  // opening: coast at most
        else target = acc;
    }
    // No gas while the wheel is turned (Damien 12-10: on a roundabout it gave gas with the wheel still round): hold the speed
    // until it is straight again (+ 0.8 s)
    {
        static double steer_until;
        float st_abs = fabsf(T.in_steer);  // a keyboard or gamepad tap sits at full lock: only an analog wheel counts
        if (st_abs > (kmh > 60 ? 0.25f : 0.15f) && st_abs < 0.95f) steer_until = wall_s() + 0.8;
        if (wall_s() < steer_until && kmh > 8 && target > kmh) target = kmh;
    }
    float e = target - kmh;
    // Too fast for a bend and the throttle is already off: a gentle brake request
    // (game_actor+0x17c, 0..1; the game slews it and uses engine brake/retarder first).
    // with hysteresis (start 3 km/h over, keep until 0.5 under): no on/off pumping of the brakes
    s_curve_braking = p_curve_brake > 0 && bend_kmh == bend_kmh && kmh > bend_kmh + (s_curve_braking ? -0.5f : 3.0f);
    if (s_curve_braking) {
        // gentle, but firmer when far too fast (Damien: roundabouts still too fast, it could not slow down enough)
        s_want_curve = fminf(kmh > bend_kmh + 8 ? 0.4f : 0.2f, fmaxf(0.04f, (kmh - bend_kmh + 0.5f) / 25));
    }
    // Junction / roundabout ahead and the profile cannot be kept by coasting: brake at least as hard as the
    // distance left needs (0.2 at 0.35 m/s2, 0.4 at 2 m/s2), also under the 3 km/h hysteresis above.
    // brake once coasting along the planned profile is not enough (threshold follows the plan: 0.35 at the old 0.4)
    float jn_thr = p_jn_dec - 0.05f;
    if (p_curve_brake > 0 && jn_need > jn_thr && kmh > jn_kmh + 3)
        s_want_curve = fmaxf(s_want_curve, fminf(0.45f, 0.2f + 0.12f * (jn_need - jn_thr)));
    float kp = 0.25f, ki = 0.08f, rate = 0.6f;               // Normal / M (per km/h, per km/h·s, per s)
    if (a == a && a > 3.01f) { kp = 0.18f; ki = 0.05f; rate = 0.4f; }        // Eco: gentle, lets it sag a bit
    else if (a == a && a > 0 && a < 2.0f) { kp = 0.35f; ki = 0.12f; rate = 1.0f; }  // Power: keen
    if (!C.on) { C.on = 1; C.u = stock; C.i = stock - kp * e; }   // bumpless start
    // at a standstill the open clutch is no shift: the throttle has to rise for Stop&Go to drive off (ts4r 189-193 and
    // 227-231 s: the lead pulled away, our throttle stayed frozen at 0.03, Damien had to give gas)
    int shifting = T.eff_clutch > 0.05f && kmh > 2;
    if (g_rolling) {   // in neutral: no throttle, keep the engine at idle; the game must not cancel cruise (ts3b)
        C.u = 0; C.i = -kp * e;
        if (g_sg_ok) { g_sg_hold = 1; g_feed = 0; }
        return pedal;
    }
    float u_raw = kp * e + C.i;
    // Following in a jam: gentle throttle at walking pace (ts4r 296-313 s, toll queue: full throttle / off every
    // second at 4-5 km/h against our own brake pulses; Damien: "full gas, gas off, full gas"). 0.4 from rest up to 1.0
    // at 18 km/h; and no wind-up while our ACC brakes.
    float u_cap = lead > 0.5f && acc >= 0 && acc < set ? fminf(1.0f, 0.4f + kmh / 30) : 1.0f;
    if (s_want_acc > 0.04f) { u_cap = 0; if (C.i > 0) C.i = 0; }
    float u = u_raw < 0 ? 0 : (u_raw > u_cap ? u_cap : u_raw);
    float step = rate * dt;
    if (shifting) step = 0;                                  // hold during the shift
    if (u > C.u + step) u = C.u + step;
    if (u < C.u - 2 * step) u = C.u - 2 * step;              // back off twice as fast
    // Integrate only while the output can follow (not shifting, not held by the
    // limits or the rate limit in the same direction): no wind-up.
    int held_up = u < u_raw - 1e-4f && e > 0, held_down = u > u_raw + 1e-4f && e < 0;
    if (!shifting && !held_up && !held_down) C.i += ki * e * dt;
    if (C.i > 1) C.i = 1;
    if (C.i < -0.5f) C.i = -0.5f;
    C.u = u;

    // Stop&Go bookkeeping (1.60 fields; only when its patches are in).
    if (g_sg_ok) {
        float lead = *(float *)(g + GA_LEAD_M);
        int slow = kmh < 3;
        double now = wall_s();
        if (!slow) g_stopped_at = 0;
        else if (!g_stopped_at) g_stopped_at = now;
        int may_wait = p_sg_auto <= 0 || (g_stopped_at && now - g_stopped_at < p_sg_auto);
        g_sg_hold = p_sg > 0 && slow && lead > 0.5f && may_wait && !T.parking_brake;
        g_feed = (p_sg > 0 && kmh < 10 && (g_sg_hold || g_stopped_at) && u > 0.02f) ? u : 0;
    }
    // Held back by something else (ts4n 433-448 s: brake air low → the game's spring brakes stopped the truck while our
    // cruise gave full throttle for 14 s, engine racing in neutral): low air, or full throttle without moving for 2 s
    // → our throttle 0 and the integrator reset, until the truck moves again.
    {
        static double stuck_since;
        int air_bad = T.air_warning || (T.air_psi > 1 && T.air_psi < p_air_min);
        int stuck = u > 0.5f && kmh < 2 && T.in_thr < 0.02f;
        if (!stuck) stuck_since = 0;
        else if (!stuck_since) stuck_since = wall_s();
        if (air_bad || (stuck_since && wall_s() - stuck_since > 4)) { C.u = 0; C.i = 0; g_feed = 0; u = 0; }
    }
    return u > pedal ? u : pedal;
}

static void *g_acc, *g_smart;
static float g_smart_player = NAN, g_smart_ours = NAN;
// the margin we use on the flat: the ini value, else the player's own game setting (restored when the game closes)
static float smart_base() { return p_smart_kmh >= 0 ? p_smart_kmh : g_smart_player; }
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
        s_press_before = g_acc_player == g_acc_player ? g_acc_player : game_cvar_get(g_acc);
        s_press_at = wall_s();
    }
    if (cvar == g_acc && g_acc_ours == g_acc_ours && g_acc_player == g_acc_player) {
        game_cvar_set(g_acc, g_acc_player);
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

static float gap_s()
{
    float t = g_level == 0 ? p_gap_s : g_level == 2 ? p_gap_l : p_gap_m;
    if (g_level == 0) return t;  // level 1 is really acc_short_s, in every mode (Damien 11-10: it came out at ~1.5 s)
    float a = game_adaptive();
    if (a == a && a > 3.01f) t *= p_eco_k;
    if (a == a && a > 0 && a < 2.0f) t *= p_power_k;
    return t;
}

// ---------------------------------------------------------------- lead speed and own ACC (docs/autohold-acc.md)
// The game's ACC fn 0x1409958b0 asks the radar fn 0x140998ce0(game_actor, out) every tick; out = {gap m, lead speed m/s,
// FLT_MAX, 0, 0}. The call site is wrapped: we copy the first two floats. The lead is only valid while the game itself
// accepts it (+0x1a8 > 0.5, gap < 100 m, no indicator) and its gap matches ours.
typedef float *(*radar_fn)(void *ga, float *out);
static radar_fn o_radar;
static struct { float gap, v; double at; int n; } s_radar;
static int g_radar_ok;
static float *radar_stub(void *ga, float *out)
{
    float *r = o_radar(ga, out);
    if (r && mem_readable(r, 20)) { s_radar.gap = r[0]; s_radar.v = r[1]; s_radar.at = wall_s(); s_radar.n++; }
    return r;
}

static float s_lead_v = NAN, s_acc_game = NAN, s_acc_own = NAN, s_dstar = NAN;
static struct { float t; int on; } OA;  // own ACC target (km/h) and whether it is following a lead

// Speed (m/s) of the game's current lead from the radar hook; NAN when there is none or the data is stale/inconsistent.
static float lead_speed(char *g)
{
    float lead = *(float *)(g + GA_LEAD_M);
    if (!g_radar_ok || !(lead > 0.5f) || !(wall_s() - s_radar.at < 0.3)) return NAN;
    if (!(fabsf(s_radar.gap - lead) <= 3 + 0.1f * lead)) return NAN;
    float v = s_radar.v;
    return v >= 0 && v < 70 ? v : NAN;
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

// The gap we want to keep (m): a standstill gap plus the time gap of the player's distance level (and mode).
// (Damien 12-10: in a queue it may close up, whatever the distance level, not onto the bumper: from walking pace the gap grows
// from 4 m to own_acc_stop_m and the time gap fades in, both full at 30 km/h)
static float stop_gap(float v_ms)
{
    float lo = fminf(4.0f, p_own_stop_m), fade = fminf(1.0f, v_ms / 8.33f);
    return lo + (p_own_stop_m - lo) * fade;
}
static float desired_gap(float v_ms)
{
    float mx = fmaxf(p_max_m, p_own_stop_m);
    float fade = fminf(1.0f, v_ms / 8.33f);
    // level 1: the standstill gap only matters at low speed (max, not sum); 2 and 3 keep the margin on top
    float m = g_level == 0 ? fmaxf(stop_gap(v_ms), gap_s() * v_ms) : stop_gap(v_ms) + gap_s() * v_ms * fade;
    return m > mx ? mx : m;
}

// Own ACC: target speed = lead speed + gain x (gap - wanted gap), slewed (a lead that appears starts from our speed),
// written into +0x1a0 so the game's controller and its overspeed braking (retarder / engine brake / service brake)
// follow it. The game's emergency braking (gap < half its D(v)) is untouched. Cut-in or a lead pulling away faster than
// we drive: never slower (acc_calm_cut_in), the target only stops rising.
static void own_acc(char *g, float dt)
{
    float *tgt = (float *)(g + GA_ACC_TARGET);
    s_acc_game = *tgt;
    s_acc_own = NAN;
    s_dstar = NAN;
    float lead = *(float *)(g + GA_LEAD_M), lv = lead_speed(g);
    s_lead_v = lv;
    if (p_own_acc <= 0 || !*(unsigned char *)(g + GA_ON) || g_level < 0 || !(lv == lv) || !(dt > 0 && dt < 0.5f)) { OA.on = 0; s_want_acc = 0; s_lv_prev = NAN; s_a_lead = 0; return; }
    float set = *(float *)(g + GA_SET), v = fabsf(T.speed), kmh = v * 3.6f;
    float dstar = desired_gap(v);
    float stopm = stop_gap(v);
    s_dstar = dstar;
    // Heavier = slower to stop (Damien: "start braking earlier at 60 t"): from 30 t up the planned deceleration shrinks
    // and the same need asks for more brake, up to 2.5x at 75 t.
    static float mass = 30000; static double mass_at = -10;
    if (wall_s() - mass_at > 2) { mass_at = wall_s(); float m = game_mass(); if (m == m && m > 500) mass = m; }
    float heavy = fminf(2.5f, fmaxf(1.0f, mass / 30000));
    float want = set;
    if (lv * 3.6f < set) {
        want = (lv + p_own_gain * (lead - dstar)) * 3.6f;
        want = want < 0 ? 0 : want > set ? set : want;
    }
    // Plan the approach (ts4o 365-377 s: 66 km/h onto a ~25 km/h lead, gap 49 → 10 m in 8 s: the game's emergency
    // braking emptied the brake air, 135 → 25 psi in 4 s): never faster than the speed from which a comfortable
    // deceleration still ends at the lead's speed at the wanted gap.
    if (p_own_plan > 0) {
        float room = fmaxf(0, lead - fmaxf(stopm, 0.6f * dstar));
        float v_ok = sqrtf(fmaxf(0, lv) * fmaxf(0, lv) + 2 * (p_own_plan / heavy) * room) * 3.6f;
        if (v_ok < want) want = v_ok;
    }
    float opening = lv - v, closing = -opening;
    // Crossing traffic (friend 11-10: cruise at ~40 km/h into roundabouts, down to a stop 3-4 times): the radar's lead
    // speed is absolute, so a car crossing in front drives at ~25 km/h while the gap closes at our own speed. Following
    // it is wrong; it is gone in a moment. Measured gap rate (0.4 s filter) against what the lead's speed explains.
    static float cr_prev = NAN, cr_rate = NAN; static double cr_since = -1, cr_said;
    if (cr_prev == cr_prev && fabsf(lead - cr_prev) < 10) {
        float r = (lead - cr_prev) / dt;
        cr_rate = cr_rate == cr_rate ? cr_rate + (r - cr_rate) * fminf(1, dt / 0.4f) : r;
    } else cr_rate = NAN;
    cr_prev = lead;
    int crossing = 0;
    if (p_acc_cross > 0 && cr_rate == cr_rate && lv > 3.0f && v > 3.0f && opening - cr_rate > fmaxf(2.5f, 0.5f * lv)) {
        if (cr_since < 0) cr_since = wall_s();
        crossing = wall_s() - cr_since > 0.4;
        if (crossing && wall_s() - cr_said > 10) {
            cr_said = wall_s();
            logv(1, "Cruise: own ACC passes crossing traffic at %.0f m (it drives %.0f km/h, the gap closes at %.0f km/h; we %.0f), junction ahead %.0f m",
                    lead, lv * 3.6f, -cr_rate * 3.6f, kmh, route_junction_m());
        }
    } else cr_since = -1;
    if (crossing) want = fmaxf(want, fminf(kmh, set));
    // level 1: the own ACC slowing hard for a lead (friend 11-10: cruise at ~40 km/h into roundabouts, 3-4 times down
    // to a stop with no pedal; with log_level 1 nothing showed why)
    {
        static double said_slow;
        double now_s = wall_s();
        if (want < fmaxf(10, 0.4f * kmh) && kmh > 15 && now_s - said_slow > 10) {
            said_slow = now_s;
            logv(1, "Cruise: own ACC slows for a lead at %.0f m, %.0f km/h (we %.0f, set %.0f, gap %+.1f m/s, from its speed %+.1f): "
                    "target %.0f km/h, junction ahead %.0f m", lead, lv * 3.6f, kmh, set, cr_rate, lv - v, want, route_junction_m());
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
    float dn = p_own_dec * 3.6f, up = p_own_up * 3.6f;
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
        float b = p_own_brake > 0 && need > 0.35f / heavy ? fminf(p_own_brake * fminf(heavy, 1.5f), (need * heavy - 0.3f) / 7.0f) : 0;
        if (b > 0 && b < 0.05f) b = 0.05f;
        // Stopping behind a lead that stops (Damien, ts4q 292-300 s: the lead stopped at 8.7 m, we crept on at 3 km/h
        // in gear 1 at idle with a 0.015 request for 7 s, at 2.6 m the game's emergency braking dumped the air 120 -> 5
        // psi). Plan the stop at stop_m - 1 m from the lead's bumper, with an offset that beats the idle creep, and
        // keep a light hold at a standstill until the lead pulls away (then Stop&Go drives off).
        if (p_own_brake > 0 && lv < 1.5f && v > 0.05f && lead < gmin + 15) {
            float s = lead - fmaxf(4.0f, stopm - 1);
            float a_t = v * v / (2 * fmaxf(0.3f, s));
            // only when the stop really needs it: ts4r 213-215 s stopped at 16.6 m behind a stopped car (a_t 0.2) with
            // our throttle fighting it; with room to spare the idle creep closes in by itself (Damien liked that)
            if (a_t > 0.4f || s < 1.5f) {
                float bs = fminf(fmaxf(p_own_brake, 0.45f), 0.08f + a_t / 7.0f);
                if (s < 1.5f) bs = fmaxf(bs, 0.25f);  // at the stop point: firm, beats the idle creep
                b = fmaxf(b, bs);
            }
        }
        if (p_own_brake > 0 && v <= 0.05f && lv < 0.5f && lead < gmin + 3) b = fmaxf(b, 0.2f);  // standing: hold
        s_want_acc = crossing ? 0 : b;
    }
    if (want < OA.t) OA.t = fmaxf(want, OA.t - dn * dt);
    else OA.t = fminf(want, OA.t + up * dt);
    // calm only when the gap is about right: ts4n followed at 8.8 m with 27 m wanted, and "the lead pulls away 1 km/h
    // faster" kept the target at our speed, so it never opened the gap (Damien: "it seems to do nothing")
    // Damien: a lead that is clearly faster (≥ ~3.5 km/h) opens the gap by itself, even when you are on its bumper:
    // no braking then, only no speeding up. Just a little faster (0.3-1 m/s): only calm when the gap is already reasonable.
    int pulling_away = opening > 1.0f || (opening > 0.3f && lead > 0.5f * dstar);
    if (p_acc_calm > 0 && pulling_away && lead > 5) OA.t = fmaxf(OA.t, fminf(kmh, set));
    // far too close: never more than the game's own ACC target (it knows how to open a gap)
    if (!pulling_away && lead < 0.4f * dstar && s_acc_game == s_acc_game && s_acc_game >= 0) OA.t = fminf(OA.t, fmaxf(s_acc_game, want));
    if (crossing) OA.t = fmaxf(OA.t, fminf(kmh, set));  // hold the speed past it (the game's emergency braking stays as the last resort)
    if (OA.t > set) OA.t = set;
    *tgt = OA.t;
    s_acc_own = OA.t;
}

static FILE *g_csv;
static void cruise_trace(char *g, float u)
{
    static double at;
    if (!g_csv) return;
    int on = *(unsigned char *)(g + GA_ON);
    if (!on && AH.st == AH_OFF && AH.r <= 0 && s_req_ours <= 0 && AE.st == AEB_OFF) return;
    double now = wall_s();
    if (now - at < 0.1) return;
    at = now;
    fprintf(g_csv, "%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.2f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%d,%.3f,%.2f,%.2f,%d,%.2f,%.0f,%.2f,%.1f,%d,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f\n", now, T.speed * 3.6f,
            on ? *(float *)(g + GA_SET) : 0.0f, *(float *)(g + GA_LEAD_M), s_radar.gap < 1e6f ? s_radar.gap : 0.0f, s_lead_v * 3.6f,
            s_lead_v == s_lead_v ? s_lead_v - fabsf(T.speed) : NAN, s_acc_game, s_acc_own, *(float *)(g + GA_ACC_TARGET), s_dstar, u, s_req_ours,
            *(float *)(g + GA_BRAKE_REQ), AH.st, AH.r, T.in_thr, T.in_brake, T.gear, T.eff_clutch, T.rpm, pitch_grade(), T.air_psi,
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
    // handlers (docs/cruise-re.md § 4), against a pooled 30.0 constant; point
    // those two loads (activate compare, handler's xmm7 for dec/resume) at our own.
    // The same two code sites also prove the 1.60 game_actor layout that g_low_ok stands for (hazards, hill start, Auto
    // Hold, ISA, easing, limiter): found → g_low_ok, whatever min_kmh is (audit 11-10: with the player preset's 30 all of
    // those were silently off, in 0.4 too).
    {
        uintptr_t act = game_scan("0F 54 05 ?? ?? ?? ?? 0F 2F 05 ?? ?? ?? ?? F3 0F 11 83 9C 01 00 00 73");
        uintptr_t head = game_scan("F3 0F 10 3D ?? ?? ?? ?? 45 33 FF 84 C9 0F 84 ?? ?? ?? ?? 44 38 BF 99 01 00 00");
        int layout = act && head && *(const float *)(act + 14 + *(const int32_t *)(act + 10)) == 30.0f &&
                     *(const float *)(head + 8 + *(const int32_t *)(head + 4)) == 30.0f;
        if (!layout) logline("Cruise: 1.60 cruise code not found: minimum speed, hazards, hill start, Auto Hold and ISA stay off");
        else {
            g_low_ok = 1;
            if (p_min_kmh > 0 && p_min_kmh < 30) {
                if (game_patch_rip_float(act + 7, 3, 7, p_min_kmh) && game_patch_rip_float(head, 4, 8, p_min_kmh))
                    logline("Cruise: on/resume from %.0f km/h, set speed at least %.0f km/h", p_min_kmh, p_min_set);
                else logline("Cruise: minimum speed patch failed, stays 30 km/h");
            }
            // The handler's xmm7 is also the ACC distance key's first threshold
            // (docs/neutral-exit-re.md § 4): give that compare its own 30.0, and
            // let a press step from the player's distance, not from our value.
            uintptr_t ak = game_scan("48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? 0F 2F C7 76 ?? 48 8D 0D ?? ?? ?? ?? E8");
            if (ak) o_acc_read = (cvar_read_fn)(ak + 12 + *(const int32_t *)(ak + 8));
            if (ak && game_patch_comiss_xmm0(ak + 12, 30.0f) && game_patch_call(ak + 7, (void *)acc_read_stub))
                logline("Cruise: ACC distance key kept working");
            else logline("Cruise: ACC distance key code not found: it may only switch to 30 now");
        }
    }

    // Cruise keys: retarder off on a press, tap / hold steps (docs/cruise-keys-re.md). Four wrapped consume calls.
    if (p_step_tap > 0 || p_ret_off > 0) {
        static const char *pat_tog = "48 85 C9 74 12 83 B9 F8 00 00 00 02 75 09 E8 ?? ?? ?? ?? B1 01 EB 02 32 C9 84 C9 74 1A 80 BF 99 01 00 00 00 75 0A";
        static const char *pat_k[3] = {
            "48 8B 8F 20 E7 01 00 48 85 C9 74 ?? 83 B9 F8 00 00 00 02 75 ?? E8 ?? ?? ?? ?? B1 01",
            "48 8B 8F 30 E7 01 00 48 85 C9 74 ?? 83 B9 F8 00 00 00 02 75 ?? E8 ?? ?? ?? ?? B1 01",
            "48 8B 8F 40 E7 01 00 48 85 C9 74 ?? 83 B9 F8 00 00 00 02 75 ?? E8 ?? ?? ?? ?? B1 01" };
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
    if (p_smooth > 0) {
        uintptr_t h = game_scan("48 8B CB F3 44 0F 11 83 E4 1C 00 00 F3 45 0F 5F C3 41 0F 28 D0 E8");
        uintptr_t site = h ? h + 21 : 0;
        if (site) o_cruise = (cruise_fn)(site + 5 + (intptr_t) * (const int32_t *)(site + 1));
        if (site && game_patch_call(site, (void *)stub_cruise)) {
            g_cruise_smooth = 1;
            logline("Cruise: own smooth cruise throttle in place");
        } else logline("Cruise: cruise throttle call not found, the game's cruise stays");
    }
    // Our brake request (bends, hill start) is > 0 for a while; the game's
    // auto-hazard rule (docs/neutral-exit-re.md § 3) would switch the hazards
    // on every time (ts3d). Its compare is skipped while only we ask.
    if (g_cruise_smooth && (p_curve_brake > 0 || p_hill > 0 || p_ah > 0)) {
        uintptr_t hz = game_scan("85 C0 74 17 F6 87 D0 01 00 00 10 75 0E 44 0F 2F 8F 7C 01 00 00 73 04 B2 01");
        if (hz && game_patch_cf0_if(hz + 13, 8, &s_hazard_mute)) logline("Cruise: our brake requests leave the hazards alone");
        else logline("Cruise: hazard rule not found: braking for bends / hill start may switch the hazards on");
    }
    // Auto Hold: the game zeroes the merged throttle while the brake request is > 0. Skip that one compare while only
    // Auto Hold holds, so the pedal builds torque against the brake and the release can follow it (docs/autohold-acc.md).
    if (p_ah > 0 && g_cruise_smooth && g_low_ok) {
        uintptr_t gt = game_scan("0F 2F CF 76 ?? 83 BB 50 01 00 00 02 75 ?? 0F 2F BB 7C 01 00 00 72 0A F3 0F 11 81 24 14");
        if (gt && game_patch_cf0_if(gt + 14, 7, &s_thr_pass)) {
            s_gate_ok = 1;
            logline("Cruise: Auto Hold in place (throttle passes while it holds, hold %.2f, release %.2f-%.2f s)", p_ah_req, p_ah_hard, p_ah_gentle);
        } else logline("Cruise: Auto Hold: throttle gate not found, the release is paced by the pedal only (no drive feedback)");
    } else if (p_ah > 0) logline("Cruise: Auto Hold needs the minimum-speed patch and our cruise throttle: off (hill start stays)");
    // Lead speed: wrap the radar call in the ACC fn (the game keeps the lead's speed in a local only).
    if (L && !strcmp(L->name, "1.60")) {
        uintptr_t rh = game_scan("48 8D 54 24 50 48 8B CB C6 83 21 01 00 00 01 E8 ?? ?? ?? ?? 0F 10 00 8B 40 10 89 44 24 48");
        uintptr_t rs = rh ? rh + 15 : 0;
        if (rs && *(const unsigned char *)rs == 0xE8) {
            o_radar = (radar_fn)(rs + 5 + (intptr_t) * (const int32_t *)(rs + 1));
            if (game_patch_call(rs, (void *)radar_stub)) {
                g_radar_ok = 1;
                logline("Cruise: lead speed hook in at %p (radar fn %p): own ACC %s", (void *)rs, (void *)o_radar, p_own_acc > 0 ? "on" : "off (hook only feeds the trace and the cut-in test)");
            } else logline("Cruise: lead speed hook failed: ACC target and cut-in test stay the game's / d(gap)/dt");
        } else logline("Cruise: radar call in the ACC fn not found: ACC target and cut-in test stay the game's / d(gap)/dt");
        // Own AEB (docs/aeb.md): the ACC's emergency part sees no lead (its gap load becomes 0) and the game's AEB stores of
        // 1.0 into +0x17c are skipped while ours runs. Needs our cruise fn hook (that is where the AEB runs) and the radar.
        if (p_aeb > 0 && g_radar_ok && g_cruise_smooth && g_low_ok) {
            uintptr_t ae = game_scan("80 BE 99 01 00 00 00 0F 84 ?? ?? ?? ?? 48 8D 0D ?? ?? ?? ?? E8 ?? ?? ?? ?? 0F 2F C6 0F 86 ?? ?? ?? ?? 0F 2F 3D ?? ?? ?? ?? 0F 87 ?? ?? ?? ?? 48 8B 46 18 F3 0F 10 BE A8 01 00 00");
            uintptr_t s1 = game_scan("41 0F 2F F8 73 14 44 0F 2F 8B 0C 01 00 00 73 0A C7 83 7C 01 00 00 00 00 80 3F");
            uintptr_t s2 = game_scan("84 C0 75 0D 83 8B 14 01 00 00 10 EB 1A 84 C0 74 16 C7 83 7C 01 00 00 00 00 80 3F");
            s_aeb_owns = 1;  // cleared again if any of the patches fails; s_aeb_watch is set per frame in aeb()
            if (ae && s1 && s2 && game_patch_zero_xmm_if(ae + 0x33, 8, 7, &s_aeb_watch) && game_patch_skip_if(s1 + 16, 10, &s_aeb_watch) &&
                game_patch_skip_if(s2 + 17, 10, &s_aeb_watch)) {
                void *eb = game_cvar("g_emergency_brake"), *bs = game_cvar("g_blind_spot");
                float ebv = eb ? game_cvar_get(eb) : NAN, bsv = bs ? game_cvar_get(bs) : NAN;
                logline("Cruise: own AEB in place (%s; warning %.1f s, partial %.1f s / %.1f m/s2, full %.1f s); the game's emergency braking is replaced only while ours watches (its AEB option: %.0f)",
                        p_aeb_manual > 0 ? "also without cruise" : "with cruise only", p_aeb_warn_s, p_aeb_part_s, p_aeb_part_dec, p_aeb_full_s, ebv);
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
        g_smart = game_cvar("g_cruise_control_smart");
        if (g_smart) g_smart_player = game_cvar_get(g_smart);
        logline("Cruise: descent braking %s (player's smart overspeed %.1f km/h, used %.1f%s)", g_smart ? "on" : "off: cvar not found",
                g_smart_player, smart_base(), p_smart_kmh >= 0 ? " from the ini" : "");
    }
    if (p_sg > 0 && g_cruise_smooth) {
        uintptr_t ped = game_scan("F3 41 0F 10 96 C4 1C 00 00 F3 41 0F 10 8E C0 1C 00 00");
        uintptr_t c1 = game_scan("C6 86 99 01 00 00 00 48 8B 0B 48 8B 01 FF 90 A0 00 00 00 84");
        uintptr_t c2 = game_scan("C6 86 99 01 00 00 00 48 8B 46 18 48 8B 88 50 01 00 00 80 B9 3D 03");
        if (ped && c1 && c2 && game_patch_feed_xmm1(ped, 18, &g_feed) && game_patch_skip_if(c1, 7, &g_sg_hold) &&
            game_patch_skip_if(c2, 7, &g_sg_hold)) {
            g_sg_ok = 1;
            logline("Cruise: Stop&Go in place (drives off %s)", p_sg_auto > 0 ? "within the set time" : "always");
        } else logline("Cruise: Stop&Go code not found");
    }
    g_acc = game_cvar("g_acc");
    if (!g_acc) { logline("Cruise: g_acc not found, ACC gap stays the game's"); return; }
    g_acc_player = game_cvar_get(g_acc);
    // A value we left in the profile (not 0/20/30/40): back to the distance the player last picked (remembered), else
    // the nearest (that turned short into middle: our 25.5 rounded to 30).
    if (g_acc_player == g_acc_player && !is_round(g_acc_player)) {
        static const float lv[4] = { 0, 20, 30, 40 };
        float r = s_acc_saved >= -1 ? lv[s_acc_saved + 1] : g_acc_player <= 0 ? 0 : g_acc_player < 25 ? 20 : g_acc_player < 35 ? 30 : 40;
        logline("Cruise: g_acc %.1f was ours, back to %.0f", g_acc_player, r);
        game_cvar_set(g_acc, r);
        g_acc_player = r;
    }
    if (g_acc_player == g_acc_player) g_level = level_of(g_acc_player);
    logline("Cruise: ACC gap by time (short %.1f / middle %.1f / long %.1f s), player's g_acc %.1f = %s", p_gap_s,
            p_gap_m, p_gap_l, g_acc_player, level_name[g_level + 1]);
}

// Hazards on/off the way the game's hazard key does it (0x14099d633, docs/
// neutral-exit-re.md § 3): bit 0x10 of game_actor+0x1d0, flasher +0x340,
// blink phase +0x1e0 when no indicator runs. 1.60 offsets (g_low_ok).
enum { GA_LIGHTS = 0x1d0, GA_HAZ_FLASH = 0x340, GA_BLINK_T = 0x1e0 };
static int hazards_on(char *ga) { return (*(unsigned char *)(ga + GA_LIGHTS) & 0x10) != 0; }
static void hazards_set(char *ga, int on)
{
    volatile unsigned char *b = (volatile unsigned char *)(ga + GA_LIGHTS);
    if (((*b & 0x10) != 0) == (on != 0)) return;
    *b ^= 0x10;
    *(volatile float *)(ga + GA_HAZ_FLASH) = on ? 1.0f : 0.0f;
    if (!(*b & 0x0c)) *(volatile int32_t *)(ga + GA_BLINK_T) = 0;
}

// Emergency stop signal (Damien 11-10: ABS braking): braking harder than
// emergency_decel above emergency_kmh switches the hazards on; they go off
// when the truck pulls away again (throttle and speeding up for 1.5 s).
// Hazards the driver switched on are never touched.
static void emergency_hazards()
{
    char *ga = g_low_ok && p_ess > 0 ? game_actor_safe() : NULL;
    if (!ga || T.paused) return;
    double now = wall_s();
    float kmh = fabsf(T.speed) * 3.6f;
    static int stopped;
    static double soft_at;
    if (s_haz_ours && !hazards_on(ga)) s_haz_ours = 0;  // the driver switched them off
    // The speed test uses the highest speed of the last 2 s: the measured decel needs ~0.6 s to build up, and a stop
    // from 65 is already below 50 by then (ts4q 84.5 s: 4.3 m/s2 first seen at 52 km/h, hazards stayed off).
    static float peak_kmh;
    static double peak_at;
    if (kmh >= peak_kmh || now - peak_at > 2.0) { peak_kmh = kmh; peak_at = now; }
    if (!s_haz_ours) {
        stopped = 0;
        soft_at = 0;
        int hard = peak_kmh > p_ess_kmh && -T.accel > p_ess_dec;
        if (!hard) s_haz_hard_at = 0;
        else if (!s_haz_hard_at) s_haz_hard_at = now;
        else if (now - s_haz_hard_at > 0.3 && !hazards_on(ga)) {
            hazards_set(ga, 1);
            s_haz_ours = 1;
            s_haz_go_at = 0;
            logline("Cruise: emergency braking at %.0f km/h (%.1f m/s2): hazards on", kmh, -T.accel);
        }
        return;
    }
    // Like a real truck (UNECE: on from 4 m/s2, off below 2.5): not stopped and the braking eases off → off again;
    // brought to a full stop → they stay on until you drive on (Damien).
    if (kmh < 1) stopped = 1;
    if (!stopped) {
        if (-T.accel < p_ess_off) { if (!soft_at) soft_at = now; } else soft_at = 0;
        if (soft_at && now - soft_at > 0.3) {
            hazards_set(ga, 0);
            s_haz_ours = 0;
            soft_at = 0;
            logline("Cruise: braking eased off before a stop: hazards off");
        }
        return;
    }
    int going = T.in_thr > 0.1f && T.accel > 0.3f;
    if (!going) s_haz_go_at = 0;
    else if (!s_haz_go_at) s_haz_go_at = now;
    else if (now - s_haz_go_at > 1.5) {
        hazards_set(ga, 0);
        s_haz_ours = 0;
        stopped = 0;
        logline("Cruise: driving on: hazards off");
    }
}

// ---------------------------------------------------------------- AEB (docs/aeb.md)
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

static void aeb_stage(int st, float gap, float kmh)
{
    double now = wall_s();
    if (AE.st == AEB_OFF && st != AEB_OFF) {
        AE.ev_t0 = now; AE.ev_kmh = kmh; AE.ev_min_gap = 1e9f; AE.ev_min_ttc = 1e9f; AE.ev_psi0 = T.air_psi;
        AE.ev_psi_min = T.air_psi; AE.ev_max = st; AE.ev_max_dec = 0; AE.ev_cap = NULL; AE.beep_at = 0;
    }
    if (st != AEB_OFF && st <= AEB_FULL && st > AE.ev_max) AE.ev_max = st;
    if (AE.st == AEB_OFF) AE.air_low = 0;
    if (st > AE.st && st <= AEB_FULL)
        logline("AEB: %s: lead %.1f m ahead at %.0f km/h (we %.0f), time to collision %.1f s, needs %.1f m/s2 (mass %.0f t)",
                aeb_name[st], gap, AE.vl * 3.6f, kmh, AE.ttc, AE.a_need, AE.mass / 1000);
    if (st == AEB_HOLD && AE.st != AEB_HOLD) {
        if (gap > 0) logline("AEB: stopped %.1f m behind the lead: holding until gas or brake", gap);
        else logline("AEB: stopped (the radar lost the lead on the way): holding until gas or brake");
    }
    if (st == AEB_OFF && AE.st != AEB_OFF) DEV_NOTE(aeb_over(AE.ev_max >= AEB_FULL ? 2 : AE.ev_max >= AEB_PARTIAL ? 1 : 0));
    if (st == AEB_OFF && AE.st != AEB_OFF)
        logline("AEB: over after %.1f s: highest %s, %.0f -> %.0f km/h, min gap %.1f m, min time to collision %.1f s, peak decel %.1f m/s2, air %.0f -> %.0f psi%s%s",
                now - AE.ev_t0, aeb_name[AE.ev_max], AE.ev_kmh, kmh, AE.ev_min_gap < 1e8f ? AE.ev_min_gap : 0.0f,
                AE.ev_min_ttc < 1e8f ? AE.ev_min_ttc : 0.0f, AE.ev_max_dec, AE.ev_psi0, AE.ev_psi_min, AE.ev_cap ? ", held back by: " : "", AE.ev_cap ? AE.ev_cap : "");
    if (st != AE.st) AE.st_at = now;
    AE.st = st;
}

// AEB incident files (Damien 11-10: light logs normally, a full trace per incident with the local time, so a player can
// say "it was about that time"): 20 rows per second in a 10 s ring all the time; at an event the file starts with the
// ring and runs until 10 s after the event is over.
enum { AEB_RING = 200, AEB_COLS = 17 };
static float s_ring[AEB_RING][AEB_COLS];
static int s_ring_n, s_ring_i;
static FILE *s_inc;
static double s_inc_off_at, s_rec_at;
static void aeb_row(FILE *f, const float *r)
{
    fprintf(f, "%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%d,%.3f,%.2f,%.2f,%.2f,%.2f,%.3f,%.1f,%d,%.0f\n", r[0], r[1], r[2], r[3], r[4], r[5],
            r[6], (int)r[7], r[8], r[9], r[10], r[11], r[12], r[13], r[14], (int)r[15], r[16]);
}
static void aeb_record(float gap, float kmh, float vc, float ttc, float a_need)
{
    double now = wall_s();
    if (g_incident_mb <= 0 || now - s_rec_at < 0.05) return;
    s_rec_at = now;
    char *ga = game_actor_safe();
    unsigned char fl = ga ? *(unsigned char *)(ga + GA_LIGHTS) & 0x0c : 0;
    float r[AEB_COLS] = { (float)now, kmh, gap, AE.vl * 3.6f, vc, ttc < 99 ? ttc : 99.0f, a_need, (float)AE.st, AE.req, AE.a_cmd,
                          T.in_thr, T.in_brake, T.accel, T.in_steer, T.yaw * 360, (float)(fl == 4 ? 1 : fl == 8 ? -1 : fl ? 2 : 0), T.air_psi };
    memcpy(s_ring[s_ring_i], r, sizeof r);
    s_ring_i = (s_ring_i + 1) % AEB_RING;
    if (s_ring_n < AEB_RING) s_ring_n++;
    if (AE.st != AEB_OFF && !s_inc) {
        char stamp[32];
        s_inc = open_incident("incidents", "aeb", g_incident_mb, stamp, sizeof stamp);
        if (s_inc) {
            fprintf(s_inc, "# DazDrive %s AEB incident at %s (local time), %.0f t, log clock %.0f s (dazdrive.log); stage 0 off 1 warning 2 partial 3 full 4 hold; indicator 1 left -1 right 2 hazards\n", SUITE_VERSION, stamp,
                    AE.mass / 1000, now);
            fputs("t,kmh,gap_m,lead_kmh,closing_ms,ttc_s,a_need,stage,request,a_cmd,thr,brake,decel,steer,yaw_deg_s,indicator,air_psi\n", s_inc);
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
    if (!(dt > 0 && dt < 0.5f)) { s_want_aeb = AE.req; return; }  // paused / hitch: keep what is set
    double now = wall_s();
    float v = T.speed, kmh = fabsf(v) * 3.6f;
    int on = *(unsigned char *)(g + GA_ON);
    int active = (on || p_aeb_manual > 0) && T.engine_on && !T.paused && T.gear >= 0 && T.dgear >= 0 && v > -0.5f && !T.parking_brake;
    s_aeb_watch = active ? 1 : 0;  // not watching (cruise off with aeb_without_cruise = 0, reversing, ...): the game's own AEB is left alone
    if (now - AE.mass_at > 2) {
        AE.mass_at = now;
        float m = game_mass();
        AE.mass = m == m && m > 500 ? m : AE.mass > 500 ? AE.mass : 30000;
    }
    float heavy = fminf(2.0f, fmaxf(1.0f, AE.mass / 30000)), hs = sqrtf(heavy);  // heavier: slower to stop, so earlier
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
        AE.gap_prev = NAN; AE.rate = NAN; AE.vl_prev = NAN; AE.a_l = 0;
        if (!AE.lost_at) AE.lost_at = now;
    }
    // the threat: time to collision and the deceleration it takes to stop 3 m behind the lead (also if the lead brakes to a stop)
    float vc = 0, ttc = INFINITY, a_need = 0;
    int cap_st = AEB_FULL, crossing = 0;
    const char *cap = NULL;
    float p_s = p_aeb_part_s * hs, f_s = p_aeb_full_s * hs;
    if (fresh) {
        vc = v - AE.vl;
        // stop point: 3 m behind a standing lead; behind a moving one only 1.5 m + 0.1 s of its speed (Damien 11-10, ride
        // 20:06: tailgating at 25 km/h in town, 5 m behind a car pulling away, he braked himself and still got partial
        // braking; 3 m + 0.3 s made every short city gap an emergency. AEB avoids the impact, the gap is the ACC's job)
        // (at speed the old margin stays, 3 m + 0.3 s: ending 3 m behind a 40 km/h truck at 80 only hands the driver the
        // next emergency; it fades in with the LEAD's speed from 18 to 32 km/h: our own speed drops while braking, and the
        // margin must not shrink then, sim "slow truck" ended 1.6 m behind; full from 32 km/h)
        float kv = fminf(1, fmaxf(0, (AE.vl - 5.0f) / 4.0f));
        float room = fmaxf(0.3f, gap - (AE.vl < 0.5f ? 3.0f : 1.5f + 0.1f * AE.vl + kv * (1.5f + 0.2f * AE.vl)));
        a_need = vc > 0 ? vc * vc / (2 * room) : 0;
        // a lead that pulls away closes the gap less and less: its acceleration counts (time to collision too)
        float la = AE.vl >= 0.5f && AE.a_l > 0.3f ? AE.a_l - 0.3f : 0;  // 0.3: radar speed noise
        if (la > 0) a_need = fmaxf(0, a_need - la);
        float b = -AE.a_l - 0.5f;  // the lead brakes (0.5: radar speed noise)
        if (AE.vl < 0.5f) a_need = fmaxf(a_need, v > 0 ? v * v / (2 * room) : 0);
        else if (b > 0.3f) a_need = fmaxf(a_need, v > 0 ? v * v / (2 * (room + AE.vl * AE.vl / (2 * b))) : 0);
        ttc = vc > 0.3f ? gap / vc : INFINITY;
        if (la > 0 && vc > 0.3f) { float d = vc * vc - 2 * la * gap; ttc = d < 0 ? INFINITY : (vc - sqrtf(d)) / la; }  // gap = vc t - la t^2 / 2
        // crossing traffic (roundabouts, junctions): the gap closes as if the lead stood still while its speed says it drives;
        // it is gone in a moment. Only when it is really in front and close: brake.
        crossing = AE.rate == AE.rate && AE.vl > 3 && v > 3 && -AE.rate - vc > fmaxf(2.5f, 0.5f * AE.vl);
        crossing = crossing && !(gap < 8 && -AE.rate > 0.3f && gap / -AE.rate < 1.0f);
        if (crossing) { cap_st = AEB_WARN; cap = "crossing traffic"; }
        // a new target must be seen a moment first (a jump in the gap, a car swerving past), unless it is close and urgent
        if (now - AE.age_t0 < 0.3 && !(gap < 12 && a_need > 3)) { cap_st = AEB_WARN; if (!cap) cap = "new target"; }
        if ((*(unsigned char *)(g + GA_LIGHTS) & 0x0c) && !(gap < 10 && ttc < 1.0f)) { cap_st = AEB_WARN; cap = "indicator"; }
        // a lead that drives near a junction (turning, merging): partial and full braking later
        float jm = route_junction_m();
        if (jm >= 0 && jm < 60 && AE.vl > 2) { p_s = fminf(p_s, 1.5f); f_s = fminf(f_s, 1.0f); }
        // the driver brakes enough by himself
        if (T.in_brake > 0.05f && -T.accel >= 0.9f * a_need) { cap_st = AEB_OFF; cap = "driver brakes"; }
        // the ACC (cruise on) already brakes enough and there is time: leave it to the ACC (Damien 12-10: the AEB went off with
        // the ACC on and braking, and he thought the ACC did fine). The AEB is back as soon as that stops being true.
        else if (on && T.in_brake < 0.05f && ttc > 3.0f && -T.accel >= 0.65f * a_need) { cap_st = AEB_OFF; cap = "ACC brakes"; }
    }
    // The only cancel (Damien 11-10): the driver gives gas. Kick-down, a firm pedal held 0.3 s, or the pedal pressed 0.25 further
    // than it was when the braking started (a pedal that was already held does not cancel by itself). Then 3 s no braking.
    if (AE.st < AEB_PARTIAL) AE.thr_at_brake = T.in_thr;
    if (T.in_thr > 0.5f) { if (!AE.thr_since) AE.thr_since = now; } else AE.thr_since = 0;
    if (T.in_thr > 0.8f || (AE.thr_since && now - AE.thr_since > 0.3) || (AE.st >= AEB_PARTIAL && T.in_thr > AE.thr_at_brake + 0.25f)) {
        if (AE.st >= AEB_PARTIAL && AE.st <= AEB_FULL) logline("AEB: overruled by the gas pedal (%.0f%%)", T.in_thr * 100);
        AE.lock_until = now + 3;
    }
    int gas = now < AE.lock_until && !(fresh && gap < 5 && vc > 0);
    if (gas) { cap_st = AEB_WARN; cap = "gas pedal"; }
    // Evasive lane change (Damien 11-10: "if it warns me and I indicate left and steer left, it must understand"): one
    // indicator and steering or turning to that side, compared with what the truck did before the warning, for 0.3 s ends
    // a warning or partial braking; 3 s no AEB for it, unless the lead is still dead ahead and very close.
    {
        unsigned char fl = *(unsigned char *)(g + GA_LIGHTS) & 0x0c;
        int side = fl == 4 ? 1 : fl == 8 ? -1 : 0;  // + = left, like the steering and the yaw rate
        if (AE.st == AEB_OFF) {
            float k = fminf(1, dt / 1.0f);
            AE.steer0 += (T.in_steer - AE.steer0) * k;
            AE.yaw0 += (T.yaw - AE.yaw0) * k;
        }
        int ev = p_aeb_evade > 0 && side && ((T.in_steer - AE.steer0) * side > p_aeb_evade || (T.yaw - AE.yaw0) * side > 0.004f);
        if (!ev) AE.evade_since = 0;
        else if (!AE.evade_since) AE.evade_since = now;
        if (ev && now - AE.evade_since > 0.3 && AE.st >= AEB_WARN && AE.st <= AEB_PARTIAL && now >= AE.evade_until) {
            AE.evade_until = now + 3;
            logline("AEB: evasive lane change to the %s (steering %+.3f, turning %+.1f deg/s): stands down", side > 0 ? "left" : "right",
                    T.in_steer - AE.steer0, (T.yaw - AE.yaw0) * 360);
        }
    }
    int evade = now < AE.evade_until && !(fresh && gap < 10 && ttc < 1.0f);
    if (evade) { cap_st = AEB_OFF; cap = "evasive lane change"; }
    if (cap && AE.st != AEB_OFF && !AE.ev_cap) AE.ev_cap = cap;
    // the stage it asks for, and the stage that may stay (hysteresis)
    int want = AEB_OFF, keep = AEB_OFF;
    // With the ACC on it brakes for the lead itself (up to ~3.5 m/s2): warn only when that is not going to be enough, like
    // a real ACC's take-over warning (Damien 11-10, ride 20:08:37: a jam on ACC, the ACC already braking 1.8 m/s2, and a
    // warning at 1.5 m/s2 needed). Partial and full braking stay the same.
    float w_ttc = on ? fminf(p_aeb_warn_s, 2.0f) : p_aeb_warn_s, w_a = on ? 2.8f : 1.5f;
    if (fresh && vc > 0.3f) {
        if (ttc < w_ttc * hs || a_need > w_a / hs) want = AEB_WARN;
        if (ttc < p_s || a_need > 2.5f / hs) want = AEB_PARTIAL;
        if (ttc < f_s || a_need > 0.6f * p_aeb_max_dec / hs) want = AEB_FULL;
    }
    if (fresh && vc > -0.5f) {
        float k = 1.3f, ka = 0.75f;
        if (ttc < w_ttc * hs * k || a_need > w_a / hs * ka) keep = AEB_WARN;
        if (ttc < p_s * k || a_need > 2.5f / hs * ka) keep = AEB_PARTIAL;
        if (ttc < f_s * k || a_need > 0.6f * p_aeb_max_dec / hs * ka) keep = AEB_FULL;
    }
    if (AE.st <= AEB_WARN && kmh < p_aeb_kmh) want = AEB_OFF;        // steps in from aeb_min_kmh
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
        if (now - AE.calm_since > (st >= AEB_PARTIAL ? 1.0 : 1.5)) { st = st - 1 > keep ? st - 1 : keep; AE.calm_since = now; }
    }
    AE.ttc = ttc; AE.a_need = a_need;
    if (st != AE.st) aeb_stage(st, fresh ? gap : 0.0f, kmh);
    if (st != AEB_OFF) {
        if (fresh) { AE.ev_min_gap = fminf(AE.ev_min_gap, gap); AE.ev_min_ttc = fminf(AE.ev_min_ttc, ttc); }
        if (T.air_psi > 1) AE.ev_psi_min = fminf(AE.ev_psi_min, T.air_psi);
        AE.ev_max_dec = fmaxf(AE.ev_max_dec, -T.accel);
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
    if (st == AEB_PARTIAL) a_cmd = fminf(fmaxf(1.15f * a_need, fl * 0.7f * p_aeb_part_dec), 1.15f * p_aeb_part_dec);
    else if (st == AEB_FULL) a_cmd = ttc < 0.8f * hs ? p_aeb_max_dec : fminf(fmaxf(1.4f * a_need, fl * p_aeb_part_dec), p_aeb_max_dec);
    if ((st == AEB_PARTIAL || st == AEB_FULL) && !fresh) a_cmd = fmaxf(a_cmd, AE.a_cmd);
    AE.a_cmd = a_cmd;
    if (st == AEB_HOLD) target = 0.4f;
    else if (a_cmd > 0) {
        float ff = (a_cmd - 0.4f) / 10.0f * heavy;
        if (now - AE.st_at > 0.6) AE.i = fminf(0.5f, fmaxf(-0.15f, AE.i + 0.08f * (a_cmd + T.accel) * dt));
        target = fminf(1, fmaxf(0.05f, ff + AE.i));
        if (T.air_psi > 1 && T.air_psi < 90 && st < AEB_FULL) target = fminf(target, 0.6f);  // save the air outside full braking
    } else AE.i = 0;
    float up = st == AEB_FULL ? 6.0f : 3.0f, dn = 1.5f;
    // low air: no pulsing while braking (each new application costs air), hold it steady
    if (T.air_warning || (T.air_psi > 1 && T.air_psi < p_air_min)) AE.air_low = 1;  // for the rest of this event
    if (AE.air_low && (st == AEB_PARTIAL || st == AEB_FULL) && vc > 0) dn = 0;
    // one application, not a pumping one: while braking and still closing in, small dips of the target are ignored (each
    // release and re-apply costs air; the sim pumped 56 times following at 7 m with the fast filters)
    if ((st == AEB_PARTIAL || st == AEB_FULL) && vc > 0 && target < AE.req && target > AE.req - 0.08f) target = AE.req;
    AE.req = target > AE.req ? fminf(target, AE.req + up * dt) : fmaxf(target, AE.req - dn * dt);
    if (AE.req < 0.005f && target <= 0) AE.req = 0;
    s_want_aeb = AE.req;
    aeb_record(fresh ? gap : -1.0f, kmh, vc, ttc, a_need);
}

static void cr_frame()
{
    g_inc.gap_m = g_radar_ok && wall_s() - s_radar.at < 0.3 && s_radar.gap > 0.3f ? s_radar.gap : NAN;
    g_inc.flags = (g_inc.flags & ~(INC_AEB | INC_ISA)) | (AE.st != AEB_OFF ? INC_AEB : 0) | (p_isa > 0 ? INC_ISA : 0);
    DEV_NOTE(cruise(T.cruise > 0.5f, T.cruise * 3.6f, g_acc ? g_level : -1, g_radar_ok ? s_radar.gap : NAN, s_radar.v,
                    s_radar.at, p_isa > 0, AE.st, AH.st, s_dip));
    if (!T.paused) {  // say when cruise drops and why it might have, and when it is back (a driver who pressed nothing wants to know)
        static int was_on; static double off_at; static float set_kmh;
        int is_on = T.cruise > 0.5f;
        if (is_on) set_kmh = T.cruise * 3.6f;
        if (was_on && !is_on) {
            off_at = wall_s();
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
    route_junction_cfg(junc_on(), p_junc_lat, p_rb_kmh, p_junc_deg, p_rb_ratio);
    if (g_keys_ok) keys_tick(game_actor_safe());
    emergency_hazards();
    // Speed-limit following on/off: hold the game's own ACC distance key (no wheel button number: the key is read from the
    // game's action object, so a wheel or a keyboard both work). The game already stepped the distance on the press, a long
    // press puts it back. Held = the action state is not 0; it must have been seen at 0 first (else a state that never
    // reads 0 would fire at start). A held key may flicker: 0.3 s up ends a hold.
    if (g_low_ok && p_isa_hold > 0 && !s_acc_key_bad) {
        double now = wall_s();
        char *ga = game_actor_safe();
        void *act = ga ? action_of(ga, ACT_ACCMODE) : NULL;
        int st = act ? *(int *)((char *)act + ACT_STATE) : 0;
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
                    game_cvar_set(g_acc, back);
                    g_acc_player = back;
                    g_level = level_of(back);
                    g_acc_ours = NAN;
                }
                s_press_before = NAN;
                p_isa = p_isa > 0 ? 0 : 1;
                s_isa_last = -1;
                state_write();  // remembered
                play_cue(p_isa > 0 ? CUE_ON : CUE_OFF);
                g_dash_flash = p_isa > 0 ? 'S' : 'X';  // S = following the speed signs, X = off (1.5 s)
                g_dash_flash_until = wall_s() + 1.5;
                logline("Cruise: speed limit following %s", p_isa > 0 ? "on" : "off");
            }
        }
        s_btn_held = held || (s_btn_up_at && now - s_btn_up_at <= 0.3);
    }
    // Following: at every new limit (and when cruise comes on) the set speed
    // becomes limit + offset; in between +/- are the driver's.
    if (g_low_ok) {
        char *ga = game_actor_safe();
        int on = ga && *(unsigned char *)(ga + GA_ON);
        if (!on || p_isa <= 0) s_isa_last = -1;
        else if (T.speed_limit > 1) {
            int lim = (int)roundf(T.speed_limit * 3.6f);
            if (s_isa_last < 0 && wall_s() - s_res_press_at > 1.0) {
                // Switched on with set (not resume): the current speed stays, like the game; ISA takes over at the
                // next sign (Damien 11-10: "set goes straight to the ISA speed, set and resume do the same").
                s_isa_last = lim;
                logv(1, "Cruise: set at %.0f km/h with ISA on: kept, limit %d follows from the next sign", *(float *)(ga + GA_SET), lim);
            } else if (lim != s_isa_last) {
                s_isa_last = lim;
                float set = fmaxf(lim + p_isa_off, p_min_set), cur = *(float *)(ga + GA_SET);
                // Lower: ease the set speed down (Damien: not full on the brakes at the sign), higher: at once.
                cur = fmaxf(set, fminf(cur, fabsf(T.speed) * 3.6f + 1));  // already slower than the old set speed: start from our speed
                if (p_isa_ramp > 0 && set < cur - 1) { isa_goal = set; isa_ours = cur; *(float *)(ga + GA_SET) = cur; }
                else { isa_goal = NAN; *(float *)(ga + GA_SET) = set; }
                logv(1, "Cruise: speed limit %d km/h: set speed %.0f%s", lim, set, isa_goal == isa_goal ? " (easing down)" : "");
            }
        }
        // Probe for the test ride: every change of the SDK limit dumps the route nodes (docs/speedlimit-ahead-re.md).
        {
            static int probe_last = -1;
            int pl = T.speed_limit > 1 ? (int)roundf(T.speed_limit * 3.6f) : 0;
            if (p_isa_probe > 0 && pl != probe_last) { probe_last = pl; route_probe_limit((float)pl); }
        }
        // A lower limit AHEAD on the route: ease so that the set speed is limit + offset at the sign
        // (comfortable braking ahead_dec, so v(d) = sqrt(v_t^2 + 2 a d)). Needs route_limit_ahead (unknown source: NAN).
        {
            float *set = (float *)(ga + GA_SET), dist = NAN, v_t = NAN;
            if (on && p_isa > 0 && p_isa_ahead > 0 && ga) {
                float v = fmaxf(fabsf(T.speed), 5.0f);
                float lim = route_limit_ahead(v * v / (2 * 0.5f) + 150, &dist);
                if (lim == lim && dist == dist && dist >= 0) {
                    int li = (int)roundf(lim);
                    v_t = fmaxf(li + p_isa_off, p_min_set) / 3.6f;
                    if (li != s_ahead_logged) { s_ahead_logged = li; logv(2, "Cruise: limit %d ahead in %.0f m", li, dist); }
                    if (s_ahead_ours == s_ahead_ours && fabsf(*set - s_ahead_ours) > 0.5f) s_ahead_ignore = li;  // the driver changed it
                    float v_ideal = sqrtf(v_t * v_t + 2 * 0.5f * dist) * 3.6f;
                    if (s_ahead_ignore != li && v_ideal < *set - 0.1f && *set > v_t * 3.6f) {
                        *set = fmaxf(v_t * 3.6f, v_ideal);
                        s_ahead_ours = *set;
                    } else s_ahead_ours = NAN;
                } else { s_ahead_ours = NAN; s_ahead_logged = -1; s_ahead_ignore = -1; }
            } else { s_ahead_ours = NAN; s_ahead_logged = -1; s_ahead_ignore = -1; }
        }
        // easing down; a +/- press by the driver (set speed not what we wrote) ends it
        if (on && isa_goal == isa_goal) {
            double now = wall_s(), dt = now - s_isa_at;
            s_isa_at = now;
            float *set = (float *)(ga + GA_SET);
            if (fabsf(*set - isa_ours) > 0.5f) isa_goal = NAN;  // the driver changed it
            else if (dt > 0 && dt < 0.5) {
                isa_ours = fmaxf(isa_goal, isa_ours - p_isa_ramp * (float)dt);
                *set = isa_ours;
                if (isa_ours <= isa_goal) isa_goal = NAN;
            }
        }
        if (!on && isa_goal == isa_goal && ga) *(float *)(ga + GA_SET) = isa_goal;  // cruise off: the goal at once
        if (!on) isa_goal = NAN;
    }
    // Switched on below min_set_kmh (or set lower with -): set speed up to min_set_kmh.
    if (g_low_ok && p_min_set > 0) {
        char *ga = game_actor_safe();
        if (ga && *(unsigned char *)(ga + GA_ON) && *(float *)(ga + GA_SET) < p_min_set) *(float *)(ga + GA_SET) = p_min_set;
    }
    // The game's speed limiter on (g_use_speed_limiter): the set speed never above it (Damien: + / hold went to 110).
    if (g_low_ok && p_limiter > 0) {
        char *ga = game_actor_safe();
        if (limiter_on() && ga && *(unsigned char *)(ga + GA_ON) && *(float *)(ga + GA_SET) > p_limiter)
            *(float *)(ga + GA_SET) = p_limiter;
    }
    // Every lower set speed eases down (Damien: "so cool, do it for everything with cruise"): -, hold -, resume
    // to a lower limit, ISA. The game brakes on the set speed itself, so the set speed is walked down; a higher
    // one is taken at once (our throttle is smooth anyway). Pressing + or a new lower value takes over.
    if (s_anim_to == s_anim_to) {
        char *ga = game_actor_safe();
        float k = p_key_slide > 0 ? (float)((wall_s() - s_anim_t0) / p_key_slide) : 1;
        if (!ga || !*(unsigned char *)(ga + GA_ON) || k >= 1) { if (ga) *(float *)(ga + GA_SET) = s_anim_to; s_anim_to = NAN; }
        else *(float *)(ga + GA_SET) = s_anim_from + (s_anim_to - s_anim_from) * k;
        if (ga) { s_ease_prev = *(float *)(ga + GA_SET); s_ease_goal = NAN; }
    }
    if (g_low_ok && p_set_ease > 0 && !(s_anim_to == s_anim_to)) {
        double now = wall_s(), dt = now - s_ease_at;
        s_ease_at = now;
        char *ga = game_actor_safe();
        if (!ga || T.paused) s_ease_prev = s_ease_goal = NAN;
        else if (!*(unsigned char *)(ga + GA_ON)) {  // cruise off: no easing in the background, the s_ease_goal at once (Damien)
            if (s_ease_goal == s_ease_goal) *(float *)(ga + GA_SET) = s_ease_goal;
            s_ease_prev = s_ease_goal = NAN;
        }
        else {
            float *set = (float *)(ga + GA_SET);
            // a step down: start from where we were, or at once from the speed we actually drive if that is lower
            // (Damien: slowed by a climb, then a lower limit: no easing from a speed we are not doing)
            if (s_ease_prev == s_ease_prev && *set < s_ease_prev - 0.3f) { s_ease_goal = *set; *set = fmaxf(s_ease_goal, fminf(s_ease_prev, fabsf(T.speed) * 3.6f + 1)); }
            else if (s_ease_prev == s_ease_prev && *set > s_ease_prev + 0.05f) s_ease_goal = NAN;              // up: at once
            if (s_ease_goal == s_ease_goal && dt > 0 && dt < 0.5) {
                *set = fmaxf(s_ease_goal, *set - p_set_ease * (float)dt);
                if (*set <= s_ease_goal) s_ease_goal = NAN;
            }
            s_ease_prev = *set;
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
            int air_ok = !T.air_warning && !(T.air_psi > 1 && T.air_psi < p_air_min);
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
            float want = down ? (s_dip ? up : fmaxf(0.0f, p_desc_kmh)) : smart_base();
            want = fmaxf(want, s_mom_up + 0.5f * (s_mom_up > 0));
            static int said = 0;  // 0 flat, 1 descent, 2 dip
            int st = down ? 1 + s_dip : 0;
            if (st != said) {
                if (st == 2) logv(2, "Cruise: dip ahead (bottom in %.0f m): up to +%.1f km/h into the climb", vy, up);
                else logv(2, "Cruise: descent %s", st ? "ahead: braking from the set speed" : "over");
                said = st;
            }
            if (!(g_smart_ours == g_smart_ours) || fabsf(want - g_smart_ours) > 0.01f) {
                game_cvar_set(g_smart, want);
                g_smart_ours = want;
            }
        }
    }
    if (!g_acc || T.paused || s_btn_held) return;  // a distance press is announced when the button is released
    double now = wall_s();
    if (now - s_gacc_at < 0.5) return;
    s_gacc_at = now;
    // A round value we did not write: the player picked a distance in the options.
    float cur = game_cvar_get(g_acc);
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
            g_dash_flash = p_acc_letter > 0 && g_level >= 0 ? 'A' : "C123"[g_level + 1];
            g_dash_flash_until = wall_s() + 1.5;
        }
        g_acc_player = cur;
        g_acc_ours = NAN;
    }
    if (!(T.cruise > 0.5f) || g_level < 0) {  // only while cruising, and never when the player set ACC off
        // not cruising: the player's own value back (ts3c started with our 45 saved in the profile)
        if (g_acc_ours == g_acc_ours && g_acc_player == g_acc_player) { game_cvar_set(g_acc, g_acc_player); g_acc_ours = NAN; }
        return;
    }
    float v = fmaxf(fabsf(T.speed), 8.0f);
    float gap_m = fminf(fmaxf(gap_s() * v, p_min_m), p_max_m);
    float want = roundf(gap_m / (0.25f + v / 30) * 2) / 2;  // g_acc for that gap at this speed
    if (is_round(want)) want += 0.1f;                         // never a value the menu could have set
    if (g_acc_ours == g_acc_ours && fabsf(want - g_acc_ours) < 2) return;
    game_cvar_set(g_acc, want);
    if (!(g_acc_ours == g_acc_ours) || fabsf(want - g_acc_ours) >= 10)
        logv(2, "Cruise: ACC gap %.0f m (%.1f s at %.0f km/h, g_acc %.1f)", gap_m, gap_s(), fabsf(T.speed) * 3.6f, want);
    g_acc_ours = want;
}

static void cr_stop()
{
    if (s_inc) { fclose(s_inc); s_inc = NULL; }
    AH.r = 0; s_want_hold = 0; s_thr_pass = 0;
    if (g_csv) fflush(g_csv);
    if (s_req_ours > 0) { char *ga = game_actor_safe(); if (ga) brake_request(ga, 0); }
    if (g_smart && g_smart_player == g_smart_player) game_cvar_set(g_smart, g_smart_player);
    if (g_acc && g_acc_player == g_acc_player && g_acc_ours == g_acc_ours) {
        game_cvar_set(g_acc, g_acc_player);
        g_acc_ours = NAN;
    }
}

// Continuity broke (see RESET_*): nothing timed or filtered may carry over. SOFT clears the key timers, Auto Hold,
// easing anchors, filters, the PI start and the hysteresis states; HARD also the own-ACC state and puts the
// player's g_acc / smart-overspeed values back (they are re-derived from the live game afterwards).
static void cr_reset(int level, const char *why)
{
    (void)why;
    char *ga = g_low_ok ? game_actor_safe() : NULL;
    // keys
    for (int d = 0; d < 2; d++) K[d] = keydir();
    s_res_ev = 0;
    s_lv_prev = NAN; s_a_lead = 0;
    s_prev_on = ga ? *(unsigned char *)(ga + GA_ON) : 0;
    s_btn_down_at = s_btn_up_at = 0;
    s_btn_fired = 0;
    s_acc_armed = 0; s_hold_before = NAN;
    s_press_before = NAN;
    s_btn_held = 0;
    // Auto Hold / hill start / our brake request back to 0
    AH.st = AH_OFF; AH.r = 0; AH.released = 0; AH.launch_t = 0; AH.grab_until = 0; AH.roll_m = 0; AH.mass_at = -10;
    s_want_hill = s_want_hold = s_want_curve = s_want_aeb = 0;
    s_aeb_was = 0;
    { float m = AE.mass; memset(&AE, 0, sizeof AE); AE.mass = m; AE.gap_prev = AE.rate = AE.vl_prev = NAN; }  // AEB: no event spans a reset
    s_hs_armed = 0; s_hs_until = 0;
    s_hill_was = s_hold_was = 0;
    s_br_at = 0;
    s_thr_pass = 0;
    if (g_dash_status == 'B') g_dash_status = 0;
    if (s_req_ours > 0 && ga) {
        float *req = (float *)(ga + GA_BRAKE_REQ);
        if (*req == s_req_ours) *req = 0;
    }
    s_req_ours = 0;
    s_hazard_mute = 0;
    // set speed and ISA easing
    s_ease_prev = s_ease_goal = NAN; s_ease_at = 0;
    isa_goal = NAN; isa_ours = 0; s_isa_at = 0;
    s_ahead_ours = NAN; s_ahead_ignore = s_ahead_logged = -1;
    // calm cut-in / own-ACC filters
    s_lead_prev = -1; s_gap_rate = 0;
    s_lead_v = s_acc_game = s_acc_own = s_dstar = NAN;
    s_radar.at = 0;
    OA.on = 0;
    // PI: bumpless restart; stop&go and descent/curve hysteresis
    C.on = 0; C.u = C.i = 0;
    g_sg_hold = 0; g_feed = 0; g_stopped_at = 0;
    s_curve_braking = 0;
    s_down_now = 0; s_dip = 0; s_dip_spent = 0; s_dip_at = 0; s_flat_since = 0; s_desc_at = 0;
    s_gacc_at = 0;
    // emergency hazards: ours go off
    if (s_haz_ours && ga && hazards_on(ga)) hazards_set(ga, 0);
    s_haz_ours = 0; s_haz_hard_at = s_haz_go_at = 0;
    if (level >= RESET_HARD) {
        s_isa_last = -1;
        OA.t = 0;
        s_ah_k = 1;
        s_air_low = 0;
        s_gacc_seen = NAN;
        if (g_smart && g_smart_player == g_smart_player) game_cvar_set(g_smart, smart_base());
        g_smart_ours = NAN;
        if (g_acc && g_acc_player == g_acc_player && g_acc_ours == g_acc_ours) game_cvar_set(g_acc, g_acc_player);
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
             p_ah > 0 && g_low_ok && g_cruise_smooth ? "on" : "off", g_sg_ok ? "on" : "off",
             p_curve > 0 && g_cruise_smooth ? "on" : "off", junc_on() ? (p_junc > 0 ? "on" : "on (own ACC)") : "off", g_smart ? "on" : "off",
             p_ess > 0 && g_low_ok ? "on" : "off", s_aeb_owns ? (p_aeb_manual > 0 ? "own (also without cruise)" : "own (with cruise)") : "game's", g_low_ok ? "yes" : "no", g_keys_ok ? "yes" : "no", g_radar_ok ? "yes" : "no");
    return b;
}

module m_cruise = { "cruise", "Cruise", g_params, sizeof g_params / sizeof g_params[0], cr_start, cr_frame, NULL,
                    cr_stop, cr_reset, cr_status };
