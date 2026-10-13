// TrueShift: makes the game's automatic shift like a real truck AMT.
//
// How it hooks in (ETS2 1.60, see docs/trueshift.md):
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
// manual. The dashboard's "A" becomes D/E/P/M.
#include "../core/suite.h"
#include "../core/game.h"
#include "../core/route.h"
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------- settings

static float p_enabled, p_takeover, p_on_kmh, p_off_kmh, p_on_rpm, p_hint_hold, p_min_gap, p_dwell_ud, p_dwell_du,
    p_max_skip, p_skip_acc, p_hyst, p_kick_pedal, p_kick_rate, p_power_from, p_power_full,
    p_cruise_rpm, p_reserve, p_learn_rpm, p_learn_res, p_learn_km, p_eco_rpm, p_power_rpm, p_eco_reserve, p_power_reserve, p_cc_damp,
    p_eb_down, p_eb_light, p_eb_skip, p_m_manual, p_m_assist, p_m_lug, p_letter, p_start_fix, p_launch_max, p_slip_s, p_launch_torque, p_launch_rpm,
    p_crr, p_cda, p_eta, p_idle, p_trace, p_lookahead, p_roll, p_roll_dec, p_roll_under, p_roll_under_eco, p_roll_under_pw, p_ret_down, p_air_psi, p_air_rpm, p_no_creep, p_wet, p_stop, p_tones, p_crest_band;

static param g_params[] = {
    { "enabled", &p_enabled, 1, "1 = TrueShift on, 0 = the game's own automatic" },
    { "takeover", &p_takeover, 1, "1 = TrueShift picks the gears, 0 = only log what it would do" },
    { "takeover_kmh", &p_on_kmh, 7, "TrueShift takes over from the game's automatic above this speed" },
    { "takeover_rpm", &p_on_rpm, 1000, "TrueShift takes over as soon as the engine has reached this rpm with the clutch closed in any gear (the clutch is locked then, so the rpm is real); 0 = by speed only (takeover_kmh)" },
    { "handback_kmh", &p_off_kmh, 4, "and hands back below this speed (starting, stopping, maneuvering)" },
    { "hint_hold_s", &p_hint_hold, 10, "seconds a gear chosen with the shift-hint buttons is held (0 = off)" },
    { "min_gap_s", &p_min_gap, 0.5f, "minimum time between two shifts" },
    { "dwell_up_down_s", &p_dwell_ud, 3.0f, "after an upshift, no downshift for this long (unless kickdown or lugging)" },
    { "dwell_down_up_s", &p_dwell_du, 2.0f, "after a downshift, no upshift for this long (unless clearly past the upshift point)" },
    { "max_skip", &p_max_skip, 2, "most gears an upshift may skip (0 = never skip)" },
    { "skip_accel", &p_skip_acc, 0.15f, "acceleration (m/s2) the skipped-to gear must still give" },
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
    { "m_full_manual", &p_m_manual, 0, "M mode: 1 = only rev limiter and stall protection, 0 = also downshifts when stopping" },
    { "m_assist", &p_m_assist, 1, "M mode: 1 = also kickdown and one gear down when lugging (like a VW DSG), 0 = full pedal at low rpm is yours" },
    { "m_lug_rpm", &p_m_lug, 1000, "M mode with m_assist: one gear down below this rpm with a lot of pedal and no pull" },
    { "dash_letter", &p_letter, 1, "1 = the dashboard shows the mode (D, E, P, M) instead of A" },
    { "start_gear_fix", &p_start_fix, 1, "1 = lower the game's start gear when the launch would be too slow" },
    { "launch_max_kmh", &p_launch_max, 14, "a start gear above the game's pick is allowed when the clutch would be closed by this speed (km/h at launch_rpm) and the slip time stays within launch_slip_s: on a light or downhill start the truck no longer revs a low gear to the limiter. Not in Power or M. 0 = never above the game's pick" },
    { "launch_slip_s", &p_slip_s, 1.0f, "longest clutch-slip time accepted for a start gear" },
    { "launch_torque", &p_launch_torque, 0.6f, "share of max torque the clutch passes while starting" },
    { "launch_rpm", &p_launch_rpm, 1000, "rpm at which the clutch closes when starting" },
    { "rolling_resistance", &p_crr, 0.006f, "rolling resistance coefficient" },
    { "drag_area", &p_cda, 6.0f, "air drag Cd*A in m2 for trucks (vehicles under 4.5 t use 1.0: cars, pickups)" },
    { "efficiency", &p_eta, 0.9f, "driveline efficiency" },
    { "idle_rpm", &p_idle, 550, "engine idle rpm to start with (learned while standing still)" },
    { "lookahead", &p_lookahead, 1, "1 = use the navigation route ahead (climbs, tops, junctions); without a route TrueShift stays careful" },
    { "crest_band_kmh", &p_crest_band, 5, "with the route (predictive cruise, PCC): no downshift for a top ahead that this gear gets over at full throttle while losing at most this much below the set speed (without cruise: below the speed when the top came into view) and staying on the torque plateau. Full at 40 t, 60 % solo, 60 % in Power; 0 = off" },
    { "eco_roll_under_kmh", &p_roll_under, 1.5f, "Normal mode, with cruise control: the roll ends when the speed is about to drop this far below the set speed (km/h; Volvo I-Roll: 1-3)" },
    { "eco_roll_under_eco_kmh", &p_roll_under_eco, 2.0f, "the same in Eco: a little more room, the most relaxed" },
    { "eco_roll_under_power_kmh", &p_roll_under_pw, 1.0f, "the same in Power: the strictest (keep it between 1 and 2 km/h; values around 0.5 give very short rolls that hunt)" },
    { "eco_roll_max_decel", &p_roll_dec, 0.04f, "eco-roll only where the truck would lose at most this much speed in neutral (m/s2; 0.04 = ~3 km/h in 20 s: a slight descent, not the flat)" },
    { "eco_roll", &p_roll, 1, "1 = roll in neutral with the pedal released on gentle stretches (Eco, Normal and Power; not in M; never with any brake)" },
    { "tones", &p_tones, 1, "1 = a short tone when the mode changes (Eco low, Normal middle, Power rising, M two short)" },
    { "retarder_downshift", &p_ret_down, 0, "1 = the retarder also makes TrueShift keep the rpm up like the engine brake (a retarder does not need it); 0 = only the engine brake does" },
    { "air_charge_psi", &p_air_psi, 100, "brake air below this (psi): keep the rpm up so the engine-driven compressor refills it; 0 = off" },
    { "air_charge_rpm", &p_air_rpm, 1200, "the rpm kept while charging (no upshift below it)" },
    { "no_creep", &p_no_creep, 1, "1 = with crawl switched off the truck never creeps on the clutch at idle: pedals free below 8 km/h opens the clutch" },
    { "smooth_stop", &p_stop, 1, "1 = when braking to a stop the clutch opens around idle +150 rpm (more when braking harder) instead of pushing to ~2 km/h" },
    { "wet_wipers", &p_wet, 0, "1 = wipers on means a slippery road: start a gear higher, shift up earlier in low gears, softer kickdown. Off by default: with automatic wipers the lever is always on, and the game's traction control handles grip" },
    { "trace", &p_trace, 1, "1 = write trueshift.csv (10 per second) for analysis" },
};

// ---------------------------------------------------------------- state

enum { MODE_NORMAL, MODE_ECO, MODE_POWER, MODE_M };
static const char *const mode_name[] = { "Normal", "Eco", "Power", "M" };
static const char mode_letter[] = { 'D', 'E', 'P', 'M' };

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

static struct {
    void *self;
    engine_info eng;
    int eng_ok;
    double eng_at;
    float plateau;          // rpm where the torque curve reaches its plateau
    float mass, grade_deg;
    double mass_at;
    float rpm_k;            // measured rpm / speed-model rpm (tire size etc.)
    float thr;              // throttle that counts: pedal, or damped cruise control
    float cc_thr;           // cruise control throttle, smoothed
    float demand;           // smoothed demand 0..1
    double demand_at;
    float f_need;           // tractive force asked for (N), up fast, down slow
    float pedal_prev;
    double pedal_at;
    double kick_until;
    int mode;
    int ours;               // above the takeover speed (with hysteresis)
    double ours_at;
    int ours_launch;        // taken over while pulling away (below 25 km/h), not after a pause or a teleport at speed
    int last_gear;
    double shift_at;
    int shift_dir;
    int commanded;          // last gear we returned to the game
    const char *cmd_why;
    const char *last_why;   // reason of the last shift we made
    double hold_until;
    int up_gear;            // last upshift we made, to judge whether it held
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
    float f_bias;           // learned extra resistance (N): what the model misses
    const char *why;        // reason of the last decision, for the trace
    int want, target;
    double blocked_log_at;
    float up_pt, dn_pt, floor_rpm, eb_level;
    int eb_full;            // the game itself would shift into the engine-brake range now
    int letter_state;       // -1 unknown, 0 not found, 1 written
    float idle;             // learned idle rpm (0 = not yet)
    int rolling;            // eco-roll: in neutral by our choice
    double roll_at;
    float roll_kmh;         // speed when the roll started
    int roll_exit;          // gear to engage after a roll (kept as target until it is in)
    double roll_exit_at;
    int wet;                // wipers on
    double pedal_on_at;     // last time the pedal was pressed
    const char *unsupported; // why this truck is left to the game's own automatic (NULL = fine)
    float rpm_odd_s;        // seconds the rpm did not follow the speed model at all (torque converter?)
    int manual_logged;
} S;

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
static float grade_pct() { return tanf(S.grade_deg * 0.0174533f) * 100; }
static float idle_rpm() { return S.idle > 300 ? S.idle : p_idle; }

static int n_gears()
{
    int n = T.fwd_gears;
    return n > 0 && n < 32 ? n : 0;
}

// Engine rpm in gear g at speed v (m/s), from the driveline, corrected by the
// measured ratio to the real rpm.
static float rpm_at(int g, float v)
{
    float r = wheel_radius();
    if (g < 1 || g > n_gears() || r <= 0.1f || T.diff <= 0) return 0;
    return v / r * T.ratio[g - 1] * T.diff * 9.5493f * S.rpm_k;
}

// Tractive force (N) in gear g at speed v with a share of the available torque.
static float force_at(int g, float v, float share)
{
    float r = wheel_radius();
    if (g < 1 || g > n_gears() || r <= 0.1f) return 0;
    float rpm = rpm_at(g, v);
    float t = S.eng.torque * curve_factor(&S.eng, rpm) * share;
    if (rpm > S.eng.rpm_limit) t = 0;
    return t * T.ratio[g - 1] * T.diff * p_eta / r;
}

// Air drag area: the truck value from the ini, a car-like one for light vehicles.
static float cda() { return S.mass > 0 && S.mass < 4500 ? 1.0f : p_cda; }

static float resist(float v)
{
    float s = sinf(S.grade_deg * 0.0174533f);
    return S.mass * 9.81f * (s + p_crr) + 0.5f * 1.2f * cda() * v * v + S.f_bias;
}

static float resist_grade(float v, float pct)
{
    float s = sinf(atanf(pct / 100));
    return S.mass * 9.81f * (s + p_crr) + 0.5f * 1.2f * cda() * v * v + S.f_bias;
}

// Start of the torque plateau: first curve point within 3% of the peak.
static float find_plateau(const engine_info *e)
{
    float peak = 0;
    for (int i = 0; i < e->n_curve; i++) peak = fmaxf(peak, e->curve[i][1]);
    for (int i = 0; i < e->n_curve; i++)
        if (e->curve[i][1] >= 0.97f * peak && e->curve[i][0] > 300) return e->curve[i][0];
    return e->range[R_HIGH][0];
}

// ---------------------------------------------------------------- shift points

static float mode_rpm() { return S.mode == MODE_ECO ? p_eco_rpm : S.mode == MODE_POWER ? p_power_rpm : 0; }
static float mode_reserve()
{
    return S.mode == MODE_ECO ? p_eco_reserve : S.mode == MODE_POWER ? p_power_reserve : p_reserve;
}

// Stock shift points interpolated from low to high gear, like the game does.
static float eco_dn(int g)
{
    int n = n_gears();
    float k = n > 1 ? (float)(g - 1) / (n - 1) : 1;
    return lerpf(S.eng.range[R_LOW][0], S.eng.range[R_HIGH][0], k);
}
static float eco_up(int g)
{
    int n = n_gears();
    float k = n > 1 ? (float)(g - 1) / (n - 1) : 1;
    return lerpf(S.eng.range[R_LOW][1], S.eng.range[R_HIGH][1], k);
}
static float pw_k() { return smooth(p_power_from, p_power_full, S.demand); }
static float pw_mid() { return 0.5f * (S.eng.range[R_POWER][0] + S.eng.range[R_POWER][1]); }
static float up_pt(int g)
{
    // wet: -100 rpm in gears 1-3, fading out by gear 7 (ts3b: a step at gear 5 felt like on/off)
    float mo = mode_rpm() - (S.wet ? 100 * clampf((7 - g) / 4.0f, 0, 1) : 0);
    return fminf(lerpf(eco_up(g) + mo, S.eng.range[R_POWER][1] + 0.5f * mo, pw_k()), S.eng.rpm_limit * 0.94f);
}
// Light-load downshift point: below the plateau, so that an early upshift to
// the plateau has room before it has to come back down.
static float base_dn(int g)
{
    return fmaxf(idle_rpm() + 220, fminf(eco_dn(g), S.plateau - 100) + 0.5f * mode_rpm());
}
// The downshift point only rises with demand near full pedal: pressing on at
// cruise in a low gear is not a request for power (kickdown and lugging are).
static float dn_k()
{
    float from = S.mode == MODE_ECO ? 0.9f : S.mode == MODE_POWER ? 0.75f : 0.85f;
    return smooth(from, p_power_full, S.demand);
}
static float dn_pt(int g) { return lerpf(base_dn(g), S.eng.range[R_POWER][0] + 0.5f * mode_rpm(), dn_k()); }
// Pull (m/s2) below which a demand downshift makes sense; a light truck with
// plenty of pull left keeps its gear (and may skip on the way up).
static float a_power() { return S.mode == MODE_ECO ? 0.2f : S.mode == MODE_POWER ? 0.45f : 0.3f; }
static lesson learned();

// Lowest landing rpm of an early upshift.
// A skip (2 steps or more) may land up to 100 rpm under the plateau (Damien 12-10: fewer shifts when pulling lightly); the
// downshift point plus hysteresis stays the lower bound, so it does not hunt back.
static float cruise_floor(int g, float under = 0)
{
    return fmaxf(S.plateau + p_cruise_rpm + mode_rpm() - under, base_dn(g) + p_hyst + 50) + learned().rpm;
}

static int skip_allowed() { return (int)p_max_skip; }
// Pull a skipped-to gear must keep: less with a slow box (every shift is a
// long gap without power). Damien (ts3b): skip also with a fast (DCT) box.
static float skip_accel() { return p_skip_acc * (1.0f - 0.25f * smooth(0.4f, 1.5f, S.eng.shift_time)); }

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
        float a_h = (force_at(h, v, 0.95f) - f_res) / S.mass;  // still accelerating hard: the plateau is enough
        if (rpm_at(h, v) < fmaxf((a_h < a_power() ? dn_pt(h) : base_dn(h)) + p_hyst, idle_rpm() + 350)) break;
        if ((force_at(h, v, 0.85f) - f_res) / S.mass < a_skip) break;
        if (force_at(h, v, 1.0f) - f_res < 0.7f * (force_at(cur + 1, v, 1.0f) - f_res)) break;
        best = h;
    }
    return best;
}

// Gear below cur whose rpm at speed v lands nearest to target, without
// landing above its own upshift point or near the limiter.
static int pick_down(int cur, float v, float target)
{
    int best = cur;
    float best_err = 1e9f;
    for (int l = cur - 1; l >= 1; l--) {
        float rpm = rpm_at(l, v);
        if (rpm > S.eng.rpm_limit * 0.93f) break;
        if (rpm > up_pt(l) - p_hyst && l < cur - 1) break;
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
    if (!(g_learn_mass > 0) || fabsf(S.mass - g_learn_mass) > 0.25f * g_learn_mass) {
        if (g_learn_mass > 0 && (g_learn[0].rpm + g_learn[1].rpm + g_learn[2].rpm + g_learn[3].rpm + g_learn[4].rpm) > 0)
            logline("load changed (%.1f -> %.1f t): learned upshift limits cleared", g_learn_mass / 1000, S.mass / 1000);
        memset(g_learn, 0, sizeof g_learn);
        g_learn_mass = S.mass;
    }
    if (p_learn_km <= 0) return;
    float k = expf(-dist_m * 0.6931f / (p_learn_km * 1000));
    for (int c = 0; c < N_CLASS; c++) { g_learn[c].rpm *= k; g_learn[c].res *= k; }
}

// ---------------------------------------------------------------- inputs

static int mode_from_cvar(float a)
{
    if (a != a) return MODE_NORMAL;
    if (a <= 0) return MODE_M;
    if (a > 3.01f) return MODE_ECO;
    if (a > 1.67f) return MODE_NORMAL;
    return MODE_POWER;
}

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
    S.demand_at = S.pedal_at = 0;           // the next dt is "unknown": filters step, no kickdown from a stale pedal
    S.ours_at = S.shift_at = none;          // dwell and take-over timers must not hold a shift back after a pause
    S.top_at = none; S.crest_ref = 0; S.crest_said = 0;  // no "over the top" hold across a teleport
    S.blocked_log_at = 0;
    S.pedal_prev = clampf(T.in_thr, 0, 1);
    if (level >= RESET_LOAD) {
        S.mass_at = none;                   // read the mass again at once
        S.f_bias = 0;
        S.f_need = 0;
        S.up_gear = 0;
        S.strain = 0;
    }
    if (level >= RESET_HARD) {
        memset(g_learn, 0, sizeof g_learn);
        g_learn_mass = 0;
        S.self = NULL;                      // refresh() starts over from the gearbox it sees next
        S.eng_ok = 0;
        S.eng_at = none;
        S.mass = 0;
        S.idle = 0;                         // idle is learned again (engine on 5 s, standing still)
        S.rpm_k = 1;
        S.rpm_odd_s = 0;
        S.unsupported = NULL;
        memset(S.grade_hist, 0, sizeof S.grade_hist);
        S.letter_state = -1;
        S.cc_thr = 0;
        S.demand = 0;
        S.rolling = 0;
        S.roll_exit = 0;
        g_rolling = 0;
    }
}

static void refresh(void *self)
{
    double now = sim_s();
    if (self != S.self) {
        // another truck (or the first update): start from its current gear
        if (S.self) ts_reset(RESET_HARD, "gearbox object changed");
        S.self = self;
        S.last_gear = S.commanded = gb_gear(self);
        S.hold_until = S.kick_until = 0;
        S.letter_state = -1;
    }
    if (!S.eng_ok || now - S.eng_at > 5.0 || now < S.eng_at) {
        engine_info e;
        memset(&e, 0, sizeof e);
        int ok = gb_engine(self, &e);
        if (e.electric) unsupported("electric motor: no gearbox to shift");
        if (ok && (!S.eng_ok || e.torque != S.eng.torque || e.rpm_limit != S.eng.rpm_limit)) {
            logline("engine: %.0f Nm, limit %.0f rpm, %d curve points, plateau from %.0f rpm, ranges brake %.0f-%.0f "
                    "low %.0f-%.0f high %.0f-%.0f power %.0f-%.0f, shift time %.2f s",
                    e.torque, e.rpm_limit, e.n_curve, find_plateau(&e), e.range[0][0], e.range[0][1], e.range[1][0],
                    e.range[1][1], e.range[2][0], e.range[2][1], e.range[3][0], e.range[3][1], e.shift_time);
            S.letter_state = -1;
            S.f_bias = 0;
        }
        if (!ok && S.eng_ok) logline("engine data unreadable, TrueShift pauses");
        if (ok) { S.eng = e; S.plateau = find_plateau(&e); }
        S.eng_ok = ok;
        S.eng_at = now;
    }
    if (now - S.mass_at > 0.5 || now < S.mass_at) {
        float m = game_mass();
        if (m == m) {
            if (!(S.mass > 0) || fabsf(m - S.mass) > 0.03f * S.mass)
                logv(1, "mass %.1f t (trailer %s, cargo %.1f t)", m / 1000, T.trailer ? "yes" : "no", T.cargo_mass / 1000);
            S.mass = m;
        } else if (!(S.mass > 0)) S.mass = 20000;
        S.mass_at = now;
    }

    // Mode from the game's adaptive-shift keybind, once a second. The game
    // ignores this cvar while cruise control is on (game_actor+0x199); we don't.
    static double mode_at = -1e9;
    if (now - mode_at > 1 || now < mode_at) {
        mode_at = now;
        mode_update();
    }

    float gd = game_grade_deg(self);
    S.grade_deg = gd == gd ? gd : 0;
    if (now - S.grade_at >= 1 || now < S.grade_at) {
        S.grade_at = now;
        memmove(S.grade_hist + 1, S.grade_hist, sizeof S.grade_hist - sizeof S.grade_hist[0]);
        S.grade_hist[0] = grade_pct();
    }
    if (!(S.rpm_k > 0.5f)) S.rpm_k = 1;

    float v = fabsf(T.speed), kmh = v * 3.6f;
    int was = S.ours;
    if (T.parking_brake || kmh < p_off_kmh) S.ours = 0;
    // By rpm: the engine has reached takeover_rpm with the clutch closed in a gear. Also above 20 km/h whatever the rpm: after a
    // pause, a load or a teleport the truck may already be cruising at 900 rpm in a high gear and never reach the rpm.
    else if (p_on_rpm > 0 ? ((T.rpm >= p_on_rpm && T.gear > 0 && clutch_seen() < 0.1f) || kmh >= fmaxf(p_on_kmh, 20.0f)) : kmh >= p_on_kmh) S.ours = 1;
    if (S.ours && !was) { S.ours_at = now; S.ours_launch = kmh < 25; }

    // Learn the rpm correction while the clutch is closed and the speed steady.
    int g = gb_gear(self);
    if (g > 0 && v > 5 && clutch_seen() < 0.02f && T.rpm > 700) {
        float raw = rpm_at(g, v) / S.rpm_k;
        if (raw > 300) {
            float kr = T.rpm / raw;
            float k = clampf(kr, 0.7f, 1.4f);
            S.rpm_k += 0.02f * (k - S.rpm_k);
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
    S.cc_thr += (float)(dt / (cc_tau + dt)) * (eff - S.cc_thr);
    if (!cc) S.cc_thr = eff;
    S.thr = cc ? fmaxf(pedal, S.cc_thr) : fmaxf(pedal, eff);

    // Demand rises fast (0.3 s) and falls slowly (2 s). Load itself does not
    // move the shift points: the force checks handle it.
    float tau = S.thr > S.demand ? 0.3f : 2.0f;
    S.demand += (float)(dt / (tau + dt)) * (S.thr - S.demand);

    // Tractive force asked for (clutch closed only): up in 0.3 s, down in 3 s.
    if (g > 0 && clutch_seen() < 0.1f && S.eng_ok) {
        float f = force_at(g, v, S.thr);
        float tf = f > S.f_need ? 0.3f : 3.0f;
        S.f_need += (float)(dt / (tf + dt)) * (f - S.f_need);
    }

    // Extra resistance the model misses (tires, wind, a wrong mass): force the
    // engine gives minus modeled resistance minus mass x acceleration, while
    // pulling steadily with the clutch closed. Slow (20 s), limited to +-2 % grade.
    if (g > 0 && clutch_seen() < 0.02f && S.eng_ok && v > 6 && eff > 0.1f && T.in_brake < 0.02f && !T.motor_brake &&
        !T.retarder && S.eb_level < 0.01f && now - S.shift_at > 2) {
        float r = force_at(g, v, eff) - (resist(v) - S.f_bias) - S.mass * T.accel;
        float lim = S.mass * 9.81f * 0.02f;
        S.f_bias = clampf(S.f_bias + (float)(dt / (20 + dt)) * (r - S.f_bias), -lim, lim);
    }

    if (T.in_thr > 0.02f) S.pedal_on_at = now;

    // Kickdown: the pedal pressed past kickdown_pedal quickly. Armed for 3 s
    // (cleared by the kickdown shift itself or by lifting below 0.8).
    double pdt = now - S.pedal_at;
    if (pdt > 0.05) {
        if (pdt < 0.5) {
            float rate = (float)((pedal - S.pedal_prev) / pdt);
            if (pedal >= p_kick_pedal && S.pedal_prev < p_kick_pedal && rate >= p_kick_rate) S.kick_until = now + 3;
        }
        S.pedal_prev = pedal;
        S.pedal_at = now;
    }
    if (pedal < 0.8f) S.kick_until = 0;
    if (S.kick_until > now) S.demand = 1;

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
        if (r > S.eng.rpm_limit * 0.9f || r < base_dn(g) + p_hyst) continue;
        float err = fabsf(r - target);
        if (err < best_err) { best_err = err; best = g; }
    }
    return best ? best : n_gears();
}

// ---------------------------------------------------------------- eco-roll

// Back into drive after a roll: a gear for what is asked now.
static int roll_exit_gear(float v, int ebrake, float brake)
{
    float target = ebrake ? S.eng.range[R_BRAKE][0] + 100
                   : brake > 0.02f ? eco_up(1) - 200
                   : lerpf(cruise_floor(n_gears()) + 100, pw_mid(), pw_k());
    return pick_for_speed(v, target);
}

// Neutral on gentle stretches with the pedal released (deliberately: 1.5 s),
// Eco and Normal only, never with the brake, engine brake or retarder. Ends
// with real pedal (> 10 %), any brake, or when the speed leaves its band:
// without cruise the start speed +6/-12 km/h, with cruise the set speed +3/-5.
// The route (if any) vetoes rolling into a steep descent. With cruise only
// when the Cruise module drives the throttle (else the game's cruise would rev
// the engine in neutral); the cruise then gives no throttle while rolling.
static int roll_wanted(float v, float f_res, int ebrake, float brake, double pedal_free_s)
{
    if (p_roll <= 0 || (S.mode != MODE_ECO && S.mode != MODE_NORMAL && S.mode != MODE_POWER) || !S.ours) return 0;
    float kmh = v * 3.6f, set = T.cruise * 3.6f;
    int cc = T.cruise > 0.5f;
    if ((cc && !g_cruise_smooth) || brake > 0.02f || ebrake) return 0;
    if (S.rolling ? T.in_thr > 0.1f : (T.in_thr > 0.02f || (!cc && T.eff_thr > 0.02f) || pedal_free_s < 1.5)) return 0;
    // Damien: at most 1-2 km/h under the set speed, then back on the gas; leave a little early (re-engaging
    // takes ~1 s) using the current deceleration
    float kmh_soon = kmh + fminf(0, T.accel) * 3.6f * 1.0f;
    // how far under the set speed it may roll: Eco most, Power least (Damien 11-10: between 1 and 2 km/h)
    float under = S.mode == MODE_ECO ? p_roll_under_eco : S.mode == MODE_POWER ? p_roll_under_pw : p_roll_under;
    if (cc && (S.rolling ? (kmh > set + 3 || kmh_soon < set - under) : (kmh > set + 2 || kmh < set - 0.5f))) return 0;
    float a_n = -f_res / S.mass;  // acceleration in neutral
    int rk = p_lookahead > 0 && route_ok();
    if (rk) {
        float steep = route_min_grade(0, fmaxf(200, v * 15));
        if (steep == steep && steep < -3.5f) return 0;
    }
    if (!S.rolling) {
        if (kmh < 50 || sim_s() - S.roll_at < 10 || sim_s() - S.shift_at < 1) return 0;
        // Only when the speed would stay in the band for ~20 s (route: the
        // grade of the next 20 s; else the live grade).
        float a_pred = a_n;
        if (rk) {
            float g = route_grade(0, v * 20);
            if (g == g) a_pred = -(resist_grade(v, g)) / S.mass;
        }
        // at most ~3 km/h lost in 20 s (Damien 11-10: it rolled at 90 on the flat down to 85, pointless;
        // -0.15 allowed a flat road at speed): in practice a slight descent
        // with cruise: only a roll that lasts >= 10 s before it reaches the band's bottom (DAF/Scania; Power's 0.5 km/h
        // band made 0.5 s rolls that hunted, sim R5b)
        // (the worse of now and the route's 20 s mean: sim R5b started at -0.03 predicted but -0.14 now and rolled 1.5 s)
        float a_chk = fminf(a_pred, a_n);
        if (cc && a_chk < 0 && (kmh - (set - under)) / 3.6f / -a_chk < 10) return 0;
        if (a_pred > 0.075f || a_pred < -p_roll_dec) {
            // over a top: a short climb left and a gentle way down after it
            float crest = rk ? route_crest_m(v * 5) : -1;
            if (!(crest >= 0 && a_n > -0.5f && route_min_grade(crest, crest + v * 10) >= -3.5f)) return 0;
            // with cruise it has to reach the top before it drops out of the band (a narrow band ended rolls after 1.5 s)
            if (cc && a_n < 0 && (kmh - (set - under)) / 3.6f / -a_n < crest / fmaxf(v, 1.0f) + 1) return 0;
        }
        return 1;
    }
    return kmh <= S.roll_kmh + 6 && kmh >= S.roll_kmh - 12 && kmh >= 45 && grade_pct() > -4;
}

// M: the gear stays where the driver's shift hints put it; TrueShift only helps.
static int decide_manual(int cur, float rpm, float v, double since, const char **why)
{
    int n = n_gears(), ebrake = T.motor_brake || T.retarder > 0 || S.eb_level > 0.01f;
    double now = sim_s();
    float brake = fmaxf(T.in_brake, T.eff_brake);
    if (rpm > S.eng.rpm_limit * (ebrake ? 0.99f : 0.96f) && cur < n) { *why = "M: rev limit"; return cur + 1; }
    if (rpm < idle_rpm() + 120 && cur > 1) { *why = "M: stall"; return pick_down(cur, v, eco_dn(1) + 300); }
    if (p_m_manual > 0) { *why = "M"; return cur; }
    if (p_m_assist > 0 && S.kick_until > now && rpm < S.eng.range[R_POWER][0] + 100 && cur > 1 && since > 0.3) {
        int l = pick_down(cur, v, pw_mid() + 100);
        if (l < cur) { *why = "M: kickdown"; return l; }
    }
    if (brake > 0.05f && rpm < idle_rpm() + 250 && cur > 1 && since > 0.5) {
        int l = pick_down(cur, v, eco_up(1) - 200);
        if (l < cur) { *why = "M: stopping"; return l; }
    }
    float fmax = force_at(cur, v, 1.0f);
    if (p_m_assist > 0 && S.thr > 0.8f && rpm < p_m_lug && cur > 1 && since > 2 &&
        (T.accel < 0.05f || fmax - resist(v) < S.mass * 0.03f)) {
        *why = "M: lugging";
        return cur - 1;
    }
    *why = "M";
    return cur;
}

// Returns the gear TrueShift wants; *why says why. 0 = leave it to the game.
// PCC-like top (Damien 11-10): a top within reach that this gear gets over at full throttle while losing at most
// crest_band_kmh below the reference speed (the set speed with cruise, else the speed when the top came into view)
// and without leaving the torque plateau: no downshift for it. Energy over the climb: v_end^2 = v^2 + 2 d (F - R) / m.
// The band under the set speed (m/s): full at 40 t, 60 % solo, 60 % in Power.
static float crest_band()
{
    float k = fminf(1.0f, fmaxf(0.0f, (S.mass - 12000) / 28000));
    return p_crest_band > 0 ? p_crest_band * (0.6f + 0.4f * k) * (S.mode == MODE_POWER ? 0.6f : 1.0f) / 3.6f : 0;
}
// Lowest rpm the band may pull the gear down to: just under the torque plateau.
static float band_rpm() { return fmaxf(S.plateau - 150, idle_rpm() + 200); }

static int top_reachable(int cur, float v, float *top_m, float *floor_kmh)
{
    *top_m = -1;
    float c = p_crest_band > 0 && p_lookahead > 0 && route_ok() && S.mass > 500 ? route_crest_m(fmaxf(400, v * 25)) : -1;
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
    return v * v + 2 * c * (f - r) / S.mass >= v_floor * v_floor;
}

// PCC momentum (Damien 11-10: only what is needed; solo differs from loaded): the extra speed over the set speed at
// the foot of the next climb on the route so that the truck gets over its top without dropping more than 1 km/h under
// the set speed, at full throttle in the highest gear that is still on the plateau at the set speed. 0 = no climb ahead
// or it holds the speed anyway; capped at 8 (cruise caps at band_up_kmh); NAN = unknown (no route, no cruise, no engine).
static float climb_need_kmh()
{
    float set = T.cruise;
    if (!(set > 5) || !route_ok() || !S.eng_ok || S.mass < 500 || p_lookahead <= 0) return NAN;
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
            float a = (force_at(g, v, 1.0f) - resist_grade(v, gr)) / S.mass;
            v = sqrtf(fmaxf(1.0f, v * v + 2 * 25 * a));
            if (v > top) v = top;  // the cruise does not run away above the speed it started with
            if (v < set - 1 / 3.6f || rpm_at(g, v) < band_rpm()) { ok = 0; break; }
        }
        if (ok) return dv;
    }
    return 8;
}

static int decide_gear(void *self, const char **why)
{
    *why = "";
    int cur = gb_gear(self), n = n_gears();
    float v = fabsf(T.speed);
    double now = sim_s();
    if (!S.eng_ok || n < 2 || cur < 0 || cur > n || wheel_radius() <= 0.1f) { *why = "no data"; return 0; }

    float f_res = resist(v);
    float brake = fmaxf(T.in_brake, T.eff_brake);
    int ebrake = T.motor_brake || T.retarder > 0 || S.eb_level > 0.01f;
    int roll = roll_wanted(v, f_res, ebrake, brake, now - S.pedal_on_at);
    if (S.rolling) {
        if (roll) { *why = "eco-roll"; return ROLL; }
        S.rolling = 0;
        S.roll_at = now;
        if (cur == 0) {
            *why = "roll end";
            S.roll_exit = roll_exit_gear(v, ebrake, brake);
            S.roll_exit_at = now;
            return S.roll_exit;
        }
    }
    if (cur < 1) { *why = "neutral"; return 0; }
    if (!S.ours) { *why = "low speed"; return 0; }

    float rpm = rpm_at(cur, v);
    float lim = S.eng.rpm_limit;
    float thr = S.thr;
    double since = now - S.shift_at;
    int kick = S.kick_until > now;
    S.up_pt = up_pt(cur);
    S.dn_pt = dn_pt(cur);
    S.floor_rpm = 0;

    if (S.mode == MODE_M) return decide_manual(cur, rpm, v, since, why);

    // Protections, also while a hint is held. Far above the limiter (a wrong
    // gear after neutral): straight to a gear that fits the speed.
    if (rpm > lim * 1.05f) { *why = "wrong gear"; return pick_for_speed(v, pw_mid()); }
    if (rpm > lim * (ebrake ? 0.99f : 0.96f) && cur < n) {
        int h = ebrake || S.thr < 0.75f ? cur + 1 : skip_target(cur, v, f_res, sinf(S.grade_deg * 0.0174533f));
        *why = h > cur + 1 ? "rev limit skip" : "rev limit";
        S.hold_until = 0;  // a held hint gear ends here, the next gear is TrueShift's again (friend 11-10: 2-3-4-5-6 all at the limiter)
        return h;
    }
    if (rpm < idle_rpm() + 120 && cur > 1) { *why = "stall"; return pick_down(cur, v, eco_dn(1) + 300); }

    // not right after a launch shift (ts4o: 1->3 by the game at 10 km/h with the pedal floored, then 3->2->3)
    if (kick && rpm < S.eng.range[R_POWER][0] + 100 && cur > 1 && since > 2.0 && clutch_seen() < 0.1f && v * 3.6f > 15) {
        int l = pick_down(cur, v, pw_mid() + (S.wet ? lerpf(-150, 100, clampf((cur - 4) / 4.0f, 0, 1)) : 100));
        // never into a gear that is at its upshift point a moment later (ts3a: 3->2->3 at 13 km/h)
        while (l < cur && rpm_at(l, v) > S.eng.range[R_POWER][1] - 100) l++;
        if (l < cur) { *why = "kickdown"; return l; }
    }

    // A hint gear is held, but not up to the limiter: pulling past the top of the power range hands it back
    // (friend 11-10: hint down to 2 at 8 km/h, then 2->3->4->5->6 each at ~1900 rpm with 30% throttle).
    if (S.hold_until > now) {
        if (!(rpm > S.eng.range[R_POWER][1] && S.thr > 0.05f)) { *why = "hint hold"; return cur; }
        S.hold_until = 0;
        logv(1, "hint hold ended at %.0f rpm (top of the power range)", rpm);
    }

    if (roll && since > 1) {
        S.rolling = 1;
        g_rolling = 1;  // right away: Cruise keeps the game from canceling cruise when the box goes to neutral
        S.roll_at = now;
        S.roll_kmh = v * 3.6f;
        *why = "eco-roll";
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
    int junction_near = jn >= 0 && jn < fmaxf(150, v * 8) && v * 3.6f > 25 && ((vb == vb && vb < v - 2.5f) || thr < 0.05f);
    // Slowing for a junction / roundabout / bend on the route: braking
    // downshifts come sooner, so the gear fits pulling out of it (Damien 11-10).
    float vs = rk && jn >= 0 && jn < fmaxf(150, v * 10) ? route_curve_speed(2.5f, 0.3f) : NAN;  // gentle 0.3 m/s2: will we have to slow down?
    int slow_ahead = vs == vs && vs < v - 2.5f;
    float top_m, top_floor = 0;
    int top_ok = top_reachable(cur, v, &top_m, &top_floor);
    if (top_ok) { S.top_at = now; S.top_floor = top_floor / 3.6f; }
    // ... and over the top: the profile no longer sees the last meters of the climb, the filtered pull lags behind
    int top_over = !top_ok && now - S.top_at < 4 && now >= S.top_at && v > S.top_floor && !(rk && route_max_grade(0, v * 4) > 1.0f);

    // Engine brake / retarder: keep the rpm up, more for a stronger stage; the
    // full brake range when the game's "downshift with engine brake" is on.
    if ((T.motor_brake || S.eb_level > 0.01f) && thr < 0.05f) {
        float lo = S.eng.range[R_BRAKE][0], hi = S.eng.range[R_BRAKE][1];
        int full = p_eb_down > 0 || (p_eb_down < 0 && S.eb_full);
        float keep = full ? lo : lerpf(fminf(p_eb_light, lo), lo, S.eb_level);
        if (rpm < keep - 50 && cur > 1 && since > p_min_gap) {
            int l = pick_down(cur, v, full ? 0.5f * (lo + hi) : fminf(keep + 150, hi - 100));
            // two at a time when the second still fits under the brake range: every downshift costs a throttle blip
            // from the game (Damien 11-10: "gas and engine brake at once", 9->8->...->2 one by one)
            if (p_eb_skip > 0 && l == cur - 1 && cur > 2) {
                float r2 = rpm_at(cur - 2, v);
                if (r2 < hi - 100 && r2 < S.eng.rpm_limit * 0.93f) l = cur - 2;
            }
            if (l < cur && rpm_at(l, v) < hi) { *why = full ? "engine brake" : "engine brake (level)"; return l; }
        }
        *why = "engine brake hold";
        return cur;
    }

    if (brake > 0.05f) {
        // stay above the game's neutral threshold, which rises with brake^2
        float b = fminf(brake, 1.0f), opens = 0.8f * idle_rpm() + b * b * idle_rpm();
        float floor = fmaxf(fmaxf(idle_rpm() + 250, eco_dn(cur) - 100), opens + 250);
        if (slow_ahead) floor = fmaxf(floor, S.plateau + 200);
        if (rpm < floor && cur > 1 && since > 0.5) {
            // land high, so that fewer downshifts are needed on the way down
            int l = pick_down(cur, v, eco_up(1) - 200);
            if (l < cur) { *why = slow_ahead ? "braking (junction ahead)" : "braking"; return l; }
        }
        *why = "braking hold";
        return cur;
    }

    if (since < p_min_gap) { *why = "min gap"; return cur; }

    // The last upshift has no pull (strain built up): back down, and learn.
    if (S.up_gear == cur && S.strain > 1.0f && cur > 1) { *why = "no pull"; return cur - 1; }

    // A climb starts within ~3 s that this gear won't hold: one down now, at
    // the foot, instead of lugging into it.
    if (rk && thr >= 0.5f && cur > 1 && since > 2 && !top_near && !top_ok) {
        float g_now = grade_pct(), g_climb = route_grade(v, v + fmaxf(200, v * 10));
        float g_start = route_max_grade(0, v * 3);
        if (g_climb == g_climb && g_start == g_start && g_climb > g_now + 1.5f && g_start > g_now + 1.0f &&
            force_at(cur, v * 0.95f, 1.0f) < resist_grade(v * 0.95f, g_climb) && rpm_at(cur - 1, v) < lim * 0.9f) {
            // the highest gear that holds the climb (else the one nearest the power band)
            int l = cur - 1;
            while (l > 1 && force_at(l, v * 0.95f, 0.95f) < resist_grade(v * 0.95f, g_climb) &&
                   rpm_at(l - 1, v) < lim * 0.9f)
                l--;
            *why = "climb ahead";
            return l;
        }
    }

    // Downshift: below the downshift point, or lugging (full pedal, losing
    // speed and no force left in this gear).
    float fmax = force_at(cur, v, 1.0f);
    int lug = thr > 0.8f && T.accel < 0 && fmax - f_res < S.mass * 0.03f && rpm < S.eng.range[R_POWER][0] + 100;
    // The demand-raised point only counts when this gear lacks pull.
    float dn_eff = (fmax - f_res) / S.mass < a_power() ? S.dn_pt : base_dn(cur);
    if (cur > 1 && (rpm < dn_eff || lug)) {
        int deep = rpm < dn_eff - 100 || rpm < idle_rpm() + 250;
        if (S.shift_dir > 0 && since < p_dwell_ud && !deep && !lug) { *why = "anti-hunt (down)"; return cur; }
        // Almost over the top: hold on, it gets easier in a moment.
        if (top_near && rpm > idle_rpm() + 300) { *why = "top ahead"; return cur; }
        // A top this gear still gets over within the band: let the speed sag instead (PCC).
        if (top_ok || top_over) {
            if (top_ok && !S.crest_said) {
                S.crest_said = 1;
                DEV_NOTE(pcc_top());
                logv(1, "top in %.0f m: holding %d at %.0f km/h (gets over above %.0f km/h)", top_m, cur, v * 3.6f, top_floor);
            }
            *why = top_ok ? "top reachable" : "over the top";
            return cur;
        }
        // In the cruise band (PCC): a few km/h under the set speed and this gear still pulls on this grade (not
        // just lugging along): let the speed come back slowly instead of a downshift for pull. Not with the driver's
        // own pedal (overtaking with cruise on wants the pull).
        if (T.cruise > 0.5f && !kick && T.in_thr < 0.1f && v >= T.cruise - crest_band() && rpm > band_rpm() && (fmax - f_res) / S.mass > 0.06f) {
            *why = "cruise band";
            return cur;
        }
        float target = lug ? pw_mid() : lerpf(0.5f * (eco_dn(cur - 1) + eco_up(cur - 1)) + 100, pw_mid(), pw_k());
        int l = pick_down(cur, v, target);
        if (l == cur) l = cur - 1;
        *why = lug ? "lugging" : (cur - l > 1 ? "down skip" : "down");
        return l;
    }

    // Brake air low (Damien: the tank ran empty twice): the compressor runs on the engine, and at ~900 rpm it barely
    // keeps up. Below air_charge_psi keep the rpm up: no upshift below air_charge_rpm, a downshift when well under it.
    if (p_air_psi > 0 && T.air_psi > 1 && T.air_psi < p_air_psi && thr > 0.02f) {
        static double said;
        if (sim_s() - said > 10) { said = sim_s(); logline("air %.0f psi: keeping the rpm up to charge it", T.air_psi); }
        if (rpm < p_air_rpm - 250 && cur > 1) {
            int l = pick_down(cur, v, p_air_rpm + 100);
            if (l < cur && rpm_at(l, v) < S.eng.rpm_limit * 0.9f) { *why = "air charge"; return l; }
        }
        if (cur < n && rpm_at(cur + 1, v) < p_air_rpm) { *why = "air charge"; return cur; }
    }

    if (junction_near) { *why = "junction ahead"; return cur; }

    int coast = thr < 0.05f;
    int skips = skip_allowed();
    float grade = sinf(S.grade_deg * 0.0174533f);
    // No skipping right after taking over from the game (it may have started
    // in a high gear) or close above the takeover speed.
    // The launch is the exception (Damien 12-10: 1 -> 2 -> 3 -> 4 -> 5 one by one between 12 and 20 km/h, "skip one, also in Eco"): from
    // the takeover speed the first gears may be skipped; the rpm and pull checks below still decide.
    // (while pulling away only 1 s after the takeover, else 3: the game starts a light truck in 4 and the next two shifts come within
    // seconds; after a pause or a teleport the gear may be wrong; the rpm and pull checks below still stop a skip that lands too low)
    if (!(cur <= 2 && v * 3.6f >= p_on_kmh) && (now - S.ours_at < (S.ours_launch ? 1.0 : 3.0) || v * 3.6f < p_on_kmh + 5)) skips = 0;
    float v2 = fmaxf(0.5f, v - f_res / S.mass * S.eng.shift_time);

    // Early upshift: light load, the new gear lands on the torque plateau and
    // gives the force asked for now with reserve to spare. Failed attempts
    // (had to come back down) block that situation for a while.
    // Without route data be careful: no early upshift when a climb starts
    // (grade up 0.5 % in 3 s), and more reserve unless downhill.
    int climbing = grade_pct() - S.grade_hist[3] > 0.5f;
    // Only while pulling steadily: not lifting off (throttle well below the
    // recent demand), not slowing down, not right after taking over.
    int steady = thr >= 0.15f && thr >= 0.6f * S.demand && T.accel > -0.15f && now - S.ours_at > 3 &&
                 v * 3.6f >= p_on_kmh + 5;
    // With the route: the grade of the next seconds counts, no extra caution,
    // and a little less reserve when nothing climbs ahead.
    float g_ahead = NAN;
    if (rk) {
        float a = route_grade(0, v * 12), b = route_max_grade(0, v * 5);
        if (a == a && b == b) g_ahead = fmaxf(a, b - 0.5f);
        climbing = 0;
    }
    if (steady && cur < n && S.demand < p_power_from && !(S.shift_dir < 0 && since < p_dwell_du) && !climbing) {
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
            if (h > cur + 1 && (grade > 0.02f || (force_at(h, v2, 0.85f) - f_res) / S.mass < 0)) break;
            best = h;
        }
        if (best > cur) { *why = best - cur > 1 ? "up early skip" : "up early"; return best; }
    }

    // Upshift on rpm: above the upshift point, the new gear must stay above its
    // own downshift point and still pull after the speed lost during the shift.
    // On a climb the grade that counts is the steepest of now, the last seconds (a peak that fades 0.15 %/s) and the route
    // ahead (Damien 11-10, ride 3 at 400 s: 8 -> 9 in a 4.1 % dip of a 5-6 % climb at 54 km/h, back to 8 after 1.6 s, lugging).
    // The speed lost in the shift is reckoned with that grade too, and a climb asks a real margin, not 0.02-0.05 m/s2.
    static double pk_at; static float pk = -99;
    float gp = grade_pct();
    pk = fmaxf(gp, pk - 0.15f * (float)clampf((float)(now - pk_at), 0, 60)); pk_at = now;
    float g_up = gp > 1.5f ? fmaxf(gp, pk) : gp;   // the peak only while still climbing (over the crest it is the route's job)
    if (g_ahead == g_ahead) g_up = fmaxf(g_up, g_ahead);
    float f_up = fmaxf(f_res, resist_grade(v, g_up));
    float v2u = fmaxf(0.5f, v - f_up / S.mass * S.eng.shift_time);
    if (cur < n && rpm > S.up_pt + (coast ? 200 : 0)) {
        if (S.shift_dir < 0 && since < p_dwell_du && rpm < S.up_pt + 100) { *why = "anti-hunt (up)"; return cur; }
        float a_req = lerpf(0.02f, 0.05f, S.demand);
        if (g_up > 2 && !coast) a_req = fmaxf(a_req, lerpf(0.05f, 0.12f, clampf((g_up - 2) / 3, 0, 1)));
        int best = cur, maxg = cur + 1 + skips;
        const char *block = "";
        for (int h = cur + 1; h <= n && h <= maxg; h++) {
            float rh = rpm_at(h, v2u);
            float a_h = (force_at(h, v2u, 0.95f) - f_up) / S.mass;
            if (rh < fmaxf((a_h < a_power() ? dn_pt(h) : base_dn(h)) + p_hyst, idle_rpm() + 350)) {
                block = "rpm too low after shift";
                break;
            }
            if (!coast && (force_at(h, v2u, 0.95f) - f_up) / S.mass < a_req) { block = "not enough pull"; break; }
            if (!coast && learned().res > 0 && rh < fmaxf(dn_pt(h) + p_hyst, idle_rpm() + 350) + learned().rpm &&
                rpm < S.up_pt + 150) { block = "no pull here before"; break; }
            if (h > cur + 1) {
                // At high demand only when the skipped-to gear still pulls hard (light truck).
                float a_skip = skip_accel() * (S.demand > 0.75f ? 2.0f : 1.0f);
                if (coast || grade > 0.02f || (force_at(h, v2u, 0.85f) - f_up) / S.mass < a_skip) break;
                // and keeps most of the pull of the next gear (a car must not jump 2 -> 5 at full throttle)
                if (S.demand > 0.75f &&
                    force_at(h, v2u, 1.0f) - f_up < 0.7f * (force_at(cur + 1, v2u, 1.0f) - f_up)) break;
            }
            best = h;
        }
        if (best > cur) { *why = best - cur > 1 ? "up skip" : "up"; return best; }
        if (now - S.blocked_log_at > 10) {
            S.blocked_log_at = now;
            logline("up blocked in %d: %s (%.0f km/h, %.0f rpm, demand %.2f, grade %.1f%%, %.1f t)", cur, block,
                    v * 3.6f, rpm, S.demand, grade_pct(), S.mass / 1000);
        }
        *why = "up blocked";
        return cur;
    }
    *why = "hold";
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
    if (S.cmd_why && !strcmp(S.cmd_why, "kickdown")) S.kick_until = 0;
    int hint = prev > 0 && g > 0 && g != S.commanded && S.ours;
    const char *why = hint ? "hint" : g == S.commanded ? S.cmd_why : "game";
    DEV_NOTE(shift(prev, g, why, hint, now));
    track_shifts(prev, g, hint, now);

    // Did the last upshift hold? A downshift out of it for lack of force (or a
    // hint down) within 25 s means it did not.
    if (S.up_gear && prev == S.up_gear && g < prev) {
        // A plain downshift only counts when power was asked for (not when
        // the driver or the cruise control lets the truck slow down).
        // A climb that started after the upshift is terrain, not a bad upshift.
        // (within 5 s it counts anyway: a climb that went on steepening right after it is what the upshift should have seen)
        int asked = S.thr >= 0.5f, same_road = grade_pct() - S.up_grade < 1.0f || now - S.up_at < 5;
        if (now - S.up_at < 25 && why && same_road &&
            (hint || !strcmp(why, "no pull") || !strcmp(why, "lugging") ||
             (asked && (!strcmp(why, "down") || !strcmp(why, "down skip")))))
            up_failed(why);
        else S.up_gear = 0;
    }
    if (!hint && why && g > prev && (!strncmp(why, "up", 2))) {
        S.up_gear = g;
        S.up_at = now;
        S.up_grade = grade_pct();
        S.up_kmh = v * 3.6f;
        float fmax = force_at(g, v, 1.0f);
        S.up_pred = fmax > 1 ? clampf(fmaxf(S.f_need, resist(v)) / fmax, 0, 1) : 1;
        S.strain = 0;
    }

    if (hint && p_hint_hold > 0 && S.mode != MODE_M) {
        S.hold_until = now + p_hint_hold;
        logv(1, "shift %d->%d by hint at %.0f km/h: hold %.0f s", prev, g, v * 3.6f, p_hint_hold);
    } else {
        logv(1, "shift %d->%d (%s) at %.0f km/h, %.0f rpm, pedal %.2f, demand %.2f, grade %.1f%%, %.1f t, %s", prev,
                g, why ? why : "?", v * 3.6f, T.rpm, T.in_thr, S.demand, grade_pct(), S.mass / 1000,
                mode_name[S.mode]);
    }
}

static float *stub_decide(void *self, float *out, unsigned gear)
{
    float *r = o_window(self, out, gear);
    if (!g_active) return r;
    refresh(self);
    note_gear_change(self);
    const char *why;
    int want = decide_gear(self, &why);
    S.why = why;
    S.want = want;
    S.target = gb_target(self);
    // Neutral goes in through the tail hook (the chooser path clamps to +-1).
    if (want > 0 && want != ROLL && want != gb_gear(self) && takeover()) gb_force(self);
    return r;
}

static int stub_choose(void *self)
{
    int stock = o_choose(self);
    if (!g_active) return stock;
    int ret = stock;
    const char *why = "game";
    if (takeover()) {
        const char *w;
        refresh(self);
        int want = decide_gear(self, &w);
        if (want == ROLL) { ret = gb_gear(self); why = w; }
        else if (want > 0) { ret = want; why = w; }
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
static float stop_threshold(float idle, float thr, float brake)
{
    float stock = 0.8f * idle;
    if (p_stop <= 0 || S.unsupported || manual_box() || thr > 0.01f || !(idle > 200 && idle < 2000)) return stock;
    // Crawl off (Damien): never creep on the clutch at idle; pedals free below 8 km/h opens it like a stop.
    // not with cruise on: in a jam the ACC rolls at walking pace, and neutral would cancel cruise (ts4p 534 s)
    if (brake < 0.05f && !(p_no_creep > 0 && !g_crawl_on && T.cruise < 0.5f && fabsf(T.speed) * 3.6f < 8)) return stock;
    if (fabsf(T.speed) * 3.6f > 15) return stock;
    return fmaxf(stock, idle + 150);
}

// Runs at the very end of every automatic update (also in neutral), right
// before the generic gearbox acts on the target gear: eco-roll goes in and
// out here (docs/lka-roll-re.md part B).
static void tail_pre(void *self)
{
    if (!g_active || !takeover() || self != S.self) return;
    int g = gb_gear(self), st = gb_state(self);
    // After a roll: our exit gear as the target; the generic gearbox then
    // parks in its gap state (clutch open, gear 0) until there is throttle
    // (ts3a-c, docs/neutral-exit-re.md § 1). There we do its own engage step.
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
        static double gap_since;
        int tgt = gb_target(self);
        if (st == 1 && g == 0 && tgt > 0 && !S.rolling && !g_crawl_on && fabsf(T.speed) * 3.6f > p_off_kmh) {
            if (!gap_since) gap_since = sim_s();
            // right after a pause the gap state is the pause's, not a shift's (pre.6 ride 609/1716 s: 1.5 s in neutral at 45-50 km/h)
            else if (sim_s() - gap_since > (sim_s() - g_reset_at < 5 ? 0.3 : 1.5) && gb_engage(self, tgt)) {
                logline("stuck in neutral for %.1f s at %.0f km/h: engaged gear %d", sim_s() - gap_since, fabsf(T.speed) * 3.6f, tgt);
                if (sim_s() - g_reset_at >= 5)  // right after a pause the gap state is normal: no incident
                    incident("neutral", "stuck in neutral for %.1f s at %.0f km/h, engaged gear %d", sim_s() - gap_since, fabsf(T.speed) * 3.6f, tgt);
                gap_since = 0;
            }
        } else gap_since = 0;
    }
    if (st != 0) return;
    if (S.rolling && g > 0) { gb_set_target(self, 0); S.commanded = 0; S.cmd_why = "eco-roll"; return; }
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
    S.cmd_why = "roll end";
}

static int stub_start(void *self)
{
    int stock = o_start(self);
    if (!g_active || stock <= 0) return stock;
    S.mass_at = -1e9;  // fresh mass for the start
    refresh(self);
    // Engaging while moving (after an eco-roll or neutral): never the start
    // gear, a gear that fits the speed.
    float v = fabsf(T.speed);
    if (takeover() && S.eng_ok && v * 3.6f > p_off_kmh && wheel_radius() > 0.1f) {
        S.rolling = 0;
        S.roll_at = sim_s();
        int g = pick_for_speed(v, lerpf(cruise_floor(n_gears()) + 100, pw_mid(), pw_k()));
        logv(1, "engage at %.0f km/h: gear %d (game %d)", v * 3.6f, g, stock);
        S.commanded = g;
        S.cmd_why = "engage";
        return g;
    }
    int pick = stock;
    float thr = clampf(T.in_thr, 0, 1);
    if (p_start_fix > 0 && takeover() && S.eng_ok && wheel_radius() > 0.1f && stock <= n_gears()) {
        float r = wheel_radius(), f_res = resist(0);
        float limit = p_slip_s * (thr >= p_kick_pedal ? 0.6f : 1.0f);
        pick = 1;
        // Above the game's pick: only by physics (lock speed and slip time at this mass, grade and pedal), never by a gear count,
        // because every gearbox numbers its gears differently.
        bool raise = p_launch_max > 0 && S.mode != MODE_POWER && S.mode != MODE_M;
        for (int g = raise ? n_gears() : stock; g >= 1; g--) {
            float ratio = T.ratio[g - 1] * T.diff;
            float v_lock = p_launch_rpm / (ratio * 9.5493f) * r;
            float a = (S.eng.torque * p_launch_torque * ratio * p_eta / r - f_res) / S.mass;
            if (g > stock && v_lock * 3.6f > p_launch_max) continue;
            if (a > 0 && v_lock / a <= limit) { pick = g; break; }
        }
    }
    // Wet road (wipers on): one gear higher, less torque at the wheels, as long
    // as the clutch would not slip much longer than allowed.
    if (S.wet && pick < stock + 1 && pick < n_gears() && wheel_radius() > 0.1f && S.eng_ok) {
        int g = pick + 1;
        float r = wheel_radius(), ratio = T.ratio[g - 1] * T.diff;
        float v_lock = p_launch_rpm / (ratio * 9.5493f) * r;
        float a = (S.eng.torque * p_launch_torque * ratio * p_eta / r - resist(0)) / S.mass;
        if (a > 0 && v_lock / a <= p_slip_s * 1.6f) pick = g;
    }
    static double logged_at;
    if (sim_s() - logged_at > 2) {
        logged_at = sim_s();
        logv(1, "start gear %d (game %d), %.1f t, grade %.1f%%, pedal %.2f%s", pick, stock, S.mass / 1000, grade_pct(),
                thr, S.wet ? ", wet" : "");
    }
    S.commanded = pick;
    S.cmd_why = pick == stock ? "start" : "start, lowered";
    return pick;
}

// ---------------------------------------------------------------- module

static char g_status[200] = "off";
static const char *ts_status() { return g_status; }

static void ts_start()
{
    if (p_enabled <= 0) { logline("TrueShift disabled in the ini"); snprintf(g_status, sizeof g_status, "off (ini)"); return; }
    o_window = (window_fn)G.window_fn;
    o_choose = (gear_fn)G.chooser_fn;
    o_start = (gear_fn)G.start_fn;
    int done = game_patch_call(G.decide_site, (void *)stub_decide), want = 1 + G.n_choose + G.n_start;
    for (int i = 0; i < G.n_choose; i++) done += game_patch_call(G.choose_sites[i], (void *)stub_choose);
    for (int i = 0; i < G.n_start; i++) done += game_patch_call(G.start_sites[i], (void *)stub_start);
    g_active = done == want;
    if (g_active && p_stop > 0) {
        if (G.stop_site && game_patch_stop(G.stop_site, stop_threshold)) logline("smooth stop hook in place");
        else { p_stop = 0; logline("smooth stop off: threshold code not found"); }
    }
    // The update's tail hook carries eco-roll AND the neutral watchdog (a shift interrupted by a pause leaves gear 0 with
    // the clutch open): both need the engage code, and the watchdog is wanted with eco_roll = 0 too.
    if (g_active) {
        int engage = game_engage_init(), tail = engage && G.tail_site && game_patch_tail(G.tail_site, (void *)tail_pre);
        if (!tail) {
            logline("%s: %s", p_roll > 0 ? "eco-roll and neutral watchdog off" : "neutral watchdog off",
                    engage ? "update tail not found" : "gearbox engage code not found");
            p_roll = 0;
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
                  "demand,up_pt,dn_pt,hold,kick,accel,rpm_k,clutch,mode,thr,f_need_kn,floor,ours,eb_level,f_bias_kn,strain,learn_rpm,roll,route,crest_m,junction_m,idle,wet\n", g_trace);
    }
}

// Dashboard: the mode letter, also stopped and in N (Damien 11-10 prefers a
// steady letter over "A" flashing in at every stop); A in reverse. Runs outside
// the gearbox update, so the gearbox comes from game_actor, not a cached pointer.
static void dash_letter()
{
    static double at;
    double now = wall_s();
    if (p_letter <= 0 || !g_active || S.unsupported || manual_box() || now - at < 0.25) return;
    at = now;
    // Only the gearbox the hooks have seen (never a stale or menu object).
    void *gb = game_gearbox();
    if (!gb || gb != S.self) return;
    mode_update();  // 4x/s, also while stopped
    char c = T.gear < 0 ? 'A' : mode_letter[S.mode];  // reverse: plain A (Damien 11-10)
    if (g_dash_status) c = g_dash_status;                            // B hill-start brake, K creep
    if (g_dash_flash && now < g_dash_flash_until) c = g_dash_flash;  // e.g. the new ACC distance for 1.5 s
    int r = game_letter(gb, c);
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
    float lim = S.eng.rpm_limit > 500 ? S.eng.rpm_limit : T.rpm_limit;
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

static void ts_frame()
{
    g_rolling = S.rolling || S.roll_exit > 0;  // neutral by our choice, also until the exit gear is in
    {
        static const int to_inc[] = { 2, 1, 3, 4 };  // Normal, Eco, Power, M -> incident file numbering
        int on = g_active && !S.unsupported && !manual_box();
        g_inc.mode = on ? to_inc[S.mode] : 0;
        g_inc.target = on && S.commanded > 0 ? S.commanded : 0;
        g_inc.grade_pct = on ? grade_pct() : NAN;
        g_inc.flags = (g_inc.flags & ~(INC_ROLL | INC_HOLD)) | (g_rolling ? INC_ROLL : 0) | (S.hold_until > sim_s() ? INC_HOLD : 0);
        if (on) watch_engine();
    }
    DEV_NOTE(gearbox(g_active && !S.unsupported && !manual_box() ? mode_letter[S.mode] : 0, S.mass, grade_pct()));
    {
        static double at;
        if (wall_s() - at > 0.5 || wall_s() < at) { at = wall_s(); g_pcc_need_kmh = g_active && !S.unsupported ? climb_need_kmh() : NAN; }
    }
    dash_letter();
    int wet = p_wet > 0 && T.wipers;
    if (wet != S.wet) { S.wet = wet; logv(1, "wipers %s: %s road behavior", wet ? "on" : "off", wet ? "wet" : "dry"); }
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
            "%c,%.2f,%.1f,%.0f,%d,%.2f,%.2f,%.2f,%.0f,%d,%d,%.0f,%.0f,%.0f,%d\n",
            now, fabsf(T.speed) * 3.6f, T.rpm, T.gear, S.target, S.want, S.why ? S.why : "", T.in_thr,
            T.eff_thr, T.in_brake, T.motor_brake, T.retarder, T.cruise * 3.6f, grade_pct(), S.mass / 1000, S.demand,
            S.up_pt, S.dn_pt, S.hold_until > sim, S.kick_until > sim, T.accel, S.rpm_k, T.eff_clutch,
            mode_letter[S.mode], S.thr, S.f_need / 1000, S.floor_rpm, S.ours, S.eb_level, S.f_bias / 1000, S.strain, learned().rpm, S.rolling, route_ok(),
            route_crest_m(2000), route_junction_m(), idle_rpm(), S.wet);
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
