// CarShift: the automatic of a car or pickup (a light vehicle, core/vehicle.h), a module of its own next to TrueShift (modules/trueshift.cpp,
// the trucks' AMT). The vehicle class picks the module set (core/vclass.h): this module runs for cars only, TrueShift for trucks only.
//
// A petrol car cruises and lugs happily low; the truck's shift window is anchored to the torque plateau, which for a car engine sits
// far up (a V8's plateau is ~3000 rpm, yet it pulls fine at 1200). So the calm shift points sit near the lug limit (car_lug: from the torque curve and the load)
// and rise toward the power range as the driver drives harder, measured by his aggression (S.agg: the pedal's level and how fast it
// moves), not by the throttle of the moment. A converter automatic is the usual box: the converter's slip, its lockup and the idle
// in gear are part of it (core/vehicle.cpp models the box, this module learns it from the drive).
//
// How it hooks in: the game's hook sites are shared with TrueShift (core/shifthook.cpp), which hands the window call of the
// automatic's update, the gear chooser, the start-gear function, the update's tail and the stop threshold to the brain of the class.
//  - window: our decision runs right before the game decides whether to look for another gear; if we want another gear we set the
//    update's force flag, so the game calls its chooser now.
//  - choose: the game's chooser runs (it clears the force flag) and we return our gear instead.
//  - start: we keep the game's pick and only lower it when the launch would take too long.
// Below the takeover speed, in reverse and in neutral the game decides as before.
//
// Modes follow the game's adaptive-shift keybind (cvar g_adaptive_shift): Eco (10), Comfort (3), Sport (1.67) and the game's "off" slot
// (0) = M, assisted manual. The dashboard's "A" becomes C/E/S/M.
#include "../core/suite.h"
#include "../core/game.h"
#include "../core/sense.h"
#include "../core/act.h"
#include "../core/vehicle.h"
#include "../core/speedtarget.h"
#include "../core/vclass.h"
#include "../core/shiftmodel.h"
#include "../core/shiftcommon.h"
#include "../core/shifthook.h"
#include "../core/gearreq.h"
#include "../core/gearscore.h"
#include "../core/rpmspan.h"
#include "../core/wish.h"
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------- settings

static float p_enabled, p_hint_hold, p_idle, p_kick_pedal, p_kick_rate, p_letter, p_max_skip, p_no_creep, p_off_kmh, p_on_kmh, p_on_rpm, p_roll, p_stop, p_tones, p_trace;

// What a car driver can tune is the table below. The rest are fixed values of the car automatic (the numbers the ini keys of the truck's TrueShift
// start with), so a shipped ini stays short.
static const float p_takeover = 1;
static const float p_eco_ceil = 200;
static const float p_mode_char = 1;
static const float p_min_gap = 0.5f;
static const float p_dwell_ud = 3.0f;
static const float p_dwell_du = 2.0f;
static const float p_skip_acc = 0.15f;
static const float p_pw_calm_thr = 0.6f;
static const float p_load_from = 0.75f;
static const float p_kick_dwell = 5;
static const float p_hyst = 50;
static const float p_power_from = 0.65f;
static const float p_power_full = 1.0f;
static const float p_reserve = 1.3f;
static const float p_eco_rpm = -75;
static const float p_power_rpm = 150;
static const float p_eco_reserve = 1.2f;
static const float p_power_reserve = 1.5f;
static const float p_cc_damp = 4;
static const float p_start_fix = 1;
static const float p_slip_s = 1.0f;
static const float p_launch_torque = 0.6f;
static const float p_tc = 0;
static const float p_tc_stall = 0;
static const float p_tc_lock = 0;

static param g_params[] = {
    { "enabled", &p_enabled, 1, "1 = CarShift on, 0 = the game's own automatic in a car or pickup" },
    { "takeover_kmh", &p_on_kmh, 7, "CarShift takes over from the game's automatic above this speed" },
    { "takeover_rpm", &p_on_rpm, 1000, "CarShift takes over as soon as the engine has reached this rpm with the clutch closed in any gear (the clutch is locked then, so the rpm is real); 0 = by speed only (takeover_kmh)" },
    { "handback_kmh", &p_off_kmh, 4, "and hands back below this speed (starting, stopping, maneuvering)" },
    { "hint_hold_s", &p_hint_hold, 10, "seconds a gear chosen with the shift-hint buttons is held (0 = off)" },
    { "max_skip", &p_max_skip, 2, "most gears an upshift may skip (0 = never skip)" },
    { "kickdown_pedal", &p_kick_pedal, 0.92f, "pedal position for kickdown" },
    { "kickdown_rate", &p_kick_rate, 1.5f, "pedal speed (per second) that makes a full press a kickdown" },
    { "dash_letter", &p_letter, 1, "1 = the dashboard shows the mode (C, E, S, M) instead of A" },
    { "idle_rpm", &p_idle, 550, "engine idle rpm to start with (learned while standing still)" },
    { "tones", &p_tones, 1, "1 = a short tone when the mode changes (Eco low, Comfort middle, Sport rising, M two short)" },
    { "no_creep", &p_no_creep, 1, "1 = with crawl switched off the car never creeps on the clutch at idle: pedals free below 8 km/h opens the clutch" },
    { "eco_roll", &p_roll, 1, "1 = roll in neutral with the pedal released on gentle stretches (Eco and Comfort; not in Sport or M; never with the brake or the cruise on)" },
    { "smooth_stop", &p_stop, 1, "1 = when braking to a stop the clutch opens around idle +150 rpm (more when braking harder) instead of pushing to ~2 km/h" },
    { "trace", &p_trace, 1, "1 = write carshift.csv (10 per second) for analysis" },
};

// ---------------------------------------------------------------- state

// The car names: Comfort / Eco / Sport, letters C / E / S (M stays M). The mode index is the game's: 0 Normal (Comfort), 1 Eco, 2 Power (Sport), 3 M
static const char *const mode_name[] = { "Comfort", "Eco", "Sport", "M" };
static const char mode_letter[] = { 'C', 'E', 'S', 'M' };
static const char *mode_label(int m) { return mode_name[m]; }
static char mode_dash(int m) { return mode_letter[m]; }

enum { ROLL = 100 };        // decide_gear: "put it in neutral" (eco-roll)
static int g_active;    // CarShift has the gearbox: on in the ini and the shared hooks (core/shifthook.cpp) are in
static struct : shift_common_state {
    void *self;
    float agg;              // car feel: how hard this driver drives (0 calm .. 1 hard), from the pedal's level and rate
    float agg_open;         // car feel, Eco: how far a floored pedal has lifted Eco's cap on the aggression (0 capped .. 1 free); rises in 0.15 s, falls in 1.5 s
    float agg_prev;         // the pedal at the last aggression update
    double agg_at;          // and when
    float up_pt, dn_pt, floor_rpm;
    // the dashboard's view (ts_view, the dev display): read-only, nothing here steers
} S;

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
    S.rolling = 0;
    S.roll_exit = 0;
    g_rolling = 0;
    logline("CarShift: this vehicle is left to the game's own automatic (%s)", why);
}
// Everything we change in the game's decisions goes through this.
static int takeover() { return p_takeover > 0 && !S.unsupported && !manual_box(); }

static FILE *g_trace;
static double g_trace_at;

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

// ---------------------------------------------------------------- car feel
// (see the top of the file: the calm shift points sit near the lug limit and rise with the driver's aggression, S.agg)
// (the lug limit, car_lug(g), is defined after a_power() below: it reads the vehicle model and the mode's pull)
static float car_lug(int g, float v = -1, bool load = true);
// The driver's aggression after the mode's policy. Eco caps it low (the lowest revs, whatever the pedal) for a partial
// pedal, but a floored pedal is a kickdown in any mode and lifts the cap (ride 31-10, the Crown Victoria in Eco: the floor went 1->2->3->4 in
// 0.8 s at 34-37 km/h, because the calm shift points of a capped 0.3 were under the converter's stall speed; the cap opens with the pedal and
// closes slowly, so a lift is no snap to the calm points either);
// Normal lets it run free; Power (Sport) lets it run free too, and only lifts the calm floor a little (car_pw_k), so calm Sport
// shifts almost like calm Normal and hard Sport is eager without hanging at the top.
static float car_agg()
{
    if (S.mode == MODE_ECO) return fminf(S.agg, 0.30f + 0.70f * S.agg_open);
    return S.agg;
}
// How far the shift window has moved from the calm point toward the power range, per mode: the aggression squared (calm stays calm,
// only real hard driving reaches the top). Sport lifts the calm floor a little (a floor of ~6 % of the way up) so it responds more
// eagerly, but hard Sport reaches no higher than hard Normal.
static float car_pw_k()
{
    float a = car_agg();
    if (S.mode == MODE_POWER) a = fmaxf(a, 0.25f);
    return a * a;
}
// The calm upshift point of gear g (defined after base_dn): the rpm where the next gear lands just above its own downshift point, so
// a calm drive changes up as soon as the next gear can pull. Never above the game's own eco point.
static float car_eco_up(int g);
// How far a kickdown may drop a light vehicle (0 = uncapped, the truck rule): a stomp (high aggression) drops deep, a modest press
// while cruising only a gear or two.
static int car_kick_drop();

// The vehicle model's forces (core/vehicle.cpp), the short names the decisions below are written in: the engine rpm in gear g
// at speed v (m/s), tractive force (N) in g at v with a share of the torque, the resistance (N) on the grade of now or
// on another grade (%). rpm_at is the ENGINE's rpm (the shift points, floors and lug checks compare it): on a torque converter
// it is the stall speed while the converter slips and the driveline's once it is coupled; on a locked box the two are the same,
// so nothing changes there.
static float rpm_at(int g, float v) { return vehicle_engine_rpm_at(g, v); }
// The DRIVELINE's rpm in gear g at v (m/s), the gear's effective ratio calibrated on a converter (core/vehicle.cpp): what the gear turns at once the
// converter is out of the way. The landing checks of an upshift (a floor against lugging, the choice floor, the early upshift's floor) read
// this: the engine's rpm of rpm_at is held at a throttle-scaled stall speed over it while the converter slips, which let every floor through
//. On every other box it is rpm_at.
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
// demand while the speed changes (the force the same share asks falls as the speed rises) and in every gear (the shaped share, before phi). This is the "asked for"
// demand the flip lock and the hunt metric compare across a shift.
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
static float eco_up(int g) { return car_eco_up(g); }   // a car's calm upshift point is near the lug limit, not the plateau
// How far the shift window has moved from the calm point to the power range: the driver's aggression (a held high pedal, a stomp), squared
// so calm stays calm and only real hard driving reaches the top (the BeamNG idea).
static float pw_k() { return car_pw_k(); }
// The power range the shift logic aims at: the engine's own (R_POWER). Its bottom stays where the engine says (the downshift point and the lug check at
// a firm pedal: a climb keeps its pull); its top comes down with power_target_frac, to the rpm that lands one gear step up on the lowest rpm
// giving the share of the peak power (never above the engine's own top)
static float pw_lo() { return V.eng.range[R_POWER][0]; }
// A light vehicle's full-pedal shift is about POWER, not torque (Damien 31-10: 'at full throttle it may rev a bit higher', 'in car mode look at the
// horsepower'). The wheel power in a gear at a speed is the engine's power at that gear's rpm (torque curve x rpm; the same speed in the next gear is the
// same power at the rpm divided by the gear step). The gear above is worth shifting to when it gives at least the power this one gives: the crossover
// P(r / step) = P(r), which lies past the power peak where the curve falls; capped just under the limiter, never into it. Cached per engine and step.
static float car_power_peak_rpm()
{
    float best = 0, br = V.eng.rpm_limit * 0.7f;
    for (float r = idle_rpm(); r <= V.eng.rpm_limit; r += 25) { float p = curve_factor(&V.eng, r) * r; if (p > best) { best = p; br = r; } }
    return br;
}
static float car_power_cross(float step)
{
    const float lim = V.eng.rpm_limit, cap = lim * 0.975f;
    float rp = car_power_peak_rpm();
    for (float r = rp; r < cap; r += 25) {
        float lo = r / step;
        if (lo < idle_rpm()) continue;
        if (curve_factor(&V.eng, lo) * lo >= curve_factor(&V.eng, r) * r) return r;
    }
    return cap;
}
// the top for the gear g: where the next gear lands on the target rpm
static float pw_hi(int g)
{
    {   // the power crossover of this gear and the next (its step), not a torque-based power range
        int n = n_gears();
        float step = 1.28f;
        if (g >= 1 && g < n) { float r0 = vehicle_rpm_at(g, 10.0f), r1 = vehicle_rpm_at(g + 1, 10.0f); if (r0 > 1 && r1 > 1) step = r0 / r1; }
        else if (g < 1 && n > 2) { float r0 = vehicle_rpm_at(n / 2, 10.0f), r1 = vehicle_rpm_at(n / 2 + 1, 10.0f); if (r0 > 1 && r1 > 1) step = r0 / r1; }
        struct cc { float step, lim, idle, tq, val; int n; };
        static cc cache[GS_MAX + 2];
        cc &c = cache[g < 0 ? 0 : g > GS_MAX ? GS_MAX + 1 : g];
        if (!(c.val > 0 && c.step == step && c.lim == V.eng.rpm_limit && c.idle == idle_rpm() && c.tq == V.eng.torque && c.n == V.eng.n_curve)) {
            c.step = step; c.lim = V.eng.rpm_limit; c.idle = idle_rpm(); c.tq = V.eng.torque; c.n = V.eng.n_curve;
            c.val = car_power_cross(step);
        }
        return c.val;
    }
}
static float pw_mid() { return car_power_peak_rpm(); }   // (a car's kickdown aims at the power peak)
// (k = how far the demand has moved it from Eco's point to the power range's top: pw_k(); 1 = a full demand)
static float up_pt_k(int g, float k)
{
    float mo = mode_rpm();
    float u = fminf(lerpf(eco_up(g) + mo, pw_hi(g) + 0.5f * mo, k), V.eng.rpm_limit * 0.975f);
    // Eco: a soft ceiling (fuel); not during a kickdown
    if (p_mode_char > 0 && p_eco_ceil > 0 && S.mode == MODE_ECO && S.kick_until <= sim_s()) {
        float capped = fminf(u, fmaxf(eco_up(g) + mo, V.eng.range[R_HIGH][1] + p_eco_ceil));
        // (a car's floored pedal lifts the ceiling too, as it lifts Eco's cap on the aggression: ride 31-10, Eco at the floor shifted up at
        // the ceiling, 3600 rpm, with the converter holding the engine at 4300 over it; S.agg_open is 0 on a truck, so there it is the line above)
        u = lerpf(capped, u, S.agg_open);
    }
    return u;
}
static float up_pt(int g) { return up_pt_k(g, pw_k()); }
// Light-load downshift point: below the plateau, so that an early upshift to
// the plateau has room before it has to come back down.
static float base_dn(int g)
{
    return idle_rpm() + ioff(250);   // a car lugs down to idle + a margin, not to the plateau
}
// The calm upshift point of a light vehicle (see car_eco_up forward declaration): the rpm where the next gear lands just above its
// own downshift point, so a calm drive changes up as soon as the next gear can pull, never under the lug limit and never above the
// game's own eco point.
static float car_eco_up(int g)
{
    int n = n_gears();
    float k = n > 1 ? (float)(g - 1) / (n - 1) : 1;
    float game = lerpf(V.eng.range[R_LOW][1], V.eng.range[R_HIGH][1], k);
    float step = 1.28f;
    if (g >= 1 && g < n) { float r0 = vehicle_rpm_at(g, 10.0f), r1 = vehicle_rpm_at(g + 1, 10.0f); if (r0 > 1 && r1 > 1) step = r0 / r1; }
    float lo = (base_dn(g + 1) + p_hyst) * step;
    return clampf(lo, car_lug(g), game);
}
// How far a kickdown may drop a light vehicle: a stomp (high aggression) may drop as deep as the truck rule, a modest press while
// cruising only a gear or two. 0 = uncapped (trucks).
static int car_kick_drop()
{
    float a = car_agg();
    return a > 0.85f ? 0 : a > 0.5f ? 2 : 1;
}
// The downshift point only rises with demand near full pedal: pressing on at
// cruise in a low gear is not a request for power (kickdown and lugging are).
static float dn_k()
{
    return car_pw_k();   // the downshift point rises with the aggression too (and the mode's policy)
}
static float dn_pt(int g) { return lerpf(base_dn(g), pw_lo() + 0.5f * mode_rpm(), dn_k()); }
// Pull (m/s2) below which a demand downshift makes sense; a light truck with
// plenty of pull left keeps its gear (and may skip on the way up).
static float a_power() { return S.mode == MODE_ECO ? 0.2f : S.mode == MODE_POWER ? 0.45f : 0.3f; }

// The lug limit of gear g: the lowest rpm the gear is left to sit at (a calm cruise, an upshift's landing) without lugging. Derived from the
// vehicle, not an offset from idle:
//  - the engine's usable start: the lowest rpm from idle up where the torque curve gives half its peak (below it the engine gives up more than
//    half its pull: that is what lugging is). It comes from the curve alone, so it is another rpm on every engine (a turbo four that wakes at
//    1300, a V8 that has 60 % of its torque at 1000);
//  - the force the car needs there: what holds the speed on the grade of now at the mass of now (a trailer is in the mass), plus the pull the mode
//    keeps in reserve (a_power(): the acceleration below which a demand downshift makes sense), carried within p_load_from of the engine's
//    torque at that rpm (the load rule's own bar: a gear that needs more would be taken off again). In gear g that is a torque of
//    force x wheel radius / (ratio x final drive x efficiency); the floor is the lowest rpm whose curve gives it. Heavier, steeper, a taller gear
//    and a livelier mode each raise it; a torque the engine cannot give at all lands on the plateau's start (the best the curve offers).
// The floor is the higher of the two (and of the clearance above the downshift point, below). v = the speed the gear is judged at (m/s; negative = now).
// load = false leaves the force part out: the engine's own usable start, for a target rpm (the gear after a roll, an engage at speed), where a torque the
// gear cannot give is no reason to aim at the plateau.
static float car_torque_rpm(float share, float idle)   // the lowest rpm (from idle up) where the torque is this share of the curve's own peak
{
    float peak = 0;
    for (int i = 0; i < V.eng.n_curve; i++) peak = fmaxf(peak, V.eng.curve[i][1]);
    if (!(peak > 0)) return idle;
    float want = fminf(share, 0.97f) * peak;      // (a share the curve never reaches would find nothing: the plateau's start is the best it has)
    for (float r = idle; r <= V.eng.rpm_limit; r += 5) if (curve_factor(&V.eng, r) >= want - 1e-3f) return r;
    return idle;
}
static float car_lug(int g, float v, bool load)
{
    float idle = idle_rpm();
    float r = car_torque_rpm(0.5f, idle);
    float rad = wheel_radius();
    if (load && g >= 1 && g <= n_gears() && rad > 0.1f && V.eng_ok && V.eng.torque > 1 && V.mass > 0) {
        float vv = v >= 0 ? v : fabsf(T.speed);
        float k = T.ratio[g - 1] * T.diff * V.eta * V.f_scale / rad;      // N at the wheels per Nm of engine torque
        if (k > 0.1f) r = fmaxf(r, car_torque_rpm((resist(vv) + V.mass * a_power()) / (k * V.eng.torque * p_load_from), idle));
    }
    // and never closer to the downshift point than its hysteresis plus one more step: a gear that lands there is taken off again by the next dip
    // (the anti-hunt clearance every upshift landing has had; on an engine that is torque-rich from idle this is the whole floor)
    return fmaxf(r, base_dn(g) + p_hyst + 50);
}

float cs_lug_rpm(int gear, float v_ms) { return car_lug(gear, v_ms); }   // for the sims: the lug limit of a gear at a speed, as the decisions below read it

// Lowest landing rpm of an early upshift.
// A skip (2 steps or more) may land up to 100 rpm under the plateau; the
// downshift point plus hysteresis stays the lower bound, so it does not hunt back.
static float cruise_floor(int g, float under = 0, bool load = true)
{
    return fmaxf(car_lug(g, -1, load) - under, base_dn(g) + p_hyst + 50);   // a car's cruise floor is the lug limit, not the plateau
}

// ---- the launch, from the vehicle (sc_launch_pick takes these three; a truck passes its ini values)
// The speed gear g is left at on a calm drive: the rpm of its calm upshift point (eco_up) in that gear. 0 = unknown.
static float calm_end_kmh(int g) { return g >= 1 && g <= n_gears() ? vehicle_kmh_at(g, eco_up(g)) : 0.0f; }
// How high a launch may reach for a start gear above the game's pick: a gear that locks beyond the speed where first gear is left is not a launch
// gear but a skip of the first gears, which the shift logic does on its own terms. Follows the gearing: a tall first gear has a long launch.
static float car_launch_kmh() { return calm_end_kmh(1); }
// Where the launch is over for the shift logic (a takeover below it counts as pulling away: the skip dwell is short): the first two gears' calm range.
static float car_launch_end_kmh() { return calm_end_kmh(2); }
// The engine speed at which the start gear locks (clutch) or couples (converter). A converter holds the engine from idle to its stall speed
// as the pedal goes down and couples when the driveline catches up with that; a clutch slips while the engine sits where its torque has risen
// (75 % of the peak).
static float car_launch_rpm(float thr)
{
    float idle = idle_rpm();
    if (V.tc && V.tc_stall > 1.0f) return idle + fmaxf(0.0f, vehicle_tc_stall_rpm() - idle) * clampf(thr, 0.06f, 1.0f);
    return car_torque_rpm(0.75f, idle);
}
float cs_launch_rpm(float thr) { return car_launch_rpm(thr); }       // for the sims
float cs_launch_kmh(int which) { return which ? car_launch_end_kmh() : car_launch_kmh(); }

// ---- one verdict on a gear for the floored pedal, asked by the kickdown AND by the upshift's skip, so that the two cannot disagree again
// (ride 01-11, the ATS RAM 3500 with a petrol engine, Sport, pedal floored: 5>2 kickdown, 2>5 up skip landing at 3094 rpm, 5>3 kickdown, 3>6 skip at 3408,
// 6>4 kickdown, 4>6 ...: the skip landed far under the power band and the kickdown, which aims at the power peak, dropped straight back).
// A pedal where a kickdown can fire or is about to: a fast press armed it, or the pedal or the demand (it falls slowly: a foot that lets go for a moment
// is still on the floor) is within a hand's width of the mode's kickdown pedal. A skip decided in that moment would meet the next press.
static bool car_kick_pedal()
{
    if (S.kick_until > sim_s()) return true;
    return fmaxf(clampf(T.in_thr, 0, 1), S.demand) >= by_mode(0.97f, p_kick_pedal, 0.85f) - 0.1f;
}
// Gear g at an engine rpm, at the floor: -1 = under the power band (the kickdown fires from here), +1 = past what the upshift would leave at this pedal
// (its crossover, or its point at the pedal of now if that is lower; the kickdown never lands there), 0 = a gear the floored pedal can stay in.
// The kickdown only lands where this is <= 0; an upshift that skips gears only lands where it is 0 (a single step is the crossover's own: always allowed).
static int car_wot_side(int g, float rpm)
{
    if (rpm < pw_lo() + 100) return -1;
    if (rpm > fminf(pw_hi(g), up_pt(g)) - 100) return 1;
    return 0;
}
// The kickdown does not look before this many seconds after a shift (it is the kickdown's own wait). A skip that lands under the band is only a
// skip the kickdown would drop if the gear is still under the band when it first looks: the rpm the gear has by then, at the pull it has (a launch
// that lands low and climbs through the band in a second is no loop; a cruise-speed landing that creeps up is).
static const double kd_wait = 2.0;
static int car_wot_side_skip(int g, float land_rpm, float v, float a)
{
    int sd = car_wot_side(g, land_rpm);
    if (sd != -1 || v < 1.0f) return sd;
    return car_wot_side(g, land_rpm * (1.0f + fmaxf(a, 0.0f) * (float)kd_wait / v)) < 0 ? -1 : 0;
}

static float skip_accel();
// Lowest landing rpm of an upshift on the rpm path: above the downshift point that applies plus the hysteresis, and at least idle + 350.
static float land_floor(int h, float a_h)
{
    return fmaxf((a_h < a_power() ? dn_pt(h) : base_dn(h)) + p_hyst, car_lug(h));
}
static int skip_allowed()
{
    int n = (int)p_max_skip;
    // a car only skips when it is being driven hard: calm is calm, a stomp may skip as set
    if (n_gears() <= 5) return 0;   // a 3-5 speed box has big steps and takes them one by one
    float a = car_agg();
    if (a < 0.45f) return 0;
    if (a < 0.8f) return 1;
    return n;
}
// The anti-hunt dwells, scaled to the box: a torque-converter automatic swaps gears almost instantly (no clutch to open, no pause), so
// its dwells are a moment, not a truck's slow AMT seconds. The anti-hunt intent stays: the values never vanish, they just shrink. Trucks
// (no converter) keep the full ini seconds, bit for bit.
static float dwell_ud() { return V.tc ? 1.0f : p_dwell_ud; }
static float dwell_du() { return V.tc ? 0.7f : p_dwell_du; }
static float kick_dwell() { return V.tc ? 1.5f : p_kick_dwell; }
static float min_gap() { return V.tc ? 0.15f : p_min_gap; }
// Pull a skipped-to gear must keep: less with a slow box (every shift is a
// long gap without power). Damien: skip also with a fast (DCT) box.
static float skip_accel() { return p_skip_acc * (1.0f - 0.25f * smooth(0.4f, 1.5f, V.eng.shift_time)); }

// Highest gear above cur+1 (up to max_skip) an upshift at high demand may jump
// to: lands on the plateau and still pulls hard and keeps most of the pull of
// cur+1 (a car must not jump 2 -> 5). Used at the rev limit, where a light
// truck arrives in the low gears before the regular upshift.
static int skip_target(int cur, float v, float f_res, float grade)
{
    int best = cur + 1, n = n_gears();
    if (grade > 0.02f || v * 3.6f < p_on_kmh) return best;
    float a_skip = skip_accel() * 2.0f;
    for (int h = cur + 2; h <= n && h <= cur + 1 + skip_allowed(); h++) {
        float a_h = (force_at(h, v, 0.95f) - f_res) / V.mass;  // still accelerating hard: the plateau is enough
        if (drv_at(h, v) < fmaxf((a_h < a_power() ? dn_pt(h) : base_dn(h)) + p_hyst, car_lug(h, v))) break;
        // at a pedal where the kickdown can fire: only a landing it would accept (one criterion with the kickdown, car_wot_side)
        if (car_kick_pedal()) { int sd = car_wot_side_skip(h, drv_at(h, v), v, a_h); if (sd < 0) break; if (sd > 0) continue; }
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
        // a car's braking downshift stays well under the limiter: its engine brake is mild, nothing is gained by revving it (ride 31-10 21:20: 2->1 at
        // 82 km/h, 4100-5800 rpm, five times)
        if (free && rpm > V.eng.rpm_limit * 0.6f) break;
        // while braking nobody shifts up, so the upshift point means nothing there: only the limiter and the eco upshift point + 150 count
        if (l < cur - 1 && rpm > (free ? fminf(V.eng.rpm_limit * 0.85f, eco_up(l) + 150) : up_pt(l) - p_hyst)) break;
        float err = fabsf(rpm - target);
        if (err < best_err) { best_err = err; best = l; }
    }
    return best;
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
static void cs_reset(int level, const char *why)
{
    double none = -1e9;
    g_reset_at = sim_s();
    S.hold_until = S.kick_until = 0;
    S.demand_at = S.pedal_at = 0;           // the next dt is "unknown": filters step, no kickdown from a stale pedal
    S.agg_at = 0; S.agg_prev = 0;           // and the aggression's rate step too
    S.ours_at = S.shift_at = none;          // dwell and take-over timers must not hold a shift back after a pause
    S.blocked_log_at = 0;
    S.gap_since = 0;                        // a pause in the gap state is not a stuck shift (the timer starts again)
    S.pedal_prev = clampf(T.in_thr, 0, 1);
    if (level >= RESET_LOAD) {
        S.f_need = 0;
        fl_reset(&S.fl);      // a load (de)couples: the resistance learned against the old mass is void (f_bias reset in core; f_scale kept)
    }
    if (level >= RESET_HARD) {
        S.calm_k = 0; S.calm_since = 0;     // Power starts with its own shift points again
        S.pedal_on_at = none; S.skip_log_at = 0; S.kd_at = none;
        S.self = NULL;                      // refresh() starts over from the gearbox it sees next
        S.eng_seen = 0;                     // the engine line is logged again
        S.idle = 0;                         // idle is learned again (engine on 5 s, standing still)
        S.rpm_odd_s = 0;
        S.tc_slip_s = 0;
        S.unsupported = NULL;
        memset(S.grade_hist, 0, sizeof S.grade_hist);
        S.pk = -99; S.pk_at = 0;            // the grade peak belongs to the road we left
        S.letter_state = -1;
        S.rolling = 0;
        S.roll_exit = 0;
        g_rolling = 0;
        S.cc_thr = 0;
        S.demand = 0;
        S.agg = 0; S.agg_open = 0;
        S.seed_demand = S.seed_force = 1;   // (sim X4: from 0 a full pedal on a climb looked like a light one and skipped to a gear without pull)
    }
}

static void refresh(void *self)
{
    double now = sim_s();
    // The vehicle model (core/vehicle.cpp) first: inside the game's gearbox update the grade is this frame's and the
    // mass is fresh right after a reset; another gearbox object makes the model start over as well.
    vehicle_update(self);
    if (self != S.self) {
        // another truck (or the first update): start from its current gear
        if (S.self) cs_reset(RESET_HARD, "gearbox object changed");
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
    if (!V.eng_ok && S.eng_seen) { logline("engine data unreadable, CarShift pauses"); S.eng_seen = 0; }

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
    int was = S.ours;
    if (T.parking_brake || kmh < p_off_kmh) S.ours = 0;
    // By rpm: the engine has reached takeover_rpm with the clutch closed in a gear. Also above 20 km/h whatever the rpm: after a
    // pause, a load or a teleport the truck may already be cruising at 900 rpm in a high gear and never reach the rpm.
    else if (p_on_rpm > 0 ? ((T.rpm >= p_on_rpm && T.gear > 0 && clutch_seen() < 0.1f) || kmh >= fmaxf(p_on_kmh, 20.0f)) : kmh >= p_on_kmh) S.ours = 1;
    if (S.ours && !was) { S.ours_at = now; S.ours_launch = kmh < car_launch_end_kmh(); }

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
    // cruise control's throttle.
    float pedal = clampf(T.in_thr, 0, 1), eff = sense_gb_thr_eff(self);
    int cc = T.cruise > 0.5f;
    static double cc_zero_since;
    if (eff > 0.02f || !cc) cc_zero_since = now;
    // Bursts are averaged over cruise_damp_s; when it stays off the throttle
    // (slowing for traffic, adaptive cruise) the damped value follows in ~1 s.
    float cc_tau = now - cc_zero_since > 1.5 ? 1.0f : p_cc_damp;
    S.cc_thr += (float)(dt / (cc_tau + dt)) * (eff - S.cc_thr);
    if (!cc) S.cc_thr = eff;
    // with the Eco / Power pedal curve the engine's own throttle (eff) is what counts, not the raw pedal
    S.thr = cc ? fmaxf(pedal, S.cc_thr) : g_pedal_mapped ? eff : fmaxf(pedal, eff);
    // a cap holds the throttle below the pedal (the own speed limiter, the rollover guard, launch control): the capped throttle counts
    if (!cc && S.thr > eff && thr_capped()) S.thr = eff;

    // Demand rises fast (0.3 s) and falls slowly (2 s). Load itself does not
    // move the shift points: the force checks handle it.
    float tau = S.thr > S.demand ? by_mode(0.5f, 0.3f, 0.2f) : by_mode(1.5f, 2.0f, 3.0f);
    if (S.seed_demand) { S.demand = S.thr; S.seed_demand = 0; }   // right after a hard reset: the pedal of now, not 0
    else S.demand += (float)(dt / (tau + dt)) * (S.thr - S.demand);

    // The driver's aggression, from the pedal's level and how fast it moves. A held high pedal is hard, a stomp reads hard at once, a
    // lift holds it and decays slowly; cruise (pedal off) is calm. The rate is taken from the raw pedal (a stomp), the level from the
    // throttle that counts.
    {
        double pdt = now - S.agg_at;
        float rate = 0;
        if (pdt > 0.05 && pdt < 0.5) rate = (float)((pedal - S.agg_prev) / pdt);
        if (pdt > 0.05) { S.agg_prev = pedal; S.agg_at = now; }
        float target = S.thr;
        // A shift cuts the engine's throttle for a moment (0.25 at a floored pedal): the driver's pedal is what counts then, or every shift of a
        // hard drive pulls the aggression down
        if (clutch_seen() > 0.02f || now - S.shift_at < 0.6) target = fmaxf(target, pedal);
        if (rate > 0.5f) {   // a fast press is a request for pull, and it reads at once: the 0.3 s rise let the shift points sit at the calm point for the first shift of the stomp
            float st = clampf(0.3f + rate * 0.6f, 0.0f, 1.0f);
            target = fmaxf(target, st);
            S.agg = fmaxf(S.agg, st);
        }
        float atau = target >= S.agg ? 0.3f : 1.5f;   // rise fast, fall slowly (a lift holds it through the next press)
        S.agg += (float)(dt / (atau + dt)) * (target - S.agg);
        S.agg = clampf(S.agg, 0.0f, 1.0f);
        // Eco's cap lifts with a (nearly) floored pedal: the raw pedal, as a kickdown reads it (a shift's throttle cut does not close it)
        float open = smooth(0.80f, 0.95f, pedal);
        float otau = open >= S.agg_open ? 0.15f : 1.5f;
        S.agg_open += (float)(dt / (otau + dt)) * (open - S.agg_open);
        S.agg_open = clampf(S.agg_open, 0.0f, 1.0f);
    }

    // Tractive force asked for (clutch closed only): up in 0.3 s, down in 3 s.
    if (g > 0 && clutch_seen() < 0.1f && V.eng_ok) {
        float f = force_at(g, v, S.thr);
        if (S.seed_force) { S.f_need = f; S.seed_force = 0; }   // right after a hard reset: what this pedal asks now, not 0
        else { float tf = f > S.f_need ? 0.3f : 3.0f; S.f_need += (float)(dt / (tf + dt)) * (f - S.f_need); }
    }

    // Extra resistance / force factor the vehicle model misses, learned into it: V.f_bias is the additive resistance
    // error as before, V.f_scale a multiplier on the modeled engine force (a vehicle the model under- or over-rates). It only learns in steady
    // moments (core/shiftmodel.cpp); the force and the resistance are lagged 0.5 s like T.accel.
    if (g > 0 && V.eng_ok && dt > 0)
        fl_step(&S.fl, g, v, eff, dt, now, fmax(S.brake_at, S.own_brake_at), S.shift_at, 0);
    if (T.in_thr > 0.02f) { S.pedal_on_at = now; S.drv_v = fabsf(T.speed); }
    if (act_request() > 0) S.own_brake_at = now;
    // Power on a flat, light, steady stretch for a while: the shift points sink to Normal's, so a long calm cruise ends in the top gear
    // and Power does not cost fuel for nothing; a firm pedal brings them back at once
    {
        // (the same calm stretch in every mode: there it gives the cruise preference Eco's strength, the top gear that has the torque in hand
        // in any mode on a calm, flat stretch, Damien 29-10; the cruise reserve keeps it out of a gear without that pull)
        bool calm_any = p_mode_char > 0 && S.thr < p_pw_calm_thr && fabsf(T.accel) < 0.2f && fabsf(grade_pct()) < 1.5f && v > 5.5f;
        bool calm = calm_any && S.mode == MODE_POWER;
        if (!calm_any) S.calm_since = now;
        float tgt_any = calm_any && now - S.calm_since > 8 ? 1.0f : 0.0f, tgt = calm ? tgt_any : 0.0f;
        float rate = tgt > S.calm_k ? 1.0f / 6 : 1.0f;
        S.calm_k += clampf(tgt - S.calm_k, -rate * (float)dt, rate * (float)dt);
    }

    // Kickdown: the pedal pressed past kickdown_pedal quickly. Armed for 3 s
    // (cleared by the kickdown shift itself or by lifting below 0.8).
    double pdt = now - S.pedal_at;
    if (pdt > 0.05) {
        if (pdt < 0.5) {
            float rate = (float)((pedal - S.pedal_prev) / pdt);
            float kp = by_mode(0.97f, p_kick_pedal, 0.85f), kr = by_mode(2.5f, p_kick_rate, 1.0f);
            if (pedal >= kp && S.pedal_prev < kp && rate >= kr) S.kick_until = now + 3;
        }
        S.pedal_prev = pedal;
        S.pedal_at = now;
    }
    if (pedal < 0.8f) S.kick_until = 0;
    if (S.kick_until > now) S.demand = 1;

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
// A car rolls in neutral with the pedal released on a gentle stretch in Eco and Comfort (never with the brake, never with the cruise on, never in
// Sport or M): the same rule as TrueShift's without the cruise and the route (a car has neither here), so the same fuel model decides.

// Back into drive after a roll: a gear for what is asked now.
static int roll_exit_gear(float v, float brake)
{
    float target = brake > 0.02f ? eco_up(1) - 200 : lerpf(cruise_floor(n_gears(), 0, false) + 100, pw_mid(), pw_k());
    return pick_for_speed(v, target);
}

static float roll_limit()   // the road's limit (+ the cap) under the car's top; 0 = neither known
{
    float lim = road_limit_ms() > 1 ? road_limit_ms() + overspeed_cap_now() / 3.6f : 0.0f, top = speed_top_kmh() / 3.6f;
    return lim > 0 ? (top > 1 ? fminf(lim, top) : lim) : top;
}
// The speed a roll may reach (m/s): the road's limit (plus overspeed_cap_kmh, where a fine starts), never over the car's own top; with nothing known the
// speed of now.
static float roll_allowed(float v)
{
    float a = roll_limit();
    return a > 1 ? fmaxf(a, v) : v;
}
// The speed worth keeping (m/s): the speed the driver last held with the pedal, never over the limit
static float roll_use()
{
    float a = roll_limit(), d = S.drv_v > 0.5f ? S.drv_v : fabsf(T.speed);
    return a > 1 ? fminf(d, a) : d;
}
// Is a roll in neutral cheaper than staying in gear over the stretch ahead? Both ways are followed for 30 s in half seconds along the grade of now with
// the fuel model the car has learned (core/vehicle.cpp: the game burns a x omega x (torque + T_f), T_f = fuel_k x peak torque; in gear with the pedal
// off nothing while the engine drags at fuel_kd x peak torque):
//   in gear   the car coasts on the engine's drag;
//   neutral   idle (omega_idle x T_f) and no drag.
// The speed each way ends with counts: the one that ends slower pays to make the difference back at 1 / efficiency per joule, up to v_use, the speed the
// driver wants. A roll that would need the brakes to stay under the allowed speed is no roll. Returns the saving of the roll as a share of the dearer
// way (> 0: roll); NAN when it cannot tell (no gear, no engine data).
static float roll_gain(int g, float v, float allowed, float v_use, int rolling)
{
    if (g < 1 || !V.eng_ok || !(V.mass > 0) || !(V.eta > 0)) return NAN;
    const float m = V.mass, tmax = V.eng.torque, tf = fuel_k() * tmax, kd = fuel_kd(), w_i = idle_rpm() * 0.10472f, ts = V.eng.shift_time;
    const float dt = 0.5f, H = 30;
    float vg = v, fg = 0;
    for (float t = 0; t < H; t += dt) {
        float R = resist_grade(vg, grade_pct()), full = force_at(g, vg, 1.0f), rpm = rpm_at(g, vg);
        if (!(full > 1)) return NAN;
        float drag = kd * (rpm / fuel_kd_rpm()) * full / fmaxf(0.05f, curve_factor(&V.eng, rpm) * V.f_scale), a;   // (the drag grows with the rpm)
        a = (-R - drag) / m;                         // coasting in gear: no fuel, the engine drags
        vg = fminf(fmaxf(0.5f, vg + a * dt), allowed);
    }
    float vn = v, fn = 0;
    for (float t = 0; t < H; t += dt) {
        float R = resist_grade(vn, grade_pct()), a;
        fn += w_i * tf * dt;
        a = -R / m;
        if (a > 0 && vn + a * dt > allowed) return -1;   // it would need the brakes: no roll (a roll that starts over it and slows is not braked)
        vn = fmaxf(0.5f, vn + a * dt);
    }
    fn += w_i * tf * ts;   // (the shift back in at the end of the roll)
    if (rolling) fg += w_i * tf * ts;                // (already rolling: the gear way starts with that shift)
    float ve = fminf(fmaxf(vg, vn), fminf(allowed, v_use));
    float cg = fg + 0.5f * m * fmaxf(0.0f, ve * ve - vg * vg) / V.eta, cn = fn + 0.5f * m * fmaxf(0.0f, ve * ve - vn * vn) / V.eta;
    return (cg - cn) / fmaxf(1.0f, fmaxf(cg, cn));   // (a gear way that costs nothing, coasting down a descent: any roll loses)
}

// Neutral on gentle stretches with the pedal released (deliberately: 1.5 s), Eco and Comfort, never with the brake or the cruise on. Ends with real
// pedal (> 10 %), any brake, or when the speed leaves what the model allows.
static int roll_wanted(float v, double pedal_free_s, float brake)
{
    if (T.cruise > 0.5f) return 0;   // a car has no route here: the cruise holds its speed in gear
    if (p_roll <= 0 || (S.mode != MODE_ECO && S.mode != MODE_NORMAL) || !S.ours) return 0;
    if (brake > 0.02f) return 0;
    if (S.rolling ? T.in_thr > 0.1f : (T.in_thr > 0.02f || T.eff_thr > 0.02f || pedal_free_s < 1.5)) return 0;
    const float allowed = roll_allowed(v);
    if (!S.rolling) {
        bool eco = S.mode == MODE_ECO;  // (a shorter pause between rolls in Eco)
        if (sim_s() - S.roll_at < (eco ? 5 : 10) || sim_s() - S.shift_at < 1) return 0;
        // A roll where it saves fuel (roll_gain: the learned fuel model against the gear's drag, the speed made back, the allowed speed)
        float gain = roll_gain(T.gear, v, allowed, roll_use(), 0);
        return gain > 0;
    }
    // rolling: on while it still saves against going back into gear now (that shift counted), and under the allowed speed
    if (v > allowed) return 0;
    float gain = roll_gain(S.roll_exit > 0 ? S.roll_exit : roll_exit_gear(v, 0), v, allowed, roll_use(), 1);
    // The roll is the driver's: it lasts until the pedal or the brake, unless it runs into a speed that wants the engine's drag
    return gain == gain && gain > -0.999f;
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

// A protection's request for the gear (core/gearreq.h): the limiter, the stall and a wrong gear outrank everything the soft law below says. The rule
// still ends the chain with the same return; the post is what the arbiter sees.
static int protect(int gear, int why)
{
    const gear_req r = { gear, why, GP_PROTECT };
    gear_req_post(&r);
    return gear;
}

static int decide_gear_raw(void *self, int *why)
{
    *why = W_NONE;
    int cur = sense_gb_gear(self), n = n_gears();
    float v = fabsf(T.speed);
    double now = sim_s();
    if (!V.eng_ok || n < 2 || cur < 0 || cur > n || wheel_radius() <= 0.1f) { *why = W_NO_DATA; return 0; }

    float f_res = resist(v);
    float brake = fmaxf(T.in_brake, T.eff_brake);
    if (brake > 0.05f) S.brake_at = sim_s();
    int roll = roll_wanted(v, now - S.pedal_on_at, brake);
    if (S.rolling) {
        if (roll) { *why = W_ECO_ROLL; return ROLL; }
        S.rolling = 0;
        S.roll_at = now;
        if (cur == 0) {
            *why = W_ROLL_END;
            S.roll_exit = roll_exit_gear(v, brake);
            S.roll_exit_at = now;
            return S.roll_exit;
        }
    }
    if (cur < 1) { *why = W_NEUTRAL; return 0; }
    if (!S.ours) { *why = W_LOW_SPEED; return 0; }

    float rpm = rpm_at(cur, v);   // the engine's rpm: the stall speed while the converter slips, else the driveline's
    float lim = V.eng.rpm_limit;
    float thr = S.thr;
    // lift-off by the pedal itself: the game's throttle reads 0.08 at pedal 0, which hid the coast and the junction check
    int coast_p = (T.cruise > 0.5f ? thr : clampf(T.in_thr, 0, 1)) < 0.05f;
    double since = now - S.shift_at;
    int kick = S.kick_until > now;
    S.up_pt = up_pt(cur);
    S.dn_pt = dn_pt(cur);
    S.floor_rpm = 0;

    if (S.mode == MODE_M) { *why = W_M; return cur; }   // M: the gear stays where the driver's hints put it (overspeed_tone warns at the limiter)

    // Protections, also while a hint is held. Far above the limiter (a wrong
    // gear after neutral): straight to a gear that fits the speed. On a converter the engine rpm is the stall speed while it slips,
    // so the limiter and the stall rule read the engine's rpm, not the driveline's.
    if (rpm > lim * 1.05f) { *why = W_WRONG_GEAR; return protect(pick_for_speed(v, pw_mid()), *why); }
    if (rpm > lim * 0.985f && cur < n) {
        int h = S.thr < 0.75f ? cur + 1 : skip_target(cur, v, f_res, sinf(V.grade_deg * 0.0174533f));
        *why = h > cur + 1 ? W_REV_LIMIT_SKIP : W_REV_LIMIT;
        S.hold_until = 0;  // a held hint gear ends here, the next gear is CarShift's again
        return protect(h, *why);
    }
    // A locked engine under idle + a margin (it would stall): one or two gears down, the highest that sits clear of the stall point,
    // taken before the rpm sags under idle. While the clutch slips (a pull-away the smooth-start law owns) the driveline rpm is not the
    // engine's, so a low rpm there is no stall: the slip law opens the clutch for it.
    // A converter is the same: while it slips the engine sits at the stall speed, never at the driveline's low rpm.
    if (rpm < idle_rpm() + ioff(120) && cur > 1 && clutch_seen() < 0.02f) { *why = W_STALL; return protect(pick_down(cur, v, idle_rpm() + ioff(250), false, 2), *why); }

    // not right after a launch shift
    if (kick && car_wot_side(cur, rpm) < 0 && cur > 1 && since > kd_wait && clutch_seen() < 0.1f && v * 3.6f > 15) {
        int l = pick_down(cur, v, pw_mid() + 100, false, car_kick_drop());   // a light vehicle only drops as deep as its aggression allows
        // never into a gear that is at its upshift point a moment later (the verdict the upshift's skip asks too: car_wot_side)
        while (l < cur && car_wot_side(l, rpm_at(l, v)) > 0) l++;
        if (l < cur) { *why = W_KICKDOWN; return l; }
    }

    // A hint gear is held, but not up to the limiter: pulling past the top of the power range hands it back
    //.
    if (S.hold_until > now) {
        if (!(rpm > pw_hi(cur) && S.thr > 0.05f)) { *why = W_HINT_HOLD; return cur; }
        S.hold_until = 0;
        logv(1, "hint hold ended at %.0f rpm (top of the power range)", rpm);
    }

    if (roll && since > 1) {
        S.rolling = 1;
        S.roll_from = cur;
        g_rolling = 1;  // right away: Cruise keeps the game from canceling cruise when the box goes to neutral
        S.roll_at = now;
        *why = W_ECO_ROLL;
        return ROLL;
    }

    if (brake > 0.05f) {
        // stay above the game's neutral threshold, which rises with brake^2
        float b = fminf(brake, 1.0f), opens = 0.8f * idle_rpm() + b * b * idle_rpm();
        float floor = fmaxf(fmaxf(idle_rpm() + ioff(250), eco_dn(cur) - 100), opens + ioff(250));
        const float v_l = v;
        if (rpm_at(cur, v_l) < floor && cur > 1 && since > 0.5) {
            // land high, so that fewer downshifts are needed on the way down
            int l = pick_down(cur, v_l, eco_up(1) + by_mode(-300, -200, -100), true);
            if (l < cur) { *why = W_BRAKING; return l; }
        }
        *why = W_BRAKING_HOLD;
        return cur;
    }

    if (since < min_gap()) { *why = W_MIN_GAP; return cur; }

    // Downshift: below the downshift point, or lugging (full pedal, losing
    // speed and no force left in this gear).
    float fmax = force_at(cur, v, 1.0f);
    // A gear that cannot even hold the speed at full throttle any more (a steep climb): no waiting for the throttle to come up (cruise and
    // ACC raise it slowly) or for the rpm to sag (Damien 24-10, 545-553 s: 10-11 % at 43 km/h in 12, held to 759 rpm and 35 km/h, the lead
    // pulled away 13 -> 58 m). It needs real throttle (> 0.25) and a speed that is falling, so a burst of gas on the flat does nothing.
    int cannot_hold = thr > 0.25f && fmax < 1.05f * f_res && T.accel < -0.1f;
    int lug = (thr > 0.8f || cannot_hold) && T.accel < 0 && fmax - f_res < V.mass * 0.03f && rpm < pw_lo() + 100;
    // The demand-raised point, and a climb's raise of it (the gear that can just hold the speed): see dn_point.
    float dn_base, dn_eff = dn_point(cur, v, thr, f_res, &dn_base);
    if (cur > 1 && (rpm < dn_eff || lug)) {
        int deep = rpm < dn_base - 100 || rpm < idle_rpm() + ioff(250);   // the load-raised point does not skip the anti-hunt dwell
        if (S.shift_dir > 0 && since < dwell_ud() && !deep && !lug) { *why = W_ANTI_HUNT_DOWN; return cur; }
        float target = lug ? pw_mid() : lerpf(0.5f * (eco_dn(cur - 1) + eco_up(cur - 1)) + 100, pw_mid(), pw_k());
        int l = pick_down(cur, v, target, false, car_kick_drop());   // a light vehicle's cruising downshift only drops as deep as its aggression allows
        if (l == cur) l = cur - 1;
        *why = lug ? W_LUGGING : (cur - l > 1 ? W_DOWN_SKIP : W_DOWN);
        return l;
    }

    int coast = coast_p;
    int skips = skip_allowed();   // (skips_free: what is allowed once the takeover is a few seconds old)
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
    // not within 6 s after braking below 60 km/h
    int steady = thr >= 0.15f && thr >= 0.6f * S.demand && T.accel > -0.15f && now - S.ours_at > 3 && (now - S.brake_at > 6 || v * 3.6f >= 60) &&
                 v * 3.6f >= p_on_kmh + 5;
    // (not within the kickdown/lugging dwell either: the early path took 9 -> 10 two seconds after a lugging 10 -> 9 on a 2.3 % climb)
    // (not with a junction ahead: the cap above lets a gear revved far past the upshift point through, but an early upshift into the
    // junction is still wrong - it only ends in a shift back)
    // (a light vehicle: the pedal and the aggression too, the demand needs 0.3 s to follow a stomp and the early upshift would take its first gear in that time)
    if (steady && cur < n && fmaxf(fmaxf(S.demand, clampf(T.in_thr, 0, 1)), S.agg) < p_power_from && !(S.shift_dir < 0 && since < dwell_du()) && !(now - S.kd_at < by_mode(3, kick_dwell(), 8)) && !climbing) {
        float need = fmaxf(S.f_need, f_res), k = mode_reserve();
        if (grade_pct() > -0.5f) k += 0.1f;
        int best = cur;
        for (int h = cur + 1; h <= n && h <= cur + 1 + skips; h++) {
            float fl = cruise_floor(h, h > cur + 1 ? 100.0f : 0.0f);
            if (h == cur + 1) S.floor_rpm = fl;
            if (drv_at(h, v2) < fl) break;
            if (force_at(h, v2, 1.0f) < k * need) break;
            // on a climb not into a gear the load rule (load_down_from) would take off again as soon as the speed dips
            if (p_load_from < 1 && grade > 0.01f && need > p_load_from * force_at(h, v2, 1.0f)) break;
            if (h > cur + 1 && (grade > 0.02f || (force_at(h, v2, 0.85f) - f_res) / V.mass < 0)) break;
            best = h;
        }
        if (best > cur) { *why = best - cur > 1 ? W_UP_EARLY_SKIP : W_UP_EARLY; return best; }
    }

    // Upshift on rpm: above the upshift point, the new gear must stay above its
    // own downshift point and still pull after the speed lost during the shift.
    // On a climb the grade that counts is the steepest of now, the last seconds (a peak that fades 0.15 %/s) and the route
    // ahead.
    // The speed lost in the shift is reckoned with that grade too, and a climb asks a real margin, not 0.02-0.05 m/s2.
    float gp = grade_pct();
    S.pk = fmaxf(gp, S.pk - 0.15f * (float)clampf((float)(now - S.pk_at), 0, 60)); S.pk_at = now;
    float g_up = gp > 1.5f ? fmaxf(gp, S.pk) : gp;   // the peak only while still climbing (over the crest it is the route's job)
    float f_up = fmaxf(f_res, resist_grade(v, g_up));
    float v2u = fmaxf(0.5f, v - f_up / V.mass * V.eng.shift_time);
    if (cur < n && rpm > S.up_pt + (coast ? by_mode(50.0f, 200.0f, 200.0f) : 0)) {
        // Power: just off the pedal the gear stays (3 s), unless the revs are nearly at the limit
        if (coast && S.mode == MODE_POWER && p_mode_char > 0 && now - S.pedal_on_at < 3 && rpm < lim * 0.9f) { *why = W_POWER_HOLDS; return cur; }
        // Rolling out or lifting: no upshift, it only ends in a shift back. Not with
        // the pedal at 0.5 or going up: whoever wants power at a junction gets it.
        bool wants_power = clampf(T.in_thr, 0, 1) >= 0.5f || thr >= 0.5f || thr > S.demand + 0.05f;
        if (!wants_power && T.accel < -0.2f && rpm < lim * 0.9f) { *why = W_UP_VETO; return cur; }
        // (not once the next gear pulls more than this one, even after the speed lost in the shift: an engine whose torque dies before its limit
        // sits on that edge and never gets 100 over the upshift point; sim 29-10, MAN D38 470 at 10 t: 6 s in 9 at 1990 rpm after a kickdown)
        bool next_pulls = force_at(cur + 1, v2u, 1.0f) >= force_at(cur, v, 1.0f);
        if (S.shift_dir < 0 && since < (now - S.kd_at < by_mode(3, kick_dwell(), 8) + 1 ? by_mode(3, kick_dwell(), 8) : by_mode(dwell_du(), dwell_du(), 4.0f)) && rpm < S.up_pt + 100 && !next_pulls) { *why = W_ANTI_HUNT_UP; return cur; }
        float a_req = lerpf(0.02f, 0.05f, S.demand);
        if (g_up > 2 && !coast) a_req = fmaxf(a_req, lerpf(0.05f, 0.12f, clampf((g_up - 2) / 3, 0, 1)));
        int best = cur, maxg = cur + 1 + (coast_p ? 0 : skips);   // no skipping with the pedal off: it only ends in the next brake (sim Y1: 6>8 at the lift-off, 8>6 a second later)
        const char *block = "";
        for (int h = cur + 1; h <= n && h <= maxg; h++) {
            float rh = drv_at(h, v2u);
            float a_h = (force_at(h, v2u, 0.95f) - f_up) / V.mass;
            if (rh < land_floor(h, a_h)) {
                block = "rpm too low after shift";
                break;
            }
            if (!coast && (force_at(h, v2u, 0.95f) - f_up) / V.mass < a_req) { block = "not enough pull"; break; }
            // on a climb the new gear must carry the load under load_down_from, or the load rule shifts straight back
            if (!coast && p_load_from < 1 && g_up > 1.0f && f_up > p_load_from * force_at(h, v2u, 1.0f)) { block = "too much load after shift"; break; }
            if (h > cur + 1) {
                // At a pedal where the kickdown can fire: only to a gear it would accept (car_wot_side); a landing past the upshift point is not taken
                if (car_kick_pedal()) {
                    int sd = car_wot_side_skip(h, rh, v2u, a_h);
                    if (sd < 0) { block = "skip: lands under the power band, the kickdown would drop it"; break; }
                    if (sd > 0) continue;
                }
                // At high demand only when the skipped-to gear still pulls hard (light truck).
                bool pull_away = v2u < 20 / 3.6f;   //
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
            // why was this a single step when a skip was allowed? one line per 5 s
            if (best == cur + 1 && maxg > cur + 1 && cur + 2 <= n && (T.in_thr < 0.3f || v < 25 / 3.6f) && now - S.skip_log_at > 5) {
                S.skip_log_at = now;
                logv(2, "single up %d->%d, no skip: %s (%.0f km/h, %.0f rpm, pedal %.2f, demand %.2f, grade %.1f%%, %.1f t)", cur, best,
                     block[0] ? block : "the next gear above was not tried", v * 3.6f, rpm, T.in_thr, S.demand, grade_pct(), V.mass / 1000);
            }
            *why = best - cur > 1 ? W_UP_SKIP : W_UP; return best;
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

static void note_gear_change(void *self)
{
    int g = sense_gb_gear(self);
    if (g == S.last_gear) return;
    double now = sim_s();
    float v = fabsf(T.speed);
    int prev = S.last_gear;
    S.shift_dir = g > prev ? 1 : -1;
    S.shift_at = now;
    S.last_gear = g;
    S.dm_last = dm_share(g);   // the demand at this shift, for the flip lock and the hunt metric
    if (S.cmd_why == W_KICKDOWN) S.kick_until = 0;
    if (g < prev && g > 0 && (S.cmd_why == W_KICKDOWN || S.cmd_why == W_LUGGING)) S.kd_at = now;
    int hint = prev > 0 && g > 0 && g != S.commanded && S.ours;
    int why = hint ? W_HINT : g == S.commanded ? S.cmd_why : W_GAME;
    if (why == W_HINT) hint = 1;   // a shift the driver's hint asked for is a hint even when we carried it out (M mode): it is not our shift to hunt-metric
    DEV_NOTE(shift(prev, g, why_text[why], hint, now));
    sc_track_shifts(S.ours, prev, g, hint, now, S.dm_last, mode_label(S.mode));

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

// The gear for now (decide_gear_raw above) with the one rule that holds for every path of it: a car on the cruise takes no 1st above 30 km/h, whichever
// rule asked (ride 31-10 21:00, the Crown Victoria: 1st at 52-59 km/h, 3500-4000 rpm of converter slip under a controller's throttle blip,
// twelve times in 25 s). 2nd stays the lowest gear the cruise asks of it. The same for a braking downshift (ride 31-10 21:20, 2->1 at 76-85 km/h,
// 4000-5800 rpm)
static int decide_gear(void *self, int *why)
{
    gear_req_begin();
    int want = decide_gear_raw(self, why);
    {   // the soft law's answer (and every rule above the protections: it is CarShift's own); a protection that posted outranks it
        const gear_req soft = { want, *why, GP_SOFT };
        gear_req_post(&soft);
        want = gear_req_resolve(why);
    }
    if (want == 1 && ((T.cruise > 0.5f && T.in_thr < 0.05f) || *why == W_BRAKING) && fabsf(T.speed) * 3.6f > 30 && sense_gb_gear(self) >= 2) {
        want = 2;
        if (sense_gb_gear(self) == 2) *why = W_HOLD;
    }
    return want;
}
// The hook bodies (core/shifthook.cpp calls them once the game's own call has run, and only while this brain has the gearbox)
static void bs_window(void *self)
{
    refresh(self);
    note_gear_change(self);
    int why;
    int want = decide_gear(self, &why);
    S.why = why_text[why];
    S.want = want;
    S.target = sense_gb_target(self);
    // Neutral goes in through the tail hook (the chooser path clamps to +-1).
    if (want > 0 && want != ROLL && want != sense_gb_gear(self) && takeover()) act_gb_force(self);
    S.view_at = wall_s(); S.view_gear = sense_gb_gear(self); S.view_state = sense_gb_state(self);   // the dashboard's view (bs_view)
}

static int bs_choose(void *self, int stock)
{
    int ret = stock;
    int why = W_GAME;
    // A shift hint of the driver in M (the game's handler moves the target one gear, before this chooser runs). Out of its rpm window
    // the chooser runs every tick and the call site writes its result into the
    // target: our "stay in this gear" erased the press until the pedal came off. So a target nobody here asked for is honored.
    int hint_tgt = 0;
    if (S.mode == MODE_M && sense_gb_state(self) == 0) {
        int cur = sense_gb_gear(self), tgt = sense_gb_target(self);
        if (cur > 0 && tgt > 0 && tgt != cur && tgt != S.commanded && abs(tgt - cur) <= 3 && tgt <= n_gears()) hint_tgt = tgt;
    }
    if (takeover()) {
        int w;
        refresh(self);
        int want = decide_gear(self, &w);
        if (want == ROLL) { ret = sense_gb_gear(self); why = w; }
        else if (want > 0) { ret = want; why = w; }
    }
    if (hint_tgt) {
        // M: the driver's hint is taken at once and never reverted by us; the only shift refused is one that would over-rev
        // (a down hint that lands past the limiter) or stall (an up hint that lands under idle + a margin).
        int cur = sense_gb_gear(self);
        float r_hint = vehicle_engine_rpm_at(hint_tgt, fabsf(T.speed));
        int refuse = (hint_tgt < cur && r_hint > V.eng.rpm_limit * 0.97f) || (hint_tgt > cur && r_hint < idle_rpm() + ioff(120));
        static double said;
        if (sim_s() - said > 0.5 || sim_s() < said) { said = sim_s(); logv(2, "hint target %d seen in %d (rpm %.0f): %s", hint_tgt, cur, T.rpm, refuse ? "refused (over-rev or stall)" : "honored"); }
        if (!refuse) { ret = hint_tgt; why = W_HINT; }
    }
    // Every gear we hand back counts as ours, so that only the hint buttons
    // show up as shifts nobody here asked for.
    if (ret != sense_gb_gear(self)) { S.commanded = ret; S.cmd_why = why; }
    return ret;
}

// The automatic's stop threshold: below it the clutch opens / neutral. Stock
// 0.8 x idle (+ a brake term the game adds), which pushes until ~2 km/h.
// While braking to a stop without throttle (below 15 km/h): idle + 150 rpm;
// the game's own brake term comes on top, so harder braking opens earlier.
// Not at speed: ride ts3a went to neutral at 74 km/h in 12 when braking hard.
static float stop_threshold(float idle, float thr, float brake)
{
    return sc_stop_threshold(p_stop, p_no_creep, 0, S.unsupported || manual_box(), idle, thr, brake);
}
// Runs at the very end of every automatic update (also in neutral), right before the generic gearbox acts on the target gear: eco-roll goes in and
// out here, and the neutral watchdog (lka-roll-re notes part B).
static void tail_pre(void *self)
{
    if (!takeover() || self != S.self) return;
    int g = sense_gb_gear(self), st = sense_gb_state(self);
    // After a roll: our exit gear as the target; the generic gearbox then parks in its gap state (clutch open, gear 0) until there is throttle
    // (neutral-exit-re notes § 1). There we do its own engage step.
    if (S.roll_exit > 0) {
        if (g != 0 || sim_s() - S.roll_exit_at > 3 || fabsf(T.speed) * 3.6f < p_off_kmh) {
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
    // Parked in the gap state while moving, with a gear as target (a pause in the middle of an upshift left gear 0 and the clutch open for 11 s
    // at 38 -> 25 km/h: our cruise throttle waits for the shift to end, the gearbox waits for throttle). After 1.5 s we do its engage step.
    {
        int tgt = sense_gb_target(self);
        if (st == 1 && g == 0 && tgt > 0 && !S.rolling && fabsf(T.speed) * 3.6f > p_off_kmh) {
            if (!S.gap_since) S.gap_since = sim_s();
            // right after a pause the gap state is the pause's, not a shift's
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
    float v = fabsf(T.speed), brake = fmaxf(T.in_brake, T.eff_brake);
    if (roll_wanted(v, sim_s() - S.pedal_on_at, brake)) { act_gb_set_target(self, 0); return; }
    S.rolling = 0;
    S.roll_at = sim_s();
    int pick = roll_exit_gear(v, brake);
    act_gb_set_target(self, pick);
    S.roll_exit = pick;
    S.roll_exit_at = sim_s();
    S.commanded = pick;
    S.cmd_why = W_ROLL_END;
}
// The start gear by physics (see bs_start): lock speed and slip time at this mass and grade, never by a gear count. `stock` = the game's
// pick; the result may be above it (launch_max_kmh) or below it.
static int launch_pick(int stock, float thr, bool force = false)
{
    sc_launch c = { takeover(), p_start_fix, car_launch_kmh(), p_slip_s, p_launch_torque, car_launch_rpm(thr), p_kick_pedal, S.mode, 0 };
    return sc_launch_pick(c, stock, thr, force);
}
static int bs_start(void *self, int stock)
{
    if (stock <= 0) return stock;
    vehicle_mass_stale();  // fresh mass for the start
    refresh(self);
    // Engaging while moving (after an eco-roll or neutral): never the start gear, a gear that fits the speed.
    float v = fabsf(T.speed);
    if (takeover() && V.eng_ok && v * 3.6f > p_off_kmh && wheel_radius() > 0.1f) {
        S.rolling = 0;
        S.roll_at = sim_s();
        // the demand filter lags a pedal that goes down hard right at the engage (25-10, 2397 s: full pedal at a junction, demand 0.02, gear 10
        // at 877 rpm on a 2.3% climb, then 10->8 two seconds later): the pedal itself counts
        // Engaging at speed is a gear for the speed, not a start gear: launch_pick assumes a standstill and would
        // pick a gear far too tall for the speed (ride 30-10: "engage at 5 km/h: gear 5", then a stall at 574 rpm). Only a
        // standstill gets the physics start gear.
        int g = pick_for_speed(v, lerpf(cruise_floor(n_gears(), 0, false) + 100, pw_mid(), smooth(p_power_from, p_power_full, fmaxf(S.demand, clampf(T.in_thr, 0, 1)))));
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

// For the wish pedal and the cab (suite.h ts_start_gear): the start gear the physics would pick, also where CarShift does not have the car
// (below its takeover speed, which is where a stop and a launch happen). The game's own pick is its default: 4 alone, 1 with a trailer.
static int bs_start_gear(float thr)
{
    int n = n_gears();
    if (n < 1 || !V.eng_ok || !(V.mass > 0) || !(wheel_radius() > 0.1f)) return 0;
    return launch_pick(T.trailer ? 1 : (n < 4 ? n : 4), clampf(thr, 0, 1), true);
}
static float bs_idle_rpm() { return idle_rpm(); }
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
    o->up_rpm = S.up_pt > 0 ? S.up_pt : 0;
    o->dn_rpm = S.dn_pt > 0 ? S.dn_pt : 0;
    o->demand = S.demand;
    return 1;
}
static void bs_commanded(int gear) { S.commanded = gear; S.cmd_why = W_START_CORRECTED; }
// ---------------------------------------------------------------- module

static char g_status[200] = "off";
static const char *cs_status() { return g_status; }

static void cs_start()
{
    if (p_enabled <= 0) { logline("CarShift disabled in the ini"); snprintf(g_status, sizeof g_status, "off (ini)"); return; }
    S.pk = -99;
    int done, want;
    g_active = shift_hooks_install(&done, &want);   // the game's hook sites are shared with TrueShift (core/shifthook.cpp)
    if (g_active && p_stop > 0) {
        if (shift_hook_stop()) logline("smooth stop hook in place");
        else { p_stop = 0; logline("smooth stop off: threshold code not found"); }
    }
    // The update's tail hook carries the neutral watchdog (a shift interrupted by a pause leaves gear 0 with the clutch open): it needs the engage code.
    if (g_active) {
        if (shift_hook_tail()) logline("eco-roll and neutral watchdog hook in place");
        else logline("eco-roll and neutral watchdog off: %s", shift_tail_engage_ok() ? "update tail not found" : "gearbox engage code not found");
    }
    logline("CarShift %s: %d/%d call sites hooked%s", g_active ? "active" : "NOT active", done, want,
            p_takeover > 0 ? "" : " (observe only)");
    snprintf(g_status, sizeof g_status, "%s, %d/%d hooks; smooth stop %s, dashboard letter %s",
             g_active ? (p_takeover > 0 ? "on" : "on (observe only)") : "NOT active", done, want,
             p_stop > 0 ? "on" : "off", p_letter > 0 ? "on" : "off");
    if (p_trace > 0) {
        g_trace = open_out("carshift.csv", "w");
        if (g_trace)
            fputs("t,kmh,rpm,gear,target,want,why,pedal,thr_eff,brake,mbrake,retarder,cruise_kmh,grade_pct,mass_t,"
                  "demand,up_pt,dn_pt,hold,kick,accel,rpm_k,clutch,mode,thr,f_need_kn,floor,ours,f_bias_kn,idle,f_scale\n", g_trace);
    }
}

static char cur_letter() { return mode_dash(S.mode); }
static void cs_frame()
{
    g_rolling = S.rolling || S.roll_exit > 0;  // neutral by our choice, also until the exit gear is in
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
    sc_dash_letter(p_letter > 0, g_active && !S.unsupported && !manual_box(), S.self, &S.letter_state, mode_update, cur_letter);
    sc_idle_learn(&S.idle, p_idle, 1);
    if (!g_trace || !S.self || T.paused) return;
    double now = wall_s();
    if (now - g_trace_at < 0.1) return;
    g_trace_at = now;
    double sim = sim_s();
    fprintf(g_trace, "%.2f,%.1f,%.0f,%d,%d,%d,%s,%.2f,%.2f,%.2f,%d,%u,%.0f,%.2f,%.2f,%.2f,%.0f,%.0f,%d,%d,%.2f,%.3f,%.2f,"
            "%c,%.2f,%.1f,%.0f,%d,%.2f,%.0f,%.3f\n",
            now, fabsf(T.speed) * 3.6f, T.rpm, T.gear, S.target, S.want, S.why ? S.why : "", T.in_thr,
            T.eff_thr, T.in_brake, T.motor_brake, T.retarder, T.cruise * 3.6f, grade_pct(), V.mass / 1000, S.demand,
            S.up_pt, S.dn_pt, S.hold_until > sim, S.kick_until > sim, T.accel, V.rpm_k, T.eff_clutch,
            mode_dash(S.mode), S.thr, S.f_need / 1000, S.floor_rpm, S.ours, V.f_bias / 1000, idle_rpm(), V.f_scale);
    static double flushed;
    if (now - flushed > 2) { flushed = now; fflush(g_trace); csv_cap(&g_trace, "carshift.csv"); }
}

static void cs_stop()
{
    if (g_trace) fflush(g_trace);
}

// The vehicle's configuration (SDK) arrived (c NULL: the dispatcher replays it, the class was not known yet): not a gearbox we manage ->
// left to the game, quietly and with one log line.
static void cs_config(const scs_telemetry_configuration_t *c)
{
    if (c && strcmp(c->id, "truck")) return;
    if (T.fwd_gears >= 1 && T.fwd_gears <= 2) unsupported("1-2 forward gears (electric or single-speed drive)");
    if (manual_box() && !S.manual_logged) {
        S.manual_logged = 1;
        logline("CarShift: gearbox type '%s' is not an automatic, staying idle", T.shifter);
    }
    if (!manual_box()) S.manual_logged = 0;
}

// This brain drives the cars; the trucks' is modules/trueshift.cpp. core/shifthook.cpp hands the game's hooks to the brain of the class.
static int bs_is_on() { return g_active; }
static const shift_brain k_brain = { "CarShift", VCM_CAR, bs_is_on, bs_window, bs_choose, bs_start, tail_pre, stop_threshold,
                                     bs_start_gear, bs_idle_rpm, bs_mode, bs_commanded, bs_view };
static struct brain_reg { brain_reg() { shift_register(&k_brain); } } k_brain_reg;
module m_carshift = { "carshift", "CarShift", g_params, sizeof g_params / sizeof g_params[0],
                      cs_start, cs_frame, cs_config, cs_stop, cs_reset, cs_status, NULL, VCM_CAR };
