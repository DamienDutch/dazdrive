// Pedal: the torque pedal, a drive-by-wire style throttle (off by default, [pedal] wish). This module is the DELIVERY of the driver's demand; the demand
// itself, one definition for the throttle and for the gearbox, is core/wish.{h,cpp} (package P).
//
// Today the pedal asks a share of the engine's torque curve, so after an upshift the same pedal gives less pull. With the torque pedal the driver's pedal
// is a share of what the truck can put on the road at this speed (the force envelope: its strongest gear inside the usable revs), and the throttle is
// what delivers that share in the gear the truck is in. [pedal] rev_gain says how much the ask follows the engine's revs inside a gear: 0 = the same push at
// the same speed in any gear, 1 = a real truck's pedal (the pull holds in a gear and steps down at an upshift).
//
// Per tick (pedal_wish, called by Cruise where its old pedal-curve stage sits):
//  1. pass       passes(): does the demand drive the throttle this tick, or does the pedal go through (below)
//  2. demand     wish_tick: the shaped share (the pedal to the mode's exponent, rise limit), the envelope, the level  (core/wish.cpp)
//  3. throttle   gas = F_d / (the force this gear gives at full throttle at this speed), 0..1, F_d = what this gear is asked (wish_in)
// While standing in a forward gear and during the launch (the smooth start holds the clutch) the shaping runs, but the throttle passes through: the
// launch reads the shaped share (wish_launch_share) for its rpm aim, so a light pedal pulls away gently in Eco and the pedal feels the same at 0 km/h as
// at 30. Everything else leaves the throttle alone (pass-through): reverse and neutral, an electric or manual box, a vehicle model that is not valid, the
// crawl, the cruise / Stop&Go drive-off (the cruise throttle, not the pedal), and whatever other owner drives the throttle (cruise, ACC, AEB, Auto
// Hold). A shift (the clutch slipping or open) also passes the throttle through, but the demand stays so the gear choice does not see a drop to zero.
//
// What the other modules read of the demand is core/wish.h (wish_get for the gearbox, wish_full for the cruise, wish_launch_share for Creep and LowSpeed):
// this module only delivers it.
//
// The log (promise P7): why the throttle is, or is not, the demand's is one line each time the reason changes (PDR_* below), with the pedal, the
// mode, the gear and the speed. The states that come and go with every shift (the demand drives the throttle, the clutch is not locked, a tick
// without a valid frame time) are written at log level 2 and at most every 0.5 s, the others (another owner has the throttle, reverse or neutral,
// the crawl, ...) at level 1.
#include "../core/suite.h"
#include "../core/game.h"
#include "../core/vehicle.h"
#include "../core/bands.h"
#include "../core/wish.h"
#include <math.h>
#include <string.h>

static float p_wish, p_curve_eco, p_curve_norm, p_curve_pw, p_curve_toe, p_rise_eco, p_rise_norm, p_rise_pw, p_rev_gain, p_grade_assist;

static param g_params[] = {
    { "wish", &p_wish, 1, "1 = torque pedal: the pedal asks a share of what the truck can pull at this speed (its strongest gear inside the usable revs), and the throttle is what delivers that in the gear the truck is in. 0 = the pedal asks a share of the engine's torque, as before. Not with the cruise on, not in reverse; through a shift the throttle passes through but the demand stays" },
    { "curve_eco", &p_curve_eco, 0.365f, "Eco: the pedal's map. The ask is s = (1 + c) p^k / (p^k + c) of what the truck can pull here and now, p the pedal, k the toe (curve_toe) and c this value: c large = the diagonal, c small = livelier in the middle of the travel (with the shipped toe 1.47 and c 0.365 a pedal of 0.2 asks 28 %, 0.3 asks 43 %, half a pedal 68 %, 0.6 asks 77 %). A full pedal is always all of it, and it is the same map in every truck. Set the toe and the three curves together. Three ready-made sets, paste all four values into [torque_pedal.curve]: Between (the default, halfway between P6b and P6c): curve_toe 1.47, curve_eco 0.365, curve_normal 0.295, curve_power 0.245. P6b (a lively light pedal): curve_toe 1, curve_eco 0.8, curve_normal 0.6, curve_power 0.4. P6c (a soft start, cruising holds near pedal 0.2): curve_toe 2.15, curve_eco 0.165, curve_normal 0.14, curve_power 0.135." },
    { "curve_normal", &p_curve_norm, 0.295f, "Normal: the same (with the shipped toe 1.47 and c 0.295 a pedal of 0.2 asks 31 %, 0.3 asks 47 %, half a pedal 71 %, 0.6 asks 80 %). Set the toe and the three curves together. Three ready-made sets, paste all four values into [torque_pedal.curve]: Between (the default, halfway between P6b and P6c): curve_toe 1.47, curve_eco 0.365, curve_normal 0.295, curve_power 0.245. P6b (a lively light pedal): curve_toe 1, curve_eco 0.8, curve_normal 0.6, curve_power 0.4. P6c (a soft start, cruising holds near pedal 0.2): curve_toe 2.15, curve_eco 0.165, curve_normal 0.14, curve_power 0.135." },
    { "curve_power", &p_curve_pw, 0.245f, "Power: the same (with the shipped toe 1.47 and c 0.245 a pedal of 0.2 asks 34 %, 0.3 asks 51 %, half a pedal 74 %, 0.6 asks 82 %; the lower c, the less the top of the pedal adds: its slope at a full pedal is k c / (1 + c), and the suite asks for 0.25 at least). Set the toe and the three curves together. Three ready-made sets, paste all four values into [torque_pedal.curve]: Between (the default, halfway between P6b and P6c): curve_toe 1.47, curve_eco 0.365, curve_normal 0.295, curve_power 0.245. P6b (a lively light pedal): curve_toe 1, curve_eco 0.8, curve_normal 0.6, curve_power 0.4. P6c (a soft start, cruising holds near pedal 0.2): curve_toe 2.15, curve_eco 0.165, curve_normal 0.14, curve_power 0.135." },
    { "curve_toe", &p_curve_toe, 1.47f, "The soft start of the pedal's map, the same in every mode: s = (1 + c) p^k / (p^k + c), k this value. 1 = no toe: the ask starts steep, so a lively light pedal (the pedal that holds 80 km/h on the flat with 17 t sits at about 0.08 in Normal). Above 1 the bottom of the pedal asks less and the middle more, so a cruising speed is held with a pedal you can dose (1.47: about 0.13, 2.15: about 0.19); a full pedal is still all of it. Between 1 and 6. Set the toe and the three curves together. Three ready-made sets, paste all four values into [torque_pedal.curve]: Between (the default, halfway between P6b and P6c): curve_toe 1.47, curve_eco 0.365, curve_normal 0.295, curve_power 0.245. P6b (a lively light pedal): curve_toe 1, curve_eco 0.8, curve_normal 0.6, curve_power 0.4. P6c (a soft start, cruising holds near pedal 0.2): curve_toe 2.15, curve_eco 0.165, curve_normal 0.14, curve_power 0.135." },
    { "rise_rate_eco", &p_rise_eco, 1.5f, "Eco: the ask may rise at most this much per second (fraction of the full ask); it falls at once, and a flat pedal is never held back" },
    { "rise_rate_normal", &p_rise_norm, 3.0f, "Normal: the same" },
    { "rise_rate_power", &p_rise_pw, 8.0f, "Power: the same" },
    { "rev_gain", &p_rev_gain, 0.8f, "0..1: how much the pedal follows the engine's revs inside a gear. 0 = the same push at the same speed in any gear (the pull fades as the revs rise: constant power). 1 = a real truck's pedal (the pull holds inside a gear and steps down at an upshift; a gear landed under the torque plateau pulls softly until the revs come up). In between = a mix; the default leans to the real truck. A full pedal is full throttle either way" },
    { "grade_assist", &p_grade_assist, 0.75f, "0..2: how much of the hill the pedal takes out. On a climb the bottom of the pedal's range is raised by this share of the force the hill takes, so the truck does not sink as far at the same pedal (0 = off: the same pedal asks the same share on a hill as on the flat, and the hill slows the truck; 1 = the pedal first gives the hill's force and spreads the rest of its travel over what the truck has left; 2 = twice that). A full pedal is always all the truck can give, on the flat and downhill nothing changes, and it needs the game to tell the truck's weight" },
};

// The pedal values the shaping reads (core/wish.h): handed over at start, when one changes, and at every tick (a sim or a live edit that writes a value
// directly is seen at once; the copy is a handful of floats)
static void pd_sync()
{
    wish_cfg c = { p_curve_eco, p_curve_norm, p_curve_pw, p_curve_toe, p_rise_eco, p_rise_norm, p_rise_pw, p_rev_gain, p_grade_assist };
    wish_configure(&c);
}

// The lock test of the clutch (the engine follows the driveline, the same test LowSpeed uses): the clutch position (0 = closed) under 0.02 is the
// noise floor of the telemetry channel (core/shiftcommon.cpp and core/shiftmodel.cpp use the same 0.02); the engine rpm within 80 of the rpm the
// driveline gives in this gear and speed (about an eighth of a truck's idle). Both are absolute and not derived from the vehicle; a truck whose
// idle is far from 650 rpm would need the second one relative (listed as open).
static const float PK_CLUTCH = 0.02f, PK_SYNC_RPM = 80;

// Why the throttle is, or is not, the demand's this tick (the log's term; WT_* below is the same cut as wish_tick wants it).
enum { PDR_NONE = 0, PDR_WISH_OFF, PDR_OTHER_OWNER, PDR_FRAME, PDR_NO_MODE, PDR_NOT_DRIVING, PDR_NO_MODEL, PDR_NEUTRAL, PDR_HELD, PDR_NO_START_GEAR,
       PDR_STANDING, PDR_LAUNCH, PDR_SHIFT, PDR_NO_FORCE, PDR_ON, PDR_N };
struct pdr_info { const char *name; int level; const char *text; };
static const pdr_info k_pdr[PDR_N] = {
    { "none",           2, "" },
    { "wish_off",       2, "[pedal] wish is 0, or the pedal reads NaN: the throttle is the game's own pedal" },
    { "other_owner",    1, "another owner drives the throttle (cruise, ACC, AEB, Auto Hold): it passes through" },
    { "frame",          2, "no valid frame time (a pause, a hitch, the first tick): it passes through" },
    { "no_mode",        1, "TrueShift has no mode for this vehicle: it passes through" },
    { "not_driving",    1, "engine off, paused, cruise on, crawl or launch control: it passes through" },
    { "no_model",       1, "the vehicle model is not valid (engine, electric, mass): it passes through" },
    { "neutral",        1, "reverse, neutral or no speed reading: it passes through" },
    { "held",           1, "the crawl, a cruise hold or drive-off, Stop&Go or the cruise's own throttle: it passes through" },
    { "no_start_gear",  1, "no start gear known yet: it passes through" },
    { "standing",       1, "standing in a forward gear: the share is shaped for the launch, the throttle passes through" },
    { "launch",         1, "the launch: the share is shaped for it, the throttle passes through" },
    { "shift",          2, "the clutch is not locked or the engine is off the driveline's rpm (a shift, a slip): the throttle passes through, the demand stays" },
    { "no_force",       1, "the gear cannot put the demand into a throttle (over the rev limit, or the model cannot tell the force): it passes through" },
    { "on",             2, "the demand drives the throttle" },
};
// The launch's hand-over (see pedal_wish): the throttle the smooth start ended with, whether it still holds the clutch, and when it ended. LAUNCH_EASE_S is the owner's hand-off hold after the clutch locks
// (LowSpeed LS_HANDOFF_S: TrueShift gets the truck at lock + 0.5 s), the time the engine's throttle has to move from the launch's to the demand's.
static const float LAUNCH_EASE_S = 0.5f;
static int g_launch_open;
static float g_launch_thr;
static double g_launch_end = -1e9;
static int g_pdr_last = PDR_NONE;
static double g_pdr_at = -1e9;   // when the last line of a flicker-prone (level 2) reason was written
static void pd_note(int pr, float pedal, int md, int gear, float v)
{
    if (pr == g_pdr_last) return;
    // This runs at the physics tick, and a clutch reading or an rpm gap hovering at its threshold can swap ON and SHIFT every tick. The flicker-prone
    // reasons (level 2) are written at most every 0.5 s (a shift takes about half a second); a reason that was held back is written on a later tick if it still stands.
    if (k_pdr[pr].level >= 2) {
        if (sim_s() - g_pdr_at < 0.5) return;
        g_pdr_at = sim_s();
    }
    g_pdr_last = pr;
    logv(k_pdr[pr].level, "Pedal: [%s] %s (pedal %.2f, mode %d, gear %d, %.0f km/h)", k_pdr[pr].name, k_pdr[pr].text, pedal, md, gear, v * 3.6f);
}

// Why the demand does not drive the throttle this tick (WT_*, core/wish.h). WT_ON: it does; WT_OFF: a hard pass-through (the pedal goes through untouched
// and the demand resets); WT_SHIFT: the clutch is not locked (a shift, a slip): the throttle goes through, the demand stays so TrueShift's gear choice does
// not see a drop to zero; WT_SHAPE: standing in a forward gear or the smooth start holding the clutch: the share is shaped for the pull-away to read, but the
// throttle goes through (the clutch does the pulling). Reads only what the other modules and the vehicle model say.
// (dt < 0.5 s: a longer tick is a pause or a hitch, nothing to integrate over, and over the 0.3 s after which core/wish.cpp calls a demand stale;
// the crawl's cruise_info fields, hold / sg_hold / feed, are Cruise's own throttle; ts_start_gear(0.3) = the start gear for a 0.3 pedal, the figure
// Creep and LowSpeed use as the launch prior; v > 0.1 m/s (0.4 km/h) is "moving" for the standing test.) *pr = the reason (PDR_*) for the log.
static int passes(float dt, int other, int md, int *gear, float *v, int *pr)
{
    if (other) { *pr = PDR_OTHER_OWNER; return WT_OFF; }
    if (md < 0) { *pr = PDR_NO_MODE; return WT_OFF; }
    if (!(dt > 0 && dt < 0.5f)) { *pr = PDR_FRAME; return WT_OFF; }
    if (!T.engine_on || T.paused || T.cruise > 0.1f || g_lv.lcx) { *pr = PDR_NOT_DRIVING; return WT_OFF; }   // (the crawl running is cr.crawl below: PDR_HELD. The crawl SWITCH on, at 50 km/h, is no reason to take the pedal's map away: promise P4)
    if (!V.eng_ok || V.electric || !V.mass_ok) { *pr = PDR_NO_MODEL; return WT_OFF; }
    *gear = T.gear;
    *v = fabsf(T.speed);
    if (*gear < 1 || !(T.speed == T.speed)) { *pr = PDR_NEUTRAL; return WT_OFF; }   // reverse, neutral
    creep_info cr; cruise_info cu;
    creep_info_get(&cr);
    cruise_info_get(&cu);
    if (cr.crawl || cu.hold || cu.sg_hold || cu.feed > 0) { *pr = PDR_HELD; return WT_OFF; }
    if (!(ts_start_gear(0.3f) > 0)) { *pr = PDR_NO_START_GEAR; return WT_OFF; }
    // standing in a forward gear or the launch: the share is shaped, the throttle passes through
    if (!(*v > 0.1f) || cr.launch) { *pr = cr.launch ? PDR_LAUNCH : PDR_STANDING; return WT_SHAPE; }
    // the clutch is closed and the engine follows the driveline (not slipping, not open, not in a shift)
    if (!(clutch_seen() < PK_CLUTCH) || !(fabsf(T.rpm - vehicle_rpm_at(*gear, *v)) < PK_SYNC_RPM)) { *pr = PDR_SHIFT; return WT_SHIFT; }
    *pr = PDR_ON;
    return WT_ON;
}

int pedal_wish(float p, float dt, int other, float *out)
{
    *out = p;
    pd_sync();
    if (!(p_wish > 0) || !(p == p)) { pd_note(PDR_WISH_OFF, p == p ? p : 0.0f, 0, 0, 0); wish_follow(p); return 0; }
    int md = ts_mode(), g = 0, pr = PDR_NONE;
    float v = 0;
    int why = passes(dt, other, md, &g, &v, &pr);
    // The launch's hand-over (WP-LS): while the smooth start holds the clutch (Creep's law, pr == PDR_LAUNCH) the engine's throttle is the pedal's, as it always was; the demand takes the throttle over
    // when the clutch has closed (WT_ON below), eased in over the owner's hand-off hold (LAUNCH_EASE_S), from the throttle the launch ended with.
    if (pr == PDR_LAUNCH) { g_launch_open = 1; g_launch_thr = p; }
    else if (g_launch_open) { g_launch_open = 0; g_launch_end = sim_s(); }
    if (!wish_tick(p, dt, md, v, g, why)) {   // pass-through, the shaping only, or a shift: the throttle is the pedal's
        // The launch (WT_SHAPE, the smooth start holds the clutch): at the bottom of the pedal (the crawl) the clutch law delivers the torque pedal's
        // DEMAND as a force through the slipping clutch (a force target, not a speed or an rpm target, WP-LS2): the engine's throttle is the demand's at
        // the engine's own rpm, so a small pedal is a gentle crawl force (the grade assist gives it a floor on a climb) and the pedal rises to the launch.
        // Above a share of ~0.4 the engine's throttle stays the pedal's (the launch as before: the rpm aim holds the clutch).
        if (why == WT_SHAPE && pr == PDR_LAUNCH && wish_launch_share(p) < 0.15f) {
            float fd = wish_level_shaped(v);
            float v_rpm = T.rpm > 0 ? vehicle_kmh_at(g, T.rpm) / 3.6f : v;
            float fg = vehicle_force_at(g, v_rpm, 1.0f);
            float gas = fg > 0 && fd >= 0 ? fminf(1.0f, fmaxf(0.0f, fd / fg)) : p;
            g_launch_thr = gas;
            pd_note(PDR_LAUNCH, p, md, g, v);
            *out = gas;
            return 1;
        }
        pd_note(why == WT_ON ? PDR_NO_FORCE : pr, p, md, g, v);   // (WT_ON and no drive: wish_tick found the gear cannot deliver the demand)
        return 0;
    }
    // the throttle that gives this gear what it is asked: F_d over the force it has at full throttle (0..1; 1 where the gear cannot deliver it)
    float fg = vehicle_force_at(g, v, 1.0f), fd = wish_in(g, v);
    float gas = fg > 0 && fd >= 0 ? fminf(1.0f, fmaxf(0.0f, fd / fg)) : NAN;
    if (!(gas == gas)) { pd_note(PDR_NO_FORCE, p, md, g, v); wish_follow(p); return 0; }
    pd_note(PDR_ON, p, md, g, v);
    // After a launch the demand's throttle comes in over the owner's hand-off hold, not at once: at the pedal of 0.5 the launch's throttle was the pedal's 0.50 and the demand's 0.73-0.76 a tick after the
    // clutch closed, a step of 0.24-0.28 in the engine's throttle (package P measured it as an 8-19 % step in force when the start gear is not gear 1). The throttle goes from the launch's own to the
    // demand's linearly over LAUNCH_EASE_S, and only upward; a flat pedal is 1.0 on both sides (promise P1). Nothing else eases: a shift ends on the pedal's throttle and the demand takes over as it always did.
    // Only UP: from = the launch's throttle or the pedal, whichever is more; a demand at or under it (a lift, promise P2) and a pedal pushed harder (a stab, P1) are delivered at once.
    if (!g_launch_open && g_launch_end > 0) {
        const float k = (float)(sim_s() - g_launch_end) / LAUNCH_EASE_S, from = fmaxf(g_launch_thr, p);
        if (k >= 0 && k < 1.0f && gas > from) gas = from + (gas - from) * k;
    }
    *out = gas;
    return 1;
}

static void pd_reset(int level, const char *) { wish_reset(level); g_pdr_last = PDR_NONE; g_pdr_at = -1e9; g_launch_open = 0; g_launch_end = -1e9; }

static char g_status[120] = "off (ini)";
static const char *pd_status() { return g_status; }
static void pd_start()
{
    pd_sync();
    if (p_wish > 0) snprintf(g_status, sizeof g_status, "on: the pedal asks a share of what the truck can pull (Cruise calls it in place of the pedal curves)");
}
static const char *pd_applied(const char *, float) { pd_sync(); return NULL; }

module m_pedal = { "pedal", "Pedal", g_params, sizeof g_params / sizeof g_params[0], pd_start, NULL, NULL, NULL, pd_reset, pd_status, pd_applied, VCM_BOTH };
