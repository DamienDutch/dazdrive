// Pedal: the wish pedal, a drive-by-wire style throttle (off by default, [pedal] wish).
//
// Today the pedal asks a share of the engine's torque curve, so after an upshift the same pedal gives less pull. With the wish pedal
// the driver's pedal is a WISH for power, and the throttle is what delivers it in the gear the truck is in: the same pedal gives the
// same push at the same speed in any gear.
//
// Per tick (pedal_wish, called by Cruise where its old pedal-curve stage sits):
//  1. shape      q = p^exponent(mode)
//  2. rise limit q_f may rise at most rise_rate(mode) per second, falls at once; never delayed from blend_from up (kickdown)
//  3. kick       Power only: a fast pedal step from near zero to the middle adds a little to the wish for a moment
//  4. wish       P_wish = q_f * P_ref (the engine's peak power); force F = min(P_wish * eta / v, q_f * F_cap), F_cap = the force the
//                start gear pulls at the speed of its torque plateau (what the truck itself can put on the road at walking pace)
//  5. throttle   gas = F / (the force this gear gives at full throttle at this speed), 0..1 (1 where the gear cannot deliver the wish)
//  6. blend      above blend_from the wish fades into the plain pedal, and at a flat pedal the gas is 1 (kickdown)
// Everything else leaves the pedal alone (pass-through): the clutch slipping or open, the low-speed range, reverse and neutral, an
// electric or manual box, a vehicle model that is not valid, and whatever other owner drives the throttle (cruise, ACC, AEB, Auto Hold).
#include "../core/suite.h"
#include "../core/game.h"
#include "../core/vehicle.h"
#include <math.h>
#include <string.h>

static float p_wish, p_blend, p_kick_rate, p_kick, p_kick_s, p_exp_eco, p_exp_norm, p_exp_pw, p_rise_eco, p_rise_norm, p_rise_pw;

static param g_params[] = {
    { "wish", &p_wish, 0, "1 = the pedal is a wish for power: the throttle is what delivers it in the gear the truck is in, so the same pedal gives the same push at the same speed in any gear. 0 = the pedal asks a share of the engine's torque, as before. Not with the cruise on, not while the clutch slips, not at pulling-away speed, not in reverse" },
    { "blend_from", &p_blend, 0.85f, "above this pedal the wish fades into the plain pedal, and a flat pedal is full throttle (kickdown)" },
    { "kick_rate", &p_kick_rate, 1.5f, "Power: a pedal that rises faster than this (fraction per second), from nearly released to the middle, gets a little extra for a moment (kick)" },
    { "kick", &p_kick, 0.15f, "Power: the extra wish of that kick (a fraction of the full wish; the total never exceeds 1). 0 = no kick" },
    { "kick_s", &p_kick_s, 0.4f, "Power: how long the kick lasts (seconds), fading out" },
    { "exponent_eco", &p_exp_eco, 1.1f, "Eco: the wish is the pedal to this power (1 = linear; above 1: half pedal asks less than half the power)" },
    { "exponent_normal", &p_exp_norm, 1.0f, "Normal: the same" },
    { "exponent_power", &p_exp_pw, 0.95f, "Power: the same (below 1: front-loaded)" },
    { "rise_rate_eco", &p_rise_eco, 1.5f, "Eco: the wish may rise at most this much per second (fraction of the full wish); it falls at once, and from blend_from up it is never held back" },
    { "rise_rate_normal", &p_rise_norm, 3.0f, "Normal: the same" },
    { "rise_rate_power", &p_rise_pw, 8.0f, "Power: the same" },
};

// The levels of the kick's gesture (a pedal that starts almost released and stops in the middle) and the lock test of the clutch (the
// engine follows the driveline, the same test LowSpeed uses).
static const float PK_FROM = 0.15f, PK_END_LO = 0.30f, PK_END_HI = 0.70f;
static const float PK_CLUTCH = 0.02f, PK_SYNC_RPM = 80;

static struct {
    float prev;       // the pedal of the last tick (NAN = none yet)
    float q_f;        // the shaped wish after the rise limit (before the kick)
    float q;          // the wish this tick after the kick (what the force comes from; sims read it)
    float w;          // the throttle of the wish before the blend (sims read it)
    int fast;         // the pedal is rising faster than kick_rate
    float from;       // where that fast rise started
    float kick_age;   // seconds since the kick started, < 0 = none
} S = { NAN, 0, 0, 0, 0, 0, -1 };

// Hand over to the pass-through: the state follows the pedal, so the wish starts from where the pedal is, not from zero.
static void follow(float p)
{
    S.q_f = S.q = p == p ? p : 0;
    S.fast = 0;
    S.kick_age = -1;
}

// The engine's peak power (W) over its torque curve, up to the rpm limit.
static float peak_power()
{
    float best = 0;
    for (float rpm = 100; rpm <= V.eng.rpm_limit; rpm += 50)
        best = fmaxf(best, V.eng.torque * curve_factor(&V.eng, rpm) * rpm * 0.10472f);
    return best;
}

// The extra wish of the kick now: full at the start, fading out to nothing at kick_s.
static float kick_now()
{
    if (S.kick_age < 0 || !(p_kick > 0) || !(p_kick_s > 0)) return 0;
    float e3 = expf(-3.0f), k = (expf(-3.0f * S.kick_age / p_kick_s) - e3) / (1.0f - e3);
    return p_kick * fmaxf(k, 0.0f);
}

// 1 = the pedal goes through untouched. Reads only what the other modules and the vehicle model say.
static int passes(float dt, int other, int md, int *gear, float *v, int *sg)
{
    if (other || md < 0 || !(dt > 0 && dt < 0.5f)) return 1;
    if (!T.engine_on || T.paused || T.cruise > 0.1f || g_crawl_on || g_lcx) return 1;
    if (!V.eng_ok || V.electric || !V.mass_ok) return 1;
    *gear = T.gear;
    *v = fabsf(T.speed);
    if (*gear < 1 || !(*v > 0.1f) || !(T.speed == T.speed)) return 1;   // reverse, neutral, standing
    creep_info cr; cruise_info cu;
    creep_info_get(&cr);
    cruise_info_get(&cu);
    if (cr.crawl || cr.launch || cu.hold || cu.sg_hold || cu.feed > 0) return 1;
    // below the low-speed range: the start gear's own top (where it reaches the end of the game's low-gear shift window); no number of ours
    *sg = ts_start_gear(0.3f);
    float top = *sg > 0 ? vehicle_kmh_at(*sg, V.eng.range[R_LOW][1]) : 0;
    if (!(top > 0) || *v * 3.6f < top) return 1;
    // the clutch is closed and the engine follows the driveline (not slipping, not open, not in a shift)
    if (!(clutch_seen() < PK_CLUTCH) || !(fabsf(T.rpm - vehicle_rpm_at(*gear, *v)) < PK_SYNC_RPM)) return 1;
    return 0;
}

int pedal_wish(float p, float dt, int other, float *out)
{
    float pp = S.prev == S.prev ? S.prev : p;
    *out = p;
    S.prev = p;
    if (!(p_wish > 0) || !(p == p)) { follow(p); return 0; }
    p = fminf(1.0f, fmaxf(0.0f, p));
    int md = ts_mode(), g = 0, sg = 0;
    float v = 0;
    if (passes(dt, other, md, &g, &v, &sg)) { follow(p); return 0; }

    // 1, 2: shape, rise limit (the 'delay'). A flat or near-flat pedal is kickdown: never held back
    float expo = md == MODE_ECO ? p_exp_eco : md == MODE_POWER ? p_exp_pw : p_exp_norm;
    float rise = md == MODE_ECO ? p_rise_eco : md == MODE_POWER ? p_rise_pw : p_rise_norm;
    float bl = fminf(0.999f, fmaxf(0.0f, p_blend));
    float q = powf(p, expo > 0 ? expo : 1.0f);
    S.q_f = p >= bl || p >= 1.0f || q <= S.q_f ? q : fminf(q, S.q_f + fmaxf(rise, 0.0f) * dt);

    // 3: the kick. A fast rise (over kick_rate) that starts almost released and ends in the middle; Power only. It ends when the rise
    // slows below kick_rate, and nothing starts from a pedal that went back under where it began
    if (S.kick_age >= 0) { S.kick_age += dt; if (S.kick_age >= p_kick_s || p < PK_FROM || md != MODE_POWER) S.kick_age = -1; }
    if ((p - pp) / dt > p_kick_rate) {
        if (!S.fast) { S.fast = 1; S.from = pp; }
    } else if (S.fast) {
        S.fast = 0;
        if (md == MODE_POWER && S.from < PK_FROM && p >= PK_END_LO && p <= PK_END_HI) S.kick_age = 0;
    }
    S.q = fminf(1.0f, S.q_f + kick_now());

    // 4, 5: the wish as a force at this speed, then the throttle that gives it in this gear
    float fg = vehicle_force_at(g, v, 1.0f), pref = peak_power();
    float fcap = sg > 0 ? vehicle_force_at(sg, vehicle_kmh_at(sg, V.plateau) / 3.6f, 1.0f) : 0;
    if (!(fg > 0) || !(pref > 0) || !(fcap > 0) || !(V.eta > 0)) return 0;
    float f = fminf(S.q * pref * V.eta / v, S.q * fcap);
    float gas = fminf(1.0f, fmaxf(0.0f, f / fg));
    S.w = gas;

    // 6: from blend_from up the wish fades into the plain pedal (never down from the wish: a gear that needs more than the pedal
    // keeps asking for it), and a flat pedal is full throttle
    if (p > bl) gas += fmaxf(0.0f, p - gas) * (p - bl) / (1.0f - bl);
    if (p >= 1.0f) gas = 1.0f;
    if (!(gas == gas)) return 0;
    *out = gas;
    return 1;
}

static void pd_reset(int, const char *) { S.prev = NAN; follow(0); }

static char g_status[120] = "off (ini)";
static const char *pd_status() { return g_status; }
static void pd_start()
{
    if (p_wish > 0) snprintf(g_status, sizeof g_status, "on: the pedal is a wish for power (Cruise calls it in place of the pedal curves)");
}

module m_pedal = { "pedal", "Pedal", g_params, sizeof g_params / sizeof g_params[0], pd_start, NULL, NULL, NULL, pd_reset, pd_status };
