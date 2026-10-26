// Pedal: the wish pedal, a drive-by-wire style throttle (off by default, [pedal] wish).
//
// Today the pedal asks a share of the engine's torque curve, so after an upshift the same pedal gives less pull. With the wish pedal
// the driver's pedal is a WISH for power, and the throttle is what delivers it in the gear the truck is in: the same pedal gives the
// same push at the same speed in any gear.
//
// Per tick (pedal_wish, called by Cruise where its old pedal-curve stage sits):
//  1. shape      q = p^exponent(mode), the exponent scaled by the truck's power per tonne (curve_ref_kw_t)
//  2. rise limit q_f may rise at most rise_rate(mode) per second, falls at once; never delayed from blend_from up (kickdown)
//  3. kick       Power only: a fast pedal step from near zero to the middle adds a little to the wish for a moment
//  4. wish       P_wish = q_f * P_ref (ref_now: the power the truck can give at this speed in the best gear the gearbox may use,
//                else the engine's peak power); force F = min(P_wish * eta / v, q_f * F_cap), F_cap = the force the
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

static float p_wish, p_blend, p_kick_rate, p_kick, p_kick_s, p_exp_eco, p_exp_norm, p_exp_pw, p_rise_eco, p_rise_norm, p_rise_pw, p_ref_kwt, p_ref_now;

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
    { "curve_ref_kw_t", &p_ref_kwt, 12, "the exponents above are for a truck with this much engine power per tonne (kW/t); a heavier, slower truck gets a more direct curve (a lower exponent, at most x0.8), a lighter, stronger one a softer one (at most x1.15). 0 = the exponents as they are" },
    { "ref_now", &p_ref_now, 1, "1 = a full wish is the power the truck can give at this speed in the best gear the gearbox may use (the forward gears whose revs lie in the usable window), the same in every gear, so a full pedal asks what the truck can give at this speed and not more. 0 = a full wish is the engine's peak power" },
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
    double on_at;     // sim_s() of the last tick that gave a wish, < 0 = the pedal goes through (TrueShift reads the wish: pedal_wish_force)
    float pref, fcap; // that tick's reference power (W) and force cap (N)
    float truck_k;    // the curve factor for this truck and load (truck_curve), 1 = the exponents as they are
} S = { NAN, 0, 0, 0, 0, 0, -1, -1, 0, 0, 1 };

// Hand over to the pass-through: the state follows the pedal, so the wish starts from where the pedal is, not from zero.
static void follow(float p)
{
    S.q_f = S.q = p == p ? p : 0;
    S.fast = 0;
    S.kick_age = -1;
    S.on_at = -1;
}

// The engine's peak power (W) over its torque curve, up to the rpm limit.
static float peak_power()
{
    float best = 0;
    for (float rpm = 100; rpm <= V.eng.rpm_limit; rpm += 50)
        best = fmaxf(best, V.eng.torque * curve_factor(&V.eng, rpm) * rpm * 0.10472f);
    return best;
}

// ref_now: the power (W, at the engine like peak_power) the truck can deliver at speed v, the best forward gear whose revs lie in the
// usable window at full throttle. The window is the score-based gear choice's (core/gearscore.h): from idle + 60 % of the way up to the
// torque plateau (at least idle + 150) to the top of the power range or 93 % of the limit. Never the gear of now, so a shift does not
// move it; 0 = no gear in the window.
static float power_now(float v)
{
    float idle = ts_idle_rpm(), lo = fmaxf(idle + 0.6f * fmaxf(0.0f, V.plateau - idle), idle + 150);
    float hi = fminf(V.eng.range[R_POWER][1], 0.93f * V.eng.rpm_limit), best = 0;
    for (int g = 1; g <= T.fwd_gears; g++) {
        float r = vehicle_rpm_at(g, v);
        if (r >= lo && r <= hi) best = fmaxf(best, vehicle_force_at(g, v, 1.0f) * v);
    }
    return V.eta > 0 ? best / V.eta : 0;
}

// Step 4's reference at speed v: the power (W) of a full wish (ref_now, else the peak power) and the force cap (N) of start gear sg; 0 = none.
static void wish_ref(float v, int sg, float *pref, float *fcap)
{
    *pref = p_ref_now > 0 ? power_now(v) : 0;
    if (!(*pref > 0)) *pref = peak_power();   // no gear in the window (or ref_now 0): the peak power
    *fcap = sg > 0 ? vehicle_force_at(sg, vehicle_kmh_at(sg, V.plateau) / 3.6f, 1.0f) : 0;
}

// The curve factor per truck: power per tonne against curve_ref_kw_t (sqrt, x0.8 ... x1.15). Taken only at a standstill, so the pedal
// never changes under the foot while driving; an unknown mass (the 20 t fallback), an electric motor or engine data that does not look
// sane keep the exponents as they are. One log line whenever it changes (another truck, a trailer, a load).
static void truck_curve()
{
    if (fabsf(T.speed) > 0.3f) return;
    float pw = peak_power(), k = 1.0f;
    if (p_ref_kwt > 0 && pw > 0 && V.mass_ok && !V.electric && V.eng_ok && V.mass > 1000)
        k = fminf(1.15f, fmaxf(0.8f, sqrtf(pw / V.mass / p_ref_kwt)));
    if (fabsf(k - S.truck_k) > 0.005f)
        logline("Pedal: curve x%.2f for this truck (%.1f kW/t: %.0f kW, %.1f t%s)", k, pw > 0 && V.mass > 0 ? pw / V.mass : 0.0f, pw / 1000,
                V.mass / 1000, V.mass_ok && !V.electric && V.eng_ok ? "" : ", not known: as it is");
    S.truck_k = k;
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
    truck_curve();
    p = fminf(1.0f, fmaxf(0.0f, p));
    int md = ts_mode(), g = 0, sg = 0;
    float v = 0;
    if (passes(dt, other, md, &g, &v, &sg)) { follow(p); return 0; }

    // 1, 2: shape, rise limit (the 'delay'). A flat or near-flat pedal is kickdown: never held back
    float expo = md == MODE_ECO ? p_exp_eco : md == MODE_POWER ? p_exp_pw : p_exp_norm;
    // per truck: the engine's power per tonne against curve_ref_kw_t, sqrt, x0.8 ... x1.15 (a 40 t truck with 350 kW: x0.85, 17 t: x1.15)
    expo *= S.truck_k;   // set at a standstill only (truck_curve)
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
    float fg = vehicle_force_at(g, v, 1.0f), pref, fcap;
    wish_ref(v, sg, &pref, &fcap);
    if (!(fg > 0) || !(pref > 0) || !(fcap > 0) || !(V.eta > 0)) { follow(p); return 0; }   // e.g. over the rev limit: the old path, as a pass-through
    float f = fminf(S.q * pref * V.eta / v, S.q * fcap);
    float gas = fminf(1.0f, fmaxf(0.0f, f / fg));
    S.w = gas;

    // 6: from blend_from up the wish fades into the plain pedal (never down from the wish: a gear that needs more than the pedal
    // keeps asking for it), and a flat pedal is full throttle
    if (p > bl) gas += fmaxf(0.0f, p - gas) * (p - bl) / (1.0f - bl);
    if (p >= 1.0f) gas = 1.0f;
    if (!(gas == gas)) { follow(p); return 0; }
    *out = gas;
    S.on_at = sim_s(); S.pref = pref; S.fcap = fcap;
    return 1;
}

float pedal_wish_force(float v)
{
    double now = sim_s();
    if (!(p_wish > 0) || S.on_at < 0 || now < S.on_at || now - S.on_at > 0.3 || !(V.eta > 0)) return -1;
    return fminf(S.q * S.pref * V.eta / fmaxf(v, 0.5f), S.q * S.fcap);
}

float pedal_wish_full(float v)
{
    if (!V.eng_ok || V.electric || !(V.eta > 0)) return 0;
    float pref, fcap;
    wish_ref(v, ts_start_gear(0.3f), &pref, &fcap);
    if (!(pref > 0) || !(fcap > 0)) return 0;
    return fminf(pref * V.eta / fmaxf(v, 0.5f), fcap);
}

// Another truck, a trailer or a load: the exponents as they are until the next standstill sets the factor for it.
static void pd_reset(int level, const char *) { S.prev = NAN; follow(0); if (level >= RESET_LOAD) S.truck_k = 1; }

static char g_status[120] = "off (ini)";
static const char *pd_status() { return g_status; }
static void pd_start()
{
    if (p_wish > 0) snprintf(g_status, sizeof g_status, "on: the pedal is a wish for power (Cruise calls it in place of the pedal curves)");
}

module m_pedal = { "pedal", "Pedal", g_params, sizeof g_params / sizeof g_params[0], pd_start, NULL, NULL, NULL, pd_reset, pd_status };
