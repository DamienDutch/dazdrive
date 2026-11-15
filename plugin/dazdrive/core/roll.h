// How far the vehicle leans, and what that allows in a bend (the maths of the roll learning; core/rolllearn.cpp feeds and stores it).
//
// The bends are planned with a sideways acceleration (Cruise's curve_lat, the route profile's bend caps, bend_now). Instead of one fixed number
// for every truck, the suite measures how much each truck and trailer LEANS per m/s2 of sideways acceleration (the roll gain, degrees per m/s2)
// and allows what keeps the lean at a fixed share of the lean where the vehicle is at the edge:
//
//     limit     = roll_budget / roll_gain            the sideways acceleration at which the lean reaches the edge
//     allowance = edge_share x limit                 how close to the edge the bends are driven (one share, every drive mode)
//
// A tall or soft load leans more per m/s2, so its allowance is lower; a stiff, low one rises, never over the comfort cap. The truck and the
// trailer each have their own gain and the larger gain (the smaller limit) rules.
//
// The gain comes from one bend at a time: the lean (the placement's roll, truck and trailer) is regressed on the sideways acceleration
// over the bend. The slope is the gain; the offset absorbs the road's bank (camber), which does not follow the acceleration. A bend only
// counts when the fit is clean (correlated, a small error against the slope, a real range of acceleration), so a banked ramp or a bump
// is left out by the data itself. Little data moves the allowance little (confidence), more lean than expected moves it quickly.
#pragma once
#include <math.h>
#include <string.h>

// [cruise] ini knobs (the params table in modules/cruise.cpp points here)
inline float g_roll_learn = 1;       // roll_learn: 1 = the bends' allowance follows the learned lean; 0 = curve_lat as set, nothing learned or stored
inline float g_roll_edge = 0.7f;     // roll_edge_share: the share of the sideways acceleration at the edge that bends are driven at
inline float g_roll_budget = 6.0f;   // roll_budget_deg: the lean (degrees) at which a vehicle is at the edge

// ---- constants (named, in one place; none of them is a vehicle figure: they say how much evidence is enough)
static const float ROLL_SIGN = -1.0f;        // the lean (outward = +) from the placement's roll: the logged rides give roll = -0.1 .. -0.5 deg per m/s2 of left-turn acceleration
static const float ROLL_CONF_N = 6.0f;       // bends for full confidence
static const float ROLL_DEV_MAX = 1.0f;      // a spread (running mean of |bend gain / gain - 1|) this large = no confidence (real bends scatter by 30-40 %)
static const float ROLL_DEV_PLAN = 0.5f;     // the gain the allowance uses is the estimate plus this share of its spread: an uneven leaner is planned for its leaning
static const float ROLL_REACH = 1.4f;        // the allowance never goes beyond this x the highest acceleration the gain was measured in (a gain says little beyond its range)
static const float ROLL_UP_RATE = 0.03f;     // m/s2 per second, the published allowance rises this slowly ...
static const float ROLL_DOWN_RATE = 0.3f;    // ... and falls this quickly
static const float ROLL_G_LO = 0.03f, ROLL_G_HI = 6.0f;   // a bend's gain outside this (deg per m/s2) is not a lean
static const float ROLL_MASS_TOL = 0.12f;    // a load that differs more than this from the slot's is another combination

struct roll_body { float g = 0, dev = 0, n = 0; };   // one body (truck or trailer): gain deg per m/s2, spread, bends counted (weighted)
struct roll_slot {                                   // one combination of a truck: alone (0) or with its trailer (1) at a load
    roll_body b[2];                                  // [0] the truck, [1] the trailer (slot 1 only)
    float max_a = 0;                                 // the highest peak acceleration of a bend that counted (m/s2)
    float mass = 0;                                  // kg, the load the gains belong to
    int tw = 0;                                      // trailer wheels (config), slot 1
    int has_trailer = 0;
    float edge = 1;                                  // x the budget: the rollover guard firing takes it down for the rest of the combination
    int dirty = 0;
};

// ---- the fit of one bend: lean y (deg) on the acceleration x (m/s2), time-weighted
struct roll_fit {
    double w = 0, wx = 0, wy = 0, wxx = 0, wxy = 0, wyy = 0;
    float peak = 0, xmin = 1e9f;
    void clear() { *this = roll_fit(); }
    void add(float dt, float x, float y)
    {
        w += dt; wx += dt * x; wy += dt * y; wxx += dt * x * x; wxy += dt * x * y; wyy += dt * y * y;
        if (x > peak) peak = x;
        if (x < xmin) xmin = x;
    }
};

// The slope (gain), the offset (the bank, deg), the correlation and the standard error relative to the slope, for a bend that is long
// enough, reaches far enough and spans a real range. tau = the seconds per independent sample (the filter's time constant).
// Returns 1 when the bend counts.
inline int roll_fit_eval(const roll_fit &f, float *slope, float *offset, float *r, float *rel_se)
{
    const float tau = 0.25f;
    if (slope) *slope = 0;
    if (offset) *offset = 0;
    if (r) *r = 0;
    if (rel_se) *rel_se = 9.0f;
    if (f.w < 2.0 || f.peak < 1.2f || f.peak - f.xmin < 0.8f) return 0;
    double sxx = f.wxx - f.wx * f.wx / f.w, sxy = f.wxy - f.wx * f.wy / f.w, syy = f.wyy - f.wy * f.wy / f.w;
    if (!(sxx > 1e-6) || !(syy > 1e-9)) return 0;
    double sl = sxy / sxx, rr = sxy / sqrt(sxx * syy);
    double sse = syy - sl * sxy;
    if (sse < 0) sse = 0;
    double neff = f.w / tau;
    if (neff < 4) return 0;
    // sse / w = the residual variance of a sample, sxx / w = the variance of x: the slope's error = sqrt(s2 / (n_eff x var_x))
    double se = sqrt((sse / f.w) * (neff / (neff - 2)) / (neff * (sxx / f.w)));
    if (offset) *offset = (float)((f.wy - sl * f.wx) / f.w);
    if (slope) *slope = (float)sl;
    if (r) *r = (float)rr;
    if (rel_se) *rel_se = sl > 1e-6 ? (float)(se / sl) : 9.0f;
    return sl >= ROLL_G_LO && sl <= ROLL_G_HI && rr >= 0.6 && se <= 0.25 * sl;
}

// A bend's gain into a body's estimate. The first one is taken as it is (with a wide spread); later ones move it by 1/(n+1), at least 0.12,
// and at least 0.35 when the bend leaned MORE than the estimate (the allowance should come down quickly); one wild bend is clamped to 0.5-2x.
inline void roll_body_add(roll_body &b, float x)
{
    x = fminf(fmaxf(x, ROLL_G_LO), ROLL_G_HI);
    if (b.n <= 0) { b.g = x; b.n = 1; b.dev = 0.15f; return; }
    float rel = fminf(fabsf(x / b.g - 1), 1.0f);
    float xc = fminf(fmaxf(x, 0.5f * b.g), 2.0f * b.g);
    float a = fmaxf(1.0f / (b.n + 1.0f), x > b.g ? 0.35f : 0.12f);
    b.g += a * (xc - b.g);
    b.dev += 0.25f * (rel - b.dev);
    b.n = fminf(b.n + 1.0f, 40.0f);
}
inline float roll_body_conf(const roll_body &b)
{
    if (b.n <= 0) return 0;
    return fminf(1.0f, b.n / ROLL_CONF_N) * fmaxf(0.0f, 1.0f - b.dev / ROLL_DEV_MAX);
}

inline float roll_body_plan(const roll_body &b) { return b.g * (1.0f + ROLL_DEV_PLAN * fminf(b.dev, 0.6f)); }   // the gain the allowance is planned with

// What a slot says: the sideways acceleration at the edge (m/s2) and the confidence (0..1). 0 when the truck's gain is not known.
// Slot 1 without a trailer gain yet (the channel never delivered) counts at half confidence: the trailer is the one that matters.
inline int roll_slot_limit(const roll_slot &s, float budget_deg, float *lim, float *conf)
{
    if (!(s.b[0].n > 0) || !(s.b[0].g > 0)) return 0;
    float l = budget_deg * s.edge / roll_body_plan(s.b[0]), c = roll_body_conf(s.b[0]);
    if (s.has_trailer) {
        if (s.b[1].n > 0 && s.b[1].g > 0) {
            l = fminf(l, budget_deg * s.edge / roll_body_plan(s.b[1]));
            c = fminf(c, roll_body_conf(s.b[1]));
        } else c *= 0.5f;
    }
    if (lim) *lim = l;
    if (conf) *conf = c;
    return c > 0;
}

// ---- the published state
struct roll_state {
    const roll_slot *cur = NULL;     // the slot of the combination now (NULL = none: the start value stands)
    double t = 0;                    // sim time of the last frame (the slew's clock)
    float out = NAN;                 // the allowance published (slewed), NAN = not learned
    double out_at = 0;
    float last_start = NAN, last_cap = NAN;   // what the last caller asked with (for the log)
    float rb_out = 1.0f;             // the roundabout's entry-speed factor published (slewed), 1 = nothing learned (WP-SP2)
    double rb_at = 0;
};
inline roll_state RL;

// The learned target for a start value and a cap, before the slew: NAN = nothing learned (use start). *conf out, *lim out.
inline float roll_target(const roll_slot *s, float start, float cap, float *conf, float *lim)
{
    float l = 0, c = 0;
    if (!s || !(g_roll_learn > 0) || !roll_slot_limit(*s, g_roll_budget, &l, &c)) return NAN;
    float tgt = g_roll_edge * l;
    float reach = fmaxf(start, ROLL_REACH * s->max_a);
    tgt = fminf(tgt, fminf(cap, reach));
    float w = tgt < start ? fminf(1.0f, 2.0f * c + 0.2f) : c;    // down on evidence of more lean is quick, up needs the confidence
    if (conf) *conf = c;
    if (lim) *lim = l;
    return start + (tgt - start) * w;
}

// The sideways acceleration the bends are planned with (m/s2 at about 92 km/h): `start` (curve_lat, the value the player set) until a lean
// has been measured, then it moves toward the learned allowance, at most `cap`. Rising is slow (ROLL_UP_RATE), falling is not.
inline float roll_bend_lat(float start, float cap)
{
    RL.last_start = start; RL.last_cap = cap;
    float des = roll_target(RL.cur, start, cap, NULL, NULL);
    if (!(des == des)) { RL.out = NAN; return start; }
    if (!(RL.out == RL.out)) { RL.out = start; RL.out_at = RL.t; }
    double dt = RL.t - RL.out_at;
    RL.out_at = RL.t;
    if (!(dt > 0)) dt = 0;
    if (dt > 0.5) dt = 0.5;
    if (des > RL.out) RL.out = fminf(des, RL.out + ROLL_UP_RATE * (float)dt);
    else RL.out = fmaxf(des, RL.out - ROLL_DOWN_RATE * (float)dt);
    return RL.out;
}

// For the traces and the logs
inline float roll_gain(int body) { return RL.cur && body >= 0 && body < 2 && RL.cur->b[body].n > 0 ? RL.cur->b[body].g : NAN; }
inline float roll_allow_now() { return RL.out; }

// ---- the rollover guard's scale (WP-LA4). The guard (core/rsc.cpp) looks for a growing swing and its thresholds are sideways accelerations (3.0 .. 8.5 m/s2).
// They were found on ONE truck, the Renault the 24-10 rollover happened with: a 35.6 t combination with a trailer. What that truck measured, from the
// roll learning in roll_gain.txt: the trailer leans 0.829 deg per m/s2 (spread 0.31) and the trailer rules, so the plan gain is 0.829 x (1 + 0.5 x 0.31) = 0.957
// and the lean reaches the budget (6 deg) at 6 / 0.957 = 6.27 m/s2. Every number of the guard is a share of that limit (3.0 = 0.48, 3.5 = 0.56, 4.0 = 0.64,
// 5.0 = 0.80, in rsc.cpp). A vehicle that leans more per m/s2 (a tall or soft load, a trailer that rolls: gain up, limit down) reaches the edge at a smaller
// sideways acceleration, so the same shares of ITS limit are smaller numbers: the guard's thresholds scale with limit / 6.27.
//
// Only down. A stiffer vehicle (limit above 6.27) keeps today's numbers: its swing would be caught later with looser ones, and "never later than today" is the
// rule for this guard (the 24-10 rollover and the 25-10 runaways are rows). Floor: 0.75. The lane-change row of sim_cruise (0.5 .. 2.2 m/s2 at 0.25 Hz, level 0 by
// design) must stay under the lowest lobe the guard counts, and 3.0 x 0.75 = 2.25 is just above its 2.2. With the confidence of the lean estimate: one bend of a
// tall load moves the scale half way (the same weight a downward move of the bend allowance gets in roll_target: min(1, 2 c + 0.2)); nothing learned = exactly 1.
static const float ROLL_GUARD_REF = 6.27f;      // m/s2, the limit of the truck the guard's numbers were found on (above)
static const float ROLL_GUARD_KMIN = 0.75f;     // the floor of the scale (above)
static const float ROLL_GUARD_UP = 0.013f;      // per second: the scale rises this slowly = ROLL_UP_RATE (0.03 m/s2 per s) on the 2.325 m/s2 start value; it falls at once
inline volatile float g_roll_guard_k = 1.0f;    // the published scale (<= 1): rsc_step multiplies its thresholds with it. Written by roll_frame, read by the guard

inline float roll_guard_k_target(const roll_slot *s)
{
    float lim = 0, c = 0;
    if (!s || !(g_roll_learn > 0) || !roll_slot_limit(*s, g_roll_budget, &lim, &c)) return 1.0f;
    float k = fminf(1.0f, lim / ROLL_GUARD_REF);
    k = 1.0f + (k - 1.0f) * fminf(1.0f, 2.0f * c + 0.2f);
    return fmaxf(ROLL_GUARD_KMIN, k);
}
// One step: down at once, up slowly (dt in s). Returns the published scale.
inline float roll_guard_k_step(const roll_slot *s, float dt)
{
    float tgt = roll_guard_k_target(s), cur = g_roll_guard_k;
    if (!(cur == cur) || cur > 1.0f) cur = 1.0f;
    if (tgt < cur) cur = tgt;
    else if (dt > 0) cur = fminf(tgt, cur + ROLL_GUARD_UP * fminf(dt, 0.5f));
    g_roll_guard_k = cur;
    return cur;
}

// ---- the roundabout's entry speed from how far THIS truck at THIS load leans (WP-SP2; the owner, 54.4: "empty a bit faster, a real 40 t softer, 25 is fine now").
// v = sqrt(a_allowed x r). The reference: the ini's roundabout speed (25 km/h) is what the reference truck, the one the guard's numbers were found on (limit 6.27 m/s2,
// ROLL_GUARD_REF), is driven through at: its allowance is the bends' edge share of that limit (a_ref = g_roll_edge x 6.27) and the roundabout's effective radius is the
// one that makes 25 km/h come out at it (r = v_ref^2 / a_ref). For another truck the allowance is the same share of ITS learned limit, so
//     v = v_ref x sqrt(limit_learned / ROLL_GUARD_REF)
// (the share and the radius cancel; a 40 t that leans more, limit 5.0, gets 22 km/h; an empty one that leans little gets more). The ratio is bounded by the roll
// learning's own limits: the allowance never goes beyond ROLL_REACH (1.4) x what the gain was measured in and the guard's scale never below ROLL_GUARD_KMIN (0.75)
// of the reference, so the factor stays within sqrt(0.75) = 0.87 .. sqrt(1.4) = 1.18 (21.7 .. 29.6 km/h from 25). It follows the confidence like the bends' target (down
// on evidence of more lean is quick, up needs the confidence). Nothing learned = exactly 1 (the ini's speed). The published factor moves slowly up and quickly down:
// ROLL_UP_RATE and ROLL_DOWN_RATE are rates of an allowance of ~3 m/s2 (1 % and 10 % per second), and the speed goes with its root, so half of that: 0.5 % and 5 % per
// second. dt in s (the caller's frame).
inline float roll_rb_factor(float dt)
{
    float lim = 0, conf = 0, tgt = 1.0f;
    if (RL.cur && g_roll_learn > 0 && roll_slot_limit(*RL.cur, g_roll_budget, &lim, &conf)) {
        float r = lim / ROLL_GUARD_REF;
        r = fminf(fmaxf(r, ROLL_GUARD_KMIN), ROLL_REACH);
        const float f = sqrtf(r), w = f < 1.0f ? fminf(1.0f, 2.0f * conf + 0.2f) : conf;
        tgt = 1.0f + (f - 1.0f) * w;
    }
    const float up = 0.005f, dn = 0.05f;
    if (!(RL.rb_out == RL.rb_out)) RL.rb_out = 1.0f;
    if (dt > 0.5f) dt = 0.5f;
    if (dt > 0) RL.rb_out = tgt > RL.rb_out ? fminf(tgt, RL.rb_out + up * dt) : fmaxf(tgt, RL.rb_out - dn * dt);
    return RL.rb_out;
}

// core/rolllearn.cpp (called by the core, not by a module)
void roll_frame();               // every frame: the bends are watched, the gains learned, stored, logged
void roll_reset(int level);      // RESET_*: LOAD = the combination is judged again at the new mass, HARD = unloaded (stored first)
void roll_stop();                // shutdown: store what is not stored yet
void roll_prime(int has_trailer, float g_truck, float g_trailer, float n, float max_a, float mass);   // sims: a slot as if learned
