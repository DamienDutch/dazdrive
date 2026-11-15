// Layer 3 (core/gearchoice.h): the gear. Hard limits first (the caller's mask, the engine's own rpm window, the
// wish within the intent's per-band share), then ONE score over the allowed gears, lowest wins, and every term in
// seconds of drive time over a common horizon. See the header for the terms.
#include "gearchoice.h"
#include "driver.h"
#include <math.h>
#include <string.h>
#include <algorithm>

static float gc_clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// The common horizon the terms are measured over, and the current gear's tie-break bonus (s): the band cost already
// sits in seconds, so a stay bonus of a few hundredths only breaks near-equal scores in favour of staying put.
static const float GC_HORIZON_SPEED = 20.0f;  // the speed the horizon may reach at most (m/s): a launch's step, not the wish's terminal speed
static const float GC_STAY = 0.05f;
// The torque pedal compensates at low revs, so with its wish on the landing gear needs no floor above the smooth-running limit as long as it delivers the
// wish: a demand within a tenth of the force a gear gives is not yet one it fails. The step from "delivers" to "cannot deliver" is a ramp over this share
// of the gear's force (a demand 1 % over the force costs a tenth of the shift, not all of it): a wall here made a gear 1 % over lose to a gear that has
// to shift again seconds later (1a0e841). The same 10 % the run shares (intent.cpp RUN_SLACK) give the gear we are in.
static const float GC_SHARE_BAND = 0.10f;

const char *gc_band_name(int band)
{
    switch (band) {
    case PT_NOGO_LO: return "nogo-lo";
    case PT_LUG: return "lug";
    case PT_ECO: return "eco";
    case PT_POW: return "pow";
    case PT_TMP: return "tmp";
    case PT_NOGO_HI: return "nogo-hi";
    }
    return "?";
}

// The crossover of gear g to g + 1 at the speed v (bands_cross with the step of the two ratios); the top gear, a step under 1 and an
// unknown ratio give the cap (the power band runs to the end).
// bands_cross walks the torque curve in steps of 5 rpm (two curve reads each) and was the hot spot of the choice (nine tenths of the path's setup, K1 round 14). Its answer depends on the engine's
// curve, the bands' P2 and cap and the step of the two ratios, and the step is the same at every speed (the speed cancels in the ratio of the two rpm): it is read at 1 m/s, so the same truck asks the
// same question every time, and the answer is remembered until the curve, the bands or the ratios change.
struct gc_cross_memo { float step, p2, cap, val; };
static gc_cross_memo s_cross_memo[GC_MAX + 2];
static decltype(V.eng.curve) s_cross_curve; static int s_cross_curve_n = -1;   // (the engine's own curve type: one size, one place)
float gc_cross(const eng_bands *b, int g, int n, float v)
{
    if (!b->ok || g < 1 || g >= n) return b->cap;
    float r0 = vehicle_rpm_at(g, v), r1 = vehicle_rpm_at(g + 1, v);
    if (!(r0 > 1) || !(r1 > 1)) return b->cap;
    if (g > GC_MAX) return bands_cross(&V.eng, b, r0 / r1);
    if (V.eng.n_curve != s_cross_curve_n || memcmp(V.eng.curve, s_cross_curve, sizeof s_cross_curve) != 0) {   // (another engine, or its curve read again: nothing remembered holds)
        memset(s_cross_memo, 0, sizeof s_cross_memo); memcpy(s_cross_curve, V.eng.curve, sizeof s_cross_curve); s_cross_curve_n = V.eng.n_curve;
    }
    const float step = vehicle_rpm_at(g, 1.0f) / vehicle_rpm_at(g + 1, 1.0f);
    gc_cross_memo *m = &s_cross_memo[g];
    if (!(m->step == step && m->p2 == b->p2 && m->cap == b->cap)) { m->step = step; m->p2 = b->p2; m->cap = b->cap; m->val = bands_cross(&V.eng, b, step); }
    return m->val;
}

// The shaft work of a litre of diesel (J): the lower heating value 35.8 MJ/L x 0.45, the indicated efficiency of the Willans line the game's fuel model follows
// (see the header: gc_fuel_litres).
static const float GC_FUEL_J_PER_L = 35.8e6f * 0.45f;

float gc_fuel_litres(float rpm, float k, float peak_nm, float demand_n, float v, float eta, float hold_s, float on)
{
    const float omega = rpm * 0.10472f;
    const float work_w = demand_n > 0 && eta > 0.1f ? demand_n * v / eta : 0.0f;   // (the work at the shaft: the same in every gear that delivers the demand)
    const float fric_w = omega * k * peak_nm;                                       // (the friction: omega x T_f, the part that grows with the rpm)
    return on * hold_s * (work_w + fric_w) / GC_FUEL_J_PER_L;
}

// What a shift costs when the drive is needed, in seconds of drive time (the score's unit): the interruption itself, dur (the driver feels the gap whatever the road
// does; it is also the time a pull to a speed is held up), and the speed the gap loses, dv = (r / m + the learned deceleration) x dur, regained at the pull of the
// strongest gear a = (fbest - r) / m. What the lost speed costs depends on what the demand is for. A demand that is a speed to reach (the acceleration part of the force
// f - r) is late by the time it takes to win the speed back, dv / a: a duration. A demand that holds a speed (the part r of f) is not late at all, it is behind by a distance:
// the area of the dip, dv x dur / 2 while the speed falls and dv^2 / (2 a) while it recovers, over the speed v, is the time that costs (the same truck, the same climb:
// 8 s as a duration, 0.12 s as drive time, which paid the 5 % climb's shift 17 s early, WP-D2). The two parts weigh as the force does: hs = r / f. gc_expect charges the
// price to a gear that cannot deliver a demand, and the shift now is priced with it while the pedal is off. (On a net descent the truck gains speed in the gap, there is
// nothing to regain: the loss is never negative, a descent does not make a shift cheaper than its dur.)
static float gc_shift_price(float dur, float decel, float m, float r, float fbest, float f, float v)
{
    const float dv = fmaxf(0.0f, r / m + decel) * dur;
    const float a = fmaxf(0.05f, (fbest - r) / m);
    const float hs = f > r && f > 1.0f ? gc_clamp(r / f, 0.0f, 1.0f) : 1.0f;   // the part of the demand that only holds the speed
    const float hold = (0.5f * dv * dur + 0.5f * dv * dv / a) / fmaxf(v, 1.0f);   // the distance the dip costs, as time at the speed
    return dur + hs * hold + (1.0f - hs) * dv / a;
}

float gc_lift_return(float tau, float w, float shift)
{
    // Until the lift has lasted the window (a shift and the time from the decision to the clutch opening) a shift decided now would not be finished when the
    // lift ends if it ends as a lift of that length does: it cannot be used for one (the owner's rule), the pedal is as good as back. After it the cost is the
    // driver profile's own chance that the pedal comes back within the window (core/driver: the scale-free prior with the shift as its unit, learned from this
    // driver's lifts, a re-press only: a lift that ends in the brake needs no drive).
    if (tau < w) return 1.0f;
    return gc_clamp(driver_lift_return_p(tau, w, shift), 0.0f, 1.0f);
}

// The seconds a gear is expected to lose to the demands still to come (see the header). Continuous in the demand and in the gear's force: the
// ramp over GC_SHARE_BAND is the only step, and it is a ramp.
float gc_expect(float fg, float fbest, float share, float dur, float decel, float m, const gc_demand *d, int nd, float v,
                const float *succ_band, float own_band, float band_h, float *band_after, float (*stay_fn)(void *), void *stay_ctx, float v_hi, float stay_q)
{
    float stay_loss = 1e30f; bool stay_got = false;   // (D5c: the stay's loss is read when a price needs it, once per call)
    float acc = 0, alive = 1, run_alive = 1, run_fail = 0;   // (run_*: an exclusive run in progress: the weight it started with and the probability of the shift so far)
    for (int k = 0; k < nd; k++) {
        const int cont = d[k].excl && k > 0 && d[k - 1].excl;   // this demand continues an exclusive run
        if (!cont) {
            if (k > 0 && d[k - 1].excl) alive = run_alive * (1.0f - run_fail);   // the run before has ended: a shift in any of its cases stops what comes after
            run_alive = alive; run_fail = 0;
        }
        const float sh = d[k].share > 0 ? d[k].share : share;
        const float cap = sh * fg, cap_best = sh * fbest;
        const float f = d[k].f, r = d[k].r, p = d[k].p * (d[k].excl ? run_alive : alive);
        float fail = cap > 0 ? gc_clamp((f - cap) / (GC_SHARE_BAND * cap), 0.0f, 1.0f) : 1.0f;
        // nobody can deliver it: a shift does not help, the gears differ by the lag below. Faded, not stepped: a demand a little over the strongest gear's force is one that
        // gear delivers short by that little, and the price of the shift to it falls to nothing over the same share (GC_SHARE_BAND) the gears' own ramp has. A step here priced
        // the climb at the foot (T3, the 4 % climb of 2 km, the demand at the strongest gear's force) as a shift for 12 and for 11 and as none a tick later.
        if (fail > 0) fail *= gc_clamp(1.0f - (f - cap_best) / (GC_SHARE_BAND * cap_best), 0.0f, 1.0f);
        if (fail > 0) {
            // the shift it will need then (the same price as the shift now)
            float price = gc_shift_price(dur, decel, m, r, fbest, f, v);
            // (D5b) staying and pulling short is the other way to meet the demand, and its price is the time the stay loses on the plan (stay_loss: this gear against the best one at the
            // same speed, the full pedal's value function), by the part of the strongest gear's surplus the demand takes of what this gear lacks. The shift is never worth more than the
            // stay loses; a gear that cannot pull (its force under twice the resistance's) keeps the whole price: the cap fades in over the surplus.
            // (D5c) The plan is asked only when the answer can change the price: it cannot where the cap does not fade in (q = 0), nor where the band-top loss alone is already the whole price.
            if (stay_fn && f > fg && fbest > fg && fg > r) {
                const float q = stay_q * gc_clamp((fg - r) / fmaxf(r, 1.0f), 0.0f, 1.0f);
                float lost = 0.0f;
                if (v_hi > v) { const float a_k = (f - r) / m, a_g = fmaxf(0.05f, (fg - r) / m); lost = (v_hi - v) * (1.0f / a_g - 1.0f / a_k); }
                if (q > 0.0f && price > lost) {
                    if (!stay_got) { stay_loss = stay_fn(stay_ctx); stay_got = true; }
                    if (stay_loss < 1e29f) {
                        const float need = gc_clamp((f - fg) / (fbest - fg), 0.0f, 1.0f);
                        price -= q * fmaxf(0.0f, price - fmaxf(need * stay_loss, lost));
                    }
                }
            }
            acc += p * fail * price;
            if (band_after && succ_band && succ_band[k] < 1e29f && band_h > 0.0f)
                // (the driver's history says the largest demand of the window comes somewhere in it, nothing more: on average half way, and the taste is kept until then)
                *band_after += p * fail * (1.0f - gc_clamp((d[k].excl ? 0.5f * band_h : d[k].tau) / band_h, 0.0f, 1.0f)) * (own_band - succ_band[k]);
            if (d[k].excl) run_fail += d[k].p * fail; else alive *= 1.0f - d[k].p * fail;
        }
        if (fail < 1.0f && f > fg) {
            // delivered inside the allowance (or by nobody) but at a lower acceleration than the demand asks: the lag over one shift
            const float a_k = (f - r) / m, a_g = (fg - r) / m;
            if (a_k > 1e-3f) acc += p * (1.0f - fail) * dur * (1.0f - gc_clamp(a_g / a_k, 0.0f, 1.0f));
        }
    }
    return acc;
}

// Seconds from v0 in gear g to v_t at the demanded acceleration a_w (capped per gear by full throttle), shifting up by
// the farthest skip of up to `span` gears that still lands inside the window when the engine reaches the top of its band
// (top[gear] = the gear's crossover: where the next gear gives as much power).
// *out_s gets the shift seconds inside; ts = 0 measures the pure acceleration time (shifts instantaneous, only the force
// in each gear counts). A gear that gives under 0.02 m/s2 counts as 0.02 (slow, not impossible).
// Every shift the plan makes (the gear we are in, or the ones a candidate needs after its own landing) loses the speed of a real shift: dur seconds at
// the resistance plus decel (the candidate's landing shift is in v0 already, so each shift counted here is a further one: it was optimistic before, with
// ts and f_res/m only, and charging the learned loss on the landing shift alone double counted it).
static int s_gain_now;   // gc_gain: 1 = the plan shifts up at once (the first step: the gear is already past its crossover and the next one pays)
static float gc_gain(int g, int n, float v0, float v_t, float a_w, float f_res, const float *top, float lug_lo, float ts, float m,
                     int span, float *out_s, float dur, float decel)
{
    float t = 0, s = 0, v = v0;
    int gear = g;
    s_gain_now = 0;
    const float lose = ts > 0 ? (f_res / m + decel) * dur : 0.0f;   // ts = 0: the pure acceleration time, shifts instantaneous
    for (int i = 0; i < 2000 && v < v_t; i++) {
        float a_full = (vehicle_force_at(gear, v, 1.0f) - f_res) / m;
        float a = a_w > 0 ? fminf(a_w, fmaxf(0.02f, a_full)) : fmaxf(0.02f, a_full);
        bool shift_up = false;
        int h = gear + 1;
        if (gear < n && vehicle_engine_rpm_at(gear, v) >= top[gear]) {
            for (int j = span; j > 1; j--)
                if (gear + j <= n && vehicle_engine_rpm_at(gear + j, v) >= lug_lo) { h = gear + j; break; }
            // Above the crossover the shift is taken only where it finishes sooner than staying: the shift's own time and speed loss, then the
            // next gear's pull (at a constant acceleration, the local view), against the rest of the way in this gear. Near the end of the
            // horizon, or where the next gear gives hardly more, staying wins (without this the rpm of the crossover decided, to the rpm,
            // whether a plan paid a shift at its very end).
            float v2 = fmaxf(0.5f, v - lose);
            float a2_full = (vehicle_force_at(h, v2, 1.0f) - f_res) / m;
            float a2 = a_w > 0 ? fminf(a_w, fmaxf(0.02f, a2_full)) : fmaxf(0.02f, a2_full);
            shift_up = ts + (v_t - v2) / a2 < (v_t - v) / a;
        }
        if (shift_up) {
            if (i == 0) s_gain_now = 1;
            gear = h;
            t += ts; s += ts;
            v = fmaxf(0.5f, v - lose);
            continue;
        }
        float step = fminf(0.25f, v_t - v);
        t += step / a;
        v += step;
    }
    *out_s = s;
    return t;
}


// The fastest way to the horizon speed at a full pedal (WP-G5; since WP-0911-K1 only the rev-limit rule's upshift, gc_full_upshift, asks for it: the pedal's choice is the score's). gc_gain plays a gear out with a rule for the shifts after it (at the crossover, the
// farthest skip that lands on the plateau): the rule is not the best a truck can do. On a flat torque plateau the farthest landing at P1 gives a third less force than a
// gear less, so 8 -> 11 at 36 km/h (937 rpm, 35 % under the best gear for 2.5 s), and on a climb single steps where the fastest way skips; replayed on the vehicle model the
// box was 4-20 % slower to 80 km/h than the best sequence of gears (and on a 4 % climb with 40 t, 22 % slower than a plain one-gear-per-crossover schedule).
// So the time to go is a value function: V(g, v) = the least time from gear g at speed v to the horizon speed, over every sequence of upshifts of up to `span` gears. A gear is
// held while its revs lie inside [floor, cap] and pulls at the full-throttle force of the model; a shift costs ts and loses the speed of the gap (the same loss and price as
// gc_gain: the resistance of now plus the learned deceleration over dur), and must land inside [floor, cap]. Solved on a speed grid by sweeps from the horizon down (a shift lands
// on a lower speed than it leaves, so a few sweeps settle it): no number of its own, the model's forces and the shifts' price are all it reads.
struct gc_plan {
    enum { NI = 192 };
    int n, ni, span;
    float v_lo, dv, v_t, ts, lose, v_now;
    float acc[GC_MAX + 1][NI];       // the gear's full-throttle acceleration at the grid speed (m/s2)
    float rpm[GC_MAX + 1][NI];       // its engine rpm there
    float lo, hi;                    // the window a gear is held in (the bands' floor and cap)
    float V[GC_MAX + 1][NI];         // the time to go
    // what does not change while the table is solved (D5c): the cost of holding a gear one cell (-1: it cannot be held there), and, for the shift out of a cell, where it lands
    float hold_c[GC_MAX + 1][NI];
    unsigned long long land_ok[NI];  // the gears whose rpm at the landing speed is inside the window (bit h)
    float shift_c[NI];               // ts x the weight of the cell's second
    float land_fr[NI];               // the landing between two cells: the fraction past cell land_i (land_kind 2), else the value stands (0: past the horizon, 1: under the grid)
    int land_i[NI], land_kind[NI];
};
static gc_plan s_plan_r;   // (the reserve's plan, D5b)
static gc_plan s_plan;   // (one plan at a time: the gearbox decides on one thread, the sims on one)
static float gc_plan_lerp(const gc_plan *P, const float (*T)[gc_plan::NI], int g, float v)
{
    float x = (v - P->v_lo) / P->dv;
    if (x < 0) x = 0;
    const int i = (int)x < P->ni - 1 ? (int)x : P->ni - 2;
    const float f = x - i;
    return T[g][i] + (T[g][i + 1] - T[g][i]) * f;
}
// The weight of a second spent at speed v: the truck is judged on its way to the horizon speed at three distances, the first quarter, the first half and the whole of it, each as the time it
// takes; a second at speed v counts for the horizons that lie beyond it (1 in the first quarter, 2/3 up to the half, 1/3 to the end). So the early advantage of a stronger gear counts (the
// response: a kickdown) and so does the last stretch (a weight that fell to 0 at the horizon, the first form of this, let a heavy truck creep up to its top speed in the gear it was in).
static float gc_plan_w(const gc_plan *P, float v)
{
    const float D = P->v_t - P->v_now;
    return ((v < P->v_now + 0.25f * D) + (v < P->v_now + 0.5f * D) + (v < P->v_t)) * (1.0f / 3.0f);
}
static float gc_plan_at(const gc_plan *P, int g, float v)       // the time to go from gear g at speed v
{
    if (v >= P->v_t - 1e-4f) return 0.0f;
    if (v < P->v_lo) return 1e9f;
    const float x = (v - P->v_lo) / P->dv;
    const int i = (int)x < P->ni - 1 ? (int)x : P->ni - 2;
    const float a = P->V[g][i], b = P->V[g][i + 1];
    return a >= 1e8f || b >= 1e8f ? 1e9f : a + (b - a) * (x - i);
}
static float gc_plan_hold(const gc_plan *P, int g, float v)     // staying in g one step from v: the time, 1e9 when the gear cannot be held there
{
    const float r = gc_plan_lerp(P, P->rpm, g, v);
    if (r < P->lo || r > P->hi) return 1e9f;
    const float a = fmaxf(0.02f, gc_plan_lerp(P, P->acc, g, v)), step = fminf(P->dv, P->v_t - v);
    return step / a * gc_plan_w(P, v + 0.5f * step) + gc_plan_at(P, g, v + step);
}
static float gc_plan_hold_cell(const gc_plan *P, int g, int i)  // the same on the grid: the next cell is exactly one step on
{
    if (P->rpm[g][i] < P->lo || P->rpm[g][i] > P->hi) return 1e9f;
    return P->dv / fmaxf(0.02f, P->acc[g][i]) * gc_plan_w(P, P->v_lo + (i + 0.5f) * P->dv) + P->V[g][i + 1];
}
static float gc_plan_shift(const gc_plan *P, int g, float v)    // the best shift out of g at v: ts + the value of the landing, 1e9 when none lands inside the window
{
    float best = 1e9f;
    const float v2 = fmaxf(0.5f, v - P->lose);
    for (int h = g + 1; h <= g + P->span && h <= P->n; h++) {
        const float r = gc_plan_lerp(P, P->rpm, h, v2);
        if (r < P->lo || r > P->hi) continue;
        const float t = P->ts * gc_plan_w(P, v) + gc_plan_at(P, h, v2);
        if (t < best) best = t;
    }
    return best;
}
// v_from: the lowest speed a state may start from (the landing of a shift now); the grid spans 3 m/s below it (the speed a few shifts lose) to the horizon speed.
static void gc_plan_fill(gc_plan *P, int n, const eng_bands *b, float v_from, float v_t, float v_now, float ts, float lose, float f_res, float m, int span)
{
    P->v_now = v_now;
    P->n = n < GC_MAX ? n : GC_MAX; P->v_t = v_t; P->ts = ts; P->lose = lose; P->span = span; P->lo = b->floor; P->hi = b->cap;
    P->v_lo = fmaxf(0.5f, v_from - 3.0f);
    const float span_v = fmaxf(v_t - P->v_lo, 0.5f);
    int steps = (int)ceilf(span_v / 0.25f);
    if (steps > gc_plan::NI - 1) steps = gc_plan::NI - 1;
    P->ni = steps + 1;
    P->dv = span_v / steps;
    for (int g = 1; g <= P->n; g++) {
        for (int i = 0; i < P->ni; i++) {
            P->rpm[g][i] = vehicle_engine_rpm_at(g, P->v_lo + i * P->dv);
            P->V[g][i] = i == P->ni - 1 ? 0.0f : 1e9f;
        }
        // the pull is read only where the gear can be held (gc_plan_hold, the hold cost below): not where it and its neighbors sit on the same side out of the window
        for (int i = 0; i < P->ni; i++) {
            const int side = P->rpm[g][i] < P->lo ? -1 : P->rpm[g][i] > P->hi ? 1 : 0;
            const bool read = side == 0 || (i > 0 && (P->rpm[g][i - 1] < P->lo ? -1 : P->rpm[g][i - 1] > P->hi ? 1 : 0) != side) || (i < P->ni - 1 && (P->rpm[g][i + 1] < P->lo ? -1 : P->rpm[g][i + 1] > P->hi ? 1 : 0) != side);
            P->acc[g][i] = read ? (vehicle_force_at(g, P->v_lo + i * P->dv, 1.0f) - f_res) / m : 0.0f;
            P->hold_c[g][i] = side != 0 ? -1.0f : P->dv / fmaxf(0.02f, P->acc[g][i]) * gc_plan_w(P, P->v_lo + (i + 0.5f) * P->dv);
        }
    }
    // (the shift out of a cell lands at the same speed whatever the gear: the landing is looked up once per cell, the gears that land inside the window once per cell)
    for (int i = 0; i < P->ni - 1; i++) {
        const float v = P->v_lo + i * P->dv, v2 = fmaxf(0.5f, v - P->lose);
        P->shift_c[i] = P->ts * gc_plan_w(P, v);
        P->land_ok[i] = 0;
        for (int h = 1; h <= P->n; h++) {
            const float r = gc_plan_lerp(P, P->rpm, h, v2);
            if (r >= P->lo && r <= P->hi) P->land_ok[i] |= 1ull << h;
        }
        if (v2 >= P->v_t - 1e-4f) P->land_kind[i] = 0;
        else if (v2 < P->v_lo) P->land_kind[i] = 1;
        else {
            const float x = (v2 - P->v_lo) / P->dv;
            const int j = (int)x < P->ni - 1 ? (int)x : P->ni - 2;
            P->land_kind[i] = 2; P->land_i[i] = j; P->land_fr[i] = x - j;
        }
    }
    for (int pass = 0; pass < 8; pass++) {
        int changed = 0;
        for (int i = P->ni - 2; i >= 0; i--) {
            float at[GC_MAX + 2];   // (the value of the landing in gear h: the same for every gear that shifts to it, and not yet moved this cell, h being above g)
            for (int h = 2; h <= P->n; h++) {
                if (!((P->land_ok[i] >> h) & 1)) continue;
                if (P->land_kind[i] == 0) at[h] = 0.0f;
                else if (P->land_kind[i] == 1) at[h] = 1e9f;
                else {
                    const float a = P->V[h][P->land_i[i]], b2 = P->V[h][P->land_i[i] + 1];
                    at[h] = a >= 1e8f || b2 >= 1e8f ? 1e9f : a + (b2 - a) * P->land_fr[i];
                }
            }
            for (int g = 1; g <= P->n; g++) {
                float t = P->hold_c[g][i] < 0.0f ? 1e9f : P->hold_c[g][i] + P->V[g][i + 1];
                float best = 1e9f;   // (the best shift out of g: ts + the value of the landing, 1e9 when none lands inside the window)
                for (int h = g + 1; h <= g + P->span && h <= P->n; h++) {
                    if (!((P->land_ok[i] >> h) & 1)) continue;
                    const float ts_t = P->shift_c[i] + at[h];
                    if (ts_t < best) best = ts_t;
                }
                t = fminf(t, best);
                if (t < P->V[g][i] - 1e-4f) { P->V[g][i] = t; changed = 1; }
            }
        }
        if (!changed) break;
    }
}

// The speed the plan runs to: where the wish (a force, N) takes the speed in GC_HORIZON seconds, capped at GC_HORIZON_SPEED ...
static float gc_horizon_speed(unsigned long long allowed, int n, float v, float f_res, float m, float wish_t, float cap)
{
    const float a_w = fmaxf(0.0f, (wish_t - f_res) / m);
    float v_t = a_w > 0.02f ? v + fminf(a_w * GC_HORIZON, GC_HORIZON_SPEED) : v;
    // ... and no farther than the speed some allowed gear can still accelerate the truck to on this resistance. Past it every gear counts as
    // 0.02 m/s2 and the plan times nothing but that floor: the times of all gears came out within 3 s of each other (39-42 s) and the gear that
    // sat on its torque curve's end at 2177 rpm with 2 kN, the others giving 13-24 kN, won by the "stay" bonus.
    if (v_t > v) {
        float reach = v;
        for (float x = v; x < v_t; x += 0.25f) {
            float best_a = -1e9f;
            for (int g = 1; g <= n; g++)
                if ((allowed >> g) & 1) best_a = fmaxf(best_a, (vehicle_force_at(g, x, 1.0f) - f_res) / m);
            if (best_a < 0.02f) break;
            reach = x;
        }
        v_t = fminf(v_t, reach);
    }
    // ... and no farther than the speed the tallest gear reaches at the top of the window: the value function holds a gear only inside
    // [floor, cap] and can only upshift, so past the top gear's cap speed no sequence of gears reaches the horizon (every gear's value is 1e9
    // there, the plan values nothing and the choice falls back to gc_gain's rule). A light truck on a many-gear box accelerates faster than this
    // cap lets it (the 10 t Mercedes G280-16: v + 20 m/s against the top gear's 32.2 m/s at cap), so the horizon must be cut here too.
    float v_top = 0;
    for (int g = 1; g <= n; g++)
        if ((allowed >> g) & 1) v_top = fmaxf(v_top, vehicle_kmh_at(g, cap) / 3.6f);
    if (v_top > v) v_t = fminf(v_t, v_top);
    return v_t;
}

// What a gear is asked of a demand of level f (N, gear-independent) at the engine rpm it runs at: f x phi(rpm) for the torque pedal, f for a controller.
static float gc_ask(const gc_in *in, float f, float rpm) { return in->phi ? f * in->phi(rpm) : f; }


// ---------------------------------------------------------------- the path (WP-0911-K1 round 7)
// The pedal sets an operating point (intent.tgt_rpm), and a gear is priced by the PATH it takes from here, not by the revs it has this instant. The first six rounds priced the miss at the speed
// of now for a whole horizon, which says a gear that sits nearest the point for 0.2 s wins 20 s of it (a light launch climbed 3 > 4 > 5 one by one, a calm pedal on a +4 % climb 10 > 11 > 10).
// Here every gear is a state of a value function over the next GC_HORIZON seconds, solved backwards on a grid of speed and in steps of time:
//  - the speed moves by the force that gear gives at THIS pedal (the pedal's ask at the gear's rpm, capped by what the gear has, less the road at that speed): a gear whose ask is under the road
//    sags, one that has the spare pulls (the wish of now and the full pedal are not the path);
//  - a second in a gear costs its miss (the distance of its rpm from the point in gear steps, shaped as the foot minds it), the band taste and the fuel of that rpm as rates over the horizon, and,
//    outside the engine's window, a wall of a path's seconds per second;
//  - every shift on the path costs ts, the later shifts of the gear we are in too (no free hand-over); a gear may hand up by the skip it is allowed and back down to its neighbour, so a gear that
//    sags pays the way back. The speed the gap loses is the shift now's price (ts + the regain, below) and not the path's: priced on the path as well, a truck threw speed away in two shifts
//    to bring a gear's rpm to the point (a dip is rpm for free when only the pace of the pedal's own path is asked);
//  - a truck behind the speed the pedal's own level would reach is late by the distance it is behind, over that speed (seconds), weighed with how much this foot wants time (w_time: a calm foot little).
// The weights are the kickdown's (1 - tgt_kd, the plan's time takes over at the end of the pedal) and the bend's (1 - lat_load). No number of its own: the seconds are the score's. With no point to
// be at (the pedal off, a controller) there is no path and the score is as it was (the later shifts of gc_gain, the landing's commitment, the band taste and the fuel at the landing).
struct gc_path {
    enum { NI = 64, K = 40 };
    int n, ni;
    float v_lo, dv, dt;
    float acc[GC_MAX + 1][NI];      // m/s2 at this pedal
    float cost[GC_MAX + 1][NI];     // s per s: the (weighted) miss, the wall outside the window
    float hx[GC_MAX + 1][NI];       // where one step of holding the gear lands, in cells
    int hj[GC_MAX + 1][NI];         // the cell under it and the fraction above it (the interpolation of gc_path_at, once, not at every step)
    float hf[GC_MAX + 1][NI];
    float rpm_e[GC_MAX + 1][NI], rpm_d[GC_MAX + 1][NI], ask[GC_MAX + 1][NI], ffull[GC_MAX + 1][NI];   // what a cell's price is made of
    float wl[GC_MAX + 1][NI];       // the wall of a cell the pull takes the current gear out of within the time a change needs (not charged for that time)
    float res[NI], hxmin[NI], hxmax[NI];   // the cell's speed-price and the lowest / highest place a hold of any gear lands at from it
    float Va[GC_MAX + 1][NI + 1], Vb[GC_MAX + 1][NI + 1];   // the value of the step after / of this step (one cell of padding: the interpolation reads the cell above)
    float VL[GC_MAX + 1][NI + 1];                            // the value of the step after the gap (the one layer the candidates that shift now are read at, below)
};
static gc_path s_gpath;
static inline int gc_clamp_i(int x, int lo, int hi) { return x < lo ? lo : (x > hi ? hi : x); }
static float gc_path_at(const float *row, int ni, float x)
{
    if (!(x > 0.0f)) return row[0];
    if (x >= (float)(ni - 1)) return row[ni - 1];
    const int i = (int)x;
    return row[i] + (row[i + 1] - row[i]) * (x - (float)i);
}
// The distance of an rpm from the pedal's point in gear steps (K1 round 9): the log of the ratio over the log of the box's own step at that gear, so one step off the point costs one second per
// second whatever the rpm, truck or mode. Skipping a gear then costs about the seconds it would have been held one step off, taking it costs ts: a middle gear is taken when it would be held
// longer than the shift (Damien's choice 1) without a number of its own. An rpm share cannot say it: a step is a different share at every rpm. Revs over the point cost as they are; revs under it
// cost by how much the foot minds them (intent.tgt_lo): a light foot asked for a calm engine and a gear that lands a little lower is only calmer, a firm foot asked for the revs and a gear that lands
// under them does not give them. The calm has a floor, the middle of the eco band (P1 .. P2) or the point if that is lower: under it the engine is not calmer, it is low (a 41 t truck crawling at
// a quarter pedal went 3 > 4 > 3 > 4 on landings of 880 rpm), and what is under it costs whole, whatever the foot. On a converter the calm floor is the launch step's landing (the middle of the
// idle and the floor, the owner's rule of 04-11/05-11).
static float gc_miss_rpm(const gc_in *in, const eng_bands *b, float r, float step_ln, float tgt_rpm, float tgt_lo)
{
    const float calm = b->p1 + 0.5f * (b->p2 - b->p1);
    const float calm_g = V.tc ? fminf(calm, 0.5f * (b->idle + b->floor)) : calm;
    const float floor_r = fminf(calm_g, tgt_rpm);   // (the floor of the calm is never above the point itself)
    const float rr = fmaxf(r, 1.0f), tg = fmaxf(tgt_rpm, 1.0f);
    const float dl = logf(rr / tg);
    const float under = rr < floor_r ? logf(floor_r / rr) : 0.0f;
    return (dl >= 0.0f ? dl : (-dl - under) * tgt_lo + under) / step_ln;
}
// Solve the path for the gears with cand[g] and write the value of each into out[g] (seconds, hold-first: the gear keeps the first step, a shift is possible from the second on, so the gear we are
// in is not credited with the shift a candidate is charged for). Every gear starts at the speed of now: the dip of the gap is the shift now's price (ts + the regain), not the path's -- except at a HOLD
// (w_dip): a hold wins nothing back, so a gear that shifts now is also read from the speed the gap leaves the truck at (v_dip), and the worse of the two starts counts.
// WHY (the limit cycle of g7 R3, 41.9 t on +6 % at a calm pedal, 4 > 5 > 4 every 12 s for ever): the pedal's force grows with the revs (phi), so at a fixed foot a gear has a tipping speed, above it the truck
// pulls on, below it it sags. 4 > 5 was read from 17.9 km/h, above g5's tipping speed (g5 pulls: tgt 9.50, a saving of 1.6 s over staying in g4 against a shift of 0.9 and a reserve of 0.8); the gap lands
// the truck at 15.4 km/h, under it, where g5 sags to the floor (tgt 11.1) and, when the dwell was over, 5 > 4 won by 0.14 s; in g4 the truck climbed back to 17.9. The prediction was 1.6 s off the value
// the choice itself gave on arrival: a decision that its own next decision undoes. Read at both starts the shift is taken only if it is good where the truck will be as well as where it is.
// (Read at the dip ALONE it was far worse -- hunting pairs over the 54 cells of g7 R3 66 > 1414 -- because the kink of the miss at the point turns a dip of 1.5 % in speed into 1.5 s of path:
//  a dip is rpm for free. The worse of the two never rewards a dip.)
static void gc_path_solve(const gc_in *in, const eng_bands *b, int n, int cur, float v, float v_dip, float w_dip, float wish, float f_res, float m, float ts, int span, float strongest, const float *cross,
                          float w_t, float w_k, float w_time, const unsigned char *cand, const float *tg_rpm, const float *tg_lo, float *out)
{
    gc_path *P = &s_gpath;
    P->n = n < GC_MAX ? n : GC_MAX;
    // The path is the plan horizon. (A hold that has lasted is expected to go on, but the age of a hold restarts at every shift: a longer path for the gear we leave than for the one we land in
    // made the two decisions of a cycle disagree, 10 > 11 > 10 on the +4 % climb.)
    // (and no longer than a pull takes to reach the horizon speed of the plan, a launch's step: a pull of 4 m/s2 is at 20 m/s in 5 s, and 40 steps of 0.5 s see a gear's whole range pass in one)
    const float a_lvl0 = (fminf(wish, strongest) - f_res) / m;
    const float T = a_lvl0 > 0.02f ? gc_clamp(GC_HORIZON_SPEED / a_lvl0, 2.0f, GC_HORIZON) : GC_HORIZON;
    P->dt = T / (float)gc_path::K;
    // The grid: a step of time moves the speed by a dt, and the value is interpolated at every step, which smears it over the speed (a few steps of a tenth of a cell blur it by whole cells). So
    // the cell is the speed a step moves the truck at the largest pull of the gears now, no finer than 0.08 m/s (the grid has gc_path::NI cells) and no coarser than half a m/s.
    float a_max = 0.05f;
    for (int g = 1; g <= P->n; g++) {
        const float rpm_e = vehicle_engine_rpm_at(g, v);
        const float F = fminf(gc_ask(in, wish, rpm_e), vehicle_force_at(g, v, 1.0f));
        a_max = fmaxf(a_max, fabsf((F - f_res) / m));
    }
    const int ni = gc_path::NI;
    // (a ladder of cell sizes and a grid anchored at multiples of the cell: the same truck in the same state gets the same grid, so a few newtons of wish do not move the interpolation)
    float dv = 0.5f;
    for (int q = 0; q < 3 && a_max * P->dt < 0.5f * dv; q++) dv *= 0.5f;
    P->ni = ni; P->dv = dv < 0.0625f ? 0.0625f : dv;
    const float below = fminf(4.0f, 0.4f * (ni - 1) * P->dv);
    P->v_lo = fmaxf(P->dv, floorf((v - below) / P->dv) * P->dv);
    const float wall = T;                                             // a second outside the window costs a whole path: a wall that is a price
    const float gap_dur = fmaxf(in->shift_dur, ts);   // (the interruption of a shift, decision to gear in: the time before a change can take effect)
    const float lo = V.tc ? fminf(b->floor, 0.5f * (b->idle + b->floor)) : b->floor, hi = b->cap;
    // The speed the pedal's own level takes the truck to (the strongest gear's force at most), the reference the path is late against: a truck behind it is behind by a distance, and a distance
    // is the seconds it takes at the speed (the price gc_shift_price has for a dip that holds a speed).
    const float a_lvl = (fminf(wish, strongest) - f_res) / m;

    // WHICH gears and WHICH cells the value is made of (K1 round 14). The path is a value over every gear and every cell of the grid, but only what the truck can come to from where it is matters, and
    // that is a small part of it: a gear whose revs are outside the window at every speed of the grid (the rpm of a gear rises with the speed, so the lowest speed's revs over the cap or the highest
    // speed's under the floor say it) is a wall at every cell and no path goes through it; and the cells the truck can come to in k steps are the ones within the pulls of the gears of where it is
    // (a hold moves the speed by acc x dt, a shift moves nothing), an interval that grows from the cell of the speed now. The value at a cell is made of the cells one step on, so it is computed
    // exactly there and nowhere else: the same numbers as the whole grid gives, at the cost of the part of it that is reached.
    const float v_top = P->v_lo + (float)(ni - 1) * P->dv;
    bool rel[GC_MAX + 2]; int gl[GC_MAX + 2], ng = 0;
    for (int g = 1; g <= P->n; g++) {
        rel[g] = cand[g] || (vehicle_engine_rpm_at(g, P->v_lo) <= hi && vehicle_rpm_at(g, v_top) >= lo);
        if (rel[g]) gl[ng++] = g;
    }
    rel[0] = false; rel[P->n + 1] = false;
    const float fuel_k_now = fuel_k();
    float step_ln_g[GC_MAX + 2];
    for (int k = 0; k < ng; k++) {   // (the box's own step at the gear: the two gears' revs at one speed, the same at every speed)
        const int g = gl[k], g2 = g < P->n ? g + 1 : g - 1;
        const float r2 = g2 >= 1 ? vehicle_rpm_at(g2, 1.0f) : 0.0f, r1 = vehicle_rpm_at(g, 1.0f);   // (read at 1 m/s as gc_cross: the speed cancels, and at a standstill v = 0 would give no step)
        step_ln_g[g] = fmaxf(r1 > 1.0f && r2 > 1.0f ? fabsf(logf(r1 / r2)) : 0.25f, 0.01f);
    }
    // what a cell's pull is (the part of its price the reach needs): its revs, what the pedal asks of the gear and what the gear has, the speed one step of holding it lands at
    auto fill_acc = [&](int i) {
        const float vi = P->v_lo + i * P->dv;
        const float res = vehicle_resist(vi);
        P->res[i] = res;
        float xlo = 1e30f, xhi = -1e30f;
        for (int k = 0; k < ng; k++) {
            const int g = gl[k];
            const float rpm_d = vehicle_rpm_at(g, vi), rpm_e = V.tc ? vehicle_engine_rpm_at(g, vi) : rpm_d;   // (a clutch box: the engine turns at the driveline's rpm, one read)
            const float ask = gc_ask(in, wish, rpm_e), ff = vehicle_force_at(g, vi, 1.0f);
            float F = fminf(ask, ff);
            if (!in->controller && vehicle_tc_slip(g, vi) < 1.0f) F = fminf(F, vehicle_foot_force_at(g, vi));
            P->rpm_e[g][i] = rpm_e; P->rpm_d[g][i] = rpm_d; P->ask[g][i] = ask; P->ffull[g][i] = ff;
            P->acc[g][i] = (F - res) / m;
            const float x = (float)i + P->acc[g][i] * P->dt / P->dv;
            P->hx[g][i] = x;
            if (x < xlo) xlo = x;
            if (x > xhi) xhi = x;
            if (!(x > 0.0f)) { P->hj[g][i] = 0; P->hf[g][i] = 0.0f; }
            else if (x >= (float)(ni - 1)) { P->hj[g][i] = ni - 1; P->hf[g][i] = 0.0f; }
            else { const int j = (int)x; P->hj[g][i] = j; P->hf[g][i] = x - (float)j; }
        }
        P->hxmin[i] = xlo; P->hxmax[i] = xhi;
    };
    int fl = 0, fh = -1;   // the cells filled so far
    auto extend = [&](int a, int z) {
        if (fh < fl) { for (int i = a; i <= z; i++) fill_acc(i); fl = a; fh = z; return; }
        for (int i = a; i < fl; i++) fill_acc(i);
        for (int i = fh + 1; i <= z; i++) fill_acc(i);
        if (a < fl) fl = a;
        if (z > fh) fh = z;
    };
    const float x0 = (v - P->v_lo) / P->dv;
    // (a gear that shifts now is read at the speed of now and, at a hold, at the speed the gap leaves the truck at as well, v_dip: w_dip)
    const float xd = (v_dip - P->v_lo) / P->dv;
    const int c0 = gc_clamp_i((int)x0, 0, ni - 1), c1 = gc_clamp_i((int)x0 + 1, 0, ni - 1);
    const int d0 = gc_clamp_i((int)xd, 0, ni - 1), d1 = gc_clamp_i((int)xd + 1, 0, ni - 1);
    extend(std::min(c0, d0), std::max(c1, d1));
    int Ilo[gc_path::K + 1], Ihi[gc_path::K + 1];   // the cells the value of step k is needed at (k = 1 .. K - 1)
    {
        float xmin = 1e30f, xmax = -1e30f;
        for (int g = 1; g <= P->n; g++) {
            if (!cand[g]) continue;
            const float x1 = x0 + gc_path_at(P->acc[g], ni, x0) * P->dt / P->dv;
            xmin = fminf(xmin, x1); xmax = fmaxf(xmax, x1);
            if (g != cur) { const float x1d = xd + gc_path_at(P->acc[g], ni, xd) * P->dt / P->dv; xmin = fminf(xmin, x1d); xmax = fmaxf(xmax, x1d); }
        }
        Ilo[1] = gc_clamp_i((int)floorf(xmin), 0, ni - 1); Ihi[1] = gc_clamp_i((int)floorf(xmax) + 1, 0, ni - 1);
    }
    extend(Ilo[1], Ihi[1]);
    {   // the interval grows by what a hold of any gear lands at from the cells in it; the extremes over the cells are kept as the interval grows (a cell added once is looked at once)
        float xmin = 1e30f, xmax = -1e30f; int sa = Ilo[1], sz = Ihi[1];
        for (int i = sa; i <= sz; i++) { if (P->hxmin[i] < xmin) xmin = P->hxmin[i]; if (P->hxmax[i] > xmax) xmax = P->hxmax[i]; }
        for (int k = 2; k <= gc_path::K - 1; k++) {
            Ilo[k] = gc_clamp_i(std::min(Ilo[k - 1], (int)floorf(xmin)), 0, ni - 1);
            Ihi[k] = gc_clamp_i(std::max(Ihi[k - 1], (int)floorf(xmax) + 1), 0, ni - 1);
            extend(Ilo[k], Ihi[k]);
            for (int i = Ilo[k]; i < sa; i++) { if (P->hxmin[i] < xmin) xmin = P->hxmin[i]; if (P->hxmax[i] > xmax) xmax = P->hxmax[i]; }
            for (int i = sz + 1; i <= Ihi[k]; i++) { if (P->hxmin[i] < xmin) xmin = P->hxmin[i]; if (P->hxmax[i] > xmax) xmax = P->hxmax[i]; }
            if (Ilo[k] < sa) sa = Ilo[k];
            if (Ihi[k] > sz) sz = Ihi[k];
        }
    }
    // what a second in a gear costs, at the cells that are reached
    if (cur >= 1 && cur <= GC_MAX) memset(P->wl[cur], 0, sizeof P->wl[cur]);
    for (int i = fl; i <= fh; i++) {
        const float vi = P->v_lo + i * P->dv;
        for (int q = 0; q < ng; q++) {
            const int g = gl[q];
            const float rpm_e = P->rpm_e[g][i], rpm_d = P->rpm_d[g][i];
            const float miss = gc_miss_rpm(in, b, rpm_e, step_ln_g[g], tg_rpm[g], tg_lo[g]);
            const int inwin = rpm_e <= hi && rpm_d >= lo;
            // The band taste and the fuel of this second (they were charged for the whole horizon at the rpm of the gear's landing, which says a gear the path leaves at once keeps its taste for 20 s):
            // the same costs as the score had, as a rate over the horizon, in the state the path is in. The gear we are in keeps the run costs.
            float bw = gc_clamp(in->intent.hold_share, 0.0f, 1.0f);
            if (!in->controller && !in->game_cruise) {
                const float idle = gc_clamp(1.0f - P->ask[g][i] / fmaxf(P->ffull[g][i], 1.0f), 0.0f, 1.0f);
                const float room = gc_clamp(1.0f - in->intent.need * in->intent.expect_weight, 0.0f, 1.0f);
                bw = fmaxf(bw, idle * idle * room * (1.0f - gc_clamp(in->lat_load, 0.0f, 1.0f)));
            }
            const int band = pt_band_of(b, rpm_e, cross[g]);
            float bc = (band == PT_ECO ? pt_eco_cost(&in->intent, b, rpm_e, g == cur) : g == cur ? in->intent.band_cost_run[band] : in->intent.band_cost_land[band]) * bw;
            if (band >= PT_POW && band <= PT_TMP && b->cap > b->p2 + 1.0f) bc *= 1.0f + gc_clamp((rpm_e - b->p2) / (b->cap - b->p2), 0.0f, 1.0f);
            const float hs = gc_clamp(in->intent.hold_share, 0.0f, 1.0f);
            const float fuel = in->intent.fuel_weight * gc_fuel_litres(rpm_e, fuel_k_now, V.eng.torque, wish, vi, V.eta, 1.0f, hs);
            const float taste = w_k * fmaxf(in->band_scale, 1.0f) * (bc / GC_HORIZON + fuel);
            P->cost[g][i] = w_t * miss + taste + (inwin ? 0.0f : wall);
            // The floor of the gear we are in is judged when a change could take effect (K1 round 16, the twin of the limiter wall's rpm_rate x latency): a cell under the floor that the pull takes the
            // gear out of within the time a shift needs (the lock of a launch sits at the slip rpm, under the floor) is not charged its wall for that time (below, `cw`).
            P->wl[g][i] = (g == cur && !inwin && rpm_d < lo && rpm_e <= hi && P->acc[g][i] > 0.0f && vi > 0.1f && rpm_d * (1.0f + P->acc[g][i] * gap_dur / vi) >= lo) ? wall : 0.0f;
        }
    }
    // the seconds the truck is late in step k at speed vi: the distance it is behind the pedal's path over the speed of that path
    auto late = [&](int k, float vi) {
        const float vr = fmaxf(v + a_lvl * (k + 0.5f) * P->dt, 0.5f);
        return w_k * w_time * P->dt * fmaxf(0.0f, vr - vi) / vr;
    };
    memset(P->Va, 0, sizeof P->Va);   // (nothing past the end of the path)
    float (*Vn)[gc_path::NI + 1] = P->Va, (*Vc)[gc_path::NI + 1] = P->Vb;
    const float shift_w = w_k * ts;
    // THE GAP (K1 round 15, the launch at a light pedal). A shift takes the box out of gear for its gap, and the gear it lands in is not engaged in it: the engine is free, the truck does not pull. The path
    // above charged the landing gear's cost from the first step and let the truck pull on through the gap, so a shift now paid ts for the interruption AND the landing's miss for the same seconds, at a
    // speed the truck never reaches: at a launch (16.5 t at a quarter pedal pulls 1.7 m/s2: over the gap it would have gained 2.2 m/s, as much as the speed it is at) the landing's miss over the gap is
    // 0.35 .. 0.5 s on a price of 0.9, and the gear that stays won the tie to 1650 rpm and then jumped 1 > 4 (g7 R1, five rows). The candidate that shifts now has its path start after the gap, from the
    // speed of now (the truck does not pull in it): the gap itself is the shift's price, ts (shiftc), and the steps of the gap are not charged (neither their miss nor their taste; the truck is then late by
    // what it did not pull, which the lateness term weighs with the foot). Only as far as the gap changes the state. x = a_lvl x dur / v is the share of the speed the pull would have added over the
    // interruption (decision to gear in); the gap matters once that is more than the box's own step (s = the ratio of two gears - 1: the gears are that far apart in speed) and fully at two steps, as a
    // smoothstep. Below it the old accounting stands: it is the hysteresis of a hold (with the credit whole at a hold R3 / R3b / R9 hunt, 10 > 11 > 10 on +4 %, and the P2 blip rows move), and at a launch the
    // credit is whole. Measured against the old accounting everywhere: g7 R1 5 FAIL -> 0, run.sh hunting pairs 62 -> 31, but R3 / R3b / R9 red.
    const int n_gap = (int)ceilf(fminf(ts, T) / P->dt);
    const float x_gap = a_lvl > 0.0f ? gc_clamp(a_lvl * gap_dur / fmaxf(v, 1.0f), 0.0f, 1.0f) : 0.0f;
    float s_gap = 0.25f;   // (the box's step in speed at the gear we are in; a quarter when the ratios are not known)
    { const int g2 = cur < P->n ? cur + 1 : cur - 1; const float r1 = cur >= 1 ? vehicle_rpm_at(cur, 1.0f) : 0.0f, r2 = g2 >= 1 ? vehicle_rpm_at(g2, 1.0f) : 0.0f; if (r1 > 1.0f && r2 > 1.0f) s_gap = fmaxf(fabsf(r1 / r2 - 1.0f), 0.05f); }
    const float y_gap = gc_clamp((x_gap - s_gap) / s_gap, 0.0f, 1.0f);
    const float w_gap = y_gap * y_gap * (3.0f - 2.0f * y_gap);
    const bool credit = w_gap > 0.0f && n_gap >= 1 && n_gap + 1 <= gc_path::K - 1;
    const int k_gap = n_gap + 1;   // the step the value after the gap is read at
    float vh[GC_MAX + 2];   // the hold of each gear at the cell in hand (a gear that is no state of the path stays at "never")
    for (int g = 0; g <= GC_MAX + 1; g++) vh[g] = 1e30f;
    for (int k = gc_path::K - 1; k >= 1; k--) {
        const int a = Ilo[k], z = Ihi[k];
        for (int i = a; i <= z; i++) {
            const float lt = late(k, P->v_lo + i * P->dv);
            for (int q = 0; q < ng; q++) {
                const int g = gl[q];
                const float *row = Vn[g]; const int j = P->hj[g][i];
                const float cw = g == cur && (float)k * P->dt < gap_dur ? P->wl[g][i] : 0.0f;
                vh[g] = P->dt * (P->cost[g][i] - cw) + lt + (row[j] + (row[j + 1] - row[j]) * P->hf[g][i]);
            }
            for (int q = 0; q < ng; q++) {   // a shift takes the best hold of the neighbour below or of the skips above, at the cost of the shift and the same speed
                const int g = gl[q];
                float best = vh[g];
                const int h_hi = g + span < P->n ? g + span : P->n;
                if (g > 1) { const float c = shift_w + vh[g - 1]; if (c < best) best = c; }
                for (int h = g + 1; h <= h_hi; h++) { const float c = shift_w + vh[h]; if (c < best) best = c; }
                Vc[g][i] = best;
            }
        }
        if (credit && k == k_gap)
            for (int q = 0; q < ng; q++) { const int g = gl[q]; for (int i = a; i <= z + 1 && i <= gc_path::NI; i++) P->VL[g][i] = Vc[g][i]; }
        float (*t)[gc_path::NI + 1] = Vn; Vn = Vc; Vc = t;
    }
    // the first step, held (Vn is the value of step 1)
    for (int g = 1; g <= P->n; g++) {
        if (!cand[g]) continue;
        auto first_step = [&](float xs, float vs) {
            float o = P->dt * (gc_path_at(P->cost[g], ni, xs) - (g == cur ? gc_path_at(P->wl[g], ni, xs) : 0.0f)) + late(0, vs) + gc_path_at(Vn[g], ni, xs + gc_path_at(P->acc[g], ni, xs) * P->dt / P->dv);
            if (credit && g != cur) {   // (the gear that is in after the gap: its first step is the step after it, from the cell of the start)
                const float after = P->dt * gc_path_at(P->cost[g], ni, xs) + late(n_gap, vs) + gc_path_at(P->VL[g], ni, xs + gc_path_at(P->acc[g], ni, xs) * P->dt / P->dv);
                o = (1.0f - w_gap) * o + w_gap * after;
            }
            return o;
        };
        out[g] = first_step(x0, v);
        if (g != cur && w_dip > 0.0f) out[g] += w_dip * fmaxf(0.0f, first_step(xd, v_dip) - out[g]);
    }
}

static int gc_choose_impl(const gc_in *in, gc_out *dbg);
// A SKIP out of a coupled gear of a converter box (over the lock gear) lands in a coupled gear: the skip candidates are judged without the converter's slip and multiplication (core/vehicle.cpp
// s_coupled_from has the ride). The gear we are in at or under the lock gear (the launch) keeps the slipping model.
int gc_choose(const gc_in *in, gc_out *dbg)
{
    const int lg = vehicle_tc_lock_gear();
    vehicle_tc_coupled_from(V.tc && lg > 0 && in->cur > lg ? lg : 0, in->cur + 2);
    const int r = gc_choose_impl(in, dbg);
    vehicle_tc_coupled_from(0, 0);
    return r;
}
static int gc_choose_impl(const gc_in *in, gc_out *dbg)
{
    memset(dbg, 0, sizeof *dbg);
    const int cur = in->cur, n = in->n;
    const float v = in->v, m = V.mass, ts = in->shift_time;
    const eng_bands *b = &in->bands;
    dbg->prev = in->prev; dbg->prev_w = in->prev_w; dbg->shift_ts = ts;   // (observation only)
    if (cur < 1 || cur > n || n < 1 || !(m > 0) || !(v > 0.1f) || !b->ok) {
        dbg->best = cur >= 1 && cur <= n ? cur : 0;
        dbg->why = GC_KEEP;
        return dbg->best;
    }

    const float f_res = vehicle_resist(v);
    // the speed lost in a shift: over the time it really takes from the decision to the gear being in (the caller learns it), at the
    // resistance of now plus whatever else the shift loses (the caller learns that too)
    const float dur = in->shift_dur > ts ? in->shift_dur : ts;
    const float decel = fmaxf(0.0f, in->shift_decel);
    const float v_after = fmaxf(0.5f, v - (f_res / m + decel) * dur);
    const float wish0 = in->intent.wish_n;
    const int lift = in->lift_n > 0;                        // the pedal is off: the wish is the demand that is expected back, with the probability lift_p
    const float wish_q = lift ? 0.0f : wish0;               // (nothing is delivered now: every gear in the window qualifies, the reserve for what comes back is gc_expect's price)
    dbg->level_n = wish0; dbg->lift = lift; dbg->phi_on = in->phi != nullptr;   // (observation only)
    dbg->hold_share = in->intent.hold_share; dbg->hold_n = in->intent.hold_n; dbg->band_scale = in->band_scale; dbg->urgency = in->intent.urgency; dbg->need = in->intent.need;
    const float pw = lift ? gc_clamp(in->lift_p, 0.0f, 1.0f) : 1.0f;   // the weight of everything that only happens when the demand comes back
    const float wall = b->cap - in->rpm_rate * in->latency;   // near the top: an upshift is due
    const float cur_rpm = vehicle_engine_rpm_at(cur, v);
    // The gear we are in keeps a tie-break bonus; in a bend it keeps lat_load shift durations on top (no shift mid-bend; a protection is no score:
    // the limiter wall and the stall rule shift anyway). Bounded at two durations: a yaw spike must not hold a gear that the road itself is leaving.
    const float stay_b = GC_STAY + gc_clamp(in->lat_load, 0.0f, 2.0f) * dur;

    // The crossover of each gear (where the next gear gives as much power): the end of its power band
    float cross[GC_MAX + 2];
    for (int g = 1; g <= n && g <= GC_MAX; g++) cross[g] = gc_cross(b, g, n, v);

    // Per gear: the rpm it runs at (after the shift for any other gear), its force, its band and the rpm window.
    int in_win_any = 0;
    float strongest = -1; int strongest_g = 0;   // the strongest gear inside the window (the kickdown fallback)
    for (int g = 1; g <= n; g++) {
        gc_term *t = &dbg->t[g];
        t->g = g;
        dbg->n++;
        t->allowed = (in->allowed >> g) & 1;
        if (!t->allowed) continue;
        float vg = g == cur ? v : v_after;
        t->rpm = vehicle_engine_rpm_at(g, vg);
        t->ask_n = gc_ask(in, wish_q, t->rpm);   // (observation only)
        t->force = vehicle_force_at(g, vg, 1.0f);
        t->pull = t->force;
        if (!in->controller && vehicle_tc_slip(g, vg) < 1.0f) t->pull = fminf(t->force, vehicle_foot_force_at(g, vg));   // the torque pedal's throttle passes the pedal through while the converter slips (modules/pedal.cpp): that gear gives the foot's own pull, not the shaped demand (the speed it regains and holds)
        t->band = pt_band_of(b, t->rpm, cross[g]);
        // A converter below its lockup that carries the wish against the road (the one test, vehicle_tc_carries) has no floor: its engine rpm is the slip's, and the gear is a lugging
        // one at worst (the lug band's price, gc_expect) -- the owner's rule of 04-11, WP-TC1. Everything else (a locked converter, a clutch box) keeps the no-go floor.
        // (one gear up at most: the relief is for the gear after this one, as a driver's step; a converter that carries 3rd at 10 km/h on a 0.2 pedal does it at an engine speed three gears
        // out of step with the driveline, heat for nothing, and the 1 > 3 of the first try came back down at the lockup, 3 > 2 > 3)
        // The owner 05-11 (ride on the Allison, 1>2 and 2>3 at ~620 rpm, the coupled 2nd and 3rd at 470-550, under the idle): the relief is for the gear the truck is in, and for
        // the one step up out of the converter's launch gear, and that step lands (once coupled) at least halfway between the idle and the floor ("midden": 550..720 -> ~635,
        // out of 1st at ~1270 on this box). Every other upshift keeps the floor of the curve, converter or not; a coast does not go up under it either (3>4 at 558, pedal off).
        const int lock_g = vehicle_tc_lock_gear();
        // (an upshift is judged at the driveline's rpm, what the engine turns once the gear couples: the converter's slip rpm let every landing through)
        const float land = g > cur ? fminf(t->rpm, vehicle_rpm_at(g, vg)) : t->rpm;
        // (the step out of the launch gear needs only to hold the road, not to carry the whole wish: a firm pedal asks more than any gear but the 1st gives, and without
        //  this the 1st was held to 2200 rpm, waiting for the 3rd to clear the floor -- sim TC2, 20 t at a 0.5 pedal)
        // (and the gear it landed in keeps the same relief while it runs up to the floor -- down to a quarter of the way, the shift's gap costs speed: an idle-650 ATS engine
        //  landed at 729 under the 735 half and went straight back, TC fleet sim 05-11 -- else the layers took it back: 1>2>1>2 in 5 s, sim TC2 at a 0.11 pedal. The
        //  gears out of the launch gear are the 2nd .. one over the lock gear's top, so a skip 1>3 is one of them; tc_relief in modules/trueshift.cpp is the same set.)
        const int top = lock_g > 1 ? lock_g : 2;
        const int ls_gear = g >= 2 && g <= top + 1 && land >= (g == cur ? 0.75f * b->idle + 0.25f * b->floor : 0.5f * (b->idle + b->floor)) && V.tc
                           && vehicle_force_at(g, vg, 1.0f) > vehicle_resist(vg);   // (holds the road; vehicle_tc_carries says no for a coupled gear)
        const int launch_step = ls_gear && (g == cur || (g > cur && cur < top));
        const int relief = land < b->floor && ((g <= cur && vehicle_tc_carries(g, vg, wish_q)) || launch_step);
        if (relief) t->band = PT_LUG;
        // The floor is judged at the rpm the gear will have when a change could take effect, as the limiter wall is judged `rpm_rate x latency` ahead (K1 round 16): the gear we are in, under the floor but taken
        // over it by the pull within the time a downshift needs, is not under it (the lock of a launch sits at the slip rpm, under the floor, and the pull takes the engine over it in a tenth of a second).
        const float land_eff = g == cur ? land + fmaxf(in->rpm_rate, 0.0f) * dur : land;
        t->in_win = (land_eff >= b->floor || relief) && t->rpm <= b->cap;
        // A downshift lands at the speed after the shift, but the pull wins that speed back at once: the gear must also clear the wall at the speed
        // of now (ATS-style climb at the limit: 10->9 landed at 1754, the truck got its 2 km/h back, 9 ran into the top and went up again, every 12 s).
        if (t->in_win && g < cur && vehicle_engine_rpm_at(g, v) > wall) t->in_win = 0;
        if (t->in_win) {
            in_win_any = 1;
            if (t->force > strongest) { strongest = t->force; strongest_g = g; }
        }
    }
    // A pedal at the end of its travel asks everything the truck can give. The torque pedal's s = 1 is the envelope (the strongest gear's force) already; a plain pedal (no wish module, or the game's
    // cruise pinned against its stop) asks the force of the gear it is in at the throttle it is at, which at the end of the pedal is that gear's own full force: no taller gear delivers it, and the
    // box could neither choose by the target nor leave the wall. So the kickdown is a blend of the two, by how far into it the foot is (continuous: the wish of a plain pedal is the gear's own below it).
    const float wish = (!lift && !in->phi && !in->controller && in->intent.tgt_kd > 0.0f && strongest > 0.0f) ? wish0 + in->intent.tgt_kd * (strongest - wish0) : wish0;
    if (!in_win_any) {
        // No gear is in the window at all: keep the current gear, or the nearest gear inside.
        float best_dist = 1e30f; int best_g = cur;
        // (in the direction that helps: a gear under the floor is not left for a taller one, which runs lower still, and a gear over the cap not for a shorter one. From a standstill the
        // current gear sat at its rpm of the speed of now, the others at the 0.5 m/s the landing is judged at, and the "nearest" was the next gear up at 0 km/h: 1 > 2, then the stall rule's
        // 2 > 1, on every pull-away that nobody holds (a converter truck, where LowSpeed does not own the launch; the sims). WP-TC1, item 4: TrueShift owns the standstill.)
        const float cur_r = dbg->t[cur].rpm;
        for (int g = 1; g <= n; g++) {
            if (!((in->allowed >> g) & 1)) continue;
            if (g > cur && cur_r < b->floor) continue;
            if (g < cur && cur_r > b->cap) continue;
            float r = dbg->t[g].rpm;
            float d = r < b->floor ? b->floor - r : r > b->cap ? r - b->cap : 0.0f;
            if (d < best_dist) { best_dist = d; best_g = g; }
        }
        dbg->best = best_g;
        dbg->why = GC_NEAREST;
        return dbg->best;
    }

    // (computed here for the score below)
    const int near_wall = cur_rpm >= wall;
    // (the horizon of a pedal at its end is timed on the strongest gear in the window, so the same truck at the same speed gets the same horizon whichever gear it is in: 12 at 1413 rpm planned to
    //  84 km/h and chose 11, 11 at 1750 planned to 78 km/h and chose 12, every 3 s on a 4 % climb with a 24 t truck, when the horizon was timed on the gear's own force)
    const float kd_h = (lift || in->controller) ? 0.0f : in->intent.tgt_kd;
    const float wish_t = kd_h > 0.0f && strongest > 0.0f ? wish + kd_h * (strongest - wish) : wish;
    const float a_w = fmaxf(0.0f, (wish_t - f_res) / m);
    const float v_t = gc_horizon_speed(in->allowed, n, v, f_res, m, wish_t, b->cap);
    const int span = 1 + (in->skips > 0 ? in->skips : 0);
    const gc_term *tcur = &dbg->t[cur];

    // Qualification (hard, inside layer 3): the wish within the intent's per-band share of the gear's full-throttle
    // force -- land shares for a target, run shares for the current gear. How deep in the lug band a gear may sit beyond that is a price
    // (gc_expect, in the score below), not a veto.
    int qualifies_any = 0;
    for (int g = 1; g <= n; g++) {
        gc_term *t = &dbg->t[g];
        if (!t->allowed || !t->in_win) continue;
        const float *share = g == cur ? in->intent.max_share_run : in->intent.max_share_land;
        // (At the end of the pedal the wish is everything, which no gear lands lower and delivers: the ask fades out with the kickdown, so every gear inside the window is a candidate the plan
        //  below times, as the full-throttle plan did before the pedal was a target. A gear that lugs pays for it in the plan's time.)
        t->qualifies = share[t->band] > 0 && (1.0f - (lift || in->controller ? 0.0f : in->intent.tgt_kd)) * gc_ask(in, wish_q, t->rpm) <= share[t->band] * t->force;
        if (t->qualifies) qualifies_any = 1;
    }
    if (!qualifies_any) {
        // Nothing can deliver the wish: keep the current gear when it qualifies on the run shares (or is only 10 % short:
        // a momentary short at full throttle is no reason to flip back and kick down), else the strongest gear in the
        // window (a kickdown).
        const gc_term *tc = &dbg->t[cur];
        float ms = in->intent.max_share_run[tc->band];
        int keep = tc->allowed && tc->in_win && ms > 0 && gc_ask(in, wish, tc->rpm) <= ms * (1.0f + GC_SHARE_BAND) * tc->force;
        int pick = strongest_g;
        if (!keep && a_w > 0.02f) {
            // The wish is an acceleration nobody delivers on the shares: the gear is the one that gets the truck to the horizon speed soonest,
            // the shift and the shifts after it counted (the same plan as the full-throttle choice above). The strongest gear of the moment
            // is not that gear: one that is past its crossover by the time it is in (9 at 1840 rpm gives less than 10) is left again
            // within seconds, and the two took turns (Power on a 6 % climb at the limit: 10 -> 9 -> 10 every 10 s).
            float bt = 1e30f; pick = 0;
            for (int g = 1; g <= n; g++) {
                const gc_term *t = &dbg->t[g];
                if (!t->allowed || !t->in_win) continue;
                float ss;
                float tt = (g != cur ? ts : 0.0f) + gc_gain(g, n, g == cur ? v : v_after, v_t, a_w, f_res, cross, b->p1, ts, m, span, &ss, dur, decel);
                if (g == cur) tt -= stay_b;
                if (g == in->prev && g != cur) tt += ts * in->prev_w;
                if (tt < bt) { bt = tt; pick = g; }
            }
            if (!pick) pick = strongest_g;
            if (pick == cur) keep = 1;
        } else if (!keep && tc->allowed && tc->in_win && strongest_g && strongest_g != cur &&
                   (dbg->t[strongest_g].force - tc->force) * GC_HORIZON <= f_res * ts) {
            // (no acceleration wished: a kickdown must pay for its shift, the force it adds over the horizon against the speed the shift loses)
            keep = 1;
        }
        dbg->best = keep ? cur : pick;
        dbg->why = keep ? GC_KEEP : GC_KICKDOWN;
        return dbg->best;
    }

    // Floored: an upshift only where the next gear gives at least the current gear's power (the crossover), or the
    // wall is on. Power at the same speed is force x v, so the forces compare.

    // The score, in seconds. The horizon: where the wish takes the speed in GC_HORIZON seconds, capped at GC_HORIZON_SPEED
    // (the ~20 s of a launch, not the power wish's terminal speed; holding a speed: no horizon, the shift term alone).

    // The demands still to come (gc_expect): the wish now (pedal off: the demand that is still expected), the road ahead with the time each stretch
    // arrives (the force that holds the speed there plus what the wish adds to holding it now), and with the pedal off the demand that was lifted,
    // kept as a reserve with the probability that it is asked again (a gear that cannot deliver it pays the shift it will need).
    gc_demand dem[AHEAD_MAX + 4 + DEMAND_LEVELS_MAX]; int nd = 0;
    if (!lift) dem[nd++] = { 0.0f, wish, f_res, 1.0f, 0.0f, 0 };   // (with the pedal off nothing is asked now: the history below is the reserve)
    if (in->road && in->road->n_samples > 0) {
        const float extra = lift || in->controller ? 0.0f : fmaxf(0.0f, wish - f_res);
        for (int i = 0; i < in->road->n_samples && i < AHEAD_MAX; i++) {
            if (in->road->sample_s[i] > GC_HORIZON) break;
            dem[nd++] = { in->road->sample_s[i], in->road->sample_resist_n[i] + extra, in->road->sample_resist_n[i], pw, 0.0f, 0 };
        }
    }
    // The driver's history: the largest demand to come is one of the levels, mutually exclusive. It is what a HELD speed leaves open: while the wish is mostly
    // acceleration (hold_share near 0) the demand is the driver's own and known, and the lugging allowance and the wish above are its price; as the wish turns
    // into holding the speed (hold_share 1) the next demand is anybody's, and the history decides how much reserve the gear keeps. So the probabilities are
    // weighted by hold_share. With the pedal off nothing is known: the whole history counts, judged at full throttle (no lugging allowance for a reserve).
    {
        const float hs = lift || in->hist_full ? 1.0f : gc_clamp(in->intent.hold_share, 0.0f, 1.0f);
        for (int i = 0; i < in->hist_n && i < DEMAND_LEVELS_MAX; i++) dem[nd++] = { 0.0f, in->hist_f[i], f_res, in->hist_p[i] * hs, lift ? 1.0f : 0.0f, 1 };
    }

    // shift (skip-aware) + the acceleration time per qualifying gear; accel is made relative to the fastest below. The
    // acceleration time is LOCAL: how long this gear's pull (capped at the wish) takes to gain the horizon speed, so a
    // gear that lugs (its force under the wish) or pulls less pays the seconds it loses, while a gear that delivers the
    // wish is not penalised.
    // The pedal at the end of its travel is timed by the fastest way to the horizon speed (the value function above: every shift after the landing chosen best), where a part pedal is timed by the
    // pull at a constant acceleration and the shifts counted by gc_gain's rule: the same score, the accel term and the later shifts swapped for the plan's time by how far into the kickdown the
    // foot is (a full pedal is everything the truck can give: the time decides, and a light many-gear box that skips three gears on a climb is the fastest sequence, not the nearest to P3).
    const float kd = (lift || in->controller) ? 0.0f : in->intent.tgt_kd;
    float plan_t[GC_MAX + 1]; float plan_min = 1e30f; bool plan_ok = false, cur_imm = false, plan_filled = false;
    for (int g = 0; g <= GC_MAX; g++) plan_t[g] = 1e30f;
    if (kd > 0.0f && v_t > v + 0.5f) {
        gc_plan *const plan = &s_plan;
        gc_plan_fill(plan, n, b, v_after, v_t, v, ts, ts > 0 ? (f_res / m + decel) * dur : 0.0f, f_res, m, span);
        plan_filled = true;
        for (int g = 1; g <= n; g++) {
            if (!dbg->t[g].qualifies) continue;
            float tp = (g != cur ? ts : 0.0f) + gc_plan_at(plan, g, g == cur ? v : v_after);
            // The gear we are in, when the fastest way from it shifts at once (or it cannot be held a step further), is not a place to stay: its plan holds that shift at the model's price, a gear
            // cheaper than the real shift to the same gear. It stays only if nothing else is a candidate.
            if (g == cur && gc_plan_shift(plan, cur, v) < gc_plan_hold(plan, cur, v) - 1e-3f) cur_imm = true;
            if (tp < 1e8f) { plan_t[g] = tp; plan_min = fminf(plan_min, tp); }
        }
        plan_ok = plan_min < 1e8f;
    }
    // The pedal's operating point over the path of each gear (gc_path_solve): the miss of every second, every shift on the way, the speed the gear gives at this pedal. It stands for the target's
    // distance, the shifts a gear needs later (gc_gain's count) and the landing's commitment, which all priced the same future three ways; with no point to be at (the pedal off, a controller)
    // those three are as they were.
    float path_v[GC_MAX + 1]; bool path_on = false;
    for (int g = 0; g <= GC_MAX; g++) path_v[g] = 0.0f;
    // How much the time lost to a dip weighs with this foot: a calm foot values the seconds little and the point a lot ("a calm pedal means low revs; if I want to pull harder I press more"), a
    // pressed one wants the time. The share of the way the foot has gone up toward the mode's full-pedal point (tgt_lo, the number that moves the point), and a press in progress (urgency) asks
    // the time at once; at the end of the pedal the plan's time takes over (kd). It weighs the lateness of the path only: a shift's own ts stays whole, the gap is an interruption whatever
    // the foot (weighed with the foot the box shifted again before the last shift had landed, LS-E, and hopped through the gears at the light pedal, LS-F).
    const float x_foot = gc_clamp(in->intent.tgt_lo, 0.0f, 1.0f);
    const float w_time = gc_clamp(x_foot + (1.0f - x_foot) * gc_clamp(in->intent.urgency, 0.0f, 1.0f), 0.0f, 1.0f);
    // How much of the decision is a hold of the speed (the same smoothstep of 2 hold_share - 1 as the foot a gear would have, below): a hold is not a pull that wins the speed of the gap back, so a gear
    // landed in at a hold is read at the speed the gap leaves the truck at too, and the worse of the two counts (gc_path_solve). A pull keeps the old accounting (the shift's price is ts + the regain),
    // and the weight is continuous in the pedal and the grade.
    const float hp_dip = gc_clamp(2.0f * in->intent.hold_share - 1.0f, 0.0f, 1.0f);
    const float w_dip = hp_dip * hp_dip * (3.0f - 2.0f * hp_dip);
    if (!lift && in->intent.tgt_rpm > 0.0f && (1.0f - kd) > 0.0f) {
        unsigned char cand[GC_MAX + 1];
        for (int g = 0; g <= GC_MAX; g++) cand[g] = g >= 1 && g <= n && dbg->t[g].qualifies;
        // The point each gear is read at: the foot of now for the gear we are in; for any other, as far as the decision is a steady hold, the foot it would have for the same force (gc_in::foot_for). The torque
        // pedal asks level x phi(rpm), so a hold of the speed needs a different raw foot in each gear (k1 41.9 t, 80 km/h: 0.19 in g11, 0.21 .. 0.23 in g12) and a gear read at the foot of the gear we are in is
        // read at a foot it does not have: each gear's own foot put the other on the near side of the edge (Power cycled g11 / g12). A pull is not that: its skip and its kickdown are priced at the foot the driver
        // has (it is his input, the path takes it as it is) and sit on the edge of their price there; the foot a pull would have in another gear moves the edge by 0.5 .. 1 s (D2c, V1 / V2).
        float tg_rpm[GC_MAX + 1], tg_lo[GC_MAX + 1];
        for (int g = 0; g <= GC_MAX; g++) { tg_rpm[g] = in->intent.tgt_rpm; tg_lo[g] = in->intent.tgt_lo; }
        if (in->foot_for && in->phi && in->intent.tgt_foot >= 0.0f) {
            const float ph_cur = in->phi(cur_rpm);
            // How much of the decision is a steady hold: the hold part of the wish, counted from the half where the holding force is more than the pull on top of it (the same 2 hold_share - 1 as the age of the hold,
            // trueshift gear_layers), as a smoothstep so that it has no corner at the half. A pull is priced at the foot the driver has (a skip and a kickdown sit on the edge of their price at the foot they were
            // asked at), a hold of the speed reads each gear at the foot it would have in it: hold_share is continuous in the pedal and the grade, and so is this.
            const float hp_ft = gc_clamp(2.0f * in->intent.hold_share - 1.0f, 0.0f, 1.0f);
            const float hs_ft = hp_ft * hp_ft * (3.0f - 2.0f * hp_ft);
            for (int g = 1; g <= n; g++) {
                if (g == cur) continue;
                const float ph_g = in->phi(vehicle_engine_rpm_at(g, v_after));
                if (!(ph_cur > 0.0f) || !(ph_g > 0.0f)) continue;
                pt_target_at(&in->intent, b, in->foot_for(in->intent.tgt_foot, 1.0f + hs_ft * (ph_cur / ph_g - 1.0f)), &tg_rpm[g], &tg_lo[g]);
            }
        }
        gc_path_solve(in, b, n, cur, v, v_after, w_dip, wish, f_res, m, ts, span, strongest > 0.0f ? strongest : tcur->force, cross,
                      (1.0f - kd) * (1.0f - gc_clamp(in->lat_load, 0.0f, 1.0f)), 1.0f - kd, w_time, cand, tg_rpm, tg_lo, path_v);
        path_on = true;
        dbg->cell_v = s_gpath.dv;
    }
    float best_acc = 1e30f;
    for (int g = 1; g <= n; g++) {
        gc_term *t = &dbg->t[g];
        if (!t->qualifies) continue;
        float ss;
        // the gear's upshift point for the future-shift count: the end of its power band (the crossover, not the no-go cap), so the
        // skips the truck will actually take are counted, and a gear one step up still saves its later shift.
        gc_gain(g, n, g == cur ? v : v_after, v_t, a_w, f_res, cross, b->p1, ts, m, span, &ss, dur, decel);
        float now = 0.0f;                                // the shift now
        if (g != cur) {
            // A shift loses the speed of the gap (dur seconds at the resistance plus the learned deceleration); the gear we land in has to win it back with what
            // it has to spare over the resistance, and a gear with hardly any spare (a marginal lug gear on a climb) wins it back in seconds, or never: that
            // time belongs to the shift, as gc_expect's price of a shift has it (dur + the speed lost regained at the pull).
            const float a_g = fmaxf(0.05f, (t->pull - f_res) / m);
            // ... at the rate the truck will win it back: the acceleration the wish asks when it is a pull (the pull is capped by the wish), the spare force
            // when it only holds the speed (the controller of the driver or the cruise uses what the gear has to spare).
            // (WP-G1b: the game's own cruise with the foot off is that second case whatever its throttle sits at. Its throttle is a proportional controller's output, not a pull
            // the driver asks for, and it rises with the speed error, so the speed a shift loses is won back at what the gear has to spare. Timed at the throttle's excess a cruise
            // that holds the speed on 0.3 .. 0.6 of the gear's force (excess 0.03 m/s2) priced a shift at 5 .. 10 s, and a gear in the power band beat a gear in the eco band on a
            // speed ramp whose upshift saved 3 s of taste: Eco ramp, the game's cruise, 1202 -> 1430 rpm on the 16-pair subset, fuel +3.1 %; with this 1150 rpm, fuel -0.8 %.
            // The plugin's own cruise and the planner ask a force they plan and stay on their excess: with the regain at the gear's spare run_pcc's hard-braking lead ran into
            // the lead (flat, 40 t, 85 km/h: 51 shifts, min gap 0 m, against 42 and 7.1 m).)
            const float a_reg = a_w > 0.02f && !lift && !in->game_cruise ? fminf(a_w, a_g) : a_g;   // (the lift: the spare of the gear, as gc_expect prices the shift it saves)
            // (A shift NOW in a pull stays a duration, dv / a: the pull is a speed to reach. Priced as the distance a dip costs, as the later shifts are, the pedal-on
            // shift got so cheap that a marginal lug gear on a climb was taken and left again: 18b hunts 2 -> 18 a mode, t70 +30 %. A net descent gains speed in the gap:
            // nothing to regain.)
            const float regain = fmaxf(0.0f, f_res / m + decel) * dur / a_reg;
            // pedal off: the shift now costs the drive it loses, the expected drive lost if the pedal returns during the shift (lift_scale: full while the
            // lift is shorter than a shift, falling after it); the dur it takes and the regain
            // (The shift now at a lift is the same price gc_expect charges a gear that must shift later, so it carries the same weight: with a response weight over 1
            // the reserve a lift saves would otherwise outweigh the shift even when the lift is shorter than one and a blip would shift.)
            // (Under a limiter the speed the shift loses is not won back for anything: the limit is the speed, and a dip below it is a distance, like the dips of the later shifts; a duration at the
            //  spare of the gear priced a shift at the limit at 5 .. 14 s, and the gear that holds it at 380 rpm less was never taken.)
            now = lift ? in->intent.expect_weight * in->lift_scale * gc_shift_price(dur, decel, m, f_res, strongest, wish, v)
                       : path_on ? ts : in->capped ? gc_shift_price(dur, decel, m, f_res, strongest, wish, v) : ts + regain;
        }
        // the shift now + the ones the gear needs before the horizon. With the pedal off the plan's shifts are the pull's, decided when the pedal is back and the
        // reserve (gc_expect) has said whether the gear can deliver it at all: shifting up now to save a later shift of a pull that this gear could not make
        // was the sprint churn (a shift in the lift, a kickdown at the re-press)
        const float k_plan = plan_ok && plan_t[g] < 1e29f ? kd : 0.0f;   // (how much of the time of this gear is the plan's)
        t->shiftc = (1.0f - k_plan) * (now + (lift || path_on ? 0.0f : ss));
        if (g == in->prev && g != cur) { t->shiftc += ts * in->prev_w; dbg->prev_price = ts * in->prev_w; }  // anti-hunting: back to the gear just left = its shift time twice, while the situation of that shift is still there (prev_w)
        t->shifts = (g != cur ? 1 : 0) + (int)(ss / ts + 0.5f);
        float a_eff = a_w > 0.02f ? fminf(a_w, fmaxf(0.02f, (t->pull - f_res) / m)) : 0.0f;
        // A landing is a commitment (WP-G7): a gear taken in a pull and left again upward within the time a shift needs to be told from a hunt (hold_s, the anti-hunt dwell) costs the drive a
        // second shift for nothing, and a chain of such landings is the "crawl gear and gear 1 a second later" the owner dislikes (LS-F; "never landed" in LS-E: the next shift is asked
        // before the last is in). The plan counts the shifts a gear needs before the horizon but not WHEN: a gear that lands low, with the truck pulling hard, is at the end of its range
        // in a second. The time the landing is held is the rpm it climbs to before the plan leaves it (its crossover, or the limiter wall) over the rate it climbs at (its rpm x the
        // acceleration the gear gives, capped by the wish, over the speed), and the shift pays its own time for the part of hold_s it falls short of. A calm pull climbs slowly (the 190.49
        // landing at 750 rpm: 8 s to the eco band's end): nothing is charged; a hard pull from a crawl charges the low landings and the skip that lasts wins. Not for the top gear (it
        // is not left), not with the pedal off (the shift there is the lift's own price), not for a controller's wish.
        if (!lift && !path_on && g > cur && g < n && a_eff > 0.0f && !in->controller && !in->game_cruise) {
            const float rate = t->rpm * a_eff / fmaxf(v_after, 0.5f);                  // rpm/s the landing gear climbs at
            const float leave = fminf(cross[g], wall);                                 // where the plan leaves it
            const float t_leave = leave > t->rpm ? (leave - t->rpm) / fmaxf(rate, 1.0f) : 0.0f;
            t->shiftc += (1.0f - k_plan) * ts * gc_clamp(1.0f - t_leave / fmaxf(in->hold_s, 0.5f), 0.0f, 1.0f);
        }
        t->accc =a_eff > 0 && !lift ? (v_t - v) / a_eff : 0.0f;   // (with the pedal off the pull that may come back is gc_expect's reserve already: not twice)   // the gear's pull, measured from the same start (the shift's own
                                                           // speed loss belongs to the shift term, not to the acceleration)
        if (t->accc < best_acc) best_acc = t->accc;
        // The fuel the gear burns over the time it is expected to be held (gc_fuel_litres). The hold: the plan horizon, and as long as such a hold lasts when the cruise
        // holds the speed with the foot off (band_scale: about as long again as it has lasted). Of it only the part that holds the speed is a hold of THIS gear: a pull
        // changes gear as the speed rises and its time is the accel term's (the same share of the horizon as the band taste has: hold_share; priced on all of it the
        // fuel made every pull a taller, slower one: t70 +4.5 % for -1.4 % fuel at 3 s per litre). With the pedal off the game cuts the fuel in gear (0 l/h) and only
        // the demand that comes back burns any, with its probability and for the share of holding the speed it delivers; the coast itself burns nothing. The demand it is
        // judged on is the wish (the expected demand at a lift).
        {
            const float hs = gc_clamp(in->intent.hold_share, 0.0f, 1.0f);
            const float on = lift ? pw * fminf(hs, f_res > 1.0f ? wish / f_res : 1.0f) : hs;
            t->fuel_l = gc_fuel_litres(t->rpm, fuel_k(), V.eng.torque, wish, v, V.eta, GC_HORIZON * fmaxf(1.0f, in->band_scale), on);
        }
    }
    // (the cheapest candidate burns what the others are charged the excess over)
    float fuel_min = 1e30f;
    for (int g = 1; g <= n; g++) {
        const gc_term *t = &dbg->t[g];
        if (!t->qualifies) continue;
        if (t->fuel_l < fuel_min) fuel_min = t->fuel_l;
    }

    float best_score = 1e30f; int best_g = 0;
    for (int g = 1; g <= n; g++) {
        gc_term *t = &dbg->t[g];
        if (!t->qualifies) continue;
        t->candidate = 1;
        { const float k_plan = plan_ok && plan_t[g] < 1e29f ? kd : 0.0f;
          t->accc = (1.0f - k_plan) * (t->accc - best_acc) + k_plan * (plan_t[g] - plan_min + (g == cur && cur_imm ? 100.0f : 0.0f)); }   // the extra time vs the fastest gear
        // The band wish is a cruise preference: it fades while the wish accelerates the truck (hold_share ~ 0), so a
        // launch is free to rev for power and to lug a little for fewer shifts; a held speed pays the full band cost.
        float bw = gc_clamp(in->intent.hold_share, 0.0f, 1.0f);
        // A calm pedal is a calm engine, in every mode (WP-G7, the owner after the ride of 04-11: at a pedal of about 10 % the box went to 1600-2000 rpm and skipped 3 -> 7). hold_share alone
        // fades the taste of every pull, however little the pedal asks: at 190.49 a 16.5 t at 15 km/h asked 9.4 kN of the 90-100 kN its gear gives and the wish was 87 % acceleration, so the
        // taste weighed 0.13 and the revs were free. A pedal that asks little of the gear needs no revs to be delivered: whatever the gear runs above the least it can is for nothing. So the
        // weight is the larger of two shares of the same wish: the part that holds the speed (the revs serve the hold) and the idle capacity of THIS gear at the rpm it would run at, one minus
        // the ask over its full-throttle force. A full ask leaves 0 and the weight is as it was (a pull that needs the engine keeps its freedom to rev and to skip: the full pedal does not come
        // here at all, the plan above takes it); a hold is 1 either way. Continuous in the pedal and the grade (a max of continuous shares, no step), no number of its own except three limits:
        //  - the idle capacity enters squared (an empirical choice, not a derivation: the lowest power that keeps the launch from rest as it was. A 41 t DC16 at a quarter pedal asks 0.4 of what
        //    its first gear gives, idle 0.6, weight 0.6 with the power 1: the first shift came 160 rpm earlier, a gear less far, and the chain 1 > 3 > 5 > 6 > 8 > 10 > 12 hopped (LS-F / LS-E, the
        //    owner's "a crawl gear and gear 1 a second later"); squared it is 0.36 and with the hop cost below the chain is the old one. Tried instead, both break LS-F / LS-E on the same launch: the idle itself (the manager's form, 1 - ask / F of the candidate) and the idle of the gear times the idle of the truck (1 - wish / the strongest gear's force); the history's chance of a bigger demand cannot help, the gear we are in at the launch holds every level),
        //  - times what the mode's own switch from the eco band to the power band leaves (a product with the idle capacity, not a cap: both must hold): 1 - need / switch point (need = the demand against what the eco band can deliver, the switch
        //    point 1 / expect_weight, intent.cpp): where the demand takes that much of the eco band the mode wishes the power band and a taste for low revs is no longer the point, so a loaded
        //    truck, a climb or a driver who has pulled hard in the last minutes get the weight they had (the 40 t on rolling hills of run.sh B4 shifted 15 times against 10 without it, and X1, the same road with a soft reset every 7 s, took one hunting pair at a tie of 0.55 against 0.55 when the room was a cap of 0.2 instead of a factor); Power
        //    switches earlier than Eco and so lets go of the weight earlier,
        //  - a bend takes it away (1 - lat_load): the stay bonus of the bend keeps the gear there (18i-5, the roundabout), the same factor the hold's extension has.
        // Only the driver's foot: a controller's wish (the cruise, the planner, the game's own cruise with the foot off) is a throttle of the moment and keeps hold_share.
        if (!lift && !in->controller && !in->game_cruise) {
            const float idle = gc_clamp(1.0f - gc_ask(in, wish, t->rpm) / fmaxf(t->force, 1.0f), 0.0f, 1.0f);
            const float room = gc_clamp(1.0f - in->intent.need * in->intent.expect_weight, 0.0f, 1.0f);
            bw = fmaxf(bw, idle * idle * room * (1.0f - gc_clamp(in->lat_load, 0.0f, 1.0f)));
        }
        // The taste is for a demand that is delivered: with the pedal off it is the demand expected back, with its probability, and a demand under the holding
        // force (a coast) delivers only its share of holding it (the tent: 1 at the holding force, E / R under it, R / E over it).
        if (lift) bw = pw * fminf(bw, f_res > 1.0f ? wish / f_res : 1.0f);
        else bw *= fmaxf(1.0f, in->band_scale);
        t->bandc = (t->band == PT_ECO ? pt_eco_cost(&in->intent, b, t->rpm, g == cur)
                                      : g == cur ? in->intent.band_cost_run[t->band] : in->intent.band_cost_land[t->band]) * bw;
        // above the eco band the cost grows with the depth: a band the mode does not wish for gets worse the deeper the rpm goes
        // into it (Eco in the power band: just over its start is less bad than near the top). A band the mode wishes for (cost 0) stays 0.
        if (t->band >= PT_POW && t->band <= PT_TMP && b->cap > b->p2 + 1.0f)
            t->bandc *= 1.0f + gc_clamp((t->rpm - b->p2) / (b->cap - b->p2), 0.0f, 1.0f);
        if (path_on) t->bandc = 0.0f;   // (its taste is a rate of the path, gc_path_solve)
        t->stay = g == cur ? stay_b : 0.0f;
        // The pedal's operating point (WP-0911-K1): the value of the path this gear takes (gc_path_solve above): the miss of every second, the shifts on the way, the speed it gives at this pedal.
        // The pedal off has no foot (the lift's own terms price the demand that may come back); a controller's throttle has no point. At the end of the pedal the weights fade the path out
        // over the kickdown's smoothstep and the plan's time decides (a full pedal is everything the truck can give: landings and skips by time); a bend takes the taste away as it takes the
        // band taste away (the stay bonus of the bend keeps the gear until the exit, 18i-5, the roundabout rows).
        t->tgtc = path_on ? path_v[g] : 0.0f;
    }
    // The band taste is charged for the whole horizon, but a gear that fails a demand is left for another one then: until the demand it keeps its own band, after it the
    // successor's (the cheapest candidate that does deliver that demand). Charged whole, a tall gear that holds the flat for three seconds and must go down at the foot
    // paid 20 s of the cheap taste against the lower gear's 20 s of the dear one, took the shift up first and came down at the foot (the known climb, 13 -> 14 -> 12 -> 14
    // -> 12), and a gear that could not hold the climb in front of the truck won by a taste it could not keep (the cruise on +4 %, 13 -> 14 -> 13 every few seconds).
    float succ[AHEAD_MAX + 4 + DEMAND_LEVELS_MAX];
    for (int k = 0; k < nd; k++) {
        succ[k] = 1e30f;
        for (int g = 1; g <= n; g++) {
            const gc_term *t = &dbg->t[g];
            if (!t->candidate) continue;
            const float *sh = g == cur ? in->intent.max_share_run : in->intent.max_share_land;
            const float cap = (dem[k].share > 0 ? dem[k].share : sh[t->band]) * t->force;
            if (dem[k].f <= cap * (1.0f + GC_SHARE_BAND) && t->bandc < succ[k]) succ[k] = t->bandc;   // (delivers it, or short by the ramp gc_expect fades the price over)
        }
    }
    // (D5b) the full pedal's value function, for the price of staying in a gear that falls short of the reserve (gc_expect, stay_loss). The plan of the pull itself (kd above) is the same
    // function when the foot is at the end of its travel; the reserve needs it in the hold too, where nobody asks for the pull yet. (D5c) It is filled when the first price asks for it (a third
    // of the decisions never do), and the pull's own plan is used when it is the same function (the same horizon).
    struct stay_ctx {
        const gc_in *in; gc_term *t; int n, cur; bool lift, plan_filled; float plan_v_t, v, v_after, f_res, m, strongest, ts, decel, dur; const eng_bands *b; int span;
        gc_plan *plan; bool filled, ok; int g;
    } sc = { in, dbg->t, n, cur, lift, plan_filled, v_t, v, v_after, f_res, m, strongest, ts, decel, dur, b, span, &s_plan_r, false, false, 0 };
    auto stay_loss_of = [](void *p) -> float {
        stay_ctx *c = (stay_ctx *)p;
        if (!c->filled) {
            c->filled = true;
            if (!c->lift && !c->in->controller && c->in->hist_n > 0) {
                const float v_tr = gc_horizon_speed(c->in->allowed, c->n, c->v, c->f_res, c->m, c->strongest, c->b->cap);
                if (v_tr > c->v + 0.5f) {
                    if (c->plan_filled && v_tr == c->plan_v_t) c->plan = &s_plan;
                    else gc_plan_fill(c->plan, c->n, c->b, c->v_after, v_tr, c->v, c->ts, c->ts > 0 ? (c->f_res / c->m + c->decel) * c->dur : 0.0f, c->f_res, c->m, c->span);
                    c->ok = true;
                }
            }
        }
        if (!c->ok) return 1e30f;
        const float vg = c->g == c->cur ? c->v : c->v_after;
        const float tg = gc_plan_at(c->plan, c->g, vg);
        float tb = 1e30f;
        for (int h = 1; h <= c->n; h++) if (c->t[h].candidate) tb = fminf(tb, gc_plan_at(c->plan, h, vg));
        return tg < 1e8f && tb < 1e8f ? fmaxf(0.0f, tg - tb) : 1e30f;
    };
    for (int g = 1; g <= n; g++) {
        gc_term *t = &dbg->t[g];
        if (!t->candidate) continue;
        const float *share = g == cur ? in->intent.max_share_run : in->intent.max_share_land;
        // The demands this gear is asked: the pedal's ask scales with phi of the rpm it runs at, the holding part of every demand (the resistance) does not. The demand
        // now is exactly what the gear is asked (wish x phi); the pedal part of the road ahead and of the history rides on top of the resistance of its moment.
        gc_demand dg[AHEAD_MAX + 4 + DEMAND_LEVELS_MAX];
        memcpy(dg, dem, sizeof(gc_demand) * nd);
        if (in->phi) {
            const float ph = in->phi(t->rpm);
            for (int k = 0; k < nd; k++)
                dg[k].f = (k == 0 && !lift) ? dem[k].f * ph : dem[k].r + fmaxf(0.0f, dem[k].f - dem[k].r) * ph;
        }
        float band_after = 0.0f;
        const float k_plan = plan_ok && plan_t[g] < 1e29f ? kd : 0.0f;   // (the plan's time is the whole future of this gear)
        sc.g = g;
        t->expc = (1.0f - k_plan) * in->intent.expect_weight * gc_expect(t->force, strongest, share[t->band], dur, decel, m, dg, nd, v, succ, t->bandc, GC_HORIZON * fmaxf(1.0f, in->band_scale), &band_after, k_plan < 1.0f ? stay_loss_of : nullptr, &sc, lift ? 0.0f : vehicle_kmh_at(g, b->p3) / 3.6f,
                                                                   gc_clamp((t->rpm - b->floor) / fmaxf(b->p1 - b->floor, 1.0f), 0.0f, 1.0f));
        t->bandc -= band_after;
        t->bandc *= 1.0f - k_plan;
        t->fuelc = path_on ? 0.0f : (1.0f - k_plan) * in->intent.fuel_weight * (t->fuel_l - fuel_min);   // (on the path the fuel is a rate of the path)
        t->score = t->shiftc + t->accc + t->bandc + t->expc + t->fuelc + t->tgtc - t->stay;
        if (t->score < best_score) { best_score = t->score; best_g = g; }
    }
    if (!best_g) {
        // Nothing scored (every candidate was blocked): keep the current gear if allowed, else the strongest.
        best_g = (in->allowed >> cur) & 1 ? cur : strongest_g;
        dbg->why = GC_KEEP;
        dbg->best = best_g;
        return dbg->best;
    }

    // The gear we are in fails the shares (the reserve) but the best gear that passes them is another one: the shares are what makes a gear
    // deliver the wish, the drive time says whether leaving it pays. A gear that passes only by being a gear down is often past its
    // crossover seconds later and the full-throttle choice above takes the truck back up (Power, DC16 at 18 t on a 6 % climb at the limit:
    // 13 -> 12 on the shares, 12 -> 13 at the crossover, every 10 s). So the current gear stays when its plan gets the truck to the horizon
    // speed no later than the best gear's, shifts counted.
    if (best_g != cur && !near_wall && a_w > 0.02f && tcur->allowed && tcur->in_win && !tcur->qualifies) {
        float ss1, ss2;
        float t_cur = gc_gain(cur, n, v, v_t, a_w, f_res, cross, b->p1, ts, m, span, &ss1, dur, decel) - stay_b;
        float t_best = ts + gc_gain(best_g, n, v_after, v_t, a_w, f_res, cross, b->p1, ts, m, span, &ss2, dur, decel) + (best_g == in->prev ? ts * in->prev_w : 0.0f);
        if (t_cur <= t_best) best_g = cur;
    }

    // The wall: near the top the current gear's best upshift is forced (full pull to the limit, nothing above).
    if (near_wall) {
        int up = 0; float up_score = 1e30f;
        for (int g = cur + 1; g <= n; g++) {
            gc_term *t = &dbg->t[g];
            if (!t->candidate) continue;
            // only a gear that still pulls after the shift: at the truck's top speed (the wish no longer gains speed in
            // this gear) the next gear would lose speed, kick back down and hunt; the current gear stays at the top of
            // its band instead (the rev limit itself is the caller's hard rule)
            if (!(t->force > f_res)) continue;
            // ... and the pedal must still hold the speed in it (WP-G7, R3: 41.9 t on +2 % at a quarter pedal, 7 -> 11 at the wall in a near tie, 14.05 against 14.2 for 10): the torque pedal asks a gear
            // level x phi of its rpm, and in 11 at 831 rpm that was 9.75 kN against 11.45 kN that hold the road, so the gear "pulled" (15.5 kN at full throttle) and the truck sagged 52 -> 44 km/h
            // until 11 -> 10 forty-seven seconds later: the hunt this test is for, with the force the pedal really gives. The wish that holds or pulls the road now (wish over f_res) must keep
            // at least what holds it, less the 10 % the gears' own ramp allows (GC_SHARE_BAND); a pedal that holds nothing (wish under f_res) asks nothing of the gear and is not tested.
            if (!lift && wish > f_res && gc_ask(in, wish, t->rpm) < f_res * (1.0f - GC_SHARE_BAND)) continue;
            if (t->score < up_score) { up_score = t->score; up = g; }
        }
        if (up) { dbg->best = up; dbg->why = GC_WALL; dbg->skip = up - cur - 1 > 0 ? up - cur - 1 : 0; return dbg->best; }
    }

    dbg->best = best_g;
    dbg->why = best_g == cur ? GC_KEEP : best_g > cur ? GC_UP : GC_DOWN;
    dbg->skip = best_g > cur + 1 ? best_g - cur - 1 : 0;
    return dbg->best;
}

// The gear the fastest way goes up to when the revs are at the wall at a full pedal (the rev-limit rule's target, TrueShift): the landing of up to `span` gears above cur that
// the plan above values lowest, a landing that lies inside the window [floor, cap]; cur + 1 when none does. The skip rule it replaces (skip_target) took no skip on a grade and
// the farthest one that landed on the plateau elsewhere, so a light truck on a 4 % climb stepped 3 > 4 > 5 > 6 > 7 in the first 25 km/h where the fastest way is 3 > 6 > 8.
int gc_full_upshift(int cur, int n, float v, float shift_time, float shift_dur, float shift_decel, int span, float tgt_rpm, float tgt_w)
{
    const eng_bands *b = bands_now();
    if (!b->ok || cur < 1 || cur >= n || !(V.mass > 0) || !(v > 0.1f)) return cur + 1;
    const float m = V.mass, ts = shift_time, dur = shift_dur > ts ? shift_dur : ts, decel = fmaxf(0.0f, shift_decel);
    const float f_res = vehicle_resist(v);
    const float lose = ts > 0 ? (f_res / m + decel) * dur : 0.0f, v_after = fmaxf(0.5f, v - (f_res / m + decel) * dur);
    unsigned long long all = 0;
    for (int g = 1; g <= n && g <= GC_MAX; g++) all |= 1ull << g;
    float strongest = 0;
    for (int g = 1; g <= n; g++) {
        const float r = vehicle_engine_rpm_at(g, g == cur ? v : v_after);
        if (r >= b->floor && r <= b->cap) strongest = fmaxf(strongest, vehicle_force_at(g, g == cur ? v : v_after, 1.0f));
    }
    const float v_t = gc_horizon_speed(all, n, v, f_res, m, strongest, b->cap);
    if (!(v_t > v + 0.5f)) return cur + 1;
    gc_plan *const plan = &s_plan;
    gc_plan_fill(plan, n, b, v_after, v_t, v, ts, lose, f_res, m, span);
    int best = cur + 1; float bt = 1e30f;
    for (int h = cur + 1; h <= cur + span && h <= n; h++) {
        const float r = vehicle_engine_rpm_at(h, v_after);
        if (r < b->floor || r > b->cap) continue;
        // (the same price as the score's target term: a landing far from where the foot asks the engine to be is left again by the layers a second later, 15 -> 13 -> 15 -> 14 -> 16 on a climb)
        const float t = gc_plan_at(plan, h, v_after) + (tgt_rpm > 0.0f ? tgt_w * fabsf(r - tgt_rpm) / fmaxf(b->p3 - b->p1, 1.0f) * GC_HORIZON : 0.0f);
        if (t < bt) { bt = t; best = h; }
    }
    return best;
}
