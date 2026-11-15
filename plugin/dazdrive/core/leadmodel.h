// The lead model (WP-SP3, docs/cruise-layers.md): ONE source of truth for the vehicle ahead. PURE: no game call, header only.
//
// What it is given each tick is what the module's sense side read: the gap and the speed of the vehicle ahead from the game's radar (the hook; on 1.61 a lead exists only
// when the radar confirms it, FX1: the raw lead field is a phantom there), our own speed, and, from the Cutin reader, the vehicle it predicts to merge into our lane. What it
// gives back is the facts that the ACC (core/req_acc.h) and, later, the AEB (SP4) need, filtered ONCE:
//   present / valid / reason   a lead we can follow (gap and speed), or why there is none (never silent)
//   gap, lv, opening, closing, ttc, age
//   a_lead                      the lead's acceleration, filtered WITHOUT LAG where a braking lead matters (see below)
//   w_brake, w_tense            0..1 weights: the lead is braking; the gap is tense (braking, or a short time to collision). Weights, not switches (anti-knik)
//   crossing, cross_w           a car crossing in front (a roundabout, a junction) is passed, not followed; cross_w is how sure the pattern is, 0..1, growing from the first tick it shows (a car that is not crossing never shows it)
//   appeared / jump_m           the tick a lead appeared or was replaced by a nearer one (a cut-in seen by the radar); a new lead never looks like a lead that brakes
//   merger_*                    the vehicle Cutin predicts to merge (LOG ONLY: nothing acts on it in this package)
// The classic ACC (modules/cruise.cpp own_acc) and the AEB keep their own reads: this model has its own state and is updated only with engine = layers.
//
// The acceleration of the lead. The classic estimate was an exponential over 0.5 s of the one-tick slope: it reaches 63 % of a braking lead after 0.5 s and 90 % after 1.15 s,
// and a braking lead is where a late estimate costs the most. Here the slope is a least-squares fit over the last LM_WIN_S seconds (a window, not a tick: the game's AI vehicles
// change their speed in small steps, which one tick of a slope turns into spikes), and the smoothing is lopsided: it follows a growing deceleration fast (LM_TAU_BRAKE_S) and
// lets it go slowly (LM_TAU_RELEASE_S). Rows in sim_speed.cpp (lead_rows): 90 % of a 3 m/s2 braking step in the time the constants say; a steady lead with speed noise stays
// under the noise floor of the weights.
#pragma once
#include <math.h>
#include <string.h>

enum { LMR_OK = 0,        // a lead we can follow
       LMR_NONE,          // the radar sees no vehicle
       LMR_NO_HOOK,       // the radar hook is not in: there is no source for a lead (1.61), nothing is followed
       LMR_STALE,         // the radar did not answer in the last 0.3 s
       LMR_NO_SPEED,      // a vehicle is there but its speed does not match the gap: present, not followed
       LMR_N };
static inline const char *lm_reason_name(int r)
{
    static const char *const n[LMR_N] = { "a lead", "the radar sees no vehicle", "the radar hook is not in", "the radar did not answer within 0.3 s", "the vehicle's speed does not match its gap" };
    return r >= 0 && r < LMR_N ? n[r] : "?";
}
enum { LMS_OK = 0, LMS_NO_HOOK, LMS_STALE };   // the module's read of the source

// ---- the constants, each with where it comes from
static const float LM_WIN_S = 0.35f;            // the slope window: about two of the game's AI speed steps (they change every ~0.15 s), so a single step is not a spike
static const float LM_TAU_BRAKE_S = 0.12f;      // follow a growing deceleration with this time constant: 90 % of a braking step within 0.3 s (the classic: 1.15 s)
static const float LM_TAU_RELEASE_S = 0.5f;     // let it go with the classic's 0.5 s (a lead that stops braking is no threat; the noise stays out)
static const float LM_BRAKE_FLOOR = 0.3f;       // m/s2: below this a lead is not braking (the classic's radar speed noise floor, ts4t 505-516 s)
static const float LM_BRAKE_SPAN = 1.2f;        // m/s2: the weight is whole at 1.5 m/s2 (the classic: 0.3 + 1.2)
static const float LM_TTC_FROM_S = 6.0f;        // the gap is tense from a time to collision of 6 s, whole at 3 s (the classic)
static const float LM_TTC_SPAN_S = 3.0f;
static const float LM_JUMP_M = 10.0f;           // a gap that changes by this much in a tick is another vehicle (the classic's crossing filter takes 10)
static const float LM_RATE_TAU_S = 0.4f;        // the gap's rate (the classic's crossing filter)
static const float LM_CROSS_HOLD_S = LM_WIN_S;  // the pattern of a crossing car is trusted whole once it held this long: the same 0.35 s the other estimates of the model need (the classic's 0.4 s was a switch: nothing for 0.4 s, then everything; here the weight grows from the first tick of the pattern, so the first 0.35 s ask a fraction)
static const float LM_CROSS_MIN_V = 3.0f;       // m/s: only a car that moves (a standing queue is not crossing), and only when we move
static const float LM_CROSS_MIN_D = 2.5f;       // m/s: the rate of the gap differs from what the lead's speed says by this much or by half the lead's speed
enum { LM_RING = 32 };

struct lm_in {
    float dt;          // s
    float v;           // our speed, m/s (>= 0)
    float gap;         // m to the vehicle ahead from the radar; <= 0.3, NAN or >= 1e5 = none
    float lv;          // m/s the radar says that vehicle drives; NAN = unknown or not matching the gap
    int src;           // LMS_*: the source is fine / the hook is not in / the answer is stale
    // the vehicle the Cutin reader predicts to merge (n counts the predictions, 0 = none yet); LOG ONLY
    unsigned merger_n, merger_id; float merger_s, merger_d, merger_closing;
};
struct lm_out {
    int present;       // a vehicle is in the radar (gap known)
    int valid;         // present and its speed is known: a lead to follow
    int reason;        // LMR_* (LMR_OK when valid)
    float gap, lv, v;  // m, m/s, m/s
    float opening;     // lv - v: + = it pulls away
    float closing;     // v - lv: + = we close in
    float a_lead;      // m/s2, filtered, + = speeding up; 0 for a new lead
    float gap_rate;    // m/s, d(gap)/dt filtered (0.4 s)
    float ttc;         // s, gap / closing while closing > 0.3 m/s, else 99
    float age_s;       // s this lead has been followed
    float w_brake;     // 0..1 the lead brakes
    float w_tense;     // 0..1 the gap is tense: the lead brakes, or the time to collision is short
    int crossing;      // a car crossing in front: passed, not followed (cross_w whole)
    float cross_w;     // 0..1 how sure the pattern is: the seconds it held over LM_CROSS_HOLD_S; it falls at twice that rate when the pattern is gone
    int appeared;      // 1 on the tick a lead appeared or was replaced by another vehicle
    float jump_m;      // on that tick: how much nearer (+) the new vehicle is than the old gap (0 for a first lead)
    // the predicted merger (log only)
    int merger_new;    // 1 on the tick a NEW prediction came in
    unsigned merger_id; float merger_s, merger_d, merger_closing;
};
struct lm_state {
    double t;                                   // s since reset
    int valid_prev; float gap_prev, lv_prev;
    float rate; int rate_init;                  // filtered d(gap)/dt; rate_init: the first measured rate seeds it (as the classic crossing filter did), the filter only smooths after
    float a_f;                                  // the filtered acceleration
    float rt[LM_RING], rv[LM_RING]; int rn, rh; // the ring of (time, lead speed) for the slope
    double age; float cross_e;                  // cross_e: the seconds the crossing pattern held (0..LM_CROSS_HOLD_S)
    unsigned merger_seen;
};
static inline void lm_reset(lm_state *s) { memset(s, 0, sizeof *s); }
static inline float lm_ramp(float x, float a, float b) { return x <= a ? 0.0f : x >= b ? 1.0f : (x - a) / (b - a); }

// the least-squares slope of the lead's speed over the window
static inline float lm_slope(const lm_state *s)
{
    if (s->rn < 3) return 0;
    const float t_end = s->rt[(s->rh + LM_RING - 1) % LM_RING];
    float st = 0, sv = 0; int n = 0;
    for (int i = 0; i < s->rn; i++) {
        const int k = (s->rh + LM_RING - 1 - i) % LM_RING;
        if (t_end - s->rt[k] > LM_WIN_S) break;
        st += s->rt[k] - t_end; sv += s->rv[k]; n++;
    }
    if (n < 3) return 0;
    const float mt = st / n, mv = sv / n;
    float num = 0, den = 0;
    for (int i = 0; i < n; i++) {
        const int k = (s->rh + LM_RING - 1 - i) % LM_RING;
        const float dt_ = s->rt[k] - t_end - mt;
        num += dt_ * (s->rv[k] - mv); den += dt_ * dt_;
    }
    return den > 1e-9f ? num / den : 0.0f;
}

static inline void lm_step(lm_state *s, const lm_in &in, lm_out *o)
{
    memset(o, 0, sizeof *o);
    o->v = in.v; o->ttc = 99.0f; o->reason = LMR_NONE;
    const float dt = in.dt > 0 && in.dt < 0.5f ? in.dt : 0.0f;
    s->t += dt;
    const int gap_ok = in.gap == in.gap && in.gap > 0.3f && in.gap < 1e5f;
    o->present = gap_ok && in.src == LMS_OK;
    o->reason = in.src == LMS_NO_HOOK ? LMR_NO_HOOK : in.src == LMS_STALE ? LMR_STALE : !gap_ok ? LMR_NONE : !(in.lv == in.lv && in.lv >= 0 && in.lv < 70) ? LMR_NO_SPEED : LMR_OK;
    o->valid = o->reason == LMR_OK;
    // the predicted merger (log only): a new prediction is one more than the last seen
    if (in.merger_n != s->merger_seen) {
        o->merger_new = in.merger_n > s->merger_seen;
        s->merger_seen = in.merger_n;
        o->merger_id = in.merger_id; o->merger_s = in.merger_s; o->merger_d = in.merger_d; o->merger_closing = in.merger_closing;
    }
    if (!o->valid || dt <= 0) {
        // no lead: the history goes (a new lead must not look like a lead that brakes, audit 11-10)
        s->valid_prev = 0; s->rn = 0; s->rh = 0; s->a_f = 0; s->rate = 0; s->rate_init = 0; s->age = 0; s->cross_e = 0;
        if (o->present) { o->gap = in.gap; }
        return;
    }
    o->gap = in.gap; o->lv = in.lv;
    o->opening = in.lv - in.v; o->closing = in.v - in.lv;
    // a first lead, or another vehicle (the gap or the speed jumped): start over, and say so
    const int jumped = s->valid_prev && (fabsf(in.gap - s->gap_prev) >= LM_JUMP_M || fabsf(in.lv - s->lv_prev) >= 5.0f);
    if (!s->valid_prev || jumped) {
        o->appeared = 1; o->jump_m = s->valid_prev ? s->gap_prev - in.gap : 0.0f;
        s->rn = 0; s->rh = 0; s->a_f = 0; s->rate = 0; s->rate_init = 0; s->age = 0; s->cross_e = 0;
    } else {
        const float r = (in.gap - s->gap_prev) / dt;
        if (!s->rate_init) { s->rate = r; s->rate_init = 1; } else s->rate += (r - s->rate) * fminf(1.0f, dt / LM_RATE_TAU_S);
    }
    s->age += dt;
    // the slope of the lead's speed over the window, lopsided smoothing: a growing deceleration at once, a release slowly
    s->rt[s->rh] = (float)s->t; s->rv[s->rh] = in.lv; s->rh = (s->rh + 1) % LM_RING; if (s->rn < LM_RING) s->rn++;
    const float a_raw = lm_slope(s);
    const float tau = a_raw < s->a_f ? LM_TAU_BRAKE_S : LM_TAU_RELEASE_S;
    s->a_f += (a_raw - s->a_f) * fminf(1.0f, dt / tau);
    o->a_lead = s->a_f; o->gap_rate = s->rate;
    o->w_brake = lm_ramp(-s->a_f, LM_BRAKE_FLOOR, LM_BRAKE_FLOOR + LM_BRAKE_SPAN);
    float w = o->w_brake;
    if (o->closing > 0.3f) { o->ttc = in.gap / o->closing; w = fmaxf(w, lm_ramp(LM_TTC_FROM_S - o->ttc, 0.0f, LM_TTC_SPAN_S)); }
    o->w_tense = w;
    // crossing traffic (friend 11-10: a car crossing in front on a roundabout was followed down to a stop 3-4 times): the radar's speed is the car's own, the gap closes at ours
    if (!(o->appeared) && in.lv > LM_CROSS_MIN_V && in.v > LM_CROSS_MIN_V && o->opening - s->rate > fmaxf(LM_CROSS_MIN_D, 0.5f * in.lv)) {
        s->cross_e = fminf(LM_CROSS_HOLD_S, s->cross_e + dt);
    } else s->cross_e = fmaxf(0.0f, s->cross_e - 2.0f * dt);
    o->cross_w = lm_ramp(s->cross_e, 0.0f, LM_CROSS_HOLD_S);
    o->crossing = o->cross_w >= 1.0f;
    s->valid_prev = 1; s->gap_prev = in.gap; s->lv_prev = in.lv;
}
