// The speed arbiter (WP-SP1, docs/cruise-layers.md). PURE: no game call, no global but its own state, header only (the sims include it as it is).
//
// A function that has an opinion about the speed posts a REQUEST: "be at v at distance d, because <why>" (d = 0 is "now"). The arbiter
// decides how hard or soft: the comfort values of the whole cruise side live in sa_cfg, in this one place, so a requester stays dumb and small.
//   1. Each request has an ALLOWED SPEED NOW: v for d = 0, else the speed from which lifting off (a_coast) reaches v at d. The lowest wins
//      (the one that needs action first); a tie goes to the lower class (PROTECT < LEGAL < FLOW < DRIVER). The winner and its reason are
//      reported so the caller can log them when they change (promises P7 and P9).
//   2. Up and hold: a = e / tau (tau = the approach time, so it arrives without overshoot), at most the mode's share of the pull the truck
//      has to spare. It becomes a FORCE at the wheels: m a + F_hold + a slow trim, rate limited per mode, frozen in a shift.
//   3. Down, coast first: the throttle is zero above the allowed speed. The brake channel is asked only when coasting is not enough: a
//      request that needs more than the truck gives by itself (+ a tolerance) AND more than the plan's comfort deceleration (a request at
//      a distance), or a d = 0 request more than `band` over (the descent allowance). ONE total deceleration goes to the channel, which
//      stages retarder, engine brake and service brake by cost. Never gas and brake together: a brake ask zeroes the force.
//   (WP-SP2) The road asks through the same shape. A request may carry the facts of its road (the mean grade to the point: gravity changes how far lifting off
//   carries), how it wants to be braked for (plan_decel: a junction brakes later and firmer than a bend), how much the truck may still gain towards it (accel_to:
//   the calm approach, no hard throttle toward something it has to slow down for) and flags (never brakes; is relative to the set; its brake is urgent). A
//   function that wants the truck ABOVE the set (momentum before a climb, the eco-roll's boost) posts a RAISE: the arbiter slews it (no step in the target) and adds
//   it to the requests that are relative to the set (the set itself, the descent hold), so the lines that are absolute (the limit line) still bind.
//   (WP-SP3) The vehicle ahead asks through the same shape (core/req_acc.h): its point MOVES (v_move_ms: the plan is made in its frame), it plans lifting off with what the truck really
//   gives (coast_decel), the calm approach is a weight that is off for a lead that drives faster than we do (nocalm), and a request may keep the slow trim from integrating while
//   it binds (SAF_NOTRIM: the gap to a vehicle is the integral). Its class FOLLOW stands between LEGAL and FLOW. With the new fields at 0 every request is what it was.
//   4. A CEILING (LIM) is not a request: sa_ceiling_cap() is a throttle cap, a continuous function of the speed error, and a kickdown
//      passes it by a blend (sa_kick_w), never a switch.
//   (WP-SP2b) Only the FORCE needs the vehicle model. Without it (in.f_full <= 0: the engine reads invalid, ride 04-11) the decision is still made, the winner and
//      its reason, the allowed speed, the brake ask (they need the mass and the road, not the engine), and out->nf says "no force was computed": the caller then lets
//      the game's own cruise hold the speed and shapes its throttle with sa_nf_weight() below.
#pragma once
#include <math.h>
#include <stdio.h>
#include <string.h>

enum { SAC_PROTECT, SAC_LEGAL, SAC_FOLLOW, SAC_FLOW, SAC_DRIVER, SAC_N };   // priority class: the lowest wins a tie; it also sets how firm a brake may be. FOLLOW (WP-SP3): the vehicle ahead, above the road's comfort (FLOW)
enum { SAO_SET, SAO_LEAD, SAO_BEND, SAO_BEND_NOW, SAO_JUNCTION, SAO_PCC, SAO_ROLL, SAO_LIMIT, SAO_DESCENT, SAO_N };   // who asks (the own ACC and ISA-ahead join later)
enum { SAK_NONE, SAK_UP, SAK_HOLD, SAK_COAST, SAK_BRAKE };      // what the arbiter does now
enum { SAF_NOBRAKE = 1,    // it limits what the truck chases and lifts the throttle, it never asks the brake channel (the set, a crest's lift, a no-gas hold)
       SAF_SET = 2,        // relative to the set: the raise is added to it (the set, the descent hold). An absolute line (the limit) is not
       SAF_URGENT = 4,     // when it brakes, the channel gets no tolerance for the retarder: a hard line
       SAF_NOTRIM = 8 };   // (WP-SP3) while it binds, the slow trim does not integrate: it settles the truck on a moving target (the gap to a vehicle), and a second integral in that loop made it hunt. The gap is the integral
// 20: set + the vehicle ahead (its time gap, its gap law, its braking, a protection for two of them, a crossing hold) + two bends + the bend in + a junction + its backstop + a
// crest's lift + the limit + the descent + room. A request over the end is counted in `dropped`, which the caller logs: a constraint never goes missing silently
#define SA_MAX_REQ 20

struct sa_req {
    int owner, cls;
    float v_ms;          // the speed asked (m/s)
    float dist_m;        // where (m ahead); 0 = now
    float band_ms;       // d = 0 only: how far over v the truck may run before the brake comes in (the descent allowance; 0 = none). Negative: a look-ahead, the brake counts the speed as that much higher
    const char *why;     // the reason, for the log (P9): never NULL
    // (WP-SP2) the facts and wishes of a request at a distance; the neutral values (what sa_post sets) behave as before
    float grade_pct;     // the mean grade of the road to the point (%, + uphill): gravity adds 0.0981 m/s2 per % to what lifting off gives
    float accel_to;      // the most the truck may gain speed towards it (m/s2): < 0 = the arbiter's bind_accel. 0 is a real value: no gain at all
    float plan_decel;    // the deceleration it is braked with (m/s2): > 0 replaces the comfort level as the point where it brakes (a junction: later and firmer than a bend)
    int flags;           // SAF_*
    // (WP-SP3) a point that MOVES: the vehicle ahead. v_ms is then the speed to have when the gap to it is dist_m, and the whole plan is made in its frame (the speed relative to it:
    // lifting off closes up to sqrt(2 a d) over its speed, the deceleration needed is (w^2 - w_at^2) / 2 d with w the speed relative to it). 0 = a fixed point, as every road request is
    float v_move_ms;
    // (WP-SP3) how much of the calm approach is off for this request, 0..1 (0 = as ever: toward a speed it has to slow down for, the truck gains at most accel_to). A vehicle ahead that
    // drives FASTER than we do is not something to slow down for: the truck may close up on it as the set allows. A weight, continuous in the speed difference, not a switch
    float nocalm;
    // (WP-SP3) what lifting off gives for THIS request, m/s2 (0 = the arbiter's coast_decel and the request's grade). The vehicle ahead is planned with what the truck really does when it lifts off
    // (the model's natural deceleration now, never above the central value): a plan that counts on 0.4 and gets 0.14 arrives at a standing vehicle with speed to spare
    float coast_decel;
    // (WP-SP3) the firmest this request may brake, m/s2 (0 = its class's: PROTECT brake_max, LEGAL / FLOW brake_firm). The vehicle ahead's protection may brake harder than the road's: a lead
    // that brakes hard is the one case where a truck needs more than the bend brake's 3.5 (the AEB, above it all, takes what is left)
    float brake_cap;
};

// Knobs, each with its derivation in docs/cruise-layers.md. Index 0 Eco, 1 Normal, 2 Power.
struct sa_cfg {
    float tau_s[3];           // approach time: the old cruise's approach_s 2.5 x its mode factor 1.2 / 1 / 0.6
    float spare_share[3];     // the share of the spare pull (F_full - F_hold) an approach may ask: the old approach_share_* 0.9 / 0.95 / 1.0
    float force_rise[3];      // the force may rise this much of the full wish per second: the old PI's rate 0.4 / 0.6 / 1.0 (it falls twice as fast)
    float trim_s;             // the trim's integral time: the old ki / kp = 0.33 per s
    float trim_max;           // the trim's limit, a share of the full wish: the old PI's integrator limit (-0.5 .. 1 of the throttle); the model's error (a grade it does not know) is its job
    float coast_decel;        // m/s2: lifting off is planned with this (the old curve_coast_decel)
    float brake_comfort;      // m/s2: the plan's comfort deceleration (curve_brake_decel); times brake_k per mode
    float brake_firm, brake_max;   // the firmest a LEGAL and a PROTECT request may brake (the levels the bend brake used)
    float brake_k[3];         // the mode factor on the deceleration: the own ACC's eco / power factor 0.75 / 1 / 1.25
    float brake_tol;          // m/s2: the brake asks only above the truck's own deceleration + this (between the channel's firm 0.1 and comfort 0.3)
    // WP-SP2
    float bind_accel;         // m/s2: the calm approach: towards a speed ahead it has to slow for, the truck gains at most this (the old ahead_accel 0.088); < 0 = off (the pure core's default)
    float raise_up, raise_dn; // m/s per s: how fast a raise moves the target. A 6 km/h raise (1.7 m/s) in about the old approach time (2.5 s) is 0.67, so 0.5 up; down twice as fast, like the force
};
static inline sa_cfg sa_cfg_default()
{
    sa_cfg c;
    const float tau[3] = { 3.0f, 2.5f, 1.5f }, sp[3] = { 0.9f, 0.95f, 1.0f }, rise[3] = { 0.4f, 0.6f, 1.0f }, bk[3] = { 0.75f, 1.0f, 1.25f };
    for (int i = 0; i < 3; i++) { c.tau_s[i] = tau[i]; c.spare_share[i] = sp[i]; c.force_rise[i] = rise[i]; c.brake_k[i] = bk[i]; }
    c.trim_s = 3.0f; c.trim_max = 0.5f;
    c.coast_decel = 0.4f; c.brake_comfort = 1.05f; c.brake_firm = 2.2f; c.brake_max = 3.5f; c.brake_tol = 0.15f;
    c.bind_accel = -1.0f; c.raise_up = 0.5f; c.raise_dn = 1.0f;
    return c;
}

// What the plant is this tick (the caller reads it off the vehicle model).
struct sa_in {
    float v;            // m/s, >= 0
    float dt;           // s
    int mode;           // 0 Eco, 1 Normal, 2 Power
    int shifting;       // the clutch is open: the force is held
    float mass;         // kg; <= 0 = unknown
    float f_hold;       // N the road needs to hold the speed now (rolling + air + the grade of now)
    float f_full;       // N the strongest in-window full wish at this speed (what a controller's full wish asks)
    float nat;          // m/s2 the truck slows by itself now (+ = slowing, - = it speeds up)
    float accel;        // m/s2 the measured acceleration (the telemetry's; filtered here)
};

struct sa_out {
    int ok;               // 0 = no request (or no mass): nothing is driven
    int nf;               // 1 = the model could not tell the force (in.f_full <= 0): the decision (winner, target, brake ask) is made, share and force_n are 0 and not a command
    int kind;             // SAK_*
    int winner;           // the request that binds the allowed speed (index in the request list; -1 none)
    int owner, cls;
    const char *why;
    float v_target_ms;    // the allowed speed now
    float dist_m;
    float a_cmd;          // m/s2 (+ = speeding up)
    float force_n;        // N to the wheels (>= 0); 0 while coasting and braking
    float share;          // force_n / f_full, 0..1
    float brake_decel;    // the TOTAL deceleration to ask the brake channel (m/s2); 0 = none
    int brake_cls;        // class of the request that asks it
    const char *brake_why;
    int brake_urgent;     // the request that asks it is a hard line (SAF_URGENT): the channel gives the retarder no tolerance
    int brake_owner;      // SAO_* of the request that asks it
    float raise_ms;       // the raise in force now (slewed), added to the requests relative to the set
    const char *raise_why;
};

struct sa_state {
    sa_req r[SA_MAX_REQ];
    int n;
    float share_prev;     // the force share of the last tick (the rate limit's anchor)
    float trim_n;         // the trim, N
    int braking;          // the brake ask is on (hysteresis)
    float acc_f;          // the measured acceleration, filtered over 0.5 s (m/s2)
    int acc_init;
    float ex;             // 0..1: how much of the rest of the pull is given because the truck does not gain speed though it is under the target (a climb)
    float raise_tgt;      // the biggest raise posted this tick (m/s) and who posted it
    const char *raise_why;
    float raise_cur;      // the raise in force: it follows the target at cfg.raise_up / raise_dn (m/s)
    int dropped;          // requests posted over SA_MAX_REQ (never silent: the caller logs it)
};
static inline void sa_begin(sa_state *s) { s->n = 0; s->raise_tgt = 0; s->raise_why = NULL; }
// A request with every attribute (WP-SP2); sa_post below is the short form with the neutral ones.
static inline void sa_post_req(sa_state *s, const sa_req &r)
{
    if (!(r.v_ms == r.v_ms)) return;
    if (s->n >= SA_MAX_REQ) { s->dropped++; return; }
    sa_req &q = s->r[s->n++];
    q = r;
    if (q.v_ms < 0) q.v_ms = 0;
    if (!(q.dist_m > 0)) q.dist_m = 0;
    if (!(q.band_ms == q.band_ms)) q.band_ms = 0;   // (a negative band is a look-ahead: the brake counts the speed as that much higher, the chase is not touched)
    if (!q.why) q.why = "?";
    if (!(q.grade_pct == q.grade_pct)) q.grade_pct = 0;
    if (!(q.accel_to == q.accel_to)) q.accel_to = -1.0f;
    if (!(q.plan_decel > 0)) q.plan_decel = 0;
    if (!(q.v_move_ms > 0)) q.v_move_ms = 0;
    if (!(q.nocalm > 0)) q.nocalm = 0; else if (q.nocalm > 1) q.nocalm = 1;
    if (!(q.coast_decel > 0)) q.coast_decel = 0;
    if (!(q.brake_cap > 0)) q.brake_cap = 0;
}
static inline void sa_post(sa_state *s, int owner, int cls, float v_ms, float dist_m, float band_ms, const char *why)
{
    sa_req q; memset(&q, 0, sizeof q);
    q.owner = owner; q.cls = cls; q.v_ms = v_ms; q.dist_m = dist_m; q.band_ms = band_ms; q.why = why; q.accel_to = -1.0f;
    sa_post_req(s, q);
}
// A wish to run ABOVE the set (m/s over it): the biggest of the tick wins, and the arbiter moves its raise towards it at a bounded rate (no step in the target).
static inline void sa_raise(sa_state *s, float amount_ms, const char *why)
{
    if (amount_ms > s->raise_tgt) { s->raise_tgt = amount_ms; s->raise_why = why; }
}
// Bumpless start (the cruise comes on): the force share the throttle had, no trim.
static inline void sa_reset(sa_state *s, float share0) { s->share_prev = share0 < 0 ? 0 : share0; s->trim_n = 0; s->braking = 0; s->n = 0; s->acc_init = 0; s->acc_f = 0; s->ex = 0; s->raise_tgt = s->raise_cur = 0; s->raise_why = NULL; }

// The deceleration lifting off gives over the way to a request: the arbiter's coast_decel and gravity (0.0981 m/s2 per % of grade, as the old coast_over), at least
// 0.1 and at most 1.0 (the old bounds) but never outside what the configured coast itself is. Grade 0 = coast_decel exactly.
static inline float sa_a_coast(const sa_cfg &c, const sa_req &q)
{
    if (q.coast_decel > 0) return q.coast_decel;   // (WP-SP3) the request names what lifting off gives for it now (the truck's own deceleration, the grade of now in it)
    if (q.grade_pct == 0) return c.coast_decel;
    return fminf(fmaxf(1.0f, c.coast_decel), fmaxf(fminf(0.1f, c.coast_decel), c.coast_decel + 0.0981f * q.grade_pct));
}
// The allowed speed now of a request: v for d = 0, else the speed from which lifting off reaches v at d (the envelope). The calm approach: while the envelope is above
// the speed of now the truck may gain towards it only at accel_to, so it chases the point where its gain meets the coast-down curve (the old ahead_cap): faster would
// be a burst followed by a cut. raise: added to a request that is relative to the set.
static inline float sa_allowed(const sa_cfg &c, const sa_req &q, float v, float raise)
{
    const float vi = q.v_ms + ((q.flags & SAF_SET) ? raise : 0.0f);
    if (!(q.dist_m > 0)) return vi;
    const float ac = sa_a_coast(c, q), ag = q.accel_to >= 0 ? q.accel_to : c.bind_accel;
    if (q.v_move_ms > 0) {   // (WP-SP3) a point that moves: the same law in its frame. Lifting off from `speed over it` w reaches wi over it at d when w^2 = wi^2 + 2 a d
        const float vm = q.v_move_ms, wi = vi - vm, w = v - vm, env = sqrtf(wi * wi + 2 * ac * q.dist_m);
        if (ag >= 0 && env > w) { const float calm = vm + sqrtf((ac * w * w + ag * env * env) / (ag + ac)); return calm + q.nocalm * (vm + env - calm); }
        return vm + env;
    }
    const float env = sqrtf(vi * vi + 2 * ac * q.dist_m);
    if (ag >= 0 && env > v) { const float calm = sqrtf((ac * v * v + ag * env * env) / (ag + ac)); return q.nocalm > 0 ? calm + q.nocalm * (env - calm) : calm; }
    return env;
}

// The deceleration a request needs now (m/s2), 0 = none. A request at a distance: the kinematics to reach v at d. d = 0: the part of the error over the band, in the approach time.
static inline float sa_need(const sa_cfg &c, const sa_req &q, float v, float tau)
{
    if (q.dist_m > 1.0f) {
        if (q.v_move_ms > 0) { const float w = v - q.v_move_ms, wi = q.v_ms - q.v_move_ms; return w > wi ? fmaxf(0.0f, (w * w - wi * wi) / (2 * q.dist_m)) : 0.0f; }   // (WP-SP3) in the frame of a moving point
        return v > q.v_ms ? (v * v - q.v_ms * q.v_ms) / (2 * q.dist_m) : 0.0f;
    }
    float e = v - q.v_ms - q.band_ms;
    (void)c;
    return e > 0 ? e / tau : 0.0f;
}

static inline void sa_decide(const sa_cfg &c, sa_state *s, const sa_in &in, sa_out *o)
{
    memset(o, 0, sizeof *o);
    o->winner = -1; o->why = ""; o->brake_why = "";
    const int md = in.mode < 0 ? 1 : in.mode > 2 ? 2 : in.mode;
    if (s->n <= 0 || !(in.mass > 0) || !(in.dt > 0)) { s->braking = 0; return; }
    const int nf = !(in.f_full > 0);   // WP-SP2b: the force needs the engine model, the decision does not
    o->ok = 1; o->nf = nf;
    const float v = in.v, tau = c.tau_s[md];
    // 0. the raise follows its target at a bounded rate (no step in the target when a climb comes into view or a crest passes)
    if (s->raise_tgt > s->raise_cur) s->raise_cur = fminf(s->raise_tgt, s->raise_cur + c.raise_up * in.dt);
    else s->raise_cur = fmaxf(s->raise_tgt, s->raise_cur - c.raise_dn * in.dt);
    if (s->raise_cur < 1e-4f) s->raise_cur = 0;
    o->raise_ms = s->raise_cur; o->raise_why = s->raise_cur > 0 ? (s->raise_why ? s->raise_why : "?") : "";
    // 1. the lowest allowed speed wins; a tie to the lower class
    float best = 1e9f; int w = -1;
    for (int i = 0; i < s->n; i++) {
        float a = sa_allowed(c, s->r[i], v, s->raise_cur);
        if (a < best - 1e-4f || (fabsf(a - best) <= 1e-4f && w >= 0 && s->r[i].cls < s->r[w].cls)) { best = a; w = i; }
    }
    const sa_req &W = s->r[w];
    o->winner = w; o->owner = W.owner; o->cls = W.cls; o->why = W.why; o->v_target_ms = best; o->dist_m = W.dist_m;
    // 2. the brake side: the request that needs the most; only above what coasting gives and above the plan's comfort deceleration
    const float a_comf = c.brake_comfort * c.brake_k[md];
    float need = 0, cap = a_comf; int bw = -1;
    for (int i = 0; i < s->n; i++) {
        sa_req q = s->r[i];
        if (q.flags & SAF_NOBRAKE) continue;   // it limits the chase and lifts the throttle; the brake is not its business
        if (q.flags & SAF_SET) q.v_ms += s->raise_cur;
        const float raw = sa_need(c, q, v, tau);
        float nd = raw;
        float qcap = q.brake_cap > 0 ? q.brake_cap : q.cls <= SAC_PROTECT ? c.brake_max : q.cls <= SAC_FLOW ? c.brake_firm : a_comf;   // the comfort level decides WHEN a request at a distance brakes; once it must, it asks what its geometry needs, up to its class
        if (nd > qcap) nd = qcap;
        // a request at a distance brakes only when the gentle plan (lifting off) is not enough: it needs more than the comfort deceleration
        // (coast first, then brake late and firm: the owner's rule 30-10); a d = 0 request has its band instead (nd is 0 inside it). A request may
        // name the deceleration it is braked with (plan_decel: a junction later and firmer than a bend); a protection that names none brakes at once
        if (q.dist_m > 1.0f) {
            const float gate = q.plan_decel > 0 ? q.plan_decel : q.cls > SAC_PROTECT ? a_comf : 0.0f;
            if (raw < gate) nd = 0;
        }
        if (nd > need) { need = nd; bw = i; cap = qcap; }
    }
    const float thr_lo = in.nat + c.brake_tol;
    // hysteresis: on above the truck's own deceleration + the tolerance, off below the own deceleration + half of it (no pumping of the ask)
    if (need > thr_lo || (s->braking && need > in.nat + 0.5f * c.brake_tol)) s->braking = 1; else s->braking = 0;
    if (s->braking && bw >= 0) { o->brake_decel = need > cap ? cap : need; o->brake_cls = s->r[bw].cls; o->brake_why = s->r[bw].why; o->brake_urgent = (s->r[bw].flags & SAF_URGENT) != 0; o->brake_owner = s->r[bw].owner; }
    // 3. up, hold and coast: a force
    const float e = best - v;
    if (nf) {   // (WP-SP2b) no engine model, no force: what the arbiter does is named by the speed alone; the caller shapes the game's own throttle (sa_nf_weight)
        o->kind = o->brake_decel > 0 ? SAK_BRAKE : e < -0.14f ? SAK_COAST : e > 0.3f ? SAK_UP : SAK_HOLD;   // (a LABEL for the log line only, nothing reads it: 0.5 km/h over the allowed speed is coasting, 1 km/h under it speeding up, as the force path's e > 0.3 m/s)
        o->a_cmd = e / tau;
        return;
    }
    // The share of the spare pull an approach may ask: the mode's, plus the rest of it once the truck does not gain speed although it is more than 1 km/h under
    // the target (a climb, a wrong mass: rises 0.5 per second, falls the same; the old cruise's approach_ceiling), and all of it when little is spare (a climb that
    // takes nearly all the pull is held at whatever it takes, as a driver would).
    if (!s->acc_init) { s->acc_f = in.accel; s->acc_init = 1; }
    else s->acc_f += (in.accel - s->acc_f) * fminf(1.0f, in.dt / 0.5f);
    const float spare = fmaxf(0.0f, in.f_full - in.f_hold);
    if (!in.shifting) {
        const int slow = s->acc_f < 0.05f && e > 1.0f / 3.6f;
        s->ex = slow ? fminf(1.0f, s->ex + 0.5f * in.dt) : fmaxf(0.0f, s->ex - 0.5f * in.dt);
    }
    float share_up = c.spare_share[md] + (1.0f - c.spare_share[md]) * s->ex;
    if (spare < 0.25f * in.f_full) share_up = 1.0f;
    float a_up_max = in.mass > 0 ? share_up * spare / in.mass : 0.0f;
    float a = e / tau;
    if (a > a_up_max) a = a_up_max;
    o->a_cmd = a;
    float f = in.mass * a + in.f_hold + s->trim_n;
    const float f_cap = fminf(in.f_full, in.f_hold + share_up * spare + c.trim_max * in.f_full);
    if (f > f_cap) f = f_cap;
    if (f < 0) f = 0;
    if (o->brake_decel > 0) f = 0;   // never gas and brake together
    // the rate limit in the share of the full wish (the old PI's rate), held in a shift
    float share = f / in.f_full;
    const float rise = c.force_rise[md] * in.dt;
    if (in.shifting) share = s->share_prev;
    else if (o->brake_decel > 0) { /* braking: the force goes at once */ }
    else if (share > s->share_prev + rise) share = s->share_prev + rise;
    else if (share < s->share_prev - 2 * rise) share = s->share_prev - 2 * rise;
    if (share < 0) share = 0;
    if (o->brake_decel > 0) share = 0;
    // the trim: a slow integral of the speed error (the model alone leaves a steady offset), only while the force can follow
    const int up_sat = (f >= f_cap - 1.0f && e > 0) || (share < f / in.f_full - 1e-4f && e > 0);
    const int down_sat = (f <= 0.0f && e < 0) || (share > f / in.f_full + 1e-4f && e < 0);
    // The trim is for the model's steady offset (a grade it does not know, a mass that is off), not for the approach, which it would overshoot: it integrates inside
    // ~1 km/h of the target, or at any error up to 3 m/s while the speed does not change (a climb that holds the truck under the target)
    const int steady = fabsf(e) < 0.3f || (fabsf(s->acc_f) < 0.1f && fabsf(e) < 3.0f);
    if (!in.shifting && !up_sat && !down_sat && o->brake_decel <= 0 && steady && !(W.flags & SAF_NOTRIM)) {
        s->trim_n += (in.mass / (tau * c.trim_s)) * e * in.dt;
        const float lim = c.trim_max * in.f_full;
        if (s->trim_n > lim) s->trim_n = lim;
        if (s->trim_n < -lim) s->trim_n = -lim;
    }
    s->share_prev = share;
    o->share = share;
    o->force_n = share * in.f_full;
    o->kind = o->brake_decel > 0 ? SAK_BRAKE : share <= 1e-4f ? SAK_COAST : e > 0.3f ? SAK_UP : SAK_HOLD;
}

// ---- the log line (P9): one short reason for what the arbiter does, "<what> to <speed> km/h [in d m], because <why>"
static inline const char *sa_kind_name(int k) { return k == SAK_UP ? "speeding up" : k == SAK_HOLD ? "holding" : k == SAK_COAST ? "coasting" : k == SAK_BRAKE ? "braking" : "idle"; }
static inline void sa_describe(const sa_out &o, char *b, size_t n)
{
    if (!o.ok) { snprintf(b, n, "no request"); return; }
    if (o.dist_m > 0) snprintf(b, n, "%s towards %.0f km/h in %.0f m, because %s", sa_kind_name(o.kind), o.v_target_ms * 3.6f, o.dist_m, o.why);
    else snprintf(b, n, "%s at %.0f km/h, because %s", sa_kind_name(o.kind), o.v_target_ms * 3.6f, o.why);
    if (o.raise_ms > 0.03f && o.dist_m <= 0) { size_t k = strlen(b); snprintf(b + k, n - k, " (set raised by %.1f km/h: %s)", o.raise_ms * 3.6f, o.raise_why); }
}
// The same line without its numbers that move (the distance, the speed): what changes when the reason changes. The caller logs a decision when this key changes.
static inline void sa_describe_key(const sa_out &o, char *b, size_t n)
{
    if (!o.ok) { snprintf(b, n, "no request"); return; }
    snprintf(b, n, "%s|%d|%s|%d|%d", sa_kind_name(o.kind), o.owner, o.why, o.dist_m > 0, o.raise_ms > 0.03f);
}

// ---- WP-SP2b: the throttle without a force model. The arbiter's force is a force: m a + F_hold needs the engine and the road's resistance. A truck whose model cannot tell
// it (the engine reads invalid) keeps the game's own cruise for the HOLD, and the arbiter only takes the throttle AWAY where its allowed speed asks for it: a weight on the game's
// throttle, w in 0..1, continuous in everything it reads.
//   we  how far under the allowed speed the truck is, in x: 0 at or over it (lifted off, coasting), whole from x under it (the game's throttle is whole). Smoothstep: no corner.
//   kd  how much of that allowed speed is the road's and not the set's: 0 when the set itself binds (a plain hold is the game's own, untouched: w = 1), whole when what binds is
//       x under the set. So a bend that comes into view lifts the throttle by a weight that grows with how far below the set it already pulls, and the set alone never does.
//   w = 1 - kd (1 - we)
// x is derived, not chosen: the speed the truck loses by coasting during one approach time (tau x the coast deceleration, 1.0 m/s = 3.6 km/h in Normal): the arbiter's own two
// time constants. At the allowed speed the truck is within x of being at it even with the throttle cut for a whole approach, so the throttle is fully off exactly there.
static inline float sa_smooth01(float x) { x = x < 0 ? 0.0f : x > 1 ? 1.0f : x; return x * x * (3.0f - 2.0f * x); }
static inline float sa_nf_x(const sa_cfg &c, int md) { const int m = md < 0 ? 1 : md > 2 ? 2 : md; const float x = c.tau_s[m] * c.coast_decel; return x > 0.05f ? x : 0.05f; }
static inline float sa_nf_weight(float v, float best_ms, float set_allowed_ms, float x_ms)
{
    const float x = x_ms > 0.05f ? x_ms : 0.05f;
    const float we = sa_smooth01((best_ms - v) / x), kd = sa_smooth01((set_allowed_ms - best_ms) / x);
    return 1.0f - kd * (1.0f - we);
}

// ---- the LIM ceiling (not a request: the arbiter does not drive towards it). The throttle cap, as a share of the gear's full force, that
// closes the gap to the limit in tau seconds against the road's resistance (the own limiter's law, core/... modules/cruise.cpp own_limit_cap):
// a strong engine runs full until a few km/h before the limit, over the limit the cap falls under what holding takes. Continuous in v.
static inline float sa_ceiling_cap(float v, float lim_ms, float tau, float mass, float f_hold, float f_gear_full)
{
    if (!(f_gear_full > 0) || !(mass > 0) || !(tau > 0)) return NAN;
    float f = mass * (lim_ms - v) / tau + f_hold;
    float c = f / f_gear_full;
    return c < 0 ? 0.0f : c > 1 ? 1.0f : c;
}
// The kickdown pass: a weight, never a switch. smoothstep over the pedal from `from` to `full` (zero slope at both ends, so 0.69 and 0.71 behave the same).
static inline float sa_kick_w(float pedal, float from, float full)
{
    if (!(full > from)) return pedal >= full ? 1.0f : 0.0f;
    float x = (pedal - from) / (full - from);
    x = x < 0 ? 0.0f : x > 1 ? 1.0f : x;
    return x * x * (3 - 2 * x);
}
static inline float sa_cap_pass(float cap, float w) { return cap + w * (1.0f - cap); }
