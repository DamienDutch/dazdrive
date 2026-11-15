// The junction and roundabout requester of the speed arbiter (WP-SP2, docs/cruise-layers.md; review cards 20, 21, 23 and 24; the cancel of card 22 is the module's,
// it writes the game). PURE: no game call, header only. It only ASKS "the junction's speed + the margin, at the plan's distance before its entry"; the arbiter lifts the
// throttle, brakes late and firm with the junction's own plan deceleration, and approaches calmly (accel_to).
//
// What the module decides before it asks (it senses the pedal, the indicator and the route): whether this junction counts at all (a gentle turn only with the
// indicator, 24; not while the gas pedal says you go straight on, 23). A junction that does not count posts nothing.
#pragma once
#include "speedarb.h"
#include "req_bend.h"

struct rj_in {
    int valid;               // a junction that counts (route in view, not filtered away)
    float dist_m;            // to its entry
    float base_ms;           // the route's speed for it (turn radius x lateral, or the roundabout's entry speed) before the margin
    float margin_ms;         // we brake only down to the speed + this ([cruise.junctions] speed_margin_kmh)
    int roundabout;
    float grade_pct;         // the mean grade of the road to the entry
    float plan_decel;        // m/s2, [junctions] brake_deceleration: later and firmer than a bend (0 = the arbiter's comfort level)
    float reach_m;           // how far the route reads
    float v;                 // m/s of now
    int brake;               // 0 = [curves] brake off: only the throttle lifts
    float hush;              // 0..1: how far the driver's own signals (the gas pedal at a petrol station, no indicator for a gentle turn) have faded it out. The module slews it
                             // over a few seconds: the request comes and goes continuously, not in a frame (0 = fully there)
};
// The plan puts the speed there well before the entry (the old route_junction_speed: "Damien: roundabouts too fast"): 45 m before a roundabout, 25 m before a junction.
static inline float rj_plan_d(float dist_m, int roundabout) { const float d = dist_m - (roundabout ? 45.0f : 25.0f); return d > 0 ? d : 0; }
// No gas on the approach (the old gate: from 400 m or 30 s out, while the truck is faster than the junction's speed + 1 km/h, the throttle stays off while coasting
// still reaches it): the gain towards the junction falls to 0, as a weight that comes in over the first 40 m inside the gate (the old fade) and over the first km/h above
// the junction's speed. Outside it, or at or under that speed, the arbiter's calm approach stands. bind < 0 = the calm approach is off: no gate either.
static const float RJ_GATE_MIN_M = 400.0f, RJ_GATE_S = 30.0f, RJ_GATE_FADE_M = 40.0f, RJ_OVER_FROM_KMH = 1.0f, RJ_OVER_FULL_KMH = 2.0f;
static inline float rj_gate_w(float dist_m, float v_ms, float vi_ms)
{
    const float gate = fmaxf(RJ_GATE_MIN_M, RJ_GATE_S * v_ms);
    float wg = (gate - dist_m) / RJ_GATE_FADE_M; wg = wg <= 0 ? 0.0f : wg >= 1 ? 1.0f : wg * wg * (3.0f - 2.0f * wg);
    float wo = ((v_ms - vi_ms) * 3.6f - RJ_OVER_FROM_KMH) / (RJ_OVER_FULL_KMH - RJ_OVER_FROM_KMH); wo = wo <= 0 ? 0.0f : wo >= 1 ? 1.0f : wo * wo * (3.0f - 2.0f * wo);
    return wg * wo;
}
// Returns the figure the traces / descent code read (km/h): the junction's envelope now (NAN = none).
static inline float rj_post(sa_state *s, const sa_cfg &c, const rj_in &in)
{
    if (!in.valid) return NAN;
    const float w = rb_fade_w(in.dist_m, in.reach_m) * (1.0f - (in.hush < 0 ? 0.0f : in.hush > 1 ? 1.0f : in.hush));   // a junction at the end of the reader's reach fades in (no step at appearance); the driver's signals fade it in and out
    if (w <= 0.001f) return NAN;
    sa_req q; memset(&q, 0, sizeof q);
    q.owner = SAO_JUNCTION; q.cls = SAC_FLOW; q.why = in.roundabout ? "roundabout ahead" : "junction ahead"; q.flags = in.brake ? 0 : SAF_NOBRAKE;
    q.v_ms = in.base_ms + in.margin_ms;
    q.dist_m = rj_plan_d(in.dist_m, in.roundabout) / w;
    q.grade_pct = in.grade_pct;
    q.plan_decel = in.plan_decel;
    q.accel_to = c.bind_accel >= 0 ? c.bind_accel * (1.0f - rj_gate_w(in.dist_m, in.v, q.v_ms)) : -1.0f;
    sa_post_req(s, q);
    if (in.brake) {   // far too fast: the protection continues the ask above the firm level (as for a bend)
        sa_req p = q; p.cls = SAC_PROTECT; p.why = in.roundabout ? "roundabout ahead, far too fast" : "junction ahead, far too fast"; p.v_ms += RB_FAR_TOO_FAST_KMH / 3.6f; p.plan_decel = c.brake_firm; p.accel_to = -1.0f;
        sa_post_req(s, p);
    }
    sa_cfg plain = c; plain.bind_accel = -1.0f;
    return sa_allowed(plain, q, in.v, 0.0f) * 3.6f;
}
