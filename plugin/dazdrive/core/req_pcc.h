// The PCC requester of the speed arbiter (WP-SP2, docs/cruise-layers.md; review cards 25 and 9). PURE: no game call, header only. With a route and no vehicle close
// ahead, it asks for two things the arbiter cannot know:
//
//   momentum before a climb    a RAISE: run a few km/h over the set into a climb that is steeper than the road now (3 .. 15 s ahead), as much as the climb needs by the
//                              gearbox's own figure, Power half (the module hands over `up`, already shaped). The arbiter slews it and the absolute lines (the limit + 5) bound it.
//   lift before a crest        a request just under the set: ease off a few km/h before a top that a descent follows, and roll over it. It never brakes (the descent after
//                              the top is the engine brake's and the retarder's); it comes in as a slew (the amount rises at the raise rate), so the throttle does not stop at a stroke.
//   a dip                      is not asked here: the descent hold (req_legal.h) reads what the game's own overspeed braking grants, which the module sets for a dip.
//   the eco-roll's boost       the gearbox's wish for a little over the set just before it rolls in neutral (TrueShift publishes the amount, bounded by the cap): a RAISE.
//                              TrueShift decides when; this only gives the speed. Lowest wins: a crest's lift binds over a boost (the old code let the boost overrule the lift).
#pragma once
#include "speedarb.h"

struct rp_in {
    int on;                   // [cruise] predictive on and the route in view
    int lead_near;            // a vehicle within ~4 s ahead: no momentum (it would only be braked away)
    float up_kmh;             // the momentum a climb ahead would get now (band_up: the gearbox's need, the mode's share, the cap)
    float down_kmh;           // the ease-off before a crest (the mode's, the weight's, scaled with the set)
    float g_now, g_up;        // % : the grade of the road 50 m ahead, the steepest 50 m stretch 3 .. 15 s ahead (NAN = unknown)
    float after;              // % : the steepest descent 50 m stretch just after the crest (NAN = no crest within 12 s)
    float boost_kmh;          // the gearbox's wish for the eco-roll's boost (0 = none), already under the cap
};
struct rp_state { float lower; };   // the crest's ease-off in force (m/s), slewed
struct rp_out { float raise_kmh, lower_kmh, mom_up_kmh; };
static inline void rp_reset(rp_state *s) { s->lower = 0; }
// set_ms: the cruise's set speed. dt in s; rate_up / rate_dn in m/s per s (the arbiter's raise rates).
static inline rp_out rp_post(sa_state *s, rp_state *st, const rp_in &in, float set_ms, float dt, float rate_up, float rate_dn)
{
    rp_out o; memset(&o, 0, sizeof o);
    float lower_tgt = 0;
    if (in.on) {
        if (in.after == in.after && in.after < -2.0f) lower_tgt = in.down_kmh / 3.6f;                                                       // a descent right after the crest
        else if (in.g_up == in.g_up && in.g_now == in.g_now && in.g_up > 2.0f && in.g_up > in.g_now + 1.0f) { o.raise_kmh = in.up_kmh; o.mom_up_kmh = in.up_kmh; }   // a climb that is steeper than now
    }
    if (o.raise_kmh > 0.05f) sa_raise(s, o.raise_kmh / 3.6f, "PCC: momentum before the climb");
    if (in.boost_kmh > 0 && !in.lead_near && o.raise_kmh <= 0.05f) { sa_raise(s, in.boost_kmh / 3.6f, "eco-roll boost"); o.raise_kmh = fmaxf(o.raise_kmh, in.boost_kmh); }
    if (lower_tgt > st->lower) st->lower = fminf(lower_tgt, st->lower + rate_up * dt);
    else st->lower = fmaxf(lower_tgt, st->lower - rate_dn * dt);
    if (st->lower > 1e-4f) {
        sa_req q; memset(&q, 0, sizeof q);
        q.owner = SAO_PCC; q.cls = SAC_FLOW; q.why = "PCC: ease off before the crest"; q.accel_to = -1; q.flags = SAF_NOBRAKE;
        q.v_ms = set_ms - st->lower;
        sa_post_req(s, q);
        o.lower_kmh = st->lower * 3.6f;
    } else st->lower = 0;
    return o;
}
