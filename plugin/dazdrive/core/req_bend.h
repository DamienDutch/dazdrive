// The bend requesters of the speed arbiter (WP-SP2, docs/cruise-layers.md; review cards 17, 18 and 19). PURE: no game call, header only. They only ASK; when to lift
// off, how gently to brake and how calmly to approach are the arbiter's (core/speedarb.h: the coast envelope, the comfort deceleration, the calm approach).
//
//   bends ahead (17)    the route's bends as points "apex speed v at distance d" (route_bend_points), with the grade of the road to each. Two are posted: the one that
//                       lifts the throttle first (the lowest allowed speed) and the one that needs the most deceleration, and a PROTECT backstop on the second.
//   the bend you are in (18)  no route needed: the path's curvature from the yaw rate; a speed cap for the radius it shows, as a weight that grows with the curvature.
//   the calm approach (19)    is the arbiter's (accel_to / bind_accel); nothing here.
#pragma once
#include "speedarb.h"

struct rb_pt { float v_ms, d_m, grade_pct; };   // an apex speed, the distance to it, the mean grade of the road to it

// A point comes into view at the end of the route's reach: its request fades in over the last stretch (d divided by the weight: at weight 0 the point is infinitely far),
// so a bend that appears at the horizon does not step the target. The route's bend window is +-75 m (3 samples of 25 m), so the last 150 m of the route cannot show a
// bend completely.
static const float RB_FADE_M = 150.0f;
static inline float rb_fade_w(float d_m, float reach_m)
{
    if (!(reach_m > 0)) return 1.0f;
    const float w = (reach_m - d_m) / RB_FADE_M;
    return w >= 1.0f ? 1.0f : w <= 0.0f ? 0.0f : w * w * (3.0f - 2.0f * w);   // smoothstep: no corner at either end
}
// The backstop: a bend taken far too fast. The comfort plan brakes up to the firm level; beyond it (more than ~8 km/h over what the point allows) the protection takes
// over and may ask up to the maximum. 8 km/h = the old bend brake's step from 2.2 to 3.5 m/s2 ("kmh > bend_kmh + 8"). Its own plan deceleration is the firm level, so it
// brakes only once the firm plan is not enough and then continues the ask (continuous at that point).
static const float RB_FAR_TOO_FAST_KMH = 8.0f;

struct rb_in {
    const rb_pt *pt; int n;      // the route's points, nearest first, each slower than every nearer one
    float reach_m;               // how far the route reads (route_ahead_m), 0 = unknown
    float v;                     // the speed of now (m/s)
    int brake;                   // 0 = the bend may not brake (curve_brake off): it only lifts the throttle
};
// Returns the lowest envelope of the points now (km/h, NAN = none): the old "bend ahead" figure (route_bend_speed) that the descent code and the traces read.
static inline float rb_post_ahead(sa_state *s, const sa_cfg &c, const rb_in &in)
{
    sa_cfg plain = c; plain.bind_accel = -1.0f;      // the envelope without the calm approach: what lifting off alone allows
    sa_req qs[16];
    int bi = -1, bn = -1, nq = 0;
    float best_a = 1e9f, best_env = 1e9f, best_need = 0;
    const int n = in.n > 16 ? 16 : in.n;
    for (int i = 0; i < n; i++) {
        const float w = rb_fade_w(in.pt[i].d_m, in.reach_m);
        if (w <= 0.001f) continue;
        sa_req q; memset(&q, 0, sizeof q);
        q.owner = SAO_BEND; q.cls = SAC_FLOW; q.why = "bend ahead"; q.accel_to = -1; q.flags = in.brake ? 0 : SAF_NOBRAKE;
        q.v_ms = in.pt[i].v_ms; q.dist_m = in.pt[i].d_m / w; q.grade_pct = in.pt[i].grade_pct;
        qs[nq] = q;
        const float a = sa_allowed(c, q, in.v, 0.0f), env = sa_allowed(plain, q, in.v, 0.0f), need = sa_need(c, q, in.v, 2.5f);
        if (a < best_a) { best_a = a; bi = nq; }
        if (env < best_env) best_env = env;
        if (need > best_need) { best_need = need; bn = nq; }
        nq++;
    }
    if (bi < 0) return NAN;
    sa_post_req(s, qs[bi]);                           // the one that lifts the throttle first
    if (bn >= 0 && bn != bi) sa_post_req(s, qs[bn]);  // the one that needs the most brake
    if (in.brake && bn >= 0) {                        // far too fast for it: the protection continues the ask above the firm level
        sa_req q = qs[bn]; q.cls = SAC_PROTECT; q.why = "bend ahead, far too fast"; q.v_ms += RB_FAR_TOO_FAST_KMH / 3.6f; q.plan_decel = c.brake_firm;
        sa_post_req(s, q);
    }
    return best_env * 3.6f;
}

// ---- the bend you are in (card 18). State: the curvature filtered over 0.5 s and its held peak (1.5 s decay), the old numbers. The weight grows from the old off
// threshold (0.0028 1/m, radius 357 m) to the old on threshold (0.004, radius 250 m); it replaces the hysteresis (a switch) with a ramp: a motorway bend at 90 km/h
// stays under 0.0024 and gets weight 0.
struct rb_now_state { float f, pk; int init; };
static const float RB_NOW_K_OFF = 0.0028f, RB_NOW_K_ON = 0.004f;
static const float RB_NOW_FREE_MS = 20.0f;   // at weight 0 the cap is this far above the speed it would be at weight 1 (no constraint in practice): the request is continuous in the weight
static inline void rb_now_reset(rb_now_state *s) { s->init = 0; s->f = s->pk = 0; }
// Posts the cap for the radius the truck is on. yaw_rev_s: the placement's yaw rate in revolutions per second; speed_ms signed; lat_ms2 the sideways allowance of the bends
// (roll learning included); lat_scale(v) the allowance's shape over speed (route.cpp's, as a factor). The brake starts 3 km/h over the cap (the old bend brake's start).
// Returns the cap (m/s, NAN = none) for the traces.
static inline float rb_post_now(sa_state *s, rb_now_state *st, float dt, float yaw_rev_s, float speed_ms, float lat_ms2, float (*lat_scale)(float), int brake, float *radius_out)
{
    const float kmh = fabsf(speed_ms) * 3.6f;
    if (!(speed_ms > 0) || kmh < 20.0f) { st->init = 0; st->pk = 0; return NAN; }
    const float k_in = fabsf(yaw_rev_s) * 6.2831853f / fmaxf(speed_ms, 5.0f);
    if (!st->init) { st->f = st->pk = k_in; st->init = 1; }
    else { st->f += (k_in - st->f) * fminf(1.0f, dt / 0.5f); st->pk = fmaxf(st->f, st->pk * expf(-dt / 1.5f)); }
    float w = (st->pk - RB_NOW_K_OFF) / (RB_NOW_K_ON - RB_NOW_K_OFF);
    w = w <= 0 ? 0.0f : w >= 1 ? 1.0f : w * w * (3.0f - 2.0f * w);
    if (w <= 0.0f) return NAN;
    float vc = sqrtf(lat_ms2 / st->pk);
    for (int it = 0; it < 3; it++) vc = sqrtf(lat_ms2 * lat_scale(vc) / st->pk);
    const float cap = fmaxf(vc, 15.0f / 3.6f);
    if (radius_out) *radius_out = 1.0f / st->pk;
    sa_req q; memset(&q, 0, sizeof q);
    q.owner = SAO_BEND_NOW; q.cls = SAC_FLOW; q.why = "in a bend (speed for its radius)"; q.accel_to = -1; q.flags = brake ? 0 : SAF_NOBRAKE;
    q.v_ms = cap + (1.0f - w) * RB_NOW_FREE_MS;
    q.band_ms = 3.0f / 3.6f;
    sa_post_req(s, q);
    return cap;
}
