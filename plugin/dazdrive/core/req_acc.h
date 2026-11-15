// The ACC requester of the speed arbiter (WP-SP3, docs/cruise-layers.md; review cards 28 (time gaps), 29 (the distance key), 30 (the US rule), 33 (cut-in), 34 (crossing); owner 46.2,
// 54.1). PURE: no game call, header only. The vehicle ahead is not a thing the ACC steers by: it is a REQUESTER of the speed arbiter like the road's. It asks
//
//   the moving target     "the lead's speed, when the gap to it is the wanted gap": a request at the room left before the wanted gap, at a point that MOVES with the lead (core/speedarb.h
//                         v_move_ms). The arbiter lifts off for it when the truck would otherwise close in, brakes late and only as hard as the geometry needs, and a lead that stands still
//                         (or brakes to a stop) is the same request with its speed 0: that is the stop at the stop distance (3 / 4 / 5 m per ACC level), not a second formula.
//   the gap law           "lead speed + gain x (gap - wanted gap)" now: what settles the truck on the wanted gap once it is there (the proportional term of the classic ACC; without it the
//                         moving target alone sits exactly on the lead's speed and pumps the brake: SP1's jam sim). Coasts, never brakes (the target above brakes when it must).
//   the braking lead      "match the lead as it will be after it has gone on braking as it does now" (its speed then, the way it covers): for one more approach time of ours if it brakes
//                         gently, until it stands if it brakes hard; with its distance divided by the braking weight: at weight 0 it is infinitely far. A lead that begins to brake raises the
//                         urgency continuously (no step).
//   a protection          the moving target and the braking lead once more, in class PROTECT, 8 km/h of relative speed higher: the arbiter's firm plan (2.2 m/s2) continues up to RA_PROTECT_MAX
//                         when the geometry needs more (as for a bend taken far too fast); AEB (SP4) is above all of it.
// A function never brakes or throttles itself; how hard, how early and how calmly is the arbiter's (coast first, the brake channel's costs, the mode's style). Off (ACC distance off, no lead,
// the lead's speed unknown) it posts nothing: P8. Every request has a reason that names the lead: P9.
//
// Lessons carried from the classic own ACC (modules/cruise.cpp own_acc), each with its date there: the wanted gap is the time gap x the LEAD's speed (30-10), sliding to ours as it gets tense
// (no tipping point); the wanted gap grows by at most 1.5 m/s and shrinks by at most 3 m/s (25-10: a change of the distance step is no reason to brake at once); the standstill gap grows into
// the time gap over 30 km/h (12-10); at a crawl the plan matches the lead's speed earlier (31-10); uphill the gap may grow by a band before the truck pulls up for it (30-10).
#pragma once
#include "speedarb.h"
#include "leadmodel.h"
#include "req_bend.h"
#include "req_overtake.h"

// ---- the owner's gaps (54.1): three time gaps 0.9 / 1.15 / 1.4 s in his ini, and NO motorway shrink for any of them. The classic shrinks the middle and the long gap to 0.8x between 75 and 95 km/h:
// with the owner's values that is 0.92 s for the middle and 1.12 s for the long one, so short and middle are one gap on the motorway, and shrinking only the middle would put the long gap
// under it (1.12 < 1.15). Here the three stay what they say, at every speed: short < middle < long always (a row).
static inline int ra_level_clamp(int level) { return level < 0 ? 1 : level > 2 ? 2 : level; }
// The US rule of thumb (card 30, as today): one second per 10 feet of vehicle length below 40 mph, one second more above (it comes in between 36 and 44 mph, not as a step), short x0.8, long x1.2
static inline float ra_us_gap_s(float ft, float v_ms, int level)
{
    const float more = fminf(1.0f, fmaxf(0.0f, (v_ms * 2.23694f - 36.0f) / 8.0f));
    return (ft / 10.0f + more) * (level == 0 ? 0.8f : level == 2 ? 1.2f : 1.0f);
}
static inline float ra_gap_s(const float t[3], int us, float us_ft, int trailer, int level, float v_ms)
{
    if (us) return ra_us_gap_s(us_ft > 0 ? us_ft : (trailer ? 70.0f : 25.0f), v_ms, level);
    return t[ra_level_clamp(level)];
}
// The standstill gap (owner 54.1 / planner-keuzes 31-10): 3 m at the closest ACC level, 4 at the middle, 5 at the far one; the vehicle class may name its own (a car 2.5 m); the ini may name one
static inline float ra_stop_base(int level, float ini_m, float class_m)
{
    if (ini_m > 0) return ini_m;
    if (class_m == class_m) return class_m;
    return 3.0f + (float)(level < 0 ? 0 : level > 2 ? 2 : level);
}
static const float RA_FADE_V = 8.33f;   // m/s (30 km/h): the standstill gap grows into the time gap over this speed (Damien 12-10: in a queue it may close up, not onto the bumper)
// The gap at speed v: a standstill gap that grows by `grow` over RA_FADE_V, and the time gap (the short level: the larger of the two; the others: the time gap on top, fading in)
static inline float ra_stop_gap(float base, float grow, float v) { return base + grow * fminf(1.0f, v / RA_FADE_V); }
static inline float ra_wanted(int level, float t_s, float v_ref, float base, float grow, float max_m)
{
    const float mx = fmaxf(max_m, base + grow), fade = fminf(1.0f, v_ref / RA_FADE_V), sg = ra_stop_gap(base, grow, v_ref);
    const float m = level == 0 ? fmaxf(sg, t_s * v_ref) : sg + t_s * v_ref * fade;
    return m > mx ? mx : m;
}

// ---- the constants of the requester, each with where it comes from
static const float RA_SLEW_UP = 1.5f, RA_SLEW_DN = 3.0f;   // m/s: how fast the wanted gap may grow / shrink (25-10)
static const float RA_REACH_M = 100.0f;                    // the game's radar sees a vehicle only within 100 m (the ACC's `gap < 100`, cruise-re): a vehicle comes into view at this distance
static const float RA_EDGE_M = 10.0f;                      // ... so its requests fade in over the last 10 m of it (distance / weight): no step in the allowed speed when a vehicle enters the radar
static const float RA_D_FLOOR_M = 1.05f;                   // the arbiter plans a request nearer than 1 m as "be at v now" (a different law, e/tau): a gap that hovers around the wanted one would flip between the two laws, and the ask with it. The posted distance stays above it
// (WP-SP3 round 2) The classic own ACC (modules/cruise.cpp own_acc) asks for 1 / ACC_GAIN_SHARE (1 / 0.7 = 1.43) times the deceleration the kinematics need: "the brake's lag and the lead's braking
// still to come" (cruise.cpp, ACC_GAIN_SHARE). The layers plan with the same share: the room they plan the approach in is that share of what is left, which is the deceleration of the plan x 1.43.
static const float RA_GAIN_SHARE = 0.7f;
// The firmest the protection brakes for a vehicle ahead, and the same 0.7 again: the ACC's authority is that share of what the AEB may ask for ([cruise] aeb_max_decel, 6.5 m/s2: the vehicle class
// may name its own, a car 9); the AEB keeps the rest. Not a scanned number: the module passes the AEB's own maximum (ra_in.aeb_max), RA_AEB_MAX_DEC is what the pure rows use. 0.7 x 6.5 = 4.55 m/s2
// (the arbiter's bend maximum 3.5 left a lead that brakes at 6 m/s2 to the AEB: 14 s of it in the hard-brake rows against none).
static const float RA_AEB_MAX_DEC = 6.5f;
static inline float ra_protect_max(float aeb_max) { return RA_GAIN_SHARE * (aeb_max > 0 ? aeb_max : RA_AEB_MAX_DEC); }
static const float RA_BRAKE_SOFT = 2.5f, RA_BRAKE_HARD = 3.5f;   // m/s2: a lead braking up to 2.5 is slowing for traffic, from 3.5 it is stopping; 3.5 is what ISO 15622 lets an ACC itself brake at most
static const float RA_COAST_MIN = 0.1f;                    // m/s2: the least lifting off is planned to give (the arbiter's own floor for a request's coast)
static const float RA_CRAWL_KMH = 30.0f;                   // below this the plan matches the lead's speed earlier at short gaps (31-10: stayed 2-6 km/h over the lead down to 5 m)
// the calm cut-in (card 33), the classic's weights: a lead that opens the gap (cut in faster than we drive, or pulls away) is no reason to slow down, only to stop speeding up. The opening
// speed (lead minus ours) counts from 0.3 m/s (the radar speed noise floor, ts4t) to whole at 0.6 when the gap is about right (from 0.5 to 0.6 of the wanted: ts4n followed at 8.8 m with 27 m
// wanted and "pulls away 1 km/h faster" held the speed, so it never opened the gap); a lead that is clearly faster (1.0 to 1.3 m/s, 3.5 to 4.7 km/h) opens the gap by itself even on the bumper
static const float RA_OPEN_FROM = 0.3f, RA_OPEN_FULL = 0.6f, RA_PULL_FROM = 1.0f, RA_PULL_FULL = 1.3f, RA_GAP_FROM = 0.5f, RA_GAP_FULL = 0.6f;
static const float RA_CALM_MIN_M = 5.0f, RA_CALM_FULL_M = 6.0f;   // nearer than 5 m nothing is calm (the classic's floor: the game's own emergency braking works there)
static const float RA_OT_S = 1.0f;                         // the overtake assist's pull towards the set comes in and goes out over this long (the arbiter's own force takes ~1.5 s for a whole throttle; a pull that came in a tick would be a kick)
static inline float ra_smooth(float x) { x = x < 0 ? 0.0f : x > 1 ? 1.0f : x; return x * x * (3.0f - 2.0f * x); }

struct ra_in {
    int on;               // the ACC is on: the cruise is on, the ACC distance is not off, the layers drive
    const lm_out *lead;   // the lead model of this tick
    float dt;             // s
    int mode;             // 0 Eco, 1 Normal, 2 Power (the arbiter's approach time)
    float v;              // our speed, m/s
    float set;            // the cruise's set speed, m/s
    int level;            // 0 short, 1 middle, 2 long
    float t_gap_s;        // the time gap of this level at this speed, after the US rule and the vehicle class (ra_gap_s)
    float stop_base_m, stop_grow_m;   // the standstill gap and how much it grows by 30 km/h
    float max_m;          // the longest gap (acc_max_m, 95 under the US rule)
    float gain;           // own_acc_gain: m/s of speed offset per metre of gap error
    float plan_to;        // own_acc_plan_to: the approach is planned to end at this share of the wanted gap
    float grade_pct;      // the road under us (+ uphill): gravity changes how far lifting off carries
    float a_dec;          // m/s2: the deceleration the truck delivers now (the brake channel's measured one, + = slowing); 0 = unknown
    float nat;            // m/s2: how fast the truck slows by itself now (the model's natural deceleration; + = slowing)
    float band;           // own_acc_band: uphill the gap may grow by this share before the truck pulls up for it
    int calm;             // acc_calm_cut_in: a lead that cuts in faster than we drive or pulls away makes the ACC hold the speed, not slow down
    int cross;            // acc_ignore_crossing: a car crossing in front is passed, not followed
    float aeb_max;        // m/s2: the AEB's maximum deceleration (the vehicle class's, or [cruise] aeb_max_decel); 0 = RA_AEB_MAX_DEC
    int overtake;         // the overtake assist is on (core/req_overtake.h): a shorter time gap and a pull towards the set, the plan still caps the close-in
};
struct ra_state {
    float dstar; int init;      // the wanted gap, slewed
    float ot_w;                 // the overtake assist's pull in force (0..1), slewed
};
struct ra_out {
    int following;      // the ACC posted the lead (a valid lead in the radar's reach, the ACC on)
    float wanted_m;     // the wanted gap now (the slewed one)
    float v_law;        // the gap law's speed (m/s)
    float edge_w;       // 0..1 how far the lead is inside the radar's reach
    int posted;         // requests posted
    float w_calm, w_open, w_pull, w_cross;   // the cut-in weights of this tick
    float w_ot;                              // the overtake assist's pull
};
static inline void ra_reset(ra_state *st) { memset(st, 0, sizeof *st); }

// The room before the wanted gap, shaped for a crawl (the classic plan_close): at low speed and short gaps the truck matches the lead's speed earlier. It is the distance to post
// (before the edge of the radar), so that the closing speed the arbiter allows is the classic plan's times f.
// The planned room: what is left before the end of the approach, less what the truck covers while the brake comes on (RA_RHO_S x the closing speed), times RA_GAIN_SHARE.
// RA_RHO_S: the plan is made for a brake that is not there yet, and the sum of what is not there yet is: the build-up of a firm ask (the arbiter's firm level 2.2 m/s2 at the brake channel's
// firm jerk, 3 m/s3: 0.73 s), the brake's own lag (0.15 s, core/brake.cpp BC_REF_LAG less the half window), the lead model's estimate of a braking lead (half its 0.35 s window: 0.18 s)
// and the retarder's build-up (its time constant, 0.4 s): 1.46 s, 1.5 s here. The same in every mode: how long the brake takes to come on is not a matter of the mode's style. Without it the plan
// arrives late and then has to catch up: a jam's tail from 85 km/h passed at a time to collision of 3.4 s and a peak of 4.3 m/s2 where the classic ACC kept 5.2 s and 3.7 m/s2.
static const float RA_RHO_S = 1.5f;
static const float RA_STAND_V0 = 1.0f, RA_STAND_V1 = 3.0f;   // m/s: a lead below the first stands, above the second it drives
// (WP-SP3 round 3, item 6, the owner chose the continuous stop) The allowance is for a brake that is not there yet. A truck that already decelerates at what the geometry needs has paid it: a_dec, the
// deceleration it delivers now, over the deceleration the room needs without the allowance (closing^2 / 2 / room): the share still to pay is 1 - that. At the start of a braking event a_dec is 0 and
// the allowance is whole; at the end of a stop the brake has been on for seconds and the allowance (1.5 s x closing speed: 8 m at 20 km/h) stands for a lag that is not there: it stopped the truck 10 -
// 14 m early. The 0.7 share is for the lag too and is paid the same way. a_dec <= 0 or NAN = unknown = nothing paid.
static inline float ra_plan_room(float gap, float stopm, float dstar, float plan_to, float v, float closing, float rho_s, float a_dec = 0.0f, float w_pay = 1.0f)
{
    const float to_go = fmaxf(0.0f, gap - fmaxf(stopm, plan_to * dstar));
    const float a_kin = closing > 0 ? closing * closing / (2.0f * fmaxf(1.0f, to_go)) : 0.0f;
    float paid = a_dec == a_dec && a_dec > 0 && a_kin > 0.3f ? fminf(1.0f, a_dec / a_kin) : (a_dec == a_dec && a_dec > 0 && closing > 0 ? 1.0f : 0.0f);
    paid *= w_pay;   // only toward a lead that stands or is coming to stand: a lead that drives on is approached as before (round 2's rows)
    const float room = (RA_GAIN_SHARE + paid * (1.0f - RA_GAIN_SHARE)) * fmaxf(0.0f, to_go - (1.0f - paid) * rho_s * fmaxf(0.0f, closing));
    const float room_share = dstar > 0.5f ? fminf(1.0f, room / dstar) : 0.0f;
    const float slow = 1.0f - fminf(1.0f, v * 3.6f / RA_CRAWL_KMH);
    const float f = 1.0f - slow * (1.0f - room_share);      // the allowed closing speed times f; the distance posted times f^2
    return room * f * f;
}

// w: how far the point is in view (the radar's edge, a crossing car that goes by): at 0 it is infinitely far, so it comes into view and goes out without a step. A request now (d = 0) is posted whole
static inline void ra_add(sa_state *s, const sa_req &q, float w, ra_out *o)
{
    sa_req r = q;
    if (r.dist_m > 0) { if (w <= 0.001f) return; r.dist_m /= w; }
    sa_post_req(s, r); o->posted++;
}

static inline ra_out ra_post(sa_state *s, ra_state *st, const sa_cfg &c, const ra_in &in)
{
    ra_out o; memset(&o, 0, sizeof o);
    const lm_out &L = *in.lead;
    const float protect_max = ra_protect_max(in.aeb_max);   // see RA_GAIN_SHARE
    if (!in.on || !L.valid) { st->init = 0; st->ot_w = 0; return o; }   // off is only off: no lead, no requests, no state
    const float v = in.v, lv = fmaxf(0.0f, L.lv), gap = L.gap, dt = in.dt > 0 && in.dt < 0.5f ? in.dt : 0.0f;
    // 1. the wanted gap: the time gap x the LEAD's speed (it aims at once for the gap it will settle at), sliding to ours as the gap gets tense (no tipping point); slewed
    const float v_ref = v + (1.0f - L.w_tense) * (lv - v);
    const float d_raw = ra_wanted(in.level, ot_gap_s(in.t_gap_s, in.overtake), v_ref, in.stop_base_m, in.stop_grow_m, in.max_m);   // (the overtake assist: a little closer, never under a second)
    if (!st->init || fabsf(d_raw - st->dstar) > 60.0f) { st->dstar = d_raw; st->init = 1; }
    else st->dstar += fmaxf(-RA_SLEW_DN * dt, fminf(RA_SLEW_UP * dt, d_raw - st->dstar));
    const float dstar = st->dstar, stopm = ra_stop_gap(in.stop_base_m, in.stop_grow_m, v);
    o.wanted_m = dstar;
    o.edge_w = ra_smooth((RA_REACH_M - gap) / RA_EDGE_M);
    if (o.edge_w <= 0.001f) return o;                      // not in the radar's reach yet
    o.following = 1;
    // The cut-in as the ACC sees it (card 33, the classic's lessons): a lead that opens the gap is no reason to slow down; a car crossing in front is passed. Weights, not switches
    const float opening = lv - v;
    o.w_open = lm_ramp(opening, RA_OPEN_FROM, RA_OPEN_FULL) * lm_ramp(gap, RA_GAP_FROM * dstar, RA_GAP_FULL * dstar);
    o.w_pull = fmaxf(lm_ramp(opening, RA_PULL_FROM, RA_PULL_FULL), o.w_open);
    o.w_calm = in.calm ? o.w_pull * lm_ramp(gap, RA_CALM_MIN_M, RA_CALM_FULL_M) : 0.0f;
    {
        o.w_cross = in.cross ? L.cross_w : 0.0f;   // (the lead model's own weight: continuous, from the first tick of the pattern: nothing needs slewing again)
        const float ot_step = dt / RA_OT_S;
        st->ot_w += fmaxf(-ot_step, fminf(ot_step, (in.overtake ? 1.0f : 0.0f) - st->ot_w));
        o.w_ot = st->ot_w;
    }
    // 2. the moving target: the lead's speed when the gap is the wanted gap (a standing lead: the same with speed 0, the stop at the stop distance)
    sa_req t; memset(&t, 0, sizeof t);
    t.owner = SAO_LEAD; t.cls = SAC_FOLLOW; t.accel_to = -1; t.flags = SAF_NOTRIM; t.grade_pct = in.grade_pct;
    t.v_ms = lv; t.v_move_ms = lv;
    // (round 3, item 6) The approach to a lead that STANDS ends at the stop distance, not at the wanted gap of OUR speed. The wanted gap slides to our speed as the gap gets tense (the reserve for a lead
    // that brakes, round 2), but a standing lead leaves no time gap to keep: the plan was made to end 8 - 11 m behind it. The end of the plan is the wanted gap at the speed the plan ENDS at, the
    // lead's: whole below 1 m/s (the 'standing' of the reason string), gone at 3 m/s; the reserve for a lead that still drives is untouched.
    const float w_stand = 1.0f - lm_ramp(lv, RA_STAND_V0, RA_STAND_V1);
    const float d_end = ra_wanted(in.level, ot_gap_s(in.t_gap_s, in.overtake), lv, in.stop_base_m, in.stop_grow_m, in.max_m);
    const float dplan = dstar + w_stand * (fminf(d_end, dstar) - dstar);
    t.dist_m = fmaxf(ra_plan_room(gap, stopm, dplan, in.plan_to, v, v - lv, RA_RHO_S, in.a_dec, w_stand), RA_D_FLOOR_M);
    // The calm approach (card 19: no gas, then brake) belongs to a lead we have to slow down for. At 1 approach time x coast deceleration of speed over it (Normal 1.0 m/s, sa_nf_x) lifting
    // off no longer closes the difference in an approach: from there on it is whole; a lead at our speed or faster is not something to slow for, we close up on it as the set allows
    t.nocalm = fmaxf(1.0f - ra_smooth((v - lv) / sa_nf_x(c, in.mode)), st->ot_w);   // (and not at all while the assist pulls towards the set: it accelerates while it closes in, the plan caps the close-in)
    // It brakes as soon as lifting off cannot do it (the classic: "brake early and gently when easing off is not enough"), not at the road's comfort level: a vehicle's braking is
    // progressive, so the ask starts at what coasting gives and grows with the geometry; the step at its start is the coast deceleration, not 1.05
    t.coast_decel = fminf(c.coast_decel, fmaxf(RA_COAST_MIN, in.nat));   // lifting off is planned with what the truck really gives now, never above the central value, never below 0.1 (sa_a_coast's floor)
    t.plan_decel = t.coast_decel;
    // (round 4, item 3) How firm the time-gap row may brake. The wanted gap is a comfort, not a protection: with the truck at it and the lead braking, "be at its speed at the wanted gap" needs the whole
    // closing speed shed in the metre or two that are left, 1.7 m/s2 where the classic ACC let the gap close up and braked at 1.07 (varying lead, 40 t, rolling road: the retarder gives 1.35, the service
    // brake the rest, 301 kJ against 171 and a time to collision of 18 s all the way). The row asks the comfort deceleration of the mode while the time to collision is long (12 s and over: twice the lead
    // model's 'tense' threshold, LM_TTC_FROM_S) and its class's whole level (the firm 2.2) from 6 s, linearly between: no step. A ramp that reaches into the 3 - 6 s window braked a cut-in later and deeper
    // (26 t, ACC level 0: minimum speed 52.7 against 55.5 km/h, +1.4 % fuel). The protection rows below are not touched: they take over where the gap gets short
    {
        const float a_comf = c.brake_comfort * c.brake_k[in.mode < 0 ? 1 : in.mode > 2 ? 2 : in.mode];
        const float w_ttc = L.closing > 0.3f ? lm_ramp(2.0f * LM_TTC_FROM_S - L.ttc, 0.0f, LM_TTC_FROM_S) : 0.0f;
        t.brake_cap = a_comf + (c.brake_firm - a_comf) * w_ttc;
    }
    t.why = lv < 1.0f ? "vehicle ahead, standing" : "vehicle ahead (time gap)";
    const float w_far = o.edge_w * (1.0f - o.w_cross);   // in view: inside the radar's reach and not a car that crosses
    ra_add(s, t, w_far, &o);
    // 3. the gap law: lead speed + gain x (gap - wanted gap), now. Uphill the gap may grow by a band before the truck pulls up for it (30-10: no chasing the lead up a climb in a low
    // gear, and a narrow band ends the pumping between a little throttle and a little retarder); it narrows to nothing as the gap gets tense
    float v_law = fmaxf(0.0f, lv + in.gain * (gap - dstar));
    if (in.band > 0) {
        const float hi = dstar * (1.0f + in.band), here = fminf(v, in.set), t_hi = fminf(in.set, fmaxf(0.0f, lv + in.gain * (gap - hi)));
        if (v_law > here) v_law += (1.0f - L.w_tense) * (fmaxf(here, t_hi) - v_law);
    }
    // the overtake assist pulls towards the set: the gap law gives way to it, over a second. The moving target above stays: it caps the pull at the speed from which the truck still ends at the
    // lead's speed at the (shorter) wanted gap
    if (st->ot_w > 0) v_law += st->ot_w * (fmaxf(v_law, in.set) - v_law);
    // a lead that opens the gap, or a car that crosses: no slowing down for it, only no speeding up: the target is not lower than our speed (the classic's hold = max(target, speed))
    const float w_hold = fmaxf(o.w_calm, o.w_cross);
    if (w_hold > 0) v_law += w_hold * (fmaxf(v_law, fminf(v, in.set)) - v_law);
    o.v_law = v_law;
    // It brakes only beyond a band: the speed the truck loses by coasting for one approach time (the arbiter's own two time constants, sa_nf_x: Normal 1 m/s). Inside it the truck coasts (a
    // gap that is a little short on a descent closes by lifting off); beyond it the excess over the law is braked in an approach time, a proportional ask without a gate
    sa_req g = t; g.brake_cap = 0; g.v_move_ms = 0; g.v_ms = v_law; g.dist_m = 0; g.flags = SAF_NOTRIM; g.band_ms = sa_nf_x(c, in.mode); g.plan_decel = 0; g.nocalm = 0;
    g.why = o.w_cross > 0.5f ? "crossing vehicle ahead (passed)" : o.w_ot > 0.5f ? "vehicle ahead, overtake assist" : o.w_calm > 0.5f ? "vehicle ahead pulls away (holding the speed)" : "vehicle ahead (gap)";
    ra_add(s, g, 1.0f, &o);
    // 4. the braking lead: stop where it will have stopped if it goes on braking as it does now. Its distance over the braking weight: weight 0 = infinitely far
    if (L.w_brake > 0.001f) {
        // How long the lead goes on braking, as it does now. A lead that brakes gently (up to RA_BRAKE_SOFT) is slowing for traffic and will soon stop braking: the truck is planned to match it as it
        // will be after one more approach time of ours (the arbiter's tau of the Normal mode, 2.5 s, in every mode: how long a lead keeps braking is not a matter of the truck's style; Power's
        // own 1.5 s left a jam's tail to the AEB), a receding horizon. A lead that brakes harder than an
        // ACC itself may (RA_BRAKE_HARD) is stopping: the truck is planned to stop behind the place where it will stand. In between the horizon grows continuously from the one to the other.
        // (A plan for a stop alone braked a lead that only slows from 85 to 70 km/h at 2.4 m/s2 with 2.1 m/s2, the classic with 1.1: 40 t, rolling road; a receding horizon alone left
        // a lead that brakes at 6 m/s2 to the AEB: 14 s of it in the hard-brake rows against none.)
        const float a_l = fmaxf(0.05f, -L.a_lead), th_stop = lv / a_l, th_soft = fminf(c.tau_s[1], th_stop);
        const float th = th_soft + lm_ramp(a_l, RA_BRAKE_SOFT, RA_BRAKE_HARD) * (th_stop - th_soft);
        const float v_pred = fmaxf(0.0f, lv - a_l * th), dx = lv * th - 0.5f * a_l * th * th;
        // it stops at least half the wanted gap behind that (the classic's gmin = max(stop gap, 0.5 x wanted): a lead that brakes harder than it has been does not leave the truck a stop
        // distance to spare, ride 12-10 20:54)
        const float gmin = fmaxf(stopm, 0.5f * dstar);
        sa_req b = t; b.brake_cap = 0; b.v_ms = v_pred; b.v_move_ms = v_pred; b.dist_m = fmaxf((fmaxf(0.0f, gap + dx - gmin)) / L.w_brake, RA_D_FLOOR_M); b.why = "vehicle ahead braking";
        ra_add(s, b, w_far, &o);
        sa_req pb = b; pb.cls = SAC_PROTECT; pb.v_ms += RB_FAR_TOO_FAST_KMH / 3.6f; pb.plan_decel = c.brake_firm; pb.brake_cap = fmaxf(c.brake_max, protect_max); pb.why = "vehicle ahead braking, closing too fast";
        ra_add(s, pb, w_far, &o);
    }
    // 5. a protection behind the moving target: the firm plan continues up to the maximum when closing in needs more than the firm level
    sa_req p = t; p.cls = SAC_PROTECT; p.v_ms += RB_FAR_TOO_FAST_KMH / 3.6f; p.plan_decel = c.brake_firm; p.brake_cap = fmaxf(c.brake_max, protect_max); p.why = "vehicle ahead, closing too fast";
    ra_add(s, p, w_far, &o);
    return o;
}
