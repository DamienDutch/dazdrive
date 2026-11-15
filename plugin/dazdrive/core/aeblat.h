// The AEB's sideways reading: is the vehicle the game's radar offers as a lead really in OUR path? Pure math, no game access: modules/cruise.cpp calls it,
// the sim (tools/sim/sim_cruise.cpp, rows in sim_aeb_lat_rows.inc) runs exactly this.
//
// The radar gives a distance and a speed, nothing sideways. Rides of 14-11 showed it offering a target 2.4 - 4.3 m beside us while we passed it (the trailer of the
// truck just overtaken, a car standing at a junction), and the AEB braked fully for it. The traffic list (core/traffic.h) knows where every vehicle is, so the radar
// target is found in it (same distance, from the rear face), put into the frame of OUR expected path (core/pathframe.h: the arc our yaw rate draws), and the space
// between our body and its box where we will be when we get there says how much of the stop the AEB may ask for: a weight, 1 for a vehicle in the path, falling to 0
// at a clear lane's width. "Where it is when we get there": the vehicle's own lateral speed across the path (its heading and speed, core/pathframe.h pf_lat_speed)
// carried forward for the time it takes to reach it when it moves TOWARD the path, so a car cutting in counts at once. And an allowance for the bend we have not entered
// yet, which moves a lead that is in the lane sideways by the square of its distance.
//
//   w = 1                         the box overlaps our path (clearance <= 0)
//   w = 1 - clearance / AL_C1     in between
//   w = 0                         a free lane between us (clearance >= AL_C1)
//
// Fails safe: no list, a stale list, no own front, no vehicle at that distance: matched = 0 and the caller keeps today's behavior (w = 1). Two vehicles at that distance:
// the one with the least clearance (the more dangerous reading) counts.
#pragma once
#define DAZ_AEBLAT 1   // (the sim rows of this reading compile their pure part only when it exists)
#include "traffic.h"
#include "pathframe.h"
#include <math.h>

// Derivations (not tunings of one truck):
//  AL_C1 = 1.0 m: the clearance a truck keeps in the next lane. A lane is 3.5 - 3.75 m (core/pathframe.h), a truck 2.55 m wide: 3.7 - 2.55 = 1.15 m centred; the AI
//          cars wobble 0.3 m inside their lane (pf_cfg next_max), so 1.0 m still means "the next lane" and less is a vehicle that is half in ours.
//  AL_PASS_M = 0.6 m: the driver signals AWAY from the target. A deliberate lane change moves a car sideways at 0.74 m/s at the slowest (pf_cfg v_tow) and a driver
//          reaction takes 0.8 s (pf_cfg tau_min): 0.74 x 0.8 = 0.6 m of clearance he has begun to make. Only toward the free side; toward the target it is no credit.
//  AL_TOL_M = 2.0 m: the radar gap and the list's box edge agree within this. The error is our own front (the list gives the front extent of the body from the
//          placement origin: 0.3 m off on the 14-11 truck, 1.0 m on the bus whose radar starts at the nose), the box centre of a long trailer (0.3 m) and the radar's
//          noise (0.3 m); the extrapolation below takes out the age of both readings.
//  (no AL_AMBIG_M since 15-11: every vehicle inside AL_TOL_M may be the radar's lead, so the one with the least clearance counts; a best fit alone let a car beside
//          a staggered queue, or a bus nose 1 m off, hide the lead in our lane: no stop at all. A missed stop is a crash, a needless one a bad moment.)
//  AL_STALE_S = 0.6 s: the list is walked every 200 ms (modules/traffic.cpp); three walks old is no reading any more.
//  AL_HORIZON_S = 3.0 s: a movement TOWARD the path is carried forward for the whole time to reach it, up to the time a lane change takes (pf_cfg horizon, the low end of
//          3 - 5 s). A movement away is not carried forward at all (below).
//  AL_BEND_LAT = 1.0 m/s2: the bend we are not in yet. A lead 90 m ahead in a bend that begins at our nose sits k x s^2 / 2 to the side (16 m on 14-11, 584 s: the truck
//          in a left bend 255 m in radius at 108 km/h) while our yaw still says straight. Roads are designed for about 1 m/s2 of unbalanced sideways acceleration at the speed
//          they are driven (a side friction factor of 0.1: R = 490 m at 80 km/h, 190 m at 50); a sharper bend is signed for a lower speed and a driver who has slowed turns
//          already, which the yaw rate shows. The allowance a_lat s^2 / (2 v^2) is taken off the clearance, and fades out as our own turning rate reaches the sharpest a road
//          has at this speed (k = a_lat / v^2): a bend we are in is the bend. It is 0.1 m at 12 m ahead and 19 m/s, 0.6 m at 9 m and 8 m/s (the false braking of 14-11 was all
//          within 13 m), 1.3 m at 47 m and 21 m/s, 4.7 m at 92 m and 30 m/s.
//  AL_DV = 3 m/s: the list's speed of the vehicle along our heading and the radar's agree within this: the radar's own noise is 0.3 m/s, the rest is the angle between the
//          two headings (a lead in a bend at 20 degrees to us: 6 % of 25 m/s = 1.5 m/s). Only for a vehicle whose speed is known (a trailer's is not).
//  The lateral speed is not differenced from the list's positions: they swing a metre per walk at 60 m when the yaw rate jitters (the offset of the arc grows with s^2).
//  It is the vehicle's own heading and speed against the path (not known for a trailer: 0).
static const float AL_C1 = 1.0f, AL_PASS_M = 0.6f, AL_TOL_M = 2.0f, AL_HORIZON_S = 3.0f, AL_BEND_LAT = 1.0f, AL_DV = 3.0f;
static const double AL_STALE_S = 0.6;

struct al_in {
    double now;          // wall_s()
    double gap_at;       // wall_s() of the radar reading
    float gap;           // the radar's gap to the lead, m
    float closing;       // our speed minus the lead's, m/s (+ = closing)
    float v;             // our speed, m/s
    float yaw;           // our turning rate, turns/s, + = left (T.yaw)
    int ind_side;        // our indicator: + left, - right, 0 none or hazards
    float lead_v;        // the radar's lead speed, m/s (NAN = not given)
};

struct al_out {
    int matched;         // 1 = the radar target was found in the list
    unsigned id;
    float d;             // m, + left of our path: the box centre across the path
    float clr;           // m: the lateral space between our body and its box now, less the allowance for a bend not entered, negative = overlap
    float clr_pred;      // m: the same where it is when we get there (its lateral speed carried forward); the weight is made of this
    float w;             // 0..1: how much of the stop the AEB may ask for (1 when not matched)
    int n_cand;          // vehicles at that distance
};

// 1 at a clearance of 0 and less, 0 at AL_C1 and more
static inline float al_weight(float clr_eff)
{
    float w = 1.0f - clr_eff / AL_C1;
    return w < 0 ? 0.0f : w > 1 ? 1.0f : w;
}

// Our front extent from the placement origin: the body the blind-spot zones follow (the player list's box when it looks right, else the wheel guess), else the wheel guess.
static inline float al_front(const traffic_around &ta)
{
    if (ta.bs_tf == ta.bs_tf && ta.bs_tf > 0.5f) return ta.bs_tf;
    if (ta.truck_front_z == ta.truck_front_z && ta.truck_front_z < -0.5f) return -ta.truck_front_z + 1.2f;
    return NAN;
}

static inline al_out al_read(const traffic_around &ta, const al_in &in)
{
    al_out o;
    o.matched = 0; o.id = 0; o.d = 0; o.clr = 0; o.clr_pred = 0; o.w = 1.0f; o.n_cand = 0;
    if (!ta.ok || !(ta.read_t > 0) || in.now - ta.read_t > AL_STALE_S || in.now < ta.read_t - 1.0) return o;
    float front = al_front(ta);
    if (!(front == front)) return o;
    const float hw = ta.bs_hw == ta.bs_hw && ta.bs_hw > 0.5f ? ta.bs_hw : 1.275f;
    // our path: the arc the yaw rate draws (a standstill's 3 m/s floor only keeps the division finite; the AEB is off there anyway)
    const float k = in.yaw * 6.2831853f / fmaxf(in.v, 3.0f);
    // the radar's gap, brought to now at its closing speed (it is a frame old at most; the list up to 0.2 s + its age)
    const float gap_now = in.gap - in.closing * (float)(in.now - in.gap_at);
    const float age = (float)(in.now - ta.read_t);
    int n = 0;
    float cl[16], dd[16], ex[16], vl[16]; unsigned ids[16];
    for (int i = 0; i < ta.n_ahead && n < 16; i++) {
        const traffic_obj &t = ta.ahead[i];
        if (!(t.dx > 0) || !(t.len > 0.5f) || t.dx != t.dx || t.dy != t.dy) continue;
        const float dxn = t.dx - in.closing * age;
        const float e = fabsf(dxn - 0.5f * t.len - front - gap_now);
        if (e > AL_TOL_M) continue;
        // not the radar's lead: one that comes toward us, or whose speed along our heading is not the lead's (the speed of a trailer is not known: no test)
        if (t.hdg == t.hdg && fabsf(t.hdg) > 100.0f) continue;
        if (in.lead_v == in.lead_v && t.v == t.v && t.hdg == t.hdg && fabsf(t.v * cosf(t.hdg * 0.017453292f) - in.lead_v) > AL_DV) continue;
        const float h = t.hdg == t.hdg ? t.hdg * 0.017453292f : 0.0f;
        const float wid = t.wid > 0.5f ? t.wid : 2.5f;
        // the box seen from the path: its centre in the path frame, and the half extent of the box across our heading (a vehicle turned by h reaches |sin h| x
        // half its length sideways, the more so the longer it is: a bounding box, the more careful reading)
        double s, d;
        pf_to_path(k, dxn, t.dy, &s, &d);
        const float ext = 0.5f * wid * fabsf(cosf(h)) + 0.5f * t.len * fabsf(sinf(h));
        const float v2 = fmaxf(in.v * in.v, 9.0f);
        const float bend = 0.5f * AL_BEND_LAT * (float)(s * s) / v2 * fmaxf(0.0f, 1.0f - fabsf(k) * v2 / AL_BEND_LAT);   // the bend we are not in yet (none, when we turn as sharply as a road does)
        cl[n] = (float)fabs(d) - hw - ext - bend; dd[n] = (float)d; ids[n] = t.id; ex[n] = ext + bend;
        vl[n] = t.kind == 0 && t.v == t.v && t.hdg == t.hdg ? (float)pf_lat_speed(k, s, h, t.v) : 0.0f;
        n++;
    }
    if (n == 0) return o;
    // the vehicle the radar means: any that fits the distance within AL_TOL_M; the one with the least clearance wins
    int pick = -1;
    for (int i = 0; i < n; i++)
        if (pick < 0 || cl[i] < cl[pick]) pick = i;
    o.matched = 1; o.id = ids[pick]; o.d = dd[pick]; o.clr = cl[pick]; o.n_cand = n;
    // where it is across the path when we get there: a vehicle that moves TOWARD the path is carried forward for the whole time to reach it (up to the time a lane change
    // takes), one that moves away is not: its heading may be the bend's, not a lane change (a lead in a bend we have not entered turns away from our straight line), and the
    // two errors are not the same size, a cut-in that is not braked for is a crash and a lead that left and was braked for is a bad moment. A vehicle that crosses the path on
    // the way is past it when we arrive.
    const float vlat = vl[pick];
    const float ttc = in.closing > 0.3f ? fmaxf(0.0f, in.gap) / in.closing : AL_HORIZON_S;
    const float tau = vlat * o.d < 0 ? fminf(AL_HORIZON_S, ttc) : 0.0f;
    o.clr_pred = (float)fabs(o.d + vlat * tau) - hw - ex[pick];
    // a driver who signals away from it and goes around: part of the clearance is made already
    const int away = in.ind_side != 0 && (in.ind_side > 0) == (o.d < 0);
    o.w = al_weight(o.clr_pred + (away ? AL_PASS_M : 0.0f));
    return o;
}
