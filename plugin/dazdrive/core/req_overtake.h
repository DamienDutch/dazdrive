// The overtake assist for the layers (WP-SP3 item 5, docs/cruise-layers.md; owner 54.5 "as intended, test first"; review card 37). PURE: no game call, header only.
//
// What the classic overtake_tick did (modules/cruise.cpp, built 30/31-10, NEVER driven), as a state machine the ACC requester reads: when the truck follows a clearly slower vehicle and the
// driver puts the LEFT indicator on, the cruise prepares the overtake as a real overtake assist does: it closes in a little (a shorter time gap, never under 1 s) and pulls towards the set
// speed, with the approach plan still capping it for a safe close-in (core/req_acc.h: the moving target stays, the gap law that would hold it at the wanted gap is not posted). The reasons it
// does not start are the classic's: only Normal and Power (never Eco, never M), a vehicle clearly slower than the set, the indicator held 0.2 s (a flicker is no lever) and a FRESH pull of the lever
// (an end while the lever is still held does not start it again), the blind-spot detector running and the left lane clear (without trusted data it withholds, and says why), no brake, no
// braking of the ACC itself, no AEB. It ends on the indicator going off (0.3 s grace: a blink is not the lever), a lane change (the lead is gone), a brake, the ACC braking, the AEB, the cruise
// off, the ini switch, or 8 s without a lane change; the gap is given back softly (the wanted gap's own slew, core/req_acc.h). Left lane only (right-hand traffic: the classic did not know the
// traffic side). Rows only: the owner tests it on a ride.
//
// The module logs the events (at most one on/off line per 2 s, a withheld line per 5 s) and tells TrueShift (cruise_overtake_on) so the pull does not become a kickdown.
#pragma once
#include <math.h>
#include <string.h>

static const float OTL_TIMEOUT_S = 8.0f;      // the assist gives up this long after it started without a lane change (two of the longest lane changes, 4 s each)
static const float OTL_GAP_MIN_S = 1.0f;      // the overtake time gap never goes below this (s)
static const float OTL_GAP_PULL_S = 0.25f;    // ... and is this much shorter than the player's own gap
static const float OTL_IND_CONFIRM_S = 0.2f;  // the signal must be on this long before it counts (a flicker does not)
static const float OTL_IND_OFF_S = 0.3f;      // and off this long before it ends (a blink is not the lever)

struct ot_in {
    double now;          // s of the simulation (a pause does not run the timeout down)
    int enabled;         // [cruise] overtake_assist and the own ACC (the layers' ACC) on
    int cruise_on, lead; // the cruise is on, a lead is followed
    float lv_ms, set_kmh;
    int mode;            // 0 Eco, 1 Normal, 2 Power, 3 M
    int ind_left;        // the left indicator is on now
    int brake;           // the driver's brake pedal
    int acc_braking;     // the ACC itself asks for a brake
    int aeb;             // the AEB is warning or braking
    int bs_trusted;      // the blind-spot detector runs on trusted data
    int bs_left;         // a vehicle in the left blind spot
};
struct ot_state { int on; double t0, ind_on_at, ind_off_at; int ind_was; };
struct ot_out {
    int on;
    int started, ended;           // an event of this tick
    const char *why;              // why it ended (NULL when it did not)
    const char *withheld;         // why it did not start although everything else asked for it (NULL otherwise): the module logs it
    double t_on;                  // how long it has been on (s)
};
static inline void ot_reset(ot_state *s) { memset(s, 0, sizeof *s); }
static inline ot_out ot_step(ot_state *s, const ot_in &in)
{
    ot_out o; memset(&o, 0, sizeof o);
    const double now = in.now;
    // the signal's confirmation and off-grace bookkeeping. Only a fresh pull of the lever starts a confirmation: an end while the lever is still held (timeout, brake) must not re-arm on the stale on-time
    if (in.ind_left) { if (!s->ind_was) s->ind_on_at = now; s->ind_off_at = 0; }
    else { s->ind_on_at = 0; if (s->on && !s->ind_off_at) s->ind_off_at = now; }
    s->ind_was = in.ind_left;
    if (s->on) {
        const char *why = NULL;
        if (!in.enabled) why = "the assist or the own ACC is off in the ini";
        else if (!in.cruise_on) why = "cruise off";
        else if (!in.lead) why = "the lead is gone (lane changed)";
        else if (in.brake || in.acc_braking || in.aeb) why = in.brake ? "brake" : in.aeb ? "the AEB" : "the ACC is braking";
        else if (!in.ind_left && now - s->ind_off_at >= OTL_IND_OFF_S) why = "the indicator went off";
        else if (now - s->t0 >= OTL_TIMEOUT_S) why = "no lane change within the timeout";
        if (why) { o.t_on = now - s->t0; s->on = 0; s->ind_off_at = 0; s->ind_on_at = 0; o.ended = 1; o.why = why; }   // (ind_was keeps a held lever from re-arming it)
        else o.t_on = now - s->t0;
        o.on = s->on;
        return o;
    }
    // not active: consider starting
    if (!in.enabled || !in.cruise_on || !in.lead || !in.ind_left || !s->ind_on_at) return o;
    if (in.mode == 0 || in.mode == 3) return o;
    if (!(in.lv_ms * 3.6f < in.set_kmh - 1.0f)) return o;
    if (in.brake || in.acc_braking || in.aeb) return o;                    // the same ends: no tick-rate flapping
    if (now - s->ind_on_at < OTL_IND_CONFIRM_S) return o;
    if (!in.bs_trusted) { o.withheld = "no trusted blind-spot data"; return o; }
    if (in.bs_left) { o.withheld = "the left lane is occupied"; return o; }
    s->on = 1; s->t0 = now; o.started = 1; o.on = 1;
    return o;
}
// The time gap during the assist: a little shorter than the driver's own, never under one second, never wider than the normal gap
static inline float ot_gap_s(float t_s, int on)
{
    if (!on) return t_s;
    const float floor_s = fminf(t_s, OTL_GAP_MIN_S);
    return fminf(t_s, fmaxf(floor_s, t_s - OTL_GAP_PULL_S));
}
