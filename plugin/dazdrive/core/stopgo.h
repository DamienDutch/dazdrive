// Stop&Go for the layers (WP-SP3 item 4, docs/cruise-layers.md; owner 54.2): the ACC follows a vehicle down to a stop and goes again by itself when it drives off, within
// the first 2 s of standing; after that the truck waits for Resume or the throttle (the real trucks': MAN, Detroit). PURE: no game call, header only.
//
// What it decides is bookkeeping, as the classic loop kept it (modules/cruise.cpp, `if (g_sg_ok)` in cruise_body and cr_layers_body: carried there verbatim, rows below hold this
// core to it tick for tick):
//   hold   the game's cruise is kept on at a standstill behind a vehicle (the patch that skips the game's cancel at a standstill and feeds the automatic at walking pace)
//   wait   the truck has stood longer than stop_go_auto_s: the cruise is not kept any more (the dash shows B), Resume or the throttle drives off
//   feed   the throttle fed to the automatic at walking pace so it does not drop to neutral
// and one thing that is new: `stay`. While the truck waits for Resume or the throttle the ACC does not drive off by itself, even when the vehicle ahead does and even while the game has
// not cancelled the cruise yet: the throttle of the cruise is 0 until the driver presses Resume (or the pedal, which wins anyway). It is not a brake: the standstill is held by what holds it
// today (the module's ACC hold at the stop gap; the low-speed owner while it owns the standstill; Auto Hold after a pedal stop), SP5 unifies them.
#pragma once
#include <math.h>
#include <string.h>

struct sg_cfg {
    int on;            // [cruise] stop_go
    float auto_s;      // [cruise] stop_go_auto_s: after standing this long driving off needs Resume or the pedal (0 = always drives off by itself)
};
struct sg_in {
    double now;        // s (any clock, only differences count)
    float kmh;         // our speed
    int owner;         // the low-speed owner (modules/lowspeed.cpp) owns the standstill
    int zone, stopped; // its Stop&Go zone and its standstill (its own definitions, used when it owns)
    int lead;          // a vehicle ahead in the radar (gap > 0.5 m)
    int parking;       // the parking brake is on
    float thr;         // the throttle the cruise commands now
    int go;            // the driver presses Resume, or the throttle, now
};
struct sg_state {
    double stopped_at; // when the truck came to a standstill (0 = it moves)
    int go_latched;    // the driver asked to go during this wait
};
struct sg_out {
    int hold, wait;    // see above
    float feed;        // the throttle to feed (0 = none)
    int stay;          // the ACC does not drive off by itself (the throttle of the cruise is 0)
    float stood_s;     // how long it has stood
};
static inline void sg_reset(sg_state *s) { memset(s, 0, sizeof *s); }
static inline sg_out sg_step(sg_state *s, const sg_cfg &c, const sg_in &in)
{
    sg_out o; memset(&o, 0, sizeof o);
    const int slow = in.owner ? in.zone : in.kmh < 3.0f, stopped = in.owner ? in.stopped : slow;
    if (!stopped) s->stopped_at = 0;
    else if (!s->stopped_at) s->stopped_at = in.now;
    const int may_wait = c.auto_s <= 0 || (s->stopped_at ? in.now - s->stopped_at < c.auto_s : in.owner);
    o.hold = c.on > 0 && slow && in.lead && may_wait && !in.parking;
    o.wait = c.on > 0 && c.auto_s > 0 && slow && s->stopped_at && !may_wait && !in.parking;
    o.feed = (c.on > 0 && in.kmh < 10.0f && (o.hold || s->stopped_at) && in.thr > 0.02f) ? in.thr : 0.0f;
    if (!o.wait) s->go_latched = 0; else if (in.go) s->go_latched = 1;
    o.stay = o.wait && !s->go_latched;
    o.stood_s = s->stopped_at ? (float)(in.now - s->stopped_at) : 0.0f;
    return o;
}
