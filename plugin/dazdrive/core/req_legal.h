// The legal class's requesters of the speed arbiter (WP-SP2, docs/cruise-layers.md; review cards 8 and 13). PURE: no game call, header only; the module hands over
// what it senses and these only ASK (core/speedarb.h decides how hard). Off = they post nothing.
//
//   the limit line   never more than the road's limit + the overspeed cap (5 km/h in ETS2, 10 % in ATS), whatever pushes the truck: a descent, the set speed, a raise.
//                    An ABSOLUTE line (it does not move with the set), urgent (the channel gives the retarder no tolerance), looked at 1.5 s ahead.
//   the descent hold the set speed + the descent allowance (8 km/h scaled with the set), + 2: the engine brake / retarder hold it first, ours comes in after them. RELATIVE to
//                    the set (a raise moves it). Posted on a road that is flat as well: the arbiter coasts first, so it only matters when something pushes the truck
//                    over (a descent, a set that was lowered), which is what the old descent block and SP1's band did.
#pragma once
#include "speedarb.h"

// ---- the limit line (card 13). line_kmh = the highest speed we hold the truck to (limit + cap, or the set itself when that is higher); NAN = none (no limit known, no cap).
// The look-ahead: the game's retarder builds up over ~1 s and the channel slews a request at 5/s, so the BRAKE counts the speed in 1.5 s at the acceleration of now
// (the old line's projection, kept; a negative band, so the chase target is the line itself and the set is not held back while it accelerates), the acceleration being
// what the slope can give (0.3 + g x the descent grade): a spike of the measured acceleration (a teleport, a hitch) must not ask a brake on the level (sim_pcc's start, 30-10).
struct rl_line_in {
    float line_kmh;
    float accel;          // m/s2, the telemetry's
    float grade_pct;      // the slope under the truck (the lower of the placement's pitch and the route's grade just ahead), % (+ uphill)
};
static const float RL_LOOK_S = 1.5f;            // s
static const float RL_LINE_MARGIN_KMH = 0.5f;   // the line is held half a km/h under, as the old line did (its start threshold)
static inline void rl_post_line(sa_state *s, const rl_line_in &in)
{
    if (!(in.line_kmh > 1)) return;
    const float gd = fmaxf(0.0f, -in.grade_pct);
    const float acc_c = fminf(fmaxf(0.0f, in.accel), 0.3f + 9.81f * gd / 100.0f);
    sa_req q; memset(&q, 0, sizeof q);
    q.owner = SAO_LIMIT; q.cls = SAC_LEGAL; q.why = "speed limit line (overspeed cap)"; q.accel_to = -1; q.flags = SAF_URGENT;
    q.v_ms = in.line_kmh / 3.6f;                                           // what the truck may chase: the line itself (a raise stops here)
    q.band_ms = -(RL_LINE_MARGIN_KMH / 3.6f + acc_c * RL_LOOK_S);          // what the brake counts: the speed in 1.5 s, half a km/h under the line (a look-ahead, not a lower target)
    sa_post_req(s, q);
}

// ---- the descent hold (card 8). set_ms is the cruise's set speed, desc_kmh the allowance (descent_overspeed_kmh x the set's scale), hold_kmh what the game's own
// overspeed braking grants now (NAN = not known; the module passes it on a descent only). The old block added 2 km/h so that its service-brake request came in after the
// game's retarder (PCC sim 29-10); here the brake channel stages the retarder and the engine brake first (the request is comfortable: the retarder first, the service
// brake only for what it cannot give), so the line is the allowance itself, as SP1's band was.
static inline void rl_post_descent(sa_state *s, float set_ms, float desc_kmh, float hold_kmh)
{
    sa_req q; memset(&q, 0, sizeof q);
    q.owner = SAO_DESCENT; q.cls = SAC_LEGAL; q.why = "descent hold (set speed + allowance)"; q.accel_to = -1; q.flags = SAF_SET;
    const float allow = fmaxf(fmaxf(0.0f, desc_kmh), hold_kmh == hold_kmh ? hold_kmh : 0.0f);
    q.v_ms = set_ms + allow / 3.6f;
    sa_post_req(s, q);
}
