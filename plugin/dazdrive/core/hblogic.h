// The high-beam assist's decision (modules/lights.cpp) as plain logic, so a sim can feed it traffic sequences. No SDK, no globals.
//
// High beam ON when it is dark, fast enough, in no tight bend, our indicator is off, and nothing was in front of us for hb_delay_s.
// High beam OFF at once (no delay) when something is: an oncoming vehicle within oncoming_m, a vehicle ahead in our direction within
// lead_m, a tight bend, too slow, the indicator, or no traffic reading. "Dark" = the low beam is on (the driver or the game's auto lights
// say it is dark) AND the game clock is outside the daytime (a driver who keeps the low beam on all day must not get a high beam at noon).
//
// What the traffic list can see (modules/traffic.cpp, the `ahead` list): vehicles up to 130 m away, within 25 m sideways, from 15 m behind.
// So oncoming_m and lead_m are clamped to 130 m, and a bend tighter than R = 130^2 / (2 x 25) = 338 m puts an oncoming vehicle beyond
// the 25 m sideways window at the end of the range: there the assist does not know, and the low beam stays.
//
// Every call names the term that decided (or, when nothing is decided, the one that holds the decision back) in hb_out::term and in the why text
// with the value and the threshold (promise P7); the module logs a line when the decision changes.
#pragma once
#include <math.h>
#include <string.h>
#include <stdio.h>

// The two figures below are the filter of the `ahead` list in modules/traffic.cpp (dist < 130 and |dy| < 25, "the wider list for the cut-in
// reading"). They are the sensor's reach, so this logic cannot derive them: a change there has to be made here too (no shared constant while
// traffic.cpp is a module; they would move to core/traffic.h when the traffic read moves into sense).
static const float HB_PROBE_RANGE_M = 130;     // traffic.cpp: the `ahead` list takes vehicles with dist < 130
static const float HB_PROBE_SIDE_M = 25;       // ... and |dy| < 25
static const float HB_BEND_R_MIN_M = HB_PROBE_RANGE_M * HB_PROBE_RANGE_M / (2 * HB_PROBE_SIDE_M);   // 338 m

struct hb_veh { float dx, dy, dist, hdg; };    // metres ahead / left, distance, heading relative to ours (degrees, + = left)

struct hb_cfg {
    float min_kmh;          // high beam only above this speed (a margin of hyst_kmh both ways)
    float hyst_kmh;
    float oncoming_m;       // an oncoming vehicle this close (clamped to the probe range) puts the beam low
    float lead_m;           // a vehicle ahead in our direction, this close
    float delay_s;          // clear for this long before the beam goes back up
    float dark_from_h, dark_to_h;   // the clock window in which it can be dark enough (hours of the game day; from > to = over midnight)
    float driver_hold_s;    // after the driver changed the beam: hands off for this long
    float lead_half_w;      // lateral half width of "ahead in our direction" (m): our lane and the next
    float list_age_s;       // a traffic reading older than this is not a reading (the module judges it with this)
};
static inline hb_cfg hb_cfg_default()
{
    hb_cfg c;
    c.min_kmh = 40;         // the player's figure: below it a town or a junction (policy; ini hb_min_kmh)
    c.hyst_kmh = 3;         // policy: a speed that wanders by a few km/h round the gate must not flicker the beam
    c.oncoming_m = 400;     // a real assist dims at 400 m; the probe sees 130 m, which is what applies
    c.lead_m = 200;
    c.delay_s = 3;          // short: the beam should not flicker with a gap in a column, but a free road should light up soon (policy; ini hb_delay_s)
    c.dark_from_h = 20;     // wider than the real dusk (summer 22 h, winter 16 h): the low-beam condition does the rest; keys in the ini
    c.dark_to_h = 6;
    c.driver_hold_s = 30;   // policy, the same hands-off time as the cab assist's
    c.lead_half_w = 6;      // from our middle to the far edge of the next lane: half a lane (1.8 m) + a lane (3.7 m) = 5.5 m, rounded up to 6
    c.list_age_s = 0.6f;    // three walks of the traffic list (traffic.cpp TICK_MS 200 ms): one missed or late walk still counts, two do not
    return c;
}

struct hb_in {
    double t;
    int ok;                  // engine on, not paused
    int probe_on;            // [traffic] probe = 1
    int traffic_ok;          // the list was read in the last second
    int low_on, high_on;     // the beam state the decision starts from (the real one when acting, our own belief when only logging)
    long long game_min;      // game clock in minutes, -1 = unknown
    float kmh;
    float yaw_rps;           // yaw rate in rotations per second (+ = left)
    int indicator;
    int driver_changed;      // the driver (not us) changed the beam since the last call
    int n;
    hb_veh v[32];
};

// The term that decided this call, or, when nothing is decided, the one that holds the decision back. The blocking terms (HBT_BLOCK_*) are
// the reason for a LOW decision and the reason a high beam is not brought up.
enum hb_term {
    HBT_NONE = 0,
    HBT_NOT_DRIVING,         // engine off or paused
    HBT_PROBE_OFF,           // [traffic] probe is 0: nothing can be seen
    HBT_NOT_DARK_DOWN,       // LOW: it is not dark any more and the beam is one we put up
    HBT_NOT_DARK,            // not dark (low beam off, or the clock is outside the dark window): nothing to do
    HBT_BLOCK_NO_LIST,       // the traffic list was not read in list_age_s
    HBT_BLOCK_INDICATOR,
    HBT_BLOCK_SPEED,         // under the speed gate (with its hysteresis)
    HBT_BLOCK_BEND,          // a bend tighter than the probe can see through
    HBT_BLOCK_ONCOMING,
    HBT_BLOCK_AHEAD,         // a vehicle ahead in our direction
    HBT_HELD_FOR_DRIVER,     // blocked, but the driver changed the beam less than driver_hold_s ago: left alone
    HBT_DRIVER_HOLD,         // clear, but the driver changed the beam less than driver_hold_s ago
    HBT_HIGH_CLEAR,          // the beam is up and the road is clear
    HBT_CLEARING,            // clear, counting delay_s
    HBT_HIGH,                // HIGH: dark, fast enough, straight, clear for delay_s
    HBT_N
};
static inline const char *hb_term_name(int t)
{
    static const char *const n[HBT_N] = { "none", "not_driving", "probe_off", "not_dark_down", "not_dark", "no_list", "indicator", "speed", "bend", "oncoming", "ahead",
                                          "held_for_driver", "driver_hold", "high_clear", "clearing", "high" };
    return t >= 0 && t < HBT_N ? n[t] : "?";
}

struct hb_out {
    int want;                // -1 nothing to do, 0 low, 1 high
    int term;                // hb_term: what decided (want set) or what holds it back (want -1)
    char why[160];
    float oncoming_m, lead_m;   // nearest oncoming / same-direction vehicle ahead (m), 1e9 = none
    float bend_r;               // 1e9 = straight
};

struct hb_state {
    double clear_since, driver_until;
    int ours_on;             // we put the beam up and it has not come down since
};

static inline void hb_reset(hb_state *s) { memset(s, 0, sizeof *s); s->clear_since = -1e9; s->driver_until = -1e9; }

static inline int hb_dark_by_clock(long long game_min, const hb_cfg &c)
{
    if (game_min < 0) return 1;                 // no clock: the low beam alone decides
    double h = (double)(game_min % 1440) / 60.0;
    return c.dark_from_h > c.dark_to_h ? (h >= c.dark_from_h || h < c.dark_to_h) : (h >= c.dark_from_h && h < c.dark_to_h);
}

static inline void hb_step(hb_state *s, const hb_in &in, const hb_cfg &c, hb_out *o)
{
    o->want = -1; o->why[0] = 0; o->term = HBT_NONE; o->oncoming_m = o->lead_m = o->bend_r = 1e9f;
    double t = in.t;
    size_t n = sizeof o->why;
    if (in.driver_changed) s->driver_until = t + c.driver_hold_s;
    if (!in.ok) { s->clear_since = -1e9; o->term = HBT_NOT_DRIVING; snprintf(o->why, n, "not driving"); return; }
    if (!in.probe_on) { s->clear_since = -1e9; o->term = HBT_PROBE_OFF; snprintf(o->why, n, "traffic probe is off"); return; }
    int armed = (in.low_on || in.high_on) && hb_dark_by_clock(in.game_min, c);
    if (!armed) {
        s->clear_since = -1e9;
        char dark[100];
        if (in.game_min < 0) snprintf(dark, sizeof dark, "clock unknown");
        else snprintf(dark, sizeof dark, "clock %02lld:%02lld, dark %.0f-%.0f h", (in.game_min % 1440) / 60, in.game_min % 60, c.dark_from_h, c.dark_to_h);
        if (s->ours_on && in.high_on) { o->want = 0; o->term = HBT_NOT_DARK_DOWN; s->ours_on = 0; snprintf(o->why, n, "not dark any more (low beam %s, %s)", in.low_on ? "on" : "off", dark); }
        else { o->term = HBT_NOT_DARK; snprintf(o->why, n, "not dark (low beam %s, %s)", in.low_on ? "on" : "off", dark); }
        return;
    }

    // the blockers, nearest cause first for the log
    const char *block = NULL;
    int bterm = HBT_NONE;
    char what[100] = "";
    float onc = 1e9f, lead = 1e9f;
    float range_onc = c.oncoming_m < HB_PROBE_RANGE_M ? c.oncoming_m : HB_PROBE_RANGE_M;
    float range_lead = c.lead_m < HB_PROBE_RANGE_M ? c.lead_m : HB_PROBE_RANGE_M;
    for (int i = 0; i < in.n; i++) {
        const hb_veh &v = in.v[i];
        if (!(v.dx > 0)) continue;
        float ah = fabsf(v.hdg);
        // headings: more than 120 degrees off ours = oncoming, less than 60 = our direction (60 degrees each side of the line of travel, cos 60 = 0.5);
        // the sector between is crossing traffic, which neither dims nor blocks. Geometry, not a tunable.
        if (ah > 120 && fabsf(v.dy) < HB_PROBE_SIDE_M && v.dist < onc) onc = v.dist;
        if (ah < 60 && fabsf(v.dy) < c.lead_half_w && v.dist < lead) lead = v.dist;
    }
    o->oncoming_m = onc; o->lead_m = lead;
    double w = in.yaw_rps * 6.2831853;   // rotations per second to rad/s
    // |w| > 1e-3 rad/s (R over 22 km at 80 km/h): the noise floor of the yaw rate; the bend limit below (R 338-406 m) is 25 times that rate at 40 km/h
    float r = fabs(w) > 1e-3 && in.kmh > 5 ? (float)((in.kmh / 3.6) / fabs(w)) : 1e9f;
    o->bend_r = r;
    float need_kmh = c.min_kmh + (in.high_on ? -c.hyst_kmh : c.hyst_kmh);
    float bend_lim = HB_BEND_R_MIN_M * (in.high_on ? 1.0f : 1.2f);   // coming up again needs a straighter road than going down does (20 % hysteresis, policy)
    if (!in.traffic_ok) { bterm = HBT_BLOCK_NO_LIST; snprintf(what, sizeof what, "traffic list not read (no reading in the last %.1f s)", c.list_age_s); block = what; }
    else if (in.indicator) { bterm = HBT_BLOCK_INDICATOR; snprintf(what, sizeof what, "indicator on"); block = what; }
    else if (in.kmh < need_kmh) { bterm = HBT_BLOCK_SPEED; snprintf(what, sizeof what, "%.0f km/h under %.0f", in.kmh, need_kmh); block = what; }
    else if (r < bend_lim) { bterm = HBT_BLOCK_BEND; snprintf(what, sizeof what, "bend R %.0f m under %.0f m", r, bend_lim); block = what; }
    else if (onc <= range_onc) { bterm = HBT_BLOCK_ONCOMING; snprintf(what, sizeof what, "oncoming vehicle at %.0f m (within %.0f)", onc, range_onc); block = what; }
    else if (lead <= range_lead) { bterm = HBT_BLOCK_AHEAD; snprintf(what, sizeof what, "vehicle ahead at %.0f m (within %.0f)", lead, range_lead); block = what; }

    if (block) {
        s->clear_since = -1e9;
        if (in.high_on && t >= s->driver_until) { o->want = 0; o->term = bterm; s->ours_on = 0; snprintf(o->why, n, "%s", block); }
        else if (in.high_on) { o->term = HBT_HELD_FOR_DRIVER; snprintf(o->why, n, "%s, but the driver changed the beam %.0f s ago (hands off for %.0f s)", block, c.driver_hold_s - (float)(s->driver_until - t), c.driver_hold_s); }
        else { o->term = bterm; snprintf(o->why, n, "%s", block); }
        return;
    }
    if (s->clear_since < -1e8) s->clear_since = t;
    if (t < s->driver_until) { o->term = HBT_DRIVER_HOLD; snprintf(o->why, n, "the driver changed the beam %.0f s ago (hands off for %.0f s)", c.driver_hold_s - (float)(s->driver_until - t), c.driver_hold_s); return; }
    if (in.high_on) { o->term = HBT_HIGH_CLEAR; snprintf(o->why, n, "high beam on, road clear"); return; }
    if (t - s->clear_since < c.delay_s) { o->term = HBT_CLEARING; snprintf(o->why, n, "clear for %.1f s of %.1f", (float)(t - s->clear_since), c.delay_s); return; }
    o->want = 1; o->term = HBT_HIGH; s->ours_on = 1;
    snprintf(o->why, n, "dark, %.0f km/h (over %.0f), no vehicle within %.0f m ahead (oncoming) or %.0f m (same direction), clear for %.1f s (of %.1f)%s", in.kmh, need_kmh, range_onc,
             range_lead, (float)(t - s->clear_since), c.delay_s, in.low_on ? "" : " (low beam off)");
}
