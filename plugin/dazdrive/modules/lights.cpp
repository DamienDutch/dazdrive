// Lights: our own high-beam assist (the rule and its thresholds: core/hblogic.h). The game has one, and it is not good enough.
//
// High beam on when it is dark (low beam on, clock outside the daytime), above ~40 km/h, in no tight bend, the indicator off and nothing in
// front of us for a few seconds; off AT ONCE when an oncoming vehicle or one ahead in our direction is in the traffic list, the road bends
// tight, we slow down or the list cannot be read. It needs the traffic probe ([traffic] probe=1: modules/traffic.cpp), which sees vehicles up to
// 130 m ahead and 25 m to the sides: shorter than a real assist's 400 m, and the log says so.
//
// LOG ONLY by default ([lights] act 0): the module keeps its own belief of the beam, decides on that and writes "would switch ON / OFF" lines
// with the reason and the beam the game really has (to compare with the game's own assist or the driver on a ride). With act 1 it presses the
// game's hblight through the plugin's input device ([core] input_device) and checks in the telemetry that the beam followed (core/toggle.h).
// When the driver changes the beam himself it keeps its hands off for 30 s.
//
// The log (promise P7): a line for every decision, "would switch the high beam ON / OFF [term]: value against threshold" (core/hblogic.h names
// the terms), a line when an acting decision ends (the term that ended it) and, at log level 1, a line when the term that holds a decision
// back changes (at most one per 10 s).
#include "../core/suite.h"
#include "../core/traffic.h"
#include "../core/hblogic.h"
#include "../core/toggle.h"
#include <math.h>
#include <string.h>

static float p_assist, p_act, p_min, p_onc, p_lead, p_delay, p_from, p_to, p_hold;

static param g_params[] = {
    { "hb_assist", &p_assist, 1, "1 = our own high-beam assist: high beam on when it is dark (low beam on, game clock outside daytime), above hb_min_kmh, no tight bend, nothing ahead for hb_delay_s; off at once for an oncoming vehicle, one ahead, a bend, a low speed or the indicator. Needs [traffic] probe=1 (it sees 130 m ahead). 0 = off" },
    { "act", &p_act, 0, "0 = log only: 'would switch on / off' lines with the reason and the beam the game really has, nothing is pressed. 1 = press the game's high beam control through the plugin's input device ([core] input_device) when the decision changes, and check that it took" },
    { "hb_min_kmh", &p_min, 40, "high beam only above this speed (km/h), with 3 km/h of hysteresis" },
    { "hb_oncoming_m", &p_onc, 400, "an oncoming vehicle within this distance (m) puts the beam low. The traffic list reaches 130 m, so that is the most that applies" },
    { "hb_lead_m", &p_lead, 200, "a vehicle ahead in our direction within this distance (m) puts the beam low (130 m at most, as above)" },
    { "hb_delay_s", &p_delay, 3, "the road must have been clear this long (s) before the beam goes up again" },
    { "hb_dark_from_h", &p_from, 20, "the game clock (hours) from which it can be dark enough for the high beam; the low beam must be on as well" },
    { "hb_dark_to_h", &p_to, 6, "the game clock (hours) until which it can be dark enough (earlier than 'from' = over midnight)" },
    { "hb_hold_s", &p_hold, 30, "after you switch the high beam yourself the assist leaves it alone this long (s)" },
};

static hb_state g_st;
static toggle_ctl g_tc;
static int g_virt = -1;          // our belief of the beam while only logging (-1 = follow the game)
static int g_prev_real = -1, g_pend = -1, g_last_armed;
static int g_last_want = -1, g_last_term;     // the last decision / holding term a line was written for (a line per change, not per frame)
static double g_last_term_at;
static int g_probe_said, g_chan_said, g_nodev_said;
static double g_last_status, g_driving_since;
static unsigned g_n_on, g_n_off, g_n_same;   // decisions; and, while logging, how often the game's beam already matched ours
static char g_status[120] = "off";

static void li_frame()
{
    if (p_assist <= 0) return;
    double now = wall_s();
    if (!T.engine_on || T.paused) { g_driving_since = 0; return; }
    if (!g_driving_since) g_driving_since = now;   // (the 20 s below: the telemetry channels of the lights are there within a few seconds of driving; 20 s is a wide margin)
    if (!traffic_probe_on()) {
        if (!g_probe_said) { logline("Lights: the high-beam assist needs [traffic] probe=1 and that is off: doing nothing"); g_probe_said = 1; }
        return;
    }
    unsigned seen = telemetry_seen();
    if ((seen & 12) != 12) {
        if (!g_chan_said && now - g_driving_since > 20) {
            logline("Lights: no low/high beam channel from the game after 20 s of driving (seen mask %u): doing nothing", seen);
            g_chan_said = 1;
        }
        return;
    }

    int real = T.light_high;
    int changed = g_prev_real >= 0 && real != g_prev_real;
    int ours = changed && toggle_ours(&g_tc, now);
    if (changed) logv(1, "Lights: the game's high beam is now %s%s", real ? "on" : "off", ours ? " (after our press)" : "");
    g_prev_real = real;

    hb_in in;
    memset(&in, 0, sizeof in);
    in.t = now;
    in.ok = 1;
    in.probe_on = 1;
    in.low_on = T.light_low;
    in.game_min = (seen & 16) ? T.game_min : -1;
    in.kmh = fabsf(T.speed) * 3.6f;
    in.yaw_rps = T.yaw;
    in.indicator = T.blink_l || T.blink_r;
    int armed_now = (T.light_low || T.light_high);
    if (p_act > 0) { in.high_on = real; in.driver_changed = changed && !ours; }
    else {
        if (g_virt < 0 || (!g_last_armed && armed_now)) g_virt = real;   // start from the game's beam when the dark begins
        in.high_on = g_virt;
    }
    g_last_armed = armed_now;

    hb_cfg c = hb_cfg_default();
    c.min_kmh = p_min; c.oncoming_m = p_onc; c.lead_m = p_lead; c.delay_s = p_delay; c.dark_from_h = p_from; c.dark_to_h = p_to; c.driver_hold_s = p_hold;
    if (armed_now) {
        traffic_around ta;
        traffic_around_get(&ta);
        in.traffic_ok = ta.ok && now - ta.read_t < c.list_age_s;   // a reading older than this is not a reading (hblogic.h: 3 walks of the list)
        if (ta.ok) {
            for (int i = 0; i < ta.n_ahead && in.n < 32; i++) {   // (32 = hb_in::v; the list holds 16)
                const traffic_obj &o = ta.ahead[i];
                if (o.hdg != o.hdg) continue;
                in.v[in.n].dx = o.dx; in.v[in.n].dy = o.dy; in.v[in.n].dist = o.dist; in.v[in.n].hdg = o.hdg;
                in.n++;
            }
        }
    }

    hb_out o;
    hb_step(&g_st, in, c, &o);

    if (o.want >= 0 && o.want == g_last_want && p_act > 0) {
        // the same decision as before: no new line and no new count (the press and its check are toggle.h's below)
    } else if (o.want >= 0) {
        if (p_act > 0) {
            if (o.want != real) { logline("Lights: %s the high beam [%s]: %s", o.want ? "switching ON" : "switching OFF", hb_term_name(o.term), o.why); }
            if (o.want) g_n_on++; else g_n_off++;
        } else {
            logline("Lights: would switch the high beam %s [%s]: %s (the game's beam is %s%s)", o.want ? "ON" : "OFF", hb_term_name(o.term), o.why, real ? "on" : "off",
                    real == o.want ? ", already" : "");
            if (o.want) g_n_on++; else g_n_off++;
            if (real == o.want) g_n_same++;
            g_virt = o.want;
        }
        g_last_term = o.term; g_last_term_at = now;
    } else if (p_act > 0 && g_last_want >= 0) {
        logline("Lights: the decision to switch the high beam %s is over [%s]: %s", g_last_want ? "ON" : "OFF", hb_term_name(o.term), o.why);
        g_last_term = o.term; g_last_term_at = now;
    } else if (o.term != HBT_NONE && o.term != g_last_term && now - g_last_term_at >= 10) {
        logv(1, "Lights: the high beam holds [%s]: %s", hb_term_name(o.term), o.why);
        g_last_term = o.term; g_last_term_at = now;
    }
    g_last_want = o.want;
    if (p_act > 0) {
        if (o.want >= 0) g_pend = o.want;
        else if (g_tc.target < 0) g_pend = -1;
        int ev = 0;
        int press = toggle_step(&g_tc, now, real, g_pend, &ev);
        if (ev == 1) logv(1, "Lights: the high beam followed our press");
        if (ev == -1) { logline("Lights: the high beam did not follow %d presses of hblight: not trying again for %.0f s", (int)TOGGLE_MAX_TRIES, TOGGLE_BACKOFF_S); g_pend = -1; }
        if (press && !input_press("hblight")) {
            if (!g_nodev_said) { logline("Lights: cannot press hblight: input device %s", input_device_state()); g_nodev_said = 1; }
            toggle_clear(&g_tc);
            g_tc.backoff_until = now + TOGGLE_BACKOFF_S;
            g_pend = -1;
        }
    }
    if (armed_now && now - g_last_status > 10) {   // (a periodic trace, not a threshold)
        g_last_status = now;
        logv(2, "Lights: dark, %.0f km/h, game beam %s%s, oncoming %.0f m, ahead %.0f m, bend R %.0f m, traffic %s (%d vehicles) [%s]: %s", in.kmh, real ? "high" : "low",
             p_act > 0 ? "" : (g_virt ? " (ours: high)" : " (ours: low)"), o.oncoming_m < 1e8f ? o.oncoming_m : -1.f, o.lead_m < 1e8f ? o.lead_m : -1.f,
             o.bend_r < 1e8f ? o.bend_r : -1.f, in.traffic_ok ? "read" : "NOT read", in.n, hb_term_name(o.term), o.why);
    }
}

static void li_reset(int level, const char *)
{
    hb_reset(&g_st);
    toggle_clear(&g_tc);
    g_virt = -1; g_prev_real = -1; g_pend = -1; g_last_armed = 0; g_driving_since = 0; g_last_want = -1; g_last_term = 0;
    (void)level;
}

static void li_start()
{
    hb_reset(&g_st);
    toggle_clear(&g_tc);
    if (p_assist <= 0) { snprintf(g_status, sizeof g_status, "off (ini)"); return; }
    snprintf(g_status, sizeof g_status, "%s; traffic probe %s, it sees %.0f m ahead", p_act > 0 ? "acting" : "log only",
             traffic_probe_on() ? "on" : "OFF (nothing happens)", HB_PROBE_RANGE_M);
}

static const char *li_status() { return g_status; }

static void li_stop()
{
    if (p_assist > 0 && (g_n_on || g_n_off))
        logline("Lights: %u ON and %u OFF decisions this session%s; presses ok %u, failed %u", g_n_on, g_n_off,
                p_act > 0 ? "" : " (log only)", g_tc.n_ok, g_tc.n_fail);
    (void)g_n_same;
}

module m_lights = { "lights", "Lights", g_params, sizeof g_params / sizeof g_params[0], li_start, li_frame, NULL, li_stop, li_reset, li_status, NULL, VCM_BOTH };
