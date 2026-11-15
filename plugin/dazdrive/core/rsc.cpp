// Rollover guard (Damien 25-10, design by an independent review of the 24-10 rollover): a second, independent check next to the lane
// guard that also works when you drive by hand. It does not use a real-truck stability threshold (those are lower than what the
// game's trucks survive), it looks for the growing sideways swing that preceded the 24-10 rollover: two opposite lateral peaks
// within 2.5 s. The lateral acceleration is speed x yaw rate. Levels: 1 = a tone and no throttle, 2 = also a soft brake,
// 3 = a firm brake when the swing grows. This file only decides: the brake wish and the throttle cap go to the actuator
// (core/actuate.cpp, owner ACT_RSC; the brake rides on the AEB's fast request path there), which calls rsc_step every tick.
// The single-bend thresholds (one bend taken too fast, no swing) sit behind rollover_bend, off by default: every physics mod
// has its own tipping point, so those limits never calibrate for everyone (Damien 25-10). The swing check does not need one.
// Every threshold below is a share of the sideways acceleration at which the Renault of the 24-10 rollover reaches the edge (6.27 m/s2, ROLL_GUARD_REF in
// core/roll.h) times gk = g_roll_guard_k, the scale the roll learning publishes (WP-LA4): 1 for a vehicle nothing is learned of and for every stiffer one, down to
// 0.75 for a vehicle that leans more per m/s2 (a tall or soft load): a vehicle that reaches its edge sooner is warned and braked sooner, never later than today.
#include "suite.h"
#include "vehicle.h"
#include "vclass.h"
#include "roll.h"
#include <math.h>
#include <stdio.h>

float g_rsc_mode = 3, g_rsc_bend = 0;
volatile int g_rsc_level;
static float s_brake, s_cap = 1;   // the brake wish (0..0.35) and the throttle cap (1 = none), both slewed here

struct lobe { int s; float peak; double t; };

static float clampf_(float v, float a, float b) { return v < a ? a : v > b ? b : v; }

void rsc_step(float dt)
{
    // A light vehicle (a car, a pickup) does not roll like a laden truck: the rollover guard stays off (its thresholds are truck
    // physics). Everything here is truck physics.
    if (!vprof()->rollover_guard) {
        g_rsc_level = 0; s_brake = 0; s_cap = 1;
        act_brake(ACT_RSC, 0); act_thr_cap(ACT_RSC, 1);
        return;
    }
    static float lp, lps;
    static lobe cur, prev, prev2;
    static int level;
    static double lvl_at, l2_at, calm_since, high_since, warn_since, warn_at;
    double now = wall_s();
    float v = fabsf(T.speed), kmh = v * 3.6f;
    // The scale of the thresholds (a sanity clamp: it is written by another thread, and only ever 0.75 .. 1)
    const float gk = g_roll_guard_k >= ROLL_GUARD_KMIN && g_roll_guard_k <= 1.0f ? (float)g_roll_guard_k : 1.0f;
    // Log for calibrating the single-bend threshold (Damien 25-10): 10 rows per second above 10 km/h, also by hand, only at log level 2
    {
        static FILE *csv; static double csv_at, flushed; static int tried;
        if (!tried && g_log_level >= 2 && g_rsc_mode > 0.5f) { tried = 1; csv = open_out("rsc.csv", "w"); if (csv) fputs("t,kmh,a_lat,lat_lp,lat_slow,yaw_dps,roll_deg,pitch_deg,steer,gear,level,trailer_roll_deg,roll_gain,trailer_roll_gain,roll_allow,guard_k\n", csv); }
        if (csv && !T.paused && kmh > 10 && now - csv_at >= 0.1) {
            csv_at = now;
            // the lean learning (core/rolllearn.cpp): the trailer's roll, the gains (deg per m/s2) and the allowance the bends are planned with
            // now (m/s2; empty while nothing is learned)
            bool tr = T.trailer && T.trailer_place_t > 0 && now - T.trailer_place_t < 0.5;
            char tro[16] = "", g0[16] = "", g1[16] = "", al[16] = "";
            if (tr) snprintf(tro, sizeof tro, "%.2f", (float)T.trailer_place.orientation.roll * 360.0f);
            if (roll_gain(0) == roll_gain(0)) snprintf(g0, sizeof g0, "%.3f", roll_gain(0));
            if (roll_gain(1) == roll_gain(1)) snprintf(g1, sizeof g1, "%.3f", roll_gain(1));
            if (roll_allow_now() == roll_allow_now()) snprintf(al, sizeof al, "%.2f", roll_allow_now());
            fprintf(csv, "%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%d,%d,%s,%s,%s,%s,%.3f\n", now, kmh, v * T.yaw * 6.2831853f, lp, lps, T.yaw * 360.0f,
                    (float)T.place.orientation.roll * 360.0f, (float)T.place.orientation.pitch * 360.0f, T.in_steer, T.gear, (int)g_rsc_level, tro, g0, g1, al, (float)g_roll_guard_k);
        }
        if (csv && now - flushed > 2) { flushed = now; fflush(csv); csv_cap(&csv, "rsc.csv"); }   // the same size cap as the other traces
    }
    // not forward (reverse), paused, a hitch, too slow: nothing to watch. Neutral counts (an eco-roll at 85 km/h can sway like any gear)
    if (g_rsc_mode < 0.5f || T.paused || !(dt > 0 && dt < 0.5f) || T.gear < 0 || kmh < 40) {
        level = 0; cur = prev = prev2 = lobe(); lp = lps = 0; calm_since = high_since = warn_since = 0;
        g_rsc_level = 0;
        if (g_rsc_mode >= 0.5f && !T.paused && dt > 0 && dt < 0.5f && T.gear >= 0) {   // only the speed fell under 40: the brake and the cap let go at their own pace,
            s_brake = fmaxf(0.0f, s_brake - 0.6f * dt);                                // not in one frame (27-10: 0.35 -> 0 and the cap 0 -> 1 at 40.0 km/h)
            s_cap = fminf(1.0f, s_cap + 1.0f * dt);
        } else { s_brake = 0; s_cap = 1; }
        act_brake(ACT_RSC, s_brake); act_thr_cap(ACT_RSC, s_cap);
        return;
    }
    float a = v * T.yaw * 6.2831853f;
    lp += (a - lp) * fminf(1.0f, dt / 0.05f);
    lps += (a - lps) * fminf(1.0f, dt / 0.3f);
    // A lobe counts above 0.48 of the limit (3.0 m/s2): its ordinary driving (lane changes, roundabouts) stays under it (sim_cruise: lane changes 2.2 m/s2 = 0.35,
    // a roundabout at 40 km/h 3.0 held, level 0), the swings of 24-10 were far over it (7.0 / -8.7, 7.9 / -10.3 m/s2 = 1.1 .. 1.6)
    const float lobe_thr = 3.0f * gk;
    int s = lp > lobe_thr ? 1 : lp < -lobe_thr ? -1 : 0;
    if (s) {
        if (cur.s == s) { cur.peak = fmaxf(cur.peak, fabsf(lp)); cur.t = now; }
        else { prev2 = prev; prev = cur; cur.s = s; cur.peak = fabsf(lp); cur.t = now; }
    }
    int want = 0;
    bool two = cur.s && prev.s && now - prev.t < 2.5 && now - cur.t < 1.0;
    if (kmh >= 50 && two) {
        float hi = fmaxf(cur.peak, prev.peak), lo = fminf(cur.peak, prev.peak);
        // level 1 (a tone, no throttle): both lobes over 0.48 of the limit (3.0) and the larger over 0.64 (4.0);
        // level 2 (the soft brake): 0.56 (3.5) and 0.80 (5.0), the swing as the 24-10 one at its first pair;
        // level 3 (the firm brake): a third lobe within 4 s, the swing growing, all over 0.56 (3.5)
        if (lo >= 3.0f * gk && hi >= 4.0f * gk) want = 1;
        if (lo >= 3.5f * gk && hi >= 5.0f * gk) want = 2;
        if (want == 2 && prev2.s && now - prev2.t < 4.0 && cur.peak > prev.peak && prev.peak >= 3.5f * gk && prev2.peak >= 3.5f * gk) want = 3;
    }
    // one bend taken too fast, no swing: a guess from the research (real trucks roll at 2.5-4.9 m/s2, the game's trucks survived 7.7 and
    // held 5 for 3 s): 3.5 m/s2 held for 1 s = a tone only (a warning, like a real system), 4.5 held for 0.5 s = throttle off and a tone,
    // 6 = the soft brake, 8.5 = the firm one. Only with rollover_bend on (see the top of the file).
    // As shares of the limit: 3.5 = 0.56 (a tone), 4.5 = 0.72 (no throttle), 6.0 = 0.96 (the soft brake: the lean about at the budget), 8.5 = 1.36 (the firm
    // brake: past it, what the 24-10 swing reached). All times gk.
    if (g_rsc_bend > 0.5f) {
        if (fabsf(lps) >= 3.5f * gk && kmh >= 40) {
            if (!warn_since) warn_since = now;
            if (now - warn_since > 1.0 && now - warn_at > 4.0 && level == 0) { warn_at = now; play_cue(CUE_AEB_WARN); logline("Rollover guard: warning, lateral %.1f m/s2 at %.0f km/h", lps, kmh); }
        } else warn_since = 0;
        if (fabsf(lps) >= 4.5f * gk && kmh >= 40) { if (!high_since) high_since = now; if (now - high_since > 0.5) want = want > 1 ? want : 1; } else high_since = 0;
        if (fabsf(lps) >= 6.0f * gk) want = want > 2 ? want : 2;
        if (fabsf(lps) >= 8.5f * gk) want = 3;
    }
    if (want > level) {
        if (level == 0) {
            play_cue(CUE_AEB_WARN);
            incident("rsc", "rollover guard level %d: lateral %.1f / %.1f m/s2 (previous peak %.1f), %.0f km/h%s", want, cur.peak, lps, prev.peak, kmh, gk < 1.0f ? ", thresholds scaled" : "");
        }
        if (want >= 2 && level < 2) l2_at = now;
        level = want; lvl_at = now;
        logline("Rollover guard: level %d (lateral peaks %.1f and %.1f m/s2, %.0f km/h, thresholds x%.2f)", level, cur.peak, prev.peak, kmh, gk);
    }
    if (fabsf(lp) < 2.5f * gk) { if (!calm_since) calm_since = now; } else calm_since = 0;   // calm = under 0.40 of the limit (2.5), a lobe's floor less a margin
    if (level > 0 && calm_since && now - calm_since > 1.0 && now - lvl_at > 1.5) { level = 0; logline("Rollover guard: calm again, released"); }
    if (level >= 2 && now - l2_at > 4.0) level = 1;                       // the brake lasts at most 4 s
    g_rsc_level = level;
    float tgt = g_rsc_mode < 2.5f ? 0.0f : level >= 3 ? 0.35f : level == 2 ? 0.2f : 0.0f;
    float up = level >= 3 ? 2.0f : 1.0f, down = 0.6f;
    s_brake = clampf_(tgt, s_brake - down * dt, s_brake + up * dt);
    act_brake(ACT_RSC, s_brake);
    float cap_tgt = level >= 1 && g_rsc_mode >= 1.5f ? 0.0f : 1.0f;
    s_cap = clampf_(cap_tgt, s_cap - 4.0f * dt, s_cap + 1.0f * dt);
    act_thr_cap(ACT_RSC, s_cap);
}
