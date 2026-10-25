// Rollover guard (Damien 25-10, design by an independent review of the 24-10 rollover): a second, independent check next to the lane
// guard that also works when you drive by hand. It does not use a real-truck stability threshold (those are lower than what the
// game's trucks survive), it looks for the growing sideways swing that preceded the 24-10 rollover: two opposite lateral peaks
// within 2.5 s. The lateral acceleration is speed x yaw rate. Levels: 1 = a tone and no throttle, 2 = also a soft brake,
// 3 = a firm brake when the swing grows. Cruise applies g_rsc_thr_cap and g_rsc_brake; this file only decides.
#include "suite.h"
#include <math.h>
#include <stdio.h>

volatile int g_rsc_level;
volatile float g_rsc_brake, g_rsc_thr_cap = 1;

struct lobe { int s; float peak; double t; };

static float clampf_(float v, float a, float b) { return v < a ? a : v > b ? b : v; }

void rsc_step(float dt)
{
    static float lp, lps;
    static lobe cur, prev, prev2;
    static int level;
    static double lvl_at, l2_at, calm_since, high_since, warn_since, warn_at;
    double now = wall_s();
    float v = fabsf(T.speed), kmh = v * 3.6f;
    // Log for calibrating the single-bend threshold (Damien 25-10): 10 rows per second above 10 km/h, also by hand, only at log level 2
    {
        static FILE *csv; static double csv_at; static int tried;
        if (!tried && g_log_level >= 2) { tried = 1; csv = open_out("rsc.csv", "w"); if (csv) fputs("t,kmh,a_lat,lat_lp,lat_slow,yaw_dps,roll_deg,pitch_deg,steer,gear,level\n", csv); }
        if (csv && !T.paused && kmh > 10 && now - csv_at >= 0.1) {
            csv_at = now;
            fprintf(csv, "%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%d,%d\n", now, kmh, v * T.yaw * 6.2831853f, lp, lps, T.yaw * 360.0f,
                    (float)T.place.orientation.roll * 360.0f, (float)T.place.orientation.pitch * 360.0f, T.in_steer, T.gear, (int)g_rsc_level);
            fflush(csv);
        }
    }
    if (T.paused || !(dt > 0 && dt < 0.5f) || T.gear <= 0 || kmh < 40) {   // not forward, paused, a hitch, too slow: nothing to watch
        level = 0; cur = prev = prev2 = lobe(); lp = lps = 0; calm_since = high_since = warn_since = 0;
        g_rsc_level = 0; g_rsc_brake = 0; g_rsc_thr_cap = 1;
        return;
    }
    float a = v * T.yaw * 6.2831853f;
    lp += (a - lp) * fminf(1.0f, dt / 0.05f);
    lps += (a - lps) * fminf(1.0f, dt / 0.3f);
    int s = lp > 3.0f ? 1 : lp < -3.0f ? -1 : 0;
    if (s) {
        if (cur.s == s) { cur.peak = fmaxf(cur.peak, fabsf(lp)); cur.t = now; }
        else { prev2 = prev; prev = cur; cur.s = s; cur.peak = fabsf(lp); cur.t = now; }
    }
    int want = 0;
    bool two = cur.s && prev.s && now - prev.t < 2.5 && now - cur.t < 1.0;
    if (kmh >= 50 && two) {
        float hi = fmaxf(cur.peak, prev.peak), lo = fminf(cur.peak, prev.peak);
        if (lo >= 3.0f && hi >= 4.0f) want = 1;
        if (lo >= 3.5f && hi >= 5.0f) want = 2;
        if (want == 2 && prev2.s && now - prev2.t < 4.0 && cur.peak > prev.peak && prev.peak >= 3.5f && prev2.peak >= 3.5f) want = 3;
    }
    // one bend taken too fast, no swing: a guess from the research (real trucks roll at 2.5-4.9 m/s2, the game's trucks survived 7.7 and
    // held 5 for 3 s): 3.5 m/s2 held for 1 s = a tone only (a warning, like a real system), 4.5 held for 0.5 s = throttle off and a tone,
    // 6 = the soft brake, 8.5 = the firm one
    if (fabsf(lps) >= 3.5f && kmh >= 40) {
        if (!warn_since) warn_since = now;
        if (now - warn_since > 1.0 && now - warn_at > 4.0 && level == 0) { warn_at = now; play_cue(CUE_AEB_WARN); logline("Rollover guard: warning, lateral %.1f m/s2 at %.0f km/h", lps, kmh); }
    } else warn_since = 0;
    if (fabsf(lps) >= 4.5f && kmh >= 40) { if (!high_since) high_since = now; if (now - high_since > 0.5) want = want > 1 ? want : 1; } else high_since = 0;
    if (fabsf(lps) >= 6.0f) want = want > 2 ? want : 2;
    if (fabsf(lps) >= 8.5f) want = 3;
    if (want > level) {
        if (level == 0) {
            play_cue(CUE_AEB_WARN);
            incident("rsc", "rollover guard level %d: lateral %.1f / %.1f m/s2 (previous peak %.1f), %.0f km/h", want, cur.peak, lps, prev.peak, kmh);
        }
        if (want >= 2 && level < 2) l2_at = now;
        level = want; lvl_at = now;
        logline("Rollover guard: level %d (lateral peaks %.1f and %.1f m/s2, %.0f km/h)", level, cur.peak, prev.peak, kmh);
    }
    if (fabsf(lp) < 2.5f) { if (!calm_since) calm_since = now; } else calm_since = 0;
    if (level > 0 && calm_since && now - calm_since > 1.0 && now - lvl_at > 1.5) { level = 0; logline("Rollover guard: calm again, released"); }
    if (level >= 2 && now - l2_at > 4.0) level = 1;                       // the brake lasts at most 4 s
    g_rsc_level = level;
    float tgt = level >= 3 ? 0.35f : level == 2 ? 0.2f : 0.0f;
    float up = level >= 3 ? 2.0f : 1.0f, down = 0.6f;
    g_rsc_brake = clampf_(tgt, g_rsc_brake - down * dt, g_rsc_brake + up * dt);
    float cap_tgt = level >= 1 ? 0.0f : 1.0f;
    g_rsc_thr_cap = clampf_(cap_tgt, g_rsc_thr_cap - 4.0f * dt, g_rsc_thr_cap + 1.0f * dt);
}
