// The road ahead ("vooruitblik", see ahead.h): the force it takes to hold the speed over the next seconds and how
// sure that is. It reads the route (core/route.h: route_ok(), route_grade(), route_ahead_m()), the vehicle model
// (core/vehicle.h: vehicle_resist_grade, V.mass via the model) and, when the planner drives, its view
// (g_planner_drive, core/suite.h). Pure read; the only state is a small cache so it recomputes at most every 0.25 s
// (the route itself refreshes on the same cadence).
#include "ahead.h"
#include "route.h"
#include "vehicle.h"
#include "suite.h"
#include <string.h>
#include <math.h>

// the grade is sampled this far apart (m), the same grid the route stores its heights on (route.cpp STEP)
static const float SAMPLE_M = 25.0f;
// the result cache: recompute at most every this many seconds
static const double CACHE_S = 0.25;

static ahead_view s_ahead;
static double s_at = -1e9;
static int s_use_route = 1;   // ahead_use_route: 0 = the view of a truck with no route, the route not read

void ahead_use_route(int on) { on = on ? 1 : 0; if (on != s_use_route) { s_use_route = on; s_at = -1e9; } }

void ahead_get(float v, float horizon_s, ahead_view *out)
{
    double now = wall_s();
    if (now - s_at < CACHE_S && now >= s_at) { *out = s_ahead; return; }
    s_at = now;

    ahead_view a;
    memset(&a, 0, sizeof a);
    a.horizon_s = horizon_s;
    a.resist_now_n = vehicle_resist(v);

    float wanted = fmaxf(0.0f, v) * horizon_s;   // the distance (m) the horizon covers
    if (!s_use_route || !route_ok() || !(wanted > SAMPLE_M)) {
        // no route (or the route switched off, ahead_use_route): only the grade under the truck is known, the worst and the mean are the now
        a.resist_worst_n = a.resist_mean_n = a.resist_now_n;
        a.conf = 0.0f;
        s_ahead = a;
        *out = a;
        return;
    }

    // sample the grade every SAMPLE_M over the horizon; the route reports NAN where it is unknown or ends
    float sum = 0.0f, worst = -1e30f;
    int n = 0;
    float covered = 0.0f;
    for (float d = 0.0f; d < wanted; d += SAMPLE_M) {
        float g = route_grade(d, d + SAMPLE_M);
        if (g != g) break;
        float r = vehicle_resist_grade(v, g);
        sum += r;
        if (r > worst) worst = r;
        if (n < AHEAD_MAX) { a.sample_s[n] = (d + SAMPLE_M / 2) / v; a.sample_resist_n[n] = r; a.n_samples = n + 1; }   // (wanted > SAMPLE_M: v > 0)
        n++;
        covered = d + SAMPLE_M;
    }

    if (n > 0) {
        a.resist_mean_n = sum / (float)n;
        a.resist_worst_n = worst;
        float conf = covered / wanted;
        if (conf > 1.0f) conf = 1.0f;
        // the planner drives: it has planned over the route, so a route that reaches the whole horizon is fully trusted
        if (g_planner_drive.on && route_ahead_m() >= wanted) conf = 1.0f;
        a.conf = conf;
    } else {
        a.resist_worst_n = a.resist_mean_n = a.resist_now_n;
        a.conf = 0.0f;
    }

    s_ahead = a;
    *out = a;
}

// The 30 s view of the powertrain layers (ahead.h): ahead_get at the speed of now, and the conf fade. The fade ran in TrueShift's gear_layers: the view
// is a property of the road, not of the gearbox, so it is kept here; same numbers, same moments (it moves when it is asked).
static ahead_view s_now;
static float s_conf_s;
static double s_conf_t = -1e9;

const ahead_view *ahead_now()
{
    ahead_get(fabsf(T.speed), 30.0f, &s_now);
    const double now = sim_s();
    const double dt = now - s_conf_t;
    if (dt < 0 || dt > 3.0) s_conf_s = s_now.conf;
    else {
        const float step = 0.25f * (float)dt;
        float d = s_now.conf - s_conf_s;
        s_conf_s += d < -step ? -step : d > step ? step : d;
    }
    s_conf_t = now;
    s_now.conf = s_conf_s;
    return &s_now;
}

void ahead_reset()
{
    s_at = -1e9;
    s_conf_t = -1e9;
}
