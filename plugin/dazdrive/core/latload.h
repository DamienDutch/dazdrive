// The lateral load of the truck, live (WP-LA5): how much sideways acceleration it has now, or by its steering is about to have, from the telemetry alone.
// No route, no junction, no cruise: with and without navigation the reading is the same. The gearbox's bend bonus reads the same thing (trueshift.cpp
// lateral_load, written first; it moves here when that module is free of the gearbox round), and the lane assist yields to the driver's hand under it (modules/lane.cpp).
//
// Two readings, the larger counts:
//   measured  the sideways acceleration, speed x yaw rate, low-passed at the body's response (0.25 s, core/rolllearn.cpp RL_TAU_A): the confirmation.
//   predicted what the steering will give: a bend is entered with the wheel before the acceleration builds, so the steering leads. The gain is a regression
//             through the origin of the measured acceleration on v^2 x steering, summed with a forgetting (the demand memory), and what is used is its lower
//             bound (the slope less its standard error, as roll_fit_eval judges a bend): with little evidence the steering predicts nothing and the measured
//             acceleration decides alone. A keyboard or pad tap sits at full lock and teaches nothing (steering over 0.95 is left out, and predicts nothing).
// Pure: the state is the caller's, the clock is passed in. tools/sim/sim_lane.cpp drives it.
#pragma once
#include <math.h>

struct lat_state {
    double t = 0;                                   // the time of the last step (the caller's clock, s)
    float af = 0;                                   // the measured sideways acceleration, filtered (m/s2, magnitude)
    float sxx = 0, sxy = 0, syy = 0, w = 0;         // the regression sums of measured on v^2 x steering, with forgetting
};
struct lat_read { float meas, lead, accel; };      // m/s2: the filtered measurement, what the steering predicts (0 = nothing), the larger of both

static const float LAT_TAU_S = 0.25f;               // s, the body's response to the steering (the same as RL_TAU_A in core/rolllearn.cpp, one independent sample per tau)
static const float LAT_MEM_S = 600.0f;              // s, how long a truck's steering -> sideways acceleration relation is taken to hold (DEMAND_MEM_S in core/demand.h: the same memory)
static const float LAT_LOCK = 0.95f;                // an analog wheel never sits above this; a digital steering key sits at 1

// yaw: rotations per second (the telemetry's unit), steer: -1..1, v: m/s. Call it every frame; a gap over 0.5 s (a pause, a hitch) starts the filter again
// and leaves the sums as they are.
inline lat_read lat_step(lat_state &S, float v, float yaw, float steer, double now)
{
    const float st = fabsf(steer), a_raw = fabsf(v * yaw * 6.2831853f), x = v * v * st;
    const double dt = now - S.t;
    S.t = now;
    if (!(dt > 0 && dt < 0.5)) S.af = a_raw;
    else {
        S.af += (a_raw - S.af) * fminf(1.0f, (float)dt / LAT_TAU_S);
        const float k = expf(-(float)dt / LAT_MEM_S);
        S.sxx *= k; S.sxy *= k; S.syy *= k; S.w *= k;
        if (st < LAT_LOCK) { const float w = (float)dt; S.sxx += w * x * x; S.sxy += w * x * a_raw; S.syy += w * a_raw * a_raw; S.w += w; }
    }
    lat_read r = { S.af, 0, S.af };
    if (st < LAT_LOCK && S.sxx > 1e-3f) {
        const float n_eff = S.w / LAT_TAU_S;
        const float slope = S.sxy / S.sxx;
        if (n_eff > 2.0f) {
            const float s2 = fmaxf(0.0f, S.syy - slope * S.sxy) / S.w * (n_eff / (n_eff - 1.0f));
            const float se = sqrtf(s2 / (n_eff * S.sxx / S.w));
            r.lead = fmaxf(0.0f, slope - se) * x;
            if (r.lead > r.accel) r.accel = r.lead;
        }
    }
    return r;
}

// The weight of an assist that has to yield to the driver's hand under a sideways load: whole up to edge_share of the limit, nothing at the limit. The limit is the
// sideways acceleration at which the lean reaches the budget (core/roll.h, ROLL_GUARD_REF x the guard's scale), edge_share the one the bends are planned at
// (g_roll_edge: the allowance is edge_share x limit, roll_target). Inside the allowance the cruise plans the bends and the assist steers them hands off; between the
// allowance and the limit there is less and less margin; at the limit one more m/s2 and the truck is over the edge. A linear fall between the two. edge_share is
// clamped (0.3 .. 0.95) so a wrong ini never gives a zero or negative width.
inline float lat_weight(float accel, float limit, float edge_share)
{
    const float e = fminf(0.95f, fmaxf(0.3f, edge_share));
    if (!(limit > 0.3f)) return 1.0f;
    const float w = 1.0f - (accel / limit - e) / (1.0f - e);
    return fminf(1.0f, fmaxf(0.0f, w));
}
