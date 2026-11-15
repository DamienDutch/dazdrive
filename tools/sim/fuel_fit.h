// tools/sim/fuel_fit.h — the fitted ETS2 fuel model for the plant, ported from fuel_model.hpp in the worktree root
// (which is a copy and is NOT committed; its source notes point at notes/planner/fuel_model.hpp). Header-only, no dependencies.
//
// FORM: fuel/s = a * omega * Tpk * curve(rpm) * g(eff_thr), a concave g (0 below thr 0.02, ~0.14 + 0.86 thr above 0.25,
// tabulated). Zero on overrun in gear (thr < 0.02). Neutral: idle fuel only near idle rpm (<= idle + 60), zero while the
// engine spins down above idle. Per-engine scale 'a' ~ peak torque (mL per rad per Nm). The plant's default model
// (sim.cpp tick(): a x omega x (T_net + g_sw_tf x Tpk)) differs from this below thr ~0.15 (a constant friction share).
#pragma once
#include <math.h>

struct FuelEngine {
    float tpk;                 // Nm, peak torque
    float idle_rpm;            // rpm held in neutral
    int n;                     // curve points
    const float (*curve)[2];   // {rpm, share of peak torque}, ascending rpm (the engine's torque curve as the game reports it)
    float a;                   // mL per rad per Nm, high map zone
};

static const float FUEL_G_THR[] = {0.00f, 0.02f, 0.03f, 0.05f, 0.07f, 0.10f, 0.14f, 0.18f, 0.225f, 0.275f, 0.30f, 0.45f, 0.60f, 0.80f, 1.00f};
static const float FUEL_G_VAL[] = {0.000f,0.000f,0.043f,0.069f,0.100f,0.139f,0.182f,0.255f,0.308f,0.365f,0.384f,0.544f,0.691f,0.861f,1.000f};
static const int FUEL_G_N = 15;
static const float FUEL_IDLE_SHARE = 0.062f;   // idle burn per rad as a share of full-throttle burn per rad at curve = 1
static const float FUEL_ZONE_LOW = 1.0f / 6.17f;

static inline float fuel_interp(const float *x, const float *y, int n, float v)
{
    if (v <= x[0]) return y[0];
    for (int i = 1; i < n; i++)
        if (v <= x[i]) return y[i - 1] + (y[i] - y[i - 1]) * (v - x[i - 1]) / (x[i] - x[i - 1]);
    return y[n - 1];
}

static inline float fuel_curve(const FuelEngine &e, float rpm)
{
    if (rpm <= e.curve[0][0]) return e.curve[0][1];
    for (int i = 1; i < e.n; i++)
        if (rpm <= e.curve[i][0]) return e.curve[i - 1][1] + (e.curve[i][1] - e.curve[i - 1][1]) * (rpm - e.curve[i - 1][0]) / (e.curve[i][0] - e.curve[i - 1][0]);
    return e.curve[e.n - 1][1];
}

// Fuel flow in l/h. in_gear = clutch closed with a gear engaged; thr = effective throttle 0..1; zone_scale 1 = outside, ~0.162 = city.
static inline float fuel_rate_lph(const FuelEngine &e, float rpm, float thr, bool in_gear, float zone_scale = 1.0f)
{
    float w = rpm * 0.10472f, ml_s;                        // rad/s
    if (!in_gear) {                                        // neutral / clutch open: the idle governor burns only near idle rpm
        if (rpm > e.idle_rpm + 60.0f) return 0.0f;         // spinning down from higher rpm: nothing (measured)
        ml_s = e.a * e.tpk * FUEL_IDLE_SHARE * e.idle_rpm * 0.10472f;
    } else {
        if (thr < 0.02f) return 0.0f;                      // overrun fuel cut (measured, exactly zero)
        float g = fuel_interp(FUEL_G_THR, FUEL_G_VAL, FUEL_G_N, thr);
        ml_s = e.a * e.tpk * fuel_curve(e, rpm) * g * w;
    }
    return ml_s * 3.6f * zone_scale;                       // mL/s -> l/h (x3600 / 1000)
}

// The per-Nm scale 'a' for the sim's engines: Renault 6.83e-4 and Scania 7.35e-4 (measured), else the +-8 % guess 7.0e-4.
static inline float fuel_fit_a(float tpk)
{
    if (tpk >= 3600.0f) return 7.35e-4f;                    // Scania
    if (tpk >= 2600.0f && tpk < 2800.0f) return 6.83e-4f;   // Renault
    return 7.0e-4f;                                          // DAF / weak (guessed)
}
