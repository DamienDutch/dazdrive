// The vehicle model: mass, grade, engine and the forces, one estimate for every module. See vehicle.h.
#include "vehicle.h"
#include <math.h>
#include <string.h>

vehicle V = { NULL, 0, 0, -1e9, 0, 0, 0, {}, 0, 0, -1e9, 0, 0.006f, 6.0f, 0.9f, 1, 0 };

// Start of the torque plateau: first curve point within 3% of the peak.
static float find_plateau(const engine_info *e)
{
    float peak = 0;
    for (int i = 0; i < e->n_curve; i++) peak = fmaxf(peak, e->curve[i][1]);
    for (int i = 0; i < e->n_curve; i++)
        if (e->curve[i][1] >= 0.97f * peak && e->curve[i][0] > 300) return e->curve[i][0];
    return e->range[R_HIGH][0];
}

void vehicle_mass_stale() { V.mass_at = -1e9; }

// Continuity broke (core/telemetry.cpp session_reset): trailer (de)coupled (LOAD: the mass, and what was learned against
// it); other truck, save/load, teleport (HARD: the engine too, read again at the next update, and the rpm correction).
void vehicle_reset(int level)
{
    double none = -1e9;
    if (level >= RESET_LOAD) {
        V.mass_at = none;
        V.f_bias = 0;
    }
    if (level >= RESET_HARD) {
        V.self = NULL;
        V.mass = 0; V.mass_ok = 0;
        V.eng_ok = 0;
        V.eng_at = none;
        V.rpm_k = 1;
    }
}

void vehicle_update(void *gb)
{
    double now = sim_s();
    if (gb && gb != V.self) {
        if (V.self) vehicle_reset(RESET_HARD);   // another gearbox object: another truck
        V.self = gb;
    }
    if (now - V.mass_at > 0.5 || now < V.mass_at) {
        float m = game_mass();
        if (m == m) {
            if (!(V.mass > 0) || fabsf(m - V.mass) > 0.03f * V.mass)
                logv(1, "mass %.1f t (trailer %s, cargo %.1f t)", m / 1000, T.trailer ? "yes" : "no", T.cargo_mass / 1000);
            V.mass = m;
            V.mass_ok = 1;
        } else if (!(V.mass > 0)) V.mass = 20000;
        V.mass_at = now;
    }
    if (gb && (!V.eng_ok || now - V.eng_at > 5.0 || now < V.eng_at)) {
        engine_info e;
        memset(&e, 0, sizeof e);
        int ok = gb_engine(gb, &e);
        V.electric = e.electric;
        if (ok && (!V.eng_ok || e.torque != V.eng.torque || e.rpm_limit != V.eng.rpm_limit)) V.f_bias = 0;  // another engine: what was learned against the old one is void
        if (ok) { V.eng = e; V.plateau = find_plateau(&e); }
        V.eng_ok = ok;
        V.eng_at = now;
    }
    if (gb) {
        float gd = game_grade_deg(gb);
        V.grade_deg = gd == gd ? gd : 0;
        V.grade_pct = tanf(V.grade_deg * 0.0174533f) * 100;
    }
    // Grade (%) from the SDK placement: pitch in turns, positive = nose up.
    V.pitch_pct = tanf((float)T.place.orientation.pitch * 6.2831853f) * 100;
}

void vehicle_frame() { vehicle_update(game_gearbox()); }

// ---------------------------------------------------------------- gear, speed, forces

static int n_gears()
{
    int n = T.fwd_gears;
    return n > 0 && n < 32 ? n : 0;
}

// Engine rpm in gear g at speed v (m/s), from the driveline, corrected by the
// measured ratio to the real rpm.
float vehicle_rpm_at(int g, float v)
{
    float r = wheel_radius();
    if (g < 1 || g > n_gears() || r <= 0.1f || T.diff <= 0) return 0;
    return v / r * T.ratio[g - 1] * T.diff * 9.5493f * V.rpm_k;
}

float vehicle_kmh_at(int g, float rpm)
{
    float r = wheel_radius();
    if (g < 1 || g > n_gears() || r <= 0.1f || T.diff <= 0 || !(T.ratio[g - 1] > 0) || !(V.rpm_k > 0)) return 0;
    return rpm / (T.ratio[g - 1] * T.diff * 9.5493f * V.rpm_k) * r * 3.6f;
}

float vehicle_kmh_idle(int g, float idle_rpm, float margin_rpm) { return vehicle_kmh_at(g, idle_rpm + margin_rpm); }

// Tractive force (N) in gear g at speed v with a share of the available torque.
float vehicle_force_at(int g, float v, float share)
{
    float r = wheel_radius();
    if (g < 1 || g > n_gears() || r <= 0.1f) return 0;
    float rpm = vehicle_rpm_at(g, v);
    float t = V.eng.torque * curve_factor(&V.eng, rpm) * share;
    if (rpm > V.eng.rpm_limit) t = 0;
    return t * T.ratio[g - 1] * T.diff * V.eta / r;
}

// Air drag area: the truck value from the ini, a car-like one for light vehicles.
static float cda() { return V.mass > 0 && V.mass < 4500 ? 1.0f : V.cda; }

float vehicle_resist(float v)
{
    float s = sinf(V.grade_deg * 0.0174533f);
    return V.mass * 9.81f * (s + V.crr) + 0.5f * 1.2f * cda() * v * v + V.f_bias;
}

float vehicle_resist_grade(float v, float pct)
{
    float s = sinf(atanf(pct / 100));
    return V.mass * 9.81f * (s + V.crr) + 0.5f * 1.2f * cda() * v * v + V.f_bias;
}

float vehicle_accel(int g, float v, float share, float pct)
{
    if (!(V.mass > 0)) return 0;
    return (vehicle_force_at(g, v, share) - (pct == pct ? vehicle_resist_grade(v, pct) : vehicle_resist(v))) / V.mass;
}
