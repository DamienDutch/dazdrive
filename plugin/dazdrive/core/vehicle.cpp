// The vehicle model: mass, grade and engine, one estimate for every module. See vehicle.h.
#include "vehicle.h"
#include <math.h>
#include <string.h>

vehicle V = { NULL, 0, 0, -1e9, 0, 0, 0, {}, 0, 0, -1e9, 0 };

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

// Continuity broke (core/telemetry.cpp session_reset): trailer (de)coupled (LOAD: the mass); other truck, save/load,
// teleport (HARD: the engine too, read again at the next update).
void vehicle_reset(int level)
{
    double none = -1e9;
    if (level >= RESET_LOAD) V.mass_at = none;
    if (level >= RESET_HARD) {
        V.self = NULL;
        V.mass = 0; V.mass_ok = 0;
        V.eng_ok = 0;
        V.eng_at = none;
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
