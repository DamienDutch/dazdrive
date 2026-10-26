// The vehicle model: one estimate of the truck for every module (core/vehicle.cpp). The mass and the road grade come from
// the game's own functions (the ones its automatic uses), the engine from its definition. vehicle_update() reads what is due:
// core calls it every frame_end (after telemetry_frame), and TrueShift's hooks call it again inside the game's gearbox
// update, so a decision there sees this frame's grade and a fresh mass right after a reset (the mass and the engine have
// their own cadence, so the second call costs a grade function call). Modules read V; only the owner and vehicle_reset
// write it.
#pragma once
#include "game.h"

struct vehicle {
    void *self;          // the gearbox object last seen (another one = another truck: everything starts over)
    // mass
    float mass;          // kg, truck + trailers, the game's figure, read every 0.5 s; 0 until the first read, 20000 when the game cannot tell
    int mass_ok;         // 1 = the game has told (mass is its figure, not the 20 t fallback)
    double mass_at;
    // grade
    float grade_deg;     // the game's road grade for the gearbox (positive uphill), 0 when unknown; TrueShift decides on it
    float grade_pct;     // the same in % (tan)
    float pitch_pct;     // grade (%) from the SDK placement's pitch (positive = nose up); Cruise holds, coasts and launches on it
    // engine (every 5 s)
    engine_info eng;     // torque, curve, limit, ranges, shift time
    int eng_ok;          // 1 = eng looks sane
    int electric;        // the last read saw an electric motor (also when the rest of the data was not sane)
    double eng_at;
    float plateau;       // rpm where the torque curve reaches its plateau
};
extern vehicle V;

void vehicle_update(void *gearbox);  // read what is due (see above); gearbox = the game's gearbox object, NULL = none now (mass and pitch still)
void vehicle_frame();                // core: vehicle_update(game_gearbox())
void vehicle_reset(int level);       // RESET_* (suite.h): LOAD = the mass is read again at once; HARD = everything, the engine too
void vehicle_mass_stale();           // read the mass again at the next update (TrueShift: a start)
