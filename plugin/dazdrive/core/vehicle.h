// The vehicle model: one estimate of the truck for every module (core/vehicle.cpp). The mass and the road grade come from
// the game's own functions (the ones its automatic uses), the engine from its definition; the forces (what a gear pulls,
// what the road asks) are ours. vehicle_update() reads what is due: core calls it every frame_end (after telemetry_frame),
// and TrueShift's hooks call it again inside the game's gearbox update, so a decision there sees this frame's grade and a
// fresh mass right after a reset (the mass and the engine have their own cadence, so the second call costs a grade function
// call). Modules read V and call the functions below; only the owner and vehicle_reset write it, except the two corrections
// TrueShift learns while it drives (rpm_k, f_bias).
#pragma once
#include "game.h"

// The one mass (kg) the model assumes while the game cannot tell. A module that needs a cautious answer then decides it
// by V.mass_ok (unknown counts as heavy, the drive estimate is not trusted, ...), not with a mass figure of its own.
static const float VEHICLE_MASS_FALLBACK = 20000;

struct vehicle {
    void *self;          // the gearbox object last seen (another one = another truck: everything starts over)
    // mass
    float mass;          // kg, truck + trailers, the game's figure, read every 0.5 s; 0 until the first read, VEHICLE_MASS_FALLBACK when the game cannot tell
    int mass_ok;         // 1 = the game has told (mass is its figure, not the fallback)
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
    // the resistance model; the ini keys live in [trueshift] (rolling_resistance, drag_area, efficiency)
    float crr;           // rolling resistance coefficient
    float cda;           // air drag Cd*A (m2) for trucks; vehicles under 4.5 t get 1.0 (cars, pickups)
    float eta;           // driveline efficiency
    // learned by TrueShift while the clutch is closed and the pull is steady
    float rpm_k;         // measured rpm / driveline rpm (tire size etc.), 1 = as the ratios say
    float f_bias;        // extra resistance (N) the model misses (tires, wind, a wrong mass); 0 again with another load or engine
    float peak_pw;       // the engine's peak power as torque factor x rpm (the highest point of the torque curve times rpm), set with the engine; 0 = not read
};
extern vehicle V;

void vehicle_update(void *gearbox);  // read what is due (see above); gearbox = the game's gearbox object, NULL = none now (mass and pitch still)
void vehicle_frame();                // core: vehicle_update(game_gearbox())
void vehicle_reset(int level);       // RESET_* (suite.h): LOAD = the mass is read again at once, f_bias 0; HARD = everything, the engine too
void vehicle_mass_stale();           // read the mass again at the next update (TrueShift: a start)

// Gear and speed (the ratios, the final drive and the wheel radius from the SDK, corrected by rpm_k). 0 when the gear or the
// truck is unknown. v in m/s like the telemetry, kmh where the name says so.
float vehicle_rpm_at(int gear, float v);                             // engine rpm in gear at speed v
float vehicle_kmh_at(int gear, float rpm);                           // the speed where gear sits at rpm (the inverse)
float vehicle_kmh_idle(int gear, float idle_rpm, float margin_rpm);  // the speed where gear sits at idle + margin
// Forces (N) and what they give. share = the share of the torque curve asked (a throttle 0..1); pct = a road grade in %
float vehicle_force_at(int gear, float v, float share);  // tractive force in gear at v; 0 over the rpm limit
float vehicle_resist(float v);                           // rolling + air + the grade of now (V.grade_deg) + f_bias
float vehicle_resist_grade(float v, float pct);          // the same on another grade (the route ahead)
// "Can this gear do it with a bit more throttle?" in one call: acceleration (m/s2) in gear at v with this share against the
// resistance on grade pct (NAN = the grade of now). For the score-based gear choice; no caller yet
float vehicle_accel(int gear, float v, float share, float pct);

// The service brake's gain, learned from steady braking (core/vehicle.cpp). All of it is off while [core] brake_learn is 0.
extern float g_brake_learn;                               // [core] brake_learn: 0 = nothing is measured, stored or offered (the modules keep their own guesses)
float vehicle_brake_gain();                               // m/s2 per unit of brake request at the mass of now; NAN until confident (or off, or the mass unknown)
float vehicle_brake_conf();                               // 0..1; 0.5 and up = confident (the gain is offered)
float vehicle_brake_req(float decel, float fallback);     // the request (0..1) for a wanted deceleration (m/s2); fallback while the gain is unknown
float vehicle_brake_hold_req(float grade_pct, float fallback);  // the request (0.2..1) that holds the truck on a grade (%) with margin; fallback while the gain is unknown
void vehicle_brake_prime(int trailer, float gm, float dev, float n);  // sims: set a slot (gm = N per request, dev = spread, n = events) as if it had been learned
