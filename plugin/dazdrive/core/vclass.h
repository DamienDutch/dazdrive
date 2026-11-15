// The vehicle class picks the module set and the profile (core/vehicle.cpp decides the class; this is what hangs on it).
//
//  - The module set: every module carries a class mask (struct module::classes: VCM_TRUCK, VCM_CAR, VCM_BOTH). The dispatcher (core/main.cpp,
//    core/telemetry.cpp) runs frame, config and reset only for the modules of the class that is active, and a module outside the set
//    behaves as if it were absent: a hook it owns in the game asks module_active() and passes straight through to the game's own
//    behavior. When the class changes (another vehicle, a load) vclass_sync() reports it and the dispatcher does a RESET_HARD of
//    every module, so the set that comes in starts clean and the set that goes out lets go of what it held.
//  - The profile: a module that runs for both classes reads vprof() for what differs between them, instead of asking whether the vehicle
//    is light. One struct per class; a feature the class does not have is absent from its profile (0), not switched off in the code.
//
// Truck values that come from the ini stay in the modules' parameters: the profile says NAN there ("the module's ini value", see
// vp_or), the car profile holds its own figure. So a truck reads exactly what it always read.
#pragma once
#include "suite.h"
#include <math.h>

struct vclass_profile {
    const char *name;               // "truck" / "car" (the log)
    // ---- Cruise: ACC (modules/cruise.cpp)
    float acc_gap_s, acc_gap_m, acc_gap_l;   // time gaps (s) of the close / middle / far distance step
    float stop_m, stop_grow;        // the gap at a standstill (m) and how much it opens up by 30 km/h (NAN = the distance-step table / the module's constant)
    // ---- Cruise: emergency stop signal (hazards after a hard stop)
    float ess_dec, ess_off;         // hard stop above this deceleration (m/s2), off below this
    int ess_aeb_stage;              // an AEB partial or full stage also counts as a hard stop
    // ---- Cruise: AEB
    float aeb_warn_s, aeb_part_s, aeb_full_s;      // time-to-collision of the warning / partial / full stage (s)
    float aeb_part_dec, aeb_max_dec;               // partial / full deceleration (m/s2)
    float aeb_warn_dec_on, aeb_warn_dec_off;       // the "braking needed" bar of the warning (m/s2) with / without the cruise on
    int aeb_off_in_sport;           // the AEB is off while the driver is in Sport (Power): the driver is in charge
    // ---- Cruise: the controller
    int gain_by_accel;              // the cruise gains scale with the acceleration a full throttle gives (a car's well over a truck's)
    // ---- Lane
    // the lane assist's integral, heading and bend gains, times this. It stays a class figure (checked 01-11): the lane learner's profile is kept
    // per steering WHEEL (steer_profile.txt, winmm name), not per vehicle, so it cannot hold a car's quicker yaw next to a truck's; it learns only
    // while a calibration is armed; and a wheel without a profile starts at a scale of 1.0 (learn_max), which would put a car above today's
    // 0.85 until calibrated. The vehicle data the lane module sees (no wheelbase, no steering ratio) does not give a better number.
    float lane_gain_k;
    // ---- what the class has (1) or has not (0)
    int rollover_guard;             // the rollover guard (core/rsc.cpp): truck physics
    int truck_limiter;              // the truck speed limiter caps the set speed
    int trailer_cap;                // a trailer brings its own cap ([cruise] car_trailer_limit_kmh) in place of the truck limiter
    int air_brakes;                 // compressed-air brakes: air gating of our brake requests, air saving in the AEB
    int route_features;             // the navigation route feeds the shift, the cruise and the planner (core/route.cpp route_ok)
    int car_speed_limit;            // the speed limit comes from the game's car limit, not the truck limit of the SDK
    // ---- features the class decides, not the ini (1 = on; -1 = the ini decides, a truck's model-year choice): a modern car has all three
    int aeb_on;                     // the own emergency brake assist (Cruise); its hooks are in whatever the ini says and pass through while it is off
    int ess_on;                     // the emergency stop signal: hazards after a hard stop (Cruise)
    int start_stop_on;              // the start-stop at an Auto Hold stop (Cab)
};

// The two profiles. Every light vehicle is a modern car (ACC, AEB and start-stop exist in it), whatever the year or the mod: the model years of
// the setup page are a truck thing and do not reach it.
inline const vclass_profile k_vprof_truck = {
    "truck",
    NAN, NAN, NAN,            // ACC gaps: the ini (acc_short_s / acc_middle_s / acc_long_s)
    NAN, NAN,                 // standstill gap: the distance-step table
    NAN, NAN, 0,              // emergency stop signal: the ini (emergency_stop_decel / _off_decel), no AEB trigger
    NAN, NAN, NAN, NAN, NAN,  // AEB: the ini
    2.8f, 1.5f,               // the AEB warning's bar with / without the cruise (m/s2)
    0,                        // the AEB stays on in Sport
    0,                        // the gains as the ini says
    1.0f,
    1, 1, 0, 1, 1, 0,
    -1, -1, -1 };
// Every light vehicle is a modern car: ACC, AEB and start-stop on, whatever the year or the mod (no truck-year era tiles for it).
inline const vclass_profile k_vprof_car = {
    "car",
    0.8f, 1.0f, 1.2f,         // ACC time gaps (s) for the close / middle / far step
    2.5f, 1.5f,               // standstill gap (m) and how much it opens up by 30 km/h
    6.0f, 3.0f, 1,            // emergency hazards: hard stop above 6 m/s2, off below 3; an AEB partial/full stage counts as hard
    2.5f, 1.7f, 1.0f,         // AEB warning / partial / full time-to-collision (s)
    4.0f, 9.0f,               // AEB partial / full deceleration (m/s2)
    4.2f, 3.2f,               // AEB warning bar with / without the cruise: a car brakes far better, a truck's ~1.5 would fire on every fast close
    1,                        // the AEB is off in Sport
    1,                        // gains scaled by the acceleration a full throttle gives
    0.85f,                    // a car steers faster than a truck: the lane loop gains a little lower
    0, 0, 1, 0, 0, 0,         // the limit is the one the game reports: car limits in Road Trip, or when the game is set to car limits (Damien 01-11)
    1, 1, 1 };

const vclass_profile *vprof();      // the profile of the class that is active now (vclass_active)
// The value of a profile field, or the module's own (ini) value where the profile says NAN
static inline float vp_or(float prof, float ini) { return prof == prof ? prof : ini; }

int vclass_active();                // VC_TRUCK or VC_LIGHT (core/vehicle.h): the class the module set runs for; moves only in vclass_sync()
int vclass_sync();                  // adopt the class core/vehicle.cpp decided; 1 = it changed (the dispatcher then resets every module hard)
int module_active(const module *m); // 1 = the module's class mask contains the active class (0 mask = both)
