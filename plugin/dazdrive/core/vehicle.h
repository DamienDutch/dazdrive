// The vehicle model: one estimate of the truck for every module (core/vehicle.cpp). The mass and the road grade come from
// the game's own functions (the ones its automatic uses), the engine from its definition; the forces (what a gear pulls,
// what the road asks) are ours. vehicle_update() reads what is due: core calls it every frame_end (after telemetry_frame),
// and TrueShift's hooks call it again inside the game's gearbox update, so a decision there sees this frame's grade and a
// fresh mass right after a reset (the mass and the engine have their own cadence, so the second call costs a grade function
// call). Modules read V and call the functions below; only the owner and vehicle_reset write it, except the three corrections
// TrueShift learns while it drives (rpm_k, f_bias, f_scale).
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
    // engine (on a new truck configuration, a load, and every 30 s)
    engine_info eng;     // torque, curve, limit, ranges, shift time
    int eng_ok;          // 1 = eng looks sane
    int electric;        // the last read saw an electric motor (also when the rest of the data was not sane)
    double eng_at;
    unsigned eng_gen;    // T.cfg_gen when eng was read
    float plateau;       // rpm where the torque curve reaches its plateau
    // the resistance model; the ini keys live in [trueshift] (rolling_resistance, drag_area, efficiency)
    float crr;           // rolling resistance coefficient
    float cda;           // air drag Cd*A (m2) for trucks; vehicles under 4.5 t get 1.0 (cars, pickups)
    float eta;           // driveline efficiency
    // learned by TrueShift while the clutch is closed and the pull is steady
    float rpm_k;         // measured rpm / driveline rpm (tire size etc.), 1 = as the ratios say
    float f_bias;        // extra resistance (N) the model misses (tires, wind, a wrong mass); 0 again with another load or engine
    float f_scale;       // multiplier on the modeled engine force (a truck the model under- or over-rates); 1 = as modeled, 0.85..1.25; survives a load change, resets with another engine
    float peak_pw;       // the engine's peak power as torque factor x rpm (the highest point of the torque curve times rpm), set with the engine; 0 = not read
    // torque converter (set by TrueShift once it detects one, primed by the sims). A TC automatic has no clutch slip to
    // manage: creep and launch are the converter's crawl, so nothing drives the clutch hook on it. Below the coupling the
    // engine runs at a throttle-scaled speed from idle to tc_stall_rpm and the converter multiplies the torque by the stall
    // ratio fading to 1 as the driveline catches up; once coupled (or above the lockup gear) rpm == the driveline rpm.
    int tc;              // 1 = torque-converter automatic
    float tc_stall;      // stall torque ratio (>= 1; 1 = no converter): the ini's when set, else learned from the launches (core/vehicle.cpp, WP-TC1; it starts at 2.0 or what the last drive stored)
    int tc_lock_fixed;   // 1 = tc_lock_gear was given (a sim's prime): it is not learned over
    int tc_stall_fixed;  // 1 = tc_stall was given (the ini, a sim's prime): it is not learned over
    int tc_lock_gear;    // first gear with lockup (0 = never locks); gearbox numbering as T.ratio
    float tc_stall_rpm;  // engine speed at full stall with the pedal floored (rpm); 0 = derive from the power range
};
extern vehicle V;
// A rev ratio that is not a plausible number (NaN, under 0.5) is the ratios' own: 1. The gearboxes call it every frame; modules do not assign to V.
static inline void vehicle_rpm_k_sane() { if (!(V.rpm_k > 0.5f)) V.rpm_k = 1; }

// A mod engine that defines only its torque (WP-TC1, core/vehicle.cpp): engine_raw_check sets e->missing from a raw read (1 = the torque is usable), engine_fill_defaults
// puts the game's own defaults where something is missing (1 = usable now). game.cpp's gb_engine and the sims' stubs call the first, vehicle_update the second.
int engine_raw_check(engine_info *e);
int engine_fill_defaults(engine_info *e, float sdk_limit);
// The converter's stall torque ratio from the game's transmission object (WP-TC2, owner decision 61: dynamic, not per truck in the ini). The object holds, in the 12 bytes
// ending at the shift time (t_shift_time, game.h layout): stall_torque_ratio at -8, differential_ratio at -4. The diff equals the telemetry's differential (the truck config's,
// `diff`, within 0.005) or the layout is not trusted. trans_stall_read: the raw figure (0 .. , a clutch box holds 0 or 1) or NAN; tr NULL = not readable. engine_read_stall
// fills e->tc_stall from it (see engine_info) and notes ONE line for vehicle_update to log.
float trans_stall_read(const char *tr, unsigned shift_off, float diff);
void engine_read_stall(engine_info *e, const char *tr, unsigned shift_off, float diff);
void vehicle_update(void *gearbox);  // read what is due (see above); gearbox = the game's gearbox object, NULL = none now (mass and pitch still)
void vehicle_frame();                // core: vehicle_update(game_gearbox())
void vehicle_reset(int level);       // RESET_* (suite.h): LOAD = the mass is read again at once, f_bias 0; HARD = everything, the engine too
void vehicle_stop();                 // core: flush the persisted learnings at shutdown
void vehicle_mass_stale();           // read the mass again at the next update (TrueShift: a start)

// ---------------------------------------------------------------- the vehicle class (light-vehicle / car mode)
// One decision for every module: is this a light vehicle (a car, a pickup) or a truck? A light vehicle is one whose OWN mass
// (without trailers) is at most VEHICLE_CLASS_LIGHT_KG (3.5 t) or, whatever its mass, a car by definition (def/vehicle/car, the
// ATS Road Trip DLC -- game_vehicle_is_car()). With a trailer a light vehicle stays light (a pickup towing is still a car), so the
// car-with-trailer rules (a lower cap instead of the truck limiter) come from the class profile (core/vclass.h) and T.trailer together.
// Only this file decides the class; the module set and the profile follow it through vclass_sync() (core/vclass.h), and a module that differs by
// class reads vprof(), not vehicle_light().
// [core] vehicle_class overrides the decision: 0 = auto (default), 1 = truck, 2 = light (the words auto / truck / light are accepted
// in the ini too). The decision is made in vehicle_update() when the mass is read and logged once per change; before the first read
// vehicle_class() reports a truck (the cautious answer).
enum { VC_AUTO = 0, VC_TRUCK = 1, VC_LIGHT = 2 };
static const float VEHICLE_CLASS_LIGHT_KG = 3500.0f;   // the own-mass rule (3.5 t)
extern float g_vehicle_class;            // [core] vehicle_class (the override, VC_AUTO by default)
int vehicle_class();                     // VC_LIGHT or VC_TRUCK (never VC_AUTO once the module is up); VC_TRUCK before the first read
int vehicle_light();                     // 1 = a light vehicle now
float vehicle_own_mass();                // the towing vehicle's own mass (kg, no trailers) the class was decided on; NAN unknown

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

// Torque converter (see the fields above). tc = 1 on a converter box; on others everything below is inert (locked, no boost).
int vehicle_tc();                                     // V.tc
float vehicle_tc_stall();                             // V.tc_stall (>= 1, or 1 = none)
void vehicle_tc_coupled_from(int lock_gear, int from_gear);   // gc_choose: judge the gears from from_gear up (and over lock_gear) as coupled for the length of one decision (lock_gear 0 = the model as it is)
int vehicle_tc_lock_gear();                           // V.tc_lock_gear
float vehicle_tc_stall_rpm();                         // the engine's stall speed (rpm), derived when tc_stall_rpm is 0
float vehicle_tc_slip(int gear, float v);             // 1 = coupled; below the coupling the slip ratio (turbine rpm / throttle-scaled couple rpm, 0..1)
float vehicle_foot_force_at(int g, float v);   // the raw pedal's pull in gear g (a slipping converter passes the pedal through, the shaped demand is not delivered there)
int vehicle_tc_locked(int gear, float v);             // 1 = the lockup clutch is in (rpm == driveline rpm), 0 = slipping
// THE converter test (WP-TC1, the owner 04-11: "as long as the lockup is off, you may take a gear that in theory could not"): 1 = a converter below its lockup in gear g at v (m/s) whose
// multiplication at that slip lets the gear carry the wish (wish_n, a force in N) against the road, so no rpm floor (the stall line, the bands' floor) may refuse or take the gear. 0 = a
// locked or coupled-and-locked gear, no converter, or a gear that cannot carry it: the rpm checks stand as they were. Every place that holds a landing against a floor asks this.
int vehicle_tc_carries(int gear, float v, float wish_n);
void vehicle_tc_stall_step(float clutch);             // the stall ratio learned from the launches (the brains call it with every update; clutch = clutch_seen()), see core/vehicle.cpp
float vehicle_engine_rpm_at(int gear, float v);       // the engine's actual rpm in gear at speed v (the couple speed while slipping)

// THE SLIPPING LAUNCH of a clutch box (K1 round 16; the ONE place of its numbers). On 1.62 the game's clutch holds the engine at about 1.25 x the idle while it slips and the truck pulls away
// (the eight pull-aways of the ride of 08-11, notes/rit-0811-analyse.md section 4: 636 .. 740 rpm at an idle of 550, 1.16 .. 1.35), the engine gives the torque it has at that rpm to the wheels
// at the pedal's throttle (the pedal goes through while the clutch slips, modules/pedal.cpp), and the clutch is closed once the driveline reaches that rpm. The start gear (core/shiftcommon.cpp
// sc_launch_pick), the layers (modules/trueshift.cpp: no decision while it slips) and the sim's plant (tools/sim/sim.cpp g_slip_k) all read this one figure. A converter has none (its own model above).
// Fitted to the ride of 08-11, not read from the game: correct it here when a ride says otherwise.
static const float CLUTCH_SLIP_K = 1.25f;
float vehicle_slip_rpm();       // the rpm the clutch holds the engine at while it slips: CLUTCH_SLIP_K x the idle; 0 on a converter box or without an idle
int vehicle_slipping();         // 1 = the clutch is slipping NOW, seen in the telemetry: a forward gear in, the engine's rpm above the driveline's of that gear (the fact, not the model)
void vehicle_tc_prime(float stall, int lock_gear, float stall_rpm);   // sims: set the converter as if detected
// The effective ratios of a converter box (core/vehicle.cpp): vehicle_rpm_at and vehicle_kmh_at carry each gear's measured figure q (the rpm
// per speed over the ratios' own) on a converter, and the tire correction V.rpm_k stays 1 there. 1 everywhere else.
enum { RATIO_COAST = 0, RATIO_PULL = 1 };
float vehicle_ratio_base(int gear);                   // the modelled driveline rpm per m/s of a gear before the calibration (0 = unknown gear)
float vehicle_ratio_q(int gear);                      // the gear's calibration (1 = as the ratios say; the mean of the calibrated gears when it has none of its own)
int vehicle_ratio_ready(int gear);                    // 1 = both sides measured, 2 = one side, 0 = a fallback
void vehicle_ratio_sample(int gear, float v, float rpm, float dt, int side);   // TrueShift: the measured engine rpm at speed v (m/s) while coasting (RATIO_COAST) or pulling steadily (RATIO_PULL); converter only
void vehicle_ratio_prime(int gear, float q);          // sims: a gear's figure as if calibrated
int vehicle_tc_stored(float *stall, int *lock_gear);   // the converter the last drive of this truck found (ratio_cal.txt), 1 = stored: TrueShift starts with it known
void vehicle_tc_stored_prime(float stall, int lock_gear);   // sims: as if stored
float vehicle_idle_stored();                          // the idle rpm TrueShift learned for this truck in an earlier drive (ratio_cal.txt), 0 = none
void vehicle_idle_note(float idle);                   // TrueShift: the idle it learned (kept for the next drive of this truck; light vehicles only)

// ---------------------------------------------------------------- the brake model (WP-M2)
//
// ONE model of how the truck brakes, for every module that brakes. The start values are derived from what the truck is (core/brakemodel.h
// has them, with their derivations and the measurements they were checked against), then learned per truck and kept in brake_gain.txt (the
// service brake) and brake_model.txt (the retarder and the engine brake, per stage) under the same truck key. A module has no brake constant
// of its own: it asks here. [core] brake_learn 1 (the default) measures and stores; 0 stops all of it that learns or stores, and the start
// values still answer.
//
// The service brake's gain, m/s2 per unit of request at the mass of now. Value and confidence are two things:
extern float g_brake_learn;                               // [core] brake_learn, default 1; 0 = nothing is measured, stored or loaded
float vehicle_brake_prior();                              // the start value, derived from the mass (never NAN): 10 m/s2 up to 30 t, the same force above
float vehicle_brake_est();                                // the estimate to size a request by (never NAN): the prior moved towards the learned slot by vehicle_brake_weight()
float vehicle_brake_weight();                             // 0..1, the learned slot's share in the estimate: its confidence (ramped over 0..0.5) x its plausibility (a slot far ABOVE the prior is down-weighted by its distance in sigmas, more slowly as events back it; weaker brakes are believed at once)
float vehicle_brake_cautious();                           // the gain a REQUEST may count on: the estimate, never above the prior (brakes measured stronger than the start value are for planning and the pedal; a request sized by this brakes as asked or harder, never softer than the start value did, and for weak brakes it is the estimate)
float vehicle_brake_usable();                             // the largest deceleration a plan may count on from the service brake: the estimate, never over 0.6 g
float vehicle_brake_conf();                               // 0..1, the learned slot's own confidence; 0.5 and up = confident (AEB's soft start, the pedal provider, the holds' sizing key on this)
// The old entry points, with their old meaning (a module that gates on "learned" keeps working; TODO(LS): lowspeed.cpp's ls_hold_req / ls_sg_hold_req /
// ls_assist_req and its LS_ASSIST_REQ / LS_HOLD_REQ / LS_SG_HOLD_REQ fallbacks move to vehicle_brake_est() with the launch package):
float vehicle_brake_gain();                               // the estimate once the slot is confident; NAN before (or off, or the mass unknown)
float vehicle_brake_req(float decel, float fallback);     // the request (0..1) for a wanted deceleration (m/s2); fallback while the gain is not confident
float vehicle_brake_hold_req(float grade_pct, float fallback);  // the request (0.2..1) that holds the truck on a grade (%) with margin; fallback while the gain is not confident
void vehicle_brake_prime(int trailer, float gm, float dev, float n);  // sims: set a slot (gm = N per request, dev = spread, n = events) as if it had been learned
//
// The retarder, force (N) at the wheels per stage (1..T.retarder_steps): the stage table of core/brakemodel.h times the stage's learned correction.
// A force, so a load change keeps it. Stages without a sample take the mean correction of those with some.
int vehicle_ret_steps();                                  // the truck's stage count (the def's retarder_step_count), 0 = no retarder
float vehicle_ret_prior(int stage);                       // N, the start value of a stage
float vehicle_ret_force(int stage);                       // N, prior x learned (0 = no retarder or no such stage)
float vehicle_ret_force_full();                           // N at the top stage (the channel's full demand); the table's top for a truck without a retarder (views, sims)
float vehicle_ret_force_frac(float frac);                 // N at a fraction 0..1 of the stages (a continuous demand: the table between the knots, the nearest stage's correction)
int vehicle_ret_events(int stage);                        // the samples behind a stage (0 = the start value only)
void vehicle_ret_note(int stage, float force_n);          // a measured force of a stage (the channel's learner, a clean lever window): blended in, stored per truck; a figure outside 0.4-2.5 x the prior is not a retarder's
float vehicle_ret_knee_ms();                              // m/s: the road speed where a retarder's n^2 torque reaches the top stage's force, = the engine's peak power over that force (core/brakemodel.h, WP-BR3b); 0 = not known (no engine read yet, no retarder)
float vehicle_ret_shape(float v);                         // 0..1: the share of a stage's force a retarder gives at the road speed v (m/s); 1 where the knee is not known. The game's retarder has no such fade (it follows the engine side); the lever assist drives the game's retarder to this shape
float vehicle_ret_shape_stage(float v, float f);          // the same for the stage that is the share f (0..1) of the top stage's force: its knee is the top stage's times sqrt(f), so a low stage fades later (core/brakemodel.h, WP-0911-K4)
// (the lever-stage windows -- a steady lever step on a level road, throttle shut, no service brake -- teach the stage in the channel: core/brake.cpp bc_learn_lever, WP-BR)
//
// The engine brake, force (N) at the wheels per stage (1..cap) in a gear: the stage's share of the engine's brake torque (a share of the peak torque)
// through the gear, times the stage's learned correction. Replaces EBrake's d_est and Decel's EB_GUESS / dfc.
float vehicle_eb_prior(int stage, int cap, int gear);     // N
float vehicle_eb_force(int stage, int cap, int gear);     // N, prior x learned
float vehicle_eb_decel(int stage, int cap, int gear);     // m/s2 at the mass of now
float vehicle_eb_rpm_share(float rpm);                   // 0..1: how much of the stage's torque the engine brake gives at this rpm (a line from idle to the low edge of the engine's brake range; 1 where the range is not known)
float vehicle_eb_decel_at(int stage, int cap, int gear, float rpm);   // m/s2 at the mass of now AND the rpm of now (vehicle_eb_decel x vehicle_eb_rpm_share)
int vehicle_eb_events_total();                            // the samples behind all stages of this truck's engine brake (a stage without a sample takes the others' mean: the model is as good as this)
int vehicle_eb_events(int stage);                         // the samples behind a stage (0 = the start value only)
void vehicle_eb_note(int stage, int cap, int gear, float force_n);   // a measured force of a stage in a gear (Decel's bridge gap, EBrake's step): blended in, stored per truck
//
// The natural deceleration: what slows the truck by itself.
float vehicle_drag_force(int gear, float v);              // N at the wheels: the engine's drag in gear with the throttle shut (the learned share of the peak torque at the rpm of the gear; TrueShift's drag_at, moved here)
float vehicle_natural_decel(int gear, float v);           // m/s2 (+ = slowing, - = it speeds up): rolling, air, the grade of now, f_bias and, in a gear (gear > 0: closed clutch, throttle shut), the engine's drag; gear 0 = neutral or the clutch open

// The fuel / drag learning (core/vehicle.cpp, always on; the next task reads it to decide gears and eco-rolls, this one only
// learns and logs). fuel_a() is the zone scale (a reference, so it jumps between map zones), fuel_k() the engine's friction
// torque as a share of its peak torque (0.02..0.25, prior 0.09), fuel_kd() the engine's drag in gear with the throttle shut
// as a share of peak torque (0.01..0.2, prior 0.06). fuel_learned() = 1 once enough windows have been seen.
float fuel_k();
float fuel_kd();
float fuel_kd_rpm();
void vehicle_fuel_prime(float k, float kd, float kdr);   // sims: set the fuel model as if it had been learned   // the rpm fuel_kd() was measured at: the drag scales with the rpm
float fuel_a();
void vehicle_fuel_counts(int *n_k, int *n_kd, int *n_a);   // sims: the windows each estimate has taken so far
int fuel_learned();
