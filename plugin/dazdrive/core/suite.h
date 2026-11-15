// Shared declarations for the suite: DazDrive, one in-game plugin DLL for ETS2/ATS with
// several modules (TrueShift now, ACC and lane keeping later).
// MIT license. The hook layer is derived from car_automatic_transmission
// (MIT, (c) 2026 Deppelover, github.com/deppelover/car_automatic_transmission).
#pragma once

#define SUITE_NAME "dazdrive"  // DazDrive: DLL, ini and output folder
#define SUITE_VERSION "0.7.0-beta.1"

#include <windows.h>
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include "scssdk_telemetry.h"

// ---------------------------------------------------------------- log.cpp

extern char g_dir[MAX_PATH];  // output folder (plugins\SUITE_NAME\), trailing backslash
void log_open();
void logline(const char *fmt, ...);  // one timestamped line in SUITE_NAME.log (and in the text ring)
void logring(const char *fmt, ...);  // only in the text ring: it reaches the disk only inside an incident file
void log_lock();
void log_unlock();
void log_ring_dump(FILE *f, double since, double t0);
extern void (*g_log_tap)(double t, const char *s);
double wall_s();                  // seconds since the DLL started
double sim_s();                   // simulation time in seconds (stops while paused)
FILE *open_out(const char *name, const char *mode);
// A csv trace past csv_max_mb moves to <name>.1 (one generation) and starts again with the same header line.
void csv_cap(FILE **f, const char *name);
extern double g_csv_max_mb;
// A new incident file <output>\<dir>\<prefix>_<local date_time>.csv; the folder is first trimmed to max_mb (oldest first).
// stamp gets the local date_time part of the name (yyyy-mm-dd_hh-mm-ss). NULL if it cannot be made.
FILE *open_incident(const char *dir, const char *prefix, double max_mb, char *stamp, size_t n);  // NULL for *.csv below log_level 2

// log_level (ini [core]): 0 = events only (start, hooks, errors, mode changes, incidents), 1 = + every shift and the
// modules' detail lines, 2 = + periodic traces and the csv trace files. Lines of level 1 are kept in the text ring anyway,
// so an incident file has them even at level 0.
extern int g_log_level;
extern double g_log_max_mb;
// LOG_BOOST: an experimental module defines it as 1 before including this file, so its detail lines already show one
// log level lower (the more experimental, the more logging by default)
#ifndef LOG_BOOST
#define LOG_BOOST 0
#endif
#define logv(lvl, ...) do { if (g_log_level + LOG_BOOST >= (lvl)) logline(__VA_ARGS__); else if ((lvl) <= 1) logring(__VA_ARGS__); } while (0)

// ---------------------------------------------------------------- incident.cpp
// An incident = something went wrong or looked odd (a hunting gearbox, the driver fighting a shift, the rpm limiter, stuck
// in neutral, a ringing lane assist, AEB). incident() writes incidents\<kind>_<local date_time>.csv: the log lines and the
// rows (10 per second) of the 20 s before, and everything until 15 s after the last trigger. The folder is kept under
// incident_mb (oldest out first); dazdrive.log only gets one line per incident.
struct incident_ctx {
    volatile int mode;        // gearbox mode: 0 off/game, 1 Eco, 2 Normal, 3 Power, 4 M
    volatile int target;      // the gear TrueShift wants (0 = none)
    volatile int flags;       // INC_* bits
    volatile float grade_pct; // road grade, % (NAN unknown)
    volatile float gap_m;     // radar gap to the vehicle ahead, m (NAN none)
};
enum { INC_ROLL = 1, INC_CRAWL = 2, INC_AEB = 4, INC_ISA = 8, INC_HOLD = 16 };
extern incident_ctx g_inc;
extern double g_incident_mb;
void incident(const char *kind, const char *fmt, ...);
void incident_frame();   // every frame, after the modules
void incident_stop();

// ---------------------------------------------------------------- config.cpp

// A module parameter. Values live in the module as floats; ints and booleans
// are stored as floats too, which keeps the table simple.
struct param {
    const char *key;
    float *val;
    float def;
    const char *help;  // one line, written into the default ini
};

struct module {
    const char *name;   // ini section
    const char *title;  // for the log
    param *params;
    int n_params;
    void (*start)();                                          // after config and discovery
    void (*frame)();                                          // SDK frame_end
    void (*config)(const scs_telemetry_configuration_t *c);   // SDK configuration event; c NULL = the dispatcher replays the vehicle's configuration (already read into T) to a module that has come into the set after it was announced
    void (*stop)();                                           // telemetry shutdown
    // Optional (NULL = none). reset: something broke continuity (see RESET_*): clear timers, filters, rate limits
    // (and with RESET_HARD learned values and cached game pointers). status: one line for the log at start.
    void (*reset)(int level, const char *why);
    const char *(*status)();
    // Optional: a live setting of this section was just changed from the phone (config_live_set): the new value is in
    // its param, old is what it was. Redo whatever start() derived from it. NULL = it simply takes effect (the code
    // reads the param every frame); else a note for the ack ("after restart"): then put the old value back if the change
    // would run half-installed.
    const char *(*applied)(const char *key, float old);
    // Which vehicle classes the module belongs to (VCM_*; 0 = both). The class picks the module set: the dispatcher runs the frame, config and
    // reset of a module only while the vehicle's class is in its mask (core/vclass.h). A module outside the set behaves as if it were absent:
    // its hooks into the game pass straight through to the game's own behavior (they ask module_active() first).
    unsigned classes;
};
// The class masks (struct module::classes). VCM_BOTH modules read the class profile (vprof(), core/vclass.h) for what differs.
enum { VCM_TRUCK = 1, VCM_CAR = 2, VCM_BOTH = 3 };

enum { RESET_SOFT = 0,   // pause menu / long hitch / engine on: timers, filters, hold states
       RESET_LOAD = 1,   // trailer (de)coupled: mass dependent values
       RESET_HARD = 2 }; // other truck, save/load, teleport (ferry, train, quick travel): everything
void session_reset(int level, const char *why);  // telemetry.cpp: logs and calls every module's reset
void vclass_switched();     // telemetry.cpp: the vehicle class changed (vclass_sync() = 1): log the module set, reset it hard, replay the configuration
void vclass_log_initial();  // telemetry.cpp: the module set before the vehicle model has told the class (a truck's), once at start
extern const char *g_truck_unsupported;  // NULL, or why this truck is left to the game's own automatic (TrueShift sets it)


extern module *g_modules[];
extern int g_n_modules;
// Live settings (the phone's {do:'set'}, core/dev.cpp): a whitelist of "section.key" that may change while the game runs.
int config_live_count();
const char *config_live_key(int i);                  // "section.key"
int config_live_get(int i, float *v);                // the value now; 0 = not available
int config_live_set(const char *name, float v, const char **err, const char **note);  // checks, applies, writes the ini; 1 = ok
int cruise_acc_step(int dir, const char **err);      // ACC distance one step (-1 closer, +1 further), clamped at the ends
void config_load();  // reads SUITE_NAME.ini, writes it with defaults first if missing
void boot_once();    // main.cpp: log_open + config_load, once, from whichever SDK entry point (telemetry or input init) the game calls first

// ---------------------------------------------------------------- telemetry.cpp

struct telemetry {
    // channels
    float rpm, in_thr, eff_thr, in_brake, eff_brake, in_clutch, eff_clutch, speed, cruise, speed_limit, air_psi;
    float in_steer;    // steering input -1..1, + = left (counterclockwise)
    float brake_temp;  // service brake temperature, deg C (SDK truck.brake.temperature)
    float fuel;        // fuel amount, liters (SDK truck.fuel.amount, 1 mL resolution, jitters with the slosh)
    float yaw;         // yaw rate, rotations per second, + = turning left
    int gear, dgear, motor_brake, parking_brake, engine_on, trailer, paused, air_warning;
    int light_brake;   // the brake lights are on (the game's own decision, whatever asked for the brake)
    unsigned retarder, retarder_steps;
    scs_value_dplacement_t place;
    int blink_l, blink_r;                 // the indicator levers (logical state, not the flashing light)
    scs_value_dplacement_t trailer_place; // the first trailer's world placement (valid while T.trailer and trailer_place_t > 0)
    double trailer_place_t;               // wall_s() of its last update, 0 = never
    scs_value_fvector_t acc;
    scs_timestamp_t sim_time;
    // truck configuration
    int fwd_gears, rev_gears, wheels;
    float ratio[32], diff, rpm_limit;
    float fuel_cap;                 // tank size in liters (truck config, 0 = unknown)
    unsigned cfg_gen;               // +1 on every truck configuration (another truck, an engine swap at the dealer)
    float wheel_radius[16];
    int wheel_powered[16];
    char shifter[32];
    float air_warn_cfg, air_emerg_cfg;   // the truck's brake air warning / emergency pressure (psi), 0 = not reported
    float cargo_mass;
    // lift axles and lights (modules/axle.cpp, modules/lights.cpp). -1 = the channel never delivered.
    int lift_axle, trailer_lift_axle;     // the control is set to lifted (truck / trailer)
    int light_low, light_high;            // low beam / high beam on
    long long game_min;                   // game.time, in-game minutes (-1 unknown); time of day = game_min % 1440
    float wheel_w[16];                    // truck wheel angular velocity, rotations per second (+ = forward); NAN = no value
    float wheel_lift[16];                 // truck wheel lift 0 (down) .. 1 (fully lifted); NAN = no value
    float t_wheel_lift[12];               // the first trailer's wheel lift, the same; NAN = no value
    int wheel_liftable[16];               // config: the truck's wheel can be lifted
    int trailer_wheel_liftable[16];       // config: the first trailer's wheel can be lifted (if the game reports it)
    float wheel_z[16], t_wheel_z[16];     // config: the wheels' position along the vehicle (z, backward positive), NAN = unknown
    int t_wheels;                         // config: the first trailer's wheel count
    // derived
    float accel;       // longitudinal acceleration from speed, m/s2, filtered (ACCEL_LAG_S, ~0.5 s: the gearbox logic)
    float accel_fast;  // the same with ~0.12 s: the AEB (its windows are 0.45-0.6 s, the 0.5 s filter saw a third of the braking by then)
};
extern telemetry T;
// The time constant (s) of the low-pass T.accel is (telemetry_frame): a change in the deceleration shows in it this late. One place for the figure; whatever reasons about what the
// deceleration still contains of a brake that has let go (TrueShift's service-brake memory, SVC_TAIL_LAG_S) starts from it.
static const float ACCEL_LAG_S = 0.5f;
unsigned telemetry_seen();   // bits of the channels that have delivered a value: 1 lift axle, 2 trailer lift axle, 4 low beam, 8 high beam, 16 game time
void telemetry_register(const scs_telemetry_init_params_v101_t *p);
void telemetry_frame();  // derived values, called first in frame_end

// ---------------------------------------------------------------- shared between modules

// sound.cpp: feedback sounds, async, one at a time; g_volume 0..1 (0 = silent)
enum Cue { CUE_MODE_ECO, CUE_MODE_NORMAL, CUE_MODE_POWER, CUE_MODE_M, CUE_ACC_1, CUE_ACC_2, CUE_ACC_3, CUE_ACC_OFF,
           CUE_ON, CUE_OFF, CUE_AEB_WARN, CUE_AEB_PARTIAL, CUE_AEB_FULL, CUE_CRUISE_DROP,
           CUE_LANE_PAUSE, CUE_LANE_RESUME, CUE_OVERSPEED, CUE_LANE_CAL_ARM, CUE_LANE_CAL_DONE, CUE_LANE_CAL_ABORT, CUE_COUNT };
void play_cue(int cue);
// input.cpp: finds the wheel (the present joystick with the most buttons) for lane's per-wheel profile
void input_poll();
const char *input_wheel_name();  // winmm name of the most button-rich present joystick ("" = none found yet)
// The semantic input device (SDK input 1.00): the plugin presses a game control by its mix name (core/inputq.h has the list). A press is
// value 1 for one game frame then 0 (hold_ms 0), or for hold_ms; several presses in one frame are all sent, in order, one frame apart.
// A held level (input_hold) is one event per change. Returns 1 = queued, 0 = could not (no device: ini, refused by the game, other game;
// unknown mix); the caller logs it once and carries on. Do not use press and hold on the same mix.
int input_press(const char *mix, int hold_ms = 0);
int input_hold(const char *mix, int on);
int input_device_ready();           // registered and not told inactive
const char *input_device_state();   // for the log
extern float g_input_device;        // [core] input_device
// Cab (modules/cab.cpp): the auto parking brake feature is on (park_auto=1). Auto Hold keeps its parking brake on at engine
// off for the cab feature to release again.
int cab_park_auto_on();
// Cab (modules/cab.cpp): our start-stop has the engine off right now (a stop we asked for, not the driver switching off).
int cab_ss_engine_off();
// Cruise (Auto Hold): the parking brake is Auto Hold's own right now (set after a long hold); Auto Hold releases it itself.
int ah_park_owned();
// Cruise (Auto Hold): Auto Hold is holding the truck now (AH.st == AH_HOLD).
int ah_holding();
// Cruise (radar): is there a vehicle ahead now? 1 = yes, *m its distance (m), *kmh its speed (km/h; 0 = unknown).
int cruise_lead(float *m, float *kmh);
extern float g_volume;
extern volatile int g_rolling;        // TrueShift has the truck in neutral (eco-roll)
inline volatile float g_bend_lat = 3.125f;   // Cruise's curve_lat (bends' sideways allowance): route_profile's bend caps use the same law as the cruise (review 30-10)
extern volatile float g_roll_boost_kmh;   // TrueShift asks Cruise for this much over the set speed (km/h) just before an eco-roll starts (0 = none)
extern volatile double g_rb_cancel_at;   // Cruise: wall time it cancelled itself for a roundabout (Lane switches lane keeping off for it, back on at the next cruise resume)
// Lane: arm / disarm the steering calibration like a long hold of the lane-keeping key (the phone button box, dev builds). NULL = done (*armed = 1 armed, 0 disarmed), else why not.
const char *lane_cal_request(int *armed);
// Lane: what the lane assist is doing right now, one value (lane.cpp sets it where its gates decide; the dev display reads it).
// OFF = lane keeping off (the game does not call the law); WAIT_* = on, but no request of ours: under min_kmh, the cruise is off
// (needs_cruise), an indicator pause, no lane seen; ACTIVE = steering; RUNAWAY = the runaway guard handed the game's own law the wheel
enum LaneState { LANE_OFF, LANE_WAIT_SLOW, LANE_WAIT_NO_CRUISE, LANE_WAIT_INDICATOR, LANE_WAIT_NO_LANE, LANE_ACTIVE, LANE_RUNAWAY, LANE_WAIT_LOAD };   // (appended: LANE_WAIT_LOAD = let go for the sideways load, WP-LA5)
extern volatile float g_clutch_ours;  // the clutch value Creep puts on the drivetrain (launch or crawl), NAN when the game's own applies
static inline float clutch_seen() { float c = g_clutch_ours; return c == c ? c : T.eff_clutch; }  // what the drivetrain sees: use instead of T.eff_clutch for 'is the clutch closed'
extern volatile float g_pcc_need_kmh;  // TrueShift: extra speed over the set speed the next climb on the route needs (NAN = unknown)
// Decel view for the dev feed: the deceleration target and the measured one (m/s2, + = slowing; NAN while neither the lever assist nor the stop assist works) and the stop assist's state name.
void decel_view(float *target, float *measured, const char **stop_assist);
// The stop assist ([decel] stop_assist, modules/decel.cpp): inline variables, so every build that includes this header has them (no definition elsewhere).
inline volatile int g_sa_nohold;       // Decel: the stop assist is braking a stop that is released at the standstill (stop_assist 1): Auto Hold must not take that brake over
// The brake channel (core/brake.cpp): inline variables, all 0 while [brake] channel and pedal are 0.
// (what the gearbox is asked of the brakes is not a global any more: the standing requests of core/gearreq.h)
inline volatile float g_brake_fed;       // Brake: the retarder demand we feed the game now (0..1): the learners must not take its deceleration for the service brake's or the engine's
inline volatile int g_brake_pedal_mapped;   // Brake: the pedal provider maps the driver's pedal: the service brake applied is T.eff_brake, not T.in_brake
// The one stop-open margin the low-speed owner and the stop assist share (review 30-10: they used 150 and 250 for the same event,
// "the box opened the clutch by itself"; the owner's 150 is the shipped value): the clutch opens when the start gear sits this far
// over idle (rpm), by the engine's span (rs_off, core/rpmspan.h).
static const float STOP_OPEN_MARGIN0 = 150;
// The pedal's noise floor (WP-LS): below this a pedal reads as released, above it the foot is on the pedal and the truck does something (promise P5: no dead zone at the bottom of the pedal).
// One count of an 8-bit pedal axis, 1/256: the coarsest game controller axis; a resting foot or a worn sensor on a finer axis reads under it. The SDK's pedal is a float, exactly 0 with the
// foot off; the old gates were 0.02 (a pedal free) and 0.06 (a launch), 5 and 15 counts of that axis.
static const float PEDAL_NOISE = 1.0f / 256.0f;
// PCC bands written in km/h are meant for a set speed of 70 km/h and scale with it (Scania's band is a percentage of the set speed):
// calmer at 60, roomier at 90. 1 when the cruise is off.
static inline float band_scale(float set_ms)
{
    if (!(set_ms > 0.5f)) return 1.0f;
    float k = set_ms * 3.6f / 70.0f;
    return k < 0.6f ? 0.6f : k > 1.4f ? 1.4f : k;
}
// The driving mode from the game's adaptive-shift cvar (one source for every module): <= 0 = M (manual hints), > 3.01 Eco, <= 1.67 Power
enum { MODE_NORMAL, MODE_ECO, MODE_POWER, MODE_M };
static inline int mode_of(float a)
{
    if (a != a) return MODE_NORMAL;
    if (a <= 0) return MODE_M;
    if (a > 3.01f) return MODE_ECO;
    if (a > 1.67f) return MODE_NORMAL;
    return MODE_POWER;
}
extern volatile int g_rsc_level;         // rollover guard: 0 off, 1 alert (tone, no throttle), 2 soft brake, 3 firm brake
extern float g_rsc_mode;                 // ini rollover_guard: 0 off, 1 tone only, 2 + no throttle, 3 + brake
extern float g_rsc_bend;                 // ini rollover_bend: 1 = also the single-bend thresholds (off: only the swing check)
void rsc_step(float dt);                // core/rsc.cpp, called by the actuator every tick; posts its brake wish and throttle cap (ACT_RSC)
// Read-only views of what the other modules decide, for the low-speed owner (modules/lowspeed.cpp). Nothing here changes a module.
struct creep_info {       // Creep (crawl and smooth start)
    int crawl;            // the crawl / maneuver mode is on (the controller drives the clutch floor)
    int launch;           // the smooth start (or launch control) holds the clutch
    float floor;          // the crawl's clutch floor, 0 closed .. 1 open
    float launch_clutch;  // the smooth start's clutch value (valid while launch)
    float launch_aim;     // the engine speed the smooth start aims for (rpm, valid while launch)
    float add_thr;        // the throttle the crawl adds for a heavy load (0 = none)
    float brake;          // the crawl's descent brake request
    float brake_off;      // ini [creep] brake: the brake pedal that takes the crawl away
    float target_kmh;     // the idle creep's target (km/h, valid while crawl): creep_kmh, lowered by the brake pedal (the pedal itself never sets it: a pressed pedal is the launch)
};
void creep_info_get(creep_info *o);
// Creep's part of the update's tail hook (WP-LS, LS3: the hook is installed once, by LowSpeed, which calls this first and then its own preselect). creep_wants_tail: the creep needs the tail (hold gear 1 / 2 at
// rest, engage from neutral); creep_tail_set: whether the one hook is in.
void creep_tail(void *self);
void creep_set_switch(int on);   // the pedals-free setting from outside (the sims; a wheel button later): 1 = creep, 0 = hold, -1 = read [creep] pedals_free (and the legacy start_on) again
int creep_wants_tail();
void creep_tail_set(int ok);
struct cruise_info {      // Cruise (Auto Hold, Stop&Go)
    int hold;             // Auto Hold: 0 off, 1 holds the standstill, 2 releasing for a launch
    float hold_req;       // the brake request Auto Hold holds now (0..1)
    float lever_take;     // Auto Hold: the request it would hold after a stop by the retarder lever now (0 = it would not take one)
    int sg_hold;          // Stop&Go holds the stop behind a lead
    float feed;           // the cruise throttle fed to the automatic's pedal at walking pace (0 = none)
};
void cruise_info_get(cruise_info *o);
// Cruise, read-only, for the planner shadow (modules/planner.cpp): what the cruise aims at now and the gap law it follows.
struct cruise_plan_info {
    float target_kmh;     // the cruise controller's target of its last tick (km/h; PCC, bends, junctions, ACC folded in), NAN while it does not drive
    int level;            // ACC distance level: -1 off, 0 short, 1 middle, 2 long
    float time_gap_s;     // the time gap of that level at the speed of now (s), NAN with ACC off
    float d0_m;           // the standstill gap the own ACC keeps at the speed of now (m)
    float want_m;         // the gap the own ACC wants at the lead's speed (its calm law: the level's time gap and standstill gap combined), NAN with ACC off
    float lead_m;         // the game's ACC lead gap (m), 0 = no lead
    float lead_ms;        // its speed from the radar (m/s), NAN = unknown (no radar hook, stale, or the radar's gap disagrees)
};
void cruise_plan_info_get(cruise_plan_info *o);
// Planner shadow (modules/planner.cpp, [planner] shadow=1): the longitudinal planner (core/planner.h) runs beside the cruise and only
// logs. Nothing here acts. planner_view_get() returns 0 (and on = 0) while it does not plan (shadow off, cruise off, under 10 km/h).
struct planner_view {
    int on;                    // a plan of the last second is in here
    double t;                  // sim_s() of that plan
    int n_plans;               // plans this session
    int ok;                    // the plan exists (0: no node reachable)
    int route;                 // 1 = the route profile, 0 = no route (flat at the grade of now)
    int mode;                  // PL_ECO .. PL_MANUAL (core/planner.h)
    int act;                   // what the plan does now: PL_GAS / PL_COAST / PL_BRAKE / PL_NEUTRAL
    int why;                   // PL_WHY_*
    int neutral_req;           // it would ask TrueShift for neutral (or to stay in it)
    int ret_level;             // retarder step it would use now (0 = none)
    int gear_pred;             // the gear its TrueShift stand-in predicts now (gearbox numbering, as T.gear)
    int late;                  // the start is above what planned braking meets
    float v_target_kmh;        // plan speed ~2 s ahead (the cruise PI's reference once it executes)
    float v_next_kmh;          // plan speed at stage 1
    float cruise_target_kmh;   // the cruise's own target at the same moment (NAN = none)
    float decel;               // decel wish now (m/s2, > 0 braking)
    float service_decel;       // its service-brake share (m/s2)
    float neutral_hold_s;      // committed neutral time left
    float v_overspeed_ok_kmh;  // the game's overspeed braking may act above this
    float v_throttle_cap_kmh;  // no throttle above this now
    float plan_g_km, base_g_km;   // the plan's and the plain cruise's fuel over the horizon (g per km, the fitted model's grams)
    float saved_g;             // running estimate this session: plain cruise minus plan over the distance driven while planning (g)
    float plan_ms, prof_ms;    // the last replan's time and the route profile's (ms)
    // for the dashboard's Planner tab (dev.cpp): the modes, the next action changes and a short profile of the plan
    int shadow, drive;         // [planner] shadow / drive as set now (filled by planner_view_get, also while there is no plan)
    int next_act[2];           // the next two changes of action along the plan (PL_*), -1 = none
    float next_at_m[2];        // the distance from now at which each begins (m)
    int n_prof;                // points in prof_kmh (0 = none)
    float prof_m;              // the plan's length (m): prof_kmh[i] is the planned speed at (i + 1) / n_prof of it
    float prof_kmh[24];
};
int planner_view_get(planner_view *o);
// Planner drive (modules/planner.cpp, [planner] drive=1): the cruise follows the plan. A read-only view for the cruise (nothing here
// steers): the planner module fills it each frame; it is zero while the planner does not drive. on = drive on AND the planner is in charge
// (cruise on, over 10 km/h, a plan was taken); fresh = that plan is ok and under ~1 s old (the cruise follows it then; else it falls back
// to its own blocks). v_* in m/s.
struct planner_drive {
    int on, fresh, ok;          // drive in charge / a fresh, ok plan to follow / the last plan exists
    float v_target_ms;          // plan speed ~2 s ahead (the cruise PI's reference)
    float v_next_ms;            // plan speed at stage 1
    float v_overspeed_ok_ms;    // the game's overspeed braking may act above this (the plan's max speed over the next 2 stages)
    float v_throttle_cap_ms;    // no throttle above this now (min(set, limit + 5))
    float force;                // the plan's wheel force now (N; feed-forward for the force-mode controller)
    float decel;                // the plan's decel wish now (m/s2, > 0 braking)
    int act;                    // PL_GAS / PL_COAST / PL_BRAKE / PL_NEUTRAL
    int why;                    // PL_WHY_*
};
inline planner_drive g_planner_drive;   // one instance across all modules (C++17 inline variable; zero-initialised while the planner is off)
int cruise_lane_engage(const char **why, float *set_kmh);   // Lane (needs_cruise, the lane-keeping key with the cruise off): switch the cruise on as Resume does with ISA (limit + offset; else the last set, else the speed); 0 = refused, why = the reason
int cruise_units_mph();            // 1 = the game shows miles per hour (g_mph): the cruise steps, the ISA offset and the dash follow; ETS2 and km/h: 0
float cruise_hold_kmh(float gp);   // how far over the set speed the game's cruise lets the truck run before its engine brake / retarder hold it (km/h), now (gp NAN) or on a grade of gp %; NAN = not known
int cruise_overspeed_view(float *from_kmh, float *target_kmh);   // where the game's own overspeed braking starts and the speed its cruise holds (km/h), as of this tick's Cruise hook; 0 = none now (cruise off, driver's throttle, overspeed setting off)
float cruise_wish_share();         // [cruise] force_mode 1: the cruise's ask as a share of a full torque-pedal wish, 0..1 (its last 0.3 s), else NAN (the feed's 'wish' while the cruise drives)
float cruise_wish_force(float v);  // [cruise] force_mode 1: the force (N) at speed v the cruise throttle asks (its last 0.3 s), else -1 (TrueShift reads it through wish_get)
float cruise_wish_ahead(float v);  // the same where the controller is heading (its output before the rate limit, never under the force it asks now), else -1: the gear for a pull is chosen for the pull (core/wish.cpp)
int cruise_overtake_on();          // [cruise] overtake_assist: the overtake assist is active now; TrueShift skips the cruise's force wish for its kickdown while this is set
// LowSpeed (modules/lowspeed.cpp): the owner of the stop, the standstill and the first metres. g_lv (the handshake below) holds what it WOULD command (with owner=1 it
// drives the stop and the standstill, through the accessors below; the rest stays a shadow).
// would_clutch 0 closed .. 1 open (NAN = the game's own), would_brake a request 0..1, would_thr the throttle wish (NAN = none).
enum LowState { LOW_GAME, LOW_HANDED, LOW_STOPPING, LOW_STOPPED_HELD, LOW_STOPPED_FREE, LOW_CRAWL, LOW_MANEUVER, LOW_LAUNCH, LOW_ROLLING_LOW,
                LOW_REVERSE, LOW_N };
// The one handshake of launch, crawl and standstill (WP-LS, LS2). It replaces g_lv.crawl_toggle, g_lv.lcx, g_lv.assist and g_low: one struct, written by Creep's and LowSpeed's frames (two halves of one owner), read by the rest.
// Readers say which meaning they need: crawl_toggle is the DRIVER'S SWITCH (Auto Hold stands aside, the stop rule keeps the clutch, the demand history does not count the crawl), crawl is the mode RUNNING now
// (the clutch floor is Creep's: the torque pedal and the cruise's pedal stage stand aside only then, promise P4), lcx the launch control phase (the pedal is the engine speed's).
struct low_view {
    // what LowSpeed would command (with owner=1 it drives the stop and the standstill, through the accessors below; the rest stays a shadow)
    int state = LOW_GAME;            // LowState
    int would_gear = 0;              // the gear it would have in (0 = none to ask for)
    float would_clutch = NAN, would_brake = 0, would_thr = NAN;   // clutch 0 closed .. 1 open (NAN = the game's own), a brake request 0..1, the throttle wish (NAN = none)
    double since_s = 0;              // seconds in this state
    const char *reason = "not started";   // why the state last changed
    // the laws' switches and states
    volatile int crawl_toggle = 0;   // the pedals-free setting is creep (the driver's switch: creep vs hold; was the crawl switch)
    volatile int crawl = 0;          // the idle creep is running now (pedals free, the creep setting)
    volatile int lcx = 0;            // launch control phase, 0 off, 1 armed (brake + flat pedal at a standstill), 2 launching (was g_lv.lcx)
    volatile int assist = 0;         // [lowspeed] owner=1: the box opened the clutch by itself with both pedals free and the stop is not finished (the stop assist starts on its edge; was g_lv.assist)
};
inline low_view g_lv;
const char *low_state_name(int state);
// With [lowspeed] owner=1 (and the owner able to work: not in GAME) the owner drives the stop and the standstill; the other modules ask it
// through these. All 0 / stock with owner=0 and in GAME.
int low_owner_wanted();          // [lowspeed] owner=1 in the ini (TrueShift installs its stop hook for it even with smooth_stop 0)
int low_owner_on();             // the owner has the stop and the standstill
int low_stopped();              // standing by the owner's one definition (< 0.3 km/h, measurement noise): the standstill states
int low_sg_zone();              // the engine is within reach of the stop threshold or the truck stands: Stop&Go's hold has to be set before the box can go to neutral
float low_stop_rpm(float idle, float thr, float brake, int gear);   // the stop threshold for gear (rpm): the clutch opens at the open point of the owner
// owner=1, the pull-away (step 3): the start gear at rest is the owner's one path, the gear is held until the clutch is locked, the hand-off to
// TrueShift is at clutch lock + 0.5 s. All 0 with owner=0 and in GAME.
int low_start_gear();            // the gear the truck starts in now (the physics pick, preselected at the stop); 0 = the owner has none to say
int low_launch_hold();           // a pull-away (LAUNCH) or the first half second of a locked clutch (ROLLING_LOW): TrueShift keeps the gear it has
int low_handed();                // HANDED: the owner has handed the truck to TrueShift
float low_top_kmh();             // the top of the owner's world (km/h), 0 = unknown: where the start gear reaches the end of its low-gear shift window
float low_drive_force();         // N at the wheels the clutch the owner commands carries now (a launch under the smooth start's law), NAN = no such launch
int ts_start_gear(float thr);   // TrueShift: the start gear by physics (mass, grade, engine) for this pedal, whether or not TrueShift has the truck; 0 = unknown
int ts_stop_hooked();            // TrueShift: the game's stop threshold hook is in (the owner's clutch-open point needs it)
float ts_idle_rpm();            // TrueShift: the idle rpm (learned at a standstill, else the ini's; not the game's engine figure Creep reads)
void ts_commanded(int gear);       // TrueShift: the owner changed the gear at a standstill (the shift is ours, not a hint of the driver)
float ts_brake_down_eta_hard(float decel);   // ... and for a deceleration the truck is about to have (the lever's retarder coming back after a shift's close): the braking rule's floor evaluated for the gear in when the rule has not looked
float ts_brake_down_eta(float decel);   // TrueShift: seconds until its next downshift in braking at this deceleration (m/s2, > 0), from the speed falling to the point the braking rules have set; NAN = none expected
// TrueShift for the dashboard (core/dev.cpp): what it does with the gears, read-only (nothing here steers). Gears in the gearbox's numbering
// (as T.gear, crawler gears counted). Returns 0 and an empty view while TrueShift does not drive this gearbox (off, manual box, unsupported,
// paused, no update for 0.5 s).
struct ts_view_t {
    int next;               // the gear it is shifting to, or forcing this update; 0 = none
    int shifting;           // 1 = a shift is under way (the box has a target that is not its gear, or its clutch is not closed again yet)
    int advice;             // M: the gear the automatic would take now when that is another one, the same for 1 s; 0 = none
    const char *why;        // the short reason of its last decision worth showing (a shift of its own, or a hold that matters); "" = none
    int why_from, why_to;   // the shift that reason belongs to (why_from 0 = a hold, in gear why_to)
    float why_age;          // seconds (simulation) since that decision; a hold that goes on stays at 0
    float up_rpm, dn_rpm;   // the shift points of the gear it is in now (rpm), 0 = not known
    float demand;           // the power demand the choice works with (0-1, filtered pedal or what drives the throttle)
};
int ts_view(ts_view_t *out);
int ts_mode();                  // TrueShift: the driving mode (MODE_ECO / MODE_NORMAL / MODE_POWER) of an automatic truck it can read; -1 = manual box, M, or an unsupported truck
// TrueShift: neutral on request (the longitudinal planner, modules/planner.cpp). ts_neutral_request every frame while the planner drives:
// want 1 = coast in neutral now, for about hold_s more (s); want 0 = stay in gear (also ends a roll TrueShift chose itself). While a request
// is fresh (refreshed within 0.3 s) it replaces TrueShift's own eco-roll economy; its safety rules stay a veto (M, any brake, the driver's
// pedal, a descent that would need the brakes, brake air, low speed, a shift; a junction or bend is the planner's to know, not TrueShift's). A roll ends when the request is
// withdrawn, expires or is vetoed: the exit gear goes in as after an eco-roll. Never called: TrueShift's eco-roll as before.
void ts_neutral_request(int want, float hold_s);
int ts_neutral_state();               // 1 while TrueShift rolls in neutral by choice (its own or a request's), else 0
const char *ts_neutral_refusal();     // why the fresh want-1 request of now is refused (a short static text), NULL if granted or none
// Pedal (modules/pedal.cpp): the torque pedal, the delivery of the demand in core/wish.h. The driver's pedal p becomes a demand; the throttle that delivers it in the gear
// the truck is in. Returns 1 and the throttle in *out where it applies, else 0 and *out = p (pass-through: the old pedal path).
// other = the caller knows something else drives the throttle (cruise, ACC, AEB). dt = the tick's seconds.
int pedal_wish(float p, float dt, int other, float *out);
extern volatile int g_pedal_mapped;     // Cruise: the throttle the engine gets is the torque pedal's (modules/pedal.cpp), not the game's own pedal, this frame
extern volatile int g_pcc_on;         // Cruise: predictive cruise (PCC) is switched on
extern volatile int g_cruise_smooth;  // the Cruise module drives the cruise throttle itself
float wheel_radius();    // powered wheels, meters (0 if unknown)

// ---------------------------------------------------------------- actuate.cpp
// The actuator: one layer moves the pedals for every module. A module posts a wish under its owner tag; the wish is held
// until the owner changes it (so a wish from frame_end lasts until the next frame); the layer merges all wishes once per
// game tick (the game's cruise fn call, hooked by core whatever the ini says) and writes them. The throttle is the base (the
// game's cruise output, or what Cruise posts from its own controller), then every floor, then every cap, then every scale.
// Brake wishes are in the game's request unit 0..1 (the top of actuate.cpp says why not m/s2); the brake channel (core/brake.h)
// takes decelerations and posts its service brake as ACT_CHAN.
enum { ACT_CRUISE, ACT_HILL, ACT_HOLD, ACT_BEND, ACT_ACC, ACT_AEB, ACT_RSC, ACT_CREEP, ACT_LCX, ACT_RET_DECEL, ACT_RET_NORM, ACT_DECEL, ACT_LOW, ACT_CHAN, ACT_N };   // ACT_CHAN: the brake channel's service brake (core/brake.cpp, [brake] channel 1)
struct act_wish { float brake, floor, cap, scale, ret; };  // brake request 0..1; throttle at least floor (0 = none), at most cap (1 = none), times scale (1 = none); retarder torque factor 0..1 (1 = none)
void act_brake(int owner, float want);
void act_thr_floor(int owner, float v);
void act_thr_cap(int owner, float v);
void act_thr_scale(int owner, float k);
void act_retarder_scale(int owner, float factor);   // the game's retarder torque times factor (0..1), held until the owner changes it; the lowest factor of all owners applies, none posted = 1 (needs [core] retarder_hook)
void act_thr_base(float v);      // this tick's base throttle in place of the game's cruise output (Cruise, from its ACT_POST hook only)
const act_wish *act_wishes();    // [ACT_N]: what every owner holds now (traces, sims)
// The merge of the throttle in one place (the tick in core/actuate.cpp applies it, the pedal's demand reads it): every cap (the smallest one), then
// every scale, in owner order. act_thr_apply(r) is the throttle after the caps and scales of what the owners hold now; act_thr_limit() is the most it
// can be (r = 1): the smallest cap times every scale, 1 = nothing holds the throttle under what is asked. Floors only raise, they are not a limit.
// Inline on act_wishes() so a sim that stands in for the actuator (its own act_wishes) needs nothing more.
static inline float act_thr_apply(float r)
{
    const act_wish *w = act_wishes();
    for (int o = 0; o < ACT_N; o++) if (w[o].cap < 1) r = fminf(r, w[o].cap);
    for (int o = 0; o < ACT_N; o++) if (w[o].scale != 1) r *= w[o].scale;
    return r;
}
static inline float act_thr_limit() { return act_thr_apply(1.0f); }
float act_request();             // the brake request we hold in game_actor+0x17c now (0 = none)
void act_air_exempt(int on);     // the brake channel is serving an urgent ask with the service brake: the low-air pause lets it through, like the AEB (the channel sets it every tick)
const char *act_owner_name(int owner);   // the owner's name (logs, the dev feed), "?" out of range
void act_brake_write(char *ga);  // merge the brake wishes and write the request, merge the retarder factors (the tick does it; sims)
// Per-tick hooks for a module that works inside the cruise fn call: ACT_PRE runs before the game's own cruise controller (its
// inputs), ACT_POST after it (stock = what it returned: post the wishes), ACT_DONE after the merge (thr = what the engine gets).
struct act_tick { char *ga; float dt, pedal, stock, thr; };
enum { ACT_PRE, ACT_POST, ACT_DONE };
void act_hook(int phase, void (*fn)(act_tick *));
int act_hooked();                // the throttle hook (the cruise fn call) is in
float act_game_pedal();          // the pedal argument of the game's cruise fn call, the last one seen (it runs every physics step, also with the engine off, when the telemetry's throttle channels read 0); NAN when none for 0.5 s
bool act_retarder_hooked();      // the drivetrain's retarder demand read is patched (the factors act on the game's retarder torque)
int act_gate_ok();               // the Auto Hold throttle gate is patched: the pedal reaches the engine while only the hold asks
// The clutch (1.60): our value replaces the drivetrain's copy of the player vehicle's clutch (0 = closed .. 1 = open) while inner is
// set; inner NULL = the game's own. mirror = also the vehicle's own clutch value (the smooth start). g_clutch_ours follows.
int act_clutch_ok();             // the drivetrain clutch hook is in
void act_clutch(char *inner, float v, int mirror);
void act_clutch_release();       // the game's own clutch until the owner posts again (the value and g_clutch_ours stay)
int act_clutch_vehicle(char *inner, float v);   // the vehicle's own clutch value, once; 1 = written
extern volatile float g_ret_scale;  // the merged retarder factor the drivetrain hook applies to the player's vehicle (1 = the game's own)
extern float g_retarder_hook;    // [core] retarder_hook: 1 = install the retarder hook
extern float g_air_min_psi;      // [cruise] air_min_psi: below it our brake requests pause (holds and the AEB excepted)
extern float g_air_from_truck;   // [core] air_from_truck: 1 = the brake air edges follow the truck's own warning pressure
// The brake air edges (psi). fixed = the ini value (0 = that rule off, it stays off). With [core] air_from_truck = 1 and a truck
// that reports its warning pressure (config brake.air.pressure.warning, above the emergency one) an edge is that warning plus
// a margin instead: our requests pause at +10 ([cruise] air_min_psi), the AEB saves air below +25, TrueShift keeps the rpm up
// to charge below +35 ([trueshift] air_charge_psi). A 65 psi warning gives the fixed 75 / 90 / 100. Not reported: fixed.
static const float AIR_MARGIN_PAUSE = 10, AIR_MARGIN_SAVE = 25, AIR_MARGIN_CHARGE = 35;
static inline int air_truck_ok() { float w = T.air_warn_cfg; return w > 1 && w < 150 && !(T.air_emerg_cfg >= w); }
static inline float air_edge_psi(float fixed, float margin)
{
    return g_air_from_truck >= 0.5f && fixed > 0 && air_truck_ok() ? T.air_warn_cfg + margin : fixed;
}
static inline float air_pause_psi() { return air_edge_psi(g_air_min_psi, AIR_MARGIN_PAUSE); }  // our brake requests pause below
void act_start();                // m_core (config.cpp): hooks before the modules start
void act_stop();
void act_reset(int level, const char *why);
const char *act_status();

// ---------------------------------------------------------------- display.cpp
// The dashboard letter: a module posts a wish (letter, priority, duration) under its owner tag, held until the owner changes it
// or its time runs out; one writer shows a flash over a status over the mode letter, the latest post within a priority.
enum { DISP_TS, DISP_CRUISE, DISP_LANE, DISP_CREEP, DISP_N };   // owners
enum { DISP_MODE, DISP_STATUS, DISP_FLASH, DISP_LEVELS };       // priorities, lowest first
int disp_post(int owner, int level, char c, double secs);       // c 0 = withdraw; secs > 0 = shown that long, 0 = until changed; 0 = refused (c not in disp_letters())
char disp_pick();                // the letter the writer would show now (0 = nobody posts)
int disp_write(void *gb, char *shown);   // write the picked letter (the mode owner calls it at its own rate): 1 done, 0 buffer not found, -1 nothing to show
const char *disp_letters();      // every letter that may be shown (game.cpp's check of the buffer uses it)
void disp_frame();               // m_core (config.cpp): the writer while no mode letter is posted ([core] dash_messages)
extern float g_dash_messages;    // [core] dash_messages: 1 = flashes and statuses also without a mode letter

// ---------------------------------------------------------------- optional local build extras
// A local build may add core/dev.cpp + core/dev.h (build.sh then defines DAZDRIVE_DEV). A release never has them: there
// DEV_INIT(), DEV_SECTION() and every DEV_NOTE(...) are empty and their arguments are not even compiled.
#ifdef DAZDRIVE_DEV
#include "dev.h"
#define DEV_INIT() dev_init()
#define DEV_SECTION(name) dev_section(name)
#define DEV_NOTE(call) do { if (dev_on()) dev_##call; } while (0)
#else
#define DEV_INIT() do { } while (0)
#define DEV_SECTION(name) 0
#define DEV_NOTE(call) do { } while (0)
#endif
