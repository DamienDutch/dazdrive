// Shared declarations for the suite: DazDrive, one in-game plugin DLL for ETS2/ATS with
// several modules (TrueShift now, ACC and lane keeping later).
// MIT license. The hook layer is derived from car_automatic_transmission
// (MIT, (c) 2026 Deppelover, github.com/deppelover/car_automatic_transmission).
#pragma once

#define SUITE_NAME "dazdrive"  // DazDrive: DLL, ini and output folder
#define SUITE_VERSION "0.6.0-beta.4"

#include <windows.h>
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
    void (*config)(const scs_telemetry_configuration_t *c);   // SDK configuration event
    void (*stop)();                                           // telemetry shutdown
    // Optional (NULL = none). reset: something broke continuity (see RESET_*): clear timers, filters, rate limits
    // (and with RESET_HARD learned values and cached game pointers). status: one line for the log at start.
    void (*reset)(int level, const char *why);
    const char *(*status)();
};

enum { RESET_SOFT = 0,   // pause menu / long hitch / engine on: timers, filters, hold states
       RESET_LOAD = 1,   // trailer (de)coupled: mass dependent values
       RESET_HARD = 2 }; // other truck, save/load, teleport (ferry, train, quick travel): everything
void session_reset(int level, const char *why);  // telemetry.cpp: logs and calls every module's reset
extern const char *g_truck_unsupported;  // NULL, or why this truck is left to the game's own automatic (TrueShift sets it)


extern module *g_modules[];
extern int g_n_modules;
void config_load();  // reads SUITE_NAME.ini, writes it with defaults first if missing

// ---------------------------------------------------------------- telemetry.cpp

struct telemetry {
    // channels
    float rpm, in_thr, eff_thr, in_brake, eff_brake, in_clutch, eff_clutch, speed, cruise, speed_limit, air_psi;
    float in_steer;    // steering input -1..1, + = left (counterclockwise)
    float yaw;         // yaw rate, rotations per second, + = turning left
    int gear, dgear, motor_brake, parking_brake, engine_on, trailer, paused, air_warning;
    unsigned retarder, retarder_steps;
    scs_value_dplacement_t place;
    scs_value_fvector_t acc;
    scs_timestamp_t sim_time;
    // truck configuration
    int fwd_gears, rev_gears, wheels;
    float ratio[32], diff, rpm_limit;
    float wheel_radius[16];
    int wheel_powered[16];
    char shifter[32];
    float cargo_mass;
    // derived
    float accel;       // longitudinal acceleration from speed, m/s2, filtered
};
extern telemetry T;
void telemetry_register(const scs_telemetry_init_params_v101_t *p);
void telemetry_frame();  // derived values, called first in frame_end

// ---------------------------------------------------------------- shared between modules

// sound.cpp: feedback sounds, async, one at a time; g_volume 0..1 (0 = silent)
enum Cue { CUE_MODE_ECO, CUE_MODE_NORMAL, CUE_MODE_POWER, CUE_MODE_M, CUE_ACC_1, CUE_ACC_2, CUE_ACC_3, CUE_ACC_OFF,
           CUE_ON, CUE_OFF, CUE_AEB_WARN, CUE_AEB_PARTIAL, CUE_AEB_FULL, CUE_CRUISE_DROP,
           CUE_LANE_PAUSE, CUE_LANE_RESUME, CUE_OVERSPEED, CUE_COUNT };
void play_cue(int cue);
// input.cpp: finds the wheel (the present joystick with the most buttons) for lane's per-wheel profile
void input_poll();
const char *input_wheel_name();  // winmm name of the most button-rich present joystick ("" = none found yet)
extern float g_volume;
extern volatile int g_crawl_on;       // Creep: the driver has crawl switched on
extern volatile int g_rolling;        // TrueShift has the truck in neutral (eco-roll)
extern volatile float g_roll_boost_kmh;   // TrueShift asks Cruise for this much over the set speed (km/h) just before an eco-roll starts (0 = none)
extern volatile double g_rb_cancel_at;   // Cruise: wall time it cancelled itself for a roundabout (Lane switches lane keeping off for it, back on at the next cruise resume)
// Lane: what the lane assist is doing right now, one value (lane.cpp sets it where its gates decide; the dev display reads it).
// OFF = lane keeping off (the game does not call the law); WAIT_* = on, but no request of ours: under min_kmh, the cruise is off
// (needs_cruise), an indicator pause, no lane seen; ACTIVE = steering; RUNAWAY = the runaway guard handed the game's own law the wheel
enum LaneState { LANE_OFF, LANE_WAIT_SLOW, LANE_WAIT_NO_CRUISE, LANE_WAIT_INDICATOR, LANE_WAIT_NO_LANE, LANE_ACTIVE, LANE_RUNAWAY };
extern volatile float g_clutch_ours;  // the clutch value Creep puts on the drivetrain (launch or crawl), NAN when the game's own applies
static inline float clutch_seen() { float c = g_clutch_ours; return c == c ? c : T.eff_clutch; }  // what the drivetrain sees: use instead of T.eff_clutch for 'is the clutch closed'
extern volatile float g_pcc_need_kmh;  // TrueShift: extra speed over the set speed the next climb on the route needs (NAN = unknown)
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
extern volatile float g_overspeed_cap;  // Cruise: ini overspeed_cap_kmh (0 = no cap)
// How many km/h the truck may still be allowed over the set speed so it stays under the speed limit + overspeed_cap_kmh (a fine
// starts a little above the limit). 1e9 = no cap (cap off or limit unknown). Used for every extra speed we grant above the set speed.
static inline float overspeed_room(float set_ms)
{
    if (!(g_overspeed_cap > 0) || !(T.speed_limit > 1.0f)) return 1e9f;
    float r = T.speed_limit * 3.6f + g_overspeed_cap - set_ms * 3.6f;
    return r > 0 ? r : 0;
}
extern volatile int g_rsc_level;         // rollover guard: 0 off, 1 alert (tone, no throttle), 2 soft brake, 3 firm brake
extern float g_rsc_mode;                 // ini rollover_guard: 0 off, 1 tone only, 2 + no throttle, 3 + brake
extern float g_rsc_bend;                 // ini rollover_bend: 1 = also the single-bend thresholds (off: only the swing check)
void rsc_step(float dt);                // core/rsc.cpp, called by the actuator every tick; posts its brake wish and throttle cap (ACT_RSC)
extern volatile int g_lcx;               // Creep: launch control phase, 0 off, 1 armed (brake + flat pedal at a standstill), 2 launching
// Read-only views of what the other modules decide, for the low-speed owner (modules/lowspeed.cpp). Nothing here changes a module.
struct creep_info {       // Creep (crawl and smooth start)
    int crawl;            // the crawl / maneuver mode is on (the controller drives the clutch floor)
    int launch;           // the smooth start (or launch control) holds the clutch
    float floor;          // the crawl's clutch floor, 0 closed .. 1 open
    float launch_clutch;  // the smooth start's clutch value (valid while launch)
    float add_thr;        // the throttle the crawl adds for a heavy load (0 = none)
    float brake;          // the crawl's descent brake request
    float brake_off;      // ini [creep] brake: the brake pedal that takes the crawl away
};
void creep_info_get(creep_info *o);
struct cruise_info {      // Cruise (Auto Hold, Stop&Go)
    int hold;             // Auto Hold: 0 off, 1 holds the standstill, 2 releasing for a launch
    float hold_req;       // the brake request Auto Hold holds now (0..1)
    float lever_take;     // Auto Hold: the request it would hold after a stop by the retarder lever now (0 = it would not take one)
    int sg_hold;          // Stop&Go holds the stop behind a lead
    float feed;           // the cruise throttle fed to the automatic's pedal at walking pace (0 = none)
};
void cruise_info_get(cruise_info *o);
float cruise_wish_force(float v);  // [cruise] force_mode 1: the force (N) at speed v the cruise throttle asks (its last 0.3 s), else -1 (TrueShift reads it as pedal_wish_force)
// LowSpeed (modules/lowspeed.cpp): the owner of the stop, the standstill and the first metres. g_low is what it WOULD command (with owner=1 it
// drives the stop and the standstill, through the accessors below; the rest stays a shadow).
// would_clutch 0 closed .. 1 open (NAN = the game's own), would_brake a request 0..1, would_thr the throttle wish (NAN = none).
enum LowState { LOW_GAME, LOW_HANDED, LOW_STOPPING, LOW_STOPPED_HELD, LOW_STOPPED_FREE, LOW_CRAWL, LOW_MANEUVER, LOW_LAUNCH, LOW_ROLLING_LOW,
                LOW_REVERSE, LOW_N };
struct low_owner {
    int state;               // LowState
    int would_gear;          // the gear it would have in (0 = none to ask for)
    float would_clutch, would_brake, would_thr;
    double since_s;          // seconds in this state
    const char *reason;      // why the state last changed
};
extern low_owner g_low;
const char *low_state_name(int state);
// With [lowspeed] owner=1 (and the owner able to work: not in GAME) the owner drives the stop and the standstill; the other modules ask it
// through these. All 0 / stock with owner=0 and in GAME.
int low_owner_wanted();          // [lowspeed] owner=1 in the ini (TrueShift installs its stop hook for it even with smooth_stop 0)
int low_owner_on();             // the owner has the stop and the standstill
int low_stopped();              // standing by the owner's one definition (< 0.3 km/h, measurement noise): the standstill states
int low_sg_zone();              // the engine is within reach of the stop threshold or the truck stands: Stop&Go's hold has to be set before the box can go to neutral
float low_stop_rpm(float idle, float thr, float brake, int gear);   // the stop threshold for gear (rpm): the clutch opens at the open point of the owner
int ts_start_gear(float thr);   // TrueShift: the start gear by physics (mass, grade, engine) for this pedal, whether or not TrueShift has the truck; 0 = unknown
int ts_stop_hooked();            // TrueShift: the game's stop threshold hook is in (the owner's clutch-open point needs it)
float ts_idle_rpm();            // TrueShift: the idle rpm (learned at a standstill, else the ini's; not the game's engine figure Creep reads)
float ts_brake_down_eta(float decel);   // TrueShift: seconds until its next downshift in braking at this deceleration (m/s2, > 0), from the speed falling to the point the braking rules have set; NAN = none expected
int ts_mode();                  // TrueShift: the driving mode (MODE_ECO / MODE_NORMAL / MODE_POWER) of an automatic truck it can read; -1 = manual box, M, or an unsupported truck
// Pedal (modules/pedal.cpp): the wish pedal. The driver's pedal p becomes a wish for power; the throttle that delivers it in the gear
// the truck is in. Returns 1 and the throttle in *out where it applies, else 0 and *out = p (pass-through: the old pedal path).
// other = the caller knows something else drives the throttle (cruise, ACC, AEB). dt = the tick's seconds.
int pedal_wish(float p, float dt, int other, float *out);
float pedal_wish_force(float v);   // the driver's wish as a force (N) at speed v while the wish pedal gives the throttle (its last 0.3 s), else -1
float pedal_wish_full(float v);    // the force (N) a full wish asks at speed v (the reference above, on or off); 0 = the vehicle model cannot tell
extern volatile int g_pedal_mapped;     // Cruise: the throttle the engine gets is the pedal through the Eco / Power curve this frame
extern volatile int g_pcc_on;         // Cruise: predictive cruise (PCC) is switched on
extern volatile int g_cruise_smooth;  // the Cruise module drives the cruise throttle itself
float wheel_radius();    // powered wheels, meters (0 if unknown)

// ---------------------------------------------------------------- actuate.cpp
// The actuator: one layer moves the pedals for every module. A module posts a wish under its owner tag; the wish is held
// until the owner changes it (so a wish from frame_end lasts until the next frame); the layer merges all wishes once per
// game tick (the game's cruise fn call, hooked by core whatever the ini says) and writes them. The throttle is the base (the
// game's cruise output, or what Cruise posts from its own controller), then every floor, then every cap, then every scale.
// Brake wishes are in the game's request unit 0..1 (the top of actuate.cpp says why not m/s2).
enum { ACT_CRUISE, ACT_HILL, ACT_HOLD, ACT_BEND, ACT_ACC, ACT_AEB, ACT_RSC, ACT_CREEP, ACT_LCX, ACT_RET_DECEL, ACT_RET_NORM, ACT_DECEL, ACT_LOW, ACT_N };
struct act_wish { float brake, floor, cap, scale, ret; };  // brake request 0..1; throttle at least floor (0 = none), at most cap (1 = none), times scale (1 = none); retarder torque factor 0..1 (1 = none)
void act_brake(int owner, float want);
void act_thr_floor(int owner, float v);
void act_thr_cap(int owner, float v);
void act_thr_scale(int owner, float k);
void act_retarder_scale(int owner, float factor);   // the game's retarder torque times factor (0..1), held until the owner changes it; the lowest factor of all owners applies, none posted = 1 (needs [core] retarder_hook)
void act_thr_base(float v);      // this tick's base throttle in place of the game's cruise output (Cruise, from its ACT_POST hook only)
const act_wish *act_wishes();    // [ACT_N]: what every owner holds now (traces, sims)
float act_request();             // the brake request we hold in game_actor+0x17c now (0 = none)
void act_brake_write(char *ga);  // merge the brake wishes and write the request, merge the retarder factors (the tick does it; sims)
// Per-tick hooks for a module that works inside the cruise fn call: ACT_PRE runs before the game's own cruise controller (its
// inputs), ACT_POST after it (stock = what it returned: post the wishes), ACT_DONE after the merge (thr = what the engine gets).
struct act_tick { char *ga; float dt, pedal, stock, thr; };
enum { ACT_PRE, ACT_POST, ACT_DONE };
void act_hook(int phase, void (*fn)(act_tick *));
int act_hooked();                // the throttle hook (the cruise fn call) is in
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
