// Shared declarations for the suite: DazDrive, one in-game plugin DLL for ETS2/ATS with
// several modules (TrueShift now, ACC and lane keeping later).
// MIT license. The hook layer is derived from car_automatic_transmission
// (MIT, (c) 2026 Deppelover, github.com/deppelover/car_automatic_transmission).
#pragma once

#define SUITE_NAME "dazdrive"  // DazDrive: DLL, ini and output folder
#define SUITE_VERSION "0.6.0-beta.3.dev.1"

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
extern volatile float g_creep_brake;  // Creep: brake request it wants (crawl held on a descent; Cruise applies it)
extern volatile int g_crawl_on;       // Creep: the driver has crawl switched on
extern volatile int g_rolling;        // TrueShift has the truck in neutral (eco-roll)
extern volatile float g_roll_boost_kmh;   // TrueShift asks Cruise for this much over the set speed (km/h) just before an eco-roll starts (0 = none)
extern volatile double g_rb_cancel_at;   // Cruise: wall time it cancelled itself for a roundabout (Lane switches lane keeping off for it, back on at the next cruise resume)
extern volatile float g_creep_thr;     // Creep asks the merged throttle for this much (0..~0.15): Cruise's stub maxes it in
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
extern volatile float g_overspeed_cap;  // Cruise: ini overspeed_cap_kmh (0 = no cap)
// How many km/h the truck may still be allowed over the set speed so it stays under the speed limit + overspeed_cap_kmh (a fine
// starts a little above the limit). 1e9 = no cap (cap off or limit unknown). Used for every extra speed we grant above the set speed.
static inline float overspeed_room(float set_ms)
{
    if (!(g_overspeed_cap > 0) || !(T.speed_limit > 1.0f)) return 1e9f;
    float r = T.speed_limit * 3.6f + g_overspeed_cap - set_ms * 3.6f;
    return r > 0 ? r : 0;
}
extern volatile int g_pcc_on;         // Cruise: predictive cruise (PCC) is switched on
extern volatile int g_cruise_smooth;  // the Cruise module drives the cruise throttle itself
extern volatile char g_dash_flash;    // a letter to show on the dashboard for a moment (ACC C/N/F/O, limit S/X, lane L/X)
extern volatile char g_dash_status;   // a letter shown while something holds (B hill-start brake, K creep); 0 = none
extern double g_dash_flash_until;     // wall_s() until which it shows
float wheel_radius();    // powered wheels, meters (0 if unknown)

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
