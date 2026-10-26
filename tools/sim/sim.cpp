// Offline test of TrueShift's decisions (modules/trueshift.cpp) on a simple
// longitudinal truck model. Emulates the game's automatic: per tick the decide
// stub, the stock window trigger, the chooser (a rough stock chooser below the
// takeover speed), a shift with a torque gap.
// Build+run: tools/sim/run.sh [-v]   (-v prints every log line)
#include "core/suite.h"
#include "core/game.h"
#include "core/route.h"
#include "core/vehicle.h"
#include "core/gearscore.h"
#include <math.h>
#include <initializer_list>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

telemetry T;
volatile int g_rolling, g_cruise_smooth, g_crawl_on;
volatile float g_roll_boost_kmh;
volatile double g_rb_cancel_at = -1e9;
volatile int g_pcc_on;
volatile int g_pedal_mapped;
volatile int g_lcx;
volatile float g_overspeed_cap = 5;
volatile float g_pcc_need_kmh = NAN;
int g_log_level = 2;
incident_ctx g_inc = { 0, 0, 0, NAN, NAN };
double g_incident_mb = 0;
int g_n_incidents;
static int g_quiet;   // the SCORES table runs print no shift lines and no incidents
void logring(const char *, ...) {}
void incident(const char *kind, const char *fmt, ...)
{
    g_n_incidents++;
    char b[300]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    if (!g_quiet) printf("   INCIDENT %s: %s\n", kind, b);
}
const char *g_truck_unsupported;
volatile float g_clutch_ours = __builtin_nanf("");
volatile char g_dash_flash, g_dash_status;
double g_dash_flash_until;
void play_cue(int) {}
void input_poll() {}
int input_pressed(int, int) { return 0; }
static int sim_btn;   // a wheel button held down in the sim (0 = none)
int input_held(int b) { return sim_btn && b == sim_btn; }
char g_dir[MAX_PATH];
const layout *L;
game_addrs G;
static double t_now;
double wall_s() { return t_now; }
double sim_s() { return t_now; }
void csv_cap(FILE **, const char *) {}
FILE *open_out(const char *name, const char *mode) { return NULL; }  // no trace file
static int verbose, n_failed, g_x_failed;
static float g_stuck_s = -1;   // the last 'stuck in neutral for N s' line (tail_pre's gap-state timer)
void logline(const char *fmt, ...)
{
    if (strstr(fmt, "no pull after")) n_failed++;
    if (!strncmp(fmt, "stuck in neutral for", 20)) { va_list ap; va_start(ap, fmt); g_stuck_s = (float)va_arg(ap, double); va_end(ap); }
    int shift = !strncmp(fmt, "shift", 5) || strstr(fmt, "no pull after") || !strncmp(fmt, "load changed", 12);
    if ((!verbose && !shift) || g_quiet) return;
    va_list ap; va_start(ap, fmt);
    printf("%7.1f  ", t_now); vprintf(fmt, ap); printf("\n");
    va_end(ap);
}
float wheel_radius();
extern module m_trueshift;

// ---- truck model (Volvo FH 2400 Nm from probe runs)
static float RATIO[16] = { 11.73f, 9.21f, 7.09f, 5.57f, 4.35f, 3.41f, 2.7f, 2.12f, 1.63f, 1.28f, 1.0f, 0.78f };
static float CURVE[8][2] = { {0,0},{600,0.62f},{900,1},{1230,1},{1600,0.77f},{1700,0.69f},{1800,0.44f},{2000,0} };
static float CDA = 6, TORQUE = 2400, LIMIT = 1900, DIFF = 2.85f, RADIUS = 0.506f, RANGES[4][2] = { {1500,1900},{750,1450},{900,1250},{1200,1650} };
static int NG = 12;
static void use_renault()  // Renault T, Optidriver Xtended AT02614F (12+2), DE13 TC 480: 2700 Nm, diff 3.40 (24-10 ride)
{
    float r[16] = { 32.04f, 17.2f };
    for (int k = 0; k < 12; k++) r[2 + k] = 11.73f * powf(0.79f / 11.73f, k / 11.0f);
    float c[8][2] = { {0,0},{700,0.60f},{900,0.97f},{1250,1},{1500,0.9f},{1700,0.72f},{1900,0.45f},{2100,0} };
    memcpy(RATIO, r, sizeof r); memcpy(CURVE, c, sizeof c);
    float g[4][2] = { {1500,1900},{750,1450},{900,1250},{1200,1600} };
    memcpy(RANGES, g, sizeof g);
    TORQUE = 2700; CDA = 6; LIMIT = 1900; DIFF = 3.40f; RADIUS = 0.538f; NG = 14;
    T.fwd_gears = 14; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
}
static void use_ato()  // Renault T, Optidriver ATO2612F R (12 speeds, no crawl gears), the same DE13 TC 480, diff 3.08 (25-10)
{
    float r[16];
    for (int k = 0; k < 12; k++) r[k] = 11.73f * powf(0.79f / 11.73f, k / 11.0f);
    float c[8][2] = { {0,0},{700,0.60f},{900,0.97f},{1250,1},{1500,0.9f},{1700,0.72f},{1900,0.45f},{2100,0} };
    memcpy(RATIO, r, sizeof r); memcpy(CURVE, c, sizeof c);
    float g[4][2] = { {1500,1900},{750,1450},{900,1250},{1200,1600} };
    memcpy(RANGES, g, sizeof g);
    TORQUE = 2700; CDA = 6; LIMIT = 1900; DIFF = 3.08f; RADIUS = 0.538f; NG = 12;
    T.fwd_gears = 12; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
}
static float RATIO0[16], CURVE0[8][2], RANGES0[4][2], CDA0, TORQUE0, LIMIT0, DIFF0, RADIUS0; static int NG0;   // the Volvo as the file starts
static void save_volvo()
{
    memcpy(RATIO0, RATIO, sizeof RATIO); memcpy(CURVE0, CURVE, sizeof CURVE); memcpy(RANGES0, RANGES, sizeof RANGES);
    CDA0 = CDA; TORQUE0 = TORQUE; LIMIT0 = LIMIT; DIFF0 = DIFF; RADIUS0 = RADIUS; NG0 = NG;
}
static void use_volvo()
{
    memcpy(RATIO, RATIO0, sizeof RATIO); memcpy(CURVE, CURVE0, sizeof CURVE); memcpy(RANGES, RANGES0, sizeof RANGES);
    CDA = CDA0; TORQUE = TORQUE0; LIMIT = LIMIT0; DIFF = DIFF0; RADIUS = RADIUS0; NG = NG0;
    T.fwd_gears = NG; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
}
static void use_car()  // a pickup-like car: 6 speeds, 450 Nm
{
    float r[16] = { 4.17f, 2.34f, 1.52f, 1.14f, 0.87f, 0.69f };
    float c[8][2] = { {0,0},{700,0.55f},{1500,0.9f},{2000,1},{4000,1},{5000,0.85f},{5800,0.6f},{6200,0} };
    memcpy(RATIO, r, sizeof r); memcpy(CURVE, c, sizeof c);
    float g[4][2] = { {3000,5000},{1500,3000},{1800,2600},{3000,5000} };
    memcpy(RANGES, g, sizeof g);
    TORQUE = 450; CDA = 1.0f; LIMIT = 6000; DIFF = 3.55f; RADIUS = 0.39f; NG = 6;
    T.fwd_gears = 6; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
}
static struct {
    int gear, target, state, parked, engaged; double shift_end; float v, mass, grade_pct, pedal, cc, brake; int force, ebrake, ret;
    float cvar, shift_time, cruise_kmh; int eb_toggle; float eb_level, mass_seen, thr_floor; int hint;   // hint: a shift-hint press the game's handler turns into target +-1 before its update
    double s;                 // distance driven (m)
    float (*road)(double s);  // grade (%) along the road, or NULL (scenario sets grade)
    int route_on;
    double junction_s;        // a junction at this distance (0 = none)
} M;

// ---- the route API on the sim road (heights integrated from the grade)
static double h_ahead(float d)
{
    double h = 0;
    for (float x = 0; x < d; x += 5) h += M.road(M.s + x + 2.5) / 100 * fminf(5, d - x);
    return h;
}
void route_update() {}
int route_ok() { return M.route_on && M.road; }
float route_ahead_m() { return route_ok() ? 1575 : 0; }
float route_grade(float d0, float d1)
{
    if (!route_ok() || d1 <= d0) return NAN;
    return (float)((h_ahead(d1) - h_ahead(d0)) / (d1 - d0) * 100);
}
static float ext_grade(float d0, float d1, int mx)
{
    if (!route_ok()) return NAN;
    float best = NAN;
    for (float d = d0; d + 50 <= d1 && d + 50 <= 1575; d += 25) {
        float g = route_grade(d, d + 50);
        if (best != best || (mx ? g > best : g < best)) best = g;
    }
    return best;
}
float route_max_grade(float d0, float d1) { return ext_grade(d0, d1, 1); }
float route_min_grade(float d0, float d1) { return ext_grade(d0, d1, 0); }
float route_crest_m(float max_d)
{
    if (!route_ok() || route_grade(0, 50) < 0.5f) return -1;
    for (float d = 25; d + 50 <= fminf(max_d, 1575); d += 25)
        if (route_grade(d, d + 50) < 0.2f) return d + 25;
    return -1;
}
// a junction on the route is a 30 km/h bend (only when the scenario has one)
float route_curve_speed(float, float a_dec, float)
{
    if (!route_ok() || !(M.junction_s > M.s)) return NAN;
    float d = (float)(M.junction_s - M.s), vj = 30 / 3.6f;
    return sqrtf(vj * vj + 2 * a_dec * d);
}
float route_junction_m() { return route_ok() && M.junction_s > M.s ? (float)(M.junction_s - M.s) : -1; }
float route_bend_speed(float, float) { return NAN; }   // the scenarios have no road bends, only the junction above
static void *SELF = (void *)&M;
static char letter = 'A';

char *gb_inner(void *) { return (char *)&M; }
int gb_gear(void *) { return M.gear; }
int gb_target(void *) { return M.target; }
int gb_state(void *) { return M.parked ? 1 : M.state; }
int gb_engage(void *, int g) { if (!M.parked) return 0; M.target = g; M.engaged = 1; M.parked = 0; return 1; }
int game_engage_init() { return 1; }
void gb_force(void *) { M.force = 1; }
float gb_thr_in(void *) { return M.pedal; }
float gb_thr_eff(void *) { return fmaxf(fmaxf(M.pedal, M.cc), M.thr_floor); }  // thr_floor: the game's 0.08 at pedal 0 (24-10, 1831 s)
float wheel_radius() { return RADIUS; }
int gb_engine(void *, engine_info *e)
{
    e->torque = TORQUE; e->rpm_limit = LIMIT; e->n_curve = 8;
    memcpy(e->curve, CURVE, sizeof CURVE);
    memcpy(e->range, RANGES, sizeof RANGES);
    e->shift_time = M.shift_time;
    return 1;
}
float game_mass() { return M.mass_seen > 0 ? M.mass_seen : M.mass; }  // mass_seen: a wrong estimate
float game_grade_deg(void *) { return atanf(M.grade_pct / 100) * 57.2958f; }
float game_adaptive() { return M.cvar; }
int game_adaptive_blocked() { return 0; }
int game_letter(void *, char c) { letter = c; return 1; }
void *game_gearbox() { return (void *)&M; }
int game_ebrake_downshift(void *) { return M.eb_toggle; }
float game_ebrake_level(void *) { return M.ebrake ? M.eb_level : (M.eb_level >= 0 ? 0 : -1); }
float curve_factor(const engine_info *e, float rpm)
{
    if (rpm <= e->curve[0][0]) return e->curve[0][1];
    for (int i = 1; i < e->n_curve; i++)
        if (rpm <= e->curve[i][0])
            return e->curve[i-1][1] + (rpm - e->curve[i-1][0]) / (e->curve[i][0] - e->curve[i-1][0]) * (e->curve[i][1] - e->curve[i-1][1]);
    return e->curve[e->n_curve-1][1];
}

static float rpm_now() { return M.gear > 0 ? M.v / RADIUS * RATIO[M.gear-1] * DIFF * 9.5493f : (TORQUE < 1000 ? 750 : 600); }

// hooks: capture the stubs
static window_fn s_decide; static gear_fn s_choose, s_start; static void (*s_tail)(void *);
static float *f_window(void *, float *out, unsigned g)
{
    float k = (g - 1) / (float)(NG - 1);
    out[0] = RANGES[1][0] + k * (RANGES[2][0] - RANGES[1][0]); out[1] = RANGES[1][1] + k * (RANGES[2][1] - RANGES[1][1]);
    return out;
}
// Rough stock chooser: one gear toward the window (two up when light).
static int f_choose(void *)
{
    M.force = 0;
    if (M.gear == 0) {  // from neutral at speed: the gear nearest 1100 rpm (a guess at the game)
        int best = 1; float be = 1e9f;
        for (int g = 1; g <= NG; g++) { float r = M.v / RADIUS * RATIO[g-1] * DIFF * 9.5493f; if (fabsf(r - 1100) < be) { be = fabsf(r - 1100); best = g; } }
        return best;
    }
    float w[2], r = rpm_now();
    f_window(0, w, M.gear);
    if (r > w[1] && M.gear < NG) return M.gear + (M.pedal < 0.6f && M.gear < NG - 1 ? 2 : 1);
    if (r < w[0] && M.gear > 1) return M.gear - 1;
    return M.gear;
}
static int f_start(void *) { return M.mass > 20000 ? 1 : 4; }
int game_patch_call(uintptr_t site, void *stub)
{
    if (site == 1) s_decide = (window_fn)stub;
    if (site == 2) s_choose = (gear_fn)stub;
    if (site == 4) s_start = (gear_fn)stub;
    return 1;
}
int game_patch_tail(uintptr_t site, void *pre) { s_tail = (void (*)(void *))pre; return 1; }
int game_patch_stop(uintptr_t, float (*)(float, float, float)) { return 0; }
void gb_set_target(void *, int g) { M.target = g; }
void game_cvar_set(void *, float v) { M.cvar = v; }

module *g_modules[] = { &m_trueshift };
int g_n_modules = 1;

void telemetry_frame()
{
    static double last_t; static float last_v;
    double dt = t_now - last_t;
    if (dt > 0.0 && dt < 0.5) { float a = (float)((fabsf(T.speed) - last_v) / dt); float k = (float)(dt / (0.5 + dt)); T.accel += k * (a - T.accel); }
    last_t = t_now; last_v = fabsf(T.speed);
}

// ---- run statistics
static int shifts, ups, hunts, last_dir; static double last_shift_t;
static double up_rpm_sum; static int up_n;
static double up_rpm_min, rpm_all_sum, rpm_all_t, skip_rpm_min, skip_pull_min; static int skips_up, hunts_ts, shifts_ts;   // RELAX study: landing rpm, mean rpm below 50 km/h, skips, and shifts/hunting above the takeover speed
static double rpm_sum, rpm_t;   // time-weighted rpm in the second half
static double g_sh_t, g_rpm_s, g_rpm_t; static int g_dskips;   // SCORES table: seconds in shifts, rpm in gear over the whole run, downward skips; g_quiet = run() prints nothing

// A session reset as core/telemetry.cpp session_reset does it: the vehicle model first, then the module.
static void sim_reset(int level, const char *why) { vehicle_reset(level); if (m_trueshift.reset) m_trueshift.reset(level, why); }

static int g_rst_every = 0, g_rst_level = 0;   // robustness runs: inject a session reset every N seconds
static void tick(double dt, int stats)
{
    t_now += dt;
    if (g_rst_every > 0 && fmod(t_now, g_rst_every) < dt) sim_reset(g_rst_level, "sim");
    T.sim_time = (scs_timestamp_t)(t_now * 1e6);
    float rpm = rpm_now();
    float idle = TORQUE < 1000 ? 750 : 600;
    if (rpm < idle) rpm = idle;   // clutch slips below idle
    T.rpm = rpm; T.speed = M.v; T.in_thr = M.pedal; T.eff_thr = fmaxf(M.pedal, M.cc);
    T.in_brake = M.brake; T.eff_brake = M.brake; T.cruise = M.cruise_kmh / 3.6f;
    T.motor_brake = M.ebrake; T.retarder = M.ret; T.eff_clutch = M.state ? 1 : 0;
    T.engine_on = 1; T.gear = M.gear; T.dgear = M.gear;
    if (M.road) M.grade_pct = M.road(M.s);
    telemetry_frame();
    vehicle_frame();
    m_trueshift.frame();
    // gearbox
    if (M.state && t_now >= M.shift_end) {
        int dir = M.target > M.gear ? 1 : -1;
        if (dir > 0 && M.gear > 0) { ups++; up_rpm_sum += rpm_now(); up_n++; if (M.target > M.gear + 1) skips_up++;
            float lr = M.v / RADIUS * RATIO[M.target-1] * DIFF * 9.5493f; if (lr < up_rpm_min && M.v * 3.6f >= 8) up_rpm_min = lr;
            if (M.target > M.gear + 1 && M.v * 3.6f >= 8) {   // a skip of ours: landing rpm and the pull right after it at the pedal of now
                engine_info e; gb_engine(0, &e);
                float F = TORQUE * curve_factor(&e, lr) * fmaxf(M.pedal, M.cc) * RATIO[M.target-1] * DIFF * 0.9f / RADIUS;
                float Fr = M.mass * 9.81f * (sinf(atanf(M.grade_pct / 100)) + 0.006f) + 0.5f * 1.2f * CDA * M.v * M.v;
                if (lr < skip_rpm_min) skip_rpm_min = lr;
                if ((F - Fr) / M.mass < skip_pull_min) skip_pull_min = (F - Fr) / M.mass;
            } }
        if (dir < 0 && M.gear > 0 && M.gear - M.target > 1) g_dskips++;
        if (M.v * 3.6f >= 8) { shifts_ts++; if (last_dir && dir != last_dir && t_now - last_shift_t < 5) hunts_ts++; }
        if (last_dir && dir != last_dir && t_now - last_shift_t < 5) hunts++;
        last_dir = dir; last_shift_t = t_now;
        M.gear = M.target; M.state = 0; shifts++;
    }
    // The game's update: in gear the decide site + chooser; in neutral (game,
    // lka-roll-re notes B) re-engage on throttle via the start-gear fn; then
    // the tail (our eco-roll hook) right before the gearbox.
    if (!M.state) {
        M.target = M.gear;
        // a hint press (the exe: target +-1 in the handler, before the update); the chooser below overwrites it, as the game's call site does
        if (M.hint && M.gear > 0) { M.target = M.gear + M.hint; if (M.target < 1) M.target = 1; if (M.target > NG) M.target = NG; }
        M.hint = 0;
        if (M.gear > 0) {
            float w[2];
            s_decide(SELF, w, M.gear);
            int stock_trigger = rpm < w[0] || rpm > w[1];
            if (M.force || stock_trigger) { M.target = s_choose(SELF); M.force = 0; }
        } else if (M.pedal > 0.01f) M.target = s_start(SELF);
        M.engaged = 0;
        if (s_tail) s_tail(SELF);
        // The generic gearbox parks in its gap state with gear 0 and a target
        // until there is throttle (neutral-exit-re notes § 1); gb_engage leaves it.
        if (M.gear == 0 && !(M.pedal > 0.01f) && !M.engaged) { M.parked = M.target > 0; M.target = 0; }
        else M.parked = 0;
        if (M.target != M.gear) { M.state = 1; M.shift_end = t_now + M.shift_time; }
    }
    // physics
    float F = 0, thr = fmaxf(M.pedal, M.cc);
    if (!M.state && M.gear > 0) {
        float r = rpm_now();
        engine_info e; gb_engine(0, &e);
        float f = r > LIMIT ? 0 : curve_factor(&e, r < idle ? idle : r);
        F = TORQUE * f * thr * RATIO[M.gear-1] * DIFF * 0.9f / RADIUS;
        if (M.ebrake && thr < 0.05f) F -= (r / LIMIT) * 150000 / (M.v + 1) * (M.eb_level >= 0 ? M.eb_level : 0.5f);
        if (thr < 0.05f) F -= TORQUE * 0.12f * RATIO[M.gear-1] * DIFF / RADIUS * (r / LIMIT);  // engine drag in gear
    }
    if (M.ret && thr < 0.05f) F -= M.ret / 5.0f * 25000 * fminf(1, M.v / 10);
    float s = sinf(atanf(M.grade_pct / 100));
    float Fres = M.mass * 9.81f * (s + 0.006f) + 0.5f * 1.2f * CDA * M.v * M.v + M.brake * M.mass * 4;
    M.v += (F - Fres) / M.mass * dt;
    if (M.v < 0) M.v = 0;
    M.s += M.v * dt;
    if (stats) { rpm_sum += rpm_now() * dt; rpm_t += dt; }
    if (M.state) g_sh_t += dt; else if (M.gear > 0) { g_rpm_s += rpm_now() * dt; g_rpm_t += dt; }
    if (M.gear > 0 && !M.state && M.v * 3.6f < 50) { rpm_all_sum += rpm_now() * dt; rpm_all_t += dt; }
}

struct setup { float mass, v0; int g0; double secs; float cvar = 3, shift_time = 0.5f, mass_seen = 0; int eb_toggle = -1; float eb_level = -1;
               float (*road)(double) = NULL; int route = 0; double junction = 0; };
static int rolls; static double roll_time, min_kmh, t_30, t_50, t_80, d_80;

static void run(const char *name, setup su, void (*ctl)(double))
{
    if (!g_quiet) printf("\n=== %s ===\n", name);
    memset(&M, 0, sizeof M);
    M.mass = su.mass; M.gear = M.target = su.g0; M.cvar = su.cvar; M.shift_time = su.shift_time;
    M.eb_level = su.eb_level; M.eb_toggle = su.eb_toggle; M.mass_seen = su.mass_seen;
    M.road = su.road; M.route_on = su.route; M.junction_s = su.junction;
    M.v = 0; tick(1 / 60.0, 0);  // register the gear at standstill (no hint)
    M.state = 0; M.gear = M.target = su.g0; M.force = 0;
    M.v = su.v0 / 3.6f; tick(1 / 60.0, 0);
    M.state = 0; M.gear = M.target = su.g0; M.force = 0;
    M.v = su.v0 / 3.6f; shifts = ups = hunts = last_dir = 0; up_rpm_sum = 0; up_n = 0; rpm_sum = rpm_t = 0;
    M.s = 0; rolls = 0; roll_time = 0; min_kmh = 1e9; t_30 = t_50 = t_80 = d_80 = -1;
    up_rpm_min = skip_rpm_min = skip_pull_min = 1e9; rpm_all_sum = rpm_all_t = 0; skips_up = hunts_ts = shifts_ts = 0;
    g_sh_t = g_rpm_s = g_rpm_t = 0; g_dskips = 0;
    int f0 = n_failed;
    double t0 = t_now, next_print = t0;
    while (t_now - t0 < su.secs) {
        ctl(t_now - t0);
        int was0 = M.gear == 0;
        tick(1 / 60.0, t_now - t0 > su.secs / 2);
        if (M.gear == 0 && M.v > 1) { roll_time += 1 / 60.0; if (!was0) rolls++; }
        if (t_now - t0 > 3 && M.v * 3.6 < min_kmh) min_kmh = M.v * 3.6;
        if (t_30 < 0 && M.v * 3.6f >= 30) t_30 = t_now - t0;
        if (t_50 < 0 && M.v * 3.6f >= 50) t_50 = t_now - t0;
        if (t_80 < 0 && M.v * 3.6f >= 80) { t_80 = t_now - t0; d_80 = M.s; }
        if (t_now >= next_print) { next_print += 10;
            if (!g_quiet) printf("        t=%5.1f  %5.1f km/h  gear %2d  rpm %4.0f  pedal %.2f cc %.2f grade %.1f%% %c\n", t_now - t0,
                   M.v * 3.6f, M.gear, rpm_now(), M.pedal, M.cc, M.grade_pct, letter); }
    }
    if (g_quiet) return;
    printf("  -> %d shifts (%d up, avg up-landing %.0f rpm), %d hunting pairs, %d no-pull lessons, end %.1f km/h gear %d, "
           "mean rpm 2nd half %.0f, min %.0f km/h, %d rolls %.0f s, %.0f m\n", shifts, ups, up_n ? up_rpm_sum / up_n : 0,
           hunts, n_failed - f0, M.v * 3.6f, M.gear, rpm_t > 0 ? rpm_sum / rpm_t : 0, min_kmh, rolls, roll_time, M.s);
    if (getenv("ACCEL")) printf("  accel: 30 km/h at %.1f s, 50 at %.1f s, 80 at %.1f s (%.0f m)\n", t_30, t_50, t_80, d_80);
}

// A driver holding a speed with the pedal (P controller, a bit lazy).
static void hold(float kmh, float gain = 0.25f)
{
    float e = kmh / 3.6f - M.v;
    M.pedal = fminf(1, fmaxf(0, M.pedal + (e * gain - (M.pedal - 0.45f) * 0.0f) * (1 / 60.0f) * 4));
    if (e < -1.5f) M.pedal = 0;
}
// The game's cruise control: bang-bang (full below, zero above, half at target).
static void cruise(float kmh)
{
    M.cruise_kmh = kmh; M.pedal = 0;
    float e = (kmh + g_roll_boost_kmh) / 3.6f - M.v;   // the cruise module raises its target by the roll boost TrueShift asks for
    M.cc = e > 0.3f ? 1.0f : e < -0.3f ? 0.0f : 0.5f;
    if (g_cruise_smooth) M.cc = fminf(1, fmaxf(0, 0.45f + e * 0.3f));  // a smooth controller
    if (g_rolling) M.cc = 0;                                            // the cruise module gives no throttle in neutral
}

static void c_full(double) { M.pedal = 1; }
static void c_p70(double) { M.pedal = 0.7f; }
static void c_p20(double) { M.pedal = 0.2f; }
static void c_p30(double) { M.pedal = 0.3f; }
static void c_p40(double) { M.pedal = 0.4f; }
static void c_p50(double) { M.pedal = 0.5f; }
static float g_ped; static void c_ped(double t) { M.pedal = g_ped >= 0 ? g_ped : 0.42f + 0.12f * sinf((float)t * 6.2832f / 5); }   // < 0: a wavering foot, 0.30-0.54 every 5 s
// Adaptive cruise set to 80 behind a truck doing 50: bursts, then long gaps.
static void c_acc(double t) { M.cruise_kmh = 80; M.pedal = 0; float e = 50 / 3.6f - M.v; M.cc = e > 0.3f ? 1.0f : e < -0.3f ? 0.0f : 0.3f; }
static void c_light(double) { M.pedal = 0.4f; }
static void c_solo80(double) { hold(80); }
static void c_cruise85(double) { M.grade_pct = 1.0f; hold(85); }
static void c_beamng(double t) { M.grade_pct = t < 30 ? 0 : t < 90 ? 3.5f : 0; hold(80); }
static void c_hills(double t) { M.grade_pct = 3 * sinf((float)t / 60 * 6.283f); hold(80); }
static void c_climb(double t) { M.grade_pct = t < 10 ? 0 : 6; M.pedal = 1; }
static void c_kick(double t) { M.pedal = t < 10 ? 0.35f : 1.0f; }
static void c_brake(double) { M.pedal = 0; M.brake = 0.25f; }
static void down70() { M.grade_pct = -5; M.pedal = 0; float e = M.v - 70 / 3.6f; M.brake = fminf(0.5f, fmaxf(0, e * 0.1f)); }
static void c_eb(double) { down70(); M.ebrake = 1; }
static void c_ret1(double) { down70(); M.ret = 1; }
static void c_ret5(double) { down70(); M.ret = 5; }
static void c_cc(double) { M.grade_pct = 1.0f; cruise(80); }
static void c_hint(double t)
{
    M.pedal = 0.6f;
    static int done;
    if (t < 0.1) done = 0;
    if (t > 15 && !done && !M.state) { done = 1; M.target = M.gear - 1; M.state = 1; M.shift_end = t_now + 0.6; printf("  (hint down)\n"); }
}
static void c_lowspeed(double t)
{
    if (t < 25) { M.pedal = 0.5f; M.brake = 0; }
    else if (t < 40) { M.pedal = 0; M.brake = 0.15f; }
    else if (t < 50) { M.pedal = 0.3f; M.brake = 0; }   // creep back up through the handover band
    else { M.pedal = 0; M.brake = 0.2f; }
}
static void c_m_full(double t) { M.pedal = 1; }
// roads (grade % along the distance)
static float r_climb(double s) { return s < 1500 ? 0 : s < 2500 ? 4.0f : 0; }
static float r_crest(double s) { return s < 700 ? 5.0f : s < 1700 ? -1.5f : 0; }
static float r_down(double s) { return -0.8f; }
static float r_climb10(double s) { return s < 300 ? 0 : s < 600 ? 10.5f : 0; }   // a steep 300 m climb after a flat (24-10, 545 s: 10-11 % at 43 km/h)
static float r_climb6(double s) { return s < 300 ? 0 : s < 700 ? 6.0f : 0; }
static float r_climb3(double s) { return s < 300 ? 0 : s < 900 ? 3.0f : 0; }   // a long 3 % climb   // a moderate 400 m climb: the gear can just hold it
static float r_hills(double s) { return 1.5f * sinf((float)(s / 1500 * 6.2832)); }
static float r_flat(double s) { return 0; }
static float r_hills25(double s) { return 2.5f * sinf((float)(s / 1000 * 6.2832)); }
static float r_short3(double s) { return s < 1500 ? 0 : s < 1900 ? 3.0f : 0; }   // 400 m at 3 %: a top within reach
static float r_short5(double s) { return s < 1500 ? 0 : s < 1800 ? 5.0f : 0; }   // 300 m at 5 %
static float r_hump3(double s) { return s < 1500 ? 0 : s < 1650 ? 3.0f : 0; }   // 150 m at 3 %
static float r_hump15(double s) { return s < 1500 ? 0 : s < 1900 ? 1.5f : 0; }  // 400 m at 1.5 %
static float r_mid3(double s) { return s < 1500 ? 0 : s < 2300 ? 3.0f : 0; }    // 800 m at 3 %
static float r_long4(double s) { return s < 1500 ? 0 : s < 3500 ? 4.0f : 0; }    // 2 km at 4 %: must downshift
// Ride 3 (11-10, 395-402 s): a 5-6 % climb with a short 4.1 % dip; 30.4 t on cruise 73 went 8 -> 9 in the dip and back after 1.6 s.
static float g_dipg = 5.8f;   // the climb's grade (H1 runs it at a few, the sim truck is not ride 3's)
static float r_dip(double s) { return s < 300 ? 1.0f : s < 900 ? g_dipg : s < 960 ? g_dipg - 1.7f : s < 1500 ? g_dipg - 0.2f : s < 1800 ? 2.0f : 0; }
static void c_cci73(double) { M.cruise_kmh = 73; M.pedal = 0; float e = 73 / 3.6f - M.v; M.cc = fminf(1, fmaxf(0, 0.5f + e * 1.0f)); }
static void c_cci80(double) { M.cruise_kmh = 80; M.pedal = 0; float e = 80 / 3.6f - M.v; M.cc = fminf(1, fmaxf(0, 0.5f + e * 1.0f)); }
static void set_param(const char *n, float v)
{
    for (int i = 0; i < m_trueshift.n_params; i++) if (!strcmp(m_trueshift.params[i].key, n)) *m_trueshift.params[i].val = v;
}
static void c_hold80(double) { hold(80); }
static void c_full_road(double) { M.pedal = 1; }
static void c_half(double) { M.pedal = 0.5f; }
// lift off at t=10 towards a junction 60 m ahead, full pedal again at t=14 (ride 24-10, 1830-1834 s: 14>16 while rolling out, then back)
static void c_lift_jn(double t) { if (t < 10) hold(70); else if (t < 14) M.pedal = 0; else M.pedal = 1.0f; }
static void c_lift_jn_bug(double t) { M.thr_floor = 0.08f; c_lift_jn(t); }
static void c_lift_jn_free(double t) { if (t < 10) hold(70); else M.pedal = 0; }
static void c_release(double t) { if (t < 10 || t > 50) hold(80); else M.pedal = 0; }
// the yard (24-10, 3853-3877 s): 13 km/h, a short brake, then gentle pedal through 5-25 km/h
static void c_yard(double t) { if (t < 2) { M.pedal = 0.2f; M.brake = 0; } else if (t < 3.5) { M.pedal = 0; M.brake = 0.15f; } else { M.pedal = 0.3f; M.brake = 0; } }
static void c_pull03(double t) { M.pedal = t < 3 ? 0 : 0.3f; M.brake = 0; }
static void c_cc46(double) { cruise(46); }   // an ACC target behind a slow lead (24-10: lead 40-46 km/h)
static void c_cc52(double) { cruise(52); }
static void c_cc63(double) { cruise(63); }
static void c_cc80(double) { cruise(80); }
static void c_car_light(double) { M.pedal = 0.35f; }
static void c_car_full(double) { M.pedal = 1; }
static void c_car_100(double) { hold(100, 0.05f); }
static void c_m_climb(double t) { M.pedal = 1; M.grade_pct = t < 5 ? 0 : 7; }
static void c_m_stop(double) { M.pedal = 0; M.brake = 0.2f; }
// The game ignores the hint in M (ride 3): only the button is pressed, TrueShift must shift itself
static void c_m_btn(double t)
{
    M.pedal = 0.5f;
    sim_btn = (t > 8 && fmod(t, 4) < 0.15 && t < 20) ? 14 : 0;
}
static void c_m_hint(double t)
{
    M.pedal = 0.5f;
    static int done;
    if (t < 0.1) done = 0;
    if (t > 8 && done < 3 && !M.state && fmod(t, 4) < 0.05) { done++; M.target = M.gear + 1; M.state = 1; M.shift_end = t_now + 0.5; printf("  (hint up)\n"); }
}

// the rev-limiter bug of the 24-10 ride (124-130 s): full pedal in M against the limiter, three + presses; the chooser runs every tick out of the window
static void c_m_limiter(double t)
{
    M.pedal = 1.0f;
    static int done;
    if (t < 0.1) done = 0;
    if (t > 8 && done < 3 && !M.state && fmod(t, 3) < 0.06) { done++; M.hint = 1; printf("  (hint up at %.1f s, rpm %.0f)\n", t, rpm_now()); }
}

// X4: full pedal up a 6 % climb, at t = 20 s a teleport (HARD reset) onto a 2 % climb at the same speed. The grade peak decide_gear
// reckons the next upshift with belongs to the road we left: the upshift must come soon on the 2 % road, not ~27 s later when a 6 % peak
// has faded (it used to be a function-local static that lived through the reset).
static int g_tp_done, g_tp_ups, g_tp_downs, g_tp_snap, g_tp_downs1; static double g_tp_up_at;   // (g_tp_downs1: the downshifts counted from 1.5 s after the teleport)
static float g_tp_pedal = 1, g_tp_g1 = 6, g_tp_g2 = 2;   // (the SCORES table runs other pedals and grades)
static void c_teleport(double t)
{
    M.pedal = g_tp_pedal; M.grade_pct = t < 20 ? g_tp_g1 : g_tp_g2;
    if (t >= 20 && !g_tp_done) { g_tp_done = 1; g_tp_ups = ups; g_tp_downs = shifts - ups; sim_reset(RESET_HARD, "sim: teleport onto a 2 % climb"); if (!g_quiet) printf("  (teleport: 6 %% -> 2 %%)\n"); }
    if (g_tp_done && !g_tp_snap && t >= 21.5) { g_tp_snap = 1; g_tp_downs1 = shifts - ups; }
    if (g_tp_done && g_tp_up_at < 0 && ups > g_tp_ups) g_tp_up_at = t;
}

static void run_m()
{
    run("M1 M mode, 40 t, full throttle from 50 in 8", {40000, 50, 8, 40, 0}, c_m_full);
    run("M2 M mode, 40 t, full throttle, 7% climb", {40000, 60, 10, 50, 0}, c_m_climb);
    run("M3 M mode, 40 t, braking to a stop from 60 in 10", {40000, 60, 10, 30, 0}, c_m_stop);
    run("M4 M mode, 40 t, 0.5 pedal, three hint-ups", {40000, 40, 7, 25, 0}, c_m_hint);
    run("M5 M mode, 40 t, 0.5 pedal, three hint-up presses the game ignores (ride 3)", {40000, 40, 7, 25, 0}, c_m_btn);
    run("M6 M mode, 40 t, FULL pedal against the rev limiter in gear 3, three hint-up presses (ride 24-10, 124-130 s)", {40000, 12, 3, 25, 0}, c_m_limiter);
    sim_btn = 0;
}

// ---- SCORES: the score-based gear choice ([trueshift] choice = 1, core/gearscore.h). Run: tools/sim/run_scores.sh
// Part 1: the pure score function on the vehicle model. Part 2 (further down): choice 0 against choice 1 in the simulator.
static int g_sc_failed;
static void sc_check(bool ok, const char *what) { printf("  %-4s %s\n", ok ? "ok" : "FAIL", what); if (!ok) g_sc_failed = 1; }
static void sc_prime(float mass, float shift_time, float kmh, int gear)   // the vehicle model reads this truck now
{
    memset(&M, 0, sizeof M);
    M.mass = mass; M.shift_time = shift_time; M.cvar = 3; M.v = kmh / 3.6f; M.gear = M.target = gear;
    t_now += 10; T.speed = M.v; T.engine_on = 1;
    vehicle_reset(RESET_HARD); vehicle_update(SELF);
}
// the settings of a mode as TrueShift builds them from the ini defaults (set_param changes win)
static gs_cfg sc_cfg(int mode, int skips)   // mode: 0 Normal, 1 Eco, 2 Power
{
    float v[9]; const char *k[9] = { "choice_comfort", "choice_comfort_eco", "choice_comfort_power", "choice_rpm_weight", "choice_shift_weight", "choice_stay", "choice_stay_power", "hysteresis_rpm", "choice_floor_rpm" };
    for (int i = 0; i < 9; i++) for (int j = 0; j < m_trueshift.n_params; j++) if (!strcmp(m_trueshift.params[j].key, k[i])) v[i] = *m_trueshift.params[j].val;
    gs_cfg c = {};
    c.comfort = v[mode == 1 ? 1 : mode == 2 ? 2 : 0]; c.w_rpm = v[3]; c.w_shift = v[4]; c.stay = v[mode == 2 ? 6 : 5]; c.hyst = v[7]; c.floor = v[8]; c.floor_cur = 1;
    c.idle = 550; c.skips = c.skips_up = c.skips_later = skips; c.horizon = 8; c.learn_res = 0;
    return c;
}
// the force a pedal asks in gear cur at the speed of now, as TrueShift's f_need holds it in steady state
static float sc_need(int cur, float pedal) { return fmaxf(vehicle_force_at(cur, M.v, pedal), vehicle_resist(M.v)); }
static void sc_show(const char *title, const gs_out &o, int cur)
{
    printf("  %s\n", title);
    for (int g = 1; g <= NG; g++) {
        if (g < cur - 3 || g > cur + 3) continue;
        const gs_term &t = o.t[g];
        if (!t.g) continue;
        printf("    gear %2d%s rpm %5.0f share %4.2f  %s  gas %.3f rpm %.3f shift %.3f (%d) stay %.3f = %7.3f\n", g, g == cur ? "*" : " ", t.rpm, t.share > 8 ? 9.0 : t.share,
               t.ok ? "ok " : "out", t.gas, t.rpmc, t.shiftc, t.shifts, t.stay, t.score);
    }
}
static int run_scores_table();
static int run_scores()
{
    printf("\n=== SCORES part 1: the score function on the vehicle model (Renault ATO 12 speeds, diff 3.08) ===\n");
    use_ato();
    gs_out o; gs_out o2;
    // 17 t at 45 km/h in 8, pedal 0.5: the demand of a pull
    sc_prime(17000, 0.5f, 45, 8);
    gs_cfg c = sc_cfg(0, 2);
    gs_eval(&o, 8, NG, M.v, sc_need(8, 0.5f), vehicle_resist(M.v), 0, c);
    sc_show("17 t, 45 km/h in 8, pedal 0.5, Normal, skips 2", o, 8);
    sc_check(o.n > 0 && o.best >= 1, "a best gear exists");
    bool infeasible_out = true; for (int g = 1; g <= NG; g++) if (o.t[g].g && o.t[g].share > 1 && o.t[g].ok) infeasible_out = false;
    sc_check(infeasible_out, "a gear that needs a share over 1 is never a candidate");
    // coasting asks nothing: every gear in the window has share 0
    gs_eval(&o2, 8, NG, M.v, sc_need(8, 0.0f), vehicle_resist(M.v), 1, c);
    bool zero = true; for (int g = 1; g <= NG; g++) if (o2.t[g].g && o2.t[g].ok && o2.t[g].share != 0) zero = false;
    sc_check(zero, "coasting: nothing is asked (share 0)");
    // one skip is one shift: for the same landing gear a skip never costs more shifts than the two single shifts it replaces
    bool skip_cheaper = true; int n_cmp = 0;
    for (float kmh : { 15.0f, 25.0f, 35.0f, 45.0f, 60.0f })
        for (float pedal : { 0.3f, 0.5f, 0.7f })
            for (int cur = 3; cur <= 11; cur++) {
                sc_prime(25000, 0.5f, kmh, cur);
                gs_eval(&o, cur, NG, M.v, sc_need(cur, pedal), vehicle_resist(M.v), 0, c);
                if (o.t[cur + 1].ok && o.t[cur + 2].ok) { n_cmp++; if (o.t[cur + 2].shifts > o.t[cur + 1].shifts) skip_cheaper = false; }
            }
    sc_check(skip_cheaper && n_cmp > 10, "the skip (cur+2) needs no more shifts than the single step (cur+1) over a grid of speeds, pedals, gears");
    // a slow gearbox pays more per shift: where the skip (cur+2) saves a later shift over the single step (cur+1), it wins by more
    // with a slow box than with a fast one (the slow box also loses more speed in the shift, so a skip may land too low and fall out: not counted)
    int wider = 0, narrower = 0, pairs = 0;
    for (float kmh : { 15.0f, 20.0f, 25.0f, 30.0f, 35.0f, 45.0f })
        for (float pedal : { 0.3f, 0.4f, 0.5f, 0.6f })
            for (float mass : { 17000.0f, 36000.0f })
                for (int cur = 3; cur <= 11; cur++) {
                    float margin[2]; bool use = true;
                    for (int k = 0; k < 2; k++) {
                        sc_prime(mass, k ? 1.5f : 0.3f, kmh, cur);
                        gs_eval(&o, cur, NG, M.v, sc_need(cur, pedal), vehicle_resist(M.v), 0, c);
                        if (!(o.t[cur + 1].ok && o.t[cur + 2].ok && o.t[cur + 2].shifts < o.t[cur + 1].shifts)) use = false;
                        else margin[k] = o.t[cur + 2].score - o.t[cur + 1].score;
                    }
                    if (!use) continue;
                    pairs++;
                    if (margin[1] > margin[0] + 0.001f) wider++;
                    if (margin[1] < margin[0] - 0.02f) narrower++;
                }
    printf("  slow box 1.5 s vs fast 0.3 s: the skip's lead over the single step is wider in %d of %d cases, narrower in %d\n", wider, pairs, narrower);
    sc_check(pairs > 5 && narrower == 0 && wider == pairs, "a slow gearbox pays more per shift: the skip wins by more");
    // the modes: Eco goes as far as Power or farther, Power the least
    int eco_far = 0, eco_near = 0, pw_far = 0;
    for (float kmh : { 15.0f, 25.0f, 35.0f, 45.0f, 60.0f })
        for (float pedal : { 0.3f, 0.4f, 0.5f, 0.6f })
            for (int cur = 3; cur <= 11; cur++) {
                int best[3];
                for (int md = 0; md < 3; md++) {
                    sc_prime(25000, 0.5f, kmh, cur);
                    gs_eval(&o, cur, NG, M.v, sc_need(cur, pedal), vehicle_resist(M.v), 0, sc_cfg(md, 2));
                    best[md] = o.best;
                }
                if (best[0] && best[1] && best[2]) { if (best[1] > best[2]) eco_far++; if (best[1] < best[2]) eco_near++; if (best[0] > best[2]) pw_far++; }
            }
    printf("  Eco higher than Power in %d cases, lower in %d; Normal higher than Power in %d\n", eco_far, eco_near, pw_far);
    sc_check(eco_near == 0 && eco_far > 0 && pw_far > 0, "Eco skips farther than Power, never less; Normal farther than Power somewhere");
    return run_scores_table();
}

// ---- part 2: choice 0 (the cascade) against choice 1 (the scores) in the simulator
struct sc_res { int shifts, skips; double shift_s, rpm, t50; int hunts; };
static sc_res sc_last;
static void sc_do(const char *name, setup su, void (*ctl)(double), int choice)
{
    set_param("choice", (float)choice);
    // a short drive first: the module keeps a once-per-standstill flag (the launch correction) until the truck moves, and a run that ends
    // in a crawl would give the one after it a different start than its twin
    g_quiet = 1; run("wash", {20000, 30, 6, 2}, c_hold80); g_quiet = 0;
    sim_reset(RESET_HARD, "sim: next scores scenario");   // every run starts from a module that knows nothing of the one before
    g_quiet = getenv("SC_VERBOSE") ? 0 : 1;
    run(name, su, ctl);
    g_quiet = 0;
    sc_last = { shifts, skips_up + g_dskips, g_sh_t, g_rpm_t > 0 ? g_rpm_s / g_rpm_t : 0, t_50, hunts };
}
static void sc_print(const char *label, int choice, const sc_res &r)
{
    char t50[16]; if (r.t50 < 0) snprintf(t50, sizeof t50, "   -"); else snprintf(t50, sizeof t50, "%4.1f", r.t50);
    printf("  %-44s %d  %6d %6d %8.1f %8.0f  %s %6d\n", label, choice, r.shifts, r.skips, r.shift_s, r.rpm, t50, r.hunts);
}
static sc_res g_cmp[2];
static void sc_pair(const char *label, setup su, void (*ctl)(double))   // both choices on the same scenario; the results stay in g_cmp
{
    for (int i = 0; i < 2; i++) { int ch = getenv("SC_REV") ? 1 - i : i; sc_do(label, su, ctl, ch); g_cmp[ch] = sc_last; sc_print(label, ch, sc_last); }
}
static void sc_header(const char *title)
{
    printf("\n  %s\n  %-44s %s  %6s %6s %8s %8s  %4s %6s\n", title, "scenario", "c", "shifts", "skips", "shift s", "mean rpm", "t50", "hunts");
}
// the regression of a real ride: the pedal ramps up, the truck skips at 45 km/h, the driver lifts to 0.00 a second after the skip landed
static int g_rg_roll0, g_rg_roll10; static double g_rg_lift, g_rg_first; static float g_rg_p0 = 0.45f; static int g_rg_g0, g_rg_sh0, g_rg_sh10; static bool g_rg_done10;
static void c_regress(double t)
{
    if (g_rg_lift < 0 && M.gear >= g_rg_g0 + 2 && M.v * 3.6f >= 40) g_rg_lift = t + 1.0;   // the skip has landed
    if (g_rg_lift < 0 && t >= 25) g_rg_lift = t;                                          // no skip came: lift anyway
    M.pedal = g_rg_lift >= 0 && t >= g_rg_lift ? 0.0f : fminf(0.73f, g_rg_p0 + 0.04f * (float)t);
    static bool at_lift; if (t < 0.1) { at_lift = false; g_rg_done10 = false; }
    if (!at_lift && g_rg_lift >= 0 && t >= g_rg_lift) { at_lift = true; g_rg_sh0 = shifts; g_rg_roll0 = rolls; }
    if (at_lift && g_rg_first < 0 && shifts > g_rg_sh0) g_rg_first = t - g_rg_lift;
    if (!g_rg_done10 && g_rg_lift >= 0 && t >= g_rg_lift + 10) { g_rg_done10 = true; g_rg_sh10 = shifts; g_rg_roll10 = rolls; }
}
static int sc_gear_for(float kmh, float rpm_min)   // the highest gear that still gives at least rpm_min at kmh
{
    int g0 = 1;
    for (int g = 1; g <= NG; g++) if ((kmh / 3.6f) / RADIUS * RATIO[g - 1] * DIFF * 9.5493f >= rpm_min) g0 = g;
    return g0;
}
static void c_hold45(double) { hold(45); }
static void c_hold70(double) { hold(70); }
static int run_scores_table()
{
    struct Tr { const char *name; void (*use)(); int g0; } trucks[2] = { { "ATO", use_ato, 1 }, { "XT", use_renault, 3 } };
    // 1. pull-away from a standstill at a light pedal (the ACCEL0 scenarios), 90 s
    printf("\n=== SCORES part 2: choice 0 (shift points) against choice 1 (scores), Normal, max_skip 2, 0.5 s box ===\n");
    printf("  shifts, skips (up + down), seconds in shifts, mean rpm in gear, time to 50 km/h (- = not reached), hunting pairs (direction reversal within 5 s)\n");
    sc_res tot[2] = {};
    int n_rows = 0, n_t50 = 0;
    sc_header("pull-away from a standstill, 90 s");
    void (*ctls[4])(double) = { c_p20, c_p30, c_p40, c_p50 }; const char *cn[4] = { "0.2", "0.3", "0.4", "0.5" };
    for (auto &tr : trucks) {
        tr.use();
        for (float mass : { 20000.0f, 36000.0f })
            for (int c = 0; c < 4; c++) {
                char nm[100]; snprintf(nm, sizeof nm, "pull-away %s %.0f t pedal %s", tr.name, mass / 1000, cn[c]);
                sc_pair(nm, {mass, 1, tr.g0, 90}, ctls[c]);
                if (g_cmp[0].shifts > 100) continue;   // the XT at pedal 0.2 never gets going in either mode (its low-speed shifting in the sim): not in the sums
                for (int ch = 0; ch < 2; ch++) { tot[ch].shifts += g_cmp[ch].shifts; tot[ch].skips += g_cmp[ch].skips; tot[ch].shift_s += g_cmp[ch].shift_s; tot[ch].rpm += g_cmp[ch].rpm; tot[ch].hunts += g_cmp[ch].hunts; if (g_cmp[0].t50 > 0 && g_cmp[1].t50 > 0) tot[ch].t50 += g_cmp[ch].t50; }
                if (g_cmp[0].t50 > 0 && g_cmp[1].t50 > 0) n_t50++;
                n_rows++;
            }
    }
    printf("  %-44s %d  %6d %6d %8.1f %8.0f  %4.1f %6d   (sums; mean rpm and t50 are means over the rows, t50 where both reach 50)\n", "pull-away, 14 rows without the XT at 0.2", 0, tot[0].shifts, tot[0].skips, tot[0].shift_s, tot[0].rpm / n_rows, tot[0].t50 / n_t50, tot[0].hunts);
    printf("  %-44s %d  %6d %6d %8.1f %8.0f  %4.1f %6d\n", "", 1, tot[1].shifts, tot[1].skips, tot[1].shift_s, tot[1].rpm / n_rows, tot[1].t50 / n_t50, tot[1].hunts);
    // 2. steady cruising
    sc_header("steady cruising (a driver holding the speed), 90 s");
    use_ato();
    float cruise_rpm_min = 1e9f;
    for (float mass : { 17000.0f, 36000.0f })
        for (float kmh : { 45.0f, 70.0f }) {
            char nm[100]; snprintf(nm, sizeof nm, "cruise %.0f km/h, ATO %.0f t", kmh, mass / 1000);
            sc_pair(nm, {mass, kmh, sc_gear_for(kmh, 1100), 90}, kmh < 50 ? c_hold45 : c_hold70);
            cruise_rpm_min = fminf(cruise_rpm_min, (float)g_cmp[1].rpm);
        }
    sc_check(cruise_rpm_min >= V.plateau + sc_cfg(0, 2).floor, "choice 1: a steady cruise does not sit under the torque plateau + choice_floor_rpm (mean rpm of all four rows)");
    // 3. the other modes and a slow box, pull-away 36 t pedal 0.3
    sc_header("the modes and a slow gearbox (ATO 36 t, pedal 0.3, 90 s)");
    sc_pair("Eco", {36000, 1, 1, 90, 10.0f}, c_p30);
    sc_pair("Normal", {36000, 1, 1, 90, 3.0f}, c_p30);
    sc_pair("Power", {36000, 1, 1, 90, 1.6667f}, c_p30);
    sc_pair("Normal, slow box 1.2 s", {36000, 1, 1, 90, 3.0f, 1.2f}, c_p30);
    // 3b. cruise control pulling up to its speed (the demand filter is still on its way up when a gear is chosen): no upshift without pull
    printf("\n  cruise control pulling 60 -> 73 km/h up a climb with a dip (30.4 t Volvo, Eco, route), the grade of the climb 5.8 ... 8.0 %%: no-pull lessons (upshifts that had to come back)\n");
    {
        int les[2] = {}, ups[2] = {};
        use_volvo();
        for (float gg : { 5.8f, 6.5f, 7.0f, 7.5f, 8.0f }) {
            g_dipg = gg;
            for (int ch = 0; ch < 2; ch++) { int f0 = n_failed; sc_do("cruise pull", {30400, 60, 9, 150, 10, 1.0f, 0, -1, -1, r_dip, 1}, c_cci73, ch); les[ch] += n_failed - f0; ups[ch] += skips_up; }
        }
        use_ato();
        printf("  choice 0: %d (%d skips up), choice 1: %d (%d skips up)\n", les[0], ups[0], les[1], ups[1]);
        sc_check(les[1] <= les[0], "choice 1: no more no-pull lessons than choice 0 with the cruise control pulling up to its speed");
    }
    // 3c. a teleport (hard reset) in the middle of a pull, onto another grade: the first decisions after it start from the pedal of now
    // and no state: no more downshifts and no-pull lessons afterwards than with choice 0 (X4 in run.sh, with other speeds, pedals, grades)
    printf("\n  teleport (hard reset) at 20 s in a pull, 40 t Volvo: downshifts after it and no-pull lessons, choice 0 / choice 1 (sums over %s)\n", "2 speeds, 3 pedals, 3 grade pairs");
    {
        int downs[2] = {}, les[2] = {}, worse = 0;
        use_volvo();
        for (int sg = 0; sg < 2; sg++)
            for (float pedal : { 0.3f, 0.5f, 1.0f })
                for (int gp = 0; gp < 3; gp++) {
                    static const float gr[3][2] = { { 6, 2 }, { 8, 3 }, { 4, 0 } };
                    int d[2], l[2];
                    for (int ch = 0; ch < 2; ch++) {
                        g_tp_done = g_tp_snap = 0; g_tp_up_at = -1; g_tp_pedal = pedal; g_tp_g1 = gr[gp][0]; g_tp_g2 = gr[gp][1];
                        int f0 = n_failed;
                        sc_do("teleport", {40000, sg ? 30.0f : 45.0f, sg ? 7 : 8, 60}, c_teleport, ch);
                        d[ch] = shifts - ups - g_tp_downs1; l[ch] = n_failed - f0; downs[ch] += d[ch]; les[ch] += l[ch];
                    }
                    if (d[1] > d[0] || l[1] > l[0]) { worse++; printf("    worse with choice 1: %.0f km/h, pedal %.1f, grade %.0f -> %.0f %%: downshifts %d / %d, lessons %d / %d\n", sg ? 30.0f : 45.0f, pedal, gr[gp][0], gr[gp][1], d[0], d[1], l[0], l[1]); }
                }
        g_tp_pedal = 1; g_tp_g1 = 6; g_tp_g2 = 2;
        use_ato();
        printf("  downshifts after the teleport: %d / %d, no-pull lessons: %d / %d\n", downs[0], downs[1], les[0], les[1]);
        sc_check(worse == 0, "choice 1: after a teleport in a pull no more downshifts and no-pull lessons than choice 0, in every one of the 18 cases");
    }
    // 4. the regression from a ride: the truck skips to the tenth at ~45 km/h and 880 rpm, the driver lifts to 0.00 one second later. Several
    // ramps and start speeds, so that the skip lands at different moments; no brake, no engine brake, no retarder: no hard rule asks for a shift
    printf("\n  regression (17 t ATO, 8 at the start speed): the pedal ramps from p0 by 0.04/s to 0.73, the driver lifts to 0.00 one second after the first gear +2 lands at >= 40 km/h\n");
    printf("  %-22s | %-44s | %-44s\n", "", "choice 0", "choice 1");
    int worst = 0, n_skip1 = 0, n_var = 0;
    for (float v0 : { 34.0f, 38.0f, 42.0f })
        for (float p0 : { 0.2f, 0.3f, 0.4f, 0.45f }) {
            if (getenv("RG_ONLY") && (fabs(atof(getenv("RG_ONLY")) - v0) > 0.01 || fabs(atof(getenv("RG_ONLY2")) - p0) > 0.01)) continue;
            char cell[2][100]; int sh[2]; 
            for (int ch = 0; ch < 2; ch++) {
                g_rg_lift = -1; g_rg_first = -1; g_rg_g0 = 8; g_rg_p0 = p0;
                sc_do("regression", {17000, v0, 8, 45}, c_regress, ch);
                bool skipped = g_rg_lift >= 0 && g_rg_lift < 25;   // the lift was triggered by a landed skip, not by the 25 s fallback
                sh[ch] = g_rg_done10 ? g_rg_sh10 - g_rg_sh0 : -1;
                const char *rl = g_rg_done10 && g_rg_roll10 > g_rg_roll0 ? " (eco-roll)" : "";
                if (g_rg_first >= 0) snprintf(cell[ch], sizeof cell[ch], "%s lift t=%4.1f: %d shifts, 1st at +%.1f s%s", skipped ? "+2 gears" : "none", g_rg_lift, sh[ch], g_rg_first, rl);
                else snprintf(cell[ch], sizeof cell[ch], "%s lift t=%4.1f: %d shifts", skipped ? "+2 gears" : "none", g_rg_lift, sh[ch]);
                if (ch == 1) { n_var++; if (skipped) n_skip1++; if (sh[1] > worst) worst = sh[1]; }
            }
            printf("  from %2.0f km/h, p0 %.2f   | %-44s | %-44s\n", v0, p0, cell[0], cell[1]);
        }
    printf("  choice 1: %d of %d ramps got the extra 2 gears; the most shifts in the 10 s after the lift: %d\n", n_skip1, n_var, worst);
    sc_check(n_skip1 > 0, "choice 1 skips in some of the ramps (else the test says nothing)");
    sc_check(worst <= 1, "choice 1: at most 1 shift in the 10 s after the lift to 0.00 in every ramp (no engine brake, no retarder, no brake; an eco-roll into neutral counts as the one shift)");
    return g_sc_failed;
}

int main(int argc, char **argv)
{
    verbose = argc > 1 && !strcmp(argv[1], "-v");
    for (int i = 0; i < m_trueshift.n_params; i++) *m_trueshift.params[i].val = m_trueshift.params[i].def;
    T.fwd_gears = 12; T.diff = 2.85f; T.wheels = 4; T.retarder_steps = 5;
    for (int i = 0; i < 12; i++) T.ratio[i] = RATIO[i];
    save_volvo();
    G.tail_site = 5; G.decide_site = 1; G.n_choose = 1; G.choose_sites[0] = 2; G.n_start = 1; G.start_sites[0] = 4;
    G.window_fn = (uintptr_t)f_window; G.chooser_fn = (uintptr_t)f_choose; G.start_fn = (uintptr_t)f_start;
    m_trueshift.start();
    if (getenv("BAND0")) set_param("crest_band_kmh", 0);
    if (getenv("NOBOOST")) set_param("eco_roll_boost_kmh", 0);
    if (getenv("NOLOAD")) set_param("load_down_from", 1);
    if (getenv("CHOICE")) set_param("choice", (float)atoi(getenv("CHOICE")));   // CHOICE=1: every scenario below with the score-based gear choice
    if (getenv("SCP")) { char b[200]; snprintf(b, sizeof b, "%s", getenv("SCP")); for (char *t = strtok(b, ","); t; t = strtok(NULL, ",")) { char *e = strchr(t, '='); if (e) { *e = 0; set_param(t, (float)atof(e + 1)); } } }   // SCP="key=value,..." overrides ini keys (tuning)

    if (getenv("SCORES")) return run_scores();
    if (getenv("RELAX")) {  // skip_relax_rpm study: pull-away at a fixed pedal from a standstill, 12 and 25 km/h, 12/20/36 t, both of his boxes
        // CSV: truck, t, pedal (-1 = wavering), from km/h, s to 30/50/80, shifts, ups, skips, hunting pairs, no-pull lessons, incidents,
        // lowest up-landing rpm (>= 8 km/h), mean rpm below 50 km/h, shifts and hunting pairs >= 8 km/h, lowest skip landing rpm, lowest pull after a skip (m/s2)
        // env: RELAX_R = skip_relax_rpm (0 = the old floor), RELAX_MS = max_skip, RELAX_IDLE = idle_rpm
        const char *ks[] = { "skip_relax_rpm", "max_skip", "idle_rpm" }, *es[] = { "RELAX_R", "RELAX_MS", "RELAX_IDLE" };
        for (int i = 0; i < 3; i++) if (getenv(es[i])) set_param(ks[i], (float)atof(getenv(es[i])));
        struct Tr { const char *name; void (*use)(); int g0; } trucks[2] = { { "ATO", use_ato, 1 }, { "XT", use_renault, 3 } };
        for (auto &tr : trucks) {
            tr.use();
            for (float mass : { 12000.0f, 20000.0f, 36000.0f })
                for (float ped : { 0.1f, 0.2f, 0.3f, 0.35f, 0.4f, 0.45f, 0.5f, 0.55f, 0.6f, 0.7f, 1.0f, -1.0f })
                    for (float v0 : { 1.0f, 12.0f, 25.0f }) {
                        int g0 = tr.g0;
                        if (v0 > 5) for (int g = 1; g <= NG; g++) { float r = (v0 / 3.6f) / RADIUS * RATIO[g - 1] * DIFF * 9.5493f; if (r >= 1000) g0 = g; }
                        g_ped = ped; int i0 = g_n_incidents, f0 = n_failed;
                        char nm[160]; snprintf(nm, sizeof nm, "RELAX %s %.0f t pedal %.2f from %.0f km/h in %d", tr.name, mass / 1000, ped, v0, g0);
                        run(nm, {mass, v0, g0, 120}, c_ped);
                        printf("CSV,%s,%.0f,%.2f,%.0f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%.0f,%.0f,%d,%d,%.0f,%.2f\n", tr.name, mass / 1000, ped, v0, t_30, t_50, t_80,
                               shifts, ups, skips_up, hunts, n_failed - f0, g_n_incidents - i0, up_rpm_min < 1e8 ? up_rpm_min : 0, rpm_all_t > 0 ? rpm_all_sum / rpm_all_t : 0, shifts_ts, hunts_ts,
                               skip_rpm_min < 1e8 ? skip_rpm_min : 0, skip_pull_min < 1e8 ? skip_pull_min : 0);
                    }
        }
        return 0;
    }
    if (getenv("ACCEL0")) {  // pull-away from a standstill at light pedal, on his two boxes: 0 to 30 and 0 to 50 km/h with 0 or 2 skipped gears
        struct Tr { const char *name; void (*use)(); int g0; } trucks[2] = { { "ATO2612F (12, diff 3.08)", use_ato, 1 }, { "Xtended 12+2 (diff 3.40)", use_renault, 3 } };
        void (*ctls[4])(double) = { c_p20, c_p30, c_p40, c_p50 }; const char *cn[4] = { "0.2", "0.3", "0.4", "0.5" };
        for (auto &tr : trucks) {
            tr.use();
            for (float mass : { 36000.0f, 20000.0f })
                for (int c = 0; c < 4; c++)
                    for (int ms : { 0, 2 }) {
                        set_param("max_skip", (float)ms);
                        char nm[160];
                        snprintf(nm, sizeof nm, "ACCEL0 %s, %.0f t, pedal %s, max_skip %d", tr.name, mass / 1000, cn[c], ms);
                        run(nm, {mass, 1, tr.g0, 90}, ctls[c]);
                    }
        }
        return 0;
    }
    if (getenv("ACCEL")) {  // acceleration study: how fast is a pull-away with 0, 1 or 2 skipped gears (no fuel in it), on his two boxes
        struct Tr { const char *name; void (*use)(); int ng; } trucks[2] = { { "ATO2612F 12 speeds, diff 3.08", use_ato, 12 }, { "Xtended 12+2, diff 3.40", use_renault, 14 } };
        for (auto &tr : trucks) {
            tr.use();
            // the start gear at 12 km/h: the highest gear that still gives >= 1000 rpm there
            int g0 = 1; for (int g = 1; g <= tr.ng; g++) { float r = (12 / 3.6f) / RADIUS * RATIO[g - 1] * DIFF * 9.5493f; if (r >= 1000) g0 = g; }
            for (int ms = 0; ms <= 2; ms += 1) {
                set_param("max_skip", (float)ms);
                char nm[160];
                snprintf(nm, sizeof nm, "ACCEL %s, max_skip %d, 36 t, pedal 1.0, from 12 km/h in %d", tr.name, ms, g0); run(nm, {36000, 12, g0, 80}, c_full);
                snprintf(nm, sizeof nm, "ACCEL %s, max_skip %d, 36 t, pedal 0.7", tr.name, ms); run(nm, {36000, 12, g0, 80}, c_p70);
                snprintf(nm, sizeof nm, "ACCEL %s, max_skip %d, 36 t, pedal 0.5", tr.name, ms); run(nm, {36000, 12, g0, 80}, c_p50);
                snprintf(nm, sizeof nm, "ACCEL %s, max_skip %d, 20 t, pedal 0.7", tr.name, ms); run(nm, {20000, 12, g0, 80}, c_p70);
            }
        }
        return 0;
    }
    run("A1 40 t, flat, full throttle from 12 km/h (DCT 0.5 s)", {40000, 12, 3, 70}, c_full);
    run("A2 40 t, flat, full throttle, slow box 1.2 s", {40000, 12, 3, 70, 3, 1.2f}, c_full);
    run("A2b 8 t solo, flat, full throttle (skips?)", {8000, 12, 4, 40}, c_full);
    run("A2c 13.4 t empty trailer, full throttle from 12 km/h in 3 (ts3b: wants 3->5)", {13400, 12, 3, 40}, c_full);
    run("A3 10 t solo, light throttle 0.4 from 12 km/h, Normal", {10000, 12, 4, 60}, c_light);
    run("A4 same, Eco", {10000, 12, 4, 60, 10}, c_light);
    run("A5 same, Power", {10000, 12, 4, 60, 1.6667f}, c_light);
    run("A6 same, Normal, slow box 1.2 s (skips?)", {10000, 12, 4, 60, 3, 1.2f}, c_light);
    run("B1 10 t solo, driver holds 80 flat (low rpm?)", {10000, 80, 10, 90}, c_solo80);
    run("B2 40 t, driver holds 85 on 1%", {40000, 85, 11, 90}, c_cruise85);
    run("B3 BeamNG test: 40 t holds 80, 3.5% from t=30 to 90", {40000, 80, 11, 150}, c_beamng);
    run("B3b BeamNG trap: 14 t solo seen as 10 t, holds 80, 3.5% t=30-90", {14000, 80, 11, 150, 3, 0.5f, 10000}, c_beamng);
    run("B3c same, Eco", {14000, 80, 11, 150, 10, 0.5f, 10000}, c_beamng);
    run("B3d long trap: 25 t seen as 14 t, holds 80 on rolling hills", {25000, 80, 11, 400, 3, 0.5f, 14000}, c_hills);
    run("B4 40 t holds 80 on rolling hills +-3%", {40000, 80, 11, 240}, c_hills);
    run("B5 40 t cruise control 80 on 1% (bang-bang throttle)", {40000, 80, 11, 90}, c_cc);
    run("B6 26 t adaptive cruise 80 behind a truck at 50", {26000, 60, 10, 60}, c_acc);
    run("C1 40 t, full throttle, 6% climb from t=10 s", {40000, 80, 11, 60}, c_climb);
    run("C2 40 t, 70 km/h at 0.35, kickdown at t=10 s", {40000, 70, 11, 25}, c_kick);
    run("C3 40 t, braking 0.25 from 80 km/h", {40000, 80, 11, 25}, c_brake);
    run("C3b same, a 30 km/h junction 250 m ahead on the route (earlier downshifts)", {40000, 80, 11, 25, 3, 0.5f, 0, -1, -1, r_flat, 1, 250}, c_brake);
    run("D1 40 t, 5% down holding 70, engine brake (level unknown = 0.5, toggle off)", {40000, 72, 12, 40}, c_eb);
    run("D2 40 t, 5% down holding 70, retarder 1/5", {40000, 72, 12, 40}, c_ret1);
    run("D3 40 t, 5% down holding 70, retarder 5/5", {40000, 72, 12, 40}, c_ret5);
    run("D4 40 t, 5% down holding 70, engine brake, game toggle on (full)", {40000, 72, 12, 40, 3, 0.5f, 0, 1}, c_eb);
    run("D5 40 t, 5% down holding 70, engine brake stage 1/3, toggle off", {40000, 72, 12, 40, 3, 0.5f, 0, 0, 0.33f}, c_eb);
    run("E1 40 t, 0.6 throttle, hint down at t=15 s", {40000, 50, 9, 35}, c_hint);
    run("E2 40 t low speed: 0.5 pedal from 0, brake, creep, stop", {40000, 0, 1, 70}, c_lowspeed);
    // M with m_style 2 (assisted: rev limit, stall, kickdown, stopping, lugging) here, 0 and 1 at the end: the sim carries
    // its state from run to run, so more runs here would move every result after them
    set_param("m_style", 2);
    printf("\n--- m_style = 2 ---");
    run_m();
    set_param("m_style", 0);

    run("R1 40 t holds 80, 4% climb at 1.5-2.5 km, NO route", {40000, 80, 11, 150, 3, 0.5f, 0, -1, -1, r_climb, 0}, c_hold80);
    run("R1b same WITH route (climb ahead)", {40000, 80, 11, 150, 3, 0.5f, 0, -1, -1, r_climb, 1}, c_hold80);
    run("R2 40 t full pedal 3% to a top at 700 m, NO route", {40000, 55, 10, 70, 3, 0.5f, 0, -1, -1, r_crest, 0}, c_full_road);
    run("R2b same WITH route (top ahead)", {40000, 55, 10, 70, 3, 0.5f, 0, -1, -1, r_crest, 1}, c_full_road);
    run("V1 12 t holds 70, lifts off at 10 s with a junction 60 m ahead, full pedal at 14 s", {12300, 70, 11, 25, 3, 0.5f, 0, -1, -1, r_flat, 1, 254}, c_lift_jn);
    run("V3 same as V1 but the game reports throttle 0.08 at pedal 0, starting in 10", {12300, 70, 10, 25, 3, 0.5f, 0, -1, -1, r_flat, 1, 254}, c_lift_jn_bug);
    run("V2 same, no junction, pedal stays off (rolls out)", {12300, 70, 11, 25, 3, 0.5f, 0, -1, -1, r_flat, 1, 0}, c_lift_jn_free);
    run("R3 26 t 0.5 pedal from 30, junction at 500 m, WITH route", {26000, 30, 6, 40, 3, 0.5f, 0, -1, -1, r_flat, 1, 500}, c_half);
    run("R4 40 t at 80, pedal released t=10-50 on -0.8% (eco-roll, Normal)", {40000, 80, 12, 70, 3, 0.5f, 0, -1, -1, r_down, 0}, c_release);
    run("R4b same in Power (no roll)", {40000, 80, 12, 70, 1.6667f, 0.5f, 0, -1, -1, r_down, 0}, c_release);
    g_cruise_smooth = 1; g_pcc_on = 1;
    run("R5 26 t cruise 80 on hills +-1.5% (roll with cruise, Eco)", {26000, 80, 12, 240, 10, 0.5f, 0, -1, -1, r_hills, 1}, c_cc80);
    run("R5b same, eco-roll off via Power", {26000, 80, 12, 240, 1.6667f, 0.5f, 0, -1, -1, r_hills, 1}, c_cc80);
    g_cruise_smooth = 0; g_pcc_on = 0;
    // PCC band (Damien 11-10): a top within reach is taken in this gear (at most -5 km/h), a long climb still downshifts.
    for (float b : std::initializer_list<float>{0.0f, 5.0f}) {
        set_param("crest_band_kmh", b);
        printf("\n--- crest_band_kmh = %.0f ---", b);
        run("T0 40 t cruise 80, 150 m at 3 % hump, route", {40000, 80, 12, 110, 3, 0.5f, 0, -1, -1, r_hump3, 1}, c_cci80);
        run("T0b 40 t cruise 80, 400 m at 1.5 %, route", {40000, 80, 12, 110, 3, 0.5f, 0, -1, -1, r_hump15, 1}, c_cci80);
        run("T0c 26 t cruise 80, 400 m at 1.5 %, route", {26000, 80, 12, 110, 3, 0.5f, 0, -1, -1, r_hump15, 1}, c_cci80);
        run("T1 40 t cruise 80, 400 m at 3 % then flat, route", {40000, 80, 12, 120, 3, 0.5f, 0, -1, -1, r_short3, 1}, c_cci80);
        run("T2 40 t cruise 80, 300 m at 5 % then flat, route", {40000, 80, 12, 120, 3, 0.5f, 0, -1, -1, r_short5, 1}, c_cci80);
        run("T3 40 t cruise 80, 2 km at 4 %, route (must downshift)", {40000, 80, 12, 160, 3, 0.5f, 0, -1, -1, r_long4, 1}, c_cci80);
        run("T2b 40 t cruise 80, 800 m at 3 %, route", {40000, 80, 12, 140, 3, 0.5f, 0, -1, -1, r_mid3, 1}, c_cci80);
        run("T4 12 t cruise 80, 300 m at 5 %, route", {12000, 80, 12, 120, 3, 0.5f, 0, -1, -1, r_short5, 1}, c_cci80);
        run("T6 40 t cruise 80 on rolling hills +-1.5 %, route", {40000, 80, 12, 300, 3, 0.5f, 0, -1, -1, r_hills, 1}, c_cci80);
        run("T6b same, Eco", {40000, 80, 12, 300, 10, 0.5f, 0, -1, -1, r_hills, 1}, c_cci80);
        run("T6c 40 t cruise 80, hills +-2.5 % every 1 km, route", {40000, 80, 12, 300, 3, 0.5f, 0, -1, -1, r_hills25, 1}, c_cci80);
        run("T5 40 t full pedal 55, 5 % to a top at 700 m, route", {40000, 55, 10, 70, 3, 0.5f, 0, -1, -1, r_crest, 1}, c_full_road);
        for (float gg : { 5.8f, 6.5f, 7.0f, 7.5f, 8.0f }) {
            g_dipg = gg; char nm[160];
            snprintf(nm, sizeof nm, "H1 30.4 t cruise 73, %.1f %% climb with a %.1f %% dip 60 m long (ride 3: 8 -> 9 -> 8), Eco, NO route", gg, gg - 1.7f);
            run(nm, {30400, 60, 9, 150, 10, 1.0f, 0, -1, -1, r_dip, 0}, c_cci73);
            run("H1b same WITH route", {30400, 60, 9, 150, 10, 1.0f, 0, -1, -1, r_dip, 1}, c_cci73);
        }
    }
    set_param("crest_band_kmh", getenv("BAND0") ? 0 : 5);
    // PCC momentum: only what the next climb needs (Damien 11-10: solo differs from loaded). Foot of the climb 500 m ahead.
    {
        float (*roads[])(double) = { r_hump15, r_short3, r_short5, r_mid3, r_long4 };
        const char *rn[] = { "400 m 1.5 %", "400 m 3 %", "300 m 5 %", "800 m 3 %", "2 km 4 %" };
        printf("\nPCC need (km/h over set 80 at the foot):\n");
        for (float mass : { 8000.f, 12000.f, 26000.f, 40000.f }) {
            printf("  %2.0f t:", mass / 1000);
            for (int r = 0; r < 5; r++) {
                run("  (need)", {mass, 80, 12, 40, 3, 0.5f, 0, -1, -1, roads[r], 1}, c_cci80);
                M.s = 1000;  // after 40 s on the flat (the force bias has settled): the foot of the climb 500 m ahead
                for (int i = 0; i < 40; i++) tick(1 / 60.0, 0);  // the frame works the need out every 0.5 s
                printf("  %s %.1f", rn[r], g_pcc_need_kmh);
            }
            printf("\n");
        }
    }

    // Robustness (polish round): resets in the middle of a drive must not break the drive; unsupported trucks stay hands-off.
    g_rst_every = 7; g_rst_level = RESET_SOFT;
    run("X1 40 t holds 80 on rolling hills, SOFT reset every 7 s (pause menu)", {40000, 80, 11, 240}, c_hills);
    g_rst_every = 11; g_rst_level = RESET_HARD;
    run("X2 same, HARD reset every 11 s (teleport / load: learned values gone)", {40000, 80, 11, 240}, c_hills);
    g_rst_every = 0;
    {
        scs_telemetry_configuration_t cfg = { "truck", NULL };
        T.fwd_gears = 2;
        m_trueshift.config(&cfg);
        run("X3 electric / 2-gear truck: TrueShift must leave the gears to the game", {40000, 80, 2, 60}, c_full);
        T.fwd_gears = 12;
        sim_reset(RESET_HARD, "sim: back to the diesel");
    }
    float st;
    run("L1 13.4 t, Eco, game starts in 4 at 12 km/h, full throttle (ride 12-10 1 > 4 > 5 > 6)", {13400, 12, 4, 50, 10.0f, 1.0f}, c_full);
    run("L2 13.4 t, Eco, from 14 km/h in 3, light throttle 0.4", {13400, 14, 3, 50, 10.0f, 1.0f}, c_light);
    // The start-gear checks are from a standstill (else the engage-at-speed path answers): L1/L2 end at ~70 km/h.
    // s_start reads the pedal from T (set by the physics step only), so set it here too.
    M.v = 0; T.speed = 0; T.in_thr = 0.5f;
    m_trueshift.frame(); M.mass = 26000; M.grade_pct = 0; M.pedal = 0.5f; st = (float)s_start(SELF); printf("\nstart gear 26 t flat: %g (game 1)\n", st);
    M.mass = 40000; M.grade_pct = 8; M.pedal = 0.5f; st = (float)s_start(SELF); printf("\nstart gear 40 t on 8%%: %g (game 1)\n", st);
    M.mass = 10000; M.grade_pct = 0; st = (float)s_start(SELF); printf("start gear 10 t flat: %g (game 4)\n", st);
    M.mass = 10000; M.grade_pct = 10; st = (float)s_start(SELF); printf("start gear 10 t on 10%%: %g (game 4)\n", st);
    // Ride 13-10 (617 s, 2087 s): 25 t, full pedal, downhill: 4->1 (start) at 12 km/h, 2000 rpm, then 1->4 (rev limit skip).
    M.mass = 25000; M.grade_pct = 0; T.in_thr = 1.0f; st = (float)s_start(SELF); printf("\nstart gear 25 t flat, full pedal: %g (game 1)\n", st);
    M.grade_pct = -4; st = (float)s_start(SELF); printf("start gear 25 t on -4%%, full pedal: %g (game 1)\n", st);
    M.v = T.speed = 12 / 3.6f; st = (float)s_start(SELF); printf("engage 25 t on -4%%, full pedal, rolling 12 km/h: %g (game 1; expect a gear near 1000-1300 rpm, not 1)\n", st);
    M.v = 0; T.speed = 0; M.mass = 10000; M.grade_pct = 10; T.in_thr = 0.5f; (void)s_start(SELF);  // back to the state the car runs started from

    // m_style 0 European (no shift of our own) and 1 protected (rev limit, stall only)
    for (float st : std::initializer_list<float>{0.0f, 1.0f}) {
        set_param("m_style", st);
        printf("\n--- m_style = %.0f ---", st);
        run_m();
    }
    set_param("m_style", 0);

    use_renault();
    run("C10 Renault 14 gears 32.5 t, Eco, cruise 46 (ACC behind a slow lead), flat then a 10.5 % climb of 300 m from 43 km/h in 12 (ride 24-10, 545 s)", {32500, 43, 12, 45, 10.0f, 1.0f, 0, -1, -1, r_climb10, 1}, c_cc46);
    run("Y1 Renault 16.4 t yard: 13 km/h in 6, a short brake, then pedal 0.3 (ride 24-10, 3856 s: 6>4>5>3>4>5>6>7>8>9 in 21 s)", {16400, 13, 6, 25, 10.0f, 1.0f, 0, -1, -1, r_flat, 0}, c_yard);
    run("C11 Renault 32.5 t, Eco, cruise 52 (ACC), flat then a 6 % climb of 400 m from 50 km/h in 13: a gear that can just hold it", {32500, 50, 13, 50, 10.0f, 1.0f, 0, -1, -1, r_climb6, 1}, c_cc52);
    run("C12 same on a long 3 % climb of 600 m from 52 km/h in 13", {32500, 52, 13, 60, 10.0f, 1.0f, 0, -1, -1, r_climb3, 1}, c_cc52);
    run("C10b same from 55 km/h in 13", {32500, 55, 13, 45, 10.0f, 1.0f, 0, -1, -1, r_climb10, 1}, c_cc63);
    // start gear of the Renault box by load (24-10: 36.7 t started in 2 and climbed 2>3>4>5 in 7 s)
    {
        M.v = 0; T.speed = 0; M.grade_pct = 0;
        for (float tons : { 16.4f, 25.0f, 34.6f, 36.7f, 42.0f })
            for (float ped : { 0.06f, 0.25f, 0.6f }) {
                M.mass = tons * 1000; M.pedal = ped; T.in_thr = ped; m_trueshift.frame();
                printf("Renault start gear %4.1f t, pedal %.2f: %d (game 1)\n", tons, ped, (int)s_start(SELF));
            }
    }
    run("G1 Renault 36.7 t at a standstill, the game started in 2, pedal 0.3 (24-10, 324 s: 2>3>4>5 in 7 s)", {36700, 0, 2, 25, 10.0f, 1.0f, 0, -1, -1, r_flat, 0}, c_pull03);
    use_car();
    run("CAR1 2.5 t pickup, light 0.35 from 15 km/h", {2500, 15, 2, 40}, c_car_light);
    run("CAR2 2.5 t pickup, full throttle from 15", {2500, 15, 2, 25}, c_car_full);
    run("CAR3 2.5 t pickup holds 100", {2500, 100, 6, 60}, c_car_100);
    // Reset scenarios, last: they move the clock and the module's once-a-second polls, which would shift the runs above a little.
    use_volvo();
    sim_reset(RESET_HARD, "sim: the Volvo again");
    g_tp_done = 0; g_tp_up_at = -1;
    { int f0 = n_failed, i0 = g_n_incidents;
    run("X4 40 t full pedal on 6 %, at 20 s a teleport (HARD reset) onto 2 % at the same speed", {40000, 45, 8, 60}, c_teleport);
    // the upshift must come within 10 s (the 6 % peak is gone), and it must hold: demand and f_need start from the pedal of now, not from 0
    // (from 0 the full pedal looked like a light one: a skip to a gear without pull, back down, a hunt incident)
    int downs_after = shifts - ups - g_tp_downs;
    printf("  first upshift %.1f s after the teleport (must be within 10 s); downshifts after it %d, no-pull lessons %d, incidents %d (all 0)\n",
           g_tp_up_at < 0 ? -1.0 : g_tp_up_at - 20, downs_after, n_failed - f0, g_n_incidents - i0);
    if (!(g_tp_up_at >= 0 && g_tp_up_at - 20 < 10) || downs_after || n_failed != f0 || g_n_incidents != i0) { printf("  X4 FAIL\n"); g_x_failed = 1; } }
    // X5: the gearbox parked in its gap state (gear 0, a target, clutch open) at 40 km/h, a pause (SOFT reset) 1.0 s into it. The stuck-in-neutral
    // timer must start again at the reset: the engage then comes 0.3 s after the pause (the pause's gap is normal), not at once as 'stuck for 1.0 s'.
    {
        memset(&M, 0, sizeof M); M.mass = 40000; M.shift_time = 0.5f; M.cvar = 3; M.v = 40 / 3.6f; M.gear = 0; M.target = 8; M.parked = 1;
        t_now += 10;
        g_stuck_s = -1; int i0 = g_n_incidents; double t0 = t_now;
        for (int k = 0; k < 120 && M.parked; k++) {
            t_now += 1 / 60.0; T.sim_time = (scs_timestamp_t)(t_now * 1e6);
            T.speed = M.v; T.gear = 0; T.dgear = 0; T.in_thr = 0; T.eff_thr = 0; T.eff_clutch = 1; T.engine_on = 1; T.rpm = 600;
            if (k == 60) sim_reset(RESET_SOFT, "sim: pause menu");
            s_tail(SELF);
        }
        printf("\nX5 gap state at 40 km/h, a pause 1.0 s into it: engaged %.2f s after the pause as 'stuck for %.1f s' (expect ~0.3 and 0.3), incidents %d (expect 0)\n",
               t_now - t0 - 1.0, g_stuck_s, g_n_incidents - i0);
        if (!(g_stuck_s >= 0 && g_stuck_s < 0.5f && g_n_incidents == i0 && !M.parked)) { printf("  X5 FAIL\n"); g_x_failed = 1; }
    }
    return g_x_failed;
}
