// Offline test of TrueShift's decisions (modules/trueshift.cpp) on a simple
// longitudinal truck model. Emulates the game's automatic: per tick the decide
// stub, the stock window trigger, the chooser (a rough stock chooser below the
// takeover speed), a shift with a torque gap.
// Build+run: tools/sim/run.sh [-v]   (-v prints every log line)
#include "core/suite.h"
#include "core/game.h"
#include "core/route.h"
#include <math.h>
#include <initializer_list>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

telemetry T;
volatile int g_rolling, g_cruise_smooth, g_crawl_on;
volatile float g_roll_boost_kmh;
volatile float g_pcc_need_kmh = NAN;
int g_log_level = 2;
incident_ctx g_inc = { 0, 0, 0, NAN, NAN };
double g_incident_mb = 0;
int g_n_incidents;
void logring(const char *, ...) {}
void incident(const char *kind, const char *fmt, ...)
{
    g_n_incidents++;
    char b[300]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    printf("   INCIDENT %s: %s\n", kind, b);
}
const char *g_truck_unsupported;
volatile float g_creep_brake;
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
static int verbose, n_failed;
void logline(const char *fmt, ...)
{
    if (strstr(fmt, "no pull after")) n_failed++;
    int shift = !strncmp(fmt, "shift", 5) || strstr(fmt, "no pull after") || !strncmp(fmt, "load changed", 12);
    if (!verbose && !shift) return;
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
static double rpm_sum, rpm_t;   // time-weighted rpm in the second half

static int g_rst_every = 0, g_rst_level = 0;   // robustness runs: inject a session reset every N seconds
static void tick(double dt, int stats)
{
    t_now += dt;
    if (g_rst_every > 0 && fmod(t_now, g_rst_every) < dt && m_trueshift.reset) m_trueshift.reset(g_rst_level, "sim");
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
    m_trueshift.frame();
    // gearbox
    if (M.state && t_now >= M.shift_end) {
        int dir = M.target > M.gear ? 1 : -1;
        if (dir > 0 && M.gear > 0) { ups++; up_rpm_sum += rpm_now(); up_n++; }
        if (last_dir && dir != last_dir && t_now - last_shift_t < 5) hunts++;
        last_dir = dir; last_shift_t = t_now;
        M.gear = M.target; M.state = 0; shifts++;
    }
    // The game's update: in gear the decide site + chooser; in neutral (game,
    // docs/lka-roll-re.md B) re-engage on throttle via the start-gear fn; then
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
        // until there is throttle (docs/neutral-exit-re.md § 1); gb_engage leaves it.
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
}

struct setup { float mass, v0; int g0; double secs; float cvar = 3, shift_time = 0.5f, mass_seen = 0; int eb_toggle = -1; float eb_level = -1;
               float (*road)(double) = NULL; int route = 0; double junction = 0; };
static int rolls; static double roll_time, min_kmh;

static void run(const char *name, setup su, void (*ctl)(double))
{
    printf("\n=== %s ===\n", name);
    memset(&M, 0, sizeof M);
    M.mass = su.mass; M.gear = M.target = su.g0; M.cvar = su.cvar; M.shift_time = su.shift_time;
    M.eb_level = su.eb_level; M.eb_toggle = su.eb_toggle; M.mass_seen = su.mass_seen;
    M.road = su.road; M.route_on = su.route; M.junction_s = su.junction;
    M.v = 0; tick(1 / 60.0, 0);  // register the gear at standstill (no hint)
    M.state = 0; M.gear = M.target = su.g0; M.force = 0;
    M.v = su.v0 / 3.6f; tick(1 / 60.0, 0);
    M.state = 0; M.gear = M.target = su.g0; M.force = 0;
    M.v = su.v0 / 3.6f; shifts = ups = hunts = last_dir = 0; up_rpm_sum = 0; up_n = 0; rpm_sum = rpm_t = 0;
    M.s = 0; rolls = 0; roll_time = 0; min_kmh = 1e9;
    int f0 = n_failed;
    double t0 = t_now, next_print = t0;
    while (t_now - t0 < su.secs) {
        ctl(t_now - t0);
        int was0 = M.gear == 0;
        tick(1 / 60.0, t_now - t0 > su.secs / 2);
        if (M.gear == 0 && M.v > 1) { roll_time += 1 / 60.0; if (!was0) rolls++; }
        if (t_now - t0 > 3 && M.v * 3.6 < min_kmh) min_kmh = M.v * 3.6;
        if (t_now >= next_print) { next_print += 10;
            printf("        t=%5.1f  %5.1f km/h  gear %2d  rpm %4.0f  pedal %.2f cc %.2f grade %.1f%% %c\n", t_now - t0,
                   M.v * 3.6f, M.gear, rpm_now(), M.pedal, M.cc, M.grade_pct, letter); }
    }
    printf("  -> %d shifts (%d up, avg up-landing %.0f rpm), %d hunting pairs, %d no-pull lessons, end %.1f km/h gear %d, "
           "mean rpm 2nd half %.0f, min %.0f km/h, %d rolls %.0f s, %.0f m\n", shifts, ups, up_n ? up_rpm_sum / up_n : 0,
           hunts, n_failed - f0, M.v * 3.6f, M.gear, rpm_t > 0 ? rpm_sum / rpm_t : 0, min_kmh, rolls, roll_time, M.s);
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

int main(int argc, char **argv)
{
    verbose = argc > 1 && !strcmp(argv[1], "-v");
    for (int i = 0; i < m_trueshift.n_params; i++) *m_trueshift.params[i].val = m_trueshift.params[i].def;
    T.fwd_gears = 12; T.diff = 2.85f; T.wheels = 4; T.retarder_steps = 5;
    for (int i = 0; i < 12; i++) T.ratio[i] = RATIO[i];
    G.tail_site = 5; G.decide_site = 1; G.n_choose = 1; G.choose_sites[0] = 2; G.n_start = 1; G.start_sites[0] = 4;
    G.window_fn = (uintptr_t)f_window; G.chooser_fn = (uintptr_t)f_choose; G.start_fn = (uintptr_t)f_start;
    m_trueshift.start();
    if (getenv("BAND0")) set_param("crest_band_kmh", 0);
    if (getenv("NOBOOST")) set_param("eco_roll_boost_kmh", 0);
    if (getenv("NOLOAD")) set_param("load_down_from", 1);

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
    g_cruise_smooth = 1;
    run("R5 26 t cruise 80 on hills +-1.5% (roll with cruise, Eco)", {26000, 80, 12, 240, 10, 0.5f, 0, -1, -1, r_hills, 1}, c_cc80);
    run("R5b same, eco-roll off via Power", {26000, 80, 12, 240, 1.6667f, 0.5f, 0, -1, -1, r_hills, 1}, c_cc80);
    g_cruise_smooth = 0;
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
        m_trueshift.reset(RESET_HARD, "sim: back to the diesel");
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
    return 0;
}
