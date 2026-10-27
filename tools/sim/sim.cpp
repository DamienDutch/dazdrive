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
#include "core/rpmspan.h"
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
volatile float g_decel_need;
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
float g_air_from_truck;   // [core] air_from_truck (core/actuate.cpp, not linked here)
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
static double g_dn1_t; static char g_dn1_why[48];   // set g_dn1_t to -1: the first downshift of ours after it, when and its reason, lands here
// g_rec_on: every shift of ours is recorded (when, from, to, why, rpm); the SCORES pull-away and stop rows read it
enum { REC_MAX = 128 };
static int g_rec_on, g_rec_n; static struct { double t; int a, b; char why[32]; float rpm; } g_rec[REC_MAX];
static float g_cf_rpm = -1, g_cf_share, g_lift_med = -1; static int g_cf_varied;   // SCORES part 4: the last 'choice floor' and 'lifts: median' lines
void logline(const char *fmt, ...)
{
    if (!strncmp(fmt, "choice floor:", 13)) {
        va_list ap; va_start(ap, fmt); g_cf_varied = !strcmp(va_arg(ap, const char *), "varies"); va_arg(ap, double);
        g_cf_rpm = (float)va_arg(ap, double); g_cf_share = (float)va_arg(ap, double); va_end(ap);
    }
    if (!strncmp(fmt, "lifts: median", 13)) { va_list ap; va_start(ap, fmt); g_lift_med = (float)va_arg(ap, double); va_end(ap); }
    if (strstr(fmt, "no pull after")) n_failed++;
    if (g_rec_on && g_rec_n < REC_MAX && !strncmp(fmt, "shift %d->%d (%s) at", 20)) {
        va_list ap; va_start(ap, fmt);
        g_rec[g_rec_n].t = t_now; g_rec[g_rec_n].a = va_arg(ap, int); g_rec[g_rec_n].b = va_arg(ap, int);
        snprintf(g_rec[g_rec_n].why, sizeof g_rec[g_rec_n].why, "%s", va_arg(ap, const char *));
        va_arg(ap, double); g_rec[g_rec_n].rpm = (float)va_arg(ap, double); va_end(ap);
        g_rec_n++;
    }
    if (g_dn1_t < 0 && !strncmp(fmt, "shift %d->%d (%s)", 17)) {
        va_list ap; va_start(ap, fmt); int a = va_arg(ap, int), b = va_arg(ap, int); const char *w = va_arg(ap, const char *); va_end(ap);
        if (b < a) { g_dn1_t = t_now; snprintf(g_dn1_why, sizeof g_dn1_why, "%s", w); }
    }
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
    float cvar, shift_time, cruise_kmh; int eb_toggle; float eb_level, mass_seen, thr_floor; int hint, wish_on; float wish_thr, brake_unseen;   // brake_unseen: a brake the telemetry does not show (AEB in the ride)   // hint: a shift-hint press the game's handler turns into target +-1 before its update
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
// a junction on the route is a 30 km/h bend (only when the scenario has one; g_jn_kmh: another speed)
static float g_jn_kmh = 30;
float route_curve_speed(float, float a_dec, float)
{
    if (!route_ok() || !(M.junction_s > M.s)) return NAN;
    float d = (float)(M.junction_s - M.s), vj = g_jn_kmh / 3.6f;
    return sqrtf(vj * vj + 2 * a_dec * d);
}
float route_junction_m() { return route_ok() && M.junction_s > M.s ? (float)(M.junction_s - M.s) : -1; }
float route_bend_speed(float, float) { return NAN; }   // the scenarios have no road bends, only the junction above
static float g_act_req;   // the brake request the suite itself holds (the deceleration assist in the ride; the telemetry does not show it)
float act_request() { return g_act_req; }
// The actuator's throttle wishes (core/actuate.cpp is not linked here): a scenario sets a cap or scale (the own speed limiter posts
// ACT_CRUISE's scale), and the throttle model below applies it, so TrueShift's thr_capped() sees it as in the game.
static act_wish g_act[ACT_N] = { {0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1},{0,0,1,1,1} };
const act_wish *act_wishes() { return g_act; }
static float sim_thr_cap()   // the merged throttle cap and scale, as the actuator applies them (base, then caps, then scales)
{
    float r = 1.0f;
    for (int o = 0; o < ACT_N; o++) if (g_act[o].cap < r) r = g_act[o].cap;
    for (int o = 0; o < ACT_N; o++) r *= g_act[o].scale;
    return r;
}
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
float gb_thr_eff(void *) { return fmaxf((M.wish_on ? M.wish_thr : fmaxf(M.pedal, M.cc)) * sim_thr_cap(), M.thr_floor); }  // thr_floor: the game's 0.08 at pedal 0 (24-10, 1831 s); wish_on: the wish pedal's throttle
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

static float g_idle_sim = 600;   // the idle of the engine of the sim (SCORES part 7 has one with 700)
static float rpm_now() { return M.gear > 0 ? M.v / RADIUS * RATIO[M.gear-1] * DIFF * 9.5493f : (TORQUE < 1000 ? 750 : g_idle_sim); }

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
int low_owner_on() { return 0; }   // the low-speed owner (lowspeed.cpp) is not part of this sim
int low_owner_wanted() { return 0; }
int low_start_gear() { return 0; }
int low_launch_hold() { return 0; }
int low_handed() { return 0; }
float low_stop_rpm(float idle, float, float, int) { return 0.8f * idle; }
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
static struct { int on, on60; double kj, rs, rt, pk, kj60, t60; } EN;   // the energy table: engine energy (kJ, torque x rpm x time), rpm in gear, over a window a controller opens and closes
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
    float idle = TORQUE < 1000 ? 750 : g_idle_sim;
    if (rpm < idle) rpm = idle;   // clutch slips below idle
    T.rpm = rpm; T.speed = M.v; T.in_thr = M.pedal; T.eff_thr = (M.wish_on ? M.wish_thr : fmaxf(M.pedal, M.cc)) * sim_thr_cap(); g_pedal_mapped = M.wish_on;
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
    float F = 0, thr = (M.wish_on ? M.wish_thr : fmaxf(M.pedal, M.cc)) * sim_thr_cap();
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
    float Fres = M.mass * 9.81f * (s + 0.006f) + 0.5f * 1.2f * CDA * M.v * M.v + (M.brake + M.brake_unseen) * M.mass * 4;
    if ((EN.on || EN.on60) && M.gear > 0 && !M.state) {
        engine_info e; gb_engine(0, &e); float r = rpm_now();
        double kj = TORQUE * curve_factor(&e, r < idle ? idle : r) * thr * r * 0.10472f * dt / 1000;
        if (EN.on) { EN.kj += kj; EN.rs += r * dt; EN.rt += dt; EN.pk = fmax(EN.pk, r); }
        if (EN.on60) EN.kj60 += kj;
    }
    if (EN.on60) { EN.t60 += dt; if (EN.t60 >= 60) EN.on60 = 0; }
    M.v += (F - Fres) / M.mass * dt;
    if (M.v < 0) M.v = 0;
    M.s += M.v * dt;
    if (stats) { rpm_sum += rpm_now() * dt; rpm_t += dt; }
    if (M.state) g_sh_t += dt; else if (M.gear > 0) { g_rpm_s += rpm_now() * dt; g_rpm_t += dt; }
    if (M.gear > 0 && !M.state && M.v * 3.6f < 50) { rpm_all_sum += rpm_now() * dt; rpm_all_t += dt; }
}

struct setup { float mass, v0; int g0; double secs; float cvar = 3, shift_time = 0.5f, mass_seen = 0; int eb_toggle = -1; float eb_level = -1;
               float (*road)(double) = NULL; int route = 0; double junction = 0; float pedal0 = 0; };   // pedal0: the pedal from the first tick on (the module's demand starts from it)
static int rolls; static double roll_time, min_kmh, t_30, t_50, t_80, d_80;

static void run(const char *name, setup su, void (*ctl)(double))
{
    if (!g_quiet) printf("\n=== %s ===\n", name);
    memset(&M, 0, sizeof M);
    for (int o = 0; o < ACT_N; o++) g_act[o] = act_wish{0, 0, 1, 1, 1};
    M.mass = su.mass; M.gear = M.target = su.g0; M.cvar = su.cvar; M.shift_time = su.shift_time;
    M.eb_level = su.eb_level; M.eb_toggle = su.eb_toggle; M.mass_seen = su.mass_seen;
    M.road = su.road; M.route_on = su.route; M.junction_s = su.junction; M.pedal = su.pedal0;
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
static int run_scores_part4();
static int run_scores_cruise();
static int run_scores_approach();
static int run_scores_engine();
static int run_scores_cross();
static int run_scores_kickwish();
static int run_scores_skipfrac();
static int run_scores_limiter();
static int run_scores()
{
    printf("\n=== SCORES part 1: the score function on the vehicle model (Renault ATO 12 speeds, diff 3.08) ===\n");
    use_ato();
    if (getenv("SC_PART9")) return run_scores_kickwish();   // (only part 9)
    if (getenv("SC_PART8")) return run_scores_cross();   // (only part 8)
    if (getenv("SC_PART7")) return run_scores_engine();   // (only part 7)
    if (getenv("SC_PART6")) return run_scores_approach();   // (only part 6; SC_VERBOSE=1 with -v prints the module log)
    if (getenv("SC_PART4")) return run_scores_part4();   // (only part 4, for a quick look; its numbers may differ a little from the full run)
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
    int r = run_scores_table();   // (part 3 after it: its many sc_prime calls move the clock, and the rows of part 2 depend on the clock's float steps)
    printf("\n=== SCORES part 3: the score function with choice_objective 1 ===\n");
    // choice_objective 1 (the time to gain speed picks the gear up), 33 t, 1.0 s box, a firm pedal: a skip may win where the gear skipped to
    // cannot give all the force asked (the score alone never takes such a gear), and Eco goes as far up as Power or farther
    int short_skip = 0, t_far = 0, t_near = 0, n_up = 0;
    for (float kmh = 10; kmh <= 70; kmh += 2.5f)
        for (float pedal : { 0.4f, 0.5f, 0.6f, 0.7f, 0.8f })
            for (int cur = 2; cur <= 11; cur++) {
                int best[3];
                for (int md = 0; md < 3; md++) {
                    sc_prime(33000, 1.0f, kmh, cur);
                    gs_cfg c = sc_cfg(md, 2);
                    const char *k = md == 1 ? "choice_time_shift_eco" : md == 2 ? "choice_time_shift_power" : "choice_time_shift";
                    for (int j = 0; j < m_trueshift.n_params; j++) {
                        if (!strcmp(m_trueshift.params[j].key, k)) c.k_shift = *m_trueshift.params[j].val;
                        if (!strcmp(m_trueshift.params[j].key, "choice_min_accel")) c.a_min = *m_trueshift.params[j].val;
                    }
                    c.time_obj = 1;
                    gs_eval(&o, cur, NG, M.v, sc_need(cur, pedal), vehicle_resist(M.v), 0, c);
                    best[md] = o.best > cur ? o.best : 0;
                    if (md == 0 && best[0]) { n_up++; if (best[0] > cur + 1 && o.t[best[0]].share > 1) short_skip++; }
                }
                if (best[1] && best[2]) { if (best[1] > best[2]) t_far++; if (best[1] < best[2]) t_near++; }
            }
    printf("  time objective, 33 t, 1.0 s box, 10-70 km/h, pedal 0.4-0.8: Normal skips to a gear short of the force asked in %d of %d upshifts; Eco higher than Power in %d, lower in %d\n",
           short_skip, n_up, t_far, t_near);
    sc_check(short_skip > 0 && t_near == 0 && t_far > 0, "time objective: a skip can win under load, Eco skips farther than Power, never less");
    r |= run_scores_part4();
    r |= run_scores_cruise();   // (last: its runs move the clock too)
    r |= run_scores_approach();
    r |= run_scores_engine();
    r |= run_scores_cross();
    r |= run_scores_kickwish();
    r |= run_scores_skipfrac();
    r |= run_scores_limiter();
    return r | g_sc_failed;
}

// ---- part 2: choice 0 (the cascade) against choice 1 (the scores) in the simulator
struct sc_res { int shifts, skips; double shift_s, rpm, t50; int hunts; };
static sc_res sc_last;
static void sc_do(const char *name, setup su, void (*ctl)(double), int choice)
{
    set_param("choice", (float)choice);
    // a short drive first: the module keeps a once-per-standstill flag (the launch correction) until the truck moves, and a run that ends
    // in a crawl would give the one after it a different start than its twin. In the scenario's mode: the module reads the mode once a
    // second, at a phase the run before sets, so the first second of an Eco or Power run was Normal or not depending on that run
    setup wash = {20000, 30, 6, 2}; wash.cvar = su.cvar;
    g_quiet = 1; run("wash", wash, c_hold80); g_quiet = 0;
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
// c_fullramp: pedal 0.55 (from the first tick: setup.pedal0) for 3 s, then a ramp by 0.4/s to 1.0 (too slow for a kickdown). g_fr_full: when
// it reached 1.0 (-1 = not yet); the first downshift from the start of the ramp on is caught (g_dn1_t)
static double g_fr_ramp, g_fr_full;
static void c_fullramp(double t)
{
    if (t <= 0) { g_fr_ramp = g_fr_full = -1; g_dn1_t = 0; }
    if (g_fr_ramp < 0 && t >= 3) { g_fr_ramp = t; g_dn1_t = -1; g_dn1_why[0] = 0; }
    M.pedal = g_fr_ramp < 0 ? 0.55f : fminf(1.0f, 0.55f + 0.4f * (float)(t - g_fr_ramp));
    if (g_fr_full < 0 && M.pedal >= 1.0f) g_fr_full = t_now;
}
// c_cc83: the cruise control holding 83 km/h (a smooth controller); r_cruise3: 2 km flat, then 1 km at 3 %, then flat
static void c_cc83(double) { M.cruise_kmh = 83; M.pedal = 0; float e = 83 / 3.6f - M.v; M.cc = fminf(1, fmaxf(0, 0.25f + e * 1.0f)); }
static float r_cruise3(double s) { return s < 2000 ? 0 : s < 3000 ? 3.0f : 0; }
static double g_c12_up; static int g_c12_g12;   // the first time in 12; 1 = 12 at the end of the climb
static void c_cruise3(double t)
{
    if (t <= 0) { g_c12_up = -1; g_c12_g12 = 0; }
    c_cc83(t);
    if (g_c12_up < 0 && M.gear == 12 && !M.state) g_c12_up = t;
    if (M.s >= 2950 && M.s < 3000) g_c12_g12 = M.gear == 12;
}
// c_pull: a fixed pedal (g_pl_ped) from a standstill; the shifts until 70 km/h (or the end) are counted from the recorder. g_pl_wish = 1: the
// engine's throttle is the wish pedal's (a power wish, the throttle that delivers it in this gear, as modules/pedal.cpp does it in Normal,
// in gear above 12 km/h), the driver's pedal stays where it is, and TrueShift reads the wish (pedal_wish_force below)
static float g_pl_ped = 0.7f; static int g_pl_wish; static double g_pl_t0, g_pl_t50, g_pl_t70, g_pl_sh70;
static int g_pl_full;   // test only: the wish is the pedal's share of the power the truck can deliver at this speed in the best gear (ref_now), no blend to the plain pedal
static float cf_full(float v);
static float sim_wish_force(float v)   // the wish pedal's force at v for the pedal g_pl_ped (exponent 1, no rise limit)
{
    if (g_pl_full) return g_pl_ped * cf_full(v);
    engine_info e; gb_engine(0, &e);
    float pref = 0;
    for (float r = 100; r <= LIMIT; r += 50) pref = fmaxf(pref, TORQUE * curve_factor(&e, r) * r * 0.10472f);
    int sg = ts_start_gear(0.3f);
    float fcap = sg > 0 ? vehicle_force_at(sg, vehicle_kmh_at(sg, V.plateau) / 3.6f, 1.0f) : 0;
    return fminf(g_pl_ped * pref * V.eta / fmaxf(v, 0.5f), g_pl_ped * fcap);
}
float pedal_wish_force(float v) { return M.wish_on ? sim_wish_force(v) : -1; }   // TrueShift reads the wish (suite.h)
static float cf_wish_force(float v);
float cruise_wish_force(float v) { return cf_wish_force(v); }   // [cruise] force_mode 1 (part 4's cruise model; -1 elsewhere)
static void c_pull(double t)
{
    if (t <= 0) { g_pl_t0 = t_now; g_pl_t50 = g_pl_t70 = -1; g_pl_sh70 = -1; g_rec_n = 0; EN = {}; EN.on = EN.on60 = 1; }
    M.pedal = g_pl_ped; M.wish_on = 0;
    if (g_pl_wish && M.gear > 0 && !M.state && M.v * 3.6f > 12) {   // the driver's pedal stays g_pl_ped (T.in_thr), the engine gets the wish's throttle
        float fg = vehicle_force_at(M.gear, M.v, 1.0f);
        if (fg > 0) { M.wish_on = 1; M.wish_thr = fminf(1, sim_wish_force(M.v) / fg); }
    }
    if (g_pl_t50 < 0 && M.v * 3.6f >= 50) g_pl_t50 = t;
    if (g_pl_t70 < 0 && M.v * 3.6f >= 70) { g_pl_t70 = t; g_pl_sh70 = g_sh_t; EN.on = 0; }
}
struct pl_res { int shifts, skips; double shift_s, t50, t70, gap; int gap_ok; };
static pl_res pl_stats(float ts)   // from the recorder: until 70 km/h (or the end of the run)
{
    pl_res r = {}; r.gap = 99; r.gap_ok = 1;
    double t_end = g_pl_t70 >= 0 ? g_pl_t0 + g_pl_t70 : 1e30, up_last = -1;
    for (int i = 0; i < g_rec_n; i++) {
        if (g_rec[i].t > t_end) break;
        r.shifts++;
        if (abs(g_rec[i].b - g_rec[i].a) > 1) r.skips++;
        if (g_rec[i].b > g_rec[i].a) {
            bool at_limit = !strncmp(g_rec[i].why, "rev limit", 9);
            if (up_last >= 0) { r.gap = fmin(r.gap, g_rec[i].t - up_last); if (g_rec[i].t - up_last < 1.5 * ts && !at_limit) r.gap_ok = 0; }
            up_last = g_rec[i].t;
        }
    }
    r.shift_s = g_pl_sh70 >= 0 ? g_pl_sh70 : g_sh_t;
    r.t50 = g_pl_t50; r.t70 = g_pl_t70;
    return r;
}
static const char *g_pl_label[3] = { "choice 0", "choice 1, same force", "choice 1, time" };
static pl_res pl_row(const char *name, setup su, int var)   // var: 0 = choice 0, 1 = choice 1 with the previous objective, 2 = choice 1 with the time
{
    set_param("choice_objective", var == 1 ? 0.0f : 1.0f);
    g_rec_on = 1;
    sc_do(name, su, c_pull, var > 0);
    g_rec_on = 0;
    set_param("choice_objective", 1);
    pl_res r = pl_stats(su.shift_time);
    char t50[16], t70[16];
    snprintf(t50, sizeof t50, r.t50 < 0 ? "   -" : "%4.1f", r.t50); snprintf(t70, sizeof t70, r.t70 < 0 ? "   -" : "%4.1f", r.t70);
    printf("  %-24s %-22s %6d %6d %8.1f  %s  %s %8.1f\n", name, g_pl_label[var], r.shifts, r.skips, r.shift_s, t50, t70, r.gap);
    if (getenv("SC_SHIFTS")) for (int i = 0; i < g_rec_n; i++) printf("      %6.1f  %d->%d (%s) %.0f rpm\n", g_rec[i].t - g_pl_t0, g_rec[i].a, g_rec[i].b, g_rec[i].why, g_rec[i].rpm);
    return r;
}
// c_stop: from the start speed with the pedal off, g_st_coast seconds coasting, then the brake g_st_brake to a stop; the stall rule and the lowest
// rpm in gear above 5 km/h (the game's own range below the handback) are counted
static float g_st_brake = 0.2f, g_st_coast = 0, g_st_rpm_min; static int g_st_stall, g_st_unseen;   // g_st_unseen: the brake is not in the telemetry
static void c_stop(double t)
{
    if (t <= 0) { g_st_rpm_min = 1e9f; g_st_stall = 0; g_rec_n = 0; g_pl_t0 = t_now; }
    M.pedal = 0;
    float b = t < g_st_coast ? 0 : fminf(g_st_brake, g_st_brake * (float)(t - g_st_coast) / 0.5f);
    if (g_st_unseen) M.brake_unseen = b; else M.brake = b;
    if (M.gear > 0 && !M.state && M.v * 3.6f >= 5) g_st_rpm_min = fminf(g_st_rpm_min, rpm_now());
}
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
    // 3d. full pedal in a gear under the torque plateau (ride: 34.6 t, gear 10, 41 km/h, 809 rpm, pedal and throttle 1.00: 0.3-0.5 m/s2 and
    // gear 10 held for 5 s, 'hold', until the kickdown rule fired). The pull the pedal asks is not what this gear gives: a lower gear
    // within 1.5 s of the full pedal, not by kickdown (the ramp of 0.4/s is too slow for one)
    printf("\n  full pedal in 10 at 41 km/h (34.6 t ATO, flat; pedal 0.55 for 3 s, then a ramp to 1.0 by 0.4/s): the first downshift\n");
    {
        double dt_dn[2]; char wy[2][48]; int in10[2];
        for (int ch = 0; ch < 2; ch++) {
            sc_do("full pedal in 10", {34600, 41, 10, 15, 3, 0.5f, 0, -1, -1, NULL, 0, 0, 0.55f}, c_fullramp, ch);
            in10[ch] = g_dn1_t != 0;
            dt_dn[ch] = g_dn1_t < 0 || g_fr_full < 0 ? 99 : g_dn1_t - g_fr_full;
            snprintf(wy[ch], sizeof wy[ch], "%s", g_dn1_t < 0 ? "none" : g_dn1_why);
            if (!in10[ch]) printf("  choice %d: never in 10 at 41 km/h\n", ch);
            else printf("  choice %d: the ramp from %.1f s, first downshift: %s, %+.1f s after the full pedal\n", ch, g_fr_ramp, wy[ch], dt_dn[ch]);
        }
        g_dn1_t = 0;
        sc_check(in10[1] && dt_dn[1] <= 1.5 && strcmp(wy[1], "kickdown"), "choice 1: full pedal in a gear under the plateau downshifts within 1.5 s of the full pedal, not by kickdown");
    }
    // 3e. a steady cruise (ride: 83 km/h on the cruise control, 20-30 % throttle, 11 at 1262 rpm held 38 s, 'hold', against 12 at ~990 rpm, which
    // then held the climb that followed): 12 is chosen on the flat and keeps the 3 % climb (22 t: 12 can hold it). The vehicle model learns
    // its resistance bias in the first ~25 s after the hard reset (the demanded acceleration is off until then)
    printf("\n  cruise control 83 km/h in 11 (22 t ATO, route), 2 km flat, then 1 km at 3 %%: when 12 comes, and 12 at the end of the climb\n");
    {
        double up12[2]; int keep12[2];
        for (int ch = 0; ch < 2; ch++) {
            sc_do("cruise 83 then 3 %", {22000, 83, 11, 140, 3, 0.5f, 0, -1, -1, r_cruise3, 1}, c_cruise3, ch);
            up12[ch] = g_c12_up; keep12[ch] = g_c12_g12;
            char at[24] = "never";
            if (up12[ch] >= 0) snprintf(at, sizeof at, "at %.1f s", up12[ch]);
            printf("  choice %d: in 12 %s, 12 at the end of the climb: %s, %d shifts\n", ch, at, keep12[ch] ? "yes" : "no", shifts);
        }
        sc_check(up12[1] >= 0 && up12[1] <= 40 && keep12[1], "choice 1: a steady cruise at 83 km/h takes 12 within 40 s and keeps it on the 3 % climb");
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
    // 5. a firm pedal from a standstill to 70 km/h on a slow box (ride: 17 t, pedal 0.66-0.77, 1->3, 3->4, 4->5, 5->6, 6->7 every 1.8-2.8 s): the
    // shifts, skips, seconds in shifts, time to 50 and 70 km/h and the shortest time between two upshifts, until 70 km/h
    printf("\n  pull-away to 70 km/h, ATO, Normal, 1.0 s box, until 70 km/h: shifts, skips, seconds in shifts, time to 50 / 70 km/h, shortest time between two upshifts (gear changes)\n");
    printf("  %-24s %-22s %6s %6s %8s  %4s  %4s %8s\n", "scenario", "", "shifts", "skips", "shift s", "t50", "t70", "up gap");
    {
        pl_res r[3], w[3], h[3], l[3];
        for (int v = 0; v < 3; v++) r[v] = pl_row("33 t pedal 0.7", {33000, 1, 1, 120, 3, 1.0f}, v);
        for (int v = 0; v < 3; v++) pl_row("33 t pedal 0.7 Eco", {33000, 1, 1, 120, 10, 1.0f}, v);
        for (int v = 0; v < 3; v++) pl_row("33 t pedal 0.7 Power", {33000, 1, 1, 120, 1.6667f, 1.0f}, v);
        g_pl_wish = 1;
        for (int v = 0; v < 3; v++) w[v] = pl_row("33 t wish pedal 0.7", {33000, 1, 1, 120, 3, 1.0f}, v);
        for (int v = 0; v < 3; v++) h[v] = pl_row("17 t wish pedal 0.7", {17000, 1, 1, 90, 3, 1.0f}, v);
        g_pl_wish = 0; g_pl_ped = 0.3f;
        for (int v = 0; v < 3; v++) l[v] = pl_row("17 t pedal 0.3", {17000, 1, 1, 90, 3, 1.0f}, v);
        g_pl_ped = 0.7f;
        sc_check(r[2].shifts < r[0].shifts && r[2].shift_s < r[0].shift_s && r[2].t70 > 0 && r[2].t70 <= r[0].t70 + 0.05,
                 "choice 1 (time), 33 t pedal 0.7: fewer shifts and seconds in shifts than choice 0, 70 km/h no later");
        sc_check(r[2].gap_ok && w[2].gap_ok && h[2].gap_ok && l[2].gap_ok, "choice 1 (time): no two upshifts closer than 1.5 x the shift time below the rev limit");
        sc_check(l[2].skips >= l[1].skips, "choice 1 (time), 17 t pedal 0.3: at least as many skips as the previous objective");
    }
    // 6. braking to a stop from 70 km/h (ride: 10->8 'braking' only at 20 km/h and 638 rpm, then the stall rule 8->6 at 419 rpm and 6->1): no
    // stall-rule shift, and the revs above 5 km/h never under the idle - 50
    printf("\n  braking to a stop from 70 km/h (ATO, Normal, pedal off): shifts, stall-rule shifts, lowest rpm in gear above 5 km/h\n");
    {
        struct St { const char *name; float mass, brake, coast; int unseen; } st[5] = { { "17 t brake 0.2", 17000, 0.2f, 0, 0 }, { "17 t coast 4 s, brake 0.6", 17000, 0.6f, 4, 0 },
            { "17 t coast 4 s, unseen 0.6", 17000, 0.6f, 4, 1 }, { "33 t brake 0.3", 33000, 0.3f, 0, 0 }, { "33 t brake 1.0", 33000, 1.0f, 0, 0 } };
        int bad = 0;
        for (auto &s : st)
            for (int v = 0; v < 3; v++) {
                g_st_brake = s.brake; g_st_coast = s.coast; g_st_unseen = s.unseen;
                set_param("choice_objective", v == 1 ? 0.0f : 1.0f);
                g_rec_on = 1;
                sc_do(s.name, {s.mass, 70, sc_gear_for(70, 1100), 40, 3, 1.0f}, c_stop, v > 0);
                g_rec_on = 0;
                set_param("choice_objective", 1);
                for (int i = 0; i < g_rec_n; i++) if (!strcmp(g_rec[i].why, "stall")) g_st_stall++;
                char seq[160] = ""; int k = 0;
                for (int i = 0; i < g_rec_n && k < (int)sizeof seq - 8; i++) k += snprintf(seq + k, sizeof seq - k, i ? ">%d" : "%d>%d", i ? g_rec[i].b : g_rec[i].a, g_rec[i].b);
                printf("  %-26s %-22s %2d shifts, %d stall, lowest %4.0f rpm  %s\n", s.name, g_pl_label[v], g_rec_n, g_st_stall, g_st_rpm_min, seq);
                if (getenv("SC_SHIFTS")) for (int i = 0; i < g_rec_n; i++) printf("      %6.1f  %d->%d (%s) %.0f rpm\n", g_rec[i].t - g_pl_t0, g_rec[i].a, g_rec[i].b, g_rec[i].why, g_rec[i].rpm);
                if (v == 2 && (g_st_stall > 0 || g_st_rpm_min < 600 - 50)) bad++;
            }
        sc_check(bad == 0, "choice 1: no stall-rule shift on a stop, the revs above 5 km/h never under idle - 50");
    }
    return g_sc_failed;
}

// ---- part 4: the power-reserve floor, our own brake request, the route's cap and the upshift on a lift
// the ride's hunt (33.1 t ATO, 1.0 s box, 0.7 %, a junction ahead; 166.8 s: 8->7->9->7 in 8.1 s at 33 km/h): in 8 at 45 km/h the pedal off and
// the engine brake at 0.25, then the deceleration assist brakes (its request does not show in the telemetry) to 33 km/h, the driver pulls at
// 0.45, then lifts (2.5 s after an upshift landed, else after 7 s) with the engine brake at 0.25 and the assist braking a little
static float r_07(double) { return 0.7f; }
static int g_hn_ph; static double g_hn_t, g_hn_up, g_hn_up_s, g_hn_hold = 7;   // g_hn_up_s: where the first upshift to 9 landed (m)
static void c_hunt(double t)
{
    if (t <= 0) { g_hn_ph = 0; g_hn_t = 0; g_hn_up = -1; g_hn_up_s = -1; g_rec_n = 0; g_pl_t0 = t_now; }
    M.pedal = 0; M.brake = 0; M.brake_unseen = 0; g_act_req = 0; M.ebrake = 0;
    if (g_hn_ph == 0 && t >= 1.0) g_hn_ph = 1;
    if (g_hn_ph == 1 && M.v * 3.6f <= 33) { g_hn_ph = 2; g_hn_t = t; }
    if (g_hn_ph == 2 && g_hn_up < 0 && M.gear >= 9 && !M.state) { g_hn_up = t; g_hn_up_s = M.s; }
    if (g_hn_ph == 2 && (t >= g_hn_t + g_hn_hold || (g_hn_up >= 0 && t >= g_hn_up + 2.5))) g_hn_ph = 3;
    if (g_hn_ph == 0) M.ebrake = 1;
    else if (g_hn_ph == 1) { M.brake_unseen = 0.2f; g_act_req = 0.09f; }
    else if (g_hn_ph == 2) M.pedal = 0.45f;
    else { M.ebrake = 1; M.brake_unseen = 0.02f; g_act_req = 0.01f; }
}
static int sc_hunts()   // the recorded shifts: three in a row that each reverse the direction within 10 s (TrueShift's 'hunt' incident, this run only)
{
    int n = 0;
    for (int i = 2; i < g_rec_n; i++)
        if (g_rec[i].t - g_rec[i - 2].t < 10 && (g_rec[i - 2].b - g_rec[i - 2].a) * (g_rec[i - 1].b - g_rec[i - 1].a) < 0 &&
            (g_rec[i - 1].b - g_rec[i - 1].a) * (g_rec[i].b - g_rec[i].a) < 0) n++;
    return n;
}
static void sc_seq(char *out, int n)   // the recorded shifts as 9>8>7 ...
{
    int k = 0; out[0] = 0;
    for (int i = 0; i < g_rec_n && k < n - 8; i++) k += snprintf(out + k, n - k, i ? ">%d" : "%d>%d", i ? g_rec[i].b : g_rec[i].a, g_rec[i].b);
}
// the lifts: the pedal at 0.45 (g_lf_ped), off at g_lf_at for g_lf_dur seconds, then back; the grade g_lf_grade
static float g_lf_at = 2, g_lf_dur = 4, g_lf_grade = 0, g_lf_ped = 0.45f;
static void c_lift(double t)
{
    if (t <= 0) { g_rec_n = 0; g_pl_t0 = t_now; }
    M.grade_pct = g_lf_grade;
    M.pedal = t >= g_lf_at && t < g_lf_at + g_lf_dur ? 0.0f : g_lf_ped;
}
// a driver who lifts the same way again and again (the session's lifts are learned): on for g_tc_on s (0.45 under 45 km/h, else 0.2), off for g_tc_off s
static float g_tc_on = 4, g_tc_off = 3;
static void c_teach(double t)
{
    float ph = (float)fmod(t, g_tc_on + g_tc_off);
    M.pedal = ph < g_tc_on ? (M.v * 3.6f < 45 ? 0.45f : 0.2f) : 0.0f;
}
// a steady foot at 0.15, or one wavering 0.05-0.25 every 5 s (g_fv_amp 0.1); the floor lines are caught from the start of the run on
static float g_fv_amp; static void c_floorped(double t)
{
    if (t <= 0) { g_cf_rpm = -1; g_cf_varied = 0; }
    M.grade_pct = 0.6f; M.pedal = 0.15f + g_fv_amp * sinf((float)t * 6.2832f / 5);
}
static int run_scores_part4()
{
    printf("\n=== SCORES part 4: the power-reserve floor, our own brake request, the route's cap, the upshift on a lift (ATO) ===\n");
    use_ato();
    const char *keys[6] = { "choice_floor_power_eco", "choice_floor_power", "choice_floor_power_power", "choice_own_brake", "choice_route_cap", "lift_upshift" };
    float def[6], sf_def = 0.3f;
    for (int i = 0; i < 6; i++) for (int j = 0; j < m_trueshift.n_params; j++) if (!strcmp(m_trueshift.params[j].key, keys[i])) def[i] = m_trueshift.params[j].def;
    for (int j = 0; j < m_trueshift.n_params; j++) if (!strcmp(m_trueshift.params[j].key, "skip_land_frac")) sf_def = m_trueshift.params[j].def;
    {   // the floor rpm per mode: the lowest rpm where the engine still gives that share of its peak power (the sim's curve, 2700 Nm)
        float id = ts_idle_rpm(), vd = 0.025f;
        for (int j = 0; j < m_trueshift.n_params; j++) if (!strcmp(m_trueshift.params[j].key, "choice_floor_power_varied")) vd = m_trueshift.params[j].def;
        printf("\n  the power-reserve floor (peak power %.0f kW; share of it -> lowest rpm, + choice_floor_power_varied -> while the pedal varies):\n", V.eng.torque * V.peak_pw * 0.10472f / 1000);
        for (int i = 0; i < 3; i++)
            printf("    %-7s %.3f -> %4.0f rpm   (varied %.3f -> %4.0f rpm)\n", i == 0 ? "Eco" : i == 1 ? "Normal" : "Power", def[i], gs_power_rpm(def[i], id), def[i] + vd, gs_power_rpm(def[i] + vd, id));
        printf("    (for comparison, the shares 0.725 / 0.775 / 0.825 on this engine: %.0f / %.0f / %.0f rpm)\n", gs_power_rpm(0.725f, id), gs_power_rpm(0.775f, id), gs_power_rpm(0.825f, id));
        sc_check(gs_power_rpm(def[0], id) < gs_power_rpm(def[1], id) && gs_power_rpm(def[1], id) < gs_power_rpm(def[2], id) && gs_power_rpm(def[1] + vd, id) > gs_power_rpm(def[1], id), "the floor rises from Eco to Normal to Power, and while the pedal varies");
    }
    {   // the floor is the higher of the power term and a distance above idle that is a share of the engine's span (choice_floor_idle_frac)
        float fr = 0.5f;
        for (int j = 0; j < m_trueshift.n_params; j++) if (!strcmp(m_trueshift.params[j].key, "choice_floor_idle_frac")) fr = m_trueshift.params[j].def;
        printf("\n  the floor rpm of choice 1 = max(power term, idle + %.2f x (plateau - idle)), per engine and mode:\n", fr);
        printf("    %-22s %6s %7s  %-7s %11s %11s %7s\n", "engine", "idle", "plateau", "mode", "power term", "idle term", "floor");
        struct Eg { const char *name; void (*use)(); } egs[2] = { { "Renault DE13 (ATO)", use_ato }, { "Volvo (2400 Nm)", use_volvo } };
        float res[2][3];
        for (int e = 0; e < 2; e++) {
            egs[e].use(); sc_prime(17000, 1, 45, 8);
            float id = ts_idle_rpm();
            for (int i = 0; i < 3; i++) {
                float pt = gs_power_rpm(def[i], id), it = id + fr * fmaxf(0.0f, V.plateau - id), f = gs_floor_rpm(def[i], id, fr);
                res[e][i] = f;
                printf("    %-22s %6.0f %7.0f  %-7s %11.0f %11.0f %7.0f\n", egs[e].name, id, V.plateau, i == 0 ? "Eco" : i == 1 ? "Normal" : "Power", pt, it, f);
                sc_check(f >= pt - 0.5f && f >= it - 0.5f && (f == pt || f == it), "the floor is the higher of the two terms");
            }
        }
        sc_check(res[0][0] <= res[0][1] && res[0][1] <= res[0][2] && res[1][0] <= res[1][1] && res[1][1] <= res[1][2], "the floor rises from Eco to Normal to Power on both engines");
        use_ato(); sc_prime(17000, 1, 45, 8);
    }
    // 1. the hunt of the ride, with the new keys off (as before) and at their defaults
    printf("\n  the ride's hunt: 33.1 t, 0.7 %%, a turn 130 m ahead (5 km/h); in 8 at 45 km/h the engine brake 0.25, an assist brake the telemetry\n"
           "  does not show to 33 km/h, pedal 0.45, then a lift with the engine brake 0.25 (2.5 s after an upshift, else after 7 s; then 10 s)\n");
    int hunt_new = -1, hunt_old = -1, inc_new = -1, up_before = 0;
    const char *rows[6] = { "new keys off (as before)", "only choice_floor_power*", "only choice_own_brake", "only choice_route_cap", "only lift_upshift", "new keys at their defaults" };
    for (int kk = 0; kk < 12; kk++) {
        int k = kk % 6; g_hn_hold = kk < 6 ? 7 : 10;
        if (k == 0) printf("  pedal up to %.0f s:\n", g_hn_hold);
        for (int i = 0; i < 6; i++) set_param(keys[i], k == 5 || (k == 1 && i < 3) || (k > 1 && i == k + 1) ? def[i] : 0.0f);
        set_param("skip_land_frac", k == 5 ? sf_def : 0.0f);   // (the ride is reproduced with the new key off; its default is part of the "at their defaults" row)
        g_rec_on = 1;
        setup su = {33100, 45, 8, 25, 3, 1.0f, 0, -1, 0.25f, r_07, 1, 130};
        g_jn_kmh = 5;
        sc_do("hunt", su, c_hunt, 1);
        g_jn_kmh = 30;
        g_rec_on = 0;
        char seq[160]; sc_seq(seq, sizeof seq);
        char up[48] = ""; if (g_hn_up_s >= 0) snprintf(up, sizeof up, " (up to 9 at %.0f m)", g_hn_up_s);
        int hi = sc_hunts();
        printf("    %-28s %2d shifts, %d hunting pairs, %d hunts  %s%s\n", rows[k], g_rec_n, hunts, hi, seq, up);
        if (getenv("SC_SHIFTS")) for (int i = 0; i < g_rec_n; i++) printf("      %6.1f  %d->%d (%s) %.0f rpm\n", g_rec[i].t - g_pl_t0, g_rec[i].a, g_rec[i].b, g_rec[i].why, g_rec[i].rpm);
        if (k == 5) { if (kk < 6) hunt_new = hunts; inc_new = inc_new > hi ? inc_new : hi; if (g_hn_up_s >= 0 && g_hn_up_s < su.junction) up_before++; }
        else if (k == 0 && kk == 0) hunt_old = hi;
    }
    g_act_req = 0;
    sc_check(hunt_old > 0, "the scenario hunts with the new keys off (it reproduces the ride: 8>7>9>7)");
    g_hn_hold = 7;
    printf("  (pedal 10 s: the sim's driver drives through the turn at ~40 km/h; past it nothing is ahead, an upshift is right and the engine brake\n"
           "  of the lift takes it back: one pair, no hunt)\n");
    sc_check(hunt_new == 0 && inc_new == 0 && up_before == 0, "new keys at their defaults: no hunting pair (pedal 7 s), no hunt (three reversals in 10 s) and no upshift before the turn (7 s and 10 s)");
    // 2. the floor at 45 km/h, 17 t: a steady foot against one wavering 0.30-0.54 every 5 s
    printf("\n  the floor from 45 km/h, 17 t, Normal on 0.6 %% (choice_floor_power %.3f, + %.3f while the pedal varies): a steady pedal 0.15 against one wavering 0.05-0.25\n", def[1], 0.025f);
    float fl_st = gs_power_rpm(def[1], ts_idle_rpm()), fl_var = 0; double rpm_st = 0, rpm_var = 0; int sh_st = 0, sh_var = 0, var_seen = 0;
    {
        g_cf_rpm = -1; g_cf_varied = 0;
        g_fv_amp = 0;
        sc_do("floor steady", {17000, 45, sc_gear_for(45, 1100), 40}, c_floorped, 1);
        rpm_st = g_rpm_t > 0 ? g_rpm_s / g_rpm_t : 0; sh_st = shifts; float v_st = M.v * 3.6f;
        int st_var = g_cf_rpm > 0 && g_cf_varied;
        g_fv_amp = 0.1f;
        sc_do("floor varied", {17000, 45, sc_gear_for(45, 1100), 40}, c_floorped, 1);
        g_fv_amp = 0;
        rpm_var = g_rpm_t > 0 ? g_rpm_s / g_rpm_t : 0; sh_var = shifts; var_seen = g_cf_varied; fl_var = g_cf_rpm;
        printf("  steady pedal:  floor %4.0f rpm (%.3f of the peak power), mean rpm %4.0f, %d shifts, end %.0f km/h%s\n", fl_st, def[1], rpm_st, sh_st, v_st, st_var ? " (seen as varying!)" : "");
        printf("  varying pedal: floor %4.0f rpm (%.3f of the peak power), mean rpm %4.0f, %d shifts, end %.0f km/h\n", fl_var, g_cf_share, rpm_var, sh_var, M.v * 3.6f);
        sc_check(!st_var && var_seen && fl_var > fl_st, "the varying foot is seen as varying, the steady one not, and its floor is higher");
    }
    // 3. the upshift on a lift: a driver whose lifts are short first (median 0.8 s: under the shift time + 0.5 s), then one who lifts for 3 s
    printf("\n  the upshift on a lift (33.1 t, 1.0 s box, 8 at 40 km/h, pedal 0.45, off at 2 s): shifts started while the pedal is off\n");
    struct Lf { const char *name; float dur, grade; int want; } lf[4] = { { "short-lift driver, lift 4 s", 4, 0, 0 }, { "lift 0.5 s", 0.5f, 0, 0 }, { "lift 4 s", 4, 0, 1 }, { "lift 4 s on a -2 % descent", 4, -2, 0 } };
    int bad = 0;
    for (int i = 0; i < 4; i++) {
        if (i == 0 || i == 1) {   // teach the driver: 70 lifts, so all the remembered ones are this driver's
            g_tc_on = 1.5f; g_tc_off = i == 0 ? 0.8f : 3.0f; g_lift_med = -1;
            sc_do("teach", {33100, 40, 9, 70 * (g_tc_on + g_tc_off)}, c_teach, 1);
            printf("  (taught: 70 lifts of %.1f s, the median learned %.1f s)\n", g_tc_off, g_lift_med);
        }
        g_lf_dur = lf[i].dur; g_lf_grade = lf[i].grade;
        g_rec_on = 1;
        sc_do(lf[i].name, {33100, 40, 8, 10, 3, 1.0f, 0, -1, -1, NULL, 0, 0, g_lf_ped}, c_lift, 1);
        g_rec_on = 0;
        int n_off = 0, n_lift = 0; double first = -1;
        for (int j = 0; j < g_rec_n; j++) {
            double tr = g_rec[j].t - g_pl_t0;
            tr -= M.shift_time;   // (recorded when the gear is in: the shift started a shift time earlier)
            if (tr >= g_lf_at && tr < g_lf_at + g_lf_dur - 0.05f) { n_off++;   // (a shift that starts as the pedal comes back is an ordinary one, not a lift's)
                if (first < 0) first = tr - g_lf_at; }
            if (!strcmp(g_rec[j].why, "up on lift")) n_lift++;
        }
        char seq[160]; sc_seq(seq, sizeof seq);
        char fs[48] = ""; if (first >= 0) snprintf(fs, sizeof fs, ", the first %.1f s after the lift", first);
        printf("  %-30s %d shifts while off%s, %d 'up on lift'  %s\n", lf[i].name, n_off, fs, n_lift, seq);
        if (getenv("SC_SHIFTS")) for (int j = 0; j < g_rec_n; j++) printf("      %6.1f  %d->%d (%s) %.0f rpm\n", g_rec[j].t - g_pl_t0, g_rec[j].a, g_rec[j].b, g_rec[j].why, g_rec[j].rpm);
        if (lf[i].want ? !(n_lift == 1 && first >= 0.55 && first <= 0.9) : n_lift > 0 || (i < 3 && n_off > 0)) bad++;
    }
    g_lf_dur = 4; g_lf_grade = 0; g_tc_on = 4; g_tc_off = 3;
    sc_check(bad == 0, "lift 4 s: one upshift 0.6 s after the lift; a short-lift driver, a 0.5 s lift and a descent: none");
    for (int i = 0; i < 6; i++) set_param(keys[i], def[i]);
    set_param("skip_land_frac", sf_def);
    return g_sc_failed;
}

// ---- part 5: the cruise control's demand as a force ([cruise] force_mode, modules/cruise.cpp) with choice 1
// The own cruise controller as modules/cruise.cpp runs it in Normal (PI per km/h: kp 0.25, ki 0.08, output rate 0.6/s, twice that down,
// the look-ahead error of approach_s 2.5 s, held in a shift, no wind-up at the limits, 0 while rolling in neutral). force_mode 0: the output
// is the throttle and TrueShift sees only that; 1: the output is a share of a full wish (the wish pedal's reference, pedal_wish_full), the
// throttle is that force in the gear of now, the gains and the rate per throttle the same, and TrueShift reads the force (cruise_wish_force)
static struct { int on, fm; float i, u, k, acc_f, v_prev; double at; } CF = { 0, 0, 0, 0, 1, 0, 0, -1 };
static float cf_full(float v)   // pedal.cpp's wish_ref: ref_now power (the best gear in the usable window at full throttle), the start gear's cap
{
    float idle = ts_idle_rpm(), lo = fmaxf(idle + 0.6f * fmaxf(0.0f, V.plateau - idle), idle + 150);
    float hi = fminf(V.eng.range[R_POWER][1], 0.93f * V.eng.rpm_limit), best = 0;
    for (int g = 1; g <= T.fwd_gears; g++) { float r = vehicle_rpm_at(g, v); if (r >= lo && r <= hi) best = fmaxf(best, vehicle_force_at(g, v, 1.0f) * v); }
    float pref = best / V.eta;
    if (!(pref > 0)) for (float r = 100; r <= V.eng.rpm_limit; r += 50) pref = fmaxf(pref, V.eng.torque * curve_factor(&V.eng, r) * r * 0.10472f);
    int sg = ts_start_gear(0.3f);
    float fcap = sg > 0 ? vehicle_force_at(sg, vehicle_kmh_at(sg, V.plateau) / 3.6f, 1.0f) : 0;
    return pref > 0 && fcap > 0 ? fminf(pref * V.eta / fmaxf(v, 0.5f), fcap) : 0;
}
static float cf_wish_force(float v) { return CF.on && CF.fm && CF.at >= 0 && t_now - CF.at <= 0.3 ? CF.u * cf_full(v) : -1; }
static float g_cf_set = 83;
// [cruise] approach_by_mode as modules/cruise.cpp has it (g_cf_app 1; 2 = a test-only variant that asks all the pull to spare in every mode):
// the throttle the approach may use is the force that holds the speed plus the mode's share of what a full wish has to spare, over what the
// gear gives; the share is the mode's own key (approach_share_eco / approach_share / approach_share_power). More when the truck gains under 0.05 m/s2 while 1 km/h under.
static int g_cf_app = 0; static float g_cf_ex = 0, g_cf_ceil = 1;
static float g_cf_sh[3] = { 0.36f, 0.6f, 1.0f };   // approach_share_eco / approach_share / approach_share_power at their defaults (SC_SHARES=a,b,c overrides, for tuning)
static float cf_ceiling(int md, float e, float acc_kmhs, float dt, int shifting)
{
    bool slow = acc_kmhs < 0.18f && e > 1.0f;
    g_cf_ex = slow ? fminf(1.0f, g_cf_ex + 0.5f * dt) : fmaxf(0.0f, g_cf_ex - 0.5f * dt);
    if (shifting) return g_cf_ceil;
    float v = M.v, full = cf_full(v), fg = M.gear > 0 ? vehicle_force_at(M.gear, v, 1.0f) : 0;
    if (!(full > 0) || !(fg > 0)) return g_cf_ceil = 1;
    float fh = fmaxf(0.0f, vehicle_resist(v)), share = g_cf_app == 2 ? 1.0f : g_cf_sh[md == MODE_ECO ? 0 : md == MODE_POWER ? 2 : 1];
    if (full - fh < 0.25f * full) return g_cf_ceil = 1;
    share += (1 - share) * g_cf_ex;
    return g_cf_ceil = fminf(1.0f, fmaxf(0.0f, (fh + share * fmaxf(0.0f, full - fh)) / fg));
}
static void c_cforce(double t)
{
    const float dt = 1 / 60.0f;
    M.cruise_kmh = g_cf_set; M.pedal = 0;
    float kmh = M.v * 3.6f, e = g_cf_set + g_roll_boost_kmh - kmh;
    float a = t <= 0 ? 0 : (M.v - CF.v_prev) / dt * 3.6f; CF.v_prev = M.v;
    if (t <= 0) { CF.on = 0; CF.at = -1; CF.acc_f = 0; }
    CF.acc_f += (a - CF.acc_f) * fminf(1.0f, dt / 0.5f);
    int md = mode_of(M.cvar);   // (Eco / Power: the gains and the rate as cruise_body has them)
    float ep = CF.acc_f > 0 ? e - 2.5f * (md == MODE_ECO ? 1.2f : md == MODE_POWER ? 0.6f : 1.0f) * CF.acc_f : e;
    float kp = 0.25f, ki = 0.08f, rate = 0.6f, k = 1;
    if (md == MODE_ECO) { kp = 0.18f; ki = 0.05f; rate = 0.4f; } else if (md == MODE_POWER) { kp = 0.35f; ki = 0.12f; rate = 1.0f; }
    int shifting = M.state != 0;
    if (CF.fm) {
        float full = cf_full(M.v), fg = M.gear > 0 ? vehicle_force_at(M.gear, M.v, 1.0f) : 0;
        k = !shifting && full > 0 && fg > 0 ? full / fg : CF.k;
        CF.k = k; kp /= k; ki /= k; rate /= k;
    }
    if (!CF.on) { CF.on = 1; CF.u = M.cc / k; CF.i = CF.u - kp * ep; }
    if (g_rolling) { CF.u = 0; CF.i = -kp * e; M.cc = 0; CF.at = -1; return; }
    float u_raw = kp * ep + CF.i, u_cap = CF.fm ? fmaxf(1.0f, 1 / k) : 1.0f;
    if (g_cf_app && e > 2.5f) u_cap = fminf(u_cap, cf_ceiling(md, e, CF.acc_f, dt, shifting) / (CF.fm ? k : 1.0f)); else g_cf_ex = 0;
    float u = u_raw < 0 ? 0 : fminf(u_raw, u_cap), step = shifting ? 0 : rate * dt;
    if (u > CF.u + step) u = CF.u + step;
    if (u < CF.u - 2 * step) u = CF.u - 2 * step;
    int held_up = (u < u_raw - 1e-4f || (CF.fm && u * k > 1)) && e > 0, held_down = u > u_raw + 1e-4f && e < 0;
    if (!shifting && !held_up && !held_down) CF.i += ki * ep * dt;
    CF.i = fminf(CF.fm ? fmaxf(1.0f, 1 / k) : 1.0f, fmaxf(CF.fm ? -0.5f / k : -0.5f, CF.i));
    CF.u = u;
    M.cc = CF.fm ? fminf(1.0f, u * k) : u;
    CF.at = M.cc > 0 ? t_now : -1;
}
static float r_cf3(double s) { return s < 2000 ? 0 : s < 4000 ? 3.0f : 0; }   // 2 km flat, 2 km at 3 %, flat
static double g_cf_lo, g_cf_e, g_cf_end; static int g_cf_n;   // the lowest speed and the speed at the end of the climb, |e| on the flat after 30 s; the shifts go to the recorder
static void c_cforce3(double t)
{
    if (t <= 0) { g_cf_lo = 1e9; g_cf_e = 0; g_cf_n = 0; g_cf_end = -1; g_rec_n = 0; g_pl_t0 = t_now; }
    c_cforce(t);
    float kmh = M.v * 3.6f;
    if (M.s >= 2000 && M.s < 4000) g_cf_lo = fmin(g_cf_lo, kmh);
    if (M.s >= 3950 && M.s < 4000) g_cf_end = kmh;
    if (t > 30 && M.s < 2000) { g_cf_e += fabsf(kmh - g_cf_set); g_cf_n++; }
}
static int run_scores_cruise()
{
    printf("\n=== SCORES part 5: the cruise control 83 km/h with choice 1, force_mode 0 (TrueShift sees its throttle) against 1 (its force) ===\n");
    printf("  ATO, Normal, route, 2 km flat, then 2 km at 3 %%, flat; the own cruise controller (modules/cruise.cpp, Normal) on the simulator\n");
    printf("  %-24s %s  %6s %6s %8s %8s %9s %9s %9s %9s\n", "scenario", "fm", "shifts", "hunts", "shift s", "mean rpm", "|e| flat", "loss 3 %", "end 3 %", "gear end");
    use_ato();
    int bad = 0;
    for (float mass : { 17000.0f, 33000.0f }) {
        double lo[2], ef[2];
        for (int fm = 0; fm < 2; fm++) {
            CF.fm = fm;
            char nm[60]; snprintf(nm, sizeof nm, "cruise 83, %.0f t", mass / 1000);
            g_rec_on = 1;
            sc_do(nm, {mass, 83, 11, 230, 3, 0.5f, 0, -1, -1, r_cf3, 1}, c_cforce3, 1);
            g_rec_on = 0;
            lo[fm] = g_cf_lo; ef[fm] = g_cf_n ? g_cf_e / g_cf_n : 0;
            printf("  %-24s %d  %6d %6d %8.1f %8.0f %9.2f %9.1f %9.1f %9d\n", nm, fm, sc_last.shifts, sc_last.hunts, sc_last.shift_s, sc_last.rpm, ef[fm], g_cf_set - lo[fm], g_cf_end, M.gear);
            if (getenv("SC_SHIFTS")) for (int i = 0; i < g_rec_n; i++) printf("      %6.1f  %d->%d (%s) %.0f rpm\n", g_rec[i].t - g_pl_t0, g_rec[i].a, g_rec[i].b, g_rec[i].why, g_rec[i].rpm);
        }
        bad |= lo[1] < lo[0] - 1.0 || ef[1] > ef[0] + 0.2;
    }
    CF.fm = 0; CF.on = 0; CF.at = -1;
    sc_check(!bad, "force_mode 1: the cruise holds 83 on the flat and loses no more than 1 km/h more on the 3 % climb than force_mode 0");
    return g_sc_failed;
}

// ---- part 6: the cruise control's approach to the set speed and its hold, with choice 1 (ride 7: 33 t, 70 -> 83 km/h, 11 held 22 s before the 12th)
// The own cruise controller of part 5 in each mode (g_cruise_smooth on: TrueShift knows the throttle is the cruise's). Flat road, route on.
// A minute at 70 km/h first (the model of the resistance learns while the speed is steady), then the set speed goes to 83: the times are
// from that moment. Reports the approach (throttle at full for how long, the highest throttle, the time to the set speed) and when each gear came
static const double AP_PRE = 60;
static double g_ap_set_t, g_ap_wot, g_ap_thr_max, g_ap_t82; static int g_ap_g0; static float g_ap_cut_from = 1e9f, g_ap_cut_s = 0, g_ap_v0 = 70, g_ap_v1 = 83;
static void c_capp(double t)
{
    const double dt = 1 / 60.0;
    if (t <= 0) { g_ap_set_t = -1; g_ap_wot = 0; g_ap_thr_max = 0; g_rec_n = 0; g_pl_t0 = t_now; }
    g_cf_set = t < AP_PRE ? g_ap_v0 : g_ap_v1;
    c_cforce(t);
    float kmh = M.v * 3.6f;
    // a cut of the cruise throttle (a bend, a junction or a limit ahead): from g_ap_cut_from (s after the approach started) for g_ap_cut_s seconds the throttle is 0
    if (t >= AP_PRE + g_ap_cut_from && t < AP_PRE + g_ap_cut_from + g_ap_cut_s) { M.cc = 0; CF.at = -1; }
    if (t < AP_PRE) return;
    if (t - AP_PRE < dt) { EN = {}; EN.on = EN.on60 = 1; g_ap_t82 = -1; g_ap_g0 = M.gear; }
    if (EN.on && g_ap_t82 < 0 && kmh >= 82) { g_ap_t82 = t - AP_PRE; EN.on = 0; }
    if (g_ap_set_t < 0 && kmh >= g_cf_set - 0.5f) g_ap_set_t = t - AP_PRE;
    if (g_ap_set_t < 0) { if (M.cc > 0.98f) g_ap_wot += dt; g_ap_thr_max = fmax(g_ap_thr_max, M.cc); }
}
static float g_ap_gr; static float r_ap_gr(double) { return g_ap_gr; }   // a constant grade for the comparison table
static float r_ap_cut(double s) { return s < 1300 ? 2.0f : 0.0f; }   // 2 % up to 1.3 km (the pre-roll), then flat
static int sc_first_to(int gear)   // index of the first recorded shift into this gear, -1 = none
{
    for (int i = 0; i < g_rec_n; i++) if (g_rec[i].b == gear) return i;
    return -1;
}
static int run_scores_approach()
{
    printf("\n=== SCORES part 6: the cruise control's approach to 83 km/h and its hold with choice 1, 33 t, flat, ATO (own controller per mode) ===\n");
    use_ato();
    g_cruise_smooth = 1;
    int bad = 0;
    g_cf_set = 83;
    if (const char *sh = getenv("SC_SHARES")) sscanf(sh, "%f,%f,%f", &g_cf_sh[0], &g_cf_sh[1], &g_cf_sh[2]);
    printf("  from 70 km/h in 10 (1371 rpm):   mode    to 83   at full thr  highest thr   gears (s after the set speed was reached)\n");
    struct Md { const char *name; float cvar; } mds[3] = { { "Eco", 10 }, { "Normal", 3 }, { "Power", 1.6667f } };
    double t12[3] = { 1e9, 1e9, 1e9 };   // (1e9 = the 12th never came)
    double thr5[3] = { 0, 0, 0 };
    g_cf_app = 1;   // approach_by_mode at its default
    for (int i = 0; i < 3; i++) {
        CF.fm = 0;
        g_rec_on = 1;
        sc_do(mds[i].name, {33000, 70, 10, AP_PRE + 70, mds[i].cvar, 0.5f, 0, -1, -1, NULL, 1}, c_capp, 1);
        g_rec_on = 0;
        char gs[200] = ""; int k = 0;
        for (int j = 0; j < g_rec_n && k < 170; j++) if (g_rec[j].t - g_pl_t0 >= AP_PRE) k += snprintf(gs + k, sizeof gs - k, "%s%d@%.1f", k ? ", " : "", g_rec[j].b, g_rec[j].t - g_pl_t0 - AP_PRE - g_ap_set_t);
        int j12 = sc_first_to(12);
        if (j12 >= 0) t12[i] = g_rec[j12].t - g_pl_t0 - AP_PRE - g_ap_set_t;
        printf("                                   %-7s %5.1f s %6.1f s      %5.2f      %s\n", mds[i].name, g_ap_set_t, g_ap_wot, g_ap_thr_max, gs);
    }
    // a deficit of 5 km/h only (70 -> 75): the highest throttle and the seconds at full throttle, by mode
    printf("  a deficit of 5 km/h (70 -> 75), approach_by_mode 1:  mode    to 75   at full thr  highest thr\n");
    for (int i = 0; i < 3; i++) {
        CF.fm = 0; g_ap_v1 = 75;
        sc_do(mds[i].name, {33000, 70, 10, AP_PRE + 60, mds[i].cvar, 0.5f, 0, -1, -1, NULL, 1}, c_capp, 1);
        thr5[i] = g_ap_thr_max;
        printf("                                                    %-7s %5.1f s %6.1f s      %5.2f\n", mds[i].name, g_ap_set_t, g_ap_wot, g_ap_thr_max);
    }
    g_ap_v1 = 83; g_cf_app = 0;
    sc_check(thr5[0] < 0.99 && thr5[1] < 0.99 && thr5[2] > 0.99, "approach_by_mode 1: a 5 km/h deficit never gives full throttle in Eco or Normal, in Power it does");
    // ride 7: the 12th within a few seconds of arriving at the set speed (it took 22 s)
    sc_check(t12[1] <= 6, "ride 7, Normal: the 12th is in within 6 s of reaching the set speed");
    sc_check(t12[0] <= 8, "ride 7, Eco: the 12th is in within 8 s of reaching the set speed");
    // ---- the comparison table for the driver's choice: the cruise's approach and the driver's full pedal, energy as a fuel indication
    printf("\n  THE CHOICE (comparison only, nothing here changes a default). ATO, choice 1, energy = the integral of the engine's torque x rpm over the window (kJ).\n"
           "  (1) cruise approach 70 -> 83 km/h (window: until 82 km/h)\n");
    printf("      %-30s %-12s %10s %7s %6s %6s %6s %7s  %s\n", "variant", "load", "to 82 km/h", "shifts", "rpm pk", "rpm av", "kJ", "kJ 60 s", "gears");
    struct Va { const char *name; float cvar; int app; } vas[5] = { { "today (approach_by_mode 0)", 3, 0 }, { "approach_by_mode 1, Eco", 10, 1 }, { "approach_by_mode 1, Normal", 3, 1 },
                                                                     { "approach_by_mode 1, Power", 1.6667f, 1 }, { "full power (test only: force_mode 1, all of it)", 3, 2 } };
    for (int ld = 0; ld < 4; ld++) {
        float mass = ld < 2 ? 33000.0f : 17000.0f; float gr = ld % 2 ? 2.0f : 0.0f;
        char lb[24]; snprintf(lb, sizeof lb, "%.0f t %s", mass / 1000, gr > 0 ? "2 %" : "flat");
        for (int vi = 0; vi < 5; vi++) {
            CF.fm = vi == 4; g_ap_v0 = 70; g_ap_v1 = 83; g_ap_cut_from = 1e9f; g_cf_app = vi == 4 ? 0 : vas[vi].app; g_cf_ex = 0; g_cf_ceil = 1;
            g_ap_gr = gr;
            g_rec_on = 1;
            sc_do(vas[vi].name, {mass, 70, 10, AP_PRE + 90, vas[vi].cvar, 0.5f, 0, -1, -1, r_ap_gr, 1}, c_capp, 1);
            g_rec_on = 0; g_cf_app = 0; CF.fm = 0;
            char gs[160] = ""; int k = 0, n = 0;
            k += snprintf(gs, sizeof gs, "%d", g_ap_g0);
            for (int j = 0; j < g_rec_n && k < 140; j++) {
                double tr = g_rec[j].t - g_pl_t0 - AP_PRE;
                if (tr >= 0 && tr <= g_ap_t82 + 0.5) { k += snprintf(gs + k, sizeof gs - k, ">%d@%.0f", g_rec[j].b, tr); n++; }
            }
            char t82[12]; if (g_ap_t82 >= 0) snprintf(t82, sizeof t82, "%.1f s", g_ap_t82); else snprintf(t82, sizeof t82, "never");
            printf("      %-30s %-12s %10s %7d %6.0f %6.0f %6.0f %7.0f  %s\n", vas[vi].name, lb, t82, n, EN.pk, EN.rt > 0 ? EN.rs / EN.rt : 0, EN.kj, EN.kj60, gs);
        }
    }
    g_ap_gr = 0;
    printf("  (2) the driver's full pedal from 30 to 70 km/h (window: until 70 km/h)\n");
    printf("      %-64s %-12s %10s %7s %6s %6s %6s %7s  %s\n", "variant", "load", "to 70 km/h", "shifts", "rpm pk", "rpm av", "kJ", "kJ 60 s", "gears");
    for (int ld = 0; ld < 2; ld++) {
        float mass = ld ? 17000.0f : 33000.0f; char lb[24]; snprintf(lb, sizeof lb, "%.0f t flat", mass / 1000);
        for (int vi = 0; vi < 2; vi++) {
            g_pl_ped = 1.0f; g_pl_wish = vi; g_pl_full = vi;
            g_rec_on = 1;
            sc_do("full pedal", {mass, 30, sc_gear_for(30, 1100), 60}, c_pull, 1);
            g_rec_on = 0; g_pl_wish = 0; g_pl_full = 0; g_pl_ped = 0.7f;
            char gs[160] = ""; int k = 0, n = 0;
            k += snprintf(gs, sizeof gs, "%d", sc_gear_for(30, 1100));
            for (int j = 0; j < g_rec_n && k < 140; j++) if (g_pl_t70 >= 0 && g_rec[j].t - g_pl_t0 <= g_pl_t70 + 0.5) { k += snprintf(gs + k, sizeof gs - k, ">%d@%.0f", g_rec[j].b, g_rec[j].t - g_pl_t0); n++; }
            char t70[12]; if (g_pl_t70 >= 0) snprintf(t70, sizeof t70, "%.1f s", g_pl_t70); else snprintf(t70, sizeof t70, "never");
            printf("      %-64s %-12s %10s %7d %6.0f %6.0f %6.0f %7.0f  %s\n", vi ? "blend_from 1 (default): flat pedal = 100 % power at this speed" : "blend_from 0.85: flat pedal = full throttle in this gear", lb, t70, n, EN.pk, EN.rt > 0 ? EN.rs / EN.rt : 0, EN.kj, EN.kj60, gs);
        }
    }
    g_cf_app = 0;
    // a cut of the cruise throttle for a bend (not the driver's lift): 72 km/h on 2 % in 10 (the road flattens as the cut begins), the throttle 0 for 8 s,
    // the cruise back after it. The upshift the scores want (a flat road, no pull asked) must not wait for the cut to end
    {
        printf("\n  a cut of the cruise throttle for a bend (not the driver's lift): 72 km/h on 2 %% in 10, flat from the cut on, the throttle 0 for 8 s\n");
        CF.fm = 0; g_ap_v0 = g_ap_v1 = 72; g_ap_cut_from = 1.0f; g_ap_cut_s = 8;
        double up_s[2] = { -1, -1 };
        for (int i = 0; i < 2; i++) {   // Normal, Eco
            g_rec_on = 1;
            sc_do(i ? "cut Eco" : "cut Normal", {33000, 72, 10, AP_PRE + 20, i ? 10.0f : 3.0f, 0.5f, 0, -1, -1, r_ap_cut, 1}, c_capp, 1);
            g_rec_on = 0;
            char gs[120] = ""; int k = 0, up = -1;
            for (int j = 0; j < g_rec_n; j++) if (g_rec[j].t - g_pl_t0 >= AP_PRE) {
                if (up < 0 && g_rec[j].b > g_rec[j].a) up = j;
                k += snprintf(gs + k, sizeof gs - k, "%s%d>%d@%.1f", k ? ", " : "", g_rec[j].a, g_rec[j].b, g_rec[j].t - g_pl_t0 - AP_PRE - 1.0);
            }
            if (up >= 0) up_s[i] = g_rec[up].t - g_pl_t0 - AP_PRE - 1.0 - M.shift_time;
            char fs[16] = "-"; if (up >= 0) snprintf(fs, sizeof fs, "%.1f", up_s[i]);
            printf("    %-7s first upshift %s s after the cut began   %s\n", i ? "Eco" : "Normal", fs, gs);
        }
        sc_check(up_s[0] >= 0 && up_s[0] <= 4, "a cut of the cruise throttle: the upshift comes at once (within 4 s), not when the cut is over (Normal)");
        g_ap_cut_from = 1e9f; g_ap_v0 = 70; g_ap_v1 = 83;
    }
    g_cruise_smooth = 0;
    return g_sc_failed;
}

// ---- part 7: [trueshift] rpm_by_engine. The fixed rpm margins of the shift rules by the engine's own span (core/rpmspan.h), on today's curve
// (idle 600, plateau 900: inside the fitted band, so nothing may change) and on an engine with a plateau at 1300 and idle 700 (the same box on
// a 1.4 x longer axle), with the key 0 and 1: the margins, a pull-away at a light pedal and braking to a stop
static void c_p30r(double t) { if (t <= 0) g_rec_n = 0; c_p30(t); }   // (the shifts of the wash run before it are not the scenario's)
static void use_hp()
{
    use_ato();
    float c[8][2] = { {0,0},{800,0.55f},{1100,0.90f},{1300,0.98f},{1600,0.93f},{1850,0.72f},{2050,0.45f},{2250,0} };
    float g[4][2] = { {1750,2100},{950,1700},{1150,1500},{1450,1850} };
    memcpy(CURVE, c, sizeof c); memcpy(RANGES, g, sizeof g);
    LIMIT = 2100; DIFF = 4.3f; T.diff = DIFF; g_idle_sim = 700;
}
static int run_scores_engine()
{
    printf("\n=== SCORES part 7: rpm_by_engine 0 / 1 on today's curve (idle 600, plateau 900) and on a high-plateau engine (idle 700, plateau 1300) ===\n");
    struct Eg { const char *name; void (*use)(); float idle; } egs[2] = { { "today's curve", use_ato, 600 }, { "plateau 1300", use_hp, 700 } };
    char sig[2][2][1200] = {};   // per engine and key: everything the scenarios gave, to compare the keys on today's curve
#define P7(...) do { if (show) printf(__VA_ARGS__); } while (0)
    for (int e = 0; e < 2; e++) {
        // pass 0 is a warm-up (the module keeps some state across a reset, so a first run differs from a second one with the same key); passes 1 and 2 are
        // the keys 0 and 1 from the same start
        for (int pass = 0; pass < 3; pass++) {
            int key = pass == 2, show = pass > 0;
            egs[e].use();
            set_param("choice", 1.0f); set_param("idle_rpm", egs[e].idle); set_param("rpm_by_engine", (float)key);
            t_now = 5000; sim_reset(RESET_HARD, "sim: next engine");
            sc_prime(17000, 1, 45, 8);
            float id = egs[e].idle;
            P7("\n  %s, rpm_by_engine %d (idle %.0f, plateau %.0f, factor %.2f)\n", egs[e].name, key, id, V.plateau, rs_k(id));
            P7("    margins: stall idle+120 -> %.0f, idle+250 -> %.0f, idle+350 -> %.0f, plateau-100 -> %.0f | m_lug 1000 -> %.0f, air_charge 1200 -> %.0f, engine-brake light 1100 -> %.0f (brake range %.0f-%.0f) | LowSpeed hand-off +150 -> %.0f\n",
                   id + rs_off(id, 120), id + rs_off(id, 250), id + rs_off(id, 350), V.plateau - rs_off(id, 100), rs_abs(id, 1000), rs_abs(id, 1200), rs_brake(1100), V.eng.range[R_BRAKE][0], V.eng.range[R_BRAKE][1], id + rs_off(id, 150));
            char *sg = sig[e][pass > 0 ? key : 0]; int k = 0;
            k += snprintf(sg + k, 1200 - k, "m %.0f %.0f %.0f %.0f %.0f %.0f %.0f|", id + rs_off(id, 120), id + rs_off(id, 250), id + rs_off(id, 350), V.plateau - rs_off(id, 100), rs_abs(id, 1000), rs_abs(id, 1200), rs_brake(1100));
            P7("    pull-away, pedal 0.3, 90 s:    mass   shifts hunts shift s mean rpm  t50  gears\n");
            for (float mass : { 20000.0f, 36000.0f }) {
                g_rec_on = 1;
                sc_do("pull-away", {mass, 1, 1, 90, 3.0f}, c_p30r, 1);
                g_rec_on = 0;
                char seq[160] = ""; int q = 0;
                for (int i = 0; i < g_rec_n && q < (int)sizeof seq - 10; i++) q += snprintf(seq + q, sizeof seq - q, "%s%d", i ? ">" : "1>", g_rec[i].b);
                P7("                                  %5.0f t %6d %5d %7.1f %8.0f %5.1f  %s\n", mass / 1000, sc_last.shifts, sc_last.hunts, sc_last.shift_s, sc_last.rpm, sc_last.t50, seq);
                k += snprintf(sg + k, 1200 - k, "p%.0f %d %d %.1f %.0f %.1f %s|", mass, sc_last.shifts, sc_last.hunts, sc_last.shift_s, sc_last.rpm, sc_last.t50, seq);
            }
            P7("    braking to a stop from 70 km/h:  load                shifts stall lowest rpm  gears\n");
            struct St { const char *name; float mass, brake; } st[2] = { { "17 t brake 0.2", 17000, 0.2f }, { "33 t brake 1.0", 33000, 1.0f } };
            for (auto &sx : st) {
                g_st_brake = sx.brake; g_st_coast = 0; g_st_unseen = 0;
                g_rec_on = 1;
                sc_do(sx.name, {sx.mass, 70, sc_gear_for(70, V.plateau + 250), 40, 3, 1.0f}, c_stop, 1);
                g_rec_on = 0;
                int stall = 0;
                for (int i = 0; i < g_rec_n; i++) if (!strcmp(g_rec[i].why, "stall")) stall++;
                char seq[160] = ""; int q = 0;
                for (int i = 0; i < g_rec_n && q < (int)sizeof seq - 10; i++) q += snprintf(seq + q, sizeof seq - q, i ? ">%d" : "%d>%d", i ? g_rec[i].b : g_rec[i].a, g_rec[i].b);
                P7("                                  %-26s %6d %5d %9.0f  %s\n", sx.name, g_rec_n, stall, g_st_rpm_min, seq);
                k += snprintf(sg + k, 1200 - k, "s%.0f %d %d %.0f %s|", sx.mass, g_rec_n, stall, g_st_rpm_min, seq);
            }
        }
    }
    set_param("rpm_by_engine", 0.0f); use_ato(); g_idle_sim = 600; set_param("idle_rpm", 600);
    sc_check(!strcmp(sig[0][0], sig[0][1]), "rpm_by_engine 1 on today's curve (idle 600, plateau 900): the margins, every shift and every rpm of the pull-aways and stops are the same as with 0");
    sc_check(strcmp(sig[1][0], sig[1][1]) != 0, "rpm_by_engine 1 on the plateau-1300 engine: the margins follow its span");
    return g_sc_failed;
}

// ---- part 8: [trueshift] power_shift_crossover. At a firm pedal in Power (0.8 up) and a full pedal in Normal (0.95 up) the window top of each gear is
// the power crossover to the next one where that lies above the top of the power range (core/gearscore.h gs_cross_rpm). Per gear pair the crossover,
// then a full pedal from 30 to 80 km/h at 17 and 33 t with the key 0 and 1: the time, the shifts and the rpm each upshift happened at
static float g_x_ped = 1; static double g_x_t80, g_x_t0;
static void c_x80(double t)
{
    if (t <= 0) { g_x_t80 = -1; g_x_t0 = t_now; g_rec_n = 0; }
    M.pedal = g_x_ped;
    if (g_x_t80 < 0 && M.v * 3.6f >= 80) g_x_t80 = t;
}
static int run_scores_cross()
{
    printf("\n=== SCORES part 8: power_shift_crossover, full pedal 30 -> 80 km/h (choice 1, 0.5 s box) ===\n");
    float sf_def = 0.3f;
    for (int j = 0; j < m_trueshift.n_params; j++) if (!strcmp(m_trueshift.params[j].key, "skip_land_frac")) sf_def = m_trueshift.params[j].def;
    set_param("skip_land_frac", 0.0f);   // (the crossover test predates skip_land_frac: it isolates the crossover key from the skip landing rule)
    struct Eg { const char *name; void (*use)(); } egs[2] = { { "Renault DE13 (ATO 12 speeds)", use_ato }, { "Volvo (2400 Nm)", use_volvo } };
    struct Va { const char *name; float cvar, ped; } vas[4] = { { "Power, pedal 1.0", 1.6667f, 1.0f }, { "Normal, pedal 1.0", 3.0f, 1.0f }, { "Normal, pedal 0.6", 3.0f, 0.6f }, { "Eco, pedal 1.0", 10.0f, 1.0f } };
    for (auto &eg : egs) {
        eg.use(); set_param("choice", 1.0f); sim_reset(RESET_HARD, "sim: next engine"); sc_prime(33000, 0.5f, 30, 8);
        float lim = V.eng.rpm_limit, base = fminf(V.eng.range[R_POWER][1], 0.93f * lim);
        printf("\n  %s: top of the power range %.0f rpm (limit %.0f), power crossover per gear pair (rpm in the lower gear; used = the window top, 95 %% of the limit = %.0f):\n   ", eg.name, V.eng.range[R_POWER][1], lim, 0.95f * lim);
        for (int g = 1; g < T.fwd_gears; g++) {
            float cr = gs_cross_rpm(g, 15, base, lim);
            printf(" %d>%d %.0f%s", g, g + 1, cr, g % 6 == 0 && g + 1 < T.fwd_gears ? "\n   " : ";");
        }
        printf("\n  %-18s %-6s %-3s %8s %7s  %s\n", "full pedal 30 -> 80", "load", "key", "to 80 km/h", "shifts", "upshifts: rpm before the shift");
        // a warm-up run: the module keeps some state across a reset, so the first run of a process differs from a second one
        g_quiet = 1; g_x_ped = 1; set_param("power_shift_crossover", 1.0f); t_now = 5000;
        sc_do("warm", {33000, 30, sc_gear_for(30, 1100), 10, 3.0f, 0.5f}, c_x80, 1);
        for (float mass : { 17000.0f, 33000.0f }) {
            for (auto &va : vas) {
                double t80[2]; char sig[2][200] = {};
                for (int key = 0; key < 2; key++) {
                    set_param("power_shift_crossover", (float)key); g_x_ped = va.ped; t_now = 5000;
                    sim_reset(RESET_HARD, "sim: next variant");
                    sc_do(va.name, {mass, 30, sc_gear_for(30, 1100), 70, va.cvar, 0.5f}, c_x80, 1);   // (the same run twice: the second starts from what the first of its own kind left)
                    t_now = 5000; sim_reset(RESET_HARD, "sim: again");
                    g_rec_on = 1;
                    sc_do(va.name, {mass, 30, sc_gear_for(30, 1100), 70, va.cvar, 0.5f}, c_x80, 1);
                    g_rec_on = 0;
                    char up[160] = ""; int q = 0, n = 0;
                    for (int i = 0; i < g_rec_n && q < (int)sizeof up - 12; i++) if (g_rec[i].b > g_rec[i].a && (g_x_t80 < 0 || g_rec[i].t - g_x_t0 <= g_x_t80 + 0.5)) {
                        q += snprintf(up + q, sizeof up - q, "%s%d>%d %.0f", n ? ", " : "", g_rec[i].a, g_rec[i].b, g_rec[i].rpm); n++;
                    }
                    t80[key] = g_x_t80;
                    snprintf(sig[key], sizeof sig[key], "%s", up);
                    char ts[12]; if (g_x_t80 >= 0) snprintf(ts, sizeof ts, "%.1f s", g_x_t80); else snprintf(ts, sizeof ts, "never");
                    printf("  %-18s %4.0f t %3d %10s %7d  %s\n", va.name, mass / 1000, key, ts, n, up);
                }
                bool fast = va.cvar < 4 && va.ped >= 0.95f || (va.cvar < 2 && va.ped >= 0.8f);
                char what[200];
                if (fast) { snprintf(what, sizeof what, "%s, %s, %.0f t: the crossover is not slower to 80 km/h (%.2f s against %.2f s)", eg.name, va.name, mass / 1000, t80[1], t80[0]); sc_check(t80[1] >= 0 && t80[0] >= 0 && t80[1] <= t80[0] + 0.05, what); }
                else { snprintf(what, sizeof what, "%s, %s, %.0f t: the key changes no shift (Eco, and Normal under a full pedal) and the time by under 0.2 s (the sim keeps a little state from run to run)", eg.name, va.name, mass / 1000); sc_check(!strcmp(sig[0], sig[1]) && fabs(t80[1] - t80[0]) < 0.2, what); }
            }
        }
        g_quiet = 0;
    }
    set_param("power_shift_crossover", 1.0f); use_ato();
    set_param("skip_land_frac", sf_def);
    return g_sc_failed;
}


// ---- part 9: kickdown_by_wish ([trueshift], choice 1 with the wish pedal): the kickdown as the driver's unmet wish, against the flat-pedal rule
// The wish pedal is on as in part 2's wish rows (ref_now: the best gear's full force at this speed is a full wish), the driver's pedal is T.in_thr.
// c_kw: the pedal holds g_kw_p0 for g_kw_hold s, then rises to g_kw_p1 in g_kw_ramp s. g_kw_r: the ratio of the wished force to this gear's force at
// full throttle at the last tick before a decision; at every kickdown it is checked (never one with the ratio under 1).
static float g_kw_p0 = 0.3f, g_kw_p1 = 1.0f, g_kw_hold = 6, g_kw_ramp = 24; static double g_kw_t0, g_kw_r;
static float g_kw_rmin = 9; static int g_kw_seen, g_kw_kd; static char g_kw_buf[2][16];
static void c_kw(double t)
{
    if (t <= 0) { g_kw_t0 = t_now; g_rec_n = 0; g_kw_seen = 0; g_kw_r = 0; g_pl_full = 1; }
    for (; g_kw_seen < g_rec_n; g_kw_seen++)
        if (!strncmp(g_rec[g_kw_seen].why, "kickdown", 8)) { g_kw_kd++; if (g_kw_r > 0) g_kw_rmin = fminf(g_kw_rmin, (float)g_kw_r); }
    float p = t < g_kw_hold ? g_kw_p0 : fminf(g_kw_p1, g_kw_p0 + (g_kw_p1 - g_kw_p0) * (float)((t - g_kw_hold) / fmax(g_kw_ramp, 1e-3)));
    M.pedal = p; g_pl_ped = p; M.wish_on = 0;
    if (M.gear > 0 && !M.state && M.v * 3.6f > 12) {
        float fg = vehicle_force_at(M.gear, M.v, 1.0f);
        if (fg > 0) { M.wish_on = 1; M.wish_thr = fminf(1, sim_wish_force(M.v) / fg); g_kw_r = sim_wish_force(M.v) / fg; }
    }
}
static float r_flat0(double) { return 0; }
static float r_two(double) { return 2.0f; }
struct kw_res { int shifts, downs, ups, pairs, kd; double first_dn; char why[24]; };
static kw_res kw_stats(double from)   // from the recorder; first_dn = seconds from `from` to the first downshift (-1 = none)
{
    kw_res r = {}; r.first_dn = -1;
    for (int i = 0; i < g_rec_n; i++) {
        r.shifts++;
        if (g_rec[i].b > g_rec[i].a) r.ups++; else { r.downs++; if (r.first_dn < 0 && g_rec[i].t >= from) { r.first_dn = g_rec[i].t - from; snprintf(r.why, sizeof r.why, "%s", g_rec[i].why); } }
        if (!strncmp(g_rec[i].why, "kickdown", 8)) r.kd++;
        for (int j = i + 1; j < g_rec_n && g_rec[j].t - g_rec[i].t < 4.0; j++) if (g_rec[i].b > g_rec[i].a && g_rec[j].b < g_rec[j].a) { r.pairs++; break; }
    }
    return r;
}
static int run_scores_kickwish()
{
    printf("\n=== SCORES part 9: kickdown_by_wish 0 (the flat-pedal rule) against 1 (the unmet wish), wish pedal, choice 1, ATO ===\n");
    use_ato(); set_param("choice", 1.0f); sim_reset(RESET_HARD, "sim: part 9");
    struct Md { const char *name; float cvar; } mds[3] = { { "Eco", 10.0f }, { "Normal", 3.0f }, { "Power", 1.6667f } };
    float lvl[3] = { 0, 0, 0 }, rat[3] = { 0, 0, 0 };
    {
        const char *kl[3] = { "kick_level_eco", "kick_level", "kick_level_power" }, *kr[3] = { "kick_ratio_eco", "kick_ratio", "kick_ratio_power" };
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < m_trueshift.n_params; j++) {
                if (!strcmp(m_trueshift.params[j].key, kl[i])) lvl[i] = *m_trueshift.params[j].val;
                if (!strcmp(m_trueshift.params[j].key, kr[i])) rat[i] = *m_trueshift.params[j].val;
            }
        printf("  levels (defaults): Eco %.3f, Normal %.3f, Power %.3f; ratio %.2f / %.2f / %.2f. Time to the kickdown at a +10 %% shortage: %.1f / %.1f / %.2f s; at +40 %%: %.2f / %.2f / %.2f s\n",
               lvl[0], lvl[1], lvl[2], rat[0], rat[1], rat[2], lvl[0] / (1.10f - rat[0]), lvl[1] / (1.10f - rat[1]), lvl[2] / (1.10f - rat[2]),
               lvl[0] / (1.40f - rat[0]), lvl[1] / (1.40f - rat[1]), lvl[2] / (1.40f - rat[2]));
    }
    // 1. the pedal sweep 0.3 -> 1.0 (6 s at 0.3, 24 s rise, 15 s at 1.0) from 50 km/h, 45 s. (a) in the highest gear that gives 1100 rpm, the gear the
    // truck would be in (9 at 1252 rpm: the wish is within 4 % of what it gives); (b) in the highest gear that gives 900 rpm (10 at 980 rpm: inside the
    // window but 25 % weaker than 9; the wish does not stay unmet)
    g_kw_kd = 0; g_kw_rmin = 9;
    int sw_pairs = 0, sw_tot[2][2] = { { 0, 0 }, { 0, 0 } }, sw_worst = 0;
    for (int part = 0; part < 2; part++) {
        float rpm_min = part ? 900.0f : 1100.0f;
        printf("\n  pedal sweep 0.3 -> 1.0 from 50 km/h, 45 s, start %s: shifts / downshifts / kickdowns / up-down pairs within 4 s     kickdown_by_wish 0   |  1\n",
               part ? "in gear 10 (980 rpm, a step too tall)" : "in gear 9 (1252 rpm, the gear it would be in)");
        for (float mass : { 17000.0f, 33000.0f })
            for (int grade = 0; grade < 2; grade++)
                for (int m = 0; m < 3; m++) {
                    kw_res r[2];
                    for (int kw = 0; kw < 2; kw++) {
                        set_param("kickdown_by_wish", (float)kw);
                        g_kw_p0 = 0.3f; g_kw_p1 = 1.0f; g_kw_hold = 6; g_kw_ramp = 24; g_rec_on = 1;
                        sc_do("kw sweep", {mass, 50, sc_gear_for(50, rpm_min), 45, mds[m].cvar, 0.5f, 0, -1, -1, grade ? r_two : r_flat0, 0}, c_kw, 1);
                        g_rec_on = 0;
                        r[kw] = kw_stats(0);
                        sw_tot[part][kw] += r[kw].shifts;
                        if (getenv("KW_LIST") && kw == 1) for (int i = 0; i < g_rec_n; i++) printf("        %5.1f s  %d->%d (%s) %.0f rpm\n", g_rec[i].t - g_kw_t0, g_rec[i].a, g_rec[i].b, g_rec[i].why, g_rec[i].rpm);
                    }
                    sw_pairs += r[1].pairs;
                    if (part) sw_worst = r[1].shifts - r[0].shifts > sw_worst ? r[1].shifts - r[0].shifts : sw_worst;
                    printf("    %2.0f t %-4s %-7s  %2d / %d / %d / %d            |  %2d / %d / %d / %d\n", mass / 1000, grade ? "2 %" : "flat", mds[m].name,
                           r[0].shifts, r[0].downs, r[0].kd, r[0].pairs, r[1].shifts, r[1].downs, r[1].kd, r[1].pairs);
                }
        printf("  shifts over the 12 runs: %d (0) against %d (1)\n", sw_tot[part][0], sw_tot[part][1]);
    }
    set_param("kickdown_by_wish", 1.0f);
    sc_check(sw_pairs == 0, "kickdown_by_wish 1: no upshift followed by a downshift within 4 s in the sweeps");
    sc_check(sw_tot[0][1] * 10 <= sw_tot[0][0] * 11, "kickdown_by_wish 1: start in the gear it would be in: no more than 10 % more shifts than 0 over the sweeps");
    sc_check(sw_worst <= 3, "kickdown_by_wish 1: start a step too tall: at most 3 shifts more than 0 in any sweep (the kickdown and the upshift back)");
    sc_check(g_kw_rmin >= 1.0f, "a kickdown by the wish never comes while this gear can give the wish (ratio under 1)");
    // 2. gear 10, 33 t flat: the pedal steps from 0.3 to 0.6 / 0.8 / 1.0 at 2 s: seconds from the step to the first downshift against the time the level
    // gives. At 41 km/h gear 10 turns 803 rpm (under the window: the lugging rule shifts first, with kickdown_by_wish 0 as well); at 50 km/h 980 rpm
    // (inside the window, weaker than 9: only the wish decides)
    int ok = 0, n = 0;
    for (float kmh : { 41.0f, 50.0f }) {
        sc_prime(33000, 0.5f, kmh, 10);
        printf("\n  gear 10 at %.0f km/h = %.0f rpm, 33 t flat, the pedal steps from 0.3 at 2 s: ratio of the wish to gear 10, seconds to the first downshift with kickdown_by_wish 1 | 0\n", kmh, rpm_now());
        for (int m = 0; m < 3; m++)
            for (float pe : { 0.6f, 0.85f, 1.0f }) {
                double dn[2] = { -2, -2 };
                char wy[24] = "";
                float r0 = 0, tb = 0;
                for (int kw = 1; kw >= 0; kw--) {
                    set_param("kickdown_by_wish", (float)kw);
                    g_kw_p0 = 0.3f; g_kw_p1 = pe; g_kw_hold = 2; g_kw_ramp = 0.1f; g_rec_on = 1;
                    sc_do("kw step", {33000, kmh, 10, 10, mds[m].cvar, 0.5f, 0, -1, -1, r_flat0, 0}, c_kw, 1);
                    g_rec_on = 0;
                    kw_res kr = kw_stats(g_kw_t0 + 2); dn[kw] = kr.first_dn; if (kw == 1) snprintf(wy, sizeof wy, "%s", kr.why);
                    g_pl_ped = pe; g_pl_full = 1;
                    r0 = sim_wish_force(kmh / 3.6f) / vehicle_force_at(10, kmh / 3.6f, 1.0f);
                }
                // the level over the excess, plus the shift time, 0.3 s of settling and a second for the pedal's own rise (a flat pedal: at once)
                tb = r0 > rat[m] ? (pe >= 1.0f ? 0.0f : lvl[m] / (r0 - rat[m])) + 0.5f + 0.3f + 1.0f : 0;
                snprintf(g_kw_buf[0], 16, dn[1] < 0 ? "none" : "%.1f s", dn[1]);
                snprintf(g_kw_buf[1], 16, dn[0] < 0 ? "none" : "%.1f s", dn[0]);
                printf("    %-6s pedal %.2f  ratio %.2f   %-6s | %-6s   (bound %.1f s)  %s\n", mds[m].name, pe, r0, g_kw_buf[0], g_kw_buf[1], tb, dn[1] >= 0 ? wy : "");
                if (r0 > rat[m] + 0.2f) { n++; if (dn[1] >= 0 && dn[1] <= tb) ok++; }
            }
    }
    sc_check(n > 0 && ok == n, "gear 10 at 41 and 50 km/h, 33 t, pedal 0.6-1.0: the downshift comes within the per-mode time of the level (each case with the ratio 0.2 over kick_ratio)");
    set_param("kickdown_by_wish", 1.0f);
    g_pl_full = 0; g_pl_ped = 0.7f; M.wish_on = 0;
    return g_sc_failed;
}

// ---- part 10: [trueshift] skip_land_frac. A skip upshift must land a share of the way up from the choice floor to the
// upshift point, so a moderate pedal ramps through fewer skips and with a smaller rpm drop per upshift; a full-pedal launch
// still skips as before.
static float g_sf_ped; static int g_sf_full;
static void c_skipfrac(double t)
{
    if (t <= 0) { g_rec_n = 0; g_pl_t0 = t_now; }
    M.pedal = g_sf_full ? 1.0f : fminf(g_sf_ped, 0.3f + 0.02f * (float)t);
}
static void sf_stats(int *skips, double *drop)   // skip upshifts, and the median rpm drop over every upshift (g_rec.rpm is the rpm before the shift)
{
    *skips = 0; double d[REC_MAX]; int n = 0;
    for (int i = 0; i < g_rec_n; i++) {
        if (g_rec[i].b <= g_rec[i].a) continue;
        if (abs(g_rec[i].b - g_rec[i].a) > 1) (*skips)++;
        d[n++] = g_rec[i].rpm * (1.0 - (double)RATIO[g_rec[i].b - 1] / RATIO[g_rec[i].a - 1]);
    }
    for (int i = 1; i < n; i++) for (int j = i; j > 0 && d[j] < d[j - 1]; j--) { double t = d[j]; d[j] = d[j - 1]; d[j - 1] = t; }
    *drop = n ? (n & 1 ? d[n / 2] : 0.5 * (d[n / 2 - 1] + d[n / 2])) : 0;
}
static int run_scores_skipfrac()
{
    printf("\n=== SCORES part 10: skip_land_frac, a moderate ramp (fewer, gentler skips) and a full launch (still skips), ATO 33 t ===\n");
    use_ato();
    set_param("choice", 1.0f); set_param("choice_objective", 1.0f);
    float sf_def = 0.3f;
    for (int j = 0; j < m_trueshift.n_params; j++) if (!strcmp(m_trueshift.params[j].key, "skip_land_frac")) sf_def = m_trueshift.params[j].def;
    int sk[2]; double dr[2];
    float fv[2] = { 0.0f, sf_def };
    g_sf_full = 0; g_sf_ped = 0.5f;
    for (int f = 0; f < 2; f++) {
        set_param("skip_land_frac", fv[f]);
        g_rec_on = 1;
        sc_do("moderate ramp", {33000, 1, 1, 120, 3, 0.5f}, c_skipfrac, 1);
        g_rec_on = 0;
        sf_stats(&sk[f], &dr[f]);
        printf("  skip_land_frac %.1f: %d skip upshifts, median drop %.0f rpm\n", fv[f], sk[f], dr[f]);
        if (getenv("SC_SHIFTS")) for (int i = 0; i < g_rec_n; i++) printf("      %6.1f  %d->%d (%s) %.0f rpm\n", g_rec[i].t - g_pl_t0, g_rec[i].a, g_rec[i].b, g_rec[i].why, g_rec[i].rpm);
    }
    sc_check(sk[1] < sk[0] && dr[1] < dr[0] - 1.0, "a moderate-pedal ramp on 33 t skips less and drops less rpm per upshift with the default than with 0");
    int fsk[2];
    g_sf_full = 1;
    for (int f = 0; f < 2; f++) {
        set_param("skip_land_frac", fv[f]);
        g_rec_on = 1;
        sc_do("full launch", {33000, 1, 1, 60, 3, 0.5f}, c_skipfrac, 1);
        g_rec_on = 0;
        double d; sf_stats(&fsk[f], &d);
        printf("  full pedal, skip_land_frac %.1f: %d skip upshifts\n", fv[f], fsk[f]);
    }
    sc_check(fsk[1] >= fsk[0] && fsk[1] > 0, "a full-pedal launch still skips as before (the default never skips less)");
    set_param("skip_land_frac", sf_def);
    return g_sc_failed;
}

// ---- part 11: the own speed limiter. At a 95 km/h cap with the pedal at 1.0 the throttle is capped below the pedal, so the
// demand is what the capped throttle gives (holding the speed): the truck ends in the top gear, not stuck one below.
static void c_limiter(double t)
{
    M.pedal = 1.0f;
    float kmh = M.v * 3.6f, lim = 95.5f;   // speed_limiter_kmh 95 + the bias
    g_act[ACT_CRUISE].scale = kmh >= lim ? 0.0f : fmaxf(0.0f, fminf(1.0f, (lim - (kmh + T.accel * 3.6f)) / 1.5f));
}
static int run_scores_limiter()
{
    printf("\n=== SCORES part 11: the own speed limiter at 95 km/h, pedal 1.0, ATO 33 t flat: the top gear ===\n");
    use_ato();
    set_param("choice", 1.0f); set_param("choice_objective", 1.0f);
    g_rec_on = 1;
    sc_do("limiter 95", {33000, 88, 11, 45, 3, 0.5f, 0, -1, -1, r_flat0, 0}, c_limiter, 1);
    g_rec_on = 0;
    int top = 0;
    for (int i = 0; i < g_rec_n; i++) if (g_rec[i].b == 12 && g_rec[i].b > g_rec[i].a) top = 1;
    printf("  end in gear %d at %.0f km/h, reached 12: %s\n", M.gear, M.v * 3.6f, top ? "yes" : "no");
    if (getenv("SC_SHIFTS")) for (int i = 0; i < g_rec_n; i++) printf("      %6.1f  %d->%d (%s) %.0f rpm\n", g_rec[i].t - g_pl_t0, g_rec[i].a, g_rec[i].b, g_rec[i].why, g_rec[i].rpm);
    sc_check(top, "at a 95 km/h cap with the pedal at 1.0 the truck ends in the top gear (12)");
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
