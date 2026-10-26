// Offline test of the low-speed owner in shadow (modules/lowspeed.cpp is included, so its statics are reachable; TrueShift and the
// vehicle model are linked for real: the start gear is TrueShift's physics pick). A truck plant stands in for the game's
// automatic, TrueShift's stop and Creep's crawl: they drive, the owner only watches and decides, and the scenarios check what it
// would have commanded. Creep's and Cruise's states (crawl, smooth start, Auto Hold, Stop&Go) are inputs the script sets.
// Plant: speed from pedals, brake, a little resistance and the grade; the engine at idle with an open clutch, on the driveline with
// a closed one; a slip phase at a launch; the box in neutral when the engine would fall under idle + 150 rpm (+ the brake term),
// one gear down when braking pulls the engine near idle, the start gear on a pedal at rest.
// Build+run: tools/sim/run_lowspeed.sh [-v]   (-v prints the log lines and the state changes)
#include "core/suite.h"
#include "core/game.h"
#include "core/route.h"
#include "core/vehicle.h"
#include <initializer_list>
#include <math.h>
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
volatile float g_clutch_ours = __builtin_nanf("");
volatile char g_dash_flash, g_dash_status;
double g_dash_flash_until;
int g_log_level = 2;
incident_ctx g_inc = { 0, 0, 0, NAN, NAN };
double g_incident_mb = 0;
const char *g_truck_unsupported;
char g_dir[MAX_PATH];
const layout *L;
game_addrs G;
static double t_now;
double wall_s() { return t_now; }
double sim_s() { return t_now; }
void csv_cap(FILE **, const char *) {}
FILE *open_out(const char *, const char *) { return NULL; }
void logring(const char *, ...) {}
void incident(const char *, const char *, ...) {}
void play_cue(int) {}
int input_held(int) { return 0; }
int input_pressed(int, int) { return 0; }
static int verbose, n_failed, n_checks;
static int n_engage_lines;   // log lines of the form "engage ...": the start gear the owner must never produce
void logline(const char *fmt, ...)
{
    char b[300]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    if (strstr(b, "engage")) n_engage_lines++;
    if (verbose) printf("%7.2f  %s\n", t_now, b);
}

// ---- the truck: a Renault ATO (12 speeds), the engine of the TrueShift sim
static const float IDLE = 650, TORQUE = 2700, LIMIT = 1900, DIFF = 3.08f, RADIUS = 0.538f;
static float RATIO[16];
static const float CURVE[8][2] = { {0,0},{700,0.60f},{900,0.97f},{1250,1},{1500,0.9f},{1700,0.72f},{1900,0.45f},{2100,0} };
static const float RANGES[4][2] = { {1500,1900},{750,1450},{900,1250},{1200,1600} };
static float g_mass = 36000, g_grade_pct;
static struct { double v; int gear; float clutch; float rpm; int launching, crawl; double down_at; } M;   // v m/s, clutch 0 closed .. 1 open
static float g_thr, g_brake;
static creep_info CI;       // what Creep would report (the script sets it)
static cruise_info CU;      // what Cruise would report
static int g_crawl_gear = 2;

float wheel_radius() { return RADIUS; }
int gb_engine(void *, engine_info *e)
{
    e->torque = TORQUE; e->rpm_limit = LIMIT; e->n_curve = 8; e->shift_time = 0.5f;
    memcpy(e->curve, CURVE, sizeof CURVE);
    memcpy(e->range, RANGES, sizeof RANGES);
    return 1;
}
float curve_factor(const engine_info *e, float rpm)
{
    if (rpm <= e->curve[0][0]) return e->curve[0][1];
    for (int i = 1; i < e->n_curve; i++)
        if (rpm <= e->curve[i][0])
            return e->curve[i-1][1] + (rpm - e->curve[i-1][0]) / (e->curve[i][0] - e->curve[i-1][0]) * (e->curve[i][1] - e->curve[i-1][1]);
    return e->curve[e->n_curve-1][1];
}
float game_mass() { return g_mass; }
float game_grade_deg(void *) { return atanf(g_grade_pct / 100) * 57.2958f; }
float game_adaptive() { return 3; }                 // Normal
void *game_gearbox() { return (void *)&M; }
int gb_gear(void *) { return M.gear; }
int gb_target(void *) { return M.gear; }
int gb_state(void *) { return 0; }
int gb_engage(void *, int) { return 0; }
int game_engage_init() { return 1; }
void gb_set_target(void *, int) {}
void gb_force(void *) {}
char *gb_inner(void *) { return NULL; }
float gb_thr_eff(void *) { return g_thr; }
int game_letter(void *, char) { return 1; }
int game_ebrake_downshift(void *) { return 0; }
float game_ebrake_level(void *) { return -1; }
int game_patch_call(uintptr_t, void *) { return 0; }
int game_patch_tail(uintptr_t, void *) { return 0; }
int game_patch_stop(uintptr_t, float (*)(float, float, float)) { return 0; }
int mem_readable(const void *, size_t) { return 0; }
char *game_actor_safe() { return NULL; }
int route_ok() { return 0; }
float route_grade(float, float) { return NAN; }
float route_max_grade(float, float) { return NAN; }
float route_min_grade(float, float) { return NAN; }
float route_crest_m(float) { return -1; }
float route_junction_m() { return -1; }
float route_curve_speed(float, float, float) { return NAN; }
int act_hooked() { return 1; }
int act_clutch_ok() { return 1; }
float act_request() { return 0; }
void creep_info_get(creep_info *o) { *o = CI; }
void cruise_info_get(cruise_info *o) { *o = CU; }

#include "modules/lowspeed.cpp"
extern module m_trueshift;

static float rpm_of(int g, double v) { return g > 0 ? (float)(v / RADIUS * RATIO[g - 1] * DIFF * 9.5493) : 0; }

// ---- the plant: one step of dt seconds. What the truck does by itself (game + TrueShift + Creep), set into the telemetry.
static void plant(double dt)
{
    float idle = IDLE, thr = g_thr, brk = g_brake;
    double kmh = fabs(M.v) * 3.6;
    // the stop: the box goes to neutral when the engine would fall under idle + 150 rpm (+ the brake term); never while crawling
    if (M.gear > 0 && thr < 0.01f && !CI.crawl && rpm_of(M.gear, M.v) < idle + 150 + brk * brk * idle) { M.gear = 0; M.launching = 0; }
    // braking pulls the engine near idle: one gear down (TrueShift's braking downshift), at most every 0.7 s
    if (M.gear > 1 && thr < 0.01f && brk > 0.05f && rpm_of(M.gear, M.v) < idle + 250 + brk * brk * idle && t_now - M.down_at > 0.7) { M.gear--; M.down_at = t_now; }
    // a pedal at a standstill or in neutral: the start gear by physics
    if (M.gear == 0 && thr > 0.01f && !CI.crawl) { M.gear = ts_start_gear(fmaxf(thr, 0.3f)); M.launching = 1; }
    if (M.gear > 0 && M.launching && thr < 0.01f && M.v < 0.05) M.launching = 0;
    // crawl: the gear and the slipping clutch of Creep; where it ends with the pedal down the smooth start carries the slipping clutch on
    if (M.crawl && !CI.crawl && thr > 0.06f && M.gear > 0) M.launching = 1;
    M.crawl = CI.crawl;
    double target = -1;
    if (CI.crawl) {
        if (M.gear <= 0) M.gear = g_crawl_gear;
        target = (5 + fmaxf(0, (thr - 0.02f) / 0.28f) * 5) / 3.6 * (1 - fminf(1, brk / 0.3f));
    }
    // acceleration
    float rlock = rpm_of(M.gear, M.v), aim = idle + 300 + 600 * thr;
    float a = 0;
    if (M.gear > 0 && !CI.crawl) a = thr * 3.0f;
    if (target >= 0) a = (float)fmax(-1.0, fmin(0.8, (target - M.v) * 2));
    a -= 0.12f + 9.81f * g_grade_pct / 100;   // rolling resistance and the grade
    a -= brk * 6;
    float v1 = (float)(M.v + a * dt);
    if (v1 < 0 && M.v >= 0 && (brk > 0 || a < 0)) v1 = 0;   // the brake and the resistance stop the truck, they do not reverse it
    M.v = v1;
    if (M.v < 0) M.v = 0;
    // clutch and engine
    if (M.gear == 0) { M.clutch = 1; M.rpm = idle; M.launching = 0; }
    else if (CI.crawl) { M.clutch = 0.4f; M.rpm = idle; }
    else if (M.launching && rlock < aim) { M.clutch = 0.35f; M.rpm = aim; }
    else { M.launching = 0; M.clutch = 0; M.rpm = fmaxf(idle, rlock); }
    // up when the engine is high and the pedal asks for it
    if (M.gear > 0 && M.gear < 12 && !M.launching && thr > 0.3f && M.rpm > 1500) M.gear++;
    // telemetry
    T.speed = (float)M.v; T.rpm = M.rpm; T.in_thr = thr; T.eff_thr = thr; T.in_brake = brk; T.eff_brake = brk;
    T.eff_clutch = M.clutch; T.gear = M.gear; T.dgear = M.gear; T.engine_on = 1; T.cruise = 0;
    static double last_t; static float last_v;
    double d = t_now - last_t;
    if (d > 0.0 && d < 0.5) { float acc = (float)((fabsf(T.speed) - last_v) / d); T.accel += (float)(d / (0.5 + d)) * (acc - T.accel); }
    last_t = t_now; last_v = fabsf(T.speed);
    (void)kmh;
}

// ---- the harness
static void set_param(module &m, const char *n, float v)
{
    for (int i = 0; i < m.n_params; i++) if (!strcmp(m.params[i].key, n)) *m.params[i].val = v;
}
static void sim_reset(int level, const char *why)   // as core/telemetry.cpp session_reset does it
{
    vehicle_reset(level);
    m_trueshift.reset(level, why);
    m_lowspeed.reset(level, why);
}

struct tick_rec { int state, would_gear; float would_clutch, would_brake, kmh; };
static int trans[64], n_trans;       // the state changes of a run, in order
static int last_state;

static void tick(double dt = 1.0 / 60)
{
    t_now += dt;
    T.sim_time = (scs_timestamp_t)(t_now * 1e6);
    plant(dt);
    vehicle_frame();
    m_lowspeed.frame();
    if (g_low.state != last_state) {
        if (n_trans < 64) trans[n_trans++] = g_low.state;
        if (verbose) printf("   [%7.2f] %s  %.2f km/h gear %d: %s\n", t_now, low_state_name(g_low.state), fabs(M.v) * 3.6, M.gear, g_low.reason);
        last_state = g_low.state;
    }
}

static void world(int gear, double kmh, float grade, float mass = 36000)
{
    memset(&T, 0, sizeof T);
    T.fwd_gears = 12; T.diff = DIFF; T.wheels = 4; T.engine_on = 1;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
    memset(&M, 0, sizeof M);
    M.gear = gear; M.v = kmh / 3.6; M.down_at = -10;
    g_thr = g_brake = 0; g_grade_pct = grade; g_mass = mass;
    memset(&CI, 0, sizeof CI); memset(&CU, 0, sizeof CU);
    CI.brake_off = 0.30f;
    g_crawl_on = 0; g_lcx = 0; g_rolling = 0;
    n_trans = 0; n_engage_lines = 0;
    sim_reset(RESET_HARD, "sim: new scenario");
    vehicle_mass_stale();
    g_low.state = LOW_GAME; last_state = LOW_GAME; g_fresh = 1;
    T.accel = 0;
    // settle the model (mass, engine, grade) and the first frame, no motion
    for (int i = 0; i < 3; i++) { t_now += 1.0 / 60; vehicle_frame(); }
    // locked at that speed in that gear
    if (gear > 0) M.rpm = fmaxf(IDLE, rpm_of(gear, M.v));
}

static int gear_for(double kmh, float rpm)   // the gear whose rpm at this speed is nearest rpm
{
    int best = 1; float be = 1e9f;
    for (int g = 1; g <= 12; g++) { float e = fabsf(rpm_of(g, kmh / 3.6) - rpm); if (e < be) { be = e; best = g; } }
    return best;
}

static void check(bool ok, const char *fmt, ...)
{
    n_checks++;
    if (!ok) {
        n_failed++;
        printf("   FAIL: ");
        va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
        printf("\n");
    }
}

static const char *seq_text(const int *s, int n, char *out, size_t cap)
{
    out[0] = 0;
    for (int i = 0; i < n; i++) { strncat(out, low_state_name(s[i]), cap - strlen(out) - 1); if (i + 1 < n) strncat(out, " ", cap - strlen(out) - 1); }
    return out;
}

// ---------------------------------------------------------------- S0: a stop, the hold, a launch, a stop, the crawl, a launch out of it
static void s0()
{
    printf("== S0 shadow state sequence: stop, hold, launch, stop, crawl, maneuver, launch out of the crawl\n");
    world(8, 30, 0);
    M.gear = gear_for(30, 1300);
    M.rpm = rpm_of(M.gear, M.v);
    double t0 = t_now;
    int phase = 0;
    double t_ph = 0;
    for (int i = 0; i < 60 * 80; i++) {
        double t = t_now - t0, kmh = fabs(M.v) * 3.6;
        // the driver's script, one phase after the other
        switch (phase) {
        case 0: if (t > 1) { g_brake = 0.3f; phase = 1; } break;                                // lift and brake from 30 km/h
        case 1: CU.hold = kmh < 0.5 && g_brake > 0.1f ? 1 : 0; CU.hold_req = CU.hold ? 0.6f : 0;
                if (kmh < 0.05) { phase = 2; t_ph = t; } break;                                 // stopped, Auto Hold holds
        case 2: CU.hold = 1; if (t - t_ph > 2) { g_brake = 0; g_thr = 0.4f; CU.hold = 0; CU.hold_req = 0; phase = 3; t_ph = t; } break;   // pedal: launch
        case 3: if (kmh > 22) { g_thr = 0; g_brake = 0.3f; phase = 4; } break;                  // at 22 km/h lift and brake to a stop
        case 4: CU.hold = kmh < 0.5 && g_brake > 0.1f ? 1 : 0; if (kmh < 0.05) { phase = 5; t_ph = t; } break;
        case 5: if (t - t_ph > 2) { g_brake = 0; CU.hold = 0; phase = 6; t_ph = t; } break;    // brake off: nothing holds
        case 6: if (t - t_ph > 1) { CI.crawl = 1; phase = 7; t_ph = t; } break;                // crawl comes on
        case 7: if (t - t_ph > 4) { g_thr = 0.15f; phase = 8; t_ph = t; } break;              // maneuvering
        case 8: if (t - t_ph > 2) { g_thr = 0; phase = 9; t_ph = t; } break;                   // pedal off: crawl again
        case 9: if (t - t_ph > 2) { g_thr = 0.6f; CI.crawl = 0; phase = 10; t_ph = t; } break; // pedal flat: the crawl ends, pull away
        case 10: if (t - t_ph > 12) phase = 11; break;
        }
        tick();
        if (phase == 11) break;
    }
    static const int expect[] = { LOW_HANDED, LOW_STOPPING, LOW_STOPPED_HELD, LOW_LAUNCH, LOW_ROLLING_LOW, LOW_HANDED, LOW_STOPPING, LOW_STOPPED_HELD,
                                  LOW_STOPPED_FREE, LOW_CRAWL, LOW_MANEUVER, LOW_CRAWL, LOW_LAUNCH, LOW_ROLLING_LOW, LOW_HANDED };
    int n = sizeof expect / sizeof expect[0];
    char a[400], b[400];
    printf("   got      %s\n   expected %s\n", seq_text(trans, n_trans, a, sizeof a), seq_text(expect, n, b, sizeof b));
    bool same = n_trans == n;
    for (int i = 0; same && i < n; i++) same = trans[i] == expect[i];
    check(same, "the state sequence differs from the expected list");
    printf("   S0 %s\n", same ? "ok" : "FAIL");
}

// ---------------------------------------------------------------- off: with trace 0 the module does nothing at all
static void s_off()
{
    printf("== S-off trace 0: frames change nothing\n");
    set_param(m_lowspeed, "trace", 0);
    world(8, 30, 0);
    g_low.state = LOW_GAME;
    for (int i = 0; i < 600; i++) { if (i == 120) g_brake = 0.3f; tick(); }
    check(g_low.state == LOW_GAME && g_low.since_s == 0 && n_trans == 0, "the module moved with trace 0 (state %s, %d changes)", low_state_name(g_low.state), n_trans);
    set_param(m_lowspeed, "trace", 1);
    printf("   S-off %s\n", g_low.state == LOW_GAME && n_trans == 0 ? "ok" : "FAIL");
}

int main(int argc, char **argv)
{
    verbose = argc > 1 && !strcmp(argv[1], "-v");
    float r[16];
    for (int k = 0; k < 12; k++) r[k] = 11.73f * powf(0.79f / 11.73f, k / 11.0f);   // Optidriver ATO2612F: 12 speeds, no crawl gears
    memcpy(RATIO, r, sizeof r);
    for (module *m : { &m_trueshift, &m_lowspeed })
        for (int i = 0; i < m->n_params; i++) *m->params[i].val = m->params[i].def;
    set_param(m_trueshift, "idle_rpm", IDLE);
    set_param(m_lowspeed, "trace", 1);
    m_lowspeed.start();
    s_off();
    s0();
    printf("\n%d checks, %d failed\n", n_checks, n_failed);
    return n_failed ? 1 : 0;
}
