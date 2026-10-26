// Offline test of the low-speed owner (modules/lowspeed.cpp is included, so its statics are reachable; TrueShift, the vehicle model
// and the actuator are linked for real: the start gear is TrueShift's physics pick, the brake request goes through the one writer).
// S0-S2 (owner=0, trace=1): a truck plant stands in for the game's automatic, TrueShift's stop and Creep's crawl: they drive, the owner
// only watches and decides, and the scenarios check what it would have commanded. Creep's and Cruise's states (crawl, smooth start, Auto
// Hold, Stop&Go) are inputs the script sets.
// S3-S6 (owner=1): the owner drives the stop and the standstill. The plant then runs the game's automatic as the notes describe it (at-rest
// rule, stop rule on the owner's threshold, TrueShift's braking downshift, the generic gearbox with its gap state), calls
// the owner's own tail hook before the generic gearbox, and stands in for Auto Hold, the ACC and Stop&Go with the arming lines of cruise.cpp
// (the stand-ins copy those lines, cruise.cpp itself is not part of this sim) next to a lead vehicle.
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
static struct { double v; int gear; float clutch; float rpm; int launching, crawl; double down_at; int target, drive, latch; } M;   // v m/s, clutch 0 closed .. 1 open; target, drive, latch: the box's gap state (S3-S6)
static int own_mode;                 // S3-S6: the plant is the game's automatic with the owner's hooks in
static float (*s_stop_fn)(float, float, float);   // TrueShift's stop threshold, as the game's stop rule calls it
static void (*s_tail_fn)(void *);    // the owner's hook at the update's tail
static int s_patch_ok;               // game_patch_call: the hooks are "found" (TrueShift starts) once the owner scenarios begin
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
int gb_target(void *) { return own_mode ? M.target : M.gear; }
int gb_state(void *) { return 0; }
int gb_engage(void *, int) { return 0; }
int game_engage_init() { return 1; }
void gb_set_target(void *, int g) { if (own_mode) M.target = g; }
void gb_rest_state(void *, int drive, int latch) { M.drive = drive; M.latch = latch; }
void gb_force(void *) {}
char *gb_inner(void *) { return NULL; }
float gb_thr_eff(void *) { return g_thr; }
int game_letter(void *, char) { return 1; }
int game_ebrake_downshift(void *) { return 0; }
float game_ebrake_level(void *) { return -1; }
int game_patch_call(uintptr_t, void *) { return s_patch_ok; }
int game_patch_tail(uintptr_t, void *pre) { s_tail_fn = (void (*)(void *))pre; return 1; }
int game_patch_stop(uintptr_t, float (*fn)(float, float, float)) { s_stop_fn = fn; return 1; }
int mem_readable(const void *, size_t) { return 0; }
char *game_actor_safe() { return NULL; }
int route_ok() { return 0; }
float route_grade(float, float) { return NAN; }
float route_max_grade(float, float) { return NAN; }
float route_min_grade(float, float) { return NAN; }
float route_crest_m(float) { return -1; }
float route_junction_m() { return -1; }
float route_curve_speed(float, float, float) { return NAN; }
uintptr_t game_scan(const char *) { return 0; }
int game_patch_clutch_floor(uintptr_t, clutch_floor *) { return 0; }
int game_patch_cf0_if(uintptr_t, int, const unsigned char *) { return 0; }
void creep_info_get(creep_info *o) { *o = CI; }
void cruise_info_get(cruise_info *o) { *o = CU; }

#include "modules/lowspeed.cpp"
#include "core/rsc.cpp"
#include "core/actuate.cpp"
extern module m_trueshift;

static float rpm_of(int g, double v) { return g > 0 ? (float)(v / RADIUS * RATIO[g - 1] * DIFF * 9.5493) : 0; }
static float kmh_of(int g, float rpm) { return rpm / (RATIO[g - 1] * DIFF * 9.5493f) * RADIUS * 3.6f; }   // the speed where gear g turns the engine at rpm

// ---- the plant: one step of dt seconds. What the truck does by itself (game + TrueShift + Creep), set into the telemetry.
static void plant(double dt)
{
    float idle = IDLE, thr = g_thr, brk = g_brake;
    double kmh = fabs(M.v) * 3.6;
    // the stop: the box goes to neutral when the engine would fall under idle + 150 rpm (+ the brake term); never while crawling
    if (M.gear > 0 && thr < 0.01f && !CI.crawl && rpm_of(M.gear, M.v) < idle + 150 + brk * brk * idle) { M.gear = 0; M.launching = 0; }
    // braking pulls the engine near idle: one gear down (TrueShift's braking downshift), at most every 0.25 s
    if (M.gear > 1 && thr < 0.01f && brk > 0.05f && rpm_of(M.gear, M.v) < idle + 250 + brk * brk * idle && t_now - M.down_at > 0.25) { M.gear--; M.down_at = t_now; }
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
    act_reset(level, why);
}

struct tick_rec { int state, would_gear, gear; float would_clutch, would_brake, kmh; };
static tick_rec rec[60 * 60];        // what the owner would command, per tick of a scenario
static int n_rec;
static int trans[64], n_trans;       // the state changes of a run, in order
static int last_state;

static void standins(double dt);
static void plant_game(double dt);
static char GA[0x400];   // the game's actor: the brake request lives at +0x17c, the actuator writes it

static void tick(double dt = 1.0 / 60)
{
    t_now += dt;
    T.sim_time = (scs_timestamp_t)(t_now * 1e6);
    if (own_mode) { standins(dt); act_brake_write(GA); plant_game(dt); }
    else plant(dt);
    vehicle_frame();
    m_lowspeed.frame();
    if (n_rec < (int)(sizeof rec / sizeof rec[0])) rec[n_rec++] = { g_low.state, g_low.would_gear, M.gear, g_low.would_clutch, g_low.would_brake, (float)(fabs(M.v) * 3.6) };
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
    n_trans = 0; n_engage_lines = 0; n_rec = 0;
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

// ---------------------------------------------------------------- S1: the stop point comes from the start gear, the open clutch is a latch
// The driver lifts at 30 km/h and brakes; the Auto Hold stand-in holds at a standstill.
static void hold_if_stopped() { CU.hold = fabs(M.v) * 3.6 < 0.5 && g_brake > 0.1f ? 1 : 0; CU.hold_req = CU.hold ? 0.6f : 0; }

static float start_at(float grade)   // the start gear (gentle pedal) at a road grade
{
    g_grade_pct = grade;
    vehicle_update(game_gearbox());
    return (float)ts_start_gear(0.3f);
}

static int entries(int state) { int n = 0; for (int i = 0; i < n_trans; i++) n += trans[i] == state; return n; }
static int first_open() { for (int i = 0; i < n_rec; i++) if (rec[i].state == LOW_STOPPING && rec[i].would_clutch > 0.5f) return i; return -1; }

static void s1_stop(float brake)
{
    world(8, 30, 0);
    M.gear = gear_for(30, 1300);
    M.rpm = rpm_of(M.gear, M.v);
    int sg = ts_start_gear(0.3f);
    float want = kmh_of(sg, IDLE + 150 + brake * brake * IDLE);   // the sim's own arithmetic: the start gear at idle + 150 rpm + the brake term
    for (int i = 0; i < 60 * 12; i++) {
        if (i == 60) g_brake = brake;
        hold_if_stopped();
        tick();
        if (i > 60 && M.v == 0 && i > 60 * 2 + 30) { if (g_low.since_s > 1) break; }
    }
    int o = first_open();
    bool before_closed = true, after_open = true, gear_const = true;
    int plant_changes = 0;
    for (int i = 0; i < n_rec; i++) {
        if (rec[i].state != LOW_STOPPING) continue;
        if (o < 0 || i < o) before_closed = before_closed && rec[i].would_clutch == 0;
        else { after_open = after_open && rec[i].would_clutch == 1; gear_const = gear_const && rec[i].would_gear == rec[o].would_gear; plant_changes += i > o && rec[i].gear != rec[i - 1].gear; }
    }
    float dv = (brake * 6 + 0.12f) * 3.6f / 60;   // the speed one tick covers
    check(entries(LOW_STOPPING) == 1, "S1 %.2f: %d STOPPING states instead of one", brake, entries(LOW_STOPPING));
    check(o > 0, "S1 %.2f: the clutch never opens", brake);
    if (o > 0) {
        check(rec[o].kmh <= want + 0.05f && rec[o].kmh > want - dv - 0.05f, "S1 %.2f: the clutch opens at %.2f km/h, the start gear %d sits at idle+150+brake term at %.2f", brake, rec[o].kmh, sg, want);
        check(before_closed && after_open, "S1 %.2f: would_clutch is not closed before the point and open after it", brake);
        check(gear_const && rec[o].would_gear == sg, "S1 %.2f: the gear to ask for changes after the point (start gear %d, at the point %d)", brake, sg, rec[o].would_gear);
        printf("   brake %.2f: one STOPPING, clutch opens at %.2f km/h (start gear %d, expected %.2f), gear to ask for %d until the stop (the truck itself changes gear %d time(s) meanwhile), ends %s\n",
               brake, rec[o].kmh, sg, want, rec[o].would_gear, plant_changes, low_state_name(g_low.state));
    }
    check(g_low.state == LOW_STOPPED_HELD, "S1 %.2f: ends in %s", brake, low_state_name(g_low.state));
    check(g_low.would_gear == sg && g_low.would_clutch == 1, "S1 %.2f: at rest it would have gear %d clutch %.1f", brake, g_low.would_gear, g_low.would_clutch);
}

// The start gear changes with the grade at the last metre. Lower after the clutch opened: it stays open. Higher before it opens: it opens at once.
static void s1_latch()
{
    float g_flat = 0, g_steep = -1;
    world(8, 30, 0);
    int a0 = (int)start_at(g_flat);
    for (float g = 1; g <= 25 && g_steep < 0; g += 0.5f) if ((int)start_at(g) < a0) g_steep = g;
    check(g_steep > 0, "S1 latch: no grade gives a lower start gear than %d", a0);
    if (g_steep < 0) return;
    int a1 = (int)start_at(g_steep);
    float v0 = kmh_of(a0, IDLE + 150 + 0.09f * IDLE), v1 = kmh_of(a1, IDLE + 150 + 0.09f * IDLE);   // idle points of both at brake 0.3
    printf("   start gear %d flat and %d at %.1f %%; at brake 0.3 their idle points are %.2f and %.2f km/h\n", a0, a1, g_steep, v0, v1);
    // lower: flat, the clutch opens at v0; three ticks later the grade steepens (the new gear's own point v1 is lower: it would still have the clutch closed)
    world(8, 30, g_flat);
    M.gear = gear_for(30, 1300); M.rpm = rpm_of(M.gear, M.v);
    int open_at = -1, changed_at = -1; float kmh_changed = 0;
    for (int i = 0; i < 60 * 12; i++) {
        if (i == 60) g_brake = 0.3f;
        hold_if_stopped();
        if (open_at < 0 && n_rec > 0 && rec[n_rec - 1].would_clutch > 0.5f && rec[n_rec - 1].state == LOW_STOPPING) open_at = i;
        if (open_at >= 0 && changed_at < 0 && i == open_at + 3) { changed_at = i; g_grade_pct = g_steep; kmh_changed = (float)(fabs(M.v) * 3.6); }
        tick();
        if (i > 60 * 2 && M.v == 0 && g_low.since_s > 1) break;
    }
    bool stays = true; int n_open = 0;
    for (int i = 0; i < n_rec; i++) if (rec[i].state == LOW_STOPPING && rec[i].would_clutch > 0.5f) n_open++;
    for (int i = 0; i < n_rec; i++) if (i >= open_at && (rec[i].state == LOW_STOPPING || rec[i].state == LOW_STOPPED_HELD)) stays = stays && rec[i].would_clutch == 1;
    check(open_at > 0 && kmh_changed > v1 && kmh_changed < v0 + 0.5f, "S1 latch lower: the grade changed at %.2f km/h, the new gear's point is %.2f", kmh_changed, v1);
    check(stays && entries(LOW_STOPPING) == 1 && g_low.state == LOW_STOPPED_HELD, "S1 latch lower: the clutch closed again or the stop was not clean (%s, %d STOPPING)", low_state_name(g_low.state), entries(LOW_STOPPING));
    check(g_low.would_gear == a1, "S1 latch lower: preselected gear at rest %d, the start gear of the end grade is %d", g_low.would_gear, a1);
    printf("   lower start gear (%d -> %d) after the clutch opened (grade change at %.2f km/h): clutch stays open for %d ticks, preselected gear at rest %d\n", a0, a1, kmh_changed, n_open, g_low.would_gear);
    // higher: steep, the clutch would open at v1; before it does, between v1 and v0 the grade flattens: the new gear's point is above us: open at once
    world(8, 30, g_steep);
    M.gear = gear_for(30, 1300); M.rpm = rpm_of(M.gear, M.v);
    int flat_at = -1, opened = -1; bool closed_before = true;
    for (int i = 0; i < 60 * 12; i++) {
        if (i == 60) g_brake = 0.3f;
        hold_if_stopped();
        if (flat_at < 0 && i > 60 && fabs(M.v) * 3.6 < 0.5 * (v0 + v1) + 0.1 && fabs(M.v) * 3.6 > v1 + 0.2) { flat_at = i; g_grade_pct = g_flat; }
        if (flat_at < 0 && n_rec > 0 && rec[n_rec - 1].would_clutch > 0.5f) closed_before = false;
        tick();
        if (flat_at >= 0 && opened < 0 && g_low.would_clutch > 0.5f) opened = i;
        if (i > 60 * 2 && M.v == 0 && g_low.since_s > 1) break;
    }
    check(flat_at > 0 && closed_before, "S1 latch higher: the grade change did not come between the two points (or the clutch opened before it)");
    check(opened >= 0 && opened <= flat_at + 1, "S1 latch higher: the clutch opened %d ticks after the grade change, not at once", opened - flat_at);
    check(entries(LOW_STOPPING) == 1 && g_low.state == LOW_STOPPED_HELD && g_low.would_gear == a0, "S1 latch higher: ends %s, %d STOPPING, preselected gear %d (start gear %d)", low_state_name(g_low.state), entries(LOW_STOPPING), g_low.would_gear, a0);
    printf("   higher start gear (%d -> %d) before the clutch opened: it opens %d tick(s) after the grade change, preselected gear at rest %d\n", a1, a0, opened - flat_at, g_low.would_gear);
}

// ---------------------------------------------------------------- S2: a pause at 4 km/h and the resume (a SOFT reset): the state is derived again
// Braking to a stop, the pause menu opens at 4 km/h. The game puts the box in neutral meanwhile. After the resume the owner derives
// its state from the telemetry alone: it must not remember a timer or a latch, and it must not ask for a gear that fits the speed
// (the old 'engage at 4 km/h: gear 12'): the start gear, with the clutch open.
static void s2(int pedal_after, int level)
{
    world(8, 30, 0);
    M.gear = gear_for(30, 1300); M.rpm = rpm_of(M.gear, M.v);
    for (int i = 0; i < 60 * 12 && fabs(M.v) * 3.6 > 4; i++) { if (i == 60) g_brake = 0.3f; hold_if_stopped(); tick(); }
    float kmh = (float)(fabs(M.v) * 3.6);
    int before = g_low.state, sg = ts_start_gear(0.3f);
    // the pause: nothing moves, the frame sees no time; the game drops the box to neutral and the plugin gets the reset on the resume
    T.paused = 1;
    m_lowspeed.frame();
    T.paused = 0;
    M.gear = 0; M.clutch = 1; M.launching = 0;
    sim_reset(level, "sim: resumed after a pause");
    g_thr = pedal_after ? 0.4f : 0;
    g_brake = pedal_after ? 0 : 0.3f;
    int fresh_before = g_fresh;
    tick();
    int expect = pedal_after ? LOW_LAUNCH : LOW_STOPPING;
    check(fresh_before == 1 && g_fresh == 0, "S2: the state was not derived again on the first frame after the reset");
    check(g_low.state == expect, "S2: after the resume the state is %s, expected %s", low_state_name(g_low.state), low_state_name(expect));
    check(g_low.would_gear == sg, "S2: it would have gear %d after the resume, the start gear is %d", g_low.would_gear, sg);
    check(pedal_after || g_low.would_clutch == 1, "S2: the clutch would not be open at once (%.1f)", g_low.would_clutch);
    int st1 = g_low.state, wg1 = g_low.would_gear; float wc1 = g_low.would_clutch;
    for (int i = 0; i < 60 * 3; i++) tick();   // and on: no gear that fits the speed shows up later either
    bool sane = true;
    for (int i = 0; i < n_rec; i++) sane = sane && rec[i].would_gear <= sg + 1;
    check(sane, "S2: a gear above the start gear was asked for");
    check(n_engage_lines == 0, "S2: %d 'engage' log lines", n_engage_lines);
    printf("   pause at %.1f km/h in %s, resumed %s: first frame %s, gear to ask for %d (start gear %d), clutch %.1f; 'engage' lines %d\n", kmh, low_state_name(before),
           pedal_after ? "on the pedal" : "braking", low_state_name(st1), wg1, sg, wc1, n_engage_lines);
}

// A teleport (HARD reset) in the middle of a launch: the truck stands afterwards, nothing of the launch is left.
static void s2_teleport()
{
    world(0, 0, 0);
    g_thr = 0.4f;
    for (int i = 0; i < 60 * 2 && fabs(M.v) * 3.6 < 2; i++) tick();
    int before = g_low.state;
    g_thr = 0; M.v = 0; M.gear = 0; M.clutch = 1; M.launching = 0;
    sim_reset(RESET_HARD, "sim: teleport");
    tick();
    check(before == LOW_LAUNCH && g_low.state == LOW_STOPPED_FREE, "S2 teleport: %s before, %s after", low_state_name(before), low_state_name(g_low.state));
    printf("   teleport during a launch: %s -> %s\n", low_state_name(before), low_state_name(g_low.state));
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


// ---------------------------------------------------------------- S3-S6 (owner=1): the game's automatic, Auto Hold, the ACC and Stop&Go as stand-ins
// The game's update, in its order: the at-rest rule (neutral, drive state 3, every update), the stop rule on TrueShift's threshold (the game
// adds its brake term on top of the hook's value), TrueShift's braking downshift, the owner's tail hook, the generic gearbox (from its gap
// state it engages the target gear only with throttle: a pedal, or Stop&Go's fed cruise throttle). Auto Hold, the ACC and Stop&Go copy the
// arming lines of cruise.cpp (auto_hold: the OFF -> HOLD test; the Stop&Go bookkeeping; the ACC's approach brake), cruise.cpp itself is not
// part of this sim. The brake requests go through the real actuator.
static struct { int on; double x, v; } LEAD;                  // the lead vehicle: x ahead of the truck's origin, v m/s (the script sets it)
static struct { int on; float set, u, feed; double stopped_at; int sg_hold; } ACC;   // cruise: on, set speed (m/s), its throttle, the fed throttle, Stop&Go
static struct { int on; int state; float r; int engaged; } AH;   // Auto Hold stand-in: enabled, 0 off 1 hold, request, engagements
static double X;                         // the truck's position
static float SG_AUTO = 2;                // [cruise] stop_go_auto_s
static int n_cancel, n_unheld_neutral;   // the game cancelled the cruise at a neutral without the hold / ticks in neutral with the cruise on and no hold
static int sg_tick = -1, neutral_tick = -1, tick_no;
static int n_pre_writes, last_pre_tgt;   // changes of the box's target while in neutral after the owner's tail (the preselect), as the generic gearbox sees them
static float peak_sg_req;                // the brake request held while Stop&Go holds


static void standins(double dt)
{
    float v = (float)M.v, kmh = v * 3.6f, gap = (float)(LEAD.x - X), lv = (float)LEAD.v;
    double now = t_now;
    LEAD.x += LEAD.v * dt;
    // the ACC: speed target from the lead (own_acc: lead speed + 0.3 x (gap - 6 m)), its throttle, the approach brake
    float vt = ACC.set;
    if (LEAD.on) vt = fminf(vt, lv < 0.1f && gap < 8 ? 0 : fmaxf(0, lv + 0.3f * (gap - 6)));
    float b = 0;
    if (ACC.on && LEAD.on && v > 0.05f && v > lv) {
        float a_t = (v - lv) * (v - lv) / (2 * fmaxf(0.3f, gap - 5));
        if (a_t > 0.4f || gap < 6.5f) b = fminf(0.45f, 0.08f + a_t / 7.0f);
        if (gap < 6.5f && b > 0) b = fmaxf(b, 0.25f);
    }
    ACC.u = ACC.on && b <= 0 ? clampf_(0.5f * (vt - v), 0, 0.6f) : 0;
    act_brake(ACT_ACC, b);   // standing: nothing, the owner's hold (cruise.cpp posts its 0.2 only without the owner)
    // Stop&Go (cruise.cpp, the bookkeeping at 890-902)
    int own = low_owner_on(), slow = own ? low_sg_zone() : kmh < 3, stopped = own ? low_stopped() : slow;
    if (ACC.on) {
        if (!stopped) ACC.stopped_at = 0;
        else if (!ACC.stopped_at) ACC.stopped_at = now;
        int may_wait = SG_AUTO <= 0 || (ACC.stopped_at ? now - ACC.stopped_at < SG_AUTO : own);
        ACC.sg_hold = slow && LEAD.on && gap > 0.5f && may_wait && !T.parking_brake;
        ACC.feed = (kmh < 10 && (ACC.sg_hold || ACC.stopped_at) && ACC.u > 0.02f) ? ACC.u : 0;
    } else { ACC.sg_hold = 0; ACC.feed = 0; ACC.stopped_at = 0; }
    if (ACC.sg_hold && sg_tick < 0) sg_tick = tick_no;
    // Auto Hold (cruise.cpp auto_hold): armed by the owner's standstill and the brake pedal; Stop&Go's hold blocks it; off with the pedal
    if (AH.on) {
        int blocked = T.parking_brake || ACC.sg_hold;
        if (blocked) AH.state = 0;
        if (AH.state == 0) {
            AH.r = AH.r > 0 ? fmaxf(0, AH.r - 3 * (float)dt) : 0;
            if (!blocked && (own ? low_stopped() : kmh < 0.5f) && g_brake > 0.1f) { AH.state = 1; AH.engaged++; }
        } else {
            AH.r = fminf(0.6f, AH.r + 3 * (float)dt);
            if (g_thr > 0.05f || kmh > 1.5f) AH.state = 0;
        }
        act_brake(ACT_HOLD, AH.r);
    }
    CU.hold = AH.state; CU.hold_req = AH.r; CU.sg_hold = ACC.sg_hold; CU.feed = ACC.feed;
    T.cruise = ACC.on ? ACC.set : 0;
    tick_no++;
}

static void plant_game(double dt)
{
    float idle = IDLE, brk = fmaxf(g_brake, *(float *)(GA + 0x17c)), thr = fmaxf(g_thr, ACC.on ? ACC.u : 0), fed = fmaxf(g_thr, ACC.feed);
    double kmh = fabs(M.v) * 3.6;
    T.gear = T.dgear = M.gear; T.in_thr = g_thr; T.eff_thr = thr; T.in_brake = g_brake; T.eff_brake = brk;
    // the at-rest rule: neutral every update
    if (M.gear == 0 && kmh < 0.3) { M.target = 0; M.drive = 3; M.latch = 1; }
    // the stop rule: neutral when the engine of the gear that is in falls under the threshold (hook value + the game's brake term)
    if (M.gear > 0 && thr < 0.01f) {
        float lim = (s_stop_fn ? s_stop_fn(idle, thr, brk) : 0.8f * idle) + brk * brk * idle;
        if (rpm_of(M.gear, M.v) < lim) {
            M.gear = 0; M.target = 0; M.drive = 3; M.latch = 1; M.launching = 0;
            if (neutral_tick < 0) neutral_tick = tick_no;
            if (ACC.on && !ACC.sg_hold) { ACC.on = 0; n_cancel++; }   // the game cancels the cruise at a neutral unless Stop&Go's hold is set
        }
    }
    if (ACC.on && M.gear == 0 && !ACC.sg_hold) { ACC.on = 0; n_cancel++; }   // ... and when the hold lapses at a standstill (stop_go_auto_s): Resume or the pedal
    if (ACC.on && M.gear == 0 && !ACC.sg_hold) n_unheld_neutral++;
    // TrueShift's braking downshift
    if (M.gear > 1 && thr < 0.01f && brk > 0.05f && rpm_of(M.gear, M.v) < idle + 250 + brk * brk * idle && t_now - M.down_at > 0.25) { M.gear--; M.down_at = t_now; M.target = M.gear; }
    if (M.gear > 0) M.target = M.gear;
    if (s_tail_fn) s_tail_fn(&M);   // the owner's tail: the preselected gear in the gap
    if (M.gear == 0) {
        if (M.target != last_pre_tgt) n_pre_writes++;
        last_pre_tgt = M.target;
    }
    // the generic gearbox: from the gap state only with throttle (a pedal, or the fed cruise throttle)
    if (M.gear == 0 && fed > 0.01f) { M.gear = M.target > 0 ? M.target : ts_start_gear(fmaxf(fed, 0.3f)); M.target = M.gear; M.launching = 1; M.drive = 4; M.latch = 0; }
    if (M.gear > 0 && M.launching && fed < 0.01f && M.v < 0.05) M.launching = 0;
    // motion
    float rlock = rpm_of(M.gear, M.v), aim = idle + 300 + 600 * fed;
    float a = 0;
    if (M.gear > 0) a = thr * 3.0f;
    a -= 0.12f + 9.81f * g_grade_pct / 100;
    a -= brk * 6;
    float v1 = (float)(M.v + a * dt);
    if (v1 < 0 && M.v >= 0 && (brk > 0 || a < 0)) v1 = 0;
    M.v = v1 < 0 ? 0 : v1;
    X += M.v * dt;
    if (M.gear == 0) { M.clutch = 1; M.rpm = idle; M.launching = 0; }
    else if (M.launching && rlock < aim) { M.clutch = 0.35f; M.rpm = aim; }
    else { M.launching = 0; M.clutch = 0; M.rpm = fmaxf(idle, rlock); }
    if (M.gear > 0 && M.gear < 12 && !M.launching && thr > 0.3f && M.rpm > 1500) { M.gear++; M.target = M.gear; }
    T.speed = (float)M.v; T.rpm = M.rpm; T.eff_clutch = M.clutch; T.gear = T.dgear = M.gear; T.engine_on = 1;
    static double last_t; static float last_v;
    double d = t_now - last_t;
    if (d > 0.0 && d < 0.5) { float acc = (float)((fabsf(T.speed) - last_v) / d); T.accel += (float)(d / (0.5 + d)) * (acc - T.accel); }
    last_t = t_now; last_v = fabsf(T.speed);
}

// TrueShift's stop hook and the owner's tail hook "found", the owner switched on: once, before the first owner scenario
static void own_setup()
{
    s_patch_ok = 1;
    G.decide_site = 1; G.stop_site = 1; G.tail_site = 0;
    m_trueshift.start();                 // installs the stop hook (the sim's stand-in captures it); no tail of its own
    G.tail_site = 1;
    set_param(m_lowspeed, "owner", 1);
    m_lowspeed.start();                  // the owner's tail hook
    own_mode = 1;
    check(s_stop_fn != NULL && s_tail_fn != NULL, "the stop hook and the owner's tail hook were not installed");
}

static void own_world(int gear, double kmh, float grade, float mass = 36000)
{
    for (int o = 0; o < ACT_N; o++) { act_brake(o, 0); act_thr_floor(o, 0); }
    memset(GA, 0, sizeof GA);
    world(gear, kmh, grade, mass);
    memset(&LEAD, 0, sizeof LEAD); memset(&ACC, 0, sizeof ACC); memset(&AH, 0, sizeof AH);
    X = 0; n_cancel = n_unheld_neutral = 0; sg_tick = neutral_tick = -1; tick_no = 0; n_pre_writes = 0; last_pre_tgt = 0; peak_sg_req = 0;
    s_patch_ok = 1;
}

static float req_now() { return *(float *)(GA + 0x17c); }
static int still(double kmh = 0.05) { return fabs(M.v) * 3.6 < kmh; }

// S3: the cruise is on, the lead in front stops; the truck stops behind it, the owner holds (Stop&Go), the lead pulls away
static void s3(float stand_s, bool tap)
{
    own_world(gear_for(25, 1100), 25, 0);
    ACC.on = 1; ACC.set = 30 / 3.6f; LEAD.on = 1; LEAD.x = 45; LEAD.v = 25 / 3.6;
    SG_AUTO = 2;
    double t_stopped = -1, t_go = -1, t_move = -1; float min_gap = 1e9f, hold_req = 0; int gear0_unheld = 0, held_ticks = 0, tapped = 0;
    double t0 = t_now;
    for (int i = 0; i < 60 * 60; i++) {
        double t = t_now - t0;
        if (t > 3 && LEAD.v > 0 && t_go < 0) LEAD.v = fmax(0, LEAD.v - 1.2 / 60);                       // the lead brakes to a stop
        if (t_stopped < 0 && still() && t > 4) t_stopped = t;
        if (t_stopped >= 0 && t_go < 0 && t - t_stopped >= stand_s) { t_go = t; }
        if (t_go >= 0) LEAD.v = fmin(15 / 3.6, LEAD.v + 1.0 / 60);                                   // the lead pulls away
        if (tap && t_go >= 0 && t - t_go > 2 && !tapped && !ACC.on) { g_thr = 0.3f; tapped = 1; }   // a gas tap after the wait (Resume would set the cruise on again)
        if (tapped && t - t_go > 3) { g_thr = 0; ACC.on = 1; }
        tick();
        min_gap = fminf(min_gap, (float)(LEAD.x - X));
        if (M.gear == 0 && ACC.on && !ACC.sg_hold) gear0_unheld++;
        if (ACC.sg_hold && still()) { held_ticks++; hold_req = req_now(); peak_sg_req = fmaxf(peak_sg_req, hold_req); }
        if (t_go >= 0 && t_move < 0 && M.v * 3.6 > 2) t_move = t;
        if (t_move >= 0 && t - t_move > 4) break;
    }
    bool within = stand_s < SG_AUTO;
    printf("   lead stops, truck stopped at %.1f s (gap %.1f m), stands %.1f s: cruise %s, %d cancel(s), sg_hold from tick %d, neutral from tick %d, hold request %.2f, drives off %s\n",
           t_stopped, min_gap, stand_s, ACC.on ? "on" : "off", n_cancel, sg_tick, neutral_tick, peak_sg_req,
           t_move < 0 ? "never" : tap && !within ? "after the tap" : "by itself");
    check(t_stopped > 0 && min_gap > 0.5f, "S3: the truck did not stop behind the lead (stopped %.1f s, min gap %.1f m)", t_stopped, min_gap);
    check(sg_tick >= 0 && (neutral_tick < 0 || sg_tick <= neutral_tick), "S3: Stop&Go's hold (tick %d) came after the neutral (tick %d)", sg_tick, neutral_tick);
    check(gear0_unheld == 0 && (within ? n_cancel == 0 : n_cancel <= 1), "S3: gear 0 for %d ticks without the hold, %d cruise cancel(s)", gear0_unheld, n_cancel);
    check(peak_sg_req >= 0.15f && peak_sg_req <= 0.65f, "S3: hold request %.2f", peak_sg_req);
    check(held_ticks > 0, "S3: the truck was never held at a standstill by Stop&Go");
    if (within) {
        check(ACC.on, "S3: the cruise is off");
        check(t_move > 0 && t_move - t_go < 3, "S3: no drive-off by itself (%.1f s after the lead left)", t_move - t_go);
        check(g_low.state == LOW_LAUNCH || g_low.state == LOW_ROLLING_LOW || g_low.state == LOW_HANDED, "S3: after the drive-off the state is %s", low_state_name(g_low.state));
    } else {
        check(t_move < 0 || tapped, "S3: drove off by itself after stop_go_auto_s");
        check(tap && t_move > 0 && ACC.on, "S3: no drive-off after the gas tap");
    }
    check(n_engage_lines == 0, "S3: %d 'engage' log lines", n_engage_lines);
}

// S4: Auto Hold engages exactly once at rest; over 20 s no neutral or gear flicker
static void s4(int ah_on)
{
    own_world(gear_for(30, 1300), 30, 0);
    AH.on = ah_on;
    int sg = 0, end = -1, gear_changes = 0, last_gear = -1, open_all = 1, states_ok = 1, req_ok = 1; double t0 = t_now, t_rest = -1;
    for (int i = 0; i < 60 * 60; i++) {
        double t = t_now - t0;
        if (i == 60) g_brake = 0.3f;
        if (still() && t_rest < 0) { t_rest = t; sg = ts_start_gear(0.3f); }
        if (t_rest >= 0 && t - t_rest > 1 && g_brake > 0) g_brake = 0;   // the driver lets go of the pedal; Auto Hold (or nothing) holds
        tick();
        if (t_rest >= 0) {
            if (M.gear != last_gear && last_gear >= 0) gear_changes++;
            last_gear = M.gear;
            open_all = open_all && M.gear == 0 && M.clutch == 1;
            states_ok = states_ok && (g_low.state == LOW_STOPPED_HELD || g_low.state == LOW_STOPPED_FREE);
            if (t - t_rest > 2 && ah_on) req_ok = req_ok && req_now() > 0.3f;
        }
        if (t_rest >= 0 && t - t_rest > 21) { end = i; break; }
    }
    check(end > 0, "S4: the truck did not stop");
    check(!ah_on || AH.engaged == 1, "S4: Auto Hold engaged %d times", AH.engaged);
    check(gear_changes == 0 && open_all, "S4: %d gear changes at rest, clutch open all the time: %d", gear_changes, open_all);
    check(n_pre_writes <= 1 && M.target == sg, "S4: the box's target changed %d times in neutral, ends on %d (start gear %d)", n_pre_writes, M.target, sg);
    check(states_ok, "S4: the state left the standstill states");
    check(req_ok, "S4: the hold was not on");
    check(g_low.state == (ah_on ? LOW_STOPPED_HELD : LOW_STOPPED_FREE), "S4: ends in %s", low_state_name(g_low.state));
    // and the pedal: the preselected gear is the one that engages
    g_thr = 0.4f;
    for (int i = 0; i < 30; i++) tick();
    check(M.gear == sg && n_engage_lines == 0, "S4: the pedal engaged gear %d, the start gear was %d; %d 'engage' log lines", M.gear, sg, n_engage_lines);
    printf("   Auto Hold %s: engaged %d time(s), %.0f s at rest: %d target write(s) in neutral (the preselect %d), %d gear change(s), clutch open throughout, state %s; pedal engages gear %d\n",
           ah_on ? "on" : "off", AH.engaged, 20.0, n_pre_writes, sg, gear_changes, ah_on ? "STOPPED_HELD" : "STOPPED_FREE", M.gear);
}

// S5: the start gear changes at the last metre (a grade change), the real plant: the clutch is open or opens at once, never closes before the pedal
static void s5()
{
    float g_flat = 0, g_steep = -1;
    own_world(8, 30, 0);
    int a0 = (int)start_at(g_flat);
    for (float g = 1; g <= 25 && g_steep < 0; g += 0.5f) if ((int)start_at(g) < a0) g_steep = g;
    check(g_steep > 0, "S5: no grade gives a lower start gear than %d", a0);
    if (g_steep < 0) return;
    int a1 = (int)start_at(g_steep);
    float v0 = kmh_of(a0, IDLE + 150 + 0.09f * IDLE), v1 = kmh_of(a1, IDLE + 150 + 0.09f * IDLE);
    // lower: the clutch opens at v0, three ticks later the road steepens
    own_world(8, 30, g_flat);
    M.gear = gear_for(30, 1300); M.rpm = rpm_of(M.gear, M.v); M.target = M.gear;
    int open_at = -1; float kmh_changed = 0; bool closed_again = false, wrote_closed = false;
    for (int i = 0; i < 60 * 14; i++) {
        if (i == 60) g_brake = 0.3f;
        if (open_at >= 0 && i == open_at + 3) { g_grade_pct = g_steep; kmh_changed = (float)(fabs(M.v) * 3.6); }
        tick();
        if (open_at < 0 && M.gear == 0) open_at = i;
        if (open_at >= 0 && (M.gear != 0 || M.clutch != 1)) closed_again = true;
        if (open_at >= 0 && (g_low.state == LOW_STOPPING || g_low.state == LOW_STOPPED_HELD) && g_low.would_clutch != 1) wrote_closed = true;
        if (open_at > 0 && still() && g_low.since_s > 2) break;
    }
    check(open_at > 0 && kmh_changed > v1 && kmh_changed < v0 + 0.5f, "S5 lower: the grade changed at %.2f km/h (points %.2f and %.2f)", kmh_changed, v1, v0);
    check(!closed_again && !wrote_closed, "S5 lower: the clutch closed again or the owner asked for it");
    check(g_low.state == LOW_STOPPED_HELD && M.target == a1, "S5 lower: ends %s, preselected gear %d, the start gear at the stop is %d", low_state_name(g_low.state), M.target, a1);
    printf("   lower start gear (%d -> %d) at %.2f km/h after the clutch opened: clutch stays open to the stop, preselected gear %d\n", a0, a1, kmh_changed, M.target);
    int pre_at_stop = M.target;
    g_thr = 0.4f; g_brake = 0;
    for (int i = 0; i < 30; i++) tick();
    check(M.gear == pre_at_stop, "S5 lower: the pedal engaged gear %d, preselected %d", M.gear, pre_at_stop);
    // higher: steep, the clutch would open at v1; between v1 and v0 the road flattens: the new gear's point is above us, open at once
    own_world(8, 30, g_steep);
    M.gear = gear_for(30, 1300); M.rpm = rpm_of(M.gear, M.v); M.target = M.gear;
    int flat_at = -1, opened = -1; bool closed_before = true; closed_again = false;
    for (int i = 0; i < 60 * 14; i++) {
        if (i == 60) g_brake = 0.3f;
        if (flat_at < 0 && i > 60 && fabs(M.v) * 3.6 < 0.5 * (v0 + v1) + 0.1 && fabs(M.v) * 3.6 > v1 + 0.2) { flat_at = i; g_grade_pct = g_flat; }
        tick();
        if (flat_at < 0 && M.gear == 0) closed_before = false;
        if (flat_at >= 0 && opened < 0 && M.gear == 0) opened = i;
        if (opened >= 0 && (M.gear != 0 || M.clutch != 1)) closed_again = true;
        if (opened > 0 && still() && g_low.since_s > 2) break;
    }
    check(flat_at > 0 && closed_before, "S5 higher: the grade change did not come between the two points (or the clutch opened before it)");
    check(opened >= 0 && opened <= flat_at + 2, "S5 higher: the clutch opened %d ticks after the grade change, not at once", opened - flat_at);
    check(!closed_again && g_low.state == LOW_STOPPED_HELD && M.target == a0, "S5 higher: ends %s, preselected gear %d (start gear %d)", low_state_name(g_low.state), M.target, a0);
    printf("   higher start gear (%d -> %d) before the clutch opened: opens %d tick(s) after the grade change, never closes, preselected gear %d\n", a1, a0, opened - flat_at, M.target);
}

// S5 gap: a braking downshift passes gear 0 above the open point (the box's shift gap): no latch, the shift's target stays, the stop is clean
static void s5_gap()
{
    own_world(8, 30, 0);
    M.gear = gear_for(30, 1300); M.rpm = rpm_of(M.gear, M.v); M.target = M.gear;
    int gap_at = -1, gap_n = 0, next = 0; float kmh_gap = 0, open_kmh = 0; bool latched = false, tgt_kept = true;
    for (int i = 0; i < 60 * 14; i++) {
        if (i == 60) g_brake = 0.15f;
        float kmh = (float)(fabs(M.v) * 3.6);
        if (gap_at < 0 && g_low.state == LOW_STOPPING && M.gear > 1 && kmh > g_open_kmh + 2) {   // STOPPING: under the top of the owner's world
            gap_at = i; next = M.gear - 1; kmh_gap = kmh; open_kmh = g_open_kmh;
        }
        if (gap_at >= 0 && gap_n < 6) { M.gear = 0; M.target = next; M.down_at = t_now; gap_n++; }   // the gap: the box in gear 0 with the next gear as its target
        else if (gap_n == 6) { M.gear = next; M.target = next; gap_n++; }
        tick();
        if (gap_at >= 0 && gap_n <= 6) { latched = latched || g_latch; tgt_kept = tgt_kept && M.target == next; }
        if (gap_at > 0 && still() && g_low.since_s > 1.5) break;
    }
    int sg = ts_start_gear(0.3f);
    check(gap_at > 0, "S5 gap: no STOPPING state above the open point in a gear over 1");
    check(!latched && tgt_kept, "S5 gap: the gap at %.2f km/h (open point %.2f) latched the clutch (%d) or the target changed (kept %d)", kmh_gap, open_kmh, latched, tgt_kept);
    check(g_low.state == LOW_STOPPED_HELD || g_low.state == LOW_STOPPED_FREE, "S5 gap: ends %s", low_state_name(g_low.state));
    check(M.gear == 0 && M.target == sg, "S5 gap: ends in gear %d, preselect %d (start gear %d)", M.gear, M.target, sg);
    printf("   a downshift's gap at %.2f km/h (open point %.2f): no latch, the shift's target %d kept, ends %s with gear %d preselected\n",
           kmh_gap, open_kmh, next, low_state_name(g_low.state), M.target);
}

// S6: a stop from 30 km/h under a brake of 0.15 and 0.5, at three masses: the real plant (the game's stop rule on the owner's threshold)
static int s6_open_gear;
static void s6(float brake, float mass_t)
{
    own_world(8, 30, 0, mass_t * 1000);
    M.gear = gear_for(30, 1300); M.rpm = rpm_of(M.gear, M.v); M.target = M.gear;
    AH.on = 1;
    int sg = ts_start_gear(0.3f);
    float want = kmh_of(sg, IDLE + 150 + brake * brake * IDLE);
    int open_at = -1, gear_after = 0; float kmh_open = 0, prev = 30;   // the speed the game's stop rule saw is the one of the tick before
    for (int i = 0; i < 60 * 14; i++) {
        if (i == 60) g_brake = brake;
        tick();
        if (open_at < 0 && M.gear == 0) { open_at = i; kmh_open = prev; }
        prev = (float)(fabs(M.v) * 3.6);
        if (open_at >= 0 && M.gear != 0) gear_after++;
        if (open_at > 0 && still() && g_low.since_s > 1.5) break;
    }
    float dv = (brake * 6 + 0.12f) * 3.6f / 60;
    s6_open_gear = sg;
    check(open_at > 0 && kmh_open <= want + 0.05f && kmh_open > want - dv - 0.05f, "S6 %.2f/%.0f t: the clutch opens at %.2f km/h, the start gear %d sits at idle+150+brake term at %.2f", brake, mass_t, kmh_open, sg, want);
    check(gear_after == 0, "S6 %.2f/%.0f t: %d ticks in gear after the clutch opened", brake, mass_t, gear_after);
    check(g_low.state == LOW_STOPPED_HELD && M.gear == 0 && M.target == sg && AH.engaged == 1, "S6 %.2f/%.0f t: ends %s, gear %d, preselect %d (start gear %d), Auto Hold %d x", brake, mass_t, low_state_name(g_low.state), M.gear, M.target, sg, AH.engaged);
    check(n_engage_lines == 0, "S6: %d 'engage' log lines", n_engage_lines);
    printf("   %4.0f t brake %.2f: start gear %2d, clutch opens at %.2f km/h (idle point %.2f), no gear after it, ends %s with gear %d preselected\n", mass_t, brake, sg, kmh_open, want, low_state_name(g_low.state), M.target);
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
    s_hooked = 1;   // the actuator hook is "in" (actuate.cpp is part of this sim)
    m_lowspeed.start();
    s_off();
    s0();
    printf("== S1 stop from 30 km/h: the clutch opens at the idle point of the start gear\n");
    s1_stop(0.15f);
    s1_stop(0.5f);
    s1_latch();
    printf("== S2 pause at 4 km/h, resume: the state is derived again, no gear for the speed\n");
    s2(0, RESET_SOFT);
    s2(1, RESET_SOFT);
    s2_teleport();
    own_setup();
    printf("== S3 stop behind a stopping lead with the cruise on: one hold by the owner, no gear 0 before it, drives off by itself within stop_go_auto_s\n");
    s3(1.0f, false);
    printf("   the same, the lead leaves after stop_go_auto_s: the cruise drops at the standstill, a gas tap drives off\n");
    s3(4.0f, true);
    printf("== S4 Auto Hold engages once at rest, 20 s with no neutral or gear flicker\n");
    s4(1);
    s4(0);
    printf("== S5 the start gear changes at the last metre: the open clutch is a latch\n");
    s5();
    s5_gap();
    printf("== S6 stop from 30 km/h: the clutch opens at the start gear's idle point, derived from the mass\n");
    int gears[6], n = 0;
    for (float b : { 0.15f, 0.5f }) for (float m : { 18.0f, 25.0f, 36.0f }) { s6(b, m); gears[n++] = s6_open_gear; }
    bool moves = false;
    for (int i = 1; i < 3; i++) moves = moves || gears[i] != gears[0];
    check(moves, "S6: the start gear is %d, %d and %d at 18, 25 and 36 t: the open point does not move with the mass", gears[0], gears[1], gears[2]);
    printf("\n%d checks, %d failed\n", n_checks, n_failed);
    return n_failed ? 1 : 0;
}
