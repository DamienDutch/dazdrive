// Offline test of the Creep module's mode logic and speed controller (modules/creep.cpp is included, so its
// static functions are reachable). A simple truck: crawl force from the slipping clutch (stiffer as the floor
// closes), lock-up speed at idle, rolling resistance, grade, brakes; a stand-in for the game's automatic
// (neutral at a standstill with free pedals, engage on the plugin's request).
// Build+run: tools/sim/run_creep.sh [v]   (v = log lines)
#include "core/suite.h"
#include "core/game.h"
#include <math.h>
#include <stdarg.h>
#include <string.h>

telemetry T;
char g_dir[MAX_PATH];
static layout g_layout;
const layout *L = &g_layout;
game_addrs G;
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
void logring(const char *, ...) {}
void incident(const char *kind, const char *fmt, ...)
{
    g_n_incidents++;
    char b[300]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    printf("   INCIDENT %s: %s\n", kind, b);
}
const char *g_truck_unsupported;
volatile char g_dash_flash, g_dash_status;
static float sim_add_thr;   // the added throttle the truck model sees (what Creep posted the tick before, as a floor on the merged throttle)
int input_held(int) { return 0; }
void play_cue(int) {}
float game_adaptive() { return 3; }
void *game_cvar(const char *) { return 0; }
void game_cvar_set(void *, float) {}
static float g_mass = 18000;
float game_mass() { return g_mass; }
float wheel_radius() { return 0.5f; }
double g_dash_flash_until;
static double t_now;
double wall_s() { return t_now; }
double sim_s() { return t_now; }
void csv_cap(FILE **, const char *) {}
FILE *open_out(const char *, const char *) { return NULL; }
static int verbose;
void logline(const char *fmt, ...)
{
    if (!verbose) return;
    va_list ap; va_start(ap, fmt); printf("%7.2f  ", t_now); vprintf(fmt, ap); printf("\n"); va_end(ap);
}
int mem_readable(const void *, size_t) { return 0; }
uintptr_t game_scan(const char *) { return 0; }
int game_patch_tail(uintptr_t, void *) { return 0; }
int game_patch_clutch_floor(uintptr_t, clutch_floor *) { return 0; }
int game_patch_retarder(uintptr_t, const volatile float *) { return 0; }
int game_patch_cf0_if(uintptr_t, int, const unsigned char *) { return 0; }
int game_patch_call(uintptr_t, void *) { return 0; }
int game_engage_init() { return 0; }
void *game_gearbox() { return NULL; }
char *game_actor_safe() { return NULL; }
char *gb_inner(void *) { return NULL; }
int gb_gear(void *) { return 0; }
int gb_target(void *) { return 0; }
int gb_state(void *) { return 0; }
void gb_set_target(void *, int) {}
int gb_engage(void *, int) { return 0; }
// the vehicle model (core/vehicle.cpp) asks for these; the sim drives creep_tick / launch_tick directly, so it never runs
int gb_engine(void *, engine_info *) { return 0; }
float game_grade_deg(void *) { return NAN; }
float curve_factor(const engine_info *, float) { return 1; }

#include "modules/creep.cpp"
#include "core/rsc.cpp"
#include "core/actuate.cpp"
#include "core/vehicle.cpp"

// ---- truck
static struct { float v; int gear; double engage_at; int lock_ok; } M;
static const float RW = 0.5f, IDLE = 650, TMAX = 450, DIFF = 3.0f;
static const float RATIO[12] = { 12.9f, 8.2f, 5.3f, 3.7f, 2.5f, 1.8f, 1.3f, 1.0f, 0.8f, 0.7f, 0.6f, 0.5f };
static float rtot(int g) { return RATIO[(g < 0 ? 0 : g) > 0 ? (g < 0 ? -g : g) - 1 : 0] * DIFF; }
static float vlock(int g) { return IDLE * 0.10472f * RW / rtot(g); }  // m/s at idle, closed clutch
static float g_grade_pct, g_thr, g_brake, g_rpm;
static int g_parking, g_cruise, g_engine = 1;

static void stock_step(creep_in &in)
{
    // the automatic at a standstill with free pedals -> neutral (unless we hold the gear)
    int free = g_thr < 0.01f && g_brake < 0.01f;
    if (M.gear != 0 && fabsf(M.v) < 0.1f && free && !g_hold) M.gear = 0;
    if (g_hold && M.gear != g_hold) {
        if (M.engage_at == 0) M.engage_at = t_now + 0.4;  // opening, gap, engage
        if (t_now >= M.engage_at) { M.gear = g_hold; M.engage_at = 0; }
    } else M.engage_at = 0;
    // driver throttle with the stock automatic: pick a start gear
    if (M.gear == 0 && g_thr > 0.01f) M.gear = 1;
    (void)in;
}

static void step(double dt)
{
    creep_in in;
    in.dt = (float)dt; in.speed = M.v; in.thr = g_thr; in.brake = g_brake; in.rpm = g_rpm; in.idle = IDLE;
    in.gear = M.gear; in.engine_on = g_engine;
    in.blocked = g_parking || g_cruise;
    in.can_engage = 1;
    in.heavy = !(g_mass < p_heavy_t * 1000);
    in.set_max = (int)(in.heavy ? p_heavy_gear : p_max_gear);
    in.crawl_gear = pick_gear(in.heavy, IDLE);
    creep_tick(in);
    stock_step(in);
    // physics with floor g_f
    float MASS = g_mass;
    float dir = M.gear > 0 ? 1.f : (M.gear < 0 ? -1.f : 0.f);
    float open_ = fmaxf(g_f, 0.f);
    float Tc = TMAX * powf(1 - open_, 2.5f);
    float F = 0;
    float vd = M.v * dir;
    float vl = vlock(M.gear ? M.gear : 1);
    float R = rtot(M.gear ? M.gear : 1);
    if (M.gear != 0 && vd < vl) F = dir * Tc * R / RW * 0.5f;                 // slipping clutch: half of Tmax at idle
    if (M.gear != 0 && g_f < 0.3f) F += dir * sim_add_thr * 2000.f * R / RW;   // closed clutch: the added throttle on top
    float res = MASS * 9.81f * (0.012f + g_grade_pct / 100) * (M.v >= 0 ? 1.f : -1.f);
    if (fabsf(M.v) < 0.02f && fabsf(F) < fabsf(res) + 1) res = fabsf(F - 0) < fabsf(res) ? F : res;
    float fb = fmaxf(g_brake, g_cbrake) * MASS * 5 * (M.v > 0 ? -1.f : (M.v < 0 ? 1.f : 0.f));
    float a = (F - res + fb) / MASS;
    float v1 = M.v + a * (float)dt;
    if (g_brake > 0 && ((M.v > 0 && v1 < 0) || (M.v < 0 && v1 > 0))) v1 = 0;
    if (M.gear == 0 && fabsf(v1) < 0.02f && fabsf(F) < 1) v1 = fabsf(res) > 1 ? v1 : 0;
    M.v = v1;
    // engine rpm: idle while slipping; dragged down when the closed clutch cannot hold the speed
    g_rpm = IDLE;
    if (M.gear != 0 && g_f < 0.08f && vd < vl && F * dir < fabsf(res) * 1.0f - 1) g_rpm = IDLE * 0.8f;
    if (M.gear != 0 && vd > vl && g_f < 0.5f) g_rpm = IDLE * fminf(vd / vl, 2.5f) * (1 - g_f);  // wheels drive the engine
    T.speed = M.v; sim_add_thr = g_cthr; T.in_thr = g_thr; T.in_brake = g_brake; T.eff_brake = g_brake; T.rpm = g_rpm;
    T.parking_brake = g_parking; T.cruise = g_cruise ? 10.f : 0.f; T.engine_on = g_engine;
    t_now += dt;
}

static void reset(int gear, float v, int armed, float grade, float mass = 18000)
{
    memset(&M, 0, sizeof M);
    M.gear = gear; M.v = v / 3.6f;
    g_on = 0; g_hold = 0; g_f = 0; g_armed = armed; g_last_dir = 1; g_free_s = 0; g_vt = 0;
    g_thr = g_brake = 0; g_rpm = IDLE; g_parking = g_cruise = 0; g_engine = 1; g_grade_pct = grade;
    memset(&T, 0, sizeof T);
    T.fwd_gears = 12; T.diff = DIFF; T.wheels = 1; T.wheel_powered[0] = 1; T.wheel_radius[0] = RW;
    for (int i = 0; i < 12; i++) T.ratio[i] = RATIO[i];
    g_cthr = 0; sim_add_thr = 0; g_mass = mass;
    p_enabled = 1;
}

struct tr { double t; float kmh; int gear; float f; int on, hold; };

static void run(const char *name, double secs, void (*ev)(double))
{
    printf("== %s\n", name);
    double t0 = t_now;
    float maxf = 0, minf = 1, vmax = 0, vmin = 99; int ons = 0, last_on = g_on; double first_move = -1, settle = -1;
    float prev_f = g_f, max_df = 0, maxthr = 0;
    for (; t_now - t0 < secs;) {
        double t = t_now - t0;
        if (ev) ev(t);
        step(1 / 60.0);
        float kmh = M.v * 3.6f;
        if (first_move < 0 && fabsf(kmh) > 0.3f) first_move = t;
        if (g_on) { maxf = fmaxf(maxf, g_f); minf = fminf(minf, g_f); }
        vmax = fmaxf(vmax, kmh); vmin = fminf(vmin, kmh);
        max_df = fmaxf(max_df, fabsf(g_f - prev_f) * 60); prev_f = g_f;
        maxthr = fmaxf(maxthr, g_cthr);
        if (g_on && !last_on) ons++;
        last_on = g_on;
        if (settle < 0 && g_on && fabsf(fabsf(kmh) - g_vt * 3.6f) < 0.3f) settle = t;
        int sec = (int)(t * 1000);
        if (verbose && sec % 1000 < 17) printf("   t=%5.1f v=%5.2f km/h gear %2d thr %.2f brk %.2f on %d hold %d f=%.2f vt=%.2f rpm=%.0f\n", t, kmh, M.gear, g_thr, g_brake, g_on, g_hold, g_f, g_vt * 3.6f, g_rpm);
    }
    printf("   first move %.1f s, settled to target %.1f s, v %.2f..%.2f km/h, floor %.2f..%.2f (max %.2f/s), ends v=%.2f gear %d on %d f=%.2f, on-edges %d, max added thr %.2f\n",
           first_move, settle, vmin, vmax, minf, maxf, max_df, M.v * 3.6f, M.gear, g_on, g_f, ons, maxthr);
}


// ---- launch: the engine as an inertia against a slipping clutch (capacity ~ (1-c)^k, k unknown in the game: both ends are run)
static void run_launch(const char *name, float thr_end, float mass, float grade_pct, float k, int gear = 1)
{
    printf("== %s\n", name);
    memset(&T, 0, sizeof T);
    T.fwd_gears = 12; T.diff = DIFF;
    for (int i = 0; i < 12; i++) T.ratio[i] = RATIO[i];
    g_ln = 0; g_ln_done = 0; g_lc = 0; g_on = 0;
    p_launch = 1;
    const float J = 8.0f, CMAX = 3500, ETQ = 1800;  // J: the engine rises ~1200 rpm/s at full pedal from idle, like the game's
    float v_lock = 0, v = 0, w = IDLE / 9.5493f, c = 0, a_prev = 0, jerk_max = 0, t = 0, thr = 0, t10 = -1, rpm_max = 0, over_max = 0, lock_t = -1;
    const double dt = 1.0 / 240;
    const float R = RATIO[gear - 1] * DIFF;
    int n = 0;
    for (; t < 9 && !(lock_t >= 0 && t > lock_t + 0.5f); t += (float)dt, n++) {
        thr = t < 0.5f ? 0 : fminf(thr_end, thr + (float)dt * 3.0f);  // the pedal comes down in ~0.3 s
        float rpm = w * 9.5493f;
        if (n % 4 == 0) {  // 60 Hz plugin frame
            t_now = t;
            launch_in li = {};
            li.dt = 1.0f / 60; li.speed = v; li.thr = thr; li.brake = 0; li.rpm = rpm; li.idle = IDLE;
            li.rpm_per_ms = R / RW * 9.5493f; li.gear = gear; li.heavy = mass >= p_heavy_t * 1000; li.handover = -1;
            launch_tick(li);
            if (g_ln) c = g_lc; else if (g_ln_done || lock_t >= 0) c = 0;
            if (!g_ln && lock_t < 0 && c == 0 && t > 0.6f) { lock_t = t; v_lock = v; }
            if (!g_ln && !g_ln_done && c > 0 && t > 0.6f && lock_t < 0) c = c;  // not started: the game's own clutch (open) holds
        }
        if (!g_ln && lock_t < 0 && c == 0 && t < 0.6f) c = 1;
        float te = thr * ETQ * (0.6f + 0.4f * fminf(1, rpm / 1100)) + fmaxf(0, fminf(400.f, (IDLE - rpm) * 3.0f)) - 60;
        if (rpm > 1900) te = fminf(te, 0);
        float res = mass * 9.81f * (0.012f + grade_pct / 100);
        float slip = rpm - v * R / RW * 9.5493f;
        float acc;
        float cap = CMAX * powf(fmaxf(1 - c, 0), k);
        if (fabsf(slip) < 30 && cap > fabsf(te) + 50) {  // locked (the clutch holds the torque)
            float Jt = J + mass * RW * RW / (R * R);
            float dw = (te - res * RW / R) / Jt;
            w += dw * (float)dt; v = w * RW / R; acc = dw * RW / R;
        } else {
            float tc = CMAX * powf(fmaxf(1 - c, 0), k) * (slip > 0 ? 1 : -1);
            if (c >= 0.999f) tc = 0;
            w += (te - tc) / J * (float)dt;
            float F = tc * R / RW - res; if (v < 0.01f && F < 0) F = 0;
            acc = F / mass; v += acc * (float)dt;
        }
        if (v < 0) v = 0;
        if (n > 0) jerk_max = fmaxf(jerk_max, fabsf(acc - a_prev) / (float)dt * (t > 0.6f ? 1 : 0));
        a_prev = acc; if (g_ln) { rpm_max = fmaxf(rpm_max, rpm); over_max = fmaxf(over_max, rpm - g_ln_ra); }
        if (t10 < 0 && v * 3.6f >= 10) t10 = t;
        if (verbose && n % 20 == 0) printf("   t=%4.2f pedal %.2f v %5.2f km/h rpm %5.0f aim %5.0f clutch %.2f launch %d acc %.2f\n", t, thr, v * 3.6f, w * 9.5493f, g_ln_ra, c, g_ln, acc);
    }
    printf("   while launching: rpm max %.0f (%.0f over the aim), launch ended %s, clutch closed at %.1f s, speed then %.1f km/h\n", rpm_max, over_max, g_ln ? "NOT" : g_ln_why, lock_t, v_lock * 3.6f);
}

static void ev_none(double) {}
static void ev_brake(double t) { g_brake = t > 12 && t < 17 ? 0.3f : 0; }
static void ev_brake_then_reverse_ok(double t) { g_brake = t > 10 && t < 14 ? 0.4f : 0; }
static void ev_throttle(double t) { g_thr = t < 8 ? 0 : fminf(0.6f, (float)(t - 8) * 0.1f); }
static void ev_throttle_rest(double t) { g_thr = t < 4 ? 0 : 0.5f; }
static void ev_launch(double t) { g_brake = t < 3 ? 0.5f : 0; g_thr = t < 3 ? 0 : 0.5f; }
static void ev_modulate(double t) { g_brake = t > 10 && t < 16 ? 0.15f : 0; }
static void ev_brake_hv(double t) { g_brake = t > 10 && t < 16 ? 0.5f : 0; }
static void ev_parking(double) { g_parking = 1; }
static void ev_jam(double t)
{
    // creep, brake to stop, wait, release, creep again, tap brake twice
    g_brake = (t > 14 && t < 22) ? 0.25f : ((t > 34 && t < 35) ? 0.2f : 0);
}

int main(int argc, char **argv)
{
    verbose = argc > 1;
    g_layout.name = "1.60";
    for (int i = 0; i < m_creep.n_params; i++) *m_creep.params[i].val = m_creep.params[i].def;
    g_tail_ok = 1; g_engage_ok = 1;
    reset(0, 0, 1, 0);        run("rest in neutral after a drive, pedals free: engage and crawl (flat)", 25, ev_none);
    reset(0, 0, 0, 0);        run("same, not armed (fresh game start): must stay put", 8, ev_none);
    reset(0, 0, 1, 0);        run("parking brake on: must stay put", 8, ev_parking);
    reset(0, 0, 1, 0);        run("crawl, brake 0.3 at 12-17 s, release: stops, crawls again", 30, ev_brake);
    reset(0, 0, 1, 0);        run("crawl, throttle ramps from 8 s to 0.6: maneuver then hand over at 0.3", 25, ev_throttle);
    reset(1, 0, 1, 0);        run("stopped in gear 1, throttle 0.5 at 4 s: launch hand-over from rest", 12, ev_throttle_rest);
    reset(0, 0, 1, 4);        run("uphill 4%: crawl (may not reach target)", 25, ev_none);
    reset(0, 0, 1, -4);       run("downhill 4%: crawl (speed above target is slowed by opening)", 25, ev_none);
    reset(1, 0, 1, 0);        g_brake = 0.5f; run("gear 1 at rest with brake held 0.5, released with throttle 0.5 at 3 s", 10, ev_launch);
    reset(-1, -2.0f, 1, 0);   run("reverse at -2 km/h, pedals free: keeps crawling backward, brake at 10-14 s", 22, ev_brake_then_reverse_ok);
    reset(0, 0, 1, 0, 9000);  run("LIGHT 9 t flat: gear 3-4, clutch only, 7 km/h", 25, ev_none);
    reset(0, 0, 1, 5, 9000);  run("LIGHT 9 t uphill 5%", 25, ev_none);
    reset(0, 0, 1, 0, 9000);  run("LIGHT 9 t flat, brake 0.15 at 10-16 s (modulation lowers the speed)", 25, ev_modulate);
    reset(0, 0, 1, 0, 28000); run("HEAVY 28 t flat: gear 2 + added throttle", 30, ev_none);
    reset(0, 0, 1, 3, 28000); run("HEAVY 28 t uphill 3%", 30, ev_none);
    reset(0, 0, 1, 0, 28000); run("HEAVY 28 t flat, brake 0.5 at 10-16 s (throttle must drop to 0 at once)", 30, ev_brake_hv);
    reset(0, 0, 1, 0);        run("jam: crawl, brake to rest (14-22 s), release, crawl, tap brake at 34 s", 45, ev_jam);
    run_launch("LAUNCH 30 t flat, pedal 0.35, soft bite (k 2.5)", 0.35f, 30000, 0, 2.5f);
    run_launch("LAUNCH 30 t flat, pedal 0.35, hard bite (k 1.6)", 0.35f, 30000, 0, 1.6f);
    run_launch("LAUNCH 30 t flat, pedal 1.0, soft bite (k 2.5)", 1.0f, 30000, 0, 2.5f);
    run_launch("LAUNCH 30 t flat, pedal 1.0, hard bite (k 1.6)", 1.0f, 30000, 0, 1.6f);
    run_launch("LAUNCH 30 t uphill 4%, pedal 0.7 (k 2.0)", 0.7f, 30000, 4, 2.0f);
    run_launch("LAUNCH 13 t flat, pedal 0.5 (k 2.0)", 0.5f, 13000, 0, 2.0f);
    run_launch("LAUNCH 13 t flat, pedal 0.15 (k 2.0)", 0.15f, 13000, 0, 2.0f);
    {   // launch control (hidden): arms after 0.7 s of brake + flat pedal at a standstill, holds the revs, launches when the brake is released
        printf("== LAUNCH CONTROL state machine\n");
        memset(&T, 0, sizeof T); T.fwd_gears = 12; T.diff = DIFF; T.rpm_limit = 1900;
        for (int i = 0; i < 12; i++) T.ratio[i] = RATIO[i];
        g_ln = 0; g_ln_done = 0; g_lcx = 0; g_lcx_cap = 1; p_launch = 1;
        double t = 0; int armed_at = -1, go_at = -1; float cap_min = 1; int bad2 = 0;
        for (int n = 0; n < 60 * 6; n++, t += 1.0 / 60) {
            launch_in li = {};
            li.dt = 1.0f / 60; li.speed = 0; li.thr = 1.0f; li.brake = t < 3.0 ? 0.8f : 0.0f; li.rpm = t < 1.5 ? 1200.0f : 1700.0f;
            li.idle = IDLE; li.rpm_per_ms = RATIO[2] * DIFF / RW * 9.5493f; li.gear = 3; li.handover = -1;
            t_now = t; launch_tick(li);
            if (g_lcx == 1 && armed_at < 0) armed_at = n;
            if (g_lcx == 2 && go_at < 0) go_at = n;
            if (g_lcx == 1) cap_min = fminf(cap_min, g_lcx_cap);
        }
        bad2 |= !(armed_at > 40 && armed_at < 50) || go_at < 170 || go_at > 190 || cap_min > 0.45f || g_lcx_cap != 1;
        printf("   armed at frame %d (expect ~42), go at frame %d (expect ~180), cap fell to %.2f, cap after go %.2f, launch control %s\n", armed_at, go_at, cap_min, g_lcx_cap, bad2 ? "FAIL" : "ok");
        if (bad2) exit(1);
        g_ln = 0; g_lcx = 0; g_lcx_cap = 1;
    }
    return 0;
}
