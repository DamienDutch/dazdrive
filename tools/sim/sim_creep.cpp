// Offline test of the Creep module's mode logic and speed controller (modules/creep.cpp is included, so its
// static functions are reachable). A simple truck: crawl force from the slipping clutch (stiffer as the floor
// closes), lock-up speed at idle, rolling resistance, grade, brakes; a stand-in for the game's automatic
// (neutral at a standstill with free pedals, engage on the plugin's request).
// Build+run: tools/sim/run_creep.sh [v]   (v = log lines)
#include "core/suite.h"
#include "core/game.h"
#include "sim_layout.h"
#include <initializer_list>
#include <math.h>
#include <stdarg.h>
#include <string.h>

telemetry T;
char g_dir[MAX_PATH];
const layout *L = &sim_layout;
game_addrs G;
volatile int g_rolling, g_cruise_smooth;
volatile float g_roll_boost_kmh;
volatile double g_rb_cancel_at = -1e9;
volatile int g_pcc_on;
volatile int g_pedal_mapped;
volatile float g_overspeed_cap = 5;
volatile float g_overspeed_pct = 0;
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
int game_letter(void *, char) { return 1; }   // the dashboard letter's writer (core/display.cpp)
static float sim_add_thr;   // the added throttle the truck model sees (what Creep posted the tick before, as a floor on the merged throttle)
int input_held(int) { return 0; }
void play_cue(int) {}
static float g_adaptive_v = 3;   // the driving mode's cvar (3 = Normal, > 3.01 Eco, <= 1.67 Power)
float game_adaptive() { return g_adaptive_v; }
void *game_cvar(const char *) { return 0; }
void game_cvar_set(void *, float) {}
static float g_mass = 18000;
float game_mass() { return g_mass; }
float game_mass_own() { return g_mass; }
int game_vehicle_is_car() { return 0; }
float game_car_speed_limit_ms() { return NAN; }
float wheel_radius() { return 0.5f; }
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
static int n_checks, n_failed;
static const float CR_REST_V_SIM = 0.15f;   // creep_tick's `rest` (|v| < 0.15 m/s)
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
uintptr_t game_scan(const char *) { return 0; }
int game_patch_tail(uintptr_t, void *) { return 0; }
int game_patch_clutch_floor(uintptr_t, clutch_floor *) { return 0; }
int game_patch_retarder(uintptr_t, const retarder_scale *) { return 0; }
int game_patch_cf0_if(uintptr_t, int, const unsigned char *) { return 0; }
int game_patch_call(uintptr_t, void *) { return 0; }
int game_engage_init() { return 0; }
float game_ebrake_level(void *) { return -1; }   // (the fuel learner in vehicle.cpp asks; this sim has no engine brake stages)
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
int low_owner_on() { return 0; }          // the low-speed owner (lowspeed.cpp) is not part of this sim
float low_top_kmh() { return 0; }
float wish_launch_share(float raw) { return raw; }   // the torque pedal (modules/pedal.cpp, core/wish.cpp) is not in this sim: the launch reads the raw pedal
// The pedal's map (core/wish.cpp map_of, [pedal] curve_*): s = (1 + c) p^k / (p^k + c), k 1.47, Eco 0.365 / Normal 0.295 / Power 0.245. The same formula and values as the module's defaults (core/wish.cpp links in sim_lowspeed.cpp,
// which asserts the crawl against the real function); this sim stands in for it so the crawl map can be swept without the vehicle model.
float wish_map(float p, int md) { const float c = md == MODE_ECO ? 0.365f : md == MODE_POWER ? 0.245f : 0.295f; p = p < 0 ? 0 : p > 1 ? 1 : p; const float pk = powf(p, 1.47f); return (1.0f + c) * pk / (pk + c); }
void cruise_info_get(cruise_info *o) { memset(o, 0, sizeof *o); }

#include "modules/creep.cpp"
#include "core/display.cpp"
#include "core/rsc.cpp"
#include "core/actuate.cpp"
#include "core/vehicle.cpp"
#include "core/bands.cpp"

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
    g_heavy_integ = 0;   // review 30-10: a fresh episode (the sim's stand-in for ck_reset)
    g_user_on = 1;       // the creep session (the crawl scenarios of this sim)
    g_hbrake = 0;
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
// A flat pedal launches as before (the wish pedal's shape never holds a flat pedal back: wish_launch_share(1) == 1, sim_pedal's L check):
// the clutch closes in the same time as before the launch read the shaped share. 30 t flat, pedal 1.0, measured the same way below.
static const float FLAT_LOCK_SOFT_S = 1.57f, FLAT_LOCK_HARD_S = 1.72f;
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
            li.dt = 1.0f / 60; li.speed = v; li.thr = thr; li.wish = thr; li.brake = 0; li.rpm = rpm; li.idle = IDLE;
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
    printf("   while launching: rpm max %.0f (%.0f over the aim), launch ended %s, clutch closed at %.1f s, speed then %.1f km/h, jerk max %.0f m/s3\n", rpm_max, over_max, g_ln ? "NOT" : g_ln_why, lock_t, v_lock * 3.6f, jerk_max);
    if (thr_end >= 1.0f && grade_pct == 0 && mass == 30000) {
        float base = k >= 2.0f ? FLAT_LOCK_SOFT_S : FLAT_LOCK_HARD_S;
        check(fabsf(lock_t - base) <= 0.02f * base, "%s: the clutch closes in %.2f s, %.2f s expected (a flat pedal launches as before)", name, lock_t, base);
    }
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

// ---- LS2: the idle creep (pedals free, the creep setting) is the bottom of the pull-away, not a mode
// With both pedals free the truck creeps at creep_kmh through the slipping clutch (the brake lowers it); a pressed pedal ends the
// creep and the launch takes over (a force, the torque pedal's demand). There is no crawl speed map any more: no max_pedal ceiling,
// no manoeuvre_kmh, no "leave the crawl with a flat pedal".
static void idle_creep_rows()
{
    printf("== IDLE CREEP: pedals free and the creep setting: creeps at creep_kmh on the flat, the brake lowers it, a pedal hands over to the launch (no speed map)\n");
    // the plant holds the creep speed with the pedals free (a light 9 t truck in its creep gear)
    {
        reset(0, 0, 1, 0, 9000);
        double t0 = t_now, sum = 0; int n = 0;
        while (t_now - t0 < 20) {
            step(1 / 60.0);
            if (t_now - t0 > 15) { sum += fabsf(M.v) * 3.6; n++; }
        }
        float v = (float)(sum / n);
        printf("   light 9 t, pedals free: %.2f km/h after settling (creep_kmh %.1f)\n", v, p_creep);
        check(g_on, "idle creep: the creep never ran");
        check(fabsf(v - p_creep) < 1.0f, "idle creep: settles at %.2f km/h, creep_kmh is %.1f", v, p_creep);
    }
    // the brake pedal lowers the creep (the pedal itself never raises it: no speed map)
    {
        reset(0, 0, 1, 0);
        double t0 = t_now;
        while (t_now - t0 < 12) step(1 / 60.0);
        g_brake = 0.2f;
        for (int i = 0; i < 60 * 3; i++) step(1 / 60.0);
        float v = fabsf(M.v) * 3.6f;
        printf("   brake 0.2 (under p_brake %.2f): %.2f km/h\n", p_brake, v);
        check(v < p_creep - 0.5f, "idle creep: the brake does not lower the creep (%.2f km/h)", v);
    }
    // a pedal ends the creep: the launch takes over (no crawl speed map any more, so no max_pedal start gate)
    {
        reset(0, 0, 1, 0);
        double t0 = t_now;
        while (t_now - t0 < 10) step(1 / 60.0);
        check(g_on, "idle creep: the creep never came on before the pedal");
        g_thr = 0.3f;
        step(1 / 60.0);
        printf("   pedal 0.3 at %.1f km/h: the creep %s\n", fabsf(M.v) * 3.6, g_on ? "still on" : "handed over to the launch");
        check(!g_on, "idle creep: a pedal of 0.3 does not end the creep (the crawl was a mode: the pedal asked a crawl speed)");
    }
    // the toggle (the gearbox-mode key): flips the pedals-free setting (hold <-> creep) for the session
    {
        reset(0, 0, 1, 0);
        double t0 = t_now;
        while (t_now - t0 < 10) step(1 / 60.0);
        check(g_on, "idle creep: the creep never came on (creep session)");
        creep_set_switch(0);   // the toggle: creep -> hold
        g_thr = 0; g_brake = 0;
        for (int i = 0; i < 3; i++) step(1 / 60.0);
        printf("   the toggle flips creep -> hold: the creep %s\n", g_on ? "still on" : "off (the hold setting holds the truck)");
        check(!g_on, "idle creep: the toggle does not flip creep -> hold (the creep stays on)");
        creep_set_switch(1);   // and back: hold -> creep
        for (int i = 0; i < 60 * 2; i++) step(1 / 60.0);
        check(g_on, "idle creep: the toggle does not flip hold -> creep back");
        creep_set_switch(-1);
    }
}

// ======================================================================= the hold setting (WP-FX1 fix 3, ride 04-11)
// pedals_free = hold: both pedals free below the creep speed. The owner: "when I release the gas below ~5 km/h it stops far too hard, I can't roll out; just roll out, and only at a standstill the hold".
// The plant: a truck along its forward axis (v signed, - = backwards) with the level road's resistance of the vehicle model, the grade's pull, and a service brake of 8 m/s2 per request behind
// a 0.15 s lag, standing still when the brake (and the road's resistance) outholds the pull. The request is creep's hold request at every frame (50 Hz). 91ff980 has no creep_hold_req: its request
// (vehicle_brake_hold_req, the one that holds a truck at REST on this grade) was asked at every speed under the creep speed.
#ifndef CREEP_HOLD_REQ
static float creep_hold_req(float, float pitch_pct, float fallback) { return vehicle_brake_hold_req(pitch_pct, fallback); }
#endif
struct holdres {
    float peak_roll;      // m/s2, the largest deceleration along the way the truck moved while it rolled at 0.15 m/s or more
    float peak_last;      // m/s2, the same over the last 1 km/h
    float t_stop;         // s, when the truck first stood (|v| < 0.01 m/s), -1 = it did not within the run
    float drift_m;        // m, how far the truck moved after it first stood, to the end of the run
    float back_m;         // m, the furthest it went against the way it was moving before it stood (a roll back or away)
    float v_end_kmh;
    float req_max, free_peak;   // the largest request; the largest deceleration of the same run with no brake at all (the road's own)
};
static holdres hold_run(float mass_t, float pitch_pct, float v0_kmh, int reverse, int brake_on = 1)
{
    holdres r; memset(&r, 0, sizeof r); r.t_stop = -1;
    memset(&T, 0, sizeof T);
    V = vehicle(); V.mass = mass_t * 1000; V.mass_ok = 1; V.crr = 0.006f; V.cda = 6.0f; V.pitch_pct = pitch_pct; V.grade_deg = atanf(pitch_pct / 100.0f) * 57.29578f;
    g_brake_learn = 1; T.trailer = 0; vehicle_brake_prime(0, 8.0f * V.mass, 0.02f, 8);
    const float G = 8.0f, lag = 0.15f, dt = 0.01f, s_ = sinf(atanf(pitch_pct / 100.0f));
    float v = (reverse ? -1.0f : 1.0f) * v0_kmh / 3.6f, y = 0, yl = 0, x = 0, x_stop = 0, prev_abs = fabsf(v);
    const float way = v < 0 ? -1.0f : 1.0f;   // the way it moves at the start: x is counted along it
    bool stood = false;
    for (int i = 0; i < 3000; i++) {   // 30 s
        const double t = i * dt;
        float req = 0;
        if (brake_on && i % 2 == 0) { req = creep_hold_req(v, pitch_pct, 0.2f); r.req_max = fmaxf(r.req_max, req); y = req; }
        yl += (y - yl) * dt / (lag + dt);
        const float b = G * yl, F = -9.81f * s_, nat = vehicle_resist_grade(fabsf(v), 0.0f) / V.mass, fr = nat + b;
        float vn;
        if (fabsf(v) < 1e-4f) vn = fabsf(F) <= fr ? 0.0f : v + (F - (F > 0 ? 1.0f : -1.0f) * fr) * dt;
        else { vn = v + (F - (v > 0 ? 1.0f : -1.0f) * fr) * dt; if (v * vn < 0 && fabsf(F) <= fr) vn = 0; }
        // the deceleration along the way the truck moves, measured on the speed
        const float dec = -(fabsf(vn) - fabsf(v)) / dt;
        if (!stood) {
            if (fabsf(v) >= CR_REST_V_SIM) r.peak_roll = fmaxf(r.peak_roll, dec);
            if (fabsf(v) < 1.0f / 3.6f && fabsf(v) > 0.01f) r.peak_last = fmaxf(r.peak_last, dec);
        }
        v = vn; x += v * dt;
        if (!stood && fabsf(v) < 0.01f) { stood = true; x_stop = x; r.t_stop = (float)t; }
        if (stood) { r.drift_m = fabsf(x - x_stop); r.back_m = fmaxf(r.back_m, way * (x_stop - x)); }
        prev_abs = fabsf(v);
    }
    (void)prev_abs;
    r.v_end_kmh = v * 3.6f;
    return r;
}
static void hold_rows()
{
    printf("== the hold setting: both pedals free under the creep speed. A rolling truck is not stopped (only a grade that would run it away is held), a truck at rest is held\n");
    printf("   per row: peak deceleration while rolling (the no-brake run's own), over the last 1 km/h, stop time, drift after it stood / roll-back, the largest request\n");
    struct { const char *name; float mass_t, pitch, kmh; int rev; } cs[] = {
        { "16.5 t reversing from 4.7 km/h, flat (589.6 s of the ride)", 16.5f, 0, 4.7f, 1 },
        { "16.5 t reversing from 3.5 km/h, flat (622.4 s)", 16.5f, 0, 3.5f, 1 },
        { "41.9 t forward from 3.6 km/h, flat (987.0 s)", 41.9f, 0, 3.6f, 0 },
        { "16.5 t forward from 4.5 km/h, flat", 16.5f, 0, 4.5f, 0 },
        { "16.5 t forward from 4.5 km/h, +2 % (nose up)", 16.5f, 2, 4.5f, 0 },
        { "16.5 t forward from 4.5 km/h, -2 % (nose down: it runs away)", 16.5f, -2, 4.5f, 0 },
        { "41.9 t forward from 3.6 km/h, +10 %", 41.9f, 10, 3.6f, 0 },
        { "41.9 t forward from 3.6 km/h, -10 %", 41.9f, -10, 3.6f, 0 },
        { "16.5 t reversing from 3.5 km/h, +2 % (it runs down backwards)", 16.5f, 2, 3.5f, 1 },
        { "16.5 t reversing from 3.5 km/h, -2 % (up backwards)", 16.5f, -2, 3.5f, 1 },
        { "41.9 t reversing from 3.0 km/h, +10 %", 41.9f, 10, 3.0f, 1 },
        { "41.9 t reversing from 3.0 km/h, -10 %", 41.9f, -10, 3.0f, 1 },
    };
    for (auto &c : cs) {
        const holdres h = hold_run(c.mass_t, c.pitch, c.kmh, c.rev), f = hold_run(c.mass_t, c.pitch, c.kmh, c.rev, 0);
        // the rolling part: where the slope does not run the truck away (flat, up, backwards up) the hold asks nothing while it rolls: its peak is the road's own (+ 0.05); where it does, the stop is
        // eased at most the stop assist's 0.5 m/s2 beyond what the slope needs
        const bool runs = (c.pitch < 0 && !c.rev) || (c.pitch > 0 && c.rev);
        // (1.5 x the taper's 0.5 m/s2: the request is sized by the start value of the brake, which for the 41.9 t plant is 12 % weaker than the plant's, and the brake lags)
        check(h.peak_roll <= f.peak_roll + (runs ? 0.75f : 0.05f), "%s: it was braked while rolling at %.2f m/s2 (the road's own %.2f)", c.name, h.peak_roll, f.peak_roll);
        // the last 1 km/h is where the hold comes in: at most 1.0 m/s2 on top of the road's own (the old request put 1.2 - 1.8 m/s2 on the whole roll)
        check(h.peak_last <= f.peak_last + 1.0f, "%s: %.2f m/s2 over the last 1 km/h (the road's own %.2f)", c.name, h.peak_last, f.peak_last);
        check(h.t_stop > 0 && h.drift_m <= 0.02f && h.back_m <= 0.05f, "%s: it must stand and be held (stopped at %.1f s, moved %.3f m afterwards, %.3f m against its way)", c.name, h.t_stop, h.drift_m, h.back_m);
        printf("   %-66s peak rolling %.2f (road's own %.2f), last 1 km/h %.2f (%.2f), stop at %4.1f s, drift %.3f m, back %.3f m, request max %.2f\n", c.name, h.peak_roll, f.peak_roll, h.peak_last, f.peak_last, h.t_stop, h.drift_m, h.back_m, h.req_max);
    }
    {   // continuous in the speed (no step where the setting begins, at creep's rest speed or at the end), on the flat and on both slopes
        float worst = 0; const float pitches[] = { 0, 2, -2, 10, -10 };
        for (float pitch : pitches) for (int dir = -1; dir <= 1; dir += 2) {
            V = vehicle(); V.mass = 30000; V.mass_ok = 1; V.crr = 0.006f; V.cda = 6.0f; V.pitch_pct = pitch; g_brake_learn = 1; vehicle_brake_prime(0, 8.0f * V.mass, 0.02f, 8);
            float prev = creep_hold_req(0, pitch, 0.2f);
            for (float k = 0.01f; k <= p_creep + 0.5f; k += 0.01f) { float q = creep_hold_req(dir * k / 3.6f, pitch, 0.2f); worst = fmaxf(worst, fabsf(q - prev)); prev = q; }
        }
        check(worst <= 0.03f, "the hold request steps by %.3f in 0.01 km/h", worst);
        printf("   the request against the speed (flat, +-2 %%, +-10 %%, both ways, 0.01 km/h steps up to creep_kmh + 0.5): the largest step %.4f %s\n", worst, worst <= 0.03f ? "ok" : "FAIL");
    }
    g_brake_learn = 0;
}

int main(int argc, char **argv)
{
    verbose = argc > 1;
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
    {   // launch_drag_rpm must stay at or under the light-pedal aim: a high-revving engine (idle 800, plateau 3000, span 2200)
        // would put the drag threshold at idle + 220 = 1020 rpm, above the 1000 rpm light-pedal aim, so the closing phase keeps
        // forcing the clutch open. The margin is capped at half the light-pedal aim's margin over idle.
        V.eng_ok = 1; V.plateau = 3000;
        float drag = launch_drag_rpm(800, 0), aim = launch_aim(0.1f, 800, 0);
        printf("== launch_drag_rpm cap: high-revving engine (idle 800, plateau 3000): drag %.0f, light-pedal aim %.0f\n", drag, aim);
        if (!(drag <= aim)) { printf("   FAIL: drag threshold above the light-pedal aim\n"); exit(1); }
    }
    {   // launch control (hidden): arms after 0.7 s of brake + flat pedal at a standstill, holds the revs, launches when the brake is released
        printf("== LAUNCH CONTROL state machine\n");
        memset(&T, 0, sizeof T); T.fwd_gears = 12; T.diff = DIFF; T.rpm_limit = 1900;
        for (int i = 0; i < 12; i++) T.ratio[i] = RATIO[i];
        g_ln = 0; g_ln_done = 0; g_lv.lcx = 0; g_lcx_cap = 1; p_launch = 1;
        double t = 0; int armed_at = -1, go_at = -1; float cap_min = 1; int bad2 = 0;
        for (int n = 0; n < 60 * 6; n++, t += 1.0 / 60) {
            launch_in li = {};
            li.dt = 1.0f / 60; li.speed = 0; li.thr = 1.0f; li.wish = 1.0f; li.brake = t < 3.0 ? 0.8f : 0.0f; li.rpm = t < 1.5 ? 1200.0f : 1700.0f;
            li.idle = IDLE; li.rpm_per_ms = RATIO[2] * DIFF / RW * 9.5493f; li.gear = 3; li.handover = -1;
            t_now = t; launch_tick(li);
            if (g_lv.lcx == 1 && armed_at < 0) armed_at = n;
            if (g_lv.lcx == 2 && go_at < 0) go_at = n;
            if (g_lv.lcx == 1) cap_min = fminf(cap_min, g_lcx_cap);
        }
        bad2 |= !(armed_at > 40 && armed_at < 50) || go_at < 170 || go_at > 190 || cap_min > 0.45f || g_lcx_cap != 1;
        printf("   armed at frame %d (expect ~42), go at frame %d (expect ~180), cap fell to %.2f, cap after go %.2f, launch control %s\n", armed_at, go_at, cap_min, g_lcx_cap, bad2 ? "FAIL" : "ok");
        if (bad2) exit(1);
        g_ln = 0; g_lv.lcx = 0; g_lcx_cap = 1;
    }
    hold_rows();
    idle_creep_rows();
    printf("\n%d checks, %d failed\n", n_checks, n_failed);
    return n_failed ? 1 : 0;
}
