// Offline test of the wish pedal (modules/pedal.cpp is included, so its statics are reachable; TrueShift and the vehicle model are
// linked for real: the start gear is TrueShift's physics pick, the forces are the vehicle model's). No plant: the truck stands at a
// given speed in a given gear with a closed clutch, the script sets the pedal and calls the stage tick by tick (60 Hz), as Cruise does.
// What it proves: the same pedal gives the same force in every gear, the rise limit and the release, the kick, the blend, every
// pass-through, wish 0 = the pedal unchanged, and never a NaN. What it cannot: how it feels, and Cruise's call site (see the report).
// Build+run: tools/sim/run_pedal.sh [-v]
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
volatile float g_decel_need;
volatile float g_clutch_ours = __builtin_nanf("");
float g_air_from_truck;   // [core] air_from_truck (core/actuate.cpp, not linked here)
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
void logline(const char *fmt, ...)
{
    char b[300]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    if (verbose) printf("%7.2f  %s\n", t_now, b);
}

// ---- the truck: a 12-speed (the engine of the TrueShift sim)
static const float IDLE = 650, TORQUE = 2700, LIMIT = 1900, DIFF = 3.08f;
static float RADIUS = 0.538f;
static float RATIO[16];
static const float CURVE[8][2] = { {0,0},{700,0.60f},{900,0.97f},{1250,1},{1500,0.9f},{1700,0.72f},{1900,0.45f},{2100,0} };
static float RANGES[4][2] = { {1500,1900},{750,1450},{900,1250},{1200,1600} };   // the low-gear window's top is moved by one test
static float g_mass = 17000;
static float g_adapt = 3;   // the game's adaptive cvar: 4 Eco, 3 Normal, 1 Power, 0 M
static creep_info CI;
static cruise_info CU;
static int g_gear;

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
float game_grade_deg(void *) { return 0; }
float game_adaptive() { return g_adapt; }
void *game_gearbox() { return (void *)&g_gear; }
int gb_gear(void *) { return g_gear; }
int gb_target(void *) { return g_gear; }
int gb_state(void *) { return 0; }
int gb_engage(void *, int) { return 0; }
int game_engage_init() { return 1; }
void gb_set_target(void *, int) {}
void gb_force(void *) {}
char *gb_inner(void *) { return NULL; }
float gb_thr_eff(void *) { return 0; }
int game_letter(void *, char) { return 1; }
int game_ebrake_downshift(void *) { return 0; }
float game_ebrake_level(void *) { return -1; }
int game_patch_call(uintptr_t, void *) { return 0; }
int game_patch_tail(uintptr_t, void *) { return 0; }
int game_patch_stop(uintptr_t, float (*)(float, float, float)) { return 0; }
int low_owner_on() { return 0; }   // the low-speed owner (lowspeed.cpp) is not part of this sim
float act_request() { return 0; }   // nor the actuator
int low_owner_wanted() { return 0; }
int low_start_gear() { return 0; }
int low_launch_hold() { return 0; }
int low_handed() { return 0; }
float low_stop_rpm(float idle, float, float, int) { return 0.8f * idle; }
int mem_readable(const void *, size_t) { return 0; }
char *game_actor_safe() { return NULL; }
int route_ok() { return 0; }
float route_grade(float, float) { return NAN; }
float route_max_grade(float, float) { return NAN; }
float route_min_grade(float, float) { return NAN; }
float route_crest_m(float) { return -1; }
float route_junction_m() { return -1; }
float route_curve_speed(float, float, float) { return NAN; }
void creep_info_get(creep_info *o) { *o = CI; }
void cruise_info_get(cruise_info *o) { *o = CU; }
float cruise_wish_force(float) { return -1; }   // [cruise] force_mode: the cruise (modules/cruise.cpp) is not in this sim

#include "modules/pedal.cpp"
extern module m_trueshift;

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
static void set_param(module &m, const char *n, float v)
{
    for (int i = 0; i < m.n_params; i++) if (!strcmp(m.params[i].key, n)) *m.params[i].val = v;
}
static float g_ref_now = 0;   // ref_now for the sections A ... F: 0 = the peak power as the reference (their numbers as before), G runs them again with 1
static void params_default()
{
    for (module *m : { &m_trueshift, &m_pedal })
        for (int i = 0; i < m->n_params; i++) *m->params[i].val = m->params[i].def;
    set_param(m_trueshift, "idle_rpm", IDLE);
    p_wish = 1;
    p_ref_kwt = 0;   // the exponents as the ini has them; the per-truck scaling has its own test (D2)
    p_ref_now = g_ref_now;
    p_tq_share = 0;   // these sections test the wish itself; torque_share (the hold in a gear) has its own sim (sim_pedalmix)
}

// The truck at a speed in a gear, clutch closed, locked. mode: MODE_*.
static void world(int gear, double kmh, int mode = MODE_NORMAL, float mass = 17000)
{
    memset(&T, 0, sizeof T);
    T.fwd_gears = 12; T.diff = DIFF; T.wheels = 4; T.engine_on = 1;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
    RADIUS = 0.538f;
    g_mass = mass; g_gear = gear;
    g_adapt = mode == MODE_ECO ? 4.0f : mode == MODE_POWER ? 1.0f : 3.0f;
    memset(&CI, 0, sizeof CI); memset(&CU, 0, sizeof CU);
    g_crawl_on = 0; g_lcx = 0; g_clutch_ours = NAN;
    p_wish = 1;
    vehicle_reset(RESET_HARD);
    m_pedal.reset(RESET_HARD, "sim: new scenario");
    vehicle_mass_stale();
    T.gear = gear; T.speed = (float)(kmh / 3.6);
    for (int i = 0; i < 3; i++) { t_now += 1.0 / 60; vehicle_frame(); }
    T.rpm = vehicle_rpm_at(gear, T.speed);
}

struct res { int on; float out; };
static res step(float p, float dt = 1.0f / 60, int other = 0)
{
    float o = -7;
    int r = pedal_wish(p, dt, other, &o);
    return { r, o };
}
// the pedal held at p long enough for the rise limit to have run out; the result of the last tick
static res hold(float p, float secs = 3)
{
    res r = { 0, p };
    for (int i = 0; i < (int)(secs * 60); i++) r = step(p);
    return r;
}
// A fresh state, then the pedal held: the steady throttle of this pedal here
static float steady(float p)
{
    m_pedal.reset(RESET_SOFT, "sim");
    return hold(p).out;
}
static float force(int g, float gas) { return gas * vehicle_force_at(g, T.speed, 1.0f); }

// The reference, worked out here again from the engine and the vehicle model (not from the module's code): K = the force the wish asks
// at pedal 1 against what the gear gives at full throttle (gas = q * K, clamped)
static float k_ref(int g, float v)
{
    float pref = 0;
    for (float rpm = 100; rpm <= LIMIT; rpm += 1) pref = fmaxf(pref, TORQUE * curve_factor(&V.eng, rpm) * rpm * (float)M_PI / 30);
    int sg = ts_start_gear(0.3f);
    float fcap = vehicle_force_at(sg, vehicle_kmh_at(sg, V.plateau) / 3.6f, 1);
    return fminf(pref * V.eta / v, fcap) / vehicle_force_at(g, v, 1);
}

static const char *mode_text(int m) { return m == MODE_ECO ? "Eco" : m == MODE_POWER ? "Power" : "Normal"; }

// ---------------------------------------------------------------- A: the same pedal gives the same force in every gear
static void gears_table()
{
    printf("== A the same pedal, the same force in gears 8, 9, 10 (45 km/h, 17 t, Normal)\n");
    world(9, 45);
    const int G3[3] = { 8, 9, 10 };
    float old_f[3], bl = p_blend;
    printf("   pedal | old gas (= pedal)      | wish gas               | wish force kN          | old force kN\n");
    printf("         |    g8     g9    g10    |    g8     g9    g10    |    g8     g9    g10    |    g8     g9    g10\n");
    float worst = 0;
    int sat_p[3] = { -1, -1, -1 };
    for (int i = 1; i <= 10; i++) {
        float p = i / 10.0f, gas[3], f[3];
        for (int k = 0; k < 3; k++) {
            world(G3[k], 45);
            gas[k] = steady(p);
            f[k] = force(G3[k], gas[k]);
            old_f[k] = force(G3[k], p);
            if (gas[k] >= 0.999f && sat_p[k] < 0) sat_p[k] = i;
        }
        printf("   %4.2f  |  %5.3f  %5.3f  %5.3f  |  %5.3f  %5.3f  %5.3f  |  %6.2f %6.2f %6.2f  |  %6.2f %6.2f %6.2f\n", p, p, p, p, gas[0], gas[1], gas[2],
               f[0] / 1000, f[1] / 1000, f[2] / 1000, old_f[0] / 1000, old_f[1] / 1000, old_f[2] / 1000);
        if (p < bl && gas[0] < 0.999f && gas[1] < 0.999f && gas[2] < 0.999f) {
            float hi = fmaxf(f[0], fmaxf(f[1], f[2])), lo = fminf(f[0], fminf(f[1], f[2]));
            worst = fmaxf(worst, (hi - lo) / hi);
            check((hi - lo) / hi <= 0.05f, "A pedal %.2f: the forces of gears 8/9/10 differ by %.1f %%", p, 100 * (hi - lo) / hi);
        }
    }
    for (int k = 0; k < 3; k++) {
        if (sat_p[k] < 0) printf("   gear %d: never at gas 1 up to pedal 1.0 (the blend reaches 1 only at the flat pedal)\n", G3[k]);
        else printf("   gear %d: gas reaches 1 at pedal %.1f (the gear cannot deliver more)\n", G3[k], sat_p[k] / 10.0f);
    }
    // the unsaturated pedals, finely: where every gear can deliver
    float p_top = 0; worst = 0;
    for (int i = 1; i < 85; i++) {
        float p = i / 100.0f, gas[3], f[3];
        bool sat = false;
        for (int k = 0; k < 3; k++) { world(G3[k], 45); gas[k] = steady(p); f[k] = force(G3[k], gas[k]); sat |= gas[k] >= 0.999f; }
        if (sat) continue;
        p_top = p;
        float hi = fmaxf(f[0], fmaxf(f[1], f[2])), lo = fminf(f[0], fminf(f[1], f[2]));
        worst = fmaxf(worst, (hi - lo) / hi);
        check((hi - lo) / hi <= 0.05f, "A pedal %.2f: the forces differ by %.1f %%", p, 100 * (hi - lo) / hi);
    }
    printf("   below the blend and while no gear saturates (pedal up to %.2f): the forces differ by at most %.2f %% across gears 8/9/10\n", p_top, 100 * worst);
    check(p_top >= 0.2f, "A: no pedal range where the three gears can all deliver (up to %.2f)", p_top);
}

// ---------------------------------------------------------------- A2: the force cap at low speed
// With the default engine the cap meets the power line just under the low-speed range, so it never binds above it. A lower window top
// (the game's range for the low gears) moves the range down and the cap shows: the force is q * F_cap, the same in every gear.
static void cap_checks()
{
    printf("== A2 the force cap: at low speed the wish is a force of q * F_cap, not q * power / speed\n");
    RANGES[R_LOW][1] = 1000;
    const int G2[2] = { 3, 4 };
    for (int kmh : { 10, 11, 12 }) {
        float f[2], fcap = 0, pw = 0;
        for (int k = 0; k < 2; k++) {
            world(G2[k], kmh);
            int sg = ts_start_gear(0.3f);
            fcap = vehicle_force_at(sg, vehicle_kmh_at(sg, V.plateau) / 3.6f, 1);
            f[k] = force(G2[k], steady(0.5f));
            pw = 0.5f * 2700 * 1.0f * 1250 * (float)M_PI / 30 * V.eta / T.speed;   // the power line at pedal 0.5 (peak at the plateau end)
            if (kmh == 10 && k == 0) printf("   gear %d at %d km/h: gate %.1f km/h, F_cap %.1f kN, the power line would ask %.1f kN at pedal 1; the wish at pedal 0.5 gives %.1f kN\n", G2[k], kmh, vehicle_kmh_at(sg, V.eng.range[R_LOW][1]), fcap / 1000, 2 * pw / 1000, f[k] / 1000);
        }
        check(fabsf(f[0] - 0.5f * fcap) < 0.005f * fcap && fabsf(f[1] - 0.5f * fcap) < 0.005f * fcap, "A2 %d km/h: forces %.0f / %.0f N, half the cap is %.0f N", kmh, f[0], f[1], 0.5f * fcap);
    }
    RANGES[R_LOW][1] = 1450;
}

// ---------------------------------------------------------------- B: 0 -> 0, 1 -> 1, monotone, continuous at blend_from
static void shape_checks()
{
    printf("== B pedal 0 -> gas 0, pedal 1 -> gas 1, monotone in the pedal, continuous at blend_from (blend_from 0.85, then 1 = no blend: the wish all the way)\n");
    float worst_dip = 0; int dips = 0;
    const float blend0 = p_blend;
    for (float bf : { 0.85f, 1.0f }) {
    p_blend = bf;
    for (int md : { MODE_ECO, MODE_NORMAL, MODE_POWER })
        for (int g : { 7, 8, 9, 10 }) {
            world(g, 45, md);
            float g0 = steady(0), g1 = steady(1);
            check(g0 == 0, "B %s gear %d: pedal 0 gives gas %g", mode_text(md), g, g0);
            check(g1 == 1, "B %s gear %d: pedal 1 gives gas %g", mode_text(md), g, g1);
            float prev = 0;
            for (int i = 1; i <= 100; i++) {
                float p = i / 100.0f, gas = steady(p);
                check(gas + 1e-6f >= prev, "B %s gear %d: gas falls from %.4f to %.4f at pedal %.2f", mode_text(md), g, prev, gas, p);
                check(gas >= 0 && gas <= 1, "B %s gear %d: gas %g out of range at pedal %.2f", mode_text(md), g, gas, p);
                // what the blend would do taken literally (wish gas fades to the plain pedal even where it is above it)
                if (p > p_blend && p < 1 && S.w > p) { float lit = S.w + (p - S.w) * (p - p_blend) / (1 - p_blend); if (S.w - lit > worst_dip) worst_dip = S.w - lit; dips++; }
                prev = gas;
            }
            float a = steady(p_blend - 1e-4f), b = steady(p_blend), c = steady(fminf(1.0f, p_blend + 1e-4f));
            check(fabsf(b - a) < 1e-3f && fabsf(c - b) < 1e-3f, "B %s gear %d: blend_from steps %g, %g", mode_text(md), g, b - a, c - b);
            // the blend: wish gas to the plain pedal, linearly, reaching the pedal at a flat pedal
            for (float p : { 0.9f, 0.95f, 0.99f }) {
                float gas = steady(p), w = S.w;
                if (verbose) printf("   %s g%d p%.2f wish gas %.4f gas %.4f K %.3f\n", mode_text(md), g, p, w, gas, k_ref(g, T.speed));
                float want = p_blend < 1 ? w + fmaxf(0.0f, p - w) * (p - p_blend) / (1 - p_blend) : w;   // (blend_from 1: the wish itself)
                check(fabsf(gas - want) < 1e-5f, "B %s gear %d: pedal %.2f gives %.4f, the blend of wish %.4f and pedal says %.4f", mode_text(md), g, p, gas, w, want);
                check(w >= p || ((p_blend < 1 ? gas > w : gas == w) && gas < p + 1e-6f), "B %s gear %d: pedal %.2f: gas %.4f is not between the wish %.4f and the pedal", mode_text(md), g, p, gas, w);
            }
            float e = steady(0.9999f);
            check(e > 0.999f && e <= 1, "B %s gear %d: gas %g just under a flat pedal", mode_text(md), g, e);
        }
    }
    p_blend = blend0;
    // the blend toward the pedal acts where the wish gas is under the pedal. In this truck the wish gas is never under the pedal in the
    // blend range with the default exponents (a gear gives at most the peak power the wish is sized on), so a flat Eco curve makes the case
    float e0 = p_exp_eco; p_exp_eco = 1.6f; p_blend = 0.85f;
    for (float p : { 0.88f, 0.9f, 0.95f }) {
        world(8, 45, MODE_ECO);
        float gas = steady(p), w = S.w, want = w + (p - w) * (p - p_blend) / (1 - p_blend);
        check(w < p && fabsf(gas - want) < 1e-5f && gas > w && gas < p, "B blend (Eco exponent 1.6, gear 8): pedal %.2f gives %.4f, wish %.4f, the blend says %.4f", p, gas, w, want);
    }
    p_exp_eco = e0; p_blend = blend0;
    printf("   where a gear needs more gas than the pedal in the blend range the wish is kept (read literally the blend would dip by up to %.3f below it, %d pedal points)\n", worst_dip, dips);
}

// ---------------------------------------------------------------- C: the rise limit, the release, the kickdown
static void rise_checks()
{
    printf("== C rise limit, release, kickdown\n");
    for (int md : { MODE_ECO, MODE_NORMAL, MODE_POWER }) {
        float rate = md == MODE_ECO ? p_rise_eco : md == MODE_POWER ? p_rise_pw : p_rise_norm;
        for (int g : { 8, 9, 10 }) {
            world(g, 45, md);
            float kick = p_kick; p_kick = 0;   // the kick has its own test
            float K = k_ref(g, T.speed), top = steady(0.6f), prev = 0, worst = 0, t_full = -1;
            m_pedal.reset(RESET_SOFT, "sim");
            hold(0, 0.5f);
            for (int i = 1; i <= 240; i++) {
                float gas = step(0.6f).out, d = gas - prev;
                worst = fmaxf(worst, d * 60);
                if (t_full < 0 && gas >= top - 1e-4f) t_full = i / 60.0f;
                prev = gas;
            }
            // gas = q * K: it may rise K times as fast as the wish, never faster
            float lim = rate * fmaxf(1.0f, K);
            check(worst <= lim * 1.001f + 1e-4f, "C %s gear %d: the gas rises %.3f /s, limit %.3f /s (rate %.2f, K %.2f)", mode_text(md), g, worst, lim, rate, K);
            float expo = md == MODE_ECO ? p_exp_eco : md == MODE_POWER ? p_exp_pw : p_exp_norm;
            float want_t = powf(0.6f, expo) / rate;
            check(t_full > 0 && t_full <= want_t + 0.1f && (K > 1 || t_full >= want_t - 0.05f), "C %s gear %d: the wish took %.2f s, %.2f s expected", mode_text(md), g, t_full, want_t);
            // release: at once, from the held wish to the steady gas of the lower pedal
            hold(0.6f);
            float down = step(0.2f).out, ref = steady(0.2f);
            check(fabsf(down - ref) < 1e-5f, "C %s gear %d: release 0.6 -> 0.2 gives %.4f in the first tick, steady %.4f", mode_text(md), g, down, ref);
            hold(0.4f);
            float zero = step(0).out;
            check(zero == 0, "C %s gear %d: release to 0 gives %g", mode_text(md), g, zero);
            // kickdown: never held back from blend_from up (blend_from 1: only a flat pedal), and a flat pedal is 1 at once
            m_pedal.reset(RESET_SOFT, "sim");
            hold(0.1f, 0.5f);
            float k90 = step(0.9f).out, s90 = steady(0.9f);
            if (p_blend < 1) check(fabsf(k90 - s90) < 1e-5f, "C %s gear %d: 0.1 -> 0.9 in one tick gives %.4f, steady %.4f", mode_text(md), g, k90, s90);
            else check(k90 < s90 - 0.05f, "C %s gear %d: blend_from 1: 0.1 -> 0.9 in one tick is held by the rise limit, gives %.4f, steady %.4f", mode_text(md), g, k90, s90);
            m_pedal.reset(RESET_SOFT, "sim");
            hold(0.1f, 0.5f);
            check(step(1.0f).out == 1.0f, "C %s gear %d: 0.1 -> 1.0 not at gas 1 at once", mode_text(md), g);
            p_kick = kick;
        }
        printf("   %s: rise limit %.1f /s held in gears 8/9/10, release and kickdown at once\n", mode_text(md), rate);
    }
}

// ---------------------------------------------------------------- D: the kick
// one run of a pedal script: ramp from p0 to p1 in `ramp` s, then hold; the gas and the wish of n ticks (from a settled p0)
static void trace(int mode, float p0, float p1, float ramp, float *gas, float *q, int n)
{
    world(9, 45, mode);
    m_pedal.reset(RESET_SOFT, "sim");
    hold(p0, 1.0f);
    int nr = (int)lroundf(ramp * 60);
    for (int i = 0; i < n; i++) {
        float p = nr > 0 && i < nr ? p0 + (p1 - p0) * (i + 1) / nr : p1;
        gas[i] = step(p).out;
        if (q) q[i] = S.q;
    }
}
static float max_diff(const float *a, const float *b, int n) { float m = 0; for (int i = 0; i < n; i++) m = fmaxf(m, fabsf(a[i] - b[i])); return m; }
static void kick_checks()
{
    printf("== D kick: Power only, on a fast pedal from near zero to the middle, never over 1\n");
    enum { N = 120 };
    static float with[N], none[N], qq[N];
    struct sc { const char *name; float p0, p1, ramp; int mode; bool kicks; };
    const sc list[] = {
        { "Power 0 -> 0.5 in 0.2 s (2.5 /s)", 0, 0.5f, 0.2f, MODE_POWER, true },
        { "Power 0 -> 0.5 in 0.05 s", 0, 0.5f, 0.05f, MODE_POWER, true },
        { "Power 0 -> 0.35 in 0.15 s", 0, 0.35f, 0.15f, MODE_POWER, true },
        { "Power 0 -> 0.5 in 1 s (0.5 /s)", 0, 0.5f, 1.0f, MODE_POWER, false },
        { "Power 0 -> 0.5 in 0.4 s (1.25 /s)", 0, 0.5f, 0.4f, MODE_POWER, false },
        { "Power 0.2 -> 0.6 in 0.1 s (starts at 0.2)", 0.2f, 0.6f, 0.1f, MODE_POWER, false },
        { "Power 0 -> 0.8 in 0.2 s (ends at 0.8)", 0, 0.8f, 0.2f, MODE_POWER, false },
        { "Power 0 -> 0.2 in 0.05 s (ends at 0.2)", 0, 0.2f, 0.05f, MODE_POWER, false },
        { "Power 0 -> 1 in 0.1 s (kickdown)", 0, 1.0f, 0.1f, MODE_POWER, false },
        { "Normal 0 -> 0.5 in 0.2 s", 0, 0.5f, 0.2f, MODE_NORMAL, false },
        { "Eco 0 -> 0.5 in 0.2 s", 0, 0.5f, 0.2f, MODE_ECO, false },
    };
    for (const sc &s : list) {
        float kick = p_kick;
        trace(s.mode, s.p0, s.p1, s.ramp, with, qq, N);
        p_kick = 0;
        trace(s.mode, s.p0, s.p1, s.ramp, none, NULL, N);
        p_kick = kick;
        float d = max_diff(with, none, N), qmax = 0;
        for (int i = 0; i < N; i++) qmax = fmaxf(qmax, qq[i]);
        if (s.kicks) {
            check(d > 0.03f, "D %s: no kick (the gas differs by %.3f)", s.name, d);
            // fades out: equal again after kick_s (plus a tick or two for the trigger)
            int from = (int)lroundf((s.ramp + p_kick_s) * 60) + 3;
            check(from < N && max_diff(with + from, none + from, N - from) < 1e-4f, "D %s: the kick has not faded out %.2f s after the step", s.name, p_kick_s);
        } else check(d == 0, "D %s: a kick where there should be none (the gas differs by %.3f)", s.name, d);
        check(qmax <= 1.0f, "D %s: the wish reaches %.3f", s.name, qmax);
        printf("   %-44s kick %s (largest gas difference %.3f)\n", s.name, d > 0 ? "yes" : "no ", d);
    }
    // the cap: a huge kick at the top of its range never takes the wish over 1
    p_kick = 0.9f;
    float worst = 0;
    for (float end : { 0.5f, 0.7f }) {
        world(9, 45, MODE_POWER);
        m_pedal.reset(RESET_SOFT, "sim");
        hold(0, 1.0f);
        for (int i = 0; i < 90; i++) { float p = i < 6 ? end * (i + 1) / 6 : end; step(p); worst = fmaxf(worst, S.q); }
    }
    check(worst <= 1.0f && worst > 0.95f, "D cap: the wish with kick 0.9 reaches %.3f (it must reach about 1 and not pass it)", worst);
    printf("   kick 0.9 at the top of the range: the wish reaches %.3f, never more than 1\n", worst);
    p_kick = 0.15f;
}

// ---------------------------------------------------------------- E: pass-through
static const float P = 0.5f;   // the pedal of the pass-through cases
static void passthrough_checks()
{
    printf("== E every pass-through returns the pedal unchanged\n");
    world(9, 45);
    res base = hold(P);
    check(base.on && base.out != P, "E: the wish does not apply in the plain case (%d, %g)", base.on, base.out);
    float top = vehicle_kmh_at(ts_start_gear(0.3f), V.eng.range[R_LOW][1]);
    printf("   (the plain case: gear 9, 45 km/h, pedal %.1f gives gas %.3f; the low-speed range ends at %.1f km/h in start gear %d)\n", P, base.out, top, ts_start_gear(0.3f));
    int n = 0;
    auto run = [&](const char *name, auto setup, float p = P, float dt = 1.0f / 60, int other = 0) {
        world(9, 45);
        setup();
        m_pedal.reset(RESET_SOFT, "sim");
        res r = { 0, 0 };
        for (int i = 0; i < 90; i++) r = step(p, dt, other);
        bool same = r.on == 0 && memcmp(&r.out, &p, sizeof p) == 0;
        check(same, "E %s: the pedal is not returned unchanged (applied %d, out %g, pedal %g)", name, r.on, r.out, p);
        n++;
    };
    run("wish = 0", [] { p_wish = 0; });
    run("clutch open", [] { g_clutch_ours = 0.5f; });
    run("clutch half closed", [] { g_clutch_ours = 0.05f; });
    run("clutch slipping (engine 200 rpm over the driveline)", [] { T.rpm += 200; });
    run("clutch slipping (engine 200 rpm under the driveline)", [] { T.rpm -= 200; });
    run("speed below the low-speed range", [] { int sg = ts_start_gear(0.3f); T.gear = sg; g_gear = sg; T.speed = vehicle_kmh_at(sg, V.eng.range[R_LOW][1]) / 3.6f * 0.97f; T.rpm = vehicle_rpm_at(sg, T.speed); });
    run("walking pace", [] { T.speed = 3 / 3.6f; T.rpm = vehicle_rpm_at(9, T.speed); });
    run("standing", [] { T.speed = 0; });
    run("reverse", [] { T.gear = -1; });
    run("neutral", [] { T.gear = 0; });
    run("electric", [] { V.electric = 1; });
    run("manual box", [] { strcpy(T.shifter, "manual"); });
    run("M mode", [] { g_adapt = 0; });
    run("vehicle model not valid (eng_ok 0)", [] { V.eng_ok = 0; });
    run("mass unknown (mass_ok 0)", [] { V.mass_ok = 0; });
    run("no engine curve (rpm limit 0)", [] { V.eng.rpm_limit = 0; });
    run("no plateau", [] { V.plateau = 0; });
    run("no wheel radius", [] { RADIUS = 0; });
    run("engine off", [] { T.engine_on = 0; });
    run("paused", [] { T.paused = 1; });
    run("crawl (Creep's switch)", [] { g_crawl_on = 1; });
    run("crawl (Creep reports it)", [] { CI.crawl = 1; });
    run("smooth start / launch (Creep)", [] { CI.launch = 1; });
    run("launch control", [] { g_lcx = 2; });
    run("cruise on", [] { T.cruise = 20; });
    run("cruise / ACC / AEB drive the throttle (caller says so)", [] {}, P, 1.0f / 60, 1);
    run("Auto Hold", [] { CU.hold = 1; });
    run("Auto Hold releasing", [] { CU.hold = 2; });
    run("Stop&Go holds", [] { CU.sg_hold = 1; });
    run("Stop&Go feeds the pedal", [] { CU.feed = 0.3f; });
    run("tick time 0", [] {}, P, 0.0f);
    run("tick time 1 s", [] {}, P, 1.0f);
    run("tick time NaN", [] {}, P, NAN);
    run("pedal NaN", [] {}, NAN);
    run("speed NaN", [] { T.speed = NAN; });
    run("rpm NaN", [] { T.rpm = NAN; });
    printf("   %d pass-through conditions checked\n", n);
    // the edges of the two measured conditions: just inside the lock and just over the low-speed range the wish applies
    world(9, 45); T.rpm += 60; m_pedal.reset(RESET_SOFT, "sim");
    check(hold(P, 1).on == 1, "E: the clutch lock test refuses 60 rpm of difference (the limit is %.0f)", PK_SYNC_RPM);
    world(9, 45); g_clutch_ours = 0.01f; m_pedal.reset(RESET_SOFT, "sim");
    check(hold(P, 1).on == 1, "E: a clutch at 0.01 should still count as closed");
    { int sg = ts_start_gear(0.3f); world(sg, 45); T.speed = vehicle_kmh_at(sg, V.eng.range[R_LOW][1]) / 3.6f * 1.03f; T.rpm = vehicle_rpm_at(sg, T.speed); m_pedal.reset(RESET_SOFT, "sim");
      check(hold(P, 1).on == 1, "E: just over the low-speed range the wish should apply"); }
    // coming back from a pass-through: the wish starts from the pedal, not from zero (no hole in the throttle)
    world(9, 45); g_crawl_on = 1; m_pedal.reset(RESET_SOFT, "sim"); hold(0.5f, 1);
    g_crawl_on = 0;
    float back = step(0.5f).out, ref = steady(0.5f);
    check(fabsf(back - ref) < 0.02f, "E: the first tick after a pass-through gives %.3f, steady %.3f", back, ref);
}

// ---------------------------------------------------------------- F: wish 0 and a fuzz (never a NaN)
// ---------------------------------------------------------------- D2: the exponent per truck (curve_ref_kw_t)
static void truck_curve_checks()
{
    printf("== D2 the exponent per truck: power per tonne against curve_ref_kw_t (12), sqrt, clamped to x0.8 ... x1.0\n");
    p_ref_kwt = 12;
    float prev = 2;
    for (float mass : { 10000.0f, 17000.0f, 25000.0f, 33000.0f, 40000.0f, 60000.0f }) {
        float q[3];
        for (int i = 0; i < 3; i++) {
            int md = i == 0 ? MODE_ECO : i == 1 ? MODE_NORMAL : MODE_POWER;
            world(9, 45, md, mass);
            { float sp = T.speed; T.speed = 0; truck_curve(); T.speed = sp; }   // the factor is taken at a standstill only
            steady(0.5f);
            float base = md == MODE_ECO ? p_exp_eco : md == MODE_POWER ? p_exp_pw : p_exp_norm;
            float k = fminf(1.0f, fmaxf(0.8f, sqrtf(peak_power() / mass / 12))), want = powf(0.5f, base * k);
            q[i] = S.q;
            check(fabsf(S.q - want) < 1e-5f, "D2 %.0f t %s: pedal 0.5 wishes %.4f, %.4f expected (x%.3f)", mass / 1000, mode_text(md), S.q, want, k);
        }
        check(q[0] <= q[1] && q[1] <= q[2], "D2 %.0f t: Eco %.4f <= Normal %.4f <= Power %.4f", mass / 1000, q[0], q[1], q[2]);
        check(q[1] >= prev - 1e-6f || prev > 1, "D2 %.0f t: a heavier truck wishes no less at the same pedal (%.4f after %.4f)", mass / 1000, q[1], prev);
        // a light, strong truck keeps the base curve (never softer): pedal 0.5 in Normal wishes 0.5^exponent_normal
        if (peak_power() / mass >= 12) check(fabsf(q[1] - powf(0.5f, p_exp_norm)) < 1e-5f, "D2 %.0f t: over 12 kW/t Normal keeps the base curve (%.4f)", mass / 1000, q[1]);
        printf("   %2.0f t (%.1f kW/t): pedal 0.5 wishes Eco %.3f, Normal %.3f, Power %.3f\n", mass / 1000, peak_power() / mass, q[0], q[1], q[2]);
        prev = q[1];
    }
    // the factor never changes while driving, and falls back to 1 when the truck is not known or after a new load
    world(9, 45, MODE_NORMAL, 17000);
    { float sp = T.speed; T.speed = 0; truck_curve(); T.speed = sp; }
    float k17 = S.truck_k;
    V.mass = 40000; truck_curve();
    check(S.truck_k == k17, "D2 moving: the mass changing to 40 t leaves the factor at x%.3f (was x%.3f)", S.truck_k, k17);
    T.speed = 0; V.mass_ok = 0; truck_curve();
    check(S.truck_k == 1, "D2 standing, mass unknown: factor x%.3f, 1 expected", S.truck_k);
    V.mass_ok = 1; V.electric = 1; truck_curve();
    check(S.truck_k == 1, "D2 standing, electric: factor x%.3f, 1 expected", S.truck_k);
    V.electric = 0; truck_curve();
    check(S.truck_k != 1, "D2 standing, known diesel at 40 t: factor x%.3f, not 1", S.truck_k);
    m_pedal.reset(RESET_LOAD, "sim");
    check(S.truck_k == 1, "D2 after a new load: factor x%.3f, 1 until the next standstill", S.truck_k);
    p_ref_kwt = 0;
    world(9, 45, MODE_NORMAL, 33000);
    steady(0.5f);
    check(fabsf(S.q - 0.5f) < 1e-6f, "D2 curve_ref_kw_t 0: Normal pedal 0.5 wishes %.4f, 0.5 expected", S.q);
}

static void zero_and_fuzz()
{
    printf("== F wish 0 = the pedal unchanged; random inputs never give a NaN or a value outside 0..1\n");
    int bad = 0, n = 0;
    p_wish = 0;
    for (int md : { MODE_ECO, MODE_NORMAL, MODE_POWER })
        for (int g : { 6, 8, 9, 10, 12 }) {
            world(g, 45, md); p_wish = 0;
            for (int i = 0; i <= 100; i++) { float p = i / 100.0f; res r = step(p); n++; if (r.on || memcmp(&r.out, &p, sizeof p)) bad++; }
        }
    check(bad == 0, "F: wish 0 changed the pedal %d times of %d", bad, n);
    printf("   wish 0: %d pedal values in 3 modes and 5 gears, the pedal comes back unchanged every time\n", n);
    params_default();
    unsigned s = 12345;
    auto rnd = [&s]() { s = s * 1664525u + 1013904223u; return (s >> 8) / (float)(1 << 24); };
    int applied = 0, nan = 0, range = 0, changed = 0, total = 0;
    for (int it = 0; it < 4000; it++) {
        int md = (int)(rnd() * 3), g = 1 + (int)(rnd() * 12);
        world(g, rnd() * 100, md == 0 ? MODE_ECO : md == 1 ? MODE_NORMAL : MODE_POWER, 3000 + rnd() * 50000);
        if (rnd() < 0.2f) T.rpm = rnd() * 2500;
        if (rnd() < 0.1f) T.speed = -T.speed;
        if (rnd() < 0.05f) T.speed = NAN;
        for (int i = 0; i < 40; i++) {
            float p = rnd() * 1.4f - 0.2f, dt = rnd() < 0.05f ? rnd() * 2 : 1.0f / 60;
            if (rnd() < 0.02f) p = NAN;
            res r = step(p, dt);
            total++;
            if (r.on) { applied++; if (!(r.out >= 0 && r.out <= 1)) range++; }
            else if (memcmp(&r.out, &p, sizeof p)) changed++;
            if (r.out != r.out && p == p) nan++;
        }
    }
    check(nan == 0 && range == 0 && changed == 0, "F fuzz: %d NaN, %d out of range, %d pass-throughs that changed the pedal", nan, range, changed);
    printf("   fuzz: %d calls, %d applied, no NaN, none out of 0..1, every pass-through returned the pedal itself\n", total, applied);
    check(applied > 200, "F fuzz: only %d calls applied, the fuzz does not reach the wish", applied);
}

// ---------------------------------------------------------------- G: the reference at this speed (ref_now)
// The reference worked out here again (not from the module's code): the gears whose rpm at v lies in the score function's window
// (idle + 60 % of the way to the plateau, at least idle + 150, up to the top of the power range or 93 % of the limit), the best of them
// at full throttle, as engine power
static bool in_window(int g, float v)
{
    float lo = fmaxf(IDLE + 0.6f * fmaxf(0.0f, V.plateau - IDLE), IDLE + 150), hi = fminf(RANGES[R_POWER][1], 0.93f * LIMIT), r = vehicle_rpm_at(g, v);
    return r >= lo && r <= hi;
}
static float pref_now(float v, int *best)   // *best = the gear it comes from, 0 = none in the window
{
    float p = 0;
    *best = 0;
    for (int g = 1; g <= 12; g++) {
        float w = vehicle_force_at(g, v, 1) * v / V.eta;
        if (in_window(g, v) && w > p) { p = w; *best = g; }
    }
    return p;
}
// the steady gas over the pedal 0 ... 0.85 (steps of 0.01) in gear g; returns the longest stretch where it does not rise, *sat = the
// first pedal at gas 1 (1 = none up to 0.85: the blend and the flat pedal take it there)
static float flat_stretch(int g, float kmh, float mass, float *curve, float *sat)
{
    float prev = -1, run = 0, worst = 0;
    *sat = 1;
    world(g, kmh, MODE_NORMAL, mass);
    for (int i = 0; i <= 85; i++) {
        float p = i / 100.0f, gas = steady(p);
        curve[i] = gas;
        if (*sat >= 1 && gas >= 0.999f) *sat = p;
        run = i > 0 && gas <= prev + 1e-4f ? run + 0.01f : 0;
        worst = fmaxf(worst, run);
        prev = gas;
    }
    return worst;
}
static void ref_now_checks()
{
    printf("== G ref_now 1: a full wish is the power the best gear in the usable window gives at this speed, the same in every gear\n");
    static float c1[86], c0[86];
    for (float kmh : { 45.0f, 70.0f })
        for (float mass : { 17000.0f, 33000.0f }) {
            p_ref_now = 1;
            world(9, kmh, MODE_NORMAL, mass);
            int gb = 0;
            float want = pref_now(T.speed, &gb), peak = peak_power();
            printf("   %2.0f km/h, %2.0f t: window gears", kmh, mass / 1000);
            for (int g = 1; g <= 12; g++) if (in_window(g, T.speed)) printf(" %d (%.0f rpm)", g, vehicle_rpm_at(g, T.speed));
            printf("; reference %.0f kW from gear %d, peak %.0f kW (%.1f %%)\n", want / 1000, gb, peak / 1000, 100 * want / peak);
            check(gb > 0 && want <= peak * 1.001f, "G %.0f km/h %.0f t: reference %.0f W from gear %d, peak %.0f W", kmh, mass / 1000, want, gb, peak);
            // the same reference in every gear at this speed (gears where the wish applies: 8 is over the rev limit at 70 km/h)
            const int G45[3] = { 8, 9, 10 }, G70[3] = { 9, 10, 11 };
            const int *gs = kmh < 50 ? G45 : G70;
            float r0 = -1;
            for (int k = 0; k < 3; k++) {
                world(gs[k], kmh, MODE_NORMAL, mass);
                steady(0.5f);
                check(S.on_at >= 0, "G %.0f km/h %.0f t gear %d: the wish does not apply", kmh, mass / 1000, gs[k]);
                if (r0 < 0) r0 = S.pref;
                check(S.pref == r0, "G %.0f km/h %.0f t: the reference in gear %d is %.1f W, in gear %d %.1f W", kmh, mass / 1000, gs[k], S.pref, gs[0], r0);
                check(fabsf(S.pref - want) <= 0.001f * want, "G %.0f km/h %.0f t gear %d: reference %.0f W, %.0f W expected", kmh, mass / 1000, gs[k], S.pref, want);
            }
            printf("   the reference in gears %d/%d/%d: %.1f W in each\n", gs[0], gs[1], gs[2], r0);
            // the curve in the gear the reference comes from, ref_now 1 against 0; then where the other gears of the window reach gas 1
            float s1, s0, f1 = flat_stretch(gb, kmh, mass, c1, &s1);
            p_ref_now = 0;
            float f0 = flat_stretch(gb, kmh, mass, c0, &s0);
            p_ref_now = 1;
            printf("   gear %2d pedal  ", gb);
            for (int i = 0; i <= 85; i += 5) printf(" %5.2f", i / 100.0f);
            printf("\n     ref_now 1 gas");
            for (int i = 0; i <= 85; i += 5) printf(" %5.3f", c1[i]);
            printf("   longest flat %.2f, gas 1 at %.2f\n     ref_now 0 gas", f1, s1);
            for (int i = 0; i <= 85; i += 5) printf(" %5.3f", c0[i]);
            printf("   longest flat %.2f, gas 1 at %.2f\n", f0, s0);
            check(f1 <= 0.05f + 1e-4f, "G %.0f km/h %.0f t gear %d: a flat stretch of %.2f pedal with ref_now 1", kmh, mass / 1000, gb, f1);
            for (int g = 1; g <= 12; g++) {
                if (g == gb || !in_window(g, kmh / 3.6f)) continue;
                float sa1, sa0, fl1 = flat_stretch(g, kmh, mass, c1, &sa1);
                p_ref_now = 0;
                float fl0 = flat_stretch(g, kmh, mass, c0, &sa0);
                p_ref_now = 1;
                printf("     gear %2d (%.0f rpm): gas 1 at pedal %.2f / %.2f (ref_now 1 / 0), longest flat %.2f / %.2f\n", g, vehicle_rpm_at(g, T.speed), sa1, sa0, fl1, fl0);
            }
            // wish 0: the pedal itself with either reference
            int bad = 0;
            for (float rn : { 0.0f, 1.0f })
                for (int k = 0; k < 3; k++) {
                    world(gs[k], kmh, MODE_NORMAL, mass);
                    p_wish = 0; p_ref_now = rn;
                    for (int i = 0; i <= 100; i++) { float p = i / 100.0f; res r = step(p); if (r.on || memcmp(&r.out, &p, sizeof p)) bad++; }
                }
            check(bad == 0, "G %.0f km/h %.0f t: wish 0 changed the pedal %d times", kmh, mass / 1000, bad);
        }
    p_ref_now = g_ref_now;
    p_tq_share = 0;   // these sections test the wish itself; torque_share (the hold in a gear) has its own sim (sim_pedalmix)
}

int main(int argc, char **argv)
{
    verbose = argc > 1 && !strcmp(argv[1], "-v");
    float r[16];
    for (int k = 0; k < 12; k++) r[k] = 11.73f * powf(0.79f / 11.73f, k / 11.0f);   // Optidriver ATO2612F: 12 speeds, no crawl gears
    memcpy(RATIO, r, sizeof r);
    params_default();
    gears_table();
    cap_checks();
    shape_checks();
    rise_checks();
    kick_checks();
    passthrough_checks();
    truck_curve_checks();
    params_default();
    zero_and_fuzz();
    // ref_now 1 (the default): the reference at this speed, then the cap, the shape, the pass-throughs and the fuzz again with it
    params_default();
    ref_now_checks();
    g_ref_now = 1;
    params_default();
    cap_checks();
    shape_checks();
    passthrough_checks();
    zero_and_fuzz();
    printf("\n%d checks, %d failed\n", n_checks, n_failed);
    return n_failed ? 1 : 0;
}
