// Offline test of the Cruise module's controller (modules/cruise.cpp is
// included, so its static functions are reachable). A simple truck: force =
// throttle x max force, a 0.5 s torque gap at each gear change, a lead
// vehicle for the ACC, hills. Compares the game's cruise (an incremental PI
// that keeps integrating while the clutch is open) with ours.
// Build+run: tools/sim/run_cruise.sh
#include "core/suite.h"
#include "core/game.h"
#include "core/route.h"
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <initializer_list>

telemetry T;
char g_dir[MAX_PATH];
const layout *L;
game_addrs G;
volatile int g_rolling, g_cruise_smooth, g_crawl_on;
volatile float g_roll_boost_kmh;
volatile double g_rb_cancel_at = -1e9;
volatile int g_pcc_on;
volatile int g_pedal_mapped;
volatile int g_lcx;
volatile float g_overspeed_cap = 5;
volatile float g_pcc_need_kmh = NAN;
volatile float g_decel_need;
float ts_brake_down_eta(float) { return NAN; }   // (TrueShift is not in this sim: no shift is expected)
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
int game_letter(void *, char) { return 1; }   // the dashboard letter's writer (core/display.cpp)
static double t_now;
double wall_s() { return t_now; }
double sim_s() { return t_now; }
void csv_cap(FILE **, const char *) {}
FILE *open_out(const char *, const char *) { return NULL; }
FILE *open_incident(const char *, const char *, double, char *s, size_t n) { if (n) s[0] = 0; return NULL; }
static int verbose;
void logline(const char *fmt, ...)
{
    if (!verbose) return;
    va_list ap; va_start(ap, fmt); printf("%7.1f  ", t_now); vprintf(fmt, ap); printf("\n"); va_end(ap);
}
void play_cue(int) {}
int input_pressed(int, int) { return 0; }
int input_held(int) { return 0; }
static float g_cvar_adaptive = 3;
float game_adaptive() { return g_cvar_adaptive; }
void *game_cvar(const char *) { return NULL; }
float game_cvar_get(void *) { return NAN; }
void game_cvar_set(void *, float) {}
uintptr_t game_scan(const char *) { return 0; }
int game_patch_call(uintptr_t, void *) { return 0; }
int game_patch_retarder(uintptr_t, const retarder_scale *) { return 0; }
float *game_patch_rip_float(uintptr_t, int, int, float) { return NULL; }
int game_patch_feed_xmm1(uintptr_t, int, float *) { return 0; }
int game_patch_skip_if(uintptr_t, int, const unsigned char *) { return 0; }
int game_patch_zero_xmm_if(uintptr_t, int, int, const unsigned char *) { return 0; }
int game_patch_cf0_if(uintptr_t, int, const unsigned char *) { return 0; }
int mem_readable(const void *p, size_t) { return p != NULL; }
static float g_sim_mass = 26000;
static engine_info g_sim_eng;
float game_mass() { return g_sim_mass; }
float game_grade_deg(void *) { return NAN; }   // the vehicle model asks; Cruise decides on the pitch
void *game_gearbox() { return (void *)&g_sim_eng; }
int gb_engine(void *, engine_info *e) { *e = g_sim_eng; return 1; }
float curve_factor(const engine_info *e, float rpm) { return rpm < 1000 ? 0.8f + 0.2f * rpm / 1000 : rpm < 1500 ? 1.0f : 1.0f - (rpm - 1500) / 3000; }
float wheel_radius() { return 0.5f; }
int pedal_wish(float p, float, int, float *out) { *out = p; return 0; }   // the wish pedal (modules/pedal.cpp) is off here: the old pedal curves
static char GA[0x20000];
char *game_actor_safe() { return GA; }
static int g_route;
static float (*g_road)(double s);
static double g_s;
int route_ok() { return g_route && g_road; }
float route_grade(float d0, float d1) { if (!route_ok()) return NAN; double h = 0; for (float x = d0; x < d1; x += 5) h += g_road(g_s + x) / 100 * 5; return (float)(h / (d1 - d0) * 100); }
float route_min_grade(float d0, float d1) { if (!route_ok()) return NAN; float b = 99; for (float d = d0; d + 50 <= d1; d += 25) b = fminf(b, route_grade(d, d + 50)); return b; }
float route_max_grade(float d0, float d1) { if (!route_ok()) return NAN; float b = -99; for (float d = d0; d + 50 <= d1; d += 25) b = fmaxf(b, route_grade(d, d + 50)); return b; }
float route_crest_m(float max_d) { if (!route_ok() || route_grade(0, 50) < 0.5f) return -1; for (float d = 25; d + 50 <= max_d; d += 25) if (route_grade(d, d + 50) < 0.2f) return d + 25; return -1; }
float route_valley_m(float) { return -1; }
static float g_bend_ms = NAN;
float route_curve_speed(float, float, float) { return route_ok() ? g_bend_ms : NAN; }
float route_bend_speed(float, float) { return route_ok() ? g_bend_ms : NAN; }   // the scenario's bend is a road bend, not a junction
float route_junction_m() { return -1; }
int route_junction_known(float) { return 0; }
void route_junction_dump(char *b, size_t n) { snprintf(b, n, "none"); }
void route_junction_cfg(int, float, float, float, float, int) {}
void route_hermite(int) {}
// a junction or roundabout the scenario puts on the route (dist_m <= 0 = none); the speed profile as route.cpp plans it
static route_junction_t g_sim_jn;
float route_junction_speed(float a_dec, route_junction_t *ji)
{
    if (!route_ok() || !(g_sim_jn.dist_m > 0)) return NAN;
    if (ji) *ji = g_sim_jn;
    float d = g_sim_jn.dist_m - (g_sim_jn.roundabout ? 45 : 25);
    return sqrtf(g_sim_jn.limit_ms * g_sim_jn.limit_ms + 2 * a_dec * (d > 0 ? d : 0));
}
float route_limit_ahead(float, float *d) { if (d) *d = NAN; return NAN; }
void route_probe_limit(float) {}

// the low-speed owner (lowspeed.cpp) is not part of this sim: off unless a scenario sets these
static int s_low_on, s_low_stopped, s_low_zone;
int low_owner_on() { return s_low_on; }
int low_stopped() { return s_low_on && s_low_stopped; }
int low_sg_zone() { return s_low_on && s_low_zone; }

#include "modules/cruise.cpp"
#include "core/display.cpp"
#include "core/rsc.cpp"
#include "core/actuate.cpp"
#include "core/vehicle.cpp"
namespace dc {   // the deceleration assist (its statics share names with Cruise's), for the stop by the lever
#include "modules/decel.cpp"
}

// ---- the game's cruise (incremental PI, see cruise-re notes § 1)
static float stock_thr;
static float stock_cruise(void *ga, float dt, float pedal)
{
    char *g = (char *)ga;
    if (!*(unsigned char *)(g + GA_ON)) { stock_thr = 0; return pedal; }
    float set = *(float *)(g + GA_SET), acc = *(float *)(g + GA_ACC_TARGET);
    float target = *(float *)(g + GA_LEAD_M) > 0.5f && acc < set ? acc : set;
    static float e_prev;
    float e = fmaxf(-2.5f, fminf(2.5f, target - fabsf(T.speed) * 3.6f));
    stock_thr += 0.8f * (e - e_prev) + 0.4f * e * dt;   // keeps integrating during shifts
    e_prev = e;
    stock_thr = fmaxf(0, fminf(1, stock_thr));
    return fmaxf(stock_thr, pedal);
}

// ======================================================================= the actuator without the Cruise module ([cruise] enabled = 0)
// Nothing of Cruise is attached (as cr_start leaves it with enabled = 0): the tick is the game's own cruise fn plus the actuator.
// Creep's descent brake must reach the brake request (ramped by the writer, the hazards muted), its crawl throttle the engine, and a
// throttle cap (launch control's; the rollover guard's goes the same way) the merged throttle. Before 26-10 all of this only went
// through Cruise's hook. Runs first, in a fresh process; what it touches (the writer's memory, the wishes, the clock) is put back so
// the runs below start as they always did.
static void no_cruise_test()
{
    int bad = 0;
    printf("--- the actuator without the Cruise module (as with [cruise] enabled = 0)\n");
    memset(GA, 0, sizeof GA); o_cruise = stock_cruise; stock_thr = 0;
    T.speed = 6 / 3.6f; T.air_psi = 120; T.air_warning = 0; T.paused = 0; T.gear = 1;   // crawling downhill at 6 km/h
    act_stub(GA, 1 / 60.0f, 0); t_now += 1 / 60.0;                                       // one quiet tick: the writer's clock runs
    // Creep posts its descent brake (0.4) and heavy-load throttle (0.12) from frame_end; the tick merges them
    act_brake(ACT_CREEP, 0.4f); act_thr_floor(ACT_CREEP, 0.12f);
    float thr = 0, req = 0, req_1s = 0;
    for (int k = 0; k < 180; k++, t_now += 1 / 60.0) {
        thr = act_stub(GA, 1 / 60.0f, 0);
        req = *(float *)(GA + GA_BRAKE_REQ);
        if (k == 59) req_1s = req;
    }
    int muted = s_hazard_mute;
    printf("no Cruise: crawl brake wish 0.40 -> request %.2f after 1 s, %.2f after 3 s (ramped at 0.25/s, settles within 0.05 under the wish), hazards muted %d; crawl throttle 0.12 -> engine %.2f with the pedal up\n",
           req_1s, req, muted, thr);
    bad |= !(req_1s > 0.2f && req_1s < 0.3f) || !(req > 0.34f && req <= 0.4f) || !muted || fabsf(thr - 0.12f) > 0.001f;
    // a throttle cap (launch control holding the revs): the merged throttle follows it, the pedal comes back when it is released
    act_thr_cap(ACT_LCX, 0.5f);
    float capped = act_stub(GA, 1 / 60.0f, 0.8f); t_now += 1 / 60.0;
    act_thr_cap(ACT_LCX, 1.0f);
    float pedal = act_stub(GA, 1 / 60.0f, 0.8f); t_now += 1 / 60.0;
    printf("no Cruise: launch control cap 0.50 -> engine %.2f at pedal 0.80; cap released -> %.2f\n", capped, pedal);
    bad |= fabsf(capped - 0.5f) > 0.001f || fabsf(pedal - 0.8f) > 0.001f;
    // the wishes withdrawn: the request fades (0.6/s) and is gone
    act_brake(ACT_CREEP, 0); act_thr_floor(ACT_CREEP, 0);
    for (int k = 0; k < 120; k++, t_now += 1 / 60.0) { thr = act_stub(GA, 1 / 60.0f, 0); req = *(float *)(GA + GA_BRAKE_REQ); }
    printf("no Cruise: wishes withdrawn -> request %.2f and throttle %.2f after 2 s (both 0), hazards muted %d (0)\n", req, thr, (int)s_hazard_mute);
    bad |= req != 0 || thr != 0 || act_request() != 0 || s_hazard_mute;
    printf("no-Cruise test: %s\n", bad ? "FAIL" : "ok");
    if (bad) exit(1);
    // a fresh start for everything below
    act_reset(RESET_HARD, "sim"); memset(GA, 0, sizeof GA); memset(&T, 0, sizeof T); stock_thr = 0; t_now = 0;
}

// ---- truck
static struct { float v, mass, grade; double shift_end; int gear; float lead_v, lead_x; int lead; } M;
static float road_flat(double) { return 0; }
static float road_hills(double s) { return 2.5f * sinf((float)(s / 1200 * 6.2832)); }
static float gear_up_kmh[] = { 8, 14, 20, 27, 35, 45, 56, 68, 80, 999 };

struct result { float max_over, min_under, max_du, mean_abs_e; int stops_held, fed; };

static result run(const char *name, int ours, float set_kmh, double secs, void (*ctl)(double), float v0)
{
    memset(&M, 0, sizeof M); memset(GA, 0, sizeof GA); memset(&C, 0, sizeof C);
    M.mass = 26000; M.v = v0 / 3.6f; g_s = 0; stock_thr = 0;
    p_smooth = ours; g_sg_ok = 1; g_low_ok = 1; g_sg_hold = 0; g_feed = 0; g_stopped_at = 0;
    *(unsigned char *)(GA + GA_ON) = 1; *(float *)(GA + GA_SET) = set_kmh;
    o_cruise = stock_cruise;
    result r = { -99, 99, 0, 0, 0, 0 };
    double t0 = t_now, sum_e = 0; int n = 0; float u_prev = 0;
    for (; t_now - t0 < secs; t_now += 1 / 60.0) {
        double t = t_now - t0;
        ctl(t);
        if (g_road) M.grade = g_road(g_s);
        T.speed = M.v; T.cruise = set_kmh / 3.6f;
        T.place.orientation.pitch = atanf(M.grade / 100) / 6.2831853f;
        // ACC: lead vehicle
        float acc_t = 0;
        if (M.lead) {
            M.lead_x += M.lead_v / 3.6f / 60;
            float gap = M.lead_x - (float)g_s, want = fmaxf(4, 2.0f * M.v);
            acc_t = fmaxf(0, M.lead_v + (gap - want) * 0.8f);
            *(float *)(GA + GA_LEAD_M) = gap;
        } else *(float *)(GA + GA_LEAD_M) = 0;
        *(float *)(GA + GA_ACC_TARGET) = M.lead ? acc_t : 0;
        // gearbox: torque gap when crossing a shift speed
        float kmh = M.v * 3.6f;
        if (t_now >= M.shift_end && kmh > gear_up_kmh[M.gear]) { M.gear++; M.shift_end = t_now + 0.5; }
        if (t_now >= M.shift_end && M.gear > 0 && kmh < gear_up_kmh[M.gear - 1] - 3) { M.gear--; M.shift_end = t_now + 0.5; }
        int shifting = t_now < M.shift_end;
        T.eff_clutch = shifting ? 1 : 0;
        *(float *)(GA + GA_BRAKE_REQ) = 0;  // the game rewrites it every tick (stock ACC: 0 unless emergency)
        vehicle_frame();
        float u = ours ? act_stub(GA, 1 / 60.0f, 0) : stock_cruise(GA, 1 / 60.0f, 0);
        float brake = *(float *)(GA + GA_BRAKE_REQ);
        if (M.lead) {  // the game's ACC brakes itself when closing in (cruise-re notes § 2)
            float gap = M.lead_x - (float)g_s, want = fmaxf(4, 2.0f * M.v);
            if (gap < want) brake = fmaxf(brake, fminf(1, (want - gap) / want * 1.5f));
        }
        if (brake > 0) u = 0;
        // stock: at standstill the automatic goes to neutral unless the plugin feeds it
        int neutral = kmh < 0.5f && g_feed < 0.01f;
        float F = shifting || neutral ? 0 : u * 20000.0f * (M.gear < 4 ? 2.0f : 1.0f);
        float res = M.mass * 9.81f * (sinf(atanf(M.grade / 100)) + 0.006f) + 0.5f * 1.2f * 6 * M.v * M.v;
        if (M.v < 0.05f && F < res) res = fminf(res, F);  // standing still: no rolling back in this model
        res += brake * M.mass * 4;
        M.v = fmaxf(0, M.v + (F - res) / M.mass / 60);
        g_s += M.v / 60;
        if (t > 20) {
            float tgt = M.lead ? fminf(set_kmh, acc_t) : set_kmh, e = kmh - tgt;
            r.max_over = fmaxf(r.max_over, e); r.min_under = fminf(r.min_under, e);
            sum_e += fabsf(e); n++;
        }
        if (t > 1) r.max_du = fmaxf(r.max_du, (u - u_prev) * 60);
        u_prev = u;
        if (g_sg_hold) r.stops_held++;
        if (g_feed > 0) r.fed++;
    }
    r.mean_abs_e = n ? (float)(sum_e / n) : 0;
    printf("%-46s %s  over %+5.1f under %+5.1f km/h, mean |e| %.2f, max throttle rise %.1f/s, end %.0f km/h, held %.1f s, fed %.1f s\n", name,
           ours ? "ours " : "game ", r.max_over, r.min_under, r.mean_abs_e, r.max_du, M.v * 3.6f,
           r.stops_held / 60.0, r.fed / 60.0);
    return r;
}

static void c_none(double) { }
static void c_lead_jam(double t) { M.lead = 1; if (t < 1) { M.lead_x = 40; } M.lead_v = t < 30 ? 30 : t < 45 ? 0 : t < 75 ? 25 : 0; }
static void c_hills(double) { g_road = road_hills; }

// ---- cruise keys: tap = one step to the next multiple of 5, hold = 1 km/h steps, retarder off on a press
static char act_buf[4][0x100];
static int ret_calls;
static void fake_retarder(void *ga, unsigned level, unsigned char) { ret_calls++; *(unsigned *)((char *)ga + 0x180) = level; }
static float stock_grid2(float s, int dir) { return dir > 0 ? (floorf((s + 0.5f) / 2) + 1) * 2 : floorf((s - 0.5f) / 2) * 2; }
// One press: key = 1 inc / 2 dec; the key is down for `down` s; repeat = the game re-fires the event every tick.
// Mimics the order of a tick: game handler (event + its own grid step), then keys_tick before the controller.
static float key_run(int key, double down, int repeat, float start, int cruise_on, int *ons = NULL)
{
    for (int i = 0; i < 4; i++) *(void **)(GA + 0x1e710 + 0x10 * i) = act_buf[i];
    memset(&K, 0, sizeof K); s_prev_on = cruise_on;
    *(unsigned char *)(GA + GA_ON) = cruise_on; *(float *)(GA + GA_SET) = start;
    int dir = key == 1 ? 1 : -1;
    unsigned *st = (unsigned *)(act_buf[key] + 0xf8);
    double t0 = t_now;
    for (; t_now - t0 < down + 0.6; t_now += 1 / 60.0) {
        double t = t_now - t0;
        int pressed = t < down;
        int fire = pressed && (t < 1e-9 || repeat);
        *st = pressed ? 1 : 0;
        if (fire) {
            *st = 2;
            key_event(GA, act_buf[key]);                   // the wrapped consume call
            *st = repeat ? 0 : 1;
            if (*(unsigned char *)(GA + GA_ON)) *(float *)(GA + GA_SET) = stock_grid2(*(float *)(GA + GA_SET), dir);
            else { *(unsigned char *)(GA + GA_ON) = 1; *(float *)(GA + GA_SET) = start; }  // activation press: set = speed
        }
        keys_tick(GA);
    }
    if (ons) *ons = *(unsigned char *)(GA + GA_ON);
    return *(float *)(GA + GA_SET);
}
static void keys_test()
{
    g_keys_ok = 1; g_ret_ok = 1; o_retarder = fake_retarder; g_low_ok = 1;
    p_step_tap = 5; p_step_hold = 1; p_hold_s = 0.4f; p_hold_rate = 8; p_ret_off = 1; p_min_set = 20; p_min_kmh = 5;
    int bad = 0;
    for (int repeat = 0; repeat <= 1; repeat++) {
        float a = key_run(1, 0.15, repeat, 83, 1), b = key_run(1, 0.15, repeat, 85, 1), c = key_run(2, 0.15, repeat, 83, 1),
              d = key_run(2, 0.15, repeat, 85, 1);
        printf("keys (%s): tap + 83 -> %.0f (85), tap + 85 -> %.0f (90), tap - 83 -> %.0f (80), tap - 85 -> %.0f (80)\n",
               repeat ? "repeating event" : "single event", a, b, c, d);
        bad |= a != 85 || b != 90 || c != 80 || d != 80;
        float h = key_run(1, 1.2, repeat, 80, 1), hd = key_run(2, 1.2, repeat, 80, 1);
        printf("keys (%s): hold + 1.2 s from 80 -> %.0f, hold - -> %.0f (expect about 87 / 73: nothing for 0.4 s, then 8/s)\n",
               repeat ? "repeating event" : "single event", h, hd);
        bad |= h < 85 || h > 87 || hd < 73 || hd > 75;
        float h2 = key_run(1, 0.45, repeat, 80, 1);
        printf("keys (%s): hold + 0.45 s from 80 -> %.0f (81: one hold step, no 5 jump)\n", repeat ? "repeating event" : "single event", h2);
        bad |= h2 < 80.5f || h2 > 82.5f;
    }
    int on;
    float act = key_run(1, 0.2, 1, 70, 0, &on);   // + while off switches on at 70; the key stays down 0.2 s
    printf("keys: + while off -> on %d, set %.0f (70: switching on is not a step)\n", on, act);
    bad |= !on || act != 70;
    *(unsigned *)(GA + 0x180) = 2; ret_calls = 0;
    *(unsigned char *)(GA + GA_ON) = 0;
    key_run(1, 0.1, 0, 70, 0);
    printf("keys: retarder level 2, + pressed with cruise off -> retarder %u, setter calls %d (0, 1)\n", *(unsigned *)(GA + 0x180), ret_calls);
    bad |= *(unsigned *)(GA + 0x180) != 0 || ret_calls != 1;
    *(unsigned *)(GA + 0x180) = 3;
    key_event(GA, act_buf[3]);  // resume
    printf("keys: retarder 3, resume pressed -> retarder %u\n", *(unsigned *)(GA + 0x180));
    bad |= *(unsigned *)(GA + 0x180) != 0;
    *(unsigned *)(GA + 0x180) = 3; *(unsigned char *)(GA + GA_ON) = 1;
    key_event(GA, act_buf[0]);  // on/off key while on = switching off: leave the retarder
    printf("keys: retarder 3, on/off pressed with cruise on -> retarder %u (3 = untouched)\n", *(unsigned *)(GA + 0x180));
    bad |= *(unsigned *)(GA + 0x180) != 3;
    // resume while on -> limit + offset at once (unknown limit = nothing)
    p_res_lim = 1; p_isa_off = 3; p_limiter = 0; p_min_set = 20;
    struct { float lim_kmh, start; } rc[] = { { 80, 100 }, { 120, 100 }, { 0, 100 }, { 15, 100 } };
    float want[] = { 83, 123, 100, 20 };  // 80 -> 83; 15 -> min_set 20
    for (int i = 0; i < 4; i++) {
        T.speed_limit = rc[i].lim_kmh / 3.6f;
        *(unsigned char *)(GA + GA_ON) = 1; *(float *)(GA + GA_SET) = rc[i].start;
        key_event(GA, act_buf[3]);
        *(float *)(GA + GA_SET) = 110;  // the game's own resume does something to the set speed
        keys_tick(GA);
        float sv = *(float *)(GA + GA_SET);
        printf("keys: resume while on, limit %.0f, set %.0f -> %.0f (expect %.0f)\n", rc[i].lim_kmh, rc[i].start, sv, want[i]);
        bad |= sv != want[i];
    }
    // overspeed cap: limit 80, set 83, cap 5 -> 2 km/h of room; no limit known or cap 0 -> no cap
    { float save = g_overspeed_cap; T.speed_limit = 80 / 3.6f; g_overspeed_cap = 5;
      float r1 = overspeed_room(83 / 3.6f), r2 = overspeed_room(90 / 3.6f); T.speed_limit = 0; float r3 = overspeed_room(83 / 3.6f);
      T.speed_limit = 80 / 3.6f; g_overspeed_cap = 0; float r4 = overspeed_room(83 / 3.6f); g_overspeed_cap = save; T.speed_limit = 0;
      printf("overspeed cap: room %.1f (expect 2), set over the cap %.1f (expect 0), no limit %s, cap 0 %s\n", r1, r2, r3 > 1e8f ? "no cap" : "CAP", r4 > 1e8f ? "no cap" : "CAP");
      bad |= fabsf(r1 - 2) > 0.01f || r2 != 0 || r3 < 1e8f || r4 < 1e8f; }
    // smart overspeed: full on the flat, gliding down to the steep-descent value between -1 % and -2 %, less with a heavy trailer
    { float sp = p_smart_kmh, dc = p_desc_kmh, cr = T.cruise; p_smart_kmh = 3; p_desc_kmh = 2; T.cruise = 80 / 3.6f;
      auto at = [](float g) { T.place.orientation.pitch = atanf(g / 100) / 6.2831853f; vehicle_frame(); return smart_dynamic(); };   // the pitch reaches Cruise through the vehicle model's frame
      float w0 = at(0), w1 = at(-1.5f), w2 = at(-3), w3 = at(+2);
      printf("smart overspeed: flat %.2f, -1.5 %% %.2f, -3 %% %.2f, uphill %.2f (expect 3.2, between, 2.0, 3.2)\n", w0, w1, w2, w3);
      bad |= !(w0 > 3.0f && w0 < 3.4f) || !(w1 < w0 && w1 > w2) || !(w2 > 1.9f && w2 < 2.2f) || fabsf(w3 - w0) > 0.01f;
      float sm = g_sim_mass; g_sim_mass = NAN; vehicle_reset(RESET_HARD); float wu = at(-3); g_sim_mass = sm; vehicle_reset(RESET_HARD);
      printf("smart overspeed, mass unknown: -3 %% %.2f (%.0f t known %.2f; expect not more)\n", wu, sm / 1000, w2);
      bad |= !(wu <= w2 + 0.001f);
      T.place.orientation.pitch = 0; vehicle_frame(); p_smart_kmh = sp; p_desc_kmh = dc; T.cruise = cr; }
    // pedal curve after a mode change: stays while the pedal is held, blends with the pedal travel
    { float se = p_pc_eco, sp = p_pc_pw, st = p_pc_travel; p_pc_eco = 1.35f; p_pc_pw = 1.6f; p_pc_travel = 1.0f;
      float e0 = pedal_blend(MODE_ECO, 0.5f);                       // first call: Eco curve
      float held = 0; for (int i = 0; i < 100; i++) held = pedal_blend(MODE_POWER, 0.5f);   // switched to Power, pedal held: still Eco
      float mid = 0; for (float p = 0.5f; p <= 0.9f; p += 0.05f) mid = pedal_blend(MODE_POWER, p);    // 0.4 of travel
      float end = 0; for (int k = 0; k < 6; k++) { pedal_blend(MODE_POWER, 0.2f); end = pedal_blend(MODE_POWER, 0.5f); }  // plenty of travel
      printf("pedal blend: eco %.2f, held after the switch %.2f (expect 0.39 both), mid %.2f, end %.2f (expect Power 0.67)\n", e0, held, mid, end);
      bad |= fabsf(e0 - 0.39f) > 0.01f || fabsf(held - e0) > 0.001f || !(mid > e0 && mid < 0.9f) || fabsf(end - 0.67f) > 0.01f;
      p_pc_eco = se; p_pc_pw = sp; p_pc_travel = st; }
    // parking brake after a long Auto Hold: engaged after auto_hold_park_s, confirmed by the game, released by gas (and the hold is taken over)
    {
        float sp = p_ah_park; p_ah_park = 2; g_low_ok = 1; p_ah = 1; p_ah_req = 0.6f;
        memset(&T, 0, sizeof T); T.engine_on = 1; T.gear = 1; T.dgear = 1; T.in_brake = 0.3f; T.fwd_gears = 12;
        memset(&AH, 0, sizeof AH); s_park_on = s_park_ok = s_park_bad = 0; s_ah_hold_since = 0; g_sg_hold = 0; g_crawl_on = 0;
        GA[0x15c] = 0; *(float *)(GA + 0x160) = 0;
        vehicle_frame();   // flat (T was cleared)
        for (int i = 0; i < 60 * 3; i++, t_now += 1 / 60.0) { auto_hold(GA, 1 / 60.0f); T.parking_brake = GA[0x15c] == 1; }  // the game shows what we wrote
        int requested = s_park_on && GA[0x15c] == 1;
        for (int i = 0; i < 60 * 2; i++, t_now += 1 / 60.0) { auto_hold(GA, 1 / 60.0f); T.parking_brake = GA[0x15c] == 1; }
        int confirmed = s_park_ok, still_holding = AH.st == AH_HOLD;
        T.in_brake = 0; T.in_thr = 0.3f;
        for (int i = 0; i < 10; i++, t_now += 1 / 60.0) { auto_hold(GA, 1 / 60.0f); T.parking_brake = GA[0x15c] == 1; }
        int released = !s_park_on && GA[0x15c] == 0 && (AH.st == AH_LAUNCH || AH.st == AH_OFF);
        // a game that does not show the parking brake: put back, off for the session
        T.parking_brake = 0; T.in_thr = 0; T.in_brake = 0.3f; memset(&AH, 0, sizeof AH); s_ah_hold_since = 0; s_park_bad = 0;
        for (int i = 0; i < 60 * 5; i++, t_now += 1 / 60.0) auto_hold(GA, 1 / 60.0f);
        int reverted = s_park_bad && !s_park_on && GA[0x15c] == 0;
        printf("auto hold park: requested %d, confirmed %d (hold %d), released by gas %d, reverted when unconfirmed %d\n", requested, confirmed, still_holding, released, reverted);
        bad |= !(requested && confirmed && still_holding && released && reverted);
        p_ah_park = sp; s_park_bad = 0; memset(&AH, 0, sizeof AH);
    }
    // own speed limiter 95: a climb at 92 keeps the throttle, a flat road at 93.5 fades, a fast approach cuts early
    { float sl = p_own_lim, sb = p_own_bias; p_own_lim = 95; p_own_bias = 0;
      float k1 = own_limit_k(92, 0), k2 = own_limit_k(94.2f, 0), k3 = own_limit_k(93.5f, 0.2f), k4 = own_limit_k(94.8f, -0.5f), k5 = own_limit_k(95.2f, 0);
      printf("own limiter: climb at 92 %.2f, flat 94.2 %.2f, 93.5 and accelerating %.2f, 94.8 and slowing %.2f, over the limit %.2f (expect 1.00 0.53 0.53 1.00 0.00)\n", k1, k2, k3, k4, k5);
      bad |= fabsf(k1 - 1) > 0.01f || fabsf(k2 - 0.53f) > 0.03f || fabsf(k3 - 0.53f) > 0.05f || fabsf(k4 - 1) > 0.01f || k5 != 0;
      p_own_lim = sl; p_own_bias = sb; }
    // rollover guard: a sway that grows (0.6 Hz, 3.5 -> 9 m/s2 at 85 km/h) is caught, normal driving is not
    {
        auto run_rsc = [](float a0, float a1, float f, float secs, float kmh, double *lvl2_at, float *brake_max, int *lvl_max) {
            T.paused = 0; T.gear = 1; T.speed = kmh / 3.6f; g_rsc_level = 0;
            *lvl2_at = -1; *brake_max = 0; *lvl_max = 0;
            for (double t = 0; t < secs; t += 1 / 60.0, t_now += 1 / 60.0) {
                float amp = a0 + (a1 - a0) * (float)(t / secs), a = f > 0 ? amp * sinf(6.2831853f * f * (float)t) : amp;
                T.yaw = a / (T.speed * 6.2831853f);
                rsc_step(1 / 60.0f);
                if (g_rsc_level >= 2 && *lvl2_at < 0) *lvl2_at = t;
                float rb = act_wishes()[ACT_RSC].brake;   // the guard's brake wish as the actuator holds it
                if (rb > *brake_max) *brake_max = rb;
                if (g_rsc_level > *lvl_max) *lvl_max = g_rsc_level;
            }
            T.yaw = 0; rsc_step(1 / 60.0f);
        };
        double l2; float bm; int lm;
        run_rsc(3.5f, 9.0f, 0.6f, 8, 85, &l2, &bm, &lm);
        int sway_ok = l2 > 0 && l2 < 4.0 && bm > 0.15f && lm >= 2;
        double l2b; float bmb; int lmb;
        run_rsc(0.5f, 2.2f, 0.25f, 12, 85, &l2b, &bmb, &lmb);      // lane changes and bends: well under the floor
        run_rsc(1.0f, 1.0f, 0.0f, 6, 85, &l2b, &bmb, &lmb);
        int calm_ok = lmb == 0 && bmb == 0;
        float lvl_normal = 0; { double q; float w; int m; run_rsc(3.0f, 3.0f, 0.05f, 10, 40, &q, &w, &m); lvl_normal = (float)m; }   // a roundabout at 40 km/h
        double q0; float w0; int m0; run_rsc(6.5f, 6.5f, 0.0f, 3, 70, &q0, &w0, &m0);    // one bend held, rollover_bend off (the default): nothing
        T.yaw = 0; run_rsc(0.0f, 0.0f, 0.0f, 3, 70, &q0, &w0, &m0);
        g_rsc_bend = 1;
        double q1; float w1; int m1; run_rsc(5.0f, 5.0f, 0.0f, 3, 70, &q1, &w1, &m1);    // one bend held at 5 m/s2: throttle off, no brake
        T.yaw = 0; run_rsc(0.0f, 0.0f, 0.0f, 3, 70, &q1, &w1, &m1);
        double q2; float w2; int m2; run_rsc(6.5f, 6.5f, 0.0f, 3, 70, &q2, &w2, &m2);     // 6.5 held: the soft brake
        g_rsc_bend = 0;
        printf("one bend: off -> level %d; on -> 5 m/s2 held level %d, 6.5 m/s2 held level %d (brake %.2f)\n", m0, m1, m2, w2);
        bad |= !(m0 == 0 && m1 == 1 && m2 >= 2 && w2 > 0.1f);
        printf("rollover guard: growing sway level %d, soft brake from %.1f s (brake %.2f), lane changes level %d, roundabout level %.0f\n", lm, l2, bm, lmb, lvl_normal);
        bad |= !(sway_ok && calm_ok && lvl_normal == 0);
        T.yaw = 0; T.speed = 0; rsc_step(1 / 60.0f);   // under 40 km/h the guard resets itself: level 0, no brake wish, no throttle cap
    }
    T.speed_limit = 0;
    printf("keys test: %s\n", bad ? "FAIL" : "ok");
    if (bad) exit(1);
}


// ======================================================================= junction / roundabout cancel (cruise_body)
// The cruise is on in front of a roundabout or a junction the route reading knows (25 km/h); the speed is scripted (kmh_of), behind a
// lead the game's ACC target is that speed. Returns the distance (m) at which the cruise cancelled itself, -1 when it stayed on.
static float jn_run(const char *name, int roundabout, float set_kmh, float d0, double secs, float (*kmh_of)(double), int lead, float *kmh_at)
{
    memset(GA, 0, sizeof GA); memset(&C, 0, sizeof C);
    o_cruise = stock_cruise; stock_thr = 0;
    p_smooth = 1; g_low_ok = 1; g_sg_ok = 1; g_sg_hold = 0; g_feed = 0; g_stopped_at = 0; s_cb_was_on = 0; s_jn_cancel_at = -1e9; g_rb_cancel_at = -1e9;
    p_ah = 0; p_hill = 0; g_radar_ok = 0; p_own_acc = 0;
    g_route = 1; g_road = road_flat; g_s = 0;
    T.engine_on = 1; T.gear = 5; T.dgear = 5; T.eff_clutch = 0; T.in_thr = 0; T.in_brake = 0; T.paused = 0; T.parking_brake = 0; T.in_steer = 0;
    T.place.orientation.pitch = 0; T.accel = 0; T.air_psi = 120; T.air_warning = 0;
    *(unsigned char *)(GA + GA_ON) = 1; *(float *)(GA + GA_SET) = set_kmh;
    g_sim_jn.dist_m = d0; g_sim_jn.turn_deg = roundabout ? 0.0f : 90.0f; g_sim_jn.limit_ms = 25 / 3.6f; g_sim_jn.ratio = roundabout ? 2.0f : 1.0f;
    g_sim_jn.roundabout = roundabout; g_sim_jn.curves = roundabout ? 2 : 1;
    double t0 = t_now; float cancel_m = -1, kmh = 0; *kmh_at = -1; int held = 0;
    for (; t_now - t0 < secs; t_now += 1 / 60.0) {
        kmh = kmh_of(t_now - t0);
        T.speed = kmh / 3.6f; T.cruise = set_kmh / 3.6f;
        *(float *)(GA + GA_LEAD_M) = lead ? 15 + 1.0f * kmh : 0;   // the lead keeps 15 m at a standstill, more at speed
        *(float *)(GA + GA_ACC_TARGET) = lead ? kmh : set_kmh;
        *(float *)(GA + GA_BRAKE_REQ) = 0;
        vehicle_frame();
        act_stub(GA, 1 / 60.0f, 0);
        if (g_sg_hold) held++;
        if (!*(unsigned char *)(GA + GA_ON)) { cancel_m = g_sim_jn.dist_m; *kmh_at = kmh; break; }
        g_sim_jn.dist_m = fmaxf(0.5f, g_sim_jn.dist_m - kmh / 3.6f / 60);   // standing: the entrance stays where it is
    }
    if (cancel_m >= 0) printf("%-78s cancelled %.0f m before the %s at %.0f km/h after %.1f s (Stop&Go held %.1f s)%s\n", name, cancel_m,
                              roundabout ? "roundabout" : "junction", kmh, t_now - t0, held / 60.0, roundabout && g_rb_cancel_at < t_now - 1 ? "  <-- lane not told" : "");
    else printf("%-78s stays on (%.1f s, ends at %.0f km/h, Stop&Go held %.1f s)\n", name, secs, kmh, held / 60.0);
    g_sim_jn.dist_m = 0; g_route = 0; g_sg_hold = 0; g_feed = 0; g_stopped_at = 0; *(unsigned char *)(GA + GA_ON) = 0;
    return cancel_m;
}
static float k_lead20(double) { return 20; }                                               // following a lead at 20 km/h
static float k_queue(double t) { return t < 10 ? 0 : fminf(20.0f, (float)(t - 10) * 5); }   // a queue: 10 s standing, then the lead drives off (5 km/h per s)
static float k_crawl(double) { return 3; }                                                 // creeping at 3 km/h
static float k_50(double) { return 50; }
static void junction_test()
{
    int bad = 0; float k;
    printf("--- junction / roundabout cancel (the cruise cancels itself a few seconds before a roundabout, also behind a vehicle)\n");
    float a = jn_run("roundabout, ACC behind a lead at 20 km/h (set 50), from 120 m", 1, 50, 120, 40, k_lead20, 1, &k);
    bad |= !(a > 35 && a < 50);                                           // junction_cancel_s 5 s at 20 km/h: ~43 m before the entrance (the old rule needed > 28 km/h)
    float b = jn_run("roundabout, queue: 10 s standing 15 m before the entrance behind a lead, then it drives off", 1, 50, 15, 40, k_queue, 1, &k);
    bad |= !(b >= 0 && k >= 5 && k < 7);                                 // not while standing (Stop&Go holds), cancelled as soon as it drives (5 km/h)
    float c = jn_run("roundabout, crawling at 3 km/h 15 m before the entrance, no lead", 1, 50, 15, 8, k_crawl, 0, &k);
    bad |= c >= 0;                                                        // under 5 km/h: never
    float d = jn_run("junction (90 deg), ACC behind a lead at 20 km/h (set 50), from 120 m", 0, 50, 120, 40, k_lead20, 1, &k);
    bad |= d >= 0;                                                        // junctions keep the old rule: only above the junction speed + 3
    float e = jn_run("roundabout, 50 km/h and nobody ahead, from 200 m (as before: handed over at 30 m)", 1, 50, 200, 40, k_50, 0, &k);
    bad |= !(e > 29 && e < 31);
    float f = jn_run("junction (90 deg), 50 km/h and nobody ahead, from 200 m (as before: handed over at 30 m)", 0, 50, 200, 40, k_50, 0, &k);
    bad |= !(f > 29 && f < 31);
    printf("junction cancel test: %s\n", bad ? "FAIL" : "ok");
    if (bad) exit(1);
}

// ======================================================================= session reset (cr_reset): nothing timed or filtered carries over
// The body's timers and filters that used to be function-local statics (and so lived through a teleport or a load) are module state now.
static void reset_test()
{
    int bad = 0;
    memset(GA, 0, sizeof GA); g_low_ok = 1;
    s_steer_until = s_jn_ignore_until = t_now + 5; s_stuck_since = t_now - 1; s_acc_f = 3; s_acc_init = 1; s_desc_braking = 1;
    s_cr_prev = 40; s_cr_rate = -2; s_cr_since = t_now - 1;
    cr_reset(RESET_SOFT, "sim");
    int soft_ok = s_steer_until == 0 && s_jn_ignore_until == 0 && s_stuck_since == 0 && s_acc_init == 0 && s_desc_braking == 0 &&
                  s_cr_prev != s_cr_prev && s_cr_rate != s_cr_rate && s_cr_since < 0;
    printf("reset SOFT: steer / junction-ignore / stuck timers, acceleration filter, descent hysteresis, crossing filter cleared: %s\n", soft_ok ? "yes" : "NO");
    bad |= !soft_ok;
    // the pedal curve blend after a mode change: a SOFT reset (pause menu) keeps a blend that is under way, a HARD one (other truck, teleport) drops it
    float se = p_pc_eco, sp = p_pc_pw, st = p_pc_travel; p_pc_eco = 1.35f; p_pc_pw = 1.6f; p_pc_travel = 1.0f;
    s_pb.from = s_pb.to = -1; s_pb.travel = 0; s_pb.prev = -1; s_pb.b = 1;
    pedal_blend(MODE_ECO, 0.5f);
    float held = pedal_blend(MODE_POWER, 0.5f);        // switched to Power, pedal held: still the Eco curve (0.39)
    cr_reset(RESET_SOFT, "sim");
    float after_soft = pedal_blend(MODE_POWER, 0.5f);  // still blending: Eco
    cr_reset(RESET_HARD, "sim");
    float after_hard = pedal_blend(MODE_POWER, 0.5f);  // no blend in progress any more: the Power curve (0.67)
    printf("reset and the pedal blend: held %.2f, after SOFT %.2f (both 0.39, the blend goes on), after HARD %.2f (0.67, Power at once)\n", held, after_soft, after_hard);
    bad |= fabsf(held - 0.39f) > 0.01f || fabsf(after_soft - 0.39f) > 0.01f || fabsf(after_hard - 0.67f) > 0.01f;
    p_pc_eco = se; p_pc_pw = sp; p_pc_travel = st;
    s_pb.from = s_pb.to = -1; s_pb.travel = 0; s_pb.prev = -1; s_pb.b = 1;
    printf("reset test: %s\n", bad ? "FAIL" : "ok");
    if (bad) exit(1);
}

// Own speed limiter on a flat road, the driver at full pedal: the speedometer settles at the configured value (with the bias), not under it.
// Then the cruise set speed (own_limit_set): it stops at the limit itself, the bias is only in the throttle fade.
// The fade settles 1.5 km/h x (share of the throttle that holding the speed needs) under the limit + bias: the model of run() needs 0.2 (rolling 0.006),
// the heavy case (soft ground, rolling 0.02, weaker engine) 0.63, where 0.5 does not bring it all the way up (informational).
static float limiter_run(float limit, float bias, float roll = 0.006f, float fmax_n = 20000)
{
    memset(GA, 0, sizeof GA); memset(&T, 0, sizeof T); T.air_psi = 120; T.gear = 1; T.engine_on = 1; T.fwd_gears = 12;
    float sl = p_own_lim, sb = p_own_bias; p_own_lim = limit; p_own_bias = bias; g_low_ok = 1; o_cruise = stock_cruise;
    double v = 60 / 3.6, a_f = 0;
    for (int k = 0; k < 60 * 60; k++, t_now += 1 / 60.0) {
        T.speed = (float)v; T.accel = (float)a_f; T.in_thr = 1; vehicle_frame();
        float u = act_stub(GA, 1 / 60.0f, 1.0f);
        double a = (u * fmax_n - (26000 * 9.81 * roll + 0.5 * 1.2 * 6 * v * v)) / 26000;
        v = fmax(0, v + a / 60);
        a_f += (a - a_f) * fmin(1.0, (1 / 60.0) / 0.15);
    }
    p_own_lim = sl; p_own_bias = sb;
    return (float)(v * 3.6);
}
static void limiter_test()
{
    float with = limiter_run(95, 0.5f), without = limiter_run(95, 0), off = limiter_run(0, 0.5f), off0 = limiter_run(0, 0);
    float h_with = limiter_run(95, 0.5f, 0.02f, 12000), h_without = limiter_run(95, 0, 0.02f, 12000);
    printf("own limiter, flat, full pedal, 60 s: bias 0.5 -> %.2f km/h, bias 0 -> %.2f km/h (the old value, under 95); limiter off -> %.2f with bias 0.5, %.2f with bias 0 (same)\n", with, without, off, off0);
    printf("   heavy truck on soft ground (informational): bias 0.5 -> %.2f km/h, bias 0 -> %.2f km/h\n", h_with, h_without);
    int bad = fabsf(with - 95) > 0.3f || !(without < with) || off != off0;
    printf("own limiter bias test: %s\n", bad ? "FAIL" : "ok");
    if (bad) exit(1);
    // the cruise set speed: clamped at the limit itself, never at limit + bias (95.5 showed as 96 on the dash)
    float sl = p_own_lim, sb = p_own_bias; p_own_lim = 95; p_own_bias = 0.5f;
    const float sets[] = { 90, 95, 95.4f, 95.5f, 96, 100, 130 };
    printf("own limiter 95, bias 0.5, cruise set speed:");
    for (float st : sets) {
        memset(GA, 0, sizeof GA); *(unsigned char *)(GA + GA_ON) = 1; *(float *)(GA + GA_SET) = st;
        own_limit_set(GA);
        float r = *(float *)(GA + GA_SET);
        printf(" %.1f -> %.1f", st, r);
        if (r > p_own_lim || r != fminf(st, p_own_lim)) bad = 1;
    }
    memset(GA, 0, sizeof GA); *(float *)(GA + GA_SET) = 100; own_limit_set(GA);   // cruise off: left alone
    if (*(float *)(GA + GA_SET) != 100) bad = 1;
    p_own_lim = sl; p_own_bias = sb;
    printf("; cruise off: %.0f\nown limiter set speed test: %s\n", *(float *)(GA + GA_SET), bad ? "FAIL" : "ok");
    if (bad) exit(1);
}

// ======================================================================= Auto Hold
// Truck stops with the brake, the driver lets go, waits, then launches (gentle / hard). Signed speed, static brake,
// engine with a clutch that bites 0.15 s after the pedal (box engages when the throttle gate lets the pedal through)
// and then closes in 0.35 s. The game's gate (neutral-exit-re notes): merged throttle 0 while the brake request > 0,
// unless our patch passes it (s_thr_pass). Brake request slews +-5/s like the game.
static float g_true_k = 1.0f;   // the engine really delivers this fraction of what the module thinks (model error)
struct ahres { float roll_m, jerk, max_a, req_end, t_move, req_max_after, t_jerk; int held_ok; };
static ahres ah_run(const char *name, float grade_pct, int hard, int gate, float est_k, int old_hill_start, int verbose_row)
{
    memset(GA, 0, sizeof GA);
    memset(&AH, 0, sizeof AH);
    g_low_ok = 1; g_sg_hold = 0; g_crawl_on = 0; s_gate_ok = gate; s_thr_pass = 0; s_req_ours = 0; s_want_hold = s_want_hill = 0;
    act_brake(ACT_HOLD, 0); act_brake(ACT_HILL, 0);
    p_ah = old_hill_start ? 0 : 1; p_hill = old_hill_start == 2 ? 0 : 1; p_hill_pct = 3; p_hill_s = 3;
    g_sim_eng.torque = 2400; g_sim_eng.n_curve = 0; g_sim_eng.rpm_limit = 2200;
    g_sim_mass = 26000 / est_k;                      // est_k: the module thinks the truck is lighter / heavier than it is
    const float m = 26000, ratio = 12.0f, diff = 2.64f, R = 0.5f;
    memset(T.ratio, 0, sizeof T.ratio); T.ratio[0] = ratio; T.diff = diff; T.fwd_gears = 12; T.wheels = 1;
    T.engine_on = 1; T.parking_brake = 0; T.paused = 0; T.dgear = 0; T.gear = 0; T.eff_thr = 0; T.eff_clutch = 1; T.rpm = 700; T.cruise = 0;
    o_cruise = stock_cruise; p_smooth = 1; *(unsigned char *)(GA + GA_ON) = 0;
    float gr = grade_pct / 100, sing = gr / sqrtf(1 + gr * gr);
    T.place.orientation.pitch = atanf(gr) / 6.2831853f;
    double v = 5, x = 0, t0 = t_now; float brake_applied = 0, thr = 0, a_f = 0, a_prev = 0, rpm = 700, clutch = 1;
    double t_stop = -1, t_launch = -1, t_gear = -1, x_stop = 0, minx = 1e9, t_mv = -1;
    ahres r = {};
    const double T_REL = 3.0, T_LAUNCH = 7.0;     // brake released 3 s after the stop is detected... launch 4 s later
    for (int k = 0; t_now - t0 < 16; k++, t_now += 1 / 60.0) {
        double t = t_now - t0, dt = 1 / 60.0;
        if (t_stop < 0 && v <= 0.0001 && t > 1) { t_stop = t; x_stop = x; }
        // driver: brake 0.5 until the stop + 1.5 s, then off; launch T s later
        float ped_brake = (t_stop < 0 || t < t_stop + 1.5) ? 0.5f : 0.0f;
        if (t_stop >= 0 && t_launch < 0 && t > t_stop + 4.0) t_launch = t;
        float tl = t_launch < 0 ? -1 : (float)(t - t_launch);
        thr = tl < 0 ? 0 : hard ? fminf(1.0f, tl / 0.2f) : fminf(0.35f, 0.35f * tl / 1.0f);
        T.in_brake = ped_brake; T.in_thr = thr; T.speed = (float)v;
        // gearbox / clutch: the pedal reaches the box only with the gate open
        float req_pre = *(float *)(GA + GA_BRAKE_REQ);
        // (the gate is evaluated by the game right after our stub; use last tick's flag for the engine this tick)
        int gate_open = req_pre <= 0 || s_thr_pass;
        T.eff_thr = gate_open ? thr : 0;
        if (T.eff_thr > 0.01f && t_gear < 0) t_gear = t;
        if (T.eff_thr <= 0.01f && v < 0.1) { t_gear = -1; }
        T.gear = (t_gear >= 0 && t - t_gear > 0.15) ? 1 : 0;
        float clutch_t = T.gear == 1 ? fmaxf(0, 1 - (float)(t - t_gear - 0.15) / 0.35f) : 1;
        clutch = clutch_t; T.eff_clutch = clutch;
        rpm += (700 + 1300 * T.eff_thr - rpm) * (float)(dt / 0.25); T.rpm = rpm;
        *(float *)(GA + GA_BRAKE_REQ) = 0;                 // AEB option off would leave it; our writer puts back what it wrote
        if (s_req_ours > 0) *(float *)(GA + GA_BRAKE_REQ) = s_req_ours;   // (what we left there last tick)
        vehicle_frame();
        float u = act_stub(GA, (float)dt, 0);
        float req = *(float *)(GA + GA_BRAKE_REQ);
        // the game's throttle gate now (this tick's flag)
        if (!(req <= 0 || s_thr_pass)) T.eff_thr = 0;
        // brake slews to max(request, pedal)
        float want_b = fmaxf(req, ped_brake);
        brake_applied += fmaxf(-5 * (float)dt, fminf(5 * (float)dt, want_b - brake_applied));
        float Fb = brake_applied * m * 4;
        float couple = 1 - clutch;
        float te = g_sim_eng.torque * curve_factor(&g_sim_eng, T.rpm) * T.eff_thr * g_true_k;
        float Fd = T.gear == 1 ? te * ratio * diff * 0.9f * couple / R : 0;
        float Fg = m * 9.81f * sing + (v > 1e-4 ? m * 9.81f * 0.006f : v < -1e-4 ? -m * 9.81f * 0.006f : 0);
        float Fnet = Fd - Fg;                             // forward positive; rolling resistance against the motion
        if (fabs(v) < 1e-4 && fabsf(Fnet) <= Fb) { v = 0; Fnet = 0; }
        else {
            float Fres = Fb * (v >= 0 ? 1.0f : -1.0f);
            if (fabs(v) < 1e-4) Fres = Fb * (Fnet > 0 ? 1.0f : -1.0f);
            double vn = v + (Fnet - Fres) / m * dt;
            if (v != 0 && ((v > 0) != (vn > 0)) && Fb > 0 && fabsf(Fnet) <= Fb) vn = 0;   // brake stops it at zero
            if (v > 0 && t_stop < 0 && vn < 0) vn = 0;
            v = vn;
        }
        x += v * dt;
        if (t_stop >= 0) minx = fmin(minx, x);
        a_f = (float)((v - (double)T.speed) / dt);
        float a_s = a_prev + (a_f - a_prev) * (float)(dt / (0.1 + dt));    // 0.1 s smoothing
        if (t_launch >= 0 && t - t_launch < 4) { float jk = fabsf(a_s - a_prev) / (float)dt; if (jk > r.jerk) { r.jerk = jk; r.t_jerk = (float)(t - t_launch); } r.max_a = fmaxf(r.max_a, a_s); }
        a_prev = a_s;
        if (t_launch >= 0 && v > 0.1 && t_mv < 0) t_mv = t - t_launch;
        if (t_launch >= 0 && t - t_launch > 1.5) r.req_max_after = fmaxf(r.req_max_after, req);
        if (verbose_row && k % 2 == 0 && t_stop >= 0 && t > t_stop + 3.5 && t < t_stop + 7) printf("   t=%5.2f thr %.2f clutch %.2f drive %6.0f N  req %.2f  brake %.2f  v %.3f km/h x %.3f\n", t - t_stop, thr, clutch, Fd, req, brake_applied, v * 3.6, x - x_stop);
    }
    // stop point: roll back below the stop position while the truck was meant to be standing
    r.roll_m = t_stop < 0 ? 99 : (float)(x_stop - minx);
    r.req_end = *(float *)(GA + GA_BRAKE_REQ);
    r.t_move = (float)t_mv;
    printf("%-44s roll back %.3f m, launch jerk %.1f m/s3 (at %.2f s), peak accel %.1f m/s2, moves after %.2f s, request after 1.5 s %.2f, at end %.2f%s\n", name,
           r.roll_m, r.jerk, r.t_jerk, r.max_a, r.t_move, r.req_max_after, r.req_end, r.req_end == 0 ? "" : "  <-- NOT 0");
    return r;
}

// A stop by the lever: the retarder (F/m: step 2 of 4 gives 1.6 m/s2 at 18 t, built up over 0.4 s, faded out between 6 and 4.5
// km/h), the box in neutral under 5.2 km/h, the service brake gain m/s2 per request 1.0 (4) behind a 0.15 s lag, rolling resistance
// 0.006. modules/decel.cpp brakes in its frame, Cruise and the actuator in the game tick (both 60 Hz). The driver puts the lever
// back 2 s after the stop and gives gas 0.35 in 1 s 4 s after it (the engine and the gate as in ah_run).
// held_min: the lowest request from the stop to the gas; roll_m: the most the truck moved either way in that time;
// req_after / hold_after: the highest request, and Auto Hold's own, 1-4 s after the stop (the assist brakes a roll back by itself).
// drop: the most the request fell under its peak from 0.5 km/h on until the gas (the hand-over from the assist to the hold).
struct lvres { double t_stop; float held_min, roll_m, req_end, t_move, req_after, hold_after, drop; };
static double g_lv_prime = 0;   // > 0: [core] brake_learn on, the vehicle model's estimate primed to this share of the lever plant's gain
static lvres lever_run(const char *name, float grade_pct, float mass_t, int ah, int lever_key, int engine_only, int owner_late, float gain = 4)
{
    memset(GA, 0, sizeof GA);
    memset(&AH, 0, sizeof AH);
    g_low_ok = 1; g_sg_hold = 0; g_crawl_on = 0; s_gate_ok = 1; s_thr_pass = 0; s_req_ours = 0; s_want_hold = s_want_hill = 0;
    for (int o = 0; o < ACT_N; o++) act_brake(o, 0);
    s_dec_was = 0; s_aeb_was = s_hill_was = s_hold_was = 0; s_br_at = t_now;
    p_ah = (float)ah; p_ah_lever = (float)lever_key; p_hill = 1; p_hill_pct = 3; p_hill_s = 3; s_ah_k = 1;
    for (auto &q : dc::g_params) *q.val = q.def;
    dc::p_enabled = 1; dc::D = decltype(dc::D)();
    int hooked = s_hooked; s_hooked = 1;
    s_low_on = owner_late; s_low_stopped = 0;
    g_sim_eng.torque = 2400; g_sim_eng.n_curve = 0; g_sim_eng.rpm_limit = 2200;
    const float m = mass_t * 1000, ratio = 12.0f, diff = 2.64f, R = 0.5f;
    g_sim_mass = m;
    g_brake_learn = g_lv_prime > 0 ? 1.0f : 0.0f;
    if (g_lv_prime > 0) vehicle_brake_prime(0, (float)(g_lv_prime * gain * m), 0.02f, 8);
    memset(T.ratio, 0, sizeof T.ratio); T.ratio[0] = ratio; T.diff = diff; T.fwd_gears = 12; T.wheels = 1;
    T.engine_on = 1; T.parking_brake = 0; T.paused = 0; T.dgear = 0; T.gear = 3; T.eff_thr = 0; T.eff_clutch = 0; T.rpm = 900; T.cruise = 0;
    T.air_warning = 0; T.in_brake = 0; T.in_thr = 0;
    T.retarder = engine_only ? 0 : 2; T.retarder_steps = engine_only ? 0 : 4; T.motor_brake = engine_only;
    o_cruise = stock_cruise; p_smooth = 1; *(unsigned char *)(GA + GA_ON) = 0;
    float gr = grade_pct / 100, sing = gr / sqrtf(1 + gr * gr);
    T.place.orientation.pitch = atanf(gr) / 6.2831853f;
    const float a_ret = (engine_only ? 0.6f : 1.6f) * 18 / mass_t;
    double v = 30 / 3.6, x = 0, t0 = t_now, t_stop = -1, t_launch = -1, t_gear = -1, x_stop = 0, t_slow = -1, t_mv = -1;
    float y = 0, thr = 0, rpm = 900, clutch = 0, xmin = 0, xmax = 0, peak = -1;
    lvres r = {}; r.t_stop = -1; r.held_min = 9; r.t_move = -1;
    for (int k = 0; t_now - t0 < 25; k++, t_now += 1 / 60.0) {
        double t = t_now - t0, dt = 1 / 60.0;
        if (t_stop < 0 && v <= 0.0001 && t > 1) { t_stop = t; x_stop = xmin = xmax = (float)x; }
        int lever = t_stop < 0 || t < t_stop + 2;
        if (!lever) { T.retarder = 0; T.motor_brake = 0; }
        if (t_stop >= 0 && t_launch < 0 && t > t_stop + 4) t_launch = t;
        float tl = t_launch < 0 ? -1 : (float)(t - t_launch);
        thr = tl < 0 ? 0 : fminf(0.35f, 0.35f * tl / 1.0f);
        T.in_thr = thr; T.speed = (float)v;
        // the owner's standstill (< 0.3 km/h) declared 0.15 s late
        if (fabs(v) * 3.6 < 0.3) { if (t_slow < 0) t_slow = t; } else t_slow = -1;
        s_low_stopped = t_slow >= 0 && t - t_slow >= 0.15;
        // the box: neutral under 5.2 km/h, gear 1 for the launch once the gate passes the pedal (as in ah_run)
        float req_pre = *(float *)(GA + GA_BRAKE_REQ);
        int gate_open = req_pre <= 0 || s_thr_pass;
        T.eff_thr = gate_open ? thr : 0;
        if (t_launch >= 0 && T.eff_thr > 0.01f && t_gear < 0) t_gear = t;
        if (t_launch >= 0) { T.gear = (t_gear >= 0 && t - t_gear > 0.15) ? 1 : 0; clutch = T.gear == 1 ? fmaxf(0, 1 - (float)(t - t_gear - 0.15) / 0.35f) : 1; }
        else { T.gear = v * 3.6 > 5.2 ? 3 : 0; clutch = T.gear ? 0 : 1; }
        T.eff_clutch = clutch;
        rpm += (700 + 1300 * T.eff_thr - rpm) * (float)(dt / 0.25); T.rpm = rpm;
        *(float *)(GA + GA_BRAKE_REQ) = 0;
        if (s_req_ours > 0) *(float *)(GA + GA_BRAKE_REQ) = s_req_ours;
        vehicle_frame();
        dc::de_frame();
        act_stub(GA, (float)dt, 0);
        float req = *(float *)(GA + GA_BRAKE_REQ);
        if (!(req <= 0 || s_thr_pass)) T.eff_thr = 0;
        y += (req - y) * (float)(dt / (0.15 + dt));
        float kmh = (float)fabs(v) * 3.6f;
        float fade = kmh >= 6 ? 1 : kmh <= 4.5f ? 0 : (kmh - 4.5f) / 1.5f;
        float a_r = lever ? a_ret * fminf(1, (float)t / 0.4f) * fade : 0;
        float Fb = y * m * gain + (v > 1e-4 ? a_r * m : 0);
        float te = g_sim_eng.torque * curve_factor(&g_sim_eng, T.rpm) * T.eff_thr;
        float Fd = T.gear == 1 ? te * ratio * diff * 0.9f * (1 - clutch) / R : 0;
        float Fg = m * 9.81f * sing + (v > 1e-4 ? m * 9.81f * 0.006f : v < -1e-4 ? -m * 9.81f * 0.006f : 0);
        float Fnet = Fd - Fg;
        if (fabs(v) < 1e-4 && fabsf(Fnet) <= y * m * gain) v = 0;
        else {
            float Fres = Fb * (v >= 0 ? 1.0f : -1.0f);
            if (fabs(v) < 1e-4) Fres = y * m * gain * (Fnet > 0 ? 1.0f : -1.0f);
            double vn = v + (Fnet - Fres) / m * dt;
            if (v != 0 && ((v > 0) != (vn > 0)) && fabsf(Fnet) <= Fb) vn = 0;   // the brake stops it at zero
            v = vn;
        }
        x += v * dt;
        if (t_launch < 0 && (peak >= 0 || fabs(v) * 3.6 < 0.5)) { peak = fmaxf(peak, req); r.drop = fmaxf(r.drop, peak - req); }
        if (t_stop >= 0 && t_launch < 0) {
            xmin = fminf(xmin, (float)x); xmax = fmaxf(xmax, (float)x);
            r.held_min = fminf(r.held_min, req);
            if (t > t_stop + 1 && t < t_stop + 4) { r.req_after = fmaxf(r.req_after, req); r.hold_after = fmaxf(r.hold_after, s_want_hold); }
        }
        if (t_launch >= 0 && v > 0.1 && t_mv < 0) t_mv = t - t_launch;
        if (verbose && k % 3 == 0 && (t_stop < 0 ? v * 3.6 < 1.5 : t < t_stop + 0.6))
            printf("   t=%6.2f v %6.3f km/h  decel %.3f  hold %.3f  req %.3f  brake %.3f\n", t, v * 3.6, dc::D.u, s_want_hold, req, y);
    }
    s_hooked = hooked; s_low_on = 0; s_low_stopped = 0; p_ah = 1; p_ah_lever = 1; g_brake_learn = 0;
    r.t_stop = t_stop;
    r.roll_m = t_stop < 0 ? 99 : fmaxf(x_stop - xmin, xmax - x_stop);
    r.req_end = *(float *)(GA + GA_BRAKE_REQ);
    r.t_move = (float)t_mv;
    if (t_stop < 0) r.held_min = 0;
    printf("%-52s stops after %5.2f s, request from the stop to the gas: lowest %.2f, 1-4 s after the stop %.2f (Auto Hold %.2f), fell %.2f; moved %.3f m; moves %.2f s after the gas, request at end %.2f\n",
           name, t_stop, r.held_min, r.req_after, r.hold_after, r.drop, r.roll_m, r.t_move, r.req_end);
    return r;
}

// ======================================================================= ACC with a lead (own ACC vs the game's target)
// A lead vehicle with a scripted speed; the game's ACC fn writes +0x1a8 (gap) and +0x1a0 (its exp target, cruise-re notes § 3)
// each tick before the cruise fn, the radar call is wrapped (radar_stub) so we see the lead's speed. Braking as in the pedal
// fn: retarder demand clamp(v - A, 0, 1) (A = target + 0.25) = 0.7 m/s2 at most, service 0.15 max = 0.6 m/s2, and the ACC's
// emergency part (gap < D/2: brake (D/2 - gap)/(D/2), up to 6 m/s2).
static float g_fake_radar[5];
static float *fake_radar(void *, float *) { return g_fake_radar; }
struct accres { float min_gap, peak_dec, max_jerk, emerg_s, min_kmh, end_kmh, mean_gap_err; int same; };
static accres acc_run(const char *name, int own, int scen)
{
    memset(GA, 0, sizeof GA); memset(&C, 0, sizeof C); memset(&OA, 0, sizeof OA); memset(&s_radar, 0, sizeof s_radar);
    p_smooth = 1; p_own_acc = own; g_radar_ok = 1; o_radar = fake_radar; g_low_ok = 1; g_level = 1; g_sg_ok = 0; p_ah = 0; p_hill = 0;
    o_cruise = stock_cruise; stock_thr = 0;
    T.engine_on = 1; T.gear = 8; T.eff_clutch = 0; T.parking_brake = 0;
    const float m = 26000, set = 90;
    double t0 = t_now, x = 0, v = scen == 6 ? 40 / 3.6 : scen == 7 ? 90 / 3.6 : 80 / 3.6, lx = 0, lv = 0, jerk_prev_a = 0, a_prev = 0;
    float set_kmh = scen == 6 ? 40 : set;
    *(unsigned char *)(GA + GA_ON) = 1; *(float *)(GA + GA_SET) = set_kmh;
    accres r = { 1e9f, 0, 0, 0, 1e9f, 0, 0, 0 };
    int lead_on = 0; float stock_acc_prev = 0;
    double sum_err = 0; int nerr = 0;
    for (; t_now - t0 < 60; t_now += 1 / 60.0) {
        double t = t_now - t0, dt = 1 / 60.0;
        // scripts
        if (scen == 0) { if (!lead_on && t >= 10) { lead_on = 1; lx = x + 28; lv = 60 / 3.6; } }                  // cut-in, slower, 28 m ahead
        else if (scen == 1) { if (!lead_on && t >= 10) { lead_on = 1; lx = x + 30; lv = 86 / 3.6; } }              // cut-in, faster than us, still under the set speed, 30 m ahead
        else if (scen == 2) {                                                                                       // lead in front, brakes hard
            if (!lead_on) { lead_on = 1; lx = x + 50; lv = 80 / 3.6; }
            if (t >= 15 && lv > 30 / 3.6) lv = fmax(30 / 3.6, lv - 3.0 * dt);
        }
        else if (scen == 3) { if (!lead_on && t >= 10) { lead_on = 1; lx = x + 70; lv = 70 / 3.6; } }             // far lead, slower: follow up smoothly
        else if (scen == 4) { if (!lead_on && t >= 10) { lead_on = 1; lx = x + 99; lv = 25 / 3.6; } }             // ts4o: closing fast on a slow lead from radar range
        else if (scen == 5) {                                                                                       // ts4p: cut-in at 27 m, 60 km/h and braking to 45
            if (!lead_on && t >= 10) { lead_on = 1; lx = x + 27; lv = 60 / 3.6; v = 69 / 3.6; }
            if (lead_on && lv > 45 / 3.6) lv = fmax(45 / 3.6, lv - 1.5 * dt);
        }
        else if (scen == 7) {                                                                                       // ride 12-10 20:54: 92 m behind a lead at 49 km/h that brakes to 33 (1.1 m/s2) while we come in at 90
            if (!lead_on && t >= 10) { lead_on = 1; lx = x + 92; lv = 49 / 3.6; }
            if (lead_on && t >= 11.5 && lv > 33 / 3.6) lv = fmax(33 / 3.6, lv - 1.1 * dt);
        }
        else if (scen == 6) {                                                                                       // a car crossing 45 m ahead (roundabout): 25 km/h across, gone after 3 s
            if (t >= 10 && t < 12 && !lead_on) { lead_on = 1; lx = x + 40; lv = 0; }
            if (t >= 12) lead_on = 0;
        }
        if (lead_on) lx += lv * dt;
        float gap = lead_on ? (float)(lx - x) : 0;
        T.speed = (float)v; T.cruise = set_kmh / 3.6f; T.accel = (float)a_prev;
        float D = fmaxf(4, 2.0f * (float)v);
        // game's ACC fn
        float acc_t = set_kmh;
        float lead_seen = 0;
        if (lead_on && gap > 0 && gap < 100) {
            lead_seen = gap;
            float e_gap = gap - D;
            if (lv * 3.6f >= set_kmh) acc_t = set_kmh;
            else {
                float a_ = 132840.0f / m;
                acc_t = lv * 3.6f + fminf(expf(fabsf(e_gap) * 2.302585f / a_), 10.0f) * e_gap;
                acc_t = acc_t < 0 ? 0 : acc_t > set_kmh ? set_kmh : acc_t;
            }
            g_fake_radar[0] = gap; g_fake_radar[1] = scen == 6 ? 25 / 3.6f : (float)lv;  // crossing: the radar reports its absolute speed
        } else { g_fake_radar[0] = 3.4e38f; g_fake_radar[1] = 0; }
        g_fake_radar[2] = 3.4e38f; g_fake_radar[3] = 0; g_fake_radar[4] = 0;
        radar_stub(GA, g_fake_radar);
        *(float *)(GA + GA_LEAD_M) = lead_seen;
        *(float *)(GA + GA_ACC_TARGET) = acc_t;
        float game_target = acc_t;
        *(float *)(GA + GA_BRAKE_REQ) = 0;
        vehicle_frame();
        float u = act_stub(GA, (float)dt, 0);
        float T_used = *(float *)(GA + GA_ACC_TARGET);
        // brakes (pedal fn): overspeed vs the target the game now reads (+0x1a0), skipped while the driver throttle is on
        float kmh = (float)v * 3.6f, Aov = T_used + 0.25f;
        float dec = 0;
        float ov = kmh - Aov;
        if (ov > 0) dec += fminf(ov, 1.0f) * 0.7f + fminf(ov * 0.15f, 0.15f) * 4.0f;
        float emerg = 0;
        dec += *(float *)(GA + GA_BRAKE_REQ) * 7.0f;   // our brake request (1.0 ~ 7 m/s2)
        if (!s_aeb_owns && lead_seen > 0 && lead_seen < D * 0.5f) { emerg = fminf(1, (D * 0.5f - lead_seen) / (D * 0.5f)); dec += emerg * 6.0f; r.emerg_s += (float)dt; }
        if (dec > 0) u = 0;
        float F = u * (kmh > 30 ? 20000.0f : 40000.0f);
        float res = m * 9.81f * 0.006f + 0.5f * 1.2f * 6 * (float)(v * v);
        double a = (F - res) / m - dec;
        v = fmax(0, v + a * dt);
        x += v * dt;
        if (lead_on && t > 10) {
            float gp = (float)(lx - x);
            r.min_gap = fminf(r.min_gap, gp);
            sum_err += fabsf(gp - D) / D; nerr++;
        }
        if (t > 5) {
            r.peak_dec = fmaxf(r.peak_dec, (float)-a);
            r.min_kmh = fminf(r.min_kmh, (float)(v * 3.6));
            if (t > 6) r.max_jerk = fmaxf(r.max_jerk, fabsf((float)a - a_prev) / (float)dt * 0 + 0);   // (jerk from the smoothed a below)
        }
        double a_s = a_prev + (a - a_prev) * (dt / (0.3 + dt));
        if (t > 6) r.max_jerk = fmaxf(r.max_jerk, (float)(fabs(a_s - jerk_prev_a) / dt));
        jerk_prev_a = a_s; a_prev = a_s;
        stock_acc_prev = game_target;
    }
    r.end_kmh = (float)(v * 3.6);
    r.mean_gap_err = nerr ? (float)(sum_err / nerr) : 0;
    printf("%-52s %s min gap %5.1f m, peak decel %.2f m/s2, jerk %.2f m/s3, emergency %.1f s, min %.0f / end %.0f km/h, |gap-D|/D %.2f\n", name, own ? "ours" : "game", r.min_gap, r.peak_dec, r.max_jerk, r.emerg_s,
           r.min_kmh, r.end_kmh, r.mean_gap_err);
    return r;
}


// ======================================================================= own AEB (aeb notes), cruise off unless said
// Brakes: the game slews +0x17c at 5/s; request r gives 0.4 + 10 r / (mass / 30 t) m/s2 (ts4* logs, 26-30 t); the driver's
// brake pedal 8 m/s2 at 1.0; throttle 20 kN at 1.0 (zeroed while the request is > 0). Air: every rise of the applied brake
// costs 60 psi per unit, the compressor gives 4 psi/s (rough; only for "one ramp vs pulses").
struct aebres { float min_gap, peak_dec, end_kmh, warn_lead_s, psi_drop, max_req, after_kd; int max_st, flips, hit; float warn_gap, brake_gap, final_gap, hit_kmh; };
static double g_aeb_v0 = 80;   // our start speed (km/h)
static double g_plant_k = 1;   // the plant's brakes against the AEB's fit (1 = the fit: 10 m/s2 per request at 30 t and less)
static double g_prime_k = 0;   // > 0: [core] brake_learn on, the vehicle model's estimate primed to this plant's gain (the learned gain is then x this of the plant's: 1 = exact)
static aebres aeb_run(const char *name, int scen, float mass, float psi0)
{
    memset(GA, 0, sizeof GA); memset(&C, 0, sizeof C); memset(&OA, 0, sizeof OA); memset(&s_radar, 0, sizeof s_radar);
    memset(&AE, 0, sizeof AE); AE.gap_prev = AE.rate = AE.vl_prev = NAN; s_req_ours = 0; s_want_aeb = 0; s_aeb_was = 0; s_br_at = 0; s_air_low = 0;
    p_smooth = 1; p_own_acc = 1; g_radar_ok = 1; o_radar = fake_radar; g_low_ok = 1; g_level = 1; g_sg_ok = 0; p_ah = 0; p_hill = 0;
    o_cruise = stock_cruise; stock_thr = 0; s_aeb_owns = 1; g_sim_mass = mass;
    g_brake_learn = g_prime_k > 0 ? 1.0f : 0.0f;
    if (g_prime_k > 0) vehicle_brake_prime(0, (float)(g_prime_k * g_plant_k * 10 / fmax(1.0, mass / 30000.0) * mass), 0.02f, 8);
    T.engine_on = 1; T.gear = 8; T.dgear = 8; T.eff_clutch = 0; T.parking_brake = 0; T.in_brake = 0; T.in_thr = 0; T.air_warning = 0; T.paused = 0;
    double t0 = t_now, x = 0, v = g_aeb_v0 / 3.6, lx = 0, lv = 0, brk = 0, psi = psi0, a_f = 0;
    int lead_on = 0, last_dir = 0;
    float cross_v = 0;
    aebres r = { 1e9f, 0, 0, -1, 0, 0, 0, 0, 0, 0, -1, -1, -1, 0 };
    double t_warn = -1, t_part = -1;
    float pedal = 0.3f, pbrake = 0; int u_hold = 0;   // a driver holding 30 % throttle who does not see it
    if (scen == 2) v = 60 / 3.6;
    if (scen == 3) v = 40 / 3.6;
    if (scen == 17) v = 30 / 3.6;
    float prev_req = 0;
    for (; t_now - t0 < 40; t_now += 1 / 60.0) {
        double t = t_now - t0, dt = 1 / 60.0;
        switch (scen) {
        case 0: case 4: case 5: case 6: case 14: case 15: if (!lead_on) { lead_on = 1; lx = x + 200; lv = 0; } break;      // stopped queue 200 m ahead
        case 1: case 10: case 11: case 16: if (!lead_on) { lead_on = 1; lx = x + 150; lv = 40 / 3.6; } break;     // slow truck
        case 2: if (!lead_on && t >= 2) { lead_on = 1; lx = x + 20; lv = 30 / 3.6; }                     // cut-in at 20 m, 30 km/h, brakes to 0
                if (lead_on && t >= 3) lv = fmax(0, lv - 3 * dt); break;
        case 3: if (t >= 2 && t < 4) { lead_on = 1; lx = x + 25; lv = 0; cross_v = 10; } else { lead_on = 0; cross_v = 0; } break;  // crossing car
        case 7: if (!lead_on) { lead_on = 1; lx = x + 25; lv = 80 / 3.6; }                               // following 25 m, lead brakes 5 m/s2 to 0
                if (t >= 3) lv = fmax(0, lv - 5 * dt); break;
        case 12: if (!lead_on) { lead_on = 1; lx = x + 30; lv = 50 / 3.6; }                              // following 30 m at 50, lead brakes 5 m/s2 to 0
                 if (t >= 3) lv = fmax(0, lv - 5 * dt);
                 break;
        case 13: if (!lead_on) { lead_on = 1; lx = x + 150; lv = 60 / 3.6; } break;                       // far, slower lead (60 km/h at 150 m, we 80)
        case 8: case 9: if (!lead_on) { lead_on = 1; lx = x + (scen == 8 ? 12 : 7); lv = 80 / 3.6; }      // tailgating 12 / 7 m at 80, lead brakes 4 m/s2 to 0
                if (t >= 5) lv = fmax(0, lv - 4 * dt); break;
        case 17: if (!lead_on) { lead_on = 1; lx = x + 6; }                                               // town traffic: the lead weaves 28-33 km/h
                 lv = (30.5 + 2.5 * sin(2 * M_PI * t / 16)) / 3.6; break;
        }
        if (lead_on) lx += lv * dt;
        float gap = lead_on ? (float)(lx - x) : 0;
        if (lead_on && gap < 0.05f) { if (!r.hit) r.hit_kmh = (float)((v - lv) * 3.6); r.hit = 1; }
        // the driver
        static double kd_at; if (t == 0) kd_at = -1;
        if (scen == 4 && AE.st >= AEB_PARTIAL && kd_at < 0) { pedal = 0.9f; kd_at = t; }   // kick-down during partial braking ...
        if (scen == 4 && kd_at >= 0 && t > kd_at + 1) lead_on = 0;                          // ... and 1 s later it changed lanes: the lead is gone
        if (scen == 4 && kd_at >= 0 && t > kd_at + 0.4 && t < kd_at + 1) r.after_kd = fmaxf(r.after_kd, *(float *)(GA + GA_BRAKE_REQ));
        if (scen == 8 || scen == 9 || scen == 12 || scen == 14 || scen == 15) { pedal = 0; u_hold = 1; }
        // 13: the driver sees the slower lead and eases off with a gentle brake from 70 m, then holds its speed
        if (scen == 13) { pedal = 0.3f; if (lead_on && gap < 70 && v > lv + 0.3) { pedal = 0; pbrake = 0.15f; u_hold = 0; } else if (gap < 70) { pedal = 0; pbrake = 0; u_hold = 1; } }
        // 10: at the warning the driver indicates left and steers left; 1.5 s later the lead is beside us (gone from the radar)
        // 11: the same indicator, but no steering (it is no evasion: keeps braking)
        static double ev_at; if (t == 0) { ev_at = -1; T.in_steer = 0; T.yaw = 0; }
        // 16: as 10, but 0.6 s later he gives up (the lane is taken): straight again, indicator off; the lead stays ahead
        if ((scen == 10 || scen == 11 || scen == 15 || scen == 16) && AE.st >= AEB_WARN && ev_at < 0) ev_at = t;
        if ((scen == 10 || scen == 11 || scen == 15 || scen == 16) && ev_at >= 0) { GA[0x1d0] = 4; if (scen != 11) { T.in_steer = 0.03f; T.yaw = 0.006f; } }
        if (scen == 16 && ev_at >= 0 && t > ev_at + 0.6) { GA[0x1d0] = 0; T.in_steer = 0; T.yaw = 0; }
        if ((scen == 10 || scen == 15) && ev_at >= 0 && t > ev_at + 1.5) lead_on = 0;
        if ((scen == 10 || scen == 15) && ev_at >= 0 && t > ev_at + 0.8 && t < ev_at + 1.5) r.after_kd = fmaxf(r.after_kd, *(float *)(GA + GA_BRAKE_REQ));
        if (scen == 6) { pedal = 0; if (lead_on && gap < 105) pbrake = 0.5f; }         // the driver brakes in time by himself
        // 17: the driver follows on the throttle alone (lifting gives -0.4 m/s2), 6 m behind, from 15 s 3 m, from 28 s 1 m: any
        // brake request is the AEB's own
        if (scen == 17) {
            double want_gap = t < 15 ? 6 : t < 28 ? 3 : 1, a_des = 0.3 * (gap - want_gap) + 0.8 * (lv - v);
            pedal = (float)fmin(1, fmax(0, (a_des + 0.4) / (20000.0 / mass)));
        }
        T.in_thr = pedal; T.in_brake = pbrake;
        T.speed = (float)v; T.accel = (float)a_f; T.air_psi = (float)psi;
        T.place.orientation.pitch = 0;
        if (lead_on && gap > 0 && gap < 100) { g_fake_radar[0] = gap; g_fake_radar[1] = cross_v > 0 ? cross_v : (float)lv; }
        else { g_fake_radar[0] = 3.4e38f; g_fake_radar[1] = 0; }
        radar_stub(GA, g_fake_radar);
        float req_before = *(float *)(GA + GA_BRAKE_REQ);
        (void)req_before;
        vehicle_frame();
        float u = act_stub(GA, (float)dt, pedal);
        float req = *(float *)(GA + GA_BRAKE_REQ);
        r.max_req = fmaxf(r.max_req, req);
        if (r.warn_gap < 0 && lead_on && AE.st >= AEB_WARN && AE.st <= AEB_FULL) r.warn_gap = gap;
        if (r.brake_gap < 0 && lead_on && req > 0.02f) r.brake_gap = gap;
        int dir = req > prev_req + 1e-4f ? 1 : req < prev_req - 1e-4f ? -1 : 0;
        if (dir && last_dir && dir != last_dir) { r.flips++; if (verbose) printf("   flip t=%.2f req %.3f st %d gap %.1f v %.1f psi %.0f a_need %.2f\n", t, req, AE.st, gap, v * 3.6, psi, AE.a_need); }
        if (dir) last_dir = dir;
        prev_req = req;
        if (getenv("AEBDBG") && lead_on && (AE.st || req > 0)) printf("  t %.2f gap %.2f v %.2f st %d a_need %.2f a_cmd %.2f req %.3f accel %.2f\n", t, gap, v, AE.st, AE.a_need, AE.a_cmd, req, a_f);
        brk = req > brk ? fmin(req, brk + 5 * dt) : fmax(req, brk - 5 * dt);   // the game's slew
        double applied = fmax(brk * 10 / fmax(1.0, mass / 30000.0) * g_plant_k, pbrake * 8.0);
        static double applied_prev; if (t == 0) applied_prev = 0;
        if (applied > applied_prev) psi -= (applied - applied_prev) / 10 * 60;
        applied_prev = applied;
        psi = fmin(psi0, psi + 4 * dt);
        if (req > 0) u = 0;
        if (u_hold && req <= 0 && pbrake <= 0) u = 0.6f;   // the tailgater keeps up with the lead (just enough throttle to hold 80)
        double a = (v > 0.01 ? -0.4 : 0) + (u_hold && req <= 0 ? 0.4 : u * 20000.0 / mass) - (v > 0.01 ? applied : 0);
        if (v <= 0.01 && a < 0) a = 0;
        v = fmax(0, v + a * dt);
        x += v * dt;
        a_f += (a - a_f) * fmin(1, dt / 0.15);
        r.peak_dec = fmaxf(r.peak_dec, (float)-a);
        if (lead_on && t > 0.5) r.min_gap = fminf(r.min_gap, (float)(lx - x));
        r.max_st = AE.st > r.max_st && AE.st <= AEB_FULL ? AE.st : r.max_st;
        if (AE.st == AEB_WARN && t_warn < 0) t_warn = t;
        if ((AE.st == AEB_PARTIAL || AE.st == AEB_FULL) && t_part < 0) t_part = t;
        r.psi_drop = fmaxf(r.psi_drop, (float)(psi0 - psi));
        if (scen != 2 && scen != 7 && v < 0.01 && t > 5 && AE.st == AEB_HOLD) { T.in_brake = 0.3f; }
    }
    r.end_kmh = (float)(v * 3.6);
    r.final_gap = lead_on ? (float)(lx - x) : -1;
    r.warn_lead_s = t_warn >= 0 && t_part >= 0 ? (float)(t_part - t_warn) : -1;
    if (r.min_gap > 1e8f) r.min_gap = 0;
    printf("%-50s max %-15s min gap %5.1f m, peak decel %.1f, max request %.2f, warning %.1f s before braking, air -%.0f psi, request reversals %d, end %.0f km/h%s\n",
           name, aeb_name[r.max_st], r.min_gap, r.peak_dec, r.max_req, r.warn_lead_s, r.psi_drop, r.flips, r.end_kmh, r.hit ? "  <-- HIT" : "");
    s_aeb_owns = 0; g_sim_mass = 26000; T.in_steer = 0; T.yaw = 0; GA[0x1d0] = 0;
    g_brake_learn = 0;
    return r;
}

// ======================================================================= force_mode: the controller asks a force, the throttle gives it in this gear
// A 12-speed box on the vehicle model (the Volvo ratios of sim.cpp, diff 2.85, this file's engine: 2400 Nm, flat 1000-1500 rpm, limit 2200),
// the force = throttle x the gear's full-throttle force (vehicle_force_at), a 0.5 s torque gap per shift. The box shifts by rpm and pull (up
// over 1500 rpm when the next gear lands at 950 or more and gives the force of now at 90 % throttle, down under 950, down also after 3 s at full
// throttle when the lower gear stays under 1700): the same box for both units, so the difference is the controller's.
static const float FM_RATIO[12] = { 11.73f, 9.21f, 7.09f, 5.57f, 4.35f, 3.41f, 2.7f, 2.12f, 1.63f, 1.28f, 1.0f, 0.78f };
static int g_fm_plant;   // the force_mode runs: the wish pedal's reference on this plant (the other runs: the model cannot tell, the old unit)
// A full wish at v as the wish pedal finds it with ref_now 1: the best gear at full throttle whose revs lie in 1000 ... 1900 rpm (its window on
// this engine), the first gear at 1000 rpm as the cap (pedal.cpp: wish_ref)
static int g_full_power;
float pedal_wish_full(float v)
{
    if (!g_fm_plant) return 0;
    float best = 0;
    // (g_full_power: as pedal.cpp has it, the power of the best gear in the window, as the force at this speed; else the strongest force)
    for (int g = 1; g <= 12; g++) { float r = vehicle_rpm_at(g, v); if (r >= 1000 && r <= (g_full_power ? 1600 : 1900)) best = fmaxf(best, vehicle_force_at(g, v, 1.0f)); }
    if (g_full_power) { float pw = 0; for (int g = 1; g <= 12; g++) { float r = vehicle_rpm_at(g, v); if (r >= 1000 && r <= 1600) pw = fmaxf(pw, vehicle_force_at(g, v, 1.0f) * v); } return pw / fmaxf(v, 0.5f); }
    float cap = vehicle_force_at(1, vehicle_kmh_at(1, 1000) / 3.6f, 1.0f);
    return best > 0 ? fminf(best, cap) : cap;
}
static float fm_road(double s) { return s < 2000 ? 0 : s < 3500 ? 3.0f : 0; }   // 2 km flat, 1.5 km at 3 %, flat
static float fm_road1(double) { return 1.0f; }                                 // 1 % all along: 12 needs ~0.75 throttle at 33 t
struct fmres { float e_flat, loss, e_climb, thr_max_near, jump, f_err, t82, thr_hi; int shifts, f_seen; };   // jump: the largest change of the pull across a shift that began under full throttle (share; -1 = none)
static fmres fm_run(float mass, int fm, float cvar, float eco_cap, float (*road)(double) = fm_road, float v0 = 83)
{
    fmres r = {}; r.jump = -1; r.t82 = -1;
    memset(GA, 0, sizeof GA); memset(&T, 0, sizeof T); memset(&C, 0, sizeof C); memset(&AH, 0, sizeof AH);
    for (int o = 0; o < ACT_N; o++) act_brake(o, 0);
    act_reset(RESET_HARD, "sim"); cr_reset(RESET_HARD, "sim");
    g_fm_plant = 1; p_force_mode = (float)fm; p_eco_cap = eco_cap; g_cvar_adaptive = cvar;
    g_sim_eng = engine_info(); g_sim_eng.torque = 2400; g_sim_eng.n_curve = 0; g_sim_eng.rpm_limit = 2200; g_sim_mass = mass;
    for (int i = 0; i < 12; i++) T.ratio[i] = FM_RATIO[i];
    T.diff = 2.85f; T.fwd_gears = 12; T.wheels = 1; T.engine_on = 1; T.air_psi = 120;
    vehicle_reset(RESET_HARD);
    o_cruise = stock_cruise; stock_thr = 0; p_smooth = 1; g_sg_ok = 1; g_low_ok = 1; g_sg_hold = 0; g_feed = 0; g_stopped_at = 0;
    *(unsigned char *)(GA + GA_ON) = 1; *(float *)(GA + GA_SET) = 83;
    float v = v0 / 3.6f, f_before = -1, u = 0, lo = 999; float f_last = 0, u_last = 1; double s = 0, t0 = t_now, sum_f = 0, sum_c = 0, shift_end = -1, full_since = -1, near_since = -1;
    int n_f = 0, n_c = 0, gear = v0 < 83 ? 11 : 12, next = gear;
    for (; t_now - t0 < 200; t_now += 1 / 60.0) {
        double t = t_now - t0;
        float grade = road(s), kmh = v * 3.6f;
        T.speed = v; T.cruise = 83 / 3.6f; T.place.orientation.pitch = atanf(grade / 100) / 6.2831853f;
        *(float *)(GA + GA_LEAD_M) = 0; *(float *)(GA + GA_ACC_TARGET) = 0; *(float *)(GA + GA_BRAKE_REQ) = 0;
        // the box
        if (shift_end >= 0 && t_now >= shift_end) { gear = next; shift_end = -1; r.shifts++; }
        int shifting = shift_end >= 0;
        if (!shifting) {
            float rpm = vehicle_rpm_at(gear, v);
            if (gear < 12 && rpm > 1500 && vehicle_rpm_at(gear + 1, v) >= 950 && u * vehicle_force_at(gear, v, 1.0f) <= 0.9f * vehicle_force_at(gear + 1, v, 1.0f)) next = gear + 1;
            else if (gear > 1 && (rpm < 950 || (full_since >= 0 && t_now - full_since > 3 && vehicle_rpm_at(gear - 1, v) < 1700))) next = gear - 1;
            if (next != gear) { shift_end = t_now + 0.5; full_since = -1; shifting = 1; }
        }
        T.gear = T.dgear = gear; T.rpm = vehicle_rpm_at(gear, v); T.eff_clutch = shifting ? 1 : 0;
        vehicle_frame();
        u = act_stub(GA, 1 / 60.0f, 0);
        T.eff_thr = u;
        if (v0 < 83) { if (r.t82 < 0 && kmh >= 82) r.t82 = (float)t; if (r.t82 < 0 && t > 1) r.thr_hi = fmaxf(r.thr_hi, u); }   // (not the first second: the bumpless start takes over the game's throttle)
        if (u < 0.99f || shifting) full_since = -1; else if (full_since < 0) full_since = t_now;
        // what TrueShift would read: the force the cruise asks; where the throttle is not at 1 it is the force the throttle gives in this gear
        float fw = cruise_wish_force(v);
        if (fw >= 0) r.f_seen++;
        if (fw >= 0 && !shifting && u > 0.05f && u < 0.98f) r.f_err = fmaxf(r.f_err, fabsf(fw - u * vehicle_force_at(gear, v, 1.0f)) / fw);
        // (the Eco cap: within 4 km/h of the target for 1 s, so the rate limit has brought a throttle from under the band down to it)
        if (kmh <= 79) near_since = -1; else if (near_since < 0) near_since = t_now;
        if (near_since >= 0 && t_now - near_since >= 1) r.thr_max_near = fmaxf(r.thr_max_near, u);
        float F = shifting ? 0 : u * vehicle_force_at(gear, v, 1.0f);
        if (!shifting && f_before > 0 && F > 0) { r.jump = fmaxf(r.jump, fabsf(F / f_before - 1)); f_before = -1; }   // the first tick after a shift
        if (!shifting) { f_last = F; u_last = u; } else if (f_before < 0 && u_last < 0.98f) f_before = f_last;
        float res = mass * 9.81f * (sinf(atanf(grade / 100)) + 0.006f) + 0.5f * 1.2f * 6 * v * v;
        if (g_full_power) T.accel = (F - res) / mass;   // (the approach test: the acceleration the cruise reads)
        v = fmaxf(0, v + (F - res) / mass / 60);
        s += v / 60;
        if (t > 20 && s < 2000) { sum_f += fabsf(kmh - 83); n_f++; }
        if (s >= 2000 && s < 3500) { sum_c += fabsf(kmh - 83); n_c++; lo = fminf(lo, kmh); }
    }
    r.e_flat = n_f ? (float)(sum_f / n_f) : 0; r.e_climb = n_c ? (float)(sum_c / n_c) : 0; r.loss = 83 - lo;
    g_fm_plant = 0; p_force_mode = 0; p_eco_cap = 1; g_cvar_adaptive = 3;
    return r;
}
static int force_mode_test()
{
    int bad = 0;
    printf("--- force_mode: cruise 83 km/h, 2 km flat, then 1.5 km at 3 %% (12 speeds, the box shifts by rpm); force_mode 0 against 1\n");
    printf("   %-14s %s  %8s %8s %8s %6s %10s %8s %8s\n", "", "fm", "|e| flat", "loss", "|e| 3 %", "shifts", "pull step", "force", "f err");
    for (float m : { 17000.0f, 33000.0f }) {
        fmres a[2]; char js[2][16];
        for (int fm = 0; fm < 2; fm++) {
            a[fm] = fm_run(m, fm, 3, 1);
            if (a[fm].jump < 0) snprintf(js[fm], sizeof js[fm], "-"); else snprintf(js[fm], sizeof js[fm], "%.0f%%", a[fm].jump * 100);
            printf("   %-14s %d  %8.2f %8.1f %8.2f %6d %10s %8s %7.1f%%\n", m < 20000 ? "17 t Normal" : "33 t Normal", fm, a[fm].e_flat, a[fm].loss, a[fm].e_climb,
                   a[fm].shifts, js[fm], a[fm].f_seen ? "read" : "-1", a[fm].f_err * 100);
        }
        bad |= a[0].f_seen != 0 || a[1].f_seen == 0 || a[1].f_err > 0.01f || a[1].jump > 0.05f;
        bad |= a[1].e_flat > a[0].e_flat + 0.1f || a[1].loss > a[0].loss + 0.5f || a[1].e_climb > a[0].e_climb + 0.2f;
    }
    for (int fm = 0; fm < 2; fm++) {
        fmres e = fm_run(33000, fm, 10, 0.6f, fm_road1);
        printf("   33 t Eco, 1 %% all along, eco_cruise_throttle_cap 0.6, force_mode %d: highest throttle 1 s within 4 km/h of 83: %.3f, lowest %.1f km/h at 2-3.5 km, %d shifts\n",
               fm, e.thr_max_near, 83 - e.loss, e.shifts);
        bad |= e.thr_max_near > 0.6f + 1e-4f;
    }
    // approach_by_mode: the approach 70 -> 83 km/h on the flat, 33 t; 0 = the throttle follows the deficit up to full, 1 = Eco and Normal use a share of the pull to spare
    printf("--- approach_by_mode: 70 -> 83 km/h, 33 t, flat; highest throttle before 82 km/h and the seconds to 82 km/h\n");
    float hi[2][3], ts[2][3];
    for (int ab = 0; ab < 2; ab++) {
        p_app_mode = (float)ab; g_full_power = 1;
        printf("   approach_by_mode %d:", ab);
        int ci = 0;
        for (float cv : { 10.0f, 3.0f, 1.6667f }) {
            fmres a = fm_run(33000, 0, cv, 1, fm_road, 70);
            hi[ab][ci] = a.thr_hi; ts[ab][ci] = a.t82; ci++;
            printf("  %s %.2f / %.1f s", cv > 5 ? "Eco" : cv > 2 ? "Normal" : "Power", a.thr_hi, a.t82);
        }
        printf("\n");
    }
    p_app_mode = 1; g_full_power = 0;
    bad |= hi[0][0] < 0.99f || hi[0][1] < 0.99f;   // (today: full throttle)
    bad |= hi[1][0] > 0.9f || hi[1][1] > 0.9f || hi[1][2] < 0.99f || !(ts[1][0] > ts[1][1] && ts[1][1] > ts[1][2]);
    printf("force_mode test: %s\n", bad ? "FAIL" : "ok");
    return bad;
}

int main(int argc, char **argv)
{
    verbose = argc > 1;
    for (int i = 0; i < m_cruise.n_params; i++) *m_cruise.params[i].val = m_cruise.params[i].def;
    no_cruise_test();   // first: the actuator alone, before the module's phases are attached
    cr_attach();        // the module's three phases in the actuator's tick (cr_start does this in the game)
    keys_test();
    g_keys_ok = 0; g_ret_ok = 0;
    memset(GA, 0, sizeof GA);
    g_road = road_flat;
    for (int ours = 0; ours <= 1; ours++) run("accelerate 20 -> 80 km/h, flat (shift bursts?)", ours, 80, 60, c_none, 20);
    for (int ours = 0; ours <= 1; ours++) run("hold 80 on hills +-2.5%", ours, 80, 180, c_hills, 80);
    g_road = road_flat;
    for (int ours = 0; ours <= 1; ours++) run("ACC jam: lead 30, stops, goes 25, stops", ours, 60, 110, c_lead_jam, 30);
    // hill start
    {
        memset(GA, 0, sizeof GA); T.speed = 0; T.in_brake = 0.5f; T.in_thr = 0;
        T.place.orientation.pitch = atanf(0.05f) / 6.2831853f; vehicle_frame();
        for (int k = 0; k < 30; k++) { t_now += 1 / 60.0; hill_start(GA); act_brake(ACT_HILL, s_want_hill); act_brake_write(GA); }
        T.in_brake = 0;
        int held = 0;
        for (int k = 0; k < 300; k++) { t_now += 1 / 60.0; *(float *)(GA + GA_BRAKE_REQ) = 0; hill_start(GA); act_brake(ACT_HILL, s_want_hill); act_brake_write(GA); if (*(float *)(GA + GA_BRAKE_REQ) > 0) held++; if (k == 90) T.in_thr = 0.3f; }
        printf("hill start 5%%: brake held %.1f s after release (gas at 1.5 s)\n", held / 60.0);
    }
    // same, game with its emergency brake option OFF (it never clears 0x17c; ts3c latch)
    {
        memset(GA, 0, sizeof GA); T.speed = 0; T.in_brake = 0.5f; T.in_thr = 0;
        for (int k = 0; k < 30; k++) { t_now += 1 / 60.0; hill_start(GA); act_brake(ACT_HILL, s_want_hill); act_brake_write(GA); }
        T.in_brake = 0;
        int held = 0;
        for (int k = 0; k < 300; k++) { t_now += 1 / 60.0; hill_start(GA); act_brake(ACT_HILL, s_want_hill); act_brake_write(GA); if (*(float *)(GA + GA_BRAKE_REQ) > 0) held++; if (k == 90) T.in_thr = 0.3f; }
        printf("hill start 5%%, AEB off: brake held %.1f s, request at the end %.2f (must be 0)\n", held / 60.0, *(float *)(GA + GA_BRAKE_REQ));
    }
    ah_run("no hold at all 0% gentle (reference)", 0, 0, 1, 1.0f, 2, 0); ah_run("no hold at all 0% hard (reference)", 0, 1, 1, 1.0f, 2, 0);
    // ACC with a lead vehicle
    {
        int bad = 0;
        const char *nm[] = { "cut-in slower (60 km/h, 28 m ahead) at 80", "cut-in faster (86 km/h, 30 m ahead) at 80, calm", "lead brakes 3 m/s2 from 80 to 30, 50 m ahead", "far lead (70 m, 70 km/h) at 80", "closing on a 25 km/h lead from 99 m at 80", "ts4p cut-in at 27 m, 60 -> 45 km/h, we 69", "we 40, a car crosses 40 m ahead at 25 km/h for 2 s", "lead 92 m ahead brakes 49 -> 33 km/h, we 90 (ride 12-10)" };
        printf("--- ACC with a lead: game target against own ACC target\n");
        accres g[8], o[8];
        for (int sc = 0; sc < 8; sc++) { g[sc] = acc_run(nm[sc], 0, sc); o[sc] = acc_run(nm[sc], 1, sc); }
        // own ACC: no emergency braking where the game's target needed it, no harder decel than the game, calm cut-in really calm
        // own ACC brakes itself early: no emergency braking where we can avoid it, never more than ~4 m/s2 (the emergency
        // hazards threshold), gaps not smaller than the game's
        for (int sc = 0; sc < 6; sc++) bad |= o[sc].emerg_s > g[sc].emerg_s + 0.1f || o[sc].peak_dec > 4.0f || o[sc].min_gap < 6;  // the gap itself is the player's choice (Damien 11-10: shorter levels)
        bad |= o[1].min_kmh < 79.5f;                 // cut-in faster: no slowing down at all
        bad |= o[6].min_kmh < 37.0f || o[6].emerg_s > 0;                 // crossing traffic: passed, not followed (friend 11-10, roundabouts)
        bad |= o[3].max_jerk > g[3].max_jerk;        // far lead: smoother than the game
        // the same with our AEB in place of the game's ACC emergency part: normal following must not need it hard
        printf("--- own ACC + own AEB (the game's ACC emergency braking off)\n");
        for (int sc = 0; sc < 8; sc++) {
            memset(&AE, 0, sizeof AE); AE.gap_prev = AE.rate = AE.vl_prev = NAN; s_aeb_owns = 1; T.in_thr = 0; T.in_brake = 0; T.dgear = 8;
            char nm2[96]; snprintf(nm2, sizeof nm2, "%s +AEB", nm[sc]);
            accres w = acc_run(nm2, 1, sc);
            printf("     AEB highest stage %d\n", AE.ev_max);
            s_aeb_owns = 0;
            bad |= w.min_gap < 4 || (sc != 2 && sc != 5 && w.peak_dec > 4.0f);
        }
        // csv trace header and rows have the same number of columns
        {
            FILE *f = tmpfile(); g_csv = f;
            fputs("t,kmh,set,gap,radar_gap,lead_kmh,opening_ms,acc_game,acc_own,acc_used,wanted_gap,thr,req_ours,req_ga,ah_state,ah_r,in_thr,in_brake,gear,eff_clutch,rpm,grade,air_psi,aeb,ttc,a_need,a_cmd,decel,bend_kmh,a_lat\n", f);
            acc_run("csv trace", 1, 0);
            rewind(f);
            char line[512]; int cols_h = 0, rows = 0, mism = 0;
            while (fgets(line, sizeof line, f)) {
                int c = 1; for (char *p = line; *p; p++) c += *p == ',';
                if (!cols_h) cols_h = c; else { rows++; mism += c != cols_h; }
            }
            printf("cruise.csv: %d columns, %d rows in 60 s (10 Hz while cruise is on), column mismatches %d\n", cols_h, rows, mism);
            bad |= mism || rows < 500;
            fclose(f); g_csv = NULL;
        }
        printf("ACC test: %s\n", bad ? "FAIL" : "ok");
        if (bad) return 1;
    }
    // own AEB
    {
        int bad = 0;
        printf("--- own AEB (cruise off, driver holds 30 %% throttle and does not react unless said)\n");
        aebres a;
        a = aeb_run("stopped queue at 80 km/h, 26 t", 0, 26000, 140); bad |= a.hit || a.min_gap < 1.5f || a.flips > 4;
        a = aeb_run("stopped queue at 80 km/h, 60 t", 0, 60000, 140); bad |= a.hit || a.min_gap < 1.0f;
        a = aeb_run("stopped queue at 80 km/h, 26 t, air 80 psi", 5, 26000, 80); bad |= a.hit || a.flips > 4;
        a = aeb_run("slow truck 40 km/h at 80", 1, 26000, 140); bad |= a.hit || a.max_st > AEB_PARTIAL || a.min_gap < 5;
        a = aeb_run("cut-in 20 m at 30 km/h (we 60), it brakes to 0", 2, 26000, 140); bad |= a.hit;
        a = aeb_run("we 40, a car crosses 25 m ahead for 2 s", 3, 26000, 140); bad |= a.max_req > 0 || a.max_st > AEB_WARN;
        a = aeb_run("stopped queue, kick-down in partial braking, lane change", 4, 26000, 140); bad |= a.after_kd > 0 || a.hit; printf("   request 0.4-1 s after the kick-down: %.2f (must be 0)\n", a.after_kd);
        a = aeb_run("stopped queue, the driver brakes himself (4 m/s2)", 6, 26000, 140); bad |= a.max_req > 0 || a.hit;
        a = aeb_run("following at 25 m, 80 km/h, the lead brakes 5 m/s2 to 0", 7, 26000, 140); bad |= a.hit;
        a = aeb_run("tailgating 12 m at 80, the lead brakes 4 m/s2 to 0", 8, 26000, 140); bad |= a.hit || a.flips > 6;
        a = aeb_run("tailgating 7 m at 80, the lead brakes 4 m/s2 to 0", 9, 26000, 140); bad |= a.hit || a.flips > 6;
        a = aeb_run("slow truck, at the warning: indicator left + steer left", 10, 26000, 140); bad |= a.hit || a.after_kd > 0;
        printf("   request 0.8-1.5 s after the evasion: %.2f (must be 0)\n", a.after_kd);
        a = aeb_run("slow truck, at the warning: indicator left, no steering", 11, 26000, 140); bad |= a.hit || a.max_st < AEB_PARTIAL;
        printf("AEB test: %s\n", bad ? "FAIL" : "ok");
        if (bad) return 1;
    }
    // Auto Hold (stop with the brake, release, wait, launch gentle / hard)
    {
        int bad = 0;
        printf("--- Auto Hold: stop on a grade, release the pedal, 2.5 s later gas (gentle: 0.35 in 1 s, hard: 1.0 in 0.2 s)\n");
        float gr[] = { 0, 5, 10 };
        for (int hard = 0; hard <= 1; hard++)
            for (int i = 0; i < 3; i++) {
                char nm[80];
                snprintf(nm, sizeof nm, "Auto Hold %2.0f%% %s", gr[i], hard ? "hard launch" : "gentle launch");
                ahres a = ah_run(nm, gr[i], hard, 1, 1.0f, 0, 0);
                bad |= a.roll_m > 0.05f || a.req_end != 0 || a.t_move < 0 || a.jerk > (hard ? 15.0f : 6.0f);
                if (gr[i] >= 5) {
                    snprintf(nm, sizeof nm, "  old hill start %2.0f%% %s", gr[i], hard ? "hard launch" : "gentle launch");
                    ahres o = ah_run(nm, gr[i], hard, 1, 1.0f, 1, 0);
                    bad |= o.req_end != 0;
                }
            }
        printf("--- Auto Hold: hard launch jerk against auto_hold_hard_s (m/s3; no hold at all = 12.3 on the flat)\n");
        for (float hs : { 0.1f, 0.15f, 0.2f, 0.25f, 0.35f, 0.5f }) {
            p_ah_hard = hs;
            float j[3]; float gg[] = { 0, 5, 10 };
            for (int i = 0; i < 3; i++) { FILE *o = stdout; (void)o; j[i] = 0; }
            // quiet runs
            int sv = dup(1); freopen("/dev/null", "w", stdout);
            for (int i = 0; i < 3; i++) j[i] = ah_run("q", gg[i], 1, 1, 1.0f, 0, 0).jerk;
            fflush(stdout); dup2(sv, 1); close(sv);
            printf("   hard_s %.2f: jerk 0%% %.1f, 5%% %.1f, 10%% %.1f\n", hs, j[0], j[1], j[2]);
        }
        p_ah_hard = 0.2f;
        printf("--- Auto Hold, harder cases (not pass/fail except the request ending at 0)\n");
        ahres a;
        a = ah_run("  no throttle gate patch, 5% gentle", 5, 0, 0, 1.0f, 0, 0); bad |= a.req_end != 0;
        a = ah_run("  no throttle gate patch, 10% hard", 10, 1, 0, 1.0f, 0, 0); bad |= a.req_end != 0;
        a = ah_run("  drive over-estimated x1.5 (mass off), 10% hard", 10, 1, 1, 1.5f, 0, 0); bad |= a.req_end != 0 || a.roll_m > 0.3f;
        a = ah_run("  drive over-estimated x1.5 (mass off), 10% gentle", 10, 0, 1, 1.5f, 0, 0); bad |= a.req_end != 0 || a.roll_m > 0.3f;
        g_true_k = 0.75f; s_ah_k = 1;
        a = ah_run("  engine really 75% of the estimate, 10% gentle", 10, 0, 1, 1.0f, 0, 0); bad |= a.req_end != 0 || a.roll_m > 0.3f;
        printf("     (learned drive factor now %.2f)\n", s_ah_k);
        a = ah_run("  ... the next stop, same truck", 10, 0, 1, 1.0f, 0, 0); bad |= a.req_end != 0 || a.roll_m > 0.1f;
        printf("     (learned drive factor now %.2f)\n", s_ah_k);
        g_true_k = 0.45f; s_ah_k = 1;
        printf("--- informational: the pedal is too light for this engine (45%% of the estimate): the guard grabs, learns, the truck may still roll\n");
        ah_run("  engine really 45%, 10% gentle", 10, 0, 1, 1.0f, 0, 0); printf("     (learned drive factor %.2f)\n", s_ah_k);
        ah_run("  ... next stop", 10, 0, 1, 1.0f, 0, 0); printf("     (learned drive factor %.2f)\n", s_ah_k);
        s_ah_k = 1;
        g_true_k = 1.0f;
        a = ah_run("  mass under-estimated /1.5, 5% gentle", 5, 0, 1, 0.6667f, 0, 0); bad |= a.req_end != 0;
        {   // the game cannot tell the mass (V.mass_ok 0): the drive estimate is not used, the pedal paces the release
            float k = s_ah_k; vehicle_reset(RESET_HARD);
            a = ah_run("  mass unknown, 10% gentle", 10, 0, 1, NAN, 0, 0); bad |= a.req_end != 0 || a.roll_m > 0.3f;
            a = ah_run("  mass unknown, 10% hard", 10, 1, 1, NAN, 0, 0); bad |= a.req_end != 0 || a.roll_m > 0.3f;
            a = ah_run("  mass unknown, 5% gentle", 5, 0, 1, NAN, 0, 0); bad |= a.req_end != 0 || a.roll_m > 0.3f;
            vehicle_reset(RESET_HARD); s_ah_k = k;
        }
        {   // the drive estimate takes the model's driveline efficiency ([trueshift] efficiency, 0.9 here otherwise)
            float k = s_ah_k, e = V.eta; V.eta = 0.8f;
            a = ah_run("  efficiency 0.8, 10% gentle", 10, 0, 1, 1.0f, 0, 0); bad |= a.req_end != 0 || a.roll_m > 0.3f;
            V.eta = e; s_ah_k = k;
        }
        printf("--- Auto Hold 10%% gentle, trace after the stop\n");
        ah_run("  trace", 10, 0, 1, 1.0f, 0, 1);
        ah_run("  trace 0% gentle", 0, 0, 1, 1.0f, 0, 1);
        // blockers: crawl on, reverse, parking brake, Stop&Go: the hold never starts
        {
            struct { const char *n; int crawl, rev, park, sg; } bl[] = { { "crawl on", 1, 0, 0, 0 }, { "reverse", 0, 1, 0, 0 }, { "parking brake", 0, 0, 1, 0 }, { "Stop&Go hold", 0, 0, 0, 1 } };
            for (auto &b : bl) {
                memset(&AH, 0, sizeof AH); memset(GA, 0, sizeof GA);
                g_low_ok = 1; p_ah = 1; g_crawl_on = b.crawl; g_sg_hold = b.sg; T.parking_brake = b.park; T.gear = b.rev ? -1 : 0; T.dgear = T.gear;
                T.engine_on = 1; T.speed = 0; T.in_brake = 0.5f; T.in_thr = 0; s_req_ours = 0;
                for (int k = 0; k < 120; k++) { t_now += 1 / 60.0; auto_hold(GA, 1 / 60.0f); act_brake(ACT_HOLD, s_want_hold); act_brake_write(GA); }
                T.in_brake = 0;
                float mx = 0;
                for (int k = 0; k < 120; k++) { t_now += 1 / 60.0; auto_hold(GA, 1 / 60.0f); act_brake(ACT_HOLD, s_want_hold); act_brake_write(GA); mx = fmaxf(mx, *(float *)(GA + GA_BRAKE_REQ)); }
                printf("Auto Hold blocked by %-14s: max request %.2f (must be 0)\n", b.n, mx);
                bad |= mx > 0;
            }
            g_crawl_on = 0; g_sg_hold = 0; T.parking_brake = 0; T.gear = 0; T.dgear = 0;
        }
        printf("Auto Hold test: %s\n", bad ? "FAIL" : "ok");
        if (bad) return 1;
    }
    // last: these leave controller state behind (the stock PI's memory, our brake slew), which would shift the runs above
    junction_test();
    reset_test();
    limiter_test();
    // own AEB by distance (aeb_mode 1) against the time stages (aeb_mode 0). The driver holds the speed and does not react.
    // After the tests above on purpose: its long runs move the clock, which would shift the timing of the Auto Hold runs.
    {
        int bad = 0;
        printf("--- own AEB, aeb_mode 1 (by the deceleration it takes to stop 0.5 m behind the lead) against aeb_mode 0 (time stages)\n");
        aebres a, t[2][3];
        const double sp[3] = { 30, 50, 80 };
        for (int m = 0; m < 2; m++)
            for (int i = 0; i < 3; i++) {
                char nm[80]; snprintf(nm, sizeof nm, "mode %d: standing lead at %.0f km/h, 26 t", m, sp[i]);
                p_aeb_mode = (float)m; g_aeb_v0 = sp[i];
                t[m][i] = aeb_run(nm, 14, 26000, 140);
            }
        for (int i = 0; i < 3; i++) {
            bad |= t[1][i].hit || t[1][i].final_gap < 0.2f || t[1][i].final_gap > 1.5f;
            if (i < 2) bad |= !(t[1][i].brake_gap < t[0][i].brake_gap) || !(t[1][i].warn_gap < t[0][i].warn_gap);   // later than mode 0 at 30 and 50
        }
        printf("   standing lead, distance to the lead (m): first warning / first braking / final gap\n");
        printf("   %-8s %-28s %-28s\n", "km/h", "mode 0 (time to collision)", "mode 1 (distance)");
        for (int i = 0; i < 3; i++)
            printf("   %-8.0f %5.1f / %5.1f / %5.2f           %5.1f / %5.1f / %5.2f\n", sp[i], t[0][i].warn_gap, t[0][i].brake_gap, t[0][i].final_gap,
                   t[1][i].warn_gap, t[1][i].brake_gap, t[1][i].final_gap);
        g_aeb_v0 = 80; p_aeb_mode = 1;
        a = aeb_run("mode 1: standing lead at 80 km/h, 60 t", 14, 60000, 140); bad |= a.hit || a.final_gap < 0.2f || a.final_gap > 1.5f;
        g_aeb_v0 = 50; a = aeb_run("mode 1: following 30 m at 50, the lead brakes 5 m/s2 to 0", 12, 26000, 140); bad |= a.hit || a.flips > 6;
        g_aeb_v0 = 80;
        a = aeb_run("mode 1: far slower lead (60 km/h at 150 m), eased off", 13, 26000, 140); bad |= a.max_req > 0 || a.max_st != AEB_OFF;   // no intervention at all
        a = aeb_run("mode 1: standing lead, at the warning: indicator + steer left", 15, 26000, 140); bad |= a.hit || a.after_kd > 0;
        printf("   request 0.8-1.5 s after the evasion: %.2f (must be 0)\n", a.after_kd);
        a = aeb_run("mode 1: standing lead, kick-down in braking, lane change", 4, 26000, 140); bad |= a.after_kd > 0;
        printf("   request 0.4-1 s after the kick-down: %.2f (must be 0)\n", a.after_kd);
        a = aeb_run("mode 1: standing lead, the driver brakes himself (4 m/s2)", 6, 26000, 140); bad |= a.max_req > 0 || a.hit;
        a = aeb_run("mode 1: we 40, a car crosses 25 m ahead for 2 s", 3, 26000, 140); bad |= a.max_req > 0 || a.max_st > AEB_WARN;
        a = aeb_run("mode 1: following 25 m at 80, the lead brakes 5 m/s2 to 0", 7, 26000, 140); bad |= a.hit;
        a = aeb_run("mode 1: slow truck 40 km/h at 80", 1, 26000, 140); bad |= a.hit;
        // the late full braking falls where "dead ahead and very close" (< 10 m, < 1 s) holds: the evasion must still stand
        a = aeb_run("mode 1: slow truck, at the warning: indicator + steer left", 10, 26000, 140); bad |= a.hit || a.after_kd > 0;
        printf("   request 0.8-1.5 s after the evasion: %.2f (must be 0)\n", a.after_kd);
        a = aeb_run("mode 1: slow truck, evasion given up after 0.6 s", 16, 26000, 140); bad |= a.hit || a.max_st != AEB_FULL;
        a = aeb_run("mode 1: slow truck, at the warning: indicator, no steering", 11, 26000, 140); bad |= a.hit || a.max_st != AEB_FULL;
        // town traffic at 28-33 km/h, closely behind (6, 3, then 1 m): no braking of its own (ride logs: full braking there)
        g_aeb_v0 = 30;
        for (int m = 1; m >= 0; m--) {
            p_aeb_mode = (float)m;
            char nm[80]; snprintf(nm, sizeof nm, "mode %d: town traffic 28-33 km/h, 6 / 3 / 1 m behind", m);
            a = aeb_run(nm, 17, 26000, 140);
            if (m == 1) bad |= a.hit || a.max_req > 0 || a.max_st != AEB_OFF;
        }
        p_aeb_mode = 1; g_aeb_v0 = 80;
        // where it does not work (informational): late means no reserve: a hard cut-in closer than the stopping distance
        printf("--- aeb_mode 1, where it falls short (not pass/fail)\n");
        for (int m = 0; m < 2; m++) {
            p_aeb_mode = (float)m;
            char nm[96]; snprintf(nm, sizeof nm, "mode %d: cut-in 20 m at 30 km/h (we 60), it brakes to 0", m);
            a = aeb_run(nm, 2, 26000, 140);
            if (a.hit) printf("   hit: we 60 km/h, impact closing speed %.0f km/h\n", a.hit_kmh);
        }
        // [core] brake_learn: the AEB's capability from the learned gain. Plants with other brakes than the fit (x0.5 ... x1.5 of 10 m/s2 per
        // request at <= 30 t, 10 / (mass / 30 t) above), the standing lead at 80 km/h and the lead braking 5 m/s2 (following 25 m), mode 1 and 0, the fit
        // against the learned gain (exact, and 25 % too high: the clamp at the fit is what stands between that and an optimistic stage). Mode 0 (time stages)
        // does not use the capability: it runs as before with brake_learn on (asserted: the same figures).
        // Asserted: no hit with the learned gain where the fit had none and the minimum gap never shorter than the fit's (0.01 m); the strong
        // plants run as the fit (the clamp). Printed: min gap and final gap, fit -> learned.
        printf("--- aeb with the learned brake gain (brake_learn 1), plants x0.5 .. x1.5 of the fit; per line: fit -> learned\n");
        int nb = 0;
        for (int m = 1; m >= 0; m--)
            for (float mass : { 26000.0f, 60000.0f })
                for (int scen : { 14, 7 })
                    for (double pk : { 0.5, 0.7, 1.0, 1.5 })
                        for (double lx : { 1.0, 1.25 }) {
                            if (scen == 7 && m == 0) continue;
                            p_aeb_mode = (float)m; g_aeb_v0 = 80; g_plant_k = pk;
                            g_prime_k = 0;
                            aebres f = aeb_run("fit", scen, mass, 140);
                            g_prime_k = lx;
                            aebres l = aeb_run("learned", scen, mass, 140);
                            g_prime_k = 0;
                            bool ok = (!l.hit || f.hit) && l.min_gap >= f.min_gap - 0.01f;
                            if (!ok) nb++;
                            bad |= !ok;
                            printf("   mode %d %s %2.0f t plant x%.1f est x%.2f | min gap %5.1f -> %5.1f, final %5.1f -> %5.1f, max req %.2f -> %.2f%s%s | %s\n", m, scen == 14 ? "standing lead" : "braking lead ", mass / 1000, pk, lx,
                                   f.min_gap, l.min_gap, f.final_gap, l.final_gap, f.max_req, l.max_req, f.hit ? " HIT(fit)" : "", l.hit ? " HIT(learned)" : "", ok ? "ok" : "FAIL");
                        }
        g_plant_k = 1; p_aeb_mode = 0; g_aeb_v0 = 80;
        printf("   %d runs failed\n", nb);
        p_aeb_mode = 0; g_aeb_v0 = 80;
        printf("AEB distance mode test: %s\n", bad ? "FAIL" : "ok");
        if (bad) return 1;
    }
    // Auto Hold after a stop by the lever. Last on purpose: its runs move the clock and the actuator's memory.
    {
        int bad = 0;
        printf("--- Auto Hold after a stop by the retarder lever (step 2 of 4) from 30 km/h, the deceleration assist on; lever off 2 s after the stop, gentle gas 4 s after it\n");
        float gr[] = { 0, 6, -6 }, ms[] = { 18, 36 };
        for (float m : ms)
            for (float g : gr) {
                char nm[96]; snprintf(nm, sizeof nm, "lever stop %2.0f t %+2.0f%%", m, g);
                lvres r = lever_run(nm, g, m, 1, 1, 0, 0);
                bad |= r.t_stop < 0 || r.held_min < 0.3f || r.drop > 0.1f || r.roll_m > 0.05f || r.req_end != 0 || r.t_move < 0;
            }
        lvres r;
        r = lever_run("lever stop 36 t -10%, weak brake (2.5 m/s2 at 1.0)", -10, 36, 1, 1, 0, 0, 2.5f); bad |= r.held_min < 0.3f || r.drop > 0.1f || r.roll_m > 0.05f || r.req_end != 0 || r.t_move < 0;
        r = lever_run("lever stop 36 t +6%, engine brake alone", 6, 36, 1, 1, 1, 0); bad |= r.held_min < 0.3f || r.drop > 0.1f || r.roll_m > 0.05f || r.req_end != 0 || r.t_move < 0;
        r = lever_run("lever stop 36 t +6%, owner's standstill 0.15 s late", 6, 36, 1, 1, 0, 1); bad |= r.held_min < 0.3f || r.drop > 0.1f || r.roll_m > 0.05f || r.req_end != 0 || r.t_move < 0;
        // [core] brake_learn: the hold sized from the learned gain for the grade (twice the slope's pull + 0.8 m/s2, 0.2-1) in place of the ini's 0.6.
        // The estimate exact, and 25 % too high (the margin has to carry it). Asserted: it stops and holds (moved <= 0.05 m), no request left after the
        // gas, the truck moves off; printed: the lowest request while standing (the ini's is 0.6; a flat stop holds with 0.2).
        lever_run("(for comparison, brake_learn 0) lever stop 36 t +12%, ini 0.6", 12, 36, 1, 1, 0, 0, 4);
        lever_run("(for comparison, brake_learn 0) lever stop 36 t -10%, ini 0.6, 2.5 m/s2", -10, 36, 1, 1, 0, 0, 2.5f);
        printf("--- Auto Hold with the learned brake gain (brake_learn 1): the hold sized for the grade\n");
        for (double lp : { 1.0, 1.25 })
            for (float m : ms)
                for (float gn : { 4.0f, 2.5f })
                    for (float g : { 0.0f, 6.0f, -6.0f, 12.0f, -10.0f }) {
                        char nm[110]; snprintf(nm, sizeof nm, "learned x%.2f: lever stop %2.0f t %+3.0f%% brake %.1f m/s2", lp, m, g, gn);
                        g_lv_prime = lp;
                        lvres rl = lever_run(nm, g, m, 1, 1, 0, 0, gn);
                        g_lv_prime = 0;
                        bad |= rl.t_stop < 0 || rl.held_min < 0.19f || rl.roll_m > 0.05f || rl.req_end != 0 || rl.t_move < 0;
                    }
        printf("--- the same with auto_hold 0 and with auto_hold_lever 0: no Auto Hold, the truck rolls back (as before)\n");
        for (float m : ms)
            for (int k = 0; k < 2; k++) {
                char nm[96]; snprintf(nm, sizeof nm, "lever stop %2.0f t +6%%, %s", m, k ? "auto_hold_lever 0" : "auto_hold 0");
                r = lever_run(nm, 6, m, !k, 0, 0, 0);
                bad |= r.hold_after > 0;
            }
        printf("Auto Hold lever test: %s\n", bad ? "FAIL" : "ok");
        if (bad) return 1;
    }
    if (force_mode_test()) return 1;   // after the lever test: it starts everything it uses afresh
    return 0;
}
