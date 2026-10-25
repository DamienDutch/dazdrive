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
volatile float g_lcx_cap = 1;
volatile float g_overspeed_cap = 5;
volatile float g_pcc_need_kmh = NAN;
volatile float g_creep_brake;
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
volatile char g_dash_flash, g_dash_status;
volatile float g_creep_thr;
volatile float g_clutch_ours = __builtin_nanf("");
double g_dash_flash_until;
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
float *game_patch_rip_float(uintptr_t, int, int, float) { return NULL; }
int game_patch_feed_xmm1(uintptr_t, int, float *) { return 0; }
int game_patch_skip_if(uintptr_t, int, const unsigned char *) { return 0; }
int game_patch_zero_xmm_if(uintptr_t, int, int, const unsigned char *) { return 0; }
int mem_readable(const void *p, size_t) { return p != NULL; }
static float g_sim_mass = 26000;
static engine_info g_sim_eng;
float game_mass() { return g_sim_mass; }
void *game_gearbox() { return (void *)&g_sim_eng; }
int gb_engine(void *, engine_info *e) { *e = g_sim_eng; return 1; }
float curve_factor(const engine_info *e, float rpm) { return rpm < 1000 ? 0.8f + 0.2f * rpm / 1000 : rpm < 1500 ? 1.0f : 1.0f - (rpm - 1500) / 3000; }
float wheel_radius() { return 0.5f; }
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
float route_junction_speed(float, route_junction_t *) { return NAN; }
float route_limit_ahead(float, float *d) { if (d) *d = NAN; return NAN; }
void route_probe_limit(float) {}

#include "modules/cruise.cpp"
#include "core/rsc.cpp"

// ---- the game's cruise (incremental PI, see docs/cruise-re.md § 1)
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
        float u = ours ? stub_cruise(GA, 1 / 60.0f, 0) : stock_cruise(GA, 1 / 60.0f, 0);
        float brake = *(float *)(GA + GA_BRAKE_REQ);
        if (M.lead) {  // the game's ACC brakes itself when closing in (docs/cruise-re.md § 2)
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
      auto at = [](float g) { T.place.orientation.pitch = atanf(g / 100) / 6.2831853f; return smart_dynamic(); };
      float w0 = at(0), w1 = at(-1.5f), w2 = at(-3), w3 = at(+2);
      printf("smart overspeed: flat %.2f, -1.5 %% %.2f, -3 %% %.2f, uphill %.2f (expect 3.2, between, 2.0, 3.2)\n", w0, w1, w2, w3);
      bad |= !(w0 > 3.0f && w0 < 3.4f) || !(w1 < w0 && w1 > w2) || !(w2 > 1.9f && w2 < 2.2f) || fabsf(w3 - w0) > 0.01f;
      T.place.orientation.pitch = 0; p_smart_kmh = sp; p_desc_kmh = dc; T.cruise = cr; }
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
    { float sl = p_own_lim; p_own_lim = 95;
      float k1 = own_limit_k(92, 0), k2 = own_limit_k(94.2f, 0), k3 = own_limit_k(93.5f, 0.2f), k4 = own_limit_k(94.8f, -0.5f), k5 = own_limit_k(95.2f, 0);
      printf("own limiter: climb at 92 %.2f, flat 94.2 %.2f, 93.5 and accelerating %.2f, 94.8 and slowing %.2f, over the limit %.2f (expect 1.00 0.53 0.53 1.00 0.00)\n", k1, k2, k3, k4, k5);
      bad |= fabsf(k1 - 1) > 0.01f || fabsf(k2 - 0.53f) > 0.03f || fabsf(k3 - 0.53f) > 0.05f || fabsf(k4 - 1) > 0.01f || k5 != 0;
      p_own_lim = sl; }
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
                if (g_rsc_brake > *brake_max) *brake_max = g_rsc_brake;
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
        T.yaw = 0; T.speed = 0; g_rsc_level = 0; g_rsc_brake = 0; g_rsc_thr_cap = 1;
    }
    T.speed_limit = 0;
    printf("keys test: %s\n", bad ? "FAIL" : "ok");
    if (bad) exit(1);
}


// ======================================================================= Auto Hold
// Truck stops with the brake, the driver lets go, waits, then launches (gentle / hard). Signed speed, static brake,
// engine with a clutch that bites 0.15 s after the pedal (box engages when the throttle gate lets the pedal through)
// and then closes in 0.35 s. The game's gate (docs/neutral-exit-re.md): merged throttle 0 while the brake request > 0,
// unless our patch passes it (s_thr_pass). Brake request slews +-5/s like the game.
static float g_true_k = 1.0f;   // the engine really delivers this fraction of what the module thinks (model error)
struct ahres { float roll_m, jerk, max_a, req_end, t_move, req_max_after, t_jerk; int held_ok; };
static ahres ah_run(const char *name, float grade_pct, int hard, int gate, float est_k, int old_hill_start, int verbose_row)
{
    memset(GA, 0, sizeof GA);
    memset(&AH, 0, sizeof AH);
    g_low_ok = 1; g_sg_hold = 0; g_crawl_on = 0; s_gate_ok = gate; s_thr_pass = 0; s_req_ours = 0; s_want_hold = s_want_hill = 0;
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
        float u = stub_cruise(GA, (float)dt, 0);
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


// ======================================================================= ACC with a lead (own ACC vs the game's target)
// A lead vehicle with a scripted speed; the game's ACC fn writes +0x1a8 (gap) and +0x1a0 (its exp target, docs/cruise-re.md § 3)
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
        float u = stub_cruise(GA, (float)dt, 0);
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


// ======================================================================= own AEB (docs/aeb.md), cruise off unless said
// Brakes: the game slews +0x17c at 5/s; request r gives 0.4 + 10 r / (mass / 30 t) m/s2 (ts4* logs, 26-30 t); the driver's
// brake pedal 8 m/s2 at 1.0; throttle 20 kN at 1.0 (zeroed while the request is > 0). Air: every rise of the applied brake
// costs 60 psi per unit, the compressor gives 4 psi/s (rough; only for "one ramp vs pulses").
struct aebres { float min_gap, peak_dec, end_kmh, warn_lead_s, psi_drop, max_req, after_kd; int max_st, flips, hit; };
static aebres aeb_run(const char *name, int scen, float mass, float psi0)
{
    memset(GA, 0, sizeof GA); memset(&C, 0, sizeof C); memset(&OA, 0, sizeof OA); memset(&s_radar, 0, sizeof s_radar);
    memset(&AE, 0, sizeof AE); AE.gap_prev = AE.rate = AE.vl_prev = NAN; s_req_ours = 0; s_want_aeb = 0; s_aeb_was = 0; s_br_at = 0; s_air_low = 0;
    p_smooth = 1; p_own_acc = 1; g_radar_ok = 1; o_radar = fake_radar; g_low_ok = 1; g_level = 1; g_sg_ok = 0; p_ah = 0; p_hill = 0;
    o_cruise = stock_cruise; stock_thr = 0; s_aeb_owns = 1; g_sim_mass = mass;
    T.engine_on = 1; T.gear = 8; T.dgear = 8; T.eff_clutch = 0; T.parking_brake = 0; T.in_brake = 0; T.in_thr = 0; T.air_warning = 0; T.paused = 0;
    double t0 = t_now, x = 0, v = 80 / 3.6, lx = 0, lv = 0, brk = 0, psi = psi0, a_f = 0;
    int lead_on = 0, last_dir = 0;
    float cross_v = 0;
    aebres r = { 1e9f, 0, 0, -1, 0, 0, 0, 0, 0, 0 };
    double t_warn = -1, t_part = -1;
    float pedal = 0.3f, pbrake = 0; int u_hold = 0;   // a driver holding 30 % throttle who does not see it
    if (scen == 2) v = 60 / 3.6;
    if (scen == 3) v = 40 / 3.6;
    float prev_req = 0;
    for (; t_now - t0 < 40; t_now += 1 / 60.0) {
        double t = t_now - t0, dt = 1 / 60.0;
        switch (scen) {
        case 0: case 4: case 5: case 6: if (!lead_on) { lead_on = 1; lx = x + 200; lv = 0; } break;      // stopped queue 200 m ahead
        case 1: case 10: case 11: if (!lead_on) { lead_on = 1; lx = x + 150; lv = 40 / 3.6; } break;     // slow truck
        case 2: if (!lead_on && t >= 2) { lead_on = 1; lx = x + 20; lv = 30 / 3.6; }                     // cut-in at 20 m, 30 km/h, brakes to 0
                if (lead_on && t >= 3) lv = fmax(0, lv - 3 * dt); break;
        case 3: if (t >= 2 && t < 4) { lead_on = 1; lx = x + 25; lv = 0; cross_v = 10; } else { lead_on = 0; cross_v = 0; } break;  // crossing car
        case 7: if (!lead_on) { lead_on = 1; lx = x + 25; lv = 80 / 3.6; }                               // following 25 m, lead brakes 5 m/s2 to 0
                if (t >= 3) lv = fmax(0, lv - 5 * dt); break;
        case 8: case 9: if (!lead_on) { lead_on = 1; lx = x + (scen == 8 ? 12 : 7); lv = 80 / 3.6; }      // tailgating 12 / 7 m at 80, lead brakes 4 m/s2 to 0
                if (t >= 5) lv = fmax(0, lv - 4 * dt); break;
        }
        if (lead_on) lx += lv * dt;
        float gap = lead_on ? (float)(lx - x) : 0;
        if (lead_on && gap < 0.05f) { r.hit = 1; }
        // the driver
        static double kd_at; if (t == 0) kd_at = -1;
        if (scen == 4 && AE.st >= AEB_PARTIAL && kd_at < 0) { pedal = 0.9f; kd_at = t; }   // kick-down during partial braking ...
        if (scen == 4 && kd_at >= 0 && t > kd_at + 1) lead_on = 0;                          // ... and 1 s later it changed lanes: the lead is gone
        if (scen == 4 && kd_at >= 0 && t > kd_at + 0.4 && t < kd_at + 1) r.after_kd = fmaxf(r.after_kd, *(float *)(GA + GA_BRAKE_REQ));
        if (scen == 8 || scen == 9) { pedal = 0; u_hold = 1; }
        // 10: at the warning the driver indicates left and steers left; 1.5 s later the lead is beside us (gone from the radar)
        // 11: the same indicator, but no steering (it is no evasion: keeps braking)
        static double ev_at; if (t == 0) { ev_at = -1; T.in_steer = 0; T.yaw = 0; }
        if ((scen == 10 || scen == 11) && AE.st >= AEB_WARN && ev_at < 0) ev_at = t;
        if ((scen == 10 || scen == 11) && ev_at >= 0) { GA[0x1d0] = 4; if (scen == 10) { T.in_steer = 0.03f; T.yaw = 0.006f; } }
        if (scen == 10 && ev_at >= 0 && t > ev_at + 1.5) lead_on = 0;
        if (scen == 10 && ev_at >= 0 && t > ev_at + 0.8 && t < ev_at + 1.5) r.after_kd = fmaxf(r.after_kd, *(float *)(GA + GA_BRAKE_REQ));
        if (scen == 6) { pedal = 0; if (lead_on && gap < 105) pbrake = 0.5f; }         // the driver brakes in time by himself
        T.in_thr = pedal; T.in_brake = pbrake;
        T.speed = (float)v; T.accel = (float)a_f; T.air_psi = (float)psi;
        T.place.orientation.pitch = 0;
        if (lead_on && gap > 0 && gap < 100) { g_fake_radar[0] = gap; g_fake_radar[1] = cross_v > 0 ? cross_v : (float)lv; }
        else { g_fake_radar[0] = 3.4e38f; g_fake_radar[1] = 0; }
        radar_stub(GA, g_fake_radar);
        float req_before = *(float *)(GA + GA_BRAKE_REQ);
        (void)req_before;
        float u = stub_cruise(GA, (float)dt, pedal);
        float req = *(float *)(GA + GA_BRAKE_REQ);
        r.max_req = fmaxf(r.max_req, req);
        int dir = req > prev_req + 1e-4f ? 1 : req < prev_req - 1e-4f ? -1 : 0;
        if (dir && last_dir && dir != last_dir) { r.flips++; if (verbose) printf("   flip t=%.2f req %.3f st %d gap %.1f v %.1f psi %.0f a_need %.2f\n", t, req, AE.st, gap, v * 3.6, psi, AE.a_need); }
        if (dir) last_dir = dir;
        prev_req = req;
        brk = req > brk ? fmin(req, brk + 5 * dt) : fmax(req, brk - 5 * dt);   // the game's slew
        double applied = fmax(brk * 10 / fmax(1.0, mass / 30000.0), pbrake * 8.0);
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
    r.warn_lead_s = t_warn >= 0 && t_part >= 0 ? (float)(t_part - t_warn) : -1;
    if (r.min_gap > 1e8f) r.min_gap = 0;
    printf("%-50s max %-15s min gap %5.1f m, peak decel %.1f, max request %.2f, warning %.1f s before braking, air -%.0f psi, request reversals %d, end %.0f km/h%s\n",
           name, aeb_name[r.max_st], r.min_gap, r.peak_dec, r.max_req, r.warn_lead_s, r.psi_drop, r.flips, r.end_kmh, r.hit ? "  <-- HIT" : "");
    s_aeb_owns = 0; g_sim_mass = 26000; T.in_steer = 0; T.yaw = 0; GA[0x1d0] = 0;
    return r;
}

int main(int argc, char **argv)
{
    verbose = argc > 1;
    for (int i = 0; i < m_cruise.n_params; i++) *m_cruise.params[i].val = m_cruise.params[i].def;
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
        T.place.orientation.pitch = atanf(0.05f) / 6.2831853f;
        for (int k = 0; k < 30; k++) { t_now += 1 / 60.0; hill_start(GA); brake_request(GA, s_want_hill); }
        T.in_brake = 0;
        int held = 0;
        for (int k = 0; k < 300; k++) { t_now += 1 / 60.0; *(float *)(GA + GA_BRAKE_REQ) = 0; hill_start(GA); brake_request(GA, s_want_hill); if (*(float *)(GA + GA_BRAKE_REQ) > 0) held++; if (k == 90) T.in_thr = 0.3f; }
        printf("hill start 5%%: brake held %.1f s after release (gas at 1.5 s)\n", held / 60.0);
    }
    // same, game with its emergency brake option OFF (it never clears 0x17c; ts3c latch)
    {
        memset(GA, 0, sizeof GA); T.speed = 0; T.in_brake = 0.5f; T.in_thr = 0;
        for (int k = 0; k < 30; k++) { t_now += 1 / 60.0; hill_start(GA); brake_request(GA, s_want_hill); }
        T.in_brake = 0;
        int held = 0;
        for (int k = 0; k < 300; k++) { t_now += 1 / 60.0; hill_start(GA); brake_request(GA, s_want_hill); if (*(float *)(GA + GA_BRAKE_REQ) > 0) held++; if (k == 90) T.in_thr = 0.3f; }
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
                for (int k = 0; k < 120; k++) { t_now += 1 / 60.0; auto_hold(GA, 1 / 60.0f); brake_request(GA, s_want_hold); }
                T.in_brake = 0;
                float mx = 0;
                for (int k = 0; k < 120; k++) { t_now += 1 / 60.0; auto_hold(GA, 1 / 60.0f); brake_request(GA, s_want_hold); mx = fmaxf(mx, *(float *)(GA + GA_BRAKE_REQ)); }
                printf("Auto Hold blocked by %-14s: max request %.2f (must be 0)\n", b.n, mx);
                bad |= mx > 0;
            }
            g_crawl_on = 0; g_sg_hold = 0; T.parking_brake = 0; T.gear = 0; T.dgear = 0;
        }
        printf("Auto Hold test: %s\n", bad ? "FAIL" : "ok");
        if (bad) return 1;
    }
    return 0;
}
