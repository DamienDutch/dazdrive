// Offline test of the deceleration assist (modules/decel.cpp and the real actuator, core/actuate.cpp, are included: the brake request
// the plant sees is what the actuator writes, with its slews and the assist's allowance). A truck plant, a descent from 60 km/h with
// the retarder lever held, the gearbox shifting every 2 s:
//  - the retarder: F/m, the measured values of an 18 t truck with 4 steps (0.8 / 1.6 / 2.4 / 3.0 m/s2, other step counts on the same
//    curve) times 18 / mass, built up over ~1 s after the pull, faded to nothing between 6 and 4.5 km/h;
//  - the shift: the clutch open 1.1 s +- 0.125 s (uniform: p10-p90 of 1.0-1.2 s as measured) every 2.0 s; the retarder's deceleration falls to the floor (0.4 m/s2 at 18 t, F/m too) within
//    0.2 s of the clutch opening and recovers over 0.3 s after it closes; the gear flips 0.35 s +- 0.05 s before the clutch closes; the box drops to neutral under 5.2 km/h and stays there
//    (the truck then coasts at 0.1 m/s2 without brake);
//  - the service brake: an unknown gain (x0.6, x1.0, x1.5 of 6 m/s2 at request 1.0, the same at every mass) behind a 0.15 s lag;
//  - the speed telemetry has a little noise; the frame rate is 50 Hz, the game tick (the actuator's write) 100 Hz.
// Asserts per mass x gain x step count N (3, 4, 5) x lever step (first, middle, last): after 1.0 s of a gap the deceleration is within
// 0.4 of min(target, reachable), reachable = the retarder's floor + the brake's gain x max_brake (a weak brake and a heavy load cannot
// give the top step's 3.0 in a gap: such lines say 'unreachable'); no overshoot over the target above 0.5; the jerk of the
// deceleration stays within 1.5 x jerk_limit; the descent ends at v = 0 with the final jerk within jerk_limit, no reversal and no
// request left. Then the cases where it must do nothing or end at once, brake air low (no windup) and the engine brake alone.
// Soften ([decel] soften with the retarder hook; the plant's retarder, its gap floor too, times the factor the actuator merged): a
// 10 t truck whose first step alone gives 2.0 m/s2 against a 0.6 target, with the gear held and with a shift every 4 s: the
// deceleration within 0.2 of the target once settled, the factor without oscillation (sign changes of its derivative outside the
// gaps), back to 1 after the lever is released; the hook off: factor 1; a 36 t truck: factor 1 at every step (these runs with
// ramp_s 0: soften alone).
// Ramp ([decel] ramp_s with the hook): a game that brings the retarder back at full strength the moment the clutch closes (no
// recovery time) and the lower gear's engine brake with it: the measured steps hold the engine brake of the gear before (scaled
// with them, a simplification), the new gear adds a step of 0.1 m/s2 at 18 t (F/m, not scaled, none in a gap). Asserted with
// the ramp, per closing above 12 km/h for 1.0 s after it: the deceleration within the target + 0.3 (less what the gap was already
// over it as the clutch closed: the gap's own error, not the ramp's), its jerk within jerk_limit; printed against the same run
// with ramp_s 0 (before). The build-up is jerk-limited (ramp_s is only its upper bound): printed per row are the length the module gave
// it (hand-over over jerk_limit) and the time the retarder factor needed for 95 % of its way; a small hand-over (light truck, first step)
// must get a clearly shorter one than a large one (heavy truck, last step). The hook off: the run is the one with ramp_s 0, no factor posted.
// Jerk is the difference of the plant's true deceleration over 0.1 s. Outside the assert: the edges the retarder makes by itself (the
// shift's fall and recovery, the pull's build-up, the fade); they are printed (edge) next to the assist-off figure.
// Build+run: tools/sim/run_decel.sh [-v]
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
volatile float g_decel_need;
int g_log_level = 2;
incident_ctx g_inc = { 0, 0, 0, NAN, NAN };
double g_incident_mb = 0;
void logring(const char *, ...) {}
void incident(const char *, const char *, ...) {}
const char *g_truck_unsupported;
void play_cue(int) {}
static double t_now;
double wall_s() { return t_now; }
double sim_s() { return t_now; }
void csv_cap(FILE **, const char *) {}
FILE *open_out(const char *, const char *) { return NULL; }
static int verbose, vstep = 20;
static struct { double t; char why[40]; } g_shlog[64];   // the gear shifts TrueShift logged (the geared runs): when, and the reason
static int g_nshlog;
void logline(const char *fmt, ...)
{
    if (!strncmp(fmt, "shift %d->%d (%s)", 17)) {
        va_list ap; va_start(ap, fmt); va_arg(ap, int); va_arg(ap, int);
        if (g_nshlog < 64) { g_shlog[g_nshlog].t = t_now; snprintf(g_shlog[g_nshlog++].why, sizeof g_shlog[0].why, "%s", va_arg(ap, const char *)); }
        va_end(ap);
    }
    if (!verbose) return;
    va_list ap; va_start(ap, fmt); printf("%7.2f  ", t_now); vprintf(fmt, ap); printf("\n"); va_end(ap);
}
int mem_readable(const void *, size_t) { return 0; }
uintptr_t game_scan(const char *) { return 0; }
int game_patch_clutch_floor(uintptr_t, clutch_floor *) { return 0; }
int game_patch_cf0_if(uintptr_t, int, const unsigned char *) { return 0; }
char *game_actor_safe() { return NULL; }
void cruise_info_get(cruise_info *o) { memset(o, 0, sizeof *o); }   // no Auto Hold here: the assist lets go at a standstill
static float g_radius = 0.5f;
float wheel_radius() { return g_radius; }

// ---- what TrueShift and the vehicle model (linked for real, for the geared runs) need of the game: the gearbox of the geared plant below
static struct { int gear, target, state, parked, engaged, force; double shift_end; } GB;
static float g_mass_kg = 18000, GB_SHIFT_S = 1.1f;
static const float G_RATIO[12] = { 11.73f, 9.21f, 7.09f, 5.57f, 4.35f, 3.41f, 2.7f, 2.12f, 1.63f, 1.28f, 1.0f, 0.78f };   // the 12-speed of the TrueShift sim (Volvo FH 2400 Nm)
static const float G_CURVE[8][2] = { {0,0},{600,0.62f},{900,1},{1230,1},{1600,0.77f},{1700,0.69f},{1800,0.44f},{2000,0} };
static const float G_RANGES[4][2] = { {1500,1900},{750,1450},{900,1250},{1200,1650} };   // engine brake, low gear, high gear, power (down, up) rpm
static const float G_DIFF = 2.85f, G_RADIUS = 0.506f, G_TORQUE = 2400, G_LIMIT = 1900;
char *gb_inner(void *) { return (char *)&GB; }
int gb_gear(void *) { return GB.gear; }
int gb_target(void *) { return GB.target; }
int gb_state(void *) { return GB.parked ? 1 : GB.state; }
int gb_engage(void *, int g) { if (!GB.parked) return 0; GB.target = g; GB.engaged = 1; GB.parked = 0; return 1; }
int game_engage_init() { return 1; }
void gb_force(void *) { GB.force = 1; }
void gb_set_target(void *, int g) { GB.target = g; }
float gb_thr_in(void *) { return 0; }
float gb_thr_eff(void *) { return 0; }
int gb_engine(void *, engine_info *e)
{
    memset(e, 0, sizeof *e);
    e->torque = G_TORQUE; e->rpm_limit = G_LIMIT; e->n_curve = 8; e->shift_time = GB_SHIFT_S;
    memcpy(e->curve, G_CURVE, sizeof G_CURVE);
    memcpy(e->range, G_RANGES, sizeof G_RANGES);
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
float game_mass() { return g_mass_kg; }
float game_grade_deg(void *) { return 0; }
float game_adaptive() { return 3; }
int game_adaptive_blocked() { return 0; }
int game_letter(void *, char) { return 1; }
void *game_gearbox() { return (void *)&GB; }
int game_ebrake_downshift(void *) { return 0; }
float game_ebrake_level(void *) { return -1; }
static window_fn s_decide; static gear_fn s_choose, s_start; static void (*s_tail)(void *);
int game_patch_call(uintptr_t site, void *stub)
{
    if (site == 1) s_decide = (window_fn)stub;
    if (site == 2) s_choose = (gear_fn)stub;
    if (site == 4) s_start = (gear_fn)stub;
    return 1;
}
int game_patch_tail(uintptr_t, void *pre) { s_tail = (void (*)(void *))pre; return 1; }
int game_patch_stop(uintptr_t, float (*)(float, float, float)) { return 0; }
int low_owner_on() { return 0; }
int low_owner_wanted() { return 0; }
float low_stop_rpm(float idle, float, float, int) { return 0.8f * idle; }
int route_ok() { return 0; }
float route_grade(float, float) { return NAN; }
float route_max_grade(float, float) { return NAN; }
float route_min_grade(float, float) { return NAN; }
float route_crest_m(float) { return -1; }
float route_junction_m() { return -1; }
float route_curve_speed(float, float, float) { return NAN; }
void creep_info_get(creep_info *o) { memset(o, 0, sizeof *o); }
float cruise_wish_force(float) { return -1; }
float pedal_wish_force(float) { return -1; }
static float *f_window(void *, float *out, unsigned g)
{
    float k = (g - 1) / 11.0f;
    out[0] = G_RANGES[1][0] + k * (G_RANGES[2][0] - G_RANGES[1][0]); out[1] = G_RANGES[1][1] + k * (G_RANGES[2][1] - G_RANGES[1][1]);
    return out;
}
static float g_rpm_now() { return GB.gear > 0 ? fabsf(T.speed) / G_RADIUS * G_RATIO[GB.gear - 1] * G_DIFF * 9.5493f : 600; }
// the game's stock chooser, roughly: one gear toward the window (the same guess as the TrueShift sim)
static int f_choose(void *)
{
    GB.force = 0;
    if (GB.gear == 0) {
        int best = 1; float be = 1e9f;
        for (int g = 1; g <= 12; g++) { float r = fabsf(T.speed) / G_RADIUS * G_RATIO[g-1] * G_DIFF * 9.5493f; if (fabsf(r - 1100) < be) { be = fabsf(r - 1100); best = g; } }
        return best;
    }
    float w[2], r = g_rpm_now();
    f_window(0, w, GB.gear);
    if (r > w[1] && GB.gear < 12) return GB.gear + 1;
    if (r < w[0] && GB.gear > 1) return GB.gear - 1;
    return GB.gear;
}
static int f_start(void *) { return g_mass_kg > 20000 ? 1 : 4; }

#include "modules/decel.cpp"
#include "core/rsc.cpp"
#include "core/actuate.cpp"

extern module m_trueshift;

// ---- plant
static const float RET18[4] = { 0.8f, 1.6f, 2.4f, 3.0f };   // the retarder's deceleration per step of a 4-step truck at 18 t (measured)
static float ret_step18(int step, int n)                    // other step counts: the same curve over the step fraction
{
    float f = n > 1 ? (float)(step - 1) / (float)(n - 1) : 1.0f, x = f * 3, lo = floorf(x);
    int i = (int)lo > 2 ? 2 : (int)lo;
    return RET18[i] + (RET18[i + 1] - RET18[i]) * (x - (float)i);
}
static const float FLOOR18 = 0.4f, COAST = 0.1f, GAP_S = 1.1f, GAP_JITTER = 0.125f, TAIL_S = 0.35f, TAIL_JITTER = 0.05f, CYCLE_S = 2.0f, FALL_S = 0.2f, RISE_S = 0.3f, LAG_S = 0.15f, BUILD_S = 0.4f;
static const float NEUTRAL_KMH = 5.2f, NOISE = 0.005f, EB_STEP18 = 0.1f;
static const double DT = 0.01;

static unsigned rng = 12345;
static float uni() { rng = rng * 1664525u + 1013904223u; return (float)(rng >> 8) / 16777216.0f; }
static float gauss() { float s = 0; for (int i = 0; i < 4; i++) { rng = rng * 1664525u + 1013904223u; s += (float)(rng >> 8) / 16777216.0f - 0.5f; } return s * 1.732f; }

struct sc {
    float mass_t, gain;      // t, x the nominal 6 m/s2
    int n, step;
    int enabled;
    float thr, brk;          // the driver's pedals from the start
    int cruise, gear_start;
    double press_t; float press_thr, press_brk;   // a pedal pressed at this time (-1 = never)
    double aeb_t; float aeb;                       // another owner's wish from this time (-1 = never)
    double air_t0, air_t1;                         // brake air low from .. to (the actuator pauses our requests; -1 = never)
    int engine_only;                               // no retarder: the engine brake alone (0.6 m/s2 at 18 t), T.retarder 0
    int soften;                                    // the retarder hook is in (soften acts; the key is at its default 1 in every run)
    float tmin;                                    // target_min for this run (0 = the default)
    float ret_abs;                                 // the held step's deceleration at this mass, m/s2 (0 = the 18 t curve x 18 / mass)
    float cycle;                                   // s between the shifts
    double release_t;                              // the lever released at this time (-1 = never)
    float ramp_s;                                  // ramp_s for this run (-1 = the default)
    int snap;                                      // the retarder back at full strength at the close, the lower gear's engine brake with it
    sc() : mass_t(18), gain(1), n(4), step(2), enabled(1), thr(0), brk(0), cruise(0), gear_start(8), press_t(-1), press_thr(0), press_brk(0), aeb_t(-1), aeb(0),
           air_t0(-1), air_t1(-1), engine_only(0), soften(0), tmin(0), ret_abs(0), cycle(CYCLE_S), release_t(-1), ramp_s(-1), snap(0) {}
};

struct res {
    double t_stop;           // s, when the truck stood still (-1 = not within the run)
    float gap_err;           // worst |deceleration - goal| in a gap (goal = the target or, if less, the most the plant can give), m/s2
    int unreach;             // a judged gap's target was beyond the plant's reach (the retarder's floor + gain x max_brake)
    int gaps;                // how many gaps were judged
    float ovs;               // worst deceleration over the target, m/s2
    float jerk, edge, final_jerk, gap_min;   // m/s3: the rest of the descent, the retarder's own edges, the taper to the stop; the lowest deceleration in a gap
    float vmin, v_rise;      // lowest speed, largest rise of the speed (reversal)
    float req_max, req_after_stop, req_at_press;   // the request: highest, after the stop, 0.04 s after a pedal press
    float tgt, ovs_t;
    int limited;             // the request sat at max_brake
    float lvl_air0, lvl_air1;   // the assist's level when the air ran low and when it came back
    float tgt_seen;          // the assist's target at 3.0 s
    float k_min, k_rel;      // soften: the lowest retarder factor, the factor 1.0 s after the lever was released
    float soft_err;          // the worst |deceleration - target| once settled (from 8 s, above 12 km/h, outside a gap and 0.8 s after it)
    float k_settled;         // the factor at the end of that window
    int k_flips, k_flips_all;   // sign changes of the factor's derivative in that window, and over the whole run
    float gap_hi, gap_lo;    // soften: the deceleration's highest and lowest against the target in a gap and 0.8 s after it (above 12 km/h)
    float rel_hi, rel_lo, rel_jerk;   // ramp: the deceleration's highest and lowest against the target and the largest jerk, 1.0 s after a closing (above 12 km/h)
    float rel_gap, rel_rise; // the most a gap was over the target as the clutch closed, and the highest against the target less that excess of its gap
    int rels;                // how many closings were judged so
    float bu_sum; int bu_n;  // ramp: the build-up after a closing = seconds until the retarder factor has made 95 % of its way (mean over the closings that moved it)
    float rt_sum; int rt_n;  // ramp: the length the jerk limit gave the build-up (the module's own figure, D.rT), mean over the closings
};

static char buf[0x200];
static float cur_req() { return *(float *)(buf + 0x17c); }

static res run(const sc &s)
{
    res r; memset(&r, 0, sizeof r);
    rng = 12345;   // the same noise and gap lengths for every scenario, with the assist on and off
    r.t_stop = -1; r.vmin = 99; r.req_at_press = -1;
    memset(&T, 0, sizeof T);
    T.engine_on = 1; T.gear = s.gear_start; T.retarder = (unsigned)s.step; T.retarder_steps = (unsigned)s.n; T.air_psi = 120;
    if (s.engine_only) { T.retarder = T.retarder_steps = 0; T.motor_brake = 1; }
    T.in_thr = s.thr; T.in_brake = s.brk; T.cruise = s.cruise ? 10.0f : 0;
    for (auto &p : g_params) *p.val = p.def;
    p_enabled = (float)s.enabled;
    D = decltype(D)();
    if (s.tmin > 0) p_tmin = s.tmin;
    if (s.ramp_s >= 0) p_ramp = s.ramp_s;
    memset(buf, 0, sizeof buf);
    for (int o = 0; o < ACT_N; o++) { act_brake(o, 0); act_retarder_scale(o, 1); }
    g_ret_scale = 1.0f; s_ret_in = s.soften != 0;
    s_req_ours = 0; s_dec_was = 0; s_br_at = 0; s_hooked = 1;
    s_aeb_was = s_hill_was = s_hold_was = 0;
    t_now = 0;
    float k_m = 18.0f / s.mass_t, G = 6.0f * s.gain;
    float v = 60 / 3.6f, y = 0, rb = 0, hist[10] = { 0 }, tgt_full = decel_target(s.step, s.n);
    r.tgt = tgt_full;
    int gear = s.gear_start;
    double gap_t0 = -1, gap_t1 = -1, next_gap = s.cycle, taper_t = -1, frame_at = 0;
    int gaps_judged = 0, in_gap = 0, neutral = 0, step_i = 0, hi = 0;
    double judged = -1, flip_t = -1;
    float prev_v = v;
    double stop_at = -1;
    bool pressed = false;
    float k_prev = 1, dk_last = 0;
    double rel_judged = -1;
    bool rt_got = false;
    float bu_f[120]; int bu_i = 0;   // the retarder factor, every tick of the 1.0 s after a closing
    auto bu_end = [&]() {            // the closing's build-up: the time the factor needs for 95 % of its way (a factor that did not move says nothing)
        if (bu_i >= 60 && fabsf(bu_f[bu_i - 1] - bu_f[0]) > 0.05f) {
            int i = 0;
            while (i < bu_i - 1 && (bu_f[i] - bu_f[0]) / (bu_f[bu_i - 1] - bu_f[0]) < 0.95f) i++;
            r.bu_sum += (float)(i * DT); r.bu_n++;
        }
        bu_i = 0;
    };
    float a_gap = 0, gap_x = 0;
    r.k_min = 1; r.k_rel = -1; r.k_settled = -1; r.rel_hi = r.rel_rise = -99; r.rel_lo = 99;
    for (; t_now < 45; step_i++) {
        double t = t_now;
        // the driver and the other owners
        if (s.press_t >= 0 && !pressed && t >= s.press_t) { pressed = true; T.in_thr = s.press_thr; T.in_brake = s.press_brk; }
        if (s.air_t0 >= 0) {
            bool low = t >= s.air_t0 && t < s.air_t1;
            if (low && T.air_psi > 100) r.lvl_air0 = D.lvl;
            if (!low && t >= s.air_t1 && T.air_psi < 100) r.lvl_air1 = D.lvl;
            T.air_psi = low ? 60.0f : 120.0f;
        }
        if (t >= 3.0 && r.tgt_seen == 0) r.tgt_seen = D.tgt;
        if (s.aeb_t >= 0 && t >= s.aeb_t) act_brake(ACT_AEB, s.aeb);
        bool released = s.release_t >= 0 && t >= s.release_t;
        if (released) T.retarder = 0;
        // the gearbox: a shift every CYCLE_S while the speed allows one, then neutral under NEUTRAL_KMH
        if (!neutral && v * 3.6f < NEUTRAL_KMH) { neutral = 1; gear = 0; in_gap = 0; }
        if (!neutral && !in_gap && t >= next_gap && v * 3.6f > 10) {
            in_gap = 1; gap_t0 = t; gap_t1 = t + GAP_S + GAP_JITTER * (2 * uni() - 1); next_gap += s.cycle;
            flip_t = gap_t1 - TAIL_S - TAIL_JITTER * (2 * uni() - 1);
        }
        if (in_gap && flip_t >= 0 && t >= flip_t) { if (gear > 1) gear--; flip_t = -1; }
        if (in_gap && t >= gap_t1) in_gap = 0;
        T.eff_clutch = in_gap ? 1.0f : 0.0f;
        T.gear = gear;
        // the retarder
        if (released) rb -= rb * (float)(DT / BUILD_S);
        else rb += (1 - rb) * (float)(DT / BUILD_S);
        float full = s.ret_abs > 0 ? s.ret_abs * rb : (s.engine_only ? 0.6f : ret_step18(s.step, s.n)) * k_m * rb, fl = fminf(FLOOR18 * k_m, full), env = 1;
        if (gap_t0 >= 0) {
            if (in_gap) env = fmaxf(0.0f, 1 - (float)(t - gap_t0) / FALL_S);
            else if (!s.snap && t - gap_t1 >= 0 && t - gap_t1 < RISE_S && gap_t1 > gap_t0) env = (float)(t - gap_t1) / RISE_S;
        }
        float fade = fminf(fmaxf((v * 3.6f - 4.5f) / 1.5f, 0.0f), 1.0f);
        float ret = neutral ? COAST : (fl + (full - fl) * env) * fade * g_ret_scale;   // the factor of the last tick (1 without soften)
        // the service brake: the request the actuator wrote, behind the lag
        y += (cur_req() - y) * (float)(DT / LAG_S);
        float bd = G * y;
        float a = ret + bd;
        if (s.snap && !in_gap && !neutral && gear < s.gear_start) a += EB_STEP18 * k_m * fade;   // the lower gear's engine brake step (not scaled)
        // the lever and the telemetry frame (50 Hz)
        if (t >= frame_at) {
            frame_at += 0.02;
            T.speed = v + NOISE * gauss();
            de_frame();
        }
        act_brake_write(buf);   // the game tick
        if (pressed && r.req_at_press < 0 && t >= s.press_t + 0.04) r.req_at_press = cur_req();
        // judge
        float jerk = (a - hist[hi]) / 0.1f;   // against the deceleration 0.1 s ago
        hist[hi] = a; hi = (hi + 1) % 10;
        r.req_max = fmaxf(r.req_max, cur_req());
        if (cur_req() >= p_max - 1e-4f) r.limited = 1;
        bool edge = false;
        if (gap_t0 >= 0 && t >= gap_t0 && t < gap_t0 + 0.45) edge = true;
        if (gap_t1 > 0 && t >= gap_t1 - 0.2 && t < gap_t1 + 0.5) edge = true;
        if (t < 1.5) edge = true;
        // one judgement per gap, 1.0 s into it, above the fade and the taper. The goal is the target or, if the plant cannot give
        // it, the most it can: the retarder now plus the brake's gain at max_brake (the lag has settled by then)
        if (in_gap && t >= gap_t0 + 1.0 && judged != gap_t0 && v * 3.6f > 8) {
            judged = gap_t0;
            float reach = ret + G * p_max, goal = fminf(tgt_full, reach);
            if (tgt_full > reach) r.unreach = 1;
            r.gap_err = fmaxf(r.gap_err, fabsf(a - goal)); gaps_judged++;
        }
        if (s.soften && (in_gap || (gap_t1 > 0 && t < gap_t1 + 0.8)) && v * 3.6f > 12) { r.gap_hi = fmaxf(r.gap_hi, a - tgt_full); r.gap_lo = fminf(r.gap_lo, a - tgt_full); }
        if (s.snap && !in_gap && gap_t1 > gap_t0 && t >= gap_t1 && t < gap_t1 + 1.0 && v * 3.6f > 12) {
            if (rel_judged != gap_t1) { bu_end(); rt_got = false; rel_judged = gap_t1; r.rels++; gap_x = fmaxf(a_gap - tgt_full, 0); r.rel_gap = fmaxf(r.rel_gap, gap_x); }
            if (bu_i < 120) bu_f[bu_i++] = g_ret_scale;
            if (!rt_got && D.rt >= 0) { rt_got = true; r.rt_sum += D.rT; r.rt_n++; }
            r.rel_hi = fmaxf(r.rel_hi, a - tgt_full); r.rel_rise = fmaxf(r.rel_rise, a - tgt_full - gap_x); r.rel_lo = fminf(r.rel_lo, a - tgt_full); r.rel_jerk = fmaxf(r.rel_jerk, fabsf(jerk));
        }
        if (in_gap) a_gap = a;
        if (in_gap && v * 3.6f > 8) r.gap_min = r.gap_min == 0 ? a : fminf(r.gap_min, a);
        if (t > 1.5 && v * 3.6f > 8 && !neutral && a - tgt_full > r.ovs) { r.ovs = a - tgt_full; r.ovs_t = (float)t; }
        if (t > 0.2) {
            if (edge || v * 3.6f < 6.5f) { if (!neutral) r.edge = fmaxf(r.edge, fabsf(jerk)); }
            else r.jerk = fmaxf(r.jerk, fabsf(jerk));
        }
        // the stop: the taper window runs from where v = tgt^2 / (2 j) to the standstill
        if (neutral && taper_t < 0 && v < tgt_full * tgt_full / (2 * p_jerk)) taper_t = t;
        if (taper_t >= 0 && v > 0.02f && stop_at < 0) r.final_jerk = fmaxf(r.final_jerk, fabsf(jerk));
        if (verbose && step_i % vstep == 0) printf("   t=%5.2f v=%5.2f a=%5.2f ret=%4.2f bd=%4.2f req=%.3f lvl=%.3f ref=%.2f meas=%.2f j=%.1f k=%.3f %s\n", t, v * 3.6f, a, ret, bd, cur_req(), D.lvl, D.ref, D.a, jerk, D.k, in_gap ? "gap" : neutral ? "neutral" : "");
        {   // soften: the factor's course, judged once settled outside the gaps and their recovery
            float kf = g_ret_scale, dk = kf - k_prev;
            bool steady = t >= 8.0 && v * 3.6f > 12 && !in_gap && !neutral && !(gap_t1 > 0 && t < gap_t1 + 0.8) && !released;
            if (fabsf(dk) > 1e-6f) {
                if (dk_last != 0 && (dk > 0) != (dk_last > 0)) { r.k_flips_all++; if (steady) r.k_flips++; }
                dk_last = dk;
            }
            k_prev = kf;
            r.k_min = fminf(r.k_min, kf);
            if (s.release_t >= 0 && r.k_rel < 0 && t >= s.release_t + 1.0) r.k_rel = kf;
            if (steady) { r.soft_err = fmaxf(r.soft_err, fabsf(a - tgt_full)); r.k_settled = kf; }
        }
        // the truck
        prev_v = v;
        v -= a * (float)DT;
        if (v < 0) v = 0;
        r.vmin = fminf(r.vmin, v);
        r.v_rise = fmaxf(r.v_rise, v - prev_v);
        t_now += DT;
        if (v <= 0 && stop_at < 0) { stop_at = t_now; r.t_stop = t_now; }
        if (stop_at >= 0 && t_now > stop_at + 0.5) r.req_after_stop = fmaxf(r.req_after_stop, cur_req());
        if (stop_at >= 0 && t_now > stop_at + 1.5) break;
        if (s.press_t >= 0 && t_now > s.press_t + 1.0) break;   // the pedal cases only look at the moment of the press
        if (s.release_t >= 0 && t_now > s.release_t + 3.0) break;
    }
    bu_end();
    r.gaps = gaps_judged;
    return r;
}

// ---- the geared plant: the same truck, with a gearbox and TrueShift (linked for real) choosing the gears, from 60 km/h to a stop with the
// lever held. What the flat plant above does not have: the retarder's torque is on the engine side, so its deceleration at the wheels
// goes through the gear ratio (RET18 is the 18 t value at the ratio of 40 km/h at 1300 rpm, RT_REF; a lower gear brakes more, a higher
// one less); it vanishes with the clutch open (down to the same floor as above); and an engine-brake term rises with the rpm, from
// nothing a little under the engine's brake range (G_RANGES, from the engine definition) to its full strength inside it. The lever's
// step sets the share of that engine brake (step / steps, as the game adds engine brake by itself to the retarder lever). The numbers
// of the engine brake and the ratio dependence are assumptions (a plant), the proof is that the logic uses the engine's ranges, not them.
static const float RT_REF = 6.2f, EB_TORQUE = 0.35f * G_TORQUE, DRAG_TORQUE = 0.05f * G_TORQUE, ETA = 0.9f;

struct gsc {
    float mass_t, gain; int n, step;
    int dec_on, pre, dsa, hook, choice;
    float kmh0, ramp_s;
    int motor_brake;         // the engine brake lever instead of the retarder
    gsc() : mass_t(18), gain(1), n(4), step(2), dec_on(1), pre(1), dsa(1), hook(1), choice(0), kmh0(60), ramp_s(-1), motor_brake(0) {}
};
struct gshift {
    double t_open, t_close; int from, to; char why[40]; float kmh; float rpm_after;
    int pre;                                          // a hand-over ahead of this shift had begun when the clutch opened
    float bf_lo, bf_hi, in_lo, in_hi, af_lo, af_hi;   // deceleration - target in the 0.8 s before the open, from the open to the close, in the 1.0 s after the close
    float af_jerk;
    bool judged() const { return to < from && kmh >= 12 && t_open >= 2.0 && t_close > 0; }   // (not the lever's pull-in, not the last shifts before neutral)
};
struct gres {
    double t_stop;           // s (-1 = no stop)
    float brake_int, brake_int8;   // the integral of the brake request over the run (request x s), and above 8 km/h
    float rpm_max;           // highest engine rpm with the clutch closed
    int n_shift, n_assist, n_hunt;
    float assist_rpm_lo, assist_rpm_hi;   // rpm after the shifts of the assist
    float bf_lo, bf_hi, in_lo, in_hi, af_lo, af_hi;   // deceleration - target before / in / after the judged downshifts (see gshift), all of them
    float pf_lo, pf_hi, pi_lo, pi_hi, pa_lo, pa_hi;   // the same over the ones a hand-over ahead of the shift had begun for
    int n_dn, n_pre;                                  // judged downshifts, and how many had a hand-over ahead of them
    float af_jerk, bu_sum; int bu_n, n_judged;   // largest jerk after a closing, the build-up of the retarder factor (mean), closings judged
    float rt_sum; int rt_n;
    int pre_started;         // how often a hand-over ahead of a shift began
    float ret_off_s;         // the longest stretch the retarder factor stayed under 0.5 outside a gap and its ramp (a hand-over ahead of a shift that did not come)
    gshift sh[40];
};

static gres run_g(const gsc &s)
{
    gres r; memset(&r, 0, sizeof r);
    r.t_stop = -1; r.bf_lo = r.in_lo = r.af_lo = r.pf_lo = r.pi_lo = r.pa_lo = 99; r.bf_hi = r.in_hi = r.af_hi = r.pf_hi = r.pi_hi = r.pa_hi = -99; r.assist_rpm_lo = 99999;
    rng = 12345; g_nshlog = 0;
    memset(&T, 0, sizeof T);
    T.fwd_gears = 12; T.diff = G_DIFF; T.wheels = 4;
    for (int i = 0; i < 12; i++) T.ratio[i] = G_RATIO[i];
    T.engine_on = 1; T.air_psi = 120;
    if (s.motor_brake) { T.motor_brake = 1; T.retarder = 0; T.retarder_steps = 0; }
    else { T.retarder = (unsigned)s.step; T.retarder_steps = (unsigned)s.n; }
    g_mass_kg = s.mass_t * 1000; g_radius = G_RADIUS;
    for (auto &p : g_params) *p.val = p.def;
    p_enabled = (float)s.dec_on; p_pre = (float)s.pre; p_dsa = (float)s.dsa;
    if (s.ramp_s >= 0) p_ramp = s.ramp_s;
    D = decltype(D)();
    for (int i = 0; i < m_trueshift.n_params; i++) *m_trueshift.params[i].val = m_trueshift.params[i].def;
    for (int i = 0; i < m_trueshift.n_params; i++) if (!strcmp(m_trueshift.params[i].key, "choice")) *m_trueshift.params[i].val = (float)s.choice;
    memset(buf, 0, sizeof buf);
    for (int o = 0; o < ACT_N; o++) { act_brake(o, 0); act_retarder_scale(o, 1); }
    g_ret_scale = 1.0f; s_ret_in = s.hook != 0;
    s_req_ours = 0; s_dec_was = 0; s_br_at = 0; s_hooked = 1;
    s_aeb_was = s_hill_was = s_hold_was = 0;
    g_decel_need = 0;
    t_now = 0;
    vehicle_reset(RESET_HARD);
    m_trueshift.reset(RESET_HARD, "sim");

    const float k_m = 18.0f / s.mass_t, G = 6.0f * s.gain;
    const float lo = G_RANGES[0][0], hi = G_RANGES[0][1];
    float v = s.kmh0 / 3.6f, y = 0, rb = 0, hist[10] = { 0 }, tgt_full = decel_target(s.step, s.n);
    memset(&GB, 0, sizeof GB);
    {   // the gear nearest 1100 rpm
        int best = 1; float be = 1e9f;
        for (int g = 1; g <= 12; g++) { float rr = v / G_RADIUS * G_RATIO[g-1] * G_DIFF * 9.5493f; if (fabsf(rr - 1100) < be) { be = fabsf(rr - 1100); best = g; } }
        GB.gear = GB.target = best;
    }
    T.speed = v; T.gear = T.dgear = GB.gear; T.rpm = g_rpm_now();
    int neutral = 0, step_i = 0, hi_i = 0;
    double frame_at = 0, gap_t0 = -1, gap_t1 = -1, flip_t = -1, stop_at = -1;
    bool in_gap = false;
    int ns = 0;
    float a = 0, prev_v = v;
    float bu_f[120]; int bu_i = 0; bool rt_got = false;
    auto bu_end = [&]() {
        if (bu_i >= 60 && fabsf(bu_f[bu_i - 1] - bu_f[0]) > 0.05f) {
            int i = 0;
            while (i < bu_i - 1 && (bu_f[i] - bu_f[0]) / (bu_f[bu_i - 1] - bu_f[0]) < 0.95f) i++;
            r.bu_sum += (float)(i * DT); r.bu_n++;
        }
        bu_i = 0;
    };
    double judged_t1 = -1, last_t = -1; float last_v = 0, off_run = 0;
    float dh[80], vh[80]; int dhn = 0;   // the last 0.8 s of decel - target and speed (the "before" of a shift is judged when it starts)
    for (; t_now < 90; step_i++) {
        double t = t_now;
        // the gearbox: neutral under 5.2 km/h (the stop assist's part), else a shift in progress or the game's update
        if (!neutral && v * 3.6f < NEUTRAL_KMH) {
            neutral = 1; GB.gear = GB.target = 0; GB.state = 0; in_gap = false;
            if (ns > 0 && r.sh[ns - 1].t_close < 0) { r.sh[ns - 1].t_close = t; r.sh[ns - 1].kmh = 0; }   // (the box goes to neutral: that shift is not judged)
        }
        if (!neutral && GB.state && flip_t >= 0 && t >= flip_t) { GB.gear = GB.target; flip_t = -1; }
        if (!neutral && GB.state && t >= GB.shift_end) {
            GB.state = 0; in_gap = false; gap_t1 = t;
            if (ns > 0 && r.sh[ns - 1].t_close < 0) { r.sh[ns - 1].t_close = t; r.sh[ns - 1].rpm_after = v / G_RADIUS * G_RATIO[GB.gear - 1] * G_DIFF * 9.5493f; }
        }
        T.eff_clutch = in_gap ? 1.0f : 0.0f;
        T.gear = T.dgear = GB.gear;
        // the plant's forces
        float rt = GB.gear > 0 ? G_RATIO[GB.gear - 1] * G_DIFF : 0, rpm = GB.gear > 0 ? v / G_RADIUS * rt * 9.5493f : 600;
        rb += (1 - rb) * (float)(DT / BUILD_S);
        float lever = s.motor_brake ? 0.0f : (ret_step18(s.step, s.n) * k_m);
        float full = lever * (rt > 0 ? rt / RT_REF * fminf(1.0f, fmaxf(0.3f, rpm / 1300.0f)) : 1.0f) * rb, fl = fminf(FLOOR18 * k_m * (s.motor_brake ? 0.0f : 1.0f), full), env = 1;
        if (in_gap) env = fmaxf(0.0f, 1 - (float)(t - gap_t0) / FALL_S);
        float fade = fminf(fmaxf((v * 3.6f - 4.5f) / 1.5f, 0.0f), 1.0f);
        float ret = neutral ? COAST : (fl + (full - fl) * env) * fade * g_ret_scale;
        y += (cur_req() - y) * (float)(DT / LAG_S);
        float bd = G * y;
        float eb = 0, drag = 0;
        if (!neutral && !in_gap && GB.gear > 0) {
            float share = s.motor_brake ? 1.0f : (float)s.step / (float)s.n;
            float fe = fminf(fmaxf((rpm - 0.6f * lo) / (0.4f * lo), 0.0f), 1.0f);
            eb = EB_TORQUE * share * fe * rt * ETA / (G_RADIUS * g_mass_kg) * fade;
            drag = DRAG_TORQUE * (rpm / G_LIMIT) * rt * ETA / (G_RADIUS * g_mass_kg) * fade;
            r.rpm_max = fmaxf(r.rpm_max, rpm);
        }
        a = ret + bd + eb + drag;
        // the frame (50 Hz): telemetry, the vehicle model, TrueShift, the assist
        if (t >= frame_at) {
            frame_at += 0.02;
            T.sim_time = (scs_timestamp_t)(t * 1e6);
            T.speed = v + NOISE * gauss();
            T.rpm = neutral ? 600 : GB.gear > 0 ? fabsf(T.speed) / G_RADIUS * G_RATIO[GB.gear - 1] * G_DIFF * 9.5493f : 600;
            double fdt = t - last_t;
            if (fdt > 0.0 && fdt < 0.5) { float aa = (float)((fabsf(T.speed) - last_v) / fdt); T.accel += (float)(fdt / (0.5 + fdt)) * (aa - T.accel); }
            last_t = t; last_v = fabsf(T.speed);
            if (!neutral) { vehicle_frame(); m_trueshift.frame(); }
            de_frame();
        }
        // the game's gearbox update (100 Hz): the window trigger, the chooser, the tail
        if (!neutral && !GB.state && GB.gear > 0) {
            GB.target = GB.gear;
            float w[2];
            s_decide((void *)&GB, w, GB.gear);
            float rr = g_rpm_now();
            if (GB.force || rr < w[0] || rr > w[1]) { GB.target = s_choose((void *)&GB); GB.force = 0; }
            GB.engaged = 0;
            if (s_tail) s_tail((void *)&GB);
            if (GB.target != GB.gear) {
                GB.state = 1; in_gap = true; gap_t0 = t; GB.shift_end = t + GB_SHIFT_S + GAP_JITTER * (2 * uni() - 1);
                flip_t = GB.shift_end - TAIL_S - TAIL_JITTER * (2 * uni() - 1);
                if (ns < 40) {
                    gshift &g = r.sh[ns++];
                    g.t_open = t; g.t_close = -1; g.from = GB.gear; g.to = GB.target; g.kmh = v * 3.6f; g.rpm_after = 0;
                    g.why[0] = 0; g.pre = D.ph > 0.05f;
                    g.bf_lo = g.in_lo = g.af_lo = 99; g.bf_hi = g.in_hi = g.af_hi = -99; g.af_jerk = 0;
                    for (int k = 0; k < dhn; k++) if (vh[k] * 3.6f > 12) { g.bf_lo = fminf(g.bf_lo, dh[k]); g.bf_hi = fmaxf(g.bf_hi, dh[k]); }
                }
            }
        }
        act_brake_write(buf);   // the game tick
        r.brake_int += cur_req() * (float)DT;
        if (v * 3.6f > 8) r.brake_int8 += cur_req() * (float)DT;
        // judge
        float jerk = (a - hist[hi_i]) / 0.1f;
        hist[hi_i] = a; hi_i = (hi_i + 1) % 10;
        (void)hi; (void)prev_v;
        if (dhn < 80) { dh[dhn] = a - tgt_full; vh[dhn] = v; dhn++; } else { for (int k = 0; k < 79; k++) { dh[k] = dh[k + 1]; vh[k] = vh[k + 1]; } dh[79] = a - tgt_full; vh[79] = v; }
        // the shifts that downshift above 12 km/h: in (the open to the close) and after (1.0 s after the close); the before is judged above
        for (int i = 0; i < ns; i++) {
            gshift &g = r.sh[i];
            if (g.to >= g.from || g.kmh < 12 || v * 3.6f < 12) continue;
            float d = a - tgt_full;
            if (t >= g.t_open && (g.t_close < 0 || t < g.t_close)) { g.in_lo = fminf(g.in_lo, d); g.in_hi = fmaxf(g.in_hi, d); }
            else if (g.t_close >= 0 && t >= g.t_close && t < g.t_close + 1.0) { g.af_lo = fminf(g.af_lo, d); g.af_hi = fmaxf(g.af_hi, d); g.af_jerk = fmaxf(g.af_jerk, fabsf(jerk)); }
        }
        // the build-up after a closing (the retarder factor)
        if (gap_t1 > 0 && !in_gap && t >= gap_t1 && t < gap_t1 + 1.0 && v * 3.6f > 12 && !neutral) {
            if (judged_t1 != gap_t1) { bu_end(); rt_got = false; judged_t1 = gap_t1; r.n_judged++; }
            if (bu_i < 120) bu_f[bu_i++] = g_ret_scale;
            if (!rt_got && D.rt >= 0) { rt_got = true; r.rt_sum += D.rT; r.rt_n++; }
        }
        if (g_ret_scale < 0.5f && !in_gap && !neutral && D.rt < 0 && D.on) { off_run += (float)DT; r.ret_off_s = fmaxf(r.ret_off_s, off_run); }
        else off_run = 0;
        if (verbose && step_i % vstep == 0) printf("   t=%5.2f v=%5.1f g=%2d rpm=%4.0f a=%5.2f ret=%4.2f bd=%4.2f eb=%4.2f req=%.3f lvl=%.3f ph=%.2f k=%.2f kr=%.2f kb=%.3f eta=%5.2f need=%.2f %s\n", t, v * 3.6f, GB.gear, rpm, a, ret, bd, eb, cur_req(), D.lvl, D.ph, g_ret_scale, D.kr, D.k, D.on ? ts_brake_down_eta(fmaxf(D.a, 0)) : NAN, g_decel_need, in_gap ? "gap" : neutral ? "neutral" : "");
        // the truck
        prev_v = v;
        v -= a * (float)DT;
        if (v < 0) v = 0;
        t_now += DT;
        if (v <= 0 && stop_at < 0) { stop_at = t_now; r.t_stop = t_now; }
        if (stop_at >= 0 && t_now > stop_at + 1.0) break;
    }
    bu_end();
    r.n_shift = ns;
    for (int i = 0; i < ns; i++) {
        double best = 1e9;   // the reason: the log line nearest the close
        for (int k = 0; k < g_nshlog; k++) if (fabs(g_shlog[k].t - r.sh[i].t_close) < best && fabs(g_shlog[k].t - r.sh[i].t_close) < 0.3) { best = fabs(g_shlog[k].t - r.sh[i].t_close); snprintf(r.sh[i].why, sizeof r.sh[i].why, "%s", g_shlog[k].why); }
        if (!strcmp(r.sh[i].why, "braking assist") && r.sh[i].to < r.sh[i].from) {
            r.n_assist++;
            r.assist_rpm_lo = fminf(r.assist_rpm_lo, r.sh[i].rpm_after); r.assist_rpm_hi = fmaxf(r.assist_rpm_hi, r.sh[i].rpm_after);
        }
        // hunting: a shift the other way within 4 s of the one before (only the ones above the neutral point)
        if (i > 0 && r.sh[i].kmh > 0 && (r.sh[i].to - r.sh[i].from) * (r.sh[i-1].to - r.sh[i-1].from) < 0 && r.sh[i].t_open - r.sh[i-1].t_open < 4.0) r.n_hunt++;
        const gshift &g = r.sh[i];
        if (!g.judged()) continue;
        r.n_dn++;
        r.bf_lo = fminf(r.bf_lo, g.bf_lo); r.bf_hi = fmaxf(r.bf_hi, g.bf_hi); r.in_lo = fminf(r.in_lo, g.in_lo); r.in_hi = fmaxf(r.in_hi, g.in_hi);
        r.af_lo = fminf(r.af_lo, g.af_lo); r.af_hi = fmaxf(r.af_hi, g.af_hi); r.af_jerk = fmaxf(r.af_jerk, g.af_jerk);
        if (g.pre) {
            r.n_pre++;
            r.pf_lo = fminf(r.pf_lo, g.bf_lo); r.pf_hi = fmaxf(r.pf_hi, g.bf_hi); r.pi_lo = fminf(r.pi_lo, g.in_lo); r.pi_hi = fmaxf(r.pi_hi, g.in_hi);
            r.pa_lo = fminf(r.pa_lo, g.af_lo); r.pa_hi = fmaxf(r.pa_hi, g.af_hi);
        }
    }
    return r;
}

static void geared_init()
{
    G.tail_site = 5; G.decide_site = 1; G.n_choose = 1; G.choose_sites[0] = 2; G.n_start = 1; G.start_sites[0] = 4;
    G.window_fn = (uintptr_t)f_window; G.chooser_fn = (uintptr_t)f_choose; G.start_fn = (uintptr_t)f_start;
    for (int i = 0; i < m_trueshift.n_params; i++) *m_trueshift.params[i].val = m_trueshift.params[i].def;
    m_trueshift.start();
}

static int fails;
static bool bits_one(float f) { return f == 1.0f; }
static void check(bool ok, const char *what, char *why, size_t n)
{
    if (ok) return;
    size_t l = strlen(why);
    snprintf(why + l, n - l, "%s%s", l ? "; " : "", what);
}

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) if (!strcmp(argv[i], "-v")) verbose = 1;
    if ((argc == 7 || argc == 8) && !strcmp(argv[1], "-t")) {   // one scenario with the trace: -t mass_t gain n step every_nth_step
        sc s; s.mass_t = (float)atof(argv[2]); s.gain = (float)atof(argv[3]); s.n = atoi(argv[4]); s.step = atoi(argv[5]); vstep = atoi(argv[6]); verbose = 1; if (argc == 8) { s.soften = 1; s.snap = 1; s.cycle = (float)atof(argv[7]); }
        res r = run(s);
        printf("ovs at %.2f s; gap err %.2f ovs %.2f jerk %.1f edge %.1f fin %.1f stop %.1f\n", r.ovs_t, r.gap_err, r.ovs, r.jerk, r.edge, r.final_jerk, r.t_stop);
        return 0;
    }
    if (argc == 9 && !strcmp(argv[1], "-g")) {   // one geared run with the trace: -g mass_t step n dsa pre hook every_nth_step
        geared_init();
        gsc g; g.mass_t = (float)atof(argv[2]); g.step = atoi(argv[3]); g.n = atoi(argv[4]); g.dsa = atoi(argv[5]); g.pre = atoi(argv[6]); g.hook = atoi(argv[7]); vstep = atoi(argv[8]); verbose = 1;
        gres r = run_g(g);
        printf("stop %.1f s, brake integral %.2f (above 8 km/h %.2f), rpm max %.0f, %d shifts (%d by the assist), hunts %d\n", r.t_stop, r.brake_int, r.brake_int8, r.rpm_max, r.n_shift, r.n_assist, r.n_hunt);
        for (int i = 0; i < r.n_shift; i++) printf("   shift %2d>%2d at %5.1f s, %4.1f km/h (%s)%s, open %.2f s, rpm after %.0f\n", r.sh[i].from, r.sh[i].to, r.sh[i].t_open, r.sh[i].kmh, r.sh[i].why, r.sh[i].pre ? ", hand-over ahead" : "", r.sh[i].t_close - r.sh[i].t_open, r.sh[i].rpm_after);
        printf("   decel - target over %d judged downshifts: before %+.2f/%+.2f in %+.2f/%+.2f after %+.2f/%+.2f jerk %.1f; the %d with a hand-over ahead: before %+.2f/%+.2f in %+.2f/%+.2f after %+.2f/%+.2f\n",
               r.n_dn, r.bf_lo, r.bf_hi, r.in_lo, r.in_hi, r.af_lo, r.af_hi, r.af_jerk, r.n_pre, r.pf_lo, r.pf_hi, r.pi_lo, r.pi_hi, r.pa_lo, r.pa_hi);
        return 0;
    }
    int nrun = 0, nfail = 0;
    const float masses[3] = { 18, 25, 36 }, gains[3] = { 0.6f, 1.0f, 1.5f };
    const int ns[3] = { 3, 4, 5 };
    printf("descent from 60 km/h, lever held; per line: gap err = |decel - min(target, reachable)| at 1.0 s into a gap (n judged), ovs = decel over target,\n"
           "jerk = outside the retarder's own edges, edge = at them (assist off: in brackets), min = lowest decel in a gap (off), fin = jerk in the taper\n");
    for (float mass : masses) for (float gain : gains) for (int n : ns) {
        int steps[3] = { 1, (n + 1) / 2, n };
        for (int si = 0; si < 3; si++) {
            sc s; s.mass_t = mass; s.gain = gain; s.n = n; s.step = steps[si];
            res r = run(s);
            sc o = s; o.enabled = 0;
            res off = run(o);
            char why[200] = "";
            check(r.gaps > 0 && r.gap_err < 0.4f, r.unreach ? "gap err >= 0.4 (of the reachable)" : "gap err >= 0.4", why, sizeof why);
            check(r.ovs < 0.5f, "overshoot >= 0.5", why, sizeof why);
            check(r.jerk <= 1.5f * p_jerk, "jerk > 1.5 limit", why, sizeof why);
            check(r.t_stop > 0, "no stop", why, sizeof why);
            check(r.final_jerk <= p_jerk, "final jerk > limit", why, sizeof why);
            check(r.vmin >= 0 && r.v_rise <= 1e-4f, "reversal", why, sizeof why);
            check(r.req_after_stop <= 0, "request after the stop", why, sizeof why);
            nrun++;
            if (why[0]) nfail++;
            printf("%2.0ft g%.1f N%d s%d tgt %.2f | gap err %.2f (%d) ovs %+.2f jerk %.1f edge %.1f (%.1f) min %.2f (%.2f) | stop %.1fs fin %.1f req max %.2f%s%s | %s%s\n",
                   mass, gain, n, s.step, r.tgt, r.gap_err, r.gaps, r.ovs, r.jerk, r.edge, off.edge, r.gap_min, off.gap_min, r.t_stop, r.final_jerk, r.req_max,
                   r.limited ? " (max)" : "", r.unreach ? " unreachable" : "", why[0] ? "FAIL: " : "ok", why);
        }
    }
    printf("%d scenarios, %d failed\n", nrun, nfail);
    fails += nfail;

    // ---- must do nothing / must end at once
    printf("cases:\n");
    struct { const char *name; sc s; } cases[8];
    int nc = 0;
    { sc s; s.thr = 0.3f; cases[nc].name = "throttle pedal down from the start"; cases[nc++].s = s; }
    { sc s; s.brk = 0.3f; cases[nc].name = "brake pedal down from the start"; cases[nc++].s = s; }
    { sc s; s.enabled = 0; cases[nc].name = "enabled=0"; cases[nc++].s = s; }
    { sc s; s.cruise = 1; cases[nc].name = "cruise set"; cases[nc++].s = s; }
    { sc s; s.gear_start = -1; cases[nc].name = "reverse gear"; cases[nc++].s = s; }
    for (int i = 0; i < nc; i++) {
        sc s = cases[i].s; s.mass_t = 36; s.step = 4; s.n = 4; s.press_t = 6.0;   // press nothing: the run is cut at 7 s
        s.press_thr = s.thr; s.press_brk = s.brk;
        res r = run(s);
        char why[100] = "";
        check(r.req_max <= 0, "a request was posted", why, sizeof why);
        printf("  %-34s max request %.3f  %s%s\n", cases[i].name, r.req_max, why[0] ? "FAIL: " : "ok", why);
        if (why[0]) fails++;
    }
    {   // a pedal pressed while the assist brakes: the request is gone within one frame and one tick
        const char *nm[2] = { "throttle pressed at 6.0 s", "brake pedal pressed at 6.0 s" };
        for (int k = 0; k < 2; k++) {
            sc s; s.mass_t = 36; s.step = 4; s.n = 4; s.press_t = 6.0; s.press_thr = k == 0 ? 0.2f : 0; s.press_brk = k == 1 ? 0.2f : 0;
            res r = run(s);
            char why[100] = "";
            check(r.req_max > 0.05f, "the assist never braked", why, sizeof why);
            check(r.req_at_press == 0, "request still held 0.04 s after the press", why, sizeof why);
            printf("  %-34s max request %.3f, 0.04 s after the press %.3f  %s%s\n", nm[k], r.req_max, r.req_at_press, why[0] ? "FAIL: " : "ok", why);
            if (why[0]) fails++;
        }
        sc s; s.mass_t = 36; s.step = 4; s.n = 4; s.aeb_t = 6.0; s.aeb = 0.9f; s.press_t = 6.0; s.press_thr = 0; s.press_brk = 0;
        res r = run(s);
        char why[100] = "";
        check(r.req_max >= 0.9f - 1e-4f, "the AEB's request did not win", why, sizeof why);
        printf("  %-34s max request %.3f  %s%s\n", "AEB asks 0.9 at 6.0 s (priority)", r.req_max, why[0] ? "FAIL: " : "ok", why);
        if (why[0]) fails++;
    }
    {   // brake air low between two shifts: the actuator pauses our request, the level must not wind up meanwhile
        sc s; s.mass_t = 36; s.step = 4; s.n = 4; s.air_t0 = 3.3; s.air_t1 = 3.9;
        res r = run(s);
        char why[100] = "";
        check(r.lvl_air0 > 0.05f, "the assist was not braking when the air ran low", why, sizeof why);
        check(r.lvl_air1 <= r.lvl_air0 + 0.02f, "the level wound up while the air was low", why, sizeof why);
        check(r.t_stop > 0 && r.req_after_stop <= 0, "no clean stop", why, sizeof why);
        printf("  %-34s level %.3f -> %.3f  %s%s\n", "brake air low 3.3-3.9 s", r.lvl_air0, r.lvl_air1, why[0] ? "FAIL: " : "ok", why);
        if (why[0]) fails++;
    }
    {   // no retarder, the engine brake alone: the first step's target, not the last one's
        sc s; s.engine_only = 1;
        res r = run(s);
        char why[100] = "";
        check(fabsf(r.tgt_seen - p_tmin) < 1e-3f, "the target is not target_min", why, sizeof why);
        check(r.t_stop > 0 && r.req_after_stop <= 0, "no clean stop", why, sizeof why);
        printf("  %-34s target %.2f  %s%s\n", "engine brake only (no retarder)", r.tgt_seen, why[0] ? "FAIL: " : "ok", why);
        if (why[0]) fails++;
    }
    {   // soften: a light truck whose first step alone is far over the target, then the cases where the factor must stay 1
        printf("soften (10 t, step 1 of 4 alone 2.0 m/s2, target 0.6; judged from 8 s above 12 km/h, outside the gaps and 0.8 s after them):\n");
        struct { const char *name; float cycle; double rel; } lt[3] = { { "gear held", 1000, -1 }, { "a shift every 4 s", 4, -1 }, { "gear held, released at 14 s", 1000, 14 } };
        for (auto &c : lt) {
            sc s; s.mass_t = 10; s.n = 4; s.step = 1; s.tmin = 0.6f; s.ret_abs = 2.0f; s.soften = 1; s.cycle = c.cycle; s.release_t = c.rel; s.ramp_s = 0;
            res r = run(s);
            char why[160] = "";
            check(r.k_settled > 0 && r.soft_err <= 0.2f, "decel not within 0.2 of the target", why, sizeof why);
            check(r.k_flips == 0, "the factor oscillates", why, sizeof why);
            check(r.k_min >= DC_SOFT_MIN - 1e-6f, "factor under its floor", why, sizeof why);
            check(r.gap_hi < 0.5f, "overshoot >= 0.5 at a shift", why, sizeof why);
            if (c.rel >= 0) check(r.k_rel == 1.0f, "factor not 1 after the release", why, sizeof why);
            else check(r.t_stop > 0 && r.final_jerk <= p_jerk && r.req_after_stop <= 0 && bits_one(g_ret_scale), "no clean stop with factor 1", why, sizeof why);
            printf("  %-28s factor %.2f (lowest %.2f), |decel - target| <= %.2f, derivative sign changes %d (whole run %d), gap err %.2f (%d) at the shifts %+.2f/%+.2f",
                   c.name, r.k_settled, r.k_min, r.soft_err, r.k_flips, r.k_flips_all, r.gap_err, r.gaps, r.gap_hi, r.gap_lo);
            if (c.rel >= 0) printf(", 1.0 s after the release %.2f", r.k_rel);
            else printf(", stop %.1fs fin %.1f, factor at the stop %.2f", r.t_stop, r.final_jerk, g_ret_scale);
            printf("  %s%s\n", why[0] ? "FAIL: " : "ok", why);
            if (why[0]) fails++;
        }
        {   // the same truck, the hook not in: the factor is never posted
            sc s; s.mass_t = 10; s.n = 4; s.step = 1; s.tmin = 0.6f; s.ret_abs = 2.0f; s.soften = 0; s.cycle = 1000;
            res r = run(s);
            char why[100] = "";
            check(r.k_min == 1.0f && W[ACT_RET_DECEL].ret == 1.0f, "a factor was posted", why, sizeof why);
            printf("  %-28s lowest factor %.2f  %s%s\n", "hook off", r.k_min, why[0] ? "FAIL: " : "ok", why);
            if (why[0]) fails++;
        }
        for (int st = 1; st <= 4; st++) {   // a heavy truck: the step never brakes harder than its target, the factor stays 1
            sc s; s.mass_t = 36; s.n = 4; s.step = st; s.soften = 1; s.ramp_s = 0;
            res r = run(s);
            char why[100] = "";
            check(r.k_min == 1.0f, "the factor left 1", why, sizeof why);
            printf("  36t g1.0 N4 s%d tgt %.2f      lowest factor %.2f, ovs %+.2f  %s%s\n", st, r.tgt, r.k_min, r.ovs, why[0] ? "FAIL: " : "ok", why);
            if (why[0]) fails++;
        }
    }
    {   // ramp: the retarder and the engine brake back at full strength at the close
        float ramp_def = 0;
        for (auto &p : g_params) if (p.val == &p_ramp) ramp_def = p.def;
        printf("ramp (the retarder back at full strength at the close plus the lower gear's engine brake; 1.0 s after each closing above 12 km/h:\n"
               "decel - target highest/lowest, largest jerk; before = ramp_s 0, after = ramp_s %.2f; gap = the most a gap was over the target as\n"
               "the clutch closed, rise = highest less that, in gaps = lowest in the gaps and 0.8 s after; asserted after: rise <= +0.30 and jerk <= %.1f):\n", ramp_def, p_jerk);
        struct { float mass, gain, tmin, ret_abs, cycle; int step; } rc[] = {
            { 10, 1.0f, 0.6f, 2.0f, 4, 1 }, { 10, 1.0f, 0.6f, 2.0f, 2, 1 },
            { 18, 1.0f, 0, 0, 2, 1 }, { 18, 1.0f, 0, 0, 2, 2 }, { 18, 1.0f, 0, 0, 2, 4 }, { 18, 0.6f, 0, 0, 2, 2 }, { 18, 1.5f, 0, 0, 2, 2 },
            { 25, 1.0f, 0, 0, 2, 1 }, { 25, 1.0f, 0, 0, 2, 3 },
            { 36, 1.0f, 0, 0, 2, 1 }, { 36, 1.0f, 0, 0, 2, 4 }, { 36, 0.6f, 0, 0, 2, 4 },
        };
        float bu_light = -1, bu_heavy = -1, rt_light = -1, rt_heavy = -1; int rci = 0;
        for (auto &c : rc) {
            sc s; s.mass_t = c.mass; s.gain = c.gain; s.n = 4; s.step = c.step; s.tmin = c.tmin; s.ret_abs = c.ret_abs; s.cycle = c.cycle;
            s.soften = 1; s.snap = 1;
            sc b = s; b.ramp_s = 0;
            res rb = run(b), r = run(s);
            char why[160] = "";
            check(r.rels > 0, "no closing judged", why, sizeof why);
            check(r.rel_rise <= 0.3f, "decel over target + 0.3 after a closing (the gap's own excess aside)", why, sizeof why);
            check(r.rel_jerk <= p_jerk, "jerk over the limit after a closing", why, sizeof why);
            check(r.t_stop > 0 && r.req_after_stop <= 0 && bits_one(g_ret_scale), "no clean stop with factor 1", why, sizeof why);
            float bu = r.bu_n ? r.bu_sum / (float)r.bu_n : -1, rl = r.rt_n ? r.rt_sum / (float)r.rt_n : -1;
            if (rci == 0) { bu_light = bu; rt_light = rl; }
            if (rci == 10) { bu_heavy = bu; rt_heavy = rl; }
            rci++;
            printf("  %2.0ft g%.1f N4 s%d tgt %.2f every %.0f s (%d) | before %+.2f/%+.2f jerk %4.1f in gaps %+.2f | after %+.2f/%+.2f jerk %4.1f in gaps %+.2f gap %.2f rise %+.2f build-up %.2f s (jerk-limited %.2f s) | stop %.1fs  %s%s\n",
                   c.mass, c.gain, c.step, r.tgt, c.cycle, r.rels, rb.rel_hi, rb.rel_lo, rb.rel_jerk, rb.gap_lo, r.rel_hi, r.rel_lo, r.rel_jerk, r.gap_lo, r.rel_gap, r.rel_rise, bu, rl, r.t_stop,
                   why[0] ? "FAIL: " : "ok", why);
            if (why[0]) fails++;
        }
        {   // the build-up is jerk-limited: a small hand-over (light truck, first step, the retarder softened to the target) is done sooner than a large one (heavy truck, last step)
            char why[100] = "";
            check(rt_light > 0 && rt_heavy > 0 && rt_heavy - rt_light >= 0.15f, "the jerk-limited length does not differ with the size of the hand-over", why, sizeof why);
            check(bu_light > 0 && bu_heavy > 0 && bu_light <= rt_light + 0.6f && bu_heavy <= rt_heavy + 0.6f, "the factor takes more than the brake's release after the jerk-limited length", why, sizeof why);
            printf("  build-up: light truck (10 t, step 1) %.2f s jerk-limited, factor settled in %.2f s; heavy truck (36 t, step 4) %.2f s, settled in %.2f s  %s%s\n", rt_light, bu_light, rt_heavy, bu_heavy, why[0] ? "FAIL: " : "ok", why);
            if (why[0]) fails++;
        }
        {   // the hook not in: today's behavior (the same run as with ramp_s 0), no factor posted
            sc s; s.mass_t = 18; s.n = 4; s.step = 2; s.snap = 1;
            sc b = s; b.ramp_s = 0;
            res rb = run(b), r = run(s);
            char why[100] = "";
            check(r.rel_hi == rb.rel_hi && r.rel_lo == rb.rel_lo && r.rel_jerk == rb.rel_jerk && r.t_stop == rb.t_stop && r.req_max == rb.req_max && r.jerk == rb.jerk,
                  "not the same as with ramp_s 0", why, sizeof why);
            check(r.k_min == 1.0f && W[ACT_RET_DECEL].ret == 1.0f, "a factor was posted", why, sizeof why);
            printf("  %-28s after %+.2f/%+.2f jerk %4.1f, lowest factor %.2f  %s%s\n", "18t s2, hook off", r.rel_hi, r.rel_lo, r.rel_jerk, r.k_min, why[0] ? "FAIL: " : "ok", why);
            if (why[0]) fails++;
        }
    }
    {   // ---- the geared plant: A (the build-up), B (the hand-over ahead of a shift), C (the earlier downshift)
        geared_init();
        const float lo = G_RANGES[0][0], hi = G_RANGES[0][1];
        printf("geared plant (TrueShift for real; the retarder through the gear ratio, an engine brake rising with the rpm), 60 km/h to a stop, lever held, a shift is a 1.1 s gap:\n");
        printf("B: pre_blend 0 against 1 (downshift_assist 0). decel - target (m/s2) lowest/highest in the 0.8 s before the clutch opens, while it is open, in the 1.0 s after it\n"
               "closes, over the downshifts above 12 km/h from 2 s on that had a hand-over ahead (n of m); asserted for pre_blend 1: in the shift within +-0.30; before and after within +-0.40 (the loop's own tracking: the first gap's guess of the brake's gain\n"
               "is ~40 %% high in this plant, as in the flat one, and a hand-over sized with it is as far off); the jerk after the close within %.1f; the retarder never off for a second outside a shift\n", p_jerk);
        struct { float mass; int step; } br[] = { { 18, 1 }, { 18, 2 }, { 25, 1 }, { 25, 2 }, { 36, 1 }, { 36, 2 } };
        for (auto &c : br) {
            gsc g; g.mass_t = c.mass; g.step = c.step; g.dsa = 0;
            gsc g0 = g; g0.pre = 0;
            gres r0 = run_g(g0), r1 = run_g(g);
            char why[160] = "";
            check(r1.n_pre >= 1, "no hand-over ahead of any shift", why, sizeof why);
            check(r1.pi_lo >= -0.3f && r1.pi_hi <= 0.3f, "in the shift off by more than 0.30", why, sizeof why);
            check(r1.pf_lo >= -0.4f && r1.pf_hi <= 0.4f, "before the open off by more than 0.40", why, sizeof why);
            check(r1.pa_lo >= -0.4f && r1.pa_hi <= 0.4f, "after the close off by more than 0.40", why, sizeof why);
            check(r1.af_jerk <= p_jerk, "jerk after the close over the limit", why, sizeof why);
            check(r1.t_stop > 0 && r1.ret_off_s < 1.0f, "the retarder stayed off for a second outside a shift", why, sizeof why);
            printf("  %2.0ft step %d | off (all %d downshifts): %+.2f/%+.2f in %+.2f/%+.2f after %+.2f/%+.2f jerk %.1f | on (%d of %d): before %+.2f/%+.2f in %+.2f/%+.2f after %+.2f/%+.2f jerk %.1f, retarder off at most %.2f s, brake integral %.2f (off %.2f)  %s%s\n",
                   c.mass, c.step, r0.n_dn, r0.bf_lo, r0.bf_hi, r0.in_lo, r0.in_hi, r0.af_lo, r0.af_hi, r0.af_jerk, r1.n_pre, r1.n_dn, r1.pf_lo, r1.pf_hi, r1.pi_lo, r1.pi_hi, r1.pa_lo, r1.pa_hi, r1.af_jerk,
                   r1.ret_off_s, r1.brake_int, r0.brake_int, why[0] ? "FAIL: " : "ok", why);
            if (why[0]) fails++;
        }
        {   // the same without the retarder hook: only the brake side, by a small amount; it must not make the shift worse
            gsc g; g.mass_t = 25; g.step = 2; g.dsa = 0; g.hook = 0;
            gsc g0 = g; g0.pre = 0;
            gres r0 = run_g(g0), r1 = run_g(g);
            char why[120] = "";
            check(r1.in_lo >= r0.in_lo - 0.05f && r1.in_hi <= r0.in_hi + 0.05f, "worse in the shift than without the hand-over", why, sizeof why);
            printf("  25t step 2, hook off | pre_blend 0: in %+.2f/%+.2f after %+.2f/%+.2f | pre_blend 1: in %+.2f/%+.2f after %+.2f/%+.2f (the brake rises ahead of the shift by at most %.1f m/s2; the retarder stays)  %s%s\n",
                   r0.in_lo, r0.in_hi, r0.af_lo, r0.af_hi, r1.in_lo, r1.in_hi, r1.af_lo, r1.af_hi, DC_PRE_OVS, why[0] ? "FAIL: " : "ok", why);
            if (why[0]) fails++;
        }
        printf("A: the build-up after the close on the geared plant (pre_blend 0, downshift_assist 0): the length the jerk limit gave it and the time the factor needed for 95 %%\n");
        struct { float mass; int step; const char *name; } ar[] = { { 10, 1, "light truck, step 1" }, { 36, 3, "heavy truck, step 3" } };
        float rt_a[2] = { -1, -1 }, bu_a[2] = { -1, -1 };
        for (int i = 0; i < 2; i++) {
            gsc g; g.mass_t = ar[i].mass; g.step = ar[i].step; g.dsa = 0; g.pre = 0;
            gres r = run_g(g);
            rt_a[i] = r.rt_n ? r.rt_sum / (float)r.rt_n : -1; bu_a[i] = r.bu_n ? r.bu_sum / (float)r.bu_n : -1;
            char why[120] = "";
            check(r.n_dn >= 1 && r.af_jerk <= p_jerk, "jerk after the close over the limit", why, sizeof why);
            char sb[24]; if (bu_a[i] >= 0) snprintf(sb, sizeof sb, "%.2f s", bu_a[i]); else snprintf(sb, sizeof sb, "n/a (stops first)");
            printf("  %-20s %2.0f t | jerk-limited %.2f s, factor settled in %s (%d closings above 12 km/h) | after the close %+.2f/%+.2f, jerk %.1f  %s%s\n", ar[i].name, ar[i].mass, rt_a[i], sb, r.bu_n, r.af_lo, r.af_hi, r.af_jerk,
                   why[0] ? "FAIL: " : "ok", why);
            if (why[0]) fails++;
        }
        {
            char why[120] = "";
            // (the geared plant's shifts come where TrueShift puts them: a heavy truck's top steps stall out above 12 km/h, so only the order is asserted here;
            // the flat plant above holds the pair that differs by a set margin)
            check(rt_a[0] > 0 && rt_a[1] >= rt_a[0], "the large hand-over was built up faster than the small one", why, sizeof why);
            printf("  the small hand-over took %.2f s, the large one %.2f s  %s%s\n", rt_a[0], rt_a[1], why[0] ? "FAIL: " : "ok", why);
            if (why[0]) fails++;
        }
        printf("C: downshift_assist 0 against 1, step 2 of 4, both gear choices. Service brake: its integral over the stop (request x s); engine brake range %.0f-%.0f rpm, rpm limit %.0f;\n"
               "asserted: the integral lower with the assist, the rpm after an assist shift inside %.0f..%.0f (never above the range's top less a tenth of its width), the engine never over the limit, no\n"
               "up-down pair within 4 s, a stop in both:\n", lo, hi, G_LIMIT, lo, hi - 0.1f * (hi - lo));
        for (int ch = 0; ch < 2; ch++) for (float mass : { 18.0f, 36.0f }) {
            gsc g; g.mass_t = mass; g.step = 2; g.choice = ch;
            gsc g0 = g; g0.dsa = 0;
            gres r0 = run_g(g0), r1 = run_g(g);
            char why[200] = "";
            check(r1.brake_int < r0.brake_int, "the service brake's integral is not lower", why, sizeof why);
            check(r1.n_assist >= 1, "no downshift by the assist", why, sizeof why);
            check(r1.assist_rpm_hi <= hi - 0.1f * (hi - lo) && r1.assist_rpm_lo >= lo * 0.9f, "rpm after an assist shift outside the range", why, sizeof why);
            check(r1.rpm_max <= G_LIMIT && r1.rpm_max < hi, "the engine went over the limit / the range's top", why, sizeof why);
            check(r1.n_hunt == 0, "hunting (an up-down pair within 4 s)", why, sizeof why);
            check(r1.t_stop > 0 && r0.t_stop > 0 && r1.t_stop < r0.t_stop + 1.5f, "no stop / a much longer one", why, sizeof why);
            printf("  choice %d, %2.0f t | brake integral %.2f -> %.2f (%.0f %%), stop %.1f -> %.1f s | %d shifts (%d by the assist, rpm after %.0f..%.0f), max rpm %.0f, hunts %d  %s%s\n",
                   ch, mass, r0.brake_int, r1.brake_int, 100.0f * (r1.brake_int / r0.brake_int - 1.0f), r0.t_stop, r1.t_stop, r1.n_shift, r1.n_assist, r1.assist_rpm_lo, r1.assist_rpm_hi, r1.rpm_max, r1.n_hunt,
                   why[0] ? "FAIL: " : "ok", why);
            if (why[0]) fails++;
        }
        {   // decel off: TrueShift is on its own, nothing of the assist reaches it
            gsc g; g.mass_t = 36; g.step = 2; g.dec_on = 0;
            gres r = run_g(g);
            gsc g2 = g; g2.dsa = 0; g2.pre = 0;
            gres r2 = run_g(g2);
            char why[100] = "";
            check(r.n_assist == 0 && r.brake_int == 0 && g_decel_need == 0, "the assist acted with decel off", why, sizeof why);
            check(r.n_shift == r2.n_shift && r.t_stop == r2.t_stop, "decel off differs with the keys at 0", why, sizeof why);
            printf("  decel off | %d shifts, no assist shift, no brake  %s%s\n", r.n_shift, why[0] ? "FAIL: " : "ok", why);
            if (why[0]) fails++;
        }
    }
    printf("%s\n", fails ? "decel sim: FAILED" : "decel sim: all asserts passed");
    return fails ? 1 : 0;
}
