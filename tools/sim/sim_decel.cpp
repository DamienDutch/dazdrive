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
// with ramp_s 0 (before). The hook off: the run is the one with ramp_s 0, no factor posted.
// Jerk is the difference of the plant's true deceleration over 0.1 s. Outside the assert: the edges the retarder makes by itself (the
// shift's fall and recovery, the pull's build-up, the fade); they are printed (edge) next to the assist-off figure.
// Build+run: tools/sim/run_decel.sh [-v]
#include "core/suite.h"
#include "core/game.h"
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
int g_log_level = 2;
incident_ctx g_inc = { 0, 0, 0, NAN, NAN };
double g_incident_mb = 0;
void logring(const char *, ...) {}
void incident(const char *, const char *, ...) {}
const char *g_truck_unsupported;
volatile char g_dash_flash, g_dash_status;
double g_dash_flash_until;
void play_cue(int) {}
static double t_now;
double wall_s() { return t_now; }
double sim_s() { return t_now; }
void csv_cap(FILE **, const char *) {}
FILE *open_out(const char *, const char *) { return NULL; }
static int verbose, vstep = 20;
void logline(const char *fmt, ...)
{
    if (!verbose) return;
    va_list ap; va_start(ap, fmt); printf("%7.2f  ", t_now); vprintf(fmt, ap); printf("\n"); va_end(ap);
}
int mem_readable(const void *, size_t) { return 0; }
uintptr_t game_scan(const char *) { return 0; }
int game_patch_clutch_floor(uintptr_t, clutch_floor *) { return 0; }
int game_patch_cf0_if(uintptr_t, int, const unsigned char *) { return 0; }
int game_patch_call(uintptr_t, void *) { return 0; }
char *game_actor_safe() { return NULL; }
void cruise_info_get(cruise_info *o) { memset(o, 0, sizeof *o); }   // no Auto Hold here: the assist lets go at a standstill
float wheel_radius() { return 0.5f; }

#include "modules/decel.cpp"
#include "core/rsc.cpp"
#include "core/actuate.cpp"

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
            if (rel_judged != gap_t1) { rel_judged = gap_t1; r.rels++; gap_x = fmaxf(a_gap - tgt_full, 0); r.rel_gap = fmaxf(r.rel_gap, gap_x); }
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
    r.gaps = gaps_judged;
    return r;
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
    if (argc == 7 && !strcmp(argv[1], "-t")) {   // one scenario with the trace: -t mass_t gain n step every_nth_step
        sc s; s.mass_t = (float)atof(argv[2]); s.gain = (float)atof(argv[3]); s.n = atoi(argv[4]); s.step = atoi(argv[5]); vstep = atoi(argv[6]); verbose = 1;
        res r = run(s);
        printf("ovs at %.2f s; gap err %.2f ovs %.2f jerk %.1f edge %.1f fin %.1f stop %.1f\n", r.ovs_t, r.gap_err, r.ovs, r.jerk, r.edge, r.final_jerk, r.t_stop);
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
            printf("  %2.0ft g%.1f N4 s%d tgt %.2f every %.0f s (%d) | before %+.2f/%+.2f jerk %4.1f in gaps %+.2f | after %+.2f/%+.2f jerk %4.1f in gaps %+.2f gap %.2f rise %+.2f | stop %.1fs  %s%s\n",
                   c.mass, c.gain, c.step, r.tgt, c.cycle, r.rels, rb.rel_hi, rb.rel_lo, rb.rel_jerk, rb.gap_lo, r.rel_hi, r.rel_lo, r.rel_jerk, r.gap_lo, r.rel_gap, r.rel_rise, r.t_stop,
                   why[0] ? "FAIL: " : "ok", why);
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
    printf("%s\n", fails ? "decel sim: FAILED" : "decel sim: all asserts passed");
    return fails ? 1 : 0;
}
