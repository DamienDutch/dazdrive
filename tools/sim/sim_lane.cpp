// Offline test of the Lane module (modules/lane.cpp is included, so lane_step and filter_a are
// the real ones). Plant = the model FITTED ON REAL RIDES (lka-sysid notes), not an assumed wheel:
//   A'  = -(al-0.3) A + be u          (be, al speed dependent; filter_a adds the other 0.3/s of leak)
//   w'  = (-w + gA A + w_driver)/tau  (FFB wheel follows the assist offset, plus whatever the hands do)
//   o2' = -c v w - d v o3 + wind      (heading error; o3 = lane curvature term, w_bend = -(d/c) o3 = -BEND_NEED o3)
//   o1' = -kk v o2                    (lateral offset, sign as logged)
// nominal c 0.19, gA 1.0, tau 0.3, kk 0.88, d 0.85; the sweeps vary c (0.6-1.5x) and tau (0.2-0.45 s).
// Hands-on = a driver who steers the bend himself (-3.3 o3, 75% of the need) with a hand tremor.
// Build+run: tools/sim/run_lane.sh
#include "core/suite.h"
#include "core/vclass.h"
#include "sim_env.h"  // SIM_SET=section.key=value,... overrides ini defaults (sweeps)
#include "core/game.h"
#include "core/lanehook.h"
#include "sim_layout.h"
#include <math.h>
#include <initializer_list>
#include <utility>
#include <algorithm>
#include <stdarg.h>
#include <string.h>
#include <string>
#include <vector>

telemetry T;
char g_dir[MAX_PATH];
const layout *L = &sim_layout;
#define GA_U        (L->a_lka_u)      // the LKA control output the law writes (per-layout)
#define V_A         (L->i_assist)     // the vehicle's LKA assist state A
#define V_STEER_IN  (L->i_steer_in)   // the vehicle's steering input
#define V_STEER     (L->i_steer)      // the vehicle's effective steering (0x640 in the sim layout; lane.cpp reads it through sense_steering)
volatile int g_rolling, g_cruise_smooth;
volatile float g_roll_boost_kmh;
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
const vclass_profile *vprof() { return &k_vprof_truck; }   // (this sim is a truck; the vehicle model is not linked here)
static int g_sim_class = 1;   // VC_TRUCK (core/vehicle.h comes in with lane.cpp, below)
int vclass_active() { return g_sim_class; }
static double t_now;
double wall_s() { return t_now; }
double sim_s() { return t_now; }
void csv_cap(FILE **, const char *) {}
FILE *open_out(const char *n, const char *m)  // only the profile file is real (SIM_DIR), csv traces are off
{
    size_t l = strlen(n); const char *d = getenv("SIM_DIR");
    if (!d || l < 4 || strcmp(n + l - 4, ".txt")) return NULL;
    char path[600]; snprintf(path, sizeof path, "%s/%s", d, n); return fopen(path, m);
}
static char sim_wheel[64] = "SimWheel";
const char *input_wheel_name() { return sim_wheel; }
static int verbose = getenv("VERBOSE") != NULL;
void logline(const char *fmt, ...)
{
    if (!verbose) return;
    va_list ap; va_start(ap, fmt); printf("%7.2f  ", t_now); vprintf(fmt, ap); printf("\n"); va_end(ap);
}
static std::vector<int> g_cues;   // the tones played, in order (the calibration and key tests read them)
void play_cue(int c) { g_cues.push_back(c); }
static double g_rp_act; static int g_rp_n, g_rp_hi;   // replay: the driver-activity reading over the rows
static int sim_route_ok; static float sim_route_straight = 1e9f;   // the route look-ahead: none, or straight for this many meters
int route_ok() { return sim_route_ok; }
float route_straight_m(float) { return sim_route_ok ? sim_route_straight : -1; }
uintptr_t game_scan(const char *) { return 0; }
int game_patch_call(uintptr_t, void *) { return 0; }
float *game_patch_rip_float(uintptr_t, int, int, float) { return NULL; }
int game_patch_store_xmm8(uintptr_t, float (*)(float)) { return 0; }
static char GA[0x20000], VEH[0x1500], GB[64];
char *game_actor_safe() { return GA; }
void *game_gearbox() { return GB; }
char *gb_inner(void *) { return VEH; }
int mem_readable(const void *, size_t) { return 1; }
// core/lanehook.cpp is the game surface of the hook installer; the sim never installs the hook (stub_law and filter_a are called directly)
int lanehook_law(lane_law_fn, lane_law_fn *) { return LH_NOT_FOUND; }
int lanehook_min_speed_patch(float) { return 0; }
int lanehook_min_speed_ok() { return 0; }
void act_lane_min_speed(float) {}
int lanehook_assist_store(float (*)(float)) { return 0; }

// Cruise's side of the lane key with needs_cruise (modules/cruise.cpp cruise_lane_engage, tested in sim_cruise): here it only says yes or no, and for yes the game's cruise byte shows
static int g_eng_ok = 1, g_eng_n, g_eng_lag; static const char *g_eng_why = "under the cruise's minimum switch-on speed";
int cruise_lane_engage(const char **why, float *set_kmh)
{
    g_eng_n++; *why = g_eng_why;
    if (!g_eng_ok) return 0;
    if (!g_eng_lag) T.cruise = 80 / 3.6f;   // (the game's byte; with lag: the cruise never shows)
    if (set_kmh) *set_kmh = 90;
    return 1;
}

#include "modules/lane.cpp"
#include "core/display.cpp"

// ---- plant
struct pl { double o1, o2, w, A, tr; };
struct pp {
    float c = 0.19f, gA = 1.0f, tw = 0.3f, kk = 0.88f, d = 0.85f;
    float ag = 1.0f;  // actuator gain: the game's A per request u relative to the ETS2 fit (ATS 1.61 trucks: ~0.15, A' = -al A + ag be u)
    pp() {
        if (getenv("PL_AG")) ag = atof(getenv("PL_AG"));  // plant override for what-if runs (ts4n: hands off c .155 gA 1.2 tau .45; in the 283 s ringing c .21 gA 2.5 tau .24)
        if (getenv("PL_C")) c = atof(getenv("PL_C"));
        if (getenv("PL_GA")) gA = atof(getenv("PL_GA"));
        if (getenv("PL_TAU")) tw = atof(getenv("PL_TAU"));
        if (getenv("PL_DELAY")) delay = atof(getenv("PL_DELAY"));
        if (getenv("PL_DB")) db = atof(getenv("PL_DB"));
    }
    float delay = 0;  // pure delay A -> wheel (s); ts4n hands-free: wheel = 1.34 A(t-0.2), tau ~0
    float stick = 0;  // stiction (rad): the wheel only moves once its target is this far away (hysteresis; the crash replay fits best with some)
    float db = 0;     // centering band of the FFB wheel (rad): it stops db short of G A (ts5b crash fit: wheel = 2.95 A(t-0.33) - 0.010 sign)
};
static int g_sim_lost;  // 1 = the game reports no lane (o1 = o2 = o3 = 0)
static int via_stub;  // 1 = the request goes through stub_law (gating, filter_a timing) like in the game
static float vkmh = 80;
static float DTS = 1.0f / 60;
static int quiet;  // 1 = no noise at all (step response)
static unsigned rng = 12345;
static float noise() { rng = rng * 1664525u + 1013904223u; return ((rng >> 8) & 0xffff) / 65535.0f - 0.5f; }
static float gauss() { return (noise() + noise() + noise() + noise()) * 0.866f * 1.0f; }  // ~N(0,0.5)

static void learn_track_reset();   // the learner's contract check below
struct cfg { const char *name; int mode; };
static struct { const char *k; float v; } ovr[32];
static int novr;
static void apply(const cfg &c)
{
    for (auto &p : g_params) *p.val = p.def;
    p_mode = c.mode; p_needs_cc = 0;   // the sim has no cruise: lane steers by itself here (needs_cruise is tested separately)
    p_bias = getenv("BIAS") ? atof(getenv("BIAS")) : 0; p_bend_out = getenv("BEND_OUT") ? atof(getenv("BEND_OUT")) : 0; p_learn = getenv("LEARN") ? atof(getenv("LEARN")) : 0; p_sway = getenv("SWAY") ? atof(getenv("SWAY")) : 0;  // the old scenarios test the fixed law
    L_cur = L_base = { 1, 1, p_l_max }; L_trial = 0; L_hold = 0; L_t = 0; L_Nw = L_Syy = L_Cxx = L_Cxy = L_Cyy = 0; for (int k = 0; k < LK_NK; k++) L_Sxx[k] = L_Sxy[k] = 0;
    L_scn = L_scs = L_sc_zc = L_since = 0; L_b_valid = 0; L_ring_cool = 0; L_nupd = 0; SW_fac = 1; SW_t = 0; SW_hold_until = 0; SW_events = 0; SW_lastact = -1e9; L_sess = 1; L_sess_at = -1e9; L_loaded = 0; L_dirty = 0; learn_block_reset(); L_dev_blocks = 0; L_blocks_ok = L_blocks_bad = 0; L_rolled = 0;
    if (c.mode == 2) for (int i = 0; i < novr; i++) for (auto &p : g_params) if (!strcmp(p.key, ovr[i].k)) *p.val = ovr[i].v;
    p_min_kmh = 5; g_ffb = 1;  // the plant model is an FFB wheel
    g_ag = 1; AG_aa = AG_hh = AG_w = 0; AG_t = -1e9; AG_hold_until = -1e9; AG_known = 0; AG_snapped = 1; AG_ah = 0; AG_dirty = 0;   // the actuator gain is measured anew
    lane_reset(); g_u = 0; g_o1_prev = 0; g_o2_prev = 0;
    L_sess_ann = 1; AG_ann = 1; learn_track_reset();
}

static layout g_lay161;   // the sim layout as a 1.61 game (game_is_161()): ATS rows run on it
static void use_layout(int l161) { if (!g_lay161.name) { g_lay161 = sim_layout; g_lay161.tail_jmp = 7; } L = l161 ? &g_lay161 : &sim_layout; LE.is161 = sense_is_161(); }

static float g_sim_uraw;
static void sim_game_law(void *ga, float, float, float, float) { *(float *)((char *)ga + GA_U) = g_sim_uraw; }
// one physics + control step. drv: 0 hands off, 1 driver steers the bend (+ tremor), 2 evasive (w_cmd)
static double g_refmax_off;   // largest |aim| of shared steering in a hands-off run: must stay ~0 (a hands-off truck must not take over its own offset)
static void print_refmax() { printf("\nshared steering: largest |aim| with hands off: %.3f m (must be < 0.05)\n", g_refmax_off); }

// WP-LA6: the learner's contract, checked on every plant step of every scenario (core/lane learner: s, dfac, the ring ceiling, the actuator gain, the session factor).
//  - a learned value does not move without a line that announces it (LEARN_LOG sets its bit in g_lv_mask in the same call): s, dfac and the ceiling at the step they change, the
//    actuator gain and the session factor (which move in small steps) when they are 5 % / 0.05 from the last announced value (the check allows 6 % / 0.06);
//  - a learner write stays inside its bounds (a value the harness put outside them may only move toward them): s and the ceiling in [learn_min, the stored maximum] (learn_max 1.0, 1.4 only on a 1.61 game, only a calibration sets it), dfac in
//    [0.5, 1], the session factor in [0.4, 1], the actuator gain in [AG_MIN, AG_MAX], the scale the law applies never over the stored maximum.
// The harness itself writes these values between runs (apply(), the ATS rows): a jump of the clock, or apply(), starts the baseline again.
struct lv_state { double t; float s, dfac, cap, ag, sess; int init; };
static lv_state LT;
static long g_lv_steps, g_lv_viol; static int g_lv_n[5]; static char g_lv_first[260];
static void learn_track_reset() { LT.init = 0; g_lv_mask = 0; }
static void learn_track()
{
    const unsigned ann = g_lv_mask; g_lv_mask = 0;
    const bool jump = !LT.init || t_now < LT.t || t_now - LT.t > 1.0;
    auto viol = [&](const char *what, float was, float now) { if (!g_lv_viol++) snprintf(g_lv_first, sizeof g_lv_first, "t %.2f: %s %.4f -> %.4f", t_now, what, was, now); };
    g_lv_steps++;
    for (int i = 0; i < 5; i++) g_lv_n[i] += (ann >> i) & 1;
    if (!jump) {
        const float smax = s_stored_max();
        if (fabsf(L_cur.s - LT.s) > 1e-5f) { if (!(ann & LV_S)) viol("s moved with no line", LT.s, L_cur.s); if (L_cur.s < fminf(p_l_min, LT.s) - 1e-4f || L_cur.s > fmaxf(smax, LT.s) + 1e-4f) viol("s outside its bounds", LT.s, L_cur.s); }
        if (fabsf(L_cur.dfac - LT.dfac) > 1e-5f) { if (!(ann & LV_DFAC)) viol("dfac moved with no line", LT.dfac, L_cur.dfac); if (L_cur.dfac < fminf(0.5f, LT.dfac) - 1e-4f || L_cur.dfac > fmaxf(1.0f, LT.dfac) + 1e-4f) viol("dfac outside its bounds", LT.dfac, L_cur.dfac); }
        if (fabsf(L_cur.cap - LT.cap) > 1e-5f) { if (!(ann & LV_CAP)) viol("ceiling moved with no line", LT.cap, L_cur.cap); if (L_cur.cap < fminf(p_l_min, LT.cap) - 1e-4f || L_cur.cap > fmaxf(smax, fmaxf(LT.cap, LT.s)) + 1e-4f) viol("ceiling outside its bounds", LT.cap, L_cur.cap); }
        if (!(ann & LV_AG) && fabsf(g_ag / LT.ag - 1) > 0.06f) viol("actuator gain moved with no line", LT.ag, g_ag);
        if (!(ann & LV_SESS) && fabsf(L_sess - LT.sess) > 0.06f) viol("session factor moved with no line", LT.sess, L_sess);
        if (g_ag < AG_MIN - 1e-6f || g_ag > AG_MAX + 1e-6f) viol("actuator gain outside its bounds", LT.ag, g_ag);
        if (L_sess < 0.4f - 1e-4f || L_sess > 1.0f + 1e-4f) viol("session factor outside its bounds", LT.sess, L_sess);
    }
    if (p_learn > 0 && p_mode >= 2 && !jump && scale_s() > fmaxf(s_stored_max(), LT.s) * 1.0001f) viol("the scale the law applies is over the stored maximum", LT.s, scale_s());
    LT.t = t_now; LT.s = L_cur.s; LT.dfac = L_cur.dfac; LT.cap = L_cur.cap; LT.init = 1;
    if (jump || (ann & LV_AG)) LT.ag = g_ag;
    if (jump || (ann & LV_SESS)) LT.sess = L_sess;
}

static float g_sim_do1, g_sim_do2;   // what the game's lane reading adds to the truck's real offset / heading error (a prefab whose lane reference jumps); 0 = the reading is true
static int g_sim_lat;                // 1 = the plant reports its yaw rate and steering in T (the lateral load the lane assist backs off from); 0 = T stays as the older rows have it
static void step(pl &p, const pp &m, float dt, float o3, int drv, float w_cmd, float wind, float *u_game_out = NULL)
{
    float kmh = vkmh, v = kmh / 3.6f;
    T.speed = v;  // lane.cpp gates on the speed (keep_on: no request below min_kmh)
    const float p1 = (float)p.o1 + g_sim_do1, p2 = (float)p.o2 + g_sim_do2;   // (the true values while the reading is true)
    float uraw = clampf(kmh * (0.075f * p2 - 0.00275f * p1 - 0.5f * o3), -1, 1);
    float before = *(float *)(GA + GA_U);
    *(float *)(GA + GA_U) = uraw != before ? uraw : uraw + 1e-6f;
    float u = *(float *)(GA + GA_U);
    *(float *)(VEH + V_STEER_IN) = (float)p.w;
    *(float *)(VEH + V_A) = (float)p.A;
    float o1m = p1, o2m = p2 + (quiet ? 0 : 0.0007f * gauss());  // lane sensor noise
    float out;
    if (via_stub) {  // the game's law writes clamp(u_raw) (it already did above), stub_law wraps it as in the game
        *(float *)(GA + GA_U) = before;
        o_law = sim_game_law; g_sim_uraw = uraw;
        float o3s = o3;
        if (g_sim_lost) { o1m = o2m = 0; o3s = 0; }   // the lane is lost: the game hands over zeros
        stub_law(GA, o1m, o2m, o3s, dt);
        out = *(float *)(GA + GA_U);
    } else {
        out = lane_step(u, o1m, o2m, o3, dt, (float)p.w, (float)p.A, kmh);
        *(float *)(GA + GA_U) = out;
        g_u = out;
    }
    if (u_game_out) *u_game_out = u;
    float be = 0.235f - 0.0010f * kmh, al = 2.6f + 0.016f * kmh - 0.3f;
    p.A += dt * (-al * p.A + m.ag * be * out);
    p.A = filter_a((float)p.A);
    // driver: tremor is an OU process on the wheel
    if (!quiet) p.tr += dt * (-p.tr / 0.6f) + sqrtf(dt) * 0.0045f * gauss() * 1.6f;
    float wd = 0;
    if (drv == 1) wd = -3.3f * o3 + (float)p.tr;
    if (drv == 2) wd = w_cmd + (float)p.tr;
    // a driver who looks at the road (the real one does): the commanded wheel plus a correction of the heading error and the offset he sees, 1 s heading time constant and 3 s
    // for the offset (wheel per rad = 1 / (c v T): 0.365 at 14 m/s)
    if (drv == 3) wd = w_cmd + 0.365f * ((float)p.o2 - 0.026f * (float)p.o1) * (14.0f / fmaxf(v, 5.0f)) + (float)p.tr;
    static double hist[4096]; static int hn; int dl = (int)(m.delay / dt + 0.5);
    static double lastt = -1; if (t_now < 1e-9 || t_now <= lastt) { hn = 0; for (auto &h : hist) h = 0; }  // a new run (also one that starts at a ride's clock)
    lastt = t_now;
    hist[hn & 4095] = p.A; double Ad = hist[(hn - dl) & 4095]; hn++;
    // centering band (no hysteresis, the logs show none): the wheel stops db short of G A, small-signal slope ~1 (ts5b bins
    // |A| .0075/.0125/.0175/.0225 -> 1.28/1.92/2.11/2.17 x; G 2.5, db .007 gives 1.65/1.95/2.1/2.2)
    double tgt = m.gA * Ad;
    if (m.db > 0 && m.gA > 1.2f) tgt -= m.db * tanh(Ad / (m.db / (m.gA - 1.0)));
    double err = tgt + wd - p.w;
    if (fabs(err) > m.stick) p.w += dt * (err - (err > 0 ? m.stick : -m.stick)) / m.tw;
    const double o2d = -m.c * v * p.w - m.d * v * o3 + wind;
    if (g_sim_lat) { T.yaw = (float)(-(v * o3 + o2d) / 6.2831853); T.in_steer = (float)p.w; }   // (left +; the lane term o3 is + in a right bend, the heading error + to the right)
    p.o2 += dt * o2d;
    p.o1 += dt * (-m.kk * v * p.o2);
    learn_track();
}

struct res { double rms, maxe, bendmax, bendss, per; int zc; double Apk; };

static float ou_o3(float &s, float dt, float sd) { s += dt * (-s / 8.0f) + sqrtf(dt) * sd * 0.55f * gauss() * 2.0f; return s; }

// scenario: 0 straight with a wandering road (gentle bends), 1 bend R~300 (o3 0.0035) entered at 10 s
static res run(const cfg &c, const pp &m, int scen, int drv, float kmh, float T = 70, float start_o1 = 0.5f)
{
    apply(c); vkmh = kmh; rng = 4242;
    pl p = {start_o1, 0, 0, 0, 0};
    res r = {};
    float dt = DTS, os = 0; double sum = 0, sb = 0; int n = 0, nb = 0; double prev = p.o1; int hold = 0;
    double firstzc = -1, lastzc = -1;
    for (int i = 0; i < (int)(T / dt); i++) {
        t_now = i * dt;
        float o3;
        if (scen == 0) o3 = ou_o3(os, dt, 0.0012f);
        else { float k = clampf((t_now - 10) / 3, 0, 1) - clampf((t_now - 45) / 3, 0, 1); o3 = 0.0035f * k * k * (3 - 2 * k); }
        step(p, m, dt, o3, drv, 0, 0.0006f * gauss());
        if (drv == 0) { g_refmax_off = fmax(g_refmax_off, fabs(g_ref)); if (fabs(g_ref) > 0.05 && getenv("WHY_REF") && fmod(t_now, 1.0) < dt) printf("   aim %.3f at %.1f s (scen %d, %.0f km/h): o1 %.2f act %.2f act_t %.1f steady %.1f hold_t %.1f pm %.4f dev %.4f wrate %.3f\n", g_ref, t_now, scen, kmh, p.o1, g_act, g_act_t, g_steady_t, g_hold_t, g_pm, g_dev, g_w_rate); }
        if (t_now > 8) {
            sum += p.o1 * p.o1; n++; r.maxe = fmax(r.maxe, fabs(p.o1));
            if (fabs(p.o1) > 0.1) { int s = p.o1 > 0 ? 1 : -1; if (hold && s != hold) { r.zc++; if (firstzc < 0) firstzc = t_now; lastzc = t_now; } hold = s; }
        }
        if (scen == 1 && t_now > 10 && t_now < 50) { r.bendmax = fmax(r.bendmax, fabs(p.o1)); if (t_now > 38 && t_now < 45) { sb += p.o1; nb++; } }
        r.Apk = fmax(r.Apk, fabs(p.A));
    }
    r.rms = sqrt(sum / n);
    r.bendss = nb ? sb / nb : 0;
    r.per = r.zc > 1 ? 2 * (lastzc - firstzc) / (r.zc - 1) : 0;  // full period from zero crossings (0 = no oscillation)
    return r;
}

// driver steers himself: 1.2 m evasive offset (3 s out, 2 s hold, 3 s back), hands on; then hands off 20 s
static void run_driver(const cfg &c, const pp &m, float kmh, double *peakA, double *opp, double *os, double *late)
{
    apply(c); vkmh = kmh; rng = 99;
    pl p = {0, 0, 0, 0, 0}; float dt = DTS; double sumopp = 0; int nopp = 0; *peakA = 0; *os = 0; *late = 0; int nl = 0;
    for (int i = 0; i < 60 * 40; i++) {
        t_now = i * dt; double ts = t_now - 3, des = 0;  // des = wanted o1 (the lane center is 0)
        if (ts > 0 && ts < 3) des = 1.2 * 0.5 * (1 - cos(M_PI * ts / 3)); else if (ts >= 3 && ts < 5) des = 1.2;
        else if (ts >= 5 && ts < 8) des = 1.2 * 0.5 * (1 + cos(M_PI * (ts - 5) / 3));
        int hand = ts > -0.3 && ts < 8.5;
        float v = vkmh / 3.6f;
        double o2des = clampf((float)(0.6 * (p.o1 - des) / (m.kk * v)), -0.06f, 0.06f);
        double wc = 1.5 * (p.o2 - o2des) / (m.c * v);
        step(p, m, dt, 0, hand ? 2 : 0, (float)wc, 0.0004f * gauss());
        if (hand && ts > 0 && ts < 8) { *peakA = fmax(*peakA, fabs(p.A)); if (p.A * wc < 0) { sumopp += fabs(p.A); } nopp++; }
        if (ts > 8.5) *os = fmax(*os, fabs(p.o1));
        if (ts > 14) { *late += fabs(p.o1); nl++; }
    }
    *opp = sumopp / nopp; *late /= nl;
}


// shared steering (24-10): the driver steers to +off m in 2 s (a cyclist), holds hold_s, lets go (hands off) and the truck is on its own for 30 s.
// Returns: mean |A| against the hand while holding, the overshoot to the other side after letting go, |o1| 12 s after letting go,
// the aim g_ref at the end and its peak, and the zero crossings of o1 after letting go.
struct fres { double opp, os, late, ref_end, ref_pk; int zc; };
static fres run_follow(const cfg &c, const pp &m, float kmh, float off, float hold_s, int follow)
{
    apply(c); vkmh = kmh; rng = 99; p_follow = follow;
    pl p = {0, 0, 0, 0, 0}; float dt = DTS; fres r = {}; double sopp = 0; int nopp = 0, last = 0;
    double t_rel = 3 + 2 + hold_s;
    for (int i = 0; i < (int)((t_rel + 30) / dt); i++) {
        t_now = i * dt; double ts = t_now - 3, des = 0;
        if (ts > 0 && ts < 2) des = off * 0.5 * (1 - cos(M_PI * ts / 2)); else if (ts >= 2 && ts < 2 + hold_s) des = off;
        int hand = ts > -0.3 && ts < 2 + hold_s;
        float v = vkmh / 3.6f;
        double o2des = clampf((float)(2.0 * (p.o1 - des) / (m.kk * v)), -0.06f, 0.06f);   // a hand that holds its position (stiffer than run_driver's)
        double wc = 3.0 * (p.o2 - o2des) / (m.c * v);
        step(p, m, dt, 0, hand ? 2 : 0, (float)wc, 0.0004f * gauss());
        if (hand && ts > 5) { if (p.A * wc < 0) sopp += fabs(p.A); nopp++; }
        r.ref_pk = fmax(r.ref_pk, fabs(g_ref));
        if (getenv("TRACE_F") && fmod(t_now, 0.5) < dt && t_now < t_rel + 8) printf("    %5.1f o1 %6.3f o2 %7.4f A %8.5f w %8.5f act %.2f pm %8.5f dev %8.5f hold_t %.1f ref %.3f steady %.1f wpred %.5f wg %.2f\n", t_now, p.o1, p.o2, p.A, p.w, g_act, g_pm, g_dev, g_hold_t, g_ref, g_steady_t, g_wpred, g_wg);
        double tr = t_now - t_rel;
        if (tr > 0) {
            if (p.o1 * off < -0.0) r.os = fmax(r.os, fabs(p.o1));
            if (fabs(p.o1) > 0.1) { int sg = p.o1 > 0 ? 1 : -1; if (last && sg != last) r.zc++; last = sg; }
            if (tr > 12 && tr < 14) r.late = fmax(r.late, fabs(p.o1));
        }
    }
    r.opp = nopp ? sopp / nopp : 0; r.ref_end = g_ref;
    return r;
}

static int follow_tests()
{
    printf("Shared steering: driver steers to the offset in 2 s, holds, lets go; 30 s hands off after\n");
    printf("%-30s %5s %5s %6s | %-9s | mean |A| against the hand | overshoot | |o1| 12 s later | aim end / peak | zc\n", "plant", "km/h", "off", "hold", "follow");
    struct pv { const char *n; float c, gA, tw, dl; };
    const pv pls[] = { { "nominal sim plant", 0.19f, 1.0f, 0.3f, 0.0f }, { "ts4n large-signal fit", 0.155f, 1.34f, 0.05f, 0.2f }, { "slow wheel (delay .35)", 0.155f, 1.34f, 0.15f, 0.35f } };
    const cfg c2 = { "mode 2", 2 };
    int bad = 0;
    for (auto &q : pls) for (float kmh : { 60.f, 80.f }) for (float off : { 0.5f, -0.5f }) for (float hold : { 6.f, 3.f }) {
        if (getenv("ONLY_F") && !(kmh == 60 && off > 0 && hold > 4 && q.c > 0.18f && q.c < 0.2f)) continue;
        pp m; m.c = q.c; m.gA = q.gA; m.tw = q.tw; m.delay = q.dl;
        fres a = run_follow(c2, m, kmh, off, hold, 0), b = run_follow(c2, m, kmh, off, hold, 1);
        printf("%-30s %5.0f %5.1f %5.0f | off       | %.4f | %.2f | %.2f | %.2f / %.2f | %d\n", q.n, kmh, off, hold, a.opp, a.os, a.late, a.ref_end, a.ref_pk, a.zc);
        printf("%-30s %5s %5s %6s | on        | %.4f | %.2f | %.2f | %.2f / %.2f | %d\n", "", "", "", "", b.opp, b.os, b.late, b.ref_end, b.ref_pk, b.zc);
        // criteria: less fighting, no worse after letting go, ends back at the center
        if (hold >= 6 && !(b.opp < 0.6 * a.opp)) { printf("   FAIL: still fights (%.4f vs %.4f)\n", b.opp, a.opp); bad++; }
        if (b.os > a.os + 0.15 || b.late > 0.25 || b.zc > a.zc + 1) { printf("   FAIL: worse after letting go\n"); bad++; }
        if (fabs(b.ref_end) > 0.05) { printf("   FAIL: aim did not return\n"); bad++; }
    }
    printf("%s (%d failures)\n", bad ? "FOLLOW TESTS FAILED" : "follow tests ok", bad);
    return bad != 0;
}

// An S-bend at speed (24-10, 919 s: the curvature +0.0065 -> -0.005 within 1.2 s at 85 km/h, hands off, learned scale 1.27): peak offset and
// heading error, ceiling frames of the request, and whether the swings die out.
struct sres { double max_o1, max_o2; int sat, zc_late; };
static sres run_sbend(const cfg &c, const pp &m, float kmh, float lscale, int guard)
{
    apply(c); vkmh = kmh; rng = 7; p_learn = 1; L_cur.s = lscale; L_base = L_cur; p_runaway = guard;
    pl p = {0, 0, 0, 0, 0}; float dt = DTS; sres r = {}; int hold = 0;
    for (int i = 0; i < (int)(30 / dt); i++) {
        t_now = i * dt; float o3 = 0;
        if (t_now >= 8 && t_now < 9.2) o3 = 0.0065f * sinf((float)(M_PI * (t_now - 8) / 1.2));
        else if (t_now >= 9.2 && t_now < 10.4) o3 = -0.005f * sinf((float)(M_PI * (t_now - 9.2) / 1.2));
        step(p, m, dt, o3, 0, 0, 0.0006f * gauss());
        if (t_now > 7) { r.max_o1 = fmax(r.max_o1, fabs(p.o1)); r.max_o2 = fmax(r.max_o2, fabs(p.o2)); if (fabs(g_u) > 0.95f) r.sat++; }
        if (t_now > 16 && fabs(p.o1) > 0.1) { int sg = p.o1 > 0 ? 1 : -1; if (hold && sg != hold) r.zc_late++; hold = sg; }
    }
    return r;
}
static int sbend_tests()
{
    printf("S-bend at speed (curvature +0.0065 -> -0.005 in 1.2 s), hands off: peak |o1| m / peak |o2| rad / frames at the ceiling / swings after 16 s\n");
    struct pv { const char *n; float c, gA, tw, dl, db; };
    const pv pls[] = { { "nominal", 0.19f, 1.0f, 0.3f, 0.0f, 0 }, { "ts4w-like gA 1.5", 0.17f, 1.5f, 0.05f, 0.33f, 0.0015f }, { "strong wheel gA 2.5", 0.19f, 2.5f, 0.05f, 0.3f, 0.007f }, { "crash fit gA 2.95", 0.17f, 2.95f, 0.05f, 0.33f, 0.010f } };
    const cfg c2 = { "mode 2", 2 };
    for (auto &q : pls) for (float kmh : { 70.f, 85.f }) {
        pp m; m.c = q.c; m.gA = q.gA; m.tw = q.tw; m.delay = q.dl; m.db = q.db;
        printf("%-22s %3.0f km/h |", q.n, kmh);
        for (float sc : { 0.87f, 1.0f, 1.27f }) { sres a = run_sbend(c2, m, kmh, sc, 0); printf(" s%.2f: %.2f/%.3f/%3d/%d |", sc, a.max_o1, a.max_o2, a.sat, a.zc_late); }
        sres g = run_sbend(c2, m, kmh, 1.27f, 1); printf(" s1.27+guard (game law): %.2f/%.3f/%3d/%d\n", g.max_o1, g.max_o2, g.sat, g.zc_late);
    }
    return 0;
}

// review 30-10: a one-sided runaway at 80 km/h. A constant cross-wind overcomes the assist, the request saturates to one side and the
// heading error keeps growing: the old guard only caught this via |o2| > 0.18 rad (late). The one-sided growth detection must disengage
// earlier. "before" is measured with the guard off (when |o2| would cross 0.18), "after" with the guard on (when g_panic_until arms).
static int runaway_tests()
{
    printf("\nRunaway guard: one-sided runaway at 80 km/h (constant cross-wind, hands off, straight road)\n");
    printf("%-8s %-8s %-8s %-8s\n", "wind", "before", "after", "gain");
    int bad = 0;
    for (float wind : { 0.22f, 0.25f, 0.28f }) {
        double t_before = -1, t_after = -1;
        {   // guard off: when the heading error alone would have crossed 0.18 rad (the old guard's only catch)
            const cfg c2 = { "runaway", 2 }; apply(c2); p_runaway = 0; vkmh = 80; rng = 5; via_stub = 0;
            g_panic_until = 0; g_sat_dir[0] = g_sat_dir[1] = g_sat_dir[2] = 0; g_sat_t[0] = g_sat_t[1] = g_sat_t[2] = 0; g_sat_since = 0;
            pl p = {0, 0, 0, 0, 0}; pp m; float dt = DTS;
            for (int i = 0; i < (int)(20 / dt) && t_before < 0; i++) {
                t_now = i * dt;
                step(p, m, dt, 0, 0, 0, wind);
                if (fabs(p.o2) > 0.18f) t_before = t_now;
            }
        }
        {   // guard on: the time the (new) guard disengages
            const cfg c2 = { "runaway", 2 }; apply(c2); p_runaway = 1; vkmh = 80; rng = 5; via_stub = 0;
            g_panic_until = 0; g_sat_dir[0] = g_sat_dir[1] = g_sat_dir[2] = 0; g_sat_t[0] = g_sat_t[1] = g_sat_t[2] = 0; g_sat_since = 0;
            pl p = {0, 0, 0, 0, 0}; pp m; float dt = DTS;
            for (int i = 0; i < (int)(20 / dt) && t_after < 0; i++) {
                t_now = i * dt;
                step(p, m, dt, 0, 0, 0, wind);
                if (g_panic_until > t_now) t_after = t_now;
            }
        }
        int ok = t_after > 0 && t_after <= t_before;   // review 30-10: must fire, and no later than the old |o2| > 0.18 catch
        printf("%-8.2f %-8.1f %-8.1f %-8.1f%s\n", wind, t_before, t_after, t_before > 0 ? t_before - t_after : 0, ok ? "" : "  <-- WRONG");
        bad |= !ok;
    }
    printf("Runaway one-sided test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

// 30-10: the same guard must NOT fire while a strong correction brings the truck back toward the center (a recovery after a lane
// change / an indicator). The request saturates and the heading error grows (turning toward the center), but the truck is not
// diverging; handing over to the game's weaker law there let it swing across the lane (rollover, 1651-1655). A fire is only allowed
// once the truck actually drifts AWAY from the center (o1 and its rate point the same way, o1 * g_do1 > 0).
static int recovery_tests()
{
    printf("\nRunaway guard: recovery toward the center (request saturates, heading grows) must not fire while approaching the center\n");
    int bad = 0;
    struct pv { const char *n; float gA, tw, dl, db; };
    const pv pls[] = { { "nominal", 1.0f, 0.3f, 0, 0 }, { "ts4w-like gA 1.3", 1.3f, 0.05f, 0.33f, 0.0015f }, { "strong gA 2.0", 2.0f, 0.05f, 0.3f, 0.004f } };
    for (auto &q : pls) for (float kmh : { 60.f, 80.f }) {
        const cfg c2 = { "recover", 2 }; apply(c2); p_runaway = 1; vkmh = kmh; rng = 5; via_stub = 0;
        g_panic_until = 0; g_sat_dir[0] = g_sat_dir[1] = g_sat_dir[2] = 0; g_sat_t[0] = g_sat_t[1] = g_sat_t[2] = 0; g_sat_since = 0;
        pp m; m.gA = q.gA; m.tw = q.tw; m.delay = q.dl; m.db = q.db;
        pl p = { 0.6f, 0.013f, 0, 0, 0 };   // off-center and already heading back toward the center (a lane change recovered)
        float o3 = -0.006f;                 // a gentle bend the recovery happens in (the feed-forward keeps the request at its ceiling)
        float dt = DTS; int fired = 0; float fire_o1 = 0, fire_do1 = 0; double sat = 0;
        for (int i = 0; i < (int)(12 / dt); i++) {
            t_now = i * dt;
            step(p, m, dt, o3, 0, 0, 0.0002f * gauss());
            if (fabsf(g_u) > 0.95f) sat++;
            if (!fired && g_panic_until > t_now) { fired = 1; fire_o1 = (float)p.o1; fire_do1 = g_do1; }
        }
        int ok = !fired || fire_o1 * fire_do1 > 0;   // may only fire once the truck drifts away from the center, not while recovering
        printf("  %-20s %3.0f km/h: %s (request at the ceiling %.0f ms)%s\n", q.n, kmh, fired ? (ok ? "fired while drifting away" : "FIRED WHILE RECOVERING") : "no fire",
               sat * dt * 1000, ok ? "" : "  <-- WRONG");
        bad |= !ok;
    }
    printf("Runaway recovery test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

// ---- learner / anti-sway tests (lka-learn notes)
struct lres { double rms_all, rms_tail, zc_tail, maxe, usable; float s, df, cap, gA, dl, c; int upd, sw, rolled; double win[12]; double first_sw; };
// hands off on a wandering road for T s; drvmode 3 = the driver weaves (hand on the wheel) from T*0.45 for 120 s
static lres runL(const pp &m, int learn, int sway, float kmh, float T, int drvmode = 0, float start_o1 = 0.5f, int bends = 0)
{
    const cfg c2 = { "x", 2 }; apply(c2); p_learn = learn; p_sway = sway; vkmh = kmh; rng = 4242;
    int ats = m.ag != 1.0f; if (ats) { use_layout(1); via_stub = 1; memset(GA, 0, sizeof GA); GA[0x1d1] = 0x20; L_cur.cap = L_base.cap = 2; g_law_at = -1e9; }   // (a 1.61 plant: through stub_law, with the estimator)
    if (learn) { g_cal = CAL_ARMED; g_cal_armed_at = 0; L_cal_ok = 0; L_cal_g_sum = L_cal_g_sum2 = 0; }   // learning runs only while a calibration is on
    pl p = {start_o1, 0, 0, 0, 0}; lres r = {}; r.first_sw = -1;
    float dt = DTS, os = 0; double sa = 0, st = 0, sT = 0; long na = 0, nt = 0, nu = 0, nn = 0; int hold = 0, zc = 0; int sw0 = 0;
    for (int i = 0; i < (int)(T / dt); i++) {
        t_now = i * dt;
        float o3 = ou_o3(os, dt, 0.0012f);
        if (bends) { float k = fmodf((float)t_now, 90.f); float e = clampf((k - 60) / 3, 0, 1) - clampf((k - 80) / 3, 0, 1); o3 += 0.0035f * e * e * (3 - 2 * e) * (((int)(t_now / 90)) % 2 ? -1 : 1); }
        int drv = 0; float wc = 0;
        if (drvmode == 3 && t_now > T * 0.45 && t_now < T * 0.45 + 120) { drv = 2; wc = 0.03f * sinf(6.2832f * (float)(t_now - T * 0.45) / 9.f) - 3.3f * o3; }
        step(p, m, dt, o3, drv, wc, 0.0006f * gauss());
        if (drvmode == 0) g_refmax_off = fmax(g_refmax_off, fabs(g_ref));
        if (drvmode == 0 && fabs(g_ref) > 0.05 && getenv("WHY_REF") && fmod(t_now, 2.0) < dt) printf("   aimL %.3f at %.1f s (%.0f km/h): o1 %.2f o2 %.4f act %.2f act_t %.1f hold_t %.1f pm %.4f dev %.4f wrate %.3f gA-plant %.2f\n", g_ref, t_now, kmh, p.o1, p.o2, g_act, g_act_t, g_hold_t, g_pm, g_dev, g_w_rate, m.gA);
        if (drvmode == 3 && getenv("TRACE_L") && fmod(t_now, 0.5) < dt && t_now > atof(getenv("TRACE_L")) && t_now < atof(getenv("TRACE_L")) + 30)
            printf("    %6.1f drv %d o1 %6.2f o2 %6.3f u %5.2f A %7.4f w %7.4f act %.2f wg %.2f wi %.2f s %.2f usable %d\n", t_now, drv, p.o1, p.o2, *(float *)(GA + GA_U), p.A, p.w, g_act, g_wg, g_wi, L_cur.s, L_usable);
        if (t_now > 8) { sa += p.o1 * p.o1; na++; r.maxe = fmax(r.maxe, fabs(p.o1)); }
        int wi = (int)(t_now / 10); if (wi < 12) r.win[wi] = fmax(r.win[wi], fabs(p.o1));
        if (t_now > T * 0.6) { st += p.o1 * p.o1; nt++; if (fabs(p.o1) > 0.1) { int sg = p.o1 > 0 ? 1 : -1; if (hold && sg != hold) zc++; hold = sg; } }
        nu += L_usable; nn++;
        if (SW_events > sw0 && r.first_sw < 0) r.first_sw = t_now;
    }
    r.rms_all = sqrt(sa / na); r.rms_tail = sqrt(st / nt); r.zc_tail = zc * 60.0 / (T * 0.4); r.usable = (double)nu / nn;
    r.s = L_cur.s; r.df = L_cur.dfac; r.cap = L_cur.cap; r.gA = L_gA; r.dl = L_dl; r.c = L_c; r.upd = L_nupd; r.sw = SW_events; r.rolled = L_rolled;
    if (ats) { use_layout(0); via_stub = 0; }
    return r;
}

static int learn_tests(int argc, char **argv)
{
    float kmh = argc > 2 ? atof(argv[2]) : 90; float T = argc > 3 ? atof(argv[3]) : 900;
    struct pv { const char *n; float c, gA, tw, dl, ag; };   // ag 0 = 1
    const pv pls[] = { { "reads like the real wheel", 0.18f, 1.0f, 0.05f, 0.3f }, { "ts4n large-signal fit (1.34, .2)", 0.155f, 1.34f, 0.05f, 0.2f }, { "real-like gA x0.6", 0.18f, 0.6f, 0.05f, 0.3f }, { "real-like gA x1.8", 0.18f, 1.8f, 0.05f, 0.3f }, { "gA x0.6", 0.155f, 0.80f, 0.05f, 0.2f }, { "gA x1.8", 0.155f, 2.41f, 0.05f, 0.2f },
                       { "gA x1.8 c x1.3", 0.20f, 2.41f, 0.05f, 0.2f }, { "slow wheel (delay .35, lag .15)", 0.155f, 1.34f, 0.15f, 0.35f }, { "fast wheel (delay .05)", 0.155f, 1.34f, 0.03f, 0.05f },
                       { "weak wheel gA x0.5 delay .3", 0.155f, 0.67f, 0.1f, 0.3f }, { "nominal sim plant", 0.19f, 1.0f, 0.3f, 0.0f },
                       { "ATS 1.61 (A x.15, gA 22, c .095)", 0.095f, 22.0f, 0.3f, 0.0f, 0.15f }, { "ATS, gA 15, c .12", 0.12f, 15.0f, 0.3f, 0.0f, 0.15f }, { "ATS, A x.25, gA 12", 0.095f, 12.0f, 0.3f, 0.0f, 0.25f } };
    printf("Learner on a wandering road, %.0f km/h, %.0f s, hands off; fixed gains vs learning (rms over the last 40%%: m, zc/min)\n", kmh, T);
    printf("%-34s | fixed: rms zc max | learn: rms zc max  s  dfac cap | est gA  delay  c  | usable updates sway rolled\n", "plant");
    for (auto &q : pls) {
        if (getenv("ONLY") && !strstr(q.n, getenv("ONLY"))) continue;   // (a what-if run of one plant)
        pp m; m.c = q.c; m.gA = q.gA; m.tw = q.tw; m.delay = q.dl; if (q.ag > 0) m.ag = q.ag;
        lres a = runL(m, 0, 0, kmh, T), b = runL(m, 1, 0, kmh, T);
        printf("%-34s | %5.2f %5.1f %5.2f | %5.2f %5.1f %5.2f %5.2f %4.2f %4.2f | %4.2f %5.2f %5.3f | %3.0f%% %3d %d %d\n", q.n, a.rms_tail, a.zc_tail, a.maxe, b.rms_tail, b.zc_tail, b.maxe, b.s, b.df, b.cap, b.gA, b.dl, b.c, b.usable * 100, b.upd, b.sw, b.rolled);
    }
    // anti-sabotage: the driver weaves for 120 s in the middle; the profile must not move
    pp m; m.c = 0.155f; m.gA = 1.34f; m.tw = 0.05f; m.delay = 0.2f;
    { lres a = runL(m, 1, 0, kmh, 360, 0); lres b = runL(m, 1, 0, kmh, 360, 3);
      printf("driver weaving 120 s (hand on the wheel): s %.3f -> %.3f, gA est %.2f -> %.2f, c %.3f -> %.3f (without weaving -> with)\n", a.s, b.s, a.gA, b.gA, a.c, b.c); }
    // persistence round trip
    if (getenv("SIM_DIR")) {
        pp m2; m2.c = 0.155f; m2.gA = 2.41f; m2.tw = 0.05f; m2.delay = 0.2f;
        remove((std::string(getenv("SIM_DIR")) + "/steer_profile.txt").c_str());
        lres a = runL(m2, 1, 0, kmh, 600); prof_load(sim_wheel); L_dirty = 1; prof_save("sim");
        lres b = runL(m2, 1, 0, kmh, 5); prof_load(sim_wheel);
        printf("profile file round trip: learned s %.3f -> after load s %.3f (wheel key '%s')\n", a.s, L_cur.s, L_key);
        FILE *f = fopen((std::string(getenv("SIM_DIR")) + "/steer_profile.txt").c_str(), "r"); char line[300]; while (f && fgets(line, sizeof line, f)) printf("  | %s", line); if (f) fclose(f);
        FILE *g = fopen((std::string(getenv("SIM_DIR")) + "/steer_profile.txt").c_str(), "w"); if (g) { fputs("wheel=SimWheel v=1 s=9.0 dfac=1 cap=2\nwheel=other s=1\n", g); fclose(g); }
        L_cur = { 1, 1, 2 }; prof_load(sim_wheel); printf("broken profile (s 9.0): s after load %.2f (must be 1.00)\n", L_cur.s);
    }
    return 0;
}

static int sway_tests(int argc, char **argv)
{
    float kmh = argc > 2 ? atof(argv[2]) : 90;
    printf("Anti-sway at %.0f km/h, learn off, hands off, start 0.5 m, 120 s: max |o1| per 10 s window; rms last 40%% / zc/min / first detection / events\n", kmh);
    struct pv { const char *n; float c, gA, tw, dl; };
    const pv pls[] = { { "ts4n fit", 0.155f, 1.34f, 0.05f, 0.2f }, { "gA x1.8", 0.155f, 2.41f, 0.05f, 0.2f }, { "gA x2.5 (rings in ts4n)", 0.19f, 2.5f, 0.08f, 0.2f }, { "gA x3.5", 0.19f, 3.5f, 0.08f, 0.2f },
                       { "delay .35", 0.155f, 1.34f, 0.05f, 0.35f }, { "delay .5", 0.155f, 1.34f, 0.05f, 0.5f } };
    for (auto &q : pls) {
        pp m; m.c = q.c; m.gA = q.gA; m.tw = q.tw; m.delay = q.dl;
        for (int sw : { 0, 1 }) {
            lres r = runL(m, 0, sw, kmh, 120);
            printf("%-24s sway %d |", q.n, sw);
            for (int i = 0; i < 12; i++) printf(" %4.2f", r.win[i]);
            printf(" | %.2f %4.1f first %5.1f ev %d\n", r.rms_tail, r.zc_tail, r.first_sw, r.sw);
        }
    }
    printf("Normal bend entries/exits (nominal and ts4n plants, bends of o3 .0035 both ways every 90 s, 450 s): events must be 0\n");
    for (float kk : { 60.f, 90.f }) for (int pi = 0; pi < 2; pi++) {
        pp m; if (pi == 0) { m.c = 0.155f; m.gA = 1.34f; m.tw = 0.05f; m.delay = 0.2f; }
        lres r = runL(m, 0, 1, kk, 450, 0, 0.0f, 1);
        printf("  %2.0f km/h plant %s: events %d, max|o1| %.2f, rms %.2f\n", kk, pi ? "nominal" : "ts4n", r.sw, r.maxe, r.rms_all);
    }
    return 0;
}

// 30-10: a detected sway must lower the gain for the rest of the session, but only the stored profile while a calibration is
// on (the driver asked it to measure this wheel). Outside a calibration the reduction is a session factor (not stored, slow
// recovery), so it sticks within the session but cannot walk steer_profile.txt to the floor across sessions.
static int sway_persist_tests()
{
    printf("\nSway persistence: a ringing wheel (gA 2.5) sways; outside a calibration only the session factor falls (stored s stays), with a calibration the profile stores it\n");
    int bad = 0;
    pp m; m.c = 0.19f; m.gA = 2.5f; m.tw = 0.08f; m.delay = 0.2f;
    {   // learn on, no calibration: the estimator is gated off, so only the sway reduction can move anything, and it must touch the session factor, not the profile
        const cfg c2 = { "swp", 2 }; apply(c2); p_learn = 1; p_sway = 1; vkmh = 90; rng = 4242;
        g_cal = CAL_IDLE;
        pl p = { 0.5, 0, 0, 0, 0 }; float dt = DTS; float os = 0;
        for (int i = 0; i < (int)(120 / dt); i++) { t_now = i * dt; step(p, m, dt, ou_o3(os, dt, 0.0012f), 0, 0, 0.0006f * gauss()); }
        int ok = SW_events >= 1 && L_sess < 0.95f && L_cur.s == 1.0f && !L_dirty;
        printf("  gA 2.5 no calibration: sway events %d, session factor %.2f (was 1.00), stored s %.2f (must stay 1.00), dirty %d -> %s\n", SW_events, L_sess, L_cur.s, L_dirty, ok ? "ok" : "<-- WRONG");
        bad |= !ok;
    }
    {   // the same wheel but a calibration armed: the learner may store the reduction (updates are held off so only the sway moves s)
        const cfg c2 = { "swp", 2 }; apply(c2); p_learn = 1; p_sway = 1; p_l_update = 1e9f; vkmh = 90; rng = 4242;
        g_cal = CAL_ARMED; g_cal_armed_at = 0; L_cal_ok = 0; L_cal_g_sum = L_cal_g_sum2 = 0;
        pl p = { 0.5, 0, 0, 0, 0 }; float dt = DTS; float os = 0;
        for (int i = 0; i < (int)(120 / dt); i++) { t_now = i * dt; step(p, m, dt, ou_o3(os, dt, 0.0012f), 0, 0, 0.0006f * gauss()); }
        int ok = SW_events >= 1 && L_cur.s < 0.95f && L_dirty;
        printf("  gA 2.5 calibration: sway events %d, stored s %.2f (was 1.00), dirty %d -> %s\n", SW_events, L_cur.s, L_dirty, ok ? "ok" : "<-- WRONG");
        bad |= !ok;
    }
    printf("Sway persistence test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

// 30-10: learning only runs while a calibration is on, the calibration also identifies the model passively in gentle bends (not only on
// a straight), and once enough fits agree it stops itself, tells the driver (the done cue) and stores.
static int count_cue(int c);
static int cal_learning_tests()
{
    printf("\nCalibration learning: no calibration -> no learning; armed on gentle bends -> fits, confidence stop, done cue\n");
    int bad = 0;
    {   // without a calibration the estimator is off (rule a)
        pp m;
        const cfg c2 = { "caln", 2 }; apply(c2); p_learn = 1; vkmh = 80; rng = 7; g_cal = CAL_IDLE;
        pl p = { 0.1f, 0, 0, 0, 0 }; float dt = DTS; float os = 0;
        for (int i = 0; i < (int)(60 / dt); i++) { t_now = i * dt; step(p, m, dt, ou_o3(os, dt, 0.0012f), 0, 0, 0.0004f * gauss()); }
        int ok = L_nupd == 0;
        printf("  no calibration: updates %d (must be 0) -> %s\n", L_nupd, ok ? "ok" : "<-- WRONG");
        bad |= !ok;
    }
    struct pv { const char *n; float gA, dl, tw, c; };
    const pv pls[] = { { "nominal", 1.0f, 0, 0.3f, 0.19f }, { "ts4n-like", 1.34f, 0.2f, 0.05f, 0.155f }, { "strong", 2.0f, 0.3f, 0.05f, 0.19f } };
    for (auto &q : pls) {
        pp m; m.gA = q.gA; m.delay = q.dl; m.tw = q.tw; m.c = q.c;
        const cfg c2 = { "calp", 2 }; apply(c2); p_learn = 1; vkmh = 80; rng = 42; via_stub = 0;
        g_cal = CAL_ARMED; g_cal_armed_at = 0; L_cal_ok = 0; L_cal_g_sum = L_cal_g_sum2 = 0; g_cues.clear();
        pl p = { 0.1f, 0, 0, 0, 0 }; float dt = DTS; int stopped = 0;
        for (int i = 0; i < (int)(240 / dt); i++) {
            t_now = i * dt;
            float o3 = 0.0035f * sinf(2 * 3.14159f * (float)t_now / 60.f);   // a gentle bend both ways every 60 s (lat ~1.7 m/s2)
            step(p, m, dt, o3, 0, 0, 0.0004f * gauss());
            if (g_cal == CAL_IDLE && t_now > 10) { stopped = 1; break; }
        }
        int ok = L_nupd >= 2 && stopped && count_cue(CUE_LANE_CAL_DONE) == 1;
        printf("  %-10s: updates %d, stopped %s, done cue %d, s %.2f (gA %.2f delay %.2f c %.3f)%s\n", q.n, L_nupd, stopped ? "yes" : "no",
               count_cue(CUE_LANE_CAL_DONE), L_cur.s, L_gA, L_dl, L_c, ok ? "" : "  <-- WRONG");
        bad |= !ok;
    }
    printf("Calibration learning test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

// review: bend_max_lat 0 is the documented "off" of the bend-feed-forward clamp. The learner's lateral-acceleration gate
// (v^2 |o3| < bend_max_lat) must then also be off, or learning and calibration die with it (v^2 |o3| < 0 is never true).
static int bend_off_test()
{
    printf("\nbend_max_lat 0 (off): the learner must still run in a gentle bend, not be silently killed by the off value\n");
    int bad = 0;
    pp m; m.gA = 1.0f; m.tw = 0.3f; m.c = 0.19f;
    const cfg c2 = { "bend0", 2 }; apply(c2); p_learn = 1; p_bend_lat = 0; vkmh = 80; rng = 7;
    g_cal = CAL_ARMED; g_cal_armed_at = 0; L_cal_ok = 0; L_cal_g_sum = L_cal_g_sum2 = 0;
    pl p = { 0.1f, 0, 0, 0, 0 }; float dt = DTS;
    for (int i = 0; i < (int)(120 / dt); i++) {
        t_now = i * dt;
        float o3 = 0.0035f * sinf(2 * 3.14159f * (float)t_now / 60.f);   // a gentle bend both ways (lat ~1.7 m/s2)
        step(p, m, dt, o3, 0, 0, 0.0004f * gauss());
    }
    int ok = L_nupd >= 1;
    printf("  bend_max_lat 0: updates %d (must be > 0) -> %s\n", L_nupd, ok ? "ok" : "<-- WRONG");
    bad |= !ok;
    printf("Bend-off learning test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

// review: the confidence stop must wait for the profile itself to converge (the last update changed s and dfac by < 3 %),
// not just for 3 agreeing loop-gain fits. A strong wheel (gA 2.0, c 0.19 -> target s 0.70) must not stop at s ~0.76.
static int cal_convergence_test()
{
    printf("\nCalibration convergence: a strong wheel (target s 0.70) must converge before the done cue, not stop early at s ~0.76\n");
    int bad = 0;
    pp m; m.gA = 2.0f; m.delay = 0.3f; m.tw = 0.05f; m.c = 0.19f;
    const cfg c2 = { "conv", 2 }; apply(c2); p_learn = 1; vkmh = 80; rng = 42; via_stub = 0;
    g_cal = CAL_ARMED; g_cal_armed_at = 0; L_cal_ok = 0; L_cal_g_sum = L_cal_g_sum2 = 0; g_cues.clear();
    pl p = { 0.1f, 0, 0, 0, 0 }; float dt = DTS; int stopped = 0;
    for (int i = 0; i < (int)(360 / dt); i++) {
        t_now = i * dt;
        float o3 = 0.0035f * sinf(2 * 3.14159f * (float)t_now / 60.f);
        step(p, m, dt, o3, 0, 0, 0.0004f * gauss());
        if (g_cal == CAL_IDLE && t_now > 10) { stopped = 1; break; }
    }
    int ok = stopped && L_cur.s <= 0.74f && L_cur.s >= 0.68f && count_cue(CUE_LANE_CAL_DONE) == 1;
    printf("  strong wheel: stopped %s, s %.3f (target 0.70, must be <= 0.74, was 0.76 before the fix), done cue %d, updates %d -> %s\n", stopped ? "yes" : "no",
           L_cur.s, count_cue(CUE_LANE_CAL_DONE), L_nupd, ok ? "ok" : "<-- WRONG");
    bad |= !ok;
    printf("Calibration convergence test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}


// ---- 02-11: ATS 1.61 (what the first rides showed): the steering field, the weaker A, the profile per game and class, a scale above learn_max
static int test_ats()
{
    int bad = 0;
    printf("\nATS 1.61 and the 1.61 layout:\n");
    // 1. the layouts: the effective steering moved +0x78 on 1.61 (A + steering is the FFB wheel target, in both 1.61 exes); read at 0x640 it was a constant
    {
        use_layout(1);
        memset(VEH, 0, sizeof VEH);
        *(float *)(VEH + 0x640) = 41.42f; *(float *)(VEH + L->i_steer) = 0.05f; *(float *)(VEH + L->i_steer_in) = 0.05f; *(float *)(VEH + L->i_assist) = 0.002f;
        float A, st, w; sense_steering(&A, &st, &w);
        use_layout(0);
        int ok = st == 0.05f;   // (the sim's 1.61 copy keeps 0x640 as i_steer: what matters is that read_veh follows the layout field, not a constant)
        g_lay161.i_steer = 0x6b8; use_layout(1);
        memset(VEH, 0, sizeof VEH); *(float *)(VEH + 0x640) = 41.42f; *(float *)(VEH + 0x6b8) = 0.07f; *(float *)(VEH + L->i_steer_in) = 0.07f;
        sense_steering(&A, &st, &w); use_layout(0);
        ok &= st == 0.07f;
        printf("  steering read through the layout (0x640 on 1.60, 0x6b8 on 1.61): %s\n", ok ? "ok" : "<-- WRONG");
        bad |= !ok;
    }
    // 2. FFB detection on ATS-sized numbers: the assist offset there is ~0.0005-0.005 (ETS2 0.01+), the threshold follows the measured actuator gain
    {
        int okk = 1;
        for (int kind = 0; kind < 2; kind++) {
            g_ffb = -1; F_n = F_sw = F_swa = F_a = 0; g_ag = 0.15f;
            for (int i = 0; i < 200; i++) {
                float A = 0.0018f * sinf(i * 0.05f), w = 0.05f * sinf(i * 0.013f);
                ffb_detect(A, kind ? w + A : w, w);
            }
            printf("  FFB detection with A of 0.0018 (actuator gain 0.15), %s: %d\n", kind ? "assist added (keyboard)" : "FFB wheel", g_ffb);
            okk &= g_ffb == (kind ? 0 : 1);
        }
        g_ag = 1; g_ffb = 1;
        bad |= !okk;
    }
    // 3. the profile key: wheel x game x class; ETS2 trucks keep the plain key the old files hold
    {
        char k[64]; int okk = 1;
        const char *exp[] = { "Wine_joystick_driver", "Wine_joystick_driver.car", "Wine_joystick_driver.ats", "Wine_joystick_driver.ats.car" };
        for (int a = 0; a < 2; a++) for (int c = 0; c < 2; c++) {
            static layout lay; lay = sim_layout; lay.game = a ? "ats" : "eut2"; L = &lay; g_sim_class = c ? 2 : 1;
            prof_key(k, sizeof k, "Wine joystick driver");
            const char *e = exp[a * 2 + c];
            // exp is ordered truck/car for eut2, truck/car for ats
            okk &= !strcmp(k, a ? (c ? exp[3] : exp[2]) : (c ? exp[1] : exp[0]));
            printf("  profile key, %s %s: %s\n", a ? "ATS " : "ETS2", c ? "car  " : "truck", k); (void)e;
        }
        L = &sim_layout; g_sim_class = 1;
        bad |= !okk;
    }
    // 4. a scale above learn_max: only on 1.61, back at learn_max after a ring/sway/runaway, held down while the live wheel gain is far above the measured
    {
        int okk = 1;
        use_layout(0); apply({ "mode 2", 2 }); p_learn = 1; L_cur = { 1.3f, 1, 1.4f }; g_wi = 3.0f; L_gA = 3.0f;
        okk &= scale_s() == 1.3f;
        s_back_to_max("test"); okk &= L_cur.s == 1.3f;                 // 1.60: unchanged
        use_layout(1); L_sess = 1; L_cur = { 1.3f, 1, 1.4f };
        g_wi = 3.0f; L_gA = 3.0f; float a = scale_s();                   // the live gain agrees: stands
        g_wi = 6.0f; float b = scale_s();                                // twice the measured gain (large signals): 1.3 x 1.25 x 3/6 = 0.81, held at learn_max
        g_wi = 3.0f; okk &= a == 1.3f && b == 1.0f;
        s_back_to_max("test"); okk &= L_cur.s == 1.0f && L_cur.cap == 1.0f && L_dirty;
        printf("  scale above learn_max: 1.60 unchanged, 1.61 stands (%.2f) / held down by a strong live wheel (%.2f) / back at learn_max after a ring, sway or runaway (%.2f): %s\n", a, b, L_cur.s, okk ? "ok" : "<-- WRONG");
        use_layout(0); L_dirty = 0; L_cur = L_base = { 1, 1, p_l_max }; L_gA = REF_GA; g_wi = 0;
        bad |= !okk;
    }
    printf("ATS test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

static float g_fake_u;
static void fake_law(void *ga, float, float, float, float) { *(float *)((char *)ga + GA_U) = g_fake_u; }
static int test_indicator()
{
    apply({ "mode 2", 2 }); p_ind_resume = 1;  // the flag write is tested here (default off)
    o_law = fake_law; T.speed = 80 / 3.6f;
    memset(GA, 0, sizeof GA); GA[0x1d1] = 0x20;
    double t0 = t_now, paused_at = -1, resumed_at = -1, full_at = -1; float max_in_pause = 0; int bit_back = 0, bad = 0, lka_during = 0;
    for (int i = 0; i < 12 * 60; i++) {
        t_now += 1.0 / 60; double t = t_now - t0;
        GA[0x1d0] = (t > 2 && t < 6) ? 4 : 0;          // left indicator 2-6 s
        if (t > 4 && t < 4.02) GA[0x1d1] &= ~0x20;      // the game switched lane keeping off
        float o1 = t < 2 ? 0.2f : t < 5 ? 0.2f + (float)(t - 2) * 0.5f : t < 5.5 ? 0 : -0.3f;  // drifting over, lost, new lane
        if (t >= 5 && t < 5.5) o1 = 0;
        float o2 = 0, o3 = 0;
        g_fake_u = -0.4f * o1;
        indicator_pause();
        stub_law(GA, o1, o2, o1 == 0 ? 0 : o3 + 1e-4f, 1.0f / 60);
        float out = *(float *)(GA + GA_U);
        if (g_pause && paused_at < 0) paused_at = t;
        if (g_pause && t - paused_at > 0.3) max_in_pause = fmaxf(max_in_pause, fabsf(out));
        if (!g_pause && paused_at > 0 && resumed_at < 0) resumed_at = t;
        if (resumed_at > 0 && full_at < 0 && t - resumed_at > 1.05) full_at = t;
        if ((GA[0x1d1] & 0x20) && t > 4.1 && !bit_back) bit_back = 1;
        if (t > 2.1 && t < 6 && (GA[0x1d1] & 0x20)) lka_during = 1;  // indicator_off: the game's lane keeping must be off
    }
    printf("\nIndicator pause: paused at %.2f s (indicator 2.00), |u| after 0.3 s of pause %.3f, resumed at %.2f s (indicator off 6.00), lane keeping back on: %s\n",
           paused_at, max_in_pause, resumed_at, bit_back ? "yes" : "NO");
    printf("Lane keeping off during the indicator: %s\n", lka_during ? "NO" : "yes");
    bad = paused_at < 2 || paused_at > 2.05 || max_in_pause > 0.01f || resumed_at < 7 || resumed_at > 10.1 || !bit_back || lka_during;
    // hazards (both bits) must not pause
    apply({ "mode 2", 2 }); g_pause = 0; GA[0x1d0] = 12; indicator_pause(); bad |= g_pause; GA[0x1d0] = 0;
    // a forgotten indicator: paused, back after 20 s although it stays on; a blinking lamp (off 0.4 s) keeps the same period
    for (int i = 0; i < 90; i++) { t_now += 0.1; GA[0x1d0] = 0; indicator_pause(); }  // a clean start
    double t1 = t_now, back = -1;
    for (int i = 0; i < 300; i++) {
        t_now += 0.1; GA[0x1d0] = (fmod(t_now - t1, 1.0) < 0.6) ? 8 : 0; indicator_pause();
        if (!g_pause && i > 5 && back < 0) back = t_now - t1;
    }
    printf("Forgotten indicator (blinking): assist back after %.1f s (must be ~20)\n", back);
    bad |= back < 19 || back > 22;
    GA[0x1d0] = 0;
    printf("Indicator pause test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

// The lane-keeping key pressed during the indicator (we hold it off): means OFF, also after the indicator (Damien 11-10)
static int test_indicator_key()
{
    apply({ "mode 2", 2 }); o_law = fake_law; T.speed = 80 / 3.6f;
    memset(GA, 0, sizeof GA); GA[0x1d1] = 0x20;
    double t0 = t_now; int on_after = 0, on_during = 0;
    for (int i = 0; i < 10 * 60; i++) {
        t_now += 1.0 / 60; double t = t_now - t0;
        GA[0x1d0] = (t > 1 && t < 4) ? 4 : 0;
        if (t > 2 && t < 2.02) GA[0x1d1] |= 0x20;   // the driver presses the key: the game toggles it on
        indicator_pause();
        if (t > 2.1 && t < 4 && (GA[0x1d1] & 0x20)) on_during = 1;
        if (t > 6 && (GA[0x1d1] & 0x20)) on_after = 1;
    }
    int bad = on_during || on_after;
    printf("Indicator + key: lane keeping during %s, after %s -> %s\n", on_during ? "ON" : "off", on_after ? "ON" : "off", bad ? "FAILED" : "ok");
    return bad;
}

// The lane state (core/suite.h LaneState, for the dev display): one value for what the assist does, set where the gates decide.
static int test_lane_state()
{
    apply({ "mode 2", 2 }); o_law = fake_law; p_needs_cc = 1; p_keep_on = 1; p_min_kmh = 5;
    memset(GA, 0, sizeof GA); GA[0x1d1] = 0x20; memset(VEH, 0, sizeof VEH); g_pause = 0; g_panic_until = 0; g_lane_state = LANE_OFF;
    int bad = 0;
    auto law = [&](float o1, float o2, float o3, int n) {   // n law calls at 60 Hz with a request that moves a little (so the law counts as run)
        for (int i = 0; i < n; i++) { t_now += 1.0 / 60; g_fake_u = 0.1f + 0.01f * (i & 1); stub_law(GA, o1, o2, o3, 1.0f / 60); }
    };
    auto expect = [&](const char *what, int want) {
        printf("lane state: %-46s %d (expect %d)%s\n", what, g_lane_state, want, g_lane_state == want ? "" : "  <-- WRONG");
        bad |= g_lane_state != want;
    };
    T.speed = 80 / 3.6f; T.cruise = 0;
    law(0.2f, 0.01f, 0.001f, 30); expect("cruise off, needs_cruise = 1 (key decision pending)", LANE_WAIT_NO_CRUISE);
    T.cruise = 80 / 3.6f;
    law(0.2f, 0.01f, 0.001f, 30); expect("cruise on, a lane seen", LANE_ACTIVE);
    law(0, 0, 0, 30); expect("lane lost (o1 = o2 = o3 = 0)", LANE_WAIT_NO_LANE);
    g_pause = 1;
    law(0.2f, 0.01f, 0.001f, 30); expect("indicator pause", LANE_WAIT_INDICATOR);
    g_pause = 0; T.speed = 3 / 3.6f;
    law(0.2f, 0.01f, 0.001f, 30); expect("3 km/h with keep_on (min_kmh 5)", LANE_WAIT_SLOW);
    T.speed = 80 / 3.6f;
    law(0.2f, 0.01f, 0.001f, 30); expect("back at 80 km/h", LANE_ACTIVE);
    g_panic_until = t_now + 4;
    law(0.2f, 0.01f, 0.001f, 30); expect("runaway guard: the game's law for 4 s", LANE_RUNAWAY);
    g_panic_until = 0;
    law(0.2f, 0.01f, 0.001f, 30); expect("guard over", LANE_ACTIVE);
    t_now += 0.3; lane_state_tick(); expect("no law call for 0.3 s (lane keeping off)", LANE_OFF);
    law(0.2f, 0.01f, 0.001f, 30); expect("the law runs again", LANE_ACTIVE);
    t_now += 1.0 / 60; g_fake_u = 0; stub_law(GA, 0.2f, 0.01f, 0.001f, 1.0f / 60); expect("the game writes request 0 (assist off)", LANE_OFF);
    printf("Lane state test: %s\n", bad ? "FAILED" : "ok");
    g_pause = 0; p_needs_cc = 0;
    return bad;
}

// cruise_switches_on: the cruise coming on (Set, later Resume) switches lane keeping on only with the key at 1; at 0 its state stays
static int test_cruise_lane_on()
{
    int bad = 0;
    for (int key : { 0, 1 }) {
        apply({ "mode 2", 2 }); p_needs_cc = 1; p_cc_lka = key; p_min_kmh = 20; g_pause = 0; T.speed = 80 / 3.6f;
        memset(GA, 0, sizeof GA);
        auto run = [&](float cc, double s) { T.cruise = cc; for (int i = 0; i < s * 60; i++) { t_now += 1.0 / 60; cruise_lane_on(); } };
        run(0, 1); run(80 / 3.6f, 1);
        int on_set = (GA[0x1d1] & 0x20) != 0;
        run(0, 1); GA[0x1d1] &= ~0x20; run(80 / 3.6f, 1);   // off again by its key while the cruise was off, then Resume
        int on_resume = (GA[0x1d1] & 0x20) != 0;
        GA[0x1d1] |= 0x20; run(0, 1); run(80 / 3.6f, 1);   // on by its key: the cruise never switches it off
        int on_kept = (GA[0x1d1] & 0x20) != 0;
        int b = on_set != key || on_resume != key || !on_kept;
        printf("Cruise switches lane keeping on, cruise_switches_on = %d: after Set %s, after Resume %s, on by its key and Resume %s -> %s\n", key,
               on_set ? "on" : "off", on_resume ? "on" : "off", on_kept ? "on" : "off", b ? "FAILED" : "ok");
        bad |= b;
    }
    T.cruise = 0; p_needs_cc = 0; p_cc_lka = 0; memset(GA, 0, sizeof GA);
    return bad;
}

// ts5b 387.4 s: the game's law saturated at -1, the same as our clamped output, so the hook took it for "not run", filter_a then
// thought lane keeping was off and released the wheel in the bend. Saturated with the lane-keeping bit on: A must keep building;
// with the bit off our law must not keep a stale -1 going (what the game does with its own stale u is the game's).
static int test_saturation()
{
    int bad = 0;
    for (int bit : { 1, 0 }) {
        apply({ "mode 2", 2 }); o_law = fake_law; T.speed = 63 / 3.6f; g_fake_u = -1.0f;
        memset(GA, 0, sizeof GA); GA[0x1d1] = bit ? 0x20 : 0; *(float *)(GA + GA_U) = -1.0f;
        float A = -0.02f, wheel = -0.06f; double t0 = t_now;
        for (int i = 0; i < 90; i++) {   // 1.5 s at 60 Hz, a sharp bend the truck runs wide in (o1 outside, heading out)
            t_now = t0 + i / 60.0;
            g_fake_u = i < 10 ? -0.8f : -1.0f;   // it runs unsaturated first (as in the ride), then saturates
            *(float *)(VEH + V_A) = A; *(float *)(VEH + V_STEER_IN) = wheel; *(float *)(VEH + V_STEER) = wheel;
            stub_law(GA, 1.2f, -0.12f, 0.015f, 1.0f / 60);
            float u = *(float *)(GA + GA_U);
            A += (1.0f / 60) * (-3.6f * A + 0.17f * u);
            A = filter_a(A);
            wheel += (1.0f / 60) * (2.9f * A - wheel) / 0.1f;
        }
        double idle = t_now - g_law_at;
        printf("Saturated law (u = -1 = our output), lane-keeping bit %s: A after 1.5 s %.4f, our law last ran %.2f s ago (%s)\n", bit ? "on" : "off", A, idle,
               bit ? "A must keep building: < -0.03" : "our law must not keep a stale -1 going: > 1 s");
        bad |= bit ? !(A < -0.03f) : !(idle > 1);
    }
    printf("Saturation test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

// Steering profile detection: an FFB wheel (steer == wheel), keyboard/gamepad (steer == wheel + A), noise only.
static int test_ffb()
{
    int bad = 0;
    for (int kind = 0; kind < 2; kind++) {
        g_ffb = -1; F_n = F_sw = F_swa = F_a = 0;
        for (int i = 0; i < 200; i++) {
            float A = 0.012f * sinf(i * 0.05f), w = 0.05f * sinf(i * 0.013f);
            ffb_detect(A, kind ? w + A : w, w);
        }
        printf("FFB detection, %s: %d\n", kind ? "assist added (keyboard)" : "FFB wheel", g_ffb);
        bad |= g_ffb != (kind ? 0 : 1);
    }
    g_ffb = 1;
    printf("FFB detection test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

// ---------------------------------------------------------------------------------------------------------------------
// Steering calibration (the lane-keeping key held) on the plant: a known wheel gain and delay must be recovered, the truck must stay in
// the inner part of the lane, every abort must leave the profile alone, the timeout must disarm.
static int count_cue(int c) { int n = 0; for (int x : g_cues) n += x == c; return n; }
struct prof_snap { float s, d, cap, gA, dl, c; int dirty; };
static prof_snap snap() { return { L_cur.s, L_cur.dfac, L_cur.cap, L_gA, L_dl, L_c, L_dirty }; }
static bool same(const prof_snap &a, const prof_snap &b) { return a.s == b.s && a.d == b.d && a.cap == b.cap && a.gA == b.gA && a.dl == b.dl && a.c == b.c && a.dirty == b.dirty; }

enum { AB_NONE, AB_INDICATOR, AB_DRIVER, AB_BEND, AB_LOST, AB_LEAD, AB_BRAKE, AB_WIND, AB_CRUISE, AB_SPEED, AB_ROUTE, AB_KEYOFF, AB_RESET, AB_N };
static const char *ab_name[AB_N] = { "none (full run)", "indicator", "driver steers", "a bend (lane term)", "lane lost", "lead closing in", "braking", "side wind (truck drifts out)",
                                     "cruise off (needs_cruise)", "speed leaves the band", "bend ahead on the route", "lane keeping switched off", "game reset (pause)" };
struct calres { int started, finished, aborted; double t_start, t_end, peak_o1, peak_o2, max_after, steer_max; float gA, dl, c; prof_snap before, after; int rerun; };

static void cal_setup(float kmh, unsigned seed)
{
    apply({ "mode 2", 2 });
    p_learn = 1; p_cal_timeout = 300; L_loaded = 1; snprintf(L_key, sizeof L_key, "SimWheel"); g_ffb = 1; via_stub = 1; quiet = 0; rng = seed;
    memset(GA, 0, sizeof GA); GA[0x1d1] = 0x20; memset(VEH, 0, sizeof VEH);
    g_cues.clear(); T.cruise = 0; T.in_brake = T.eff_brake = 0; g_inc.gap_m = NAN; sim_route_ok = 0; g_sim_lost = 0; g_cal_reset = 0;
    g_pause = 0; g_ind = 0; g_cal = CAL_IDLE; g_law_at = g_stub_at = -1e9; g_soft_at = g_soft_end = g_centre_since = -1; g_panic_until = 0; g_resume_at = -1e9;
    g_lane_ok_at = g_lane_bad_at = -1e9; L_cur = L_base = { 1, 1, p_l_max }; L_gA = REF_GA; L_dl = REF_DELAY; L_c = REF_C; L_dirty = 0; L_hold = 0; L_trial = 0;
    vkmh = kmh;
}

static int g_cal_verbose = getenv("CALV") != NULL;   // CALV=1: the log lines of the calibration runs
static float g_cal_cap;   // > 0: the profile ceiling a calibration run starts with (a fresh profile holds 2; apply() would use learn_max)
static calres cal_run(const pp &m, float kmh, int ab, double ab_after = 2.0, double tmax = 150, bool arm = true)
{
    cal_setup(kmh, 777); if (g_cal_cap > 0) L_cur.cap = L_base.cap = g_cal_cap; int vsave = verbose; if (g_cal_verbose) verbose = 1;
    pl p = { 0.03, 0, 0, 0, 0 };
    calres r = {};
    r.before = snap();
    float dt = DTS, os = 0;
    if (arm) cal_arm(0);
    double after_end = -1;
    for (int i = 0; i < (int)(tmax / dt); i++) {
        t_now = i * dt;
        float o3 = 0.0002f * gauss(); (void)os;
        float wind = 0.0002f * gauss(); int drv = 0; float wcmd = 0; g_sim_lost = 0;
        if (g_cal == CAL_RUN && !r.started) { r.started = 1; r.t_start = t_now; }
        if (!arm && !r.started && t_now >= 13.0) { r.started = 1; r.t_start = 13.0; }   // the same disturbance without any calibration (the baseline)
        double ta = r.started ? t_now - r.t_start : -1;
        // the bend, lost lane, wind and a steering driver are 2 s long (a real one may last, the centering law then deals with it); the rest stay
        bool brief = ab == AB_BEND || ab == AB_LOST || ab == AB_WIND || ab == AB_DRIVER;
        if (r.started && ab != AB_NONE && ta >= ab_after && (!brief || ta < ab_after + 2.0)) {
            switch (ab) {
            case AB_INDICATOR: GA[0x1d0] = 4; break;
            case AB_DRIVER: drv = 2; wcmd = 0.03f * (float)sin(2 * 3.14159 * 1.5 * ta); break;
            case AB_BEND: o3 = 0.0045f; break;
            case AB_LOST: g_sim_lost = 1; break;
            case AB_LEAD: g_inc.gap_m = 2.0f * kmh / 3.6f; break;
            case AB_BRAKE: T.in_brake = 0.3f; break;
            case AB_WIND: wind += 0.02f; break;
            case AB_CRUISE: p_needs_cc = 1; T.cruise = 0; break;
            case AB_SPEED: vkmh = 120; break;
            case AB_ROUTE: sim_route_ok = 1; sim_route_straight = 40; break;
            case AB_KEYOFF: GA[0x1d1] = 0; break;
            case AB_RESET: if (g_cal == CAL_RUN) ln_reset(0, "pause"); break;
            }
        }
        if (ab == AB_KEYOFF && ta >= ab_after) ln_frame();   // the law is not called any more: only the frame can notice
        else step(p, m, dt, o3, drv, wcmd, wind);
        if (r.started && g_cal == CAL_RUN) { r.peak_o1 = fmax(r.peak_o1, fabs(p.o1)); r.peak_o2 = fmax(r.peak_o2, fabs(p.o2)); }
        if (r.started && r.t_end == 0 && (arm ? g_cal == CAL_IDLE : t_now - r.t_start >= ab_after)) { r.t_end = t_now; after_end = t_now; }
        if (after_end > 0) r.max_after = fmax(r.max_after, fabs(p.o1));
        if (g_cal == CAL_RUN && r.t_end > 0) r.rerun = 1;
        if (r.t_end > 0 && t_now > r.t_end + 40) break;
    }
    r.finished = count_cue(CUE_LANE_CAL_DONE); r.aborted = count_cue(CUE_LANE_CAL_ABORT);
    r.gA = L_gA; r.dl = L_dl; r.c = L_c; r.after = snap();
    verbose = vsave; g_sim_lost = 0; GA[0x1d0] = 0; p_needs_cc = 0; T.in_brake = 0; g_inc.gap_m = NAN; sim_route_ok = 0; T.cruise = 0;
    return r;
}

static int test_cal_fit()
{
    int bad = 0;
    printf("\nSteering calibration, fit of a known wheel (plant gA / delay + lag / c; tolerance gA 10 %%, delay 0.08 s, c 15 %%; the inner band is |o1| < 0.5 m):\n");
    struct pc { float gA, delay, tw, c, kmh; const char *name; float ag; };   // ag: the plant's actuator gain (0 = 1); the fit reads gA per MODEL A, so gA x ag is expected
    const pc cs[] = { { 1.0f, 0, 0.3f, 0.19f, 80, "nominal wheel" }, { 1.0f, 0, 0.3f, 0.19f, 55, "nominal, 55 km/h" }, { 1.0f, 0, 0.3f, 0.19f, 95, "nominal, 95 km/h" },
                      { 0.7f, 0.30f, 0.05f, 0.17f, 80, "weak, slow wheel" }, { 1.5f, 0.15f, 0.05f, 0.21f, 80, "strong, quick wheel" }, { 1.3f, 0.35f, 0.10f, 0.15f, 70, "slow, strong wheel" },
                      { 22.0f, 0, 0.3f, 0.095f, 80, "ATS (A x.15, gA 22)", 0.15f }, { 22.0f, 0, 0.3f, 0.095f, 60, "ATS, 60 km/h", 0.15f }, { 15.0f, 0.15f, 0.05f, 0.12f, 80, "ATS, gA 15, c .12", 0.15f }, { 12.0f, 0.1f, 0.2f, 0.095f, 70, "ATS, A x.25, gA 12", 0.25f } };
    for (auto &c : cs) {
        pp m; m.gA = c.gA; m.delay = c.delay; m.tw = c.tw; m.c = c.c; if (c.ag > 0) m.ag = c.ag;
        use_layout(c.ag > 0); g_cal_cap = c.ag > 0 ? 2.0f : 0;
        calres r = cal_run(m, c.kmh, AB_NONE);
        use_layout(0); g_cal_cap = 0;
        double exp_dl = c.delay + c.tw, exp_gA = c.gA * (c.ag > 0 ? c.ag : 1);
        int ok = r.started && r.finished == 1 && r.aborted == 0 && fabs(r.gA / exp_gA - 1) <= 0.10 && fabs(r.dl - exp_dl) <= 0.08 && fabs(r.c / c.c - 1) <= 0.15 && r.peak_o1 < 0.5 && r.max_after < 0.5 && !r.rerun;
        printf("  %-20s %3.0f km/h: started %.1f s, ended %.1f s | gA %.2f (plant %.2f) delay %.2f (plant %.2f) c %.3f (plant %.3f) | peak |o1| %.2f m, |o2| %.3f | profile s %.2f dfac %.2f%s\n",
               c.name, c.kmh, r.t_start, r.t_end, r.gA, exp_gA, r.dl, exp_dl, r.c, c.c, r.peak_o1, r.peak_o2, r.after.s, r.after.d, ok ? "" : "  <-- WRONG");
        bad |= !ok;
    }
    printf("Calibration fit test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

static int test_cal_aborts()
{
    int bad = 0;
    printf("\nSteering calibration, every abort stops at once, plays the stop tone and leaves the profile alone (no run again without a new hold):\n");
    pp m;
    for (int ab = AB_INDICATOR; ab < AB_N; ab++) {
        calres r = cal_run(m, 80, ab, 2.0, 80), b = cal_run(m, 80, ab, 2.0, 80, false);   // b: the same disturbance with no calibration armed
        double allow = ab == AB_WIND ? 2.0 : 0.45;   // a drift is only seen once the heading error is 0.03 rad
        int ok = r.started && r.aborted == 1 && r.finished == 0 && same(r.before, r.after) && !r.rerun && r.t_end - r.t_start < 2.0 + allow && r.peak_o1 < 0.5 && (ab == AB_CRUISE || (r.max_after < 0.7 && r.max_after <= fmax(r.peak_o1, b.max_after) + 0.35));   // the centering law takes over again: no worse than the disturbance alone (or where the truck was); not with the cruise off, where nothing steers any more (15-11)
        printf("  %-30s: stopped %.2f s after the abort cue, tones: stop %d done %d, profile %s, peak |o1| %.2f, after %.2f m (no calibration: %.2f m)%s\n", ab_name[ab], r.t_end - r.t_start - 2.0, r.aborted, r.finished,
               same(r.before, r.after) ? "unchanged" : "CHANGED", r.peak_o1, r.max_after, b.max_after, ok ? "" : "  <-- WRONG");
        bad |= !ok;
    }
    printf("Calibration abort test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

// the 11-11 ride: the truck sways in its lane and the plain rate gate (0.08 m/s) never let the armed test start (300 s, then "no suitable stretch"; lane.csv:
// median |rate| 0.09 m/s on the straight, the longest stretch that passed was 2.7 s of the 3 s settle). Now the gate asks where the offset will be 1.2 s ahead.
static int test_cal_sway()
{
    int bad = 0;
    printf("\nSteering calibration start gate on a swaying truck (the 11-11 ride: median offset rate 0.09 m/s):\n");
    pp m;
    cal_setup(90, 31);
    pl p = { 0.03, 0, 0, 0, 0 };
    cal_arm(0);
    for (int i = 0; i < (int)(11.5 / DTS); i++) { t_now = i * DTS; step(p, m, DTS, 0.0002f * gauss(), 0, 0, 0.0002f * gauss()); }
    if (g_cal != CAL_ARMED) { printf("  setup: the test did not stay armed  <-- WRONG\n"); return 1; }
    struct row { float o1, do1; int must_pass; const char *what; };
    const row rows[] = {
        { 0.03f,  0.09f, 1, "centered, drifting out at the swaying truck's median rate (0.09 m/s)" },
        { 0.08f, -0.09f, 1, "at the edge of the start band, swaying back in" },
        { 0.08f,  0.09f, 1, "at the edge of the start band, swaying out at the median rate (0.08 + 1.2 x 0.09 = 0.19 m, inside twice the band)" },
        { 0.03f,  0.40f, 0, "centered but moving out at 0.4 m/s (0.51 m in 1.2 s: leaves the band)" },
        { 0.09f,  0.15f, 0, "near the edge and moving out at 0.15 m/s (0.27 m in 1.2 s)" },
        { 0.15f,  0.00f, 0, "off center by 0.15 m (over the start band)" },
    };
    for (const row &r : rows) {
        g_do1 = r.do1;
        const char *why = cal_unsuitable(90, r.o1, 0, 0, t_now);
        int ok = r.must_pass ? why == NULL : (why && !strcmp(why, "not centered and settled"));
        printf("  %-110s: %s%s\n", r.what, why ? why : "suitable", ok ? "" : "  <-- WRONG");
        bad |= !ok;
    }
    g_do1 = 0; g_cal = CAL_IDLE;
    printf("Calibration sway test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

// 11-11 t 1225-1275: the cruise went off with needs_cruise on, filter_a let the wheel go (A leaks to 0) while the law still computed requests, the actuator-gain
// estimator read "the game builds nothing" and slid 0.999 -> 0.2, every A term of the law was then amplified 5x and the truck ran away. The estimator must learn
// nothing while the wheel is let go (on ETS2 the snap to 1 hides the slide until the reading drops under 0.4; the ride's wheel let A leak further). ETS2, a swaying truck, 90 s with the cruise off.
static int test_ag_no_cruise()
{
    int bad = 0;
    printf("\nActuator gain with the cruise off (needs_cruise on, the wheel let go):\n");
    pp m;
    for (int cc = 0; cc < 2; cc++) {
        cal_setup(90, 41);
        p_learn = 1; p_needs_cc = 1; T.cruise = cc ? 1 : 0; g_ag = 1; AG_snapped = 1; AG_known = 0; AG_aa = AG_hh = AG_w = 0; AG_hold_until = -1e9; AG_t = -1e9;
        pl p = { 0.03, 0, 0, 0, 0 };
        float lo = 1, hi = 1;
        for (int i = 0; i < 30 * 90; i++) {
            t_now = i * DTS;
            step(p, m, DTS, 0.0002f * gauss(), 0, 0, 0.006f * sinf(0.7f * t_now) + 400 * 0.0002f * gauss());   // a heading disturbance that asks real requests (the ride's were up to 0.5)
            if (fabsf(p.o1) > 0.8) { p.o1 = 0.3; p.o2 = 0; }   // (the driver steers back)
            lo = fminf(lo, g_ag); hi = fmaxf(hi, g_ag);
        }
        // learned nothing while the wheel is let go (the estimator's accepted time), and still learns with the cruise on (else the row proves nothing)
        int ok = cc ? AG_w > 5 : AG_w < 1;
        printf("  cruise %-3s: the estimator accepted %5.1f s of data, reads %.2f, actuator gain %.3f .. %.3f (cruise off: must accept < 1 s; on: > 5 s)%s\n", cc ? "on" : "off", AG_w,
               AG_hh > 0 ? AG_aa / AG_hh : -1.0, lo, hi, ok ? "" : "  <-- WRONG");
        bad |= !ok;
        p_needs_cc = 0; T.cruise = 0;
    }
    printf("Actuator gain without cruise test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

// 15-11 (replaces W3 of 12-11, the game's own assist without the cruise): with needs_cruise and the cruise off the plugin writes no request at all, and the game's own
// law is let go like in an indicator pause (the lane key's decision, tap or hold, is only short in the air; cc_gate ends it). With the cruise on our law comes in.
static int test_game_own()
{
    int bad = 0;
    printf("\nWithout the cruise (needs_cruise on) there is no request, with it our law steers:\n");
    pp m;
    cal_setup(90, 51);
    p_needs_cc = 1; p_cal_timeout = 300; T.cruise = 0;
    pl p = { 0.5, 0, 0, 0, 0 };
    int steered = 0, frames = 0; float peakA = 0, peak_o1_cc = 0;
    const int nudge_i = (int)(20.0 / DTS) + 2;
    for (int i = 0; i < (int)(40.0 / DTS); i++) {
        t_now = i * DTS;
        if (t_now >= 20) T.cruise = 25;
        if (i == nudge_i) { p.o1 = 0.4; p.o2 = 0; }   // a nudge when our law takes over: it must bring the truck back
        step(p, m, DTS, 0.0002f * gauss(), 0, 0, 0.0002f * gauss());
        if (t_now > 1 && t_now < 19.9) {
            frames++;
            if (fabsf(*(float *)(GA + GA_U)) > 0.01f) steered++;
            peakA = fmaxf(peakA, fabsf((float)p.A));
        } else if (t_now > 30) peak_o1_cc = fmaxf(peak_o1_cc, fabsf((float)p.o1));
    }
    int ok1 = steered == 0, ok2 = peakA < 0.002f, ok3 = peak_o1_cc < 0.3f, ok4 = g_law_at > 20.0;
    printf("  cruise off, 1..20 s: the request is not 0 in %d of %d frames (must be 0)%s\n", steered, frames, ok1 ? "" : "  <-- WRONG");
    printf("  the game's A peaked at %.4f (must stay under 0.002: nothing steers)%s\n", peakA, ok2 ? "" : "  <-- WRONG");
    printf("  cruise on from 20 s, nudged 0.4 m off: peak |o1| after 30 s %.2f m (under 0.3)%s\n", peak_o1_cc, ok3 ? "" : "  <-- WRONG");
    printf("  our law ran again with the cruise on: %s%s\n", ok4 ? "yes" : "no", ok4 ? "" : "  <-- WRONG");
    bad = !(ok1 && ok2 && ok3 && ok4);
    p_needs_cc = 0; T.cruise = 0;
    printf("No lane request without the cruise test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

// 15-11, the owner's wish: with needs_cruise lane keeping exists only with the cruise. The lane key with the cruise off switches the cruise on (a tap) or only arms the calibration (a hold);
// the cruise going off switches lane keeping off; the cruise coming on alone does not switch it on. (key_reset, press, frames: the lane key test below.)
static int test_cc_key();

// Owner's wish W2 (12-11): the lane assist at every speed while the cruise is on. It steers from min_kmh (40 on the owner's ini, 20 as shipped); how low can it go?
// The law is fitted on 30..100 km/h (MODEL_KMH_LO / HI); below, the model's be/al are clamped at 30. Each speed x plant: start 0.8 m off and 0.03 rad of heading,
// a gust at 15 s, noise; the numbers are the settle (first time |o1| < 0.15 for good), the peak after the gust, swings (|o1| > 0.1 sign changes after 25 s),
// the peak request, and runaway guards fired. Hard rows: no runaway guard, the peak after the gust under 0.9 m, no swinging; GAP lines otherwise.
static int test_low_speed()
{
    int bad = 0, gaps = 0;
    printf("\nLane assist at low speed (the law is fitted on 30..100 km/h): settle s / peak after a gust m / swings / peak |u| / runaway guards\n");
    struct pv { const char *n; float c, gA, tw, dl, db; };
    const pv pls[] = { { "nominal", 0.19f, 1.0f, 0.3f, 0.0f, 0 }, { "ts4w-like gA 1.5", 0.17f, 1.5f, 0.05f, 0.33f, 0.0015f }, { "crash fit gA 2.95", 0.17f, 2.95f, 0.05f, 0.33f, 0.010f } };
    for (auto &q : pls) {
        for (float kmh : { 12.f, 15.f, 20.f, 25.f, 30.f, 40.f, 60.f }) {
            pp m; m.c = q.c; m.gA = q.gA; m.tw = q.tw; m.delay = q.dl; m.db = q.db;
            cal_setup(kmh, 61);
            p_learn = 0; p_min_kmh = 5; p_keep_on = 1; via_stub = 1;
            pl p = { 0.8, 0.03, 0, 0, 0 };
            int inc0 = g_n_incidents; double settle = -1, bad_since = 0; float peak_gust = 0, peak_u = 0; int swings = 0, hold = 0;
            for (int i = 0; i < (int)(60.0 / DTS); i++) {
                t_now = i * DTS;
                float wind = 0.0002f * gauss() + (t_now > 15 && t_now < 16 ? 0.02f : 0);
                step(p, m, DTS, 0.0002f * gauss(), 0, 0, wind);
                if (t_now < 15) { if (fabs(p.o1) > 0.15) bad_since = t_now; else if (settle < 0 && t_now - bad_since > 2) settle = bad_since; }
                if (t_now > 15 && t_now < 30) peak_gust = fmaxf(peak_gust, fabsf((float)p.o1));
                if (t_now > 25 && fabs(p.o1) > 0.1) { int sg = p.o1 > 0 ? 1 : -1; if (hold && sg != hold) swings++; hold = sg; }
                peak_u = fmaxf(peak_u, fabsf(g_u));
            }
            int guards = g_n_incidents - inc0;
            int ok = guards == 0 && peak_gust < 0.9f && swings == 0 && settle >= 0;
            printf("  %-18s %3.0f km/h: %5.1f / %.2f / %d / %.2f / %d%s\n", q.n, kmh, settle, peak_gust, swings, peak_u, guards, ok ? "" : (kmh >= 30 ? "  <-- WRONG" : "  GAP"));
            if (!ok) { if (kmh >= 30) bad = 1; else gaps++; }
        }
    }
    printf("Low speed test: %s (%d low-speed GAP rows under 30 km/h)\n", bad ? "FAILED" : "ok", gaps);
    return bad;
}

// The runaway guard (lanelaw.h) only acts above 40 km/h. A constant disturbance on the heading (rad/s; a crosswind, a camber the assist cannot hold) at lower speeds:
// the peak heading error and offset over 20 s, hands off, with the guard in the code as it is (it cannot fire under 40 km/h). Printed as GAP where the truck leaves the lane
// (1.5 m: half the lane's room) or the heading error passes the guard's own 0.18 rad, so the minimum speed is not lowered on the strength of the stability rows alone.
static int test_low_speed_runaway()
{
    printf("\nOne-sided disturbance below the runaway guard's 40 km/h (constant heading rate, hands off, 20 s): peak |o2| rad / peak |o1| m\n");
    int gaps = 0;
    for (float kmh : { 25.f, 35.f, 60.f }) {
        printf("  %3.0f km/h:", kmh);
        for (float wind : { 0.02f, 0.05f, 0.10f, 0.22f }) {
            const cfg c2 = { "runaway", 2 }; apply(c2); p_runaway = 1; vkmh = kmh; rng = 5; via_stub = 0;
            g_panic_until = 0; g_sat_dir[0] = g_sat_dir[1] = g_sat_dir[2] = 0; g_sat_t[0] = g_sat_t[1] = g_sat_t[2] = 0; g_sat_since = 0;
            pl p = { 0, 0, 0, 0, 0 }; pp m; float pk2 = 0, pk1 = 0; int fired = 0;
            for (int i = 0; i < (int)(20 / DTS); i++) { t_now = i * DTS; step(p, m, DTS, 0, 0, 0, wind); pk2 = fmaxf(pk2, fabsf((float)p.o2)); pk1 = fmaxf(pk1, fabsf((float)p.o1)); fired |= g_panic_until > t_now; }
            int gap = pk1 > 1.5f || pk2 > 0.18f;
            printf("  wind %.2f: %.3f / %.2f%s%s", wind, pk2, pk1, fired ? " (guard fired)" : "", gap ? " GAP" : "");
            gaps += gap && kmh < 40;
        }
        printf("\n");
    }
    printf("Low speed disturbance rows: %d GAP below 40 km/h\n", gaps);
    return 0;
}

// armed but never suitable: nothing starts, the profile stays, the timeout disarms with the stop tone
static int test_cal_waits()
{
    int bad = 0;
    printf("\nSteering calibration, unsuitable stretches (nothing starts) and the timeout (20 s here):\n");
    enum { W_SLOW, W_LEAD, W_BRAKE, W_ROUTE, W_NOCRUISE, W_N };
    const char *name[W_N] = { "speed under the band (40 km/h)", "a vehicle 3 s ahead", "braking", "bend ahead on the route", "cruise off (needs_cruise)" };
    pp m;
    for (int w = 0; w < W_N; w++) {
        cal_setup(w == W_SLOW ? 40 : 80, 31);
        p_cal_timeout = 20;
        if (w == W_LEAD) g_inc.gap_m = 3.0f * 80 / 3.6f;
        if (w == W_BRAKE) T.in_brake = 0.1f;
        if (w == W_ROUTE) { sim_route_ok = 1; sim_route_straight = 100; }
        if (w == W_NOCRUISE) p_needs_cc = 1;
        prof_snap b = snap();
        pl p = { 0.03, 0, 0, 0, 0 };
        cal_arm(0);
        int started = 0; double off_at = -1;
        for (int i = 0; i < 30 * 60; i++) {
            t_now = i * DTS;
            step(p, m, DTS, 0.0002f * gauss(), 0, 0, 0.0002f * gauss());
            started |= g_cal == CAL_RUN;
            if (g_cal == CAL_IDLE && off_at < 0) off_at = t_now;
        }
        int ok = !started && off_at > 19.9 && off_at < 20.2 && count_cue(CUE_LANE_CAL_ABORT) == 1 && same(b, snap());
        printf("  %-32s: started %s, disarmed at %.1f s (timeout 20), stop tone %d, profile %s%s\n", name[w], started ? "YES" : "no", off_at, count_cue(CUE_LANE_CAL_ABORT),
               same(b, snap()) ? "unchanged" : "CHANGED", ok ? "" : "  <-- WRONG");
        bad |= !ok;
        p_needs_cc = 0; g_inc.gap_m = NAN; T.in_brake = 0; sim_route_ok = 0;
    }
    // the timeout also runs from the frame (no law calls: lane keeping off)
    cal_setup(80, 31); p_cal_timeout = 20; o_law = fake_law; GA[0x1d1] = 0;
    cal_arm(0);
    double off_at = -1;
    for (int i = 0; i < 30 * 60; i++) { t_now = i * DTS; ln_frame(); if (g_cal == CAL_IDLE && off_at < 0) off_at = t_now; }
    int ok = off_at > 19.9 && off_at < 20.2;
    printf("  %-32s: disarmed at %.1f s%s\n", "lane keeping off (frame only)", off_at, ok ? "" : "  <-- WRONG");
    bad |= !ok;
    printf("Calibration wait/timeout test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

// The lane-keeping key: found from its presses, tap = toggle with its tone when the key is up, hold = arm / disarm, control 0 = plain toggle.
static char ACT[6][0x100];
static void key_reset(int control)
{
    apply({ "mode 2", 2 });
    p_learn = 1; L_loaded = 1; snprintf(L_key, sizeof L_key, "SimWheel"); g_ffb = 1; p_control = control; o_law = fake_law; via_stub = 0;
    memset(GA, 0, sizeof GA); memset(ACT, 0, sizeof ACT); GA[0x1d1] = 0x20;
    *(void **)(GA + 0x1e780) = ACT[3];   // the lane key
    *(void **)(GA + 0x1e760) = ACT[1];   // another key
    *(void **)(GA + 0x1e610) = ACT[2];   // a readable object nobody presses
    *(void **)(GA + 0x1e710) = ACT[4];   // a cruise key's slot: never looked at
    g_key_off = g_key_bad = 0; K_miss = 0; K_down_at = K_up_at = K_fire_at = 0; K_fired = 0; K_pend = {}; memset(K_ptr, 0, sizeof K_ptr); memset(K_ok, 0, sizeof K_ok);
    for (int i = 0; i < KEY_N; i++) { K_nz[i] = 0; K_zero[i] = K_bad[i] = K_votes[i] = 0; }
    g_cal = CAL_IDLE; g_cues.clear(); T.speed = 80 / 3.6f; T.cruise = 0; g_pause = 0;
    g_lka_expect = -1;
}
static void frames(double s) { for (int i = 0; i < (int)(s * 60 + 0.5); i++) { t_now += 1.0 / 60; ln_frame(); } }
// the key (ACT[slot]) down for `down` s; toggles the lane-keeping bit on the press when asked, like the game
static void press(int slot, double down, bool toggles, int also = -1)
{
    t_now += 1.0 / 60;
    if (toggles) GA[0x1d1] ^= 0x20;
    *(int *)(ACT[slot] + 0xf8) = 2; if (also >= 0) *(int *)(ACT[also] + 0xf8) = 2;
    ln_frame();
    *(int *)(ACT[slot] + 0xf8) = 1; if (also >= 0) *(int *)(ACT[also] + 0xf8) = 1;
    frames(down);
    *(int *)(ACT[slot] + 0xf8) = 0; if (also >= 0) *(int *)(ACT[also] + 0xf8) = 0;
    frames(0.6);
}
static int bit_on() { return (GA[0x1d1] & 0x20) != 0; }
static int test_key()
{
    int bad = 0;
    printf("\nThe lane-keeping key (control 1):\n");
    auto check = [&](const char *what, bool ok) { printf("  %-78s %s\n", what, ok ? "ok" : "<-- WRONG"); bad |= !ok; };
    key_reset(1);
    frames(0.5);
    press(3, 0.15, true, 1);     // tap 1 with another key down at the same time: two candidates, no vote
    check("tap 1 (another key down too): off, tone at once, key not found", !bit_on() && g_cues.size() == 1 && g_cues[0] == CUE_OFF && !g_key_off);
    press(3, 0.15, true);        // tap 2: first vote
    check("tap 2: on, tone at once, key not found yet (one vote)", bit_on() && g_cues.size() == 2 && g_cues[1] == CUE_ON && !g_key_off);
    press(3, 0.15, true);        // tap 3: second vote in a row
    check("tap 3: off, tone at once, key found at game_actor+0x1e780", !bit_on() && g_cues.size() == 3 && g_key_off == 0x1e780);
    press(3, 0.15, true);        // tap 4, the key is known: the tone waits for the release
    check("tap 4 (key known): on, one tone", bit_on() && g_cues.size() == 4 && g_cues[3] == CUE_ON);
    // the tone comes with the release, not with the press
    {
        size_t n0 = g_cues.size();
        t_now += 1.0 / 60; GA[0x1d1] ^= 0x20; *(int *)(ACT[3] + 0xf8) = 2; ln_frame();
        *(int *)(ACT[3] + 0xf8) = 1; frames(0.3);
        size_t during = g_cues.size() - n0;
        *(int *)(ACT[3] + 0xf8) = 0; frames(0.6);
        check("tap 5: no tone while the key is down, the tone after the release", during == 0 && g_cues.size() == n0 + 1 && g_cues.back() == CUE_OFF && !bit_on());
    }
    press(3, 0.5, true);         // back on
    g_cues.clear();
    // hold 1.2 s from on: the press switched it off, 0.8 s later it is on again and the calibration is armed
    press(3, 1.2, true);
    check("hold from on: lane keeping on, calibration armed, one arm tone, no on/off tone", bit_on() && g_cal == CAL_ARMED && g_cues.size() == 1 && g_cues[0] == CUE_LANE_CAL_ARM);
    g_cues.clear();
    press(3, 1.2, true);
    check("hold again: disarmed, lane keeping on, one stop tone", bit_on() && g_cal == CAL_IDLE && g_cues.size() == 1 && g_cues[0] == CUE_LANE_CAL_ABORT);
    g_cues.clear();
    press(3, 0.3, true);         // off
    press(3, 1.2, true);         // hold from off: the press switched it on, armed
    check("hold from off: lane keeping on, armed", bit_on() && g_cal == CAL_ARMED);
    g_cues.clear();
    press(3, 0.3, true);         // tap while armed: off, stays armed
    check("tap while armed: off, still armed", !bit_on() && g_cal == CAL_ARMED);
    press(3, 0.3, true);
    g_cal = CAL_IDLE; g_cues.clear();
    press(3, 0.6, true);         // shorter than 0.8 s = a tap
    check("0.6 s press (under calib_hold_s) is a tap", g_cal == CAL_IDLE && g_cues.size() == 1);
    // the letter
    {
        press(3, 0.3, true);     // on
        g_cues.clear();
        t_now += 1.0 / 60; GA[0x1d1] ^= 0x20; *(int *)(ACT[3] + 0xf8) = 2; ln_frame(); *(int *)(ACT[3] + 0xf8) = 1;
        frames(0.9);
        char c = disp_pick();
        *(int *)(ACT[3] + 0xf8) = 0; frames(0.6);
        check("the dashboard shows T once armed", c == 'T');
        g_cal = CAL_IDLE;
    }
    // how the game handles a long press is not known: toggle on the press (above), on the release, or not at all; the hold must arm in all three
    for (int mode = 1; mode <= 2; mode++) {
        key_reset(1);
        frames(0.5);
        press(3, 0.15, true); press(3, 0.15, true); press(3, 0.15, true); press(3, 0.15, true);   // found; bit on
        g_cues.clear();
        t_now += 1.0 / 60; *(int *)(ACT[3] + 0xf8) = 2; ln_frame(); *(int *)(ACT[3] + 0xf8) = 1; frames(1.2);
        *(int *)(ACT[3] + 0xf8) = 0; if (mode == 1) GA[0x1d1] ^= 0x20; frames(0.8);   // mode 1: the game toggles on the release; mode 2: not at all
        check(mode == 1 ? "hold, the game toggles on the release: armed, lane keeping on, only the arm tone" : "hold, the game does nothing: armed, lane keeping on, only the arm tone",
              bit_on() && g_cal == CAL_ARMED && g_cues.size() == 1 && g_cues[0] == CUE_LANE_CAL_ARM);
    }
    // control 0: plain toggles, no search, no arming
    key_reset(0);
    frames(0.5);
    press(3, 1.2, true);
    press(3, 0.15, true); press(3, 0.15, true); press(3, 0.15, true);
    check("control 0: a hold is a toggle (tone at once), nothing armed, no key search", g_cal == CAL_IDLE && !g_key_off && g_cues.size() == 4 && count_cue(CUE_LANE_CAL_ARM) == 0);
    // a game-made toggle (not a press) three times: the key is looked for again
    key_reset(1);
    frames(0.5);
    press(3, 0.15, true); press(3, 0.15, true); press(3, 0.15, true);
    bool found = g_key_off == 0x1e780;
    for (int k = 0; k < 3; k++) { t_now += 0.7; GA[0x1d1] ^= 0x20; frames(0.5); }
    check("three toggles without the key down: the key is looked for again", found && g_key_off == 0);
    // a key that reads held for 6 s: tap and hold off for the session
    key_reset(1);
    frames(0.5);
    press(3, 0.15, true); press(3, 0.15, true); press(3, 0.15, true);
    *(int *)(ACT[3] + 0xf8) = 1; frames(7);
    check("a key stuck at held for 6 s: given up, a press is a plain toggle", g_key_bad == 1);
    printf("Lane key test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

static int test_cc_key()
{
    int bad = 0;
    printf("\nThe lane key with needs_cruise (15-11):\n");
    auto check = [&](const char *what, bool ok) { printf("  %-96s %s\n", what, ok ? "ok" : "<-- WRONG"); bad |= !ok; };
    // key_reset leaves lane keeping on and the cruise off; start from lane keeping off, the key known (found earlier), 80 km/h
    auto start = [&](int control, int eng_ok, int lag) {
        static double base = 1e5; base += 1000; t_now = base;   // (the frame's 0.1 s poll keeps its last time in a static: the earlier tests' clocks were reset)
        key_reset(control); g_key_off = control ? 0x1e780 : 0; p_needs_cc = 1; GA[0x1d1] = 0; T.cruise = 0; T.speed = 80 / 3.6f;
        g_eng_ok = eng_ok; g_eng_n = 0; g_eng_lag = lag; g_cross_at = -10; g_cc_wait_at = g_cc_grace = -1;
        frames(1.0); g_cues.clear();   // (the key has been up for a while: K_up_at is 'up since')
    };
    auto dump = [&]() { if (getenv("CCDBG")) { printf("    [cues:"); for (int c : g_cues) printf(" %d", c); printf("; eng_n %d bit %d]\n", g_eng_n, bit_on()); } };
    auto cues_are = [&](int a, int b) { dump(); return (int)g_cues.size() == (a >= 0) + (b >= 0) && (a < 0 || g_cues[0] == a) && (b < 0 || g_cues[1] == b); };
    // 1. a tap, the cruise off, the cruise can come on
    start(1, 1, 0);
    press(3, 0.15, true);
    check("tap, cruise off: the cruise is switched on once, lane keeping on, one on tone", g_eng_n == 1 && bit_on() && cues_are(CUE_ON, -1));
    // 2. the cruise goes off (brake, AEB, the key: anything)
    g_cues.clear(); T.cruise = 0; frames(0.1);
    check("cruise goes off: lane keeping off at once, one off tone, one X", !bit_on() && cues_are(CUE_OFF, -1) && disp_pick() == 'X');
    frames(0.5);
    check("  and it stays off, no second tone", !bit_on() && g_cues.size() == 1 && g_eng_n == 1);
    // 3. the cruise cannot come on (under its minimum speed)
    start(1, 0, 0);
    press(3, 0.15, true);
    check("tap, cruise cannot come on (refused): lane keeping off, one off tone, no on tone", g_eng_n == 1 && !bit_on() && cues_are(CUE_OFF, -1));
    // 4. a hold: only the calibration is armed
    start(1, 1, 0);
    press(3, 1.2, true);
    check("hold, cruise off: no cruise-on, lane keeping off, calibration armed", g_eng_n == 0 && !bit_on() && g_cal == CAL_ARMED && T.cruise == 0);
    check("  the arm tone is given, no on tone", count_cue(CUE_LANE_CAL_ARM) == 1 && count_cue(CUE_ON) == 0);
    // 4b. hold where the game toggles on the release
    start(1, 1, 0);
    t_now += 1.0 / 60; *(int *)(ACT[3] + 0xf8) = 2; ln_frame(); *(int *)(ACT[3] + 0xf8) = 1; frames(1.2);
    *(int *)(ACT[3] + 0xf8) = 0; GA[0x1d1] ^= 0x20; frames(0.8);
    check("hold, the game toggles on the release: no cruise-on, lane keeping off, armed", g_eng_n == 0 && !bit_on() && g_cal == CAL_ARMED && count_cue(CUE_ON) == 0);
    // 5. Resume or Set alone: the cruise comes on, lane keeping does not
    start(1, 1, 0);
    T.cruise = 80 / 3.6f; frames(2);
    check("cruise on alone (Resume/Set): lane keeping stays off, the cruise is not touched", !bit_on() && g_eng_n == 0 && g_cues.empty());
    // 6. the game switching lane keeping on by itself (not the key): off again, silently, the cruise not switched on
    start(1, 1, 0);
    GA[0x1d1] |= 0x20; frames(0.5);
    check("the game's own switch-on without the key: off again, no cruise-on, no tone", !bit_on() && g_eng_n == 0 && g_cues.empty());
    // 7. the plugin itself never switches lane keeping on with the cruise off (indicator resume, hold, the phone)
    start(1, 1, 0);
    { char *ga = sense_actor(); lka_set(ga, 1); }
    check("lka_set(on) with the cruise off is refused", !bit_on());
    // 8. the key not found (control 0): every toggle is a tap
    start(0, 1, 0);
    press(3, 0.15, true);
    check("key not known (control 0): a toggle on is a tap, the cruise is switched on, lane keeping on", g_eng_n == 1 && bit_on());
    start(0, 0, 0);
    press(3, 0.15, true);
    check("key not known, refused: lane keeping off again, one off tone", g_eng_n == 1 && !bit_on() && count_cue(CUE_OFF) == 1 && count_cue(CUE_ON) == 0);
    // 9. the cruise byte does not show within a second after we switched it on: lane keeping off
    start(1, 1, 1);
    press(3, 0.15, true);
    frames(1.5);
    check("cruise switched on but never shows (1 s): lane keeping off again", g_eng_n == 1 && !bit_on());
    // 10. the cruise byte shows a moment late (a tick): lane keeping stays on
    start(1, 1, 1);
    press(3, 0.15, true); T.cruise = 80 / 3.6f; frames(0.1);
    check("cruise byte shows late: lane keeping stays on", bit_on() && g_eng_n == 1);
    // 11. needs_cruise 0: as before, no cruise-on, lane keeping independent of the cruise
    start(1, 1, 0);
    p_needs_cc = 0;
    press(3, 0.15, true);
    check("needs_cruise 0: the tap toggles lane keeping with the cruise off, the cruise untouched", bit_on() && g_eng_n == 0 && T.cruise == 0 && cues_are(CUE_ON, -1));
    frames(2);
    check("  and it stays on with the cruise off", bit_on());
    // 12. mode 0 (the game's own law steers; a friend's ini, 15-11): the switch works the same, the key is not known (control needs mode 2)
    {   const float msave = p_mode;
        start(0, 1, 0); p_mode = 0;
        press(3, 0.15, true);
        check("mode 0, tap with the cruise off: the cruise is switched on, lane keeping on, one on tone", g_eng_n == 1 && bit_on() && cues_are(CUE_ON, -1));
        g_cues.clear(); T.cruise = 0; frames(0.1);
        check("mode 0, the cruise goes off: lane keeping off, one off tone", !bit_on() && cues_are(CUE_OFF, -1));
        start(0, 1, 0); p_mode = 0;
        T.cruise = 80 / 3.6f; frames(2);
        check("mode 0, the cruise on alone (Resume/Set): lane keeping stays off", !bit_on() && g_eng_n == 0 && g_cues.empty());
        p_mode = msave; }
    p_needs_cc = 0; T.cruise = 0; g_cal = CAL_IDLE; g_eng_ok = 1;
    printf("Lane key with needs_cruise test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

// Shared steering: the offset pull's back-off share swept across follow_bend_o3 (hand on the wheel, activity 0.7): it stepped by (1 - follow_backoff) there
static int follow_edge_test()
{
    for (auto &p : g_params) *p.val = p.def;
    p_follow = 1;
    float worst = 0, prev = follow_fb(0.0030f, 0.7f);
    for (int i = 1; i <= 200; i++) { float fb = follow_fb(0.0030f + 0.00001f * i, 0.7f); worst = fmaxf(worst, fabsf(fb - prev)); prev = fb; }
    float at_bend = follow_fb(p_f_bend, 0.7f), far_in = follow_fb(0.5f * p_f_bend, 0.7f);
    printf("shared steering at follow_bend_o3 (|o3| 0.0030-0.0050, 1e-5 steps): largest step of the offset-pull factor %.4f (was %.2f), factor 1 at the bend value %.2f, backed off to %.2f inside\n", worst, 1 - p_f_back, at_bend, far_in);
    int bad = worst > 0.05f || at_bend != 1.0f || far_in > p_f_back + 0.01f;
    printf("follow edge test: %s\n", bad ? "FAIL" : "ok");
    p_follow = 0;
    return bad;
}

// ---------------------------------------------------------------------------------------------------------------------------------------------
// WP-LA5: roundabouts and fast bends from the live lateral load (not from the cruise cancelling for a roundabout the route knew).
// 25-10 16:00: a prefab 148 m ahead was no known junction or roundabout; in it the game's lane reference jumped (o1 +0.13 -> -1.09 m in 2 s, heading error -0.186 rad),
// the game's request and ours went to -1.0 together, the runaway guard handed the wheel to the game's law (which sat at the ceiling too) and the driver steered in at
// 7.2 m/s2 at 52 km/h (rollover guard level 2). Here: the same lane reading in a synthetic roundabout (R 25 m, 7 m/s2 at 52 km/h), the driver steering it (a driver who looks
// at the road, drv 3, from 0.3 s into the entrance), the assist yielding to his hand (roundabout_off 1) and not (0).
struct rbres { double w50, guard_at, mean_a, peak_a, mean_u, peak_u, peak_o1, peak_acc, end_o1; int ceil_frames; double guard_u; int guard_n; };   // guard_u / guard_n: sum of |u| and frames while the runaway guard's hand-over is in force
static rbres run_roundabout(const pp &m, float kmh, int on, int drv, float o3_peak = 0.034f, bool jump = true, double hold_s = 6, double ramp_s = 1.5, float ag0 = 0)
{
    const cfg c2 = { "rb", 2 }; apply(c2);
    p_rb_off = on;
    if (ag0 > 0) { g_ag = ag0; AG_ann = ag0; AG_known = 1; AG_snapped = 0; }   // an actuator gain already measured (a stored profile, or after a few seconds of requests)
    g_lat = lat_state(); g_latw = 1; g_lat_off = 0; g_lat_w_at = -1e9; g_lat_r = { 0, 0, 0 }; g_roll_guard_k = 1.0f;
    via_stub = 1; memset(GA, 0, sizeof GA); GA[0x1d1] = 0x20; g_law_at = -1e9; g_panic_until = 0; g_soft_at = g_soft_end = -1; rng = 77; g_sim_lat = 1;
    pl p = { 0, 0, 0, 0, 0 }; vkmh = kmh; float dt = DTS, os = 0;
    const double t0 = 30;                       // the entrance
    rbres r = { -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; double sa = 0, su = 0; int n = 0;
    for (int i = 0; i < (int)((t0 + ramp_s + hold_s + 8) / dt); i++) {
        t_now = 5000 + i * dt;
        const double t = i * dt - t0;           // seconds from the entrance
        auto ramp = [](double x, double a) { double k = x <= 0 ? 0 : x >= a ? 1 : x / a; return k * k * (3 - 2 * k); };
        float o3 = t < 0 ? ou_o3(os, dt, 0.0010f) : (float)(o3_peak * (ramp(t, ramp_s) - ramp(t - ramp_s - hold_s, ramp_s)));
        g_sim_do1 = jump ? (float)(-1.1 * (ramp(t - 0.3, 1.5) - ramp(t - ramp_s - hold_s, 1.0))) : 0;     // the lane reference of the prefab
        g_sim_do2 = jump ? (float)(0.2 * (ramp(t - 0.3, 1.5) - ramp(t - ramp_s - hold_s, 1.0))) : 0;
        float w_cmd = t < 0.3 ? 0.0f : -(m.d / m.c) * o3;   // the driver turns in 0.3 s into the entrance and steers the whole bend (he knows his own truck's wheel for a bend)
        lat_frame();
        step(p, m, dt, o3, t < 0.3 ? 0 : drv, w_cmd, 0.0004f * gauss());
        if (t < 0) continue;
        float u = *(float *)(GA + GA_U);
        if (getenv("TRACE_RB") && atoi(getenv("TRACE_RB")) == on + 2 * (int)(m.gA * 10) && fmod(t, 0.25) < dt) printf("    %5.2f o1 %6.2f o2 %6.3f o3 %.4f u %5.2f A %7.4f w %7.4f act %.2f hand %.2f lw %.2f eff %.2f acc %.1f lead %.1f guard %d hold %.2f\n", t, p.o1, p.o2, o3, u, p.A, p.w, g_act, lat_hand(), g_latw, lat_eff(), g_lat_r.accel, g_lat_r.lead, t_now < g_panic_until, fabsf(g_hwheel) >= 0.03f ? fabsf(g_hdev) / fabsf(g_hwheel) : -1.0f);
        if (r.w50 < 0 && lat_eff() < 0.5f) r.w50 = t;
        if (r.guard_at < 0 && t_now < g_panic_until) r.guard_at = t;
        if (t_now < g_panic_until) { r.guard_u += fabs(u); r.guard_n++; }
        if (t < ramp_s + hold_s) { sa += fabs(p.A); su += fabs(u); n++; r.peak_u = fmax(r.peak_u, fabs(u)); r.ceil_frames += fabs(u) > 0.95f; r.peak_a = fmax(r.peak_a, fabs(p.A)); }
        r.peak_o1 = fmax(r.peak_o1, fabs(p.o1));
        r.peak_acc = fmax(r.peak_acc, fabs(T.yaw) * 6.2831853 * kmh / 3.6);
        if (t > ramp_s + hold_s + 6) r.end_o1 = p.o1;
    }
    r.mean_a = n ? sa / n : 0; r.mean_u = n ? su / n : 0;
    g_sim_lat = 0; g_sim_do1 = g_sim_do2 = 0; T.yaw = 0; T.in_steer = 0; via_stub = 0;
    return r;
}

static int lat_tests()
{
    int bad = 0;
    printf("\nThe lateral load (core/latload.h) and the assist that yields to the driver's hand under it\n");
    {   // the weight function: whole up to edge_share of the limit, nothing at the limit
        const float L = 6.27f;
        float w0 = lat_weight(0.5f * L, L, 0.7f), w1 = lat_weight(0.7f * L, L, 0.7f), w2 = lat_weight(0.85f * L, L, 0.7f), w3 = lat_weight(L, L, 0.7f), w4 = lat_weight(1.6f * L, L, 0.7f);
        int ok = w0 == 1 && w1 == 1 && fabsf(w2 - 0.5f) < 1e-3f && w3 == 0 && w4 == 0 && lat_weight(9, 0, 0.7f) == 1 && lat_weight(9, L, 0.0f) >= 0 && lat_weight(0, L, 5.0f) == 1;
        printf("  weight at 0.5 / 0.7 / 0.85 / 1.0 / 1.6 x the limit: %.2f %.2f %.2f %.2f %.2f %s\n", w0, w1, w2, w3, w4, ok ? "ok" : "<-- WRONG");
        bad |= !ok;
    }
    {   // the steering leads: a regression of the measured on v^2 x steering predicts the load before the body has it
        lat_state S; double t = 0; float v = 20;
        for (int i = 0; i < 60 * 40; i++) { t += 1.0 / 60; float st = 0.05f * sinf(6.2831853f * (float)t / 8), a = 0.227f * v * v * st; lat_step(S, v, a / v / 6.2831853f, st, t); }   // 40 s of gentle bends: a = 0.227 v^2 st
        lat_read r = lat_step(S, v, 0.0f, 0.12f, t + 1.0 / 60);   // the wheel turns in, nothing measured yet (the filter still holds the last bend's tail)
        float want = 0.227f * v * v * 0.12f;
        int ok = r.lead > 0.8f * want && r.lead <= 1.01f * want && r.accel >= r.lead;
        printf("  steering 0.12 at 20 m/s with the body still: the steering predicts %.1f m/s2 (the truck's own gain gives %.1f), measured %.1f %s\n", r.lead, want, r.meas, ok ? "ok" : "<-- WRONG");
        bad |= !ok;
        lat_state D; t = 0;   // no evidence, a digital wheel at full lock: nothing predicted
        for (int i = 0; i < 60 * 3; i++) { t += 1.0 / 60; lat_step(D, v, 0.0f, 1.0f, t); }
        r = lat_step(D, v, 0.0f, 1.0f, t + 1.0 / 60);
        ok = r.lead == 0 && r.accel == r.meas;
        printf("  a key at full lock, no evidence: predicts %.1f %s\n", r.lead, ok ? "ok" : "<-- WRONG");
        bad |= !ok;
    }
    printf("Unknown roundabout (R 25 m, 7 m/s2 at 52 km/h; the lane reference jumps -1.1 m / +0.2 rad; the driver steers it): the assist without / with its yielding\n");
    printf("  %-22s %-5s | yield <0.5 | guard | mean|A| peak|A| | mean|u| peak|u| ceil | peak |o1| true | o1 8 s after | lateral\n", "plant", "yield");
    struct pv { const char *n; float gA, db, dl, tw, c; };
    const pv pls[] = { { "nominal (gA 1, lag .3)", 1.0f, 0, 0, 0.3f, 0.19f }, { "ts4w-like (gA 1.5)", 1.5f, 0.0015f, 0.33f, 0.05f, 0.17f }, { "crash fit (gA 2.95)", 2.95f, 0.010f, 0.33f, 0.05f, 0.21f } };
    for (auto &q : pls) {
        pp m; m.gA = q.gA; m.db = q.db; m.delay = q.dl; m.tw = q.tw; m.c = q.c; m.d = q.c * BEND_NEED;
        rbres off = run_roundabout(m, 52, 0, 3), on = run_roundabout(m, 52, 1, 3);
        for (int k = 0; k < 2; k++) {
            const rbres &r = k ? on : off;
            printf("  %-22s %-5s | %-10.2f | %-5.2f | %.4f  %.4f | %.2f    %.2f   %-4d| %-14.2f | %-12.2f | %.1f m/s2\n", q.n, k ? "on" : "off", r.w50, r.guard_at, r.mean_a, r.peak_a, r.mean_u, r.peak_u, r.ceil_frames, r.peak_o1, r.end_o1, r.peak_acc);
        }
        // the assist yields while the driver holds the wheel through the load (the weight under 0.5, within 2.5 s of the entrance: the hold has to show first, so on two of
        // the three wheels the runaway guard has fired by then and the guard is not beaten; what the yield changes is the push against the driver, in the guard's window too),
        // asks less, and the driver holds the lane no worse than without
        // while the runaway guard's hand-over is in force (the game's own request at the ceiling, handed to the wheel) and a hand is on it, what goes out is faded: the mean |u|
        // inside that window is at most half of what goes out without the yield
        const double gu_on = on.guard_n ? on.guard_u / on.guard_n : 0, gu_off = off.guard_n ? off.guard_u / off.guard_n : 0;
        int ok = on.w50 >= 0 && on.w50 <= 2.5 && off.w50 < 0 && on.mean_u < off.mean_u && on.peak_o1 <= off.peak_o1 + 0.05 && (gu_off == 0 || gu_on <= 0.5 * gu_off);
        printf("    -> mean |u| inside the guard's hand-over: %.2f without, %.2f with (%d / %d frames) %s\n", gu_off, gu_on, off.guard_n, on.guard_n, ok ? "ok" : "<-- WRONG (the assist must yield within 2.5 s, ask less, fade what the guard hands over, and the driver hold the lane no worse)");
        bad |= !ok;
    }
    printf("Hands off nothing changes (the 5.7 m/s2 bend of the ts5b crash, 0.9 x the limit, at 63 km/h; a wheel that holds it): the run with and without the yielding is the same, bit for bit\n");
    for (int w = 0; w < 2; w++) {
        pp m; if (w) { m.gA = 2.95f; m.db = 0.010f; m.delay = 0.33f; m.tw = 0.05f; m.c = 0.21f; m.d = m.c * BEND_NEED; } else { m.gA = 1.5f; m.db = 0.0015f; m.delay = 0.33f; m.tw = 0.05f; m.c = 0.17f; m.d = m.c * BEND_NEED; }
        rbres a = run_roundabout(m, 63, 0, 0, 0.0187f, false, 12), b = run_roundabout(m, 63, 1, 0, 0.0187f, false, 12);
        float acc = (63 / 3.6f) * (63 / 3.6f) * 0.0187f;
        int ok = a.peak_o1 == b.peak_o1 && a.mean_u == b.mean_u && a.mean_a == b.mean_a && a.guard_at == b.guard_at && b.w50 < 0;
        printf("  %-20s hands off (%.1f m/s2): peak |o1| %.2f / %.2f m, mean |u| %.3f / %.3f, runaway guard %s / %s, yielded %s %s\n", w ? "crash fit (gA 2.95)" : "ts4w-like (gA 1.5)", acc, a.peak_o1, b.peak_o1, a.mean_u, b.mean_u,
               a.guard_at >= 0 ? "yes" : "no", b.guard_at >= 0 ? "yes" : "no", b.w50 >= 0 ? "YES" : "no", ok ? "ok" : "<-- WRONG");
        bad |= !ok;
    }
    printf("Hands off on an ATS 1.61 truck (A x0.15 per request, gA 22, c .095; the actuator gain read as 1 until it has been measured, then 0.15): the same bend, the same identity\n");
    for (int conv = 0; conv < 2; conv++) {
        pp m; m.ag = 0.15f; m.gA = 22.0f; m.c = 0.095f; m.d = m.c * BEND_NEED;
        use_layout(1);
        rbres a = run_roundabout(m, 63, 0, 0, 0.0187f, false, 12, 1.5, conv ? 0.15f : 0), b = run_roundabout(m, 63, 1, 0, 0.0187f, false, 12, 1.5, conv ? 0.15f : 0);
        use_layout(0);
        int ok = a.peak_o1 == b.peak_o1 && a.mean_u == b.mean_u && a.mean_a == b.mean_a && a.guard_at == b.guard_at && b.w50 < 0;
        printf("  ATS, actuator gain %s: hands off, peak |o1| %.2f / %.2f m, mean |u| %.3f / %.3f, runaway guard %s / %s, yielded %s %s\n", conv ? "converged" : "not yet measured", a.peak_o1, b.peak_o1, a.mean_u, b.mean_u,
               a.guard_at >= 0 ? "yes" : "no", b.guard_at >= 0 ? "yes" : "no", b.w50 >= 0 ? "YES" : "no", ok ? "ok" : "<-- WRONG");
        bad |= !ok;
    }
    printf("Ordinary bends with a hand on the wheel: no yielding up to 70 %% of the limit (4.4 m/s2)\n");
    for (float kmh : { 60.0f, 80.0f }) for (float o3p : { 0.0035f, 0.0080f }) {
        pp m;
        rbres r = run_roundabout(m, kmh, 1, 1, o3p, false, 20);
        float a = (kmh / 3.6f) * (kmh / 3.6f) * o3p;
        int ok = a < 4.0f ? r.w50 < 0 : 1;
        printf("  o3 %.4f at %.0f km/h, hands on: %.1f m/s2 (%.2f x the limit) | yielded: %s | peak |o1| %.2f m %s\n", o3p, kmh, a, a / 6.27f, r.w50 >= 0 ? "yes" : "no", r.peak_o1, ok ? "ok" : "<-- WRONG");
        bad |= !ok;
    }
    printf("Lateral load test: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

static int main_inner(int argc, char **argv);
int main(int argc, char **argv)
{
    const bool whole_run = argc <= 1 || (atoi(argv[1]) > 0 && !strchr(argv[1], '='));   // the default run, with or without the sweep flag
    int rc = main_inner(argc, argv);
    if (g_lv_viol) { printf("\nLearner contract: %ld violations, the first at %s\n", g_lv_viol, g_lv_first); rc |= 1; }
    else if (whole_run) printf("\nLearner contract (every change of a learned value announced by a log line, every learner write inside its bounds): %ld plant steps checked, announcements: s %d, dfac %d, ceiling %d, actuator gain %d, session factor %d, 0 violations: ok\n", g_lv_steps, g_lv_n[0], g_lv_n[1], g_lv_n[2], g_lv_n[3], g_lv_n[4]);
    return rc;
}
static int main_inner(int argc, char **argv)
{
    sim_env_apply("lane", g_params, sizeof g_params / sizeof g_params[0]);
    atexit(print_refmax);
    if (argc > 1 && !strcmp(argv[1], "learn")) return learn_tests(argc, argv);
    if (argc > 1 && !strcmp(argv[1], "sway")) return sway_tests(argc, argv);
    if (getenv("DT")) DTS = atof(getenv("DT"));  // frame time of the law calls (default 1/60 s)
    const cfg all2[] = { { "mode 2", 2 } };  // mode 1 (0.4.14) removed 11-10
    // name=value arguments override mode-2 ini parameters (for tuning): run_lane.sh bend_assist=2.9 heading_gain=12
    { int k = 0; for (int i = 1; i < argc; i++) { char *e = strchr(argv[i], '='); if (e && novr < 32) { *e = 0; ovr[novr].k = argv[i]; ovr[novr++].v = atof(e + 1); } else argv[k++ + 1] = argv[i]; } argc = k + 1; }
    if (argc > 1 && !strcmp(argv[1], "follow")) return follow_tests();
    if (argc > 1 && !strcmp(argv[1], "lat")) return lat_tests();
    if (argc > 1 && !strcmp(argv[1], "sbend")) return sbend_tests();
    if (argc > 2 && !strcmp(argv[1], "replay")) {  // open loop: recorded lane.csv inputs through the law. replay <lane.csv> [t0 t1] [mode]
        FILE *f = fopen(argv[2], "r"); if (!f) { printf("cannot open %s\n", argv[2]); return 1; }
        double t0 = argc > 3 ? atof(argv[3]) : -1e9, t1 = argc > 4 ? atof(argv[4]) : 1e9; int mode = argc > 5 ? atoi(argv[5]) : 2;
        const cfg c2 = { "r", mode }; apply(c2);
        char line[512]; fgets(line, sizeof line, f);
        double t, kmh, o1, o2, o3, ug, uo, A, st, w, pt = -1; int n = 0, big = 0, flip = 0, weak = 0, outside = 0, outwrong = 0, gamewrong = 0; double maxopp = 0;
        printf("   t   kmh    o1     o2     o3   u_game  u_logged  u_new   A      (rows where |u_game|>0.25 or |o1|>0.8)\n");
        while (fgets(line, sizeof line, f)) {
            if (sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &t, &kmh, &o1, &o2, &o3, &ug, &uo, &A, &st, &w) < 10) continue;
            if (t < t0 || t > t1) continue;
            double dt = pt > 0 && t - pt < 0.3 ? t - pt : 0.05; if (pt < 0 || t - pt >= 0.3) { lane_reset(); g_u = 0; g_o1_prev = o1; }
            pt = t; t_now = t; vkmh = kmh;
            static double ag_r = getenv("REPLAY_AG") ? atof(getenv("REPLAY_AG")) : 1.0;   // REPLAY_AG: A in the log is raw; divide by the game's actuator gain
            float out = lane_step((float)ug, (float)o1, (float)o2, (float)o3, (float)dt, (float)w, (float)(A / ag_r), (float)kmh); g_u = out; n++;
            g_rp_act += g_act; g_rp_n++; g_rp_hi += g_act > 0.3f;
            if (fabs(ug) > 0.3) { big++; if (out * ug <= 0) flip++; else if (fabs(out) < 0.5 * fabs(ug) - 1e-6) weak++; }
            if (fabs(o1) > 0.8) { outside++; if (out * o1 > 0) { outwrong++; if (getenv("SHOWWRONG")) printf("WRONG t %.2f o1 %.2f o2 %.3f o3 %.4f u_game %.3f u_new %.3f A %.4f\\n", t, o1, o2, o3, ug, out, A); } if (ug * o1 > 0) gamewrong++; }
            if (!getenv("QUIET") && (fabs(ug) > 0.25 || fabs(o1) > 0.8)) printf("%7.2f %4.0f %6.2f %6.3f %6.4f %7.3f %7.3f %7.3f %7.4f\n", t, kmh, o1, o2, o3, ug, uo, out, A);
        }
        if (p_learn > 0 || p_sway > 0) printf("learner after replay: gA %.2f delay %.2f c %.3f (R2 %.2f/%.2f) | s %.3f dfac %.3f cap %.2f | updates %d rolled %d blocks ok %d bad %d | sway events %d (fac %.2f)\n", L_gA, L_dl, L_c, L_r2a, L_r2c, L_cur.s, L_cur.dfac, L_cur.cap, L_nupd, L_rolled, L_blocks_ok, L_blocks_bad, SW_events, SW_fac);
        if (g_rp_n) printf("driver activity over %d rows: mean %.3f, share above 0.3: %.1f%% (g_wg %.2f g_wi %.2f at the end)\n", g_rp_n, g_rp_act / g_rp_n, 100.0 * g_rp_hi / g_rp_n, g_wg, g_wi);
        printf("rows %d; |u_game|>0.3: %d, of which our output opposite/zero %d, weaker than half %d; |o1|>0.8 m: %d rows, output pushing outwards %d (the game itself: %d)\n", n, big, flip, weak, outside, outwrong, gamewrong);
        return 0;
    }
    if (argc > 4 && !strcmp(argv[1], "road")) {  // closed loop on a real road: o3(t) and speed(t) from a recorded lane.csv segment. road <csv> t0 t1
        FILE *f = fopen(argv[2], "r"); if (!f) return 1;
        double t0 = atof(argv[3]), t1 = atof(argv[4]); static double T_[20000], V_[20000], O3_[20000]; int n = 0;
        char line[512]; fgets(line, sizeof line, f); double t, kmh, o1, o2, o3;
        while (fgets(line, sizeof line, f) && n < 20000) if (sscanf(line, "%lf,%lf,%lf,%lf,%lf", &t, &kmh, &o1, &o2, &o3) == 5 && t >= t0 && t <= t1) { T_[n] = t; V_[n] = kmh; O3_[n] = o3; n++; }
        if (n < 20) { printf("no data\n"); return 1; }
        printf("road %s %.0f-%.0f s (%d samples): o3 range %.4f..%.4f, %0.f-%.0f km/h\n", argv[2], t0, t1, n, *std::min_element(O3_, O3_ + n), *std::max_element(O3_, O3_ + n), *std::min_element(V_, V_ + n), *std::max_element(V_, V_ + n));
        for (int drv : { 0, 1 }) for (auto &c : all2) {
            apply(c); pl p = {0, 0, 0, 0, 0}; pp m; double sum = 0, mx = 0; int cnt = 0, zc = 0, hold = 0; float dt = DTS; rng = 7; int k = 0;
            for (double tt = T_[0]; tt < T_[n - 1]; tt += dt) {
                while (k < n - 2 && T_[k + 1] < tt) k++;
                double a = (tt - T_[k]) / fmax(T_[k + 1] - T_[k], 1e-6); a = a < 0 ? 0 : (a > 1 ? 1 : a);
                vkmh = (float)(V_[k] + a * (V_[k + 1] - V_[k])); float o3v = (float)(O3_[k] + a * (O3_[k + 1] - O3_[k])); t_now = tt;
                step(p, m, dt, o3v, drv, 0, 0.0004f * gauss());
                if (tt - T_[0] > 3) { sum += p.o1 * p.o1; cnt++; mx = fmax(mx, fabs(p.o1)); if (fabs(p.o1) > 0.1) { int sg = p.o1 > 0 ? 1 : -1; if (hold && sg != hold) zc++; hold = sg; } }
            }
            printf("  %s %-14s rms %.2f max|o1| %.2f  zc/min %.1f\n", drv ? "hands on (driver follows the bend)" : "hands off                        ", c.name, sqrt(sum / cnt), mx, zc * 60.0 / (T_[n - 1] - T_[0] - 3));
        }
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "release")) {  // the driver steers a long bend himself (75% of it + tremor) and lets go halfway: max |o1| after letting go
        printf("bend o3 %s at 70 km/h, driver steers 75%% of it 12-30 s, then hands off: max |o1| while steering / in the 6 s after letting go / rms after\n", "0.004 / 0.008");
        struct pv { const char *n; float gA, db, dl, tw, c; };
        const pv pls[] = { { "nominal (gA 1, lag .3)", 1.0f, 0, 0, 0.3f, 0.19f }, { "ts4w-like (gA 1.5)", 1.5f, 0.0015f, 0.33f, 0.05f, 0.17f }, { "crash fit (gA 2.95)", 2.95f, 0.010f, 0.33f, 0.05f, 0.21f } };
        for (auto &q : pls) for (float ob : { 0.004f, 0.008f }) {
            const cfg c2 = { "r", 2 }; apply(c2);
            pp m; m.gA = q.gA; m.db = q.db; m.delay = q.dl; m.tw = q.tw; m.c = q.c; m.d = q.c * BEND_NEED;
            via_stub = 1; memset(GA, 0, sizeof GA); GA[0x1d1] = 0x20; rng = 5; pl p = { 0, 0, 0, 0, 0 }; float dt = DTS; vkmh = 70;
            double m1 = 0, m2 = 0, s2 = 0; int n2 = 0;
            for (int i = 0; i < (int)(45 / dt); i++) {
                t_now = i * dt; float k = clampf((float)(t_now - 5) / 3, 0, 1); float o3 = ob * k * k * (3 - 2 * k);
                int hands = t_now > 12 && t_now < 30;
                step(p, m, dt, o3, hands ? 1 : 0, 0, 0.0004f * gauss());
                if (hands && t_now > 15) m1 = fmax(m1, fabs(p.o1));
                if (t_now >= 30 && t_now < 36) m2 = fmax(m2, fabs(p.o1));
                if (t_now >= 36) { s2 += p.o1 * p.o1; n2++; }
                static int ti = 0; if (getenv("TRACE") && atoi(getenv("TRACE")) == ti && fmod(t_now, 0.5) < dt) printf("    %5.1f o1 %6.2f o2 %6.3f u %5.2f A %7.4f w %7.4f need %7.4f act %.2f wg %.2f wi %.2f ff %.4f sw %.2f ls %.2f\n", t_now, p.o1, p.o2, *(float *)(GA + GA_U), p.A, p.w, -BEND_NEED * o3, g_act, g_wg, g_wi, g_ff, SW_fac, L_cur.s);
                if (i == (int)(45 / dt) - 1) ti++;
            }
            via_stub = 0;
            printf("  %-24s o3 %.3f: steering %.2f m | after letting go %.2f m | rms after %.2f\n", q.n, ob, m1, m2, sqrt(s2 / n2));
        }
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "ats")) {  // 02-11: the ATS 1.61 actuator (A 0.15x per request, wheel ~11x per A, c 0.55x) through stub_law: the gain estimator against none, and the profile scale s
        printf("Actuator gain: stub_law on a 1.61 layout, hands off, wandering road 70 s / bend o3 .0035. rms o1 / max|o1| / zc per min | bend max / steady offset | runaways | g_ag at the end\n");
        struct pv { const char *n; float ag, gA, c; };
        const pv pls[] = { { "ETS2 plant (ag 1, gA 1.0, c .19)", 1.0f, 1.0f, 0.19f }, { "ATS plant (ag .15, gA 22, c .095)", 0.15f, 22.0f, 0.095f }, { "ATS, ag .15, gA 15, c .12", 0.15f, 15.0f, 0.12f }, { "ATS, ag .25, gA 12, c .095", 0.25f, 12.0f, 0.095f }, { "ATS, ag .10, gA 30, c .095", 0.10f, 30.0f, 0.095f } };
        struct var { const char *n; int est; float s; };
        const var vars[] = { { "no estimator, s 1  ", 0, 1.0f }, { "estimator, s 1     ", 1, 1.0f }, { "estimator, s 0.85  ", 1, 0.85f }, { "estimator, s 0.7   ", 1, 0.7f }, { "estimator, s 0.55  ", 1, 0.55f }, { "estimator, s 0.45  ", 1, 0.45f }, { "estimator, s 1.15  ", 1, 1.15f }, { "estimator, s 1.3   ", 1, 1.3f }, { "estimator, s 1.5   ", 1, 1.5f } };
        for (auto &q : pls) for (auto &vr : vars) for (float kmh : { 60.0f, 80.0f }) {
            if (q.ag == 1.0f && (vr.s < 0.85f || getenv("ATS_ONLY"))) continue;
            if (getenv("ATS_ONLY") && !vr.est) continue;
            pp m; m.ag = q.ag; m.gA = q.gA; m.c = q.c; m.d = q.c * BEND_NEED;
            const cfg c2 = { "a", 2 };
            double rs[2] = {}, mx[2] = {}, zc[2] = {}, bs = 0, bm = 0; int inc0 = g_n_incidents; float agend = 0;
            use_layout(1);
            for (int scen = 0; scen < 2; scen++) {
                apply(c2); p_learn = 1; L_cur = L_base = { vr.s, 1, p_l_max }; L_loaded = 1;   // (apply() zeroes the actuator-gain state)
                via_stub = 1; memset(GA, 0, sizeof GA); GA[0x1d1] = 0x20; g_law_at = -1e9;
                vkmh = kmh; rng = 4242; pl p = { 0.5, 0, 0, 0, 0 }; float dt = DTS, os = 0; double sum = 0, sb = 0; int n = 0, nb = 0, hold = 0, z = 0; double mxx = 0, bmx = 0;
                for (int i = 0; i < (int)(70 / dt); i++) {
                    t_now = 2000 + i * dt;
                    if (!vr.est) AG_w = -1e9;   // no estimator: A is read as it is (what the build did before)
                    float o3; if (scen == 0) o3 = ou_o3(os, dt, 0.0012f); else { float k = clampf((float)(t_now - 2010) / 3, 0, 1) - clampf((float)(t_now - 2045) / 3, 0, 1); o3 = 0.0035f * k * k * (3 - 2 * k); }
                    step(p, m, dt, o3, 0, 0, 0.0006f * gauss());
                    if (t_now - 2000 > 8) { sum += p.o1 * p.o1; n++; mxx = fmax(mxx, fabs(p.o1)); if (fabs(p.o1) > 0.1) { int sg = p.o1 > 0 ? 1 : -1; if (hold && sg != hold) z++; hold = sg; } }
                    if (scen == 1 && t_now - 2000 > 10 && t_now - 2000 < 50) { bmx = fmax(bmx, fabs(p.o1)); if (t_now - 2000 > 38 && t_now - 2000 < 45) { sb += p.o1; nb++; } }
                }
                rs[scen] = sqrt(sum / n); mx[scen] = mxx; zc[scen] = z * 60.0 / 62; if (scen == 1) { bs = nb ? sb / nb : 0; bm = bmx; } agend = g_ag;
                via_stub = 0;
            }
            use_layout(0);
            printf("  %-34s %s %2.0f km/h | wander %.2f %.2f %.1f | bend %.2f %.2f | runaways %d | g_ag %.2f\n", q.n, vr.n, kmh, rs[0], mx[0], zc[0], bm, bs, g_n_incidents - inc0, agend);
        }
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "engage")) {  // lane keeping switched on with the truck off-center and drifting out (11-10: -0.9 m, heading out)
        printf("switched on at o1 %s, hands off, through stub_law: peak wheel speed (rad/s) in the first 2 s / max |o1| / overshoot / s until |o1| < 0.2 for good\n", "-0.9 m drifting out / 0.4 m / 0");
        struct pv { const char *n; float gA, db, dl, tw, c; };
        const pv pls[] = { { "ts4w-like (gA 1.5)", 1.5f, 0.0015f, 0.33f, 0.05f, 0.17f }, { "ts5b avg (gA 2.5)", 2.5f, 0.007f, 0.3f, 0.05f, 0.19f } };
        for (auto &q : pls) for (float o10 : { -0.9f, 0.4f, 0.0f }) for (float soft : { 0.0f, 0.35f }) {
            const cfg c2 = { "e", 2 }; apply(c2); p_soft = soft; g_law_at = -1e9;
            pp m; m.gA = q.gA; m.db = q.db; m.delay = q.dl; m.tw = q.tw; m.c = q.c; m.d = q.c * BEND_NEED;
            via_stub = 1; memset(GA, 0, sizeof GA); GA[0x1d1] = 0x20; rng = 3; pl p = { o10, o10 < -0.5f ? 0.05 : 0.0, 0, 0, 0 }; float dt = DTS; vkmh = 44;
            double pk = 0, mx = 0, os = 0, settle = 0, wprev = 0;
            for (int i = 0; i < (int)(30 / dt); i++) {
                t_now = 1000 + i * dt;
                step(p, m, dt, 0, 0, 0, 0.0004f * gauss());
                double wr = fabs(p.w - wprev) / dt; wprev = p.w; if (i * dt < 2 && i > 0) pk = fmax(pk, wr);
                mx = fmax(mx, fabs(p.o1)); if (o10 != 0 && p.o1 * o10 < 0) os = fmax(os, fabs(p.o1)); if (fabs(p.o1) > 0.2) settle = i * dt;
            }
            via_stub = 0;
            printf("  %-20s start %5.2f soft %.2f: peak wheel speed %.3f | max |o1| %.2f | overshoot %.2f | settled %4.1f s\n", q.n, o10, soft, pk, mx, os, settle);
        }
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "gsweep")) {  // straight wander, hands off, wheel gain G 1..3.5 (delay .33, band from the ts5b fit scaled): rms / max / zc per min
        float kmh = argc > 2 ? atof(argv[2]) : 73; float s = argc > 3 ? atof(argv[3]) : 0.955f, df = argc > 4 ? atof(argv[4]) : 1.0f;
        printf("straight wander %.0f km/h, profile s %.3f dfac %.3f, hands off, 90 s from 0.5 m: rms / max / zc per min (last 60 s)\n", kmh, s, df);
        for (float st : { 0.0f, 0.004f }) for (float G : { 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f }) {
            const cfg c2 = { "g", 2 }; apply(c2); p_learn = 1; p_sway = 1; L_cur = L_base = { s, df, 2 }; L_hold = 1000;
            pp m; m.gA = G; m.db = G > 1.2f ? 0.0034f * (G - 1.0f) * 1.5f : 0; m.delay = 0.33f; m.tw = 0.05f; m.c = 0.19f; m.d = 0.19f * BEND_NEED; m.kk = 0.9f; m.stick = st;
            via_stub = 1; memset(GA, 0, sizeof GA); GA[0x1d1] = 0x20; rng = 11; pl p = { 0.5, 0, 0, 0, 0 }; float dt = DTS, os = 0; vkmh = kmh;
            double sum = 0, mx = 0; int nn = 0, zc = 0, hold = 0;
            for (int i = 0; i < (int)(90 / dt); i++) {
                t_now = i * dt; float o3 = ou_o3(os, dt, 0.0012f);
                step(p, m, dt, o3, 0, 0, 0.0006f * gauss());
                if (t_now > 30) { sum += p.o1 * p.o1; nn++; mx = fmax(mx, fabs(p.o1)); if (fabs(p.o1) > 0.1) { int sg = p.o1 > 0 ? 1 : -1; if (hold && sg != hold) zc++; hold = sg; } }
            }
            via_stub = 0;
            printf("  stiction %.3f G %.1f: rms %.2f max %.2f zc/min %4.1f  (wi %.2f wg %.2f, sway ev %d)\n", st, G, sqrt(sum / nn), mx, zc * 1.0, g_wi, g_wg, SW_events);
        }
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "crash")) {  // ts5b 383-391 s: the sharp bend at 60-63 km/h, closed loop on the recorded o3/speed. crash [csv t0 t1]
        const char *fn = argc > 2 ? argv[2] : "runs/ts5b/lane.csv"; double t0 = argc > 3 ? atof(argv[3]) : 368, t1 = argc > 4 ? atof(argv[4]) : 388.2;
        FILE *f = fopen(fn, "r"); if (!f) { printf("cannot open %s\n", fn); return 1; }
        static double T_[20000], V_[20000], O3_[20000]; int n = 0; char line[512]; fgets(line, sizeof line, f); double t, kmh, o1, o2, o3;
        while (fgets(line, sizeof line, f) && n < 20000) if (sscanf(line, "%lf,%lf,%lf,%lf,%lf", &t, &kmh, &o1, &o2, &o3) == 5 && t >= t0 && t <= t1) { T_[n] = t; V_[n] = kmh; O3_[n] = o3; n++; }
        fclose(f);
        if (n < 20) { printf("no data\n"); return 1; }
        // plants: the crash fit (hands off, 383-387 s: wheel = 2.95 A(t-0.33) - 0.010 sign, c 0.21, kk 0.92), the ts5b average, ts4w-like, the old nominal
        struct pv { const char *n; float gA, db, dl, tw, c, kk, st; int lim; };
        const pv pls[] = { { "crash fit (gA 2.95, band .010)", 2.95f, 0.010f, 0.33f, 0.05f, 0.21f, 0.92f, 0 }, { "crash fit + stiction .004", 2.95f, 0.006f, 0.33f, 0.05f, 0.21f, 0.92f, 0.004f },
                           { "ts5b avg (gA 2.5, band .007)", 2.5f, 0.007f, 0.3f, 0.05f, 0.19f, 0.9f, 0 }, { "gA 2.0, band .004*", 2.0f, 0.004f, 0.33f, 0.05f, 0.19f, 0.9f, 0, 1 },
                           { "ts4w-like (gA 1.5, band .0015)*", 1.5f, 0.0015f, 0.33f, 0.05f, 0.17f, 0.88f, 0, 1 }, { "old nominal (gA 1, lag .3)*", 1.0f, 0.0f, 0.0f, 0.3f, 0.19f, 0.88f, 0, 1 } };
        struct pf { const char *n; float s, df, gA, dl, c; };
        const pf pfs[] = { { "ts5b profile", 0.707f, 0.748f, 0.706f, 0.796f, 0.1815f }, { "ts4w profile", 0.955f, 1.0f, 1.144f, 0.413f, 0.1613f }, { "ts5c profile", 0.749f, 0.970f, 1.503f, 0.456f, 0.1701f } };
        int tr = getenv("TRACE") ? atoi(getenv("TRACE")) : -1, k_i = 0, bad = 0;
        printf("crash bend %s %.0f-%.1f s: o3 %.4f..%.4f, %.0f-%.0f km/h; hands off, through stub_law, learn + anti-sway on\n", fn, t0, t1, *std::min_element(O3_, O3_ + n), *std::max_element(O3_, O3_ + n), *std::min_element(V_, V_ + n), *std::max_element(V_, V_ + n));
        printf("%-32s %-13s | max|o1| all/383+ | |u|=1 s | reversals | wheel < 50%% need s | wheel > 160%% need s | sway ev\n", "plant", "profile");
        for (auto &q : pls) for (auto &pr : pfs) {
            const cfg c2 = { "crash", 2 }; apply(c2); p_learn = 1; p_sway = 1;
            L_cur = L_base = { pr.s, pr.df, 2 }; L_gA = pr.gA; L_dl = pr.dl; L_c = pr.c;
            pp m; m.gA = q.gA; m.db = q.db; m.delay = q.dl; m.tw = q.tw; m.c = q.c; m.kk = q.kk; m.d = q.c * BEND_NEED; m.stick = q.st;
            via_stub = 1; memset(GA, 0, sizeof GA); GA[0x1d1] = 0x20; rng = 7; pl p = { 0, 0, 0, 0, 0 }; float dt = DTS; int k = 0;  // lane keeping on (key)
            double mx = 0, mxb = 0, sat = 0, under = 0, over = 0; int rev = 0, lastsg = 0; double last_tr = -1;
            for (double tt = T_[0]; tt < T_[n - 1]; tt += dt) {
                while (k < n - 2 && T_[k + 1] < tt) k++;
                double a = (tt - T_[k]) / fmax(T_[k + 1] - T_[k], 1e-6); a = a < 0 ? 0 : (a > 1 ? 1 : a);
                vkmh = (float)(V_[k] + a * (V_[k + 1] - V_[k])); float o3v = (float)(O3_[k] + a * (O3_[k + 1] - O3_[k])); t_now = tt;
                step(p, m, dt, o3v, 0, 0, 0.0004f * gauss());
                float u = *(float *)(GA + GA_U);
                if (tt - T_[0] > 3) {
                    mx = fmax(mx, fabs(p.o1)); if (tt >= 383) mxb = fmax(mxb, fabs(p.o1));
                    if (fabs(u) > 0.99f) sat += dt;
                    int sg = u > 0.3f ? 1 : (u < -0.3f ? -1 : 0); if (sg && lastsg && sg != lastsg) rev++; if (sg) lastsg = sg;
                    double need = -BEND_NEED * o3v;
                    if (fabs(o3v) > 0.008) { if (p.w * need < 0.5 * need * need) under += dt; if (p.w * need > 1.6 * need * need) over += dt; }
                }
                if (tr == k_i && tt - last_tr >= 0.1) { last_tr = tt; printf("  %7.2f %4.1f o1 %6.2f o2 %6.3f o3 %.4f u %6.2f A %7.4f wheel %7.4f need %7.4f act %.2f wg %.2f wi %.2f ff %.4f\n", tt, vkmh, p.o1, p.o2, o3v, u, p.A, p.w, -BEND_NEED * o3v, g_act, g_wg, g_wi, g_ff); }
            }
            via_stub = 0;
            if (mx > 0.9 && !q.lim) bad++;
            printf("%-32s %-13s | %5.2f %5.2f m | %5.2f   | %3d       | %5.2f              | %5.2f               | %d\n", q.n, pr.n, mx, mxb, sat, rev, under, over, SW_events);
            k_i++;
        }
        printf("crash test: %s (max |o1| must stay under 0.9 m; * = the bend needs more than the assist can give: u = 1 gives A ~0.047)\n", bad ? "FAILED" : "ok");
        return bad;
    }
    if (argc > 2 && !strcmp(argv[1], "variants")) {
        float kmh = atof(argv[2]); const cfg nw2 = { "new", argc > 3 ? atoi(argv[3]) : 2 };
        for (float cm : { 0.6f, 1.0f, 1.5f }) for (float tw : { 0.2f, 0.3f, 0.45f }) for (float gA : { 0.7f, 1.0f }) {
            pp m; m.c *= cm; m.tw = tw; m.gA = gA; res r = run(nw2, m, 0, 0, kmh), b = run(nw2, m, 1, 0, kmh);
            printf("c x%.1f tau %.2f gA %.1f | straight rms %.2f max %.2f zc/min %.1f | bend max %.2f ss %.2f\n", cm, tw, gA, r.rms, r.maxe, r.zc * 60.0 / 62, b.bendmax, b.bendss);
        }
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "step")) {  // 0.5 m offset, hands off, nothing else: overshoot and settling
        const cfg c2 = { "x", argc > 2 ? atoi(argv[2]) : 2 };
        printf("step from o1 = 0.5 m (mode %d): overshoot (m, opposite side) / time until |o1| < 0.05 for good (s) / time to first crossing\n", c2.mode);
        quiet = 1;
        for (float kmh : { 40.f, 60.f, 80.f, 90.f }) {
            printf("  %2.0f km/h:", kmh);
            for (auto vv : { std::pair<float, float>(1.0f, 0.3f), std::pair<float, float>(1.5f, 0.45f), std::pair<float, float>(0.6f, 0.2f), std::pair<float, float>(1.5f, 0.2f), std::pair<float, float>(0.6f, 0.45f) }) {
                pp m; m.c *= vv.first; m.tw = vv.second; apply(c2); vkmh = kmh; pl p = {0.5, 0, 0, 0, 0};
                double os = 0, settle = 0, cross = -1; float dt = DTS;
                for (int i = 0; i < (int)(60 / dt); i++) { t_now = i * dt; step(p, m, dt, 0, 0, 0, 0); if (p.o1 < 0) { os = fmax(os, -p.o1); if (cross < 0) cross = t_now; } if (fabs(p.o1) > 0.05) settle = t_now; }
                printf("  [c x%.1f tau %.2f] %.2f/%4.1f/%4.1f", vv.first, vv.second, os, settle, cross);
            }
            printf("\n");
        }
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "low")) {  // lane keeping from ~5 km/h (step from 0.5 m, wander, robustness)
        const cfg c2 = { "mode 2", 2 };
        printf("low speed (mode 2): step 0.5 m hands off -> overshoot / settle (s) ; straight wander rms / max / zc per min ; worst over the plant variants\n");
        for (float kmh : { 5.f, 10.f, 15.f, 20.f, 30.f, 40.f }) {
            quiet = 1; pp m0; apply(c2); vkmh = kmh; pl p = {0.5, 0, 0, 0, 0}; double os = 0, settle = 0; float dt = DTS;
            for (int i = 0; i < (int)(60 / dt); i++) { t_now = i * dt; step(p, m0, dt, 0, 0, 0, 0); if (p.o1 < 0) os = fmax(os, -p.o1); if (fabs(p.o1) > 0.05) settle = t_now; }
            quiet = 0; res r = run(c2, m0, 0, 0, kmh);
            double wr = 0, wm = 0; int bad = 0;
            for (float cm : { 0.6f, 1.0f, 1.5f }) for (float tw : { 0.2f, 0.3f, 0.45f }) for (float gA : { 0.7f, 1.0f }) {
                pp m; m.c *= cm; m.tw = tw; m.gA = gA; res q = run(c2, m, 0, 0, kmh); wr = fmax(wr, q.rms); wm = fmax(wm, q.maxe); if (q.rms > 0.8) bad++;
            }
            printf("  %2.0f km/h: step os %.2f m settle %4.1f s | wander rms %.2f max %.2f zc/min %4.1f | worst rms %.2f max %.2f diverged %d/18\n",
                   kmh, os, settle, r.rms, r.maxe, r.zc * 60.0 / 62, wr, wm, bad);
        }
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "stress")) {  // how far can the plant be off? c (steering gain) up, wheel lag up: ring or diverge?
        const cfg c2 = { "x", argc > 2 ? atoi(argv[2]) : 2 };
        quiet = 1;
        printf("stress (mode %d), step from 0.5 m at 90 km/h, hands off: peak |o1| after t = 15 s (0 = settled; > 0.5 = growing oscillation)\n", c2.mode);
        for (float tw : { 0.3f, 0.6f, 0.9f, 1.2f }) {
            printf("  tau %.1f:", tw);
            for (float cm : { 1.0f, 2.0f, 3.0f, 4.0f, 6.0f }) {
                pp m; m.c *= cm; m.tw = tw; apply(c2); vkmh = 90; pl p = {0.5, 0, 0, 0, 0}; double pk = 0; float dt = DTS;
                for (int i = 0; i < (int)(60 / dt); i++) { t_now = i * dt; step(p, m, dt, 0, 0, 0, 0); if (t_now > 15) pk = fmax(pk, fabs(p.o1)); }
                printf("  c x%.0f %6.2f", cm, pk);
            }
            printf("\n");
        }
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "score")) {  // one line per parameter set (tuning)
        const cfg nw2 = { "new", 2 };
        double sr = 0, sz = 0, bs = 0, bm = 0, hs = 0, hm = 0, wr = 0, wz = 0, wbm = 0; int n = 0;
        for (float kmh : { 40.f, 60.f, 80.f, 90.f }) {
            pp nomp; res r = run(nw2, nomp, 0, 0, kmh), b = run(nw2, nomp, 1, 0, kmh), h = run(nw2, nomp, 1, 1, kmh);
            sr += r.rms; sz += r.zc * 60.0 / 62; bs += fabs(b.bendss); bm = fmax(bm, b.bendmax); hs = fmax(hs, fabs(h.bendss)); hm = fmax(hm, h.bendmax); n++;
            for (float cm : { 0.7f, 1.0f, 1.5f }) for (float tw : { 0.2f, 0.45f }) for (float gA : { 0.8f, 1.0f }) {
                pp m; m.c *= cm; m.tw = tw; m.gA = gA; res q = run(nw2, m, 0, 0, kmh), bb = run(nw2, m, 1, 0, kmh);
                wr = fmax(wr, q.rms); wz = fmax(wz, q.zc * 60.0 / 62); wbm = fmax(wbm, bb.bendmax);
            }
        }
        printf("straight rms %.2f zc/min %.1f | worst-var rms %.2f zc %.1f bendmax %.2f | bend off: ss %.2f max %.2f | bend on: ss(max) %.2f max %.2f\n", sr / n, sz / n, wr, wz, wbm, bs / n, bm, hs, hm);
        return 0;
    }
    int sweep = argc > 1 ? atoi(argv[1]) : 1;
    const cfg nw = { "mode 2", 2 };
    const cfg all[] = { nw };  // mode 1 (0.4.14) removed 11-10
    pp nom;
    printf("Plant: fitted on real rides (c %.2f gA %.1f tau %.2f kk %.2f d %.2f); hands-off unless noted; start offset 0.5 m\n", nom.c, nom.gA, nom.tw, nom.kk, nom.d);
    printf("%-14s %-34s %6s %6s %7s %6s %8s\n", "law", "scenario (nominal plant)", "rms o1", "max|e|", "zc/min", "T(s)", "bend max/ss");
    struct sc { const char *n; int s, d; float kmh; };
    const sc scs[] = { {"straight wander 40 km/h", 0, 0, 40}, {"straight wander 60 km/h", 0, 0, 60}, {"straight wander 80 km/h", 0, 0, 80}, {"straight wander 90 km/h", 0, 0, 90},
                       {"straight wander 80, hands on", 0, 1, 80}, {"bend o3 .0035 60 km/h hands off", 1, 0, 60}, {"bend o3 .0035 80 km/h hands off", 1, 0, 80}, {"bend o3 .0035 80 km/h hands on", 1, 1, 80} };
    for (auto &s : scs)
        for (auto &c : all) {
            res r = run(c, nom, s.s, s.d, s.kmh, 70, s.s ? 0.0f : 0.5f);
            printf("%-14s %-34s %6.2f %6.2f %7.1f %6.1f %4.2f/%5.2f  Apk %.3f\n", c.name, s.n, r.rms, r.maxe, r.zc * 60.0 / 62, r.per, r.bendmax, r.bendss, r.Apk);
        }
    if (sweep) {
        printf("\nRobustness: worst case over c x(0.6, 1, 1.5), tau (0.2, 0.3, 0.45) s, gA (0.7, 1.0)\n");
        for (auto &c : all)
            for (float kmh : { 40.f, 60.f, 80.f, 90.f }) {
                double wr = 0, wm = 0, wz = 0, wb = 0; int unstable = 0, n = 0;
                for (float cm : { 0.6f, 1.0f, 1.5f }) for (float tw : { 0.2f, 0.3f, 0.45f }) for (float gA : { 0.7f, 1.0f }) {
                    pp m; m.c *= cm; m.tw = tw; m.gA = gA;
                    res r = run(c, m, 0, 0, kmh); res b = run(c, m, 1, 0, kmh);
                    wr = fmax(wr, r.rms); wm = fmax(wm, fmax(r.maxe, b.maxe)); wz = fmax(wz, r.zc * 60.0 / 62); wb = fmax(wb, fabs(b.bendss));
                    if (r.rms > 0.8 || b.maxe > 2) unstable++; n++;
                }
                printf("  %-14s %2.0f km/h  worst rms %.2f  worst max|e| %.2f  worst zc/min %4.1f  worst bend steady offset %.2f  diverged %d/%d\n", c.name, kmh, wr, wm, wz, wb, unstable, n);
            }
    }
    printf("\nDriver evasive 1.2 m (hands on 8 s), then hands off:\n");
    for (auto &c : all)
        for (float kmh : { 60.f, 80.f }) {
            double pk, opp, os, late; run_driver(c, nom, kmh, &pk, &opp, &os, &late);
            printf("  %-14s %2.0f km/h  peak A %.4f  mean A opposing the driver %.4f  overshoot after letting go %.2f m  |o1| at 14-32 s after %.2f\n", c.name, kmh, pk, opp, os, late);
        }
    int rc = follow_edge_test() | test_indicator() | test_indicator_key() | test_ffb() | test_saturation() | test_lane_state() | test_cruise_lane_on();
    rc |= test_key(); rc |= test_cal_fit(); rc |= test_cal_aborts(); rc |= test_cal_waits(); rc |= test_cal_sway(); rc |= test_ag_no_cruise(); rc |= test_game_own(); rc |= test_cc_key(); rc |= test_low_speed(); rc |= test_low_speed_runaway();   // in this order: they share the module state
    rc |= runaway_tests();   // review 30-10: the one-sided runaway scenario (guard)
    rc |= recovery_tests();  // 30-10: a recovery toward the center must not fire the guard
    rc |= sway_persist_tests();  // 30-10: a sway lowers the learned scale for the rest of the session
    rc |= cal_learning_tests();  // 30-10: learning only while a calibration is on, confidence stop, passive bends
    rc |= bend_off_test();       // review: bend_max_lat 0 (off) must not kill the learner/calibration
    rc |= cal_convergence_test();  // review: the confidence stop must wait for the profile to converge
    rc |= test_ats();              // 02-11: the 1.61 layout, the weaker A, the profile key, a scale above learn_max
    rc |= lat_tests();             // WP-LA5: roundabouts and fast bends from the live lateral load
    return rc;
}
