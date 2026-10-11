// Offline test of the Lane module (modules/lane.cpp is included, so lane_step and filter_a are
// the real ones). Plant = the model FITTED ON REAL RIDES (docs/lka-sysid.md), not an assumed wheel:
//   A'  = -(al-0.3) A + be u          (be, al speed dependent; filter_a adds the other 0.3/s of leak)
//   w'  = (-w + gA A + w_driver)/tau  (FFB wheel follows the assist offset, plus whatever the hands do)
//   o2' = -c v w - d v o3 + wind      (heading error; o3 = lane curvature term, w_bend = -(d/c) o3 = -4.4 o3)
//   o1' = -kk v o2                    (lateral offset, sign as logged)
// nominal c 0.19, gA 1.0, tau 0.3, kk 0.88, d 0.85; the sweeps vary c (0.6-1.5x) and tau (0.2-0.45 s).
// Hands-on = a driver who steers the bend himself (-3.3 o3, 75% of the need) with a hand tremor.
// Build+run: tools/sim/run_lane.sh
#include "core/suite.h"
#include "core/game.h"
#include <math.h>
#include <initializer_list>
#include <utility>
#include <algorithm>
#include <stdarg.h>
#include <string.h>
#include <string>

telemetry T;
char g_dir[MAX_PATH];
static layout g_layout = { "1.60", "eut2" };  // ln_start hooks only on the 1.60 layout
const layout *L = &g_layout;
volatile int g_rolling, g_cruise_smooth, g_crawl_on;
volatile float g_pcc_need_kmh = NAN;
int g_log_level = 2;
const char *g_truck_unsupported;
volatile float g_creep_brake;
volatile char g_dash_flash, g_dash_status;
double g_dash_flash_until;
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
void play_tones(const float *, int, int) {}
uintptr_t game_scan(const char *) { return 0; }
int game_patch_call(uintptr_t, void *) { return 0; }
float *game_patch_rip_float(uintptr_t, int, int, float) { return NULL; }
int game_patch_store_xmm8(uintptr_t, float (*)(float)) { return 0; }
static char GA[0x2000], VEH[0x1500], GB[64];
char *game_actor_safe() { return GA; }
void *game_gearbox() { return GB; }
char *gb_inner(void *) { return VEH; }
int mem_readable(const void *, size_t) { return 1; }

#include "modules/lane.cpp"

// ---- plant
struct pl { double o1, o2, w, A, tr; };
struct pp {
    float c = 0.19f, gA = 1.0f, tw = 0.3f, kk = 0.88f, d = 0.85f;
    pp() {  // plant override for what-if runs (ts4n: hands off c .155 gA 1.2 tau .45; in the 283 s ringing c .21 gA 2.5 tau .24)
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
static int via_stub;  // 1 = the request goes through stub_law (gating, filter_a timing) like in the game
static float vkmh = 80;
static float DTS = 1.0f / 60;
static int quiet;  // 1 = no noise at all (step response)
static unsigned rng = 12345;
static float noise() { rng = rng * 1664525u + 1013904223u; return ((rng >> 8) & 0xffff) / 65535.0f - 0.5f; }
static float gauss() { return (noise() + noise() + noise() + noise()) * 0.866f * 1.0f; }  // ~N(0,0.5)

struct cfg { const char *name; int mode; };
static struct { const char *k; float v; } ovr[32];
static int novr;
static void apply(const cfg &c)
{
    for (auto &p : g_params) *p.val = p.def;
    p_mode = c.mode;
    p_learn = getenv("LEARN") ? atof(getenv("LEARN")) : 0; p_sway = getenv("SWAY") ? atof(getenv("SWAY")) : 0;  // the old scenarios test the fixed law
    L_cur = L_base = { 1, 1, p_l_max }; L_trial = 0; L_hold = 0; L_t = 0; L_Nw = L_Syy = L_Cxx = L_Cxy = L_Cyy = 0; for (int k = 0; k < LK_NK; k++) L_Sxx[k] = L_Sxy[k] = 0;
    L_scn = L_scs = L_sc_zc = L_since = 0; L_b_valid = 0; L_ring_cool = 0; L_nupd = 0; SW_fac = 1; SW_t = 0; SW_hold_until = 0; SW_events = 0; SW_lastact = -1e9; L_loaded = 0; L_dirty = 0; learn_block_reset(); L_dev_blocks = 0; L_blocks_ok = L_blocks_bad = 0; L_rolled = 0;
    if (c.mode == 2) for (int i = 0; i < novr; i++) for (auto &p : g_params) if (!strcmp(p.key, ovr[i].k)) *p.val = ovr[i].v;
    p_min_kmh = 5; g_ffb = 1;  // the plant model is an FFB wheel
    lane_reset(); g_u = 0; g_o1_prev = 0; g_o2_prev = 0;
}

static float g_sim_uraw;
static void sim_game_law(void *ga, float, float, float, float) { *(float *)((char *)ga + GA_U) = g_sim_uraw; }
// one physics + control step. drv: 0 hands off, 1 driver steers the bend (+ tremor), 2 evasive (w_cmd)
static void step(pl &p, const pp &m, float dt, float o3, int drv, float w_cmd, float wind, float *u_game_out = NULL)
{
    float kmh = vkmh, v = kmh / 3.6f;
    T.speed = v;  // lane.cpp gates on the speed (keep_on: no request below min_kmh)
    float uraw = clampf(kmh * (0.075f * (float)p.o2 - 0.00275f * (float)p.o1 - 0.5f * o3), -1, 1);
    float before = *(float *)(GA + GA_U);
    *(float *)(GA + GA_U) = uraw != before ? uraw : uraw + 1e-6f;
    float u = *(float *)(GA + GA_U);
    *(float *)(VEH + V_STEER_IN) = (float)p.w;
    *(float *)(VEH + V_A) = (float)p.A;
    float o1m = (float)p.o1, o2m = (float)p.o2 + (quiet ? 0 : 0.0007f * gauss());  // lane sensor noise
    float out;
    if (via_stub) {  // the game's law writes clamp(u_raw) (it already did above), stub_law wraps it as in the game
        *(float *)(GA + GA_U) = before;
        o_law = sim_game_law; g_sim_uraw = uraw;
        stub_law(GA, o1m, o2m, o3, dt);
        out = *(float *)(GA + GA_U);
    } else {
        out = lane_step(u, o1m, o2m, o3, dt, (float)p.w, (float)p.A, kmh);
        *(float *)(GA + GA_U) = out;
        g_u = out;
    }
    if (u_game_out) *u_game_out = u;
    float be = 0.235f - 0.0010f * kmh, al = 2.6f + 0.016f * kmh - 0.3f;
    p.A += dt * (-al * p.A + be * out);
    p.A = filter_a((float)p.A);
    // driver: tremor is an OU process on the wheel
    if (!quiet) p.tr += dt * (-p.tr / 0.6f) + sqrtf(dt) * 0.0045f * gauss() * 1.6f;
    float wd = 0;
    if (drv == 1) wd = -3.3f * o3 + (float)p.tr;
    if (drv == 2) wd = w_cmd + (float)p.tr;
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
    p.o2 += dt * (-m.c * v * p.w - m.d * v * o3 + wind);
    p.o1 += dt * (-m.kk * v * p.o2);
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


// ---- learner / anti-sway tests (docs/lka-learn.md)
struct lres { double rms_all, rms_tail, zc_tail, maxe, usable; float s, df, cap, gA, dl, c; int upd, sw, rolled; double win[12]; double first_sw; };
// hands off on a wandering road for T s; drvmode 3 = the driver weaves (hand on the wheel) from T*0.45 for 120 s
static lres runL(const pp &m, int learn, int sway, float kmh, float T, int drvmode = 0, float start_o1 = 0.5f, int bends = 0)
{
    const cfg c2 = { "x", 2 }; apply(c2); p_learn = learn; p_sway = sway; vkmh = kmh; rng = 4242;
    pl p = {start_o1, 0, 0, 0, 0}; lres r = {}; r.first_sw = -1;
    float dt = DTS, os = 0; double sa = 0, st = 0, sT = 0; long na = 0, nt = 0, nu = 0, nn = 0; int hold = 0, zc = 0; int sw0 = 0;
    for (int i = 0; i < (int)(T / dt); i++) {
        t_now = i * dt;
        float o3 = ou_o3(os, dt, 0.0012f);
        if (bends) { float k = fmodf((float)t_now, 90.f); float e = clampf((k - 60) / 3, 0, 1) - clampf((k - 80) / 3, 0, 1); o3 += 0.0035f * e * e * (3 - 2 * e) * (((int)(t_now / 90)) % 2 ? -1 : 1); }
        int drv = 0; float wc = 0;
        if (drvmode == 3 && t_now > T * 0.45 && t_now < T * 0.45 + 120) { drv = 2; wc = 0.03f * sinf(6.2832f * (float)(t_now - T * 0.45) / 9.f) - 3.3f * o3; }
        step(p, m, dt, o3, drv, wc, 0.0006f * gauss());
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
    return r;
}

static int learn_tests(int argc, char **argv)
{
    float kmh = argc > 2 ? atof(argv[2]) : 90; float T = argc > 3 ? atof(argv[3]) : 900;
    struct pv { const char *n; float c, gA, tw, dl; };
    const pv pls[] = { { "reads like the real wheel", 0.18f, 1.0f, 0.05f, 0.3f }, { "ts4n large-signal fit (1.34, .2)", 0.155f, 1.34f, 0.05f, 0.2f }, { "real-like gA x0.6", 0.18f, 0.6f, 0.05f, 0.3f }, { "real-like gA x1.8", 0.18f, 1.8f, 0.05f, 0.3f }, { "gA x0.6", 0.155f, 0.80f, 0.05f, 0.2f }, { "gA x1.8", 0.155f, 2.41f, 0.05f, 0.2f },
                       { "gA x1.8 c x1.3", 0.20f, 2.41f, 0.05f, 0.2f }, { "slow wheel (delay .35, lag .15)", 0.155f, 1.34f, 0.15f, 0.35f }, { "fast wheel (delay .05)", 0.155f, 1.34f, 0.03f, 0.05f },
                       { "weak wheel gA x0.5 delay .3", 0.155f, 0.67f, 0.1f, 0.3f }, { "nominal sim plant", 0.19f, 1.0f, 0.3f, 0.0f } };
    printf("Learner on a wandering road, %.0f km/h, %.0f s, hands off; fixed gains vs learning (rms over the last 40%%: m, zc/min)\n", kmh, T);
    printf("%-34s | fixed: rms zc max | learn: rms zc max  s  dfac cap | est gA  delay  c  | usable updates sway rolled\n", "plant");
    for (auto &q : pls) {
        pp m; m.c = q.c; m.gA = q.gA; m.tw = q.tw; m.delay = q.dl;
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

// Indicator pause (indicator_pause = 1): a lane change to the left with the indicator; the game drops lane keeping halfway.
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

int main(int argc, char **argv)
{
    if (argc > 1 && !strcmp(argv[1], "learn")) return learn_tests(argc, argv);
    if (argc > 1 && !strcmp(argv[1], "sway")) return sway_tests(argc, argv);
    if (getenv("DT")) DTS = atof(getenv("DT"));  // frame time of the law calls (default 1/60 s)
    const cfg all2[] = { { "mode 2", 2 } };  // mode 1 (0.4.14) removed 11-10
    // name=value arguments override mode-2 ini parameters (for tuning): run_lane.sh bend_assist=2.9 heading_gain=12
    { int k = 0; for (int i = 1; i < argc; i++) { char *e = strchr(argv[i], '='); if (e && novr < 32) { *e = 0; ovr[novr].k = argv[i]; ovr[novr++].v = atof(e + 1); } else argv[k++ + 1] = argv[i]; } argc = k + 1; }
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
            float out = lane_step((float)ug, (float)o1, (float)o2, (float)o3, (float)dt, (float)w, (float)A, (float)kmh); g_u = out; n++;
            if (fabs(ug) > 0.3) { big++; if (out * ug <= 0) flip++; else if (fabs(out) < 0.5 * fabs(ug) - 1e-6) weak++; }
            if (fabs(o1) > 0.8) { outside++; if (out * o1 > 0) { outwrong++; if (getenv("SHOWWRONG")) printf("WRONG t %.2f o1 %.2f o2 %.3f o3 %.4f u_game %.3f u_new %.3f A %.4f\\n", t, o1, o2, o3, ug, out, A); } if (ug * o1 > 0) gamewrong++; }
            if (!getenv("QUIET") && (fabs(ug) > 0.25 || fabs(o1) > 0.8)) printf("%7.2f %4.0f %6.2f %6.3f %6.4f %7.3f %7.3f %7.3f %7.4f\n", t, kmh, o1, o2, o3, ug, uo, out, A);
        }
        if (p_learn > 0 || p_sway > 0) printf("learner after replay: gA %.2f delay %.2f c %.3f (R2 %.2f/%.2f) | s %.3f dfac %.3f cap %.2f | updates %d rolled %d blocks ok %d bad %d | sway events %d (fac %.2f)\n", L_gA, L_dl, L_c, L_r2a, L_r2c, L_cur.s, L_cur.dfac, L_cur.cap, L_nupd, L_rolled, L_blocks_ok, L_blocks_bad, SW_events, SW_fac);
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
            pp m; m.gA = q.gA; m.db = q.db; m.delay = q.dl; m.tw = q.tw; m.c = q.c; m.d = q.c * 4.6f;
            via_stub = 1; memset(GA, 0, sizeof GA); GA[0x1d1] = 0x20; rng = 5; pl p = { 0, 0, 0, 0, 0 }; float dt = DTS; vkmh = 70;
            double m1 = 0, m2 = 0, s2 = 0; int n2 = 0;
            for (int i = 0; i < (int)(45 / dt); i++) {
                t_now = i * dt; float k = clampf((float)(t_now - 5) / 3, 0, 1); float o3 = ob * k * k * (3 - 2 * k);
                int hands = t_now > 12 && t_now < 30;
                step(p, m, dt, o3, hands ? 1 : 0, 0, 0.0004f * gauss());
                if (hands && t_now > 15) m1 = fmax(m1, fabs(p.o1));
                if (t_now >= 30 && t_now < 36) m2 = fmax(m2, fabs(p.o1));
                if (t_now >= 36) { s2 += p.o1 * p.o1; n2++; }
                static int ti = 0; if (getenv("TRACE") && atoi(getenv("TRACE")) == ti && fmod(t_now, 0.5) < dt) printf("    %5.1f o1 %6.2f o2 %6.3f u %5.2f A %7.4f w %7.4f need %7.4f act %.2f wg %.2f wi %.2f ff %.4f sw %.2f ls %.2f\n", t_now, p.o1, p.o2, *(float *)(GA + GA_U), p.A, p.w, -4.6 * o3, g_act, g_wg, g_wi, g_ff, SW_fac, L_cur.s);
                if (i == (int)(45 / dt) - 1) ti++;
            }
            via_stub = 0;
            printf("  %-24s o3 %.3f: steering %.2f m | after letting go %.2f m | rms after %.2f\n", q.n, ob, m1, m2, sqrt(s2 / n2));
        }
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "engage")) {  // lane keeping switched on with the truck off-center and drifting out (11-10: -0.9 m, heading out)
        printf("switched on at o1 %s, hands off, through stub_law: peak wheel speed (rad/s) in the first 2 s / max |o1| / overshoot / s until |o1| < 0.2 for good\n", "-0.9 m drifting out / 0.4 m / 0");
        struct pv { const char *n; float gA, db, dl, tw, c; };
        const pv pls[] = { { "ts4w-like (gA 1.5)", 1.5f, 0.0015f, 0.33f, 0.05f, 0.17f }, { "ts5b avg (gA 2.5)", 2.5f, 0.007f, 0.3f, 0.05f, 0.19f } };
        for (auto &q : pls) for (float o10 : { -0.9f, 0.4f, 0.0f }) for (float soft : { 0.0f, 0.35f }) {
            const cfg c2 = { "e", 2 }; apply(c2); p_soft = soft; g_law_at = -1e9;
            pp m; m.gA = q.gA; m.db = q.db; m.delay = q.dl; m.tw = q.tw; m.c = q.c; m.d = q.c * 4.6f;
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
            pp m; m.gA = G; m.db = G > 1.2f ? 0.0034f * (G - 1.0f) * 1.5f : 0; m.delay = 0.33f; m.tw = 0.05f; m.c = 0.19f; m.d = 0.19f * 4.6f; m.kk = 0.9f; m.stick = st;
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
            pp m; m.gA = q.gA; m.db = q.db; m.delay = q.dl; m.tw = q.tw; m.c = q.c; m.kk = q.kk; m.d = q.c * 4.6f; m.stick = q.st;
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
                    double need = -4.6 * o3v;
                    if (fabs(o3v) > 0.008) { if (p.w * need < 0.5 * need * need) under += dt; if (p.w * need > 1.6 * need * need) over += dt; }
                }
                if (tr == k_i && tt - last_tr >= 0.1) { last_tr = tt; printf("  %7.2f %4.1f o1 %6.2f o2 %6.3f o3 %.4f u %6.2f A %7.4f wheel %7.4f need %7.4f act %.2f wg %.2f wi %.2f ff %.4f\n", tt, vkmh, p.o1, p.o2, o3v, u, p.A, p.w, -4.6 * o3v, g_act, g_wg, g_wi, g_ff); }
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
    return test_indicator() | test_indicator_key() | test_ffb() | test_saturation();
}
