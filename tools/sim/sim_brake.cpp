// Offline test of the learned service-brake gain (core/vehicle.cpp, B1 of the dynamic-threshold audit; [core] brake_learn).
// A point-mass truck on the vehicle model's own resistance, a brake of an unknown gain (x0.6 / x1.0 / x1.5 of 6 m/s2 per request at
// 18 t, the same force at every load) behind a 0.15 s lag; braking events from 22 m/s by the driver (pedal) and by "us" (act_request).
// Asserts:
//  - convergence: per gain x mass (18, 36 t) x grade (flat, +3 deg up, -4 deg down), after 5 events the estimate is within 10 % of the truth
//    and confident (the gain is offered) after 4;
//  - outliers do not move it: a slope change the game reports late (a sample off by a lot), a clutch event in the window, a throttle blip,
//    a retarder step, a wheel slip (the plant brakes at 60 % for a moment: ABS) -- within 10 % afterwards;
//  - a load change keeps it: 18 -> 36 t (and back) gives the new mass's gain within 10 % before one new event;
//  - a trailer is a separate slot: NAN until learned, the truck alone unchanged;
//  - hot brakes (fade: x0.7 at 3x the temperature of the learned samples) neither move the estimate nor count as new brakes;
//  - brakes that really change (x1.0 -> x0.6) are followed after four agreeing outliers;
//  - persistence: after a (hard) reset the file brings the estimate back at once, confident; another truck's line is kept;
//  - brake_learn = 0: nothing offered, no file.
// Build+run: tools/sim/run_brake.sh [-v]
#include "core/suite.h"
#include "core/game.h"
#include "core/vehicle.h"
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <initializer_list>

telemetry T;
char g_dir[MAX_PATH];
static layout g_layout;
const layout *L = &g_layout;
game_addrs G;
int g_log_level = 2;
static double t_now;
static int verbose;
static char g_logbuf[64][200];
static int g_nlog;
void logring(const char *, ...) {}
void logline(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    if (g_nlog < 64) vsnprintf(g_logbuf[g_nlog++], sizeof g_logbuf[0], fmt, ap);
    va_end(ap);
    if (!verbose) return;
    va_start(ap, fmt); printf("%7.2f  ", t_now); vprintf(fmt, ap); printf("\n"); va_end(ap);
}
double wall_s() { return t_now; }
double sim_s() { return t_now; }
static char g_file[256];
FILE *open_out(const char *name, const char *mode)
{
    char p[300]; snprintf(p, sizeof p, "%s/%s", g_file, name);
    return fopen(p, mode);
}
static float g_act_req;
float act_request() { return g_act_req; }
static float g_mass = 18000, g_grade_deg, g_grade_seen_deg;
float game_mass() { return g_mass; }
float game_grade_deg(void *) { return g_grade_seen_deg; }
static int GB;
void *game_gearbox() { return &GB; }
float wheel_radius() { return 0.5f; }
float curve_factor(const engine_info *, float) { return 1; }
int gb_engine(void *, engine_info *e)
{
    memset(e, 0, sizeof *e);
    e->torque = 2400; e->rpm_limit = 1900; e->n_curve = 0;
    return 1;
}

static int bad, nchk;
static void check(bool ok, const char *fmt, ...)
{
    nchk++;
    va_list ap; va_start(ap, fmt);
    char b[300]; vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    if (!ok) { bad++; printf("FAIL  %s\n", b); } else if (verbose) printf("ok    %s\n", b);
}

// ---- the plant
static float g_gm_true;                   // the brakes' force at request 1 (N)
static double g_v, g_y, g_x;
static float g_slip = 1;                  // the share of the brake force the wheels give (ABS / slip)
static float g_noise = 0.004f;            // speed telemetry noise (m/s)
static float g_ret;                       // a retarder step, in the plant: an extra 1.5 m/s2 at 18 t
static float g_thr;                       // throttle
static float g_clutch;
static unsigned g_seed = 1;
static int g_wheels = 6;
static float rnd() { g_seed = g_seed * 1664525u + 1013904223u; return ((g_seed >> 8) & 0xffff) / 65536.0f - 0.5f; }

// One 50 Hz frame; req_driver / req_ours are the requests. The vehicle model's resistance is the plant's (the true grade g_grade_deg).
static void frame(float req_driver, float req_ours)
{
    const double dt = 0.02;
    t_now += dt;
    float seen = g_grade_seen_deg; (void)seen;
    float req = fmaxf(req_driver, req_ours);
    g_y += (req - g_y) * dt / (0.15 + dt);
    // the plant's resistance: rolling + air + the true grade (the model reads the grade the game reports)
    float s = sinf(g_grade_deg * 0.0174533f);
    float res = g_mass * 9.81f * (s + V.crr) + 0.5f * 1.2f * V.cda * (float)(g_v * g_v);
    float f = -(float)g_y * g_gm_true * g_slip - res - g_ret * 1.5f * 18000 + g_thr * 80000;
    g_v = fmax(0.0, g_v + f / g_mass * dt);
    g_x += g_v * dt;
    T.speed = (float)g_v + g_noise * rnd();
    T.sim_time = (scs_timestamp_t)(t_now * 1e6);
    T.in_brake = T.eff_brake = req_driver;
    g_act_req = req_ours;
    T.in_thr = T.eff_thr = g_thr;
    T.eff_clutch = g_clutch;
    T.gear = T.dgear = 8; T.engine_on = 1; T.fwd_gears = 12; T.wheels = g_wheels;
    T.retarder = g_ret > 0 ? 2 : 0;
    vehicle_frame();
}

// A braking event: from 22 m/s, the pedal (driver) or our request at level r for secs, then coast.
static void event(float r, float secs, int ours)
{
    // get to speed (throttle on: the estimator sees a throttle and ignores it)
    g_thr = 1;
    for (int i = 0; i < 50 * 60 && g_v < 22; i++) frame(0, 0);
    g_thr = 0;
    for (int i = 0; i < 25; i++) frame(0, 0);                   // 0.5 s of coasting before the pedal
    for (int i = 0; i < (int)(secs * 50); i++) frame(ours ? 0 : r, ours ? r : 0);
    for (int i = 0; i < 25; i++) frame(0, 0);
}

static void reset_all(float gm_factor, float mass, float grade_deg, const char *dir)
{
    memset(&T, 0, sizeof T);
    T.ratio[0] = 10; T.fwd_gears = 12; T.diff = 3; T.wheels = 6;
    snprintf(g_file, sizeof g_file, "%s", dir);
    g_mass = mass; g_grade_deg = g_grade_seen_deg = grade_deg;
    g_gm_true = gm_factor * 6.0f * 18000.0f;
    g_v = 22; g_y = 0; g_x = 0; g_slip = 1; g_ret = 0; g_thr = 0; g_clutch = 0; g_act_req = 0;
    vehicle_reset(RESET_HARD);   // (saves the last case's line: a new case starts without a file)
    { char p[300]; snprintf(p, sizeof p, "%s/brake_gain.txt", dir); remove(p); }
    V.mass = 0; V.mass_ok = 0;
    for (int i = 0; i < 20; i++) frame(0, 0);   // the vehicle model reads its mass, engine
}

static float truth(float mass) { return g_gm_true / mass; }
static float err() { float g = vehicle_brake_gain(); return g == g ? g / truth(g_mass) - 1 : NAN; }

int main(int argc, char **argv)
{
    verbose = argc > 1 && !strcmp(argv[1], "-v");
    char dir[] = "/tmp/sim_brake_XXXXXX";
    if (!mkdtemp(dir)) return 2;
    char cmd[300];
    g_brake_learn = 1;
    // ---- convergence
    printf("--- convergence: gain x mass x grade, events of 4 s at 0.3-0.7 (driver and ours alternating)\n");
    printf("%-26s %s\n", "case", "error after 3 / 4 / 5 / 8 events (conf at 4)");
    const float levels[8] = { 0.4f, 0.6f, 0.3f, 0.7f, 0.5f, 0.35f, 0.65f, 0.45f };
    float worst5 = 0;
    for (float gf : { 0.6f, 1.0f, 1.5f })
        for (float mass : { 18000.0f, 36000.0f })
            for (float gr : { 0.0f, 3.0f, -4.0f }) {
                if (gf * 6 * 18000 / mass < 1.5f) continue;
                snprintf(cmd, sizeof cmd, "rm -f %s/brake_gain.txt", dir); if (system(cmd)) {}
                reset_all(gf, mass, gr, dir);
                float e[9] = {}; float conf4 = 0; int offered4 = 0;
                for (int i = 0; i < 8; i++) {
                    event(levels[i], 4.0f, i & 1);
                    float er = err();
                    e[i + 1] = er;
                    if (i == 3) { conf4 = vehicle_brake_conf(); offered4 = er == er; }
                }
                char nm[80]; snprintf(nm, sizeof nm, "x%.1f %2.0ft %+.0f deg", gf, mass / 1000, gr);
                printf("%-26s %+6.1f%% %+6.1f%% %+6.1f%% %+6.1f%%  (conf %.2f)\n", nm, e[3] * 100, e[4] * 100, e[5] * 100, e[8] * 100, conf4);
                check(offered4, "%s: the gain is offered after 4 events", nm);
                check(e[5] == e[5] && fabsf(e[5]) <= 0.10f, "%s: within 10 %% after 5 events (%+.1f %%)", nm, e[5] * 100);
                check(e[8] == e[8] && fabsf(e[8]) <= 0.10f, "%s: within 10 %% after 8 events (%+.1f %%)", nm, e[8] * 100);
                worst5 = fmaxf(worst5, fabsf(e[5]));
            }
    printf("worst error after 5 events: %.1f %%\n", worst5 * 100);
    // a noisy speed channel (the game's speed is a float in m/s, but the real brake may be less even than the plant)
    for (float nz : { 0.03f, 0.1f }) {
        g_noise = nz;
        snprintf(cmd, sizeof cmd, "rm -f %s/brake_gain.txt", dir); if (system(cmd)) {}
        reset_all(1.0f, 18000, 0, dir);
        for (int i = 0; i < 6; i++) event(levels[i], 4.0f, i & 1);
        check(err() == err() && fabsf(err()) <= 0.10f, "speed noise %.2f m/s: within 10 %% after 6 events (%+.1f %%)", nz, err() * 100);
    }
    g_noise = 0.004f;

    // ---- outliers
    printf("--- outliers (after 6 clean events; x1.0, 18 t, flat)\n");
    snprintf(cmd, sizeof cmd, "rm -f %s/brake_gain.txt", dir); if (system(cmd)) {}
    reset_all(1.0f, 18000, 0, dir);
    for (int i = 0; i < 6; i++) event(levels[i], 4.0f, i & 1);
    float base = vehicle_brake_gain();
    check(base == base && fabsf(err()) < 0.05f, "clean: converged (%+.1f %%)", err() * 100);
    // (a) the road tilts 5 deg uphill-to-downhill mid-event and the game reports it half a second late
    for (int k = 0; k < 4; k++) {
        g_thr = 1; for (int i = 0; i < 3000 && g_v < 22; i++) frame(0, 0); g_thr = 0;
        for (int i = 0; i < 25; i++) frame(0, 0);
        for (int i = 0; i < 75; i++) frame(0.5f, 0);
        g_grade_deg = k & 1 ? 6.0f : -6.0f;                         // the slope changes now ...
        for (int i = 0; i < 25; i++) frame(0.5f, 0);                // ... the game says so after 0.5 s
        g_grade_seen_deg = g_grade_deg;
        for (int i = 0; i < 100; i++) frame(0.5f, 0);
        g_grade_deg = g_grade_seen_deg = 0;
        for (int i = 0; i < 25; i++) frame(0, 0);
    }
    check(fabsf(err()) <= 0.10f, "slope change reported late: still within 10 %% (%+.1f %%)", err() * 100);
    // (b) clutch event (a shift) in the window
    for (int k = 0; k < 4; k++) {
        g_thr = 1; for (int i = 0; i < 3000 && g_v < 22; i++) frame(0, 0); g_thr = 0;
        for (int i = 0; i < 25; i++) frame(0, 0);
        for (int i = 0; i < 100; i++) frame(0.5f, 0);
        g_clutch = 1; for (int i = 0; i < 50; i++) frame(0.5f, 0);   // the clutch opens: the window breaks
        g_clutch = 0; for (int i = 0; i < 100; i++) frame(0.5f, 0);
        for (int i = 0; i < 25; i++) frame(0, 0);
    }
    check(fabsf(err()) <= 0.10f, "clutch events: still within 10 %% (%+.1f %%)", err() * 100);
    // (c) a retarder step and a throttle blip during braking
    for (int k = 0; k < 4; k++) {
        g_thr = 1; for (int i = 0; i < 3000 && g_v < 22; i++) frame(0, 0); g_thr = 0;
        for (int i = 0; i < 25; i++) frame(0, 0);
        for (int i = 0; i < 100; i++) frame(0.5f, 0);
        g_ret = 1; for (int i = 0; i < 100; i++) frame(0.5f, 0); g_ret = 0;
        g_thr = 0.3f; for (int i = 0; i < 20; i++) frame(0.5f, 0); g_thr = 0;
        for (int i = 0; i < 100; i++) frame(0.5f, 0);
        for (int i = 0; i < 25; i++) frame(0, 0);
    }
    check(fabsf(err()) <= 0.10f, "retarder and throttle: still within 10 %% (%+.1f %%)", err() * 100);
    // (d) wheel slip: the wheels give 55 % of the brake in some windows (ABS on a patch of ice), the rest is normal
    for (int k = 0; k < 6; k++) {
        g_thr = 1; for (int i = 0; i < 3000 && g_v < 22; i++) frame(0, 0); g_thr = 0;
        for (int i = 0; i < 25; i++) frame(0, 0);
        for (int i = 0; i < 100; i++) frame(0.6f, 0);
        g_slip = 0.55f; for (int i = 0; i < 100; i++) frame(0.6f, 0); g_slip = 1;
        for (int i = 0; i < 100; i++) frame(0.6f, 0);
        for (int i = 0; i < 25; i++) frame(0, 0);
    }
    check(fabsf(err()) <= 0.10f, "wheel slip windows: still within 10 %% (%+.1f %%)", err() * 100);

    // ---- load change
    printf("--- load change\n");
    float e18 = err();
    g_mass = 36000; vehicle_reset(RESET_LOAD); for (int i = 0; i < 40; i++) frame(0, 0);
    float g36 = vehicle_brake_gain();
    check(g36 == g36 && fabsf(g36 / truth(36000) - 1) <= 0.10f, "18 -> 36 t: the gain at the new mass at once, no new event (est %.2f, true %.2f m/s2 per request)", g36, truth(36000));
    g_mass = 18000; vehicle_reset(RESET_LOAD); for (int i = 0; i < 40; i++) frame(0, 0);
    check(fabsf(err()) <= 0.10f, "and back to 18 t (%+.1f %%, was %+.1f %%)", err() * 100, e18 * 100);

    // ---- a trailer: its own slot
    printf("--- a trailer\n");
    float tr_factor = 0.8f;                                           // the combination brakes differently: more force in all
    float gm_solo = g_gm_true;
    T.trailer = 1; g_mass = 36000; g_gm_true = gm_solo * 2.0f * tr_factor; vehicle_reset(RESET_LOAD); for (int i = 0; i < 40; i++) frame(0, 0);
    check(vehicle_brake_gain() != vehicle_brake_gain(), "with a trailer: nothing offered before it is learned");
    for (int i = 0; i < 5; i++) event(levels[i], 4.0f, i & 1);
    check(fabsf(err()) <= 0.10f, "with a trailer: learned in 5 events (%+.1f %%)", err() * 100);
    T.trailer = 0; g_mass = 18000; g_gm_true = gm_solo; vehicle_reset(RESET_LOAD); for (int i = 0; i < 40; i++) frame(0, 0);
    check(fabsf(err()) <= 0.10f, "and the truck alone is as it was (%+.1f %%)", err() * 100);

    // ---- fade: hot brakes give less, the estimate keeps the cool brakes
    printf("--- fade (x0.7 while hot)\n");
    {
        float gm_cool = g_gm_true;
        T.brake_temp = 60;
        for (int i = 0; i < 4; i++) event(levels[i % 8], 4.0f, i & 1);   // the temperature of the learned samples
        float e0 = err();
        T.brake_temp = 180; g_gm_true = 0.7f * gm_cool;
        for (int i = 0; i < 8; i++) event(levels[i % 8], 4.0f, i & 1);
        g_gm_true = gm_cool;
        check(fabsf(err()) <= 0.10f, "8 hot events leave it at the cool brakes (%+.1f %% -> %+.1f %%)", e0 * 100, err() * 100);
        T.brake_temp = 65;
    }

    // ---- the brakes change for good
    printf("--- brakes that change (x1.0 -> x0.6, a physics mod)\n");
    g_gm_true = 0.6f * 6.0f * 18000.0f;
    int n_ev = 0;
    for (; n_ev < 12; n_ev++) { event(levels[n_ev % 8], 4.0f, n_ev & 1); if (fabsf(err()) <= 0.10f) break; }
    check(n_ev < 12, "follows after %d events (%+.1f %%)", n_ev + 1, err() * 100);

    // ---- persistence
    printf("--- persistence\n");
    float before = vehicle_brake_gain();
    vehicle_reset(RESET_HARD);                                     // saves, forgets
    check(vehicle_brake_conf() == 0, "after a hard reset nothing is held in memory");
    for (int i = 0; i < 40; i++) frame(0, 0);                      // the file is read at the first frame
    float after = vehicle_brake_gain();
    check(after == after && fabsf(after / before - 1) < 0.03f, "reloaded from the file at once, confident (%.2f -> %.2f m/s2 per request, conf %.2f)", before, after, vehicle_brake_conf());
    {   // another truck's line stays, this one's is replaced
        char p[300]; snprintf(p, sizeof p, "%s/brake_gain.txt", dir);
        FILE *f = fopen(p, "r"); char buf[2000] = {}; size_t n = f ? fread(buf, 1, sizeof buf - 1, f) : 0; if (f) fclose(f); buf[n] = 0;
        check(strstr(buf, "truck=E2400-1900-G12-W6 ") != NULL, "the file has this truck's line:  %.80s", buf);
        g_wheels = 8; vehicle_reset(RESET_HARD); for (int i = 0; i < 40; i++) frame(0, 0);
        check(vehicle_brake_gain() != vehicle_brake_gain(), "another truck (8 wheels) starts with nothing");
        vehicle_brake_prime(0, 100000, 0.02f, 6); for (int i = 0; i < 5; i++) { event(0.5f, 4.0f, 0); }
        vehicle_reset(RESET_HARD);
        f = fopen(p, "r"); n = f ? fread(buf, 1, sizeof buf - 1, f) : 0; if (f) fclose(f); buf[n] = 0;
        check(strstr(buf, "W6 ") && strstr(buf, "W8 "), "both trucks' lines are in the file");
        g_wheels = 6;
    }

    // ---- off
    printf("--- brake_learn = 0\n");
    snprintf(cmd, sizeof cmd, "rm -f %s/brake_gain.txt", dir); if (system(cmd)) {}
    g_brake_learn = 0;
    reset_all(1.0f, 18000, 0, dir);
    for (int i = 0; i < 5; i++) event(0.5f, 4.0f, 0);
    check(vehicle_brake_gain() != vehicle_brake_gain() && vehicle_brake_conf() == 0, "off: no gain offered");
    {
        char p[300]; snprintf(p, sizeof p, "%s/brake_gain.txt", dir);
        check(access(p, F_OK) != 0, "off: no file written");
    }
    check(vehicle_brake_req(2.0f, 0.37f) == 0.37f && vehicle_brake_hold_req(8, 0.6f) == 0.6f, "off: the helpers answer the fallback");
    g_brake_learn = 1;

    // ---- the helpers
    printf("--- helpers (primed gain 6 m/s2 per request at 18 t)\n");
    reset_all(1.0f, 18000, 0, dir);
    vehicle_brake_prime(0, 6.0f * 18000, 0.02f, 8);
    check(fabsf(vehicle_brake_req(3.0f, 0.1f) - 0.5f) < 0.01f, "request for 3 m/s2 = %.2f", vehicle_brake_req(3.0f, 0.1f));
    check(vehicle_brake_req(9.0f, 0.1f) == 1.0f, "request for 9 m/s2 is capped at 1");
    float h0 = vehicle_brake_hold_req(0, 0.6f), h12 = vehicle_brake_hold_req(12, 0.6f), h25 = vehicle_brake_hold_req(25, 0.6f);
    printf("hold request: flat %.2f, 12 %% %.2f, 25 %% %.2f\n", h0, h12, h25);
    check(h0 == 0.2f && h12 > h0 && h25 > h12 && h25 <= 1.0f, "hold request grows with the grade, floor 0.2, cap 1");
    check(h12 * 6.0f >= 2.0f * 9.81f * sinf(atanf(0.12f)), "12 %%: the hold's deceleration (%.2f m/s2) is at least twice the slope's pull", h12 * 6.0f);
    snprintf(cmd, sizeof cmd, "rm -rf %s", dir); if (system(cmd)) {}
    printf("brake gain sim: %s (%d checks)\n", bad ? "FAIL" : "ok", nchk);
    return bad ? 1 : 0;
}
