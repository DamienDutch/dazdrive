// WP-TC1 rows (run: tools/sim/run_tc1.sh = sim.cpp with TC1=1; included by sim.cpp before main). TrueShift on a torque-converter truck and on a mod engine with no torque curve,
// from the ride of 04-11 (the Spectrans Scania with the Allison GA867: 6 speeds 4.70 .. 0.67, diff 3.91, wheel 0.512, dc13_450.sii = `torque: 2350` and nothing else).
// Every row is a plain check (tc_check, ok / FAIL with the numbers). The rows are written against the API of 2aa64e7 (plus -DTC1_BASE for the one stub that differs), so they can
// be run against that commit: what fails there is the ride's symptom. Notes on the ride: notes/ (analysis in the scratchpad of the work package), docs/powerband-change.md "TC1".
#pragma once
#include "core/shiftmodel.h"

// The Spectrans truck. known: the plugin has the converter from the start (the file of the last drive); else the plant has it and the plugin finds it. mod_engine: the plugin's
// read of the engine object holds the torque only (g_mod_engine). stall_rpm: the engine speed at full stall in the plant (the model's, vehicle.cpp, derives 0.85 x the power
// range's bottom = 1190 for the game's default ranges, which is what the plant uses so the plugin's model and the plant agree).
static void use_tc_spectrans(bool known, bool mod_engine, int lock_gear = 1, float stall = 2.42f)
{
    tc_reset();
    float r[16] = { 4.70f, 2.21f, 1.53f, 1.00f, 0.76f, 0.67f };
    memcpy(RATIO, r, sizeof r);
    float c[7][2] = { { 300, 0 }, { 440, 0.5f }, { 1000, 1 }, { 1300, 1 }, { 1900, 0.77f }, { 2400, 0.4f }, { 2600, 0 } }; NCURVE = 7;   // the game's default curve (no torque_curve in the def)
    memcpy(CURVE, c, sizeof c);
    float g[4][2] = { { 1500, 2500 }, { 700, 1500 }, { 1000, 1350 }, { 1400, 1900 } };   // the game's default ranges (brake, low gear, high gear, power)
    memcpy(RANGES, g, sizeof g);
    TORQUE = 2350; CDA = 6; LIMIT = 2500; DIFF = 3.91f; RADIUS = 0.512f; NG = 6;
    T.fwd_gears = 6; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
    if (known) tc_set(stall, lock_gear, 1190);
    else { TC_STALL = stall; TC_LOCK_GEAR = lock_gear; TC_STALL_RPM = 1190; vehicle_tc_prime(0, 0, 0); }
    g_mod_engine = mod_engine; g_mod_limit0 = 0;
}

static int g_tc1_eng_lines;
static void tc1_tap(const char *l) { if (!strncmp(l, "engine definition gives", 23)) g_tc1_eng_lines++; }

// ---- item 1: the engine data of a mod engine
static void tc1_engine_rows()
{
    printf("\n=== TC1 item 1: a mod engine whose definition holds the torque only ===\n");
    for (int variant = 0; variant < 2; variant++) {   // 0: the limit is in the object; 1: it is not either (the SDK's 2500 stands in)
        sim_reset(RESET_HARD, "sim: tc1 mod engine");
        use_tc_spectrans(true, true);
        g_mod_limit0 = variant;
        g_sim_log_tap = tc1_tap; g_tc1_eng_lines = 0;
        g_rec_on = 1; g_rec_n = 0;
        run(variant ? "Spectrans 20 t, launch 8 -> 80, the object holds no limit either" : "Spectrans 20 t, launch 8 -> 80, the object holds the torque and the limit", { 20000, 8, 1, 60 }, c_full);
        g_rec_on = 0; g_sim_log_tap = NULL;
        const char *v = variant ? "no limit" : "limit read";
        tc_check(V.eng_ok == 1, "%s: the engine is usable (eng_ok %d, expect 1; 2aa64e7: 0, the ride's \"no data\")", v, V.eng_ok);
        const float *c2 = V.eng.curve[2];
        tc_check(V.eng.n_curve == 7 && c2[0] == 1000 && c2[1] == 1 && V.eng.curve[6][0] == 2600, "%s: the game's default curve stands in (%d points, 3rd point %.0f rpm: %.2f; expect 7, 1000: 1.00, ending at 2600)", v, V.eng.n_curve, c2[0], c2[1]);
        tc_check(V.eng.range[R_LOW][0] == 700 && V.eng.range[R_LOW][1] == 1500 && V.eng.range[R_HIGH][0] == 1000 && V.eng.range[R_HIGH][1] == 1350 &&
                 V.eng.range[R_POWER][0] == 1400 && V.eng.range[R_POWER][1] == 1900 && V.eng.range[R_BRAKE][0] == 1500 && V.eng.range[R_BRAKE][1] == 2500,
                 "%s: the game's default ranges (low %.0f-%.0f high %.0f-%.0f power %.0f-%.0f brake %.0f-%.0f; expect 700-1500, 1000-1350, 1400-1900, 1500-2500)", v,
                 V.eng.range[R_LOW][0], V.eng.range[R_LOW][1], V.eng.range[R_HIGH][0], V.eng.range[R_HIGH][1], V.eng.range[R_POWER][0], V.eng.range[R_POWER][1], V.eng.range[R_BRAKE][0], V.eng.range[R_BRAKE][1]);
        tc_check(V.eng.rpm_limit == 2500 && V.eng.torque == 2350, "%s: limit %.0f, torque %.0f (expect 2500, 2350: the torque scales the curve, the limit is the object's or the SDK's)", v, V.eng.rpm_limit, V.eng.torque);
        tc_check(g_tc1_eng_lines == 1, "%s: ONE log line names what was missing (%d lines)", v, g_tc1_eng_lines);
        int game = rec_whys("game"), total = g_rec_n;
        tc_check(total >= 3 && game == 0, "%s: the layers decide: %d shifts, %d of them the game's (expect none; 2aa64e7: all)", v, total, game);
        tc_check(hunts == 0 && g_n_incidents == 0, "%s: %d hunting pairs, %d incidents (expect none)", v, hunts, g_n_incidents);
    }
    // an engine that gives its own data is read as before: no default, no line
    sim_reset(RESET_HARD, "sim: tc1 own engine");
    use_tc_allison_2700();
    g_sim_log_tap = tc1_tap; g_tc1_eng_lines = 0;
    run("Allison 20 t (its own curve and ranges), launch 8 -> 80", { 20000, 8, 1, 30 }, c_full);
    g_sim_log_tap = NULL;
    tc_check(V.eng_ok == 1 && V.eng.n_curve == NCURVE && g_tc1_eng_lines == 0, "an engine with its own data: read as it is (%d points of %d), %d default lines (expect 0)", V.eng.n_curve, NCURVE, g_tc1_eng_lines);
    g_mod_engine = 0;
}

// ---- item 2: the converter is detected, and its stall ratio learned
// The ride's launches (trueshift.csv of 04-11, the Spectrans Scania; t = the log's seconds, km/h, the engine rpm, the gear, the clutch the game reported). Window A: the first
// launch of the drive (17 t, pedal 0.11), the clutch is our own Smooth start's until 144.3 s and the converter couples at 8 km/h; B: 362 s (17 t, +6.9 %), the engine 1.17 times the
// driveline at most; C: 620 s (41.5 t on +5.5 %, pedal 0.22 .. 0.24), the converter alone with the clutch closed: 1.67 times over at 3.6 km/h down to 1.13 at 8.5 km/h.
#ifdef TC1_BASE
#define TC1_FIXED 0   // (2aa64e7 has no such field)
#else
#define TC1_FIXED V.tc_stall_fixed
#endif
// the owner's ini: detect, no stall ratio set; the idle of the ride (550)
#ifdef TC1_BASE
static sm_tc_cfg tc1_cfg() { sm_tc_cfg c = { 0, 0, 0 }; return c; }
#else
static sm_tc_cfg tc1_cfg() { sm_tc_cfg c = { 0, 0, 0, 550 }; return c; }
#endif
struct tc1_row { double t; float kmh; int rpm, g; float cl; };
static const tc1_row k_ride_a[] = {
    {142.00,0.0,581,1,0.99}, {142.11,0.0,639,1,0.96}, {142.22,0.1,704,1,0.93}, {142.32,0.3,727,1,0.88}, {142.42,0.6,715,1,0.85}, {142.52,1.0,701,1,0.83}, 
    {142.64,1.4,695,1,0.83}, {142.75,1.9,702,1,0.82}, {142.85,2.3,707,1,0.81}, {142.96,2.8,711,1,0.79}, {143.07,3.3,715,1,0.77}, {143.18,3.7,718,1,0.75}, 
    {143.29,4.2,722,1,0.72}, {143.41,4.6,726,1,0.69}, {143.52,5.1,731,1,0.65}, {143.63,5.5,738,1,0.61}, {143.73,5.9,747,1,0.55}, {143.84,6.3,757,1,0.49}, 
    {143.96,6.7,770,1,0.42}, {144.07,7.2,787,1,0.24}, {144.18,7.5,804,1,0.16}, {144.28,7.9,824,1,0.03}, {144.39,8.2,841,1,0.00}, {144.49,8.5,860,1,0.00}, 
    {144.59,8.8,881,1,0.00}, {144.69,9.1,902,1,0.00}, {144.80,9.4,926,1,0.00}, {144.91,9.6,946,1,0.00}, {145.01,9.9,969,1,0.00}, {145.12,10.1,988,1,0.00}, 
    {145.22,10.3,1007,1,0.00}, {145.34,10.6,1027,1,0.00}, {145.45,10.8,1047,1,0.00}, {145.55,11.0,1063,1,0.00}, {145.66,11.1,1079,1,0.00}, 
    {145.77,11.3,1097,1,0.00}, {145.88,11.5,1111,1,0.00}, {145.99,11.7,1127,1,0.00},
};
static const tc1_row k_ride_b[] = {
    {362.36,5.5,712,1,0.34}, {362.47,5.9,712,1,0.20}, {362.59,6.3,722,1,0.06}, {362.71,6.7,747,1,0.00}, {362.82,7.1,785,1,0.00}, {362.93,7.5,819,1,0.00}, 
    {363.03,7.8,848,1,0.00}, {363.14,8.3,877,1,0.00}, {363.24,8.6,904,1,0.00}, {363.34,9.0,932,1,0.00}, {363.45,9.5,967,1,0.00}, {363.56,9.9,1002,1,0.00},
};
static const tc1_row k_ride_c[] = {
    {619.90,2.8,616,1,0.42}, {620.03,3.0,590,1,0.28}, {620.13,3.2,578,1,0.16}, {620.25,3.4,569,1,0.02}, {620.35,3.6,572,1,0.00}, {620.45,3.9,582,1,0.00}, 
    {620.57,4.1,594,1,0.00}, {620.67,4.4,607,1,0.00}, {620.77,4.6,619,1,0.00}, {620.87,4.8,631,1,0.00}, {621.00,5.0,644,1,0.00}, {621.10,5.2,657,1,0.00}, 
    {621.20,5.4,675,1,0.00}, {621.31,5.6,693,1,0.00}, {621.42,5.9,713,1,0.00}, {621.52,6.1,731,1,0.00}, {621.64,6.3,751,1,0.00}, {621.76,6.5,771,1,0.00}, 
    {621.86,6.7,787,1,0.00}, {621.97,6.9,807,1,0.00}, {622.09,7.1,827,1,0.00}, {622.19,7.3,838,1,0.00}, {622.31,7.5,850,1,0.00}, {622.42,7.7,862,1,0.00}, 
    {622.52,7.9,872,1,0.00}, {622.64,8.1,885,1,0.00}, {622.76,8.3,898,1,0.00}, {622.87,8.5,912,1,0.00}, {622.98,8.7,924,1,0.00}, {623.09,8.9,939,1,0.00}, 
    {623.20,9.0,952,1,0.00}, {623.30,9.3,967,1,0.00},
};
static double g_tc1_det_t;
static void tc1_feed(const tc1_row *w, int n, float *slip, float *odd)
{
    const sm_tc_cfg cfg = tc1_cfg();
    for (int i = 0; i < n; i++) {
        const double dt = i ? w[i].t - w[i - 1].t : 0.11;
        T.rpm = (float)w[i].rpm; T.eff_clutch = w[i].cl; T.speed = w[i].kmh / 3.6f; T.gear = w[i].g;
        sm_learn_rpm(cfg, w[i].g, w[i].kmh / 3.6f, dt, slip, odd);
        if (V.tc && g_tc1_det_t < 0) g_tc1_det_t = w[i].t;
    }
}
static void tc1_detect_rows()
{
    printf("\n=== TC1 item 2: the converter is found on the ride's launches ===\n");
    sim_reset(RESET_HARD, "sim: tc1 detection");
    use_tc_spectrans(false, false);   // (the plant's ratios, diff and wheel for vehicle_rpm_at; the plugin does NOT know the converter)
    float slip = 0, odd = 0; g_tc1_det_t = -1;
    tc1_feed(k_ride_a, sizeof k_ride_a / sizeof k_ride_a[0], &slip, &odd);
    tc_check(!V.tc, "the first launch (144 s, our own clutch slipping, then kr 1.08 and falling): not taken for a converter (tc %d, slip seconds %.2f)", V.tc, slip);
    tc1_feed(k_ride_b, sizeof k_ride_b / sizeof k_ride_b[0], &slip, &odd);
    tc_check(!V.tc, "the 362 s launch (the engine 1.17 times the driveline at most, under the 1.18 line): not taken for a converter (tc %d, slip seconds %.2f)", V.tc, slip);
    tc1_feed(k_ride_c, sizeof k_ride_c / sizeof k_ride_c[0], &slip, &odd);
    tc_check(V.tc == 1 && g_tc1_det_t >= 620.3 && g_tc1_det_t <= 622.0, "the 620 s launch (41.5 t, +5.5 %%): the converter is found at %.2f s of the log (expect 620.4 .. 622.0, the engine over 1.1 idle from 620.7; 2aa64e7: never, the gate was 18 km/h)", g_tc1_det_t);
    tc_check(V.tc == 1 && fabsf(V.rpm_k - 1.0f) < 0.001f && V.tc_stall == 2.0f && !TC1_FIXED, "detected with the tire correction untouched (%.3f), the stall ratio at the prior %.2f and free to learn (fixed %d)", V.rpm_k, V.tc_stall, TC1_FIXED);
    // a clutch box does not look like one: the same launches on a truck with a locked driveline (kr 1.00) and the ride's clutch values
    sim_reset(RESET_HARD, "sim: tc1 clutch box");
    use_ato();
    slip = odd = 0;
    for (int i = 0; i < 600; i++) {   // 10 s of a closed clutch at 6 km/h in 1st .. 2nd at an engine that follows the driveline exactly
        T.eff_clutch = 0; T.speed = 6 / 3.6f; T.gear = 1; T.rpm = vehicle_rpm_at(1, T.speed); sm_learn_rpm(tc1_cfg(), 1, T.speed, 1 / 60.0, &slip, &odd);
    }
    tc_check(!V.tc, "a locked driveline for 10 s in 1st: no converter (tc %d)", V.tc);
}

// 20 s cycles from a standstill: 7 s of a 0.5 pedal (the launch), the brake down to a stop, standing: the launches the stall ratio is learned on
static float g_tc1_stall_at[24]; static int g_tc1_launches, g_tc1_lock_at[24];
static void c_tc1_cycles(double t)
{
    const double ph = fmod(t, 20.0);
    if (ph < 7) { M.pedal = g_ped; M.brake = 0; }
    else if (ph < 16) { M.pedal = 0; M.brake = M.v > 0.3f ? 0.3f : 0.0f; }
    else { M.pedal = 0; M.brake = 0.2f; }
    const int n = (int)(t / 20.0);
    if (ph >= 7 && ph < 7.1 && n < 24 && g_tc1_stall_at[n] == 0) { g_tc1_stall_at[n] = V.tc_stall; g_tc1_lock_at[n] = V.tc_lock_gear; g_tc1_launches = n + 1; }
}
static void tc1_stall_rows()
{
    printf("\n=== TC1 item 2: the stall ratio is learned from the launches ===\n");
    for (int pass = 0; pass < 2; pass++) {   // 0: the converter is 2.42 and the ini says nothing; 1: the ini says 2.10 (counts as given, not learned over)
        for (int i = 0; i < 24; i++) g_tc1_stall_at[i] = 0;
        g_tc1_launches = 0;
        sim_reset(RESET_HARD, "sim: tc1 stall");
        use_tc_spectrans(false, true, 1, 2.42f);   // the plant has the converter; the plugin finds it
        if (pass == 1) set_param("tc_stall", 2.10f);
        g_ped = 0.5f;
        run(pass ? "Spectrans 20 t, launches at a 0.5 pedal, the ini sets the stall ratio to 2.10" : "Spectrans 20 t, launches at a 0.5 pedal, the ini sets nothing", { 20000, 0, 1, 20 * 10 }, c_tc1_cycles);
        set_param("tc_stall", 0);
        printf("        stall ratio at the end of each launch:");
        for (int i = 0; i < g_tc1_launches; i++) printf(" %.2f", g_tc1_stall_at[i]);
        printf("\n");
        if (pass == 0) {
            int first = 0; while (first < g_tc1_launches && g_tc1_stall_at[first] == 0) first++;
            tc_check(V.tc == 1 && !TC1_FIXED && first <= 3, "the plugin found the converter by launch %d (tc %d; 0.5 pedal on 20 t slips for about a second) and learns its ratio", first + 1, V.tc);
            const float l3 = g_tc1_launches > 3 ? g_tc1_stall_at[3] : 0, l9 = g_tc1_launches > 9 ? g_tc1_stall_at[9] : 0;
            tc_check(fabsf(l3 - 2.42f) < 0.25f, "after 4 launches: %.2f (the plant's 2.42, expect within 0.25)", l3);
            tc_check(fabsf(l9 - 2.42f) < 0.15f && fabsf(V.tc_stall - 2.42f) < 0.15f, "after 10 launches: %.2f, at the end %.2f (the plant's 2.42, expect within 0.15; the prior is 2.0)", l9, V.tc_stall);
        } else {
            tc_check(V.tc == 1 && TC1_FIXED && V.tc_stall == 2.10f, "the ini's 2.10 is used as given, not learned over (tc_stall %.2f, fixed %d)", V.tc_stall, TC1_FIXED);
        }
        tc_check(hunts == 0 && g_n_incidents == 0, "%d hunting pairs, %d incidents (expect none)", hunts, g_n_incidents);
    }
    g_mod_engine = 0;
}

// ---- item 3: below the lockup the converter's multiplication decides, not an rpm line
// The rows run the Spectrans on its own engine data (use_tc_spectrans(true, false): item 1 is its own rows) with the converter known, the lock gear not learned (0: the state of a drive
// where the lockup was never seen, and the learner of it never fires in detect mode, shiftmodel.cpp).
static double g_tc1_press_at = 1.0;
static float g_tc1_k12;   // the speed at which the box first stood in 2nd (by the hint or by itself)
static void c_tc1_press(double t)   // 17 t on -1.8 %, a 0.10 pedal (the ride's 520 s): the driver presses + once at g_tc1_press_at
{
    if (t < 0.01) g_tc1_k12 = 0;
    if (M.gear >= 2 && g_tc1_k12 == 0) g_tc1_k12 = M.v * 3.6f;
    M.grade_pct = -1.8f; M.pedal = 0.10f;
    if (t > g_tc1_press_at && t < g_tc1_press_at + 0.04 && !M.state && M.gear == 1) M.hint = 1;
}
static int rec_pair(int a, int b, double t_from = -1e9)   // shifts a -> b the recorder saw after t_from
{
    int n = 0;
    for (int i = 0; i < g_rec_n; i++) if (g_rec[i].a == a && g_rec[i].b == b && g_rec[i].t >= t_from) n++;
    return n;
}
static void tc1_press_rows()
{
    printf("\n=== TC1 item 3: the driver's + in 1st on a 0.10 pedal (the ride's 520 s: 13 .. 14.5 km/h, 586 .. 643 rpm of driveline in 2nd) ===\n");
    // A: M mode (the layers do not shift, the driver does): 11 km/h on a 0.10 pedal, 2nd would run 500 rpm of driveline; the press is the driver's and the converter carries it
    sim_reset(RESET_HARD, "sim: tc1 press");
    use_tc_spectrans(true, false, 0);
    g_rec_on = 1; g_rec_n = 0; g_tc1_press_at = 0.5;
    { setup su = { 17000, 11.0f, 1, 14, 0 }; su.settle = 1.5; run("Spectrans 17 t, M mode, -1.8 %, pedal 0.10 from 11 km/h, + pressed at 0.5 s", su, c_tc1_press); }
    g_rec_on = 0;
    int hint = 0; float hk = 0; for (int i = 0; i < g_rec_n; i++) if (g_rec[i].a == 1 && g_rec[i].b == 2 && !strncmp(g_rec[i].why, "hint", 4)) { hint++; hk = g_rec[i].kmh; }
    tc_check(hint >= 1, "the press is taken: 1 -> 2 by the hint at %.1f km/h (%d; 2aa64e7 refuses it by the stall line)", hk, hint);
    tc_check(rec_pair(2, 1) == 0 && hunts == 0, "and held for the 13 s after it: %d shifts 2 > 1, %d hunting pairs (expect none)", rec_pair(2, 1), hunts);
    // B: the ride's own speeds: 13 km/h, the press at once; in 2nd however it came (the layers go there by themselves at ~11 km/h now), and no way back
    sim_reset(RESET_HARD, "sim: tc1 press ride");
    use_tc_spectrans(true, false, 0);
    g_rec_on = 1; g_rec_n = 0; g_tc1_press_at = 0.02;
    run("Spectrans 17 t, -1.8 %, pedal 0.10 from 13 km/h, + pressed at once", { 17000, 13.0f, 1, 14 }, c_tc1_press);
    g_rec_on = 0;
    const float k12 = g_tc1_k12;
    tc_check(k12 > 0 && k12 <= 14.5f && M.gear >= 2 && rec_pair(2, 1) == 0 && hunts == 0, "at the ride's speeds: in 2nd by %.1f km/h (the ride's presses were at 13 .. 14.5; the game went at 15 .. 16), then gear %d, %d shifts 2 > 1, %d hunting pairs (expect <= 14.5, 0, 0)", k12, M.gear, rec_pair(2, 1), hunts);
}
static void c_tc1_525(double t)   // the ride's 525 s: 16 km/h in 2nd, the pedal 0.05 / 0.13 / 0.10 in turn on -1.7 %
{
    M.grade_pct = -1.7f;
    const double ph = fmod(t, 6.0);
    M.pedal = ph < 2 ? 0.05f : ph < 4 ? 0.13f : 0.10f;   // (never 0: with the foot off on a descent the layers take the drag gear on purpose -- the pedal-off pick, owner point 19.4)
}
static void tc1_hunt_rows()
{
    printf("\n=== TC1 item 3: no 1-2-1 hunt at 16 km/h (the ride's 525 s) ===\n");
    sim_reset(RESET_HARD, "sim: tc1 525");
    use_tc_spectrans(true, false, 0);
    g_rec_on = 1; g_rec_n = 0;
    run("Spectrans 17 t, -1.7 %, 16 km/h in 2nd, the pedal 0.05 / 0.13 / 0.10", { 17000, 16.0f, 2, 24 }, c_tc1_525);
    g_rec_on = 0;
    tc_check(rec_pair(2, 1) == 0 && rec_whys("stall") == 0 && hunts == 0, "no 2 > 1 (%d), no stall downshift (%d), %d hunting pairs, %d shifts in all (expect none of the first three; the game took 2 > 1 at 695 .. 710 rpm)", rec_pair(2, 1), rec_whys("stall"), hunts, g_rec_n);
}
static void c_tc1_calm(double) { M.grade_pct = 0; M.pedal = 0.2f; }
static void tc1_launch_rows()
{
    printf("\n=== TC1 item 3: a calm launch shifts 1 > 2 well before the game's 1400 rpm ===\n");
    for (int pass = 0; pass < 2; pass++) {   // 0: the lock gear is not known (0); 1: it is 2 (the lockup is in from 2nd once the converter couples)
        sim_reset(RESET_HARD, "sim: tc1 launch");
        use_tc_spectrans(true, false, pass ? 2 : 0);
        g_rec_on = 1; g_rec_n = 0;
        run(pass ? "Spectrans 17 t, pedal 0.2 from 8 km/h (lock gear 2)" : "Spectrans 17 t, pedal 0.2 from 8 km/h (lock gear not known)", { 17000, 8.0f, 1, 40 }, c_tc1_calm);
        g_rec_on = 0;
        int up1 = rec_first_up_rpm(); float kmh1 = 0; for (int i = 0; i < g_rec_n; i++) if (g_rec[i].b > g_rec[i].a) { kmh1 = g_rec[i].kmh; break; }
        tc_check(up1 > 0 && up1 < 1400, "the first upshift leaves 1st at %d rpm (%.0f km/h): under the game's 1400 .. 1500 (the ride: 1410 .. 1530 rpm at 14 .. 16 km/h)", up1, kmh1);
        tc_check(hunts == 0 && g_n_incidents == 0, "%d hunting pairs, %d incidents (expect none)", hunts, g_n_incidents);
    }
}
static void c_tc1_climb(double) { M.grade_pct = 6.0f; M.pedal = 0.6f; }
static void tc1_climb_rows()
{
    printf("\n=== TC1 item 3: a steep heavy climb keeps 1st until 2nd can carry it ===\n");
    sim_reset(RESET_HARD, "sim: tc1 climb");
    use_tc_spectrans(true, false, 0);
    g_rec_on = 1; g_rec_n = 0;
    run("Spectrans 41.5 t, +6 %, pedal 0.6 from 5 km/h", { 41500, 5.0f, 1, 40 }, c_tc1_climb);
    g_rec_on = 0;
    // the speed from which 2nd carries what 1st gives at this pedal: the most of 2nd (full pedal, the converter at its slip) against the pull of 1st at the pedal and the road
    float v_ok = 0;
    M.mass = 41500; M.grade_pct = 6.0f; V.mass = 41500; V.grade_deg = atanf(0.06f) * 57.2958f;
    for (float v = 1.0f; v < 12.0f; v += 0.05f) if (vehicle_force_at(2, v, 1.0f) > fmaxf(vehicle_force_at(1, v, 0.6f), vehicle_resist(v))) { v_ok = v; break; }
    float first_kmh = 0; for (int i = 0; i < g_rec_n; i++) if (g_rec[i].a == 1 && g_rec[i].b == 2) { first_kmh = g_rec[i].kmh; break; }
    tc_check(first_kmh == 0 || first_kmh >= v_ok * 3.6f - 0.5f, "1 > 2 at %.1f km/h (0 = stays in 1st); 2nd carries the pedal's pull from %.1f km/h (the test's own figure; expect not before)", first_kmh, v_ok * 3.6f);
    tc_check(hunts == 0 && min_kmh > 3.0f && g_n_incidents == 0, "no hunting pairs (%d), the speed never under %.1f km/h, %d incidents", hunts, min_kmh, g_n_incidents);
}

// ---- item 4: TrueShift owns every shift (decision 60): no takeover speed, no rpm, no handback
static void c_tc1_stop(double t)   // 20 km/h on the flat, the pedal off and the brake down to a stop, then standing
{
    M.grade_pct = 0;
    if (t < 1.0) { M.pedal = 0.15f; M.brake = 0; }
    else { M.pedal = 0; M.brake = M.v > 0.3f ? 0.3f : 0.2f; }
}
static void tc1_own_rows()
{
    printf("\n=== TC1 item 4: TrueShift owns every shift, from the standstill on ===\n");
    // 1. a launch from rest: every shift is ours (the game's stock chooser took the first ones below 10 km/h / 1000 rpm)
    sim_reset(RESET_HARD, "sim: tc1 own launch");
    use_tc_spectrans(true, false, 0);
    g_rec_on = 1; g_rec_n = 0;
    run("Spectrans 17 t, a calm launch from rest (pedal 0.2)", { 17000, 0.0f, 1, 40 }, c_tc1_calm);
    g_rec_on = 0;
    int up1 = rec_first_up_rpm(); float k1 = 0; for (int i = 0; i < g_rec_n; i++) if (g_rec[i].b > g_rec[i].a) { k1 = g_rec[i].kmh; break; }
    tc_check(g_rec_n >= 2 && rec_whys("game") == 0, "from a standstill: %d shifts, %d of them the game's (expect none), the first upshift at %d rpm (%.0f km/h)", g_rec_n, rec_whys("game"), up1, k1);
    tc_check(hunts == 0 && g_n_incidents == 0, "%d hunting pairs, %d incidents (expect none)", hunts, g_n_incidents);
    // 2. the stop: braking from 12 km/h in 3rd (under the old takeover rpm: the game's chooser took the downshifts here) to rest: ours, no hunt
    sim_reset(RESET_HARD, "sim: tc1 own stop");
    use_tc_spectrans(true, false, 0);
    g_rec_on = 1; g_rec_n = 0;
    run("Spectrans 17 t, 12 km/h in 3rd, the brake down to a stop", { 17000, 12.0f, 3, 25 }, c_tc1_stop);
    g_rec_on = 0;
    tc_check(rec_whys("game") == 0 && hunts == 0 && g_n_incidents == 0, "braking to a stop: %d shifts, %d of them the game's (expect none), %d hunting pairs, %d incidents; end %.1f km/h in gear %d", g_rec_n, rec_whys("game"), hunts, g_n_incidents, M.v * 3.6f, M.gear);
    // 3. the 525 s of the ride: 16 km/h in 2nd on a calm, wavering pedal: the converter carries it, no way back to 1st
    sim_reset(RESET_HARD, "sim: tc1 525");
    use_tc_spectrans(true, false, 0);
    g_rec_on = 1; g_rec_n = 0;
    run("Spectrans 17 t, -1.7 %, 16 km/h in 2nd, the pedal 0.05 / 0.13 / 0.10", { 17000, 16.0f, 2, 24 }, c_tc1_525);
    g_rec_on = 0;
    tc_check(rec_pair(2, 1) == 0 && rec_whys("stall") == 0 && rec_whys("game") == 0 && hunts == 0, "no 2 > 1 (%d), no stall downshift (%d), none of the game's (%d), %d hunting pairs, %d shifts in all (the game took 2 > 1 at 695 .. 710 rpm)", rec_pair(2, 1), rec_whys("stall"), rec_whys("game"), hunts, g_rec_n);
}

// ---- item 2 (round 2): the lock gear is learned, so the force test is for the gear under the lockup and the force correction learns on a converter
static void tc1_lock_rows()
{
    printf("\n=== TC1 item 2: the lock gear is learned from the launches; f_scale learns on a converter ===\n");
    for (int i = 0; i < 24; i++) { g_tc1_stall_at[i] = 0; g_tc1_lock_at[i] = 0; }
    g_tc1_launches = 0;
    sim_reset(RESET_HARD, "sim: tc1 lock");
    use_tc_spectrans(false, true, 1, 2.42f);   // the plant couples in 1st above ~9 km/h at a 0.5 pedal; the plugin finds the converter, then the lock gear
    g_plant_eta = 0.80f;                       // the plant's driveline is 0.80 where the model says 0.90: f_scale has something to learn (0.89)
    g_ped = 0.5f;
    run("Spectrans 20 t, launches at a 0.5 pedal, the plant's driveline efficiency 0.80", { 20000, 0, 1, 20 * 6 }, c_tc1_cycles);
    int found = 0; while (found < g_tc1_launches && g_tc1_lock_at[found] == 0) found++;
    printf("        lock gear at the end of each launch:");
    for (int i = 0; i < g_tc1_launches; i++) printf(" %d", g_tc1_lock_at[i]);
    printf("\n");
    tc_check(V.tc == 1 && V.tc_lock_gear == 1 && found < g_tc1_launches, "the lock gear is learned: gear %d, known from launch %d of %d (the plant couples in 1st; expect 1, within 6 launches)", V.tc_lock_gear, found + 1, g_tc1_launches);
    // above it the rpm checks apply: coupled in 1st at 15 km/h on a 0.5 pedal is locked and carries no relief; slipping (4 km/h) it is not locked and may
    T.eff_thr = 0.5f;
    const float v15 = 15 / 3.6f, v4 = 4 / 3.6f;
#ifndef TC1_BASE   // (vehicle_tc_carries is new)
    tc_check(vehicle_tc_locked(1, v15) == 1 && vehicle_tc_carries(1, v15, 0) == 0, "coupled in the lock gear (1st at 15 km/h, pedal 0.5): locked %d, relief %d (expect 1, 0: the rpm checks apply)", vehicle_tc_locked(1, v15), vehicle_tc_carries(1, v15, 0));
    tc_check(vehicle_tc_locked(1, v4) == 0 && vehicle_tc_carries(1, v4, 0) == 1, "slipping (1st at 4 km/h, pedal 0.5): locked %d, relief %d (expect 0, 1: below the lockup)", vehicle_tc_locked(1, v4), vehicle_tc_carries(1, v4, 0));
#endif
    run("Spectrans 20 t, a steady 0.5 pedal from 60 km/h in 5th, the plant's efficiency 0.80", { 20000, 60.0f, 5, 40 }, c_p50);
    tc_check(V.f_scale < 0.95f && V.f_scale > 0.85f, "f_scale learned on the converter: %.3f (the plant's 0.80 over the model's 0.90 = 0.89; expect 0.85 .. 0.95; 2aa64e7: it stays 1.000, a converter is never 'locked')", V.f_scale);
    g_plant_eta = 0.9f;
    g_mod_engine = 0;
}

static int run_tc1()
{
    g_tc_failed = 0;
    tc1_engine_rows();
    tc1_detect_rows();
    tc1_stall_rows();
    tc1_lock_rows();
    tc1_press_rows();
    tc1_launch_rows();
    tc1_climb_rows();
    tc1_own_rows();
    printf("\nTC1: %s\n", g_tc_failed ? "FAILED" : "ok");
    return g_tc_failed;
}
