// R0611 rows: the Scania of the ride of 06-11 (notes/ride-0611: 12 forward gears, diff 3.08, wheel 0.512, 2500 Nm, limit 2500, 17 t, Eco). Included by sim.cpp; R0611=1 runs only these (run_r0611.sh).
//   a  a hint nobody pressed: the slow-down to 5.5 km/h and the neutral of the stop, the game's own launch gear (1.62 puts the max of the start gear and its lower bound in, g_start_floor), then a calm pedal. The box must not read the gear it
//      comes in with as the driver's hint (ride t 2108.22 "shift 5->2 by hint at 7 km/h: hold 6 s", t 2127.51 "7->2 at 6 km/h"): no hint hold, and the first gear is left at the revs of a calm pull.
//   b  the first upshift after the engine-brake cascade (ride t 2071.37: 4 -> 9 at 30 km/h, 738 rpm, pedal 0.38; max_gears_skipped = 2), PRINTED, no verdict: max_gears_skipped is the span of the plan's later
//      shifts, not a cap on the shift now, and a cap on the landing (tried, gc_in::land_max) fails the frozen run_g7 row R3 (calm15 normal 41.9 t +2 %: 4 > 8 at 2077 rpm becomes 4 > 7 > 9 > 8, 3 downshifts, 39.4 instead of 61.0 km/h).
// Nothing else here sets a number to pass: the bar of row a is "no hint without a press".
#pragma once
#include "core/shiftcommon.h"

static int r6_fail;
static void r6_verdict(bool ok, const char *fmt, ...)
{
    char b[400]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    printf("  R0611 %s: %s\n", ok ? "ok" : "FAIL", b);
    if (!ok) r6_fail = 1;
}
// The Scania of the ride (its engine file gives 7 curve points; ranges from the log: brake 1500-2500, low 700-1500, high 1000-1350, power 1400-1900), idle 550. Returns the box's shift time.
static float r6_truck()
{
    const float sh = sw_engine_def("T=2500|L=2500|C=300:0 440:0.5 1000:1 1300:1 1900:0.77 2400:0.4 2600:0|R=700,1500,1000,1350,1400,1900,1500,2500|G=9.204 7.404 5.87 4.696 3.771 3.027 2.466 1.966 1.561 1.247 1.003 0.802|D=3.08|W=0.512|S=1.0");
    g_idle_sim = 550;
    return sh;
}
static void r6_begin() { set_param("gear_logic", 1.0f); set_param("eco_roll", 0); g_fuel_raw = 1000.0; T.fuel = 1000.0f; }

// ---- a: the stop, the game's launch gear, the calm pedal
static struct { double t_stop, t_go; int hold_seen, g_first, g_up; double rpm_first_max, t_up; int shifts; } r6a;
static float g_r6_ped = 0.2f;
static void r6a_track(double)
{
    if (r6a.t_go >= 0 && t_now - r6a.t_go < 12.0) {
        if (g_inc.flags & INC_HOLD) r6a.hold_seen = 1;
        if (M.gear > 0 && r6a.g_first == 0) r6a.g_first = M.gear;
        if (M.gear > 0 && M.gear == r6a.g_first && !r6a.g_up) r6a.rpm_first_max = fmax(r6a.rpm_first_max, rpm_now());
        if (M.gear > r6a.g_first && r6a.g_first > 0 && !r6a.g_up) { r6a.g_up = M.gear; r6a.t_up = t_now - r6a.t_go; }
    }
}
static void c_r6a(double t)
{
    M.wish_on = 0; g_pl_wish = 1;
    if (r6a.t_go < 0) {
        if (t < 10) { hold(30.0f); M.brake = 0; }                       // 30 km/h in 7, a driver holding it
        else if (r6a.t_stop < 0) {                                         // the slow-down; at 5.5 km/h the game lets the clutch go (ride t 2125-2127: 6 -> 0 at 14 km/h, the foot down again at 5.5)
            M.pedal = 0; M.brake = 0.25f;
            if (M.v * 3.6f <= 5.5f) { M.gear = M.target = 0; M.state = 0; M.brake = 0; r6a.t_stop = t_now; }
        } else { M.pedal = 0; M.brake = 0; }
        if (r6a.t_stop >= 0 && t_now - r6a.t_stop > 2.0) r6a.t_go = t_now;   // 2 s in neutral (ride: 2125 .. 2127), then the foot
    }
    if (r6a.t_go >= 0) { M.brake = 0; M.pedal = g_r6_ped; }
    sim_pedal_wish();
}
static void r6_a_rows()
{
    printf("\n=== R0611 a: the stop, the game's own launch gear (lower bound 2), a calm pedal %.2f, Eco, 17 t ===\n", g_r6_ped);
    const float sh = r6_truck();
    r6_begin();
    setup wash = {17000, 30, 7, 6, 10.0f, sh}; g_quiet = 1; g_pl_wish = 1; run("wash", wash, c_hold80); g_quiet = 0;
    sim_reset(RESET_HARD, "sim: R0611 a");
    r6a = {}; r6a.t_stop = r6a.t_go = -1;
    g_start_floor = 2; g_tick_hook = r6a_track; g_quiet = getenv("R6V") ? 0 : 1;
    setup su = {17000, 30, 7, 40, 10.0f, sh};
    run("R0611a", su, c_r6a);
    g_tick_hook = NULL; g_quiet = 0; g_pl_wish = 0; g_start_floor = 0;
    printf("  first gear %d after the stop, held up to %.0f rpm, first upshift %s (%d, %.1f s after the foot), hint hold seen: %s\n", r6a.g_first, r6a.rpm_first_max,
           r6a.g_up ? "at" : "none", r6a.g_up, r6a.t_up, r6a.hold_seen ? "yes" : "no");
    r6_verdict(r6a.t_go >= 0 && r6a.g_first > 0, "the row has its situation (stopped, the foot down, a first gear: %d)", r6a.g_first);
    r6_verdict(!r6a.hold_seen, "the gear the box came in with is no hint (no hint hold within 12 s of the foot)");
}

// ---- b: the first upshift of a calm pull from the engine-brake gear
static struct { int from, to; double t; } r6b;
static void r6b_track(double)
{
    if (!r6b.from && M.state && M.target > M.gear && M.gear > 0) { r6b.from = M.gear; r6b.to = M.target; r6b.t = t_now; }
}
static void c_r6b(double) { M.wish_on = 0; g_pl_wish = 1; M.pedal = 0.38f; sim_pedal_wish(); }
static void r6_b_rows()
{
    printf("\n=== R0611 b: Eco, 17 t, pedal 0.38 at 30 km/h in 4 (after the engine brake took the box down): the first upshift; max_gears_skipped %.0f ===\n", ts_param("max_skip"));
    const int maxskip = (int)ts_param("max_skip");
    for (int g0 : { 3, 4, 5 }) {
        const float sh = r6_truck();
        r6_begin();
        setup wash = {17000, 30, 7, 6, 10.0f, sh}; g_quiet = 1; g_pl_wish = 1; run("wash", wash, c_hold80); g_quiet = 0;
        sim_reset(RESET_HARD, "sim: R0611 b");
        r6b = {}; g_tick_hook = r6b_track; g_quiet = getenv("R6V") ? 0 : 1;
        setup su = {17000, 30, g0, 10, 10.0f, sh};
        run("R0611b", su, c_r6b);
        g_tick_hook = NULL; g_quiet = 0; g_pl_wish = 0;
        if (r6b.from) printf("  from %d at 30 km/h: %d -> %d (%d skipped)\n", g0, r6b.from, r6b.to, r6b.to - r6b.from - 1);
        else printf("  from %d at 30 km/h: no upshift in 10 s\n", g0);
        (void)maxskip;
    }
}

static int run_r0611()
{
    printf("\n--- R0611 ---");
    r6_fail = 0;
    r6_a_rows();
    r6_b_rows();
    use_volvo();
    return r6_fail;
}
