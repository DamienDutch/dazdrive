// G1 rows (WP-0911-G1): two things of the gearbox before the pedal rework. Included by sim.cpp; G1=1 runs only these (run_g1.sh).
//   a  the service brake alone never raises the revs (owner 05-11, memory rem-geen-toeren, docs/core-values.md 5): the Scania of the ride of 08-11 (SpecTrans, 12 gears, Eco, 34.9 t) braking at 55 km/h in 11 with
//      the pedal FEATHERING, the ride's own pedal trace (t 270.8 .. 276: 0.39 0.21 0.01 0.00 0.00 0.06 0.12 0.13 0.02 0.00 0.00 0.16, one sample per 0.11 s). The pedal reads 0.00 in the gaps while the truck
//      still brakes (the deceleration is a low-pass of the speed and the game's brake lets go over a moment), and the braking rule read the momentary pedal: 11>9 landed at 1286 rpm and 9>7 at 1492 rpm in
//      the ride, on 880 and 990 rpm that were nowhere near a stall. The row: every braking downshift of the episode takes the tallest gear that holds the floor of the rule (a gear above the landing would
//      be under it at the speed the shift ends at). A brake the telemetry does not show (AEB: M.brake_unseen) is not the driver's foot: it keeps the layers' landing (guard row, no verdict on the number).
//   b  the route layer is OFF in the gearbox ([trueshift] lookahead = 0, the default): with a route present, in every scene below, the gears are the gears of a truck with no route at all. With the switch
//      on, the same scenes differ (the rows have teeth), so the route code is still there and still reached.
#pragma once
#include "core/shiftcommon.h"

static int g1_fail;
static void g1_verdict(bool ok, const char *fmt, ...)
{
    char b[600]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    printf("  G1 %s: %s\n", ok ? "ok" : "FAIL", b);
    if (!ok) g1_fail = 1;
}
float ts_brake_floor();

// ---- a: the feathering pedal
// The ride's pedal trace (08-11, 270.8 .. 276 s), looped, after a first stop of 0.45 for 0.8 s (what made the truck decelerate at 1.4 m/s2 when the pedal first read 0.00).
static const float G1_PED[12] = { 0.39f, 0.21f, 0.01f, 0.00f, 0.00f, 0.06f, 0.12f, 0.13f, 0.02f, 0.00f, 0.00f, 0.16f };
static const double G1_DT = 0.11;
static float g_g1_unseen;   // > 0: the brake is not in the telemetry (a plugin brake; AEB), the pedal reads 0
static void c_g1_hold60(double) { hold(60.0f); }   // (the wash before a scene: it ends at the speed the scene starts at, so the acceleration the telemetry low-passes is not a jump from another speed)
static void c_g1_feather(double t)
{
    M.pedal = 0;
    const double t0 = 4.0;                                      // 4 s at 60 km/h first (the driver holds the speed), then the brake
    if (t < t0) { hold(60.0f); M.brake = 0; M.brake_unseen = 0; return; }
    const double u = t - t0;
    if (u < 0.8) { M.brake = 0.45f; M.brake_unseen = 0; }       // the stop that gets the deceleration to ~1.5 m/s2 (the ride: -1.4 at the first 0.00)
    else { M.brake = G1_PED[(int)((u - 0.8) / G1_DT) % 12]; M.brake_unseen = 0; }
    if (g_g1_unseen > 0) { M.brake_unseen = u < 0.8 ? 0.45f : g_g1_unseen * G1_PED[(int)((u - 0.8) / G1_DT) % 12] / 0.39f; M.brake = 0; }
}
struct g1_dn { double t; int a, b; float kmh, land, floor_, next_rpm, v_l; };
static g1_dn g1_dns[32]; static int g1_ndn, g1_prev_state, g1_seen;
static float g1_shift_s;
// A shift is logged when it completes; the rule's floor and the speed it judged are those of the tick it was decided in: the edge of M.state. The reason comes with the log line, later.
static g1_dn g1_cap[64]; static int g1_ncap;
static void g1_track(double)
{
    if (M.state && !g1_prev_state && M.target < M.gear && M.gear > 0 && g1_ncap < 64) {
        const float v = M.v, decel = fmaxf(0.0f, -T.accel);
        const float v_l = fmaxf(0.5f, v - decel * (g1_shift_s + 0.25f));   // the speed at the end of the shift, as the rule judges it
        g1_cap[g1_ncap++] = { t_now, M.gear, M.target, v * 3.6f, vehicle_engine_rpm_at(M.target, v_l), ts_brake_floor(), vehicle_engine_rpm_at(M.target + 1, v_l), v_l };
    }
    g1_prev_state = M.state;
    while (g1_seen < g_rec_n) {
        const auto &r = g_rec[g1_seen++];
        if (r.b >= r.a || strncmp(r.why, "braking", 7) || !strncmp(r.why, "braking assist", 14) || g1_ndn >= 32) continue;
        for (int i = g1_ncap - 1; i >= 0; i--) if (g1_cap[i].a == r.a && g1_cap[i].b == r.b) { g1_dns[g1_ndn++] = g1_cap[i]; break; }
    }
}
static void g1_a_scene(const char *what, float unseen, bool must_tall)
{
    printf("\n=== G1 a: %s (Scania of the ride of 08-11, Eco, 34.9 t, 11 at 55-60 km/h, the pedal of the ride) ===\n", what);
    const float sh = r6_truck();
    r6_begin();
    g1_shift_s = sh;
    setup wash = {34900, 60, 11, 10, 10.0f, sh}; g_quiet = 1; g_pl_wish = 1; run("wash", wash, c_g1_hold60); g_quiet = 0;
    sim_reset(RESET_HARD, "sim: G1 a");
    g_g1_unseen = unseen; g1_ndn = g1_seen = g1_ncap = g1_prev_state = 0; g_rec_n = 0; g_rec_on = 1; g_tick_hook = g1_track; g_quiet = getenv("G1V") ? 0 : 1;
    setup su = {34900, 60, 11, 22, 10.0f, sh};
    run("G1a", su, c_g1_feather);
    g_tick_hook = NULL; g_quiet = 0; g_rec_on = 0; g_pl_wish = 0; g_g1_unseen = 0;
    int bad = 0, tall_ok = 0;
    for (int i = 0; i < g1_ndn; i++) {
        const auto &d = g1_dns[i];
        const bool tallest = d.b == d.a - 1 || d.next_rpm < d.floor_;   // one step, or the gear above the landing is under the rule's floor at the speed the shift ends at
        printf("  %5.1f s  %2d>%-2d at %4.1f km/h  lands %4.0f rpm (the gear above: %4.0f, the rule's floor %4.0f)%s\n", d.t, d.a, d.b, d.kmh, d.land, d.next_rpm, d.floor_, tallest ? "" : "  <- a taller gear holds the floor");
        if (!tallest) bad++; else tall_ok++;
    }
    if (!must_tall) {
        // the guard: a brake nobody pressed keeps the layers' landing: it must not be turned into the service-brake-only landing (that is the owner's foot)
        g1_verdict(g1_ndn > 0 && bad > 0, "a brake the telemetry does not show keeps the layers' landing (%d braking downshifts, %d of them higher than the service-brake floor needs)", g1_ndn, bad);
    } else {
        g1_verdict(bad == 0, "every braking downshift on the feathering pedal takes the tallest gear that holds the floor (%d downshifts, %d not)", g1_ndn, bad);
    }
}
static void g1_a_rows()
{
    g1_a_scene("the pedal feathers", 0.0f, true);
    g1_a_scene("the same braking, but nobody's foot (AEB: the brake is not in the telemetry)", 0.39f, false);
}


// ---- b: the route layer is off in the gearbox
// Every scene runs three times from the same round time (the caches the layers read are keyed to the absolute time): ON = [trueshift] lookahead 1 with the route; OFF = lookahead 0 (the default) with
// the route; NONE = lookahead 0 and no route at all. OFF must be NONE to the last shift, and OFF must not ask the route anything (the reads of the sim's route stubs: 0). ON must differ from NONE in
// the scenes that are about the route (the rows have teeth: the route code is still there and still reached with the key on).
static float g1_r_desc(double s) { return s < 600 ? 0.0f : s < 2600 ? -4.0f : 0.0f; }     // a flat, then 2 km of -4 %
static float g1_r_wall(double s) { return s < 500 ? 0.0f : s < 1500 ? 7.0f : 0.0f; }     // a flat, then 1 km of 7 % (an upshift on the flat that the climb ahead would take back)
static float g1_r_gentle(double s) { return s < 300 ? 0.0f : -1.5f; }                      // a gentle descent ahead (the roll's forecast)
static void c_g1_hold80(double) { hold(80); M.brake = 0; sim_pedal_wish(); }
static void c_g1_full(double) { M.pedal = 1; M.brake = 0; sim_pedal_wish(); }
static void c_g1_coast(double t) { if (t < 6) hold(80); else M.pedal = 0; M.brake = 0; sim_pedal_wish(); }   // 80 km/h, then the foot off
static void c_g1_planner(double t) { if (t < 4) hold(80); else M.pedal = 0; M.brake = 0; if (t >= 4) ts_neutral_request(1, 6.0f); sim_pedal_wish(); }   // the planner wants neutral from 4 s
static void c_g1_cc(double t) { M.brake = 0; cruise(80); sim_pedal_wish(); }                                 // the game's cruise at 80 on a climb the route knows
static double g_g1_need_max;
static void g1_need_track(double) { if (g_pcc_need_kmh == g_pcc_need_kmh && g_pcc_need_kmh > g_g1_need_max) g_g1_need_max = g_pcc_need_kmh; }
struct g1_sc { const char *name; float mass, kmh; int g0; double secs; float cvar; float (*road)(double); void (*ctl)(double); int eco_roll; int cruise; int teeth; };
static const g1_sc G1_SC[] = {
    { "40 t holds 80 km/h, a 4 % climb the route knows from 1.5 km (R1b)",              40000, 80, 11, 150, 3.0f,    r_climb,     c_g1_hold80,     0, 0, 1 },
    { "40 t full pedal from 55 km/h up a 5 % climb to a top at 700 m (R2b)",            40000, 55, 10, 70,  3.0f,    r_crest,     c_g1_full,  0, 0, 0 },
    { "40 t full pedal from 40 km/h, 1 km of 7 % from 500 m (the plan's worst stretch: no upshift into a climb its landing gear cannot pull)", 40000, 40, 8, 60, 3.0f, g1_r_wall, c_g1_full, 0, 0, 1 },
    { "40 t holds 80, pedal off at 6 s, 2 km of -4 % from 600 m (the descent control)",  40000, 80, 12, 60,  3.0f,    g1_r_desc,   c_g1_coast,        0, 0, 0 },
    { "26 t Eco, the game's cruise at 80 on rolling hills (a roll with the cruise only with a route that confirms it, R5)", 26000, 80, 12, 240, 10.0f, r_hills, c_cc80, 1, 1, 1 },
    { "40 t Eco, the game's cruise at 80, a 4 % climb at 1.5 km (the climb the cruise is told about)", 40000, 80, 12, 150, 10.0f, r_climb,    c_g1_cc,           0, 1, 1 },
    { "40 t, the planner asks for neutral from 4 s on the flat (eco-roll on)",          40000, 80, 12, 40,  3.0f,    r_flat,      c_g1_planner,      1, 0, 1 },
};
struct g1_out { std::string sig; long reads; double need; };
static g1_out g1_b_run(const g1_sc &c, int lookahead, int route)
{
    use_volvo(); lb_begin();
    set_param("lookahead", (float)lookahead); set_param("eco_roll", (float)c.eco_roll);
    g_cruise_smooth = c.cruise; g_pcc_on = c.cruise;
    t_now = ceil(t_now / 100.0) * 100.0 + 100.0;
    g_g1_need_max = 0; g_pcc_need_kmh = NAN;
    setup su = { c.mass, c.kmh, c.g0, c.secs, c.cvar, 0.5f, 0, -1, -1, c.road, route };
    g_rt_reads = 0; memset(g_route_reads, 0, sizeof g_route_reads);
    lb_run("G1b", su, c.ctl, 1, g1_need_track, 0);
    long reads = g_rt_reads; for (int i = 0; i < RR_N; i++) reads += g_route_reads[i];
    std::string q; char b[96];
    for (int i = 0; i < g_rec_n; i++) { snprintf(b, sizeof b, "%s%d>%d@%.2f", i ? " " : "", g_rec[i].a, g_rec[i].b, lround((g_rec[i].t - g_lb_t0) * 60.0) / 60.0); q += b; }   // (whole ticks: the sum of 1/60 s steps is not exact)
    snprintf(b, sizeof b, " | %d rolls %.1f s | end %.1f km/h in %d | need %.1f", rolls, roll_time, M.v * 3.6f, M.gear, g_g1_need_max); q += b;
    g_cruise_smooth = 0; g_pcc_on = 0;
    return { q, reads, g_g1_need_max };
}
static void g1_b_rows()
{
    printf("\n=== G1 b: [trueshift] lookahead 0 (the default) cuts the route out of the gearbox: a route present changes no gear ===\n");
    int teeth = 0, toothless = 0, wanted = 0, scenes = 0, same = 0, silent = 0;
    for (const g1_sc &c : G1_SC) {
        g1_b_run(c, 0, 0);   // (a primer: the first drive of a scene after another carries the state of that other one for a few ticks, the later ones do not)
        const g1_out none = g1_b_run(c, 0, 0), off = g1_b_run(c, 0, 1), on = g1_b_run(c, 1, 1);
        scenes++; wanted += c.teeth;
        const bool eq = off.sig == none.sig, quiet = off.reads == 0, tooth = on.sig != none.sig;
        if (eq) same++;
        if (quiet) silent++;
        if (tooth) teeth++;
        if (c.teeth && !tooth) toothless++;
        printf("  %s\n", c.name);
        printf("      key on : %s   (%ld route reads)\n      key off: %s   (%ld route reads)\n      no route: %s\n", on.sig.c_str(), on.reads, off.sig.c_str(), off.reads, none.sig.c_str());
        g1_verdict(eq && quiet, "with the key at 0 the route changes nothing here (same drive as without a route: %s; route reads with the key off: %ld)", eq ? "yes" : "NO", off.reads);
    }
    g1_verdict(same == scenes && silent == scenes, "the key at 0 cuts the route out of the gearbox in all %d scenes (identical to no route in %d, silent in %d)", scenes, same, silent);
    g1_verdict(toothless == 0, "with the key at 1 the scenes that are about the route differ from the no-route drive: %d of %d (the route code is still reached; %d scenes in all differ)", wanted - toothless, wanted, teeth);
}

static int run_g1()
{
    printf("\n--- G1 ---");
    g1_fail = 0;
    g1_a_rows();
    g1_b_rows();
    use_volvo();
    return g1_fail;
}
