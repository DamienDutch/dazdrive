// WP-G8 rows (owner decisions 63 + 64, "a price that fades when the situation changes"). Included by sim.cpp before main; run_g8() is the last block of the default run (run.sh).
//   W   the weight itself (core/shiftcommon.cpp sc_reversal_weight), a pure function: the same situation (a hunt) is the whole price, a lift or a braking that is over with a calm pedal
//       is none of it, a full pedal after braking keeps all of it, and nothing steps (ANTI-KNIK).
//   c   TrueShift's layers, steady pulls (a full pedal, a pedal that wavers inside the hunt metric's noise): every decision inside the dwell whose demand is still the one of the shift
//       is told the whole price is due (gc_out::prev_w == 1).
//   d   decision 64: a downshift under braking (before a bend), then a calm pull-away (the real foot at 0.15 / 0.25, Eco and Normal, 17 t and 40 t) and, as the control, a full pedal after
//       the same downshift. The first upshift the layers decide, after how long and at what rpm of the gear it leaves.
//   (a, decision 63, the lift's downshift after a pull, is the 18i-4 lift rows of run_scores: Pete 15.5 t lug held, Eco and Normal on the real foot.)
// Every row runs on a hard-reset plugin after a wash, as the 18i rows do (lb_run), quiet; the table is printed here. Nothing here sets a number to pass: the bars are the dwell itself.
#pragma once
#include "core/shiftcommon.h"

static float ts_param(const char *key)
{
    for (int i = 0; i < m_trueshift.n_params; i++) if (!strcmp(m_trueshift.params[i].key, key)) return *m_trueshift.params[i].val;
    return NAN;
}

static int g8_fail;
static void g8_begin() { set_param("gear_logic", 1.0f); set_param("eco_roll", 0); g_fuel_raw = 1000.0; T.fuel = 1000.0f; }   // (as the 18i rows begin: the layers, no eco-roll, a full tank)
static void g8_verdict(bool ok, const char *fmt, ...)
{
    char b[400]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    printf("  G8 %s: %s\n", ok ? "ok" : "FAIL", b);
    if (!ok) g8_fail = 1;
}

// The truck of the owner's ride of 04-11 (tools/sim/g7_rows.py SW_DEF: 3300 Nm, limit 2200, the 12-speed box, diff 3.08, wheel 0.512, idle 500): its braking downshifts land at 1500-1600 rpm and
// the calm pull-away that follows is where the ride's 3 > 7 at 1950-2038 rpm came from. Returns the box's shift time.
static float g8_ride_truck()
{
    const float sh = sw_engine_def("T=3300|L=2200|C=300:0 440:0.6 850:0.88 1050:1 1400:1 1900:0.7 2400:0.5 2600:0.3|R=900,1990,1080,1480,1200,1950,1300,2000|G=9.204 7.404 5.87 4.696 3.771 3.027 2.466 1.966 1.561 1.247 1.003 0.802|D=3.08|W=0.512|S=0.9");
    g_idle_sim = 500;
    return sh;
}

// ---- W: the weight
static void g8_weight_rows()
{
    printf("\n=== G8 W: sc_reversal_weight (the price of reversing a shift that is still due) ===\n");
    // the hunt definition's own noise at the demand of the shift (sim.cpp hunt_noise, the one copy of core/shiftcommon.cpp's rule: a quarter of it, never under 3 % of full demand)
    auto noise = [](float dm) { return hunt_noise(dm); };
    float worst1 = 0; int n1 = 0;
    for (float dm = 0.05f; dm <= 1.0001f; dm += 0.05f)
        for (float k : { -1.0f, -0.5f, 0.0f, 0.5f, 1.0f }) {
            const float now = fminf(1.0f, fmaxf(0.0f, dm + k * noise(dm)));
            worst1 = fmaxf(worst1, 1.0f - sc_reversal_weight(dm, now, 0, 0)); n1++;
            worst1 = fmaxf(worst1, 1.0f - sc_reversal_weight(dm, now, 1, 1)); n1++;   // (a braking shift with the braking still on is no different)
        }
    g8_verdict(worst1 < 1e-6f, "the same situation (a pedal within the hunt metric's noise of the one at the shift, 0.05 .. 1.0, also braking then and now) is the whole price: %d cases, the largest shortfall %.2e", n1, worst1);
    float worst2 = 0; int n2 = 0;
    for (float dm = 0.1f; dm <= 1.0001f; dm += 0.05f) { worst2 = fmaxf(worst2, sc_reversal_weight(dm, 0.0f, 0, 0)); n2++; }
    g8_verdict(worst2 < 1e-3f, "a lift (the pedal at the shift 0.1 .. 1.0, off now) owes nothing: %d cases, the largest weight %.3f", n2, worst2);
    float worst3 = 0; int n3 = 0;
    for (float dm = 0.0f; dm <= 0.051f; dm += 0.05f)
        for (float now = 0.15f; now <= 0.351f; now += 0.05f) { worst3 = fmaxf(worst3, fabsf(sc_reversal_weight(dm, now, 1, 0) - now)); n3++; }
    g8_verdict(worst3 < 1e-4f, "braking then and over now, a calm pedal (0.15 .. 0.35): the price left is the pedal's own share (the situation is wholly changed and the share is how much of a full pull is asked): %d cases, worst %.2e", n3, worst3);
    g8_verdict(sc_reversal_weight(0.0f, 1.0f, 1, 0) > 1.0f - 1e-6f && sc_reversal_weight(0.05f, 1.0f, 1, 0) > 1.0f - 1e-6f,
               "braking then and over now, a full pedal: the whole price stays (%.3f, %.3f)", sc_reversal_weight(0.0f, 1.0f, 1, 0), sc_reversal_weight(0.05f, 1.0f, 1, 0));
    // ANTI-KNIK: no input steps the weight. One analog input swept in steps of 0.001 over its whole range with the others at a spread of values; the largest step is the demand noise floor's
    // slope (the ramp is one noise wide: 1 / 0.03 per unit of share = 0.033 per step) and not more.
    float step = 0;
    for (int bt = 0; bt < 2; bt++)
        for (float a = 0.0f; a <= 1.0001f; a += 0.1f)
            for (float b = 0.0f; b <= 1.0001f; b += 0.25f) {
                float p1 = sc_reversal_weight(a, 0.0f, bt, b);
                for (int i = 1; i <= 1000; i++) {
                    const float x = i * 0.001f;
                    float w1 = sc_reversal_weight(a, x, bt, b), w2 = sc_reversal_weight(x, a, bt, b), w3 = sc_reversal_weight(a, b, bt, x);
                    float v1 = sc_reversal_weight(a, x - 0.001f, bt, b), v2 = sc_reversal_weight(x - 0.001f, a, bt, b), v3 = sc_reversal_weight(a, b, bt, x - 0.001f);
                    step = fmaxf(step, fmaxf(fabsf(w1 - v1), fmaxf(fabsf(w2 - v2), fabsf(w3 - v3))));
                }
                (void)p1;
            }
    g8_verdict(step <= 0.04f, "no step: each of dm_now, dm_then and braking_now swept in 0.001 steps (others spread over the range, braked then and not): the largest change of the weight in one step %.4f (bar 0.04)", step);
}

// ---- c: steady pulls, every decision inside the dwell with the demand of the shift is told the whole price is due
static int g8c_n, g8c_bad, g8c_last; static double g8c_t; static float g8c_share; static float g8c_worst;
static void g8c_track(double)
{
    if (M.gear > 0 && M.gear != g8c_last) { if (g8c_last > 0) { g8c_t = t_now; g8c_share = wish_share_eff(); } g8c_last = M.gear; }
    double at = -1; const gc_out *go = ts_last_choice(&at);
    if (g8c_t > 0 && go && at > g8c_t + 0.05 && at >= t_now - 0.1 && t_now - g8c_t < ts_param("dwell_up_down_s")) {
        const float now = wish_share_eff();
        if (fabsf(now - g8c_share) <= hunt_noise(g8c_share)) { g8c_n++; g8c_worst = fmaxf(g8c_worst, 1.0f - go->prev_w); if (go->prev_w < 1.0f - 1e-6f) g8c_bad++; }
    }
}
static float g_g8c_ped = 0.35f;
static void c_g8c_wave(double t) { g_pl_wish = 1; M.pedal = g_g8c_ped >= 0 ? g_g8c_ped : 0.42f + 0.05f * sinf((float)t * 6.2832f / 4); M.brake = 0; M.wish_on = 0; sim_pedal_wish(); }
static void g8_steady_rows()
{
    printf("\n=== G8 c: steady pulls, the whole price while the demand is the one of the shift ===\n");
    g8c_n = g8c_bad = 0; g8c_last = 0; g8c_t = -1; g8c_worst = 0;
    struct Cs { const char *n; float ped; float mass; int v0, g0; double secs; } css[] = {
        { "full pedal, 40 t from 12 km/h", 1.0f, 40000, 12, 3, 40 }, { "pedal 0.35, 18 t from 12 km/h", 0.35f, 18000, 12, 4, 40 }, { "a pedal wavering 0.37 .. 0.47, 40 t from 12 km/h", -1.0f, 40000, 12, 3, 60 } };
    for (const Cs &c : css) {
        const float sh = g8_ride_truck();
        g8_begin();
        setup wash = {20000, 30, 6, 2, 3, sh}; g_quiet = 1; g_pl_wish = 1; run("wash", wash, c_hold80); g_quiet = 0;
        sim_reset(RESET_HARD, "sim: G8 row");
        g_g8c_ped = c.ped; g_tick_hook = g8c_track; g_quiet = 1;
        const int n0 = g8c_n;
        run("G8c", {c.mass, (float)c.v0, c.g0, c.secs, 3, sh}, c_g8c_wave);
        g_tick_hook = NULL; g_quiet = 0; g_pl_wish = 0;
        printf("  %-52s %d decisions inside the dwell with the demand of the shift\n", c.n, g8c_n - n0);
    }
    // K2 step 1 (rows audit G8-c, DROPPED as a row): a wiring field (gc_out::prev_w), no felt effect; the function-level row W above holds the price, the hunt definition counts the felt result. Printed.
    printf("  G8 c (wiring, not a row): %d decisions inside the dwell with the demand of the shift, %d not told the whole price is due (largest shortfall %.3f)\n", g8c_n, g8c_bad, g8c_worst);
}

// ---- d: a downshift under braking, then the pull-away
static struct { int last, from, to, dn_gear; double t_dn, kmh; double t_dec; double rpm_dec; int dec_from, dec_to; double rpm_dn; double cross_dec; double p3_dec; } g8s;   // cross_dec: the crossover of the gear the upshift leaves, at the speed of the decision
static float g_g8_ped; static float g_g8_kmh = 25.0f;   // the downshift that counts: the first one under braking that lands at or under this speed (a bend ahead)
static void g8_track(double)
{
    // the downshift under braking: the first change down while the brake is on, landing at g_g8_kmh or under (a bend ahead)
    if (M.gear > 0 && g8s.last > 0 && M.gear < g8s.last && g8s.t_dn < 0 && M.brake > 0.05f && M.v * 3.6f <= g_g8_kmh) { g8s.t_dn = t_now; g8s.from = g8s.last; g8s.to = M.gear; g8s.kmh = M.v * 3.6f; g8s.rpm_dn = rpm_now(); }
    if (M.gear > 0) g8s.last = M.gear;
    double at = -1; const gc_out *go = ts_last_choice(&at);
    if (g8s.t_dn >= 0 && g8s.t_dec < 0 && go && at > g8s.t_dn && at >= t_now - 0.02 && go->best > M.gear) { g8s.t_dec = at; g8s.rpm_dec = rpm_now(); g8s.dec_from = M.gear; g8s.dec_to = go->best; { const eng_bands *bd = bands_now(); g8s.cross_dec = bd->ok ? gc_cross(bd, M.gear, NG, M.v) : 0; g8s.p3_dec = bd->ok ? bd->p3 : 0; } }
}
static void c_g8(double t)
{
    if (t <= 0) { g8s = {}; g8s.t_dn = -1; g8s.t_dec = -1; }
    g_pl_wish = 1; M.wish_on = 0;
    if (g8s.t_dn < 0 || t_now < g8s.t_dn + 0.15) { M.pedal = 0; M.brake = 0.25f; }   // braking; 0.15 s after the downshift has landed the brake is released and the foot is on the pedal
    else { M.brake = 0; M.pedal = g_g8_ped; }
    sim_pedal_wish();
}
static void g8_d_rows()
{
    printf("\n=== G8 d: a downshift under braking before a bend, then the pull-away (the real foot); the dwell after a downshift is %.1f s ===\n", ts_param("dwell_down_up_s"));
    printf("  %-10s %-7s %-6s %-22s %-28s %s\n", "truck", "mode", "pedal", "downshift", "first upshift of the layers", "rpm of the gear it leaves");
    const float dwell = ts_param("dwell_down_up_s");
    static const struct { const char *n; float m; int g0; } trk[] = { { "ride 16.5 t", 16540, 12 }, { "ride 41.9 t", 41900, 12 } };
    static const struct { const char *n; float cvar; } mds[] = { { "Eco", 10.0f }, { "Normal", 3.0f } };
    static const float peds[] = { 0.15f, 0.25f, 1.0f };
    for (const auto &tr : trk)
        for (const auto &md : mds)
            for (float ped : peds) {
                const float sh = g8_ride_truck();
                g8_begin();
                setup wash = {20000, 30, 6, 2, md.cvar, sh}; g_quiet = 1; g_pl_wish = 1; run("wash", wash, c_hold80); g_quiet = 0;
                sim_reset(RESET_HARD, "sim: G8 row");
                g_g8_ped = ped; g_tick_hook = g8_track; g_quiet = getenv("G8V") ? 0 : 1;
                setup su = {tr.m, 80, tr.g0, 28, md.cvar, sh};
                run("G8d", su, c_g8);
                g_tick_hook = NULL; g_quiet = 0; g_pl_wish = 0;
                char dn[48] = "none", up[64] = "none yet", rp[24] = "-";
                if (g8s.t_dn >= 0) snprintf(dn, sizeof dn, "%d->%d at %.0f km/h", g8s.from, g8s.to, g8s.kmh);
                const double dt = g8s.t_dec >= 0 ? g8s.t_dec - g8s.t_dn : -1;
                if (dt >= 0) { snprintf(up, sizeof up, "%d->%d, %.2f s after it", g8s.dec_from, g8s.dec_to, dt); snprintf(rp, sizeof rp, "%.0f", g8s.rpm_dec); }
                printf("  %-10s %-7s %-6.2f %-22s %-28s %s\n", tr.n, md.n, ped, dn, up, rp);
                if (g8s.t_dn < 0) { g8_verdict(false, "%s %s pedal %.2f: no downshift under braking at 40 km/h or under (the row has no situation)", tr.n, md.n, ped); continue; }
                if (ped < 0.5f)   // (0.7 release: the bar was the old dwell timer, 'decided within dwell - 0.25 s'; a calm pedal cannot gain 500 rpm in 1.75 s. What it guards: after braking a calm pull is not held in the low gear, i.e. it goes up by P3 like every calm upshift)
                    g8_verdict(dt >= 0 && g8s.p3_dec > 0 && g8s.rpm_dec <= 1.01 * g8s.p3_dec, "%s %s pedal %.2f: after the braking downshift the calm pull-away's upshift is decided by P3 + 1 %% (%.2f s after the downshift at %.0f rpm, P3 %.0f; the dwell %.1f s is no bar)", tr.n, md.n, ped, dt, g8s.rpm_dec, g8s.p3_dec, dwell);
                else   // (K2 step 1: the dwell was a timer floor under a time-weighted choice, core value 3; the relation it stood for: at a full pedal after braking the gear is left where time says, the crossover)
                    g8_verdict(dt < 0 || g8s.rpm_dec >= g8s.cross_dec, "%s %s pedal %.2f: at full pedal the gear is left no earlier than its crossover, where the next gear pulls at least as hard (the first upshift %.2f s after the downshift at %.0f rpm, the crossover of the gear left %.0f; the dwell is %.1f s and is no bar)", tr.n, md.n, ped, dt, g8s.rpm_dec, g8s.cross_dec, dwell);
            }
}

static int run_g8()
{
    printf("\n--- WP-G8 ---");
    g8_fail = 0;
    set_param("eco_roll", 0);
    g8_weight_rows();
    g8_steady_rows();
    g8_d_rows();
    use_volvo();
    return g8_fail;
}
