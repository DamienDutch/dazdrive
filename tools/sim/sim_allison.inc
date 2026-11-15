// R0611-ALLISON rows (run: tools/sim/run_allison.sh = sim.cpp with ALLISON=1; included by sim.cpp before main). The MAN with the Allison of the ride of 06-11 (ETS2 1.62, 14.6 t, Eco,
// notes/ride-0611/: 6 speeds 4.70 .. 0.67, diff 3.70, wheel 0.538, 2100 Nm, limit 2100, the converter's stall ratio 2.42, lock gear 1 learned at 112 s). The owner: "the Allison shifts almost
// instantly, but it felt as if there is a penalty for shifting just one gear instead of several". The ride: of the 10 skips 3 > 5 and 2 > 4 under a pull or the game's cruise, 9 were
// undone within 3 s (347 s 3 > 5 > 4 > 3 > 4 in four seconds), the 4 > 6 skips of the highway all stood. These two sim rows are GUARDS: this plant takes single steps and passes before and after; the ride's
// moment that failed before is the replay row, allison_rows.py (the pedal of the 200 s pull as recorded). Every row is a plain check (tc_check, ok / FAIL with the numbers).
// The plant: the gears over the lock gear run locked (TC_PLANT_LOCK = 1: the ride's engine sat 1.02 .. 1.18 over the driveline in 2nd .. 6th, 1.0 is the near side of that; the other
// converter rows keep the plant that slips everywhere).
#pragma once

static void use_tc_man_allison()
{
    tc_reset();
    float r[16] = { 4.70f, 2.21f, 1.53f, 1.00f, 0.76f, 0.67f };
    float c[7][2] = { { 0, 0 }, { 440, 0.5f }, { 930, 1 }, { 1400, 1 }, { 1800, 0.81f }, { 1900, 0.75f }, { 2100, 0 } }; NCURVE = 7;   // the log: "engine curve: 0: 0.00, 440: 0.50, 930: 1.00 ..."
    memcpy(RATIO, r, sizeof r); memcpy(CURVE, c, sizeof c);
    float g[4][2] = { { 1500, 2100 }, { 700, 1500 }, { 1000, 1350 }, { 1400, 1900 } };   // brake, low, high, power (the log: ranges brake 1500-2100 low 700-1500 high 1000-1350 power 1400-1900)
    memcpy(RANGES, g, sizeof g);
    TORQUE = 2100; CDA = 6; LIMIT = 2100; DIFF = 3.70f; RADIUS = 0.538f; NG = 6;
    T.fwd_gears = 6; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
    tc_set(2.42f, 1, 1190);   // the plugin has the converter from the file of the last drive (stall rpm: its own derivation, 0.85 x the power range's bottom)
    TC_PLANT_LOCK = 1;
}

struct al_res { int skips, undone, ups, downs; };
// the skips the recorder saw (an upshift of 2 or more gears) and how many of them a downshift took back within 3 s
static al_res al_count()
{
    al_res o = { 0, 0, 0, 0 };
    for (int i = 0; i < g_rec_n; i++) {
        if (g_rec[i].b > g_rec[i].a) o.ups++; else o.downs++;
        if (g_rec[i].b - g_rec[i].a >= 2) {
            o.skips++;
            for (int j = i + 1; j < g_rec_n && g_rec[j].t - g_rec[i].t < 3.0; j++) if (g_rec[j].b < g_rec[j].a) { o.undone++; break; }
        }
    }
    return o;
}
static void al_list(const char *what)
{
    printf("        %s:", what);
    for (int i = 0; i < g_rec_n && i < 14; i++) printf(" %.1fs %d>%d@%.0fkm/h/%.0frpm", g_rec[i].t, g_rec[i].a, g_rec[i].b, g_rec[i].kmh, g_rec[i].rpm);
    printf("\n");
}
// 1. the pull of 200 s: 40 km/h in 2nd, a 0.55 pedal, flat
static void c_al_pull(double) { M.grade_pct = 0.5f; M.pedal = 0.55f; }
// 2. the game's cruise raised to 83 from 55 km/h in 3rd (347 s, 467 s, 483 s): the cruise's own throttle pulls, the foot is off
static void c_al_ramp(double t) { M.grade_pct = 0.5f; hold_cc(83, t); }
static void al_row(const char *name, setup su, void (*ctl)(double), int want_skips_min, bool must_stand)
{
    sim_reset(RESET_HARD, "sim: allison");
    use_tc_man_allison();
    g_idle_sim = 550; set_param("idle_rpm", 550);
    su.cvar = 10; su.shift_time = 1.0f;   // Eco (g_adaptive_shift = 10); the game's default shift time, as the ride's log has it
    g_rec_on = 1; g_rec_n = 0;
    run(name, su, ctl);
    g_rec_on = 0;
    const al_res a = al_count();
    al_list("shifts");
    tc_check(a.undone == 0, "%s: %d skips, %d of them taken back within 3 s (the ride: 9 of 10 3 > 5 / 2 > 4 skips were; expect 0)", name, a.skips, a.undone);
    tc_check(hunts == 0 && g_n_incidents == 0, "%s: %d hunting pairs, %d incidents (expect none)", name, hunts, g_n_incidents);
    if (must_stand) tc_check(a.skips >= want_skips_min && M.gear == 6, "%s: the highway skip stays: %d skips (expect >= %d), end in gear %d (expect 6)", name, a.skips, want_skips_min, M.gear);
}
static int run_allison()
{
    g_tc_failed = 0;
    layers_begin();   // the context of the part-18 rows: idle 550, the driver profile fed (a fresh session: the uniform prior), the shipped keys
    printf("\n=== R0611 Allison: the box shifts under power; a skip must not be one the pull takes back ===\n");
    al_row("MAN Allison 14.6 t, Eco, pedal 0.55 from 25 km/h in 2nd", { 14600, 25.0f, 2, 40 }, c_al_pull, 0, false);
    al_row("MAN Allison 14.6 t, Eco, the cruise raised to 83 from 55 km/h in 3rd", { 14600, 55.0f, 3, 50 }, c_al_ramp, 0, false);
    layers_end();
    printf("\nAllison: %s\n", g_tc_failed ? "FAILED" : "ok");
    return g_tc_failed;
}
