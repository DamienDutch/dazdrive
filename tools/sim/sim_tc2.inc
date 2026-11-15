// WP-TC2 rows (run: tools/sim/run_tc2.sh = sim.cpp with TC2=1; included by sim.cpp after sim_tc1.inc, which it builds on: the Spectrans truck, tc_check, the recorder). The converter's stall
// torque ratio is READ from the game's transmission object (live RE of 05-11, ETS2 1.61: stall_torque_ratio at the shift time - 8, differential_ratio at - 4; the Spectrans Scania's
// Allison 4000 holds 2.42, 3.91, 1.0 there) and trusted only where the diff beside it equals the truck config's. A trusted figure over 1 declares the converter at once (V.tc, V.tc_stall,
// fixed); 1 (a clutch box) declares nothing and keeps the file of the last drive out; an untrusted read leaves everything to TC1 (the file, the slip detection, the learner).
// The sim's gb_engine stub reads a fake transmission object (g_fake_tr, fake_tr_set) with the same engine_read_stall as game.cpp, so the rows run the real path.
#pragma once

static int g_tc2_lines; static char g_tc2_line[240];
static void tc2_tap(const char *l)
{
    if (!strncmp(l, "transmission: stall torque ratio", 32)) { g_tc2_lines++; snprintf(g_tc2_line, sizeof g_tc2_line, "%s", l); }
}

// ---- item 1: the read and its layout check, on the bytes as dumped
static void tc2_read_rows()
{
    printf("\n=== TC2 item 1: the stall ratio from the transmission object, trusted by the diff beside it ===\n");
    char tr[0x240];
    auto put = [&](float stall, float diff, float shift) { memset(tr, 0, sizeof tr); memcpy(tr + 0x1c0, &stall, 4); memcpy(tr + 0x1c4, &diff, 4); memcpy(tr + 0x1c8, &shift, 4); };
    put(2.42f, 3.91f, 1.0f);   // the dump of this morning: +0x1c0 2.42, +0x1c4 3.91, +0x1c8 1.0
    float s = trans_stall_read(tr, 0x1c8, 3.91f);
    tc_check(s == 2.42f, "the bytes as dumped (2.42, 3.91, shift time 1.0), the truck config's diff 3.91: the stall reads %.2f, trusted (expect 2.42)", s);
    engine_info e; memset(&e, 0, sizeof e);
    engine_read_stall(&e, tr, 0x1c8, 3.91f);
    tc_check(e.tc_stall == 2.42f, "engine_info.tc_stall %.2f (expect 2.42)", e.tc_stall);
    s = trans_stall_read(tr, 0x1c8, 3.914f);
    tc_check(s == 2.42f, "the config's diff 3.914 (0.004 off, inside 0.005): still trusted (%.2f)", s);
    s = trans_stall_read(tr, 0x1c8, 3.916f);
    tc_check(s != s, "the config's diff 3.916 (0.006 off): not trusted (%.2f, expect NAN)", s);
    s = trans_stall_read(tr, 0x1c8, 3.70f);
    tc_check(s != s, "a mismatching diff (the object 3.91, the truck config 3.70): NAN (%.2f)", s);
    memset(&e, 0, sizeof e); e.tc_stall = 7; engine_read_stall(&e, tr, 0x1c8, 3.70f);
    tc_check(e.tc_stall == 0.0f, "engine_info.tc_stall for the mismatch: %.2f (expect 0 = not read)", e.tc_stall);
    s = trans_stall_read(tr, 0x1c8, 0.0f);
    tc_check(s != s, "the truck config has no diff yet (0): NAN (%.2f)", s);
    char z[0x240]; memset(z, 0, sizeof z);
    s = trans_stall_read(z, 0x1c8, 0.0f);
    tc_check(s != s, "a zeroed object against an unknown diff (0 = 0): NAN, not a match (%.2f)", s);
    s = trans_stall_read(z, 0x1c8, 3.91f);
    tc_check(s != s, "a zeroed object against the config's 3.91: NAN (%.2f)", s);
    s = trans_stall_read(NULL, 0x1c8, 3.91f);
    tc_check(s != s, "no transmission object (not readable): NAN (%.2f)", s);
    s = trans_stall_read(tr, 0, 3.91f);
    tc_check(s != s, "a layout without the shift time (offset 0): NAN (%.2f)", s);
    put(0.0f, 3.91f, 0.8f); memset(&e, 0, sizeof e); engine_read_stall(&e, tr, 0x1c8, 3.91f);
    tc_check(trans_stall_read(tr, 0x1c8, 3.91f) == 0.0f && e.tc_stall == 1.0f, "a clutch box (stall 0, diff checks): trusted, engine_info.tc_stall %.2f (expect 1 = no converter)", e.tc_stall);
    put(1.0f, 3.91f, 0.8f); memset(&e, 0, sizeof e); engine_read_stall(&e, tr, 0x1c8, 3.91f);
    tc_check(e.tc_stall == 1.0f, "a stall of exactly 1 (diff checks): engine_info.tc_stall %.2f (expect 1 = no converter)", e.tc_stall);
    put(NAN, 3.91f, 0.8f); memset(&e, 0, sizeof e); engine_read_stall(&e, tr, 0x1c8, 3.91f);
    tc_check(e.tc_stall == 0.0f, "a NAN where the stall should be (diff checks): not read, engine_info.tc_stall %.2f (expect 0)", e.tc_stall);
    put(-2.0f, 3.91f, 0.8f); memset(&e, 0, sizeof e); engine_read_stall(&e, tr, 0x1c8, 3.91f);
    tc_check(e.tc_stall == 0.0f, "a negative stall (diff checks): not read, engine_info.tc_stall %.2f (expect 0)", e.tc_stall);
    memset(&e, 0, sizeof e);
    tc_check(e.tc_stall == 0.0f, "a zeroed engine_info says 'not read' (%.2f): every other stub, every sim truck, is as before", e.tc_stall);
}

// ---- item 1/2: the line, once per truck; the declaration from the first metre
static int g_tc2_tc_t0; static float g_tc2_stall_t0; static int g_tc2_fixed_t0, g_tc2_lock_t0; static double g_tc2_decl_t;
static void c_tc2_calm(double t)   // pedal 0.2 on the flat, as the TC1 launch rows; notes the converter's state at the first tick and when it first shows
{
    if (t < 0.001) { g_tc2_tc_t0 = V.tc; g_tc2_stall_t0 = V.tc_stall; g_tc2_fixed_t0 = V.tc_stall_fixed; g_tc2_lock_t0 = V.tc_lock_gear; g_tc2_decl_t = V.tc ? 0.0 : -1.0; }
    else if (g_tc2_decl_t < 0 && V.tc) g_tc2_decl_t = t;
    M.grade_pct = 0; M.pedal = 0.2f;
}
static void tc2_line_rows()
{
    printf("\n=== TC2 item 1: ONE log line per truck; the converter is declared from the first metre ===\n");
    // the Spectrans, the plant has the converter (2.42), the plugin knows nothing of it but what the transmission object says
    sim_reset(RESET_HARD, "sim: tc2 line");
    use_tc_spectrans(false, false, 2, 2.42f);
    fake_tr_set(2.42f, DIFF, 1.0f);
    g_sim_log_tap = tc2_tap; g_tc2_lines = 0; g_tc2_line[0] = 0;
    run("Spectrans 17 t, a calm launch from rest (pedal 0.2), 40 s: the engine is read at the start and again at 30 s", { 17000, 0.0f, 1, 40 }, c_tc2_calm);
    g_sim_log_tap = NULL;
    tc_check(g_tc2_lines == 1 && !strcmp(g_tc2_line, "transmission: stall torque ratio 2.42 (diff 3.91 checks)"), "%d line (expect 1, the brief's text): \"%s\"", g_tc2_lines, g_tc2_line);
    tc_check(g_tc2_tc_t0 == 1 && g_tc2_stall_t0 == 2.42f && g_tc2_fixed_t0 == 1 && V.tc == 1 && V.tc_stall == 2.42f, "the converter is declared before the first metre: tc %d, stall %.2f, fixed %d (expect 1, 2.42, 1); at the end tc %d, stall %.2f", g_tc2_tc_t0, g_tc2_stall_t0, g_tc2_fixed_t0, V.tc, V.tc_stall);
    // a mismatching object: one line saying why, nothing declared at the start
    sim_reset(RESET_HARD, "sim: tc2 line mismatch");
    use_tc_spectrans(false, false, 2, 2.42f);
    fake_tr_set(2.42f, 3.70f, 1.0f);
    g_sim_log_tap = tc2_tap; g_tc2_lines = 0; g_tc2_line[0] = 0;
    run("Spectrans 17 t, the same, the object's diff reads 3.70 (another layout)", { 17000, 0.0f, 1, 40 }, c_tc2_calm);
    g_sim_log_tap = NULL;
    tc_check(g_tc2_lines == 1 && strstr(g_tc2_line, "not trusted"), "%d line (expect 1, saying why): \"%s\"", g_tc2_lines, g_tc2_line);
    tc_check(g_tc2_tc_t0 == 0 && g_tc2_stall_t0 == 0.0f && !g_tc2_fixed_t0, "nothing is declared at the start (tc %d, stall %.2f, fixed %d; expect 0, 0, 0): TC1's detection is the fallback", g_tc2_tc_t0, g_tc2_stall_t0, g_tc2_fixed_t0);
    // no line while the truck config has not told its diff; the line comes with the diff
    sim_reset(RESET_HARD, "sim: tc2 no diff yet");
    use_tc_spectrans(false, false, 2, 2.42f);
    fake_tr_set(2.42f, DIFF, 1.0f);
    g_sim_log_tap = tc2_tap; g_tc2_lines = 0; g_tc2_line[0] = 0;
    T.diff = 0; V.eng_at = -1e9; vehicle_update(SELF);
    const int lines0 = g_tc2_lines;
    T.diff = DIFF; T.cfg_gen++; vehicle_update(SELF);
    g_sim_log_tap = NULL;
    tc_check(lines0 == 0 && g_tc2_lines == 1 && V.eng.tc_stall == 2.42f, "the config's diff not known yet: %d lines; with it (and the config event's re-read): %d line, tc_stall %.2f (expect 0, 1, 2.42)", lines0, g_tc2_lines, V.eng.tc_stall);
    // a clutch box: trusted 0, one line, no converter
    sim_reset(RESET_HARD, "sim: tc2 clutch box");
    use_ato();
    fake_tr_set(0.0f, DIFF, 0.8f);
    g_sim_log_tap = tc2_tap; g_tc2_lines = 0; g_tc2_line[0] = 0;
    run("Renault T (a clutch box), the object holds stall 0 and its diff", { 17000, 0.0f, 1, 40 }, c_tc2_calm);
    g_sim_log_tap = NULL;
    tc_check(g_tc2_lines == 1 && strstr(g_tc2_line, "no converter"), "%d line (expect 1): \"%s\"", g_tc2_lines, g_tc2_line);
    tc_check(V.tc == 0 && V.eng.tc_stall == 1.0f && hunts == 0, "no converter declared (tc %d, engine tc_stall %.2f; expect 0, 1), %d hunting pairs", V.tc, V.eng.tc_stall, hunts);
    g_fake_tr_on = 0;
}

// ---- item 2: who wins, and what stays the fallback
static void tc2_prec_rows()
{
    printf("\n=== TC2 item 2: the ini, the game's figure, the file of the last drive, the learner ===\n");
    // 1. the ini's figure wins over the read
    sim_reset(RESET_HARD, "sim: tc2 ini");
    use_tc_spectrans(false, false, 2, 2.42f);
    fake_tr_set(2.42f, DIFF, 1.0f);
    set_param("tc_stall", 2.10f);
    run("Spectrans 17 t, calm launch, the ini sets the stall ratio to 2.10, the game says 2.42", { 17000, 0.0f, 1, 10 }, c_tc2_calm);
    set_param("tc_stall", 0);
    tc_check(V.tc == 1 && V.tc_stall == 2.10f && V.tc_stall_fixed, "the ini's 2.10 wins (tc_stall %.2f, fixed %d)", V.tc_stall, V.tc_stall_fixed);
    // 2. the game's figure is not learned over: ten launches at a 0.5 pedal move a learned figure (TC1: 2.0 -> 2.32 on the same plant), this one stays exactly where the game put it
    for (int i = 0; i < 24; i++) g_tc1_stall_at[i] = 0;
    g_tc1_launches = 0;
    sim_reset(RESET_HARD, "sim: tc2 fixed");
    use_tc_spectrans(false, true, 1, 2.42f);
    fake_tr_set(2.42f, DIFF, 1.0f);
    g_ped = 0.5f;
    run("Spectrans 20 t, launches at a 0.5 pedal, the game says 2.42", { 20000, 0, 1, 20 * 10 }, c_tc1_cycles);
    bool same = g_tc1_launches > 3; for (int i = 0; i < g_tc1_launches; i++) if (g_tc1_stall_at[i] != 2.42f) same = false;
    tc_check(same && V.tc_stall == 2.42f && V.tc == 1, "10 launches: the stall ratio is 2.42 at the end of every one (%d seen) and now %.2f (a learned figure moves)", g_tc1_launches, V.tc_stall);
    tc_check(hunts == 0 && g_n_incidents == 0, "%d hunting pairs, %d incidents (expect none)", hunts, g_n_incidents);
    // 3. a file of the last drive declared the converter first (the lock gear is the file's), then the read lands: the figure is the game's, the lock gear stays
    sim_reset(RESET_HARD, "sim: tc2 file then read");
    use_tc_spectrans(false, false, 2, 2.42f);
    vehicle_tc_stored_prime(2.0f, 2);
    fake_tr_set(2.42f, DIFF, 1.0f);
    run("Spectrans 17 t, the file of the last drive says 2.00 / lock gear 2, the game says 2.42", { 17000, 0.0f, 1, 10 }, c_tc2_calm);
    tc_check(g_tc2_tc_t0 == 1 && g_tc2_stall_t0 == 2.42f && g_tc2_fixed_t0 && g_tc2_lock_t0 == 2, "the game's figure over the file's, the file's lock gear kept: at the first metre stall %.2f fixed %d, lock gear %d (expect 2.42, 1, 2; the lock-gear learner may lower it later, TC1: now %d)", g_tc2_stall_t0, g_tc2_fixed_t0, g_tc2_lock_t0, V.tc_lock_gear);
    // 4. a trusted 1 (a clutch box) keeps the file out: no converter from it; the slip detection is still the fallback (a plant that does slip)
    sim_reset(RESET_HARD, "sim: tc2 trusted one");
    use_ato();
    vehicle_tc_stored_prime(2.2f, 1);
    fake_tr_set(1.0f, DIFF, 0.8f);
    run("Renault T (a clutch box), a stale file says converter 2.20, the object says stall 1.0", { 17000, 0.0f, 1, 10 }, c_tc2_calm);
    tc_check(V.tc == 0, "a trusted 1 keeps the file's converter out (tc %d, expect 0)", V.tc);
    sim_reset(RESET_HARD, "sim: tc2 trusted one, slipping plant");
    use_tc_spectrans(false, true, 1, 2.42f);
    vehicle_tc_stored_prime(0, 0);
    fake_tr_set(1.0f, DIFF, 1.0f);
    for (int i = 0; i < 24; i++) g_tc1_stall_at[i] = 0;
    g_tc1_launches = 0; g_ped = 0.5f;
    run("Spectrans 20 t, the plant slips like a converter, the object says stall 1.0 (trusted)", { 20000, 0, 1, 20 * 6 }, c_tc1_cycles);
    int first = 0; while (first < g_tc1_launches && g_tc1_stall_at[first] == 0) first++;
    tc_check(V.tc == 1 && !V.tc_stall_fixed && first <= 3, "the slip detection is still the fallback: found by launch %d (tc %d, fixed %d; expect found within 3, not fixed)", first + 1, V.tc, V.tc_stall_fixed);
    g_fake_tr_on = 0;
    g_mod_engine = 0;
}

// ---- item 3: the ride's first launch, with and without the read
// Cells: the ride's two launches that the converter shaped, 17 t (the first launch of the drive, pedal 0.11) and 41.5 t (the 620 s launch, pedal 0.23), each on the flat and on the ride's -0.5 %;
// and the same four at a firm 0.5 pedal.
// Without: the plant has the converter (2.42), the plugin does not know it (a first drive: no file) and has to find it from the slip. With: the object says 2.42 (the layout checks).
static float g_tc2_ped, g_tc2_grade;
static double g_tc2_fnd_t; static float g_tc2_fnd_kmh;
static void c_tc2_cell(double t)
{
    if (t < 0.001) { g_tc2_fnd_t = V.tc ? 0.0 : -1.0; g_tc2_fnd_kmh = 0; }
    else if (g_tc2_fnd_t < 0 && V.tc) { g_tc2_fnd_t = t; g_tc2_fnd_kmh = M.v * 3.6f; }
    M.grade_pct = g_tc2_grade; M.pedal = g_tc2_ped;
}
struct tc2_cell { float mass, pedal, grade; };
static const int TC2_NCELL = 8;
static tc2_cell g_tc2_cells[8] = { { 17000, 0.11f, 0.0f }, { 17000, 0.11f, -0.5f }, { 41500, 0.23f, 0.0f }, { 41500, 0.23f, -0.5f },   // the ride's own pedals (0.11 the first launch, 0.23 the 620 s one)
                                   { 17000, 0.50f, 0.0f }, { 17000, 0.50f, -0.5f }, { 41500, 0.50f, 0.0f }, { 41500, 0.50f, -0.5f } };  // and a firm pedal, where the plant leaves the slip before it has run a second
static float g_tc2_up_kmh[8][2]; static int g_tc2_up_rpm[8][2]; static bool g_tc2_never[8];
static void tc2_launch_rows()
{
    printf("\n=== TC2 item 3: the first launch of a drive, without and with the read (the plugin knows the converter from the object, not from slip) ===\n");
    for (int c = 0; c < TC2_NCELL; c++) {
        for (int with = 0; with < 2; with++) {
            sim_reset(RESET_HARD, "sim: tc2 launch");
            use_tc_spectrans(false, false, 2, 2.42f);   // (the plant has the converter, the plugin the ratios; no file, no lock gear)
            vehicle_tc_stored_prime(0, 0);
            if (with) fake_tr_set(2.42f, DIFF, 1.0f);
            g_tc2_ped = g_tc2_cells[c].pedal; g_tc2_grade = g_tc2_cells[c].grade;
            g_rec_on = 1; g_rec_n = 0;
            char nm[200]; snprintf(nm, sizeof nm, "Spectrans %.1f t, %s %.1f %%, pedal %.2f from rest, %s", g_tc2_cells[c].mass / 1000, g_tc2_cells[c].grade ? "grade" : "flat", g_tc2_cells[c].grade, g_tc2_cells[c].pedal, with ? "WITH the read" : "without it");
            run(nm, { g_tc2_cells[c].mass, 0.0f, 1, 40 }, c_tc2_cell);
            g_rec_on = 0;
            float kmh = 0; int rpm = 0, g2 = 0;
            for (int i = 0; i < g_rec_n; i++) if (g_rec[i].a == 1 && g_rec[i].b > 1) { kmh = g_rec[i].kmh; rpm = (int)g_rec[i].rpm; g2 = g_rec[i].b; break; }
            g_tc2_up_kmh[c][with] = kmh; g_tc2_up_rpm[c][with] = rpm;
            printf("        first upshift out of 1st: %.1f km/h, %d rpm (to gear %d); converter %s\n", kmh, rpm, g2,
                   g_tc2_fnd_t < 0 ? "not found in this run" : g_tc2_fnd_t == 0 ? "known at the first metre" : (snprintf(nm, sizeof nm, "found at %.1f s (%.1f km/h)", g_tc2_fnd_t, g_tc2_fnd_kmh), nm));
            if (!with) g_tc2_never[c] = g_tc2_fnd_t < 0;
            if (with) tc_check(hunts == 0 && g_n_incidents == 0 && g_tc2_fnd_t == 0.0, "  %d hunting pairs, %d incidents, the converter known at the first metre (%s)", hunts, g_n_incidents, g_tc2_fnd_t == 0.0 ? "yes" : "NO");
        }
        // What a read changes in the first upshift is what the slip detection has not yet told by then: where the plugin found the converter before the first shift (the ride's pedals: 2.4 .. 2.5 s,
        // 5 km/h, the shift at 7 .. 11) the shift is the same; where it never found it in the 40 s (17 t at a firm pedal: the plant leaves the slip before a second has run) the read is the difference,
        // and there the box must leave 1st earlier. Everywhere the first upshift is under the game's own (1400 .. 1500 rpm in the ride: TC1's yardstick).
        tc_check(g_tc2_up_kmh[c][1] > 0 && g_tc2_up_rpm[c][1] < 1400 && (!g_tc2_never[c] || g_tc2_up_kmh[c][1] < g_tc2_up_kmh[c][0]),
                 "%.1f t, %.1f %%, pedal %.2f: 1 > 2 at %.1f km/h / %d rpm with the read, %.1f km/h / %d rpm without%s (expect under 1400 rpm; earlier where the converter was never found without)",
                 g_tc2_cells[c].mass / 1000, g_tc2_cells[c].grade, g_tc2_cells[c].pedal, g_tc2_up_kmh[c][1], g_tc2_up_rpm[c][1], g_tc2_up_kmh[c][0], g_tc2_up_rpm[c][0], g_tc2_never[c] ? " (converter never found)" : "");
    }
    g_fake_tr_on = 0;
}

static int run_tc2()
{
    g_tc_failed = 0;
    tc2_read_rows();
    tc2_line_rows();
    tc2_prec_rows();
    tc2_launch_rows();
    printf("\nTC2: %s\n", g_tc_failed ? "FAILED" : "ok");
    return g_tc_failed;
}
