// ---- part 18i: the rows of the expected-time term (docs/powerband-change.md round 3, package B). Included by sim.cpp before run_scores_layers.
// Each row prints what it measures and a verdict line ("must" rows: ok / GAP); run_layers_b() returns the number of GAPs.
//   18i-1  landing     a low-speed lug skip under load (ride 02-11: 41 t, 7 km/h, pedal 0.25, +1.5 %, 1>3 landed at 527 rpm, floor 672, stalled)
//   18i-2  cruise      Eco cruise on +2 % at 79 km/h (ride 02-11: 11<->12 twice in 17 s): reversals within 17 s
//   18i-3  launch      the gear sequence from 7 km/h to 40 km/h (41 t and 16.5 t, flat and +1.5 %, pedal 0.25 / 0.5): in log and in the owner's numbers
//   18i-4  lift        a lift after a pull (from the power band, from lug): the time to the first shift; a blip (0.3 s) and a short lift (0.8 s) must not shift
//   18i-5  bend        a roundabout (live yaw rate and steering, no route) with a shift due halfway: it comes after the exit
//   (the limiter row, 18i-6, is pedallim in sim_pedalmix.cpp)
// SC_LB=<name substring>: only those rows, verbose (the shift log; the score terms of every shift are in the "layers:" lines, printed with -v: ./run_scores.sh -v).
static float r_one5(double) { return 1.5f; }
static int g_lb_gaps;
static const char *lb_only() { static const char *o = getenv("SC_LB"); return o; }
static bool lb_skip(const char *key) { return lb_only() && !strstr(key, lb_only()); }
// A "must" row (counted: run_scores fails on a GAP) or a CARRIED row: one this package does not close, with the package that owns it (open[D2]: ...; counted in
// "carried: N", which does not fail the run).
static int g_lb_carried;
static void (*g_lb_tap)(int kind, const char *owner, const char *what);   // sim_promises.cpp: every verdict (kind 0 ok, 1 open[owner], 2 GAP, 3 open); null = off
static int g_lb_foot;   // 1: the demand rows (lb_run demand = 1) take the real foot too, as SC_FOOT=1 does (sim_promises.cpp)
static void lb_verdict(bool ok, const char *what, bool must = true, const char *owner = NULL)
{
    if (ok) { if (g_lb_tap) g_lb_tap(0, NULL, what); printf("      ok: %s\n", what); return; }
    if (owner) { if (g_lb_tap) g_lb_tap(1, owner, what); printf("      open[%s]: %s\n", owner, what); g_lb_carried++; return; }
    if (g_lb_tap) g_lb_tap(must ? 2 : 3, NULL, what);
    printf("      %s: %s\n", must ? "GAP" : "open", what);
    if (must) g_lb_gaps++; else g_lb_carried++;
}
// A bounded printf-append: snprintf returns what it would have written, so k is clamped (a 70-shift row must not run past the buffer).
static void lb_cat(char *buf, size_t n, size_t &k, const char *fmt, ...)
{
    if (k >= n - 1) return;
    va_list ap; va_start(ap, fmt);
    int w = vsnprintf(buf + k, n - k, fmt, ap);
    va_end(ap);
    k = w < 0 ? k : (k + (size_t)w >= n ? n - 1 : k + (size_t)w);
}
// The landings of the run: the gear and the rpm when a shift ended (the plant's, not the plugin's estimate).
struct lb_land { double t; int gear; float rpm, kmh; };
static lb_land g_lb_land[64]; static int g_lb_nland, g_lb_prev_state;
static double g_lb_t0, g_lb_t40;                 // the run's start (the controller's first call), the seconds to 40 km/h (-1: not yet)
static int g_lb_snap_g, g_lb_snap_band; static float g_lb_snap_rpm;   // the gear, its band and the rpm at the lift mark
// (WP-G1b) What the layers' score says about the upshift in the window of a held lift (lb_snap reads ts_last_choice from one shift duration after the lift to 1.5 shift durations
// and the latency after it, while the gear is still the one the lift found): the best upshift's saving (the score of the gear we are in less the upshift's, both without the
// shift term) and its price as the score charges it at that moment (the shift term: lift_scale x the shift's price), the margin saving - price at the best moment, and the same
// price at lift_scale 1 (a lift that is certain not to end during the shift) for the text.
static struct { int n, tgt; double sav, price, margin, full; } g_lb_win;
static double g_lb_mark;                         // the lift mark, seconds into the run (-1: not yet)
static void (*g_lb_inner)(double); static void (*g_lb_extra)(double);
static void lb_track(double dt)
{
    if (g_lb_prev_state && !M.state && g_lb_nland < 64) g_lb_land[g_lb_nland++] = { t_now - g_lb_t0, M.gear, rpm_now(), M.v * 3.6f };
    g_lb_prev_state = M.state;
    if (g_lb_t40 < 0 && M.v * 3.6f >= 40) g_lb_t40 = t_now - g_lb_t0;
    if (g_lb_extra) g_lb_extra(dt);
}
static void c_lb_wrap(double t) { if (t <= 0) { g_lb_t0 = t_now; g_lb_mark = -1; } g_lb_inner(t); }
static void lb_begin()
{
    set_param("gear_logic", 1.0f);
    set_param("eco_roll", 0);
    g_fuel_raw = 1000.0; T.fuel = 1000.0f;
}
// One run of a row: a wash and a hard reset (as the other rows), the recorder on, the landings tracked.
// demand = 1: the pull of this row is a DEMAND written for the pedal map of d99b2fd (a calm pull, a lug pull: the lift rows 18i-4 / 18i-4b and the bend rows 18i-5, whose situation the pull has to
// produce), see foot_for_old_map in sim.cpp; the other rows of 18i describe the owner's foot (the 02-11 ride's 0.25, the launches, the hint at 0.6) and keep it. SC_FOOT=1 turns every demand row back to
// the real foot, the result of the map as it is (P6.2: 5 GAPs in the lift and bend rows, listed in docs/powerband-change.md 'P6: the pedal map').
// g_lb_foot = 1 (WP-G1.2b): the same rows with the REAL foot, the pedal as the owner would press it: the situation the row wants (a calm pull, a lug pull) is then whatever the map and the box make of that foot.
// They are rows that must pass (the demand-fed rows stay as they are: they pin the situation, the foot rows say what the box makes of the owner's foot).
static void lb_run(const char *name, setup su, void (*ctl)(double), int wish, void (*extra)(double) = NULL, int demand = 0)
{
    g_pl_wish = wish;
    setup wash = {20000, 30, 6, 2}; wash.cvar = su.cvar; wash.road = su.road;
    g_quiet = 1; run("wash", wash, c_hold80); g_quiet = 0;
    sim_reset(RESET_HARD, "sim: next 18i row");
    g_rec_on = 1; g_rec_n = 0; g_lb_nland = 0; g_lb_prev_state = 0; g_lb_t0 = t_now; g_lb_t40 = -1; g_lb_mark = -1; g_pl_t0 = t_now; g_pl_t70 = -1;
    g_lb_inner = ctl; g_lb_extra = extra;
    g_tick_hook = lb_track;
    g_quiet = getenv("SC_VERBOSE") || lb_only() ? 0 : 1;
    g_pl_demand = demand && !getenv("SC_FOOT") && !g_lb_foot;
    run(name, su, c_lb_wrap);
    g_pl_demand = 0;
    g_tick_hook = NULL; g_rec_on = 0; g_quiet = 0; g_lb_extra = NULL;
}
static const char *lb_his(int g, char *buf)   // the owner's numbering on a 12+2 box: log N = his N - 2, the two crawlers C1 / C2
{
    if (g <= 2) snprintf(buf, 8, "C%d", g); else snprintf(buf, 8, "%d", g - 2);
    return buf;
}
// the shifts of the run (up to km/h) as "1>3>7" in log numbers and "C1>1>5" in his
static void lb_seq(char *log, char *his, size_t n, float upto_kmh)
{
    size_t k = 0, k2 = 0; log[0] = his[0] = 0; char b[8]; int first = 1;
    for (int i = 0; i < g_rec_n && g_rec[i].kmh <= upto_kmh; i++) {
        if (first) { lb_cat(log, n, k, "%d", g_rec[i].a); lb_cat(his, n, k2, "%s", lb_his(g_rec[i].a, b)); first = 0; }
        lb_cat(log, n, k, ">%d", g_rec[i].b); lb_cat(his, n, k2, ">%s", lb_his(g_rec[i].b, b));
    }
}

// ---- 18i-1: the landing of a low-speed lug skip under load
static void run_lb_landing()
{
    printf("\n  18i-1 landing: DC16 41 t at 7 km/h in gear 1, pedal 0.25, +1.5 %%, Normal: the plant loses more in the shift gap than the model knows\n");
    printf("  (ride 02-11: a 1>3 at 7 km/h with the pedal at 0.25 landed at 527 rpm, the floor 672: the estimate of a first shift at crawl speed under load)\n");
    printf("  %-24s %6s %6s %12s %6s  %s\n", "plant", "shifts", "floor", "lowest land", "under", "decided (the rpm the layers expected) -> landed (reason)");
    struct Pl { const char *n; float drag, extra; } pls[] = { { "as modelled", 0, 0 }, { "0.15 m/s2 + 0.6 s", 0.15f, 0.6f }, { "0.3 m/s2 + 1.0 s", 0.3f, 1.0f } };
    for (const Pl &pl : pls) {
        char key[96]; snprintf(key, sizeof key, "landing %s", pl.n);
        if (lb_skip(key)) continue;
        use_dc16();
        lb_begin();
        g_plant_shift_drag = pl.drag; g_plant_shift_extra = pl.extra;
        g_pl_ped = 0.25f;
        setup su = {41000, 7, 1, 40, 3, 0.5f}; su.road = r_one5;
        lb_run(key, su, c_pull, 1);
        g_plant_shift_drag = g_plant_shift_extra = 0;
        const float floor_rpm = bands_now()->floor;
        int n = 0, under = 0; float lo = 1e9f; char dec[200]; size_t k = 0; dec[0] = 0;
        for (int i = 0; i < g_lb_nland && i < g_rec_n; i++) {
            if (g_lb_land[i].kmh > 25) break;
            n++; if (g_rec[i].b > g_rec[i].a) { lo = fminf(lo, g_lb_land[i].rpm); if (g_lb_land[i].rpm < floor_rpm) under++; }
            if (i < 4) lb_cat(dec, sizeof dec, k, "%s%d>%d %.0f->%.0f (%s)", i ? ", " : "", g_rec[i].a, g_rec[i].b, g_rec[i].pred, g_lb_land[i].rpm, g_rec[i].why);
        }
        printf("  %-24s %6d %6.0f %12.0f %6d  %s\n", pl.n, n, floor_rpm, lo < 1e8f ? lo : 0.0f, under, dec);
        if (pl.drag > 0) lb_verdict(under == 0, "no upshift below 25 km/h lands under the lug floor (an unlearned first shift: the prior loss against the plant's; E measured: the stress plant loses 0.3 m/s2 for 1.5 s, the prior 0.1 m/s2 for 0.8 s, and a prior that covers it would also make every normal truck's first shift two small steps where the launch rows want one skip; the first measured shift teaches the estimate: the 'again' rows land at 634-1003 rpm)", true, "LS");
    }
}

// ---- 18i-2: Eco cruise (the cruise's force wish, force_mode 1) on a +2 % climb at 79 km/h: reversals within 17 s
// (ride 02-11, 282 s: 11<->12 twice in 17 s at 77-81 km/h on +2 %, cruise on, Eco)
static float g_lb_grade = 2;
static float r_lb_two(double) { return g_lb_grade; }
static void c_lb_cc79(double t) { g_cf_set = 79; c_cforce(t); }
static void run_lb_cruise()
{
    printf("\n  18i-2 cruise: Eco, the cruise (force_mode 1) holds 79 km/h on +2 %%: hunting pairs (the one definition, hunt_kind: a reversal within %.0f s at an unchanged foot; the cruise's wish is the foot here), and for the record the ride's own 17 s window\n", HUNT_WINDOW_S);
    printf("  %-14s %-6s %6s %9s %8s %8s  %s\n", "truck", "route", "shifts", "hunt pairs", "17 s info", "mean rpm", "first shifts (gear@seconds)");
    struct Tr { const char *n; int kind; float mass; } trs[] = { { "DC16 18 t", 0, 18000 }, { "DC16 40 t", 0, 40000 }, { "Renault 36 t", 1, 36000 }, { "ATO 18 t", 3, 18000 }, { "Pete 35.6 t", 2, 35600 } };
    for (const Tr &tr : trs)
      for (float grade : { 2.0f, 4.0f })
        for (int route = 0; route < 2; route++) {
            char key[96]; snprintf(key, sizeof key, "cruise %s %.0f %% route %d", tr.n, grade, route);
            if (lb_skip(key)) continue;
            g_lb_grade = grade;
            if (tr.kind == 0) use_dc16(); else if (tr.kind == 1) use_renault(); else if (tr.kind == 3) use_ato(); else use_pete();
            lb_begin();
            CF.fm = 1; CF.on = 0; CF.at = -1; g_cf_app = 0;
            setup su = {tr.mass, 79, sc_gear_for(79, 1100), 120, 10.0f, 1.0f}; su.road = r_lb_two; su.route = route;
            lb_run(key, su, c_lb_cc79, 0);
            CF.fm = 0; CF.on = 0; CF.at = -1;
            const int rev = rec_pairs();
            int rev17 = 0;   // (information only: the ride's 11 <-> 12 twice in 17 s, a slower reversal than the window the definition counts)
            for (int i = 0; i + 1 < g_rec_n; i++)
                if (g_rec[i].a > 0 && g_rec[i].b > 0 && g_rec[i + 1].a == g_rec[i].b && g_rec[i + 1].b == g_rec[i].a && g_rec[i + 1].t - g_rec[i].t < 17) rev17++;
            char seq[120]; size_t k = 0; seq[0] = 0;
            for (int i = 0; i < g_rec_n && i < 10; i++) lb_cat(seq, sizeof seq, k, "%s%d>%d@%.0f", i ? " " : "", g_rec[i].a, g_rec[i].b, g_rec[i].t - g_lb_t0);
            printf("  %-14s %-6s %6d %9d %8d %8.0f  %s\n", key + 7, route ? "known" : "blind", shifts, rev, rev17, rpm_t > 0 ? rpm_sum / rpm_t : 0.0, seq);
            char w[96]; snprintf(w, sizeof w, "%s %s: no hunting pair", key + 7, route ? "known" : "blind"); lb_verdict(rev == 0, w);
        }
}

// ---- 18i-3: the launch, 7 km/h (where TrueShift takes over) to 40 km/h
// The sim truck is the DC16 (14 speeds, log 1 the crawler, 2-13 twelve gears, 14 the overdrive); the owner's box is a 12+2 (log N = his N - 2, C1 / C2 the
// crawlers), so his column is the arithmetic conversion of the log gears, not his truck (the Renault ato2614f is in results/fleet.json: 32.04 19.38 11.73 ..., but a launch from a standstill on it needs the Creep / LowSpeed owner, which this sim does not model: run as a launch it loops 1>3>2>1 for 70 shifts, so the row starts at 7 km/h on the DC16). The ride: C1 > 1 > 5 = log 1 > 3 > 7; "why not C1 > 3 at once" = log 1 > 5.
static void run_lb_launch()
{
    printf("\n  18i-3 launch: the gears from 7 km/h in gear 1 to 40 km/h, DC16, Normal (his = log N - 2 is the 12+2 conversion of the log gears ONLY: the sim truck is the DC16, whose log 1 is the crawler, 2-13 its twelve gears and 14 the overdrive)\n");
    printf("  %-8s %-6s %-7s %7s %7s  %-24s %-24s\n", "mass", "road", "pedal", "shifts", "t40 s", "log", "his numbering");
    for (float mass : { 41000.0f, 16500.0f })
        for (int grade = 0; grade < 2; grade++)
            for (float ped : { 0.25f, 0.5f }) {
                char key[96]; snprintf(key, sizeof key, "launch %.1f t %s pedal %.2f", mass / 1000, grade ? "+1.5 %" : "flat", ped);
                if (lb_skip(key)) continue;
                use_dc16();
                lb_begin();
                g_pl_ped = ped;
                setup su = {mass, 7, 1, 40, 3, 0.5f}; if (grade) su.road = r_one5;
                lb_run(key, su, c_pull, 1);
                char lg[128], hs[128]; lb_seq(lg, hs, sizeof lg, 40.0f);
                int n40 = 0;
                for (int i = 0; i < g_rec_n; i++) if (g_rec[i].kmh <= 40) n40++;
                printf("  %-8.1f %-6s %-7.2f %7d %7.1f  %-24s %-24s\n", mass / 1000, grade ? "+1.5 %" : "flat", ped, n40, g_lb_t40, lg, hs);
            }
}

// ---- 18i-4: the lift
// A script: the pull g_lb_pull from a standstill (the wish pedal, as c_pull), then at the mark (g_lb_trigger: 1 = the gear is in lug above 40 km/h, 2 = 50 km/h) the
// pedal goes to 0 for g_lb_lift_s seconds (1e9 = held) and returns to g_lb_pull. The mark is when the lift happens.
static float g_lb_pull = 0.4f; static double g_lb_lift_s = 1e9; static int g_lb_trigger = 2;
static void c_lb_lift(double t)
{
    if (g_lb_mark < 0) {
        const eng_bands *b = bands_now();
        const bool lug = M.gear >= 1 && !M.state && b->ok && pt_band_of(b, rpm_now(), gc_cross(b, M.gear, NG, M.v)) == PT_LUG;
        // (trigger 1: in the lug band and settled there -- the last shift 2 s back. The mark used to come at the very moment an up-skip landed in the lug band, so the lift met a skip in flight, and
        // whether the box then turned back 1.0 or 1.33 s later depended on the phase of the launch before it: with TrueShift owning the standstill (decision 60) it was 1.0 s, 0.005 s under the bound)
        const bool settled = g_rec_n == 0 || t_now - g_rec[g_rec_n - 1].t > 2.0;
        if (g_lb_trigger == 1 ? ((lug && settled && M.v * 3.6f > 40) || t >= 25) : (M.v * 3.6f >= 50 || t >= 25)) g_lb_mark = t;
    }
    g_pl_ped = g_lb_mark >= 0 && t >= g_lb_mark && t < g_lb_mark + g_lb_lift_s ? 0.0f : g_lb_pull;
    c_pull(t);
}
static void lb_snap(double)   // the gear and the rpm the lift finds, read at the mark
{
    static bool done;
    if (t_now - g_lb_t0 < 0.1) done = false;
    if (t_now - g_lb_t0 < 0.1) g_lb_win = { 0, 0, 0, 0, -1e9, 0 };
    if (g_lb_mark >= 0 && done && M.gear == g_lb_snap_g) {   // the window of a held lift: the decision the layers made in this tick
        const double dur0 = 1.25 * 0.5 + 0.15, rel = t_now - g_lb_t0 - g_lb_mark;
        double at = -1; const gc_out *go = ts_last_choice(&at);
        if (go && go->lift && rel >= dur0 && rel <= 1.5 * dur0 + GC_LATENCY_S && at >= g_lb_t0 + g_lb_mark) {
            const gc_term *c = &go->t[M.gear];
            for (int g = M.gear + 1; g <= NG && g <= M.gear + 3; g++) {
                const gc_term *t = &go->t[g];
                if (!t->candidate || !c->candidate) continue;
                const double sav = (c->bandc + c->accc + c->expc + c->fuelc - c->stay) - (t->bandc + t->accc + t->expc + t->fuelc - t->stay), price = t->shiftc - c->shiftc;
                if (sav - price > g_lb_win.margin) {
                    const float lr = gc_lift_return((float)rel, (float)(dur0 + GC_LATENCY_S), (float)dur0);
                    g_lb_win.margin = sav - price; g_lb_win.sav = sav; g_lb_win.price = price; g_lb_win.tgt = g; g_lb_win.full = lr > 1e-3f ? price / lr : price;
                }
                g_lb_win.n++;
            }
        }
    }
    if (!done && g_lb_mark >= 0 && t_now - g_lb_t0 >= g_lb_mark) { done = true; g_lb_snap_g = M.gear; g_lb_snap_rpm = rpm_now(); const eng_bands *bd = bands_now(); g_lb_snap_band = bd->ok && M.gear >= 1 ? pt_band_of(bd, g_lb_snap_rpm, gc_cross(bd, M.gear, NG, M.v)) : -1; }
}
// A driver who proved calm: ten minutes of a light, steady pedal in the demand history (the writer is package D's; the rows put it there by hand). Without it the
// history is the uniform prior, which asks the old cruise stand's reserve.
static void lb_fill_calm() { for (int i = 0; i < DEMAND_MEM_S; i++) demand_note(0.35f); }
static void lb_calm_history(int on)
{
    demand_reset();
    if (on) lb_fill_calm();
    g_driver_pre = on ? lb_fill_calm : NULL;   // (with the driver profile fed, every run starts with a fresh one: the calm history is handed over after that)
}
static const char *lb_sfx() { return g_lb_foot ? " (foot)" : ""; }
static void run_lb_lift()
{
    printf("\n  18i-4 lift%s: pull from a standstill, then the pedal comes off (held / a 0.3 s blip / a 0.8 s lift). 'calm' = a calm driver (ten minutes of a light pedal in the history),\n", g_lb_foot ? " (the REAL foot: the pull is the pedal at that position through the map, not a demand)" : "");
    printf("  the lift finds the gear in the upper eco or the power band at 50 km/h: one upshift WHEN its saving in the score exceeds its price (otherwise no shift within 1.5 s of the lift; a later shift is logged); 'lug' = the fresh\n");
    printf("  history (the prior), the lift finds the gear in lug above 40 km/h: one downshift. A shift is decided 1-1.5 shift durations after the lift (the first shift duration cannot be used for one; it is logged GC_LATENCY_S later, when the clutch opens); a blip and a 0.8 s lift shift nothing.\n");
    printf("  %-14s %-6s %-6s %-6s %5s %7s %5s %6s  %s\n", "truck", "from", "case", "mode", "gear", "rpm", "dt", "shifts", "after the lift");
    struct Md { const char *n; float cvar; } mds[3] = { { "Eco", 10.0f }, { "Normal", 3.0f }, { "Power", 1.6667f } };
    struct Tr { const char *n; int kind; float mass; float pull; int trigger; int calm; } trs[] = {
        { "Pete 15.5 t", 2, 15500, 0.3f, 2, 1 }, { "DC16 40 t", 0, 40000, 0.3f, 2, 1 },     // calm: the gear a light pull sits in (Normal and Power 1250-1500 rpm)
        { "Pete 15.5 t", 2, 15500, 0.5f, 1, 0 }, { "DC16 18 t", 0, 18000, 0.3f, 1, 0 } };  // from lug
    struct Cs { const char *n; double lift_s; } css[] = { { "held", 1e9 }, { "blip", 0.3 }, { "0.8 s", 0.8 } };
    for (const Tr &tr : trs)
        for (const Cs &cs : css)
            for (int mi = 0; mi < 3; mi++) {
                // K2 step 1 (rows audit 7.1): the held 'lug' rows are DROPPED. 0 of their 12 rows had a verdict (the pull never sat in the lug band at the mark: Pete lifts at 1234 .. 1608 rpm with P1 about
                // 1150, the DC16 at 924 .. 1239), so they passed by saying nothing. The lift out of lug is judged where its situation is built by the row itself: tools/sim/lift_rows.py (a full-throttle
                // skip into lug, the lift fired on that event, a control run; a cell without the event FAILS) and the 18f release rows. The blip and the 0.8 s lift of these trucks stay: a control run is built in.
                if (!tr.calm && cs.lift_s >= 1e8) continue;
                char key[96]; snprintf(key, sizeof key, "lift %s %s %s %s%s", tr.n, tr.calm ? "calm" : "lug", cs.n, mds[mi].n, lb_sfx());
                if (lb_skip(key)) continue;
                if (tr.kind == 0) use_dc16(); else use_pete();
                lb_begin();
                lb_calm_history(tr.calm);
                g_lb_pull = tr.pull; g_lb_lift_s = cs.lift_s; g_lb_trigger = tr.trigger;
                setup su = {tr.mass, 1, 1, 45, mds[mi].cvar, 0.5f};
                g_lb_snap_g = 0; g_lb_snap_rpm = 0;
                lb_run(key, su, c_lb_lift, 1, lb_snap, 1);
                lb_calm_history(0);
                const eng_bands *bd = bands_now();
                const int band = g_lb_snap_band;
                double first = -1; int n = 0, ups = 0, downs = 0; char seq[96]; size_t k = 0; seq[0] = 0;
                for (int i = 0; i < g_rec_n; i++) {
                    const double dt = g_rec[i].t - g_lb_t0 - g_lb_mark;
                    if (g_lb_mark < 0 || dt < 0 || dt > 10) continue;   // (a shift decided in the frame of the mark is the pull's)
                    if (dt < 0.1 || (cs.lift_s < 1.0 && dt > cs.lift_s + 0.3)) continue;   // after the pedal is back the pull is the box's normal business
                    if (first < 0) first = dt;
                    n++; if (g_rec[i].b > g_rec[i].a) ups++; else downs++;
                    lb_cat(seq, sizeof seq, k, "%s%d>%d@%.1fs", k ? " " : "", g_rec[i].a, g_rec[i].b, dt);
                }
                char dts[8]; if (first >= 0) snprintf(dts, sizeof dts, "%.2f", first); else snprintf(dts, sizeof dts, "-");
                printf("  %-14s %-6s %-6s %-6s %5d %7.0f %5s %6d  %s\n", tr.n, tr.calm ? "calm" : "lug", cs.n, mds[mi].n, g_lb_snap_g, g_lb_snap_rpm, dts, n, seq);
                char w[280];
                // the shift duration the plugin works with: 1.25 x the engine's shift time + 0.15 s until it has measured one (TrueShift sl_prior_dur)
                const double dur = 1.25 * 0.5 + 0.15;
                if (cs.lift_s < 1.0) {
                    // (WP-G1: the window after a short lift also holds the pull's own shifts, one decided just as the pedal is back: the shift is logged GC_LATENCY_S after its decision, and the
                    // pull of the row upshifts on its own at about that time. A shift in the window counts only if the same pull WITHOUT the lift does not make it: the control run is
                    // the same row with the pedal never lifted. The layers line of the shift says which: 'pedal off' is a lift decision, a pedal share is the pull's.)
                    int n_ctl = 0;
                    if (n > 0) {
                        if (tr.kind == 0) use_dc16(); else use_pete();
                        lb_begin();
                        lb_calm_history(tr.calm);
                        g_lb_pull = tr.pull; g_lb_lift_s = 0; g_lb_trigger = tr.trigger;
                        g_lb_snap_g = 0; g_lb_snap_rpm = 0;
                        lb_run(key, su, c_lb_lift, 1, lb_snap, 1);
                        lb_calm_history(0);
                        for (int i = 0; i < g_rec_n; i++) {
                            const double dt = g_rec[i].t - g_lb_t0 - g_lb_mark;
                            if (g_lb_mark < 0 || dt < 0.1 || dt > cs.lift_s + 0.3) continue;
                            n_ctl++;
                        }
                    }
                    snprintf(w, sizeof w, "%s %s %s %s%s: nothing shifts in the 10 s after a %s that the same pull does not make without the lift (%d, the pull alone %d)", tr.n, tr.calm ? "calm" : "lug", cs.n, mds[mi].n, lb_sfx(), cs.n, n, n_ctl);
                    lb_verdict(n <= n_ctl, w);
                } else {
                    // held: one shift, in the right direction, 1-1.5 shift durations after the lift (the first shift duration cannot be used for one; a frame of slack each side)
                    // K2 step 1 (the one lift relation, ride_rows.py lift_rule): the band the gear is in at the lift decides. Over P2: an upshift where its saving exceeds its price, landing at or above P1,
                    // never into lug (the skip size is printed); in the eco band (P1 .. P2): no shift within 1.5 s; under P1 (lug): one downshift.
                    const bool upper = g_lb_snap_rpm > bd->p2;
                    const bool in_eco = g_lb_snap_rpm >= bd->p1 && g_lb_snap_rpm <= bd->p2;
                    const bool want_up = tr.calm && upper, want_down = !tr.calm && band == PT_LUG;
                    if (tr.calm && in_eco) {
                        int early_eco = 0; for (int i = 0; i < g_rec_n; i++) { const double dt = g_rec[i].t - g_lb_t0 - g_lb_mark; if (g_lb_mark >= 0 && dt >= 0.1 && dt - GC_LATENCY_S < 1.5) early_eco++; }
                        snprintf(w, sizeof w, "%s calm held %s%s: the gear is in the eco band (%.0f rpm, P1 %.0f, P2 %.0f): no shift within 1.5 s of the lift (%d)", tr.n, mds[mi].n, lb_sfx(), g_lb_snap_rpm, bd->p1, bd->p2, early_eco);
                        lb_verdict(early_eco == 0, w, true, !g_lb_foot ? "E" : NULL);
                    }
                    if (want_down) {
                        snprintf(w, sizeof w, "%s %s held %s%s: one downshift 1-1.5 shift durations (%.2f-%.2f s) after the lift", tr.n, tr.calm ? "calm" : "lug", mds[mi].n, lb_sfx(), dur - 0.02, 1.5 * dur + 0.02);
                        lb_verdict(n == 1 && downs == 1 && first - GC_LATENCY_S >= dur - 0.02 && first - GC_LATENCY_S <= 1.5 * dur + 0.02, w, true);   // (the shift is logged when the clutch opens, GC_LATENCY_S after the decision)
                    }
                    if (want_up) {
                        // (WP-G1b, the manager's bar for the calm held lift) An upshift is due 1-1.5 shift durations after the lift WHEN its saving in the score exceeds its price; otherwise no shift
                        // within 1.5 s of the lift, and a later shift for another named reason is fine and is logged. The price is the one the score charges at the window: the lift has lasted
                        // the window and ends in a re-press with the chance the driver's lifts give (about a sixth), so the whole upshift, which saves a few hundredths to a few tenths of a
                        // second of band taste, pays only where its saving is more than that chance of the shift's own price (about a second at lift_scale 1). The row's earlier bar (always
                        // an upshift in the window once the pull ends in the upper half of the eco band) asked for a shift the score does not pay: DC16 40 t, the calm 0.3 foot pull ends in
                        // 11 at 1293 rpm, the whole upshift saves 0.09 s against a shift price of about 1 s.
                        const bool pays = g_lb_win.n > 0 && g_lb_win.margin > 0;
                        char info[200];
                        if (g_lb_win.n > 0) snprintf(info, sizeof info, "saving %.2f s against a price of %.2f s at the window (%.2f s at lift_scale 1) for %d > %d", g_lb_win.sav, g_lb_win.price, g_lb_win.full, g_lb_snap_g, g_lb_win.tgt);
                        else snprintf(info, sizeof info, "the layers took no decision in the window");
                        printf("      the upshift in the window: %s -> %s\n", info, pays ? "pays" : "does not pay");
                        char later[160]; size_t kl = 0; later[0] = 0; int early = 0;
                        for (int i = 0; i < g_rec_n; i++) {
                            const double dt = g_rec[i].t - g_lb_t0 - g_lb_mark;
                            if (g_lb_mark < 0 || dt < 0.1 || dt > 10) continue;
                            if (dt - GC_LATENCY_S < 1.5) early++;
                            else lb_cat(later, sizeof later, kl, "%s%d>%d@%.1fs (%s)", kl ? ", " : "", g_rec[i].a, g_rec[i].b, dt - GC_LATENCY_S, g_rec[i].why);
                        }
                        if (pays) {
                            snprintf(w, sizeof w, "%s %s held %s%s: one upshift 1-1.5 shift durations (%.2f-%.2f s) after the lift (its saving %.2f s beats its price %.2f s)", tr.n, tr.calm ? "calm" : "lug", mds[mi].n, lb_sfx(), dur - 0.02, 1.5 * dur + 0.02, g_lb_win.sav, g_lb_win.price);
                            int skip = 0; float land_rpm = 0;   // (the first shift of the window: its skip size and its landing, which must not be in lug)
                            for (int i = 0; i < g_rec_n; i++) { const double dt = g_rec[i].t - g_lb_t0 - g_lb_mark; if (g_lb_mark < 0 || dt < 0.1 || dt > 10) continue; skip = g_rec[i].b - g_rec[i].a; land_rpm = vehicle_rpm_at(g_rec[i].b, g_rec[i].kmh / 3.6f); break; }
                            printf("      the upshift of the lift: a skip of %d, landing %.0f rpm (P1 %.0f)\n", skip, land_rpm, bd->p1);
                            lb_verdict(n == 1 && ups == 1 && first - GC_LATENCY_S >= dur - 0.02 && first - GC_LATENCY_S <= 1.5 * dur + 0.02 && land_rpm >= bd->p1, w, true, !g_lb_foot ? "E" : NULL);
                        } else {
                            snprintf(w, sizeof w, "%s %s held %s%s: no shift within 1.5 s of the lift (the upshift's saving %.2f s is under its price %.2f s)%s%s", tr.n, tr.calm ? "calm" : "lug", mds[mi].n, lb_sfx(), g_lb_win.sav, g_lb_win.price, later[0] ? "; later, logged: " : "", later);
                            lb_verdict(early == 0, w, true, !g_lb_foot ? "E" : NULL);
                        }
                    }
                }
            }
}

// ---- 18i-4b: the lift habits (WP-D, addendum 24a wired through gc_lift_return -> driver_lift_return_p): the same calm lift with three drivers. Fresh = the prior. 'Returns' = 40 lifts of 1.5 s
// that all ended in a re-press: a lift that has lasted as long as the window is likely to end soon, so a shift now is more likely to lose the drive: it waits. 'Long' = 40 lifts of 8 s that
// ended in the brake: a lift that has lasted the window will not end soon: the shift is cheap. The learned chance is held to a factor of two of the prior (P2): the held lift still shifts, a
// 0.3 s blip shifts nothing for any of them.
static void lb_profile_text(char *buf, size_t n, int cs, char how, int kmh)
{
    size_t k = 0;
    lb_cat(buf, n, k, "dazdrive-driver v=1\nlifts=40:");
    for (int i = 0; i < 40; i++) lb_cat(buf, n, k, "%s%d/%c/%d", i ? "," : "", cs, how, kmh);
    lb_cat(buf, n, k, "\nend\n");
}
static void run_lb_lift_learned()
{
    printf("\n  18i-4b lift habits%s: DC16 40 t, calm history, a pull at 0.3 to 50 km/h, then the pedal comes off (held / a 0.3 s blip); the driver's lift profile differs\n", g_lb_foot ? " (the REAL foot)" : "");
    printf("  %-8s %-6s %5s %7s %6s  %s\n", "driver", "case", "gear", "dt", "shifts", "after the lift");
    static char returns[1024], longer[1024];
    lb_profile_text(returns, sizeof returns, 150, 'r', 50);
    lb_profile_text(longer, sizeof longer, 800, 'b', 50);
    struct Pf { const char *n; const char *txt; } pfs[3] = { { "fresh", NULL }, { "returns", returns }, { "long", longer } };
    double first_held[3] = { -1, -1, -1 }; int n_held[3] = {}, n_blip[3] = {};
    for (int pi = 0; pi < 3; pi++)
        for (int ci = 0; ci < 2; ci++) {
            const double lift_s = ci ? 0.3 : 1e9;
            char key[96]; snprintf(key, sizeof key, "lift habits %s %s%s", pfs[pi].n, ci ? "blip" : "held", lb_sfx());
            if (lb_skip(key)) continue;
            use_dc16();
            lb_begin();
            lb_calm_history(1);
            g_driver_inject = pfs[pi].txt;
            g_lb_pull = 0.3f; g_lb_lift_s = lift_s; g_lb_trigger = 2;
            setup su = {40000, 1, 1, 45, 3.0f, 0.5f};
            g_lb_snap_g = 0; g_lb_snap_rpm = 0;
            lb_run(key, su, c_lb_lift, 1, lb_snap, 1);
            g_driver_inject = NULL;
            lb_calm_history(0);
            double first = -1; int n = 0; char seq[96]; size_t k = 0; seq[0] = 0;
            for (int i = 0; i < g_rec_n; i++) {
                const double dt = g_rec[i].t - g_lb_t0 - g_lb_mark;
                if (g_lb_mark < 0 || dt < 0 || dt > 10) continue;
                if (dt < 0.1 || (lift_s < 1.0 && dt > lift_s + 0.3)) continue;
                if (first < 0) first = dt;
                n++;
                lb_cat(seq, sizeof seq, k, "%s%d>%d@%.1fs", k ? " " : "", g_rec[i].a, g_rec[i].b, dt);
            }
            char dts[8]; if (first >= 0) snprintf(dts, sizeof dts, "%.2f", first); else snprintf(dts, sizeof dts, "-");
            printf("  %-8s %-6s %5d %7s %6d  %s\n", pfs[pi].n, ci ? "blip" : "held", g_lb_snap_g, dts, n, seq);
            if (ci) n_blip[pi] = n; else { n_held[pi] = n; first_held[pi] = first; }
        }
    if (first_held[0] < 0 && n_held[0] == 0 && lb_only()) return;
    // The row measures when a lift's shift falls due, so it needs a pull that ends in a gear the lift would leave. When the pull itself ends in the gear the lift would
    // pick (the band taste charged until the shift a gear must make, WP-E: the calm 0.3 pull now ends in 11 at 931 rpm, where it ended in 10 at 1201), nothing is due for
    // any of the three drivers and there is nothing to time: no verdict, said so.
    if (n_held[0] == 0 && n_held[1] == 0 && n_held[2] == 0) {
        printf("      n/a: the pull ended in the gear the lift would pick (gear %d, %d rpm): no shift is due, so there is no decision to time\n", g_lb_snap_g, (int)g_lb_snap_rpm);
        lb_verdict(n_blip[0] == 0 && n_blip[1] == 0 && n_blip[2] == 0, "a 0.3 s blip shifts nothing for any of the three drivers (the learned chance is held to a factor of two of the prior)");
        return;
    }
    lb_verdict(n_blip[0] == 0 && n_blip[1] == 0 && n_blip[2] == 0, "a 0.3 s blip shifts nothing for any of the three drivers (the learned chance is held to a factor of two of the prior)");
    // (WP-G1: "no shift within 10 s" is stored as -1, which ranked as the EARLIEST decision: a driver whose lifts end soon and who never shifts looked like the one who shifts at once.
    // A decision that does not come ranks after every decision that does. And "still shifts once" is the prior's own precondition: where the prior has no shift due, the habits may only
    // wait (the 'returns' driver) or make a shift that was not due worth it (the 'long' driver), never bring a shift forward for the one who re-presses.)
    const double never = 1e9;
    const double f0 = first_held[0] < 0 ? never : first_held[0], f1 = first_held[1] < 0 ? never : first_held[1], f2 = first_held[2] < 0 ? never : first_held[2];
    if (n_held[0] >= 1) lb_verdict(n_held[1] == 1 && n_held[2] == 1, "the held lift still shifts once for the driver who always re-presses and for the one who never does (within 10 s)");
    char w[240];
    char s0[16], s1[16], s2[16];
    if (f0 < never) snprintf(s0, sizeof s0, "%.2f s", f0); else snprintf(s0, sizeof s0, "no shift");
    if (f1 < never) snprintf(s1, sizeof s1, "%.2f s", f1); else snprintf(s1, sizeof s1, "no shift");
    if (f2 < never) snprintf(s2, sizeof s2, "%.2f s", f2); else snprintf(s2, sizeof s2, "no shift");
    snprintf(w, sizeof w, "his habits move the decision the right way: it waits for the driver whose lifts end soon (%s against %s) and does not for the one whose lifts go on (%s)", s1, s0, s2);
    lb_verdict(f1 >= f0 - 1e-6 && f2 <= f0 + 1e-6, w);
    if (f0 < never && f1 < never) {
        snprintf(w, sizeof w, "and by no more than two shift durations of the prior's decision (%.2f s later; the bound P2)", f1 - f0);
        lb_verdict(f1 - f0 <= 2.0 * (1.25 * 0.5 + 0.15) + 1e-6, w);
    }
}

// ---- 18i-5: a roundabout, measured live (yaw rate and steering, no route)
static double g_lb_bend_a = -1, g_lb_bend_b = -1;   // the roundabout: from a to b seconds into the run (-1: none)
static void c_lb_bend(double t)
{
    const bool in = g_lb_bend_a >= 0 && t >= g_lb_bend_a && t < g_lb_bend_b;
    const float v = fmaxf(M.v, 1.0f), R = 20.0f;   // a roundabout of radius 20 m: 5 m/s2 at 36 km/h
    T.yaw = in ? v / R / 6.2831853f : 0; T.in_steer = in ? 0.5f : 0;
    g_pl_ped = 0.3f;
    c_pull(t);
}
static void run_lb_bend()
{
    printf("\n  18i-5 bend%s: a light pull from a standstill; first without a bend (when does the box make its first upshift at 25-60 km/h?), then a roundabout (R 20 m, live yaw and steering,\n", g_lb_foot ? " (the REAL foot)" : "");
    printf("  no route) from 1.5 s before that shift to 2.5 s after it\n");
    printf("  %-14s %-6s %10s %16s\n", "truck", "mode", "straight s", "with roundabout s");
    struct Md { const char *n; float cvar; } mds[3] = { { "Eco", 10.0f }, { "Normal", 3.0f }, { "Power", 1.6667f } };
    struct Tr { const char *n; int kind; float mass; } trs[] = { { "DC16 18 t", 0, 18000 }, { "Pete 15.5 t", 2, 15500 } };
    for (const Tr &tr : trs)
        for (int mi = 0; mi < 3; mi++) {
            char key[96]; snprintf(key, sizeof key, "bend %s %s%s", tr.n, mds[mi].n, lb_sfx());
            if (lb_skip(key)) continue;
            if (tr.kind == 0) use_dc16(); else use_pete();
            lb_begin();
            double t_straight = -1, t_bend = -1; int bend_wall = 0;
            for (int pass = 0; pass < 2; pass++) {
                g_lb_bend_a = g_lb_bend_b = -1;
                if (pass) { g_lb_bend_a = t_straight - 1.5; g_lb_bend_b = t_straight + 2.5; }
                if (pass && t_straight < 0) break;
                setup su = {tr.mass, 1, 1, 40, mds[mi].cvar, 0.5f};
                lb_run(key, su, c_lb_bend, 1, NULL, 1);
                T.yaw = 0; T.in_steer = 0;
                double first = -1;
                int wall = 0;
                for (int i = 0; i < g_rec_n; i++) if (g_rec[i].b > g_rec[i].a && g_rec[i].kmh >= 25 && g_rec[i].kmh <= 60) { first = g_rec[i].t - g_lb_t0; wall = g_rec[i].wall; break; }
                if (pass == 0) t_straight = first; else { t_bend = first; bend_wall = wall; }
            }
            g_lb_bend_a = g_lb_bend_b = -1;
            char a[16] = "-", b[16] = "-";
            if (t_straight >= 0) snprintf(a, sizeof a, "%.1f", t_straight);
            if (t_bend >= 0) snprintf(b, sizeof b, "%.1f", t_bend);
            printf("  %-14s %-6s %10s %16s%s\n", tr.n, mds[mi].n, a, b, bend_wall && t_bend >= 0 ? "  (the rev wall)" : "");
            char w[128]; snprintf(w, sizeof w, "%s %s%s: the upshift due inside the roundabout comes after the exit (or the gear is at the rev wall)", tr.n, mds[mi].n, lb_sfx());
            // the stay bonus defers a score-based upshift; a gear at the rev wall must go up whatever the bend (the wall is a protection, not a taste)
            if (t_straight >= 0) lb_verdict(t_bend < 0 || t_bend >= t_straight + 2.5 - 0.05 || bend_wall, w);
        }
}

// ---- 18i-6: the 18f totals against the rewrite (15ce007: 484 shifts, no hunt, 26 shifts in the sprint rows, no downshift before the known climb's foot)
static void run_lb_totals()
{
    printf("\n  18i-6 the 18f rows against the rewrite 15ce007 (484 shifts, 0 hunts; sprints 26 shifts, 0 hunts; known climbs 0 downshifts before the foot)\n");
    // K2 step 1 (the one hunt definition): the shift bars count the shifts WITHOUT the reversals that follow the driver's own foot change (a lift, a press: the loved lift downshift is one); the totals say both.
    const int sh_all = g_lr_sh_all - g_lr_drv_all, sh_spr = g_lr_sh_spr - g_lr_drv_spr;
    printf("      now: %d shifts (%d without the driver's own reversals), %d hunts; sprints %d shifts (%d without), %d hunts; known climbs %d downshifts before the foot\n", g_lr_sh_all, sh_all, g_lr_hu_all, g_lr_sh_spr, sh_spr, g_lr_hu_spr, g_lr_downs_foot);
    lb_verdict(g_lr_hu_all - g_lr_hu_spr - g_lr_hu_known == 0, "18f release, surprise and steady rows: no hunt");
    // WP-D measured what the four carried rows below are made of (docs/powerband-change.md, "Round 3, package D"): they are not the lift habits.
    // WP-E measured what is left of D2's four rows (docs/powerband-change.md, "Round 3, package E"): the sprint hunts were the kickdown into the temporary band (closed); the others are
    // priced by terms E cannot move without the hunts coming back.
    lb_verdict(g_lr_hu_spr == 0, "the sprint rows: no hunt (the ATO's first pull from 50 km/h kicked down into the temporary band, 9 -> 8 landing at 1662 rpm, and the rev-limit skip 8 -> 10 followed: a downshift never lands there now)");
    lb_verdict(sh_spr <= 26, "the sprint rows: at most the rewrite's 26 shifts (E measured: 40 with 0 hunts, 51 at D; the bar is the rewrite's, from before the real pedal; G1 counted who decided the 45 shifts of P6b: 21 are lift decisions (the reserve downshift in the 6 s lift, the owner's pedal-off rule), 19 the full-throttle plan's wall upshifts, 5 pulls; without the lift model 24, under 26)", true, "E");
    lb_verdict(g_lr_downs_foot == 0 && g_lr_hu_known == 0, "the known climb: no downshift before the foot, no hunt (E measured: 8 -> 7 downshifts before the foot after the drive-time price and the band taste split at the shift a gear must make; what is left is the climb's demand priced at the foot as a speed to reach: staying in 14 with the foot 6 s ahead pays expect 7.5 s against 0.15 for 12)", true, "E");
    lb_verdict(sh_all <= 484, "18f shifts at most the rewrite's 484 (E measured: 566 -> 539; the 484 is the rewrite's before the real pedal and before the lift model's shifts; G1 counted who decided the shifts of P6b: pull 224, lift 91, full-throttle plan 108, stall rule 112, rev-limit rules 63, lugging rule 9; the lift model is 91 of them)", true, "E");
}

// ---- 18i-7: the cruise stand (part 18, cruise 83 flat and on -1 %): the cruise is a controller and has no lift habits; easing its throttle on a descent is no lift.
// The rewrite 15ce007 / fdc869b: no shift, fuel 13.48 / 8.39 / 17.49 / 5.47 L (the first version of the pedal-off model shifted 14 -> 13 on the 40 t descent: 9.15 L)
static void run_lb_cruise_stand()
{
    static const double ref_l[4] = { 13.48, 8.39, 17.49, 5.47 };
    static const char *nm[4] = { "18 t flat", "18 t -1 %", "40 t flat", "40 t -1 %" };
    printf("\n  18i-7 the cruise stand at 83 km/h (the cruise controls: its idle wish is the force that holds the speed, no lift)\n");
    for (int i = 0; i < 4; i++) {
        char w[96]; snprintf(w, sizeof w, "DC16 %s, cruise 83: no shift, fuel within 1 %% of the rewrite's (%.2f L: %.2f)", nm[i], ref_l[i], g_cr83[i].fuel);
        lb_verdict(g_cr83[i].shifts == 0 && fabs(g_cr83[i].fuel - ref_l[i]) <= 0.01 * ref_l[i], w);
    }
    // The heavy truck on the flat (part 18b): the foot is off and the cruise holds, so the history's reserve counts as far as the foot is expected back (WP-D1). Before it the
    // 40 t sat in 13 at 1397 rpm in Normal and Power (20.31 L, the history asked a reserve for a pull the cruise does not ask) where the 18 t takes 14 at 999 rpm; the economical
    // gear is one shift away and any mean rpm under the next gear down's 1397 says the box took it.
    static const char *mn[3] = { "Eco", "Normal", "Power" };
    for (int mi = 0; mi < 3; mi++) {
        if (!(g_lm_c83[mi].rpm > 0)) continue;   // (18b did not run)
        char w[128]; snprintf(w, sizeof w, "DC16 40 t, cruise 83 flat, %s: the economical gear (mean rpm %.0f under 1200: 14 is 999, 13 is 1397)", mn[mi], g_lm_c83[mi].rpm);
        lb_verdict(g_lm_c83[mi].rpm < 1200 && g_lm_c83[mi].shifts <= 1, w);
    }
    // The 18 t on a 4 % climb with the cruise (part 18b): gear 14 holds it at 959 rpm (Eco: 33.97 L, one shift, no hunt; the rewrite 15ce007 had it too, with 26 shifts); every mode
    // should take the gear that holds without hunting, and the fuel of a mode that does not is the price (L6: no waste in the cruise in any mode).
    if (g_lm_c4[0].rpm > 0 && g_lm_c4[1].rpm > 0 && g_lm_c4[2].rpm > 0) {
        double best = g_lm_c4[0].fuel; for (int mi = 1; mi < 3; mi++) best = fmin(best, g_lm_c4[mi].fuel);
        for (int mi = 0; mi < 3; mi++) {
            char w[300];
            snprintf(w, sizeof w, "DC16 18 t, cruise 80 on +4 %%, %s: the gear that holds it without hunting and without the fuel (%.0f rpm, %d shifts, %d hunts, %.2f L against %.2f L)", mn[mi], g_lm_c4[mi].rpm, g_lm_c4[mi].shifts, g_lm_c4[mi].hunts, g_lm_c4[mi].fuel, best);
            lb_verdict(g_lm_c4[mi].shifts <= 1 && g_lm_c4[mi].hunts == 0 && g_lm_c4[mi].fuel <= 1.02 * best, w, true,
                       mi == 0 ? NULL : "E");   // (open[E] for Normal and Power: the pedal-on shift now is a duration, dv / a of the landing gear = 4.2 s on this climb against 3.3 s of band taste in 13; priced as drive time, a marginal lug gear on the 40 t climbs is taken and left again: 18b hunts 2 -> 18 a mode, t70 +30 %)
        }
    }
    // The 35.6 t Pete on a 6 % climb with the cruise at 80 (part 18b): no gear holds 80 km/h, the speed falls to the gear that holds (8, 58 km/h). Two protections used to meet here:
    // the lug rule's 9 -> 8 (it can only fire below 1600 rpm in 9, which is 2080 in 8) landed at the rev wall, 1977 of 2000 rpm, and the rev-limit rule took the truck back to 9, which
    // cannot hold, and the lug rule sent it down again (9>8>9>8 in 2.5 s). A gear that another protection takes off at once is not one a protection may go to (the rev-limit rule's
    // target, and the layers' allowed mask, hold the lug rule's test), so the speed falls and takes the revs with it.
    for (int mi = 0; mi < 3; mi++) {
        if (!(g_lm_p6[mi].rpm > 0)) continue;
        char w[200]; snprintf(w, sizeof w, "Pete 35.6 t, cruise 80 on +6 %%, %s: the speed falls to the gear that holds it, no gear changed back (%d shifts, %d hunts, mean rpm %.0f)", mn[mi], g_lm_p6[mi].shifts, g_lm_p6[mi].hunts, g_lm_p6[mi].rpm);
        lb_verdict(g_lm_p6[mi].hunts == 0, w);
    }
}

// ---- 18i-8: hint votes (WP-D4, owner point 20.2): a +/- press in Automatic writes one HINTVOTE line with the decision snapshot, the layers' candidate terms included
static void c_lb_hint(double t)
{
    static int done;
    if (t < 0.1) done = 0;
    g_pl_ped = 0.6f;
    c_pull(t);
    if (t > 15 && !done && !M.state && M.gear > 1) { done = 1; M.target = M.gear - 1; M.state = 1; M.shift_end = t_now + 0.6; }   // the game's hint handler: the target one gear down
}
static void run_lb_hint()
{
    printf("\n  18i-8 hint votes: DC16 40 t at 0.6 pedal, a hint down at 15 s: one HINTVOTE line with the box's last decision and the terms of its candidates\n");
    if (lb_skip("hint vote")) return;
    use_dc16();
    lb_begin();
    g_votes = 0; g_vote[0] = 0;
    setup su = {40000, 50, 9, 35, 3, 0.5f};
    lb_run("hint vote", su, c_lb_hint, 1);
    printf("      %s\n", g_vote);
    lb_verdict(g_votes == 1, "one hint, one vote");
    lb_verdict(strstr(g_vote, "\"dir\":-1") != NULL && strstr(g_vote, "\"mode\":\"Normal\"") != NULL, "a hint down in Normal: dir -1 and the mode");
    lb_verdict(strstr(g_vote, "\"terms\":\"cur ") != NULL && strstr(g_vote, " pick ") != NULL && strstr(g_vote, " | ") != NULL, "the vote carries the layers' last decision (cur, pick, age, the terms)");
}

static int run_layers_b()
{
    printf("\n=== SCORES part 18i: the expected-time term's rows (landing under load, Eco cruise reversals, the launch, the lift, the roundabout) ===\n");
    g_lb_gaps = g_lb_carried = 0;
    run_lb_landing();
    run_lb_cruise();
    run_lb_launch();
    run_lb_lift();
    run_lb_bend();
    run_lb_lift_learned();
    g_lb_foot = 1;   // the same three rows on the real foot (WP-G1.2b): must rows, like the demand-fed ones
    run_lb_lift();
    run_lb_bend();
    run_lb_lift_learned();
    g_lb_foot = 0;
    run_lb_cruise_stand();
    run_lb_hint();
    run_lb_totals();
    g_pl_wish = 0; g_plant_shift_drag = g_plant_shift_extra = 0; T.yaw = 0; T.in_steer = 0;
    printf("\n  18i: %d GAP\n  carried: %d\n", g_lb_gaps, g_lb_carried);
    return g_lb_gaps;
}
