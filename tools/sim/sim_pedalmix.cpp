// The wish pedal's torque_share ([pedal] torque_share, modules/pedal.cpp) on the TrueShift simulator's plant. sim.cpp is included whole (its
// main renamed away, its stand-in for the wish force renamed so the REAL modules/pedal.cpp is the one TrueShift reads), the pedal module
// runs in the control step exactly as Cruise calls it, and TrueShift chooses the gears with the score-based choice (choice 1) on the wish.
// What it shows: in a gear, at a steady pedal, how the wheel torque and the engine power move from the landing to the next upshift for
// torque_share 0 / 0.5 / 0.75 / 1 (0 = constant power, 1 = constant torque), what that costs in time and shifts, and that a wavering pedal
// does not make the throttle jitter. Build+run: tools/sim/run_pedalmix.sh [-v]
#define main sim_orig_main
#define pedal_wish_force sim_old_pedal_wish_force
#include "sim.cpp"
#undef pedal_wish_force
#undef main

static creep_info CI0; static cruise_info CU0;
void creep_info_get(creep_info *o) { *o = CI0; }
void cruise_info_get(cruise_info *o) { *o = CU0; }
#include "modules/pedal.cpp"

static void pm_set(const char *n, float v)
{
    for (int i = 0; i < m_pedal.n_params; i++) if (!strcmp(m_pedal.params[i].key, n)) *m_pedal.params[i].val = v;
}

// ---- the run: a pedal script, the grade, and a recorder of the samples
enum { PM_N = 20000 };
static struct { double t; float kmh, rpm, thr, tq, pw; int gear, state; } pm_s[PM_N];
static int pm_n; static float pm_grade; static int pm_kind; static float pm_ped;   // kind 0: steady pedal, 1: sinusoid around pm_ped (+-0.2, 6 s), 2: a shaky foot (+-0.02, 0.9 s), 3: steps (+0.2 at 25 s, back at 50 s)
static float pm_thr_prev, pm_ped_prev; static double pm_tv, pm_maxstep; static int pm_reversals, pm_wrong; static float pm_dprev;

static void c_mix(double t)
{
    if (t <= 0) {
        pm_n = 0; pm_tv = 0; pm_maxstep = 0; pm_reversals = 0; pm_wrong = 0; pm_dprev = 0; pm_thr_prev = NAN; pm_ped_prev = NAN;
        S.truck_k = fminf(1.0f, fmaxf(0.8f, sqrtf(peak_power() / M.mass / p_ref_kwt)));   // what truck_curve sets at a standstill
    }
    M.grade_pct = pm_grade;
    M.pedal = pm_kind == 1 ? pm_ped + 0.2f * sinf((float)t * 6.2832f / 6.0f) : pm_kind == 2 ? pm_ped + 0.02f * sinf((float)t * 6.2832f / 0.9f) :
              pm_kind == 3 ? pm_ped + 0.2f * fminf(1.0f, fmaxf(0.0f, fminf((float)(t - 25), (float)(51 - t)))) : pm_ped;
    float out = M.pedal;
    M.wish_on = pedal_wish(M.pedal, 1 / 60.0f, 0, &out);
    M.wish_thr = out;
    float thr = M.wish_on ? out : M.pedal;
    if (pm_n < PM_N && M.gear > 0) {
        engine_info e; gb_engine(0, &e);
        float r = rpm_now(), tq = TORQUE * curve_factor(&e, r) * thr;
        pm_s[pm_n++] = { t_now, M.v * 3.6f, r, thr, tq * RATIO[M.gear - 1] * DIFF * 0.9f, tq * r * 0.10472f, M.gear, M.state };
    }
    if (M.gear > 0 && !M.state && pm_thr_prev == pm_thr_prev && M.v * 3.6f > 20) {   // jitter: the throttle's path length, its largest step, and how often it turns round
        float d = thr - pm_thr_prev;
        pm_tv += fabsf(d); pm_maxstep = fmax(pm_maxstep, fabsf(d));
        if (fabsf(d) > 0.002f) { if (pm_dprev * d < 0) pm_reversals++; pm_dprev = d; }
        float dp = M.pedal - pm_ped_prev;
        if (pm_ped_prev == pm_ped_prev && ((dp > 0.001f && d < -0.02f) || (dp < -0.001f && d > 0.02f))) pm_wrong++;   // the throttle went the other way from the pedal by 0.02 in one tick
    }
    pm_ped_prev = M.pedal;
    pm_thr_prev = M.gear > 0 && !M.state ? thr : NAN;
}

struct pm_res { int ok; double t3070; int shifts, pairs; double med_tq, med_pw; int segs; float thr_lo, thr_hi; double tv, maxstep; int rev, wrong; };
static double pm_median(double *a, int n)
{
    if (n == 0) return NAN;
    for (int i = 1; i < n; i++) { double x = a[i]; int j = i - 1; while (j >= 0 && a[j] > x) { a[j + 1] = a[j]; j--; } a[j + 1] = x; }
    return n % 2 ? a[n / 2] : 0.5 * (a[n / 2 - 1] + a[n / 2]);
}
static pm_res pm_row(float mass, float grade, int kind, float ped, float share)
{
    pm_set("wish", 1); pm_set("torque_share", share);
    pm_grade = grade; pm_kind = kind; pm_ped = ped;
    m_pedal.reset(RESET_HARD, "sim");
    sc_do("pedalmix", {mass, 20, sc_gear_for(20, 1100), 150, 3, 0.8f}, c_mix, 1);
    if (getenv("PM_DUMP")) for (int i = 0; i < pm_n; i += 6) printf("%7.2f %5.1f km/h g%2d st%d rpm %4.0f thr %.3f tq %6.0f pw %5.0f\n", pm_s[i].t, pm_s[i].kmh, pm_s[i].gear, pm_s[i].state, pm_s[i].rpm, pm_s[i].thr, pm_s[i].tq, pm_s[i].pw / 1000);
    pm_res r = {}; r.t3070 = NAN; r.thr_lo = 9; r.thr_hi = -1;
    double t30 = -1, t70 = -1, t80 = 1e30;
    for (int i = 0; i < pm_n; i++) {
        if (t30 < 0 && pm_s[i].kmh >= 30) t30 = pm_s[i].t;
        if (t70 < 0 && pm_s[i].kmh >= 70) t70 = pm_s[i].t;
        if (pm_s[i].kmh >= 80) { t80 = pm_s[i].t; break; }
    }
    if (t30 >= 0 && t70 >= 0) r.t3070 = t70 - t30;
    // the window: 20 km/h until 80 (or the end of the run)
    int n = 0, last_g = 0, last_dir = 0; double last_t = -99, med_tq[64], med_pw[64];
    int seg0 = -1;
    for (int i = 0; i < pm_n && pm_s[i].t < t80; i++) {
        auto &s = pm_s[i];
        if (!s.state && s.t - pm_s[0].t > 0.5) { r.ok = 1; if (s.thr < r.thr_lo) r.thr_lo = s.thr; if (s.thr > r.thr_hi) r.thr_hi = s.thr; }
        if (s.gear != last_g && last_g > 0) {   // a shift landed
            int dir = s.gear > last_g ? 1 : -1;
            r.shifts++;
            if (last_dir && dir != last_dir && s.t - last_t < 4.0) r.pairs++;
            last_dir = dir; last_t = s.t;
            // the segment of the gear that ended: from landing + 0.3 s to the sample before this shift, only when an UPSHIFT ended it
            if (dir > 0 && seg0 >= 0) {
                int a = seg0, b = i - 1;
                while (a < b && pm_s[a].t < pm_s[seg0].t + 0.3) a++;
                while (b > a && pm_s[b].state) b--;
                if (pm_s[b].t - pm_s[a].t >= 1.5 && pm_s[a].tq > 1 && pm_s[a].pw > 1 && n < 64) {
                    med_tq[n] = (pm_s[b].tq / pm_s[a].tq - 1) * 100; med_pw[n] = (pm_s[b].pw / pm_s[a].pw - 1) * 100; n++;
                }
            }
            seg0 = -1;
        }
        if (seg0 < 0 && !s.state && s.gear > 0) seg0 = i;
        last_g = s.gear;
    }
    r.segs = n; r.med_tq = pm_median(med_tq, n); r.med_pw = pm_median(med_pw, n);
    r.tv = pm_tv; r.maxstep = pm_maxstep; r.rev = pm_reversals; r.wrong = pm_wrong;
    return r;
}

static void pm_print(const char *what, float share, const pm_res &r)
{
    char t[16] = "  -", a[16] = "   -", b[16] = "   -";
    if (r.t3070 == r.t3070) snprintf(t, sizeof t, "%5.1f", r.t3070);
    if (r.med_tq == r.med_tq) { snprintf(a, sizeof a, "%+5.0f%%", r.med_tq); snprintf(b, sizeof b, "%+5.0f%%", r.med_pw); }
    printf("  %-26s %5.2f  %6s  %6d  %5d   %6s %6s  (%2d)   %.2f-%.2f\n", what, share, t, r.shifts, r.pairs, a, b, r.segs, r.thr_lo > r.thr_hi ? 0.0f : r.thr_lo, r.thr_hi < 0 ? 0.0f : r.thr_hi);
}

static int pm_main()
{
    use_ato();
    int failed = 0;
    auto check = [&](bool ok, const char *what) { printf("  %-4s %s\n", ok ? "ok" : "FAIL", what); if (!ok) failed = 1; };
    const float shares[4] = { 0, 0.5f, 0.75f, 1 };
    printf("\n=== pedalmix: [pedal] torque_share on the wish pedal, Renault ATO 12 speeds, Normal, choice 1, steady pedal from 20 km/h (150 s or 80 km/h) ===\n");
    printf("  gear segments: from landing (+0.3 s) to the next upshift; median change of the wheel torque and of the engine power over such a segment\n");
    printf("  %-26s %5s  %6s  %6s  %5s   %6s %6s  %4s   %s\n", "row", "share", "30-70s", "shifts", "u-d4s", "torque", "power", "segs", "throttle");
    static pm_res R[2][2][3][4];   // mass, grade, pedal, share
    const float masses[2] = { 33000, 17000 }, grades[2] = { 0, 2 }, peds[3] = { 0.4f, 0.6f, 0.8f };
    for (int m = 0; m < 2; m++)
        for (int g = 0; g < 2; g++)
            for (int p = 0; p < 3; p++)
                for (int s = 0; s < 4; s++) {
                    char nm[64]; snprintf(nm, sizeof nm, "%.0f t, %d%%, pedal %.1f", masses[m] / 1000, (int)grades[g], peds[p]);
                    R[m][g][p][s] = pm_row(masses[m], grades[g], 0, peds[p], shares[s]);
                    pm_print(nm, shares[s], R[m][g][p][s]);
                    if (s == 3) printf("\n");
                }
    printf("  moving pedals, flat, around 0.6 (sinusoid +-0.2 with a 6 s period; a shaky foot +-0.02 with a 0.9 s period; steps +0.2 at 25 s and back at 51 s):\n"
           "  the throttle's path length per second above 20 km/h, its largest step in one tick (1/60 s), how often it turns round, ticks where it went the other way from the pedal\n");
    printf("  %-26s %5s  %6s  %6s  %5s   %6s %6s  %4s   %s\n", "row", "share", "30-70s", "shifts", "u-d4s", "torque", "power", "segs", "throttle");
    static pm_res W[2][3][4]; static double wtv[2][3][4];
    const char *kn[3] = { "sinusoid", "shaky foot", "steps" };
    for (int m = 0; m < 2; m++)
        for (int k = 0; k < 3; k++) {
            for (int s = 0; s < 4; s++) {
                char nm[64]; snprintf(nm, sizeof nm, "%.0f t, flat, %s", masses[m] / 1000, kn[k]);
                W[m][k][s] = pm_row(masses[m], 0, k + 1, 0.6f, shares[s]);
                pm_print(nm, shares[s], W[m][k][s]);
                double secs = 0; for (int i = 0; i < pm_n; i++) if (!pm_s[i].state && pm_s[i].kmh > 20) secs += 1 / 60.0;
                wtv[m][k][s] = secs > 0 ? W[m][k][s].tv / secs : 0;
                printf("      path %.3f /s, largest step %.3f, turns %d, wrong-way ticks %d\n", wtv[m][k][s], W[m][k][s].maxstep, W[m][k][s].rev, W[m][k][s].wrong);
            }
            printf("\n");
        }
    printf("\n  checks\n");
    for (int m = 0; m < 2; m++) {
        auto &a = R[m][0][1][0], &b = R[m][0][1][3];
        char w[160];
        snprintf(w, sizeof w, "%.0f t flat pedal 0.6: share 1 keeps the torque in a gear better than share 0 (|%+.0f%%| < |%+.0f%%|)", masses[m] / 1000, b.med_tq, a.med_tq);
        check(a.segs > 0 && b.segs > 0 && fabs(b.med_tq) < fabs(a.med_tq), w);
        snprintf(w, sizeof w, "%.0f t flat pedal 0.6: the power rises with the revs at share 1 (%+.0f%%) and at share 0 it does not (%+.0f%%)", masses[m] / 1000, b.med_pw, a.med_pw);
        check(a.segs > 0 && b.segs > 0 && b.med_pw > a.med_pw + 5, w);
        for (int s = 1; s < 4; s++) {
            snprintf(w, sizeof w, "%.0f t flat pedal 0.6: the steady-pedal power change grows with the share (%.2f)", masses[m] / 1000, shares[s]);
            check(R[m][0][1][s].segs > 0 && R[m][0][1][s].med_pw >= R[m][0][1][s - 1].med_pw - 3, w);
        }
        for (int k = 0; k < 3; k++) {
            snprintf(w, sizeof w, "%.0f t, %s at share 0.75: the throttle's path length within 25 %% of share 0, largest tick step within +0.02, no more turns than share 0 plus 30", masses[m] / 1000, kn[k]);
            check(wtv[m][k][2] <= 1.25 * wtv[m][k][0] + 0.01 && W[m][k][2].maxstep <= W[m][k][0].maxstep + 0.02 && W[m][k][2].rev <= W[m][k][0].rev + 30, w);
            snprintf(w, sizeof w, "%.0f t, %s at share 1: the same", masses[m] / 1000, kn[k]);
            check(wtv[m][k][3] <= 1.25 * wtv[m][k][0] + 0.01 && W[m][k][3].maxstep <= W[m][k][0].maxstep + 0.02 && W[m][k][3].rev <= W[m][k][0].rev + 30, w);
            snprintf(w, sizeof w, "%.0f t, %s: no more wrong-way ticks at share 0.75 than at share 0 (+2)", masses[m] / 1000, kn[k]);
            check(W[m][k][2].wrong <= W[m][k][0].wrong + 2, w);
        }
        snprintf(w, sizeof w, "%.0f t, a shaky foot at share 1 keeps the hold: the torque change in a gear stays within 12 points of a steady pedal's (%+.0f%% against %+.0f%%)", masses[m] / 1000, W[m][1][3].med_tq, b.med_tq);
        check(W[m][1][3].segs > 0 && W[m][1][3].med_tq > b.med_tq - 12, w);
    }
    int bad = 0, nan = 0;
    for (int m = 0; m < 2; m++) for (int g = 0; g < 2; g++) for (int p = 0; p < 3; p++) for (int s = 0; s < 4; s++) {
        auto &r = R[m][g][p][s];
        if (r.thr_hi > 1.0001f || (r.ok && r.thr_lo < 0)) bad++;
        if (r.t3070 != r.t3070 && R[m][g][p][0].t3070 == R[m][g][p][0].t3070) nan++;   // a row that does not reach 70 km/h where share 0 does
    }
    check(bad == 0, "the throttle stays within 0..1 in every row");
    check(nan == 0, "no row fails to reach 70 km/h where share 0 reaches it");
    pm_set("torque_share", 0);
    printf("\n%s\n", failed ? "FAILED" : "all ok");
    return failed;
}

int main(int argc, char **argv)
{
    verbose = argc > 1 && !strcmp(argv[1], "-v");
    for (int i = 0; i < m_trueshift.n_params; i++) *m_trueshift.params[i].val = m_trueshift.params[i].def;
    for (int i = 0; i < m_pedal.n_params; i++) *m_pedal.params[i].val = m_pedal.params[i].def;
    T.fwd_gears = 12; T.diff = 2.85f; T.wheels = 4; T.retarder_steps = 5;
    for (int i = 0; i < 12; i++) T.ratio[i] = RATIO[i];
    save_volvo();
    G.tail_site = 5; G.decide_site = 1; G.n_choose = 1; G.choose_sites[0] = 2; G.n_start = 1; G.start_sites[0] = 4;
    G.window_fn = (uintptr_t)f_window; G.chooser_fn = (uintptr_t)f_choose; G.start_fn = (uintptr_t)f_start;
    m_trueshift.start();
    return pm_main();
}
