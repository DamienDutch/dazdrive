#include <algorithm>
// ================================================================ SWEEP (included by sim.cpp; driver: tools/sim/sweep_power.py)
// One run per process: SWEEP=1 SW_ENG=scania|renault|daf|weak SW_SCEN=acc|ovt|hill|eco|cr85|ramp SW_MODE=eco|normal|power SW_MASS=kg SW_GRADE=pct,
// the ini keys to try in SCP="k=v,...". Prints one CSV line "SW,...". The engines daf and weak are ESTIMATES (torque curve, gearbox).
// SW_DIST_M=<m> (WP-1111-D1): the same distance for every mode. A scenario with a target speed does not stop there: the driver holds the speed and the run stops after <m> metres from the window start;
//   t_hit and fuel_g are then the seconds and the diesel for the distance, and a second line "SWD,t to the speed,m to the speed,highest km/h over the target,t_hit to 0.001 s" is printed. Unset: nothing changes.
//
// Columns: scen, eng, mode, mass t, grade %, t_hit (s to the target speed of acc 80 / ovt 85 km/h, -1 = not reached), end km/h, lowest km/h,
//   mean rpm, p90 rpm (time-weighted, in gear), shifts in the window, fuel proxy g, distance m, share of the time in the top gear, OD-in km/h (ramp), engine kWh
// (the last two: the share of the time above the top of the power range minus 100 rpm, and the up-down or down-up pairs within 5 s)
// Fuel proxy: P x bsfc, bsfc = 190 g/kWh x (1 + 0.35 x ((rpm - r_best) / 1000)^2) x (1 + 0.7 x (1 - throttle)^2); r_best = middle of the torque plateau;
//   no fuel while the throttle is under 0.05 (overrun cut). A guess with the right shape (best at the plateau, dear at high revs and at a low load), no efficiency map.
extern float ts_mode_q(int mode);   // modules/trueshift.cpp: the mode's one fact, its worth of a second against Normal's (core/powertrain.h pt_mode_q), mode in MODE_*'s order
extern float ts_target_rpm(float *kd);   // modules/trueshift.cpp: the operating point the foot asked of the engine at the last layers decision, and its kickdown (read-only)
static float SW_IDLE = getenv("SW_IDLE") ? (float)atof(getenv("SW_IDLE")) : 550;   // (SW_IDLE: the truck's idle rpm, for the ride replay)
static void sw_geo(float *r, int n, float first, float last) { for (int k = 0; k < n; k++) r[k] = first * powf(last / first, k / (float)(n - 1)); }
// SW_DEF: an engine and gearbox from the game's definitions (tools/sim/sweep_fleet.py): "T=torque|L=rpm limit|C=rpm:factor rpm:factor ...|
// R=low0,low1,high0,high1,power0,power1,brake0,brake1|G=ratios (forward gears, low to high)|D=diff|W=wheel radius|S=shift time" and, for a
// torque-converter box, "K=stall ratio|B=lock gear (0 = never)|N=stall rpm"
static float sw_engine_def(const char *def)
{
    char buf[4096]; snprintf(buf, sizeof buf, "%s", def);
    float shift = 1.0f; NG = 0; NCURVE = 0; memset(RATIO, 0, sizeof RATIO); memset(CURVE, 0, sizeof CURVE);
    tc_reset();
    for (char *f = strtok(buf, "|"); f; f = strtok(NULL, "|")) {
        const char *v = f + 2;
        if (f[0] == 'T') TORQUE = (float)atof(v);
        else if (f[0] == 'L') LIMIT = (float)atof(v);
        else if (f[0] == 'D') DIFF = (float)atof(v);
        else if (f[0] == 'W') RADIUS = (float)atof(v);
        else if (f[0] == 'S') shift = (float)atof(v);
        else if (f[0] == 'K') TC_STALL = (float)atof(v);
        else if (f[0] == 'B') TC_LOCK_GEAR = (int)atof(v);
        else if (f[0] == 'N') TC_STALL_RPM = (float)atof(v);
        else if (f[0] == 'P') TC_PLANT_LOCK = (int)atof(v);   // the plant runs the gears over this one locked (the game's Allison; 0 = the plant slips in every gear)
        else if (f[0] == 'R') { float x[8]; sscanf(v, "%f,%f,%f,%f,%f,%f,%f,%f", x, x+1, x+2, x+3, x+4, x+5, x+6, x+7);
            // RANGES: [R_BRAKE, R_LOW, R_HIGH, R_POWER]
            RANGES[0][0] = x[6]; RANGES[0][1] = x[7]; RANGES[1][0] = x[0]; RANGES[1][1] = x[1]; RANGES[2][0] = x[2]; RANGES[2][1] = x[3]; RANGES[3][0] = x[4]; RANGES[3][1] = x[5]; }
        else if (f[0] == 'C') { char c2[2048]; snprintf(c2, sizeof c2, "%s", v); char *sp; for (char *p = strtok_r(c2, " ", &sp); p && NCURVE < 24; p = strtok_r(NULL, " ", &sp)) { float a, b; if (sscanf(p, "%f:%f", &a, &b) == 2) { CURVE[NCURVE][0] = a; CURVE[NCURVE][1] = b; NCURVE++; } } }
        else if (f[0] == 'G') { char c2[1024]; snprintf(c2, sizeof c2, "%s", v); char *sp; for (char *p = strtok_r(c2, " ", &sp); p && NG < 16; p = strtok_r(NULL, " ", &sp)) RATIO[NG++] = (float)atof(p); }
    }
    CDA = 6;
    T.fwd_gears = NG; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
    g_idle_sim = SW_IDLE;
    if (TC_STALL > 1.0f) vehicle_tc_prime(TC_STALL, TC_LOCK_GEAR, TC_STALL_RPM);
    return shift;
}
static float sw_engine(const char *name)
{
    if (getenv("SW_DEF")) return sw_engine_def(getenv("SW_DEF"));
    tc_reset();
    memset(RATIO, 0, sizeof RATIO);
    auto curve = [](std::initializer_list<float> v) { int i = 0; for (float x : v) ((float *)CURVE)[i++] = x; NCURVE = i / 2; };
    auto ranges = [](float a, float b, float c, float d, float e2, float f, float g2, float h) { float r[4][2] = { {a,b},{c,d},{e2,f},{g2,h} }; memcpy(RANGES, r, sizeof r); };
    float shift = 0.7f;
    if (!strcmp(name, "scania")) {   // Damien's V8, 3700 Nm; 13+1 CMR: raw 1 crawl, 2..13 the 12 gears, 14 the overdrive (ratios from the game's G25CM definition)
        float r[16] = { 20.81f, 16.16f, 12.57f, 9.76f, 7.56f, 5.87f, 4.55f, 3.53f, 2.77f, 2.15f, 1.66f, 1.29f, 1.00f, 0.78f };
        memcpy(RATIO, r, sizeof r); NG = 14; DIFF = 2.59f; RADIUS = 0.538f; TORQUE = 3700; LIMIT = 2500;
        curve({ 300,0.10f, 440,0.40f, 1000,1.00f, 1450,1.00f, 1600,0.91f, 1800,0.82f, 2300,0.40f, 2500,0.10f });
        ranges(1500,2500, 700,1500, 1000,1350, 1400,1900); shift = 0.7f;
    } else if (!strcmp(name, "renault")) {   // Damien's Renault T 480, 2700 Nm (curve and ranges from the log), the 12-speed box with diff 3.08 (ratios as use_ato)
        { float r[16] = { 11.73f, 9.21f, 7.09f, 5.57f, 4.35f, 3.41f, 2.70f, 2.12f, 1.63f, 1.28f, 1.00f, 0.79f }; memcpy(RATIO, r, sizeof r); } NG = 12; DIFF = 3.08f; RADIUS = 0.538f; TORQUE = 2700; LIMIT = 1900;
        curve({ 0,0, 550,0.5f, 900,1, 1250,1, 1600,0.79f, 2000,0.30f, 2100,0, 2100,0 });
        ranges(1500,1900, 750,1450, 900,1250, 1200,1600); shift = 1.0f;
    } else if (!strcmp(name, "volvo")) {   // the Volvo FH 2400 Nm of the file's start (run.sh's default truck): its own tables, saved at the start of main
        use_volvo(); shift = 0.5f;
    } else if (!strcmp(name, "daf")) {   // (est.) DAF MX-13 530-like: 2500 Nm, plateau 900-1400, peak power near 1600; 12-speed AMT (ratios guessed)
        sw_geo(RATIO, 12, 12.33f, 0.78f); NG = 12; DIFF = 2.64f; RADIUS = 0.52f; TORQUE = 2500; LIMIT = 2100;
        curve({ 0,0, 500,0.5f, 900,1, 1400,1, 1600,0.93f, 1800,0.80f, 2000,0.50f, 2200,0 });
        ranges(1500,2100, 700,1400, 1000,1500, 1300,1800); shift = 0.6f;
    } else {                              // (est.) weak 400 hp, 1900 Nm; 12-speed AMT (ratios guessed)
        sw_geo(RATIO, 12, 12.9f, 0.82f); NG = 12; DIFF = 3.36f; RADIUS = 0.51f; TORQUE = 1900; LIMIT = 2200;
        curve({ 0,0, 500,0.55f, 1000,1, 1400,1, 1700,0.86f, 1900,0.70f, 2100,0.35f, 2300,0 });
        ranges(1500,2200, 700,1400, 1000,1400, 1400,1900); shift = 0.8f;
    }
    CDA = 6;
    T.fwd_gears = NG; T.diff = DIFF;
    for (int i = 0; i < 16; i++) T.ratio[i] = RATIO[i];
    g_idle_sim = SW_IDLE;
    return shift;
}
static struct { int on, latched, ramp, sh0; double t0, rpm_s, rpm_t, fuel_g, kj, top_t, all_t, hist[400], bt[32], bt_top[32], t_hit, rpm_max; float vmin, vlast, hit_kmh, ropt, hit_m; double s0;
    double dist_m, t_speed, d_speed, v_over, vprev; int reached;   // SW_DIST_M: the distance to cover, and the moment the target speed was reached (the run goes on there)
    double lug_t, lug2_t, lug_all_t, t50, t70, fuel70, t80, fuel80;   // lugging: in gear above 30 km/h, throttle over 0.9 and the revs under the torque plateau (-50 rpm); lug2 (soft): throttle over 0.5 there; lug_all: the same without the speed floor
    double up_min_sum, up_min, up_t0; int up_min_n, up_active, gear_prev;   // the lowest rpm after each upshift (5 s or until the next shift)
    int seg_g, nseg; double seg_t, seg_t0; float seg_tq0, seg_pw0, seg_tq1, seg_pw1, seg_n0, seg_dtq[64], seg_dpw[64]; } SW;
static void sw_up_commit() { if (SW.up_active) { SW.up_min_sum += SW.up_min; SW.up_min_n++; SW.up_active = 0; } }
static double sw_hi_frac()   // the share of the time in gear above the top of the power range (minus 100 rpm)
{
    double tot = 0, hi = 0; int b0 = (int)((RANGES[3][1] - 100) / 10);
    for (int i = 0; i < 400; i++) { tot += SW.hist[i]; if (i >= b0) hi += SW.hist[i]; }
    return tot > 0 ? hi / tot : 0;
}
static double sw_p90()
{
    double tot = 0; for (int i = 0; i < 400; i++) tot += SW.hist[i];
    double acc = 0; for (int i = 0; i < 400; i++) { acc += SW.hist[i]; if (tot > 0 && acc >= 0.9 * tot) return i * 10 + 5; }
    return 0;
}
static void sw_hook(double dt)
{
    if (!SW.on) return;
    float kmh = M.v * 3.6f;
    {   // SW_TERMS=<seconds>: every so often the state and the terms of the layers' last decision (the candidates around the gear we are in: rpm after the shift, band, and the
        // score terms in seconds, shift / accel / band / expect / fuel / stay, = score), to see WHICH term keeps a gear (observation only)
        static const double tr_dt = getenv("SW_TERMS") ? atof(getenv("SW_TERMS")) : 0.0;
        static double tr_next = -1e9;
        if (tr_dt > 0 && t_now - SW.t0 >= tr_next) {
            tr_next = t_now - SW.t0 + tr_dt;
            double at = -1; const gc_out *gco = ts_last_choice(&at);
            printf("LT t=%6.1f %6.1f km/h gear %2d rpm %5.0f pedal %.2f cc %.2f | layers at %.1f best %d why %d lift %d lvl %.0f N\n", t_now - SW.t0, kmh, M.gear, rpm_now(), M.pedal, M.cc, at - SW.t0, gco ? gco->best : -1, gco ? gco->why : -1, gco ? gco->lift : -1, gco ? gco->level_n : 0.0f);
            if (gco) { const demand_view *dv = demand_now(); printf("LT hs %.2f hold %.0f bs %.2f ur %.2f need %.2f | demand p %.2f %.2f %.2f %.2f | grade %.2f\n", gco->hold_share, gco->hold_n, gco->band_scale, gco->urgency, gco->need, dv->p[0], dv->p[1], dv->p[2], dv->p[3], M.grade_pct); }
            if (gco)
                for (int g = M.gear - 2; g <= M.gear + 3; g++) {
                    if (g < 1 || g > NG) continue;
                    const gc_term *t = &gco->t[g];
                    printf("LT    g%-2d %s%s rpm %5.0f %-4s | sh %6.2f acc %6.2f band %6.2f exp %6.2f fuel %6.2f tgt %6.2f stay %5.2f = %7.2f  (%.3f L)\n", g, t->candidate ? "cand" : t->qualifies ? "qual" : "    ", g == gco->best ? "*" : " ", t->rpm, gc_band_name(t->band), t->shiftc, t->accc, t->bandc, t->expc, t->fuelc, t->tgtc, t->stay, t->score, t->fuel_l);
                }
        }
    }
    if (SW.ramp && kmh >= 50 && kmh < 114 && M.gear > 0) { int b = (int)((kmh - 50) / 2); if (b >= 0 && b < 32) { SW.bt[b] += dt; if (M.gear == NG) SW.bt_top[b] += dt; } }
    SW.vmin = fminf(SW.vmin, kmh); SW.vprev = SW.vlast; SW.vlast = kmh; SW.all_t += dt;
    if (SW.reached && kmh - SW.hit_kmh > SW.v_over) SW.v_over = kmh - SW.hit_kmh;
    if (SW.t50 <= 0 && kmh >= 50) SW.t50 = t_now - SW.t0;
    if (SW.t70 <= 0 && kmh >= 70) { SW.t70 = t_now - SW.t0; SW.fuel70 = SW.fuel_g; }
    if (SW.t80 <= 0 && kmh >= 80) { SW.t80 = t_now - SW.t0; SW.fuel80 = SW.fuel_g; }
    { float r_up = rpm_now();
      if (M.gear != SW.gear_prev) {
          { static const bool lsh = getenv("SW_LSH") != NULL; if (lsh && M.gear > 0) printf("LSH %.2f %d %d %.1f\n", t_now - SW.t0, SW.gear_prev, M.gear, kmh); }
          sw_up_commit();
          if (SW.gear_prev > 0 && M.gear > SW.gear_prev && !M.state) { SW.up_active = 1; SW.up_min = r_up; SW.up_t0 = t_now; }
          SW.gear_prev = M.gear;
      } else if (SW.up_active) {
          if (t_now - SW.up_t0 > 5.0) sw_up_commit();
          else if (!M.state && r_up < SW.up_min) SW.up_min = r_up;
      } }
    if (SW.hit_kmh > 0 && kmh >= SW.hit_kmh && !SW.latched) {
        if (SW.dist_m > 0) { if (!SW.reached) { SW.reached = 1; SW.t_speed = t_now - SW.t0; SW.d_speed = M.s - SW.s0; } }   // (SW_DIST_M: the speed is not where the run stops; the driver holds it, the distance stops the run below)
        else { SW.latched = 1; SW.t_hit = t_now - SW.t0; }
    }
    if (SW.hit_m > 0 && M.s - SW.s0 >= SW.hit_m && !SW.latched) { SW.latched = 1; SW.t_hit = t_now - SW.t0; }   // (a distance to cover: coast)
    // fuel as the game burns it (fuelcheck 29-10, ride logs of a Renault T and the Scania V8): proportional to the engine's work plus a friction
    // torque of ~0.09 x its peak torque (both at the rpm of now); in gear with the throttle shut nothing (the overrun cut); out of gear or in a
    // shift the idle: the friction at idle rpm. 220 g/kWh (the absolute scale does not matter, the sweeps compare ratios)
    static const float tf_k = getenv("SW_TF") ? (float)atof(getenv("SW_TF")) : 0.09f;   // SW_TF: the friction share (fit 0.054-0.12)
    const float TF = tf_k * TORQUE;
    if (M.gear < 1 || M.state) { SW.fuel_g += SW_IDLE * 0.10472f * TF / 1000 * 220 / 3600 * dt; return; }
    engine_info e; gb_engine(0, &e);
    float r = rpm_now(), idle = SW_IDLE, thr = (M.wish_on ? M.wish_thr : fmaxf(M.pedal, M.cc)) * sim_thr_cap();
    float f = r > LIMIT ? 0 : curve_factor(&e, r < idle ? idle : r);
    float P = TORQUE * f * thr * r * 0.10472f / 1000;   // kW at the flywheel
    SW.kj += P * dt;
    if (thr >= 0.02f) { float Pp = TORQUE * f * plant_thr(thr) * r * 0.10472f / 1000; SW.fuel_g += (Pp + r * 0.10472f * TF / 1000) * 220 / 3600 * dt; }
    int b = (int)(r / 10); if (b > 399) b = 399; if (b < 0) b = 0;
    SW.hist[b] += dt; SW.rpm_s += r * dt; SW.rpm_t += dt; if (M.gear == NG) SW.top_t += dt;
    if (r > SW.rpm_max) SW.rpm_max = r;   // (the highest rpm in gear, the last column of the SW line)
    // gear segments under a steady pedal: the change of the engine power and of the wheel torque from the landing (+0.3 s) to the last sample
    // before the next upshift (the 'more power low in the revs than high' check of the torque pedal's rev_gain)
    if (V.plateau > 0) {
        if (thr > 0.9f && r < V.plateau - 50) { SW.lug_all_t += dt; if (kmh > 30) SW.lug_t += dt; }
        if (kmh > 30 && thr > 0.5f && r < V.plateau - 50) SW.lug2_t += dt;
    }
    { float tq = TORQUE * f * plant_thr(thr), pw = tq * r;   // (the plant's: SW_THR_K)
      if (M.gear != SW.seg_g) {
          if (M.gear > SW.seg_g && SW.seg_g > 0 && SW.seg_n0 >= 1.5 && SW.seg_tq0 > 1 && SW.seg_pw0 > 1 && SW.nseg < 64) {
              SW.seg_dtq[SW.nseg] = SW.seg_tq1 / SW.seg_tq0 - 1; SW.seg_dpw[SW.nseg] = SW.seg_pw1 / SW.seg_pw0 - 1; SW.nseg++; }
          SW.seg_g = M.gear; SW.seg_t = t_now; SW.seg_tq0 = SW.seg_pw0 = 0; SW.seg_n0 = 0;
      } else if (t_now - SW.seg_t >= 0.3) {
          if (SW.seg_tq0 <= 0) { SW.seg_tq0 = tq; SW.seg_pw0 = pw; SW.seg_t0 = t_now; }
          SW.seg_tq1 = tq; SW.seg_pw1 = pw; SW.seg_n0 = (float)(t_now - SW.seg_t0); } }
    static double tr_next; static const bool tr = getenv("SW_TRACE") != NULL;   // SW_TRACE: a line every 0.5 s
    if (tr && t_now - SW.t0 >= tr_next) { tr_next = t_now - SW.t0 + 0.5; { float kd = 0; const float tg = ts_target_rpm(&kd); printf("TR %6.1f %5.1f km/h g%d %4.0f rpm thr %.2f f %.2f cap %.2f tgt %.0f kd %.2f\n", t_now - SW.t0, kmh, M.gear, r, thr, f, sim_thr_cap(), tg, kd); } }
}
static float sw_grade, sw_vt;
// SW_WISH: the driver's pedal is the torque pedal ([pedal] wish 1, as Damien drives): the real modules/pedal.cpp gives the engine the throttle that delivers its
// demand in this gear (sim_pedal_wish in sim.cpp); not with the cruise control (cruise_kmh set), not at pulling-away speed, not during a shift. [pedal] keys:
// pd_<key>=v in SCP or PMP="key=v,..."
static const bool sw_wish_on = getenv("SW_WISH") != NULL;
static void sw_wish() { sim_pedal_wish(); }
static int sw_start_gear(float kmh, float want)
{
    int best = 1; float be = 1e9f;
    for (int g = 1; g <= NG; g++) { float r = kmh / 3.6f / RADIUS * RATIO[g-1] * DIFF * 9.5493f; if (r < LIMIT * 0.9f && fabsf(r - want) < be) { be = fabsf(r - want); best = g; } }
    return best;
}
static void sw_c_acc(double) { M.grade_pct = sw_grade; M.pedal = M.v < 88 / 3.6f ? 1.0f : M.v < 90 / 3.6f ? 0.3f : 0.0f; }   // full throttle up to the game's 90 km/h limiter
static float sw_ped = getenv("SW_PED") ? (float)atof(getenv("SW_PED")) : 0.6f;
static void sw_c_pull(double) { M.grade_pct = sw_grade; M.pedal = sw_ped; }
static void sw_c_ovt2(double t);
// launch: a pull-away from a standstill at a fixed SHARE of the torque pedal (SW_SCEN=launchNN, NN = the shaped share in percent; the foot is wish_unmap of it in the mode), to SW_HIT km/h (20): the truck
// stands in neutral for SW_LAUNCH_AT s (0.5), the pedal comes up over SW_RAMP s (1.0) and stays; the plugin's own start path picks and engages the gear (bs_start, launch_correct) as in the game, and the
// plant slips at the idle. SW_LSH=1 prints "LSH t from to km/h" at every gear change (the engage 0 > g first); the SW line has the seconds (t_hit, from the start of the run: minus SW_LAUNCH_AT)
// and the fuel proxy to the speed (the run is counted from the pedal's first move)
static float sw_launch_share = 0.3f;
static int sw_launch_md = MODE_NORMAL;
static void sw_c_launch(double t)
{
    static const float at = getenv("SW_LAUNCH_AT") ? (float)atof(getenv("SW_LAUNCH_AT")) : 0.5f, ramp = getenv("SW_RAMP") ? (float)atof(getenv("SW_RAMP")) : 1.0f;
    M.grade_pct = sw_grade;
    M.pedal = t < at ? 0.0f : wish_unmap(sw_launch_share, sw_launch_md) * fminf(1.0f, (float)(t - at) / fmaxf(ramp, 0.01f));
}
// coast: the driver lifts at v0 on the grade SW_GRADE (a descent: negative) for the first 1500 m, then holds v0 on the flat to 3000 m (the
// calm driver); fuel and time over the 3 km. Compares rolling in neutral (eco-roll) with coasting in gear (no fuel, engine drag) fairly:
// the speed the roll keeps is fuel the driver does not spend after it
static float sw_v0;
static void sw_driver(float kmh);
static bool sw_coast_on;
// (the driver gets back on the pedal after 1500 m or once the speed is 15 km/h under where he lifted, whichever comes first: a flat
// lift at 50 km/h otherwise rolled to a standstill and stood idling forever, which no driver does)
static void sw_c_coast(double t) { if (t <= 0) sw_coast_on = true; if (sw_coast_on && (M.s >= 1500 || M.v < (sw_v0 - 15) / 3.6f)) sw_coast_on = false;
    if (sw_coast_on) { M.grade_pct = sw_grade; M.pedal = t < 0.2 ? 0.05f : 0; /* (a touch at the start: the driver held v0 before he lifted) */ } else { float g = sw_grade; sw_grade = M.s < 1500 ? g : 0; sw_driver(sw_v0); sw_grade = g; } }
static void sw_c_ovt(double t) { M.grade_pct = sw_grade; if (t < 12) hold(60); else M.pedal = M.v < 88 / 3.6f ? 1.0f : M.v < 90 / 3.6f ? 0.3f : 0.0f; }
// a calm driver: PI on the speed with damping on the acceleration (the sim's hold() swings the pedal 0.05-0.75 every 10 s, which no driver does)
static float sw_int, sw_pv;
static void sw_driver(float kmh)
{
    M.grade_pct = sw_grade;
    float e = kmh / 3.6f - M.v, a = (M.v - sw_pv) * 60; sw_pv = M.v;
    sw_int = fminf(0.8f, fmaxf(0, sw_int + 0.02f * e / 60));
    M.pedal = fminf(1, fmaxf(0, sw_int + 0.10f * e - 0.25f * a));
    if (e < -1.5f) M.pedal = 0;
}
static void sw_c_hold(double) { sw_driver(sw_vt); }
// the hand-over from a scenario's own pedal to the calm driver (SW_DIST_M). The driver starts from where the foot is, not from zero: sw_pv = 0 reads the speed as an acceleration of 1000 m/s2 (a lift for one
// tick), and an integrator at zero sags the speed for half a minute. A pedal held as it is (a full pedal) overshoots the target and chatters on the driver's 1.5 m/s switch. The foot that holds the
// speed comes from the force balance of the last tick: the engine's force is about proportional to the foot, so the foot that cancels the acceleration is the foot times Fres / (Fres + m a)
// (Fres: the plant's own resistance, as tick() sums it, without the brake). The driver's PI takes over from there. Measured on 84 cells (2 engines, 7 scenarios, 2 masses, 3 modes): the speed over the
// target at most 0.2 km/h, at the end of the run at most 0.6 km/h under (a pedal held: 3.0 over; an integrator at zero: 2.4 under).
static void sw_hold_seed(double a_ms2)
{
    const float a = fmaxf(0.0f, (float)a_ms2), v = M.v, s = sinf(atanf(sw_grade / 100));
    const float Fres = M.mass * 9.81f * (s + 0.006f) + 0.5f * 1.2f * CDA * v * v + g_plant_res;
    sw_pv = v;
    sw_int = fminf(0.8f, fmaxf(0, M.pedal * Fres / (Fres + M.mass * a)));
}
// calm: pull away at the fixed pedal SW_PED (the launch pedal) to 80 km/h (the run stops there).
// (The start is 5 km/h, not 0: in the 32.04 first-gear boxes a true standstill in the crawl gear makes the sim's start logic skip
// 1->3 and hunt 3->2->1 forever, which is not a reserve effect.)
static void sw_c_calm(double) { M.grade_pct = sw_grade; M.pedal = sw_ped; }
// hard: a hard launch at the fixed pedal SW_PED (hard100 = 1.0, hard80 = 0.8) to 80 km/h
static void sw_c_hard(double) { M.grade_pct = sw_grade; M.pedal = sw_ped; }
static void sw_c_ovt2(double t) { if (t < 30) sw_driver(60); else { M.grade_pct = sw_grade; M.pedal = M.v < 88 / 3.6f ? 1.0f : M.v < 90 / 3.6f ? 0.3f : 0.0f; } }
// hwy: 85 km/h on the torque pedal over a gentle grade change 0 -> 1.5 % -> 0 (a 10 s ramp each way); the wish mostly holds the speed
static void sw_c_hwy(double t)
{
    float g = sw_grade, gg = sw_grade;
    if (t >= 60 && t < 70) g = gg + 1.5f * (float)(t - 60) / 10;
    else if (t >= 70 && t < 160) g = gg + 1.5f;
    else if (t >= 160 && t < 170) g = gg + 1.5f * (float)(170 - t) / 10;
    sw_grade = g; sw_driver(85); sw_grade = gg;
}
// climb: a long part-pedal grade (SW_PED, e.g. 0.35) at the start speed, 70 km/h, on SW_GRADE (3-4 %); the speed sags if the pedal cannot hold it
static void sw_c_climb(double) { M.grade_pct = sw_grade; M.pedal = sw_ped; }
static void sw_c_ramp(double t) { sw_driver(55 + 0.15f * (float)t); }
// the game's cruise control (proportional, as c_cci80): the driver's pedal is off, the suite's own cruise holds the speed
static void sw_cc(float kmh) { M.grade_pct = sw_grade; M.cruise_kmh = kmh; M.pedal = 0; float e = kmh / 3.6f - M.v; M.cc = fminf(1, fmaxf(0, 0.5f + e * 1.0f)); }
static void sw_c_cc(double) { sw_cc(sw_vt); }
// a rolling road: the grade swings +-1 % around SW_GRADE with a 600 m period (a 'flat' road in the game is seldom flat); the cruise control
// at sw_vt, or the calm driver (wave_d). Shows hunting between the top gear and the one under it ('zoeken')
static const float sw_wave_a = getenv("SW_WAVE") ? (float)atof(getenv("SW_WAVE")) : 1.0f;   // SW_WAVE: the swing of the rolling road (+- %)
static void sw_c_wave(double) { sw_cc(sw_vt); M.grade_pct = sw_grade + sw_wave_a * sinf((float)M.s * 6.2832f / 600.0f); }
static void sw_c_waved(double) { sw_driver(sw_vt); M.grade_pct = sw_grade + sw_wave_a * sinf((float)M.s * 6.2832f / 600.0f); }
static void sw_c_ccramp(double t) { sw_cc(55 + 0.15f * (float)t); }
// ---- RIDE (WP-G6): SW_SCEN=ride SW_RIDE=<file> replays a stretch of a real ride (tools/sim/ride_prep.py writes the file from the ride's trueshift.csv and dazdrive.log) on the real modules: the driver's pedal, brake and
// retarder lever as recorded, the road under the truck by DISTANCE (the route the plugin sees where the ride's log says its look-ahead was on), and the driver's demand history of the 600 s before the stretch (put in second by
// second, then fed live as in the game; the lifts and presses from the profile file). SW_RIDE_V=1 forces the speed as recorded (a diagnostic: the terms along the real speed, the gears the box picks on it); without it the
// plant drives the speed (the counterfactual: what a change does to the same pedal). Lines: "mass kg", "v0 km/h", "gear n", "profile path", "P t pedal brake kmh ret" (t from the start, rising), "R dist_m grade_pct" (rising),
// "O t_on t_off" (the route is known), "S share" (the history, oldest first).
struct ride_p { double t; float ped, brk, kmh, ret; };
struct ride_r { float d, g; };
static struct { int on, forced; double secs; float kmh0, mass; int gear; char profile[512]; std::vector<ride_p> p; std::vector<ride_r> r; std::vector<std::pair<double, double>> o; std::vector<float> hist; size_t i; } g_ride;
static void ride_load(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "SW_RIDE: cannot open %s\n", path); exit(2); }
    char ln[1024];
    while (fgets(ln, sizeof ln, f)) {
        double a, b, c, d, e;
        if (sscanf(ln, "mass %lf", &a) == 1) g_ride.mass = (float)a;
        else if (sscanf(ln, "v0 %lf", &a) == 1) g_ride.kmh0 = (float)a;
        else if (sscanf(ln, "gear %lf", &a) == 1) g_ride.gear = (int)a;
        else if (!strncmp(ln, "profile ", 8)) {   // (a relative path is the ride file's own directory: the committed fixtures, tools/sim/fixtures/ride/)
            char nm[512]; snprintf(nm, sizeof nm, "%s", ln + 8); nm[strcspn(nm, "\r\n")] = 0;
            const char *sl = strrchr(path, '/');
            if (nm[0] == '/' || !sl) snprintf(g_ride.profile, sizeof g_ride.profile, "%s", nm);
            else snprintf(g_ride.profile, sizeof g_ride.profile, "%.*s/%s", (int)(sl - path), path, nm);
        }
        else if (ln[0] == 'P' && sscanf(ln + 1, "%lf %lf %lf %lf %lf", &a, &b, &c, &d, &e) == 5) g_ride.p.push_back({ a, (float)b, (float)c, (float)d, (float)e });
        else if (ln[0] == 'R' && sscanf(ln + 1, "%lf %lf", &a, &b) == 2) g_ride.r.push_back({ (float)a, (float)b });
        else if (ln[0] == 'O' && sscanf(ln + 1, "%lf %lf", &a, &b) == 2) g_ride.o.push_back({ a, b });
        else if (ln[0] == 'S' && sscanf(ln + 1, "%lf", &a) == 1) g_ride.hist.push_back((float)a);
    }
    fclose(f);
    if (g_ride.p.size() < 2) { fprintf(stderr, "SW_RIDE: no pedal rows in %s\n", path); exit(2); }
    g_ride.secs = g_ride.p.back().t; g_ride.on = 1; g_ride.forced = getenv("SW_RIDE_V") != NULL;
}
static float ride_road(double s)   // the grade (%) at distance s from the start of the stretch
{
    const std::vector<ride_r> &r = g_ride.r;
    if (r.empty()) return 0.0f;
    if (s <= r.front().d) return r.front().g;
    if (s >= r.back().d) return r.back().g;
    size_t lo = 0, hi = r.size() - 1;
    while (hi - lo > 1) { size_t mid = (lo + hi) / 2; if (r[mid].d <= s) lo = mid; else hi = mid; }
    return r[lo].g;
}
static void sw_c_ride(double t)
{
    std::vector<ride_p> &p = g_ride.p;
    size_t &i = g_ride.i;
    while (i + 1 < p.size() && p[i + 1].t <= t) i++;
    const ride_p &a = p[i], &b = p[i + 1 < p.size() ? i + 1 : i];
    const float k = b.t > a.t ? (float)fmin(1.0, fmax(0.0, (t - a.t) / (b.t - a.t))) : 0.0f;
    M.pedal = a.ped + (b.ped - a.ped) * k;
    M.brake = a.brk + (b.brk - a.brk) * k;
    if (g_ride.forced) { M.v = (a.kmh + (b.kmh - a.kmh) * k) / 3.6f; M.ret = (int)(a.ret + 0.5f); }
    // SW_NOLEARN=1 (the target rows, R11): the speed is forced, so what the force learner would read off it (a pedal that does not hold the speed on a climb is no road: f_bias fell to -6.7 kN in 40 s)
    // is nothing to measure; the road stays the model's. (The learner still takes one step of ~10 N inside a frame: a target moves by 1 rpm at most.)
    { static const int nolearn = getenv("SW_NOLEARN") != NULL; if (g_ride.forced && nolearn) { V.f_bias = 0.0f; V.f_scale = 1.0f; } }
    // SW_RET=<step> / SW_EB=<level> / SW_EBTOG=1 (the descent rows, WP-1111-limiter): the driver's retarder lever, his engine brake (a level; 0 = off) and the game's "downshift with the engine brake" setting, held for the whole ride
    { static const int ret = getenv("SW_RET") ? atoi(getenv("SW_RET")) : -1; static const float eb = getenv("SW_EB") ? (float)atof(getenv("SW_EB")) : -1.0f; static const int tog = getenv("SW_EBTOG") ? atoi(getenv("SW_EBTOG")) : -1;
      if (ret >= 0 && !g_ride.forced && !getenv("SW_LEVER_HOLD")) M.ret = ret;
      // SW_LEVER_HOLD=<km/h>: a driver who holds that speed down a descent with the retarder lever (a step up when 2 km/h over it, a step down when 2 under, at most one move in 2 s), the pedal off: the descent rows
      static const float hold_kmh = getenv("SW_LEVER_HOLD") ? (float)atof(getenv("SW_LEVER_HOLD")) : 0.0f; static double last_move = -1e9;
      if (hold_kmh > 0.0f && !g_ride.forced && t_now - last_move >= 2.0) {
          const float kmh = M.v * 3.6f;
          if (kmh > hold_kmh + 2.0f && M.ret < 5) { M.ret++; last_move = t_now; }
          else if (kmh < hold_kmh - 2.0f && M.ret > 0) { M.ret--; last_move = t_now; }
      }
      if (eb >= 0.0f) { M.ebrake = eb > 0.0f; M.eb_level = eb; }
      if (tog >= 0) M.eb_toggle = tog; }
    M.route_on = 0;
    for (auto &o : g_ride.o) if (t >= o.first && t < o.second) M.route_on = 1;
}
// SW_DEC=1: one line per decision of the layers ("layers: a->b" in the log), at the moment it is made: DEC t a b rpm_of_a kmh pedal share (the rpm the box left, which the shift line, written when the shift is in,
// does not say: the engine keeps revving through the gap; the raw pedal, and the share the box saw: the pedal as the torque pedal shaped it, which the layers' line calls `pedal`). The G7 rows (g7_rows.py)
// judge the revs a calm pedal goes to by it.
static void sw_dec_tap(const char *line)
{
    static double last = -1e9; int a, b;
    if (strncmp(line, "layers: ", 8) || sscanf(line + 8, "%d->%d", &a, &b) != 2 || t_now - last < 0.01) return;
    last = t_now;
    float share = 0;
    for (const char *p = strstr(line, ", pedal "); p; p = strstr(p + 1, ", pedal ")) if (p[8] >= '0' && p[8] <= '9') { share = (float)atof(p + 8); break; }   // (on a lift ", pedal off" comes first)
    float kd = 0; const float tg = ts_target_rpm(&kd);   // (the target the decision was made against: the last layers run set it)
    printf("DEC %.2f %d %d %.0f %.1f %.2f %.2f tgt %.0f kd %.2f\n", t_now, a, b, vehicle_engine_rpm_at(a, M.v), M.v * 3.6f, M.pedal, share, tg, kd);
}
static void ride_begin()   // after the run set-up: the road, the driver's history and habits
{
    g_ride.i = 0;
    M.road = ride_road; route_build(ride_road);
    g_driver_feed = 1; driver_reset();
    if (g_ride.profile[0]) {
        FILE *f = fopen(g_ride.profile, "r");
        if (f) { std::string txt; char ln[8192]; while (fgets(ln, sizeof ln, f)) if (strncmp(ln, "demand=", 7)) txt += ln; fclose(f); driver_parse(txt.c_str()); }
    }
    for (float s : g_ride.hist) demand_note(s);
}
static struct { double t_hit, v_end, v_min, rpm_mean, rpm_p90, fuel_g, dist, top_frac, od_in, kwh, hi_frac; int shifts, hunts, skips; double seg_tq, seg_pw, lug, lug2, land_rpm, t50, t70, fuel70; double t80, fuel80, lug_s, up_rpm_min; int kd_up; double rpm_max; int hunts_drv; } SWR;   // the last sw_one()
static void sw_one(const char *eng, const char *scen, const char *mode, float mass, float grade)
{
    memset(&SW, 0, sizeof SW); sw_int = 0; sw_pv = 0; g_cruise_smooth = 0;
    sw_grade = grade;
    const bool ride = !strcmp(scen, "ride");
    if (ride) { if (!getenv("SW_RIDE")) { fprintf(stderr, "SW_SCEN=ride needs SW_RIDE=<file>\n"); exit(2); } ride_load(getenv("SW_RIDE")); if (g_ride.mass > 0) mass = g_ride.mass; }
    if (!strncmp(scen, "calm", 4) || !strncmp(scen, "climb", 5)) sw_ped = (float)atof(scen + (!strncmp(scen, "calm", 4) ? 4 : 5)) / 100.0f;
    if (!strncmp(scen, "hard", 4)) sw_ped = !strcmp(scen, "hard80") ? 0.8f : 1.0f;
    if (getenv("SW_FOOT")) sw_ped = (float)atof(getenv("SW_FOOT"));   // SW_FOOT: the raw foot of a fixed-foot scenario (calmNN, hardNN, pull, climbNN), whatever the name says: the bound rows ask every mode for the same WISH, which is a different raw foot per mode (the pedal map)
    float shift = sw_engine(eng);
    float cvar = !strcmp(mode, "eco") ? 10.0f : !strcmp(mode, "power") ? 1.6667f : 3.0f;
    { engine_info e; gb_engine(0, &e); float a = 1e9f, b = 0; for (int i = 0; i < e.n_curve; i++) if (e.curve[i][1] >= 0.97f && e.curve[i][0] > 300) { a = fminf(a, e.curve[i][0]); b = fmaxf(b, e.curve[i][0]); } SW.ropt = 0.5f * (a + b); }
    setup su; su.mass = mass; su.cvar = cvar; su.shift_time = shift;
    double win0 = 0; SW.hit_kmh = 0; void (*ctl)(double) = sw_c_acc; float v0 = 8, want = 1000;
    if (!strncmp(scen, "calm", 4)) { su.secs = 400; SW.hit_kmh = 80; ctl = sw_c_calm; v0 = 5; want = 1100; }
    else if (!strncmp(scen, "hard", 4)) { su.secs = 400; SW.hit_kmh = 80; ctl = sw_c_hard; v0 = 5; want = 1100; }
    else if (!strncmp(scen, "launch", 6)) { sw_launch_share = (float)atof(scen + 6) / 100.0f; sw_launch_md = !strcmp(mode, "eco") ? MODE_ECO : !strcmp(mode, "power") ? MODE_POWER : MODE_NORMAL;
        g_slip_k = getenv("SW_SLIP_K") ? (float)atof(getenv("SW_SLIP_K")) : CLUTCH_SLIP_K;   // (the game clutch slips at CLUTCH_SLIP_K x idle, core/vehicle.h, sim.cpp g_slip_k)
        su.secs = 60; win0 = getenv("SW_LAUNCH_AT") ? atof(getenv("SW_LAUNCH_AT")) : 0.5; SW.hit_kmh = getenv("SW_HIT") ? (float)atof(getenv("SW_HIT")) : 20.0f; ctl = sw_c_launch; v0 = 0; want = 1100; }
    else if (!strcmp(scen, "hwy")) { su.secs = 240; win0 = 0; ctl = sw_c_hwy; v0 = 85; want = 1300; }   // 85 km/h, 0 -> 1.5 -> 0 % grade
    else if (!strncmp(scen, "climb", 5)) { su.secs = 180; win0 = 0; ctl = sw_c_climb; v0 = 70; want = 1300; }   // a long part-pedal climb
    else if (!strcmp(scen, "acc")) { su.secs = 200; SW.hit_kmh = 80; ctl = sw_c_acc; v0 = 8; want = 1000; }
    else if (!strcmp(scen, "pull")) { su.secs = 150; win0 = 0; SW.hit_kmh = 80; ctl = sw_c_pull; v0 = 20; want = 1100; }   // a steady pedal SW_PED from 20 km/h until 80
    else if (!strcmp(scen, "ovt2")) { su.secs = 100; win0 = 30; SW.hit_kmh = 85; ctl = sw_c_ovt2; v0 = 60; want = 1300; sw_vt = 60; }   // 30 s steady at 60 with the calm driver, then a full pedal to 85
    else if (!strcmp(scen, "wave")) { su.secs = 300; win0 = 40; ctl = sw_c_wave; sw_vt = 85; v0 = 85; want = 1300; g_cruise_smooth = 1; }   // the cruise control at 85 on a rolling road
    else if (!strcmp(scen, "waved")) { su.secs = 300; win0 = 40; ctl = sw_c_waved; sw_vt = 85; v0 = 85; want = 1300; }   // the calm driver at 85 on a rolling road
    else if (!strncmp(scen, "coast", 5)) { int kmh = atoi(scen + 5); sw_v0 = kmh > 0 ? kmh : 70; su.secs = 400; win0 = 0; SW.hit_m = 3000; ctl = sw_c_coast; v0 = sw_v0; want = 1200; }   // coast50 / coast70 / coast85
    else if (!strcmp(scen, "ovt")) { su.secs = 100; win0 = 12; SW.hit_kmh = 85; ctl = sw_c_ovt; v0 = 60; want = 1300; }
    else if (!strcmp(scen, "hill")) { su.secs = 100; win0 = 5; ctl = sw_c_acc; v0 = 60; want = 1300; }
    else if (!strcmp(scen, "eco")) { su.secs = 160; win0 = 40; ctl = sw_c_hold; sw_vt = 80; v0 = 80; want = 1300; }
    else if (!strcmp(scen, "cr85d")) { su.secs = 180; win0 = 50; ctl = sw_c_hold; sw_vt = 85; v0 = 85; want = 1300; }   // the driver's pedal
    else if (!strcmp(scen, "rampd")) { su.secs = 400; win0 = 0; ctl = sw_c_ramp; v0 = 55; want = 1300; SW.ramp = 1; }
    else if (!strcmp(scen, "cr85")) { su.secs = 180; win0 = 50; ctl = sw_c_cc; sw_vt = 85; v0 = 85; want = 1300; g_cruise_smooth = 1; }   // the cruise control
    else if (!strcmp(scen, "ramp")) { su.secs = 400; win0 = 0; ctl = sw_c_ccramp; v0 = 55; want = 1300; SW.ramp = 1; g_cruise_smooth = 1; }
    else if (ride) { su.secs = g_ride.secs; win0 = 0; ctl = sw_c_ride; v0 = g_ride.kmh0; want = 1300; }
    // SW_DIST_M=<m>: the same distance for every mode. A scenario with a target speed does not stop there: the driver holds the speed (the calm driver, sw_driver) and the run stops once SW_DIST_M
    // metres are covered from the window start. t_hit = the seconds for the distance, fuel_g = the diesel over it; t80 / fuel80 stay the time and diesel to the speed. (hit_m is the same latch the
    // coast scenarios use.) The run gets the time to cover the distance at the target speed on top of its own length. Unset, nothing here runs.
    const double dist_m = getenv("SW_DIST_M") ? atof(getenv("SW_DIST_M")) : 0.0;
    if (dist_m > 0 && SW.hit_kmh > 0) { SW.dist_m = dist_m; SW.hit_m = (float)dist_m; su.secs += 20 + dist_m / (SW.hit_kmh / 3.6f); }
    if (getenv("SW_V0")) v0 = (float)atof(getenv("SW_V0"));       // SW_V0 / SW_WANT: the start speed (km/h) and the rpm the start gear is chosen for (a launch: 10 and 1600)
    if (getenv("SW_WANT")) want = (float)atof(getenv("SW_WANT"));
    int g0 = sw_start_gear(v0, want);
    if (ride && g_ride.gear > 0) g0 = g_ride.gear;
    if (!strncmp(scen, "launch", 6)) g0 = 0;   // (neutral at a standstill: the plugin's start path engages the gear)
    if (!strcmp(scen, "acc") || !strncmp(scen, "calm", 4) || !strncmp(scen, "hard", 4)) {   // the start gear: the highest that still pulls 0.2 m/s2 at full throttle at the start speed and is at 900 rpm or more
        for (; g0 > 1; g0--) { engine_info e; gb_engine(0, &e); float v = v0 / 3.6f, r = v / RADIUS * RATIO[g0-1] * DIFF * 9.5493f;
            float F = TORQUE * curve_factor(&e, r) * RATIO[g0-1] * DIFF * 0.9f / RADIUS, Fr = mass * 9.81f * (sinf(atanf(sw_grade / 100)) + 0.006f);
            if (r >= 900 && (F - Fr) / mass >= 0.2f) break; }
    }
    // the run() set-up
    g_quiet = getenv("SW_V") ? 0 : 1;
    memset(&M, 0, sizeof M);
    for (int o = 0; o < ACT_N; o++) g_act[o] = act_wish{0, 0, 1, 1, 1};
    T.speed_limit = getenv("SW_LIMIT") ? (float)atof(getenv("SW_LIMIT")) / 3.6f : 0.0f;   // SW_LIMIT: the road's limit (km/h) the plugin sees (0 = none)
    M.mass = su.mass; M.gear = M.target = g0; M.cvar = su.cvar; M.shift_time = su.shift_time; M.eb_level = -1; M.eb_toggle = -1; M.grade_pct = sw_grade;
    if (ride) ride_begin();
    g_sim_log_tap = getenv("SW_DEC") ? sw_dec_tap : NULL;
    M.v = 0; tick(1 / 60.0, 0);
    if (!strncmp(scen, "launch", 6)) { g0 = (int)s_start(SELF); if (g0 < 1) g0 = 1; if (getenv("SW_G0")) g0 = atoi(getenv("SW_G0")); }   // (SW_G0: the start gear forced, for the start-gear experiments)   // the start gear the plugin has ready at the standstill, the pedal still off (the game's log: "start gear N (game 1), pedal 0.00")
    M.state = 0; M.gear = M.target = g0; M.force = 0;
    M.v = v0 / 3.6f; tick(1 / 60.0, 0);
    M.state = 0; M.gear = M.target = g0; M.force = 0;
    M.v = v0 / 3.6f; shifts = ups = hunts = last_dir = 0; M.s = 0; g_rec_on = 1; g_rec_n = 0;
    // the fuel model as the game would have taught it by now (the learner needs minutes; a run is one): the plant's friction and drag
    if (!getenv("SW_NOPRIME")) vehicle_fuel_prime(g_sw_tf, g_sw_drag * 1100 / LIMIT / V.eta, 1100);
    double t0 = t_now; SW.t0 = t0; SW.vmin = 1e9f; SW.t_hit = -1; SW.latched = 0;
    int shifts0 = 0, hunts0 = 0, hdrv0 = 0; double s0 = 0; bool started = false;
    g_tick_hook = sw_hook;
    bool handed = false;
    while (t_now - t0 < su.secs) {
        if (SW.reached) {   // SW_DIST_M: the target speed is reached; the calm driver takes over and holds it
            if (!handed) { handed = true; sw_hold_seed((SW.vlast - SW.vprev) / 3.6 * 60.0); }
            sw_driver(SW.hit_kmh);
        } else ctl(t_now - t0);
        if (sw_wish_on) sw_wish();
        if (!started && t_now - t0 >= win0) { started = true; SW.on = 1; shifts0 = shifts; hunts0 = hunts; hdrv0 = hunts_drv; s0 = M.s; SW.s0 = M.s; }
        tick(1 / 60.0, 0);
        if (SW.latched) break;
    }
    SW.on = 0; g_rec_on = 0; sw_up_commit();
    double dist = M.s - s0; int nsh = shifts - shifts0, nhunt = hunts - hunts0;
    double od_in = 0;
    if (SW.ramp) {   // the lowest speed from which every 2 km/h bin (from 50 km/h) up has the top gear for at least half of its time
        for (int b = 31; b >= 0; b--) { if (SW.bt[b] < 0.5) continue; if (SW.bt_top[b] >= 0.5 * SW.bt[b]) od_in = 50 + 2 * b + 1; else break; }
    }
    float mt = NAN, mp = NAN;   // the medians of the gear segments
    if (SW.nseg > 0) { float a[64], b[64]; int n = SW.nseg; memcpy(a, SW.seg_dtq, n * sizeof(float)); memcpy(b, SW.seg_dpw, n * sizeof(float)); std::sort(a, a + n); std::sort(b, b + n); mt = a[n / 2]; mp = b[n / 2]; }
    double land_sum = 0; int land_n = 0, nskip = 0;   // the landing rpm of each upshift and the shifts that jump over a gear
    for (int i = 0; i < g_rec_n; i++) if (g_rec[i].b > g_rec[i].a) { land_sum += g_rec[i].rpm * RATIO[g_rec[i].b-1] / RATIO[g_rec[i].a-1]; land_n++; if (g_rec[i].b > g_rec[i].a + 1) nskip++; }
    int kd_up = 0; double up_at = -1e9; bool has_up = false;   // a downshift within 5 s of an upshift, by kickdown or lugging: a failed lug
    for (int i = 0; i < g_rec_n; i++) {
        if (g_rec[i].t < SW.t0) continue;
        if (g_rec[i].b > g_rec[i].a) { up_at = g_rec[i].t; has_up = true; }
        else if (g_rec[i].b > 0 && g_rec[i].b < g_rec[i].a && has_up && g_rec[i].t - up_at < 5.0 &&
                 (strstr(g_rec[i].why, "kickdown") || strstr(g_rec[i].why, "lugging"))) kd_up++;
    }
    SWR = { SW.latched ? SW.t_hit : -1.0, M.v * 3.6, SW.vmin, SW.rpm_t > 0 ? SW.rpm_s / SW.rpm_t : 0, sw_p90(), SW.fuel_g, dist, SW.rpm_t > 0 ? SW.top_t / SW.rpm_t : 0, od_in, SW.kj / 3600.0, sw_hi_frac(), nsh, nhunt, nskip, mt, mp, SW.rpm_t > 0 ? SW.lug_t / SW.rpm_t : 0, SW.rpm_t > 0 ? SW.lug2_t / SW.rpm_t : 0, land_n ? land_sum / land_n : 0, SW.t50 > 0 ? SW.t50 : -1, SW.t70 > 0 ? SW.t70 : -1, SW.t70 > 0 ? SW.fuel70 : -1, SW.t80 > 0 ? SW.t80 : -1, SW.t80 > 0 ? SW.fuel80 : -1, SW.lug_all_t, SW.up_min_n ? SW.up_min_sum / SW.up_min_n : 0, kd_up, SW.rpm_max, hunts_drv - hdrv0 };
    // SW_HOLD=1 (the limiter rows, WP-1111-limiter): the speed each gear holds at full throttle on the grade of the start of the ride, from the plant's own force and resistance (the oracle of the rows: the gear a calm foot
    // would choose at a limit is read off it). One line per gear: HOLD g km/h rpm (the highest speed at which the gear's force still meets the resistance, within the engine's rev limit).
    if (ride && !g_quiet && getenv("SW_HOLD")) {
        engine_info e; gb_engine(0, &e);
        const float gr0 = ride_road(0.0), sn = sinf(atanf(gr0 / 100));
        for (int g = 1; g <= NG; g++) {
            float vbest = 0, rbest = 0;
            for (float kmh = 5.0f; kmh < 160.0f; kmh += 0.05f) {
                const float v = kmh / 3.6f, r = v / RADIUS * RATIO[g-1] * DIFF * 9.5493f;
                if (r > LIMIT * 0.96f) break;
                const float F = TORQUE * curve_factor(&e, r < SW_IDLE ? SW_IDLE : r) * RATIO[g-1] * DIFF * g_plant_eta / RADIUS;
                const float Fr = M.mass * 9.81f * (sn + 0.006f) + 0.5f * 1.2f * CDA * v * v + g_plant_res;
                if (F >= Fr) { vbest = kmh; rbest = r; }
            }
            // ... and the throttle the gear needs to hold the limit (SW_TOP; 9.9 where its revs at the limit are outside the engine's window): the oracle of the rows
            float need = 9.9f, rtop = 0;
            { const float topv = getenv("SW_TOP") ? (float)atof(getenv("SW_TOP")) : 0.0f, v = topv / 3.6f; const eng_bands *bb = bands_now();
              rtop = v / RADIUS * RATIO[g-1] * DIFF * 9.5493f;
              if (topv > 1.0f && rtop >= bb->floor && rtop <= LIMIT * 0.96f) {
                  const float F = TORQUE * curve_factor(&e, rtop < SW_IDLE ? SW_IDLE : rtop) * RATIO[g-1] * DIFF * g_plant_eta / RADIUS;
                  need = (M.mass * 9.81f * (sn + 0.006f) + 0.5f * 1.2f * CDA * v * v + g_plant_res) / F; } }
            printf("HOLD %d %.2f %.0f need %.3f rpm_top %.0f\n", g, vbest, rbest, need, rtop);
        }
    }
    if ((ride || !strncmp(scen, "launch", 6)) && !g_quiet) { const eng_bands *bb = bands_now(); printf("BANDS floor %.0f p1 %.0f p2 %.0f cap %.0f p3 %.0f\n", bb->floor, bb->p1, bb->p2, bb->cap, bb->p3); printf("MODEQ eco %.4f normal %.4f power %.4f tau %.4f %.4f %.4f\n", ts_mode_q(MODE_ECO), ts_mode_q(MODE_NORMAL), ts_mode_q(MODE_POWER), pt_mode_tau(ts_mode_q(MODE_ECO)), pt_mode_tau(ts_mode_q(MODE_NORMAL)), pt_mode_tau(ts_mode_q(MODE_POWER))); }   // (the ride and launch rows read the bands the box ran on)
    g_tick_hook = NULL; g_cruise_smooth = 0; if (ride) g_driver_feed = 0;
}
static int run_sweep()
{
    const char *eng = getenv("SW_ENG") ? getenv("SW_ENG") : "renault", *scen = getenv("SW_SCEN") ? getenv("SW_SCEN") : "acc", *mode = getenv("SW_MODE") ? getenv("SW_MODE") : "normal";
    float mass = getenv("SW_MASS") ? (float)atof(getenv("SW_MASS")) : 30000, grade = getenv("SW_GRADE") ? (float)atof(getenv("SW_GRADE")) : 0;
    if (getenv("SW_FEED")) g_driver_feed = 1;   // the foot's history as the plugin keeps it (driverstore.cpp driver_frame): a fresh profile, then the demand ring and the learners fed once a second; unset = the empty memory the sweeps always had
    sw_one(eng, scen, mode, mass, grade);
    printf("SW,%s,%s,%s,%.0f,%g,%.1f,%.2f,%.1f,%.0f,%.0f,%d,%.1f,%.0f,%.3f,%.1f,%.3f,%.3f,%d,%.3f,%.3f,%.4f,%.4f,%.0f,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.0f,%d,%.0f,%d\n", scen, eng, mode, mass / 1000, grade, SWR.t_hit,
           SWR.v_end, SWR.v_min, SWR.rpm_mean, SWR.rpm_p90, SWR.shifts, SWR.fuel_g, SWR.dist, SWR.top_frac, SWR.od_in, SWR.kwh, SWR.hi_frac, SWR.hunts, SWR.seg_tq, SWR.seg_pw, SWR.lug, SWR.lug2, SWR.land_rpm, SWR.skips, SWR.t50, SWR.t70, SWR.fuel70, SWR.t80, SWR.fuel80, SWR.lug_s, SWR.up_rpm_min, SWR.kd_up, SWR.rpm_max, SWR.hunts_drv);
    if (SW.dist_m > 0) printf("SWD,%.2f,%.1f,%.2f,%.3f\n", SW.reached ? SW.t_speed : -1.0, SW.reached ? SW.d_speed : -1.0, SW.v_over, SWR.t_hit);   // SW_DIST_M: seconds and metres to the target speed (-1: not reached), the highest speed over the target after the hand-over (km/h), the seconds for the distance to the tick (the SW line has it to 0.1 s)
    return 0;
}
