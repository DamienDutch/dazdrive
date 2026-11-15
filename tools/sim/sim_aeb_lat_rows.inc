// Rows of the AEB's sideways reading (core/aeblat.h; rides of 14-11: the AEB braked FULL for the trailer of the truck just passed, a car standing 3.8 m beside the
// lane, a truck in the next lane, all offered by the game's radar, with the driver on the gas and/or the indicator on). Included by sim_cruise.cpp after aeb_run
// (the plant, AE, radar_stub and the traffic stub g_sim_ta are its).
//
// The world: a road (straight or one steady bend) with our truck on it and other vehicles; the truck follows a scripted lane change (a cosine in y), the radar offers the
// nearest vehicle within a cone beside our position on the road (as the rides showed: targets 2.4 - 4.3 m to the side), the traffic list is read every 200 ms (its age is part of
// the test). Ground truth: every scenario runs twice, with the AEB and without it; without it the truck keeps its speed and the true clearance to every box is
// known (the closest approach). A FULL braking in a scenario that is clear without it (>= 0.3 m at the closest) is false braking; a scenario that hits without it must
// be braked for. The sweeps count both, x of y.
#pragma once
#include <vector>

struct lw_veh {
    float s0, y0;          // at t = 0: the middle of the box, s ahead of our placement origin (m, along the road), y to the left of the road's middle (m)
    float kmh, L, W;       // speed along the road, box length and width
    unsigned id;
    float stop_t = -1;     // brakes to a stop (4 m/s2) from this time
    float y1 = 0, y_t0 = -1, y_dur = 2.0f;   // a lateral move: from y0 to y1 from y_t0 over y_dur s (a cosine): a cut-in, a lead leaving
    int radar_late = 0;    // the radar only takes it once its gap is under 1 m (the trailer we pass, picked up at its corner)
    float s = 0, y = 0, v = 0, yprev = 0, hd = 0;   // state (hd: its heading against the road, from its sideways move)
};

struct lw_scn {
    float mass = 26000, kmh = 85, kappa = 0;   // our start speed; the road's curvature (1/m, + = left)
    std::vector<lw_veh> veh;
    float lc_t0 = -1, lc_dur = 2.2f, lc_dy = 3.7f;   // our lane change: start, duration, sideways (+ = left)
    float ind_t0 = -1; int ind_side = 1;       // the indicator
    float thr_t0 = -1, thr = 1.0f;             // the pedal from this time (else 0.3, a driver who does not react)
    int list = 1;                              // the traffic list is there (0 = not found: today's behavior)
    float cone = 4.5f;                         // the radar takes a vehicle this far beside us on the road (as the rides showed it offer targets 2.4 - 4.3 m to the side)
    float t_end = 14;
    int aeb = 1;                               // 0 = the reference run
    int mode = getenv("LWMODE") ? atoi(getenv("LWMODE")) : 1;   // aeb_mode (1 = the shipped default; LWMODE=0 runs the same rows with the time stages)
};

struct lw_res {
    int max_st; float max_req, min_gap, end_kmh, peak_dec, min_true_clr, w_at_first, clr_at_first; int hit, gas_lines;
    double t_first, t_full;
};

static void lw_point(float k, float s, float y, double *x, double *yg)
{
    if (fabsf(k) < 1e-6f) { *x = s; *yg = y; return; }
    const double R = 1.0 / k, ph = k * s;
    *x = (R - y) * sin(ph); *yg = R - (R - y) * cos(ph);
}

static lw_res lw_run(const lw_scn &sc0, const char *name, int print)
{
    lw_scn sc = sc0;
    memset(GA, 0, sizeof GA); memset(&C, 0, sizeof C); memset(&OA, 0, sizeof OA); memset(&s_radar, 0, sizeof s_radar);
    memset(&AE, 0, sizeof AE); AE.gap_prev = AE.rate = AE.vl_prev = NAN; s_req_ours = 0; s_want_aeb = 0; s_aeb_was = 0; s_br_at = 0; s_air_low = 0;
    p_smooth = 1; p_own_acc = 1; g_radar_ok = 1; o_radar = fake_radar; g_low_ok = 1; g_level = 1; g_sg_ok = 0; p_ah = 0; p_hill = 0;
    o_cruise = stock_cruise; stock_thr = 0; s_aeb_owns = sc.aeb; g_sim_mass = sc.mass; g_brake_learn = 0;
    const float mode_was = p_aeb_mode; p_aeb_mode = (float)sc.mode;
    T.engine_on = 1; T.gear = 8; T.dgear = 8; T.eff_clutch = 0; T.parking_brake = 0; T.in_brake = 0; T.in_thr = 0; T.air_warning = 0; T.paused = 0;
    memset(&g_sim_ta, 0, sizeof g_sim_ta);
    g_sim_ta.truck_front_z = g_sim_ta.truck_rear_z = g_sim_ta.truck_hook_z = g_sim_ta.trailer_hook_z = g_sim_ta.trailer_rear_z = NAN;
    const float HW = 1.275f, FRONT = 2.85f;   // our half width, the radar's start: our front from the placement origin (the 14-11 truck: the list's body says 2.8, the radar 2.85)
    for (auto &v : sc.veh) { v.s = v.s0; v.y = v.y0; v.yprev = v.y0; v.hd = 0; v.v = v.kmh / 3.6f; }
    const double t0 = t_now;
    double so = 0, yo = 0, v = sc.kmh / 3.6, brk = 0, psi = 140, a_f = 0, th_prev = 0, last_read = -1;
    double dt = 1 / 60.0;
    lw_res r = { 0, 0, 1e9f, 0, 0, 1e9f, 1, 99, 0, 0, -1, -1 };
    int gas_lines0 = g_gas_lines;
    int first_seen = 0;
    for (; t_now - t0 < sc.t_end; t_now += dt) {
        const double t = t_now - t0;
        // our lane change: y(t) a cosine, the heading from its slope
        double yt = 0, yd = 0;
        if (sc.lc_t0 >= 0 && t > sc.lc_t0) {
            double u = fmin(1.0, (t - sc.lc_t0) / sc.lc_dur);
            yt = sc.lc_dy * (1 - cos(M_PI * u)) / 2;
            yd = u < 1 ? sc.lc_dy * M_PI * sin(M_PI * u) / 2 / sc.lc_dur : 0;
        }
        const double th = atan2(yd, fmax(v, 1.0)), thd = (th - th_prev) / dt; th_prev = th;
        const double kap = sc.kappa;
        yo = yt;
        so += v * cos(th) / (1 - kap * yo) * dt;
        const double psi_o = kap * so + th;
        const double yaw_rate = kap * v + thd;   // rad/s: the road's turn we follow, and our lane change
        double ox, oy;
        lw_point((float)kap, (float)so, (float)yo, &ox, &oy);
        // the others
        for (auto &o : sc.veh) {
            if (o.stop_t >= 0 && t > o.stop_t) o.v = fmaxf(0.0f, o.v - 4.0f * (float)dt);
            o.s += o.v * (float)dt;
            if (o.y_t0 >= 0 && t > o.y_t0) { double u = fmin(1.0, (t - o.y_t0) / o.y_dur); o.y = o.y0 + (o.y1 - o.y0) * (float)((1 - cos(M_PI * u)) / 2); }
            o.hd = o.v > 1 ? atan2f((o.y - o.yprev) / (float)dt, o.v) : 0.0f; o.yprev = o.y;
        }
        struct rel { float dx, dy, hdg, gap; int ok; };
        std::vector<rel> rl(sc.veh.size());
        float best_gap = 3.4e38f, best_v = 0; int best = -1;
        for (size_t i = 0; i < sc.veh.size(); i++) {
            const auto &o = sc.veh[i];
            double lx, ly;
            lw_point((float)kap, o.s, o.y, &lx, &ly);
            const double ddx = lx - ox, ddy = ly - oy;
            const double cx = cos(psi_o), sx = sin(psi_o);
            rel q;
            q.dx = (float)(ddx * cx + ddy * sx); q.dy = (float)(-ddx * sx + ddy * cx);
            q.hdg = (float)((kap * o.s + o.hd - psi_o) * 57.29577951308232);
            q.gap = q.dx - 0.5f * o.L - FRONT;
            q.ok = 1;
            rl[i] = q;
            // the true space: nobody brakes in the reference run, this is the closest approach
            const float h = q.hdg * 0.017453292f;
            const float ext = 0.5f * o.W * fabsf(cosf(h)) + 0.5f * o.L * fabsf(sinf(h));
            const float lat = fabsf(q.dy) - HW - ext;
            const float lon = q.dx - 0.5f * o.L - FRONT;                         // our front to its rear
            const float lon2 = q.dx + 0.5f * o.L + 6.0f;                          // its front to our rear (6 m behind the origin: a tractor and a bit)
            if (lon < 0 && lon2 > 0) { r.min_true_clr = fminf(r.min_true_clr, lat); if (lat < 0) r.hit = 1; }
            if (lon < 0 && lon2 > 0 && lat < 0 && !r.hit) r.hit = 1;
            const float seen_gap = q.gap;
            if (seen_gap > 0.3f && seen_gap < 100 && fabsf(o.y - (float)yo) < sc.cone && (!o.radar_late || seen_gap < 1.0f) && seen_gap < best_gap) { best_gap = seen_gap; best_v = o.v; best = (int)i; }
        }
        // the traffic list: read every 200 ms, as the game's walk
        if (sc.list && t - last_read >= 0.2 - 1e-9) {
            last_read = t;
            g_sim_ta.ok = 1; g_sim_ta.read_t = t_now; g_sim_ta.n_ahead = 0;
            g_sim_ta.bs_tf = 2.8f; g_sim_ta.bs_hw = HW;
            for (size_t i = 0; i < sc.veh.size() && g_sim_ta.n_ahead < 16; i++) {
                const auto &o = sc.veh[i];
                if (!(rl[i].dx > -15 && rl[i].dx < 130 && fabsf(rl[i].dy) < 25)) continue;
                traffic_obj &q = g_sim_ta.ahead[g_sim_ta.n_ahead++];
                q.dx = rl[i].dx; q.dy = rl[i].dy; q.dist = hypotf(rl[i].dx, rl[i].dy); q.v = o.v; q.hdg = rl[i].hdg; q.len = o.L; q.wid = o.W; q.kind = 0; q.id = o.id;
                q.rv_long = o.v - (float)v; q.rv_lat = 0;
            }
        } else if (!sc.list) { g_sim_ta.ok = 0; }
        // the driver
        float pedal = 0.3f;
        if (sc.thr_t0 >= 0 && t >= sc.thr_t0) pedal = sc.thr;
        GA[0x1d0] = sc.ind_t0 >= 0 && t >= sc.ind_t0 ? (sc.ind_side > 0 ? 4 : 8) : 0;
        const double yaw_deg = yaw_rate * 57.29577951308232;
        T.yaw = (float)(yaw_rate / (2 * M_PI)); T.in_steer = (float)(0.008 * yaw_deg);
        T.in_thr = pedal; T.in_brake = 0;
        T.speed = (float)v; T.accel = (float)a_f; T.accel_fast = (float)a_f; T.air_psi = (float)psi; T.place.orientation.pitch = 0;
        if (best >= 0) { g_fake_radar[0] = best_gap; g_fake_radar[1] = best_v; }
        else { g_fake_radar[0] = 3.4e38f; g_fake_radar[1] = 0; }
        g_fake_radar[2] = 3.4e38f; g_fake_radar[3] = 0; g_fake_radar[4] = 0;
        radar_stub(GA, g_fake_radar);
        vehicle_frame();
        float u = act_stub(GA, (float)dt, pedal);
        float req = *(float *)(GA + GA_BRAKE_REQ);
        r.max_req = fmaxf(r.max_req, req);
        brk = req > brk ? fmin(req, brk + 5 * dt) : fmax(req, brk - 5 * dt);
        double applied = fmax(brk * 10 / fmax(1.0, sc.mass / 30000.0), 0.0);
        if (applied > 0) psi = fmax(20.0, psi - applied * dt * 0.5);
        psi = fmin(140.0, psi + 4 * dt);
        if (req > 0) u = 0;
        double a = (v > 0.01 ? -0.4 : 0) + u * 20000.0 / sc.mass - (v > 0.01 ? applied : 0);
        if (v <= 0.01 && a < 0) a = 0;
        v = fmax(0, v + a * dt);
        a_f += (a - a_f) * fmin(1, dt / 0.15);
        r.peak_dec = fmaxf(r.peak_dec, (float)-a);
        if (getenv("LWDBG") && print && (int)(t * 60 + 0.5) % 12 == 0) printf("   t %5.2f v %5.1f kmh yo %+5.2f yaw %+5.1f | %s gap %6.2f dx %6.2f dy %+6.2f hdg %+5.1f | st %d a_need %6.2f req %.2f%s\n", t, v * 3.6, yo, yaw_deg, best >= 0 ? "radar" : "none ", best >= 0 ? best_gap : -1.0f, sc.veh.empty() ? 0 : rl[0].dx, sc.veh.empty() ? 0 : rl[0].dy, sc.veh.empty() ? 0 : rl[0].hdg, AE.st, AE.a_need, req, "");
        if (best >= 0) r.min_gap = fminf(r.min_gap, best_gap);
        if (AE.st > r.max_st && AE.st <= AEB_FULL) r.max_st = AE.st;
        if (AE.st >= AEB_WARN && AE.st <= AEB_FULL && !first_seen) {
            first_seen = 1; r.t_first = t;
#ifdef DAZ_AEBLAT
            r.w_at_first = AE.lat_ok ? AE.lat_w : 1.0f; r.clr_at_first = AE.lat_ok ? AE.lat_clr : 99.0f;
#endif
        }
        if (AE.st == AEB_FULL && r.t_full < 0) r.t_full = t;
    }
    r.end_kmh = (float)(v * 3.6);
    r.gas_lines = g_gas_lines - gas_lines0;
    if (r.min_gap > 1e8f) r.min_gap = 0;
    if (print) printf("%-62s max %-15s request %.2f, first at %5.1f s (clearance %+5.1f m, asked with %3.0f %%), FULL at %5.1f s, true closest clearance %+5.1f m%s, we %.0f -> %.0f km/h\n",
                      name, aeb_name[r.max_st], r.max_req, r.t_first, r.clr_at_first, r.w_at_first * 100, r.t_full, r.min_true_clr < 1e8f ? r.min_true_clr : 99.0f, r.hit ? " HIT" : "", sc.kmh, r.end_kmh);
    s_aeb_owns = 0; g_sim_mass = 26000; T.in_steer = 0; T.yaw = 0; GA[0x1d0] = 0; p_aeb_mode = mode_was;
    memset(&g_sim_ta, 0, sizeof g_sim_ta);
    return r;
}

// the pure reading (core/aeblat.h) on a made-up list
#ifdef DAZ_AEBLAT
static traffic_around lw_list(double read_t, std::initializer_list<traffic_obj> objs)
{
    traffic_around ta; memset(&ta, 0, sizeof ta);
    ta.ok = 1; ta.read_t = read_t; ta.bs_tf = 2.8f; ta.bs_hw = 1.275f;
    ta.truck_front_z = ta.truck_rear_z = ta.truck_hook_z = ta.trailer_hook_z = ta.trailer_rear_z = NAN;
    for (const auto &o : objs) ta.ahead[ta.n_ahead++] = o;
    return ta;
}
static traffic_obj lw_obj(float dx, float dy, float len, float wid, float hdg, unsigned id)
{
    traffic_obj o; memset(&o, 0, sizeof o);
    o.dx = dx; o.dy = dy; o.len = len; o.wid = wid; o.hdg = hdg; o.id = id; o.v = 10; o.rv_long = 0; o.rv_lat = 0; o.dist = hypotf(dx, dy);
    return o;
}
#endif

static int lw_rows()
{
    int bad = 0;
    auto row = [&](bool ok, const char *fmt, ...) {
        char b[600]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
        printf("   %s  %s\n", ok ? "ok  " : "FAIL", b);
        bad |= !ok;
    };
    printf("--- the AEB's sideways reading (core/aeblat.h): the radar's lead must be in our path to be braked for\n");
#ifdef DAZ_AEBLAT
    {   // the pure reading
        const double now = 100.0;
        al_in in = { now, now, 8.0f, 8.0f, 22.0f, 0.0f, 0, NAN };
        // 1. a box centred on our path (dx 17 m, 6 m long: its rear face 14 m ahead of the origin = 11.15 m from our front): clearance -2.3 m, weight 1
        {   in.gap = 17.0f - 3.0f - 2.85f; traffic_around ta = lw_list(now, { lw_obj(17, 0, 6, 2.0f, 0, 1) });
            al_out o = al_read(ta, in);
            row(o.matched && o.w == 1.0f && o.clr < -2.0f, "a vehicle dead ahead (dx 17, dy 0): found, clearance %.2f m, weight %.2f", o.clr, o.w); }
        // 2. 3.8 m beside (a parked car): clearance 3.8 - 1.275 - 1.0 = 1.5 m less 0.2 m for a bend not entered, weight 0
        {   in.gap = 17.0f - 3.0f - 2.85f; traffic_around ta = lw_list(now, { lw_obj(17, 3.8f, 6, 2.0f, 0, 1) });
            al_out o = al_read(ta, in);
            row(o.matched && o.w == 0.0f && o.clr > 1.2f && o.clr < 1.5f, "a parked car 3.8 m to the side: clearance %.2f m (1.5 less the allowance for a bend not entered), weight %.2f", o.clr, o.w); }
        // 3. the weight is a ramp in the clearance, no corner: sweep dy and take the largest slope (the ramp's own is 1 / AL_C1 per m)
        {   float worst = 0, prev = 0; bool first = true;
            for (float dy = 0; dy <= 5; dy += 0.002f) {
                in.gap = 17.0f - 3.0f - 2.85f; traffic_around ta = lw_list(now, { lw_obj(17, dy, 6, 2.0f, 0, 1) });
                al_out o = al_read(ta, in);
                if (!first) worst = fmaxf(worst, fabsf(o.w - prev) / 0.002f); prev = o.w; first = false;
            }
            row(worst <= 1.0f / AL_C1 + 0.05f, "the weight is a ramp in the sideways distance: the largest slope %.2f per m (the ramp's: %.2f)", worst, 1.0f / AL_C1); }
        // 4. a bend: we turn left at R = 120 m (22 m/s: 10.5 deg/s); a vehicle in our lane 30 m ahead sits 3.75 m to the left of our heading line (30^2 / 240): in the path
        {   in.yaw = 22.0f / 120 / 6.2831853f; in.gap = 30.0f - 3.0f - 2.85f; traffic_around ta = lw_list(now, { lw_obj(30, 3.75f, 6, 2.0f, 14.0f, 1) });
            al_out o = al_read(ta, in);
            row(o.matched && o.w == 1.0f, "in a bend (R 120 m) a vehicle in the lane 30 m ahead (3.75 m to the left of the heading line): path offset %.2f m, clearance %.2f m, weight %.2f", o.d, o.clr, o.w);
            in.yaw = 0;
            al_out s = al_read(ta, in);
            row(s.matched && s.w >= 0.5f, "the same vehicle for a truck that does not turn yet (yaw 0; R 120 m at 22 m/s is a sharper bend than a road has at that speed): clearance %.2f m with the allowance for a bend not entered, weight %.2f (in play)", s.clr, s.w); }
        // 5. the front: the bus's radar starts at its nose, the list's body is 1 m short; a 2 m window takes it
        {   for (float fe : { -1.0f, 0.0f, 1.0f }) {
                in.gap = 17.0f - 3.0f - 2.85f - fe; traffic_around ta = lw_list(now, { lw_obj(17, 0, 6, 2.0f, 0, 1) });
                al_out o = al_read(ta, in);
                row(o.matched, "the radar's gap %.1f m against the list's %.1f m (the front %+0.1f m off): found", in.gap, 17.0f - 3.0f - 2.85f, fe);
            } }
        // 6. two vehicles at about that distance (a truck ahead in the lane and a car beside it): the one with the least clearance counts
        {   in.gap = 17.0f - 3.0f - 2.85f; traffic_around ta = lw_list(now, { lw_obj(17.5f, 3.8f, 6, 2.0f, 0, 2), lw_obj(17, 0, 6, 2.0f, 0, 1) });
            al_out o = al_read(ta, in);
            row(o.matched && o.id == 1 && o.n_cand == 2 && o.w == 1.0f, "two vehicles at that distance (dy 3.8 and dy 0): id %u counts, weight %.2f", o.id, o.w); }
        // 6b. the same, but the one beside fits the radar's gap 1 m better than the one in the lane (a staggered queue, the bus nose 1 m off): any vehicle that
        // fits within the window may be the radar's lead, so the one in the path still counts (a missed stop is a crash, a needless one a bad moment)
        {   in.gap = 17.0f - 3.0f - 2.85f; traffic_around ta = lw_list(now, { lw_obj(17, 3.8f, 6, 2.0f, 0, 2), lw_obj(18, 0, 6, 2.0f, 0, 1) });
            al_out o = al_read(ta, in);
            row(o.matched && o.id == 1 && o.n_cand == 2 && o.w == 1.0f, "the one beside fits the gap 1 m better than the one in the lane: id %u counts, weight %.2f", o.id, o.w); }
        // 7. no reading: a stale list, none, a vehicle elsewhere: matched 0, weight 1 (today's behavior)
        {   in.gap = 17.0f - 3.0f - 2.85f;
            traffic_around ta = lw_list(now - 1.0, { lw_obj(17, 3.8f, 6, 2.0f, 0, 1) }); al_out a = al_read(ta, in);
            traffic_around tb = lw_list(now, { lw_obj(40, 3.8f, 6, 2.0f, 0, 1) }); al_out b = al_read(tb, in);
            traffic_around tc; memset(&tc, 0, sizeof tc); al_out c = al_read(tc, in);
            row(!a.matched && a.w == 1 && !b.matched && b.w == 1 && !c.matched && c.w == 1, "a stale list, a vehicle 23 m further, no list: not matched, weight 1 (as today)"); }
        // 8. the indicator away from the target is a credit of AL_PASS_M of clearance, toward it none
        {   in.gap = 17.0f - 3.0f - 2.85f; traffic_around ta = lw_list(now, { lw_obj(17, -2.7f, 6, 2.0f, 0, 1) });   // a car's width beside, on the right: clearance 0.2 m
            in.ind_side = 1; al_out away = al_read(ta, in);
            in.ind_side = -1; al_out toward = al_read(ta, in);
            in.ind_side = 0; al_out none = al_read(ta, in);
            row(away.w < none.w - 0.3f && toward.w == none.w, "target 2.7 m to the right (clearance %+.2f m): weight %.2f with the indicator left (away), %.2f with it right, %.2f without", none.clr, away.w, toward.w, none.w); }
        // 9. the vehicle's own lateral speed: of two vehicles 3.6 m to the left turned by 8 degrees (the same box), the one heading toward our path (a cut-in, 1.4 m/s at 10 m/s) weighs
        // more than the one heading away (a movement away is not believed: it may be the bend's, so it weighs what its box does); a vehicle that comes toward us, or whose speed is
        // not the radar's lead's, is not the lead
        {   al_in q = { now, now, 17.0f - 3.0f - 2.85f, 8.0f, 22.0f, 0.0f, 0, 10.0f };
            traffic_around ta = lw_list(now, { lw_obj(17, 3.6f, 6, 2.0f, -8.0f, 1) }); float w_in = al_read(ta, q).w;
            ta = lw_list(now, { lw_obj(17, 3.6f, 6, 2.0f, 8.0f, 1) }); float w_out = al_read(ta, q).w;
            row(w_in > w_out + 0.3f, "a vehicle 3.6 m to the left, turned by 8 degrees, weighs %.2f when it heads toward the path and %.2f when it heads away", w_in, w_out); }
        {
            al_in q = { now, now, 17.0f - 3.0f - 2.85f, 8.0f, 22.0f, 0.0f, 0, 10.0f };
            traffic_around ta = lw_list(now, { lw_obj(17, 0, 6, 2.0f, 175.0f, 2), lw_obj(17.5f, 6.0f, 6, 2.0f, 0, 3) });   // an oncoming one in our lane's place, and one at 10 m/s further out
            al_out o1 = al_read(ta, q);
            ta.ahead[1].v = 25.0f;   // at 25 m/s along our heading: not the lead (radar 10 m/s)
            al_out o2 = al_read(ta, q);
            row(o1.matched && o1.id == 3 && !o2.matched, "an oncoming vehicle is not the lead (id %u picked, not 2); one whose speed is 15 m/s off the radar's is no lead either (matched %d)", o1.id, o2.matched); }
        // 10. the cost of a reading: per call and the worst (16 vehicles in the list)
        {   in.gap = 17.0f - 3.0f - 2.85f; traffic_around ta = lw_list(now, {}); in.lead_v = NAN;
            for (int i = 0; i < 16; i++) ta.ahead[ta.n_ahead++] = lw_obj(17.0f + 0.3f * i, 3.0f * (i % 5) - 6.0f, 6, 2.0f, 0, 10 + i);
            struct timespec a0, a1; clock_gettime(CLOCK_MONOTONIC, &a0);
            volatile float sink = 0;
            const int N = 200000;
            for (int i = 0; i < N; i++) { in.gap = 14.0f + (i & 7) * 0.01f; sink += al_read(ta, in).w; }
            clock_gettime(CLOCK_MONOTONIC, &a1);
            double us = ((a1.tv_sec - a0.tv_sec) * 1e9 + (a1.tv_nsec - a0.tv_nsec)) / 1e3 / N;
            traffic_around cp; struct timespec b0, b1; clock_gettime(CLOCK_MONOTONIC, &b0);
            for (int i = 0; i < N; i++) { cp = ta; sink += cp.ahead[i & 15].dx; }
            clock_gettime(CLOCK_MONOTONIC, &b1);
            double us2 = ((b1.tv_sec - b0.tv_sec) * 1e9 + (b1.tv_nsec - b0.tv_nsec)) / 1e3 / N;
            row(us < 20.0, "cost: al_read with 16 vehicles %.3f us per call, copying the list (traffic_around_get) %.3f us; only called while closing on a lead that asks a stop", us, us2); }
    }
#endif

    // ---- scenarios
    // (a) an overtake of a slower truck + trailer (13.5 x 2.6 m, 55 km/h) from 85 km/h on a straight road, the indicator left, the pedal full, a lane change to the left
    // that starts late enough for the radar to offer the trailer beside us: the sweep of the start. Without the AEB the truck either clears it (>= 0.3 m) or does not.
    printf("   (a) overtaking a truck + trailer (55 km/h) from 85, indicator left, pedal full, the lane change starts at t0; ground truth from the run without the AEB\n");
    {
        int n_clear = 0, n_hit = 0, false_full = 0, real_miss = 0, partial = 0;
        for (float t0 = 0.4f; t0 <= 6.4f; t0 += 0.4f) {
            lw_scn sc; sc.kmh = 85;
            lw_veh o = { 0, 0, 55, 13.5f, 2.6f, 0x0e0d22 }; o.s0 = 55.0f + 2.85f + 6.75f; sc.veh.push_back(o);   // 55 m from our front to its rear
            sc.lc_t0 = t0; sc.ind_t0 = 0.3f; sc.ind_side = 1; sc.thr_t0 = 0.3f; sc.thr = 1.0f; sc.t_end = 12;
            lw_scn rf = sc; rf.aeb = 0; lw_res ref = lw_run(rf, "", 0);
            lw_res a = lw_run(sc, "", 0);
            char nm[96]; snprintf(nm, sizeof nm, "    lane change at t0 = %.1f s", t0);
            const bool clear = !ref.hit && ref.min_true_clr >= 0.3f;
            if (clear) { n_clear++; if (a.max_st >= AEB_FULL) false_full++; else if (a.max_req > 0) partial++; }
            else if (ref.hit) { n_hit++; lw_scn nl = sc; nl.list = 0; lw_res b = lw_run(nl, "", 0); if (a.max_st < b.max_st || a.min_true_clr < b.min_true_clr - 0.3f) real_miss++; }
            printf("%-62s %s: without the AEB closest %+5.1f m%s; with it %s, request %.2f, first at %.1f s, closest %+5.1f m%s\n", nm, clear ? "clear" : ref.hit ? "hit  " : "edge ", ref.min_true_clr < 1e8f ? ref.min_true_clr : 99.0f, ref.hit ? " HIT" : "", aeb_name[a.max_st], a.max_req, a.t_first, a.min_true_clr < 1e8f ? a.min_true_clr : 99.0f, a.hit ? " HIT" : "");
        }
        row(false_full == 0 && partial == 0, "(a) lane changes that clear the trailer without the AEB (%d starts): FULL braking in %d, any request in %d", n_clear, false_full, partial);
        row(real_miss == 0, "(a) lane changes that hit the trailer without the AEB (%d starts): as many stages and as large a closest clearance as without the list, except in %d", n_hit, real_miss);
    }
    // (b) a car standing 3.8 m beside the lane (a parked car, the 14-11 junction): we pass it at 29 km/h in a bend to the right, steering away from it, pedal light
    {
        lw_scn sc; sc.kmh = 29; sc.kappa = -1.0f / 70;
        lw_veh o = { 0, 0, 0, 6.0f, 2.0f, 0x0e088e }; o.s0 = 25; o.y0 = 3.8f; sc.veh.push_back(o);
        sc.thr_t0 = 0.0f; sc.thr = 0.1f; sc.t_end = 8;
        lw_scn rf = sc; rf.aeb = 0; lw_res ref = lw_run(rf, "    (b) without the AEB", 1);
        lw_res a = lw_run(sc, "(b) a car 3.8 m beside the lane, 29 km/h, steering away", 1);
        row(ref.min_true_clr >= 0.3f && a.max_st < AEB_FULL && a.max_req == 0, "(b) a car standing 3.8 m beside the lane while we steer away: highest %s, request %.2f (clear by %.1f m without the AEB)", aeb_name[a.max_st], a.max_req, ref.min_true_clr);
    }
    // (b2) the same on a straight road, 50 km/h, a standing car 4.3 m to the side (796.5: 4.3 m, a truck). At 80 km/h the allowance for a bend not entered (aeblat.h) keeps the radar\'s lead in play out to ~45 m, where the stop starts: as today
    {
        lw_scn sc; sc.kmh = 50;
        lw_veh o = { 0, 0, 0, 4.5f, 1.8f, 0x0e088f }; o.s0 = 100; o.y0 = 4.3f; sc.veh.push_back(o); sc.t_end = 9;
        lw_res a = lw_run(sc, "(b2) a car standing 4.3 m beside the road ahead, 50 km/h", 1);
        row(a.max_st < AEB_FULL && a.max_req == 0, "(b2) a car standing 4.3 m to the side of the road, 50 km/h: highest %s, request %.2f", aeb_name[a.max_st], a.max_req);
    }
    // (c) a real lead dead ahead, closing fast: a standing queue car, straight and in a bend; FULL braking and no hit, and what the radar alone (no list) would do
    for (int kind = 0; kind < 3; kind++) {
        lw_scn sc; sc.kmh = kind ? 60 : 80;
        if (kind >= 1) sc.kappa = 1.0f / 112;   // 30 m ahead the lane is 4 m to the left of our heading line (30^2 / 224); 60 km/h is 2.5 m/s2 sideways in it
        if (kind == 2) sc.kappa = -1.0f / 112;
        lw_veh o = { 0, 0, 0, 4.5f, 1.8f, 0x0e0001 }; o.s0 = 150; sc.veh.push_back(o); sc.t_end = 14;
        lw_scn nl = sc; nl.list = 0;
        const char *what = kind == 0 ? "straight road" : kind == 1 ? "left bend R 112 m" : "right bend R 112 m";
        char nm[96]; snprintf(nm, sizeof nm, "(c) a standing car in the lane, %s", what);
        lw_res a = lw_run(sc, nm, 1);
        snprintf(nm, sizeof nm, "    the same without the list (today)");
        lw_res b = lw_run(nl, nm, 1);
        row(a.max_st == AEB_FULL && !a.hit && b.max_st == AEB_FULL && fabsf(a.min_gap - b.min_gap) < 1.0f && a.end_kmh < 1.0f, "(c) %s: FULL, no hit, the smallest gap %.1f m against %.1f m without the list", what, a.min_gap, b.min_gap);
    }
    // (d) a vehicle that cuts in and ends in our lane: ahead in the next lane at 40 km/h, we do 80; it moves over from t = 1 s over 2 s (the cosine starts slowly). Braked as today, a
    // little later: the radar's lead is taken once its box is coming into the path, not while it is 3.6 m away in the next lane. 32 m ahead there is room for that; the same at 25 m
    // is the edge of what the brakes do (closing 11 m/s from 25 m needs the whole of them from the first tick): printed, not a row, with what it costs.
    for (int kind = 0; kind < 3; kind++) {
        lw_scn sc; sc.kmh = 80;
        const float gap0 = kind == 2 ? 25.0f : 32.0f;
        lw_veh o = { 0, 0, 40, 4.5f, 1.8f, 0x0e0003 }; o.s0 = gap0 + 2.85f + 2.25f; o.y0 = 3.7f; o.y1 = 0; o.y_t0 = kind == 1 ? 0.5f : 1.0f; o.y_dur = 2.0f; sc.veh.push_back(o); sc.t_end = 5;
        lw_scn nl = sc; nl.list = 0;
        char nm[96]; snprintf(nm, sizeof nm, "(d) a car cuts in %.0f m ahead (40 km/h) from t = %.1f s", gap0, o.y_t0);
        lw_res a = lw_run(sc, nm, 1);
        lw_res b = lw_run(nl, "    the same without the list (today)", 1);
        if (kind == 2) { printf("   (edge, not a row) the cut-in at 25 m: FULL at %.1f s against %.1f s, the smallest gap %.2f m against %.2f m%s\n", a.t_full, b.t_full, a.min_gap, b.min_gap, a.hit && !b.hit ? " (a hit where today's braking just clears)" : ""); continue; }
        row(!a.hit && a.max_st == b.max_st && a.t_full <= b.t_full + 0.3 && a.min_gap > b.min_gap - 1.0f, "(d) the cut-in at %.0f m: no hit, highest %s as without the list (%s), FULL at %.1f s against %.1f s, the smallest gap %.1f m against %.1f m", gap0, aeb_name[a.max_st], aeb_name[b.max_st], a.t_full, b.t_full, a.min_gap, b.min_gap);
    }
    // (e) a corner picked up late: the trailer we pass appears in the radar at 0.5 m (14-11 590.75: "0.4 m", 118 m/s2); even in our path no FULL for it
    {
        lw_scn sc; sc.kmh = 77;
        lw_veh o = { 0, 0, 48, 13.5f, 2.6f, 0x0e0d22 }; o.s0 = 25; o.y0 = 0; o.radar_late = 1; sc.veh.push_back(o); sc.t_end = 6; sc.thr_t0 = 0; sc.thr = 1.0f;
        lw_res a = lw_run(sc, "(e) a trailer in the path picked up by the radar at 1 m", 1);
        row(a.max_st < AEB_FULL, "(e) a vehicle that appears under 1 m ahead asking a hundred m/s2 is not a lead: highest %s", aeb_name[a.max_st]);
    }
    // (f) the log: the gas pedal held under 5 m logs one line
    {
        lw_scn sc; sc.kmh = 77;
        lw_veh o = { 0, 0, 30, 6.0f, 2.0f, 0x0e0004 }; o.s0 = 20; o.y0 = 0; sc.veh.push_back(o); sc.t_end = 6; sc.thr_t0 = 1.0f; sc.thr = 1.0f; sc.list = 0;
        lw_res a = lw_run(sc, "(f) a lead in the path, the pedal full from 1 s", 1);
        row(a.gas_lines <= 1, "(f) \"overruled by the gas pedal\" is logged %d time(s) for one overrule (once, not once a tick)", a.gas_lines);
    }
    printf("sideways reading rows: %s\n", bad ? "FAIL" : "ok");
    return bad;
}
