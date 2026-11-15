// Planner shadow: the longitudinal planner (core/planner.h, a pure function) runs beside the cruise and ONLY LOGS. While the cruise is on
// and the truck goes over 10 km/h it builds the planner's inputs from the live game, replans [planner] rate_hz times a second and writes
// planner.csv (with the log at its highest level) and a read-only view (planner_view_get, suite.h) for the dashboard stream. It posts no
// throttle, brake, retarder, neutral or gear wish and writes no cvar or game field: it is the measurement step before the planner drives
// (notes/planner/plan-draft.md, phase 5).
//
// The inputs (exact = what the game or the suite's own model says; approx = a stand-in until something better exists):
//   route    route_profile() at 10 m, up to 6.4 km (grade, bend and junction caps; NAN grade beyond the route's end is cut off). No
//            route: a flat profile at the grade of now (V.grade_pct, the game's own) over 2.1 km: the plan is then a cruise + ACC.
//            Limits: the road limit of now everywhere (the game has no limit preview; route_limit_ahead is always NAN).
//   vehicle  mass V.mass (exact, the game's figure; the fallback 20 t while unknown), rolling V.crr + V.f_bias folded in (approx: the
//            learned bias is a force, here a share of the weight), Cd*A V.cda (ini) and 1.0 under 4.5 t, eta V.eta; ratios T.ratio + T.diff
//            (exact; over 16 forward gears the lowest are dropped), wheel radius / V.rpm_k (the learned tire correction folded into the
//            radius: rpm exact, force off by rpm_k); torque curve V.eng x V.f_scale (exact, max 16 points, resampled), idle ts_idle_rpm(),
//            max rpm the engine's limit; engine drag fuel_kd() x peak torque at fuel_kd_rpm(), proportional to rpm (learned, approx);
//            retarder 0.8/1.6/2.4/3.0 m/s2 at 18 t as a force (notes/brake-measure.md, one truck: approx), sampled over the truck's
//            step count, linear under 8 m/s; service brake vehicle_brake_usable() (the model's estimate, never over 0.6 g: 6 m/s2 on the start value);
//            gear window idle + 350 .. 0.76 x rpm limit (the stand-in for TrueShift: approx, the CSV logs the disagreement).
//   fuel     the ETS2 fit (notes/planner/fuelfit.md): a = 7.35e-4 / 6.83e-4 / 7.0e-4 mL per rad per Nm by peak torque (Scania, Renault,
//            other: approx), the curve shares from the engine, idle from TrueShift; the zone scale from vehicle.cpp's learned 'a' when it
//            has learned it and it is plausible (0.08..2 of the fit), else 1 (outside cities). The decisions do not depend on the scale
//            (the planner scales its weights with it); the grams it reports do.
//   state    speed, gear (T.gear), neutral = TrueShift's eco-roll (g_rolling) and its seconds so far, the action class of now (throttle,
//            brake / retarder / our brake request, else coasting), mode = sense_mode() (M = PL_MANUAL: no neutral), set =
//            T.cruise (ISA writes the set speed itself), limit = T.speed_limit, the overspeed cap = [cruise] overspeed_cap_kmh over the
//            limit (5 when 0), the overspeed bucket kept here with pl_over_bucket() (approx: the executor will own it).
//   lead     the game's ACC lead (gap) and the radar's speed while the ACC distance is not off (cruise_plan_info_get); without a radar
//            speed: ours + d(gap)/dt (approx); its acceleration filtered here over 0.5 s, as the own ACC does; the time gap and the
//            standstill gap from the cruise's own wanted gap at the lead's speed (desired_gap): the planner's d0 + time gap x v_lead is
//            fitted to it at the lead's speed of now (exact there; at other lead speeds level 0 differs a little: approx).
//
// Timing: the game thread only builds the inputs (the route profile is the costly part, timed as prof_ms); pl_plan runs on a worker
// thread at below-normal priority (timed as plan_ms: ~0.5 ms mean on the host without a lead, 1-7 ms following one; the full 48 x 4 grid), so
// a slow plan delays the next replan instead of a frame. The sims build with PSH_SYNC: the plan runs in the frame.
//
// planner.csv, one row per replan (columns in psh_start). Dashboard JSON (core/dev.cpp is local; fields to add from planner_view):
//   "pl": { "on", "ok", "route", "mode", "act", "why", "nreq" (neutral_req), "ret", "gp" (gear_pred), "late", "vt" (v_target_kmh),
//           "vn" (v_next_kmh), "ct" (cruise_target_kmh), "dec" (decel), "sdec" (service_decel), "nh" (neutral_hold_s),
//           "vok" (v_overspeed_ok_kmh), "vcap" (v_throttle_cap_kmh), "pgk" (plan_g_km), "bgk" (base_g_km), "saved" (saved_g), "ms" (plan_ms) }
#include "../core/suite.h"
#include "../core/sense.h"
#include "../core/route.h"
#include "../core/vehicle.h"
#include "../core/speedtarget.h"
#include "../core/planner.h"
#include "../core/brake.h"
#include <math.h>
#include <string.h>
#include <chrono>

static float psh_p_shadow, psh_p_rate, psh_p_drive;

static param g_params[] = {
    { "shadow", &psh_p_shadow, 1, "1 = the longitudinal planner runs beside the cruise (cruise on, over 10 km/h) and only logs what it would do: planner.csv (needs the log at its highest level). Nothing is driven, braked or shifted from it. 0 = off" },
    { "drive", &psh_p_drive, 0, "1 = the planner drives the cruise: the cruise controller tracks the plan's target (v_target), the plan's deceleration goes to the brake channel (BRQ_PLANNER, the channel picks retarder vs service), neutral is requested from TrueShift every frame (M never), and the game's overspeed braking is kept quiet above the plan's v_overspeed_ok. Needs the cruise on and over ~10 km/h; [brake] channel = 1 for the brakes. A stale (> ~1 s) or failed plan falls back to the cruise's own blocks. 0 = off (the shadow above only logs)" },
    { "rate_hz", &psh_p_rate, 4, "replans per second (2-5)" },
};

// The neutral request API (trueshift.cpp, built in a parallel task): weak no-op defaults so this branch builds without that work.
// Called every frame while the planner wants neutral; ts_neutral_state() = 1 while TrueShift is in neutral on our request.
__attribute__((weak)) void ts_neutral_request(int want, float hold_s) { (void)want; (void)hold_s; }
__attribute__((weak)) int ts_neutral_state() { return 0; }

static const float PSH_MIN_MS = 10.0f / 3.6f;   // under 10 km/h the planner stays out (stops are the stop logic's)
static const float PSH_DS = 10.0f;               // profile spacing (m), the planner's recommendation
static const float PSH_STALE_S = 1.0f;           // a plan older than this is stale: the cruise falls back to its blocks

static PlWork *psh_w;
static PlVehicle psh_veh;
static PlProfile psh_prof;
static PlInput psh_in;
static PlOutput psh_out;
static route_prof psh_rp;
static FILE *psh_csv;
static planner_view psh_view;
static char psh_status[96] = "off";
static int psh_take_ok;                  // the plan under way may be taken (0 after a reset or the shadow going inactive)
static int psh_drive_was;                // drive mode posted wishes last tick (so a hand-back can clear them)
static double psh_next_at, psh_last_t;   // sim time of the next replan, of the last frame
static int psh_gear_off;                 // gears dropped below the planner's 16 (T.gear - off = the planner's gear)
static int psh_route;                    // the profile of the pending plan came from the route
static float psh_bucket, psh_neutral_s;  // the overspeed bucket and the seconds in neutral so far
static float psh_lead_prev_gap, psh_lead_prev_v = NAN, psh_lead_a, psh_gap_rate;
static float psh_zone = 1;
static double psh_plan_s;                // distance at the last plan (m), for the saved estimate
static double psh_dist;                  // distance driven while planning (m)
static float psh_saved_rate;             // g per m, plain cruise minus plan, of the last plan
// session stats (log line at stop; the sims read them through planner_view and psh_stats)
static struct { int plans, fails, cmp, diff2, lower2, higher2, gear_cmp, gear_ok, slow; double ms_sum, ms_max, prof_sum; double diff_sum; } psh_st;

static double psh_now_ms() { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
static inline float psh_q(float x, float step) { return roundf(x / step) * step; }
static inline float psh_clamp(float x, float a, float b) { return x < a ? a : x > b ? b : x; }

static int psh_pl_mode()
{
    switch (sense_mode()) {
    case MODE_ECO: return PL_ECO;
    case MODE_POWER: return PL_POWER;
    case MODE_M: return PL_MANUAL;
    default: return PL_NORMAL;
    }
}

static float psh_fuel_a(float tpk)   // mL per rad per Nm, high map zone (fuelfit.md: Scania / Renault measured, others the +-8 % guess)
{
    if (tpk >= 3600.0f) return 7.35e-4f;
    if (tpk >= 2600.0f && tpk < 2800.0f) return 6.83e-4f;
    return 7.0e-4f;
}

// The vehicle from the suite's model. 0 = not known well enough to plan (no engine data, electric, no gears or wheels). Every value that
// jitters is quantised so the planner's per-vehicle tables (compared byte for byte) are not rebuilt every replan.
static int psh_vehicle(PlVehicle *pv)
{
    float r = wheel_radius();
    int ng = T.fwd_gears;
    if (!V.eng_ok || V.electric || ng < 1 || ng > 32 || r < 0.1f || !(T.diff > 0) || !(V.eng.torque > 0) || V.eng.n_curve < 2) return 0;
    memset(pv, 0, sizeof *pv);
    float m = V.mass_ok && V.mass > 0 ? V.mass : VEHICLE_MASS_FALLBACK;
    pv->mass = psh_q(m, 100);
    pv->crr = psh_q(psh_clamp(V.crr + V.f_bias / (pv->mass * 9.81f), 0.001f, 0.05f), 1e-4f);
    pv->cda = pv->mass < 4500 ? 1.0f : V.cda;
    pv->rho = 1.2f;
    psh_gear_off = ng > PL_GEARS ? ng - PL_GEARS : 0;
    pv->ngear = ng - psh_gear_off;
    for (int i = 0; i < pv->ngear; i++) pv->ratio[i] = T.ratio[psh_gear_off + i];
    for (int i = 0; i < pv->ngear; i++) if (!(pv->ratio[i] > 0)) return 0;
    pv->final_drive = T.diff;
    float rk = V.rpm_k > 0.7f && V.rpm_k < 1.4f ? V.rpm_k : 1.0f;
    pv->wheel_r = psh_q(r / rk, 1e-3f);
    pv->eta = V.eta > 0.5f && V.eta <= 1 ? V.eta : 0.9f;
    float idle = psh_q(ts_idle_rpm(), 10), tpk = V.eng.torque, fs = V.f_scale > 0.5f && V.f_scale < 1.5f ? psh_q(V.f_scale, 0.01f) : 1.0f;
    pv->idle_rpm = idle; pv->max_rpm = V.eng.rpm_limit > idle + 500 ? V.eng.rpm_limit : idle + 1500;
    int nc = V.eng.n_curve < 24 ? V.eng.n_curve : 24, n = nc < PL_CURVE ? nc : PL_CURVE;
    pv->ncurve = n;
    for (int i = 0; i < n; i++) {
        int j = n == nc ? i : (int)roundf(i * (nc - 1) / (float)(n - 1));   // evenly over the engine's points, both ends kept
        pv->curve_rpm[i] = V.eng.curve[j][0]; pv->curve_nm[i] = tpk * fs * V.eng.curve[j][1];
        pv->ets2.curve_rpm[i] = V.eng.curve[j][0]; pv->ets2.curve_share[i] = V.eng.curve[j][1];
    }
    for (int i = 1; i < n; i++) if (pv->curve_rpm[i] <= pv->curve_rpm[i-1]) pv->curve_rpm[i] = pv->ets2.curve_rpm[i] = pv->curve_rpm[i-1] + 1;
    // engine drag in gear: the learned share of peak torque at its rpm, proportional to the rpm
    float kd = psh_q(psh_clamp(fuel_kd(), 0.01f, 0.2f), 0.002f), kdr = psh_q(psh_clamp(fuel_kd_rpm(), 600, 2500), 10);
    pv->drag_lo_rpm = 1000; pv->drag_lo_nm = kd * tpk * 1000 / kdr;
    pv->drag_hi_rpm = 1430; pv->drag_hi_nm = kd * tpk * 1430 / kdr;
    // retarder: the vehicle model's force per stage (the measured 18 t steps as the start value, learned per truck), the truck's stages up to PL_RET_LEVELS
    // (more stages are sampled evenly over the table)
    int steps = (int)T.retarder_steps, lv = steps < PL_RET_LEVELS ? steps : PL_RET_LEVELS;
    pv->ret_levels = lv > 0 ? lv : 0;
    for (int i = 0; i < pv->ret_levels; i++) pv->ret_force[i] = steps <= PL_RET_LEVELS ? vehicle_ret_force(i + 1) : vehicle_ret_force_frac((float)(i + 1) / pv->ret_levels);
    pv->ret_v_full = 8;
    pv->brake_decel_max = psh_q(psh_clamp(vehicle_brake_usable(), 2, 9), 0.1f);   // what a plan may count on from the service brake: the model's estimate, never over 0.6 g (it was the learned gain up to 9, else 6)
    pv->gear_rpm_lo = idle + 350; pv->gear_rpm_hi = fmaxf(pv->gear_rpm_lo + 300, 0.76f * pv->max_rpm); pv->gear_reserve = 0.92f;
    // the ETS2 fuel fit
    pv->ets2.a_ml_per_rad_nm = psh_fuel_a(tpk); pv->ets2.tpk = tpk; pv->ets2.idle_rpm = idle; pv->ets2.g_per_ml = 0.84f; pv->ets2.ncurve = n;
    psh_zone = 1;
    if (fuel_learned()) {   // the learned 'a' (L per rad per Nm on a x omega x (T_net + k Tpk)) against the fit's at half throttle
        float z = fuel_a() * 1000 * (0.5f + fuel_k()) / (0.593f * pv->ets2.a_ml_per_rad_nm);
        if (z >= 0.08f && z <= 2.0f) psh_zone = psh_q(z, 0.01f);
    }
    pv->ets2.zone_scale = psh_zone;
    pv->fuel = pl_fuel_ets2; pv->fuel_ctx = 0;
    pv->willans.a_g_per_j = 220.0f / 3.6e6f; pv->willans.fric_nm = 0.09f * tpk; pv->willans.idle_rpm = idle;
    return 1;
}

// The road ahead. Returns 1 when it came from the route.
static int psh_profile(PlProfile *p)
{
    p->ds = PSH_DS; p->stop_m = -1;
    int n = 0;
    // a car has no navigation-fed (predictive) cruise: the light-vehicle class plans on the road it sees, never on the route (Damien 31-10)
    if (route_ok() && route_profile(PSH_DS, PL_PROF_MAX < ROUTE_PROF_MAX ? PL_PROF_MAX : ROUTE_PROF_MAX, &psh_rp) && psh_rp.n > 2) {
        float last = V.grade_pct / 100;
        for (int i = 0; i < psh_rp.n; i++) {
            float g = psh_rp.grade[i];
            if (!(g == g)) { if (i * PSH_DS > 2100) break; g = last; }   // a hole: the last grade; beyond the horizon: the profile ends
            last = g;
            p->grade[n] = psh_clamp(g, -0.25f, 0.25f);
            float c = psh_rp.vmax[i];
            p->vmax[n] = c == c && c > 0 ? c : 0;
            p->limit[n] = 0;   // no preview: the input's limit
            n++;
        }
    }
    if (n > 2) { p->n = n; return 1; }
    n = (int)(2100 / PSH_DS);
    float g = psh_clamp(V.grade_pct / 100, -0.25f, 0.25f);
    for (int i = 0; i < n; i++) { p->grade[i] = g; p->vmax[i] = 0; p->limit[i] = 0; }
    p->n = n;
    return 0;
}

static int psh_active()
{
    return (psh_p_shadow > 0 || psh_p_drive > 0) && psh_w && !T.paused && T.cruise > 0.5f && fabsf(T.speed) > PSH_MIN_MS && T.gear >= 0;
}

static int psh_cls_now()
{
    if (g_rolling) return PL_COAST;
    if (T.eff_brake > 0.02f || T.retarder > 0 || act_request() > 0.02f) return PL_BRAKE;
    if (T.eff_thr > 0.02f) return PL_GAS;
    return PL_COAST;
}

// The state part of the input (fresh in the plan's frame).
static void psh_state(PlInput *in, const cruise_plan_info &ci)
{
    in->drive_mode = psh_pl_mode();
    pl_mode_defaults(in->drive_mode, &in->mode);
    pl_safety_defaults(&in->safety);
    in->safety.limit_over_kmh = g_overspeed_cap > 0 ? overspeed_cap_now() : 5;
    in->v0 = fabsf(T.speed);
    int g = T.gear - psh_gear_off;
    in->gear0 = g_rolling || T.gear <= 0 ? 0 : g < 1 ? 1 : g > psh_veh.ngear ? psh_veh.ngear : g;
    in->neutral0 = g_rolling ? 1 : 0;
    in->neutral_s0 = psh_neutral_s;
    in->cls0 = psh_cls_now();
    in->over_used_s = psh_bucket;
    in->set = T.cruise;
    in->limit = road_limit_ms() > 1 ? road_limit_ms() : 0;
    in->lead.present = 0;
    if (ci.level >= 0 && ci.lead_m > 0.5f) {
        in->lead.present = 1;
        in->lead.gap = ci.lead_m;
        in->lead.v = ci.lead_ms == ci.lead_ms ? ci.lead_ms : fmaxf(0.0f, in->v0 + psh_gap_rate);
        in->lead.a = psh_lead_a;
        // the cruise's wanted gap at the lead's speed (level 0 takes the larger of standstill and time gap, the others add them): the
        // planner's d0 + time_gap x v_lead is made to give the same gap at the lead's speed of now, d0 at most the standstill gap
        if (ci.want_m == ci.want_m && ci.want_m > 1) {
            in->safety.d0 = fminf(ci.d0_m, 0.5f * ci.want_m);
            in->safety.time_gap = psh_clamp((ci.want_m - in->safety.d0) / fmaxf(in->lead.v, 3.0f), 0.3f, 6.0f);
        }
    }
}

static void psh_row(const cruise_plan_info &ci, float prof_ms, float plan_ms)
{
    if (!psh_csv) return;
    const PlOutput &o = psh_out;
    const PlInput &in = psh_in;
    float minc = 1e9f, minc_m = -1;
    for (int i = 0; i < psh_prof.n; i++) if (psh_prof.vmax[i] > 0 && psh_prof.vmax[i] < minc) { minc = psh_prof.vmax[i]; minc_m = i * PSH_DS; }
    int a1 = o.n > 1 ? o.st[1].act : -1, a2 = o.n > 2 ? o.st[2].act : -1;
    fprintf(psh_csv, "%.2f,%.2f,%.1f,%.1f,%d,%d,%d,%d,%d,%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f,%d,%d,%d,%.2f,%.2f,%.2f,%d,%d,%d,%d,%.1f,%.2f,%d,%.2f,%.1f,%.2f,%.2f,%.1f,%.0f,"
                     "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%d,%d,%d,%d,%.3f,%.3f,%.1f,%.2f,%.2f,%.2f\n",
            sim_s(), in.v0 * 3.6f, in.set * 3.6f, in.limit * 3.6f, T.gear, o.gear_pred > 0 ? o.gear_pred + psh_gear_off : 0, g_rolling, in.drive_mode, psh_route,
            psh_prof.n > 0 ? psh_prof.grade[0] * 100 : 0.0f, psh_veh.mass / 1000,
            in.lead.present ? in.lead.gap : 0.0f, in.lead.present ? in.lead.v * 3.6f : 0.0f, in.lead.present ? in.lead.a : 0.0f,
            in.lead.present ? in.safety.time_gap : 0.0f, ci.target_kmh, o.ok, o.why, o.late,
            o.v_target * 3.6f, o.v_next * 3.6f, o.v_next2 * 3.6f, o.act, a1, a2, o.neutral_req, o.neutral_hold_s, o.decel, o.ret_level, o.service_decel,
            o.force / 1000, o.v_overspeed_ok * 3.6f, o.v_throttle_cap * 3.6f, minc < 1e8f ? minc * 3.6f : 0.0f, minc_m,
            o.cost, o.c_fuel, o.c_time, o.c_pen, o.c_term, o.base_cost, o.fuel_g, o.base_fuel_g, o.time_s, o.base_time_s,
            o.base_fuel_g > 0 ? 100 * (o.base_fuel_g - o.fuel_g) / o.base_fuel_g : 0.0f, o.min_gap_margin == o.min_gap_margin ? o.min_gap_margin : 0.0f,
            o.labels_used, o.transitions, o.n, o.stages_fine, prof_ms, plan_ms,
            psh_bucket, psh_neutral_s, psh_zone, psh_view.saved_g);
}

// ---- the worker: pl_plan off the game thread (a plan following a lead takes ~2-7 ms, a third of a frame). The frame thread fills
// sets psh_done; the frame thread takes the result (psh_out), then clears psh_busy and psh_done. A replan is only started when the
// sets psh_done; the frame thread takes the result (psh_out), then clears psh_done and psh_busy. A replan is only started when the
// worker is idle, so a slow plan lowers the rate instead of piling up. PSH_SYNC (the sims): the plan runs in the frame instead.
static PlOutput psh_wout;
static volatile float psh_wms;          // the worker's time for the plan in psh_wout (ms)
#ifdef PSH_SYNC
typedef long psh_flag;
static inline long psh_get(volatile psh_flag *f) { return *f; }
static inline void psh_set(volatile psh_flag *f, long v) { *f = v; }
#else
typedef LONG psh_flag;
static inline long psh_get(volatile psh_flag *f) { return InterlockedCompareExchange(f, 0, 0); }   // (the value, with a full barrier)
static inline void psh_set(volatile psh_flag *f, long v) { InterlockedExchange(f, v); }   // (a full barrier: what was written before is seen first)
#endif
static volatile psh_flag psh_busy, psh_done;
static cruise_plan_info psh_ci;         // the cruise's view when the inputs were taken (the comparison belongs to the same moment)
#ifndef PSH_SYNC
static HANDLE psh_thread, psh_wake;
static volatile psh_flag psh_quit;
static DWORD WINAPI psh_worker(LPVOID)
{
    while (!psh_quit) {
        WaitForSingleObject(psh_wake, 500);
        if (psh_quit) break;
        if (psh_get(&psh_busy) != 1 || psh_get(&psh_done)) continue;
        double a = psh_now_ms();
        pl_plan(&psh_in, &psh_wout, psh_w);
        psh_wms = (float)(psh_now_ms() - a);
        psh_set(&psh_done, 1);   // (the result is written before the flag)
    }
    return 0;
}
#endif

// A result: stats, the view, the csv row.
static void psh_take(double now)
{
    psh_out = psh_wout;
    const PlOutput &o = psh_out;
    const PlInput &in = psh_in;
    const cruise_plan_info &ci = psh_ci;
    float ms = psh_wms;
    psh_st.plans++; if (!o.ok) psh_st.fails++;
    psh_st.ms_sum += ms; if (ms > psh_st.ms_max) psh_st.ms_max = ms; psh_st.prof_sum += psh_view.prof_ms;
    if (ms > 2) psh_st.slow++;
    static double said;
    if (ms > 25 && now - said > 60) { said = now; logv(1, "Planner: a replan took %.1f ms (%d stages, %d transitions)", ms, o.n, o.transitions); }
    float tgt = o.v_target * 3.6f;
    if (o.ok && ci.target_kmh == ci.target_kmh) {
        float d = tgt - ci.target_kmh;
        psh_st.cmp++; psh_st.diff_sum += d;
        if (fabsf(d) > 2) { psh_st.diff2++; if (d < 0) psh_st.lower2++; else psh_st.higher2++; }
    }
    if (o.ok && in.gear0 > 0 && !in.neutral0) { psh_st.gear_cmp++; if (o.gear_pred == in.gear0) psh_st.gear_ok++; }
    // saved estimate: plain cruise minus plan per metre of the horizon, over the distance driven until the next plan
    float s_end = o.n > 1 ? o.st[o.n - 1].s : 0;
    psh_view.saved_g += psh_saved_rate * (float)(psh_dist - psh_plan_s);
    psh_plan_s = psh_dist;
    psh_saved_rate = o.ok && s_end > 100 ? (o.base_fuel_g - o.fuel_g) / s_end : 0;
    planner_view &v = psh_view;
    v.on = 1; v.t = now; v.n_plans = psh_st.plans; v.ok = o.ok; v.route = psh_route; v.mode = in.drive_mode; v.act = o.act; v.why = o.why;
    v.neutral_req = o.neutral_req; v.ret_level = o.ret_level; v.gear_pred = o.gear_pred > 0 ? o.gear_pred + psh_gear_off : 0; v.late = o.late;
    v.v_target_kmh = tgt; v.v_next_kmh = o.v_next * 3.6f; v.cruise_target_kmh = ci.target_kmh; v.decel = o.decel; v.service_decel = o.service_decel;
    v.neutral_hold_s = o.neutral_hold_s; v.v_overspeed_ok_kmh = o.v_overspeed_ok * 3.6f; v.v_throttle_cap_kmh = o.v_throttle_cap * 3.6f;
    v.plan_g_km = s_end > 100 ? o.fuel_g / s_end * 1000 : 0; v.base_g_km = s_end > 100 ? o.base_fuel_g / s_end * 1000 : 0;
    v.plan_ms = ms;
    // the next two changes of action and a profile of 8 planned speeds along the plan (the dashboard's Planner tab)
    v.next_act[0] = v.next_act[1] = -1; v.next_at_m[0] = v.next_at_m[1] = 0; v.n_prof = 0; v.prof_m = 0;
    if (o.ok && o.n > 1) {
        int last = o.act, nn = 0;
        for (int i = 1; i < o.n && nn < 2; i++) {
            int a = o.st[i].act;
            if (a == PL_ANY || a == last) continue;
            v.next_act[nn] = a; v.next_at_m[nn] = o.st[i - 1].s; nn++; last = a;
        }
        if (s_end > 100) {
            const int np = (int)(sizeof v.prof_kmh / sizeof v.prof_kmh[0]);
            v.n_prof = np; v.prof_m = s_end;
            int j = 1;
            for (int k = 0; k < np; k++) {
                float sk = s_end * (k + 1) / np;
                while (j < o.n - 1 && o.st[j].s < sk) j++;
                float s0 = o.st[j - 1].s, s1 = o.st[j].s, f = s1 > s0 ? (sk - s0) / (s1 - s0) : 1;
                f = f < 0 ? 0 : f > 1 ? 1 : f;
                v.prof_kmh[k] = (o.st[j - 1].v + f * (o.st[j].v - o.st[j - 1].v)) * 3.6f;
            }
        }
    }
    psh_row(ci, psh_view.prof_ms, ms);
    static double flushed;
    if (psh_csv && now - flushed > 2) { flushed = now; fflush(psh_csv); csv_cap(&psh_csv, "planner.csv"); }
}

// Drive mode: publish the plan's view to the cruise (g_planner_drive) and post its wishes to the executors every frame (held until
// changed); both are cleared when the planner is not in charge. The plan's decel -> the brake channel (BRQ_PLANNER, the channel picks
// retarder vs service); neutral -> TrueShift. A stale or failed plan asks nothing (the cruise falls back to its blocks).
static void psh_drive_frame(double now)
{
    // (review 30-10: no driving without the brake channel: the plan's braking would be a silent no-op while the game's own overspeed
    // braking is moved up for it)
    if (psh_p_drive > 0 && g_brake_channel < 0.5f) {
        static int said;
        if (!said) { said = 1; logv(1, "Planner: drive = 1 needs [brake] channel = 1: not driving (shadow only)"); }
    }
    // a light vehicle: the planner only shadows. It is tuned for trucks; in a car it planned late and then braked at 8 m/s2 every
    // ~10 s in town (Damien 31-10: 'why does the cruise hit the emergency brake every time'). The cruise's own blocks drive a car.
    if (psh_p_drive <= 0 || !psh_w || g_brake_channel < 0.5f) {
        if (psh_drive_was) { brake_ask(BRQ_PLANNER, 0, BRU_COMFORT); ts_neutral_request(0, 0); psh_drive_was = 0; }
        memset(&g_planner_drive, 0, sizeof g_planner_drive);
        return;
    }
    if (!psh_active()) {
        if (psh_drive_was) { brake_ask(BRQ_PLANNER, 0, BRU_COMFORT); ts_neutral_request(0, 0); psh_drive_was = 0; }
        memset(&g_planner_drive, 0, sizeof g_planner_drive);
        return;
    }
    psh_drive_was = 1;
    const PlOutput &o = psh_out;
    int fresh = psh_take_ok && o.ok && psh_view.on && now - psh_view.t <= PSH_STALE_S;
    planner_drive &pd = g_planner_drive;
    memset(&pd, 0, sizeof pd);
    pd.on = 1;
    pd.ok = o.ok;
    pd.fresh = fresh;
    pd.v_target_ms = o.v_target;
    pd.v_next_ms = o.v_next;
    pd.v_overspeed_ok_ms = o.v_overspeed_ok;
    pd.v_throttle_cap_ms = o.v_throttle_cap;
    pd.force = o.force;
    pd.decel = o.decel;
    pd.act = o.act;
    pd.why = o.why;
    if (fresh) {
        if (o.act == PL_BRAKE && o.decel > 0) brake_ask(BRQ_PLANNER, o.decel, o.late ? BRU_URGENT : BRU_FIRM);
        else brake_ask(BRQ_PLANNER, 0, BRU_COMFORT);
        ts_neutral_request(o.neutral_req, o.neutral_hold_s);
    } else {
        brake_ask(BRQ_PLANNER, 0, BRU_COMFORT);
        ts_neutral_request(0, 0);
    }
}

// Per frame: the executor-side state (bucket, neutral time, the lead's acceleration), a finished plan, a new replan when due.
static void psh_frame()
{
    if ((psh_p_shadow <= 0 && psh_p_drive <= 0) || !psh_w) return;
    double now = sim_s();
    float dt = (float)(now - psh_last_t);
    psh_last_t = now;
    if (!(dt > 0 && dt < 0.5f)) dt = 0;
    cruise_plan_info ci;
    cruise_plan_info_get(&ci);
    // lead: acceleration from the radar speed (or our speed + the gap rate), 0.5 s filter, reset on a new lead
    if (ci.lead_m > 0.5f && dt > 0) {
        float jump = psh_lead_prev_gap > 0.5f ? fabsf(ci.lead_m - psh_lead_prev_gap) : 1e9f;
        if (jump < 15) psh_gap_rate += ((ci.lead_m - psh_lead_prev_gap) / dt - psh_gap_rate) * fminf(1, dt / 0.5f);
        else { psh_gap_rate = 0; psh_lead_prev_v = NAN; psh_lead_a = 0; }
        float lv = ci.lead_ms == ci.lead_ms ? ci.lead_ms : fabsf(T.speed) + psh_gap_rate;
        if (psh_lead_prev_v == psh_lead_prev_v) psh_lead_a += ((lv - psh_lead_prev_v) / dt - psh_lead_a) * fminf(1, dt / 0.5f);
        psh_lead_a = psh_clamp(psh_lead_a, -8, 4);
        psh_lead_prev_v = lv;
        psh_lead_prev_gap = ci.lead_m;
    } else if (!(ci.lead_m > 0.5f)) {
        // the radar lost the lead: it is gone (31-10: holding it as a stopped obstacle at its last spot, shrinking as we rolled on,
        // braked a car at 8 m/s2 for nothing every time a car ahead turned off or cut out in town; the fleet never needed it)
        psh_lead_prev_gap = 0; psh_lead_prev_v = NAN; psh_lead_a = 0; psh_gap_rate = 0;
    }
    int active = psh_active();
    if (dt > 0) {
        if (active) psh_dist += fabsf(T.speed) * dt;   // metres while planning (the saved estimate)
        psh_neutral_s = g_rolling ? psh_neutral_s + dt : 0;
        // a descent the coast accelerates on (-2 %): gravity overspeed is free, so the bucket does not count it (a pure gravity
        // descent may run up to set + over for as long as the descent lasts); above set + over / limit + 5 the hard caps still hold.
        psh_bucket = T.cruise > 0.5f ? (V.grade_pct <= -2.0f ? psh_bucket : pl_over_bucket(psh_bucket, fabsf(T.speed), T.cruise, dt, 0.5f)) : 0;
    }
    // a finished plan (dropped when the shadow went inactive or was reset meanwhile)
    if (psh_get(&psh_done)) {
        if (active && psh_take_ok) psh_take(now);
        psh_set(&psh_busy, 0);   // (busy first: the worker only plans while busy and not done, so it cannot rerun the old inputs)
        psh_set(&psh_done, 0);
    }
    psh_drive_frame(now);   // (drive mode posts the brake / neutral wishes from the plan; clears them when it hands back)
    if (!active) { psh_view.on = 0; psh_take_ok = 0; return; }
    if (now - psh_view.t > 1.0) psh_view.on = 0;
    if (psh_busy || now < psh_next_at) return;
    // a replan: the inputs, then the worker
    psh_next_at = now + 1.0 / psh_clamp(psh_p_rate, 1, 10);
    if (!psh_vehicle(&psh_veh)) { psh_next_at = now + 1; return; }
    double a = psh_now_ms();
    psh_route = psh_profile(&psh_prof);
    PlInput &in = psh_in;
    pl_input_defaults(&in);
    in.veh = &psh_veh; in.prof = &psh_prof;
    psh_state(&in, ci);
    psh_ci = ci;
    psh_view.prof_ms = (float)(psh_now_ms() - a);
    psh_take_ok = 1;
#ifdef PSH_SYNC
    a = psh_now_ms();
    pl_plan(&psh_in, &psh_wout, psh_w);
    psh_wms = (float)(psh_now_ms() - a);
    psh_take(now);
#else
    psh_set(&psh_busy, 1);
    SetEvent(psh_wake);
#endif
}

static void psh_start()
{
    if (psh_p_shadow <= 0 && psh_p_drive <= 0) { snprintf(psh_status, sizeof psh_status, "off (ini)"); return; }
    if (!psh_w) psh_w = pl_work_create();
    if (!psh_w) { snprintf(psh_status, sizeof psh_status, "off: no memory for the planner"); return; }
#ifndef PSH_SYNC
    if (!psh_thread) {
        psh_wake = CreateEventA(NULL, FALSE, FALSE, NULL);
        psh_thread = psh_wake ? CreateThread(NULL, 0, psh_worker, NULL, 0, NULL) : NULL;
        if (!psh_thread) { psh_p_shadow = 0; psh_p_drive = 0; snprintf(psh_status, sizeof psh_status, "off: no planner thread"); return; }
        SetThreadPriority(psh_thread, THREAD_PRIORITY_BELOW_NORMAL);
    }
#endif
    psh_csv = open_out("planner.csv", "w");
    if (psh_csv) fputs("t,kmh,set,limit,gear,gear_pred,rolling,mode,route,grade_pct,mass_t,lead_m,lead_kmh,lead_a,time_gap_s,cruise_tgt,ok,why,late,"
                       "v_target,v_next,v_next2,act,act1,act2,neutral_req,neutral_hold_s,decel,ret,service_decel,force_kn,v_overspeed_ok,v_throttle_cap,"
                       "min_cap_kmh,min_cap_m,cost,c_fuel,c_time,c_pen,c_term,base_cost,fuel_g,base_fuel_g,time_s,base_time_s,saved_pct,min_gap_margin,"
                       "labels_used,transitions,stages,stages_fine,prof_ms,plan_ms,over_used_s,neutral_s,zone,saved_g\n", psh_csv);
    if (psh_p_drive > 0)
        snprintf(psh_status, sizeof psh_status, "drive (the planner drives the cruise%s), %.0f Hz, %s", brake_channel_on() ? "" : ", brakes need [brake] channel 1", psh_clamp(psh_p_rate, 1, 10), psh_csv ? "planner.csv" : "no csv");
    else
        snprintf(psh_status, sizeof psh_status, "shadow (logs only, drives nothing), %.0f Hz, %s", psh_clamp(psh_p_rate, 1, 10), psh_csv ? "planner.csv" : "no csv");
}

static void psh_reset(int, const char *)
{
    psh_view.on = 0; psh_take_ok = 0;   // a plan under way belongs to the state before the reset: dropped when it arrives
    psh_bucket = 0; psh_neutral_s = 0;
    psh_lead_prev_gap = 0; psh_lead_prev_v = NAN; psh_lead_a = 0; psh_gap_rate = 0;
    psh_saved_rate = 0; psh_plan_s = psh_dist;
    if (psh_drive_was) { brake_ask(BRQ_PLANNER, 0, BRU_COMFORT); ts_neutral_request(0, 0); psh_drive_was = 0; }
}

static const char *psh_status_fn() { return psh_status; }

static void psh_stop()
{
#ifndef PSH_SYNC
    if (psh_thread) {   // (the DLL stays loaded: the worker is only asked to end, briefly waited for)
        psh_set(&psh_quit, 1); SetEvent(psh_wake);
        if (WaitForSingleObject(psh_thread, 200) == WAIT_OBJECT_0) { CloseHandle(psh_thread); CloseHandle(psh_wake); psh_thread = NULL; psh_wake = NULL; }
    }
#endif
    if (psh_csv) fflush(psh_csv);
    if (psh_p_shadow > 0 && psh_st.plans)
        logline("Planner: %d shadow plans (%d failed), %.2f ms mean, %.2f max, %d over 2 ms; target off the cruise's by > 2 km/h in %.0f %% (lower %d, higher %d), "
                "gear stand-in agreed %.0f %%, saved estimate %.0f g",
                psh_st.plans, psh_st.fails, psh_st.ms_sum / psh_st.plans, psh_st.ms_max, psh_st.slow,
                psh_st.cmp ? 100.0 * psh_st.diff2 / psh_st.cmp : 0.0, psh_st.lower2, psh_st.higher2,
                psh_st.gear_cmp ? 100.0 * psh_st.gear_ok / psh_st.gear_cmp : 0.0, psh_view.saved_g);
}

int planner_view_get(planner_view *o)
{
    *o = psh_view;
    o->shadow = psh_p_shadow > 0; o->drive = psh_p_drive > 0;
    if (!(psh_view.on && sim_s() - psh_view.t <= 1.0)) { o->on = 0; return 0; }
    return 1;
}

module m_planner = { "planner", "Planner", g_params, sizeof g_params / sizeof g_params[0], psh_start, psh_frame, NULL, psh_stop, psh_reset, psh_status_fn, NULL, VCM_TRUCK };
