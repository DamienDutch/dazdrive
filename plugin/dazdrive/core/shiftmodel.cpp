// The learning every gear brain does into the vehicle model (see shiftmodel.h). Moved verbatim out of modules/trueshift.cpp when the car
// automatic got its own module (modules/carshift.cpp), so the two share one copy.
#include "shiftmodel.h"
#include "game.h"
#include <string.h>

int sm_thr_capped()
{
    const act_wish *w = act_wishes();
    for (int o = 0; o < ACT_N; o++)
        if (w[o].cap < 1.0f || w[o].scale < 1.0f) return 1;
    return 0;
}

void sm_tc_known(const sm_tc_cfg &cfg)
{
    if (cfg.tc >= 2) return;
    // The game's own figure comes first (WP-TC2, owner decision 61): the transmission object holds the stall torque ratio and the engine read has checked the layout (engine_read_stall).
    // Above 1 the box IS a converter, from the first metre: no slip to wait for, no file of the last drive. The ini's figure still wins (cfg.stall); the game's is exact, so it is
    // fixed like the ini's (vehicle_tc_stall_step learns nothing over it). A converter that the file or the detection declared before the read landed gets the figure too.
    // Exactly 1 (read and trusted: a clutch box) means no converter from the file either; only the slip detection below can still say otherwise (it is the fallback).
    const float rd = V.eng_ok ? V.eng.tc_stall : 0.0f;
    if (rd > 1.0f && !(V.tc && V.tc_stall_fixed)) {
        if (!V.tc) {
            float st = 0; int lk = 0;
            vehicle_tc_stored(&st, &lk);   // (only the lock gear of the last drive is wanted; lk stays 0 without a file)
            V.tc = 1;
            V.tc_lock_gear = cfg.lock > 0 ? (int)cfg.lock : lk;
            V.rpm_k = 1.0f;
            logline("torque converter read from the game (stall %.2f%s, lock gear %d%s): creep and launch are the converter's, the clutch is never driven",
                    cfg.stall > 1.0f ? cfg.stall : rd, cfg.stall > 1.0f ? " as set" : "", V.tc_lock_gear, V.tc_lock_gear > 0 ? "" : ", learned");
        } else if (cfg.stall <= 1.0f) logline("torque converter: the stall ratio is the game's %.2f (it was %.2f, from the file or the launches)", rd, V.tc_stall);
        V.tc_stall = cfg.stall > 1.0f ? cfg.stall : rd;
        V.tc_stall_fixed = 1;
        return;
    }
    // A converter the last drive of this vehicle found is known from the start (the detection below needs a second of slip, and the first upshifts of
    // a launch were a manual box's: ride 31-10, the Crown Victoria 1->2 at 26 km/h before the detection, back at 38 km/h)
    if (!V.tc && rd != 1.0f) {
        float st = 0; int lk = 0;
        if (vehicle_tc_stored(&st, &lk)) {
            V.tc = 1;
            V.tc_stall = cfg.stall > 1.0f ? cfg.stall : st;   // (the ini's when set, else what the last drive learned; it goes on learning from there, core/vehicle.cpp)
            V.tc_stall_fixed = cfg.stall > 1.0f;
            V.tc_lock_gear = cfg.lock > 0 ? (int)cfg.lock : lk;
            V.rpm_k = 1.0f;
            logline("torque converter known from the last drive of this truck (stall %.2f, lock gear %d): creep and launch are the converter's, the clutch is never driven", V.tc_stall, V.tc_lock_gear);
        }
    }
}

static float s_coupled_s[32];   // per gear: seconds seen coupled while pulling (the lock-gear learner below)
int sm_learn_rpm(const sm_tc_cfg &cfg, int g, float v, double step_s, float *tc_slip_s, float *rpm_odd_s)
{
    int odd = 0;
    const float step = (float)step_s;
    // The converter is found from its slip, which belongs to a launch: the first gears, the clutch closed (a converter has none to slip), the driveline turning and the engine well
    // over it. This had the gate of the tire learning below (v > 5 AND T.rpm > 700), and v is in m/s: 18 km/h and the engine over 700, where the slip of a converter like the
    // Spectrans Scania's is over (it slips under ~10 km/h, ride 04-11: 572 rpm over a driveline at 343 at 3.6 km/h, 1.67 times, down to 1.13 at 8.5 km/h) and the engine
    // sits at 570 .. 700 for the first seconds. So: gear 1 or 2 with the clutch closed, the driveline over 300 rpm (the speed in the gear: the same floor that kept a standstill's
    // kr of 15 out), 1.18 times over it (a locked box reads 0.96 .. 1.09 here, the same figure as before), for a second in all. The seconds are kept across launches, as before
    // (a light launch slips for a third of a second), and leak away slowly (1 s per 50 s coupled): a closed-clutch transient of a clutch box is forgotten, a converter's launches come
    // faster than that. And the engine over its idle by a tenth (the idle governor's own wobble is a few percent): a driveline slower than the idle holds the engine AT the idle,
    // which is no slip but the clamp of the engine (a clutch box whose clutch reads closed at a crawl, and the sims' plants, show exactly that and read 1.5 times over; a converter
    // that slips is loaded, so its engine rises over the idle within a second: 572 .. 607 rpm in the ride's first launch seconds, found one second later than without this).
    if (g > 0 && cfg.tc < 2 && !V.tc && clutch_seen() < 0.02f && T.rpm > 1.1f * cfg.idle) {
        const float raw = vehicle_rpm_at(g, v) / V.rpm_k;   // the driveline rpm
        if (raw > 300) {
            const float kr = T.rpm / raw;
            if (kr > 1.18f && g <= 2) *tc_slip_s += step;
            else *tc_slip_s = fmaxf(0, *tc_slip_s - 0.02f * step);
            if (*tc_slip_s > 1.0f) {
                V.tc = 1;
                V.tc_stall = cfg.stall > 1.0f ? cfg.stall : 2.0f;   // (the prior: learned from the launches from here, core/vehicle.cpp; the ini's figure when set)
                V.tc_stall_fixed = cfg.stall > 1.0f;
                V.tc_lock_gear = cfg.lock > 0 ? (int)cfg.lock : 0;
                // The second of slip before the detection was learned as a tire correction (ride 31-10, Crown Victoria: 1.000 -> 1.177
                // over 0.9 s, and a converter learns nothing more, so the model's rpm stayed 18 % high all session and every shift
                // point fired early): give it back. Even the clean value was a coast's (0.89 in the 31-10 ride, the pedal up before the
                // detection): a converter's rpm is above the driveline's pulling and under it coasting, so no tire size is read from it.
                V.rpm_k = 1.0f;   // (and on a converter it is never learned again: the gears' own effective ratios take what the converter does to the rpm, core/vehicle.cpp)
                logline("torque converter detected (stall %.2f%s, lock gear %s): creep and launch are the converter's, the clutch is never driven",
                        V.tc_stall, V.tc_stall_fixed ? " as set" : ", learned from the launches from here", V.tc_lock_gear > 0 ? "as set" : "learned");
            }
        }
    }
    // Learn the vehicle model's rpm correction while the clutch is closed and the speed steady. On a torque-converter box the
    // engine runs faster than the driveline below the lockup (kr > 1), so nothing is learned from a slipping converter: the tire
    // correction, the force correction and the shift points all assume a locked driveline. (v in m/s: over 18 km/h.)
    if (g > 0 && v > 5 && clutch_seen() < 0.02f && T.rpm > 700) {
        const float raw = vehicle_rpm_at(g, v) / V.rpm_k;   // the driveline rpm (the thing the correction learns against), not the engine's
        if (raw > 300) {
            const float kr = T.rpm / raw;
            // The tire correction only off a converter: its rpm is the converter's (above the driveline's pulling, under it coasting), and the
            // gears' effective ratios (ratio_feed) take that.
            if (!V.tc) {
                float k = clampf(kr, 0.7f, 1.4f);
                V.rpm_k += 0.02f * (k - V.rpm_k);
                // An engine slower than the driveline with the clutch closed for a long time is not a box we model (a hybrid,
                // an unusual mod): stay off. A converter never does this (it only slips faster).
                if (kr < 0.7f) *rpm_odd_s += step;
                else *rpm_odd_s = fmaxf(0, *rpm_odd_s - 2 * step);
                if (*rpm_odd_s > 20) odd = 1;
            }
        }
    }
    // The lockup gear of a detected converter: the lowest gear seen coupled. This sat inside the tire learner's gate (over 18 km/h, engine over 700) AND inside the "engine 1.18 times over the
    // driveline" condition, so in detect mode it could never run (kr < 1.08 inside kr > 1.18): vehicle_tc_locked was false whatever the gear, the force test of WP-TC1 applied at every speed, and
    // f_scale / f_bias never learned on a converter (fl_step learns only when locked). Now: pulling (the throttle over 0.1: a coast reads under the driveline in any gear), the clutch closed, the
    // driveline over 300 rpm, the engine over 1.1 idle, the engine within 8 % of the driveline (the figure the detection's 1.18 stays clear of), for 0.4 s in that gear (four telemetry samples of the ride; a gear is coupled for a few tenths per launch before the next upshift, so the seconds are kept across launches) -- then that gear is
    // coupled, and the lock gear is the lowest such. A gear under the lock gear counts as unlocked; a lock gear that was set (the ini, a sim's prime) is not learned over.
    if (!V.tc) memset(s_coupled_s, 0, sizeof s_coupled_s);   // (another truck, or none yet: the seconds start over)
    if (V.tc && cfg.tc < 2 && cfg.lock <= 0 && !V.tc_lock_fixed && g > 0 && g < 32 && clutch_seen() < 0.02f && T.eff_thr > 0.1f && T.rpm > 1.1f * cfg.idle) {
        const float raw = vehicle_rpm_at(g, v) / V.rpm_k;
        if (raw > 300 && T.rpm < 1.08f * raw) {
            s_coupled_s[g] += step;
            if (s_coupled_s[g] > 0.4f && (V.tc_lock_gear == 0 || g < V.tc_lock_gear)) {
                V.tc_lock_gear = g;
                logline("torque converter: coupled in gear %d (engine %.0f rpm over a driveline at %.0f): the lock gear is %d", g, T.rpm, raw, g);
            }
        }
    }
    vehicle_tc_stall_step(clutch_seen());   // (the converter's stall ratio, learned from the launches: core/vehicle.cpp)
    return odd;
}

// ---------------------------------------------------------------- the force correction

void fl_reset(force_learner *f)
{
    f->F = f->R = 0;
    f->w = f->sf = f->sff = f->sy = f->sfy = 0;
    f->aprev = T.accel; f->atprev = sim_s();
    f->eff_settle = -1e9; f->eff_prev = -1;
}

// The vehicle model's force correction, two parameters learned at once: V.f_scale (a multiplier on the modeled engine force:
// a truck the model under- or over-rates by a factor) and V.f_bias (a constant extra resistance, as before). The measurement
// is the engine's real pull y = mass * a + resistance-without-f_bias = f_scale * F - f_bias, with F the engine force the model
// gives at scale 1 (force_at / f_scale). The caller has already lagged F and the resistance with the same 0.5 s first-order
// lag as T.accel, so the two stay consistent while the force changes. A recursive least squares with a forgetting factor of
// ~60 s of samples solves for both. f_scale is only identifiable while F varies, so when its variance is small the slope is
// left as it is and only f_bias moves (otherwise a steady throttle folds the whole factor into f_bias as a fake resistance,
// the old behaviour).
static void fl_learn(force_learner *f, float eff, double dt, double now)
{
    // steady gates: the throttle stayed within +-0.03 for the last second, and the acceleration is not changing fast
    // (against the value it settled at, not frame to frame: a slow ramp moves 0.01 a frame and would pass that)
    if (!(f->eff_prev >= 0) || fabsf(eff - f->eff_prev) > 0.03f) { f->eff_settle = now; f->eff_prev = eff; }
    double adt = now - f->atprev;
    float da = adt > 0 ? (float)((T.accel - f->aprev) / adt) : 1e9f;
    f->aprev = T.accel; f->atprev = now;
    if (now - f->eff_settle < 1.0) return;
    if (fabsf(da) > 0.5f) return;

    double lam = 1.0 - dt / 60.0; if (lam < 0.9) lam = 0.9;   // forget over ~60 s of samples
    double F = f->F;
    double y = V.mass * (double)T.accel + f->R;
    f->w   = lam * f->w   + 1;
    f->sf  = lam * f->sf  + F;
    f->sff = lam * f->sff + F * F;
    f->sy  = lam * f->sy  + y;
    f->sfy = lam * f->sfy + F * y;
    if (f->w < 20) return;   // not enough samples yet

    // the slope from the window's covariance, pulled toward 1 by a prior worth a 5 % spread of F (ridge): with a throttle that hardly
    // varied the window cannot tell a factor from an offset, and the slope then stays near 1 while the offset takes the residual
    double fm = f->sf / f->w, ym = f->sy / f->w;
    double fvar = fmax(0.0, f->sff / f->w - fm * fm), cov = f->sfy / f->w - fm * ym;
    double prior = 0.05 * 0.05 * fm * fm;
    float s = (float)((cov + prior * 1.0) / (fvar + prior + 1e-9));
    float b = (float)(s * fm - ym);
    // the window's solution is followed slowly (10 s), never jumped to: every force the shift logic weighs carries f_scale, and a
    // fresh solve can swing when the throttle's spread crosses the identifiability edge
    float lim = V.mass * 9.81f * 0.02f, k = (float)(dt / (10.0 + dt));
    V.f_scale += k * (clampf(s, 0.85f, 1.25f) - V.f_scale);
    V.f_bias += k * (clampf(b, -lim, lim) - V.f_bias);
}

void fl_step(force_learner *f, int g, float v, float eff, double dt, double now, double last_brake_at, double shift_at, int eb_active)
{
    float cl = clutch_seen();
    // The lag runs every frame (also when the learner below is gated) so it never falls out of step with T.accel; F is
    // the engine force the wheels see, 0 while the clutch is open (a shift's torque gap, when the accel says coast).
    float F0 = cl < 0.02f ? vehicle_force_at(g, v, eff) / V.f_scale : 0.0f;
    float R0 = vehicle_resist(v) - V.f_bias;      // the modeled resistance without the learned bias
    float k = (float)(dt / (ACCEL_LAG_S + dt));   // the same lag as the telemetry's acceleration
    f->F += k * (F0 - f->F);
    f->R += k * (R0 - f->R);
    // (only in the engine's usable band, from the torque plateau's start to the top of its power range: above it the curve falls
    // steeply toward the limit and the 0.5 s lag of the force no longer matches the telemetry's; sim, the V8 revving to its
    // power top drifted f_scale to 1.035 with the plant equal to the model)
    // (and 3 s after any brake, the game's or our own, as after a shift: the lagged acceleration is still deep in the braking while the
    // truck already pulls; a handful of such windows tilted the fit, sim: f_scale 1.035 with the plant equal to the model)
    const bool band_ok = T.rpm >= 0.8f * V.plateau && T.rpm <= V.eng.range[R_POWER][1] && !sm_thr_capped() && now - last_brake_at > 3;
    if (cl < 0.02f && band_ok && v > 6 && eff > 0.1f && T.in_brake < 0.02f && !eb_active &&
        now - shift_at > 3 && (!V.tc || vehicle_tc_locked(g, v))) fl_learn(f, eff, dt, now);   // (3 s: the 0.5 s lag of the shift's torque gap has died out, e^-6; a converter only when locked)
}

// ---------------------------------------------------------------- the effective ratios of a converter box

// The calibration of a converter box's effective ratios (core/vehicle.cpp, "the effective ratios of a converter box"): the measured engine rpm
// per speed in the gear, from the two moments the converter is nearest to 1:1, the pedal up with the car rolling (the wheels drive it) and a
// steady moderate pull (it drives). A sample counts after the state has held for a second: the same gear for 1.5 s after any shift, the same
// class of pedal, the throttle and the ratio itself steady (quarter-second steps), the clutch closed, no engine brake, and for the coast the
// rpm well over the idle in gear (the game keeps ~870 under it whatever the speed). Nothing happens off a converter.
void ratio_feed(ratio_feeder *r, int g, float v, double now, double shift_at, float idle)
{
    double &at = r->at, &ref_at = r->ref_at, &held = r->held, &gsince = r->gsince;
    int &pg = r->pg, &pcls = r->pcls;
    float &pq = r->pq, &pthr = r->pthr;
    double dt = now - at;
    if (dt == 0) return;   // (a second call in the same frame: the hooks call refresh more than once)
    at = now;
    float base = V.tc && g > 0 ? vehicle_ratio_base(g) : 0;
    if (!(base > 0) || T.paused || !(dt > 0 && dt < 0.25) || clutch_seen() > 0.02f || v < 4.5f || !(T.rpm > 0) || T.motor_brake || T.retarder > 0) { held = 0; pg = 0; return; }
    if (g != pg) { gsince = now; held = 0; pg = g; pcls = 0; }
    float q = T.rpm / (v * base), thr = T.eff_thr;
    int cls = T.in_thr < 0.02f && thr < 0.03f ? 1 : thr > 0.08f && thr < 0.7f ? 2 : 0;
    if (cls != pcls || cls == 0) { pcls = cls; held = 0; ref_at = now; pq = q; pthr = thr; return; }
    if (now - ref_at >= 0.25) {
        if (fabsf(q - pq) > 0.012f || fabsf(thr - pthr) > 0.05f) held = 0;
        ref_at = now; pq = q; pthr = thr;
    }
    held += dt;
    if (held < 1.0 || now - gsince < 1.5 || now - shift_at < 1.5) return;
    if (cls == 1 ? !(T.rpm > 1.35f * idle) : !(T.rpm < 0.8f * V.eng.rpm_limit)) return;
    vehicle_ratio_sample(g, v, T.rpm, (float)dt, cls == 1 ? RATIO_COAST : RATIO_PULL);
}
