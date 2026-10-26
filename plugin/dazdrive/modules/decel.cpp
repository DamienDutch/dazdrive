// Decel: a closed-loop deceleration assist for the retarder and engine-brake lever. It adds service brake; it lowers what the
// game's own retarder does only with soften (below) and the retarder hook of [core] retarder_hook.
//
// While the driver holds a lever step (T.retarder, T.motor_brake) and both pedals are free, the assist aims at the target
// deceleration of that step and measures the real one (the speed's derivative, smoothed over 0.3 s). The retarder comes first
// and is already on; whatever is missing the service brake adds through the actuator (ACT_DECEL). No brake calibration: the
// loop closes on the measured deceleration, and what it learns (request per m/s2) is in the request's own unit.
// Three things it is for (brake-measure notes): the gearbox shift (clutch open ~1.1 s: the retarder falls to a floor within
// ~0.2 s and is back ~0.1-0.4 s after the clutch closes), the last metres (the retarder fades to nothing below ~6 km/h, the
// truck then coasts), and a heavy load (the same step gives less deceleration, F/m).
//
// Structure (decel_tick): measure, gate, target of the step, jerk-limited reference, then a PI on the request with a feedforward
// at the shift: when the clutch opens, the level that held the target in the last gap (learned: request per m/s2 of the gap) is
// asked at once, and when the clutch closes the level goes back at once to what it was before the gap. At once both times: the
// retarder falls within 0.2 s and recovers within 0.3 s, the service brake follows a request with a lag of ~0.15 s, so a ramp
// would add its own length to the brake's lag (a dip at the start, an overshoot at the end); and the close is taken when it
// happens, not predicted (the gap's length varies by +-0.1 s, the retarder's recovery is 8 m/s3 at the top step). Once the gap
// has settled, one correction step for what the feedforward missed; it also measures the brake's gain (see decel_tick).
// Stop assist: the reference tapers with speed along a = sqrt(2 j v), the deceleration that a ramp of jerk j brings to 0 exactly
// at v = 0 (j from jerk_limit, so no km/h figure is needed), led by the brake's lag; at a standstill the assist stops posting,
// unless Auto Hold takes the stop over ([cruise] auto_hold_lever): then its request stays until the hold's has risen to it.
// Other brake owners (AEB, Auto Hold, hill start) keep their priority: the actuator merges by the biggest wish.
// Soften: the other side of the target. A light truck gets far more than the target from the step alone (empty, the first step
// can give several times it), and a brake cannot take that away. When the measured deceleration is over the target by more than
// a tolerance and none of our brake is on, a factor on the game's retarder torque (act_retarder_scale, ACT_RET_DECEL) comes down,
// an integral on the error over what the retarder gives at 1, rate-limited, with a dead band so the noise does not move it; it
// goes back up as soon as the deceleration is short, and while it is under 1 the brake does not add (the retarder comes first).
// In a gap the factor goes back to 1 as the retarder falls (the gap's feedforward is made for the game's own floor) and down to
// the factor of before as the retarder recovers, both ramped so the two together do not make a peak. Back to 1 at once when the
// assist ends, rate-limited when the lever is released. Only the player's truck gets it (core/actuate.cpp).
#include "../core/suite.h"
#include <math.h>
#include <string.h>

static float p_enabled, p_tmin, p_tmax, p_tcurve, p_jerk, p_max, p_soften;

static param g_params[] = {
    { "enabled", &p_enabled, 0, "1 = while a retarder or engine-brake step is held and both pedals are free, the service brake adds what is missing to reach that step's target deceleration (bridges the gearbox shifts, helps at low speed and with a heavy load). 0 = off" },
    { "target_min", &p_tmin, 0.8f, "target deceleration (m/s2) at the first retarder step" },
    { "target_max", &p_tmax, 3.0f, "target deceleration (m/s2) at the last retarder step" },
    { "target_curve", &p_tcurve, 1.0f, "how the steps between the first and the last are spread: 1 = evenly, below 1 = the lower steps get more" },
    { "jerk_limit", &p_jerk, 4.0f, "most the added braking may change per second (m/s3), rising and falling" },
    { "max_brake", &p_max, 0.6f, "most brake request (0-1) the assist may ask for" },
    { "soften", &p_soften, 1, "1 = when the step alone brakes harder than its target (a light truck), the game's retarder is made weaker until the target holds; full again when the lever is released. Needs [core] retarder_hook = 1. 0 = the retarder stays as the game has it" },
};

// Constants of the controller (physics of the measurements, not preferences).
static const float DC_G_NOM = 6.0f;      // m/s2 per request 1.0 the jerk limit is converted with (nominal; the real gain is learned by the loop, not needed)
static const float DC_FLOOR = 0.4f;      // m/s2 the truck still decelerates with while the clutch is open (measured 0.3-0.65)
static const float DC_WIN = 0.3f;        // s, the measured deceleration is the speed's fall over this window (delay: half of it)
static const float DC_TOL = 0.25f;       // m/s2 over the reference where the overshoot guard backs off at twice the rate
static const float DC_FF_S = 0.2f;       // s, the retarder's fall at the start of a shift (the measurement shows it this long)
static const float DC_REL_S = 0.3f;      // s, the retarder's recovery after the clutch closes (the measurement shows it this long)
static const float DC_LAG_S = 0.15f;     // s, the service brake's lag behind the request
static const float DC_FF_FIRST = 0.75f;  // share of the nominal feedforward at the first shift, before the brake's gain is known
static const float DC_REFM_S = 0.3f;     // s, the delay of the measurement path (window + brake lag): the reference is compared with the measurement through the same delay
static const float DC_TAPER_J = 0.5f;    // the taper to a standstill uses this share of the jerk limit: the brake's lag and the measurement's delay eat the rest
static const float DC_BLIND_S = 0.35f;   // s, the measurement (window delay, brake lag) needs this long to show what a change of the level did
static const float DC_G_FIX = 4.5f;      // m/s2 per request 1.0 the first gap's correction assumes when it is short (see decel_tick)
static const float DC_FIX_E = 0.15f;     // m/s2 of error in a settled gap worth the gap's correction step
static const float DC_KP = 0.15f;        // request per m/s2 of error
static const float DC_KI = 0.30f;        // request per m/s2 of error per second
static const float DC_STOP_V = 0.08f;    // m/s, standing (0.3 km/h, as LowSpeed): the assist stops posting
static const float DC_PEDAL = 0.02f;     // a pedal above this ends the assist
static const float DC_OPEN = 0.05f;      // clutch above this is open (a shift), under DC_CLOSED closed again
static const float DC_CLOSED = 0.02f;
static const float DC_BAD_S = 0.5f;      // s without a usable speed derivative: the assist lets go
static const float DC_HAND_S = 0.6f;     // s at most the request waits at a standstill for Auto Hold's to rise (its ramp to 0.6 takes 0.2 s)
static const float DC_SOFT_TOL = 0.15f;  // m/s2 over the target before the retarder is lowered
static const float DC_SOFT_DB = 0.08f;   // m/s2 dead band around the target where the factor holds (the measurement's noise is ~0.03)
static const float DC_SOFT_KI = 2.0f;    // per second: share of the error (over the retarder's deceleration at 1) the factor corrects
static const float DC_SOFT_DN = 0.5f;    // factor per second, falling
static const float DC_SOFT_UP = 1.0f;    // factor per second, rising
static const float DC_SOFT_MIN = 0.2f;   // the lowest factor
static const float DC_SOFT_U = 0.01f;    // our request under this counts as none (the brake backs off before the retarder is lowered)
static const float DC_SOFT_GAP_UP = 0.4f;   // s, the factor's way back to 1 in a gap (twice the retarder's fall: the two together dip, not peak)
static const float DC_SOFT_GAP_DN = 0.2f;   // s, its way back down after the clutch closes (a little under the retarder's recovery)

// What one tick sees (the sim sets the same fields).
struct decel_in {
    float dt, speed, thr, brake, clutch;
    int gear, retarder, steps, motor_brake, engine_on, paused, cruise;
    float other;       // the brake wish of the cruise side (ACC, bends): it owns the brake then
    float held;        // the request the actuator holds now (brake air low: none of ours)
    float hold, take;  // Auto Hold: the request it holds now, and what it will hold after a stop by the lever (0 = it will not take one)
    int soften;        // the retarder factor may be posted (soften on, the hook in)
};

static struct {
    int on;               // working: lever on, nothing stops it
    float a;              // measured deceleration (m/s2, + = slowing)
    float hv[64], hy[64]; // the speed samples of the last moments, and the brake model at each
    double ht[64], clk;   // their (running) times (double: the clock runs for hours)
    float y;              // the brake model: the request through the service brake's lag (what of it acts now)
    float ya;             // the model's mean over the measurement's window: what of it the measured deceleration shows
    int hn, hi;
    float bad_s;          // seconds without a usable derivative
    float tgt, ref, refm; // the step's target (tapered), the jerk-limited reference toward it, and that reference as the measurement can see it (delayed like it)
    float lvl, u;         // the integrator (the level that holds the reference), the request posted
    int gap;              // clutch open or neutral
    float gap_s;          // seconds in the gap
    int fix;              // the gap's correction: 0 = not yet, 1 = stepped (waiting to see its effect), 2 = done
    float fix_y, fix_a;   // the brake model as the measurement shows it, and the measured deceleration, before the step (a two-point gain measurement)
    float fix_r;          // the request's rate while the correction ramps in (the jerk limit with the gain the step assumed)
    float pre;            // the level before the gap (what the release goes back to)
    float blind;          // seconds left in which the measurement still shows the shift's transient, not an error to correct
    float k = 1.0f / DC_G_NOM;   // learned request per m/s2 (the inverse of the brake's gain), the nominal one until a gap has taught it
    int k_known;
    int k_fix;            // k comes from a gap's correction step (measured, not inferred from the level at the close)
    int step, n;          // what the target was made for (log)
    float hand_s;         // seconds standing while Auto Hold takes over
    float kr = 1.0f;      // soften: the loop's retarder factor (held in a gap)
    float ks = 1.0f;      // the factor shaped through a gap (to 1 as the retarder falls, back as it recovers)
    float kp = 1.0f;      // the factor posted last
} D;

// The target deceleration of a lever step, 1..n of n steps.
static float decel_target(int step, int n)
{
    if (n < 1) n = 1;
    if (step < 1) step = 1;
    if (step > n) step = n;
    if (n == 1) return p_tmax;
    float f = (float)(step - 1) / (float)(n - 1), c = p_tcurve > 0.05f ? p_tcurve : 0.05f;
    return p_tmin + (p_tmax - p_tmin) * powf(f, c);
}

static void soft_post(float k) { if (k != D.kp) { D.kp = k; act_retarder_scale(ACT_RET_DECEL, k); } }
static void soft_off() { D.kr = D.ks = 1.0f; soft_post(1.0f); }

static void decel_off()
{
    D.on = 0; D.lvl = D.u = D.pre = 0; D.gap = 0; D.gap_s = 0; D.blind = 0; D.bad_s = 0;
    act_brake(ACT_DECEL, 0);
}

// The clutch closed: learn what the gap needed (the level now, plus what the error left would still have needed; not at the
// ceiling, where the error says nothing about the gain) and go back to the level before the gap.
static void decel_release(float need)
{
    if (!D.k_fix && D.gap_s > 0.6f && need > 0.3f && D.lvl < p_max - 1e-3f) {
        float kn = (D.lvl + (D.refm - D.a) * D.k) / need;
        if (kn > 0.2f / DC_G_NOM && kn < 5.0f / DC_G_NOM) { D.k = D.k_known ? 0.5f * (D.k + kn) : kn; D.k_known = 1; }
    }
    D.lvl = fminf(D.lvl, D.pre);
    D.blind = DC_REL_S + DC_BLIND_S;
}

static void decel_tick(const decel_in &in)
{
    float dt = in.dt;
    if (!(dt > 0 && dt < 0.5f)) { if (dt != 0) D.hn = 0; return; }   // paused or a hitch: no time step, and the speed history is broken
    // the measured deceleration: the speed's derivative, smoothed; a sample that is not a number or looks like noise is dropped
    float v = fabsf(in.speed);
    int ok = 0;
    D.y += (D.u - D.y) * dt / (DC_LAG_S + dt);
    if (in.speed == in.speed) {
        D.clk += dt;
        D.hi = (D.hi + 1) % 64; D.hv[D.hi] = v; D.hy[D.hi] = D.y; D.ht[D.hi] = D.clk;
        if (D.hn < 64) D.hn++;
        float ys = D.y;
        for (int i = 1; i < D.hn; i++) {   // the newest sample that is at least the window old
            int j = (D.hi - i + 64) % 64;
            float span = (float)(D.clk - D.ht[j]);
            if (span >= DC_WIN) {
                float raw = (D.hv[j] - v) / span;
                if (fabsf(raw) < 30.0f) { D.a = raw; D.ya = ys / (float)i; ok = 1; }
                break;
            }
            ys += D.hy[j];
        }
    }
    D.bad_s = ok ? 0 : D.bad_s + dt;
    float am = D.a;
    // the actuator does not write our request (brake air low): the level must not wind up meanwhile
    int starved = D.u > 0.02f && in.held < 0.5f * D.u;
    int lever = in.retarder > 0 || in.motor_brake;
    // Ends at once: the driver's pedals, cruise (it owns the brake), reverse, engine off, a pause, no usable measurement, standing.
    // Without the lever the request winds down at the jerk limit instead.
    int ends = in.paused || !in.engine_on || in.gear < 0 || in.thr > DC_PEDAL || in.brake > DC_PEDAL || in.cruise || in.other > 0 ||
               D.bad_s > DC_BAD_S, standing = !(v > DC_STOP_V), hard = ends || standing;
    float rlim = p_jerk * D.k;          // request per second at the jerk limit (the learned request per m/s2, the nominal one at first)
    // Standing with nothing else ending it and Auto Hold taking the stop: the request stays as it is until the hold's has risen to it
    // (or to what the hold will hold, if less), then it goes at once: the actuator merges by the biggest wish, so the brake never lets go.
    if (!standing) D.hand_s = 0;
    else if (!ends && D.on && in.take > 0) {
        D.hand_s += dt;
        if (D.hand_s < DC_HAND_S && in.hold < fminf(D.u, in.take) - 1e-3f) { act_brake(ACT_DECEL, D.u); soft_off(); return; }
    }
    if (hard) { if (D.on) decel_off(); soft_off(); return; }
    if (!lever) {
        if (D.kr < 1) { D.kr = D.ks = fminf(D.kr + DC_SOFT_UP * dt, 1.0f); soft_post(D.ks); }   // the lever released: back up, not a jolt
        if (D.on) {
            D.u = fmaxf(D.u - rlim * dt, 0);
            D.lvl = D.u;
            act_brake(ACT_DECEL, D.u);
            if (D.u <= 0) decel_off();
        }
        return;
    }
    if (!D.on) {   // starting: the reference starts at what the truck does now and climbs at the jerk limit, as the retarder builds up
        D.on = 1; D.lvl = D.u = D.pre = 0; D.gap = 0; D.gap_s = 0; D.blind = 0; D.hand_s = 0;
        D.ref = D.refm = fmaxf(D.a, 0);
    }
    int step = in.retarder > 0 ? (int)in.retarder : 1;   // an engine brake alone counts as the first step (of however many)
    int n = in.retarder > 0 ? (in.steps > step ? in.steps : step) : (in.steps > 1 ? in.steps : 2);
    D.step = step; D.n = n;
    // the target, tapered toward a standstill along a = sqrt(2 j v) (a ramp of jerk j from a ends exactly at v = 0); the brake
    // lags, so the request leads that ramp by the lag: j x lag less (else the truck arrives braking and the last step is a jolt)
    float tgt = decel_target(step, n), jt = DC_TAPER_J * fmaxf(p_jerk, 0.1f);
    float taper = sqrtf(2.0f * jt * v) - jt * DC_LAG_S;
    if (taper < tgt) tgt = fmaxf(taper, 0);
    D.tgt = tgt;
    float dr = fmaxf(p_jerk, 0.1f) * dt;
    float ref0 = D.ref;
    D.ref += D.ref < tgt ? fminf(dr, tgt - D.ref) : -fminf(dr, D.ref - tgt);
    // the gap: the clutch opens at a shift, or the box is in neutral (stop assist)
    int open = in.clutch > DC_OPEN || in.gear == 0, closed = in.clutch < DC_CLOSED && in.gear != 0;
    float need = fmaxf(D.ref - DC_FLOOR, 0);
    int step_now = 0;   // the level jumped (the shift's feedforward or its release): the request follows at once
    float lvl = D.lvl;
    if (!D.gap && open) {
        D.gap = 1; D.gap_s = 0; D.fix = 0; D.pre = lvl;
        // the level that held the target in the last gap, at once; before a gap has taught it, only part of what the nominal brake
        // would need (too little the loop makes up in a moment, too much is a lurch)
        lvl = fminf(fmaxf(D.k * need * (D.k_known ? 1.0f : DC_FF_FIRST), lvl), p_max);
        D.blind = DC_FF_S + DC_WIN;
        step_now = 1;
    } else if (D.gap && closed) {
        D.lvl = lvl;
        decel_release(need);
        lvl = D.lvl;
        D.gap = 0;
        step_now = 1;
    } else if (D.gap) D.gap_s += dt;
    D.refm += dt / (DC_REFM_S + dt) * (D.ref - D.refm);
    if (in.soften) {
        // soften: lowered only while over the target with none of our brake on, raised while short (out of the transients)
        if (!D.gap && D.blind <= 0) {
            float over = am - D.tgt, r = fmaxf(am / D.kr, 0.5f), dk = 0;   // r: the deceleration at 1 (the drag in it too: a lower gain, never a higher one)
            if (over > DC_SOFT_DB && D.u < DC_SOFT_U && (D.kr < 1 || over > DC_SOFT_TOL)) dk = -DC_SOFT_KI * (over - DC_SOFT_DB) / r;
            else if (over < -DC_SOFT_DB && D.kr < 1) dk = DC_SOFT_KI * (-over - DC_SOFT_DB) / r;
            D.kr = fminf(fmaxf(D.kr + fminf(fmaxf(dk, -DC_SOFT_DN), DC_SOFT_UP) * dt, DC_SOFT_MIN), 1.0f);
        }
        if (D.gap) D.ks = fminf(D.ks + (1 - D.kr) / DC_SOFT_GAP_UP * dt, 1.0f);
        else D.ks = D.kr >= D.ks ? D.kr : fmaxf(D.kr, D.ks - (1 - D.kr) / DC_SOFT_GAP_DN * dt);
        soft_post(D.ks);
    }
    float e = D.refm - am;
    if (in.soften && D.kr < 1 && !D.gap && e > 0) e = 0;   // short with the retarder lowered: it comes back first, the brake waits
    int guard = am > D.refm + DC_TOL;
    // the level: the steps of the shift above, the PI integral on top (rate-limited, no windup)
    float up = rlim, dn = guard ? 2 * rlim : rlim;
    if (D.gap && D.fix == 1) { up = fmaxf(up, D.fix_r); dn = fmaxf(dn, D.fix_r); }
    // the feedback waits while the measurement still shows the transient of the last change (only the overshoot guard acts then)
    if (D.blind > 0) { D.blind -= dt; if (!guard) e = 0; }
    else if (D.gap && D.fix == 0 && in.gear != 0 && !starved) {
        // The gap has settled (the retarder at its floor, the feedforward in): what is still off is the feedforward's error (the
        // first gap's guess of the gain, a new load). The integral is rate-limited and slow next to a gap of 1.1 s, so one step of
        // the level for it, ramped at the jerk limit. Before a gap has taught the gain: a shortfall means a brake weaker than the
        // first share assumed (8 m/s2), so the step assumes 4.5 (the leftover is then 0.1 of the need both for the weakest brake
        // made for, 3.6, and for the nominal 6); an excess means a stronger brake, the nominal gain. The step is also a clean
        // measurement of the gain: the floor of the gap does not move between before and after it.
        D.fix = 2;
        if (fabsf(e) > DC_FIX_E) {
            float kc = D.k_known || e < 0 ? D.k : 1.0f / DC_G_FIX;
            // (less what of the request the measurement does not show yet: the brake's lag and the window still hold part of the feedforward)
            float to = fminf(fmaxf(lvl + e * kc - fmaxf(D.u - D.ya, 0), 0), p_max);
            if (fabsf(to - lvl) > 0.02f) {
                D.fix = 1; D.fix_y = D.ya; D.fix_a = am; D.fix_r = fmaxf(p_jerk, 0.1f) * kc;
                D.blind = fabsf(to - lvl) / D.fix_r + DC_BLIND_S;
                lvl = to;
            }
        }
        e = 0;
    } else if (D.gap && D.fix == 1) {
        // the step's effect is in the measurement: the gain, request per m/s2 (the floor of the gap cancels; the brake model, seen
        // through the same window as the measurement, says how much of the step the measured deceleration holds by now)
        D.fix = 2;
        // (only a step big enough to stand out of the noise; a later one refines the first by half)
        float dy = D.ya - D.fix_y, da = am - D.fix_a, kn = da != 0 ? dy / da : 0;
        if (fabsf(dy) > 0.04f && fabsf(da) > 0.2f && kn > 1.0f / 12.0f && kn < 1.0f / 2.0f) {
            D.k = D.k_fix ? 0.5f * (D.k + kn) : kn; D.k_known = D.k_fix = 1;
        }
    }
    // a falling reference (the taper to a stop, a lower step) takes its share of the level with it, and in a gap (the brake
    // alone holds the deceleration there) a rising one too: the integral alone would lag the ramp
    if (D.ref < ref0 || D.gap) lvl = fminf(fmaxf(lvl + (D.ref - ref0) * D.k, 0), p_max);
    float di = fminf(fmaxf(DC_KI * e, -dn), starved ? 0 : up) * dt;
    if (!((lvl >= p_max && di > 0) || (lvl <= 0 && di < 0))) lvl += di;
    D.lvl = fminf(fmaxf(lvl, 0), p_max);
    float want = fminf(fmaxf(D.lvl + DC_KP * e, 0), p_max);
    D.u = step_now ? want : fminf(fmaxf(want, D.u - dn * dt), D.u + up * dt);
    act_brake(ACT_DECEL, D.u);
}

static void de_frame()
{
    static double last;
    double now = sim_s();
    decel_in in;
    in.dt = (float)(now - last);
    last = now;
    if (p_enabled <= 0 || !act_hooked()) { if (D.on) decel_off(); soft_off(); return; }
    const act_wish *w = act_wishes();
    in.speed = T.speed; in.thr = T.in_thr; in.brake = T.in_brake; in.clutch = clutch_seen();   // our own clutch (crawl, smooth start) counts
    in.gear = T.gear; in.retarder = T.retarder; in.steps = (int)T.retarder_steps; in.motor_brake = T.motor_brake;
    in.engine_on = T.engine_on; in.paused = T.paused; in.cruise = T.cruise > 0.1f;
    in.other = fmaxf(fmaxf(w[ACT_CRUISE].brake, w[ACT_ACC].brake), w[ACT_BEND].brake);
    in.held = act_request();
    cruise_info cu;
    cruise_info_get(&cu);
    in.hold = w[ACT_HOLD].brake; in.take = cu.lever_take;
    in.soften = p_soften > 0 && act_retarder_hooked();
    bool was = D.on, soft_was = D.kr < 1;
    decel_tick(in);
    if (D.on != was) logv(1, "Decel: %s (step %d of %d, target %.2f m/s2, %.0f km/h)", D.on ? "on" : "off", D.step, D.n, D.tgt, fabsf(T.speed) * 3.6f);
    if ((D.kr < 1) != soft_was) logv(1, "Decel: retarder %s (step %d of %d, target %.2f m/s2, measured %.2f, %.0f km/h)", D.kr < 1 ? "softened" : "back to the game's own", D.step, D.n, D.tgt, D.a, fabsf(T.speed) * 3.6f);
}

// Continuity broke (pause, hitch, other truck, a new load): no state carries over. The learned request per m/s2 depends on the load.
static void de_reset(int level, const char *)
{
    D.on = 0; D.lvl = D.u = D.pre = 0; D.gap = 0; D.gap_s = 0; D.blind = 0; D.bad_s = 0; D.hn = 0; D.a = 0; D.y = D.ya = 0;
    if (level >= RESET_LOAD) { D.k_known = D.k_fix = 0; D.k = 1.0f / DC_G_NOM; }
    act_brake(ACT_DECEL, 0);
    soft_off();
}

static char g_status[96] = "off (ini)";
static const char *de_status() { return g_status; }

static void de_start()
{
    if (p_enabled <= 0) return;
    p_max = p_max > 0 ? fminf(p_max, 1.0f) : 0;   // a request is 0..1 (and a NaN from the ini is none)
    if (!(p_jerk >= 0.1f)) p_jerk = 0.1f;         // the slews divide and multiply by it
    snprintf(g_status, sizeof g_status, "on: %.1f-%.1f m/s2, jerk %.1f m/s3, max brake %.2f%s%s", p_tmin, p_tmax, p_jerk, p_max,
             p_soften > 0 && g_retarder_hook >= 0.5f ? ", soften" : "", act_hooked() ? "" : " (no actuator hook)");
    static bool told;
    if (p_soften > 0 && g_retarder_hook < 0.5f && !told) { told = true; logline("Decel: soften is on but [core] retarder_hook is 0: the game's retarder stays as it is, the service brake only adds"); }
}

static void de_stop() { act_brake(ACT_DECEL, 0); soft_off(); }

module m_decel = { "decel", "Decel", g_params, sizeof g_params / sizeof g_params[0], de_start, de_frame, NULL, de_stop, de_reset, de_status };
