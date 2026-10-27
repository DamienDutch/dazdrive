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
// In a gap (without the ramp, below) the factor goes back to 1 as the retarder falls (the gap's feedforward is made for the
// game's own floor) and down to the factor of before as the retarder recovers, both ramped so the two together do not make a
// peak. Back to 1 at once when the assist ends, rate-limited when the lever is released. Only the player's truck gets it
// (core/actuate.cpp).
// Ramp: the game may bring its retarder back at full strength the moment the clutch closes (the lower gear's engine brake with
// it) while the brake still holds the gap's level, and a factor could follow the close only a frame later: the deceleration
// jumps. With ramp_s and the hook (and a retarder step held) the retarder's comeback is ours: in a gap the factor goes to 0 over
// the retarder's fall time, the brake's lag after the open (the feedforward counts no floor: the brake holds the whole target
// there), and from the close the level goes linearly from the gap's to the one before it while the factor climbs from 0 to its
// own (soften's, or 1) as the brake model gives up the level, so the two sum to the target on the way. The build-up is
// jerk-limited: it lasts as long as the deceleration it hands over (the level's change over the brake's gain) needs at
// jerk_limit, so a small difference is done almost at once and a large one takes longer; ramp_s is only the longest it may
// take. Without the hook all of this is off (one log line) and the close is as above.
// Gear ratio: the retarder and the engine brake work through the gearbox, so after a downshift the same step brakes harder at the
// wheels. The level the brake goes back to at the close is the one that holds the target with the retarder's share of before times the
// change of the ratio (vehicle_rpm_at of the two gears), and when that share alone is over the target soften's factor starts lower.
// Pre-blend (pre_blend): TrueShift can tell when its next downshift in braking is due (ts_brake_down_eta: the speed falling to the
// point where its braking rules judge the gear, at the deceleration of now). Once that is nearer than the hand-over needs (the
// retarder's share over the jerk limit, the brake's lag and a margin), the share goes from the retarder to the service brake at the
// jerk limit, the factor following the brake model, so the clutch opens on a brake that already holds the target. It unwinds the same
// way when the shift does not come, and is given up after a while when it is ready and nothing happens. Without the hook (or with
// ramp_s 0) only the brake rises ahead of the shift, by a small amount (the retarder stays: more would overshoot).
// Downshift assist (downshift_assist): the order is the retarder, then a lower gear, then the service brake. While the brake is needed (its level outside
// the gaps over DC_NEED, 1.5 s after the lever was pulled, when the retarder has built up) g_decel_need tells TrueShift, which then
// takes a lower gear that puts the engine into its own brake range (modules/trueshift.cpp, assist_down) earlier than its own rules do.
#include "../core/suite.h"
#include "../core/vehicle.h"
#include "../core/rpmspan.h"
#include <math.h>
#include <string.h>

static float p_enabled, p_tmin, p_tmax, p_tcurve, p_jerk, p_max, p_soften, p_ramp, p_pre, p_dsa, p_sa, p_sad;

static param g_params[] = {
    { "enabled", &p_enabled, 0, "1 = while a retarder or engine-brake step is held and both pedals are free, the service brake adds what is missing to reach that step's target deceleration (bridges the gearbox shifts, helps at low speed and with a heavy load). 0 = off" },
    { "target_min", &p_tmin, 0.8f, "target deceleration (m/s2) at the first retarder step" },
    { "target_max", &p_tmax, 3.0f, "target deceleration (m/s2) at the last retarder step" },
    { "target_curve", &p_tcurve, 1.0f, "how the steps between the first and the last are spread: 1 = evenly, below 1 = the lower steps get more" },
    { "jerk_limit", &p_jerk, 4.0f, "most the added braking may change per second (m/s3), rising and falling" },
    { "max_brake", &p_max, 0.6f, "most brake request (0-1) the assist may ask for" },
    { "soften", &p_soften, 1, "1 = when the step alone brakes harder than its target (a light truck), the game's retarder is made weaker until the target holds; full again when the lever is released. Needs [core] retarder_hook = 1. 0 = the retarder stays as the game has it" },
    { "ramp_s", &p_ramp, 1.0f, "most seconds the game's retarder takes to build back up after a gearbox shift. The service brake hands over at the rate the jerk limit allows (a small difference takes a fraction of a second, a large one longer, never more than this), so the deceleration does not jump when the clutch closes; in the shift the brake alone holds the target. Needs [core] retarder_hook = 1. 0 = the retarder comes back as the game has it" },
    { "pre_blend", &p_pre, 1, "1 = when the gearbox is about to shift down while braking, the retarder's share is handed to the service brake before the clutch opens, at the jerk limit, so the deceleration holds through the shift. The retarder's side needs [core] retarder_hook = 1 and ramp_s above 0; without it only the service brake rises ahead of the shift, by a small amount. 0 = the brake takes over when the clutch opens" },
    { "downshift_assist", &p_dsa, 1, "1 = while the service brake is needed to reach the target, the gearbox shifts down earlier when a lower gear puts the engine into its own engine-brake rpm range (never near the top of the range or over the rpm limit): the retarder and the engine brake work through the lower gear before the service brake does. 0 = the gear is chosen as without the assist" },
    { "stop_assist", &p_sa, 1, "What happens when you lift off at low speed and the gearbox opens the clutch by itself (no throttle, no brake, no retarder or engine-brake lever, no cruise): 0 = the truck rolls on in neutral, as in the game. 1 = the service brake slows it gently to a standstill and lets go there. 2 = the same, and Auto Hold holds the truck at the standstill (needs Auto Hold and its lever setting). The throttle or the brake pedal ends it at once. Works with the deceleration assist on or off" },
    { "stop_assist_decel", &p_sad, 0.5f, "deceleration (m/s2) of the stop assist; it eases off along the way to the standstill. Keep it under 0.7, the level below which a car's regenerative braking does not light the brake lights" },
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
static const float DC_FF_PRIOR = 0.9f;   // the same when the gain is the vehicle model's learned one (a good estimate, not a measurement of this event: a quarter off would overshoot at the full level)
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
static const float DC_PRE_MARGIN = 0.2f; // s, the hand-over before a shift is done this long ahead of the prediction (its error)
static const float DC_PRE_HYS = 0.3f;    // s, a hand-over under way holds while the shift is expected up to this much later than it started
static const float DC_PRE_OVS = 0.2f;    // m/s2, without the retarder hook the brake is raised ahead of a shift by this much at most (the retarder stays)
static const float DC_PRE_MIN = 0.1f;    // m/s2, a smaller share of the retarder is not worth preparing
static const float DC_PRE_HOLD_S = 1.5f; // s, a hand-over complete this long and no shift: given up (until the prediction clears)
static const float DC_NEED = 0.05f;      // request, the service brake that counts as needed for downshift_assist
static const float DC_NEED_S = 1.5f;     // s, the lever has been held this long (the retarder has built up) before the need counts
static const float DC_RAMP_TAIL = 0.6f;  // s, a build-up ends this long after its level did at the latest (four lags of the brake)
static const float DC_RAMP_MIN = 0.05f;  // s, the shortest build-up after a shift (a few frames: a hand-over of nothing is done at once)

// What one tick sees (the sim sets the same fields).
struct decel_in {
    float dt, speed, thr, brake, clutch;
    int gear, retarder, steps, motor_brake, engine_on, paused, cruise;
    float other;       // the brake wish of the cruise side (ACC, bends): it owns the brake then
    float held;        // the request the actuator holds now (brake air low: none of ours)
    float hold, take;  // Auto Hold: the request it holds now, and what it will hold after a stop by the lever (0 = it will not take one)
    int soften;        // the retarder factor may be posted (soften on, the hook in)
    int ramp;          // the retarder's comeback after a shift is ramped (ramp_s on, the hook in)
    int pre;           // pre_blend on
    float eta;         // seconds until the gearbox shifts down in braking (NAN = none expected)
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
    int g0;               // the gear as the gap began (the retarder's torque goes through the ratio: a lower gear brakes more)
    float blind;          // seconds left in which the measurement still shows the shift's transient, not an error to correct
    float k = 1.0f / DC_G_NOM;   // learned request per m/s2 (the inverse of the brake's gain), the nominal one until a gap has taught it
    int k_known;
    int k_prior;          // k comes from the vehicle model's learned gain ([core] brake_learn), not yet from a gap of this event
    int k_fix;            // k comes from a gap's correction step (measured, not inferred from the level at the close)
    int step, n;          // what the target was made for (log)
    float hand_s;         // seconds standing while Auto Hold takes over
    float kr = 1.0f;      // soften: the loop's retarder factor (held in a gap)
    float ks = 1.0f;      // the factor shaped through a gap (to 1 as the retarder falls, back as it recovers)
    float kp = 1.0f;      // the factor posted last
    int rg;               // ramp: this gap is one (the retarder taken off in it)
    float kg;             // the factor as the gap began (down to 0 from there)
    float rt = -1;        // seconds since the close while the ramp runs (-1 = none)
    float rdl, r0;        // the ramp's change of the level (gap's to before) and its factor at the close
    float rT;             // the ramp's length (s): the hand-over (m/s2) over the jerk limit, at most ramp_s
    float rs;             // the retarder's share of the ramp so far: the share of the hand-over the brake model has made
    float ry0, rye;       // the brake model at the close and the level the ramp ends at (the hand-over, in request)
    float ph;             // pre-blend: the deceleration (m/s2) of the retarder's share handed to the brake so far
    float pbs;            // pre-blend: that share as the retarder's factor follows it (through the brake's lag)
    float pb_s;           // pre-blend: seconds the complete hand-over has been held
    int pb_lock;          // the hold ran out: no new hand-over until the prediction clears
    float on_s;           // seconds the assist has been working (the retarder builds up over ~1 s after the lever is pulled: its shortfall then is no need)
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
static void soft_off() { D.kr = D.ks = 1.0f; D.rg = 0; D.rt = -1; D.ph = D.pbs = D.pb_s = 0; D.pb_lock = 0; soft_post(1.0f); }

static void decel_off()
{
    D.on = 0; D.lvl = D.u = D.pre = 0; D.gap = 0; D.gap_s = 0; D.blind = 0; D.bad_s = 0;
    act_brake(ACT_DECEL, 0);
}

// The clutch closed: learn what the gap needed (the level now, plus what the error left would still have needed; not at the
// ceiling, where the error says nothing about the gain) and go back to the level before the gap: at once, or with the ramp over
// ramp_s.
static void decel_release(float need, int g1, int soften)
{
    if (!D.k_fix && D.gap_s > 0.6f && need > 0.3f && D.lvl < p_max - 1e-3f) {
        float kn = (D.lvl + (D.refm - D.a) * D.k) / need;
        if (kn > 0.2f / DC_G_NOM && kn < 5.0f / DC_G_NOM) { D.k = D.k_known ? 0.5f * (D.k + kn) : kn; D.k_known = 1; D.k_prior = 0; }
    }
    // A lower gear: the retarder (and the engine brake) work through the ratio, so the same step brakes harder at the wheels. The level
    // the brake goes back to is the one that holds the target with the retarder's share of before times the change of the ratio.
    float rr = 1.0f;
    if (D.g0 > 0 && g1 > 0 && g1 < D.g0) {
        float a0 = vehicle_rpm_at(D.g0, 10.0f), a1 = vehicle_rpm_at(g1, 10.0f);
        if (a0 > 0 && a1 > a0) rr = fminf(a1 / a0, 2.0f);
    }
    if (rr > 1.0f) {
        float share = fmaxf(D.ref - D.pre / D.k, 0);   // what the retarder carried before the gap (m/s2)
        D.pre = fminf(D.pre, fmaxf(D.ref - share * rr, 0) * D.k);
        // (stronger alone than the target now: soften's factor starts from the one that holds the target, not from the old one)
        if (soften && share > 0.05f && share * rr > D.ref) D.kr = fminf(fmaxf(D.kr * D.ref / (share * rr), DC_SOFT_MIN), 1.0f);
    }
    float to = fminf(D.lvl, D.pre);
    if (D.rg) {
        D.rdl = to - D.lvl; D.rt = 0; D.rs = 0; D.r0 = D.kp; D.rg = 0; D.ry0 = D.y; D.rye = to;
        // the build-up takes as long as the change of the deceleration it hands over needs at the jerk limit: a small difference
        // is done at once, a large one takes longer; ramp_s is only the longest it may take
        D.rT = fminf(fmaxf(-D.rdl / D.k / fmaxf(p_jerk, 0.1f), DC_RAMP_MIN), fmaxf(p_ramp, DC_RAMP_MIN));
        D.ks = D.kr;   // soften's own way down after the close is the ramp's
        D.blind = DC_BLIND_S;
    } else {
        D.lvl = to;
        D.blind = DC_REL_S + DC_BLIND_S;
    }
}

static void decel_tick(const decel_in &in)
{
    float dt = in.dt;
    if (!(dt > 0 && dt < 0.5f)) { if (dt != 0) D.hn = 0; return; }   // paused or a hitch: no time step, and the speed history is broken
    // [core] brake_learn: the brake's gain as the vehicle model has measured it (core/vehicle.cpp) in place of the nominal one, until a gap has taught
    // this event its own: the first shift then asks the full level (no DC_FF_FIRST share) and a shortfall in the first gap is corrected with the real
    // gain (not DC_G_FIX). Inside the range the loop itself accepts (2-12 m/s2 per request); a load change resets k_known and brings this back at the new mass.
    if (g_brake_learn > 0 && !D.k_known) {
        float g = vehicle_brake_gain();
        if (g == g && g >= 2.0f && g <= 12.0f) { D.k = 1.0f / g; D.k_known = D.k_prior = 1; logv(1, "Decel: brake gain %.1f m/s2 per request from the vehicle model (nominal %.1f)", g, DC_G_NOM); }
    }
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
        if (D.rg || D.rt >= 0 || D.pbs > 0) { D.rg = 0; D.rt = -1; soft_post(D.ks); }   // in a ramp's gap or ramp, or a hand-over ahead of a shift: to soften's factor (1 without it)
        D.ph = D.pbs = D.pb_s = 0; D.pb_lock = 0;
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
        D.on = 1; D.rg = 0; D.rt = -1; D.lvl = D.u = D.pre = 0; D.gap = 0; D.gap_s = 0; D.blind = 0; D.hand_s = 0; D.ph = D.pbs = D.pb_s = 0; D.pb_lock = 0; D.on_s = 0;
        D.ref = D.refm = fmaxf(D.a, 0);
    }
    D.on_s += dt;
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
    int rg = D.gap ? D.rg : in.ramp && in.retarder > 0;   // a ramp's gap (the retarder taken off in it: no floor of its own)
    float need = fmaxf(D.ref - (rg ? 0 : DC_FLOOR), 0);
    int step_now = 0;   // the level jumped (the shift's feedforward or its release): the request follows at once
    float lvl = D.lvl;
    if (!D.gap && open) {
        if (D.rt >= 0) { lvl = fmaxf(lvl + D.rdl * (1 - fminf(D.rt / D.rT, 1.0f)), 0); D.rt = -1; }   // a ramp cut short: where it went
        D.g0 = in.gear;
        D.gap = 1; D.gap_s = 0; D.fix = 0; D.pre = fmaxf(lvl - D.k * D.ph, 0); D.rg = rg; D.kg = D.kp;   // (what a hand-over ahead of the shift added is not the level before the gap)
        D.ph = D.pbs = D.pb_s = 0; D.pb_lock = 0;
        // the level that held the target in the last gap, at once; before a gap has taught it, only part of what the nominal brake
        // would need (too little the loop makes up in a moment, too much is a lurch)
        lvl = fminf(fmaxf(D.k * need * (D.k_known ? (D.k_prior ? DC_FF_PRIOR : 1.0f) : DC_FF_FIRST), lvl), p_max);
        D.blind = DC_FF_S + DC_WIN;
        step_now = 1;
    } else if (D.gap && closed) {
        D.lvl = lvl;
        decel_release(need, in.gear, in.soften);
        lvl = D.lvl;
        D.gap = 0;
        step_now = 1;
    } else if (D.gap) D.gap_s += dt;
    if (D.rt >= 0) {   // the ramp's share of the level for this tick
        float s0 = fminf(D.rt / D.rT, 1.0f);
        D.rt += dt;
        lvl = fmaxf(lvl + D.rdl * (fminf(D.rt / D.rT, 1.0f) - s0), 0);
    }
    D.refm += dt / (DC_REFM_S + dt) * (D.ref - D.refm);
    // Pre-blend: the gearbox is about to shift down (its rpm falling to the point of a braking downshift at the deceleration of now).
    // When the shift is nearer than the hand-over needs (the retarder's share over the jerk limit, the brake's lag and a margin), the
    // retarder's share goes to the service brake at the jerk limit, the factor following the brake model, so that the clutch opens on a
    // brake that already holds the target. The share unwinds the same way when the shift does not come.
    if (!D.gap && D.rt < 0) {
        float base = fmaxf(lvl - D.k * D.ph, 0);
        float share = fmaxf(D.ref - base / D.k, 0);   // what the retarder carries now (m/s2)
        float cap = in.ramp ? share : fminf(share, DC_PRE_OVS);
        float near_s = share / fmaxf(p_jerk, 0.1f) + DC_LAG_S + DC_PRE_MARGIN + (D.ph > 0 ? DC_PRE_HYS : 0);
        int expect = in.pre && share > DC_PRE_MIN && in.eta == in.eta && in.eta < near_s;
        if (!expect) D.pb_lock = 0;
        float goal = expect && !D.pb_lock ? cap : 0;
        if (goal > 0 && D.ph >= cap - 1e-3f) {
            D.pb_s += dt;
            if (D.pb_s > DC_PRE_HOLD_S) { D.pb_lock = 1; goal = 0; logv(2, "Decel: no shift after the hand-over was ready, given up"); }
        } else if (goal <= 0) D.pb_s = 0;
        float st = fminf(fmaxf(goal - D.ph, -fmaxf(p_jerk, 0.1f) * dt), fmaxf(p_jerk, 0.1f) * dt);
        if (st != 0) {
            float l2 = fminf(fmaxf(lvl + D.k * st, 0), p_max);
            D.ph = fmaxf(D.ph + (l2 - lvl) / D.k, 0);
            lvl = l2;
        }
        if (in.ramp) D.pbs += ((share > 1e-3f ? fminf(D.ph / share, 1.0f) : 0) - D.pbs) * dt / (DC_LAG_S + dt);
        else D.pbs = 0;
    }
    if (in.soften) {
        // soften: lowered only while over the target with none of our brake on, raised while short (out of the transients)
        if (!D.gap && D.blind <= 0 && D.ph <= 0 && D.pbs < 0.01f) {
            float over = am - D.tgt, r = fmaxf(am / D.kr, 0.5f), dk = 0;   // r: the deceleration at 1 (the drag in it too: a lower gain, never a higher one)
            if (over > DC_SOFT_DB && D.u < DC_SOFT_U && (D.kr < 1 || over > DC_SOFT_TOL)) dk = -DC_SOFT_KI * (over - DC_SOFT_DB) / r;
            else if (over < -DC_SOFT_DB && D.kr < 1) dk = DC_SOFT_KI * (-over - DC_SOFT_DB) / r;
            D.kr = fminf(fmaxf(D.kr + fminf(fmaxf(dk, -DC_SOFT_DN), DC_SOFT_UP) * dt, DC_SOFT_MIN), 1.0f);
        }
        if (D.gap) D.ks = fminf(D.ks + (1 - D.kr) / DC_SOFT_GAP_UP * dt, 1.0f);
        else D.ks = D.kr >= D.ks ? D.kr : fmaxf(D.kr, D.ks - (1 - D.kr) / DC_SOFT_GAP_DN * dt);
    }
    float kf = in.soften ? D.ks : 1.0f;
    int ramp_on = D.rt >= 0;   // (the tick that ends the ramp still posts the request unslewed)
    if (D.rg) kf = D.kg * fminf(fmaxf(1 - (D.gap_s - DC_LAG_S) / DC_FF_S, 0), 1.0f);   // a ramp's gap: off as the brake's feedforward arrives
    else if (D.rt >= 0) {   // the ramp: the retarder takes over what the brake gives up, as the brake model has it (the request leads by the lag, below)
        float span = D.ry0 - D.rye;
        // (the hand-over only goes forward: the loop may take the brake up again meanwhile, that is not the retarder going back; and
        // it ends at the latest a few lags after the level did, whatever the brake model does)
        D.rs = fmaxf(D.rs, span > 0.01f ? fminf(fmaxf((D.ry0 - D.y) / span, 0), 1.0f) : 1.0f);
        if ((D.rs >= 0.97f && D.rt >= D.rT) || D.rt >= D.rT + DC_RAMP_TAIL) D.rt = -1;
        else kf = D.r0 + (kf - D.r0) * D.rs;
    } else kf *= 1.0f - D.pbs;
    if (in.soften || in.ramp) soft_post(kf);
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
            D.k = D.k_fix ? 0.5f * (D.k + kn) : kn; D.k_known = D.k_fix = 1; D.k_prior = 0;
        }
    }
    // a falling reference (the taper to a stop, a lower step) takes its share of the level with it, and in a gap (the brake
    // alone holds the deceleration there) a rising one too: the integral alone would lag the ramp
    if (D.ref < ref0 || D.gap) lvl = fminf(fmaxf(lvl + (D.ref - ref0) * D.k, 0), p_max);
    float di = fminf(fmaxf(DC_KI * e, -dn), starved ? 0 : up) * dt;
    if (!((lvl >= p_max && di > 0) || (lvl <= 0 && di < 0))) lvl += di;
    D.lvl = fminf(fmaxf(lvl, 0), p_max);
    // in the ramp the request leads the level by the brake's lag, so the deceleration the brake gives (the request through the lag)
    // falls as the level does and the retarder can follow the level's share at once
    float lead = ramp_on && D.rt >= 0 ? D.rdl / D.rT * DC_LAG_S : 0;
    float want = fminf(fmaxf(D.lvl + DC_KP * e + lead, 0), p_max);
    D.u = step_now || ramp_on ? want : fminf(fmaxf(want, D.u - dn * dt), D.u + up * dt);
    act_brake(ACT_DECEL, D.u);
}

// ---------------------------------------------------------------- Stop assist ([decel] stop_assist)
// Lifting off at low speed, the box opens the clutch at the start gear's idle point and the truck coasts on in neutral (~0.07 m/s2: it
// rolls at walking pace for a long time). The stop assist finishes that stop: the service brake slows the truck at stop_assist_decel
// to a standstill and, with stop_assist 2, Auto Hold takes over there. It shares the controller of the lever assist (the measured
// deceleration, a feedforward from the brake's gain with a PI on the request, the taper along a = sqrt(2 j v)) but is a feature of its
// own: it works with [decel] enabled 0, and it leaves D, the lever assist's state, alone.
// Trigger = the event, no km/h figure: the box opened the clutch by itself with both pedals free (no throttle, no brake, no lever, no
// cruise, not reversing, not paused, not crawling, not eco-roll). With [lowspeed] owner=1 that is the owner's own state (g_low_assist,
// which knows its open point). Without it the assist derives it: the clutch comes open from a gear whose driveline sits at the box's stop
// threshold (idle + 150 rpm; the stock 0.8 idle is lower still), and stays open for longer than a gearbox shift takes (a downshift at
// low revs opens it too, but closes it again within about a second).
// Before the brake comes on, the deceleration of the coast is measured (the grade is in it): the brake only adds what the target needs
// over it. The request is capped (SA_MAX): where the target cannot be reached (a steep descent) the driver is left to it, once logged.
// Ends at once on the throttle, the brake pedal, the lever, the cruise, reverse, a pause, another brake owner, or the clutch closing in a
// gear. At the standstill stop_assist 1 lets go (and Auto Hold is kept from taking the brake over, g_sa_nohold); stop_assist 2 keeps a
// small request until Auto Hold's has risen to it, as the lever's stop does.
static const float SA_MAX = 0.3f;        // most request the stop assist asks (~1.8 m/s2 at the nominal brake: a gentle stop never needs more)
static const float SA_OPEN_RPM = 250;    // rpm over idle (by the engine's span) the last gear's driveline may sit at when the clutch opens: the box's stop threshold plus a margin for the model
static const float SA_SHIFT_PAD = 0.4f;  // s over the shift time the clutch has to stay open before it counts as a stop (not a shift)
static const float SA_STOP_V = 0.03f;    // m/s, standing for the stop assist (the taper reaches 0 at 0.0225)
static const float SA_HAND_REQ = 0.03f;  // request kept at a standstill for Auto Hold to take over (the taper has run out by then)
static const float SA_CAP_S = 1.2f;      // s at the cap and far short of the target: left to the driver
enum { SA_OFF, SA_ARMED, SA_PEND, SA_RUN, SA_HAND };
static struct {
    int st;                 // SA_OFF = not armed (waits for the clutch to be closed in a gear again)
    float hv[64]; double ht[64], clk; int hn, hi;   // the speed samples (as D's)
    float a, bad_s;         // measured deceleration (m/s2, + = slowing), seconds without a usable one
    int lastg; float lastv; // the gear and speed while the clutch was last closed in a gear
    float t_pend;           // s the clutch has been open (derived trigger)
    float a0;               // the coast's deceleration at the start (grade included)
    float ref, refm, lvl, u, k, ff;   // reference, its delayed copy, level, request, request per m/s2, share of the feedforward
    float v0, t_run, tgt0, cap_s, hand_s, lvl_max;
    float lit_s;            // s of the run with the brake lights on (the game decides them; the log says whether ours lit them)
    int mode, posted, capped;
} SA;

static void sa_log(const char *why, float v)
{
    if (SA.st != SA_RUN && SA.st != SA_HAND) return;
    logv(2, "Decel: stop assist %d from %.1f km/h, target %.2f m/s2, coast %.2f, request up to %.2f; mean %.2f m/s2 over %.1f s (brake lights on %.0f %% of it); ended at %.1f km/h: %s",
         SA.mode, SA.v0 * 3.6f, SA.tgt0, SA.a0, SA.lvl_max, SA.t_run > 0 ? (SA.v0 - v) / SA.t_run : 0.0f, SA.t_run,
         SA.t_run > 0 ? 100 * SA.lit_s / SA.t_run : 0.0f, v * 3.6f, why);
}

// To the next state (OFF = disarmed, ARMED = may start at the next opening); our request and the Auto Hold block go with it.
static void sa_to(int st)
{
    if (SA.posted) { act_brake(ACT_DECEL, 0); SA.posted = 0; }
    g_sa_nohold = 0;
    SA.u = SA.lvl = 0; SA.cap_s = SA.hand_s = 0;
    SA.st = st;
}

static void sa_frame(float dt)
{
    int mode = p_sa >= 1.5f ? 2 : p_sa >= 0.5f ? 1 : 0;
    if (mode == 0 || !act_hooked()) { if (SA.st != SA_OFF) sa_to(SA_OFF); return; }
    if (!(dt > 0 && dt < 0.5f)) { if (dt != 0) SA.hn = 0; return; }   // paused or a hitch: the speed history is broken (a request in force stays)
    float v = fabsf(T.speed);
    // the measured deceleration, as the lever assist has it (the speed's fall over the last 0.3 s)
    int ok = 0;
    if (T.speed == T.speed) {
        SA.clk += dt;
        SA.hi = (SA.hi + 1) % 64; SA.hv[SA.hi] = v; SA.ht[SA.hi] = SA.clk;
        if (SA.hn < 64) SA.hn++;
        for (int i = 1; i < SA.hn; i++) {
            int j = (SA.hi - i + 64) % 64;
            float span = (float)(SA.clk - SA.ht[j]);
            if (span >= DC_WIN) { float raw = (SA.hv[j] - v) / span; if (fabsf(raw) < 30.0f) { SA.a = raw; ok = 1; } break; }
        }
    }
    SA.bad_s = ok ? 0 : SA.bad_s + dt;
    const act_wish *w = act_wishes();
    float other = 0;   // another owner's brake (Auto Hold's own is not one: it is the hand-over)
    for (int o = 0; o < ACT_N; o++) if (o != ACT_DECEL && o != ACT_HOLD) other = fmaxf(other, w[o].brake);
    float cl = clutch_seen();
    int closed = cl < DC_CLOSED && T.gear > 0, opened = cl > DC_OPEN || T.gear == 0;
    int manual = T.shifter[0] && strcmp(T.shifter, "automatic") && strcmp(T.shifter, "arcade");
    // the pedals and the rest that must be free
    int free = !(T.in_thr > DC_PEDAL) && !(T.in_brake > DC_PEDAL) && !T.retarder && !T.motor_brake && T.cruise <= 0.1f && !g_crawl_on && T.gear >= 0 &&
               T.dgear >= 0 && T.engine_on && !T.paused && !T.parking_brake && !g_rolling && !(other > 0) && !manual && !V.electric && !g_truck_unsupported && !D.on;
    if (closed) {
        SA.lastg = T.gear; SA.lastv = v;
        sa_log("the clutch closed", v);
        sa_to(SA_ARMED);
        return;
    }
    if (SA.st == SA_ARMED || SA.st == SA_PEND) {
        if (!opened) return;                                          // (a shift's clutch between the two thresholds, or no gear to speak of)
        if (!free) { sa_to(SA_OFF); return; }                         // opened with a pedal, the lever, the cruise ...: not a stop of this kind
        if (!(T.speed > 0)) return;
        int go = 0;
        if (low_owner_on()) go = g_low_assist;                        // the owner knows its open point
        else if (SA.st == SA_ARMED) {
            float idle = ts_idle_rpm(), r = SA.lastg > 0 ? vehicle_rpm_at(SA.lastg, SA.lastv) : 0;
            if (idle > 200 && idle < 2000 && r > 0 && r <= idle + rs_off(idle, SA_OPEN_RPM)) { SA.st = SA_PEND; SA.t_pend = 0; }
        } else {
            SA.t_pend += dt;
            go = SA.t_pend >= fmaxf(V.eng_ok ? V.eng.shift_time : 1.2f, 0.5f) + SA_SHIFT_PAD;
        }
        if (!go || v < SA_STOP_V) return;
        SA.st = SA_RUN; SA.mode = mode; SA.v0 = v; SA.t_run = 0; SA.cap_s = SA.hand_s = 0; SA.capped = 0; SA.lvl_max = 0; SA.lit_s = 0;
        SA.a0 = fminf(fmaxf(SA.a, -2.0f), 0.5f);
        SA.ref = SA.refm = fmaxf(SA.a0, 0);
        SA.k = 1.0f / DC_G_NOM; SA.ff = DC_FF_FIRST;
        if (g_brake_learn > 0) {   // the brake's gain from the vehicle model, once it is confident
            float g = vehicle_brake_gain();
            if (g == g && g >= 2.0f && g <= 12.0f) { SA.k = 1.0f / g; SA.ff = DC_FF_PRIOR; }
        }
        SA.tgt0 = fminf(fmaxf(p_sad, 0.1f), 0.69f);
        float cap0 = fminf(p_max > 0 ? fminf(p_max, 1.0f) : 0.0f, SA_MAX);
        SA.lvl = fminf(SA.k * SA.ff * fmaxf(SA.ref - SA.a0, 0.0f), cap0);
        SA.u = 0;
    }
    if (SA.st != SA_RUN && SA.st != SA_HAND) return;
    // ---- running
    SA.t_run += dt;
    if (T.light_brake) SA.lit_s += dt;
    const char *end = NULL;
    if (!free) end = "pedal, lever, cruise or another brake";
    else if (SA.bad_s > DC_BAD_S) end = "no usable speed";
    else if (T.speed < 0) end = "rolling backwards";
    else if (v > SA.v0 + 0.5f) end = "speeding up";
    if (end) { sa_log(end, v); sa_to(SA_OFF); return; }
    float cap = fminf(p_max > 0 ? fminf(p_max, 1.0f) : 0.0f, SA_MAX);
    float jerk = fmaxf(p_jerk, 0.1f), jt = DC_TAPER_J * jerk, rlim = jerk * SA.k;
    int standing = !(v > SA_STOP_V);
    cruise_info cu;
    cruise_info_get(&cu);
    if (SA.st == SA_RUN && standing) {
        if (SA.mode == 2 && cu.lever_take > 0) { SA.st = SA_HAND; SA.hand_s = 0; g_sa_nohold = 0; }   // Auto Hold arms on the request at a standstill
        else {
            static bool told;
            if (SA.mode == 2 && !told) { told = true; logline("Decel: stop_assist 2 needs Auto Hold with [cruise] auto_hold_lever: the stop is released at the standstill"); }
            sa_log("stopped", v); sa_to(SA_OFF); return;
        }
    }
    if (SA.st == SA_HAND) {   // standing: a small request stays until Auto Hold's has risen to it (or to what it will hold), as the lever's stop does
        SA.hand_s += dt;
        if (SA.hand_s >= DC_HAND_S || cu.lever_take <= 0 || w[ACT_HOLD].brake >= fminf(SA_HAND_REQ, cu.lever_take) - 1e-3f) { sa_log("handed to Auto Hold", v); sa_to(SA_OFF); return; }
        act_brake(ACT_DECEL, SA_HAND_REQ); SA.posted = 1;
        return;
    }
    g_sa_nohold = 1;   // Auto Hold arms on the posted request below 0.5 km/h: not yet (stop_assist 2 lets it at the standstill, stop_assist 1 never)
    // reference: the target, tapered toward the standstill along a = sqrt(2 j v) led by the brake's lag, slewed at the taper's jerk
    float tgt = fminf(SA.tgt0, fmaxf(sqrtf(2.0f * jt * v) - jt * DC_LAG_S, 0.0f));
    float ref0 = SA.ref;
    SA.ref += SA.ref < tgt ? fminf(jt * dt, tgt - SA.ref) : -fminf(jt * dt, SA.ref - tgt);
    SA.refm += dt / (DC_REFM_S + dt) * (SA.ref - SA.refm);
    float e = SA.refm - SA.a;
    int starved = SA.u > 0.02f && act_request() < 0.5f * SA.u;
    // the feedforward moves the level with the reference (the rise and the taper), the PI takes what is left
    SA.lvl = fminf(fmaxf(SA.lvl + (SA.ref - ref0) * SA.k * SA.ff, 0.0f), cap);
    float di = fminf(fmaxf(DC_KI * e, -rlim), starved ? 0.0f : rlim) * dt;
    if (!((SA.lvl >= cap && di > 0) || (SA.lvl <= 0 && di < 0))) SA.lvl += di;
    SA.lvl = fminf(fmaxf(SA.lvl, 0.0f), cap);
    float want = fminf(fmaxf(SA.lvl + DC_KP * e, 0.0f), cap);
    SA.u = fminf(fmaxf(want, SA.u - rlim * dt), SA.u + rlim * dt);
    SA.lvl_max = fmaxf(SA.lvl_max, SA.u);
    // the target out of reach (a steep descent): at the cap and far short of the target for a while = left to the driver
    if (SA.u >= cap - 1e-3f && SA.a < 0.6f * SA.ref) SA.cap_s += dt; else SA.cap_s = 0;
    if (SA.cap_s > SA_CAP_S) {
        static bool told;
        if (!told) { told = true; logline("Decel: stop assist cannot reach %.2f m/s2 here (request capped at %.2f, %.2f m/s2 measured): left to the driver", SA.tgt0, cap, SA.a); }
        sa_log("the target is out of reach", v); sa_to(SA_OFF); return;
    }
    act_brake(ACT_DECEL, SA.u); SA.posted = 1;
}

void decel_view(float *target, float *measured, const char **stop_assist)
{
    static const char *const names[] = { "off", "armed", "pending", "running", "handing over" };
    *target = *measured = NAN;
    if (D.on) { *target = D.tgt; *measured = D.a; }
    else if (SA.st == SA_RUN || SA.st == SA_HAND) { *target = SA.ref; *measured = SA.a; }
    *stop_assist = SA.st >= 0 && SA.st <= SA_HAND ? names[SA.st] : "off";
}

static void de_frame()
{
    static double last;
    double now = sim_s();
    decel_in in;
    in.dt = (float)(now - last);
    last = now;
    sa_frame(in.dt);   // the stop assist: its own feature, with or without the lever assist
    if (p_enabled <= 0 || !act_hooked()) { if (D.on) decel_off(); soft_off(); g_decel_need = 0; return; }
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
    in.ramp = p_ramp > 0 && act_retarder_hooked();
    in.pre = p_pre > 0;
    in.eta = in.pre && D.on ? ts_brake_down_eta(fmaxf(D.a, 0)) : NAN;
    bool was = D.on, soft_was = D.kr < 1;
    decel_tick(in);
    // downshift_assist: the service brake the lever needs besides the retarder, as the gearbox sees it (the level of before a gap while
    // one runs, without what a hand-over ahead of a shift has added)
    float need = 0;
    if (p_dsa > 0 && D.on && D.on_s > DC_NEED_S) { float lv = D.gap || D.rt >= 0 ? D.pre : fmaxf(D.lvl - D.k * D.ph, 0); need = lv > DC_NEED ? lv : 0; }
    g_decel_need = need;
    if (D.on != was) logv(1, "Decel: %s (step %d of %d, target %.2f m/s2, %.0f km/h)", D.on ? "on" : "off", D.step, D.n, D.tgt, fabsf(T.speed) * 3.6f);
    if ((D.kr < 1) != soft_was) logv(1, "Decel: retarder %s (step %d of %d, target %.2f m/s2, measured %.2f, %.0f km/h)", D.kr < 1 ? "softened" : "back to the game's own", D.step, D.n, D.tgt, D.a, fabsf(T.speed) * 3.6f);
}

// Continuity broke (pause, hitch, other truck, a new load): no state carries over. The learned request per m/s2 depends on the load.
static void de_reset(int level, const char *)
{
    D.on = 0; D.lvl = D.u = D.pre = 0; D.gap = 0; D.gap_s = 0; D.blind = 0; D.bad_s = 0; D.hn = 0; D.a = 0; D.y = D.ya = 0;
    sa_to(SA_OFF); SA.hn = 0; SA.a = 0;
    if (level >= RESET_LOAD) { D.k_known = D.k_fix = D.k_prior = 0; D.k = 1.0f / DC_G_NOM; }
    act_brake(ACT_DECEL, 0);
    soft_off();
    g_decel_need = 0;
}

static char g_status[240] = "off (ini)";
static const char *de_status() { return g_status; }

static void de_start()
{
    int sa = p_sa >= 1.5f ? 2 : p_sa >= 0.5f ? 1 : 0;
    if (p_enabled <= 0) {
        if (sa) snprintf(g_status, sizeof g_status, "lever assist off; stop assist %d: %.2f m/s2%s", sa, fminf(fmaxf(p_sad, 0.1f), 0.69f), act_hooked() ? "" : " (no actuator hook)");
        return;
    }
    p_max = p_max > 0 ? fminf(p_max, 1.0f) : 0;   // a request is 0..1 (and a NaN from the ini is none)
    if (!(p_jerk >= 0.1f)) p_jerk = 0.1f;         // the slews divide and multiply by it
    snprintf(g_status, sizeof g_status, "on: %.1f-%.1f m/s2, jerk %.1f m/s3, max brake %.2f%s%s%s%s%s", p_tmin, p_tmax, p_jerk, p_max,
             p_soften > 0 && g_retarder_hook >= 0.5f ? ", soften" : "", p_ramp > 0 && g_retarder_hook >= 0.5f ? ", ramp" : "",
             p_pre > 0 ? ", pre-blend" : "", p_dsa > 0 ? ", downshift assist" : "", act_hooked() ? "" : " (no actuator hook)");
    if (sa) { size_t l = strlen(g_status); snprintf(g_status + l, sizeof g_status - l, "; stop assist %d: %.2f m/s2", sa, fminf(fmaxf(p_sad, 0.1f), 0.69f)); }
    static bool told;
    if (p_soften > 0 && g_retarder_hook < 0.5f && !told) { told = true; logline("Decel: soften is on but [core] retarder_hook is 0: the game's retarder stays as it is, the service brake only adds"); }
    static bool told_ramp;
    if (p_ramp > 0 && g_retarder_hook < 0.5f && !told_ramp) { told_ramp = true; logline("Decel: ramp_s is on but [core] retarder_hook is 0: after a shift the retarder comes back as the game has it, the brake goes back at once"); }
}

static void de_stop() { act_brake(ACT_DECEL, 0); soft_off(); g_decel_need = 0; sa_to(SA_OFF); }

module m_decel = { "decel", "Decel", g_params, sizeof g_params / sizeof g_params[0], de_start, de_frame, NULL, de_stop, de_reset, de_status };
