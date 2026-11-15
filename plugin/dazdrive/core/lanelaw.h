// Lane's decide layer (WP-LA3): the law of the lane assist (mode 2: offset, heading, bend feed-forward, feedback on the assist state A), the steering learner and its
// calibration, the sway detector, the live wheel gain, the actuator-gain estimator, the soft start and the lateral-load yield. It reads its inputs from its arguments, the
// telemetry T, the route's straight stretch and one struct the hook fills from the game (LE below); it writes nothing to the game: the hook (modules/lane.cpp stub_law) turns
// the result into the game's control output through act_lka_u. No offset, no game_* reader, no sense_* call: the sim (tools/sim/sim_lane.cpp) includes the module and drives
// lane_step / cal_tick / stub_law directly. The parameters (p_*) are the module's: its table binds them to the ini; the module declares them before this file comes in.
// What the file persists goes through the module's prof_save (steer_profile.txt).
#pragma once
#include "suite.h"
#include "route.h"
#include "vclass.h"
#include "roll.h"
#include "latload.h"
#include <math.h>
#include <string.h>

// What the law needs from the game that is not an argument: the build (a 1.61 layout: the calibration may set the scale above learn_max there) and the lights byte (an
// indicator stops a running calibration). The hook sets them on every call; the module's start sets the build; the sim sets them where it switches the layout.
struct lane_env { int is161; unsigned char lights; };
static lane_env LE;
static void prof_save(const char *why);   // the module's: the profile file (steer_profile.txt)

typedef void (*law_fn)(void *ga, float o1, float o2, float o3, float dt);
static float g_u;
// driver-in-the-loop and centering state (reset when the assist is off)
static double g_soft_at = -1, g_soft_end = -1, g_centre_since = -1;   // soft start state (below), declared here for the runaway guard
static double g_panic_until, g_sat_t[3]; static int g_sat_dir[3];   // runaway guard: the last two ceiling hits (time, direction), and when the 4 s release ends
static double g_sat_since;   // review 30-10: when the request first saturates one-sided (0 = not now), for the sustained one-sided runaway
static float g_bias;   // lane_bias_m as it is now (m, negative = right of the lane center: o1 is + to the left), eased in and out
static float g_share;   // bend_blend: the share of the bend steering that is yours (0-1), filtered
static float g_ref, g_act_t = 1e3f, g_steady_t, g_dev, g_pm, g_hold_t, g_age;   // shared steering: the aim (m from the lane center, 0 = center), seconds since you last steered, seconds the offset has been steady
static float g_do1, g_act, g_scale = 1, g_w_prev, g_w_rate, g_ci, g_o2_prev, g_o1_prev, g_d2, g_damp_u;
static int g_have_w, g_have_o, g_engaged;
static float g_ff;  // last bend feed-forward request (mode 2, log)
static float g_o3_prev, g_do3; static int g_have_o3;
static float g_lf = 1, g_li = 1; static int g_hold;   // last live scales and bend-hold state (lane.csv)
static double g_law_at = -1e9;  // last call of the control law (LKA active)
static int g_lane_state = LANE_OFF;   // LaneState (core/suite.h): what the assist is doing now, set where the gates below decide
static double g_stub_at = -1e9;       // last call of stub_law: the game calls the law only while lane keeping is on

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// Every change of a learned value is announced (P7, WP-LA6): the scale of the gains s, the extra reduction for a slow wheel dfac, the ring ceiling cap, the actuator gain g_ag and the
// session factor L_sess. LEARN_LOG is logv plus the bits of the values the line announces; sim_lane reads and clears the mask on every plant step and fails a change of a value
// that no line announced (learn_track).
enum { LV_S = 1, LV_DFAC = 2, LV_CAP = 4, LV_AG = 8, LV_SESS = 16 };
static unsigned g_lv_mask;
#define LEARN_LOG(lvl, bits, ...) do { g_lv_mask |= (bits); logv(lvl, __VA_ARGS__); } while (0)

// ---------------------------------------------------------------------------------------------------------------
// Roundabouts and fast bends, from the live lateral load (WP-LA5; it used to come from the cruise cancelling for a roundabout it knew from the route, g_rb_cancel_at).
// 25-10, a prefab 148 m ahead was no known junction or roundabout: the lane reference jumped (o1 +0.13 -> -1.09 m in 2 s), the request went to the ceiling with the
// game's own, the runaway guard handed the wheel to the game's law (also at the ceiling) and the driver steered in at 7.2 m/s2 at 52 km/h. Damien: "the lane assist should
// let go in every bend with 7 m/s2". The assist works inside the envelope the bends are planned in; over it, with the driver's hand on the wheel, the assist yields.
// The load is the telemetry's (speed x yaw rate, and the steering that leads it: core/latload.h, the same reading the gearbox's bend bonus makes), never the route.
// The limit is the sideways acceleration where the lean reaches the budget: the rollover guard's, ROLL_GUARD_REF x its learned scale (core/roll.h), so the assist lets
// go where the guard starts to act (a vehicle that leans more has a smaller limit and both come sooner). The weight is whole up to roll_edge_share of it (0.7: the bend
// allowance) and nothing at the limit. It applies only while a hand holds the wheel (lat_hand below): hands off the assist is whole at any load as before, since without a hand on the
// wheel the game's own law alone cannot hold a bend and the truck leaves it (the 24-10 sim: 12-38 m; this one: 2000 m); the rollover guard brakes there, the runaway
// guard catches a runaway of the request.
// lat_frame runs every frame (ln_frame), the weight is read by the law (stub_law, the game's thread).
static lat_state g_lat;
static lat_read g_lat_r = { 0, 0, 0 };
static float g_latw = 1;               // the weight from the lateral load alone, 0..1 (whole hands off)
static int g_lat_off;                  // yielding: from an effective weight under 0.5 until it is back over 0.9 (hysteresis, one tone each way)
static double g_lat_w_at = -1e9;
static const float LAT_RESUME_S = 1.0f;   // the weight comes back in 1 s: the ramp after an indicator pause and after the low-speed gate (g_resume_at), one resume feel
static float lat_limit() { const float k = g_roll_guard_k; return ROLL_GUARD_REF * (k >= ROLL_GUARD_KMIN && k <= 1.0f ? k : 1.0f); }
// The share of the driver's hand on the wheel, 0 .. 1: a HOLD, not a movement. A steady hold has no wheel speed (through a roundabout the wheel sits still at 0.15), and the
// wheel speed beyond what the assist explains (g_act) is a poor witness anyway: at the entrance of a bend with the hands off it reads 0.6 .. 1.0 for a second (the sim, ATS 1.61:
// the wheel gain and the actuator gain are still being measured), exactly when a wrong "hand" would take the assist away. So the reading is the wheel against where the assist alone
// puts it (g_wpred: the live wheel gain x A 0.33 s ago), 0.5 s filtered (g_hdev, the filter shared steering uses): the part of the wheel the assist does not explain. Hands off the
// assist explains the wheel and the part is small (the sim: 0.02 on an ETS2 wheel, up to 0.52 on the ATS truck before its actuator gain is known); with a hand it is most of the
// wheel (0.9 after 2 s, 1.0 after 3 s). From 70 % of the wheel (the assist may be wrong by 30 % and still be no hand; the highest hands-off reading above is 52 %) to 95 %, on a wheel
// of at least 0.03 (under it nothing steers visibly: hand tremor, the centering band of the wheel). Hands off the weight is exactly 1.
static float g_hdev, g_hwheel;       // the wheel not explained by the assist (0.5 s filter) and the wheel, for lat_hand
static float lat_hand()
{
    return fabsf(g_hwheel) >= 0.03f ? clampf((fabsf(g_hdev) / fabsf(g_hwheel) - 0.7f) / 0.25f, 0, 1) : 0.0f;
}
static float lat_eff() { return 1 - lat_hand() * (1 - g_latw); }   // the weight the law applies: whole hands off, the load's weight with a hand on the wheel
static void lat_frame()
{
    const double now = wall_s();
    g_lat_r = lat_step(g_lat, fabsf(T.speed), T.yaw, T.in_steer, now);
    const float tgt = p_rb_off > 0 && p_mode >= 2 ? lat_weight(g_lat_r.accel, lat_limit(), g_roll_edge) : 1.0f;
    double dt = now - g_lat_w_at;
    g_lat_w_at = now;
    if (!(dt > 0 && dt < 0.5)) dt = 0.016;
    if (tgt < g_latw) g_latw = tgt;                                          // down at once
    else g_latw = fminf(tgt, g_latw + (float)dt / LAT_RESUME_S);            // up in LAT_RESUME_S
    const float w = lat_eff();
    const bool on = now - g_law_at < 0.3;                                    // our law is steering: only then a tone and a letter
    if (!g_lat_off && w < 0.5f) {
        g_lat_off = 1;
        if (on) { if (p_tone > 0) play_cue(CUE_LANE_PAUSE); disp_post(DISP_LANE, DISP_FLASH, 'X', 1.5); }
        if (on) logv(1, "Lane: yields to your hand for the sideways load: %.1f m/s2 = %.2f x the %.1f m/s2 limit (measured %.1f, the steering predicts %.1f), %.0f km/h, you steer %.2f, weight %.2f",
                     g_lat_r.accel, g_lat_r.accel / lat_limit(), lat_limit(), g_lat_r.meas, g_lat_r.lead, fabsf(T.speed) * 3.6f, lat_hand(), w);   // (no line when the assist is not steering: a fast roundabout by hand)
    } else if (g_lat_off && w >= 0.9f) {
        g_lat_off = 0;
        if (on) { if (p_tone > 0) play_cue(CUE_LANE_RESUME); disp_post(DISP_LANE, DISP_FLASH, 'L', 1.5); }
        if (on) logv(1, "Lane: the sideways load is down to %.1f m/s2 (%.2f x the limit) or your hands are off: the assist is back (weight %.2f)", g_lat_r.accel, g_lat_r.accel / lat_limit(), w);
    }
}

static float g_wg, g_wi;            // live wheel gain: wheel / A 0.33 s earlier, and its incremental slope (wheel_gain_ref until measured)
static float g_wpred = NAN;         // where the assist alone puts the wheel now (g_wg x A 0.33 s earlier, NAN = not known)
static float g_wexp_rate;           // d/dt of A 0.33 s earlier: the direction the assist moves the wheel in now
static float g_pipe;                // wheel motion already on its way: g_wi x (mean A of the last 0.33 s - A 0.33 s ago)

// How much the driver is steering (0..1): wheel speed beyond what the assist
// itself explains (veh+0x1408 differentiated). Fast attack, slow release.
static void driver_activity(float wheel, float A, float dt)
{
    float in = 0;
    if (wheel == wheel) {
        if (g_have_w) {
            float dw = (wheel - g_w_prev) / dt;
            g_w_rate += (dw - g_w_rate) * clampf(dt / 0.1f, 0, 1);
        } else g_w_rate = 0;
        g_have_w = 1;
        g_w_prev = wheel;
        float dead = p_drv_rate / 3, span = p_drv_rate - dead;
        // the assist moves the wheel too: the way A moved 0.33 s ago, up to 3.5x as fast (the steepest wheel seen: 2.95 in the ts5b
        // crash, whose own fast unwinding was read as the driver, 0.74-0.99, and driver_leak then let A go). Motion against that
        // direction, or faster than that, is yours. No gain estimate in it (a wrong one made a sway look like hands on the wheel).
        // Without the A history the old rule: along with A up to driver_follow x |A|, against it always yours.
        float r;
        if (g_wpred == g_wpred) r = g_w_rate * g_wexp_rate >= 0 ? fabsf(g_w_rate) - fmaxf(3.5f, 1.15f * fmaxf(g_wg, g_wi)) * fabsf(g_wexp_rate) : fabsf(g_w_rate);   // (3.5 x, or 15% above the wheel gain measured live: ATS 1.61 reads ~3.4 per model A, where 3.5 x made the assist's own wheel motion look like hands)
        else { r = fabsf(g_w_rate); if (g_w_rate * A > 0) r -= p_drv_follow * fabsf(A); }
        in = clampf((r - dead) / span, 0, 1);
    }
    float tau = in > g_act ? p_drv_attack : p_drv_release;
    g_act += (in - g_act) * clampf(dt / (tau > 0.01f ? tau : 0.01f), 0, 1);
    g_scale = 1 - g_act * (1 - clampf(p_backoff, 0, 1));
}

// Actuator model fitted on runs/ts3*, ts4* (lka-sysid notes): A' = -al*A + be*u.
// the speed band the fitted actuator model (and so the learner and the calibration test) is valid in
static const float MODEL_KMH_LO = 30, MODEL_KMH_HI = 100, LEARN_MIN_KMH = 50;
// review 30-10: the fitted wheel a bend needs per unit of o3 (lane.cpp + sim_lane.cpp; the blend/hold used 4.6, the feed-forward 4.4)
static const float BEND_NEED = 4.4f;
static float model_be(float kmh) { return 0.235f - 0.0010f * clampf(kmh, MODEL_KMH_LO, MODEL_KMH_HI); }
static float model_al(float kmh) { return 2.6f + 0.016f * clampf(kmh, MODEL_KMH_LO, MODEL_KMH_HI); }

// ---------------------------------------------------------------------------------------------------------------
// Actuator gain (02-11, from the first ATS rides). The game integrates our request u into the assist state A (A' = -al A + be u);
// model_be/model_al above are the ETS2 1.60 fit (A/u at rest 0.048 at 60 km/h). The law works in those units: it asks the model for the A
// it wants, feeds back on A (feedback_gain 40) and measures the wheel per A. In ATS 1.61 the same u builds only ~0.15 of that A (lane.csv:
// A 0.0024 for u 0.3-0.6 where ETS2 gives 0.0167; the steering-input code is identical in both exes, so it is the truck's data) and the
// wheel moves ~11x more per A: the loop from u to the heading is about the same, but every A term of the law is off by a factor. So A is
// read in model units: An = A / g_ag, g_ag = the measured A per model A. The estimator follows the model's A (integrated from the request
// that was in force) and regresses the game's A on it. ETS2 reads 0.85-1.0 (our own leak on A included) and snaps to exactly 1, so
// nothing changes there; the stored profile keeps the last value as the starting guess for the next session.
static float g_ag = 1;                                  // A per model A (1 = as fitted)
static float AG_ann = 1;                                // the value of g_ag a line last announced
static double AG_aa, AG_hh, AG_w, AG_t = -1e9;          // forgetting sums of A*Ah, Ah^2 and the accepted time; the last call
static float AG_ah;                                     // the model's A from the requests so far (model units)
static int AG_known, AG_snapped = 1, AG_dirty;   // AG_dirty: the profile file should be written again (the value moved)
static const float AG_SNAP_IN = 0.6f, AG_SNAP_OUT = 0.4f, AG_SNAP_HI = 1.7f, AG_SNAP_HI_OUT = 2.5f;   // snapped to 1 within 0.6..1.7; leaves it below 0.4 or above 2.5
static const float AG_MIN = 0.03f, AG_MAX = 30;
static float ag_target(float r)
{
    r = clampf(r, AG_MIN, AG_MAX);
    if (AG_snapped) { if (r < AG_SNAP_OUT || r > AG_SNAP_HI_OUT) AG_snapped = 0; }
    else if (r >= AG_SNAP_IN && r <= AG_SNAP_HI) AG_snapped = 1;
    return AG_snapped ? 1.0f : r;
}
// The estimator only means something while the game's A follows our requests. It does not while the wheel is let go (the cruise off with needs_cruise: filter_a
// leaks A to 0 while the law still computes requests; an indicator pause; too slow): the regression then reads "the game builds nothing" and the gain slid 0.999 -> 0.2
// in 45 s on the 11-11 ride (t 1228-1270), the law amplified every A term by 1/g_ag and the second runaway followed (t 1275). So it learns nothing from those
// moments and for 3 time constants of the A model (1/al, ~0.3 s at 90 km/h: ~1 s) after them, until A has built up again from our own requests.
static double AG_hold_until = -1e9;
static void ag_freeze(double now, float kmh) { AG_hold_until = now + 3.0f / model_al(kmh); AG_t = -1e9; }
// one law call: u_prev = the request that was in force since the last call, A the game's A now (raw), dt the time between the calls
static void ag_step(float u_prev, float A, float dt, float kmh, double now)
{
    if (now < AG_hold_until) { AG_t = -1e9; return; }
    if (!(A == A) || !(dt > 0 && dt < 0.2f)) { AG_t = -1e9; return; }
    if (now - AG_t > 0.3) AG_ah = A / g_ag;   // the model's A starts from what the game has (a new switch-on, a pause)
    AG_t = now;
    float be = model_be(kmh), al = model_al(kmh);
    AG_ah += (be / al * u_prev - AG_ah) * (1 - expf(-al * dt));
    if (!(kmh >= MODEL_KMH_LO && kmh <= MODEL_KMH_HI) || fabsf(u_prev) < 0.01f || fabsf(AG_ah) < 0.0006f) return;   // (small signals count little: the sums are weighted by the model's A squared)
    double lam = exp(-dt / 40.0);
    AG_aa = lam * AG_aa + (double)A * AG_ah * dt; AG_hh = lam * AG_hh + (double)AG_ah * AG_ah * dt; AG_w = lam * AG_w + dt;
    if (AG_w < 4 || AG_hh < 8e-6) return;
    float tgt = ag_target((float)(AG_aa / AG_hh));
    if (tgt == g_ag) { AG_known = 1; return; }
    float old = g_ag;
    if (!AG_known) g_ag = tgt;                                                                  // the first reading: take it
    else g_ag = expf(logf(g_ag) + (logf(tgt) - logf(g_ag)) * clampf(dt / 8, 0, 1));            // later ones: slowly, in the log domain
    if (fabsf(g_ag / old - 1) > 0.05f || (!AG_known && g_ag != 1.0f)) AG_dirty = 1;   // (ETS2 reads 1: nothing new to store)
    if (!AG_known || (old == 1.0f) != (g_ag == 1.0f)) {
        g_lv_mask |= LV_AG; AG_ann = g_ag;
        logline("Lane: the game builds %.2f x the modelled assist offset per request%s", AG_aa / AG_hh, g_ag == 1.0f ? " (as fitted on ETS2: nothing to correct)" : ": the law reads the assist offset in model units from now on");
    } else if (fabsf(g_ag / AG_ann - 1) > 0.05f) {   // later moves: a line whenever it has moved 5 % from the last announced value
        LEARN_LOG(2, LV_AG, "Lane learn: the actuator gain moves %.3f -> %.3f (the game builds %.2f x the modelled assist offset per request)", AG_ann, g_ag, AG_aa / AG_hh);
        AG_ann = g_ag;
    }
    AG_known = 1;
}


// ---------------------------------------------------------------------------------------------------------------
// Self-learning steering profile (lka-learn notes). Identifies the wheel loop online on usable stretches only
// (hands off, fast enough, near-straight, steady) and scales the mode-2 gains to keep the loop gain at the hand-tuned
// ts4n value: wheel = gA x A(t - delay), heading rate = -c v wheel. The reference (REF_*) is what THIS estimator reads when it replays
// the ts4n hands-free ride (gA 0.98, delay 0.35 s, c 0.168: the calm small-signal values, lower than the 1.34/0.2/0.155 large-signal fit
// because the FFB wheel has a dead band), so the hand-tuned gains stay as they are on that wheel.
// Safety: bounded scales, a rollback when the score gets worse, a ring guard, a fallback to the fixed gains.
static void sway_reset();
static int sway_active();
static void learn_calibrated_check();
static void cal_status(int on);
enum { CAL_IDLE, CAL_ARMED, CAL_RUN };   // the steering calibration state (declared here: learn_step and learn_update gate on it)
static int g_cal;
enum { LK_NK = 19, LK_RING = 160, LK_ZC = 8 };  // delays 0..0.9 s (pre.6 ride: pinned at the old 0.5 s end for 95% of the time)
static const float REF_GA = 0.98f, REF_C = 0.168f, REF_DELAY = 0.35f, LK_D = 0.78f, LK_TICK = 0.05f;
// the wheel gain per (model) A a fit may read, relative to REF_GA: ETS2 wheels read 0.6-2.4. An ATS 1.61 truck reads ~3.4 (its wheel moves ~11x more per A, A in model units
// ~0.15x: the A gain is divided out first, the wheel is not), so the old 2.5 rejected every fit there and nothing was ever learned.
static const float GA_RATIO_MAX = 5.0f;
struct lprof { float s, dfac, cap; };            // scale on all gains, extra offset/heading reduction for slow wheels, ring ceiling
static lprof L_cur = { 1, 1, 2 }, L_base = { 1, 1, 2 };
static float L_sess = 1, L_sess_ann = 1; static double L_sess_at = -1e9;   // 30-10: session-only sway reduction (not stored), and when it was last lowered
static double L_t, L_warm, L_quiet, L_acc, L_ring_cool;
static int L_init, L_trial, L_hold, L_usable, L_nupd, L_dirty, L_loaded, L_rolled;
static float L_Am, L_wm, L_lp2, L_lpw, L_lpo, L_mx, L_my, L_lpo3, L_xa2;
static double L_Sxx[LK_NK], L_Sxy[LK_NK], L_Syy, L_Nw, L_Cxx, L_Cxy, L_Cyy;
// one block (5 s of contiguous usable driving) is checked on its own before it joins the sums: a block where the wheel does not follow A
// (the driver is in the loop) or that disagrees with what we know is thrown away (anti-sabotage)
static double B_Sxx[LK_NK], B_Sxy[LK_NK], B_Syy, B_Cxx, B_Cxy, B_Cyy; static int B_n, L_dev_blocks, L_blocks_ok, L_blocks_bad;
static const int LK_BLOCK = 100;
static double L_rt[LK_RING]; static float L_rx[LK_RING]; static int L_rn;
static float L_gA = REF_GA, L_c = REF_C, L_dl = REF_DELAY, L_r2a, L_r2c;   // latest estimates (log, file)
static double L_since, L_scn, L_scs, L_sc_zc; static int L_zc_n, L_zc_sign;
static double L_b_rms, L_b_zc; static int L_b_valid;
static int L_ar_sign; static double L_ar_t[LK_ZC];
static char L_key[48] = "";
static double L_save_at;
static int L_cal_ok;                     // accepted fits in this calibration (the confidence measure)
static double L_cal_g_sum, L_cal_g_sum2; // sum / sum of squares of the accepted loop-gain ratio, for the spread

// A(t - d) from the ring (linear interpolation); returns 0 when the ring does not reach back far enough
static int ring_at(double tq, float *out)
{
    int n = L_rn < LK_RING ? L_rn : LK_RING;
    for (int i = 1; i < n; i++) {
        int j1 = (L_rn - i) % LK_RING, j0 = (L_rn - i - 1 + 2 * LK_RING) % LK_RING;
        if (L_rt[j0] <= tq && tq <= L_rt[j1]) {
            double d = L_rt[j1] - L_rt[j0];
            *out = d > 1e-6 ? L_rx[j0] + (float)((tq - L_rt[j0]) / d) * (L_rx[j1] - L_rx[j0]) : L_rx[j1];
            return 1;
        }
    }
    return 0;
}

static void learn_block_reset();

// short-term state only (continuity broke / LKA off): the sums and the profile are plant properties and stay
static void learn_flush()
{
    sway_reset(); learn_block_reset(); L_init = 0; L_rn = 0; L_trial = L_trial; L_xa2 = 0; L_ar_sign = 0; for (auto &t : L_ar_t) t = -1e9; L_zc_sign = 0; L_zc_n = 0;
}

// A scale above learn_max (02-11; 1.61 games only, the 1.60 sims keep the old ceiling). The 24-10 rollover was a scale of 1.27 read from a small signal on
// a wheel that answers 3x stronger to large ones. So it is only ever set by a calibration on a straight stretch, it stands only while the wheel gain
// measured live (g_wi, wheel per A in model units, large signals included) stays within 25% of the one it was measured for, and a ring, a sway or a
// runaway take it back to learn_max (s_back_to_max).
static float s_stored_max() { return LE.is161 ? fmaxf(p_l_max, p_l_cal_max) : p_l_max; }   // what a stored profile may hold
static float s_new_max() { return g_cal != CAL_IDLE ? s_stored_max() : p_l_max; }               // what a fit may set now: only inside an armed calibration
static int learn_small_signal();
static float scale_s()
{
    if (!(p_learn > 0 && p_mode >= 2)) return 1;
    float s = L_cur.s;
    if (s > p_l_max && LE.is161 && g_wi > 1.25f * L_gA) s = fmaxf(p_l_max, s * 1.25f * L_gA / g_wi);
    return s * L_sess;
}
static void s_back_to_max(const char *why)
{
    if (!(L_cur.s > p_l_max) || !LE.is161) return;
    LEARN_LOG(0, LV_S | LV_CAP, "Lane learn: %s with the scale at %.2f: back to %.2f, the calibration has to be run again to set more", why, L_cur.s, p_l_max);
    L_cur.s = L_cur.cap = p_l_max; L_base = L_cur; L_trial = 0; L_dirty = 1;
}
// The wheel gain the law ends up with. The live scaling (lf / li = wref / g_wg, never below 0.4) already divides every A term by the wheel gain
// above wref, so for a plant outside the ETS2 fit (the actuator gain read as not 1: ATS 1.61) the learner must not take that reduction a second time:
// 22x the A gain made it want s 0.5, which the sim shows worsens the lane keeping there (rms 0.12 -> 0.22 m). ETS2 keeps the old reading.
static float gA_eff(float gA) { return g_ag != 1.0f && p_wref > 0 ? fminf(gA, fmaxf(p_wref, 0.4f * gA)) : gA; }
static float scale_df() { return p_learn > 0 && p_mode >= 2 ? L_cur.dfac : 1; }


// estimates from the sums; false when the data is not good enough to act on
static int learn_estimate()
{
    if (L_Nw * LK_TICK < 20) return 0;
    int best = -1; double br = 1e30;
    for (int k = 0; k < LK_NK; k++) {
        if (L_Sxx[k] < 1e-12) continue;
        double res = L_Syy - L_Sxy[k] * L_Sxy[k] / L_Sxx[k];
        if (res < br) { br = res; best = k; }
    }
    if (best < 0 || L_Syy < 1e-12 || L_Cxx < 1e-12 || L_Cyy < 1e-12) return 0;
    double dl = best * 0.05;
    if (best > 0 && best < LK_NK - 1) {   // parabola through the neighbors
        double r0 = L_Syy - L_Sxy[best - 1] * L_Sxy[best - 1] / L_Sxx[best - 1], r2 = L_Syy - L_Sxy[best + 1] * L_Sxy[best + 1] / L_Sxx[best + 1];
        double den = r0 - 2 * br + r2;
        if (den > 1e-15) dl += 0.05 * clampf((float)(0.5 * (r0 - r2) / den), -0.5f, 0.5f);
    }
    double rms = sqrt(L_Sxx[best] / L_Nw);
    L_gA = (float)(L_Sxy[best] / L_Sxx[best]);
    L_dl = (float)dl;
    L_r2a = (float)(1 - br / L_Syy);
    L_c = (float)(L_Cxy / L_Cxx);
    L_r2c = (float)(L_Cxy * L_Cxy / (L_Cxx * L_Cyy));
    return rms >= 0.0015 && L_r2a >= 0.4f && L_r2c >= 0.08f && L_gA / REF_GA >= 0.4f && L_gA / REF_GA <= GA_RATIO_MAX && L_c / REF_C >= 0.4f && L_c / REF_C <= 2.5f;
}

static void learn_update()
{
    double dur = L_scn * LK_TICK;
    int sc_ok = dur >= 25;
    double rms = sc_ok ? sqrt(L_scs / L_scn) : 0, zc = sc_ok ? L_sc_zc * 60.0 / dur : 0;
    int ok = learn_estimate();
    L_nupd++;
    logv(2, "Lane learn: gA %.2f delay %.2f c %.3f (R2 %.2f/%.2f)%s | score %.2f m, %.1f zc/min | s %.2f dfac %.2f cap %.2f", L_gA, L_dl, L_c, L_r2a, L_r2c, ok ? "" : " [rejected]", rms, zc, L_cur.s, L_cur.dfac, L_cur.cap);
    if (L_trial) {   // the last profile has been driven: not worse than the one before?
        L_trial = 0;
        if (sc_ok && L_b_valid && (rms > 1.25 * L_b_rms + 0.02 || zc > 1.5 * L_b_zc + 3)) {
            LEARN_LOG(1, LV_S | LV_DFAC | LV_CAP, "Lane learn: profile s %.2f/%.2f scored worse (%.2f m, %.1f zc/min against %.2f, %.1f), rolled back to %.2f/%.2f", L_cur.s, L_cur.dfac, rms, zc, L_b_rms, L_b_zc, L_base.s, L_base.dfac);
            L_cur = L_base; L_hold = 4; L_dirty = 1; L_rolled++;
        } else { L_base = L_cur; L_b_rms = rms; L_b_zc = zc; L_b_valid = sc_ok; }
    } else if (sc_ok) { L_b_rms = rms; L_b_zc = zc; L_b_valid = 1; }
    L_scn = L_scs = L_sc_zc = 0; L_zc_n = 0; L_since = 0;
    {   // a ceiling set by a ring comes back 2% per update (was 1%: 0.70 -> 0.95 took 30 updates)
        const float cap0 = L_cur.cap;
        L_cur.cap = fminf(s_stored_max(), L_cur.cap * 1.02f);
        if (L_cur.cap != cap0) LEARN_LOG(2, LV_CAP, "Lane learn: the ring ceiling is now %.3f (was %.3f; it recovers 2 %% per update, never above %.2f)", L_cur.cap, cap0, s_stored_max());
    }
    if (L_hold > 0) { L_hold--; return; }
    if (!ok || sway_active()) return;
    // 30-10: learning runs only while a calibration is on; an accepted fit feeds the confidence measure (loop-gain agreement)
    if (g_cal != CAL_IDLE) {
        double G = (L_gA * L_c) / (REF_GA * REF_C);
        L_cal_ok++; L_cal_g_sum += G; L_cal_g_sum2 += G * G;
    }
    // the loop gain G = gA c should stay at the ts4n fit; a slower wheel (delay) gets a calmer offset term
    float st = clampf(REF_GA * REF_C / (gA_eff(L_gA) * L_c), p_l_min, s_new_max());
    // no change up to 25% more delay than the reference wheel. Readings over 0.55 s are the centering band (small A barely moves the
    // wheel, which reads as delay: ts5a/ts5b 0.7-0.9 s; output-error fits of the same rides give 0.30-0.45 s), so they count as 0.55
    float dt_ = clampf(1.25f * REF_DELAY / clampf(L_dl, 0.05f, 0.55f), 0.5f, 1.0f);
    float rate = clampf(p_l_rate, 0.05f, 1);
    float ns = L_cur.s * clampf(powf(st / L_cur.s, rate), 0.8f, 1.25f), nd = L_cur.dfac * clampf(powf(dt_ / L_cur.dfac, rate), 0.8f, 1.25f);
    ns = clampf(fminf(ns, L_cur.cap), p_l_min, s_new_max());
    if (ns > p_l_max && !learn_small_signal()) ns = fminf(ns, fmaxf(p_l_max, L_cur.s));   // above learn_max only from a straight (a bend or a strong wheel answer: not now)
    nd = clampf(nd, 0.5f, 1.0f);
    if (fabsf(ns / L_cur.s - 1) < 0.03f && fabsf(nd / L_cur.dfac - 1) < 0.03f) { learn_calibrated_check(); return; }
    L_base = L_cur; L_cur.s = ns; L_cur.dfac = nd; L_trial = 1; L_dirty = 1;
    LEARN_LOG(1, LV_S | LV_DFAC, "Lane learn: new profile s %.2f (was %.2f), dfac %.2f (was %.2f) from gA %.2f delay %.2f c %.3f", ns, L_base.s, nd, L_base.dfac, L_gA, L_dl, L_c);
    // 30-10: the confidence stop only fires from the converged branch above (last change of s and dfac under 3 %), not on
    // a profile that is still moving, so a calibration is not stored before the gains have actually reached the target.
}

// ring guard: a sustained oscillation of the assist offset (period ~1-2.5 s, ts4n 1.35 s) lowers the gains at once
// The profile is the wheel's small-signal answer on a straight. A sway or ring in a sharp bend, or while the wheel answers much
// stronger than wheel_gain_ref (centering band, large A: ts5b crash, ts5a 978/1171 s), is the live part's business (the live wheel
// scale and the temporary anti-sway cut); it must not push the profile down for good (ts5a/ts5b: s to its floor 0.70 that way).
static int learn_small_signal() { return fabsf(L_lpo3) < 0.003f && !(p_wref > 0 && g_wi > 1.15f * p_wref); }

static void learn_ring(float xA, float dt)
{
    L_xa2 += (xA * xA - L_xa2) * clampf(dt / 4, 0, 1);
    // hands on the wheel: their swings are not ours (sim, a driver weaving 120 s: the crossings from the weave plus the settling
    // after letting go made 6 in 8 s and cut the profile x0.85)
    if (g_act > 0.15f || !learn_small_signal()) { for (auto &t : L_ar_t) t = -1e9; L_ar_sign = 0; L_xa2 = 0; return; }
    int sg = xA > 0.0025f ? 1 : (xA < -0.0025f ? -1 : 0);
    if (sg && sg != L_ar_sign) {
        if (L_ar_sign) { for (int i = 0; i < LK_ZC - 1; i++) L_ar_t[i] = L_ar_t[i + 1]; L_ar_t[LK_ZC - 1] = L_t; }
        L_ar_sign = sg;
    }
    int n = 0;
    for (auto t : L_ar_t) if (L_t - t < 8) n++;
    if (n >= 6 && sqrtf(L_xa2) >= 0.006f && L_t > L_ring_cool) {
        s_back_to_max("the assist offset rings");
        float old = L_cur.s;
        L_cur.s = fmaxf(p_l_min, L_cur.s * 0.85f);
        L_cur.cap = L_cur.s;
        L_base = L_cur; L_trial = 0; L_hold = 2; L_ring_cool = L_t + 10; L_dirty = 1;
        LEARN_LOG(1, LV_S | LV_CAP, "Lane learn: the assist offset rings (A rms %.4f, %d crossings in 8 s): gains x%.2f -> s %.2f", sqrtf(L_xa2), n, L_cur.s / old, L_cur.s);
        incident("lane", "the lane assist rings: %d crossings in 8 s (A rms %.4f), gains lowered x%.2f", n, sqrtf(L_xa2), L_cur.s / old);
        for (auto &t : L_ar_t) t = -1e9;
    }
}


// the learner's reading of a sway episode: this wheel and truck want less gain (never counted while the driver was steering). The
// reduction is stored (L_dirty) and only comes back through the ceiling's slow recovery, so a sway lowers the gain for the rest of
// the session and the next sway does not start from the full gains again (30-10).
static void learn_sway_hit(float fac_after)
{
    if (!(p_learn > 0 && p_mode >= 2)) return;
    s_back_to_max("a sway");
    if (g_cal != CAL_IDLE) {
        // inside a calibration the learner may store the reduction (the driver asked it to measure this wheel)
        float old = L_cur.s;
        L_cur.s = fmaxf(p_l_min, L_cur.s * 0.85f);
        L_cur.cap = L_cur.s; L_base = L_cur; L_trial = 0; L_hold = 3; L_dirty = 1;
        LEARN_LOG(1, LV_S | LV_CAP, "Lane learn: sway counts as too much gain: s %.2f -> %.2f (ceiling %.2f)", old, L_cur.s, L_cur.cap);
    } else {
        // 30-10: outside a calibration a sway lowers only a session factor (not stored), so the reduction sticks for the
        // session but cannot walk the stored profile to the floor; it comes back slowly after a calm stretch.
        float old = L_sess;
        L_sess = fmaxf(0.4f, L_sess * 0.85f);
        L_sess_at = wall_s();
        L_sess_ann = L_sess;
        LEARN_LOG(1, LV_SESS, "Lane learn: sway lowers the session factor x%.2f -> x%.2f (stored s %.2f stays)", old, L_sess, L_cur.s);
    }
}


static void learn_block_reset()
{
    B_n = 0; B_Syy = B_Cxx = B_Cxy = B_Cyy = 0;
    for (int k = 0; k < LK_NK; k++) B_Sxx[k] = B_Sxy[k] = 0;
}

// a finished block: own fit (A -> wheel, delay bank; wheel -> heading); merged into the sums only when it is clean and agrees
static void learn_block()
{
    int best = -1; double br = 1e30;
    for (int k = 0; k < LK_NK; k++) if (B_Sxx[k] > 1e-12) { double res = B_Syy - B_Sxy[k] * B_Sxy[k] / B_Sxx[k]; if (res < br) { br = res; best = k; } }
    int good = best >= 0 && B_Syy > 1e-12 && B_Cxx > 1e-12 && B_Cyy > 1e-12;
    double g = 0, r2a = 0, r2c = 0, cc = 0, rms = 0;
    if (good) {
        g = B_Sxy[best] / B_Sxx[best]; r2a = 1 - br / B_Syy; cc = B_Cxy / B_Cxx; r2c = B_Cxy * B_Cxy / (B_Cxx * B_Cyy); rms = sqrt(B_Sxx[best] / B_n);
        good = rms >= 0.0012 && r2a >= 0.5 && r2c >= 0.05 && g / REF_GA >= 0.4 && g / REF_GA <= GA_RATIO_MAX && cc / REF_C >= 0.4 && cc / REF_C <= 2.5;
    }
    int deviates = 0;
    if (good && L_Nw * LK_TICK >= 30 && L_Sxx[0] >= 0) {
        // against the long-term fit: gain (and delay) of this block within +-35%/0.15 s
        int lb = -1; double lr = 1e30;
        for (int k = 0; k < LK_NK; k++) if (L_Sxx[k] > 1e-12) { double res = L_Syy - L_Sxy[k] * L_Sxy[k] / L_Sxx[k]; if (res < lr) { lr = res; lb = k; } }
        if (lb >= 0) { double lg = L_Sxy[lb] / L_Sxx[lb]; deviates = fabs(g / lg - 1) > 0.35 || fabs((best - lb) * 0.05) > 0.15; }
    }
    if (good && deviates) {
        if (++L_dev_blocks >= 10) { for (int k = 0; k < LK_NK; k++) L_Sxx[k] = L_Sxy[k] = 0; L_Syy = L_Nw = L_Cxx = L_Cxy = L_Cyy = 0; L_dev_blocks = 0; deviates = 0;
            logv(1, "Lane learn: the wheel loop has changed for good (gA %.2f delay %.2f), fitting anew", g, best * 0.05); }
    } else if (good) L_dev_blocks = 0;
    if (good && !deviates) {
        double lam = exp(-LK_TICK * B_n / 60.0);
        for (int k = 0; k < LK_NK; k++) { L_Sxx[k] = lam * L_Sxx[k] + B_Sxx[k]; L_Sxy[k] = lam * L_Sxy[k] + B_Sxy[k]; }
        L_Syy = lam * L_Syy + B_Syy; L_Nw = lam * L_Nw + B_n; L_Cxx = lam * L_Cxx + B_Cxx; L_Cxy = lam * L_Cxy + B_Cxy; L_Cyy = lam * L_Cyy + B_Cyy;
        L_blocks_ok++;
    } else L_blocks_bad++;
    learn_block_reset();
}

static int g_ind;   // an indicator is set (lane change on purpose): not a time to learn
// Steering profile, recognized from the data (lka-roll-re notes A1): with an FFB wheel the assist only moves the wheel
// (steer == wheel, ts4w: |steer - wheel| 2e-5 while |A| 0.011); without one the game adds it: steer == wheel + A.
static int g_ffb = -1;              // -1 not known yet, 1 FFB wheel (mode 2), 0 assist added to the steering angle
static double F_n, F_sw, F_swa, F_a;
static void ffb_detect(float A, float steer, float wheel)
{
    if (!(A == A && steer == steer && wheel == wheel) || fabsf(A) < 0.005f * fminf(g_ag, 1.0f)) return;   // (A in the game's own units: the smaller A of a weak actuator still counts)
    F_n++; F_sw += fabsf(steer - wheel); F_swa += fabsf(steer - wheel - A); F_a += fabsf(A);
    if (F_n < 60) return;
    static int prev = -1;  // a known profile only changes after two windows agree (a single odd window is noise)
    int f = F_sw < 0.2 * F_a ? 1 : F_swa < 0.3 * F_a ? 0 : -1;
    if (f >= 0 && f != g_ffb && (g_ffb < 0 || f == prev))
        logline("Lane: %s", f ? "force-feedback wheel: model-based law (mode 2)"
                              : (p_nonffb >= 2 ? "no force-feedback wheel (assist adds to the steering angle): mode 2 anyway (non_ffb_mode = 2, not tuned)"
                                               : "no force-feedback wheel (assist adds to the steering angle): the game's own law (mode 2 is fitted on FFB wheels)"));
    else if (f < 0 && g_ffb < 0) logline("Lane: steering profile unclear (steer-wheel %.4f, steer-wheel-A %.4f, A %.4f): the game's own law for now", F_sw / F_n, F_swa / F_n, F_a / F_n);
    int take = f >= 0 && (g_ffb < 0 || f == prev);
    prev = f;
    if (take) g_ffb = f;
    F_n = F_sw = F_swa = F_a = 0;
}
// mode 2 runs on an FFB wheel (or when forced); until the profile is known the game's own law steers
static int law2() { return p_mode >= 2 && (g_ffb == 1 || (g_ffb == 0 && p_nonffb >= 2)); }

static int g_pause;                 // indicator pause (indicator_pause): no request of ours, the wheel is the driver's
static double g_resume_at = -1e9;   // when the pause ended: the request ramps back in over 1 s
static double g_lane_ok_at = -1e9, g_lane_bad_at = -1e9;  // last law call with a lane near its center / without one
static void learn_step(float u, float o1, float o2, float o3, float dt, float wheel, float A, float kmh)
{
    if (!(p_learn > 0 && p_mode >= 2) || !(wheel == wheel) || !(A == A)) { L_usable = 0; return; }
    L_t += dt;
    float v = kmh / 3.6f;
    if (!L_init) {
        L_Am = A; L_wm = wheel; L_lp2 = o2; L_lpw = v * wheel; L_lpo = v * o3; L_lpo3 = o3; L_mx = L_my = 0; L_xa2 = 0;
        L_init = 1; L_warm = L_t + 4; L_quiet = 0; L_acc = L_t;
    }
    float a3 = clampf(dt / 3, 0, 1), a03 = clampf(dt / 0.3f, 0, 1), a10 = clampf(dt / 10, 0, 1);
    L_Am += (A - L_Am) * a3; L_wm += (wheel - L_wm) * a3;
    L_lp2 += (o2 - L_lp2) * a03; L_lpw += (v * wheel - L_lpw) * a03; L_lpo += (v * o3 - L_lpo) * a03;
    L_lpo3 += (o3 - L_lpo3) * clampf(dt / 1.0f, 0, 1);
    float xA = A - L_Am, yw = wheel - L_wm;
    float cy = (o2 - L_lp2) / 0.3f + LK_D * L_lpo, cx = -L_lpw;      // heading rate + bend part  vs  -v wheel  (slope = c)
    L_mx += (cx - L_mx) * a10; L_my += (cy - L_my) * a10;
    L_rt[L_rn % LK_RING] = L_t; L_rx[L_rn % LK_RING] = xA; L_rn++;
    learn_ring(xA, dt);
    if (g_act > 0.15f) L_quiet = L_t + 3;
    // 30-10: learning runs only while a calibration is on (the driver arms it), and now also in gentle bends: a lateral-acceleration
    // cap (v^2 |o3| < bend_max_lat) replaces the near-straight lane term, and a slow bend entry is allowed (the 1 s lane-term filter
    // lags only a little); the R2 gates reject the rest. The sway/ring reductions are outside this gate and stay active.
    L_usable = g_cal != CAL_IDLE && kmh >= LEARN_MIN_KMH && g_act < 0.08f && L_t > L_warm && L_t > L_quiet &&
               (!(p_bend_lat > 0) || v * v * fabsf(o3) < p_bend_lat) && fabsf(o3 - L_lpo3) < 0.002f &&
               fabsf(o1) < 1.0f && fabsf(u) < 0.3f && !g_ind && fabsf(A) < 0.045f && dt < 0.2f && L_t > L_ring_cool - 6 && !sway_active() && lat_eff() >= 0.99f;
    if (!L_usable) { learn_block_reset(); return; }
    if (L_t - L_acc < LK_TICK) return;
    L_acc = L_t;
    float xd[LK_NK];
    for (int k = 0; k < LK_NK; k++) if (!ring_at(L_t - 0.05 * k, &xd[k])) return;
    for (int k = 0; k < LK_NK; k++) { B_Sxx[k] += xd[k] * xd[k]; B_Sxy[k] += xd[k] * yw; }
    B_Syy += yw * yw;
    double xs = cx - L_mx, ys = cy - L_my;
    B_Cxx += xs * xs; B_Cxy += xs * ys; B_Cyy += ys * ys;
    if (++B_n >= LK_BLOCK) learn_block();
    // score of the profile in use: offset rms and sway (zero crossings of o1 beyond 0.1 m)
    L_scs += o1 * o1; L_scn++;
    int sg = o1 > 0.1f ? 1 : (o1 < -0.1f ? -1 : 0);
    if (sg) { if (L_zc_sign && sg != L_zc_sign) L_sc_zc++; L_zc_sign = sg; }
    L_since += LK_TICK;
    if (L_since >= p_l_update) learn_update();
}

// ---- anti-sway (Damien, dev.6 ride): o1 zig-zagging around the center, not dying out -> cut the gains now, give them back slowly
static double SW_t, SW_hold_until, SW_lastact = -1e9, SW_ex_t[3]; static float SW_ex_v[3], SW_cand, SW_fac = 1;
static double SW_cand_t; static int SW_dir, SW_n, SW_events;
static void sway_reset() { SW_dir = 0; SW_n = 0; }
static int sway_active() { return p_sway > 0 && (SW_fac < 0.999f || SW_t < SW_hold_until); }
static void learn_sway_hit(float fac_after);
static void sway_step(float o1, float dt, int indicator)
{
    if (!(p_sway > 0)) { SW_fac = 1; return; }
    SW_t += dt;
    if (g_act > 0.15f) SW_lastact = SW_t;
    // recovery: nothing for 5 s after the last hit, then linear back to 1 over sway_recover_s
    if (SW_t > SW_hold_until && SW_fac < 1) SW_fac = fminf(1, SW_fac + dt * (1 - 0.4f) / fmaxf(p_sw_rec, 1.0f));
    // 30-10: the session factor recovers only after a calm stretch (4x sway_recover_s), and 4x as slowly again
    if (L_sess < 1 && wall_s() - L_sess_at > 4 * fmaxf(p_sw_rec, 1.0f)) {
        L_sess = fminf(1, L_sess + dt / (4 * fmaxf(p_sw_rec, 1.0f)));
        if (L_sess - L_sess_ann >= 0.05f || (L_sess >= 1 && L_sess_ann < 1)) {   // the recovery: a line for every 0.05 of it
            LEARN_LOG(2, LV_SESS, "Lane learn: the session factor recovers x%.2f -> x%.2f", L_sess_ann, L_sess);
            L_sess_ann = L_sess;
        }
    }
    if (p_follow > 0 && fabsf(g_ref) > 0.03f) { SW_dir = 0; return; }   // while you hold an offset (shared steering) a swing is yours (13-10: two sway hits right after steering)
    if (lat_eff() < 0.5f) { SW_dir = 0; SW_n = 0; return; }           // yielding to the driver's hand for the sideways load: the swing of o1 is the roundabout's or the driver's, not the gains' (WP-LA5)
    if (SW_dir == 0) { SW_dir = o1 >= 0 ? 1 : -1; SW_cand = o1; SW_cand_t = SW_t; return; }
    float amp = fmaxf(p_sw_amp, 0.05f), rev = 0.5f * amp;   // a reversal of rev confirms an extremum; swings (peak to peak) must be >= 1.5 amp
    int confirm = 0;
    if (SW_dir > 0) { if (o1 > SW_cand) { SW_cand = o1; SW_cand_t = SW_t; } else if (SW_cand - o1 > rev) confirm = 1; }
    else { if (o1 < SW_cand) { SW_cand = o1; SW_cand_t = SW_t; } else if (o1 - SW_cand > rev) confirm = 1; }
    if (!confirm) return;
    float val = SW_cand; double tv = SW_cand_t;
    SW_dir = -SW_dir; SW_cand = o1; SW_cand_t = SW_t;
    if (indicator) { SW_n = 0; return; }
    // consecutive extrema alternate max/min by construction; keep the last three, each swing big enough and quick enough
    // half periods over 1.6 s are a slow wander (too little gain), not the over-gain sway (0.7-1.4 s in ts4n-t): ts4u
    // counted 2-3 s wander as sway, cut the gains, wandered more and spiraled down to x0.2 (a car left had to brake)
    if (SW_n > 0 && (fabsf(val - SW_ex_v[SW_n - 1]) < 1.5f * amp || tv - SW_ex_t[SW_n - 1] > 1.6 || tv - SW_ex_t[SW_n - 1] < 0.4)) SW_n = 0;
    if (SW_n == 3) { SW_ex_v[0] = SW_ex_v[1]; SW_ex_t[0] = SW_ex_t[1]; SW_ex_v[1] = SW_ex_v[2]; SW_ex_t[1] = SW_ex_t[2]; SW_n = 2; }
    SW_ex_v[SW_n] = val; SW_ex_t[SW_n] = tv; SW_n++;
    if (SW_n < 3) return;
    double h2 = SW_ex_t[2] - SW_ex_t[1];
    // the second swing is not smaller than 85% of the first (and the first swing was a real one): not dying out
    float p1 = fabsf(SW_ex_v[1] - SW_ex_v[0]), p2 = fabsf(SW_ex_v[2] - SW_ex_v[1]);
    if (p2 < 0.85f * p1) return;
    float before = SW_fac;
    SW_fac = fmaxf(0.4f, SW_fac * clampf(p_sw_cut, 0.3f, 0.95f));
    SW_hold_until = SW_t + 5; SW_events++; SW_n = 0;
    DEV_NOTE(sway());
    logv(1, "Lane: sway detected (o1 extremes %.2f / %.2f / %.2f m, half period %.1f s), gains x%.2f -> x%.2f", SW_ex_v[0], SW_ex_v[1], SW_ex_v[2], h2, before, SW_fac);
    incident("lane", "the lane assist sways (o1 extremes %.2f / %.2f / %.2f m, half period %.1f s), gains x%.2f -> x%.2f at %.0f km/h", SW_ex_v[0], SW_ex_v[1],
             SW_ex_v[2], h2, before, SW_fac, fabsf(T.speed) * 3.6f);
    // 30-10: a detected sway lowers the profile too, so the reduction lasts past the temporary cut (which comes back in sway_recover_s).
    // Only while the driver was not steering it (SW_lastact): a swing they steer themselves is not the gains' fault. The small-signal
    // gate is gone: a sway in a bend is still a sway that must not repeat with the full gains (the ceiling comes back slowly).
    if (SW_t - SW_lastact > 8) learn_sway_hit(SW_fac);
}

static double W_t, W_last, W_rt[64]; static float W_ra[64]; static int W_n; static double W_sxy, W_sxx, W_ixy, W_ixx; static float W_ma, W_mw, W_adp, W_adr; static int W_have_ad;
static float g_acmd_prev; static int g_have_acmd;
static void lane_reset()
{
    W_n = 0; W_sxy = W_sxx = W_ixy = W_ixx = 0; W_ma = W_mw = 0; g_wg = g_wi = p_wref; g_wpred = NAN; g_have_acmd = 0; W_have_ad = 0; g_pipe = g_wexp_rate = 0;
    g_lf = g_li = 1; g_hold = 0;
    g_act = g_ci = g_d2 = g_damp_u = g_w_rate = g_ff = 0; g_have_o3 = 0; g_do3 = 0;
    g_ref = 0; g_act_t = 1e3f; g_steady_t = 0; g_dev = g_pm = g_hold_t = g_age = 0; g_share = 0; g_bias = 0;
    g_scale = 1;
    g_hdev = g_hwheel = 0;
    g_have_w = g_have_o = 0;
    g_engaged = 0;
    learn_flush();
}

// Live wheel gain (ts5b crash, lane notes): hands off the wheel follows the assist offset as wheel = G A(t - 0.33 s) minus a
// centering band (crash fit G 2.95, band 0.010; ts4w G ~1.5): on a straight with small A that is ~1.2x, in a sharp bend 2-3x, so the
// loop gain triples exactly where the bend asks most. A short least-squares ratio (1.5 s memory, a prior at wheel_gain_ref) follows it.
static const float W_DELAY = 0.33f;
static float w_delayed(float *out)
{
    int n = W_n < 64 ? W_n : 64; double tq = W_t - W_DELAY;
    for (int i = 1; i < n; i++) {
        int j1 = (W_n - i) % 64, j0 = (W_n - i - 1) % 64;
        if (W_rt[j0] <= tq && tq <= W_rt[j1]) { float a = W_rt[j1] > W_rt[j0] ? (float)((tq - W_rt[j0]) / (W_rt[j1] - W_rt[j0])) : 1; *out = W_ra[j0] + a * (W_ra[j1] - W_ra[j0]); return 1; }
    }
    return 0;
}
static void wheel_gain_step(float A, float wheel, float dt)
{
    if (!(p_wref > 0)) { g_wg = g_wi = 1; g_wpred = NAN; return; }
    if (!(g_wg > 0)) g_wg = p_wref;
    if (!(g_wi > 0)) g_wi = p_wref;
    if (!(A == A && wheel == wheel)) { g_wpred = NAN; return; }
    W_t += dt;
    if (W_n == 0 || W_t - W_last >= 0.02) { W_rt[W_n % 64] = W_t; W_ra[W_n % 64] = A; W_n++; W_last = W_t; }
    float Ad;
    if (!w_delayed(&Ad)) { g_wpred = NAN; W_have_ad = 0; g_pipe = 0; return; }
    if (!W_have_ad) { W_ma = W_adp = Ad; W_mw = wheel; W_adr = 0; W_have_ad = 1; }
    W_adr += ((Ad - W_adp) / dt - W_adr) * clampf(dt / 0.1f, 0, 1);   // same 0.1 s filter as the wheel speed
    W_adp = Ad;
    double lam = exp(-dt / 1.5);
    W_sxy *= lam; W_sxx *= lam; W_ixy *= lam; W_ixx *= lam;
    int quiet = g_act < 0.15f;
    if (fabsf(Ad) > 0.006f && quiet && Ad * wheel > 0) { W_sxy += (double)Ad * wheel * dt * 50; W_sxx += (double)Ad * Ad * dt * 50; }
    // incremental: deviations from a 1.5 s mean (the loop sees this slope when it swings around a bend's operating point)
    float a15 = clampf(dt / 1.5f, 0, 1);
    W_ma += (Ad - W_ma) * a15; W_mw += (wheel - W_mw) * a15;
    float xa = Ad - W_ma, yw = wheel - W_mw;
    // gated on clear driver activity only (0.3, not quiet's 0.15): the activity is judged with this very slope, and a wrong slope
    // must not make a sway look like hands on the wheel (sim of ts5b with the ts4w profile: frozen at 1.3 in a 1.3 m sway); letting
    // go of a bend you steered yourself is a clear jump (sim: read as slope 3.0, the feed-forward halved for a second)
    if (g_act < 0.3f) { W_ixy += (double)xa * yw * dt * 50; W_ixx += (double)xa * xa * dt * 50; }
    const double P = 0.008 * 0.008 * 20, Pi = 0.003 * 0.003 * 15;   // priors at wheel_gain_ref: back there a few seconds after the signal is gone
    g_wg = clampf((float)((W_sxy + p_wref * P) / (W_sxx + P)), 0.5f, 5.0f);
    g_wi = clampf((float)((W_ixy + p_wref * Pi) / (W_ixx + Pi)), 0.5f, 5.0f);
    // a centering band makes the ratio smaller than the slope (G A - band), so the ratio is never above the slope: hands holding the
    // wheel into a bend inflate the ratio (it is steady) but not the slope (sim: the feed-forward dropped, 2.9 m wide while steering)
    if (g_wg > g_wi) g_wg = fmaxf(g_wi, p_wref < g_wg ? p_wref : g_wg);
    g_wpred = g_wg * Ad;
    g_wexp_rate = W_adr;
    // mean A over the pipe (samples newer than t - delay)
    double sa = 0; int na = 0, n = W_n < 64 ? W_n : 64;
    for (int i = 1; i <= n; i++) { int j = (W_n - i) % 64; if (W_rt[j] < W_t - W_DELAY) break; sa += W_ra[j]; na++; }
    g_pipe = na ? g_wi * (float)(sa / na - Ad) : 0;
}

// Mode 2 control (the model-based law; mode 1, the 0.4.14 law, was removed 11-10).
// Pure function of its inputs (tools/sim/sim_lane.cpp runs it).
// Shared steering (follow 1): the share of the offset pull left while your hand is on the wheel. Not in a bend (|o3| over follow_bend_o3):
// there the pull is whole. A weight over the last fifth under that value, not a switch at it (27-10: the pull stepped by (1 - follow_backoff) x
// the offset term there); the whole pull comes back a little earlier than before, never later.
static float follow_fb(float o3, float act)
{
    if (!(p_follow > 0)) return 1;
    float bend = clampf((fabsf(o3) - 0.8f * p_f_bend) / fmaxf(0.2f * p_f_bend, 1e-6f), 0, 1);
    return 1 - (1 - bend) * clampf((act - 0.3f) / 0.4f, 0, 1) * (1 - p_f_back);
}
static float lane_step(float u, float o1, float o2, float o3, float dt, float wheel, float A, float kmh)
{
    if (!(dt > 0 && dt < 0.5f)) dt = 0.016f;
    g_law_at = wall_s();
    wheel_gain_step(A, wheel, dt);
    driver_activity(wheel, A, dt);
    if (g_wpred == g_wpred && wheel == wheel) { g_hdev += ((wheel - g_wpred) - g_hdev) * clampf(dt / 0.5f, 0, 1); g_hwheel = wheel; } else { g_hdev = 0; g_hwheel = 0; }   // (for lat_hand)
    // the lane changed under us (o1 jumps): do not differentiate across it
    if (g_have_o && fabsf(o1 - g_o1_prev) > 0.6f) g_have_o = 0;
    if (g_have_o) {
        float d = (o2 - g_o2_prev) / dt;
        g_d2 += (d - g_d2) * clampf(dt / 0.3f, 0, 1);
        float d1 = (o1 - g_o1_prev) / dt;  // drift speed away from / towards the center (m/s)
        g_do1 += (clampf(d1, -3, 3) - g_do1) * clampf(dt / 0.4f, 0, 1);
    } else { g_d2 = 0; g_do1 = 0; }
    g_o1_prev = o1;
    g_o2_prev = o2;
    g_have_o = 1;
    // Shared steering (Damien 24-10: hands on the wheel now and then, not only hands-free). Holding an offset (passing a cyclist) makes
    // it the aim for a while: only the offset terms (offset, integral) look at e = o1 - aim, the heading damping, the bend steering and
    // the feedback on A stay whole (ts5b: they must never give way). The aim goes back to the lane center when you let go.
    if (p_follow > 0) {
        if (g_act > p_f_act) g_act_t = 0; else g_act_t += dt;
        g_age += dt;
        g_steady_t = fabsf(g_do1) < 0.15f ? g_steady_t + dt : 0;
        // A hand held still against the assist: the wheel is not where the assist alone would put it (wheel - g_wpred, 0.5 s filter) and
        // the assist pulls hard enough to show it. Steering speed (g_act) does not see a gentle offset: 0.5 m in 2 s hardly moves the wheel.
        if (g_wpred == g_wpred && wheel == wheel) {
            float k = clampf(dt / 0.5f, 0, 1);
            g_dev += ((wheel - g_wpred) - g_dev) * k; g_pm += (g_wpred - g_pm) * k;
        } else g_dev = g_pm = 0;
        bool against = fabsf(g_pm) > p_f_pull && g_dev * g_pm < 0 && fabsf(g_dev) > 0.6f * fabsf(g_pm) && fabsf(g_w_rate) < 0.05f;   // and a still wheel: a hands-off sway moves it
        // not in the first 3 s after the assist started (its own start-up lags the wheel), and the truck parallel to the lane
        bool holding = against && fabsf(o1) > 0.2f && fabsf(o2) < 0.01f && g_steady_t > 0.3f && g_age > 3;
        g_hold_t = holding ? g_hold_t + dt : fmaxf(0, g_hold_t - 2 * dt);   // leaky: hand tremor must not restart it every other frame
        bool straight = fabsf(o3) < p_f_bend && !g_pause;
        bool take = straight && fabsf(o1) > 0.05f &&
                    ((g_act_t < p_f_win && g_steady_t > p_f_hold && fabsf(o1) > 0.2f && fabsf(o2) < 0.01f) || g_hold_t > p_f_hold);   // (steering speed alone also fires on a hands-off sway of a very strong wheel: sim gA 3.5 at 90 km/h)
        if (g_ref != 0 && (g_ref * o1 < 0 || fabsf(o1) < fabsf(g_ref))) {
            // the truck is between the lane center and the aim (you steered back, or it comes back by itself): the aim comes along at once,
            // it never pulls the truck away from the center (sim: the evasive 1.2 m and back left the aim at 0.3 m and the truck overshot 0.36 m)
            g_ref += (o1 - g_ref) * clampf(dt / 0.2f, 0, 1);
        } else if (take) {
            g_ref += (clampf(o1, -p_f_max, p_f_max) - g_ref) * clampf(dt / fmaxf(p_f_tau, 0.05f), 0, 1);
        } else if (g_act < 0.15f) {
            float back = g_ref * (1 - expf(-dt / fmaxf(p_f_ret, 0.5f)));
            g_ref -= clampf(back, -p_f_rate * dt, p_f_rate * dt);
        }
    } else g_ref = 0;
    // Keep right (Damien 24-10: "iets rechts van het midden", 0.325 m): the aim moves to the right on a straight, slowly (0.1 m/s), and
    // back to the center in a bend (fully gone from |o3| 0.0045), under 40 km/h, with the indicator paused, or when the lane is not clean.
    {
        float bend = clampf((fabsf(o3) - 0.0015f) / 0.003f, 0, 1), straight = 1 - bend;
        // keep right on a straight; in a bend the outside (trailer room): + is left, a right bend has o3 > 0 and its outside is the left
        float sharp = 1 - clampf((fabsf(o3) - 0.006f) / 0.003f, 0, 1);   // the outside shrinks again in a very sharp bend (from R 170 m): the lane edge is near
        float want = -clampf(p_bias, 0, 0.6f) * straight + clampf(p_bend_out, 0, 0.6f) * bend * sharp * (o3 > 0 ? 1.0f : -1.0f);
        if (!(kmh > 40 && !g_pause && fabsf(o1) < 1.2f && (p_bias > 0 || p_bend_out > 0))) want = 0;
        // The aim must not be a step for the loop: 24-10 evening, four sways in 11 min, each while the keep-right aim ran from 0 to
        // -0.33 m at 0.15 m/s (the real wheel answers stronger than the plant model). So 0.05 m/s (a bend's outside needs 6 m of road for
        // 0.3 m), and frozen while the sway cure is active (gains cut) or the lane was just lost.
        float st = 0.05f * dt * (want == 0 ? 2.0f : 1.0f);   // out twice as fast as in
        if (SW_fac < 0.99f && want != 0) st = 0;
        g_bias += clampf(want - g_bias, -st, st);
    }
    float e1 = o1 - g_ref - g_bias;   // the offset the assist works on (aim: your held offset and the keep-right bias)
    // slow integral on the offset: only while the assist is in charge, frozen / bled off while you steer
    // A light vehicle (car) steers faster than a truck (less mass, quicker yaw): the loop gains are a little lower. A first guess,
    // to be tuned on a car; the measured wheel response (lf / li) and the learner still adapt on top of it.
    float lk = vprof()->lane_gain_k;
    float ci_k = p_i2 * lk, ci_max = p_i2max;  // in units of A
    if (g_act < 0.3f && fabsf(e1) < 1.2f) g_ci -= ci_k * e1 * dt;
    g_ci *= expf(-dt * (0.1f + 1.5f * g_act));
    g_ci = clampf(g_ci, -ci_max, ci_max);
    {
        // Model-based law (lka-sysid notes). Request per physical effect:
        //  - offset: stiffer towards the line (edge_boost), slow integral for a steady offset
        //  - heading o2: the damping; do1/dt = -0.88 v o2, so it is the lead the 0.5 s actuator lag needs
        //  - bend feed-forward: the bend needs wheel -BEND_NEED o3 (fitted); the game only adds -0.5 v o3, ~1/4 of that
        //  - feedback on A: the game's integrator (tau 0.26 s) is closed into a fast loop
        float ahead = o1 + g_do1 * p_edge_look;
        float off = fmaxf(fabsf(o1), o1 * ahead > 0 || fabsf(o1) < 0.05f ? fabsf(ahead) : 0);
        float k = clampf((off - p_edge_from) / fmaxf(p_edge_full - p_edge_from, 0.05f), 0, 1);
        float stiff = 1 + p_edge_boost * k * k * (3 - 2 * k);
        float be = model_be(kmh), al = model_al(kmh);
        float afb = (A == A) ? p_k_afb : 0;
        // wanted assist offset A (the bend's own steering is -BEND_NEED o3, fitted)
        // heading gain lower at speed: at 89 km/h after a bend it rang with a 1.35 s period (Damien: sways after a bend)
        float k_head = p_k_head * lk * fminf(1.0f, fmaxf(0.5f, p_head_ref / fmaxf(kmh, 1.0f)));
        // learned profile (1 when learn = 0): offset/heading gains and bend/slow terms follow 1/(gA c); a slower wheel calms offset more than heading
        learn_step(u, e1, o2, o3, dt, wheel, A, kmh);   // (the error about the aim, not about the center: the bias is no sway and no plant)
        sway_step(e1, dt, g_ind);
        float ls = scale_s(), ldf = scale_df();
        float swf = p_sway > 0 ? SW_fac : 1;
        // the wheel answers stronger than the gains are tuned for (sharp bend, large A): every A term scaled down (never up)
        // the feed-forward by the ratio (wheel per A where it is), the feedback by the slope (what the loop sees)
        float lf = p_wref > 0 ? clampf(p_wref / fmaxf(g_wg, 0.01f), 0.35f, 1.0f) : 1.0f;
        float li = p_wref > 0 ? clampf(p_wref / fmaxf(g_wi, 0.01f), 0.4f, 1.0f) : 1.0f;
        // the bend's own steering is static (no loop gain): by the measured ratio only, not by the learned / sway scales
        // bend term ahead: o3 + bend_lead_s x its rate (0.2 s filter), the extra never more than half of |o3| + 0.002
        if (g_have_o3) g_do3 += (clampf((o3 - g_o3_prev) / dt, -0.05f, 0.05f) - g_do3) * clampf(dt / 0.2f, 0, 1); else g_do3 = 0;
        g_o3_prev = o3; g_have_o3 = 1;
        float lead = clampf(p_bend_lead * g_do3, -(0.5f * fabsf(o3) + 0.002f), 0.5f * fabsf(o3) + 0.002f);
        float o3f = o3 + lead;
        if (p_bend_lat > 0) { float v = fmaxf(kmh / 3.6f, 3.0f); o3f = clampf(o3f, -p_bend_lat / (v * v), p_bend_lat / (v * v)); }   // a lane term that jumps beyond any real bend: not a bend
        // (+25% x (1 - lf): the centering band makes the ratio read high exactly where the bend needs most; sim of ts5b: 76% of the
        // need instead of 82%, the truck 0.3-0.9 m wide, with it 0.4)
        float ff = -p_bend_a * lk * (p_wref > 0 ? lf * (1 + 0.25f * (1 - lf)) : ls) * o3f;
        // towards the line the offset term gets the learned / sway scales back (ts5b profile: 0.25x of it held the truck in balance
        // 1.9 m out in a sharp bend in the sim): from 0.5 m to full at edge_full. The live wheel scale li stays (that is the plant).
        float so = ls * ldf * swf, ek = clampf((fabsf(o1) - 0.5f) / fmaxf(p_edge_full - 0.5f, 0.05f), 0, 1);
        so = (so + (fmaxf(so, 1.0f) - so) * ek) * li;
        // (a Smith predictor on the heading, the wheel motion still in the pipe, was tried 11-10: no gain in the ts5b bend and it rang
        // at 90 km/h on a slow wheel, delay 0.35 + lag 0.15)
        // your hand is clearly on the wheel (activity over 0.3) on a straight: less pull on the offset only. Not in a bend (there the
        // activity can be a misreading, ts5b) and not for the small activity of hand noise.
        float fb = follow_fb(o3, g_act);
        float a_cmd = -p_k_off * lk * so * stiff * fb * e1 + (k_head * ls * sqrtf(ldf) * (0.5f + 0.5f * swf) * o2 + g_ci * ls) * li + ff;
        // Bend blend (Damien 24-10, his Polo's Travel Assist: what you already steer counts, the assist adds the rest). The wheel minus
        // where the assist alone puts it (g_dev, 0.5 s filter) is your own steering; the part of it that goes into the bend, over a dead
        // zone of a quarter of the need, takes that share off the assist's bend steering (never more than bend_blend_max). It builds
        // back within 0.3 s when you let go. Not on a straight, not slower than 30 km/h, not when the wheel gain is unknown.
        {
            float need_b = BEND_NEED * fabsf(o3), sbw = o3 > 0 ? -1.0f : 1.0f;
            float share = 0;
            if (p_blend > 0 && g_wpred == g_wpred && need_b > 0.012f && kmh > 30) {
                float dpart = g_dev * sbw;   // your steering into the bend, in wheel range
                share = fminf(clampf((dpart - 0.25f * need_b) / (0.5f * need_b), 0, 1), clampf(p_blend_max, 0, 1));
            }
            g_share += (share - g_share) * clampf(dt / (share > g_share ? 0.5f : 0.3f), 0, 1);
            ff *= 1 - g_share;
        }
        g_ff = ff;
        // Bend hold (ts5b crash: the request reversed and the wheel unwound to 0 in a sharp bend, the truck ran out of the lane;
        // the assist can unwind fast but winds back slowly: A' = be u - al A is small near its ceiling). In a bend, while the wheel
        // (yours included) is heading under bend_hold of what the bend needs: no unwinding of A, and at least the A that puts the
        // wheel there. Unwinding no faster than unwind_s while the wheel is not past the need (an overshoot unwinds freely).
        float sb = o3 > 0 ? -1.0f : 1.0f, need = BEND_NEED * fabsf(o3);   // bend direction of A / wheel, the wheel the bend needs (fitted)
        float bk = clampf((fabsf(o3) - 0.003f) / 0.003f, 0, 1);     // 0 on a straight, full from o3 0.006 (R ~ 170 m)
        // fades out when the truck is already on the inside of the bend (0.5 -> 1.0 m): there unwinding is the cure (sim: a hand that
        // steered the bend too, 2.3 m inside, held there)
        bk *= clampf((1.0f - o1 * sb) / 0.5f, 0, 1);
        int hold_a = 0;
        float wfb = (wheel == wheel ? wheel + g_pipe : NAN) * sb;   // where the wheel will be in 0.33 s, in the bend's direction
        if (p_bend_hold > 0 && bk > 0 && wfb == wfb) {
            float thr = p_bend_hold * bk * need;
            if (wfb < thr) {
                // in A: what puts the wheel at bend_hold of the need by the measured ratio, at least bend_hold of the feed-forward
                float floor_a = p_bend_hold * bk * fmaxf(fabsf(ff), p_wref > 0 ? need / fmaxf(g_wg, 0.5f) : 0);
                if (a_cmd * sb < floor_a) a_cmd = sb * floor_a;
                hold_a = A * sb > 0 && a_cmd * sb < A * sb;
            }
            if (p_unwind_s > 0 && g_have_acmd && (g_acmd_prev - a_cmd) * sb > 0 && !(wfb > need)) {
                float maxd = bk * fabsf(ff) / p_unwind_s * dt;
                if ((g_acmd_prev - a_cmd) * sb > maxd) a_cmd = g_acmd_prev - sb * maxd;
            }
        }
        g_acmd_prev = a_cmd; g_have_acmd = 1;
        g_lf = lf; g_li = li; g_hold = bk > 0 && p_bend_hold > 0 && wfb == wfb && wfb < p_bend_hold * bk * need ? (hold_a ? 2 : 1) : 0;
        // request that makes the game's integrator (A' = -al A + be u) follow a_cmd: leak compensation + fast feedback
        float r2 = (al / be + afb) * a_cmd - (afb > 0 ? afb * A : 0);
        g_damp_u = -(afb > 0 ? afb * A : 0);
        if (hold_a) { float hold_r = al / be * A; if ((r2 - hold_r) * sb < 0) r2 = hold_r; }   // keep A where it is (leak compensated)
        float want2 = fabsf(r2) < p_dead ? 0 : r2 * p_gain * g_scale;
        want2 = clampf(want2, -p_umax, p_umax);
        // Safety (ts4i, Damien nearly crashed): when the game asks for a firm correction, ours goes the same way and at least half as strong
        // In a bend not when the game says unwind and the wheel is under bend_hold of the need (sim of ts5b: it unwound the wheel to 0).
        if (fabsf(u) > 0.3f && (want2 * u <= 0 || fabsf(want2) < 0.5f * fabsf(u)) && !(bk > 0 && p_bend_hold > 0 && u * sb < 0 && wfb < p_bend_hold * bk * need)) want2 = 0.5f * u;
        // Runaway guard (24-10 evening: the request at its ceiling both ways, o2 0.36 rad at 75 km/h, the truck tipped over; the sway detector
        // needs several swings and was far too late). Three ceiling hits that alternate (+ - +) within 3.5 s, a sustained one-sided saturation
        // whose heading error keeps growing, or a heading error over 0.18 rad: let go for 4 s and come back softly (the soft start).
        if (p_runaway > 0 && kmh > 40) {
            double nowp = wall_s();
            // Yielding to the driver's hand for the sideways load (WP-LA5): the request is no longer the truck's, so there is nothing to catch, and a hand-over to the
            // game's law that the next seconds would bring back at the ceiling (25-10: the game's own request went there with ours in the roundabout) is worse than the
            // fade. The history of ceiling hits is dropped, so the guard starts clean when the weight is back.
            const bool away = lat_eff() < 0.05f;
            if (away) { g_sat_dir[0] = g_sat_dir[1] = g_sat_dir[2] = 0; g_sat_since = 0; }
            if (fabsf(want2) > 0.95f) {
                int d = want2 > 0 ? 1 : -1;
                if (g_sat_dir[0] != d || nowp - g_sat_t[0] > 0.3) {
                    g_sat_dir[2] = g_sat_dir[1]; g_sat_t[2] = g_sat_t[1]; g_sat_dir[1] = g_sat_dir[0]; g_sat_t[1] = g_sat_t[0];
                    g_sat_dir[0] = d; g_sat_t[0] = nowp;
                    g_sat_since = nowp;   // review 30-10: (re)start the one-sided saturation on a direction change or a 0.3 s gap
                } else g_sat_t[0] = nowp;
            } else g_sat_since = 0;   // review 30-10: not saturated, so no one-sided run in progress
            // three ceiling hits that alternate (+ - +) within 3.5 s: a swing, not one hard correction (two alternating hits fired on recoverable bends)
            bool flip = g_sat_dir[2] != 0 && g_sat_dir[0] != g_sat_dir[1] && g_sat_dir[1] != g_sat_dir[2] && nowp - g_sat_t[2] < 3.5;
            // review 30-10: a single sustained runaway to one side is only caught by the 0.18 rad heading error, late. A request pinned one
            // way for longer than the loop's latency (the actuator's speed-dependent pole + the wheel lag, no new fixed number) whose
            // heading error is still growing is diverging, not turning into a bend: catch it now.
            // 30-10 (fix): heading error alone grows while any strong correction steers the truck back toward the center (a recovery after
            // a lane change or an indicator, a bend entry): the request is at its ceiling because the actuator model inverts al/be + the
            // fast feedback (assist_feedback) amplify a modest command, and the old test handed over to the weaker game law there, which
            // let the truck swing across the lane (rollover, 1651-1655). A one-sided run must actually drift AWAY from the center: the
            // offset and its rate point the same way (do1/dt = -0.88 v o2, so o1 * g_do1 > 0), not just have a heading error that grows.
            float settle = 1.0f / model_al(kmh) + W_DELAY;
            bool oneside = g_sat_since > 0 && nowp - g_sat_since > settle && o1 * g_do1 > 0 && o2 * g_d2 > 0;
            if ((flip || oneside || fabsf(o2) > 0.18f) && nowp >= g_panic_until && !away) {
                g_panic_until = nowp + 4;
                g_sat_dir[0] = g_sat_dir[1] = g_sat_dir[2] = 0;
                g_sat_since = 0;   // review 30-10: do not re-fire on the same run right after the release
                g_soft_at = nowp + 4;   // the soft start after it
                play_cue(CUE_LANE_PAUSE);
                s_back_to_max("a runaway");
                logline("Lane: runaway at %.0f km/h (request %.2f, heading error %.3f rad, o1 %.2f m): the game's own assist for 4 s", kmh, want2, o2, o1);
                incident("lane", "the lane assist ran away at %.0f km/h (request %.2f, heading %.3f rad, offset %.2f m): the game's own assist for 4 s", kmh, want2, o2, o1);
            }
            // The game's own law takes over (it is stable, just weak), not nothing: a truck with no steering at all leaves a bend (sim S-bend: 12-38 m)
            if (nowp < g_panic_until) { g_u = u; g_ci = 0; g_lane_state = LANE_RUNAWAY; return u; }
        }
        float st = p_rate2 * dt;
        return want2 > g_u + st ? g_u + st : (want2 < g_u - st ? g_u - st : want2);
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Steering calibration (the game's lane-keeping key held: `control`). A short sequence of small request pulses on a straight
// stretch, and a fit of what the wheel does with them, stored in the per-wheel profile like the learner's own estimate.
//   - Why pulses: the learner only sees what the closed loop happens to do (A rms 0.0015 on a calm drive). Pulses of the size the
//     assist really works at (A peak ~0.008) excite the wheel on purpose and are identified in seconds.
//   - Open loop, request replaced: the centering law would cancel a pulse within 0.1 s (assist_feedback closes a loop around A).
//     Safety instead comes from where it starts (centered, settled, straight, nobody close), a pulse set that returns the heading
//     to zero (two pairs, + - then - +; the offset moves one way and back, ~0.25 m at 80 km/h) and aborts the moment anything
//     looks wrong; centering then takes over with the soft start.
//   - Amplitude: the lateral excursion of a pulse is ~ c v^2 gA A, so A peak = 0.008 (80/km/h)^2 x (reference loop gain / the
//     profile's loop gain), within 0.004-0.011: about the same excursion at every speed and wheel.
static int g_cal_reset;   // g_cal_reset: the physics thread resets the centering state at its next tick (a stop from the frame must not touch it)
static double g_cal_armed_at, g_cal_ok_since, g_cal_t0, g_cal_straight_since, g_cal_why_at;
static const float CAL_LEN_S = 6.0f, CAL_PRE_S = 1.2f, CAL_PW = 0.6f, CAL_BAND_M = 0.5f, CAL_HEAD_MAX = 0.03f, CAL_START_M = 0.10f, CAL_START_HEAD = 0.008f, CAL_SETTLE_S = 3.0f;
static const float CAL_P0[4] = { 0, 0.9f, 2.4f, 3.3f }, CAL_PS[4] = { 1, -1, -1, 1 };   // pulse starts (s) and signs
enum { CAL_N = 1500, CAL_PRE_N = 256 };
struct cal_samp { float t, A, w, o1, o2; };
static cal_samp C_pre[CAL_PRE_N]; static int C_pre_n;      // ring: the last CAL_PRE_S while armed
static cal_samp C_run[CAL_N]; static int C_n;
static float C_amp, C_v, C_peak_o1, C_peak_o2;
static const char *g_cal_why = "";                         // why it is not (yet) suitable, for the log

static void cal_status(int on) { disp_post(DISP_LANE, DISP_STATUS, on ? 'T' : 0, 0); }

// 30-10: once a calibration has accepted several fits whose loop gain agrees (low spread), it judges itself calibrated well enough:
// it tells the driver (the calibration's own done sound/dash), stops learning and stores the result. The confidence state is reset
// when the driver arms again (cal_arm).
static void learn_calibrated_check()
{
    if (g_cal == CAL_IDLE || L_cal_ok < 3) return;
    double mean = L_cal_g_sum / L_cal_ok, var = L_cal_g_sum2 / L_cal_ok - mean * mean;
    if (var < 0 || sqrt(var) > 0.08f * fmaxf((float)mean, 0.5f)) return;   // the loop-gain estimates must agree
    g_cal = CAL_IDLE; cal_status(0);
    if (p_tone > 0) play_cue(CUE_LANE_CAL_DONE);
    disp_post(DISP_LANE, DISP_FLASH, 'L', 1.5);
    logline("Lane calibration: calibrated from %d accepted fits (loop gain %.2f +- %.2f), stored", L_cal_ok, mean, sqrt(var));
    prof_save("calibration");
    L_cal_ok = 0; L_cal_g_sum = L_cal_g_sum2 = 0;
}

// 0 = suitable now; else the reason it is not. The same gates as the learner's usable stretch (L_usable: hands off, quiet 3 s, a gentle
// bend at most, |o1| < 1, no indicator, small A, speed from LEARN_MIN_KMH) plus what the pulse test needs on top: a straight stretch.
static const char *cal_unsuitable(float kmh, float o1, float o2, float o3, double now)
{
    float v = kmh / 3.6f;
    if (g_pause || g_ind) return "indicator";
    if (p_needs_cc > 0 && !(T.cruise > 0.5f)) return "cruise off: lane centering does not steer";
    if (now < g_panic_until || g_soft_at >= 0) return "lane centering is still settling in";
    if (kmh < fmaxf(LEARN_MIN_KMH, p_min_kmh) || kmh > MODEL_KMH_HI) return "speed outside the band the model holds for";
    if (!L_usable) return "not a quiet stretch (steering, bend, small start-up)";
    if (LE.is161 && !AG_known) return "the actuator gain is not known yet (it is read from the lane centering's own requests)";   // (1.60 is the fit the law was made on)
    if (fabsf(o3) > 0.0025f || fabsf(o3 - L_lpo3) > 0.0008f) return "a bend (the pulse test wants a straight, steady stretch)";
    if (T.in_brake > 0.02f || T.eff_brake > 0.02f) return "braking";
    float gap = g_inc.gap_m;
    if (gap == gap && gap < 6.0f * v) return "a vehicle ahead";
    if (!(g_lane_ok_at > g_lane_bad_at && now - g_lane_bad_at > 5)) return "lane not seen steadily";
    // Centered, aligned, and not about to leave the start band: where the offset will be CAL_PRE_S ahead at its present rate (the window the fit's
    // pre-samples cover) must stay within twice the start band. The old plain rate gate (0.08 m/s) was never met by a truck that sways in its lane:
    // the 11-11 ride's lane.csv has a median |rate| of 0.09 m/s on the straight, so 25 % of the time passed it and no stretch lasted the 3 s settle.
    if (fabsf(o1) > CAL_START_M || fabsf(o2) > CAL_START_HEAD || fabsf(o1 + g_do1 * CAL_PRE_S) > 2 * CAL_START_M) return "not centered and settled";
    float need = v * (CAL_LEN_S + 1) + 50;
    float st = route_ok() ? route_straight_m(0.06f) : -1;
    if (st >= 0) { if (st < need) return "a bend ahead on the route"; }
    else if (!(now - g_cal_straight_since > 10)) return "no route and under 10 s of straight lane";
    return NULL;
}

static void cal_arm(double now)
{
    g_cal = CAL_ARMED; g_cal_armed_at = now; g_cal_ok_since = 0; g_cal_straight_since = now; C_pre_n = 0; g_cal_why = ""; g_cal_why_at = 0;
    L_cal_ok = 0; L_cal_g_sum = L_cal_g_sum2 = 0;   // 30-10: a new calibration starts the confidence from scratch
    if (p_tone > 0) play_cue(CUE_LANE_CAL_ARM);
    disp_post(DISP_LANE, DISP_FLASH, 'T', 1.5);
    logline("Lane calibration: armed (waits for a straight stretch, gives up after %.0f s)", p_cal_timeout);
}

// back to idle. A running test hands the wheel back: centering resumes with the soft start. No profile change.
static void cal_abort(const char *why)
{
    int run = g_cal == CAL_RUN;
    if (g_cal == CAL_IDLE) return;
    g_cal = CAL_IDLE; cal_status(0);
    if (p_tone > 0) play_cue(CUE_LANE_CAL_ABORT);
    logline("Lane calibration: %s: %s%s", run ? "stopped" : "disarmed", why, run ? " (profile unchanged, hold the lane-keeping key to arm again)" : "");
    if (run) g_cal_reset = 1;   // centering takes over at full strength (the next law tick resets its state): the truck is where it was a moment ago, not a fresh switch-on (the soft start would let a bend or a drift run, sim)
}

static float cal_interp(const cal_samp *s, int n, float t, int field)
{
    auto f = [&](int i) { return field == 0 ? s[i].A : field == 1 ? s[i].w : field == 2 ? s[i].o1 : s[i].o2; };
    if (t <= s[0].t) return f(0);
    for (int i = 1; i < n; i++) if (s[i].t >= t) {
        float d = s[i].t - s[i - 1].t;
        return d > 1e-6f ? f(i - 1) + (f(i) - f(i - 1)) * (t - s[i - 1].t) / d : f(i);
    }
    return f(n - 1);
}

// The fit. A -> wheel: wheel = gA A(t - d) over a bank of delays (as the learner's), c from the heading: o2(t) - o2(0) = c x -v integral of
// the wheel (+ a drift, for a wind or a camber). Returns 1 when the gain and delay are good enough to store.
struct cal_fit_t { float gA, dl, c, r2a, r2c, rms; int c_ok; };
static int cal_fit(float t0, cal_fit_t *r)
{
    static cal_samp S[CAL_N + CAL_PRE_N];
    int n = 0;
    for (int k = 0; k < CAL_PRE_N && k < C_pre_n; k++) {
        const cal_samp &p = C_pre[((C_pre_n - 1 - k) % CAL_PRE_N + CAL_PRE_N) % CAL_PRE_N];
        if (p.t - t0 < -CAL_PRE_S - 0.5f) break;
        S[n] = p; S[n].t = p.t - t0; n++;
    }
    for (int i = 0, m = n; i < m / 2; i++) { cal_samp x = S[i]; S[i] = S[m - 1 - i]; S[m - 1 - i] = x; }   // oldest first
    int n0 = n;
    for (int i = 0; i < C_n && n < CAL_N + CAL_PRE_N; i++) { S[n] = C_run[i]; S[n].t = C_run[i].t - t0; n++; }
    if (n0 < 8 || n - n0 < 40) return 0;
    // The wheel's answer is a pure delay d and a first-order lag tau: wheel = gA LP_tau(A(t - d)), gA the gain at standstill of the pulse
    // (DC), d + tau the delay the learner's pure-delay fit would read at low frequency (the lag's group delay at DC is tau). Both are
    // searched on a 20 Hz grid: tau in a short list, d in 0.05 s steps with a parabola over the best; the plain pure-delay fit alone
    // reads a lagging wheel 25 % too weak (the pulses are slow, the lag takes the top off them).
    const float dtg = 0.05f;
    enum { NG = 160, NKD = 17 };                       // grid points, delays 0..0.8 s
    static const float TAUS[] = { 0, 0.05f, 0.1f, 0.15f, 0.2f, 0.3f, 0.4f, 0.5f };
    float tg0 = S[0].t;                                // grid start (the oldest pre-roll sample)
    int ngrid = (int)((CAL_LEN_S - tg0) / dtg) + 1;
    if (ngrid > NG || ngrid < NKD + 40) return 0;
    static float Ag[NG], Wg[NG], F[NG];
    for (int g = 0; g < ngrid; g++) { Ag[g] = cal_interp(S, n, tg0 + g * dtg, 0); Wg[g] = cal_interp(S, n, tg0 + g * dtg, 1); }
    int i0 = NKD - 1, nw = ngrid - i0;
    double wm = 0, Syy = 0;
    for (int i = i0; i < ngrid; i++) wm += Wg[i];
    wm /= nw;
    for (int i = i0; i < ngrid; i++) Syy += (Wg[i] - wm) * (Wg[i] - wm);
    if (Syy < 1e-12) return 0;
    int bt = -1, bk = -1; double bres = 1e30; static double RES[8][NKD], SXY[8][NKD], SXX[8][NKD];
    for (int ti = 0; ti < 8; ti++) {
        float a = TAUS[ti] > 0 ? dtg / (TAUS[ti] + dtg) : 1;
        F[0] = Ag[0];
        for (int g = 1; g < ngrid; g++) F[g] = F[g - 1] + a * (Ag[g] - F[g - 1]);
        for (int k = 0; k < NKD; k++) {
            double am = 0, sxx = 0, sxy = 0;
            for (int i = i0; i < ngrid; i++) am += F[i - k];
            am /= nw;
            for (int i = i0; i < ngrid; i++) { double x = F[i - k] - am, y = Wg[i] - wm; sxx += x * x; sxy += x * y; }
            SXX[ti][k] = sxx; SXY[ti][k] = sxy; RES[ti][k] = sxx > 1e-12 ? Syy - sxy * sxy / sxx : 1e30;
            if (RES[ti][k] < bres) { bres = RES[ti][k]; bt = ti; bk = k; }
        }
    }
    if (bt < 0) return 0;
    // a lag only when it clearly pays: 8 % less residual than the same fit without one
    double r_nolag = 1e30; for (int k = 0; k < NKD; k++) r_nolag = fmin(r_nolag, RES[0][k]);
    if (bt > 0 && bres > 0.92 * r_nolag) { bt = 0; bk = 0; bres = 1e30; for (int k = 0; k < NKD; k++) if (RES[0][k] < bres) { bres = RES[0][k]; bk = k; } }
    double dl = bk * dtg;
    if (bk > 0 && bk < NKD - 1) {
        double r0 = RES[bt][bk - 1], r2 = RES[bt][bk + 1], den = r0 - 2 * bres + r2;
        if (den > 1e-15) dl += dtg * clampf((float)(0.5 * (r0 - r2) / den), -0.5f, 0.5f);
    }
    int ng = nw;
    r->gA = (float)(SXY[bt][bk] / SXX[bt][bk]); r->dl = (float)(dl + TAUS[bt]); r->r2a = (float)(1 - bres / Syy); r->rms = (float)sqrt(SXX[bt][bk] / ng);
    const float dmax = 0.9f;
    // heading: integrate the wheel from 0, regress o2 on it with a drift term
    {
        int nh = (int)(CAL_LEN_S / dtg);
        double wmean = 0; for (int g = 0; g <= nh; g++) wmean += cal_interp(S, n, g * dtg, 1); wmean /= (nh + 1);
        static double X[200], Y[200], Tt[200]; double acc = 0, mx = 0, my = 0, mt = 0;
        for (int g = 0; g <= nh && g < 200; g++) {
            if (g) acc += 0.5 * ((cal_interp(S, n, g * dtg, 1) - wmean) + (cal_interp(S, n, (g - 1) * dtg, 1) - wmean)) * dtg;
            X[g] = -C_v * acc; Y[g] = cal_interp(S, n, g * dtg, 3); Tt[g] = g * dtg; mx += X[g]; my += Y[g]; mt += Tt[g];
        }
        int m = nh + 1 < 200 ? nh + 1 : 200; mx /= m; my /= m; mt /= m;
        double sxx = 0, sxt = 0, stt = 0, sxy = 0, sty = 0, syy = 0;
        for (int g = 0; g < m; g++) { double x = X[g] - mx, y = Y[g] - my, t = Tt[g] - mt; sxx += x * x; sxt += x * t; stt += t * t; sxy += x * y; sty += t * y; syy += y * y; }
        double det = sxx * stt - sxt * sxt;
        r->c_ok = 0; r->c = L_c; r->r2c = 0;
        if (det > 1e-12 && syy > 1e-12) {
            double c = (sxy * stt - sty * sxt) / det, b = (sty * sxx - sxy * sxt) / det;
            double res = syy - c * sxy - b * sty;
            r->c = (float)c; r->r2c = (float)(1 - res / syy);
            r->c_ok = r->r2c >= 0.4f && r->c / REF_C >= 0.4f && r->c / REF_C <= 2.5f;
            if (!r->c_ok) r->c = L_c;
        }
    }
    return r->rms >= 0.001f && r->r2a >= 0.6f && r->gA / REF_GA >= 0.4f && r->gA / REF_GA <= GA_RATIO_MAX && r->dl >= 0 && r->dl <= dmax - 0.05f;
}

// The test is over: fit, store, report. The profile is a trial like any learner update (rolled back when the next period scores worse).
static void cal_finish()
{
    cal_fit_t r = {};
    int ok = cal_fit((float)(g_cal_t0 - g_cal_armed_at), &r);
    g_cal = CAL_IDLE; cal_status(0);
    // the wheel the test saw must not be the driver's: the fit has to explain the wheel by A alone
    if (!ok) {
        if (p_tone > 0) play_cue(CUE_LANE_CAL_ABORT);
        logline("Lane calibration: inconclusive (gA %.2f delay %.2f s R2 %.2f, A rms %.4f): profile unchanged", r.gA, r.dl, r.r2a, r.rms);
        return;
    }
    float old_gA = L_gA, old_dl = L_dl, old_c = L_c; lprof old = L_cur;
    float st = clampf(REF_GA * REF_C / (gA_eff(r.gA) * r.c), p_l_min, s_stored_max());   // (g_cal is idle again here: the calibration that just ended may set the scale above learn_max)
    if (st > p_l_max && !(r.c_ok && r.r2a >= 0.9f && r.r2c >= 0.6f)) st = p_l_max;       // above learn_max only from a clean fit of both the wheel and the heading
    float nd = clampf(1.25f * REF_DELAY / clampf(r.dl, 0.05f, 0.55f), 0.5f, 1.0f);
    float ns = clampf(fminf(st, L_cur.cap), p_l_min, s_stored_max());
    L_base = L_cur; L_cur.s = ns; L_cur.dfac = nd; L_trial = 1; L_dirty = 1; L_hold = 2;
    L_gA = r.gA; L_dl = r.dl; L_c = r.c;
    for (int k = 0; k < LK_NK; k++) L_Sxx[k] = L_Sxy[k] = 0;   // the slow estimate starts over from here
    L_Syy = L_Nw = L_Cxx = L_Cxy = L_Cyy = 0; L_dev_blocks = 0; learn_block_reset();
    if (p_tone > 0) play_cue(CUE_LANE_CAL_DONE);
    disp_post(DISP_LANE, DISP_FLASH, 'L', 1.5);
    g_lv_mask |= LV_S | LV_DFAC;
    logline("Lane calibration: done. gA %.2f (was %.2f), delay %.2f s (was %.2f), c %.3f (was %.3f%s) | R2 %.2f/%.2f, peak offset %.2f m | profile s %.2f -> %.2f, dfac %.2f -> %.2f%s",
            r.gA, old_gA, r.dl, old_dl, r.c, old_c, r.c_ok ? "" : ", not fitted", r.r2a, r.r2c, C_peak_o1, old.s, L_cur.s, old.dfac, L_cur.dfac,
            st > L_cur.cap + 1e-4f ? " (capped by the ring ceiling)" : "");
    prof_save("calibration");
}

// One law tick while armed or running. Returns 1 when the test owns the request (*out); 0 = the centering law runs as usual.
// Called from stub_law with the lane keeping on and the law running.
static int cal_tick(float o1, float o2, float o3, float dt, float wheel, float A, float kmh, float *out)
{
    double now = wall_s();
    if (g_cal == CAL_IDLE) return 0;
    float v = kmh / 3.6f;
    if (g_cal == CAL_ARMED) {
        C_pre[C_pre_n++ % CAL_PRE_N] = { (float)(now - g_cal_armed_at), A, wheel, o1, o2 };
        if (!(fabsf(o3) < 0.0015f && fabsf(o3 - L_lpo3) < 0.0008f)) g_cal_straight_since = now;
        if (now - g_cal_armed_at > p_cal_timeout) { cal_abort("no suitable stretch in time"); return 0; }
        const char *why = cal_unsuitable(kmh, o1, o2, o3, now);
        if (why) {
            g_cal_ok_since = 0;
            if (strcmp(why, g_cal_why) && now - g_cal_why_at > 5) { g_cal_why = why; g_cal_why_at = now; logv(2, "Lane calibration: waiting (%s)", why); }
            return 0;
        }
        if (!g_cal_ok_since) g_cal_ok_since = now;
        if (now - g_cal_ok_since < CAL_SETTLE_S) return 0;
        float scale = clampf(REF_GA * REF_C / fmaxf(L_gA * L_c, 0.01f), 0.5f, 1.5f);
        float apk = clampf(0.008f * (80.0f / kmh) * (80.0f / kmh) * scale, 0.004f, 0.011f);
        C_amp = clampf(apk * model_al(kmh) / model_be(kmh) / 0.85f, 0.05f, 0.5f);   // 0.85: a 0.6 s pulse gets that share of the static A through the game's integrator
        C_v = v; C_n = 0; C_peak_o1 = C_peak_o2 = 0; g_cal_t0 = now; g_cal = CAL_RUN;
        cal_status(1);
        if (p_tone > 0) play_cue(CUE_LANE_RESUME);
        disp_post(DISP_LANE, DISP_FLASH, 'T', 1.5);
        logline("Lane calibration: starting at %.0f km/h, request pulse %.2f (A peak ~%.4f), o1 %.2f m", kmh, C_amp, apk, o1);
    }
    // running: the wheel and driver tracking of lane_step, then the checks, then the pulse
    float tt = (float)(now - g_cal_t0);
    wheel_gain_step(A, wheel, dt);
    driver_activity(wheel, A, dt);
    g_law_at = now;
    const char *stop = NULL;
    float gap = g_inc.gap_m;
    const unsigned char fl = LE.lights;   // (the hook read it from the game just before)
    if (g_pause || (fl & 12)) stop = "indicator";
    else if (p_needs_cc > 0 && !(T.cruise > 0.5f)) stop = "cruise off";
    else if (g_act > 0.25f) stop = "you are steering";
    else if (T.in_brake > 0.05f || T.eff_brake > 0.05f) stop = "braking";
    else if (gap == gap && gap < 3.5f * v) stop = "a vehicle closing in";
    else if (fabsf(o3) > 0.003f) stop = "a bend";
    else if (route_ok() && route_straight_m(0.06f) >= 0 && route_straight_m(0.06f) < v * (CAL_LEN_S - tt) + 30) stop = "a bend ahead on the route";
    else if (fabsf(o1) > CAL_BAND_M || fabsf(o2) > CAL_HEAD_MAX) stop = "the truck left the inner part of the lane";
    else if (o1 == 0 && o2 == 0 && o3 == 0) stop = "the lane was lost";
    else if (kmh < LEARN_MIN_KMH - 3 || kmh > MODEL_KMH_HI + 3) stop = "the speed left the band";
    else if (!(dt > 0 && dt < 0.2f)) stop = "a hitch in the game";
    if (stop) { cal_abort(stop); return 0; }
    if (C_n < CAL_N) C_run[C_n++] = { (float)(now - g_cal_armed_at), A, wheel, o1, o2 };
    C_peak_o1 = fmaxf(C_peak_o1, fabsf(o1)); C_peak_o2 = fmaxf(C_peak_o2, fabsf(o2));
    if (tt >= CAL_LEN_S) {
        cal_finish();
        g_cal_reset = 1;   // centering comes back at full strength (g_u carries on from the last pulse value)
        return 0;
    }
    float u = 0;
    for (int k = 0; k < 4; k++) {
        float x = tt - CAL_P0[k];
        if (x >= 0 && x < CAL_PW) u += CAL_PS[k] * C_amp * 0.5f * (1 - cosf(6.2831853f * x / CAL_PW));
    }
    *out = u; g_u = u; g_lane_state = LANE_ACTIVE;
    return 1;
}

// Soft start (Damien 11-10, the ride after dev.10: "a hard jolt when I first switched it on", stay gentle until it has been in the
// middle once). That jolt: the game's own law steered the first 4 s (FFB not recognized yet) and let the truck drift to -0.9 m,
// then mode 2 took over at full strength at once (u 0.4 -> 1.0, wheel 0.01 -> 0.16 rad in 0.5 s).
static float soft_share(float o1, float o2, float kmh, double now)
{
    if (g_soft_at < 0) return 1;
    if (g_soft_end < 0) {
        if (fabsf(o1 - g_bias - g_ref) < 0.25f && fabsf(o2) < 0.012f) { if (g_centre_since < 0) g_centre_since = now; }
        else g_centre_since = -1;
        if ((g_centre_since >= 0 && now - g_centre_since > 0.5) || now - g_soft_at > 20) {
            g_soft_end = now;
            logv(2, "Lane: soft start over after %.1f s (%s)", now - g_soft_at, now - g_soft_at > 20 ? "20 s" : "near the center");
        }
    }
    float k = p_soft * (float)fmin(1.0, (now - g_soft_at) / 1.0);
    if (g_soft_end >= 0) {
        k = p_soft + (1 - p_soft) * (float)fmin(1.0, (now - g_soft_end) / 2.0);
        if (k >= 1) { g_soft_at = g_soft_end = -1; return 1; }
    }
    // heading for the line (where it is 1.5 s from now, o1' = -0.88 v o2, beyond 0.8 -> 1.4 m): full strength, ramped in over 0.5 s
    // (sim: soft 35% let a truck switched on at -0.9 m and drifting out reach 1.8 m)
    float ahead = o1 - 0.88f * (kmh / 3.6f) * o2 * 1.5f;
    float urgent = clampf((fmaxf(fabsf(o1), fabsf(ahead)) - 0.8f) / 0.6f, 0, 1) * (float)fmin(1.0, (now - g_soft_at) / 0.5);
    if (urgent > 0.5f && g_soft_end < 0) {   // once it had to act firmly the soft phase is over (dropping back to 35% halfway: 0.3 m overshoot)
        g_soft_end = now - 2;
        logv(2, "Lane: soft start over after %.1f s (heading for the line)", now - g_soft_at);
    }
    return fmaxf(k, urgent);
}

