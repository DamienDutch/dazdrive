// Lane: a calmer lane keeping assist (0.3 concept).
//
// Why the stock LKA sways with a force-feedback wheel (lka-roll-re notes A):
// the control law 0x140995f80 writes u (game_actor+0x1d5c) from the lane
// offsets o1..o3; with FFB the assist is not added to the steering angle but
// becomes a position target for the wheel motor (wheel + A, A = a leaky
// integrator of u in the vehicle). Chain of integrators under a P law = the
// oscillation. Gains are fixed floats (2.78x higher with FFB).
//
// Mode 2 (11-10, lka-sysid notes): law designed on a model fitted to the real rides (offset + heading + bend feed-forward
// + feedback on A). Modes 0/1 below are older.
// Built: a call-site hook on the control law. mode 0 = measure only: lane.csv
// with o1..o3, u, A, steering and wheel at 20 Hz while LKA is active, so a
// short ride shows signs, units and the sway period. mode 1 = calm: u scaled
// (gain) and rate-limited, small values dropped (dead zone gives the wheel
// back). Next, after the measuring ride: damping from d(o1)/dt and a clamp /
// faster leak on A (store hook at 0x140625c20).
#define LOG_BOOST 1  // experimental: more detail in the log by default
#include "../core/suite.h"
#include "../core/game.h"
#include "../core/route.h"
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

static float *g_minp, p_on_kmh, p_a_max, p_a_leak, p_min_kmh, p_mode, p_gain, p_dead, p_tone;
static float p_runaway, p_bend_out, p_bias, p_blend, p_blend_max, p_f_pull, p_follow, p_f_act, p_f_hold, p_f_win, p_f_max, p_f_tau, p_f_ret, p_f_rate, p_f_back, p_f_bend;   // shared steering
static float p_edge_look, p_edge_from, p_edge_full, p_backoff, p_drv_rate, p_drv_follow, p_drv_attack, p_drv_release, p_drv_leak;
static float p_needs_cc, p_cc_lka, p_rb_off, p_umax, p_ind_pause, p_ind_resume, p_ind_off, p_keep_on, p_nonffb, p_slow_resume;
static float p_sway, p_sw_cut, p_sw_rec, p_sw_amp;
static float p_learn, p_l_rate, p_l_min, p_l_max, p_l_update, p_l_save;
static float p_head_ref, p_k_off, p_k_head, p_k_afb, p_bend_a, p_edge_boost, p_rate2, p_i2, p_i2max;
static float p_wref, p_bend_hold, p_unwind_s, p_bend_lead, p_soft, p_bend_lat;
static float p_control, p_cal_hold, p_cal_timeout;

static param g_params[] = {
    { "mode", &p_mode, 2, "lane assist: 0 = the game's own law (the plugin only measures it, in lane.csv), 2 = model-based law fitted on real rides: offset + heading + bend feed-forward + assist feedback (1 is treated as 2)" },
    { "offset_gain", &p_k_off, 0.009f, "mode 2: assist offset A asked per meter of offset from the lane center (the game's own law: 0.015 per m at 80 km/h)" },
    { "heading_gain", &p_k_head, 0.55f, "mode 2: assist offset A asked per unit of heading error o2 (the game's own: 0.24). This is the damping: do1/dt = -0.88 v o2, so heading is the lead that the ~0.5 s of lag in game integrator + wheel needs" },
    { "assist_feedback", &p_k_afb, 40.0f, "mode 2: request per unit of error in A (fast loop around the game's integrator: its time constant goes from 0.26 s to ~0.1 s). 0 = off" },
    { "bend_assist", &p_bend_a, 2.9f, "mode 2: assist offset A per unit of curvature term o3 that the bend feed-forward asks for (a bend needs 4.4 when you do not steer; 2.9 = 65%, the offset term and I do the rest; you steering yourself in the bend adds to it)" },
    { "edge_boost", &p_edge_boost, 1.0f, "mode 2: extra stiffness of the offset term between edge_from and edge_full (1 = twice as stiff at edge_full): firm when drifting" },
    { "i_gain", &p_i2, 0.0010f, "mode 2: slow correction: assist offset A per meter of offset per second (steady offset, also in long bends)" },
    { "i_max", &p_i2max, 0.004f, "mode 2: the slow correction never gets bigger than this assist offset A" },
    { "wheel_gain_ref", &p_wref, 1.3f, "mode 2: how far the wheel turns per unit of assist offset A that the gains are tuned for (typically 1.2-1.5). The wheel's real answer is measured live (it can be up to 3x in a sharp bend) and every A term is scaled down by this / the live value (never up, never below 0.35)" },
    { "bend_max_lat", &p_bend_lat, 2.5f, "mode 2: the bend feed-forward never asks for more than a bend of this sideways acceleration (m/s2) at the speed you drive: a road does not bend sharper than that at highway speed, so a lane term that jumps there is a shifting or splitting lane, not a bend (seen at 94 km/h on a straight: the lane term flipped sign within a second and the truck swung 1.5 m). 0 = off" },
    { "bend_lead_s", &p_bend_lead, 0.35f, "mode 2: the bend feed-forward looks this many seconds further ahead than the game's lane term (it sees ~0.25 s ahead, the wheel follows ~0.45 s late): o3 + this x its rate of change, so the wheel is not left short in a sharp bend that comes on quickly. 0 = off" },
    { "bend_hold", &p_bend_hold, 0.5f, "mode 2: in a bend the assist always keeps at least this share of the bend feed-forward, and it does not unwind while the wheel is under this share of what the bend needs (so the wheel does not unwind to 0 in a sharp bend). 0 = off" },
    { "unwind_s", &p_unwind_s, 0.8f, "mode 2: in a bend the wanted assist offset unwinds (towards the outside) at most this fast: seconds from the full bend feed-forward to 0. 0 = no limit" },
    { "soft_start", &p_soft, 0.35f, "mode 2: right after lane keeping comes on (or finds its lane again) it steers with only this share, ramped in over 1 s, until the truck has been near the center for 0.5 s (or 20 s), then grows to full over 2 s; heading for the line (1.5 s ahead beyond 0.8 -> 1.4 m) it is full anyway. Avoids a hard jolt when switching it on; 0 = off" },
    { "heading_ref_kmh", &p_head_ref, 60, "mode 2: above this speed the heading gain shrinks with 1/speed (at most to half)" },
    { "rate2", &p_rate2, 12.0f, "mode 2: how fast the request may change (per second; the assist feedback needs it fast)" },
    { "anti_sway", &p_sway, 1, "mode 2: 1 = a live sway detector: o1 swinging to and fro around the center (period 1-6 s, not dying out) cuts the offset and heading gains at once and gives them back slowly; the learner takes it as 'too much gain'" },
    { "sway_cut", &p_sw_cut, 0.65f, "anti-sway: share of the gains that is left after each detected sway (0.65 = cut by 35%, twice = 42%; never below 0.4)" },
    { "sway_recover_s", &p_sw_rec, 25.0f, "anti-sway: seconds to give the cut back once it is calm (it waits 5 s first)" },
    { "sway_amp", &p_sw_amp, 0.15f, "anti-sway: swings (peak to peak) smaller than 1.5x this many meters do not count" },
    { "learn", &p_learn, 1, "mode 2: 1 = the law learns how your wheel responds (strength, damping, delay) and scales its gains to keep the loop calm; stored per wheel in steer_profile.txt. 0 = the fixed gains" },
    { "learn_rate", &p_l_rate, 0.4f, "learning: share of the way to the estimated best gain scale taken per update (0.1 = slow, 1 = at once)" },
    { "learn_min", &p_l_min, 0.7f, "learning: the gains are never scaled below this (x the ini gains)" },
    { "learn_max", &p_l_max, 1.0f, "learning: and never above this: 1 = the learner may only soften the gains (a scale of 1.27 on a strong real wheel rang in an S-bend at 85 km/h)" },
    { "learn_update_s", &p_l_update, 30.0f, "learning: seconds of usable driving (hands off, straight-ish, fast enough) between two profile updates" },
    { "learn_save_s", &p_l_save, 240.0f, "learning: the profile file is written at most this often (and when the game closes)" },
    { "assist_max", &p_a_max, 0.12f, "the assist's wheel offset never grows past this (the game lets it build up to 1)" },
    { "assist_leak", &p_a_leak, 0.005f, "share of the assist offset that leaks away every physics step (stops the build-up)" },
    { "gain", &p_gain, 1.0f, "share of the game's steering request" },
    { "dead_zone", &p_dead, 0.01f, "requests smaller than this become 0 (the wheel is yours)" },
    { "u_max", &p_umax, 1.0f, "the request never gets bigger than this (the game's own maximum is 1)" },
    { "driver_backoff", &p_backoff, 1.0f, "share of the assist that is left while you steer yourself (1 = no backing off, the default: the assist should not care whether you hold the wheel)" },
    { "edge_from", &p_edge_from, 0.3f, "offset from the lane center (m) where the assist starts to grow (like a VW: gentle in the middle, firmer towards the line)" },
    { "edge_full", &p_edge_full, 0.9f, "and the offset (m) where it is at full strength" },
    { "edge_look_s", &p_edge_look, 1.5f, "the strength follows where you will be this many seconds ahead (drifting off: wheel wrong for the bend, hands off) and not only where you are" },
    { "driver_rate", &p_drv_rate, 0.06f, "wheel speed (wheel range per second) at which you count as fully steering; a third of it is ignored (hand noise)" },
    { "driver_follow", &p_drv_follow, 3.0f, "wheel speed the assist itself may cause per unit of assist offset (same direction as the assist; not counted as you steering)" },
    { "driver_attack", &p_drv_attack, 0.15f, "seconds until the assist is backed off when you start steering" },
    { "driver_release", &p_drv_release, 1.0f, "seconds until the assist is back at full strength after you let go" },
    { "follow", &p_follow, 0, "mode 2: 1 = shared steering: an offset you hold with the wheel (passing a cyclist, a parked car) becomes the aim for a while, the assist then keeps damping and steering the bends but stops pulling back to the lane center; 0 = off" },
    { "lane_bias_m", &p_bias, 0, "mode 2: the assist aims this far to the RIGHT of the lane center (m), like a driver who keeps right (0 = the center). It builds up slowly and fades out in bends (the truck cuts those inside by itself) and is gone in a sharp one" },
    { "runaway_guard", &p_runaway, 1, "mode 2: 1 = a safety net: when the assist runs away (the request hits its ceiling three times in alternating directions within 3.5 s, or the heading error goes over 0.18 rad above 40 km/h) the game's own assist takes over for 4 s, with a tone, then ours comes back softly; 0 = off" },
    { "lane_bend_out_m", &p_bend_out, 0, "mode 2: in a bend the assist aims this far to the OUTSIDE of the bend (m): the tractor swings wide so the trailer has room (a right bend: to the left of the lane center, a left bend: to the right). It comes with the bend (o3), instead of the keep-right bias" },
    { "bend_blend", &p_blend, 0, "shared steering in a bend: 1 = while you steer into the bend yourself the assist leaves its own bend steering out for the part you already do, and builds it back within 0.3 s when you let go; 0 = the assist always adds its full bend steering (can cut the bend inside when you steer too)" },
    { "bend_blend_max", &p_blend_max, 0.6f, "bend_blend: the most of the assist's own bend steering it leaves out (0-1)" },
    { "follow_pull", &p_f_pull, 0.002f, "shared steering: the assist must pull the wheel at least this much (wheel range) for a still hand against it to show; a gentle offset hardly moves the wheel, so steering speed alone does not see it" },
    { "follow_act", &p_f_act, 0.3f, "shared steering: how much steering (0-1) counts as you holding a position" },
    { "follow_hold_s", &p_f_hold, 0.4f, "shared steering: the offset must stay steady (not drifting) this long before it is taken over" },
    { "follow_window_s", &p_f_win, 3.0f, "shared steering: the aim follows your offset only within this many seconds after you last steered" },
    { "follow_max_m", &p_f_max, 0.6f, "shared steering: the most of an offset it takes over (m)" },
    { "follow_tau_s", &p_f_tau, 0.5f, "shared steering: seconds in which the aim moves to your offset" },
    { "return_tau_s", &p_f_ret, 6.0f, "shared steering: seconds in which the aim goes back to the lane center after you let go" },
    { "return_rate", &p_f_rate, 0.1f, "shared steering: the aim never goes back faster than this (m/s)" },
    { "follow_backoff", &p_f_back, 0.3f, "shared steering: share of the offset pull left while you are steering (the heading damping and the bend steering stay at 100%)" },
    { "follow_bend_o3", &p_f_bend, 0.004f, "shared steering: no taking over of an offset in a bend sharper than this (|o3|): steering there is the bend, not a position" },
    { "driver_leak", &p_drv_leak, 0.02f, "extra share of the assist offset that leaks away per physics step while you steer" },
    { "min_kmh", &p_min_kmh, 20, "the assist steers from this speed (game: 55 km/h); below it the wheel is free" },
    { "slow_resume", &p_slow_resume, 0, "0 = after lane keeping went off on a slow stretch it stays off until you press its key (a switch-on by itself when the speed comes back to about 55 km/h is undone); 1 = leave that to the game" },
    { "keep_on", &p_keep_on, 1, "1 = the game never switches lane keeping off for low speed: it can be switched on standing still and waits until min_kmh (mode 2 gives no request below it). 0 = off below min_kmh, back on from on_kmh" },
    { "on_kmh", &p_on_kmh, 39, "keep_on = 0: after dropping below min_kmh it only comes back from this speed" },
    { "indicator_pause", &p_ind_pause, 1, "1 = an indicator (one side, not the hazards) pauses the assist: the wheel is yours for the lane change; after the indicator goes off and a lane is found again (or 4 s) it comes back gently over 1 s. 0 = only no steering back against the indicator" },
    { "indicator_off", &p_ind_off, 1, "with indicator_pause: the indicator also switches the game's lane keeping itself off (silently), and it is switched on again after the lane change. Not when you switch it back on yourself meanwhile" },
    { "needs_cruise", &p_needs_cc, 1, "1 = lane centering steers only while the cruise control is on; without the cruise only the game's warning stays. 0 = also without the cruise" },
    { "cruise_switches_on", &p_cc_lka, 0, "with needs_cruise: 1 = switching the cruise on (also Resume) switches lane keeping on (like a travel assist); 0 = the cruise leaves lane keeping as it is (roundabout_off still switches it back on at the Resume after a roundabout)" },
    { "roundabout_off", &p_rb_off, 1, "1 = when the cruise control cancels itself for a roundabout, lane keeping goes off with it (a roundabout is yours to steer), and comes back on when the cruise is resumed (within 90 s). Junctions: use your indicator" },
    { "indicator_resume", &p_ind_resume, 0, "with indicator_pause: lane keeping that went off during the lane change (above min_kmh) is switched on again afterwards (sets the game's lane-keeping flag). Off by default: the game does not switch it off for an indicator, and with keep_on not for speed, so it would mostly undo your own key press" },
    { "non_ffb_mode", &p_nonffb, 0, "without a force-feedback wheel (keyboard, gamepad, mouse steering: the game adds the assist to the steering angle instead of turning the wheel; recognized from the steering data in ~1 s): 0 = the game's own law (mode 2 is fitted on FFB wheels), 2 = mode 2 anyway (not tuned)" },
    { "tone", &p_tone, 1, "1 = a rising tone when lane keeping is switched on, a falling one when it goes off" },
    { "control", &p_control, 1, "1 = the game's lane-keeping key tells a tap from a hold: a tap switches lane keeping on or off as always, holding it (calib_hold_s) arms the steering calibration test. Needs mode 2 and learn 1. The key is found by watching its first presses: until then a hold is a plain toggle. 0 = the key only toggles" },
    { "calib_hold_s", &p_cal_hold, 0.8f, "control: hold the lane-keeping key this long (s) to arm the steering calibration; hold it again to disarm. Lane keeping is on afterwards" },
    { "calib_timeout_s", &p_cal_timeout, 300, "control: an armed calibration that finds no suitable stretch (straight road, a lane seen, lane centering steering, nobody close ahead, a speed the model is valid at) gives up after this many seconds, with a tone" },
};

enum { GA_U = 0x1d5c, V_STEER_IN = 0x1408, V_A = 0x1420, V_STEER = 0x640 };

typedef void (*law_fn)(void *ga, float o1, float o2, float o3, float dt);
static law_fn o_law;
static FILE *g_csv;
static float g_u;
// driver-in-the-loop and centering state (reset when the assist is off)
static double g_soft_at = -1, g_soft_end = -1, g_centre_since = -1;   // soft start state (below), declared here for the runaway guard
static double g_panic_until, g_sat_t[3]; static int g_sat_dir[3];   // runaway guard: the last two ceiling hits (time, direction), and when the 4 s release ends
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

// The vehicle's assist state, steering and wheel (NAN when not readable).
static void read_veh(float *A, float *steer, float *wheel)
{
    *A = *steer = *wheel = NAN;
    void *gb = game_gearbox();
    char *veh = gb ? gb_inner(gb) : NULL;
    if (veh && mem_readable(veh + V_A, 4)) {
        *A = *(float *)(veh + V_A);
        *steer = *(float *)(veh + V_STEER);
        *wheel = *(float *)(veh + V_STEER_IN);
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
        if (g_wpred == g_wpred) r = g_w_rate * g_wexp_rate >= 0 ? fabsf(g_w_rate) - 3.5f * fabsf(g_wexp_rate) : fabsf(g_w_rate);
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
static float model_be(float kmh) { return 0.235f - 0.0010f * clampf(kmh, MODEL_KMH_LO, MODEL_KMH_HI); }
static float model_al(float kmh) { return 2.6f + 0.016f * clampf(kmh, MODEL_KMH_LO, MODEL_KMH_HI); }


// ---------------------------------------------------------------------------------------------------------------
// Self-learning steering profile (lka-learn notes). Identifies the wheel loop online on usable stretches only
// (hands off, fast enough, near-straight, steady) and scales the mode-2 gains to keep the loop gain at the hand-tuned
// ts4n value: wheel = gA x A(t - delay), heading rate = -c v wheel. The reference (REF_*) is what THIS estimator reads when it replays
// the ts4n hands-free ride (gA 0.98, delay 0.35 s, c 0.168: the calm small-signal values, lower than the 1.34/0.2/0.155 large-signal fit
// because the FFB wheel has a dead band), so the hand-tuned gains stay as they are on that wheel.
// Safety: bounded scales, a rollback when the score gets worse, a ring guard, a fallback to the fixed gains.
static void sway_reset();
static int sway_active();
enum { LK_NK = 19, LK_RING = 160, LK_ZC = 8 };  // delays 0..0.9 s (pre.6 ride: pinned at the old 0.5 s end for 95% of the time)
static const float REF_GA = 0.98f, REF_C = 0.168f, REF_DELAY = 0.35f, LK_D = 0.78f, LK_TICK = 0.05f;
struct lprof { float s, dfac, cap; };            // scale on all gains, extra offset/heading reduction for slow wheels, ring ceiling
static lprof L_cur = { 1, 1, 2 }, L_base = { 1, 1, 2 };
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

static float scale_s() { return p_learn > 0 && p_mode >= 2 ? L_cur.s : 1; }
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
    return rms >= 0.0015 && L_r2a >= 0.4f && L_r2c >= 0.08f && L_gA / REF_GA >= 0.4f && L_gA / REF_GA <= 2.5f && L_c / REF_C >= 0.4f && L_c / REF_C <= 2.5f;
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
            logv(1, "Lane learn: profile s %.2f/%.2f scored worse (%.2f m, %.1f zc/min against %.2f, %.1f), rolled back to %.2f/%.2f", L_cur.s, L_cur.dfac, rms, zc, L_b_rms, L_b_zc, L_base.s, L_base.dfac);
            L_cur = L_base; L_hold = 4; L_dirty = 1; L_rolled++;
        } else { L_base = L_cur; L_b_rms = rms; L_b_zc = zc; L_b_valid = sc_ok; }
    } else if (sc_ok) { L_b_rms = rms; L_b_zc = zc; L_b_valid = 1; }
    L_scn = L_scs = L_sc_zc = 0; L_zc_n = 0; L_since = 0;
    L_cur.cap = fminf(p_l_max, L_cur.cap * 1.02f);   // a ceiling set by a ring comes back 2% per update (was 1%: 0.70 -> 0.95 took 30 updates)
    if (L_hold > 0) { L_hold--; return; }
    if (!ok || sway_active()) return;
    // the loop gain G = gA c should stay at the ts4n fit; a slower wheel (delay) gets a calmer offset term
    float st = clampf(REF_GA * REF_C / (L_gA * L_c), p_l_min, p_l_max);
    // no change up to 25% more delay than the reference wheel. Readings over 0.55 s are the centering band (small A barely moves the
    // wheel, which reads as delay: ts5a/ts5b 0.7-0.9 s; output-error fits of the same rides give 0.30-0.45 s), so they count as 0.55
    float dt_ = clampf(1.25f * REF_DELAY / clampf(L_dl, 0.05f, 0.55f), 0.5f, 1.0f);
    float rate = clampf(p_l_rate, 0.05f, 1);
    float ns = L_cur.s * clampf(powf(st / L_cur.s, rate), 0.8f, 1.25f), nd = L_cur.dfac * clampf(powf(dt_ / L_cur.dfac, rate), 0.8f, 1.25f);
    ns = clampf(fminf(ns, L_cur.cap), p_l_min, p_l_max);
    nd = clampf(nd, 0.5f, 1.0f);
    if (fabsf(ns / L_cur.s - 1) < 0.03f && fabsf(nd / L_cur.dfac - 1) < 0.03f) return;
    L_base = L_cur; L_cur.s = ns; L_cur.dfac = nd; L_trial = 1; L_dirty = 1;
    logv(1, "Lane learn: new profile s %.2f (was %.2f), dfac %.2f (was %.2f) from gA %.2f delay %.2f c %.3f", ns, L_base.s, nd, L_base.dfac, L_gA, L_dl, L_c);
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
        float old = L_cur.s;
        L_cur.s = fmaxf(p_l_min, L_cur.s * 0.85f);
        L_cur.cap = L_cur.s;
        L_base = L_cur; L_trial = 0; L_hold = 2; L_ring_cool = L_t + 10; L_dirty = 1;
        logv(1, "Lane learn: the assist offset rings (A rms %.4f, %d crossings in 8 s): gains x%.2f -> s %.2f", sqrtf(L_xa2), n, L_cur.s / old, L_cur.s);
        incident("lane", "the lane assist rings: %d crossings in 8 s (A rms %.4f), gains lowered x%.2f", n, sqrtf(L_xa2), L_cur.s / old);
        for (auto &t : L_ar_t) t = -1e9;
    }
}


// the learner's reading of a sway episode: this wheel and truck want less gain (never counted while the driver was steering)
static void learn_sway_hit(float fac_after)
{
    if (!(p_learn > 0 && p_mode >= 2)) return;
    float old = L_cur.s;
    L_cur.s = fmaxf(fmaxf(p_l_min, 0.8f), L_cur.s * 0.9f);  // a sway episode never pushes the profile under 0.8
    L_cur.cap = L_cur.s; L_base = L_cur; L_trial = 0; L_hold = 3; L_dirty = 1;
    logv(1, "Lane learn: sway counts as too much gain: s %.2f -> %.2f (ceiling %.2f)", old, L_cur.s, L_cur.cap);
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
        good = rms >= 0.0012 && r2a >= 0.5 && r2c >= 0.05 && g / REF_GA >= 0.4 && g / REF_GA <= 2.5 && cc / REF_C >= 0.4 && cc / REF_C <= 2.5;
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
    if (!(A == A && steer == steer && wheel == wheel) || fabsf(A) < 0.005f) return;
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

static int g_lka_expect = -1;       // a lane-keeping state we set ourselves: the on/off tone stays silent for it
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
    L_usable = kmh >= LEARN_MIN_KMH && g_act < 0.08f && L_t > L_warm && L_t > L_quiet && fabsf(o3) < 0.0025f && fabsf(o3 - L_lpo3) < 0.0008f &&
               fabsf(o1) < 1.0f && fabsf(u) < 0.3f && !g_ind && fabsf(A) < 0.045f && dt < 0.2f && L_t > L_ring_cool - 6 && !sway_active();
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

// ---- per-wheel profile file (a few lines of text, never MB). Errors are logged once and otherwise ignored.
static int prof_read_all(char *buf, int cap)
{
    FILE *f = open_out("steer_profile.txt", "r");
    if (!f) return 0;
    int n = (int)fread(buf, 1, cap - 1, f);
    fclose(f);
    buf[n > 0 ? n : 0] = 0;
    return n > 0 ? n : 0;
}

static void prof_load(const char *name)
{
    snprintf(L_key, sizeof L_key, "%s", name && *name ? name : "default");
    for (char *c = L_key; *c; c++) if (!isalnum((unsigned char)*c) && !strchr("-._", *c)) *c = '_';
    L_loaded = 1;
    char buf[4096];
    int n = prof_read_all(buf, sizeof buf);
    char want[64]; snprintf(want, sizeof want, "wheel=%s ", L_key);
    for (char *ln = buf; n > 0 && *ln;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (!strncmp(ln, want, strlen(want))) {
            float s = 0, df = 0, cap = 0, ga = 0, c = 0, dl = 0; int nu = 0;
            const char *q;
            if ((q = strstr(ln, " s="))) s = (float)atof(q + 3);
            if ((q = strstr(ln, " dfac="))) df = (float)atof(q + 6);
            if ((q = strstr(ln, " cap="))) cap = (float)atof(q + 5);
            if ((q = strstr(ln, " ga="))) ga = (float)atof(q + 4);
            if ((q = strstr(ln, " c="))) c = (float)atof(q + 3);
            if ((q = strstr(ln, " delay="))) dl = (float)atof(q + 7);
            if ((q = strstr(ln, " n="))) nu = atoi(q + 3);
            bool good = s >= p_l_min && s <= p_l_max && df >= 0.5f && df <= 1.0f && cap >= p_l_min && cap <= p_l_max && s == s && df == df && cap == cap;
            if (!good) { logv(1, "Lane learn: profile for '%s' is broken or out of bounds, using the fixed gains", L_key); return; }
            L_cur.s = fminf(s, cap); L_cur.dfac = df; L_cur.cap = cap; L_base = L_cur; L_trial = 0;
            if (ga > 0.2f && c > 0.02f && dl >= 0 && dl < 1) { L_gA = ga; L_c = c; L_dl = dl; }
            // a profile exists only for a wheel mode 2 ran on, i.e. a force-feedback one: no 4 s of the game's own law first (11-10:
            // it let the truck drift to -0.9 m before mode 2 took over). The steering data still re-check it (two windows to flip).
            if (g_ffb < 0 && nu > 0) { g_ffb = 1; logline("Lane: force-feedback wheel assumed from its profile (re-checked from the steering data)"); }
            L_nupd = nu;
            logv(1, "Lane learn: profile for '%s' loaded: s %.2f dfac %.2f (gA %.2f delay %.2f c %.3f, %d updates)", L_key, L_cur.s, L_cur.dfac, L_gA, L_dl, L_c, nu);
            return;
        }
        if (!nl) break;
        ln = nl + 1;
    }
    logv(1, "Lane learn: no profile for '%s' yet, starting from the fixed gains", L_key);
}

static void prof_save(const char *why)
{
    if (!L_loaded || !(p_learn > 0) || !L_dirty) return;
    char buf[4096], out[4096]; int n = prof_read_all(buf, sizeof buf), m = 0, lines = 0;
    char want[64]; snprintf(want, sizeof want, "wheel=%s ", L_key);
    for (char *ln = buf; n > 0 && *ln && lines < 16;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (*ln && strncmp(ln, want, strlen(want)) && strncmp(ln, "wheel=", 6) == 0 && m + (int)strlen(ln) + 2 < 3500) { m += snprintf(out + m, sizeof out - m, "%s\n", ln); lines++; }
        if (!nl) break;
        ln = nl + 1;
    }
    m += snprintf(out + m, sizeof out - m, "wheel=%s v=1 s=%.4f dfac=%.4f cap=%.4f ga=%.3f c=%.4f delay=%.3f n=%d\n", L_key, L_cur.s, L_cur.dfac, L_cur.cap, L_gA, L_c, L_dl, L_nupd);
    FILE *f = open_out("steer_profile.txt", "w");
    L_save_at = wall_s();
    if (!f) { static int warned; if (!warned++) logv(1, "Lane learn: cannot write steer_profile.txt (the profile stays in memory)"); return; }
    size_t w = fwrite(out, 1, m, f);
    fclose(f);
    if (w == (size_t)m) { L_dirty = 0; logv(1, "Lane learn: profile for '%s' saved (%s): s %.2f dfac %.2f", L_key, why, L_cur.s, L_cur.dfac); }
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
    if (p_follow > 0 && fabsf(g_ref) > 0.03f) { SW_dir = 0; return; }   // while you hold an offset (shared steering) a swing is yours (13-10: two sway hits right after steering)
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
    if (SW_t - SW_lastact > 8 && learn_small_signal()) learn_sway_hit(SW_fac);
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
    float ci_k = p_i2, ci_max = p_i2max;  // in units of A
    if (g_act < 0.3f && fabsf(e1) < 1.2f) g_ci -= ci_k * e1 * dt;
    g_ci *= expf(-dt * (0.1f + 1.5f * g_act));
    g_ci = clampf(g_ci, -ci_max, ci_max);
    {
        // Model-based law (lka-sysid notes). Request per physical effect:
        //  - offset: stiffer towards the line (edge_boost), slow integral for a steady offset
        //  - heading o2: the damping; do1/dt = -0.88 v o2, so it is the lead the 0.5 s actuator lag needs
        //  - bend feed-forward: the bend needs wheel -4.4 o3 (fitted); the game only adds -0.5 v o3, ~1/4 of that
        //  - feedback on A: the game's integrator (tau 0.26 s) is closed into a fast loop
        float ahead = o1 + g_do1 * p_edge_look;
        float off = fmaxf(fabsf(o1), o1 * ahead > 0 || fabsf(o1) < 0.05f ? fabsf(ahead) : 0);
        float k = clampf((off - p_edge_from) / fmaxf(p_edge_full - p_edge_from, 0.05f), 0, 1);
        float stiff = 1 + p_edge_boost * k * k * (3 - 2 * k);
        float be = model_be(kmh), al = model_al(kmh);
        float afb = (A == A) ? p_k_afb : 0;
        // wanted assist offset A (the bend's own steering is -4.4 o3, fitted)
        // heading gain lower at speed: at 89 km/h after a bend it rang with a 1.35 s period (Damien: sways after a bend)
        float k_head = p_k_head * fminf(1.0f, fmaxf(0.5f, p_head_ref / fmaxf(kmh, 1.0f)));
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
        float ff = -p_bend_a * (p_wref > 0 ? lf * (1 + 0.25f * (1 - lf)) : ls) * o3f;
        // towards the line the offset term gets the learned / sway scales back (ts5b profile: 0.25x of it held the truck in balance
        // 1.9 m out in a sharp bend in the sim): from 0.5 m to full at edge_full. The live wheel scale li stays (that is the plant).
        float so = ls * ldf * swf, ek = clampf((fabsf(o1) - 0.5f) / fmaxf(p_edge_full - 0.5f, 0.05f), 0, 1);
        so = (so + (fmaxf(so, 1.0f) - so) * ek) * li;
        // (a Smith predictor on the heading, the wheel motion still in the pipe, was tried 11-10: no gain in the ts5b bend and it rang
        // at 90 km/h on a slow wheel, delay 0.35 + lag 0.15)
        // your hand is clearly on the wheel (activity over 0.3) on a straight: less pull on the offset only. Not in a bend (there the
        // activity can be a misreading, ts5b) and not for the small activity of hand noise.
        float fb = follow_fb(o3, g_act);
        float a_cmd = -p_k_off * so * stiff * fb * e1 + (k_head * ls * sqrtf(ldf) * (0.5f + 0.5f * swf) * o2 + g_ci * ls) * li + ff;
        // Bend blend (Damien 24-10, his Polo's Travel Assist: what you already steer counts, the assist adds the rest). The wheel minus
        // where the assist alone puts it (g_dev, 0.5 s filter) is your own steering; the part of it that goes into the bend, over a dead
        // zone of a quarter of the need, takes that share off the assist's bend steering (never more than bend_blend_max). It builds
        // back within 0.3 s when you let go. Not on a straight, not slower than 30 km/h, not when the wheel gain is unknown.
        {
            float need_b = 4.6f * fabsf(o3), sbw = o3 > 0 ? -1.0f : 1.0f;
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
        float sb = o3 > 0 ? -1.0f : 1.0f, need = 4.6f * fabsf(o3);   // bend direction of A / wheel, the wheel the bend needs (fitted)
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
        // needs several swings and was far too late). Two ceiling hits in opposite directions within 2.5 s, or a heading error over 0.12 rad
        // fast: let go for 4 s and come back softly (the soft start).
        if (p_runaway > 0 && kmh > 40) {
            double nowp = wall_s();
            if (fabsf(want2) > 0.95f) {
                int d = want2 > 0 ? 1 : -1;
                if (g_sat_dir[0] != d || nowp - g_sat_t[0] > 0.3) {
                    g_sat_dir[2] = g_sat_dir[1]; g_sat_t[2] = g_sat_t[1]; g_sat_dir[1] = g_sat_dir[0]; g_sat_t[1] = g_sat_t[0];
                    g_sat_dir[0] = d; g_sat_t[0] = nowp;
                } else g_sat_t[0] = nowp;
            }
            // three ceiling hits that alternate (+ - +) within 3.5 s: a swing, not one hard correction (two alternating hits fired on recoverable bends)
            bool flip = g_sat_dir[2] != 0 && g_sat_dir[0] != g_sat_dir[1] && g_sat_dir[1] != g_sat_dir[2] && nowp - g_sat_t[2] < 3.5;
            if ((flip || fabsf(o2) > 0.18f) && nowp >= g_panic_until) {
                g_panic_until = nowp + 4;
                g_sat_dir[0] = g_sat_dir[1] = g_sat_dir[2] = 0;
                g_soft_at = nowp + 4;   // the soft start after it
                play_cue(CUE_LANE_PAUSE);
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
enum { CAL_IDLE, CAL_ARMED, CAL_RUN };
static int g_cal, g_cal_reset;   // g_cal_reset: the physics thread resets the centering state at its next tick (a stop from the frame must not touch it)
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

// 0 = suitable now; else the reason it is not. The same gates as the learner's usable stretch (L_usable: hands off, quiet 3 s, straight
// and steady lane term, |o1| < 1, no indicator, small A, speed from LEARN_MIN_KMH) plus what a test needs on top.
static const char *cal_unsuitable(float kmh, float o1, float o2, double now)
{
    float v = kmh / 3.6f;
    if (g_pause || g_ind) return "indicator";
    if (p_needs_cc > 0 && !(T.cruise > 0.5f)) return "cruise off: lane centering does not steer";
    if (now < g_panic_until || g_soft_at >= 0) return "lane centering is still settling in";
    if (kmh < fmaxf(LEARN_MIN_KMH, p_min_kmh) || kmh > MODEL_KMH_HI) return "speed outside the band the model holds for";
    if (!L_usable) return "not a quiet straight stretch (steering, bend, small start-up)";
    if (T.in_brake > 0.02f || T.eff_brake > 0.02f) return "braking";
    float gap = g_inc.gap_m;
    if (gap == gap && gap < 6.0f * v) return "a vehicle ahead";
    if (!(g_lane_ok_at > g_lane_bad_at && now - g_lane_bad_at > 5)) return "lane not seen steadily";
    if (fabsf(o1) > CAL_START_M || fabsf(o2) > CAL_START_HEAD || fabsf(g_do1) > 0.08f) return "not centered and settled";
    float need = v * (CAL_LEN_S + 1) + 50;
    float st = route_ok() ? route_straight_m(0.06f) : -1;
    if (st >= 0) { if (st < need) return "a bend ahead on the route"; }
    else if (!(now - g_cal_straight_since > 10)) return "no route and under 10 s of straight lane";
    return NULL;
}

static void cal_arm(double now)
{
    g_cal = CAL_ARMED; g_cal_armed_at = now; g_cal_ok_since = 0; g_cal_straight_since = now; C_pre_n = 0; g_cal_why = ""; g_cal_why_at = 0;
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
    return r->rms >= 0.001f && r->r2a >= 0.6f && r->gA / REF_GA >= 0.4f && r->gA / REF_GA <= 2.5f && r->dl >= 0 && r->dl <= dmax - 0.05f;
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
    float st = clampf(REF_GA * REF_C / (r.gA * r.c), p_l_min, p_l_max);
    float nd = clampf(1.25f * REF_DELAY / clampf(r.dl, 0.05f, 0.55f), 0.5f, 1.0f);
    float ns = clampf(fminf(st, L_cur.cap), p_l_min, p_l_max);
    L_base = L_cur; L_cur.s = ns; L_cur.dfac = nd; L_trial = 1; L_dirty = 1; L_hold = 2;
    L_gA = r.gA; L_dl = r.dl; L_c = r.c;
    for (int k = 0; k < LK_NK; k++) L_Sxx[k] = L_Sxy[k] = 0;   // the slow estimate starts over from here
    L_Syy = L_Nw = L_Cxx = L_Cxy = L_Cyy = 0; L_dev_blocks = 0; learn_block_reset();
    if (p_tone > 0) play_cue(CUE_LANE_CAL_DONE);
    disp_post(DISP_LANE, DISP_FLASH, 'L', 1.5);
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
        const char *why = cal_unsuitable(kmh, o1, o2, now);
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
    unsigned char fl = *(unsigned char *)(game_actor_safe() + 0x1d0);
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

static void stub_law(void *ga, float o1, float o2, float o3, float dt)
{
    char *g = (char *)ga;
    float before = g ? *(float *)(g + GA_U) : 0;
    o_law(ga, o1, o2, o3, dt);
    if (!g) return;
    g_stub_at = wall_s();
    float u = *(float *)(g + GA_U), out = u;
    float A, steer, wheel;
    read_veh(&A, &steer, &wheel);
    // Lane lost (o1 = o2 = o3 = 0, e.g. an exit or lane change): the game's request stays where it was
    // (ts4g: -0.88 for 20 s, the wheel held to one side). Bring it to 0 and let filter_a release the wheel.
    int lane_ok = !(o1 == 0 && o2 == 0 && o3 == 0) && fabsf(o1) < 0.9f;
    if (lane_ok) { if (g_lane_bad_at >= g_lane_ok_at) g_lane_ok_at = wall_s(); } else g_lane_bad_at = wall_s();
    // Below min_kmh (keep_on: the game keeps lane keeping on at any speed): no request, the wheel is free; back
    // above it (+1 km/h hysteresis), ramped in like after an indicator pause.
    static int slow;
    float kmh_now = T.speed * 3.6f;  // reversing counts as slow (the law is fitted driving forward)
    int was_slow = slow;
    slow = p_keep_on > 0 && (slow ? kmh_now < p_min_kmh + 1 : kmh_now < p_min_kmh);
    if (was_slow && !slow) g_resume_at = wall_s();
    // Indicator pause: no request at all (the game's u decays to 0 in a few frames), short-term state forgotten.
    if (p_mode >= 1 && (g_pause || slow)) {
        if (g_cal == CAL_RUN) cal_abort(g_pause ? "indicator" : "too slow");
        g_lane_state = g_pause ? LANE_WAIT_INDICATOR : LANE_WAIT_SLOW;
        float d = before * 0.85f;
        *(float *)(g + GA_U) = fabsf(d) < 1e-3f ? 0 : d;
        g_u = 0; g_ci = 0; g_ref = 0; g_have_o = 0;
        return;
    }
    if (p_mode >= 1 && o1 == 0 && o2 == 0 && o3 == 0) {
        if (g_cal == CAL_RUN) cal_abort("the lane was lost");
        g_lane_state = LANE_WAIT_NO_LANE;
        float d = before * 0.9f;
        *(float *)(g + GA_U) = fabsf(d) < 1e-3f ? 0 : d;
        g_u = 0;
        return;
    }
    if (p_mode >= 1) ffb_detect(A, steer, wheel);
    // the law ran: it wrote a new u. Saturated it writes exactly +-1, the same as our clamped output, so u == before then too
    // (ts5b 387.4 s: skipped while saturated -> filter_a took the law for off and released the wheel in the bend). It leaves u
    // unchanged for |o1| > 1.95 (lka-roll-re notes A2) and, presumably, when lane keeping is not active: so only with the
    // lane-keeping bit on and our law running a moment ago (never keep a stale +-1 going after it went off).
    int ran = u != before || (fabsf(u) >= 0.999f && fabsf(o1) <= 1.95f && (*(unsigned char *)(g + 0x1d1) & 0x20) && wall_s() - g_law_at < 0.15);
    if (law2() && ran) {
        g_ind = (*(unsigned char *)(g + 0x1d0) & 12) != 0;
        if (p_soft > 0 && p_soft < 1 && wall_s() - g_law_at > 0.5) {   // our law did not run for 0.5 s: switched on / lane found again
            g_soft_at = wall_s(); g_soft_end = g_centre_since = -1;
            logv(2, "Lane: soft start (%.0f%% until near the center), o1 %.2f m", p_soft * 100, o1);
        }
        g_lane_state = LANE_ACTIVE;   // the runaway guard in lane_step overrides it
        float cal_out = 0;
        if (g_cal_reset) { g_cal_reset = 0; lane_reset(); }
        int cal = g_cal != CAL_IDLE && cal_tick(o1, o2, o3, dt, wheel, A, fabsf(T.speed) * 3.6f, &cal_out);   // the steering calibration owns the request
        if (g_cal_reset) { g_cal_reset = 0; lane_reset(); }
        out = cal ? cal_out : lane_step(u, o1, o2, o3, dt, wheel, A, fabsf(T.speed) * 3.6f);
        if (g_soft_at >= 0 && !cal) { out *= soft_share(o1, o2, fabsf(T.speed) * 3.6f, wall_s()); g_u = out; }
        // Indicator set (the game's own clamp, lka-roll-re notes A2): the assist may not steer back against a lane change.
        // Mode 2 is much firmer than the game's law, so it must keep this rule too.
        unsigned char fl = *(unsigned char *)(g + 0x1d0);
        if (p_mode >= 2) { if (fl & 4) out = fmaxf(out, 0); if (fl & 8) out = fminf(out, 0); }
        // back from an indicator pause: ramp in over 1 s (no jerk on the wheel in the new lane)
        double since = wall_s() - g_resume_at;
        if (p_mode >= 2 && since >= 0 && since < 1) { out *= (float)since; g_u = out; }
        *(float *)(g + GA_U) = out;
        if (!g_engaged && g_act > 0.5f) {
            g_engaged = 1;
            logline("Lane: you are steering (wheel %.3f), assist backs off to %.0f%%", wheel, g_scale * 100);
        }
    } else if (p_mode >= 1 && !law2()) {
        // the game's own law (no FFB / profile not known yet): the same ramp after a pause or the low-speed gate
        if (ran) g_lane_state = LANE_ACTIVE;
        double since = wall_s() - g_resume_at;
        if (since >= 0 && since < 1) { out = u * (float)since; *(float *)(g + GA_U) = out; }
    }
    // needs_cruise with the cruise off: the request is computed but filter_a lets the wheel target go (the same test as there): waiting
    if (p_mode >= 1 && p_needs_cc > 0 && !(T.cruise > 0.5f) && g_lane_state == LANE_ACTIVE) g_lane_state = LANE_WAIT_NO_CRUISE;
    if (u == 0) lane_reset();  // the game switched the assist off: let go at once
    if (u == 0) { out = 0; g_lane_state = LANE_OFF; }
    g_u = out;
    if (!g_csv || u == 0) return;
    static double at;
    double now = wall_s();
    if (now - at < 0.05) return;
    at = now;
    fprintf(g_csv, "%.2f,%.1f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.2f,%.2f,%.2f,%.2f,%d,%.3f,%.3f,%.3f\n", now, fabsf(T.speed) * 3.6f, o1,
            o2, o3, u, out, A, steer, wheel, g_act, g_scale, g_ci, g_damp_u, g_ff, scale_s(), scale_df(), L_gA, L_dl, L_c, p_sway > 0 ? SW_fac : 1.0f, L_usable,
            g_wg, g_wi, g_lf, g_li, g_hold, g_ref, g_share, g_bias);
}

// The vehicle's assist state A (veh+0x1420): with an FFB wheel it is the
// wheel's target lead, and the game integrates u into it -> it builds up and
// the wheel sways. Calm mode keeps it small and lets it leak (faster while you steer).
static float filter_a(float a)
{
    // Lane keeping just went off (key, too slow, lines lost): the control law is no longer called, but the
    // wheel target A stayed and held the wheel to one side for ~5 s (Damien 11-10). Let it go in ~0.3 s.
    if (p_mode >= 1 && wall_s() - g_law_at > 0.15) {
        g_u = 0;
        g_ci = 0;
        return fabsf(a) < 1e-4f ? 0 : a * 0.94f;
    }
    if (p_mode >= 1 && (g_pause || (p_needs_cc > 0 && !(T.cruise > 0.5f)) || (p_keep_on > 0 && T.speed * 3.6f < p_min_kmh))) return fabsf(a) < 1e-4f ? 0 : a * 0.94f;  // indicator pause / too slow: release the wheel in ~0.3 s
    if (p_mode < 1 || g_u == 0 || !law2()) return a;
    a *= 1 - clampf(p_a_leak + g_act * p_drv_leak, 0, 0.5f);
    return clampf(a, -p_a_max, p_a_max);
}

static char g_status[160] = "off";
static const char *ln_status() { return g_status; }

// Continuity broke (pause menu, hitch, other truck, teleport, ...): the offsets in the filters belong to another place.
// The control law runs on the game's physics step with its own dt; only our state must not carry over.
static void ln_reset(int level, const char *why)
{
    if (g_cal == CAL_RUN) cal_abort("the game was paused or moved");
    lane_reset();
    g_u = g_ci = g_do1 = g_o1_prev = g_o2_prev = g_w_prev = 0;
    g_law_at = -1e9;  // filter_a lets the assist offset go until the law runs again
    g_lane_state = LANE_OFF;
}

static void ln_start()
{
    if (p_mode > 0 && p_mode < 2) { logline("Lane: mode 1 no longer exists, using mode 2"); p_mode = 2; }
    // The LKA call-site pattern carries no struct offsets, so it can match on another build (it does on ATS 1.61) while
    // the fields the law reads and writes (GA_U and friends) are 1.60's: hook only on the 1.60 layout, like creep.
    snprintf(g_status, sizeof g_status, "off (not the 1.60 layout)");
    if (!L || strcmp(L->name, "1.60")) { logline("Lane: only for the 1.60 layout, lane assist stays the game's"); return; }
    snprintf(g_status, sizeof g_status, "off (LKA code not found: 1.60 only)");
    uintptr_t h = game_scan("41 0F 28 DF F3 44 0F 11 64 24 20 41 0F 28 D6 41 0F 28 CD 48 8B CB E8");
    uintptr_t site = h ? h + 0x16 : 0;
    if (!site) { logline("Lane: LKA control law call not found (1.60 only), lane assist stays the game's"); return; }
    o_law = (law_fn)(site + 5 + (intptr_t) * (const int32_t *)(site + 1));
    if (!game_patch_call(site, (void *)stub_law)) { logline("Lane: hook failed"); return; }
    // The control law switches itself off below 55 km/h (comiss against a
    // pooled 55.0): point that one load at our own value.
    if (p_mode < 1) p_keep_on = 0;  // the game's own law (mode 0) keeps its own speed limits: untested at a standstill
    if ((p_min_kmh > 0 && p_min_kmh < 55) || p_keep_on > 0) {
        uintptr_t m = game_scan("F3 0F 59 C7 0F 54 C6 0F 2F 05 ?? ?? ?? ?? 73 5E 8B 83 14 01 00 00");
        if (m && *(const float *)(m + 14 + *(const int32_t *)(m + 10)) == 55.0f &&
            (g_minp = game_patch_rip_float(m + 7, 3, 7, p_keep_on > 0 ? 0.0f : fmaxf(p_on_kmh, p_min_kmh))) != NULL) {
            if (p_keep_on > 0) logline("Lane: lane keeping stays on at any speed (also switched on standing still), steers from %.0f km/h", p_min_kmh);
            else logline("Lane: lane keeping from %.0f km/h, down to %.0f", fmaxf(p_on_kmh, p_min_kmh), p_min_kmh);
        }
        else logline("Lane: minimum speed code not found, stays 55 km/h");
    }
    if (p_mode >= 1) {
        uintptr_t st = game_scan("F3 44 0F 11 87 20 14 00 00 84 DB 75 04");
        if (st && game_patch_store_xmm8(st, filter_a)) logline("Lane: assist build-up limited (max %.2f, leak %.2f)", p_a_max, p_a_leak);
        else logline("Lane: assist store not found, only the request is calmed");
    }
    g_csv = open_out("lane.csv", "w");
    if (g_csv) fputs("t,kmh,o1,o2,o3,u_game,u_out,A,steer,wheel,activity,scale,ci,damp,ff,l_s,l_dfac,l_gA,l_delay,l_c,sway_fac,l_usable,w_ratio,w_slope,lf,li,bend_hold,ref,share,bias\n", g_csv);
    snprintf(g_status, sizeof g_status, "on, hook in place, mode %g (%s)", p_mode, p_mode >= 2 ? "model-based" : "the game's own law, measuring only");
    logline("Lane: hook in place, mode %g (%s)", p_mode, p_mode >= 2 ? "model-based" : "measure only");
}

static void ln_stop()
{
    prof_save("shutdown");
    if (g_csv) fflush(g_csv);
}

// Indicator pause: one indicator on = a lane change on purpose -> our assist lets
// go of the wheel; indicator off -> back once a lane is found near its center (or after 4 s), ramped in. Lane keeping
// that went off meanwhile (above min_kmh) is switched on again: game_actor+0x1d1 bit 0x20 (lka-roll-re notes A2).
static void lka_set(char *ga, int on)
{
    g_lka_expect = on;
    if (on) __atomic_fetch_or((unsigned char *)(ga + 0x1d1), (unsigned char)0x20, __ATOMIC_SEQ_CST);  // the game thread writes this byte too
    else __atomic_fetch_and((unsigned char *)(ga + 0x1d1), (unsigned char)~0x20, __ATOMIC_SEQ_CST);
}

static void indicator_pause()
{
    char *ga = game_actor_safe();
    if (!ga) return;
    static int was_on, we_off;    // lane keeping was on at the indicator / we switched it off (indicator_off)
    static double off_at, on_at;  // indicator off since / this indicator period on since (0 = none)
    unsigned char fl = *(unsigned char *)(ga + 0x1d0);
    int one = (fl & 12) == 4 || (fl & 12) == 8;  // both = hazards: no lane change
    int lka = (*(unsigned char *)(ga + 0x1d1) & 0x20) != 0;
    double now = wall_s();
    // The lane-keeping key pressed while we hold it off for the indicator: that means OFF (Damien 11-10), not on again.
    // Off it stays, also after the indicator; one falling tone and the X confirm it (indicator_pause runs before the tone
    // watcher in ln_frame, so it never sees the short "on").
    if (we_off && lka && g_lka_expect != 1) {
        __atomic_fetch_and((unsigned char *)(ga + 0x1d1), (unsigned char)~0x20, __ATOMIC_SEQ_CST);
        we_off = 0; lka = 0;
        if (p_tone > 0) play_cue(CUE_OFF);
        disp_post(DISP_LANE, DISP_FLASH, 'X', 1.5);
        logline("Lane: key pressed during the indicator: lane keeping off (stays off after the indicator)");
    }
    if (one) {
        off_at = 0;
        if (!on_at) on_at = now;
        // a forgotten indicator: after 20 s the assist is back (with the old rule: no steering against the indicator)
        if (now - on_at > 20) {
            if (g_pause) {
                g_pause = 0; g_resume_at = now;
                if (we_off && !lka) lka_set(ga, 1);
                we_off = 0;
                if (p_tone > 0 && was_on) play_cue(CUE_LANE_RESUME);
                logline("Lane: indicator on for 20 s: assist back (no steering against the indicator)");
            }
            return;
        }
        if (!g_pause) {
            g_pause = 1; was_on = lka;
            DEV_NOTE(ind_pause());
            lane_reset();
            if (p_ind_off > 0 && lka) { lka_set(ga, 0); we_off = 1; }
            if (p_tone > 0 && lka) play_cue(CUE_LANE_PAUSE);  // only when there was an assist to pause
            logv(2, "Lane: indicator %s: assist paused%s", (fl & 4) ? "left" : "right",
                 !lka ? " (lane keeping was off)" : we_off ? ", lane keeping switched off" : "");
        }
        return;
    }
    if (!off_at) off_at = now;
    if (now - off_at > 1) on_at = 0;  // off for a second (not a blink pause): the next indicator is a new period
    if (!g_pause) return;
    if (now - off_at < 1) return;
    // we switched lane keeping off: back on after 1 s (the game only calls the law, which sees the lane, while it is on);
    // our request stays paused until the lane is found
    if (we_off) {
        we_off = 0;
        if (!lka) {
            lka_set(ga, 1);
            g_lane_bad_at = now;  // only a lane seen from now on counts
            logv(2, "Lane: indicator off, lane keeping back on");
            return;
        }
    }
    // a lane near its center for 0.5 s (after the indicator has been off for 1 s), or 4 s at most
    int lane = g_lane_ok_at > g_lane_bad_at && now - g_lane_ok_at > 0.5;
    if (!lane && now - off_at < 4) return;
    g_pause = 0;
    g_resume_at = now;
    float kmh = fabsf(T.speed) * 3.6f;
    if (p_ind_resume > 0 && was_on && !lka && kmh > fmaxf(p_min_kmh, 5) + 2) {
        lka_set(ga, 1);
        logline("Lane: lane keeping went off during the lane change: switched on again (%.0f km/h)", kmh);
    } else logv(2, "Lane: indicator off, assist back%s", lane ? "" : " (no lane found within 4 s)");
    if (p_tone > 0 && was_on && (lka || (p_ind_resume > 0 && kmh > fmaxf(p_min_kmh, 5) + 2))) play_cue(CUE_LANE_RESUME);
    we_off = 0;
}

// Roundabout: the cruise cancelled itself for one (g_rb_cancel_at): lane keeping off with it, back on at the next cruise resume.
// Not if the driver switches lane keeping on again meanwhile (its key), nor after 90 s.
static void roundabout_off()
{
    static int we_off;
    static double off_at, seen_at = -1e9;
    char *ga = game_actor_safe();
    if (!ga) return;
    int lka = (*(unsigned char *)(ga + 0x1d1) & 0x20) != 0;
    double now = wall_s();
    if (p_rb_off > 0 && g_rb_cancel_at > seen_at) {
        seen_at = g_rb_cancel_at;
        if (lka && !g_pause) {
            lka_set(ga, 0);
            we_off = 1; off_at = now;
            disp_post(DISP_LANE, DISP_FLASH, 'X', 1.5);
            logline("Lane: cruise cancelled for a roundabout: lane keeping off, back on at the next resume");
        }
    }
    if (!we_off) return;
    if (lka || now - off_at > 90) { we_off = 0; return; }  // the driver switched it on, or too long ago
    if (now - off_at > 1 && T.cruise > 0.5f) {              // cruise back on (resume)
        float kmh = fabsf(T.speed) * 3.6f;
        we_off = 0;
        if (kmh > fmaxf(p_min_kmh, 5) + 2) {
            lka_set(ga, 1);
            g_lane_bad_at = now;
            if (p_tone > 0) play_cue(CUE_LANE_RESUME);
            logline("Lane: cruise resumed after a roundabout: lane keeping back on (%.0f km/h)", kmh);
        }
    }
}

// The cruise coming on switches lane keeping on (needs_cruise and cruise_switches_on): the lane toggle then only decides while the cruise runs
static void cruise_lane_on()
{
    static int prev_cc; static double cc_on_at;
    int cc = T.cruise > 0.5f;
    double now = wall_s();
    if (cc && !prev_cc) cc_on_at = now;
    prev_cc = cc;
    static double done_at = -1;
    if (p_needs_cc > 0 && p_cc_lka > 0 && cc && cc_on_at > 0 && now - cc_on_at > 0.3 && done_at < cc_on_at && !g_pause) {
        char *ga = game_actor_safe();
        float kmh = fabsf(T.speed) * 3.6f;
        if (!ga || kmh <= fmaxf(p_min_kmh, 5) + 2) return;   // too slow yet (a Resume in a jam): try again when the speed is there
        done_at = now;
        if (!(*(unsigned char *)(ga + 0x1d1) & 0x20)) {
            lka_set(ga, 1);
            if (p_tone > 0) play_cue(CUE_LANE_RESUME);
            logline("Lane: cruise on, lane keeping on (%.0f km/h)", kmh);
        }
    }
}

// The game calls the law only while lane keeping is on: no call for 0.15 s = off (filter_a uses the same gap for the wheel target)
static void lane_state_tick()
{
    if (g_lane_state != LANE_OFF && wall_s() - g_stub_at > 0.15) g_lane_state = LANE_OFF;
}

// ---------------------------------------------------------------------------------------------------------------
// The game's lane-keeping key, tap or hold (`control`). The game toggles lane keeping (game_actor+0x1d1 bit 0x20) on the press, so a
// hold cannot be told from a tap by the bit alone: it needs the key's own state. The game keeps one action object per key
// (game_actor + offset -> object, state at +0xf8: 2 fresh press, 1 held, 0 up: cruise-keys notes), read the way cruise.cpp reads the ACC
// distance key. The offset of the lane key's pointer is not in our notes, so it is found from the presses (key_vote): the slots next to
// the cruise keys that read non-zero just before the bit flipped, twice in a row for the same slot. Until it is found, or if it
// goes wrong, every press is a plain toggle with its tone at once, as with control 0.
//   TAP  (released before calib_hold_s): the game's toggle stands; its tone and letter come when the key is released.
//   HOLD (calib_hold_s): arms the steering calibration (cal_arm); lane keeping is on afterwards (the toggle of the press undone).
//   A later gesture on the same key (about 3 s held) belongs in key_tick next to the hold: the hold fires at calib_hold_s and
//   K_down_at keeps counting, so a longer threshold can be tested there. Not built.
enum { KEY_LO = 0x1e600, KEY_N = 96, KEY_STATE = 0xf8 };
static int g_key_off, g_key_bad;                                  // game_actor offset of the lane key's action pointer (0 = not found), gave up
static double K_nz[KEY_N], K_down_at, K_up_at, K_fire_at;         // per slot: when its state was last non-zero; the found key: held since / up since; when a hold last acted
static unsigned char K_zero[KEY_N], K_bad[KEY_N], K_votes[KEY_N], K_ok[KEY_N];
static void *K_ptr[KEY_N];                                        // the object each slot pointed to at the last look, and whether it is readable (K_ok)
static int K_miss, K_fired;                                       // toggles in a row without the key down; the hold of this press has acted
static struct { int active; double at; } K_pend;                  // a toggle whose tone waits for the key to come up (or for the hold)

static int control_on() { return p_control > 0 && p_mode >= 2 && p_learn > 0 && o_law; }

// The action state at ga+off: -1 = no action object there. slot < 0: not cached (the found key); else the readable check of the object is
// only repeated when the pointer in the slot changes (96 slots at frame rate would be a lot of VirtualQuery calls).
static int key_state(char *ga, int off, int slot = -1)
{
    if (slot < 0 && !mem_readable(ga + off, 8)) return -1;
    void *p = *(void **)(ga + off);
    if ((uintptr_t)p < 0x10000 || ((uintptr_t)p & 7)) return -1;
    if (slot >= 0) {
        if (p != K_ptr[slot]) { K_ptr[slot] = p; K_ok[slot] = mem_readable(p, KEY_STATE + 4); }
        if (!K_ok[slot]) return -1;
    } else if (!mem_readable(p, KEY_STATE + 4)) return -1;
    return *(int *)((char *)p + KEY_STATE);
}

static void key_cue(int on)   // the tone, the dashboard letter and the log line of a toggle
{
    if (p_tone > 0) play_cue(on ? CUE_ON : CUE_OFF);
    disp_post(DISP_LANE, DISP_FLASH, on ? 'L' : 'X', 1.5);  // L = lane keeping on, X = off (1.5 s on the dashboard)
    logline("Lane: lane keeping %s", on ? "on" : "off");
}

// A toggle that the driver did (not ours): which slot was held just before? Two votes in a row for the same one find the key.
static void key_vote(double now)
{
    if (g_key_off) return;
    int hit = -1, n = 0;
    for (int i = 0; i < KEY_N; i++) if (!K_bad[i] && K_zero[i] && K_nz[i] > 0 && now - K_nz[i] < 0.5) { n++; hit = i; }
    if (n != 1) return;
    K_nz[hit] = 0;
    for (int i = 0; i < KEY_N; i++) if (i != hit) K_votes[i] = 0;
    if (++K_votes[hit] >= 2) {
        g_key_off = KEY_LO + 8 * hit;
        K_down_at = K_up_at = K_fire_at = 0; K_miss = 0; K_fired = 0;
        logline("Lane: the lane-keeping key is the action at game_actor+0x%x: a hold of %.1f s arms the steering calibration", g_key_off, p_cal_hold);
    } else logv(2, "Lane: lane-keeping key candidate game_actor+0x%x", KEY_LO + 8 * hit);
}

// A toggle seen by ln_frame (the driver's key, not our own switching). 1 = the key is known: tone and letter wait for the key to come up
// (or for the hold to act); 0 = give the tone at once.
static int key_edge(int on)
{
    (void)on;
    if (!control_on()) return 0;
    double now = wall_s();
    key_vote(now);
    if (!g_key_off || g_key_bad) return 0;
    if (!(K_down_at || (K_up_at && now - K_up_at < 0.5))) {   // the key was not down: the game switched it, not a press
        if (++K_miss >= 3) {
            logline("Lane: lane-keeping key at game_actor+0x%x was not held at 3 toggles in a row: looking for it again", g_key_off);
            g_key_off = 0; K_miss = 0; for (auto &v : K_votes) v = 0;
        }
        return 0;
    }
    K_miss = 0;
    // The game toggled while or just after a hold had acted (a toggle on the release of a long press): lane keeping stays on, no tone.
    if (K_fire_at > 0 && now - K_fire_at < 5 && (K_fired || !K_down_at)) {
        char *ga = game_actor_safe();
        if (ga && !on) lka_set(ga, 1);
        K_pend.active = 0;
        return 1;
    }
    K_pend.active = 1; K_pend.at = now;
    return 1;
}

static void key_tick(char *ga)
{
    double now = wall_s();
    static double scan_at;
    if (!g_key_off) {
        if (now - scan_at < 0.03) return;   // 30 Hz is plenty for a key that is down for 50 ms or more
        scan_at = now;
        for (int i = 0; i < KEY_N; i++) {
            int off = KEY_LO + 8 * i;
            if (K_bad[i] || (off >= 0x1e710 && off <= 0x1e750 && (off & 0xf) == 0)) continue;   // the cruise keys' own slots
            int st = key_state(ga, off, i);
            if (st < 0) continue;
            if (st > 2) { K_bad[i] = 1; continue; }        // not a key state
            if (st == 0) K_zero[i] = 1;
            else if (K_zero[i]) K_nz[i] = now;
        }
        return;
    }
    if (g_key_bad) { if (K_pend.active) { K_pend.active = 0; key_cue((*(unsigned char *)(ga + 0x1d1) & 0x20) != 0); } return; }
    int held = key_state(ga, g_key_off) > 0;
    if (held) { K_up_at = 0; if (!K_down_at) K_down_at = now; }
    else { if (!K_up_at) K_up_at = now; if (now - K_up_at > 0.1) { K_down_at = 0; K_fired = 0; } }
    if (K_down_at && now - K_down_at > 6) {   // nobody holds a key that long: the state does not return to 0
        g_key_bad = 1; K_pend.active = 0;
        logline("Lane: the lane-keeping key reads held for 6 s: tap and hold are off for this session (every press is a plain toggle)");
        return;
    }
    int on = (*(unsigned char *)(ga + 0x1d1) & 0x20) != 0;
    // The hold. Acts on the key alone (whether the game toggled on the press, on the release, or not at all): lane keeping on afterwards.
    if (!K_fired && K_down_at && now - K_down_at >= p_cal_hold) {
        K_fired = 1; K_fire_at = now; K_pend.active = 0;
        if (!on) lka_set(ga, 1);   // the press switched it off: a hold is not a toggle (silently)
        if (g_cal != CAL_IDLE) cal_abort("disarmed by the driver");
        else cal_arm(now);
        return;
    }
    // A tap: the toggle's tone when the key is up again before the hold
    if (K_pend.active && !K_down_at) { K_pend.active = 0; key_cue(on); }
}

static void ln_frame()
{
    lane_state_tick();
    if (control_on()) {
        char *ga0 = game_actor_safe();
        if (ga0) key_tick(ga0);
        if (g_cal == CAL_ARMED && wall_s() - g_cal_armed_at > p_cal_timeout) cal_abort("no suitable stretch in time");
        if (g_cal == CAL_RUN && wall_s() - g_stub_at > 0.3) cal_abort("lane keeping went off");
    }
    // Hysteresis on the minimum speed: once above on_kmh it works down to min_kmh; below that it waits for on_kmh.
    if (g_minp) {
        static int latched;
        float kmh = fabsf(T.speed) * 3.6f;
        if (kmh >= p_on_kmh) latched = 1;
        else if (kmh < p_min_kmh) latched = 0;
        *g_minp = p_keep_on > 0 ? 0.0f : latched ? p_min_kmh : fmaxf(p_on_kmh, p_min_kmh);
    }
    if (o_law && p_mode >= 1) { roundabout_off(); cruise_lane_on(); }   // mode 0 leaves the game's own assist alone, also its switch
    if (p_ind_pause > 0 && p_mode >= 1 && o_law) indicator_pause();  // first: it may undo a key press (see there)
    // Lane keeping switched on/off by its key: game_actor+0x1d1 bit 0x20 (lka-roll-re notes A2).
    static int was = -1, slow_off;
    static double at;
    static float prev_k;
    static double cross_at = -10;  // when the speed last came up through the game's minimum (55): the game's own switch-on happens then, a key press does not
    { float k = fabsf(T.speed) * 3.6f; if (prev_k < 55 && k >= 55) cross_at = wall_s(); prev_k = k; }
    if ((p_tone > 0 || control_on()) && o_law && wall_s() - at > 0.1) {  // offsets are 1.60's: only where the hook matched
        at = wall_s();
        char *ga = game_actor_safe();
        int on = ga ? (*(unsigned char *)(ga + 0x1d1) & 0x20) != 0 : -1;
        if (on >= 0 && was >= 0 && on != was && on == g_lka_expect) {
            logv(2, "Lane: lane keeping %s (by the plugin)", on ? "on" : "off");
            g_lka_expect = -1;
        } else if (on >= 0 && was >= 0 && on != was && on == 1 && p_slow_resume <= 0 && slow_off && wall_s() - cross_at < 0.6 && ga) {
            // Switched on by itself right where the speed comes back up through the game's minimum (55) after a slow stretch (Damien
            // 12-10: it came on after standing still). Undo it quietly; a key press at any other moment (the speed not just crossing 55) stays.
            lka_set(ga, 0);
            logline("Lane: lane keeping came on by itself at %.0f km/h after a slow stretch: switched off again (press its key to use it)", fabsf(T.speed) * 3.6f);
        } else if (on >= 0 && was >= 0 && on != was) {
            if (!on && fabsf(T.speed) * 3.6f < 56) slow_off = 1;  // went off at low speed (the game's rule or the key)
            else if (on) slow_off = 0;                           // switched on by the key at another speed: normal again
            g_lka_expect = -1;
            if (!key_edge(on)) key_cue(on);   // (with `control` and the key known, key_tick gives the tone when the key comes up)
            g_engaged = 0;
        }
        if (on >= 0) was = on;
    }
    DEV_NOTE(lane(o_law ? game_actor_safe() : NULL, o_law ? g_pause : -1, g_lane_state));  // game_actor offsets are 1.60's: only where the hook matched
    // learning profile: per wheel (winmm name), loaded once the wheel is found (or after 45 s as "default"), saved at most every learn_save_s
    if (p_learn > 0 && p_mode >= 2 && o_law) {
        if (!L_loaded) {
            const char *wn = input_wheel_name();
            if ((wn && *wn) || wall_s() > 45) prof_load(wn && *wn ? wn : "default");
        } else if (L_dirty && wall_s() - L_save_at > p_l_save) prof_save("periodic");
    }
    static double flushed;
    if (g_csv && wall_s() - flushed > 2) { flushed = wall_s(); fflush(g_csv); csv_cap(&g_csv, "lane.csv"); }
}

module m_lane = { "lane", "Lane", g_params, sizeof g_params / sizeof g_params[0], ln_start, ln_frame, NULL, ln_stop, ln_reset, ln_status };
