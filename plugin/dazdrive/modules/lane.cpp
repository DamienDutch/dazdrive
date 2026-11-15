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
#include "../core/sense.h"
#include "../core/act.h"
#include "../core/lanehook.h"
#include "../core/lanesense.h"
#include "../core/route.h"
#include "../core/vehicle.h"
#include "../core/vclass.h"
#include "../core/roll.h"
#include "../core/latload.h"
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

static float p_on_kmh, p_a_max, p_a_leak, p_min_kmh, p_mode, p_gain, p_dead, p_tone;
static float p_runaway, p_bend_out, p_bias, p_blend, p_blend_max, p_f_pull, p_follow, p_f_act, p_f_hold, p_f_win, p_f_max, p_f_tau, p_f_ret, p_f_rate, p_f_back, p_f_bend;   // shared steering
static float p_edge_look, p_edge_from, p_edge_full, p_backoff, p_drv_rate, p_drv_follow, p_drv_attack, p_drv_release, p_drv_leak;
static float p_needs_cc, p_cc_lka, p_rb_off, p_umax, p_ind_pause, p_ind_resume, p_ind_off, p_keep_on, p_nonffb, p_slow_resume;
static float p_sway, p_sw_cut, p_sw_rec, p_sw_amp;
static float p_learn, p_l_rate, p_l_min, p_l_max, p_l_cal_max, p_l_update, p_l_save;
static float p_head_ref, p_k_off, p_k_head, p_k_afb, p_bend_a, p_edge_boost, p_rate2, p_i2, p_i2max;
static float p_wref, p_bend_hold, p_unwind_s, p_bend_lead, p_soft, p_bend_lat;
static float p_control, p_cal_hold, p_cal_timeout;
#include "../core/lanelaw.h"

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
    { "learn_cal_max", &p_l_cal_max, 1.4f, "learning, 1.61 games: a steering calibration may set the scale up to this (above learn_max) when it measured, on a straight stretch, a wheel that needs more gain. The live wheel gain, a ring, a sway or a runaway take it back to learn_max. 1 = never above learn_max" },
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
    { "needs_cruise", &p_needs_cc, 1, "1 = lane keeping exists only while the cruise control is on (also with mode 0, the game's own steering). The lane-keeping key with the cruise off switches the cruise on (set speed as Resume does with speed limit following; the last set speed or the current speed without a limit) and lane keeping with it; the cruise going off switches lane keeping off; Resume or Set alone leave it off. 0 = lane keeping is independent of the cruise" },
    { "cruise_switches_on", &p_cc_lka, 0, "with needs_cruise: 1 = switching the cruise on (also Resume) switches lane keeping on (like a travel assist); 0 = the cruise leaves lane keeping as it is" },
    { "roundabout_off", &p_rb_off, 1, "1 = lane keeping yields to your hand when the truck's sideways load is over what a bend is planned with (a roundabout, a bend taken too fast, a junction you steer through): while you hold the wheel through it, the assist fades out between 70 % of the sideways acceleration where the lean reaches its edge and that acceleration itself, with a tone, and is back within 1 s when the load eases. Hands off, nothing changes. It reads the sideways acceleration and your steering, never the route or the cruise; the key name is older (it used to follow the cruise cancelling for a roundabout). 0 = off" },
    { "indicator_resume", &p_ind_resume, 0, "with indicator_pause: lane keeping that went off during the lane change (above min_kmh) is switched on again afterwards (sets the game's lane-keeping flag). Off by default: the game does not switch it off for an indicator, and with keep_on not for speed, so it would mostly undo your own key press" },
    { "non_ffb_mode", &p_nonffb, 0, "without a force-feedback wheel (keyboard, gamepad, mouse steering: the game adds the assist to the steering angle instead of turning the wheel; recognized from the steering data in ~1 s): 0 = the game's own law (mode 2 is fitted on FFB wheels), 2 = mode 2 anyway (not tuned)" },
    { "tone", &p_tone, 1, "1 = a rising tone when lane keeping is switched on, a falling one when it goes off" },
    { "control", &p_control, 1, "1 = the game's lane-keeping key tells a tap from a hold: a tap switches lane keeping on or off as always, holding it (calib_hold_s) arms the steering calibration test. Needs mode 2 and learn 1. The key is found by watching its first presses: until then a hold is a plain toggle. 0 = the key only toggles" },
    { "calib_hold_s", &p_cal_hold, 0.8f, "control: hold the lane-keeping key this long (s) to arm the steering calibration; hold it again to disarm. Lane keeping is on afterwards (with needs_cruise only if the cruise is on)" },
    { "calib_timeout_s", &p_cal_timeout, 300, "control: an armed calibration that finds no suitable stretch (straight road, a lane seen, lane centering steering, nobody close ahead, a speed the model is valid at) gives up after this many seconds, with a tone" },
};

// GA_U / V_STEER_IN / V_A / V_STEER are per-layout (L->a_lka_u / L->i_steer_in / L->i_assist / L->i_steer). V_STEER was a fixed 0x640 until 02-11: on 1.61 the
// field is at 0x6b8 (+0x78), so the steering read as a constant 41.42 there, the force-feedback wheel was never recognized from the data and in ATS (no stored
// profile to assume it from) the game's own law steered for good.
// (the offsets are read in core/sense.h since WP-S: sense_steering, sense_lka_u)

static law_fn o_law;
static FILE *g_csv;
static double g_csv_row_at = -1; static int g_csv_quiet;   // when lane.csv last got a row; 1 = a 'quiet' line was logged since
static int g_lka_expect = -1;       // a lane-keeping state we set ourselves: the on/off tone stays silent for it
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

// The profile belongs to a wheel on a game and a vehicle class (02-11): "<wheel>" for a truck in ETS2 (the key the old files hold), "<wheel>.ats" in ATS,
// "<wheel>.car" for a car. The suffixes come after the sanitized name, so a wheel called like one of them cannot collide with a plain key.
static char L_wheel[40] = "";   // the wheel name the profile was asked for
static int L_cls = -1;           // the vehicle class the loaded profile is for (-1 = none loaded through prof_load)
static void prof_key(char *out, size_t n, const char *name)
{
    snprintf(L_wheel, sizeof L_wheel, "%s", name && *name ? name : "default");
    L_cls = vclass_active();
    snprintf(out, n, "%s%s%s", L_wheel, sense_is_ats() ? ".ats" : "", L_cls == VC_LIGHT ? ".car" : "");
    for (char *c = out; *c; c++) if (!isalnum((unsigned char)*c) && !strchr("-._", *c)) *c = '_';
}
static void prof_load(const char *name)
{
    prof_key(L_key, sizeof L_key, name);
    L_loaded = 1;
    char buf[4096];
    int n = prof_read_all(buf, sizeof buf);
    char want[64]; snprintf(want, sizeof want, "wheel=%s ", L_key);
    for (char *ln = buf; n > 0 && *ln;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (!strncmp(ln, want, strlen(want))) {
            float s = 0, df = 0, cap = 0, ga = 0, c = 0, dl = 0, agv = 0; int nu = 0;
            const char *q;
            if ((q = strstr(ln, " ag="))) agv = (float)atof(q + 4);
            if ((q = strstr(ln, " s="))) s = (float)atof(q + 3);
            if ((q = strstr(ln, " dfac="))) df = (float)atof(q + 6);
            if ((q = strstr(ln, " cap="))) cap = (float)atof(q + 5);
            if ((q = strstr(ln, " ga="))) ga = (float)atof(q + 4);
            if ((q = strstr(ln, " c="))) c = (float)atof(q + 3);
            if ((q = strstr(ln, " delay="))) dl = (float)atof(q + 7);
            if ((q = strstr(ln, " n="))) nu = atoi(q + 3);
            bool good = s >= p_l_min && s <= s_stored_max() && df >= 0.5f && df <= 1.0f && cap >= p_l_min && cap <= s_stored_max() && s == s && df == df && cap == cap;
            if (!good) { logv(1, "Lane learn: profile for '%s' is broken or out of bounds, using the fixed gains", L_key); return; }
            L_cur.s = fminf(s, cap); L_cur.dfac = df; L_cur.cap = cap; L_base = L_cur; L_trial = 0;
            unsigned lv_bits = LV_S | LV_DFAC | LV_CAP;
            if (ga > 0.2f && c > 0.02f && dl >= 0 && dl < 1) { L_gA = ga; L_c = c; L_dl = dl; }
            // a profile exists only for a wheel mode 2 ran on, i.e. a force-feedback one: no 4 s of the game's own law first (11-10:
            // it let the truck drift to -0.9 m before mode 2 took over). The steering data still re-check it (two windows to flip).
            if (g_ffb < 0 && nu > 0) { g_ffb = 1; logline("Lane: force-feedback wheel assumed from its profile (re-checked from the steering data)"); }
            if (agv >= AG_MIN && agv <= AG_MAX) { g_ag = agv; AG_ann = agv; AG_snapped = agv == 1.0f; AG_known = 1; lv_bits |= LV_AG; }   // the actuator gain of the last session: the starting value, the estimator keeps checking it
            L_nupd = nu;
            LEARN_LOG(1, lv_bits, "Lane learn: profile for '%s' loaded: s %.2f dfac %.2f ceiling %.2f actuator gain %.3f (gA %.2f delay %.2f c %.3f, %d updates)", L_key, L_cur.s, L_cur.dfac, L_cur.cap, g_ag, L_gA, L_dl, L_c, nu);
            return;
        }
        if (!nl) break;
        ln = nl + 1;
    }
    logv(1, "Lane learn: no profile for '%s' yet, starting from the prior: the reference loop (gA %.2f, delay %.2f s, c %.3f from one hands-free ride on one wheel) so s 1, dfac 1, ceiling %.2f; "
            "each update moves it %.0f %% of the way (the prior counts for %.1f updates), by x0.8 .. x1.25 at most, inside s %.2f .. %.2f", L_key, REF_GA, REF_DELAY, REF_C, L_cur.cap,
         100 * clampf(p_l_rate, 0.05f, 1), (1 - clampf(p_l_rate, 0.05f, 1)) / clampf(p_l_rate, 0.05f, 1), p_l_min, p_l_max);
}

static void prof_save(const char *why)
{
    if (!L_loaded || !(p_learn > 0) || !L_dirty) return;
    char buf[4096], out[4096]; int n = prof_read_all(buf, sizeof buf), m = 0, lines = 0;
    char want[64]; snprintf(want, sizeof want, "wheel=%s ", L_key);
    char agtxt[24] = ""; if (g_ag != 1.0f) snprintf(agtxt, sizeof agtxt, " ag=%.4f", g_ag);   // (only a game whose actuator differs from the ETS2 fit has one)
    for (char *ln = buf; n > 0 && *ln && lines < 16;) {
        char *nl = strchr(ln, '\n'); if (nl) *nl = 0;
        if (*ln && strncmp(ln, want, strlen(want)) && strncmp(ln, "wheel=", 6) == 0 && m + (int)strlen(ln) + 2 < 3500) { m += snprintf(out + m, sizeof out - m, "%s\n", ln); lines++; }
        if (!nl) break;
        ln = nl + 1;
    }
    m += snprintf(out + m, sizeof out - m, "wheel=%s v=1 s=%.4f dfac=%.4f cap=%.4f ga=%.3f c=%.4f delay=%.3f%s n=%d\n", L_key, L_cur.s, L_cur.dfac, L_cur.cap, L_gA, L_c, L_dl, agtxt, L_nupd);
    FILE *f = open_out("steer_profile.txt", "w");
    L_save_at = wall_s();
    if (!f) { static int warned; if (!warned++) logv(1, "Lane learn: cannot write steer_profile.txt (the profile stays in memory)"); return; }
    size_t w = fwrite(out, 1, m, f);
    fclose(f);
    if (w == (size_t)m) { L_dirty = 0; logv(1, "Lane learn: profile for '%s' saved (%s): s %.2f dfac %.2f", L_key, why, L_cur.s, L_cur.dfac); }
}

static void stub_law(void *ga, float o1, float o2, float o3, float dt)
{
    char *g = (char *)ga;
    LE.is161 = sense_is_161();   // what the decide layer (core/lanelaw.h) needs from the game besides its arguments
    float before = g ? sense_lka_u(g) : 0;
    o_law(ga, o1, o2, o3, dt);
    if (!g) return;
    g_stub_at = wall_s();
    float u = sense_lka_u(g), out = u;
    float A, steer, wheel;
    sense_steering(&A, &steer, &wheel);
    // needs_cruise with the cruise off (15-11): lane keeping exists only with the cruise. The law is still called for the moments the key's decision waits
    // (the tap or hold is not told yet, the cruise is being switched on): no request then, the game's own is let go like in an indicator pause.
    if (p_mode >= 1 && p_needs_cc > 0 && !(T.cruise > 0.5f)) {
        if (g_lane_state != LANE_WAIT_NO_CRUISE) lane_reset();   // our short-term state is forgotten once, as after a pause
        ag_freeze(wall_s(), fabsf(T.speed) * 3.6f);
        if (g_cal == CAL_RUN) cal_abort("cruise off");
        if (g_cal == CAL_ARMED && wall_s() - g_cal_armed_at > p_cal_timeout) cal_abort("no suitable stretch in time");   // (cal_tick, which times it out otherwise, does not run)
        float d = before * 0.85f;
        act_lka_u(NULL, g, fabsf(d) < 1e-3f ? 0 : d);
        g_u = 0; g_ci = 0;
        g_lane_state = u != 0 ? LANE_WAIT_NO_CRUISE : LANE_OFF;
        return;
    }
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
        ag_freeze(wall_s(), kmh_now);
        if (g_cal == CAL_RUN) cal_abort(g_pause ? "indicator" : "too slow");
        g_lane_state = g_pause ? LANE_WAIT_INDICATOR : LANE_WAIT_SLOW;
        float d = before * 0.85f;
        act_lka_u(NULL, g, fabsf(d) < 1e-3f ? 0 : d);
        g_u = 0; g_ci = 0; g_ref = 0; g_have_o = 0;
        return;
    }
    if (p_mode >= 1 && o1 == 0 && o2 == 0 && o3 == 0) {
        if (g_cal == CAL_RUN) cal_abort("the lane was lost");
        g_lane_state = LANE_WAIT_NO_LANE;
        float d = before * 0.9f;
        act_lka_u(NULL, g, fabsf(d) < 1e-3f ? 0 : d);
        g_u = 0;
        return;
    }
    sense_lane_publish(o3);   // the road's curvature for the readers that want it (core/lanesense.h: Cutin)
    if (p_mode >= 1) ffb_detect(A, steer, wheel);
    ag_step(before, A, dt, fabsf(T.speed) * 3.6f, wall_s());
    const float A_raw = A;
    if (A == A) A /= g_ag;   // from here on the assist offset is in model units (g_ag is 1 on ETS2)
    // the law ran: it wrote a new u. Saturated it writes exactly +-1, the same as our clamped output, so u == before then too
    // (ts5b 387.4 s: skipped while saturated -> filter_a took the law for off and released the wheel in the bend). It leaves u
    // unchanged for |o1| > 1.95 (lka-roll-re notes A2) and, presumably, when lane keeping is not active: so only with the
    // lane-keeping bit on and our law running a moment ago (never keep a stale +-1 going after it went off).
    int ran = u != before || (fabsf(u) >= 0.999f && fabsf(o1) <= 1.95f && (sense_lka_byte(g) & 0x20) && wall_s() - g_law_at < 0.15);
    if (law2() && ran) {
        g_ind = (sense_lights(g) & 12) != 0;
        if (p_soft > 0 && p_soft < 1 && wall_s() - g_law_at > 0.5) {   // our law did not run for 0.5 s: switched on / lane found again
            g_soft_at = wall_s(); g_soft_end = g_centre_since = -1;
            logv(2, "Lane: soft start (%.0f%% until near the center), o1 %.2f m", p_soft * 100, o1);
        }
        g_lane_state = LANE_ACTIVE;   // the runaway guard in lane_step overrides it
        float cal_out = 0;
        if (g_cal_reset) { g_cal_reset = 0; lane_reset(); }
        LE.lights = sense_lights(g);
        int cal = g_cal != CAL_IDLE && cal_tick(o1, o2, o3, dt, wheel, A, fabsf(T.speed) * 3.6f, &cal_out);   // the steering calibration owns the request
        if (g_cal_reset) { g_cal_reset = 0; lane_reset(); }
        out = cal ? cal_out : lane_step(u, o1, o2, o3, dt, wheel, A, fabsf(T.speed) * 3.6f);
        if (g_soft_at >= 0 && !cal) { out *= soft_share(o1, o2, fabsf(T.speed) * 3.6f, wall_s()); g_u = out; }
        // Indicator set (the game's own clamp, lka-roll-re notes A2): the assist may not steer back against a lane change.
        // Mode 2 is much firmer than the game's law, so it must keep this rule too.
        unsigned char fl = sense_lights(g);
        if (p_mode >= 2) { if (fl & 4) out = fmaxf(out, 0); if (fl & 8) out = fminf(out, 0); }
        // back from an indicator pause: ramp in over 1 s (no jerk on the wheel in the new lane)
        double since = wall_s() - g_resume_at;
        if (p_mode >= 2 && since >= 0 && since < 1) { out *= (float)since; g_u = out; }
        // The sideways load (lat_frame above): with a hand on the wheel the whole request, ours or the game's own that the runaway guard handed over, yields by the load's
        // weight (a hand on the wheel through a roundabout is the driver's; the game's law would push against it, as it did in the prefab of 25-10). Hands off: whole.
        if (g_latw < 1.0f && !cal) {
            const float w = lat_eff();
            if (w < 1.0f) { out *= w; g_u = out; }
            if (g_lat_off && g_lane_state == LANE_ACTIVE) g_lane_state = LANE_WAIT_LOAD;
        }
        act_lka_u(NULL, g, out);
        if (!g_engaged && g_act > 0.5f) {
            g_engaged = 1;
            logline("Lane: you are steering (wheel %.3f), assist backs off to %.0f%%", wheel, g_scale * 100);
        }
    } else if (p_mode >= 1 && !law2()) {
        // the game's own law (no FFB / profile not known yet): the same ramp after a pause or the low-speed gate
        if (ran) g_lane_state = LANE_ACTIVE;
        double since = wall_s() - g_resume_at;
        if (since >= 0 && since < 1) { out = u * (float)since; act_lka_u(NULL, g, out); }
    }
    if (u == 0) lane_reset();  // the game switched the assist off: let go at once
    if (u == 0) { out = 0; g_lane_state = LANE_OFF; }
    g_u = out;
    if (!g_csv || u == 0) return;
    static double at;
    double now = wall_s();
    if (now - at < 0.05) return;
    at = now;
    if (g_csv_quiet) { g_csv_quiet = 0; logline("Lane: lane.csv writes again (lane keeping is on)"); }
    g_csv_row_at = now;
    fprintf(g_csv, "%.2f,%.1f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.2f,%.2f,%.2f,%.2f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f\n", now, fabsf(T.speed) * 3.6f, o1,
            o2, o3, u, out, A_raw, steer, wheel, g_act, g_scale, g_ci, g_damp_u, g_ff, scale_s(), scale_df(), L_gA, L_dl, L_c, p_sway > 0 ? SW_fac : 1.0f, L_usable,
            g_wg, g_wi, g_lf, g_li, g_hold, g_ref, g_share, g_bias, g_ag, g_latw, g_lat_r.accel);
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
    if (p_mode >= 1 && (g_pause || (p_keep_on > 0 && T.speed * 3.6f < p_min_kmh))) return fabsf(a) < 1e-4f ? 0 : a * 0.94f;  // indicator pause / too slow: release the wheel in ~0.3 s
    if (p_mode < 1 || g_u == 0 || !law2()) return a;
    a *= 1 - clampf(p_a_leak + g_act * p_drv_leak, 0, 0.5f);
    float lim = p_a_max * fminf(g_ag, 1.0f);   // the ceiling is in model units (a weaker actuator: A is smaller for the same request)
    return clampf(a, -lim, lim);
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
    g_latw = 1; g_lat_off = 0; g_lat = lat_state();   // the sideways load starts again (a pause, a teleport)
}

static int s_mode_off_at_start;   // the assist limiter (filter_a) is only hooked when the game starts with lane mode on

static void ln_start()
{
    LE.is161 = sense_is_161();
    if (p_mode > 0 && p_mode < 2) { logline("Lane: mode 1 no longer exists, using mode 2"); p_mode = 2; }
    s_mode_off_at_start = p_mode < 1;
    // The hook is installed by core/lanehook.cpp (the call site of the game's control law, the minimum-speed constant, the store of A): this module has no scan, offset or
    // patch of its own (WP-LA2). The call site's pattern carries no struct offsets, so it can match on any build while the fields the law reads and writes are per-layout:
    // it hooks only when the layout has them.
    snprintf(g_status, sizeof g_status, "off (the layout has no LKA offsets)");
    const int hooked = lanehook_law(stub_law, &o_law);
    if (hooked == LH_NO_LAYOUT) { logline("Lane: the layout has no LKA offsets, lane assist stays the game's"); return; }
    snprintf(g_status, sizeof g_status, "off (LKA code not found)");
    if (hooked == LH_NOT_FOUND) { logline("Lane: LKA control law call not found, lane assist stays the game's"); return; }
    if (hooked == LH_FAILED) { logline("Lane: hook failed"); return; }
    act_lka_owner("lane");   // the request the stub writes into the game's control output is ours from here (logged once)
    // The control law switches itself off below 55 km/h: point that one compare at our own value.
    if (p_mode < 1) p_keep_on = 0;  // the game's own law (mode 0) keeps its own speed limits: untested at a standstill
    if ((p_min_kmh > 0 && p_min_kmh < 55) || p_keep_on > 0) {
        if (lanehook_min_speed_patch(p_keep_on > 0 ? 0.0f : fmaxf(p_on_kmh, p_min_kmh))) {
            if (p_keep_on > 0) logline("Lane: lane keeping stays on at any speed (also switched on standing still), steers from %.0f km/h", p_min_kmh);
            else logline("Lane: lane keeping from %.0f km/h, down to %.0f", fmaxf(p_on_kmh, p_min_kmh), p_min_kmh);
        }
        else logline("Lane: minimum speed code not found, stays 55 km/h");
    }
    if (p_mode >= 1) {
        if (lanehook_assist_store(filter_a)) logline("Lane: assist build-up limited (max %.2f, leak %.2f)", p_a_max, p_a_leak);
        else logline("Lane: assist store not found, only the request is calmed");
    }
    g_csv = open_out("lane.csv", "w");
    if (g_csv) fputs("t,kmh,o1,o2,o3,u_game,u_out,A,steer,wheel,activity,scale,ci,damp,ff,l_s,l_dfac,l_gA,l_delay,l_c,sway_fac,l_usable,w_ratio,w_slope,lf,li,bend_hold,ref,share,bias,ag,latw,lat_accel\n", g_csv);
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
    if (on && p_needs_cc > 0 && !(T.cruise > 0.5f)) { logv(2, "Lane: lane keeping not switched on (needs_cruise, the cruise is off)"); return; }   // (the lane key with the cruise off: cc_tap)
    g_lka_expect = on;
    if (on) act_lka_bit_set("lane", ga);  // (the game thread writes this byte too: atomic)
    else act_lka_bit_clear("lane", ga);
}

static void indicator_pause()
{
    char *ga = sense_actor();
    if (!ga) return;
    static int was_on, we_off;    // lane keeping was on at the indicator / we switched it off (indicator_off)
    static double off_at, on_at;  // indicator off since / this indicator period on since (0 = none)
    unsigned char fl = sense_lights(ga);
    int one = (fl & 12) == 4 || (fl & 12) == 8;  // both = hazards: no lane change
    int lka = (sense_lka_byte(ga) & 0x20) != 0;
    double now = wall_s();
    // The lane-keeping key pressed while we hold it off for the indicator: that means OFF (Damien 11-10), not on again.
    // Off it stays, also after the indicator; one falling tone and the X confirm it (indicator_pause runs before the tone
    // watcher in ln_frame, so it never sees the short "on").
    if (we_off && lka && g_lka_expect != 1) {
        act_lka_bit_clear("lane", ga);
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
        char *ga = sense_actor();
        float kmh = fabsf(T.speed) * 3.6f;
        if (!ga || kmh <= fmaxf(p_min_kmh, 5) + 2) return;   // too slow yet (a Resume in a jam): try again when the speed is there
        done_at = now;
        if (!(sense_lka_byte(ga) & 0x20)) {
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
static double g_cross_at = -10;                                   // when the speed last came up through the game's minimum (55): the game's own switch-on happens then, a key press does not
static int g_key_off, g_key_bad;                                  // game_actor offset of the lane key's action pointer (0 = not found), gave up
static double K_nz[KEY_N], K_down_at, K_up_at, K_fire_at;         // per slot: when its state was last non-zero; the found key: held since / up since; when a hold last acted
static unsigned char K_zero[KEY_N], K_bad[KEY_N], K_votes[KEY_N], K_ok[KEY_N];
static void *K_ptr[KEY_N];                                        // the object each slot pointed to at the last look, and whether it is readable (K_ok)
static int K_miss, K_fired;                                       // toggles in a row without the key down; the hold of this press has acted
static struct { int active; double at; } K_pend;                  // a toggle whose tone waits for the key to come up (or for the hold)

static int control_on() { return p_control > 0 && p_mode >= 2 && p_learn > 0 && o_law; }

// The action state at ga+off: -1 = no action object there. slot < 0: not cached (the found key); else the readable check of the object is
// only repeated when the pointer in the slot changes (96 slots at frame rate would be a lot of VirtualQuery calls).
static int key_state(char *ga, int off, int slot = -1) { return sense_action_state_at(ga, off, slot, K_ptr, K_ok); }

static void key_cue(int on, const char *why = NULL)   // the tone, the dashboard letter and the log line of a toggle (why: the reason, when it was not the key alone)
{
    if (p_tone > 0) play_cue(on ? CUE_ON : CUE_OFF);
    disp_post(DISP_LANE, DISP_FLASH, on ? 'L' : 'X', 1.5);  // L = lane keeping on, X = off (1.5 s on the dashboard)
    logline("Lane: lane keeping %s%s%s%s", on ? "on" : "off", why ? " (" : "", why ? why : "", why ? ")" : "");
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
        char *ga = sense_actor();
        if (ga && !on) lka_set(ga, 1);
        K_pend.active = 0;
        return 1;
    }
    K_pend.active = 1; K_pend.at = now;
    return 1;
}

// ---------------------------------------------------------------------------------------------------------------
// needs_cruise (15-11, the owner's wish): lane keeping exists only while the cruise is on.
//  - The cruise goes off (key, brake, AEB, a junction): lane keeping goes off with the off tone and the X (cc_gate, one log line).
//  - The lane key with the cruise off: a TAP switches the cruise on as Resume does with ISA (Cruise: cruise_lane_engage) and lane keeping stays on; where the
//    cruise cannot come on (under its minimum speed, reversing, the limiter) neither comes on. A HOLD only arms the calibration, as always (the toggle of the press is
//    undone: the calibration runs with lane keeping on, that is with the cruise). The game switching lane keeping on by itself (speed up through its minimum) is no key.
//  - Resume or Set alone never switch lane keeping on (cruise_switches_on = 1 does, in cruise_lane_on). With the cruise off lka_set refuses to switch it on.
static double g_cc_wait_at = -1;   // the lane key switched lane keeping on with the cruise off, the tap or hold not told yet (-1 = none)
static double g_cc_grace = -1;     // until when the cruise byte may take to show after we switched the cruise on (-1 = none)
static char g_cc_why[96];

// The tap: switch the cruise on, or refuse. 1 = lane keeping stays on. cue: give the off tone and the X now when refused (the key path gives its own).
static int cc_tap(char *ga, int cue)
{
    g_cc_wait_at = -1;
    const char *why = ""; float set = 0;
    if (cruise_lane_engage(&why, &set)) { g_cc_grace = wall_s() + 1.0; return 1; }
    snprintf(g_cc_why, sizeof g_cc_why, "cruise stays off: %s", why);
    lka_set(ga, 0);
    K_pend.active = 0;
    if (cue) key_cue(0, g_cc_why);
    return 0;
}

static void cc_gate()
{
    static int prev_lka = -1;
    char *ga = sense_actor();
    // (any mode: the gate is about the switch, not the law; in mode 0 the game's own law steers while the cruise is on)
    if (!(p_needs_cc > 0 && o_law) || !ga || T.paused) { prev_lka = -1; g_cc_wait_at = g_cc_grace = -1; return; }
    const double now = wall_s();
    const int lka = (sense_lka_byte(ga) & 0x20) != 0;
    const int was = prev_lka; prev_lka = lka;
    if (T.cruise > 0.5f || !lka) { g_cc_wait_at = -1; if (T.cruise > 0.5f) g_cc_grace = -1; return; }
    // lane keeping on, the cruise off
    if (g_cc_grace > 0) {
        if (now < g_cc_grace) return;   // we just switched the cruise on
        g_cc_grace = -1;
        lka_set(ga, 0); K_pend.active = 0;
        key_cue(0, "the cruise did not come on");
        return;
    }
    const int key_known = control_on() && g_key_off && !g_key_bad;
    if (g_cc_wait_at < 0) {
        if (was != 0) { lka_set(ga, 0); K_pend.active = 0; key_cue(0, "cruise off"); return; }   // on already (the cruise went off, or from before the start)
        // a fresh switch-on with the cruise off
        if (key_known && K_fire_at > 0 && now - K_fire_at < 5 && (K_fired || !K_down_at)) { lka_set(ga, 0); return; }   // the toggle of a hold's release: no tap
        const int pressed = key_known ? (K_down_at || (K_up_at && now - K_up_at < 0.5)) : now - g_cross_at >= 0.6;
        if (!pressed) { lka_set(ga, 0); logv(2, "Lane: lane keeping came on by itself with the cruise off: switched off again"); return; }
        g_cc_wait_at = now;
    }
    if (key_known) {
        if (K_fired) {   // a hold: the calibration is armed, lane keeping waits for the cruise
            g_cc_wait_at = -1; lka_set(ga, 0);
            logline("Lane: calibration armed with the cruise off: lane keeping comes on with the cruise (press the lane key)");
            return;
        }
        if (K_down_at && now - g_cc_wait_at < p_cal_hold + 2) return;   // the key is still down: tap or hold is not told yet
    }
    cc_tap(ga, 1);
}

static void key_tick(char *ga)
{
    double now = wall_s();
    static double scan_at;
    if (!g_key_off) {
        if (now - scan_at < 0.03) return;   // 30 Hz is plenty for a key that is down for 50 ms or more
        scan_at = now;
        int klo = sense_key_off(0);   // the cruise keys' own slots (toggle..accmode, 5 slots)
        for (int i = 0; i < KEY_N; i++) {
            int off = KEY_LO + 8 * i;
            if (K_bad[i] || (off >= klo && off <= klo + 0x40 && (off & 0xf) == 0)) continue;
            int st = key_state(ga, off, i);
            if (st < 0) continue;
            if (st > 2) { K_bad[i] = 1; continue; }        // not a key state
            if (st == 0) K_zero[i] = 1;
            else if (K_zero[i]) K_nz[i] = now;
        }
        return;
    }
    if (g_key_bad) { if (K_pend.active) { K_pend.active = 0; key_cue((sense_lka_byte(ga) & 0x20) != 0); } return; }
    int held = key_state(ga, g_key_off) > 0;
    if (held) { K_up_at = 0; if (!K_down_at) K_down_at = now; }
    else { if (!K_up_at) K_up_at = now; if (now - K_up_at > 0.1) { K_down_at = 0; K_fired = 0; } }
    if (K_down_at && now - K_down_at > 6) {   // nobody holds a key that long: the state does not return to 0
        g_key_bad = 1; K_pend.active = 0;
        logline("Lane: the lane-keeping key reads held for 6 s: tap and hold are off for this session (every press is a plain toggle)");
        return;
    }
    int on = (sense_lka_byte(ga) & 0x20) != 0;
    // The hold. Acts on the key alone (whether the game toggled on the press, on the release, or not at all): lane keeping on afterwards.
    if (!K_fired && K_down_at && now - K_down_at >= p_cal_hold) {
        K_fired = 1; K_fire_at = now; K_pend.active = 0;
        if (!on) lka_set(ga, 1);   // the press switched it off: a hold is not a toggle (silently)
        if (g_cal != CAL_IDLE) cal_abort("disarmed by the driver");
        else cal_arm(now);
        return;
    }
    // A tap: the toggle's tone when the key is up again before the hold (with needs_cruise and the cruise off, the tap first switches the cruise on)
    if (K_pend.active && !K_down_at) {
        K_pend.active = 0;
        if (g_cc_wait_at >= 0 && p_needs_cc > 0 && !(T.cruise > 0.5f)) on = cc_tap(ga, 0);
        key_cue(on, on ? NULL : g_cc_why);
    }
}

// The phone's button box (dev builds): arm the steering calibration, or disarm it, like a long hold of the lane-keeping key. Called from
// the game's frame thread (like key_tick). NULL = done (*armed 1 = armed now, 0 = disarmed), else the reason it could not be done.
const char *lane_cal_request(int *armed)
{
    if (!(p_mode >= 2 && p_learn > 0 && o_law)) return "needs lane mode 2, learn 1 and the lane hook";
    char *ga = sense_actor();
    if (!ga) return "game not ready";
    if (g_cal != CAL_IDLE) { cal_abort("disarmed from the phone"); *armed = 0; return NULL; }
    if (!(sense_lka_byte(ga) & 0x20)) lka_set(ga, 1);   // lane keeping is on afterwards, as after the hold
    cal_arm(wall_s());
    *armed = 1;
    return NULL;
}

static void ln_frame()
{
    LE.is161 = sense_is_161();
    lane_state_tick();
    if (control_on()) {
        char *ga0 = sense_actor();
        if (ga0) key_tick(ga0);
        if (g_cal == CAL_ARMED && wall_s() - g_cal_armed_at > p_cal_timeout) cal_abort("no suitable stretch in time");
        if (g_cal == CAL_RUN && wall_s() - g_stub_at > 0.3) cal_abort("lane keeping went off");
    }
    // Hysteresis on the minimum speed: once above on_kmh it works down to min_kmh; below that it waits for on_kmh.
    if (lanehook_min_speed_ok()) {
        static int latched;
        float kmh = fabsf(T.speed) * 3.6f;
        if (kmh >= p_on_kmh) latched = 1;
        else if (kmh < p_min_kmh) latched = 0;
        act_lane_min_speed(p_keep_on > 0 ? 0.0f : latched ? p_min_kmh : fmaxf(p_on_kmh, p_min_kmh));
    }
    if (o_law && p_mode >= 1) cruise_lane_on();   // mode 0 leaves the game's own assist alone, also its switch
    cc_gate();
    lat_frame();
    if (p_ind_pause > 0 && p_mode >= 1 && o_law) indicator_pause();  // first: it may undo a key press (see there)
    // Lane keeping switched on/off by its key: game_actor+0x1d1 bit 0x20 (lka-roll-re notes A2).
    static int was = -1, slow_off;
    static double at;
    static float prev_k;
    { float k = fabsf(T.speed) * 3.6f; if (prev_k < 55 && k >= 55) g_cross_at = wall_s(); prev_k = k; }
    if ((p_tone > 0 || control_on()) && o_law && wall_s() - at > 0.1) {  // offsets are 1.60's: only where the hook matched
        at = wall_s();
        char *ga = sense_actor();
        int on = ga ? (sense_lka_byte(ga) & 0x20) != 0 : -1;
        if (on >= 0 && was >= 0 && on != was && on == g_lka_expect) {
            logv(2, "Lane: lane keeping %s (by the plugin)", on ? "on" : "off");
            g_lka_expect = -1;
        } else if (on >= 0 && was >= 0 && on != was && on == 1 && p_slow_resume <= 0 && slow_off && wall_s() - g_cross_at < 0.6 && ga) {
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
    {   // (no game actor in a menu: the garage, the job screens; the lane bit is then unknown)
        char *dga = o_law ? sense_actor() : NULL;
        DEV_NOTE(lane(dga, sense_lka_state(dga), o_law ? g_pause : -1, g_lane_state));
    }
    // learning profile: per wheel (winmm name), loaded once the wheel is found (or after 45 s as "default"), saved at most every learn_save_s
    if (p_learn > 0 && p_mode >= 2 && o_law) {
        if (!L_loaded) {
            const char *wn = input_wheel_name();
            if ((wn && *wn) || wall_s() > 45) prof_load(wn && *wn ? wn : "default");
        } else {
            if (AG_dirty) { AG_dirty = 0; L_dirty = 1; }
            if (L_cls >= 0 && vclass_active() != L_cls) {   // another vehicle class: its own profile (a car steers differently from a truck)
                prof_save("class change");
                learn_flush(); L_init = 0; L_Nw = L_Syy = L_Cxx = L_Cxy = L_Cyy = 0; for (int k = 0; k < LK_NK; k++) L_Sxx[k] = L_Sxy[k] = 0;
                L_cur = L_base = { 1, 1, 2 }; L_gA = REF_GA; L_dl = REF_DELAY; L_c = REF_C; L_nupd = 0; L_trial = 0; L_hold = 0; L_dirty = 0;
                LEARN_LOG(1, LV_S | LV_DFAC | LV_CAP, "Lane learn: another vehicle class: the profile starts from the prior again (s 1, dfac 1, ceiling 2)");
                if (g_cal != CAL_IDLE) cal_abort("another vehicle");
                prof_load(L_wheel);
            } else if (L_dirty && wall_s() - L_save_at > p_l_save) prof_save("periodic");
        }
    }
    static double flushed;
    if (g_csv && wall_s() - flushed > 2) { flushed = wall_s(); fflush(g_csv); csv_cap(&g_csv, "lane.csv"); }
    // lane.csv gets a row per law call, and the game calls the law only while its lane keeping is on: after a runaway (the game's switch goes off) or with
    // the key not pressed the file is silent by design, as on the 11-11 ride after t 1275. Say so once, so a silent file is not taken for a broken one.
    if (g_csv && g_csv_row_at >= 0 && !g_csv_quiet && wall_s() - g_csv_row_at > 10) {
        g_csv_quiet = 1;
        logline("Lane: lane.csv has had no row for %.0f s: it gets one per law call and the game calls the law only while its lane keeping is on (now %s%s)",
                wall_s() - g_csv_row_at, g_lane_state == LANE_OFF ? "off" : "on", p_needs_cc > 0 && !(T.cruise > 0.5f) ? ", cruise off" : "");
    }
}

// A live setting changed (config_live_set). Mode 1 does not exist (it is 2). Off -> on needs the assist limiter hooked at start: not live then.
static const char *ln_applied(const char *key, float old)
{
    if (strcmp(key, "mode")) return NULL;
    const char *note = NULL;
    if (p_mode > 0 && p_mode < 2) { p_mode = 2; note = "mode 1 is mode 2"; }
    if (p_mode >= 1 && s_mode_off_at_start) { p_mode = old; return "after restart"; }
    return note;
}

module m_lane = { "lane", "Lane", g_params, sizeof g_params / sizeof g_params[0], ln_start, ln_frame, NULL, ln_stop, ln_reset, ln_status, ln_applied, VCM_BOTH };
