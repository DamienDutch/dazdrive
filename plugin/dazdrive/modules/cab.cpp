// Cab: two assists, like the electronic parking brake of a real truck and the start-stop of a modern car (rules and thresholds:
// core/cablogic.h). The parking brake: when you switch the engine off while standing and no parking brake is on, the parking
// brake goes on by itself; when you drive off (a gear in, gas, engine on) it comes off by itself - also one you set by hand,
// because that "drive off" is you asking the truck to do everything for you. When you change the parking brake yourself in
// between, the assist leaves it alone for a while (cablogic.h's driver hold), except that drive-off release. The start-stop: at
// every stop Auto Hold holds, the engine goes off after ss_after_s and starts again by itself on gas, on the vehicle ahead
// driving off, on low air, when the hold ends or after ss_max_s; an engine off from the start-stop is not a "switch off", so the
// parking brake does not apply on it.
//
// LOG ONLY by default ([cab] act 0): the module decides and writes "would apply / would release / would stop / would start" lines
// with the reason and the state the game really has. With act 1 it presses the game's parking brake and engine controls through
// the plugin's input device (core/input.cpp, mixes "parkingbrake" and "engine") and checks in the telemetry that they followed
// (core/toggle.h: one retry, then 30 s of silence; the engine gets a 4 s verify so a slow start is never retried).
//
// The log (promise P7): a line for every decision, "would apply / release / stop / start [term]: value against threshold" (core/cablogic.h
// names the terms), a line when an acting decision ends (the term that ended it) and, at log level 1, a line when the term that holds a
// decision back changes (at most one per 10 s; the engine's own events are never held back).
#include "../core/suite.h"
#include "../core/vclass.h"
#include "../core/cablogic.h"
#include "../core/toggle.h"
#include <math.h>
#include <string.h>

// How long a press of the game control is held (ms). One frame was too short in the game (30-10 ride: the engine never followed two
// presses, the parking brake missed its first try), a key a driver taps is down for about this long. Constants of the game's input handling,
// set by that ride; not derivable from the vehicle. The start hold (ignitionstrt, the starter itself) is a second of cranking.
static const int CAB_PRESS_MS = 250, CAB_ENGINE_PRESS_MS = 450, CAB_START_HOLD_MS = 900;
// The game's engine control only STARTS the engine; stopping it took the electrics off with it (30-10 test). So a stop turns the key:
// ignition off, then this long after, ignition on again (the electrics back, the engine stays off), as a keyed ignition does.
static const double CAB_IGN_BACK_S = 0.6;
static double g_ign_back_at;   // > 0: ignition on is due at this time

static float c_gas_edge() { return cab_cfg_default().release_thr; }
static float p_auto, p_act;
static float p_start_stop, p_ss_after, p_ss_min_psi, p_ss_max;
static int ss_on() { return p_start_stop > 0 || vprof()->start_stop_on > 0; }   // a car always has it, a truck by its ini

static param g_params[] = {
    { "park_auto", &p_auto, 1, "1 = auto parking brake: when you switch the engine off while standing, the parking brake goes on by itself; it comes off when you drive off (a gear in, gas, engine on). See act for what is done with the decision. 0 = off" },
    { "start_stop", &p_start_stop, 1, "1 = start-stop: while Auto Hold holds the truck at a standstill (forward gear or neutral, no gas, air fine, parking brake off) the engine switches off after ss_after_s and starts again by itself on gas, the vehicle ahead driving off, low air, the hold ending, reverse or after ss_max_s. See act for what is done with the decision. 0 = off" },
    { "ss_after_s", &p_ss_after, 3, "start-stop: the engine switches off after Auto Hold has held this long (s)" },
    { "ss_min_psi", &p_ss_min_psi, 100, "start-stop: the engine only switches off at this brake air pressure or more (psi), and starts again when the pressure drops 10 psi under it. A truck that reports no pressure (<= 1) counts as fine" },
    { "ss_max_s", &p_ss_max, 180, "start-stop: the engine comes back on after this long stopped (s), whatever else, as a safety" },
    { "act", &p_act, 0, "0 = log only: 'would apply', 'would release', 'would stop' and 'would start' lines with the reason and the state the game really has, nothing is pressed. 1 = press the game's parking brake and engine controls through the plugin's input device ([core] input_device) when the decision changes, and check that they took" },
};

static cab_state g_st;
static toggle_ctl g_tc, g_tc_ss;
static int g_virt = -1;          // our belief of the brake while only logging (-1 = follow the game)
static int g_prev_real = -1, g_pend = -1;
static int g_nodev_said, g_last_want = -1;   // the last decision written to the log (a line per change, not per frame)
static unsigned g_n_on, g_n_off;   // apply / release decisions
static int g_pend_ss = -1, g_last_ss_want = -1;
static int g_last_term, g_last_ss_term;          // the last term a line was written for while nothing was decided
static double g_last_term_at, g_last_ss_term_at;   // ... and when (one hold-term line per 10 s)
static unsigned g_n_ss_stop, g_n_ss_start;   // stop / start decisions
static char g_status[120] = "off";

int cab_park_auto_on() { return p_auto > 0; }
static int g_veng = -1, g_prev_eng = -1;   // log only: the engine as the start-stop believes it (-1 = follow the game), and the game's last state

int cab_ss_engine_off() { return ss_on() && p_act > 0 && g_st.ss_off; }

static void cab_frame()
{
    if (p_auto <= 0 && !ss_on()) return;
    double now = wall_s();
    if (T.paused) return;

    int real = T.parking_brake;
    int changed = g_prev_real >= 0 && real != g_prev_real;
    int ours = changed && toggle_ours(&g_tc, now);
    if (changed) logv(1, "Cab: the parking brake is now %s%s", real ? "on" : "off", ours ? " (after our press)" : "");
    g_prev_real = real;

    cab_in in;
    memset(&in, 0, sizeof in);
    in.t = now;
    // log only: our belief of the engine, so the log shows the whole stop / start cycle the start-stop would run (the engine
    // really stays on); a real change of the engine (the driver's key) puts the belief back on the game's state
    if (p_act <= 0 && ss_on()) {
        if (g_veng < 0 || T.engine_on != g_prev_eng) g_veng = T.engine_on;
        in.engine_on = g_veng;
    } else in.engine_on = T.engine_on;
    g_prev_eng = T.engine_on;
    in.paused = T.paused;
    in.kmh = fabsf(T.speed) * 3.6f;
    in.dgear = T.dgear;
    in.thr = cab_gas(T.engine_on, T.in_thr, T.eff_thr, act_game_pedal());
    {   // say it once when the game's channels disagree with the engine off (that is what the restart relies on then)
        static int said;
        float gp = act_game_pedal();
        if (!said && !T.engine_on && T.in_thr < 0.01f && (T.eff_thr > c_gas_edge() || (gp == gp && gp > c_gas_edge()))) {
            said = 1;
            logline("Cab: the engine is off and the game reports input_throttle %.2f, effective_throttle %.2f, the cruise function's pedal %.2f: the pedal is read from the largest of the three", T.in_thr, T.eff_thr, gp == gp ? gp : 0.0f);
        }
    }
    if (ss_on()) {
        in.ah_holding = ah_holding();
        in.air_psi = T.air_psi;
        in.air_warning = T.air_warning;
        in.lead_seen = cruise_lead(&in.lead_m, &in.lead_kmh);
    }
    // Auto Hold's own parking brake (cruise.cpp) is released by Auto Hold itself on gas; a press of ours in the same frame would
    // toggle it back on. It hands the brake over to us when the engine goes off.
    int ah_owns = ah_park_owned();
    if (p_act > 0) { in.parking = real; in.driver_changed = changed && !ours; }
    else { if (g_virt < 0) g_virt = real; in.parking = g_virt; }

    cab_cfg c = cab_cfg_default();
    c.ss_after_s = p_ss_after;
    c.ss_min_psi = p_ss_min_psi;
    c.ss_max_s = p_ss_max;
    cab_out o;
    cab_step(&g_st, in, c, &o);

    // --- auto parking brake ---
    if (p_auto > 0) {
        if (ah_owns && o.want == 0) {
            o.want = -1; o.term = CAT_AH_OWNS;
            snprintf(o.why, sizeof o.why, "driving off (gas, a gear, engine on) but Auto Hold owns the parking brake and lets it go on gas itself");
        }
        if (o.want >= 0 && o.want == g_last_want && p_act > 0) {
            // the same decision as before: no new line (the press and its check are toggle.h's below)
        } else if (o.want >= 0) {
            if (p_act > 0) {
                if (o.want != real) logline("Cab: %s the parking brake [%s]: %s", o.want ? "applying" : "releasing", cab_term_name(o.term), o.why);
                if (o.want) g_n_on++; else g_n_off++;
            } else {
                logline("Cab: would %s the parking brake [%s]: %s (the game's brake is %s%s)", o.want ? "apply" : "release", cab_term_name(o.term), o.why,
                        real ? "on" : "off", real == o.want ? ", already" : "");
                if (o.want) g_n_on++; else g_n_off++;
                g_virt = o.want;
            }
        }
        if (o.want < 0) {
            if (p_act > 0 && g_last_want >= 0) {
                logline("Cab: the decision to %s the parking brake is over [%s]: %s", g_last_want ? "apply" : "release", cab_term_name(o.term), o.why);
                g_last_term = o.term; g_last_term_at = now;
            } else if (o.term != CAT_NONE && o.term != g_last_term && now - g_last_term_at >= 10) {
                logv(1, "Cab: the parking brake holds [%s]: %s", cab_term_name(o.term), o.why);
                g_last_term = o.term; g_last_term_at = now;
            }
        }
        g_last_want = o.want;   // -1 (nothing to do, or the brake followed) lets the next decision log again
        if (p_act > 0) {
            if (o.want >= 0) g_pend = o.want;
            else if (g_tc.target < 0) g_pend = -1;
            int ev = 0;
            int press = toggle_step(&g_tc, now, real, g_pend, &ev);
            if (ev == 1) logv(1, "Cab: the parking brake followed our press");
            if (ev == -1) { logline("Cab: the parking brake did not follow %d presses of parkingbrake: not trying again for %.0f s", (int)TOGGLE_MAX_TRIES, TOGGLE_BACKOFF_S); g_pend = -1; }
            if (press && !input_press("parkingbrake", CAB_PRESS_MS)) {
                if (!g_nodev_said) { logline("Cab: cannot press parkingbrake: input device %s", input_device_state()); g_nodev_said = 1; }
                toggle_clear(&g_tc);
                g_tc.backoff_until = now + TOGGLE_BACKOFF_S;
                g_pend = -1;
            }
        }
    }

    // --- start-stop ---
    if (ss_on()) {
        if (o.ss_want >= 0 && o.ss_want == g_last_ss_want && p_act > 0) {
            // the same decision as before: no new line (the press and its check are toggle.h's below)
        } else if (o.ss_want >= 0) {
            if (p_act > 0) {
                if (o.ss_want != T.engine_on) logline("Cab: %s the engine [%s]: %s", o.ss_want ? "starting" : "stopping", cab_ss_term_name(o.ss_term), o.ss_why);
                if (o.ss_want) g_n_ss_start++; else g_n_ss_stop++;
            } else {
                logline("Cab: would %s the engine [%s]: %s (the engine is %s%s)", o.ss_want ? "start" : "stop", cab_ss_term_name(o.ss_term), o.ss_why,
                        T.engine_on ? "on" : "off", T.engine_on == o.ss_want ? ", already" : "");
                if (o.ss_want) g_n_ss_start++; else g_n_ss_stop++;
                g_veng = o.ss_want;
            }
        }
        if (o.ss_want < 0) {
            // the engine's own events (our stop happened, ended, or was dropped) are always written; the other holding terms once per change, at most every 10 s
            int event = o.ss_term == CST_STOPPED || o.ss_term == CST_ENGINE_BACK || o.ss_term == CST_STOP_DROPPED;
            if (p_act > 0 && g_last_ss_want >= 0 && !event) {
                logline("Cab: the decision to %s the engine is over [%s]: %s", g_last_ss_want ? "start" : "stop", cab_ss_term_name(o.ss_term), o.ss_why);
                g_last_ss_term = o.ss_term; g_last_ss_term_at = now;
            } else if (o.ss_term != CST_NONE && o.ss_term != g_last_ss_term && (event || now - g_last_ss_term_at >= 10)) {
                logv(1, "Cab: the start-stop holds [%s]: %s", cab_ss_term_name(o.ss_term), o.ss_why);
                g_last_ss_term = o.ss_term; g_last_ss_term_at = now;
            }
        }
        g_last_ss_want = o.ss_want;
        if (p_act > 0) {
            if (o.ss_want >= 0) g_pend_ss = o.ss_want;
            else if (g_tc_ss.target < 0) g_pend_ss = -1;
            int ev = 0;
            int press = toggle_step(&g_tc_ss, now, T.engine_on, g_pend_ss, &ev);
            if (ev == 1) logv(1, "Cab: the engine followed our press");
            // (a stop that never happened must not stay pending: an engine the driver switches off later is not ours to restart)
            if (ev == -1) { logline("Cab: the engine did not follow %d presses of %s: not trying again for %.0f s", (int)TOGGLE_MAX_TRIES, g_pend_ss ? "engine" : "ignitionoff", TOGGLE_BACKOFF_S); g_pend_ss = -1; g_st.ss_pend = 0; }
            if (g_ign_back_at > 0 && now >= g_ign_back_at) {
                g_ign_back_at = 0;
                if (!input_press("ignitionon", CAB_PRESS_MS)) logline("Cab: cannot press ignitionon: input device %s", input_device_state());
            }
            int stop = g_pend_ss == 0;
            if (press && stop) g_ign_back_at = now + CAB_IGN_BACK_S;
            // (a start that the 'engine' control did not bring about is tried once more with the starter itself, ignitionstrt, held to crank)
            const char *smix = stop ? "ignitionoff" : g_tc_ss.tries >= 1 ? "ignitionstrt" : "engine";
            if (press && !stop) logv(1, "Cab: starting the engine with %s", smix);
            if (press && !input_press(smix, stop ? CAB_PRESS_MS : g_tc_ss.tries >= 1 ? CAB_START_HOLD_MS : CAB_ENGINE_PRESS_MS)) {
                g_ign_back_at = 0;
                if (!g_nodev_said) { logline("Cab: cannot press %s: input device %s", smix, input_device_state()); g_nodev_said = 1; }
                toggle_clear(&g_tc_ss);
                g_tc_ss.backoff_until = now + TOGGLE_BACKOFF_S;
                g_pend_ss = -1;
            }
        }
    }
}

static void cab_reset(int level, const char *)
{
    cab_reset(&g_st);
    toggle_clear(&g_tc);
    toggle_clear(&g_tc_ss);
    g_tc_ss.verify_s = 4;
    g_virt = -1; g_prev_real = -1; g_pend = -1; g_last_want = -1; g_veng = -1; g_prev_eng = -1;
    g_pend_ss = -1; g_last_ss_want = -1; g_ign_back_at = 0; g_last_term = g_last_ss_term = 0;
    (void)level;
}

static void cab_start()
{
    cab_reset(&g_st);
    toggle_clear(&g_tc);
    toggle_clear(&g_tc_ss);
    g_tc_ss.verify_s = 4;
    if (p_auto <= 0 && !ss_on()) { snprintf(g_status, sizeof g_status, "off (ini)"); return; }
    char b[48];
    if (p_auto > 0) snprintf(b, sizeof b, "; parking brake on at engine off, off on drive-off");
    else b[0] = 0;
    snprintf(g_status, sizeof g_status, "%s%s%s", p_act > 0 ? "acting" : "log only", b,
             ss_on() ? "; start-stop at Auto Hold stops" : "");
}

static const char *cab_status() { return g_status; }

static void cab_stop()
{
    if (p_auto > 0 && (g_n_on || g_n_off))
        logline("Cab: %u apply and %u release decisions this session%s; presses ok %u, failed %u", g_n_on, g_n_off,
                p_act > 0 ? "" : " (log only)", g_tc.n_ok, g_tc.n_fail);
    if (ss_on() && (g_n_ss_stop || g_n_ss_start))
        logline("Cab: %u stop and %u start decisions this session%s; presses ok %u, failed %u", g_n_ss_stop, g_n_ss_start,
                p_act > 0 ? "" : " (log only)", g_tc_ss.n_ok, g_tc_ss.n_fail);
}

module m_cab = { "cab", "Cab", g_params, sizeof g_params / sizeof g_params[0], cab_start, cab_frame, NULL, cab_stop, cab_reset, cab_status, NULL, VCM_BOTH };
