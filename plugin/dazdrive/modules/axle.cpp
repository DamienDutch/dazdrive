// Axle: raises a lift axle by itself when the load allows it (rule and thresholds: core/axlelogic.h). The game lowers a lift axle on load,
// in the workshop and at a coupling, and never raises it again; a real truck does.
//
// LOG ONLY by default ([axle] act 0): the module reads the control from the telemetry (truck.lift_axle, truck.trailer.lift_axle: "the
// control is set to lifted"), decides, and writes "would raise / would lower" lines with the reason. With act 1 it presses the game's own
// mix through the plugin's input device (core/input.cpp): liftaxle for the truck, liftaxlet for the trailer, and checks in the telemetry
// that the control followed (core/toggle.h: one retry, then 30 s of silence). A change of the control that was not ours (the driver, the
// game) is left alone for 60 s; when the game lowers it after a load change, its mass becomes a cap for the limit (axlelogic.h).
//
// What the telemetry gives: the control state of both lift axles (SDK 1.17+), the wheels' lift 0..1, the wheel positions (axles are
// counted from them), which wheels are liftable. Wheel slip is the mean of the driven, lowered wheels' rolling speed against the truck's.
// Not known to the SDK: the axle loads. The mass is the vehicle model's (V.mass, truck + trailers) and the limit is the legal gross weight
// of the axles that would stay down, so a light truck raises and a heavy one does not.
//
// The log (promise P7): a line for every decision change, "would raise / would lower [term]: value against threshold", a line when a decision
// ends (the term that ended it) and, at log level 1, a line when the term that holds a decision back changes (at most one per 10 s per axle).
#include "../core/suite.h"
#include "../core/vehicle.h"
#include "../core/axlelogic.h"
#include "../core/toggle.h"
#include <math.h>
#include <string.h>

static float p_auto, p_act, p_frac, p_min, p_steady, p_grade, p_slip;

static param g_params[] = {
    { "auto_up", &p_auto, 1, "1 = watch the lift axles (the truck's and the trailer's) and decide when to raise or lower them by the load and the driving; see act for what is done with the decision. 0 = off" },
    { "act", &p_act, 0, "0 = log only: 'would raise' and 'would lower' lines with the reason, nothing is pressed. 1 = press the game's lift axle control through the plugin's input device ([core] input_device) when the decision changes, and check that it took. The game lowers the axle itself on load and never raises it" },
    { "raise_pct", &p_frac, 80, "raise a lift axle when the mass is at most this share (%) of the legal gross weight of the axles that would stay down (EU: 2 axles 18 t, 3 axles 26 t, 4 axles 32 t; with a trailer 3 axles 28 t, 4 axles 36 t, 5 or more 40 t). Lowered again over 100 %" },
    { "min_kmh", &p_min, 20, "raise only at a steady speed over this (km/h); under it a slipping truck gets the axle lowered" },
    { "steady_s", &p_steady, 5, "speed, grade and mass must have been steady this long (s) before a raise" },
    { "max_grade_pct", &p_grade, 4, "no raising on a road steeper than this (%)" },
    { "slip_pct", &p_slip, 10, "driven-wheel slip (%) counted as traction trouble: no raising for 5 s, and the axle is lowered at low speed" },
};

static axle_state g_st;
static toggle_ctl g_tc[2];
static int g_prev_up[2] = { -1, -1 }, g_last_want[2] = { -1, -1 }, g_pend[2] = { -1, -1 };
static int g_nodev_logged, g_inferred[2], g_cfg_logged;
static double g_last_status, g_last_remind[2], g_last_term_log[2];
static int g_last_term[2];          // the last term a line was written for while nothing was decided (hold terms: one line per change, at most one per 10 s)
static unsigned g_n_would[2][2];    // [axle][0 lower, 1 raise]
static char g_status[96] = "off";
static const char *const NAME[2] = { "truck", "trailer" };
static const char *const MIX[2] = { "liftaxle", "liftaxlet" };

static int axles_of(int a)
{
    if (a == 0) { int n = axl_count(T.wheel_z, NULL, T.wheels); return n >= 0 ? n : (T.wheels + 1) / 2; }
    if (!T.trailer) return 0;
    int n = axl_count(T.t_wheel_z, NULL, T.t_wheels);
    return n >= 0 ? n : (T.t_wheels + 1) / 2;
}

static int liftable_of(int a)
{
    int n = a == 0 ? axl_count(T.wheel_z, T.wheel_liftable, T.wheels) : axl_count(T.t_wheel_z, T.trailer_wheel_liftable, T.t_wheels);
    if (n > 0) return n;
    // the game did not say which wheels lift: a wheel that has been seen lifted does (one axle assumed)
    if (g_inferred[a]) return 1;
    return 0;
}

static float driven_slip()
{
    double sum = 0;
    int n = 0;
    for (int i = 0; i < T.wheels && i < 16; i++) {
        // radius > 0.1 m: a radius under 10 cm is a missing value (the smallest truck wheel is about 0.4 m); lift is 0..1 (SDK), over half = off the road
        if (!T.wheel_powered[i] || !(T.wheel_radius[i] > 0.1f) || T.wheel_w[i] != T.wheel_w[i]) continue;
        if (T.wheel_lift[i] == T.wheel_lift[i] && T.wheel_lift[i] > 0.5f) continue;   // lifted: not on the road
        sum += fabsf(T.wheel_w[i]) * 6.2831853 * T.wheel_radius[i];
        n++;
    }
    if (!n) return NAN;
    float v = fabsf(T.speed);
    // a floor under the speed: a wheel that spins at a standstill is a big number, not a division by zero. 2 m/s (7 km/h) = a walking-pace
    // truck, under the 20 km/h the raise needs; it only matters for the lowering on slip at low speed
    return (float)((sum / n - v) / fmaxf(v, 2.0f));
}

static void ax_note_config()
{
    if (g_cfg_logged) return;
    logline("Axle: truck %d axles (%d wheels), lift axles %d%s; trailer %s", axles_of(0), T.wheels, liftable_of(0),
            axl_count(T.wheel_z, T.wheel_liftable, T.wheels) > 0 ? "" : " (none reported by the game)",
            T.trailer ? "coupled" : "not coupled");
    g_cfg_logged = 1;
}

static void ax_frame()
{
    if (p_auto <= 0) return;
    double now = wall_s();
    unsigned seen = telemetry_seen();
    // a lifted wheel in the telemetry shows a lift axle even where the config does not name one (0.05 = the noise floor of the 0..1 channel)
    for (int i = 0; i < 16; i++) if (T.wheel_lift[i] == T.wheel_lift[i] && T.wheel_lift[i] > 0.05f) g_inferred[0] = 1;
    for (int i = 0; i < 12; i++) if (T.t_wheel_lift[i] == T.t_wheel_lift[i] && T.t_wheel_lift[i] > 0.05f) g_inferred[1] = 1;

    axle_in in;
    memset(&in, 0, sizeof in);
    in.t = now;
    in.ok = T.engine_on && !T.paused;
    in.kmh = fabsf(T.speed) * 3.6f;
    in.accel = T.accel;
    in.grade_pct = V.pitch_pct;
    in.mass_kg = V.mass; in.mass_ok = V.mass_ok;
    in.slip = driven_slip();
    in.has_trailer = T.trailer;
    for (int a = 0; a < 2; a++) {
        in.axles[a] = axles_of(a);
        in.liftable[a] = liftable_of(a);
        in.known[a] = (seen & (a ? 2u : 1u)) != 0 && (a == 0 || T.trailer);
        in.up[a] = a ? T.trailer_lift_axle : T.lift_axle;
        int changed = in.known[a] && g_prev_up[a] >= 0 && g_prev_up[a] != in.up[a];
        in.ours[a] = changed && toggle_ours(&g_tc[a], now);
        if (changed) logv(1, "Axle: the %s lift axle control is now %s%s", NAME[a], in.up[a] ? "lifted" : "down", in.ours[a] ? " (after our press)" : "");
        g_prev_up[a] = in.known[a] ? in.up[a] : -1;
    }
    ax_note_config();

    axle_cfg c = axle_cfg_default();
    c.raise_frac = p_frac / 100.0f; c.min_kmh = p_min; c.steady_s = p_steady; c.grade_max_pct = p_grade; c.slip_max = p_slip / 100.0f;
    axle_dec d;
    axle_step(&g_st, in, c, &d);

    for (int a = 0; a < 2; a++) {
        if (d.learned[a])
            logline("Axle: the game lowered the %s lift axle at %.1f t right after a load change: that mass now caps the limit", NAME[a], g_st.cap_kg[a] / 1000);
        int w = d.want[a];
        // P7: a line when the decision changes (and every 60 s while it stands) naming the term that decided, with its value and threshold;
        // when it ends, the term that ended it; while nothing is decided, a line when the term that holds it back changes (at most one per 10 s)
        if (w != g_last_want[a] || (w >= 0 && now - g_last_remind[a] > 60)) {
            if (w >= 0) {
                if (w != g_last_want[a]) g_n_would[a][w]++;
                logline("Axle: %s the %s lift axle [%s]: %s%s", p_act > 0 ? (w ? "raising" : "lowering") : (w ? "would raise" : "would lower"), NAME[a],
                        axle_term_name(d.term[a]), d.why[a], p_act > 0 ? "" : " (log only)");
            } else {
                logline("Axle: the %s lift axle: the decision to %s is over [%s]: %s", NAME[a], g_last_want[a] ? "raise" : "lower", axle_term_name(d.term[a]), d.why[a]);
            }
            g_last_want[a] = w;
            g_last_remind[a] = now;
            g_last_term[a] = d.term[a];
        } else if (w < 0 && d.term[a] != AXT_NONE && d.term[a] != g_last_term[a] && now - g_last_term_log[a] >= 10) {
            logv(1, "Axle: the %s lift axle holds [%s]: %s", NAME[a], axle_term_name(d.term[a]), d.why[a]);
            g_last_term[a] = d.term[a];
            g_last_term_log[a] = now;
        }
        if (p_act > 0) {
            if (w >= 0 && g_pend[a] < 0) g_pend[a] = w;
            if (g_pend[a] >= 0 && (!in.known[a] || in.up[a] == g_pend[a])) {
                if (in.known[a] && in.up[a] == g_pend[a]) axle_acted(&g_st, a, now);
                g_pend[a] = -1;
            }
            int ev = 0;
            int press = toggle_step(&g_tc[a], now, in.known[a] ? in.up[a] : -1, g_pend[a], &ev);
            if (ev == 1) logline("Axle: the %s lift axle followed our press", NAME[a]);
            if (ev == -1) { logline("Axle: the %s lift axle did not follow %u presses of %s: not trying again for %.0f s", NAME[a], (unsigned)TOGGLE_MAX_TRIES, MIX[a], TOGGLE_BACKOFF_S); g_pend[a] = -1; }
            if (press) {
                axle_acted(&g_st, a, now);
                if (!input_press(MIX[a])) {
                    if (!g_nodev_logged) { logline("Axle: cannot press %s: input device %s", MIX[a], input_device_state()); g_nodev_logged = 1; }
                    toggle_clear(&g_tc[a]);
                    g_tc[a].backoff_until = now + TOGGLE_BACKOFF_S;
                    g_pend[a] = -1;
                } else logline("Axle: pressed %s (%s the %s lift axle)", MIX[a], g_pend[a] ? "raise" : "lower", NAME[a]);
            }
        }
    }
    // the status line when an axle's state or reason changes (it was every 20 s: a ride of 20 min logged 145 identical pairs)
    if (now - g_last_status > 20 && T.engine_on && !T.paused && (in.liftable[0] > 0 || in.liftable[1] > 0)) {
        g_last_status = now;
        static int s_key[2] = {-99, -99};
        for (int a = 0; a < 2; a++) {
            const int key = in.liftable[a] > 0 ? (in.known[a] ? in.up[a] : 2) * 64 + (int)d.term[a] : -1;
            if (key == s_key[a]) continue;
            s_key[a] = key;
            if (in.liftable[a] > 0)
                logv(2, "Axle: %s %s, %.1f t, %.0f km/h, slip %.1f %% [%s]: %s", NAME[a], in.known[a] ? (in.up[a] ? "up" : "down") : "state unknown",
                     in.mass_kg / 1000, in.kmh, in.slip == in.slip ? in.slip * 100 : -1.f, axle_term_name(d.term[a]), d.why[a]);
        }
    }
}

static void ax_config(const scs_telemetry_configuration_t *c)
{
    if (!c) return;
    if (strcmp(c->id, "truck") && strcmp(c->id, "trailer.0")) return;
    g_cfg_logged = 0;
    g_inferred[!strcmp(c->id, "truck") ? 0 : 1] = 0;
    if (!strcmp(c->id, "trailer.0") && T.t_wheels > 0)
        logline("Axle: trailer %d axles (%d wheels), lift axles %d%s", axles_of(1), T.t_wheels, liftable_of(1),
                axl_count(T.t_wheel_z, T.trailer_wheel_liftable, T.t_wheels) > 0 ? "" : " (none reported by the game)");
}

static void ax_reset(int level, const char *)
{
    if (level >= RESET_LOAD) axle_load_changed(&g_st, wall_s());
    if (level >= RESET_HARD) {
        axle_reset(&g_st, 1);
        for (int a = 0; a < 2; a++) { toggle_clear(&g_tc[a]); g_prev_up[a] = g_last_want[a] = g_pend[a] = -1; g_inferred[a] = 0; g_last_term[a] = 0; }
        g_cfg_logged = 0;
    }
    if (level == RESET_LOAD) { g_inferred[1] = 0; g_prev_up[1] = g_last_want[1] = g_pend[1] = -1; toggle_clear(&g_tc[1]); }
}

static void ax_start()
{
    axle_reset(&g_st, 1);
    for (int a = 0; a < 2; a++) toggle_clear(&g_tc[a]);
    if (p_auto <= 0) { snprintf(g_status, sizeof g_status, "off (ini)"); return; }
    snprintf(g_status, sizeof g_status, "%s (raise at %.0f %% of the legal weight)", p_act > 0 ? "acting" : "log only", p_frac);
}

static const char *ax_status() { return g_status; }

static void ax_stop()
{
    if (p_auto > 0)
        logline("Axle: %u / %u would-raise / would-lower decisions for the truck's axle, %u / %u for the trailer's; presses ok %u, failed %u",
                g_n_would[0][1], g_n_would[0][0], g_n_would[1][1], g_n_would[1][0], g_tc[0].n_ok + g_tc[1].n_ok, g_tc[0].n_fail + g_tc[1].n_fail);
}

module m_axle = { "axle", "Axle", g_params, sizeof g_params / sizeof g_params[0], ax_start, ax_frame, ax_config, ax_stop, ax_reset, ax_status, NULL, VCM_TRUCK };
