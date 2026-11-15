// EBrake: the engine brake's stage keys work like a retarder lever. In the game the engine brake has an on/off toggle and a pair of
// stage keys (up / down); the stage keys only change a stage that is kept while the brake is off, so with only up / down on the wheel
// the brake could never be switched on or off. With this module: up from off switches it on at stage 1, up goes on to the next stage (to
// the engine's last stage and no further), down goes back one stage, and down at stage 1 switches it off.
//
// How: the module needs no ini key with a button number and presses nothing. It wraps the call with which the game's key handler
// consumes the player's engine brake up / down event (core/game.cpp game_ebrake_hook): that call runs before the handler acts on the
// press, so whatever buttons are bound to the two keys, the module sees each press with its direction. Where the handler's own rules
// would not give the lever behavior (up while the brake is off, down at stage 1) the module first puts the on byte and the stage where
// they do (core/ebrakelogic.h), through the game's own stage setter, and the handler then does the rest itself. Other presses (up while
// on, down above stage 1) are the game's own and untouched. Nothing happens while the game is paused; with the engine off only switching the brake OFF works (start-stop at a standstill must not leave it stuck on). A driver press that switches it on or raises a stage cancels the cruise, like a retarder lever.
//
// The descent control ([ebrake] descent): under cruise, a truck WITHOUT a retarder holds its speed on a descent with the engine brake stages
// (core/ebrakelogic.h has the rule and why: stages one at a time within a margin of where the game's own overspeed braking starts, off again
// near the set speed, never with the throttle on). Since WP-BR the control is the brake channel's engine-brake stage (core/brake.cpp, the
// second stage after what the truck does by itself, before the retarder and the service brake): this module only pushes its keys to the
// channel. It sets the game's own state (on byte, stage through the game's stage setter, the handler's flag) the way the lever does, from the
// actuator's ACT_POST tick inside the game's pedal function (game_ebrake_put in core/game.cpp has the call graph), stays out of the way of the
// driver (it only starts with the brake off and no key held, hands the brake back at once when the state is not what it left it -- a key
// press --, and puts the driver's previous state back when it ends). A truck with a retarder has the engine brake too: first, with the retarder for what it
// cannot hold (and, since WP-BR3, for every other event of the brake channel: core/brake.cpp, [brake] engine_brake). The gearbox follows by itself: the channel tells it, through the arbiter's standing request (core/gearreq.h), that it holds a stage,
// and TrueShift keeps the gear in the engine's brake range (the full range, whatever the stage).
//
// The first drives log what happens: one line per press the module changed, and (for the first 150 changes) the state it reads from the
// game, so the offsets can be checked against what the dashboard shows.
#include "../core/suite.h"
#include "../core/game.h"
#include "../core/sense.h"
#include "../core/act.h"
#include "../core/ebrakelogic.h"
#include "../core/vclass.h"
#include "../core/vehicle.h"
#include "../core/brake.h"
#include <string.h>
#include <math.h>

extern module m_ebrake;

static float p_lever, p_desc, p_margin, p_step, p_hold;
static param g_params[] = {
    { "lever", &p_lever, 1, "1 = the engine brake stage keys work like a retarder lever: the stage up key switches the brake on at stage 1 when it is off, then goes to the next stage (up to the engine's last stage); the stage down key goes back one stage and switches the brake off at stage 1. Works with whatever buttons are bound to the engine brake stage up / down keys in the game. 0 = the game's own behavior (the stage keys do nothing while the brake is off)" },
    { "descent", &p_desc, 1, "true = on a descent under cruise, a truck with an engine brake holds its speed with it: one stage at a time as the speed nears where the game's own overspeed braking would start, off again near the set speed, never with the throttle on. On a truck with a retarder too, the engine brake runs with the retarder (the engine brake first, the retarder for what it cannot hold). Gives the brake back at once when you press an engine brake key, and puts your own setting back afterwards. false = the game's own (all stages at once)" },
    { "descent_margin_kmh", &p_margin, 0.5f, "the first stage comes this far (km/h) under the speed where the game's own overspeed braking starts (the set speed plus the descent overspeed)" },
    { "descent_step_s", &p_step, 2.0f, "at least this many seconds between two stages up; a stage down waits twice as long (no on / off pumping, and a gear change in between)" },
    { "descent_hold_s", &p_hold, 20, "when the engine brake comes off at the end of a stretch of descent, the gearbox keeps the gear for this many seconds (no shifting up just to shift down again at the next stage); 0 = the gearbox decides at once" },
};

static int g_hooked;
static unsigned g_n_up, g_n_down, g_n_changed;
static int g_probe_left = 150;
static int g_pv = -1, g_p_held, g_p_on, g_p_flag;
static unsigned g_p_stage, g_p_cap;
static char g_status[120] = "off";

// Inside the game's key handler, before it acts on the press (the game's thread).
static void on_press(char *ga, int dir)
{
    if (p_lever <= 0 || !module_active(&m_ebrake)) return;
    int held, on, flag;
    unsigned stage;
    if (!sense_ebrake_state(ga, &held, &on, &stage, &flag)) return;
    unsigned cap = sense_ebrake_cap(ga);
    eb_state s = { on, stage, cap };
    eb_pre p = eb_before_press(dir, s, T.paused, T.engine_on);
    if (dir > 0) g_n_up++; else g_n_down++;
    // the driver's lever up (on, or a stage higher) cancels the cruise, as a retarder lever does in the game. Our own descent control never
    // comes through here (it sets the stage through the setter, not through the key). Cruise sees the drop by itself.
    if (p.cancel_cruise && T.cruise > 0.5f) {
        act_cruise_on(ga, 0);
        logline("EBrake: up pressed with the cruise on: cruise cancelled (the engine brake lever cancels it, like a retarder lever)");
    }
    if (!p.act) {
        if (g_probe_left > 0) {
            g_probe_left--;
            logline("EBrake: %s pressed, brake %s at stage %u of %u (flag %d%s): left to the game", dir > 0 ? "up" : "down", on ? "on" : "off", stage, cap, flag,
                    T.paused ? ", paused" : !T.engine_on ? ", engine off" : cap < 1 || cap > EB_CAP_MAX ? ", no usable stage count" : "");
        }
        return;
    }
    g_n_changed++;
    act_ebrake_put(NULL, ga, p.on, p.stage);   // (the game's key handler calls this: nothing is logged from there)
    logline("EBrake: %s pressed, brake %s at stage %u of %u (flag %d): %s", dir > 0 ? "up" : "down", on ? "on" : "off", stage, cap, flag,
            p.on ? "switched on, the game takes it to stage 1" : "switched off");
}

// The probe: the state the game holds, logged when it changes (the first changes of a session only).
static int s_desc_in;   // the stage setter and the actuator hook are there: the channel may run the descent control
static void eb_frame()
{
    brake_eb_config(s_desc_in && p_desc > 0 ? 1.0f : 0.0f, p_margin, p_step, p_hold);   // (the descent control is the brake channel's, every frame the module says it is on)
    if (!g_hooked || g_probe_left <= 0) return;
    char *ga = sense_actor();
    int held, on, flag;
    unsigned stage;
    if (!ga || !sense_ebrake_state(ga, &held, &on, &stage, &flag)) { g_pv = -1; return; }
    unsigned cap = sense_ebrake_cap(ga);
    if (g_pv >= 0 && held == g_p_held && on == g_p_on && stage == g_p_stage && flag == g_p_flag && cap == g_p_cap) return;
    int first = g_pv < 0 || cap != g_p_cap;
    g_pv = 1; g_p_held = held; g_p_on = on; g_p_stage = stage; g_p_flag = flag; g_p_cap = cap;
    g_probe_left--;
    logline("EBrake: state held %d on %d stage %u of %u flag %d%s", held, on, stage, cap, flag, first ? " (new vehicle or engine)" : "");
}

// ---------------------------------------------------------------- the descent control
// Moved to the brake channel (core/brake.cpp, WP-BR): the engine-brake stage is the channel's second stage, between what the truck does by itself and the service brake. This
// module keeps the lever keys (the driver's input) and the ini keys of the descent control, which it pushes to the channel every frame (brake_eb_config): whether the
// control is on, its margin, the least seconds between two stages up, how long the gearbox keeps its gear after the stages came off. The control needs the stage setter and the
// actuator hook: found here, once, at start.

static void eb_reset(int, const char *) { g_pv = -1; }

static void eb_start()
{
    g_pv = -1;
    if (p_desc > 0) {
        const char *why = NULL;
        if (!act_hooked()) logline("EBrake: descent control not installed: no actuator hook");
        else if (!act_ebrake_setter(&why)) logline("EBrake: descent control not installed: %s", why ? why : "?");
        else {
            s_desc_in = 1;
            brake_attach();   // (the channel's tick runs the control, also with [brake] channel 0)
            logline("EBrake: descent control in place (run by the brake channel): with the cruise on and no retarder, the engine brake stages hold the speed on a descent (margin %.1f km/h, %.1f s between steps up)", p_margin, p_step);
        }
    }
    if (p_lever <= 0) { snprintf(g_status, sizeof g_status, s_desc_in ? "descent control" : "off (ini)"); return; }
    const char *why = NULL;
    g_hooked = act_ebrake_hook(on_press, &why);
    if (!g_hooked) {
        snprintf(g_status, sizeof g_status, "not installed: %s", why ? why : "?");
        logline("EBrake: lever not installed: %s; the engine brake keys stay the game's", why ? why : "?");
        return;
    }
    snprintf(g_status, sizeof g_status, s_desc_in ? "lever on the stage keys, descent control" : "lever on the stage keys");
    const sense_eb_layout el = sense_ebrake_layout();
    logline("EBrake: lever in place (layout %s): up from off = stage 1, up = next stage, down = previous stage, down at stage 1 = off. Game offsets: held +0x%x, on +0x%x, stage +0x%x, handler flag +0x%x, up key +0x%x, down key +0x%x, stage count in the engine +0x%x",
            el.name, el.held, el.on, el.stage, el.flag, el.up, el.dwn, el.cap);
}

static void eb_stop()
{
    if (g_hooked) logline("EBrake: %u up and %u down presses seen this session, %u changed by the lever", g_n_up, g_n_down, g_n_changed);
}

static const char *eb_status() { return g_status; }

module m_ebrake = { "ebrake", "EBrake", g_params, sizeof g_params / sizeof g_params[0], eb_start, eb_frame, NULL, eb_stop, eb_reset, eb_status, NULL, VCM_BOTH };
