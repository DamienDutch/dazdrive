// Two things about input.
// 1. Wheel name for the lane module's per-wheel profile (winmm joyGetDevCaps, works under Proton). The plugin reads no buttons
//    (the game's keys drive it) and there is no DirectInput: just a scan for the present joystick with the most buttons (the
//    wheel; pedals and shifters have fewer), every 5 s until one is found and then every 5 s for a change.
// 2. The SEMANTIC INPUT DEVICE (SCS input SDK 1.00): the plugin presses game controls itself (high beam, lift axle, ...) by
//    registering one device of type semantical whose inputs are named like the game's mixes (core/inputq.h has the table and
//    the event queue). The game pulls events from it every frame; modules call input_press / input_hold. Needs no binding in
//    the game: the input name is the mix name. [core] input_device 0 = no device.
#include "suite.h"
#include "inputq.h"
#include <mmsystem.h>
#include <string.h>

#include "scssdk_input.h"

enum { N_JOY = 16 };
static int g_present[N_JOY];
static double g_scan_at = -1e9;
static char g_wheel_name[64] = "";

const char *input_wheel_name() { return g_wheel_name; }

void input_poll()
{
    double now = wall_s();
    if (now - g_scan_at < 5) return;
    g_scan_at = now;
    for (int j = 0; j < N_JOY; j++) {
        JOYINFOEX ji;
        ji.dwSize = sizeof ji;
        ji.dwFlags = JOY_RETURNBUTTONS;
        int ok = joyGetPosEx(j, &ji) == JOYERR_NOERROR;
        if (ok == g_present[j]) continue;
        g_present[j] = ok;
        if (!ok) continue;
        logline("input: joystick %d found", j);
        JOYCAPSA caps;
        static int best_buttons;
        if (joyGetDevCapsA(j, &caps, sizeof caps) == JOYERR_NOERROR && (int)caps.wNumButtons > best_buttons) {
            best_buttons = caps.wNumButtons;
            snprintf(g_wheel_name, sizeof g_wheel_name, "%.60s", caps.szPname);
        }
    }
}

// ---------------------------------------------------------------- the semantic device

float g_input_device = 1;            // [core] input_device
static inq g_q;
static int g_dev_registered;         // the game accepted the device
static int g_dev_seen_active;        // it has been active at least once

static const char *result_name(scs_result_t r)
{
    switch (r) {
    case SCS_RESULT_ok: return "ok";
    case SCS_RESULT_unsupported: return "unsupported";
    case SCS_RESULT_invalid_parameter: return "invalid_parameter";
    case SCS_RESULT_already_registered: return "already_registered";
    case SCS_RESULT_not_found: return "not_found";
    case SCS_RESULT_unsupported_type: return "unsupported_type";
    case SCS_RESULT_not_now: return "not_now";
    case SCS_RESULT_generic_error: return "generic_error";
    default: return "other";
    }
}

static SCSAPI_VOID dev_active(const scs_u8_t active, const scs_context_t)
{
    inq_set_active(&g_q, active != 0);
    logline("input device: %s by the game", active ? "active" : "inactive");
    if (active) g_dev_seen_active = 1;
}

// Called repeatedly in each frame while the device is active, until it returns not_found.
static SCSAPI_RESULT dev_event(scs_input_event_t *const ev, const scs_u32_t flags, const scs_context_t)
{
    if (flags & SCS_INPUT_EVENT_CALLBACK_FLAG_first_after_activation) {   // active again: levels are sent anew, presses in flight are gone
        inq_set_active(&g_q, 0);
        inq_set_active(&g_q, 1);
        if (!g_dev_seen_active) logline("input device: the game is pulling events (first call)");
        g_dev_seen_active = 1;
    }
    if (flags & SCS_INPUT_EVENT_CALLBACK_FLAG_first_in_frame) inq_frame(&g_q, wall_s());
    int idx, val;
    if (!inq_pop(&g_q, &idx, &val)) return SCS_RESULT_not_found;
    memset(ev, 0, sizeof *ev);
    ev->input_index = (scs_u32_t)idx;
    ev->value_bool.value = val ? 1 : 0;
    return SCS_RESULT_ok;
}

SCSAPI_RESULT scs_input_init(const scs_u32_t version, const scs_input_init_params_t *const params)
{
    if (version != SCS_INPUT_VERSION_1_00) return SCS_RESULT_unsupported;
    const scs_input_init_params_v100_t *p = (const scs_input_init_params_v100_t *)params;
    boot_once();   // the game may call this before scs_telemetry_init: the log and the ini must be there
    if (strcmp(p->common.game_id, "eut2") && strcmp(p->common.game_id, "ats")) {
        logline("input device: not registered (game %s is not supported, ETS2/ATS only)", p->common.game_id);
        return SCS_RESULT_ok;
    }
    if (!(g_input_device > 0)) { logline("input device: off (ini [core] input_device=0)"); return SCS_RESULT_ok; }
    static scs_input_device_input_t inputs[IQ_N];
    for (int i = 0; i < IQ_N; i++) {
        inputs[i].name = IQ_MIXES[i];
        inputs[i].display_name = IQ_MIXES[i];
        inputs[i].value_type = SCS_VALUE_TYPE_bool;
    }
    scs_input_device_t dev;
    memset(&dev, 0, sizeof dev);
    dev.name = "dazdrive";
    dev.display_name = "DazDrive";
    dev.type = SCS_INPUT_DEVICE_TYPE_semantical;
    dev.input_count = IQ_N;
    dev.inputs = inputs;
    dev.input_active_callback = dev_active;
    dev.input_event_callback = dev_event;
    scs_result_t r = p->register_device(&dev);
    if (r == SCS_RESULT_ok) {
        g_dev_registered = 1;
        inq_set_active(&g_q, 1);   // taken as active from the start (the activity callback may come later or not at all); an unpulled press is dropped as stale after 2 s
        char list[600];
        size_t n = 0;
        list[0] = 0;
        for (int i = 0; i < IQ_N && n < sizeof list - 24; i++) n += snprintf(list + n, sizeof list - n, "%s%s", i ? " " : "", IQ_MIXES[i]);
        logline("input device: registered \"dazdrive\" (semantical, %d inputs, SDK result ok): %s", (int)IQ_N, list);
    } else {
        logline("input device: the game refused it (SDK result %d %s); presses are dropped, nothing else changes", (int)r, result_name(r));
    }
    return SCS_RESULT_ok;   // never fail hard: the telemetry side must go on
}

SCSAPI_VOID scs_input_shutdown(void)
{
    if (g_dev_registered)
        logline("input device: shut down (%u presses, %u events, %u refused, %u stale)", g_q.n_press, g_q.n_events, g_q.n_refused, g_q.n_stale);
    g_dev_registered = 0;
    inq_set_active(&g_q, 0);
}

// 1 = queued for the game's next frames. 0 = not possible now (no device, the game has not activated it, unknown mix); the
// caller logs that and carries on. hold_ms 0 = one frame high, then low.
int input_press(const char *mix, int hold_ms)
{
    if (!g_dev_registered) return 0;
    return inq_press(&g_q, mix, hold_ms, wall_s());
}

int input_hold(const char *mix, int on)
{
    if (!g_dev_registered) return 0;
    return inq_hold(&g_q, mix, on);
}

int input_device_ready() { return g_dev_registered && g_q.active; }
const char *input_device_state()
{
    return !g_dev_registered ? (g_input_device > 0 ? "not registered (refused, or not offered)" : "off (ini)")
                             : g_q.active ? "active" : g_dev_seen_active ? "registered, inactive now" : "registered, not activated by the game yet";
}
