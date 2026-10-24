// Wheel name for the lane module's per-wheel profile (winmm joyGetDevCaps, works under Proton).
// The plugin has no buttons of its own (the game's keys drive it), so nothing here reads button states
// and there is no DirectInput: just a scan for the present joystick with the most buttons (the wheel;
// pedals and shifters have fewer), every 5 s until one is found and then every 5 s for a change.
#include "suite.h"
#include <mmsystem.h>

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
