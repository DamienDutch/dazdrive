// Wheel and joystick buttons, read directly (winmm joyGetPosEx, works under
// Proton), so modules can have their own buttons (numbers in the ini).
// Buttons are numbered from 1 like Windows' game controller panel; every new
// press is written to the log (first 40), so the number of a button is easy
// to find: press it once and look in dazdrive.log.
// winmm sees only buttons 1-32; buttons 33-128 (big wheels, button boxes) come
// from DirectInput 8, read on its own thread so a device scan never stalls a
// frame. The two numberings agree: winmm sits on DirectInput/HID in Windows and Wine.
#define DIRECTINPUT_VERSION 0x0800
#include "suite.h"
#include <mmsystem.h>
#include <dinput.h>
#include <atomic>

enum { N_JOY = 16 };
static DWORD g_now[N_JOY], g_was[N_JOY];
static int g_present[N_JOY];
static double g_scan_at = -1e9, g_poll_at;
static int g_logged;
static char g_wheel_name[64] = "";

const char *input_wheel_name() { return g_wheel_name; }

// ---- DirectInput: buttons 33..128, as 3 words per device (bits 32..127)
enum { N_HI = 3 };
static std::atomic<uint32_t> g_di_shared[N_JOY][N_HI];  // written by the DI thread
static uint32_t g_hi_now[N_JOY][N_HI], g_hi_was[N_JOY][N_HI];
static IDirectInput8A *g_di;
static IDirectInputDevice8A *g_dev[N_JOY];
static int g_n_dev;

static BOOL CALLBACK di_enum(LPCDIDEVICEINSTANCEA inst, LPVOID)
{
    if (g_n_dev >= N_JOY) return DIENUM_STOP;
    IDirectInputDevice8A *d;
    if (FAILED(g_di->CreateDevice(inst->guidInstance, &d, NULL))) return DIENUM_CONTINUE;
    DIDEVCAPS caps;
    caps.dwSize = sizeof caps;
    // only devices with more than 32 buttons: the rest is winmm's
    if (FAILED(d->GetCapabilities(&caps)) || caps.dwButtons <= 32 || FAILED(d->SetDataFormat(&c_dfDIJoystick2))) {
        d->Release();
        return DIENUM_CONTINUE;
    }
    d->SetCooperativeLevel(NULL, DISCL_BACKGROUND | DISCL_NONEXCLUSIVE);  // the default; NULL is refused on some builds, harmless
    logline("input: %.60s has %lu buttons, 33 and up read via DirectInput", inst->tszProductName, (unsigned long)caps.dwButtons);
    g_dev[g_n_dev++] = d;
    return DIENUM_CONTINUE;
}

static DWORD WINAPI di_thread(LPVOID)
{
    HMODULE m = LoadLibraryA("dinput8.dll");
    typedef HRESULT(WINAPI * create_fn)(HINSTANCE, DWORD, REFIID, LPVOID *, LPUNKNOWN);
    create_fn create = m ? (create_fn)(void *)GetProcAddress(m, "DirectInput8Create") : NULL;
    if (!create || FAILED(create(GetModuleHandleA(NULL), DIRECTINPUT_VERSION, IID_IDirectInput8A, (void **)&g_di, NULL))) {
        logline("input: DirectInput not available, only buttons 1-32");
        return 0;
    }
    double scan_at = -1e9;
    int lost = 0;
    for (;;) {
        double now = wall_s();
        // look again every 5 s while nothing was found, or when a device stopped answering (unplugged)
        if ((g_n_dev == 0 || lost) && now - scan_at > 5) {
            lost = 0;
            scan_at = now;
            for (int j = 0; j < g_n_dev; j++) { g_dev[j]->Unacquire(); g_dev[j]->Release(); }
            g_n_dev = 0;
            g_di->EnumDevices(DI8DEVCLASS_GAMECTRL, di_enum, NULL, DIEDFL_ATTACHEDONLY);
            for (int j = g_n_dev; j < N_JOY; j++)
                for (int w = 0; w < N_HI; w++) g_di_shared[j][w] = 0;
        }
        for (int j = 0; j < g_n_dev; j++) {
            DIJOYSTATE2 st;
            uint32_t bits[N_HI] = {};
            g_dev[j]->Acquire();
            g_dev[j]->Poll();
            if (SUCCEEDED(g_dev[j]->GetDeviceState(sizeof st, &st))) {
                for (int b = 32; b < 128; b++)
                    if (st.rgbButtons[b] & 0x80) bits[b / 32 - 1] |= 1u << (b % 32);
            } else lost = 1;
            for (int w = 0; w < N_HI; w++) g_di_shared[j][w] = bits[w];
        }
        Sleep(10);
    }
}

// button b (1..128) down in this poll (now) or the previous one
static int hi_down(int b, int now)
{
    int w = (b - 1) / 32 - 1, bit = (b - 1) % 32;
    for (int j = 0; j < N_JOY; j++)
        if ((now ? g_hi_now : g_hi_was)[j][w] & (1u << bit)) return 1;
    return 0;
}

void input_poll()
{
    double now = wall_s();
    if (now - g_poll_at < 0.02) return;  // 50 Hz
    g_poll_at = now;
    int rescan = now - g_scan_at > 5;  // look for (re)connected devices every 5 s
    if (rescan) g_scan_at = now;
    static int di_started;
    if (!di_started) {
        di_started = 1;
        HANDLE t = CreateThread(NULL, 0, di_thread, NULL, 0, NULL);
        if (t) CloseHandle(t);
    }
    static int hi_logged[N_JOY][N_HI];
    for (int j = 0; j < N_JOY; j++)
        for (int w = 0; w < N_HI; w++) {
            g_hi_was[j][w] = g_hi_now[j][w];
            g_hi_now[j][w] = g_di_shared[j][w];
            uint32_t pressed = g_hi_now[j][w] & ~g_hi_was[j][w];
            for (int b = 0; pressed && b < 32; b++)
                if ((pressed & (1u << b)) && (g_logged < 40 || !(hi_logged[j][w] & (1u << b)))) {
                    logline("input: device %d button %d pressed (DirectInput)", j, 32 * (w + 1) + b + 1);
                    g_logged++;
                    hi_logged[j][w] |= 1u << b;
                }
        }
    for (int j = 0; j < N_JOY; j++) {
        if (!g_present[j] && !rescan) continue;
        JOYINFOEX ji;
        ji.dwSize = sizeof ji;
        ji.dwFlags = JOY_RETURNBUTTONS;
        int ok = joyGetPosEx(j, &ji) == JOYERR_NOERROR;
        if (ok != g_present[j]) {
            g_present[j] = ok;
            if (ok) {
                logline("input: joystick %d found", j);
                // the wheel = the present device with the most buttons (pedals and shifters have fewer)
                JOYCAPSA caps;
                static int best_buttons;
                if (joyGetDevCapsA(j, &caps, sizeof caps) == JOYERR_NOERROR && (int)caps.wNumButtons > best_buttons) {
                    best_buttons = caps.wNumButtons;
                    snprintf(g_wheel_name, sizeof g_wheel_name, "%.60s", caps.szPname);
                }
            }
        }
        g_was[j] = g_now[j];
        g_now[j] = ok ? ji.dwButtons : 0;
        DWORD pressed = g_now[j] & ~g_was[j];
        // the first 40 presses, and after that every button the first time it is pressed (to find numbers)
        static DWORD seen[N_JOY];
        for (int b = 0; pressed && b < 32; b++)
            if ((pressed & (1u << b)) && (g_logged < 40 || !(seen[j] & (1u << b)))) {
                logline("input: joystick %d button %d pressed", j, b + 1);
                g_logged++;
                seen[j] |= 1u << b;
            }
    }
}

int input_held(int b)
{
    if (b > 32 && b <= 128) return hi_down(b, 1);
    if (b < 1 || b > 32) return 0;
    for (int j = 0; j < N_JOY; j++)
        if (g_now[j] & (1u << (b - 1))) return 1;
    return 0;
}

int input_pressed(int button, int modifier)
{
    if (button > 32 && button <= 128)
        return hi_down(button, 1) && !hi_down(button, 0) && (modifier <= 0 || input_held(modifier));
    if (button < 1 || button > 32) return 0;
    for (int j = 0; j < N_JOY; j++) {
        DWORD bit = 1u << (button - 1);
        if ((g_now[j] & bit) && !(g_was[j] & bit) && (modifier <= 0 || input_held(modifier))) return 1;
    }
    return 0;
}
