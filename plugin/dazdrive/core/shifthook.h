// The shared plumbing of the gear brains (core/shifthook.cpp). The game's automatic gearbox has ONE set of places where a brain gets in: the
// window call of its update (the decision), the gear chooser, the start-gear function, the tail of the update and the stop threshold.
// Each is hooked once, here. A hook asks which brain belongs to the active vehicle class (core/vclass.h) and hands over to it: TrueShift
// (modules/trueshift.cpp) for trucks, CarShift (modules/carshift.cpp) for cars. A class with no brain, or one that is off in the ini,
// is left to the game's own automatic: the hook passes straight through.
#pragma once
#include "suite.h"
#include "game.h"

struct shift_brain {
    const char *name;
    unsigned classes;                   // VCM_TRUCK or VCM_CAR: the class this brain drives
    int (*is_on)();                     // the brain has the gearbox (on in the ini, hooks in): the hook may hand over
    // The hook bodies. The game's own call has already run: stock is what it returned.
    void (*on_window)(void *self);                      // the update's decision window (decide and force a shift here)
    int (*on_choose)(void *self, int stock);            // the gear chooser: the gear to return
    int (*on_start)(void *self, int stock);             // the start-gear function: the gear to start in
    void (*on_tail)(void *self);                        // the end of the update (neutral watchdog, eco-roll)
    float (*stop_threshold)(float idle, float thr, float brake);   // the automatic's stop threshold (rpm), only if the stop hook is in
    // What the other modules ask of the brain of the class (suite.h: ts_*)
    int (*start_gear)(float thr);       // the start gear by physics for this pedal (0 = unknown)
    float (*idle_rpm)();                // the idle rpm (learned, else the ini's)
    int (*mode)();                      // the driving mode (MODE_*), -1 = manual box, M or unsupported
    void (*commanded)(int gear);        // the owner changed the gear at a standstill: the shift is ours
    int (*view)(ts_view_t *out);        // the dashboard's view (0 = the brain does not drive this gearbox now)
};

void shift_register(const shift_brain *b);       // a brain announces itself (a static object per brain file)
const shift_brain *shift_active();               // the registered brain of the active vehicle class, NULL = none

// Installation, idempotent: the first call puts the hooks in, a later one reports what the first did.
// *done / *want: how many of the window, chooser and start call sites went in. Returns 1 = all of them (the brains may act).
int shift_hooks_install(int *done, int *want);
int shift_hooks_ok();                            // the window, chooser and start hooks are in
int shift_hook_stop();                           // puts the stop threshold hook in (once); 1 = it is in
int shift_hook_tail();                           // puts the update's tail hook in (once, needs the gearbox engage code); 1 = it is in
int shift_tail_engage_ok();                      // the gearbox engage code was found (the neutral watchdog can engage a gear)
