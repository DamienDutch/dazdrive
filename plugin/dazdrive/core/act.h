// The act layer's writes to the game that are not pedals (WP-S step S5). The pedals and the brake request are posted to the actuator (act_brake, act_thr_*,
// act_clutch in core/suite.h, merged once per tick by core/actuate.cpp). Everything else a module used to write itself is one function here: the cruise's
// on byte and set speed, the ACC target, the hazards, the parking brake, the LKA bit and output, the engine brake lever, the settings cvars, the gearbox's
// target / force / at-rest state, and the installers of the update's tail hook. A module that writes the game calls one of these; the layer lint
// (tools/check_layers.py, rule `act`) fails a module that writes a cast pointer or calls the game_* / gb_* writers itself.
//
// Like core/sense.h: header-inline (the sims need no other source file and run exactly the expression the game would get), the same expression, type and order
// the module had, nothing added to a write's timing. The `who` of the functions that take one is the module's name: the first writer of an actuator and
// every change of writer is logged (level 2, "Actuator: ..."), so that a log shows who held the cvar or the lever. Where the call runs inside a stub on the
// game's thread, pass NULL: nothing is logged from there.
#pragma once
#include "suite.h"
#include "game.h"
#include "sense.h"
#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------- who wrote it
// key: the actuator (a cvar object, or ACT_KEY_* cast to a pointer); what: its name for the log.
enum { ACT_KEY_EBRAKE = 1, ACT_KEY_LKA_U = 2, ACT_KEY_LKA_BIT = 3 };
inline void act_note_writer(const void *key, const char *what, const char *who)
{
    if (!who) return;
    static const void *keys[16];
    static const char *whos[16];
    static int n;
    static char busy;   // two module threads may write at once: a first-write log line is not worth a wait, so a call that finds it busy skips the note
    if (__atomic_test_and_set(&busy, __ATOMIC_ACQUIRE)) return;
    // (g_log_level, not logv: logv adds the module's LOG_BOOST, which would make this inline function differ between the modules that include it)
    int i = 0;
    for (; i < n; i++)
        if (keys[i] == key) {
            if (whos[i] != who && strcmp(whos[i], who)) { if (g_log_level >= 2) logline("Actuator: %s is written by %s now (it was %s)", what, who, whos[i]); whos[i] = who; }
            break;
        }
    if (i == n && n < 16) { keys[n] = key; whos[n] = who; n++; if (g_log_level >= 2) logline("Actuator: %s is written by %s", what, who); }
    __atomic_clear(&busy, __ATOMIC_RELEASE);
}

// ---------------------------------------------------------------- the settings cvars (g_acc, g_cruise_control_smart, g_adaptive_shift): set like the options menu does
static inline void act_cvar_set(const char *who, void *cvar, float value)
{
    act_note_writer(cvar, "a settings cvar", who);
    game_cvar_set(cvar, value);
}

// ---------------------------------------------------------------- the engine brake lever
static inline int act_ebrake_hook(ebrake_press_fn cb, const char **why) { return game_ebrake_hook(cb, why); }   // wrap the two key calls: 1 = in
static inline int act_ebrake_setter(const char **why) { return game_ebrake_setter(why); }   // find the stage setter without wrapping the keys: 1 = put / set work
// The on byte and the stage, like the key handler does (from the callback, or an actuator hook: see game.cpp).
static inline void act_ebrake_put(const char *who, char *ga, int on, unsigned stage)
{
    act_note_writer((const void *)(uintptr_t)ACT_KEY_EBRAKE, "the engine brake lever", who);
    game_ebrake_put(ga, on, stage);
}
// put, plus the handler's flag byte (flag < 0 = leave it)
static inline void act_ebrake_set(const char *who, char *ga, int on, unsigned stage, int flag)
{
    act_note_writer((const void *)(uintptr_t)ACT_KEY_EBRAKE, "the engine brake lever", who);
    game_ebrake_set(ga, on, stage, flag);
}

// ---------------------------------------------------------------- the automatic gearbox (the hook gave the module `self`; sense_gb_* read it)
static inline void act_gb_force(void *self) { gb_force(self); }
static inline void act_gb_set_target(void *self, int g) { gb_set_target(self, g); }
static inline void act_gb_rest_state(void *self, int drive, int latch) { gb_rest_state(self, drive, latch); }   // drive state (4 forward gear wanted, 3 at rest in neutral) and the at-rest latch
static inline int act_gb_engage(void *self, int g) { return gb_engage(self, g); }   // gap state with gear 0: engage g like the box does on throttle (1.60)
static inline int act_gb_engage_init() { return game_engage_init(); }   // find the engage code (once, at start): 1 = found
static inline void act_gb_drive_state(void *self, int v) { *(int *)((char *)self + L->s_drive) = v; }   // 4 forward / 0 reverse / 3 at rest in neutral (the game's own values)
static inline void act_gb_latch(void *self, unsigned char v) { *(unsigned char *)((char *)self + L->s_latch) = v; }   // the at-rest latch
// The update's tail (the generic gearbox is next): pre(self) runs before the tail jump goes on. 1 = hooked. Several modules chain it.
static inline int act_hook_tail(void *pre) { return G.tail_site && game_patch_tail(G.tail_site, pre); }

// ---------------------------------------------------------------- game_actor: the cruise, the ACC target, the hazards, the parking brake, lane keeping (per layout)
static inline void act_cruise_on(char *ga, unsigned char v) { *(unsigned char *)(ga + L->a_cruise_on) = v; }
static inline void act_cruise_set(char *ga, float v) { *(float *)(ga + L->a_cruise_set) = v; }   // km/h
static inline void act_acc_target(char *ga, float v) { *(float *)(ga + L->a_acc_target) = v; }   // km/h: the game's controller and its overspeed braking read it
static inline void act_park_flag(char *ga, unsigned char v) { *(unsigned char *)(ga + 0x15c) = v; }
static inline void act_park_val(char *ga, float v) { *(float *)(ga + 0x160) = v; }
// Hazards on / off the way the game's hazard key does it (0x14099d633, neutral-exit-re notes § 3): bit 0x10 of the lights byte, the flasher, the blink phase
// when no indicator runs.
static inline void act_hazards(char *ga, int on)
{
    volatile unsigned char *b = (volatile unsigned char *)(ga + L->a_lights);
    if (((*b & 0x10) != 0) == (on != 0)) return;
    *b ^= 0x10;
    *(volatile float *)(ga + L->a_haz_flash) = on ? 1.0f : 0.0f;
    if (!(*b & 0x0c)) *(volatile int32_t *)(ga + L->a_blink_t) = 0;
}
// The retarder lever level, set with the game's own setter (cruise-keys-re notes: the activate fn refuses while the level is above 0). `setter` = the
// game's function as the module found it at start; the call is the one the module made itself.
static inline void act_retarder_level(void (*setter)(void *ga, unsigned level, unsigned char snap), void *ga, unsigned level, unsigned char snap) { setter(ga, level, snap); }
// Lane keeping has one owner for each of its two writes (WP-LA2): the control output u (from the hook, on the game's thread: who = NULL there, nothing is logged from a stub;
// act_lka_owner names the owner once, where the hook is installed) and the on / off bit (from the module's frame: who = the module, logged on the first write and on a change).
static inline void act_lka_owner(const char *who) { act_note_writer((const void *)(uintptr_t)ACT_KEY_LKA_U, "the lane-keeping request (the control law's output)", who); }
static inline void act_lka_u(const char *who, char *ga, float v)
{
    act_note_writer((const void *)(uintptr_t)ACT_KEY_LKA_U, "the lane-keeping request (the control law's output)", who);
    *(float *)(ga + L->a_lka_u) = v;   // the control output of the game's lane-keeping law (-1..1)
}
static inline void act_lka_bit_set(const char *who, char *ga)
{
    act_note_writer((const void *)(uintptr_t)ACT_KEY_LKA_BIT, "the lane-keeping switch", who);
    __atomic_fetch_or((unsigned char *)(ga + L->a_lka), (unsigned char)0x20, __ATOMIC_SEQ_CST);    // (the game thread writes this byte too)
}
static inline void act_lka_bit_clear(const char *who, char *ga)
{
    act_note_writer((const void *)(uintptr_t)ACT_KEY_LKA_BIT, "the lane-keeping switch", who);
    __atomic_fetch_and((unsigned char *)(ga + L->a_lka), (unsigned char)~0x20, __ATOMIC_SEQ_CST);
}
