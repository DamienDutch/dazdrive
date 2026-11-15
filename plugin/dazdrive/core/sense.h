// The sense layer's reads of the game (docs/architecture.md, "The sense layer"). A module reads the game's memory only through the functions of this file
// (and T, route_*, traffic_*, the model); the layer lint (tools/check_layers.py) fails a module that spells an offset, a cast pointer or a game_*
// reader itself. WP-S moved the modules' reads here without changing any of them.
//
// How they are written, and why:
//  - Header-inline, on the same game_* / gb_* accessors the sims already stand in for (sim_cruise's fake game_actor buffer, game_adaptive as a
//    variable), so no sim needs another source file and each sim still runs the exact expression the game would.
//  - ON DEMAND, at the place the module read before. Not a snapshot per frame: key handling writes the set speed and reads it back in the same
//    call, the gearboxes ask gb_gear / gb_state right after gb_force, and the sims change a cvar between two calls. A snapshot would move those reads.
//  - One accessor = one expression: the offset (the layout field, read at the time of the call), the type (volatile and width kept) and the order of
//    the reads are the ones the module had. A guard (`g &&`, `mem_readable`, `L &&`) stays where the module had it unless the accessor's name says it
//    carries it (sense_hill_timer, sense_lights_guarded, sense_lights_aligned), because the sites differ and the sims run with L = NULL in places.
#pragma once
#include "suite.h"
#include "game.h"
#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------- the driving mode (WP-S step S2: the one reader)
// The game's adaptive-shift cvar g_adaptive_shift: 0 = M, 10 Eco, 3 Normal, 1.67 Power; NAN = unknown.
static inline float sense_adaptive() { return game_adaptive(); }
// The mode it means (mode_of in suite.h: NAN = Normal), for every module that wants the driver's mode. Not ts_mode(): that is the gearbox brain's view
// (-1 for M, a manual box and an unsupported truck), where this one is the cvar itself, also while no gearbox is hooked, and has MODE_M.
static inline int sense_mode() { return mode_of(game_adaptive()); }

// ---------------------------------------------------------------- the game build
// The layout of this game is ATS's (L->game, the SDK game id); the 1.61 update (the tail jump epilogue, game_is_161 in game.h). NULL layout = not ATS.
static inline int sense_is_ats() { return L && L->game && !strcmp(L->game, "ats"); }
static inline int sense_is_161() { return game_is_161(); }
static inline int sense_is_162() { return game_is_162(); }   // 1.62 and later (the automatic rewrite, action objects +8)

// ---------------------------------------------------------------- the automatic gearbox (WP-S step S4: the gb_* readers of core/game.h, under the sense name)
// self = the gearbox object the hook gave the module (or sense_gearbox(), the player's). The writers are act_gb_* (core/act.h).
static inline void *sense_gearbox() { return game_gearbox(); }
static inline char *sense_gb_inner(void *self) { return gb_inner(self); }
static inline int sense_gb_gear(void *self) { return gb_gear(self); }
static inline int sense_gb_target(void *self) { return gb_target(self); }
static inline int sense_gb_state(void *self) { return gb_state(self); }
static inline float sense_gb_thr_eff(void *self) { return gb_thr_eff(self); }
// self + s_drive: the drive state (4 forward gear wanted, 3 at rest in neutral, 0 reverse), as the game's at-rest rule sets it.
static inline int sense_gb_drive_state(void *self) { return *(int *)((char *)self + L->s_drive); }
// The game's own idle (the engine's figure + the drive's idle add), the one its clutch and gearbox code compare the rpm with. `inner` = the vehicle object
// (sense_gb_inner). 0 where a pointer is not readable or the figure is not an idle. TrueShift's ts_idle_rpm() is the measured one.
static inline float sense_game_idle_rpm(char *inner)
{
    enum { I_DRIVE = 0x150, D_IDLE_ADD = 0x2dc };   // unchanged across layouts; the engine's idle is L->e_idle
    const unsigned D_ENGINE = game_is_162() ? 0x368 : 0x360;   // 1.62: the drive's engine pointer moved 8 up
    if (!inner || !mem_readable(inner + I_DRIVE, 8)) return 0;
    char *drive = *(char **)(inner + I_DRIVE);
    if (!drive || !mem_readable(drive + D_ENGINE, 8)) return 0;
    char *eng = *(char **)(drive + D_ENGINE);
    if (!eng || !mem_readable(eng + L->e_idle, 4) || !mem_readable(drive + D_IDLE_ADD, 4)) return 0;
    float i = *(float *)(eng + L->e_idle) + *(float *)(drive + D_IDLE_ADD);
    return i > 200 && i < 2000 ? i : 0;
}

// ---------------------------------------------------------------- the game's settings cvars (the writer is act_cvar_set)
static inline void *sense_cvar(const char *name) { return game_cvar(name); }   // a settings cvar object by name (e.g. "g_acc"); NULL if not found. A scan of the game's memory on a miss: look once, keep the pointer
static inline float sense_cvar_get(void *cvar) { return game_cvar_get(cvar); }   // its value as float (NAN if unknown)

// ---------------------------------------------------------------- the engine brake lever (the accessors of core/game.h; the writers are act_ebrake_*)
static inline int sense_ebrake_state(char *ga, int *held, int *on, unsigned *stage, int *flag) { return game_ebrake_state(ga, held, on, stage, flag); }   // 1 = read
static inline unsigned sense_ebrake_cap(char *ga) { return game_ebrake_cap(ga); }   // stage count of the player's engine, 0 = unknown
static inline float sense_ebrake_level(void *self) { return game_ebrake_level(self); }   // strength 0..1 (stage/stages, held = 1, auto); -1 unknown
static inline int sense_ebrake_downshift(void *self) { return game_ebrake_downshift(self); }   // the game wants a downshift into the engine-brake range now: 1/0, -1 unknown

// The engine brake's offsets in this layout, for the log line that says where the lever sits (L must be set: the module is only started with a layout).
struct sense_eb_layout { const char *name; unsigned held, on, stage, flag, up, dwn, cap; };
static inline sense_eb_layout sense_ebrake_layout()
{
    sense_eb_layout e = { L->name, L->a_eb_held, L->a_eb_on, L->a_eb_stage, L->a_eb_flag, L->a_eb_up, L->a_eb_dwn, L->e_eb_cap };
    return e;
}

// ---------------------------------------------------------------- the player's game_actor (the object most of the per-driver state sits in)
// The game_actor outside the hooks (driving only), else NULL: what a module hands to the readers below. Same call as game_actor_safe().
static inline char *sense_actor() { return game_actor_safe(); }
// n bytes at p can be read (a probe of the game's memory: VirtualQuery).
static inline int sense_readable(const void *p, size_t n) { return mem_readable(p, n); }
// A pointer that is 8-aligned and readable for n bytes (the guard Traffic used for every pointer it followed).
static inline int sense_ptr_ok(const void *p, size_t n) { return p && !((uintptr_t)p & 7) && mem_readable(p, n); }
// game_actor + a_hill: the game's own hill-hold timer (> 0 = the hold is on). Creep's form: 0 where ga is NULL or the field is not readable.
static inline float sense_hill_timer(char *ga) { return ga && mem_readable(ga + L->a_hill, 4) ? *(float *)(ga + L->a_hill) : 0; }
// LowSpeed's form: the hold is on (readable and > 0); the caller has tested ga.
static inline int sense_hill_on(char *ga) { return mem_readable(ga + L->a_hill, 4) && *(float *)(ga + L->a_hill) > 0; }
// The action object of the game's gearbox-mode key (game_actor + 0x1ea40 on 1.60, + 0x1eb90 on 1.61, + 0x1eb98 on 1.62): NULL where ga is NULL or the slot is not readable.
// An action object: +0xf8 its state (2 = a press event, 1 = held after the game took the event), +0x178 its value.
static inline char *sense_mode_key_action(char *ga)
{
    unsigned mode_key = game_is_162() ? 0x1eb98 : game_is_161() ? 0x1eb90 : 0x1ea40;   // 1.62: every action object +8
    return ga && mem_readable(ga + mode_key, 8) ? *(char **)(ga + mode_key) : NULL;
}
static inline int sense_action_readable(char *act) { return mem_readable(act + 0xf8, 4); }
static inline int sense_action_state(const void *act) { return *(const int *)((const char *)act + 0xf8); }
static inline unsigned sense_action_state_u(const void *act) { return *(const unsigned *)((const char *)act + 0xf8); }   // (the same word, as the cruise keys test it)
static inline float sense_action_value(char *act) { return *(float *)(act + 0x178); }
// The cruise keys' action objects: game_actor + 0x1e710 (toggle), 0x1e720 (inc), 0x1e730 (dec), 0x1e740 (resume), 0x1e750 (accmode) on 1.60; +0x100 higher
// on 1.61 (ETS2 and ATS). key 0..4 in that order.
static inline unsigned sense_key_off(int key) { return (game_is_162() ? 0x1e818 : game_is_161() ? 0x1e810 : 0x1e710) + key * 0x10; }   // 1.62: +8
// The action object whose pointer sits at ga + off: NULL where it is NULL or not readable up to its state word (+0xf8).
static inline void *sense_action_at(char *ga, unsigned off)
{
    void *a = *(void **)(ga + off);
    return a && mem_readable(a, 0xf8 + 4) ? a : NULL;
}
// The state word of the action object at ga + off: -1 = none there. slot < 0: not cached (the found key); else the readability of the object is only
// checked again when the pointer in the slot changes (ptr / ok = the caller's per-slot cache; 96 slots at frame rate would be a lot of VirtualQuery calls).
static inline int sense_action_state_at(char *ga, int off, int slot, void **ptr, unsigned char *ok)
{
    enum { KEY_STATE = 0xf8 };
    if (slot < 0 && !mem_readable(ga + off, 8)) return -1;
    void *p = *(void **)(ga + off);
    if ((uintptr_t)p < 0x10000 || ((uintptr_t)p & 7)) return -1;
    if (slot >= 0) {
        if (p != ptr[slot]) { ptr[slot] = p; ok[slot] = mem_readable(p, KEY_STATE + 4); }
        if (!ok[slot]) return -1;
    } else if (!mem_readable(p, KEY_STATE + 4)) return -1;
    return *(int *)((char *)p + KEY_STATE);
}

// ---------------------------------------------------------------- game_actor fields (cruise-re notes § 4), per layout. ga = sense_actor() or the hook's game_actor.
static inline unsigned char sense_cruise_on(const char *ga) { return *(const unsigned char *)(ga + L->a_cruise_on); }   // the game's cruise is on
static inline float sense_cruise_set(const char *ga) { return *(const float *)(ga + L->a_cruise_set); }   // its set speed (km/h)
static inline float sense_acc_target(const char *ga) { return *(const float *)(ga + L->a_acc_target); }   // the ACC target speed (km/h; the game's overspeed braking reads it)
static inline float sense_lead_m(const char *ga) { return *(const float *)(ga + L->a_lead_m); }   // the gap to the game's lead (m; ATS 1.61: not that, see cruise.cpp lead_gap_m)
static inline float sense_brake_req(const char *ga) { return *(const float *)(ga + L->a_req); }   // the brake request 0..1 the game holds
static inline unsigned sense_retarder_level(const char *ga) { return *(const unsigned *)(ga + L->a_retarder); }   // the retarder lever level
static inline unsigned char sense_lim_on(const char *ga) { return *(const unsigned char *)(ga + L->a_lim_on); }   // the speed limiter's on byte (ATS 1.61: stays set with the option off)
static inline float sense_lim(const char *ga) { return *(const float *)(ga + L->a_lim); }   // the limiter's value (km/h)
static inline unsigned char sense_lights(const char *ga) { return *(const unsigned char *)(ga + L->a_lights); }   // indicator / hazard flag byte: bit 4 left, 8 right, 0x10 hazard
// The same byte where the caller has not tested ga or the layout: 0 then (Cruise's overtake assist).
static inline unsigned char sense_lights_guarded(const char *ga) { return ga && L && L->a_lights ? *(const unsigned char *)(ga + L->a_lights) : 0; }
static inline unsigned char sense_lka_byte(const char *ga) { return *(const unsigned char *)(ga + L->a_lka); }   // lane keeping: bit 0x20 = active
// 1 / 0 = lane keeping on / off, -1 = not known (no game_actor or no offset in the layout): what the dev feed shows.
static inline int sense_lka_state(const char *ga) { return ga && L && L->a_lka ? (sense_lka_byte(ga) & 0x20) != 0 : -1; }
static inline float sense_lka_u(const char *ga) { return *(const float *)(ga + L->a_lka_u); }   // the game's lane-keeping control output u (-1..1)
// The parking brake flag and value (cruise-re.md, the cancel hub, inferred; fixed offsets, not in the layout).
static inline unsigned char sense_park_flag(const char *ga) { return *(const unsigned char *)(ga + 0x15c); }
static inline float sense_park_val(const char *ga) { return *(const float *)(ga + 0x160); }
// The player's steering state in the vehicle object (sense_gb_inner(sense_gearbox())): the LKA assist state A, the effective steering and the wheel. NAN where
// the gearbox / vehicle is not found or not readable.
static inline void sense_steering(float *A, float *steer, float *wheel)
{
    *A = *steer = *wheel = NAN;
    void *gb = game_gearbox();
    char *veh = gb ? gb_inner(gb) : NULL;
    if (veh && mem_readable(veh + L->i_assist, 4)) {
        *A = *(float *)(veh + L->i_assist);
        *steer = *(float *)(veh + L->i_steer);
        *wheel = *(float *)(veh + L->i_steer_in);
    }
}
// game_actor + a_lights: the indicator / hazard flag byte (bit 4 left, 8 right, 0x10 hazard) behind the guards the Traffic module had (ga set, a layout with
// the offset, the byte 8-aligned and readable); 0 where one fails.
static inline unsigned sense_lights_aligned(char *ga)
{
    unsigned fl = 0;
    if (ga && L && L->a_lights && sense_ptr_ok(ga + L->a_lights, 1)) fl = *(unsigned char *)(ga + L->a_lights);
    return fl;
}
