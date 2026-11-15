// The game side: per-build layout, code discovery by byte patterns, call-site
// hooks and typed access to the automatic gearbox, vehicle and engine objects.
#pragma once
#include "suite.h"
#include <stddef.h>
#include <string.h>

// One entry per game build family. The first layout whose patterns and
// self-checks all pass wins; if none does, nothing is patched.
struct layout {
    const char *name;
    const char *game;          // SDK game_id the layout belongs to ("eut2" = ETS2, "ats" = ATS): never tried on another game
    // patterns
    const char *pat_window;    // shift window fn (down/up rpm for a gear)
    const char *pat_chooser;   // inside the gear chooser of the automatic
    const char *pat_lobound;   // lowest-gear-allowed fn (its first call = grade fn)
    const char *pat_mass;      // in the automatic update: base_ctrl slot, game_actor offset, mass fn
    const char *chk_target;    // bytes right after one chooser call: store to the target gear
    const char *chk_force;     // in the update: test of the force/kick flag
    const char *chk_decide;    // bytes right before the window call in the update
    const char *chk_torque;    // in the chooser: read of the engine torque
    const char *pat_adaptive;  // in the update: read of the g_adaptive_shift cvar (optional)
    // gearbox object (self)
    unsigned s_inner, s_state, s_target, s_force;
    // vehicle object (inner)
    unsigned i_gear, i_thr_in, i_thr_eff, i_engine, i_trans;
    // engine object
    unsigned e_torque, e_curve, e_curve_n, e_rpm_limit, e_ranges;
    // transmission object
    unsigned t_shift_time;
    // game_actor
    unsigned a_adaptive_block;  // byte: game ignores g_adaptive_shift while set (cruise control?)
    // dashboard letter: string_dyn auto_mode_name in the transmission (manual_mode_name 0x20 later),
    // transmission = resolve(inner+i_trans) or [inner+i_tx_alt]+x_tx_alt (probe run 2 path)
    unsigned t_auto_name, i_tx_alt, x_tx_alt;
    // engine brake (engine-brake-re notes): inner flag (brake-range window on) and demand, engine def
    // engine_brake_downshift and stage count, game_actor brake fraction
    unsigned i_eb_flag, i_eb_demand, e_eb_downshift, e_eb_cap, a_eb_frac;
    unsigned a_gearbox;  // game_actor -> the automatic gearbox object (probe run 2)
    const char *pat_gps; // gps_manager offset in game_ctrl at +3 (ets2la_plugin)
    const char *pat_tail;// the update's tail jump into the generic gearbox (E9 at tail_jmp)
    const char *pat_stop;// mulss xmm7,[0.8]: the automatic's stop threshold (0.8 x idle)
    unsigned e_type;     // engine def: 64-bit type id; ELECTRIC_ENGINE_ID = an electric motor (engine-brake-re notes). 1.61: inferred (+8)
    // ---------------------------------------------------------------- feature offsets
    // Every feature is gated on "this layout has the offsets" (non-zero) instead of the layout name. A value of 0
    // means the feature that reads it stays off on this layout. Game side names: "self" = the gearbox, "inner" = the
    // vehicle (self+0x10), "drive" = inner+0x150, "engine" = resolve(inner+i_engine), "trans" = resolve(inner+i_trans).
    unsigned a_state, a_engine_run;    // game_actor: player-driven state (2 = driven), engine running byte
    unsigned a_req;                    // game_actor: brake request 0..1 (zeroes the merged throttle while > 0)
    unsigned a_retarder;               // game_actor: retarder level
    unsigned a_lim, a_lim_on;          // game_actor: speed limiter value (km/h) / on byte
    unsigned a_cruise_on, a_cruise_set;// game_actor: cruise enabled byte / set speed (km/h)
    unsigned a_acc_target, a_lead_m;   // game_actor: ACC target speed (km/h) / lead gap (m)
    unsigned a_lights, a_lka;          // game_actor: indicator/hazard flags byte (bit 4 left, 8 right, 0x10 hazard), lane-keeping active byte (bit 0x20)
    unsigned a_lka_u;                  // game_actor: LKA control output u (float, -1..1)
    unsigned a_haz_flash, a_blink_t;   // game_actor: hazard flasher float, blink phase
    unsigned a_hill;                   // game_actor: hill-hold timer float
    unsigned a_thr_pedal, a_brake_pedal; // game_actor: driver throttle / brake pedal (fed to the gearbox)
    unsigned i_clutch, i_retarder;     // vehicle: clutch (0 closed..1 open), retarder demand (float)
    unsigned i_steer_in, i_assist;     // vehicle: steering input, LKA assist state A (float)
    unsigned i_steer;                  // vehicle: effective steering (float; = the wheel with an FFB wheel, wheel + A without). 1.60 0x640, 1.61 0x6b8 (+0x78 like the handles; both 1.61 exes: addss xmm0,[rax+0x6b8] after movss xmm0,[rax+0x14b0])
    unsigned s_amode, s_latch, s_drive;// gearbox: automatic-mode flag, at-rest latch, drive state
    unsigned e_idle;                   // engine: idle rpm
    unsigned tail_jmp;                 // pat_tail: byte offset of the E9 within the pattern
    // ---------------------------------------------------------------- feature patterns (embedded offsets)
    const char *pat_actuator;          // the cruise-fn call in the throttle fn (actuate.cpp)
    const char *pat_hazard;            // the auto-hazard rule (actuate.cpp)
    const char *pat_gate;              // the merged-throttle store gate (actuate.cpp)
    const char *bp_pat, *bp_getter, *bp_req, *bp_hill;   // brake pedal hook (game.cpp): main, getter call, request read, hill-start max
    const char *bl_pat, *bl_on, *bl_getter;              // brake light hook: main, 'on' store, getter call
    const char *rf_pat, *rf_before, *rf_after;           // retarder feed hook: main, flags before, flags after
    // Engine brake lever (modules/ebrake.cpp, core/ebrakelogic.h). game_actor: held byte, on byte, stage (u32), the byte the key handler
    // writes on every press, and the pointers to the up and down key actions. The patterns start at the E8 of the call that consumes the
    // key event and end with the E8 of the stage setter call (the last 5 bytes). 0 / NULL = this layout is not verified: the module is off.
    unsigned a_eb_held, a_eb_on, a_eb_stage, a_eb_flag, a_eb_up, a_eb_dwn;
    const char *eb_pat_up, *eb_pat_dwn;
};
enum : unsigned long long { ELECTRIC_ENGINE_ID = 0x16829f66307ULL };

extern const layout *L;

// The layout's build as a number (160, 161, 162, ...), from its name ("1.62"); 0 without a layout.
// A layout without a version name (the simulators' "sim") counts as 1.60, or 1.61 when it has the 1.61 tail jump (tail_jmp 7).
static inline int game_ver()
{
    if (!L) return 0;
    const char *n = L->name;
    if (n[0] == '1' && n[1] == '.' && n[2] >= '0' && n[2] <= '9' && n[3] >= '0' && n[3] <= '9') return 100 + (n[2] - '0') * 10 + (n[3] - '0');
    return L->tail_jmp == 7 ? 161 : 160;
}
// 1.61 and later: the gated features that keep a 1.60-only pattern/register mapping alongside a 1.61 one (the brake-pedal/stop
// stubs, the lane min-speed, ...). Where 1.62 differs again from 1.61 the code asks game_ver() >= 162.
static inline int game_is_161() { return game_ver() >= 161; }
static inline int game_is_162() { return game_ver() >= 162; }

struct game_addrs {
    uintptr_t window_fn, resolve_fn, chooser_fn, start_fn, lobound_fn, grade_fn, mass_fn;
    uintptr_t base_ctrl_slot;
    unsigned game_actor_off;
    uintptr_t cvar_adaptive, cvar_read_fn;  // 0 if not found
    unsigned gps_off;                       // 0 if not found
    uintptr_t tail_site;                    // 0 if not found
    uintptr_t stop_site;                    // 0 if not found
    uintptr_t update_lo, update_hi;
    uintptr_t choose_sites[4];
    int n_choose;
    uintptr_t decide_site;
    uintptr_t start_sites[8];
    int n_start;
};
extern game_addrs G;

const char *game_discover(const char *game_id);  // NULL on success, else what failed (every layout of this game)
int game_exe_stamp(unsigned *timestamp, unsigned *image_size);  // PE header of the game exe (identifies the build); 0 if unreadable
int game_patch_call(uintptr_t site, void *stub);
int game_patch_comiss_xmm0(uintptr_t site, float value);
int game_patch_cf0_if(uintptr_t site, int len, const unsigned char *flag);  // compare skipped (CF=0) while *flag  // comiss xmm0,xmm7; jbe -> against our own constant
int game_patch_tail(uintptr_t site, void *pre);  // pre(self) runs before the tail jump goes on
int game_patch_feed_xmm1(uintptr_t site, int len, float *feed);       // after len bytes: xmm1 = max(xmm1, *feed)
int game_patch_skip_if(uintptr_t site, int len, const unsigned char *skip);  // the instruction runs only while !*skip
int game_patch_zero_xmm_if(uintptr_t site, int len, int reg, const unsigned char *flag);  // after the load: xmm<reg> = 0 while *flag
int game_patch_store_xmm8(uintptr_t site, float (*fn)(float));
int game_patch_stop(uintptr_t site, float (*fn)(float idle, float thr, float brake));
// Clutch floor for one vehicle at the drivetrain's clutch copy (0x1408c82cf, creep-re notes): the drivetrain
// sees max(clutch, floor) for the vehicle `inner` (0 = closed .. 1 = open). The plugin sets inner and floor.
struct clutch_floor { void *inner; float floor; };
int game_patch_clutch_floor(uintptr_t site, clutch_floor *cf);
// Retarder demand in the drivetrain (0x1408c87a8, ETS2 1.60.1.7): after `mov rdi,[rdi+0x4d0]` (the retarder definition) the function
// reads the retarder DEMAND, movss xmm0,[rax+0x1444] (rax = the vehicle's inner), and calls a curve lookup with it; the definition's
// torque (movss xmm8,[rdi+0xa8]) follows. site = that 8-byte read. The code runs for every vehicle with this drivetrain (AI trucks
// too, possibly): the patched read returns demand x rs->scale for the vehicle rs->inner only (bit-identical with scale == 1.0f),
// every other vehicle and inner NULL get the demand as it is. The dashboard, the animation and the pedal function read inner+0x1444
// elsewhere and are not touched. The plugin sets inner and scale.
struct retarder_scale { void *volatile inner; volatile float scale; };   // volatile: written in this order on one thread, read on another
static_assert(sizeof(void *) == 8, "the stub reads the scale at rs+8");
int game_patch_retarder(uintptr_t site, const retarder_scale *rs);
// The stub, as bytes (inline so that the sims can run exactly what the game would): the original read, then, if rax is rs->inner,
// xmm0 *= rs->scale unless the scale is exactly 1.0f (compared as an integer, so the demand keeps its bits: NaN payloads, denormals
// under DAZ). It changes xmm0 (the original does too) and nothing else: rdx is saved around its use, the flags (the compares) are
// saved with pushfq/popfq, rax is only read, no other register or xmm is touched, no call, nothing on the stack but the two slots
// (flags, rdx) it pushes under the return address.
static inline size_t retarder_stub_code(unsigned char *out, const retarder_scale *rs)
{
    unsigned char *p = out;
    *p++ = 0xF3; *p++ = 0x0F; *p++ = 0x10; *p++ = 0x80;    // movss xmm0,[rax+disp32] (the retarder demand, L->i_retarder)
    unsigned d = L->i_retarder;
    for (int i = 0; i < 4; i++) *p++ = (unsigned char)(d >> (8 * i));
    *p++ = 0x9C;                                          // pushfq
    *p++ = 0x52;                                          // push rdx
    *p++ = 0x48; *p++ = 0xBA;                             // mov rdx, imm64 (rs)
    uint64_t a = (uint64_t)(uintptr_t)rs;
    for (int i = 0; i < 8; i++) *p++ = (unsigned char)(a >> (8 * i));
    *p++ = 0x48; *p++ = 0x3B; *p++ = 0x02;                // cmp rax,[rdx] (the player's vehicle?)
    *p++ = 0x75; *p++ = 0x0E;                             // jne +14 (another vehicle: leave xmm0 alone)
    *p++ = 0x81; *p++ = 0x7A; *p++ = 0x08; *p++ = 0x00; *p++ = 0x00; *p++ = 0x80; *p++ = 0x3F;   // cmp dword [rdx+8], 0x3F800000 (1.0f)
    *p++ = 0x74; *p++ = 0x05;                             // je +5 (scale is 1: leave xmm0 alone)
    *p++ = 0xF3; *p++ = 0x0F; *p++ = 0x59; *p++ = 0x42; *p++ = 0x08;   // mulss xmm0,[rdx+8]
    *p++ = 0x5A;                                          // pop rdx
    *p++ = 0x9D;                                          // popfq
    *p++ = 0xC3;                                          // ret
    return (size_t)(p - out);
}

// ---------------------------------------------------------------- brake pedal, brake light and retarder feed hooks (ETS2 1.60 only)
// Each is installed at start only with its [core] switch (brake_pedal_hook, brake_light_hook, retarder_feed_hook; default 0) and
// only on the 1.60 layout of ETS2 with every self-check passing (game_brake_hooks_start, brakepedal-re notes; tools/check_suite.py
// checks the same offline). They are the mechanism only: until a module registers a provider / requests the light / feeds the
// retarder, the game brakes, lights and retards exactly as before. The stubs and the decision are inline here so that
// tools/sim/test_brakehook.cpp runs exactly what the game would.
//
// The service-brake pedal (pedal fn 0x140992d40, ETS2 1.60.1.7): the game reads the pedal once per tick (the getter 0x140990ee0,
// result kept in xmm14) and at 0x14099357a takes `maxss xmm7,xmm14` = max(slewed brake request +0x17c, pedal) for the wheel
// brakes (the next call, vtbl+0x178, stores it). Only that use is replaced: xmm7 = max(xmm7, game_brake_pedal_fn(raw, request,
// game_actor)). The cruise cancel, the auto retarder / engine brake, the hill start and the brake lights keep reading the raw
// pedal. The fn returns raw unless a provider is registered; with one it still returns raw when the call is not for the player's
// game_actor, raw is NaN or >= 0.98 (a full press is never touched), the getter did not return the brake field (the pedals-swapped
// case: it returns the throttle +0x1cc0 then), the truck is electric (its code sets the service brake again after this site), the
// provider's heartbeat is stale (more than BP_STALE_CALLS calls since its last game_brake_pedal_heartbeat()), or the provider returns
// NaN. Otherwise the provider's value clamped to 0..1: it may be above raw (it then also replaces the +0x17c route's braking).
typedef float (*brake_pedal_provider)(float raw, float req, void *ga);
enum { BP_FB_OTHER, BP_FB_NAN, BP_FB_FULL, BP_FB_SWAPPED, BP_FB_ELECTRIC, BP_FB_STALE, BP_FB_N };   // fallback reasons (counters)
enum { BP_STALE_CALLS = 30, BP_BRAKE_FIELD = 0x1cc4 };   // game_actor+0x1cc4: the pedal the getter returns unless swapped
struct brake_pedal_counts {
    unsigned long long calls;          // every call of the hooked site
    unsigned long long used;           // the provider's value went to the brakes
    unsigned long long fb[BP_FB_N];    // the raw pedal went to the brakes although a provider was registered, by reason
};
struct brake_pedal_state {
    brake_pedal_provider volatile provider;   // NULL = raw
    volatile unsigned beat;                   // bumped by the provider's side every frame (game_brake_pedal_heartbeat)
    unsigned seen, since;                     // the hook's own: the beat it saw last and its player calls since then
    brake_pedal_counts n;
};
static inline void brake_pedal_set(brake_pedal_state *s, brake_pedal_provider fn)
{
    s->since = 0; s->seen = s->beat - 1u;   // a new provider starts fresh, as right after a beat (no stale verdict from an old count)
    s->provider = fn;
}
static inline void brake_pedal_beat(brake_pedal_state *s) { s->beat = s->beat + 1; }
// The decision: player = the call is for the player's game_actor, same = raw is bit for bit the brake field (not swapped).
static inline float brake_pedal_decide(brake_pedal_state *s, float raw, float req, void *ga, int player, int same, int electric)
{
    s->n.calls++;
    brake_pedal_provider p = s->provider;
    if (!p) return raw;
    int why = -1;
    if (!player) why = BP_FB_OTHER;
    else {
        unsigned b = s->beat;                 // the heartbeat counts the player's calls only
        if (b != s->seen) { s->seen = b; s->since = 0; }
        else if (s->since <= BP_STALE_CALLS) s->since++;
        if (raw != raw) why = BP_FB_NAN;
        else if (raw >= 0.98f) why = BP_FB_FULL;
        else if (!same) why = BP_FB_SWAPPED;
        else if (electric) why = BP_FB_ELECTRIC;
        else if (s->since > BP_STALE_CALLS) why = BP_FB_STALE;
    }
    if (why < 0) {
        float r = p(raw, req, ga);
        if (r == r) { s->n.used++; return r < 0.0f ? 0.0f : (r > 1.0f ? 1.0f : r); }
        why = BP_FB_NAN;
    }
    s->n.fb[why]++;
    return raw;
}
// What game_brake_pedal_fn does around the decision (ga = rsi at the site, player = the player's game_actor).
static inline float brake_pedal_hook_body(brake_pedal_state *s, float raw, float req, void *ga, const void *player, int electric)
{
    if (!s->provider) { s->n.calls++; return raw; }   // no provider: no memory read at all
    int is_player = ga && ga == player, same = 0;
    if (is_player) {
        uint32_t a, b;
        memcpy(&a, &raw, 4); memcpy(&b, (const char *)ga + L->a_brake_pedal, 4);
        same = a == b;
    }
    return brake_pedal_decide(s, raw, req, ga, is_player, same, electric);
}
// The stub the 5-byte `maxss xmm7,xmm14` (F3 41 0F 5F FE) becomes a call to. At the site rsi = game_actor, rbx = inner, rax = the
// vehicle's vtable (the call after it uses rax), xmm7 = the slewed request, xmm14 = the raw pedal; rcx rdx r8-r11 xmm0-5 and the
// flags are dead (the next instructions set rcx and xmm1 and call). The game's frame keeps rsp 16-aligned at the site, so after our
// call, push rax and the 32-byte shadow space the fn is entered with rsp = 8 mod 16 as the Win64 ABI wants. fn is
// float fn(float raw [xmm0], float req [xmm1], void *ga [r8]) (Win64), which keeps rbx rsi rdi rbp r12-r15 and xmm6-15.
static inline size_t brake_pedal_stub_code(unsigned char *out, const void *fn)
{
    unsigned char *p = out;
    *p++ = 0x50;                                          // push rax
    *p++ = 0x48; *p++ = 0x83; *p++ = 0xEC; *p++ = 0x20;   // sub rsp,0x20
    *p++ = 0x41; *p++ = 0x0F; *p++ = 0x28; *p++ = 0xC6;   // movaps xmm0,xmm14 (raw)
    *p++ = 0x0F; *p++ = 0x28; *p++ = 0xCF;                // movaps xmm1,xmm7 (request)
    *p++ = 0x4C; *p++ = 0x8B; *p++ = 0xC6;                // mov r8,rsi (game_actor)
    *p++ = 0x49; *p++ = 0xBB;                             // mov r11, imm64 (fn)
    uint64_t a = (uint64_t)(uintptr_t)fn;
    for (int i = 0; i < 8; i++) *p++ = (unsigned char)(a >> (8 * i));
    *p++ = 0x41; *p++ = 0xFF; *p++ = 0xD3;                // call r11
    *p++ = 0x48; *p++ = 0x83; *p++ = 0xC4; *p++ = 0x20;   // add rsp,0x20
    *p++ = 0x58;                                          // pop rax
    *p++ = 0xF3; *p++ = 0x0F; *p++ = 0x5F; *p++ = 0xF8;   // maxss xmm7,xmm0 (the original's max, with our pedal)
    *p++ = 0xC3;                                          // ret
    return (size_t)(p - out);
}
// The 1.61 service-brake max (0x1409edb89 `maxss xmm1,xmm6`) moved registers: rcx = the vehicle, rsi = game_actor,
// xmm14 = the raw pedal, xmm1 = the request (game_actor+0x1e4), xmm6 = the slewed brake. The 7 bytes `mov rax,[rcx];
// maxss xmm1,xmm6` (48 8B 01 F3 0F 5F CE) become a call to this stub: the original vtable load, then xmm1 =
// max(request, fn(raw, request, game_actor)). Only rax and xmm0-5 are clobbered besides xmm1; rcx, rsi and xmm6-15 are
// left as the game had them (rax is reloaded from rcx like the original did). fn = float fn(float raw, float req, void *ga).
static inline size_t brake_pedal_stub_code_161(unsigned char *out, const void *fn)
{
    unsigned char *p = out;
    *p++ = 0x48; *p++ = 0x8B; *p++ = 0x01;                // mov rax,[rcx] (the vehicle vtable, the original)
    *p++ = 0x50;                                          // push rax
    *p++ = 0x48; *p++ = 0x83; *p++ = 0xEC; *p++ = 0x30;   // sub rsp,0x30
    *p++ = 0xF3; *p++ = 0x0F; *p++ = 0x11; *p++ = 0x4C; *p++ = 0x24; *p++ = 0x28;   // movss [rsp+0x28],xmm1 (the request)
    *p++ = 0x41; *p++ = 0x0F; *p++ = 0x28; *p++ = 0xC6;   // movaps xmm0,xmm14 (raw)
    *p++ = 0x4C; *p++ = 0x8B; *p++ = 0xC6;                // mov r8,rsi (game_actor)
    *p++ = 0x49; *p++ = 0xBB;                             // mov r11, imm64 (fn)
    uint64_t a = (uint64_t)(uintptr_t)fn;
    for (int i = 0; i < 8; i++) *p++ = (unsigned char)(a >> (8 * i));
    *p++ = 0x41; *p++ = 0xFF; *p++ = 0xD3;                // call r11
    *p++ = 0xF3; *p++ = 0x0F; *p++ = 0x10; *p++ = 0x4C; *p++ = 0x24; *p++ = 0x28;   // movss xmm1,[rsp+0x28] (the request)
    *p++ = 0xF3; *p++ = 0x0F; *p++ = 0x5F; *p++ = 0xC8;   // maxss xmm1,xmm0 (the request, with our pedal)
    *p++ = 0x48; *p++ = 0x83; *p++ = 0xC4; *p++ = 0x30;   // add rsp,0x30
    *p++ = 0x58;                                          // pop rax (the vtable)
    *p++ = 0xC3;                                          // ret
    return (size_t)(p - out);
}

// The brake light (light fn 0x1409943e0, rbx = game_actor, rax = inner): the game decides bit 0x20 of inner+0x2a0 from the pedal,
// +0x160, the retarder demand and (g_brake_light_all) the parking / engine brake; our brake request (+0x17c) never lights it. Its
// 'off' store `and dword [rax+0x2a0],0xffffffdf` (7 bytes at 0x1409946ee) becomes a call to a data-only stub: the original and,
// then, only for the player's game_actor (blk.ga), while blk.on and blk.ttl has not run out (each call takes one tick off it) and
// the engine runs (+0x154, the game's own condition for any brake light), the bit goes back on. It runs only on the game's off
// path, so it only ever adds. A ttl that runs out clears on: the game's value stays. rdx is saved; flags are dead (a cmp follows).
struct brake_light_block { void *volatile ga; volatile int32_t ttl; volatile unsigned char on; };
static_assert(offsetof(brake_light_block, ttl) == 8 && offsetof(brake_light_block, on) == 12, "the stub reads ttl at +8, on at +12");
enum { BL_TTL = 6 };   // ticks a request keeps the light on (renewed every tick while it lasts)
static inline size_t brake_light_stub_code(unsigned char *out, const brake_light_block *blk)
{
    static const unsigned char orig[] = { 0x83, 0xA0, 0xA0, 0x02, 0x00, 0x00, 0xDF };   // and dword [rax+0x2a0],0xffffffdf
    unsigned char *p = out;
    for (unsigned i = 0; i < sizeof orig; i++) *p++ = orig[i];
    *p++ = 0x52;                                          // push rdx
    *p++ = 0x48; *p++ = 0xBA;                             // mov rdx, imm64 (blk)
    uint64_t a = (uint64_t)(uintptr_t)blk;
    for (int i = 0; i < 8; i++) *p++ = (unsigned char)(a >> (8 * i));
    *p++ = 0x48; *p++ = 0x3B; *p++ = 0x1A;                // cmp rbx,[rdx] (the player's game_actor?)
    *p++ = 0x75; *p++ = 0x1B;                             // jne done (+27)
    *p++ = 0x80; *p++ = 0x7A; *p++ = 0x0C; *p++ = 0x00;   // cmp byte [rdx+12],0 (on?)
    *p++ = 0x74; *p++ = 0x15;                             // je done (+21)
    *p++ = 0xFF; *p++ = 0x4A; *p++ = 0x08;                // dec dword [rdx+8] (ttl)
    *p++ = 0x78; *p++ = 0x12;                             // js stale (+18)
    *p++ = 0x80; *p++ = 0xBB;                             // cmp byte [rbx+disp32],0 (engine on?)
    unsigned e = L->a_engine_run;
    for (int i = 0; i < 4; i++) *p++ = (unsigned char)(e >> (8 * i));
    *p++ = 0x00;                                          // the imm8 (compare against 0)
    *p++ = 0x74; *p++ = 0x07;                             // je done (+7)
    *p++ = 0x83; *p++ = 0x88; *p++ = 0xA0; *p++ = 0x02; *p++ = 0x00; *p++ = 0x00; *p++ = 0x20;   // or dword [rax+0x2a0],0x20
    *p++ = 0x5A;                                          // done: pop rdx
    *p++ = 0xC3;                                          // ret
    *p++ = 0xC6; *p++ = 0x42; *p++ = 0x0C; *p++ = 0x00;   // stale: mov byte [rdx+12],0 (the request is over)
    *p++ = 0x5A;                                          // pop rdx
    *p++ = 0xC3;                                          // ret
    return (size_t)(p - out);
}

// The retarder demand feed (brakepedal-re notes "Retarder blending"): the pedal fn stores the retarder demand inner+0x1444 =
// max(lever, cruise overspeed demand, auto-retarder demand) at 0x140993beb: `mov eax,[rax]` (rax = the chosen float) then
// `mov [rcx+0x1444],eax` (rcx = inner), 8 bytes. They become a call to a data-only stub: the original load, then, only for the
// player's game_actor (rsi = blk.ga), while blk.on and blk.ttl has not run out (one tick off per call) and the fed bits are a
// float in 0..1 (bits <= 0x3F800000: no NaN, no negative), eax = max(eax, fed) compared as signed integers (for floats >= 0 the
// order of their bits; a negative game value loses to ours, a NaN of the game's wins and is stored as the game had it); then the
// original store. A ttl that runs out clears on: the game's value. The comiss at 0x140993be7 sets flags that the cmovbe at
// 0x140993bfc uses, so the stub keeps them (pushfq/popfq); rax is dead after the store (lea rax follows), rdx is saved.
// What the fed demand does: the drivetrain applies it as retarder torque, the brake lights come on (inner+0x1444 > 0), the HUD
// shows it; it does not cancel cruise (the cancel reads the lever). Interactions: the drivetrain factor hook (game_patch_retarder,
// [core] retarder_hook, 0x1408c87a8) reads this same inner+0x1444 and scales it, so a fed demand is multiplied by the owners'
// retarder factor too (a feeder that wants its value as is posts factor 1 or divides by it); with the game's g_retarder_auto on,
// its own pedal demand also enters the max and wins wherever it is higher.
struct retarder_feed_block { void *volatile ga; volatile int32_t ttl; volatile unsigned char on; volatile uint32_t bits; };
static_assert(offsetof(retarder_feed_block, ttl) == 8 && offsetof(retarder_feed_block, on) == 12 && offsetof(retarder_feed_block, bits) == 16,
              "the stub reads ttl at +8, on at +12, the fed float's bits at +16");
static inline size_t retarder_feed_stub_code(unsigned char *out, const retarder_feed_block *blk)
{
    unsigned char *p = out;
    *p++ = 0x8B; *p++ = 0x00;                             // mov eax,[rax] (original: the game's demand)
    *p++ = 0x9C;                                          // pushfq
    *p++ = 0x52;                                          // push rdx
    *p++ = 0x48; *p++ = 0xBA;                             // mov rdx, imm64 (blk)
    uint64_t a = (uint64_t)(uintptr_t)blk;
    for (int i = 0; i < 8; i++) *p++ = (unsigned char)(a >> (8 * i));
    *p++ = 0x48; *p++ = 0x3B; *p++ = 0x32;                // cmp rsi,[rdx] (the player's game_actor?)
    *p++ = 0x75; *p++ = 0x1C;                             // jne store (+28)
    *p++ = 0x80; *p++ = 0x7A; *p++ = 0x0C; *p++ = 0x00;   // cmp byte [rdx+12],0 (on?)
    *p++ = 0x74; *p++ = 0x16;                             // je store (+22)
    *p++ = 0xFF; *p++ = 0x4A; *p++ = 0x08;                // dec dword [rdx+8] (ttl)
    *p++ = 0x78; *p++ = 0x1A;                             // js stale (+26)
    *p++ = 0x81; *p++ = 0x7A; *p++ = 0x10; *p++ = 0x00; *p++ = 0x00; *p++ = 0x80; *p++ = 0x3F;   // cmp dword [rdx+16],0x3F800000
    *p++ = 0x77; *p++ = 0x08;                             // ja store (+8): not a float in 0..1
    *p++ = 0x3B; *p++ = 0x42; *p++ = 0x10;                // cmp eax,[rdx+16]
    *p++ = 0x7D; *p++ = 0x03;                             // jge store (+3): the game's is at least ours
    *p++ = 0x8B; *p++ = 0x42; *p++ = 0x10;                // mov eax,[rdx+16] (ours)
    *p++ = 0x89; *p++ = 0x81;                             // store: mov [rcx+disp32],eax (the retarder demand, L->i_retarder)
    unsigned r = L->i_retarder;
    for (int i = 0; i < 4; i++) *p++ = (unsigned char)(r >> (8 * i));
    *p++ = 0x5A;                                          // pop rdx
    *p++ = 0x9D;                                          // popfq
    *p++ = 0xC3;                                          // ret
    *p++ = 0xC6; *p++ = 0x42; *p++ = 0x0C; *p++ = 0x00;   // stale: mov byte [rdx+12],0 (the feed is over)
    *p++ = 0xEB; *p++ = 0xF1;                             // jmp store (-15)
    return (size_t)(p - out);
}

// The writers behind game_brake_light_request / game_retarder_feed (ga = the player's game_actor, NULL = not driving). Their order
// (ttl, ga, value, then on) lets a stub on the game thread see either the old request or the whole new one. 1 = taken.
static inline int brake_light_set(brake_light_block *b, void *ga, int on)
{
    if (!on || !ga) { b->on = 0; return !on; }
    b->ttl = BL_TTL; b->ga = ga; b->on = 1;
    return 1;
}
static inline int retarder_feed_set(retarder_feed_block *b, void *ga, float demand)
{
    uint32_t bits;
    memcpy(&bits, &demand, 4);
    if (!(demand > 0.0f) || bits > 0x3F800000u) { b->on = 0; return demand <= 0.0f; }   // <= 0: withdrawn; NaN or > 1: refused
    if (!ga) { b->on = 0; return 0; }
    b->ttl = BL_TTL; b->ga = ga; b->bits = bits; b->on = 1;
    return 1;
}

float game_brake_pedal_fn(float raw, float req, void *ga);           // what the pedal stub calls (game thread)
void game_brake_pedal_set_provider(float (*fn)(float raw, float req, void *ga));   // NULL = the raw pedal again
void game_brake_pedal_heartbeat();                                   // the provider's side, once per frame: "still here"
int game_brake_pedal_ok();                                           // installed (1.60, every self-check passed)
const brake_pedal_counts *game_brake_pedal_counts();                 // for the log
int game_brake_light_request(int on);                                // on: light for BL_TTL more ticks (renew every tick); 0 = withdraw. 1 = taken
int game_brake_light_ok();                                           // installed (1.60, every self-check passed)
int game_retarder_feed(float demand);                                // our retarder demand 0..1 for BL_TTL more ticks (renew every tick); <= 0 = withdraw. 1 = taken
int game_retarder_feed_ok();                                         // installed (1.60, every self-check passed)
void game_brake_hooks_start(int pedal, int light, int feed);         // m_core start: install what [core] asks for, one log line each

typedef float *(*window_fn)(void *self, float *out, unsigned gear);
typedef int (*gear_fn)(void *self);

// Engine data the game uses for its own automatic.
struct engine_info {
    float torque;                 // N·m
    int n_curve;
    float curve[24][2];           // (rpm, factor)
    float rpm_limit;
    float range[4][2];            // engine brake, low gear, high gear, power: (down, up) rpm
    float shift_time;             // s
    int electric;                 // 1 = electric motor (no gearbox to manage, regen), 0 = not (or unknown)
    int missing;                  // EM_* bits: what the engine's definition did not give (a mod engine with only `torque:`), set by engine_raw_check
    float tc_stall;               // WP-TC2: the transmission's stall_torque_ratio as the game holds it, trusted only where the diff beside it equals the truck config's (engine_read_stall, core/vehicle.cpp).
                                  // 0 = not read or not trusted (also what a zeroed engine_info says), 1 = read and trusted: no converter (a clutch box), > 1 = the converter's stall torque ratio
};
enum { R_BRAKE, R_LOW, R_HIGH, R_POWER };
// What a read of the engine's definition did not hold (vehicle.cpp engine_raw_check / engine_fill_defaults): the game's own defaults stand in for it (WP-TC1)
enum { EM_CURVE = 1, EM_LIMIT = 2, EM_RANGE0 = 4 };   // EM_RANGE0 << r = the range pair r (R_BRAKE .. R_POWER) is missing or insane

char *gb_inner(void *self);
int gb_gear(void *self);
int gb_target(void *self);
int gb_state(void *self);
void gb_force(void *self);
void gb_set_target(void *self, int g);
void gb_rest_state(void *self, int drive, int latch);  // drive state (4 forward gear wanted, 3 at rest in neutral) and the at-rest latch, as the game's at-rest rule sets them (1.60)
int gb_engage(void *self, int g);  // gap state with gear 0: engage g like the box does on throttle (1.60)
int game_engage_init();
float gb_thr_in(void *self);
float gb_thr_eff(void *self);
int gb_engine(void *self, engine_info *e);
#ifdef DAZDRIVE_DEV
void game_tune_restore();      // chiptune test: put the original engine torque back
void game_tune_set(float k);   // chiptune test: the engine torque = the original x k
int game_tune_engines(char **eng, float *orig, int max);   // chiptune test: the engine objects written
#endif  // 1 if the engine data looks sane
float game_mass();                          // truck + trailers, kg; NAN if unknown
float game_mass_own();                      // the towing vehicle's own mass (no trailers), kg; NAN if unknown. The class detection (vehicle.cpp) uses it for the 3.5 t rule
int game_vehicle_is_car();                  // 1 = the player vehicle is a car (def/vehicle/car, the ATS Road Trip DLC), 0 = not, -1 = unknown. Filled by the ATS game layer; the mass rule covers every stock car
float game_car_speed_limit_ms();            // the CAR speed limit the game shows (m/s) where it keeps truck and car limits apart (ATS); NAN = unknown (ETS2 reports one limit, trucks use truck.navigation.speed.limit)
float game_grade_deg(void *self);           // positive uphill; NAN if unknown
float game_adaptive();                      // g_adaptive_shift: 0 off, 10 eco, 3 normal, 1.67 power; NAN if unknown
int game_adaptive_blocked();                // game_actor flag that disables adaptive mode; -1 if unknown
void *game_gearbox();                       // the player truck's gearbox object via game_actor; NULL if not sane
int game_letter(void *self, char c);        // dashboard: transmission auto_mode_name := c (0 = the letter the game had before our first write); 1 done, 0 not found
// Engine brake lever (modules/ebrake.cpp). The callback gets every press of the player's engine brake up (+1) / down (-1) key, inside the
// game's key handler and before it acts on the press (game thread). game_ebrake_hook: 1 = the two key calls are wrapped, else *why says why not.
typedef void (*ebrake_press_fn)(char *game_actor, int dir);
int game_ebrake_hook(ebrake_press_fn cb, const char **why);
unsigned game_ebrake_cap(char *game_actor);   // stage count of the player's engine, 0 = unknown
int game_ebrake_state(char *game_actor, int *held, int *on, unsigned *stage, int *flag);   // 1 = read
void game_ebrake_put(char *game_actor, int on, unsigned stage);   // set the on byte and the stage like the key handler (from the callback, or an actuator hook: see game.cpp)
int game_ebrake_setter(const char **why);   // find the stage setter without wrapping the keys: 1 = game_ebrake_put / _set work, else *why says why not
void game_ebrake_set(char *game_actor, int on, unsigned stage, int flag);   // put, plus the handler's flag byte (flag < 0 = leave it)
int game_ebrake_downshift(void *self);     // the game wants a downshift into the engine-brake range now: 1/0, -1 unknown
float game_ebrake_level(void *self);        // engine brake strength 0..1 (stage/stages, held = 1, auto); -1 unknown
float curve_factor(const engine_info *e, float rpm);
int mem_readable(const void *p, size_t n);
char *game_actor_safe();                   // game_actor outside the hooks (driving only), else NULL
char *game_ctrl();                          // base_ctrl when it is a game_ctrl (driving), else NULL
float *game_patch_rip_float(uintptr_t instr, int disp_off, int len, float value);  // own constant for one instruction
uintptr_t game_scan(const char *pattern);  // the one hit of a pattern in .text, else 0
int game_scan_hits(const char *pattern, uintptr_t *hits, int max);  // every hit in .text (count; -1 = more than max)
void *game_cvar(const char *name);          // a settings cvar object by name (e.g. "g_acc"); NULL if not found
float game_cvar_get(void *cvar);            // its value as float (NAN if unknown)
void game_cvar_set(void *cvar, float value);// set it like the game's options menu does
