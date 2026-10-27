// The game side: per-build layout, code discovery by byte patterns, call-site
// hooks and typed access to the automatic gearbox, vehicle and engine objects.
#pragma once
#include "suite.h"

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
    const char *pat_tail;// the update's tail jump into the generic gearbox (E9 at +8)
    const char *pat_stop;// mulss xmm7,[0.8]: the automatic's stop threshold (0.8 x idle)
    unsigned e_type;     // engine def: 64-bit type id; ELECTRIC_ENGINE_ID = an electric motor (engine-brake-re notes). 1.61: inferred (+8)
};
enum : unsigned long long { ELECTRIC_ENGINE_ID = 0x16829f66307ULL };

extern const layout *L;

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
    static const unsigned char rd[] = { 0xF3, 0x0F, 0x10, 0x80, 0x44, 0x14, 0x00, 0x00 };   // movss xmm0,[rax+0x1444]
    unsigned char *p = out;
    for (unsigned i = 0; i < sizeof rd; i++) *p++ = rd[i];
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
};
enum { R_BRAKE, R_LOW, R_HIGH, R_POWER };

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
int gb_engine(void *self, engine_info *e);  // 1 if the engine data looks sane
float game_mass();                          // truck + trailers, kg; NAN if unknown
float game_grade_deg(void *self);           // positive uphill; NAN if unknown
float game_adaptive();                      // g_adaptive_shift: 0 off, 10 eco, 3 normal, 1.67 power; NAN if unknown
int game_adaptive_blocked();                // game_actor flag that disables adaptive mode; -1 if unknown
void *game_gearbox();                       // the player truck's gearbox object via game_actor; NULL if not sane
int game_letter(void *self, char c);        // dashboard: transmission auto_mode_name := c (0 = the letter the game had before our first write); 1 done, 0 not found
int game_ebrake_downshift(void *self);      // the game wants a downshift into the engine-brake range now: 1/0, -1 unknown
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
