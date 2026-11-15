// The cruise state (WP-SP1, docs/cruise-layers.md): off / CC / LIM, the set speed, the 3x toggle, ISA. PURE: no game call, header only.
//
// Our state is the truth. In CC the game's own cruise byte and set speed ARE the state (the game's keys, its cancel on brake and its HUD
// keep working: the model mirrors them). In LIM the model owns on and the limit; the game's cruise stays off unless show_set is 1 (the
// limit is then put in its set speed for the HUD). The caller feeds the keys it saw (the toggle "SET", +, -, resume, with their times)
// and what the game's cruise byte and set speed are after the game's own handler ran; cs_step() says what the state is now and what the
// caller must write into the game. The key table is in docs/cruise-layers.md.
#pragma once
#include <math.h>
#include <string.h>

enum { CSM_CC = 0, CSM_LIM = 1 };
enum { CSK_TOG = 0, CSK_INC, CSK_DEC, CSK_RES };   // the game's cruise keys, in the order of its action objects
// Cue events (the hook: speedlayer.h sl_cue maps each to a placeholder tone; the spoken cue plugs in there)
enum { SLE_NONE = 0, SLE_CC_ON, SLE_CC_OFF, SLE_LIM_ON, SLE_LIM_OFF, SLE_NO };

struct cs_cfg {
    int presses;            // 3: the toggle gesture (owner's proposal, card 14b)
    float window_s;         // 1.5: first to last press
    float min_on_kmh;       // the lowest speed the cruise can be switched on at (the game's patched minimum, [cruise] min_kmh)
    float min_set_kmh;      // the set speed never goes below this ([cruise] min_set_kmh; 0 = none)
    float max_set_kmh;      // the set speed never goes above this (the limiter's cap; 0 = none)
    int show_set;           // LIM: keep the game's cruise on, set = the limit, for the HUD (the live experiment)
    float isa_off_kmh;      // ISA: set = limit + this (in the game's units: mph when the game shows mph)
    float isa_pct;          // ATS only: set = limit + this share of the limit in whole units (0 = isa_off_kmh)
    int ats;
    float unit_kmh;         // 1, or 1.609344 when the game shows mph
};
static inline cs_cfg cs_cfg_default()
{
    cs_cfg c; memset(&c, 0, sizeof c);
    c.presses = 3; c.window_s = 1.5f; c.min_on_kmh = 5; c.min_set_kmh = 20; c.isa_off_kmh = 3; c.isa_pct = 5; c.unit_kmh = 1;
    return c;
}

struct cs_ev { int key; double t; };
struct cs_in {
    double now;
    int game_on;            // the game's cruise byte, after its own handler ran
    float game_set;         // its set speed (km/h)
    float speed_kmh;
    int brake;              // the brake pedal is pressed (LIM with show_set: the game cancels on it, the byte is put back after it)
    int isa;                // ISA is on
    float limit_ms;         // the road's limit (m/s); <= 1 = unknown
    double since_res_s;     // seconds since the last resume press
    int n_ev;               // the key presses since the last call, oldest first
    cs_ev ev[6];
};
struct cs_out {
    int on, mode;
    float set;              // km/h: the set speed (CC) or the limit (LIM)
    int want_game_on;       // -1 = leave the game's byte, else write it
    float want_game_set;    // NAN = leave
    int cue;                // SLE_* of this step
    const char *note;       // what happened, for the log ("" = nothing)
    int lim_on;             // LIM is on: the ceiling is `set`
};
struct cs_state {
    int mode, on;
    float set, mem_set[2];  // mem_set: the last set per mode (resume)
    double tog_t[8]; int n_tog;
    int pre_on; float pre_set;   // the state before the first press of the running gesture
    int isa_last;
};
static inline void cs_reset(cs_state *s) { memset(s, 0, sizeof *s); s->isa_last = -1; }

// ISA: the set speed that follows a limit (km/h), as modules/cruise.cpp isa_set_kmh. lim_u = the limit in whole units.
static inline float cs_isa_set(const cs_cfg &c, float limit_ms, int *lim_u)
{
    int lim = (int)roundf(limit_ms * 3.6f / c.unit_kmh);
    if (lim_u) *lim_u = lim;
    float off = (c.isa_pct > 0 && c.ats) ? roundf(lim * c.isa_pct / 100.0f) : c.isa_off_kmh;
    return fmaxf((lim + off) * c.unit_kmh, c.min_set_kmh);
}

static inline float cs_clamp_set(const cs_cfg &c, float v)
{
    if (c.max_set_kmh > 0 && v > c.max_set_kmh) v = c.max_set_kmh;
    if (v < c.min_set_kmh) v = c.min_set_kmh;
    return v;
}

static inline void cs_step(const cs_cfg &c, cs_state *s, const cs_in &in, cs_out *o)
{
    memset(o, 0, sizeof *o);
    o->want_game_on = -1; o->want_game_set = NAN; o->note = "";
    const int prev_on = s->on; const float prev_set = s->set;
    // 1. CC: the game's cruise is the state (its keys, its cancel on brake, AEB, a junction cancel of ours)
    if (s->mode == CSM_CC) {
        s->on = in.game_on ? 1 : 0;
        if (s->on && in.game_set > 0) s->set = in.game_set;
        if (s->on) s->mem_set[CSM_CC] = s->set;
    } else if (s->on && in.game_set > 0 && (!c.show_set || in.game_on)) {
        // LIM: the limit is the game's set field, which the key code steps. The game's handler overwrites it for a toggle press (the speed of the
        // activation) and for a resume (its own memory): those presses are ours to read, so this step keeps the limit it had (a resume with ISA is the
        // key code's: it writes limit + offset into the field and that is adopted)
        int own = 0;
        for (int i = 0; i < in.n_ev; i++)
            if (in.ev[i].key == CSK_TOG || (in.ev[i].key == CSK_RES && !(in.isa && in.limit_ms > 1))) own = 1;
        if (!own) {   // (the limit never passes the cap of the range: a + at the top fires the 'no' cue and the game's field is put back)
            s->set = cs_clamp_set(c, in.game_set); s->mem_set[CSM_LIM] = s->set;
            if (fabsf(s->set - in.game_set) > 1e-3f) o->want_game_set = s->set;
        }
    }
    // 2. the key presses, oldest first
    for (int i = 0; i < in.n_ev; i++) {
        const cs_ev &e = in.ev[i];
        if (e.key != CSK_TOG) { s->n_tog = 0; }
        if (e.key == CSK_TOG) {
            // the sequence: presses older than the window from the first one start a new one
            while (s->n_tog > 0 && e.t - s->tog_t[0] > c.window_s) { for (int j = 1; j < s->n_tog; j++) s->tog_t[j - 1] = s->tog_t[j]; s->n_tog--; }
            if (s->n_tog == 0) { s->pre_on = prev_on; s->pre_set = prev_set; }   // the state before the press (the game's handler already acted on it)
            if (s->n_tog < 8) s->tog_t[s->n_tog++] = e.t;
            if (s->n_tog >= c.presses) {   // the gesture: the other mode, ON; the set from before it if the cruise was on, else the speed of now
                s->mode = s->mode == CSM_CC ? CSM_LIM : CSM_CC;
                s->on = 1;
                s->set = cs_clamp_set(c, s->pre_on && s->pre_set > 0 ? s->pre_set : in.speed_kmh);
                s->mem_set[s->mode] = s->set;
                s->n_tog = 0;
                o->cue = s->mode == CSM_LIM ? SLE_LIM_ON : SLE_CC_ON;
                o->note = s->mode == CSM_LIM ? "3x SET: limiter (LIM) on" : "3x SET: cruise (CC) on";
                o->want_game_on = (s->mode == CSM_CC || c.show_set) ? 1 : 0;
                o->want_game_set = s->set;
                s->isa_last = -1;
                continue;
            }
        }
        if (s->mode != CSM_LIM) continue;      // CC: the game did what the key says
        // LIM: the game's handler switched its own byte on for a press while it was off; what the press means is ours
        if (e.key == CSK_TOG && s->on) {
            s->on = 0; o->cue = SLE_LIM_OFF; o->note = "SET: limiter off";
            o->want_game_on = 0; o->want_game_set = s->set;
        } else if (!s->on && (e.key == CSK_TOG || e.key == CSK_INC || e.key == CSK_DEC || e.key == CSK_RES)) {
            if (in.speed_kmh < c.min_on_kmh && e.key != CSK_RES) {
                o->cue = SLE_NO; o->note = "limiter: below the minimum switch-on speed, refused";
                o->want_game_on = 0;
            } else {
                float v = (e.key == CSK_RES && s->mem_set[CSM_LIM] > 0) ? s->mem_set[CSM_LIM] : in.speed_kmh;   // resume: the remembered limit
                s->on = 1; s->set = cs_clamp_set(c, v); s->mem_set[CSM_LIM] = s->set; s->isa_last = -1;
                o->cue = SLE_LIM_ON; o->note = e.key == CSK_RES ? "RES: limiter on at the remembered limit" : "limiter on at the speed";
                o->want_game_on = c.show_set ? 1 : 0; o->want_game_set = s->set;
            }
        } else if (s->on && e.key == CSK_RES && !(in.isa && in.limit_ms > 1)) {
            o->cue = SLE_NO; o->note = "limiter: RES while on has nothing to do without ISA";
        } else if (s->on && e.key == CSK_INC && c.max_set_kmh > 0 && s->set >= c.max_set_kmh - 0.01f) {
            o->cue = SLE_NO; o->note = "limiter: + at the top of the range";
        } else if (s->on && e.key == CSK_DEC && s->set <= c.min_set_kmh + 0.01f) {
            o->cue = SLE_NO; o->note = "limiter: - at the bottom of the range";
        }
    }
    // 3. ISA, in both modes: at every new sign (and when the cruise or the limiter comes on by resume) the set becomes limit + offset
    if (!s->on || !in.isa) s->isa_last = -1;
    else if (in.limit_ms > 1) {
        int lim; float v = cs_isa_set(c, in.limit_ms, &lim);
        if (s->isa_last < 0 && in.since_res_s > 1.0) { s->isa_last = lim; }   // switched on with set: the speed stays, the next sign takes over
        else if (lim != s->isa_last) {
            s->isa_last = lim;
            s->set = cs_clamp_set(c, v); s->mem_set[s->mode] = s->set;
            o->want_game_set = s->set;
            if (!o->note[0]) o->note = "ISA: the set follows the sign";
        }
    }
    // 4. LIM: where the game's byte must be
    if (s->mode == CSM_LIM) {
        if (!c.show_set) { if (in.game_on && o->want_game_on < 0) o->want_game_on = 0; }
        else if (s->on && !in.game_on && !in.brake && o->want_game_on < 0) { o->want_game_on = 1; o->want_game_set = s->set; }
        else if (!s->on && in.game_on && o->want_game_on < 0) o->want_game_on = 0;
    }
    o->on = s->on; o->mode = s->mode; o->set = s->set; o->lim_on = s->mode == CSM_LIM && s->on;
}
