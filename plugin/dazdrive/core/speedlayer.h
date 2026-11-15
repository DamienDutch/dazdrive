// The speed layer's glue (WP-SP1, docs/cruise-layers.md): the pure cores (speedarb.h, cruisestate.h) and the game. Header only, included by
// modules/cruise.cpp after its own includes: the sims that include the module get it without another file in their build. Everything here runs only
// when [cruise] engine = layers; with `classic` nothing of it is called and the module behaves as it did.
//
// Reads: the sense layer (sense_cruise_on / sense_cruise_set), T and the model (V, vehicle_resist, vehicle_natural_decel, wish_full). Writes: through core/act.h
// (act_cruise_on / act_cruise_set), the brake channel (brake_ask, by the caller), the display (disp_post, by the caller) and the cue (play_cue).
#pragma once
#include "speedarb.h"
#include "cruisestate.h"
#include "suite.h"
#include "act.h"
#include "sense.h"
#include "vehicle.h"
#include "wish.h"

#define SL_LIM_LETTER 0     // no dash letter for LIM (owner decision 66, 04-11: 'L' is lane keeping's; LIM answers by its sound and the set speed shown). 0 = no wish: the letter stays whose it is

// The knobs the module's param table holds, handed over each tick (cruise.cpp sl_configure()).
struct sl_cfg {
    sa_cfg arb;
    float presses, window_s;       // the toggle gesture
    float show_set;                // LIM: keep the game's cruise on for the HUD (the live experiment)
    float lim_tau_s;               // the ceiling closes the gap to the limit in this long (the own limiter's g_lim_tau)
    float kick_from, kick_full;    // the pedal zone in which a kickdown passes the ceiling
};
// What the module knows of its own params and the road, per tick.
struct sl_env {
    float min_on_kmh, min_set_kmh, max_set_kmh, isa_off_kmh, isa_pct, unit_kmh;
    int ats, isa;
    float limit_ms;
    double since_res_s;
};

static sl_cfg g_slc = { sa_cfg_default(), 3, 1.5f, 0, 0.75f, 0.92f, 1.0f };
static cs_state g_cs;                 // the cruise state (mode, on, set)
static sa_state g_sa;                 // the arbiter's requests of this tick and its force state
static sa_out g_sl_out;               // the last decision (the sims read it)
static cs_ev g_sl_ev[6]; static int g_sl_nev;
static int g_sl_cue_n[8];             // sims: how often each SLE_* fired
static char g_sl_line[220]; static double g_sl_line_t = -1e9;
static float g_lim_last = NAN; static double g_lim_last_at = -1;   // the ceiling held through a shift (the clutch open: the gear's force is unknown)

static void sl_reset_all()
{
    cs_reset(&g_cs); sa_reset(&g_sa, 0); memset(&g_sl_out, 0, sizeof g_sl_out); g_sl_nev = 0; g_sl_line[0] = 0; g_lim_last = NAN; g_lim_last_at = -1;
}
// A press of one of the cruise keys (from the wrapped consume call, any thread of the module: the pairs are read in cr_pre)
static void sl_key(int key, double t)
{
    if (g_sl_nev >= 6) { for (int i = 1; i < 6; i++) g_sl_ev[i - 1] = g_sl_ev[i]; g_sl_nev = 5; }
    g_sl_ev[g_sl_nev].key = key; g_sl_ev[g_sl_nev].t = t; g_sl_nev++;
}
static int sl_lim_active() { return g_cs.mode == CSM_LIM && g_cs.on; }
// "is the cruise on" for the key code: in LIM it is ours, not the game's byte
static int sl_eff_on(const char *ga) { return g_cs.mode == CSM_LIM ? g_cs.on : (ga ? sense_cruise_on(ga) : 0); }
static char sl_letter() { return sl_lim_active() ? SL_LIM_LETTER : 0; }

// The cue hook (decision 48): ONE function per event; a placeholder existing tone each. The spoken cue ("Limiter", "Cruise", "Eco") plugs in here later.
static void sl_cue(int ev)
{
    if (ev <= SLE_NONE || ev > SLE_NO) return;
    g_sl_cue_n[ev]++;
    static const int tone[] = { -1, CUE_ACC_1, CUE_ACC_OFF, CUE_ACC_2, CUE_ACC_OFF, CUE_OFF };   // CC on, CC off, LIM on, LIM off, no (placeholders)
    play_cue(tone[ev]);
}

// Once per tick before the game's own controller (cr_pre, after the key code): the state step, and what it says goes into the game. Returns 1 when
// it wrote the game's cruise byte (the caller stamps its key time so the cruise-drop tone and the Stop&Go re-arm do not take it for a drop).
static int sl_pre(char *ga, const sl_env &e)
{
    if (!ga) return 0;
    cs_cfg c = cs_cfg_default();
    c.presses = (int)(g_slc.presses + 0.5f); c.window_s = g_slc.window_s; c.min_on_kmh = e.min_on_kmh; c.min_set_kmh = e.min_set_kmh; c.max_set_kmh = e.max_set_kmh;
    c.show_set = g_slc.show_set > 0; c.isa_off_kmh = e.isa_off_kmh; c.isa_pct = e.isa_pct; c.ats = e.ats; c.unit_kmh = e.unit_kmh;
    cs_in in; memset(&in, 0, sizeof in);
    in.now = wall_s(); in.game_on = sense_cruise_on(ga) ? 1 : 0; in.game_set = sense_cruise_set(ga); in.speed_kmh = fabsf(T.speed) * 3.6f;
    in.brake = T.in_brake > 0.05f; in.isa = e.isa; in.limit_ms = e.limit_ms; in.since_res_s = e.since_res_s;
    in.n_ev = g_sl_nev; for (int i = 0; i < g_sl_nev; i++) in.ev[i] = g_sl_ev[i];
    g_sl_nev = 0;
    cs_out o; cs_step(c, &g_cs, in, &o);
    int wrote = 0;
    if (o.want_game_on >= 0 && (sense_cruise_on(ga) != 0) != (o.want_game_on != 0)) { act_cruise_on(ga, (unsigned char)(o.want_game_on ? 1 : 0)); wrote = 1; }
    if (o.want_game_set == o.want_game_set && fabsf(sense_cruise_set(ga) - o.want_game_set) > 1e-4f) act_cruise_set(ga, o.want_game_set);
    if (o.cue) sl_cue(o.cue);
    if (o.note[0]) logline("Cruise (layers): %s (%s, %s, set %.0f km/h)", o.note, g_cs.mode == CSM_LIM ? "LIM" : "CC", g_cs.on ? "on" : "off", g_cs.set);
    return wrote;
}

// The arbiter for the CC (one tick). set_kmh: the cruise set; band_kmh: how far over it the truck may run before the brake comes in (the descent allowance);
// lead_kmh: the game's ACC target when it follows a vehicle below the set (NAN = none). The plant is read off the model.
// WP-SP2: the cruise's set goes in first (begin), then whoever asks about the road, then the decision (run). The set limits what the truck chases and lifts the throttle;
// it never brakes by itself: a truck that is far over the set (a set that was lowered, a descent) is the descent hold's and the limit line's business, which brake only
// when coasting is not enough (core/req_legal.h). It is relative to the set (a raise from the PCC or the eco-roll's boost lifts it).
static void sl_cc_begin(float set_kmh, int isa_following)
{
    sa_begin(&g_sa);
    sa_req q; memset(&q, 0, sizeof q);
    q.owner = SAO_SET; q.cls = SAC_DRIVER; q.v_ms = set_kmh / 3.6f; q.accel_to = -1; q.flags = SAF_SET | SAF_NOBRAKE;
    q.why = isa_following ? "cruise set (speed limit)" : "cruise set";
    sa_post_req(&g_sa, q);
}
static void sl_cc_run(float dt, int mode, int shifting, int no_force = 0)
{
    const float v = fabsf(T.speed);
    // (no_force: WP-SP2b, the model cannot tell the force of this truck: the arbiter decides without one, out.nf = 1)
    sa_in in; in.v = v; in.dt = dt; in.mode = mode; in.shifting = shifting; in.mass = V.mass; in.f_hold = vehicle_resist(v); in.f_full = no_force ? 0.0f : wish_full(v);
    in.nat = vehicle_natural_decel(T.gear, v); in.accel = T.accel;
    sa_decide(g_slc.arb, &g_sa, in, &g_sl_out);
}
// The log line of a decision (P7, P9): when what it does, who asks or why changes (not the numbers that move all the way along an approach), at most once a second.
// The detail (the distance, the speed asked) is in the line; a decision that only moves along a plan does not repeat it.
static void sl_log_decision(const sa_out &o)
{
    char key[160]; sa_describe_key(o, key, sizeof key);
    if (g_sa.dropped) { logline("Cruise (layers): %d requests over the arbiter's %d were dropped", g_sa.dropped, SA_MAX_REQ); g_sa.dropped = 0; }
    if (strcmp(key, g_sl_line) && wall_s() - g_sl_line_t > 1.0) {
        char b[220]; sa_describe(o, b, sizeof b);
        logv(1, "Cruise (layers): %s", b); snprintf(g_sl_line, sizeof g_sl_line, "%s", key); g_sl_line_t = wall_s();
    }
}

// The fade where the model cannot tell (no gear, the clutch open and no cap held, no engine data, an electric truck): the old own limiter's, on the limit of LIM: full while
// the speed in 1 s is more than 1.5 km/h under it, none once it reaches it. 1 = no fade now.
static float sl_lim_fade(float kmh, float accel)
{
    if (!sl_lim_active()) return 1.0f;
    const float lim = g_cs.set + 0.25f;
    if (kmh >= lim) return 0.0f;
    const float proj = kmh + accel * 3.6f * 1.0f;
    return fmaxf(0.0f, fminf(1.0f, (lim - proj) / 1.5f));
}

// The top speed the model and the gearbox see (speed_top_kmh, core/speedtarget.cpp: wish_limit_force, the gear choice), as the own limiter publishes its own: with
// LIM on it is the ceiling, moved out of the way by the kickdown blend (limit + 100 km/h x the weight: further than any truck goes, and continuous in the pedal);
// the own limiter's value (own_top, 0 = none) is the smaller of the two. Without it a flat pedal against LIM kept the gear the pull-away chose (rewrite sim,
// 33 t at 62: gear 10, +8 % fuel against gear 11 under the own limiter). Written only while LIM is on and once when it ends: with LIM off nothing changes.
static int g_sl_top_pub;
static float sl_top_kmh(float pedal, float own_top, int *write)
{
    if (!sl_lim_active()) { *write = g_sl_top_pub; g_sl_top_pub = 0; return own_top; }
    float lt = g_cs.set + sa_kick_w(pedal, g_slc.kick_from, g_slc.kick_full) * 100.0f;
    *write = 1; g_sl_top_pub = 1;
    return own_top > 0 ? fminf(own_top, lt) : lt;
}

// LIM's throttle ceiling for the one cap post of the module (act_thr_cap(ACT_CRUISE)): a share of the gear's full force; 1 = no ceiling now. NAN = the
// model cannot tell (no gear, the clutch open and no cap held, no engine data). pedal = the driver's pedal 0..1 (the kickdown pass), v in m/s.
static float sl_lim_cap(float pedal, float v)
{
    if (!sl_lim_active()) { g_lim_last = NAN; return 1.0f; }
    if (!V.eng_ok || V.electric || !(V.mass > 0)) return NAN;
    float full = T.gear >= 1 && clutch_seen() <= 0.05f ? vehicle_force_at(T.gear, v, 1.0f) : 0;
    if (!(full > 0)) return sim_s() - g_lim_last_at < 2 ? g_lim_last : NAN;   // a shift: the cap of before it (as the own limiter)
    const float lim = (g_cs.set + 0.25f) / 3.6f;   // it settles on its target, so a quarter over it: the speedometer shows the limit, not one under (as the own limiter)
    float cap = sa_ceiling_cap(v, lim, g_slc.lim_tau_s, V.mass, vehicle_resist(v), full);
    if (!(cap == cap)) return NAN;
    const float w = sa_kick_w(pedal, g_slc.kick_from, g_slc.kick_full);
    cap = sa_cap_pass(cap, w);
    g_lim_last = cap; g_lim_last_at = sim_s();
    {   // the log (P7, P9): when the ceiling starts to hold the pedal, when a kickdown passes it, when it lets go (at most once a second)
        const int st = pedal > cap + 0.02f ? (w >= 0.5f ? 2 : 1) : 0;
        static int last_st; static double last_at = -1e9;
        if (st != last_st && wall_s() - last_at > 1.0) {
            last_st = st; last_at = wall_s();
            if (st == 1) logv(1, "Cruise (layers): limiter %.0f km/h holds the throttle at %.2f (pedal %.2f, %.1f km/h)", g_cs.set, cap, pedal, v * 3.6f);
            else if (st == 2) logv(1, "Cruise (layers): the kickdown passes the limiter %.0f km/h (pedal %.2f, %.1f km/h)", g_cs.set, pedal, v * 3.6f);
            else logv(1, "Cruise (layers): the limiter %.0f km/h lets the pedal through (pedal %.2f, %.1f km/h)", g_cs.set, pedal, v * 3.6f);
        }
    }
    return cap;
}
