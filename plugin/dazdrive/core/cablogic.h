// The cab assist's decisions (modules/cab.cpp) as plain logic, so a sim can feed it engine / gear / throttle / hold sequences.
// No SDK, no globals.
//
// Auto parking brake (the electronic parking brake of a real truck), built now:
//   APPLY    when the engine is switched off while standing (under stand_kmh) and the parking brake is off.
//   RELEASE  when the driver drives off: engine on, a gear selected (dgear != 0), throttle over release_thr and the parking
//            brake on. This releases any parking brake, also one the driver set himself (he asked "everything done for me").
//            Not while paused.
// When the driver (not us) changes the parking brake, the apply is left alone for driver_hold_s; the drive-off release is not
// held back (that is the whole point of it). An engine off from our own start-stop (ss_off) is NOT the driver switching off,
// so the parking brake is left alone then.
//
// Start-stop (like a modern car), built now:
//   STOP   when Auto Hold is holding, standing (< stand_kmh), the engine has run at least 60 s since it last started, forward
//          gear or neutral (not reverse), no throttle, the air pressure is fine, not paused, and the parking brake is off -
//          and all of that has held for ss_after_s.
//   START  again (only an engine we stopped, never one the driver switched off) on throttle, the radar lead moving away
//          (faster than ss_lead_kmh or its distance grew by ss_lead_gain_m), the air pressure under ss_min_psi - 10, Auto Hold
//          letting go (but not when the parking brake holds the truck instead: that handover is not a reason to start), reverse
//          selected, or after ss_max_s as a safety.
//
// Two decisions come out each call: the parking brake (want) and the engine (ss_want, the desired engine state: 0 = stop it,
// 1 = start it, -1 = leave it). The engine control is the game's toggle "engine": a press flips it, so the module never presses
// while the state is right and never re-presses a slow start (toggle.h's per-control verify time, 4 s for the engine).
//
// Every call names the term that decided (or, when nothing is decided, the one that holds the decision back) for both decisions, in cab_out::term /
// ss_term and in the why texts with the value and the threshold (promise P7); the module logs a line when a decision changes.
#pragma once
#include <math.h>
#include <string.h>
#include <stdio.h>

struct cab_cfg {
    float stand_kmh;       // at most this speed (km/h) counts as standing
    float release_thr;     // throttle (0..1) over this is "gas": the drive-off release, and the start-stop's restart
    float driver_hold_s;   // after the driver changed the parking brake: leave the apply alone this long
    // start-stop
    float ss_after_s;      // engine off after Auto Hold has held this long (s)
    float ss_min_psi;      // no stop below this brake air pressure (psi); air_psi <= 1 = not reported = fine
    float ss_max_s;        // safety: the engine comes back on after this long stopped (s)
    float ss_lead_kmh;     // restart when the lead drives off faster than this (km/h)
    float ss_lead_gain_m;  // ... or when the lead's distance grew by this (m) since the stop
    float ss_min_run_s;    // the engine must have run this long (s) since it started before the start-stop stops it
    float ss_psi_hyst;     // the engine restarts when the air pressure falls this far (psi) under ss_min_psi
};
static inline cab_cfg cab_cfg_default()
{
    cab_cfg c;
    c.stand_kmh = 1;       // |speed| under 1 km/h = standing (the spec's figure, not the 0.5 m/s of the game's own); constant: the game's speed at rest is a few cm/s of noise
    c.release_thr = 0.05f; // the same pedal edge the rest of the suite uses for "gives gas" (the lift zone of core/wish.h, the gearbox's "pedal off")
    c.driver_hold_s = 30;  // like the lights' driver hold; policy, not a vehicle quantity
    c.ss_after_s = 3;      // a real start-stop waits a short while, then goes off every stop; policy (ini ss_after_s)
    // Air: a truck's compressor governor cuts in at about 100 psi (6.9 bar) and out at 120-130 psi, the low-air warning comes on at 60-75 psi (4.5-5.2 bar).
    // Stopping needs the reservoir above the cut-in, and the engine restarts 10 psi under it, well before the warning.
    c.ss_min_psi = 100;
    c.ss_psi_hyst = 10;
    c.ss_max_s = 180;      // a start-stop that lasts this long is a fault: start it again; policy (ini ss_max_s)
    c.ss_lead_kmh = 3;     // the lead is clearly driving off: a crawl, faster than any creep of a queue; not derived
    c.ss_lead_gain_m = 1.5f;   // the same scale as ss_lead_kmh over the time an engine takes to crank (3 km/h x 1.8 s); not derived
    c.ss_min_run_s = 60;   // a diesel with a turbocharger idles 30-60 s before it is shut down (oil to the turbo); a stop at a light right after the start waits
    return c;
}

// The pedal as the assists read it. With the engine off the game's telemetry can show the pedal nowhere (ATS 1.61, Crown Victoria:
// ride 31-10 input_throttle 0.00 while effective_throttle read 0.23-1.0; ride 31-10 later both channels stayed 0.00 for the whole
// 35 s of the stop while the driver held the pedal, and the engine only came back when he turned the key). What does still arrive is the
// pedal argument of the game's cruise function, called every physics step, engine on or off (the actuator keeps its last value: the
// game_pedal here, NaN when it has none fresh). So with the engine off the largest of the three counts; with the engine on the input is
// what it always was.
static inline float cab_gas(int engine_on, float in_thr, float eff_thr, float game_pedal = NAN)
{
    float g = in_thr == in_thr ? in_thr : 0;
    if (!engine_on) {
        if (eff_thr == eff_thr && eff_thr > g) g = eff_thr;
        if (game_pedal == game_pedal && game_pedal > g) g = game_pedal;
    }
    return g < 0 ? 0 : g > 1 ? 1 : g;
}

struct cab_in {
    double t;
    int engine_on;        // the engine is running
    int paused;           // the game is paused: no decision
    float kmh;            // |speed|, km/h
    int parking;          // the parking brake state (0/1) the decision starts from
    int dgear;            // the gear the driver selected (0 = none / neutral, < 0 = reverse)
    float thr;            // throttle input, 0..1
    int driver_changed;   // the driver (not us) changed the parking brake since the last call
    // start-stop inputs
    int ah_holding;       // Auto Hold is holding the truck now (cruise.cpp: AH.st == AH_HOLD)
    float air_psi;        // brake air pressure (psi; <= 1 = not reported)
    int air_warning;      // the truck's low-air warning is on
    int lead_seen;        // a vehicle ahead is seen (cruise.cpp: cruise_lead)
    float lead_m;         // its distance (m)
    float lead_kmh;       // its speed (km/h; 0 = unknown)
};

// The term that decided the parking brake this call, or, when nothing is decided, the one that holds it back (CAT_NONE = not looked at).
enum cab_term {
    CAT_NONE = 0,
    CAT_PAUSED,           // the game is paused
    CAT_DRIVE_OFF,        // RELEASE: engine on, a gear, gas over release_thr, the brake on
    CAT_AH_OWNS,          // (the module) the release is left to Auto Hold, which holds the brake itself
    CAT_ENGINE_OFF,       // APPLY: engine off at a standstill, the brake off
    CAT_DRIVER_HOLD,      // engine off at a standstill, but the driver changed the brake less than driver_hold_s ago
    CAT_SS_STOP,          // engine off by our start-stop: the brake is left alone
    CAT_ROLLING,          // engine off, rolling
    CAT_ALREADY_ON,       // engine off, the brake is already on
    CAT_NO_DRIVE_OFF,     // the brake is on, no drive-off
    CAT_IDLE,             // engine on, brake off: nothing to do
    CAT_N
};
static inline const char *cab_term_name(int t)
{
    static const char *const n[CAT_N] = { "none", "paused", "drive_off", "ah_owns", "engine_off", "driver_hold", "ss_stop", "rolling", "already_on", "no_drive_off", "idle" };
    return t >= 0 && t < CAT_N ? n[t] : "?";
}
// The same for the engine (start-stop).
enum cab_ss_term {
    CST_NONE = 0,
    CST_PAUSED,
    // the engine is off because of us: why it starts again, or why it stays off
    CST_ENGINE_BACK,      // the engine is on again (somebody else started it): our stop is over
    CST_START_GAS,        // START: gas over release_thr
    CST_START_LEAD,       // START: the lead drives off (speed or distance)
    CST_START_AIR,        // START: air pressure under ss_min_psi - ss_psi_hyst
    CST_START_HOLD_ENDED, // START: Auto Hold lets go and the parking brake does not hold instead
    CST_START_REVERSE,    // START: reverse selected
    CST_START_SAFETY,     // START: stopped for ss_max_s
    CST_OFF_PARK_HOLDS,   // stays off: the parking brake holds the truck
    CST_OFF_HOLDING,      // stays off: Auto Hold still holds
    // a stop is in flight
    CST_STOPPING,         // STOP goes on: the engine is switching off
    CST_STOP_DROPPED,     // the stop was dropped: no longer standing in the hold
    CST_STOPPED,          // the engine is off now (start-stop)
    // the engine is on
    CST_STOP,             // STOP: everything held for ss_after_s
    CST_COUNTDOWN,        // everything holds, counting ss_after_s
    CST_NOT_OURS,         // the engine is off and it is not our stop
    CST_NO_HOLD,          // Auto Hold is not holding
    CST_ROLLING,          // moving
    CST_PARKING,          // the parking brake is on
    CST_REVERSE,          // reverse selected
    CST_GAS,              // gas
    CST_AIR,              // air pressure or its warning
    CST_YOUNG,            // the engine started less than ss_min_run_s ago
    CST_N
};
static inline const char *cab_ss_term_name(int t)
{
    static const char *const n[CST_N] = { "none", "paused", "engine_back", "start_gas", "start_lead", "start_air", "start_hold_ended", "start_reverse", "start_safety",
                                          "off_park_holds", "off_holding", "stopping", "stop_dropped", "stopped", "stop", "countdown", "not_ours", "no_hold", "rolling",
                                          "parking", "reverse", "gas", "air", "young" };
    return t >= 0 && t < CST_N ? n[t] : "?";
}

struct cab_out {
    int want;             // -1 nothing to do, 0 release, 1 apply  (the parking brake)
    int term;             // cab_term: what decided (want set) or what holds it back (want -1)
    char why[160];        // why (when want is set) or what holds it back (when not), with the value and the threshold
    int ss_want;          // -1 nothing to do, 0 stop the engine, 1 start it  (the desired engine state)
    int ss_term;          // cab_ss_term, the same for the engine
    char ss_why[160];     // why (when ss_want is set) or what holds it back (when not)
};

struct cab_state {
    double driver_until;  // no apply before this time (the driver had his hands on the brake)
    // start-stop
    double engine_since;  // when the engine last came on (the 60 s minimum run)
    double ss_ready;      // the stop conditions have held since (0 = not now)
    double ss_stop_at;    // when our start-stop switched the engine off (0 = it is not off by us)
    double ss_lead_m;     // the lead's distance when the engine stopped
    int ss_had_lead;      // a lead was seen when the engine stopped (for the "distance grew" restart)
    int ss_off;           // 1 = our start-stop has the engine off right now
    int ss_pend;          // 1 = we decided to stop and are waiting for the engine to go off
    int prev_engine;      // the engine state of the last call (to see a start)
};

static inline void cab_reset(cab_state *s) { memset(s, 0, sizeof *s); s->driver_until = -1e9; s->engine_since = -1e9; s->prev_engine = -1; }

static inline void cab_step(cab_state *s, const cab_in &in, const cab_cfg &c, cab_out *o)
{
    o->want = -1; o->why[0] = 0; o->term = CAT_NONE;
    o->ss_want = -1; o->ss_why[0] = 0; o->ss_term = CST_NONE;
    size_t n = sizeof o->why, ns = sizeof o->ss_why;
    double t = in.t;
    if (s->prev_engine < 0) { if (in.engine_on) s->engine_since = t; }                 // review 30-10: first sight of a running engine counts as just started
    else if (in.engine_on && !s->prev_engine) s->engine_since = t;                     // the engine just started
    s->prev_engine = in.engine_on;
    if (in.driver_changed) s->driver_until = t + c.driver_hold_s;
    if (in.paused) { o->term = CAT_PAUSED; o->ss_term = CST_PAUSED; snprintf(o->why, n, "paused"); snprintf(o->ss_why, ns, "paused"); return; }

    // ---- start-stop ----
    int air_fine = in.air_psi <= 1 || (!in.air_warning && in.air_psi >= c.ss_min_psi);
    if (s->ss_off) {
        // The engine is off because of us: start it again only on gas, the lead driving off, low air, the hold ending, reverse,
        // or after ss_max_s. A running engine means someone else started it - our stop is simply over.
        if (in.engine_on) {
            s->ss_off = 0; s->ss_pend = 0;
            o->ss_term = CST_ENGINE_BACK;
            snprintf(o->ss_why, ns, "the engine is on again (not our stop any more)");
        } else {
            int air_low = in.air_psi > 1 && in.air_psi < c.ss_min_psi - c.ss_psi_hyst;
            int lead_fast = in.lead_seen && in.lead_kmh > c.ss_lead_kmh;
            int lead_far = in.lead_seen && s->ss_had_lead && in.lead_m > s->ss_lead_m + c.ss_lead_gain_m;
            if (in.thr > c.release_thr) {
                o->ss_want = 1; o->ss_term = CST_START_GAS; snprintf(o->ss_why, ns, "gas %.0f %% over %.0f %%", in.thr * 100, c.release_thr * 100);
            } else if (lead_fast || lead_far) {
                o->ss_want = 1; o->ss_term = CST_START_LEAD;
                if (lead_fast) snprintf(o->ss_why, ns, "the lead drives off at %.0f km/h (over %.0f), %.0f m ahead", in.lead_kmh, c.ss_lead_kmh, in.lead_m);
                else snprintf(o->ss_why, ns, "the lead is %.1f m away, %.1f m further than at the stop (over %.1f m)", in.lead_m, in.lead_m - (float)s->ss_lead_m, c.ss_lead_gain_m);
            } else if (air_low) {
                o->ss_want = 1; o->ss_term = CST_START_AIR; snprintf(o->ss_why, ns, "air %.0f psi under %.0f psi", in.air_psi, c.ss_min_psi - c.ss_psi_hyst);
            } else if (!in.ah_holding && !in.parking) {
                o->ss_want = 1; o->ss_term = CST_START_HOLD_ENDED; snprintf(o->ss_why, ns, "Auto Hold is not holding any more");
            } else if (in.dgear < 0) {
                o->ss_want = 1; o->ss_term = CST_START_REVERSE; snprintf(o->ss_why, ns, "reverse selected (gear %d)", in.dgear);
            } else if (t - s->ss_stop_at >= c.ss_max_s) {
                o->ss_want = 1; o->ss_term = CST_START_SAFETY; snprintf(o->ss_why, ns, "stopped %.0f s (the safety limit is %.0f s)", (float)(t - s->ss_stop_at), c.ss_max_s);
            } else if (!in.ah_holding) {
                o->ss_term = CST_OFF_PARK_HOLDS; snprintf(o->ss_why, ns, "engine off (start-stop), the parking brake holds the truck");
            } else {
                o->ss_term = CST_OFF_HOLDING; snprintf(o->ss_why, ns, "engine off (start-stop), still holding");
            }
        }
    } else if (s->ss_pend) {
        // The stop was decided and the press is in flight: keep it until the engine is off, or drop it if the hold is over.
        if (in.engine_on) {
            int can_stop = in.ah_holding && in.kmh < c.stand_kmh && !in.parking && in.dgear >= 0
                           && in.thr <= 0 && air_fine && (t - s->engine_since >= c.ss_min_run_s);
            if (can_stop) { o->ss_want = 0; o->ss_term = CST_STOPPING; snprintf(o->ss_why, ns, "engine switching off"); }
            else { s->ss_pend = 0; o->ss_term = CST_STOP_DROPPED; snprintf(o->ss_why, ns, "no longer standing in the hold"); }
        } else {
            s->ss_off = 1; s->ss_stop_at = t; s->ss_lead_m = in.lead_seen ? in.lead_m : 0; s->ss_had_lead = in.lead_seen; s->ss_pend = 0;
            o->ss_term = CST_STOPPED;
            snprintf(o->ss_why, ns, "engine off (start-stop)");
        }
    } else {
        int can_stop = in.engine_on && in.ah_holding && in.kmh < c.stand_kmh && !in.parking && in.dgear >= 0
                       && in.thr <= 0 && air_fine && (t - s->engine_since >= c.ss_min_run_s);
        if (can_stop) {
            if (!s->ss_ready) s->ss_ready = t;
            if (t - s->ss_ready >= c.ss_after_s) {
                o->ss_want = 0; o->ss_term = CST_STOP;
                snprintf(o->ss_why, ns, "Auto Hold holds, %.1f km/h (under %.1f), no gas, air %.0f psi (from %.0f), held %.1f s (of %.1f)", in.kmh, c.stand_kmh, in.air_psi,
                         c.ss_min_psi, (float)(t - s->ss_ready), c.ss_after_s);
                s->ss_pend = 1; s->ss_ready = 0;
            } else {
                o->ss_term = CST_COUNTDOWN;
                snprintf(o->ss_why, ns, "%.1f s to the stop (of %.1f)", c.ss_after_s - (float)(t - s->ss_ready), c.ss_after_s);
            }
        } else {
            s->ss_ready = 0;
            if (!in.engine_on) { o->ss_term = CST_NOT_OURS; snprintf(o->ss_why, ns, "engine off (not ours)"); }
            else if (!in.ah_holding) { o->ss_term = CST_NO_HOLD; snprintf(o->ss_why, ns, "Auto Hold is not holding"); }
            else if (in.kmh >= c.stand_kmh) { o->ss_term = CST_ROLLING; snprintf(o->ss_why, ns, "rolling at %.1f km/h (standing is under %.1f)", in.kmh, c.stand_kmh); }
            else if (in.parking) { o->ss_term = CST_PARKING; snprintf(o->ss_why, ns, "parking brake on"); }
            else if (in.dgear < 0) { o->ss_term = CST_REVERSE; snprintf(o->ss_why, ns, "reverse selected (gear %d)", in.dgear); }
            else if (in.thr > 0) { o->ss_term = CST_GAS; snprintf(o->ss_why, ns, "gas %.0f %%", in.thr * 100); }
            else if (!air_fine) { o->ss_term = CST_AIR; { if (in.air_warning) snprintf(o->ss_why, ns, "low-air warning on (air %.0f psi)", in.air_psi); else snprintf(o->ss_why, ns, "air %.0f psi under %.0f psi", in.air_psi, c.ss_min_psi); }  }
            else { o->ss_term = CST_YOUNG; snprintf(o->ss_why, ns, "engine started %.0f s ago (the stop needs %.0f s)", t - s->engine_since, c.ss_min_run_s); }
        }
    }

    // ---- auto parking brake ----
    // drive off: the driver asked "everything done for me" - release whatever is on, also his own
    if (in.engine_on && in.dgear != 0 && in.thr > c.release_thr && in.parking) {
        o->want = 0; o->term = CAT_DRIVE_OFF;
        snprintf(o->why, n, "driving off: engine on, gear %d, throttle %.0f %% (over %.0f %%)", in.dgear, in.thr * 100, c.release_thr * 100);
        return;
    }
    // engine off at a standstill: put the brake on - unless that engine off is our own start-stop (not the driver switching off)
    if (!in.engine_on && !s->ss_off && in.kmh < c.stand_kmh && !in.parking) {
        if (t < s->driver_until) {
            o->term = CAT_DRIVER_HOLD;
            snprintf(o->why, n, "engine off at a standstill, but the driver changed the parking brake %.0f s ago (hands off for %.0f s)", c.driver_hold_s - (float)(s->driver_until - t), c.driver_hold_s);
            return;
        }
        o->want = 1; o->term = CAT_ENGINE_OFF;
        snprintf(o->why, n, "engine off at a standstill (%.1f km/h, under %.1f)", in.kmh, c.stand_kmh);
        return;
    }
    if (!in.engine_on && s->ss_off) { o->term = CAT_SS_STOP; snprintf(o->why, n, "engine off (start-stop), parking brake left alone"); }
    else if (!in.engine_on && in.kmh >= c.stand_kmh) { o->term = CAT_ROLLING; snprintf(o->why, n, "engine off and rolling at %.1f km/h (standing is under %.1f)", in.kmh, c.stand_kmh); }
    else if (!in.engine_on) { o->term = CAT_ALREADY_ON; snprintf(o->why, n, "engine off, parking brake already on"); }
    else if (in.parking) { o->term = CAT_NO_DRIVE_OFF; snprintf(o->why, n, "parking brake on, no drive-off (gear %d, throttle %.0f %%, needs over %.0f %%)", in.dgear, in.thr * 100, c.release_thr * 100); }
    else { o->term = CAT_IDLE; snprintf(o->why, n, "nothing to do"); }
}
