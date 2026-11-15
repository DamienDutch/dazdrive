// The engine brake lever (modules/ebrake.cpp) as plain logic, so a sim can run it against a model of the game's own handler.
// No SDK, no globals.
//
// What the game does (1.61 key handler, static RE): the engine brake has an on/off toggle and a stage lever (up / down). The state is
// `on` (a byte), `stage` (0..cap, kept while the brake is off) and a flag it writes on every press. The handler's own rules:
//   toggle  flips `on`; switching on at stage 0 sets stage = cap.
//   up      if stage < cap: stage + 1, and `on` = 1 only when a call (a binding check on the toggle) returns false.
//   down    if stage > (that call's result): stage - 1; reaching 0 sets `on` = 0.
// So with the brake off and the stage left at cap by an earlier toggle, up does nothing; with that call true, up never switches the
// brake on and down never reaches off. A retarder lever is simpler: up from off = stage 1, up = next stage, down = previous, down
// at stage 1 = off.
//
// The module wraps the call that consumes the up / down key event, which runs BEFORE the handler acts on the press, and puts the
// state where the handler's own rules then give exactly the lever behavior, for either result of that call:
//   up   while off: on = 1, stage = 0   (the handler then raises it to 1)
//   down while on at stage <= 1: on = 0, stage = 0   (the handler then finds nothing to lower)
// Everything else is left to the handler (up while on = next stage, clamped at cap; down while on above 1 = previous stage).
#pragma once
#include <math.h>

struct eb_state { int on; unsigned stage; unsigned cap; };

enum { EB_CAP_MAX = 16 };   // a sane stage count (real engines have 1..6); anything else is a misread and the lever stays the game's

// act = 0: leave the press to the game. act = 1: set `on` and `stage` to the values given before the game handles the press.
// cancel_cruise = 1: the press switches the brake on or raises a stage (a driver's press, like a retarder lever in ETS2): the game's cruise
// goes off with it. A press down or off does not cancel (see the module).
struct eb_pre { int act; int on; unsigned stage; int cancel_cruise; };

// paused: the module does nothing while the game is paused (a press then is the game's own). engine_on: switching the brake ON and the
// stages up need the engine, but switching it OFF (down at stage 1) works with the engine off too: start-stop (Cab) stops the engine at a
// standstill and the lever would otherwise be stuck on (ride log 01-11: down pressed 6 times, "engine off: left to the game").
static inline eb_pre eb_before_press(int dir, const eb_state &s, int paused, int engine_on)
{
    eb_pre p = { 0, s.on, s.stage, 0 };
    if (paused) return p;
    if (s.cap < 1 || s.cap > EB_CAP_MAX) return p;
    if (dir < 0 && s.on && s.stage <= 1) { p.act = 1; p.on = 0; p.stage = 0; return p; }
    if (!engine_on) return p;
    if (dir > 0 && !s.on) { p.act = 1; p.on = 1; p.stage = 0; p.cancel_cruise = 1; }
    else if (dir > 0 && s.stage < s.cap) p.cancel_cruise = 1;
    return p;
}

// ---------------------------------------------------------------- the descent control (modules/ebrake.cpp)
//
// Under cruise on a descent, a truck WITHOUT a retarder holds its speed with the engine brake stages. What real trucks do: a Cummins or
// Detroit engine brake on "auto" with the cruise set applies the stages one at a time when the speed goes over the set speed by a few
// mph, and takes them off again one at a time when it is back; trucks with a retarder use the retarder first (it is stronger, quiet, and
// needs no gear). So this control only runs without a retarder (a truck with one keeps the retarder and the brake channel as before).
//
// What the game does by itself (cruise rules in its pedal function): with no retarder and an engine brake, the cruise's overspeed
// braking asks the engine brake at min(speed - A, 1.0) of the full strength (A = where its overspeed braking starts; 1 km/h over = all
// stages at once), and, with the handler's flag set, no more than the stage value of the lever. Full at 1 km/h over A and nothing a
// little under it: on a long descent that is a full-on / off cycle with a gear change each time. This control puts the stages in between:
//   up    one stage when the speed (plus what it gains in `lead_s`) is within `margin` of A and the truck is not already slowing well,
//         at most one step per `step_s` and not during a gear change (the engine brake does nothing then), unless far over A
//   down  one stage when the speed (less what it loses in `lead_dn_s`) is back near the set speed, at most one step per `down_s`, and
//         only if one stage less would not speed the truck up again (what a stage is worth is measured on the steps up; otherwise it
//         would only have to step back up: the on / off cycle); all
//         stages at once when the speed is at the set speed, the throttle is on (the cruise adds throttle under the set speed: never both) or the cruise is off
// The state is the game's own (on, stage, flag): the control puts it through the stage setter and gives the driver's previous state
// back when it ends. The driver's own keys win: any change of the state we did not make hands the brake back to the driver.

struct eb_full { int on; unsigned stage; int flag; int held; };

struct eb_desc_cfg {
    float margin_kmh;   // the first stage comes this far under A (the game's own overspeed braking would start at A)
    float lead_s;       // look-ahead on a rising speed (s)
    float slow_ms2;     // slowing at least this much (m/s2) holds the stage (no step up) while within `far_kmh` of A
    float far_kmh;      // over A by this much a step up goes ahead whatever (gear change, slowing)
    float step_s;       // minimum time between steps up (s)
    float shift_s;      // no step up this long after a gear change (s)
    float down_s;       // minimum time between a step and a step down (s)
    float lead_dn_s;    // look-ahead on a falling speed for a step down (s)
};
static inline eb_desc_cfg eb_desc_defaults() { eb_desc_cfg c = { 0.5f, 1.0f, 0.15f, 2.0f, 2.0f, 1.5f, 4.0f, 2.0f }; return c; }

struct eb_desc {
    int stage;           // the stage the control asks now (0 = none)
    double changed_at;   // when it last changed
    double gear_at;      // when a gear change was last seen
    int gear;            // the gear then
    double step_at;      // when the last step up was made, and the acceleration then (a_step): the next look 1.8 s later measures the stage's worth
    float a_step;
    int measured;
    int step_stage;      // the stage that step went to
    float d_meas;        // OUT: how much slower (m/s2) the truck accelerated for that step (> 0 once measured; the caller notes it to the vehicle model and sets it to 0)
    int meas_stage;      // OUT: the stage d_meas belongs to
};
struct eb_desc_in {
    double now;
    float kmh, from_kmh, target_kmh, accel_ms2, thr, pedal;
    int cruise_on, engine_on, paused, clutch_open, gear;
    unsigned cap;        // stages of the engine
    float d_stage;       // what one stage is worth here (m/s2 slower per stage up): the vehicle model's, learned per truck (core/vehicle.h, the engine brake)
};

// The stage to ask now (0..cap). `why` (optional) says what decided, for the log.
static inline int eb_desc_target(eb_desc &s, const eb_desc_in &in, const eb_desc_cfg &c, const char **why = 0)
{
    const char *w = "hold";
    int st = s.stage;
    if (in.gear != s.gear) { s.gear = in.gear; s.gear_at = in.now; }
    bool go = in.cruise_on && in.engine_on && !in.paused && in.cap >= 1 && in.cap <= EB_CAP_MAX && in.from_kmh == in.from_kmh && in.target_kmh == in.target_kmh;
    if (!go) { st = 0; w = "no cruise"; }
    else if (in.thr > 0.08f || in.pedal > 0.02f) { st = 0; w = "throttle"; }
    else {
        float A = in.from_kmh, T0 = in.target_kmh;
        float down_edge = fminf(1.5f, fmaxf(0.6f, 0.1f * (A - T0)));         // back near the target: the stages come off (the wider the band, the longer a cycle between two stages)
        float up_edge = fmaxf(A - c.margin_kmh, T0 + down_edge + 0.5f);       // (a narrow allowance still leaves a band)
        float pred = in.kmh + fminf(1.5f, fmaxf(0.0f, in.accel_ms2)) * 3.6f * c.lead_s;
        float pred_dn = in.kmh + fminf(0.0f, in.accel_ms2) * 3.6f * c.lead_dn_s;   // a stage that slows the truck hard comes off before the speed is down at the target
        double since = in.now - s.changed_at;
        bool settling = in.clutch_open || in.now - s.gear_at < c.shift_s;
        // what one stage is worth here (m/s2, slower per stage up) is the vehicle model's (in.d_stage); the step up that just ran is a measurement of it, handed back in d_meas
        if (s.step_at > 0 && !s.measured && in.now - s.step_at >= 1.8 && !in.clutch_open && in.gear == s.gear) {
            float d = s.a_step - in.accel_ms2;
            if (d > 0.03f) { s.d_meas = d; s.meas_stage = s.step_stage; }
            s.measured = 1;
        }
        bool low_ok = in.accel_ms2 + in.d_stage <= 0.10f;   // one stage less would not speed it up again (then it only has to step back up: an on / off cycle)
        if (st > 0 && in.kmh < T0 + 0.3f) { st = 0; w = "under the target"; }
        else if ((unsigned)st < in.cap && pred > up_edge && since >= c.step_s &&
                 (in.accel_ms2 > -c.slow_ms2 || in.kmh > A + c.far_kmh) && (!settling || in.kmh > A + c.far_kmh + 0.5f)) { st++; w = "up"; s.step_at = in.now; s.a_step = in.accel_ms2; s.measured = 0; s.step_stage = st; }
        else if (st > 0 && pred_dn < T0 + down_edge && low_ok && since >= c.down_s) { st--; w = st ? "down" : "off"; }
    }
    if (st != s.stage) s.changed_at = in.now;
    s.stage = st;
    if (why) *why = w;
    return st;
}

// Did the driver touch the engine brake since we last set it? `ours` = the state we left it in. A key press changes the on byte or the stage;
// a held key (the momentary one) sets `held`.
static inline int eb_driver_touched(const eb_full &ours, const eb_full &now)
{
    return now.held || now.on != ours.on || now.stage != ours.stage;
}
