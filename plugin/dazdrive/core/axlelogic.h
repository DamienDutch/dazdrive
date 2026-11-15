// The lift axle rule (modules/axle.cpp) as plain logic, so a sim can feed it mass and speed sequences. No SDK, no globals.
//
// The game lowers a lift axle by itself (load, workshop, coupling) and never raises it. A real truck raises it when the remaining
// axles can carry the load. The rule, per lift axle (the truck's and the first trailer's, each on its own):
//   RAISE  when it is down and, for the same time, speed is steady above min_kmh, the road is not steep, no wheel slips, nobody
//          touched the control for ext_hold_s, and the mass is at most raise_frac x the legal gross weight of the axles that would
//          stay down (axl_gvw_limit_kg) -- and not over the cap learned from the game (below).
//   LOWER  when it is up and the mass is over that limit (no margin: raise_frac is the hysteresis band), or the wheels slip at low speed.
// The cap: when the control goes down by itself (not our press) within 10 s of a load change (a coupling, or the mass stepping up
// by 3 % or more), the mass at that moment is an upper bound of what the game itself accepts with that axle up. The limit is then
// min(legal limit, that mass). It is forgotten with another truck.
//
// Every call names the term that decided (or, when nothing is decided, the one that holds the decision back) in axle_dec::term and in
// the why text with its value and threshold (promise P7); the module logs a line when the decision changes.
#pragma once
#include <math.h>
#include <string.h>
#include <stdio.h>

// Legal gross weight (kg) of a vehicle with this many axles in contact with the road, EU Directive 96/53/EC (the rule the game's
// trucks follow): rigid 2 axles 18 t, 3 axles 26 t, 4 axles 32 t; tractor + semitrailer or truck + trailer: 3 axles 28 t, 4 axles 36 t,
// 5 or more 40 t (the 44 t of intermodal transport is not assumed).
// Not derivable from the vehicle (the SDK gives no axle loads or legal weights): a legal table, EU only. The US limits differ (ATS: 80,000 lb = 36.3 t
// for 5 axles, the federal bridge formula for the rest) and choosing between them needs a region signal nobody has; the cap learned from the game
// (below) replaces it for that truck as soon as the game lowers the axle once. Listed as open in docs/architecture.md (migration, small readers).
static inline float axl_gvw_limit_kg(int axles, int has_trailer)
{
    if (!has_trailer) return axles <= 2 ? 18000.f : axles == 3 ? 26000.f : 32000.f;
    return axles <= 3 ? 28000.f : axles == 4 ? 36000.f : 40000.f;
}

// Axles from the wheels' positions along the vehicle: wheels within `tol` metres of each other are one axle (dual tyres share a z).
// Only wheels with use[i] set count (NULL = all). NAN positions are skipped. -1 = no position known.
// tol 0.5 m: dual tires share a z exactly, and the closest axles of a real group are about 1.0 m apart (96/53/EC's smallest axle spacing in a
// tri-axle group), so half of that separates the two with a margin on both sides. A constant of the geometry, not a tunable.
static inline int axl_count(const float *z, const int *use, int n, float tol = 0.5f)
{
    float seen[16];
    int ns = 0, any = 0;
    for (int i = 0; i < n && i < 16; i++) {
        if (use && !use[i]) continue;
        if (z[i] != z[i]) continue;
        any = 1;
        int dup = 0;
        for (int k = 0; k < ns; k++) if (fabsf(seen[k] - z[i]) < tol) dup = 1;
        if (!dup) seen[ns++] = z[i];
    }
    return any ? ns : -1;
}

struct axle_cfg {
    float raise_frac;     // raise at or under this share of the legal limit of the axles that stay down
    float min_kmh;        // steady speed needed to raise; under it a slipping truck gets its axle lowered
    float steady_s;       // how long speed, grade and mass must have been steady
    float accel_max;      // m/s2 still counted as steady
    float grade_max_pct;  // steeper than this: no raising
    float slip_max;       // mean driven-wheel slip (fraction) counted as trouble
    float ext_hold_s;     // after somebody else changed the control: leave it alone this long
    float act_gap_s;      // at least this long between two decisions for one axle
    float mass_step_frac; // a change of the mass by this share between two calls is a coupling or a load, not fuel burn (restarts the steady time)
    float learn_window_s; // the game lowering the axle within this long of a load change caps the limit at that mass
    float mass_settle_s;  // after a mass step: no raise for this long
    float slip_hold_s;    // after a slip over slip_max: no raise for this long
};
static inline axle_cfg axle_cfg_default()
{
    axle_cfg c;
    c.raise_frac = 0.80f;      // margin for an uneven axle load (the drive axle takes most of a tractor's share) and for the mass estimate
    c.min_kmh = 20;            // below it a manoeuvre, a junction or a queue: the axle is left as it is
    c.steady_s = 5;
    c.accel_max = 0.4f;        // an easy cruise; a launch or a braking is over 0.5
    c.grade_max_pct = 4;       // a motorway grade is up to 4-6 %; steeper loads the driven axle more
    c.slip_max = 0.10f;        // tyre creep on tarmac is a few %, the bend differential cancels in the mean of the driven wheels; 10 % is spin or lock
    c.ext_hold_s = 60;         // policy: the driver who lifts or lowers the axle by hand is left alone for a minute (no vehicle quantity decides it)
    c.act_gap_s = 20;          // policy: with act 1 a press and the game's answer take a few seconds (core/toggle.h: 2 tries + verify); 20 s is longer than both
    // The mass is the game's figure read every 0.5 s (core/vehicle.h), not an estimate: it moves between two reads by the fuel burnt (well under
    // 0.01 %) and by nothing else except a coupling or a load (the lightest trailer is a good tenth of a combination). 3 % sits between the two.
    c.mass_step_frac = 0.03f;
    c.learn_window_s = 10;     // not derived (the game's own delay after a coupling is not documented): long against the 0.5 s mass read, short against a key press
    c.mass_settle_s = 3;       // six reads of the mass (0.5 s each) without a step
    c.slip_hold_s = 5;         // not derived: the scale of steady_s, so a spin is not forgotten before the steady time would restart
    return c;
}

// The term that decided this call for one axle, or, when nothing is decided, the one that holds the decision back. The module logs it with the
// decision; the sims assert on it. AXT_NONE = the axle is not looked at (no lift axle, or the control's state is unknown).
enum axle_term {
    AXT_NONE = 0,
    AXT_NOT_DRIVING,      // engine off or paused
    AXT_MASS_UNKNOWN,     // the game has not told the mass
    AXT_OVER_LIMIT,       // up, the mass is over the limit: LOWER
    AXT_OVER_LIMIT_WAIT,  // ... but a decision was made less than act_gap_s ago
    AXT_SLIP_LOW,         // up, the driven wheels slip at a speed under min_kmh: LOWER
    AXT_SLIP_LOW_WAIT,
    AXT_UP_OK,            // up, the mass is within the limit: nothing to do
    AXT_FOREIGN_HOLD,     // down, somebody else changed the control less than ext_hold_s ago
    AXT_MASS_HIGH,        // down, the mass is over raise_frac x the limit
    AXT_NOT_STEADY,       // down, speed, acceleration or grade not steady for steady_s
    AXT_MASS_CHANGING,    // down, the mass stepped less than mass_settle_s ago
    AXT_SLIP_UNKNOWN,     // down, no wheel slip to judge by
    AXT_SLIP,             // down, the driven wheels slipped over slip_max less than slip_hold_s ago
    AXT_RAISE_WAIT,       // down, all holds, but a decision was made less than act_gap_s ago
    AXT_RAISE,            // down, all holds: RAISE
    AXT_N
};
static inline const char *axle_term_name(int t)
{
    static const char *const n[AXT_N] = { "none", "not_driving", "mass_unknown", "over_limit", "over_limit_wait", "slip_low", "slip_low_wait", "up_ok",
                                          "foreign_hold", "mass_high", "not_steady", "mass_changing", "slip_unknown", "slip", "raise_wait", "raise" };
    return t >= 0 && t < AXT_N ? n[t] : "?";
}

struct axle_in {
    double t;
    int ok;                     // engine on, not paused: nothing is decided otherwise
    float kmh, accel, grade_pct;
    float mass_kg; int mass_ok;
    float slip;                 // mean slip of the driven wheels, NAN = not known
    int has_trailer;
    int axles[2];               // axles in all: [0] truck, [1] first trailer (0 = none / unknown)
    int liftable[2];            // lift axles of each (0 = none known)
    int known[2], up[2];        // control state known / lifted
    int ours[2];                // a change of up[] since the last call answers our own press
};

struct axle_dec {
    int want[2];                // -1 nothing to do, 0 lower, 1 raise
    int term[2];                // axle_term: what decided (want set) or what holds it back (want -1)
    char why[2][160];           // why (when want is set) or what holds it back (when not), with the value and the threshold
    int learned[2];             // this call learned a cap from the game lowering the axle
};

struct axle_state {
    int last_up[2], last_known[2];
    double steady_since, mass_since, mass_up_t, couple_t, slip_until;
    float last_mass;
    double ext_until[2], act_at[2];
    float cap_kg[2];            // learned from the game, 0 = none
    float limit_kg[2];          // the limit used in the last decision (for the log)
};

static inline void axle_reset(axle_state *s, int hard)
{
    float cap[2] = { s->cap_kg[0], s->cap_kg[1] };
    memset(s, 0, sizeof *s);
    s->steady_since = s->mass_since = s->mass_up_t = s->couple_t = s->slip_until = -1e9;
    s->ext_until[0] = s->ext_until[1] = s->act_at[0] = s->act_at[1] = -1e9;
    s->last_up[0] = s->last_up[1] = s->last_known[0] = s->last_known[1] = -1;
    if (!hard) { s->cap_kg[0] = cap[0]; s->cap_kg[1] = cap[1]; }
}

// A trailer was (un)coupled: the mass is about to change and so is the game's idea of the axle.
static inline void axle_load_changed(axle_state *s, double t) { s->couple_t = t; s->steady_since = -1e9; s->mass_since = -1e9; s->last_mass = 0; }

static inline void axle_step(axle_state *s, const axle_in &in, const axle_cfg &c, axle_dec *out)
{
    for (int a = 0; a < 2; a++) { out->want[a] = -1; out->term[a] = AXT_NONE; out->why[a][0] = 0; out->learned[a] = 0; }
    double t = in.t;

    // steadiness
    int fast = in.kmh >= c.min_kmh;
    int calm = fabsf(in.accel) < c.accel_max && fabsf(in.grade_pct) < c.grade_max_pct;
    if (!in.ok || !fast || !calm) s->steady_since = -1e9;
    else if (s->steady_since < -1e8) s->steady_since = t;
    if (in.mass_ok) {
        if (s->last_mass > 0 && fabsf(in.mass_kg - s->last_mass) > c.mass_step_frac * s->last_mass) {
            s->mass_since = t;
            if (in.mass_kg > s->last_mass) s->mass_up_t = t;
        } else if (s->mass_since < -1e8) s->mass_since = t;
        s->last_mass = in.mass_kg;
    }
    if (in.slip == in.slip && fabsf(in.slip) > c.slip_max) s->slip_until = t + c.slip_hold_s;
    int slip_bad = t < s->slip_until;
    int slip_known = in.slip == in.slip;

    for (int a = 0; a < 2; a++) {
        if (in.liftable[a] <= 0 || !in.known[a]) { s->last_up[a] = in.up[a]; s->last_known[a] = in.known[a]; continue; }
        // somebody else changed the control?
        if (s->last_known[a] == 1 && in.up[a] != s->last_up[a] && !in.ours[a]) {
            s->ext_until[a] = t + c.ext_hold_s;
            if (!in.up[a] && in.mass_ok && (t - s->mass_up_t < c.learn_window_s || t - s->couple_t < c.learn_window_s)) {
                s->cap_kg[a] = in.mass_kg;
                out->learned[a] = 1;
            }
        }
        s->last_up[a] = in.up[a]; s->last_known[a] = 1;

        int other = 1 - a;
        int total = in.axles[0] + in.axles[1];
        int down = total - in.liftable[a] - (in.up[other] && in.known[other] ? in.liftable[other] : 0);
        float lim = axl_gvw_limit_kg(down, in.has_trailer);
        if (s->cap_kg[a] > 0 && s->cap_kg[a] < lim) lim = s->cap_kg[a];
        s->limit_kg[a] = lim;
        char *w = out->why[a];
        size_t n = sizeof out->why[a];
        if (!in.ok) { out->term[a] = AXT_NOT_DRIVING; snprintf(w, n, "not driving (engine off or paused)"); continue; }
        if (!in.mass_ok) { out->term[a] = AXT_MASS_UNKNOWN; snprintf(w, n, "mass unknown (the game has not told it)"); continue; }
        if (in.up[a]) {
            if (in.mass_kg > lim) {
                if (t - s->act_at[a] < c.act_gap_s) { out->term[a] = AXT_OVER_LIMIT_WAIT; snprintf(w, n, "mass %.1f t over the %.1f t limit, waiting %.0f s between decisions", in.mass_kg / 1000, lim / 1000, c.act_gap_s); continue; }
                out->want[a] = 0; out->term[a] = AXT_OVER_LIMIT;
                snprintf(w, n, "mass %.1f t over the %.1f t limit of %d axles%s", in.mass_kg / 1000, lim / 1000, down, s->cap_kg[a] > 0 && lim == s->cap_kg[a] ? " (learned from the game)" : "");
            } else if (slip_bad && in.kmh < c.min_kmh) {
                if (t - s->act_at[a] < c.act_gap_s) { out->term[a] = AXT_SLIP_LOW_WAIT; snprintf(w, n, "wheel slip at %.0f km/h (under %.0f), waiting %.0f s between decisions", in.kmh, c.min_kmh, c.act_gap_s); continue; }
                out->want[a] = 0; out->term[a] = AXT_SLIP_LOW;
                snprintf(w, n, "wheel slip over %.0f %% within %.0f s at %.0f km/h (under %.0f)", c.slip_max * 100, c.slip_hold_s, in.kmh, c.min_kmh);
            } else { out->term[a] = AXT_UP_OK; snprintf(w, n, "up: mass %.1f t within the %.1f t limit", in.mass_kg / 1000, lim / 1000); }
            continue;
        }
        // down: all of it has to hold
        if (t < s->ext_until[a]) { out->term[a] = AXT_FOREIGN_HOLD; snprintf(w, n, "the control was changed by someone else %.0f s ago (hands off for %.0f s)", c.ext_hold_s - (float)(s->ext_until[a] - t), c.ext_hold_s); continue; }
        if (!(in.mass_kg <= c.raise_frac * lim)) { out->term[a] = AXT_MASS_HIGH; snprintf(w, n, "mass %.1f t over %.0f %% of the %.1f t limit of %d axles (%.1f t)", in.mass_kg / 1000, c.raise_frac * 100, lim / 1000, down, c.raise_frac * lim / 1000); continue; }
        if (s->steady_since < -1e8 || t - s->steady_since < c.steady_s) { out->term[a] = AXT_NOT_STEADY; snprintf(w, n, "speed or grade not steady (need over %.0f km/h, under %.1f m/s2 and %.0f %% grade, for %.0f s)", c.min_kmh, c.accel_max, c.grade_max_pct, c.steady_s); continue; }
        if (s->mass_since < -1e8 || t - s->mass_since < c.mass_settle_s) { out->term[a] = AXT_MASS_CHANGING; snprintf(w, n, "mass still changing (a step over %.0f %% less than %.0f s ago)", c.mass_step_frac * 100, c.mass_settle_s); continue; }
        if (!slip_known) { out->term[a] = AXT_SLIP_UNKNOWN; snprintf(w, n, "wheel slip unknown"); continue; }
        if (slip_bad) { out->term[a] = AXT_SLIP; snprintf(w, n, "wheel slip over %.0f %% within the last %.0f s", c.slip_max * 100, c.slip_hold_s); continue; }
        if (t - s->act_at[a] < c.act_gap_s) { out->term[a] = AXT_RAISE_WAIT; snprintf(w, n, "waiting %.0f s between decisions", c.act_gap_s); continue; }
        out->want[a] = 1; out->term[a] = AXT_RAISE;
        snprintf(w, n, "mass %.1f t <= %.0f %% of the %.1f t limit of %d axles, %.0f km/h steady for %.0f s", in.mass_kg / 1000, c.raise_frac * 100,
                 lim / 1000, down, in.kmh, (float)(t - s->steady_since));
    }
}

// The module acted (or logged that it would) on a wish: starts the gap between decisions.
static inline void axle_acted(axle_state *s, int a, double t) { s->act_at[a] = t; }
